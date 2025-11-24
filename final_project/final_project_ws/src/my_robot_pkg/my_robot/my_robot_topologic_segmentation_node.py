#!/usr/bin/env python3
"""
Topologic segmentation node for CoppeliaSim/ROS:
- Repairs thin wall gaps so rooms stay watertight.
- Computes a distance-based potential field, extracts peaks, and runs watershed/Voronoi segmentation.
- Publishes `/topologic_map` plus optional debug overlays for semantic modules downstream.
- Tunable knobs via ROS params:
    * `min_room_area_m2`: higher values merge closets into neighbors; lower values keep tiny rooms separate.
    * `wall_fix_kernel_size`: higher values fill wider gaps (risk of overfilling); lower values preserve narrow doors.
    * `potential_blur_sigma`: higher values smooth peaks for fewer, larger rooms; lower values sharpen peaks for more rooms.
    * `min_room_distance_px`: higher values enforce spaced seeds for coarser segmentation; lower values allow dense seeds for finer segmentation.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid, MapMetaData
import numpy as np
import cv2
import threading
from skimage.feature import peak_local_max
import sys
import os
import yaml
import argparse
from geometry_msgs.msg import Pose, Point, Quaternion

class TopologicSegmentationNode(Node):
    """
    Watershed segmentation + wall repair + fusion of small rooms.
    Supports running via ROS (topics) or offline via static files.
    """

    def __init__(self, offline_config=None):
        super().__init__('topologic_segmentation_node')

        # --- CONFIGURATION (Hybrid: ROS parameters or manual config) ---
        self.offline_mode = offline_config is not None
        
        if self.offline_mode:
            # Terminal mode: use the supplied dictionary
            self.p_min_area = offline_config.get('min_room_area_m2', 3.0)
            self.p_kernel_size = offline_config.get('wall_fix_kernel_size', 3)
            self.p_blur_sigma = offline_config.get('potential_blur_sigma', 2.0)
            self.p_min_dist = offline_config.get('min_room_distance_px', 15)
            self.get_logger().info("Starting in OFFLINE MODE (no ROS topics).")
        else:
            # ROS mode: declare and read parameters from the server
            self.declare_parameter('min_room_area_m2', 3.0)
            self.declare_parameter('wall_fix_kernel_size', 3)
            self.declare_parameter('potential_blur_sigma', 2.0)
            self.declare_parameter('min_room_distance_px', 15)
            
            self.p_min_area = self.get_parameter('min_room_area_m2').value
            self.p_kernel_size = self.get_parameter('wall_fix_kernel_size').value
            self.p_blur_sigma = self.get_parameter('potential_blur_sigma').value
            self.p_min_dist = self.get_parameter('min_room_distance_px').value

            # QoS and subscriptions only when running online
            map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
            self.map_sub = self.create_subscription(OccupancyGrid, '/map', self._map_callback, map_qos)
            self.room_pub = self.create_publisher(OccupancyGrid, '/topologic_map', map_qos)
            self.debug_walls_pub = self.create_publisher(OccupancyGrid, '/debug/repaired_walls', map_qos)
            self.timer = self.create_timer(2.0, self._processing_loop)
            self.get_logger().info('Topologic Segmentation Node Ready.')

        self._current_map = None
        self._lock = threading.Lock()

    def _map_callback(self, msg: OccupancyGrid):
        with self._lock:
            self._current_map = msg

    def _processing_loop(self):
        if self._current_map is None: return
        with self._lock: map_msg = self._current_map
        
        try:
            segmented = self.process_segmentation(map_msg)
            if segmented and not self.offline_mode: 
                self.room_pub.publish(segmented)
        except Exception as e:
            self.get_logger().error(f'Processing Error: {e}')

    def _repair_walls(self, grid, height, width):
        """Create solid walls to avoid leaks."""
        # Aggressive binarization
        binary_walls = np.zeros((height, width), dtype=np.uint8)
        binary_walls[grid != 0] = 255 

        # Morphological closing
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (self.p_kernel_size, self.p_kernel_size))
        walls_fixed = cv2.morphologyEx(binary_walls, cv2.MORPH_CLOSE, kernel)
        
        return cv2.bitwise_not(walls_fixed)

    def _merge_small_rooms(self, markers, min_pixels):
        """Iterate over all detected rooms and merge the small ones."""
        unique, counts = np.unique(markers, return_counts=True)
        area_map = dict(zip(unique, counts))
        kernel = np.ones((3,3), np.uint8)
        merged_count = 0
        sorted_ids = sorted(unique, key=lambda x: area_map[x])

        for uid in sorted_ids:
            if uid <= 0: continue 
            area = area_map[uid]
            if area >= min_pixels: continue

            mask = (markers == uid).astype(np.uint8)
            dilated = cv2.dilate(mask, kernel, iterations=1)
            border_mask = dilated - mask
            neighbors = markers[border_mask > 0]
            valid_neighbors = neighbors[(neighbors > 0) & (neighbors != uid)]
            
            if len(valid_neighbors) > 0:
                counts = np.bincount(valid_neighbors)
                dominant_neighbor = np.argmax(counts)
                markers[mask == 1] = dominant_neighbor
                merged_count += 1
            else:
                markers[mask == 1] = 0  # Isolated island

        if merged_count > 0:
            self.get_logger().debug(f'Merge: {merged_count} small areas attached to bigger neighbors.')
        return markers

    # Renamed from _process_segmentation to process_segmentation (public method)
    def process_segmentation(self, map_msg: OccupancyGrid) -> OccupancyGrid:
        h, w = map_msg.info.height, map_msg.info.width
        res = map_msg.info.resolution
        
        # Use the parameters stored on self
        grid = np.array(map_msg.data, dtype=np.int8).reshape((h, w))

        # Step 1: Repair the free-space mask so walls are watertight
        free_space_mask = self._repair_walls(grid, h, w)
        if not self.offline_mode:
             self._publish_debug(self.debug_walls_pub, (free_space_mask == 0).astype(np.uint8)*100, map_msg)

        # Step 2: Build a smoothed potential field to seed room centers
        dist_map = cv2.distanceTransform(free_space_mask, cv2.DIST_L2, 5)
        
        # Step 3: Blur the distance transform so seed peaks form in the center of wide spaces (avoid false peaks due to furniture)
        potential_field = cv2.GaussianBlur(
            dist_map, (0, 0), sigmaX=self.p_blur_sigma, sigmaY=self.p_blur_sigma
        )

        # Step 4: Extract local maxima that will become watershed seeds
        local_maxima = peak_local_max(
            potential_field, 
            min_distance=int(self.p_min_dist),
            labels=free_space_mask,
            exclude_border=False
        )

        markers = np.zeros_like(grid, dtype=np.int32)
        room_count = 0
        for (y, x) in local_maxima:
            if potential_field[y, x] > 0.5:
                room_count += 1
                markers[y, x] = room_count

        if room_count == 0:
            self.get_logger().warn("No rooms detected.")
            return None

        # Step 5: Flood the potential field via watershed to assign room IDs
        base_img = cv2.cvtColor(free_space_mask, cv2.COLOR_GRAY2BGR)
        markers_final = markers.copy()
        markers_final[free_space_mask == 0] = -1
        cv2.watershed(base_img, markers_final)

        # Step 6: Merge or drop tiny regions so only meaningful rooms remain
        min_pixels = int(self.p_min_area / (res * res))
        markers_final = self._merge_small_rooms(markers_final, min_pixels)

        # Step 7: Emit an OccupancyGrid that encodes both rooms and obstacles
        final_grid = np.full_like(grid, -1, dtype=np.int8)
        room_area = (markers_final > 0) & (free_space_mask == 255)
        
        if np.any(room_area):
            ids = markers_final[room_area]
            # Simple hash-style mapping for consistent colors in ROS
            vals = ((ids * 37) % 85) + 10 
            final_grid[room_area] = vals.astype(np.int8)

        # Original obstacles
        final_grid[grid == 100] = 100
        final_grid[(grid == 0) & (final_grid == -1)] = 0

        out_msg = OccupancyGrid()
        out_msg.header = map_msg.header
        out_msg.info = map_msg.info
        out_msg.data = final_grid.flatten().tolist()
        return out_msg

    def _publish_debug(self, pub, data, ref):
        if pub.get_subscription_count() > 0:
            msg = OccupancyGrid()
            msg.header = ref.header
            msg.info = ref.info
            msg.data = data.flatten().astype(np.int8).tolist()
            pub.publish(msg)

# --- OFFLINE UTILITY FUNCTIONS ---

def load_map_from_yaml(yaml_path):
    """Load the .yaml/.pgm pair and convert to an OccupancyGrid."""
    with open(yaml_path, 'r') as f:
        map_data = yaml.safe_load(f)

    # Resolve the image path relative to the YAML description
    image_path = map_data['image']
    if not os.path.isabs(image_path):
        image_path = os.path.join(os.path.dirname(yaml_path), image_path)

    if not os.path.exists(image_path):
        raise FileNotFoundError(f"Map image not found: {image_path}")

    # Load the grayscale map
    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise ValueError("Failed to open map image with OpenCV.")

    # PGM loaded via OpenCV: 255 = free (white), 0 = occupied (black), ~205 = unknown
    # ROS OccupancyGrid: 0 = free, 100 = occupied, -1 = unknown
    
    # If the mode is 'trinary', convert accordingly
    mode = map_data.get('mode', 'trinary')
    thresh_occ = map_data.get('occupied_thresh', 0.65)
    thresh_free = map_data.get('free_thresh', 0.196)
    negate = map_data.get('negate', 0)

    # Normalize to 0-1
    img_norm = img.astype(float) / 255.0
    if negate:
        img_norm = 1.0 - img_norm

    grid_data = np.full(img.shape, -1, dtype=np.int8)
    
    # Same logic as map_server
    grid_data[img_norm >= thresh_occ] = 100
    grid_data[img_norm <= thresh_free] = 0
    # O resto fica -1

    # Flip Y (OpenCV images start at top-left; ROS maps usually start bottom-left)
    # NOTE: map_server often loads this correctly, but depending on how the pgm was saved
    # a flip might still be needed. ROS maps are vertically flipped relative to regular images.
    grid_data = np.flipud(grid_data)

    # Assemble the OccupancyGrid
    msg = OccupancyGrid()
    msg.info.resolution = map_data['resolution']
    msg.info.width = img.shape[1]
    msg.info.height = img.shape[0]
    
    origin = map_data['origin']  # [x, y, yaw]
    msg.info.origin = Pose(position=Point(x=origin[0], y=origin[1], z=0.0))
    # (Quaternion math skipped for simplicity since only the grid is used)
    
    msg.data = grid_data.flatten().tolist()
    return msg

def save_colored_map(occupancy_msg, output_path):
    """Convert a segmented OccupancyGrid into a colored JPG."""
    w = occupancy_msg.info.width
    h = occupancy_msg.info.height
    data = np.array(occupancy_msg.data, dtype=np.int8).reshape((h, w))
    
    # Un-flip vertically so the saved visualization matches the ROS map
    data = np.flipud(data)

    # Create a color (BGR) image
    # Background (unknown) = gray
    # Obstacles = black
    # Rooms = pseudo colors
    
    # Normalize room IDs to 0-255 before applying the colormap
    # IDs in the OccupancyGrid stay between ~10 and ~100 per node logic, or -1/100
    
    visual_img = np.zeros((h, w, 3), dtype=np.uint8)
    
    # Masks
    mask_unknown = (data == -1)
    mask_obstacle = (data == 100)
    mask_rooms = (data >= 0) & (data != 100)

    # Paint unknown cells light gray
    visual_img[mask_unknown] = [200, 200, 200]
    
    # Paint obstacles black
    visual_img[mask_obstacle] = [0, 0, 0]

    # Color the rooms
    if np.any(mask_rooms):
        room_vals = data[mask_rooms]
        
        # Expand the range to leverage the entire 0-255 color space
        # The node generates values between ~10 and ~95, so stretch those IDs.
        normalized = ((room_vals.astype(float) * 5) % 255).astype(np.uint8)
        
        colored_layer = np.zeros((h, w), dtype=np.uint8)
        colored_layer[mask_rooms] = normalized
        
        # Apply the heatmap
        heatmap = cv2.applyColorMap(colored_layer, cv2.COLORMAP_JET)
        
        # Copy only the room pixels into the final visualization
        visual_img[mask_rooms] = heatmap[mask_rooms]

    cv2.imwrite(output_path, visual_img)
    print(f"Result saved to: {output_path}")

def main(args=None):
    # Handle command-line arguments
    parser = argparse.ArgumentParser(description="Watershed segmentation for ROS maps.")
    parser.add_argument('--map', type=str, help="Path to the map .yaml file.")
    parser.add_argument('--output', type=str, default="segmented_map.jpg", help="Path used to save the resulting JPG.")
    
    # Allow argparse and rclpy to coexist by filtering known args and passing the rest through
    known_args, unknown_args = parser.parse_known_args()

    if known_args.map:
        # --- OFFLINE MODE (TERMINAL) ---
        print(f"Loading map: {known_args.map}")
        
        try:
            # Step 1: Load the YAML map and convert it to OccupancyGrid
            map_msg = load_map_from_yaml(known_args.map)
            
            # Step 2: Initialize ROS (minimally, to allow creating the Node)
            rclpy.init(args=unknown_args)
            
            # Manual configuration (tweak as needed)
            config = {
                'min_room_area_m2': 3.0,   # drop rooms smaller than ~3 m²
                'wall_fix_kernel_size': 3, # convolution kernel used to close wall gaps
                'potential_blur_sigma': 2.0, # Gaussian blur applied to the distance transform
                'min_room_distance_px': 15  # minimum pixel spacing between seed peaks
            }
            
            node = TopologicSegmentationNode(offline_config=config)
            
            # Step 3: Run the segmentation pipeline
            print("Running segmentation...")
            segmented_msg = node.process_segmentation(map_msg)
            
            if segmented_msg:
                # Step 4: Save the colored output for inspection
                save_colored_map(segmented_msg, known_args.output)
            else:
                print("Error: segmentation did not return any result.")
            
            node.destroy_node()
            rclpy.shutdown()
            
        except Exception as e:
            print(f"Critical error: {e}")
            sys.exit(1)

    else:
        # --- ONLINE MODE (ROS NODE) ---
        rclpy.init(args=args)
        node = TopologicSegmentationNode()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
