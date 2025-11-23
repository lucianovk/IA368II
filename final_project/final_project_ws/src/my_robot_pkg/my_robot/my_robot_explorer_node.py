import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist, Point, PoseStamped, Pose
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
import numpy as np
import math
import heapq
from scipy.ndimage import distance_transform_edt, binary_dilation

# --- Robot configuration ---
ROBOT_RADIUS = 0.16
INFLATION_RADIUS = 0.25 
COVERAGE_RADIUS = 0.30  # Radius the robot "paints" as visited while moving

# Speed limits
MAX_LINEAR_SPEED = 0.5 
MAX_ANGULAR_SPEED = 0.8
MAX_ACCEL = 0.2         
MAX_ANGULAR_ACCEL = 0.5 
KP_ANGULAR = 1.8        

# --- Navigation configuration ---
ALIGNMENT_THRESHOLD = 0.20  # (rad) ~11 degrees.

# --- Sensor configuration and hysteresis ---
SCAN_FOV_DEG = 60          
COLLISION_DIST_NORMAL = 0.35  
COLLISION_DIST_RESCUE = 0.15  
SCAN_CLEAR_DIST = 0.75      
SCAN_SAFE_BUBBLE = 0.36     

# --- Cost configuration ---
WALL_PENALTY_FACTOR = 10.0 
SAFE_DIST_METERS = 0.6     

class AStarNode:
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position 
        self.g = 0.0
        self.h = 0.0
        self.f = 0.0

    def __eq__(self, other):
        return self.position == other.position
    
    def __lt__(self, other):
        return self.f < other.f

class MyRobotCoverage(Node):
    def __init__(self):
        super().__init__('my_robot_coverage_node')

        # QoS
        qos_sensor = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        qos_map = QoSProfile(durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1)

        # Subscribers
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_sensor)
        self.sub_map = self.create_subscription(OccupancyGrid, '/map', self.map_callback, qos_map)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Publishers
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_planned_path = self.create_publisher(Path, '/planned_path', 10)
        self.pub_followed_path = self.create_publisher(Path, '/followed_path', 10)
        self.pub_inflated = self.create_publisher(OccupancyGrid, '/inflated_obstacle', qos_map)
        
        # Publisher for the visited map visualization
        self.pub_visited = self.create_publisher(OccupancyGrid, '/visited_map', qos_map)

        # Internal state
        self.map_data = None
        self.inflated_map_data = None
        
        # Visited grid storage
        self.visited_grid = None 
        
        self.cost_map = None
        self.map_info = None
        self.robot_pose = None 
        self.current_path = [] 
        self.followed_path_msg = Path()
        self.followed_path_msg.header.frame_id = 'map'
        
        self.state = "IDLE" 
        self.scan_ranges = []
        self.scan_angle_min = 0.0
        self.scan_angle_inc = 0.0
        
        self.avoid_angle = 0.0
        self.min_front_dist = float('inf') 
        self.min_360_dist = float('inf')

        self.is_rescuing = False

        # Smooth control
        self.last_linear_vel = 0.0
        self.last_angular_vel = 0.0
        self.dt = 0.05 

        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info("Coverage Node (Fill the Map) started")

    def euler_from_quaternion(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def odom_callback(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = self.euler_from_quaternion(q)
        self.robot_pose = (p.x, p.y, yaw)

        # Update the visited map with the current position
        self.update_visited_map()

        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.header.frame_id = 'map'
        pose_stamped.pose = msg.pose.pose
        
        if len(self.followed_path_msg.poses) > 5000:
            self.followed_path_msg.poses.pop(0)
        
        if not self.followed_path_msg.poses or \
           math.dist((p.x, p.y), (self.followed_path_msg.poses[-1].pose.position.x, self.followed_path_msg.poses[-1].pose.position.y)) > 0.05:
            self.followed_path_msg.poses.append(pose_stamped)
            self.pub_followed_path.publish(self.followed_path_msg)

    def map_callback(self, msg):
        self.map_info = msg.info
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        
        # 0 = free, 100 = occupied, -1 = unknown
        grid = np.array(msg.data, dtype=np.int8).reshape((height, width))
        
        # Initialize or resize the visited map if needed.
        # If the map grows (slam_toolbox can do this) we reset or expand the visited grid.
        # For simplicity, recreate it when the size changes.
        if self.visited_grid is None or self.visited_grid.shape != grid.shape:
            self.visited_grid = np.zeros_like(grid, dtype=np.int8)
            self.get_logger().info(f"Visited map initialized: {width}x{height}")

        cells_radius = int(math.ceil(INFLATION_RADIUS / resolution))
        obstacles = (grid > 0)
        
        y, x = np.ogrid[-cells_radius:cells_radius+1, -cells_radius:cells_radius+1]
        kernel = x**2 + y**2 <= cells_radius**2
        
        inflated = binary_dilation(obstacles, structure=kernel)
        
        self.inflated_map_data = np.copy(grid)
        self.inflated_map_data[inflated] = 100 
        
        dist_grid = distance_transform_edt(grid == 0) * resolution 
        
        self.cost_map = np.zeros_like(dist_grid)
        mask_close = dist_grid < SAFE_DIST_METERS
        self.cost_map[mask_close] = WALL_PENALTY_FACTOR * (SAFE_DIST_METERS - dist_grid[mask_close])
        
        inf_msg = OccupancyGrid()
        inf_msg.header = msg.header
        inf_msg.info = msg.info
        inf_msg.data = self.inflated_map_data.flatten().tolist()
        self.pub_inflated.publish(inf_msg)
        
        self.map_data = grid
        
        if self.state == "IDLE" and self.robot_pose is not None:
            self.state = "PLANNING"

    def update_visited_map(self):
        """Mark the robot's current neighborhood as visited (value 100)."""
        if self.map_info is None or self.visited_grid is None or self.robot_pose is None:
            return

        rx, ry, _ = self.robot_pose
        grid_pos = self.world_to_grid(rx, ry)
        if not grid_pos: return

        r, c = grid_pos
        res = self.map_info.resolution
        radius_cells = int(math.ceil(COVERAGE_RADIUS / res))

        # Create a simple circular mask
        rows, cols = self.visited_grid.shape
        y, x = np.ogrid[-radius_cells:radius_cells+1, -radius_cells:radius_cells+1]
        mask = x**2 + y**2 <= radius_cells**2

        # Define os limites para o slice
        r_start = max(0, r - radius_cells)
        r_end = min(rows, r + radius_cells + 1)
        c_start = max(0, c - radius_cells)
        c_end = min(cols, c + radius_cells + 1)

        # Adjust the mask when near the map boundary
        mask_h = r_end - r_start
        mask_w = c_end - c_start
        
        # Iterate over the bounding box and apply the mask
        for i in range(r_start, r_end):
            for j in range(c_start, c_end):
                if (i - r)**2 + (j - c)**2 <= radius_cells**2:
                    self.visited_grid[i, j] = 100  # Mark as visited

        # Publica o mapa de visitados
        visited_msg = OccupancyGrid()
        visited_msg.header.frame_id = "map"
        visited_msg.header.stamp = self.get_clock().now().to_msg()
        visited_msg.info = self.map_info
        visited_msg.data = self.visited_grid.flatten().tolist()
        self.pub_visited.publish(visited_msg)

    def get_best_escape_angle(self, ranges, angle_min, angle_inc):
        clean_ranges = np.array(ranges)
        clean_ranges[clean_ranges == float('inf')] = 10.0
        clean_ranges = np.nan_to_num(clean_ranges, nan=0.0)

        window_size = 10
        if len(clean_ranges) < window_size: return 0.0
        
        kernel = np.ones(window_size) / window_size
        smoothed = np.convolve(clean_ranges, kernel, mode='same')
        
        best_idx = np.argmax(smoothed)
        best_angle = angle_min + (best_idx * angle_inc)
        
        return math.atan2(math.sin(best_angle), math.cos(best_angle))

    def scan_callback(self, msg):
        self.scan_ranges = msg.ranges
        self.scan_angle_min = msg.angle_min
        self.scan_angle_inc = msg.angle_increment

        if self.robot_pose is None: return

        all_ranges = np.array(msg.ranges)
        all_ranges[all_ranges == float('inf')] = 10.0
        all_ranges = np.nan_to_num(all_ranges, nan=10.0)
        self.min_360_dist = np.min(all_ranges)

        fov_rad = math.radians(SCAN_FOV_DEG / 2)
        min_idx = int(( -fov_rad - self.scan_angle_min ) / self.scan_angle_inc)
        max_idx = int(( fov_rad - self.scan_angle_min ) / self.scan_angle_inc)
        
        ranges_len = len(msg.ranges)
        self.min_front_dist = float('inf') 

        for i in range(min_idx, max_idx + 1):
            idx = i % ranges_len
            d = msg.ranges[idx]
            if 0.01 < d < self.min_front_dist:
                self.min_front_dist = d
        
        current_collision_threshold = COLLISION_DIST_NORMAL
        if self.is_rescuing:
            current_collision_threshold = COLLISION_DIST_RESCUE

        if self.state == "MOVING" and self.min_front_dist < current_collision_threshold:
            self.get_logger().warn(f"Collision ({self.min_front_dist:.2f} m). Stopping and starting escape mode.")
            self.stop_robot()
            self.state = "AVOIDING"
            self.avoid_angle = self.get_best_escape_angle(msg.ranges, msg.angle_min, msg.angle_increment)

        elif self.state == "AVOIDING":
            self.avoid_angle = self.get_best_escape_angle(msg.ranges, msg.angle_min, msg.angle_increment)
            is_front_clear = self.min_front_dist > SCAN_CLEAR_DIST
            is_aligned = abs(self.avoid_angle) < ALIGNMENT_THRESHOLD 
            
            safe_bubble_threshold = SCAN_SAFE_BUBBLE
            if self.is_rescuing:
                 safe_bubble_threshold = COLLISION_DIST_NORMAL 

            is_safe_bubble = self.min_360_dist > safe_bubble_threshold
            
            if is_front_clear and is_aligned and is_safe_bubble:
                self.get_logger().info(f"Livre e Alinhado. Replanejando.")
                self.stop_robot()
                self.state = "PLANNING"

    def world_to_grid(self, wx, wy):
        if self.map_info is None: return None
        mx = int((wx - self.map_info.origin.position.x) / self.map_info.resolution)
        my = int((wy - self.map_info.origin.position.y) / self.map_info.resolution)
        if 0 <= mx < self.map_info.width and 0 <= my < self.map_info.height:
            return (my, mx) 
        return None

    def grid_to_world(self, r, c):
        if self.map_info is None: return None
        wx = (c * self.map_info.resolution) + self.map_info.origin.position.x + (self.map_info.resolution/2)
        wy = (r * self.map_info.resolution) + self.map_info.origin.position.y + (self.map_info.resolution/2)
        return (wx, wy)

    def find_unvisited_targets(self):
        """
        Replacement for find_frontiers.
        Finds cells that are FREE in the map (0) but NOT VISITED (0) in visited_grid.
        Only returns cells that are also safe (0 in inflated_map).
        """
        if self.inflated_map_data is None or self.visited_grid is None: return []
        
        # Criteria: free in the physical map AND not visited AND safe (not inflated)
        # map_data == 0 -> free
        # visited_grid == 0 -> not yet visited
        # inflated_map_data == 0 -> far from walls
        candidates_mask = (self.map_data == 0) & (self.visited_grid == 0) & (self.inflated_map_data == 0)
        
        candidate_indices = np.argwhere(candidates_mask)
        
        if len(candidate_indices) == 0: return []

        candidates = []
        # Aggressive downsampling for performance since many cells will be unvisited
        step = max(1, len(candidate_indices) // 50) 
        
        for i in range(0, len(candidate_indices), step):
            r, c = candidate_indices[i]
            wx, wy = self.grid_to_world(r, c)
            candidates.append((wx, wy))
            
        return candidates

    def find_nearest_safe_cell(self, start_grid, max_radius_cells=30):
        rows, cols = self.inflated_map_data.shape
        queue = [start_grid]
        visited = set([start_grid])
        
        while queue:
            curr = queue.pop(0)
            r, c = curr
            
            if self.inflated_map_data[r, c] == 0:
                return (r, c)
            
            if abs(r - start_grid[0]) > max_radius_cells or abs(c - start_grid[1]) > max_radius_cells:
                continue

            neighbors = [(0,1),(0,-1),(1,0),(-1,0)]
            for dr, dc in neighbors:
                nr, nc = r + dr, c + dc
                if 0 <= nr < rows and 0 <= nc < cols and (nr, nc) not in visited:
                    visited.add((nr, nc))
                    queue.append((nr, nc))
        return None

    def a_star(self, start_grid, end_grid, ignore_inflation=False):
        grid_to_use = self.map_data if ignore_inflation else self.inflated_map_data

        if self.map_data[end_grid[0], end_grid[1]] > 0:
            return None
            
        if not ignore_inflation and self.inflated_map_data[end_grid[0], end_grid[1]] > 0:
            return None

        start_node = AStarNode(None, start_grid)
        end_node = AStarNode(None, end_grid)
        
        open_list = []
        heapq.heappush(open_list, start_node)
        visited_costs = {} 
        
        max_iterations = 100000 
        iter_count = 0
        
        rows, cols = grid_to_use.shape
        neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]
        H_WEIGHT = 2.0

        while len(open_list) > 0:
            iter_count += 1
            if iter_count > max_iterations: return None

            current_node = heapq.heappop(open_list)
            pos = current_node.position
            
            if pos in visited_costs and visited_costs[pos] <= current_node.g:
                continue
            visited_costs[pos] = current_node.g
            
            if pos == end_grid:
                path = []
                curr = current_node
                while curr is not None:
                    path.append(curr.position)
                    curr = curr.parent
                return path[::-1]
            
            (r, c) = pos
            
            for next_pos in neighbors:
                nr, nc = r + next_pos[0], c + next_pos[1]
                
                if 0 <= nr < rows and 0 <= nc < cols:
                    if grid_to_use[nr, nc] > 0: continue
                    
                    base_cost = math.hypot(next_pos[0], next_pos[1])
                    penalty = 0.0
                    if self.cost_map is not None:
                        penalty = self.cost_map[nr, nc]
                    
                    new_g = current_node.g + base_cost + penalty

                    if (nr, nc) in visited_costs and visited_costs[(nr, nc)] <= new_g:
                        continue

                    new_node = AStarNode(current_node, (nr, nc))
                    new_node.g = new_g
                    new_node.h = math.hypot(nr - end_grid[0], nc - end_grid[1]) * H_WEIGHT
                    new_node.f = new_node.g + new_node.h
                    heapq.heappush(open_list, new_node)
                    
        return None

    def simplify_path(self, grid_path):
        if not grid_path or len(grid_path) < 3: return grid_path
        simplified = [grid_path[0]]
        current_idx = 0
        while current_idx < len(grid_path) - 1:
            check_idx = len(grid_path) - 1
            found = False
            while check_idx > current_idx + 1:
                if self.is_line_free(grid_path[current_idx], grid_path[check_idx]):
                    simplified.append(grid_path[check_idx])
                    current_idx = check_idx
                    found = True
                    break
                check_idx -= 1
            if not found:
                current_idx += 1
                simplified.append(grid_path[current_idx])
        return simplified

    def is_line_free(self, p1, p2):
        r0, c0 = p1
        r1, c1 = p2
        length = int(math.hypot(r1-r0, c1-c0))
        if length == 0: return True
        for i in range(length + 1):
            r = int(r0 + (r1 - r0) * i / length)
            c = int(c0 + (c1 - c0) * i / length)
            if 0 <= r < self.inflated_map_data.shape[0] and 0 <= c < self.inflated_map_data.shape[1]:
                if self.inflated_map_data[r, c] > 0: return False
        return True

    def get_farthest_reachable(self):
        if self.inflated_map_data is None: return None
        # Find a random free and safe cell (ignores visited state)
        # Used as fallback when no coverage targets are available
        free_indices = np.argwhere(self.inflated_map_data == 0)
        if len(free_indices) == 0: return None
        
        step = max(1, len(free_indices) // 100)
        max_dist = -1.0
        best_pt = None
        curr_grid = self.world_to_grid(self.robot_pose[0], self.robot_pose[1])
        
        for i in range(0, len(free_indices), step):
            pt = tuple(free_indices[i])
            dist = math.hypot(pt[0]-curr_grid[0], pt[1]-curr_grid[1])
            if dist > max_dist:
                max_dist = dist
                best_pt = self.grid_to_world(pt[0], pt[1])
        return best_pt

    def plan_route(self):
        if self.inflated_map_data is None or self.robot_pose is None or self.visited_grid is None:
            return

        start_grid = self.world_to_grid(self.robot_pose[0], self.robot_pose[1])
        in_inflation = (self.inflated_map_data[start_grid[0], start_grid[1]] > 0)
        
        path_found = None
        self.is_rescuing = False

        if in_inflation:
            self.get_logger().warn("Robot inside inflation zone. Planning a rescue route...")
            safe_grid = self.find_nearest_safe_cell(start_grid)
            
            if safe_grid:
                escape_path = self.a_star(start_grid, safe_grid, ignore_inflation=True)
                if escape_path:
                    self.is_rescuing = True 
                    # Try to reach any unvisited area after escaping
                    targets = self.find_unvisited_targets()
                    safe_world = self.grid_to_world(safe_grid[0], safe_grid[1])
                    targets.sort(key=lambda p: math.hypot(p[0]-safe_world[0], p[1]-safe_world[1]))
                    
                    for target_world in targets:
                        target_grid = self.world_to_grid(target_world[0], target_world[1])
                        if target_grid and self.inflated_map_data[target_grid[0], target_grid[1]] == 0:
                            main_path = self.a_star(safe_grid, target_grid, ignore_inflation=False)
                            if main_path:
                                full_path = escape_path + main_path[1:]
                                path_found = self.simplify_path(full_path)
                                break
        else:
            # Coverage strategy: search for the closest NOT VISITED target
            targets = self.find_unvisited_targets()
            
            if not targets:
                self.get_logger().info("No unvisited targets found. Coverage complete!")
            else:
                targets.sort(key=lambda p: math.hypot(p[0]-self.robot_pose[0], p[1]-self.robot_pose[1]))
                
                # Try the 5 nearest targets to avoid A* failures
                for target_world in targets[:5]:
                    target_grid = self.world_to_grid(target_world[0], target_world[1])
                    if target_grid:
                        raw_path = self.a_star(start_grid, target_grid, ignore_inflation=False)
                        if raw_path:
                            path_found = self.simplify_path(raw_path)
                            break
        
        # Fallback: if no route is found or coverage ended, move toward the farthest free point
        # to ensure we are not stuck (or finish if nothing else works).
        if not path_found and not in_inflation and targets:
             self.get_logger().warn("A* failed for nearby targets. Trying a random unvisited target.")
             import random
             random_target = random.choice(targets)
             target_grid = self.world_to_grid(random_target[0], random_target[1])
             raw_path = self.a_star(start_grid, target_grid, ignore_inflation=False)
             if raw_path:
                path_found = self.simplify_path(raw_path)

        if path_found:
            self.current_path = [self.grid_to_world(p[0], p[1]) for p in path_found]
            
            path_msg = Path()
            path_msg.header.frame_id = "map"
            path_msg.header.stamp = self.get_clock().now().to_msg()
            for wp in self.current_path:
                ps = PoseStamped()
                ps.pose.position.x = wp[0]
                ps.pose.position.y = wp[1]
                path_msg.poses.append(ps)
            self.pub_planned_path.publish(path_msg)
            self.state = "MOVING"
        else:
            if in_inflation:
                self.get_logger().error("Rescue maneuver failed. Robot is immobile.")
            elif not targets:
                self.get_logger().info("Coverage completed (100% visited or inaccessible).")
            else:
                self.get_logger().warn("Falha no planejamento. Tentando novamente...")
            
            self.stop_robot()
            self.state = "IDLE"

    def stop_robot(self):
        twist = Twist()
        self.pub_cmd.publish(twist)

    def control_loop(self):
        target_v = 0.0
        target_w = 0.0

        if self.state == "MOVING" and self.is_rescuing:
            if self.robot_pose:
                 curr_grid = self.world_to_grid(self.robot_pose[0], self.robot_pose[1])
                 if curr_grid and self.inflated_map_data[curr_grid[0], curr_grid[1]] == 0:
                     self.is_rescuing = False

        if self.state == "PLANNING":
            target_v = 0.0
            target_w = 0.0
            self.plan_route()
        
        elif self.state == "AVOIDING":
            yaw_err = self.avoid_angle
            if abs(yaw_err) > ALIGNMENT_THRESHOLD: 
                target_v = 0.0
                target_w = np.clip(yaw_err * KP_ANGULAR, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED)
            else:
                speed_factor = (1.0 - (abs(yaw_err) / ALIGNMENT_THRESHOLD)) 
                target_v = MAX_LINEAR_SPEED * 0.5 * max(0.0, speed_factor)
                target_w = np.clip(yaw_err * KP_ANGULAR, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED)

        elif self.state == "MOVING":
            if not self.current_path:
                self.state = "PLANNING"
                target_v = 0.0
                target_w = 0.0
            else:
                target_x, target_y = self.current_path[0]
                dx = target_x - self.robot_pose[0]
                dy = target_y - self.robot_pose[1]
                distance = math.hypot(dx, dy)
                
                if distance < 0.20: 
                    self.current_path.pop(0)
                    if not self.current_path:
                        target_v = 0.0
                        target_w = 0.0
                    else:
                        target_x, target_y = self.current_path[0]
                        dx = target_x - self.robot_pose[0]
                        dy = target_y - self.robot_pose[1]
                        distance = math.hypot(dx, dy)

                if self.current_path: 
                    target_yaw = math.atan2(dy, dx)
                    yaw_err = target_yaw - self.robot_pose[2]
                    yaw_err = math.atan2(math.sin(yaw_err), math.cos(yaw_err))
                    
                    if abs(yaw_err) > ALIGNMENT_THRESHOLD:
                        target_v = 0.0
                        target_w = np.clip(yaw_err * KP_ANGULAR, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED)
                    else:
                        target_v = np.clip(distance, 0.0, MAX_LINEAR_SPEED)
                        target_w = np.clip(yaw_err * KP_ANGULAR, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED)

        lin_diff = target_v - self.last_linear_vel
        ang_diff = target_w - self.last_angular_vel
        
        max_lin_change = MAX_ACCEL * self.dt
        max_ang_change = MAX_ANGULAR_ACCEL * self.dt
        
        actual_v = self.last_linear_vel + np.clip(lin_diff, -max_lin_change, max_lin_change)
        actual_w = self.last_angular_vel + np.clip(ang_diff, -max_ang_change, max_ang_change)
        
        if abs(actual_v) < 0.01: actual_v = 0.0
        if abs(actual_w) < 0.01: actual_w = 0.0

        twist = Twist()
        twist.linear.x = float(actual_v)
        twist.angular.z = float(actual_w)
        self.pub_cmd.publish(twist)
        
        self.last_linear_vel = actual_v
        self.last_angular_vel = actual_w

def main(args=None):
    rclpy.init(args=args)
    node = MyRobotCoverage()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
