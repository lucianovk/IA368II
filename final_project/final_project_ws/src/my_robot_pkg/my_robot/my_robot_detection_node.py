#!/usr/bin/env python3
import json
import math
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple, Set

import numpy as np
import rclpy
from rclpy.exceptions import ParameterAlreadyDeclaredException
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener, TransformException
from geometry_msgs.msg import PointStamped
from tf2_geometry_msgs import do_transform_point

try:
    import cv2  # type: ignore
except Exception:  # pragma: no cover
    cv2 = None

try:
    from ultralytics import YOLO  # type: ignore
except Exception:
    YOLO = None


class MyRobotDetectionNode(Node):
    """
    Continuously subscribes to /camera, runs YOLO, and maintains a set of unique detections.
    Each new detection is published on /detections_markers and logged to disk.
    """

    def __init__(self):
        super().__init__('my_robot_detection_node')

        default_params = [
            ('camera_topic', '/camera'),
            ('depth_topic', '/camera_depth'),
            ('base_frame', 'base_footprint'),
            ('map_topic', '/map'),
            ('fx', 585.5),  # defaults for 640x480 and ~57° FOV
            ('fy', 585.5),
            ('cx', 319.5),
            ('cy', 239.5),
            ('fov_deg', 57.0),
            ('use_sim_time', False),
            ('detection_tolerance', 1.0),  # meters
            ('confidence_threshold', 0.7),
            ('model_path', ''),  # optional override
        ]
        for param_name, default_value in default_params:
            try:
                self.declare_parameter(param_name, default_value)
            except ParameterAlreadyDeclaredException:
                continue

        self.camera_topic = self.get_parameter('camera_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.base_frame = self.get_parameter('base_frame').value
        self.map_topic = self.get_parameter('map_topic').value
        self.fx = float(self.get_parameter('fx').value)
        self.fy = float(self.get_parameter('fy').value)
        self.cx = float(self.get_parameter('cx').value)
        self.cy = float(self.get_parameter('cy').value)
        self.fov_deg = float(self.get_parameter('fov_deg').value)
        self.detection_tolerance = float(self.get_parameter('detection_tolerance').value)
        self.confidence_threshold = float(self.get_parameter('confidence_threshold').value)
        self.intrinsics_computed = False
        self.last_image: Optional[Image] = None
        self.last_depth: Optional[Tuple[Image, np.ndarray]] = None
        self.map_msg: Optional[OccupancyGrid] = None
        self.map_np: Optional[np.ndarray] = None
        self.det_counter = 0
        self.processing_detection = False
        self.no_detection_logged = False
        self.map_log_interval = Duration(seconds=10.0)
        self.last_map_log_time: Optional[Time] = None
        self.warned_messages: Set[str] = set()

        self.sub = self.create_subscription(
            Image, self.camera_topic, self.image_callback, 10
        )
        self.depth_sub = self.create_subscription(
            Image, self.depth_topic, self.depth_callback, 10
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid, self.map_topic, self.map_callback, 10
        )

        # TF buffer used to convert camera_link -> map
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publisher for the detection map
        self.map_pub = self.create_publisher(MarkerArray, '/detections_markers', 10)
        self.map_timer = self.create_timer(2.0, self.publish_detection_map)

        # Package root (used for models and detections directory)
        self.package_root = self.resolve_package_root()

        # YOLO model
        if YOLO is None:
            self.get_logger().error(
                'ultralytics package not available. Install it and try again.'
            )
            self.model = None
        else:
            model_path = self.resolve_model_path()
            if model_path is None:
                self.get_logger().error(
                    'Failed to locate my_robot.pt. Set parameter model_path or place it under models/.'
                )
                self.model = None
            else:
                try:
                    self.model = YOLO(str(model_path))
                    self.get_logger().info(f'my_robot model loaded: {model_path}')
                except Exception as exc:
                    self.get_logger().error(f'Failed to load my_robot: {exc}')
                    self.model = None

        # Directory used to persist detections
        self.detections_dir = self.package_root / 'detections'
        self.detections_dir.mkdir(parents=True, exist_ok=True)
        self.detections_file = self.detections_dir / 'detections.json'
        self.detection_records: List[Dict[str, Any]] = self.load_existing_detections()
        self.allowed_labels = self.load_semantic_labels()
        self.ignored_labels_file = self.detections_dir / 'ignored_detections.txt'
        self.reset_ignored_labels_log()
        if self.detection_records:
            max_id = max(x['id'] for x in self.detection_records)
            self.det_counter = max_id + 1
            self.publish_detection_map()

        self.get_logger().info('Continuous detection initialized on /camera.')

    def image_callback(self, msg: Image):
        # Compute intrinsics once based on resolution and FOV if they were not provided
        if not self.intrinsics_computed and msg.width > 0 and msg.height > 0:
            cx_calc = (msg.width - 1) * 0.5
            cy_calc = (msg.height - 1) * 0.5
            fov_rad = math.radians(self.fov_deg if self.fov_deg > 0 else 57.0)
            fx_calc = msg.width / (2.0 * math.tan(fov_rad * 0.5))
            fy_calc = fx_calc
            if self.cx <= 0:
                self.cx = cx_calc
            if self.cy <= 0:
                self.cy = cy_calc
            if self.fx <= 0:
                self.fx = fx_calc
            if self.fy <= 0:
                self.fy = fy_calc
            self.intrinsics_computed = True
            self.get_logger().info(
                f'Intrinsics defined: fx={self.fx:.2f}, fy={self.fy:.2f}, '
                f'cx={self.cx:.2f}, cy={self.cy:.2f} (res {msg.width}x{msg.height}, fov {self.fov_deg:.2f}°)'
            )
        self.last_image = msg
        if (
            self.model is None
            or self.processing_detection
            or self.last_depth is None
            or msg.encoding.lower() not in ('rgb8', 'bgr8')
        ):
            return
        self.processing_detection = True
        try:
            self.run_detection()
        finally:
            self.processing_detection = False

    def depth_callback(self, msg: Image):
        if msg.encoding.lower() not in ('32fc1', '32f'):
            self.warn_once('depth_encoding', f'Unsupported depth encoding: {msg.encoding}')
            return
        arr = np.frombuffer(msg.data, dtype=np.float32)
        expected = msg.width * msg.height
        if arr.size != expected:
            self.warn_once('depth_size', f'Unexpected depth size: {arr.size} vs {expected}')
            return
        arr = arr.reshape((msg.height, msg.width))
        self.last_depth = (msg, arr)

    def map_callback(self, msg: OccupancyGrid) -> None:
        try:
            np_data = np.array(msg.data, dtype=np.int16).reshape(
                (msg.info.height, msg.info.width)
            )
        except Exception:
            self.get_logger().warn('Map received with incompatible dimensions.')
            return
        self.map_msg = msg
        self.map_np = np_data

    def run_detection(self):
        # Make sure we have a model and at least one RGB frame to work with
        if self.model is None:
            return
        if self.last_image is None:
            return

        msg = self.last_image
        depth_tuple = self.last_depth
        # Depth is required to project detections to the map frame
        if depth_tuple is None:
            self.get_logger().warn('No synchronized depth for the current detection.')
            return
        depth_msg, depth_arr = depth_tuple

        img_np = np.frombuffer(msg.data, dtype=np.uint8)
        expected = msg.width * msg.height * 3
        if img_np.size != expected:
            self.get_logger().warn(
                f'Unexpected image size: {img_np.size} vs {expected}'
            )
            return
        img_np = img_np.reshape((msg.height, msg.width, 3))
        if msg.encoding.lower() == 'rgb8':
            img_rgb = img_np
        else:
            img_rgb = img_np[:, :, ::-1]

        try:
            # Run YOLO inference on the current frame
            results = self.model.predict(img_rgb, verbose=False)
        except Exception as exc:
            self.get_logger().error(f'YOLO inference error: {exc}')
            return

        if not results or len(results) == 0:
            self._report_no_detection('YOLO did not return any results.')
            return

        det = results[0]
        if det.boxes is None or len(det.boxes) == 0:
            self._report_no_detection('No objects detected.')
            return
        self._mark_detection_activity()

        if depth_msg.width != msg.width or depth_msg.height != msg.height:
            self.get_logger().warn(
                'Depth and RGB dimensions differ; skipping pose estimation.'
            )
            return

        # Use intrinsics; if absent assume image center
        cx = self.cx if self.cx > 0 else (msg.width - 1) * 0.5
        cy = self.cy if self.cy > 0 else (msg.height - 1) * 0.5
        if self.fx > 0 and self.fy > 0:
            fx = self.fx
            fy = self.fy
        else:
            # Estimate from FOV and resolution (assuming symmetric lens)
            fov_rad = math.radians(self.fov_deg if self.fov_deg > 0 else 57.0)
            fx = float(msg.width) / (2.0 * math.tan(fov_rad * 0.5))
            fy = fx

        try:
            tf_map_cam = self.tf_buffer.lookup_transform(
                'map',
                msg.header.frame_id if msg.header.frame_id else 'camera_link',
                rclpy.time.Time(),
            )
        except TransformException as exc:
            self.get_logger().warn(f'Could not get TF camera -> map: {exc}')
            return

        boxes = det.boxes.xyxy.cpu().numpy()
        labels_idx = det.boxes.cls.cpu().numpy().astype(int)
        confidences = det.boxes.conf.cpu().numpy() if det.boxes.conf is not None else None
        names = det.names if hasattr(det, 'names') else self.model.names

        if self.map_msg is None or self.map_np is None:
            self.warn_once('missing_map', '/map has not been received yet; detections will be ignored until then.')
            return

        map_updated = False
        # Process every bounding box returned by YOLO
        # Every detection is projected into 3D and validated before being stored
        for i, box in enumerate(boxes):
            x1, y1, x2, y2 = box
            u = int((x1 + x2) * 0.5)
            v = int((y1 + y2) * 0.5)
            # clamp
            u = max(0, min(u, depth_msg.width - 1))
            v = max(0, min(v, depth_msg.height - 1))

            # Search inside the bbox for the closest valid depth sample
            u1 = max(0, min(int(math.floor(x1)), depth_msg.width - 1))
            v1 = max(0, min(int(math.floor(y1)), depth_msg.height - 1))
            u2 = max(0, min(int(math.ceil(x2)), depth_msg.width - 1))
            v2 = max(0, min(int(math.ceil(y2)), depth_msg.height - 1))
            region = depth_arr[v1 : v2 + 1, u1 : u2 + 1]
            valid = np.isfinite(region) & (region > 0.0)
            if not np.any(valid):
                self.get_logger().info(
                    f'Detection {i}: no valid depth samples in bbox ({u1},{v1})-({u2},{v2})'
                )
                continue
            z = float(np.min(region[valid]))
            if z <= 0.0 or np.isnan(z) or np.isinf(z):
                self.get_logger().info(
                    f'Detection {i}: invalid min depth in bbox ({u1},{v1})-({u2},{v2}) -> z={z:.3f}'
                )
                continue

            # Ray direction in the camera frame (pinhole)
            nx = (u - cx) / fx
            ny = (v - cy) / fy
            dir_cam = np.array([nx, ny, 1.0], dtype=float)
            norm = np.linalg.norm(dir_cam)
            if norm <= np.finfo(float).eps:
                self.get_logger().info(
                    f'Detection {i}: degenerate ray direction at ({u},{v})'
                )
                continue
            dir_cam /= norm

            # Use z as the distance along the ray (camera-to-point) to recover 3D point
            point_cam_3d = dir_cam * z

            # Build PointStamped in the camera frame
            p_cam = PointStamped()
            p_cam.header.frame_id = msg.header.frame_id if msg.header.frame_id else 'camera_link'
            p_cam.header.stamp = msg.header.stamp
            p_cam.point.x = float(point_cam_3d[0])
            p_cam.point.y = float(point_cam_3d[1])
            p_cam.point.z = float(point_cam_3d[2])

            try:
                p_map = do_transform_point(p_cam, tf_map_cam)
            except Exception as exc:
                self.get_logger().warn(f'Failed to transform point camera->map: {exc}')
                continue

            x_map = p_map.point.x
            y_map = p_map.point.y

            label = names[labels_idx[i]] if names and labels_idx[i] in names else str(
                labels_idx[i]
            )
            confidence = (
                float(confidences[i]) if confidences is not None and len(confidences) > i else 0.0
            )
            if confidence < self.confidence_threshold:
                continue
            if self.allowed_labels and label not in self.allowed_labels:
                self.log_ignored_label(label, confidence)
                self.get_logger().debug(
                    f'Detection ignored: label "{label}" not in semantic class list.'
                )
                continue

            match = self.find_matching_detection(label, x_map, y_map)
            if match is not None:
                idx, existing = match
                if confidence > existing.get('confidence', 0.0):
                    self.replace_detection(idx, float(x_map), float(y_map), confidence, img_rgb, box)
                    map_updated = True
                self.get_logger().info(
                    f'Detection {existing["id"]} replaced: {label} new conf={confidence:.2f}'
                )
                continue

            # Drop detections that fall outside the known map region
            if not self.is_position_mapped(x_map, y_map):
                self.get_logger().debug(
                    f'Detection ignored in unmapped area ({x_map:.2f},{y_map:.2f}).'
                )
                continue

            det_id = self.det_counter
            self.det_counter += 1
            self.detection_records.append(
                {
                    'id': det_id,
                    'label': label,
                    'x': float(x_map),
                    'y': float(y_map),
                    'confidence': confidence,
                    'replacements': 0,
                }
            )
            self.save_detection_assets(det_id, img_rgb, box, label, confidence)
            map_updated = True
            self.get_logger().info(
                f'New detection {det_id}: {label} at (x={x_map:.2f}, y={y_map:.2f}) conf={confidence:.2f}'
            )

        if map_updated:
            self.persist_detections()

    @staticmethod
    def save_image(path: str, img_bgr: np.ndarray) -> bool:
        if cv2 is not None:
            try:
                cv2.imwrite(path, img_bgr)
                return True
            except Exception:
                pass
        try:
            import imageio.v2 as iio  # type: ignore

            iio.imwrite(path, img_bgr[:, :, ::-1])  # imageio expects RGB
            return True
        except Exception:
            return False

    def save_detection_assets(
        self,
        det_id: int,
        img_rgb: np.ndarray,
        box: np.ndarray,
        label: str,
        confidence: float,
    ) -> None:
        img_bgr = img_rgb[:, :, ::-1].copy()
        input_path = self.detections_dir / f'input_{det_id}.jpg'
        self.save_image(str(input_path), img_bgr)

        annotated = img_bgr.copy()
        self.draw_bbox(annotated, box, (0, 255, 0))
        self.draw_label(annotated, box, f'{label} {confidence:.2f}')
        detection_path = self.detections_dir / f'detection_{det_id}.jpg'
        self.save_image(str(detection_path), annotated)

    @staticmethod
    def draw_bbox(image: np.ndarray, box: np.ndarray, color: Tuple[int, int, int], thickness: int = 2) -> None:
        x1, y1, x2, y2 = [int(max(0, round(v))) for v in box]
        h, w = image.shape[:2]
        x1 = max(0, min(x1, w - 1))
        x2 = max(0, min(x2, w - 1))
        y1 = max(0, min(y1, h - 1))
        y2 = max(0, min(y2, h - 1))
        if x2 <= x1 or y2 <= y1:
            return
        image[y1 : y1 + thickness, x1:x2] = color
        image[y2 - thickness:y2, x1:x2] = color
        image[y1:y2, x1 : x1 + thickness] = color
        image[y1:y2, x2 - thickness:x2] = color

    def draw_label(self, image: np.ndarray, box: np.ndarray, text: str) -> None:
        if not text:
            return
        x1, y1, _, _ = [int(max(0, round(v))) for v in box]
        h, w = image.shape[:2]
        x1 = max(0, min(x1, w - 1))
        y1 = max(0, min(y1, h - 1))
        if cv2 is not None:
            font = cv2.FONT_HERSHEY_SIMPLEX
            scale = 0.5
            thickness = 1
            (text_w, text_h), baseline = cv2.getTextSize(text, font, scale, thickness)
            bg_x1 = x1
            bg_y2 = max(0, y1 - 4)
            bg_y1 = max(0, bg_y2 - text_h - 6)
            bg_x2 = min(w - 1, bg_x1 + text_w + 6)
            cv2.rectangle(image, (bg_x1, bg_y1), (bg_x2, bg_y2), (0, 0, 0), cv2.FILLED)
            text_org = (bg_x1 + 3, bg_y2 - 3)
            cv2.putText(image, text, text_org, font, scale, (255, 255, 255), thickness, cv2.LINE_AA)
            return
        try:
            from PIL import Image, ImageDraw, ImageFont  # type: ignore
        except Exception:
            return
        rgb = image[:, :, ::-1]
        pil_img = Image.fromarray(rgb)
        draw = ImageDraw.Draw(pil_img)
        try:
            font = ImageFont.load_default()
        except Exception:
            font = None
        try:
            bbox = draw.textbbox((0, 0), text, font=font)
            text_w = bbox[2] - bbox[0]
            text_h = bbox[3] - bbox[1]
        except Exception:
            text_w, text_h = draw.textsize(text, font=font)
        bg_x1 = x1
        bg_y2 = max(0, y1 - 2)
        bg_y1 = max(0, bg_y2 - text_h - 6)
        bg_x2 = min(w - 1, bg_x1 + text_w + 6)
        draw.rectangle([bg_x1, bg_y1, bg_x2, bg_y2], fill=(0, 0, 0))
        draw.text((bg_x1 + 3, bg_y1 + 3), text, fill=(255, 255, 255), font=font)
        bgr = np.array(pil_img)[:, :, ::-1]
        image[:, :, :] = bgr

    def load_semantic_labels(self) -> Set[str]:
        config_path = self.package_root / 'config' / 'semantic_room_seg_classes.json'
        if not config_path.exists():
            self.get_logger().warn(f'Semantic class config not found: {config_path}')
            return set()
        try:
            with config_path.open('r', encoding='utf-8') as fp:
                data = json.load(fp)
            labels = {str(k) for k in data.keys()}
            self.get_logger().info(
                f'Loaded {len(labels)} semantic classes from semantic_room_seg_classes.json'
            )
            return labels
        except Exception as exc:
            self.get_logger().error(f'Failed to load semantic class list: {exc}')
            return set()

    def reset_ignored_labels_log(self) -> None:
        stamp = self.get_clock().now().to_msg()
        header = [
            'Ignored detections (labels not in semantic class list)',
            f'Start time: {stamp.sec}.{stamp.nanosec:09d}',
            '',
        ]
        try:
            with self.ignored_labels_file.open('w', encoding='utf-8') as fp:
                fp.write('\n'.join(header))
        except Exception as exc:
            self.warn_once('ignored_log_init', f'Failed to initialize ignored detections log: {exc}')

    def log_ignored_label(self, label: str, confidence: float) -> None:
        try:
            stamp = self.get_clock().now().to_msg()
            timestamp = f'{stamp.sec}.{stamp.nanosec:09d}'
        except Exception:
            timestamp = '0.0'
        try:
            with self.ignored_labels_file.open('a', encoding='utf-8') as fp:
                fp.write(f'{timestamp}, label="{label}", conf={confidence:.3f}\n')
        except Exception as exc:
            self.warn_once('ignored_log_write', f'Failed to append ignored detection: {exc}')

    def find_matching_detection(self, label: str, x: float, y: float) -> Optional[Tuple[int, Dict[str, Any]]]:
        tol = max(self.detection_tolerance, 0.0)
        for idx, det in enumerate(self.detection_records):
            if det['label'] != label:
                continue
            dx = det['x'] - x
            dy = det['y'] - y
            if math.hypot(dx, dy) <= tol:
                return idx, det
        return None

    def replace_detection(
        self,
        index: int,
        new_x: float,
        new_y: float,
        new_confidence: float,
        img_rgb: np.ndarray,
        box: np.ndarray,
    ) -> None:
        record = self.detection_records[index]
        replacement_count = int(record.get('replacements', 0)) + 1
        self.archive_detection_assets(record['id'])
        record['x'] = new_x
        record['y'] = new_y
        record['confidence'] = new_confidence
        record['replacements'] = replacement_count
        self.save_detection_assets(record['id'], img_rgb, box, record['label'], new_confidence)

    def archive_detection_assets(self, det_id: int) -> None:
        slots = [self.detections_dir / f'input_{det_id}_replaced_{i}.jpg' for i in (1, 2, 3)]
        det_slots = [self.detections_dir / f'detection_{det_id}_replaced_{i}.jpg' for i in (1, 2, 3)]
        self.rotate_slots(slots)
        self.rotate_slots(det_slots)
        input_file = self.detections_dir / f'input_{det_id}.jpg'
        detection_file = self.detections_dir / f'detection_{det_id}.jpg'
        if input_file.exists():
            input_file.rename(slots[2])
        if detection_file.exists():
            detection_file.rename(det_slots[2])

    def rotate_slots(self, slot_paths: List[Path]) -> None:
        if slot_paths[0].exists():
            slot_paths[0].unlink()
        if slot_paths[1].exists():
            slot_paths[1].rename(slot_paths[0])
        if slot_paths[2].exists():
            slot_paths[2].rename(slot_paths[1])

    def resolve_package_root(self) -> Path:
        current = Path(__file__).resolve()
        for ancestor in current.parents:
            candidate = ancestor / 'src' / 'my_robot_pkg'
            if candidate.exists():
                return candidate
        try:
            from ament_index_python.packages import get_package_share_directory

            share_dir = Path(get_package_share_directory('my_robot_pkg'))
            return share_dir
        except Exception:
            return current.parent

    def resolve_model_path(self) -> Optional[Path]:
        """
        Try to locate the YOLO model.
        Order:
        1) Parameter model_path (relative paths resolved against package_root).
        2) Workspace models directory (../.. /models/my_robot.pt).
        3) Package-local models directory (./models/my_robot.pt inside package_root).
        4) Src sibling models directory (../models/my_robot.pt).
        """
        param_value = self.get_parameter('model_path').value
        candidates: List[Path] = []
        if param_value:
            custom = Path(str(param_value))
            if not custom.is_absolute():
                custom = (self.package_root / custom).resolve()
            candidates.append(custom)

        candidates.extend(
            [
                (self.package_root.parent.parent / 'models' / 'my_robot.pt').resolve(),  # workspace root/models
                (self.package_root / 'models' / 'my_robot.pt').resolve(),  # package share/models
                (self.package_root.parent / 'models' / 'my_robot.pt').resolve(),  # src/models
            ]
        )

        for path in candidates:
            if path.exists():
                return path
        return None

    def publish_detection_map(self) -> None:
        markers = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        for det in self.detection_records:
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = stamp
            marker.ns = 'detections'
            marker.id = int(det['id'])
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = float(det['x'])
            marker.pose.position.y = float(det['y'])
            marker.pose.position.z = 0.05
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.15
            marker.scale.y = 0.15
            marker.scale.z = 0.1
            marker.color.r = 0.2
            marker.color.g = 0.7
            marker.color.b = 0.3
            marker.color.a = 0.85
            markers.markers.append(marker)

            text_marker = Marker()
            text_marker.header.frame_id = 'map'
            text_marker.header.stamp = stamp
            text_marker.ns = 'detections_text'
            text_marker.id = int(det['id']) + 100000
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = float(det['x'])
            text_marker.pose.position.y = float(det['y'])
            text_marker.pose.position.z = 0.25
            text_marker.pose.orientation.w = 1.0
            text_marker.scale.z = 0.2
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 0.9
            text_marker.text = f"{det['label']} ({det['id']})"
            markers.markers.append(text_marker)

        if markers.markers:
            self.map_pub.publish(markers)
            now = self.get_clock().now()
            if self.last_map_log_time is None:
                should_log = True
            else:
                delta = now - self.last_map_log_time
                should_log = delta.nanoseconds >= self.map_log_interval.nanoseconds
            if should_log:
                self.get_logger().info(
                    f'/detections_markers published with {len(self.detection_records)} detections.'
                )
                self.last_map_log_time = now

    def _report_no_detection(self, message: str) -> None:
        if not self.no_detection_logged:
            self.get_logger().info(message)
            self.no_detection_logged = True

    def _mark_detection_activity(self) -> None:
        self.no_detection_logged = False

    def warn_once(self, key: str, message: str) -> None:
        if key not in self.warned_messages:
            self.get_logger().warn(message)
            self.warned_messages.add(key)

    def is_position_mapped(self, x_map: float, y_map: float) -> bool:
        if self.map_msg is None or self.map_np is None:
            return False
        info = self.map_msg.info
        origin = info.origin.position
        res = info.resolution
        width = info.width
        height = info.height
        mx = int((x_map - origin.x) / res)
        my = int((y_map - origin.y) / res)
        if mx < 0 or my < 0 or mx >= width or my >= height:
            return False
        cell = int(self.map_np[my, mx])
        return cell >= 0

    def persist_detections(self) -> None:
        data = []
        for det in self.detection_records:
            data.append(
                {
                    'id': det['id'],
                    'label': det['label'],
                    'position': {'x': det['x'], 'y': det['y']},
                    'confidence': det.get('confidence', 0.0),
                    'replacements': det.get('replacements', 0),
                }
            )
        try:
            with self.detections_file.open('w', encoding='utf-8') as fp:
                json.dump(data, fp, indent=2)
        except Exception as exc:
            self.get_logger().error(f'Failed to save detections.json: {exc}')

    def load_existing_detections(self) -> List[Dict[str, Any]]:
        if not self.detections_file.exists():
            return []
        try:
            with self.detections_file.open('r', encoding='utf-8') as fp:
                data = json.load(fp)
        except Exception as exc:
            self.get_logger().warn(f'Unable to load detections.json: {exc}')
            return []
        loaded: List[Dict[str, Any]] = []
        for entry in data:
            pos = entry.get('position', {})
            loaded.append(
                {
                    'id': int(entry.get('id', len(loaded))),
                    'label': str(entry.get('label', 'unknown')),
                    'x': float(pos.get('x', 0.0)),
                    'y': float(pos.get('y', 0.0)),
                    'confidence': float(entry.get('confidence', 0.0)),
                    'replacements': int(entry.get('replacements', 0)),
                }
            )
        return loaded


def main(args=None):
    rclpy.init(args=args)
    node = MyRobotDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
