#!/usr/bin/env python3
import datetime
import math
import time
from pathlib import Path
import threading
from typing import Optional
import select
import sys
import termios
import tty
import yaml

from ament_index_python.packages import get_package_share_directory
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.exceptions import ParameterAlreadyDeclaredException
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

try:  # ROS distros differ on available exception types
    from rclpy.exceptions import RCLException as _RCLException  # type: ignore
except ImportError:  # pragma: no cover - fallback for older/newer distros
    _RCLException = Exception


def quat_to_yaw(q):
    """Return yaw from geometry_msgs.msg.Quaternion"""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_angle(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


class FrontierExplorer(Node):
    """Simple frontier explorer: drives toward nearest frontier during SLAM."""

    def __init__(self):
        super().__init__('frontier_explorer')

        defaults = self._load_default_params()

        self.declare_parameter('map_topic', defaults.get('map_topic', '/map'))
        self.declare_parameter('base_frame', defaults.get('base_frame', 'base_link'))
        self.declare_parameter('linear_speed', defaults.get('linear_speed', 0.5))
        self.declare_parameter('angular_speed', defaults.get('angular_speed', 0.8))
        self.declare_parameter('goal_tolerance', defaults.get('goal_tolerance', 0.25))
        self.declare_parameter('min_frontier_neighbors', defaults.get('min_frontier_neighbors', 1))
        self.declare_parameter('timer_period', defaults.get('timer_period', 0.5))
        self.declare_parameter('safety_stop_distance', defaults.get('safety_stop_distance', 0.40))
        self.declare_parameter('safety_angle_deg', defaults.get('safety_angle_deg', 45.0))
        self.declare_parameter('stuck_distance_thresh', defaults.get('stuck_distance_thresh', 0.05))
        self.declare_parameter('stuck_time_sec', defaults.get('stuck_time_sec', 5.0))
        self.declare_parameter('stuck_spin_speed', defaults.get('stuck_spin_speed', 0.8))
        self.declare_parameter('tf_buffer_duration', defaults.get('tf_buffer_duration', 30.0))
        try:
            self.declare_parameter('use_sim_time', defaults.get('use_sim_time', False))
        except ParameterAlreadyDeclaredException:
            pass
        default_map_dir = self._choose_default_map_dir()
        self.declare_parameter('map_save_directory', str(defaults.get('map_save_directory', default_map_dir)))

        self.map_topic = self.get_parameter('map_topic').value
        self.base_frame = self.get_parameter('base_frame').value
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)
        self.min_frontier_neighbors = int(self.get_parameter('min_frontier_neighbors').value)
        self.safety_stop_distance = float(self.get_parameter('safety_stop_distance').value)
        self.safety_angle_rad = math.radians(float(self.get_parameter('safety_angle_deg').value))
        self.stuck_distance_thresh = float(self.get_parameter('stuck_distance_thresh').value)
        self.stuck_time_sec = float(self.get_parameter('stuck_time_sec').value)
        self.stuck_spin_speed = float(self.get_parameter('stuck_spin_speed').value)
        self.tf_buffer_duration = float(self.get_parameter('tf_buffer_duration').value)
        self.map_save_directory = Path(
            str(self.get_parameter('map_save_directory').value)
        ).expanduser()

        self._last_obstacle_warn = None
        self._last_no_frontier = None
        self._last_reached_frontier = None
        self._last_tf_warn = None
        self._last_motion_time = self.get_clock().now()
        self._last_motion_pose: Optional[tuple[float, float]] = None
        self._last_state: Optional[str] = None
        self._quit_requested = False

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.map_sub = self.create_subscription(OccupancyGrid, self.map_topic, self.map_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.tf_buffer = Buffer(cache_time=Duration(seconds=self.tf_buffer_duration))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.latest_map: Optional[OccupancyGrid] = None
        self.latest_scan: Optional[LaserScan] = None

        period = float(self.get_parameter('timer_period').value)
        self.timer = self.create_timer(period, self.control_loop)

        self.get_logger().info(
            f'Frontier explorer started (map_topic={self.map_topic}, base_frame={self.base_frame})'
        )
        self._start_keyboard_monitor()

    def map_callback(self, msg: OccupancyGrid):
        self.latest_map = msg

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg

    def _keyboard_monitor(self):
        """Listen for 'q' on stdin to trigger a clean shutdown when run in a terminal."""
        if not sys.stdin.isatty():
            if rclpy.ok():
                self.get_logger().warn('stdin is not a TTY; q quit shortcut disabled')
            return
        settings = termios.tcgetattr(sys.stdin)
        while rclpy.ok() and not self._quit_requested:
            try:
                tty.setcbreak(sys.stdin.fileno())
                rlist, _, _ = select.select([sys.stdin], [], [], 0.2)
            except Exception:
                break
            finally:
                try:
                    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
                except Exception:
                    pass
            if not rlist:
                continue
            ch = sys.stdin.read(1)
            if ch.lower() == 'q':
                self._quit_requested = True
                if rclpy.ok():
                    self.get_logger().info('Keyboard quit requested (q), initiating shutdown...')
                break

    def _start_keyboard_monitor(self):
        t = threading.Thread(target=self._keyboard_monitor, daemon=True)
        t.start()


    def get_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.latest_map.header.frame_id,
                self.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1),
            )
            trans = tf.transform.translation
            rot = tf.transform.rotation
            return trans.x, trans.y, quat_to_yaw(rot)
        except TransformException as e:
            if self._should_log('_last_tf_warn', 2.0):
                self.get_logger().warn(f'TF lookup failed: {e}')
            return None

    # --------------------------
    #  DOOR / PASSAGE HEURISTIC
    # --------------------------
    def door_passage_bonus(self, frontiers, grid, res, origin,
                           width_m: float = 0.8,
                           depth_m: float = 1.2) -> np.ndarray:
        """
        Give extra weight to frontier points that look like narrow doors/passages.
        """
        h, w = grid.shape
        bonuses = np.ones(len(frontiers), dtype=np.float32)

        width_cells = max(1, int(width_m / res))
        depth_cells = max(1, int(depth_m / res))

        for i, (fx, fy) in enumerate(frontiers):
            cx = int((fx - origin[0]) / res)
            cy = int((fy - origin[1]) / res)
            if cx < 1 or cy < 1 or cx >= w - 1 or cy >= h - 1:
                continue

            y_min = max(0, cy - depth_cells)
            y_max = min(h, cy + depth_cells)
            x_min = max(0, cx - width_cells)
            x_max = min(w, cx + width_cells)

            patch = grid[y_min:y_max, x_min:x_max]
            if patch.size == 0:
                continue

            unknown_frac = np.mean(patch == -1)

            left_col = patch[:, 0]
            right_col = patch[:, -1]
            left_occ_frac = np.mean(left_col == 100) if left_col.size > 0 else 0.0
            right_occ_frac = np.mean(right_col == 100) if right_col.size > 0 else 0.0

            looks_like_corridor = (left_occ_frac > 0.3 and right_occ_frac > 0.3)
            lots_unknown = unknown_frac > 0.4

            if looks_like_corridor and lots_unknown:
                bonuses[i] += 1.0  # double weight

        return bonuses

    # --------------------------
    #   FRONTIER PROCESSING
    # --------------------------

    def find_frontiers(self, grid: np.ndarray, res: float, origin):
        unknown = grid == -1
        free = grid == 0

        neighbor_free = (
            np.roll(free, 1, axis=0) | np.roll(free, -1, axis=0) |
            np.roll(free, 1, axis=1) | np.roll(free, -1, axis=1)
        )
        frontier_mask = unknown & neighbor_free

        if self.min_frontier_neighbors > 1:
            kernel = (
                neighbor_free.astype(np.int8) +
                np.roll(neighbor_free, 1, axis=0).astype(np.int8) +
                np.roll(neighbor_free, -1, axis=0).astype(np.int8) +
                np.roll(neighbor_free, 1, axis=1).astype(np.int8) +
                np.roll(neighbor_free, -1, axis=1).astype(np.int8)
            )
            frontier_mask &= kernel >= self.min_frontier_neighbors

        ys, xs = np.nonzero(frontier_mask)
        if len(xs) == 0:
            return None

        wx = origin[0] + (xs + 0.5) * res
        wy = origin[1] + (ys + 0.5) * res
        return np.stack([wx, wy], axis=1)

    def pick_frontier_target(self, frontiers: np.ndarray, pos: np.ndarray,
                             door_bonus: Optional[np.ndarray] = None
                             ) -> tuple[np.ndarray, int, np.ndarray]:
        """Select a frontier with attraction + door passage bonus."""
        vecs = frontiers - pos
        dists = np.linalg.norm(vecs, axis=1) + 1e-6

        base_weights = 1.0 / dists

        if door_bonus is None:
            weights = base_weights
        else:
            weights = base_weights * door_bonus

        attraction = np.sum((vecs / dists[:, None]) * weights[:, None], axis=0)

        if np.linalg.norm(attraction) > 1e-6:
            desired_dir = math.atan2(attraction[1], attraction[0])
            ang_errors = np.abs(np.arctan2(vecs[:, 1], vecs[:, 0]) - desired_dir)
            ang_errors = np.mod(ang_errors + math.pi, 2 * math.pi) - math.pi
            idx = int(np.argmin(np.abs(ang_errors)))
        else:
            idx = int(np.argmin(dists))

        return frontiers[idx], idx, dists

    # --------------------------
    #   SAFETY & MOVEMENT
    # --------------------------

    def closest_obstacle_ahead(self) -> Optional[float]:
        if self.latest_scan is None:
            return None

        scan = self.latest_scan
        ranges = np.array(scan.ranges, dtype=np.float32)
        angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
        mask = np.isfinite(ranges) & (np.abs(angles) <= self.safety_angle_rad)
        if not np.any(mask):
            return None
        return float(np.min(ranges[mask]))

    def turn_direction_by_clearance(self) -> float:
        if self.latest_scan is None:
            return 1.0

        scan = self.latest_scan
        ranges = np.array(scan.ranges, dtype=np.float32)
        angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment

        left_mask = np.isfinite(ranges) & (angles > 0.0) & (angles <= math.pi / 2.0)
        right_mask = np.isfinite(ranges) & (angles < 0.0) & (angles >= -math.pi / 2.0)

        def min_dist(mask):
            return np.min(ranges[mask]) if np.any(mask) else 0.0

        left = min_dist(left_mask)
        right = min_dist(right_mask)

        if left == 0.0 and right == 0.0:
            return 1.0
        if left > right:
            return 1.0
        return -1.0

    def warn_obstacle_once(self, distance: float, period: float = 1.0):
        now = self.get_clock().now()
        if self._last_obstacle_warn is None or (now - self._last_obstacle_warn).nanoseconds > period * 1e9:
            self._last_obstacle_warn = now
            self.get_logger().warn(f'Obstacle at {distance:.2f} m, rotating to avoid')

    def _should_log(self, attr_name: str, period: float) -> bool:
        now = self.get_clock().now()
        last = getattr(self, attr_name, None)
        if last is None or (now - last).nanoseconds > period * 1e9:
            setattr(self, attr_name, now)
            return True
        return False

    def _log_state(self, state: str, twist: Twist):
        """Log state transitions with commanded speeds."""
        if state != self._last_state:
            self._last_state = state
            self.get_logger().info(
                f'State -> {state}: lin {twist.linear.x:.2f} m/s, ang {twist.angular.z:.2f} rad/s'
            )

    def stop_robot(self, repeats: int = 3, delay_s: float = 0.1):
        """Publish a zero twist to stop the robot, with a few repeats to ensure delivery."""
        if not rclpy.ok():
            return
        self.get_logger().info('Stopping robot (zero cmd_vel)...')
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
                
        for _ in range(max(1, repeats)):
            try:
                self.cmd_pub.publish(twist)
            except _RCLException as exc:  # pragma: no cover - defensive logging
                # If the context is invalid, skip further publishes.
                if rclpy.ok():
                    self.get_logger().warn(f'Failed to publish stop command: {exc}')
                break
            time.sleep(max(0.0, delay_s))

    def _choose_default_map_dir(self) -> Path:
        """
        Prefer the package source map folder if it exists; otherwise fall back to the
        installed share directory. This keeps saved maps in the repo during development.
        """
        base_path = Path(__file__).resolve()
        # Walk up to locate a workspace root that has src/my_robot_pkg/map
        for anc in base_path.parents:
            candidate = anc.parent / 'src' / 'my_robot_pkg' / 'map'
            if candidate.exists():
                return candidate
        return Path(get_package_share_directory('my_robot_pkg')) / 'map'

    def _load_default_params(self) -> dict:
        """Load default parameters from explore_config.yaml if available."""
        config_path = Path(get_package_share_directory('my_robot_pkg')) / 'config' / 'explore_config.yaml'
        try:
            with open(config_path, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
            return data.get('frontier_explorer', {}).get('ros__parameters', {}) or {}
        except Exception as exc:  # pragma: no cover - defensive logging
            self.get_logger().warn(f'Could not load explore_config.yaml defaults: {exc}')
            return {}

    def save_latest_map(self) -> Optional[tuple[Path, Path]]:
        """Persist the latest known occupancy grid as PGM + YAML with a timestamped name."""
        if self.latest_map is None:
            self.get_logger().warn('No map received yet; nothing to save')
            return None

        map_msg = self.latest_map
        grid = np.array(map_msg.data, dtype=np.int16).reshape(
            (map_msg.info.height, map_msg.info.width)
        )
        if grid.size == 0:
            self.get_logger().warn('Latest map is empty; nothing to save')
            return None

        try:
            self.map_save_directory.mkdir(parents=True, exist_ok=True)
        except Exception as exc:  # pragma: no cover - defensive logging
            self.get_logger().error(
                f'Failed to create map directory {self.map_save_directory}: {exc}'
            )
            return None

        stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        stem = f'map_{stamp}'
        image_path = self.map_save_directory / f'{stem}.pgm'
        yaml_path = self.map_save_directory / f'{stem}.yaml'

        # Convert occupancy values to grayscale map (match ROS map_saver convention).
        image = np.full(grid.shape, 205, dtype=np.uint8)
        image[grid == 0] = 254
        image[grid == 100] = 0
        image = np.flipud(image)  # flip so origin matches PGM top-left convention

        header = (
            f'P5\n'
            f'# CREATOR: frontier_explorer\n'
            f'{map_msg.info.width} {map_msg.info.height}\n'
            f'255\n'
        )

        try:
            with open(image_path, 'wb') as f:
                f.write(header.encode('ascii'))
                f.write(image.tobytes())
        except Exception as exc:  # pragma: no cover - defensive logging
            self.get_logger().error(f'Failed to write map image {image_path}: {exc}')
            return None

        origin_yaw = quat_to_yaw(map_msg.info.origin.orientation)
        yaml_content = (
            f'image: {image_path.name}\n'
            f'mode: trinary\n'
            f'resolution: {map_msg.info.resolution}\n'
            f'origin: [{map_msg.info.origin.position.x}, '
            f'{map_msg.info.origin.position.y}, {origin_yaw}]\n'
            f'negate: 0\n'
            f'occupied_thresh: 0.65\n'
            f'free_thresh: 0.25\n'
        )

        try:
            yaml_path.write_text(yaml_content, encoding='ascii')
        except Exception as exc:  # pragma: no cover - defensive logging
            self.get_logger().error(f'Failed to write map metadata {yaml_path}: {exc}')
            return None

        if rclpy.ok():
            self.get_logger().info(f'Saved map to {yaml_path} (image: {image_path})')
        else:
            print(f'[frontier_explorer] Saved map to {yaml_path} (image: {image_path})')
        return image_path, yaml_path

    # --------------------------
    #        CONTROL LOOP
    # --------------------------

    def control_loop(self):
        if self._quit_requested:
            self.stop_robot()
            self.save_latest_map()
            rclpy.shutdown()
            return
        if self.latest_map is None:
            return

        pose = self.get_pose()
        if pose is None:
            return

        px, py, yaw = pose

        now = self.get_clock().now()
        if self._last_motion_pose is None:
            self._last_motion_pose = (px, py)
            self._last_motion_time = now
        else:
            dist_moved = math.hypot(px - self._last_motion_pose[0], py - self._last_motion_pose[1])
            if dist_moved > self.stuck_distance_thresh:
                self._last_motion_pose = (px, py)
                self._last_motion_time = now

        stuck = (now - self._last_motion_time).nanoseconds > self.stuck_time_sec * 1e9

        msg = self.latest_map
        res = msg.info.resolution
        origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        grid = np.array(msg.data, dtype=np.int16).reshape((msg.info.height, msg.info.width))

        frontiers = self.find_frontiers(grid, res, origin)
        if frontiers is None:
            twist = Twist()
            twist.angular.z = 0.3
            self.cmd_pub.publish(twist)
            if self._should_log('_last_no_frontier', 5.0):
                self.get_logger().info('No frontiers found, rotating...')
            self._log_state('search_rotate', twist)
            return

        pos = np.array([px, py])

        # NEW: door/passage attraction
        door_bonus = self.door_passage_bonus(frontiers, grid, res, origin)

        # Pick a frontier
        target, idx, dists = self.pick_frontier_target(frontiers, pos, door_bonus=door_bonus)

        dx, dy = target - pos
        angle_to_target = math.atan2(dy, dx)
        ang_err = wrap_angle(angle_to_target - yaw)

        twist = Twist()

        obstacle = self.closest_obstacle_ahead()
        if obstacle is not None and obstacle < self.safety_stop_distance:
            twist.linear.x = 0.0
            direction = self.turn_direction_by_clearance()
            twist.angular.z = direction * max(0.3, self.angular_speed)
            self.cmd_pub.publish(twist)
            self.warn_obstacle_once(obstacle, period=1.0)
            self._log_state('avoid_obstacle', twist)
            return

        if stuck:
            twist = Twist()
            twist.angular.z = self.stuck_spin_speed
            self.cmd_pub.publish(twist)
            if self._should_log('_last_no_frontier', 5.0):
                self.get_logger().warn('Stuck for {:.1f}s, spinning to free'.format(self.stuck_time_sec))
            self._log_state('stuck_spin', twist)
            return

        if dists[idx] < self.goal_tolerance:
            twist.angular.z = 0.4
            if self._should_log('_last_reached_frontier', 2.0):
                self.get_logger().info('Reached frontier, rotating to seek new ones')
            self._log_state('goal_rotate', twist)
        else:
            twist.linear.x = self.linear_speed
            twist.angular.z = max(-self.angular_speed, min(self.angular_speed, 1.5 * ang_err))
            self._log_state('move_ahead', twist)

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if rclpy.ok():
            node.get_logger().info('Keyboard interrupt received, stopping exploration...')
    finally:
        try:
            node.stop_robot()
        except Exception as exc:  # pragma: no cover - defensive logging
            # Avoid crashing on shutdown if ROS context is already invalid.
            if rclpy.ok():
                node.get_logger().warn(f'Error while stopping robot: {exc}')
        try:
            node.save_latest_map()
        except Exception as exc:  # pragma: no cover - defensive logging
            if rclpy.ok():
                node.get_logger().warn(f'Error while saving map: {exc}')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
