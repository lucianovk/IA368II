#!/usr/bin/env python3
"""
Keyboard teleoperation node for /cmd_vel.

Maps simple key presses to Twist commands so the simulated robot can be driven
without additional tooling. Includes helpers to adjust speed on-the-fly and to
gracefully stop the robot.
"""

import atexit
import datetime
from pathlib import Path
import select
import sys
import termios
import tty
import time

from ament_index_python.packages import get_package_share_directory
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node


def quat_to_yaw(q):
    """Return yaw from geometry_msgs.msg.Quaternion"""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return float(np.arctan2(siny_cosp, cosy_cosp))


def build_help(linear_speed, angular_speed):
    """Return a formatted help text summarizing current speed limits."""
    return (
        "Keyboard teleoperation active:\n"
        "  w/s : move forward/backward\n"
        "  a/d : rotate left/right\n"
        "  i/k : increase/decrease linear speed\n"
        "  j/l : increase/decrease angular speed\n"
        "  m   : save map snapshot\n"
        "  space: stop\n"
        "  q   : quit node\n"
        f"  current speeds -> linear: {linear_speed:.2f} m/s | angular: {angular_speed:.2f} rad/s\n"
    )

# Mapping between key presses and normalized linear/angular commands.
MOVE_BINDINGS = {
    'w': (1.0, 0.0),
    's': (-1.0, 0.0),
    'a': (0.0, 1.0),
    'd': (0.0, -1.0),
}


class MyRobotKeyboardNode(Node):
    def __init__(self):
        super().__init__('my_robot_keyboard_node')
        # Node responsible for translating keyboard commands into velocity messages.

        # Parameters that define the nominal speed values used for each key press.
        self.declare_parameter('linear_speed', 0.2)
        self.declare_parameter('angular_speed', 1.0)
        self.declare_parameter('map_topic', '/map')
        default_map_dir = self._choose_default_map_dir()
        self.declare_parameter('map_save_directory', str(default_map_dir))
        self.declare_parameter('map_save_occupied_thresh', 0.65)
        self.declare_parameter('map_save_free_thresh', 0.25)
        try:
            self.declare_parameter('use_sim_time', False)
        except Exception:
            pass

        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        self.linear_increment = self.linear_speed * 0.1
        self.angular_increment = self.angular_speed * 0.1
        self.map_topic = self.get_parameter('map_topic').value
        self.map_save_directory = Path(str(self.get_parameter('map_save_directory').value)).expanduser()
        self.map_save_occupied_thresh = float(self.get_parameter('map_save_occupied_thresh').value)
        self.map_save_free_thresh = float(self.get_parameter('map_save_free_thresh').value)

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.map_sub = self.create_subscription(
            OccupancyGrid, self.map_topic, self.map_callback, 10
        )

        # Track terminal state in order to restore it after the node exits.
        self._settings = termios.tcgetattr(sys.stdin)
        atexit.register(self._restore_terminal)

        self.get_logger().info(build_help(self.linear_speed, self.angular_speed))

        self.current_twist = Twist()
        self.pending_stop_publishes = 0
        self.latest_map: OccupancyGrid | None = None

        # Timer acts as a polling loop that reads keys and publishes commands.
        self.timer = self.create_timer(0.05, self._tick)

    def _publish_current(self):
        """Send the most recent Twist command to the topic."""
        self.publisher.publish(self.current_twist)

    def _restore_terminal(self):
        """Restore terminal echo/canonical mode on exit."""
        try:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._settings)
        except termios.error:
            pass

    def _get_key(self):
        """Read a single key press from stdin in non-blocking raw mode."""
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._settings)
        return key

    def map_callback(self, msg: OccupancyGrid):
        """Cache the latest occupancy grid for saving."""
        self.latest_map = msg

    def stop_robot(self):
        """Publish a zero Twist to stop the robot."""
        if not rclpy.ok():
            return
        twist = Twist()
        try:
            self.publisher.publish(twist)
        except Exception as exc:  # pragma: no cover - defensive logging
            self.get_logger().warn(f'Failed to publish stop command: {exc}')
            return
        time.sleep(0.1)

    def save_latest_map(self):
        """Write the latest map to a timestamped PGM/YAML, if available."""
        if self.latest_map is None:
            self.get_logger().warn('No map received yet; nothing to save')
            return

        msg = self.latest_map
        grid = np.array(msg.data, dtype=np.int16).reshape(
            (msg.info.height, msg.info.width)
        )
        if grid.size == 0:
            self.get_logger().warn('Latest map is empty; nothing to save')
            return

        try:
            self.map_save_directory.mkdir(parents=True, exist_ok=True)
        except Exception as exc:  # pragma: no cover - defensive logging
            self.get_logger().error(f'Failed to create map directory {self.map_save_directory}: {exc}')
            return

        stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        stem = f'map_{stamp}'
        image_path = self.map_save_directory / f'{stem}.pgm'
        yaml_path = self.map_save_directory / f'{stem}.yaml'

        image = np.full(grid.shape, 205, dtype=np.uint8)
        image[grid == 0] = 254
        image[grid == 100] = 0
        image = np.flipud(image)

        header = (
            f'P5\n'
            f'# CREATOR: my_robot_keyboard_node\n'
            f'{msg.info.width} {msg.info.height}\n'
            f'255\n'
        )

        try:
            with open(image_path, 'wb') as f:
                f.write(header.encode('ascii'))
                f.write(image.tobytes())
        except Exception as exc:  # pragma: no cover - defensive logging
            self.get_logger().error(f'Failed to write map image {image_path}: {exc}')
            return

        origin_yaw = quat_to_yaw(msg.info.origin.orientation)
        yaml_content = (
            f'image: {image_path.name}\n'
            f'mode: trinary\n'
            f'resolution: {msg.info.resolution}\n'
            f'origin: [{msg.info.origin.position.x}, '
            f'{msg.info.origin.position.y}, {origin_yaw}]\n'
            f'negate: 0\n'
            f'occupied_thresh: {self.map_save_occupied_thresh}\n'
            f'free_thresh: {self.map_save_free_thresh}\n'
        )

        try:
            yaml_path.write_text(yaml_content, encoding='ascii')
        except Exception as exc:  # pragma: no cover - defensive logging
            self.get_logger().error(f'Failed to write map metadata {yaml_path}: {exc}')
            return

        self.get_logger().info(f'Saved map to {yaml_path} (image: {image_path})')

    def _choose_default_map_dir(self) -> Path:
        """Prefer source map folder when available; otherwise use installed share."""
        base_path = Path(__file__).resolve()
        for anc in base_path.parents:
            candidate = anc.parent / 'src' / 'my_robot_pkg' / 'map'
            if candidate.exists():
                return candidate
        return Path(get_package_share_directory('my_robot_pkg')) / 'map'

    def _tick(self):
        """Main loop: interpret key presses and publish motion commands."""
        key = self._get_key()
        if key == '':
            if self.pending_stop_publishes > 0:
                self._publish_current()
                self.pending_stop_publishes -= 1
            return

        if key == 'q':
            self.get_logger().info('Keyboard teleop shutting down.')
            try:
                self.stop_robot()
                self.save_latest_map()
            finally:
                if rclpy.ok():
                    rclpy.shutdown()
            return
        if key == 'm':
            self.save_latest_map()
            return

        if key == ' ':
            self.current_twist = Twist()
            self._publish_current()
            self.pending_stop_publishes = 5
            self.get_logger().info('Stop command sent (space).')
            return

        if key == 'i':
            self.linear_speed += self.linear_increment
            self.get_logger().info(f'Linear speed: {self.linear_speed:.3f} m/s')
            return
        if key == 'k':
            self.linear_speed = max(0.0, self.linear_speed - self.linear_increment)
            self.get_logger().info(f'Linear speed: {self.linear_speed:.3f} m/s')
            return
        if key == 'j':
            self.angular_speed = max(0.1, self.angular_speed - self.angular_increment)
            self.get_logger().info(f'Angular speed: {self.angular_speed:.3f} rad/s')
            return
        if key == 'l':
            self.angular_speed += self.angular_increment
            self.get_logger().info(f'Angular speed: {self.angular_speed:.3f} rad/s')
            return

        if key in MOVE_BINDINGS:
            linear, angular = MOVE_BINDINGS[key]
            self.current_twist.linear.x = linear * self.linear_speed
            self.current_twist.angular.z = angular * self.angular_speed
            self._publish_current()
        else:
            self.get_logger().info(build_help(self.linear_speed, self.angular_speed))


def main(args=None):
    rclpy.init(args=args)
    node = MyRobotKeyboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received, stopping teleop...')
    finally:
        try:
            node.stop_robot()
        except Exception as exc:  # pragma: no cover - defensive logging
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
