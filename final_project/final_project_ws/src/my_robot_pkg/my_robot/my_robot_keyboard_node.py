#!/usr/bin/env python3
"""
Keyboard teleoperation node for /cmd_vel.

Maps simple key presses to Twist commands so the simulated robot can be driven
without additional tooling. Includes helpers to adjust speed on-the-fly and to
gracefully stop the robot.
"""

import atexit
import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


def build_help(linear_speed, angular_speed):
    """Return a formatted help text summarizing current speed limits."""
    return (
        "Keyboard teleoperation active:\n"
        "  w/s : move forward/backward\n"
        "  a/d : rotate left/right\n"
        "  i/k : increase/decrease linear speed\n"
        "  j/l : increase/decrease angular speed\n"
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

        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        self.linear_increment = self.linear_speed * 0.1
        self.angular_increment = self.angular_speed * 0.1

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Track terminal state in order to restore it after the node exits.
        self._settings = termios.tcgetattr(sys.stdin)
        atexit.register(self._restore_terminal)

        self.get_logger().info(build_help(self.linear_speed, self.angular_speed))

        self.current_twist = Twist()
        self.pending_stop_publishes = 0

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
            if rclpy.ok():
                rclpy.shutdown()
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
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
