#!/usr/bin/env python3
"""
cleaning_server
---------------
- Lifecycle: configured/activated by Nav2 lifecycle manager.
- Operational: acts only when behaviour_server publishes Mode == CLEANING.
- Publishes a simple "vacuuming" motion command (cmd_vel) when active.

Real logic ideas:
- Replace constant twist with a coverage planner or SLAM-based exploration.
- Use bump sensors / lidar to avoid obstacles.
"""
import rclpy
from rclpy.lifecycle import Node as LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from .modes import Mode

class Cleaning(LifecycleNode):
    def __init__(self):
        super().__init__('cleaning_server')
        self.pub_cmd = None
        self._mode = Mode.IDLE

    def on_configure(self, _):
        qos = QoSProfile(depth=1)
        self.pub_cmd = self.create_publisher(Twist, '/myRobot/cmd_vel', qos)
        self.create_subscription(String, '/myRobot/behaviour/mode', self._on_mode, qos)
        self.create_timer(0.5, self._tick)
        self.get_logger().info('cleaning configured')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, _):
        self.get_logger().info('cleaning activated')
        return TransitionCallbackReturn.SUCCESS

    def _on_mode(self, msg: String):
        try:
            self._mode = Mode(msg.data)
        except Exception:
            self._mode = Mode.IDLE

    def _tick(self):
        if self._mode != Mode.CLEANING:
            return
        twist = Twist()
        twist.linear.x = 0.2
        twist.angular.z = 0.3
        self.pub_cmd.publish(twist)

def main():
    rclpy.init()
    rclpy.spin(Cleaning())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
