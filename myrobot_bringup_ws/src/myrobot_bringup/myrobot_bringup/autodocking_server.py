#!/usr/bin/env python3
"""
autodocking_server
------------------
- Lifecycle: configured/activated by Nav2 lifecycle manager.
- Operational: acts only when Mode == AUTODOCKING.
- Reads docking signal (strength + relative angle) and drives toward the base.

Real logic ideas:
- Implement a visual servoing or IR beacon homing controller.
- Add safety stop when obstacle detected.
"""
import rclpy
from rclpy.lifecycle import Node as LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.qos import QoSProfile
from std_msgs.msg import Float32, String
from geometry_msgs.msg import Twist
from .modes import Mode

class AutoDocking(LifecycleNode):
    def __init__(self):
        super().__init__('autodocking_server')
        self._mode = Mode.IDLE
        self._last_strength = 0.0
        self._last_angle = 0.0

    def on_configure(self, _):
        qos = QoSProfile(depth=1)
        self.pub_cmd = self.create_publisher(Twist, '/myRobot/cmd_vel', qos)
        self.create_subscription(Float32, '/myRobot/charging_base/strengthSignal', self._on_strength, qos)
        self.create_subscription(Float32, '/myRobot/charging_base/relativeAngle', self._on_angle, qos)
        self.create_subscription(String, '/myRobot/behaviour/mode', self._on_mode, qos)
        self.create_timer(0.2, self._tick)
        self.get_logger().info('autodocking configured')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, _):
        self.get_logger().info('autodocking activated')
        return TransitionCallbackReturn.SUCCESS

    def _on_mode(self, msg: String):
        try:
            self._mode = Mode(msg.data)
        except Exception:
            self._mode = Mode.IDLE

    def _on_strength(self, msg: Float32):
        self._last_strength = msg.data

    def _on_angle(self, msg: Float32):
        self._last_angle = msg.data

    def _tick(self):
        if self._mode != Mode.AUTODOCKING:
            return
        twist = Twist()
        twist.angular.z = -0.5 * self._last_angle
        twist.linear.x = 0.1 + 0.2 * self._last_strength
        self.pub_cmd.publish(twist)

def main():
    rclpy.init()
    rclpy.spin(AutoDocking())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
