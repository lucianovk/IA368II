#!/usr/bin/env python3
"""
charged_server
--------------
- Lifecycle: configured/activated by Nav2 lifecycle manager.
- Operational: does nothing unless Mode == CHARGED.

Real logic ideas:
- Advertise "ready" status.
- Perform self-checks while docked.
"""
import rclpy
from rclpy.lifecycle import Node as LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
from std_msgs.msg import String
from .modes import Mode

class Charged(LifecycleNode):
    def __init__(self):
        super().__init__('charged_server')
        self._mode = Mode.IDLE

    def on_configure(self, _):
        self.create_subscription(String, '/myRobot/behaviour/mode', self._on_mode, 1)
        self.get_logger().info('charged configured')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, _):
        self.get_logger().info('charged activated (idle at dock)')
        return TransitionCallbackReturn.SUCCESS

    def _on_mode(self, msg: String):
        try:
            self._mode = Mode(msg.data)
        except Exception:
            self._mode = Mode.IDLE
        # Place background/maintenance tasks here if Mode == CHARGED

def main():
    rclpy.init()
    rclpy.spin(Charged())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
