#!/usr/bin/env python3
"""
charging_server
---------------
- Lifecycle: configured/activated by Nav2 lifecycle manager.
- Operational: monitors charging while Mode == CHARGING.

Real logic ideas:
- Control a relay or docking latch.
- Verify charge current/voltage via hardware interface.
"""
import rclpy
from rclpy.lifecycle import Node as LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.qos import QoSProfile
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String
from .modes import Mode

class Charging(LifecycleNode):
    def __init__(self):
        super().__init__('charging_server')
        self._mode = Mode.IDLE
        self._batt = 1.0

    def on_configure(self, _):
        qos =  QoSProfile(depth=1)
        self.create_subscription(BatteryState, '/myRobot/battery_state', self._on_batt, qos)
        self.create_subscription(String, '/myRobot/behaviour/mode', self._on_mode, qos)
        self.get_logger().info('charging configured')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, _):
        self.get_logger().info('charging activated')
        return TransitionCallbackReturn.SUCCESS

    def _on_mode(self, msg: String):
        try:
            self._mode = Mode(msg.data)
        except Exception:
            self._mode = Mode.IDLE

    def _on_batt(self, msg: BatteryState):
        self._batt = msg.percentage
        if self._mode == Mode.CHARGING:
            # In real robot, decide when to signal "full" or manage trickle charge.
            pass

def main():
    rclpy.init()
    rclpy.spin(Charging())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
