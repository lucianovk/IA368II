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
import threading
import time

import rclpy
from rclpy.lifecycle import Node as LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.qos import QoSProfile
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String
from .modes import Mode

class Charging(LifecycleNode):
    def __init__(self):
        super().__init__('charging_server')
        self._mode = Mode.IDLE
        self._batt = 1.0
        self._batt_sub = None
        self._mode_sub = None
        self._shutdown_event = threading.Event()
        self._shutdown_deadline = None
        self._shutdown_grace = 2.0

    def on_configure(self, _):
        qos =  QoSProfile(depth=1)
        self._batt_sub = self.create_subscription(BatteryState, '/myRobot/battery_state', self._on_batt, qos)
        self._mode_sub = self.create_subscription(String, '/myRobot/behaviour/mode', self._on_mode, qos)
        self.get_logger().info('charging configured')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, _):
        self.get_logger().info('charging activated')
        return TransitionCallbackReturn.SUCCESS

    def _destroy_resources(self):
        if self._batt_sub is not None:
            self.destroy_subscription(self._batt_sub)
            self._batt_sub = None
        if self._mode_sub is not None:
            self.destroy_subscription(self._mode_sub)
            self._mode_sub = None

    def on_deactivate(self, _):
        self.get_logger().info('charging deactivating (no timers to cancel)')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, _):
        self.get_logger().info('charging cleaning up resources')
        self._destroy_resources()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, _):
        self.get_logger().info('charging shutting down')
        self._destroy_resources()
        self._shutdown_event.set()
        self._shutdown_deadline = time.monotonic() + self._shutdown_grace
        return TransitionCallbackReturn.SUCCESS

    def should_exit(self):
        if not self._shutdown_event.is_set():
            return False
        if self._shutdown_deadline is None:
            return False
        return time.monotonic() >= self._shutdown_deadline

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
    node = Charging()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)
            if node.should_exit():
                break
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received, shutting down early')
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
