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
import threading
import time

import rclpy
from rclpy.lifecycle import Node as LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
from .modes import Mode

class Charged(LifecycleNode):
    def __init__(self):
        super().__init__('charged_server')
        self._mode = Mode.IDLE
        self._mode_sub = None
        self._shutdown_event = threading.Event()
        self._shutdown_deadline = None
        self._shutdown_grace = 2.0

    def on_configure(self, _):
        self._mode_sub = self.create_subscription(String, '/myRobot/behaviour/mode', self._on_mode, 1)
        self.get_logger().info('charged configured')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, _):
        self.get_logger().info('charged activated (idle at dock)')
        return TransitionCallbackReturn.SUCCESS

    def _destroy_resources(self):
        if self._mode_sub is not None:
            self.destroy_subscription(self._mode_sub)
            self._mode_sub = None

    def on_deactivate(self, _):
        self.get_logger().info('charged deactivating (no timers to cancel)')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, _):
        self.get_logger().info('charged cleaning up resources')
        self._destroy_resources()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, _):
        self.get_logger().info('charged shutting down')
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
        # Place background/maintenance tasks here if Mode == CHARGED

def main():
    rclpy.init()
    node = Charged()
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
