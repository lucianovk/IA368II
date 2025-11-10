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
import threading
import time

import rclpy
from rclpy.lifecycle import Node as LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.qos import QoSProfile
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from .modes import Mode

class Cleaning(LifecycleNode):
    def __init__(self):
        super().__init__('cleaning_server')
        self.pub_cmd = None
        self._mode_sub = None
        self._tick_timer = None
        self._mode = Mode.IDLE
        self._shutdown_event = threading.Event()
        self._shutdown_deadline = None
        self._shutdown_grace = 2.0

    def on_configure(self, _):
        qos = QoSProfile(depth=1)
        self.pub_cmd = self.create_publisher(Twist, '/myRobot/cmd_vel', qos)
        self._mode_sub = self.create_subscription(String, '/myRobot/behaviour/mode', self._on_mode, qos)
        self._tick_timer = self.create_timer(0.5, self._tick)
        self.get_logger().info('cleaning configured')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, _):
        if self._tick_timer is not None and self._tick_timer.is_canceled():
            self._tick_timer.reset()
        self.get_logger().info('cleaning activated')
        return TransitionCallbackReturn.SUCCESS

    def _cancel_timer(self):
        if self._tick_timer is not None and not self._tick_timer.is_canceled():
            self._tick_timer.cancel()

    def _destroy_resources(self):
        if self._tick_timer is not None:
            self.destroy_timer(self._tick_timer)
            self._tick_timer = None
        if self.pub_cmd is not None:
            self.destroy_publisher(self.pub_cmd)
            self.pub_cmd = None
        if self._mode_sub is not None:
            self.destroy_subscription(self._mode_sub)
            self._mode_sub = None

    def on_deactivate(self, _):
        self.get_logger().info('cleaning deactivating: cancelling timers')
        self._cancel_timer()
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, _):
        self.get_logger().info('cleaning cleaning up resources')
        self._cancel_timer()
        self._destroy_resources()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, _):
        self.get_logger().info('cleaning shutting down')
        self._cancel_timer()
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

    def _tick(self):
        if self._mode != Mode.CLEANING:
            return
        twist = Twist()
        twist.linear.x = 0.2
        twist.angular.z = 0.3
        self.pub_cmd.publish(twist)

def main():
    rclpy.init()
    node = Cleaning()
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
