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
import threading
import time

import rclpy
from rclpy.lifecycle import Node as LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.qos import QoSProfile
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Float32, String
from geometry_msgs.msg import Twist
from .modes import Mode

class AutoDocking(LifecycleNode):
    def __init__(self):
        super().__init__('autodocking_server')
        self._mode = Mode.IDLE
        self._last_strength = 0.0
        self._last_angle = 0.0
        self._cmd_pub = None
        self._strength_sub = None
        self._angle_sub = None
        self._mode_sub = None
        self._tick_timer = None
        self._shutdown_event = threading.Event()
        self._shutdown_deadline = None
        self._shutdown_grace = 2.0

    def on_configure(self, _):
        qos = QoSProfile(depth=1)
        self._cmd_pub = self.create_publisher(Twist, '/myRobot/cmd_vel', qos)
        self._strength_sub = self.create_subscription(Float32, '/myRobot/charging_base/strengthSignal', self._on_strength, qos)
        self._angle_sub = self.create_subscription(Float32, '/myRobot/charging_base/relativeAngle', self._on_angle, qos)
        self._mode_sub = self.create_subscription(String, '/myRobot/behaviour/mode', self._on_mode, qos)
        self._tick_timer = self.create_timer(0.2, self._tick)
        self.get_logger().info('autodocking configured')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, _):
        if self._tick_timer is not None and self._tick_timer.is_canceled():
            self._tick_timer.reset()
        self.get_logger().info('autodocking activated')
        return TransitionCallbackReturn.SUCCESS

    def _cancel_timer(self):
        if self._tick_timer is not None and not self._tick_timer.is_canceled():
            self._tick_timer.cancel()

    def _destroy_resources(self):
        if self._tick_timer is not None:
            self.destroy_timer(self._tick_timer)
            self._tick_timer = None
        if self._cmd_pub is not None:
            self.destroy_publisher(self._cmd_pub)
            self._cmd_pub = None
        for attr in ('_strength_sub', '_angle_sub', '_mode_sub'):
            sub = getattr(self, attr)
            if sub is not None:
                self.destroy_subscription(sub)
                setattr(self, attr, None)

    def on_deactivate(self, _):
        self.get_logger().info('autodocking deactivating: cancelling timers')
        self._cancel_timer()
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, _):
        self.get_logger().info('autodocking cleaning up resources')
        self._cancel_timer()
        self._destroy_resources()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, _):
        self.get_logger().info('autodocking shutting down')
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
        if self._cmd_pub is not None:
            self._cmd_pub.publish(twist)

def main():
    rclpy.init()
    node = AutoDocking()
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
