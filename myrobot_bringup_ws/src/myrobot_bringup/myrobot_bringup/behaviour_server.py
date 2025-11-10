#!/usr/bin/env python3
"""
behaviour_server
----------------
Manages the robot's *operational state* (independent from lifecycle state).
Publishes the mode to /myRobot/behaviour/mode, which all nodes listen to.

This keeps Nav2 lifecycle_manager responsible ONLY for lifecycle config/activate,
while this node orchestrates *behavior* according to telemetry + user commands.
"""
import threading
import time

import rclpy
from rclpy.lifecycle import Node as LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.qos import QoSProfile
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32, String
from myrobot_interfaces.srv import StartStop
from .modes import Mode

BATTERY_LOW = 0.25

class BehaviourManager(LifecycleNode):
    def __init__(self):
        super().__init__('behaviour_server')
        self._battery = 1.0
        self._near_base = False
        self._stop_requested = False
        self._mode = Mode.CHARGED  # start docked & full
        self._mode_pub = None
        self._start_stop_srv = None
        self._tick_timer = None
        self._status_timer = None
        self._shutdown_event = threading.Event()
        self._shutdown_deadline = None

    def _shutdown_grace_period(self) -> float:
        # Give lifecycle manager time to query /get_state after shutdown transition
        return 2.0

    def _set_mode(self, new_mode: Mode, reason: str):
        """Update the behaviour mode and log the transition."""
        if new_mode == self._mode:
            return False
        old_mode = self._mode
        self._mode = new_mode
        self.get_logger().info(
            f'Behaviour mode {old_mode.value} -> {new_mode.value} ({reason})'
        )
        return True

    def _cancel_timers(self):
        for timer in (self._tick_timer, self._status_timer):
            if timer is not None and not timer.is_canceled():
                timer.cancel()

    def _destroy_timers(self):
        for name in ('_tick_timer', '_status_timer'):
            timer = getattr(self, name)
            if timer is not None:
                self.destroy_timer(timer)
                setattr(self, name, None)

    def _destroy_comms(self):
        if self._mode_pub is not None:
            self.destroy_publisher(self._mode_pub)
            self._mode_pub = None
        if self._start_stop_srv is not None:
            self.destroy_service(self._start_stop_srv)
            self._start_stop_srv = None

    def should_exit(self):
        if not self._shutdown_event.is_set():
            return False
        if self._shutdown_deadline is None:
            return False
        return time.monotonic() >= self._shutdown_deadline

    def on_configure(self, _):
        qos = QoSProfile(depth=1)
        # Inputs
        self.create_subscription(BatteryState, '/myRobot/battery_state', self._on_batt, qos)
        self.create_subscription(Float32, '/myRobot/charging_base/strengthSignal', self._on_strength, qos)
        # Output: operational mode
        self._mode_pub = self.create_publisher(String, '/myRobot/behaviour/mode', qos)
        # Service: absolute name so it's easy to call
        self._start_stop_srv = self.create_service(StartStop, '/myRobot/start_stop', self._on_start_stop)
        # Ticker that evaluates rules and publishes mode periodically
        self._tick_timer = self.create_timer(0.2, self._tick)
        # Slow ticker just to surface telemetry context in the logs
        self._status_timer = self.create_timer(5.0, self._log_battery_status)
        self.get_logger().info('behaviour configured')
        self.get_logger().info(
            'Initial behaviour state: mode=%s | battery=%.1f%% | near_base=%s | stop_requested=%s'
            % (
                self._mode.value,
                self._battery * 100.0,
                self._near_base,
                self._stop_requested,
            )
        )
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, _):
        for timer in (self._tick_timer, self._status_timer):
            if timer is not None and timer.is_canceled():
                timer.reset()
        self.get_logger().info('behaviour activated')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, _):
        self.get_logger().info('behaviour deactivating: cancelling timers')
        self._cancel_timers()
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, _):
        self.get_logger().info('behaviour cleaning up resources')
        self._cancel_timers()
        self._destroy_timers()
        self._destroy_comms()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, _):
        self.get_logger().info('behaviour shutting down')
        self._cancel_timers()
        self._destroy_timers()
        self._destroy_comms()
        self._shutdown_event.set()
        self._shutdown_deadline = time.monotonic() + self._shutdown_grace_period()
        return TransitionCallbackReturn.SUCCESS

    # -------- Inputs --------
    def _on_batt(self, msg: BatteryState):
        self._battery = float(msg.percentage)

    def _on_strength(self, msg: Float32):
        # "Near base" when signal strong enough; tweak threshold as needed
        self._near_base = msg.data > 0.8

    # -------- Service --------
    def _on_start_stop(self, req: StartStop.Request, resp: StartStop.Response):
        cmd = (req.command or '').strip().lower()
        if cmd not in ('start', 'stop'):
            resp.accepted = False
            resp.message = 'Use "start" or "stop".'
            return resp

        if cmd == 'start':
            self._stop_requested = False
            # If we are CHARGED, go to CLEANING; else keep current rules ticking
            self._set_mode(Mode.CLEANING, 'start command received')
        else:
            self._stop_requested = True
            if self._mode == Mode.CLEANING:
                self._set_mode(Mode.AUTODOCKING, 'stop command received')

        resp.accepted = True
        resp.message = f'Command {cmd} accepted.'
        return resp

    # -------- Behaviour Rules --------
    def _tick(self):
        # Apply transitions based on current telemetry
        if self._mode == Mode.CLEANING and self._battery <= BATTERY_LOW:
            self._set_mode(Mode.AUTODOCKING, f'battery low ({self._battery:.2f})')
        elif self._mode == Mode.AUTODOCKING and self._near_base:
            self._set_mode(Mode.CHARGING, 'charging base detected')
        elif self._mode == Mode.CHARGING and self._battery >= 0.999:
            next_mode = Mode.CHARGED if self._stop_requested else Mode.CLEANING
            reason = 'fully charged + stop requested' if self._stop_requested else 'fully charged'
            self._set_mode(next_mode, reason)

        # Publish the operational mode (idempotent)
        if self._mode_pub is not None:
            msg = String()
            msg.data = self._mode.value
            self._mode_pub.publish(msg)

    def _log_battery_status(self):
        self.get_logger().info(
            'Battery status: %.1f%% | mode=%s | near_base=%s | stop_requested=%s'
            % (
                self._battery * 100.0,
                self._mode.value,
                self._near_base,
                self._stop_requested,
            )
        )

def main():
    rclpy.init()
    node = BehaviourManager()
    executor = MultiThreadedExecutor()
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
