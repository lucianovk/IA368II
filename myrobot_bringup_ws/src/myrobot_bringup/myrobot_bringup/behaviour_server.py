#!/usr/bin/env python3
"""
behaviour_server
----------------
Manages the robot's *operational state* (independent from lifecycle state).
Publishes the mode to /myRobot/behaviour/mode, which all nodes listen to.

This keeps Nav2 lifecycle_manager responsible ONLY for lifecycle config/activate,
while this node orchestrates *behavior* according to telemetry + user commands.
"""
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
        qos = QoSProfile(depth=1)
        self._battery = 1.0
        self._near_base = False
        self._stop_requested = False
        self._mode = Mode.CHARGED  # start docked & full

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

    def on_configure(self, _):
        qos = QoSProfile(depth=1)
        # Inputs
        self.create_subscription(BatteryState, '/myRobot/battery_state', self._on_batt, qos)
        self.create_subscription(Float32, '/myRobot/charging_base/strengthSignal', self._on_strength, qos)
        # Output: operational mode
        self.pub_mode = self.create_publisher(String, '/myRobot/behaviour/mode', qos)
        # Service: absolute name so it's easy to call
        self.create_service(StartStop, '/myRobot/start_stop', self._on_start_stop)
        # Ticker that evaluates rules and publishes mode periodically
        self.create_timer(0.2, self._tick)
        # Slow ticker just to surface telemetry context in the logs
        self.create_timer(5.0, self._log_battery_status)
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
        self.get_logger().info('behaviour activated')
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
        msg = String()
        msg.data = self._mode.value
        self.pub_mode.publish(msg)

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
    rclpy.spin(BehaviourManager(), executor=MultiThreadedExecutor())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
