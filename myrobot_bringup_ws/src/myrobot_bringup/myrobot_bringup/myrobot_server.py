#!/usr/bin/env python3
"""
myrobot_server
--------------
Represents the robot platform. It:
- Publishes:
    /myRobot/charging_base/strengthSignal (std_msgs/Float32)
    /myRobot/charging_base/relativeAngle  (std_msgs/Float32)
    /myRobot/battery_state                (sensor_msgs/BatteryState)
- Subscribes:
    /myRobot/cmd_vel (geometry_msgs/Twist)
- Listens to the behaviour server's operational mode:
    /myRobot/behaviour/mode (std_msgs/String)

Lifecycle vs Operational State
------------------------------
* Lifecycle (managed by Nav2 lifecycle_manager): configure -> activate.
* Operational (managed by behaviour_server): CLEANING / AUTODOCKING / CHARGING / CHARGED.
  We don't activate/deactivate the lifecycle when behavior changes; we *act* differently
  depending on the current operational mode.

Where to add real logic
-----------------------
- Replace the random docking signal with your perception/IR beacons.
- Replace the battery model with your power estimator.
- Apply command velocity bounds/motion interface to your base.
"""
import math, random
import rclpy
from rclpy.lifecycle import Node as LifecycleNode
from rclpy.lifecycle import State, TransitionCallbackReturn
from rclpy.qos import QoSProfile
from std_msgs.msg import Float32, String
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import Twist
from .modes import Mode

class MyRobot(LifecycleNode):
    def __init__(self):
        super().__init__('myrobot')
        self._battery = 1.0
        self._near_base = False
        self._last_cmd = Twist()
        self._mode = Mode.IDLE
        self._timer = None

    # ---------------- Lifecycle Callbacks ----------------
    def on_configure(self, state: State):
        self.get_logger().info('myrobot configured')
        qos = QoSProfile(depth=1)
        self.pub_strength = self.create_publisher(Float32, '/myRobot/charging_base/strengthSignal', qos)
        self.pub_angle = self.create_publisher(Float32, '/myRobot/charging_base/relativeAngle', qos)
        self.pub_batt = self.create_publisher(BatteryState, '/myRobot/battery_state', qos)
        self.sub_cmd = self.create_subscription(Twist, '/myRobot/cmd_vel', self._on_cmd, qos)
        # Operational mode subscription (2nd state)
        self.sub_mode = self.create_subscription(String, '/myRobot/behaviour/mode', self._on_mode, qos)
        # Sim loop
        self._timer = self.create_timer(0.2, self._on_timer)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State):
        self.get_logger().info('myrobot activated')
        return TransitionCallbackReturn.SUCCESS

    # ---------------- Operational Logic ----------------
    def _on_mode(self, msg: String):
        try:
            self._mode = Mode(msg.data)
        except Exception:
            self._mode = Mode.IDLE

    def _on_cmd(self, msg: Twist):
        self._last_cmd = msg  # In real robot, forward to motor controllers

    def _on_timer(self):
        # Publish synthetic docking signal
        strength = Float32()
        angle = Float32()
        near = False

        if self._mode in (Mode.AUTODOCKING, Mode.CHARGING, Mode.CHARGED):
            near = random.random() > 0.7  # pretend we sometimes see the dock strong
            strength.data = 0.5 + (0.5 if near else 0.0)
            angle.data = 0.0 if near else random.uniform(-math.pi/4, math.pi/4)
        else:
            strength.data = random.uniform(0.0, 0.4)
            angle.data = random.uniform(-math.pi, math.pi)

        self._near_base = near
        self.pub_strength.publish(strength)
        self.pub_angle.publish(angle)

        # Battery model driven by operational mode
        if self._mode == Mode.CLEANING:
            self._battery = max(0.0, self._battery - 0.002)  # drains slowly
        elif self._mode == Mode.CHARGING:
            self._battery = min(1.0, self._battery + 0.004)  # charges faster

        bs = BatteryState()
        bs.percentage = float(self._battery)
        bs.power_supply_status = (
            BatteryState.POWER_SUPPLY_STATUS_CHARGING
            if self._mode == Mode.CHARGING
            else BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        )
        self.pub_batt.publish(bs)

def main():
    rclpy.init()
    node = MyRobot()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
