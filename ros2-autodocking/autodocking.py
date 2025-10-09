"""ROS 2 node coordinating autonomous wandering and docking of the robot."""

import math
import random
from enum import Enum, auto
from typing import Optional

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

from geometry_msgs.msg import Twist, Wrench
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32, Int32


class Mode(Enum):
    AUTO = auto()
    DOCKING = auto()


class DockingPhase(Enum):
    SEARCH = auto()
    APPROACH = auto()
    WAIT_FOR_CHARGE = auto()


def map_range(x, in_min, in_max, out_min, out_max):
    """Clamp ``x`` into the input range and map it linearly to the output range."""
    if x is None:
        return out_min
    if x < in_min:
        x = in_min
    elif x > in_max:
        x = in_max
    if in_max == in_min:
        return out_min
    ratio = (x - in_min) / (in_max - in_min)
    return out_min + ratio * (out_max - out_min)

def wrap_to_pi(a):
    """Wrap a raw angle into the (-pi, pi] interval."""
    return (a + math.pi) % (2 * math.pi) - math.pi

def clamp(value: float, lower: float, upper: float) -> float:
    """Restrict ``value`` to stay between ``lower`` and ``upper``."""
    return min(max(value, lower), upper)


class AutoDockingNode(Node):
    def __init__(self) -> None:
        """Wire ROS interfaces and initialise docking state."""
        super().__init__("autodocking_node")

        self.cmd_vel_pub = self.create_publisher(Twist, "/myRobot/cmd_vel", 10)
        self.docking_state_pub = self.create_publisher(Int32, "/myRobot/docking_mode", 10)

        self.create_subscription(Float32, "/myRobot/charging_base/strengthSignal", self.strength_callback, 10)
        self.create_subscription(Float32, "/myRobot/charging_base/relativeAngle", self.angle_callback, 10)
        self.create_subscription(BatteryState, "/myRobot/battery_state", self.battery_callback, 10)
        self.create_subscription(Wrench, "/myRobot/bumper", self.bumper_callback, 10)

        self.control_timer = self.create_timer(0.1, self.control_loop)

        self.mode = Mode.AUTO
        self.previous_mode = None
        self.docking_phase = DockingPhase.SEARCH
        self.previous_docking_phase = None

        self.last_strength: Optional[float] = None
        self.last_angle: Optional[float] = None
        self.last_strength_stamp = None
        self.last_angle_stamp = None
        self.battery_percentage: Optional[float] = None
        self.battery_status: Optional[int] = None
        
        self.bump_threshold = 2.0
        self.last_bump_stamp = None
        self.bump_timeout = Duration(seconds=0.2)
        self.control_timeout = Duration(seconds=5)

        # Battery checkpoints drive the AUTO↔DOCKING decision.
        self.low_battery_threshold = 80.0
        self.full_battery_threshold = 100.0

        # Auto mode random walk configuration and cached twist.
        self.max_linear_vel = 0.5
        self.auto_current_twist = Twist()
        self.auto_current_twist.linear.x = self.max_linear_vel
        self.auto_current_twist.angular.z = 0.0
        now = self.get_clock().now()
        self.auto_next_update = now
        self.signal_timeout = Duration(seconds=1.5)
        # High signal strength is interpreted as the robot being docked.
        self.dock_strength_threshold = 0.95
        
        
        random.seed()
        self.get_logger().info("Auto docking node started in AUTO mode.")
        self._report_status_changes()

    def strength_callback(self, msg: Float32) -> None:
        """Cache latest charging beacon strength and timestamp."""
        self.last_strength = msg.data
        self.last_strength_stamp = self.get_clock().now()

    def angle_callback(self, msg: Float32) -> None:
        """Cache relative angle toward the docking beacon."""
        self.last_angle = msg.data
        self.last_angle_stamp = self.get_clock().now()

    def battery_callback(self, msg: BatteryState) -> None:
        """Switch to docking mode whenever the battery drops below threshold."""
        self.battery_percentage = msg.percentage
        self.battery_status = msg.power_supply_status

        if self.mode == Mode.AUTO and self.battery_percentage is not None:
            if self.battery_percentage <= self.low_battery_threshold:
                self.get_logger().info("Battery low, switching to DOCKING mode.")
                self.enter_docking()
    
    def bumper_callback(self, msg: Wrench) -> None:
        magnitude = math.sqrt(msg.force.x ** 2 + msg.force.y ** 2 + msg.force.z ** 2)
        if magnitude >= self.bump_threshold:
            self.last_bump_stamp = self.get_clock().now()
    
    def control_loop(self) -> None:
        """Publish the current docking state and drive the appropriate controller."""
        docking_state_msg = Int32()
        if self.mode == Mode.DOCKING:
            docking_state_msg.data = 1 
        else:
            docking_state_msg.data = 0 
        self.docking_state_pub.publish(docking_state_msg)

        twist = Twist()
        if self.mode == Mode.AUTO:
            twist = self.compute_auto_twist()
        else:
            twist = self.compute_docking_twist()

        self.cmd_vel_pub.publish(twist)
        self._report_status_changes()

    def compute_auto_twist(self) -> Twist:
        """Produce a wandering twist, backing up if the bumper just triggered."""
        now = self.get_clock().now()
        twist = Twist()
        if now >= self.auto_next_update:
            # Periodically refresh the wandering command so the robot keeps exploring.
            self.auto_next_update = now + self.control_timeout
            if self.bump_recent(self.last_bump_stamp):    
                self.get_logger().info(f"BUMPER: activated!")
                # Escape behaviour: random turning angle plus either reverse or stop.
                current_turn_angle_deg = random.randint(-90, 90)
                angularVel = clamp(math.radians(current_turn_angle_deg),-math.pi, math.pi)
                linearVel = random.choice([-self.max_linear_vel, 0.0])
            else:
                # Otherwise continue cruising forward at nominal velocity.
                linearVel = self.max_linear_vel
                current_turn_angle_deg = 0.0
                angularVel = math.radians(current_turn_angle_deg) 
            twist.linear.x = linearVel
            twist.angular.z = angularVel
            # Cache the freshly sampled command so we can reuse it until the next refresh.
            self.auto_current_twist = twist
            self.get_logger().info(f"linearVel:{linearVel} m/s - angularVel: {current_turn_angle_deg:.0f} °/s")
        else:
            # Timer not elapsed: reuse last command to avoid twitching.
            twist = self.auto_current_twist
        return twist

    def compute_docking_twist(self) -> Twist:
        """Run the docking finite-state machine."""
        twist = Twist()

        if self.docking_phase != DockingPhase.SEARCH and not self.signal_recent(self.last_strength_stamp):
            # Lose the beacon: reset to SEARCH so we can reacquire it.
            self.docking_phase = DockingPhase.SEARCH
        
        if self.docking_phase == DockingPhase.SEARCH:
            if self.signal_recent(self.last_strength_stamp) and self.last_strength and self.last_strength < self.dock_strength_threshold:
                # Beacon detected but not yet docked: transition to alignment phase.
                self.docking_phase = DockingPhase.APPROACH
                twist.linear.x = 0.0
                twist.angular.z = 0.0
            else:
                # Keep roaming while the beacon is missing or docked already.
                twist = self.compute_auto_twist()    
        elif self.docking_phase == DockingPhase.APPROACH:
                angle = float(self.last_angle)
                signal = float(self.last_strength)
                if self.signal_recent(self.last_angle_stamp) and angle:
                    angle = wrap_to_pi(angle)
                    if abs(angle) >= 0.5:
                        # Large heading error: pause forward motion until aligned.
                        linearVel = 0.0
                    else:
                        # Drive forward slowing as the signal strengthens (closer to dock).
                        linearVel = map_range(signal or 0.0, 0.0, 1.0, self.max_linear_vel,self.max_linear_vel/10.0)

                    # Angle feedback generates a proportional turn command capped at ±90°/s.
                    angularVel = map_range(angle, -math.pi, math.pi, -math.pi/2.0, math.pi/2.0)
                    self.get_logger().info(f"Docking: angle={angle:.1f} rad, angle={math.degrees(angle):.1f}°, strength={signal:.2f}, linearVel={linearVel:.2f} m/s angularVel={math.degrees(angularVel):.1f} °/s")
                    twist.linear.x = linearVel
                    twist.angular.z = angularVel
                    if self.last_strength and self.last_strength >= self.dock_strength_threshold:
                        # Strong signal implies contact with dock; wait for charging to complete.
                        self.docking_phase = DockingPhase.WAIT_FOR_CHARGE
                        twist.linear.x = 0.0
                        twist.angular.z = 0.0
        elif self.docking_phase == DockingPhase.WAIT_FOR_CHARGE:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            if self.last_strength and self.last_strength >= self.dock_strength_threshold:
                if self.battery_percentage is not None and self.battery_percentage >= self.full_battery_threshold:
                    self.get_logger().info("Charging complete, leaving docking mode.")
                    self.exit_docking()
            else:
                # If signal drops, slide back to SEARCH to reacquire the dock pose.
                self.docking_phase = DockingPhase.SEARCH
        return twist

    def signal_recent(self, stamp) -> bool:
        """Check whether the cached signal timestamp is still fresh."""
        if stamp is None:
            return False
        return (self.get_clock().now() - stamp) <= self.signal_timeout

    def bump_recent(self, stamp) -> bool:
        """Check whether a bumper event occurred within the debounce window."""
        if stamp is None:
            return False
        return (self.get_clock().now() - stamp) <= self.bump_timeout

    def enter_docking(self) -> None:
        """Reset docking state when switching from AUTO to DOCKING."""
        self.mode = Mode.DOCKING
        self.docking_phase = DockingPhase.SEARCH
        self.auto_current_twist = Twist()
        self.auto_current_twist.linear.x = self.max_linear_vel
        self.auto_current_twist.angular.z = 0.0
        self.auto_next_update = self.get_clock().now()

    def exit_docking(self) -> None:
        """Return to AUTO mode after the battery reaches the goal."""
        self.mode = Mode.AUTO
        self.docking_phase = DockingPhase.SEARCH
        self.auto_current_twist = Twist()
        self.auto_current_twist.linear.x = self.max_linear_vel
        self.auto_current_twist.angular.z = 0.0
        self.auto_next_update = self.get_clock().now()

    def _report_status_changes(self) -> None:
        """Log transitions for debugging purposes."""
        if self.mode != self.previous_mode:
            if self.previous_mode is None:
                self.get_logger().info(f"Initial mode: {self.mode.name}.")
            else:   
                self.get_logger().info(f"Mode changed: {self.previous_mode} -> {self.mode.name}.")
            self.previous_mode = self.mode

        if self.docking_phase != self.previous_docking_phase:
            if self.previous_docking_phase is None:
                self.get_logger().info(f"Initial docking phase: {self.docking_phase.name}.")
            else:
                self.get_logger().info(f"Docking phase changed: {self.previous_docking_phase.name} -> {self.docking_phase.name}.")
            self.previous_docking_phase = self.docking_phase


def main(args=None) -> None:
    """Launch the ROS 2 node and spin it until shutdown."""
    rclpy.init(args=args)
    node = AutoDockingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
