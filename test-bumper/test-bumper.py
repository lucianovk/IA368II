import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Wrench



class SubscriberBumper(Node):
    def __init__(self):
        super().__init__('subscriber_bumper')
        # Listen to bumper wrench updates from the simulator.
        self.subscriber = self.create_subscription(Wrench, 'myRobot/bumper', self.callback, 10)
        self.forceVector = (0.0, 0.0, 0.0)
        self.torqueVector = (0.0, 0.0, 0.0)

    def callback(self, msg):
        # Cache the latest sensed force/torque so the velocity node can react.
        self.forceVector = msg.force.x, msg.force.y, msg.force.z
        self.torqueVector = msg.torque.x, msg.torque.y, msg.torque.z

class PublisherVelocity(Node):
    def __init__(self):
        super().__init__('publisher_velocity')
        # Publishes velocity commands to the robot base.
        self.publisher = self.create_publisher(Twist, 'myRobot/cmd_vel', 10)
        self.linVel = -0.2
        self.rotVel = 0.0
        self.state = 0
        # Thresholds and timers that drive the bumper recovery Finite State Machine.
        self.force_threshold = 1.0
        self.stop_time = 2.0  # seconds
        self.forward_time = 2.0  # seconds
        self.last_state_change_time = None
        self.forceVector = (0.0, 0.0, 0.0)
        self.last_collision_detected = None
        self.last_reported_state = None
    
    def move(self, forceVector=None):
        if forceVector is not None:
            self.forceVector = forceVector

        force_magnitude = max(abs(component) for component in self.forceVector)
        collision_detected = force_magnitude > self.force_threshold
        # Enter STOPPED state on collision while backing up.
        if self.state == 0 and collision_detected:
            self.linVel = 0.0
            self.rotVel = 0.0
            self.last_state_change_time = self.get_clock().now()
            self.state = 1
        # Hold still for a short cooldown window.
        if self.state == 1:
            elapsed_time = (self.get_clock().now() - self.last_state_change_time).nanoseconds / 1e9
            if elapsed_time < self.stop_time:
                self.linVel = 0.0
                self.rotVel = 0.0
            else:
                self.last_state_change_time = self.get_clock().now()
                self.state = 2
        # After the stop, drive forward for a bit before resuming the backward motion.
        if self.state == 2:
            elapsed_time = (self.get_clock().now() - self.last_state_change_time).nanoseconds / 1e9
            if elapsed_time < self.forward_time:
                self.linVel = 0.2  # Forward speed
                self.rotVel = 0.5  # No rotation
            else:
                self.last_state_change_time = self.get_clock().now()
                self.state = 0
                self.linVel = -0.2  # Bacward speed
                self.rotVel = 0.0  # No rotation
        if (collision_detected != self.last_collision_detected) or (self.state != self.last_reported_state):
            state_label = {0: 'BACKWARD', 1: 'STOPPED', 2: 'FORWARD'}.get(self.state, f'UNKNOWN({self.state})')
            print(f"State: {state_label}, Collision: {collision_detected}, Max force component: {force_magnitude:.4f}")
            self.last_collision_detected = collision_detected
            self.last_reported_state = self.state
        msg = Twist()
        msg.linear.x = self.linVel
        msg.angular.z = self.rotVel
        self.publisher.publish(msg)
        
    
def main(args=None):
    rclpy.init(args=args)

    subscriber_node = SubscriberBumper()
    publisher_node = PublisherVelocity()

    try:
        while rclpy.ok():
            rclpy.spin_once(subscriber_node, timeout_sec=0)
            publisher_node.move(subscriber_node.forceVector)
            rclpy.spin_once(publisher_node, timeout_sec=0)
    except KeyboardInterrupt:
        pass

    publisher_node.destroy_node()
    subscriber_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
