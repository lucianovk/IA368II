#!/usr/bin/env python3
import math
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import TransformBroadcaster

from coppeliasim_zmqremoteapi_client import RemoteAPIClient


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class CoppeliaOdometryPublisher(Node):
    def __init__(self):
        super().__init__('my_robot_odometry_node')

        self.pub = self.create_publisher(Odometry, '/myrobot/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.client = RemoteAPIClient()
        self.sim = self.client.require('sim')

        self.robot_handle = self.sim.getObject('/myRobot')

        self.last_pose: Optional[Tuple[float, float, float]] = None
        self.last_stamp: Optional[Time] = None

        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info('CoppeliaOdometryPublisher connected to CoppeliaSim.')

    def timer_callback(self):
        """Poll the simulator buffer, publish odometry, and broadcast the TF frame."""
        buf = self.sim.getBufferProperty(
            self.robot_handle,
            'customData.Odometry',
            {'noError': True},
        )
        if not buf:
            return

        data = self.sim.unpackTable(buf)
        if len(data) < 3:
            return

        x, y, theta = data[0], data[1], data[2]

        now = self.get_clock().now()

        msg = Odometry()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'my_robot_odom'
        msg.child_frame_id = 'base_footprint'

        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = yaw_to_quaternion(theta)

        if self.last_pose and self.last_stamp:
            dt = (now - self.last_stamp).nanoseconds * 1e-9
            if dt > 0.0:
                # Estimate linear/angular velocity from pose deltas
                dx = x - self.last_pose[0]
                dy = y - self.last_pose[1]
                dtheta = theta - self.last_pose[2]
                dtheta = math.atan2(math.sin(dtheta), math.cos(dtheta))

                msg.twist.twist.linear.x = dx / dt
                msg.twist.twist.linear.y = dy / dt
                msg.twist.twist.angular.z = dtheta / dt

        self.last_pose = (x, y, theta)
        self.last_stamp = now

        self.pub.publish(msg)

        # Broadcast odom -> base_footprint so the rest of the stack can consume TF
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'my_robot_odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = CoppeliaOdometryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
