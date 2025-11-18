#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from coppeliasim_zmqremoteapi_client import RemoteAPIClient


class CoppeliaCmdVelBridge(Node):
    def __init__(self):
        super().__init__('my_robot_cmd_vel_node')

        self.declare_parameter('wheel_radius', 0.015)
        self.declare_parameter('wheel_separation', 0.2)

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_separation = self.get_parameter('wheel_separation').value

        self.client = RemoteAPIClient()
        self.sim = self.client.require('sim')
        self.left_motor = self.sim.getObject('/myRobot/leftMotor')
        self.right_motor = self.sim.getObject('/myRobot/rightMotor')

        self.sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10,
        )

        self.get_logger().info(
            'CmdVel bridge ready (wheel_radius=%.3f, wheel_separation=%.3f)'
            % (self.wheel_radius, self.wheel_separation)
        )

    def cmd_vel_callback(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z

        v_right = v + (self.wheel_separation * 0.5) * w
        v_left = v - (self.wheel_separation * 0.5) * w

        right_joint_vel = -v_right / self.wheel_radius
        left_joint_vel = -v_left / self.wheel_radius

        self.sim.setJointTargetVelocity(self.right_motor, right_joint_vel)
        self.sim.setJointTargetVelocity(self.left_motor, left_joint_vel)

        self.get_logger().debug(
            f'cmd_vel: v={v:.3f} w={w:.3f} -> left={v_left:.3f} right={v_right:.3f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = CoppeliaCmdVelBridge()
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
