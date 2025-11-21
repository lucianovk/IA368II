#!/usr/bin/env python3
import math

import rclpy
from rclpy.exceptions import ParameterAlreadyDeclaredException
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan

from coppeliasim_zmqremoteapi_client import RemoteAPIClient


class CoppeliaLaserPublisher(Node):
    def __init__(self):
        super().__init__('coppelia_laser_publisher')

        # use_sim_time parameter is set via launch; declare it so /clock is honored.
        try:
            self.declare_parameter('use_sim_time', False)
        except ParameterAlreadyDeclaredException:
            pass

        # Publicador LaserScan
        self.pub = self.create_publisher(LaserScan, '/scan', 10)

        # Conexão com CoppeliaSim
        self.client = RemoteAPIClient()  # host=localhost port=23000
        self.sim = self.client.require('sim')

        # Parâmetros do scan
        self.angle_min = -math.pi
        self.angle_max = math.pi
        self.range_min = 0.05
        self.range_max = 10.0

        # Timer para atualizar LaserScan
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info('CoppeliaLaserPublisher iniciado.')

    def timer_callback(self):
        buf = self.sim.getBufferProperty(
            self.sim.handle_scene,
            'signal.myRobot_scan',
            {'noError': True},
        )
        # getBufferProperty retorna bytes; se não existir, costuma vir b'' ou None

        if not buf:
            return

        ranges = self.sim.unpackFloatTable(buf)

        n = len(ranges)
        if n < 2:
            return

        now = self.get_clock().now()

        msg = LaserScan()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'laser_link'

        msg.angle_min = self.angle_min
        msg.angle_max = self.angle_max
        msg.angle_increment = (self.angle_max - self.angle_min) / (n - 1)

        msg.time_increment = 0.0
        msg.scan_time = 0.0

        msg.range_min = self.range_min
        msg.range_max = self.range_max
        msg.ranges = ranges

        self.pub.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = CoppeliaLaserPublisher()
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
