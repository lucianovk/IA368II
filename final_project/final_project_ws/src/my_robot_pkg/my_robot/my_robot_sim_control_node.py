#!/usr/bin/env python3
import atexit

import rclpy
from rclpy.node import Node

from coppeliasim_zmqremoteapi_client import RemoteAPIClient


class CoppeliaSimControlNode(Node):
    def __init__(self):
        super().__init__('my_robot_sim_control_node')

        self.client = RemoteAPIClient()
        self.sim = self.client.require('sim')

        self.sim.startSimulation()
        self.get_logger().info('CoppeliaSim simulation started.')

        atexit.register(self._stop_simulation)

    def destroy_node(self):
        self._stop_simulation()
        super().destroy_node()

    def _stop_simulation(self):
        state = self.sim.getSimulationState()
        if state != self.sim.simulation_stopped:
            self.get_logger().info('Stopping CoppeliaSim simulation...')
            self.sim.stopSimulation()


def main(args=None):
    rclpy.init(args=args)
    node = CoppeliaSimControlNode()
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
