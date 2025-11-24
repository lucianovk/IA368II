#!/usr/bin/env python3
"""
Simple lifecycle helper that starts/stops the CoppeliaSim simulation for this project.
"""

import atexit

import rclpy
from rclpy.node import Node

from coppeliasim_zmqremoteapi_client import RemoteAPIClient


class CoppeliaSimControlNode(Node):
    def __init__(self):
        super().__init__('my_robot_sim_control_node')

        # Lazily connect to the simulator once and keep the handle around.
        self.client = RemoteAPIClient()
        self.sim = self.client.require('sim')

        # Kick off the simulation immediately—other nodes assume the world is running.
        self.sim.startSimulation()
        self.get_logger().info('CoppeliaSim simulation started.')

        atexit.register(self._stop_simulation)

    def destroy_node(self):
        self._stop_simulation()
        super().destroy_node()

    def _stop_simulation(self):
        """Stop the simulator gracefully so the next launch starts cleanly."""
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
