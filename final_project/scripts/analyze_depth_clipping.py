#!/usr/bin/env python3
"""
Lê um frame de /camera_depth e mostra estatísticas do depth bruto (sem clipping)
e do depth convertido com near/far do sensor (modelo perspectiva Coppelia).

Uso: python3 scripts/analyze_depth_clipping.py
Posicione o robô ~1 m da parede e compare valores.
"""

import sys
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from coppeliasim_zmqremoteapi_client import RemoteAPIClient


class DepthAnalyzer(Node):
    def __init__(self):
        super().__init__('depth_analyzer')
        self.sub = self.create_subscription(Image, '/camera_depth', self.cb, 1)
        self.got_frame = False

    def cb(self, msg: Image):
        if self.got_frame:
            return
        arr = np.frombuffer(msg.data, dtype=np.float32)
        if arr.size != msg.width * msg.height:
            self.get_logger().warn(
                f'Tamanho inesperado: {arr.size} vs {msg.width*msg.height}'
            )
            rclpy.shutdown()
            return
        arr = arr.reshape((msg.height, msg.width))
        u = msg.width // 2
        v = msg.height // 2

        # valores brutos
        print(f'Frame {msg.header.frame_id} {msg.width}x{msg.height}')
        print(f'Bruto depth: min={arr.min():.4f} max={arr.max():.4f} mean={arr.mean():.4f}')
        print(f'Bruto pixel centro [{v},{u}]: {arr[v,u]:.4f}')

        # Clipping do sensor via API
        try:
            client = RemoteAPIClient()
            sim = client.require('sim')
            vision = sim.getObject('/myRobot/visionSensor')
            near = sim.getObjectFloatParam(vision, sim.visionfloatparam_near_clipping)
            far = sim.getObjectFloatParam(vision, sim.visionfloatparam_far_clipping)
            depth_conv = (far * near) / (far - (far - near) * arr)
            print(
                f'Convertido (near={near:.3f}, far={far:.3f}): '
                f'min={depth_conv.min():.4f} max={depth_conv.max():.4f} '
                f'mean={depth_conv.mean():.4f}'
            )
            print(f'Convertido pixel centro: {depth_conv[v,u]:.4f} m')
        except Exception as exc:
            print(f'Falha ao obter near/far ou converter: {exc}')

        self.got_frame = True
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = DepthAnalyzer()
    try:
        while rclpy.ok() and not node.got_frame:
            rclpy.spin_once(node, timeout_sec=1.0)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
