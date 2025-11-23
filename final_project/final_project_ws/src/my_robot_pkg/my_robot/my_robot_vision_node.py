#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.exceptions import ParameterAlreadyDeclaredException
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped

from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from tf2_ros import StaticTransformBroadcaster


class CoppeliaVisionPublisher(Node):
    def __init__(self):
        super().__init__('my_robot_vision_node')

        try:
            self.declare_parameter('use_sim_time', False)
        except ParameterAlreadyDeclaredException:
            pass

        self.pub = self.create_publisher(Image, '/camera', 10)
        self.depth_pub = self.create_publisher(Image, '/camera_depth', 10)

        self.client = RemoteAPIClient()
        self.sim = self.client.require('sim')
        self.vision_handle = self.sim.getObject('/myRobot/visionSensor')
        # Planos de corte obtidos diretamente do sensor
        try:
            self.near_clip = self.sim.getObjectFloatParam(
                self.vision_handle, self.sim.visionfloatparam_near_clipping
            )
            self.far_clip = self.sim.getObjectFloatParam(
                self.vision_handle, self.sim.visionfloatparam_far_clipping
            )
            fov_rad = self.sim.getObjectFloatParam(
                self.vision_handle, self.sim.visionfloatparam_perspective_angle
            )
            self.fov_deg = math.degrees(fov_rad)
            self.get_logger().info(f'FOV do sensor: {self.fov_deg:.2f} graus')
            self.get_logger().info(
                f'Clipping do sensor: near={self.near_clip:.3f} m, far={self.far_clip:.3f} m'
            )
        except Exception as exc:
            self.get_logger().warn(f'Falha ao obter clipping/FOV do sensor: {exc}')
            self.near_clip = 0.2
            self.far_clip = 3.5
            self.fov_deg = 57.0

        # Static TF camera_link 0.12 m acima de base_link
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        static_t = TransformStamped()
        static_t.header.stamp = self.get_clock().now().to_msg()
        static_t.header.frame_id = 'base_link'
        static_t.child_frame_id = 'camera_link'
        static_t.transform.translation.x = 0.0
        static_t.transform.translation.y = 0.0
        static_t.transform.translation.z = 0.12
        # Orientação: alinhada com eixo X, sem inclinação
        static_t.transform.rotation.x = 0.0
        static_t.transform.rotation.y = 0.70710678118
        static_t.transform.rotation.z = 0.0
        static_t.transform.rotation.w = 0.70710678118
        self.static_tf_broadcaster.sendTransform(static_t)

        # 10 Hz
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('CoppeliaVisionPublisher iniciado.')
        self.first_frame_logged = False

    def timer_callback(self):
        try:
            # Garante que o sensor é processado antes de ler imagem/depth
            self.sim.handleVisionSensor(self.vision_handle)
        except Exception as exc:
            self.get_logger().warn(f'Falha ao processar vision sensor: {exc}')
            return

        try:
            raw = self.sim.getVisionSensorCharImage(self.vision_handle)
        except Exception as exc:  # pragma: no cover - melhor logar que travar
            self.get_logger().warn(f'Falha ao ler visão: {exc}')
            return

        # CoppeliaSim Python API pode retornar (img, resX, resY) ou (res, img)
        image = None
        resolution = None
        if isinstance(raw, (list, tuple)):
            if len(raw) == 3:
                image, res_x, res_y = raw
                resolution = (res_x, res_y)
            elif len(raw) == 2:
                a, b = raw
                # tenta inferir qual é a resolução
                if isinstance(a, (list, tuple)) and len(a) == 2:
                    resolution, image = a, b
                elif isinstance(b, (list, tuple)) and len(b) == 2:
                    resolution, image = b, a
        if resolution is None or image is None:
            self.get_logger().warn(f'Retorno inesperado do sensor de visao: {type(raw)} {raw}')
            return

        if resolution is None or image is None:
            return

        width, height = int(resolution[0]), int(resolution[1])
        if width <= 0 or height <= 0:
            self.get_logger().warn(f'Resolucao invalida recebida: {resolution}')
            return

        arr = np.frombuffer(image, dtype=np.uint8)
        expected = width * height * 3
        if arr.size != expected:
            self.get_logger().warn(
                f'Tamanho inesperado da imagem: {arr.size} vs esperado {expected} '
                f'(resolucao {width}x{height})'
            )
            return

        arr = arr.reshape((height, width, 3))

        # Ajuste de orientação: se visualizar de cabeça para baixo, troque para np.flipud(arr)
        # ou rotacione conforme necessário. Aqui mantemos sem flips.

        lfarr=np.fliplr(arr)
        arr_uint8 = lfarr.copy()

        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_link'
        msg.height = height
        msg.width = width
        msg.encoding = 'rgb8'
        msg.is_bigendian = 0
        msg.step = width * 3
        msg.data = arr_uint8.tobytes()

        self.pub.publish(msg)

        if not self.first_frame_logged:
            self.get_logger().info(
                f'Primeiro frame publicado: {width}x{height}, encoding rgb8, step {msg.step}'
            )
            self.first_frame_logged = True

        # Depth image
        try:
            depth_raw = self.sim.getVisionSensorDepth(self.vision_handle)
        except Exception as exc:  # pragma: no cover
            self.get_logger().warn(f'Falha ao ler depth: {exc}')
            return

        depth_image = None
        depth_res = None
        if isinstance(depth_raw, (list, tuple)):
            if len(depth_raw) == 3:
                depth_image, res_x, res_y = depth_raw
                depth_res = (res_x, res_y)
            elif len(depth_raw) == 2:
                a, b = depth_raw
                if isinstance(a, (list, tuple)) and len(a) == 2:
                    depth_res, depth_image = a, b
                elif isinstance(b, (list, tuple)) and len(b) == 2:
                    depth_res, depth_image = b, a

        if depth_res is None or depth_image is None:
            self.get_logger().warn(f'Retorno inesperado do depth: {type(depth_raw)} {depth_raw}')
            return

        d_width, d_height = int(depth_res[0]), int(depth_res[1])
        if d_width != width or d_height != height:
            self.get_logger().warn(
                f'Resolucao depth difere da RGB: depth {d_width}x{d_height} vs rgb {width}x{height}'
            )

        # depth_image pode vir como bytes ou lista de floats
        if isinstance(depth_image, (bytes, bytearray)):
            depth_arr = np.frombuffer(depth_image, dtype=np.float32)
        else:
            depth_arr = np.array(depth_image, dtype=np.float32)
        expected_depth = d_width * d_height
        if depth_arr.size != expected_depth:
            self.get_logger().warn(
                f'Tamanho depth inesperado: {depth_arr.size} vs esperado {expected_depth} '
                f'(resolucao {d_width}x{d_height})'
            )
            return

        depth_arr = depth_arr.reshape((d_height, d_width))

        # Converte depth normalizado (0..1) para metros usando near/far do sensor
        try:
            near = self.near_clip
            far = self.far_clip
            depth_arr = near + depth_arr * (far - near)
        except Exception as exc:
            self.get_logger().warn(f'Falha ao converter depth para metros: {exc}')

        # Garante mesma orientação horizontal aplicada na imagem RGB
        depth_arr = np.fliplr(depth_arr)

        depth_msg = Image()
        depth_msg.header.stamp = msg.header.stamp  # usar mesmo timestamp da RGB
        depth_msg.header.frame_id = 'camera_link'
        depth_msg.height = d_height
        depth_msg.width = d_width
        depth_msg.encoding = '32FC1'
        depth_msg.is_bigendian = 0
        depth_msg.step = d_width * 4
        depth_msg.data = depth_arr.tobytes()

        self.depth_pub.publish(depth_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CoppeliaVisionPublisher()
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
