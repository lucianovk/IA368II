#!/usr/bin/env python3
"""
CoppeliaSim bridge that publishes RGB/depth streams and a camera TF so ROS nodes can
consume simulated vision data just like a physical sensor.
"""
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
        # Query clipping planes and field of view so we can convert depth to meters downstream
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
            self.get_logger().info(f'Sensor FOV: {self.fov_deg:.2f} degrees')
            self.get_logger().info(
                f'Sensor clipping: near={self.near_clip:.3f} m, far={self.far_clip:.3f} m'
            )
        except Exception as exc:
            self.get_logger().warn(f'Failed to query sensor clipping/FOV: {exc}')
            self.near_clip = 0.2
            self.far_clip = 3.5
            self.fov_deg = 57.0

        # Publish a static TF so downstream nodes know where the camera sits relative to base_link
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        static_t = TransformStamped()
        static_t.header.stamp = self.get_clock().now().to_msg()
        static_t.header.frame_id = 'base_link'
        static_t.child_frame_id = 'camera_link'
        static_t.transform.translation.x = 0.0
        static_t.transform.translation.y = 0.0
        static_t.transform.translation.z = 0.12
        # Orientation: aligned with the X axis, no tilt
        static_t.transform.rotation.x = 0.0
        static_t.transform.rotation.y = 0.70710678118
        static_t.transform.rotation.z = 0.0
        static_t.transform.rotation.w = 0.70710678118
        self.static_tf_broadcaster.sendTransform(static_t)

        # Run at 10 Hz; faster loops waste CPU without adding much value for this sim setup
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('CoppeliaVisionPublisher started.')
        self.first_frame_logged = False

    def timer_callback(self):
        try:
            # Ensure the sensor is processed before retrieving the image/depth
            self.sim.handleVisionSensor(self.vision_handle)
        except Exception as exc:
            self.get_logger().warn(f'Failed to process vision sensor: {exc}')
            return

        try:
            raw = self.sim.getVisionSensorCharImage(self.vision_handle)
        except Exception as exc:  # pragma: no cover - melhor logar que travar
            self.get_logger().warn(f'Failed to read vision sensor: {exc}')
            return

        # The CoppeliaSim Python API may return (img, resX, resY) or (res, img)
        image = None
        resolution = None
        if isinstance(raw, (list, tuple)):
            if len(raw) == 3:
                image, res_x, res_y = raw
                resolution = (res_x, res_y)
            elif len(raw) == 2:
                a, b = raw
                # attempt to infer which entry contains the resolution
                if isinstance(a, (list, tuple)) and len(a) == 2:
                    resolution, image = a, b
                elif isinstance(b, (list, tuple)) and len(b) == 2:
                    resolution, image = b, a
        if resolution is None or image is None:
            self.get_logger().warn(f'Unexpected vision sensor return: {type(raw)} {raw}')
            return

        if resolution is None or image is None:
            return

        width, height = int(resolution[0]), int(resolution[1])
        if width <= 0 or height <= 0:
            self.get_logger().warn(f'Invalid resolution received: {resolution}')
            return

        arr = np.frombuffer(image, dtype=np.uint8)
        expected = width * height * 3
        if arr.size != expected:
            self.get_logger().warn(
                f'Unexpected image size: {arr.size} vs expected {expected} '
                f'(resolution {width}x{height})'
            )
            return

        arr = arr.reshape((height, width, 3))

        # Flip horizontally so the virtual camera matches the robot's RViz view
        lfarr = np.fliplr(arr)
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
                f'First frame published: {width}x{height}, encoding rgb8, step {msg.step}'
            )
            self.first_frame_logged = True

        # Depth image
        try:
            depth_raw = self.sim.getVisionSensorDepth(self.vision_handle)
        except Exception as exc:  # pragma: no cover
            self.get_logger().warn(f'Failed to read depth: {exc}')
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
            self.get_logger().warn(f'Unexpected depth return: {type(depth_raw)} {depth_raw}')
            return

        d_width, d_height = int(depth_res[0]), int(depth_res[1])
        if d_width != width or d_height != height:
            self.get_logger().warn(
                f'Depth resolution differs from RGB: depth {d_width}x{d_height} vs rgb {width}x{height}'
            )

        # depth_image may be bytes or a float list
        if isinstance(depth_image, (bytes, bytearray)):
            depth_arr = np.frombuffer(depth_image, dtype=np.float32)
        else:
            depth_arr = np.array(depth_image, dtype=np.float32)
        expected_depth = d_width * d_height
        if depth_arr.size != expected_depth:
            self.get_logger().warn(
                f'Unexpected depth size: {depth_arr.size} vs expected {expected_depth} '
                f'(resolution {d_width}x{d_height})'
            )
            return

        depth_arr = depth_arr.reshape((d_height, d_width))

        # Convert normalized depth (0..1) to meters using the sensor near/far planes
        try:
            near = self.near_clip
            far = self.far_clip
            depth_arr = near + depth_arr * (far - near)
        except Exception as exc:
            self.get_logger().warn(f'Failed to convert depth to meters: {exc}')

        # Apply the same horizontal flip as the RGB image
        depth_arr = np.fliplr(depth_arr)

        depth_msg = Image()
        depth_msg.header.stamp = msg.header.stamp  # reuse the RGB timestamp
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
