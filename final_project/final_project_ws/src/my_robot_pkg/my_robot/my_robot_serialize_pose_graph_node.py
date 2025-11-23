#!/usr/bin/env python3
"""
Nó que chama periodicamente o serviço slam_toolbox/SerializePoseGraph
para salvar o grafo de poses em disco.
"""

from pathlib import Path
from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.task import Future
from slam_toolbox.srv import SerializePoseGraph


class PoseGraphSerializerNode(Node):
    def __init__(self) -> None:
        super().__init__('my_robot_serialize_pose_graph_node')
        self.declare_parameter('service_name', '/slam_toolbox/serialize_map')
        self.declare_parameter('output_directory', 'pose_graphs')
        self.declare_parameter('interval_seconds', 60.0)

        self.service_name = self.get_parameter('service_name').value
        output_dir_param = self.get_parameter('output_directory').value
        self.output_dir = Path(output_dir_param).expanduser()
        if not self.output_dir.is_absolute():
            package_root = self.resolve_package_root()
            self.output_dir = (package_root / self.output_dir).resolve()
        self.output_dir.mkdir(parents=True, exist_ok=True)

        self.interval = float(self.get_parameter('interval_seconds').value)
        if self.interval <= 0:
            self.interval = 60.0

        self.client = self.create_client(SerializePoseGraph, self.service_name)
        self.pending_future: Optional[Future] = None
        self.timer = self.create_timer(self.interval, self.trigger_serialization)
        self.initial_timer = self.create_timer(1.0, self._initial_serialization_check)

        self.get_logger().info(
            f'Serialização periódica habilitada. Serviço={self.service_name} '
            f'saída={self.output_dir} intervalo={self.interval:.1f}s'
        )

    def trigger_serialization(self) -> None:
        if self.pending_future is not None and not self.pending_future.done():
            return
        if not self.client.wait_for_service(timeout_sec=0.0):
            self.get_logger().warn('Serviço SerializePoseGraph indisponível.')
            return
        filename = self.output_dir / 'my_robot_pose_graph'
        request = SerializePoseGraph.Request()
        request.filename = str(filename)
        self.get_logger().info(f'Serializando pose graph em {request.filename}')
        self.pending_future = self.client.call_async(request)
        self.pending_future.add_done_callback(self._handle_response)

    def _handle_response(self, future: Future) -> None:
        try:
            response = future.result()
        except Exception as exc:  # pragma: no cover
            self.get_logger().error(f'Erro ao serializar grafo de poses: {exc}')
            self.pending_future = None
            return

        result_code = getattr(response, 'result', None)
        if result_code == 0:
            self.get_logger().info('SerializePoseGraph concluído com sucesso.')
        else:
            self.get_logger().warn(f'SerializePoseGraph retornou código {result_code}')
        self.pending_future = None

    def _initial_serialization_check(self) -> None:
        if self.pending_future is None:
            if self.client.wait_for_service(timeout_sec=0.0):
                self.trigger_serialization()
                self.initial_timer.cancel()

    def resolve_package_root(self) -> Path:
        current = Path(__file__).resolve()
        for ancestor in current.parents:
            direct = ancestor / 'src' / 'my_robot_pkg'
            if direct.exists():
                return direct
            workspace = ancestor / 'final_project_ws' / 'src' / 'my_robot_pkg'
            if workspace.exists():
                return workspace
        try:
            from ament_index_python.packages import get_package_share_directory

            return Path(get_package_share_directory('my_robot_pkg'))
        except Exception:
            return current.parent


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PoseGraphSerializerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
