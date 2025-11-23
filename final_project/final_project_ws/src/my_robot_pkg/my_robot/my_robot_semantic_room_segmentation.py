#!/usr/bin/env python3
"""
Converte detecções de objetos em segmentos semânticos de salas.
Lê /detections_map (MarkerArray), associa labels aos cômodos definidos
em semantic_room_seg_classes.json e colore /room_map com o cômodo estimado,
publicando o resultado como OccupancyGrid em /semantic_map.
"""

from pathlib import Path
from typing import Dict, Optional

import json
import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


class SemanticRoomSegmentationNode(Node):
    def __init__(self) -> None:
        super().__init__('my_robot_semantic_room_segmentation')

        self.declare_parameter(
            'semantic_map_file',
            str(
                Path(__file__)
                .resolve()
                .parent.parent
                / 'config'
                / 'semantic_room_seg_classes.json'
            ),
        )
        self.declare_parameter('room_map_topic', '/room_map')
        self.declare_parameter('detections_topic', '/detections_map')
        self.declare_parameter('semantic_map_topic', '/semantic_room_map')

        semantic_map_path = Path(self.get_parameter('semantic_map_file').value)
        if not semantic_map_path.exists():
            fallback = self._resolve_semantic_file()
            if fallback is not None:
                self.get_logger().warn(
                    f'{semantic_map_path} não encontrado; usando {fallback} como fallback.'
                )
                semantic_map_path = fallback
            else:
                self.get_logger().warn(
                    f'{semantic_map_path} não encontrado e nenhum fallback disponível.'
                )
        self.label_to_room = self._load_semantic_classes(semantic_map_path)
        self.get_logger().info(
            f'{len(self.label_to_room)} labels semânticos carregados de {semantic_map_path}'
        )

        self.room_map_topic = self.get_parameter('room_map_topic').value
        self.detections_topic = self.get_parameter('detections_topic').value
        self.semantic_topic = self.get_parameter('semantic_map_topic').value

        self.room_map_msg: Optional[OccupancyGrid] = None
        self.room_map_np: Optional[np.ndarray] = None
        self.room_value_map: Dict[str, int] = {}
        self.latest_assignments: Dict[int, str] = {}
        self.semantic_dirty = False

        self.room_sub = self.create_subscription(
            OccupancyGrid, self.room_map_topic, self._room_map_cb, 10
        )
        self.detections_sub = self.create_subscription(
            MarkerArray, self.detections_topic, self._detections_cb, 10
        )
        self.semantic_pub = self.create_publisher(OccupancyGrid, self.semantic_topic, 10)
        self.publish_timer = self.create_timer(10.0, self._publish_semantic_map)
        self.labels_pub = self.create_publisher(MarkerArray, '/semantic_room_labels', 10)

    def _load_semantic_classes(self, path: Path) -> Dict[str, str]:
        if not path.exists():
            self.get_logger().warn(f'Arquivo {path} não encontrado; mapa semântico vazio.')
            return {}
        try:
            with path.open('r', encoding='utf-8') as fp:
                data = json.load(fp)
        except Exception as exc:
            self.get_logger().error(f'Falha ao carregar {path}: {exc}')
            return {}
        mapping = {k.strip().lower(): v for k, v in data.items()}
        return mapping

    def _room_map_cb(self, msg: OccupancyGrid) -> None:
        self.room_map_msg = msg
        np_data = np.array(msg.data, dtype=np.int16).reshape(
            (msg.info.height, msg.info.width)
        )
        self.room_map_np = np_data
        self.get_logger().info(
            f'/room_map atualizado ({msg.info.width}x{msg.info.height}). Recalculando mapa semântico.'
        )
        self.semantic_dirty = True

    def _detections_cb(self, msg: MarkerArray) -> None:
        if self.room_map_msg is None or self.room_map_np is None:
            return
        updated = False
        for marker in msg.markers:
            if marker.ns != 'detections_text' or not marker.text:
                continue
            label = marker.text.split('(')[0].strip().lower()
            if not label:
                continue
            room_name = self.label_to_room.get(label)
            if room_name is None:
                continue
            room_id = self._lookup_room_id(marker.pose)
            if room_id is None or room_id <= 0:
                continue
            if (
                room_id not in self.latest_assignments
                or self.latest_assignments[room_id] != room_name
            ):
                self.latest_assignments[room_id] = room_name
                self.get_logger().info(
                    f'Deteccao atribuída: room_id={room_id} -> {room_name}'
                )
                updated = True

        if updated:
            self.semantic_dirty = True

    def _publish_semantic_map(self) -> None:
        if not self.semantic_dirty:
            return
        if (
            self.room_map_msg is None
            or self.room_map_np is None
            or not self.latest_assignments
        ):
            return

        semantic_grid = np.full_like(self.room_map_np, fill_value=-1, dtype=np.int16)
        room_name_to_value: Dict[str, int] = {}
        room_name_to_ids: Dict[str, list[int]] = {}
        for room_id, room_name in self.latest_assignments.items():
            mask = self.room_map_np == room_id
            if not np.any(mask):
                continue
            if room_name not in room_name_to_value:
                room_name_to_value[room_name] = self._semantic_value(room_name)
                room_name_to_ids[room_name] = []
            room_name_to_ids[room_name].append(room_id)
            semantic_value = room_name_to_value[room_name]
            semantic_grid[mask] = semantic_value

        semantic_grid[self.room_map_np == 100] = 100
        semantic_grid[(self.room_map_np == 0) & (semantic_grid == -1)] = 0

        out_msg = OccupancyGrid()
        out_msg.header = self.room_map_msg.header
        out_msg.info = self.room_map_msg.info
        out_msg.data = semantic_grid.flatten().tolist()
        self.semantic_pub.publish(out_msg)
        self._publish_labels(room_name_to_value, room_name_to_ids)
        self.get_logger().info(
            f'/semantic_room_map publicado com {len(room_name_to_value)} classes.'
        )
        self.semantic_dirty = False

    def _publish_labels(
        self, room_values: Dict[str, int], room_ids: Dict[str, list[int]]
    ) -> None:
        ma = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        for room_name, value in room_values.items():
            ids = room_ids.get(room_name, [])
            if not ids:
                continue
            total_mask = np.zeros_like(self.room_map_np, dtype=bool)
            for rid in ids:
                total_mask |= self.room_map_np == rid
            if not np.any(total_mask):
                continue
            ys, xs = np.where(total_mask)
            cx = xs.mean()
            cy = ys.mean()
            info = self.room_map_msg.info  # type: ignore
            world_x = info.origin.position.x + (cx + 0.5) * info.resolution
            world_y = info.origin.position.y + (cy + 0.5) * info.resolution

            marker = Marker()
            marker.header.frame_id = info.header.frame_id if hasattr(info, 'header') else 'map'
            marker.header.stamp = stamp
            marker.ns = 'semantic_room_labels'
            marker.id = hash(room_name) & 0x7FFFFFFF
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.pose.position.x = float(world_x)
            marker.pose.position.y = float(world_y)
            marker.pose.position.z = 0.5
            marker.pose.orientation.w = 1.0
            marker.scale.z = 0.4
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.color.a = 0.9
            marker.text = room_name
            ma.markers.append(marker)

        if ma.markers:
            self.labels_pub.publish(ma)
            self.get_logger().info(
                f'/semantic_room_labels publicado com {len(ma.markers)} marcadores.'
            )
        else:
            self.get_logger().info('/semantic_room_labels não teve marcadores para publicar.')

    def _lookup_room_id(self, pose: Pose) -> Optional[int]:
        info = self.room_map_msg.info  # type: ignore
        origin = info.origin.position
        res = info.resolution
        width = info.width
        height = info.height
        mx = int((pose.position.x - origin.x) / res)
        my = int((pose.position.y - origin.y) / res)
        if mx < 0 or my < 0 or mx >= width or my >= height:
            return None
        return int(self.room_map_np[my, mx])  # type: ignore

    def _semantic_value(self, room_name: str) -> int:
        if room_name not in self.room_value_map:
            base = 30
            step = 20
            next_val = base + step * len(self.room_value_map)
            next_val = ((next_val - base) % 120) + base
            self.room_value_map[room_name] = next_val
        return self.room_value_map[room_name]

    def _resolve_semantic_file(self) -> Optional[Path]:
        current = Path(__file__).resolve()
        rel_path = Path('config') / 'semantic_room_seg_classes.json'
        for ancestor in current.parents:
            direct = ancestor / 'src' / 'my_robot_pkg' / rel_path
            if direct.exists():
                return direct
            workspace = ancestor / 'final_project_ws' / 'src' / 'my_robot_pkg' / rel_path
            if workspace.exists():
                return workspace
        try:
            from ament_index_python.packages import get_package_share_directory

            share_dir = Path(get_package_share_directory('my_robot_pkg'))
            candidate = share_dir / rel_path
            if candidate.exists():
                return candidate
        except Exception:
            pass
        return None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SemanticRoomSegmentationNode()
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
