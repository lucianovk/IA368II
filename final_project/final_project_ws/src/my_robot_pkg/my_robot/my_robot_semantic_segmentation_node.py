#!/usr/bin/env python3
"""
Convert object detections into semantic room segments.
Reads /detections_markers (MarkerArray), associates labels to the rooms defined
in semantic_room_seg_classes.json, and colors /topologic_map with the estimated room,
publishing the result as an OccupancyGrid on /semantic_map.
"""

import colorsys
import hashlib
import math
from pathlib import Path
from typing import Dict, Optional, Tuple

import json
import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


class SemanticRoomSegmentationNode(Node):
    def __init__(self) -> None:
        super().__init__('my_robot_semantic_segmentation_node')

        default_semantic_path = self._resolve_semantic_file()
        if default_semantic_path is None:
            default_semantic_path = (
                Path(__file__).resolve().parent.parent
                / 'config'
                / 'semantic_room_seg_classes.json'
            )
        self.declare_parameter('semantic_map_file', str(default_semantic_path))
        self.declare_parameter('room_map_topic', '/topologic_map')
        self.declare_parameter('detections_topic', '/detections_markers')
        self.declare_parameter('semantic_map_topic', '/semantic_map')
        self.declare_parameter('label_refresh_period', 5.0)
        self.declare_parameter('label_shift_threshold', 0.25)
        self.declare_parameter('legend_topic', '/semantic_map_legend_markers')
        self.declare_parameter('legend_origin_x', 0.0)
        self.declare_parameter('legend_origin_y', 0.0)
        self.declare_parameter('legend_spacing', 0.4)

        semantic_map_path = Path(self.get_parameter('semantic_map_file').value)
        if not semantic_map_path.exists():
            fallback = self._resolve_semantic_file()
            if fallback is not None:
                self.get_logger().warn(
                    f'{semantic_map_path} not found; using {fallback} as a fallback.'
                )
                semantic_map_path = fallback
            else:
                self.get_logger().warn(
                    f'{semantic_map_path} not found and no fallback available.'
                )
        self.label_to_room = self._load_semantic_classes(semantic_map_path)
        self.get_logger().info(
            f'{len(self.label_to_room)} semantic labels loaded from {semantic_map_path}'
        )

        self.room_map_topic = self.get_parameter('room_map_topic').value
        self.detections_topic = self.get_parameter('detections_topic').value
        self.semantic_topic = self.get_parameter('semantic_map_topic').value
        self.label_refresh_period = float(self.get_parameter('label_refresh_period').value)
        shift_param = float(self.get_parameter('label_shift_threshold').value)
        self.label_shift_threshold = 0.01 if shift_param < 0.01 else shift_param
        self.legend_topic = self.get_parameter('legend_topic').value
        self.legend_origin_x = float(self.get_parameter('legend_origin_x').value)
        self.legend_origin_y = float(self.get_parameter('legend_origin_y').value)
        self.legend_spacing = float(self.get_parameter('legend_spacing').value)

        self.room_map_msg: Optional[OccupancyGrid] = None
        self.room_map_np: Optional[np.ndarray] = None
        self.room_value_map: Dict[str, int] = {}
        self.latest_assignments: Dict[int, str] = {}
        self.semantic_dirty = False
        self.last_room_values: Dict[str, int] = {}
        self.last_room_ids: Dict[str, list[int]] = {}
        self.last_label_positions: Dict[str, Tuple[float, float]] = {}

        self.room_sub = self.create_subscription(
            OccupancyGrid, self.room_map_topic, self._room_map_cb, 10
        )
        self.detections_sub = self.create_subscription(
            MarkerArray, self.detections_topic, self._detections_cb, 10
        )
        self.semantic_pub = self.create_publisher(OccupancyGrid, self.semantic_topic, 10)
        self.publish_timer = self.create_timer(10.0, self._publish_semantic_map)
        self.labels_pub = self.create_publisher(MarkerArray, '/semantic_map_labels_markers', 10)
        self.legend_pub = self.create_publisher(MarkerArray, self.legend_topic, 10)
        self.labels_position_timer = None
        if self.label_refresh_period > 0.0:
            self.labels_position_timer = self.create_timer(
                self.label_refresh_period, self._labels_timer_cb
            )

    def _load_semantic_classes(self, path: Path) -> Dict[str, str]:
        """Read the label->room mapping JSON and normalize labels."""
        if not path.exists():
            self.get_logger().warn(f'File {path} not found; semantic map empty.')
            return {}
        try:
            with path.open('r', encoding='utf-8') as fp:
                data = json.load(fp)
        except Exception as exc:
            self.get_logger().error(f'Failed to load {path}: {exc}')
            return {}
        mapping = {k.strip().lower(): v for k, v in data.items()}
        return mapping

    def _room_map_cb(self, msg: OccupancyGrid) -> None:
        """Cache the most recent region map and mark semantic map for rebuild."""
        self.room_map_msg = msg
        np_data = np.array(msg.data, dtype=np.int16).reshape(
            (msg.info.height, msg.info.width)
        )
        self.room_map_np = np_data
        self.get_logger().info(
            f'/topologic_map updated ({msg.info.width}x{msg.info.height}). Rebuilding semantic map.'
        )
        self.semantic_dirty = True

    def _detections_cb(self, msg: MarkerArray) -> None:
        """Associate incoming labels with rooms using the last known static map."""
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
                    f'Detection assigned: room_id={room_id} -> {room_name}'
                )
                updated = True

        if updated:
            self.semantic_dirty = True

    def _publish_semantic_map(self) -> None:
        """Convert the last assignments into an OccupancyGrid and publish it."""
        if not self.semantic_dirty:
            return
        if (
            self.room_map_msg is None
            or self.room_map_np is None
            or not self.latest_assignments
        ):
            return

        # Default to -1 so we keep track of cells that never received any semantic label.
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

        # Preserve walls/obstacles and free cells without labels so downstream consumers
        # can still reason about traversability.
        semantic_grid[self.room_map_np == 100] = 100
        semantic_grid[(self.room_map_np == 0) & (semantic_grid == -1)] = 0

        out_msg = OccupancyGrid()
        out_msg.header = self.room_map_msg.header
        out_msg.info = self.room_map_msg.info
        out_msg.data = semantic_grid.flatten().tolist()
        self.semantic_pub.publish(out_msg)
        self.last_room_values = dict(room_name_to_value)
        self.last_room_ids = {name: ids[:] for name, ids in room_name_to_ids.items()}
        self._publish_labels(room_name_to_value, room_name_to_ids)
        self._publish_legend(room_name_to_value)
        self.get_logger().info(
            f'/semantic_map published with {len(room_name_to_value)} classes.'
        )
        self.semantic_dirty = False

    def _labels_timer_cb(self) -> None:
        """Periodically re-evaluate whether label markers moved."""
        if not self.last_room_values or not self.last_room_ids:
            return
        self._publish_labels(self.last_room_values, self.last_room_ids, periodic=True)

    def _publish_labels(
        self,
        room_values: Dict[str, int],
        room_ids: Dict[str, list[int]],
        periodic: bool = False,
    ) -> None:
        """Publish semantic room labels and optionally refresh them periodically."""
        if self.room_map_msg is None or self.room_map_np is None:
            return
        ma = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        new_positions: Dict[str, Tuple[float, float]] = {}
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
            new_positions[room_name] = (float(world_x), float(world_y))

        if not ma.markers:
            self.last_label_positions = {}
            if not periodic:
                self.get_logger().info('/semantic_map_labels_markers had no markers to publish.')
            return

        should_publish = True
        if periodic and self.last_label_positions:
            should_publish = self._labels_positions_changed(new_positions)
        if not should_publish:
            return

        self.labels_pub.publish(ma)
        self.last_label_positions = new_positions
        if periodic:
            self.get_logger().info(
                f'/semantic_map_labels_markers refreshed with {len(ma.markers)} markers.'
            )
        else:
            self.get_logger().info(
                f'/semantic_map_labels_markers published with {len(ma.markers)} markers.'
            )

    def _publish_legend(self, room_values: Dict[str, int]) -> None:
        """Publish a simple color legend so RViz users can match colors to rooms."""
        if not room_values or self.legend_pub is None:
            return
        frame_id = 'map'
        if self.room_map_msg is not None and self.room_map_msg.header.frame_id:
            frame_id = self.room_map_msg.header.frame_id
        ma = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        sorted_rooms = sorted(room_values.keys())
        base_x = self.legend_origin_x
        base_y = self.legend_origin_y
        spacing = 0.4 if self.legend_spacing <= 0 else self.legend_spacing
        for idx, room_name in enumerate(sorted_rooms):
            y_offset = base_y + spacing * idx
            color = self._semantic_color(room_name)
            cube = Marker()
            cube.header.frame_id = frame_id
            cube.header.stamp = stamp
            cube.ns = 'semantic_room_legend'
            cube.id = idx
            cube.type = Marker.CUBE
            cube.action = Marker.ADD
            cube.pose.position.x = float(base_x)
            cube.pose.position.y = float(y_offset)
            cube.pose.position.z = 0.2
            cube.pose.orientation.w = 1.0
            cube.scale.x = 0.25
            cube.scale.y = 0.25
            cube.scale.z = 0.05
            cube.color.r, cube.color.g, cube.color.b = color
            cube.color.a = 1.0
            ma.markers.append(cube)

            text = Marker()
            text.header.frame_id = frame_id
            text.header.stamp = stamp
            text.ns = 'semantic_room_legend_text'
            text.id = 1000 + idx
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = float(base_x + 0.35)
            text.pose.position.y = float(y_offset)
            text.pose.position.z = 0.3
            text.pose.orientation.w = 1.0
            text.scale.z = 0.25
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.color.a = 0.95
            text.text = room_name
            ma.markers.append(text)

        if ma.markers:
            self.legend_pub.publish(ma)
            self.get_logger().info(
                f'/semantic_map_legend_markers published with {len(sorted_rooms)} entries.'
            )

    def _labels_positions_changed(
        self, new_positions: Dict[str, Tuple[float, float]]
    ) -> bool:
        """Return True if any label moved farther than the configured threshold."""
        if not self.last_label_positions:
            return True
        if len(new_positions) != len(self.last_label_positions):
            return True
        threshold = self.label_shift_threshold
        for room_name, coords in new_positions.items():
            previous = self.last_label_positions.get(room_name)
            if previous is None:
                return True
            if math.hypot(coords[0] - previous[0], coords[1] - previous[1]) >= threshold:
                return True
        return False

    def _lookup_room_id(self, pose: Pose) -> Optional[int]:
        """Convert a pose in map coordinates to the discrete room/region id."""
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
        """Assign pseudo-color map values spaced apart to separate room IDs."""
        if room_name not in self.room_value_map:
            base = 30
            step = 20
            next_val = base + step * len(self.room_value_map)
            next_val = ((next_val - base) % 120) + base
            self.room_value_map[room_name] = next_val
        return self.room_value_map[room_name]

    def _semantic_color(self, room_name: str) -> Tuple[float, float, float]:
        """Derive a repeatable RGB color for the legend based on the room name."""
        digest = hashlib.sha1(room_name.encode('utf-8')).hexdigest()
        seed = int(digest[:6], 16)
        hue = (seed % 360) / 360.0
        sat = 0.65
        val = 0.95
        r, g, b = colorsys.hsv_to_rgb(hue, sat, val)
        return float(r), float(g), float(b)

    def _resolve_semantic_file(self) -> Optional[Path]:
        """Try to locate the semantic class file in common workspaces/install spaces."""
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
