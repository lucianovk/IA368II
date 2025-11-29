#!/usr/bin/env python3
"""
Traverse the current CoppeliaSim scene, list every object that exposes a valid
3D bounding box, position the robot so its vision sensor faces the object from
floor level, capture a single RGB frame, and save YOLO-style annotations.

Run from inside `final_project/final_project_ws`:

    python ../scripts/capture_scene_objects.py --output ./synthetic_export
"""

from __future__ import annotations

import argparse
import json
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List, Optional, Tuple

import numpy as np
from PIL import Image

try:
    from coppeliasim_zmqremoteapi_client import RemoteAPIClient
except ImportError as exc:  # pragma: no cover
    raise SystemExit(
        'coppeliasim_zmqremoteapi_client is required. Install it with pip.'
    ) from exc


@dataclass
class SceneObject:
    handle: int
    name: str
    position: Tuple[float, float, float]
    bbox_min: np.ndarray
    bbox_max: np.ndarray


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='127.0.0.1', help='Remote API host.')
    parser.add_argument('--port', type=int, default=23000, help='Remote API port.')
    parser.add_argument('--robot', default='/myRobot', help='Robot base alias/path.')
    parser.add_argument('--vision-sensor', default='/myRobot/visionSensor', help='Vision sensor path.')
    parser.add_argument('--delay', type=float, default=0.2, help='Delay after moving before capture (s).')
    parser.add_argument('--output', type=Path, required=True, help='Directory where images and labels will be stored.')
    parser.add_argument('--image-format', default='png', choices=('png', 'jpg', 'jpeg'), help='Image format to save.')
    parser.add_argument('--min-area', type=float, default=0.01, help='Minimum normalized bbox area to accept.')
    parser.add_argument('--flip-horizontal', action='store_true', help='Mirror images horizontally.')
    parser.add_argument('--flip-vertical', action='store_true', help='Flip images vertically (default off).')
    parser.add_argument('--max-bbox-diagonal', type=float, default=6.0, help='Skip objects whose bbox diagonal exceeds this (meters).')
    parser.add_argument('--yaw-samples', type=int, default=4, help='Number of yaw angles to try around each object.')
    parser.add_argument('--arena-height', type=float, default=3.0, help='Height offset (m) above the original floor for the capture arena.')
    return parser.parse_args()




def connect(host: str, port: int):
    client = RemoteAPIClient(host=host, port=port)
    return client.require('sim')


def load_allowed_names(path: Path) -> set[str]:
    try:
        with path.open('r', encoding='utf-8') as fp:
            data = json.load(fp)
    except Exception as exc:
        raise SystemExit(f'Failed to read {path}: {exc}') from exc
    if isinstance(data, dict):
        if 'objects' in data and isinstance(data['objects'], list):
            data = data['objects']
        else:
            raise SystemExit(f'JSON {path} must be a list of names or contain an "objects" list.')
    if not isinstance(data, list):
        raise SystemExit(f'JSON {path} must be a list of names.')
    names = {str(item) for item in data}
    if not names:
        raise SystemExit(f'JSON {path} did not contain any object names.')
    return names


def ensure_sim_running(sim) -> None:
    """Stop any older simulation, enable stepping, and start fresh."""
    try:
        sim.stopSimulation()
        while sim.getSimulationState() != sim.simulation_stopped:
            time.sleep(0.05)
    except Exception:
        pass
    sim.setStepping(True)
    sim.startSimulation()


def get_handle(sim, path: str) -> int:
    try:
        return sim.getObject(path)
    except Exception:
        return sim.getObjectHandle(path)


def clone_sensor(sim, source_handle: int) -> int:
    """Duplicate the source vision sensor so we can move it freely."""
    for option in (1, 0):
        try:
            copies = sim.copyPasteObjects([source_handle], option)
        except Exception:
            copies = []
        if copies:
            break
    if not copies:
        raise RuntimeError('Failed to duplicate vision sensor.')
    new_handle = copies[0]
    sim.setObjectParent(new_handle, sim.handle_world, True)
    alias = sim.getObjectAlias(source_handle, 0) or 'visionSensor'
    sim.setObjectAlias(new_handle, f'{alias}_capture')
    return new_handle


def duplicate_object(sim, handle: int) -> Optional[int]:
    """Copy an object (with children) so we can move it into the capture arena."""
    for option in (1, 0):
        try:
            copies = sim.copyPasteObjects([handle], option)
        except Exception:
            copies = []
        if copies:
            sim.setObjectParent(copies[0], sim.handle_world, True)
            return copies[0]
    return None


def remove_object_tree(sim, handle: int) -> None:
    """Remove a duplicated object, falling back to removeModel when suitable."""
    try:
        sim.removeModel(handle)
        return
    except Exception:
        pass
    try:
        sim.removeObject(handle)
    except Exception:
        pass


def create_capture_floor(sim, origin: np.ndarray, size: float = 4.0) -> int:
    shape_handle = sim.createPrimitiveShape(
        sim.primitiveshape_plane,
        [size, size, 0.01],
        0,
    )
    sim.setObjectPosition(shape_handle, sim.handle_world, origin.tolist())
    sim.setObjectOrientation(shape_handle, sim.handle_world, [0.0, 0.0, 0.0])
    sim.setShapeColor(shape_handle, None, sim.colorcomponent_ambient_diffuse, [0.65, 0.65, 0.65])
    sim.setShapeColor(shape_handle, None, sim.colorcomponent_specular, [0.1, 0.1, 0.1])
    sim.setObjectAlias(shape_handle, 'capture_floor')
    return shape_handle


def capture_rgb(
    sim, sensor_handle: int, flip_horizontal: bool, flip_vertical: bool
) -> Tuple[np.ndarray, Tuple[int, int]]:
    sim.handleVisionSensor(sensor_handle)
    raw = sim.getVisionSensorCharImage(sensor_handle)
    data = None
    res = None
    if isinstance(raw, (list, tuple)):
        if len(raw) == 3:
            data, rx, ry = raw
            res = (int(rx), int(ry))
        elif len(raw) == 2:
            a, b = raw
            if isinstance(a, (list, tuple)) and len(a) == 2:
                res, data = (int(a[0]), int(a[1])), b
            elif isinstance(b, (list, tuple)) and len(b) == 2:
                res, data = (int(b[0]), int(b[1])), a
    if res is None or data is None:
        raise RuntimeError(f'Unexpected vision sensor result: {raw}')
    width, height = res
    arr = np.frombuffer(data, dtype=np.uint8)
    expected = width * height * 3
    if arr.size != expected:
        raise RuntimeError(f'Unexpected RGB buffer length {arr.size} vs {expected}')
    arr = arr.reshape((height, width, 3))
    if flip_vertical:
        arr = np.flipud(arr)
    if flip_horizontal:
        arr = np.fliplr(arr)
    return arr.copy(), (width, height)


def query_sensor_info(sim, sensor_handle: int) -> Tuple[int, int, float]:
    res = sim.getVisionSensorResolution(sensor_handle)
    width, height = int(res[0]), int(res[1])
    try:
        fov = sim.getObjectFloatParam(sensor_handle, sim.visionfloatparam_perspective_angle)
        fov_deg = math.degrees(fov)
    except Exception:
        fov_deg = 57.0
    return width, height, fov_deg


def object_bbox(sim, handle: int) -> Optional[Tuple[np.ndarray, np.ndarray]]:
    try:
        min_x = sim.getObjectFloatParam(handle, sim.objfloatparam_modelbbox_min_x)
        min_y = sim.getObjectFloatParam(handle, sim.objfloatparam_modelbbox_min_y)
        min_z = sim.getObjectFloatParam(handle, sim.objfloatparam_modelbbox_min_z)
        max_x = sim.getObjectFloatParam(handle, sim.objfloatparam_modelbbox_max_x)
        max_y = sim.getObjectFloatParam(handle, sim.objfloatparam_modelbbox_max_y)
        max_z = sim.getObjectFloatParam(handle, sim.objfloatparam_modelbbox_max_z)
    except Exception:
        return None
    if None in (min_x, min_y, min_z, max_x, max_y, max_z):
        return None
    return (
        np.array((float(min_x), float(min_y), float(min_z)), dtype=float),
        np.array((float(max_x), float(max_y), float(max_z)), dtype=float),
    )


def object_matrix(sim, handle: int) -> np.ndarray:
    data = sim.getObjectMatrix(handle, sim.handle_world)
    mat = np.eye(4)
    mat[0, :3] = data[0:3]
    mat[1, :3] = data[3:6]
    mat[2, :3] = data[6:9]
    mat[:3, 3] = data[9:12]
    return mat


def bbox_corners(mins: np.ndarray, maxs: np.ndarray) -> List[np.ndarray]:
    corners: List[np.ndarray] = []
    for dx in (mins[0], maxs[0]):
        for dy in (mins[1], maxs[1]):
            for dz in (mins[2], maxs[2]):
                corners.append(np.array([dx, dy, dz], dtype=float))
    return corners


def transform_points(mat: np.ndarray, points: Iterable[np.ndarray]) -> List[np.ndarray]:
    out: List[np.ndarray] = []
    for pt in points:
        vec = np.array([pt[0], pt[1], pt[2], 1.0])
        world = mat @ vec
        out.append(world[:3])
    return out


def euler_to_rot(euler: Tuple[float, float, float]) -> np.ndarray:
    alpha, beta, gamma = euler
    ca, cb, cg = math.cos(alpha), math.cos(beta), math.cos(gamma)
    sa, sb, sg = math.sin(alpha), math.sin(beta), math.sin(gamma)
    return np.array(
        [
            [cg * cb, cg * sb * sa - sg * ca, cg * sb * ca + sg * sa],
            [sg * cb, sg * sb * sa + cg * ca, sg * sb * ca - cg * sa],
            [-sb, cb * sa, cb * ca],
        ]
    )


def rotation_to_euler(rot: np.ndarray) -> Tuple[float, float, float]:
    sy = math.sqrt(rot[0, 0] ** 2 + rot[1, 0] ** 2)
    singular = sy < 1e-6
    if not singular:
        alpha = math.atan2(rot[2, 1], rot[2, 2])
        beta = math.atan2(-rot[2, 0], sy)
        gamma = math.atan2(rot[1, 0], rot[0, 0])
    else:
        alpha = math.atan2(-rot[1, 2], rot[1, 1])
        beta = math.atan2(-rot[2, 0], sy)
        gamma = 0.0
    return (alpha, beta, gamma)


def look_at_matrix(position: np.ndarray, target: np.ndarray) -> np.ndarray:
    forward = target - position
    norm = np.linalg.norm(forward)
    if norm < 1e-9:
        raise ValueError('Camera coincides with target.')
    forward /= norm
    up = np.array([0.0, 0.0, 1.0])
    if abs(np.dot(forward, up)) > 0.95:
        up = np.array([0.0, 1.0, 0.0])
    right = np.cross(forward, up)
    right_norm = np.linalg.norm(right)
    if right_norm < 1e-9:
        raise ValueError('Degenerate camera orientation.')
    right /= right_norm
    true_up = np.cross(right, forward)
    rot = np.column_stack((right, true_up, forward))
    return rot


def project_points(
    points: Iterable[np.ndarray],
    cam_pos: np.ndarray,
    cam_rot: np.ndarray,
    width: int,
    height: int,
    fx: float,
    fy: float,
) -> Optional[Tuple[float, float, float, float]]:
    cx = width * 0.5
    cy = height * 0.5
    rot_t = cam_rot.T
    projected: List[Tuple[float, float]] = []
    for pt in points:
        rel = pt - cam_pos
        cam_coord = rot_t @ rel
        if cam_coord[2] <= 0:
            return None
        u = fx * (cam_coord[0] / cam_coord[2]) + cx
        v = fy * (cam_coord[1] / cam_coord[2]) + cy
        projected.append((u, v))
    xs = [p[0] for p in projected]
    ys = [p[1] for p in projected]
    min_u, max_u = min(xs), max(xs)
    min_v, max_v = min(ys), max(ys)
    if max_u <= 0 or max_v <= 0 or min_u >= width or min_v >= height:
        return None
    min_u = max(0.0, min_u)
    min_v = max(0.0, min_v)
    max_u = min(float(width), max_u)
    max_v = min(float(height), max_v)
    return (min_u, min_v, max_u, max_v)


def to_yolo(
    bbox: Tuple[float, float, float, float],
    width: int,
    height: int,
    flip_horizontal: bool,
) -> Tuple[float, float, float, float]:
    min_u, min_v, max_u, max_v = bbox
    if flip_horizontal:
        min_u, max_u = width - max_u, width - min_u
    cx = (min_u + max_u) * 0.5 / width
    cy = (min_v + max_v) * 0.5 / height
    w = (max_u - min_u) / width
    h = (max_v - min_v) / height
    return (cx, cy, w, h)


def save_image(image: np.ndarray, path: Path) -> None:
    Image.fromarray(image).save(path)


def write_label(path: Path, class_id: int, bbox: Tuple[float, float, float, float]) -> None:
    with path.open('w', encoding='utf-8') as fp:
        fp.write(f'{class_id} {bbox[0]:.6f} {bbox[1]:.6f} {bbox[2]:.6f} {bbox[3]:.6f}\n')


def label_key(name: str) -> str:
    return name.replace(' ', '_')


def _aligned_euler(euler: Tuple[float, float, float]) -> Tuple[float, float, float]:
    """Return Coppelia-friendly Euler orientation so optical axis matches +X."""
    return (euler[0], euler[1] - math.pi * 0.5, euler[2])


def main() -> int:
    args = parse_args()
    print(f'[capture_scene_objects] Export directory: {args.output}', flush=True)
    sim = connect(args.host, args.port)
    print(f'[capture_scene_objects] Connected to {args.host}:{args.port}', flush=True)
    ensure_sim_running(sim)
    print('[capture_scene_objects] Simulation running (stepping mode).', flush=True)

    robot_handle = get_handle(sim, args.robot)
    source_sensor = get_handle(sim, args.vision_sensor)
    sensor_handle = clone_sensor(sim, source_sensor)
    robot_base_pos = sim.getObjectPosition(robot_handle, sim.handle_world)
    width, height, fov_deg = query_sensor_info(sim, sensor_handle)
    fy = (height * 0.5) / math.tan(math.radians(fov_deg * 0.5))
    fx = fy * (width / height)
    print(f'[capture_scene_objects] Camera resolution {width}x{height}, fov={fov_deg:.1f}°', flush=True)

    capture_origin = np.array(
        [
            float(robot_base_pos[0]),
            float(robot_base_pos[1]),
            float(robot_base_pos[2]) + max(0.5, args.arena_height) + 3.0,
        ]
    )
    floor_handle = create_capture_floor(sim, capture_origin)
    try:
        default_cam = sim.getObject('/DefaultCamera')
    except Exception:
        default_cam = None
    if default_cam is not None:
        eye_offset = np.array([0.0, -4.0, 2.0])
        eye_pos = capture_origin + eye_offset
        target = capture_origin.copy()
        target[2] += 0.25
        sim.setObjectPosition(default_cam, sim.handle_world, eye_pos.tolist())
        try:
            rot = look_at_matrix(eye_pos, target)
            euler = _aligned_euler(rotation_to_euler(rot))
            sim.setObjectOrientation(default_cam, sim.handle_world, list(euler))
        except ValueError:
            pass

    default_filter = Path(__file__).resolve().parent / 'capture_objects.json'
    allowed_names = load_allowed_names(default_filter)

    handles = sim.getObjectsInTree(sim.handle_scene, sim.handle_all, 0)
    scene_objects: List[SceneObject] = []
    seen_names: set[str] = set()
    for handle in handles:
        if handle in (robot_handle, sensor_handle):
            continue
        bbox = object_bbox(sim, handle)
        if bbox is None:
            continue
        name = sim.getObjectAlias(handle, 0) or f'obj_{handle}'
        if allowed_names is not None and name not in allowed_names:
            continue
        if name in seen_names:
            continue
        seen_names.add(name)
        diag = np.linalg.norm(bbox[1] - bbox[0])
        if diag > args.max_bbox_diagonal:
            continue
        pos = sim.getObjectPosition(handle, sim.handle_world)
        scene_objects.append(
            SceneObject(
                handle=handle,
                name=name,
                position=(float(pos[0]), float(pos[1]), float(pos[2])),
                bbox_min=bbox[0],
                bbox_max=bbox[1],
            )
        )
    if not scene_objects:
        print('[capture_scene_objects] No objects with valid bounding boxes found.', flush=True)
        sim.stopSimulation()
        return 0

    print('[capture_scene_objects] Objects eligible for capture:')
    for idx, obj in enumerate(scene_objects):
        print(f'  [{idx:02d}] {obj.name} at {obj.position}')

    images_dir = args.output / 'images'
    labels_dir = args.output / 'labels'
    images_dir.mkdir(parents=True, exist_ok=True)
    labels_dir.mkdir(parents=True, exist_ok=True)
    class_ids = {label_key(obj.name): i for i, obj in enumerate(scene_objects)}

    sensor_height = float(capture_origin[2] + 0.2)
    sim.setObjectPosition(sensor_handle, sim.handle_world, [capture_origin[0], capture_origin[1], sensor_height])
    sim.setObjectOrientation(sensor_handle, sim.handle_world, list(_aligned_euler((0.0, 0.0, 0.0))))
    capture_center = capture_origin.copy()
    total = 0
    for obj in scene_objects:
        print(f'[capture_scene_objects] Capturing {obj.name}...', flush=True)
        dup_handle = duplicate_object(sim, obj.handle)
        if dup_handle is None:
            print('  -> skipping: failed to duplicate object.')
            continue
        try:
            sim.setObjectOrientation(dup_handle, sim.handle_world, [0.0, 0.0, 0.0])
        except Exception:
            pass
        bbox = object_bbox(sim, dup_handle)
        if bbox is None:
            print('  -> skipping: duplicate missing bbox.')
            remove_object_tree(sim, dup_handle)
            continue
        mins, maxs = bbox
        center_local = (mins + maxs) * 0.5
        size = maxs - mins
        place_offset = np.array(
            [
                -center_local[0],
                -center_local[1],
                -mins[2],
            ],
            dtype=float,
        )
        object_pos = capture_center + place_offset
        sim.setObjectPosition(dup_handle, sim.handle_world, object_pos.tolist())
        target_world = np.array([capture_center[0], capture_center[1], sensor_height])
        base_extent = max(size[0], size[1])
        successful = False
        last_reason = 'unknown'
        default_distance = 1.2
        yaw_angles = np.linspace(0.0, 2.0 * math.pi, num=max(1, args.yaw_samples), endpoint=False) + (math.pi * 0.5)
        for yaw in yaw_angles:
            for scale in (1.0, 0.7, 0.5, 0.35):
                adaptive_distance = max(0.3, scale * default_distance, 0.4 * base_extent)
                offset = np.array(
                    [
                        adaptive_distance * math.cos(yaw),
                        adaptive_distance * math.sin(yaw),
                        0.0,
                    ]
                )
                cam_pos = target_world + offset
                cam_pos[2] = sensor_height
                try:
                    rot = look_at_matrix(cam_pos, target_world)
                    euler = rotation_to_euler(rot)
                except ValueError as exc:
                    last_reason = str(exc)
                    continue
                sim.setObjectPosition(sensor_handle, sim.handle_world, cam_pos.tolist())
                sim.setObjectOrientation(sensor_handle, sim.handle_world, list(_aligned_euler(euler)))
                sim.step()
                time.sleep(max(0.0, args.delay))

                try:
                    image, (img_w, img_h) = capture_rgb(
                        sim,
                        sensor_handle,
                        flip_horizontal=args.flip_horizontal,
                        flip_vertical=args.flip_vertical,
                    )
                except Exception as exc:
                    last_reason = f'capture failed: {exc}'
                    continue

                corners = bbox_corners(mins, maxs)
                world_matrix = object_matrix(sim, dup_handle)
                corners_world = transform_points(world_matrix, corners)
                bbox_proj = project_points(corners_world, cam_pos, rot, img_w, img_h, fx, fy)
                if bbox_proj is None:
                    last_reason = 'bbox out of view'
                    continue

                yolo_box = to_yolo(bbox_proj, img_w, img_h, args.flip_horizontal)
                if yolo_box[2] * yolo_box[3] < args.min_area:
                    last_reason = 'projected area too small'
                    continue

                safe_name = label_key(obj.name)
                image_name = f'{safe_name}_{total:04d}.{args.image_format}'
                label_name = f'{safe_name}_{total:04d}.txt'
                save_image(image, images_dir / image_name)
                write_label(labels_dir / label_name, class_ids[safe_name], yolo_box)
                print(f'  -> saved {image_name}')
                total += 1
                successful = True
                break
            if successful:
                break

        if not successful:
            print(f'  -> skipping {obj.name}: {last_reason}.')

        remove_object_tree(sim, dup_handle)

    remove_object_tree(sim, sensor_handle)
    remove_object_tree(sim, floor_handle)
    sim.stopSimulation()
    print(f'[capture_scene_objects] Completed with {total} captured frames.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
