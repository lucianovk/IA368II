#!/usr/bin/env python3
"""
Generate synthetic YOLO data from a CoppeliaSim scene.

HEIGHT CORRECTION (Z POSITIONING):
- Places the object center on a virtual reference plane (no physical floor).
- Uses the sensor depth map to calculate real dimensions and adjust camera/labels.
- Keeps the sensor automatic (Implicit) with faithful cloning.

How to run:
    python capture_scene_objects.py --output ./synthetic_export
"""

from __future__ import annotations

import argparse
import json
import math
import os
import queue
import select
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np
from PIL import Image, ImageDraw, ImageTk

try:
    import termios
    import tty
except ImportError:  # pragma: no cover - plataforma sem termios
    termios = None
    tty = None

try:
    import tkinter as tk
except ImportError:  # pragma: no cover - sistemas sem Tk
    tk = None

try:
    from coppeliasim_zmqremoteapi_client import RemoteAPIClient
except ImportError as exc:
    raise SystemExit('Install coppeliasim-zmqremoteapi-client') from exc


@dataclass
class SceneObject:
    handle: int
    name: str
    bbox_min: np.ndarray
    bbox_max: np.ndarray


@dataclass
class DepthAnalysis:
    bbox_px: Tuple[int, int, int, int]
    width_m: float
    height_m: float
    world_min_z: Optional[float] = None
    world_max_z: Optional[float] = None


class QuitRequested(RuntimeError):
    """Raised when the user asks to abort with 'q'."""


class QuitWatcher:
    def __init__(self) -> None:
        self.enabled = bool(sys.stdin.isatty() and termios and tty)
        self.fd = None
        self.prev_settings = None
        self.requested = False

    def __enter__(self):
        if self.enabled:
            try:
                self.fd = sys.stdin.fileno()
                self.prev_settings = termios.tcgetattr(self.fd)
                tty.setcbreak(self.fd)
            except Exception:
                self.enabled = False
        return self

    def __exit__(self, exc_type, exc, tb):
        if self.enabled and self.fd is not None and self.prev_settings is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.prev_settings)

    def poll(self) -> bool:
        if not self.enabled or self.requested:
            return self.requested
        rlist, _, _ = select.select([sys.stdin], [], [], 0)
        if rlist:
            ch = sys.stdin.read(1)
            if ch.lower() == 'q':
                self.requested = True
        return self.requested


def check_quit(watcher: Optional[QuitWatcher], viewer: Optional['DebugViewer'] = None) -> None:
    if watcher and watcher.poll():
        raise QuitRequested()
    if viewer and viewer.stop_requested:
        raise QuitRequested()


class DebugViewer:
    """Tkinter viewer kept in the main thread to avoid async delete errors."""

    def __init__(self, enabled: bool):
        self.enabled = bool(
            enabled and tk is not None and threading.current_thread() is threading.main_thread()
        )
        self._root = None
        self.stop_requested = False
        if self.enabled:
            try:
                self._root = tk.Tk()
                self._root.title('Captura Debug Viewer')
                btn = tk.Button(self._root, text='Stop (q)', command=self._on_stop)
                btn.pack(fill='x')
                self._label = tk.Label(self._root)
                self._label.pack(fill='both', expand=True)
                self._caption = tk.Label(self._root, text='', anchor='w')
                self._caption.pack(fill='x')
            except Exception:
                self.enabled = False
                self._root = None

    def _on_stop(self):
        self.stop_requested = True
        if self._root is not None:
            try:
                self._root.destroy()
            except Exception:
                pass
        self.enabled = False

    def show(self, img_arr: np.ndarray, caption: str):
        if not self.enabled or self._root is None:
            return
        pil_img = Image.fromarray(img_arr)
        photo = ImageTk.PhotoImage(pil_img)
        self._label.configure(image=photo)
        self._label.image = photo
        self._caption.configure(text=caption)
        try:
            self._root.update_idletasks()
            self._root.update()
        except Exception:
            # If window was closed manually, disable further updates
            self.enabled = False

    def stop(self):
        if not self.enabled or self._root is None:
            return
        try:
            self._root.destroy()
        except Exception:
            pass
        self.enabled = False


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=23000)
    parser.add_argument('--robot', default='/myRobot')
    parser.add_argument('--vision-sensor', default='/myRobot/visionSensor')
    parser.add_argument('--output', type=Path, default=Path('./synthetic_export'))
    parser.add_argument('--delay', type=float, default=0.2)
    parser.add_argument('--fov', type=float, default=57.0)
    parser.add_argument('--max-bbox-diagonal', type=float, default=10.0)
    parser.add_argument('--samples', type=int, default=8)
    parser.add_argument('--arena-height', type=float, default=5.0)
    parser.add_argument('--depth-band', type=float, default=0.35, help='Meters above nearest depth to keep as object mask.')
    return parser.parse_args()


def connect(host: str, port: int):
    client = RemoteAPIClient(host=host, port=port)
    return client.require('sim')


def get_handle(sim, path: str) -> int:
    try:
        return sim.getObject(path)
    except:
        return sim.getObjectHandle(path)


def object_path(sim, handle: int) -> str:
    """Return hierarchical path for an object starting at scene root."""
    parts = []
    current = handle
    while current not in (sim.handle_scene, -1):
        alias = sim.getObjectAlias(current, 0) or f'h={current}'
        parts.append(alias)
        current = sim.getObjectParent(current)
    parts.reverse()
    return '/' + '/'.join(parts)


def remove_scripts_only(sim, handle: int) -> None:
    try:
        # 15 = sim.object_script_type
        scripts = sim.getObjectsInTree(handle, 15, 0)
        for s_h in scripts:
            sim.removeObject(s_h)
    except: pass


def remove_object_tree(sim, handle: int) -> None:
    try:
        sim.removeModel(handle)
    except:
        try:
            sim.removeObject(handle)
        except: pass


def clone_sensor_simple(sim, source_handle: int) -> int:
    """Clone sensor while keeping original settings."""
    copies = sim.copyPasteObjects([source_handle], 0)
    new_handle = copies[0]
    sim.setObjectParent(new_handle, sim.handle_world, True)
    sim.setObjectAlias(new_handle, 'visionSensor_clone')
    remove_scripts_only(sim, new_handle)
    return new_handle


def read_bbox_params(sim, handle: int, start: int) -> Optional[Tuple[np.ndarray, np.ndarray]]:
    try:
        vals = [sim.getObjectFloatParam(handle, x) for x in range(start, start + 6)]
    except Exception:
        return None
    if any(v is None for v in vals):
        return None
    mins = np.array([vals[0], vals[2], vals[4]])
    maxs = np.array([vals[1], vals[3], vals[5]])
    low = np.minimum(mins, maxs)
    high = np.maximum(mins, maxs)
    if np.max(high - low) < 0.02:
        return None
    return low, high


def get_bbox_from_params(sim, handle: int, name: str) -> Optional[Tuple[np.ndarray, np.ndarray]]:
    """Read BBox via object float parameters (initial filter)."""
    bbox = read_bbox_params(sim, handle, 21)
    if bbox:
        return bbox
    return read_bbox_params(sim, handle, 15)


def capture_rgbd(sim, sensor_handle: int, width: int, height: int, near_clip: float, far_clip: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    try:
        sim.handleVisionSensor(sensor_handle)
    except Exception:
        pass

    img_char = sim.getVisionSensorCharImage(sensor_handle)
    raw_data = img_char
    if isinstance(img_char, (list, tuple)):
        for item in img_char:
            if isinstance(item, bytes):
                raw_data = item
                break
    if not isinstance(raw_data, bytes):
        raw_data = sim.getVisionSensorCharImage(sensor_handle)

    img_arr = np.frombuffer(raw_data, dtype=np.uint8)
    if img_arr.shape[0] != width * height * 3:
        rgb = np.zeros((height, width, 3), dtype=np.uint8)
    else:
        rgb = np.flipud(img_arr.reshape((height, width, 3)))

    depth_m = np.full((height, width), far_clip, dtype=np.float32)
    depth_norm = np.ones((height, width), dtype=np.float32)
    try:
        depth_raw = sim.getVisionSensorDepth(sensor_handle)
    except Exception:
        depth_raw = None

    depth_img = None
    depth_res = None
    if isinstance(depth_raw, (list, tuple)):
        if len(depth_raw) == 3:
            depth_img, res_x, res_y = depth_raw
            depth_res = (int(res_x), int(res_y))
        elif len(depth_raw) == 2:
            a, b = depth_raw
            if isinstance(a, (list, tuple)) and len(a) == 2:
                depth_res, depth_img = (int(a[0]), int(a[1])), b
            elif isinstance(b, (list, tuple)) and len(b) == 2:
                depth_res, depth_img = (int(b[0]), int(b[1])), a
    elif depth_raw is not None:
        depth_img = depth_raw
        depth_res = (width, height)

    if depth_img is not None and depth_res is not None:
        d_w, d_h = depth_res
        if isinstance(depth_img, (bytes, bytearray)):
            depth_arr = np.frombuffer(depth_img, dtype=np.float32)
        else:
            depth_arr = np.array(depth_img, dtype=np.float32)
        if d_w > 0 and d_h > 0 and depth_arr.size == d_w * d_h:
            depth_arr = depth_arr.reshape((d_h, d_w))
            depth_arr = np.flipud(depth_arr)
            depth_norm = depth_arr.copy()
            depth_m = near_clip + depth_arr * (far_clip - near_clip)

    return rgb, depth_m, depth_norm


def get_look_at_matrix(eye: np.ndarray, target: np.ndarray) -> List[float]:
    z_axis = target - eye
    dist = np.linalg.norm(z_axis)
    if dist < 1e-5: z_axis = np.array([0, 0, 1.0])
    else: z_axis /= dist
    
    global_up = np.array([0, 0, 1.0])
    if abs(np.dot(z_axis, global_up)) > 0.98: global_up = np.array([0, 1.0, 0])
    
    x_axis = np.cross(global_up, z_axis)
    x_axis /= np.linalg.norm(x_axis)
    y_axis = np.cross(z_axis, x_axis)
    
    m = [x_axis[0], y_axis[0], z_axis[0], eye[0],
         x_axis[1], y_axis[1], z_axis[1], eye[1],
         x_axis[2], y_axis[2], z_axis[2], eye[2]]
    return m


def adjust_object_height_from_depth(sim, obj_handle: int, floor_z: float, analysis: Optional[DepthAnalysis], log_fn=None) -> bool:
    """Rebaixa ou eleva o objeto para alinhar a base ao plano virtual usando profundidade."""
    if not analysis or analysis.world_min_z is None:
        return False
    delta = floor_z - analysis.world_min_z
    if abs(delta) < 5e-4:
        return False
    pos = np.array(sim.getObjectPosition(obj_handle, sim.handle_world))
    pos[2] += delta
    sim.setObjectPosition(obj_handle, sim.handle_world, pos.tolist())
    if log_fn:
        log_fn(f'Ajuste Z objeto {obj_handle}: delta {delta:+.4f} m')
    return True


def analyze_depth_frame(
    depth_m: np.ndarray,
    depth_norm: np.ndarray,
    far_clip: float,
    fx: float,
    fy: float,
    sensor_matrix: Optional[np.ndarray],
    band: float,
) -> Optional[DepthAnalysis]:
    mask = np.isfinite(depth_m)
    if depth_norm is not None:
        mask &= depth_norm < (1.0 - 1e-6)
    else:
        mask &= depth_m < far_clip - 1e-4
    if not np.any(mask):
        return None
    all_depths = depth_m[mask]
    if all_depths.size == 0:
        return None
    min_depth = float(all_depths.min())
    depth_span_p = float(np.percentile(all_depths, 95) - min_depth)
    adaptive_band = max(band, depth_span_p + 1e-3)
    band_limit = min(far_clip, min_depth + adaptive_band)
    obj_mask = mask & (depth_m <= band_limit)
    if not np.any(obj_mask):
        obj_mask = mask
    ys, xs = np.where(obj_mask)
    min_x, max_x = int(xs.min()), int(xs.max())
    min_y, max_y = int(ys.min()), int(ys.max())
    if fx <= 0 or fy <= 0:
        bbox = (min_x, min_y, max_x, max_y)
        return DepthAnalysis(bbox, 0.0, 0.0)
    depths = depth_m[ys, xs]
    cx = depth_m.shape[1] / 2.0
    cy = depth_m.shape[0] / 2.0
    x_cam = (xs - cx) * depths / fx
    y_cam = (cy - ys) * depths / fy
    width_m = float(np.max(x_cam) - np.min(x_cam)) if x_cam.size else 0.0
    height_m = float(np.max(y_cam) - np.min(y_cam)) if y_cam.size else 0.0
    world_min_z = None
    world_max_z = None
    if sensor_matrix is not None and depths.size:
        R = sensor_matrix[:, :3]
        t = sensor_matrix[:, 3]
        cam_pts = np.stack([x_cam, y_cam, depths], axis=1)
        world_pts = cam_pts @ R.T + t
        if world_pts.size:
            world_min_z = float(np.min(world_pts[:, 2]))
            world_max_z = float(np.max(world_pts[:, 2]))
    bbox = (min_x, min_y, max_x, max_y)
    return DepthAnalysis(bbox, width_m, height_m, world_min_z, world_max_z)


def bbox_px_to_yolo(bbox_px: Tuple[int, int, int, int], img_w: int, img_h: int) -> Tuple[float, float, float, float]:
    min_x, min_y, max_x, max_y = bbox_px
    min_x = np.clip(min_x, 0, img_w - 1)
    max_x = np.clip(max_x, 0, img_w - 1)
    min_y = np.clip(min_y, 0, img_h - 1)
    max_y = np.clip(max_y, 0, img_h - 1)
    w = max(1, max_x - min_x)
    h = max(1, max_y - min_y)
    cx = (min_x + max_x) / 2.0 / img_w
    cy = (min_y + max_y) / 2.0 / img_h
    return cx, cy, w / img_w, h / img_h


def estimate_size_with_depth(
    sim,
    sensor_handle: int,
    arena_center: np.ndarray,
    cam_z: float,
    res_w: int,
    res_h: int,
    near_clip: float,
    far_clip: float,
    fx: float,
    fy: float,
    delay: float,
    band: float,
    log_fn = None,
) -> Optional[DepthAnalysis]:
    log = log_fn or (lambda _msg: None)
    log('estimate_size_with_depth: positioning test sensor')
    test_dist = 3.0
    cam_pos = np.array([arena_center[0] + test_dist, arena_center[1], cam_z])
    target = arena_center.copy()
    target[2] = cam_z
    mat = get_look_at_matrix(cam_pos, target)
    sim.setObjectMatrix(sensor_handle, sim.handle_world, mat)
    log('estimate_size_with_depth: initial step')
    sim.step()
    if delay > 0:
        log(f'estimate_size_with_depth: sleeping {delay:.3f}s to render')
    time.sleep(delay)
    log('estimate_size_with_depth: capturing RGBD...')
    rgb, depth, depth_norm = capture_rgbd(sim, sensor_handle, res_w, res_h, near_clip, far_clip)
    log(f'estimate_size_with_depth: capture finished (RGB sum={np.sum(rgb)})')
    sensor_matrix = np.array(mat, dtype=float).reshape((3, 4))
    stats = analyze_depth_frame(depth, depth_norm, far_clip, fx, fy, sensor_matrix, band)
    if not stats:
        log('estimate_size_with_depth: depth_info None')
        return None
    log(
        'estimate_size_with_depth: width={:.3f} height={:.3f} minZ={}'.format(
            stats.width_m,
            stats.height_m,
            'NA' if stats.world_min_z is None else f'{stats.world_min_z:.3f}',
        )
    )
    return stats


def capture_frame_with_depth_analysis(
    sim,
    sensor_handle: int,
    res_w: int,
    res_h: int,
    near_clip: float,
    far_clip: float,
    fx: float,
    fy: float,
    delay: float,
    band: float,
    quit_watcher: Optional[QuitWatcher] = None,
    viewer: Optional['DebugViewer'] = None,
) -> Tuple[Optional[np.ndarray], Optional[DepthAnalysis]]:
    for attempt in range(3):
        check_quit(quit_watcher, viewer)
        sim.step()
        time.sleep(delay)
        rgb, depth, depth_norm = capture_rgbd(sim, sensor_handle, res_w, res_h, near_clip, far_clip)
        if np.sum(rgb) == 0:
            continue
        sensor_matrix_np = np.array(sim.getObjectMatrix(sensor_handle, sim.handle_world)).reshape((3, 4))
        analysis = analyze_depth_frame(depth, depth_norm, far_clip, fx, fy, sensor_matrix_np, band)
        if not analysis:
            continue
        return rgb, analysis
    return None, None


def optimize_camera_distance_for_pose(
    sim,
    sensor_handle: int,
    arena_center: np.ndarray,
    cam_z: float,
    yaw: float,
    res_w: int,
    res_h: int,
    near_clip: float,
    far_clip: float,
    fx: float,
    fy: float,
    delay: float,
    band: float,
    dbg,
    quit_watcher: Optional[QuitWatcher] = None,
    viewer: Optional['DebugViewer'] = None,
) -> Optional[Tuple[np.ndarray, DepthAnalysis]]:
    max_dist = 6.0
    min_dist = 0.7
    target_margin_px = max(5, int(0.10 * min(res_w, res_h)))
    dist = max_dist
    best: Optional[Tuple[np.ndarray, DepthAnalysis, float, float]] = None
    for step in range(10):
        check_quit(quit_watcher, viewer)
        cx = arena_center[0] + dist * math.cos(yaw)
        cy = arena_center[1] + dist * math.sin(yaw)
        cam_pos = np.array([cx, cy, cam_z])
        target = arena_center.copy()
        target[2] = cam_z
        mat = get_look_at_matrix(cam_pos, target)
        sim.setObjectMatrix(sensor_handle, sim.handle_world, mat)
        dbg(f'Pose yaw={math.degrees(yaw):.1f}° tentativa dist={dist:.2f} m')
        rgb, analysis = capture_frame_with_depth_analysis(
            sim,
            sensor_handle,
            res_w,
            res_h,
            near_clip,
            far_clip,
            fx,
            fy,
            delay,
            band,
            quit_watcher,
            viewer,
        )
        if analysis is None or rgb is None:
            dist = max(dist * 0.85, min_dist)
            continue
        bbox = analysis.bbox_px
        width_px = max(1, bbox[2] - bbox[0])
        height_px = max(1, bbox[3] - bbox[1])
        coverage = (width_px / res_w) * (height_px / res_h)
        margin = min(bbox[0], res_w - 1 - bbox[2], bbox[1], res_h - 1 - bbox[3])
        dbg(
            f'Pose yaw={math.degrees(yaw):.1f}° dist={dist:.2f} m -> coverage={coverage:.3f} margin={margin:.1f}px'
        )
        if best is None or coverage > best[2]:
            best = (rgb, analysis, coverage, dist)
        if margin <= target_margin_px or dist <= min_dist + 1e-3:
            return rgb, analysis
        new_dist = max(dist * 0.85, min_dist)
        if abs(new_dist - dist) < 1e-3:
            break
        dist = new_dist
    if best:
        dbg(
            f'Pose yaw={math.degrees(yaw):.1f}° usando melhor cobertura {best[2]:.3f} dist={best[3]:.2f} m'
        )
        return best[0], best[1]
    return None


def main():
    args = parse_args()
    for d in ['images', 'labels', 'debug']:
        (args.output / d).mkdir(parents=True, exist_ok=True)

    debug_enabled = os.environ.get('CAPTURE_SCENE_DEBUG', '').lower() in {'1', 'true', 'yes'}

    def dbg(msg: str) -> None:
        if debug_enabled:
            ts = time.strftime('%H:%M:%S')
            print(f"[DEBUG {ts}] {msg}")

    sim = None
    sensor = None
    total = 0
    viewer = DebugViewer(True)
    if not viewer.enabled:
        print('Warning: Tkinter unavailable - debug viewer disabled.')
    with QuitWatcher() as quit_watcher:
        try:
            print(f"--- Connecting to {args.host}:{args.port} ---")
            sim = connect(args.host, args.port)
            dbg('Requesting stop of current simulation...')
            try:
                sim.stopSimulation()
                time.sleep(0.5)
            except Exception:
                dbg('Ignoring error while stopping simulation, continuing...')
            sim.setStepping(True)
            sim.startSimulation()
            dbg('Simulation restarted in stepping mode.')

            robot = get_handle(sim, args.robot)
            orig_sensor = get_handle(sim, args.vision_sensor)
            dbg(f'Robot handle={robot}, sensor handle={orig_sensor}')

            res_w, res_h = 640, 480
            near_clip, far_clip = 0.01, 10.0
            try:
                res = sim.getVisionSensorResolution(orig_sensor)
                if isinstance(res, list):
                    res_w, res_h = int(res[0]), int(res[1])
                args.fov = math.degrees(sim.getObjectFloatParam(orig_sensor, 1004))
                near_clip = float(sim.getObjectFloatParam(orig_sensor, sim.visionfloatparam_near_clipping))
                far_clip = float(sim.getObjectFloatParam(orig_sensor, sim.visionfloatparam_far_clipping))
            except Exception:
                pass
            print(f"Sensor config: {res_w}x{res_h}, FOV={args.fov:.1f}")
            fov_x_rad = math.radians(args.fov)
            fov_y_rad = 2.0 * math.atan((res_h / res_w) * math.tan(fov_x_rad / 2.0))
            fx = (res_w / 2.0) / math.tan(fov_x_rad / 2.0)
            fy = (res_h / 2.0) / math.tan(fov_y_rad / 2.0)

            arena_center = np.array([0.0, 0.0, args.arena_height + 5.0])
            floor_z = arena_center[2]

            # Clone the original sensor to keep the automatic camera untouched.
            dbg('Cloning sensor for capture...')
            sensor = clone_sensor_simple(sim, orig_sensor)
            try:
                sensor_fov_rad = sim.getObjectFloatParam(sensor, 1004)
                if sensor_fov_rad:
                    args.fov = math.degrees(sensor_fov_rad)
                    fov_x_rad = sensor_fov_rad
                    fov_y_rad = 2.0 * math.atan((res_h / res_w) * math.tan(fov_x_rad / 2.0))
                    fx = (res_w / 2.0) / math.tan(fov_x_rad / 2.0)
                    fy = (res_h / 2.0) / math.tan(fov_y_rad / 2.0)
                    print(f"Sensor clonado config: {res_w}x{res_h}, FOV={args.fov:.1f}")
                near_clip = float(sim.getObjectFloatParam(sensor, sim.visionfloatparam_near_clipping))
                far_clip = float(sim.getObjectFloatParam(sensor, sim.visionfloatparam_far_clipping))
            except Exception:
                print("Cloned sensor config: incomplete parameters")

            allowed_lookup = None
            allowed_order = None
            jp = Path(__file__).parent / 'capture_objects.json'
            if jp.exists():
                try:
                    with open(jp) as f:
                        data = json.load(f)
                    if isinstance(data, list):
                        allowed_order = data
                    elif isinstance(data, dict):
                        allowed_order = data.get('objects', [])
                    if allowed_order:
                        allowed_lookup = set(allowed_order)
                except Exception:
                    pass

            scene_objects = []
            dbg('Listando todos objetos da cena...')
            all_objs = sim.getObjectsInTree(sim.handle_scene, sim.handle_all, 0)

            print("Escaneando objetos...")
            for h in all_objs:
                check_quit(quit_watcher, viewer)
                if h in [robot, orig_sensor, sensor]:
                    continue
                name = object_path(sim, h)
                # Respect allowlist when provided (expects full object paths).
                if allowed_lookup and name not in allowed_lookup:
                    dbg(f'Ignoring {name}: not listed in capture_objects.json')
                    continue

                bbox = get_bbox_from_params(sim, h, name)
                if bbox:
                    diag = np.linalg.norm(bbox[1] - bbox[0])
                    dbg(f'Object {name}: bbox diag={diag:.2f}')
                    if diag < args.max_bbox_diagonal:
                        scene_objects.append(
                            SceneObject(
                                h,
                                name,
                                bbox[0],
                                bbox[1],
                                None,
                                None,
                            )
                        )
                    else:
                        dbg(f'Object {name}: discarded, diag {diag:.2f} > limit {args.max_bbox_diagonal}')
                else:
                    dbg(f'Object {name}: no valid bbox, ignored')

            print(f"Found {len(scene_objects)} valid objects.")
            scene_name_set = {obj.name for obj in scene_objects}
            if allowed_order:
                class_names = [name for name in allowed_order if name in scene_name_set]
            else:
                class_names = sorted(scene_name_set)
            class_map = {name: i for i, name in enumerate(class_names)}

            cam_z_fixed = floor_z + 0.2
            dbg(f'Starting capture for {len(scene_objects)} objects...')
            for obj in scene_objects:
                check_quit(quit_watcher, viewer)
                print(f"Capturando: {obj.name}")
                dbg(f'Duplicando {obj.name} (handle {obj.handle})...')

                copies = sim.copyPasteObjects([obj.handle], 0)
                if not copies:
                    continue
                dup_h = copies[0]
                sim.setObjectParent(dup_h, sim.handle_world, True)
                remove_scripts_only(sim, dup_h)
                dbg(f'{obj.name}: clone handle {dup_h}')

                center_local = (obj.bbox_min + obj.bbox_max) / 2.0
                obj_matrix = np.array(sim.getObjectMatrix(obj.handle, sim.handle_world)).reshape((3, 4))
                obj_rot = obj_matrix[:, :3]
                obj_center_target = arena_center - obj_rot @ center_local
                obj_orientation = sim.getObjectOrientation(obj.handle, sim.handle_world)
                half_height = max(float(obj.bbox_max[2] - obj.bbox_min[2]) / 2.0, 0.02)
                obj_center_target[2] = floor_z + half_height

                sim.setObjectOrientation(dup_h, sim.handle_world, obj_orientation)
                sim.setObjectPosition(dup_h, sim.handle_world, obj_center_target.tolist())

                sz = obj.bbox_max - obj.bbox_min
                print(f"Geometric dimensions {obj.name}: X={sz[0]:.3f} Y={sz[1]:.3f} Z={sz[2]:.3f}")
                dbg(f'{obj.name}: estimating dimensions via depth...')
                # Depth-based size + Z refinement to keep objects floating over the virtual floor.
                depth_stats = estimate_size_with_depth(
                    sim,
                    sensor,
                    arena_center,
                    cam_z_fixed,
                    res_w,
                    res_h,
                    near_clip,
                    far_clip,
                    fx,
                    fy,
                    args.delay,
                    args.depth_band,
                    dbg,
                )
                if depth_stats:
                    print(f"Depth dimensions {obj.name}: L={depth_stats.width_m:.3f} m H={depth_stats.height_m:.3f} m")
                    if adjust_object_height_from_depth(sim, dup_h, floor_z, depth_stats, dbg):
                        dbg(f'{obj.name}: recalculating stats after Z adjustment...')
                        refreshed = estimate_size_with_depth(
                            sim,
                            sensor,
                            arena_center,
                            cam_z_fixed,
                            res_w,
                            res_h,
                            near_clip,
                            far_clip,
                            fx,
                            fy,
                            args.delay,
                            args.depth_band,
                            dbg,
                        )
                        if refreshed:
                            depth_stats = refreshed
                else:
                    print(f"Depth dimensions {obj.name}: unavailable (geometric fallback)")

                angles = np.linspace(0, 2 * math.pi, args.samples, endpoint=False)
                dbg(f'{obj.name}: starting {len(angles)} orbital samples with dynamic adjustment')
                for i, yaw in enumerate(angles):
                    check_quit(quit_watcher, viewer)
                    dbg(f'{obj.name}: sample {i}/{len(angles)} yaw={math.degrees(yaw):.1f}°')
                    optimized = optimize_camera_distance_for_pose(
                        sim,
                        sensor,
                        arena_center,
                        cam_z_fixed,
                        yaw,
                        res_w,
                        res_h,
                        near_clip,
                        far_clip,
                        fx,
                        fy,
                        args.delay,
                        args.depth_band,
                        dbg,
                        quit_watcher,
                        viewer,
                    )
                    if optimized is None:
                        dbg(f'{obj.name}: unable to optimize pose {i}')
                        continue
                    rgb, analysis = optimized
                    dbg(
                        f'{obj.name}: pose {i} final dims {analysis.width_m:.3f}x{analysis.height_m:.3f} m bbox {analysis.bbox_px}'
                    )

                    bbox_px = analysis.bbox_px
                    yolo = bbox_px_to_yolo(bbox_px, res_w, res_h)
                    safe_name = obj.name.replace('/', '_')
                    Image.fromarray(rgb).save(args.output / f"images/{safe_name}_{i:03d}.png")
                    with open(args.output / f"labels/{safe_name}_{i:03d}.txt", "w") as f:
                        f.write(
                            f"{class_map[obj.name]} {yolo[0]:.6f} {yolo[1]:.6f} {yolo[2]:.6f} {yolo[3]:.6f}\n"
                        )
                    min_x, min_y, max_x, max_y = bbox_px
                    debug_img = Image.fromarray(rgb.copy())
                    draw = ImageDraw.Draw(debug_img)
                    draw.rectangle([(min_x, min_y), (max_x, max_y)], outline=(255, 0, 0), width=2)
                    debug_path = args.output / f"debug/{safe_name}_{i:03d}.png"
                    debug_img.save(debug_path)
                    viewer.show(np.array(debug_img), f"{obj.name} pose {i}")
                    total += 1
                    dbg(f'{obj.name}: amostra {i} salva com bbox {bbox_px}')

                remove_object_tree(sim, dup_h)
                dbg(f'{obj.name}: clone removido')

            print(f"Sucesso! {total} imagens geradas.")
        except QuitRequested:
            print("Exit requested by user (q). Shutting down...")
        finally:
            viewer.stop()
            if sensor and sim:
                try:
                    sim.removeObject(sensor)
                except Exception:
                    pass
            if sim:
                try:
                    sim.stopSimulation()
                except Exception:
                    pass

if __name__ == "__main__":
    main()
