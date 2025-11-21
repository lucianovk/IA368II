import argparse
import logging
from collections import deque
import os

import cv2
import numpy as np
import yaml


def load_slam_map(pgm_path, yaml_path):
    """Load PGM + YAML map from SLAM Toolbox or ROS."""
    # --- Load YAML ---
    with open(yaml_path, 'r') as f:
        y = yaml.safe_load(f)

    resolution = float(y["resolution"])     # meters/pixel
    origin = y["origin"]                    # [x, y, yaw]
    negate = int(y.get("negate", 0))
    occ_th = y.get("occupied_thresh", 0.65)
    free_th = y.get("free_thresh", 0.196)

    # --- Load PGM (8-bit grayscale) ---
    img = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise RuntimeError("Could not read PGM map.")

    return img, resolution, occ_th, free_th, negate, origin


def pgm_to_occupancy(img, occ_th=0.65, free_th=0.196, negate=0):
    """
    Convert ROS/SLAM toolbox PGM into occupancy grid:
      - 0   = free
      - 100 = occupied
      - -1  = unknown
    """
    # Map server saves pixel = round((1 - occ_prob) * 255) when negate=0
    # so high values are free. When negate=1, it is inverted.

    norm = img.astype(np.float32) / 255.0
    occ = np.full(img.shape, -1, dtype=np.int8)

    if negate == 0:
        occ_mask = norm <= (1.0 - occ_th)   # darker → occupied
        free_mask = norm >= (1.0 - free_th)  # lighter → free
    else:
        occ_mask = norm >= occ_th            # inverted map
        free_mask = norm <= free_th

    occ[occ_mask] = 100
    occ[free_mask] = 0

    return occ


def segment_rooms_from_slam_map(
    pgm_path,
    yaml_path,
    door_width_m=0.70,    # typical 70 cm door
    min_room_area_px=100,
    output_path="rooms_from_slam.png",
    debug_dir="debug_outputs",
    door_dilate_iters=2,
):
    # --- Load map ---
    img, resolution, occ_th, free_th, negate, origin = load_slam_map(pgm_path, yaml_path)

    # --- Convert to occupancy grid ---
    occ = pgm_to_occupancy(img, occ_th, free_th, negate)

    # --- Wall mask ---
    # occupied (100) OR unknown (-1) = wall/obstacle
    walls = ((occ == 100) | (occ == -1)).astype(np.uint8) * 255
    # Ensure the map border is treated as wall to prevent outside leakage
    walls[0, :] = 255
    walls[-1, :] = 255
    walls[:, 0] = 255
    walls[:, -1] = 255

    # --- Convert door width (meters) → pixels ---
    door_width_px = int(door_width_m / resolution)
    door_width_px = max(3, min(door_width_px, 60))  # safety clamp

    # --- Close doors: dilate walls by door width ---
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                  (door_width_px, door_width_px))
    walls_closed = cv2.dilate(walls, k, iterations=door_dilate_iters)

    # --- Free space (in binary) ---
    free = cv2.bitwise_not(walls_closed)
    _, free_bin = cv2.threshold(free, 127, 255, cv2.THRESH_BINARY)

    # --- Connected components ---
    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(
        free_bin, connectivity=4
    )

    h, w = labels.shape

    # --- Remove outside region (touching borders) ---
    border = set(np.unique(labels[0, :])) | set(np.unique(labels[-1, :])) | \
             set(np.unique(labels[:, 0])) | set(np.unique(labels[:, -1]))

    # --- Seed labels for accepted rooms ---
    vis = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    rng = np.random.default_rng(10)

    room_count = 0
    seed_labels = np.zeros_like(labels, dtype=np.int32)
    colors = {}

    for lab in range(1, num_labels):
        if lab in border:
            continue
        if stats[lab, cv2.CC_STAT_AREA] < min_room_area_px:
            continue

        color = rng.integers(0, 255, size=3, dtype=np.uint8)
        seed_labels[labels == lab] = lab
        colors[lab] = color
        room_count += 1

    # --- Propagate labels to all original free pixels ---
    prop_labels = seed_labels.copy()
    free_mask = (occ == 0)
    q = deque(zip(*np.nonzero(seed_labels)))
    neighbors = ((1, 0), (-1, 0), (0, 1), (0, -1))

    while q:
        y, x = q.popleft()
        lab = prop_labels[y, x]
        for dy, dx in neighbors:
            ny, nx = y + dy, x + dx
            if 0 <= ny < h and 0 <= nx < w and free_mask[ny, nx] and prop_labels[ny, nx] == 0:
                prop_labels[ny, nx] = lab
                q.append((ny, nx))

    for lab, color in colors.items():
        vis[prop_labels == lab] = color

    cv2.imwrite(output_path, vis)
    logging.info("Detected %d rooms", room_count)
    logging.info("Saved segmented rooms to %s", output_path)
    logging.info("Door width: %.2f m (%d px), dilation iterations: %d", door_width_m, door_width_px, door_dilate_iters)

    if debug_dir:
        os.makedirs(debug_dir, exist_ok=True)
        cv2.imwrite(os.path.join(debug_dir, "01_input_map.png"), img)
        cv2.imwrite(os.path.join(debug_dir, "02_walls_mask.png"), walls)
        cv2.imwrite(os.path.join(debug_dir, "03_walls_closed.png"), walls_closed)
        cv2.imwrite(os.path.join(debug_dir, "04_free_binary.png"), free_bin)
        cv2.imwrite(os.path.join(debug_dir, "05_labels_visualization.png"), vis)
        logging.info("Saved debug layers to %s", debug_dir)

    return labels, vis, resolution, origin


def main():
    logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")

    parser = argparse.ArgumentParser(description="Segment rooms from a SLAM map (PGM + YAML)")
    parser.add_argument("--map", required=True, help="Path to the ROS map YAML file")
    parser.add_argument(
        "--image",
        required=False,
        help="Optional override for the PGM image path. If omitted, uses the 'image' field from the YAML",
    )
    parser.add_argument("--door-width", type=float, default=0.70, help="Door width in meters")
    parser.add_argument("--min-room-area", type=int, default=100, help="Minimum room area in pixels")
    parser.add_argument(
        "--output",
        default="rooms_from_slam.png",
        help="Output image path for the segmented rooms visualization",
    )
    parser.add_argument(
        "--debug-dir",
        default="debug_outputs",
        help="Directory to save intermediate images (walls, dilation, free space, labels)",
    )
    parser.add_argument(
        "--door-dilate-iters",
        type=int,
        default=2,
        help="Number of dilation iterations applied to walls to close doors",
    )

    args = parser.parse_args()

    yaml_path = args.map
    with open(yaml_path, "r") as f:
        yaml_data = yaml.safe_load(f)

    pgm_path = args.image or yaml_data.get("image")
    if not pgm_path:
        raise ValueError("Could not determine PGM image path. Provide --image or ensure YAML has an 'image' field.")

    # If using the YAML-provided image path and it's relative, resolve it against the YAML directory
    if not args.image and not os.path.isabs(pgm_path):
        pgm_path = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), pgm_path)

    segment_rooms_from_slam_map(
        pgm_path=pgm_path,
        yaml_path=yaml_path,
        door_width_m=args.door_width,
        min_room_area_px=args.min_room_area,
        output_path=args.output,
        debug_dir=args.debug_dir,
        door_dilate_iters=args.door_dilate_iters,
    )


if __name__ == "__main__":
    main()
