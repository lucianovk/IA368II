# Topological Segmentation Node

This script ([`my_robot_topologic_segmentation_node.py`](./final_project_ws/src/my_robot_pkg/my_robot/my_robot_topologic_segmentation_node.py)) uses computer vision techniques to segment a standard OccupancyGrid map into distinct rooms (Topological Map). It identifies structure using a Watershed algorithm based on distance transforms.

# Overview

This node transforms a metric map (walls and pixels) into a semantic-ready map (Room 1, Room 2, Corridor). It essentially "colors in" the rooms of a floor plan so that other nodes (like the semantic segmenter) can refer to specific regions. It supports both **Online** mode (subscribing to a live topic) and **Offline** mode (processing a file).

# Logic Steps Explanation

The segmentation pipeline processes the map in sequential stages:

1.  **Wall Repair (Pre-processing):**
    * The raw map is binarized.
    * Morphological closing (dilation followed by erosion) is applied to close small gaps in walls. This ensures rooms are "watertight" so the segmentation algorithm doesn't leak between rooms through open doors.

2.  **Potential Field Generation:**
    * A **Distance Transform** is calculated for every free pixel, representing its distance to the nearest wall.
    * This field is smoothed using a Gaussian Blur. The result is a "topographic" map where the center of large rooms form "peaks" (high values).

3.  **Seed Detection:**
    * The algorithm detects local maxima (peaks) in the potential field.
    * These peaks act as "seeds" or "markers" for the center of each room.

4.  **Watershed Segmentation:**
    * The **Watershed Algorithm** is applied. It "floods" the map starting from the seeds, expanding outwards until the "water" from different seeds meets at the narrow points (doorways).
    * These meeting points form the boundaries between rooms.

5.  **Filtering & Merging:**
    * Tiny regions (noise) that are smaller than `min_room_area_m2` are identified.
    * These small regions are merged into their largest neighboring room to ensure a clean final map.

# Key Features

* **Watershed Segmentation:** Robustly separates rooms based on geometry.
* **Wall Repair:** Automatically closes gaps to prevent segmentation leaks.
* **Room Merging:** Cleans up noise by merging small artifacts.
* **Dual Mode:** Runs as a ROS node or a standalone CLI script.

# Dependencies

* `rclpy`
* `numpy`
* `opencv-python` (`cv2`)
* `scikit-image` (for `peak_local_max`)
* `PyYAML`

# Parameters (Tunable)

| Parameter | Default | Description |
| :--- | :--- | :--- |
| `min_room_area_m2` | 3.0 | Minimum area for a room. Smaller regions are merged. |
| `wall_fix_kernel_size` | 3 | Size of the kernel used to close gaps in walls. |
| `potential_blur_sigma` | 2.0 | Smoothing factor for the distance field. |
| `min_room_distance_px` | 15 | Minimum distance between room centers (seeds). |