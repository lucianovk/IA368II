# Semantic Room Segmentation for Autonomous Vacuum Robots

Final project for **IA368II_2025S2 – Design and Development of Intelligent Autonomous Robots**  

## 1. Goal and Motivation
The project aims to classify household rooms by combining geometric room segmentation with semantic cues extracted from a top-mounted RGB-D camera. A YOLO11-based detector identifies objects in real time; detections are then matched against a list of typical household objects to estimate which room the robot is visiting. This knowledge can drive cleaning policies, contextual navigation, or task scheduling.

Key ideas:
- Build an occupancy map with SLAM (LiDAR or RGB-D) and split it into rooms.
- Detect objects with YOLO11 on the RGB stream.
- Assign each detection to the room polygon that contains it and infer the most likely room label.
- Publish a semantic map (`/semantic_map`) and text markers for visualization in RViz.

## 2. System Architecture

| Block | Description |
| --- | --- |
| **Perception** | [`my_robot_scan_node`](./final_project_ws/src/my_robot_pkg/my_robot/my_robot_scan_node.py), [`rf2o_laser_odometry_node`](./final_project_ws/src/rf2o_laser_odometry/src/CLaserOdometry2DNode.cpp), and Cartographer maintain the occupancy grid (`/map`). |
| **Room Segmentation** | [`my_robot_topologic_segmentation_node`](./final_project_ws/src/my_robot_pkg/my_robot/my_robot_topologic_segmentation_node.py) colors the occupancy grid by room IDs (`/topologic_map`). |
| **Vision** | [`my_robot_vision_node`](./final_project_ws/src/my_robot_pkg/my_robot/my_robot_vision_node.py) and [`my_robot_detection_node`](./final_project_ws/src/my_robot_pkg/my_robot/my_robot_detection_node.py) run YOLO11 and publish detections as `MarkerArray` messages. |
| **Semantic Fusion** | [`my_robot_semantic_segmentation_node`](./final_project_ws/src/my_robot_pkg/my_robot/my_robot_semantic_segmentation_node.py) reads detections, maps each to a segmented room ID, and publishes `/semantic_map` plus `/semantic_map_labels_markers`. |
| **Exploration / Mapping** | [`my_robot_cartographer_node`](./final_project_ws/src/my_robot_pkg/my_robot/my_robot_cartographer_node.py) provides autonomous navigation with frontier exploration and safety behaviors; [`my_robot_serialize_pose_graph_node`](./final_project_ws/src/my_robot_pkg/my_robot/my_robot_serialize_pose_graph_node.py) periodically saves pose graphs. |

Additional tools:
- RViz configuration ([`my_robot.rviz`](./final_project_ws/src/my_robot_pkg/rviz/my_robot.rviz)) for monitoring detections, topologic segments, and semantic labels.
- Scene descriptions in [`scenes/`](./scenes/) for CoppeliaSim or RViz playback.
- Synthetic data helpers in [`scripts/`](./scripts/) such as `capture_scene_objects.py`, which orients the robot camera toward every object that exposes a valid 3D bounding box and saves YOLO-ready image/label pairs for retraining.

**Node deep dives**  
For implementation notes and extended explanations see:
- [Cartographer / exploration node](./cartograher_node.md)
- [Explorer control loop](./explorer_node.md)
- [Topologic segmentation node](./topologic_segmentation_node.md)
- [Semantic segmentation node](./semantic_segmentation_node.md)

## 3. Semantic Pipeline

1. **Occupancy & Room Segmentation**  

   ![Topologic segmentation](./screenshots/topologic_room_segmentation.png)

   The SLAM stack generates a 2D occupancy grid. [`my_robot_topologic_segmentation_node`](./final_project_ws/src/my_robot_pkg/my_robot/my_robot_topologic_segmentation_node.py) repairs walls, extracts a distance transform, seeds local maxima, and runs a watershed/Voronoi segmentation to assign consistent room IDs.  
   
   
2. **Object Detection**  

   ![Bathroom example](./screenshots/toilet.jpg)
   
   [`my_robot_detection_node`](./final_project_ws/src/my_robot_pkg/my_robot/my_robot_detection_node.py) loads the local YOLO11x checkpoint (`final_project_ws/models/yolo11x.pt`), subscribes to RGB and depth frames, triangulates 3D centroids with the TF tree, and publishes persistent detections as `/detections_markers` plus saved crops under `detections/`. Each crop is rendered with the label and confidence, depth is derived from the closest valid measurement inside the box, and detections are accepted only if their label exists in [`semantic_room_seg_classes.json`](./final_project_ws/src/my_robot_pkg/config/semantic_room_seg_classes.json) and pass the configurable `confidence_threshold` parameter (default `0.7`).  

3. **Semantic Assignment**  

   ![Semantic room label coloring](./screenshots/semantic_room_segmentation.png)

   [`my_robot_semantic_segmentation_node`](./final_project_ws/src/my_robot_pkg/my_robot/my_robot_semantic_segmentation_node.py) subscribes to `/topologic_map` and `/detections_markers`, normalizes detections via [`config/semantic_room_seg_classes.json`](./final_project_ws/src/my_robot_pkg/config/semantic_room_seg_classes.json), assigns the dominant room label per segmented polygon, and republishes both `/semantic_map` and `/semantic_map_labels_markers`.  

## 4. Repository Layout

```
final_project/
├── final_project_ws/         # ROS 2 workspace (src/, build/, install/)
│   ├── src/my_robot_pkg/     # Package with nodes, launch files, and configs
│   ├── src/rf2o_laser_odometry/
│   └── ...
├── screenshots/              # Figures used in this README
├── scenes/                   # Coppelia scenes
└── scripts/                  # Helper scripts (generate_detection_report)
```

Important paths:
- [`final_project_ws/src/my_robot_pkg/my_robot/`](./final_project_ws/src/my_robot_pkg/my_robot/) – Python nodes (vision, detection, semantic fusion, exploration).
- [`final_project_ws/src/my_robot_pkg/launch/`](./final_project_ws/src/my_robot_pkg/launch/) – Launch files for mapping and localization.
- [`final_project_ws/src/my_robot_pkg/config/semantic_room_seg_classes.json`](./final_project_ws/src/my_robot_pkg/config/semantic_room_seg_classes.json) – room label lookup.

## 5. Software Stack & Dependencies

- **ROS 2 Jazzy (recommended)** with nav/visualization packages.
- `slam_toolbox`, `rf2o_laser_odometry`.
- `scipy`, `ultralytics` (for YOLO11).
- GPU (optional) for faster inference; CPU mode works for smaller models.

Install base dependencies:
```bash
sudo apt update
sudo apt install ros-jazzy-slam-toolbox
```

## 6. Building the Workspace

```bash
cd final_project/final_project_ws
colcon build --symlink-install
source install/setup.bash
```

If you frequently switch terminals, call `source final_project_ws/install/setup.bash` after every build.

## 7. Running the System

### 7.1 Mapping Session (create a new map)
```bash
cd final_project
source final_project_ws/install/setup.bash
ros2 launch my_robot_pkg my_robot_mapping.launch.py
```
- Starts RF2O odometry, LiDAR scan node, command-velocity node, Cartographer, semantic modules, and `slam_toolbox` (delayed to allow TF start-up).
- `my_robot_cartographer_node` explores automatically; monitor `/planned_path`, `/followed_path`, `/semantic_map`.
- Pose graphs are serialized by `my_robot_serialize_pose_graph_node` for later optimization.

### 7.2 Localization + Semantic Labeling on an Existing Map
```bash
cd final_project
source final_project_ws/install/setup.bash
ros2 launch my_robot_pkg my_robot_localize.launch.py
```
- Brings up the same perception stack but without pose graph serialization.
- Suitable for experiments where a prebuilt map is loaded (from `map/` or `pose_graphs/`).

### 7.3 Visualizing in RViz
```bash
rviz2 -d my_robot.rviz
```
Topics of interest: `/map`, `/topologic_map`, `/semantic_map`, `/detections_markers`, `/semantic_map_labels_markers`.

## 8. References

1. Ye, Y. et al., “InteriorVerse: Multimodal Indoor Scene Understanding,” [arXiv:2403.12920](https://arxiv.org/abs/2403.12920).  
2. SpatialVerse – InteriorGS dataset, [Hugging Face](https://huggingface.co/datasets/SpatialVerse/InteriorVerse).  
3. K. Okada et al., “Room Recognition in 2D Maps,” [ICRA 2016](https://ieeexplore.ieee.org/document/7487532).  
4. ROS Wiki – [ipa_room_segmentation](https://wiki.ros.org/ipa_room_segmentation).  
5. ROS Wiki – [rose2](https://wiki.ros.org/rose2).  
6. IPA 320 – [ipa_coverage_planning](https://github.com/ipa320/ipa_coverage_planning).
