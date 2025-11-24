# Semantic Room Segmentation Node

This ROS2 node ([`my_robot_semantic_segmentation_node.py`](./final_project_ws/src/my_robot_pkg/my_robot/my_robot_semantic_segmentation_node.py)) acts as the bridge between object detection and topological mapping. It takes raw object detections (like "oven", "bed") and assigns semantic labels (like "kitchen", "bedroom") to the specific rooms identified in the topological map.

# Overview

While a topological map tells you *where* the distinct rooms are (Room 1, Room 2), it doesn't know *what* they are. This node listens for text markers from an object detection system (e.g., YOLO), checks which room the object is located in, and permanently labels that room on a new `/semantic_map`. This allows the robot to understand commands like "Go to the Kitchen" by translating "Kitchen" into specific grid coordinates.

# Logic Steps Explanation

The node operates asynchronously using callbacks to process maps and detections as they arrive:

1.  **Initialization:**
    * Loads a JSON configuration file (e.g., `semantic_room_seg_classes.json`) that defines mappings from object labels (e.g., "oven") to room types (e.g., "kitchen").
    * Sets up subscribers for the topological map and object detection markers.

2.  **Map Cache (`_room_map_cb`):**
    * When a new topological map is received, it is cached as a NumPy array.
    * This map contains distinct IDs (e.g., 1, 2, 3) for each room.

3.  **Detection Processing (`_detections_cb`):**
    * The node iterates through incoming detection markers.
    * For each detection, it checks if the text label exists in the JSON config.
    * It calculates which room ID corresponds to the detection's coordinates.
    * It updates an internal dictionary mapping `Room ID -> Semantic Label` (e.g., Room 5 is now "Kitchen").

4.  **Semantic Map Publication (`_publish_semantic_map`):**
    * On a timer loop, the node reconstructs the map.
    * It creates a new grid where pixels corresponding to specific Room IDs are replaced with a unique "Semantic Value" (color) associated with that room type.
    * It publishes this new colored grid to `/semantic_map` and places text markers at the center of each room for visualization.

# Key Features

* **Dynamic Labeling:** Associates object detections with room IDs in real-time.
* **Semantic Map Generation:** Publishes a new `OccupancyGrid` where cell values represent specific room types.
* **JSON Configuration:** Uses a configuration file to map detected labels to room names.
* **Visual Debugging:** Publishes floating text markers (`/semantic_map_labels_markers`) in the center of recognized rooms.

# Dependencies

* `rclpy`
* `numpy`
* `geometry_msgs`, `nav_msgs`, `visualization_msgs`

# Configuration

The node relies on a JSON file to map objects to rooms. By default, it looks in `config/semantic_room_seg_classes.json`.

**Example JSON structure:**
```json
{
    "oven": "kitchen",
    "fridge": "kitchen",
    "bed": "bedroom",
    "sofa": "living_room"
}