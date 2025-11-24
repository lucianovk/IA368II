# Robot Coverage Node (Full Area Visit)

**Note:** Although the file is named [`my_robot_explorer_node.py`](./final_project_ws/src/my_robot_pkg/my_robot/my_robot_explorer_node.py), the logic inside implements a **Coverage Algorithm** (`MyRobotCoverage` class). Unlike a standard explorer that seeks *unknown* space, this node seeks *known but unvisited* space.

# Overview

This node is designed for tasks like cleaning, painting, or inspection where the entire floor plan must be traversed. It assumes the map is already known (or being built) and ensures the robot physically passes over every accessible free cell in the environment. It maintains an internal "Visited Grid" and uses path planning to systematically visit unvisited areas.

# Logic Steps Explanation

The node runs a control loop (`control_loop`) that manages a Finite State Machine (FSM):

1.  **Visited Marking:**
    * In every cycle, the robot projects its footprint (plus a `COVERAGE_RADIUS`) onto an internal "Visited Grid".
    * Cells under the robot are marked with a value of 100 (Visited).

2.  **Target Selection:**
    * The node scans the map for "Candidates": cells that are **Free** (in the static map), **Safe** (not inside inflation zones), and **Unvisited** (0 in the visited grid).
    * It creates a list of these candidates.

3.  **Path Planning (A*):**
    * The candidates are sorted by distance from the robot.
    * The node attempts to plan a path to the nearest candidate using the A* algorithm.
    * If the nearest candidate is unreachable (e.g., blocked by a temporary obstacle), it retries with the next nearest target.

4.  **Movement & Recovery:**
    * **Moving:** The robot follows the path. It monitors LiDAR data for dynamic obstacles.
    * **Avoiding:** If an obstacle appears too close, the robot stops, enters "Avoiding" mode, and rotates in place to find a clear exit angle before replanning.
    * **Rescue:** If the robot finds itself inside an inflation zone (unsafe area), it prioritizes finding the nearest "safe" cell and escaping before resuming coverage.

# Key Features

* **Visited Map Tracking:** Creates a secondary grid that paints "visited" cells as the robot moves.
* **Coverage Path Planning:** Uses A* to find paths to free cells that have not yet been visited.
* **Rescue Behavior:** Automatically maneuvers out of inflation zones if the robot gets stuck.
* **Safety & Hysteresis:** Includes "Rotate-Then-Move" logic and LiDAR collision avoidance.

# Dependencies

* `rclpy`
* `numpy`
* `scipy` (for `binary_dilation` and `distance_transform_edt`)
* `nav_msgs`, `geometry_msgs`, `sensor_msgs`