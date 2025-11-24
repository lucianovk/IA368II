# My Robot Cartographer Node

This ROS2 node ([`my_robot_cartographer_node.py`](./final_project_ws/src/my_robot_pkg/my_robot/my_robot_cartographer_node.py)) implements an autonomous exploration algorithm designed to map unknown environments. It combines frontier-based exploration with robust collision avoidance and path planning.

# Overview

The primary goal of this node is to construct a complete map of an unknown area without human intervention. It works by identifying "Frontiers"—the boundaries between known free space and unexplored space. The robot iteratively travels to these frontiers, using its sensors to "push back" the unknown areas until the entire accessible environment is mapped. It handles dynamic obstacles (via LiDAR) and static obstacles (via the map) simultaneously.

# Logic Steps Explanation

The node operates using a Finite State Machine (FSM) managed within the `control_loop`. The main logical flow is:

1.  **Map Processing & Inflation:**
    * Upon receiving a new map from the SLAM system, the node creates an "Inflated Map". It expands obstacles by a safety radius (`INFLATION_RADIUS`) to ensure the robot never plans a path too close to walls.
    * It also generates a "Cost Map" based on distance to the nearest wall, which penalizes paths that hug obstacles too closely, encouraging safer, central routes.

2.  **Frontier Detection:**
    * The node scans the map to find edges where "Free Space" pixels touch "Unknown Space" pixels.
    * These edges are grouped into frontiers. The nearest reachable frontier is selected as the next target.

3.  **Path Planning (A*):**
    * A custom A* (A-Star) algorithm calculates the optimal path from the robot's current position to the selected frontier.
    * The planner uses the cost map to avoid high-risk areas. If the robot is stuck inside an inflation zone (e.g., after a SLAM correction), it triggers a special "Rescue" mode to find the nearest safe cell and escape.

4.  **Navigation & Avoidance:**
    * **Moving:** The robot follows the planned path.
    * **Avoiding:** If the LiDAR detects an obstacle closer than `COLLISION_DIST_NORMAL` (even if the map says it's clear), the robot stops immediately. It enters an "Avoiding" state, rotates in place to find the widest clear opening, and only resumes movement once aligned with a safe direction.

# Key Features

* **Frontier Exploration:** Automatically seeks out unknown areas to maximize map coverage.
* **Safety Inflation:** Mathematically expands obstacles to create a "no-go" buffer zone around walls.
* **Reactive Avoidance:** Uses real-time LiDAR data to prevent collisions with dynamic objects (people, pets) that aren't on the static map.
* **Rotate-Then-Move:** Ensures precise movement by forcing the robot to rotate in place until aligned before driving forward.
* **Rescue Maneuver:** Capable of self-recovery if the robot "spawns" or gets trapped inside a restricted area.

# Dependencies

* `rclpy`
* `numpy`
* `scipy` (for `binary_dilation` and `distance_transform_edt`)
* `geometry_msgs`, `nav_msgs`, `sensor_msgs`

# Configuration

The node has several hardcoded constants at the top of the file that define the robot's physical constraints:

* `ROBOT_RADIUS`: 0.16m
* `INFLATION_RADIUS`: 0.25m
* `SCAN_FOV_DEG`: 60 degrees (Frontal sector monitored for collisions)