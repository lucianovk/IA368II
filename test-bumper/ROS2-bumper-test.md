# ROS 2 Bumper Test

This guide shows how to run the bumper recovery example that integrates ROS 2 with CoppeliaSim using the [`coppeliasim_zmqremoteapi_client`](https://github.com/CoppeliaRobotics/zmqRemoteApi) bridge together with the accompanying Python scripts [`remoteAPI-ROS2-bridge.py`](remoteAPI-ROS2-bridge.py) and [`test-bumper.py`](test-bumper.py).

![Bumper Test Setup](test-bumper.png)

[▶ Watch the bumper test](https://youtu.be/xgwVoYoOphQ)

## Prerequisites

- CoppeliaSim with the Remote API server enabled
- ROS 2 (tested with Jazzy) on the host machine
- Python packages: `rclpy`, `geometry_msgs`, `coppeliasim_zmqremoteapi_client`

## Simulation Setup

1. Open CoppeliaSim and load the evaluation scene `Evaluation scene3.2_students.ttt` (available through the AI368II Moodle resources).
2. Start simulation once so the Remote API server advertises the available objects. The script `remoteAPI-ROS2-bridge.py` will automatically launch the simulation when it connects.

## Launching the Bridge

Run the bridge [`remoteAPI-ROS2-bridge.py`](remoteAPI-ROS2-bridge.py) that relays joint commands and sensor data between CoppeliaSim and ROS 2:

```bash
python3 test-bumper/remoteAPI-ROS2-bridge.py
```

This script connects through ZMQ, reads bumper forces, and updates ROS 2 topics.

## Running the ROS 2 Nodes

In a separate terminal sourced for ROS 2, start the control node [`test-bumper.py`](test-bumper.py):

```bash
ros2 run test_bumper test_bumper.py
```

The node publishes velocity commands and monitors bumper forces. When a collision is detected, it triggers a finite state machine that stops, drives forward briefly, and then resumes backward motion. Console logs report state transitions together with the maximum bumper force component.

## Tuning

You can adjust responsiveness by editing `test-bumper/test-bumper.py`:

- `force_threshold`: minimum force before a collision is declared.
- `stop_time`: how long to hold position after a hit.
- `forward_time`: duration of the forward recovery maneuver.
