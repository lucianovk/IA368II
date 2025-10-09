# ROS 2 Autonomous Docking

[autodocking.py](autodocking.py) implements a single ROS 2 node that drives a mobile base until its
battery depletes, then guides it back to a charging dock using a simulated beacon.
The node is written in Python with `rclpy` and is intended to run together with the
CoppeliaSim scenes under `autodocking/` in this repository.

[▶ Watch the ROS2 autonomous docking](https://youtu.be/v7VR-wGiVs4)

## Features

- Random-walk “AUTO” mode with bumper escape behaviour while the battery is healthy.
- Beacon-based docking controller that aligns on the beacon angle and strength.
- Charging wait state that keeps the robot still until the battery recovers.
- Published docking state flag for UI panels or monitoring tools.

## Runtime Interfaces

| Direction | Topic | Type | Purpose |
|-----------|-------|------|---------|
| publish   | `/myRobot/cmd_vel` | `geometry_msgs/Twist`  | Wheel command for the base. |
| publish   | `/myRobot/docking_mode` | `std_msgs/Int32` | `0` for AUTO, `1` for DOCKING. |
| subscribe | `/myRobot/charging_base/strengthSignal` | `std_msgs/Float32` | Beacon strength (0–1). |
| subscribe | `/myRobot/charging_base/relativeAngle` | `std_msgs/Float32` | Beacon bearing in radians. |
| subscribe | `/myRobot/battery_state` | `sensor_msgs/BatteryState` | Battery percentage & status. |
| subscribe | `/myRobot/bumper` | `geometry_msgs/Wrench` | Contact forces from the bumper. |

> The node assumes the topics follow the same naming scheme as the provided
> CoppeliaSim scenes. Adjust the strings if your robot uses different namespace
> prefixes.

## Running the Node

1. Source your ROS 2 environment, e.g. `source /opt/ros/humble/setup.bash`.
2. Launch the CoppeliaSim scene `autodocking/lvk Evaluation scene3.0.ttt` so the
   simulated robot publishes the topics listed above.
3. In another terminal, start the node:

   ```bash
   ros2 run <your_package_name> autodocking
   ```

   If you have not turned the script into an installable ROS 2 package yet,
   you can also execute it directly:

   ```bash
   python3 ros2-autodocking/autodocking.py
   ```

4. Watch the console logs for state transitions (`AUTO`, `DOCKING`, `SEARCH`,
   `APPROACH`, `WAIT_FOR_CHARGE`) and adjust the simulation UI accordingly.

## Behaviour Overview

The controller toggles between two high-level modes:

- **AUTO** — Robot performs a random walk, refreshing its twist every 5 s and
  backing up when the bumper detects a collision.
- **DOCKING** — A finite-state machine progresses through:
  - `SEARCH`: reuse AUTO behaviour until the beacon signal is received.
  - `APPROACH`: steer toward the beacon using proportional control on angle and
    linear speed reduced as the signal strengthens.
  - `WAIT_FOR_CHARGE`: hold still while monitoring battery percentage until it
    reaches the “full” threshold.

The battery callback triggers the transition to DOCKING when the percentage
drops below `low_battery_threshold` (default `80%`). Once docked, the node
returns to AUTO after the battery percentage reaches `full_battery_threshold`
(default `100%`).

## Tuning

The following constants near the top of `AutoDockingNode.__init__` control the
behaviour:

- `low_battery_threshold` / `full_battery_threshold`
- `max_linear_vel` for auto cruising
- `dock_strength_threshold` (default `0.95`) used to decide when the beacon
  indicates the robot has reached the dock
- `bump_threshold`, `control_timeout`, `signal_timeout` for sensing hysteresis

Edit the script or expose ROS parameters if you need to adjust these at runtime.

## Testing Tips

- Publish synthetic `BatteryState` messages to `/myRobot/battery_state` to force
  AUTO↔DOCKING transitions without waiting for the simulation UI.
- Temporarily spoof beacon data with `ros2 topic pub` to verify the docking
  finite-state machine outside CoppeliaSim.
- Monitor `/myRobot/docking_mode` to drive dashboards or integrate with other
  autonomous behaviours.
