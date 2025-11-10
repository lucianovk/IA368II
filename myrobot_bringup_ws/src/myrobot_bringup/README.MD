# MyRobot Bringup Example

`myrobot_bringup` is a self-contained ROS 2 example that demonstrates how to
decouple lifecycle management (configure/activate) from *behavioural* state for
a battery-powered mobile robot. Nav2's lifecycle manager keeps every node
configured and activated, while `behaviour_server` publishes an operational mode
that other nodes subscribe to so they can react to high-level robot behaviour.

The example intentionally keeps the motion/charging logic simple so you can
swap in real hardware interfaces while reusing the same topics, service, and
state machine.

## Building the Workspace

```bash
cd ~/myrobot_bringup_ws
colcon build --packages-select myrobot_interfaces myrobot_bringup --symlink-install
```

Tips:

- Run a clean build (`rm -rf build install log`) if dependencies change.
- `myrobot_bringup` depends on `myrobot_interfaces`, so keep both packages in
  the build list (or run `colcon build --symlink-install` to build everything).
- Keep `/opt/ros/jazzy` sourced *before* rebuilding so the toolchain is on the
  PATH.
- If your shell uses `set -u`, wrap the build environment setup like so:

  ```bash
  set +u
  source /opt/ros/jazzy/setup.bash
  source install/setup.bash
  set -u
  ```

  Add that block to a helper function in `~/.bashrc` if you want it to happen
  automatically for every new terminal.

## Node Graph at a Glance

| Node | Role | Key I/O |
| --- | --- | --- |
| `myrobot_server` | Simulated mobile base publishing telemetry. | Publishes docking beacon signal, relative angle, and `sensor_msgs/BatteryState`; subscribes to `/myRobot/cmd_vel` commands and behaviour mode. |
| `behaviour_server` | Central operational state machine. | Subscribes to battery %, docking strength, and `/myRobot/start_stop`; publishes `/myRobot/behaviour/mode`; logs every transition plus periodic battery summaries. |
| `cleaning_server` | Emits a simple cleaning twist when mode is `CLEANING`. | Watches behaviour mode, publishes `/myRobot/cmd_vel`. |
| `autodocking_server` | Drives toward the charging base when mode is `AUTODOCKING`. | Consumes docking signal topics and publishes `/myRobot/cmd_vel`. |
| `charging_server` | Stub for charge supervision logic. | Listens to battery + behaviour topics; place charge control logic here. |
| `charged_server` | Placeholder for maintenance tasks while docked. | Subscribes to behaviour mode. |
| `nav2_lifecycle_manager` | Configures and activates every node listed in `config/lifecycle_manager.yaml`. | Ensures the lifecycle state is managed independently from operational behaviour. |

Every node uses `QoSProfile(depth=1)` so transitions propagate quickly without
back pressure. All topics/services live under the `/myRobot` namespace for
clarity.

## Behaviour State Machine

`behaviour_server` owns the operational state. It initializes in `CHARGED`, then
uses battery telemetry, “near dock” signal strength, and the Start/Stop service
to decide mode transitions.

```
Start-up -> CHARGED
CHARGED --(Start command)--> CLEANING
CLEANING --(Battery <= 25%)--> AUTODOCKING
AUTODOCKING --(Dock signal strong)--> CHARGING
CHARGING --(Battery >= 99.9% & stop requested)--> CHARGED
CHARGING --(Battery >= 99.9% & no stop)--> CLEANING
CLEANING --(Stop command)--> AUTODOCKING
```

Implementation highlights:

- `BATTERY_LOW` is set to `0.25`. Crossing that threshold triggers automatic
  docking if the robot is cleaning.
- `_set_mode()` centralizes transitions and emits a log of the form
  `Behaviour mode CLEANING -> AUTODOCKING (battery low (0.24))` so you can trace
  *why* modes change.
- `_log_battery_status()` fires every 5 s to log
  `Battery status: 75.0% | mode=CLEANING | near_base=False | stop_requested=False`.
- During `on_configure()` the node logs the initial state snapshot so you know
  the starting mode and telemetry values when the application boots.

Depending on the mode, the other nodes either publish motion commands or remain
idle. Because lifecycle activation never changes, you can add actuators and
sensors to those nodes without worrying about state churn.

## Running the Bringup

```bash
cd ~/myrobot_bringup_ws
colcon build --packages-select myrobot_interfaces myrobot_bringup
set +u
source /opt/ros/jazzy/setup.bash
source install/setup.bash
set -u
ros2 launch myrobot_bringup bringup.launch.py
```

`bringup.launch.py` starts all six robot nodes plus Nav2's lifecycle manager.
Watch the console: you should see the lifecycle “configured/activated” logs
followed by the behaviour server’s initial snapshot and periodic battery data.

## Interacting with the Behaviour Server

- **Start cleaning** (from dock):

  ```bash
  cd ~/myrobot_bringup_ws
  set +u
  source /opt/ros/jazzy/setup.bash
  source install/setup.bash
  set -u
  
  ros2 service call /myRobot/start_stop myrobot_interfaces/srv/StartStop '{command: "start"}'
  ```

- **Request a stop** (forces auto docking when currently cleaning):

  ```bash
  cd ~/myrobot_bringup_ws
  set +u
  source /opt/ros/jazzy/setup.bash
  source install/setup.bash
  set -u
 
  ros2 service call /myRobot/start_stop myrobot_interfaces/srv/StartStop '{command: "stop"}'
  ```

- **Monitor logs**: `ros2 log echo /rosout` or use `grep` on the node output to
  see transition reasons and battery summaries.

- **Inspect topics**: `ros2 topic echo /myRobot/behaviour/mode` or
  `/myRobot/battery_state` to visualize the state machine working.

Because `myrobot_server` simulates the docking signal and battery level, you can
observe the entire cycle (clean → dock → charge → charged) without extra
hardware.

## Customizing for Real Robots

- Replace the synthetic battery model in `myrobot_server` with your real power
  estimation or BMS feed.
- Swap the random docking signal with perception or fiducial-based detection.
- Enhance `cleaning_server` with coverage planning, SLAM, or obstacle avoidance.
- Implement real docking controllers inside `autodocking_server`, including
  safety interlocks.
- Extend `charging_server` / `charged_server` with IO control, self-checks, or
  telemetry uploads performed only when the robot is on the dock.

Keep the topic/service names and the `Mode` enum consistent so every node stays
loosely coupled while reacting to the shared operational state published by the
behaviour server.
