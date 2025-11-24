# Autonomous Docking

This walkthrough shows how to run the autonomous docking routine that combines the on-robot controller [`python_controller.py`](python_controller.py) with the docking helper script [`python_autodock.py`](python_autodock.py). The setup mirrors the ROS 2 bumper test but focuses on returning the robot to the charging base using the beacon sensor mounted on `/dockingSensor`.

[▶ Watch the autonomous docking](https://youtu.be/IeCzR4ks9Uo)

## Prerequisites

- CoppeliaSim (tested with 4.6+) with the Python integration enabled.
- Repository cloned locally; scene [`autodocking/lvk Evaluation scene3.0.ttt`](autodocking/lvk%20Evaluation%20scene3.0.ttt) available.
- Python support inside CoppeliaSim so that non-threaded child scripts can run `.py` files.

## Simulation Setup

1. Launch CoppeliaSim and open [`autodocking/lvk Evaluation scene3.0.ttt`](autodocking/lvk%20Evaluation%20scene3.0.ttt).
2. The scene already references `python_controller.py` as the non-threaded child script on `/myRobot` and `python_autodock.py` as the simulation-level script that handles docking. If you import the robot into a different scene, re-attach these scripts manually (drag the files onto the scene or copy/paste the script contents).
3. Start the simulation once so that the UI titled **Robot Control** appears and the Remote API signals are created.

## Using The Controller

- **Manual driving:** With the simulation running, use `W`/`S` to change forward speed, `A`/`D` to yaw left/right, and `SPACE` to stop. The UI sliders mirror the keyboard commands; buttons zero the velocities.
- **Autonomous patrol:** Check `Activate auto` in the UI to let `python_controller.py` run the random obstacle avoidance routine while maintaining odometry and battery readouts.
- **Docking mode:** Toggle the `docking` checkbox (or set the int signal `<robotHandle>Docking` to `1`) to let `python_autodock.py` take over. The script:
  - Reads the infrared beacon via `getBeaconInfo` from `/chargingBase/beacon`.
  - Maps the beacon strength to linear speed (up to `max_linear_vel / 2`) and the bearing to differential wheel speeds.
  - Declares the robot docked when the signal reaches `max_signal` (default `0.95`) and freezes the wheels.

## Searching For The Beacon

If the docking sensor loses sight of the beacon, `python_autodock.py` raises the `<robotHandle>LookingForBeacon` signal. The controller responds by enabling auto-navigation, letting the robot wander while periodically querying the beacon. When the signal returns, the docking script cancels the search and resumes the approach.

## Battery-Based Trigger

`python_autodock.py` monitors the float signal `<robotHandle>Battery`. When it drops below `25%`, the script prints `LOW BATTERY!` and forces docking mode by setting `<robotHandle>Docking = 1`. Reset the battery to `100%` to exit docking mode (`BATTERY RECHARGED!`). You can publish these signals from another script or the Remote API to simulate battery depletion.

## Tuning

Edit [`python_autodock.py`](python_autodock.py) to adjust behaviour:

- `lowBatt`: threshold that triggers docking automatically.
- `max_signal`: beacon strength that counts as “docked”.
- `max_linear_vel`: cap for the linear component while approaching the base.

Likewise, [`python_controller.py`](python_controller.py) exposes UI labels, random avoidance behaviour, and odometry logging that you can tweak for different robots or sensor layouts.
