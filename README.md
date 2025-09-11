# Vacuum Cleaner Robot Simulation (CoppeliaSim + Python) 

![Spiral Coverage](lvk_navigation.png)

This repository contains three Python scripts for simulating a **vacuum cleaner robot** in **CoppeliaSim**.  
All scripts support **manual or autonomous navigation** patterns commonly found in household robots and include odometry-based updates.

1. **Goal-based navigation with reactive obstacle avoidance** 
2. **Pure reactive obstacle avoidance** (randomized turns)  
3. **Systematic area coverage using spiral waypoints**

---

## 1. Goal-Based Navigation + Reactive Obstacle Avoidance (with Temporary Avoidance Goals)

Scene file: `myRobot.ttt`

Watch on YouTube:

[![Watch on YouTube](https://img.youtube.com/vi/bmU25OcMK54/hqdefault.jpg)](https://youtu.be/bmU25OcMK54)

**Script focus:** navigate to a list of randomly placed goals, avoiding obstacles by inserting **temporary local avoidance goals** ahead/aside of the robot, then resuming the original goal list.

### Algorithmic Behavior
- **Go-to-point controller (`go_to_point`)**  
  - Computes distance and heading error to the target.  
  - **Large heading error (> 90°)** → pure rotation in place with capped angular velocity; tiny forward bias to avoid stalling.  
  - **Moderate error (5°–90°)** → proportional angular correction with clipped ω and reduced linear speed (uses `cos(error)` to taper).  
  - **Small error (< 5°)** → straight-line motion at nominal `v` with **no** angular correction to minimize drift.  
  - Converts (v, ω) into per-wheel angular speeds with saturation and applies them to motors.  
  - Declares **arrival** when within a tolerance radius and stops both wheels.

  See details: [go_to_point controller](go_to_point.md)

- **Obstacle handling (proximity sensor)**  
  - On detection, the robot **creates a temporary avoidance goal** in front, offset laterally (left/right) and slightly beyond the obstacle distance.  
  - If already avoiding and a new obstacle is seen, it **removes the current goal** and keeps the lateral side consistent to prevent oscillation.  
  - After reaching the avoidance goal, it resumes the normal goal list.

- **Goal management (`GoalManager`)**  
  - Maintains an ordered list of goals with operations: get current, advance to next, insert before current, remove current, reset.  
  - Ends the simulation when all goals are completed.

- **Utilities**  
  - `angle_to_goal`: computes goal/robot angles and normalized error in CoppeliaSim’s frame (yaw=0 → +Y).  
  
<p align="center">
  <img src="angle_to_goal.png" alt="Angle to Goal" width="360" />
  
</p>

- `local_to_world`: transforms offsets from robot frame to world frame.  
  - Random **obstacle generation** with non-overlapping cuboids (AABB checks), color/props set for detection/collision.

### Flow-Style Outline
1. **Initialization**  
   - Load robot, motor, sensor, and goal-marker handles.  
   - Spawn **N obstacles** randomly without overlap.  
   - Generate **N goals** uniformly in workspace; initialize `GoalManager`.  
   - Set current goal and place a visible goal marker.

2. **Control Loop (Actuation)**  
   - Read proximity sensor → `(detected, distance, …)`.  
   - **If obstacle detected**:  
     - If already avoiding → remove current goal; choose a **consistent lateral offset**.  
     - Else → choose random side (left/right).  
     - Compute **avoidance waypoint** in front+lateral using `local_to_world` and **insert before current**.  
     - Set current goal to the avoidance waypoint; mark “avoidance started”.  
   - Call `go_to_point` to drive toward `current_goal`.  
   - **On arrival**:  
     - If it was avoidance → clear avoidance flag; resume original goal.  
     - Else → log “goal reached”.  
     - Advance to next goal (if any), update goal marker; otherwise **stop simulation**.

3. **Sensing & Update**  
   - (Optional) Add odometry/trail and periodic logging.

4. **Cleanup**  
   - Remove generated objects if needed; stop simulation gracefully.

---


## 2. Vacuum Cleaner Robot — Obstacle Avoidance Mode

Scene file: `lvk_obstacle_avoidance.ttt`

Watch on YouTube:

[![Watch on YouTube](https://img.youtube.com/vi/FzbglLwmXV4/hqdefault.jpg)](https://youtu.be/FzbglLwmXV4)

### Algorithmic Behavior
The robot mimics the behavior of a simple household vacuum cleaner:

- **Manual mode**:  
  - User controls the robot with the keyboard (W/S = forward/backward, A/D = turn, SPACE = stop, Q = quit) or via a small UI panel with sliders for velocities.  
  - Commands are applied through differential-drive kinematics.  

- **Autonomous mode**:  
  - The robot performs **randomized obstacle avoidance**, a typical strategy in low-cost robotic vacuum cleaners.  
  - Sensors:  
    - **Proximity sensor** (detects walls/furniture).  
    - **Step sensors** (detect stairs/cliffs).  
    - **Bumper** (detects physical collisions).  
  - When an obstacle is detected:  
    - The robot stops forward motion.  
    - Executes either a random turn (45°–180°) or a forced 180° turn if a step is detected.  
    - Resumes straight-line cleaning when clear.  
  - Timeout logic prevents endless turning loops.  

Additional features:  
- **Odometry trail** (red) showing estimated path.  
- **Pose logging** (x, y, yaw) for debugging.  

### Flow-Style Outline
1. **Initialization**  
   - Load robot and sensors.  
   - Create UI and reset odometry.  

2. **Control Loop**  
   - **If manual** → follow user inputs.  
   - **If autonomous**:  
     - Sense obstacles.  
     - If none → move forward at max speed.  
     - If detected → stop and perform avoidance turn.  

3. **Sensing & Update**  
   - Integrate odometry from wheel encoders.  
   - Periodically log pose.  
   - Optionally draw odometry trail.  

4. **Cleanup**  
   - Remove UI and drawings.  

---

## 3. Vacuum Cleaner Robot — Spiral Coverage Mode

Scene file: `lvk_navigation.ttt`

Watch on YouTube:

[![Watch on YouTube](https://img.youtube.com/vi/jaj2KY_krco/hqdefault.jpg)](https://youtu.be/jaj2KY_krco)

### Algorithmic Behavior
This script models a more systematic vacuum cleaner with **coverage navigation** using **spiral waypoints**:

- **Manual mode**:  
  - Same as above (keyboard or UI-based control).  

- **Autonomous mode**:  
  - Generates waypoints along an **Archimedean spiral** to cover the cleaning area.  
  - Navigation to each waypoint uses a **go-to-point controller**:  
    - Large heading error → rotate in place until aligned.  
    - Moderate error → proportional angular correction with reduced forward motion.  
    - Small error → move straight at nominal speed.  
  - Each waypoint is marked **completed** once reached within tolerance.  
  - If stuck too long, the waypoint is skipped, and the robot proceeds with the next.  

Additional features:  
- **Odometry trail** (red) and **ground-truth trail** (blue).  
- **Waypoint markers**: green (pending) and black (completed).  
- **Periodic logs** for current pose and goals.  

### Flow-Style Outline
1. **Initialization**  
   - Generate spiral waypoints (CW or CCW).  
   - Build UI and draw waypoint markers.  

2. **Control Loop**  
   - **If manual** → apply user inputs.  
   - **If autonomous**:  
     - Select next spiral waypoint.  
     - Compute heading and distance.  
     - Decide motion:  
       - Rotate, correct, or move straight depending on error.  
     - Mark waypoint as reached if within tolerance.  
     - Skip if stuck for too long.  

3. **Sensing & Update**  
   - Update odometry from encoders.  
   - Draw trails (red odometry, blue ground truth).  
   - Update waypoint markers (green → black).  
   - Log pose and goal progress.  

4. **Cleanup**  
   - Remove UI, markers, and trails.  

---
