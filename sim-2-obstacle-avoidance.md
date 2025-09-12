# Vacuum Cleaner Robot — Obstacle Avoidance Mode

Scene file: `lvk_obstacle_avoidance.ttt`
Python script: [python_controller-child-lvk_obstacle_avoidance.py](python_controller-child-lvk_obstacle_avoidance.py)

Watch on YouTube:

[![Watch on YouTube](https://img.youtube.com/vi/FzbglLwmXV4/hqdefault.jpg)](https://youtu.be/FzbglLwmXV4)

## Algorithmic Behavior
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

## Execution Flow
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
