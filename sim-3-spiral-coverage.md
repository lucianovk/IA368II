# Vacuum Cleaner Robot — Spiral Coverage Mode

Scene file: `lvk_navigation.ttt`

Watch on YouTube:

[![Watch on YouTube](https://img.youtube.com/vi/jaj2KY_krco/hqdefault.jpg)](https://youtu.be/jaj2KY_krco)

## Algorithmic Behavior
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

## Execution Flow
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
