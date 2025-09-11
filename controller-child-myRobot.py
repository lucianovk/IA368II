import math
import random

def go_to_point(sim, robot_handle, left_motor_handle, right_motor_handle,
                goal, v=0.1, L=0.2, d=0.1, tolerance=0.04):
    """
    Move the differential robot to the 'goal' point.
    Straight line movement when angular error is small.
    """
    r = d / 2.0

    pos = sim.getObjectPosition(robot_handle, -1)
    x, y = pos[0], pos[1]
    
    dx_goal = goal[0] - x
    dy_goal = goal[1] - y
    dist = math.hypot(dx_goal, dy_goal)

    if dist < tolerance:
        sim.setJointTargetVelocity(left_motor_handle, 0)
        sim.setJointTargetVelocity(right_motor_handle, 0)
        return True

    # Angular error
    theta_goal_rad, theta_robot_rad, angle_error_rad = angle_to_goal(sim, robot_handle, goal)
    theta_goal_deg = math.degrees(theta_goal_rad)
    theta_robot_deg = math.degrees(theta_robot_rad)
    angle_error_deg = math.degrees(angle_error_rad)
    
    abs_error = abs(angle_error_rad)
    
    if abs_error > math.radians(90):  # > 90°: Pure rotation
        omega_max = 3.0
        #Return the value of the first parameter and the sign of the second parameter
        omega = omega_max * math.copysign(1, angle_error_rad)
        v_linear = 0.01
        
    elif abs_error >= math.radians(5):  # 5°- 90°: Smooth proportional control
        k_omega = 1.0  # Reduced gain
        omega = k_omega * angle_error_rad
        omega = max(-2.0, min(2.0, omega))
        v_linear = v * max(0.7, math.cos(angle_error_rad))
    
    # For small errors (< 5°), use purely linear movement to avoid drift
    else:  # < 5°: Straight line movement
        omega = 0.0  # NO angular correction to maintain straight line
        v_linear = v
        
    
    # Wheel velocity calculation
    v_r = v_linear + (L/2.0) * omega
    v_l = v_linear - (L/2.0) * omega
    
    # Conversion to rad/s
    omega_r = v_r / r
    omega_l = v_l / r
    
    # Saturation
    omega_max_wheel = 6.0
    omega_r = max(-omega_max_wheel, min(omega_max_wheel, omega_r))
    omega_l = max(-omega_max_wheel, min(omega_max_wheel, omega_l))

    #print(f"angle_error_deg:{angle_error_deg:.2f}, theta_robot_deg:{theta_robot_deg:.2f}, theta_goal_deg:{theta_goal_deg:.2f}")
    #print(f"dist: {dist:.3f}, v_linear: {v_linear:.3f}, omega: {omega:.2f}, omega_r:{omega_r:.2f}, omega_l:{omega_l:.2f}")

    # Normal application with angular correction
    sim.setJointTargetVelocity(left_motor_handle, -omega_l)
    sim.setJointTargetVelocity(right_motor_handle, -omega_r)
    
    return False


def angle_to_goal(sim, robot_handle, goal_pos):
    """
    Calculate the angular error between the robot front and the goal.
    In CoppeliaSim, when theta_robot=0°, the robot points to +Y (north).
    """
    # Robot position and orientation
    pos_robot = sim.getObjectPosition(robot_handle, -1)     # [x,y,z]
    ori_robot = sim.getObjectOrientation(robot_handle, -1)  # [?,?,theta]

    # Goal position
    dx = goal_pos[0] - pos_robot[0]
    dy = goal_pos[1] - pos_robot[1]

    # Angle to goal in standard coordinate system (0° = +X)
    theta_goal_standard = math.atan2(dy, dx)
    
    #If robot points Y+ when theta=0°, 
    # then we need to convert from standard system to robot system
    theta_goal = theta_goal_standard - math.pi/2
    
    # Normalize theta_goal to [-pi, pi]
    theta_goal = math.atan2(math.sin(theta_goal), math.cos(theta_goal))

    # Current robot orientation (normalized)
    theta_robot = ori_robot[2]
    theta_robot = math.atan2(math.sin(theta_robot), math.cos(theta_robot))

    # Angular error normalized to [-pi, pi]
    angle_error = math.atan2(math.sin(theta_goal - theta_robot),
                             math.cos(theta_goal - theta_robot))

    return theta_goal, theta_robot, angle_error


def local_to_world(sim, robot_handle, x_local, y_local):
    """
    Convert local robot coordinates to global coordinates.
    Robot front aligned with +Y when theta=0.
    """
    pos = sim.getObjectPosition(robot_handle, -1)        # [x,y,z]
    ori = sim.getObjectOrientation(robot_handle, -1)     # yaw in ori[2]

    x_r, y_r = pos[0], pos[1]
    theta = ori[2]

    x_world = x_r + x_local * math.cos(theta) - y_local * math.sin(theta)
    y_world = y_r + x_local * math.sin(theta) + y_local * math.cos(theta)

    return x_world, y_world


def sysCall_init():
    sim = require('sim')
    self.robot_handle = sim.getObject("/myRobot")
    self.left_motor_handle = sim.getObject("/leftMotor")
    self.right_motor_handle = sim.getObject("/rightMotor")
    self.proximity_sensor_handle = sim.getObject("/proximitySensor")
    self.goal_mark_handle = sim.getObject("/goalMark")
    create_random_obstacles(sim,n_obstacles=10)
    n_goals = 10
    goals = [(random.uniform(-2.0, 2.0), random.uniform(-2.0, 2.0)) for _ in range(n_goals)]
    
    self.goal_manager = init_goal_manager(sim, goals)
    self.current_goal = self.goal_manager.get_current_goal()
    if self.current_goal:
        sim.setObjectPosition(self.goal_mark_handle, -1,
                             [self.current_goal[0], self.current_goal[1], 0.002])
    
    self.reached = False
    self.avoiding_obstacle = False
    
    

def sysCall_actuation():
    
    proximity_sensor_data = sim.readProximitySensor(self.proximity_sensor_handle)
    obstacle_detected = proximity_sensor_data[0]
    obstacle_distance = proximity_sensor_data[1]
    
    ori = sim.getObjectOrientation(self.robot_handle, -1)
    current_theta_deg = math.degrees(ori[2])
    
    # Improved obstacle detection
    if obstacle_detected == 1:
        if self.avoiding_obstacle:
            self.goal_manager.remove_current_goal()
            lateral_offset = math.copysign(0.4, self.current_goal[0])  # Keep same side as before
        else:
            lateral_offset = random.choice([0.4, -0.4])  # Randomly choose left or right
        print("Obstacle avoidance STARTED")
        self.avoiding_obstacle = True
        # Create avoidance point
        avoidance_goal = local_to_world(sim, self.robot_handle, lateral_offset, obstacle_distance + 0.2)
        self.goal_manager.add_goal_before_current(avoidance_goal[0], avoidance_goal[1])
        self.current_goal = avoidance_goal
        print(f"OBSTACLE detected at {obstacle_distance:.2f}m!")
        print(f"Robot at {current_theta_deg:.1f}°, creating avoidance goal: {self.current_goal}")
                
    if not self.reached:
        self.reached = go_to_point(sim, self.robot_handle, self.left_motor_handle,
                                   self.right_motor_handle, self.current_goal)
        sim.setObjectPosition(self.goal_mark_handle, -1,
                             [self.current_goal[0], self.current_goal[1], 0.002])
        
        if self.reached:
            if self.avoiding_obstacle:
                self.avoiding_obstacle = False
                print("Obstacle avoidance COMPLETED")
            else:
                print(f"GOAL REACHED: {self.current_goal}")
            
            next_goal = self.goal_manager.next_goal()
            if next_goal:
                self.current_goal = next_goal
                self.reached = False
                
                print(f"NEW TARGET: {self.current_goal}")
                sim.setObjectPosition(self.goal_mark_handle, -1,
                                     [self.current_goal[0], self.current_goal[1], 0.002])
            else:
                print("ALL GOALS COMPLETED!")
                sim.stopSimulation()


def sysCall_sensing():
    """
    Sensing callback - called every simulation step.
    Place sensor reading and processing code here.
    """
    pass


def sysCall_cleanup():
    """
    Cleanup callback - called when simulation ends.
    Use for resource cleanup and final operations.
    """
    pass
    
def boxes_overlap(pos1, size1, pos2, size2):
    """Check if two AABB boxes overlap in XY plane."""
    x1_min, x1_max = pos1[0] - size1[0]/2, pos1[0] + size1[0]/2
    y1_min, y1_max = pos1[1] - size1[1]/2, pos1[1] + size1[1]/2

    x2_min, x2_max = pos2[0] - size2[0]/2, pos2[0] + size2[0]/2
    y2_min, y2_max = pos2[1] - size2[1]/2, pos2[1] + size2[1]/2

    return not (x1_max < x2_min or x2_max < x1_min or
                y1_max < y2_min or y2_max < y1_min)


def clear_existing_obstacles(sim, prefix="obstacle_"):
    """Remove all obstacles with given prefix from the scene."""
    all_objects = sim.getObjectsInTree(sim.handle_scene, sim.object_shape_type, 0)
    for obj in all_objects:
        name = sim.getObjectName(obj)
        if name and name.startswith(prefix):
            sim.removeObject(obj)
            print(f"??? Removed existing {name}")


def create_random_obstacles(sim, n_obstacles=20,
                            x_range=(-2.0, 2.0),
                            y_range=(-2.0, 2.0),
                            size_ranges=((0.1, 0.2),   # width
                                         (0.1, 0.2),   # depth
                                         (0.1, 0.1)),  # height
                            z_height=0.0,
                            max_attempts=50,
                            prefix="obstacle_"):
    """
    Create random cuboid obstacles ensuring no overlaps.

    Args:
        sim: CoppeliaSim simulation object
        n_obstacles: number of obstacles
        x_range, y_range: placement ranges
        size_ranges: tuple of 3 ranges for width, depth, height
        z_height: base level
        max_attempts: max retries per obstacle
        prefix: naming prefix

    Returns:
        list of dicts with {handle, position, size, name}
    """
    # Step 1: remove old obstacles
    clear_existing_obstacles(sim, prefix)

    obstacles = []

    for i in range(n_obstacles):
        placed = False
        attempt = 0

        # Random size for this obstacle
        size = [random.uniform(*r) for r in size_ranges]

        while not placed and attempt < max_attempts:
            attempt += 1
            x = random.uniform(*x_range)
            y = random.uniform(*y_range)
            z = z_height + size[2]/2  # place by center
            pos = [x, y, z]

            # Check overlap
            overlap = any(boxes_overlap(pos, size, o["position"], o["size"])
                          for o in obstacles)

            if not overlap:
                # Create cuboid in CoppeliaSim
                handle = sim.createPrimitiveShape(sim.primitiveshape_cuboid,
                                                  size, 1)
                sim.setObjectPosition(handle, -1, pos)
                name = f"{prefix}{i+1}"
                sim.setObjectName(handle, name)
                sim.setShapeColor(handle, None,
                                  sim.colorcomponent_ambient_diffuse, [1, 0, 0.5])
                # colidable e detectable
                sim.setObjectInt32Param(handle, sim.shapeintparam_static, 1)        # cant move
                sim.setObjectInt32Param(handle, sim.shapeintparam_respondable, 1)   # respondable
                sim.setObjectSpecialProperty(handle, sim.objectspecialproperty_collidable)
                sim.setObjectSpecialProperty(handle, sim.objectspecialproperty_detectable_all)

                obstacles.append({"handle": handle,
                                  "position": pos,
                                  "size": size,
                                  "name": name})
                print(f"? Created {name} at {pos} size={size}")
                placed = True

        if not placed:
            print(f"Could not place obstacle {i+1} after {max_attempts} attempts")

    return obstacles

class GoalManager:
    """
    Manages a list of XY goals for robot navigation with various operations.
    """
    
    def __init__(self):
        """Initialize empty goal manager"""
        self.goals = []  # List of (x, y) tuples
        self.current_index = 0  # Index of current goal
        
    def initialize_goals(self, goal_list):
        """
        Initialize goal list with n goals.
        
        Args:
            goal_list: List of (x, y) tuples representing goals
            
        Example:
            goal_list = [(1.0, 2.0), (3.0, 4.0), (5.0, 6.0)]
        """
        self.goals = goal_list.copy()  # Create a copy to avoid external modification
        self.current_index = 0
        print(f"Initialized {len(self.goals)} goals: {self.goals}")
        
    def add_goal_end(self, x, y):
        """
        Add a new goal at the end of the list.
        
        Args:
            x: X coordinate of new goal
            y: Y coordinate of new goal
        """
        new_goal = (x, y)
        self.goals.append(new_goal)
        print(f"Added goal {new_goal} at end. Total goals: {len(self.goals)}")
        
    def add_goal_before_current(self, x, y):
        """
        Add a new goal before the current goal.
        This becomes the new current goal.
        
        Args:
            x: X coordinate of new goal
            y: Y coordinate of new goal
        """
        new_goal = (x, y)
        
        if len(self.goals) == 0:
            # If no goals exist, just add it
            self.goals.append(new_goal)
            self.current_index = 0
        else:
            # Insert before current goal
            self.goals.insert(self.current_index, new_goal)
            # Current index now points to the inserted goal
            
        print(f"Added goal {new_goal} before current goal. New current goal: {new_goal}")
        
    def get_current_goal(self):
        """
        Return the current goal.
        
        Returns:
            tuple: (x, y) coordinates of current goal, or None if no goals
        """
        if len(self.goals) == 0:
            print("No goals available")
            return None
            
        if self.current_index >= len(self.goals):
            print("All goals completed")
            return None
            
        current_goal = self.goals[self.current_index]
        print(f"Current goal [{self.current_index}]: {current_goal}")
        return current_goal
        
    def next_goal(self):
        """
        Move to the next goal in the list.
        
        Returns:
            tuple: (x_abs, y_abs) coordinates of next goal, or None if no more goals
        """
        if len(self.goals) == 0:
            print("No goals available")
            return None
            
        self.current_index += 1
        
        if self.current_index >= len(self.goals):
            print("All goals completed - no more goals")
            return None
            
        next_goal = self.goals[self.current_index]
        print(f"Advanced to goal [{self.current_index}]: {next_goal}")
        return next_goal
        
    def has_more_goals(self):
        """
        Check if there are more goals after the current one.
        
        Returns:
            bool: True if more goals exist, False otherwise
        """
        return self.current_index < len(self.goals) - 1
        
    def is_goal_list_complete(self):
        """
        Check if all goals have been completed.
        
        Returns:
            bool: True if all goals completed, False otherwise
        """
        return len(self.goals) == 0 or self.current_index >= len(self.goals)
        
    def get_goal_info(self):
        """
        Get information about the goal list status.
        
        Returns:
            dict: Information about current state
        """
        info = {
            'total_goals': len(self.goals),
            'current_index': self.current_index,
            'goals_remaining': max(0, len(self.goals) - self.current_index),
            'current_goal': self.get_current_goal() if not self.is_goal_list_complete() else None,
            'all_goals': self.goals.copy()
        }
        return info
        
    def reset_to_first_goal(self):
        """Reset current index to first goal"""
        self.current_index = 0
        print(f"Reset to first goal: {self.get_current_goal()}")
        
    def remove_current_goal(self):
        """
        Remove the current goal from the list.
        Current index points to what was the next goal.
        """
        if len(self.goals) == 0:
            print("No goals to remove")
            return
            
        if self.current_index >= len(self.goals):
            print("No current goal to remove")
            return
            
        removed_goal = self.goals.pop(self.current_index)
        
        # Adjust current_index if necessary
        if self.current_index >= len(self.goals) and len(self.goals) > 0:
            self.current_index = len(self.goals) - 1
            
        print(f"Removed goal {removed_goal}. New current goal: {self.get_current_goal()}")


# Convenience functions for integration with existing code
def init_goal_manager(sim, absolute_goals):
    # Create and initialize goal manager
    goal_manager = GoalManager()
    goal_manager.initialize_goals(absolute_goals)
    
    return goal_manager

