# -*- coding: utf-8 -*-
"""
CoppeliaSim Python script — differential-drive robot with:
- Manual/Auto modes
- Go-to-point navigation over spiral waypoints
- Simple UI for mode + velocities
- Odometry trail (red) and optional ground-truth trail (blue)

Refactor style:
- English names in snake_case
- Centralized configuration (RobotConfig)
- Encapsulated logic (RobotController)
"""

import math
import random
from dataclasses import dataclass
from typing import List, Tuple, Optional

CTRL: Optional["RobotController"] = None  # set in sysCall_init()

# ============================== CONFIG ==============================

@dataclass(frozen=True)
class RobotConfig:
    # Robot geometry/dynamics
    wheel_radius: float = 0.015        # m
    wheel_base: float = 0.20           # m (distance between wheels)

    # Velocity limits
    max_linear_vel: float = 0.50       # m/s
    max_angular_vel: float = math.radians(90.0)  # rad/s

    # Go-to-point control
    nominal_linear_vel: float = 0.20   # m/s when heading is ok
    close_turn_omega_max: float = 3.0  # rad/s for pure rotation
    ctrl_gain_omega: float = 1.0       # proportional gain
    ctrl_omega_clip: float = 2.0       # |omega| clip in proportional region
    heading_pure_rot_thresh_deg: float = 90.0  # deg
    heading_linear_only_thresh_deg: float = 5.0  # deg
    wheel_omega_limit: float = 6.0     # rad/s per wheel

    # Waypoints (spiral) generation
    spiral_step: float = 0.50          # m (target arc step between points)
    spiral_radius: float = 2.0         # m (max radius)
    spiral_ccw_random: bool = True     # choose CCW/CW randomly

    # UI / drawing
    ui_title: str = "Robot Control"
    trail_max_items: int = 20000
    log_every_ticks: int = 200

    # Behavior
    initial_mode: str = "auto"         # "auto" or "manual"
    
    # Auto-mode status printing
    goal_log_every_ticks: int = 100

    # Arrival tolerance as fraction of spiral_step (keeps original 0.6*0.2)
    arrival_tol_factor: float = 0.60


# ============================== UTILS ===============================

def clamp(val: float, lo: float, hi: float) -> float:
    return max(lo, min(val, hi))


def angle_diff(a: float, b: float) -> float:
    """Minimal signed difference a-b normalized to (-pi, pi]."""
    d = a - b
    return (d + math.pi) % (2 * math.pi) - math.pi


# =========================== CONTROLLER =============================

class RobotController:
    def __init__(self, sim, simUI, cfg: RobotConfig):
        self.sim = sim
        self.simUI = simUI
        self.cfg = cfg

        # Handles
        self.robot_base = self.sim.getObject("/myRobot")
        self.right_motor = self.sim.getObject("/rightMotor")
        self.left_motor = self.sim.getObject("/leftMotor")
        self.bumper = self.sim.getObject("/bumper")
        self.proximity = self.sim.getObject("/proximitySensor")

        # Mode & velocities
        self.mode = cfg.initial_mode
        self.linear_vel = 0.0
        self.angular_vel = 0.0

        # Odometry
        self.pose = [0.0, 0.0, 0.0]  # x, y, yaw(rad)
        self.pose_shift = self.sim.getObjectPosition(self.robot_base, self.sim.handle_world)
        self.last_right_angle = self.sim.getJointPosition(self.right_motor)
        self.last_left_angle = self.sim.getJointPosition(self.left_motor)
        self.tick_counter = 0

        # Goals (spiral)
        ccw = random.choice([True, False]) if cfg.spiral_ccw_random else True
        self.goals: List[Tuple[float, float]] = self.spiral_coordinates(
            distance=cfg.spiral_step, radius=cfg.spiral_radius, ccw=ccw
        )
        self.goal_index = 0
        self.goal_trial_count = 0
        self.goal_trial_last_index = 0

        # Drawing
        self.odom_trail = self.sim.addDrawingObject(
            self.sim.drawing_linestrip + self.sim.drawing_cyclic,
            2.0, 0.0, -1, self.cfg.trail_max_items, [1.0, 0.0, 0.0]  # red
        )
        self.truth_trail = self.sim.addDrawingObject(
            self.sim.drawing_linestrip + self.sim.drawing_cyclic,
            2.0, 0.0, -1, self.cfg.trail_max_items, [0.0, 0.5, 1.0]  # blue
        )
        self.goal_points_handle = None

        # UI
        self.ui = self._build_ui()

        self.draw_goals_markers()

        print("Keyboard: W/S = linear ±, A/D = angular ±, SPACE=stop, Q=quit")
        print("Odometry initialized: x=0.00 m, y=0.00 m, yaw=0.0°")

    # ------------------------ Public API ------------------------

    def step_actuation(self) -> None:
        if self.mode == "auto":
            self._auto_go_to_goals()
        else:
            self._manual_keyboard_control()

    def step_sensing(self) -> None:
        self._update_odometry()
        self._draw_odometry()
        # optional ground-truth trail (world pose)
        px, py, _ = self.sim.getObjectPosition(self.robot_base, self.sim.handle_world)
        self.sim.addDrawingObjectItem(self.truth_trail, [px, py, 0.0])
        self._log_periodically()

    def cleanup(self) -> None:
        self.simUI.destroy(self.ui)

    # ----------------------- UI Callbacks -----------------------

    def on_linear_slider(self, new_val: int) -> None:
        if self.mode == "manual":
            self.linear_vel = (new_val / 50.0) * self.cfg.max_linear_vel
            print(f"UI linear set: {self.linear_vel:.2f} m/s")

    def on_angular_slider(self, new_val: int) -> None:
        if self.mode == "manual":
            self.angular_vel = (new_val / 50.0) * self.cfg.max_angular_vel
            print(f"UI angular set: {math.degrees(self.angular_vel):.1f} deg/s")

    def on_zero_linear(self) -> None:
        if self.mode == "manual":
            self.linear_vel = 0.0
            print("Linear velocity set to 0")

    def on_zero_angular(self) -> None:
        if self.mode == "manual":
            self.angular_vel = 0.0
            print("Angular velocity set to 0")

    def on_toggle_mode(self, checked: int) -> None:
        if checked > 0:
            self.mode = "auto"
            print("Switched to AUTO mode")
        else:
            self.mode = "manual"
            self.linear_vel = 0.0
            self.angular_vel = 0.0
            print("Switched to MANUAL mode")

    # --------------------- Internal: UI build --------------------

    def _build_ui(self):
        xml = f'''
        <ui title="{self.cfg.ui_title}" closeable="true" resizable="true" activate="true" layout="vbox">
            <group layout="hbox" flat="true">
                <label text="Auto mode:" />
                <checkbox id="10" text="Activate" on-change="modeChanged" checked="{'true' if self.mode=='auto' else 'false'}" />
            </group>

            <group layout="hbox" flat="true">
                <label text="Linear velocity" />
                <stretch />
                <vslider id="2" minimum="-50" maximum="50" value="0" on-change="vslider_changed" />
            </group>

            <group layout="hbox" flat="true">
                <button text="Zero Linear Vel" on-click="stopLinearPressed" />
                <stretch />
            </group>

            <group layout="hbox" flat="true">
                <label text="Angular velocity" />
                <stretch />
                <hslider id="1" minimum="-50" maximum="50" value="0" on-change="hslider_changed" />
            </group>

            <group layout="hbox" flat="true">
                <button text="Zero Angular Vel" on-click="stopAngularPressed" />
                <stretch />
            </group>
        </ui>
        '''
        return self.simUI.create(xml)

    # -------------------- Internal: Control ----------------------

    def _apply_diff_drive(self, linear: float, angular: float) -> None:
        r = linear + (self.cfg.wheel_base / 2.0) * angular
        l = linear - (self.cfg.wheel_base / 2.0) * angular
        self.sim.setJointTargetVelocity(self.right_motor, -r / self.cfg.wheel_radius)
        self.sim.setJointTargetVelocity(self.left_motor, -l / self.cfg.wheel_radius)

    def _manual_keyboard_control(self) -> None:
        _, keys, _ = self.sim.getSimulatorMessage()
        if not keys:
            self._apply_diff_drive(
                clamp(self.linear_vel, -self.cfg.max_linear_vel, self.cfg.max_linear_vel),
                clamp(self.angular_vel, -self.cfg.max_angular_vel, self.cfg.max_angular_vel),
            )
            return

        code = keys[0]
        if 0 <= code < 256:
            key = chr(code).lower()
            if key == 'w':
                self.linear_vel = clamp(self.linear_vel + 0.05, -self.cfg.max_linear_vel, self.cfg.max_linear_vel)
                print(f"[W] linear = {self.linear_vel:.2f} m/s")
            elif key == 's':
                self.linear_vel = clamp(self.linear_vel - 0.05, -self.cfg.max_linear_vel, self.cfg.max_linear_vel)
                print(f"[S] linear = {self.linear_vel:.2f} m/s")
            elif key == 'a':
                self.angular_vel = clamp(self.angular_vel + math.radians(5), -self.cfg.max_angular_vel, self.cfg.max_angular_vel)
                print(f"[A] angular = {math.degrees(self.angular_vel):.1f} deg/s")
            elif key == 'd':
                self.angular_vel = clamp(self.angular_vel - math.radians(5), -self.cfg.max_angular_vel, self.cfg.max_angular_vel)
                print(f"[D] angular = {math.degrees(self.angular_vel):.1f} deg/s")
            elif key == ' ':
                self.linear_vel = 0.0
                self.angular_vel = 0.0
                print("[SPACE] stop: linear=0.00, angular=0.00")
            elif key == 'q':
                print("[Q] stopping simulation")
                self.sim.stopSimulation()

        self._apply_diff_drive(
            clamp(self.linear_vel, -self.cfg.max_linear_vel, self.cfg.max_linear_vel),
            clamp(self.angular_vel, -self.cfg.max_angular_vel, self.cfg.max_angular_vel),
        )

    # -------------------- Auto: Go-to goals ----------------------

    def _auto_go_to_goals(self) -> None:
        # periodic log
        if self.goal_index < len(self.goals) and (self.tick_counter % self.cfg.goal_log_every_ticks == 0):
            gx, gy = self.goals[self.goal_index]
            print(f"Heading to goal {self.goal_index}: ({gx:.2f}, {gy:.2f})")

        if self.goal_index >= len(self.goals):
            # stop wheels
            self.sim.setJointTargetVelocity(self.left_motor, 0.0)
            self.sim.setJointTargetVelocity(self.right_motor, 0.0)
            return

        gx, gy = self.goals[self.goal_index]
        arrived = self.go_to_point(
            goal=(gx, gy),
            v=self.cfg.nominal_linear_vel,
            L=self.cfg.wheel_base,
            wheel_radius=self.cfg.wheel_radius,
            tolerance=self.cfg.arrival_tol_factor * self.cfg.spiral_step,
        )

        if arrived:
            print(f"Goal {self.goal_index} reached: ({gx:.2f}, {gy:.2f})")
            self.goal_index += 1

        # fallback: skip a goal stuck for too long
        if self.goal_index == self.goal_trial_last_index:
            if self.goal_trial_count > 500:
                self.goal_index += 1
                print(f"Goal {self.goal_index} not reached in 500 cycles, skipping.")
                self.goal_trial_count = 0
                self.goal_trial_last_index = self.goal_index
            else:
                self.goal_trial_count += 1
        else:
            self.goal_trial_last_index = self.goal_index
            self.goal_trial_count = 0

    # -------------------- Odometry / Drawing ---------------------

    def _update_odometry(self) -> None:
        curr_right = self.sim.getJointPosition(self.right_motor)
        curr_left = self.sim.getJointPosition(self.left_motor)

        d_right = angle_diff(curr_right, self.last_right_angle)
        d_left = angle_diff(curr_left, self.last_left_angle)
        self.last_right_angle = curr_right
        self.last_left_angle = curr_left

        s_right = d_right * self.cfg.wheel_radius
        s_left = d_left * self.cfg.wheel_radius
        ds = 0.5 * (s_right + s_left)

        # Absolute yaw from simulator (rad)
        _, _, yaw = self.sim.getObjectOrientation(self.robot_base, self.sim.handle_world)
        self.pose[2] = yaw

        # Integrate x,y with ds and global heading
        self.pose[0] += ds * math.sin(yaw)
        self.pose[1] -= ds * math.cos(yaw)

    def _draw_odometry(self) -> None:
        self.sim.addDrawingObjectItem(
            self.odom_trail,
            [self.pose[0] + self.pose_shift[0],
             self.pose[1] + self.pose_shift[1],
             0.0]
        )

    def _log_periodically(self) -> None:
        self.tick_counter += 1
        if self.tick_counter % self.cfg.log_every_ticks == 0:
            print(f"ODO: x={self.pose[0]:.3f} m, y={self.pose[1]:.3f} m, yaw={math.degrees(self.pose[2]):.1f}°")

    # -------------------- Navigation helpers --------------------

    def go_to_point(
        self,
        goal: Tuple[float, float],
        v: float,
        L: float,
        wheel_radius: float,
        tolerance: float = 0.04,
    ) -> bool:
        """
        Move the differential robot to 'goal'. Straight line when angular error is small.
        Returns True if goal is reached within 'tolerance'.
        """
        r = wheel_radius
        x, y, yaw = self.pose

        dx = goal[0] - x
        dy = goal[1] - y
        dist = math.hypot(dx, dy)

        if dist < tolerance:
            self.sim.setJointTargetVelocity(self.left_motor, 0.0)
            self.sim.setJointTargetVelocity(self.right_motor, 0.0)
            return True

        # Heading error (CoppeliaSim: yaw=0 points to +Y)
        _, _, angle_error_rad = self.angle_to_goal(self.pose, goal)
        abs_err = abs(angle_error_rad)

        if abs_err > math.radians(self.cfg.heading_pure_rot_thresh_deg):
            # pure rotation
            omega = math.copysign(self.cfg.close_turn_omega_max, angle_error_rad)
            v_linear = 0.01
        elif abs_err >= math.radians(self.cfg.heading_linear_only_thresh_deg):
            # proportional correction
            omega = self.cfg.ctrl_gain_omega * angle_error_rad
            omega = clamp(omega, -self.cfg.ctrl_omega_clip, self.cfg.ctrl_omega_clip)
            v_linear = v * max(0.7, math.cos(angle_error_rad))
        else:
            # straight line
            omega = 0.0
            v_linear = v

        # Wheel velocities
        v_r = v_linear + (L / 2.0) * omega
        v_l = v_linear - (L / 2.0) * omega

        omega_r = v_r / r
        omega_l = v_l / r

        # Saturation
        lim = self.cfg.wheel_omega_limit
        omega_r = clamp(omega_r, -lim, lim)
        omega_l = clamp(omega_l, -lim, lim)

        self.sim.setJointTargetVelocity(self.left_motor, -omega_l)
        self.sim.setJointTargetVelocity(self.right_motor, -omega_r)

        return False

    def angle_to_goal(self, robot_pose: List[float], goal_pos: Tuple[float, float]):
        """
        Angular error between robot forward and goal.
        In CoppeliaSim, yaw=0 => robot points to +Y (north).
        """
        dx = goal_pos[0] - robot_pose[0]
        dy = goal_pos[1] - robot_pose[1]

        # Standard axis (0 deg = +X)
        gamma_goal_standard = math.atan2(dy, dx)

        # Convert to robot axis (0 deg = +Y) -> subtract 90 deg
        gamma_goal = gamma_goal_standard - math.pi / 2.0
        gamma_goal = math.atan2(math.sin(gamma_goal), math.cos(gamma_goal))

        gamma_robot = math.atan2(math.sin(robot_pose[2]), math.cos(robot_pose[2]))

        angle_error = math.atan2(math.sin(gamma_goal - gamma_robot), math.cos(gamma_goal - gamma_robot))
        return gamma_goal, gamma_robot, angle_error

    def local_to_world(self, robot_pose: List[float], x_local: float, y_local: float) -> Tuple[float, float]:
        """
        Convert local robot coordinates to world coordinates.
        Robot forward is +Y when yaw=0.
        """
        x_r, y_r = robot_pose[0], robot_pose[1]
        gamma = robot_pose[2]
        x_world = x_r + x_local * math.cos(gamma) - y_local * math.sin(gamma)
        y_world = y_r + x_local * math.sin(gamma) + y_local * math.cos(gamma)
        return x_world, y_world

    # -------------------- Spiral + Markers -----------------------

    def spiral_coordinates(self, distance: float, radius: float, ccw: bool = True) -> List[Tuple[float, float]]:
        """
        Generates (x,y) points of an Archimedean spiral r = b*theta,
        sampled approximately every 'distance' of arc length until 'radius'.
        """
        if distance <= 0 or radius <= 0:
            raise ValueError("distance and radius must be > 0.")

        direction = 1.0 if ccw else -1.0
        b = distance / (2.0 * math.pi)  # radial spacing per turn ~ distance

        pts: List[Tuple[float, float]] = [(0.0, 0.0)]
        theta = 0.0
        r = 0.0
        min_sep = distance * 0.5  # avoid nearly duplicated points

        while r <= radius:
            denom = math.hypot(r, b)
            dtheta = distance / denom
            theta += dtheta
            r = b * theta

            ang = direction * theta
            x = r * math.cos(ang)
            y = r * math.sin(ang)

            lx, ly = pts[-1]
            if (x - lx) ** 2 + (y - ly) ** 2 >= (min_sep ** 2):
                pts.append((x, y))

        return pts

    def draw_goals_markers(self) -> None:
        """Draw simple green 2D discs for each goal (billboarded)."""
        self.goal_points_handle = self.sim.addDrawingObject(
            self.sim.drawing_discpoints,
            0.08,      # item size in world units (meters)
            0.0,
            -1,
            len(self.goals),
            [0.0, 1.0, 0.0]  # green
        )
        z = 0.02
        for gx, gy in self.goals:
            self.sim.addDrawingObjectItem(self.goal_points_handle, [float(gx), float(gy), z])


# ============================ ENTRYPOINTS ===========================

def sysCall_init():
    global CTRL
    sim = require('sim')
    simUI = require('simUI')
    
    CTRL = RobotController(sim=sim, simUI=simUI, cfg=RobotConfig())


def sysCall_actuation():
    CTRL.step_actuation()


def sysCall_sensing():
    CTRL.step_sensing()


def sysCall_cleanup():
    CTRL.cleanup()


# ============================ UI BRIDGE =============================

def vslider_changed(_ui, _id, new_val):
    CTRL.on_linear_slider(new_val)

def hslider_changed(_ui, _id, new_val):
    CTRL.on_angular_slider(new_val)

def stopLinearPressed(_ui, _id):
    CTRL.on_zero_linear()

def stopAngularPressed(_ui, _id):
    CTRL.on_zero_angular()

def modeChanged(_ui, _id, new_val):
    CTRL.on_toggle_mode(new_val)
