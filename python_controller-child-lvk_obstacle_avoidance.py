# -*- coding: utf-8 -*-
"""
CoppeliaSim Python script: differential-drive robot with manual/auto modes,
UI controls, simple obstacle avoidance (random turn), and odometry trail.
"""

import math
import random
from dataclasses import dataclass
from typing import Tuple, Optional

CTRL = None  # type: Optional["RobotController"]


# ============================== CONFIG ==============================

@dataclass(frozen=True)
class RobotConfig:
    wheel_radius: float = 0.015
    wheel_base: float = 0.20
    max_linear_vel: float = 0.50
    max_angular_vel: float = math.radians(180)
    proximity_threshold: float = 0.30
    log_every_ticks: int = 200
    turn_angles_deg: list[int] = (45,180)

    ui_title: str = "Robot Control"
    trail_max_items: int = 2000
    initial_mode: str = "auto"


# ============================== UTILS ===============================

def clamp(val: float, lo: float, hi: float) -> float:
    return max(lo, min(val, hi))


def angle_diff(a: float, b: float) -> float:
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
        self.stepLeft = self.sim.getObject("/stepLeftSensor")
        self.stepRight = self.sim.getObject("/stepRightSensor")

        # Motion state
        self.mode = cfg.initial_mode
        self.linear_vel = 0.0
        self.angular_vel = 0.0

        # Odometry
        self.pose = [0.0, 0.0, 0.0]
        self.pose_shift = self.sim.getObjectPosition(self.robot_base, self.sim.handle_world)
        self.last_right_angle = self.sim.getJointPosition(self.right_motor)
        self.last_left_angle = self.sim.getJointPosition(self.left_motor)
        self.tick_counter = 0

        # Obstacle avoidance state
        self.obstacle_cycles = 0
        self.turn_direction = 1
        self._current_turn_angle_deg = 0

        # Drawing
        self.odom_trail = self.sim.addDrawingObject(
            self.sim.drawing_linestrip + self.sim.drawing_cyclic,
            2.0, 0.0, -1, self.cfg.trail_max_items, [1.0, 0.0, 0.0]  # red
        )
        
        # UI
        self.ui = self._build_ui()

        print("Keyboard: W/S = linear ±, A/D = angular ±, SPACE=stop, Q=quit")
        print("Odometry initialized: x=0.00 m, y=0.00 m, yaw=0.0°")

    # ------------------------ Public API ------------------------
    def step_actuation(self) -> None:
        if self.mode == "auto":
            self._auto_obstacle_avoidance()
        else:
            self._manual_keyboard_control()

    def step_sensing(self) -> None:
        self._update_odometry()
        self._draw_odometry()
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
        r = linear + (self.cfg.wheel_base / 2) * angular
        l = linear - (self.cfg.wheel_base / 2) * angular
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

    def _auto_obstacle_avoidance(self) -> None:
        pstate, pdist, *_ = self.sim.readProximitySensor(self.proximity)
        slstate, sldist, *_ = self.sim.readProximitySensor(self.stepLeft)
        srstate, srdist, *_ = self.sim.readProximitySensor(self.stepRight)
        proximity = (pstate > 0 and pdist <= self.cfg.proximity_threshold)
        step = (slstate == 0 or srstate == 0)
        
        collided, _ = self.sim.checkCollision(self.bumper, self.sim.handle_all)
        bumper = (collided > 0)

        if proximity or bumper or step:
            if self.obstacle_cycles == 0:
                base_angle = random.randint(self.cfg.turn_angles_deg[0],self.cfg.turn_angles_deg[1])
                self.turn_direction = random.choice([+1, -1])
                self._current_turn_angle_deg = base_angle * self.turn_direction
                print(f"Obstacle: turning {self._current_turn_angle_deg:.0f}° (RANDOM)")
            if step:
                print("step")
                self._current_turn_angle_deg = 180    
            self.linear_vel = 0.0
            self.angular_vel = clamp(
                math.radians(self._current_turn_angle_deg),
                -self.cfg.max_angular_vel, self.cfg.max_angular_vel
            )

            self.obstacle_cycles += 1
        else:
            self.angular_vel = 0.0
            self.linear_vel = self.cfg.max_linear_vel
            self.obstacle_cycles = 0

        if self.obstacle_cycles >= 200:
            self.obstacle_cycles = 0

        self._apply_diff_drive(
            clamp(self.linear_vel, -self.cfg.max_linear_vel, self.cfg.max_linear_vel),
            clamp(self.angular_vel, -self.cfg.max_angular_vel, self.cfg.max_angular_vel),
        )

    # -------------------- Internal: Odometry ---------------------
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

        _, _, yaw = self.sim.getObjectOrientation(self.robot_base, self.sim.handle_world)
        self.pose[2] = yaw
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
