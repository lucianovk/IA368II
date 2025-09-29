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
    turn_angles_deg: list[int] = (45, 180)

    ui_title: str = "Robot Control"
    trail_max_items: int = 2000
    initial_mode: str = "manual"


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
        self.dockingSensor = self.sim.getObject('/dockingSensor')

        # Beacon script
        self.beaconScript = self.sim.getScript(
            self.sim.scripttype_simulation,
            self.sim.getObject('/chargingBase/beacon')
        )

        # Motion state
        self.mode = cfg.initial_mode
        self.previous_mode = self.mode
        self.linear_vel = 0.0
        self.angular_vel = 0.0

        # Odometry state
        self.pose = [0.0, 0.0, 0.0]
        self.pose_shift = self.sim.getObjectPosition(self.robot_base, self.sim.handle_world)
        self.last_right_angle = self.sim.getJointPosition(self.right_motor)
        self.last_left_angle = self.sim.getJointPosition(self.left_motor)
        self.tick_counter = 0

        # Obstacle avoidance state
        self.obstacle_cycles = 0
        self.turn_direction = 1
        self._current_turn_angle_deg = 0
        self.lookingForBeacon = False
        self.dockingSignal = None
        self.dockingAngle = None

        # Drawing (red line-strip for odometry trail)
        self.odom_trail = self.sim.addDrawingObject(
            self.sim.drawing_linestrip + self.sim.drawing_cyclic,
            2.0, 0.0, -1, self.cfg.trail_max_items, [1.0, 0.0, 0.0]
        )

        # UI element IDs
        self.ui_id_rot_label = 3000
        self.ui_id_lin_label = 4000
        self.ui_id_batt_label = 4100
        self.ui_id_odom_label = 4200
        self.ui_id_docking_cb = 10
        self.ui_id_mode_cb = 11
        self.robot_handle_str = str(self.robot_base)
        self.ui = self._build_ui()

        print("Keyboard: W/S = linear ±, A/D = angular ±, SPACE = stop, Q = quit")
        print("Odometry initialized: x=0.00 m, y=0.00 m, yaw=0.0°")

    # ------------------------ Public API ------------------------
    def step_actuation(self) -> None:
        # External overrides via float signals "<handle>leftVel"/"rightVel"
        r_override = self.sim.getFloatSignal(self.robot_handle_str + "rightVel")
        l_override = self.sim.getFloatSignal(self.robot_handle_str + "leftVel")
        if l_override is not None or r_override is not None:
            if l_override is not None:
                self.sim.clearFloatSignal(self.robot_handle_str + "leftVel")
            if r_override is not None:
                self.sim.clearFloatSignal(self.robot_handle_str + "rightVel")
            rightVel = r_override if r_override is not None else (
                self.linear_vel - (self.cfg.wheel_base / 2) * self.angular_vel
            )
            leftVel = l_override if l_override is not None else (
                self.linear_vel + (self.cfg.wheel_base / 2) * self.angular_vel
            )
            # Apply as linear/angular approximation
            v = clamp((rightVel + leftVel) / 2, -self.cfg.max_linear_vel, self.cfg.max_linear_vel)
            w = clamp((rightVel - leftVel) / self.cfg.wheel_base,
                      -self.cfg.max_angular_vel, self.cfg.max_angular_vel)
            self._apply_diff_drive(v, w)
            return
        else:
            if self.mode == "auto":
                if self.lookingForBeacon:
                    if self.dockingSignal is None:
                        self._auto_obstacle_avoidance()
                    else:
                        print("Beacon detected!")
                        self.linear_vel = 0.0
                        self.angular_vel = 0.0
                        self._apply_diff_drive(self.linear_vel, self.angular_vel)
                        self.lookingForBeacon = False
                        self.sim.setInt32Signal(str(self.robot_base) + "Docking", 1)
                        return
                else:
                    self._auto_obstacle_avoidance()
            else:
                self._manual_keyboard_control()

        # Battery safety: stop when 0%
        batt = self.sim.getFloatSignal(self.robot_handle_str + "Battery")
        if batt is not None and batt == 0:
            self._apply_diff_drive(0.0, 0.0)

        # External docking signal
        forceDocking = self.sim.getInt32Signal(self.robot_handle_str + "Docking")
        self.sim.clearInt32Signal(self.robot_handle_str + "Docking")
        try:
            if forceDocking is not None:
                if forceDocking == 1:
                    print("Received signal to enable docking")
                    self.simUI.setCheckboxValue(self.ui, self.ui_id_docking_cb, 2)
                    self.on_docking_changed(2)
                elif forceDocking == 0:
                    print("Received signal to disable docking")
                    self.simUI.setCheckboxValue(self.ui, self.ui_id_docking_cb, 0)
                    self.on_docking_changed(0)
                    if self.previous_mode == "auto":
                        self.simUI.setCheckboxValue(self.ui, self.ui_id_mode_cb, 2)
                        self.on_toggle_mode(2)
        except Exception:
            pass

        # External "looking for beacon" signal
        lookingForBeacon = self.sim.getInt32Signal(self.robot_handle_str + "LookingForBeacon")
        self.sim.clearInt32Signal(self.robot_handle_str + "LookingForBeacon")
        try:
            if lookingForBeacon is not None:
                if lookingForBeacon == 1:
                    print("Received signal to start searching for beacon")
                    self.lookingForBeacon = True
                    self.simUI.setCheckboxValue(self.ui, self.ui_id_mode_cb, 2)
                    self.on_toggle_mode(2)
                else:
                    self.lookingForBeacon = False
                    print("Received signal to stop searching for beacon")
                    self.simUI.setCheckboxValue(self.ui, self.ui_id_mode_cb, 0)
                    self.on_toggle_mode(0)
        except Exception:
            pass

    def step_sensing(self) -> None:
        self._update_odometry()
        self._draw_odometry()
        self._log_periodically()

        # UI: battery and odometry labels
        try:
            batt = self.sim.getFloatSignal(self.robot_handle_str + "Battery")
            if batt is not None:
                self.simUI.setLabelText(self.ui, self.ui_id_batt_label, f"Battery: {round(batt, 2)} % ")
        except Exception:
            pass
        try:
            self.simUI.setLabelText(
                self.ui,
                self.ui_id_odom_label,
                f"odometry (x,y,theta): ({self.pose[0]:.2f}, {self.pose[1]:.2f}, {math.degrees(self.pose[2]):.2f})"
            )
        except Exception:
            pass

        # Query beacon info each sensing step
        self.dockingSignal, self.dockingAngle = self.sim.callScriptFunction(
            'getBeaconInfo', self.beaconScript, self.dockingSensor
        )

    def cleanup(self) -> None:
        self.simUI.destroy(self.ui)

    # ----------------------- UI Callbacks -----------------------
    def on_linear_slider(self, new_val: int) -> None:
        if self.mode == "manual":
            self.linear_vel = (new_val / 50.0) * self.cfg.max_linear_vel
            try:
                self.simUI.setLabelText(self.ui, self.ui_id_lin_label, f"{self.linear_vel:.2f} m/s ")
            except Exception:
                pass
            print(f"UI linear set: {self.linear_vel:.2f} m/s")

    def on_angular_slider(self, new_val: int) -> None:
        if self.mode == "manual":
            self.angular_vel = (new_val / 50.0) * self.cfg.max_angular_vel
            try:
                self.simUI.setLabelText(self.ui, self.ui_id_rot_label, f"{math.degrees(self.angular_vel):.1f} deg/s ")
            except Exception:
                pass
            print(f"UI angular set: {math.degrees(self.angular_vel):.1f} deg/s")

    def on_stop_all(self) -> None:
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        try:
            self.simUI.setLabelText(self.ui, self.ui_id_rot_label, "0 deg/s ")
            self.simUI.setLabelText(self.ui, self.ui_id_lin_label, "0 m/s ")
            self.simUI.setSliderValue(self.ui, 1, 0)
            self.simUI.setSliderValue(self.ui, 2, 0)
        except Exception:
            pass
        print("STOP: velocities zeroed and sliders centered.")

    def on_docking_changed(self, checked: int) -> None:
        enabled = (checked == 2)
        mode = self.mode
        if enabled:
            # Turn off auto when docking is enabled
            try:
                self.simUI.setCheckboxValue(self.ui, self.ui_id_mode_cb, 0)
                self.on_toggle_mode(0)
            except Exception:
                pass
        msg = {'id': 'dockingMode', 'data': [enabled, mode]}
        try:
            self.sim.broadcastMsg(msg)
        except Exception:
            # Fallback if broadcast not available in this context
            self.sim.setInt32Signal(self.robot_handle_str + "Docking", 1 if enabled else 0)
        print(f"Docking checkbox -> {enabled}")

    def on_zero_linear(self) -> None:
        if self.mode == "manual":
            self.linear_vel = 0.0
            print("Linear velocity set to 0")

    def on_zero_angular(self) -> None:
        if self.mode == "manual":
            self.angular_vel = 0.0
            print("Angular velocity set to 0")

    def on_toggle_mode(self, checked: int) -> None:
        if self.previous_mode != self.mode:
            self.previous_mode = self.mode
        if checked > 0:
            self.mode = "auto"
            try:
                self.simUI.setCheckboxValue(self.ui, self.ui_id_docking_cb, 0)
                self.on_docking_changed(0)
            except Exception:
                pass
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
                <button text="STOP" on-click="stopAllPressed" />
                <stretch />
                <label text="Auto mode:" />
                <checkbox id="{self.ui_id_docking_cb}" text="docking" on-change="dockingChanged" />
                <label text="   " />
                <checkbox id="{self.ui_id_mode_cb}" text="Activate auto" on-change="modeChanged" checked="{'true' if self.mode=='auto' else 'false'}" />
            </group>

            <group layout="hbox" flat="true">
                <label text="   0 m/s  " id="{self.ui_id_lin_label}" word-wrap="false" />
                <button text="0 Lin Vel" on-click="stopLinearPressed" />
                <vslider id="2" minimum="-50" maximum="50" value="0" on-change="vslider_changed" />
                <stretch />
            </group>

            <group layout="hbox" flat="true">
                <label text="   0 deg/s  " id="{self.ui_id_rot_label}" word-wrap="false" />
                <button text="0 rot vel" on-click="stopAngularPressed" />
                <hslider id="1" minimum="-50" maximum="50" value="0" on-change="hslider_changed" />
            </group>

            <group layout="hbox" flat="true">
                <label text="Battery: --- % " id="{self.ui_id_batt_label}" word-wrap="false" />
                <label text="odometry (x,y,theta): (_,_,_) " id="{self.ui_id_odom_label}" word-wrap="false" />
            </group>
        </ui>
        '''
        return self.simUI.create(xml)

    # -------------------- Internal: Control ----------------------
    def _apply_diff_drive(self, linear: float, angular: float) -> None:
        r = linear + (self.cfg.wheel_base / 2) * angular
        l = linear - (self.cfg.wheel_base / 2) * angular
        # Send 'l' (left) to the right motor and 'r' (right) to the left motor (motor orientation compensation)
        self.sim.setJointTargetVelocity(self.right_motor, -l / self.cfg.wheel_radius)
        self.sim.setJointTargetVelocity(self.left_motor,  -r / self.cfg.wheel_radius)

    def _manual_keyboard_control(self) -> None:
        # Consume all simulator messages and handle only keypress events
        while True:
            msg, auxData, _ = self.sim.getSimulatorMessage()
            if msg == -1:
                break
            if msg == self.sim.message_keypress and len(auxData) >= 1:
                key_code = auxData[0]  # ASCII code
                try:
                    key = chr(key_code).lower()
                except ValueError:
                    continue

                if key == 'w':
                    self.linear_vel = clamp(self.linear_vel + 0.05, -self.cfg.max_linear_vel, self.cfg.max_linear_vel)
                    print(f"[W] linear = {self.linear_vel:.2f} m/s")
                elif key == 's':
                    self.linear_vel = clamp(self.linear_vel - 0.05, -self.cfg.max_linear_vel, self.cfg.max_linear_vel)
                    print(f"[S] linear = {self.linear_vel:.2f} m/s")
                elif key == 'a':
                    self.angular_vel = clamp(self.angular_vel - math.radians(5), -self.cfg.max_angular_vel, self.cfg.max_angular_vel)
                    print(f"[A] angular = {math.degrees(self.angular_vel):.1f} deg/s")
                elif key == 'd':
                    self.angular_vel = clamp(self.angular_vel + math.radians(5), -self.cfg.max_angular_vel, self.cfg.max_angular_vel)
                    print(f"[D] angular = {math.degrees(self.angular_vel):.1f} deg/s")
                elif key_code == 32:  # SPACE
                    self.linear_vel = 0.0
                    self.angular_vel = 0.0
                    print("[SPACE] stop: linear=0.00, angular=0.00")
                elif key == 'q':
                    print("[Q] stopping simulation")
                    self.sim.stopSimulation()

        # Apply current (clamped) state even if no key arrived this step
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
                self.linear_vel = self.cfg.max_linear_vel
                base_angle = random.randint(self.cfg.turn_angles_deg[0], self.cfg.turn_angles_deg[1])
                self.turn_direction = random.choice([+1, -1])
                self._current_turn_angle_deg = base_angle * self.turn_direction
                if bumper:
                    self.linear_vel = random.choice([self.cfg.max_linear_vel, 0.0, -0.5])
                    print(f"BUMPER: activated! - linear_vel: {self.linear_vel}")
                print(f"Obstacle: turning {self._current_turn_angle_deg:.0f}° (RANDOM)")
            if step:
                print("Step detected!")
                self.linear_vel = random.choice([self.cfg.max_linear_vel, 0.0, -0.5])
                self._current_turn_angle_deg = 180
            self.angular_vel = clamp(
                math.radians(self._current_turn_angle_deg),
                -self.cfg.max_angular_vel, self.cfg.max_angular_vel
            )

            self.obstacle_cycles += 1
        else:
            self.angular_vel = 0.0
            self.linear_vel = self.cfg.max_linear_vel
            self.obstacle_cycles = 0

        if self.obstacle_cycles >= 10:
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
    print(f"UI Bridge modeChanged:{new_val}")
    CTRL.on_toggle_mode(new_val)

def stopAllPressed(_ui, _id):
    CTRL.on_stop_all()

def dockingChanged(_ui, _id, new_val):
    print(f"UI Bridge dockingChanged:{new_val}")
    CTRL.on_docking_changed(new_val)
