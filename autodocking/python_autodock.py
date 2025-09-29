import math

# Flags / states
lowBatt = 25.0
lowBattDetected = False
dockingMode = False
docked = False
auto_mode = False
dockingState = 0

signal = None
angle = None
linearVel = 0.0
angularVel = 0.0
max_linear_vel = 0.5
max_signal = 0.95


def sysCall_init():
    sim = require('sim')

    global robotHandle, sensorHandle, beaconScript
    global lowBatt, dockingMode, docked, dockingState
    global signal, angle, linearVel, angularVel, max_linear_vel, max_signal, lowBattDetected

    # Objects
    robotHandle = sim.getObject('/myRobot')
    sensorHandle = sim.getObject('/dockingSensor')

    beaconScript = sim.getScript(sim.scripttype_simulation, sim.getObject('/chargingBase/beacon'))


def sysCall_msg(message, origin=None):
    """ Expects {'id': 'dockingMode', 'data': [True/False]} """
    global dockingMode, dockingState
    if isinstance(message, dict) and message.get('id') == 'dockingMode':
        data = message.get('data', [])
        if data:
            dockingMode = bool(data[0])
            if dockingMode:
                dockingState = 0


def setVel(leftMotorVel, rightMotorVel):
    """ Sends wheel linear speeds (m/s) to the locomotion controller. """
    sim.setFloatSignal(f"{robotHandle}leftVel", float(leftMotorVel))
    sim.setFloatSignal(f"{robotHandle}rightVel", float(rightMotorVel))


def _map_range(x, in_min, in_max, out_min, out_max):
    """ Linear mapping with clamping. """
    if x is None:
        return out_min
    # clamp input
    if x < in_min:
        x = in_min
    elif x > in_max:
        x = in_max
    # avoid div/0
    if in_max == in_min:
        return out_min
    ratio = (x - in_min) / (in_max - in_min)
    return out_min + ratio * (out_max - out_min)


def wrap_to_pi(a):
    """ Normalize angle to (-?, ?]. """
    return (a + math.pi) % (2 * math.pi) - math.pi


def sysCall_actuation():
    global dockingMode, docked, dockingState
    global signal, angle, linearVel, angularVel, lowBatt, lowBattDetected

    batt = sim.getFloatSignal(str(robotHandle) + "Battery")

    if not dockingMode:
        if batt is not None and batt <= lowBatt and not lowBattDetected:
            print("LOW BATTERY!")
            lowBattDetected = True
            sim.setInt32Signal(str(robotHandle) + "Docking", 1)
        return

    if sim.getSimulationTime() <= 0.5:
        return

    if batt is not None and batt == 100 and lowBattDetected:
        print("BATTERY RECHARGED!")
        lowBattDetected = False
        sim.setInt32Signal(str(robotHandle) + "Docking", 0)
        return

    # Read beacon (signal in fraction 0..1; angle in rad; angle = None if not read)
    sig, ang = sim.callScriptFunction('getBeaconInfo', beaconScript, sensorHandle)

    # Convert to expected ranges
    if ang is not None:
        ang = wrap_to_pi(ang)

    signal = sig
    angle = ang if ang is not None else angle  # keep last value if None

    # --- Docking logic ---
    if signal is not None and signal >= max_signal:
        # Consider docked when the signal reaches/exceeds max_signal
        docked = True
        linearVel = 0.0
        angularVel = 0.0
        setVel(0.0, 0.0)
    else:
        docked = False

    if docked:
        return

    if dockingState == 0:
        # STATE 0 and Beacon detected: simple approach/turn
        if signal is not None:

            # If angle is largely behind (|ang| >= ~3 rad), do not advance: just turn
            if abs(angle) >= 3.0:
                linearVel = 0.0
            else:
                # NORMALIZATION of linearVel:
                # 0..max_signal  ->  0..(max_linear_vel/2)
                linearVel = _map_range(signal or 0.0, 0.0, max_signal, 0.0, max_linear_vel / 2.0)

            # NORMALIZATION of angularVel (differential term):
            # [-pi, pi]  ->  [-(max_linear_vel/2), +(max_linear_vel/2)]
            if angle is None:
                ang_term = 0.0
            else:
                ang_term = _map_range(angle, -math.pi, math.pi, -max_linear_vel / 2.0, max_linear_vel / 2.0)
            angularVel = ang_term

            # Safety clamp per wheel (do not exceed ±max_linear_vel)
            l_vel = max(-max_linear_vel, min(max_linear_vel, linearVel + angularVel))
            r_vel = max(-max_linear_vel, min(max_linear_vel, linearVel - angularVel))

            setVel(l_vel, r_vel)

        # STATE 0 and Beacon not detected: switch to auto mode until detected
        else:
            dockingState = 1
            sim.setInt32Signal(str(robotHandle) + "LookingForBeacon", 1)
