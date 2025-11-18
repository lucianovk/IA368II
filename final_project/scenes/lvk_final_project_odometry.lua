-- Differential-drive odometry script for the IA368 final project robot.
-- The script reads wheel encoder data provided by the motor encoder scripts,
-- integrates linear/angular motion, and exposes the pose via the
-- "customData.Odometry" buffer so external tools (ROS 2 bridge) can consume it.

local sim = require('sim')

-- Handles populated at init.
local robotHandle
local leftWheelHandle
local rightWheelHandle
local leftEncoderScript
local rightEncoderScript

-- Robot geometric parameters.
local wheelRadius
local halfWheelSeparation -- half the distance between wheels

-- Integrated pose state (relative to initial pose).
local x0, y0, theta0
local accDx, accDy, accDTheta
local lastTime

local EPS = 0.01

local function normalizeAngle(angle)
    return math.atan2(math.sin(angle), math.cos(angle))
end

function sysCall_init()
    robotHandle = sim.getObjectParent(sim.getObject('.'))
    leftWheelHandle = sim.getObject('/myRobot/leftWheel')
    rightWheelHandle = sim.getObject('/myRobot/rightWheel')

    -- Wheel radius derived from bounding box (float params 15/18 hold min/max Z).
    local _, zMin = sim.getObjectFloatParameter(leftWheelHandle, 15)
    local _, zMax = sim.getObjectFloatParameter(leftWheelHandle, 18)
    wheelRadius = (zMax - zMin) / 2.0

    -- Half wheel separation (distance between wheel centers divided by two).
    local lPos = sim.getObjectPosition(leftWheelHandle, -1)
    local rPos = sim.getObjectPosition(rightWheelHandle, -1)
    halfWheelSeparation = math.sqrt(
        (lPos[1] - rPos[1]) ^ 2 +
        (lPos[2] - rPos[2]) ^ 2 +
        (lPos[3] - rPos[3]) ^ 2
    ) / 2.0

    -- References to the encoder scripts that expose "getEncoder".
    leftEncoderScript = sim.getScript(sim.scripttype_simulation, '/myRobot/leftMotor/encoder')
    rightEncoderScript = sim.getScript(sim.scripttype_simulation, '/myRobot/rightMotor/encoder')

    -- Initial pose (used as reference frame for the dead reckoning).
    local robot = sim.getObject('/myRobot')
    local posInit = sim.getObjectPosition(robot, -1)
    local oriInit = sim.getObjectOrientation(robot, -1)

    x0 = posInit[1]
    y0 = posInit[2]
    theta0 = oriInit[3]

    accDx, accDy, accDTheta = 0.0, 0.0, 0.0
    lastTime = sim.getSimulationTime()

    sim.addStatusbarMessage(string.format('[odometry] robot handle: %d', robotHandle))
end

local function integrateMotion(vLeft, vRight, thetaPrev, dTheta, dt)
    -- Straight motion: both wheels move with similar speed.
    if math.abs(vRight - vLeft) < EPS then
        local v = (vRight + vLeft) * 0.5
        local dx = v * math.cos(thetaPrev) * dt
        local dy = v * math.sin(thetaPrev) * dt
        return dx, dy
    end

    -- Arc motion: compute ICC based update.
    local thetaNew = thetaPrev + dTheta
    local radius = halfWheelSeparation * (vRight + vLeft) / (vRight - vLeft)
    local dx = radius * (math.sin(thetaNew) - math.sin(thetaPrev))
    local dy = -radius * (math.cos(thetaNew) - math.cos(thetaPrev))
    return dx, dy
end

function sysCall_actuation()
    -- Wheel angular speeds (rad/s) and total joint position from encoders.
    local rDot, _ = sim.callScriptFunction('getEncoder', rightEncoderScript)
    local lDot, _ = sim.callScriptFunction('getEncoder', leftEncoderScript)

    local currentTime = sim.getSimulationTime()
    local dt = currentTime - lastTime
    if dt <= 0 then
        return
    end

    local vRight = rDot * wheelRadius
    local vLeft = lDot * wheelRadius

    local thetaPrev = normalizeAngle(theta0 + accDTheta)
    local dTheta = (vRight - vLeft) / (2.0 * halfWheelSeparation) * dt
    accDTheta = accDTheta + dTheta
    local theta = normalizeAngle(theta0 + accDTheta)

    local dx, dy = integrateMotion(vLeft, vRight, thetaPrev, dTheta, dt)
    accDx = accDx + dx
    accDy = accDy + dy

    local x = x0 + accDx
    local y = y0 + accDy

    -- Share pose with external clients. PackTable produces a compact binary blob.
    sim.setBufferProperty(robotHandle, 'customData.Odometry', sim.packTable({ x, y, theta }))

    lastTime = currentTime
end

function getOdometry()
    local theta = normalizeAngle(theta0 + accDTheta)
    return x0 + accDx, y0 + accDy, theta
end
