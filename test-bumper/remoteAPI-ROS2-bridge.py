from coppeliasim_zmqremoteapi_client import RemoteAPIClient

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Wrench

class PublisherBumper(Node):
    def __init__(self):
        super().__init__('publisher_bumper')
        self.publisher = self.create_publisher(Wrench, 'myRobot/bumper', 10)

    def publish(self):
        result, forceVector, torqueVector = sim.readForceSensor(forceSensorHandle)
        fx, fy, fz = forceVector
        tx, ty, tz = torqueVector
        msg = Wrench()
        msg.force.x, msg.force.y, msg.force.z = fx, fy, fz
        msg.torque.x, msg.torque.y, msg.torque.z = tx, ty, tz
        self.publisher.publish(msg)

class Subscriber_Velocity(Node):
    def __init__(self):
        super().__init__('subscriber_velocity')
        self.subscription = self.create_subscription(Twist, 'myRobot/cmd_vel', self.callback, 10)

    def callback(self, msg):
        linVel = msg.linear.x
        rotVel = msg.angular.z
        rightVel = (linVel + L/2 * rotVel)
        leftVel  = (linVel - L/2 * rotVel)
        sim.setFloatSignal(str(robotHandle)+"rightVel",rightVel)
        sim.setFloatSignal(str(robotHandle)+"leftVel",leftVel)

# sysCall_init():
client = RemoteAPIClient()
sim = client.require('sim')
simROS2 = client.require('simROS2')
sim.setStepping(True) # enables the stepping operation mode for a threaded script

robotHandle=sim.getObject("/myRobot")
forceSensorHandle=sim.getObject("/myRobot/forceSensor")
    
linVel, rotVel = 0, 0
L=0.2 

rclpy.init()
publisher_bumper_node = PublisherBumper()
subscriber_velocity_node = Subscriber_Velocity()

simulationState = sim.getSimulationState()
if (simulationState == sim.simulation_stopped):
    sim.startSimulation()
    simulationState = sim.getSimulationState()

while(simulationState != sim.simulation_stopped):   
    # sysCall_actuation():

    # sysCall_sensing():
    publisher_bumper_node.publish()
    rclpy.spin_once(publisher_bumper_node, timeout_sec=0)
    rclpy.spin_once(subscriber_velocity_node, timeout_sec=0)

    simulationState = sim.getSimulationState()
    sim.step()  # triggers next simulation step

#sysCall_cleanup():
publisher_bumper_node.destroy_node()
subscriber_velocity_node.destroy_node()
rclpy.shutdown()    
sim.stopSimulation()