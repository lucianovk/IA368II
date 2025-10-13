import math
deg2rad=3.14/180.
rad2deg=180./3.14
# Cache external wheel commands so they keep overriding the UI sliders until the user touches the UI again.
remote_override_active = False
last_remote_left = None
last_remote_right = None


def clear_remote_override():
    global remote_override_active, last_remote_left, last_remote_right
    # Any local UI interaction cancels the remote override so sliders regain control.
    remote_override_active = False
    last_remote_left = None
    last_remote_right = None

def sysCall_init():
    sim = require('sim')
    
    self.robotHandle=sim.getObject("/myRobot")
    self.rightMotorHandle=sim.getObject("/rightMotor")
    self.leftMotorHandle=sim.getObject("/leftMotor")
    print("python_controller: Keyboard listener started... (press ESC to stop simulation)")
    
    global ui, simUI, linVel, rotVel, L, max_linVel , max_rotVel, wheelradius
    global remote_override_active, last_remote_left, last_remote_right
    
    linVel=0
    rotVel=0
    wheelradius=0.015
    L=0.2 
    max_linVel = 0.5 # m/s
    max_rotVel = 90 * deg2rad # rad/seg
    remote_override_active = False
    last_remote_left = None
    last_remote_right = None
    # Load the simUI plugin
    sim = require('sim')
    simUI = require('simUI')
        
    
    # Define the UI
    xml = '''
    <ui title="joystick" closeable="true" resizable="true" activate="true" layout="vbox">
        
        <!-- Top button -->
        <button text="STOP" on-click="stopButtonPressed" />

        <!-- Vertical slider centered -->
        <group layout="hbox" flat="true">

            <label text="   0 m/s  " id="4000" word-wrap="false" />
            <button text="0 Lin Vel" on-click="linVelButtonPressed" />
            <vslider id="2" minimum="-50" maximum="50" value="0" on-change="vslider_changed" />
            <checkbox id="10" text="docking" on-change="dockingButtonPressed" />            
            <stretch />
        </group>

        <!-- Horizontal slider with button at left -->
        <group layout="hbox" flat="true">
            <label text="   0 deg/s  " id="3000" word-wrap="false" />
            <button text="0 rot vel" on-click="rotButtonPressed" />
            <hslider id="1" minimum="-50" maximum="50" value="0" on-change="hslider_changed" />
        </group>
        <group layout="hbox" flat="true">
            <label text="battery: --- % "id="4100" word-wrap="false"   />
            <label text="odometry (x,y,theta): (_,_,_) "id="4200" word-wrap="false"   />
            <label text="wheel vel (L,R): 0.00, 0.00 m/s "id="4300" word-wrap="false"   />
        </group>

    </ui>
    '''
    ui = simUI.create(xml)

def vslider_changed(ui, id, newVal):
    global linVel
    clear_remote_override()
    linVel=(newVal)/50 * max_linVel 
    #print(f"python_controller: vertical slider value: {newVal}", linVel )
    simUI.setLabelText(ui,4000,str(round(linVel,2))+" m/s ")


def hslider_changed(ui, id, newVal):
    global rotVel
    clear_remote_override()
    rotVel=-(newVal)/50 * max_rotVel 
    #print(f"python_controller: Horizontal slider value: {newVal}", rotVel  ) 
    simUI.setLabelText(ui,3000,str(round(rotVel,1))+" deg/s ")


def stopButtonPressed(ui, id):
    global linVel, rotVel
    clear_remote_override()
    linVel=0
    rotVel=0
    #print("python_controller: stopping")
    simUI.setSliderValue(ui,1,0)
    simUI.setSliderValue(ui,2,0)
    simUI.setLabelText(ui,3000,"0 deg/s ")
    simUI.setLabelText(ui,4000,"0 m/s ")
   
def linVelButtonPressed(ui, id):
    global linVel
    clear_remote_override()
    linVel=0
    #print("python_controller: lin vel is 0")
    simUI.setSliderValue(ui,2,0)
    simUI.setLabelText(ui,4000,"0 m/s ")

def rotButtonPressed(ui, id):
    global rotVel
    clear_remote_override()
    rotVel=0
    print("python_controller: rot vel is 0")
    simUI.setSliderValue(ui,1,0)
    simUI.setLabelText(ui,3000,"0 deg/s ")

def dockingButtonPressed(ui,id,newVal):
    sim.setInt32Signal(str(self.robotHandle)+"Docking", 10+newVal)
    print("Docking button pressed:", 10+newVal)
    

def sysCall_actuation():
    global remote_override_active, last_remote_left, last_remote_right

    # inverse kinematic equations
    rightVel = linVel + L/2 * rotVel
    leftVel  = linVel - L/2 * rotVel
    
    # check if for external motor control commands
    rightVelExt = sim.getFloatSignal(str(self.robotHandle)+"rightVel")
    leftVelExt  = sim.getFloatSignal(str(self.robotHandle)+"leftVel")
    # External commands arrive as one-shot signals; store the most recent values so they persist until cleared.
    new_remote_command = False
    if leftVelExt is not None:
        sim.clearFloatSignal(str(self.robotHandle)+"leftVel")
        last_remote_left = leftVelExt
        new_remote_command = True
    if rightVelExt is not None:
        sim.clearFloatSignal(str(self.robotHandle)+"rightVel")
        last_remote_right = rightVelExt
        new_remote_command = True
    if new_remote_command:
        remote_override_active = True
    if remote_override_active:
        # While an override is active, feed the cached remote velocities to the wheels instead of UI derived values.
        if last_remote_left is not None:
            leftVel = last_remote_left
        if last_remote_right is not None:
            rightVel = last_remote_right
        #print (f"overridden wheel vel L:{leftVel} R:{rightVel}")
    
    
    # check and update battery status
    batt=sim.getFloatSignal(str(self.robotHandle)+"Battery")
    if batt is not None: 
        simUI.setLabelText(ui,4100,"Battery: "+str(round(batt,2))+" % ")
    if batt == 0:
        #battery is dead
        rightVel = 0
        leftVel  = 0
    #check and set external docking mode setting
    forceDocking=sim.getInt32Signal(str(self.robotHandle)+"Docking")
    if forceDocking is not None:
        if forceDocking == 2:
            simUI.setCheckboxValue(ui,10,2,False)
            sim.clearInt32Signal(str(self.robotHandle)+"Docking")
        if forceDocking == 0:
            simUI.setCheckboxValue(ui,10,0,False)
            sim.clearInt32Signal(str(self.robotHandle)+"Docking")
        
    # check and update odometry
    #odomPack=sim.getStringSignal(str(self.robotHandle)+"Odometry")
    #odomPack = sim.getStringProperty(self.robotHandle, "customData.Odometry", {'noError' : True})
    #print("python_controler :",odomPack,self.robotHandle)
    #odomPack = sim.getBufferProperty(sim.handle_app, str(self.robotHandle)+"Odometry", {'noError' : True})
    #if odomPack:
    #    odom = sim.unpackTable(odomPack)   
    odomPack = sim.getBufferProperty(self.robotHandle, "customData.Odometry", {'noError' : True})

    if odomPack: 
        odom = sim.unpackTable(odomPack)
        odomStr = f"({odom[0]:.2f}, {odom[1]:.2f}, {math.degrees(odom[2]):.2f})"        
        #print("python_controler: ",odomStr, odom)
        simUI.setLabelText(ui, 4200, f"odometry (x,y,theta): "+odomStr)
        #simUI.setLabelText(ui,4200,"odometry (x,y,theta): (: "+str(round(batt,2))+") ")
    if batt == 0:
    #    #battery is dead
        rightVel = 0
        leftVel  = 0    
    
    simUI.setLabelText(ui,4300,f"wheel vel (L,R): {leftVel:.2f}, {rightVel:.2f} m/s ")
    # update motors veloicity
    sim.setJointTargetVelocity(self.rightMotorHandle,-rightVel/wheelradius)
    sim.setJointTargetVelocity(self.leftMotorHandle, -leftVel/wheelradius)
    

    # Check if any key was pressed
    #_, keys, _ = sim.getSimulatorMessage()
    #key=chr(keys[0])

    #if key == 'w':
    #    print("Key pressed:", key)
    #pass


def sysCall_sensing():
    # put your sensing code here
    pass

def sysCall_cleanup():
    simUI.destroy(ui)
    pass
