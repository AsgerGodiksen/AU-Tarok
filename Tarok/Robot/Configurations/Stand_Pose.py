

import numpy as np
import sys
import os
import time
import can

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

from Robot.Tarok_Dymensions import Tarok_Dymensions_Class
from Robot.Kinematics import Inverse_Kinematics as IK
# Some of the imported Transformations are from "Constant_Transforms.py" and "Forward_Kinematics"
from Robot.Kinematics import T0_B, TB_0, P0_2, P0_3, P0_end
from Robot.Hardware import Position_Control


Tarok_Class = Tarok_Dymensions_Class()  # instantiate first


# Importing the Inital Foot Position
Front_Left_Foot, Front_Right_Foot, Hind_Left_Foot, Hond_Right_Foot = Tarok_Class.Initial_Foot_Positions()

# Transform desired end-effector position from body frame to leg base frames
P_FL_Base = T0_B(Front_Left_Foot,'FL')
P_FR_Base = T0_B(Front_Right_Foot,'FR')
P_HL_Base = T0_B(Hind_Left_Foot,'HL')
P_HR_Base = T0_B(Hond_Right_Foot,'HR')

# Then we can determine the Inverse Kinematics
Theta_FL = IK(P_FL_Base,'FL')
Theta_FR = IK(P_FR_Base,'FR')
Theta_HL = IK(P_HL_Base,'HL')
Theta_HR = IK(P_HR_Base,'HR')

# Convert angles from radians to degrees for motor control
Theta_FL = np.degrees(Theta_FL)
Theta_FR = np.degrees(Theta_FR)
Theta_HL = np.degrees(Theta_HL)
Theta_HR = np.degrees(Theta_HR)

#### Initiating the Control of the Motors

######### Send commands to motors ############
print("Initializing CAN buses...")
# Define motor IDs (might be specific to chosen physical leg)
ID_1 = 0x141  # Motor for theta1
ID_2 = 0x142  # Motor for theta2
ID_3 = 0x143  # Motor for theta3

# Connect to CAN bus
bus0 = can.interface.Bus(interface='socketcan', channel='can0', bitrate=1000000)
#bus1 = can.interface.Bus(interface='socketcan', channel='can1', bitrate=1000000)
#bus2 = can.interface.Bus(interface='socketcan', channel='can2', bitrate=1000000)
#bus3 = can.interface.Bus(interface='socketcan', channel='can3', bitrate=1000000)

#### REMEMBER The Other Busses when having all 4 legs
# Drain any stale messages from the buses
for bus in [bus0]: #, bus1, bus2,bus3
    for i in range(100):		# Now Listens for any signals 100 times with 0.01 s in between. Prints if any.
        msg = bus.recv(0.01)
        if msg:
            print(msg)
            
print("Initialization complete, moving to zero position...")

# Move to zero position 
Position_Control(bus0,ID_1,0,20)
Position_Control(bus0,ID_2,0,20)
Position_Control(bus0,ID_3,0,20)
#Position_Control(bus1,ID_1,0,20)
#Position_Control(bus1,ID_2,0,20)
#Position_Control(bus1,ID_3,0,20)
#Position_Control(bus2,ID_1,0,20)
#Position_Control(bus2,ID_2,0,20)
#Position_Control(bus2,ID_3,0,20)
#Position_Control(bus3,ID_1,0,20)
#Position_Control(bus3,ID_2,0,20)
#Position_Control(bus3,ID_3,0,20)

time.sleep(4)

print("Moved to zero position, moving to initial trajectory position...")

# Move to initial position
Position_Control(bus0,ID_1,Theta_FL[0],20)
Position_Control(bus0,ID_2,Theta_FL[1],20)
Position_Control(bus0,ID_3,Theta_FL[2],20)
#Position_Control(bus1,ID_1,Theta_FR[0],20)
#Position_Control(bus1,ID_2,Theta_FR[1],20)
#Position_Control(bus1,ID_3,Theta_FR[2],20)
#Position_Control(bus2,ID_1,Theta_HL[0],20)
#Position_Control(bus2,ID_2,Theta_HL[1],20)
#Position_Control(bus2,ID_3,Theta_HL[2],20)
#Position_Control(bus3,ID_1,Theta_HR[0],20)
#Position_Control(bus3,ID_2,Theta_HR[1],20)
#Position_Control(bus3,ID_3,Theta_HR[2],20)