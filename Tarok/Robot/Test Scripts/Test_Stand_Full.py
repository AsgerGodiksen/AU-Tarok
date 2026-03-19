# Script for stand test of full quadruped using position control

# Based on combination of Test_Visualization.py and Test_UpDown_FT_final.py (onedrive)

import sys
import os

# Add Robot/ to path so Kinematics and Hardware can be imported directly
#sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..')))
#sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../Hardware')))


# Imports
from Robot.Hardware.Motor_Controls import Position_Control, Motor_Stop
from Robot.Kinematics.Inverse_Kinematics import Inverse_Kinematics
from Robot.Kinematics.Constant_Transforms import T0_B
import time
import can
import numpy as np


###### MAIN SCRIPT ######
## PRECOMPUTATIONS ##
# Define kinematic body lengths
l_k = 0.7048  # Length of body in kinematic model (meters)
w_k = 0.220   # Width of body in kinematic model (meters)

# Define desired end-effector position for standing posture (z=-0.41 meters) for all 4 legs
x_FL = x_FR = l_k/2 # x-position of front legs
x_HL = x_HR = -l_k/2 # x-position of hind legs
y_FL = y_HL = w_k/2 + 0.078 # y-position of left legs
y_FR = y_HR = -w_k/2 - 0.078 # y-position of right legs
z = -0.41 # z-position for standing posture (similar for all legs)

# Combine trajectories into position arrays for each leg
P_FL_body = np.array([x_FL, y_FL, z])
P_FR_body = np.array([x_FR, y_FR, z])
P_HL_body = np.array([x_HL, y_HL, z])
P_HR_body = np.array([x_HR, y_HR, z])

# Transform desired end-effector trajectory from body frame to leg base frames
P_FL_base = T0_B(P_FL_body.reshape((3,1)), 'FL')
P_FR_base = T0_B(P_FR_body.reshape((3,1)), 'FR')
P_HL_base = T0_B(P_HL_body.reshape((3,1)), 'HL')
P_HR_base = T0_B(P_HR_body.reshape((3,1)), 'HR')

# Determine joint angles for all 4 legs using inverse kinematics
Theta_FL = Inverse_Kinematics(P_FL_base, 'FL')
Theta_FR = Inverse_Kinematics(P_FR_base, 'FR')
Theta_HL = Inverse_Kinematics(P_HL_base, 'HL')
Theta_HR = Inverse_Kinematics(P_HR_base, 'HR')

# Convert angles from radians to degrees for motor control
Theta_FL = np.degrees(Theta_FL)
Theta_FR = np.degrees(Theta_FR)
Theta_HL = np.degrees(Theta_HL)
Theta_HR = np.degrees(Theta_HR)

######### Send commands to motors ############
print("Initializing CAN buses...")
# Define motor IDs (might be specific to chosen physical leg)
ID_1 = 0x141  # Motor for theta1
ID_2 = 0x142  # Motor for theta2
ID_3 = 0x143  # Motor for theta3

# Connect to CAN bus
bus0 = can.interface.Bus(interface='socketcan', channel='can0', bitrate=1000000)
bus1 = can.interface.Bus(interface='socketcan', channel='can1', bitrate=1000000)
bus2 = can.interface.Bus(interface='socketcan', channel='can2', bitrate=1000000)
bus3 = can.interface.Bus(interface='socketcan', channel='can3', bitrate=1000000)

# Drain any stale messages from the buses
for bus in [bus0, bus1, bus2, bus3]:
    for i in range(100):		# Now Listens for any signals 100 times with 0.01 s in between. Prints if any.
        msg = bus.recv(0.01)
        if msg:
            print(msg)

print("Initialization complete, moving to zero position...")

# Move to zero position 
Position_Control(bus0,ID_1,0,20)
Position_Control(bus0,ID_2,0,20)
Position_Control(bus0,ID_3,0,20)
Position_Control(bus1,ID_1,0,20)
Position_Control(bus1,ID_2,0,20)
Position_Control(bus1,ID_3,0,20)
Position_Control(bus2,ID_1,0,20)
Position_Control(bus2,ID_2,0,20)
Position_Control(bus2,ID_3,0,20)
Position_Control(bus3,ID_1,0,20)
Position_Control(bus3,ID_2,0,20)
Position_Control(bus3,ID_3,0,20)

time.sleep(8)

print("Moved to zero position, moving to initial trajectory position...")

# Move to initial position
Position_Control(bus0,ID_1,Theta_FL[0],20)
Position_Control(bus0,ID_2,Theta_FL[1],20)
Position_Control(bus0,ID_3,Theta_FL[2],20)
Position_Control(bus1,ID_1,Theta_FR[0],20)
Position_Control(bus1,ID_2,Theta_FR[1],20)
Position_Control(bus1,ID_3,Theta_FR[2],20)
Position_Control(bus2,ID_1,Theta_HL[0],20)
Position_Control(bus2,ID_2,Theta_HL[1],20)
Position_Control(bus2,ID_3,Theta_HL[2],20)
Position_Control(bus3,ID_1,Theta_HR[0],20)
Position_Control(bus3,ID_2,Theta_HR[1],20)
Position_Control(bus3,ID_3,Theta_HR[2],20)

time.sleep(6)

print("Moved to initial trajectory position, starting trajectory execution...")
print("Loop started - Press ctrl+c in terminal for shutdown")

# Note start time

try:
    while True:       
        # Send commands to motors
        Position_Control(bus0,ID_1,Theta_FL[0],20)
        Position_Control(bus0,ID_2,Theta_FL[1],20)
        Position_Control(bus0,ID_3,Theta_FL[2],20)
        Position_Control(bus1,ID_1,Theta_FR[0],20)
        Position_Control(bus1,ID_2,Theta_FR[1],20)
        Position_Control(bus1,ID_3,Theta_FR[2],20)
        Position_Control(bus2,ID_1,Theta_HL[0],20)
        Position_Control(bus2,ID_2,Theta_HL[1],20)
        Position_Control(bus2,ID_3,Theta_HL[2],20)
        Position_Control(bus3,ID_1,Theta_HR[0],20)
        Position_Control(bus3,ID_2,Theta_HR[1],20)
        Position_Control(bus3,ID_3,Theta_HR[2],20)
        time.sleep(0.05)


# Stop loop with Ctrl+C
except KeyboardInterrupt:
    print("KeyboardInterrupt received, shutting down...")

    print("Stopping motors...")
    Motor_Stop(bus0,ID_1)
    Motor_Stop(bus0,ID_2)
    Motor_Stop(bus0,ID_3)
    Motor_Stop(bus1,ID_1)
    Motor_Stop(bus1,ID_2)
    Motor_Stop(bus1,ID_3)
    Motor_Stop(bus2,ID_1)
    Motor_Stop(bus2,ID_2)
    Motor_Stop(bus2,ID_3)
    Motor_Stop(bus3,ID_1)
    Motor_Stop(bus3,ID_2)
    Motor_Stop(bus3,ID_3)
    print("Motors stopped")

    print("Shutting down CAN buses...")
    bus0.shutdown()
    bus1.shutdown()
    bus2.shutdown()
    bus3.shutdown()
    print("CAN buses shut down")

    print("Shutdown complete.")