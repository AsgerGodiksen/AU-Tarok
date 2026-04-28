# Script for stand test of full quadruped using position control

# Based on combination of Test_Visualization.py and Test_UpDown_FT_final.py (onedrive)
# And build on Main_Template.py for the structure of the code

# Imports
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))
import can
import time
import numpy as np
from Robot import*

# Old: CAN initialization in terminal: "sudo ip link set dev canX up type can bitrate 1000000" - with "X" being 0, 1, 2 and 3 for each bus
# New: CAN initialization in terminal: "for i in 0 1 2 3; do sudo ip link set dev can$i up type can bitrate 1000000 && sudo ip link set can$i txqueuelen 1000; done"

### SCRIPT START ###
## PRECOMPUTATIONS ##
print("Performing pre-computations...")
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

print("Pre-computations complete.")

## INITIALIZATION ##
print("Starting the Robot")
print("Initializing CAN buses...")

# Defining motor IDs: Should be ID_1 for joint 1 etc.
ID_1 = 0x141
ID_2 = 0x142
ID_3 = 0x143

# Connect to CAN bus
bus0 = can.interface.Bus(channel="can0", interface="socketcan")
bus1 = can.interface.Bus(channel="can1", interface="socketcan")
bus2 = can.interface.Bus(channel="can2", interface="socketcan")
bus3 = can.interface.Bus(channel="can3", interface="socketcan")

# Drain any stale messages from the buses
for bus in [bus0, bus1, bus2, bus3]:
    for i in range(100):		# Now Listens for any signals 100 times with 0.01 s in between. Prints if any.
        msg = bus.recv(0.01)
        if msg:
            print(msg)

# Loop count
count = 0

# Pre-loop sequence
print("Initialization complete, starting pre-loop sequence...")

# Move to zero position 
print("Moving to zero position...")
Position_Control(bus0,ID_1,0,30)
Position_Control(bus0,ID_2,0,30)
Position_Control(bus0,ID_3,0,30)
Position_Control(bus1,ID_1,0,30)
Position_Control(bus1,ID_2,0,30)
Position_Control(bus1,ID_3,0,30)
Position_Control(bus2,ID_1,0,30)
Position_Control(bus2,ID_2,0,30)
Position_Control(bus2,ID_3,0,30)
Position_Control(bus3,ID_1,0,30)
Position_Control(bus3,ID_2,0,30)
Position_Control(bus3,ID_3,0,30)

time.sleep(10)

# Move to initial position
print("Moved to zero position, moving to initial trajectory position...")
Position_Control(bus0,ID_1,Theta_FL[0],30)
Position_Control(bus0,ID_2,Theta_FL[1],30)
Position_Control(bus0,ID_3,Theta_FL[2],30)
Position_Control(bus1,ID_1,Theta_FR[0],30)
Position_Control(bus1,ID_2,Theta_FR[1],30)
Position_Control(bus1,ID_3,Theta_FR[2],30)
Position_Control(bus2,ID_1,Theta_HL[0],30)
Position_Control(bus2,ID_2,Theta_HL[1],30)
Position_Control(bus2,ID_3,Theta_HL[2],30)
Position_Control(bus3,ID_1,Theta_HR[0],30)
Position_Control(bus3,ID_2,Theta_HR[1],30)
Position_Control(bus3,ID_3,Theta_HR[2],30)

time.sleep(6)

print("Pre-loop sequence complete, starting loop...")
print("Loop started - Press ctrl+c in terminal for shutdown")

try:
    while True:
        # Send repeated commands to motors
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
        time.sleep(0.5)

# Stop loop with Ctrl+C in terminal
except KeyboardInterrupt:
    print("KeyboardInterrupt received, shutting down...")

    # Stop motors
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

    # Shutdown CAN buses
    print("Shutting down CAN buses...")
    bus0.shutdown()
    bus1.shutdown()
    bus2.shutdown()
    bus3.shutdown()
    print("CAN buses shut down")
    print("Shutdown complete.")