# This is a template based on Main.py
# Can be used as start for any script that needs to operate with TAROK, such as walking gaits, trajectory control, etc.

# Imports
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..'))) # Change level of path based on file location (the ../../)
import can
import time
import numpy as np
from Robot import*

# Old: CAN initialization in terminal: "sudo ip link set dev canX up type can bitrate 1000000" - with "X" being 0, 1, 2 and 3 for each bus
# New: CAN initialization in terminal: "for i in 0 1 2 3; do sudo ip link set dev can$i up type can bitrate 1000000 && sudo ip link set can$i txqueuelen 1000; done"

### SCRIPT START ###
## PRECOMPUTATIONS ##
print("Performing pre-computations...")

# Define time series
t = np.linspace(0, 1, 2) # Time from 0 to 1 seconds, with 2 time steps

# Define kinematic body lengths
l_k = 0.7048  # Length of body in kinematic model (meters)
w_k = 0.220   # Width of body in kinematic model (meters)

# Define desired end-effector position for standing posture (z=-0.41 meters) for all 4 legs
x_FL = x_FR = l_k/2*np.ones_like(t) # x-position of front legs
x_HL = x_HR = -l_k/2*np.ones_like(t) # x-position of hind legs
y_FL = y_HL = (w_k/2 + 0.078)*np.ones_like(t) # y-position of left legs
y_FR = y_HR = (-w_k/2 - 0.078)*np.ones_like(t) # y-position of right legs
z = np.array([-0.41, -0.36]) # z-position for standing posture (similar for all legs)

# Combine trajectories into position arrays for each leg
P_FL_body = np.array([x_FL, y_FL, z])
P_FR_body = np.array([x_FR, y_FR, z])
P_HL_body = np.array([x_HL, y_HL, z])
P_HR_body = np.array([x_HR, y_HR, z])

# Transform desired end-effector trajectory from body frame to leg base frames
P_FL_base = np.array([T0_B(P_FL_body[:, i].reshape((3, 1)), 'FL') for i in range(len(t))])
P_FR_base = np.array([T0_B(P_FR_body[:, i].reshape((3, 1)), 'FR') for i in range(len(t))])
P_HL_base = np.array([T0_B(P_HL_body[:, i].reshape((3, 1)), 'HL') for i in range(len(t))])
P_HR_base = np.array([T0_B(P_HR_body[:, i].reshape((3, 1)), 'HR') for i in range(len(t))])

# Determine joint angles for all 4 legs using inverse kinematics
# Determine joint angles for all 4 legs using inverse kinematics
Theta_FL = np.array([Inverse_Kinematics(P_FL_base[i], 'FL') for i in range(len(t))]) # Shape (num_time_steps, 3), containing theta1, theta2, theta3 for each time step
Theta_FR = np.array([Inverse_Kinematics(P_FR_base[i], 'FR') for i in range(len(t))]) # Shape (num_time_steps, 3), containing theta1, theta2, theta3 for each time step
Theta_HL = np.array([Inverse_Kinematics(P_HL_base[i], 'HL') for i in range(len(t))]) # Shape (num_time_steps, 3), containing theta1, theta2, theta3 for each time step
Theta_HR = np.array([Inverse_Kinematics(P_HR_base[i], 'HR') for i in range(len(t))]) # Shape (num_time_steps, 3), containing theta1, theta2, theta3 for each time step

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

## PRE-LOOP SEQUENCE ##
print("Initialization complete, starting pre-loop sequence...")

# Parameters for PID Tuning:
# Manufacturing
pid_params_old = {
    'angle_kp':  100,
    'angle_ki':  100,
    'speed_kp':  50,
    'speed_ki':  40,
    'torque_kp': 50,
    'torque_ki': 50
}

pid_params_new = {
    'angle_kp':  110,
    'angle_ki':  50,
    'speed_kp':  55,
    'speed_ki':  20,
    'torque_kp': 55,
    'torque_ki': 25
}

print("Setting all PID parameters to old values...")
PID_RAM_Control(bus0,ID_1, pid_params_old)
PID_RAM_Control(bus0,ID_2, pid_params_old)
PID_RAM_Control(bus0,ID_3, pid_params_old)
PID_RAM_Control(bus1,ID_1, pid_params_old)
PID_RAM_Control(bus1,ID_2, pid_params_old)
PID_RAM_Control(bus1,ID_3, pid_params_old)
PID_RAM_Control(bus2,ID_1, pid_params_old)  
PID_RAM_Control(bus2,ID_2, pid_params_old)
PID_RAM_Control(bus2,ID_3, pid_params_old)
PID_RAM_Control(bus3,ID_1, pid_params_old)
PID_RAM_Control(bus3,ID_2, pid_params_old)
PID_RAM_Control(bus3,ID_3, pid_params_old)

time.sleep(2)

# Move to zero position 
print("Moving to zero position...")
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

time.sleep(6)

# Move to initial position
print("Moved to zero position, moving to initial trajectory position...")
Position_Control(bus0,ID_1,Theta_FL[0, 0],20)
Position_Control(bus0,ID_2,Theta_FL[0, 1],20)
Position_Control(bus0,ID_3,Theta_FL[0, 2],20)
Position_Control(bus1,ID_1,Theta_FR[0, 0],20)
Position_Control(bus1,ID_2,Theta_FR[0, 1],20)
Position_Control(bus1,ID_3,Theta_FR[0, 2],20)
Position_Control(bus2,ID_1,Theta_HL[0, 0],20)
Position_Control(bus2,ID_2,Theta_HL[0, 1],20)
Position_Control(bus2,ID_3,Theta_HL[0, 2],20)
Position_Control(bus3,ID_1,Theta_HR[0, 0],20)
Position_Control(bus3,ID_2,Theta_HR[0, 1],20)
Position_Control(bus3,ID_3,Theta_HR[0, 2],20)

time.sleep(6)

print("Pre-loop sequence complete, starting loop...")

## MAIN LOOP ##
print("Loop started - Press ctrl+c in terminal for shutdown")

try:
    while True:
            # Move to second position in trajectory
            print("Moving to second position in trajectory with old PID parameters...")
            Position_Control(bus0,ID_1,Theta_FL[1, 0],20)
            Position_Control(bus0,ID_2,Theta_FL[1, 1],20)
            Position_Control(bus0,ID_3,Theta_FL[1, 2],20)
            Position_Control(bus1,ID_1,Theta_FR[1, 0],20)
            Position_Control(bus1,ID_2,Theta_FR[1, 1],20)
            Position_Control(bus1,ID_3,Theta_FR[1, 2],20)
            Position_Control(bus2,ID_1,Theta_HL[1, 0],20)
            Position_Control(bus2,ID_2,Theta_HL[1, 1],20)
            Position_Control(bus2,ID_3,Theta_HL[1, 2],20)
            Position_Control(bus3,ID_1,Theta_HR[1, 0],20)
            Position_Control(bus3,ID_2,Theta_HR[1, 1],20)
            Position_Control(bus3,ID_3,Theta_HR[1, 2],20)

            time.sleep(5)

            # Update to new PID parameters for HL leg
            print("Updating to new PID parameters for HL leg...")
            PID_RAM_Control(bus2,ID_1, pid_params_new)
            PID_RAM_Control(bus2,ID_2, pid_params_new)
            PID_RAM_Control(bus2,ID_3, pid_params_new)

            print("The New PID parameters are:")
            Read_PID(bus2,ID_1)
            Read_PID(bus2,ID_2)
            Read_PID(bus2,ID_3)

            time.sleep(5)

            # Move back to first position in trajectory
            print("Moving back to first position in trajectory with new PID parameters for HL leg...")
            Position_Control(bus0,ID_1,Theta_FL[0, 0],20)
            Position_Control(bus0,ID_2,Theta_FL[0, 1],20)
            Position_Control(bus0,ID_3,Theta_FL[0, 2],20)
            Position_Control(bus1,ID_1,Theta_FR[0, 0],20)
            Position_Control(bus1,ID_2,Theta_FR[0, 1],20)
            Position_Control(bus1,ID_3,Theta_FR[0, 2],20)
            Position_Control(bus2,ID_1,Theta_HL[0, 0],20)
            Position_Control(bus2,ID_2,Theta_HL[0, 1],20)
            Position_Control(bus2,ID_3,Theta_HL[0, 2],20)
            Position_Control(bus3,ID_1,Theta_HR[0, 0],20)
            Position_Control(bus3,ID_2,Theta_HR[0, 1],20)
            Position_Control(bus3,ID_3,Theta_HR[0, 2],20)

            time.sleep(5)

            # Move to second position in trajectory again
            print("Moving to second position in trajectory again with new PID parameters for HL leg...")
            Position_Control(bus0,ID_1,Theta_FL[1, 0],20)
            Position_Control(bus0,ID_2,Theta_FL[1, 1],20)
            Position_Control(bus0,ID_3,Theta_FL[1, 2],20)
            Position_Control(bus1,ID_1,Theta_FR[1, 0],20)
            Position_Control(bus1,ID_2,Theta_FR[1, 1],20)
            Position_Control(bus1,ID_3,Theta_FR[1, 2],20)
            Position_Control(bus2,ID_1,Theta_HL[1, 0],20)
            Position_Control(bus2,ID_2,Theta_HL[1, 1],20)
            Position_Control(bus2,ID_3,Theta_HL[1, 2],20)
            Position_Control(bus3,ID_1,Theta_HR[1, 0],20)
            Position_Control(bus3,ID_2,Theta_HR[1, 1],20)
            Position_Control(bus3,ID_3,Theta_HR[1, 2],20)

            # Update to old PID parameters for HL leg
            print("Updating back to old PID parameters for HL leg...")
            PID_RAM_Control(bus2,ID_1, pid_params_old)
            PID_RAM_Control(bus2,ID_2, pid_params_old)
            PID_RAM_Control(bus2,ID_3, pid_params_old)

            print("The Old PID parameters are:")
            Read_PID(bus2,ID_1)
            Read_PID(bus2,ID_2)
            Read_PID(bus2,ID_3)

            time.sleep(5)

            # Move back to first position in trajectory
            print("Moving back to first position in trajectory with new PID parameters for HL leg...")
            Position_Control(bus0,ID_1,Theta_FL[0, 0],20)
            Position_Control(bus0,ID_2,Theta_FL[0, 1],20)
            Position_Control(bus0,ID_3,Theta_FL[0, 2],20)
            Position_Control(bus1,ID_1,Theta_FR[0, 0],20)
            Position_Control(bus1,ID_2,Theta_FR[0, 1],20)
            Position_Control(bus1,ID_3,Theta_FR[0, 2],20)
            Position_Control(bus2,ID_1,Theta_HL[0, 0],20)
            Position_Control(bus2,ID_2,Theta_HL[0, 1],20)
            Position_Control(bus2,ID_3,Theta_HL[0, 2],20)
            Position_Control(bus3,ID_1,Theta_HR[0, 0],20)
            Position_Control(bus3,ID_2,Theta_HR[0, 1],20)
            Position_Control(bus3,ID_3,Theta_HR[0, 2],20)

            time.sleep(5)

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