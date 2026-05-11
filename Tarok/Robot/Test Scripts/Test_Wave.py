# Test script for Standing on 3 legs and waving one leg in the air
# Start in standing pose 
# -> Move COM towards hind left (move legs towards front right)
# -> Lift and wave front right leg
# -> Move all the way back to stand

# Imports
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../..'))) # Change level of path based on file location (the ../../)
import can
import time
import numpy as np
from Robot import*

# CAN initialization in terminal: "for i in 0 1 2 3; do sudo ip link set dev can$i up type can bitrate 1000000 && sudo ip link set can$i txqueuelen 1000; done"

# PI Parameters:
# Stand state PI parameters (test 10)
pi_stand = {
    'angle_kp':  120,
    'angle_ki':  25,
    'speed_kp':  60,
    'speed_ki':  10,
    'torque_kp': 60,
    'torque_ki': 13
}

# Low Battery state PI parameters (Compromise parameters)
pi_low_battery = {
    'angle_kp':  110,
    'angle_ki':  50,
    'speed_kp':  55,
    'speed_ki':  20,
    'torque_kp': 55,
    'torque_ki': 25
}

# Up/Down state PI parameters (compromise parameters)
pi_up_down = {
    'angle_kp':  110,
    'angle_ki':  50,
    'speed_kp':  55,
    'speed_ki':  20,
    'torque_kp': 55,
    'torque_ki': 25
}

# Bezier walk state PI parameters - Tuned parameters (from tuning 30/4 2026)
pi_bezier_walk = {
    'angle_kp':  120,
    'angle_ki':  90,
    'speed_kp':  60,
    'speed_ki':  80,
    'torque_kp': 60,
    'torque_ki': 60
}

### SCRIPT START ###
## PRECOMPUTATIONS ##
print("Performing pre-computations...")

# Offsets for COM transfer during stand phase
x_offset = 0.06 # [m] how much to move COM forward during transfer
y_offset = 0.05 # [m] how much to move COM to the left during transfer

# Time parameters
dt = 0.005 # seconds (200 Hz)

# Offset Time parameters
Offset_Transfer_Time = 4 # Seconds
Offset_Transfer_Steps = int(Offset_Transfer_Time / dt) 
t_Offset_Transfer = np.linspace(0, Offset_Transfer_Time, Offset_Transfer_Steps)

# Wave Time parameters
Wave_Time = 10 # Seconds
Wave_Time_Steps = int(Wave_Time / dt) 
t_Wave = np.linspace(0, Wave_Time, Wave_Time_Steps)

# ----------------------------- #
### STAND GENERATION ###
# ----------------------------- #

# Define kinematic body lengths
l_k = 0.7048  # Length of body in kinematic model (meters)
w_k = 0.220   # Width of body in kinematic model (meters)

# Define desired end-effector position for standing posture (z=-0.41 meters) for all 4 legs
x_FL_stand = x_FR_stand = l_k/2 # x-position of front legs
x_HL_stand = x_HR_stand = -l_k/2 # x-position of hind legs
y_FL_stand = y_HL_stand = w_k/2 + 0.078 # y-position of left legs
y_FR_stand = y_HR_stand = -w_k/2 - 0.078 # y-position of right legs
z_stand = -0.41 # z-position for standing posture (similar for all legs)

# Combine trajectories into position arrays for each leg
P_FL_body_stand = np.array([x_FL_stand, y_FL_stand, z_stand])
P_FR_body_stand = np.array([x_FR_stand, y_FR_stand, z_stand])
P_HL_body_stand = np.array([x_HL_stand, y_HL_stand, z_stand])
P_HR_body_stand = np.array([x_HR_stand, y_HR_stand, z_stand])

# Transform desired end-effector trajectory from body frame to leg base frames
P_FL_base_stand = T0_B(P_FL_body_stand.reshape((3,1)), 'FL')
P_FR_base_stand = T0_B(P_FR_body_stand.reshape((3,1)), 'FR')
P_HL_base_stand = T0_B(P_HL_body_stand.reshape((3,1)), 'HL')
P_HR_base_stand = T0_B(P_HR_body_stand.reshape((3,1)), 'HR')

# Determine joint angles for all 4 legs using inverse kinematics
Theta_FL_stand = Inverse_Kinematics(P_FL_base_stand, 'FL')
Theta_FR_stand = Inverse_Kinematics(P_FR_base_stand, 'FR')
Theta_HL_stand = Inverse_Kinematics(P_HL_base_stand, 'HL')
Theta_HR_stand = Inverse_Kinematics(P_HR_base_stand, 'HR')

# Convert angles from radians to degrees for motor control
Theta_FL_stand = np.degrees(Theta_FL_stand)
Theta_FR_stand = np.degrees(Theta_FR_stand)
Theta_HL_stand = np.degrees(Theta_HL_stand)
Theta_HR_stand = np.degrees(Theta_HR_stand)

# ----------------------------- #
### OFFSET GENERATION ###
# ----------------------------- #

# Compute cos interpolation from stand position to offset position for each leg
P_FL_body_Offset = cos_interp(t_Offset_Transfer, P_FL_body_stand.reshape((3, 1)), P_FL_body_stand.reshape((3, 1)) + np.array([[x_offset], [-y_offset], [0]]), 0, Offset_Transfer_Time)
P_FR_body_Offset = cos_interp(t_Offset_Transfer, P_FR_body_stand.reshape((3, 1)), P_FR_body_stand.reshape((3, 1)) + np.array([[x_offset], [-y_offset], [0]]), 0, Offset_Transfer_Time)
P_HL_body_Offset = cos_interp(t_Offset_Transfer, P_HL_body_stand.reshape((3, 1)), P_HL_body_stand.reshape((3, 1)) + np.array([[x_offset], [-y_offset], [0]]), 0, Offset_Transfer_Time)
P_HR_body_Offset = cos_interp(t_Offset_Transfer, P_HR_body_stand.reshape((3, 1)), P_HR_body_stand.reshape((3, 1)) + np.array([[x_offset], [-y_offset], [0]]), 0, Offset_Transfer_Time)

# Compute cos interpolation derivatives from stand position to offset position for each leg
V_FL_body_Offset = cos_interp_dot(t_Offset_Transfer, P_FL_body_stand.reshape((3, 1)), P_FL_body_stand.reshape((3, 1)) + np.array([[x_offset], [-y_offset], [0]]), 0, Offset_Transfer_Time)
V_FR_body_Offset = cos_interp_dot(t_Offset_Transfer, P_FR_body_stand.reshape((3, 1)), P_FR_body_stand.reshape((3, 1)) + np.array([[x_offset], [-y_offset], [0]]), 0, Offset_Transfer_Time)
V_HL_body_Offset = cos_interp_dot(t_Offset_Transfer, P_HL_body_stand.reshape((3, 1)), P_HL_body_stand.reshape((3, 1)) + np.array([[x_offset], [-y_offset], [0]]), 0, Offset_Transfer_Time)
V_HR_body_Offset = cos_interp_dot(t_Offset_Transfer, P_HR_body_stand.reshape((3, 1)), P_HR_body_stand.reshape((3, 1)) + np.array([[x_offset], [-y_offset], [0]]), 0, Offset_Transfer_Time)

### TRANSFORMATIONS ###
# Transform the stand to walk trajectory from Body Frame to Leg Base Frames
P_FL_Base_Offset = np.array([T0_B(P_FL_body_Offset[:, i].reshape((3, 1)), 'FL') for i in range((len(t_Offset_Transfer)))])
P_FR_Base_Offset = np.array([T0_B(P_FR_body_Offset[:, i].reshape((3, 1)), 'FR') for i in range((len(t_Offset_Transfer)))])
P_HL_Base_Offset = np.array([T0_B(P_HL_body_Offset[:, i].reshape((3, 1)), 'HL') for i in range((len(t_Offset_Transfer)))])
P_HR_Base_Offset = np.array([T0_B(P_HR_body_Offset[:, i].reshape((3, 1)), 'HR') for i in range((len(t_Offset_Transfer)))])

# Transform desired end-effector velocity from body frame to leg base frames
V_FL_base_Offset = np.array([R0_B(V_FL_body_Offset[:, i].reshape((3, 1)), 'FL') for i in range((len(t_Offset_Transfer)))])
V_FR_base_Offset = np.array([R0_B(V_FR_body_Offset[:, i].reshape((3, 1)), 'FR') for i in range((len(t_Offset_Transfer)))])
V_HL_base_Offset = np.array([R0_B(V_HL_body_Offset[:, i].reshape((3, 1)), 'HL') for i in range((len(t_Offset_Transfer)))])
V_HR_base_Offset = np.array([R0_B(V_HR_body_Offset[:, i].reshape((3, 1)), 'HR') for i in range((len(t_Offset_Transfer)))])

### Kinematics ###
# Determine joint angles for all 4 legs using inverse kinematics
Theta_FL_Offset = np.array([Inverse_Kinematics(P_FL_Base_Offset[i], 'FL') for i in range((len(t_Offset_Transfer)))]) # Shape (num_time_steps, 3), containing theta1, theta2, theta3 for each time step
Theta_FR_Offset = np.array([Inverse_Kinematics(P_FR_Base_Offset[i], 'FR') for i in range((len(t_Offset_Transfer)))]) # Shape (num_time_steps, 3), containing theta1, theta2, theta3 for each time step
Theta_HL_Offset = np.array([Inverse_Kinematics(P_HL_Base_Offset[i], 'HL') for i in range((len(t_Offset_Transfer)))]) # Shape (num_time_steps, 3), containing theta1, theta2, theta3 for each time step
Theta_HR_Offset = np.array([Inverse_Kinematics(P_HR_Base_Offset[i], 'HR') for i in range((len(t_Offset_Transfer)))]) # Shape (num_time_steps, 3), containing theta1, theta2, theta3 for each time step

# Determine joint velocities for all 4 legs using Jacobian
# Damped least squares inverse to avoid singularities - theta_dot = (J^T*J + damp^2*I)^-1 * J^T * cartesian_velocity 
Theta_dot_FL_Offset = np.zeros((3, len(t_Offset_Transfer)))  # Initialize joint velocity array
Theta_dot_FR_Offset = np.zeros((3, len(t_Offset_Transfer)))  # Initialize joint velocity array
Theta_dot_HL_Offset = np.zeros((3, len(t_Offset_Transfer)))  # Initialize joint velocity array
Theta_dot_HR_Offset = np.zeros((3, len(t_Offset_Transfer)))  # Initialize joint velocity array
damp = 0.001  # Damping factor
for i in range((int(Offset_Transfer_Time / dt))):
    Jac_i_FL = Jacobian(Theta_FL_Offset[i, 0], Theta_FL_Offset[i, 1], Theta_FL_Offset[i, 2], 'FL')
    Jac_i_FR = Jacobian(Theta_FR_Offset[i, 0], Theta_FR_Offset[i, 1], Theta_FR_Offset[i, 2], 'FR')
    Jac_i_HL = Jacobian(Theta_HL_Offset[i, 0], Theta_HL_Offset[i, 1], Theta_HL_Offset[i, 2], 'HL')
    Jac_i_HR = Jacobian(Theta_HR_Offset[i, 0], Theta_HR_Offset[i, 1], Theta_HR_Offset[i, 2], 'HR')
    JT_FL = Jac_i_FL.T
    JT_FR = Jac_i_FR.T
    JT_HL = Jac_i_HL.T
    JT_HR = Jac_i_HR.T
    term_FL = JT_FL @ Jac_i_FL + (damp**2)*np.eye(3)
    term_FR = JT_FR @ Jac_i_FR + (damp**2)*np.eye(3)
    term_HL = JT_HL @ Jac_i_HL + (damp**2)*np.eye(3)
    term_HR = JT_HR @ Jac_i_HR + (damp**2)*np.eye(3)
    Theta_dot_FL_Offset[:, i] = np.linalg.solve(term_FL, JT_FL @ V_FL_base_Offset[i].flatten())
    Theta_dot_FR_Offset[:, i] = np.linalg.solve(term_FR, JT_FR @ V_FR_base_Offset[i].flatten())
    Theta_dot_HL_Offset[:, i] = np.linalg.solve(term_HL, JT_HL @ V_HL_base_Offset[i].flatten())
    Theta_dot_HR_Offset[:, i] = np.linalg.solve(term_HR, JT_HR @ V_HR_base_Offset[i].flatten())

# Convert joint angles and velocities to degrees and abs(degrees/s) for right units for motor control
Theta_FL_Offset = np.rad2deg(Theta_FL_Offset)
Theta_FR_Offset = np.rad2deg(Theta_FR_Offset)
Theta_HL_Offset = np.rad2deg(Theta_HL_Offset)
Theta_HR_Offset = np.rad2deg(Theta_HR_Offset)
Theta_dot_FL_Offset = np.abs(np.rad2deg(Theta_dot_FL_Offset))
Theta_dot_FR_Offset = np.abs(np.rad2deg(Theta_dot_FR_Offset))
Theta_dot_HL_Offset = np.abs(np.rad2deg(Theta_dot_HL_Offset))
Theta_dot_HR_Offset = np.abs(np.rad2deg(Theta_dot_HR_Offset))

# ----------------------------- #
### WAVE GENERATION ###
# ----------------------------- #

# Generate stand trajectory for 3 legs
P_FL_body_Wave = np.tile(P_FL_body_Offset[:, -1].reshape((3, 1)), (1, len(t_Wave))) # Shape (3, len(t_Wave)), where each column is the offset position
P_HL_body_Wave = np.tile(P_HL_body_Offset[:, -1].reshape((3, 1)), (1, len(t_Wave))) # Shape (3, len(t_Wave)), where each column is the offset position
P_HR_body_Wave = np.tile(P_HR_body_Offset[:, -1].reshape((3, 1)), (1, len(t_Wave))) # Shape (3, len(t_Wave)), where each column is the offset position)

# Generate stand velocites for 3 legs
V_FL_body_Wave = np.zeros((3, len(t_Wave))) # Shape (3, len(t_Wave)), where each column is zero velocity
V_HL_body_Wave = np.zeros((3, len(t_Wave))) # Shape (3, len(t_Wave)), where each column is zero velocity
V_HR_body_Wave = np.zeros((3, len(t_Wave))) # Shape (3, len(t_Wave)), where each column is zero velocity

# Generate wave trajectory for front right leg
# Move foot 25 cm forward and 15 cm up  in 3 seconds using cos interpolation
# Then move foot 5 cm up and 5 down  2 times using cos interpolation with a period of 2 seonds 
# Then move foot back to offset position in 3 seconds using cos interpolation

# Time parameters for wave trajectory segments
t_Stand_To_Wave = np.linspace(0, 3, int(3 / dt))
t_Wave_Up_Down = np.linspace(0, 1, int(1 / dt))

# Compute wave trajectory for front right leg
P_FR_body_Stand_To_Wave = cos_interp(t_Stand_To_Wave, P_FR_body_Offset[:, -1].reshape((3, 1)), P_FR_body_Offset[:, -1].reshape((3, 1)) + np.array([[0.25], [0], [0.15]]), 0, 3)
P_FR_body_Wave_Up = cos_interp(t_Wave_Up_Down, P_FR_body_Stand_To_Wave[:, -1].reshape((3, 1)), P_FR_body_Offset[:, -1].reshape((3, 1)) + np.array([[0.25], [0], [0.25]]), 0, 1)
P_FR_body_Wave_Down = cos_interp(t_Wave_Up_Down, P_FR_body_Wave_Up[:, -1].reshape((3, 1)), P_FR_body_Offset[:, -1].reshape((3, 1)) + np.array([[0.25], [0], [0.15]]), 0, 1)
P_FR_body_Wave_To_Stand = cos_interp(t_Stand_To_Wave, P_FR_body_Wave_Down[:, -1].reshape((3, 1)), P_FR_body_Offset[:, -1].reshape((3, 1)), 0, 3)
# Combine wave trajectory segments for front right leg
P_FR_body_Wave = np.hstack((P_FR_body_Stand_To_Wave, P_FR_body_Wave_Up, P_FR_body_Wave_Down, P_FR_body_Wave_Up, P_FR_body_Wave_Down, P_FR_body_Wave_To_Stand))

# Compute wave velocities for front right leg
V_FR_body_Stand_To_Wave = cos_interp_dot(t_Stand_To_Wave, P_FR_body_Offset[:, -1].reshape((3, 1)), P_FR_body_Offset[:, -1].reshape((3, 1)) + np.array([[0.25], [0], [0.15]]), 0, 3)
V_FR_body_Wave_Up = cos_interp_dot(t_Wave_Up_Down, P_FR_body_Stand_To_Wave[:, -1].reshape((3, 1)), P_FR_body_Offset[:, -1].reshape((3, 1)) + np.array([[0.25], [0], [0.25]]), 0, 1)
V_FR_body_Wave_Down = cos_interp_dot(t_Wave_Up_Down, P_FR_body_Wave_Up[:, -1].reshape((3, 1)), P_FR_body_Offset[:, -1].reshape((3, 1)) + np.array([[0.25], [0], [0.15]]), 0, 1)
V_FR_body_Wave_To_Stand = cos_interp_dot(t_Stand_To_Wave, P_FR_body_Wave_Down[:, -1].reshape((3, 1)), P_FR_body_Offset[:, -1].reshape((3, 1)), 0, 3)
# Combine wave velocity segments for front right leg
V_FR_body_Wave = np.hstack((V_FR_body_Stand_To_Wave, V_FR_body_Wave_Up, V_FR_body_Wave_Down, V_FR_body_Wave_Up, V_FR_body_Wave_Down, V_FR_body_Wave_To_Stand))

### TRANSFORMATIONS ###
# Transform the stand to walk trajectory from Body Frame to Leg Base Frames
P_FL_Base_Wave = np.array([T0_B(P_FL_body_Wave[:, i].reshape((3, 1)), 'FL') for i in range((len(t_Wave)))])
P_FR_Base_Wave = np.array([T0_B(P_FR_body_Wave[:, i].reshape((3, 1)), 'FR') for i in range((len(t_Wave)))])
P_HL_Base_Wave = np.array([T0_B(P_HL_body_Wave[:, i].reshape((3, 1)), 'HL') for i in range((len(t_Wave)))])
P_HR_Base_Wave = np.array([T0_B(P_HR_body_Wave[:, i].reshape((3, 1)), 'HR') for i in range((len(t_Wave)))])

# Transform desired end-effector velocity from body frame to leg base frames
V_FL_base_Wave = np.array([R0_B(V_FL_body_Wave[:, i].reshape((3, 1)), 'FL') for i in range((len(t_Wave)))])
V_FR_base_Wave = np.array([R0_B(V_FR_body_Wave[:, i].reshape((3, 1)), 'FR') for i in range((len(t_Wave)))])
V_HL_base_Wave = np.array([R0_B(V_HL_body_Wave[:, i].reshape((3, 1)), 'HL') for i in range((len(t_Wave)))])
V_HR_base_Wave = np.array([R0_B(V_HR_body_Wave[:, i].reshape((3, 1)), 'HR') for i in range((len(t_Wave)))])

### Kinematics ###
# Determine joint angles for all 4 legs using inverse kinematics
Theta_FL_Wave = np.array([Inverse_Kinematics(P_FL_Base_Wave[i], 'FL') for i in range((len(t_Wave)))]) # Shape (num_time_steps, 3), containing theta1, theta2, theta3 for each time step
Theta_FR_Wave = np.array([Inverse_Kinematics(P_FR_Base_Wave[i], 'FR') for i in range((len(t_Wave)))]) # Shape (num_time_steps, 3), containing theta1, theta2, theta3 for each time step
Theta_HL_Wave = np.array([Inverse_Kinematics(P_HL_Base_Wave[i], 'HL') for i in range((len(t_Wave)))]) # Shape (num_time_steps, 3), containing theta1, theta2, theta3 for each time step
Theta_HR_Wave = np.array([Inverse_Kinematics(P_HR_Base_Wave[i], 'HR') for i in range((len(t_Wave)))]) # Shape (num_time_steps, 3), containing theta1, theta2, theta3 for each time step

# Determine joint velocities for all 4 legs using Jacobian
# Damped least squares inverse to avoid singularities - theta_dot = (J^T*J + damp^2*I)^-1 * J^T * cartesian_velocity 
Theta_dot_FL_Wave = np.zeros((3, len(t_Wave)))  # Initialize joint velocity array
Theta_dot_FR_Wave = np.zeros((3, len(t_Wave)))  # Initialize joint velocity array
Theta_dot_HL_Wave = np.zeros((3, len(t_Wave)))  # Initialize joint velocity array
Theta_dot_HR_Wave = np.zeros((3, len(t_Wave)))  # Initialize joint velocity array
damp = 0.001  # Damping factor
for i in range((int(Wave_Time / dt))):
    Jac_i_FL = Jacobian(Theta_FL_Wave[i, 0], Theta_FL_Wave[i, 1], Theta_FL_Wave[i, 2], 'FL')
    Jac_i_FR = Jacobian(Theta_FR_Wave[i, 0], Theta_FR_Wave[i, 1], Theta_FR_Wave[i, 2], 'FR')
    Jac_i_HL = Jacobian(Theta_HL_Wave[i, 0], Theta_HL_Wave[i, 1], Theta_HL_Wave[i, 2], 'HL')
    Jac_i_HR = Jacobian(Theta_HR_Wave[i, 0], Theta_HR_Wave[i, 1], Theta_HR_Wave[i, 2], 'HR')
    JT_FL = Jac_i_FL.T
    JT_FR = Jac_i_FR.T
    JT_HL = Jac_i_HL.T
    JT_HR = Jac_i_HR.T
    term_FL = JT_FL @ Jac_i_FL + (damp**2)*np.eye(3)
    term_FR = JT_FR @ Jac_i_FR + (damp**2)*np.eye(3)
    term_HL = JT_HL @ Jac_i_HL + (damp**2)*np.eye(3)
    term_HR = JT_HR @ Jac_i_HR + (damp**2)*np.eye(3)
    Theta_dot_FL_Wave[:, i] = np.linalg.solve(term_FL, JT_FL @ V_FL_base_Wave[i].flatten())
    Theta_dot_FR_Wave[:, i] = np.linalg.solve(term_FR, JT_FR @ V_FR_base_Wave[i].flatten())
    Theta_dot_HL_Wave[:, i] = np.linalg.solve(term_HL, JT_HL @ V_HL_base_Wave[i].flatten())
    Theta_dot_HR_Wave[:, i] = np.linalg.solve(term_HR, JT_HR @ V_HR_base_Wave[i].flatten())

# Convert joint angles and velocities to degrees and abs(degrees/s) for right units for motor control
Theta_FL_Wave = np.rad2deg(Theta_FL_Wave)
Theta_FR_Wave = np.rad2deg(Theta_FR_Wave)
Theta_HL_Wave = np.rad2deg(Theta_HL_Wave)
Theta_HR_Wave = np.rad2deg(Theta_HR_Wave)
Theta_dot_FL_Wave = np.abs(np.rad2deg(Theta_dot_FL_Wave))
Theta_dot_FR_Wave = np.abs(np.rad2deg(Theta_dot_FR_Wave))
Theta_dot_HL_Wave = np.abs(np.rad2deg(Theta_dot_HL_Wave))
Theta_dot_HR_Wave = np.abs(np.rad2deg(Theta_dot_HR_Wave))

# ----------------------------- #
### RETURN ARRAYS ###
# ----------------------------- #

# Create arrays for return to stand pose using reversed trajectories
Theta_FL_Offset_Return = Theta_FL_Offset[::-1, :]
Theta_FR_Offset_Return = Theta_FR_Offset[::-1, :]
Theta_HL_Offset_Return = Theta_HL_Offset[::-1, :]
Theta_HR_Offset_Return = Theta_HR_Offset[::-1, :]
Theta_dot_FL_Offset_Return = Theta_dot_FL_Offset[:, ::-1]
Theta_dot_FR_Offset_Return = Theta_dot_FR_Offset[:, ::-1]
Theta_dot_HL_Offset_Return = Theta_dot_HL_Offset[:, ::-1]
Theta_dot_HR_Offset_Return = Theta_dot_HR_Offset[:, ::-1]

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

print("Initialization complete, starting pre-loop sequence...")

## PRE-LOOP SEQUENCE ##

# Write PI parameters to motors

print("Writing PI parameters to motors...")
PID_RAM_Control(bus0,ID_1, pi_stand)
PID_RAM_Control(bus0,ID_2, pi_stand)
PID_RAM_Control(bus0,ID_3, pi_stand)
PID_RAM_Control(bus1,ID_1, pi_stand)
PID_RAM_Control(bus1,ID_2, pi_stand)
PID_RAM_Control(bus1,ID_3, pi_stand)
PID_RAM_Control(bus2,ID_1, pi_stand)  
PID_RAM_Control(bus2,ID_2, pi_stand)
PID_RAM_Control(bus2,ID_3, pi_stand)
PID_RAM_Control(bus3,ID_1, pi_stand)
PID_RAM_Control(bus3,ID_2, pi_stand)
PID_RAM_Control(bus3,ID_3, pi_stand)

time.sleep(0.2) # Sleep for a short time to ensure parameters are written before starting loop, adjust as needed

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

time.sleep(6)

# Move to Stand position
print("Moved to zero position, moving to stand position...")
Position_Control(bus0,ID_1,Theta_FL_stand[0],30)
Position_Control(bus0,ID_2,Theta_FL_stand[1],30)
Position_Control(bus0,ID_3,Theta_FL_stand[2],30)
Position_Control(bus1,ID_1,Theta_FR_stand[0],30)
Position_Control(bus1,ID_2,Theta_FR_stand[1],30)
Position_Control(bus1,ID_3,Theta_FR_stand[2],30)
Position_Control(bus2,ID_1,Theta_HL_stand[0],30)
Position_Control(bus2,ID_2,Theta_HL_stand[1],30)
Position_Control(bus2,ID_3,Theta_HL_stand[2],30)
Position_Control(bus3,ID_1,Theta_HR_stand[0],30)
Position_Control(bus3,ID_2,Theta_HR_stand[1],30)
Position_Control(bus3,ID_3,Theta_HR_stand[2],30)

time.sleep(10)

# Move to offset position
print("Moved to stand position, moving to offset position...")

print("Writing PI parameters to motors...")
PID_RAM_Control(bus0,ID_1, pi_bezier_walk)
PID_RAM_Control(bus0,ID_2, pi_bezier_walk)
PID_RAM_Control(bus0,ID_3, pi_bezier_walk)
PID_RAM_Control(bus1,ID_1, pi_bezier_walk)
PID_RAM_Control(bus1,ID_2, pi_bezier_walk)
PID_RAM_Control(bus1,ID_3, pi_bezier_walk)
PID_RAM_Control(bus2,ID_1, pi_bezier_walk)  
PID_RAM_Control(bus2,ID_2, pi_bezier_walk)
PID_RAM_Control(bus2,ID_3, pi_bezier_walk)
PID_RAM_Control(bus3,ID_1, pi_bezier_walk)
PID_RAM_Control(bus3,ID_2, pi_bezier_walk)
PID_RAM_Control(bus3,ID_3, pi_bezier_walk)
time.sleep(0.1) # Sleep for a short time to ensure parameters are written before starting loop, adjust as needed

Offset_Statement = True
# Note start time
start_time = cycle_start = current_time = time.monotonic()
# Run until transition is done
while Offset_Statement:
    # Time Management
    current_time = time.monotonic()
    elapsed_cycle = current_time - cycle_start
    if elapsed_cycle >= Offset_Transfer_Time:
        Offset_Statement = False
        continue

    # Find closest value in t to elapsed in current cycle
    index = min(int(elapsed_cycle / dt), len(t_Offset_Transfer) - 1)

    # Send position control commands to motors for current time step
    Position_Control(bus0, ID_1, Theta_FL_Offset[index, 0], Theta_dot_FL_Offset[0, index])
    Position_Control(bus0, ID_2, Theta_FL_Offset[index, 1], Theta_dot_FL_Offset[1, index])
    Position_Control(bus0, ID_3, Theta_FL_Offset[index, 2], Theta_dot_FL_Offset[2, index])
    Position_Control(bus1, ID_1, Theta_FR_Offset[index, 0], Theta_dot_FR_Offset[0, index])
    Position_Control(bus1, ID_2, Theta_FR_Offset[index, 1], Theta_dot_FR_Offset[1, index])
    Position_Control(bus1, ID_3, Theta_FR_Offset[index, 2], Theta_dot_FR_Offset[2, index])
    Position_Control(bus2, ID_1, Theta_HL_Offset[index, 0], Theta_dot_HL_Offset[0, index])
    Position_Control(bus2, ID_2, Theta_HL_Offset[index, 1], Theta_dot_HL_Offset[1, index])
    Position_Control(bus2, ID_3, Theta_HL_Offset[index, 2], Theta_dot_HL_Offset[2, index])
    Position_Control(bus3, ID_1, Theta_HR_Offset[index, 0], Theta_dot_HR_Offset[0, index])
    Position_Control(bus3, ID_2, Theta_HR_Offset[index, 1], Theta_dot_HR_Offset[1, index])
    Position_Control(bus3, ID_3, Theta_HR_Offset[index, 2], Theta_dot_HR_Offset[2, index])

time.sleep(1) # Sleep for a short time to ensure transition to walk start position is complete before starting loop, adjust as needed

# Do Wave motion for 1 cycle
print("Offset position reached, performing wave motion...")

Wave_Statement = True
# Note start time
start_time = cycle_start = current_time = time.monotonic()
# Run until transition is done
while Wave_Statement:
    # Time Management
    current_time = time.monotonic()
    elapsed_cycle = current_time - cycle_start
    if elapsed_cycle >= Wave_Time:
        Wave_Statement = False
        continue

    # Find closest value in t to elapsed in current cycle
    index = min(int(elapsed_cycle / dt), len(t_Wave) - 1)

    # Send position control commands to motors for current time step
    Position_Control(bus0, ID_1, Theta_FL_Wave[index, 0], Theta_dot_FL_Wave[0, index])
    Position_Control(bus0, ID_2, Theta_FL_Wave[index, 1], Theta_dot_FL_Wave[1, index])
    Position_Control(bus0, ID_3, Theta_FL_Wave[index, 2], Theta_dot_FL_Wave[2, index])
    Position_Control(bus1, ID_1, Theta_FR_Wave[index, 0], Theta_dot_FR_Wave[0, index])
    Position_Control(bus1, ID_2, Theta_FR_Wave[index, 1], Theta_dot_FR_Wave[1, index])
    Position_Control(bus1, ID_3, Theta_FR_Wave[index, 2], Theta_dot_FR_Wave[2, index])
    Position_Control(bus2, ID_1, Theta_HL_Wave[index, 0], Theta_dot_HL_Wave[0, index])
    Position_Control(bus2, ID_2, Theta_HL_Wave[index, 1], Theta_dot_HL_Wave[1, index])
    Position_Control(bus2, ID_3, Theta_HL_Wave[index, 2], Theta_dot_HL_Wave[2, index])
    Position_Control(bus3, ID_1, Theta_HR_Wave[index, 0], Theta_dot_HR_Wave[0, index])
    Position_Control(bus3, ID_2, Theta_HR_Wave[index, 1], Theta_dot_HR_Wave[1, index])
    Position_Control(bus3, ID_3, Theta_HR_Wave[index, 2], Theta_dot_HR_Wave[2, index])

time.sleep(1) # Sleep for a short time to ensure wave motion is complete before starting next transition, adjust as needed

Offset_Statement = True
# Note start time
start_time = cycle_start = current_time = time.monotonic()
# Run until transition is done
while Offset_Statement:
    # Time Management
    current_time = time.monotonic()
    elapsed_cycle = current_time - cycle_start
    if elapsed_cycle >= Offset_Transfer_Time:
        Offset_Statement = False
        continue

    # Find closest value in t to elapsed in current cycle
    index = min(int(elapsed_cycle / dt), len(t_Offset_Transfer) - 1)

    # Send position control commands to motors for current time step
    Position_Control(bus0, ID_1, Theta_FL_Offset_Return[index, 0], Theta_dot_FL_Offset_Return[0, index])
    Position_Control(bus0, ID_2, Theta_FL_Offset_Return[index, 1], Theta_dot_FL_Offset_Return[1, index])
    Position_Control(bus0, ID_3, Theta_FL_Offset_Return[index, 2], Theta_dot_FL_Offset_Return[2, index])
    Position_Control(bus1, ID_1, Theta_FR_Offset_Return[index, 0], Theta_dot_FR_Offset_Return[0, index])
    Position_Control(bus1, ID_2, Theta_FR_Offset_Return[index, 1], Theta_dot_FR_Offset_Return[1, index])
    Position_Control(bus1, ID_3, Theta_FR_Offset_Return[index, 2], Theta_dot_FR_Offset_Return[2, index])
    Position_Control(bus2, ID_1, Theta_HL_Offset_Return[index, 0], Theta_dot_HL_Offset_Return[0, index])
    Position_Control(bus2, ID_2, Theta_HL_Offset_Return[index, 1], Theta_dot_HL_Offset_Return[1, index])
    Position_Control(bus2, ID_3, Theta_HL_Offset_Return[index, 2], Theta_dot_HL_Offset_Return[2, index])
    Position_Control(bus3, ID_1, Theta_HR_Offset_Return[index, 0], Theta_dot_HR_Offset_Return[0, index])
    Position_Control(bus3, ID_2, Theta_HR_Offset_Return[index, 1], Theta_dot_HR_Offset_Return[1, index])
    Position_Control(bus3, ID_3, Theta_HR_Offset_Return[index, 2], Theta_dot_HR_Offset_Return[2, index])

time.sleep(1) # Sleep for a short time to ensure transition to walk start position is complete before starting loop, adjust as needed

print("Pre-loop sequence complete, starting loop...")

## MAIN LOOP ##
# The loop is simply standing in the stand pose until shutdown 

print("Writing PI parameters to motors...")
PID_RAM_Control(bus0,ID_1, pi_stand)
PID_RAM_Control(bus0,ID_2, pi_stand)
PID_RAM_Control(bus0,ID_3, pi_stand)
PID_RAM_Control(bus1,ID_1, pi_stand)
PID_RAM_Control(bus1,ID_2, pi_stand)
PID_RAM_Control(bus1,ID_3, pi_stand)
PID_RAM_Control(bus2,ID_1, pi_stand)  
PID_RAM_Control(bus2,ID_2, pi_stand)
PID_RAM_Control(bus2,ID_3, pi_stand)
PID_RAM_Control(bus3,ID_1, pi_stand)
PID_RAM_Control(bus3,ID_2, pi_stand)
PID_RAM_Control(bus3,ID_3, pi_stand)

print("Loop started - Press ctrl+c in terminal for shutdown")

try:
    while True:
            Position_Control(bus0,ID_1,Theta_FL_stand[0],20)
            Position_Control(bus0,ID_2,Theta_FL_stand[1],20)
            Position_Control(bus0,ID_3,Theta_FL_stand[2],20)
            Position_Control(bus1,ID_1,Theta_FR_stand[0],20)
            Position_Control(bus1,ID_2,Theta_FR_stand[1],20)
            Position_Control(bus1,ID_3,Theta_FR_stand[2],20)
            Position_Control(bus2,ID_1,Theta_HL_stand[0],20)
            Position_Control(bus2,ID_2,Theta_HL_stand[1],20)
            Position_Control(bus2,ID_3,Theta_HL_stand[2],20)
            Position_Control(bus3,ID_1,Theta_HR_stand[0],20)
            Position_Control(bus3,ID_2,Theta_HR_stand[1],20)
            Position_Control(bus3,ID_3,Theta_HR_stand[2],20)

            time.sleep(1) # Sleep for a short time to prevent spamming commands, adjust as needed

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