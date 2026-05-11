# Visualization script for Standing on 3 legs and waving one leg in the air
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

import matplotlib.pyplot as plt
import matplotlib.animation as animation

# CAN initialization in terminal: "for i in 0 1 2 3; do sudo ip link set dev can$i up type can bitrate 1000000 && sudo ip link set can$i txqueuelen 1000; done"

### SCRIPT START ###
## PRECOMPUTATIONS ##
print("Performing pre-computations...")

# Parameters From Tarok Dimensions
Tarok = TarokDymensions()
COLORS = Tarok.COLORS
PHASE_OFFSET = Tarok.CRAWL_OFFSETS_Mixed

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
# Move foot 20 cm forward and 10 cm up  in 3 seconds using cos interpolation
# Then move foot 5 cm up and 5 down  2 times using cos interpolation with a period of 2 seonds 
# Then move foot back to offset position in 3 seconds using cos interpolation

# Time parameters for wave trajectory segments
t_Stand_To_Wave = np.linspace(0, 3, int(3 / dt))
t_Wave_Up_Down = np.linspace(0, 1, int(1 / dt))

# Compute wave trajectory for front right leg
P_FR_body_Stand_To_Wave = cos_interp(t_Stand_To_Wave, P_FR_body_Offset[:, -1].reshape((3, 1)), P_FR_body_Offset[:, -1].reshape((3, 1)) + np.array([[0.2], [0], [0.1]]), 0, 3)
P_FR_body_Wave_Up = cos_interp(t_Wave_Up_Down, P_FR_body_Stand_To_Wave[:, -1].reshape((3, 1)), P_FR_body_Offset[:, -1].reshape((3, 1)) + np.array([[0.2], [0], [0.2]]), 0, 1)
P_FR_body_Wave_Down = cos_interp(t_Wave_Up_Down, P_FR_body_Wave_Up[:, -1].reshape((3, 1)), P_FR_body_Offset[:, -1].reshape((3, 1)) + np.array([[0.2], [0], [0.1]]), 0, 1)
P_FR_body_Wave_To_Stand = cos_interp(t_Stand_To_Wave, P_FR_body_Wave_Down[:, -1].reshape((3, 1)), P_FR_body_Offset[:, -1].reshape((3, 1)), 0, 3)
# Combine wave trajectory segments for front right leg
P_FR_body_Wave = np.hstack((P_FR_body_Stand_To_Wave, P_FR_body_Wave_Up, P_FR_body_Wave_Down, P_FR_body_Wave_Up, P_FR_body_Wave_Down, P_FR_body_Wave_To_Stand))

# Compute wave velocities for front right leg
V_FR_body_Stand_To_Wave = cos_interp_dot(t_Stand_To_Wave, P_FR_body_Offset[:, -1].reshape((3, 1)), P_FR_body_Offset[:, -1].reshape((3, 1)) + np.array([[0.2], [0], [0.1]]), 0, 3)
V_FR_body_Wave_Up = cos_interp_dot(t_Wave_Up_Down, P_FR_body_Stand_To_Wave[:, -1].reshape((3, 1)), P_FR_body_Offset[:, -1].reshape((3, 1)) + np.array([[0.2], [0], [0.2]]), 0, 1)
V_FR_body_Wave_Down = cos_interp_dot(t_Wave_Up_Down, P_FR_body_Wave_Up[:, -1].reshape((3, 1)), P_FR_body_Offset[:, -1].reshape((3, 1)) + np.array([[0.2], [0], [0.1]]), 0, 1)
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





#### PLOTTING ####

# Print shape of P_FL_body_stand
print("P_FL_body_stand:", P_FL_body_stand.shape)

# Print shape of P_FL_body_Offset
print("P_FL_body_Offset shape:", P_FL_body_Offset.shape)

# Print shape of P_FL_body_Wave
print("P_FL_body_Wave shape:", P_FL_body_Wave.shape)



FL_Trajectory_Stacked = np.hstack((P_FL_body_stand.reshape(3, 1), P_FL_body_Offset, P_FL_body_Wave))
FR_Trajectory_Stacked = np.hstack((P_FR_body_stand.reshape(3, 1), P_FR_body_Offset, P_FR_body_Wave))
HL_Trajectory_Stacked = np.hstack((P_HL_body_stand.reshape(3, 1), P_HL_body_Offset, P_HL_body_Wave))
HR_Trajectory_Stacked = np.hstack((P_HR_body_stand.reshape(3, 1), P_HR_body_Offset, P_HR_body_Wave))


# Print shape of FL_Trajectory_Stacked
print("Shape of FL_Trajectory_Stacked:", FL_Trajectory_Stacked.shape)


# Plot stacked trajectory for all legs
# With a thin vertical line a transition point between each of the 4 trajectory segments (stand height, stand to walk height, walk height to walk start, walk start to walk with transfer)
t_stacked = np.linspace(0, dt + Offset_Transfer_Time + Wave_Time, FL_Trajectory_Stacked.shape[1]) # Create time array for stacked trajectory
plt.subplot(3, 1, 1)
plt.plot(t_stacked, FL_Trajectory_Stacked[0, :], label='FL')
plt.plot(t_stacked, FR_Trajectory_Stacked[0, :], label='FR')
plt.plot(t_stacked, HL_Trajectory_Stacked[0, :], label='HL')
plt.plot(t_stacked, HR_Trajectory_Stacked[0, :], label='HR')
plt.title('Stand to Walk Trajectory')
plt.xlabel('Time (s)')
plt.ylabel('X Position (m)')
plt.legend()
plt.axvline(x=dt, color='k', linestyle='--', linewidth=0.5)
plt.axvline(x=dt + Offset_Transfer_Time, color='k', linestyle='--', linewidth=0.5)
plt.axvline(x=dt + Offset_Transfer_Time + Wave_Time, color='k', linestyle='--', linewidth=0.5)


plt.subplot(3, 1, 2)
plt.plot(t_stacked, FL_Trajectory_Stacked[1, :], label='FL')
plt.plot(t_stacked, FR_Trajectory_Stacked[1, :], label='FR')
plt.plot(t_stacked, HL_Trajectory_Stacked[1, :], label='HL')
plt.plot(t_stacked, HR_Trajectory_Stacked[1, :], label='HR')
plt.xlabel('Time (s)')
plt.ylabel('Y Position (m)')
plt.legend()
plt.axvline(x=dt, color='k', linestyle='--', linewidth=0.5)
plt.axvline(x=dt + Offset_Transfer_Time, color='k', linestyle='--', linewidth=0.5)
plt.axvline(x=dt + Offset_Transfer_Time + Wave_Time, color='k', linestyle='--', linewidth=0.5)
plt.axvline(x=dt + Offset_Transfer_Time + Wave_Time + Wave_Time, color='k', linestyle='--', linewidth=0.5)

plt.subplot(3, 1, 3)
plt.plot(t_stacked, FL_Trajectory_Stacked[2, :], label='FL')
plt.plot(t_stacked, FR_Trajectory_Stacked[2, :], label='FR')
plt.plot(t_stacked, HL_Trajectory_Stacked[2, :], label='HL')
plt.plot(t_stacked, HR_Trajectory_Stacked[2, :], label='HR')
plt.xlabel('Time (s)')
plt.ylabel('Z Position (m)')
plt.legend()
plt.axvline(x=dt, color='k', linestyle='--', linewidth=0.5)
plt.axvline(x=dt + Offset_Transfer_Time, color='k', linestyle='--', linewidth=0.5)
plt.axvline(x=dt + Offset_Transfer_Time + Wave_Time, color='k', linestyle='--', linewidth=0.5)
plt.axvline(x=dt + Offset_Transfer_Time + Wave_Time + Wave_Time, color='k', linestyle='--', linewidth=0.5)
plt.show()










# Stack Theta_XX arrays and plot
Theta_FL_Stacked = np.vstack((Theta_FL_stand, Theta_FL_Offset, Theta_FL_Wave))
Theta_FR_Stacked = np.vstack((Theta_FR_stand, Theta_FR_Offset, Theta_FR_Wave))
Theta_HL_Stacked = np.vstack((Theta_HL_stand, Theta_HL_Offset, Theta_HL_Wave))
Theta_HR_Stacked = np.vstack((Theta_HR_stand, Theta_HR_Offset, Theta_HR_Wave))

# Plot stacked joint angle time series for all legs
# With a thin vertical line a transition point between each of the 4 trajectory segments (stand height, stand to walk height, walk height to walk start, walk start to walk with transfer)
plt.subplot(3, 1, 1)
plt.plot(t_stacked, Theta_FL_Stacked[:, 0], label='FL')
plt.plot(t_stacked, Theta_FR_Stacked[:, 0], label='FR')
plt.plot(t_stacked, Theta_HL_Stacked[:, 0], label='HL')
plt.plot(t_stacked, Theta_HR_Stacked[:, 0], label='HR')
plt.title('Stand to Walk Joint Angles')
plt.xlabel('Time (s)')
plt.ylabel('Theta 1 (degrees)')
plt.legend()
plt.axvline(x=dt, color='k', linestyle='--', linewidth=0.5)
plt.axvline(x=dt + Offset_Transfer_Time, color='k', linestyle='--', linewidth=0.5)
plt.axvline(x=dt + Offset_Transfer_Time + Wave_Time, color='k', linestyle='--', linewidth=0.5)

plt.subplot(3, 1, 2)
plt.plot(t_stacked, Theta_FL_Stacked[:, 1], label='FL')
plt.plot(t_stacked, Theta_FR_Stacked[:, 1], label='FR')
plt.plot(t_stacked, Theta_HL_Stacked[:, 1], label='HL')
plt.plot(t_stacked, Theta_HR_Stacked[:, 1], label='HR')
plt.xlabel('Time (s)')
plt.ylabel('Theta 2 (degrees)')
plt.legend()
plt.axvline(x=dt, color='k', linestyle='--', linewidth=0.5)
plt.axvline(x=dt + Offset_Transfer_Time, color='k', linestyle='--', linewidth=0.5)
plt.axvline(x=dt + Offset_Transfer_Time + Wave_Time, color='k', linestyle='--', linewidth=0.5)

plt.subplot(3, 1, 3)
plt.plot(t_stacked, Theta_FL_Stacked[:, 2], label='FL')
plt.plot(t_stacked, Theta_FR_Stacked[:, 2], label='FR')
plt.plot(t_stacked, Theta_HL_Stacked[:, 2], label='HL')
plt.plot(t_stacked, Theta_HR_Stacked[:, 2], label='HR')
plt.xlabel('Time (s)')
plt.ylabel('Theta 3 (degrees)')
plt.legend()
plt.axvline(x=dt, color='k', linestyle='--', linewidth=0.5)
plt.axvline(x=dt + Offset_Transfer_Time, color='k', linestyle='--', linewidth=0.5)
plt.axvline(x=dt + Offset_Transfer_Time + Wave_Time, color='k', linestyle='--', linewidth=0.5)
plt.show()





### PLOT WITH RETURN

FL_Trajectory_Stacked_Return = np.hstack((P_FL_body_stand.reshape(3, 1), P_FL_body_Offset, P_FL_body_Wave, P_FL_body_Offset[:, ::-1], P_FL_body_stand.reshape(3, 1)))
FR_Trajectory_Stacked_Return = np.hstack((P_FR_body_stand.reshape(3, 1), P_FR_body_Offset, P_FR_body_Wave, P_FR_body_Offset[:, ::-1], P_FR_body_stand.reshape(3, 1)))
HL_Trajectory_Stacked_Return = np.hstack((P_HL_body_stand.reshape(3, 1), P_HL_body_Offset, P_HL_body_Wave, P_HL_body_Offset[:, ::-1], P_HL_body_stand.reshape(3, 1)))
HR_Trajectory_Stacked_Return = np.hstack((P_HR_body_stand.reshape(3, 1), P_HR_body_Offset, P_HR_body_Wave, P_HR_body_Offset[:, ::-1], P_HR_body_stand.reshape(3, 1)))

# Plot stacked trajectory for all legs
# With a thin vertical line a transition point between each of the 4 trajectory segments (stand height, stand to walk height, walk height to walk start, walk start to walk with transfer)
t_stacked_Return = np.linspace(0, dt + Offset_Transfer_Time + Wave_Time + Offset_Transfer_Time + dt, FL_Trajectory_Stacked_Return.shape[1]) # Create time array for stacked trajectory with return
plt.subplot(3, 1, 1)
plt.plot(t_stacked_Return, FL_Trajectory_Stacked_Return[0, :], label='FL')
plt.plot(t_stacked_Return, FR_Trajectory_Stacked_Return[0, :], label='FR')
plt.plot(t_stacked_Return, HL_Trajectory_Stacked_Return[0, :], label='HL')
plt.plot(t_stacked_Return, HR_Trajectory_Stacked_Return[0, :], label='HR')
plt.title('Stand to Walk to Stand Trajectory')
plt.xlabel('Time (s)')
plt.ylabel('X Position (m)')
plt.legend()
plt.axvline(x=dt, color='k', linestyle='--', linewidth=0.5)
plt.axvline(x=dt + Offset_Transfer_Time, color='k', linestyle='--', linewidth=0.5)
plt.axvline(x=dt + Offset_Transfer_Time + Wave_Time, color='k', linestyle='--', linewidth=0.5)
plt.axvline(x=dt + Offset_Transfer_Time + Wave_Time + Offset_Transfer_Time, color='k', linestyle='--', linewidth=0.5)
plt.axvline(x=dt + Offset_Transfer_Time + Wave_Time + Offset_Transfer_Time + dt, color='k', linestyle='--', linewidth=0.5)

plt.subplot(3, 1, 2)
plt.plot(t_stacked_Return, FL_Trajectory_Stacked_Return[1, :], label='FL')
plt.plot(t_stacked_Return, FR_Trajectory_Stacked_Return[1, :], label='FR')
plt.plot(t_stacked_Return, HL_Trajectory_Stacked_Return[1, :], label='HL')
plt.plot(t_stacked_Return, HR_Trajectory_Stacked_Return[1, :], label='HR')
plt.xlabel('Time (s)')
plt.ylabel('Y Position (m)')
plt.legend()
plt.axvline(x=dt, color='k', linestyle='--', linewidth=0.5)
plt.axvline(x=dt + Offset_Transfer_Time, color='k', linestyle='--', linewidth=0.5)
plt.axvline(x=dt + Offset_Transfer_Time + Wave_Time, color='k', linestyle='--', linewidth=0.5)
plt.axvline(x=dt + Offset_Transfer_Time + Wave_Time + Offset_Transfer_Time, color='k', linestyle='--', linewidth=0.5)
plt.axvline(x=dt + Offset_Transfer_Time + Wave_Time + Offset_Transfer_Time + dt, color='k', linestyle='--', linewidth=0.5)

plt.subplot(3, 1, 3)
plt.plot(t_stacked_Return, FL_Trajectory_Stacked_Return[2, :], label='FL')
plt.plot(t_stacked_Return, FR_Trajectory_Stacked_Return[2, :], label='FR')
plt.plot(t_stacked_Return, HL_Trajectory_Stacked_Return[2, :], label='HL')
plt.plot(t_stacked_Return, HR_Trajectory_Stacked_Return[2, :], label='HR')
plt.xlabel('Time (s)')
plt.ylabel('Z Position (m)')
plt.legend()
plt.axvline(x=dt, color='k', linestyle='--', linewidth=0.5)
plt.axvline(x=dt + Offset_Transfer_Time, color='k', linestyle='--', linewidth=0.5)
plt.axvline(x=dt + Offset_Transfer_Time + Wave_Time, color='k', linestyle='--', linewidth=0.5)
plt.axvline(x=dt + Offset_Transfer_Time + Wave_Time + Offset_Transfer_Time, color='k', linestyle='--', linewidth=0.5)
plt.axvline(x=dt + Offset_Transfer_Time + Wave_Time + Offset_Transfer_Time + dt, color='k', linestyle='--', linewidth=0.5)
plt.show()


# Stack Theta_XX arrays with return
Theta_FL_Stacked_Return = np.vstack((Theta_FL_stand, Theta_FL_Offset, Theta_FL_Wave, Theta_FL_Offset[::-1, :], Theta_FL_stand))
Theta_FR_Stacked_Return = np.vstack((Theta_FR_stand, Theta_FR_Offset, Theta_FR_Wave, Theta_FR_Offset[::-1, :], Theta_FR_stand))
Theta_HL_Stacked_Return = np.vstack((Theta_HL_stand, Theta_HL_Offset, Theta_HL_Wave, Theta_HL_Offset[::-1, :], Theta_HL_stand))
Theta_HR_Stacked_Return = np.vstack((Theta_HR_stand, Theta_HR_Offset, Theta_HR_Wave, Theta_HR_Offset[::-1, :], Theta_HR_stand))


# Deg to rad for animation
Theta_FL_anim = np.deg2rad(Theta_FL_Stacked_Return)
Theta_FR_anim = np.deg2rad(Theta_FR_Stacked_Return)
Theta_HL_anim = np.deg2rad(Theta_HL_Stacked_Return)
Theta_HR_anim = np.deg2rad(Theta_HR_Stacked_Return)


#### ANIMATION #### (Note: not true time)
# Animation which plots the trajectory of the legs in 3D space based on the computed joint angles and forward kinematics, showing the up/down motion of the legs as defined by the desired end-effector trajectory. The animation will show the movement of the legs over time, with the foot positions being updated according to the forward kinematics computed from the inverse kinematics joint angles.
# Note that the end-effector positions computed from the forward kinematics should be transformed to the body frame using CT.TB_0xx functions before plotting, to ensure that the trajectory is visualized in the correct frame of reference.
# Prepare figure
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim(-0.5, 0.5)
ax.set_ylim(-0.5, 0.5)
ax.set_zlim(-0.5, 0)
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Z (m)')
ax.set_title('Stand to Walk Motion')
ax.view_init(elev=20, azim=225)
lineFL, = ax.plot([], [], [], 'o-', lw=2, color=COLORS['FL'], label='FL')
lineFR, = ax.plot([], [], [], 'o-', lw=2, color=COLORS['FR'], label='FR')
lineHL, = ax.plot([], [], [], 'o-', lw=2, color=COLORS['HL'], label='HL')
lineHR, = ax.plot([], [], [], 'o-', lw=2, color=COLORS['HR'], label='HR')
ax.legend(loc='upper right')
def init():
    lineFL.set_data([], [])
    lineFL.set_3d_properties([])
    lineFR.set_data([], [])
    lineFR.set_3d_properties([])
    lineHL.set_data([], [])
    lineHL.set_3d_properties([])
    lineHR.set_data([], [])
    lineHR.set_3d_properties([])
    return lineFL, lineFR, lineHL, lineHR
def _pt(P, i):
    # Extract scalar coordinate i from a point, regardless of shape
    return float(np.array(P).ravel()[i])
def update(num):
    # Get current joint angles
    th1_FL, th2_FL, th3_FL = Theta_FL_anim[num]
    th1_FR, th2_FR, th3_FR = Theta_FR_anim[num]
    th1_HL, th2_HL, th3_HL = Theta_HL_anim[num]
    th1_HR, th2_HR, th3_HR = Theta_HR_anim[num]
    
    '''
    # Get current joint angles (for reverse direction)
    th1_FL, th2_FL, th3_FL = Theta_HR_anim[num]
    th1_FR, th2_FR, th3_FR = Theta_HL_anim[num]
    th1_HL, th2_HL, th3_HL = Theta_FR_anim[num]
    th1_HR, th2_HR, th3_HR = Theta_FL_anim[num]
    '''

    # Compute forward kinematics in leg base frames
    P0_1_FL = P0_1_FR = P0_1_HL = P0_1_HR = np.array([[0], [0], [0]])  # Placeholder for P0_1
    P0_2_FL = P0_2(th1_FL, th2_FL, th3_FL, 'FL')
    P0_2_FR = P0_2(th1_FR, th2_FR, th3_FR, 'FR')
    P0_2_HL = P0_2(th1_HL, th2_HL, th3_HL, 'HL')
    P0_2_HR = P0_2(th1_HR, th2_HR, th3_HR, 'HR') 
    P0_3_FL = P0_3(th1_FL, th2_FL, th3_FL, 'FL')
    P0_3_FR = P0_3(th1_FR, th2_FR, th3_FR, 'FR')
    P0_3_HL = P0_3(th1_HL, th2_HL, th3_HL, 'HL')
    P0_3_HR = P0_3(th1_HR, th2_HR, th3_HR, 'HR') 
    P0_end_FL = P0_end(th1_FL, th2_FL, th3_FL, 'FL')
    P0_end_FR = P0_end(th1_FR, th2_FR, th3_FR, 'FR')
    P0_end_HL = P0_end(th1_HL, th2_HL, th3_HL, 'HL')
    P0_end_HR = P0_end(th1_HR, th2_HR, th3_HR, 'HR') 

    # Transform end-effector positions to body frame
    P0_1_FL = TB_0(P0_1_FL, 'FL')
    P0_1_FR = TB_0(P0_1_FR, 'FR')
    P0_1_HL = TB_0(P0_1_HL, 'HL')
    P0_1_HR = TB_0(P0_1_HR, 'HR')
    P0_2_FL = TB_0(P0_2_FL, 'FL')
    P0_2_FR = TB_0(P0_2_FR, 'FR')
    P0_2_HL = TB_0(P0_2_HL, 'HL')
    P0_2_HR = TB_0(P0_2_HR, 'HR')
    P0_3_FL = TB_0(P0_3_FL, 'FL')
    P0_3_FR = TB_0(P0_3_FR, 'FR')
    P0_3_HL = TB_0(P0_3_HL, 'HL')
    P0_3_HR = TB_0(P0_3_HR, 'HR')
    P0_end_FL = TB_0(P0_end_FL, 'FL')
    P0_end_FR = TB_0(P0_end_FR, 'FR')
    P0_end_HL = TB_0(P0_end_HL, 'HL')
    P0_end_HR = TB_0(P0_end_HR, 'HR')

    # Update line data for each leg
    for line, pts in [
        (lineFL, [P0_1_FL, P0_2_FL, P0_3_FL, P0_end_FL]),
        (lineFR, [P0_1_FR, P0_2_FR, P0_3_FR, P0_end_FR]),
        (lineHL, [P0_1_HL, P0_2_HL, P0_3_HL, P0_end_HL]),
        (lineHR, [P0_1_HR, P0_2_HR, P0_3_HR, P0_end_HR]),
    ]:
        xs = [_pt(p, 0) for p in pts]
        ys = [_pt(p, 1) for p in pts]
        zs = [_pt(p, 2) for p in pts]
        line.set_data(xs, ys)
        line.set_3d_properties(zs)

    return lineFL, lineFR, lineHL, lineHR
ani = animation.FuncAnimation(fig, update, frames=(len(t_stacked_Return)), init_func=init,
                              interval=1, blit=True)
plt.show()
