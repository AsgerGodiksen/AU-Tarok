# Script to perform stand to walk transistion and walk for x cycles while logging torque and position data
# Used to update and tune PI parameters for walking
# Used to log position and torque data

import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..')))
import can
import time
import numpy as np
import struct
import csv
import concurrent.futures
from Robot import *
#from Logger_Functions import *

# CAN initialization in terminal: "for i in 0 1 2 3; do sudo ip link set dev can$i up type can bitrate 1000000 && sudo ip link set can$i txqueuelen 1000; done"

# Parameters From Tarok Dimensions
Tarok = TarokDymensions()
LEGS = Tarok.LEGS
COLORS = Tarok.COLORS
PHASE_OFFSET = Tarok.CRAWL_OFFSETS_Mixed

# ─────────────────────────────────────────────────────────────────────────────
# PI Parameters 
# ─────────────────────────────────────────────────────────────────────────────
# Manufacturing parameters for walking
PI_Walk = {
    'angle_kp':  100,
    'angle_ki':  100,
    'speed_kp':  50,
    'speed_ki':  40,
    'torque_kp': 50,
    'torque_ki': 50
}

# Parameters for standing
PI_Stand = {
    'angle_kp':  120,
    'angle_ki':  25,
    'speed_kp':  60,
    'speed_ki':  10,
    'torque_kp': 60,
    'torque_ki': 13
}

PI_UD = {
    'angle_kp':  110,
    'angle_ki':  50,
    'speed_kp':  55,
    'speed_ki':  20,
    'torque_kp': 55,
    'torque_ki': 25
}

# Manufacturing parameters (No?)
PI_Params = {
    #'angle_kp':  120,
    #'angle_ki':  50,
    #'speed_kp':  60,
    #'speed_ki':  80,
    #'torque_kp': 60,
    #'torque_ki': 40
}

# ─────────────────────────────────────────────────────────────────────────────
# Pre-computations  
# ─────────────────────────────────────────────────────────────────────────────
print("Performing pre-computations...")
# Offsets for COM transfer during stand phase
x_offset = 0.03 # [m] how much to move COM forward during transfer
y_offset = 0.04 # [m] how much to move COM to the left during transfer

# Time parameters
dt = 0.005 # seconds (200 Hz)

Swing_Time_Scalar = 1    # [s] swing phase duration
Stand_Time_Scalar  = 3 * Swing_Time_Scalar # [s] stand phase duration
Transfer_Time_Scalar = 1.25 # [s] duration of the COM transfer

total_time = Swing_Time_Scalar + Stand_Time_Scalar
Total_Time_Steps = int(total_time / dt)
total_time_with_transfer = total_time + 4 * Transfer_Time_Scalar
Total_Time_Steps_with_transfer = int(total_time_with_transfer / dt)

t_swing = np.linspace(0, Swing_Time_Scalar - dt, int(Swing_Time_Scalar / dt)) # Time array for swing phase
t_stand = np.linspace(Swing_Time_Scalar, total_time - dt, int(Stand_Time_Scalar / dt)) # Time array for stand phase
t_transfer = np.linspace(0, Transfer_Time_Scalar - dt, int(Transfer_Time_Scalar / dt)) # Time array for transfer phase

t = np.concatenate((t_swing, t_stand)) # Full time array for one cycle of the gait
t_with_transfer = np.linspace(0, total_time_with_transfer - dt, Total_Time_Steps_with_transfer)

Swing_Time_Steps = len(t_swing)
Stand_Time_Steps = len(t_stand)
Transfer_Time_Steps = len(t_transfer)

# Define time parameters for transition from stand height to walk height
total_time_SWH = 1 # [s] total time for transition from stand height to walk height
num_time_steps_SWH = int(total_time_SWH / dt)
t_SWH = np.linspace(0, total_time_SWH, num_time_steps_SWH) # time array for transition from stand height to walk height

# Define time parameters for transition from walk height to walk start position
STW_Swing_Time_Scalar = 2.5 # [s] duration of swing phase for transition from stand height to walk start
t_swing_STW = np.linspace(0, STW_Swing_Time_Scalar - dt, int(STW_Swing_Time_Scalar / dt)) # Time array for swing phase
STW_Swing_Time_Steps = len(t_swing_STW) 
total_time_STW = 4 * STW_Swing_Time_Scalar
t_STW = np.linspace(0, total_time_STW - dt, int(total_time_STW / dt)) # Full time array for transition from stand height to walk start

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
### WALK GENERATION ###
# ----------------------------- #

# Trajectory generation (Bezier curve in body frame)
Bezier_Trajectory, Bezier_Velocities, _ = Building_Bezier_Trajectories(
                                    Swing_Time_Scalar,
                                    Stand_Time_Scalar,
                                    Swing_Time_Steps,
                                    Stand_Time_Steps,
                                    Stand_Phase_type = "Constant",
                                    Leg = "FL"
                                    )

# Translation to four shoulder
Front_Left_Shoulder, Front_Right_Shoulder, Hind_Left_Shoulder, Hind_Right_Shoulder = Tarok.Shoulder_Positions()
FL_Bezier_Trajectory = (Bezier_Trajectory + Front_Left_Shoulder) 
FR_Bezier_Trajectory = (Bezier_Trajectory + Front_Right_Shoulder)
HL_Bezier_Trajectory = (Bezier_Trajectory + Hind_Left_Shoulder)  
HR_Bezier_Trajectory = (Bezier_Trajectory + Hind_Right_Shoulder) 

# Apply phase offsets for crawl gait
FL_Bezier_Trajectory, FL_Bezier_Velocities = Apply_Phase_Offset(FL_Bezier_Trajectory, Bezier_Velocities, PHASE_OFFSET['FL'])
FR_Bezier_Trajectory, FR_Bezier_Velocities = Apply_Phase_Offset(FR_Bezier_Trajectory, Bezier_Velocities, PHASE_OFFSET['FR'])
HL_Bezier_Trajectory, HL_Bezier_Velocities = Apply_Phase_Offset(HL_Bezier_Trajectory, Bezier_Velocities, PHASE_OFFSET['HL'])
HR_Bezier_Trajectory, HR_Bezier_Velocities = Apply_Phase_Offset(HR_Bezier_Trajectory, Bezier_Velocities, PHASE_OFFSET['HR'])

### ADD TRANSFER PHASE TO TRAJECTORY ###
FL_Bezier_Trajectory_With_Transfer, FR_Bezier_Trajectory_With_Transfer, HL_Bezier_Trajectory_With_Transfer, HR_Bezier_Trajectory_With_Transfer, FL_Bezier_Velocities_With_Transfer, FR_Bezier_Velocities_With_Transfer, HL_Bezier_Velocities_With_Transfer, HR_Bezier_Velocities_With_Transfer = Bezier_Add_Transfer_Phase(
            t,
            t_transfer,
            t_with_transfer,
            Total_Time_Steps,
            Total_Time_Steps_with_transfer,
            Transfer_Time_Steps,
            Swing_Time_Steps,
            Transfer_Time_Scalar,
            PHASE_OFFSET,
            x_offset,
            y_offset,
            FL_Bezier_Trajectory,
            FR_Bezier_Trajectory,
            HL_Bezier_Trajectory,
            HR_Bezier_Trajectory,
            FL_Bezier_Velocities,
            FR_Bezier_Velocities,
            HL_Bezier_Velocities,
            HR_Bezier_Velocities
)

### TRANSFORMATIONS ###
# Transform the Bezier trajectory from Body Frame to Leg Base Frames
P_FL_Base = np.array([T0_B(FL_Bezier_Trajectory_With_Transfer[:, i].reshape((3, 1)), 'FL') for i in range((len(t_with_transfer)))])
P_FR_Base = np.array([T0_B(FR_Bezier_Trajectory_With_Transfer[:, i].reshape((3, 1)), 'FR') for i in range((len(t_with_transfer)))])
P_HL_Base = np.array([T0_B(HL_Bezier_Trajectory_With_Transfer[:, i].reshape((3, 1)), 'HL') for i in range((len(t_with_transfer)))])
P_HR_Base = np.array([T0_B(HR_Bezier_Trajectory_With_Transfer[:, i].reshape((3, 1)), 'HR') for i in range((len(t_with_transfer)))])

# Transform desired end-effector velocity from body frame to leg base frames
V_FL_base = np.array([R0_B(FL_Bezier_Velocities_With_Transfer[:, i].reshape((3, 1)), 'FL') for i in range((len(t_with_transfer)))])
V_FR_base = np.array([R0_B(FR_Bezier_Velocities_With_Transfer[:, i].reshape((3, 1)), 'FR') for i in range((len(t_with_transfer)))])
V_HL_base = np.array([R0_B(HL_Bezier_Velocities_With_Transfer[:, i].reshape((3, 1)), 'HL') for i in range((len(t_with_transfer)))])
V_HR_base = np.array([R0_B(HR_Bezier_Velocities_With_Transfer[:, i].reshape((3, 1)), 'HR') for i in range((len(t_with_transfer)))])

### Kinematics ###
# Determine joint angles for all 4 legs using inverse kinematics
Theta_FL = np.array([Inverse_Kinematics(P_FL_Base[i], 'FL') for i in range((len(t_with_transfer)))]) # Shape (num_time_steps, 3), containing theta1, theta2, theta3 for each time step
Theta_FR = np.array([Inverse_Kinematics(P_FR_Base[i], 'FR') for i in range((len(t_with_transfer)))]) # Shape (num_time_steps, 3), containing theta1, theta2, theta3 for each time step
Theta_HL = np.array([Inverse_Kinematics(P_HL_Base[i], 'HL') for i in range((len(t_with_transfer)))]) # Shape (num_time_steps, 3), containing theta1, theta2, theta3 for each time step
Theta_HR = np.array([Inverse_Kinematics(P_HR_Base[i], 'HR') for i in range((len(t_with_transfer)))]) # Shape (num_time_steps, 3), containing theta1, theta2, theta3 for each time step

# Determine joint velocities for all 4 legs using Jacobian
# Damped least squares inverse to avoid singularities - theta_dot = (J^T*J + damp^2*I)^-1 * J^T * cartesian_velocity 
Theta_dot_FL = np.zeros((3, len(t_with_transfer)))  # Initialize joint velocity array
Theta_dot_FR = np.zeros((3, len(t_with_transfer)))  # Initialize joint velocity array
Theta_dot_HL = np.zeros((3, len(t_with_transfer)))  # Initialize joint velocity array
Theta_dot_HR = np.zeros((3, len(t_with_transfer)))  # Initialize joint velocity array
damp = 0.001  # Damping factor
for i in range((Total_Time_Steps_with_transfer)):
    Jac_i_FL = Jacobian(Theta_FL[i, 0], Theta_FL[i, 1], Theta_FL[i, 2], 'FL')
    Jac_i_FR = Jacobian(Theta_FR[i, 0], Theta_FR[i, 1], Theta_FR[i, 2], 'FR')
    Jac_i_HL = Jacobian(Theta_HL[i, 0], Theta_HL[i, 1], Theta_HL[i, 2], 'HL')
    Jac_i_HR = Jacobian(Theta_HR[i, 0], Theta_HR[i, 1], Theta_HR[i, 2], 'HR')
    JT_FL = Jac_i_FL.T
    JT_FR = Jac_i_FR.T
    JT_HL = Jac_i_HL.T
    JT_HR = Jac_i_HR.T
    term_FL = JT_FL @ Jac_i_FL + (damp**2)*np.eye(3)
    term_FR = JT_FR @ Jac_i_FR + (damp**2)*np.eye(3)
    term_HL = JT_HL @ Jac_i_HL + (damp**2)*np.eye(3)
    term_HR = JT_HR @ Jac_i_HR + (damp**2)*np.eye(3)
    Theta_dot_FL[:, i] = np.linalg.solve(term_FL, JT_FL @ V_FL_base[i].flatten())
    Theta_dot_FR[:, i] = np.linalg.solve(term_FR, JT_FR @ V_FR_base[i].flatten())
    Theta_dot_HL[:, i] = np.linalg.solve(term_HL, JT_HL @ V_HL_base[i].flatten())
    Theta_dot_HR[:, i] = np.linalg.solve(term_HR, JT_HR @ V_HR_base[i].flatten())

# Convert joint angles and velocities to degrees and abs(degrees/s) for right units for motor control
Theta_FL = np.rad2deg(Theta_FL)
Theta_FR = np.rad2deg(Theta_FR)
Theta_HL = np.rad2deg(Theta_HL)
Theta_HR = np.rad2deg(Theta_HR)
Theta_dot_FL = np.abs(np.rad2deg(Theta_dot_FL))
Theta_dot_FR = np.abs(np.rad2deg(Theta_dot_FR))
Theta_dot_HL = np.abs(np.rad2deg(Theta_dot_HL))
Theta_dot_HR = np.abs(np.rad2deg(Theta_dot_HR))

# ----------------------------- #
### STAND HEIGHT TO WALK HEIGHT GENERATION ###
# ----------------------------- #

# Define desired end-effector trajectory for transition from stand height to walk height
x_FL_SWH = x_FR_SWH = (l_k/2) * np.ones_like(t_SWH) # X position in meters (constant)
x_HL_SWH = x_HR_SWH = (-l_k/2) * np.ones_like(t_SWH) # X position in meters (constant)
y_FL_SWH = y_HL_SWH = (w_k/2 + 0.078) * np.ones_like(t_SWH) # Y position in meters (constant)
y_FR_SWH = y_HR_SWH = (-w_k/2 - 0.078) * np.ones_like(t_SWH) # Y position in meters (constant)
z_SWH = cos_interp(t_SWH, z_stand, -P_FL_Base[0, 0], 0, total_time_SWH) # Z position in meters (cosine interpolation from stand height to walk height)

# Define desired end-effector velocity for transition from stand height to walk height
x_dot_SWH = np.zeros_like(t_SWH) # X velocity in m/s (constant)
y_dot_SWH = np.zeros_like(t_SWH) # Y velocity in m/s (constant)
z_dot_SWH = cos_interp_dot(t_SWH, z_stand, -P_FL_Base[0, 0], 0, total_time_SWH) # Z velocity in m/s (derivative of cosine interpolation)

### Transformations ###
# Combine trajectories into position arrays for each leg
P_FL_body_SWH = np.vstack((x_FL_SWH, y_FL_SWH, z_SWH))
P_FR_body_SWH = np.vstack((x_FR_SWH, y_FR_SWH, z_SWH))
P_HL_body_SWH = np.vstack((x_HL_SWH, y_HL_SWH, z_SWH))
P_HR_body_SWH = np.vstack((x_HR_SWH, y_HR_SWH, z_SWH))

# Transform desired end-effector trajectory from body frame to leg base frames
P_FL_base_SWH = np.array([T0_B(P_FL_body_SWH[:, i].reshape((3, 1)), 'FL') for i in range(len(t_SWH))])
P_FR_base_SWH = np.array([T0_B(P_FR_body_SWH[:, i].reshape((3, 1)), 'FR') for i in range(len(t_SWH))])
P_HL_base_SWH = np.array([T0_B(P_HL_body_SWH[:, i].reshape((3, 1)), 'HL') for i in range(len(t_SWH))])
P_HR_base_SWH = np.array([T0_B(P_HR_body_SWH[:, i].reshape((3, 1)), 'HR') for i in range(len(t_SWH))])

# Combine body frame trajectory cartesian velocities into array
V_body_SWH = np.vstack((x_dot_SWH, y_dot_SWH, z_dot_SWH))
# Transform desired end-effector velocity from body frame to leg base frames
V_FL_base_SWH = np.array([R0_B(V_body_SWH[:, i].reshape((3, 1)), 'FL') for i in range(len(t_SWH))])
V_FR_base_SWH = np.array([R0_B(V_body_SWH[:, i].reshape((3, 1)), 'FR') for i in range(len(t_SWH))])
V_HL_base_SWH = np.array([R0_B(V_body_SWH[:, i].reshape((3, 1)), 'HL') for i in range(len(t_SWH))])
V_HR_base_SWH = np.array([R0_B(V_body_SWH[:, i].reshape((3, 1)), 'HR') for i in range(len(t_SWH))])

### Kinematics ###
# Determine joint angles for all 4 legs using inverse kinematics
Theta_FL_SWH = np.array([Inverse_Kinematics(P_FL_base_SWH[i], 'FL') for i in range(len(t_SWH))]) # Shape (num_time_steps, 3), containing theta1, theta2, theta3 for each time step
Theta_FR_SWH = np.array([Inverse_Kinematics(P_FR_base_SWH[i], 'FR') for i in range(len(t_SWH))]) # Shape (num_time_steps, 3), containing theta1, theta2, theta3 for each time step
Theta_HL_SWH = np.array([Inverse_Kinematics(P_HL_base_SWH[i], 'HL') for i in range(len(t_SWH))]) # Shape (num_time_steps, 3), containing theta1, theta2, theta3 for each time step
Theta_HR_SWH = np.array([Inverse_Kinematics(P_HR_base_SWH[i], 'HR') for i in range(len(t_SWH))]) # Shape (num_time_steps, 3), containing theta1, theta2, theta3 for each time step

# Determine joint velocities for all 4 legs using Jacobian
# Damped least squares inverse to avoid singularities - theta_dot = (J^T*J + damp^2*I)^-1 * J^T * cartesian_velocity 
Theta_dot_FL_SWH = np.zeros((3, len(t_SWH)))  # Initialize joint velocity array
Theta_dot_FR_SWH = np.zeros((3, len(t_SWH)))  # Initialize joint velocity array
Theta_dot_HL_SWH = np.zeros((3, len(t_SWH)))  # Initialize joint velocity array
Theta_dot_HR_SWH = np.zeros((3, len(t_SWH)))  # Initialize joint velocity array
damp = 0.001  # Damping factor
for i in range(len(t_SWH)):
    Jac_i_FL = Jacobian(Theta_FL_SWH[i, 0], Theta_FL_SWH[i, 1], Theta_FL_SWH[i, 2], 'FL')
    Jac_i_FR = Jacobian(Theta_FR_SWH[i, 0], Theta_FR_SWH[i, 1], Theta_FR_SWH[i, 2], 'FR')
    Jac_i_HL = Jacobian(Theta_HL_SWH[i, 0], Theta_HL_SWH[i, 1], Theta_HL_SWH[i, 2], 'HL')
    Jac_i_HR = Jacobian(Theta_HR_SWH[i, 0], Theta_HR_SWH[i, 1], Theta_HR_SWH[i, 2], 'HR')
    JT_FL = Jac_i_FL.T
    JT_FR = Jac_i_FR.T
    JT_HL = Jac_i_HL.T
    JT_HR = Jac_i_HR.T
    term_FL = JT_FL @ Jac_i_FL + (damp**2)*np.eye(3)
    term_FR = JT_FR @ Jac_i_FR + (damp**2)*np.eye(3)
    term_HL = JT_HL @ Jac_i_HL + (damp**2)*np.eye(3)
    term_HR = JT_HR @ Jac_i_HR + (damp**2)*np.eye(3)
    Theta_dot_FL_SWH[:, i] = np.linalg.solve(term_FL, JT_FL @ V_FL_base_SWH[i].flatten())
    Theta_dot_FR_SWH[:, i] = np.linalg.solve(term_FR, JT_FR @ V_FR_base_SWH[i].flatten())
    Theta_dot_HL_SWH[:, i] = np.linalg.solve(term_HL, JT_HL @ V_HL_base_SWH[i].flatten())
    Theta_dot_HR_SWH[:, i] = np.linalg.solve(term_HR, JT_HR @ V_HR_base_SWH[i].flatten())

# Convert joint angles and velocities to degrees and abs(degrees/s) for right units for motor control
Theta_FL_SWH = np.rad2deg(Theta_FL_SWH)
Theta_FR_SWH = np.rad2deg(Theta_FR_SWH)
Theta_HL_SWH = np.rad2deg(Theta_HL_SWH)
Theta_HR_SWH = np.rad2deg(Theta_HR_SWH)
Theta_dot_FL_SWH = np.abs(np.rad2deg(Theta_dot_FL_SWH))
Theta_dot_FR_SWH = np.abs(np.rad2deg(Theta_dot_FR_SWH))
Theta_dot_HL_SWH = np.abs(np.rad2deg(Theta_dot_HL_SWH))
Theta_dot_HR_SWH = np.abs(np.rad2deg(Theta_dot_HR_SWH))

# ----------------------------- #
### WALK HEIGHT TO WALK START GENERATION ###
# ----------------------------- #

# Trajectory generation
t_Stand_To_Walk_With_Transfer, FL_Stand_To_Walk, FR_Stand_To_Walk, HL_Stand_To_Walk, HR_Stand_To_Walk, FL_Stand_To_Walk_Velocities, FR_Stand_To_Walk_Velocities, HL_Stand_To_Walk_Velocities, HR_Stand_To_Walk_Velocities = Building_Stand_To_Walk_Trajectories(
                                    STW_Swing_Time_Scalar, 
                                    STW_Swing_Time_Steps,
                                    PHASE_OFFSET, 
                                    FL_Bezier_Trajectory_With_Transfer[0, 0], 
                                    FR_Bezier_Trajectory_With_Transfer[0, 0], 
                                    HL_Bezier_Trajectory_With_Transfer[0, 0], 
                                    HR_Bezier_Trajectory_With_Transfer[0, 0])
total_time_STW_with_transfer = t_Stand_To_Walk_With_Transfer[-1]

### TRANSFORMATIONS ###
# Transform the stand to walk trajectory from Body Frame to Leg Base Frames
P_FL_Base_STW = np.array([T0_B(FL_Stand_To_Walk[:, i].reshape((3, 1)), 'FL') for i in range((len(t_Stand_To_Walk_With_Transfer)))])
P_FR_Base_STW = np.array([T0_B(FR_Stand_To_Walk[:, i].reshape((3, 1)), 'FR') for i in range((len(t_Stand_To_Walk_With_Transfer)))])
P_HL_Base_STW = np.array([T0_B(HL_Stand_To_Walk[:, i].reshape((3, 1)), 'HL') for i in range((len(t_Stand_To_Walk_With_Transfer)))])
P_HR_Base_STW = np.array([T0_B(HR_Stand_To_Walk[:, i].reshape((3, 1)), 'HR') for i in range((len(t_Stand_To_Walk_With_Transfer)))])

# Transform desired end-effector velocity from body frame to leg base frames
V_FL_base_STW = np.array([R0_B(FL_Stand_To_Walk_Velocities[:, i].reshape((3, 1)), 'FL') for i in range((len(t_Stand_To_Walk_With_Transfer)))])
V_FR_base_STW = np.array([R0_B(FR_Stand_To_Walk_Velocities[:, i].reshape((3, 1)), 'FR') for i in range((len(t_Stand_To_Walk_With_Transfer)))])
V_HL_base_STW = np.array([R0_B(HL_Stand_To_Walk_Velocities[:, i].reshape((3, 1)), 'HL') for i in range((len(t_Stand_To_Walk_With_Transfer)))])
V_HR_base_STW = np.array([R0_B(HR_Stand_To_Walk_Velocities[:, i].reshape((3, 1)), 'HR') for i in range((len(t_Stand_To_Walk_With_Transfer)))])

### Kinematics ###
# Determine joint angles for all 4 legs using inverse kinematics
Theta_FL_STW = np.array([Inverse_Kinematics(P_FL_Base_STW[i], 'FL') for i in range((len(t_Stand_To_Walk_With_Transfer)))]) # Shape (num_time_steps, 3), containing theta1, theta2, theta3 for each time step
Theta_FR_STW = np.array([Inverse_Kinematics(P_FR_Base_STW[i], 'FR') for i in range((len(t_Stand_To_Walk_With_Transfer)))]) # Shape (num_time_steps, 3), containing theta1, theta2, theta3 for each time step
Theta_HL_STW = np.array([Inverse_Kinematics(P_HL_Base_STW[i], 'HL') for i in range((len(t_Stand_To_Walk_With_Transfer)))]) # Shape (num_time_steps, 3), containing theta1, theta2, theta3 for each time step
Theta_HR_STW = np.array([Inverse_Kinematics(P_HR_Base_STW[i], 'HR') for i in range((len(t_Stand_To_Walk_With_Transfer)))]) # Shape (num_time_steps, 3), containing theta1, theta2, theta3 for each time step

# Determine joint velocities for all 4 legs using Jacobian
# Damped least squares inverse to avoid singularities - theta_dot = (J^T*J + damp^2*I)^-1 * J^T * cartesian_velocity 
Theta_dot_FL_STW = np.zeros((3, len(t_Stand_To_Walk_With_Transfer)))  # Initialize joint velocity array
Theta_dot_FR_STW = np.zeros((3, len(t_Stand_To_Walk_With_Transfer)))  # Initialize joint velocity array
Theta_dot_HL_STW = np.zeros((3, len(t_Stand_To_Walk_With_Transfer)))  # Initialize joint velocity array
Theta_dot_HR_STW = np.zeros((3, len(t_Stand_To_Walk_With_Transfer)))  # Initialize joint velocity array
damp = 0.001  # Damping factor
for i in range((int(total_time_STW_with_transfer / dt))):
    Jac_i_FL = Jacobian(Theta_FL_STW[i, 0], Theta_FL_STW[i, 1], Theta_FL_STW[i, 2], 'FL')
    Jac_i_FR = Jacobian(Theta_FR_STW[i, 0], Theta_FR_STW[i, 1], Theta_FR_STW[i, 2], 'FR')
    Jac_i_HL = Jacobian(Theta_HL_STW[i, 0], Theta_HL_STW[i, 1], Theta_HL_STW[i, 2], 'HL')
    Jac_i_HR = Jacobian(Theta_HR_STW[i, 0], Theta_HR_STW[i, 1], Theta_HR_STW[i, 2], 'HR')
    JT_FL = Jac_i_FL.T
    JT_FR = Jac_i_FR.T
    JT_HL = Jac_i_HL.T
    JT_HR = Jac_i_HR.T
    term_FL = JT_FL @ Jac_i_FL + (damp**2)*np.eye(3)
    term_FR = JT_FR @ Jac_i_FR + (damp**2)*np.eye(3)
    term_HL = JT_HL @ Jac_i_HL + (damp**2)*np.eye(3)
    term_HR = JT_HR @ Jac_i_HR + (damp**2)*np.eye(3)
    Theta_dot_FL_STW[:, i] = np.linalg.solve(term_FL, JT_FL @ V_FL_base_STW[i].flatten())
    Theta_dot_FR_STW[:, i] = np.linalg.solve(term_FR, JT_FR @ V_FR_base_STW[i].flatten())
    Theta_dot_HL_STW[:, i] = np.linalg.solve(term_HL, JT_HL @ V_HL_base_STW[i].flatten())
    Theta_dot_HR_STW[:, i] = np.linalg.solve(term_HR, JT_HR @ V_HR_base_STW[i].flatten())

# Convert joint angles and velocities to degrees and abs(degrees/s) for right units for motor control
Theta_FL_STW = np.rad2deg(Theta_FL_STW)
Theta_FR_STW = np.rad2deg(Theta_FR_STW)
Theta_HL_STW = np.rad2deg(Theta_HL_STW)
Theta_HR_STW = np.rad2deg(Theta_HR_STW)
Theta_dot_FL_STW = np.abs(np.rad2deg(Theta_dot_FL_STW))
Theta_dot_FR_STW = np.abs(np.rad2deg(Theta_dot_FR_STW))
Theta_dot_HL_STW = np.abs(np.rad2deg(Theta_dot_HL_STW))
Theta_dot_HR_STW = np.abs(np.rad2deg(Theta_dot_HR_STW))

print("Pre-computations complete.")

# ─────────────────────────────────────────────────────────────────────────────
# Initialization
# ─────────────────────────────────────────────────────────────────────────────
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

# Write Walk PI parameters to motors for data logging
print("Writing Walk PI parameters to motors...")
PID_RAM_Control(bus0,ID_1, PI_Walk)
PID_RAM_Control(bus0,ID_2, PI_Walk)
PID_RAM_Control(bus0,ID_3, PI_Walk)
PID_RAM_Control(bus1,ID_1, PI_Walk)
PID_RAM_Control(bus1,ID_2, PI_Walk)
PID_RAM_Control(bus1,ID_3, PI_Walk)
PID_RAM_Control(bus2,ID_1, PI_Walk)  
PID_RAM_Control(bus2,ID_2, PI_Walk)
PID_RAM_Control(bus2,ID_3, PI_Walk)
PID_RAM_Control(bus3,ID_1, PI_Walk)
PID_RAM_Control(bus3,ID_2, PI_Walk)
PID_RAM_Control(bus3,ID_3, PI_Walk)

time.sleep(0.2) # Sleep for a short time to ensure parameters are written before starting loop, adjust as needed

# ─────────────────────────────────────────────────────────────────────────────
# Data logging setup
# ─────────────────────────────────────────────────────────────────────────────
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
log_dir = os.path.join(SCRIPT_DIR, "TEST_DATA_PI_Stand_To_Walk")
os.makedirs(log_dir, exist_ok=True)
timestamp_str = time.strftime('%Y-%m-%d_%H-%M-%S')
log_filename = os.path.join(log_dir, f"STW_TorquePos_Log_Test{timestamp_str}.csv")
pid_filename = os.path.join(log_dir, f"STW_TorquePos_Log_Test{timestamp_str}_PID.txt")

# ─────────────────────────────────────────────────────────────────────────────
# Read and save PID parameters from all 12 motors
# ─────────────────────────────────────────────────────────────────────────────
print("Reading PID parameters from motors...")
_LEG_BUSES  = [("FL", bus0), ("FR", bus1), ("HL", bus2), ("HR", bus3)]
_JOINT_IDS  = [("J1", ID_1), ("J2", ID_2), ("J3", ID_3)]

with open(pid_filename, 'w') as _pid_file:
    _pid_file.write(f"PID Parameters — read {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
    _pid_file.write("=" * 52 + "\n\n")
    for leg, bus in _LEG_BUSES:
        _pid_file.write(f"Leg {leg}\n")
        _pid_file.write("-" * 30 + "\n")
        for joint, motor_id in _JOINT_IDS:
            result = Read_PID(bus, motor_id)
            if result is not None:
                pos_kp, pos_ki, spd_kp, spd_ki, torq_kp, torq_ki = result
                _pid_file.write(f"  {joint} (ID 0x{motor_id:03X}):\n")
                _pid_file.write(f"    Position Loop : Kp={pos_kp:5.0f}  Ki={pos_ki:5.0f}\n")
                _pid_file.write(f"    Speed Loop    : Kp={spd_kp:5.0f}  Ki={spd_ki:5.0f}\n")
                _pid_file.write(f"    Torque Loop   : Kp={torq_kp:5.0f}  Ki={torq_ki:5.0f}\n")
            else:
                _pid_file.write(f"  {joint} (ID 0x{motor_id:03X}): no response\n")
        _pid_file.write("\n")

print(f"PID parameters saved to {os.path.basename(pid_filename)}")

# ─────────────────────────────────────────────────────────────────────────────
# Prepare CSV file for logging and preallocate data array for efficient logging in the loop
# ─────────────────────────────────────────────────────────────────────────────

with open(log_filename, 'w', newline='') as csvfile:
    csv.writer(csvfile).writerow([
        "Timestamp (s)", "Trajectory Index",
        # 12 read torques
        "FL_J1_Torque", "FL_J2_Torque", "FL_J3_Torque",
        "FR_J1_Torque", "FR_J2_Torque", "FR_J3_Torque",
        "HL_J1_Torque", "HL_J2_Torque", "HL_J3_Torque",
        "HR_J1_Torque", "HR_J2_Torque", "HR_J3_Torque",
        # 12 read positions
        "FL_J1_Pos (deg)", "FL_J2_Pos (deg)", "FL_J3_Pos (deg)",
        "FR_J1_Pos (deg)", "FR_J2_Pos (deg)", "FR_J3_Pos (deg)",
        "HL_J1_Pos (deg)", "HL_J2_Pos (deg)", "HL_J3_Pos (deg)",
        "HR_J1_Pos (deg)", "HR_J2_Pos (deg)", "HR_J3_Pos (deg)",
        # 12 position commands (analytical trajectory setpoints)
        "FL_J1_Cmd (deg)", "FL_J2_Cmd (deg)", "FL_J3_Cmd (deg)",
        "FR_J1_Cmd (deg)", "FR_J2_Cmd (deg)", "FR_J3_Cmd (deg)",
        "HL_J1_Cmd (deg)", "HL_J2_Cmd (deg)", "HL_J3_Cmd (deg)",
        "HR_J1_Cmd (deg)", "HR_J2_Cmd (deg)", "HR_J3_Cmd (deg)",
    ])

# Preallocate: timestamp + index  + 12 torques + 12 positions + 12 commands = 38 columns
data = np.zeros((500000, 38))
data_count = 0

# Use a ThreadPoolExecutor to send commands and receive feedback from all 4 legs in parallel, minimizing latency and maximizing loop frequency
executor = concurrent.futures.ThreadPoolExecutor(max_workers=4)

time.sleep(0.2) # Sleep for a short time to ensure all setup is complete before starting pre-loop movements, adjust as needed

### Pre-loop Movement ###

# ─────────────────────────────────────────────────────────────────────────────
# Move to zero configuration
# ─────────────────────────────────────────────────────────────────────────────
print("Writing stand PI parameters and moving to zero position...")
for bus in [bus0, bus1, bus2, bus3]:
    PID_RAM_Control(bus, ID_1, PI_Stand)
    PID_RAM_Control(bus, ID_2, PI_Stand)
    PID_RAM_Control(bus, ID_3, PI_Stand)
time.sleep(0.2) # Sleep for a short time to ensure parameters are written before sending position commands, adjust as needed

for bus in [bus0, bus1, bus2, bus3]:
    Position_Control(bus, ID_1, 0, 30)
    Position_Control(bus, ID_2, 0, 30)
    Position_Control(bus, ID_3, 0, 30)
time.sleep(6)

# ─────────────────────────────────────────────────────────────────────────────
# Move to Stand Position
# ─────────────────────────────────────────────────────────────────────────────
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

# ─────────────────────────────────────────────────────────────────────────────
# Move to Walk Height
# ─────────────────────────────────────────────────────────────────────────────
print("Moved to stand position, writing up/down PI parameters and moving to walk height...")
for bus in [bus0, bus1, bus2, bus3]:
    PID_RAM_Control(bus, ID_1, PI_UD)
    PID_RAM_Control(bus, ID_2, PI_UD)
    PID_RAM_Control(bus, ID_3, PI_UD)
time.sleep(0.2) # Sleep for a short time to ensure parameters are written before sending position commands, adjust as needed

SWH_Statement = True
# Note start time
start_time = cycle_start = current_time = time.monotonic()
# Run until transition is done
while SWH_Statement:
    # Time Management
    current_time = time.monotonic()
    elapsed_cycle = current_time - cycle_start
    if elapsed_cycle >= total_time_SWH:
        SWH_Statement = False
        continue

    # Find closest value in t to elapsed in current cycle
    index = min(int(elapsed_cycle / dt), len(t_SWH) - 1)

    # Send position control commands to motors for current time step
    Position_Control(bus0, ID_1, Theta_FL_SWH[index, 0], Theta_dot_FL_SWH[0, index])
    Position_Control(bus0, ID_2, Theta_FL_SWH[index, 1], Theta_dot_FL_SWH[1, index])
    Position_Control(bus0, ID_3, Theta_FL_SWH[index, 2], Theta_dot_FL_SWH[2, index])
    Position_Control(bus1, ID_1, Theta_FR_SWH[index, 0], Theta_dot_FR_SWH[0, index])
    Position_Control(bus1, ID_2, Theta_FR_SWH[index, 1], Theta_dot_FR_SWH[1, index])
    Position_Control(bus1, ID_3, Theta_FR_SWH[index, 2], Theta_dot_FR_SWH[2, index])
    Position_Control(bus2, ID_1, Theta_HL_SWH[index, 0], Theta_dot_HL_SWH[0, index])
    Position_Control(bus2, ID_2, Theta_HL_SWH[index, 1], Theta_dot_HL_SWH[1, index])
    Position_Control(bus2, ID_3, Theta_HL_SWH[index, 2], Theta_dot_HL_SWH[2, index])
    Position_Control(bus3, ID_1, Theta_HR_SWH[index, 0], Theta_dot_HR_SWH[0, index])
    Position_Control(bus3, ID_2, Theta_HR_SWH[index, 1], Theta_dot_HR_SWH[1, index])
    Position_Control(bus3, ID_3, Theta_HR_SWH[index, 2], Theta_dot_HR_SWH[2, index])

time.sleep(0.5) # Sleep for a short time to ensure transition to walk height is complete before continuing, adjust as needed

# ─────────────────────────────────────────────────────────────────────────────
# Move to Walk Start Position
# ─────────────────────────────────────────────────────────────────────────────
print("Moved to walk height, writing walk PI parameters and moving to walk start position...")
for bus in [bus0, bus1, bus2, bus3]:
    PID_RAM_Control(bus, ID_1, PI_Walk)
    PID_RAM_Control(bus, ID_2, PI_Walk)
    PID_RAM_Control(bus, ID_3, PI_Walk)
time.sleep(0.2) # Sleep for a short time to ensure parameters are written before sending position commands, adjust as needed

STW_Statement = True
# Note start time
start_time = cycle_start = current_time = time.monotonic()
# Run until transition is done
while STW_Statement:
    # Time Management
    current_time = time.monotonic()
    elapsed_cycle = current_time - cycle_start
    if elapsed_cycle >= total_time_STW_with_transfer:
        STW_Statement = False
        continue

    # Find closest value in t to elapsed in current cycle
    index = min(int(elapsed_cycle / dt), len(t_Stand_To_Walk_With_Transfer) - 1)

    # Send position control commands to motors for current time step
    Position_Control(bus0, ID_1, Theta_FL_STW[index, 0], Theta_dot_FL_STW[0, index])
    Position_Control(bus0, ID_2, Theta_FL_STW[index, 1], Theta_dot_FL_STW[1, index])
    Position_Control(bus0, ID_3, Theta_FL_STW[index, 2], Theta_dot_FL_STW[2, index])
    Position_Control(bus1, ID_1, Theta_FR_STW[index, 0], Theta_dot_FR_STW[0, index])
    Position_Control(bus1, ID_2, Theta_FR_STW[index, 1], Theta_dot_FR_STW[1, index])
    Position_Control(bus1, ID_3, Theta_FR_STW[index, 2], Theta_dot_FR_STW[2, index])
    Position_Control(bus2, ID_1, Theta_HL_STW[index, 0], Theta_dot_HL_STW[0, index])
    Position_Control(bus2, ID_2, Theta_HL_STW[index, 1], Theta_dot_HL_STW[1, index])
    Position_Control(bus2, ID_3, Theta_HL_STW[index, 2], Theta_dot_HL_STW[2, index])
    Position_Control(bus3, ID_1, Theta_HR_STW[index, 0], Theta_dot_HR_STW[0, index])
    Position_Control(bus3, ID_2, Theta_HR_STW[index, 1], Theta_dot_HR_STW[1, index])
    Position_Control(bus3, ID_3, Theta_HR_STW[index, 2], Theta_dot_HR_STW[2, index])

time.sleep(1) # Sleep for a short time to ensure transition to walk start position is complete before starting loop, adjust as needed

print("Pre-loop sequence complete, starting loop...")

time.sleep(1) # Sleep for a short time to ensure all transitions are complete before starting loop, adjust as needed

# ─────────────────────────────────────────────────────────────────────────────
# Main loop — cyclic Bezier walk trajectory with torque + position + command logging
# ─────────────────────────────────────────────────────────────────────────────
LOOP_PERIOD = 1.0 / 200.0   # Minimum loop period (200 Hz)
NUM_CYCLES  = 3             # robot performs this many Walk cycles then holds still
loop_times  = []
cycle_count = 0
#print("SET IN POSITION")
#time.sleep(10)
#print("\nPre-loop sequence complete.")
#print(">>> STARTING in 0.5 seconds — remove your hands! <<<")
#time.sleep(0.5)

#print("Pre-loop sequence complete, starting loop...")
print(f"Loop started — robot will complete {NUM_CYCLES} cycles then hold still.")
print("Press Ctrl+C at any time to shut down.")
print("Loop started - Press ctrl+c in terminal for shutdown")

# Note start time
start_time = cycle_start  = time.monotonic()

try:
    while True:
        # Loop time management
        loop_start = time.monotonic()
        elapsed_total = loop_start - start_time
        elapsed_cycle = loop_start - cycle_start

        #loop_elapsed = time.monotonic() - loop_start
        #loop_times.append(loop_elapsed)
        #sleep_time = LOOP_PERIOD - loop_elapsed
        #time.sleep(max(sleep_time, 0.001))

        # Check if current cycle is complete -> Start new cycle or end test if all cycles complete
        if elapsed_cycle >= total_time_with_transfer:
            cycle_start += total_time_with_transfer
            cycle_count += 1

            # If all cycles complete, print message and hold position
            if cycle_count >= NUM_CYCLES:
                # ── Test complete: logging stops, robot holds start position ──
                print("\n" + "=" * 54)
                print(f"  TEST COMPLETE — {NUM_CYCLES} cycles finished.")
                print("  Data logging has stopped.")
                print("  Robot is holding the walk start position.")
                print("  It is now safe to lift the robot back into the holder.")
                print("  Press Ctrl+C to shut down the motors when in the holder.")
                print("=" * 54 + "\n")

                hold_angles = [
                    (bus0, Theta_FL[0]),
                    (bus1, Theta_FR[0]),
                    (bus2, Theta_HL[0]),
                    (bus3, Theta_HR[0]),
                ]
                while True:
                    for bus, angles in hold_angles:
                        Position_Control(bus, ID_1, angles[0], 30)
                        Position_Control(bus, ID_2, angles[1], 30)
                        Position_Control(bus, ID_3, angles[2], 30)
                    time.sleep(0.5) # Sleep to reduce CPU usage while holding position
            continue

        # Find closest value in t to elapsed in current cycle
        index = min(int(elapsed_cycle / dt), len(t_with_transfer) - 1)

        # ── Send + receive all 4 legs in parallel ──────────────────────────
        motor_ids = [ID_1, ID_2, ID_3]
        f_FL = executor.submit(send_leg, bus0, Theta_FL[index], Theta_dot_FL[:, index],motor_ids)
        f_FR = executor.submit(send_leg, bus1, Theta_FR[index], Theta_dot_FR[:, index],motor_ids)
        f_HL = executor.submit(send_leg, bus2, Theta_HL[index], Theta_dot_HL[:, index],motor_ids)
        f_HR = executor.submit(send_leg, bus3, Theta_HR[index], Theta_dot_HR[:, index],motor_ids)

        # Extract feedback results
        fb_FL = f_FL.result()
        fb_FR = f_FR.result()
        fb_HL = f_HL.result()
        fb_HR = f_HR.result()

        # ── Log torque + position + command ────────────────────────────────
        data[data_count, :] = [
            elapsed_total, index,
            # 12 read torques (Nm)
            safe_torque(fb_FL, 0), safe_torque(fb_FL, 1), safe_torque(fb_FL, 2),
            safe_torque(fb_FR, 0), safe_torque(fb_FR, 1), safe_torque(fb_FR, 2),
            safe_torque(fb_HL, 0), safe_torque(fb_HL, 1), safe_torque(fb_HL, 2),
            safe_torque(fb_HR, 0), safe_torque(fb_HR, 1), safe_torque(fb_HR, 2),
            # 12 read positions (deg, output shaft, multi-turn)
            safe_position(fb_FL, 0), safe_position(fb_FL, 1), safe_position(fb_FL, 2),
            safe_position(fb_FR, 0), safe_position(fb_FR, 1), safe_position(fb_FR, 2),
            safe_position(fb_HL, 0), safe_position(fb_HL, 1), safe_position(fb_HL, 2),
            safe_position(fb_HR, 0), safe_position(fb_HR, 1), safe_position(fb_HR, 2),
            # 12 position commands (deg, analytical trajectory setpoints)
            Theta_FL[index, 0], Theta_FL[index, 1], Theta_FL[index, 2],
            Theta_FR[index, 0], Theta_FR[index, 1], Theta_FR[index, 2],
            Theta_HL[index, 0], Theta_HL[index, 1], Theta_HL[index, 2],
            Theta_HR[index, 0], Theta_HR[index, 1], Theta_HR[index, 2],
        ]
        data_count += 1

        # Loop time management
        loop_elapsed = time.monotonic() - loop_start
        loop_times.append(loop_elapsed)
        sleep_time = LOOP_PERIOD - loop_elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)# Stop loop with Ctrl+C in terminal
    
except KeyboardInterrupt:
    print("\nKeyboardInterrupt received, shutting down...")
    executor.shutdown(wait=False)

    # ── Loop timing diagnostic ──────────────────────────────────────────────
    if loop_times:
        arr = np.array(loop_times) * 1000
        print(f"\nLoop timing over {len(arr)} cycles:")
        print(f"  Mean:  {arr.mean():.2f} ms  ({1000/arr.mean():.1f} Hz)")
        print(f"  Min:   {arr.min():.2f} ms")
        print(f"  Max:   {arr.max():.2f} ms")
        print(f"  >5 ms: {(arr > 5).sum()} cycles ({100*(arr>5).mean():.1f}%)\n")

    # ── Stop motors ─────────────────────────────────────────────────────────
    print("Stopping motors...")
    for bus in [bus0, bus1, bus2, bus3]:
        while bus.recv(timeout=0.001) is not None:
            pass
    for bus in [bus0, bus1, bus2, bus3]:
        for motor_id in [ID_1, ID_2, ID_3]:
            try:
                Motor_Stop(bus, motor_id)
            except RuntimeError as e:
                print(f"  Warning: {e}")
    print("Motors stopped.")

    print("Shutting down CAN buses...")
    for bus in [bus0, bus1, bus2, bus3]:
        bus.shutdown()
    print("CAN buses shut down.")

    # ── Write logged data ────────────────────────────────────────────────────
    print("Storing logged data to file...")
    with open(log_filename, "a") as file:
        for i in range(data_count):
            file.write(",".join(f"{v:.4f}" for v in data[i, :]) + "\n")
    print(f"Logged {data_count} rows to {log_filename}")
    print("Shutdown complete.")
