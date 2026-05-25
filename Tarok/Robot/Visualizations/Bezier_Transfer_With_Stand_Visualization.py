# Test script for bezier trajectory with transfer phase before each swing phase
# Moving COM away from upcoming swing leg
# And with Start in stand phase - as preperation for implementation in state machine

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
import matplotlib

matplotlib.rcParams["font.family"] = "Liberation Serif"

# CAN initialization in terminal: "for i in 0 1 2 3; do sudo ip link set dev can$i up type can bitrate 1000000 && sudo ip link set can$i txqueuelen 1000; done"

### SCRIPT START ###
## PRECOMPUTATIONS ##
print("Performing pre-computations...")

LABEL_SIZE  = 17
TICK_SIZE   = 13
TITLE_SIZE  = 13
LEGEND_SIZE = 13

SWING_TIME    = 1.0
STAND_TIME    = 3.0
TRANSFER_TIME = 1.

# Parameters From Tarok Dimensions
Tarok = TarokDymensions()
LEGS = Tarok.LEGS
COLORS = Tarok.COLORS
PHASE_OFFSET = Tarok.CRAWL_OFFSETS_Mixed

# Offsets for COM transfer during stand phase
x_offset = 0.03 # [m] how much to move COM forward during transfer
y_offset = 0.04 # [m] how much to move COM to the left during transfer

# Time parameters
dt = 0.005 # seconds (200 Hz)

Swing_Time_Scalar = 0.8    # [s] swing phase duration
Stand_Time_Scalar  = 3 * Swing_Time_Scalar # [s] stand phase duration
Transfer_Time_Scalar = 1 # [s] duration of the COM transfer

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

# Translation to four shoulders
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

# Define time parameters for transition from stand height to walk height
total_time_SWH = 1 # [s] total time for transition from stand height to walk height
num_time_steps_SWH = int(total_time_SWH / dt)
t_SWH = np.linspace(0, total_time_SWH, num_time_steps_SWH) # time array for transition from stand height to walk height

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

STW_Swing_Time_Scalar = 0.8 # [s] duration of swing phase for transition from stand height to walk start
t_swing_STW = np.linspace(0, STW_Swing_Time_Scalar - dt, int(STW_Swing_Time_Scalar / dt)) # Time array for swing phase
STW_Swing_Time_Steps = len(t_swing_STW) 
total_time_STW = 4 * STW_Swing_Time_Scalar
t_STW = np.linspace(0, total_time_STW - dt, int(total_time_STW / dt)) # Full time array for transition from stand height to walk start

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


#### PLOTTING ####

# Plot FL_stand_to_walk trajectory for verification
##### FIGURE 1 ##########
plt.subplot(3, 1, 1)
plt.plot(t_Stand_To_Walk_With_Transfer, FL_Stand_To_Walk[0, :], label='FL Stand to Walk X')
plt.title('FL Stand to Walk Trajectory X Coordinate')
plt.xlabel('Time (s)')
plt.ylabel('X Position (m)')
plt.legend()    
plt.subplot(3, 1, 2)
plt.plot(t_Stand_To_Walk_With_Transfer, FL_Stand_To_Walk[1, :], label='FL Stand to Walk Y')
plt.title('FL Stand to Walk Trajectory Y Coordinate')
plt.xlabel('Time (s)')
plt.ylabel('Y Position (m)')
plt.legend()
plt.subplot(3, 1, 3)
plt.plot(t_Stand_To_Walk_With_Transfer, FL_Stand_To_Walk[2, :], label='FL Stand to Walk Z')
plt.title('FL Stand to Walk Trajectory Z Coordinate')
plt.xlabel('Time (s)')
plt.ylabel('Z Position (m)')
#plt.legend()




# Plot_FL_Stand_To_Walk_Velocities for verification
##### FIGURE 2 ##########
plt.figure(figsize=(10, 8))
plt.subplot(3, 1, 1)
plt.plot(t_Stand_To_Walk_With_Transfer, FL_Stand_To_Walk_Velocities[0, :], label='FL Stand to Walk X Velocity')
plt.title('FL Stand to Walk Trajectory X Velocity')
plt.xlabel('Time (s)')
plt.ylabel('X Velocity (m/s)')
plt.legend()
plt.subplot(3, 1, 2)
plt.plot(t_Stand_To_Walk_With_Transfer, FL_Stand_To_Walk_Velocities[1, :], label='FL Stand to Walk Y Velocity')
plt.title('FL Stand to Walk Trajectory Y Velocity')
plt.xlabel('Time (s)')
plt.ylabel('Y Velocity (m/s)')
plt.legend()
plt.subplot(3, 1, 3)
plt.plot(t_Stand_To_Walk_With_Transfer, FL_Stand_To_Walk_Velocities[2, :], label='FL Stand to Walk Z Velocity')
plt.title('FL Stand to Walk Trajectory Z Velocity')
plt.xlabel('Time (s)')
plt.ylabel('Z Velocity (m/s)')
plt.legend()
#plt.show()


# print shape of P_FL_body_stand = np.array([x_FL_stand, y_FL_stand, z_stand])
print("Shape of P_FL_body_stand:", P_FL_body_stand.shape)

# print shape of P_FL_body_SWH = np.vstack((x_FL_SWH, y_FL_SWH, z_SWH))
print("Shape of P_FL_body_SWH:", P_FL_body_SWH.shape)

# print shape of FL_Stand_To_Walk
print("Shape of FL_Stand_To_Walk_With_Transfer:", FL_Stand_To_Walk.shape)

# print shape of FL_Bezier_Trajectory_With_Transfer
print("Shape of FL_Bezier_Trajectory_With_Transfer:", FL_Bezier_Trajectory_With_Transfer.shape)


FL_Trajectory_Stacked = np.hstack((P_FL_body_stand.reshape(3, 1), P_FL_body_SWH, FL_Stand_To_Walk, FL_Bezier_Trajectory_With_Transfer))
FR_Trajectory_Stacked = np.hstack((P_FR_body_stand.reshape(3, 1), P_FR_body_SWH, FR_Stand_To_Walk, FR_Bezier_Trajectory_With_Transfer))
HL_Trajectory_Stacked = np.hstack((P_HL_body_stand.reshape(3, 1), P_HL_body_SWH, HL_Stand_To_Walk, HL_Bezier_Trajectory_With_Transfer))
HR_Trajectory_Stacked = np.hstack((P_HR_body_stand.reshape(3, 1), P_HR_body_SWH, HR_Stand_To_Walk, HR_Bezier_Trajectory_With_Transfer))


# Print shape of FL_Trajectory_Stacked
print("Shape of FL_Trajectory_Stacked:", FL_Trajectory_Stacked.shape)



##### FIGURE 3 ##########
# Plot stacked trajectory for all legs
# With a thin vertical line a transition point between each of the 4 trajectory segments (stand height, stand to walk height, walk height to walk start, walk start to walk with transfer)
t_stacked = np.linspace(0, dt + total_time_SWH + total_time_STW_with_transfer + total_time_with_transfer, FL_Trajectory_Stacked.shape[1]) # Create time array for stacked trajectory
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
plt.axvline(x=dt + total_time_SWH, color='k', linestyle='--', linewidth=0.5)
plt.axvline(x=dt + total_time_SWH + total_time_STW_with_transfer, color='k', linestyle='--', linewidth=0.5)
plt.axvline(x=dt + total_time_SWH + total_time_STW_with_transfer + total_time_with_transfer, color='k', linestyle='--', linewidth=0.5)

plt.subplot(3, 1, 2)
plt.plot(t_stacked, FL_Trajectory_Stacked[1, :], label='FL')
plt.plot(t_stacked, FR_Trajectory_Stacked[1, :], label='FR')
plt.plot(t_stacked, HL_Trajectory_Stacked[1, :], label='HL')
plt.plot(t_stacked, HR_Trajectory_Stacked[1, :], label='HR')
plt.xlabel('Time (s)')
plt.ylabel('Y Position (m)')
plt.legend()
plt.axvline(x=dt, color='k', linestyle='--', linewidth=0.5)
plt.axvline(x=dt + total_time_SWH, color='k', linestyle='--', linewidth=0.5)
plt.axvline(x=dt + total_time_SWH + total_time_STW_with_transfer, color='k', linestyle='--', linewidth=0.5)
plt.axvline(x=dt + total_time_SWH + total_time_STW_with_transfer + total_time_with_transfer, color='k', linestyle='--', linewidth=0.5)

plt.subplot(3, 1, 3)
plt.plot(t_stacked, FL_Trajectory_Stacked[2, :], label='FL')
plt.plot(t_stacked, FR_Trajectory_Stacked[2, :], label='FR')
plt.plot(t_stacked, HL_Trajectory_Stacked[2, :], label='HL')
plt.plot(t_stacked, HR_Trajectory_Stacked[2, :], label='HR')
plt.xlabel('Time (s)')
plt.ylabel('Z Position (m)')
plt.legend()
plt.axvline(x=dt, color='k', linestyle='--', linewidth=0.5)
plt.axvline(x=dt + total_time_SWH, color='k', linestyle='--', linewidth=0.5)
plt.axvline(x=dt + total_time_SWH + total_time_STW_with_transfer, color='k', linestyle='--', linewidth=0.5)
plt.axvline(x=dt + total_time_SWH + total_time_STW_with_transfer + total_time_with_transfer, color='k', linestyle='--', linewidth=0.5)
#plt.show()




# Stack Theta_XX arrays and plot
Theta_FL_Stacked = np.vstack((Theta_FL_stand, Theta_FL_SWH, Theta_FL_STW, Theta_FL))
Theta_FR_Stacked = np.vstack((Theta_FR_stand, Theta_FR_SWH, Theta_FR_STW, Theta_FR))
Theta_HL_Stacked = np.vstack((Theta_HL_stand, Theta_HL_SWH, Theta_HL_STW, Theta_HL))
Theta_HR_Stacked = np.vstack((Theta_HR_stand, Theta_HR_SWH, Theta_HR_STW, Theta_HR))


##### FIGURE 4 ##########
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
plt.axvline(x=dt + total_time_SWH, color='k', linestyle='--', linewidth=0.5)
plt.axvline(x=dt + total_time_SWH + total_time_STW_with_transfer, color='k', linestyle='--', linewidth=0.5)
plt.axvline(x=dt + total_time_SWH + total_time_STW_with_transfer + total_time_with_transfer, color='k', linestyle='--', linewidth=0.5)

plt.subplot(3, 1, 2)
plt.plot(t_stacked, Theta_FL_Stacked[:, 1], label='FL')
plt.plot(t_stacked, Theta_FR_Stacked[:, 1], label='FR')
plt.plot(t_stacked, Theta_HL_Stacked[:, 1], label='HL')
plt.plot(t_stacked, Theta_HR_Stacked[:, 1], label='HR')
plt.xlabel('Time (s)')
plt.ylabel('Theta 2 (degrees)')
plt.legend()
plt.axvline(x=dt, color='k', linestyle='--', linewidth=0.5)
plt.axvline(x=dt + total_time_SWH, color='k', linestyle='--', linewidth=0.5)
plt.axvline(x=dt + total_time_SWH + total_time_STW_with_transfer, color='k', linestyle='--', linewidth=0.5)
plt.axvline(x=dt + total_time_SWH + total_time_STW_with_transfer + total_time_with_transfer, color='k', linestyle='--', linewidth=0.5)

plt.subplot(3, 1, 3)
plt.plot(t_stacked, Theta_FL_Stacked[:, 2], label='FL')
plt.plot(t_stacked, Theta_FR_Stacked[:, 2], label='FR')
plt.plot(t_stacked, Theta_HL_Stacked[:, 2], label='HL')
plt.plot(t_stacked, Theta_HR_Stacked[:, 2], label='HR')
plt.xlabel('Time (s)')
plt.ylabel('Theta 3 (degrees)')
plt.legend()
plt.axvline(x=dt, color='k', linestyle='--', linewidth=0.5)
plt.axvline(x=dt + total_time_SWH, color='k', linestyle='--', linewidth=0.5)
plt.axvline(x=dt + total_time_SWH + total_time_STW_with_transfer, color='k', linestyle='--', linewidth=0.5)
plt.axvline(x=dt + total_time_SWH + total_time_STW_with_transfer + total_time_with_transfer, color='k', linestyle='--', linewidth=0.5)
#plt.show()





### PLOT WITH RETURN

FL_Trajectory_Stacked_Return = np.hstack((P_FL_body_stand.reshape(3, 1), P_FL_body_SWH, FL_Stand_To_Walk, FL_Bezier_Trajectory_With_Transfer, FL_Stand_To_Walk[:, ::-1], P_FL_body_SWH[:, ::-1], P_FL_body_stand.reshape(3, 1)))
FR_Trajectory_Stacked_Return = np.hstack((P_FR_body_stand.reshape(3, 1), P_FR_body_SWH, FR_Stand_To_Walk, FR_Bezier_Trajectory_With_Transfer, FR_Stand_To_Walk[:, ::-1], P_FR_body_SWH[:, ::-1], P_FR_body_stand.reshape(3, 1)))
HL_Trajectory_Stacked_Return = np.hstack((P_HL_body_stand.reshape(3, 1), P_HL_body_SWH, HL_Stand_To_Walk, HL_Bezier_Trajectory_With_Transfer, HL_Stand_To_Walk[:, ::-1], P_HL_body_SWH[:, ::-1], P_HL_body_stand.reshape(3, 1)))
HR_Trajectory_Stacked_Return = np.hstack((P_HR_body_stand.reshape(3, 1), P_HR_body_SWH, HR_Stand_To_Walk, HR_Bezier_Trajectory_With_Transfer, HR_Stand_To_Walk[:, ::-1], P_HR_body_SWH[:, ::-1], P_HR_body_stand.reshape(3, 1)))



###################
###################
# Plot stacked trajectory for all legs
# With a thin vertical line a transition point between each of the 4 trajectory segments (stand height, stand to walk height, walk height to walk start, walk start to walk with transfer)
t_stacked_Return = np.linspace(0, dt + total_time_SWH + total_time_STW_with_transfer + total_time_with_transfer + total_time_STW_with_transfer + total_time_SWH + dt, FL_Trajectory_Stacked_Return.shape[1]) # Create time array for stacked trajectory with return
plt.figure(figsize=(14, 8))
plt.plot(t_stacked_Return, FL_Trajectory_Stacked_Return[0, :], label='FL')
plt.plot(t_stacked_Return, FR_Trajectory_Stacked_Return[0, :], label='FR')
plt.plot(t_stacked_Return, HL_Trajectory_Stacked_Return[0, :], label='HL')
plt.plot(t_stacked_Return, HR_Trajectory_Stacked_Return[0, :], label='HR')
#plt.title('Stand to Walk to Stand Trajectory')
plt.xlabel('Time (s)', fontsize=LABEL_SIZE)
plt.ylabel('X Position (m)', fontsize=LABEL_SIZE)
plt.axvspan(0, 1, color="lightgray", alpha=1, zorder=0, label="Change of Height")
plt.axvspan(t_stacked_Return[-1] - 1, t_stacked_Return[-1], color="lightgray", alpha=1, zorder=0)
plt.axvspan(1, dt + total_time_SWH + total_time_STW_with_transfer, color="lightgray", alpha=0.4, zorder=0, label="Stand to Walk Transition")
plt.axvspan(t_stacked_Return[-1] - (dt + total_time_SWH + total_time_STW_with_transfer), t_stacked_Return[-1] - 1, color="lightgray", alpha=0.4, zorder=0)
plt.axvline(x=dt, color='k', linestyle='--', linewidth=0.5)
plt.axvline(x=dt + total_time_SWH, color='k', linestyle='--', linewidth=0.5)
plt.axvline(x=dt + total_time_SWH + total_time_STW_with_transfer, color='k', linestyle='--', linewidth=0.5)
plt.axvline(x=dt + total_time_SWH + total_time_STW_with_transfer + total_time_with_transfer, color='k', linestyle='--', linewidth=0.5)
plt.axvline(x=dt + total_time_SWH + total_time_STW_with_transfer + total_time_with_transfer + total_time_STW_with_transfer, color='k', linestyle='--', linewidth=0.5)
plt.axvline(x=dt + total_time_SWH + total_time_STW_with_transfer + total_time_with_transfer + total_time_STW_with_transfer + total_time_SWH, color='k', linestyle='--', linewidth=0.5)
plt.tick_params(axis='both', labelsize=LABEL_SIZE)
plt.legend(loc="lower center", ncol=6, fontsize=LEGEND_SIZE, framealpha=0.9, bbox_to_anchor=(0.5, -0.15))
plt.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig(os.path.join(os.path.dirname(__file__), 'X_Position_Return.png'), dpi=300, bbox_inches='tight')




plt.figure(figsize=(14, 8))
plt.plot(t_stacked_Return, FL_Trajectory_Stacked_Return[1, :], label='FL',linewidth=3)
plt.plot(t_stacked_Return, FR_Trajectory_Stacked_Return[1, :], label='FR',linewidth=3)
plt.plot(t_stacked_Return, HL_Trajectory_Stacked_Return[1, :], label='HL',linestyle='--')
plt.plot(t_stacked_Return, HR_Trajectory_Stacked_Return[1, :], label='HR',linestyle='--')
plt.xlabel('Time (s)', fontsize=LABEL_SIZE)
plt.ylabel('Y Position (m)', fontsize=LABEL_SIZE)
plt.axvspan(0, 1, color="lightgray", alpha=0.9, zorder=0, label="Change of Height")
plt.axvspan(t_stacked_Return[-1] - 1, t_stacked_Return[-1], color="lightgray", alpha=0.9, zorder=0)
plt.axvspan(1, dt + total_time_SWH + total_time_STW_with_transfer, color="lightgray", alpha=0.2, zorder=0, label="Stand to Walk Transition")
plt.axvspan(t_stacked_Return[-1] - (dt + total_time_SWH + total_time_STW_with_transfer), t_stacked_Return[-1] - 1, color="lightgray", alpha=0.2, zorder=0)
plt.axvline(x=dt, color='k', linestyle='--', linewidth=0.5)
plt.axvline(x=dt + total_time_SWH, color='k', linestyle='--', linewidth=0.5)
plt.axvline(x=dt + total_time_SWH + total_time_STW_with_transfer, color='k', linestyle='--', linewidth=0.5)
plt.axvline(x=dt + total_time_SWH + total_time_STW_with_transfer + total_time_with_transfer, color='k', linestyle='--', linewidth=0.5)
plt.axvline(x=dt + total_time_SWH + total_time_STW_with_transfer + total_time_with_transfer + total_time_STW_with_transfer, color='k', linestyle='--', linewidth=0.5)
plt.axvline(x=dt + total_time_SWH + total_time_STW_with_transfer + total_time_with_transfer + total_time_STW_with_transfer + total_time_SWH, color='k', linestyle='--', linewidth=0.5)
plt.tick_params(axis='both', labelsize=LABEL_SIZE)
plt.legend(loc="lower center", ncol=6, fontsize=LEGEND_SIZE, framealpha=0.9, bbox_to_anchor=(0.5, -0.15))
plt.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig(os.path.join(os.path.dirname(__file__), 'Y_Position_Return.png'), dpi=300, bbox_inches='tight')

plt.figure(figsize=(14, 8))
plt.plot(t_stacked_Return, FL_Trajectory_Stacked_Return[2, :], label='FL')
plt.plot(t_stacked_Return, FR_Trajectory_Stacked_Return[2, :], label='FR')
plt.plot(t_stacked_Return, HL_Trajectory_Stacked_Return[2, :], label='HL')
plt.plot(t_stacked_Return, HR_Trajectory_Stacked_Return[2, :], label='HR')
plt.xlabel('Time (s)', fontsize=LABEL_SIZE)
plt.ylabel('Z Position (m)', fontsize=LABEL_SIZE)
plt.axvspan(0, 1, color="lightgray", alpha=0.9, zorder=0, label="Change of Height")
plt.axvspan(t_stacked_Return[-1] - 1, t_stacked_Return[-1], color="lightgray", alpha=0.9, zorder=0)
plt.axvspan(1, dt + total_time_SWH + total_time_STW_with_transfer, color="lightgray", alpha=0.2, zorder=0, label="Stand to Walk Transition")
plt.axvspan(t_stacked_Return[-1] - (dt + total_time_SWH + total_time_STW_with_transfer), t_stacked_Return[-1] - 1, color="lightgray", alpha=0.2, zorder=0)
plt.axvline(x=dt, color='k', linestyle='--', linewidth=0.5)
plt.axvline(x=dt + total_time_SWH, color='k', linestyle='--', linewidth=0.5)
plt.axvline(x=dt + total_time_SWH + total_time_STW_with_transfer, color='k', linestyle='--', linewidth=0.5)
plt.axvline(x=dt + total_time_SWH + total_time_STW_with_transfer + total_time_with_transfer, color='k', linestyle='--', linewidth=0.5)
plt.axvline(x=dt + total_time_SWH + total_time_STW_with_transfer + total_time_with_transfer + total_time_STW_with_transfer, color='k', linestyle='--', linewidth=0.5)
plt.axvline(x=dt + total_time_SWH + total_time_STW_with_transfer + total_time_with_transfer + total_time_STW_with_transfer + total_time_SWH, color='k', linestyle='--', linewidth=0.5)
plt.tick_params(axis='both', labelsize=LABEL_SIZE)
plt.legend(loc="lower center", ncol=6, fontsize=LEGEND_SIZE, framealpha=0.9, bbox_to_anchor=(0.5, -0.15))
plt.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig(os.path.join(os.path.dirname(__file__), 'Z_Position_Return.png'), dpi=300, bbox_inches='tight')
plt.show()




# Stack Theta_XX arrays with return
Theta_FL_Stacked_Return = np.vstack((Theta_FL_stand, Theta_FL_SWH, Theta_FL_STW, Theta_FL, Theta_FL_STW[::-1, :], Theta_FL_SWH[::-1, :], Theta_FL_stand))
Theta_FR_Stacked_Return = np.vstack((Theta_FR_stand, Theta_FR_SWH, Theta_FR_STW, Theta_FR, Theta_FR_STW[::-1, :], Theta_FR_SWH[::-1, :], Theta_FR_stand))
Theta_HL_Stacked_Return = np.vstack((Theta_HL_stand, Theta_HL_SWH, Theta_HL_STW, Theta_HL, Theta_HL_STW[::-1, :], Theta_HL_SWH[::-1, :], Theta_HL_stand))
Theta_HR_Stacked_Return = np.vstack((Theta_HR_stand, Theta_HR_SWH, Theta_HR_STW, Theta_HR, Theta_HR_STW[::-1, :], Theta_HR_SWH[::-1, :], Theta_HR_stand))








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
