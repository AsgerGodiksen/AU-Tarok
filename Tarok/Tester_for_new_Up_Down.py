# Test script for new Up/Down

import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..')))
import numpy as np
from Robot import*
import matplotlib.pyplot as plt



# Helper functions for trajectory generation
def cos_interp(t, z_start, z_end, t_start, t_end):
    '''Smooth cosine interpolation from z_start to z_end over [t_start, t_end]'''
    tau = (t - t_start) / (t_end - t_start)  # Normalized time 0->1
    return z_start + (z_end - z_start) * 0.5 * (1 - np.cos(np.pi * tau))

def cos_interp_dot(t, z_start, z_end, t_start, t_end):
    '''Derivative of cosine interpolation'''
    duration = t_end - t_start
    tau = (t - t_start) / duration
    return (z_end - z_start) * 0.5 * np.pi / duration * np.sin(np.pi * tau)

# Define kinematic body lengths
l_k = 0.7048  # Length of body in kinematic model (meters)
w_k = 0.220   # Width of body in kinematic model (meters)

# Define local time series
dt = 0.005 # seconds (200 Hz) - not directly the control frequency, but the discretization used for precomputations
total_time = 6  # Total time in seconds of one cycle of the trajectory
num_time_steps = int(total_time / dt) + 1
t = np.linspace(0, total_time, num_time_steps)

# Segment boundaries for the trajectory: 0->3s, 3->6s
conditions = [(t >= 0)   & (t < 3),   # Up:    -0.36 -> -0.46
              (t >= 3)   & (t < 6),   # Down:  -0.46 -> -0.36
              (t >= 6)]               # Hold:  -0.36 (hold at -0.36 after 6s))

# Define desired end-effector trajectory as function of time for all 4 legs (in body frame)
x_FL = x_FR = (l_k/2)*np.ones_like(t)  # X position in meters (constant)
x_HL = x_HR = (-l_k/2)*np.ones_like(t)  # X position in meters (constant)
y_FL = y_HL = (w_k/2 + 0.078)*np.ones_like(t)  # Y position in meters (constant)
y_FR = y_HR = (-w_k/2 - 0.078)*np.ones_like(t)  # Y position in meters (constant)
z = np.piecewise(t, conditions, [lambda t: cos_interp(t, -0.36, -0.46, 0, 3),
                                 lambda t: cos_interp(t, -0.46, -0.36, 3, 6),
                                 lambda t: -0.36*np.ones_like(t)])  # Z position in meters (cosine wave from -0.36 to -0.46, then to -0.36, then hold at -0.36)

# Define desired end effector velocity (foot velocity) as functions of time for all 4 legs (in body frame) - Note, it is the same for all legs in body frame for this trajectory
x_dot = np.zeros_like(t)  # X velocity in meters/second (constant)
y_dot = np.zeros_like(t)  # Y velocity in meters/second (constant)
z_dot = np.piecewise(t, conditions, [lambda t: cos_interp_dot(t, -0.36, -0.46, 0, 3),
                                     lambda t: cos_interp_dot(t, -0.46, -0.36, 3, 6),
                                     lambda t: np.zeros_like(t)])

### Transformations ###
# Combine trajectories into position arrays for each leg
P_FL_body = np.vstack((x_FL, y_FL, z))
P_FR_body = np.vstack((x_FR, y_FR, z))
P_HL_body = np.vstack((x_HL, y_HL, z))
P_HR_body = np.vstack((x_HR, y_HR, z))

# Transform desired end-effector trajectory from body frame to leg base frames
P_FL_base = np.array([T0_B(P_FL_body[:, i].reshape((3, 1)), 'FL') for i in range(len(t))])
P_FR_base = np.array([T0_B(P_FR_body[:, i].reshape((3, 1)), 'FR') for i in range(len(t))])
P_HL_base = np.array([T0_B(P_HL_body[:, i].reshape((3, 1)), 'HL') for i in range(len(t))])
P_HR_base = np.array([T0_B(P_HR_body[:, i].reshape((3, 1)), 'HR') for i in range(len(t))])

# Combine body frame trajectory cartesian velocities into array
V_body = np.vstack((x_dot, y_dot, z_dot))
# Transform desired end-effector velocity from body frame to leg base frames
V_FL_base = np.array([R0_B(V_body[:, i].reshape((3, 1)), 'FL') for i in range(len(t))])
V_FR_base = np.array([R0_B(V_body[:, i].reshape((3, 1)), 'FR') for i in range(len(t))])
V_HL_base = np.array([R0_B(V_body[:, i].reshape((3, 1)), 'HL') for i in range(len(t))])
V_HR_base = np.array([R0_B(V_body[:, i].reshape((3, 1)), 'HR') for i in range(len(t))])

### Kinematics ###
# Determine joint angles for all 4 legs using inverse kinematics
Theta_FL = np.array([Inverse_Kinematics(P_FL_base[i], 'FL') for i in range(len(t))]) # Shape (num_time_steps, 3), containing theta1, theta2, theta3 for each time step
Theta_FR = np.array([Inverse_Kinematics(P_FR_base[i], 'FR') for i in range(len(t))]) # Shape (num_time_steps, 3), containing theta1, theta2, theta3 for each time step
Theta_HL = np.array([Inverse_Kinematics(P_HL_base[i], 'HL') for i in range(len(t))]) # Shape (num_time_steps, 3), containing theta1, theta2, theta3 for each time step
Theta_HR = np.array([Inverse_Kinematics(P_HR_base[i], 'HR') for i in range(len(t))]) # Shape (num_time_steps, 3), containing theta1, theta2, theta3 for each time step

# Determine joint velocities for all 4 legs using Jacobian
# Damped least squares inverse to avoid singularities - theta_dot = (J^T*J + damp^2*I)^-1 * J^T * cartesian_velocity 
Theta_dot_FL = np.zeros((3, len(t)))  # Initialize joint velocity array
Theta_dot_FR = np.zeros((3, len(t)))  # Initialize joint velocity array
Theta_dot_HL = np.zeros((3, len(t)))  # Initialize joint velocity array
Theta_dot_HR = np.zeros((3, len(t)))  # Initialize joint velocity array
damp = 0.001  # Damping factor
for i in range(len(t)):
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

# Roll arrays by 501 time steps to start at the standing position
Theta_FL = np.roll(Theta_FL, 501, axis=0)
Theta_FR = np.roll(Theta_FR, 501, axis=0)
Theta_HL = np.roll(Theta_HL, 501, axis=0)
Theta_HR = np.roll(Theta_HR, 501, axis=0)
Theta_dot_FL = np.roll(Theta_dot_FL, 501, axis=1)
Theta_dot_FR = np.roll(Theta_dot_FR, 501, axis=1)
Theta_dot_HL = np.roll(Theta_dot_HL, 501, axis=1)
Theta_dot_HR = np.roll(Theta_dot_HR, 501, axis=1)


# Plot z and z_dot vs time in different subplots to verify trajectory
plt.figure()
plt.subplot(2, 1, 1)
plt.plot(t, z)
plt.xlabel('Time')
plt.ylabel('Z Position')
plt.title('End-Effector Trajectory')
plt.subplot(2, 1, 2)
plt.plot(t, z_dot)
plt.xlabel('Time')
plt.ylabel('Z Velocity')



# plot Theta_Fl[0, :], Theta_FL[1, :], Theta_FL[2, :] vs time in different subplots to verify joint angles
plt.figure()
plt.subplot(3, 1, 1)
plt.plot(t, Theta_FL[:, 0])
plt.xlabel('Time')
plt.ylabel('Theta1 (degrees)')
plt.title('Joint Angles for Front Left Leg')
plt.subplot(3, 1, 2)
plt.plot(t, Theta_FL[:, 1])
plt.xlabel('Time')
plt.ylabel('Theta2 (degrees)')
plt.subplot(3, 1, 3)
plt.plot(t, Theta_FL[:, 2])
plt.xlabel('Time')
plt.ylabel('Theta3 (degrees)')
plt.show()