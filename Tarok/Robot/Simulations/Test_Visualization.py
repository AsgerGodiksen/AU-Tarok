import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..')))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../Hardware')))

# Test script for visualization of up/down motion of all 4 legs based on forward and inverse kinematics functions
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
import time


from Robot.Kinematics.Inverse_Kinematics import*
from Robot.Kinematics.Forward_Kinematics import*
from Robot.Kinematics.Constant_Transforms import*
from Robot.Kinematics.Jacobian import*

################## MAIN SCRIPT ####################
### Trajectory generation ###
# Define kinematic body lengths
l_k = 0.7048  # Length of body in kinematic model (meters)
w_k = 0.220   # Width of body in kinematic model (meters)

# Define time series
dt = 0.01 # seconds (100 Hz)
total_time = 10  # Total time in seconds
num_time_steps = int(total_time / dt) + 1
t = np.linspace(0, total_time, num_time_steps)

# Define desired end-effector trajectory as function of time for all 4 legs (in body frame)
x_FL = x_FR = (l_k/2)*np.ones_like(t)  # X position in meters (constant)
x_HL = x_HR = (-l_k/2)*np.ones_like(t)  # X position in meters (constant)
y_FL = y_HL = (w_k/2 + 0.078)*np.ones_like(t)  # Y position in meters (constant)
y_FR = y_HR = (-w_k/2 - 0.078)*np.ones_like(t)  # Y position in meters (constant)
z = np.piecewise(t, [t < 5, t >=5], [lambda t: -0.46 + (0.12/5)*t, lambda t: -0.34 - (0.12/5)*(t-5)])  # Z position in meters (linear wave from -0.46 to -0.34 and back to -0.46 in 10 seconds)

# Define desired end effector velocity (foot velocity) as functions of time for all 4 legs (in body frame) - Note, it is the same for all legs in body frame for this trajectory
x_dot = np.zeros_like(t)  # X velocity in meters/second (constant)
y_dot = np.zeros_like(t)  # Y velocity in meters/second (constant)
z_dot = np.piecewise(t, [t < 5, t >= 5], [lambda t: 0.12/5*np.ones_like(t), lambda t: -0.12/5*np.ones_like(t)])  # Z velocity in meters/second (linear wave from 0.12 m/s to -0.12 m/s and back to 0.12 m/s in 10 seconds)

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

### Plotting ###
# Plot with 4 subplots showing joint velocities theta_dot1, theta_dot2, theta_dot3 for each leg over time
plt.figure(figsize=(12, 8))
plt.subplot(2, 2, 1)
plt.plot(t, np.rad2deg(Theta_dot_FL[0, :]), label='theta_dot1')
plt.plot(t, np.rad2deg(Theta_dot_FL[1, :]), label='theta_dot2')
plt.plot(t, np.rad2deg(Theta_dot_FL[2, :]), label='theta_dot3')
plt.ylabel('Joint Velocity (deg/s)')
plt.title('FL Leg')
plt.legend()

plt.subplot(2, 2, 2)
plt.plot(t, np.rad2deg(Theta_dot_FR[0, :]), label='theta_dot1')
plt.plot(t, np.rad2deg(Theta_dot_FR[1, :]), label='theta_dot2')
plt.plot(t, np.rad2deg(Theta_dot_FR[2, :]), label='theta_dot3')
plt.ylabel('Joint Velocity (deg/s)')
plt.title('FR Leg')
plt.legend()

plt.subplot(2, 2, 3)
plt.plot(t, np.rad2deg(Theta_dot_HL[0, :]), label='theta_dot1')
plt.plot(t, np.rad2deg(Theta_dot_HL[1, :]), label='theta_dot2')
plt.plot(t, np.rad2deg(Theta_dot_HL[2, :]), label='theta_dot3')
plt.xlabel('Time (s)')
plt.ylabel('Joint Velocity (deg/s)')
plt.title('HL Leg')
plt.legend()

plt.subplot(2, 2, 4)
plt.plot(t, np.rad2deg(Theta_dot_HR[0, :]), label='theta_dot1')
plt.plot(t, np.rad2deg(Theta_dot_HR[1, :]), label='theta_dot2')
plt.plot(t, np.rad2deg(Theta_dot_HR[2, :]), label='theta_dot3')
plt.xlabel('Time (s)')
plt.ylabel('Joint Velocity (deg/s)')
plt.title('HR Leg')
plt.legend()


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
ax.set_title('Leg Trajectories Up/Down Motion')
ax.view_init(elev=20, azim=45, roll=0,)
lineFL, = ax.plot([], [], [], 'o-', lw=2)
lineFR, = ax.plot([], [], [], 'o-', lw=2)
lineHL, = ax.plot([], [], [], 'o-', lw=2)
lineHR, = ax.plot([], [], [], 'o-', lw=2)
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
    th1_FL, th2_FL, th3_FL = Theta_FL[num]
    th1_FR, th2_FR, th3_FR = Theta_FR[num]
    th1_HL, th2_HL, th3_HL = Theta_HL[num]
    th1_HR, th2_HR, th3_HR = Theta_HR[num]

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
ani = animation.FuncAnimation(fig, update, frames=len(t), init_func=init,
                              interval=1, blit=True)
plt.show()