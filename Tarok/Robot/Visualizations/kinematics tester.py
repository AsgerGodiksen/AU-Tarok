import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

# Test script for playing around with kinematics functions and more
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
import time
from Robot import *

################## MAIN SCRIPT ####################
### Trajectory generation ###
# Define kinematic body lengths
l_k = 0.7048  # Length of body in kinematic model (meters)
w_k = 0.220   # Width of body in kinematic model (meters)
L_1 = 0.078  # Length of first leg segment (meters)

t = np.linspace(0, 5, 100)

x_FL = np.linspace(-0.05, 0.05, 100) + l_k/2
y_FL = np.zeros_like(t) + w_k/2 + L_1
z_FL = np.zeros_like(t) - 0.41

x_FR = np.linspace(-0.05, 0.05, 100) + l_k/2
y_FR = np.zeros_like(t) - w_k/2 - L_1
z_FR = np.zeros_like(t) - 0.41

# FL leg
# plot x, y and z vs t
plt.figure()
plt.subplot(1, 3, 1)
plt.plot(t, x_FL, label='X Position')
plt.xlabel('Time (s)')
plt.ylabel('Position (m)')
plt.legend()
plt.grid()

plt.subplot(1, 3, 2)
plt.plot(t, y_FL, label='Y Position')
plt.xlabel('Time (s)')
plt.ylabel('Position (m)')
plt.legend()
plt.title('FL Leg Position vs Time')
plt.grid()

plt.subplot(1, 3, 3)
plt.plot(t, z_FL, label='Z Position')
plt.xlabel('Time (s)')
plt.ylabel('Position (m)')
plt.legend()
plt.grid()
plt.grid()


# 3d plot x, y, z
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(x_FL, y_FL, z_FL, label='Leg Trajectory')
ax.set_xlabel('X Position (m)')
ax.set_ylabel('Y Position (m)')
ax.set_zlabel('Z Position (m)')
ax.set_title('FL 3D Leg Trajectory')
ax.legend()


# FR leg
# plot x, y and z vs t
plt.figure()
plt.subplot(1, 3, 1)
plt.plot(t, x_FR, label='X Position')
plt.xlabel('Time (s)')
plt.ylabel('Position (m)')
plt.legend()
plt.grid()

plt.subplot(1, 3, 2)
plt.plot(t, y_FR, label='Y Position')
plt.xlabel('Time (s)')
plt.ylabel('Position (m)')
plt.legend()
plt.title('FR Leg Position vs Time')
plt.grid()

plt.subplot(1, 3, 3)
plt.plot(t, z_FR, label='Z Position')
plt.xlabel('Time (s)')
plt.ylabel('Position (m)')
plt.legend()
plt.grid()
plt.grid()


# 3d plot x, y, z
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(x_FR, y_FR, z_FR, label='Leg Trajectory')
ax.set_xlabel('X Position (m)')
ax.set_ylabel('Y Position (m)')
ax.set_zlabel('Z Position (m)')
ax.set_title('FR 3D Leg Trajectory')
ax.legend()
plt.show()









# Combine x, y, z into a single array of shape (3, num_time_steps)
P_body_FL = np.vstack((x_FL, y_FL, z_FL))
P_body_FR = np.vstack((x_FR, y_FR, z_FR))

# Transform trajectory from body frame to base frame using T0_B for each point in the trajectory
P_base_FL = np.array([T0_B(P_body_FL[:, i].reshape((3, 1)), 'FL') for i in range(len(t))])
P_base_FR = np.array([T0_B(P_body_FR[:, i].reshape((3, 1)), 'FR') for i in range(len(t))])

# Compute inverse kinematics for each point in the trajectory
Theta_FL = np.array([Inverse_Kinematics(P_base_FL[i], 'FL') for i in range(len(t))]) # Shape (num_time_steps, 3), containing theta1, theta2, theta3 for each time step
Theta_FR = np.array([Inverse_Kinematics(P_base_FR[i], 'FR') for i in range(len(t))]) # Shape (num_time_steps, 3), containing theta1, theta2, theta3 for each time step


# plot theta 1, theta2 and theta3 vs t
plt.figure()
plt.subplot(1, 3, 1)
plt.plot(t, np.rad2deg(Theta_FL[:, 0]), label='Theta 1')
plt.xlabel('Time (s)')
plt.ylabel('Angle (deg)')
plt.legend()
plt.grid()

plt.subplot(1, 3, 2)
plt.plot(t, np.rad2deg(Theta_FL[:, 1]), label='Theta 2')
plt.xlabel('Time (s)')
plt.ylabel('Angle (deg)')
plt.legend()
plt.title('FL Leg Angles vs Time')
plt.grid()

plt.subplot(1, 3, 3)
plt.plot(t, np.rad2deg(Theta_FL[:, 2]), label='Theta 3')
plt.xlabel('Time (s)')
plt.ylabel('Angle (deg)')
plt.legend()
plt.grid()
plt.grid()


# plot theta 1, theta2 and theta3 vs t
plt.figure()
plt.subplot(1, 3, 1)
plt.plot(t, np.rad2deg(Theta_FR[:, 0]), label='Theta 1')
plt.xlabel('Time (s)')
plt.ylabel('Angle (deg)')
plt.legend()
plt.grid()

plt.subplot(1, 3, 2)
plt.plot(t, np.rad2deg(Theta_FR[:, 1]), label='Theta 2')
plt.xlabel('Time (s)')
plt.ylabel('Angle (deg)')
plt.legend()
plt.title('FR Leg Angles vs Time')
plt.grid()

plt.subplot(1, 3, 3)
plt.plot(t, np.rad2deg(Theta_FR[:, 2]), label='Theta 3')
plt.xlabel('Time (s)')
plt.ylabel('Angle (deg)')
plt.legend()
plt.grid()
plt.grid()


plt.show()





'''
theta1 = np.deg2rad(20)
theta2 = np.deg2rad(0)
theta3 = np.deg2rad(0)

Left_foot = P0_end(theta1, theta2, theta3, 'FL')

right_foot = P0_end(theta1, theta2, theta3, 'FR')

left_hind_foot = P0_end(theta1, theta2, theta3, 'HL')

right_hind_foot = P0_end(theta1, theta2, theta3, 'HR')


print(Left_foot)
print(right_foot)
print(left_hind_foot)
print(right_hind_foot)



left_theta1, left_theta2, left_theta3 = Inverse_Kinematics(Left_foot, 'FL')
right_theta1, right_theta2, right_theta3 = Inverse_Kinematics(right_foot, 'FR')
left_hind_theta1, left_hind_theta2, left_hind_theta3 = Inverse_Kinematics(left_hind_foot, 'HL')
right_hind_theta1, right_hind_theta2, right_hind_theta3 = Inverse_Kinematics(right_hind_foot, 'HR')


print(np.rad2deg(left_theta1), np.rad2deg(left_theta2), np.rad2deg(left_theta3))
print(np.rad2deg(right_theta1), np.rad2deg(right_theta2), np.rad2deg(right_theta3))
print(np.rad2deg(left_hind_theta1), np.rad2deg(left_hind_theta2), np.rad2deg(left_hind_theta3))
print(np.rad2deg(right_hind_theta1), np.rad2deg(right_hind_theta2), np.rad2deg(right_hind_theta3))

'''