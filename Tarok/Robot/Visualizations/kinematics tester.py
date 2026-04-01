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

