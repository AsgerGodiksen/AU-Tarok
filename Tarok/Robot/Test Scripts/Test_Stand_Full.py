# Script for stand test of full quadruped using position control

# Based on combination of Test_Visualization.py and Test_UpDown_FT_final.py (onedrive)

import sys
import os

# Add Robot/ to path so Kinematics and Hardware can be imported directly
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

# Imports
from Hardware.Motor_Controls import Position_Control
from Kinematics.Inverse_Kinematics import Inverse_Kinematics
from Kinematics.Constant_Transforms import T0_B
import time
import can
import numpy as np


###### MAIN SCRIPT ######
## PRECOMPUTATIONS ##
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
# forsæt linje 61 test_visualization.py


# Convert to motor control - tjek lmkring 114 og frem



# Send command to motors linje 138 og frem