# This file will visualize the Robots Initial configuration


import sys
import os
# Add Robot/ to path so Kinematics and Hardware can be imported directly
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))


from Robot.Kinematics.Inverse_Kinematics import*
from Robot.Kinematics.Forward_Kinematics import*
from Robot.Kinematics.Constant_Transforms import*
from Robot.Kinematics.Jacobian import*

import matplotlib.pyplot as plt


l_k = 0.7048  # Length of body in kinematic model (meters)
w_k = 0.220   # Width of body in kinematic model (meters)
z = -0.41

# Define End-Effetor position
x_FL = x_FR = (l_k/2) # X position in meters (constant)
x_HL = x_HR = (-l_k/2) # X position in meters (constant)
y_FL = y_HL = (w_k/2 + 0.078)  # Y position in meters (constant)
y_FR = y_HR = (-w_k/2 - 0.078) # Y position in meters (constant)


# Combine trajectories into position arrays for each leg
P_FL_body = np.vstack((x_FL, y_FL, z))
P_FR_body = np.vstack((x_FR, y_FR, z))
P_HL_body = np.vstack((x_HL, y_HL, z))
P_HR_body = np.vstack((x_HR, y_HR, z)) 

# Transform desired end-effector velocity from body frame to leg base frames
P_FL_Base = T0_B(P_FL_body,'FL')
P_FR_Base = T0_B(P_FR_body,'FR')
P_HL_Base = T0_B(P_HL_body,'HL')
P_HR_Base = T0_B(P_HR_body,'HR')

### Kinematics ###
# Determine joint angles for all 4 legs using inverse kinematics'
Theta_FL = Inverse_Kinematics(P_FL_Base,'FL')
Theta_FR = Inverse_Kinematics(P_FR_Base,'FR')
Theta_HL = Inverse_Kinematics(P_HL_Base,'HL')
Theta_HR = Inverse_Kinematics(P_HR_Base,'HR')

# Get current joint angles
th1_FL, th2_FL, th3_FL = Theta_FL
th1_FR, th2_FR, th3_FR = Theta_FR
th1_HL, th2_HL, th3_HL = Theta_HL
th1_HR, th2_HR, th3_HR = Theta_HR

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


# Plotting
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
plt.title('Initial Position Configuration')
ax.view_init(elev=20, azim=225)

def get_xyz(p):
    """Extract x, y, z from a (3,1) or (3,) array."""
    p = np.array(p).flatten()
    return p[0], p[1], p[2]

def plot_leg(ax, p1, p2, p3, p4, color, label):
    """Plot one leg: joint1 → joint2 → joint3 → end effector."""
    xs = [get_xyz(p)[0] for p in [p1, p2, p3, p4]]
    ys = [get_xyz(p)[1] for p in [p1, p2, p3, p4]]
    zs = [get_xyz(p)[2] for p in [p1, p2, p3, p4]]
    ax.plot(xs, ys, zs, 'o-', lw=2, color=color, label=label)

plot_leg(ax, P0_1_FL, P0_2_FL, P0_3_FL, P0_end_FL, color='blue',   label='FL')
plot_leg(ax, P0_1_FR, P0_2_FR, P0_3_FR, P0_end_FR, color='red',    label='FR')
plot_leg(ax, P0_1_HL, P0_2_HL, P0_3_HL, P0_end_HL, color='green',  label='HL')
plot_leg(ax, P0_1_HR, P0_2_HR, P0_3_HR, P0_end_HR, color='orange', label='HR')
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Z (m)')
ax.legend()
plt.tight_layout()
plt.show()