## SIMPLE SCRIPT FOR TESTING HOW TO GENERATE BEZIER FROM STAND TO WALK


# Imports
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..')))
import numpy as np
from Robot import*



# Starts directly at first control point
# Ends directly at last control point

# Two following points with same y or x yields zero velocity in that direction
# Three different points in a row with same y or x yields zero acceleration in that direction


# Remember normalized to real time for velocity


###########
# REMEMBER RIGHT X AND Y OFFSET
###########
############


# Standard
X = np.array([-0.1, -0.12, -0.14, -0.14, 0, 0, 0.14, 0.14, 0.12, 0.1]) # [m]
Y = np.array([0.44, 0.44, 0.38, 0.38, 0.40, 0.40, 0.38, 0.38, 0.44, 0.44]) # [m]



# FL
FL_end_x = -0.0334
X = np.array([0, 0.01, 0.02, 0.02, -0.05, -0.05, -0.12, -0.12, -0.11, -0.1]) # [m]
X = np.array([0, -0.1*FL_end_x, -0.2*FL_end_x, -0.2*FL_end_x, 0.5*FL_end_x, 0.5*FL_end_x, 1.2*FL_end_x, 1.2*FL_end_x, 1.1*FL_end_x, FL_end_x]) # [m]
Y = np.array([0.44, 0.44, 0.38, 0.38, 0.40, 0.40, 0.38, 0.38, 0.44, 0.44]) # [m]

# FR
FR_end_x = 0.1
X = np.array([0, -0.1*FR_end_x, -0.2*FR_end_x, -0.2*FR_end_x, 0.5*FR_end_x, 0.5*FR_end_x, 1.2*FR_end_x, 1.2*FR_end_x, 1.1*FR_end_x, FR_end_x]) # [m]
Y = np.array([0.44, 0.44, 0.38, 0.38, 0.40, 0.40, 0.38, 0.38, 0.44, 0.44]) # [m]

# HL
HL_end_x = -0.1
X = np.array([0, -0.1*HL_end_x, -0.2*HL_end_x, -0.2*HL_end_x, 0.5*HL_end_x, 0.5*HL_end_x, 1.2*HL_end_x, 1.2*HL_end_x, 1.1*HL_end_x, HL_end_x]) # [m]
Y = np.array([0.44, 0.44, 0.38, 0.38, 0.40, 0.40, 0.38, 0.38, 0.44, 0.44]) # [m]

# HR
HR_end_x = 0.0334
X = np.array([0, -0.1*HR_end_x, -0.2*HR_end_x, -0.2*HR_end_x, 0.5*HR_end_x, 0.5*HR_end_x, 1.2*HR_end_x, 1.2*HR_end_x, 1.1*HR_end_x, HR_end_x]) # [m]
Y = np.array([0.44, 0.44, 0.38, 0.38, 0.40, 0.40, 0.38, 0.38, 0.44, 0.44]) # [m]



Scaling_Factor_X = 1
Scaling_Factor_Y = 1
Offset = 0.0

c_kX = Scaling_Factor_X * X
c_kY = Offset + Scaling_Factor_Y * Y

c_k = np.column_stack((c_kX, c_kY))



Total_Time = 1
Time_Steps = 101

Time_Vector = np.linspace(0, Total_Time, Time_Steps)

Time_Norm = Time_Vector / Total_Time

# arrays
Trajectory = np.zeros((Time_Steps, 2))
Velocity = np.zeros((Time_Steps, 2))


for i in range(Time_Steps):
    Trajectory[i, :], Velocity[i, :] = Bezier_Curve(c_k, Time_Norm[i])

# Convert velocity to real time
Velocity = Velocity / Total_Time


# Plot trajectory x,y with inverted y-axis
import matplotlib.pyplot as plt
plt.figure()
plt.plot(Trajectory[:, 0], Trajectory[:, 1], label='Trajectory')
plt.scatter(c_k[:, 0], c_k[:, 1], color='red', label='Control Points')
plt.title('Bezier Curve Trajectory')
plt.xlabel('X [m]')
plt.ylabel('Y [m]')
plt.legend()
plt.axis('equal')
plt.gca().invert_yaxis()  # Invert y-axis
plt.grid()
plt.show()

# Plot velocity x and y over time
plt.figure()
plt.plot(Time_Vector, Velocity[:, 0], label='Velocity X')
plt.plot(Time_Vector, -Velocity[:, 1], label='Velocity Y')
plt.title('Bezier Curve Velocity')
plt.xlabel('Time [s]')
plt.ylabel('Velocity [m/s]')
plt.legend()
plt.grid()
plt.show()





# Transform to body frame

# Generating arrays Trajectory
x = Trajectory[:,0]
y = np.zeros(Time_Steps)
z = - Trajectory[:,1]

# Generating Arrays for the Velocity
x_dot = Trajectory[:,0]
y_dot = np.zeros(Time_Steps)    
z_dot = - Trajectory[:,1]

# Exports
Position_Body_Bezier = np.vstack((x,y,z))
Velocity_Body_Bezier = np.vstack((x_dot, y_dot, z_dot))


# Plot trajectory x,z
plt.figure()
plt.plot(x, z, label='Trajectory')
plt.title('Bezier Curve Trajectory')
plt.xlabel('X [m]')
plt.ylabel('Z [m]')
plt.legend()
plt.axis('equal')
plt.grid()
plt.show()