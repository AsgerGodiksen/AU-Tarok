# Jacobian of front left leg of quadruped robot
import numpy as np

def FL_Jacobian(L1,L2,L3,theta1,theta2,theta3):
    # Input:
        # L1, L2, L3: lengths of the leg segments
        # theta1, theta2, theta3: joint angles in radians
    # Output:
        # J: Jacobian matrix of the end effector (foot)   
 
    # Compute Jacobian matrix
    J = np.array([[-np.sin(theta1)*np.cos(theta2 + theta3)*L3 - np.sin(theta1)*np.cos(theta2)*L2 - np.cos(theta1)*L1, 
                   -np.cos(theta1)*np.sin(theta2 + theta3)*L3 - np.cos(theta1)*np.sin(theta2)*L2, 
                   -np.cos(theta1)*np.sin(theta2 + theta3)*L3],
                  [ np.cos(theta1)*np.cos(theta2 + theta3)*L3 + np.cos(theta1)*np.cos(theta2)*L2 - np.sin(theta1)*L1, 
                   -np.sin(theta1)*np.sin(theta2 + theta3)*L3 - np.sin(theta1)*np.sin(theta2)*L2, 
                   -np.sin(theta1)*np.sin(theta2 + theta3)*L3],
                  [0, 
                   -np.cos(theta2 + theta3)*L3 - np.cos(theta2)*L2, 
                   -np.cos(theta2 + theta3)*L3]])
    return J