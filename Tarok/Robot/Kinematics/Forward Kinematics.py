
import numpy as np

# Forward Kinematics function
def FL_Forward_Kinematics(L1,L2,L3,theta1,theta2,theta3):
    # Input:
        # L1, L2, L3: lengths of the leg segments
        # theta1, theta2, theta3: joint angles in radians
    # Output:
        # R: Rotation matrix of the end effector (foot)
        # P_org: Position of the end effector (foot) in the base frame of Left leg    
 
    # Compute rotation matrix
    R = np.array([[np.cos(theta1)*np.cos(theta2 + theta3), -np.cos(theta1)*np.sin(theta2 + theta3), -np.sin(theta1)],
                  [np.sin(theta1)*np.cos(theta2 + theta3), -np.sin(theta1)*np.sin(theta2 + theta3),  np.cos(theta1)],
                  [-np.sin(theta2 + theta3),               -np.cos(theta2 + theta3),                 0]])
  
    # Compute position of the end effector
    P_org = np.array([[np.cos(theta1)*np.cos(theta2 + theta3)*L3 + np.cos(theta1)*np.cos(theta2)*L2 - np.sin(theta1)*L1],
                      [np.sin(theta1)*np.cos(theta2 + theta3)*L3 + np.sin(theta1)*np.cos(theta2)*L2 + np.cos(theta1)*L1],
                      [-np.sin(theta2 + theta3)*L3 - np.sin(theta2)*L2]])
    return R, P_org