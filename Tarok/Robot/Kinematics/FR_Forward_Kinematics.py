# Forward kinematics of front right leg of quadruped robot
import numpy as np

# Constant parameters used:
    # L1 = 0.078 m (length of the first leg segment)
    # L2 = 0.20 m (length of the second leg segment)
    # L3 = 0.30 m (length of the third leg segment)

def FR_Forward_Kinematics(theta1,theta2,theta3):
    # Input:
        # theta1, theta2, theta3: joint angles in radians
    # Output:
        # R: Rotation matrix of the end effector (foot)
        # P_org: Position of the end effector (foot) in the base frame of Right leg    

    # Compute rotation matrix
    R = np.array([[np.cos(theta1)*np.cos(theta2 + theta3), -np.cos(theta1)*np.sin(theta2 + theta3),  np.sin(theta1)],
                  [np.sin(theta1)*np.cos(theta2 + theta3), -np.sin(theta1)*np.sin(theta2 + theta3), -np.cos(theta1)],
                  [np.sin(theta2 + theta3),                 np.cos(theta2 + theta3),                 0]])
  
    # Compute position of the end effector
    P_org = np.array([[np.cos(theta1)*np.cos(theta2 + theta3)*0.3 + np.cos(theta1)*np.cos(theta2)*0.2 + np.sin(theta1)*0.078],
                      [np.sin(theta1)*np.cos(theta2 + theta3)*0.3 + np.sin(theta1)*np.cos(theta2)*0.2 - np.cos(theta1)*0.078],
                      [np.sin(theta2 + theta3)*0.3 + np.sin(theta2)*0.2]])
    return R, P_org

def FR_T0_1(theta1, theta2, theta3):
    # Input:
        # theta1, theta2, theta3: joint angles in radians
    # Output:
        # R: Rotation matrix of the first joint (with respect to base frame)
        # P_org: Position of the first joint in the base frame of Right leg    
 
    # Compute rotation matrix
    R = np.array([[np.cos(theta1), -np.sin(theta1), 0],
                  [np.sin(theta1),  np.cos(theta1), 0],
                  [0,               0,              1]])

    # Compute position of the end effector
    P_org = np.array([[0],
                      [0],
                      [0]])
    return R, P_org

def FR_T0_2(theta1, theta2, theta3):
    # Input:
        # theta1, theta2, theta3: joint angles in radians
    # Output:
        # R: Rotation matrix of the second joint (with respect to base frame)
        # P_org: Position of the second joint in the base frame of Right leg

    # Compute rotation matrix
    R = np.array([[np.cos(theta1)*np.cos(theta2), -np.cos(theta1)*np.sin(theta2),  np.sin(theta1)],
                  [np.sin(theta1)*np.cos(theta2), -np.sin(theta1)*np.sin(theta2), -np.cos(theta1)],
                  [np.sin(theta2),                 np.cos(theta2),                 0]])

    # Compute position of the second joint
    P_org = np.array([[np.sin(theta1)*0.078],
                      [-np.cos(theta1)*0.078],
                      [0]])
    return R, P_org

def FR_T0_3(theta1, theta2, theta3):
    # Input:
        # theta1, theta2, theta3: joint angles in radians
    # Output:
        # R: Rotation matrix of the third joint (with respect to base frame)
        # P_org: Position of the third joint in the base frame of Right leg

    # Compute rotation matrix
    R = np.array([[np.cos(theta1)*np.cos(theta2 + theta3), -np.cos(theta1)*np.sin(theta2 + theta3),  np.sin(theta1)],
                  [np.sin(theta1)*np.cos(theta2 + theta3), -np.sin(theta1)*np.sin(theta2 + theta3), -np.cos(theta1)],
                  [np.sin(theta2 + theta3),                 np.cos(theta2 + theta3),                 0]])

    # Compute position of the third joint
    P_org = np.array([[np.cos(theta1)*np.cos(theta2)*0.2 + np.sin(theta1)*0.078],
                      [np.sin(theta1)*np.cos(theta2)*0.2 - np.cos(theta1)*0.078],
                      [np.sin(theta2)*0.2]])
    return R, P_org

def FR_P0_end(theta1, theta2, theta3):
    # Input:
        # theta1, theta2, theta3: joint angles in radians
    # Output:
        # P_org: Position of the end effector (foot) in the base frame of Right leg    
 
    # Compute position of the end effector
    P_org = np.array([[np.cos(theta1)*np.cos(theta2 + theta3)*0.3 + np.cos(theta1)*np.cos(theta2)*0.2 + np.sin(theta1)*0.078],
                      [np.sin(theta1)*np.cos(theta2 + theta3)*0.3 + np.sin(theta1)*np.cos(theta2)*0.2 - np.cos(theta1)*0.078],
                      [np.sin(theta2 + theta3)*0.3 + np.sin(theta2)*0.2]])
    return P_org

def FR_P0_2(theta1, theta2, theta3):
    # Input:
        # theta1, theta2, theta3: joint angles in radians
    # Output:
        # P_org: Position of the second joint in the base frame of Right leg

    # Compute position of the second joint
    P_org = np.array([[np.sin(theta1)*0.078],
                      [-np.cos(theta1)*0.078],
                      [0]])
    return P_org

def FR_P0_3(theta1, theta2, theta3):
    # Input:
        # theta1, theta2, theta3: joint angles in radians
    # Output:
        # P_org: Position of the third joint in the base frame of Right leg

    # Compute position of the third joint
    P_org = np.array([[np.cos(theta1)*np.cos(theta2)*0.2 + np.sin(theta1)*0.078],
                      [np.sin(theta1)*np.cos(theta2)*0.2 - np.cos(theta1)*0.078],
                      [np.sin(theta2)*0.2]])
    return P_org