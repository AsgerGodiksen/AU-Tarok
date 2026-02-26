
import numpy as np

def FL_T0_1(L1, L2, L3, theta1, theta2, theta3):
    # Input:
        # L1, L2, L3: lengths of the leg segments
        # theta1, theta2, theta3: joint angles in radians
    # Output:
        # R: Rotation matrix of the first joint (with respect to base frame)
        # P_org: Position of the first joint in the base frame of Left leg    
 
    # Compute rotation matrix
    R = np.array([[np.cos(theta1), -np.sin(theta1), 0],
                  [np.sin(theta1),  np.cos(theta1), 0],
                  [0,               0,              1]])

    # Compute position of the end effector
    P_org = np.array([[0],
                      [0],
                      [0]])
    return R, P_org

def FL_T0_2(L1, L2, L3, theta1, theta2, theta3):
    # Input:
        # L1, L2, L3: lengths of the leg segments
        # theta1, theta2, theta3: joint angles in radians
    # Output:
        # R: Rotation matrix of the second joint (with respect to base frame)
        # P_org: Position of the second joint in the base frame of Left leg

    # Compute rotation matrix
    R = np.array([[np.cos(theta1)*np.cos(theta2), -np.cos(theta1)*np.sin(theta2), -np.sin(theta1)],
                  [np.sin(theta1)*np.cos(theta2),  np.sin(theta1)*np.sin(theta2),  np.cos(theta1)],
                  [-np.sin(theta2),               -np.cos(theta2),                 0]])

    # Compute position of the second joint
    P_org = np.array([[-np.sin(theta1)*L1],
                      [ np.cos(theta1)*L1],
                      [0]])
    return R, P_org

def FL_T0_3(L1, L2, L3, theta1, theta2, theta3):
    # Input:
        # L1, L2, L3: lengths of the leg segments
        # theta1, theta2, theta3: joint angles in radians
    # Output:
        # R: Rotation matrix of the third joint (with respect to base frame)
        # P_org: Position of the third joint in the base frame of Left leg

    # Compute rotation matrix
    R = np.array([[np.cos(theta1)*np.cos(theta2 + theta3), -np.cos(theta1)*np.sin(theta2 + theta3), -np.sin(theta1)],
                  [np.sin(theta1)*np.cos(theta2 + theta3), -np.sin(theta1)*np.sin(theta2 + theta3),  np.cos(theta1)],
                  [-np.sin(theta2 + theta3),               -np.cos(theta2 + theta3),                 0]])

    # Compute position of the third joint
    P_org = np.array([[np.cos(theta1)*np.cos(theta2)*L2 - np.sin(theta1)*L1],
                      [np.sin(theta1)*np.cos(theta2)*L2 + np.cos(theta1)*L1],
                      [-np.sin(theta2)*L2]])
    return R, P_org

def FL_P0_end(L1, L2, L3, theta1, theta2, theta3):
    # Input:
        # L1, L2, L3: lengths of the leg segments
        # theta1, theta2, theta3: joint angles in radians
    # Output:
        # P_org: Position of the end effector (foot) in the base frame of Left leg    
 
    # Compute position of the end effector
    P_org = np.array([[np.cos(theta1)*np.cos(theta2 + theta3)*L3 + np.cos(theta1)*np.cos(theta2)*L2 - np.sin(theta1)*L1],
                      [np.sin(theta1)*np.cos(theta2 + theta3)*L3 + np.sin(theta1)*np.cos(theta2)*L2 + np.cos(theta1)*L1],
                      [-np.sin(theta2 + theta3)*L3 - np.sin(theta2)*L2]])
    return P_org

def FL_P0_2(L1, L2, L3, theta1, theta2, theta3):
    # Input:
        # L1, L2, L3: lengths of the leg segments
        # theta1, theta2, theta3: joint angles in radians
    # Output:
        # P_org: Position of the second joint in the base frame of Left leg

    # Compute position of the second joint
    P_org = np.array([[-np.sin(theta1)*L1],
                      [ np.cos(theta1)*L1],
                      [0]])
    return P_org

def FL_P0_3(L1, L2, L3, theta1, theta2, theta3):
    # Input:
        # L1, L2, L3: lengths of the leg segments
        # theta1, theta2, theta3: joint angles in radians
    # Output:
        # P_org: Position of the third joint in the base frame of Left leg

    # Compute position of the third joint
    P_org = np.array([[np.cos(theta1)*np.cos(theta2)*L2 - np.sin(theta1)*L1],
                      [np.sin(theta1)*np.cos(theta2)*L2 + np.cos(theta1)*L1],
                      [-np.sin(theta2)*L2]])
    return P_org