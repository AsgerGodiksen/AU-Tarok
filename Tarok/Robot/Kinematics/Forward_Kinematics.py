# Forward kinematics of quadruped robot
import numpy as np

# Constant parameters used:
    # L1 = 0.078 m (length of the first leg segment)
    # L2 = 0.20 m (length of the second leg segment)
    # L3 = 0.30 m (length of the third leg segment)

def Forward_Kinematics(theta1,theta2,theta3, Leg):
    # Input:
        # theta1, theta2, theta3: joint angles in radians
        # Leg: 'FL', 'FR', 'HL' or 'HR' to specify considered leg
    # Output:
        # R: Rotation matrix of the end effector (foot)
        # P_org: Position of the end effector (foot) in the base frame of the specified leg 
 
    # Specify config variable based on leg
    if Leg == 'FL' or Leg == 'HR':
        config = 1
    elif Leg == 'FR' or Leg == 'HL':
        config = -1

    # Compute rotation matrix
    R = np.array([[ np.cos(theta1)*np.cos(theta2 + theta3), -np.cos(theta1)*np.sin(theta2 + theta3), -config*np.sin(theta1)],
                  [ np.sin(theta1)*np.cos(theta2 + theta3), -np.sin(theta1)*np.sin(theta2 + theta3),  config*np.cos(theta1)],
                  [-config*np.sin(theta2 + theta3),         -config*np.cos(theta2 + theta3),          0]])
  
    # Compute position of the end effector
    P_org = np.array([[np.cos(theta1)*np.cos(theta2 + theta3)*0.3 + np.cos(theta1)*np.cos(theta2)*0.2 - config*np.sin(theta1)*0.078],
                      [np.sin(theta1)*np.cos(theta2 + theta3)*0.3 + np.sin(theta1)*np.cos(theta2)*0.2 + config*np.cos(theta1)*0.078],
                      [-config*np.sin(theta2 + theta3)*0.3 - config*np.sin(theta2)*0.2]])
    return R, P_org

def T0_1(theta1, theta2, theta3, Leg):
    # Input:
        # theta1, theta2, theta3: joint angles in radians
        # Leg: 'FL', 'FR', 'HL' or 'HR' to specify considered leg
    # Output:
        # R: Rotation matrix of the first joint (with respect to base frame)
        # P_org: Position of the first joint in the base frame of the specified leg    
 
    # Compute rotation matrix
    R = np.array([[np.cos(theta1), -np.sin(theta1), 0],
                  [np.sin(theta1),  np.cos(theta1), 0],
                  [0,               0,              1]])

    # Compute position of the end effector
    P_org = np.array([[0],
                      [0],
                      [0]])
    return R, P_org

def T0_2(theta1, theta2, theta3, Leg):
    # Input:
        # theta1, theta2, theta3: joint angles in radians
        # Leg: 'FL', 'FR', 'HL' or 'HR' to specify considered leg
    # Output:
        # R: Rotation matrix of the second joint (with respect to base frame)
        # P_org: Position of the second joint in the base frame of the specified leg

    # Specify config variable based on leg
    if Leg == 'FL' or Leg == 'HR':
        config = 1
    elif Leg == 'FR' or Leg == 'HL':
        config = -1

    # Compute rotation matrix
    R = np.array([[np.cos(theta1)*np.cos(theta2), -np.cos(theta1)*np.sin(theta2), -config*np.sin(theta1)],
                  [np.sin(theta1)*np.cos(theta2),  np.sin(theta1)*np.sin(theta2),  config*np.cos(theta1)],
                  [-config*np.sin(theta2),        -config*np.cos(theta2),          0]])

    # Compute position of the second joint
    P_org = np.array([[-config*np.sin(theta1)*0.078],
                      [ config*np.cos(theta1)*0.078],
                      [ 0]])
    return R, P_org

def T0_3(theta1, theta2, theta3, Leg):
    # Input:
        # theta1, theta2, theta3: joint angles in radians
        # Leg: 'FL', 'FR', 'HL' or 'HR' to specify considered leg
    # Output:
        # R: Rotation matrix of the third joint (with respect to base frame)
        # P_org: Position of the third joint in the base frame of the specified leg

    # Specify config variable based on leg
    if Leg == 'FL' or Leg == 'HR':
        config = 1
    elif Leg == 'FR' or Leg == 'HL':
        config = -1

    # Compute rotation matrix
    R = np.array([[np.cos(theta1)*np.cos(theta2 + theta3), -np.cos(theta1)*np.sin(theta2 + theta3), -config*np.sin(theta1)],
                  [np.sin(theta1)*np.cos(theta2 + theta3), -np.sin(theta1)*np.sin(theta2 + theta3),  config*np.cos(theta1)],
                  [-config*np.sin(theta2 + theta3),               -config*np.cos(theta2 + theta3),                 0]])

    # Compute position of the third joint
    P_org = np.array([[np.cos(theta1)*np.cos(theta2)*0.2 - config*np.sin(theta1)*0.078],
                      [np.sin(theta1)*np.cos(theta2)*0.2 + config*np.cos(theta1)*0.078],
                      [-config*np.sin(theta2)*0.2]])
    return R, P_org

def P0_end(theta1, theta2, theta3, Leg):
    """ 
    Position of the End Effector

    Args:
        theta1: joint angle in radians
        theta2: joint angle in radians
        theta3: joint angle in radians
        Leg: 'FL', 'FR', 'HL' or 'HR' to specify considered leg

    Returns:
        P_org: Position of the end effector (foot) in the base frame of the specified leg
    """
    # Specify config variable based on leg
    if Leg == 'FL' or Leg == 'HR':
        config = 1
    elif Leg == 'FR' or Leg == 'HL':
        config = -1    
 
    # Compute position of the end effector
    P_org = np.array([[np.cos(theta1)*np.cos(theta2 + theta3)*0.3 + np.cos(theta1)*np.cos(theta2)*0.2 - config*np.sin(theta1)*0.078],
                      [np.sin(theta1)*np.cos(theta2 + theta3)*0.3 + np.sin(theta1)*np.cos(theta2)*0.2 + config*np.cos(theta1)*0.078],
                      [-config*np.sin(theta2 + theta3)*0.3 - config*np.sin(theta2)*0.2]])
    return P_org

def P0_2(theta1, theta2, theta3, Leg):
    """
    Postion for second Joint

    Args:
        theta1: joint angle in radians
        theta2: joint angle in radians
        theta3: joint angle in radians
        Leg: 'FL', 'FR', 'HL' or 'HR' to specify considered leg

    Returns:
        P_org: Position of the second joint in the base frame of the specified leg
    """
    # Input:
        # theta1, theta2, theta3: joint angles in radians
        # Leg: 'FL', 'FR', 'HL' or 'HR' to specify considered leg
    # Output:
        # P_org: Position of the second joint in the base frame of the specified leg

    # Specify config variable based on leg
    if Leg == 'FL' or Leg == 'HR':
        config = 1
    elif Leg == 'FR' or Leg == 'HL':
        config = -1

    # Compute position of the second joint
    P_org = np.array([[-config*np.sin(theta1)*0.078],
                      [ config*np.cos(theta1)*0.078],
                      [0]])
    return P_org

def P0_3(theta1, theta2, theta3, Leg):
    """
    Postion for Thrid Joint

    Args:
        theta1: joint angle in radians
        theta2: joint angle in radians
        theta3: joint angle in radians
        Leg: 'FL', 'FR', 'HL' or 'HR' to specify considered leg

    Returns:
        P_org: Position of the third joint in the base frame of the specified leg
    """
    # Specify config variable based on leg
    if Leg == 'FL' or Leg == 'HR':
        config = 1
    elif Leg == 'FR' or Leg == 'HL':
        config = -1

    # Compute position of the third joint
    P_org = np.array([[np.cos(theta1)*np.cos(theta2)*0.2 - config*np.sin(theta1)*0.078],
                      [np.sin(theta1)*np.cos(theta2)*0.2 + config*np.cos(theta1)*0.078],
                      [-config*np.sin(theta2)*0.2]])
    return P_org