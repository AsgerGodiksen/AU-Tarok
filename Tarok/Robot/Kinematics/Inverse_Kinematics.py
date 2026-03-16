# Inverse kinematics of quadruped robot
import numpy as np

# Constant parameters used:
    # L1 = 0.078 m (length of the first leg segment)
    # L2 = 0.20 m (length of the second leg segment)
    # L3 = 0.30 m (length of the third leg segment)

def Inverse_Kinematics(P_org, Leg):
    # Input:
        # P_org: Position of the end effector (foot) in the base frame
        # Leg: 'FL', 'FR', 'HL' or 'HR' to specify considered leg
    # Output:
        # theta1, theta2, theta3: joint angles in radians

    # Compute joint angles using inverse kinematics
    x = P_org[0, 0]  # Extract scalar value from numpy array
    y = P_org[1, 0]  # Extract scalar value from numpy array
    z = P_org[2, 0]  # Extract scalar value from numpy array

    # Specify config varaible based on leg
    if Leg == 'FL' or Leg == 'HR':
        config = 1
    elif Leg == 'FR' or Leg == 'HL':
        config = -1
    
    # Check workspace
    lower_first_eq = 0.078**2
    distance_first_eq = x**2 + y**2
    lower_second_eq = abs(0.2 - 0.3)
    upper_second_eq = 0.2 + 0.3
    distance_second_eq = np.sqrt(x**2 + y**2 + z**2 - 0.078**2)
    if distance_first_eq < lower_first_eq or distance_second_eq < lower_second_eq or distance_second_eq > upper_second_eq:
        raise ValueError("The desired position is outside the reachable workspace of the specified leg.")

    # Check cartesian space limits (Only checking end-effector for obstruction with bounding box of body (including addition of physical foot size))
    if config == 1: # FL and HR legs
        if x > -0.101171 and x < 0.101171 and y > -0.309 and y < 0.089 and z > -0.6594 and z < -0.0454:
            raise ValueError("The desired position is inside the bounding box of the body.")
    elif config == -1: # FR and HL legs
        if x > -0.101171 and x < 0.101171 and y > -0.089 and y < 0.309 and z > -0.6594 and z < -0.0454:
            raise ValueError("The desired position is inside the bounding box of the body.")

    # Determine horizontal reach in plane of the leg
    H = np.sqrt(x**2 + y**2 - 0.078**2)

    # Compute theta1
    theta1 = np.arctan2(config*y, x) - np.arctan2(0.078, H)

    # Determine constants needed
    rsqr = H**2 + z**2
    costheta3 = (rsqr - 0.2**2 - 0.3**2) / (2 * 0.2 * 0.3)

    # Compute theta3
    # (Note: Two possible solutions for theta3, +np.sqrt and -np.sqrt for elbow in/out, depending on leg as well)
    theta3 = np.arctan2(-config*np.sqrt(1 - costheta3**2), costheta3)
    
    # Determine constants needed for theta2
    beta = np.arctan2(0.3 * np.sin(theta3), 0.2 + 0.3 * np.cos(theta3))
    gamma = np.arctan2(-z, H)

    # Compute theta2
    theta2 = gamma - beta
   
    # Check joint space limits
    if Leg == 'FL' or Leg == 'HR':
        if theta3 < -1.74532925 or theta3 > 0: # -100 to 0 degrees in radians is allowed for these legs
            raise ValueError("The desired position is outside the joint space limits of the specified leg.")
    elif Leg == 'FR' or Leg == 'HL':
        if theta3 < 0 or theta3 > 1.74532925: # 0 to 100 degrees in radians is allowed for these legs
            raise ValueError("The desired position is outside the joint space limits of the specified leg.")
    return theta1, theta2, theta3