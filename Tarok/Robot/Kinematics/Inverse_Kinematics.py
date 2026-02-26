# In this file the inverse kinematics for the legs are defined

import numpy as np


# Taken from Mikkels codes for the R&D project
# NOTE: HAVE TO BE CHNAGED  
def FL_Inverse_Kinematics(L1,L2,L3,P_org):
    # Input:
        # L1, L2, L3: lengths of the leg segments
        # P_org: Position of the end effector (foot) in the base frame of Left leg
    # Output:
        # theta1, theta2, theta3: joint angles in radians

    # Compute joint angles using inverse kinematics
    x = P_org[0, 0]  # Extract scalar value from numpy array
    y = P_org[1, 0]  # Extract scalar value from numpy array
    z = P_org[2, 0]  # Extract scalar value from numpy array

    # Check workspace
    lower_bound = abs(L2 - L3)
    upper_bound = L2 + L3
    distance = np.sqrt(x**2 + y**2 + z**2 - L1**2)
    if distance < lower_bound or distance > upper_bound:
        raise ValueError("The desired position is outside the reachable workspace of the leg.")

    # Determine horizontal reach in plane of the leg
    H = np.sqrt(x**2 + y**2 - L1**2)

    # Compute theta1
    theta1 = np.arctan2(y, x) - np.arctan2(L1, H)

    # Determine constants needed
    rsqr = H**2 + z**2
    costheta3 = (rsqr - L2**2 - L3**2) / (2 * L2 * L3)

    # Compute theta3
    # (Note: Two possible solutions for theta3, +np.sqrt for elbow out, -np.sqrt for elbow in)
    theta3 = np.arctan2(-np.sqrt(1 - costheta3**2), costheta3)
    
    # Determine constants needed for theta2
    beta = np.arctan2(L3 * np.sin(theta3), L2 + L3 * np.cos(theta3))
    gamma = np.arctan2(-z, H)

    # Compute theta2
    theta2 = gamma - beta
   
    return theta1, theta2, theta3
