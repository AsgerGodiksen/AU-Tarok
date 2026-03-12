# Constant Transforms from body frame to leg base frames
import numpy as np

# Constant parameters used:
    # l_k = 0.7048 m (length of body in kinematic model)
    # w_k = 0.220 m (width of body in kinematic model)
# Note, often l_k/2 and w_k/2 are used in the transforms.

# Function to transform positions from leg base frames to body frame
def TB_0(P_0, Leg):
    # Input:
        # P_0: Position in the base frame of the specific leg
        # Leg: 'FL', 'FR', 'HL' or 'HR' to specify considered leg
    # Output:
        # P_B: Position in the body frame of the robot

    # Specify config varaible based on leg
    if   Leg == 'FL':
        configA = 1
        configB = 1
    elif Leg == 'FR':
        configA = 1
        configB = -1
    elif Leg == 'HL':
        configA = -1
        configB = 1
    elif Leg == 'HR':
        configA = -1
        configB = -1

    # Define transformation matrix
    T = np.array([[ 0, 0,         configA*1, configA*0.3524],
                  [ 0, configA*1, 0,         configB*0.110 ],
                  [-1, 0,         0,         0             ],
                  [ 0, 0,         0,         1             ]])
    
    # Transform position from leg base frame to body frame
    P_B = T @ np.append(P_0.flatten(), 1)
    P_B = P_B[:3].reshape((3, 1))  # Extract the position part and reshape to (3, 1)
    return P_B

# Function to transform positions from body frame to leg base frames
def T0_B(P_B, Leg):
    # Input:
        # P_B: Position in the body frame of the robot
        # Leg: 'FL', 'FR', 'HL' or 'HR' to specify considered leg
    # Output:
        # P_0: Position in the base frame of the specific leg

    # Specify config varaible based on leg
    if   Leg == 'FL':
        configA = 1
        configB = -1
    elif Leg == 'FR':
        configA = 1
        configB = 1
    elif Leg == 'HL':
        configA = -1
        configB = 1
    elif Leg == 'HR':
        configA = -1
        configB = -1

    # Define transformation matrix (inverse of TB_0)
    T_inv = np.array([[ 0,         0,         -1,  0           ],
                      [ 0,         configA*1,  0,  configB*0.11],
                      [ configA*1, 0,          0, -0.3524      ],
                      [ 0,         0,          0,  1           ]])

    # Transform position from body frame to leg base frame
    P_0 = T_inv @ np.append(P_B.flatten(), 1)
    P_0 = P_0[:3].reshape((3, 1))  # Extract the position part and reshape to (3, 1)
    return P_0

# Function to rotate velocities from leg base frames to body frame
def RB_0(V_0, Leg):
    # Input:
        # V_0: Velocity in the base frame of the specific leg
        # Leg: 'FL', 'FR', 'HL' or 'HR' to specify considered leg
    # Output:
        # V_B: Velocity in the body frame of the robot

    # Specify config varaible based on leg
    if   Leg == 'FL' or Leg == 'FR':
        config = 1
    elif Leg == 'HL' or Leg == 'HR':
        config = -1

    # Define rotation matrix
    R = np.array([[ 0, 0,         config*1],
                  [ 0, config*1,  0       ],
                  [-1, 0,         0       ]])
    
    # Rotate velocity from leg base frame to body frame
    V_B = R @ V_0
    return V_B

# Function to rotate velocities from body frame to leg base frames
def R0_B(V_B, Leg):
    # Input:
        # V_B: Velocity in the body frame of the robot
        # Leg: 'FL', 'FR', 'HL' or 'HR' to specify considered leg
    # Output:
        # V_0: Velocity in the base frame of the specific leg

    # Specify config varaible based on leg
    if   Leg == 'FL' or Leg == 'FR':
        config = 1
    elif Leg == 'HL' or Leg == 'HR':
        config = -1

    # Define transformation matrix (inverse of RB_0)
    R_inv = np.array([[0,         0,         -1],
                      [0,         config*1,   0],
                      [config*1,  0,          0]])

    # Rotate velocity from body frame to leg base frame
    V_0 = R_inv @ V_B
    return V_0