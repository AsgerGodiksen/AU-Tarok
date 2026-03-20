# Constant Transforms from body frame to leg base frames
import numpy as np
from scipy.spatial.transform import Rotation

# Constant parameters used:
    # l_k = 0.7048 m (length of body in kinematic model)
    # w_k = 0.220 m (width of body in kinematic model)
# Note, often l_k/2 and w_k/2 are used in the transforms.


def TB_0(P_0, Leg):
    """ 
    From Constand_Transform.py
    
    Function to transform positions from leg base frames to body frame
    
    Args:
        P_0 (_type_): Position in the base frame of the specific leg 
        Leg (_type_): 'FL', 'FR', 'HL' or 'HR' to specify considered leg

    Returns:
        P_B: Position in the body frame of the robot
    """
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
    """
    From Constant_Transform.py
    
    Function to transform positions from body frame to leg base frames

    Args:
        P_B (_type_): Position in the body frame of the robot
        Leg (_type_): 'FL', 'FR', 'HL' or 'HR' to specify considered leg

    Returns:
        P_0: Position in the base frame of the specific leg
    """

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

# Define rotation from IMU frame to body frame using scipy Rotation class and constant rotation matrix
qB_IMU = Rotation.from_matrix(np.array([[ 0, 0, -1], 
                                        [-1, 0,  0], 
                                        [ 0, 1,  0]]))

# Define rotation from body frame to IMU frame using scipy Rotation class and constant rotation matrix
qIMU_B = Rotation.from_matrix(np.array([[ 0, 0, -1],
                                        [-1, 0,  0],
                                        [ 0, 1,  0]]).T)

# Function to rotate orientations from IMU frame to body frame using quaternions
def RB_IMU(q_IMU):
    """Function that Rotates Orientation From IMU Frame to BOdy Frame using Quaternions

    Args:
        q_IMU (_type_): IMU Quaternion

    Returns:
        q_B: Rotated Orientating from IMU frame to Body Frame
    """
    #print("Now Being Rotated")
    # Define rotation from input quaternion using scipy Rotation class
    q_IMU = Rotation.from_quat(q_IMU) # Quaternion in (x, y, z, w) format
    # Rotate orientation from IMU frame to body frame
    q_B = qB_IMU * q_IMU
    #print(q_B.as_quat())
    return q_B.as_quat()

# Function to rotate orientations from body frame to IMU frame using quaternions
def RIMU_B(q_B):
    # Input:
        # q_B: Quaternion in the body frame of the robot
    # Output:
        # q_IMU: Quaternion in the IMU frame

    # Define rotation from input quaternion using scipy Rotation class
    q_B = Rotation.from_quat(q_B) # Quaternion in (x, y, z, w) format

    # Rotate orientation from body frame to IMU frame
    q_IMU = qIMU_B * q_B
    return q_IMU.as_quat()







# Example usage of the functions to verify they work as intended
if __name__ == "__main__":
    # Test orientation transformation from IMU frame to body frame and back
    q_IMU = np.array([0.5, -0.5, 0.5, 0.5]) # Quaternion in IMU frame (x, y, z, w)
    q_B = RB_IMU(q_IMU) # Rotate to body frame
    q_IMU_check = RIMU_B(q_B.as_quat()) # Rotate back to IMU frame

    print("\nOriginal quaternion in IMU frame (q_IMU):")
    print(q_IMU)
    print("\nRotated quaternion in body frame (q_B):")
    print(q_B.as_quat()) # Print as (x, y, z, w) format
    print("\nRotated back quaternion in IMU frame (q_IMU_check):")
    print(q_IMU_check.as_quat()) # Print as (x, y, z, w) format

    check_variable = (q_IMU_check.as_quat() / q_IMU)
    print("\nCheck variable (should be close to [1, 1, 1, 1]):")
    print(check_variable)