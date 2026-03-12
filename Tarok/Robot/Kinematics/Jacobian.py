# Jacobian of quadruped robot
import numpy as np

# Constant parameters used:
    # L1 = 0.078 m (length of the first leg segment)
    # L2 = 0.20 m (length of the second leg segment)
    # L3 = 0.30 m (length of the third leg segment)

def Jacobian(theta1,theta2,theta3, Leg):
    # Input:
        # theta1, theta2, theta3: joint angles in radians
        # Leg: 'FL', 'FR', 'HL' or 'HR' to specify considered leg
    # Output:
        # J: Jacobian matrix of the end effector (foot)   

    # Specify config varaible based on leg
    if Leg == 'FL' or Leg == 'HR':
        config = 1
    elif Leg == 'FR' or Leg == 'HL':
        config = -1

    # Compute Jacobian matrix
    J = np.array([[-np.sin(theta1)*np.cos(theta2 + theta3)*0.3 - np.sin(theta1)*np.cos(theta2)*0.2 - config*np.cos(theta1)*0.078, 
                   -np.cos(theta1)*np.sin(theta2 + theta3)*0.3 - np.cos(theta1)*np.sin(theta2)*0.2, 
                   -np.cos(theta1)*np.sin(theta2 + theta3)*0.3],
                  [ np.cos(theta1)*np.cos(theta2 + theta3)*0.3 + np.cos(theta1)*np.cos(theta2)*0.2 - config*np.sin(theta1)*0.078, 
                   -np.sin(theta1)*np.sin(theta2 + theta3)*0.3 - np.sin(theta1)*np.sin(theta2)*0.2, 
                   -np.sin(theta1)*np.sin(theta2 + theta3)*0.3],
                  [0, 
                   -config*np.cos(theta2 + theta3)*0.3 - config*np.cos(theta2)*0.2, 
                   -config*np.cos(theta2 + theta3)*0.3]])
    return J

