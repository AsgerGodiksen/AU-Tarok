# Function for rotation matrix from quaternion (eq. B.14)
import numpy as np
from numpy.typing import NDArray

# Equations from Jain Comp Dyn textbook

# Function for skew-symmetric matrix (eq. 1.9)
def skew(v: NDArray) -> NDArray:
    v = v.flatten() # Ensure v is a 1D array
    return np.array([[0, -v[2], v[1]],
                    [v[2], 0, -v[0]],
                    [-v[1], v[0], 0]])

def quat_to_rot_matrix(quaternion: NDArray) -> NDArray:
    # Extract quaternion components (q0 is the last component)
    q = quaternion[:3]
    q0 = quaternion[3]

    # Determine skew-symmetric matrix of vector part
    q_tilt = skew(q)

    # Determine rotation matrix
    rot = np.eye(3) + 2 * (q0 * np.eye(3) + q_tilt) @ q_tilt
    return rot
