import numpy as np
from scipy.special import comb

def Bezier_Curve(c_k, t):
    """Bezier Curve
    Generate single point and velocity along a Bezier curve defined by control points c_k at time t
    Input time t must be normalized between 0 and 1
    n Control points are assumed
    Vectorized version
    Args:
        c_k (np array (n, 2)):  Control points
        t (_type_):  Time instance at which to evaluate the Bezier curve (0 <= t <= 1)

    Returns:
        Position (np Array (2,)): Containing the trajectory point at time t (cartesian coordinates of end-effector)
                (pos[0] = forward position, pos[1] = downward position)
                
        Velocity (np Array (2,)): containing the instantaneous velocity at time t (cartesian coordinates of end-effector)
                Note: that the velocity iBezier_2 import Bezier_s corresponding to normalized time, to get real velocity multiply with 1/T_swing where T_swing is 
    """

    # Bernstein Polynomial Degree
    n = len(c_k) - 1
    
    # Indices for the Control Points
    k = np.arange(0, n + 1)
    
    # Bernstein basis function for position (degree n)
    B = comb(n, k) * (t ** k) * ((1 - t) ** (n - k)) 
    
    # Bernstein basis function for velocity (degree n-1)
    Bv = comb(n - 1, k[:-1]) * (t ** k[:-1]) * ((1 - t) ** (n - 1 - k[:-1])) 
    
    # Bezier point at time t - weighted sum of control points
    Position = np.dot(B, c_k) 
    
    # Bezier velocity at time t - weighted sum of control point differences
    Velocity = n * np.dot(Bv, np.diff(c_k, axis=0)) 
    
    return Position, Velocity 


def Bezier_Control_Points():
    """
    Function that gives the the Bezier Curve Control points
    
    Returns:
        c_kX Control Points in x
        
        c_kY Control Points in y
    """
    # Listed Values is taken from 
    # High speed trot-running: Implementation of a hierarchical controller using proprioceptive impedance control on the MIT Cheetah
    # By Dong Jin Hyun, et al. 2014
    X = np.array([-0.200, -0.2805, -0.300, -0.300, -0.300, 0, 0, 0, 0.3032, 0.3032, 0.2826, 0.200]) # [m]
    Y = np.array([0.500,  0.500,   0.3611, 0.3611, 0.3611, 0.3611, 0.3611, 0.3214, 0.3214, 0.3214, 0.500, 0.500]) # [m]
    Scaling_Factor = 0.6
    Offset = 0.16
    
    c_kX = Scaling_Factor * X
    c_kY = Offset + Scaling_Factor * Y
    
    c_k = np.column_stack((c_kX, c_kY))
    
    return c_k

def Bezier_Time_Parameters(Swing_Time,Stand_Time):
    # Define time parameters
    Swing_time = 3.0  # seconds
    Stand_time = 3*Swing_time  # seconds
    Num_time_steps = 300 # 100 Hz (manually set)
    t_Swing = np.linspace(0, Swing_time, Num_time_steps)
    t_Stand = np.linspace(Swing_time, Swing_time + Stand_time, 3 * Num_time_steps)
    t = np.concatenate((t_Swing, t_Stand))
    dt = t[1] - t[0]  # Time step duration (constant)
    
    t_Bezier = t_Swing / Swing_time  # Normalize time to [0, 1] for Bezier curve function
    
    Trajectory_Swing = np.zeros((Num_time_steps, 2))  # Initialize trajectory array
    Velocity_Swing = np.zeros((Num_time_steps, 2))    # Initialize velocity array
    
    return t_Bezier, Trajectory_Swing, Velocity_Swing
    
    
