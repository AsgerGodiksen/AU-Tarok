import numpy as np
from scipy.special import comb

from Robot.Kinematics import Inverse_Kinematics

def Bezier_Curve(c_k, t):
    """
    Bezier Curve - Generated in Bézier Frame
    Generate single point and velocity along a Bezier curve defined by control points c_k at time t
    Input time t must be normalized between 0 and 1
    n Control points are assumed
    Vectorized version
    Args:
        c_k (np array (n, 2)):  Control points
        t (_type_):  Time instance at which to evaluate the Bézier curve (0 <= t <= 1)

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
    Function that gives the the Bézier Curve Control points in Bézier frame
    
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
    
    # Scaling and Offsetting the Control points
    c_kX = Scaling_Factor * X
    c_kY = Offset + Scaling_Factor * Y
    
    # Assebling the Control points into one array
    c_k = np.column_stack((c_kX, c_kY))
    
    return c_k



def Generate_Bezier_Swing_Trajectory(c_k,Swing_Time_Scalar,Swing_Time_Steps):
    """
    Generate the Bézier swing-phase trajectory for one step cycle. 
    Happens in Bézier Frame

    Args:
        c_k    (np.ndarray, (n, 2)): Bézier control points.
        Swing_Time          (float): Duration of the swing phase [s].
        Swing_Time_Steps      (int): Number of discrete time steps for the swing phase.

    Returns:
        Swing_Time_Vector     (np.ndarray, shape (num_steps,)): Absolute time vector [s].
        Swing_Trajectory  (np.ndarray, shape (num_steps, 2)):
                        [:, 0] = forward position (Bézier frame x),
                        [:, 1] = height position  (Bézier frame y).
        Swing_Velocity    (np.ndarray, shape (num_steps, 2)): Real velocity [m/s].
    """
    # Time Vector
    Swing_Time_Vector = np.linspace(0,Swing_Time_Scalar,Swing_Time_Steps)
    Time_Norm = Swing_Time_Vector / Swing_Time_Scalar   # Normalised to [0, 1]
        
    # Initiating the Returns of the function
    Swing_Trajectory = np.zeros((Swing_Time_Steps, 2))
    Swing_Velocity   = np.zeros((Swing_Time_Steps, 2))
    
    # Calculating the Trajectory & Velocity for the Curve
    for i in range(Swing_Time_Steps):
        Swing_Trajectory[i,:] , Swing_Velocity[i,:] = Bezier_Curve(c_k, Time_Norm[i])
        
    # Convert from normalised to real time for the velocity
    Swing_Velocity = Swing_Velocity / Swing_Time_Scalar
    
    return Swing_Time_Vector, Swing_Trajectory , Swing_Velocity
    
    
def Generate_Bezier_Stand_Trajectory(Swing_Trajectory,Stand_Time_Scalar,Stand_Time_Steps):
    """
    Generate the linear stand-phase trajectory that follows a swing phase.

    The foot returns linearly (in the Backward direction) from the swing end-point
    back to the swing start-point, while holding a constant height.
    
    Happens in Bézier Frame

    Args:
        Swing_Trajectory (np.ndarray, shape (N, 2)): Output of Generate_Bezier_Swing_Trajectory.
        Stand_Time  (float): Duration of the stand phase [s].
        Stand_Time_Steps (int):  Number of discrete time steps for the stand phase.

    Returns:
        Stand_Time_Vector     (np.ndarray, shape (num_steps,)): Time vector, offset to start
                        at T_swing (must be shifted by caller if needed).
        Stand_Trajectory  (np.ndarray, shape (num_steps, 2)):
                        [:, 0] = forward position (Bézier frame x),
                        [:, 1] = height position  (Bézier frame y).
        Stand_Velocity    (np.ndarray, shape (num_steps, 2)):
                        [:, 0] = forward velocity (Bézier frame x),
                        [:, 1] = height velocity  (Bézier frame y).
    """
    # Extrating the start and end points of the Swing Trajectory
    x_start = Swing_Trajectory[-1, 0]   # Where swing ended
    x_end   = Swing_Trajectory[ 0, 0]   # Where swing started (return target)
    y_const = Swing_Trajectory[-1, 1]   # Height is constant during stand
    
    # ─────────────────────────────────────────────
    # OUTPUTS
    
    # Stand Coordinates for the end effectors
    Stand_Time_Vector = np.linspace(0,Stand_Time_Scalar,Stand_Time_Steps)
    x_Stand = np.linspace(x_start,x_end,Stand_Time_Steps)
    y_Stand = np.full(Stand_Time_Steps,y_const)     # Having a constant y values for the stand trajectory
    # Assembling the Stand Trajectory
    Stand_Trajectory = np.column_stack((x_Stand, y_Stand))
    
    # Stand Velocities
    x_dot_Stand = np.full(Stand_Time_Steps, (x_end - x_start) / Stand_Time_Scalar)
    y_dot_Stand = np.zeros(Stand_Time_Steps)  
    # Assembling the Stand Velocity
    Stand_Velocity = np.column_stack((x_dot_Stand, y_dot_Stand))

    return Stand_Time_Vector,Stand_Trajectory, Stand_Velocity
    

def Assemble_Bezier_Trajectory(
                                Swing_Trajectory,
                                Swing_Velocity,
                                Stand_Trajectory,
                                Stand_Velocity,
                                Swing_Time_Scalar,
                                Stand_Time_Scalar,
                                Swing_Time_Steps,
                                Stand_Time_Steps
                                ):
    """
    Combine swing + stand into one full gait cycle
    Inputs are in Bezier Frame - Output in BODY FRAME

    The Bézier curve lives in a 2-D (forward, height) plane.  This function
    maps it into the 3-D

    Args:
        Swing_Trajectory  (np.ndarray, shape (N_sw, 2)): Swing trajectory.
        Swing_Velocity   (np.ndarray, shape (N_sw, 2)):  Swing velocity.
        Stand_Trajectory  (np.ndarray, shape (N_st, 2)): Stand trajectory.
        Stand_Velocity   (np.ndarray, shape (N_st, 2)):  Stand velocity.
        Swing_Time_Scalar     (float): Swing duration [s].
        Stand_Time_Scalar     (float): Stand duration [s].
        Swing_Time_Steps      (float): Number of steps
        Stand_Time_Steps      (float): Number of steps

    Returns:
        Position_Bezier  (np.ndarray, shape (3, N_total)): Position  trajectory [m].
        Velocity_Bezier  (np.ndarray, shape (3, N_total)): Velocity trajectory [m/s].
        Time_Bezier  (np.ndarray, shape (N_total,))  : Time vector [s].
    """
    
    Time_Steps_Bezier = Swing_Time_Steps + Stand_Time_Steps

    # Generating arrays Trajectory
    x = np.concatenate((Swing_Trajectory[:,0], Stand_Trajectory[:,0]))
    y = np.zeros(Time_Steps_Bezier)
    z = - np.concatenate((Swing_Trajectory[:,1], Stand_Trajectory[:,1]))
    
    # Generating Arrays for the Velocity
    x_dot = np.concatenate((Swing_Velocity[:,0], Stand_Velocity[:,0]))
    y_dot = np.zeros(Time_Steps_Bezier)    
    z_dot = - np.concatenate((Swing_Velocity[:,1], Stand_Velocity[:,1]))
    
    # Generating Array for the Time vector
    Swing_Time_Vector = np.linspace(0,Swing_Time_Scalar,Swing_Time_Steps)
    Stand_Time_Vector = np.linspace(Swing_Time_Scalar,Swing_Time_Scalar + Stand_Time_Scalar,Stand_Time_Steps)
    
    # Exports
    Position_Body_Bezier = np.vstack((x,y,z))
    Velocity_Body_Bezier = np.vstack((x_dot, y_dot, z_dot))
    Time_Bezier = np.concatenate((Swing_Time_Vector,Stand_Time_Vector))
     
    return Position_Body_Bezier, Velocity_Body_Bezier, Time_Bezier
    

def Building_Bezier_Trajectories(
                                Swing_Time_Scalar,
                                Stand_Time_Scalar,
                                Swing_Time_Steps,
                                Stand_Time_Steps
                                ):
    """
    Building the Bezier Trajectory for one leg in the Body Frame
    
    Args:
        Swing_Time_Scalar (float): The Swing Time
        Stand_Time_Scalar (float): The Stand Time
        Swing_Time_Steps (int): Swing Time Steps
        Stand_Time_Steps (int): Stand Time Steps

    Returns:
        Position_Body_Bezier (np.ndarray):   The Bezier Curve in the Body Frame
        Velocity_Body_Bezier (np.ndarray):   The Velocity of the Bezier Curve in the Body Frame
        Time_Bezier (np.ndarray):       The Time Vector for the Bezier Curve
    """
    # Finding the points for the Bezier Curve
    c_k = Bezier_Control_Points()

    # Generating the Swing Phase for the Bezier Curve
    Swing_Time_Vector, Swing_Trajectory , Swing_Velocity = Generate_Bezier_Swing_Trajectory(c_k,Swing_Time_Scalar,Swing_Time_Steps)
    
    # Generating the Stand Phase
    Stand_Time_Vector,Stand_Trajectory, Stand_Velocity = Generate_Bezier_Stand_Trajectory(Swing_Trajectory,Stand_Time_Scalar,Stand_Time_Steps)

    # Assembling the Swing and Stand Phase
    Position_Body_Bezier, Velocity_Body_Bezier, Time_Bezier = Assemble_Bezier_Trajectory(
                                Swing_Trajectory,
                                Swing_Velocity,
                                Stand_Trajectory,
                                Stand_Velocity,
                                Swing_Time_Scalar,
                                Stand_Time_Scalar,
                                Swing_Time_Steps,
                                Stand_Time_Steps
    )
    
    return Position_Body_Bezier, Velocity_Body_Bezier, Time_Bezier


def Apply_Phase_Offset(Trajectory, Velocity, Phase_Offset):
    N_total = Trajectory.shape[1]
    shift   = int(round(Phase_Offset * N_total))
    return np.roll(Trajectory, shift, axis=1), np.roll(Velocity, shift, axis=1)


def Compute_Joint_Angles(Trajectory, leg):
    """
    Run IK over every time step for the Trajectory
    
    Args:
        Trajectory: The Position trajectory of the End Effector
        leg:   The Name of the desired leg 
    
    Returns
        Theta (N, 3): The Angles of the three actuators
        
    """
    Time_Steps = Trajectory.shape[1]
    Theta = np.zeros((Time_Steps, 3))
    for i in range(Time_Steps):
        
        # Extrating the End Effetor Position
        End_Effector_Postition = Trajectory[:, i].reshape(3, 1)
        
        # Using Inverse Kinematics to find the corresponding Angles for the End Effetor Position
        Theta[i] = Inverse_Kinematics(End_Effector_Postition, leg)
        
        
        
    return Theta