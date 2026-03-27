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



def Generate_Bezier_Swing_Trajectory(c_k,Swing_Time_Scalar,Swing_Time_Steps):
    """
    Generate the Bézier swing-phase trajectory for one step cycle.

    The Frame are the Leg Base Frame
    Args:
        c_k       (np.ndarray, (n, 2)): Bézier control points.
        Swing_Time   (float): Duration of the swing phase [s].
        Swing_Time_Steps (int):   Number of discrete time steps for the swing phase.

    Returns:
        Swing_Time_Vector     (np.ndarray, shape (num_steps,)): Absolute time vector [s].
        Trajectory  (np.ndarray, shape (num_steps, 2)):
                        [:, 0] = forward position (Bézier frame z),
                        [:, 1] = height position  (Bézier frame x).
        Velocity    (np.ndarray, shape (num_steps, 2)): Real velocity [m/s].
    """
    # Time Vector
    Swing_Time_Vector = np.linspace(0,Swing_Time_Scalar,Swing_Time_Steps)
    Time_Norm = Swing_Time_Vector / Swing_Time_Scalar   # Normalised to [0, 1]
        
    # Initiating the Returns of the function
    Trajectory = np.zeros((Swing_Time_Steps, 2))
    Velocity   = np.zeros((Swing_Time_Steps, 2))
    
    # Calculating the Trajectory & Velocity for the Curve
    for i in range(Swing_Time_Steps):
        Trajectory[i,:] , Velocity[i,:] = Bezier_Curve(c_k, Time_Norm[i])
    # Convert from normalised to real time for the velocity
    Velocity = Velocity / Swing_Time_Scalar
    
    return Swing_Time_Vector, Trajectory , Velocity
    
    
def Generate_Bezier_Stand_Trajectory(Swing_Trajectory,Stand_Time_Scalar,Stand_Time_Steps):
    """
    Generate the linear stand-phase trajectory that follows a swing phase.

    The foot returns linearly (in the Backward direction) from the swing end-point
    back to the swing start-point, while holding a constant height.
    
    The Leg Base Frame

    Args:
        Swing_Trajectory (np.ndarray, shape (N, 2)): Output of Generate_Bezier_Swing_Trajectory.
        Stand_Time  (float): Duration of the stand phase [s].
        Stand_Time_Steps (int):  Number of discrete time steps for the stand phase.

    Returns:
        t_Stand     (np.ndarray, shape (num_steps,)): Time vector, offset to start
                        at T_swing (must be shifted by caller if needed).
        z_stand     (np.ndarray, shape (num_steps,)): Forward position [m].
        x_stand     (np.ndarray, shape (num_steps,)): Height position  [m] (constant).
        z_dot_stand (np.ndarray, shape (num_steps,)): Forward velocity [m/s] (constant).
        x_dot_stand (np.ndarray, shape (num_steps,)): Height velocity  [m/s] (zero).
    """
    # Extrating the start and end points of the Swing Trajectory
    z_start = Swing_Trajectory[-1, 0]   # Where swing ended
    z_end   = Swing_Trajectory[ 0, 0]   # Where swing started (return target)
    x_const = Swing_Trajectory[-1, 1]   # Height is constant during stand
    
    
    # ─────────────────────────────────────────────
    # OUTPUTS
    
    # Stand Coordinates for the end effectors
    Stand_Time_Vector = np.linspace(0,Stand_Time_Scalar,Stand_Time_Steps)
    z_Stand = np.linspace(z_start,z_end,Stand_Time_Steps)
    x_Stand = np.full(Stand_Time_Steps,x_const)     # Having a constant x values for the stand trajectory
    
    # Stand Velocities
    z_dot_Stand = np.full(Stand_Time_Steps, (z_end - z_start) / Stand_Time_Scalar)
    x_dot_Stand = np.zeros(Stand_Time_Steps)  

    return Stand_Time_Vector, z_Stand, x_Stand, z_dot_Stand, x_dot_Stand
    


def Assemble_Bezier_Trajectory(
                                Swing_Trajectory,
                                Swing_Velocity,
                                x_Stand,
                                z_Stand,
                                x_dot_Stand,
                                z_dot_Stand,
                                Swing_Time_Scalar,
                                Stand_Time_Scalar,
                                Swing_Time_Steps,
                                Stand_Time_Steps):
    """
    Combine swing + stand into one full gait cycle in the leg base frame.

    The Bézier curve lives in a 2-D (forward, height) plane.  This function
    maps it into the 3-D leg base frame: (x=height, y=lateral, z=forward).

    Args:
        Swing_Trajectory  (np.ndarray, shape (N_sw, 2)): Swing trajectory.
        Swing_Velocity   (np.ndarray, shape (N_sw, 2)): Swing velocity.
        z_Stand     (np.ndarray, shape (N_st,))  : Stand forward positions.
        x_Stand     (np.ndarray, shape (N_st,))  : Stand heights.
        z_dot_Stand (np.ndarray, shape (N_st,))  : Stand forward velocities.
        x_dot_Stand (np.ndarray, shape (N_st,))  : Stand height velocities.
        Swing_Time_Scalar     (float): Swing duration [s].
        Stand_Time_Scalar     (float): Stand duration [s].
        Swing_Time_Steps      (float): Number of steps
        Stand_Time_Steps      (float): Number of steps

    Returns:
        Position_Bezier  (np.ndarray, shape (3, N_total)): Position  trajectory [m].
        Velocity_Bezier  (np.ndarray, shape (3, N_total)): Velocity trajectory [m/s].
        Time_Bezier  (np.ndarray, shape (N_total,))  : Time vector [s].
    
    """
    y_offset=0.078
    Time_Steps_Bezier = Swing_Time_Steps + Stand_Time_Steps
    # Generating arrays for the cooridantes from both Swing and Stand Phase
    x = np.concatenate((Swing_Trajectory[:,1], x_Stand))
    z = np.concatenate((Swing_Trajectory[:,0], z_Stand))
    y = np.full(Time_Steps_Bezier, y_offset)
    
    # Number of steps
    
    
    # Generating array for Velocity Componets
    x_dot = np.concatenate((Swing_Velocity[:,1], x_dot_Stand))
    z_dot = np.concatenate((Swing_Velocity[:,0],z_dot_Stand))
    y_dot = np.zeros(Time_Steps_Bezier)
    
    
    # Generating Array for the Time vector
    Swing_Time_Vector = np.linspace(0,Swing_Time_Scalar,Swing_Time_Steps)
    Stand_Time_Vector = np.linspace(Swing_Time_Scalar,Swing_Time_Scalar + Stand_Time_Scalar,Stand_Time_Steps)
    
    # Exports
    Position_Bezier = np.vstack((x,y,z))
    Velocity_Bezier = np.vstack((x_dot, y_dot, z_dot))
    Time_Bezier = np.concatenate((Swing_Time_Vector,Stand_Time_Vector))
     
    return Position_Bezier, Velocity_Bezier, Time_Bezier
    

def Building_Bezier_Trajectories(
                                    Swing_Time_Scalar,
                                    Stand_Time_Scalar,
                                    Swing_Time_Steps,
                                    Stand_Time_Steps,
                                    PHASE_OFFSET,
                                    leg):
    """_summary_

    Args:
        Swing_Time_Scalar (_type_): _description_
        Stand_Time_Scalar (_type_): _description_
        Swing_Time_Steps (_type_): _description_
        Stand_Time_Steps (_type_): _description_
        PHASE_OFFSET (_type_): _description_
        leg (_type_): _description_

    Returns:
        _type_: _description_
    """
    # Finding the points for the Bezier Curve
    c_k = Bezier_Control_Points()

    # Generating the Swing Phase for the Bezier Curve
    Swing_Time_Vector, Swing_Trajectory , Swing_Velocity = Generate_Bezier_Swing_Trajectory(c_k,Swing_Time_Scalar,Swing_Time_Steps)
    
    # Generating the Stand Phase
    Stand_Time_Vector, z_Stand, x_Stand, z_dot_Stand, x_dot_Stand = Generate_Bezier_Stand_Trajectory(Swing_Trajectory,Stand_Time_Scalar,Stand_Time_Steps)

    # Assempling the Swing and Stand Phase
    Position_Bezier, Velocity_Bezier, Time_Bezier = Assemble_Bezier_Trajectory(
                                    Swing_Trajectory,
                                    Swing_Velocity,
                                    x_Stand,
                                    z_Stand,
                                    x_dot_Stand,
                                    z_dot_Stand,
                                    Swing_Time_Scalar,
                                    Stand_Time_Scalar,
                                    Swing_Time_Steps,
                                    Stand_Time_Steps)
    
    # Implement the shift in the legs so each legs are different
    N_total = Position_Bezier.shape[1]
    shift   = int(round(PHASE_OFFSET[leg] * N_total))
    
    Position_shifted = np.roll(Position_Bezier, shift, axis=1)
    Velocity_shifted = np.roll(Velocity_Bezier, shift, axis=1)
    
    return Position_shifted, Velocity_shifted, Time_Bezier