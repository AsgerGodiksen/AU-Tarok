import numpy as np
from scipy.special import comb
from Robot.Kinematics import Inverse_Kinematics
from Robot.Kinematics.Interpolation import cos_interp, cos_interp_dot

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
    # Listed Values are taken from 
    # High speed trot-running: Implementation of a hierarchical controller using proprioceptive impedance control on the MIT Cheetah
    # By Dong Jin Hyun, et al. 2014
    X = np.array([-0.200, -0.2805, -0.300, -0.300, -0.300, 0, 0, 0, 0.3032, 0.3032, 0.2826, 0.200]) # [m]
    Y = np.array([0.500,  0.500,   0.3611, 0.3611, 0.3611, 0.3611, 0.3611, 0.3214, 0.3214, 0.3214, 0.500, 0.500]) # [m]
    Scaling_Factor_X = 0.5
    Scaling_Factor_Y = 0.4
    Offset = 0.24
    # Note that the height of stand to swing phase transition is at H = Scaling_Factor_Y * 0.500 + Offset
    # For example, Scaling_factor_Y = 0.6 and Offset = 0.16 gives transition at H = 0.46 m
    # This should always be below 0.48 m 
    
    # IMPORTANT NOTE: IF OFFSET + SCALING_FACTOR_Y * Y[0] IS CHANGED FROM H=44 cm -> 
    # MANUALLY CHANGE IN Building_Stand_To_Walk_Transition_Trajectory

    # Scaling and Offsetting the Control points
    c_kX = Scaling_Factor_X * X
    c_kY = Offset + Scaling_Factor_Y * Y
    
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

def Generate_Bezier_Stand_Trajectory_Modified(Swing_Trajectory,Stand_Time_Scalar,Stand_Time_Steps,Leg):
    """
    Generate the modified stand-phase trajectory that follows a swing phase.

    The foot returns linearly (in the Backward direction) from the swing end-point
    back to the swing start-point, while holding a constant height.
    
    Happens in Bézier Frame

    Args:
        Swing_Trajectory (np.ndarray, shape (N, 2)): Output of Generate_Bezier_Swing_Trajectory.
        Stand_Time  (float): Duration of the stand phase [s].
        Stand_Time_Steps (int):  Number of discrete time steps for the stand phase.
        Leg (str): The leg for which to generate the trajectory, either "FL", "FR", "HL" or "HR" - Note that FL and HL are equal, and FR and HR are equal.
        
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
    y_normal = Swing_Trajectory[-1, 1]   # Normal height for stand phase

    # Computing height for "stretch" and "fold" cases
    y_stretch = y_normal + 0.01  # Example: 1 cm higher than normal
    y_fold = y_normal - 0.01     # Example: 1 cm lower than normal

    # Time variables
    t = np.linspace(0, Stand_Time_Scalar, Stand_Time_Steps) # Time vector for the total stand phase
    Stand_Time_Vector = t # Output variable
    one_segment_time = Stand_Time_Scalar / 3  # Time duration for each segment of the stand phase

    # Segment boundaries for the stand phase trajectory
    conditions = [(t >= 0) & (t < 0.4*one_segment_time),                        # "Bezier" -> "Stretch" or "Fold"
                  (t >= 0.4*one_segment_time) & (t < 0.8*one_segment_time),     # "Stretch" or "Fold"
                  (t >= 0.8*one_segment_time) & (t < 1.2*one_segment_time),     # "Stretch" or "Fold" -> "Normal"
                  (t >= 1.2*one_segment_time) & (t < 1.8*one_segment_time),     # "Normal"
                  (t >= 1.8*one_segment_time) & (t < 2.2*one_segment_time),     # "Normal" -> "Stretch" or "Fold"
                  (t >= 2.2*one_segment_time) & (t < 2.6*one_segment_time),     # "Stretch" or "Fold"
                  (t >= 2.6*one_segment_time) & (t <= 3*one_segment_time)]      # "Stretch" or "Fold" -> "Bezier"
    
    # Stand coordinates for the end effector
    x_Stand = np.linspace(x_start,x_end,Stand_Time_Steps)
    if Leg == "FL" or Leg == "HL":
        # Sequence: "Bezier" -> "Stretch" -> "Normal" -> "Fold"
        y_Stand = np.piecewise(t, conditions, [lambda t: cos_interp(t, y_normal, y_stretch, 0, 0.4*one_segment_time),
                                               lambda t: y_stretch*np.ones_like(t),
                                               lambda t: cos_interp(t, y_stretch, y_normal, 0.8*one_segment_time, 1.2*one_segment_time),
                                               lambda t: y_normal*np.ones_like(t),
                                               lambda t: cos_interp(t, y_normal, y_fold, 1.8*one_segment_time, 2.2*one_segment_time),
                                               lambda t: y_fold*np.ones_like(t),
                                               lambda t: cos_interp(t, y_fold, y_normal, 2.6*one_segment_time, 3*one_segment_time)])  
    elif Leg == "FR" or Leg == "HR":
        # Sequence: "Bezier" -> "Fold" -> "Normal" -> "Stretch"
        y_Stand = np.piecewise(t, conditions, [lambda t: cos_interp(t, y_normal, y_fold, 0, 0.4*one_segment_time),
                                               lambda t: y_fold*np.ones_like(t),
                                               lambda t: cos_interp(t, y_fold, y_normal, 0.8*one_segment_time, 1.2*one_segment_time),
                                               lambda t: y_normal*np.ones_like(t),
                                               lambda t: cos_interp(t, y_normal, y_stretch, 1.8*one_segment_time, 2.2*one_segment_time),
                                               lambda t: y_stretch*np.ones_like(t),
                                               lambda t: cos_interp(t, y_stretch, y_normal, 2.6*one_segment_time, 3*one_segment_time)])  
    # Assembling the Stand Trajectory
    Stand_Trajectory = np.column_stack((x_Stand, y_Stand))

    # Stand Velocities
    x_dot_Stand = np.full(Stand_Time_Steps, (x_end - x_start) / Stand_Time_Scalar)
    if Leg == "FL" or Leg == "HL":
        y_dot_Stand = np.piecewise(t, conditions, [lambda t: cos_interp_dot(t, y_normal, y_stretch, 0, 0.4*one_segment_time),
                                                   lambda t: np.zeros_like(t),
                                                   lambda t: cos_interp_dot(t, y_stretch, y_normal, 0.8*one_segment_time, 1.2*one_segment_time),
                                                   lambda t: np.zeros_like(t),
                                                   lambda t: cos_interp_dot(t, y_normal, y_fold, 1.8*one_segment_time, 2.2*one_segment_time),
                                                   lambda t: np.zeros_like(t),
                                                   lambda t: cos_interp_dot(t, y_fold, y_normal, 2.6*one_segment_time, 3*one_segment_time)])
    elif Leg == "FR" or Leg == "HR":
        y_dot_Stand = np.piecewise(t, conditions, [lambda t: cos_interp_dot(t, y_normal, y_fold, 0, 0.4*one_segment_time),
                                                   lambda t: np.zeros_like(t),
                                                   lambda t: cos_interp_dot(t, y_fold, y_normal, 0.8*one_segment_time, 1.2*one_segment_time),
                                                   lambda t: np.zeros_like(t),
                                                   lambda t: cos_interp_dot(t, y_normal, y_stretch, 1.8*one_segment_time, 2.2*one_segment_time),
                                                   lambda t: np.zeros_like(t),
                                                   lambda t: cos_interp_dot(t, y_stretch, y_normal, 2.6*one_segment_time, 3*one_segment_time)]) 
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
                                Stand_Time_Steps,
                                Stand_Phase_type = "Constant",
                                Leg = "FL"
                                ):
    """
    Building the Bezier Trajectory for one leg in the Body Frame
    
    Args:
        Swing_Time_Scalar (float): The Swing Time
        Stand_Time_Scalar (float): The Stand Time
        Swing_Time_Steps (int): Swing Time Steps
        Stand_Time_Steps (int): Stand Time Steps
        Stand_Phase_type (str): The type of stand phase trajectory, either "Constant", "Modified" or "Hyun"
        Leg (str): The leg for which to generate the trajectory, either "FL", "FR", "HL" or "HR"
        
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
    if Stand_Phase_type == "Constant":
         Stand_Time_Vector, Stand_Trajectory, Stand_Velocity = Generate_Bezier_Stand_Trajectory(Swing_Trajectory,Stand_Time_Scalar,Stand_Time_Steps)
    elif Stand_Phase_type == "Modified":
        if Leg == "FL" or Leg == "HL":
            Stand_Time_Vector, Stand_Trajectory, Stand_Velocity = Generate_Bezier_Stand_Trajectory_Modified(Swing_Trajectory,Stand_Time_Scalar,Stand_Time_Steps,Leg)
        elif Leg == "FR" or Leg == "HR":
            Stand_Time_Vector, Stand_Trajectory, Stand_Velocity = Generate_Bezier_Stand_Trajectory_Modified(Swing_Trajectory,Stand_Time_Scalar,Stand_Time_Steps,Leg)
    elif Stand_Phase_type == "Hyun":
        print("Hyun stand phase not implemented yet")
        return None, None, None
        #Stand_Time_Vector, Stand_Trajectory, Stand_Velocity = Generate_Bezier_Stand_Trajectory_Hyun(Swing_Trajectory,Stand_Time_Scalar,Stand_Time_Steps)

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

def Bezier_Add_Transfer_Phase(t, t_transfer, t_with_transfer, Total_Time_Steps, Total_Time_Steps_With_Transfer, Transfer_Time_Steps, Swing_Time_Steps, Transfer_Time_Scalar, Phase_Offset, x_offset, y_offset, FL_Bezier_Trajectory, FR_Bezier_Trajectory, HL_Bezier_Trajectory, HR_Bezier_Trajectory, FL_Bezier_Velocities, FR_Bezier_Velocities, HL_Bezier_Velocities, HR_Bezier_Velocities):
    """
    Add transfer phases to the already computed Bezier trajectory

    Currently only done for "mixed" swing sequence, FL -> HR -> FR -> HL

    FL swing: x offset positive, y offset positive for all legs
    HR swing: x offset negative, y offset negative for all legs
    FR swing: x offset positive, y offset negative for all legs
    HL swing: x offset negative, y offset positive for all legs

    Args:
        t: Time vector for the original trajectory without transfer
        t_transfer: Time vector for transfer phase
        t_with_transfer: Time vector for the trajectory with transfer
        Total_Time_Steps: Total number of time steps without transfer
        Total_Time_Steps_With_Transfer: Total number of time steps with transfer
        Transfer_Time_Steps: Number of time steps in the transfer phase
        Swing_Time_Steps: Number of time steps in the swing phase
        Transfer_Time_Scalar: Duration of the transfer phase
        Phase_Offset: List with phase offset for specific gait sequence
        x_offset: X-axis offset for COM transfer
        y_offset: Y-axis offset for COM transfer
        FL_Bezier_Trajectory: FL leg Bezier trajectory without transfer
        FR_Bezier_Trajectory: FR leg Bezier trajectory without transfer
        HL_Bezier_Trajectory: HL leg Bezier trajectory without transfer
        HR_Bezier_Trajectory: HR leg Bezier trajectory without transfer
        FL_Bezier_Velocities: FL leg Bezier velocities without transfer
        FR_Bezier_Velocities: FR leg Bezier velocities without transfer
        HL_Bezier_Velocities: HL leg Bezier velocities without transfer
        HR_Bezier_Velocities: HR leg Bezier velocities without transfer

    Returns
        FL_Bezier_Trajectory_With_Transfer: FL leg Bezier trajectory with transfer
        FR_Bezier_Trajectory_With_Transfer: FR leg Bezier trajectory with transfer
        HL_Bezier_Trajectory_With_Transfer: HL leg Bezier trajectory with transfer
        HR_Bezier_Trajectory_With_Transfer: HR leg Bezier trajectory with transfer
        FL_Bezier_Velocities_With_Transfer: FL leg Bezier velocities with transfer
        FR_Bezier_Velocities_With_Transfer: FR leg Bezier velocities with transfer
        HL_Bezier_Velocities_With_Transfer: HL leg Bezier velocities with transfer
        HR_Bezier_Velocities_With_Transfer: HR leg Bezier velocities with transfer
    """
    # Preallocate new trajectory arrays with transfer
    FL_Bezier_Trajectory_With_Transfer = np.zeros((3, Total_Time_Steps_With_Transfer))
    FR_Bezier_Trajectory_With_Transfer = np.zeros((3, Total_Time_Steps_With_Transfer))
    HL_Bezier_Trajectory_With_Transfer = np.zeros((3, Total_Time_Steps_With_Transfer))
    HR_Bezier_Trajectory_With_Transfer = np.zeros((3, Total_Time_Steps_With_Transfer))

    # Preallocate new velocity arrays with transfer
    FL_Bezier_Velocities_With_Transfer = np.zeros((3, Total_Time_Steps_With_Transfer))
    FR_Bezier_Velocities_With_Transfer = np.zeros((3, Total_Time_Steps_With_Transfer))
    HL_Bezier_Velocities_With_Transfer = np.zeros((3, Total_Time_Steps_With_Transfer))
    HR_Bezier_Velocities_With_Transfer = np.zeros((3, Total_Time_Steps_With_Transfer))

    # Swing phase start — first discrete step where each leg enters swing
    SWING_SEQUENCE_Mixed = ['FL', 'HR', 'FR', 'HL']

    Swing_Start_Index = {leg: int(round(Phase_Offset[leg] * Total_Time_Steps))
                         for leg in SWING_SEQUENCE_Mixed}
    Swing_Start_Time  = {leg: t[Swing_Start_Index[leg]]
                         for leg in SWING_SEQUENCE_Mixed}

    # Create mew swing start index variable with Transfer_Time_Steps added to FL, 2*Transfer_Time_Steps added to HR, 3*Transfer_Time_Steps added to FR and 4*Transfer_Time_Steps added to HL
    Swing_Start_Index_with_transfer = {leg: Swing_Start_Index[leg] + (i + 1) * Transfer_Time_Steps
                                     for i, leg in enumerate(SWING_SEQUENCE_Mixed)}
    Swing_Start_Time_with_transfer  = {leg: t_with_transfer[Swing_Start_Index_with_transfer[leg]]
                                     for leg in SWING_SEQUENCE_Mixed}

    ## Trajectories: Swing and Stand phases ##
    # Take swing and stand phases of FL_Bezier_Trajectory and add x and y offset and allocate in new trajectory array
    FL_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['FL'] : Swing_Start_Index_with_transfer['FL'] + Swing_Time_Steps] = FL_Bezier_Trajectory[:, :Swing_Start_Index['FL'] + Swing_Time_Steps] + np.array([[x_offset], [y_offset], [0]])
    FL_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['HR'] : Swing_Start_Index_with_transfer['HR'] + Swing_Time_Steps] = FL_Bezier_Trajectory[:, Swing_Start_Index['HR'] : Swing_Start_Index['HR'] + Swing_Time_Steps] + np.array([[-x_offset], [-y_offset], [0]])
    FL_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['FR'] : Swing_Start_Index_with_transfer['FR'] + Swing_Time_Steps] = FL_Bezier_Trajectory[:, Swing_Start_Index['FR'] : Swing_Start_Index['FR'] + Swing_Time_Steps] + np.array([[x_offset], [-y_offset], [0]])
    FL_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['HL'] : Swing_Start_Index_with_transfer['HL'] + Swing_Time_Steps] = FL_Bezier_Trajectory[:, Swing_Start_Index['HL'] : Swing_Start_Index['HL'] + Swing_Time_Steps] + np.array([[-x_offset], [y_offset], [0]])

    # Take swing and stand phases of HR_Bezier_Trajectory and add x and y offset and allocate in new trajectory array
    HR_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['FL'] : Swing_Start_Index_with_transfer['FL'] + Swing_Time_Steps] = HR_Bezier_Trajectory[:, :Swing_Start_Index['FL'] + Swing_Time_Steps] + np.array([[x_offset], [y_offset], [0]])
    HR_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['HR'] : Swing_Start_Index_with_transfer['HR'] + Swing_Time_Steps] = HR_Bezier_Trajectory[:, Swing_Start_Index['HR'] : Swing_Start_Index['HR'] + Swing_Time_Steps] + np.array([[-x_offset], [-y_offset], [0]])
    HR_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['FR'] : Swing_Start_Index_with_transfer['FR'] + Swing_Time_Steps] = HR_Bezier_Trajectory[:, Swing_Start_Index['FR'] : Swing_Start_Index['FR'] + Swing_Time_Steps] + np.array([[x_offset], [-y_offset], [0]])
    HR_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['HL'] : Swing_Start_Index_with_transfer['HL'] + Swing_Time_Steps] = HR_Bezier_Trajectory[:, Swing_Start_Index['HL'] : Swing_Start_Index['HL'] + Swing_Time_Steps] + np.array([[-x_offset], [y_offset], [0]])

    # Take swing and stand phases of FR_Bezier_Trajectory and add x and y offset and allocate in new trajectory array
    FR_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['FL'] : Swing_Start_Index_with_transfer['FL'] + Swing_Time_Steps] = FR_Bezier_Trajectory[:, :Swing_Start_Index['FL'] + Swing_Time_Steps] + np.array([[x_offset], [y_offset], [0]])
    FR_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['HR'] : Swing_Start_Index_with_transfer['HR'] + Swing_Time_Steps] = FR_Bezier_Trajectory[:, Swing_Start_Index['HR'] : Swing_Start_Index['HR'] + Swing_Time_Steps] + np.array([[-x_offset], [-y_offset], [0]])
    FR_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['FR'] : Swing_Start_Index_with_transfer['FR'] + Swing_Time_Steps] = FR_Bezier_Trajectory[:, Swing_Start_Index['FR'] : Swing_Start_Index['FR'] + Swing_Time_Steps] + np.array([[x_offset], [-y_offset], [0]])
    FR_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['HL'] : Swing_Start_Index_with_transfer['HL'] + Swing_Time_Steps] = FR_Bezier_Trajectory[:, Swing_Start_Index['HL'] : Swing_Start_Index['HL'] + Swing_Time_Steps] + np.array([[-x_offset], [y_offset], [0]])

    # Take swing and stand phases of HL_Bezier_Trajectory and add x and y offset and allocate in new trajectory array
    HL_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['FL'] : Swing_Start_Index_with_transfer['FL'] + Swing_Time_Steps] = HL_Bezier_Trajectory[:, :Swing_Start_Index['FL'] + Swing_Time_Steps] + np.array([[x_offset], [y_offset], [0]])
    HL_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['HR'] : Swing_Start_Index_with_transfer['HR'] + Swing_Time_Steps] = HL_Bezier_Trajectory[:, Swing_Start_Index['HR'] : Swing_Start_Index['HR'] + Swing_Time_Steps] + np.array([[-x_offset], [-y_offset], [0]])
    HL_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['FR'] : Swing_Start_Index_with_transfer['FR'] + Swing_Time_Steps] = HL_Bezier_Trajectory[:, Swing_Start_Index['FR'] : Swing_Start_Index['FR'] + Swing_Time_Steps] + np.array([[x_offset], [-y_offset], [0]])
    HL_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['HL'] : Swing_Start_Index_with_transfer['HL'] + Swing_Time_Steps] = HL_Bezier_Trajectory[:, Swing_Start_Index['HL'] : Swing_Start_Index['HL'] + Swing_Time_Steps] + np.array([[-x_offset], [y_offset], [0]])

    ## Trajectories: Transfer phases ##
    # Compute cos interpolation from previous position to next position for all transfer phases for FL leg and allocate in new trajectory array
    FL_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['FL'] - Transfer_Time_Steps : Swing_Start_Index_with_transfer['FL']] = cos_interp(t_transfer, FL_Bezier_Trajectory_With_Transfer[:, -1].reshape((3, 1)), FL_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['FL']].reshape((3, 1)), 0, Transfer_Time_Scalar)
    FL_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['HR'] - Transfer_Time_Steps : Swing_Start_Index_with_transfer['HR']] = cos_interp(t_transfer, FL_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['HR'] - Transfer_Time_Steps - 1].reshape((3, 1)), FL_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['HR']].reshape((3, 1)), 0, Transfer_Time_Scalar)
    FL_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['FR'] - Transfer_Time_Steps : Swing_Start_Index_with_transfer['FR']] = cos_interp(t_transfer, FL_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['FR'] - Transfer_Time_Steps - 1].reshape((3, 1)), FL_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['FR']].reshape((3, 1)), 0, Transfer_Time_Scalar)
    FL_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['HL'] - Transfer_Time_Steps : Swing_Start_Index_with_transfer['HL']] = cos_interp(t_transfer, FL_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['HL'] - Transfer_Time_Steps - 1].reshape((3, 1)), FL_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['HL']].reshape((3, 1)), 0, Transfer_Time_Scalar)

    # Compute cos interpolation from previous position to next position for all transfer phases for HR leg and allocate in new trajectory array
    HR_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['FL'] - Transfer_Time_Steps : Swing_Start_Index_with_transfer['FL']] = cos_interp(t_transfer, HR_Bezier_Trajectory_With_Transfer[:, -1].reshape((3, 1)), HR_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['FL']].reshape((3, 1)), 0, Transfer_Time_Scalar)
    HR_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['HR'] - Transfer_Time_Steps : Swing_Start_Index_with_transfer['HR']] = cos_interp(t_transfer, HR_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['HR'] - Transfer_Time_Steps - 1].reshape((3, 1)), HR_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['HR']].reshape((3, 1)), 0, Transfer_Time_Scalar)
    HR_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['FR'] - Transfer_Time_Steps : Swing_Start_Index_with_transfer['FR']] = cos_interp(t_transfer, HR_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['FR'] - Transfer_Time_Steps - 1].reshape((3, 1)), HR_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['FR']].reshape((3, 1)), 0, Transfer_Time_Scalar)
    HR_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['HL'] - Transfer_Time_Steps : Swing_Start_Index_with_transfer['HL']] = cos_interp(t_transfer, HR_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['HL'] - Transfer_Time_Steps - 1].reshape((3, 1)), HR_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['HL']].reshape((3, 1)), 0, Transfer_Time_Scalar)

    # Compute cos interpolation from previous position to next position for all transfer phases for FR leg and allocate in new trajectory array
    FR_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['FL'] - Transfer_Time_Steps : Swing_Start_Index_with_transfer['FL']] = cos_interp(t_transfer, FR_Bezier_Trajectory_With_Transfer[:, -1].reshape((3, 1)), FR_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['FL']].reshape((3, 1)), 0, Transfer_Time_Scalar)
    FR_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['HR'] - Transfer_Time_Steps : Swing_Start_Index_with_transfer['HR']] = cos_interp(t_transfer, FR_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['HR'] - Transfer_Time_Steps - 1].reshape((3, 1)), FR_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['HR']].reshape((3, 1)), 0, Transfer_Time_Scalar)
    FR_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['FR'] - Transfer_Time_Steps : Swing_Start_Index_with_transfer['FR']] = cos_interp(t_transfer, FR_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['FR'] - Transfer_Time_Steps - 1].reshape((3, 1)), FR_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['FR']].reshape((3, 1)), 0, Transfer_Time_Scalar)
    FR_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['HL'] - Transfer_Time_Steps : Swing_Start_Index_with_transfer['HL']] = cos_interp(t_transfer, FR_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['HL'] - Transfer_Time_Steps - 1].reshape((3, 1)), FR_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['HL']].reshape((3, 1)), 0, Transfer_Time_Scalar)

    # Compute cos interpolation from previous position to next position for all transfer phases for HL leg and allocate in new trajectory array
    HL_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['FL'] - Transfer_Time_Steps : Swing_Start_Index_with_transfer['FL']] = cos_interp(t_transfer, HL_Bezier_Trajectory_With_Transfer[:, -1].reshape((3, 1)), HL_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['FL']].reshape((3, 1)), 0, Transfer_Time_Scalar)
    HL_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['HR'] - Transfer_Time_Steps : Swing_Start_Index_with_transfer['HR']] = cos_interp(t_transfer, HL_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['HR'] - Transfer_Time_Steps - 1].reshape((3, 1)), HL_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['HR']].reshape((3, 1)), 0, Transfer_Time_Scalar)
    HL_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['FR'] - Transfer_Time_Steps : Swing_Start_Index_with_transfer['FR']] = cos_interp(t_transfer, HL_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['FR'] - Transfer_Time_Steps - 1].reshape((3, 1)), HL_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['FR']].reshape((3, 1)), 0, Transfer_Time_Scalar)
    HL_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['HL'] - Transfer_Time_Steps : Swing_Start_Index_with_transfer['HL']] = cos_interp(t_transfer, HL_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['HL'] - Transfer_Time_Steps - 1].reshape((3, 1)), HL_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['HL']].reshape((3, 1)), 0, Transfer_Time_Scalar)

    ## Velocities: Swing and Stand phases ##
    # Take swing and stand phases of FL_Bezier_Velocities and allocate in new trajectory array (no offset for velocities)
    FL_Bezier_Velocities_With_Transfer[:, Swing_Start_Index_with_transfer['FL'] : Swing_Start_Index_with_transfer['FL'] + Swing_Time_Steps] = FL_Bezier_Velocities[:, :Swing_Start_Index['FL'] + Swing_Time_Steps]
    FL_Bezier_Velocities_With_Transfer[:, Swing_Start_Index_with_transfer['HR'] : Swing_Start_Index_with_transfer['HR'] + Swing_Time_Steps] = FL_Bezier_Velocities[:, Swing_Start_Index['HR'] : Swing_Start_Index['HR'] + Swing_Time_Steps]
    FL_Bezier_Velocities_With_Transfer[:, Swing_Start_Index_with_transfer['FR'] : Swing_Start_Index_with_transfer['FR'] + Swing_Time_Steps] = FL_Bezier_Velocities[:, Swing_Start_Index['FR'] : Swing_Start_Index['FR'] + Swing_Time_Steps]
    FL_Bezier_Velocities_With_Transfer[:, Swing_Start_Index_with_transfer['HL'] : Swing_Start_Index_with_transfer['HL'] + Swing_Time_Steps] = FL_Bezier_Velocities[:, Swing_Start_Index['HL'] : Swing_Start_Index['HL'] + Swing_Time_Steps]

    # Take swing and stand phases of HR_Bezier_Velocities and add x and y offset and allocate in new trajectory array
    HR_Bezier_Velocities_With_Transfer[:, Swing_Start_Index_with_transfer['FL'] : Swing_Start_Index_with_transfer['FL'] + Swing_Time_Steps] = HR_Bezier_Velocities[:, :Swing_Start_Index['FL'] + Swing_Time_Steps]
    HR_Bezier_Velocities_With_Transfer[:, Swing_Start_Index_with_transfer['HR'] : Swing_Start_Index_with_transfer['HR'] + Swing_Time_Steps] = HR_Bezier_Velocities[:, Swing_Start_Index['HR'] : Swing_Start_Index['HR'] + Swing_Time_Steps]
    HR_Bezier_Velocities_With_Transfer[:, Swing_Start_Index_with_transfer['FR'] : Swing_Start_Index_with_transfer['FR'] + Swing_Time_Steps] = HR_Bezier_Velocities[:, Swing_Start_Index['FR'] : Swing_Start_Index['FR'] + Swing_Time_Steps]
    HR_Bezier_Velocities_With_Transfer[:, Swing_Start_Index_with_transfer['HL'] : Swing_Start_Index_with_transfer['HL'] + Swing_Time_Steps] = HR_Bezier_Velocities[:, Swing_Start_Index['HL'] : Swing_Start_Index['HL'] + Swing_Time_Steps]

    # Take swing and stand phases of FR_Bezier_Velocities and add x and y offset and allocate in new trajectory array
    FR_Bezier_Velocities_With_Transfer[:, Swing_Start_Index_with_transfer['FL'] : Swing_Start_Index_with_transfer['FL'] + Swing_Time_Steps] = FR_Bezier_Velocities[:, :Swing_Start_Index['FL'] + Swing_Time_Steps]
    FR_Bezier_Velocities_With_Transfer[:, Swing_Start_Index_with_transfer['HR'] : Swing_Start_Index_with_transfer['HR'] + Swing_Time_Steps] = FR_Bezier_Velocities[:, Swing_Start_Index['HR'] : Swing_Start_Index['HR'] + Swing_Time_Steps]
    FR_Bezier_Velocities_With_Transfer[:, Swing_Start_Index_with_transfer['FR'] : Swing_Start_Index_with_transfer['FR'] + Swing_Time_Steps] = FR_Bezier_Velocities[:, Swing_Start_Index['FR'] : Swing_Start_Index['FR'] + Swing_Time_Steps]
    FR_Bezier_Velocities_With_Transfer[:, Swing_Start_Index_with_transfer['HL'] : Swing_Start_Index_with_transfer['HL'] + Swing_Time_Steps] = FR_Bezier_Velocities[:, Swing_Start_Index['HL'] : Swing_Start_Index['HL'] + Swing_Time_Steps]

    # Take swing and stand phases of HL_Bezier_Velocities and add x and y offset and allocate in new trajectory array
    HL_Bezier_Velocities_With_Transfer[:, Swing_Start_Index_with_transfer['FL'] : Swing_Start_Index_with_transfer['FL'] + Swing_Time_Steps] = HL_Bezier_Velocities[:, :Swing_Start_Index['FL'] + Swing_Time_Steps]
    HL_Bezier_Velocities_With_Transfer[:, Swing_Start_Index_with_transfer['HR'] : Swing_Start_Index_with_transfer['HR'] + Swing_Time_Steps] = HL_Bezier_Velocities[:, Swing_Start_Index['HR'] : Swing_Start_Index['HR'] + Swing_Time_Steps]
    HL_Bezier_Velocities_With_Transfer[:, Swing_Start_Index_with_transfer['FR'] : Swing_Start_Index_with_transfer['FR'] + Swing_Time_Steps] = HL_Bezier_Velocities[:, Swing_Start_Index['FR'] : Swing_Start_Index['FR'] + Swing_Time_Steps]
    HL_Bezier_Velocities_With_Transfer[:, Swing_Start_Index_with_transfer['HL'] : Swing_Start_Index_with_transfer['HL'] + Swing_Time_Steps] = HL_Bezier_Velocities[:, Swing_Start_Index['HL'] : Swing_Start_Index['HL'] + Swing_Time_Steps] 

    ## Velocities: Transfer phases ##
    # Compute cos interpolation derivatives from previous position to next position for all transfer phases for FL leg and allocate in new trajectory array
    FL_Bezier_Velocities_With_Transfer[:, Swing_Start_Index_with_transfer['FL'] - Transfer_Time_Steps : Swing_Start_Index_with_transfer['FL']] = cos_interp_dot(t_transfer, FL_Bezier_Trajectory_With_Transfer[:, -1].reshape((3, 1)), FL_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['FL']].reshape((3, 1)), 0, Transfer_Time_Scalar)
    FL_Bezier_Velocities_With_Transfer[:, Swing_Start_Index_with_transfer['HR'] - Transfer_Time_Steps : Swing_Start_Index_with_transfer['HR']] = cos_interp_dot(t_transfer, FL_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['HR'] - Transfer_Time_Steps - 1].reshape((3, 1)), FL_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['HR']].reshape((3, 1)), 0, Transfer_Time_Scalar)
    FL_Bezier_Velocities_With_Transfer[:, Swing_Start_Index_with_transfer['FR'] - Transfer_Time_Steps : Swing_Start_Index_with_transfer['FR']] = cos_interp_dot(t_transfer, FL_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['FR'] - Transfer_Time_Steps - 1].reshape((3, 1)), FL_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['FR']].reshape((3, 1)), 0, Transfer_Time_Scalar)
    FL_Bezier_Velocities_With_Transfer[:, Swing_Start_Index_with_transfer['HL'] - Transfer_Time_Steps : Swing_Start_Index_with_transfer['HL']] = cos_interp_dot(t_transfer, FL_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['HL'] - Transfer_Time_Steps - 1].reshape((3, 1)), FL_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['HL']].reshape((3, 1)), 0, Transfer_Time_Scalar)

    # Compute cos interpolation derivatives from previous position to next position for all transfer phases for HR leg and allocate in new trajectory array
    HR_Bezier_Velocities_With_Transfer[:, Swing_Start_Index_with_transfer['FL'] - Transfer_Time_Steps : Swing_Start_Index_with_transfer['FL']] = cos_interp_dot(t_transfer, HR_Bezier_Trajectory_With_Transfer[:, -1].reshape((3, 1)), HR_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['FL']].reshape((3, 1)), 0, Transfer_Time_Scalar)
    HR_Bezier_Velocities_With_Transfer[:, Swing_Start_Index_with_transfer['HR'] - Transfer_Time_Steps : Swing_Start_Index_with_transfer['HR']] = cos_interp_dot(t_transfer, HR_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['HR'] - Transfer_Time_Steps - 1].reshape((3, 1)), HR_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['HR']].reshape((3, 1)), 0, Transfer_Time_Scalar)
    HR_Bezier_Velocities_With_Transfer[:, Swing_Start_Index_with_transfer['FR'] - Transfer_Time_Steps : Swing_Start_Index_with_transfer['FR']] = cos_interp_dot(t_transfer, HR_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['FR'] - Transfer_Time_Steps - 1].reshape((3, 1)), HR_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['FR']].reshape((3, 1)), 0, Transfer_Time_Scalar)
    HR_Bezier_Velocities_With_Transfer[:, Swing_Start_Index_with_transfer['HL'] - Transfer_Time_Steps : Swing_Start_Index_with_transfer['HL']] = cos_interp_dot(t_transfer, HR_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['HL'] - Transfer_Time_Steps - 1].reshape((3, 1)), HR_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['HL']].reshape((3, 1)), 0, Transfer_Time_Scalar)

    # Compute cos interpolation derivatives from previous position to next position for all transfer phases for FR leg and allocate in new trajectory array
    FR_Bezier_Velocities_With_Transfer[:, Swing_Start_Index_with_transfer['FL'] - Transfer_Time_Steps : Swing_Start_Index_with_transfer['FL']] = cos_interp_dot(t_transfer, FR_Bezier_Trajectory_With_Transfer[:, -1].reshape((3, 1)), FR_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['FL']].reshape((3, 1)), 0, Transfer_Time_Scalar)
    FR_Bezier_Velocities_With_Transfer[:, Swing_Start_Index_with_transfer['HR'] - Transfer_Time_Steps : Swing_Start_Index_with_transfer['HR']] = cos_interp_dot(t_transfer, FR_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['HR'] - Transfer_Time_Steps - 1].reshape((3, 1)), FR_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['HR']].reshape((3, 1)), 0, Transfer_Time_Scalar)
    FR_Bezier_Velocities_With_Transfer[:, Swing_Start_Index_with_transfer['FR'] - Transfer_Time_Steps : Swing_Start_Index_with_transfer['FR']] = cos_interp_dot(t_transfer, FR_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['FR'] - Transfer_Time_Steps - 1].reshape((3, 1)), FR_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['FR']].reshape((3, 1)), 0, Transfer_Time_Scalar)
    FR_Bezier_Velocities_With_Transfer[:, Swing_Start_Index_with_transfer['HL'] - Transfer_Time_Steps : Swing_Start_Index_with_transfer['HL']] = cos_interp_dot(t_transfer, FR_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['HL'] - Transfer_Time_Steps - 1].reshape((3, 1)), FR_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['HL']].reshape((3, 1)), 0, Transfer_Time_Scalar)

    # Compute cos interpolation derivatives from previous position to next position for all transfer phases for HL leg and allocate in new trajectory array
    HL_Bezier_Velocities_With_Transfer[:, Swing_Start_Index_with_transfer['FL'] - Transfer_Time_Steps : Swing_Start_Index_with_transfer['FL']] = cos_interp_dot(t_transfer, HL_Bezier_Trajectory_With_Transfer[:, -1].reshape((3, 1)), HL_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['FL']].reshape((3, 1)), 0, Transfer_Time_Scalar)
    HL_Bezier_Velocities_With_Transfer[:, Swing_Start_Index_with_transfer['HR'] - Transfer_Time_Steps : Swing_Start_Index_with_transfer['HR']] = cos_interp_dot(t_transfer, HL_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['HR'] - Transfer_Time_Steps - 1].reshape((3, 1)), HL_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['HR']].reshape((3, 1)), 0, Transfer_Time_Scalar)
    HL_Bezier_Velocities_With_Transfer[:, Swing_Start_Index_with_transfer['FR'] - Transfer_Time_Steps : Swing_Start_Index_with_transfer['FR']] = cos_interp_dot(t_transfer, HL_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['FR'] - Transfer_Time_Steps - 1].reshape((3, 1)), HL_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['FR']].reshape((3, 1)), 0, Transfer_Time_Scalar)
    HL_Bezier_Velocities_With_Transfer[:, Swing_Start_Index_with_transfer['HL'] - Transfer_Time_Steps : Swing_Start_Index_with_transfer['HL']] = cos_interp_dot(t_transfer, HL_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['HL'] - Transfer_Time_Steps - 1].reshape((3, 1)), HL_Bezier_Trajectory_With_Transfer[:, Swing_Start_Index_with_transfer['HL']].reshape((3, 1)), 0, Transfer_Time_Scalar)
    
    # Roll everything by one swing phase and one half transfer phase to ensure start with COM centered
    Roll_amount = (Swing_Time_Steps + 0.5 * Transfer_Time_Steps) / Total_Time_Steps_With_Transfer
    FL_Bezier_Trajectory_With_Transfer, FL_Bezier_Velocities_With_Transfer = Apply_Phase_Offset(FL_Bezier_Trajectory_With_Transfer, FL_Bezier_Velocities_With_Transfer, Roll_amount)
    FR_Bezier_Trajectory_With_Transfer, FR_Bezier_Velocities_With_Transfer = Apply_Phase_Offset(FR_Bezier_Trajectory_With_Transfer, FR_Bezier_Velocities_With_Transfer, Roll_amount)
    HL_Bezier_Trajectory_With_Transfer, HL_Bezier_Velocities_With_Transfer = Apply_Phase_Offset(HL_Bezier_Trajectory_With_Transfer, HL_Bezier_Velocities_With_Transfer, Roll_amount)
    HR_Bezier_Trajectory_With_Transfer, HR_Bezier_Velocities_With_Transfer = Apply_Phase_Offset(HR_Bezier_Trajectory_With_Transfer, HR_Bezier_Velocities_With_Transfer, Roll_amount)

    return FL_Bezier_Trajectory_With_Transfer, FR_Bezier_Trajectory_With_Transfer, HL_Bezier_Trajectory_With_Transfer, HR_Bezier_Trajectory_With_Transfer, FL_Bezier_Velocities_With_Transfer, FR_Bezier_Velocities_With_Transfer, HL_Bezier_Velocities_With_Transfer, HR_Bezier_Velocities_With_Transfer

def Building_Stand_To_Walk_Trajectories(Swing_Time_Scalar, Swing_Time_Steps, FL_walk_start_x, FR_walk_start_x, HL_walk_start_x, HR_walk_start_x):
    """
    Building the trajectories for the transition from stand to walk.
    Specific for each leg.
    All trajectories represented in the body frame.
    Does not include the transfer phase - add it using the Bezier_Add_Transfer_Phase function
    
    Args:
        Swing_Time_Scalar (float): The Swing Time
        Swing_Time_Steps (int): Swing Time Steps
        FL_walk_start_x: x position of the front left foot at the start of the walk cycle (in body frame) [m]
        FR_walk_start_x: x position of the front right foot at the start of the walk cycle (in body frame) [m]
        HL_walk_start_x: x position of the hind left foot at the start of the walk cycle (in body frame) [m]
        HR_walk_start_x: x position of the hind right foot at the start of the walk cycle (in body frame) [m]

    Returns: (All represented in the body frame)
        FL_Stand_To_Walk:               FL leg Bezier curve trajectory from standing position to walk start position
        FR_Stand_To_Walk:               FR leg Bezier curve trajectory from standing position to walk start position
        HL_Stand_To_Walk:               HL leg Bezier curve trajectory from standing position to walk start position
        HR_Stand_To_Walk:               HR leg Bezier curve trajectory from standing position to walk start position
        FL_Stand_To_Walk_Velocities:    FL leg Bezier curve velocities from standing position to walk start position
        FR_Stand_To_Walk_Velocities:    FR leg Bezier curve velocities from standing position to walk start position
        HL_Stand_To_Walk_Velocities:    HL leg Bezier curve velocities from standing position to walk start position
        HR_Stand_To_Walk_Velocities:    HR leg Bezier curve velocities from standing position to walk start position
    """

    # Define kinematic body lengths
    l_k = 0.7048  # Length of body in kinematic model (meters)
    w_k = 0.220   # Width of body in kinematic model (meters)

    # Compute End x position for each leg in Bezier frame
    FL_end_x = FL_walk_start_x - l_k/2 # FL end x position in Bezier frame
    FR_end_x = FR_walk_start_x - l_k/2 # FR end x position in Bezier frame
    HL_end_x = HL_walk_start_x + l_k/2 # HL end x position in Bezier frame
    HR_end_x = HR_walk_start_x + l_k/2 # HR end x position in Bezier frame

    # Define Bezier control points for each legs swing phase (Bezier frame)
    # Front Left Leg:
    c_kX_FL = np.array([0, -0.1*FL_end_x, -0.2*FL_end_x, -0.2*FL_end_x, 0.5*FL_end_x, 0.5*FL_end_x, 1.2*FL_end_x, 1.2*FL_end_x, 1.1*FL_end_x, FL_end_x]) # [m]
    c_kY_FL = np.array([0.44, 0.44, 0.38, 0.38, 0.40, 0.40, 0.38, 0.38, 0.44, 0.44]) # [m]
    c_k_FL = np.column_stack((c_kX_FL, c_kY_FL)) # Control points for FL leg in Bezier frame
    # Front Right Leg:
    c_kX_FR = np.array([0, -0.1*FR_end_x, -0.2*FR_end_x, -0.2*FR_end_x, 0.5*FR_end_x, 0.5*FR_end_x, 1.2*FR_end_x, 1.2*FR_end_x, 1.1*FR_end_x, FR_end_x]) # [m]
    c_kY_FR = np.array([0.44, 0.44, 0.38, 0.38, 0.40, 0.40, 0.38, 0.38, 0.44, 0.44]) # [m]
    c_k_FR = np.column_stack((c_kX_FR, c_kY_FR)) # Control points for FR leg in Bezier frame
    # Hind Left Leg:
    c_kX_HL = np.array([0, -0.1*HL_end_x, -0.2*HL_end_x, -0.2*HL_end_x, 0.5*HL_end_x, 0.5*HL_end_x, 1.2*HL_end_x, 1.2*HL_end_x, 1.1*HL_end_x, HL_end_x]) # [m]
    c_kY_HL = np.array([0.44, 0.44, 0.38, 0.38, 0.40, 0.40, 0.38, 0.38, 0.44, 0.44]) # [m]
    c_k_HL = np.column_stack((c_kX_HL, c_kY_HL)) # Control points for HL leg in Bezier frame
    # Hind Right Leg:
    c_kX_HR = np.array([0, -0.1*HR_end_x, -0.2*HR_end_x, -0.2*HR_end_x, 0.5*HR_end_x, 0.5*HR_end_x, 1.2*HR_end_x, 1.2*HR_end_x, 1.1*HR_end_x, HR_end_x]) # [m]
    c_kY_HR = np.array([0.44, 0.44, 0.38, 0.38, 0.40, 0.40, 0.38, 0.38, 0.44, 0.44]) # [m]
    c_k_HR = np.column_stack((c_kX_HR, c_kY_HR)) # Control points for HR leg in Bezier frame

    # Generate the swing phase for each leg
    _, Swing_Trajectory_FL, Swing_Velocity_FL = Generate_Bezier_Swing_Trajectory(c_k_FL, Swing_Time_Scalar, Swing_Time_Steps)
    _, Swing_Trajectory_FR, Swing_Velocity_FR = Generate_Bezier_Swing_Trajectory(c_k_FR, Swing_Time_Scalar, Swing_Time_Steps)
    _, Swing_Trajectory_HL, Swing_Velocity_HL = Generate_Bezier_Swing_Trajectory(c_k_HL, Swing_Time_Scalar, Swing_Time_Steps)
    _, Swing_Trajectory_HR, Swing_Velocity_HR = Generate_Bezier_Swing_Trajectory(c_k_HR, Swing_Time_Scalar, Swing_Time_Steps)
    
    # Generate stand trajectories in body frame
    x_FL_stand_backend = FL_end_x * np.ones(3 * Swing_Time_Steps)
    x_FR_stand_frontend = np.zeros(2 * Swing_Time_Steps)
    x_FR_stand_backend = FR_end_x * np.ones(Swing_Time_Steps)
    x_HL_stand_frontend = np.zeros(3 * Swing_Time_Steps)
    x_HR_stand_frontend = np.zeros(Swing_Time_Steps)
    x_HR_stand_backend = HR_end_x * np.ones(2 * Swing_Time_Steps)
    z_stand = 0.44 * np.ones(Swing_Time_Steps)

    # Assemble all trajectories in body frame
    # FL
    x_FL = np.concatenate((Swing_Trajectory_FL[:,0], x_FL_stand_backend))
    y_FL = np.zeros(4 * Swing_Time_Steps)
    z_FL = - np.concatenate((Swing_Trajectory_FL[:,1], z_stand, z_stand, z_stand))
    FL_Stand_To_Walk = np.vstack((x_FL, y_FL, z_FL))
    # FR
    x_FR = np.concatenate((x_FR_stand_frontend, Swing_Trajectory_FR[:,0], x_FR_stand_backend))
    y_FR = np.zeros(4 * Swing_Time_Steps)
    z_FR = - np.concatenate((z_stand, z_stand, Swing_Trajectory_FR[:,1], z_stand))
    FR_Stand_To_Walk = np.vstack((x_FR, y_FR, z_FR))
    # HL
    x_HL = np.concatenate((x_HL_stand_frontend, Swing_Trajectory_HL[:,0]))
    y_HL = np.zeros(4 * Swing_Time_Steps)
    z_HL = - np.concatenate((z_stand, z_stand, z_stand, Swing_Trajectory_HL[:,1]))
    HL_Stand_To_Walk = np.vstack((x_HL, y_HL, z_HL))
    # HR
    x_HR = np.concatenate((x_HR_stand_frontend, Swing_Trajectory_HR[:,0], x_HR_stand_backend))
    y_HR = np.zeros(4 * Swing_Time_Steps)
    z_HR = - np.concatenate((z_stand, Swing_Trajectory_HR[:,1], z_stand, z_stand))
    HR_Stand_To_Walk = np.vstack((x_HR, y_HR, z_HR))

    # Velocities - zero
    dot_stand = np.zeros(Swing_Time_Steps)

    # Assemble all velocity arrays
    # FL
    x_dot_FL = np.concatenate((Swing_Velocity_FL[:,0], np.zeros(3 * Swing_Time_Steps)))
    y_dot_FL = np.zeros(4 * Swing_Time_Steps)
    z_dot_FL = - np.concatenate((Swing_Velocity_FL[:,1], dot_stand, dot_stand, dot_stand))
    FL_Stand_To_Walk_Velocities = np.vstack((x_dot_FL, y_dot_FL, z_dot_FL))
    # FR
    x_dot_FR = np.concatenate((np.zeros(2 * Swing_Time_Steps), Swing_Velocity_FR[:,0], np.zeros(Swing_Time_Steps)))
    y_dot_FR = np.zeros(4 * Swing_Time_Steps)
    z_dot_FR = - np.concatenate((dot_stand, dot_stand, Swing_Velocity_FR[:,1], dot_stand))
    FR_Stand_To_Walk_Velocities = np.vstack((x_dot_FR, y_dot_FR, z_dot_FR))
    # HL
    x_dot_HL = np.concatenate((np.zeros(3 * Swing_Time_Steps), Swing_Velocity_HL[:,0]))
    y_dot_HL = np.zeros(4 * Swing_Time_Steps)
    z_dot_HL = - np.concatenate((dot_stand, dot_stand, dot_stand, Swing_Velocity_HL[:,1]))
    HL_Stand_To_Walk_Velocities = np.vstack((x_dot_HL, y_dot_HL, z_dot_HL))
    # HR
    x_dot_HR = np.concatenate((np.zeros(Swing_Time_Steps), Swing_Velocity_HR[:,0], np.zeros(2 * Swing_Time_Steps)))
    y_dot_HR = np.zeros(4 * Swing_Time_Steps)
    z_dot_HR = - np.concatenate((dot_stand, Swing_Velocity_HR[:,1], dot_stand, dot_stand))
    HR_Stand_To_Walk_Velocities = np.vstack((x_dot_HR, y_dot_HR, z_dot_HR))

    # Translate to shoulders from COM location
    Front_Left_Shoulder     = np.array([[ l_k/2], [ w_k/2 + 0.078], [0]])
    Front_Right_Shoulder    = np.array([[ l_k/2], [-w_k/2 - 0.078], [0]])
    Hind_Left_Shoulder      = np.array([[-l_k/2], [ w_k/2 + 0.078], [0]])
    Hind_Right_Shoulder     = np.array([[-l_k/2], [-w_k/2 - 0.078], [0]])
    FL_Stand_To_Walk = (FL_Stand_To_Walk + Front_Left_Shoulder) 
    FR_Stand_To_Walk = (FR_Stand_To_Walk + Front_Right_Shoulder)
    HL_Stand_To_Walk = (HL_Stand_To_Walk + Hind_Left_Shoulder)
    HR_Stand_To_Walk = (HR_Stand_To_Walk + Hind_Right_Shoulder)

    return FL_Stand_To_Walk, FR_Stand_To_Walk, HL_Stand_To_Walk, HR_Stand_To_Walk, FL_Stand_To_Walk_Velocities, FR_Stand_To_Walk_Velocities, HL_Stand_To_Walk_Velocities, HR_Stand_To_Walk_Velocities