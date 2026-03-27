import numpy as np
from scipy.special import comb


# ─────────────────────────────────────────────
#  Core Bézier Evaluation
# ─────────────────────────────────────────────

def Bezier_Curve(c_k, t):
    """Evaluate a single point and velocity on a Bézier curve.

    Args:
        c_k (np.ndarray, shape (n, 2)): Control points.
            Column 0 = forward/x direction (leg base frame z-axis in swing)
            Column 1 = height/downward direction
        t (float): Normalised time in [0, 1].

    Returns:
        Position (np.ndarray, shape (2,)): [forward_pos, height_pos]
        Velocity (np.ndarray, shape (2,)): velocity in normalised time.
            Multiply by 1/T_swing to get real velocity [m/s].
    """
    n = len(c_k) - 1
    k = np.arange(0, n + 1)

    B  = comb(n,     k     ) * (t ** k     ) * ((1 - t) ** (n     - k     ))
    Bv = comb(n - 1, k[:-1]) * (t ** k[:-1]) * ((1 - t) ** (n - 1 - k[:-1]))

    Position = np.dot(B,  c_k)
    Velocity = n * np.dot(Bv, np.diff(c_k, axis=0))

    return Position, Velocity


def Bezier_Control_Points():
    """Return the standard Bézier control points (Hyun et al. 2014 scaling).

    Returns:
        c_k (np.ndarray, shape (12, 2)): Stacked [x_ctrl, y_ctrl] columns.
    """
    X = np.array([-0.200, -0.2805, -0.300, -0.300, -0.300,
                   0,      0,       0,      0.3032, 0.3032, 0.2826, 0.200])
    Y = np.array([ 0.500,  0.500,   0.3611, 0.3611, 0.3611,
                   0.3611, 0.3611,  0.3214, 0.3214, 0.3214, 0.500,  0.500])
    Scaling_Factor = 0.6
    Offset         = 0.16

    c_kX = Scaling_Factor * X
    c_kY = Offset + Scaling_Factor * Y

    return np.column_stack((c_kX, c_kY))


# ─────────────────────────────────────────────
#  Trajectory Generation — Swing Phase
# ─────────────────────────────────────────────

def Generate_Swing_Trajectory(c_k, T_swing, num_steps):
    """Generate the Bézier swing-phase trajectory for one step cycle.

    Args:
        c_k       (np.ndarray, shape (n, 2)): Bézier control points.
        T_swing   (float): Duration of the swing phase [s].
        num_steps (int):   Number of discrete time steps for the swing phase.

    Returns:
        t_swing     (np.ndarray, shape (num_steps,)): Absolute time vector [s].
        trajectory  (np.ndarray, shape (num_steps, 2)):
                        [:, 0] = forward position (Bézier frame z),
                        [:, 1] = height position  (Bézier frame x).
        velocity    (np.ndarray, shape (num_steps, 2)): Real velocity [m/s].
    """
    t_swing    = np.linspace(0, T_swing, num_steps)
    t_norm     = t_swing / T_swing          # Normalised to [0, 1]

    trajectory = np.zeros((num_steps, 2))
    velocity   = np.zeros((num_steps, 2))

    for i in range(num_steps):
        trajectory[i, :], velocity[i, :] = Bezier_Curve(c_k, t_norm[i])

    velocity /= T_swing                     # Convert from normalised to real time

    return t_swing, trajectory, velocity


# ─────────────────────────────────────────────
#  Trajectory Generation — Stand Phase
# ─────────────────────────────────────────────

def Generate_Stand_Trajectory(swing_trajectory, T_stand, num_steps):
    """Generate the linear stand-phase trajectory that follows a swing phase.

    The foot returns linearly (in the forward direction) from the swing end-point
    back to the swing start-point, while holding a constant height.

    Args:
        swing_trajectory (np.ndarray, shape (N, 2)): Output of Generate_Swing_Trajectory.
        T_stand  (float): Duration of the stand phase [s].
        num_steps (int):  Number of discrete time steps for the stand phase.

    Returns:
        t_stand     (np.ndarray, shape (num_steps,)): Time vector, offset to start
                        at T_swing (must be shifted by caller if needed).
        z_stand     (np.ndarray, shape (num_steps,)): Forward position [m].
        x_stand     (np.ndarray, shape (num_steps,)): Height position  [m] (constant).
        z_dot_stand (np.ndarray, shape (num_steps,)): Forward velocity [m/s] (constant).
        x_dot_stand (np.ndarray, shape (num_steps,)): Height velocity  [m/s] (zero).
    """
    z_start = swing_trajectory[-1, 0]   # Where swing ended
    z_end   = swing_trajectory[ 0, 0]   # Where swing started (return target)
    x_const = swing_trajectory[-1, 1]   # Height is constant during stand

    t_stand     = np.linspace(0, T_stand, num_steps)
    z_stand     = np.linspace(z_start, z_end, num_steps)
    x_stand     = np.full(num_steps, x_const)
    z_dot_stand = np.full(num_steps, (z_end - z_start) / T_stand)
    x_dot_stand = np.zeros(num_steps)

    return t_stand, z_stand, x_stand, z_dot_stand, x_dot_stand


# ─────────────────────────────────────────────
#  Full Cycle Assembly
# ─────────────────────────────────────────────

def Assemble_Full_Cycle(swing_traj, swing_vel, stand_z, stand_x,
                         stand_z_dot, stand_x_dot,
                         T_swing, T_stand,
                         y_offset=0.078):
    """Combine swing + stand into one full gait cycle in the leg base frame.

    The Bézier curve lives in a 2-D (forward, height) plane.  This function
    maps it into the 3-D leg base frame: (x=height, y=lateral, z=forward).

    Args:
        swing_traj  (np.ndarray, shape (N_sw, 2)): Swing trajectory.
        swing_vel   (np.ndarray, shape (N_sw, 2)): Swing velocity.
        stand_z     (np.ndarray, shape (N_st,))  : Stand forward positions.
        stand_x     (np.ndarray, shape (N_st,))  : Stand heights.
        stand_z_dot (np.ndarray, shape (N_st,))  : Stand forward velocities.
        stand_x_dot (np.ndarray, shape (N_st,))  : Stand height velocities.
        T_swing     (float): Swing duration [s].
        T_stand     (float): Stand duration [s].
        y_offset    (float): Lateral (y) position of the foot in leg base frame [m].

    Returns:
        P  (np.ndarray, shape (3, N_total)): Position  trajectory [m].
        V  (np.ndarray, shape (3, N_total)): Velocity trajectory [m/s].
        t  (np.ndarray, shape (N_total,))  : Time vector [s].
    """
    N_sw = swing_traj.shape[0]
    N_st = stand_z.shape[0]
    N    = N_sw + N_st
    
    x = np.concatenate((swing_traj[:, 1], stand_x   ))   # height  → x in leg frame
    z = np.concatenate((swing_traj[:, 0], stand_z   ))   # forward → z in leg frame
    y = np.full(N, y_offset)

    x_dot = np.concatenate((swing_vel[:, 1], stand_x_dot))
    z_dot = np.concatenate((swing_vel[:, 0], stand_z_dot))
    y_dot = np.zeros(N)

    t_swing_abs = np.linspace(0, T_swing, N_sw)
    t_stand_abs = np.linspace(T_swing, T_swing + T_stand, N_st)
    t = np.concatenate((t_swing_abs, t_stand_abs))

    P = np.vstack((x, y, z))
    V = np.vstack((x_dot, y_dot, z_dot))

    return P, V, t