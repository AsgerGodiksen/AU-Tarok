import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..')))

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from Robot.Kinematics.Inverse_Kinematics import Inverse_Kinematics
from Robot.Kinematics.Forward_Kinematics import P0_2, P0_3, P0_end
from Robot.Kinematics.Constant_Transforms import T0_B, TB_0

from Bezier_2 import Bezier_Control_Points, Generate_Swing_Trajectory, Generate_Stand_Trajectory, Assemble_Full_Cycle

# ─────────────────────────────────────────────
#  Parameters
# ─────────────────────────────────────────────

L1 = 0.078  # [m] hip abduction link
l_k = 0.7048  # [m] body length (kinematic)
w_k = 0.220   # [m] body width  (kinematic)

T_swing    = 3.0          # [s] swing phase duration
T_stand    = 3 * T_swing  # [s] stand phase duration
N_swing    = 300
N_stand    = 3 * N_swing

LEGS = ['FL', 'FR', 'HL', 'HR']
COLORS = {'FL': 'blue', 'FR': 'red', 'HL': 'green', 'HR': 'orange'}

# Lateral y-offsets for each leg in body frame
Y_BODY = {
    'FL':  w_k / 2 + L1,
    'FR': -(w_k / 2 + L1),
    'HL':  w_k / 2 + L1,
    'HR': -(w_k / 2 + L1),
}

# Phase offsets for a crawl gait (fraction of full cycle, one leg at a time)
# FL=0, HR=0.25, FR=0.5, HL=0.75
T_cycle = T_swing + T_stand
PHASE_OFFSET = {
    'FL': 0.00,
    'HR': 0.25,
    'FR': 0.50,
    'HL': 0.75,
}


# ─────────────────────────────────────────────
#  Build per-leg trajectory (one full cycle, phase-shifted)
# ─────────────────────────────────────────────

def Build_Leg_Trajectory(leg):
    """Return position P (3, N) and velocity V (3, N) arrays for one leg,
    phase-shifted according to the crawl gait offset."""

    c_k = Bezier_Control_Points()

    # --- generate base swing + stand ---
    _, swing_traj, swing_vel = Generate_Swing_Trajectory(c_k, T_swing, N_swing)
    _, stand_z, stand_x, stand_z_dot, stand_x_dot = Generate_Stand_Trajectory(
        swing_traj, T_stand, N_stand
    )

    # y-offset in the leg base frame: T0_B maps body→leg, so we get the leg-frame y
    # For a foot directly below the shoulder the body-frame y is Y_BODY[leg], z=-stand_height.
    # The lateral offset in leg base frame is simply L1 (always positive in leg frame).
    P_full, V_full, t_full = Assemble_Full_Cycle(
        swing_traj, swing_vel,
        stand_z, stand_x, stand_z_dot, stand_x_dot,
        T_swing, T_stand,
        y_offset=L1,
    )

    # --- apply phase shift by rolling the arrays ---
    N_total = P_full.shape[1]
    shift   = int(round(PHASE_OFFSET[leg] * N_total))
    P_shifted = np.roll(P_full, shift, axis=1)
    V_shifted = np.roll(V_full, shift, axis=1)

    return P_shifted, V_shifted, t_full


# ─────────────────────────────────────────────
#  Compute joint angles for all legs
# ─────────────────────────────────────────────

def Compute_Joint_Angles(P_leg, leg):
    """Run IK over every time step. Returns Theta (N, 3)."""
    N = P_leg.shape[1]
    Theta = np.zeros((N, 3))
    for i in range(N):
        p = P_leg[:, i].reshape(3, 1)
        Theta[i] = Inverse_Kinematics(p, leg)
    return Theta


# ─────────────────────────────────────────────
#  Forward kinematics → body-frame joint positions
# ─────────────────────────────────────────────

def FK_Body_Frame(theta, leg):
    """Compute the 4 body-frame joint positions for a single time step.

    Returns four (3,) arrays: origin, J2, J3, foot.
    """
    th1, th2, th3 = theta

    p_origin = TB_0(np.zeros((3, 1)),                    leg).flatten()
    p_j2     = TB_0(P0_2(th1, th2, th3, leg),            leg).flatten()
    p_j3     = TB_0(P0_3(th1, th2, th3, leg),            leg).flatten()
    p_foot   = TB_0(P0_end(th1, th2, th3, leg),          leg).flatten()

    return p_origin, p_j2, p_j3, p_foot


# ─────────────────────────────────────────────
#  Main — build trajectories
# ─────────────────────────────────────────────

print("Building trajectories...")
trajectories = {}
thetas       = {}

for leg in LEGS:
    P, V, t_full = Build_Leg_Trajectory(leg)
    trajectories[leg] = P
    thetas[leg]        = Compute_Joint_Angles(P, leg)
    print(f"  {leg} done")

N_frames = thetas['FL'].shape[0]
print(f"Total frames: {N_frames}")


# ─────────────────────────────────────────────
#  Animation
# ─────────────────────────────────────────────

fig = plt.figure(figsize=(9, 7))
ax  = fig.add_subplot(111, projection='3d')

ax.set_xlim(-0.5, 0.5)
ax.set_ylim(-0.5, 0.5)
ax.set_zlim(-0.6, 0.1)
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Z (m)')
ax.set_title('Danish Horse — Bézier Crawl Gait (All 4 Legs)')
ax.view_init(elev=20, azim=225)

# One line object per leg
lines = {leg: ax.plot([], [], [], 'o-', lw=2, color=COLORS[leg], label=leg)[0]
         for leg in LEGS}

# Foot trace (last 30 frames) per leg
TRACE_LEN = 30
traces = {leg: ax.plot([], [], [], '-', lw=1, color=COLORS[leg], alpha=0.35)[0]
          for leg in LEGS}

foot_history = {leg: np.full((TRACE_LEN, 3), np.nan) for leg in LEGS}

ax.legend(loc='upper right')


def _pts(p_list):
    """Extract xs, ys, zs from a list of (3,) arrays."""
    xs = [p[0] for p in p_list]
    ys = [p[1] for p in p_list]
    zs = [p[2] for p in p_list]
    return xs, ys, zs


def init():
    for leg in LEGS:
        lines[leg].set_data([], [])
        lines[leg].set_3d_properties([])
        traces[leg].set_data([], [])
        traces[leg].set_3d_properties([])
    return list(lines.values()) + list(traces.values())


def update(frame):
    artists = []
    for leg in LEGS:
        theta = thetas[leg][frame]
        pts   = FK_Body_Frame(theta, leg)   # (origin, j2, j3, foot)

        # Update leg line
        xs, ys, zs = _pts(pts)
        lines[leg].set_data(xs, ys)
        lines[leg].set_3d_properties(zs)

        # Update foot trace
        foot_history[leg] = np.roll(foot_history[leg], -1, axis=0)
        foot_history[leg][-1] = pts[3]     # foot position
        valid = ~np.isnan(foot_history[leg][:, 0])
        traces[leg].set_data(foot_history[leg][valid, 0],
                             foot_history[leg][valid, 1])
        traces[leg].set_3d_properties(foot_history[leg][valid, 2])

        artists += [lines[leg], traces[leg]]

    return artists


ani = animation.FuncAnimation(
    fig, update,
    frames=N_frames,
    init_func=init,
    interval=10,
    blit=True,
)

plt.tight_layout()
plt.show()