##### Roll Visualisation — Interactive Slider #####
#
# Conventions:
#   Body frame  : X = forward, Y = left, Z = up.  Origin at torso centre.
#   Leg frame   : x = down (extension), y = lateral, z = fore-aft.
#   Mapping is handled by TB_0 / T0_B from Constant_Transforms.py.
#
# Positive roll = lean right  (left side rises, right side drops).
#
# Usage:
#   Place this file in the same folder as:
#       Constant_Transforms.py
#       Forward_Kinematics.py
#       Inverse_Kinematics.py
#       Pitch_And_Roll.py
#   then run:  python Simulation_Roll_Slider.py

import sys, os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.widgets as widgets
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401
from math import pi, sin, cos, atan
import io, contextlib

from Robot.Kinematics.Inverse_Kinematics import*
from Robot.Kinematics.Forward_Kinematics import*
from Robot.Kinematics.Constant_Transforms import*
from Robot.Kinematics.Jacobian import*
from Robot.Kinematics.Pitch_And_Roll import*

# ─────────────────────────── ROBOT PARAMETERS ────────────────────────────────
L1      = 0.078
TORSO_L = 0.7048    # matches L_Body inside Pitch_And_Roll.py
TORSO_W = 0.220     # matches W_Body inside Pitch_And_Roll.py
W_HIP   = TORSO_W / 2   # lateral distance from centre to shoulder = 0.110
 
LEG_NAMES = ['FL', 'FR', 'HL', 'HR']
COLORS    = {'FL': 'tab:blue', 'FR': 'tab:orange', 'HL': 'tab:green', 'HR': 'tab:red'}
 
# Nominal foot positions in the body frame (torso centre at origin, left = +Y)
NOMINAL_FEET = [
    [ TORSO_L/2,  W_HIP + L1, -0.40],   # FL  (idx 0)
    [ TORSO_L/2, -(W_HIP + L1), -0.40], # FR  (idx 1)
    [-TORSO_L/2,  W_HIP + L1, -0.40],   # HL  (idx 2)
    [-TORSO_L/2, -(W_HIP + L1), -0.40], # HR  (idx 3)
]
 
LEG_IDX = {leg: i for i, leg in enumerate(LEG_NAMES)}
 
# Shoulder positions in body frame (from TB_0 applied to the zero-vector)
SHOULDER_BODY = {
    leg: TB_0(np.zeros((3, 1)), leg).flatten()
    for leg in LEG_NAMES
}
 
# Inverse_Kinematics / Forward_Kinematics for FR and HL (config = -1) are
# inconsistent when theta1 != 0 (which occurs during roll).  Fix: mirror FR
# through FL and HL through HR (both config = +1), solve there, then negate
# the y coordinates of every joint to reflect back to the correct side.
MIRROR_LEG = {'FL': 'FL', 'FR': 'FL', 'HL': 'HR', 'HR': 'HR'}
 
 
# ─────────────────────────── FOOT POSITIONS ──────────────────────────────────
def _get_roll_feet(roll_rad: float):
    """
    Call Inverse_Roll from Pitch_And_Roll.py to compute foot positions.
 
    Inverse_Roll was designed with shoulder reference at ±W_Body (= 0.220),
    while our body frame places shoulders at ±W_HIP (= 0.110).  We bridge
    this by shifting the feet ±W_HIP in y before the call and reversing
    afterward (including correcting Inverse_Roll's output y-sign convention,
    which is opposite to ours).
 
    Returns a list of 4 foot positions [[x, y, z], ...] in our body frame.
    """
    # Zero-pitch shoulder heights (all equal to 0.40 at nominal stance)
    with contextlib.redirect_stdout(io.StringIO()):
        _, sh_heights = Inverse_Pitch(0.0, [list(f) for f in NOMINAL_FEET])
 
    # Shift feet to Inverse_Roll's expected frame (shoulder at ±W_Body)
    shifted = [
        [f[0], f[1] + (W_HIP if (i == 0 or i == 2) else -W_HIP), f[2]]
        for i, f in enumerate(NOMINAL_FEET)
    ]
 
    # Call Inverse_Roll (suppressing its debug prints)
    with contextlib.redirect_stdout(io.StringIO()):
        raw = Inverse_Roll(roll_rad, shifted, sh_heights)
 
    # Convert output back to our frame:
    #   Inverse_Roll output uses left = -Y, we use left = +Y  => negate y
    #   then unshift the W_HIP offset
    result = []
    for i, f in enumerate(raw):
        is_left = (i == 0 or i == 2)
        y_back = -f[1] - (W_HIP if is_left else -W_HIP)
        result.append([f[0], y_back, f[2]])
 
    return result
 
 
# ─────────────────────────── POSE SOLVER ─────────────────────────────────────
def solve_pose(roll_deg: float):
    """
    Given a roll angle (degrees, positive = lean left):
      1. _get_roll_feet   -> foot positions in body frame  (via Inverse_Roll)
      2. Mirror FR->FL and HL->HR; apply T0_B; run Inverse_Kinematics
      3. P0_2 / P0_3 / P0_end -> joint positions in mirror-leg frame
      4. TB_0 -> body frame; un-mirror y for FR / HL
 
    Returns {leg: [shoulder, j2, j3, foot]} -- all points in body frame.
    """
    roll_rad = roll_deg * pi / 180.0
    new_feet = _get_roll_feet(roll_rad)
 
    chains = {}
 
    for leg in LEG_NAMES:
        i      = LEG_IDX[leg]
        mirror = MIRROR_LEG[leg]
        needs_mirror = (leg != mirror)
 
        # Mirror y for FR / HL before transforming to leg frame
        foot_B   = np.array(new_feet[i])
        foot_B_m = foot_B.copy()
        if needs_mirror:
            foot_B_m[1] *= -1
 
        # Body frame -> mirror-leg frame
        foot_0    = T0_B(foot_B_m.reshape(3, 1), mirror)
 
        # Fore-aft sign fix (applies if mirror is FR or HL, which never happens
        # with the current MIRROR_LEG map, but kept for safety)
        foot_0_ik = foot_0.copy()
        if mirror in ('FR', 'HL'):
            foot_0_ik[2] *= -1
 
        # Inverse kinematics
        try:
            t1, t2, t3 = Inverse_Kinematics(foot_0_ik, mirror)
        except ValueError:
            scale = (0.2 + 0.3 - 1e-4) / (np.linalg.norm(foot_0_ik) + 1e-9)
            t1, t2, t3 = Inverse_Kinematics(foot_0_ik * scale, mirror)
 
        # Forward kinematics -> body frame
        p2_B  = TB_0(P0_2(t1,  t2, t3, mirror), mirror).flatten()
        p3_B  = TB_0(P0_3(t1,  t2, t3, mirror), mirror).flatten()
        end_B = TB_0(P0_end(t1, t2, t3, mirror), mirror).flatten()
 
        # Un-mirror y for FR / HL
        if needs_mirror:
            p2_B[1]  *= -1
            p3_B[1]  *= -1
            end_B[1] *= -1
 
        chains[leg] = [SHOULDER_BODY[leg], p2_B, p3_B, end_B]
 
    return chains
 
 
# ─────────────────────────── FIGURE SETUP ────────────────────────────────────
fig = plt.figure(figsize=(10, 8))
fig.suptitle('TAROK — Roll Visualisation', fontsize=14, fontweight='bold')
 
ax = fig.add_subplot(1, 1, 1, projection='3d')
plt.subplots_adjust(bottom=0.18)
 
ax.set_xlim(-0.55, 0.55)
ax.set_ylim(-0.45, 0.45)
ax.set_zlim(-0.55, 0.25)
ax.set_xlabel('X — forward (m)')
ax.set_ylabel('Y — lateral (m)')
ax.set_zlabel('Z — up (m)')
ax.set_title('Body frame  |  torso centre at origin  |  +roll = lean left')
ax.view_init(elev=18, azim=210)
 
ax.scatter(0, 0, 0, color='black', s=40, zorder=10)
 
leg_lines = {
    leg: ax.plot([], [], [], 'o-', color=COLORS[leg], lw=2.5, ms=6, label=leg)[0]
    for leg in LEG_NAMES
}
torso_line = ax.plot([], [], [], 'k-', lw=3, alpha=0.85)[0]
 
ax.legend(loc='upper right', fontsize=9)
 
info_text = ax.text2D(
    0.02, 0.97, '',
    transform=ax.transAxes, fontsize=9,
    verticalalignment='top',
    bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.6)
)
 
 
# ─────────────────────────── DRAW FUNCTION ───────────────────────────────────
def draw_pose(roll_deg: float):
    chains = solve_pose(roll_deg)
 
    for leg in LEG_NAMES:
        pts = chains[leg]
        leg_lines[leg].set_data([p[0] for p in pts],
                                  [p[1] for p in pts])
        leg_lines[leg].set_3d_properties([p[2] for p in pts])
 
    # Torso rectangle: FL -> FR -> HR -> HL -> FL
    sh    = {leg: chains[leg][0] for leg in LEG_NAMES}
    order = ['FL', 'FR', 'HR', 'HL', 'FL']
    torso_line.set_data([sh[l][0] for l in order],
                         [sh[l][1] for l in order])
    torso_line.set_3d_properties([sh[l][2] for l in order])
 
    info_text.set_text(f'Roll = {roll_deg:+.1f}°')
    fig.canvas.draw_idle()
 
 
# ─────────────────────────── SLIDER ──────────────────────────────────────────
ax_slider = plt.axes([0.15, 0.06, 0.70, 0.04])
slider    = widgets.Slider(
    ax_slider, 'Roll (°)', -10.0, 10.0,
    valinit=0.0, valstep=0.1, color='steelblue'
)
 
slider.on_changed(lambda val: draw_pose(slider.val))
 
draw_pose(0.0)
 
try:
    plt.show()
except KeyboardInterrupt:
    print("\nStopping visualisation...")
finally:
    plt.close('all')
    print("Done.")
 