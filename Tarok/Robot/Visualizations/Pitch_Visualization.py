



import sys
import os
# Add Robot/ to path so Kinematics and Hardware can be imported directly
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))


from Robot.Kinematics.Inverse_Kinematics import*
from Robot.Kinematics.Forward_Kinematics import*
from Robot.Kinematics.Constant_Transforms import*
from Robot.Kinematics.Jacobian import*
from Robot.Kinematics.Pitch_And_Roll import*

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.widgets as widgets
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401
from math import pi

# ─────────────────────────── ROBOT PARAMETERS ────────────────────────────────
L1       = 0.078
TORSO_L  = 0.7048    # matches L_Body inside Pitch_And_Roll.py
TORSO_W  = 0.220     # matches W_Body inside Pitch_And_Roll.py
z = 0.41
 
LEG_NAMES = ['FL', 'FR', 'HL', 'HR']
COLORS    = {'FL': 'tab:blue', 'FR': 'tab:orange', 'HL': 'tab:green', 'HR': 'tab:red'}
 
# Nominal foot positions in the body frame (torso centre at origin)
NOMINAL_FEET = [
    [ TORSO_L/2,  TORSO_W/2 + L1, -z],   # FL  (idx 0)
    [ TORSO_L/2, -(TORSO_W/2 + L1), -z],  # FR  (idx 1)
    [-TORSO_L/2,  TORSO_W/2 + L1, -z],   # HL  (idx 2)
    [-TORSO_L/2, -(TORSO_W/2 + L1), -z],  # HR  (idx 3)
]
 
LEG_IDX = {leg: i for i, leg in enumerate(LEG_NAMES)}
 
# Shoulder positions in body frame (from TB_0 applied to the zero-vector)
SHOULDER_BODY = {
    leg: TB_0(np.zeros((3, 1)), leg).flatten()
    for leg in LEG_NAMES
}
 
 
# ─────────────────────────── POSE SOLVER ─────────────────────────────────────
def solve_pose(pitch_deg: float):
    """
    Given a pitch angle (degrees, positive = nose UP):
      1. Inverse_Pitch  → new foot positions in body frame
      2. T0_B           → foot in leg frame  (IK input)
      3. Inverse_Kinematics → joint angles
      4. P0_2 / P0_3 / P0_end → joint positions in leg frame
      5. TB_0           → all positions back to body frame for plotting
 
    Returns a dict  {leg: [shoulder, j2, j3, foot]}  all in body frame.
    """
    pitch_rad = pitch_deg * pi / 180.0
 
    # Inverse_Pitch sign convention: positive argument = nose DOWN → negate
    new_feet_body, _ = Inverse_Pitch(-pitch_rad,
                                     [list(f) for f in NOMINAL_FEET])
 
    chains = {}
 
    for leg in LEG_NAMES:
        i = LEG_IDX[leg]
 
        # Foot in body frame (column vector)
        foot_B = np.array(new_feet_body[i]).reshape(3, 1)
 
        # Transform foot to leg-local frame
        foot_0 = T0_B(foot_B, leg)
 
        # The fore-aft axis (index 2) in the leg frame has opposite sign
        # convention between T0_B and the IK/FK for FR and HL (config = -1).
        # Negate it so that IK and FK are consistent for those legs.
        foot_0_ik = foot_0.copy()
        if leg in ('FR', 'HL'):
            foot_0_ik[2] *= -1
 
        # Inverse kinematics → joint angles
        try:
            t1, t2, t3 = Inverse_Kinematics(foot_0_ik, leg)
        except ValueError:
            # Clamp to workspace boundary if slightly out of reach
            scale = (0.2 + 0.3 - 1e-4) / (np.linalg.norm(foot_0_ik) + 1e-9)
            t1, t2, t3 = Inverse_Kinematics(foot_0_ik * scale, leg)
 
        # Forward kinematics → joint positions in leg-local frame
        p2_0   = P0_2(t1, t2, t3, leg)
        p3_0   = P0_3(t1, t2, t3, leg)
        end_0  = P0_end(t1, t2, t3, leg)
 
        # Transform every joint position back to body frame
        shoulder_B = SHOULDER_BODY[leg]
        p2_B       = TB_0(p2_0,  leg).flatten()
        p3_B       = TB_0(p3_0,  leg).flatten()
        end_B      = TB_0(end_0, leg).flatten()
 
        chains[leg] = [shoulder_B, p2_B, p3_B, end_B]
 
    return chains
 
 
# ─────────────────────────── FIGURE SETUP ────────────────────────────────────
fig = plt.figure(figsize=(10, 8))
fig.suptitle('TAROK — Pitch Visualisation', fontsize=14, fontweight='bold')
 
ax = fig.add_subplot(1, 1, 1, projection='3d')
plt.subplots_adjust(bottom=0.18)
 
ax.set_xlim(-0.55, 0.55)
ax.set_ylim(-0.45, 0.45)
ax.set_zlim(-0.55, 0.25)
ax.set_xlabel('X — forward (m)')
ax.set_ylabel('Y — lateral (m)')
ax.set_zlabel('Z — up (m)')
ax.set_title('Body frame  ·  torso centre at origin')
ax.view_init(elev=18, azim=210)
 
# Origin dot
ax.scatter(0, 0, 0, color='black', s=40, zorder=10)
 
# ── Animated objects (one per leg + torso rectangle) ──
leg_lines  = {
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
def draw_pose(pitch_deg: float):
    chains = solve_pose(pitch_deg)
 
    for leg in LEG_NAMES:
        pts = chains[leg]
        leg_lines[leg].set_data([p[0] for p in pts],
                                 [p[1] for p in pts])
        leg_lines[leg].set_3d_properties([p[2] for p in pts])
 
    # Torso rectangle: connect the four shoulder positions in order FL→FR→HR→HL→FL
    sh = {leg: chains[leg][0] for leg in LEG_NAMES}
    order = ['FL', 'FR', 'HR', 'HL', 'FL']
    torso_line.set_data([sh[l][0] for l in order],
                         [sh[l][1] for l in order])
    torso_line.set_3d_properties([sh[l][2] for l in order])
 
    info_text.set_text(f'Pitch = {pitch_deg:+.1f}°')
    fig.canvas.draw_idle()
 
 
# ─────────────────────────── SLIDER ──────────────────────────────────────────
ax_slider = plt.axes([0.15, 0.06, 0.70, 0.04])
slider    = widgets.Slider(
    ax_slider, 'Pitch (°)', -15.0, 15.0,
    valinit=0.0, valstep=0.1, color='steelblue'
)
 
slider.on_changed(lambda val: draw_pose(slider.val))
 
# ── Initial draw at 0° ──
draw_pose(0.0)
 
plt.show()
 