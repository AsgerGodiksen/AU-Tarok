##### Full Robot Pitch Oscillation Animation #####
# Feet are fixed on the ground (z = 0).
# The torso pitches sinusoidally — positive = nose up.
# Uses FL_Inverse_Kinematics and Inverse_Pitch from the project Kinematics folder.

import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D   # noqa: F401
from math import pi

from Kinematics.FL_Forward_Kinematics import FL_P0_2, FL_P0_3
from Kinematics.FR_Forward_Kinematics import FR_P0_2, FR_P0_3, FR_P0_end
from Kinematics.Inverse_Kinematics import FL_Inverse_Kinematics
from Kinematics.Pitch_And_Roll import Inverse_Pitch


################## PARAMETERS ######################
L1, L2, L3       = 0.078, 0.20, 0.30
TORSO_L, TORSO_W = 0.479, 0.2179
NOMINAL_Z        = -0.40
DISPLAY_SHIFT    = abs(NOMINAL_Z)   # feet at z=0, torso at ~0.40

PITCH_AMP_DEG = 6.0    # oscillation amplitude (degrees) — keep ≤ 7° for workspace
dt            = 0.05
total_time    = 8.0
t             = np.linspace(0, total_time, int(total_time / dt))

# Sinusoidal pitch trajectory  (positive = nose up)
pitch_deg_traj = PITCH_AMP_DEG * np.sin(2 * pi * t / total_time)

SHOULDERS_IK = {
    'FL': np.array([ TORSO_L/2,  TORSO_W/2, 0.0]),
    'FR': np.array([ TORSO_L/2, -TORSO_W/2, 0.0]),
    'HL': np.array([-TORSO_L/2,  TORSO_W/2, 0.0]),
    'HR': np.array([-TORSO_L/2, -TORSO_W/2, 0.0]),
}

NOMINAL_FEET = [
    [ TORSO_L/2,  TORSO_W/2 + L1, NOMINAL_Z],   # FL (idx 0)
    [ TORSO_L/2, -TORSO_W/2 - L1, NOMINAL_Z],   # FR (idx 1)
    [-TORSO_L/2,  TORSO_W/2 + L1, NOMINAL_Z],   # HL (idx 2)
    [-TORSO_L/2, -TORSO_W/2 - L1, NOMINAL_Z],   # HR (idx 3)
]

LEG_NAMES = ['FL', 'FR', 'HL', 'HR']
LEG_IDX   = {'FL': 0, 'FR': 1, 'HL': 2, 'HR': 3}
IS_LEFT   = {'FL': True,  'FR': False, 'HL': True,  'HR': False}
IS_FRONT  = {'FL': True,  'FR': True,  'HL': False, 'HR': False}
COLORS    = {'FL': 'tab:blue', 'FR': 'tab:orange', 'HL': 'tab:green', 'HR': 'tab:red'}


################## FK HELPERS ######################
def FL_P0_end_fixed(t1, t2, t3):
    """FL end-effector with corrected L1 sign."""
    return np.array([
        [ np.cos(t1)*np.cos(t2+t3)*L3 + np.cos(t1)*np.cos(t2)*L2 - np.sin(t1)*L1],
        [ np.sin(t1)*np.cos(t2+t3)*L3 + np.sin(t1)*np.cos(t2)*L2 + np.cos(t1)*L1],
        [-np.sin(t2+t3)*L3 - np.sin(t2)*L2]
    ])

def to_world(p_local):
    """FK local -> world.  local:[ext, abd, fore-aft] -> world:[X, Y, Z]"""
    return np.array([p_local[2], p_local[1], -p_local[0]])

def get_chain(t1, t2, t3, shoulder_display, is_left, is_front):
    """Return [shoulder, J1, J2, foot] in display frame."""
    # Inward knee: front s=+1 (apex backward), hind s=-1 (apex forward)
    s = +1 if is_front else -1
    _t2, _t3 = s*t2, s*t3

    if is_left:
        p2  = FL_P0_2(t1,  _t2,  _t3).flatten()
        p3  = FL_P0_3(t1,  _t2,  _t3).flatten()
        end = FL_P0_end_fixed(t1, _t2, _t3).flatten()
    else:
        p2  = FR_P0_2(t1, -_t2, -_t3).flatten()
        p3  = FR_P0_3(t1, -_t2, -_t3).flatten()
        end = FR_P0_end(t1, -_t2, -_t3).flatten()

    return [shoulder_display,
            shoulder_display + to_world(p2),
            shoulder_display + to_world(p3),
            shoulder_display + to_world(end)]


################## SOLVE IK FOR ALL FRAMES ######################
# Pre-compute angles and shoulder positions for every timestep
all_angles       = []   # list of dicts {leg: (t1,t2,t3)}
all_shoulder_disp = []  # list of dicts {leg: np.array}

for pitch_deg in pitch_deg_traj:
    # Inverse_Pitch: positive = nose DOWN → negate for nose-UP convention
    new_feet, sh_heights = Inverse_Pitch(-pitch_deg * pi / 180.0,
                                         [list(f) for f in NOMINAL_FEET])
    frame_angles = {}
    frame_shoulders = {}

    for leg in LEG_NAMES:
        sh_nom = SHOULDERS_IK[leg]
        i      = LEG_IDX[leg]

        # Shoulder display position: x/y fixed, z from sh_heights (vertical drop to foot)
        sh_display = np.array([sh_nom[0], sh_nom[1], sh_heights[i]])
        frame_shoulders[leg] = sh_display

        # Feet are FIXED on the ground — use nominal foot x, not Inverse_Pitch's slid value
        foot_x = NOMINAL_FEET[i][0]   # fixed world X
        foot_y = NOMINAL_FEET[i][1]   # fixed world Y

        # IK inputs in leg-local frame:
        #   extension  = vertical drop from shoulder to foot
        #   abduction  = lateral distance (always L1)
        #   fore-aft   = fixed foot X minus rotated shoulder X
        ik_x = sh_heights[i]
        ik_y = abs(foot_y - sh_nom[1])
        ik_z = foot_x - sh_nom[0]
        P = np.array([[ik_x], [ik_y], [ik_z]])

        try:
            a1, a2, a3 = FL_Inverse_Kinematics(L1, L2, L3, P)
        except ValueError:
            scale = (L2 + L3 - 1e-4) / np.linalg.norm([ik_x, ik_y, ik_z])
            a1, a2, a3 = FL_Inverse_Kinematics(L1, L2, L3, P * scale)

        frame_angles[leg] = (a1, a2, a3)

    all_angles.append(frame_angles)
    all_shoulder_disp.append(frame_shoulders)

# Collect per-joint angle time series for the plots (FL representative)
theta1 = np.array([all_angles[i]['FL'][0] for i in range(len(t))])
theta2 = np.array([all_angles[i]['FL'][1] for i in range(len(t))])
theta3 = np.array([all_angles[i]['FL'][2] for i in range(len(t))])


################## FIGURE SETUP ######################
fig = plt.figure(figsize=(17, 9))
fig.suptitle('TAROK — Pitch Oscillation Simulation', fontsize=14, fontweight='bold')

ax3d = fig.add_subplot(1, 2, 1, projection='3d')
ax3d.set_xlim(-0.55, 0.55)
ax3d.set_ylim(-0.45, 0.45)
ax3d.set_zlim(-0.05, 0.65)
ax3d.set_xlabel('X (m)'); ax3d.set_ylabel('Y (m)'); ax3d.set_zlabel('Z (m)')
ax3d.set_title('3D View')
ax3d.view_init(elev=18, azim=210)

# Ground rectangle
gx = [ TORSO_L/2,  TORSO_L/2, -TORSO_L/2, -TORSO_L/2,  TORSO_L/2]
gy = [ TORSO_W/2, -TORSO_W/2, -TORSO_W/2,  TORSO_W/2,  TORSO_W/2]
ax3d.plot(gx, gy, [0]*5, color='saddlebrown', lw=1, ls='--', alpha=0.4)

# Data subplots (right column)
ax_ang = fig.add_subplot(3, 2, 2)
ax_vel = fig.add_subplot(3, 2, 4)
ax_pitch = fig.add_subplot(3, 2, 6)
plt.subplots_adjust(left=0.05, right=0.97, top=0.92, bottom=0.07,
                    wspace=0.30, hspace=0.55)

JNT_COL = ['tab:blue', 'tab:orange', 'tab:green']

# Static joint angle curves
for j, (col, lbl) in enumerate(zip(JNT_COL, ['θ₁ Hip', 'θ₂ Thigh', 'θ₃ Knee'])):
    ax_ang.plot(t, np.rad2deg([theta1, theta2, theta3][j]), color=col, lw=1.3, label=lbl)

# Joint velocity (finite difference)
dt_arr = t[1] - t[0]
for j, (col, lbl) in enumerate(zip(JNT_COL, ['θ₁ Hip', 'θ₂ Thigh', 'θ₃ Knee'])):
    vel = np.gradient([theta1, theta2, theta3][j], dt_arr)
    ax_vel.plot(t, np.rad2deg(vel), color=col, lw=1.3, label=lbl)

# Pitch trajectory
ax_pitch.plot(t, pitch_deg_traj, color='black', lw=1.5, label='Pitch (deg)')
ax_pitch.axhline(0, color='gray', lw=0.8, ls='--')

for ax, title, ylabel in [
        (ax_ang,   'FL Joint Angles',     'deg'),
        (ax_vel,   'FL Joint Velocities', 'deg/s'),
        (ax_pitch, 'Pitch Angle',         'deg')]:
    ax.set_title(title); ax.set_xlabel('Time (s)'); ax.set_ylabel(ylabel)
    ax.grid(True, ls='--', alpha=0.5); ax.legend(fontsize=8, loc='upper right')

# Time cursor lines
vline_ang   = ax_ang.axvline(0,   color='gray', lw=1, ls='--')
vline_vel   = ax_vel.axvline(0,   color='gray', lw=1, ls='--')
vline_pitch = ax_pitch.axvline(0, color='gray', lw=1, ls='--')

# Animated objects
leg_lines   = {leg: ax3d.plot([], [], [], 'o-', color=COLORS[leg], lw=2, ms=5, label=leg)[0]
               for leg in LEG_NAMES}
torso_line  = ax3d.plot([], [], [], 'k-', lw=2.5, alpha=0.7)[0]

# Shoulder markers (static reference)
for leg, sh in SHOULDERS_IK.items():
    ax3d.scatter(*sh + np.array([0, 0, DISPLAY_SHIFT]),
                 color=COLORS[leg], s=20, zorder=5, alpha=0.3)

ax3d.legend(loc='upper right', fontsize=8)

time_text = ax3d.text2D(0.02, 0.97, '', transform=ax3d.transAxes, fontsize=8,
                        verticalalignment='top',
                        bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))


################## ANIMATION ######################
def init():
    for obj in list(leg_lines.values()) + [torso_line]:
        obj.set_data([], [])
        obj.set_3d_properties([])
    time_text.set_text('')
    return []


def update(num):
    frame_angles    = all_angles[num]
    frame_shoulders = all_shoulder_disp[num]
    current_t       = t[num]
    pitch_now       = pitch_deg_traj[num]

    # Draw legs
    for leg in LEG_NAMES:
        a1, a2, a3 = frame_angles[leg]
        chain = get_chain(a1, a2, a3, frame_shoulders[leg], IS_LEFT[leg], IS_FRONT[leg])
        xs = [p[0] for p in chain]
        ys = [p[1] for p in chain]
        zs = [p[2] for p in chain]
        leg_lines[leg].set_data(xs, ys)
        leg_lines[leg].set_3d_properties(zs)

    # Draw torso rectangle (connect all 4 shoulders)
    fl = frame_shoulders['FL']; fr = frame_shoulders['FR']
    hl = frame_shoulders['HL']; hr = frame_shoulders['HR']
    tx = [fl[0], fr[0], hr[0], hl[0], fl[0]]
    ty = [fl[1], fr[1], hr[1], hl[1], fl[1]]
    tz = [fl[2], fr[2], hr[2], hl[2], fl[2]]
    torso_line.set_data(tx, ty)
    torso_line.set_3d_properties(tz)

    # Update time cursors
    for vl in [vline_ang, vline_vel, vline_pitch]:
        vl.set_xdata([current_t, current_t])

    time_text.set_text(f"t = {current_t:.2f} s\n"
                       f"Pitch = {pitch_now:+.1f}°\n"
                       f"θ₁ = {np.rad2deg(frame_angles['FL'][0]):+.1f}°\n"
                       f"θ₂ = {np.rad2deg(frame_angles['FL'][1]):+.1f}°\n"
                       f"θ₃ = {np.rad2deg(frame_angles['FL'][2]):+.1f}°")
    return []


ani = animation.FuncAnimation(fig, update, frames=len(t),
                               init_func=init, interval=50,
                               blit=False, repeat=True)
plt.show()