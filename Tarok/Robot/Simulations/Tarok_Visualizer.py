# Tarok Robot - Torso & Front Left Leg Visualizer
# The FOOT stays fixed on the ground.
# The TORSO rotates with pitch and roll, and IK drives the leg joints.

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.widgets import Slider
from math import pi

# ─────────────────────────────────────────────
#  ROBOT DIMENSIONS
# ─────────────────────────────────────────────
TORSO_LENGTH  = 0.479
TORSO_WIDTH   = 0.3179
TORSO_HEIGHT  = 0.09919
D_BODY_LENGTH = 0.3277      # half-length
D_BODY_WIDTH  = 0.07298     # half-width
UPPER_LEG     = 0.2         # L2
LOWER_LEG     = 0.2         # L3
SHOULDER_LINK = 0.078       # L1

# Neutral torso height (center of torso above ground)
TORSO_Z0 = UPPER_LEG + LOWER_LEG + TORSO_HEIGHT / 2

# Fixed foot position in world frame (FL leg, stays here always)
FOOT_FIXED = np.array([
     D_BODY_LENGTH,
     D_BODY_WIDTH + SHOULDER_LINK,
     0.0   # on the ground
])

# ─────────────────────────────────────────────
#  ROTATION MATRICES
# ─────────────────────────────────────────────
def Rx(a):
    """Rotation about X (roll)."""
    return np.array([
        [1,          0,           0],
        [0,  np.cos(a), -np.sin(a)],
        [0,  np.sin(a),  np.cos(a)]
    ])

def Ry(a):
    """Rotation about Y (pitch)."""
    return np.array([
        [ np.cos(a), 0, np.sin(a)],
        [0,          1,         0],
        [-np.sin(a), 0, np.cos(a)]
    ])

# ─────────────────────────────────────────────
#  INVERSE KINEMATICS  (FL leg)
# ─────────────────────────────────────────────
def FL_Inverse_Kinematics(L1, L2, L3, P):
    """P is a 3-vector: foot position in shoulder local frame."""
    x, y, z = float(P[0]), float(P[1]), float(P[2])

    H  = np.sqrt(max(x**2 + y**2 - L1**2, 1e-9))
    theta1 = np.arctan2(y, x) - np.arctan2(L1, H)

    rsqr   = H**2 + z**2
    cos3   = np.clip((rsqr - L2**2 - L3**2) / (2 * L2 * L3), -1.0, 1.0)
    theta3 = np.arctan2(-np.sqrt(1 - cos3**2), cos3)

    beta   = np.arctan2(L3 * np.sin(theta3), L2 + L3 * np.cos(theta3))
    gamma  = np.arctan2(-z, H)
    theta2 = gamma - beta

    return theta1, theta2, theta3

# ─────────────────────────────────────────────
#  TORSO BOX
# ─────────────────────────────────────────────
def torso_corners(center, R):
    L, W, H = TORSO_LENGTH, TORSO_WIDTH, TORSO_HEIGHT
    local = np.array([
        [-L/2, -W/2, -H/2], [ L/2, -W/2, -H/2],
        [ L/2,  W/2, -H/2], [-L/2,  W/2, -H/2],
        [-L/2, -W/2,  H/2], [ L/2, -W/2,  H/2],
        [ L/2,  W/2,  H/2], [-L/2,  W/2,  H/2],
    ])
    return np.array([center + R @ c for c in local])

def draw_torso_box(ax, corners):
    faces_idx = [[0,1,2,3],[4,5,6,7],[0,1,5,4],[2,3,7,6],[0,3,7,4],[1,2,6,5]]
    faces = [[corners[i] for i in f] for f in faces_idx]
    box = Poly3DCollection(faces, alpha=0.22, facecolor='steelblue',
                           edgecolor='#4a9eff', linewidth=0.7)
    ax.add_collection3d(box)

# ─────────────────────────────────────────────
#  DRAW REFERENCE FRAME
# ─────────────────────────────────────────────
def draw_frame(ax, origin, R, scale=0.045, label=None, label_color='white'):
    for j, color in enumerate(['#ff4444', '#44ff44', '#4488ff']):
        d = R[:, j] * scale
        ax.quiver(origin[0], origin[1], origin[2],
                  d[0], d[1], d[2],
                  color=color, linewidth=1.6, arrow_length_ratio=0.35)
    if label:
        ax.text(origin[0], origin[1], origin[2] + scale * 1.2,
                label, color=label_color, fontsize=8, fontweight='bold')

# ─────────────────────────────────────────────
#  COMPUTE FULL SCENE
# ─────────────────────────────────────────────
def compute_scene(pitch_deg, roll_deg):
    pitch = pitch_deg * pi / 180
    roll  = roll_deg  * pi / 180
    L1, L2, L3 = SHOULDER_LINK, UPPER_LEG, LOWER_LEG

    # Torso rotation (pitch around Y, roll around X)
    R_torso  = Ry(pitch) @ Rx(roll)
    torso_center = np.array([0.0, 0.0, TORSO_Z0])

    # FL shoulder position: front-right corner of torso box in local frame
    shoulder_local = np.array([ TORSO_LENGTH/2, TORSO_WIDTH/2, -TORSO_HEIGHT/2])
    shoulder_world = torso_center + R_torso @ shoulder_local

    # Foot is FIXED — express it in shoulder (torso) local frame for IK
    foot_in_shoulder = R_torso.T @ (FOOT_FIXED - shoulder_world)

    try:
        t1, t2, t3 = FL_Inverse_Kinematics(L1, L2, L3, foot_in_shoulder)
    except Exception:
        t1, t2, t3 = 0.0, -pi/4, -pi/2

    # ── Joint positions via FK (in shoulder local frame, then to world) ──
    c1, s1  = np.cos(t1), np.sin(t1)
    c2, s2  = np.cos(t2), np.sin(t2)
    c23     = np.cos(t2 + t3)
    s23     = np.sin(t2 + t3)

    # Local joint frames (relative to shoulder)
    R0 = np.eye(3)
    R1 = np.array([[ c1, 0,-s1],[ s1, 0, c1],[  0,-1,  0]])
    R2 = np.array([[ c1*c2,-c1*s2,-s1],[ s1*c2,-s1*s2, c1],[-s2,-c2,  0]])
    R3 = np.array([[ c1*c23,-c1*s23,-s1],[ s1*c23,-s1*s23, c1],[-s23,-c23, 0]])

    p0 = np.zeros(3)
    p1 = p0 + np.array([-s1*L1,  c1*L1,   0.0   ])
    p2 = p1 + np.array([ c1*c2*L2, s1*c2*L2, -s2*L2])
    p3 = p2 + np.array([ c1*c23*L3, s1*c23*L3, -s23*L3])

    def w(p_loc): return shoulder_world + R_torso @ p_loc
    def rw(R_loc): return R_torso @ R_loc

    joints = {
        'Shoulder': (w(p0), rw(R0)),
        'Hip':      (w(p1), rw(R1)),
        'Knee':     (w(p2), rw(R2)),
        'Foot':     (w(p3), rw(R3)),
    }

    corners = torso_corners(torso_center, R_torso)
    return joints, (t1, t2, t3), torso_center, R_torso, corners

# ─────────────────────────────────────────────
#  FIGURE SETUP
# ─────────────────────────────────────────────
fig = plt.figure(figsize=(13, 8), facecolor='#0d1117')
fig.suptitle("TAROK  –  Fixed Foot  /  Rotating Torso  (FL Leg)",
             color='white', fontsize=14, fontweight='bold', y=0.97)

ax      = fig.add_axes([0.03, 0.18, 0.62, 0.77], projection='3d')
ax_info = fig.add_axes([0.68, 0.28, 0.29, 0.58])
ax_pitch = fig.add_axes([0.08, 0.10, 0.56, 0.025])
ax_roll  = fig.add_axes([0.08, 0.05, 0.56, 0.025])

ax.set_facecolor('#0d1117')
ax_info.set_facecolor('#161b22')
for a in [ax_pitch, ax_roll]:
    a.set_facecolor('#161b22')

slider_pitch = Slider(ax_pitch, 'Pitch (°)', -20, 20, valinit=0,
                      color='#238636', track_color='#21262d')
slider_roll  = Slider(ax_roll,  'Roll  (°)', -20, 20, valinit=0,
                      color='#1f6feb', track_color='#21262d')
for s in [slider_pitch, slider_roll]:
    s.label.set_color('white')
    s.valtext.set_color('white')

# ─────────────────────────────────────────────
#  REDRAW
# ─────────────────────────────────────────────
def redraw(val=None):
    ax.cla()
    ax_info.cla()
    ax_info.set_xticks([]); ax_info.set_yticks([])
    for spine in ax_info.spines.values():
        spine.set_edgecolor('#30363d')

    pitch_deg = slider_pitch.val
    roll_deg  = slider_roll.val

    joints, (t1, t2, t3), torso_center, R_torso, corners = \
        compute_scene(pitch_deg, roll_deg)

    # Ground plane
    gs = 0.5
    xx, yy = np.meshgrid(np.linspace(-0.1, gs, 2), np.linspace(-0.05, gs, 2))
    ax.plot_surface(xx, yy, np.zeros_like(xx), alpha=0.07, color='#aaaaaa')

    # Fixed foot marker
    ax.scatter(*FOOT_FIXED, color='#ff7b72', s=100, zorder=10)
    ax.text(FOOT_FIXED[0]+0.01, FOOT_FIXED[1]+0.01, FOOT_FIXED[2]+0.025,
            'Foot (fixed)', color='#ff7b72', fontsize=8)

    # Torso box
    draw_torso_box(ax, corners)

    # Torso centre frame
    draw_frame(ax, torso_center, R_torso, scale=0.07,
               label='Torso', label_color='white')

    # Leg skeleton
    pts = np.array([joints[k][0] for k in ['Shoulder','Hip','Knee','Foot']])
    ax.plot(pts[:,0], pts[:,1], pts[:,2],
            '-o', color='#58a6ff', linewidth=2.2, markersize=6, zorder=5)

    # Joint frames
    joint_colors = {
        'Shoulder': '#f0883e',
        'Hip':      '#d2a8ff',
        'Knee':     '#7ee787',
        'Foot':     '#ff7b72',
    }
    for name, (origin, R) in joints.items():
        draw_frame(ax, origin, R, scale=0.04,
                   label=name, label_color=joint_colors[name])

    # Axes style
    ax.set_xlim(-0.15, 0.52)
    ax.set_ylim(-0.05, 0.52)
    ax.set_zlim( 0.00, 0.72)
    ax.set_xlabel('X (m)', color='#666', fontsize=8)
    ax.set_ylabel('Y (m)', color='#666', fontsize=8)
    ax.set_zlabel('Z (m)', color='#666', fontsize=8)
    ax.tick_params(colors='#444', labelsize=7)
    for pane in [ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane]:
        pane.fill = False
        pane.set_edgecolor('#1c2128')
    ax.grid(True, color='#1c2128', linewidth=0.5)

    # Legend
    legend_items = [
        mpatches.Patch(color='#ff4444',  label='X axis'),
        mpatches.Patch(color='#44ff44',  label='Y axis'),
        mpatches.Patch(color='#4488ff',  label='Z axis'),
        mpatches.Patch(color='#58a6ff',  label='FL Leg'),
        mpatches.Patch(color='steelblue',label='Torso'),
    ]
    ax.legend(handles=legend_items, loc='upper left', fontsize=7,
              facecolor='#161b22', edgecolor='#30363d',
              labelcolor='white', framealpha=0.85)

    # Info panel
    ax_info.set_facecolor('#161b22')
    ax_info.text(0.08, 0.95, 'Joint Angles', color='white',
                 fontsize=10, fontweight='bold', transform=ax_info.transAxes)

    info_lines = [
        ('θ₁  Abduction', t1 * 180/pi, '#f0883e'),
        ('θ₂  Hip flex',  t2 * 180/pi, '#d2a8ff'),
        ('θ₃  Knee',      t3 * 180/pi, '#7ee787'),
    ]
    for k, (label, val, color) in enumerate(info_lines):
        yp = 0.80 - k * 0.16
        ax_info.text(0.08, yp,       label,         color='#8b949e',
                     fontsize=9, transform=ax_info.transAxes)
        ax_info.text(0.08, yp-0.08, f'{val:+.2f}°', color=color,
                     fontsize=13, fontweight='bold', transform=ax_info.transAxes)

    ax_info.axhline(0.31, color='#30363d', linewidth=0.8, xmin=0.05, xmax=0.95)
    ax_info.text(0.08, 0.27, 'Torso Rotation', color='white',
                 fontsize=10, fontweight='bold', transform=ax_info.transAxes)
    ax_info.text(0.08, 0.18, f'Pitch:  {pitch_deg:+.1f}°', color='#238636',
                 fontsize=10, transform=ax_info.transAxes)
    ax_info.text(0.08, 0.09, f'Roll:   {roll_deg:+.1f}°',  color='#1f6feb',
                 fontsize=10, transform=ax_info.transAxes)

    sw = joints['Shoulder'][0]
    ax_info.text(0.08, 0.02,
                 f'Shoulder ({sw[0]:.3f}, {sw[1]:.3f}, {sw[2]:.3f})',
                 color='#555', fontsize=7, transform=ax_info.transAxes)

    fig.canvas.draw_idle()


slider_pitch.on_changed(redraw)
slider_roll.on_changed(redraw)
redraw()

plt.show()