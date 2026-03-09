# ─────────────────────────────────────────────────────────────────────────────
#  TAROK – Full Robot Visualizer  (V2)
#  Style: Up_Down_FT clean animation aesthetic + torso box + all 4 legs
#  Controls:
#    • Pitch / Roll sliders   – tilt the torso (feet stay fixed)
#    • Height slider          – raise / lower the torso
#    • Animate button         – play an up/down bounce animation
# ─────────────────────────────────────────────────────────────────────────────

import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.animation as animation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.widgets import Slider, Button
from math import pi

# ─────────────────────────────────────────────
#  ROBOT DIMENSIONS  (from Tarok_Dymensions.py)
# ─────────────────────────────────────────────
TORSO_LENGTH  = 0.479
TORSO_WIDTH   = 0.3179
TORSO_HEIGHT  = 0.09919
D_BODY_LENGTH = 0.3277 / 2      # half of D_BODY_LENGTH  (shoulder X offset)
D_BODY_WIDTH  = 0.07298         # shoulder Y offset from torso edge
UPPER_LEG     = 0.2             # L2  (femur)
LOWER_LEG     = 0.2             # L3  (tibia)
SHOULDER_LINK = 0.078           # L1  (abduction link)

# ── Nominal torso height so legs are at neutral (fully extended = max height) ──
TORSO_Z0     = UPPER_LEG + LOWER_LEG          # center of torso above ground
HEIGHT_MIN   = 0.15
HEIGHT_MAX   = TORSO_Z0 - 0.02
HEIGHT_INIT  = (HEIGHT_MIN + HEIGHT_MAX) * 0.55

# ─────────────────────────────────────────────
#  LEG DEFINITIONS
#  Four legs: FL, FR, RL, RR
#  shoulder_local: position of shoulder joint in torso body frame
#  sign_y: +1 = left side, -1 = right side  (controls abduction direction)
# ─────────────────────────────────────────────
LEGS = {
    'FL': dict(
        shoulder_local=np.array([ TORSO_LENGTH/2,  TORSO_WIDTH/2, -TORSO_HEIGHT/2]),
        sign_y=+1,
        color='#58a6ff',
    ),
    'FR': dict(
        shoulder_local=np.array([ TORSO_LENGTH/2, -TORSO_WIDTH/2, -TORSO_HEIGHT/2]),
        sign_y=-1,
        color='#f0883e',
    ),
    'RL': dict(
        shoulder_local=np.array([-TORSO_LENGTH/2,  TORSO_WIDTH/2, -TORSO_HEIGHT/2]),
        sign_y=+1,
        color='#7ee787',
    ),
    'RR': dict(
        shoulder_local=np.array([-TORSO_LENGTH/2, -TORSO_WIDTH/2, -TORSO_HEIGHT/2]),
        sign_y=-1,
        color='#d2a8ff',
    ),
}

# Fixed foot positions in world frame (on the ground, z = 0)
def default_foot(shoulder_local):
    """Foot directly below shoulder at ground level, plus abduction offset."""
    sx, sy, _ = shoulder_local
    # foot is offset laterally by SHOULDER_LINK from shoulder projection
    fy = sy + np.sign(sy) * SHOULDER_LINK if sy != 0 else SHOULDER_LINK
    return np.array([sx, fy, 0.0])

for name, leg in LEGS.items():
    leg['foot_fixed'] = default_foot(leg['shoulder_local'])

# ─────────────────────────────────────────────
#  ROTATION MATRICES
# ─────────────────────────────────────────────
def Rx(a):
    return np.array([[1, 0, 0],
                     [0, np.cos(a), -np.sin(a)],
                     [0, np.sin(a),  np.cos(a)]])

def Ry(a):
    return np.array([[ np.cos(a), 0, np.sin(a)],
                     [0,          1, 0         ],
                     [-np.sin(a), 0, np.cos(a)]])

# ─────────────────────────────────────────────
#  INVERSE KINEMATICS  (works for both left/right via sign_y)
# ─────────────────────────────────────────────
def FL_Inverse_Kinematics(L1, L2, L3, P, sign_y=+1):
    """
    P: foot position in shoulder local frame (3-vector).
    sign_y flips the abduction for right-side legs.
    Returns (theta1, theta2, theta3).
    """
    x, y, z = float(P[0]), float(P[1]), float(P[2])
    y *= sign_y   # mirror right-side legs

    H = np.sqrt(max(x**2 + y**2 - L1**2, 1e-9))
    theta1 = np.arctan2(y, x) - np.arctan2(L1, H)
    theta1 *= sign_y

    rsqr   = H**2 + z**2
    cos3   = np.clip((rsqr - L2**2 - L3**2) / (2 * L2 * L3), -1.0, 1.0)
    theta3 = np.arctan2(-np.sqrt(1 - cos3**2), cos3)

    beta   = np.arctan2(L3 * np.sin(theta3), L2 + L3 * np.cos(theta3))
    gamma  = np.arctan2(-z, H)
    theta2 = gamma - beta

    return theta1, theta2, theta3

# ─────────────────────────────────────────────
#  FORWARD KINEMATICS  (joint positions)
# ─────────────────────────────────────────────
def leg_joint_positions(L1, L2, L3, t1, t2, t3):
    """Returns [p0, p1, p2, p3] in shoulder local frame."""
    c1, s1  = np.cos(t1), np.sin(t1)
    c2, s2  = np.cos(t2), np.sin(t2)
    c23     = np.cos(t2 + t3)
    s23     = np.sin(t2 + t3)
    p0 = np.zeros(3)
    p1 = np.array([-s1*L1,    c1*L1,    0.0        ])
    p2 = p1 + np.array([ c1*c2*L2,  s1*c2*L2, -s2*L2 ])
    p3 = p2 + np.array([ c1*c23*L3, s1*c23*L3, -s23*L3])
    return p0, p1, p2, p3

# ─────────────────────────────────────────────
#  TORSO BOX
# ─────────────────────────────────────────────
def torso_corners(center, R):
    L, W, H = TORSO_LENGTH, TORSO_WIDTH, TORSO_HEIGHT
    local = np.array([
        [-L/2,-W/2,-H/2],[ L/2,-W/2,-H/2],
        [ L/2, W/2,-H/2],[-L/2, W/2,-H/2],
        [-L/2,-W/2, H/2],[ L/2,-W/2, H/2],
        [ L/2, W/2, H/2],[-L/2, W/2, H/2],
    ])
    return np.array([center + R @ c for c in local])

def draw_torso_box(ax, corners, alpha=0.20):
    faces_idx = [[0,1,2,3],[4,5,6,7],[0,1,5,4],[2,3,7,6],[0,3,7,4],[1,2,6,5]]
    faces = [[corners[i] for i in f] for f in faces_idx]
    box = Poly3DCollection(faces, alpha=alpha, facecolor='#1f4068',
                           edgecolor='#4a9eff', linewidth=0.8)
    ax.add_collection3d(box)

# ─────────────────────────────────────────────
#  COMPUTE FULL SCENE
# ─────────────────────────────────────────────
def compute_scene(pitch_deg, roll_deg, torso_z):
    pitch    = np.radians(pitch_deg)
    roll     = np.radians(roll_deg)
    R_torso  = Ry(pitch) @ Rx(roll)
    center   = np.array([0.0, 0.0, torso_z])
    corners  = torso_corners(center, R_torso)

    leg_pts = {}
    angles  = {}

    for name, leg in LEGS.items():
        sl      = leg['shoulder_local']
        foot_w  = leg['foot_fixed']
        sign_y  = leg['sign_y']

        shoulder_world = center + R_torso @ sl
        foot_in_sh     = R_torso.T @ (foot_w - shoulder_world)

        try:
            t1, t2, t3 = FL_Inverse_Kinematics(
                SHOULDER_LINK, UPPER_LEG, LOWER_LEG, foot_in_sh, sign_y)
        except Exception:
            t1, t2, t3 = 0.0, -pi/4, -pi/2

        p0, p1, p2, p3 = leg_joint_positions(
            SHOULDER_LINK, UPPER_LEG, LOWER_LEG, t1, t2, t3)

        # transform from shoulder local → world (p3 is already the foot via IK)
        pts_world = [shoulder_world + R_torso @ p for p in (p0, p1, p2, p3)]

        leg_pts[name] = np.array(pts_world)
        angles[name]  = (t1, t2, t3)

    return center, R_torso, corners, leg_pts, angles

# ─────────────────────────────────────────────
#  FIGURE LAYOUT
# ─────────────────────────────────────────────
fig = plt.figure(figsize=(14, 8), facecolor='#0d1117')
fig.suptitle("TAROK  –  Full Robot  (Fixed Feet / Rotating Torso)",
             color='white', fontsize=14, fontweight='bold', y=0.97)

ax      = fig.add_axes([0.03, 0.18, 0.60, 0.77], projection='3d')
ax_info = fig.add_axes([0.66, 0.28, 0.31, 0.60])

ax_pitch  = fig.add_axes([0.08, 0.125, 0.54, 0.022])
ax_roll   = fig.add_axes([0.08, 0.085, 0.54, 0.022])
ax_height = fig.add_axes([0.08, 0.045, 0.54, 0.022])
ax_btn    = fig.add_axes([0.66, 0.14,  0.10, 0.055])

for a in [ax, ax_info, ax_pitch, ax_roll, ax_height]:
    a.set_facecolor('#0d1117')
ax_info.set_facecolor('#161b22')

slider_pitch  = Slider(ax_pitch,  'Pitch (°)', -20, 20,  valinit=0,
                        color='#238636', track_color='#21262d')
slider_roll   = Slider(ax_roll,   'Roll  (°)', -20, 20,  valinit=0,
                        color='#1f6feb', track_color='#21262d')
slider_height = Slider(ax_height, 'Height (m)', HEIGHT_MIN, HEIGHT_MAX,
                        valinit=HEIGHT_INIT,
                        color='#9b59b6', track_color='#21262d')

for s in [slider_pitch, slider_roll, slider_height]:
    s.label.set_color('white')
    s.valtext.set_color('white')

btn_anim = Button(ax_btn, '▶ Animate', color='#21262d', hovercolor='#30363d')
btn_anim.label.set_color('white')

# ─────────────────────────────────────────────
#  DRAW SCENE
# ─────────────────────────────────────────────
def redraw(pitch_deg, roll_deg, torso_z):
    ax.cla()
    ax_info.cla()
    ax_info.set_xticks([])
    ax_info.set_yticks([])
    for spine in ax_info.spines.values():
        spine.set_edgecolor('#30363d')
    ax_info.set_facecolor('#161b22')

    center, R_torso, corners, leg_pts, angles = \
        compute_scene(pitch_deg, roll_deg, torso_z)

    # ── Ground plane ──────────────────────────
    gx = np.linspace(-0.35, 0.65, 2)
    gy = np.linspace(-0.35, 0.55, 2)
    xx, yy = np.meshgrid(gx, gy)
    ax.plot_surface(xx, yy, np.zeros_like(xx), alpha=0.06, color='#aaaaaa')

    # ── Torso box ─────────────────────────────
    draw_torso_box(ax, corners)

    # ── Legs ──────────────────────────────────
    for name, leg in LEGS.items():
        pts  = leg_pts[name]
        col  = leg['color']
        foot = leg['foot_fixed']

        # skeleton: shoulder → hip → knee → ankle → foot
        ax.plot(pts[:,0], pts[:,1], pts[:,2],
                'o-', color=col, linewidth=2.0, markersize=5, zorder=5)

        # fixed foot highlight
        ax.scatter(*foot, color=col, s=60, marker='s', zorder=10)

    # ── Torso origin marker ───────────────────
    ax.scatter(*center, color='white', s=30, zorder=10)

    # ── Axes style ────────────────────────────
    ax.set_xlim(-0.35, 0.65)
    ax.set_ylim(-0.35, 0.55)
    ax.set_zlim( 0.00, 0.65)
    ax.set_xlabel('X (m)', color='#555', fontsize=8)
    ax.set_ylabel('Y (m)', color='#555', fontsize=8)
    ax.set_zlabel('Z (m)', color='#555', fontsize=8)
    ax.tick_params(colors='#444', labelsize=7)
    for pane in [ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane]:
        pane.fill = False
        pane.set_edgecolor('#1c2128')
    ax.grid(True, color='#1c2128', linewidth=0.5)
    ax.view_init(elev=22, azim=-50)

    # ── Legend ────────────────────────────────
    legend_items = [mpatches.Patch(color=LEGS[n]['color'], label=n) for n in LEGS]
    legend_items.append(mpatches.Patch(color='#1f4068', label='Torso'))
    ax.legend(handles=legend_items, loc='upper left', fontsize=8,
              facecolor='#161b22', edgecolor='#30363d',
              labelcolor='white', framealpha=0.9)

    # ── Info panel ────────────────────────────
    ax_info.text(0.06, 0.97, 'Joint Angles  (°)', color='white',
                 fontsize=10, fontweight='bold', transform=ax_info.transAxes,
                 va='top')

    row_h  = 0.185
    labels = ['θ₁ Abduction', 'θ₂ Hip flex', 'θ₃ Knee']
    for col_i, (name, leg) in enumerate(LEGS.items()):
        t1, t2, t3 = angles[name]
        col = leg['color']
        cx  = 0.06 + col_i * 0.245
        ax_info.text(cx, 0.90, name, color=col, fontsize=9,
                     fontweight='bold', transform=ax_info.transAxes)
        for row_i, (lbl, val) in enumerate(zip(labels,
                                               [np.degrees(t1),
                                                np.degrees(t2),
                                                np.degrees(t3)])):
            y0 = 0.82 - row_i * row_h
            ax_info.text(cx, y0,       lbl,           color='#8b949e',
                         fontsize=7.5, transform=ax_info.transAxes)
            ax_info.text(cx, y0-0.075, f'{val:+.1f}°', color=col,
                         fontsize=9, fontweight='bold',
                         transform=ax_info.transAxes)

    ax_info.axhline(0.27, color='#30363d', linewidth=0.8, xmin=0.03, xmax=0.97)
    ax_info.text(0.06, 0.23, 'Torso Pose', color='white',
                 fontsize=10, fontweight='bold', transform=ax_info.transAxes)
    ax_info.text(0.06, 0.16, f'Pitch :  {pitch_deg:+.1f}°',
                 color='#238636', fontsize=9, transform=ax_info.transAxes)
    ax_info.text(0.06, 0.10, f'Roll  :  {roll_deg:+.1f}°',
                 color='#1f6feb', fontsize=9, transform=ax_info.transAxes)
    ax_info.text(0.06, 0.04, f'Height:  {torso_z:.3f} m',
                 color='#9b59b6', fontsize=9, transform=ax_info.transAxes)

    fig.canvas.draw_idle()


def on_slider(val=None):
    redraw(slider_pitch.val, slider_roll.val, slider_height.val)

slider_pitch.on_changed(on_slider)
slider_roll.on_changed(on_slider)
slider_height.on_changed(on_slider)

# ─────────────────────────────────────────────
#  UP/DOWN ANIMATION
# ─────────────────────────────────────────────
_anim_obj   = [None]
_animating  = [False]

def run_animation(event=None):
    if _animating[0]:
        # Stop
        if _anim_obj[0] is not None:
            _anim_obj[0].event_source.stop()
        _animating[0] = False
        btn_anim.label.set_text('▶ Animate')
        fig.canvas.draw_idle()
        return

    _animating[0] = True
    btn_anim.label.set_text('■  Stop')
    fig.canvas.draw_idle()

    N      = 120
    frames = np.arange(N)
    h_lo   = HEIGHT_MIN + 0.02
    h_hi   = HEIGHT_MAX - 0.02

    def update(frame):
        t       = frame / N
        h       = h_lo + (h_hi - h_lo) * 0.5 * (1 - np.cos(2 * pi * t))
        pitch   = slider_pitch.val
        roll    = slider_roll.val
        redraw(pitch, roll, h)
        slider_height.set_val(h)
        return []

    _anim_obj[0] = animation.FuncAnimation(
        fig, update, frames=frames, interval=30, blit=False, repeat=True)

btn_anim.on_clicked(run_animation)

# ─────────────────────────────────────────────
#  INITIAL DRAW
# ─────────────────────────────────────────────
redraw(0, 0, HEIGHT_INIT)
plt.show()