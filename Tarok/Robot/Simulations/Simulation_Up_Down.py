##### Full Robot Up/Down Simulation #####
# All four legs performing synchronised up/down motion
# Range: max height 46 cm, min height 34 cm
import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401
import time

from Kinematics.FL_Forward_Kinematics import*
from Kinematics.FR_Forward_Kinematics import*
from Kinematics.Inverse_Kinematics import FL_Inverse_Kinematics
from Kinematics.Jacobians import FL_Jacobian

print("Loaded from package.")

################## BUG FIX ######################
# FL_P0_end in FL_Forward_Kinematics.py has a sign error on the L1 (0.078) term
# in the Y component: should be +cos(t1)*L1, not -cos(t1)*L1.
# Confirmed by comparing with FL_Forward_Kinematics (the full T matrix function).
def FL_P0_end(theta1, theta2, theta3):
    return np.array([
        [ np.cos(theta1)*np.cos(theta2+theta3)*0.3 + np.cos(theta1)*np.cos(theta2)*0.2 - np.sin(theta1)*0.078],
        [ np.sin(theta1)*np.cos(theta2+theta3)*0.3 + np.sin(theta1)*np.cos(theta2)*0.2 + np.cos(theta1)*0.078],
        [-np.sin(theta2+theta3)*0.3 - np.sin(theta2)*0.2]
    ])


################## PARAMETERS ######################
# Link lengths
L1 = 0.078  # Hip abduction offset (m)
L2 = 0.20   # Upper leg (m)
L3 = 0.30   # Lower leg (m)

# Shoulder positions in world frame [x, y, z]
# Front = +X, Left = +Y, Right = -Y
TORSO_L = 0.479   # Torso length (m)
TORSO_W = 0.2179  # Torso width  (m)
SHOULDERS = {
    'FL': np.array([ TORSO_L/2,  TORSO_W/2, 0.0]),
    'FR': np.array([ TORSO_L/2, -TORSO_W/2, 0.0]),
    'HL': np.array([-TORSO_L/2,  TORSO_W/2, 0.0]),
    'HR': np.array([-TORSO_L/2, -TORSO_W/2, 0.0]),
}

# Time series
dt         = 0.1
total_time = 10.0
t          = np.linspace(0, total_time, int(total_time / dt))

# Foot trajectory in leg-local frame — x sweeps 0.46 → 0.34 → 0.46 m
x = np.piecewise(t, [t < 5, t >= 5],
                 [lambda t: 0.46 - (0.12/5)*t,
                  lambda t: 0.34 + (0.12/5)*(t-5)])
y = L1 * np.ones_like(t)   # Abduction offset — constant
z = np.zeros_like(t)

# Foot velocity
x_dot = np.piecewise(t, [t < 5, t >= 5],
                     [lambda t: -0.12/5 * np.ones_like(t),
                      lambda t:  0.12/5 * np.ones_like(t)])
y_dot = np.zeros_like(t)
z_dot = np.zeros_like(t)

# Validation load: 3 actuators (565 g each) + 0.5 kg plastic
F_x = -np.ones_like(t) * (3*0.565 + 0.5) * 9.82
F   = np.vstack((F_x, np.zeros_like(t), np.zeros_like(t)))


################## SOLVE IK + DYNAMICS ######################
# Joint angles via inverse kinematics
theta1 = np.zeros_like(t)
theta2 = np.zeros_like(t)
theta3 = np.zeros_like(t)
for i in range(len(t)):
    P = np.array([[x[i]], [y[i]], [z[i]]])
    theta1[i], theta2[i], theta3[i] = FL_Inverse_Kinematics(L1, L2, L3, P)

# Joint velocities — damped least squares (avoids singularities)
cartesian_velocity = np.vstack((x_dot, y_dot, z_dot))
theta_dot_DLS      = np.zeros((3, len(t)))
damp = 0.001
for i in range(len(t)):
    J  = FL_Jacobian(L1, L2, L3, theta1[i], theta2[i], theta3[i])
    JT = J.T
    theta_dot_DLS[:, i] = np.linalg.solve(JT @ J + damp**2 * np.eye(3),
                                           JT @ cartesian_velocity[:, i])

# Joint torques — tau = J^T * F
tau = np.zeros((3, len(t)))
for i in range(len(t)):
    J = FL_Jacobian(L1, L2, L3, theta1[i], theta2[i], theta3[i])
    tau[:, i] = J.T @ F[:, i]

# Print summary
print("\n══════ TAROK  Up/Down  —  Joint Analysis ══════")
for j, name in enumerate(['Hip (θ1)', 'Thigh (θ2)', 'Knee (θ3)']):
    th_deg = np.rad2deg([theta1, theta2, theta3][j])
    print(f"\n  {name}")
    print(f"    Angle    : {th_deg.min():.1f}° → {th_deg.max():.1f}°  "
          f"(ROM {th_deg.max()-th_deg.min():.1f}°)")
    print(f"    Avg speed: {np.mean(np.abs(np.rad2deg(theta_dot_DLS[j]))):.2f} deg/s")
    print(f"    Torque   : {tau[j].min():.2f} → {tau[j].max():.2f} Nm")
print()


################## ANIMATION ######################
fig = plt.figure(figsize=(17, 9))
fig.suptitle('TAROK — Full Robot Up/Down Simulation', fontsize=14, fontweight='bold')

# 3D animation (left)
ax3d = fig.add_subplot(1, 2, 1, projection='3d')
ax3d.set_xlim(-0.40, 0.40)
ax3d.set_ylim(-0.40, 0.40)
ax3d.set_zlim(-0.60, 0.20)
ax3d.set_xlabel('X (m)')
ax3d.set_ylabel('Y (m)')
ax3d.set_zlabel('Z (m)')
ax3d.set_title('Robot Animation')
ax3d.view_init(elev=20, azim=225)

# Data plots (right column)
ax_ang = fig.add_subplot(3, 2, 2)
ax_vel = fig.add_subplot(3, 2, 4)
ax_tau = fig.add_subplot(3, 2, 6)
plt.subplots_adjust(left=0.05, right=0.97, top=0.92, bottom=0.06,
                    wspace=0.30, hspace=0.50)

COLORS = {'FL': 'tab:blue', 'FR': 'tab:orange', 'HL': 'tab:green', 'HR': 'tab:red'}
JNT_COL = ['tab:blue', 'tab:orange', 'tab:green']

for ax, title, ylabel in [(ax_ang, 'Joint Angles',           'Angle (deg)'),
                           (ax_vel, 'Joint Velocities (DLS)', 'deg/s'),
                           (ax_tau, 'Joint Torques',          'Nm')]:
    ax.set_title(title); ax.set_xlabel('Time (s)'); ax.set_ylabel(ylabel)
    ax.grid(True, linestyle='--', alpha=0.5)

for j, (col, lbl) in enumerate(zip(JNT_COL, ['θ₁ Hip', 'θ₂ Thigh', 'θ₃ Knee'])):
    ax_ang.plot(t, np.rad2deg([theta1, theta2, theta3][j]), color=col, lw=1.3, label=lbl)
    ax_vel.plot(t, np.rad2deg(theta_dot_DLS[j]),            color=col, lw=1.3, label=lbl)
    ax_tau.plot(t, tau[j],                                  color=col, lw=1.3, label=lbl)

for ax in [ax_ang, ax_vel, ax_tau]:
    ax.legend(fontsize=8, loc='upper right')

# Time cursor lines on data plots
vline_ang = ax_ang.axvline(0, color='gray', lw=1, ls='--')
vline_vel = ax_vel.axvline(0, color='gray', lw=1, ls='--')
vline_tau = ax_tau.axvline(0, color='gray', lw=1, ls='--')

# Leg lines + foot dots + foot trace
leg_lines = {leg: ax3d.plot([], [], [], 'o-', color=col, lw=2,   ms=5,  label=leg)[0]
             for leg, col in COLORS.items()}
foot_dots = {leg: ax3d.plot([], [], [], '*',  color=col, ms=10)[0]
             for leg, col in COLORS.items()}
foot_traces  = {leg: ax3d.plot([], [], [], '-', color=col, lw=0.8, alpha=0.4)[0]
                for leg, col in COLORS.items()}
foot_history = {leg: [] for leg in COLORS}
TRACE_LEN = 40

# Shoulder markers (static reference points)
for leg, sh in SHOULDERS.items():
    ax3d.scatter(*sh, color=COLORS[leg], s=30, zorder=6)
    ax3d.text(sh[0], sh[1], sh[2] + 0.015, leg, fontsize=8, color=COLORS[leg])

ax3d.legend(loc='upper right', fontsize=8)

# Time annotation
time_text = ax3d.text2D(0.02, 0.97, '', transform=ax3d.transAxes,
                        fontsize=8, verticalalignment='top',
                        bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))


def to_world(p_local):
    """
    Rotate leg-local frame to world frame so legs hang downward.
    FK local axes:  x = leg extension (forward/down),  y = abduction,  z = lateral
    World axes:     X = robot forward,  Y = robot left,  Z = up
    Mapping:  local x → world -Z  (extension goes down)
              local y → world  Y  (abduction stays left/right)
              local z → world  X  (lateral maps to forward)
    """
    lx, ly, lz = p_local[0], p_local[1], p_local[2]
    return np.array([lz, ly, -lx])


def get_chain(th1, th2, th3, shoulder, side):
    """Return [shoulder, J2, J3, foot] in world frame."""
    if side == 'L':
        p2  = FL_P0_2  (th1, th2, th3).flatten()
        p3  = FL_P0_3  (th1, th2, th3).flatten()
        end = FL_P0_end(th1, th2, th3).flatten()
    else:
        p2  = FR_P0_2  (th1, th2, th3).flatten()
        p3  = FR_P0_3  (th1, th2, th3).flatten()
        end = FR_P0_end(th1, th2, th3).flatten()
    return [shoulder,
            shoulder + to_world(p2),
            shoulder + to_world(p3),
            shoulder + to_world(end)]


def init():
    for obj in list(leg_lines.values()) + list(foot_dots.values()) + list(foot_traces.values()):
        obj.set_data([], [])
        obj.set_3d_properties([])
    time_text.set_text('')
    return []


def update(num):
    th1, th2, th3 = theta1[num], theta2[num], theta3[num]
    current_t = t[num]

    for leg, side in [('FL', 'L'), ('FR', 'R'), ('HL', 'L'), ('HR', 'R')]:
        chain = get_chain(th1, th2, th3, SHOULDERS[leg], side)
        xs = [p[0] for p in chain]
        ys = [p[1] for p in chain]
        zs = [p[2] for p in chain]

        leg_lines[leg].set_data(xs, ys)
        leg_lines[leg].set_3d_properties(zs)

        foot_dots[leg].set_data([xs[-1]], [ys[-1]])
        foot_dots[leg].set_3d_properties([zs[-1]])

        foot_history[leg].append((xs[-1], ys[-1], zs[-1]))
        if len(foot_history[leg]) > TRACE_LEN:
            foot_history[leg].pop(0)
        hist = np.array(foot_history[leg])
        foot_traces[leg].set_data(hist[:, 0], hist[:, 1])
        foot_traces[leg].set_3d_properties(hist[:, 2])

    time_text.set_text(f"t = {current_t:.2f} s\n"
                       f"θ₁ = {np.rad2deg(th1):+.1f}°\n"
                       f"θ₂ = {np.rad2deg(th2):+.1f}°\n"
                       f"θ₃ = {np.rad2deg(th3):+.1f}°")

    vline_ang.set_xdata([current_t, current_t])
    vline_vel.set_xdata([current_t, current_t])
    vline_tau.set_xdata([current_t, current_t])
    return []


ani = animation.FuncAnimation(fig, update, frames=len(t), init_func=init,
                               interval=20, blit=False, repeat=True)
plt.show()