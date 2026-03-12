##### Real-Time IMU Pitch & Roll — Live Joint Angle Plot #####
# Run from: ~/AU-Tarok/Tarok/Robot/Simulations/
# Reads pitch and roll from BNO085 IMU in real-time.
# Computes IK for all 4 legs and displays scrolling joint angle plots.

import sys
import os

# Add Robot/ to path so Kinematics and Hardware can be imported directly
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

from Kinematics.Inverse_Kinematics import Inverse_Kinematics
from Kinematics.Pitch_And_Roll import Inverse_Pitch, Inverse_Roll
from Hardware.IMU_BNO085 import IMU_Initialization, Get_Quaternion, Quaternion_To_Euler


################## PARAMETERS ######################
L1, L2, L3       = 0.078, 0.20, 0.30
TORSO_L, TORSO_W = 0.479, 0.2179
NOMINAL_Z        = -0.40

WINDOW_S = 10.0   # seconds of history shown
PLOT_DT  = 0.05   # update interval (~20 Hz)
MAXLEN   = int(WINDOW_S / PLOT_DT)

NOMINAL_FEET = [
    [ TORSO_L/2,  TORSO_W/2 + L1, NOMINAL_Z],   # FL (idx 0)
    [ TORSO_L/2, -TORSO_W/2 - L1, NOMINAL_Z],   # FR (idx 1)
    [-TORSO_L/2,  TORSO_W/2 + L1, NOMINAL_Z],   # HL (idx 2)
    [-TORSO_L/2, -TORSO_W/2 - L1, NOMINAL_Z],   # HR (idx 3)
]

SHOULDERS = {
    'FL': np.array([ TORSO_L/2,  TORSO_W/2]),
    'FR': np.array([ TORSO_L/2, -TORSO_W/2]),
    'HL': np.array([-TORSO_L/2,  TORSO_W/2]),
    'HR': np.array([-TORSO_L/2, -TORSO_W/2]),
}

LEG_NAMES = ['FL', 'FR', 'HL', 'HR']
LEG_IDX   = {'FL': 0, 'FR': 1, 'HL': 2, 'HR': 3}


################## IK SOLVER ######################
def solve_ik(pitch_rad, roll_rad):
    new_feet, sh_heights = Inverse_Pitch(pitch_rad, [list(f) for f in NOMINAL_FEET])
    new_feet = Inverse_Roll(roll_rad, new_feet, sh_heights)

    angles = {}
    for leg in LEG_NAMES:
        i      = LEG_IDX[leg]
        sh_nom = SHOULDERS[leg]

        ik_x = sh_heights[i]
        ik_y = abs(new_feet[i][1] - sh_nom[1])
        ik_z = new_feet[i][0] - sh_nom[0]
        P = np.array([[ik_x], [ik_y], [ik_z]])

        try:
            a1, a2, a3 = Inverse_Kinematics(P, leg)
            angles[leg] = (a1, a2, a3)
        except ValueError:
            angles[leg] = None

    return angles


################## BUFFERS ######################
t_buf     = deque(maxlen=MAXLEN)
pitch_buf = deque(maxlen=MAXLEN)
roll_buf  = deque(maxlen=MAXLEN)

joint_bufs = {
    leg: [deque(maxlen=MAXLEN) for _ in range(3)]
    for leg in LEG_NAMES
}


################## FIGURE ######################
LEG_COLORS = {'FL': 'tab:blue', 'FR': 'tab:orange', 'HL': 'tab:green', 'HR': 'tab:red'}
JNT_LABELS = ['θ₁ Hip', 'θ₂ Thigh', 'θ₃ Knee']
JNT_STYLES = ['-', '--', ':']

fig, axes = plt.subplots(3, 2, figsize=(15, 9))
fig.suptitle('TAROK — Real-Time IMU Pitch & Roll', fontsize=14, fontweight='bold')
plt.subplots_adjust(left=0.07, right=0.97, top=0.92, bottom=0.07,
                    wspace=0.30, hspace=0.50)

ax_pitch = axes[0, 0]
ax_roll  = axes[0, 1]
leg_axes = {'FL': axes[1, 0], 'FR': axes[1, 1], 'HL': axes[2, 0], 'HR': axes[2, 1]}

line_pitch, = ax_pitch.plot([], [], color='black',  lw=1.5, label='Pitch')
line_roll,  = ax_roll.plot([],  [], color='purple', lw=1.5, label='Roll')

for ax, title in [(ax_pitch, 'Pitch (deg)'), (ax_roll, 'Roll (deg)')]:
    ax.set_title(title)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('deg')
    ax.axhline(0, color='gray', lw=0.8, ls='--')
    ax.grid(True, ls='--', alpha=0.5)
    ax.legend(fontsize=8)

leg_lines = {}
for leg, ax in leg_axes.items():
    ax.set_title(f'{leg} Joint Angles')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('deg')
    ax.axhline(0, color='gray', lw=0.8, ls='--')
    ax.grid(True, ls='--', alpha=0.5)
    lines = [ax.plot([], [], color=LEG_COLORS[leg], ls=JNT_STYLES[j],
                     lw=1.3, label=JNT_LABELS[j])[0] for j in range(3)]
    ax.legend(fontsize=8, loc='upper right')
    leg_lines[leg] = lines


################## IMU INIT ######################
print("Initialising IMU...")
bno, i2c = IMU_Initialization()
print("IMU ready. Starting plot...")

t_start = time.time()


################## ANIMATION ######################
def update(_frame):
    try:
        quat  = Get_Quaternion(bno)
        euler = Quaternion_To_Euler(quat)   # [roll, pitch, yaw] in degrees
    except Exception:
        return []

    roll_deg  = euler[0]
    pitch_deg = euler[1]
    now       = time.time() - t_start

    angles = solve_ik(np.radians(pitch_deg), np.radians(roll_deg))

    t_buf.append(now)
    pitch_buf.append(pitch_deg)
    roll_buf.append(roll_deg)

    for leg in LEG_NAMES:
        result = angles[leg]
        for j in range(3):
            val = np.degrees(result[j]) if result is not None else np.nan
            joint_bufs[leg][j].append(val)

    t_arr = np.array(t_buf)
    t_min = t_arr[-1] - WINDOW_S

    for ax, line, buf in [(ax_pitch, line_pitch, pitch_buf),
                          (ax_roll,  line_roll,  roll_buf)]:
        line.set_data(t_arr, np.array(buf))
        ax.set_xlim(t_min, t_arr[-1] + 0.1)
        ax.relim()
        ax.autoscale_view(scalex=False, scaley=True)

    for leg in LEG_NAMES:
        ax = leg_axes[leg]
        for j in range(3):
            leg_lines[leg][j].set_data(t_arr, np.array(joint_bufs[leg][j]))
        ax.set_xlim(t_min, t_arr[-1] + 0.1)
        ax.relim()
        ax.autoscale_view(scalex=False, scaley=True)

    return []


ani = animation.FuncAnimation(fig, update, interval=int(PLOT_DT * 1000),
                               blit=False, cache_frame_data=False)

try:
    plt.show()
except KeyboardInterrupt:
    pass
finally:
    i2c.deinit()
    print("IMU disconnected.")