##### Pitch & Roll Visualisation — Live IMU Input (BNO085) #####
#
# Conventions (body frame):
#   X = forward,  Y = left,  Z = up.  Origin at torso centre.
#   Positive pitch = nose UP.
#   Positive roll  = lean LEFT (left side drops, right side rises).
#
# All kinematics delegated to project files:
#   Pitch_And_Roll.py      -> Inverse_Pitch, Inverse_Roll
#   Inverse_Kinematics.py  -> Inverse_Kinematics
#   Forward_Kinematics.py  -> P0_2, P0_3, P0_end
#   Constant_Transforms.py -> TB_0, T0_B
#   IMU_BNO085.py          -> IMU_Initialization, Get_Quaternion, Quaternion_To_Euler
#
# Usage:
#   Place this file in the same folder as the five files above and run:
#       python Simulation_PitchRoll_IMU.py

import sys, os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

import io, contextlib, threading
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D   # noqa: F401
from math import pi

from Robot.Kinematics.Inverse_Kinematics import*
from Robot.Kinematics.Forward_Kinematics import*
from Robot.Kinematics.Constant_Transforms import*
from Robot.Kinematics.Jacobian import*
from Robot.Kinematics.Pitch_And_Roll import*
from Robot.Hardware.IMU_BNO085           import*


# ─────────────────────────── ROBOT PARAMETERS ────────────────────────────────
L1      = 0.078
TORSO_L = 0.7048    # matches L_Body inside Pitch_And_Roll.py
TORSO_W = 0.220     # matches W_Body inside Pitch_And_Roll.py
W_HIP   = TORSO_W / 2   # lateral distance from centre to shoulder = 0.110
z = -0.41

LEG_NAMES = ['FL', 'FR', 'HL', 'HR']
COLORS    = {'FL': 'tab:blue', 'FR': 'tab:orange', 'HL': 'tab:green', 'HR': 'tab:red'}

# Nominal foot positions in body frame (torso centre at origin, left = +Y)
NOMINAL_FEET = [
    [ TORSO_L/2,  W_HIP + L1, z],   # FL  (idx 0)
    [ TORSO_L/2, -(W_HIP + L1), z], # FR  (idx 1)
    [-TORSO_L/2,  W_HIP + L1, z],   # HL  (idx 2)
    [-TORSO_L/2, -(W_HIP + L1), z], # HR  (idx 3)
]

LEG_IDX = {leg: i for i, leg in enumerate(LEG_NAMES)}

# Shoulder positions in body frame
SHOULDER_BODY = {
    leg: TB_0(np.zeros((3, 1)), leg).flatten()
    for leg in LEG_NAMES
}

# IK/FK for FR and HL (config = -1) are inconsistent when theta1 != 0.
# Fix: mirror FR -> FL and HL -> HR (config = +1), solve there, negate y back.
MIRROR_LEG = {'FL': 'FL', 'FR': 'FL', 'HL': 'HR', 'HR': 'HR'}


# ─────────────────────────── IMU THREAD ──────────────────────────────────────
_imu_lock    = threading.Lock()
_latest_pitch = 0.0   # degrees
_latest_roll  = 0.0   # degrees
_imu_status   = 'Initialising IMU...'


def _imu_thread_fn():
    """Background thread: initialise the BNO085, then stream pitch and roll."""
    global _latest_pitch, _latest_roll, _imu_status

    try:
        bno, i2c = IMU_Initialization()
        _imu_status = 'IMU OK'
    except Exception as e:
        _imu_status = f'IMU FAILED: {e}'
        return

    try:
        while True:
            try:
                quat   = Get_Quaternion(bno)
                angles = Quaternion_To_Euler(quat)  # [roll, pitch, yaw] degrees
                with _imu_lock:
                    _latest_roll  = angles[0]
                    _latest_pitch = angles[1]
            except Exception:
                pass   # keep last good values on a bad read
    finally:
        try:
            i2c.deinit()
        except Exception:
            pass


_thread = threading.Thread(target=_imu_thread_fn, daemon=True)
_thread.start()


# ─────────────────────────── POSE SOLVER ─────────────────────────────────────
def solve_pose(pitch_deg: float, roll_deg: float):
    """
    Given pitch and roll angles (degrees):

      1. Inverse_Pitch  -> pitched foot positions + shoulder heights
      2. Inverse_Roll   -> apply roll on top of the pitched feet
         (feet are shifted to Inverse_Roll's shoulder reference frame
          before the call and converted back afterward)
      3. Mirror FR->FL / HL->HR; T0_B; Inverse_Kinematics
      4. P0_2 / P0_3 / P0_end; TB_0 -> body frame; un-mirror FR / HL

    Returns {leg: [shoulder, j2, j3, foot]} -- all points in body frame.
    """
    pitch_rad = pitch_deg * pi / 180.0
    roll_rad  = roll_deg  * pi / 180.0

    # ── Step 1: Pitch ────────────────────────────────────────────────────────
    # Inverse_Pitch convention: positive argument = nose DOWN, so negate.
    with contextlib.redirect_stdout(io.StringIO()):
        pitched_feet, sh_heights = Inverse_Pitch(
            -pitch_rad, [list(f) for f in NOMINAL_FEET]
        )

    # ── Step 2: Roll (chained onto pitched feet) ─────────────────────────────
    # Inverse_Roll expects shoulder reference at ±W_Body (= 0.220).
    # Our shoulders sit at ±W_HIP (= 0.110).  Shift y by ±W_HIP before the
    # call, then reverse afterward.  Also correct the y-sign convention:
    # Inverse_Roll outputs left = -Y; our frame uses left = +Y.
    shifted = [
        [f[0], f[1] + (W_HIP if (i == 0 or i == 2) else -W_HIP), f[2]]
        for i, f in enumerate(pitched_feet)
    ]
    with contextlib.redirect_stdout(io.StringIO()):
        raw_roll = Inverse_Roll(roll_rad, shifted, sh_heights)

    new_feet = []
    for i, f in enumerate(raw_roll):
        is_left = (i == 0 or i == 2)
        y_back = -f[1] - (W_HIP if is_left else -W_HIP)
        new_feet.append([f[0], y_back, f[2]])

    # ── Step 3 & 4: IK → FK → body frame ────────────────────────────────────
    chains = {}

    for leg in LEG_NAMES:
        i      = LEG_IDX[leg]
        mirror = MIRROR_LEG[leg]
        needs_mirror = (leg != mirror)

        foot_B   = np.array(new_feet[i])
        foot_B_m = foot_B.copy()
        if needs_mirror:
            foot_B_m[1] *= -1

        foot_0    = T0_B(foot_B_m.reshape(3, 1), mirror)
        foot_0_ik = foot_0.copy()
        if mirror in ('FR', 'HL'):
            foot_0_ik[2] *= -1

        try:
            t1, t2, t3 = Inverse_Kinematics(foot_0_ik, mirror)
        except ValueError:
            scale = (0.2 + 0.3 - 1e-4) / (np.linalg.norm(foot_0_ik) + 1e-9)
            t1, t2, t3 = Inverse_Kinematics(foot_0_ik * scale, mirror)

        p2_B  = TB_0(P0_2(t1,  t2, t3, mirror), mirror).flatten()
        p3_B  = TB_0(P0_3(t1,  t2, t3, mirror), mirror).flatten()
        end_B = TB_0(P0_end(t1, t2, t3, mirror), mirror).flatten()

        if needs_mirror:
            p2_B[1]  *= -1
            p3_B[1]  *= -1
            end_B[1] *= -1

        chains[leg] = [SHOULDER_BODY[leg], p2_B, p3_B, end_B]

    return chains


# ─────────────────────────── FIGURE SETUP ────────────────────────────────────
fig = plt.figure(figsize=(10, 8))
fig.suptitle('TAROK — Live Pitch & Roll Visualisation (BNO085)',
             fontsize=14, fontweight='bold')

ax = fig.add_subplot(1, 1, 1, projection='3d')

ax.set_xlim(-0.55, 0.55)
ax.set_ylim(-0.45, 0.45)
ax.set_zlim(-0.55, 0.25)
ax.set_xlabel('X — forward (m)')
ax.set_ylabel('Y — lateral (m)')
ax.set_zlabel('Z — up (m)')
ax.set_title('Body frame  |  torso centre at origin')
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
def draw_pose(pitch_deg: float, roll_deg: float):
    chains = solve_pose(pitch_deg, roll_deg)

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

    with _imu_lock:
        status = _imu_status

    info_text.set_text(
        f'Pitch = {pitch_deg:+.2f}°\n'
        f'Roll  = {roll_deg:+.2f}°\n'
        f'{status}'
    )


# ─────────────────────────── ANIMATION LOOP ──────────────────────────────────
def _update(_frame):
    with _imu_lock:
        pitch = _latest_pitch
        roll  = _latest_roll
    draw_pose(pitch, roll)
    return list(leg_lines.values()) + [torso_line, info_text]


# First frame while IMU initialises
draw_pose(0.0, 0.0)

ani = animation.FuncAnimation(
    fig,
    _update,
    interval=50,           # ~20 Hz
    blit=False,
    cache_frame_data=False,
)

try:
    plt.show()
except KeyboardInterrupt:
    print("\nStopping visualisation...")
finally:
    plt.close('all')
    print("Done.")