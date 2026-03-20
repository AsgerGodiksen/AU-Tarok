##### Pitch Visualisation — Live IMU Input (BNO085) #####
#
# Conventions:
#   Body frame  : X = forward, Y = left, Z = up.  Origin at torso centre.
#   Leg frame   : x = down (extension), y = lateral, z = fore-aft.
#   Mapping is handled by TB_0 / T0_B from Constant_Transforms.py.
#
# Usage:
#   Place this file in the same folder as:
#       Constant_Transforms.py
#       Forward_Kinematics.py
#       Inverse_Kinematics.py
#       Pitch_And_Roll.py
#       IMU_BNO085.py
#   then run:  python Simulation_Pitch_IMU.py

import sys, os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

import threading
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401
from math import pi

from Robot.Kinematics.Constant_Transforms import TB_0, T0_B
from Robot.Kinematics.Forward_Kinematics   import P0_2, P0_3, P0_end
from Robot.Kinematics.Inverse_Kinematics   import Inverse_Kinematics
from Robot.Kinematics.Pitch_And_Roll       import Inverse_Pitch
from Robot.Hardware.IMU_BNO085           import IMU_Initialization, Get_Quaternion, Quaternion_To_Euler


# ─────────────────────────── ROBOT PARAMETERS ────────────────────────────────
L1      = 0.078
TORSO_L = 0.7048    # matches L_Body inside Pitch_And_Roll.py
TORSO_W = 0.220     # matches W_Body inside Pitch_And_Roll.py
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


# ─────────────────────────── IMU THREAD ──────────────────────────────────────
# Shared state: latest pitch reading in degrees, protected by a lock.
_imu_lock       = threading.Lock()
_latest_pitch   = 0.0   # degrees
_imu_status     = 'Initialising IMU...'

def _imu_thread_fn():
    """Background thread: initialise IMU, then continuously update _latest_pitch."""
    global _latest_pitch, _imu_status

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
                angles = Quaternion_To_Euler(quat)   # [roll, pitch, yaw] in degrees
                pitch  = angles[1]
                with _imu_lock:
                    _latest_pitch = pitch
            except Exception:
                pass   # skip bad reads silently; keep last good value
    finally:
        try:
            i2c.deinit()
        except Exception:
            pass

# Start IMU thread as a daemon so it dies automatically when the plot window closes
_thread = threading.Thread(target=_imu_thread_fn, daemon=True)
_thread.start()


# ─────────────────────────── POSE SOLVER ─────────────────────────────────────
def solve_pose(pitch_deg: float):
    """
    Given a pitch angle (degrees, positive = nose UP):
      1. Inverse_Pitch       → new foot positions in body frame
      2. T0_B                → foot in leg frame  (IK input)
      3. Inverse_Kinematics  → joint angles
      4. P0_2 / P0_3 / P0_end → joint positions in leg frame
      5. TB_0                → all positions back to body frame for plotting

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
        p2_0  = P0_2(t1, t2, t3, leg)
        p3_0  = P0_3(t1, t2, t3, leg)
        end_0 = P0_end(t1, t2, t3, leg)

        # Transform every joint position back to body frame
        shoulder_B = SHOULDER_BODY[leg]
        p2_B       = TB_0(p2_0,  leg).flatten()
        p3_B       = TB_0(p3_0,  leg).flatten()
        end_B      = TB_0(end_0, leg).flatten()

        chains[leg] = [shoulder_B, p2_B, p3_B, end_B]

    return chains


# ─────────────────────────── FIGURE SETUP ────────────────────────────────────
fig = plt.figure(figsize=(10, 8))
fig.suptitle('TAROK — Live Pitch Visualisation (BNO085)', fontsize=14, fontweight='bold')

ax = fig.add_subplot(1, 1, 1, projection='3d')

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

# Animated objects
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
def draw_pose(pitch_deg: float):
    chains = solve_pose(pitch_deg)

    for leg in LEG_NAMES:
        pts = chains[leg]
        leg_lines[leg].set_data([p[0] for p in pts],
                                  [p[1] for p in pts])
        leg_lines[leg].set_3d_properties([p[2] for p in pts])

    # Torso rectangle: FL → FR → HR → HL → FL
    sh    = {leg: chains[leg][0] for leg in LEG_NAMES}
    order = ['FL', 'FR', 'HR', 'HL', 'FL']
    torso_line.set_data([sh[l][0] for l in order],
                         [sh[l][1] for l in order])
    torso_line.set_3d_properties([sh[l][2] for l in order])

    with _imu_lock:
        status = _imu_status

    info_text.set_text(
        f'Pitch = {pitch_deg:+.2f}°\n'
        f'{status}'
    )


# ─────────────────────────── ANIMATION LOOP ──────────────────────────────────
def _update(_frame):
    with _imu_lock:
        pitch = _latest_pitch
    draw_pose(pitch)
    return list(leg_lines.values()) + [torso_line, info_text]


# Draw a first frame immediately so the window isn't blank while IMU initialises
draw_pose(0.0)

ani = animation.FuncAnimation(
    fig,
    _update,
    interval=50,      # redraw every 50 ms  (~20 Hz)
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
 