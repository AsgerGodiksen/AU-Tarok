"""
Test_Balance_Control.py
=======================
Two-mode test for Tarok balance control.

    BALANCE_ENABLED = False  →  Test 1: Robot stands passively while you tilt
                                         the plate. IMU + foot positions are
                                         logged but no corrections are sent.

    BALANCE_ENABLED = True   →  Test 2: PD balance controller is active. Robot
                                         reacts to IMU tilt and adjusts foot
                                         positions to level the torso.

Both tests log the full signal chain to a timestamped CSV in LOG_DIR.

Motor convention
----------------
Each leg has its own CAN bus (can0–can3).
All three joints on every leg share the same IDs:
    J1 (hip abduction)  → 0x141
    J2 (shoulder swing) → 0x142
    J3 (knee)           → 0x143
Motors are zeroed at the straight/neutral position.
Position commands are sent in degrees (np.degrees of IK output).

IMU pipeline
------------
Raw quaternion → RB_IMU() [IMU frame → body frame] → Quaternion_To_Euler()
→ [Pitch, Roll, Yaw] in body frame, in DEGREES.
Balance_Control() expects radians, so degrees are converted before passing.

Foot position convention
------------------------
Nominal foot positions are defined in the BODY frame (x-forward, y-left, z-up).
Balance_Control / Inverse_Pitch / Inverse_Roll work with a 4×3 plain list:
    [ [x, y, z],   # leg 0 : Front Left
      [x, y, z],   # leg 1 : Front Right
      [x, y, z],   # leg 2 : Hind  Left
      [x, y, z] ]  # leg 3 : Hind  Right
"""

# ---------------------------------------------------------------------------
# USER CONFIGURATION — edit these before each run
# ---------------------------------------------------------------------------

BALANCE_ENABLED = False   # False = Test 1 (passive log), True = Test 2 (active)
FREQ            = 20      # Control / logging rate [Hz]
LOG_DIR         = "."     # Directory for CSV output (current working directory)

# Standing height — vertical distance from hip origins to feet [m]
# Adjust to match the actual standing pose you want.
STAND_Z = -0.41           # negative because z-down is "into the ground"

# ---------------------------------------------------------------------------
# IMPORTS
# ---------------------------------------------------------------------------

import can
import csv
import time
import os
import numpy as np
from datetime import datetime

# Kinematics
from Robot.Kinematics.Inverse_Kinematics  import Inverse_Kinematics
from Robot.Kinematics.Constant_Transforms import T0_B, RB_IMU
from Robot.Kinematics.Balance_Control     import Balance_Control

# Hardware
from Robot.Hardware.IMU_BNO085   import IMU_Initialization, Get_Quaternion, Quaternion_To_Euler
from Robot.Hardware.Motor_Controls import Position_Control

# ---------------------------------------------------------------------------
# CONSTANTS
# ---------------------------------------------------------------------------

# Motor IDs (same on every CAN bus)
ID_J1 = 0x141   # Hip abduction  / adduction
ID_J2 = 0x142   # Shoulder swing (sagittal)
ID_J3 = 0x143   # Knee (via four-bar)

# Motor speed limit for position commands [deg/s]
# Keep conservative during balance tests.
MAX_SPEED = 100  # deg/s

# Body dimensions [m] — must match Constant_Transforms & Pitch_And_Roll
L_BODY = 0.7048
W_BODY = 0.220
L1     = 0.078   # Hip abduction link

# Desired torso orientation (level)
PITCH_DESIRED = 0.0   # radians
ROLL_DESIRED  = 0.0   # radians

# Leg index → string label (used for IK and transforms)
LEG_LABELS = {0: 'FL', 1: 'FR', 2: 'HL', 3: 'HR'}

# CAN bus channel per leg
CAN_CHANNELS = {0: 'can0', 1: 'can1', 2: 'can2', 3: 'can3'}

# ---------------------------------------------------------------------------
# NOMINAL STANDING FOOT POSITIONS  (body frame, 4×3 list)
# ---------------------------------------------------------------------------
# Matches the convention in Initial_Position_Visualization.py and
# Inverse_Pitch / Inverse_Roll:
#   index 0 = FL, 1 = FR, 2 = HL, 3 = HR

def make_nominal_foot_positions(z=STAND_Z):
    """Return nominal foot positions as a 4x3 plain list in the body frame."""
    x_front =  L_BODY / 2
    x_hind  = -L_BODY / 2
    y_left  =  W_BODY / 2 + L1
    y_right = -(W_BODY / 2 + L1)

    return [
        [x_front,  y_left,  z],   # 0: FL
        [x_front,  y_right, z],   # 1: FR
        [x_hind,   y_left,  z],   # 2: HL
        [x_hind,   y_right, z],   # 3: HR
    ]

# ---------------------------------------------------------------------------
# HELPERS
# ---------------------------------------------------------------------------

def foot_positions_to_joint_angles(foot_positions):
    """
    Convert a 4×3 body-frame foot position list to joint angles.

    Returns a list of 4 tuples: [(th1, th2, th3), ...]
    Raises ValueError (from IK) if any foot is outside the workspace.
    """
    angles = []
    for i, label in LEG_LABELS.items():
        p_body = np.array(foot_positions[i]).reshape(3, 1)
        p_base = T0_B(p_body, label)
        th1, th2, th3 = Inverse_Kinematics(p_base, label)
        angles.append((th1, th2, th3))
    return angles


def send_joint_angles(buses, joint_angles):
    """
    Send position commands (degrees) to all 12 motors.

    buses        : dict {leg_index: can.Bus}
    joint_angles : list of 4 tuples [(th1, th2, th3), ...]
    """
    for i, (th1, th2, th3) in enumerate(joint_angles):
        bus = buses[i]
        Position_Control(bus, ID_J1, np.degrees(th1), MAX_SPEED)
        Position_Control(bus, ID_J2, np.degrees(th2), MAX_SPEED)
        Position_Control(bus, ID_J3, np.degrees(th3), MAX_SPEED)


def read_imu_body_frame_rad(bno):
    """
    Read IMU, rotate into body frame, return Euler angles in RADIANS.
    Returns np.array([pitch, roll, yaw]) in radians.
    """
    q_raw     = Get_Quaternion(bno)           # [i, j, k, real] = [x, y, z, w]
    q_body    = RB_IMU(q_raw)                 # Rotation object in body frame
    euler_deg = Quaternion_To_Euler(q_body.as_quat())  # degrees
    euler_rad = np.radians(euler_deg)          # radians: [pitch, roll, yaw]
    return euler_rad


def make_log_row(t, euler_rad, foot_positions, joint_angles,
                 pitch_error=None, roll_error=None,
                 pitch_pd=None,    roll_pd=None):
    """
    Pack one loop iteration into a flat list for CSV logging.
    pitch_error / roll_error / pitch_pd / roll_pd are None in Test 1.
    """
    row = [round(t, 4)]

    # IMU
    row += [round(float(euler_rad[0]), 6),   # pitch_rad
            round(float(euler_rad[1]), 6),   # roll_rad
            round(float(euler_rad[2]), 6)]   # yaw_rad

    # PD signals (NaN when balance is disabled)
    row += [round(float(pitch_error), 6) if pitch_error is not None else float('nan'),
            round(float(roll_error),  6) if roll_error  is not None else float('nan'),
            round(float(pitch_pd),    6) if pitch_pd    is not None else float('nan'),
            round(float(roll_pd),     6) if roll_pd     is not None else float('nan')]

    # Foot positions (body frame)
    for fp in foot_positions:
        row += [round(float(fp[0]), 6),
                round(float(fp[1]), 6),
                round(float(fp[2]), 6)]

    # Joint angles (degrees)
    for th1, th2, th3 in joint_angles:
        row += [round(np.degrees(th1), 4),
                round(np.degrees(th2), 4),
                round(np.degrees(th3), 4)]

    return row


CSV_HEADER = (
    ['t_s',
     'pitch_rad', 'roll_rad', 'yaw_rad',
     'pitch_error', 'roll_error', 'pitch_pd', 'roll_pd'] +
    [f'{leg}_{ax}' for leg in ['FL', 'FR', 'HL', 'HR'] for ax in ['x', 'y', 'z']] +
    [f'{leg}_th{j}' for leg in ['FL', 'FR', 'HL', 'HR'] for j in [1, 2, 3]]
)

# ---------------------------------------------------------------------------
# MAIN
# ---------------------------------------------------------------------------

def main():
    mode_label = "ACTIVE_BALANCE" if BALANCE_ENABLED else "PASSIVE_LOG"
    print(f"\n{'='*55}")
    print(f"  Tarok — Balance Control Test")
    print(f"  Mode  : {mode_label}")
    print(f"  Rate  : {FREQ} Hz")
    print(f"{'='*55}\n")

    # --- Open CAN buses ---
    print("Opening CAN buses...")
    buses = {}
    try:
        for leg_idx, channel in CAN_CHANNELS.items():
            buses[leg_idx] = can.interface.Bus(channel, bustype='socketcan')
            print(f"  can{leg_idx} ({LEG_LABELS[leg_idx]}) → {channel} OK")
    except Exception as e:
        print(f"  ERROR opening CAN bus: {e}")
        raise

    # --- Initialize IMU ---
    print("\nInitializing IMU...")
    bno, i2c = IMU_Initialization()
    print("  IMU OK")

    # --- Compute nominal standing pose ---
    nominal_foot_positions = make_nominal_foot_positions()
    print("\nNominal foot positions (body frame):")
    for i, label in LEG_LABELS.items():
        fp = nominal_foot_positions[i]
        print(f"  {label}: x={fp[0]:.4f}  y={fp[1]:.4f}  z={fp[2]:.4f}")

    # --- Move to standing pose ---
    print("\nMoving to standing pose...")
    try:
        nominal_angles = foot_positions_to_joint_angles(nominal_foot_positions)
    except ValueError as e:
        print(f"  IK ERROR on nominal pose: {e}")
        raise

    send_joint_angles(buses, nominal_angles)
    print("  Commands sent. Waiting 2 s for robot to settle...")
    time.sleep(2.0)
    print("  Ready.\n")

    # --- Prepare logging ---
    timestamp     = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_filename  = os.path.join(LOG_DIR, f"balance_test_{mode_label}_{timestamp}.csv")
    log_rows      = []

    # --- Control loop ---
    dt         = 1.0 / FREQ
    t_start    = time.time()
    loop_count = 0

    print(f"Running. Press Ctrl+C to stop and save log to:\n  {log_filename}\n")

    try:
        while True:
            t_loop_start = time.time()
            t            = t_loop_start - t_start

            # 1. Read IMU — body frame, radians
            euler_rad = read_imu_body_frame_rad(bno)

            # 2. Determine commanded foot positions
            pitch_error = roll_error = pitch_pd = roll_pd = None

            if BALANCE_ENABLED:
                # --- Test 2: PD balance controller ---
                # Balance_Control returns adjusted 4×3 foot position list.
                # It also internally tracks previous errors and filtered angles,
                # so we call it and then recover the signals for logging.
                from Robot.Kinematics.Balance_Control import (
                    _pitch_filter, _roll_filter,
                    _pitch_prev_error, _roll_prev_error,
                    PITCH_P, PITCH_D, ROLL_P, ROLL_D,
                )
                pitch_measured = _pitch_filter.Previous   # already updated this step
                roll_measured  = _roll_filter.Previous

                commanded_foot_positions = Balance_Control(
                    Euler_Angle    = euler_rad,
                    Pitch_Desired  = PITCH_DESIRED,
                    Roll_Desired   = ROLL_DESIRED,
                    Foot_Positions = nominal_foot_positions,
                )

                # Recover error signals for logging (post-update values)
                pitch_error = PITCH_DESIRED - _pitch_filter.Previous
                roll_error  = ROLL_DESIRED  - _roll_filter.Previous
                pitch_pd    = pitch_error * PITCH_P + (pitch_error - _pitch_prev_error) * PITCH_D / dt
                roll_pd     = roll_error  * ROLL_P  + (roll_error  - _roll_prev_error)  * ROLL_D  / dt

            else:
                # --- Test 1: Passive — hold nominal pose ---
                commanded_foot_positions = nominal_foot_positions

            # 3. IK → motor commands
            try:
                joint_angles = foot_positions_to_joint_angles(commanded_foot_positions)
                send_joint_angles(buses, joint_angles)
            except ValueError as e:
                # Foot position went outside workspace — skip this command,
                # log the event, keep running.
                print(f"  [t={t:.2f}s] IK workspace violation: {e}")
                joint_angles = nominal_angles   # log last safe angles

            # 4. Log
            row = make_log_row(
                t, euler_rad, commanded_foot_positions, joint_angles,
                pitch_error, roll_error, pitch_pd, roll_pd
            )
            log_rows.append(row)

            # 5. Rate limiting
            loop_count += 1
            elapsed = time.time() - t_loop_start
            sleep_time = dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
            elif loop_count % 100 == 0:
                print(f"  [WARNING] Loop running slow: {elapsed*1000:.1f} ms > {dt*1000:.0f} ms budget")

    except KeyboardInterrupt:
        print(f"\nStopped after {loop_count} iterations ({loop_count/FREQ:.1f} s).")

    finally:
        # --- Save CSV ---
        print(f"Saving log ({len(log_rows)} rows) → {log_filename}")
        with open(log_filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(CSV_HEADER)
            writer.writerows(log_rows)
        print("  Saved OK.")

        # --- Shutdown ---
        print("Closing IMU and CAN buses...")
        try:
            i2c.deinit()
        except Exception:
            pass
        for bus in buses.values():
            try:
                bus.shutdown()
            except Exception:
                pass
        print("Done.\n")


if __name__ == "__main__":
    main()