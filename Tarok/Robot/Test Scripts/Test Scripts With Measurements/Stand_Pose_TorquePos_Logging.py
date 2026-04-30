# Logging script for stand-pose hold: torque, actual position, and command position
# for all 12 motors. Based on Stand_Pose_Torque.py.
#
# Each motor step does two CAN round trips:
#   1. 0xA4  — position command → torque decoded from reply bytes [2-3]
#   2. 0x92  — read multi-turn angle → output shaft degrees (multi-turn, signed)
#
# CSV columns:
#   Timestamp (s),
#   FL_J1_Torque .. HR_J3_Torque  (Nm,  from 0xA4 reply),
#   FL_J1_Pos    .. HR_J3_Pos     (deg, output shaft, multi-turn, from 0x92 reply),
#   FL_J1_Cmd    .. HR_J3_Cmd     (deg, IK setpoint sent to motor — constant)

import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..')))

import can
import time
import numpy as np
import struct
import csv
import concurrent.futures

from Robot import *


# ─────────────────────────────────────────────────────────────────────────────
# Position_Control_With_Feedback — returns torque AND actual position
# ─────────────────────────────────────────────────────────────────────────────
def Position_Control_With_Feedback(bus, motor_id, new_position, max_speed, timeout=0.005):
    """
    Step 1 — 0xA4 position command:
      Reply bytes [2-3]: iq current (int16) → torque_Nm

    Step 2 — 0x92 read multi-turn angle:
      Reply bytes [1-7]: signed 56-bit angle, 0.01 deg/LSB motor shaft → / 900 = output deg

    Returns (torque_Nm, angle_deg), or (None, None) on timeout.
    """
    # ── Step 1: position command, read torque ─────────────────────────────────
    data = [0xA4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]

    speed_raw   = int(max_speed * 9)
    speed_bytes = struct.pack('<h', speed_raw)
    data[2] = speed_bytes[0]
    data[3] = speed_bytes[1]

    if new_position >= 0:
        pos_enc   = int(new_position * 100 * 9)
        pos_bytes = struct.pack('<i', pos_enc)
    else:
        pos_enc   = int(new_position * 100 * 9) & 0xFFFFFFFF
        pos_bytes = struct.pack('<I', pos_enc)

    data[4] = pos_bytes[0]
    data[5] = pos_bytes[1]
    data[6] = pos_bytes[2]
    data[7] = pos_bytes[3]

    msg_a4 = can.Message(arbitration_id=motor_id, data=data, is_extended_id=False)
    for attempt in range(3):
        try:
            bus.send(msg_a4)
            break
        except can.CanOperationError:
            time.sleep(0.001)
    else:
        return None, None

    deadline  = time.monotonic() + timeout
    torque_Nm = None
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return None, None
        reply = bus.recv(remaining)
        if reply is None:
            return None, None
        if reply.arbitration_id == motor_id and reply.data[0] == 0xA4:
            iq_raw    = struct.unpack('<h', bytes([reply.data[2], reply.data[3]]))[0]
            torque_Nm = (iq_raw / 2048.0) * 33.0 * 2.09
            break

    # ── Step 2: read multi-turn angle ─────────────────────────────────────────
    msg_92 = can.Message(arbitration_id=motor_id,
                         data=[0x92, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
                         is_extended_id=False)
    try:
        bus.send(msg_92)
    except can.CanOperationError:
        return torque_Nm, None

    deadline = time.monotonic() + timeout
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return torque_Nm, None
        reply = bus.recv(remaining)
        if reply is None:
            return torque_Nm, None
        if reply.arbitration_id == motor_id and reply.data[0] == 0x92:
            raw_bytes = bytes(reply.data[1:8]) + b'\x00'
            angle_raw = struct.unpack('<q', raw_bytes)[0]
            if reply.data[7] & 0x80:
                angle_raw -= (1 << 56)
            angle_deg = float(angle_raw) / 900.0
            return torque_Nm, angle_deg


def send_leg(bus, angles):
    """Command one leg's 3 motors and collect torque + position feedback."""
    return [
        Position_Control_With_Feedback(bus, motor_id, angle, 20)
        for motor_id, angle in zip([ID_1, ID_2, ID_3], angles)
    ]


def safe_torque(fb, i):
    val = fb[i][0]
    return val if val is not None else 0.0

def safe_position(fb, i):
    val = fb[i][1]
    return val if val is not None else 0.0


# ─────────────────────────────────────────────────────────────────────────────
# Pre-computations — stand pose IK (single static target)
# ─────────────────────────────────────────────────────────────────────────────
print("Performing pre-computations...")

l_k = 0.7048
w_k = 0.220

x_FL = x_FR =  l_k / 2
x_HL = x_HR = -l_k / 2
y_FL = y_HL =  w_k / 2 + 0.078
y_FR = y_HR = -w_k / 2 - 0.078
z = -0.41

P_FL_base = T0_B(np.array([x_FL, y_FL, z]).reshape((3, 1)), 'FL')
P_FR_base = T0_B(np.array([x_FR, y_FR, z]).reshape((3, 1)), 'FR')
P_HL_base = T0_B(np.array([x_HL, y_HL, z]).reshape((3, 1)), 'HL')
P_HR_base = T0_B(np.array([x_HR, y_HR, z]).reshape((3, 1)), 'HR')

Theta_FL = np.degrees(Inverse_Kinematics(P_FL_base, 'FL'))
Theta_FR = np.degrees(Inverse_Kinematics(P_FR_base, 'FR'))
Theta_HL = np.degrees(Inverse_Kinematics(P_HL_base, 'HL'))
Theta_HR = np.degrees(Inverse_Kinematics(P_HR_base, 'HR'))

print("Pre-computations complete.")
print(f"  FL cmd: {Theta_FL}")
print(f"  FR cmd: {Theta_FR}")
print(f"  HL cmd: {Theta_HL}")
print(f"  HR cmd: {Theta_HR}")

# ─────────────────────────────────────────────────────────────────────────────
# Initialization
# ─────────────────────────────────────────────────────────────────────────────
print("Starting the Robot")
print("Initializing CAN buses...")

ID_1 = 0x141
ID_2 = 0x142
ID_3 = 0x143

bus0 = can.interface.Bus(channel="can0", interface="socketcan")
bus1 = can.interface.Bus(channel="can1", interface="socketcan")
bus2 = can.interface.Bus(channel="can2", interface="socketcan")
bus3 = can.interface.Bus(channel="can3", interface="socketcan")

for bus in [bus0, bus1, bus2, bus3]:
    for _ in range(100):
        msg = bus.recv(0.01)
        if msg:
            print(msg)

# ─────────────────────────────────────────────────────────────────────────────
# Data logging setup
# ─────────────────────────────────────────────────────────────────────────────
SCRIPT_DIR   = os.path.dirname(os.path.abspath(__file__))
log_dir      = os.path.join(SCRIPT_DIR, "TEST_DATA_PID_28_04")
os.makedirs(log_dir, exist_ok=True)
timestamp_str = time.strftime('%Y-%m-%d_%H-%M-%S')
log_filename  = os.path.join(log_dir, f"Stand_Pose_TorquePos_Log_TEST_6_{timestamp_str}.csv")
pid_filename  = os.path.join(log_dir, f"Stand_Pose_TorquePos_Log_TEST_6_{timestamp_str}_PID.txt")

with open(log_filename, 'w', newline='') as csvfile:
    csv.writer(csvfile).writerow([
        "Timestamp (s)",
        # 12 read torques
        "FL_J1_Torque", "FL_J2_Torque", "FL_J3_Torque",
        "FR_J1_Torque", "FR_J2_Torque", "FR_J3_Torque",
        "HL_J1_Torque", "HL_J2_Torque", "HL_J3_Torque",
        "HR_J1_Torque", "HR_J2_Torque", "HR_J3_Torque",
        # 12 read positions
        "FL_J1_Pos (deg)", "FL_J2_Pos (deg)", "FL_J3_Pos (deg)",
        "FR_J1_Pos (deg)", "FR_J2_Pos (deg)", "FR_J3_Pos (deg)",
        "HL_J1_Pos (deg)", "HL_J2_Pos (deg)", "HL_J3_Pos (deg)",
        "HR_J1_Pos (deg)", "HR_J2_Pos (deg)", "HR_J3_Pos (deg)",
        # 12 position commands (constant IK setpoints)
        "FL_J1_Cmd (deg)", "FL_J2_Cmd (deg)", "FL_J3_Cmd (deg)",
        "FR_J1_Cmd (deg)", "FR_J2_Cmd (deg)", "FR_J3_Cmd (deg)",
        "HL_J1_Cmd (deg)", "HL_J2_Cmd (deg)", "HL_J3_Cmd (deg)",
        "HR_J1_Cmd (deg)", "HR_J2_Cmd (deg)", "HR_J3_Cmd (deg)",
    ])

# Preallocate: timestamp + 12 torques + 12 positions + 12 commands = 37 columns
data       = np.zeros((15000000, 37))
data_count = 0

# ─────────────────────────────────────────────────────────────────────────────
# Read and save PID parameters from all 12 motors
# ─────────────────────────────────────────────────────────────────────────────
print("Reading PID parameters from motors...")
_LEG_BUSES = [("FL", bus0), ("FR", bus1), ("HL", bus2), ("HR", bus3)]
_JOINT_IDS = [("J1", ID_1), ("J2", ID_2), ("J3", ID_3)]

with open(pid_filename, 'w') as _pid_file:
    _pid_file.write(f"PID Parameters — read {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
    _pid_file.write("=" * 52 + "\n\n")
    for leg, bus in _LEG_BUSES:
        _pid_file.write(f"Leg {leg}\n")
        _pid_file.write("-" * 30 + "\n")
        for joint, motor_id in _JOINT_IDS:
            result = Read_PID(bus, motor_id)
            if result is not None:
                pos_kp, pos_ki, spd_kp, spd_ki, torq_kp, torq_ki = result
                _pid_file.write(f"  {joint} (ID 0x{motor_id:03X}):\n")
                _pid_file.write(f"    Position Loop : Kp={pos_kp:5.0f}  Ki={pos_ki:5.0f}\n")
                _pid_file.write(f"    Speed Loop    : Kp={spd_kp:5.0f}  Ki={spd_ki:5.0f}\n")
                _pid_file.write(f"    Torque Loop   : Kp={torq_kp:5.0f}  Ki={torq_ki:5.0f}\n")
            else:
                _pid_file.write(f"  {joint} (ID 0x{motor_id:03X}): no response\n")
        _pid_file.write("\n")

print(f"PID parameters saved to {os.path.basename(pid_filename)}")

executor = concurrent.futures.ThreadPoolExecutor(max_workers=4)

# ─────────────────────────────────────────────────────────────────────────────
# Move to zero, then stand pose
# ─────────────────────────────────────────────────────────────────────────────
print("Moving to zero position...")
for bus in [bus0, bus1, bus2, bus3]:
    Position_Control(bus, ID_1, 0, 30)
    Position_Control(bus, ID_2, 0, 30)
    Position_Control(bus, ID_3, 0, 30)
time.sleep(2)

print("Moving to stand pose...")
for bus, angles in [(bus0, Theta_FL), (bus1, Theta_FR),
                    (bus2, Theta_HL), (bus3, Theta_HR)]:
    Position_Control(bus, ID_1, angles[0], 30)
    Position_Control(bus, ID_2, angles[1], 30)
    Position_Control(bus, ID_3, angles[2], 30)
time.sleep(5)

# ─────────────────────────────────────────────────────────────────────────────
# Main loop — target 200 Hz
# ─────────────────────────────────────────────────────────────────────────────
LOOP_PERIOD = 1.0 / 200.0
loop_times  = []
print("SET IN POSITION")
time.sleep(10)
print("\nPre-loop sequence complete.")
print(">>> STARTING in 0.5 seconds — remove your hands! <<<")
time.sleep(0.5)
LOG_DURATION = 20.0  # seconds to log before stopping

print("LOGGING STARTED — logging for 20 seconds. Press Ctrl+C to stop motors.")
start_time    = time.monotonic()
logging_active = True

try:
    while True:
        loop_start    = time.monotonic()
        elapsed_total = loop_start - start_time

        # ── Stop logging after LOG_DURATION, but keep motors running ───────
        if logging_active and elapsed_total >= LOG_DURATION:
            logging_active = False
            print(f"\nLOGGING STOPPED after {elapsed_total:.1f} s — {data_count} rows recorded.")
            print(f"Saving data to {os.path.basename(log_filename)}...")
            with open(log_filename, "a") as file:
                for i in range(data_count):
                    file.write(",".join(f"{v:.4f}" for v in data[i, :]) + "\n")
            print("Data saved. Actuators still running — press Ctrl+C to stop.")

        # ── Send + receive all 4 legs in parallel ──────────────────────────
        f_FL = executor.submit(send_leg, bus0, Theta_FL)
        f_FR = executor.submit(send_leg, bus1, Theta_FR)
        f_HL = executor.submit(send_leg, bus2, Theta_HL)
        f_HR = executor.submit(send_leg, bus3, Theta_HR)

        fb_FL = f_FL.result()
        fb_FR = f_FR.result()
        fb_HL = f_HL.result()
        fb_HR = f_HR.result()

        # ── Log torque + position + command ────────────────────────────────
        if logging_active:
            data[data_count, :] = [
                elapsed_total,
                # 12 read torques (Nm)
                safe_torque(fb_FL, 0), safe_torque(fb_FL, 1), safe_torque(fb_FL, 2),
                safe_torque(fb_FR, 0), safe_torque(fb_FR, 1), safe_torque(fb_FR, 2),
                safe_torque(fb_HL, 0), safe_torque(fb_HL, 1), safe_torque(fb_HL, 2),
                safe_torque(fb_HR, 0), safe_torque(fb_HR, 1), safe_torque(fb_HR, 2),
                # 12 read positions (deg, output shaft, multi-turn)
                safe_position(fb_FL, 0), safe_position(fb_FL, 1), safe_position(fb_FL, 2),
                safe_position(fb_FR, 0), safe_position(fb_FR, 1), safe_position(fb_FR, 2),
                safe_position(fb_HL, 0), safe_position(fb_HL, 1), safe_position(fb_HL, 2),
                safe_position(fb_HR, 0), safe_position(fb_HR, 1), safe_position(fb_HR, 2),
                # 12 position commands (deg, constant IK setpoints)
                Theta_FL[0], Theta_FL[1], Theta_FL[2],
                Theta_FR[0], Theta_FR[1], Theta_FR[2],
                Theta_HL[0], Theta_HL[1], Theta_HL[2],
                Theta_HR[0], Theta_HR[1], Theta_HR[2],
            ]
            data_count += 1

        loop_elapsed = time.monotonic() - loop_start
        loop_times.append(loop_elapsed)
        sleep_time = LOOP_PERIOD - loop_elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)

except KeyboardInterrupt:
    print("\nKeyboardInterrupt received, shutting down...")
    executor.shutdown(wait=False)

    # ── Loop timing diagnostic ──────────────────────────────────────────────
    if loop_times:
        arr = np.array(loop_times) * 1000
        print(f"\nLoop timing over {len(arr)} cycles:")
        print(f"  Mean:  {arr.mean():.2f} ms  ({1000/arr.mean():.1f} Hz)")
        print(f"  Min:   {arr.min():.2f} ms")
        print(f"  Max:   {arr.max():.2f} ms")
        print(f"  >5 ms: {(arr > 5).sum()} cycles ({100*(arr>5).mean():.1f}%)\n")

    # ── Stop motors ─────────────────────────────────────────────────────────
    print("Stopping motors...")
    for bus in [bus0, bus1, bus2, bus3]:
        while bus.recv(timeout=0.001) is not None:
            pass
    for bus in [bus0, bus1, bus2, bus3]:
        for motor_id in [ID_1, ID_2, ID_3]:
            try:
                Motor_Stop(bus, motor_id)
            except RuntimeError as e:
                print(f"  Warning: {e}")
    print("Motors stopped.")

    print("Shutting down CAN buses...")
    for bus in [bus0, bus1, bus2, bus3]:
        bus.shutdown()
        time.sleep(0.01)
    print("CAN buses shut down.")

    # ── Write logged data (only if logging was interrupted before 20 s) ─────
    if logging_active and data_count > 0:
        print("Storing logged data to file...")
        with open(log_filename, "a") as file:
            for i in range(data_count):
                file.write(",".join(f"{v:.4f}" for v in data[i, :]) + "\n")
        print(f"Logged {data_count} rows to {log_filename}")
    print("Shutdown complete.")