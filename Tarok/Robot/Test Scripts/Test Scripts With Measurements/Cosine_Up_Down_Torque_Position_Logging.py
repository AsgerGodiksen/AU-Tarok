# Script for up/down movement using cosine interpolation with torque, position AND command data logging for all 12 motors.
# Trajectory from Test_Up_Down_Cosine.py; logging infrastructure from Up_Down_Torque_Position_Logging.py.
#
# Each motor step does two CAN round trips:
#   1. 0xA4  — position command → torque decoded from reply bytes [2-3]
#   2. 0x92  — read multi-turn angle → signed 56-bit angle, 0.01 deg/LSB motor shaft → /900 = output shaft deg
#
# CSV columns:
#   Timestamp (s), Trajectory Index,
#   FL_J1_Torque .. HR_J3_Torque  (Nm,  from 0xA4 reply),
#   FL_J1_Pos    .. HR_J3_Pos     (deg, output shaft, multi-turn, from 0x92 reply),
#   FL_J1_Cmd    .. HR_J3_Cmd     (deg, analytical trajectory command sent to motor)

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
# Position_Control_With_Feedback  — returns torque AND actual position
# ─────────────────────────────────────────────────────────────────────────────
def Position_Control_With_Feedback(bus, motor_id, new_position, max_speed, timeout=0.005):
    """
    Send a 0xA4 position command and a 0x92 read command per motor step.

    Step 1 — 0xA4 position command:
      Reply bytes [2-3]: iq current (int16) → torque_Nm

    Step 2 — 0x92 read multi-turn angle (mirrors Read_Angle in Motor_Readings.py):
      Reply bytes [1-7]: signed 56-bit angle, 0.01 deg/LSB motor shaft → / 900 = output deg

    Returns (torque_Nm, angle_deg), or (None, None) on timeout.
    """
    # ── Step 1: send 0xA4 position command, read torque from reply ────────────
    data = [0xA4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]

    speed_raw = int(max_speed * 9)
    speed_bytes = struct.pack('<h', speed_raw)
    data[2] = speed_bytes[0]
    data[3] = speed_bytes[1]

    if new_position >= 0:
        pos_enc = int(new_position * 100 * 9)
        pos_bytes = struct.pack('<i', pos_enc)
    else:
        pos_enc = int(new_position * 100 * 9) & 0xFFFFFFFF
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

    deadline = time.monotonic() + timeout
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

    # ── Step 2: send 0x92 read angle command, decode multi-turn position ─────
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
            raw_bytes  = bytes(reply.data[1:8]) + b'\x00'  # pad to 8 bytes for int64
            angle_raw  = struct.unpack('<q', raw_bytes)[0]
            if reply.data[7] & 0x80:                        # sign-extend from bit 55
                angle_raw -= (1 << 56)
            angle_deg = float(angle_raw) / 900.0            # output shaft degrees
            return torque_Nm, angle_deg


def send_leg(bus, angles, speeds):
    """
    Command one leg's 3 motors and collect torque + position feedback.
    Returns list of (torque_Nm, position_deg) for [J1, J2, J3].
    """
    return [
        Position_Control_With_Feedback(bus, motor_id, angle, speed)
        for motor_id, angle, speed in zip([ID_1, ID_2, ID_3], angles, speeds)
    ]


def safe_torque(fb, i):
    """Return torque_Nm from feedback, 0.0 on timeout."""
    val = fb[i][0]
    return val if val is not None else 0.0


def safe_position(fb, i):
    """Return encoder position (deg) from feedback, 0 on timeout."""
    val = fb[i][1]
    return val if val is not None else 0


# ─────────────────────────────────────────────────────────────────────────────
# Cosine interpolation helpers
# ─────────────────────────────────────────────────────────────────────────────
def cos_interp(t, z_start, z_end, t_start, t_end):
    """Smooth cosine interpolation from z_start to z_end over [t_start, t_end]."""
    tau = (t - t_start) / (t_end - t_start)
    return z_start + (z_end - z_start) * 0.5 * (1 - np.cos(np.pi * tau))

def cos_interp_dot(t, z_start, z_end, t_start, t_end):
    """Derivative of cosine interpolation."""
    duration = t_end - t_start
    tau = (t - t_start) / duration
    return (z_end - z_start) * 0.5 * np.pi / duration * np.sin(np.pi * tau)


# ─────────────────────────────────────────────────────────────────────────────
# Pre-computations — cosine up/down trajectory
# ─────────────────────────────────────────────────────────────────────────────
print("Performing pre-computations...")

l_k = 0.7048
w_k = 0.220

dt = 0.005  # 200 Hz
total_time = 6  # seconds
num_time_steps = int(total_time / dt) + 1
t = np.linspace(0, total_time, num_time_steps)

# Segment boundaries: 0->3s (down), 3->6s (up), >=6s (hold)
conditions = [(t >= 0) & (t < 3),
              (t >= 3) & (t < 6),
              (t >= 6)]

x_FL = x_FR = (l_k / 2) * np.ones_like(t)
x_HL = x_HR = (-l_k / 2) * np.ones_like(t)
y_FL = y_HL = (w_k / 2 + 0.078) * np.ones_like(t)
y_FR = y_HR = (-w_k / 2 - 0.078) * np.ones_like(t)

z = np.piecewise(t, conditions, [lambda t: cos_interp(t, -0.36, -0.46, 0, 3),
                                 lambda t: cos_interp(t, -0.46, -0.36, 3, 6),
                                 lambda t: -0.36 * np.ones_like(t)])

x_dot = np.zeros_like(t)
y_dot = np.zeros_like(t)
z_dot = np.piecewise(t, conditions, [lambda t: cos_interp_dot(t, -0.36, -0.46, 0, 3),
                                     lambda t: cos_interp_dot(t, -0.46, -0.36, 3, 6),
                                     lambda t: np.zeros_like(t)])

P_FL_body = np.vstack((x_FL, y_FL, z))
P_FR_body = np.vstack((x_FR, y_FR, z))
P_HL_body = np.vstack((x_HL, y_HL, z))
P_HR_body = np.vstack((x_HR, y_HR, z))

P_FL_base = np.array([T0_B(P_FL_body[:, i].reshape((3, 1)), 'FL') for i in range(len(t))])
P_FR_base = np.array([T0_B(P_FR_body[:, i].reshape((3, 1)), 'FR') for i in range(len(t))])
P_HL_base = np.array([T0_B(P_HL_body[:, i].reshape((3, 1)), 'HL') for i in range(len(t))])
P_HR_base = np.array([T0_B(P_HR_body[:, i].reshape((3, 1)), 'HR') for i in range(len(t))])

V_body = np.vstack((x_dot, y_dot, z_dot))

V_FL_base = np.array([R0_B(V_body[:, i].reshape((3, 1)), 'FL') for i in range(len(t))])
V_FR_base = np.array([R0_B(V_body[:, i].reshape((3, 1)), 'FR') for i in range(len(t))])
V_HL_base = np.array([R0_B(V_body[:, i].reshape((3, 1)), 'HL') for i in range(len(t))])
V_HR_base = np.array([R0_B(V_body[:, i].reshape((3, 1)), 'HR') for i in range(len(t))])

Theta_FL = np.array([Inverse_Kinematics(P_FL_base[i], 'FL') for i in range(len(t))])
Theta_FR = np.array([Inverse_Kinematics(P_FR_base[i], 'FR') for i in range(len(t))])
Theta_HL = np.array([Inverse_Kinematics(P_HL_base[i], 'HL') for i in range(len(t))])
Theta_HR = np.array([Inverse_Kinematics(P_HR_base[i], 'HR') for i in range(len(t))])

Theta_dot_FL = np.zeros((3, len(t)))
Theta_dot_FR = np.zeros((3, len(t)))
Theta_dot_HL = np.zeros((3, len(t)))
Theta_dot_HR = np.zeros((3, len(t)))
damp = 0.001
for i in range(len(t)):
    Jac_i_FL = Jacobian(Theta_FL[i, 0], Theta_FL[i, 1], Theta_FL[i, 2], 'FL')
    Jac_i_FR = Jacobian(Theta_FR[i, 0], Theta_FR[i, 1], Theta_FR[i, 2], 'FR')
    Jac_i_HL = Jacobian(Theta_HL[i, 0], Theta_HL[i, 1], Theta_HL[i, 2], 'HL')
    Jac_i_HR = Jacobian(Theta_HR[i, 0], Theta_HR[i, 1], Theta_HR[i, 2], 'HR')
    JT_FL = Jac_i_FL.T
    JT_FR = Jac_i_FR.T
    JT_HL = Jac_i_HL.T
    JT_HR = Jac_i_HR.T
    Theta_dot_FL[:, i] = np.linalg.solve(JT_FL @ Jac_i_FL + (damp**2) * np.eye(3), JT_FL @ V_FL_base[i].flatten())
    Theta_dot_FR[:, i] = np.linalg.solve(JT_FR @ Jac_i_FR + (damp**2) * np.eye(3), JT_FR @ V_FR_base[i].flatten())
    Theta_dot_HL[:, i] = np.linalg.solve(JT_HL @ Jac_i_HL + (damp**2) * np.eye(3), JT_HL @ V_HL_base[i].flatten())
    Theta_dot_HR[:, i] = np.linalg.solve(JT_HR @ Jac_i_HR + (damp**2) * np.eye(3), JT_HR @ V_HR_base[i].flatten())

Theta_FL = np.rad2deg(Theta_FL)
Theta_FR = np.rad2deg(Theta_FR)
Theta_HL = np.rad2deg(Theta_HL)
Theta_HR = np.rad2deg(Theta_HR)
Theta_dot_FL = np.abs(np.rad2deg(Theta_dot_FL))
Theta_dot_FR = np.abs(np.rad2deg(Theta_dot_FR))
Theta_dot_HL = np.abs(np.rad2deg(Theta_dot_HL))
Theta_dot_HR = np.abs(np.rad2deg(Theta_dot_HR))

# Roll arrays so the trajectory starts at the standing position (z = -0.36)
roll_amount = num_time_steps - int((3/4 * total_time) / dt)
Theta_FL = np.roll(Theta_FL, roll_amount, axis=0)
Theta_FR = np.roll(Theta_FR, roll_amount, axis=0)
Theta_HL = np.roll(Theta_HL, roll_amount, axis=0)
Theta_HR = np.roll(Theta_HR, roll_amount, axis=0)
Theta_dot_FL = np.roll(Theta_dot_FL, roll_amount, axis=1)
Theta_dot_FR = np.roll(Theta_dot_FR, roll_amount, axis=1)
Theta_dot_HL = np.roll(Theta_dot_HL, roll_amount, axis=1)
Theta_dot_HR = np.roll(Theta_dot_HR, roll_amount, axis=1)

print("Pre-computations complete.")

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
# Write PI parameters to all 12 motors
# ─────────────────────────────────────────────────────────────────────────────
PI_Params = {
    'angle_kp':  110,
    'angle_ki':  50,
    'speed_kp':  55,
    'speed_ki':  20,
    'torque_kp': 55,
    'torque_ki': 25
}

print("Writing PI parameters to motors...")
for bus in [bus0, bus1, bus2, bus3]:
    for motor_id in [ID_1, ID_2, ID_3]:
        PID_RAM_Control(bus, motor_id, PI_Params)

time.sleep(0.2)

# ─────────────────────────────────────────────────────────────────────────────
# Data logging setup
# ─────────────────────────────────────────────────────────────────────────────
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
log_dir = os.path.join(SCRIPT_DIR, "TEST_DATA_COSINE_UP_DOWN")
os.makedirs(log_dir, exist_ok=True)
timestamp_str = time.strftime('%Y-%m-%d_%H-%M-%S')
log_filename = os.path.join(log_dir, f"Cosine_Up_Down_TorquePos_Log_{timestamp_str}.csv")
pid_filename = os.path.join(log_dir, f"Cosine_Up_Down_TorquePos_Log_{timestamp_str}_PID.txt")

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

with open(log_filename, 'w', newline='') as csvfile:
    csv.writer(csvfile).writerow([
        "Timestamp (s)", "Trajectory Index",
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
        # 12 position commands (analytical trajectory setpoints)
        "FL_J1_Cmd (deg)", "FL_J2_Cmd (deg)", "FL_J3_Cmd (deg)",
        "FR_J1_Cmd (deg)", "FR_J2_Cmd (deg)", "FR_J3_Cmd (deg)",
        "HL_J1_Cmd (deg)", "HL_J2_Cmd (deg)", "HL_J3_Cmd (deg)",
        "HR_J1_Cmd (deg)", "HR_J2_Cmd (deg)", "HR_J3_Cmd (deg)",
    ])

# Preallocate: timestamp + index + 12 torques + 12 positions + 12 commands = 38 columns
data = np.zeros((1500000, 38))
data_count = 0

executor = concurrent.futures.ThreadPoolExecutor(max_workers=4)

# ─────────────────────────────────────────────────────────────────────────────
# Move to zero, then initial trajectory position
# ─────────────────────────────────────────────────────────────────────────────
print("Moving to zero position...")
for bus in [bus0, bus1, bus2, bus3]:
    Position_Control(bus, ID_1, 0, 30)
    Position_Control(bus, ID_2, 0, 30)
    Position_Control(bus, ID_3, 0, 30)
time.sleep(4)

print("Moving to initial trajectory position...")
for bus, angles in [(bus0, Theta_FL[0]), (bus1, Theta_FR[0]),
                    (bus2, Theta_HL[0]), (bus3, Theta_HR[0])]:
    Position_Control(bus, ID_1, angles[0], 30)
    Position_Control(bus, ID_2, angles[1], 30)
    Position_Control(bus, ID_3, angles[2], 30)
time.sleep(4)

# ─────────────────────────────────────────────────────────────────────────────
# Main loop — cyclic cosine up/down trajectory with torque + position + command logging
# ─────────────────────────────────────────────────────────────────────────────
LOOP_PERIOD = 1.0 / 200.0
NUM_CYCLES  = 3          # robot performs this many up/down cycles then holds still
loop_times  = []
cycle_count = 0
print("SET IN POSITION")
time.sleep(10)
print("\nPre-loop sequence complete.")
print(">>> STARTING in 0.5 seconds — remove your hands! <<<")
time.sleep(0.5)

print("Pre-loop sequence complete, starting loop...")
print(f"Loop started — robot will complete {NUM_CYCLES} cycles then hold still.")
print("Press Ctrl+C at any time to shut down.")
start_time = cycle_start = time.monotonic()

try:
    while True:
        loop_start = time.monotonic()
        elapsed_total = loop_start - start_time
        elapsed_cycle = loop_start - cycle_start

        if elapsed_cycle >= total_time:
            cycle_start += total_time
            cycle_count += 1

            if cycle_count >= NUM_CYCLES:
                print("\n" + "=" * 54)
                print(f"  TEST COMPLETE — {NUM_CYCLES} cycles finished.")
                print("  Data logging has stopped.")
                print("  Robot is holding the start position.")
                print("  It is now safe to lift the robot back into the holder.")
                print("  Press Ctrl+C to shut down the motors.")
                print("=" * 54 + "\n")

                hold_angles = [
                    (bus0, Theta_FL[0]),
                    (bus1, Theta_FR[0]),
                    (bus2, Theta_HL[0]),
                    (bus3, Theta_HR[0]),
                ]
                while True:
                    for bus, angles in hold_angles:
                        Position_Control(bus, ID_1, angles[0], 30)
                        Position_Control(bus, ID_2, angles[1], 30)
                        Position_Control(bus, ID_3, angles[2], 30)
                    time.sleep(0.1)

            continue

        index = min(int(elapsed_cycle / dt), len(t) - 1)

        # ── Send + receive all 4 legs in parallel ──────────────────────────
        f_FL = executor.submit(send_leg, bus0, Theta_FL[index], Theta_dot_FL[:, index])
        f_FR = executor.submit(send_leg, bus1, Theta_FR[index], Theta_dot_FR[:, index])
        f_HL = executor.submit(send_leg, bus2, Theta_HL[index], Theta_dot_HL[:, index])
        f_HR = executor.submit(send_leg, bus3, Theta_HR[index], Theta_dot_HR[:, index])

        fb_FL = f_FL.result()
        fb_FR = f_FR.result()
        fb_HL = f_HL.result()
        fb_HR = f_HR.result()

        # ── Log torque + position + command ────────────────────────────────
        data[data_count, :] = [
            elapsed_total, index,
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
            # 12 position commands (deg, analytical trajectory setpoints)
            Theta_FL[index, 0], Theta_FL[index, 1], Theta_FL[index, 2],
            Theta_FR[index, 0], Theta_FR[index, 1], Theta_FR[index, 2],
            Theta_HL[index, 0], Theta_HL[index, 1], Theta_HL[index, 2],
            Theta_HR[index, 0], Theta_HR[index, 1], Theta_HR[index, 2],
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
    print("CAN buses shut down.")

    # ── Write logged data ────────────────────────────────────────────────────
    print("Storing logged data to file...")
    with open(log_filename, "a") as file:
        for i in range(data_count):
            file.write(",".join(f"{v:.4f}" for v in data[i, :]) + "\n")
    print(f"Logged {data_count} rows to {log_filename}")
    print("Shutdown complete.")
