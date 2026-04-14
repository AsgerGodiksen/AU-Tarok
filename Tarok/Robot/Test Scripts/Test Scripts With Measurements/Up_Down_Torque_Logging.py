# Script for up/down movement with torque data logging for all 12 motors.
# Combines trajectory from Test_Up_Down_Full.py with parallel feedback/logging
# pattern from Stand_Pose_Torque_3.py.

# Imports
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
# Position_Control_With_Feedback  (same as Stand_Pose_Torque_3.py)
# ─────────────────────────────────────────────────────────────────────────────
def Position_Control_With_Feedback(bus, motor_id, new_position, max_speed, timeout=0.005):
    """
    Send a position command and return the torque feedback from the reply.
    Returns (torque_current_A, torque_Nm), or (None, None) on timeout.
    """
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

    msg = can.Message(arbitration_id=motor_id, data=data, is_extended_id=False)
    for attempt in range(3):
        try:
            bus.send(msg)
            break
        except can.CanOperationError:
            time.sleep(0.001)  # give the HW buffer a moment to drain
    else:
        return None, None  # all retries failed — treat like a timeout

    deadline = time.monotonic() + timeout
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return None, None
        msg = bus.recv(remaining)
        if msg is None:
            return None, None
        if msg.arbitration_id == motor_id and msg.data[0] == 0xA4:
            break

    iq_raw = struct.unpack('<h', bytes([msg.data[2], msg.data[3]]))[0]
    torque_current_A = (iq_raw / 2048.0) * 33.0
    torque_Nm        = torque_current_A * 2.09
    return torque_current_A, torque_Nm


def send_leg(bus, angles, speeds):
    """
    Command one leg's 3 motors with individual speeds and collect torque feedback.
    Returns list of (torque_current_A, torque_Nm) for [J1, J2, J3].
    """
    return [
        Position_Control_With_Feedback(bus, motor_id, angle, speed)
        for motor_id, angle, speed in zip([ID_1, ID_2, ID_3], angles, speeds)
    ]


def safe_torque(fb, i):
    """Return torque_Nm from feedback list, 0.0 on timeout (None)."""
    val = fb[i][1]
    return val if val is not None else 0.0


# ─────────────────────────────────────────────────────────────────────────────
# Pre-computations  (from Test_Up_Down_Full.py)
# ─────────────────────────────────────────────────────────────────────────────
print("Performing pre-computations...")

l_k = 0.7048
w_k = 0.220

dt = 0.005  # 200 Hz
total_time = 10
num_time_steps = int(total_time / dt) + 1
t = np.linspace(0, total_time, num_time_steps)

# End-effector trajectory (body frame)
x_FL = x_FR = (l_k / 2) * np.ones_like(t)
x_HL = x_HR = (-l_k / 2) * np.ones_like(t)
y_FL = y_HL = (w_k / 2 + 0.078) * np.ones_like(t)
y_FR = y_HR = (-w_k / 2 - 0.078) * np.ones_like(t)
z = np.piecewise(t, [t < 5, t >= 5],
                 [lambda t: -0.46 + (0.10 / 5) * t,
                  lambda t: -0.36 - (0.10 / 5) * (t - 5)])

# End-effector velocity (body frame)
x_dot = np.zeros_like(t)
y_dot = np.zeros_like(t)
z_dot = np.piecewise(t, [t < 5, t >= 5],
                     [lambda t: 0.10 / 5 * np.ones_like(t),
                      lambda t: -0.10 / 5 * np.ones_like(t)])

### Transformations ###
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

### Kinematics ###
Theta_FL = np.array([Inverse_Kinematics(P_FL_base[i], 'FL') for i in range(len(t))])
Theta_FR = np.array([Inverse_Kinematics(P_FR_base[i], 'FR') for i in range(len(t))])
Theta_HL = np.array([Inverse_Kinematics(P_HL_base[i], 'HL') for i in range(len(t))])
Theta_HR = np.array([Inverse_Kinematics(P_HR_base[i], 'HR') for i in range(len(t))])

# Joint velocities via damped least squares
Theta_dot_FL = np.zeros((3, len(t)))
Theta_dot_FR = np.zeros((3, len(t)))
Theta_dot_HL = np.zeros((3, len(t)))
Theta_dot_HR = np.zeros((3, len(t)))
damp = 0.001
for i in range(len(t)):
    for Th, Th_dot, V_base, leg in [
        (Theta_FL, Theta_dot_FL, V_FL_base, 'FL'),
        (Theta_FR, Theta_dot_FR, V_FR_base, 'FR'),
        (Theta_HL, Theta_dot_HL, V_HL_base, 'HL'),
        (Theta_HR, Theta_dot_HR, V_HR_base, 'HR'),
    ]:
        J = Jacobian(Th[i, 0], Th[i, 1], Th[i, 2], leg)
        JT = J.T
        Th_dot[:, i] = np.linalg.solve(JT @ J + (damp**2) * np.eye(3),
                                        JT @ V_base[i].flatten())

# Convert to degrees
Theta_FL = np.rad2deg(Theta_FL)
Theta_FR = np.rad2deg(Theta_FR)
Theta_HL = np.rad2deg(Theta_HL)
Theta_HR = np.rad2deg(Theta_HR)
Theta_dot_FL = np.abs(np.rad2deg(Theta_dot_FL))
Theta_dot_FR = np.abs(np.rad2deg(Theta_dot_FR))
Theta_dot_HL = np.abs(np.rad2deg(Theta_dot_HL))
Theta_dot_HR = np.abs(np.rad2deg(Theta_dot_HR))

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
# Data logging setup
# ─────────────────────────────────────────────────────────────────────────────
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
log_dir = os.path.join(SCRIPT_DIR, "TEST_DATA_PID_UP_DOWN")
os.makedirs(log_dir, exist_ok=True)
log_filename = os.path.join(
    log_dir, f"Up_Down_Torque_Log_Test_1_{time.strftime('%Y-%m-%d_%H-%M-%S')}.csv"
)

with open(log_filename, 'w', newline='') as csvfile:
    csv.writer(csvfile).writerow([
        "Timestamp (s)", "Trajectory Index",
        "FL_J1", "FL_J2", "FL_J3",
        "FR_J1", "FR_J2", "FR_J3",
        "HL_J1", "HL_J2", "HL_J3",
        "HR_J1", "HR_J2", "HR_J3",
    ])

# Preallocate: timestamp + index + 12 torques = 14 columns
data = np.zeros((1500000, 14))
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
time.sleep(6)

print("Moving to initial trajectory position...")
for bus, angles in [(bus0, Theta_FL[0]), (bus1, Theta_FR[0]),
                    (bus2, Theta_HL[0]), (bus3, Theta_HR[0])]:
    Position_Control(bus, ID_1, angles[0], 30)
    Position_Control(bus, ID_2, angles[1], 30)
    Position_Control(bus, ID_3, angles[2], 30)
time.sleep(6)

# ─────────────────────────────────────────────────────────────────────────────
# Main loop — cyclic up/down trajectory with torque logging
# ─────────────────────────────────────────────────────────────────────────────
LOOP_PERIOD = 1.0 / 200.0
loop_times = []

print("Pre-loop sequence complete, starting loop...")
print("Loop started - Press Ctrl+C in terminal for shutdown")
start_time = cycle_start = time.monotonic()

try:
    while True:
        loop_start = time.monotonic()
        elapsed_total = loop_start - start_time
        elapsed_cycle = loop_start - cycle_start

        # ── Precise timing ──────────────────────────────────────────────────
        loop_elapsed = time.monotonic() - loop_start
        loop_times.append(loop_elapsed)
        sleep_time = LOOP_PERIOD - loop_elapsed
        time.sleep(max(sleep_time, 0.001))  # always sleep at least 1 ms

        # Check if current cycle is over -> start new cycle
        if elapsed_cycle >= total_time:
            cycle_start += total_time
            continue

        # Find closest trajectory index
        index = min(int(elapsed_cycle / dt), len(t) - 1)

        # ── Send + receive all 4 legs in parallel ──────────────────────────
        f_FL = executor.submit(send_leg, bus0,
                               Theta_FL[index], Theta_dot_FL[:, index])
        f_FR = executor.submit(send_leg, bus1,
                               Theta_FR[index], Theta_dot_FR[:, index])
        f_HL = executor.submit(send_leg, bus2,
                               Theta_HL[index], Theta_dot_HL[:, index])
        f_HR = executor.submit(send_leg, bus3,
                               Theta_HR[index], Theta_dot_HR[:, index])

        fb_FL = f_FL.result()
        fb_FR = f_FR.result()
        fb_HL = f_HL.result()
        fb_HR = f_HR.result()

        # ── Log torque data ─────────────────────────────────────────────────
        data[data_count, :] = [
            elapsed_total, index,
            safe_torque(fb_FL, 0), safe_torque(fb_FL, 1), safe_torque(fb_FL, 2),
            safe_torque(fb_FR, 0), safe_torque(fb_FR, 1), safe_torque(fb_FR, 2),
            safe_torque(fb_HL, 0), safe_torque(fb_HL, 1), safe_torque(fb_HL, 2),
            safe_torque(fb_HR, 0), safe_torque(fb_HR, 1), safe_torque(fb_HR, 2),
        ]
        data_count += 1

        # ── Precise timing ──────────────────────────────────────────────────
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