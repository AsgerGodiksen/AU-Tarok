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
# Position_Control_With_Feedback
#
# Identical CAN send to Position_Control, but parses and returns the torque
# feedback embedded in the motor's 0xA4 reply — so we get torque data
# without any extra CAN round-trip.
#
# Why this replaces the listener thread:
#   The old design had a separate thread calling Read_Torque_Current (command
#   0x9C) on bus0 while the main loop also sent Position_Control (0xA4) on
#   bus0. Their replies interleaved in the RX buffer, causing missed reads and
#   bus contention. By parsing the torque from the 0xA4 reply we already
#   receive, we eliminate the extra reads and the thread entirely.
#
# TODO: move to Motor_Controls.py once validated.
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




def send_leg(bus, angles):
    """
    Command one leg's 3 motors and collect their torque feedback.
    This function runs in its own thread (one per bus) so all 4 legs
    are commanded in parallel rather than sequentially.
    Returns list of (torque_current_A, torque_Nm) for [J1, J2, J3].
    """
    return [
        Position_Control_With_Feedback(bus, motor_id, angle, 20)
        for motor_id, angle in zip([ID_1, ID_2, ID_3], angles)
    ]


# ─────────────────────────────────────────────────────────────────────────────
# Pre-computations
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

# Drain any stale messages
for bus in [bus0, bus1, bus2, bus3]:
    for _ in range(100):
        msg = bus.recv(0.01)
        if msg:
            print(msg)

# ─────────────────────────────────────────────────────────────────────────────
# Data logging setup — now logging all 12 motor torques
# ─────────────────────────────────────────────────────────────────────────────
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
log_dir = os.path.join(SCRIPT_DIR, "TEST_DATA")
os.makedirs(log_dir, exist_ok=True)
log_filename = os.path.join(
    log_dir, f"Stand_Pose_Torque_Log_{time.strftime('%Y-%m-%d_%H-%M-%S')}.csv"
)

with open(log_filename, 'w', newline='') as csvfile:
    csv.writer(csvfile).writerow([
        "Timestamp (s)",
        "FL_J1", "FL_J2", "FL_J3",
        "FR_J1", "FR_J2", "FR_J3",
        "HL_J1", "HL_J2", "HL_J3",
        "HR_J1", "HR_J2", "HR_J3",
    ])

# Preallocate: timestamp + 12 torque values = 13 columns
data = np.zeros((15000000, 13))
data_count = 0

# Thread pool: 4 workers, one per bus, created once and reused every cycle
executor = concurrent.futures.ThreadPoolExecutor(max_workers=4)

# ─────────────────────────────────────────────────────────────────────────────
# Move to zero, then stand pose (sequential — startup only, speed not critical)
# ─────────────────────────────────────────────────────────────────────────────
print("Initialization complete, moving to zero position")
for bus in [bus0, bus1, bus2, bus3]:
    Position_Control(bus, ID_1, 0, 30)
    Position_Control(bus, ID_2, 0, 30)
    Position_Control(bus, ID_3, 0, 30)
time.sleep(5)

print("Moved to zero position, moving to stand pose...")
for bus, angles in [(bus0, Theta_FL), (bus1, Theta_FR),
                    (bus2, Theta_HL), (bus3, Theta_HR)]:
    Position_Control(bus, ID_1, angles[0], 30)
    Position_Control(bus, ID_2, angles[1], 30)
    Position_Control(bus, ID_3, angles[2], 30)
time.sleep(6)

# ─────────────────────────────────────────────────────────────────────────────
# Main loop — target 200 Hz (5 ms budget per cycle)
#
# Each cycle:
#   1. Submit send_leg() for all 4 buses concurrently (ThreadPoolExecutor).
#      Each thread does 3 CAN round-trips on its own bus — no contention.
#   2. Collect results (torque feedback from every motor).
#   3. Log data.
#   4. Sleep only the remaining time in the 5 ms window (precise timing).
#
# Achievable frequency depends on CAN round-trip latency. On RPi5 with
# socketcan at 1 Mbit/s, each round-trip is typically ~1–2 ms, so 3 motors
# per bus ≈ 3–6 ms per bus. The 4 buses run in parallel, so loop time is
# set by the slowest bus. If you consistently exceed 5 ms, lower the target
# to LOOP_PERIOD = 1/150 and investigate latency with the diagnostic below.
# ─────────────────────────────────────────────────────────────────────────────
LOOP_PERIOD = 1.0 / 200.0   # 5 ms

# Diagnostic: track actual loop timing (printed on shutdown)
loop_times = []

print("Pre-loop sequence complete, starting loop...")
print("Loop started - Press Ctrl+C in terminal for shutdown")
start_time = time.monotonic()

def safe_torque(fb, i):
    """Return torque_Nm from feedback list, 0.0 on timeout (None)."""
    val = fb[i][1]
    return val if val is not None else 0.0

try:
    while True:
        loop_start = time.monotonic()
        elapsed_total = loop_start - start_time

        loop_elapsed = time.monotonic() - loop_start
        loop_times.append(loop_elapsed)
        sleep_time = LOOP_PERIOD - loop_elapsed
        time.sleep(max(sleep_time, 0.001))  # always sleep at least 1 ms

        # ── Send + receive all 4 legs in parallel ──────────────────────────
        f_FL = executor.submit(send_leg, bus0, Theta_FL)
        f_FR = executor.submit(send_leg, bus1, Theta_FR)
        f_HL = executor.submit(send_leg, bus2, Theta_HL)
        f_HR = executor.submit(send_leg, bus3, Theta_HR)

        fb_FL = f_FL.result()
        fb_FR = f_FR.result()
        fb_HL = f_HL.result()
        fb_HR = f_HR.result()

        # ── Log torque data ─────────────────────────────────────────────────
        data[data_count, :] = [
            elapsed_total,
            safe_torque(fb_FL, 0), safe_torque(fb_FL, 1), safe_torque(fb_FL, 2),
            safe_torque(fb_FR, 0), safe_torque(fb_FR, 1), safe_torque(fb_FR, 2),
            safe_torque(fb_HL, 0), safe_torque(fb_HL, 1), safe_torque(fb_HL, 2),
            safe_torque(fb_HR, 0), safe_torque(fb_HR, 1), safe_torque(fb_HR, 2),
        ]
        data_count += 1

        # ── Precise timing: sleep only what remains of the 5 ms window ─────
        loop_elapsed = time.monotonic() - loop_start
        loop_times.append(loop_elapsed)
        sleep_time = LOOP_PERIOD - loop_elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)
        # If sleep_time <= 0 the loop is already over budget — no sleep,
        # next cycle starts immediately. The diagnostic will show this.

except KeyboardInterrupt:
    print("\nKeyboardInterrupt received, shutting down...")
    executor.shutdown(wait=False)

    # ── Print loop timing diagnostic ────────────────────────────────────────
    if loop_times:
        arr = np.array(loop_times) * 1000  # convert to ms
        print(f"\nLoop timing over {len(arr)} cycles:")
        print(f"  Mean:  {arr.mean():.2f} ms  ({1000/arr.mean():.1f} Hz)")
        print(f"  Min:   {arr.min():.2f} ms")
        print(f"  Max:   {arr.max():.2f} ms")
        print(f"  >5 ms: {(arr > 5).sum()} cycles ({100*(arr>5).mean():.1f}%)\n")

    # ── Stop motors ─────────────────────────────────────────────────────────
    print("Stopping motors...")
    for bus in [bus0, bus1, bus2, bus3]:
        while bus.recv(timeout=0.001) is not None:
            pass  # drain stale replies
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