# Trajectory_Up_Down.py
# Controls the FRONT LEFT (FL) leg via CAN using RMD X8-v2 actuators
# Motors: Hip=ID_HIP, Thigh=ID_THIGH, Knee=ID_KNEE — all on can0
#
# Trajectory: body height sweeps 46 cm → 34 cm → 46 cm over 10 seconds
#
# Key fix for smooth motion:
#   After each Position_Control send, bus.recv() is called to read the
#   motor's acknowledgement. This naturally throttles the send rate to
#   what the CAN bus can handle, preventing TX buffer overflow and the
#   resulting jumpy motion.
'''
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..')))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../Hardware')))
import can
import numpy as np
from time import sleep
import time

from Robot.Hardware.Motor_Controls import Position_Control, Motor_Stop
from Robot.Kinematics.Inverse_Kinematics import Inverse_Kinematics
from Robot.Kinematics.Jacobian import Jacobian

# ─────────────────────────────────────────────
# CONFIGURATION
# ─────────────────────────────────────────────
L1 = 0.078   # Hip abduction offset (m)
L2 = 0.20    # Upper leg (m)
L3 = 0.30    # Lower leg (m)

ID_HIP   = 0x141
ID_THIGH = 0x142
ID_KNEE  = 0x143

dt         = 0.005        
total_time = 10.0
num_steps  = int(total_time / dt) + 1
t_arr      = np.linspace(0, total_time, num_steps)

# recv timeout per motor per step — 3 motors × 0.01 s = ~30 ms per loop iteration
# giving an effective command rate of ~33 Hz, well within CAN bus limits
RECV_TIMEOUT = 0.001   # seconds

# ─────────────────────────────────────────────
# SINUSOIDAL TRAJECTORY
# Cosine gives zero velocity at top/bottom — no abrupt direction reversal
#   x(t) = mean + amplitude * cos(2π/T * t)
# ─────────────────────────────────────────────
x_mean = (0.46 + 0.34) / 2
x_amp  = (0.46 - 0.34) / 2
omega  = 2 * np.pi / total_time

x_arr = x_mean + x_amp * np.cos(omega * t_arr)
y_arr = L1 * np.ones_like(t_arr)
z_arr = np.zeros_like(t_arr)

x_dot = -x_amp * omega * np.sin(omega * t_arr)
y_dot = np.zeros_like(t_arr)
z_dot = np.zeros_like(t_arr)

# ─────────────────────────────────────────────
# PRE-COMPUTE IK
# ─────────────────────────────────────────────
theta1 = np.zeros_like(t_arr)
theta2 = np.zeros_like(t_arr)
theta3 = np.zeros_like(t_arr)

print("Pre-computing inverse kinematics...")
for i in range(num_steps):
    P = np.array([[x_arr[i]], [y_arr[i]], [z_arr[i]]])
    theta1[i], theta2[i], theta3[i] = Inverse_Kinematics(L1, L2, L3, P)

theta1_deg = np.rad2deg(theta1)
theta2_deg = np.rad2deg(theta2)
theta3_deg = np.rad2deg(theta3)

print(f"  θ1 (Hip)   range: {theta1_deg.min():.1f}° → {theta1_deg.max():.1f}°")
print(f"  θ2 (Thigh) range: {theta2_deg.min():.1f}° → {theta2_deg.max():.1f}°")
print(f"  θ3 (Knee)  range: {theta3_deg.min():.1f}° → {theta3_deg.max():.1f}°")
print()

# ─────────────────────────────────────────────
# PRE-COMPUTE SPEEDS VIA JACOBIAN (DLS)
# ─────────────────────────────────────────────
print("Pre-computing joint speeds via Jacobian (DLS)...")
cartesian_velocity = np.vstack((x_dot, y_dot, z_dot))
damp  = 0.001
speed = np.zeros((num_steps, 3))

for i in range(num_steps):
    J    = FL_Jacobian(L1, L2, L3, theta1[i], theta2[i], theta3[i])
    JT   = J.T
    term = JT @ J + (damp ** 2) * np.eye(3)
    theta_dot   = np.linalg.solve(term, JT @ cartesian_velocity[:, i])
    speed[i, :] = np.abs(theta_dot) * 9 * 180 / np.pi

speed = np.clip(speed, 30, 300)
print(f"  Speed range: {speed.min():.1f} – {speed.max():.1f} deg/s")
print()

# ─────────────────────────────────────────────
# SAFETY CHECK
# ─────────────────────────────────────────────
print("First setpoint (t=0):")
print(f"  θ1={theta1_deg[0]:.2f}°  θ2={theta2_deg[0]:.2f}°  θ3={theta3_deg[0]:.2f}°")
print("Last  setpoint (t=10):")
print(f"  θ1={theta1_deg[-1]:.2f}°  θ2={theta2_deg[-1]:.2f}°  θ3={theta3_deg[-1]:.2f}°")
print()
input(">>> Press ENTER to connect to CAN and start — or Ctrl+C to abort <<<")

# ─────────────────────────────────────────────
# CAN SETUP
# ─────────────────────────────────────────────
bus = can.interface.Bus(interface='socketcan', channel='can0', bitrate=1000000)
print("CAN bus connected.")

# ─────────────────────────────────────────────
# MOVE TO START POSITION
# ─────────────────────────────────────────────
print("Moving to zero position...")
Position_Control(bus, ID_HIP,   0, 50)
bus.recv(RECV_TIMEOUT)
Position_Control(bus, ID_THIGH, 0, 50)
bus.recv(RECV_TIMEOUT)
Position_Control(bus, ID_KNEE,  0, 50)
bus.recv(RECV_TIMEOUT)
sleep(2)

print("Moving to start position of trajectory...")
Position_Control(bus, ID_HIP,   theta1_deg[0], 50)
bus.recv(RECV_TIMEOUT)
Position_Control(bus, ID_THIGH, theta2_deg[0], 50)
bus.recv(RECV_TIMEOUT)
Position_Control(bus, ID_KNEE,  theta3_deg[0], 50)
bus.recv(RECV_TIMEOUT)
sleep(2)

print("At start position.")
input(">>> Press ENTER to begin trajectory <<<")

# ─────────────────────────────────────────────
# EXECUTE TRAJECTORY
# Position_Control sends the CAN message; the recv immediately after
# reads the motor's acknowledgement, which throttles the loop to the
# natural round-trip time of the bus (~1–3 ms per message) instead of
# running at full CPU speed and overflowing the TX buffer.
# ─────────────────────────────────────────────
print("Running trajectory... Press Ctrl+C to stop.")

start_time  = time.monotonic()
cycle_start = start_time
last_index  = -1

try:
    while True:
        now           = time.monotonic()
        elapsed_cycle = now - cycle_start

        # Roll over to next cycle
        if elapsed_cycle >= total_time:
            cycle_start += total_time
            elapsed_cycle -= total_time

        index = min(int(elapsed_cycle / dt), num_steps - 1)

        # Only send when the trajectory index has advanced
        if index != last_index:
            s1 = int(speed[index, 0])
            s2 = int(speed[index, 1])
            s3 = int(speed[index, 2])

            Position_Control(bus, ID_HIP,   theta1_deg[index], s1)
            bus.recv(RECV_TIMEOUT)
            Position_Control(bus, ID_THIGH, theta2_deg[index], s2)
            bus.recv(RECV_TIMEOUT)
            Position_Control(bus, ID_KNEE,  theta3_deg[index], s3)
            bus.recv(RECV_TIMEOUT)

            last_index = index

            # Status print once per second
            if index % 200 == 0:
                elapsed_total = now - start_time
                print(f"  t={t_arr[index]:.2f}s | "
                      f"θ1={theta1_deg[index]:+.1f}°  "
                      f"θ2={theta2_deg[index]:+.1f}°  "
                      f"θ3={theta3_deg[index]:+.1f}°  "
                      f"[wall {elapsed_total:.1f}s]")

except KeyboardInterrupt:
    print("\nInterrupted by user.")

finally:
    print("Stopping motors and shutting down CAN...")
    for _ in range(3):
        try:
            Motor_Stop(bus, ID_HIP)
            Motor_Stop(bus, ID_THIGH)
            Motor_Stop(bus, ID_KNEE)
            break
        except Exception:
            sleep(0.05)
    try:
        bus.shutdown()
        print("Done.")
    except Exception as e:
        print(f"Warning during shutdown: {e}")

        '''