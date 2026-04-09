# Plot_IMU_Data.py
import csv
import numpy as np
import matplotlib.pyplot as plt
import os

# ---------------------------------------------------------------
# CONFIGURATION — point this to your log file
# ---------------------------------------------------------------
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
LOG_FILE   = os.path.join(SCRIPT_DIR, "Up_Down_Torque_Log_2026-04-09_13-13-01.csv")

# ---------------------------------------------------------------
# READ DATA
# ---------------------------------------------------------------
Timestamp = []
FL_J1_Torque   = []
FL_J2_Torque   = []
FL_J3_Torque   = []

with open(LOG_FILE, 'r') as f:
    reader = csv.DictReader(f)
    for row in reader:
        Timestamp.append(float(row["Timestamp (s)"]))
        FL_J1_Torque.append(float(row["FL_J1_Torque"]))
        FL_J2_Torque.append(float(row["FL_J2_Torque"]))
        FL_J3_Torque.append(float(row["FL_J3_Torque"]))

Timestamp = np.array(Timestamp)
FL_J1_Torque   = np.array(FL_J1_Torque)
FL_J2_Torque   = np.array(FL_J2_Torque)
FL_J3_Torque   = np.array(FL_J3_Torque)

# ---------------------------------------------------------------
# STATS
# ---------------------------------------------------------------
total_time   = Timestamp[-1] - Timestamp[0]
num_samples  = len(Timestamp)
dts          = np.diff(Timestamp)
avg_freq     = 1.0 / np.mean(dts)
min_freq     = 1.0 / np.max(dts)
max_freq     = 1.0 / np.min(dts)

print("=" * 40)
print(f"  Log file : {os.path.basename(LOG_FILE)}")
print(f"  Samples  : {num_samples}")
print(f"  Duration : {total_time:.2f} s")
print(f"  Avg freq : {avg_freq:.1f} Hz")
print(f"  Min freq : {min_freq:.1f} Hz  (longest gap: {np.max(dts)*1000:.1f} ms)")
print(f"  Max freq : {max_freq:.1f} Hz  (shortest gap: {np.min(dts)*1000:.1f} ms)")
print("=" * 40)

# ---------------------------------------------------------------
# PLOT
# ---------------------------------------------------------------
plt.figure(figsize=(12, 5))
plt.title("Motor Torques Over Time")
plt.plot(Timestamp, FL_J1_Torque, label="FL_J1_Torque", color='blue')
plt.plot(Timestamp, FL_J2_Torque, label="FL_J2_Torque", color='red')
plt.plot(Timestamp, FL_J3_Torque, label="FL_J3_Torque", color='green')
plt.xlabel("Time [s]")
plt.ylabel("Torque [Nm]")
plt.grid(True)
plt.axhline(0, color='black', linewidth=0.5, linestyle='--')
plt.legend()
plt.tight_layout()
plt.show()