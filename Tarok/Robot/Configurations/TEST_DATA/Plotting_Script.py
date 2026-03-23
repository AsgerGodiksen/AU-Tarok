# Plot_IMU_Data.py
# Reads a CSV log file and plots Pitch and Roll over time as separate subplots

import csv
import numpy as np
import matplotlib.pyplot as plt
import os

# ---------------------------------------------------------------
# CONFIGURATION — point this to your log file
# ---------------------------------------------------------------
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
LOG_FILE   = os.path.join(SCRIPT_DIR, "Stand_Pose_TEST_IMU_Log_2026-03-23_13-16-35.csv")

# ---------------------------------------------------------------
# READ DATA
# ---------------------------------------------------------------
timestamps = []
pitch      = []
roll       = []

with open(LOG_FILE, 'r') as f:
    reader = csv.DictReader(f)
    for row in reader:
        timestamps.append(float(row["Timestamp (s)"]))
        pitch.append(float(row["Pitch_deg"]))
        roll.append(float(row["Roll_deg"]))

timestamps = np.array(timestamps)
pitch      = np.array(pitch)
roll       = np.array(roll)

# ---------------------------------------------------------------
# PLOT
# ---------------------------------------------------------------
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 6), sharex=True)
fig.suptitle("IMU Data — Pitch and Roll", fontsize=14)

ax1.plot(timestamps, pitch, color='blue', linewidth=1)
ax1.set_ylabel("Pitch [deg]")
ax1.grid(True)
ax1.axhline(0, color='black', linewidth=0.5, linestyle='--')

ax2.plot(timestamps, roll, color='red', linewidth=1)
ax2.set_ylabel("Roll [deg]")
ax2.set_xlabel("Time [s]")
ax2.grid(True)
ax2.axhline(0, color='black', linewidth=0.5, linestyle='--')

plt.tight_layout()
plt.show()