import csv
import numpy as np
import matplotlib.pyplot as plt
import os

# ---------------------------------------------------------------
# CONFIGURATION — point this to your log file
# ---------------------------------------------------------------
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
LOG_FILE   = os.path.join(SCRIPT_DIR, "Stand_Pose_Torque_Log_2026-04-13_11-23-55.csv")

# ---------------------------------------------------------------
# READ DATA
# ---------------------------------------------------------------
columns = {
    "Timestamp (s)": [],
    "FL_J1": [], "FL_J2": [], "FL_J3": [],
    "FR_J1": [], "FR_J2": [], "FR_J3": [],
    "HL_J1": [], "HL_J2": [], "HL_J3": [],
    "HR_J1": [], "HR_J2": [], "HR_J3": [],
}

with open(LOG_FILE, 'r') as f:
    reader = csv.DictReader(f)
    for row in reader:
        for key in columns:
            columns[key].append(float(row[key]))

# Convert all to numpy arrays
for key in columns:
    columns[key] = np.array(columns[key])

Timestamp = columns["Timestamp (s)"]

# ---------------------------------------------------------------
# STATS
# ---------------------------------------------------------------
total_time  = Timestamp[-1] - Timestamp[0]
num_samples = len(Timestamp)
dts         = np.diff(Timestamp)
avg_freq    = 1.0 / np.mean(dts)
min_freq    = 1.0 / np.max(dts)
max_freq    = 1.0 / np.min(dts)

print("=" * 40)
print(f"  Log file : {os.path.basename(LOG_FILE)}")
print(f"  Samples  : {num_samples}")
print(f"  Duration : {total_time:.2f} s")
print(f"  Avg freq : {avg_freq:.1f} Hz")
print(f"  Min freq : {min_freq:.1f} Hz  (longest gap:  {np.max(dts)*1000:.1f} ms)")
print(f"  Max freq : {max_freq:.1f} Hz  (shortest gap: {np.min(dts)*1000:.1f} ms)")
print("=" * 40)

# ---------------------------------------------------------------
# PLOT — 2x2 subplots, one per leg
# ---------------------------------------------------------------
LEGS = [
    ("FL", "Front Left",  "upper left"),
    ("FR", "Front Right", "upper right"),
    ("HL", "Hind Left",   "lower left"),
    ("HR", "Hind Right",  "lower right"),
]
COLORS = ["blue", "red", "green"]

fig, axes = plt.subplots(2, 2, figsize=(14, 8), sharex=True, sharey=True)
fig.suptitle("Motor Torques Over Time — All Legs", fontsize=14, fontweight='bold')

for ax, (prefix, title, _) in zip(axes.flat, LEGS):
    for j, color in zip([1, 2, 3], COLORS):
        key = f"{prefix}_J{j}"
        ax.plot(Timestamp, columns[key], label=f"J{j}", color=color, linewidth=0.8)
    ax.set_title(title)
    ax.set_ylabel("Torque [Nm]")
    ax.set_xlabel("Time [s]")
    ax.axhline(0, color='black', linewidth=0.5, linestyle='--')
    ax.grid(True)
    ax.legend()

plt.tight_layout()
plt.show()