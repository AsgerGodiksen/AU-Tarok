# Plot_Torque_Comparison.py
# Shows 4 test results side by side, each as a 2x2 leg subplot.
# Layout: 2 rows x 8 columns (4 tests × 2 leg-columns each)

import csv
import numpy as np
import matplotlib.pyplot as plt
import os

# ---------------------------------------------------------------
# CONFIGURATION — add/change your log files and labels here
# ---------------------------------------------------------------
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

TESTS = [
    {"file": "Stand_Pose_Torque_Log_NEW_PID_Test_1.csv", "label": "PID Test 1"},
    {"file": "Stand_Pose_Torque_Log_NEW_PID_Test_2.csv", "label": "PID Test 2"},
    {"file": "Stand_Pose_Torque_Log_NEW_PID_Test_3.csv", "label": "PID Test 3"},
    {"file": "Stand_Pose_Torque_Log_NEW_PID_Test_4.csv", "label": "PID Test 4"},
]

# ---------------------------------------------------------------
# READ DATA
# ---------------------------------------------------------------
def read_log(filepath):
    columns = {
        "Timestamp (s)": [],
        "FL_J1": [], "FL_J2": [], "FL_J3": [],
        "FR_J1": [], "FR_J2": [], "FR_J3": [],
        "HL_J1": [], "HL_J2": [], "HL_J3": [],
        "HR_J1": [], "HR_J2": [], "HR_J3": [],
    }
    with open(filepath, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            for key in columns:
                columns[key].append(float(row[key]))
    for key in columns:
        columns[key] = np.array(columns[key])
    return columns

all_data = []
for t in TESTS:
    path = os.path.join(SCRIPT_DIR, t["file"])
    data = read_log(path)
    ts = data["Timestamp (s)"]
    dts = np.diff(ts)
    print(f"  {t['label']:20s} | {len(ts):6d} samples | {ts[-1]-ts[0]:.1f}s | {1/np.mean(dts):.0f} Hz avg")
    all_data.append(data)

# ---------------------------------------------------------------
# PLOT — one figure per test, each with 2x2 leg subplots
# ---------------------------------------------------------------
LEGS   = [("FL", "Front Left"),  ("FR", "Front Right"),
          ("HL", "Hind Left"),   ("HR", "Hind Right")]
COLORS = ["blue", "red", "green"]

for test, data in zip(TESTS, all_data):
    ts = data["Timestamp (s)"]

    fig, axes = plt.subplots(2, 2, figsize=(14, 8), sharex=True, sharey=True)
    fig.suptitle(f"Motor Torques — {test['label']}", fontsize=14, fontweight='bold')

    for idx, (ax, (prefix, leg_title)) in enumerate(zip(axes.flat, LEGS)):
        row, col = divmod(idx, 2)
        for j, color in zip([1, 2, 3], COLORS):
            key = f"{prefix}_J{j}"
            ax.plot(ts, data[key], label=f"J{j}", color=color, linewidth=0.8)
        ax.set_title(leg_title)
        if col == 0:
            ax.set_ylabel("Torque [Nm]")
        if row == 1:
            ax.set_xlabel("Time [s]")
        ax.axhline(0, color='black', linewidth=0.5, linestyle='--')
        ax.grid(True)
        ax.legend()

    plt.tight_layout()

plt.show()