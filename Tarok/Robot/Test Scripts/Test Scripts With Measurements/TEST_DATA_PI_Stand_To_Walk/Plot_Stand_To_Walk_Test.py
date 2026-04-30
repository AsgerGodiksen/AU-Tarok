"""
Plot_Bezier_Walk.py

Plots logged Bezier walk data across four 2×2 figures (one subplot per leg):
  Figure 1 — Desired (commanded) joint positions
  Figure 2 — Measured joint positions
  Figure 3 — Position error  (Measured − Desired)
  Figure 4 — Measured joint torques

Usage:
    python Plot_Bezier_Walk.py                        # interactive prompt
    python Plot_Bezier_Walk.py 1                      # load test 1
    python Plot_Bezier_Walk.py path/to/file.csv       # load CSV directly
"""

import sys
import os
import glob
import datetime

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.lines as mlines

# ─────────────────────────────────────────────────────────────────────────────
# Configuration
# ─────────────────────────────────────────────────────────────────────────────
DATA_DIR = os.path.dirname(os.path.abspath(__file__))

LEGS   = ["FL", "FR", "HL", "HR"]
JOINTS = ["J1", "J2", "J3"]
JOINT_COLORS = ["tab:blue", "tab:orange", "tab:green"]
JOINT_LABELS = [r"$\theta_1$ (Hip)", r"$\theta_2$ (Upper)", r"$\theta_3$ (Lower)"]

# Bezier gait cycle time (must match Bezier_TorquePos_Log.py)
SWING_TIME    = 1.0
STAND_TIME    = 3.0
TRANSFER_TIME = 1.5
CYCLE_TIME    = SWING_TIME + STAND_TIME + 4 * TRANSFER_TIME  # 8 s


# ─────────────────────────────────────────────────────────────────────────────
# 1. Load CSV
# ─────────────────────────────────────────────────────────────────────────────
def load_csv(arg: str | None = None) -> tuple[pd.DataFrame, str]:
    if arg and os.path.isfile(arg):
        df = pd.read_csv(arg)
        print(f"  Loaded: {os.path.basename(arg)}  ({len(df)} rows)")
        return df, arg

    if arg and arg.isdigit():
        test_num = int(arg)
    else:
        raw = input("Enter test number (or full path to CSV): ").strip()
        if os.path.isfile(raw):
            df = pd.read_csv(raw)
            print(f"  Loaded: {os.path.basename(raw)}  ({len(df)} rows)")
            return df, raw
        test_num = int(raw)

    pattern = os.path.join(DATA_DIR, f"STW_TorquePos_Log_Test_{test_num}_*.csv")
    matches = sorted(glob.glob(pattern))
    if not matches:
        print(f"  [ERROR] No CSV found for test {test_num} in {DATA_DIR}")
        sys.exit(1)
    if len(matches) == 1:
        csv_path = matches[0]
    else:
        print(f"  Multiple files for test {test_num}:")
        for i, p in enumerate(matches):
            print(f"    {chr(ord('a') + i)}) {os.path.basename(p)}")
        choice   = input("  Choose (a/b/…): ").strip().lower()
        csv_path = matches[ord(choice) - ord('a')]

    df = pd.read_csv(csv_path)
    print(f"  Loaded: {os.path.basename(csv_path)}  ({len(df)} rows)")
    return df, csv_path


print("\nLoading Bezier walk log …")
arg = sys.argv[1] if len(sys.argv) > 1 else None
df, csv_path = load_csv(arg)

t = df["Timestamp (s)"].values
t_duration = t[-1] - t[0]
num_cycles = max(1, int(round(t_duration / CYCLE_TIME)))

cycle_boundaries = [i * CYCLE_TIME for i in range(1, num_cycles + 1)
                    if i * CYCLE_TIME < t_duration]

LEG_POS = {"FL": (0, 0), "FR": (0, 1), "HL": (1, 0), "HR": (1, 1)}


# ─────────────────────────────────────────────────────────────────────────────
# Helper: add cycle markers + formatting to every subplot
# ─────────────────────────────────────────────────────────────────────────────
def format_ax(ax, leg, ylabel):
    for tb in cycle_boundaries:
        ax.axvline(tb, color="grey", linewidth=0.9, linestyle="--", alpha=0.5)
    ax.set_title(f"{leg} Leg", fontsize=11)
    ax.set_ylabel(ylabel)
    ax.set_xlim(t[0], t[-1])
    ax.grid(True, alpha=0.3)


# ─────────────────────────────────────────────────────────────────────────────
# Shared legend handles
# ─────────────────────────────────────────────────────────────────────────────
joint_handles = [
    mlines.Line2D([], [], color=c, linewidth=2, label=lbl)
    for c, lbl in zip(JOINT_COLORS, JOINT_LABELS)
]


# ══════════════════════════════════════════════════════════════════════════════
# Figure 1 — Desired (Commanded) Joint Positions
# ══════════════════════════════════════════════════════════════════════════════
fig1, axes1 = plt.subplots(2, 2, figsize=(15, 9))
fig1.suptitle("Desired (Commanded) Joint Positions — Bezier Walk",
              fontsize=13, fontweight="bold")

for leg, (r, c) in LEG_POS.items():
    ax = axes1[r, c]
    for j, (jname, jcol) in enumerate(zip(JOINTS, JOINT_COLORS)):
        ax.plot(t, df[f"{leg}_{jname}_Cmd (deg)"].values,
                color=jcol, linewidth=1.3)
        ax.set_ylim(-80, 80)
    format_ax(ax, leg, "Position (deg)")

axes1[1, 0].set_xlabel("Time (s)")
axes1[1, 1].set_xlabel("Time (s)")
axes1[0, 0].set_ylabel("Position (deg)")
axes1[1, 0].set_ylabel("Position (deg)")
fig1.legend(handles=joint_handles, loc="lower center",
            ncol=3, fontsize=9, framealpha=0.9, bbox_to_anchor=(0.5, -0.01))
fig1.tight_layout(rect=[0, 0.04, 1, 1])


# ══════════════════════════════════════════════════════════════════════════════
# Figure 2 — Measured Joint Positions
# ══════════════════════════════════════════════════════════════════════════════
fig2, axes2 = plt.subplots(2, 2, figsize=(15, 9))
fig2.suptitle("Measured Joint Positions — Bezier Walk",
              fontsize=13, fontweight="bold")

for leg, (r, c) in LEG_POS.items():
    ax = axes2[r, c]
    for j, (jname, jcol) in enumerate(zip(JOINTS, JOINT_COLORS)):
        ax.plot(t, df[f"{leg}_{jname}_Pos (deg)"].values,
                color=jcol, linewidth=1.3)
    format_ax(ax, leg, "Position (deg)")

axes2[1, 0].set_xlabel("Time (s)")
axes2[1, 1].set_xlabel("Time (s)")
fig2.legend(handles=joint_handles, loc="lower center",
            ncol=3, fontsize=9, framealpha=0.9, bbox_to_anchor=(0.5, -0.01))
fig2.tight_layout(rect=[0, 0.04, 1, 1])


# ══════════════════════════════════════════════════════════════════════════════
# Figure 3 — Position Error (Measured − Desired)
# ══════════════════════════════════════════════════════════════════════════════
error_max = int(np.ceil(max(
    np.abs(df[f"{leg}_{jname}_Pos (deg)"].values - df[f"{leg}_{jname}_Cmd (deg)"].values).max()
    for leg in LEGS for jname in JOINTS
)))

fig3, axes3 = plt.subplots(2, 2, figsize=(15, 9))
fig3.suptitle("Position Error (Measured − Desired) — Bezier Walk",
              fontsize=13, fontweight="bold")

for leg, (r, c) in LEG_POS.items():
    ax = axes3[r, c]
    for j, (jname, jcol) in enumerate(zip(JOINTS, JOINT_COLORS)):
        error = df[f"{leg}_{jname}_Pos (deg)"].values - df[f"{leg}_{jname}_Cmd (deg)"].values
        ax.plot(t[::2], error[::2], color=jcol, linewidth=1.3)
    ax.axhline(0, color="black", linewidth=0.8, linestyle=":", zorder=1)
    #ax.set_ylim(-error_max, error_max)
    ax.set_ylim(-3 , 3)
    format_ax(ax, leg, "Error (deg)")

axes3[1, 0].set_xlabel("Time (s)")
axes3[1, 1].set_xlabel("Time (s)")
fig3.legend(handles=joint_handles, loc="lower center",
            ncol=3, fontsize=9, framealpha=0.9, bbox_to_anchor=(0.5, -0.01))
fig3.tight_layout(rect=[0, 0.04, 1, 1])


# ══════════════════════════════════════════════════════════════════════════════
# Figure 4 — Measured Joint Torques
# ══════════════════════════════════════════════════════════════════════════════
torque_cols = [f"{leg}_{jname}_Torque" for leg in LEGS for jname in JOINTS]
torque_ylim = int(np.ceil(df[torque_cols].abs().values.max()))

fig4, axes4 = plt.subplots(2, 2, figsize=(15, 9))
fig4.suptitle("Measured Joint Torques — Bezier Walk",
              fontsize=13, fontweight="bold")

t_ds  = t[::2]
for leg, (r, c) in LEG_POS.items():
    ax = axes4[r, c]
    for j, (jname, jcol) in enumerate(zip(JOINTS, JOINT_COLORS)):
        ax.plot(t_ds, df[f"{leg}_{jname}_Torque"].values[::2],
                color=jcol, linewidth=1.0, alpha=0.85)
    ax.axhline(0, color="black", linewidth=0.8, linestyle=":", zorder=1)
    ax.set_ylim(-torque_ylim, torque_ylim)
    format_ax(ax, leg, "Torque (Nm)")

axes4[1, 0].set_xlabel("Time (s)")
axes4[1, 1].set_xlabel("Time (s)")
fig4.legend(handles=joint_handles, loc="lower center",
            ncol=3, fontsize=9, framealpha=0.9, bbox_to_anchor=(0.5, -0.01))
fig4.tight_layout(rect=[0, 0.04, 1, 1])

# ══════════════════════════════════════════════════════════════════════════════
# Figure 5 — Zoomed Position Error (Measured − Desired)
# ══════════════════════════════════════════════════════════════════════════════
error_max = int(np.ceil(max(
    np.abs(df[f"{leg}_{jname}_Pos (deg)"].values - df[f"{leg}_{jname}_Cmd (deg)"].values).max()
    for leg in LEGS for jname in JOINTS
)))

fig5, axes5 = plt.subplots(2, 2, figsize=(15, 9))
fig5.suptitle("Zoomed Position Error (Measured − Desired) — Bezier Walk",
              fontsize=13, fontweight="bold")

for leg, (r, c) in LEG_POS.items():
    ax = axes5[r, c]
    for j, (jname, jcol) in enumerate(zip(JOINTS, JOINT_COLORS)):
        error = df[f"{leg}_{jname}_Pos (deg)"].values - df[f"{leg}_{jname}_Cmd (deg)"].values
        ax.plot(t[::2], error[::2], color=jcol, linewidth=1.3)
    ax.axhline(0, color="black", linewidth=0.8, linestyle=":", zorder=1)
    #ax.set_ylim(-error_max, error_max)
    ax.set_ylim(-0.5 , 0.5)
    format_ax(ax, leg, "Error (deg)")

axes5[1, 0].set_xlabel("Time (s)")
axes5[1, 1].set_xlabel("Time (s)")
fig5.legend(handles=joint_handles, loc="lower center",
            ncol=3, fontsize=9, framealpha=0.9, bbox_to_anchor=(0.5, -0.01))
fig5.tight_layout(rect=[0, 0.04, 1, 1])


# ─────────────────────────────────────────────────────────────────────────────
# Statistics report
# ─────────────────────────────────────────────────────────────────────────────
lines: list[str] = []

def emit(text: str = "") -> None:
    print(text)
    lines.append(text)

emit("=" * 62)
emit(f"  Bezier Walk — Position Tracking Report")
emit(f"  Generated : {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
emit(f"  CSV       : {os.path.basename(csv_path)}")
emit(f"  Duration  : {t_duration:.1f} s  ({num_cycles} cycles)")
emit("=" * 62)
emit(f"\n{'Leg':<6}{'Joint':<8}{'RMS error (deg)':>16}{'Max |error| (deg)':>20}{'RMS torque (Nm)':>18}")
emit("─" * 68)

total_rms = 0.0
for leg in LEGS:
    for j, jname in enumerate(JOINTS):
        cmd    = df[f"{leg}_{jname}_Cmd (deg)"].values
        meas   = df[f"{leg}_{jname}_Pos (deg)"].values
        torque = df[f"{leg}_{jname}_Torque"].values
        error  = meas - cmd
        rms_e  = np.sqrt(np.mean(error ** 2))
        max_e  = np.max(np.abs(error))
        total_rms += rms_e
        emit(f"{leg:<6}{jname:<8}{rms_e:>16.3f}{max_e:>20.3f}")

emit("─" * 68)
emit(f"Sum of 12 RMS position errors: {total_rms:.3f} deg")
emit("─" * 68)

# Append FL leg PI parameters from the matching PID.txt
pid_path = os.path.splitext(csv_path)[0] + "_PID.txt"
if os.path.isfile(pid_path):
    with open(pid_path) as pf:
        pid_lines = pf.readlines()
    # Extract header line and the FL leg block
    fl_block = []
    in_fl = False
    for line in pid_lines:
        if line.startswith("PID Parameters"):
            fl_block.append(line.rstrip())
        elif line.startswith("Leg FL"):
            in_fl = True
            fl_block.append(line.rstrip())
        elif in_fl and line.startswith("Leg "):
            break
        elif in_fl:
            fl_block.append(line.rstrip())
    emit("")
    emit("PI Parameters (FL leg)")
    emit("─" * 68)
    for ln in fl_block:
        emit(ln)
else:
    emit(f"\n[WARNING] PID file not found: {os.path.basename(pid_path)}")


# ─────────────────────────────────────────────────────────────────────────────
# Save outputs
# ─────────────────────────────────────────────────────────────────────────────
timestamp = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
base_name = os.path.splitext(os.path.basename(csv_path))[0]

# Extract test number from filename (e.g. Bezier_TorquePos_Log_Test_3_...)
import re
_m = re.search(r"_Test_(\d+)_", base_name)
test_num_str = _m.group(1).zfill(2) if _m else "00"

output_dir = os.path.join(DATA_DIR, f"Plots_For_Tests_{test_num_str}")
os.makedirs(output_dir, exist_ok=True)

fig_names = {
    fig1: "Desired_Positions",
    fig2: "Measured_Positions",
    fig3: "Position_Error",
    fig4: "Torques",
    fig5: "Zoomed_Position_Error"
}
for fig, label in fig_names.items():
    fig_path = os.path.join(output_dir, f"{base_name}_{label}_{timestamp}.png")
    fig.savefig(fig_path, dpi=150, bbox_inches="tight")
    print(f"Saved: {os.path.relpath(fig_path, DATA_DIR)}")

report_path = os.path.join(output_dir, f"{base_name}_Report_{timestamp}.txt")
with open(report_path, "w") as f:
    f.write("\n".join(lines) + "\n")
print(f"Report saved: {os.path.relpath(report_path, DATA_DIR)}")

