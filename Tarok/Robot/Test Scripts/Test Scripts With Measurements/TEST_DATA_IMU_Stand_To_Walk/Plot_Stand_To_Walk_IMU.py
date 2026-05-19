"""
Plot_Stand_To_Walk_IMU.py

Plots logged IMU orientation data (Pitch & Roll) from a Stand-to-Walk test.

Usage:
    python Plot_Stand_To_Walk_IMU.py                   # interactive prompt
    python Plot_Stand_To_Walk_IMU.py 1                 # load test 1
    python Plot_Stand_To_Walk_IMU.py path/to/file.csv  # load CSV directly
"""

import sys
import os
import glob
import datetime
import re

import matplotlib
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.lines as mlines
import matplotlib.patches as mpatches
    
matplotlib.rcParams["font.family"] = "Liberation Serif"
# ─────────────────────────────────────────────────────────────────────────────
# Configuration
# ─────────────────────────────────────────────────────────────────────────────
DATA_DIR = os.path.dirname(os.path.abspath(__file__))

# Bezier gait cycle time (must match Stand_To_Walk_IMU_Log.py)
SWING_TIME    = 1.0
STAND_TIME    = 3.0
TRANSFER_TIME = 1.5
CYCLE_TIME    = SWING_TIME + STAND_TIME + 4 * TRANSFER_TIME  # 8 s

LABEL_SIZE  = 17
TICK_SIZE   = 13
LEGEND_SIZE = 13

# Swing-phase intervals within one 10 s gait cycle (start, end) in seconds
SWING_INTERVALS = {
    "HL": (0.75, 1.75),
    "FL": (3.25, 4.25),
    "HR": (5.75, 6.75),
    "FR": (8.25, 9.25),
}


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

    pattern = os.path.join(DATA_DIR, f"STW_IMU_Log_{test_num}_*.csv")
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


print("\nLoading IMU walk log …")
arg = sys.argv[1] if len(sys.argv) > 1 else None
df, csv_path = load_csv(arg)

t     = df["Timestamp (s)"].values
pitch = df["Pitch (deg)"].values
roll  = df["Roll (deg)"].values

t_duration = t[-1] - t[0]
num_cycles = max(1, int(round(t_duration / CYCLE_TIME)))

cycle_boundaries = [i * CYCLE_TIME for i in range(1, num_cycles + 1)
                    if i * CYCLE_TIME < t_duration]


# ─────────────────────────────────────────────────────────────────────────────
# Helper: shared axis formatting
# ─────────────────────────────────────────────────────────────────────────────
def add_swing_spans(ax, t_start, t_end):
    """Draw light-gray boxes for every swing phase of all legs across all cycles."""
    for s0, s1 in SWING_INTERVALS.values():
        cycle = 0
        while True:
            x0 = t_start + cycle * CYCLE_TIME + s0
            x1 = t_start + cycle * CYCLE_TIME + s1
            if x0 >= t_end:
                break
            ax.axvspan(x0, min(x1, t_end), color="lightgray", alpha=0.6, zorder=0)
            cycle += 1


def format_ax(ax, ylabel):
    for tb in cycle_boundaries:
        ax.axvline(tb, color="grey", linewidth=0.9, linestyle="--", alpha=0.5)
    ax.axhline(0, color="grey", linewidth=0.8, linestyle=":")
    ax.set_ylabel(ylabel, fontsize=LABEL_SIZE)
    ax.set_xlabel("Time (s)", fontsize=LABEL_SIZE)
    ax.set_xlim(t[0], t[-1])
    ax.grid(True, alpha=0.3)
    ax.tick_params(labelsize=TICK_SIZE)


# ══════════════════════════════════════════════════════════════════════════════
# Figure — Pitch and Roll
# ══════════════════════════════════════════════════════════════════════════════
fig, (ax_pitch, ax_roll) = plt.subplots(1, 2, figsize=(14, 5))

add_swing_spans(ax_pitch, t[0], t[-1])
add_swing_spans(ax_roll,  t[0], t[-1])

ax_pitch.plot(t, pitch, color="black",  linewidth=1.3)
ax_roll.plot( t, roll,  color="purple", linewidth=1.3)

format_ax(ax_pitch, "Pitch (°)")
format_ax(ax_roll,  "Roll (°)")

pitch_handle = mlines.Line2D([], [], color="black",  linewidth=1.3, label="Pitch")
roll_handle  = mlines.Line2D([], [], color="purple", linewidth=1.3, label="Roll")
cycle_handle = mlines.Line2D([], [], color="grey", linewidth=0.9,
                              linestyle="--", alpha=0.5, label="New Cycle")
swing_handle = mpatches.Patch(facecolor="lightgray", alpha=0.6, label="Swing Phase")

fig.legend(handles=[pitch_handle, roll_handle, cycle_handle, swing_handle],
           loc="lower center", ncol=4, fontsize=LEGEND_SIZE,
           framealpha=0.9, bbox_to_anchor=(0.5, -0.03))
fig.tight_layout()
fig.subplots_adjust(bottom=0.18)


# ─────────────────────────────────────────────────────────────────────────────
# Statistics report
# ─────────────────────────────────────────────────────────────────────────────
lines: list[str] = []

def emit(text: str = "") -> None:
    print(text)
    lines.append(text)

emit("=" * 54)
emit(f"  IMU Orientation — Walk Report")
emit(f"  Generated : {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
emit(f"  CSV       : {os.path.basename(csv_path)}")
emit(f"  Duration  : {t_duration:.1f} s  ({num_cycles} cycles)")
emit("=" * 54)
emit(f"\n{'Channel':<10}{'Mean (°)':>12}{'Std (°)':>12}{'Min (°)':>12}{'Max (°)':>12}")
emit("─" * 58)
for label, arr in [("Pitch", pitch), ("Roll", roll)]:
    valid = arr[~np.isnan(arr)]
    emit(f"{label:<10}{np.mean(valid):>12.3f}{np.std(valid):>12.3f}"
         f"{np.min(valid):>12.3f}{np.max(valid):>12.3f}")
emit("─" * 58)


# ─────────────────────────────────────────────────────────────────────────────
# Save outputs
# ─────────────────────────────────────────────────────────────────────────────
timestamp = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
base_name = os.path.splitext(os.path.basename(csv_path))[0]

_m = re.search(r"IMU_Log_(\d+)_", base_name)
test_num_str = _m.group(1).zfill(2) if _m else "00"

output_dir = os.path.join(os.path.dirname(csv_path), f"Plots_For_Tests_{test_num_str}")
os.makedirs(output_dir, exist_ok=True)

fig_path = os.path.join(output_dir, f"{base_name}_Pitch_Roll_{timestamp}.png")
fig.savefig(fig_path, dpi=150, bbox_inches="tight")
print(f"Saved: {os.path.relpath(fig_path, DATA_DIR)}")

report_path = os.path.join(output_dir, f"{base_name}_Report_{timestamp}.txt")
with open(report_path, "w") as f:
    f.write("\n".join(lines) + "\n")
print(f"Report saved: {os.path.relpath(report_path, DATA_DIR)}")

plt.show()