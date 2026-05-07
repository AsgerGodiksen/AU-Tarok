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

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.lines as mlines

# ─────────────────────────────────────────────────────────────────────────────
# Configuration
# ─────────────────────────────────────────────────────────────────────────────
DATA_DIR = os.path.dirname(os.path.abspath(__file__))

# Bezier gait cycle time (must match Stand_To_Walk_IMU_Log.py)
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

    pattern = os.path.join(DATA_DIR, f"TEST_DATA_IMU_Stand_To_Walk", f"STW_IMU_Log_{test_num}_*.csv")
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
def format_ax(ax, title, ylabel):
    for tb in cycle_boundaries:
        ax.axvline(tb, color="grey", linewidth=0.9, linestyle="--", alpha=0.5)
    ax.axhline(0, color="grey", linewidth=0.8, linestyle=":")
    ax.set_title(title, fontsize=11)
    ax.set_ylabel(ylabel)
    ax.set_xlabel("Time (s)")
    ax.set_xlim(t[0], t[-1])
    ax.grid(True, alpha=0.3)


# ══════════════════════════════════════════════════════════════════════════════
# Figure — Pitch and Roll
# ══════════════════════════════════════════════════════════════════════════════
fig, (ax_pitch, ax_roll) = plt.subplots(1, 2, figsize=(14, 5))
fig.suptitle("IMU Orientation — Bezier Walk  (Pitch & Roll)",
             fontsize=13, fontweight="bold")
plt.subplots_adjust(left=0.07, right=0.97, top=0.88, bottom=0.12, wspace=0.30)

ax_pitch.plot(t, pitch, color="black",  linewidth=1.3, label="Pitch")
ax_roll.plot( t, roll,  color="purple", linewidth=1.3, label="Roll")

format_ax(ax_pitch, "Pitch", "Pitch (deg)")
format_ax(ax_roll,  "Roll",  "Roll (deg)")

ax_pitch.legend(fontsize=9)
ax_roll.legend(fontsize=9)

fig.tight_layout()


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
emit(f"\n{'Channel':<10}{'Mean (deg)':>12}{'Std (deg)':>12}{'Min (deg)':>12}{'Max (deg)':>12}")
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