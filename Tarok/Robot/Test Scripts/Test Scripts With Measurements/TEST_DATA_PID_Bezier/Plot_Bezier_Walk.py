"""
Plot_Bezier_Walk.py

Plots logged Bezier walk data with four vertically-stacked subplots per leg:
  1. Desired (commanded) joint positions
  2. Measured joint positions
  3. Position error  (Measured − Desired)
  4. Measured joint torques

One figure is created per leg (FL, FR, HL, HR).
Cycle boundaries are marked with vertical dashed lines.

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
JOINT_COLORS  = ["tab:blue", "tab:orange", "tab:green"]
JOINT_LABELS  = [r"$\theta_1$ (Hip)",  r"$\theta_2$ (Upper)",  r"$\theta_3$ (Lower)"]

# Bezier gait cycle time (must match your Bezier_TorquePos_Log.py settings)
SWING_TIME    = 1.0          # s
STAND_TIME    = 3.0          # s
TRANSFER_TIME = 1.0          # s
CYCLE_TIME    = SWING_TIME + STAND_TIME + 4 * TRANSFER_TIME   # 8 s


# ─────────────────────────────────────────────────────────────────────────────
# 1. Load CSV
# ─────────────────────────────────────────────────────────────────────────────
def load_csv(arg: str | None = None) -> tuple[pd.DataFrame, str]:
    """Load a Bezier walk log CSV — by test number, path, or interactive prompt."""

    # Direct path
    if arg and os.path.isfile(arg):
        df = pd.read_csv(arg)
        print(f"  Loaded: {os.path.basename(arg)}  ({len(df)} rows)")
        return df, arg

    # Test number
    if arg and arg.isdigit():
        test_num = int(arg)
    else:
        raw = input("Enter test number (or full path to CSV): ").strip()
        if os.path.isfile(raw):
            df = pd.read_csv(raw)
            print(f"  Loaded: {os.path.basename(raw)}  ({len(df)} rows)")
            return df, raw
        test_num = int(raw)

    pattern = os.path.join(DATA_DIR, f"Bezier_TorquePos_Log_Test_{test_num}_*.csv")
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


# ─────────────────────────────────────────────────────────────────────────────
# 2. Extract arrays
# ─────────────────────────────────────────────────────────────────────────────
t = df["Timestamp (s)"].values
t_duration = t[-1] - t[0]
num_cycles = max(1, int(round(t_duration / CYCLE_TIME)))

# Build cycle-boundary times for vertical markers
cycle_boundaries = [i * CYCLE_TIME for i in range(1, num_cycles + 1)
                    if i * CYCLE_TIME < t_duration]


# ─────────────────────────────────────────────────────────────────────────────
# 3. Plot — one figure per leg, 4 subplots each
# ─────────────────────────────────────────────────────────────────────────────
figures = {}

for leg in LEGS:
    fig, axes = plt.subplots(4, 1, figsize=(14, 11), sharex=True)
    fig.suptitle(f"{leg} Leg — Bezier Walk  ({num_cycles} cycles)",
                 fontsize=14, fontweight="bold", y=0.97)

    ax_cmd, ax_meas, ax_err, ax_torq = axes

    # ── Subplot 1: Desired (commanded) positions ─────────────────────────
    for j, (jname, jcol, jlabel) in enumerate(zip(JOINTS, JOINT_COLORS, JOINT_LABELS)):
        cmd = df[f"{leg}_{jname}_Cmd (deg)"].values
        ax_cmd.plot(t, cmd, color=jcol, linewidth=1.2, label=jlabel)

    ax_cmd.set_ylabel("Position (deg)")
    ax_cmd.set_title("Desired (Commanded) Joint Positions", fontsize=10, loc="left")
    ax_cmd.legend(loc="upper right", fontsize=8, ncol=3, framealpha=0.85)
    ax_cmd.grid(True, alpha=0.3)

    # ── Subplot 2: Measured positions ────────────────────────────────────
    for j, (jname, jcol, jlabel) in enumerate(zip(JOINTS, JOINT_COLORS, JOINT_LABELS)):
        meas = df[f"{leg}_{jname}_Pos (deg)"].values
        ax_meas.plot(t, meas, color=jcol, linewidth=1.2, label=jlabel)

    ax_meas.set_ylabel("Position (deg)")
    ax_meas.set_title("Measured Joint Positions", fontsize=10, loc="left")
    ax_meas.legend(loc="upper right", fontsize=8, ncol=3, framealpha=0.85)
    ax_meas.grid(True, alpha=0.3)

    # ── Subplot 3: Position error ────────────────────────────────────────
    for j, (jname, jcol, jlabel) in enumerate(zip(JOINTS, JOINT_COLORS, JOINT_LABELS)):
        cmd  = df[f"{leg}_{jname}_Cmd (deg)"].values
        meas = df[f"{leg}_{jname}_Pos (deg)"].values
        error = meas - cmd
        ax_err.plot(t, error, color=jcol, linewidth=1.2, label=jlabel)

    ax_err.axhline(0, color="black", linewidth=0.8, linestyle=":", zorder=1)
    ax_err.set_ylabel("Error (deg)")
    ax_err.set_title("Position Error (Measured − Desired)", fontsize=10, loc="left")
    ax_err.legend(loc="upper right", fontsize=8, ncol=3, framealpha=0.85)
    ax_err.grid(True, alpha=0.3)

    # ── Subplot 4: Measured torques ──────────────────────────────────────
    for j, (jname, jcol, jlabel) in enumerate(zip(JOINTS, JOINT_COLORS, JOINT_LABELS)):
        torque = df[f"{leg}_{jname}_Torque"].values
        ax_torq.plot(t, torque, color=jcol, linewidth=1.0, alpha=0.85, label=jlabel)

    ax_torq.axhline(0, color="black", linewidth=0.8, linestyle=":", zorder=1)
    ax_torq.set_ylabel("Torque (Nm)")
    ax_torq.set_xlabel("Time (s)")
    ax_torq.set_title("Measured Joint Torques", fontsize=10, loc="left")
    ax_torq.legend(loc="upper right", fontsize=8, ncol=3, framealpha=0.85)
    ax_torq.grid(True, alpha=0.3)

    # ── Cycle boundary markers on all subplots ───────────────────────────
    for ax in axes:
        for tb in cycle_boundaries:
            ax.axvline(tb, color="grey", linewidth=0.9, linestyle="--", alpha=0.5)
        ax.set_xlim(t[0], t[-1])

    fig.tight_layout(rect=[0, 0, 1, 0.95])
    figures[leg] = fig


# ─────────────────────────────────────────────────────────────────────────────
# 4. Statistics report
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
        cmd   = df[f"{leg}_{jname}_Cmd (deg)"].values
        meas  = df[f"{leg}_{jname}_Pos (deg)"].values
        torque = df[f"{leg}_{jname}_Torque"].values
        error = meas - cmd
        rms_e = np.sqrt(np.mean(error ** 2))
        max_e = np.max(np.abs(error))
        rms_t = np.sqrt(np.mean(torque ** 2))
        total_rms += rms_e
        emit(f"{leg:<6}{jname:<8}{rms_e:>16.3f}{max_e:>20.3f}{rms_t:>18.3f}")

emit("─" * 68)
emit(f"Sum of 12 RMS position errors: {total_rms:.3f} deg")
emit("─" * 68)


# ─────────────────────────────────────────────────────────────────────────────
# 5. Save outputs
# ─────────────────────────────────────────────────────────────────────────────
timestamp = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
base_name = os.path.splitext(os.path.basename(csv_path))[0]

# Save figures
for leg, fig in figures.items():
    fig_path = os.path.join(DATA_DIR, f"{base_name}_{leg}_{timestamp}.png")
    fig.savefig(fig_path, dpi=150, bbox_inches="tight")
    print(f"Saved: {os.path.basename(fig_path)}")

# Save report
report_path = os.path.join(DATA_DIR, f"{base_name}_Report_{timestamp}.txt")
with open(report_path, "w") as f:
    f.write("\n".join(lines) + "\n")
print(f"Report saved: {os.path.basename(report_path)}")

# Append stats to companion PID file if it exists
pid_path = csv_path.replace(".csv", "_PID.txt")
if os.path.isfile(pid_path):
    with open(pid_path, "a") as f:
        f.write(f"\n\nBEZIER WALK STATISTICS — appended {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write("=" * 52 + "\n")
        f.write(f"{'Leg':<6}{'Joint':<8}{'RMS error (deg)':>16}{'Max |error| (deg)':>20}\n")
        f.write("─" * 52 + "\n")
        for leg in LEGS:
            for j, jname in enumerate(JOINTS):
                cmd  = df[f"{leg}_{jname}_Cmd (deg)"].values
                meas = df[f"{leg}_{jname}_Pos (deg)"].values
                error = meas - cmd
                rms_e = np.sqrt(np.mean(error ** 2))
                max_e = np.max(np.abs(error))
                f.write(f"{leg:<6}{jname:<8}{rms_e:>16.3f}{max_e:>20.3f}\n")
        f.write("─" * 52 + "\n")
        f.write(f"Sum of 12 RMS errors: {total_rms:.3f} deg\n")
    print(f"Statistics appended to {os.path.basename(pid_path)}")

plt.show()
