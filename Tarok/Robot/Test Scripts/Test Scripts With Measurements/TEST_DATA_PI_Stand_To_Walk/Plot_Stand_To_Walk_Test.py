"""
Plot_Stand_To_Walk_Test.py

Processes all STW_TorquePos_Log_Test_*.csv files in the script directory.
For each file, generates 5 figures and a statistics report, saved to a
per-test subfolder: Plots_For_Tests_<NN>/

Figures per test:
  1 — Desired (commanded) joint positions
  2 — Measured joint positions
  3 — Position error  (Measured − Desired)
  4 — Measured joint torques
  5 — Zoomed position error
"""

import sys
import os
import glob
import re
import datetime

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.lines as mlines

# ─────────────────────────────────────────────────────────────────────────────
# Configuration
# ─────────────────────────────────────────────────────────────────────────────
DATA_DIR = os.path.dirname(os.path.abspath(__file__))

LEGS         = ["FL", "FR", "HL", "HR"]
JOINTS       = ["J1", "J2", "J3"]
JOINT_COLORS = ["tab:blue", "tab:orange", "tab:green"]
JOINT_LABELS = [r"$\theta_1$ (Hip)", r"$\theta_2$ (Upper)", r"$\theta_3$ (Lower)"]
LEG_POS      = {"FL": (0, 0), "FR": (0, 1), "HL": (1, 0), "HR": (1, 1)}

SWING_TIME    = 1.0
STAND_TIME    = 3.0
TRANSFER_TIME = 1.5
CYCLE_TIME    = SWING_TIME + STAND_TIME + 4 * TRANSFER_TIME  # 8 s

joint_handles = [
    mlines.Line2D([], [], color=c, linewidth=2, label=lbl)
    for c, lbl in zip(JOINT_COLORS, JOINT_LABELS)
]


# ─────────────────────────────────────────────────────────────────────────────
# Per-file processing
# ─────────────────────────────────────────────────────────────────────────────
def format_ax(ax, leg, ylabel, t, cycle_boundaries):
    for tb in cycle_boundaries:
        ax.axvline(tb, color="grey", linewidth=0.9, linestyle="--", alpha=0.5)
    ax.set_title(f"{leg} Leg", fontsize=11)
    ax.set_ylabel(ylabel)
    ax.set_xlim(t[0], t[-1])
    ax.grid(True, alpha=0.3)


def process_file(csv_path: str) -> None:
    base_name = os.path.splitext(os.path.basename(csv_path))[0]
    print(f"\nProcessing: {os.path.basename(csv_path)}")

    df = pd.read_csv(csv_path)
    print(f"  Loaded {len(df)} rows")

    t          = df["Timestamp (s)"].values
    t_duration = t[-1] - t[0]
    num_cycles = max(1, int(round(t_duration / CYCLE_TIME)))
    cycle_boundaries = [i * CYCLE_TIME for i in range(1, num_cycles + 1)
                        if i * CYCLE_TIME < t_duration]

    def fmt(ax, leg, ylabel):
        format_ax(ax, leg, ylabel, t, cycle_boundaries)

    # ── Figure 1 — Desired Positions ─────────────────────────────────────────
    fig1, axes1 = plt.subplots(2, 2, figsize=(15, 9))
    fig1.suptitle("Desired (Commanded) Joint Positions — Bezier Walk",
                  fontsize=13, fontweight="bold")
    for leg, (r, c) in LEG_POS.items():
        ax = axes1[r, c]
        for jname, jcol in zip(JOINTS, JOINT_COLORS):
            ax.plot(t, df[f"{leg}_{jname}_Cmd (deg)"].values, color=jcol, linewidth=1.3)
        ax.set_ylim(-80, 80)
        fmt(ax, leg, "Position (deg)")
    axes1[1, 0].set_xlabel("Time (s)")
    axes1[1, 1].set_xlabel("Time (s)")
    axes1[0, 0].set_ylabel("Position (deg)")
    axes1[1, 0].set_ylabel("Position (deg)")
    fig1.legend(handles=joint_handles, loc="lower center",
                ncol=3, fontsize=9, framealpha=0.9, bbox_to_anchor=(0.5, -0.01))
    fig1.tight_layout(rect=[0, 0.04, 1, 1])

    # ── Figure 2 — Measured Positions ────────────────────────────────────────
    fig2, axes2 = plt.subplots(2, 2, figsize=(15, 9))
    fig2.suptitle("Measured Joint Positions — Bezier Walk",
                  fontsize=13, fontweight="bold")
    for leg, (r, c) in LEG_POS.items():
        ax = axes2[r, c]
        for jname, jcol in zip(JOINTS, JOINT_COLORS):
            ax.plot(t, df[f"{leg}_{jname}_Pos (deg)"].values, color=jcol, linewidth=1.3)
        fmt(ax, leg, "Position (deg)")
    axes2[1, 0].set_xlabel("Time (s)")
    axes2[1, 1].set_xlabel("Time (s)")
    fig2.legend(handles=joint_handles, loc="lower center",
                ncol=3, fontsize=9, framealpha=0.9, bbox_to_anchor=(0.5, -0.01))
    fig2.tight_layout(rect=[0, 0.04, 1, 1])

    # ── Figure 3 — Position Error ─────────────────────────────────────────────
    fig3, axes3 = plt.subplots(2, 2, figsize=(15, 9))
    fig3.suptitle("Position Error (Measured − Desired) — Bezier Walk",
                  fontsize=13, fontweight="bold")
    for leg, (r, c) in LEG_POS.items():
        ax = axes3[r, c]
        for jname, jcol in zip(JOINTS, JOINT_COLORS):
            error = df[f"{leg}_{jname}_Pos (deg)"].values - df[f"{leg}_{jname}_Cmd (deg)"].values
            ax.plot(t[::2], error[::2], color=jcol, linewidth=1.3)
        ax.axhline(0, color="black", linewidth=0.8, linestyle=":", zorder=1)
        ax.set_ylim(-3, 3)
        fmt(ax, leg, "Error (deg)")
    axes3[1, 0].set_xlabel("Time (s)")
    axes3[1, 1].set_xlabel("Time (s)")
    fig3.legend(handles=joint_handles, loc="lower center",
                ncol=3, fontsize=9, framealpha=0.9, bbox_to_anchor=(0.5, -0.01))
    fig3.tight_layout(rect=[0, 0.04, 1, 1])

    # ── Figure 4 — Torques ────────────────────────────────────────────────────
    torque_cols = [f"{leg}_{jname}_Torque" for leg in LEGS for jname in JOINTS]
    torque_ylim = int(np.ceil(df[torque_cols].abs().values.max()))
    fig4, axes4 = plt.subplots(2, 2, figsize=(15, 9))
    fig4.suptitle("Measured Joint Torques — Bezier Walk",
                  fontsize=13, fontweight="bold")
    t_ds = t[::2]
    for leg, (r, c) in LEG_POS.items():
        ax = axes4[r, c]
        for jname, jcol in zip(JOINTS, JOINT_COLORS):
            ax.plot(t_ds, df[f"{leg}_{jname}_Torque"].values[::2],
                    color=jcol, linewidth=1.0, alpha=0.85)
        ax.axhline(0, color="black", linewidth=0.8, linestyle=":", zorder=1)
        ax.set_ylim(-torque_ylim, torque_ylim)
        fmt(ax, leg, "Torque (Nm)")
    axes4[1, 0].set_xlabel("Time (s)")
    axes4[1, 1].set_xlabel("Time (s)")
    fig4.legend(handles=joint_handles, loc="lower center",
                ncol=3, fontsize=9, framealpha=0.9, bbox_to_anchor=(0.5, -0.01))
    fig4.tight_layout(rect=[0, 0.04, 1, 1])

    # ── Figure 5 — Zoomed Position Error ──────────────────────────────────────
    fig5, axes5 = plt.subplots(2, 2, figsize=(15, 9))
    fig5.suptitle("Zoomed Position Error (Measured − Desired) — Bezier Walk",
                  fontsize=13, fontweight="bold")
    for leg, (r, c) in LEG_POS.items():
        ax = axes5[r, c]
        for jname, jcol in zip(JOINTS, JOINT_COLORS):
            error = df[f"{leg}_{jname}_Pos (deg)"].values - df[f"{leg}_{jname}_Cmd (deg)"].values
            ax.plot(t[::2], error[::2], color=jcol, linewidth=1.3)
        ax.axhline(0, color="black", linewidth=0.8, linestyle=":", zorder=1)
        ax.set_ylim(-0.5, 0.5)
        fmt(ax, leg, "Error (deg)")
    axes5[1, 0].set_xlabel("Time (s)")
    axes5[1, 1].set_xlabel("Time (s)")
    fig5.legend(handles=joint_handles, loc="lower center",
                ncol=3, fontsize=9, framealpha=0.9, bbox_to_anchor=(0.5, -0.01))
    fig5.tight_layout(rect=[0, 0.04, 1, 1])

    # ── Statistics report ─────────────────────────────────────────────────────
    lines: list[str] = []

    def emit(text: str = "") -> None:
        print(text)
        lines.append(text)

    emit("=" * 62)
    emit("  Bezier Walk — Position Tracking Report")
    emit(f"  Generated : {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    emit(f"  CSV       : {os.path.basename(csv_path)}")
    emit(f"  Duration  : {t_duration:.1f} s  ({num_cycles} cycles)")
    emit("=" * 62)
    emit(f"\n{'Leg':<6}{'Joint':<8}{'RMS error (deg)':>16}{'Max |error| (deg)':>20}{'RMS torque (Nm)':>18}")
    emit("─" * 68)

    total_rms = 0.0
    for leg in LEGS:
        for jname in JOINTS:
            cmd   = df[f"{leg}_{jname}_Cmd (deg)"].values
            meas  = df[f"{leg}_{jname}_Pos (deg)"].values
            error = meas - cmd
            rms_e = np.sqrt(np.mean(error ** 2))
            max_e = np.max(np.abs(error))
            total_rms += rms_e
            emit(f"{leg:<6}{jname:<8}{rms_e:>16.3f}{max_e:>20.3f}")

    emit("─" * 68)
    emit(f"Sum of 12 RMS position errors: {total_rms:.3f} deg")
    emit("─" * 68)

    pid_path = os.path.splitext(csv_path)[0] + "_PID.txt"
    if os.path.isfile(pid_path):
        with open(pid_path) as pf:
            pid_lines = pf.readlines()
        fl_block, in_fl = [], False
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

    # ── Save outputs ──────────────────────────────────────────────────────────
    timestamp = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')

    _m = re.search(r"_Test_(\d+)_", base_name)
    test_num_str = _m.group(1).zfill(2) if _m else "00"

    output_dir = os.path.join(DATA_DIR, f"Plots_For_Tests_{test_num_str}")
    os.makedirs(output_dir, exist_ok=True)

    for fig, label in [
        (fig1, "Desired_Positions"),
        (fig2, "Measured_Positions"),
        (fig3, "Position_Error"),
        (fig4, "Torques"),
        (fig5, "Zoomed_Position_Error"),
    ]:
        fig_path = os.path.join(output_dir, f"{base_name}_{label}_{timestamp}.png")
        fig.savefig(fig_path, dpi=150, bbox_inches="tight")
        print(f"  Saved: {os.path.relpath(fig_path, DATA_DIR)}")

    report_path = os.path.join(output_dir, f"{base_name}_Report_{timestamp}.txt")
    with open(report_path, "w") as f:
        f.write("\n".join(lines) + "\n")
    print(f"  Report: {os.path.relpath(report_path, DATA_DIR)}")

    plt.close("all")


# ─────────────────────────────────────────────────────────────────────────────
# Main — process all CSV files
# ─────────────────────────────────────────────────────────────────────────────
if len(sys.argv) > 1:
    # Accept explicit file paths or test numbers as arguments
    csv_files = []
    for arg in sys.argv[1:]:
        if os.path.isfile(arg):
            csv_files.append(arg)
        elif arg.isdigit():
            pattern = os.path.join(DATA_DIR, f"STW_TorquePos_Log_Test_{arg}_*.csv")
            csv_files.extend(sorted(glob.glob(pattern)))
        else:
            print(f"[WARN] Skipping unrecognised argument: {arg}")
else:
    csv_files = sorted(
        glob.glob(os.path.join(DATA_DIR, "STW_TorquePos_Log_Test_*.csv")),
        key=lambda p: int(m.group(1)) if (m := re.search(r"_Test_(\d+)_", p)) else 0,
    )

if not csv_files:
    print("[ERROR] No CSV files found.")
    sys.exit(1)

print(f"\nFound {len(csv_files)} CSV file(s) to process.")
for csv_path in csv_files:
    process_file(csv_path)

print("\nDone.")
