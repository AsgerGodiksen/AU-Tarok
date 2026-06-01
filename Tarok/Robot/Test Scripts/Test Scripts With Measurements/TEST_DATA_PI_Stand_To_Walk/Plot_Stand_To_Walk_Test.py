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
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.lines as mlines
import matplotlib.colors as mcolors
import matplotlib.patches as mpatches

matplotlib.rcParams["font.family"] = "Liberation Serif"

# ─────────────────────────────────────────────────────────────────────────────
# Configuration
# ─────────────────────────────────────────────────────────────────────────────
DATA_DIR = os.path.dirname(os.path.abspath(__file__))

LEGS         = ["FL", "FR", "HL", "HR"]
JOINTS       = ["J1", "J2", "J3"]
JOINT_COLORS = ["tab:blue", "tab:orange", "tab:green"]
DARK_COLORS  = [tuple(x * 0.65 for x in mcolors.to_rgb(c)) for c in JOINT_COLORS]
LIGHT_COLORS = [tuple(x + (1 - x) * 0.5 for x in mcolors.to_rgb(c)) for c in JOINT_COLORS]
JOINT_LABELS = [r"$\theta_1$", r"$\theta_2$", r"$\theta_3$"]
LEG_POS      = {"FL": (0, 0), "FR": (0, 1), "HL": (1, 0), "HR": (1, 1)}

LABEL_SIZE  = 17
TICK_SIZE   = 13
TITLE_SIZE  = 13
LEGEND_SIZE = 13

SWING_TIME    = 1.0
STAND_TIME    = 3.0
TRANSFER_TIME = 1.5
CYCLE_TIME    = SWING_TIME + STAND_TIME + 4 * TRANSFER_TIME  # 8 s

# Swing-phase intervals within one 10 s gait cycle (start, end) in seconds
SWING_INTERVALS = {
    "HL": (0.75,  1.75),
    "FL": (3.25,  4.25),
    "HR": (5.75,  6.75),
    "FR": (8.25,  9.25),
}

joint_handles = [
    mlines.Line2D([], [], color=c, linewidth=2, label=lbl)
    for c, lbl in zip(JOINT_COLORS, JOINT_LABELS)
]

cycle_handle = mlines.Line2D([], [], color="grey", linewidth=0.9,
                              linestyle="--", alpha=0.5, label="New Cycle")
swing_handle = mpatches.Patch(facecolor="lightgray", alpha=0.6, label="Swing Phase")


# ─────────────────────────────────────────────────────────────────────────────
# Per-file processing
# ─────────────────────────────────────────────────────────────────────────────
def format_ax(ax, leg, ylabel, t, cycle_boundaries):
    for tb in cycle_boundaries:
        ax.axvline(tb, color="grey", linewidth=0.9, linestyle="--", alpha=0.5)
    ax.set_title(f"{leg} Leg", fontsize=TITLE_SIZE, fontweight="bold")
    ax.set_ylabel(ylabel, fontsize=LABEL_SIZE)
    ax.set_xlim(t[0], t[-1])
    ax.grid(True, alpha=0.3)
    ax.tick_params(labelsize=TICK_SIZE)


def add_swing_spans(ax, leg, t_start, t_end):
    """Draw light-gray boxes for every swing phase of *leg* across all cycles."""
    s0, s1 = SWING_INTERVALS[leg]
    cycle = 0
    while True:
        x0 = t_start + cycle * CYCLE_TIME + s0
        x1 = t_start + cycle * CYCLE_TIME + s1
        if x0 >= t_end:
            break
        ax.axvspan(x0, min(x1, t_end), color="lightgray", alpha=0.6, zorder=0)
        cycle += 1


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
    fig1, axes1 = plt.subplots(2, 2, figsize=(14, 8), sharex=True)
    for leg, (r, c) in LEG_POS.items():
        ax = axes1[r, c]
        for jname, jcol in zip(JOINTS, JOINT_COLORS):
            ax.plot(t, df[f"{leg}_{jname}_Cmd (deg)"].values, color=jcol, linewidth=1.3)
        ax.set_ylim(-80, 80)
        fmt(ax, leg, "Position (deg)")
    for ax in [axes1[0, 1], axes1[1, 1]]:
        ax.tick_params(labelleft=False)
        ax.set_ylabel("")
    axes1[1, 0].set_xlabel("Time (s)", fontsize=LABEL_SIZE)
    axes1[1, 1].set_xlabel("Time (s)", fontsize=LABEL_SIZE)
    fig1.legend(handles=joint_handles, loc="lower center",
                ncol=3, fontsize=LEGEND_SIZE, framealpha=0.9, bbox_to_anchor=(0.5, -0.0))
    fig1.tight_layout()
    fig1.subplots_adjust(bottom=0.09)

    # ── Figure 2 — Measured Positions ────────────────────────────────────────
    fig2, axes2 = plt.subplots(2, 2, figsize=(14, 8), sharex=True)
    for leg, (r, c) in LEG_POS.items():
        ax = axes2[r, c]
        for jname, jcol in zip(JOINTS, JOINT_COLORS):
            ax.plot(t, df[f"{leg}_{jname}_Pos (deg)"].values, color=jcol, linewidth=1.3)
        fmt(ax, leg, "Position (deg)")
    for ax in [axes2[0, 1], axes2[1, 1]]:
        ax.tick_params(labelleft=False)
        ax.set_ylabel("")
    axes2[1, 0].set_xlabel("Time (s)", fontsize=LABEL_SIZE)
    axes2[1, 1].set_xlabel("Time (s)", fontsize=LABEL_SIZE)
    fig2.legend(handles=joint_handles, loc="lower center",
                ncol=3, fontsize=LEGEND_SIZE, framealpha=0.9, bbox_to_anchor=(0.5, -0.0))
    fig2.tight_layout()
    fig2.subplots_adjust(bottom=0.09)

    # ── Figure 3 — Position Error ─────────────────────────────────────────────
    fig3, axes3 = plt.subplots(2, 2, figsize=(14, 8), sharex=True)
    for leg, (r, c) in LEG_POS.items():
        ax = axes3[r, c]
        add_swing_spans(ax, leg, t[0], t[-1])
        for jname, jcol in zip(JOINTS, JOINT_COLORS):
            error = df[f"{leg}_{jname}_Pos (deg)"].values - df[f"{leg}_{jname}_Cmd (deg)"].values
            ax.plot(t[::2], error[::2], color=jcol, linewidth=1.3)
        ax.axhline(0, color="black", linewidth=0.8, linestyle=":", zorder=1)
        ax.set_ylim(-2, 2)
        fmt(ax, leg, r"Error ($^\circ$)")
    for ax in [axes3[0, 1], axes3[1, 1]]:
        ax.tick_params(labelleft=False)
        ax.set_ylabel("")
    axes3[1, 0].set_xlabel("Time (s)", fontsize=LABEL_SIZE)
    axes3[1, 1].set_xlabel("Time (s)", fontsize=LABEL_SIZE)
    fig3.legend(handles=joint_handles + [cycle_handle, swing_handle], loc="lower center",
                ncol=5, fontsize=LEGEND_SIZE, framealpha=0.9, bbox_to_anchor=(0.5, -0.03))
    fig3.tight_layout()
    fig3.subplots_adjust(bottom=0.09)

    # ── Figure 4 — Torques ────────────────────────────────────────────────────
    torque_cols = [f"{leg}_{jname}_Torque" for leg in LEGS for jname in JOINTS]
    torque_ylim = int(np.ceil(df[torque_cols].abs().values.max()))
    fig4, axes4 = plt.subplots(2, 2, figsize=(14, 8), sharex=True)
    t_ds = t[::2]
    for leg, (r, c) in LEG_POS.items():
        ax = axes4[r, c]
        add_swing_spans(ax, leg, t[0], t[-1])
        for jname, jcol in zip(JOINTS, JOINT_COLORS):
            ax.plot(t_ds, df[f"{leg}_{jname}_Torque"].values[::2],
                    color=jcol, linewidth=1.0, alpha=0.85)
        ax.axhline(0, color="black", linewidth=0.8, linestyle=":", zorder=1)
        ax.set_ylim(-torque_ylim, torque_ylim)
        fmt(ax, leg, "Torque (Nm)")
    for ax in [axes4[0, 1], axes4[1, 1]]:
        ax.tick_params(labelleft=False)
        ax.set_ylabel("")
    axes4[1, 0].set_xlabel("Time (s)", fontsize=LABEL_SIZE)
    axes4[1, 1].set_xlabel("Time (s)", fontsize=LABEL_SIZE)
    fig4.legend(handles=joint_handles + [cycle_handle, swing_handle], loc="lower center",
                ncol=5, fontsize=LEGEND_SIZE, framealpha=0.9, bbox_to_anchor=(0.5, -0.03))
    fig4.tight_layout()
    fig4.subplots_adjust(bottom=0.09)

    # ── Figure 5 — Zoomed Position Error ──────────────────────────────────────
    fig5, axes5 = plt.subplots(2, 2, figsize=(14, 8), sharex=True)
    for leg, (r, c) in LEG_POS.items():
        ax = axes5[r, c]
        for jname, jcol in zip(JOINTS, JOINT_COLORS):
            error = df[f"{leg}_{jname}_Pos (deg)"].values - df[f"{leg}_{jname}_Cmd (deg)"].values
            ax.plot(t[::2], error[::2], color=jcol, linewidth=1.3)
        ax.axhline(0, color="black", linewidth=0.8, linestyle=":", zorder=1)
        ax.set_ylim(-0.5, 0.5)
        fmt(ax, leg, r"Error ($^\circ$)")
    for ax in [axes5[0, 1], axes5[1, 1]]:
        ax.tick_params(labelleft=False)
        ax.set_ylabel("")
    axes5[1, 0].set_xlabel("Time (s)", fontsize=LABEL_SIZE)
    axes5[1, 1].set_xlabel("Time (s)", fontsize=LABEL_SIZE)
    fig5.legend(handles=joint_handles, loc="lower center",
                ncol=3, fontsize=LEGEND_SIZE, framealpha=0.9, bbox_to_anchor=(0.5, -0.0))
    fig5.tight_layout()
    fig5.subplots_adjust(bottom=0.09)

    # ── Figure 6 — Desired (darker solid) vs Measured (dashed) Positions ────────
    fig6, axes6 = plt.subplots(2, 2, figsize=(14, 8), sharex=True)
    for leg, (r, c) in LEG_POS.items():
        ax = axes6[r, c]
        for jname, dcol, lcol in zip(JOINTS, DARK_COLORS, LIGHT_COLORS):
            ax.plot(t, df[f"{leg}_{jname}_Cmd (deg)"].values,
                    color=dcol, linewidth=2.0, linestyle="-")
            ax.plot(t, df[f"{leg}_{jname}_Pos (deg)"].values,
                    color=lcol, linewidth=1.3, linestyle="--")
        ax.set_ylim(-80, 80)
        fmt(ax, leg, r"Position ($^\circ$)")
    for ax in [axes6[0, 1], axes6[1, 1]]:
        ax.tick_params(labelleft=False)
        ax.set_ylabel("")
    axes6[1, 0].set_xlabel("Time (s)", fontsize=LABEL_SIZE)
    axes6[1, 1].set_xlabel("Time (s)", fontsize=LABEL_SIZE)
    style_handles = [
        mlines.Line2D([], [], color="black", linewidth=1.5, linestyle="-",  label="Desired"),
        mlines.Line2D([], [], color="black", linewidth=1.5, linestyle="--", label="Measured"),
    ]
    fig6.legend(handles=joint_handles + style_handles, loc="lower center",
                ncol=5, fontsize=LEGEND_SIZE, framealpha=0.9, bbox_to_anchor=(0.5, -0.0))
    fig6.tight_layout()
    fig6.subplots_adjust(bottom=0.13)

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
        (fig6, "Desired_vs_Measured"),
    ]:
        fig_path = os.path.join(output_dir, f"{base_name}_{label}_{timestamp}.png")
        fig.savefig(fig_path, dpi=600, bbox_inches="tight")
        print(f"  Saved: {os.path.relpath(fig_path, DATA_DIR)}")

    report_path = os.path.join(output_dir, f"{base_name}_Report_{timestamp}.txt")
    with open(report_path, "w") as f:
        f.write("\n".join(lines) + "\n")
    print(f"  Report: {os.path.relpath(report_path, DATA_DIR)}")

    plt.close("all")


# ─────────────────────────────────────────────────────────────────────────────
# Main — process all CSV files
# ─────────────────────────────────────────────────────────────────────────────

# Discover all available test CSV files sorted by test number
all_csv_files = sorted(
    glob.glob(os.path.join(DATA_DIR, "STW_TorquePos_Log_Test_*.csv")),
    key=lambda p: int(m.group(1)) if (m := re.search(r"_Test_(\d+)_", p)) else 0,
)

if not all_csv_files:
    print("[ERROR] No CSV files found in the directory.")
    sys.exit(1)

# Determine available test numbers
test_numbers = []
for p in all_csv_files:
    m = re.search(r"_Test_(\d+)_", p)
    if m:
        test_numbers.append(int(m.group(1)))

print(f"\nAvailable tests: {test_numbers}")

if len(sys.argv) > 1:
    # Command-line mode: accept individual test numbers or file paths
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
    # Interactive mode: ask for start and stop test numbers
    def prompt_test_number(prompt_text: str, available: list[int]) -> int:
        while True:
            raw = input(prompt_text).strip()
            if raw == "":
                return available[0] if "start" in prompt_text.lower() else available[-1]
            if raw.isdigit() and int(raw) in available:
                return int(raw)
            print(f"  Please enter one of: {available}")

    start_num = prompt_test_number(
        f"Start from test number [{test_numbers[0]}]: ", test_numbers
    )
    stop_num = prompt_test_number(
        f"Stop at test number [{test_numbers[-1]}]: ", test_numbers
    )

    if start_num > stop_num:
        print(f"[ERROR] Start test ({start_num}) is greater than stop test ({stop_num}).")
        sys.exit(1)

    csv_files = [
        p for p, n in zip(all_csv_files, test_numbers)
        if start_num <= n <= stop_num
    ]
    print(f"Processing tests {start_num} to {stop_num} ({len(csv_files)} file(s)).")

if not csv_files:
    print("[ERROR] No matching CSV files found for the selected range.")
    sys.exit(1)

print(f"\nFound {len(csv_files)} CSV file(s) to process.")
for csv_path in csv_files:
    process_file(csv_path)

print("\nDone.")
