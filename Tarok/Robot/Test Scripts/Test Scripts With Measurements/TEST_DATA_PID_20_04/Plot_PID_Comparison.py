"""
Plot_PID_Comparison.py

Overlays measured joint positions on top of the command reference so that
different PID tunings can be compared against the same reference.

Supports two CSV formats:
  New (_2 logger): contains FL_J1_Cmd (deg) … HR_J3_Cmd (deg) columns —
                   reference is read directly from the CSV (fast).
  Old (original):  no Cmd columns — reference is recomputed from inverse
                   kinematics (slower, kept for backward compatibility).

Usage:
    python Plot_PID_Comparison.py 1 2 3   # compare tests 1, 2 and 3
    python Plot_PID_Comparison.py          # interactive prompt

Output:
    Figure 1 — Measured vs Command/Reference Joint Position (one subplot per leg)
    Figure 2 — Position Error: Measured − Command/Reference  (one subplot per leg)
"""

import sys
import os
import glob

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.lines as mlines

import matplotlib
matplotlib.rcParams["font.family"] = "Liberation Serif"

DATA_DIR = os.path.dirname(os.path.abspath(__file__))

LEGS         = ["FL", "FR", "HL", "HR"]
JOINTS       = ["J1", "J2", "J3"]
JOINT_LABELS = [r"$\theta_1$", r"$\theta_2$", r"$\theta_3$"]
JOINT_COLORS = ["tab:blue", "tab:orange", "tab:green"]

LABEL_SIZE  = 17
TICK_SIZE   = 13
TITLE_SIZE  = 13
LEGEND_SIZE = 13

FIG2_DPI = 600


# ══════════════════════════════════════════════════════════════════════════════
# 1.  Load measured test data
# ══════════════════════════════════════════════════════════════════════════════
def load_test(test_num: int) -> tuple[pd.DataFrame, str] | None:
    pattern = os.path.join(DATA_DIR, f"Stand_Pose_TorquePos_Log_TEST_{test_num}_*.csv")
    matches = sorted(glob.glob(pattern))
    if not matches:
        print(f"  [WARNING] No TorquePos CSV found for test {test_num}.")
        return None
    if len(matches) == 1:
        csv_path = matches[0]
    else:
        print(f"  Multiple files found for test {test_num}:")
        for i, p in enumerate(matches):
            print(f"    {chr(ord('a') + i)}) {os.path.basename(p)}")
        choice   = input("  Choose (a/b/…): ").strip().lower()
        csv_path = matches[ord(choice) - ord('a')]
    df = pd.read_csv(csv_path)
    print(f"  Loaded: {os.path.basename(csv_path)}  ({len(df)} rows)")
    return df, csv_path


if len(sys.argv) > 1:
    test_numbers = [int(x) for x in sys.argv[1:]]
else:
    raw = input("Enter test number(s) to compare (space-separated): ")
    test_numbers = [int(x) for x in raw.split()]

print(f"\nLoading {len(test_numbers)} test(s) …")
tests:      dict[int, pd.DataFrame] = {}
test_paths: dict[int, str]          = {}   # csv path per test (for companion PID file lookup)
for num in test_numbers:
    result = load_test(num)
    if result is not None:
        tests[num], test_paths[num] = result

if not tests:
    print("No valid test data loaded. Exiting.")
    sys.exit(1)

# ── Detect CSV format ──────────────────────────────────────────────────────────
HAS_CMD_COLS = "FL_J1_Cmd (deg)" in next(iter(tests.values())).columns


# ══════════════════════════════════════════════════════════════════════════════
# 2.  Reference trajectory
#     New format: read Cmd columns from the first loaded test (fast).
#     Old format: recompute from inverse kinematics (slow, backward-compatible).
# ══════════════════════════════════════════════════════════════════════════════
if HAS_CMD_COLS:
    print("\nNew CSV format detected — reading reference from Cmd columns.")

    # Use the first test's Cmd columns as the single shared reference.
    # All tests share the same analytical trajectory so their Cmd columns are identical.
    ref_df  = next(iter(tests.values()))
    t_ref   = ref_df["Timestamp (s)"].values

    # analytical[leg] shape: (N, 3) — same layout as the kinematics path
    analytical = {
        leg: np.column_stack([ref_df[f"{leg}_{j}_Cmd (deg)"].values for j in JOINTS])
        for leg in LEGS
    }

    def get_ref_vals(leg: str, df: pd.DataFrame, j: int) -> np.ndarray:
        """Reference values aligned to each row of df (same-row Cmd column)."""
        return df[f"{leg}_{JOINTS[j]}_Cmd (deg)"].values

    # For Figure 1 the reference is already the full test duration — no tiling needed.
    t_ref_plot    = t_ref
    analytical_plot = analytical

else:
    print("\nOld CSV format detected — computing reference from inverse kinematics …")

    sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../..')))
    from Robot.Kinematics.Inverse_Kinematics import *
    from Robot.Kinematics.Constant_Transforms import *

    l_k = 0.7048
    w_k = 0.220
    dt_ref         = 0.01
    total_time     = 10
    num_time_steps = int(total_time / dt_ref) + 1
    t_ref = np.linspace(0, total_time, num_time_steps)

    x_FL = x_FR = (l_k / 2) * np.ones_like(t_ref)
    x_HL = x_HR = (-l_k / 2) * np.ones_like(t_ref)
    y_FL = y_HL = (w_k / 2 + 0.078) * np.ones_like(t_ref)
    y_FR = y_HR = (-w_k / 2 - 0.078) * np.ones_like(t_ref)
    z = np.piecewise(
        t_ref,
        [t_ref < 5, t_ref >= 5],
        [lambda t: -0.46 + (0.12 / 5) * t,
         lambda t: -0.34 - (0.12 / 5) * (t - 5)],
    )
    P_body = {
        "FL": np.vstack((x_FL, y_FL, z)),
        "FR": np.vstack((x_FR, y_FR, z)),
        "HL": np.vstack((x_HL, y_HL, z)),
        "HR": np.vstack((x_HR, y_HR, z)),
    }
    analytical = {}
    for leg in LEGS:
        P_base    = np.array([T0_B(P_body[leg][:, i].reshape((3, 1)), leg) for i in range(len(t_ref))])
        theta_rad = np.array([Inverse_Kinematics(P_base[i], leg) for i in range(len(t_ref))])
        analytical[leg] = np.rad2deg(theta_rad)
    print("  Done.\n")

    def get_ref_vals(leg: str, df: pd.DataFrame, j: int) -> np.ndarray:
        """Reference values looked up via Trajectory Index column."""
        idx = np.clip(df["Trajectory Index"].values.round().astype(int), 0, num_time_steps - 1)
        return analytical[leg][idx, j]

    # Tile reference 3× for Figure 1 so it spans the full test duration
    NUM_REPEATS     = 3
    t_ref_plot      = np.concatenate([t_ref + i * total_time for i in range(NUM_REPEATS)])
    analytical_plot = {leg: np.tile(analytical[leg], (NUM_REPEATS, 1)) for leg in LEGS}

print()


# ══════════════════════════════════════════════════════════════════════════════
# 3.  Figure 1 — Measured vs Reference Joint Position
# ══════════════════════════════════════════════════════════════════════════════
fig1, axes1 = plt.subplots(2, 2, figsize=(15, 9))
fig1.suptitle("Joint Position: Measured vs Reference Command",
              fontsize=13, fontweight="bold")

leg_axes1 = {"FL": axes1[0, 0], "FR": axes1[0, 1],
             "HL": axes1[1, 0], "HR": axes1[1, 1]}

mstyles = ["-", ":", "-."]

for leg, ax in leg_axes1.items():
    # Reference — dashed, one color per joint
    for j, (jname, jcol) in enumerate(zip(JOINTS, JOINT_COLORS)):
        ax.plot(t_ref_plot, analytical_plot[leg][:, j],
                color=jcol, linestyle="--", linewidth=1.4, alpha=0.75,
                zorder=2)

    # Measured — solid/dotted/dash-dot per test, same joint colors
    for k, (num, df) in enumerate(tests.items()):
        t_meas = df["Timestamp (s)"].values
        mstyle = mstyles[k % len(mstyles)]
        for j, (jname, jcol) in enumerate(zip(JOINTS, JOINT_COLORS)):
            ax.plot(t_meas, df[f"{leg}_{jname}_Pos (deg)"].values,
                    color=jcol, linestyle=mstyle, linewidth=1.5,
                    zorder=3)

    ax.set_title(f"{leg} Leg", fontsize=11)
    ax.set_ylabel("Position (deg)")
    ax.set_xlim(left=0)
    ax.grid(True, alpha=0.3)

axes1[1, 0].set_xlabel("Time (s)")
axes1[1, 1].set_xlabel("Time (s)")

# Shared legend
joint_handles = [
    mlines.Line2D([], [], color=c, linewidth=2, label=j)
    for j, c in zip(JOINTS, JOINT_COLORS)
]
source_handles = [
    mlines.Line2D([], [], color="black", linestyle="--", linewidth=1.4,
                  alpha=0.75, label="Reference cmd")
]
for k, num in enumerate(tests):
    source_handles.append(
        mlines.Line2D([], [], color="black",
                      linestyle=mstyles[k % len(mstyles)],
                      linewidth=1.5, label=f"Test {num}")
    )

fig1.legend(
    handles=joint_handles + source_handles,
    loc="lower center",
    ncol=len(joint_handles) + len(source_handles),
    fontsize=9,
    framealpha=0.9,
    bbox_to_anchor=(0.5, -0.02),
)
fig1.tight_layout(rect=[0, 0.05, 1, 1])


# ══════════════════════════════════════════════════════════════════════════════
# 4.  Figure 2 — Position Error (Measured − Reference)
# ══════════════════════════════════════════════════════════════════════════════
fig2, axes2 = plt.subplots(2, 2, figsize=(14, 8), sharex=True)

leg_axes2 = {"FL": axes2[0, 0], "FR": axes2[0, 1],
             "HL": axes2[1, 0], "HR": axes2[1, 1]}

for leg, ax in leg_axes2.items():
    for k, (num, df) in enumerate(tests.items()):
        t_meas = df["Timestamp (s)"].values
        mstyle = mstyles[k % len(mstyles)]

        for j, (jname, jcol) in enumerate(zip(JOINTS, JOINT_COLORS)):
            error = df[f"{leg}_{jname}_Pos (deg)"].values - get_ref_vals(leg, df, j)
            ax.plot(t_meas, error,
                    color=jcol, linestyle=mstyle, linewidth=1.3)

    ax.axhline(0, color="black", linewidth=0.8, linestyle=":", zorder=2)
    ax.set_title(f"{leg} Leg", fontsize=TITLE_SIZE, fontweight="bold")
    ax.set_ylim(-0.1, 0.1)
    ax.set_xlim(left=0)
    ax.grid(True, alpha=0.3)
    ax.tick_params(labelsize=TICK_SIZE)

for ax in [axes2[0, 1], axes2[1, 1]]:
    ax.tick_params(labelleft=False)

axes2[0, 0].set_ylabel(r"Error ($^\circ$)", fontsize=LABEL_SIZE)
axes2[1, 0].set_ylabel(r"Error ($^\circ$)", fontsize=LABEL_SIZE)
axes2[1, 0].set_xlabel("Time (s)", fontsize=LABEL_SIZE)
axes2[1, 1].set_xlabel("Time (s)", fontsize=LABEL_SIZE)

joint_handles2 = [
    mlines.Line2D([], [], color=c, linewidth=2, label=lbl)
    for lbl, c in zip(JOINT_LABELS, JOINT_COLORS)
]
test_handles2 = [
    mlines.Line2D([], [], color="black", linestyle=mstyles[k % len(mstyles)],
                  linewidth=1.5, label=f"Test {num}")
    for k, num in enumerate(tests)
] if len(tests) > 1 else []
fig2.legend(
    handles=joint_handles2 + test_handles2,
    loc="lower center",
    ncol=len(joint_handles2) + len(test_handles2),
    fontsize=LEGEND_SIZE,
    framealpha=0.9,
    bbox_to_anchor=(0.5, 0.0),
)
fig2.tight_layout()
fig2.subplots_adjust(bottom=0.09)

out_fig2 = os.path.join(
    DATA_DIR,
    f"PID_Comparison_Error_{'_vs_'.join(str(n) for n in tests)}.png",
)
fig2.savefig(out_fig2, dpi=FIG2_DPI)
print(f"Figure 2 saved: {os.path.basename(out_fig2)}")



# ══════════════════════════════════════════════════════════════════════════════
# 5.  Figure 3 — Measured Torque over Time
# ══════════════════════════════════════════════════════════════════════════════
fig3, axes3 = plt.subplots(2, 2, figsize=(15, 9))
fig3.suptitle("Measured Joint Torque over Time",
              fontsize=13, fontweight="bold")

leg_axes3 = {"FL": axes3[0, 0], "FR": axes3[0, 1],
             "HL": axes3[1, 0], "HR": axes3[1, 1]}

for leg, ax in leg_axes3.items():
    for k, (num, df) in enumerate(tests.items()):
        t_meas = df["Timestamp (s)"].values
        mstyle = mstyles[k % len(mstyles)]
        for j, (jname, jcol) in enumerate(zip(JOINTS, JOINT_COLORS)):
            col = f"{leg}_{jname}_Torque"
            if col in df.columns:
                ax.plot(t_meas, df[col].values,
                        color=jcol, linestyle=mstyle, linewidth=1.3,
                        label=f"T{num} {jname}")
    ax.set_title(f"{leg} Leg", fontsize=11)
    ax.set_ylabel("Torque (Nm)")
    ax.set_ylim(-15, 15)
    ax.set_xlim(left=0)
    ax.axhline(0, color="black", linewidth=0.6, linestyle=":", zorder=2)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right", fontsize=7,
              ncol=max(1, len(tests)), framealpha=0.85)

axes3[1, 0].set_xlabel("Time (s)")
axes3[1, 1].set_xlabel("Time (s)")
fig3.tight_layout()


# ══════════════════════════════════════════════════════════════════════════════
# 6.  Summary statistics + report file
# ══════════════════════════════════════════════════════════════════════════════
import datetime

report_filename = os.path.join(
    DATA_DIR,
    f"PID_Comparison_Report_{'_vs_'.join(str(n) for n in tests)}"
    f"_{datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.txt",
)

def _load_pid_file(csv_path: str) -> str:
    """Return contents of the companion _PID.txt file, or a notice if not found."""
    pid_path = csv_path.replace(".csv", "_PID.txt")
    if os.path.isfile(pid_path):
        with open(pid_path) as f:
            return f.read()
    return "  (no companion PID file found)\n"

lines: list[str] = []

def emit(text: str = "") -> None:
    """Print and append to report lines simultaneously."""
    print(text)
    lines.append(text)

emit("=" * 60)
emit(f"  PID Comparison Report")
emit(f"  Generated : {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
emit(f"  Tests     : {', '.join(str(n) for n in tests)}")
emit("=" * 60)

# ── PID parameters section ────────────────────────────────────────────────────
emit("\nPID PARAMETERS")
emit("─" * 60)
for num, csv_path in test_paths.items():
    emit(f"\n  Test {num}  —  {os.path.basename(csv_path)}")
    for line in _load_pid_file(csv_path).splitlines():
        emit("  " + line)

# ── Statistics table ──────────────────────────────────────────────────────────
emit("\n\nSTATISTICS")
emit("─" * 60)
emit(f"{'Test':<8}{'Leg':<6}{'Joint':<8}{'RMS error (deg)':>16}{'Max |error| (deg)':>20}")
emit("─" * 60)
for num, df in tests.items():
    for leg in LEGS:
        for j, jname in enumerate(JOINTS):
            error   = df[f"{leg}_{jname}_Pos (deg)"].values - get_ref_vals(leg, df, j)
            rms     = np.sqrt(np.mean(error ** 2))
            max_err = np.max(np.abs(error))
            emit(f"{num:<8}{leg:<6}{jname:<8}{rms:>16.3f}{max_err:>20.3f}")
emit("─" * 60)

# ── Write combined report file ────────────────────────────────────────────────
with open(report_filename, "w") as f:
    f.write("\n".join(lines) + "\n")
print(f"\nReport saved to {os.path.basename(report_filename)}")

# ── Append statistics to each test's companion _PID.txt ──────────────────────
for num, df in tests.items():
    pid_path = test_paths[num].replace(".csv", "_PID.txt")
    if not os.path.isfile(pid_path):
        continue
    with open(pid_path, "a") as f:
        f.write(f"\n\nSTATISTICS — appended {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write("=" * 52 + "\n")
        f.write(f"{'Leg':<6}{'Joint':<8}{'RMS error (deg)':>16}{'Max |error| (deg)':>20}\n")
        f.write("─" * 52 + "\n")
        for leg in LEGS:
            for j, jname in enumerate(JOINTS):
                error   = df[f"{leg}_{jname}_Pos (deg)"].values - get_ref_vals(leg, df, j)
                rms     = np.sqrt(np.mean(error ** 2))
                max_err = np.max(np.abs(error))
                f.write(f"{leg:<6}{jname:<8}{rms:>16.3f}{max_err:>20.3f}\n")
        f.write("─" * 52 + "\n")
    print(f"Statistics appended to {os.path.basename(pid_path)}")

plt.show()
