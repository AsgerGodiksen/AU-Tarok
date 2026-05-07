"""
Stand_To_Walk_Data_Analysis.py

Computes per-leg, per-joint RMSE of position tracking error (Measured − Desired)
and mean absolute torque, split into swing and stand phases, aggregated across
all 3 sequences in each test.

Gait timing (from Stand_To_Walk_TorquePos_Log.py, one 10-second cycle):
  t =  0.00 s  cycle starts
  t =  0.75 s  HL swing starts
  t =  1.75 s  HL swing ends
  t =  3.25 s  FL swing starts   (1.5 s CoM transfer before)
  t =  4.25 s  FL swing ends
  t =  5.75 s  HR swing starts   (1.5 s CoM transfer before)
  t =  6.75 s  HR swing ends
  t =  8.25 s  FR swing starts   (1.5 s CoM transfer before)
  t =  9.25 s  FR swing ends
  t = 10.00 s  cycle repeats     (0.75 s CoM transfer after FR)

Phase definitions:
  Swing phase = 1 s window when the leg's foot is in the air.
  Stand phase = all remaining time (9 s), including CoM transfer movements.

Cycle boundaries are detected from Trajectory Index resets in the data.
Phase membership is determined purely from the elapsed timestamp within each cycle.

Output: Plots_For_Tests_<NN>/Data_Analysis_For_Test_<NN>_<timestamp>.txt
"""

import sys
import os
import glob
import re
import datetime

import numpy as np
import pandas as pd

# ─────────────────────────────────────────────────────────────────────────────
# Configuration
# ─────────────────────────────────────────────────────────────────────────────
DATA_DIR = os.path.dirname(os.path.abspath(__file__))

LEGS   = ["FL", "FR", "HL", "HR"]
JOINTS = ["J1", "J2", "J3"]

# Swing windows within one cycle: (t_start, t_end) in seconds.
SWING_TIMES = {
    "HL": (0.75, 1.75),
    "FL": (3.25, 4.25),
    "HR": (5.75, 6.75),
    "FR": (8.25, 9.25),
}

CYCLE_DURATION = 10.0   # seconds per gait cycle


# ─────────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────────
def rmse(errors: np.ndarray) -> float:
    if len(errors) == 0:
        return float("nan")
    return float(np.sqrt(np.mean(errors ** 2)))


def find_cycle_starts(df: pd.DataFrame) -> np.ndarray:
    """Return timestamps where each gait cycle begins, detected from Trajectory Index resets."""
    ti = df["Trajectory Index"].values
    t  = df["Timestamp (s)"].values
    reset_rows = np.where(np.diff(ti) < 0)[0] + 1
    return np.concatenate([[t[0]], t[reset_rows]])


def time_in_cycle(t: np.ndarray, cycle_starts: np.ndarray) -> np.ndarray:
    """For each timestamp, return elapsed time since the start of its cycle."""
    idx = np.searchsorted(cycle_starts, t, side="right") - 1
    idx = np.clip(idx, 0, len(cycle_starts) - 1)
    return t - cycle_starts[idx]


def swing_mask(t_cyc: np.ndarray, leg: str) -> np.ndarray:
    """Boolean mask: True where the leg's foot is in swing (in the air)."""
    t0, t1 = SWING_TIMES[leg]
    return (t_cyc >= t0) & (t_cyc < t1)


# ─────────────────────────────────────────────────────────────────────────────
# Per-file processing
# ─────────────────────────────────────────────────────────────────────────────
def process_file(csv_path: str) -> None:
    base_name = os.path.splitext(os.path.basename(csv_path))[0]
    print(f"\nProcessing: {os.path.basename(csv_path)}")

    df = pd.read_csv(csv_path)
    print(f"  Loaded {len(df)} rows")

    cycle_starts = find_cycle_starts(df)
    n_seq = len(cycle_starts)
    print(f"  Detected {n_seq} cycle(s)  (start times: {np.round(cycle_starts, 3)})")

    t_all = df["Timestamp (s)"].values
    t_cyc = time_in_cycle(t_all, cycle_starts)

    # ── Compute RMSE and mean absolute torque per leg, joint, phase ────────────
    results = {}
    for leg in LEGS:
        is_swing = swing_mask(t_cyc, leg)
        is_stand = ~is_swing
        results[leg] = {}
        for jnt in JOINTS:
            error  = (df[f"{leg}_{jnt}_Pos (deg)"].values
                      - df[f"{leg}_{jnt}_Cmd (deg)"].values)
            torque = df[f"{leg}_{jnt}_Torque"].values
            results[leg][jnt] = {
                "stand_rmse":            rmse(error[is_stand]),
                "swing_rmse":            rmse(error[is_swing]),
                "stand_n":               int(is_stand.sum()),
                "swing_n":               int(is_swing.sum()),
                "stand_mean_torque":     float(np.mean(torque[is_stand])),
                "swing_mean_torque":     float(np.mean(torque[is_swing])),
                "stand_mean_abs_torque": float(np.mean(np.abs(torque[is_stand]))),
                "swing_mean_abs_torque": float(np.mean(np.abs(torque[is_swing]))),
            }

    # ── Build report ──────────────────────────────────────────────────────────
    lines: list[str] = []

    def emit(text: str = "") -> None:
        print(text)
        lines.append(text)

    ts = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    emit("=" * 72)
    emit("  Stand-To-Walk — Phase RMSE Analysis")
    emit(f"  Generated : {ts}")
    emit(f"  CSV       : {os.path.basename(csv_path)}")
    emit(f"  Cycles    : {n_seq}  (detected from Trajectory Index resets)")
    emit("=" * 72)
    emit()
    emit("RMSE of position error (Measured − Desired) in [deg].")
    emit("Stand phase (9 s/cycle) includes CoM transfer movements.")
    emit("Swing phase (1 s/cycle) is when the leg's foot is in the air.")
    emit()

    # ── Position RMSE table ───────────────────────────────────────────────────
    col_w = 14
    emit(f"{'Leg':<6}{'Joint':<8}"
         f"{'Stand RMSE (deg)':>{col_w}}  {'Swing RMSE (deg)':>{col_w}}"
         f"  {'Stand N':>8}  {'Swing N':>8}")
    emit("─" * 72)

    total_stand_rms = 0.0
    total_swing_rms = 0.0

    for leg in LEGS:
        leg_stand = 0.0
        leg_swing = 0.0
        for jnt in JOINTS:
            r = results[leg][jnt]
            leg_stand += r["stand_rmse"]
            leg_swing += r["swing_rmse"]
            emit(f"{leg:<6}{jnt:<8}"
                 f"{r['stand_rmse']:>{col_w}.4f}  {r['swing_rmse']:>{col_w}.4f}"
                 f"  {r['stand_n']:>8}  {r['swing_n']:>8}")
        emit(f"{'':6}{'Sum':8}{leg_stand:>{col_w}.4f}  {leg_swing:>{col_w}.4f}")
        emit("─" * 72)
        total_stand_rms += leg_stand
        total_swing_rms += leg_swing

    emit()
    emit(f"Sum of all 12 joint stand RMSE : {total_stand_rms:.4f} deg")
    emit(f"Sum of all 12 joint swing RMSE : {total_swing_rms:.4f} deg")
    emit()

    # ── Mean torque table ─────────────────────────────────────────────────────
    emit("=" * 72)
    emit("  Average Torque per Actuator  [Nm]")
    emit("=" * 72)
    emit()
    emit("Signed mean: average torque including direction.")
    emit("Mean |torque|: average load magnitude regardless of direction.")
    emit()

    emit(f"{'Leg':<6}{'Joint':<8}"
         f"{'Stand mean (Nm)':>{col_w}}  {'Swing mean (Nm)':>{col_w}}"
         f"  {'Stand |mean| (Nm)':>{col_w+3}}  {'Swing |mean| (Nm)':>{col_w+3}}")
    emit("─" * 72)

    for leg in LEGS:
        for jnt in JOINTS:
            r = results[leg][jnt]
            emit(f"{leg:<6}{jnt:<8}"
                 f"{r['stand_mean_torque']:>{col_w}.4f}  {r['swing_mean_torque']:>{col_w}.4f}"
                 f"  {r['stand_mean_abs_torque']:>{col_w+3}.4f}  {r['swing_mean_abs_torque']:>{col_w+3}.4f}")
        emit("─" * 72)
    emit()

    # ── Swing timing reference ────────────────────────────────────────────────
    emit("─" * 72)
    emit(f"Swing timing  (cycle duration = {CYCLE_DURATION:.1f} s, sequence: HL → FL → HR → FR):")
    emit(f"  {'Leg':<5}  {'Swing window':^20}  {'Duration':>10}  {'Stand window':^30}")
    for leg in ["HL", "FL", "HR", "FR"]:
        t0, t1 = SWING_TIMES[leg]
        emit(f"  {leg:<5}  [{t0:.2f} – {t1:.2f}] s            {t1-t0:.1f} s"
             f"     rest of cycle ({CYCLE_DURATION - (t1 - t0):.1f} s)")
    emit()

    # ── PID parameters ────────────────────────────────────────────────────────
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
        emit("PI Parameters (FL leg — representative)")
        emit("─" * 72)
        for ln in fl_block:
            emit(ln)
    else:
        emit(f"[WARNING] PID file not found: {os.path.basename(pid_path)}")

    # ── Save report ───────────────────────────────────────────────────────────
    file_ts = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    _m = re.search(r"_Test_(\d+)_", base_name)
    test_num_str = _m.group(1).zfill(2) if _m else "00"

    output_dir = os.path.join(DATA_DIR, f"Plots_For_Tests_{test_num_str}")
    os.makedirs(output_dir, exist_ok=True)

    report_path = os.path.join(output_dir, f"Data_Analysis_For_Test_{test_num_str}_{file_ts}.txt")
    with open(report_path, "w") as f:
        f.write("\n".join(lines) + "\n")
    print(f"  Saved: {os.path.relpath(report_path, DATA_DIR)}")


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────
if len(sys.argv) > 1:
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
    print("[ERROR] No CSV files found. Pass a test number, e.g.:  python3 Stand_To_Walk_Data_Analysis.py 1")
    sys.exit(1)

print(f"\nFound {len(csv_files)} CSV file(s) to process.")
for csv_path in csv_files:
    process_file(csv_path)

print("\nDone.")