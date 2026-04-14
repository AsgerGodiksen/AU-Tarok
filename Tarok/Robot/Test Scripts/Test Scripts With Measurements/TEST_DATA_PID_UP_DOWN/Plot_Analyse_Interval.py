"""
PID Interval Analyser — Up/Down
--------------------------------
Usage:
    python3 Plot_Analyse_Interval.py <test_number>
    python3 Plot_Analyse_Interval.py          # prompts for test number

For the chosen test the script will:
  - Ask for a start and end time (seconds)
  - Plot the interval: 2x2 leg torques + trajectory index strip
  - Save the PNG next to the CSV files
  - Save a txt report with PID-relevant metrics
"""

import os
import sys
import glob
import numpy as np
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
from scipy.signal import find_peaks

matplotlib.use("Agg")

DATA_DIR = os.path.dirname(os.path.abspath(__file__))

LEGS   = ["FL", "FR", "HL", "HR"]
JOINTS = ["J1", "J2", "J3"]
COLORS = ["tab:blue", "tab:orange", "tab:green"]

SETTLE_BAND_NM = 0.3   # ±Nm around steady-state to count as "settled"


# ──────────────────────────────────────────────────────────────────────────────
# Metrics
# ──────────────────────────────────────────────────────────────────────────────

def dominant_frequency(time, values):
    """Return the dominant oscillation frequency (Hz) via FFT, ignoring DC."""
    n  = len(values)
    if n < 4:
        return float("nan")
    dt   = np.mean(np.diff(time))
    fft  = np.abs(np.fft.rfft(values - values.mean()))
    freq = np.fft.rfftfreq(n, d=dt)
    fft[0] = 0
    dominant = freq[np.argmax(fft)]
    return dominant


def settling_time(time, values, band=SETTLE_BAND_NM):
    """
    Time from the start of the interval until the signal stays within
    ±band Nm of its steady-state value (mean of last 20 % of the interval).
    Returns None if it never settles.
    """
    steady = np.mean(values[int(0.8 * len(values)):])
    settled_indices = np.where(np.abs(values - steady) <= band)[0]
    if len(settled_indices) == 0:
        return None
    outside = np.where(np.abs(values - steady) > band)[0]
    if len(outside) == 0:
        return time.iloc[0] - time.iloc[0]
    last_outside = outside[-1]
    if last_outside + 1 >= len(time):
        return None
    return float(time.iloc[last_outside + 1] - time.iloc[0])


def overshoot_percent(values):
    """
    Signed overshoot relative to the steady-state value (last 20 %).
    Positive = overshot above steady-state, negative = undershot.
    """
    steady = np.mean(values[int(0.8 * len(values)):])
    if abs(steady) < 1e-6:
        return float("nan")
    peak_pos = values.max()
    peak_neg = values.min()
    if abs(peak_pos - steady) >= abs(peak_neg - steady):
        return 100.0 * (peak_pos - steady) / abs(steady)
    else:
        return 100.0 * (peak_neg - steady) / abs(steady)


def analyse_joint(time, values):
    """Return a dict of all PID-relevant metrics for one joint signal."""
    ss_value = float(np.mean(values[int(0.8 * len(values)):]))
    error    = values - ss_value

    peaks_hi, _ = find_peaks( values, prominence=0.05)
    peaks_lo, _ = find_peaks(-values, prominence=0.05)

    all_peaks = np.sort(np.concatenate([peaks_hi, peaks_lo]))
    if len(all_peaks) >= 2:
        intervals       = np.diff(time.iloc[all_peaks])
        avg_half_period = np.mean(intervals)
        freq_peaks      = 1.0 / (2 * avg_half_period) if avg_half_period > 0 else float("nan")
    else:
        freq_peaks = float("nan")

    freq_fft = dominant_frequency(time.values, values.values)

    return {
        "mean_ss"    : ss_value,
        "peak_pos"   : float(values.max()),
        "peak_neg"   : float(values.min()),
        "pk2pk"      : float(values.max() - values.min()),
        "rms"        : float(np.sqrt(np.mean(values**2))),
        "std"        : float(values.std()),
        "rmse_ss"    : float(np.sqrt(np.mean(error**2))),
        "freq_fft"   : freq_fft,
        "freq_peaks" : freq_peaks,
        "settle_s"   : settling_time(time, values),
        "overshoot"  : overshoot_percent(values),
        "n_peaks"    : len(peaks_hi) + len(peaks_lo),
    }


# ──────────────────────────────────────────────────────────────────────────────
# Plot
# ──────────────────────────────────────────────────────────────────────────────

def plot_interval(df, t_start, t_end, output_name):
    mask = (df["Timestamp (s)"] >= t_start) & (df["Timestamp (s)"] <= t_end)
    sub  = df[mask].copy()
    time = sub["Timestamp (s)"]

    fig, axes = plt.subplots(2, 2, figsize=(14, 8), sharex=True)
    fig.suptitle(
        f"{output_name.replace('_', ' ')}  [{t_start:.2f} s – {t_end:.2f} s]",
        fontsize=13, fontweight="bold"
    )

    leg_axes = {"FL": axes[0, 0], "FR": axes[0, 1], "HL": axes[1, 0], "HR": axes[1, 1]}

    for leg, ax in leg_axes.items():
        for joint, color in zip(JOINTS, COLORS):
            col = f"{leg}_{joint}"
            if col in sub.columns:
                ax.plot(time, sub[col], label=joint, color=color, linewidth=1.2)
        ax.axhline(0, color="black", linewidth=0.6, linestyle="--", alpha=0.4)
        ax.set_title(f"{leg} Leg", fontsize=11)
        ax.set_ylabel("Torque (Nm)")
        ax.set_ylim(-6, 6)
        ax.legend(loc="upper right", fontsize=8)
        ax.grid(True, alpha=0.3)

    axes[1, 0].set_xlabel("Time (s)")
    axes[1, 1].set_xlabel("Time (s)")

    plt.tight_layout()
    out_path = os.path.join(DATA_DIR, f"{output_name}.png")
    plt.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f"\nPlot saved → {out_path}\n")


# ──────────────────────────────────────────────────────────────────────────────
# Report → txt file
# ──────────────────────────────────────────────────────────────────────────────

def save_report(df, t_start, t_end, output_name):
    mask = (df["Timestamp (s)"] >= t_start) & (df["Timestamp (s)"] <= t_end)
    sub  = df[mask].copy()
    time = sub["Timestamp (s)"].reset_index(drop=True)

    lines = []
    w = lines.append

    interval_s = t_end - t_start
    w("=" * 64)
    w(f"  PID ANALYSIS   interval: {t_start:.3f} s -> {t_end:.3f} s"
      f"  ({interval_s:.3f} s)   samples: {len(sub)}")
    w("=" * 64)

    for leg in LEGS:
        w("")
        w("  " + "-" * 56)
        w(f"  LEG: {leg}")
        w("  " + "-" * 56)
        w(f"  {'Joint':<8} {'Steady(Nm)':>10} {'Pk-Pk(Nm)':>10}"
          f" {'RMSE(Nm)':>9} {'Std(Nm)':>8}"
          f" {'FreqFFT(Hz)':>11} {'FreqPk(Hz)':>10}"
          f" {'Settle(s)':>9} {'Overshoot%':>11}")
        w(f"  {'-'*8} {'-'*10} {'-'*10} {'-'*9} {'-'*8}"
          f" {'-'*11} {'-'*10} {'-'*9} {'-'*11}")

        for joint in JOINTS:
            col = f"{leg}_{joint}"
            if col not in sub.columns:
                continue
            values = sub[col].reset_index(drop=True)
            m = analyse_joint(time, values)

            settle = f"{m['settle_s']:.3f}" if m["settle_s"] is not None else "N/A"
            ovs    = f"{m['overshoot']:+.1f}" if not np.isnan(m["overshoot"]) else "N/A"
            ffft   = f"{m['freq_fft']:.2f}"  if not np.isnan(m["freq_fft"])  else "N/A"
            fpk    = f"{m['freq_peaks']:.2f}" if not np.isnan(m["freq_peaks"]) else "N/A"

            w(f"  {joint:<8} {m['mean_ss']:>10.3f} {m['pk2pk']:>10.3f}"
              f" {m['rmse_ss']:>9.3f} {m['std']:>8.3f}"
              f" {ffft:>11} {fpk:>10}"
              f" {settle:>9} {ovs:>11}")

    w("")
    w("=" * 64)
    w("  HOW TO READ THESE NUMBERS")
    w("=" * 64)
    w("")
    w("  STEADY (Nm)  — The average torque the motor holds at the end of the")
    w("    interval. This is the 'resting load' once things have calmed down.")
    w("    A value close to zero means the joint is nearly unloaded.")
    w("    A large value means the motor is working hard just to hold position.")
    w("")
    w("  PK-PK (Nm)  — The total swing from the lowest to the highest torque")
    w("    value seen. Think of it as 'how wild was the signal?'")
    w("    Small Pk-Pk = the joint is calm and stable.")
    w("    Large Pk-Pk = the joint is bouncing around a lot.")
    w("    Goal: get this as small as possible without slowing the response.")
    w("")
    w("  RMSE (Nm)  — On average, how far was the torque from its steady value?")
    w("    Lower is better. This is your main PID performance score —")
    w("    if two tests have the same Pk-Pk, the one with lower RMSE")
    w("    spent less time far from the target.")
    w("")
    w("  STD (Nm)  — How much the torque 'jitters' around its average.")
    w("    A low Std but high RMSE means the signal is smooth but offset.")
    w("    A high Std means the signal is noisy or still oscillating.")
    w("    Compare Std between tests with the same PID gains to judge noise.")
    w("")
    w("  FREQFFT (Hz)  — The dominant oscillation frequency found by")
    w("    mathematically decomposing the whole signal (FFT method).")
    w("    FreqPk finds the same thing by measuring the time between peaks.")
    w("    If both agree -> the signal has a clear, repeating oscillation.")
    w("    If they disagree -> the oscillation is irregular or fading out.")
    w("    Higher frequency = faster wobbling. A PID that is too aggressive")
    w("    (high P or I) will push this frequency up.")
    w("")
    w(f"  SETTLE (s)  — How many seconds passed before the torque stayed")
    w(f"    within +-{SETTLE_BAND_NM} Nm of its final value and did not leave again.")
    w("    This is how long the robot took to 'settle in' to its pose.")
    w("    Shorter = better. A PID that never settles will show 'N/A'.")
    w("    Tip: if Settle is long but Pk-Pk is small, the robot is slow but")
    w("    stable — try increasing P a little.")
    w("")
    w("  OVERSHOOT (%)  — How far past the final resting torque did the signal")
    w("    fly before settling? Expressed as a percentage of the steady value.")
    w("    +120 % means the torque peaked at more than double its final value.")
    w("    Negative sign means it undershot (went the wrong way first).")
    w("    High overshoot = too much P or too little D.")
    w("    Near zero overshoot with long Settle = too much D (overdamped).")
    w("")
    w("  QUICK TUNING GUIDE")
    w("  ------------------")
    w("  Oscillating / high Pk-Pk and Std  -> reduce P or increase D")
    w("  Slow to settle / low overshoot    -> increase P or reduce D")
    w("  Steady-state offset (Steady != 0) -> increase I")
    w("  Noisy but stable                  -> reduce P slightly, check D filter")
    w("  FreqFFT and FreqPk very different -> oscillation is irregular,")
    w("                                       focus on Settle and RMSE instead")
    w("=" * 64)

    txt_path = os.path.join(DATA_DIR, f"{output_name}.txt")
    with open(txt_path, "w") as f:
        f.write("\n".join(lines) + "\n")
    print(f"Report saved → {txt_path}")


# ──────────────────────────────────────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────────────────────────────────────

if len(sys.argv) > 1:
    test_num = int(sys.argv[1])
else:
    test_num = int(input("Enter test number: "))

pattern = os.path.join(DATA_DIR, f"Up_Down_Torque_Log_Test_{test_num}_*.csv")
matches = sorted(glob.glob(pattern))

if not matches:
    print(f"No CSV files found for Test_{test_num}.")
    sys.exit(1)

# Pick file (handle a/b)
if len(matches) == 1:
    csv_path  = matches[0]
    base_name = f"Up_Down_Torque_log_Test_{test_num:02d}"
else:
    print("Multiple files found for this test number:")
    for i, p in enumerate(matches):
        suffix = chr(ord("a") + i)
        print(f"  {suffix}) {os.path.basename(p)}")
    choice    = input("Choose (a/b/...): ").strip().lower()
    idx       = ord(choice) - ord("a")
    csv_path  = matches[idx]
    base_name = f"Up_Down_Torque_log_Test_{test_num:02d}{choice}"

df    = pd.read_csv(csv_path)
t_min = df["Timestamp (s)"].iloc[0]
t_max = df["Timestamp (s)"].iloc[-1]
print(f"\nData range: {t_min:.3f} s -> {t_max:.3f} s  ({t_max - t_min:.3f} s total)")

t_start = float(input(f"Start time (s) [{t_min:.3f}]: ").strip() or t_min)
t_end   = float(input(f"End   time (s) [{t_max:.3f}]: ").strip() or t_max)

if t_start >= t_end:
    print("Error: start time must be before end time.")
    sys.exit(1)

interval_tag = f"{t_start:.2f}-{t_end:.2f}s"
output_name  = f"{base_name}_interval_{interval_tag}"

plot_interval(df, t_start, t_end, output_name)
save_report(df, t_start, t_end, output_name)