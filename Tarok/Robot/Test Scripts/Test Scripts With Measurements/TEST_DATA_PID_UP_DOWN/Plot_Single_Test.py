import os
import glob
import re
import sys
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
matplotlib.use("Agg")

DATA_DIR = os.path.dirname(os.path.abspath(__file__))

LEGS   = ["FL", "FR", "HL", "HR"]
JOINTS = ["J1", "J2", "J3"]
COLORS = ["tab:blue", "tab:orange", "tab:green"]


def plot_test(csv_path, output_name):
    df   = pd.read_csv(csv_path).iloc[::10]
    time = df["Timestamp (s)"]

    fig, axes = plt.subplots(2, 2, figsize=(14, 8), sharex=True)
    fig.suptitle(output_name.replace("_", " "), fontsize=14, fontweight="bold")

    leg_axes = {"FL": axes[0, 0], "FR": axes[0, 1], "HL": axes[1, 0], "HR": axes[1, 1]}

    for leg, ax in leg_axes.items():
        for joint, color in zip(JOINTS, COLORS):
            col = f"{leg}_{joint}"
            if col in df.columns:
                ax.plot(time, df[col], label=joint, color=color, linewidth=1.2)
        ax.set_title(f"{leg} Leg", fontsize=11)
        ax.set_ylabel("Torque (Nm)")
        ax.set_ylim(-10, 10)
        ax.legend(loc="upper right", fontsize=8)
        ax.grid(True, alpha=0.3)

    axes[1, 0].set_xlabel("Time (s)")
    axes[1, 1].set_xlabel("Time (s)")

    plt.tight_layout()
    out_path = os.path.join(DATA_DIR, f"{output_name}.png")
    plt.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f"Saved: {out_path}")


# --- Get test number from argument or prompt ---
if len(sys.argv) > 1:
    test_num = int(sys.argv[1])
else:
    test_num = int(input("Enter test number to plot: "))

# Find matching files
pattern = os.path.join(DATA_DIR, f"Up_Down_Torque_Log_Test_{test_num}_*.csv")
matches = sorted(glob.glob(pattern))

if not matches:
    print(f"No CSV files found for Test_{test_num}.")
    sys.exit(1)

if len(matches) == 1:
    name = f"Up_Down_Torque_log_Test_{test_num:02d}"
    plot_test(matches[0], name)
else:
    for idx, path in enumerate(matches):
        suffix = chr(ord("a") + idx)
        name = f"Up_Down_Torque_log_Test_{test_num:02d}{suffix}"
        plot_test(path, name)

print("Done.")