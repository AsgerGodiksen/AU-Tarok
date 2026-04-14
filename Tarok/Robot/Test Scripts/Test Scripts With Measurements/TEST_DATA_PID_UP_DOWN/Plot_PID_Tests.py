import os
import glob
import re
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
matplotlib.use("Agg")

DATA_DIR = os.path.dirname(os.path.abspath(__file__))

# --- Collect and group CSV files by test number ---
csv_files = glob.glob(os.path.join(DATA_DIR, "Up_Down_Torque_Log_Test_*.csv"))
csv_files.sort()  # Sort by filename (timestamp embedded ensures correct a/b order)

groups = {}  # test_number (int) -> list of sorted file paths
for path in csv_files:
    m = re.search(r"Test_(\d+)_", os.path.basename(path))
    if m:
        num = int(m.group(1))
        groups.setdefault(num, []).append(path)

LEGS   = ["FL", "FR", "HL", "HR"]
JOINTS = ["J1", "J2", "J3"]
COLORS = ["tab:blue", "tab:orange", "tab:green"]


def plot_test(csv_path, output_name):
    df   = pd.read_csv(csv_path)
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
        ax.set_ylim(-6, 6)
        ax.legend(loc="upper right", fontsize=8)
        ax.grid(True, alpha=0.3)

    axes[1, 0].set_xlabel("Time (s)")
    axes[1, 1].set_xlabel("Time (s)")

    plt.tight_layout()
    out_path = os.path.join(DATA_DIR, f"{output_name}.png")
    plt.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f"Saved: {out_path}")


# --- Generate plots ---
for test_num in sorted(groups.keys()):
    files = groups[test_num]
    if len(files) == 1:
        name = f"Up_Down_Torque_log_Test_{test_num:02d}"
        plot_test(files[0], name)
    else:
        # Multiple files for same test number → label a, b, c...
        for idx, path in enumerate(files):
            suffix = chr(ord("a") + idx)
            name = f"Up_Down_Torque_log_Test_{test_num:02d}{suffix}"
            plot_test(path, name)

print("Done.")