import os
import glob
import sys
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib

matplotlib.use("Agg")
matplotlib.rcParams["font.family"] = "Liberation Serif"

DATA_DIR = os.path.dirname(os.path.abspath(__file__))

LEGS = ["FL", "FR", "HL", "HR"]
JOINTS       = ["J1", "J2", "J3"]
JOINT_LABELS = ["θ1", "θ2", "θ3"]
COLORS = ["tab:blue", "tab:orange", "tab:green"]

LABEL_SIZE = 17
TICK_SIZE = 13
TITLE_SIZE = 13
LEGEND_SIZE = 13


def plot_test(csv_path, output_name):
    df = pd.read_csv(csv_path)
    time = df["Timestamp (s)"]

    # --- Torque figure ---
    fig_t, axes_t = plt.subplots(2, 2, figsize=(14, 8), sharex=True)

    leg_axes = {"FL": axes_t[0, 0], "FR": axes_t[0, 1], "HL": axes_t[1, 0], "HR": axes_t[1, 1]}

    for leg, ax in leg_axes.items():
        for joint, lbl, color in zip(JOINTS, JOINT_LABELS, COLORS):
            col = f"{leg}_{joint}_Torque"
            if col in df.columns:
                ax.plot(time, df[col], label=lbl, color=color, linewidth=1.2)
        ax.set_title(f"{leg} Leg", fontsize=TITLE_SIZE, fontweight="bold")
        ax.set_ylim(-8, 8)
        ax.grid(True, alpha=0.3)
        ax.tick_params(labelsize=TICK_SIZE)

    for ax in [axes_t[0, 1], axes_t[1, 1]]:
        ax.tick_params(labelleft=False)

    axes_t[1, 0].set_xlabel("Time (s)", fontsize=LABEL_SIZE)
    axes_t[1, 1].set_xlabel("Time (s)", fontsize=LABEL_SIZE)
    axes_t[0, 0].set_ylabel("Torque (Nm)", fontsize=LABEL_SIZE)
    axes_t[1, 0].set_ylabel("Torque (Nm)", fontsize=LABEL_SIZE)

    handles, labels = axes_t[0, 0].get_legend_handles_labels()
    fig_t.legend(handles, labels, loc="lower center", ncol=len(JOINTS), fontsize=LEGEND_SIZE, bbox_to_anchor=(0.5, 0.0))

    plt.tight_layout()
    fig_t.subplots_adjust(bottom=0.08)
    out_t = os.path.join(DATA_DIR, f"{output_name}_Torque.png")
    fig_t.savefig(out_t, dpi=150)
    plt.close(fig_t)
    print(f"Saved: {out_t}")

    # --- Position figure (actual vs commanded) ---
    fig_p, axes_p = plt.subplots(2, 2, figsize=(14, 8), sharex=True)

    leg_axes_p = {"FL": axes_p[0, 0], "FR": axes_p[0, 1], "HL": axes_p[1, 0], "HR": axes_p[1, 1]}

    for leg, ax in leg_axes_p.items():
        for joint, lbl, color in zip(JOINTS, JOINT_LABELS, COLORS):
            col_pos = f"{leg}_{joint}_Pos (deg)"
            col_cmd = f"{leg}_{joint}_Cmd (deg)"
            if col_pos in df.columns:
                ax.plot(time, df[col_pos], label=f"{lbl} actual", color=color, linewidth=1.2)
            if col_cmd in df.columns:
                ax.plot(time, df[col_cmd], label=f"{lbl} cmd", color=color, linewidth=1.2, linestyle="--", alpha=0.6)
        ax.set_title(f"{leg} Leg", fontsize=TITLE_SIZE, fontweight="bold")
        ax.set_ylabel("Position (deg)", fontsize=LABEL_SIZE)
        ax.grid(True, alpha=0.3)
        ax.tick_params(labelsize=TICK_SIZE)

    for ax in [axes_p[0, 1], axes_p[1, 1]]:
        ax.tick_params(labelleft=False)
        ax.set_ylabel("")

    axes_p[1, 0].set_xlabel("Time (s)", fontsize=LABEL_SIZE)
    axes_p[1, 1].set_xlabel("Time (s)", fontsize=LABEL_SIZE)

    handles, labels = axes_p[0, 0].get_legend_handles_labels()
    fig_p.legend(handles, labels, loc="lower center", ncol=len(JOINTS) * 2, fontsize=LEGEND_SIZE, bbox_to_anchor=(0.5, 0.0))

    plt.tight_layout()
    fig_p.subplots_adjust(bottom=0.08)
    out_p = os.path.join(DATA_DIR, f"{output_name}_Position.png")
    fig_p.savefig(out_p, dpi=150)
    plt.close(fig_p)
    print(f"Saved: {out_p}")


# --- Get test number from argument or prompt ---
if len(sys.argv) > 1:
    test_num = int(sys.argv[1])
else:
    test_num = int(input("Enter test number to plot: "))

pattern = os.path.join(DATA_DIR, f"Stand_Pose_TorquePos_Log_TEST_{test_num}_*.csv")
matches = sorted(glob.glob(pattern))

if not matches:
    print(f"No CSV files found for TEST_{test_num}.")
    sys.exit(1)

if len(matches) == 1:
    name = f"Stand_Pose_TorquePos_Log_Test_{test_num:02d}"
    plot_test(matches[0], name)
else:
    for idx, path in enumerate(matches):
        suffix = chr(ord("a") + idx)
        name = f"Stand_Pose_TorquePos_Log_Test_{test_num:02d}{suffix}"
        plot_test(path, name)

print("Done.")
