import os
import glob
import sys
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib

matplotlib.rcParams["font.family"] = "Liberation Serif"

DATA_DIR = os.path.dirname(os.path.abspath(__file__))

LEGS   = ["FL", "FR", "HL", "HR"]
JOINTS       = ["J1", "J2", "J3"]
JOINT_LABELS = ["θ1", "θ2", "θ3"]
COLORS = ["tab:blue", "tab:orange", "tab:green"]

LABEL_SIZE  = 17
TICK_SIZE   = 13
TITLE_SIZE  = 13
LEGEND_SIZE = 13

# --- Get test number ---
if len(sys.argv) > 1:
    test_num = int(sys.argv[1])
else:
    test_num = int(input("Enter test number to plot: "))

pattern = os.path.join(DATA_DIR, f"Up_Down_TorquePos_Log_Test_{test_num}_*.csv")
matches = sorted(glob.glob(pattern))

if not matches:
    print(f"No CSV files found for Test_{test_num}.")
    sys.exit(1)

if len(matches) == 1:
    csv_path = matches[0]
else:
    print("Multiple files found:")
    for i, p in enumerate(matches):
        suffix = chr(ord("a") + i)
        print(f"  {suffix}) {os.path.basename(p)}")
    choice  = input("Choose (a/b/...): ").strip().lower()
    csv_path = matches[ord(choice) - ord("a")]

df   = pd.read_csv(csv_path).iloc[::10]
time = df["Timestamp (s)"]
t_min = time.iloc[0]
t_max = time.iloc[-1]
title_base = os.path.splitext(os.path.basename(csv_path))[0].replace("_", " ")

# ── Figure 1: Torque ──────────────────────────────────────────────────────────
fig1, axes1 = plt.subplots(2, 2, figsize=(14, 8), sharex=True)

leg_axes1 = {"FL": axes1[0, 0], "FR": axes1[0, 1],
             "HL": axes1[1, 0], "HR": axes1[1, 1]}

for leg, ax in leg_axes1.items():
    for joint, lbl, color in zip(JOINTS, JOINT_LABELS, COLORS):
        col = f"{leg}_{joint}_Torque"
        if col in df.columns:
            ax.plot(time, df[col], label=lbl, color=color, linewidth=1.2)
    ax.set_title(f"{leg} Leg", fontsize=TITLE_SIZE, fontweight="bold")
    ax.set_xlim(t_min, t_max)
    ax.set_ylim(-11, 11)
    ax.grid(True, alpha=0.3)
    ax.tick_params(labelsize=TICK_SIZE)

for ax in [axes1[0, 1], axes1[1, 1]]:
    ax.tick_params(labelleft=False)

axes1[0, 0].set_ylabel("Torque (Nm)", fontsize=LABEL_SIZE)
axes1[1, 0].set_ylabel("Torque (Nm)", fontsize=LABEL_SIZE)
axes1[1, 0].set_xlabel("Time (s)", fontsize=LABEL_SIZE)
axes1[1, 1].set_xlabel("Time (s)", fontsize=LABEL_SIZE)

handles1, labels1 = axes1[0, 0].get_legend_handles_labels()
fig1.legend(handles1, labels1, loc="lower center", ncol=len(JOINTS),
            fontsize=LEGEND_SIZE, bbox_to_anchor=(0.5, 0.0))
fig1.tight_layout()
fig1.subplots_adjust(bottom=0.08)

# ── Figure 2: Position ────────────────────────────────────────────────────────
fig2, axes2 = plt.subplots(2, 2, figsize=(14, 8), sharex=True)
#fig2.suptitle(f"Position — {title_base}", fontsize=13, fontweight="bold")

leg_axes2 = {"FL": axes2[0, 0], "FR": axes2[0, 1],
             "HL": axes2[1, 0], "HR": axes2[1, 1]}

for leg, ax in leg_axes2.items():
    for joint, color in zip(JOINTS, COLORS):
        col = f"{leg}_{joint}_Pos (deg)"
        if col in df.columns:
            ax.plot(time, df[col], label=joint, color=color, linewidth=1.2)
    ax.set_title(f"{leg} Leg", fontsize=11)
    ax.set_ylabel("Position (deg)")
    ax.set_xlim(t_min, t_max)
    ax.set_ylim(-100, 100)
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.3)

axes2[1, 0].set_xlabel("Time (s)")
axes2[1, 1].set_xlabel("Time (s)")
fig2.tight_layout()

base_name = os.path.splitext(os.path.basename(csv_path))[0]
fig1.savefig(os.path.join(DATA_DIR, f"{base_name}_Torque.png"), dpi=150, bbox_inches="tight")
fig2.savefig(os.path.join(DATA_DIR, f"{base_name}_Position.png"), dpi=150, bbox_inches="tight")
print(f"Saved: {base_name}_Torque.png and {base_name}_Position.png")

plt.show()