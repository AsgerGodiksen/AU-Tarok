# Script for running up/down motion and plotting used for validation section of kinematics chapter in masters thesis

# Imports
import sys
import os
import re
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import matplotlib.lines as mlines
from Robot.Kinematics.Inverse_Kinematics import*
from Robot.Kinematics.Forward_Kinematics import*
from Robot.Kinematics.Constant_Transforms import*
from Robot.Kinematics.Jacobian import*
from Robot.Kinematics.Interpolation import*

def save_figure_from_title(title, folder="."):
    # Saves the current Matplotlib figure as PNG using the title as filename.
    
    # Sanitize filename (replace spaces and forbidden characters)
    safe_name = re.sub(r'[^a-zA-Z0-9]+', '_', title)
    
    png_path = os.path.join(folder, f"{safe_name}.png")

    plt.savefig(png_path, format="png", dpi=600, bbox_inches="tight")

    print(f"Saved: {png_path}")

# Save plots toggle
save_plots = False

################## MAIN SCRIPT ####################
### Trajectory generation ###
# Define kinematic body lengths
l_k = 0.7048  # Length of body in kinematic model (meters)
w_k = 0.220   # Width of body in kinematic model (meters)

# Define time series
dt = 0.005 # seconds (100 Hz)
total_time = 4  # Total time in seconds
num_time_steps = int(total_time / dt) + 1
t = np.linspace(0, total_time, num_time_steps)

# Segment boundaries for the trajectory: 0->2s, 3->4s
conditions = [(t >= 0)   & (t < 2),   # Up:    -0.36 -> -0.46
              (t >= 2)   & (t < 4),   # Down:  -0.46 -> -0.36
              (t >= 4)]               # Hold:  -0.36 (hold at -0.36 after 4s))

# Define desired end-effector trajectory as function of time for all 4 legs (in body frame)
x_FL = x_FR = (l_k/2)*np.ones_like(t)  # X position in meters (constant)
x_HL = x_HR = (-l_k/2)*np.ones_like(t)  # X position in meters (constant)
y_FL = y_HL = (w_k/2 + 0.078)*np.ones_like(t)  # Y position in meters (constant)
y_FR = y_HR = (-w_k/2 - 0.078)*np.ones_like(t)  # Y position in meters (constant)
z = np.piecewise(t, conditions, [lambda t: cos_interp(t, -0.36, -0.46, 0, 2),
                                 lambda t: cos_interp(t, -0.46, -0.36, 2, 4),
                                 lambda t: -0.36*np.ones_like(t)])  # Z position in meters (cosine wave from -0.36 to -0.46, then to -0.36, then hold at -0.36)

# Define desired end effector velocity (foot velocity) as functions of time for all 4 legs (in body frame) - Note, it is the same for all legs in body frame for this trajectory
x_dot = np.zeros_like(t)  # X velocity in meters/second (constant)
y_dot = np.zeros_like(t)  # Y velocity in meters/second (constant)
z_dot = np.piecewise(t, conditions, [lambda t: cos_interp_dot(t, -0.36, -0.46, 0, 2),
                                     lambda t: cos_interp_dot(t, -0.46, -0.36, 2, 4),
                                     lambda t: np.zeros_like(t)])

### Transformations ###
# Combine trajectories into position arrays for each leg
P_FL_body = np.vstack((x_FL, y_FL, z))
P_FR_body = np.vstack((x_FR, y_FR, z))
P_HL_body = np.vstack((x_HL, y_HL, z))
P_HR_body = np.vstack((x_HR, y_HR, z)) 

# Transform desired end-effector trajectory from body frame to leg base frames
P_FL_base = np.array([T0_B(P_FL_body[:, i].reshape((3, 1)), 'FL') for i in range(len(t))])
P_FR_base = np.array([T0_B(P_FR_body[:, i].reshape((3, 1)), 'FR') for i in range(len(t))])
P_HL_base = np.array([T0_B(P_HL_body[:, i].reshape((3, 1)), 'HL') for i in range(len(t))])
P_HR_base = np.array([T0_B(P_HR_body[:, i].reshape((3, 1)), 'HR') for i in range(len(t))])

# Combine body frame trajectory cartesian velocities into array
V_body = np.vstack((x_dot, y_dot, z_dot))

# Transform desired end-effector velocity from body frame to leg base frames
V_FL_base = np.array([R0_B(V_body[:, i].reshape((3, 1)), 'FL') for i in range(len(t))])
V_FR_base = np.array([R0_B(V_body[:, i].reshape((3, 1)), 'FR') for i in range(len(t))])
V_HL_base = np.array([R0_B(V_body[:, i].reshape((3, 1)), 'HL') for i in range(len(t))])
V_HR_base = np.array([R0_B(V_body[:, i].reshape((3, 1)), 'HR') for i in range(len(t))])

### Kinematics ###
# Determine joint angles for all 4 legs using inverse kinematics
Theta_FL = np.array([Inverse_Kinematics(P_FL_base[i], 'FL') for i in range(len(t))]) # Shape (num_time_steps, 3), containing theta1, theta2, theta3 for each time step
Theta_FR = np.array([Inverse_Kinematics(P_FR_base[i], 'FR') for i in range(len(t))]) # Shape (num_time_steps, 3), containing theta1, theta2, theta3 for each time step
Theta_HL = np.array([Inverse_Kinematics(P_HL_base[i], 'HL') for i in range(len(t))]) # Shape (num_time_steps, 3), containing theta1, theta2, theta3 for each time step
Theta_HR = np.array([Inverse_Kinematics(P_HR_base[i], 'HR') for i in range(len(t))]) # Shape (num_time_steps, 3), containing theta1, theta2, theta3 for each time step

# Determine joint velocities for all 4 legs using Jacobian
# Damped least squares inverse to avoid singularities - theta_dot = (J^T*J + damp^2*I)^-1 * J^T * cartesian_velocity 
Theta_dot_FL = np.zeros((3, len(t)))  # Initialize joint velocity array
Theta_dot_FR = np.zeros((3, len(t)))  # Initialize joint velocity array
Theta_dot_HL = np.zeros((3, len(t)))  # Initialize joint velocity array
Theta_dot_HR = np.zeros((3, len(t)))  # Initialize joint velocity array
damp = 0.001  # Damping factor
for i in range(len(t)):
    Jac_i_FL = Jacobian(Theta_FL[i, 0], Theta_FL[i, 1], Theta_FL[i, 2], 'FL')
    Jac_i_FR = Jacobian(Theta_FR[i, 0], Theta_FR[i, 1], Theta_FR[i, 2], 'FR')
    Jac_i_HL = Jacobian(Theta_HL[i, 0], Theta_HL[i, 1], Theta_HL[i, 2], 'HL')
    Jac_i_HR = Jacobian(Theta_HR[i, 0], Theta_HR[i, 1], Theta_HR[i, 2], 'HR')
    JT_FL = Jac_i_FL.T
    JT_FR = Jac_i_FR.T
    JT_HL = Jac_i_HL.T
    JT_HR = Jac_i_HR.T
    term_FL = JT_FL @ Jac_i_FL + (damp**2)*np.eye(3)
    term_FR = JT_FR @ Jac_i_FR + (damp**2)*np.eye(3)
    term_HL = JT_HL @ Jac_i_HL + (damp**2)*np.eye(3)
    term_HR = JT_HR @ Jac_i_HR + (damp**2)*np.eye(3)
    Theta_dot_FL[:, i] = np.linalg.solve(term_FL, JT_FL @ V_FL_base[i].flatten())
    Theta_dot_FR[:, i] = np.linalg.solve(term_FR, JT_FR @ V_FR_base[i].flatten())
    Theta_dot_HL[:, i] = np.linalg.solve(term_HL, JT_HL @ V_HL_base[i].flatten())
    Theta_dot_HR[:, i] = np.linalg.solve(term_HR, JT_HR @ V_HR_base[i].flatten())

# Map joint angles back to cartesian space using forward kinematics for validation
P_FL_base_FK = np.array([P0_end(Theta_FL[i,0], Theta_FL[i,1], Theta_FL[i,2], 'FL') for i in range(len(t))]) # Shape (num_time_steps, 3, 1), containing position of end effector in base frame for each time step
P_FR_base_FK = np.array([P0_end(Theta_FR[i,0], Theta_FR[i,1], Theta_FR[i,2], 'FR') for i in range(len(t))]) # Shape (num_time_steps, 3, 1), containing position of end effector in base frame for each time step
P_HL_base_FK = np.array([P0_end(Theta_HL[i,0], Theta_HL[i,1], Theta_HL[i,2], 'HL') for i in range(len(t))]) # Shape (num_time_steps, 3, 1), containing position of end effector in base frame for each time step
P_HR_base_FK = np.array([P0_end(Theta_HR[i,0], Theta_HR[i,1], Theta_HR[i,2], 'HR') for i in range(len(t))]) # Shape (num_time_steps, 3, 1), containing position of end effector in base frame for each time step

# Convert joint angles and velocities to degrees and degrees/s 
Theta_FL = np.rad2deg(Theta_FL)
Theta_FR = np.rad2deg(Theta_FR)
Theta_HL = np.rad2deg(Theta_HL)
Theta_HR = np.rad2deg(Theta_HR)
Theta_dot_FL = np.rad2deg(Theta_dot_FL)
Theta_dot_FR = np.rad2deg(Theta_dot_FR)
Theta_dot_HL = np.rad2deg(Theta_dot_HL)
Theta_dot_HR = np.rad2deg(Theta_dot_HR)



### Plotting ###
# ─── Formatting constants ────────────────────────────────────────────────────
matplotlib.rcParams["font.family"] = "serif"

LEGS         = ["FL", "FR", "HL", "HR"]
LEG_POS      = {"FL": (0, 0), "FR": (0, 1), "HL": (1, 0), "HR": (1, 1)}
COLORS       = ["tab:blue", "tab:orange", "tab:green"]
JOINT_LABELS     = [r"$\theta_1$",       r"$\theta_2$",       r"$\theta_3$"]
JOINT_VEL_LABELS = [r"$\dot{\theta}_1$", r"$\dot{\theta}_2$", r"$\dot{\theta}_3$"]
CART_LABELS      = [r"$X$",              r"$Y$",              r"$Z$"]
CART_VEL_LABELS  = [r"$\dot{X}$",        r"$\dot{Y}$",        r"$\dot{Z}$"]

LABEL_SIZE  = 17
TICK_SIZE   = 13
TITLE_SIZE  = 13
LEGEND_SIZE = 13

# ─── Legend handles ──────────────────────────────────────────────────────────
joint_handles = [
    mlines.Line2D([], [], color=c, linewidth=2, label=lbl)
    for c, lbl in zip(COLORS, JOINT_LABELS)
]

cart_handles = [
    mlines.Line2D([], [], color=c, linewidth=2, label=lbl)
    for c, lbl in zip(COLORS, CART_LABELS)
]

joint_vel_handles = [
    mlines.Line2D([], [], color=c, linewidth=2, label=lbl)
    for c, lbl in zip(COLORS, JOINT_VEL_LABELS)
]

cart_vel_handles = [
    mlines.Line2D([], [], color=c, linewidth=2, label=lbl)
    for c, lbl in zip(COLORS, CART_VEL_LABELS)
]

### FIGURES ###
# Figure 1: Cartesian space body frame trajectory of all 4 legs (P_xx_body: x, y, z vs time)   
# ─── Data ────────────────────────────────────────────────────────────────────
P_body_data = {
    "FL": P_FL_body.T,
    "FR": P_FR_body.T,
    "HL": P_HL_body.T,
    "HR": P_HR_body.T,
}

# ─── Figure ──────────────────────────────────────────────────────────────────
fig, axes = plt.subplots(2, 2, figsize=(14, 8), sharex=True)
for leg, (r, c) in LEG_POS.items():
    ax = axes[r, c]
    for j, (jcol, jlbl) in enumerate(zip(COLORS, CART_LABELS)):
        ax.plot(t, P_body_data[leg][:, j], color=jcol, linewidth=1.3)
    ax.set_title(f"{leg} Leg", fontsize=TITLE_SIZE, fontweight="bold")
    ax.set_ylabel("Position (m)", fontsize=LABEL_SIZE)
    ax.set_xlim(t[0], t[-1])
    ax.set_ylim(-0.5, 0.5)
    ax.yaxis.set_major_locator(ticker.MultipleLocator(0.1))
    ax.grid(True, alpha=0.3)
    ax.tick_params(labelsize=TICK_SIZE)

# Right column: suppress redundant y-axis labels
for ax in [axes[0, 1], axes[1, 1]]:
    ax.tick_params(labelleft=False)
    ax.set_ylabel("")

# Bottom row: x-axis labels
axes[1, 0].set_xlabel("Time (s)", fontsize=LABEL_SIZE)
axes[1, 1].set_xlabel("Time (s)", fontsize=LABEL_SIZE)

# Shared legend at bottom
fig.legend(handles=cart_handles, loc="lower center",
           ncol=3, fontsize=LEGEND_SIZE, framealpha=0.9, bbox_to_anchor=(0.5, -0.03))
fig.tight_layout()
fig.subplots_adjust(bottom=0.1)
title = 'Kinematics Validation - Cartesian Space Body Frame Trajectory'
if save_plots:
    save_figure_from_title(title)

# Figure 2: Cartesian space body frame velocity of all 4 legs (V_body: x_dot, y_dot, z_dot vs time) (identical for all 4 but plotted for completeness)
# ─── Data ────────────────────────────────────────────────────────────────────
V_body_data = {
    "FL": V_body.T,
    "FR": V_body.T,
    "HL": V_body.T,
    "HR": V_body.T,
}

# ─── Figure ──────────────────────────────────────────────────────────────────
fig, axes = plt.subplots(2, 2, figsize=(14, 8), sharex=True)
for leg, (r, c) in LEG_POS.items():
    ax = axes[r, c]
    for j, (jcol, jlbl) in enumerate(zip(COLORS, CART_VEL_LABELS)):
        ax.plot(t, V_body_data[leg][:, j], color=jcol, linewidth=1.3)
    ax.set_title(f"{leg} Leg", fontsize=TITLE_SIZE, fontweight="bold")
    ax.set_ylabel("Velocity (m/s)", fontsize=LABEL_SIZE)
    ax.set_xlim(t[0], t[-1])
    ax.set_ylim(-0.1, 0.1)
    ax.yaxis.set_major_locator(ticker.MultipleLocator(0.02))
    ax.grid(True, alpha=0.3)
    ax.tick_params(labelsize=TICK_SIZE)

# Right column: suppress redundant y-axis labels
for ax in [axes[0, 1], axes[1, 1]]:
    ax.tick_params(labelleft=False)
    ax.set_ylabel("")

# Bottom row: x-axis labels
axes[1, 0].set_xlabel("Time (s)", fontsize=LABEL_SIZE)
axes[1, 1].set_xlabel("Time (s)", fontsize=LABEL_SIZE)

# Shared legend at bottom
fig.legend(handles=cart_vel_handles, loc="lower center",
           ncol=3, fontsize=LEGEND_SIZE, framealpha=0.9, bbox_to_anchor=(0.5, -0.03))
fig.tight_layout()
fig.subplots_adjust(bottom=0.1)
title = 'Kinematics Validation - Cartesian Space Body Frame Velocity'
if save_plots:
    save_figure_from_title(title)

# Figure 3: Cartesian space base frame trajectory of all 4 legs (P_xx_base: x, y, z vs time)
# ─── Data ────────────────────────────────────────────────────────────────────
P_base_data = {
    "FL": P_FL_base,
    "FR": P_FR_base,
    "HL": P_HL_base,
    "HR": P_HR_base,
}

# ─── Figure ──────────────────────────────────────────────────────────────────
fig, axes = plt.subplots(2, 2, figsize=(14, 8), sharex=True)
for leg, (r, c) in LEG_POS.items():
    ax = axes[r, c]
    for j, (jcol, jlbl) in enumerate(zip(COLORS, CART_LABELS)):
        ax.plot(t, P_base_data[leg][:, j], color=jcol, linewidth=1.3)
    ax.set_title(f"{leg} Leg", fontsize=TITLE_SIZE, fontweight="bold")
    ax.set_ylabel("Position (m)", fontsize=LABEL_SIZE)
    ax.set_xlim(t[0], t[-1])
    ax.set_ylim(-0.5, 0.5)
    ax.yaxis.set_major_locator(ticker.MultipleLocator(0.1))
    ax.grid(True, alpha=0.3)
    ax.tick_params(labelsize=TICK_SIZE)

# Right column: suppress redundant y-axis labels
for ax in [axes[0, 1], axes[1, 1]]:
    ax.tick_params(labelleft=False)
    ax.set_ylabel("")

# Bottom row: x-axis labels
axes[1, 0].set_xlabel("Time (s)", fontsize=LABEL_SIZE)
axes[1, 1].set_xlabel("Time (s)", fontsize=LABEL_SIZE)

# Shared legend at bottom
fig.legend(handles=cart_handles, loc="lower center",
           ncol=3, fontsize=LEGEND_SIZE, framealpha=0.9, bbox_to_anchor=(0.5, -0.03))
fig.tight_layout()
fig.subplots_adjust(bottom=0.1)
title = 'Kinematics Validation - Cartesian Space Base Frame Trajectory'
if save_plots:
    save_figure_from_title(title)

# Figure 4: Cartesian space base frame velocity of all 4 legs (V_xx_base: x_dot, y_dot, z_dot vs time)
# ─── Data ────────────────────────────────────────────────────────────────────
V_base_data = {
    "FL": V_FL_base,
    "FR": V_FR_base,
    "HL": V_HL_base,
    "HR": V_HR_base,
}

# ─── Figure ──────────────────────────────────────────────────────────────────
fig, axes = plt.subplots(2, 2, figsize=(14, 8), sharex=True)
for leg, (r, c) in LEG_POS.items():
    ax = axes[r, c]
    for j, (jcol, jlbl) in enumerate(zip(COLORS, CART_VEL_LABELS)):
        ax.plot(t, V_base_data[leg][:, j], color=jcol, linewidth=1.3)
    ax.set_title(f"{leg} Leg", fontsize=TITLE_SIZE, fontweight="bold")
    ax.set_ylabel("Velocity (m/s)", fontsize=LABEL_SIZE)
    ax.set_xlim(t[0], t[-1])
    ax.set_ylim(-0.1, 0.1)
    ax.yaxis.set_major_locator(ticker.MultipleLocator(0.02))
    ax.grid(True, alpha=0.3)
    ax.tick_params(labelsize=TICK_SIZE)

# Right column: suppress redundant y-axis labels
for ax in [axes[0, 1], axes[1, 1]]:
    ax.tick_params(labelleft=False)
    ax.set_ylabel("")

# Bottom row: x-axis labels
axes[1, 0].set_xlabel("Time (s)", fontsize=LABEL_SIZE)
axes[1, 1].set_xlabel("Time (s)", fontsize=LABEL_SIZE)

# Shared legend at bottom
fig.legend(handles=cart_vel_handles, loc="lower center",
           ncol=3, fontsize=LEGEND_SIZE, framealpha=0.9, bbox_to_anchor=(0.5, -0.03))
fig.tight_layout()
fig.subplots_adjust(bottom=0.1)
title = 'Kinematics Validation - Cartesian Space Base Frame Velocity'
if save_plots:
    save_figure_from_title(title)

# Figure 5: Joint space, joint angles time series of all 4 legs (Theta_xx: theta1, theta2, theta3 vs time)
# ─── Data ────────────────────────────────────────────────────────────────────
Theta_data = {
    "FL": Theta_FL,   # shape (num_time_steps, 3)
    "FR": Theta_FR,
    "HL": Theta_HL,
    "HR": Theta_HR,
}

# ─── Figure ──────────────────────────────────────────────────────────────────
fig, axes = plt.subplots(2, 2, figsize=(14, 8), sharex=True)
for leg, (r, c) in LEG_POS.items():
    ax = axes[r, c]
    for j, (jcol, jlbl) in enumerate(zip(COLORS, JOINT_LABELS)):
        ax.plot(t, Theta_data[leg][:, j], color=jcol, linewidth=1.3)
    ax.set_title(f"{leg} Leg", fontsize=TITLE_SIZE, fontweight="bold")
    ax.set_ylabel("Position (°)", fontsize=LABEL_SIZE)
    ax.set_xlim(t[0], t[-1])
    ax.set_ylim(-95, 95)
    ax.yaxis.set_major_locator(ticker.MultipleLocator(15))
    ax.grid(True, alpha=0.3)
    ax.tick_params(labelsize=TICK_SIZE)

# Right column: suppress redundant y-axis labels
for ax in [axes[0, 1], axes[1, 1]]:
    ax.tick_params(labelleft=False)
    ax.set_ylabel("")

# Bottom row: x-axis labels
axes[1, 0].set_xlabel("Time (s)", fontsize=LABEL_SIZE)
axes[1, 1].set_xlabel("Time (s)", fontsize=LABEL_SIZE)

# Shared legend at bottom
fig.legend(handles=joint_handles, loc="lower center",
           ncol=3, fontsize=LEGEND_SIZE, framealpha=0.9, bbox_to_anchor=(0.5, -0.03))
fig.tight_layout()
fig.subplots_adjust(bottom=0.1)
title = 'Kinematics Validation - Joint Space Joint Angles'
if save_plots:
    save_figure_from_title(title)

# Figure 6: Joint space, joint velocities time series of all 4 legs (Theta_dot_xx: theta_dot1, theta_dot2, theta_dot3 vs time)
# ─── Data ────────────────────────────────────────────────────────────────────
Theta_dot_data = {
    "FL": Theta_dot_FL.T,
    "FR": Theta_dot_FR.T,
    "HL": Theta_dot_HL.T,
    "HR": Theta_dot_HR.T,
}

# ─── Figure ──────────────────────────────────────────────────────────────────
fig, axes = plt.subplots(2, 2, figsize=(14, 8), sharex=True)
for leg, (r, c) in LEG_POS.items():
    ax = axes[r, c]
    for j, (jcol, jlbl) in enumerate(zip(COLORS, JOINT_VEL_LABELS)):
        ax.plot(t, Theta_dot_data[leg][:, j], color=jcol, linewidth=1.3)
    ax.set_title(f"{leg} Leg", fontsize=TITLE_SIZE, fontweight="bold")
    ax.set_ylabel("Velocity (°/s)", fontsize=LABEL_SIZE)
    ax.set_xlim(t[0], t[-1])
    ax.set_ylim(-50, 50)
    ax.yaxis.set_major_locator(ticker.MultipleLocator(15))
    ax.grid(True, alpha=0.3)
    ax.tick_params(labelsize=TICK_SIZE)

# Right column: suppress redundant y-axis labels
for ax in [axes[0, 1], axes[1, 1]]:
    ax.tick_params(labelleft=False)
    ax.set_ylabel("")

# Bottom row: x-axis labels
axes[1, 0].set_xlabel("Time (s)", fontsize=LABEL_SIZE)
axes[1, 1].set_xlabel("Time (s)", fontsize=LABEL_SIZE)

# Shared legend at bottom
fig.legend(handles=joint_vel_handles, loc="lower center",
           ncol=3, fontsize=LEGEND_SIZE, framealpha=0.9, bbox_to_anchor=(0.5, -0.03))
fig.tight_layout()
fig.subplots_adjust(bottom=0.1)
title = 'Kinematics Validation - Joint Space Joint Velocities'
if save_plots:
    save_figure_from_title(title)

# Figure 7: Cartesian space validation of forward kinematics of all 4 legs in base frame (P_xx_base_FK: x, y, z vs time)
# ─── Data ────────────────────────────────────────────────────────────────────
P_base_FK_data = {
    "FL": P_FL_base_FK,
    "FR": P_FR_base_FK,
    "HL": P_HL_base_FK,
    "HR": P_HR_base_FK,
}

# ─── Figure ──────────────────────────────────────────────────────────────────
fig, axes = plt.subplots(2, 2, figsize=(14, 8), sharex=True)
for leg, (r, c) in LEG_POS.items():
    ax = axes[r, c]
    for j, (jcol, jlbl) in enumerate(zip(COLORS, CART_LABELS)):
        ax.plot(t, P_base_FK_data[leg][:, j], color=jcol, linewidth=1.3)
    ax.set_title(f"{leg} Leg", fontsize=TITLE_SIZE, fontweight="bold")
    ax.set_ylabel("Position (m)", fontsize=LABEL_SIZE)
    ax.set_xlim(t[0], t[-1])
    ax.set_ylim(-0.5, 0.5)
    ax.yaxis.set_major_locator(ticker.MultipleLocator(0.1))
    ax.grid(True, alpha=0.3)
    ax.tick_params(labelsize=TICK_SIZE)

# Right column: suppress redundant y-axis labels
for ax in [axes[0, 1], axes[1, 1]]:
    ax.tick_params(labelleft=False)
    ax.set_ylabel("")

# Bottom row: x-axis labels
axes[1, 0].set_xlabel("Time (s)", fontsize=LABEL_SIZE)
axes[1, 1].set_xlabel("Time (s)", fontsize=LABEL_SIZE)

# Shared legend at bottom
fig.legend(handles=cart_handles, loc="lower center",
           ncol=3, fontsize=LEGEND_SIZE, framealpha=0.9, bbox_to_anchor=(0.5, -0.03))
fig.tight_layout()
fig.subplots_adjust(bottom=0.1)
title = 'Kinematics Validation - Cartesian Space Base Frame Trajectory after FK'
if save_plots:
    save_figure_from_title(title)

# Show plots
plt.show()