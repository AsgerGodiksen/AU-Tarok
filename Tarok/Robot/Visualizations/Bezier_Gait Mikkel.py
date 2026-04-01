# Imports
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))


from Robot import*

import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

Tarok = TarokDymensions()

# ─────────────────────────────────────────────
#  Parameters From Tarok Dimensions
# ─────────────────────────────────────────────
LEGS = Tarok.LEGS
COLORS = Tarok.COLORS
PHASE_OFFSET = Tarok.CRAWL_OFFSETS

# ─────────────────────────────────────────────
#  Parameters - For Time related things
# ─────────────────────────────────────────────

dt = 0.005 # seconds (200 Hz)

Swing_Time_Scalar = 3.0     # [s] swing phase duration
Stand_Time_Scalar  = 3 * Swing_Time_Scalar # [s] stand phase duration


Total_Time_Scalar = Swing_Time_Scalar + Stand_Time_Scalar
Total_Time_Steps = int(Total_Time_Scalar / dt) + 1

t_swing = np.arange(0, Swing_Time_Scalar + dt/2, dt) # Time array for swing phase
t_stand = np.arange(Swing_Time_Scalar + dt, Swing_Time_Scalar + Stand_Time_Scalar + dt/2, dt) # Time array for stand phase
t = np.concatenate((t_swing, t_stand)) # Full time array for one cycle of the gait

Swing_Time_Steps = len(t_swing)
Stand_Time_Steps = len(t_stand)

# ─────────────────────────────────────────────
#  Building the Trajectories for the Legs
# ─────────────────────────────────────────────

print("Building trajectories...")
Leg_Trajectories = {}
Thetas           = {}


Bezier_Trajectory, Bezier_Velocities, _ = Building_Bezier_Trajectories(
                                    Swing_Time_Scalar,
                                    Stand_Time_Scalar,
                                    Swing_Time_Steps,
                                    Stand_Time_Steps
                                    )
print(Bezier_Trajectory.shape)
# With the Bezier trajectory in the Body frame

# Plot t, Bezier_Trajectory[0, :], label='X')
plt.figure()
plt.plot(t, Bezier_Trajectory[0, :], label='X')
plt.xlabel('Time (s)')
plt.ylabel('X Position (m)')
plt.title('X Position of Bezier Trajectory Over Time')

plt.figure()
plt.plot(t, Bezier_Trajectory[1, :], label='Y')
plt.xlabel('Time (s)')
plt.ylabel('Y Position (m)')
plt.title('Y Position of Bezier Trajectory Over Time')

plt.figure()
plt.plot(t, Bezier_Trajectory[2, :], label='Z')
plt.xlabel('Time (s)')
plt.ylabel('Z Position (m)')
plt.title('Z Position of Bezier Trajectory Over Time')


# Define kinematic body lengths
l_k = 0.7048  # Length of body in kinematic model (meters)
w_k = 0.220   # Width of body in kinematic model (meters)

# Translating the body centered trajectory to the four leg base frame positions, to get four seperate trajectories
FL_Bezier_Trajectory = Bezier_Trajectory + np.array([[l_k/2], [w_k/2 + 0.078], [0]]) # add l_k/2 to all elements in the first column
FR_Bezier_Trajectory = Bezier_Trajectory + np.array([[l_k/2], [-w_k/2 -0.078], [0]]) # add l_k/2 to all elements in the first column, and subtract w_k/2 from all elements in the second column
HL_Bezier_Trajectory = Bezier_Trajectory + np.array([[-l_k/2], [w_k/2 + 0.078], [0]]) # add -l_k/2 to all elements in the first column, and add w_k/2 to all elements in the second column
HR_Bezier_Trajectory = Bezier_Trajectory + np.array([[-l_k/2], [-w_k/2 -0.078], [0]]) # add -l_k/2 to all elements in the first column, and subtract w_k/2 from all elements in the second column










# We can translate the Desired trajectory out to the four shoulders
Front_Left_Shoulder, Front_Right_Shoulder, Hind_Left_Shoulder, Hind_Right_Shoulder = Tarok.Shoulder_Positions()
#print(Front_Left_Shoulder.shape)
#FL_Bezier_Trajectory = (Bezier_Trajectory + Front_Left_Shoulder) 
# FR_Bezier_Trajectory = Bezier_Trajectory * np.array([[-1], [1], [1]]) + Front_Right_Shoulder
# HL_Bezier_Trajectory = Bezier_Trajectory * np.array([[-1], [1], [1]]) + Hind_Left_Shoulder
# FR_Bezier_Trajectory = (Bezier_Trajectory + Front_Right_Shoulder)
# HL_Bezier_Trajectory = (Bezier_Trajectory + Hind_Left_Shoulder)  
# HR_Bezier_Trajectory = (Bezier_Trajectory + Hind_Right_Shoulder) 



# Plot t, Bezier_Trajectory[0, :], label='X')
plt.figure()
plt.plot(t, FR_Bezier_Trajectory[0, :], label='X')
plt.xlabel('Time (s)')
plt.ylabel('X Position (m)')
plt.title('X Position of FR Bezier Trajectory Over Time')

plt.figure()
plt.plot(t, FR_Bezier_Trajectory[1, :], label='Y')
plt.xlabel('Time (s)')
plt.ylabel('Y Position (m)')
plt.title('Y Position of FR Bezier Trajectory Over Time')

plt.figure()
plt.plot(t, FR_Bezier_Trajectory[2, :], label='Z')
plt.xlabel('Time (s)')
plt.ylabel('Z Position (m)')
plt.title('Z Position of FR Bezier Trajectory Over Time')

plt.show()






# Create 2x2 subplot grid
fig, axs = plt.subplots(2, 2, figsize=(10, 8))

# Flatten axes array for easier indexing
axs = axs.flatten()

# Plot each trajectory
axs[0].plot(FL_Bezier_Trajectory[0, :], FL_Bezier_Trajectory[2, :],
            label='FL', color=COLORS['FL'])
axs[0].set_title('Front Left')

axs[1].plot(FR_Bezier_Trajectory[0, :], FR_Bezier_Trajectory[2, :],
            label='FR', color=COLORS['FR'])
axs[1].set_title('Front Right')

axs[2].plot(HL_Bezier_Trajectory[0, :], HL_Bezier_Trajectory[2, :],
            label='HL', color=COLORS['HL'])
axs[2].set_title('Hind Left')

axs[3].plot(HR_Bezier_Trajectory[0, :], HR_Bezier_Trajectory[2, :],
            label='HR', color=COLORS['HR'])
axs[3].set_title('Hind Right')

# Optional: make all plots look consistent
for ax in axs:
    ax.set_xlabel('X')
    ax.set_ylabel('Z')
    ax.legend()
    ax.grid(True)
fig.suptitle("Bezier Trajectories In Body Frame",
             fontsize=14, fontweight='bold')
plt.tight_layout()


FL_Bezier_Trajectory, FL_Bezier_Velocities = Apply_Phase_Offset(FL_Bezier_Trajectory, Bezier_Velocities, PHASE_OFFSET['FL'])
FR_Bezier_Trajectory, FR_Bezier_Velocities = Apply_Phase_Offset(FR_Bezier_Trajectory, Bezier_Velocities, PHASE_OFFSET['FR'])
HL_Bezier_Trajectory, HL_Bezier_Velocities = Apply_Phase_Offset(HL_Bezier_Trajectory, Bezier_Velocities, PHASE_OFFSET['HL'])
HR_Bezier_Trajectory, HR_Bezier_Velocities = Apply_Phase_Offset(HR_Bezier_Trajectory, Bezier_Velocities, PHASE_OFFSET['HR'])


# Create 2x2 subplot grid
fig, axs = plt.subplots(2, 2, figsize=(10, 8))

# Flatten axes array for easier indexing
axs = axs.flatten()

# Plot each trajectory
axs[0].plot(FL_Bezier_Trajectory[0, :], FL_Bezier_Trajectory[2, :],
            label='FL', color=COLORS['FL'])
axs[0].scatter(FL_Bezier_Trajectory[0, 0], FL_Bezier_Trajectory[2, 0],
            color=COLORS['FL'], s=200, zorder=3, label='Start')
axs[0].scatter(FL_Bezier_Trajectory[0, 100], FL_Bezier_Trajectory[2, 100],
            color=COLORS['FL'], s=100, zorder=3, label='Point 100')
axs[0].set_title('Front Left')

axs[1].plot(FR_Bezier_Trajectory[0, :], FR_Bezier_Trajectory[2, :],
            label='FR', color=COLORS['FR'])
axs[1].scatter(FR_Bezier_Trajectory[0, 0], FR_Bezier_Trajectory[2, 0],
            color=COLORS['FR'], s=200, zorder=3, label='Start')
axs[1].scatter(FR_Bezier_Trajectory[0, 100], FR_Bezier_Trajectory[2, 100],
            color=COLORS['FR'], s=100, zorder=3, label='Point 100')
axs[1].set_title('Front Right')

axs[2].plot(HL_Bezier_Trajectory[0, :], HL_Bezier_Trajectory[2, :],
            label='HL', color=COLORS['HL'])
axs[2].scatter(HL_Bezier_Trajectory[0, 0], HL_Bezier_Trajectory[2, 0],
            color=COLORS['HL'], s=200, zorder=3, label='Start')
axs[2].scatter(HL_Bezier_Trajectory[0, 100], HL_Bezier_Trajectory[2, 100],
            color=COLORS['HL'], s=100, zorder=3, label='Point 100')
axs[2].set_title('Hind Left')

axs[3].plot(HR_Bezier_Trajectory[0, :], HR_Bezier_Trajectory[2, :],
            label='HR', color=COLORS['HR'])
axs[3].scatter(HR_Bezier_Trajectory[0, 0], HR_Bezier_Trajectory[2, 0],
            color=COLORS['HR'], s=200, zorder=3, label='Start')
axs[3].scatter(HR_Bezier_Trajectory[0, 100], HR_Bezier_Trajectory[2, 100],
            color=COLORS['HR'], s=100, zorder=3, label='Point 100')
axs[3].set_title('Hind Right')

# Optional: make all plots look consistent
for ax in axs:
    ax.set_xlabel('X')
    ax.set_ylabel('Z')
    ax.legend()
    ax.grid(True)
fig.suptitle("Bezier Trajectories in BOdy With OffSets",
             fontsize=14, fontweight='bold')
plt.tight_layout()



# Transform the Bezier trajectory from Body Frame to Leg Base Frames
P_FL_Base = np.array([T0_B(FL_Bezier_Trajectory[:, i].reshape((3, 1)), 'FL') for i in range((len(t)))])
P_FR_Base = np.array([T0_B(FR_Bezier_Trajectory[:, i].reshape((3, 1)), 'FR') for i in range((len(t)))])
P_HL_Base = np.array([T0_B(HL_Bezier_Trajectory[:, i].reshape((3, 1)), 'HL') for i in range((len(t)))])
P_HR_Base = np.array([T0_B(HR_Bezier_Trajectory[:, i].reshape((3, 1)), 'HR') for i in range((len(t)))])

fig, axs = plt.subplots(2, 2, figsize=(10, 8))
axs = axs.flatten()

# Helper to plot + mark points
def plot_with_markers(ax, X, Z, label, color):
    ax.plot(X, Z, label=label, color=color)

    # First point
    ax.scatter(X[0], Z[0], color='green', s=80, zorder=3, label='Start')

    # 100th point (make sure it exists)
    if len(X) > 100:
        ax.scatter(X[100], Z[100], color='red', s=80, zorder=3, label='Point 100')

# Apply to each leg
plot_with_markers(axs[0], P_FL_Base[:, 2], -P_FL_Base[:, 0], 'FL', COLORS['FL'])
axs[0].set_title('Front Left')

plot_with_markers(axs[1], P_FR_Base[:, 2], -P_FR_Base[:, 0], 'FR', COLORS['FR'])
axs[1].set_title('Front Right')

plot_with_markers(axs[2], -P_HL_Base[:, 2], -P_HL_Base[:, 0], 'HL', COLORS['HL'])
axs[2].set_title('Hind Left')

plot_with_markers(axs[3], -P_HR_Base[:, 2], -P_HR_Base[:, 0], 'HR', COLORS['HR'])
axs[3].set_title('Hind Right')

# Formatting
for ax in axs:
    ax.set_xlabel('X')
    ax.set_ylabel('Z')
    ax.grid(True)
    ax.legend()
fig.suptitle("Bezier Trajectories Transformed from Body Frame to Leg Base Frames",
             fontsize=14, fontweight='bold')
plt.tight_layout()


# Transform desired end-effector velocity from body frame to leg base frames
V_FL_base = np.array([R0_B(FL_Bezier_Velocities[:, i].reshape((3, 1)), 'FL') for i in range((len(t)))])
V_FR_base = np.array([R0_B(FR_Bezier_Velocities[:, i].reshape((3, 1)), 'FR') for i in range((len(t)))])
V_HL_base = np.array([R0_B(HL_Bezier_Velocities[:, i].reshape((3, 1)), 'HL') for i in range((len(t)))])
V_HR_base = np.array([R0_B(HR_Bezier_Velocities[:, i].reshape((3, 1)), 'HR') for i in range((len(t)))])

### Kinematics ###
# Determine joint angles for all 4 legs using inverse kinematics
Theta_FL = np.array([Inverse_Kinematics(P_FL_Base[i], 'FL') for i in range((len(t)))]) # Shape (num_time_steps, 3), containing theta1, theta2, theta3 for each time step
Theta_FR = np.array([Inverse_Kinematics(P_FR_Base[i], 'FR') for i in range((len(t)))]) # Shape (num_time_steps, 3), containing theta1, theta2, theta3 for each time step
Theta_HL = np.array([Inverse_Kinematics(P_HL_Base[i], 'HL') for i in range((len(t)))]) # Shape (num_time_steps, 3), containing theta1, theta2, theta3 for each time step
Theta_HR = np.array([Inverse_Kinematics(P_HR_Base[i], 'HR') for i in range((len(t)))]) # Shape (num_time_steps, 3), containing theta1, theta2, theta3 for each time step

# Create time vector
t = np.arange(Total_Time_Steps)

# Create 2x2 subplot grid
fig, axs = plt.subplots(2, 2, figsize=(12, 8))
axs = axs.flatten()

Theta_FL_ = np.rad2deg(Theta_FL)
Theta_FR_ = np.rad2deg(Theta_FR)
Theta_HL_ = np.rad2deg(Theta_HL)
Theta_HR_ = np.rad2deg(Theta_HR)
# Helper function to plot 3 angles
def plot_leg(ax, Theta, title, color):
    ax.plot(t, Theta[:, 0], label=r'$\theta_1$', linestyle='-')
    ax.plot(t, Theta[:, 1], label=r'$\theta_2$', linestyle='--')
    ax.plot(t, Theta[:, 2], label=r'$\theta_3$', linestyle=':')

    ax.set_title(title)
    ax.set_xlabel('Time step')
    ax.set_ylabel('Angle (degrees)')
    ax.grid(True)
    ax.legend()

# Plot each leg
plot_leg(axs[0], Theta_FL_, 'Front Left', COLORS['FL'])
plot_leg(axs[1], Theta_FR_, 'Front Right', COLORS['FR'])
plot_leg(axs[2], Theta_HL_, 'Hind Left', COLORS['HL'])
plot_leg(axs[3], Theta_HR_, 'Hind Right', COLORS['HR'])

plt.tight_layout()



# Determine joint velocities for all 4 legs using Jacobian
# Damped least squares inverse to avoid singularities - theta_dot = (J^T*J + damp^2*I)^-1 * J^T * cartesian_velocity 
Theta_dot_FL = np.zeros((3, Total_Time_Steps))  # Initialize joint velocity array
Theta_dot_FR = np.zeros((3, Total_Time_Steps))  # Initialize joint velocity array
Theta_dot_HL = np.zeros((3, Total_Time_Steps))  # Initialize joint velocity array
Theta_dot_HR = np.zeros((3, Total_Time_Steps))  # Initialize joint velocity array

damp = 0.001  # Damping factor

for i in range((Total_Time_Steps)):
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


# ─────────────────────────────────────────────
#  Animation
# ─────────────────────────────────────────────

#### ANIMATION #### (Note: not true time)
# Animation which plots the trajectory of the legs in 3D space based on the computed joint angles and forward kinematics, showing the up/down motion of the legs as defined by the desired end-effector trajectory. The animation will show the movement of the legs over time, with the foot positions being updated according to the forward kinematics computed from the inverse kinematics joint angles.
# Note that the end-effector positions computed from the forward kinematics should be transformed to the body frame using CT.TB_0xx functions before plotting, to ensure that the trajectory is visualized in the correct frame of reference.
# Prepare figure
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim(-0.5, 0.5)
ax.set_ylim(-0.5, 0.5)
ax.set_zlim(-0.5, 0)
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Z (m)')
ax.set_title('Bezier Trajectory Motion')
ax.view_init(elev=20, azim=225)
lineFL, = ax.plot([], [], [], 'o-', lw=2, color=COLORS['FL'], label='FL')
lineFR, = ax.plot([], [], [], 'o-', lw=2, color=COLORS['FR'], label='FR')
lineHL, = ax.plot([], [], [], 'o-', lw=2, color=COLORS['HL'], label='HL')
lineHR, = ax.plot([], [], [], 'o-', lw=2, color=COLORS['HR'], label='HR')
ax.legend(loc='upper right')
def init():
    lineFL.set_data([], [])
    lineFL.set_3d_properties([])
    lineFR.set_data([], [])
    lineFR.set_3d_properties([])
    lineHL.set_data([], [])
    lineHL.set_3d_properties([])
    lineHR.set_data([], [])
    lineHR.set_3d_properties([])
    return lineFL, lineFR, lineHL, lineHR
def _pt(P, i):
    # Extract scalar coordinate i from a point, regardless of shape
    return float(np.array(P).ravel()[i])
def update(num):
    # Get current joint angles
    th1_FL, th2_FL, th3_FL = Theta_FL[num]
    th1_FR, th2_FR, th3_FR = Theta_FR[num]
    th1_HL, th2_HL, th3_HL = Theta_HL[num]
    th1_HR, th2_HR, th3_HR = Theta_HR[num]

    # Compute forward kinematics in leg base frames
    P0_1_FL = P0_1_FR = P0_1_HL = P0_1_HR = np.array([[0], [0], [0]])  # Placeholder for P0_1
    P0_2_FL = P0_2(th1_FL, th2_FL, th3_FL, 'FL')
    P0_2_FR = P0_2(th1_FR, th2_FR, th3_FR, 'FR')
    P0_2_HL = P0_2(th1_HL, th2_HL, th3_HL, 'HL')
    P0_2_HR = P0_2(th1_HR, th2_HR, th3_HR, 'HR') 
    P0_3_FL = P0_3(th1_FL, th2_FL, th3_FL, 'FL')
    P0_3_FR = P0_3(th1_FR, th2_FR, th3_FR, 'FR')
    P0_3_HL = P0_3(th1_HL, th2_HL, th3_HL, 'HL')
    P0_3_HR = P0_3(th1_HR, th2_HR, th3_HR, 'HR') 
    P0_end_FL = P0_end(th1_FL, th2_FL, th3_FL, 'FL')
    P0_end_FR = P0_end(th1_FR, th2_FR, th3_FR, 'FR')
    P0_end_HL = P0_end(th1_HL, th2_HL, th3_HL, 'HL')
    P0_end_HR = P0_end(th1_HR, th2_HR, th3_HR, 'HR') 

    # Transform end-effector positions to body frame
    P0_1_FL = TB_0(P0_1_FL, 'FL')
    P0_1_FR = TB_0(P0_1_FR, 'FR')
    P0_1_HL = TB_0(P0_1_HL, 'HL')
    P0_1_HR = TB_0(P0_1_HR, 'HR')
    P0_2_FL = TB_0(P0_2_FL, 'FL')
    P0_2_FR = TB_0(P0_2_FR, 'FR')
    P0_2_HL = TB_0(P0_2_HL, 'HL')
    P0_2_HR = TB_0(P0_2_HR, 'HR')
    P0_3_FL = TB_0(P0_3_FL, 'FL')
    P0_3_FR = TB_0(P0_3_FR, 'FR')
    P0_3_HL = TB_0(P0_3_HL, 'HL')
    P0_3_HR = TB_0(P0_3_HR, 'HR')
    P0_end_FL = TB_0(P0_end_FL, 'FL')
    P0_end_FR = TB_0(P0_end_FR, 'FR')
    P0_end_HL = TB_0(P0_end_HL, 'HL')
    P0_end_HR = TB_0(P0_end_HR, 'HR')

    # Update line data for each leg
    for line, pts in [
        (lineFL, [P0_1_FL, P0_2_FL, P0_3_FL, P0_end_FL]),
        (lineFR, [P0_1_FR, P0_2_FR, P0_3_FR, P0_end_FR]),
        (lineHL, [P0_1_HL, P0_2_HL, P0_3_HL, P0_end_HL]),
        (lineHR, [P0_1_HR, P0_2_HR, P0_3_HR, P0_end_HR]),
    ]:
        xs = [_pt(p, 0) for p in pts]
        ys = [_pt(p, 1) for p in pts]
        zs = [_pt(p, 2) for p in pts]
        line.set_data(xs, ys)
        line.set_3d_properties(zs)

    return lineFL, lineFR, lineHL, lineHR
ani = animation.FuncAnimation(fig, update, frames=(Total_Time_Steps), init_func=init,
                              interval=1, blit=True)
plt.show()

