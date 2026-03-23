# Script for testing up and down movement of the robot.
# Build on Main_Template.py for the structure of the code, and Test_visualization.py for the precomputation etc.

# Imports
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))
import can
import time
import numpy as np
from Robot import*

# Old: CAN initialization in terminal: "sudo ip link set dev canX up type can bitrate 1000000" - with "X" being 0, 1, 2 and 3 for each bus
# New: CAN initialization in terminal: "for i in 0 1 2 3; do sudo ip link set dev can$i up type can bitrate 1000000 && sudo ip link set can$i txqueuelen 1000; done"

### SCRIPT START ###
## PRECOMPUTATIONS ##
print("Performing pre-computations...")

# Define kinematic body lengths
l_k = 0.7048  # Length of body in kinematic model (meters)
w_k = 0.220   # Width of body in kinematic model (meters)

# Define time series
dt = 0.005 # seconds (200 Hz)
total_time = 10  # Total time in seconds
num_time_steps = int(total_time / dt) + 1
t = np.linspace(0, total_time, num_time_steps)

# Define desired end-effector trajectory as function of time for all 4 legs (in body frame)
x_FL = x_FR = (l_k/2)*np.ones_like(t)  # X position in meters (constant)
x_HL = x_HR = (-l_k/2)*np.ones_like(t)  # X position in meters (constant)
y_FL = y_HL = (w_k/2 + 0.078)*np.ones_like(t)  # Y position in meters (constant)
y_FR = y_HR = (-w_k/2 - 0.078)*np.ones_like(t)  # Y position in meters (constant)
z = np.piecewise(t, [t < 5, t >=5], [lambda t: -0.46 + (0.10/5)*t, lambda t: -0.36 - (0.10/5)*(t-5)])  # Z position in meters (linear wave from -0.46 to -0.36 and back to -0.46 in 10 seconds)

# Define desired end effector velocity (foot velocity) as functions of time for all 4 legs (in body frame) - Note, it is the same for all legs in body frame for this trajectory
x_dot = np.zeros_like(t)  # X velocity in meters/second (constant)
y_dot = np.zeros_like(t)  # Y velocity in meters/second (constant)
z_dot = np.piecewise(t, [t < 5, t >= 5], [lambda t: 0.10/5*np.ones_like(t), lambda t: -0.10/5*np.ones_like(t)])  # Z velocity in meters/second (linear wave from 0.10 m/s to -0.10 m/s and back to 0.10 m/s in 10 seconds)

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

print("Pre-computations complete.")

## INITIALIZATION ##
print("Starting the Robot")
print("Initializing CAN buses...")

# Defining motor IDs: Should be ID_1 for joint 1 etc.
ID_1 = 0x141
ID_2 = 0x142
ID_3 = 0x143

# Connect to CAN bus
bus0 = can.interface.Bus(channel="can0", interface="socketcan")
bus1 = can.interface.Bus(channel="can1", interface="socketcan")
bus2 = can.interface.Bus(channel="can2", interface="socketcan")
bus3 = can.interface.Bus(channel="can3", interface="socketcan")

# Drain any stale messages from the buses
for bus in [bus0, bus1, bus2, bus3]:
    for i in range(100):		# Now Listens for any signals 100 times with 0.01 s in between. Prints if any.
        msg = bus.recv(0.01)
        if msg:
            print(msg)

# Loop count
count = 0

print("Initialization complete, starting pre-loop sequence...")

# Move to zero position 
print("Moving to zero position...")
Position_Control(bus0,ID_1,0,30)
Position_Control(bus0,ID_2,0,30)
Position_Control(bus0,ID_3,0,30)
Position_Control(bus1,ID_1,0,30)
Position_Control(bus1,ID_2,0,30)
Position_Control(bus1,ID_3,0,30)
Position_Control(bus2,ID_1,0,30)
Position_Control(bus2,ID_2,0,30)
Position_Control(bus2,ID_3,0,30)
Position_Control(bus3,ID_1,0,30)
Position_Control(bus3,ID_2,0,30)
Position_Control(bus3,ID_3,0,30)

time.sleep(6)

# Move to initial position
print("Moved to zero position, moving to initial trajectory position...")
Position_Control(bus0,ID_1,Theta_FL[0],30)
Position_Control(bus0,ID_2,Theta_FL[1],30)
Position_Control(bus0,ID_3,Theta_FL[2],30)
Position_Control(bus1,ID_1,Theta_FR[0],30)
Position_Control(bus1,ID_2,Theta_FR[1],30)
Position_Control(bus1,ID_3,Theta_FR[2],30)
Position_Control(bus2,ID_1,Theta_HL[0],30)
Position_Control(bus2,ID_2,Theta_HL[1],30)
Position_Control(bus2,ID_3,Theta_HL[2],30)
Position_Control(bus3,ID_1,Theta_HR[0],30)
Position_Control(bus3,ID_2,Theta_HR[1],30)
Position_Control(bus3,ID_3,Theta_HR[2],30)

time.sleep(6)

print("Pre-loop sequence complete, starting loop...")
print("Loop started - Press ctrl+c in terminal for shutdown")

# Note start time
start_time = cycle_start = current_time = time.monotonic()

try:
    while True:
            # Loop count
            count = count + 1

            # Loop time managment
            current_time = time.monotonic()
            elapsed_cycle = current_time - cycle_start # Elapsed time in current cycle
            elapsed_total = current_time - start_time  # Elapsed time since start of program
            # Check if current cycle is over -> start new cycle
            if elapsed_cycle >= total_time:
                cycle_start += total_time # Force next cycle start time to be exactly total trajectory time after previous cycle start time to avoid drift
                continue

            # Find closest value in t to elapsed in current cycle
            index = min(int(elapsed_cycle / dt), len(t) - 1)

            # Send position control commands to motors for current time step
            Position_Control(bus0, ID_1, Theta_FL[index, 0], Theta_dot_FL[0, index])
            Position_Control(bus0, ID_2, Theta_FL[index, 1], Theta_dot_FL[1, index])
            Position_Control(bus0, ID_3, Theta_FL[index, 2], Theta_dot_FL[2, index])
            Position_Control(bus1, ID_1, Theta_FR[index, 0], Theta_dot_FR[0, index])
            Position_Control(bus1, ID_2, Theta_FR[index, 1], Theta_dot_FR[1, index])
            Position_Control(bus1, ID_3, Theta_FR[index, 2], Theta_dot_FR[2, index])
            Position_Control(bus2, ID_1, Theta_HL[index, 0], Theta_dot_HL[0, index])
            Position_Control(bus2, ID_2, Theta_HL[index, 1], Theta_dot_HL[1, index])
            Position_Control(bus2, ID_3, Theta_HL[index, 2], Theta_dot_HL[2, index])
            Position_Control(bus3, ID_1, Theta_HR[index, 0], Theta_dot_HR[0, index])
            Position_Control(bus3, ID_2, Theta_HR[index, 1], Theta_dot_HR[1, index])
            Position_Control(bus3, ID_3, Theta_HR[index, 2], Theta_dot_HR[2, index])

# Stop loop with Ctrl+C in terminal
except KeyboardInterrupt:
    print("KeyboardInterrupt received, shutting down...")

    # Stop motors
    print("Stopping motors...")
    Motor_Stop(bus0,ID_1)
    Motor_Stop(bus0,ID_2)
    Motor_Stop(bus0,ID_3)
    Motor_Stop(bus1,ID_1)
    Motor_Stop(bus1,ID_2)
    Motor_Stop(bus1,ID_3)
    Motor_Stop(bus2,ID_1)
    Motor_Stop(bus2,ID_2)
    Motor_Stop(bus2,ID_3)
    Motor_Stop(bus3,ID_1)
    Motor_Stop(bus3,ID_2)
    Motor_Stop(bus3,ID_3)
    print("Motors stopped")

    # Shutdown CAN buses
    print("Shutting down CAN buses...")
    bus0.shutdown()
    bus1.shutdown()
    bus2.shutdown()
    bus3.shutdown()
    print("CAN buses shut down")
    print("Shutdown complete.")