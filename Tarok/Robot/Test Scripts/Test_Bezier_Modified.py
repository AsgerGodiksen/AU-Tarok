# Test script for bezier trajectory with modified stand phase trajectory based on our own idea
# The idea of modifying stand phase height individually for each leg to force COM movement

# Imports
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../..'))) # Change level of path based on file location (the ../../)
import can
import time
import numpy as np
from Robot import*


# Old: CAN initialization in terminal: "sudo ip link set dev canX up type can bitrate 1000000" - with "X" being 0, 1, 2 and 3 for each bus
# New: CAN initialization in terminal: "for i in 0 1 2 3; do sudo ip link set dev can$i up type can bitrate 1000000 && sudo ip link set can$i txqueuelen 1000; done"

### SCRIPT START ###
## PRECOMPUTATIONS ##
print("Performing pre-computations...")

# Parameters From Tarok Dimensions
Tarok = TarokDymensions()
LEGS = Tarok.LEGS
COLORS = Tarok.COLORS
PHASE_OFFSET = Tarok.CRAWL_OFFSETS

# Time parameters
dt = 0.005 # seconds (200 Hz)

Swing_Time_Scalar = 1.5    # [s] swing phase duration
Stand_Time_Scalar  = 3 * Swing_Time_Scalar # [s] stand phase duration

total_time = Swing_Time_Scalar + Stand_Time_Scalar
Total_Time_Steps = int(total_time / dt) + 1

t_swing = np.arange(0, Swing_Time_Scalar + dt/2, dt) # Time array for swing phase
t_stand = np.arange(Swing_Time_Scalar + dt, Swing_Time_Scalar + Stand_Time_Scalar + dt/2, dt) # Time array for stand phase
t = np.concatenate((t_swing, t_stand)) # Full time array for one cycle of the gait

Swing_Time_Steps = len(t_swing)
Stand_Time_Steps = len(t_stand)
Total_Time_Steps = len(t)

# Trajectory generation (Bezier curve in body frame)
Bezier_Trajectory_Left, Bezier_Velocities_Left, _ = Building_Bezier_Trajectories(
                                    Swing_Time_Scalar,
                                    Stand_Time_Scalar,
                                    Swing_Time_Steps,
                                    Stand_Time_Steps,
                                    Stand_Phase_type = "Modified",
                                    Leg = "FL"
                                    )

Bezier_Trajectory_Right, Bezier_Velocities_Right, _ = Building_Bezier_Trajectories(
                                    Swing_Time_Scalar,
                                    Stand_Time_Scalar,
                                    Swing_Time_Steps,
                                    Stand_Time_Steps,
                                    Stand_Phase_type = "Modified",
                                    Leg = "FR"
                                    )

# Translation to four shoulder
Front_Left_Shoulder, Front_Right_Shoulder, Hind_Left_Shoulder, Hind_Right_Shoulder = Tarok.Shoulder_Positions()
FL_Bezier_Trajectory = (Bezier_Trajectory_Left + Front_Left_Shoulder) 
FR_Bezier_Trajectory = (Bezier_Trajectory_Right + Front_Right_Shoulder)
HL_Bezier_Trajectory = (Bezier_Trajectory_Left + Hind_Left_Shoulder)  
HR_Bezier_Trajectory = (Bezier_Trajectory_Right + Hind_Right_Shoulder) 

# Apply phase offsets for crawl gait
FL_Bezier_Trajectory, FL_Bezier_Velocities = Apply_Phase_Offset(FL_Bezier_Trajectory, Bezier_Velocities_Left, PHASE_OFFSET['FL'])
FR_Bezier_Trajectory, FR_Bezier_Velocities = Apply_Phase_Offset(FR_Bezier_Trajectory, Bezier_Velocities_Right, PHASE_OFFSET['FR'])
HL_Bezier_Trajectory, HL_Bezier_Velocities = Apply_Phase_Offset(HL_Bezier_Trajectory, Bezier_Velocities_Left, PHASE_OFFSET['HL'])
HR_Bezier_Trajectory, HR_Bezier_Velocities = Apply_Phase_Offset(HR_Bezier_Trajectory, Bezier_Velocities_Right, PHASE_OFFSET['HR'])

# Transform the Bezier trajectory from Body Frame to Leg Base Frames
P_FL_Base = np.array([T0_B(FL_Bezier_Trajectory[:, i].reshape((3, 1)), 'FL') for i in range((len(t)))])
P_FR_Base = np.array([T0_B(FR_Bezier_Trajectory[:, i].reshape((3, 1)), 'FR') for i in range((len(t)))])
P_HL_Base = np.array([T0_B(HL_Bezier_Trajectory[:, i].reshape((3, 1)), 'HL') for i in range((len(t)))])
P_HR_Base = np.array([T0_B(HR_Bezier_Trajectory[:, i].reshape((3, 1)), 'HR') for i in range((len(t)))])

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

# Determine joint velocities for all 4 legs using Jacobian
# Damped least squares inverse to avoid singularities - theta_dot = (J^T*J + damp^2*I)^-1 * J^T * cartesian_velocity 
Theta_dot_FL = np.zeros((3, len(t)))  # Initialize joint velocity array
Theta_dot_FR = np.zeros((3, len(t)))  # Initialize joint velocity array
Theta_dot_HL = np.zeros((3, len(t)))  # Initialize joint velocity array
Theta_dot_HR = np.zeros((3, len(t)))  # Initialize joint velocity array
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

# Convert joint angles and velocities to degrees and abs(degrees/s) for right units for motor control
Theta_FL = np.rad2deg(Theta_FL)
Theta_FR = np.rad2deg(Theta_FR)
Theta_HL = np.rad2deg(Theta_HL)
Theta_HR = np.rad2deg(Theta_HR)
Theta_dot_FL = np.abs(np.rad2deg(Theta_dot_FL))
Theta_dot_FR = np.abs(np.rad2deg(Theta_dot_FR))
Theta_dot_HL = np.abs(np.rad2deg(Theta_dot_HL))
Theta_dot_HR = np.abs(np.rad2deg(Theta_dot_HR))

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

print("Initialization complete, starting pre-loop sequence...")

## PRE-LOOP SEQUENCE ##

# Write PI parameters to motors - adjust as needed for your application, or remove if not needed
# Note: This is the ones tuned for up/down (Test 19)
PI_Params = {
    'angle_kp':  110,
    'angle_ki':  40,
    'speed_kp':  55,
    'speed_ki':  16,
    'torque_kp': 55,
    'torque_ki': 20
}

print("Writing PI parameters to motors...")
PID_RAM_Control(bus0,ID_1, PI_Params)
PID_RAM_Control(bus0,ID_2, PI_Params)
PID_RAM_Control(bus0,ID_3, PI_Params)
PID_RAM_Control(bus1,ID_1, PI_Params)
PID_RAM_Control(bus1,ID_2, PI_Params)
PID_RAM_Control(bus1,ID_3, PI_Params)
PID_RAM_Control(bus2,ID_1, PI_Params)  
PID_RAM_Control(bus2,ID_2, PI_Params)
PID_RAM_Control(bus2,ID_3, PI_Params)
PID_RAM_Control(bus3,ID_1, PI_Params)
PID_RAM_Control(bus3,ID_2, PI_Params)
PID_RAM_Control(bus3,ID_3, PI_Params)

time.sleep(0.2) # Sleep for a short time to ensure parameters are written before starting loop, adjust as needed

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
Position_Control(bus0,ID_1,Theta_FL[0, 0],30)
Position_Control(bus0,ID_2,Theta_FL[0, 1],30)
Position_Control(bus0,ID_3,Theta_FL[0, 2],30)
Position_Control(bus1,ID_1,Theta_FR[0, 0],30)
Position_Control(bus1,ID_2,Theta_FR[0, 1],30)
Position_Control(bus1,ID_3,Theta_FR[0, 2],30)
Position_Control(bus2,ID_1,Theta_HL[0, 0],30)
Position_Control(bus2,ID_2,Theta_HL[0, 1],30)
Position_Control(bus2,ID_3,Theta_HL[0, 2],30)
Position_Control(bus3,ID_1,Theta_HR[0, 0],30)
Position_Control(bus3,ID_2,Theta_HR[0, 1],30)
Position_Control(bus3,ID_3,Theta_HR[0, 2],30)

time.sleep(6)

print("Pre-loop sequence complete, starting loop...")

## MAIN LOOP ##
print("Loop started - Press ctrl+c in terminal for shutdown")

# Note start time
start_time = cycle_start = current_time = time.monotonic()

try:
    while True:
            # Loop time managment (if needed)
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