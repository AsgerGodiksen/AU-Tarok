##### MAIN SCRIPT ######
# Connect to CAN buses in terminal according to:
# CAN initialization in terminal: "for i in 0 1 2 3; do sudo ip link set dev can$i up type can bitrate 1000000 && sudo ip link set can$i txqueuelen 1000; done"

# This is a state machine
# Run this script with the quadruped initially in the test stand, verify the correct startup positions were upheld when powering on the actuators, and then move it away from test stand when it has reached standing pose
# Main/Basis state is standing pose - every requested state starts and ends in standing pose

# Keyboard controls for state changes in main loop:
# Press "S" to switch to standing pose state, which is the basis state that every other state starts and ends in. This is also the initial state when starting the script, so it will start in standing pose.
# Press "U" to switch to up/down state, press "S" to switch back to standing pose state
# Press "W" to switch to walking gait state, press "S" to switch back to standing pose state

# Additional states can be added as needed

# Imports
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..')))
import can
import time
import numpy as np
from pynput import keyboard
import threading
from Robot import*

### DEFINITIONS ###
lock = threading.Lock()     # Create a lock for thread-safe access to shared variables
pre_loop_running = True     # Flag to control the pre-loop sequence, initially set to True to run the pre-loop sequence
State = "STAND"             # Define state variable, initial state is standing pose

### FUNCTION DEFINITIONS ###

# Thread-safe function to handle key presses for pre-loop sequence
def pre_on_press(key):
    # If pressing "enter", exit pre-loop verification sequence and continue to main loop
    global pre_loop_running
    with lock:
        if key == keyboard.Key.enter:
            print("Enter key pressed - Ending pre-loop verification sequence and continuing pre-loop sequence")
            pre_loop_running = False

# Thread-safe function to handle key presses for main loop state changes
def main_on_press(key):
    global State
    with lock:
        if key == keyboard.KeyCode.from_char('u'):
            State = "UP/DOWN"
            print("U key pressed - Changing state to UP/DOWN")
        if key == keyboard.KeyCode.from_char('s'):
            State = "STAND"
            print("S key pressed - Changing state to STAND")

### STATE FUNCTIONS ###
# Define functions for each state here, such as standing pose, walking gait, etc. Each function should implement the behavior for that state and return to standing pose at the end of the state.

# Up/Down state function
def Up_Down_State(bus0, bus1, bus2, bus3, ID_1, ID_2, ID_3):
    '''
    State function for up/down movement.
    This function computes an up/down trajectory and runs it until the state is changed by the user.

    Press "U" to switch to up/down state, press "S" to switch back to standing pose state

    Inputs:
    - bus0, bus1, bus2, bus3: CAN bus objects for each leg
    - ID_1, ID_2, ID_3: Motor IDs for each joint

    Returns:
    - None, but runs the up/down trajectory until state is changed by user, at which point it returns to standing pose
    '''

    print("Executing UP/DOWN state")
    print("Performing pre-computations for UP/DOWN state...")

    # Define kinematic body lengths
    l_k = 0.7048  # Length of body in kinematic model (meters)
    w_k = 0.220   # Width of body in kinematic model (meters)

    # Define local time series
    dt = 0.005 # seconds (200 Hz) - not directly the control frequency, but the discretization used for precomputations
    total_time = 10  # Total time in seconds of one up/down cycle
    num_time_steps = int(total_time / dt) + 1
    t = np.linspace(0, total_time, num_time_steps)

    # Define desired end-effector trajectory as function of time for all 4 legs (in body frame)
    x_FL_UD = x_FR_UD = (l_k/2)*np.ones_like(t)  # X position in meters (constant)
    x_HL_UD = x_HR_UD = (-l_k/2)*np.ones_like(t)  # X position in meters (constant)
    y_FL_UD = y_HL_UD = (w_k/2 + 0.078)*np.ones_like(t)  # Y position in meters (constant)
    y_FR_UD = y_HR_UD = (-w_k/2 - 0.078)*np.ones_like(t)  # Y position in meters (constant)
    z_UD = np.piecewise(t, [t < 5, t >=5], [lambda t: -0.46 + (0.10/5)*t, lambda t: -0.36 - (0.10/5)*(t-5)])  # Z position in meters (linear wave from -0.46 to -0.36 and back to -0.46 in 10 seconds)

    # Define desired end effector velocity (foot velocity) as functions of time for all 4 legs (in body frame) - Note, it is the same for all legs in body frame for this trajectory
    x_dot_UD = np.zeros_like(t)  # X velocity in meters/second (constant)
    y_dot_UD = np.zeros_like(t)  # Y velocity in meters/second (constant)
    z_dot_UD = np.piecewise(t, [t < 5, t >= 5], [lambda t: 0.10/5*np.ones_like(t), lambda t: -0.10/5*np.ones_like(t)])  # Z velocity in meters/second (linear wave from 0.10 m/s to -0.10 m/s and back to 0.10 m/s in 10 seconds)

    ### Transformations ###
    # Combine trajectories into position arrays for each leg
    P_FL_body_UD = np.vstack((x_FL_UD, y_FL_UD, z_UD))
    P_FR_body_UD = np.vstack((x_FR_UD, y_FR_UD, z_UD))
    P_HL_body_UD = np.vstack((x_HL_UD, y_HL_UD, z_UD))
    P_HR_body_UD = np.vstack((x_HR_UD, y_HR_UD, z_UD))

    # Transform desired end-effector trajectory from body frame to leg base frames
    P_FL_base_UD = np.array([T0_B(P_FL_body_UD[:, i].reshape((3, 1)), 'FL') for i in range(len(t))])
    P_FR_base_UD = np.array([T0_B(P_FR_body_UD[:, i].reshape((3, 1)), 'FR') for i in range(len(t))])
    P_HL_base_UD = np.array([T0_B(P_HL_body_UD[:, i].reshape((3, 1)), 'HL') for i in range(len(t))])
    P_HR_base_UD = np.array([T0_B(P_HR_body_UD[:, i].reshape((3, 1)), 'HR') for i in range(len(t))])

    # Combine body frame trajectory cartesian velocities into array
    V_body_UD = np.vstack((x_dot_UD, y_dot_UD, z_dot_UD))

    # Transform desired end-effector velocity from body frame to leg base frames
    V_FL_base_UD = np.array([R0_B(V_body_UD[:, i].reshape((3, 1)), 'FL') for i in range(len(t))])
    V_FR_base_UD = np.array([R0_B(V_body_UD[:, i].reshape((3, 1)), 'FR') for i in range(len(t))])
    V_HL_base_UD = np.array([R0_B(V_body_UD[:, i].reshape((3, 1)), 'HL') for i in range(len(t))])
    V_HR_base_UD = np.array([R0_B(V_body_UD[:, i].reshape((3, 1)), 'HR') for i in range(len(t))])

    ### Kinematics ###
    # Determine joint angles for all 4 legs using inverse kinematics
    Theta_FL_UD = np.array([Inverse_Kinematics(P_FL_base_UD[i], 'FL') for i in range(len(t))]) # Shape (num_time_steps, 3), containing theta1, theta2, theta3 for each time step
    Theta_FR_UD = np.array([Inverse_Kinematics(P_FR_base_UD[i], 'FR') for i in range(len(t))]) # Shape (num_time_steps, 3), containing theta1, theta2, theta3 for each time step
    Theta_HL_UD = np.array([Inverse_Kinematics(P_HL_base_UD[i], 'HL') for i in range(len(t))]) # Shape (num_time_steps, 3), containing theta1, theta2, theta3 for each time step
    Theta_HR_UD = np.array([Inverse_Kinematics(P_HR_base_UD[i], 'HR') for i in range(len(t))]) # Shape (num_time_steps, 3), containing theta1, theta2, theta3 for each time step

    # Determine joint velocities for all 4 legs using Jacobian
    # Damped least squares inverse to avoid singularities - theta_dot = (J^T*J + damp^2*I)^-1 * J^T * cartesian_velocity 
    Theta_dot_FL_UD = np.zeros((3, len(t)))  # Initialize joint velocity array
    Theta_dot_FR_UD = np.zeros((3, len(t)))  # Initialize joint velocity array
    Theta_dot_HL_UD = np.zeros((3, len(t)))  # Initialize joint velocity array
    Theta_dot_HR_UD = np.zeros((3, len(t)))  # Initialize joint velocity array
    damp = 0.001  # Damping factor
    for i in range(len(t)):
        Jac_i_FL_UD = Jacobian(Theta_FL_UD[i, 0], Theta_FL_UD[i, 1], Theta_FL_UD[i, 2], 'FL')
        Jac_i_FR_UD = Jacobian(Theta_FR_UD[i, 0], Theta_FR_UD[i, 1], Theta_FR_UD[i, 2], 'FR')
        Jac_i_HL_UD = Jacobian(Theta_HL_UD[i, 0], Theta_HL_UD[i, 1], Theta_HL_UD[i, 2], 'HL')
        Jac_i_HR_UD = Jacobian(Theta_HR_UD[i, 0], Theta_HR_UD[i, 1], Theta_HR_UD[i, 2], 'HR')
        JT_FL_UD = Jac_i_FL_UD.T
        JT_FR_UD = Jac_i_FR_UD.T
        JT_HL_UD = Jac_i_HL_UD.T
        JT_HR_UD = Jac_i_HR_UD.T
        term_FL_UD = JT_FL_UD @ Jac_i_FL_UD + (damp**2)*np.eye(3)
        term_FR_UD = JT_FR_UD @ Jac_i_FR_UD + (damp**2)*np.eye(3)
        term_HL_UD = JT_HL_UD @ Jac_i_HL_UD + (damp**2)*np.eye(3)
        term_HR_UD = JT_HR_UD @ Jac_i_HR_UD + (damp**2)*np.eye(3)
        Theta_dot_FL_UD[:, i] = np.linalg.solve(term_FL_UD, JT_FL_UD @ V_FL_base_UD[:, i].flatten())
        Theta_dot_FR_UD[:, i] = np.linalg.solve(term_FR_UD, JT_FR_UD @ V_FR_base_UD[:, i].flatten())
        Theta_dot_HL_UD[:, i] = np.linalg.solve(term_HL_UD, JT_HL_UD @ V_HL_base_UD[:, i].flatten())
        Theta_dot_HR_UD[:, i] = np.linalg.solve(term_HR_UD, JT_HR_UD @ V_HR_base_UD[:, i].flatten())

    # Convert joint angles and velocities to degrees and abs(degrees/s) for right units for motor control
    Theta_FL_UD = np.rad2deg(Theta_FL_UD)
    Theta_FR_UD = np.rad2deg(Theta_FR_UD)
    Theta_HL_UD = np.rad2deg(Theta_HL_UD)
    Theta_HR_UD = np.rad2deg(Theta_HR_UD)
    Theta_dot_FL_UD = np.abs(np.rad2deg(Theta_dot_FL_UD))
    Theta_dot_FR_UD = np.abs(np.rad2deg(Theta_dot_FR_UD))
    Theta_dot_HL_UD = np.abs(np.rad2deg(Theta_dot_HL_UD))
    Theta_dot_HR_UD = np.abs(np.rad2deg(Theta_dot_HR_UD))

    # Move to intial position (top of the up/down movement)
    print("Moving to initial position for UP/DOWN movement...")
    Position_Control(bus0,ID_1,Theta_FL_UD[0, 0],30)
    Position_Control(bus0,ID_2,Theta_FL_UD[0, 1],30)
    Position_Control(bus0,ID_3,Theta_FL_UD[0, 2],30)
    Position_Control(bus1,ID_1,Theta_FR_UD[0, 0],30)
    Position_Control(bus1,ID_2,Theta_FR_UD[0, 1],30)
    Position_Control(bus1,ID_3,Theta_FR_UD[0, 2],30)
    Position_Control(bus2,ID_1,Theta_HL_UD[0, 0],30)
    Position_Control(bus2,ID_2,Theta_HL_UD[0, 1],30)
    Position_Control(bus2,ID_3,Theta_HL_UD[0, 2],30)
    Position_Control(bus3,ID_1,Theta_HR_UD[0, 0],30)
    Position_Control(bus3,ID_2,Theta_HR_UD[0, 1],30)
    Position_Control(bus3,ID_3,Theta_HR_UD[0, 2],30)

    time.sleep(6)

    print("Pre-loop sequence complete, starting UP/DOWN loop...")
    print("Press 'S' to switch to standing pose - Press ctrl+c in terminal for shutdown")

    # Note start time
    start_time = cycle_start = current_time = time.monotonic()
    try:
        while True:
            # Loop time managment
            current_time = time.monotonic()
            elapsed_cycle = current_time - cycle_start # Elapsed time in current cycle
            #elapsed_total = current_time - start_time  # Elapsed time since start of program
            # Check if current cycle is over -> start new cycle
            if elapsed_cycle >= total_time:
                cycle_start += total_time # Force next cycle start time to be exactly total trajectory time after previous cycle start time to avoid drift
                continue

            # Find closest value in t to elapsed in current cycle
            index = min(int(elapsed_cycle / dt), len(t) - 1)
            
            # Send position control commands to motors for current time step
            Position_Control(bus0, ID_1, Theta_FL_UD[index, 0], Theta_dot_FL_UD[0, index])
            Position_Control(bus0, ID_2, Theta_FL_UD[index, 1], Theta_dot_FL_UD[1, index])
            Position_Control(bus0, ID_3, Theta_FL_UD[index, 2], Theta_dot_FL_UD[2, index])
            Position_Control(bus1, ID_1, Theta_FR_UD[index, 0], Theta_dot_FR_UD[0, index])
            Position_Control(bus1, ID_2, Theta_FR_UD[index, 1], Theta_dot_FR_UD[1, index])
            Position_Control(bus1, ID_3, Theta_FR_UD[index, 2], Theta_dot_FR_UD[2, index])
            Position_Control(bus2, ID_1, Theta_HL_UD[index, 0], Theta_dot_HL_UD[0, index])
            Position_Control(bus2, ID_2, Theta_HL_UD[index, 1], Theta_dot_HL_UD[1, index])
            Position_Control(bus2, ID_3, Theta_HL_UD[index, 2], Theta_dot_HL_UD[2, index])
            Position_Control(bus3, ID_1, Theta_HR_UD[index, 0], Theta_dot_HR_UD[0, index])
            Position_Control(bus3, ID_2, Theta_HR_UD[index, 1], Theta_dot_HR_UD[1, index])
            Position_Control(bus3, ID_3, Theta_HR_UD[index, 2], Theta_dot_HR_UD[2, index])
            
            with lock:
                if State != "UP/DOWN":  # Check if state has been changed, if so exit this state function to switch to new state
                    print("State change detected, exiting UP/DOWN state function")
                    break

    except KeyboardInterrupt:
        raise # Just raise the exception to be caught in the main loop try-except block for shutdown

    # Move to standing pose at the end of the state
    print("Moving to standing pose at the end of UP/DOWN state...")
    Position_Control(bus0,ID_1,Theta_FL_stand[0],30)
    Position_Control(bus0,ID_2,Theta_FL_stand[1],30)
    Position_Control(bus0,ID_3,Theta_FL_stand[2],30)
    Position_Control(bus1,ID_1,Theta_FR_stand[0],30)
    Position_Control(bus1,ID_2,Theta_FR_stand[1],30)
    Position_Control(bus1,ID_3,Theta_FR_stand[2],30)
    Position_Control(bus2,ID_1,Theta_HL_stand[0],30)
    Position_Control(bus2,ID_2,Theta_HL_stand[1],30)
    Position_Control(bus2,ID_3,Theta_HL_stand[2],30)
    Position_Control(bus3,ID_1,Theta_HR_stand[0],30)
    Position_Control(bus3,ID_2,Theta_HR_stand[1],30)
    Position_Control(bus3,ID_3,Theta_HR_stand[2],30)
    print("Returned to standing pose at the end of UP/DOWN state")

### SCRIPT START ###
## PRECOMPUTATIONS ##
print("Performing pre-computations...")

'''
# Define time series
dt = 0.005 # seconds (200 Hz) - adjust as needed for your application - not directly the control frequency, but the discretization used for precomputations
total_time = 10  # Total time in seconds of basis state cycle
num_time_steps = int(total_time / dt) + 1
t = np.linspace(0, total_time, num_time_steps)
'''

# Define kinematic body lengths
l_k = 0.7048  # Length of body in kinematic model (meters)
w_k = 0.220   # Width of body in kinematic model (meters)

## Stand Pose Precomputations
# Define desired end-effector position for standing posture (z=-0.41 meters) for all 4 legs
x_FL_stand = x_FR_stand = l_k/2 # x-position of front legs
x_HL_stand = x_HR_stand = -l_k/2 # x-position of hind legs
y_FL_stand = y_HL_stand = w_k/2 + 0.078 # y-position of left legs
y_FR_stand = y_HR_stand = -w_k/2 - 0.078 # y-position of right legs
z_stand = -0.41 # z-position for standing posture (similar for all legs)

# Combine trajectories into position arrays for each leg
P_FL_body_stand = np.array([x_FL_stand, y_FL_stand, z_stand])
P_FR_body_stand = np.array([x_FR_stand, y_FR_stand, z_stand])
P_HL_body_stand = np.array([x_HL_stand, y_HL_stand, z_stand])
P_HR_body_stand = np.array([x_HR_stand, y_HR_stand, z_stand])

# Transform desired end-effector trajectory from body frame to leg base frames
P_FL_base_stand = T0_B(P_FL_body_stand.reshape((3,1)), 'FL')
P_FR_base_stand = T0_B(P_FR_body_stand.reshape((3,1)), 'FR')
P_HL_base_stand = T0_B(P_HL_body_stand.reshape((3,1)), 'HL')
P_HR_base_stand = T0_B(P_HR_body_stand.reshape((3,1)), 'HR')

# Determine joint angles for all 4 legs using inverse kinematics
Theta_FL_stand = Inverse_Kinematics(P_FL_base_stand, 'FL')
Theta_FR_stand = Inverse_Kinematics(P_FR_base_stand, 'FR')
Theta_HL_stand = Inverse_Kinematics(P_HL_base_stand, 'HL')
Theta_HR_stand = Inverse_Kinematics(P_HR_base_stand, 'HR')

# Convert angles from radians to degrees for motor control
Theta_FL_stand = np.degrees(Theta_FL_stand)
Theta_FR_stand = np.degrees(Theta_FR_stand)
Theta_HL_stand = np.degrees(Theta_HL_stand)
Theta_HR_stand = np.degrees(Theta_HR_stand)

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

print("Initialization complete, starting pre-loop verification sequence...")

## PRE-LOOP VERIFICATION SEQUENCE ##
# Pre-loop sequence for checking that power on positions are corerct before doing any movements
# Need human verification of correct startup positions before moving the legs to avoid damage to the robot

# Starting listener for pre-loop sequence - waiting for user to verify correct startup positions and then press "enter" to continue to main loop, or Ctrl+C in terminal to shutdown if positions are not correct to avoid damage to the robot
pre_listener = keyboard.Listener(on_press=pre_on_press)
pre_listener.start() 

# Loop for verifiying correct startup positions - will print current positions of all legs every 1.5 seconds until user presses "enter" to continue to main loop, or Ctrl+C in terminal to shutdown if positions are not correct to avoid damage to the robot
try:
    while True:
      # Weak torque control to make movement of legs easy
        Torque_Control(bus0, ID_1, 0.02)
        Torque_Control(bus0, ID_2, 0.02)
        Torque_Control(bus0, ID_3, 0.02)
        Torque_Control(bus1, ID_1, 0.02)
        Torque_Control(bus1, ID_2, 0.02)
        Torque_Control(bus1, ID_3, 0.02)
        Torque_Control(bus2, ID_1, 0.02)
        Torque_Control(bus2, ID_2, 0.02)
        Torque_Control(bus2, ID_3, 0.02)
        Torque_Control(bus3, ID_1, 0.02)
        Torque_Control(bus3, ID_2, 0.02)
        Torque_Control(bus3, ID_3, 0.02)

        time.sleep(0.5)

        # Read current positions for all 4 legs
        FL_theta1 = Read_Angle(bus0,ID_1)
        FL_theta2 = Read_Angle(bus0,ID_2)
        FL_theta3 = Read_Angle(bus0,ID_3)
        FR_theta1 = Read_Angle(bus1,ID_1)
        FR_theta2 = Read_Angle(bus1,ID_2)
        FR_theta3 = Read_Angle(bus1,ID_3)
        HL_theta1 = Read_Angle(bus2,ID_1)
        HL_theta2 = Read_Angle(bus2,ID_2)
        HL_theta3 = Read_Angle(bus2,ID_3)
        HR_theta1 = Read_Angle(bus3,ID_1)
        HR_theta2 = Read_Angle(bus3,ID_2)
        HR_theta3 = Read_Angle(bus3,ID_3)

        # Print the current positions of all 4 legs
        print("  ")
        print(f"Joint positions for Front Left:   theta1 = {FL_theta1:.2f},  theta2 = {FL_theta2:.2f},  theta3 = {FL_theta3:.2f}")
        print(f"Joint positions for Front Right:  theta1 = {FR_theta1:.2f},  theta2 = {FR_theta2:.2f},  theta3 = {FR_theta3:.2f}")
        print(f"Joint positions for Hind Left:    theta1 = {HL_theta1:.2f},  theta2 = {HL_theta2:.2f},  theta3 = {HL_theta3:.2f}")
        print(f"Joint positions for Hind Right:   theta1 = {HR_theta1:.2f},  theta2 = {HR_theta2:.2f},  theta3 = {HR_theta3:.2f}")
        print(f"========== ======== {count}")
        print("Check startup positions - Move legs to zero-configuration and check that angles are correct")
        print("Press 'enter' to continue if correct positions are verified")
        print("Press 'ctrl+c' in terminal for shutdown if positions are not correct to avoid damage to the robot")

        # Check for user input once per iteration: Press 'Enter' in listener thread to exit loop and continue to main loop, or Ctrl+C in terminal to shutdown if positions are not correct to avoid damage to the robot
        with lock:
            if not pre_loop_running:
                break
        
        time.sleep(1.5)

except KeyboardInterrupt:
    raise # Just raise the exception to be caught in the main loop try-except block for shutdown

# Stopping listener for pre-loop sequence
pre_listener.stop()  

### PRE-LOOP MOVEMENT SEQUENCE ###
print("Pre-loop verification sequence complete, starting pre-loop movement sequence...")

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

# Move to standing position
print("Moved to zero position, moving to standing position...")
Position_Control(bus0,ID_1,Theta_FL_stand[0],30)
Position_Control(bus0,ID_2,Theta_FL_stand[1],30)
Position_Control(bus0,ID_3,Theta_FL_stand[2],30)
Position_Control(bus1,ID_1,Theta_FR_stand[0],30)
Position_Control(bus1,ID_2,Theta_FR_stand[1],30)
Position_Control(bus1,ID_3,Theta_FR_stand[2],30)
Position_Control(bus2,ID_1,Theta_HL_stand[0],30)
Position_Control(bus2,ID_2,Theta_HL_stand[1],30)
Position_Control(bus2,ID_3,Theta_HL_stand[2],30)
Position_Control(bus3,ID_1,Theta_HR_stand[0],30)
Position_Control(bus3,ID_2,Theta_HR_stand[1],30)
Position_Control(bus3,ID_3,Theta_HR_stand[2],30)

time.sleep(6)

print("Pre-loop sequence complete, starting loop...")

## MAIN LOOP ##
# Starting listener for Main loop
main_listener = keyboard.Listener(on_press=main_on_press)
main_listener.start()  # Start the listener in a separate thread

print("Loop started - Press ctrl+c in terminal for shutdown")

# Note start time
start_time = cycle_start = current_time = time.monotonic()

try:
    while True:
        # Print loop info
        print("Current state: Standing pose - Loop running - Press ctrl+c in terminal for shutdown")

        #### To do later: Check battery voltage and raise warning if low   #####
        ### If battery voltage is low: 20% - make small known movement to show that robot is alive but needs charging, and print warning message about low battery voltage
        # Trap it in a loop that can only be exited by pressing ctrl+c to shutdown
        # In the loop - go quick small/up down every 10 second or something like that

        # Keep stand position
        Position_Control(bus0,ID_1,Theta_FL_stand[0],30)
        Position_Control(bus0,ID_2,Theta_FL_stand[1],30)
        Position_Control(bus0,ID_3,Theta_FL_stand[2],30)
        Position_Control(bus1,ID_1,Theta_FR_stand[0],30)
        Position_Control(bus1,ID_2,Theta_FR_stand[1],30)
        Position_Control(bus1,ID_3,Theta_FR_stand[2],30)
        Position_Control(bus2,ID_1,Theta_HL_stand[0],30)
        Position_Control(bus2,ID_2,Theta_HL_stand[1],30)
        Position_Control(bus2,ID_3,Theta_HL_stand[2],30)
        Position_Control(bus3,ID_1,Theta_HR_stand[0],30)
        Position_Control(bus3,ID_2,Theta_HR_stand[1],30)
        Position_Control(bus3,ID_3,Theta_HR_stand[2],30)

        # Keep checking state change often for 30 second before continuing to run full main loop
        for _ in range(100):
            time.sleep(0.3)
            with lock:
                if State != "STAND":  # Check if state has been changed, if so break to switch to new state
                    break

        # Check if state has been changed, if so execute new state function
        with lock:
            current_state = State
        if current_state == "UP/DOWN":
            Up_Down_State(bus0, bus1, bus2, bus3, ID_1, ID_2, ID_3)
            with lock:
                State = "STAND" # After finishing the state function, switch back to standing pose state
        # add more states here as needed with elif statements

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

    # Stopping listeners if they were started and are still running
    try:
        pre_listener.stop()
    except:
        pass
    try:        
        main_listener.stop()
    except:
        pass

    print("Shutdown complete.")