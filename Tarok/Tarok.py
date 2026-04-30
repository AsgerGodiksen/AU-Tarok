##### MAIN SCRIPT ######
# Connect to CAN buses in terminal according to:
# CAN initialization in terminal: "for i in 0 1 2 3; do sudo ip link set dev can$i up type can bitrate 1000000 && sudo ip link set can$i txqueuelen 1000; done"

# This is a state machine
# Run this script with the quadruped initially in the test stand, verify the correct startup positions were upheld when powering on the actuators, and then move it away from test stand when it has reached standing pose
# Main/Basis state is standing pose - every requested state starts and ends in standing pose

# If the battery voltage is low, the system will automatically switch to a low battery state, which will run a small recognizable trajectory until the system is shut down. It is not possible to escape this state without shutting down the system.

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

### PARAMETER IMPORTS ###
Tarok = TarokDymensions()
#PHASE_OFFSET = Tarok.CRAWL_OFFSETS
PHASE_OFFSET = Tarok.CRAWL_OFFSETS_Mixed

## PI PARAMETERS ## 
# Stand state PI parameters (test 10)
pi_stand = {
    'angle_kp':  120,
    'angle_ki':  25,
    'speed_kp':  60,
    'speed_ki':  10,
    'torque_kp': 60,
    'torque_ki': 13
}

# Low Battery state PI parameters (Compromise parameters)
pi_low_battery = {
    'angle_kp':  110,
    'angle_ki':  50,
    'speed_kp':  55,
    'speed_ki':  20,
    'torque_kp': 55,
    'torque_ki': 25
}

'''
# Up/Down state PI parameters (Manufacturing parameters)
pi_up_down = {
    'angle_kp':  100,
    'angle_ki':  100,
    'speed_kp':  50,
    'speed_ki':  40,
    'torque_kp': 50,
    'torque_ki': 50
}
'''

# Up/Down state PI parameters (compromise parameters)
pi_up_down = {
    'angle_kp':  110,
    'angle_ki':  50,
    'speed_kp':  55,
    'speed_ki':  20,
    'torque_kp': 55,
    'torque_ki': 25
}

'''
# Bezier walk state PI parameters - currently the ones tuned for up/down (Test 19)
pi_bezier_walk = {
    'angle_kp':  110,
    'angle_ki':  40,
    'speed_kp':  55,
    'speed_ki':  16,
    'torque_kp': 55,
    'torque_ki': 20
}
'''
'''
# Bezier walk state PI parameters - Manufacturing - the ones used in Test_Bezier_Transfer_With_Stand.py
pi_bezier_walk = {
    'angle_kp':  100,
    'angle_ki':  100,
    'speed_kp':  50,
    'speed_ki':  40,
    'torque_kp': 50,
    'torque_ki': 50
}
'''
# Bezier walk state PI parameters - Tuned parameters from tuning 30/4 2026
pi_bezier_walk = {
    'angle_kp':  120,
    'angle_ki':  90,
    'speed_kp':  60,
    'speed_ki':  80,
    'torque_kp': 60,
    'torque_ki': 60
}

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
        if key == keyboard.KeyCode.from_char('w'):
            State = "BEZIER WALK GAIT"
            print("W key pressed - Changing state to BEZIER WALK GAIT")
        if key == keyboard.KeyCode.from_char('r'):
            State = "REVERSE BEZIER WALK GAIT"
            print("R key pressed - Changing state to REVERSE BEZIER WALK GAIT")

### STATE FUNCTIONS ###
# Define functions for each state here, such as standing pose, walking gait, etc. Each function should implement the behavior for that state and return to standing pose at the end of the state.

# Low battery state function
def Low_Battery_State(bus0, bus1, bus2, bus3, ID_1, ID_2, ID_3):
    '''
    State function for low battery condition.
    This function computes a small recognizable trajectory and runs it until the system is shut down.

    It is not possible to escape this state without shutting down the system

    Inputs:
    - bus0, bus1, bus2, bus3: CAN bus objects for each leg
    - ID_1, ID_2, ID_3: Motor IDs for each joint

    Returns:
    - None, but runs the low battery trajectory until the system is shut down    
    '''

    print("Executing LOW BATTERY state")
    print("Performing pre-computations for LOW BATTERY state...")

    # Define kinematic body lengths
    l_k = 0.7048  # Length of body in kinematic model (meters)
    w_k = 0.220   # Width of body in kinematic model (meters)

    # Define local time series
    dt = 0.005 # seconds (200 Hz) - not directly the control frequency, but the discretization used for precomputations
    total_time = 10  # Total time in seconds of one cycle of the trajectory
    num_time_steps = int(total_time / dt) + 1
    t = np.linspace(0, total_time, num_time_steps)

    # Segment boundaries for the trajectory: 0->1s, 1->1.5s, 1.5->2s, 2->3s, 3->10s
    conditions = [(t >= 0)   & (t < 1),    # down: -0.41 -> -0.36
                  (t >= 1)   & (t < 1.5),  # up:   -0.36 -> -0.385
                  (t >= 1.5) & (t < 2),    # down: -0.385 -> -0.36
                  (t >= 2)   & (t < 3),    # up:   -0.36 -> -0.41
                  (t >= 3)]                # hold: -0.41

    # Define desired end-effector trajectory as function of time for all 4 legs (in body frame)
    x_FL = x_FR = (l_k/2)*np.ones_like(t)  # X position in meters (constant)
    x_HL = x_HR = (-l_k/2)*np.ones_like(t)  # X position in meters (constant)
    y_FL = y_HL = (w_k/2 + 0.078)*np.ones_like(t)  # Y position in meters (constant)
    y_FR = y_HR = (-w_k/2 - 0.078)*np.ones_like(t)  # Y position in meters (constant)
    z = np.piecewise(t, conditions, [lambda t: cos_interp(t, -0.41, -0.36, 0, 1),
                                     lambda t: cos_interp(t, -0.36, -0.385, 1, 1.5),
                                     lambda t: cos_interp(t, -0.385, -0.36, 1.5, 2),
                                     lambda t: cos_interp(t, -0.36, -0.41, 2, 3),
                                     lambda t: -0.41 * np.ones_like(t)]) # Z position in meters, piecewise cosine interpolation for smooth trajectory between points

    # Define desired end effector velocity (foot velocity) as functions of time for all 4 legs (in body frame) - Note, it is the same for all legs in body frame for this trajectory
    x_dot = np.zeros_like(t)  # X velocity in meters/second (constant)
    y_dot = np.zeros_like(t)  # Y velocity in meters/second (constant)
    z_dot = np.piecewise(t, conditions, [lambda t: cos_interp_dot(t, -0.41, -0.36, 0, 1),
                                         lambda t: cos_interp_dot(t, -0.36, -0.385, 1, 1.5),
                                         lambda t: cos_interp_dot(t, -0.385, -0.36, 1.5, 2),
                                         lambda t: cos_interp_dot(t, -0.36, -0.41, 2, 3),
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

    # Convert joint angles and velocities to degrees and abs(degrees/s) for right units for motor control
    Theta_FL = np.rad2deg(Theta_FL)
    Theta_FR = np.rad2deg(Theta_FR)
    Theta_HL = np.rad2deg(Theta_HL)
    Theta_HR = np.rad2deg(Theta_HR)
    Theta_dot_FL = np.abs(np.rad2deg(Theta_dot_FL))
    Theta_dot_FR = np.abs(np.rad2deg(Theta_dot_FR))
    Theta_dot_HL = np.abs(np.rad2deg(Theta_dot_HL))
    Theta_dot_HR = np.abs(np.rad2deg(Theta_dot_HR))

    print("Pre-loop sequence complete, writing low battery state PI parameters and starting LOW BATTERY loop...")
    
    # Write low battery state PI parameters
    PID_RAM_Control(bus0,ID_1, pi_low_battery)
    PID_RAM_Control(bus0,ID_2, pi_low_battery)
    PID_RAM_Control(bus0,ID_3, pi_low_battery)
    PID_RAM_Control(bus1,ID_1, pi_low_battery)
    PID_RAM_Control(bus1,ID_2, pi_low_battery)
    PID_RAM_Control(bus1,ID_3, pi_low_battery)
    PID_RAM_Control(bus2,ID_1, pi_low_battery)  
    PID_RAM_Control(bus2,ID_2, pi_low_battery)
    PID_RAM_Control(bus2,ID_3, pi_low_battery)
    PID_RAM_Control(bus3,ID_1, pi_low_battery)
    PID_RAM_Control(bus3,ID_2, pi_low_battery)
    PID_RAM_Control(bus3,ID_3, pi_low_battery)

    print("Press ctrl+c in terminal for shutdown")

    # Note start time
    cycle_start = current_time = time.monotonic()
    try:
        while True:
            # Loop time managment
            current_time = time.monotonic()
            elapsed_cycle = current_time - cycle_start # Elapsed time in current cycle

            # Check if current cycle is over -> start new cycle
            if elapsed_cycle >= total_time:
                print("LOW BATTERY! Press ctrl+c in terminal to shutdown the system")
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

    except KeyboardInterrupt:
        raise # Just raise the exception to be caught in the main loop try-except block for shutdown

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
    total_time = 4  # Total time in seconds of one cycle of the trajectory
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

    # Convert joint angles and velocities to degrees and abs(degrees/s) for right units for motor control
    Theta_FL = np.rad2deg(Theta_FL)
    Theta_FR = np.rad2deg(Theta_FR)
    Theta_HL = np.rad2deg(Theta_HL)
    Theta_HR = np.rad2deg(Theta_HR)
    Theta_dot_FL = np.abs(np.rad2deg(Theta_dot_FL))
    Theta_dot_FR = np.abs(np.rad2deg(Theta_dot_FR))
    Theta_dot_HL = np.abs(np.rad2deg(Theta_dot_HL))
    Theta_dot_HR = np.abs(np.rad2deg(Theta_dot_HR))

    # Roll arrays by 25% of time steps to start at the standing position
    roll_amount = num_time_steps - int((3/4 * total_time) / dt)
    Theta_FL = np.roll(Theta_FL, roll_amount, axis=0)
    Theta_FR = np.roll(Theta_FR, roll_amount, axis=0)
    Theta_HL = np.roll(Theta_HL, roll_amount, axis=0)
    Theta_HR = np.roll(Theta_HR, roll_amount, axis=0)
    Theta_dot_FL = np.roll(Theta_dot_FL, roll_amount, axis=1)
    Theta_dot_FR = np.roll(Theta_dot_FR, roll_amount, axis=1)
    Theta_dot_HL = np.roll(Theta_dot_HL, roll_amount, axis=1)
    Theta_dot_HR = np.roll(Theta_dot_HR, roll_amount, axis=1)

    print("Pre-loop sequence complete, writing up/down state PI parameters...")

    # Write up/down state PI parameters
    PID_RAM_Control(bus0,ID_1, pi_up_down)
    PID_RAM_Control(bus0,ID_2, pi_up_down)
    PID_RAM_Control(bus0,ID_3, pi_up_down)
    PID_RAM_Control(bus1,ID_1, pi_up_down)
    PID_RAM_Control(bus1,ID_2, pi_up_down)
    PID_RAM_Control(bus1,ID_3, pi_up_down)
    PID_RAM_Control(bus2,ID_1, pi_up_down)  
    PID_RAM_Control(bus2,ID_2, pi_up_down)
    PID_RAM_Control(bus2,ID_3, pi_up_down)
    PID_RAM_Control(bus3,ID_1, pi_up_down)
    PID_RAM_Control(bus3,ID_2, pi_up_down)
    PID_RAM_Control(bus3,ID_3, pi_up_down)

    print("Press 'S' to switch to standing pose - Press ctrl+c in terminal for shutdown")

    # Note start time
    cycle_start = current_time = time.monotonic()
    try:
        while True:
            # Loop time managment
            current_time = time.monotonic()
            elapsed_cycle = current_time - cycle_start # Elapsed time in current cycle

            # Check if current cycle is over -> start new cycle
            if elapsed_cycle >= total_time:
                cycle_start += total_time # Force next cycle start time to be exactly total trajectory time after previous cycle start time to avoid drift
                # Check if state has been changed, if so exit this state function to switch to new state
                with lock:
                    if State != "UP/DOWN":  # Check if state has been changed, if so exit this state function to switch to new state
                        print("State change detected, exiting UP/DOWN state function")
                        break
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

    except KeyboardInterrupt:
        raise # Just raise the exception to be caught in the main loop try-except block for shutdown

    print("Returned to standing pose at the end of UP/DOWN state")

# Bezier Walk gait state function
def Bezier_Walk_State(bus0, bus1, bus2, bus3, ID_1, ID_2, ID_3, Direction = "Forward"):
    '''
    State function for Bezier walk gait.
    This function computes a Bezier walk gait trajectory and runs it until the state is changed by the user.
    The State includes a transfer phase from standing pose to initial walk position and back to standing pose at the end of the state.

    Press "W" to switch to walking gait state, press "S" to switch back to standing pose state

    Inputs:
    - bus0, bus1, bus2, bus3: CAN bus objects for each leg
    - ID_1, ID_2, ID_3: Motor IDs for each joint
    - Direction: "Forward" or "Backward", determines the direction of the walk, default is "Forward"

    Returns:
    - None, but runs the Bezier walk gait trajectory until state is changed by user, at which point it returns to standing pose
    '''

    print("Executing BEZIER WALK GAIT state")
    print("Performing pre-computations for BEZIER WALK GAIT state...")

    # Offsets for COM transfer during stand phase
    x_offset = 0.03 # [m] how much to move COM forward/backward during transfer
    y_offset = 0.04 # [m] how much to move COM to the left/right during transfer

    # Time parameters
    dt = 0.005 # seconds (200 Hz)

    Swing_Time_Scalar = 0.8    # [s] swing phase duration
    Stand_Time_Scalar  = 3 * Swing_Time_Scalar # [s] stand phase duration
    Transfer_Time_Scalar = 1.2 # [s] duration of the COM transfer

    total_time = Swing_Time_Scalar + Stand_Time_Scalar
    Total_Time_Steps = int(total_time / dt)
    total_time_with_transfer = total_time + 4 * Transfer_Time_Scalar
    Total_Time_Steps_with_transfer = int(total_time_with_transfer / dt)

    t_swing = np.linspace(0, Swing_Time_Scalar - dt, int(Swing_Time_Scalar / dt)) # Time array for swing phase
    t_stand = np.linspace(Swing_Time_Scalar, total_time - dt, int(Stand_Time_Scalar / dt)) # Time array for stand phase
    t_transfer = np.linspace(0, Transfer_Time_Scalar - dt, int(Transfer_Time_Scalar / dt)) # Time array for transfer phase

    t = np.concatenate((t_swing, t_stand)) # Full time array for one cycle of the gait
    t_with_transfer = np.linspace(0, total_time_with_transfer - dt, Total_Time_Steps_with_transfer)

    Swing_Time_Steps = len(t_swing)
    Stand_Time_Steps = len(t_stand)
    Transfer_Time_Steps = len(t_transfer)

    # Define time parameters for transition from stand height to walk height
    total_time_SWH = 1 # [s] total time for transition from stand height to walk height
    num_time_steps_SWH = int(total_time_SWH / dt)
    t_SWH = np.linspace(0, total_time_SWH, num_time_steps_SWH) # time array for transition from stand height to walk height

    # Define time parameters for transition from walk height to walk start position
    STW_Swing_Time_Scalar = 2.5 # [s] duration of swing phase for transition from stand height to walk start
    t_swing_STW = np.linspace(0, STW_Swing_Time_Scalar - dt, int(STW_Swing_Time_Scalar / dt)) # Time array for swing phase
    STW_Swing_Time_Steps = len(t_swing_STW) 
    total_time_STW = 4 * STW_Swing_Time_Scalar
    t_STW = np.linspace(0, total_time_STW - dt, int(total_time_STW / dt)) # Full time array for transition from stand height to walk start

    # ----------------------------- #
    ### WALK GENERATION ###
    # ----------------------------- #

    # Trajectory generation (Bezier curve in body frame)
    Bezier_Trajectory, Bezier_Velocities, _ = Building_Bezier_Trajectories(
                                        Swing_Time_Scalar,
                                        Stand_Time_Scalar,
                                        Swing_Time_Steps,
                                        Stand_Time_Steps,
                                        Stand_Phase_type = "Constant",
                                        Leg = "FL"
                                        )

    # Translation to four shoulder
    Front_Left_Shoulder, Front_Right_Shoulder, Hind_Left_Shoulder, Hind_Right_Shoulder = Tarok.Shoulder_Positions()
    FL_Bezier_Trajectory = (Bezier_Trajectory + Front_Left_Shoulder) 
    FR_Bezier_Trajectory = (Bezier_Trajectory + Front_Right_Shoulder)
    HL_Bezier_Trajectory = (Bezier_Trajectory + Hind_Left_Shoulder)  
    HR_Bezier_Trajectory = (Bezier_Trajectory + Hind_Right_Shoulder) 

    # Apply phase offsets for crawl gait
    FL_Bezier_Trajectory, FL_Bezier_Velocities = Apply_Phase_Offset(FL_Bezier_Trajectory, Bezier_Velocities, PHASE_OFFSET['FL'])
    FR_Bezier_Trajectory, FR_Bezier_Velocities = Apply_Phase_Offset(FR_Bezier_Trajectory, Bezier_Velocities, PHASE_OFFSET['FR'])
    HL_Bezier_Trajectory, HL_Bezier_Velocities = Apply_Phase_Offset(HL_Bezier_Trajectory, Bezier_Velocities, PHASE_OFFSET['HL'])
    HR_Bezier_Trajectory, HR_Bezier_Velocities = Apply_Phase_Offset(HR_Bezier_Trajectory, Bezier_Velocities, PHASE_OFFSET['HR'])

    ### ADD TRANSFER PHASE TO TRAJECTORY ###
    FL_Bezier_Trajectory_With_Transfer, FR_Bezier_Trajectory_With_Transfer, HL_Bezier_Trajectory_With_Transfer, HR_Bezier_Trajectory_With_Transfer, FL_Bezier_Velocities_With_Transfer, FR_Bezier_Velocities_With_Transfer, HL_Bezier_Velocities_With_Transfer, HR_Bezier_Velocities_With_Transfer = Bezier_Add_Transfer_Phase(
                t,
                t_transfer,
                t_with_transfer,
                Total_Time_Steps,
                Total_Time_Steps_with_transfer,
                Transfer_Time_Steps,
                Swing_Time_Steps,
                Transfer_Time_Scalar,
                PHASE_OFFSET,
                x_offset,
                y_offset,
                FL_Bezier_Trajectory,
                FR_Bezier_Trajectory,
                HL_Bezier_Trajectory,
                HR_Bezier_Trajectory,
                FL_Bezier_Velocities,
                FR_Bezier_Velocities,
                HL_Bezier_Velocities,
                HR_Bezier_Velocities
    )

    ### TRANSFORMATIONS ###
    # Transform the Bezier trajectory from Body Frame to Leg Base Frames
    P_FL_Base = np.array([T0_B(FL_Bezier_Trajectory_With_Transfer[:, i].reshape((3, 1)), 'FL') for i in range((len(t_with_transfer)))])
    P_FR_Base = np.array([T0_B(FR_Bezier_Trajectory_With_Transfer[:, i].reshape((3, 1)), 'FR') for i in range((len(t_with_transfer)))])
    P_HL_Base = np.array([T0_B(HL_Bezier_Trajectory_With_Transfer[:, i].reshape((3, 1)), 'HL') for i in range((len(t_with_transfer)))])
    P_HR_Base = np.array([T0_B(HR_Bezier_Trajectory_With_Transfer[:, i].reshape((3, 1)), 'HR') for i in range((len(t_with_transfer)))])

    # Transform desired end-effector velocity from body frame to leg base frames
    V_FL_base = np.array([R0_B(FL_Bezier_Velocities_With_Transfer[:, i].reshape((3, 1)), 'FL') for i in range((len(t_with_transfer)))])
    V_FR_base = np.array([R0_B(FR_Bezier_Velocities_With_Transfer[:, i].reshape((3, 1)), 'FR') for i in range((len(t_with_transfer)))])
    V_HL_base = np.array([R0_B(HL_Bezier_Velocities_With_Transfer[:, i].reshape((3, 1)), 'HL') for i in range((len(t_with_transfer)))])
    V_HR_base = np.array([R0_B(HR_Bezier_Velocities_With_Transfer[:, i].reshape((3, 1)), 'HR') for i in range((len(t_with_transfer)))])

    ### Kinematics ###
    # Determine joint angles for all 4 legs using inverse kinematics
    Theta_FL = np.array([Inverse_Kinematics(P_FL_Base[i], 'FL') for i in range((len(t_with_transfer)))]) # Shape (num_time_steps, 3), containing theta1, theta2, theta3 for each time step
    Theta_FR = np.array([Inverse_Kinematics(P_FR_Base[i], 'FR') for i in range((len(t_with_transfer)))]) # Shape (num_time_steps, 3), containing theta1, theta2, theta3 for each time step
    Theta_HL = np.array([Inverse_Kinematics(P_HL_Base[i], 'HL') for i in range((len(t_with_transfer)))]) # Shape (num_time_steps, 3), containing theta1, theta2, theta3 for each time step
    Theta_HR = np.array([Inverse_Kinematics(P_HR_Base[i], 'HR') for i in range((len(t_with_transfer)))]) # Shape (num_time_steps, 3), containing theta1, theta2, theta3 for each time step

    # Determine joint velocities for all 4 legs using Jacobian
    # Damped least squares inverse to avoid singularities - theta_dot = (J^T*J + damp^2*I)^-1 * J^T * cartesian_velocity 
    Theta_dot_FL = np.zeros((3, len(t_with_transfer)))  # Initialize joint velocity array
    Theta_dot_FR = np.zeros((3, len(t_with_transfer)))  # Initialize joint velocity array
    Theta_dot_HL = np.zeros((3, len(t_with_transfer)))  # Initialize joint velocity array
    Theta_dot_HR = np.zeros((3, len(t_with_transfer)))  # Initialize joint velocity array
    damp = 0.001  # Damping factor
    for i in range((Total_Time_Steps_with_transfer)):
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

    # ----------------------------- #
    ### STAND HEIGHT TO WALK HEIGHT GENERATION ###
    # ----------------------------- #

    # Define desired end-effector trajectory for transition from stand height to walk height
    x_FL_SWH = x_FR_SWH = (l_k/2) * np.ones_like(t_SWH) # X position in meters (constant)
    x_HL_SWH = x_HR_SWH = (-l_k/2) * np.ones_like(t_SWH) # X position in meters (constant)
    y_FL_SWH = y_HL_SWH = (w_k/2 + 0.078) * np.ones_like(t_SWH) # Y position in meters (constant)
    y_FR_SWH = y_HR_SWH = (-w_k/2 - 0.078) * np.ones_like(t_SWH) # Y position in meters (constant)
    z_SWH = cos_interp(t_SWH, z_stand, -P_FL_Base[0, 0], 0, total_time_SWH) # Z position in meters (cosine interpolation from stand height to walk height)

    # Define desired end-effector velocity for transition from stand height to walk height
    x_dot_SWH = np.zeros_like(t_SWH) # X velocity in m/s (constant)
    y_dot_SWH = np.zeros_like(t_SWH) # Y velocity in m/s (constant)
    z_dot_SWH = cos_interp_dot(t_SWH, z_stand, -P_FL_Base[0, 0], 0, total_time_SWH) # Z velocity in m/s (derivative of cosine interpolation)

    ### Transformations ###
    # Combine trajectories into position arrays for each leg
    P_FL_body_SWH = np.vstack((x_FL_SWH, y_FL_SWH, z_SWH))
    P_FR_body_SWH = np.vstack((x_FR_SWH, y_FR_SWH, z_SWH))
    P_HL_body_SWH = np.vstack((x_HL_SWH, y_HL_SWH, z_SWH))
    P_HR_body_SWH = np.vstack((x_HR_SWH, y_HR_SWH, z_SWH))

    # Transform desired end-effector trajectory from body frame to leg base frames
    P_FL_base_SWH = np.array([T0_B(P_FL_body_SWH[:, i].reshape((3, 1)), 'FL') for i in range(len(t_SWH))])
    P_FR_base_SWH = np.array([T0_B(P_FR_body_SWH[:, i].reshape((3, 1)), 'FR') for i in range(len(t_SWH))])
    P_HL_base_SWH = np.array([T0_B(P_HL_body_SWH[:, i].reshape((3, 1)), 'HL') for i in range(len(t_SWH))])
    P_HR_base_SWH = np.array([T0_B(P_HR_body_SWH[:, i].reshape((3, 1)), 'HR') for i in range(len(t_SWH))])

    # Combine body frame trajectory cartesian velocities into array
    V_body_SWH = np.vstack((x_dot_SWH, y_dot_SWH, z_dot_SWH))
    # Transform desired end-effector velocity from body frame to leg base frames
    V_FL_base_SWH = np.array([R0_B(V_body_SWH[:, i].reshape((3, 1)), 'FL') for i in range(len(t_SWH))])
    V_FR_base_SWH = np.array([R0_B(V_body_SWH[:, i].reshape((3, 1)), 'FR') for i in range(len(t_SWH))])
    V_HL_base_SWH = np.array([R0_B(V_body_SWH[:, i].reshape((3, 1)), 'HL') for i in range(len(t_SWH))])
    V_HR_base_SWH = np.array([R0_B(V_body_SWH[:, i].reshape((3, 1)), 'HR') for i in range(len(t_SWH))])

    ### Kinematics ###
    # Determine joint angles for all 4 legs using inverse kinematics
    Theta_FL_SWH = np.array([Inverse_Kinematics(P_FL_base_SWH[i], 'FL') for i in range(len(t_SWH))]) # Shape (num_time_steps, 3), containing theta1, theta2, theta3 for each time step
    Theta_FR_SWH = np.array([Inverse_Kinematics(P_FR_base_SWH[i], 'FR') for i in range(len(t_SWH))]) # Shape (num_time_steps, 3), containing theta1, theta2, theta3 for each time step
    Theta_HL_SWH = np.array([Inverse_Kinematics(P_HL_base_SWH[i], 'HL') for i in range(len(t_SWH))]) # Shape (num_time_steps, 3), containing theta1, theta2, theta3 for each time step
    Theta_HR_SWH = np.array([Inverse_Kinematics(P_HR_base_SWH[i], 'HR') for i in range(len(t_SWH))]) # Shape (num_time_steps, 3), containing theta1, theta2, theta3 for each time step

    # Determine joint velocities for all 4 legs using Jacobian
    # Damped least squares inverse to avoid singularities - theta_dot = (J^T*J + damp^2*I)^-1 * J^T * cartesian_velocity 
    Theta_dot_FL_SWH = np.zeros((3, len(t_SWH)))  # Initialize joint velocity array
    Theta_dot_FR_SWH = np.zeros((3, len(t_SWH)))  # Initialize joint velocity array
    Theta_dot_HL_SWH = np.zeros((3, len(t_SWH)))  # Initialize joint velocity array
    Theta_dot_HR_SWH = np.zeros((3, len(t_SWH)))  # Initialize joint velocity array
    damp = 0.001  # Damping factor
    for i in range(len(t_SWH)):
        Jac_i_FL = Jacobian(Theta_FL_SWH[i, 0], Theta_FL_SWH[i, 1], Theta_FL_SWH[i, 2], 'FL')
        Jac_i_FR = Jacobian(Theta_FR_SWH[i, 0], Theta_FR_SWH[i, 1], Theta_FR_SWH[i, 2], 'FR')
        Jac_i_HL = Jacobian(Theta_HL_SWH[i, 0], Theta_HL_SWH[i, 1], Theta_HL_SWH[i, 2], 'HL')
        Jac_i_HR = Jacobian(Theta_HR_SWH[i, 0], Theta_HR_SWH[i, 1], Theta_HR_SWH[i, 2], 'HR')
        JT_FL = Jac_i_FL.T
        JT_FR = Jac_i_FR.T
        JT_HL = Jac_i_HL.T
        JT_HR = Jac_i_HR.T
        term_FL = JT_FL @ Jac_i_FL + (damp**2)*np.eye(3)
        term_FR = JT_FR @ Jac_i_FR + (damp**2)*np.eye(3)
        term_HL = JT_HL @ Jac_i_HL + (damp**2)*np.eye(3)
        term_HR = JT_HR @ Jac_i_HR + (damp**2)*np.eye(3)
        Theta_dot_FL_SWH[:, i] = np.linalg.solve(term_FL, JT_FL @ V_FL_base_SWH[i].flatten())
        Theta_dot_FR_SWH[:, i] = np.linalg.solve(term_FR, JT_FR @ V_FR_base_SWH[i].flatten())
        Theta_dot_HL_SWH[:, i] = np.linalg.solve(term_HL, JT_HL @ V_HL_base_SWH[i].flatten())
        Theta_dot_HR_SWH[:, i] = np.linalg.solve(term_HR, JT_HR @ V_HR_base_SWH[i].flatten())

    # Convert joint angles and velocities to degrees and abs(degrees/s) for right units for motor control
    Theta_FL_SWH = np.rad2deg(Theta_FL_SWH)
    Theta_FR_SWH = np.rad2deg(Theta_FR_SWH)
    Theta_HL_SWH = np.rad2deg(Theta_HL_SWH)
    Theta_HR_SWH = np.rad2deg(Theta_HR_SWH)
    Theta_dot_FL_SWH = np.abs(np.rad2deg(Theta_dot_FL_SWH))
    Theta_dot_FR_SWH = np.abs(np.rad2deg(Theta_dot_FR_SWH))
    Theta_dot_HL_SWH = np.abs(np.rad2deg(Theta_dot_HL_SWH))
    Theta_dot_HR_SWH = np.abs(np.rad2deg(Theta_dot_HR_SWH))

    # ----------------------------- #
    ### WALK HEIGHT TO WALK START GENERATION ###
    # ----------------------------- #

    # Trajectory generation
    t_Stand_To_Walk_With_Transfer, FL_Stand_To_Walk, FR_Stand_To_Walk, HL_Stand_To_Walk, HR_Stand_To_Walk, FL_Stand_To_Walk_Velocities, FR_Stand_To_Walk_Velocities, HL_Stand_To_Walk_Velocities, HR_Stand_To_Walk_Velocities = Building_Stand_To_Walk_Trajectories(
                                        STW_Swing_Time_Scalar, 
                                        STW_Swing_Time_Steps,
                                        PHASE_OFFSET, 
                                        FL_Bezier_Trajectory_With_Transfer[0, 0], 
                                        FR_Bezier_Trajectory_With_Transfer[0, 0], 
                                        HL_Bezier_Trajectory_With_Transfer[0, 0], 
                                        HR_Bezier_Trajectory_With_Transfer[0, 0])
    total_time_STW_with_transfer = t_Stand_To_Walk_With_Transfer[-1]

    ### TRANSFORMATIONS ###
    # Transform the stand to walk trajectory from Body Frame to Leg Base Frames
    P_FL_Base_STW = np.array([T0_B(FL_Stand_To_Walk[:, i].reshape((3, 1)), 'FL') for i in range((len(t_Stand_To_Walk_With_Transfer)))])
    P_FR_Base_STW = np.array([T0_B(FR_Stand_To_Walk[:, i].reshape((3, 1)), 'FR') for i in range((len(t_Stand_To_Walk_With_Transfer)))])
    P_HL_Base_STW = np.array([T0_B(HL_Stand_To_Walk[:, i].reshape((3, 1)), 'HL') for i in range((len(t_Stand_To_Walk_With_Transfer)))])
    P_HR_Base_STW = np.array([T0_B(HR_Stand_To_Walk[:, i].reshape((3, 1)), 'HR') for i in range((len(t_Stand_To_Walk_With_Transfer)))])

    # Transform desired end-effector velocity from body frame to leg base frames
    V_FL_base_STW = np.array([R0_B(FL_Stand_To_Walk_Velocities[:, i].reshape((3, 1)), 'FL') for i in range((len(t_Stand_To_Walk_With_Transfer)))])
    V_FR_base_STW = np.array([R0_B(FR_Stand_To_Walk_Velocities[:, i].reshape((3, 1)), 'FR') for i in range((len(t_Stand_To_Walk_With_Transfer)))])
    V_HL_base_STW = np.array([R0_B(HL_Stand_To_Walk_Velocities[:, i].reshape((3, 1)), 'HL') for i in range((len(t_Stand_To_Walk_With_Transfer)))])
    V_HR_base_STW = np.array([R0_B(HR_Stand_To_Walk_Velocities[:, i].reshape((3, 1)), 'HR') for i in range((len(t_Stand_To_Walk_With_Transfer)))])

    ### Kinematics ###
    # Determine joint angles for all 4 legs using inverse kinematics
    Theta_FL_STW = np.array([Inverse_Kinematics(P_FL_Base_STW[i], 'FL') for i in range((len(t_Stand_To_Walk_With_Transfer)))]) # Shape (num_time_steps, 3), containing theta1, theta2, theta3 for each time step
    Theta_FR_STW = np.array([Inverse_Kinematics(P_FR_Base_STW[i], 'FR') for i in range((len(t_Stand_To_Walk_With_Transfer)))]) # Shape (num_time_steps, 3), containing theta1, theta2, theta3 for each time step
    Theta_HL_STW = np.array([Inverse_Kinematics(P_HL_Base_STW[i], 'HL') for i in range((len(t_Stand_To_Walk_With_Transfer)))]) # Shape (num_time_steps, 3), containing theta1, theta2, theta3 for each time step
    Theta_HR_STW = np.array([Inverse_Kinematics(P_HR_Base_STW[i], 'HR') for i in range((len(t_Stand_To_Walk_With_Transfer)))]) # Shape (num_time_steps, 3), containing theta1, theta2, theta3 for each time step

    # Determine joint velocities for all 4 legs using Jacobian
    # Damped least squares inverse to avoid singularities - theta_dot = (J^T*J + damp^2*I)^-1 * J^T * cartesian_velocity 
    Theta_dot_FL_STW = np.zeros((3, len(t_Stand_To_Walk_With_Transfer)))  # Initialize joint velocity array
    Theta_dot_FR_STW = np.zeros((3, len(t_Stand_To_Walk_With_Transfer)))  # Initialize joint velocity array
    Theta_dot_HL_STW = np.zeros((3, len(t_Stand_To_Walk_With_Transfer)))  # Initialize joint velocity array
    Theta_dot_HR_STW = np.zeros((3, len(t_Stand_To_Walk_With_Transfer)))  # Initialize joint velocity array
    damp = 0.001  # Damping factor
    for i in range((int(total_time_STW_with_transfer / dt))):
        Jac_i_FL = Jacobian(Theta_FL_STW[i, 0], Theta_FL_STW[i, 1], Theta_FL_STW[i, 2], 'FL')
        Jac_i_FR = Jacobian(Theta_FR_STW[i, 0], Theta_FR_STW[i, 1], Theta_FR_STW[i, 2], 'FR')
        Jac_i_HL = Jacobian(Theta_HL_STW[i, 0], Theta_HL_STW[i, 1], Theta_HL_STW[i, 2], 'HL')
        Jac_i_HR = Jacobian(Theta_HR_STW[i, 0], Theta_HR_STW[i, 1], Theta_HR_STW[i, 2], 'HR')
        JT_FL = Jac_i_FL.T
        JT_FR = Jac_i_FR.T
        JT_HL = Jac_i_HL.T
        JT_HR = Jac_i_HR.T
        term_FL = JT_FL @ Jac_i_FL + (damp**2)*np.eye(3)
        term_FR = JT_FR @ Jac_i_FR + (damp**2)*np.eye(3)
        term_HL = JT_HL @ Jac_i_HL + (damp**2)*np.eye(3)
        term_HR = JT_HR @ Jac_i_HR + (damp**2)*np.eye(3)
        Theta_dot_FL_STW[:, i] = np.linalg.solve(term_FL, JT_FL @ V_FL_base_STW[i].flatten())
        Theta_dot_FR_STW[:, i] = np.linalg.solve(term_FR, JT_FR @ V_FR_base_STW[i].flatten())
        Theta_dot_HL_STW[:, i] = np.linalg.solve(term_HL, JT_HL @ V_HL_base_STW[i].flatten())
        Theta_dot_HR_STW[:, i] = np.linalg.solve(term_HR, JT_HR @ V_HR_base_STW[i].flatten())

    # Convert joint angles and velocities to degrees and abs(degrees/s) for right units for motor control
    Theta_FL_STW = np.rad2deg(Theta_FL_STW)
    Theta_FR_STW = np.rad2deg(Theta_FR_STW)
    Theta_HL_STW = np.rad2deg(Theta_HL_STW)
    Theta_HR_STW = np.rad2deg(Theta_HR_STW)
    Theta_dot_FL_STW = np.abs(np.rad2deg(Theta_dot_FL_STW))
    Theta_dot_FR_STW = np.abs(np.rad2deg(Theta_dot_FR_STW))
    Theta_dot_HL_STW = np.abs(np.rad2deg(Theta_dot_HL_STW))
    Theta_dot_HR_STW = np.abs(np.rad2deg(Theta_dot_HR_STW))

    # ----------------------------- #
    ### DIRECTION ###
    # ----------------------------- #
    # If Direction is forward, no changes needed as trajectories are generated for forward direction
    # If Direction is backward, FL and HR are swapped and FR and HL are swapped for all arrays used for control
    if Direction == "Backward":
        # Walk
        Theta_FL, Theta_FR, Theta_HL, Theta_HR = Theta_HR, Theta_HL, Theta_FR, Theta_FL
        Theta_dot_FL, Theta_dot_FR, Theta_dot_HL, Theta_dot_HR = Theta_dot_HR, Theta_dot_HL, Theta_dot_FR, Theta_dot_FL

        # Stand height to walk height
        Theta_FL_SWH, Theta_FR_SWH, Theta_HL_SWH, Theta_HR_SWH = Theta_HR_SWH, Theta_HL_SWH, Theta_FR_SWH, Theta_FL_SWH
        Theta_dot_FL_SWH, Theta_dot_FR_SWH, Theta_dot_HL_SWH, Theta_dot_HR_SWH = Theta_dot_HR_SWH, Theta_dot_HL_SWH, Theta_dot_FR_SWH, Theta_dot_FL_SWH

        # Walk height to walk start
        Theta_FL_STW, Theta_FR_STW, Theta_HL_STW, Theta_HR_STW = Theta_HR_STW, Theta_HL_STW, Theta_FR_STW, Theta_FL_STW
        Theta_dot_FL_STW, Theta_dot_FR_STW, Theta_dot_HL_STW, Theta_dot_HR_STW = Theta_dot_HR_STW, Theta_dot_HL_STW, Theta_dot_FR_STW, Theta_dot_FL_STW

    # ----------------------------- #
    ### RETURN ARRAYS ###
    # ----------------------------- #
    # Create arrays for return to stand pose using reversed trajectories (Note that forward and backward direction is already handled in previous section)
    # Walk height to stand height
    Theta_FL_SWH_Return = Theta_FL_SWH[::-1, :]
    Theta_FR_SWH_Return = Theta_FR_SWH[::-1, :]
    Theta_HL_SWH_Return = Theta_HL_SWH[::-1, :]
    Theta_HR_SWH_Return = Theta_HR_SWH[::-1, :]
    Theta_dot_FL_SWH_Return = Theta_dot_FL_SWH[:, ::-1]
    Theta_dot_FR_SWH_Return = Theta_dot_FR_SWH[:, ::-1]
    Theta_dot_HL_SWH_Return = Theta_dot_HL_SWH[:, ::-1]
    Theta_dot_HR_SWH_Return = Theta_dot_HR_SWH[:, ::-1]

    # Walk start to walk height
    Theta_FL_STW_Return = Theta_FL_STW[::-1, :]
    Theta_FR_STW_Return = Theta_FR_STW[::-1, :]
    Theta_HL_STW_Return = Theta_HL_STW[::-1, :]
    Theta_HR_STW_Return = Theta_HR_STW[::-1, :]
    Theta_dot_FL_STW_Return = Theta_dot_FL_STW[:, ::-1]
    Theta_dot_FR_STW_Return = Theta_dot_FR_STW[:, ::-1]
    Theta_dot_HL_STW_Return = Theta_dot_HL_STW[:, ::-1]
    Theta_dot_HR_STW_Return = Theta_dot_HR_STW[:, ::-1]

    ### PRE-LOOP ###   
    # Pre-loop sequence to move to initial walk position in a reasonable way
    print("Pre-computations complete, executing pre-loop sequence for BEZIER WALK GAIT state...")
    print("Moving to walk height...")

    # Writing up/down PI parameters for transition from stand height to walk height
    PID_RAM_Control(bus0,ID_1, pi_up_down)
    PID_RAM_Control(bus0,ID_2, pi_up_down)
    PID_RAM_Control(bus0,ID_3, pi_up_down)
    PID_RAM_Control(bus1,ID_1, pi_up_down)
    PID_RAM_Control(bus1,ID_2, pi_up_down)
    PID_RAM_Control(bus1,ID_3, pi_up_down)
    PID_RAM_Control(bus2,ID_1, pi_up_down)
    PID_RAM_Control(bus2,ID_2, pi_up_down)
    PID_RAM_Control(bus2,ID_3, pi_up_down)
    PID_RAM_Control(bus3,ID_1, pi_up_down)
    PID_RAM_Control(bus3,ID_2, pi_up_down)
    PID_RAM_Control(bus3,ID_3, pi_up_down)
    time.sleep(0.1) # Short sleep to ensure PI parameters are written before sending position commands

    SWH_Statement = True
    # Note start time
    start_time = cycle_start = current_time = time.monotonic()
    # Run until transition is done
    while SWH_Statement:
        # Time Management
        current_time = time.monotonic()
        elapsed_cycle = current_time - cycle_start
        if elapsed_cycle >= total_time_SWH:
            SWH_Statement = False
            continue

        # Find closest value in t to elapsed in current cycle
        index = min(int(elapsed_cycle / dt), len(t_SWH) - 1)

        # Send position control commands to motors for current time step
        Position_Control(bus0, ID_1, Theta_FL_SWH[index, 0], Theta_dot_FL_SWH[0, index])
        Position_Control(bus0, ID_2, Theta_FL_SWH[index, 1], Theta_dot_FL_SWH[1, index])
        Position_Control(bus0, ID_3, Theta_FL_SWH[index, 2], Theta_dot_FL_SWH[2, index])
        Position_Control(bus1, ID_1, Theta_FR_SWH[index, 0], Theta_dot_FR_SWH[0, index])
        Position_Control(bus1, ID_2, Theta_FR_SWH[index, 1], Theta_dot_FR_SWH[1, index])
        Position_Control(bus1, ID_3, Theta_FR_SWH[index, 2], Theta_dot_FR_SWH[2, index])
        Position_Control(bus2, ID_1, Theta_HL_SWH[index, 0], Theta_dot_HL_SWH[0, index])
        Position_Control(bus2, ID_2, Theta_HL_SWH[index, 1], Theta_dot_HL_SWH[1, index])
        Position_Control(bus2, ID_3, Theta_HL_SWH[index, 2], Theta_dot_HL_SWH[2, index])
        Position_Control(bus3, ID_1, Theta_HR_SWH[index, 0], Theta_dot_HR_SWH[0, index])
        Position_Control(bus3, ID_2, Theta_HR_SWH[index, 1], Theta_dot_HR_SWH[1, index])
        Position_Control(bus3, ID_3, Theta_HR_SWH[index, 2], Theta_dot_HR_SWH[2, index])

    time.sleep(1) # Sleep for a short time to ensure transition to walk height is complete before continuing, adjust as needed

    # Move to walk start position
    print("Moved to walk height, moving to walk start position...")

    print("Writing PI parameters to motors...")
    PID_RAM_Control(bus0,ID_1, pi_bezier_walk)
    PID_RAM_Control(bus0,ID_2, pi_bezier_walk)
    PID_RAM_Control(bus0,ID_3, pi_bezier_walk)
    PID_RAM_Control(bus1,ID_1, pi_bezier_walk)
    PID_RAM_Control(bus1,ID_2, pi_bezier_walk)
    PID_RAM_Control(bus1,ID_3, pi_bezier_walk)
    PID_RAM_Control(bus2,ID_1, pi_bezier_walk)  
    PID_RAM_Control(bus2,ID_2, pi_bezier_walk)
    PID_RAM_Control(bus2,ID_3, pi_bezier_walk)
    PID_RAM_Control(bus3,ID_1, pi_bezier_walk)
    PID_RAM_Control(bus3,ID_2, pi_bezier_walk)
    PID_RAM_Control(bus3,ID_3, pi_bezier_walk)
    time.sleep(0.1) # Sleep for a short time to ensure parameters are written before starting loop, adjust as needed

    STW_Statement = True
    # Note start time
    start_time = cycle_start = current_time = time.monotonic()
    # Run until transition is done
    while STW_Statement:
        # Time Management
        current_time = time.monotonic()
        elapsed_cycle = current_time - cycle_start
        if elapsed_cycle >= total_time_STW_with_transfer:
            STW_Statement = False
            continue

        # Find closest value in t to elapsed in current cycle
        index = min(int(elapsed_cycle / dt), len(t_Stand_To_Walk_With_Transfer) - 1)

        # Send position control commands to motors for current time step
        Position_Control(bus0, ID_1, Theta_FL_STW[index, 0], Theta_dot_FL_STW[0, index])
        Position_Control(bus0, ID_2, Theta_FL_STW[index, 1], Theta_dot_FL_STW[1, index])
        Position_Control(bus0, ID_3, Theta_FL_STW[index, 2], Theta_dot_FL_STW[2, index])
        Position_Control(bus1, ID_1, Theta_FR_STW[index, 0], Theta_dot_FR_STW[0, index])
        Position_Control(bus1, ID_2, Theta_FR_STW[index, 1], Theta_dot_FR_STW[1, index])
        Position_Control(bus1, ID_3, Theta_FR_STW[index, 2], Theta_dot_FR_STW[2, index])
        Position_Control(bus2, ID_1, Theta_HL_STW[index, 0], Theta_dot_HL_STW[0, index])
        Position_Control(bus2, ID_2, Theta_HL_STW[index, 1], Theta_dot_HL_STW[1, index])
        Position_Control(bus2, ID_3, Theta_HL_STW[index, 2], Theta_dot_HL_STW[2, index])
        Position_Control(bus3, ID_1, Theta_HR_STW[index, 0], Theta_dot_HR_STW[0, index])
        Position_Control(bus3, ID_2, Theta_HR_STW[index, 1], Theta_dot_HR_STW[1, index])
        Position_Control(bus3, ID_3, Theta_HR_STW[index, 2], Theta_dot_HR_STW[2, index])

    time.sleep(1) # Sleep for a short time to ensure transition to walk start position is complete before starting loop, adjust as needed

    # Main loop for BEZIER WALK GAIT state
    print("Pre-loop sequence complete, starting BEZIER WALK GAIT loop...")
    print("Press 'S' to switch to standing pose - Press ctrl+c in terminal for shutdown")

    # Note start time
    start_time = cycle_start = current_time = time.monotonic()
    try:
        while True:
            # Loop time managment (if needed)
            current_time = time.monotonic()
            elapsed_cycle = current_time - cycle_start # Elapsed time in current cycle
            #elapsed_total = current_time - start_time  # Elapsed time since start of program
            # Check if current cycle is over -> start new cycle
            if elapsed_cycle >= total_time_with_transfer:
                cycle_start += total_time_with_transfer # Force next cycle start time to be exactly total trajectory time after previous cycle start time to avoid drift
                # Check if state has been changed, if so exit this state function to switch to new state
                with lock:
                    if State != "BEZIER WALK GAIT" and State != "REVERSE BEZIER WALK GAIT":  # Check if state has been changed, if so exit this state function to switch to new state
                        print("State change detected, exiting WALK state function")
                        break
                continue

            # Find closest value in t to elapsed in current cyclew
            index = min(int(elapsed_cycle / dt), len(t_with_transfer) - 1)
            
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

    except KeyboardInterrupt:
        raise # Just raise the exception to be caught in the main loop try-except block for shutdown
    
    # Move to walk height
    print("Moving to walk height...")

    STW_Statement = True
    # Note start time
    start_time = cycle_start = current_time = time.monotonic()
    # Run until transition is done
    while STW_Statement:
        # Time Management
        current_time = time.monotonic()
        elapsed_cycle = current_time - cycle_start
        if elapsed_cycle >= total_time_STW_with_transfer:
            STW_Statement = False
            continue

        # Find closest value in t to elapsed in current cycle
        index = min(int(elapsed_cycle / dt), len(t_Stand_To_Walk_With_Transfer) - 1)

        # Send position control commands to motors for current time step
        Position_Control(bus0, ID_1, Theta_FL_STW_Return[index, 0], Theta_dot_FL_STW_Return[0, index])
        Position_Control(bus0, ID_2, Theta_FL_STW_Return[index, 1], Theta_dot_FL_STW_Return[1, index])
        Position_Control(bus0, ID_3, Theta_FL_STW_Return[index, 2], Theta_dot_FL_STW_Return[2, index])
        Position_Control(bus1, ID_1, Theta_FR_STW_Return[index, 0], Theta_dot_FR_STW_Return[0, index])
        Position_Control(bus1, ID_2, Theta_FR_STW_Return[index, 1], Theta_dot_FR_STW_Return[1, index])
        Position_Control(bus1, ID_3, Theta_FR_STW_Return[index, 2], Theta_dot_FR_STW_Return[2, index])
        Position_Control(bus2, ID_1, Theta_HL_STW_Return[index, 0], Theta_dot_HL_STW_Return[0, index])
        Position_Control(bus2, ID_2, Theta_HL_STW_Return[index, 1], Theta_dot_HL_STW_Return[1, index])
        Position_Control(bus2, ID_3, Theta_HL_STW_Return[index, 2], Theta_dot_HL_STW_Return[2, index])
        Position_Control(bus3, ID_1, Theta_HR_STW_Return[index, 0], Theta_dot_HR_STW_Return[0, index])
        Position_Control(bus3, ID_2, Theta_HR_STW_Return[index, 1], Theta_dot_HR_STW_Return[1, index])
        Position_Control(bus3, ID_3, Theta_HR_STW_Return[index, 2], Theta_dot_HR_STW_Return[2, index])

    time.sleep(1) # Sleep for a short time to ensure transition to stand height position is complete before continuing, adjust as needed

    print("Moved to walk height, moving to standing pose")
    # Writing up/down PI parameters for transition from walk height to stand height
    PID_RAM_Control(bus0,ID_1, pi_up_down)
    PID_RAM_Control(bus0,ID_2, pi_up_down)
    PID_RAM_Control(bus0,ID_3, pi_up_down)
    PID_RAM_Control(bus1,ID_1, pi_up_down)
    PID_RAM_Control(bus1,ID_2, pi_up_down)
    PID_RAM_Control(bus1,ID_3, pi_up_down)
    PID_RAM_Control(bus2,ID_1, pi_up_down)
    PID_RAM_Control(bus2,ID_2, pi_up_down)
    PID_RAM_Control(bus2,ID_3, pi_up_down)
    PID_RAM_Control(bus3,ID_1, pi_up_down)
    PID_RAM_Control(bus3,ID_2, pi_up_down)
    PID_RAM_Control(bus3,ID_3, pi_up_down)
    time.sleep(0.1) # Short sleep to ensure PI parameters are written before sending position commands

    SWH_Statement = True
    # Note start time
    start_time = cycle_start = current_time = time.monotonic()
    # Run until transition is done
    while SWH_Statement:
        # Time Management
        current_time = time.monotonic()
        elapsed_cycle = current_time - cycle_start
        if elapsed_cycle >= total_time_SWH:
            SWH_Statement = False
            continue

        # Find closest value in t to elapsed in current cycle
        index = min(int(elapsed_cycle / dt), len(t_SWH) - 1)

        # Send position control commands to motors for current time step
        Position_Control(bus0, ID_1, Theta_FL_SWH_Return[index, 0], Theta_dot_FL_SWH_Return[0, index])
        Position_Control(bus0, ID_2, Theta_FL_SWH_Return[index, 1], Theta_dot_FL_SWH_Return[1, index])
        Position_Control(bus0, ID_3, Theta_FL_SWH_Return[index, 2], Theta_dot_FL_SWH_Return[2, index])
        Position_Control(bus1, ID_1, Theta_FR_SWH_Return[index, 0], Theta_dot_FR_SWH_Return[0, index])
        Position_Control(bus1, ID_2, Theta_FR_SWH_Return[index, 1], Theta_dot_FR_SWH_Return[1, index])
        Position_Control(bus1, ID_3, Theta_FR_SWH_Return[index, 2], Theta_dot_FR_SWH_Return[2, index])
        Position_Control(bus2, ID_1, Theta_HL_SWH_Return[index, 0], Theta_dot_HL_SWH_Return[0, index])
        Position_Control(bus2, ID_2, Theta_HL_SWH_Return[index, 1], Theta_dot_HL_SWH_Return[1, index])
        Position_Control(bus2, ID_3, Theta_HL_SWH_Return[index, 2], Theta_dot_HL_SWH_Return[2, index])
        Position_Control(bus3, ID_1, Theta_HR_SWH_Return[index, 0], Theta_dot_HR_SWH_Return[0, index])
        Position_Control(bus3, ID_2, Theta_HR_SWH_Return[index, 1], Theta_dot_HR_SWH_Return[1, index])
        Position_Control(bus3, ID_3, Theta_HR_SWH_Return[index, 2], Theta_dot_HR_SWH_Return[2, index])

    time.sleep(0.1) # Sleep for a short time to ensure transition to stand height is complete before continuing, adjust as needed

    print("Returned to standing pose at the end of BEZIER WALK GAIT state")

### SCRIPT START ###
## PRECOMPUTATIONS ##
print("Performing pre-computations...")

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
        if None in [FL_theta1, FL_theta2, FL_theta3]:
            print("Error reading angles for Front Left leg, skipping printout for this leg")
        else:
            print(f"Joint positions for Front Left:   theta1 = {FL_theta1:1f},  theta2 = {FL_theta2:1f},  theta3 = {FL_theta3:1f}")
        if None in [FR_theta1, FR_theta2, FR_theta3]:
            print("Error reading angles for Front Right leg, skipping printout for this leg")
        else:
            print(f"Joint positions for Front Right:  theta1 = {FR_theta1:1f},  theta2 = {FR_theta2:1f},  theta3 = {FR_theta3:1f}")
        if None in [HL_theta1, HL_theta2, HL_theta3]:
            print("Error reading angles for Hind Left leg, skipping printout for this leg")
        else:
            print(f"Joint positions for Hind Left:    theta1 = {HL_theta1:1f},  theta2 = {HL_theta2:1f},  theta3 = {HL_theta3:1f}")
        if None in [HR_theta1, HR_theta2, HR_theta3]:
            print("Error reading angles for Hind Right leg, skipping printout for this leg")
        else:
            print(f"Joint positions for Hind Right:   theta1 = {HR_theta1:1f},  theta2 = {HR_theta2:1f},  theta3 = {HR_theta3:1f}")       
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
print("Pre-loop verification sequence complete, writing stand state PI parameters and starting pre-loop movement sequence...")

# Write stand state PI parameters
PID_RAM_Control(bus0,ID_1, pi_stand)
PID_RAM_Control(bus0,ID_2, pi_stand)
PID_RAM_Control(bus0,ID_3, pi_stand)
PID_RAM_Control(bus1,ID_1, pi_stand)
PID_RAM_Control(bus1,ID_2, pi_stand)
PID_RAM_Control(bus1,ID_3, pi_stand)
PID_RAM_Control(bus2,ID_1, pi_stand)  
PID_RAM_Control(bus2,ID_2, pi_stand)
PID_RAM_Control(bus2,ID_3, pi_stand)
PID_RAM_Control(bus3,ID_1, pi_stand)
PID_RAM_Control(bus3,ID_2, pi_stand)
PID_RAM_Control(bus3,ID_3, pi_stand)

time.sleep(1) # Wait for 1 second to ensure PID parameters are written before starting movement sequence

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

time.sleep(8)

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

time.sleep(4)

print("Pre-loop sequence complete, starting loop...")

## MAIN LOOP ##
# Starting listener for Main loop
main_listener = keyboard.Listener(on_press=main_on_press)
main_listener.start()  # Start the listener in a separate thread

print("Loop started - Press ctrl+c in terminal for shutdown")

try:
    while True:
        # Print loop info
        print("Current state: Standing pose - Loop running - Press ctrl+c in terminal for shutdown")

        # Set returned from state flag
        returned_from_state = False

        # Check battery voltage percentage and if low switch to low battery state to indicating need for recharge
        _, Voltage_Percentage = Battery_Voltage(bus0, ID_1) # Read battery voltage percentage from one of the motors (assuming all motors have the same battery voltage)
        if Voltage_Percentage is None:
            print("Error reading battery voltage percentage, skipping battery check for this iteration")
        elif Voltage_Percentage <= 20:
            with lock:
                State = "LOW BATTERY"
            Low_Battery_State(bus0, bus1, bus2, bus3, ID_1, ID_2, ID_3)

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
            returned_from_state = True
        elif current_state == "BEZIER WALK GAIT":
            Bezier_Walk_State(bus0, bus1, bus2, bus3, ID_1, ID_2, ID_3, Direction="Forward")
            with lock:
                State = "STAND" # After finishing the state function, switch back to standing pose state
            returned_from_state = True
        elif current_state == "REVERSE BEZIER WALK GAIT":
            Bezier_Walk_State(bus0, bus1, bus2, bus3, ID_1, ID_2, ID_3, Direction="Backward")
            with lock:
                State = "STAND" # After finishing the state function, switch back to standing pose state
            returned_from_state = True
        # add more states here as needed with elif statements

        # If returned from any state: Update PI parameters to be for stand state
        if returned_from_state:
            PID_RAM_Control(bus0,ID_1, pi_stand)
            PID_RAM_Control(bus0,ID_2, pi_stand)
            PID_RAM_Control(bus0,ID_3, pi_stand)
            PID_RAM_Control(bus1,ID_1, pi_stand)
            PID_RAM_Control(bus1,ID_2, pi_stand)
            PID_RAM_Control(bus1,ID_3, pi_stand)
            PID_RAM_Control(bus2,ID_1, pi_stand)  
            PID_RAM_Control(bus2,ID_2, pi_stand)
            PID_RAM_Control(bus2,ID_3, pi_stand)
            PID_RAM_Control(bus3,ID_1, pi_stand)
            PID_RAM_Control(bus3,ID_2, pi_stand)
            PID_RAM_Control(bus3,ID_3, pi_stand)

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