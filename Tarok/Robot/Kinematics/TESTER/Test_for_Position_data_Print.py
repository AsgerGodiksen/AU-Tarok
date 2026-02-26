## Qualitative test for position data investigation - using Bezier Femur/Tibia trajectory (arbitrary)
# In a folder containing:
#   FL_Jacobian.py
#   ASGER_METHODS.py
# Remember to calibrate the motors before running this script! (Use 3_motor_calibration.py)

# Used to run arbitrary (Bezier) trajectory for FT front left leg and printing theoretical, read_status and read_angle position at random time instances
# No data logging - only print to console for quick investigation

from ASGER_METHODS import*
import time
import can
import numpy as np
import FL_Jacobian as FLJ
from scipy.special import comb

def Bezier_Curve(c_k, t):
    # Generate single point and velocity along a Bezier curve defined by control points c_k at time t
    # Input time t must be normalized between 0 and 1
    # n Control points are assumed
    # Vectorized version
    # Input:
        # c_k: Control points as a numpy array of shape (n, 2)
        # t: Time instance at which to evaluate the Bezier curve (0 <= t <= 1)
    # Output:
        # pos: Numpy array of shape (2,) containing the trajectory point at time t (cartesian coordinates of end-effector)
        #      (pos[0] = forward position, pos[1] = downward position)
        # vel: Numpy array of shape (2,) containing the instantaneous velocity at time t (cartesian coordinates of end-effector)
        # Note: that the velocity is corresponding to normalized time, to get real velocity multiply with 1/T_swing where T_swing is the duration of the Bezier curve trajectory
    
    n = len(c_k) - 1 # Bernstein polynomial degree
    k = np.arange(0, n + 1) # Indices for control points
    B = comb(n, k) * (t ** k) * ((1 - t) ** (n - k)) # Bernstein basis function for position (degree n)
    Bv = comb(n - 1, k[:-1]) * (t ** k[:-1]) * ((1 - t) ** (n - 1 - k[:-1])) # Bernstein basis function for velocity (degree n-1)
    pos = np.dot(B, c_k) # Bezier point at time t - weighted sum of control points
    vel = n * np.dot(Bv, np.diff(c_k, axis=0)) # Bezier velocity at time t - weighted sum of control point differences
    return pos, vel 

# Inverse Kinematics function
def FL_Inverse_Kinematics(L1,L2,L3,P_org):
    # Input:
        # L1, L2, L3: lengths of the leg segments
        # P_org: Position of the end effector (foot) in the base frame of Left leg
    # Output:
        # theta1, theta2, theta3: joint angles in radians

    # Compute joint angles using inverse kinematics
    x = P_org[0, 0]  # Extract scalar value from numpy array
    y = P_org[1, 0]  # Extract scalar value from numpy array
    z = P_org[2, 0]  # Extract scalar value from numpy array

    # Check workspace
    lower_bound = abs(L2 - L3)
    upper_bound = L2 + L3
    distance = np.sqrt(x**2 + y**2 + z**2 - L1**2)
    if distance < lower_bound or distance > upper_bound:
        raise ValueError("The desired position is outside the reachable workspace of the leg.")

    # Determine horizontal reach in plane of the leg
    H = np.sqrt(x**2 + y**2 - L1**2)

    # Compute theta1
    theta1 = np.arctan2(y, x) - np.arctan2(L1, H)

    # Determine constants needed
    rsqr = H**2 + z**2
    costheta3 = (rsqr - L2**2 - L3**2) / (2 * L2 * L3)

    # Compute theta3
    # (Note: Two possible solutions for theta3, +np.sqrt for elbow out, -np.sqrt for elbow in)
    theta3 = np.arctan2(-np.sqrt(1 - costheta3**2), costheta3)
    
    # Determine constants needed for theta2
    beta = np.arctan2(L3 * np.sin(theta3), L2 + L3 * np.cos(theta3))
    gamma = np.arctan2(-z, H)

    # Compute theta2
    theta2 = gamma - beta
   
    return theta1, theta2, theta3

########## SCRIPT STARTS HERE #######################
# Remember to calibrate the motors before running this script! (Use 3_motor_calibration.py)

##### DEFINE PARAMETERS #####
# Define link lengths
L1 = 0.078  # Length of first segment in meters
L2 = 0.20   # Length of second segment in meters
L3 = 0.30   # Length of third segment in meters

# Define control points for Bezier curve (Scaling of Hyun et al.)
c_kX = 0.6*np.array([-0.200, -0.2805, -0.300, -0.300, -0.300, 0, 0, 0, 0.3032, 0.3032, 0.2826, 0.200]) # [m]
c_kY = 0.16 + 0.6*np.array([0.500, 0.500, 0.3611, 0.3611, 0.3611, 0.3611, 0.3611, 0.3214, 0.3214, 0.3214, 0.500, 0.500]) # [m]
c_k = np.column_stack((c_kX, c_kY))

# Define time parameters (new version - check RUN_v4 for old version)
Swing_time = 3.0 # seconds
Stand_time = 3*Swing_time # seconds
total_time = Swing_time + Stand_time # seconds
dt = 0.005 # seconds (200 Hz)
num_time_steps = int(Swing_time / dt) + 1
t_swing = np.arange(0, Swing_time + dt/2, dt)  # Time array for swing phase
t_stand = np.arange(Swing_time + dt, Swing_time + Stand_time + dt/2, dt)  # Time array for stand phase
t = np.concatenate((t_swing, t_stand))  # Full time array

##### GENERATE TRAJECTORY #####
# Generate swing phase of trajectory (using bezier curve function)
t_bezier = t_swing / Swing_time  # Normalize time to [0, 1] for Bezier curve function
trajectory_swing = np.zeros((num_time_steps, 2))  # Initialize trajectory array
velocity_swing = np.zeros((num_time_steps, 2))    # Initialize velocity array
for i in range(num_time_steps):
    trajectory_swing[i, :], velocity_swing[i, :] = Bezier_Curve(c_k, t_bezier[i])
velocity_swing /= Swing_time  # Convert to real velocity (from normalized time)
# Note: the two above arrays have first column as forward position/velocity and second column as downward position/velocity (this is z and x in leg base frame)

# Generate stand phase of trajectory (leg base frame - not Bezier frame)
z_stand = np.linspace(trajectory_swing[-1, 0], trajectory_swing[0, 0], len(t_stand))         # Linear between end and start z position
x_stand = np.ones_like(t_stand) * trajectory_swing[-1, 1]                                    # Constant x position
z_dot_stand = np.ones_like(t_stand) * (z_stand[1] - z_stand[0]) / (t_stand[1] - t_stand[0])  # Constant z velocity
x_dot_stand = np.zeros_like(t_stand)                                                         # Constant x velocity (zero)

# Combine swing and stand trajectories
z = np.concatenate((trajectory_swing[:, 0], z_stand))        # Z position in meters
x = np.concatenate((trajectory_swing[:, 1], x_stand))        # X position in meters
z_dot = np.concatenate((velocity_swing[:, 0], z_dot_stand))  # Z velocity in meters/second
x_dot = np.concatenate((velocity_swing[:, 1], x_dot_stand))  # X velocity in meters/second

# Generate full y-position trajectory and velocity
y = 0.078*np.ones_like(t)   # Y position in meters (constant)
y_dot = np.zeros_like(t)    # Y velocity in meters/second (constant zero)

##### INVERSE KINEMATICS AND JOINT VELOCITIES ####
# Determine joint angles using inverse kinematics
theta1 = np.zeros_like(t)   # Initialize joint angle arrays
theta2 = np.zeros_like(t)   # Initialize joint angle arrays
theta3 = np.zeros_like(t)   # Initialize joint angle arrays
for i in range(len(t)):
    P_org = np.array([[x[i]], [y[i]], [z[i]]])
    theta1[i], theta2[i], theta3[i] = FL_Inverse_Kinematics(L1, L2, L3, P_org)

# Convert angles from radians to degrees for motor control
theta1_deg = np.rad2deg(theta1)
theta2_deg = np.rad2deg(theta2)
theta3_deg = np.rad2deg(theta3)

# Convert thetai_deg to python scalar integers for motor control
theta1_deg = [int(angle) for angle in theta1_deg]
theta2_deg = [int(angle) for angle in theta2_deg]
theta3_deg = [int(angle) for angle in theta3_deg]

# Determine joint velocities using Jacobian
# Damped Least Squares method - to avoid singularities - theta_dot = (J^T*J + damp^2*I)^-1 * J^T * cartesian_velocity
cartesian_velocity = np.vstack((x_dot, y_dot, z_dot))  # Shape (3, len(t))
theta_dot_DLS = np.zeros((3, len(t)))                  # Initialize joint velocity array
speed = np.zeros((len(t), 3))                          # Initialize speed array for three joints
speed_ALLE = np.zeros((len(t), 3))                  # Initialize speed array for three joints with absolute values (for motor speed commands)
damp = 0.001                                           # Damping factor
for i in range(len(t)):
    Jac_i = FLJ.FL_Jacobian(L1, L2, L3, theta1[i], theta2[i], theta3[i])
    JT = Jac_i.T
    term = JT @ Jac_i + (damp**2)*np.eye(3)
    theta_dot_DLS[:, i] = np.linalg.solve(term, JT @ cartesian_velocity[:, i])
    speed_ALLE[i,:] = (theta_dot_DLS[:, i]) * 9 * 180/np.pi  # Scale factor to convert to motor speed units
    speed[i,:] = np.abs(theta_dot_DLS[:, i]) * 9 * 180/np.pi  # Scale factor to convert to motor speed units 

# Note: speed isE: This script assumes the front left leg and the corresponding leg
# base frame with z-axis pointing forward, x-axis pointing downwards and 
# y-axis pointing to the left
# Remem in degrees/second and multiplied with factor 9 to match motor speed units with 1:9 gearing

# Now we want take [0:3] and get the speed for each joint out
speed1 = speed_ALLE[:,0]
speed2 = speed_ALLE[:,1]
speed3 = speed_ALLE[:,2]

speed1_For_TEST = [int(speed/9) for speed in speed1]
speed2_For_TEST = [int(speed/9) for speed in speed2]
speed3_For_TEST = [int(speed/9) for speed in speed3]


######### Send commands to motors ############
print("Starting CAN bus")
# Define motor IDs (might be specific to chosen physical leg)
ID_1 = 0x141  # Motor for theta1
ID_2 = 0x142  # Motor for theta2
ID_3 = 0x143  # Motor for theta3

# Connect to CAN bus
bus = can.interface.Bus(bustype='socketcan', channel='can1', bitrate=1000000)
bus.flush_tx_buffer()		# Clean any CAN signals on the line
for i in range(100):		# Now Listens for any signals 100 times with 0.01 s in between. Prints if any.
    msg = bus.recv(0.01)
    if msg:
        print(msg)

print("Initialization complete, moving to zero position")

# Move to zero position 
position_control(bus,ID_1,0,200)
position_control(bus,ID_2,0,200)
position_control(bus,ID_3,0,200)

time.sleep(3)

print("Moved to zero position, moving to initial trajectory position")

# Move to initial position
position_control(bus,ID_1,theta1_deg[0],200)
position_control(bus,ID_2,theta2_deg[0],200)
position_control(bus,ID_3,theta3_deg[0],200)

time.sleep(3)

print("Moved to initial trajectory position, starting trajectory execution random printing of position data")
print("Loop started - Press ctrl+c for shutdown")

# Note start time
start_time = cycle_start = current_time = time.monotonic()

try:
    while True:       
        current_time = time.monotonic()
        elapsed_cycle = current_time - cycle_start # Elapsed time in current cycle
        
        # find closest value in t to elapsed in current cycle
        index = min(int(elapsed_cycle / dt), len(t) - 1)

        # Generate random number between 0 and 1
        rand = np.random.rand()

        # Print position data and time if rand is in interval of [0.200:0.201] - this will print at random time instances with an average of 1 print every 5 seconds
        if 0.200 <= rand < 0.201:
            print(f"Time: {elapsed_cycle:.3f}s ")
            print(f"Theoretical Speed (deg/s):  [{speed1_For_TEST[index]:7.2f}, {speed2_For_TEST[index]:7.2f}, {speed3_For_TEST[index]:7.2f}]")
            #print(f"Theoretical Position (deg): [{theta1_deg[index]:7.2f}, {theta2_deg[index]:7.2f}, {theta3_deg[index]:7.2f}] \n")
            read_status_Speed_1= Read_Speed(bus,ID_1)
            read_status_Speed_2= Read_Speed(bus,ID_2)
            read_status_Speed_3= Read_Speed(bus,ID_3)
            #read_angle_1 = read_angle(bus,ID_1)
            #read_angle_2 = read_angle(bus,ID_2)
            #read_angle_3 = read_angle(bus,ID_3)
            #_,Read_Torque_1 = Read_Torque_Current(bus,ID_1)
            #_,Read_Torque_2 = Read_Torque_Current(bus,ID_2)
            #_,Read_Torque_3 = Read_Torque_Current(bus,ID_3)

            print(f"Read status Speed:          [{read_status_Speed_1:7.2f}, {read_status_Speed_2:7.2f}, {read_status_Speed_3:7.2f}]")
            #print(f"Read Torque:                [{read_status_torque_1:7.2f}, {read_status_torque_2:7.2f}, {read_status_torque_3:7.2f}] \n")
            print(" ")

        # Time updated after printing to ensure that the printing time does not affect the timing of the trajectory execution - this way we ensure that the trajectory is executed in real-time and that the printed position data corresponds to the actual position at that time instance (or close to it, considering some delay in reading status and angles)
        current_time = time.monotonic()
        elapsed_cycle = current_time - cycle_start # Elapsed time in current cycle
        
        # Check if current cycle is over -> start new cycle
        if elapsed_cycle >= total_time:
            cycle_start += total_time # Force next cycle start time to be exactly total trajectory time after previous cycle start time
            continue

        # find closest value in t to elapsed in current cycle
        index = min(int(elapsed_cycle / dt), len(t) - 1)

        # Send commands to motors
        position_control(bus,ID_1,theta1_deg[index],int(speed[index,0]))
        position_control(bus,ID_2,theta2_deg[index],int(speed[index,1]))
        position_control(bus,ID_3,theta3_deg[index],int(speed[index,2]))


# Stop loop with Ctrl+C
except KeyboardInterrupt:
    print("KeyboardInterrupt received, shutting down...")
    print("Shutting down CAN bus.")
    bus.flush_tx_buffer()
    bus.shutdown()
    print("CAN bus shut down")
    print("Shutdown complete.")