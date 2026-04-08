

# Imports
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..'))) # Change level of path based on file location (the ../../)

import can
import time
import numpy as np
import threading
import csv

from Robot import*

def can_listener(bus, bus_label):
    # Thread function to continuously receive CAN messages and update latest_status
    while running:
        for motor_id in [ID_1, ID_2, ID_3]:
            Torque_Current, Torque = Read_Torque_Current(bus,motor_id, debug = False)
            if Torque_Current is None:
                continue  # Skip updating if no response received
            
            with lock:
                latest_status[(bus_label, motor_id)] = (Torque_Current, Torque)


print("Performing pre-computations...")
# Define kinematic body lengths
l_k = 0.7048  # Length of body in kinematic model (meters)
w_k = 0.220   # Width of body in kinematic model (meters)

# Define desired end-effector position for standing posture (z=-0.41 meters) for all 4 legs
x_FL = x_FR = l_k/2 # x-position of front legs
x_HL = x_HR = -l_k/2 # x-position of hind legs
y_FL = y_HL = w_k/2 + 0.078 # y-position of left legs
y_FR = y_HR = -w_k/2 - 0.078 # y-position of right legs
z = -0.41 # z-position for standing posture (similar for all legs)

# Combine trajectories into position arrays for each leg
P_FL_body = np.array([x_FL, y_FL, z])
P_FR_body = np.array([x_FR, y_FR, z])
P_HL_body = np.array([x_HL, y_HL, z])
P_HR_body = np.array([x_HR, y_HR, z])

# Transform desired end-effector trajectory from body frame to leg base frames
P_FL_base = T0_B(P_FL_body.reshape((3,1)), 'FL')
P_FR_base = T0_B(P_FR_body.reshape((3,1)), 'FR')
P_HL_base = T0_B(P_HL_body.reshape((3,1)), 'HL')
P_HR_base = T0_B(P_HR_body.reshape((3,1)), 'HR')

# Determine joint angles for all 4 legs using inverse kinematics
Theta_FL = Inverse_Kinematics(P_FL_base, 'FL')
Theta_FR = Inverse_Kinematics(P_FR_base, 'FR')
Theta_HL = Inverse_Kinematics(P_HL_base, 'HL')
Theta_HR = Inverse_Kinematics(P_HR_base, 'HR')

# Convert angles from radians to degrees for motor control
Theta_FL = np.degrees(Theta_FL)
Theta_FR = np.degrees(Theta_FR)
Theta_HL = np.degrees(Theta_HL)
Theta_HR = np.degrees(Theta_HR)

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
            
# Define threading variables
latest_status = {
    ("bus0", ID_1): None,  ("bus0", ID_2): None,  ("bus0", ID_3): None,
    ("bus1", ID_1): None,  ("bus1", ID_2): None,  ("bus1", ID_3): None,
    ("bus2", ID_1): None,  ("bus2", ID_2): None,  ("bus2", ID_3): None,
    ("bus3", ID_1): None,  ("bus3", ID_2): None,  ("bus3", ID_3): None,
}
lock = threading.Lock()
running = True

# Initialize receiver thread
thread0 = threading.Thread(target=can_listener, args=(bus0, "bus0"), daemon=True)
thread1 = threading.Thread(target=can_listener, args=(bus1, "bus1"), daemon=True)
thread2 = threading.Thread(target=can_listener, args=(bus2, "bus2"), daemon=True)
thread3 = threading.Thread(target=can_listener, args=(bus3, "bus3"), daemon=True)
thread0.start()
thread1.start()
thread2.start()
thread3.start()

### Initiating Data Logging
## Initiating Data Logging — By Making a Unique file
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
log_dir = os.path.join(SCRIPT_DIR, "TEST_DATA")
os.makedirs(log_dir, exist_ok=True)
log_filename = os.path.join(log_dir, f"Stand_Pose_Balance_TEST_IMU_Log_{time.strftime('%Y-%m-%d_%H-%M-%S')}.csv")

# Writing the header line
with open(log_filename, 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow(["Timestamp (s)", "bus0 ID_1", "bus0 ID_2", "bus0 ID_3"])
        
# Preallocate data storage array
data = np.zeros((1500000,4))  # Adjust size as needed (current run for max of 83.33 minutes at 200 Hz)
data_count = 0


print("Initialization complete, moving to zero position")
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

time.sleep(5)

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
start_time = cycle_start = current_time = time.monotonic()

try:
    while True:
        # Loop time managment      
        current_time = time.monotonic()
        elapsed_cycle = current_time - cycle_start # Elapsed time in current cycle
        elapsed_total = current_time - start_time # Elapsed time since start of program
        
        
        # Check if current cycle is over -> start new cycle
        #if elapsed_cycle >= total_time:
        #    cycle_start += total_time # Force next cycle start time to be exactly total trajectory time after previous cycle start time
        #    continue
        
        # find closest value in t to elapsed in current cycle
        #index = min(int(elapsed_cycle / dt), len(t) - 1)

        # Log data using thread
        with lock:
            s_FL1 = latest_status[("bus0", ID_1)]
            s_FL2 = latest_status[("bus0", ID_2)]
            s_FL3 = latest_status[("bus0", ID_3)]
            # Clear after reading so we never log stale data again
            latest_status[("bus0", ID_1)] = None
            latest_status[("bus0", ID_2)] = None
            latest_status[("bus0", ID_3)] = None


        if  s_FL1 is None or s_FL2 is None or s_FL3 is None:
            time.sleep(0.005)
            continue  # skip this cycle until all motors have reported

        _, Torque_1 = s_FL1
        _, Torque_2 = s_FL2
        _, Torque_3 = s_FL3    
            
            
        data[data_count, :] = [elapsed_total, Torque_1, Torque_2, Torque_3]
        data_count += 1

        # Send commands to motors
        Position_Control(bus0,ID_1,Theta_FL[0],20)
        Position_Control(bus0,ID_2,Theta_FL[1],20)
        Position_Control(bus0,ID_3,Theta_FL[2],20)
        Position_Control(bus1,ID_1,Theta_FR[0],20)
        Position_Control(bus1,ID_2,Theta_FR[1],20)
        Position_Control(bus1,ID_3,Theta_FR[2],20)
        Position_Control(bus2,ID_1,Theta_HL[0],20)
        Position_Control(bus2,ID_2,Theta_HL[1],20)
        Position_Control(bus2,ID_3,Theta_HL[2],20)
        Position_Control(bus3,ID_1,Theta_HR[0],20)
        Position_Control(bus3,ID_2,Theta_HR[1],20)
        Position_Control(bus3,ID_3,Theta_HR[2],20)
        time.sleep(0.005) 
        
# Stop loop and store logged data with Ctrl+C
except KeyboardInterrupt:
    print("KeyboardInterrupt received, shutting down...")
    running = False
    thread0.join(timeout=1.0)
    thread1.join(timeout=1.0)
    thread2.join(timeout=1.0)
    thread3.join(timeout=1.0)
    print("Receiver thread has been terminated.")
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
    print("Shutting down CAN buses...")
    bus0.shutdown()
    bus1.shutdown()
    bus2.shutdown()
    bus3.shutdown()
    print("CAN bus shut down")
    print("Storing logged data to file")
    # Store logged data in file
    with open(log_filename,"a") as file:
        for i in range(data_count):
            row = data[i,:]
            file.write(f"{row[0]:.4f},{row[1]:.4f},{row[2]:.4f},{row[3]:.4f}\n")
    print("Logged data stored")
    print("Shutdown complete.")



