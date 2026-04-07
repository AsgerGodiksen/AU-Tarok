# This is a template based on Main.py
# Can be used as start for any script that needs to operate with TAROK, such as walking gaits, trajectory control, etc.

# Imports
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..'))) # Change level of path based on file location (the ../../)
import can
import time
import numpy as np
from Robot import*

# Old: CAN initialization in terminal: "sudo ip link set dev canX up type can bitrate 1000000" - with "X" being 0, 1, 2 and 3 for each bus
# New: CAN initialization in terminal: "for i in 0 1 2 3; do sudo ip link set dev can$i up type can bitrate 1000000 && sudo ip link set can$i txqueuelen 1000; done"

### SCRIPT START ###
## PRECOMPUTATIONS ##
print("Performing pre-computations...")

# Define time series
dt = 0.005 # seconds (200 Hz) - adjust as needed for your application - not directly the control frequency, but the discretization used for precomputations
total_time = 10  # Total time in seconds of a cycle of the tTarok/Main_Template.pyrajectory, adjust as needed for your application
num_time_steps = int(total_time / dt) + 1
t = np.linspace(0, total_time, num_time_steps)

# Insert optional pre-computations here, such as trajectory generation, inverse kinematics calculations, etc.

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

print(" Reading the PID Parameters")
Read_PID(bus0,ID_1)

time.sleep(2)
# Parameters for PID Tuning:
pid_params = {
    'angle_kp':  110,
    'angle_ki':  50,
    'speed_kp':  55,
    'speed_ki':  13,
    'torque_kp': 55,
    'torque_ki': 17
}

print("Update to new PID parameters USING RAM")
# PID_ROM_Control(bus0,ID_1, pid_params)
# PID_ROM_Control(bus0,ID_2, pid_params)
# PID_ROM_Control(bus0,ID_3, pid_params)
# 
# PID_ROM_Control(bus1,ID_1, pid_params)
# PID_ROM_Control(bus1,ID_2, pid_params)
# PID_ROM_Control(bus1,ID_3, pid_params)
# 
# PID_ROM_Control(bus2,ID_1, pid_params)
# PID_ROM_Control(bus2,ID_2, pid_params)
# PID_ROM_Control(bus2,ID_3, pid_params)
# 
# PID_ROM_Control(bus3,ID_1, pid_params)
# PID_ROM_Control(bus3,ID_2, pid_params)
# PID_ROM_Control(bus3,ID_3, pid_params)
time.sleep(2)



print("The New PID parameters are:")
Read_PID(bus0,ID_1)








## PRE-LOOP SEQUENCE ##

# Insert optional pre-loop sequence here

print("Pre-loop sequence complete, starting loop...")

## MAIN LOOP ##
print("Loop started - Press ctrl+c in terminal for shutdown")

# Note start time
start_time = cycle_start = current_time = time.monotonic()




try:
    while True:
            # Loop count (if needed)
            count = count + 1

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
            
            # Example usage of indexing:
            # Position_Control(bus0, ID_1, Theta_FL[index, 0], Theta_dot_FL[0, index]

            # IF in doubt about use of time management for index finding for trajectory control - check usage in Test_Up_Down_Full.py or Test_UpDown_FT_finalpy (Onedrive)

            # Insert main loop code here

            # Sleep for a short time to prevent busy looping, adjust as needed for your application (maybe even remove if you want to run as fast as possible)
            time.sleep(1)

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