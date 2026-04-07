# This is the main fil for testing functionality when working with Tarok

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
total_time = 10  # Total time in seconds of a cycle of the trajectory, adjust as needed for your application
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

            
            Battery_Voltage(bus0,ID_1)

            Read_Motor_Temperature(bus0,ID_1)

            time.sleep(20)

            '''
            Position_Control(bus0, ID_1, 0, 90)

            sleep(2)

            Position_Control(bus0, ID_1, -180, 90)

            sleep(4)
            

            
            # Weak torque control to make movement of legs easy
            Torque_Control(bus0, ID_1, 0.15)
            Torque_Control(bus0, ID_2, 0.02)
            Torque_Control(bus0, ID_3, 0.02)
            Torque_Control(bus1, ID_1, 0.02)
            Torque_Control(bus1, ID_2, 0.02)
            Torque_Control(bus1, ID_3, 0.02)
            Torque_Control(bus2, ID_1, 0.02)
            Torque_Control(bus2, ID_2, 0.02)
            Torque_Control(bus2, ID_3, 0.15)
            Torque_Control(bus3, ID_1, 0.02)
            Torque_Control(bus3, ID_2, 0.02)
            Torque_Control(bus3, ID_3, 0.02)
            sleep(0.5)
            
            
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
            print(f"Joint positions for Front Left: theta1 = {FL_theta1}, theta2 = {FL_theta2}, theta3 = {FL_theta3}")
            print(f"Joint positions for Front Right: theta1 = {FR_theta1}, theta2 = {FR_theta2}, theta3 = {FR_theta3}")
            print(f"Joint positions for Hind Left: theta1 = {HL_theta1}, theta2 = {HL_theta2}, theta3 = {HL_theta3}")
            print(f"Joint positions for Hind Right: theta1 = {HR_theta1}, theta2 = {HR_theta2}, theta3 = {HR_theta3}")
            print(f"========== ======== {count}")
            print("Loop running - Move legs to desired zero position - Press ctrl+c in terminal for shutdown")

            #Sleep
            sleep(3)
            '''


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