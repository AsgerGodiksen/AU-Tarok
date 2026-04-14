# Show and print angle (angular position) of all 3 actuators in all 4 legs

# Recommend to run this script every time the robot is powered on:
# - Use to check that all legs were in correct position when powered on (all 3 actuators of a leg should be within the interval of -40:0 degrees)
# - Used to check that calibration is as it should be
# - Check these things by running this script and moving the legs near the zero-configuration,
#   and check that the angles are as expected, which is around 0 degrees. 
#   - If the angles are shifted by +- x*40 degrees, then the leg was not in correct position when powered on
#   - If the angles are wrong by some other amount, then the calibration is likely wrong and should be redone by running the "Calibration.py" script

# Imports
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..')))
import can
import time
import numpy as np
from Robot import*

# Old: CAN initialization in terminal: "sudo ip link set dev canX up type can bitrate 1000000" - with "X" being 0, 1, 2 and 3 for each bus

### SCRIPT START ###
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

print("Initialization complete, starting loop")
print("Loop started - Press ctrl+c in terminal for shutdown")

try:
    while True:
            # Loop count
            count = count + 1

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
            print(f"========== ======== {count}")
            print("Loop running - Move legs to zero-configuration and check that angles are correct - Press ctrl+c in terminal for shutdown")

            #Sleep
            time.sleep(1.5)

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