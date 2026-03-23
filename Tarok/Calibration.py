# Calibration of all 12 actuators

# Run this script to calibrate zero-position of all 12 actuators.
# Power off after running this script to finalize the calibration.
# Position is stored even when the robot is powered off.
# But the legs must be in the correct position when turned on again.
# The correct position is any position in which all 3 actuators of a leg is within the interval of -40:0 degrees.

# Imports
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..')))
import time
import can
from Robot import *

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

print("Initialization complete, starting calibration loop...")
print("Loop started - Move legs to desired zero position - Press ctrl+c in terminal for shutdown")

try:
    while True:
            # Loop count
            count = count + 1

            # Weak torque control command to make movement of legs easy
            Torque_Control(bus0,ID_1,0.02)
            Torque_Control(bus0,ID_2,0.02)
            Torque_Control(bus0,ID_3,0.02)
            Torque_Control(bus1,ID_1,0.02)
            Torque_Control(bus1,ID_2,0.02)
            Torque_Control(bus1,ID_3,0.02)
            Torque_Control(bus2,ID_1,0.02)
            Torque_Control(bus2,ID_2,0.02)
            Torque_Control(bus2,ID_3,0.02)
            Torque_Control(bus3,ID_1,0.02)
            Torque_Control(bus3,ID_2,0.02)
            Torque_Control(bus3,ID_3,0.02)

            # Short sleep
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
            print(f"Joint positions for Front Left:   theta1 = {FL_theta1},  theta2 = {FL_theta2},  theta3 = {FL_theta3}")
            print(f"Joint positions for Front Right:  theta1 = {FR_theta1},  theta2 = {FR_theta2},  theta3 = {FR_theta3}")
            print(f"Joint positions for Hind Left:    theta1 = {HL_theta1},  theta2 = {HL_theta2},  theta3 = {HL_theta3}")
            print(f"Joint positions for Hind Right:   theta1 = {HR_theta1},  theta2 = {HR_theta2},  theta3 = {HR_theta3}")
            print(f"========== ======== {count}")
            print("Loop running - Move legs to desired zero position - Press ctrl+c in terminal to finalize calibration and shutdown")

            # Sleep for a short time to prevent busy looping, adjust as needed for your application (maybe even remove if you want to run as fast as possible)
            time.sleep(1.5)

# Stop loop with Ctrl+C in terminal
except KeyboardInterrupt:
    print("KeyboardInterrupt received, finalizing calibration and shutting down...")

    # 0x19 command to save zero position to ROM
    send_msg_1 = can.Message(arbitration_id=ID_1, data=[0x19,0x00,0x00,0x00,0x00,0x00,0x00,0x00], is_extended_id=False) 
    send_msg_2 = can.Message(arbitration_id=ID_2, data=[0x19,0x00,0x00,0x00,0x00,0x00,0x00,0x00], is_extended_id=False) 
    send_msg_3 = can.Message(arbitration_id=ID_3, data=[0x19,0x00,0x00,0x00,0x00,0x00,0x00,0x00], is_extended_id=False) 

    # Send the save command to all motors
    bus0.send(send_msg_1)
    time.sleep(0.001)
    bus0.send(send_msg_2)
    time.sleep(0.001)
    bus0.send(send_msg_3)
    time.sleep(0.001)
    bus1.send(send_msg_1)
    time.sleep(0.001)
    bus1.send(send_msg_2)
    time.sleep(0.001)
    bus1.send(send_msg_3)
    time.sleep(0.001)
    bus2.send(send_msg_1)
    time.sleep(0.001)
    bus2.send(send_msg_2)
    time.sleep(0.001)
    bus2.send(send_msg_3)
    time.sleep(0.001)
    bus3.send(send_msg_1)
    time.sleep(0.001)
    bus3.send(send_msg_2)
    time.sleep(0.001)
    bus3.send(send_msg_3)
    time.sleep(0.5) # Wait a bit to ensure messages are sent before shutting down buses
    print("Zero positions saved to ROM")

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
    print("Shutdown complete. Please power off the robot to finalize the calibration.")