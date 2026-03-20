# This will be the main file when oberating with TAROK
import can
from time import sleep,time
#import numpy as np
from Robot import*

# CAN initialization in terminal: sudo ip link set dev canX up type can bitrate 1000000
# with "X" being 0, 1, 2 and 3 for each bus

# Tarok dimensions if needed
Tarok = Tarok_Dymensions_Class

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

count = 0

print("Initialization complete, starting pre-loop sequence")

# Insert optional pre-oop sequence here

print("Pre-loop sequence complete, starting loop")
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


# Stop loop with Ctrl+C in terminal
except KeyboardInterrupt:
    print("KeyboardInterrupt received, shutting down...")

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
    print("CAN buses shut down")

    print("Shutdown complete.")