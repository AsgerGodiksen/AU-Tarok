# Calibration of all 12 actuators

# Run this script to calibrate zero-position of all 12 actuators.
# Power off after running this script to finalize the calibration.
# Position is stored even when the robot is powered off.
# But the legs must be in the correct position when turned on again.
# The correct position is any position in which all 3 actuators of a leg is within the interval of -40:0 degrees.

import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..')))
import time
import can
from Robot.Hardware.Motor_Controls import Torque_Control, Motor_Stop
from Robot.Hardware.Motor_Readings import Read_Angle

# Calibration
if __name__ == "__main__":
    try:
        print("Starting calibration, please wait...")

        print("Initializing CAN buses...")
        # Define motor IDs (might be specific to chosen physical leg)
        ID_1 = 0x141  # Motor for theta1
        ID_2 = 0x142  # Motor for theta2
        ID_3 = 0x143  # Motor for theta3

        # Connect to CAN bus
        bus0 = can.interface.Bus(interface='socketcan', channel='can0', bitrate=1000000)
        bus1 = can.interface.Bus(interface='socketcan', channel='can1', bitrate=1000000)
        bus2 = can.interface.Bus(interface='socketcan', channel='can2', bitrate=1000000)
        bus3 = can.interface.Bus(interface='socketcan', channel='can3', bitrate=1000000)    

        # Drain any stale messages from the buses
        for bus in [bus0, bus1, bus2, bus3]:
            for i in range(100):		# Now Listens for any signals 100 times with 0.01 s in between. Prints if any.
                msg = bus.recv(0.01)
                if msg:
                    print(msg)

        print("Initialization complete, starting calibration loop...")   
        print("Loop started - Move legs to desired zero position - Press ctrl+c in terminal for shutdown")
        i = 0
        while True:
            msg = bus0.recv(0.05)
            msg = bus1.recv(0.05)
            msg = bus2.recv(0.05)
            msg = bus3.recv(0.05)
            i = i+1
            
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
            print(f"Joint positions for Front Left: theta1 = {FL_theta1}, theta2 = {FL_theta2}, theta3 = {FL_theta3}")
            print(f"Joint positions for Front Right: theta1 = {FR_theta1}, theta2 = {FR_theta2}, theta3 = {FR_theta3}")
            print(f"Joint positions for Hind Left: theta1 = {HL_theta1}, theta2 = {HL_theta2}, theta3 = {HL_theta3}")
            print(f"Joint positions for Hind Right: theta1 = {HR_theta1}, theta2 = {HR_theta2}, theta3 = {HR_theta3}")
            print(f"========== ======== {i}")
            print("Loop running - Move legs to desired zero position - Press ctrl+c in terminal for shutdown")
            time.sleep(1)
            
    except KeyboardInterrupt:
    # Handle the keyboard interrupt (Ctrl+C)
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
        print("Shutdown complete. Please power off the robot to finalize the calibration.")