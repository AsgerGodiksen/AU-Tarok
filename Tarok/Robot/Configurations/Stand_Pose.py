

import numpy as np
import sys
import os
import time
import can
import csv

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

from Robot.Tarok_Dymensions import Tarok_Dymensions_Class
from Robot.Kinematics import Inverse_Kinematics as IK
# Some of the imported Transformations are from "Constant_Transforms.py" and "Forward_Kinematics"
from Robot.Kinematics import T0_B
from Robot.Hardware import Position_Control
from Robot.Hardware import Motor_Stop

from Robot.Hardware.IMU_BNO085 import IMU_Initialization, Get_Quaternion, Quaternion_To_Euler


Tarok_Class = Tarok_Dymensions_Class()  # instantiate first


# Importing the Inital Foot Position
Front_Left_Foot, Front_Right_Foot, Hind_Left_Foot, Hond_Right_Foot = Tarok_Class.Initial_Foot_Positions()

# Transform desired end-effector position from body frame to leg base frames
P_FL_Base = T0_B(Front_Left_Foot,'FL')
P_FR_Base = T0_B(Front_Right_Foot,'FR')
P_HL_Base = T0_B(Hind_Left_Foot,'HL')
P_HR_Base = T0_B(Hond_Right_Foot,'HR')

# Then we can determine the Inverse Kinematics
Theta_FL = IK(P_FL_Base,'FL')
Theta_FR = IK(P_FR_Base,'FR')
Theta_HL = IK(P_HL_Base,'HL')
Theta_HR = IK(P_HR_Base,'HR')

# Convert angles from radians to degrees for motor control
Theta_FL = np.degrees(Theta_FL)
Theta_FR = np.degrees(Theta_FR)
Theta_HL = np.degrees(Theta_HL)
Theta_HR = np.degrees(Theta_HR)

#### Initiating the Control of the Motors

######### Send commands to motors ############
print("Initializing CAN buses...")
# Define motor IDs (might be specific to chosen physical leg)
ID_1 = 0x141  # Motor for theta1
ID_2 = 0x142  # Motor for theta2
ID_3 = 0x143  # Motor for theta3

# Connect to CAN bus
# bus0 = can.interface.Bus(interface='socketcan', channel='can0', bitrate=1000000)
# #bus1 = can.interface.Bus(interface='socketcan', channel='can1', bitrate=1000000)
# #bus2 = can.interface.Bus(interface='socketcan', channel='can2', bitrate=1000000)
# #bus3 = can.interface.Bus(interface='socketcan', channel='can3', bitrate=1000000)

#### REMEMBER The Other Busses when having all 4 legs
# Drain any stale messages from the buses
# for bus in [bus0]: #, bus1, bus2,bus3
#     for i in range(100):		# Now Listens for any signals 100 times with 0.01 s in between. Prints if any.
#         msg = bus.recv(0.01)
#         if msg:
#             print(msg)
            
print("Initialization complete, moving to zero position...")

# Move to zero position 
# Position_Control(bus0,ID_1,0,20)
# Position_Control(bus0,ID_2,0,20)
# Position_Control(bus0,ID_3,0,20)
# #Position_Control(bus1,ID_1,0,20)
# #Position_Control(bus1,ID_2,0,20)
# #Position_Control(bus1,ID_3,0,20)
# #Position_Control(bus2,ID_1,0,20)
# #Position_Control(bus2,ID_2,0,20)
# #Position_Control(bus2,ID_3,0,20)
# #Position_Control(bus3,ID_1,0,20)
# #Position_Control(bus3,ID_2,0,20)
# #Position_Control(bus3,ID_3,0,20)

time.sleep(4)

print("Moved to zero position, moving to initial trajectory position...")

# Move to initial position
# Position_Control(bus0,ID_1,Theta_FL[0],20)
# Position_Control(bus0,ID_2,Theta_FL[1],20)
# Position_Control(bus0,ID_3,Theta_FL[2],20)
# #Position_Control(bus1,ID_1,Theta_FR[0],20)
# #Position_Control(bus1,ID_2,Theta_FR[1],20)
# #Position_Control(bus1,ID_3,Theta_FR[2],20)
# #Position_Control(bus2,ID_1,Theta_HL[0],20)
# #Position_Control(bus2,ID_2,Theta_HL[1],20)
# #Position_Control(bus2,ID_3,Theta_HL[2],20)
# #Position_Control(bus3,ID_1,Theta_HR[0],20)
# #Position_Control(bus3,ID_2,Theta_HR[1],20)
# #Position_Control(bus3,ID_3,Theta_HR[2],20)

time.sleep(6)

print("Moved to initial trajectory position, starting trajectory execution...")
print("Loop started - Press ctrl+c in terminal for shutdown")

print("Initialising IMU...")
bno, i2c = IMU_Initialization()
print("IMU ready")
time.sleep(0.5)


log_filename = f"TEST_DATA/Standing_TEST_IMU_Log_{time.strftime('%Y-%m-%d_%H-%M-%S')}.csv"
with open(log_filename, 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow(["Timestamp (s)", "Pitch_deg", "Roll_deg", "Yaw_deg"])  # Header

# Preallocate data storage array
data = np.zeros((1500000,4))  # Adjust size as needed (current run for max of 83.33 minutes at 200 Hz)
data_count = 0


try:
    while True:     
            quat  = Get_Quaternion(bno)
            euler = Quaternion_To_Euler(quat)

            timestamp = time.time()
            
            Pitch = euler[0]
            Roll  = euler[1]
            Yaw   = euler[2]
            
            data[data_count, :] = [timestamp,Pitch,Roll,Yaw]
            data_count += 1
        
            time.sleep(0.05)

# Stop loop with Ctrl+C
except KeyboardInterrupt:
    print("KeyboardInterrupt received, shutting down...")

    print("Stopping motors...")
    # Motor_Stop(bus0,ID_1)
    # Motor_Stop(bus0,ID_2)
    # Motor_Stop(bus0,ID_3)
    # #Motor_Stop(bus1,ID_1)
    # #Motor_Stop(bus1,ID_2)
    # #Motor_Stop(bus1,ID_3)
    # #Motor_Stop(bus2,ID_1)
    # #Motor_Stop(bus2,ID_2)
    # #Motor_Stop(bus2,ID_3)
    # #Motor_Stop(bus3,ID_1)
    # #Motor_Stop(bus3,ID_2)
    # #Motor_Stop(bus3,ID_3)
    print("Motors stopped")

    print("Shutting down CAN buses...")
    # bus0.shutdown()
    # #bus1.shutdown()
    # #bus2.shutdown()
    # #bus3.shutdown()
    print("CAN buses shut down")

    print("Shutdown complete.")
    
    i2c.deinit()
    print("IMU disconnected.")
    
    print("Storing logged data to file")
    # Store logged data in file
    with open(log_filename,"a") as file:
        for i in range(data_count):
            row = data[i,:]
            file.write([f"{data[0]:.4f}", f"{data[1]:.4f}", f"{data[2]:.4f}", f"{data[3]:.4f}"])
    print("Logged data stored")
    print("Shutdown complete.")