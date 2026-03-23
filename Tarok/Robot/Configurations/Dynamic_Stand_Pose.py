# Importing the nessesary libaries
import numpy as np
import sys
import os
import time
import can
import csv

### Ensuring the Path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

### Importing the nessesary functions from the Project Directory 
from Robot.Tarok_Dymensions import TarokDymensions
from Robot.Kinematics import Inverse_Kinematics as IK
from Robot.Kinematics.Balance_Control import BalanceControl

### Some of the imported Transformations are from "Constant_Transforms.py" and "Forward_Kinematics"
from Robot.Kinematics import T0_B

from Robot.Hardware import Position_Control
from Robot.Hardware import Motor_Stop
from Robot.Hardware import IMU_Initialization, Get_Quaternion, Quaternion_To_Euler, IMU_To_Body_Frame
from Robot.Hardware import SMBus2I2C


## Importing the Tarok Dimension Class
Tarok_Class = TarokDymensions()

# Importing the Initial Foot Positions
Front_Left_Foot, Front_Right_Foot, Hind_Left_Foot, Hind_Right_Foot = Tarok_Class.Initial_Foot_Positions()

# Transform desired end-effector position from body frame to leg base frames
P_FL_Base = T0_B(Front_Left_Foot, 'FL')
P_FR_Base = T0_B(Front_Right_Foot, 'FR')
P_HL_Base = T0_B(Hind_Left_Foot, 'HL')
P_HR_Base = T0_B(Hind_Right_Foot, 'HR')


# Determine joint angles using inverse kinematics
Theta_FL = IK(P_FL_Base, 'FL')
Theta_FR = IK(P_FR_Base, 'FR')
Theta_HL = IK(P_HL_Base, 'HL')
Theta_HR = IK(P_HR_Base, 'HR')

# Convert angles from radians to degrees for motor control
Theta_FL = np.degrees(Theta_FL)
Theta_FR = np.degrees(Theta_FR)
Theta_HL = np.degrees(Theta_HL)
Theta_HR = np.degrees(Theta_HR)

# Define motor IDs
ID_1 = 0x141  # Motor for theta1
ID_2 = 0x142  # Motor for theta2
ID_3 = 0x143  # Motor for theta3

# Connect to CAN buses
print("Initializing CAN buses...")
bus0 = can.interface.Bus(interface='socketcan', channel='can0', bitrate=1000000)
#bus1 = can.interface.Bus(interface='socketcan', channel='can1', bitrate=1000000)
#bus2 = can.interface.Bus(interface='socketcan', channel='can2', bitrate=1000000)
#bus3 = can.interface.Bus(interface='socketcan', channel='can3', bitrate=1000000)

# Drain any stale messages from the buses
for bus in [bus0]: # , bus1, bus2, bus3
    for i in range(100):
        msg = bus.recv(0.01)
        if msg:
            print(msg)

print("Initialization complete, moving to zero position...")

# Move to zero position
Position_Control(bus0, ID_1, 0, 20)
Position_Control(bus0, ID_2, 0, 20)
Position_Control(bus0, ID_3, 0, 20)
#Position_Control(bus1, ID_1, 0, 20)
#Position_Control(bus1, ID_2, 0, 20)
#Position_Control(bus1, ID_3, 0, 20)
#Position_Control(bus2, ID_1, 0, 20)
#Position_Control(bus2, ID_2, 0, 20)
#Position_Control(bus2, ID_3, 0, 20)
#Position_Control(bus3, ID_1, 0, 20)
#Position_Control(bus3, ID_2, 0, 20)
#Position_Control(bus3, ID_3, 0, 20)

time.sleep(2)

print("Moved to zero position, moving to standing position...")

# Move to standing position
Position_Control(bus0, ID_1, Theta_FL[0], 20)
Position_Control(bus0, ID_2, Theta_FL[1], 20)
Position_Control(bus0, ID_3, Theta_FL[2], 20)
#Position_Control(bus1, ID_1, Theta_FR[0], 20)
#Position_Control(bus1, ID_2, Theta_FR[1], 20)
#Position_Control(bus1, ID_3, Theta_FR[2], 20)
#Position_Control(bus2, ID_1, Theta_HL[0], 20)
#Position_Control(bus2, ID_2, Theta_HL[1], 20)
#Position_Control(bus2, ID_3, Theta_HL[2], 20)
#Position_Control(bus3, ID_1, Theta_HR[0], 20)
#Position_Control(bus3, ID_2, Theta_HR[1], 20)
#Position_Control(bus3, ID_3, Theta_HR[2], 20)

time.sleep(2)

print("Moved to standing position, starting balance control...")


print("Initialising IMU...")
bno, i2c = IMU_Initialization()
print("IMU ready")
time.sleep(0.5)

### Initiating Data Logging
## Initiating Data Logging — By Making a Unique file
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
log_dir = os.path.join(SCRIPT_DIR, "TEST_DATA")
os.makedirs(log_dir, exist_ok=True)
log_filename = os.path.join(log_dir, f"Stand_Pose_Balance_TEST_IMU_Log_{time.strftime('%Y-%m-%d_%H-%M-%S')}.csv")


# Writing the header line
with open(log_filename, 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow(["Timestamp (s)", "Pitch_deg", "Roll_deg", "Yaw_deg"])

# Preallocate data storage array
data = np.zeros((1500000, 4))
data_count = 0
start_time = time.monotonic()

print("Loop started - Press ctrl+c in terminal for shutdown")

# Assemble nominal foot positions as 4x3 plain list for Balance_Control
# Order: [FL, FR, HL, HR]
Foot_Positions = [
    Front_Left_Foot.flatten().tolist(),
    Front_Right_Foot.flatten().tolist(),
    Hind_Left_Foot.flatten().tolist(),
    Hind_Right_Foot.flatten().tolist(),
]

# Initiating the Balance_Controller
Balance_Controller = BalanceControl()


i = 1
try:
    while True:
        print("Iteration",i )
        # Read IMU
        quat  = Get_Quaternion(bno)
        Euler = Quaternion_To_Euler(quat)
        
        Pitch = Euler[0]
        Roll  = Euler[1]
        Yaw   = Euler[2]
        #print(f"Measured Pitch is:",Pitch,"And Measured Roll is",Roll)

        # Convert to radians for Balance_Control
        euler_rad = np.radians(Euler)

        # Call Balance_Control to get corrected foot positions
        Corrected_Foot_Positions = Balance_Controller.update(euler_rad, 0.0, 0.0, Foot_Positions)

        print(f"Pitch_rad: {euler_rad[0]:.5f}  Roll_rad: {euler_rad[1]:.5f}")
        # IK for each leg
        P_FL_Base = T0_B(np.array(Corrected_Foot_Positions[0]).reshape(3,1), 'FL')
        P_FR_Base = T0_B(np.array(Corrected_Foot_Positions[1]).reshape(3,1), 'FR')
        P_HL_Base = T0_B(np.array(Corrected_Foot_Positions[2]).reshape(3,1), 'HL')
        P_HR_Base = T0_B(np.array(Corrected_Foot_Positions[3]).reshape(3,1), 'HR')

        #print("\n")
        #print("P_FL_Base",P_FL_Base[0],P_FL_Base[1],P_FL_Base[2])
        #print("P_FR_Base",P_FR_Base[0],P_FR_Base[1],P_FR_Base[2])
        #print("P_HL_Base",P_HL_Base[0],P_HL_Base[1],P_HL_Base[2])
        #print("P_HR_Base",P_HR_Base[0],P_HR_Base[1],P_HR_Base[2])

        Theta_FL_new = np.degrees(IK(P_FL_Base, 'FL'))
        Theta_FR_new = np.degrees(IK(P_FR_Base, 'FR'))
        Theta_HL_new = np.degrees(IK(P_HL_Base, 'HL'))
        Theta_HR_new = np.degrees(IK(P_HR_Base, 'HR'))

        # Send motor commands
        Position_Control(bus0, ID_1, Theta_FL_new[0], 20)
        Position_Control(bus0, ID_2, Theta_FL_new[1], 20)
        Position_Control(bus0, ID_3, Theta_FL_new[2], 20)
        #Position_Control(bus1, ID_1, Theta_FR_new[0], 20)
        #Position_Control(bus1, ID_2, Theta_FR_new[1], 20)
        #Position_Control(bus1, ID_3, Theta_FR_new[2], 20)
        #Position_Control(bus2, ID_1, Theta_HL_new[0], 20)
        #Position_Control(bus2, ID_2, Theta_HL_new[1], 20)
        #Position_Control(bus2, ID_3, Theta_HL_new[2], 20)
        #Position_Control(bus3, ID_1, Theta_HR_new[0], 20)
        #Position_Control(bus3, ID_2, Theta_HR_new[1], 20)
        #Position_Control(bus3, ID_3, Theta_HR_new[2], 20)

        # Log data
        current_time = time.monotonic()
        timestamp = current_time - start_time
        data[data_count, :] = [timestamp, Pitch, Roll, Yaw]
        data_count += 1
        
        # Update for next iteration
        Foot_Positions = Corrected_Foot_Positions

        # 20 Hz
        time.sleep(0.1)
        i = i +1

        
        
except KeyboardInterrupt:
    print("KeyboardInterrupt received, shutting down...")

    print("Stopping motors...")
    Motor_Stop(bus0, ID_1)
    Motor_Stop(bus0, ID_2)
    Motor_Stop(bus0, ID_3)
    #Motor_Stop(bus1, ID_1)
    #Motor_Stop(bus1, ID_2)
    #Motor_Stop(bus1, ID_3)
    #Motor_Stop(bus2, ID_1)
    #Motor_Stop(bus2, ID_2)
    #Motor_Stop(bus2, ID_3)
    #Motor_Stop(bus3, ID_1)
    #Motor_Stop(bus3, ID_2)
    #Motor_Stop(bus3, ID_3)
    print("Motors stopped")

    print("Shutting down CAN buses...")
    bus0.shutdown()
    #bus1.shutdown()
    #bus2.shutdown()
    #bus3.shutdown()
    print("CAN buses shut down")

    i2c.deinit()
    print("IMU disconnected.")

    print("Storing logged data to file...")
    with open(log_filename, "a", newline='') as file:
        writer = csv.writer(file)
        for i in range(data_count):
            row = data[i, :]
            writer.writerow([f"{row[0]:.4f}", f"{row[1]:.4f}", f"{row[2]:.4f}", f"{row[3]:.4f}"])
    print("Logged data stored")
    print("Shutdown complete.")