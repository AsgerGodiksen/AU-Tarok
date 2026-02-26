import sys
import os

import can




# This file is meant to ensure a inital pose for Tarok, before starting further movement.
Motor_ID: list[str]= [0x141, 0x142, 0x143] 
# Motor IDs for the Motors in each leg


# Staring with the angles for the joints in the legs 
FL = [0, -28, 47]   # Front Left Leg
FR = [0, 0, 0]      # Front Right Leg

HL = [0, 0, 0]      # Hind Left Leg
HR = [0, 0, 0]      # Hind Right Leg


# Moving to the initial pose, slowly so we ensure time to stop



if __name__ == "__main__":
    sys.path.append(os.path.join(os.path.dirname(__file__), '../..'))
    from Robot import*

    print("Setting zero pose for Tarok")
    Front_Left  = can.interface.Bus(bustype='socketcan', channel='can2', bitrate=1000000)
    Position_Control(Front_Left, Motor_ID[0], 0, 100)
    Position_Control(Front_Left, Motor_ID[1], 0, 100)
    Position_Control(Front_Left, Motor_ID[2], 0, 100)
    sleep(5)
    print(" have Moved to zero position, now moving to 10 degrees")


    # Moving to 10 degrees to ensire that the motors are working, and to have a reference point for the next movement
    for i in range(3):
        Position_Control(Front_Left, Motor_ID[i], 10, 50)
        sleep(0.001)
    sleep(5)
    print(" have Moved to 10 degrees, now moving to zero position")
    for i in range(3):
        Position_Control(Front_Left, Motor_ID[i], 0, 50)
        print(f"Moving Motor {Motor_ID[i]} to 0 degrees")
        sleep(0.001)

    sleep(5)
    print(" have Moved to zero position, now moving to initial pose")
    
    # Move to zero position
    for i in range(3):
        Position_Control(Front_Left, Motor_ID[i], FL[i], 50)
        print(f"Moving Motor {Motor_ID[i]} to {FL[i]} degrees")
        sleep(2)

    # Shut down the CAN bus
    Front_Left.shutdown()