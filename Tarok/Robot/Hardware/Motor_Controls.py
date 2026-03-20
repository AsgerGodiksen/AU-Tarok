# This version is a copy of the one located in Test Simulations Final on 17/02 2026
# It might be subject to later change 

import time
import can
import re
import binascii
import struct

def Torque_Control(bus,id,Current_Amps):
    """ 
    This method takes the wanted current in Amps and converts it
    to the corresponding iq control as int16_t type.
    The input can range from -32 A to 32 A, and corresponds to iq control values from -2000 to 2000.
    Input: 
        - bus: The CAN bus object to send the message on.
        - id: The ID of the motor to send the command to.
        - Current_Amps (float): Desired current in Amps, should be between -32 and 32.
    Output: None. The function sends a CAN message to the specified motor to control its torque.
    """ 

    # To avoid sending full torque (value = 0)
    if Current_Amps < 0.02:
        Current_Amps = 0.02 # Resulting in iq_control being 1, which is the minimum non-zero value and practically just free movement of the output.

    data = [0xA1,0x00,0x00,0x00,0x00,0x00,0x00,0x00]

    iq_control = int(Current_Amps * 62.5)

    iq_bytes = struct.pack('<h', iq_control)  # Convert to little-endian signed short

    # Inserting the values inside the data array
    data[4] = iq_bytes[0]  # Low byte
    data[5] = iq_bytes[1]  # High byte

    send_msg = can.Message(
                    arbitration_id=id, 
                    data=data, 
                    is_extended_id=False)
    bus.send(send_msg)
    # Flush reply
    bus.recv(0.1) # Flush reply, dont care about the content, just want to make sure we dont have any stale replies in the buffer for later commands

def Position_Control(bus,id,New_Position,Max_Rotation_Speed):
    """
    This method takes the bus, ID, wanted position and speed as input and sends the corresponding command to the motor.
    Input:
    - bus: The CAN bus object to send the message on.
    - id: The ID of the motor to send the command to.
    - New_Position: Desired position in degrees, where 360 degrees corresponds to 360000 in the motor's encoder units. Can be positive or negative.
    - Max_Rotation_Speed: Desired maximum rotation speed in degrees per second.
    Output: None. The function sends a CAN message to the specified motor to control its position and speed.
    """

    data = [0xA4,0x00,0x00,0x00,0x00,0x00,0x00,0x00]

    # The speed is converted as 1dps/LSB to uint16_t 
    speed_raw = int(Max_Rotation_Speed)  # Convert to the motor's expected format
    speed_bytes = struct.pack('<h', speed_raw)  # Convert to little-endian signed short
    data[2] = speed_bytes[0]  # Low byte
    data[3] = speed_bytes[1]  # High byte

    
    # Now Looking at the New_Position
    # The New_Position is a int32_t type. And given in degress 
    # With 360000 representing 360 degrees
    if New_Position >= 0:
        New_Position_Encoder_Data = int(New_Position * 100*9)  # Convert to encoder data
        New_Position_In_Bytes = struct.pack('<i', New_Position_Encoder_Data)  # Convert to little-endian signed int
        data[4] = New_Position_In_Bytes[0]
        data[5] = New_Position_In_Bytes[1]
        data[6] = New_Position_In_Bytes[2]
        data[7] = New_Position_In_Bytes[3]
    else: # For Negative angle values
        New_Position_Encoder_Data = int(New_Position * 100*9)  # Convert to encoder data
        New_Position_Encoder_Data = New_Position_Encoder_Data & 0xFFFFFFFF  # Convert to unsigned 32-bit
        New_Position_In_Bytes = struct.pack('<I', New_Position_Encoder_Data)  # Convert to little-endian unsigned int
        data[4] = New_Position_In_Bytes[0]
        data[5] = New_Position_In_Bytes[1]
        data[6] = New_Position_In_Bytes[2]
        data[7] = New_Position_In_Bytes[3]

    send_msg = can.Message(
                    arbitration_id=id, 
                    data=data, 
                    is_extended_id=False)
    bus.send(send_msg)
    # Flush reply
    bus.recv(0.1) # Flush reply, dont care about the content, just want to make sure we dont have any stale replies in the buffer for later commands

def Speed_Control(bus,id,speed):
    ##### NOTE: NOT SURE IF CORRECT UNIT IS USED FOR SPEED, CHECK COMMAND PROTOCOL #####
    # This method takes the bus, ID and a arbitrary speed value
    # The speed is given as Degrees pr second.
    speed = speed * 10  # To get the desired number of degrees pr second, we need to multiply the input by 10, because the motor protocol uses a resolution of 0.1 degree pr second.

    data = [0xA2,0x00,0x00,0x00,0x00,0x00,0x00,0x00]

    # The speed value must be converted into  int_32_t integer
    # We must tranlate the value into data[4] to data[7]
    speed_raw = int(round(speed*100))
    speed_bytes = speed_raw.to_bytes(4, byteorder='little', signed=True)

    data[4] = speed_bytes[0]
    data[5] = speed_bytes[1]
    data[6] = speed_bytes[2]
    data[7] = speed_bytes[3]

    msg = can.Message(
        arbitration_id=id,
        data=data,
        is_extended_id=False
    )
    bus.send(msg)
    # Flush reply
    bus.recv(0.1) # Flush reply, dont care about the content, just want to make sure we dont have any stale replies in the buffer for later commands
        
def Motor_Stop(bus,id):
    # This command will stop the motor from running, but not remove the earlier command.  
    data = [0x81,0x00,0x00,0x00,0x00,0x00,0x00,0x00]

    send_msg = can.Message(
                    arbitration_id=id, 
                    data=data, 
                    is_extended_id=False)
    
    bus.send(send_msg)

    # stricter pattern for flush compared to other motor control commands - we want to be on the safe side with motor stop
    while True:
        msg = bus.recv(0.5)
        if msg is None:
            raise RuntimeError(f"Motor {id} did not confirm stop command.")
        if msg.arbitration_id == id and msg.data[0] == 0x81:
            break

def Map_Value(value, from_low, from_high, to_low, to_high):
    # Scale the value from the input range to the output range
    return (value - from_low) * (to_high - to_low) / (from_high - from_low) + to_low

def PID_RAM_Control(bus, id,
                    Position_Kp=None, Position_Ki=None,
                    Speed_Kp=None, Speed_Ki=None,
                    Torque_Kp=None, Torque_Ki=None):
    """
    # default table
    defaults = {
        'Position_Kp': 10,
        'Position_Ki': 0,
        'Speed_Kp': 20,
        'Speed_Ki': 0,
        'Torque_Kp': 30,
        'Torque_Ki': 0,
    }
    # This method is used to control the PID values of the motor. 
    # It takes the bus, ID and the wanted PID values as input.
    data = [0x31,0x00,0x00,0x00,0x00,0x00,0x00,0x00]

    data[2] = Position_Kp
    data[3] = Position_Ki
    data[4] = Speed_Kp
    data[5] = Speed_Ki
    data[6] = Torque_Kp
    data[7] = Torque_Ki

    send_msg = can.Message(arbitration_id=id, data=data, is_extended_id=False)
    bus.send(send_msg)
    """


