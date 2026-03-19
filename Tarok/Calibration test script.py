# Calibration test script




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
import struct
import re


# FROM ASGER_METHODS.py
def return_command_torque(torque):
    # This method converges the input Torque
    # into a data string that can be sent on the CAN
    if torque>2000 or torque <-2000:
        return
    data=[0xA1,0x00,0x00,0x00,0x00,0x00,0x00,0x00]
    if torque==0:
        return data
    if torque < 0 :
        hex_torque=2 #NegToHex(torque,4)
        data[5]=int('0x'+hex_torque[0:2],16)
        data[4]=int('0x'+hex_torque[2:4],16)
        # hex_torque=hex(torque)
        return data
    if torque<256: # if the torque <256, then it's two bits, the other two bits are zero
        data[4]=torque
    else: #if torque > 256, then the bit should be 4
        hex_torque=hex(torque)
        hex_torque=hex_torque[2:6]
        if len(hex_torque)==3:
            data[5]=int('0x'+hex_torque[0],16)
            data[4]=int('0x'+hex_torque[1:3],16)
        else:
            data[5]=int('0x'+hex_torque[0:2],16)
            data[4]=int('0x'+hex_torque[2:4],16)
    return data

def read_angle(bus,id):
    # bus = can.interface.Bus(channel=self.can_id, bustype='socketcan')
    # print(bus)
    
    send_msg = can.Message(arbitration_id=id, data=[0x92,0x00,0x00,0x00,0x00,0x00,0x00,0x00], is_extended_id=False)
    # print(send_msg)
    bus.send(send_msg)
    # sleep(0.0001)
    msg = bus.recv(4)    
    
    can_data = msg.data  # this is a bytes object (or bytearray)

    # Build 64-bit integer from bytes 1–7 (like in C++)
    multi_angle_position = (
        (0x00 << 56)
        | (can_data[7] << 48)
        | (can_data[6] << 40)
        | (can_data[5] << 32)
        | (can_data[4] << 24)
        | (can_data[3] << 16)
        | (can_data[2] << 8)
        | can_data[1]
    )

    # If MSB (bit 63) of byte[7] is set, sign-extend
    if can_data[7] & 0x80:
        # Convert to signed 64-bit
        multi_angle_position -= 1 << 56

    encoder_dec = multi_angle_position
    angle_float = float(encoder_dec) / 900.0

    # print(f"Encoder value: {angle_float}")
    return angle_float

def position_control(position,speed):
    #print(f"Sending a command for moveing to the new position at: {position} with the speed {speed}")
    
    data=[0xA4,0x00,0x00,0x00,0x00,0x00,0x00,0x00]
    speed_hex=hex(speed)
    if speed<=255:
        speed_hex=speed_hex[2:6]
        data[2]=speed
    else:
        if len(speed_hex)==5:
            data[2]=int('0x'+speed_hex[3:5],16)
            data[3]=int('0x'+speed_hex[2],16)
        else:
            data[2]=int('0x'+speed_hex[4:6],16)
            data[3]=int('0x'+speed_hex[2:4],16)

    #cur_pos=read_angle(bus,id)
    #print(str(hex(id))+' '+'current position:'+ str(cur_pos))
    new_pos=position
    
    if new_pos>=0:
        new_encoder_data=int(new_pos*100*9)
        new_encoder_data_hex=list(hex(new_encoder_data))
        temp=list('0x00000000')
        for i in range(len(new_encoder_data_hex)-2):
            temp[-(i+1)]=new_encoder_data_hex[-(i+1)]
        temp=re.findall(r'\w{1,2}',''.join(temp))
        
        data[-1]=int('0x'+temp[1],16)
        data[-2]=int('0x'+temp[2],16)
        data[-3]=int('0x'+temp[3],16)
        data[-4]=int('0x'+temp[4],16)

    else:
        new_encoder_data=int(new_pos*100*9)
        new_pos_hex=hex(new_encoder_data & 0xFFFFFFFF)
        new_pos_hex=new_pos_hex[2:10]
        data[7]=int('0x'+new_pos_hex[0:2],16)
        data[6]=int('0x'+new_pos_hex[2:4],16)
        data[5]=int('0x'+new_pos_hex[4:6],16)
        data[4]=int('0x'+new_pos_hex[6:8],16)    
    return data



# From Motor_Controls.py
def Torque_Control(Current_Amps):
    """ 
    This method takes the wanted current in Amps and converts it
    to the corresponding iq control as int16_t type.
    The input can range from -32 A to 32 A, and corresponds to iq control values from -2000 to 2000.
    Input: Current_Amps (float): Desired current in Amps, should be between -32 and 32.
    Output: data (list): A list of 8 bytes representing the CAN message to be sent to the motor for torque control.
    """ 
    data = [0xA1,0x00,0x00,0x00,0x00,0x00,0x00,0x00]

    iq_control = int(Current_Amps * 62.5)

    iq_bytes = struct.pack('<h', iq_control)  # Convert to little-endian signed short

    # Inserting the values inside the data array
    data[4] = iq_bytes[0]  # Low byte
    data[5] = iq_bytes[1]  # High byte

    return data

def Position_Control(New_Position,Max_Rotation_Speed):
    # This method takes the bus, ID, wanted position and speed as input and sends the corresponding command to the motor.
    data = [0xA4,0x00,0x00,0x00,0x00,0x00,0x00,0x00]

    # The New_Position is a int32_t type. And given in degress 
    # With 360000 representing 360 degrees
    # Can go both in positive and negative values

    '''
    # Rotation direction is determined by the target and current Position.
    Max_Rotation_Speed = Max_Rotation_Speed*10  # To get the desired number of degrees pr second, we need to multiply the input by 10, because the motor protocol uses a resolution of 0.1 degree pr second.
    # The Max Rotation Speed 
    Max_Speed_In_Hex = hex(Max_Rotation_Speed) 
    if Max_Rotation_Speed <= 255:
        data[2] = Max_Rotation_Speed
        Max_Speed_In_Hex = Max_Speed_In_Hex[2:6]
    else:
        if len(Max_Speed_In_Hex) == 5:
            data[2] = int('0x' + Max_Speed_In_Hex[3:5], 16)
            data[3] = int('0x' + Max_Speed_In_Hex[2], 16)
        else:
            data[2] = int('0x' + Max_Speed_In_Hex[4:6], 16)
            data[3] = int('0x' + Max_Speed_In_Hex[2:4], 16)
    '''

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
    return data


# From Motor_Readings.py
def Read_Angle(bus,id):

    data = [0x92,0x00,0x00,0x00,0x00,0x00,0x00,0x00]
    
    send_msg = can.Message(
                        arbitration_id=id, 
                        data=data, 
                        is_extended_id=False)
    
    bus.send(send_msg)
    sleep(0.0001)
    msg = bus.recv(5)    
    if msg is None:
        # print("No response received.")    # only print for debug - slows prcoess down
        return None
    
    can_data = msg.data  # this is a bytes object (or bytearray)

    # Build 64-bit integer from bytes 1–7 (like in C++)
    multi_angle_position = (
        (0x00 << 56)
        | (can_data[7] << 48)
        | (can_data[6] << 40)
        | (can_data[5] << 32)
        | (can_data[4] << 24)
        | (can_data[3] << 16)
        | (can_data[2] << 8)
        | can_data[1]
    )

    # If MSB (bit 63) of byte[7] is set, sign-extend
    if can_data[7] & 0x80:
        # Convert to signed 64-bit
        multi_angle_position -= 1 << 56

    encoder_dec = multi_angle_position
    angle_float = float(encoder_dec) / 900.0

    # print(f"Encoder value: {angle_float}")
    return angle_float




# Check that torque control produce the same
data_AM = return_command_torque(1)
data_MC = Torque_Control(0.02)
print(f"Torque control data from ASGER_METHODS: {data_AM}")
print(f"Torque control data from Motor_Controls: {data_MC}")





# Check that read angle produce the same



# Check that old and new position control produce the same
data_AM = position_control(30.26,20)
data_MC = Position_Control(30.26,20)
print(f"Position control data from ASGER_METHODS: {data_AM}")
print(f"Position control data from Motor_Controls: {data_MC}")

