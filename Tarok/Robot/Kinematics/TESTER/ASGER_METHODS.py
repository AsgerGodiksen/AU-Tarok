# This version is a copy of the one located in Test Simulations Final on 17/02 2026
# It might be subject to later change 

import time
import can
import re
import binascii
import struct


def torque_control(bus,id,torque):
    data=return_command_torque(torque) #positive torque is counterclockwise!!!
    # bus = can.interface.Bus(channel=self.can_id, bustype='socketcan')
    
    bus.send(can.Message(arbitration_id=id, data=data, is_extended_id=False))
    
    msg=bus.recv(1)
    
    
    can_data = msg.data
   
    command_byte = can_data[0]
    motor_temperature = can_data[1]
    torque_current_low = can_data[2]
    torque_current_high = can_data[3]
    speed_low = can_data[4]
    speed_high = can_data[5]
    encoder_position_low = can_data[6]
    encoder_position_high = can_data[7]
    
    # Combine bytes to get 16-bit values
    torque_current = (torque_current_high << 8) | torque_current_low
    speed = (speed_high << 8) | speed_low
    encoder_position = (encoder_position_high << 8) | encoder_position_low
    
    
    if torque_current > 2048:
        torque_current = -(65536 - torque_current)
    torque_current = map_value(torque_current, -2048, 2048, -33, 33)
    encoder_position = map_value(encoder_position, 0, 16383, 0, 40)
    
    if speed > 32768:
        speed = -(65536 - speed)
    speed = speed/9
    
    return torque_current, speed, encoder_position


def return_command_torque(torque):
    # This method converges the input Torque
    # into a data string that can be sent on the CAN
    if torque>2000 or torque <-2000:
        return
    data=[0xA1,0x00,0x00,0x00,0x00,0x00,0x00,0x00]
    if torque==0:
        return data
    if torque < 0 :
        hex_torque=NegToHex(torque,4)
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



def read_status(bus,id, debug = False):
    # This Method send command to read the values in the motors
    send_msg = can.Message(arbitration_id=id, data=[0x9C,0x00,0x00,0x00,0x00,0x00,0x00,0x00], is_extended_id=False)
    bus.send(send_msg)

    Response = bus.recv(0.01)
    if Response is None:
        # print("No response received.") # only print for debug - slows prcoess down
        return None
    can_data = Response.data
    # print(f"Raw feedback: {[hex(b) for b in can_data]}")
   
    command_byte = can_data[0]
    motor_temperature = can_data[1]
    torque_current_low = can_data[2]
    torque_current_high = can_data[3]
    speed_low = can_data[4]
    speed_high = can_data[5]
    encoder_position_low = can_data[6]
    encoder_position_high = can_data[7]
    
    # Combine bytes to get 16-bit values
    torque_current = (torque_current_high << 8) | torque_current_low
    speed = (speed_high << 8) | speed_low
    encoder_position = (encoder_position_high << 8) | encoder_position_low
    
    
    if torque_current > 2048: # 32768
        torque_current = -(65536 - torque_current)
    torque_current = map_value(torque_current, -2048, 2048, -33, 33)
    encoder_position = map_value(encoder_position, 0, 65536, 0, 40)
    
    if speed > 32768:
        speed = -(65536 - speed)
    speed = speed/9
    
    # Converting the Current Torque into an actual torque
    torque_constant = 2.09  # in Nm/A - Found in the Data Sheet
    torque_ = torque_current * torque_constant
    
    
    #print(f"The Torque Current is: {torque_current:7.3f} A | Motor Torque is: {torque_:7.3f} Nm")
    
    return torque_current, speed, encoder_position, torque_


def map_value(value, from_low, from_high, to_low, to_high):
    # Scale the value from the input range to the output range
    return (value - from_low) * (to_high - to_low) / (from_high - from_low) + to_low


def position_control(bus,id,position,speed):
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

    #print("The Message sent to the motor are\n", data)
    # bus = can.interface.Bus(channel='can0', bustype='socketcan')
    # print(bus)
    
    send_msg = can.Message(arbitration_id=id, data=data, is_extended_id=False)
    #print("\n In computor language it is \n", send_msg)
    bus.send(send_msg)

    msg = bus.recv(0.1) #read the data to aviod block, timeout is 4s
    if msg is None:
        # print("No response received.")    # only print for debug - slows prcoess down
        return None
    # print("\n", msg)
    # new_pos=self.read_pos.read_angle()
    # print(str(hex(self.id))+' '+'new position:'+ str(new_pos))
    # Extracting data from the CAN message
    can_data = msg.data
   
    command_byte = can_data[0]
    motor_temperature = can_data[1]
    torque_current_low = can_data[2]
    torque_current_high = can_data[3]
    speed_low = can_data[4]
    speed_high = can_data[5]
    encoder_position_low = can_data[6]
    encoder_position_high = can_data[7]
    
    # Combine bytes to get 16-bit values
    torque_current = (torque_current_high << 8) | torque_current_low
    speed = (speed_high << 8) | speed_low
    encoder_position = (encoder_position_high << 8) | encoder_position_low
    
    
    if torque_current > 2048:
        torque_current = -(65536 - torque_current)
    torque_current = map_value(torque_current, -2048, 2048, -33, 33)
    encoder_position = map_value(encoder_position, 0, 16383, 0, 40)
    
    if speed>2048:
        speed=-(65536 - speed)/62
    else:
        speed=speed/62
    #print(f"Motor Temperature are: {motor_temperature}")
    #print(f"Motor Torque are: {torque_current}")
    return torque_current, speed, encoder_position

def read_angle2(bus,id):
    # bus = can.interface.Bus(channel=self.can_id, bustype='socketcan')
    # print(bus)
    
    send_msg = can.Message(arbitration_id=id, data=[0x92,0x00,0x00,0x00,0x00,0x00,0x00,0x00], is_extended_id=False)
    # print(send_msg)
    bus.send(send_msg)
    # sleep(0.0001)
    msg = str(bus.recv(4))
    #print(msg)
    
    data=msg[76:99]
    if data[0:2]!='92':
        return None
    data=data.replace(' ','')
    encoder_data=data[2:16]
    if encoder_data[-1]=='f': #if the position is negetive
        add='ff'
        encoder_data=encoder_data+add #
        neghex_pack=binascii.unhexlify(encoder_data)
        neg=struct.unpack('q', neghex_pack) #long long type int
        encoder_dec=neg[0]
    else:
        encoder_data=re.findall(r'\w{1,2}',encoder_data) #divide the data to groups, '3e92' to ['3e','92']
        encoder_data=encoder_data[::-1]
        encoder_data = ''.join(encoder_data)
        encoder_dec=int(encoder_data,16)
    return float(encoder_dec)/100/9


def read_angle(bus,id):
    # bus = can.interface.Bus(channel=self.can_id, bustype='socketcan')
    # print(bus)
    
    send_msg = can.Message(arbitration_id=id, data=[0x92,0x00,0x00,0x00,0x00,0x00,0x00,0x00], is_extended_id=False)
    # print(send_msg)
    bus.send(send_msg)
    # sleep(0.0001)
    msg = bus.recv(0.01)    
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



def Read_Torque_Current(bus, id):
    data = [0x9C,0x00,0x00,0x00,0x00,0x00,0x00,0x00]
    # Using Command 0x9C from the motor Protocol
    # This method will read the Torque Current
    send_msg = can.Message(
                        arbitration_id=id, 
                        data=data, 
                        is_extended_id=False)
    bus.send(send_msg)
    time.sleep(0.0001)
    while True:
        msg = bus.recv(1)
        if msg is not None:
            if msg.arbitration_id == id and msg.data[0] == 0x9C:
                break
    can_data = msg.data
    # Based on the Motor Protocol the Torque Current is sent as two bytes
    # can_data[2] is the low byte
    # can_data[3] is the high byte,
    torque_current_low = can_data[2]
    torque_current_high = can_data[3]

    # we need to combine them to get the actual torque current value
    # Combine bytes to get 16-bit values
    torque_current = (torque_current_high << 8) | torque_current_low
    if torque_current > 2048:
        torque_current = -(65536 - torque_current)
    torque_current = map_value(torque_current, -2048, 2048, -33, 33) 

    # Converting the Torque Current into an actual Torque value
    torque = torque_current * 2.09 # 2.09 Nm/A from the datasheet of the motor
    return torque_current, torque


def Read_Speed(bus,id):
    # This method is used to find out the speed of the motor
    data = [0x9C,0x00,0x00,0x00,0x00,0x00,0x00,0x00]
    #
    send_msg = can.Message(
                        arbitration_id=id, 
                        data=data, 
                        is_extended_id=False)
    bus.send(send_msg)
    Responce = bus.recv(0.01)
    if Responce is None:
        # print("No response received.")    # only print for debug - slows prcoess down
        return None
    can_data = Responce.data

    # Inserting the Values on the bytes
    speed_low = can_data[4]
    speed_high = can_data[5]

    # we need to combine them to get the actual speed value
    # Combine bytes to get 16-bit values
    speed = (speed_high << 8) | speed_low

    if speed > 32768:
        speed = -(65536 - speed)
    speed = speed / 9 

    return speed