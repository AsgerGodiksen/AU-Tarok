from time import sleep, time
import can




def Read_Voltage(bus,id):
    data = [0x9A,0x00,0x00,0x00,0x00,0x00,0x00,0x00]
    # Using command 0x9A from the motor protocpol to read voltage
    # Can also read temperature and error state,
    # but for now we are only interested in voltage
    
    send_msg = can.Message(
                        arbitration_id=id, 
                        data=data, 
                        is_extended_id=False)
    
    bus.send(send_msg)
    
    sleep(0.0001)
    while True:
        msg = bus.recv(1)
        if msg is not None:
            if msg.arbitration_id == id and msg.data[0] == 0x9A:
                break

    can_data = msg.data
     # Based on the Motor Protocol the voltage is sent as two bytes
    # can_data[3] is the low byte
    # can_data[4] is the high byte,
    voltage_low = can_data[3]
    voltage_high = can_data[4]  

    # we need to combine them to get the actual voltage value
    
    # Combine bytes to get 16-bit values
    voltage = (voltage_high << 8) | voltage_low
    #print(voltage)
    if voltage > 2048:
        voltage = -(65536 - voltage)
    voltage = voltage*0.1 #0.1V/LSB
    
    return voltage


def Read_Torque_Current(bus, id):
    data = [0x9C,0x00,0x00,0x00,0x00,0x00,0x00,0x00]
    # Using Command 0x9C from the motor Protocol
    # This method will read the Torque Current
    send_msg = can.Message(
                        arbitration_id=id, 
                        data=data, 
                        is_extended_id=False)
    bus.send(send_msg)
    sleep(0.0001)
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
    torque_current = Map_Value(torque_current, -2048, 2048, -33, 33) 

    # Converting the Torque Current into an actual Torque value
    torque = torque_current * 2.09 # 2.09 Nm/A from the datasheet of the motor

    return torque_current, torque


def Read_Encoder_Postion(bus,id):
    # Thios method is used to find out where the encoder is positionen in the +- 32767 degree span
    data = [0x9C,0x00,0x00,0x00,0x00,0x00,0x00,0x00]
    # Using Command 0x9C from the motor Protocol
    send_msg = can.Message(
                        arbitration_id=id, 
                        data=data, 
                        is_extended_id=False)
    bus.send(send_msg)
    sleep(0.0001)
    msg = bus.recv(4)
    can_data = msg.data

    encoder_position_low = can_data[6]
    encoder_position_high = can_data[7]
    # we need to combine them to get the actual encoder position value
    # Combine bytes to get 16-bit values
    encoder_position = (encoder_position_high << 8) | encoder_position_low
    print(f"Encoder Position: {encoder_position}")

    return encoder_position

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



def Read_Motor_Temperature(bus,id):
    # This method is used to find out the temperature of the motor
    data = [0x9A,0x00,0x00,0x00,0x00,0x00,0x00,0x00]
    # Using Command 0x9C from the motor Protocol
    send_msg = can.Message(
                        arbitration_id=id, 
                        data=data, 
                        is_extended_id=False)
    bus.send(send_msg)
    sleep(0.0001)
    msg = bus.recv(4)
    can_data = msg.data

    motor_temperature = can_data[1] / 10
    print(f"Motor Temperature: {motor_temperature}°C")

    return motor_temperature


def Read_Speed(bus,id):
    # This method is used to find out the speed of the motor
    data = [0x9C,0x00,0x00,0x00,0x00,0x00,0x00,0x00]
    #
    send_msg = can.Message(
                        arbitration_id=id, 
                        data=data, 
                        is_extended_id=False)
    bus.send(send_msg)
    while True:
            msg = bus.recv(1)
            if msg is not None:
                if msg.arbitration_id == id and msg.data[0] == 0x9C:
                    break
    can_data = msg.data

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


def Read_PID(bus,id):
    # This method is used to read the PID values of the motor
    data = [0x30,0x00,0x00,0x00,0x00,0x00,0x00,0x00]
    send_msg = can.Message(
                        arbitration_id=id, 
                        data=data, 
                        is_extended_id=False)
    bus.send(send_msg)
    sleep(0.0001)
    msg = bus.recv(4)
    can_data = msg.data

    # We extract the differnt PID values from the CAN data, based on the Motor Protocol
    command_byte = can_data[0]

    Position_Loop_Kp = can_data[2]
    Position_Loop_Ki = can_data[3]
    Speed_Loop_Kp = can_data[4]
    Speed_Loop_Ki = can_data[5]
    Torque_Loop_Kp = can_data[6]
    Torque_Loop_Ki = can_data[7]

    # Now we convert the bytes into a readable format
    #print(f"PID Data: {msg}")
    print("For Bus:",bus,"and ID:",id)
    print(f"Position Loop Kp: {Position_Loop_Kp:5.0f}, and Ki: {Position_Loop_Ki:5.0f}")
    print(f"Speed Loop Kp:    {Speed_Loop_Kp:5.0f}, and Ki: {Speed_Loop_Ki:5.0f}")
    print(f"Torque Loop Kp:   {Torque_Loop_Kp:5.0f}, and Ki: {Torque_Loop_Ki:5.0f}")
    print(" ")

def Map_Value(value, from_low, from_high, to_low, to_high):
    # Scale the value from the input range to the output range
    return (value - from_low) * (to_high - to_low) / (from_high - from_low) + to_low

