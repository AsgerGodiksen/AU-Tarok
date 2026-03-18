#!/bin/bash
# bin//media/pi/3C11-A839/JP/ASGER_TESTER/3_Motor_Calibration.py

from ASGER_METHODS import*


# This code will be used to calibrate 3 moters on one leg so the Zero Position are also as on the motors

if __name__ == "__main__":
    try:
        bus0 = can.interface.Bus(channel="can0", interface="socketcan")
        bus0.flush_tx_buffer()		# Clean any CAN signals on the line
        for i in range(100):		# Now Listens for any signals 100 times with 0,001 s in between. Prints if any.
            msg = bus0.recv(0.01)
            if msg:
                print(msg)
        # ID depends on leg        
        id_1 = 0x141
        id_2 = 0x142
        id_3 = 0x143
        #id_1 = 0x141
        #id_2 = 0x142
        #id_3 = 0x143
        i = 0
        while True:
            msg = bus0.recv(0.05)
            i = i+1
            
            torque_current_1, speed_1, encoder_position_1 = torque_control(bus0,id_1,1)
            torque_current_2, speed_2, encoder_position_2 = torque_control(bus0,id_2,1)
            torque_current_3, speed_3, encoder_position_3 = torque_control(bus0,id_3,1)
            
            #print("  ")
            #print(f"Position For Motor 1: {encoder_position_1:.2f}")
            #print(f"Position For Motor 2: {encoder_position_2:.2f}")
            #print(f"Position For Motor 3: {encoder_position_3:.2f}")
            #print("==========  ======== ")
            time.sleep(0.5)
            
            
            Angle_1 = read_angle(bus0,id_1)
            Angle_2 = read_angle(bus0,id_2)
            Angle_3 = read_angle(bus0,id_3)

            print("  ")
            print(f"Position For Motor 1: {Angle_1}")
            print(f"Position For Motor 2: {Angle_2}")
            print(f"Position For Motor 3: {Angle_3}")
            print("========== ======== ",{i})
            time.sleep(1)
            
    except KeyboardInterrupt:
    # Handle the keyboard interrupt (Ctrl+C)
        send_msg_1 = can.Message(arbitration_id=id_1, data=[0x19,0x00,0x00,0x00,0x00,0x00,0x00,0x00], is_extended_id=False) # Calibration
        send_msg_2 = can.Message(arbitration_id=id_2, data=[0x19,0x00,0x00,0x00,0x00,0x00,0x00,0x00], is_extended_id=False) # Calibration
        send_msg_3 = can.Message(arbitration_id=id_3, data=[0x19,0x00,0x00,0x00,0x00,0x00,0x00,0x00], is_extended_id=False) # Calibration

        # The 0x19 makes sure that the current position is saved inside the Actuators ROM
        bus0.send(send_msg_1)
        time.sleep(0.0001)
        bus0.send(send_msg_2)
        time.sleep(0.0001)
        bus0.send(send_msg_3)
        time.sleep(0.0001)
        msg = bus0.recv(4)
        print(msg)
        print("Keyboard interrupt detected. Shutting down CAN bus.")
        bus0.flush_tx_buffer()
        bus0.shutdown()