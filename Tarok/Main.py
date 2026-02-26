# This will be the main file when oberating with TAROK
import can
from time import sleep,time
import numpy as np
from Robot import*


Tarok = Tarok_Dimensions_Class()

print("Starting the Robot")

ID_1 = 0x141
ID_2 = 0x142
ID_3 = 0x143

# Connect to CAN bus
bus0 = can.interface.Bus(bustype='socketcan', channel='can0', bitrate=1000000)
bus1 = can.interface.Bus(bustype='socketcan', channel='can1', bitrate=1000000)
bus2 = can.interface.Bus(bustype='socketcan', channel='can2', bitrate=1000000)

# Flux the channels
bus0.flush_tx_buffer()
bus1.flush_tx_buffer()
bus2.flush_tx_buffer()

print("Initialization complete")
Position_Control(bus2,ID_1,360,100)
Position_Control(bus2,ID_2,360,100)
Position_Control(bus2,ID_3,360,100)


try:
    while True:
        # Read motor data

        # Sleep for a short time before the next read
        sleep(0.5)

 

# Stop loop with Ctrl+C
except KeyboardInterrupt:
    print("KeyboardInterrupt received, shutting down...")
    print("Shutting down CAN bus.")
    bus0.flush_tx_buffer()
    bus1.flush_tx_buffer()
    bus2.flush_tx_buffer()
    Motor_Stop(bus0,ID_1)
    Motor_Stop(bus0,ID_2)
    Motor_Stop(bus0,ID_3)
    Motor_Stop(bus1,ID_1)
    Motor_Stop(bus1,ID_2)
    Motor_Stop(bus1,ID_3)
    Motor_Stop(bus2,ID_1)
    Motor_Stop(bus2,ID_2)
    Motor_Stop(bus2,ID_3)
    bus0.shutdown()
    bus1.shutdown()
    bus2.shutdown()

    print("CAN bus shut down")
    print("Shutdown complete.")