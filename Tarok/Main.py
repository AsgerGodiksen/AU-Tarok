# This will be the main file when oberating with TAROK
import can
from time import sleep,time
#import numpy as np
from Robot import*

# CAN initialization in terminal: sudo ip link set dev canX up type can bitrate 1000000
# with "X" being 0, 1, 2 and 3 for each bus



Tarok = Tarok_Dymensions_Class

print("Starting the Robot")

ID_1 = 0x141
ID_2 = 0x142
ID_3 = 0x143

# Connect to CAN bus
bus0 = can.interface.Bus(interface='socketcan', channel='can0', bitrate=1000000)
bus1 = can.interface.Bus(interface='socketcan', channel='can1', bitrate=1000000)
bus2 = can.interface.Bus(interface='socketcan', channel='can2', bitrate=1000000)
#bus3 = can.interface.Bus(interface='socketcan', channel='can3', bitrate=1000000)

#### REMEMBER BUS 3 when having all 4 legs
# Drain any stale messages from the buses
for bus in [bus0, bus1, bus2]: #bus3
    for i in range(100):		# Now Listens for any signals 100 times with 0.01 s in between. Prints if any.
        msg = bus.recv(0.01)
        if msg:
            print(msg)

print("Initialization complete")
Position_Control(bus0,ID_1,0,100)
Position_Control(bus0,ID_2,0,100)
Position_Control(bus0,ID_3,0,100)

Position_Control(bus1,ID_1,0,100)
Position_Control(bus1,ID_2,0,100)
Position_Control(bus1,ID_3,0,100)

Position_Control(bus2,ID_1,0,100)
Position_Control(bus2,ID_2,0,100)
Position_Control(bus2,ID_3,0,100)




print("Loop started - Press ctrl+c in terminal for shutdown")
try:
    while True:
        # Read motor data
        
        # Sleep for a short time before the next read
        sleep(0.5)




# Stop loop with Ctrl+C
except KeyboardInterrupt:
    print("KeyboardInterrupt received, shutting down...")

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
    #Motor_Stop(bus3,ID_1)
    #Motor_Stop(bus3,ID_2)
    #Motor_Stop(bus3,ID_3)
    print("Motors stopped")

    print("Shutting down CAN buses...")
    bus0.shutdown()
    bus1.shutdown()
    bus2.shutdown()
    #bus3.shutdown()
    print("CAN buses shut down")

    print("Shutdown complete.")