# This file tries to inveestigate the torque readings from the actuators



import can
from Robot.Hardware.Motor_Controls import *

print("Starting the Robot")

ID_1 = 0x141
ID_2 = 0x142
ID_3 = 0x143

# Connect to CAN bus
bus0 = can.interface.Bus(bustype='socketcan', channel='can1', bitrate=1000000)

bus0.flush_tx_buffer()		# Clean any CAN signals on the line
for i in range(100):		# Now Listens for any signals 100 times with 0.01 s in between. Prints if any.
    msg = bus0.recv(0.01)
    if msg:
        print(msg)

print("Initialization complete, moving to zero position")
# Move to zero position
#position_control(bus0,ID_1,0,400)
#position_control(bus0,ID_2,0,400)
#position_control(bus0,ID_3,0,400)