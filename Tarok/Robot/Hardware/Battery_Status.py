import can
from time import sleep, time
#from Motor_Readings import*
#from Motor_Controls import*
from Robot.Hardware.Motor_Readings import*

# This code will be used to find the input voltage to the actuators
#class BatteryProcentageCalculator:
def Voltage_Mapping():
    # Voltage to Procentage Mapping.
    # Values are found from the battery datasheet for one cell

    Voltage_Mapping = [
        (100, 4.17),
        (90, 4.07),
        (80, 3.96),
        (70, 3.905),
        (60, 3.84),
        (50, 3.8),
        (40, 3.775),
        (30, 3.75),
        (20, 3.715),
        (10, 3.67),
        (7.5, 3.66),
        (5, 3.62),
        (2.5, 3.47),
        (0, 3),
        ]
    return Voltage_Mapping

def Calculate_Percentage(Cell_Voltage):
    mapping = Voltage_Mapping()
    # Check if the Cell Voltage is above the maximum
    if Cell_Voltage >= mapping[0][1]:
        print("Careful Battery is OVERCHARGED")
        return 100
    
    # Check if the given voltage is below 20%
    if Cell_Voltage <= mapping[-5][1]:
        print("CHARGE Battery")
        return 0  

    # Perfom Linear Interpolation between the given Voltage points
    for i in range(len(mapping) -1):
        p1, v1 = mapping[i]
        p2, v2 = mapping[i +1]

        if v2 <= Cell_Voltage <= v1:
            Percentage = p1 + (p2 - p1) * (Cell_Voltage - v1) / (v2 - v1)
            return round(Percentage)
    
def Battery_Voltage(bus,id):
    # Function that will print out the average cell voltage
    # And give a Battery procentage that are left
    #bus0 = can.interface.Bus(channel="can1", interface="socketcan")
    Battery_Voltage = Read_Voltage(bus, id)
    print("Battery Voltage: ", f"{Battery_Voltage:7.2f}", "V")
    Cell_Voltage = Battery_Voltage/6
    Voltage_Procentage = Calculate_Percentage(Cell_Voltage)
    print("Cell Voltage: ", round(Cell_Voltage,1), "V  ", "Percentage: ",Voltage_Procentage,"%")
    print("--------------------------------------------------")
    print("")
    #bus0.flush_tx_buffer()
    #bus0.shutdown()
    return Cell_Voltage , Voltage_Procentage
