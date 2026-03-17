
def Balance_Control(Euler_Angle,Pitch_Desired,Roll_Desired,Foot_Positions):
    """
    Docstring for Balance_Control
    
    :param Euler_Angle: Euler angle from the IMU sensor
    :param Pitch_Desired: Description
    :param Roll_Desired: Description
    :param Foot_Positions: Description
    """

    return

def Kalman_Filter(Angle,Previous_Angle_Filter):
    """
    Funtion that uses a Kalman_Filter
    
    :param Angle: Takes the Measured value from the IMU Sensor
    :param Previous_Angle_Filter: Previus Calculated Angle using the filter
    """
    Kalman_Gain = 0.8
    Angle_Filter = Angle * Kalman_Gain + (1 - Kalman_Gain) * Previous_Angle_Filter
    return Angle_Filter



Kal