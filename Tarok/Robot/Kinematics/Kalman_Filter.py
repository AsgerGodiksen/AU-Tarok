

class KalmanFilter:
    """
        Attributes:
        gain (float): The Kalman gain, between 0 and 1. Defaults to 0.8.
        previous (float): The last filtered output, initialised to 0.0.
    """
    def __init__(self, Kalman_Gain=0.8):
        """
        :param gain: Blending factor between 0 and 1. Defaults to 0.8.
        :type gain: float
        """
        self.Kalman_Gain = Kalman_Gain
        self.Previous = 0.0

    def update(self, Angle):
        """
        :param angle: Raw measured angle from the IMU sensor.
        :type angle: float
        :returns: The filtered angle in radians.
        :rtype: float
        """

        Filtered_Angle = Angle * self.Kalman_Gain + (1 - self.Kalman_Gain) * self.Previous
        self.Previous = Filtered_Angle
        
        return Filtered_Angle