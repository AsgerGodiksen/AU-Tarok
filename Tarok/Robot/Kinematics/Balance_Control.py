from Robot.Kinematics.Kalman_Filter import KalmanFilter
from Robot.Kinematics.Pitch_And_Roll import Inverse_Pitch, Inverse_Roll


class BalanceControl:
    """
    
    Balance Control Class
    
    Methods
    -------
    __init__():
        Initialises the controller with PD gains, Kalman filter,
        and control frequency.

    reset():
        Resets the controller state — clears filter history and
        previous errors. Call this before starting a new test
        without restarting Python.

    update(Euler_Angle, Pitch_Desired, Roll_Desired, Foot_Positions):
        Computes one control iteration. Takes IMU angles and current
        foot positions, returns corrected foot positions. 
    """
    def __init__(self):
        """
        :param Freq:        Control loop frequency [Hz].
        :param Kalman_Gain: Kalman filter blending factor, between 0 and 1.
        :param Pitch_p:     Proportional gain for pitch.
        :param Pitch_d:     Derivative gain for pitch.
        :param Roll_p:      Proportional gain for roll.
        :param Roll_d:      Derivative gain for roll.
        """
        self.Freq=20
        self.Kalman_Gain=0.8
        self.PITCH_P=2.5 
        self.PITCH_D=0.3
        self.ROLL_P=2.0  
        self.ROLL_D=0.0
        
        
        # Filter instances
        self.Pitch_Filter = KalmanFilter(Kalman_Gain=self.Kalman_Gain)
        self.Roll_Filter  = KalmanFilter(Kalman_Gain=self.Kalman_Gain)

        # PD controller state
        self.Pitch_Previous_Error = 0.0
        self.Roll_Previous_Error  = 0.0
        
    def reset(self):
        """
        Reset the controller state — clears filter history and previous errors.
        Call this before starting a new balance test without restarting Python.
        """
        self.Pitch_Filter.Previous = 0.0
        self.Roll_Filter.Previous  = 0.0
        self.Pitch_Previous_Error  = 0.0
        self.Roll_Previous_Error   = 0.0

    def update(self, Euler_Angle, Pitch_Desired, Roll_Desired, Foot_Positions):
        """
        Compute corrected foot positions for one control iteration.

        :param Euler_Angle:    [Pitch, Roll, Yaw] from the IMU sensor, in radians.
        :param Pitch_Desired:  Desired pitch angle of the torso, in radians.
        :param Roll_Desired:   Desired roll angle of the torso, in radians.
        :param Foot_Positions: Current foot positions as a 4x3 plain list [[x,y,z], ...]
                               in the body frame. Order: [FL, FR, HL, HR].
                               On each iteration this should be the corrected positions
                               from the previous iteration, not the nominal positions.
        :returns: New_foot_positions — corrected 4x3 foot position list [FL, FR, HL, HR]
        """
        # Uses Kalman Filter on the Raw Measurements
        Pitch_Measured = self.Pitch_Filter.update(Euler_Angle[0])
        Roll_Measured  = self.Roll_Filter.update(Euler_Angle[1])
        
        # Timestep
        dt = 1 / self.Freq
        
        # Errors
        Pitch_Error = Pitch_Desired - Pitch_Measured
        Roll_Error  = Roll_Desired  - Roll_Measured
        
        # Implementing the Contributions from both the p and d terms
        Pitch_p = Pitch_Error * self.PITCH_P
        Roll_p  = Roll_Error  * self.ROLL_P
        Pitch_d = (Pitch_Error - self.Pitch_Previous_Error) * self.PITCH_D / dt
        Roll_d  = (Roll_Error  - self.Roll_Previous_Error)  * self.ROLL_D  / dt
        
        # Combineing the terms
        Pitch_pd = Pitch_p + Pitch_d
        Roll_pd  = Roll_p + Roll_d

        # Scale PD output by dt to make corrections incremental
        Pitch_Increment = Pitch_pd * dt
        Roll_Increment  = Roll_pd  * dt
        
        # Updating for next Iteration
        self.Pitch_Previous_Error = Pitch_Error
        self.Roll_Previous_Error  = Roll_Error
        
        # Inverse kinematics
        New_Foot_Positions_From_Pitch, shoulder_heights = Inverse_Pitch(Pitch_Increment, Foot_Positions)
        New_Foot_Positions = Inverse_Roll(Roll_Increment, New_Foot_Positions_From_Pitch, shoulder_heights)

        return New_Foot_Positions
