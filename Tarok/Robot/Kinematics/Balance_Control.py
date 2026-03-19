from Kalman_Filter import KalmanFilter
from Pitch_And_Roll import Inverse_Pitch, Inverse_Roll


# --- Filter instances (one per axis) ---
_pitch_filter = KalmanFilter(Kalman_Gain=0.8)
_roll_filter  = KalmanFilter(Kalman_Gain=0.8)

# --- PD controller state ---
_pitch_prev_error = 0.0
_roll_prev_error  = 0.0

# --- PD gains ---
PITCH_P, PITCH_D = 2.5, 0.3
ROLL_P,  ROLL_D  = 2.0, 0.0

FREQ = 20  # Hz - How Often the Balance_Control runs each second.



def Balance_Control(Euler_Angle, Pitch_Desired, Roll_Desired, Foot_Positions):
    """
    :param Euler_Angle:    [Pitch, Roll, Yaw] from the IMU sensor, in radians.
    :param Pitch_Desired:  Desired pitch of the torso, in radians.
    :param Roll_Desired:   Desired roll of the torso, in radians.
    :param Foot_Positions: Current foot positions before balance adjustment.
    :returns: Adjusted foot positions after pitch and roll compensation.
    """
    global _pitch_prev_error, _roll_prev_error

    # Filter raw measurements
    pitch_measured = _pitch_filter.update(Euler_Angle[0])
    roll_measured  = _roll_filter.update(Euler_Angle[1])

    # PD control
    dt = 1 / FREQ
    pitch_error = Pitch_Desired - pitch_measured
    roll_error  = Roll_Desired  - roll_measured


        # Maybe a sign error in the parenteses
    # Implementing the Contributions from both the p and d terms
    Pitch_d = (pitch_error - _pitch_prev_error) * PITCH_D / dt
    Roll_d  = (roll_error  - _roll_prev_error)  * ROLL_D  / dt
    Pitch_p = pitch_error * PITCH_P
    Roll_p  = roll_error  * ROLL_P
    
    # Combineing the terms
    pitch_pd = Pitch_p + Pitch_d
    roll_pd  = Roll_p + Roll_d

    _pitch_prev_error = pitch_error
    _roll_prev_error  = roll_error

    # Inverse kinematics
    New_foot_positions_From_Pitch, shoulder_heights = Inverse_Pitch(pitch_pd, Foot_Positions)
    New_foot_positions = Inverse_Roll(roll_pd, New_foot_positions_From_Pitch, shoulder_heights)

    return New_foot_positions , pitch_pd, roll_pd
