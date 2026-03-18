#!/usr/bin/env python3
"""
Balance Control Test
--------------------
Places the robot in its initial standing configuration on a tiltable plate.
The balance controller continuously adjusts foot positions to keep the torso level.
A warning is logged if the torso error exceeds the allowed threshold.
"""
import time
import numpy as np
import sys, os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..')))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../Hardware')))

from Robot.Kinematics import Inverse_Kinematics,Balance_Control
#from Robot.Kinematics.Constant_Transforms import TB_0
#from Robot.Kinematics.Inverse_Kinematics import*
from Robot.Hardware.IMU_BNO085 import*
#from Robot.Hardware.Motor_Controls import send_joint_angles

# -------------------------------------------------------------------
# Robot geometry
# -------------------------------------------------------------------
L1       = 0.078
TORSO_L  = 0.7048    # matches L_Body inside Pitch_And_Roll.py
TORSO_W  = 0.220     # matches W_Body inside Pitch_And_Roll.py
z = 0.41
 
# Nominal foot positions in the body frame (torso centre at origin)
INITIAL_FOOT_POSITIONS = [
    [ TORSO_L/2,  TORSO_W/2 + L1, -z],   # FL  (idx 0)
    [ TORSO_L/2, -(TORSO_W/2 + L1), -z],  # FR  (idx 1)
    [-TORSO_L/2,  TORSO_W/2 + L1, -z],   # HL  (idx 2)
    [-TORSO_L/2, -(TORSO_W/2 + L1), -z],  # HR  (idx 3)
]

# -------------------------------------------------------------------
# Test parameters
# -------------------------------------------------------------------
FREQ            = 20        # Hz — target control loop frequency
ERROR_THRESHOLD = 5.0       # degrees — log a warning above this

PITCH_DESIRED   = 0.0       # radians — keep torso level
ROLL_DESIRED    = 0.0       # radians — keep torso level




# -------------------------------------------------------------------
# Main test loop
# -------------------------------------------------------------------
def run_test():
    print("[INFO] Starting balance control test...")
    print(f"[INFO] Target: Pitch={PITCH_DESIRED} rad, Roll={ROLL_DESIRED} rad")
    print(f"[INFO] Warning threshold: ±{ERROR_THRESHOLD}°")
    print("[INFO] Press Ctrl+C to stop.\n")
    
    bno, i2c   = IMU_Initialization()
    foot_positions = [row[:] for row in INITIAL_FOOT_POSITIONS]
    dt_target  = 1.0 / FREQ

    try:
        while True:
            loop_start = time.time()

            # --- Read IMU ---
            euler = Quaternion_To_Euler(Get_Quaternion(bno))  # [Pitch, Roll, Yaw] in degrees


            # --- Check error ---
            if abs(euler[0] - PITCH_DESIRED) > ERROR_THRESHOLD:
                print(f"[WARNING] Pitch error {abs(euler[0]):.2f}° exceeds threshold of {ERROR_THRESHOLD}°")
            if abs(euler[1] - ROLL_DESIRED) > ERROR_THRESHOLD:
                print(f"[WARNING] Roll error {abs(euler[1]):.2f}° exceeds threshold of {ERROR_THRESHOLD}°")

            # --- Balance controller (expects radians) ---
            foot_positions = Balance_Control(
                euler_angle    = np.radians(euler),
                pitch_desired  = np.radians(PITCH_DESIRED),
                roll_desired   = np.radians(ROLL_DESIRED),
                foot_positions = foot_positions,
            )

            # --- Transform to shoulder frame and run IK ---
            joint_angles = Inverse_Kinematics((foot_positions))

            # --- Send to motors ---
            send_joint_angles(joint_angles)

            # --- Logging ---
            print(
                f"Pitch: {euler[0]:+.2f}° | "
                f"Roll:  {euler[1]:+.2f}° | "
                f"Loop:  {(time.time() - loop_start)*1e3:.1f} ms"
            )

            # --- Timing ---
            elapsed = time.time() - loop_start
            if dt_target - elapsed > 0:
                time.sleep(dt_target - elapsed)

    except KeyboardInterrupt:
        print("\n[INFO] Test stopped by user.")
        i2c.deinit()
        print("[INFO] I2C connection closed.")


if __name__ == "__main__":
    run_test()