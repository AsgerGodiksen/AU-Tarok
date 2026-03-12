import time
import numpy as np
import smbus2
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import BNO_REPORT_ROTATION_VECTOR
from numpy.typing import NDArray
from SMBUS2I2C import SMBus2I2C


def skew(v: NDArray) -> NDArray:
    v = v.flatten()
    return np.array([[0, -v[2], v[1]],
                     [v[2], 0, -v[0]],
                     [-v[1], v[0], 0]])


def quat_to_rot_matrix(quaternion: NDArray) -> NDArray:
    q = quaternion[:3]   # [i, j, k] - vector part
    q0 = quaternion[3]   # real/w - scalar part

    q_tilt = skew(q)
    rot = np.eye(3) + 2 * (q0 * np.eye(3) + q_tilt) @ q_tilt
    return rot


def IMU_Initialization():
    i2c = SMBus2I2C(1, 0x4A)
    time.sleep(2.0)

    bno = BNO08X_I2C(i2c, address=0x4A)
    time.sleep(2.0)

    for attempt in range(5):
        try:
            bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
            print("Feature enabled!")
            return bno, i2c
        except RuntimeError as e:
            print(f"Attempt {attempt + 1} failed: {e}")
            time.sleep(1.0)

    raise RuntimeError("Could not enable BNO085 feature after 5 attempts")


def Get_Quaternion(bno) -> NDArray:
    quat_i, quat_j, quat_k, quat_real = bno.quaternion

    # Pack into numpy array [i, j, k, real] = [x, y, z, w]
    quaternion = np.array([quat_i, quat_j, quat_k, quat_real])
    return quaternion

def Quaternion_To_Euler(quaternion: NDArray) -> NDArray:
    i, j, k, w = quaternion

    # Roll (x-axis rotation)
    roll = np.arctan2(2*(w*i + j*k), 1 - 2*(i**2 + j**2))

    # Pitch (y-axis rotation)
    pitch = np.arcsin(2*(w*j - k*i))

    # Yaw (z-axis rotation)
    yaw = np.arctan2(2*(w*k + i*j), 1 - 2*(j**2 + k**2))

    # Convert to degrees
    return np.degrees(np.array([roll, pitch, yaw]))

if __name__ == "__main__":
    bno, i2c = IMU_Initialization()

    try:
        while True:
            quaternion = Get_Quaternion(bno)

            # Option 1 - from quaternion directly
            angles = Quaternion_To_Euler(quaternion)

            print(f"Roll:  {angles[0]:.2f}°")
            print(f"Pitch: {angles[1]:.2f}°")
            print(f"Yaw:   {angles[2]:.2f}°")
            print("---")

            time.sleep(1)

    except KeyboardInterrupt:
        print("Stopping...")
        i2c.deinit()
        print("I2C connection closed.")