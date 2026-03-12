import time
import smbus2
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import BNO_REPORT_ROTATION_VECTOR
from SMBUS2I2C import SMBus2I2C


def IMU_Initialization():
    # --- Initialisation ---

    # Give the sensor time to power up before attempting communication
    i2c = SMBus2I2C(1, 0x4A)
    time.sleep(1.0)

    # Give the sensor time to complete its internal boot sequence
    bno = BNO08X_I2C(i2c, address=0x4A)
    time.sleep(1.0)

    bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
    print("Feature enabled!")
    return bno


def Get_Quaternion(bno):
    rot_x, rot_y, rot_z, rot_w = bno.quaternion
    print(f"X: {rot_x:.4f}  Y: {rot_y:.4f}  Z: {rot_z:.4f}  W: {rot_w:.4f}")



if __name__ == "__main__":
    pass
try:
    while True:
        quat_i, quat_j, quat_k, quat_real = bno.quaternion
        print(f"I: {quat_i:.6f}  J: {quat_j:.6f}  K: {quat_k:.6f}  Real: {quat_real:.6f}")
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Stopping...")
    i2c.deinit()
    print("I2C connection closed.")