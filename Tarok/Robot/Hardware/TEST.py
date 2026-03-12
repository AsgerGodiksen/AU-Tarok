import time
import smbus2
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import BNO_REPORT_ROTATION_VECTOR

class SMBus2I2C:
    def __init__(self, bus_num, address):
        self._bus = smbus2.SMBus(bus_num)
        self._address = address

    def readfrom_into(self, addr, buf, start=0, end=None):
        if end is None:
            end = len(buf)
        length = end - start
        # Use raw I2C read instead of SMBus block read
        msg = smbus2.i2c_msg.read(addr, length)
        self._bus.i2c_rdwr(msg)
        for i, b in enumerate(msg):
            buf[start + i] = b

    def writeto(self, addr, buf, start=0, end=None):
        if end is None:
            end = len(buf)
        # Use raw I2C write instead of SMBus block write
        msg = smbus2.i2c_msg.write(addr, list(buf[start:end]))
        self._bus.i2c_rdwr(msg)

    def writeto_then_readfrom(self, addr, out_buf, in_buf, out_start=0, out_end=None, in_start=0, in_end=None):
        if out_end is None:
            out_end = len(out_buf)
        if in_end is None:
            in_end = len(in_buf)
        write = smbus2.i2c_msg.write(addr, list(out_buf[out_start:out_end]))
        read = smbus2.i2c_msg.read(addr, in_end - in_start)
        self._bus.i2c_rdwr(write, read)
        for i, b in enumerate(read):
            in_buf[in_start + i] = b

    def try_lock(self): return True
    def unlock(self): pass
    def deinit(self): self._bus.close()

i2c = SMBus2I2C(1, 0x4A)
time.sleep(1.0)

bno = BNO08X_I2C(i2c, address=0x4A)
time.sleep(1.0)

bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
print("Feature enabled!")

while True:
    quat_i, quat_j, quat_k, quat_real = bno.quaternion
    print(f"I: {quat_i:.6f}  J: {quat_j:.6f}  K: {quat_k:.6f}  Real: {quat_real:.6f}")
    time.sleep(0.1)