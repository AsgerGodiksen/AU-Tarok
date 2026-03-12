import time
import smbus2
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import BNO_REPORT_ACCELEROMETER


class SMBus2I2C:
    """
    Custom I2C wrapper for the Adafruit BNO085 IMU on Raspberry Pi 5.

    The standard Adafruit Blinka I2C driver ignores the I2C frequency set
    in /boot/firmware/config.txt and falls back to 100kHz. The BNO085
    requires 400kHz to enable features reliably, causing enable_feature()
    to fail with a RuntimeError.

    This wrapper bypasses Blinka entirely and communicates directly with
    the Linux kernel I2C driver via smbus2 using raw I2C messages (i2c_msg)
    instead of SMBus protocol calls. This is necessary because the BNO085
    does not use register-based addressing, making standard SMBus block
    read/write functions incompatible.

    Requirements:
        - /boot/firmware/config.txt must contain:
            dtparam=i2c_arm=on
            dtparam=i2c_arm_baudrate=400000
        - smbus2 must be installed: pip install smbus2

    Usage:
        i2c = SMBus2I2C(1, 0x4A)
        bno = BNO08X_I2C(i2c, address=0x4A)
    """

    def __init__(self, bus_num, address):
        """
        Initialise the SMBus2 connection.

        Args:
            bus_num (int): Linux I2C bus number. Use 1 for the standard
                           Raspberry Pi I2C bus (/dev/i2c-1).
            address (int): I2C address of the target device.
                           BNO085 default is 0x4A, alternate is 0x4B.
        """
        self._bus = smbus2.SMBus(bus_num)
        self._address = address

    def readfrom_into(self, addr, buf, start=0, end=None):
        """
        Read bytes from the I2C device into a buffer.

        Uses a raw i2c_msg read instead of SMBus block read, because
        the BNO085 does not use register-based addressing. SMBus block
        reads prepend a register byte which causes an OSError on the BNO085.

        Args:
            addr  (int):       I2C address of the device to read from.
            buf   (bytearray): Buffer to read data into.
            start (int):       Start index in buf to write into. Default 0.
            end   (int|None):  End index (exclusive) in buf. Defaults to len(buf).
        """
        if end is None:
            end = len(buf)
        length = end - start

        # Use raw I2C read — no register byte prepended
        msg = smbus2.i2c_msg.read(addr, length)
        self._bus.i2c_rdwr(msg)

        for i, b in enumerate(msg):
            buf[start + i] = b

    def writeto(self, addr, buf, start=0, end=None):
        """
        Write bytes to the I2C device.

        Uses a raw i2c_msg write instead of SMBus block write, because
        the BNO085 does not use register-based addressing. SMBus block
        writes prepend a register byte which the BNO085 does not expect.

        Args:
            addr  (int):       I2C address of the device to write to.
            buf   (bytearray): Buffer containing bytes to write.
            start (int):       Start index in buf to read from. Default 0.
            end   (int|None):  End index (exclusive) in buf. Defaults to len(buf).
        """
        if end is None:
            end = len(buf)

        # Use raw I2C write — no register byte prepended
        msg = smbus2.i2c_msg.write(addr, list(buf[start:end]))
        self._bus.i2c_rdwr(msg)

    def writeto_then_readfrom(self, addr, out_buf, in_buf,
                               out_start=0, out_end=None,
                               in_start=0, in_end=None):
        """
        Perform a combined write-then-read as a single atomic I2C transaction.

        Required by the Adafruit bus device library for certain internal
        communication patterns. Uses i2c_rdwr() to send both messages
        together without a STOP condition in between.

        Args:
            addr      (int):       I2C address of the device.
            out_buf   (bytearray): Buffer containing bytes to write.
            in_buf    (bytearray): Buffer to read response bytes into.
            out_start (int):       Start index in out_buf. Default 0.
            out_end   (int|None):  End index in out_buf. Defaults to len(out_buf).
            in_start  (int):       Start index in in_buf. Default 0.
            in_end    (int|None):  End index in in_buf. Defaults to len(in_buf).
        """
        if out_end is None:
            out_end = len(out_buf)
        if in_end is None:
            in_end = len(in_buf)

        write = smbus2.i2c_msg.write(addr, list(out_buf[out_start:out_end]))
        read = smbus2.i2c_msg.read(addr, in_end - in_start)

        # Send both messages as one atomic transaction
        self._bus.i2c_rdwr(write, read)

        for i, b in enumerate(read):
            in_buf[in_start + i] = b

    def try_lock(self):
        """
        Required by the Adafruit bus device interface.

        Bus locking is handled by the Linux kernel driver,
        so this always returns True without doing anything.

        Returns:
            bool: Always True.
        """
        return True

    def unlock(self):
        """
        Required by the Adafruit bus device interface.

        No operation — bus unlocking is handled by the Linux kernel driver.
        """
        pass

    def deinit(self):
        """
        Close the SMBus connection and release the file descriptor.

        Call this when finished using the sensor to free system resources.
        """
        self._bus.close()


# --- Initialisation ---

# Give the sensor time to power up before attempting communication
i2c = SMBus2I2C(1, 0x4A)
time.sleep(1.0)

# Give the sensor time to complete its internal boot sequence
bno = BNO08X_I2C(i2c, address=0x4A)
time.sleep(1.0)

bno.enable_feature(BNO_REPORT_ACCELEROMETER)
print("Feature enabled!")

#while True:
#    accel_x, accel_y, accel_z = bno.acceleration
#    print(f"X: {accel_x:.4f}  Y: {accel_y:.4f}  Z: {accel_z:.4f}")
#    time.sleep(0.1)