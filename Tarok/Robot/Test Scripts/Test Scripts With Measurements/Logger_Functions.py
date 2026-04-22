import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..')))

import can
import time
import numpy as np
import struct
import csv
import concurrent.futures

from Robot import *



# ─────────────────────────────────────────────────────────────────────────────
# Position_Control_With_Feedback  — returns torque AND actual position
# ─────────────────────────────────────────────────────────────────────────────
def Position_Control_With_Feedback(bus, motor_id, new_position, max_speed, timeout=0.005):
    """
    Send a 0xA4 position command and a 0x9C read command per motor step.

    Step 1 — 0xA4 position command:
      Reply bytes [2-3]: iq current (int16) → torque_Nm

    Step 2 — 0x92 read multi-turn angle (mirrors Read_Angle in Motor_Readings.py):
      Reply bytes [1-7]: signed 56-bit angle, 0.01 deg/LSB motor shaft → / 900 = output deg

    Returns (torque_Nm, angle_deg), or (None, None) on timeout.
    """
    # ── Step 1: send 0xA4 position command, read torque from reply ────────────
    data = [0xA4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]

    speed_raw = int(max_speed * 9)
    speed_bytes = struct.pack('<h', speed_raw)
    data[2] = speed_bytes[0]
    data[3] = speed_bytes[1]

    if new_position >= 0:
        pos_enc = int(new_position * 100 * 9)
        pos_bytes = struct.pack('<i', pos_enc)
    else:
        pos_enc = int(new_position * 100 * 9) & 0xFFFFFFFF
        pos_bytes = struct.pack('<I', pos_enc)

    data[4] = pos_bytes[0]
    data[5] = pos_bytes[1]
    data[6] = pos_bytes[2]
    data[7] = pos_bytes[3]

    msg_a4 = can.Message(arbitration_id=motor_id, data=data, is_extended_id=False)
    for attempt in range(3):
        try:
            bus.send(msg_a4)
            break
        except can.CanOperationError:
            time.sleep(0.001)
    else:
        return None, None

    deadline = time.monotonic() + timeout
    torque_Nm = None
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return None, None
        reply = bus.recv(remaining)
        if reply is None:
            return None, None
        if reply.arbitration_id == motor_id and reply.data[0] == 0xA4:
            iq_raw    = struct.unpack('<h', bytes([reply.data[2], reply.data[3]]))[0]
            torque_Nm = (iq_raw / 2048.0) * 33.0 * 2.09
            break

    # ── Step 2: send 0x92 read angle command, decode multi-turn position ─────
    # Mirrors Read_Angle in Motor_Readings.py exactly.
    msg_92 = can.Message(arbitration_id=motor_id,
                         data=[0x92, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
                         is_extended_id=False)
    try:
        bus.send(msg_92)
    except can.CanOperationError:
        return torque_Nm, None

    deadline = time.monotonic() + timeout
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return torque_Nm, None
        reply = bus.recv(remaining)
        if reply is None:
            return torque_Nm, None
        if reply.arbitration_id == motor_id and reply.data[0] == 0x92:
            # Bytes [1-7]: signed 56-bit angle, little-endian, 0.01 deg/LSB (motor shaft)
            raw_bytes  = bytes(reply.data[1:8]) + b'\x00'  # pad to 8 bytes for int64
            angle_raw  = struct.unpack('<q', raw_bytes)[0]
            if reply.data[7] & 0x80:                        # sign-extend from bit 55
                angle_raw -= (1 << 56)
            angle_deg = float(angle_raw) / 900.0            # output shaft degrees
            return torque_Nm, angle_deg


def send_leg(bus, angles, speeds, motor_ids):
    """
    Command one leg's 3 motors and collect torque + position feedback.
    Returns list of (torque_Nm, position_deg) for [J1, J2, J3].
    """
    return [
        Position_Control_With_Feedback(bus, motor_id, angle, speed)
        for motor_id, angle, speed in zip(motor_ids, angles, speeds)
    ]


def safe_torque(fb, i):
    """Return torque_Nm from feedback, 0.0 on timeout."""
    val = fb[i][0]
    return val if val is not None else 0.0


def safe_position(fb, i):
    """Return encoder position (0-65535) from feedback, 0 on timeout."""
    val = fb[i][1]
    return val if val is not None else 0