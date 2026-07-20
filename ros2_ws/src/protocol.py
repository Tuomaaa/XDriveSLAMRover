"""SummerSLAM CAN protocol.

Sensor feedback IDs:
  0x200: cumulative encoder ticks for motors 0/1
  0x201: cumulative encoder ticks for motors 2/3
  0x202: right/back ultrasonic distances and validity bits
"""
from enum import IntEnum
import struct
from dataclasses import dataclass


# ── Message IDs ──

class MsgId(IntEnum):
    VEL_CMD   = 0x100
    ENCODER_0 = 0x200   # Motor 0 + Motor 1
    ENCODER_1 = 0x201   # Motor 2 + Motor 3
    ULTRASONIC = 0x202  # right_mm + back_mm + validity status
    HEARTBEAT = 0x300
    PID_KP    = 0x400
    PID_KI    = 0x401
    PID_KD    = 0x402

ERROR_CATEGORY = 0x000
ACK_CATEGORY   = 0x500
CATEGORY_MASK  = 0x700

HEARTBEAT_PERIOD_MS  = 100
HEARTBEAT_TIMEOUT_MS = 200



@dataclass
class EncoderMsg:
    motor_id: int        # 0 = motor 0+1, 1 = motor 2+3
    motor_a_ticks: int
    motor_b_ticks: int

@dataclass
class ErrorMsg:
    error_code: int      # arbitration ID low 8 bits
    data: bytes          # raw payload, format TBD by STM32 firmware

@dataclass
class AckMsg:
    cmd_id: int          # byte0: sub-ID of the original command being ACK'd
    data: bytes          # byte1+: optional return data



def msg_category(arbitration_id: int) -> int:
    return arbitration_id & CATEGORY_MASK



def encode_vel_cmd(rpm0: int, rpm1: int, rpm2: int, rpm3: int) -> bytes:
    return struct.pack('<4h', rpm0, rpm1, rpm2, rpm3)

def encode_heartbeat() -> bytes:
    return b''

def encode_pid(motor: int, value: float) -> bytes:
    return struct.pack('<Bf', motor, value)

def encode_ultrasonic(right_mm: int, back_mm: int, status: int) -> bytes:
    """Encode the STM32->RPi 0x202 payload (primarily for tests)."""
    return struct.pack('<HHB', right_mm, back_mm, status)



def decode(arbitration_id: int, data: bytes):
    # Fixed-ID messages
    if arbitration_id == MsgId.ENCODER_0:
        motor_a, motor_b = struct.unpack('<2i', data)
        return EncoderMsg(motor_id=0, motor_a_ticks=motor_a, motor_b_ticks=motor_b)

    if arbitration_id == MsgId.ENCODER_1:
        motor_a, motor_b = struct.unpack('<2i', data)
        return EncoderMsg(motor_id=1, motor_a_ticks=motor_a, motor_b_ticks=motor_b)

    if arbitration_id == MsgId.ULTRASONIC:
        right_mm, back_mm, status = struct.unpack('<HHB', data)
        return {
            "right_mm": right_mm,
            "back_mm": back_mm,
            "right_valid": bool(status & 0x01),
            "back_valid": bool(status & 0x02),
        }

    # Category messages
    category = msg_category(arbitration_id)

    if category == ERROR_CATEGORY:
        error_code = arbitration_id & 0xFF
        return ErrorMsg(error_code=error_code, data=data)

    if category == ACK_CATEGORY:
        cmd_id = data[0] if len(data) > 0 else 0
        remaining = data[1:] if len(data) > 1 else b''
        return AckMsg(cmd_id=cmd_id, data=remaining)

    return None

if __name__ == '__main__':
    payload = encode_ultrasonic(203, 0xFFFF, 0x01)
    assert len(payload) == 5
    assert decode(MsgId.ULTRASONIC, payload) == {
        "right_mm": 203,
        "back_mm": 0xFFFF,
        "right_valid": True,
        "back_valid": False,
    }
    print('protocol smoke tests passed')
