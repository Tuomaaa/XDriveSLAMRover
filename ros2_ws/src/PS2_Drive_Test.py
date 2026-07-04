"""
PS2 → CAN velocity command with x-drive kinematics.

Left stick Y  → vx (forward/backward)
Left stick X  → vy (left/right strafe)
Right stick X → omega (rotation)

X-drive kinematics:
    motor0 = vx + vy + omega
    motor1 = vx - vy - omega
    motor2 = vx - vy + omega
    motor3 = vx + vy - omega

Signs may need flipping depending on motor mounting direction.

Usage: sudo python3 ps2_drive_test.py
"""

import can
import struct
import time
from PS2 import PS2Controller

# ── Config ──
MAX_RPM = 200          # max RPM per motor
DEADZONE = 15          # joystick center deadzone (around 128)
SEND_INTERVAL = 0.05   # 50ms = 20Hz


def map_axis(value):
    """
    Map joystick axis (0~255) to -1.0 ~ +1.0.
    0 = negative max, 128 = center, 255 = positive max.
    """
    centered = value - 128
    if abs(centered) < DEADZONE:
        return 0.0
    return centered / 128.0


def clamp_rpm(rpm):
    """Clamp to ±MAX_RPM and convert to int."""
    return int(max(-MAX_RPM, min(MAX_RPM, rpm)))


def main():
    ps2 = PS2Controller()
    bus = can.Bus(channel='can0', interface='socketcan', bitrate=500000)

    print("PS2 X-Drive Test — Ctrl+C to stop")
    print(f"Max RPM: ±{MAX_RPM}")
    print("Left stick: move  |  Right stick X: rotate")

    try:
        while True:
            data = ps2.read()
            if data is None:
                print("No controller        ", end='\r')
                time.sleep(0.5)
                continue

            # Map joystick axes
            # Left Y: up=0, down=255 → negate so up=positive
            vx    = -map_axis(data['ly'])    # forward/backward
            vy    =  -map_axis(data['lx'])    # left/right strafe
            omega =  map_axis(data['rx'])    # rotation

            # X-drive inverse kinematics
            m0 = (vx + vy + omega) * MAX_RPM
            m1 = (vx - vy - omega) * MAX_RPM
            m2 = (vx - vy + omega) * MAX_RPM
            m3 = (vx + vy - omega) * MAX_RPM

            # Scale down if any motor exceeds MAX_RPM
            max_val = max(abs(m0), abs(m1), abs(m2), abs(m3), 1)
            if max_val > MAX_RPM:
                scale = MAX_RPM / max_val
                m0 *= scale
                m1 *= scale
                m2 *= scale
                m3 *= scale

            rpm0 = clamp_rpm(m0)
            rpm1 = clamp_rpm(m1)
            rpm2 = clamp_rpm(m2)
            rpm3 = clamp_rpm(m3)

            # Send CAN 0x100
            payload = struct.pack('<4h', rpm0, rpm1, rpm2, rpm3)
            msg = can.Message(
                arbitration_id=0x100,
                data=payload,
                is_extended_id=False,
            )
            bus.send(msg)

            print(
                f"vx:{vx:+.2f} vy:{vy:+.2f} ω:{omega:+.2f}  "
                f"M0:{rpm0:+4d} M1:{rpm1:+4d} M2:{rpm2:+4d} M3:{rpm3:+4d}",
                end='\r'
            )

            time.sleep(SEND_INTERVAL)

    except KeyboardInterrupt:
        stop = struct.pack('<4h', 0, 0, 0, 0)
        bus.send(can.Message(arbitration_id=0x100, data=stop, is_extended_id=False))
        print("\nStopped — sent zero RPM")
    finally:
        bus.shutdown()
        ps2.cleanup()


if __name__ == '__main__':
    main()