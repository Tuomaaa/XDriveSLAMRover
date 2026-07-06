"""
PS2 → CAN velocity command with x-drive kinematics.

Left stick Y  → vx (forward/backward)
Left stick X  → vy (left/right strafe)
Right stick X → omega (rotation)
D-pad         → fine-adjust crawl (up/down=fwd/back, left/right=strafe)

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
import threading
import time
from PS2 import PS2Controller, BTN_UP, BTN_DOWN, BTN_LEFT, BTN_RIGHT

# ── Config ──
MAX_RPM = 200          # max RPM per motor
MICRO_RPM = 50         # D-pad fine-adjust crawl speed (per wheel), for nudging
                       # the robot onto floor marks -- sticks are too coarse
DEADZONE = 15          # joystick center deadzone (around 128)
SEND_INTERVAL = 0.05   # 50ms = 20Hz
HEARTBEAT_INTERVAL = 0.1   # 100ms, well under STM32's 200ms timeout


def heartbeat_loop(bus, stop_event):
    """Send CAN 0x300 (DLC=0) every 100ms on its own thread.

    Runs independent of the PS2 read loop so PS2Controller.read()'s
    blocking bit-bang I/O (and the "no controller" 0.5s sleep) can never
    cause a heartbeat gap long enough to trip the STM32's 200ms timeout.
    """
    msg = can.Message(arbitration_id=0x300, data=b'', is_extended_id=False)
    while not stop_event.is_set():
        bus.send(msg)
        stop_event.wait(HEARTBEAT_INTERVAL)


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

    hb_stop = threading.Event()
    hb_thread = threading.Thread(target=heartbeat_loop, args=(bus, hb_stop), daemon=True)
    hb_thread.start()

    print("PS2 X-Drive Test — Ctrl+C to stop")
    print(f"Max RPM: ±{MAX_RPM}")
    print("Left stick: move  |  Right stick X: rotate")
    print(f"D-pad: fine-adjust ±{MICRO_RPM} RPM (up/down=fwd/back, left/right=strafe)")
    print(f"Heartbeat: 0x300 every {int(HEARTBEAT_INTERVAL*1000)}ms")

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

            # D-pad fine adjustment: adds a small fixed-RPM crawl on top of the
            # sticks so you can nudge the robot onto a mark. With the sticks
            # centered it gives pure slow motion. Same body-frame convention as
            # the sticks (+vx fwd, +vy left); held = continuous, tapped = pulse.
            micro = MICRO_RPM / MAX_RPM
            if ps2.is_pressed(data['btn1'], BTN_UP):
                vx += micro
            if ps2.is_pressed(data['btn1'], BTN_DOWN):
                vx -= micro
            if ps2.is_pressed(data['btn1'], BTN_LEFT):
                vy += micro
            if ps2.is_pressed(data['btn1'], BTN_RIGHT):
                vy -= micro

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
        hb_stop.set()
        hb_thread.join(timeout=1.0)
        bus.shutdown()
        ps2.cleanup()


if __name__ == '__main__':
    main()