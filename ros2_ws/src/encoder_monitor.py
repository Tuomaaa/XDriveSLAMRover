"""
encoder_monitor.py
Real-time encoder tick monitor for ENCODER_SIGN calibration.

Run this alongside PS2_Drive_Test.py (in a second terminal — SocketCAN
allows multiple readers on the same interface) while driving pure vy or
pure omega motion, and watch which wheels count up vs. down.

Expected sign pattern per the inverse-kinematics convention in
odometry.py / ps2_drive_test.py (fl = vx+vy+wR, fr = vx-vy-wR,
rl = vx-vy+wR, rr = vx+vy-wR):
    pure +vy (strafe):  fl=+  fr=-  rl=-  rr=+
    pure +omega (spin): fl=+  fr=-  rl=+  rr=-

If a wheel's printed delta doesn't match its expected sign for the
motion you drove, flip that motor's entry in odometry.py's
ENCODER_SIGN dict to -1. Don't guess — only change signs you've
actually observed disagree with the pattern above.

Usage: sudo python3 encoder_monitor.py
"""

import time

from can_interface import CanInterface
from protocol import EncoderMsg, decode

# Motor index -> physical wheel position (matches odometry.py's MOTOR_MAP).
# ENCODER_0 (0x200) carries motors 0,1 ; ENCODER_1 (0x201) carries motors 2,3.
WHEEL_NAMES = {0: "FL", 1: "FR", 2: "RL", 3: "RR"}


def main():
    iface = CanInterface()
    print("Encoder Monitor — Ctrl+C to stop")
    print("Waiting for first encoder frame to zero the baseline...")

    baseline = {}
    latest = {}

    try:
        while True:
            msg = iface.recv(timeout=1.0)
            if msg is None:
                print("No CAN frames received        ", end='\r')
                continue

            result = decode(msg.arbitration_id, msg.data)
            if not isinstance(result, EncoderMsg):
                continue

            if result.motor_id == 0:
                latest[0] = result.motor_a_ticks
                latest[1] = result.motor_b_ticks
            else:
                latest[2] = result.motor_a_ticks
                latest[3] = result.motor_b_ticks

            if len(baseline) < 4 and len(latest) == 4:
                baseline = dict(latest)

            if len(baseline) < 4:
                continue

            deltas = {i: latest[i] - baseline[i] for i in range(4)}
            print(
                "  ".join(
                    f"{WHEEL_NAMES[i]}:{deltas[i]:+6d}" for i in range(4)
                ),
                end='\r'
            )

    except KeyboardInterrupt:
        print("\nStopped")
    finally:
        iface.shutdown()


if __name__ == '__main__':
    main()
