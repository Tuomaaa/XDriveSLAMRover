"""Motor channel diagnostic — isolate RR (M3) failure mode.

Sends hardcoded CAN velocity commands in sequence to pinpoint
whether M3 fails alone or only when other motors run simultaneously.

Usage: sudo python3 motor_diag.py
"""

import can
import struct
import threading
import time

SEND_HZ = 20
HEARTBEAT_INTERVAL = 0.1
RPM = 150


def heartbeat_loop(bus, stop_event):
    msg = can.Message(arbitration_id=0x300, data=b'', is_extended_id=False)
    while not stop_event.is_set():
        bus.send(msg)
        stop_event.wait(HEARTBEAT_INTERVAL)


def send_rpm(bus, m0, m1, m2, m3, duration, label):
    print(f"\n>>> {label}")
    print(f"    M0={m0:+4d}  M1={m1:+4d}  M2={m2:+4d}  M3={m3:+4d}")
    print(f"    Watch RR motor for {duration}s...")

    payload = struct.pack('<4h', m0, m1, m2, m3)
    msg = can.Message(arbitration_id=0x100, data=payload, is_extended_id=False)

    end = time.time() + duration
    while time.time() < end:
        bus.send(msg)
        time.sleep(1.0 / SEND_HZ)


def stop_all(bus):
    payload = struct.pack('<4h', 0, 0, 0, 0)
    bus.send(can.Message(arbitration_id=0x100, data=payload, is_extended_id=False))


def main():
    bus = can.Bus(channel='can0', interface='socketcan', bitrate=500000)
    hb_stop = threading.Event()
    hb_thread = threading.Thread(target=heartbeat_loop, args=(bus, hb_stop), daemon=True)
    hb_thread.start()

    R = RPM

    tests = [
        # (m0, m1, m2, m3, duration, label)
        (0,   R,   0,   0,   3, "TEST 1: M1(FR) alone forward"),
        (0,  -R,   0,   0,   3, "TEST 2: M1(FR) alone backward"),
        (R,   R,   R,   R,   3, "TEST 3: all forward"),
        (R,  -R,  -R,   R,   3, "TEST 4: strafe L (M1 rev)"),
        (-R,  R,   R,  -R,   3, "TEST 5: strafe R (M1 fwd)"),
        (R,  -R,   R,  -R,   3, "TEST 6: rotation CW (M1 rev)"),
        (R,   R,   0,   0,   3, "TEST 7: M0+M1 same dir (same TB6612 chip)"),
        (R,  -R,   0,   0,   3, "TEST 8: M0 fwd + M1 rev (same chip, opposite)"),
    ]

    print("=" * 50)
    print("Motor diagnostic — watch FR (M1) motor")
    print(f"RPM = {R}")
    print("=" * 50)

    try:
        for m0, m1, m2, m3, dur, label in tests:
            send_rpm(bus, m0, m1, m2, m3, dur, label)
            stop_all(bus)
            input("    Press ENTER for next test (or Ctrl+C to stop)...")

    except KeyboardInterrupt:
        pass
    finally:
        stop_all(bus)
        hb_stop.set()
        hb_thread.join(timeout=1.0)
        bus.shutdown()
        print("\nDone — all motors stopped.")


if __name__ == '__main__':
    main()
