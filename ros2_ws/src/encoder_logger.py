"""Log encoder odometry to CSV for trajectory comparison.

Standalone script (no ROS2). Reads CAN encoder frames directly
and runs OdometryEstimator. Press Ctrl+C to stop.

Usage: sudo python3 encoder_logger.py [output_file]

Run in a separate terminal alongside visual_odometry.py while
driving the robot along a test path.
"""

import csv
import sys
import time

from can_interface import CanInterface
from protocol import EncoderMsg, decode
from odometry import OdometryEstimator


def main():
    output_file = sys.argv[1] if len(sys.argv) > 1 else "encoder_trajectory.csv"

    iface = CanInterface()
    odo = OdometryEstimator()

    ticks = {}
    t0 = time.monotonic()

    log_file = open(output_file, "w", newline="")
    log_writer = csv.writer(log_file)
    log_writer.writerow(["timestamp", "x", "y", "theta"])

    sample_count = 0
    print(f"Encoder logger -> {output_file}")
    print("Waiting for CAN frames... Ctrl+C to stop.")

    try:
        while True:
            msg = iface.recv(timeout=1.0)
            if msg is None:
                continue

            result = decode(msg.arbitration_id, msg.data)
            if not isinstance(result, EncoderMsg):
                continue

            base = result.motor_id * 2
            ticks[base] = result.motor_a_ticks
            ticks[base + 1] = result.motor_b_ticks

            if result.motor_id == 1 and len(ticks) == 4:
                odom = odo.update(dict(ticks), msg.timestamp)
                if odom is None:
                    continue

                x, y, theta, vx, vy, omega = odom
                elapsed = time.monotonic() - t0
                log_writer.writerow([f"{elapsed:.3f}", f"{x:.4f}",
                                     f"{y:.4f}", f"{theta:.4f}"])

                sample_count += 1
                if sample_count % 25 == 0:
                    print(f"  x={x:+.3f} y={y:+.3f} theta={theta:+.2f}  "
                          f"[{sample_count} samples]", end="\r")

    except KeyboardInterrupt:
        print(f"\nSaved {output_file} ({sample_count} samples)")
    finally:
        log_file.close()
        iface.shutdown()


if __name__ == "__main__":
    main()
