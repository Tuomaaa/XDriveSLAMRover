"""Simultaneous VO + encoder capture with shared clock.

Runs visual odometry and encoder odometry in one process so timestamps
are directly comparable. Encoder runs on a background thread; VO runs
on the main thread.

Output: vo_encoder_data.csv with columns:
  timestamp, vo_x, vo_y, vo_z, enc_x, enc_y, enc_theta

Usage: sudo python3 vo_encoder_capture.py [--headless]
  --headless: no GUI windows, prints live stats, Ctrl+C to stop
  default: shows camera + trajectory windows, press Q to quit
"""

import csv
import math
import signal
import sys
import threading
import time

import cv2
import numpy as np

from camera_utils import open_camera, read_frame
from can_interface import CanInterface
from protocol import EncoderMsg, decode
from odometry import OdometryEstimator

MIN_MATCH = 15
MIN_INLIER = 10
MIN_INLIER_RATIO = 0.3


def load_calibration(path="calibration.yaml"):
    fs = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
    K = fs.getNode("camera_matrix").mat()
    dist = fs.getNode("dist_coeffs").mat()
    fs.release()
    return K, dist


class EncoderThread:
    def __init__(self, t0):
        self.t0 = t0
        self.lock = threading.Lock()
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self._stop = threading.Event()
        self._rows = []
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=2.0)

    def get_pose(self):
        with self.lock:
            return self.x, self.y, self.theta

    def get_rows(self):
        with self.lock:
            return list(self._rows)

    def _run(self):
        iface = CanInterface()
        odo = OdometryEstimator()
        ticks = {}

        try:
            while not self._stop.is_set():
                msg = iface.recv(timeout=0.1)
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
                    elapsed = time.monotonic() - self.t0
                    with self.lock:
                        self.x = x
                        self.y = y
                        self.theta = theta
                        self._rows.append((elapsed, x, y, theta))
        finally:
            iface.shutdown()


def main():
    headless = "--headless" in sys.argv

    K, dist = load_calibration()

    orb = cv2.ORB_create(nfeatures=1500)
    bf = cv2.BFMatcher(cv2.NORM_HAMMING)

    cap = open_camera()

    t0 = time.monotonic()

    enc = EncoderThread(t0)
    enc.start()

    prev_gray = None
    prev_kp = None
    prev_des = None

    R_w = np.eye(3)
    t_w = np.zeros((3, 1))

    if not headless:
        traj = np.zeros((600, 600, 3), dtype=np.uint8)
        scale = 50

    frame_count = 0
    update_count = 0
    skip_count = 0

    vo_rows = []
    running = True

    def on_sigint(sig, frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, on_sigint)

    mode = "headless (Ctrl+C to stop)" if headless else "GUI (Q to quit)"
    print(f"VO + Encoder capture — {mode}")
    print(f"ORB features: 1500, ratio test: 0.75, min inlier ratio: {MIN_INLIER_RATIO}")

    while running:
        frame = read_frame(cap)
        if frame is None:
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        kp, des = orb.detectAndCompute(gray, None)

        info = "init"

        if prev_des is not None and des is not None and len(kp) >= MIN_MATCH:
            raw_matches = bf.knnMatch(prev_des, des, k=2)

            good = []
            for pair in raw_matches:
                if len(pair) == 2:
                    m, n = pair
                    if m.distance < 0.75 * n.distance:
                        good.append(m)

            if len(good) >= MIN_MATCH:
                pts1 = np.float32([prev_kp[m.queryIdx].pt for m in good])
                pts2 = np.float32([kp[m.trainIdx].pt for m in good])

                pts1_u = cv2.undistortPoints(
                    pts1.reshape(-1, 1, 2), K, dist, P=K
                ).reshape(-1, 2)
                pts2_u = cv2.undistortPoints(
                    pts2.reshape(-1, 1, 2), K, dist, P=K
                ).reshape(-1, 2)

                E, mask_e = cv2.findEssentialMat(
                    pts1_u, pts2_u, K,
                    method=cv2.RANSAC, prob=0.999, threshold=1.0
                )

                if E is not None:
                    inliers, R, t, _ = cv2.recoverPose(
                        E, pts1_u, pts2_u, K
                    )

                    ratio = inliers / len(good) if len(good) > 0 else 0
                    if inliers > MIN_INLIER and ratio > MIN_INLIER_RATIO:
                        t_world = R_w.T @ t
                        t_w = t_w + t_world
                        R_w = R @ R_w
                        update_count += 1
                    else:
                        skip_count += 1

                    info = f"in:{inliers}/{len(good)} upd:{update_count}"
                else:
                    info = "E failed"
            else:
                info = f"match:{len(good)}"

        elapsed = time.monotonic() - t0
        enc_x, enc_y, enc_theta = enc.get_pose()
        vo_rows.append((
            elapsed,
            t_w[0, 0], t_w[1, 0], t_w[2, 0],
            enc_x, enc_y, enc_theta
        ))

        frame_count += 1

        if headless:
            if frame_count % 30 == 0:
                print(f"  f:{frame_count} upd:{update_count} skip:{skip_count} "
                      f"enc:({enc_x:+.3f},{enc_y:+.3f}) "
                      f"vo_z:{t_w[2,0]:+.3f}  {info}", end="\r")
        else:
            draw_x = int(t_w[2, 0] * scale) + 300
            draw_y = int(-t_w[0, 0] * scale) + 300
            if 0 <= draw_x < 600 and 0 <= draw_y < 600:
                cv2.circle(traj, (draw_x, draw_y), 1, (0, 255, 0), 1)

            ed_x = int(enc_x * scale) + 300
            ed_y = int(-enc_y * scale) + 300
            if 0 <= ed_x < 600 and 0 <= ed_y < 600:
                cv2.circle(traj, (ed_x, ed_y), 1, (255, 100, 0), 1)

            cv2.putText(frame, info, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(frame, f"f:{frame_count} skip:{skip_count}", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            display = cv2.drawKeypoints(
                frame, kp, None, color=(0, 255, 0),
                flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS
            )

            cv2.imshow("VO", display)
            cv2.imshow("Trajectory", traj)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

        prev_gray = gray
        prev_kp = kp
        prev_des = des

    enc.stop()
    cap.release()
    if not headless:
        cv2.destroyAllWindows()

    output = "vo_encoder_data.csv"
    with open(output, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow([
            "timestamp",
            "vo_x", "vo_y", "vo_z",
            "enc_x", "enc_y", "enc_theta"
        ])
        for row in vo_rows:
            w.writerow([
                f"{row[0]:.3f}",
                f"{row[1]:.4f}", f"{row[2]:.4f}", f"{row[3]:.4f}",
                f"{row[4]:.4f}", f"{row[5]:.4f}", f"{row[6]:.4f}"
            ])

    print(f"\nSaved {output} ({frame_count} frames, {update_count} VO updates, "
          f"{skip_count} skipped)")
    print(f"VO final: x={t_w[0,0]:.3f} y={t_w[1,0]:.3f} z={t_w[2,0]:.3f}")


if __name__ == "__main__":
    main()
