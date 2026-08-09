"""Monocular visual odometry: ORB features + epipolar geometry.

Pipeline (corresponds to ch7 sections 7.1-7.4):
  1. ORB feature detection + descriptor extraction
  2. Brute-force matching with cross-check
  3. cv2.findEssentialMat (RANSAC)
  4. cv2.recoverPose -> R, t (t is unit vector, no scale)

Displays live camera feed with keypoints and a top-down trajectory.
Press Q to quit.
"""

import numpy as np
import cv2
from camera_utils import open_camera, read_frame

MIN_MATCH = 15
MIN_INLIER = 10


def load_calibration(path="calibration.yaml"):
    fs = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
    K = fs.getNode("camera_matrix").mat()
    dist = fs.getNode("dist_coeffs").mat()
    fs.release()
    return K, dist


def main():
    K, dist = load_calibration()

    orb = cv2.ORB_create(nfeatures=1000)
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

    cap = open_camera()

    prev_kp = None
    prev_des = None

    R_w = np.eye(3)
    t_w = np.zeros((3, 1))

    traj = np.zeros((600, 600, 3), dtype=np.uint8)
    scale = 10

    frame_count = 0

    while True:
        frame = read_frame(cap)
        if frame is None:
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        kp, des = orb.detectAndCompute(gray, None)

        info = "init"

        if prev_des is not None and des is not None and len(kp) >= MIN_MATCH:
            matches = bf.match(prev_des, des)
            matches = sorted(matches, key=lambda m: m.distance)
            good = matches[:min(150, len(matches))]

            if len(good) >= MIN_MATCH:
                pts1 = np.float32([prev_kp[m.queryIdx].pt for m in good])
                pts2 = np.float32([kp[m.trainIdx].pt for m in good])

                # undistort matched points (re-project with K so output stays in pixel coords)
                pts1_u = cv2.undistortPoints(pts1.reshape(-1, 1, 2), K, dist, P=K).reshape(-1, 2)
                pts2_u = cv2.undistortPoints(pts2.reshape(-1, 1, 2), K, dist, P=K).reshape(-1, 2)

                E, mask_e = cv2.findEssentialMat(
                    pts1_u, pts2_u, K, method=cv2.RANSAC, prob=0.999, threshold=1.0
                )

                if E is not None:
                    inliers, R, t, _ = cv2.recoverPose(E, pts1_u, pts2_u, K)

                    if inliers > MIN_INLIER:
                        t_w = t_w + R_w @ t
                        R_w = R @ R_w

                    info = f"inliers: {inliers}/{len(good)}"
                else:
                    info = "E failed"
            else:
                info = f"matches: {len(good)}"

        # draw trajectory (X-Z top-down view)
        draw_x = int(t_w[0, 0] * scale) + 300
        draw_z = int(t_w[2, 0] * scale) + 300
        if 0 <= draw_x < 600 and 0 <= draw_z < 600:
            cv2.circle(traj, (draw_x, draw_z), 1, (0, 255, 0), 1)

        # draw info and keypoints
        frame_count += 1
        cv2.putText(frame, info, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(frame, f"frame: {frame_count}", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        display = cv2.drawKeypoints(frame, kp, None, color=(0, 255, 0),
                                    flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        cv2.imshow("VO", display)
        cv2.imshow("Trajectory", traj)

        prev_kp = kp
        prev_des = des

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()
    print(f"Final position: x={t_w[0,0]:.2f} y={t_w[1,0]:.2f} z={t_w[2,0]:.2f}")


if __name__ == "__main__":
    main()
