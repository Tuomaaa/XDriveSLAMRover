"""Capture checkerboard images for camera calibration.

Usage: python3 calibration_capture.py [output_dir]

Controls:
  SPACE  - save current frame
  Q      - quit

Aim for 15-20 images from different angles, distances, and positions.
Cover all regions of the frame (corners and edges matter most for distortion).
"""

import os
import sys
import cv2
from camera_utils import open_camera, read_frame

CHECKERBOARD = (9, 6)  # inner corners (cols, rows)

def main():
    out_dir = sys.argv[1] if len(sys.argv) > 1 else "calibration_images"
    os.makedirs(out_dir, exist_ok=True)

    cap = open_camera()
    count = 0
    print(f"Saving to {out_dir}/. Press SPACE to capture, Q to quit.")

    while True:
        frame = read_frame(cap)
        if frame is None:
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

        display = frame.copy()
        if found:
            cv2.drawChessboardCorners(display, CHECKERBOARD, corners, found)

        status = f"[{count} saved] {'PATTERN FOUND' if found else 'no pattern'}"
        cv2.putText(display, status, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0) if found else (0, 0, 255), 2)
        cv2.imshow("Calibration Capture", display)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        if key == ord(" ") and found:
            path = os.path.join(out_dir, f"calib_{count:03d}.png")
            cv2.imwrite(path, frame)
            count += 1
            print(f"Saved {path}")

    cap.release()
    cv2.destroyAllWindows()
    print(f"Done. {count} images saved to {out_dir}/")


if __name__ == "__main__":
    main()
