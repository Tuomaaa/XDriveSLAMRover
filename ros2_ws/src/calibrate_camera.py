"""Run camera calibration from captured checkerboard images.

Usage: python3 calibrate_camera.py [image_dir] [output_file]

Outputs camera matrix and distortion coefficients to a YAML file
that OpenCV can load directly with cv2.FileStorage.
"""

import os
import sys
import glob
import numpy as np
import cv2

CHECKERBOARD = (9, 6)  # inner corners (cols, rows)
SQUARE_SIZE_MM = 30.0  # measure with a ruler after printing


def main():
    image_dir = sys.argv[1] if len(sys.argv) > 1 else "calibration_images"
    output_file = sys.argv[2] if len(sys.argv) > 2 else "calibration.yaml"

    images = sorted(glob.glob(os.path.join(image_dir, "*.png")))
    if not images:
        print(f"No .png files in {image_dir}/")
        return

    objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
    objp *= SQUARE_SIZE_MM

    obj_points = []
    img_points = []
    img_size = None

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    for path in images:
        img = cv2.imread(path)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if img_size is None:
            img_size = gray.shape[::-1]

        found, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)
        if found:
            corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            obj_points.append(objp)
            img_points.append(corners_refined)
            print(f"  OK: {os.path.basename(path)}")
        else:
            print(f"SKIP: {os.path.basename(path)} (pattern not found)")

    if len(obj_points) < 5:
        print(f"Only {len(obj_points)} valid images. Need at least 5.")
        return

    print(f"\nCalibrating with {len(obj_points)} images...")
    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        obj_points, img_points, img_size, None, None
    )

    print(f"\nRMS reprojection error: {ret:.4f}")
    print(f"Camera matrix:\n{camera_matrix}")
    print(f"Distortion coefficients: {dist_coeffs.ravel()}")

    fs = cv2.FileStorage(output_file, cv2.FILE_STORAGE_WRITE)
    fs.write("camera_matrix", camera_matrix)
    fs.write("dist_coeffs", dist_coeffs)
    fs.write("image_width", img_size[0])
    fs.write("image_height", img_size[1])
    fs.write("rms_error", ret)
    fs.write("num_images", len(obj_points))
    fs.write("square_size_mm", SQUARE_SIZE_MM)
    fs.release()

    print(f"\nSaved to {output_file}")

    # per-image reprojection error
    print("\nPer-image reprojection error:")
    for i, path in enumerate(images):
        if i < len(rvecs):
            reproj, _ = cv2.projectPoints(obj_points[i], rvecs[i], tvecs[i],
                                          camera_matrix, dist_coeffs)
            err = cv2.norm(img_points[i], reproj, cv2.NORM_L2) / len(reproj)
            print(f"  {os.path.basename(path)}: {err:.4f}")


if __name__ == "__main__":
    main()
