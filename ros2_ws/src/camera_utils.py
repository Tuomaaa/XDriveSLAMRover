"""Shared camera helpers. All scripts import from here."""

import cv2

CAMERA_DEVICE = "/dev/video1"
FLIP_CODE = -1  # -1 = rotate 180 (camera mounted upside-down)


def open_camera(width=640, height=480):
    cap = cv2.VideoCapture(CAMERA_DEVICE)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open {CAMERA_DEVICE}")
    return cap


def read_frame(cap):
    ret, frame = cap.read()
    if not ret:
        return None
    return cv2.flip(frame, FLIP_CODE)
