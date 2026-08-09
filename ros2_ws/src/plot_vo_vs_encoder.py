"""Plot VO trajectory vs encoder odometry for qualitative comparison.

Monocular VO has no scale, so the VO path is normalized to match
the encoder path length before overlaying.

Camera frame -> robot frame mapping:
  camera z (depth/forward) -> robot x (forward)
  camera -x (left)         -> robot y (left)

Usage: python3 plot_vo_vs_encoder.py [vo_csv] [encoder_csv]
"""

import sys
import numpy as np
import matplotlib.pyplot as plt


def load_csv(path):
    data = np.genfromtxt(path, delimiter=",", skip_header=1)
    return data


def path_length(xy):
    diffs = np.diff(xy, axis=0)
    return np.sum(np.sqrt(diffs[:, 0]**2 + diffs[:, 1]**2))


def main():
    vo_path = sys.argv[1] if len(sys.argv) > 1 else "vo_trajectory.csv"
    enc_path = sys.argv[2] if len(sys.argv) > 2 else "encoder_trajectory.csv"

    vo = load_csv(vo_path)
    enc = load_csv(enc_path)

    # VO columns: timestamp, x, y, z (camera frame)
    # map to robot frame: forward = z, left = -x
    vo_robot_x = vo[:, 3]   # camera z -> robot forward
    vo_robot_y = -vo[:, 1]  # camera -x -> robot left

    # encoder columns: timestamp, x, y, theta
    enc_x = enc[:, 1]
    enc_y = enc[:, 2]

    # normalize VO scale to match encoder path length
    vo_xy = np.column_stack([vo_robot_x, vo_robot_y])
    enc_xy = np.column_stack([enc_x, enc_y])

    vo_len = path_length(vo_xy)
    enc_len = path_length(enc_xy)

    if vo_len > 1e-6:
        scale = enc_len / vo_len
        vo_robot_x *= scale
        vo_robot_y *= scale
        print(f"VO scale factor: {scale:.2f}")

    print(f"VO: {len(vo)} samples, encoder: {len(enc)} samples")
    print(f"Encoder path length: {enc_len:.3f} m")

    fig, ax = plt.subplots(1, 1, figsize=(8, 8))
    ax.plot(enc_x, enc_y, "b-", linewidth=2, label="Encoder odometry")
    ax.plot(vo_robot_x, vo_robot_y, "r-", linewidth=1.5, alpha=0.8,
            label="Visual odometry (scaled)")

    ax.plot(enc_x[0], enc_y[0], "go", markersize=10, label="Start")
    ax.plot(enc_x[-1], enc_y[-1], "rs", markersize=10, label="End (encoder)")

    ax.set_xlabel("x (m) - forward")
    ax.set_ylabel("y (m) - left")
    ax.set_title("VO vs Encoder Odometry")
    ax.legend()
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig("vo_vs_encoder.png", dpi=150)
    print("Saved vo_vs_encoder.png")
    plt.show()


if __name__ == "__main__":
    main()
