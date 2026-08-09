"""Plot VO vs encoder from the merged vo_encoder_data.csv.

Both sensors share the same clock, so trajectories are temporally aligned.
VO is scaled to match encoder path length (monocular has no absolute scale).

Camera frame -> robot frame mapping:
  camera z (depth/forward) -> robot x (forward)
  camera -x (left)         -> robot y (left)

Usage: python3 plot_vo_vs_encoder.py [csv_file]
"""

import sys
import numpy as np
import matplotlib.pyplot as plt


def path_length(xy):
    diffs = np.diff(xy, axis=0)
    return np.sum(np.sqrt(diffs[:, 0]**2 + diffs[:, 1]**2))


def main():
    csv_path = sys.argv[1] if len(sys.argv) > 1 else "vo_encoder_data.csv"
    data = np.genfromtxt(csv_path, delimiter=",", skip_header=1)

    ts = data[:, 0]
    # VO columns: vo_x, vo_y, vo_z (camera frame)
    vo_robot_x = data[:, 3]    # camera z -> robot forward
    vo_robot_y = -data[:, 1]   # camera -x -> robot left
    # Encoder columns: enc_x, enc_y, enc_theta
    enc_x = data[:, 4]
    enc_y = data[:, 5]

    # trim leading zeros: find first frame where either sensor moves
    enc_moved = np.abs(enc_x) + np.abs(enc_y) > 0.005
    vo_moved = np.abs(vo_robot_x) + np.abs(vo_robot_y) > 0.005
    either = enc_moved | vo_moved
    if np.any(either):
        start = np.argmax(either)
        start = max(0, start - 5)
    else:
        start = 0

    ts = ts[start:]
    vo_robot_x = vo_robot_x[start:]
    vo_robot_y = vo_robot_y[start:]
    enc_x = enc_x[start:]
    enc_y = enc_y[start:]

    # normalize VO scale to match encoder path length
    vo_xy = np.column_stack([vo_robot_x, vo_robot_y])
    enc_xy = np.column_stack([enc_x, enc_y])

    vo_len = path_length(vo_xy)
    enc_len = path_length(enc_xy)

    if vo_len > 1e-6:
        scale = enc_len / vo_len
        vo_robot_x *= scale
        vo_robot_y *= scale
        print(f"VO scale factor: {scale:.4f}")

    print(f"Samples: {len(ts)} (trimmed from index {start})")
    print(f"Encoder path length: {enc_len:.3f} m")
    print(f"Duration: {ts[-1] - ts[0]:.1f} s")

    fig, axes = plt.subplots(1, 2, figsize=(14, 6))

    # left: XY trajectory overlay
    ax = axes[0]
    ax.plot(enc_x, enc_y, "b-", linewidth=2, label="Encoder")
    ax.plot(vo_robot_x, vo_robot_y, "r-", linewidth=1.5, alpha=0.8,
            label="VO (scaled)")
    ax.plot(enc_x[0], enc_y[0], "go", markersize=10, label="Start")
    ax.plot(enc_x[-1], enc_y[-1], "rs", markersize=10, label="End")
    ax.set_xlabel("x (m) - forward")
    ax.set_ylabel("y (m) - left")
    ax.set_title("Trajectory")
    ax.legend()
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)

    # right: x and y vs time
    ax2 = axes[1]
    t_rel = ts - ts[0]
    ax2.plot(t_rel, enc_x, "b-", linewidth=1.5, label="Enc x")
    ax2.plot(t_rel, enc_y, "b--", linewidth=1.5, label="Enc y")
    ax2.plot(t_rel, vo_robot_x, "r-", linewidth=1, alpha=0.8, label="VO x")
    ax2.plot(t_rel, vo_robot_y, "r--", linewidth=1, alpha=0.8, label="VO y")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Position (m)")
    ax2.set_title("X/Y vs Time")
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig("vo_vs_encoder.png", dpi=150)
    print("Saved vo_vs_encoder.png")
    plt.show()


if __name__ == "__main__":
    main()
