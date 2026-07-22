"""Hardware-free convergence test for the MCL particle filter.

Drives a simulated rover along an L inside the KT-board map. The odometry
handed to the filter is deliberately corrupted the way the real open-loop
chassis is -- a few percent short on translation plus a steady yaw bias --
while the range readings carry the Week 5 calibrated noise. The filter has
to beat dead reckoning.

The path is an open L rather than a closed loop on purpose: on a loop the
dead-reckoning error partly cancels when the return legs run back through
the same accumulated heading error, which flatters the baseline and makes
the comparison much less decisive.
"""

import math
import random

from mcl import MCL, SENSOR_CONFIGS, default_beam_model, default_map


# Corruption applied to the odometry the filter sees. The rover under-reads
# translation slightly and accumulates yaw; 0.002 rad/step over 120 steps is
# ~14 degrees, in line with the observed drift.
ODOM_SCALE = 0.97
ODOM_YAW_BIAS = 0.002

STEP_M = 0.005      # 5mm per cycle, ~8Hz -> a slow crawl
STEPS = 120
START_POSE = (0.30, 0.30, 0.0)


def simulate(steps, rng):
    """Yield (odom_delta, measurements, true_pose) along the L path.

    Ground truth advances exactly and never rotates, so body and world
    frames coincide for the true pose; all the heading error lives in the
    odometry the filter is given. Half the steps go forward (+x), half
    strafe left (+y), 300mm each, ending at (0.60, 0.60).
    """
    rect_map = default_map()
    true_x, true_y, true_theta = START_POSE
    leg_length = steps // 2
    legs = ((STEP_M, 0.0), (0.0, STEP_M))

    for index in range(steps):
        dx_body, dy_body = legs[min(index // leg_length, len(legs) - 1)]
        true_x += dx_body
        true_y += dy_body
        true_pose = (true_x, true_y, true_theta)

        measurements = []
        for angle, offset in SENSOR_CONFIGS:
            expected = rect_map.expected_range(true_pose, angle, offset)
            sigma = 0.0017 + 0.0078 * expected
            measurements.append(expected + rng.gauss(0.0, sigma))

        odom_delta = (dx_body * ODOM_SCALE, dy_body * ODOM_SCALE,
                      ODOM_YAW_BIAS)
        yield odom_delta, measurements, true_pose


def run_convergence_test():
    rng = random.Random(20260722)
    # Looser than the mcl.py default on purpose: the simulated odometry is
    # more corrupt than the real chassis, and the cloud needs enough spread
    # each step to follow the injected drift instead of locking onto a
    # stale position. Real tuning happens against a recorded run.
    mcl = MCL(default_map(), default_beam_model(), SENSOR_CONFIGS,
              num_particles=500, alpha=(0.25, 0.05, 0.05, 0.15), rng_seed=7)
    mcl.initialize(*START_POSE, xy_spread=0.03, theta_spread=0.03)

    dead_x, dead_y, dead_theta = START_POSE
    true_pose = START_POSE

    for odom_delta, measurements, true_pose in simulate(STEPS, rng):
        dx, dy, dtheta = odom_delta

        # Dead reckoning: the same deltas, integrated with no correction.
        dead_x += dx * math.cos(dead_theta) - dy * math.sin(dead_theta)
        dead_y += dx * math.sin(dead_theta) + dy * math.cos(dead_theta)
        dead_theta += dtheta

        mcl.predict(dx, dy, dtheta)
        mcl.update(measurements)

    est_x, est_y, est_theta = mcl.estimate()
    mcl_error = math.hypot(est_x - true_pose[0], est_y - true_pose[1])
    dead_error = math.hypot(dead_x - true_pose[0], dead_y - true_pose[1])

    print(f'true       x={true_pose[0]:.3f} y={true_pose[1]:.3f}')
    print(f'dead-reck  x={dead_x:.3f} y={dead_y:.3f}  '
          f'error={dead_error * 100:.1f}cm')
    print(f'mcl        x={est_x:.3f} y={est_y:.3f}  '
          f'error={mcl_error * 100:.1f}cm  '
          f'theta={math.degrees(est_theta):+.1f}deg')

    # A filter that silently stopped correcting would converge toward
    # dead_error, so a bare "better than dead reckoning" comparison is close
    # to a coin flip. Require a clear margin instead, which also pins the
    # alpha this test ships with -- the library default scores materially
    # worse on this same scenario.
    assert mcl_error < 0.04, f'MCL off by {mcl_error * 100:.1f}cm'
    assert mcl_error < dead_error / 2.0, (mcl_error, dead_error)
    print('\nMCL convergence test passed')


if __name__ == '__main__':
    run_convergence_test()
