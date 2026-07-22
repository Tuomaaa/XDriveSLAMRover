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


# ---- Heading tracking under sustained rotation ----
#
# The scenario above never rotates, which hides the filter's sharpest
# failure mode. Heading is the weakly-observed dimension, so resampling
# impoverishes it first: with too small a SIGMA_ROT_FLOOR the cloud commits
# to a heading within a few seconds and the error then grows without bound,
# because there are no surviving particles holding the alternative to
# select. Under motion this is self-reinforcing -- a wrong heading rotates
# the body-frame delta the wrong way, which drags position with it.
#
# CALIBRATED 2026-07-22 against this arc: final heading error is ~20deg at
# a 0.002 rot floor versus ~3deg at 0.010. The parked-cloud test in mcl.py
# cannot catch this -- a stationary rover's heading barely affects the
# predicted ranges, so nothing prunes the spread there and it looks healthy
# at either floor. It takes motion to expose it.

ARC_STEPS = 320             # 40s at 8Hz
ARC_RADIUS = 0.25
ARC_CENTRE = 0.5825
ARC_SWEEP = math.pi         # heading rotates through 180deg


def arc_true_pose(index):
    """Rover following a half circle, nose tangent to the path."""
    angle = ARC_SWEEP * index / ARC_STEPS
    return (ARC_CENTRE + ARC_RADIUS * math.cos(angle),
            ARC_CENTRE + ARC_RADIUS * math.sin(angle),
            angle)


def run_heading_tracking_test():
    rng = random.Random(7)
    rect_map = default_map()
    mcl = MCL(rect_map, default_beam_model(), SENSOR_CONFIGS,
              num_particles=500, alpha=(0.25, 0.05, 0.05, 0.15), rng_seed=3)
    mcl.initialize(*arc_true_pose(0), xy_spread=0.03, theta_spread=0.03)

    previous = arc_true_pose(0)
    for index in range(1, ARC_STEPS):
        current = arc_true_pose(index)

        dx_world = current[0] - previous[0]
        dy_world = current[1] - previous[1]
        cos_p, sin_p = math.cos(previous[2]), math.sin(previous[2])
        dx_body = dx_world * cos_p + dy_world * sin_p
        dy_body = -dx_world * sin_p + dy_world * cos_p

        mcl.predict(dx_body * ODOM_SCALE, dy_body * ODOM_SCALE,
                    current[2] - previous[2] + ODOM_YAW_BIAS)

        measurements = []
        for angle, offset in SENSOR_CONFIGS:
            expected = rect_map.expected_range(current, angle, offset)
            sigma = 0.0017 + 0.0078 * expected
            measurements.append(expected + rng.gauss(0.0, sigma))
        mcl.update(measurements)
        previous = current

    est_x, est_y, est_theta = mcl.estimate()
    heading_error = abs(math.degrees(math.atan2(
        math.sin(est_theta - previous[2]),
        math.cos(est_theta - previous[2]))))
    position_error = math.hypot(est_x - previous[0], est_y - previous[1])

    print(f'\narc true   x={previous[0]:.3f} y={previous[1]:.3f} '
          f'theta={math.degrees(previous[2]):+.1f}deg')
    print(f'arc mcl    x={est_x:.3f} y={est_y:.3f} '
          f'theta={math.degrees(est_theta):+.1f}deg')
    print(f'           heading error {heading_error:.1f}deg   '
          f'position error {position_error * 100:.1f}cm')

    assert heading_error < 10.0, (
        f'heading error {heading_error:.1f}deg -- the cloud has probably '
        'lost its heading diversity; check SIGMA_ROT_FLOOR')
    assert position_error < 0.03, f'position off by {position_error * 100:.1f}cm'
    print('\nheading tracking test passed')


if __name__ == '__main__':
    run_convergence_test()
    run_heading_tracking_test()
