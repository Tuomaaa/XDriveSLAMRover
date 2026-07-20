"""Hardware-free integration smoke test for map and measurement models."""

import math
import random

from map_model import RectMap
from measurement_model import BeamModel


RIGHT_SENSOR = (-math.pi / 2, (0.0, -0.09))
BACK_SENSOR = (math.pi, (-0.09, 0.0))
SENSOR_CONFIGS = (RIGHT_SENSOR, BACK_SENSOR)


def score_pose(model, rect_map, measurements, pose):
    return model.total_log_likelihood(
        measurements, pose, rect_map, SENSOR_CONFIGS)


def expected_ranges(rect_map, pose):
    return [
        rect_map.expected_range(pose, angle, offset)
        for angle, offset in SENSOR_CONFIGS
    ]


def wrong_poses(true_pose, rect_map):
    x, y, theta = true_pose
    wrong_x = x + 0.25 if x + 0.25 < rect_map.x_max else x - 0.25
    wrong_y = y + 0.20 if y + 0.20 < rect_map.y_max else y - 0.20
    return (
        (wrong_x, y, theta),
        (x, wrong_y, theta),
        (x, y, theta + math.pi / 2),
    )


def run_smoke_test():
    rng = random.Random(20260715)
    rect_map = RectMap(0.0, 1.5, 0.0, 1.0)
    # CALIBRATED 2026-07-20 from HC-SR04 bench data (100/300/600/1000mm,
    # 100 samples each, both sensors, KT board target).
    # sigma_hit fitted: sigma(d) = 0.0017 + 0.0078*d (combined right+rear).
    # w_max from 2.9% overall invalid rate; w_short kept small (no observed
    # short-reading anomalies); w_rand as uniform floor.
    model = BeamModel(
        z_max=4.0,
        sigma_hit=(0.0017, 0.0078),
        lambda_short=0.5,
        w_hit=0.94,
        w_short=0.01,
        w_max=0.03,
        w_rand=0.02,
    )
    true_poses = (
        (0.50, 0.30, 0.0),
        (1.00, 0.70, math.pi / 2),
        (0.80, 0.50, -math.pi / 2),
    )

    for true_pose in true_poses:
        expected = expected_ranges(rect_map, true_pose)
        measurements = [
            max(0.0, value + rng.gauss(0.0, 0.01))
            for value in expected
        ]
        true_score = score_pose(model, rect_map, measurements, true_pose)
        print(
            f'\nTrue pose {true_pose}: '
            f'z_right={measurements[0]:.3f}m '
            f'z_back={measurements[1]:.3f}m '
            f'log_lik={true_score:.3f}'
        )

        for wrong_pose in wrong_poses(true_pose, rect_map):
            wrong_expected = expected_ranges(rect_map, wrong_pose)
            wrong_score = score_pose(
                model, rect_map, measurements, wrong_pose)
            print(
                f'Wrong pose {wrong_pose}: '
                f'expected=({wrong_expected[0]:.3f}, '
                f'{wrong_expected[1]:.3f})m '
                f'log_lik={wrong_score:.3f}'
            )
            assert true_score >= wrong_score

    print('\nmeasurement_model integration smoke test passed')


if __name__ == '__main__':
    run_smoke_test()