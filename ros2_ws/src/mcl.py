"""
mcl.py
Monte Carlo Localization (Probabilistic Robotics Ch.8) for the X-drive rover.

Consumes body-frame odometry deltas for the motion update and ultrasonic
ranges (metres) for the measurement update. Knows nothing about CAN, ROS,
or files -- feed it plain numbers. Wire it up in mcl_offline.py (CSV
replay) or, later, an online node.

Design notes:
  - Standard MCL: predict -> weight -> resample on every cycle, using
    low-variance resampling (Probabilistic Robotics Table 4.4).
  - The textbook odometry motion model splits a step into rot1/trans/rot2,
    which assumes a differential drive that must turn in order to
    translate. This chassis is holonomic, so noise is applied directly to
    the body-frame (dx, dy, dtheta) instead. The alpha parameters keep
    their meaning: uncertainty grows with distance travelled and angle
    turned.
  - The measurement update delegates to BeamModel/RectMap, which already
    carry the Week 5 bench calibration.
"""

import math

import numpy as np

from map_model import RectMap
from measurement_model import BeamModel


# ---- Week 5 bench calibration (see PROJECT_STATE.md) ----
# sigma(d) = 0.0017 + 0.0078*d metres, least-squares fit over 8 groups of
# 100 HC-SR04 samples at 100/300/600/1000mm against a KT board.
SIGMA_HIT = (0.0017, 0.0078)
BEAM_WEIGHTS = {'w_hit': 0.94, 'w_short': 0.01, 'w_max': 0.03, 'w_rand': 0.02}
Z_MAX = 4.0
LAMBDA_SHORT = 0.5

# Sensor mounting: (bearing on the robot, (x, y) offset from base_link),
# REP-103 body frame (+x forward, +y left).
RIGHT_SENSOR = (-math.pi / 2, (0.0, -0.09))
BACK_SENSOR = (math.pi, (-0.09, 0.0))
SENSOR_CONFIGS = (RIGHT_SENSOR, BACK_SENSOR)

# KT-board enclosure, measured 2026-07-22: 1165mm square.
MAP_BOUNDS = (0.0, 1.165, 0.0, 1.165)


def default_map():
    """RectMap for the measured KT-board enclosure."""
    return RectMap(*MAP_BOUNDS)


def default_beam_model():
    """BeamModel carrying the Week 5 calibration."""
    return BeamModel(
        z_max=Z_MAX,
        sigma_hit=SIGMA_HIT,
        lambda_short=LAMBDA_SHORT,
        **BEAM_WEIGHTS,
    )


def _wrap_angle(angle):
    """Wrap to (-pi, pi]. Works on scalars and numpy arrays alike."""
    return np.arctan2(np.sin(angle), np.cos(angle))


class MCL:
    """Particle filter over 2D poses (x, y, theta) in a rectangular map."""

    def __init__(self, rect_map, beam_model, sensor_configs,
                 num_particles=500, alpha=(0.05, 0.01, 0.01, 0.05),
                 rng_seed=None):
        """alpha is (a_trans_trans, a_trans_rot, a_rot_trans, a_rot_rot):
        how much translation and rotation noise each unit of travelled
        distance and turned angle contributes."""
        if int(num_particles) < 1:
            raise ValueError('num_particles must be positive')
        if len(alpha) != 4:
            raise ValueError('alpha must have four entries')
        if any(value < 0.0 for value in alpha):
            raise ValueError('alpha entries must be non-negative')

        self.rect_map = rect_map
        self.beam_model = beam_model
        self.sensor_configs = tuple(sensor_configs)
        self.num_particles = int(num_particles)
        self.alpha = tuple(float(value) for value in alpha)
        self._rng = np.random.default_rng(rng_seed)

        self.particles = np.zeros((self.num_particles, 3))
        self.weights = np.full(self.num_particles, 1.0 / self.num_particles)

    def initialize(self, x, y, theta, xy_spread=0.05, theta_spread=0.1):
        """Scatter particles as a Gaussian cloud around a known pose."""
        count = self.num_particles
        self.particles[:, 0] = self._rng.normal(x, xy_spread, count)
        self.particles[:, 1] = self._rng.normal(y, xy_spread, count)
        self.particles[:, 2] = _wrap_angle(
            self._rng.normal(theta, theta_spread, count))
        self._clamp_to_map()
        self.weights = np.full(count, 1.0 / count)

    def estimate(self):
        """Weighted mean pose; theta uses a circular mean so that headings
        either side of +/-pi do not average to zero."""
        x = float(np.sum(self.weights * self.particles[:, 0]))
        y = float(np.sum(self.weights * self.particles[:, 1]))
        sin_sum = float(np.sum(self.weights * np.sin(self.particles[:, 2])))
        cos_sum = float(np.sum(self.weights * np.cos(self.particles[:, 2])))
        return x, y, math.atan2(sin_sum, cos_sum)

    def _clamp_to_map(self):
        """Keep every particle inside the walls.

        A particle outside the rectangle has no meaningful expected range
        (most rays cast from outside miss entirely and come back as
        infinity), so instead of dying it would linger on the w_rand floor.
        Clamping is cheap and keeps every particle scoreable. The cost is a
        slight pile-up against the walls, which only shows up when the
        filter is already badly lost.
        """
        np.clip(self.particles[:, 0], self.rect_map.x_min,
                self.rect_map.x_max, out=self.particles[:, 0])
        np.clip(self.particles[:, 1], self.rect_map.y_min,
                self.rect_map.y_max, out=self.particles[:, 1])


def _test_defaults_match_calibration():
    model = default_beam_model()
    assert model.z_max == 4.0
    assert model.sigma_hit == (0.0017, 0.0078)
    assert math.isclose(
        model.w_hit + model.w_short + model.w_max + model.w_rand, 1.0)

    rect_map = default_map()
    assert math.isclose(rect_map.x_max - rect_map.x_min, 1.165)
    assert math.isclose(rect_map.y_max - rect_map.y_min, 1.165)

    # At dead centre both sensors face a wall 1.165/2 away, minus the 90mm
    # mount offset: 0.5825 - 0.09 = 0.4925 m. This is the number the Phase 6
    # bench check confirmed to within 1.3mm.
    centre = (0.5825, 0.5825, 0.0)
    for angle, offset in SENSOR_CONFIGS:
        assert math.isclose(
            rect_map.expected_range(centre, angle, offset), 0.4925)


def _test_initialize_and_estimate():
    mcl = MCL(default_map(), default_beam_model(), SENSOR_CONFIGS,
              num_particles=2000, rng_seed=1)
    mcl.initialize(0.5825, 0.5825, 0.0, xy_spread=0.05, theta_spread=0.1)

    assert mcl.particles.shape == (2000, 3), mcl.particles.shape
    assert math.isclose(float(np.sum(mcl.weights)), 1.0)

    x, y, theta = mcl.estimate()
    assert abs(x - 0.5825) < 0.01, x
    assert abs(y - 0.5825) < 0.01, y
    assert abs(theta) < 0.02, theta

    # Nothing may sit outside the walls.
    assert np.all(mcl.particles[:, 0] >= mcl.rect_map.x_min)
    assert np.all(mcl.particles[:, 0] <= mcl.rect_map.x_max)
    assert np.all(mcl.particles[:, 1] >= mcl.rect_map.y_min)
    assert np.all(mcl.particles[:, 1] <= mcl.rect_map.y_max)


def _test_estimate_uses_circular_mean():
    """Headings straddling +/-pi must average to pi, not to 0."""
    mcl = MCL(default_map(), default_beam_model(), SENSOR_CONFIGS,
              num_particles=2, rng_seed=1)
    mcl.particles[:] = [[0.5, 0.5, math.radians(179)],
                        [0.5, 0.5, math.radians(-179)]]
    mcl.weights[:] = 0.5
    _, _, theta = mcl.estimate()
    assert abs(abs(theta) - math.pi) < math.radians(2), math.degrees(theta)


def _run_tests():
    _test_defaults_match_calibration()
    _test_initialize_and_estimate()
    _test_estimate_uses_circular_mean()
    print('mcl tests passed')


if __name__ == '__main__':
    _run_tests()
