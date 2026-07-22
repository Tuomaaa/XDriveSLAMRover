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
    the body-frame (dx, dy, dtheta) instead. The alpha parameters keep only
    the same ROLE as Probabilistic Robotics Table 5.6 (uncertainty grows
    with distance travelled and angle turned), not the same NUMBERS: the
    book's alphas multiply squared deltas to produce a variance, while
    these multiply an un-squared delta to produce a standard deviation
    directly. Do not carry Table 5.6 values over here -- they would be off
    by orders of magnitude.
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

# Floor on the per-step process noise. Two separate reasons, and the second
# one is what actually sizes SIGMA_ROT_FLOOR.
#
# 1. A strictly proportional-to-motion model injects exactly zero noise when
#    the rover is parked, while update() keeps resampling every cycle --
#    which collapses 500 particles to 1 within about 1.5s of standstill.
#    A floor also reflects reality: this chassis's dominant odometry error
#    is systematic (~3% scale, 7-11deg yaw per run), which no
#    motion-proportional term can express at zero motion, and encoder
#    quantization bounds what position can be known regardless.
#
# 2. Heading is the weakly-observed dimension here (see the note in
#    predict), so resampling impoverishes it FIRST. CALIBRATED 2026-07-22
#    against a 60s synthetic wander in the box: at a 0.002 rot floor the
#    cloud's heading spread collapsed to 0.4deg within ~7s and never
#    recovered, and heading error then grew without bound -- 0.1deg at
#    start to 28.3deg by 52s, dragging position RMS to 10.2cm. Raising ONLY
#    this constant to 0.005+ (translation floor and alpha untouched) held
#    the spread near 1deg and the filter tracked heading to ~2deg, cutting
#    position RMS to 1.0cm. Behaviour is flat from 0.005 to 0.020, so 0.010
#    sits mid-plateau rather than on an edge; it degrades again by 0.040.
#    The translation floor was swept the same way and 0.001 is mid-plateau.
SIGMA_TRANS_FLOOR = 0.001   # metres per step
SIGMA_ROT_FLOOR = 0.010     # radians per step

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
    """Wrap to [-pi, pi]. Works on scalars and numpy arrays alike."""
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

    def predict(self, dx_body, dy_body, dtheta):
        """Advance every particle by a body-frame odometry delta plus noise.

        dx_body / dy_body are the translation the robot believes it made in
        its OWN frame since the last call; dtheta is the heading change.
        Each particle rotates that delta by its own heading, so particles
        that disagree about which way they are pointing end up in different
        places -- that divergence is exactly what update() then prunes.
        """
        count = self.num_particles
        a_trans_trans, a_trans_rot, a_rot_trans, a_rot_rot = self.alpha

        distance = math.hypot(dx_body, dy_body)
        turned = abs(dtheta)
        sigma_trans = max(a_trans_trans * distance + a_trans_rot * turned,
                          SIGMA_TRANS_FLOOR)
        sigma_rot = max(a_rot_trans * distance + a_rot_rot * turned,
                        SIGMA_ROT_FLOOR)

        # The floor keeps a parked cloud alive: predict(0, 0, 0) would
        # otherwise inject exactly zero spread, and update() resamples every
        # cycle regardless of whether the robot moved, so with no floor the
        # population collapses onto whichever particle currently scores
        # highest within a couple of seconds of standing still.
        noisy_dx = dx_body + self._rng.normal(0.0, sigma_trans, count)
        noisy_dy = dy_body + self._rng.normal(0.0, sigma_trans, count)
        noisy_dtheta = dtheta + self._rng.normal(0.0, sigma_rot, count)

        heading = self.particles[:, 2].copy()
        cos_h = np.cos(heading)
        sin_h = np.sin(heading)

        self.particles[:, 0] += noisy_dx * cos_h - noisy_dy * sin_h
        self.particles[:, 1] += noisy_dx * sin_h + noisy_dy * cos_h
        self.particles[:, 2] = _wrap_angle(heading + noisy_dtheta)
        self._clamp_to_map()

    def update(self, measurements):
        """Reweight particles against range readings, then resample.

        measurements aligns with sensor_configs; use None for a sensor that
        returned no valid echo this cycle. An all-None cycle is a no-op --
        with nothing observed there is nothing to correct, and resampling
        on uniform weights would only throw away diversity.
        """
        if len(measurements) != len(self.sensor_configs):
            raise ValueError('measurements must align with sensor_configs')
        if all(value is None for value in measurements):
            return

        log_weights = np.empty(self.num_particles)
        for index in range(self.num_particles):
            pose = (float(self.particles[index, 0]),
                    float(self.particles[index, 1]),
                    float(self.particles[index, 2]))
            log_weights[index] = self.beam_model.total_log_likelihood(
                measurements, pose, self.rect_map, self.sensor_configs)

        finite = np.isfinite(log_weights)
        if not np.any(finite):
            # Every particle is impossible. Keep the cloud and the uniform
            # weights rather than dividing by zero; the next measurement
            # gets another chance to discriminate.
            self.weights = np.full(
                self.num_particles, 1.0 / self.num_particles)
            return

        # Subtract the max before exponentiating: the log-likelihoods run
        # to a few hundred negative, and exp() of those underflows to a
        # column of zeros.
        shifted = log_weights - np.max(log_weights[finite])
        weights = np.exp(shifted)
        self.weights = weights / float(np.sum(weights))
        self._low_variance_resample()

    def _low_variance_resample(self):
        """Probabilistic Robotics Table 4.4: one random offset, then N
        evenly spaced draws through the cumulative weights.

        Cheaper and lower-variance than N independent draws -- a particle
        of weight w is guaranteed floor(w*N) or ceil(w*N) copies instead of
        a binomial spread around that.
        """
        count = self.num_particles
        positions = (self._rng.uniform(0.0, 1.0 / count)
                     + np.arange(count) / count)

        cumulative = np.cumsum(self.weights)
        cumulative[-1] = 1.0   # float error can leave the sum just under 1
        indexes = np.searchsorted(cumulative, positions)
        np.clip(indexes, 0, count - 1, out=indexes)

        self.particles = self.particles[indexes]
        self.weights = np.full(count, 1.0 / count)

    def estimate(self):
        """Weighted mean pose; theta uses a circular mean so that headings
        either side of +/-pi do not average to zero.

        In the normal predict/update flow this is actually an unweighted
        mean: update() ends every cycle with a resample that resets weights
        to uniform, so by the time estimate() is called there is nothing
        left to weight by. Like any mean estimator, it also assumes the
        cloud is unimodal -- a cloud with two separated clusters produces a
        pose in between them, not the more likely of the two.
        """
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


def _test_predict_advances_and_spreads():
    mcl = MCL(default_map(), default_beam_model(), SENSOR_CONFIGS,
              num_particles=2000, rng_seed=2)
    mcl.initialize(0.3, 0.5825, 0.0, xy_spread=0.01, theta_spread=0.01)
    spread_before = float(np.std(mcl.particles[:, 0]))

    mcl.predict(0.2, 0.0, 0.0)   # 200mm straight ahead, no turn

    x, y, theta = mcl.estimate()
    assert abs(x - 0.5) < 0.01, x          # world +x by 0.2
    assert abs(y - 0.5825) < 0.01, y       # no sideways drift
    assert abs(theta) < 0.02, theta
    assert float(np.std(mcl.particles[:, 0])) > spread_before


def _test_predict_respects_each_particle_heading():
    """Facing +90deg, a body-frame forward step must land on world +y."""
    mcl = MCL(default_map(), default_beam_model(), SENSOR_CONFIGS,
              num_particles=2000, rng_seed=3)
    mcl.initialize(0.5825, 0.3, math.pi / 2, xy_spread=0.01, theta_spread=0.01)
    mcl.predict(0.2, 0.0, 0.0)

    x, y, _ = mcl.estimate()
    assert abs(x - 0.5825) < 0.01, x
    assert abs(y - 0.5) < 0.01, y


def _test_predict_uses_per_particle_heading_not_a_shared_one():
    """Split the cloud between opposite headings and step forward.

    Particles facing 0 must move to +x while particles facing pi move to
    -x. Any implementation that rotates by one shared heading -- including
    a shared value derived from the particles, such as their mean -- moves
    the whole cloud the same way and fails here. The two tests above
    cannot catch that: their headings are a tight cluster, so a shared
    mean is numerically indistinguishable from each particle's own value.
    """
    mcl = MCL(default_map(), default_beam_model(), SENSOR_CONFIGS,
              num_particles=1000, rng_seed=11)
    mcl.initialize(0.5825, 0.5825, 0.0, xy_spread=0.001, theta_spread=0.001)
    half = mcl.num_particles // 2
    mcl.particles[:half, 2] = 0.0
    mcl.particles[half:, 2] = math.pi

    mcl.predict(0.2, 0.0, 0.0)

    forward_half = float(np.mean(mcl.particles[:half, 0]))
    backward_half = float(np.mean(mcl.particles[half:, 0]))
    assert forward_half > 0.75, forward_half
    assert backward_half < 0.42, backward_half


def _test_predict_keeps_a_parked_cloud_alive():
    """A stationary rover must not collapse the cloud.

    predict(0,0,0) injects zero motion-proportional noise, but update()
    resamples every cycle regardless, so without a noise floor the cloud
    drops to a single distinct particle within ~1.5s of standstill and the
    filter stops representing its own uncertainty.

    The heading assertion is the one that matters most. Heading is the
    weakly-observed dimension, so resampling impoverishes it first: at a
    0.002 rot floor the spread collapsed to 0.4deg and heading error then
    grew without bound over a 60s run. Position spread alone would not
    have caught that.
    """
    rect_map = default_map()
    mcl = MCL(rect_map, default_beam_model(), SENSOR_CONFIGS,
              num_particles=500, rng_seed=12)
    true_pose = (0.5825, 0.5825, 0.0)
    mcl.initialize(*true_pose, xy_spread=0.05, theta_spread=0.02)
    measurements = [rect_map.expected_range(true_pose, angle, offset)
                    for angle, offset in SENSOR_CONFIGS]

    for _ in range(40):          # 5 seconds at 8Hz
        mcl.predict(0.0, 0.0, 0.0)
        mcl.update(measurements)

    distinct = len(np.unique(mcl.particles[:, 0]))
    assert distinct > 20, f'cloud collapsed to {distinct} distinct particles'
    assert float(np.std(mcl.particles[:, 0])) > 1e-4, np.std(mcl.particles[:, 0])

    # Sustained tracking needs roughly a degree of heading spread; below
    # that the filter commits to a heading it can never revise.
    theta_spread = math.degrees(float(np.std(mcl.particles[:, 2])))
    assert theta_spread > 0.5, f'heading spread collapsed to {theta_spread:.2f}deg'


def _test_update_concentrates_on_the_truth():
    rect_map = default_map()
    mcl = MCL(rect_map, default_beam_model(), SENSOR_CONFIGS,
              num_particles=2000, rng_seed=4)
    true_pose = (0.5825, 0.5825, 0.0)
    mcl.initialize(*true_pose, xy_spread=0.05, theta_spread=0.02)

    measurements = [rect_map.expected_range(true_pose, angle, offset)
                    for angle, offset in SENSOR_CONFIGS]
    spread_before = float(np.std(mcl.particles[:, 1]))

    mcl.update(measurements)

    spread_after = float(np.std(mcl.particles[:, 1]))
    x, y, _ = mcl.estimate()
    assert abs(x - true_pose[0]) < 0.02, x
    assert abs(y - true_pose[1]) < 0.02, y
    assert spread_after < spread_before, (spread_before, spread_after)
    assert mcl.particles.shape == (2000, 3), mcl.particles.shape
    assert math.isclose(float(np.sum(mcl.weights)), 1.0)


def _test_update_tolerates_a_dead_sensor():
    """A None entry is skipped, not treated as a zero-range reading."""
    rect_map = default_map()
    mcl = MCL(rect_map, default_beam_model(), SENSOR_CONFIGS,
              num_particles=1000, rng_seed=5)
    true_pose = (0.5825, 0.5825, 0.0)
    mcl.initialize(*true_pose, xy_spread=0.05, theta_spread=0.02)

    back_range = rect_map.expected_range(true_pose, *BACK_SENSOR)
    mcl.update([None, back_range])

    x, _, _ = mcl.estimate()
    assert abs(x - true_pose[0]) < 0.02, x
    assert math.isclose(float(np.sum(mcl.weights)), 1.0)

    # All-None must be a no-op rather than a divide-by-zero.
    before = mcl.particles.copy()
    mcl.update([None, None])
    assert np.array_equal(before, mcl.particles)


def _test_resample_duplicates_by_weight():
    """A single dominant particle must take over the whole population."""
    mcl = MCL(default_map(), default_beam_model(), SENSOR_CONFIGS,
              num_particles=100, rng_seed=6)
    mcl.particles[:, 0] = np.arange(100) / 100.0
    mcl.particles[:, 1] = 0.5
    mcl.particles[:, 2] = 0.0
    mcl.weights[:] = 0.0
    mcl.weights[42] = 1.0

    mcl._low_variance_resample()

    assert np.allclose(mcl.particles[:, 0], 0.42), mcl.particles[:5, 0]
    assert mcl.particles.shape == (100, 3)
    assert math.isclose(float(np.sum(mcl.weights)), 1.0)


def _run_tests():
    _test_defaults_match_calibration()
    _test_initialize_and_estimate()
    _test_estimate_uses_circular_mean()
    _test_predict_advances_and_spreads()
    _test_predict_respects_each_particle_heading()
    _test_predict_uses_per_particle_heading_not_a_shared_one()
    _test_predict_keeps_a_parked_cloud_alive()
    _test_update_concentrates_on_the_truth()
    _test_update_tolerates_a_dead_sensor()
    _test_resample_duplicates_by_weight()
    print('mcl tests passed')


if __name__ == '__main__':
    _run_tests()
