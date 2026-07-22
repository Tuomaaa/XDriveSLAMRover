# MCL Particle Filter Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement Monte Carlo Localization (Probabilistic Robotics Ch.8) that localizes the X-drive rover inside the measured KT-board rectangle using encoder odometry for prediction and two HC-SR04 sensors for correction, validated offline against recorded robot data.

**Architecture:** Four flat modules in `ros2_ws/src/`. `mcl.py` holds the filter (predict / update / resample / estimate) and knows nothing about CAN, ROS, or files. `mcl_log.py` owns the CSV contract so the logger and the replay cannot drift apart. `can_bridge_node.py` gains opt-in logging. `mcl_offline.py` replays a log and renders a GIF. Online ROS integration (`mcl_node.py`) is deliberately out of scope until offline replay works.

**Tech Stack:** Python 3, numpy (particle arrays and vectorized resampling), matplotlib (animation), plus the existing `map_model.RectMap` and `measurement_model.BeamModel`. Both numpy (2.4.2) and matplotlib (3.11.0) are present locally; on the Pi, ROS2 Jazzy already pulls in numpy, and matplotlib is only needed by `mcl_offline.py`, which runs on the laptop.

**Verification status:** every code block in Tasks 1-5 and 7 was written out and executed before this plan was finalized. All unit tests pass, the convergence test passes deterministically across runs, and the synthetic end-to-end replay produces a correct GIF. The expected outputs quoted below are real captured output, not estimates. Tasks 6 and 8 need hardware and are unverified.

## Global Constraints

- Flat imports only (`from mcl import ...`). There is no colcon package in `ros2_ws/src/`; every script runs as `python3 xxx.py`. Match the existing files.
- Tests follow the project convention: a module-level `_run_tests()` invoked from `if __name__ == '__main__':`, using bare `assert`. There is no pytest and no `tests/` directory. Do not introduce either.
- All distances in the filter are **metres**, all angles **radians**. Only the CSV stores millimetres, matching the CAN payload and `ultrasonic_collect.py`.
- REP-103 body frame: **+x forward, +y left, +theta CCW**. `odometry.py` already emits this.
- Week 5 calibration values, copied verbatim, are the defaults: `sigma_hit = (0.0017, 0.0078)`, `w_hit=0.94`, `w_short=0.01`, `w_max=0.03`, `w_rand=0.02`, `z_max=4.0`, `lambda_short=0.5`.
- Map bounds: `RectMap(0.0, 1.165, 0.0, 1.165)` — the measured KT-board square.
- Sensor mounting: right `(-pi/2, (0.0, -0.09))`, back `(pi, (-0.09, 0.0))`.
- Every test that draws random numbers must pass a fixed seed so failures are reproducible.
- Work from the repo root: `C:\Users\panh3\Documents\ROSE\Personal Project\SummerSLAM`. All test commands run from `ros2_ws/src/`.

---

## Amendments — read before transcribing any code block below

This plan was executed on branch `week6-mcl`. Task and final reviews found six defects **in this plan's own code blocks**. The shipped code is correct; the code blocks below are not, except where a task's text says otherwise. If you are re-executing this plan, apply these on top:

| # | Plan defect | Shipped fix |
|---|---|---|
| 1 | Task 2's two `predict` tests both use a tight heading cluster (`theta_spread=0.01`), so rotating by a single shared *mean* heading passes them — proven by mutation. The property the whole filter depends on had no regression net. | `0dc27f5` adds `_test_predict_uses_per_particle_heading_not_a_shared_one`, splitting the cloud between 0 and pi so the halves must move in opposite directions. |
| 2 | Task 5's round-trip test uses `odom_x == odom_y` in its fixture and never asserts `timestamp_s`. Swapping the coordinates or dropping the timestamp inside `build_row` both passed — proven by mutation. | `187a06d` gives row 0 distinct x/y and asserts all six fields `read_log` returns, on both rows. |
| 3 | Task 7's `main()` does `frames[-1]` with no guard. A one-row log clears `read_log`'s empty check but produces zero filter steps, so it dies on `IndexError`. A recording cut short after one CAN frame is plausible. | `2ffe573` exits with a clear message. Also validates `--fps`, which could reach a division by zero. |
| 4 | **`predict` injects exactly zero noise when the rover is stationary**, while `update()` resamples every cycle regardless — collapsing 500 particles to 1 within ~1.5s of standstill. Every recording opens with idle rows, so this is not a corner case. | `e5a2033` adds `SIGMA_TRANS_FLOOR` / `SIGMA_ROT_FLOOR`, which also reflect this chassis's *systematic* (not motion-proportional) odometry error. Regression test included. |
| 5 | Task 7's replay CLI hardcodes the library default `alpha`, though tuning it against a recording is the tool's stated purpose and the default scores materially worse (3.75cm vs 2.29cm on the test's own scenario). | `e5a2033` adds `--alpha`. |
| 6 | Task 6 opens the log with mode `'w'`, silently truncating an existing recording if the operator forgets to change the parameter. Hardware sessions are expensive to repeat. | `e5a2033` refuses to overwrite. |

Also corrected: the heading-observability explanation in Task 4 Step 3 and Task 8 Step 4 was wrong in mechanism (it said heading was *weakly* observed and noise-swamped, and gave the position bias in the wrong direction). Both sections now carry the exact statement. See Task 4 Step 3.

Task 4's expected MCL error is now **2.1cm**, not 2.3cm — fix 4's rotation floor engages in that scenario.

---

## File Structure

| File | Responsibility |
|------|---------------|
| `ros2_ws/src/mcl.py` | **Create.** Particle filter core + Week 5 calibrated defaults. Depends on `map_model`, `measurement_model`, numpy. |
| `ros2_ws/src/mcl_log.py` | **Create.** CSV column list, row builder, row parser. Pure stdlib — importable by the ROS node without dragging in numpy or matplotlib. |
| `ros2_ws/src/mcl_test.py` | **Create.** Hardware-free convergence test: simulated robot, drifting odometry, checks MCL beats dead reckoning. |
| `ros2_ws/src/mcl_offline.py` | **Create.** CSV replay + GIF rendering. Owns the odom→map frame math. |
| `ros2_ws/src/can_bridge_node.py` | **Modify.** Opt-in `mcl_log` parameter writing one row per ultrasonic frame. |

---

### Task 1: MCL scaffold — calibrated defaults, initialization, estimate

**Files:**
- Create: `ros2_ws/src/mcl.py`

**Interfaces:**
- Consumes: `map_model.RectMap(x_min, x_max, y_min, y_max)`; `measurement_model.BeamModel(z_max, sigma_hit, lambda_short, w_hit, w_short, w_max, w_rand)`.
- Produces: module constants `SENSOR_CONFIGS`, `MAP_BOUNDS`, `SIGMA_HIT`, `BEAM_WEIGHTS`, `Z_MAX`, `LAMBDA_SHORT`; factories `default_map() -> RectMap` and `default_beam_model() -> BeamModel`; helper `_wrap_angle(array_or_float)`; class `MCL(rect_map, beam_model, sensor_configs, num_particles=500, alpha=(0.05, 0.01, 0.01, 0.05), rng_seed=None)` with attributes `particles` (numpy (N,3) of x,y,theta), `weights` (numpy (N,)), and methods `initialize(x, y, theta, xy_spread=0.05, theta_spread=0.1) -> None` and `estimate() -> (x, y, theta)`.

- [ ] **Step 1: Write the failing test**

Create `ros2_ws/src/mcl.py` containing only the test block for now:

```python
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
```

- [ ] **Step 2: Run test to verify it fails**

Run from `ros2_ws/src/`:

```bash
python mcl.py
```

Expected: `NameError: name 'MCL' is not defined` (the constants tests pass, `_test_initialize_and_estimate` fails).

- [ ] **Step 3: Write minimal implementation**

Insert the `MCL` class into `mcl.py` between `_wrap_angle` and `_test_defaults_match_calibration`:

```python
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
```

- [ ] **Step 4: Run test to verify it passes**

```bash
python mcl.py
```

Expected: `mcl tests passed`

- [ ] **Step 5: Commit**

```bash
git add ros2_ws/src/mcl.py && git commit -m "week6: MCL scaffold with calibrated defaults, init, estimate"
```

---

### Task 2: Motion model (`predict`)

**Files:**
- Modify: `ros2_ws/src/mcl.py`

**Interfaces:**
- Consumes: `MCL.particles`, `MCL.alpha`, `MCL._rng`, `MCL._clamp_to_map()`, `_wrap_angle()` from Task 1.
- Produces: `MCL.predict(dx_body, dy_body, dtheta) -> None`, which advances every particle by a noisy copy of that body-frame delta rotated into the world by *that particle's own* heading.

- [ ] **Step 1: Write the failing test**

Add these two test functions to `mcl.py` above `_run_tests`, and add the two calls into `_run_tests` after `_test_estimate_uses_circular_mean()`:

```python
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
```

- [ ] **Step 2: Run test to verify it fails**

```bash
python mcl.py
```

Expected: `AttributeError: 'MCL' object has no attribute 'predict'`

- [ ] **Step 3: Write minimal implementation**

Add `predict` to the `MCL` class, directly after `initialize`:

```python
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
        sigma_trans = a_trans_trans * distance + a_trans_rot * turned
        sigma_rot = a_rot_trans * distance + a_rot_rot * turned

        # numpy accepts scale=0.0 and returns the mean, so a stationary
        # step correctly adds no spread.
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
```

- [ ] **Step 4: Run test to verify it passes**

```bash
python mcl.py
```

Expected: `mcl tests passed`

- [ ] **Step 5: Commit**

```bash
git add ros2_ws/src/mcl.py && git commit -m "week6: MCL motion model with holonomic odometry noise"
```

---

### Task 3: Measurement update and low-variance resampling

**Files:**
- Modify: `ros2_ws/src/mcl.py`

**Interfaces:**
- Consumes: `BeamModel.total_log_likelihood(measurements, pose, rect_map, sensor_configs) -> float` from `measurement_model.py`; `RectMap.expected_range(pose, sensor_angle, sensor_offset) -> float`.
- Produces: `MCL.update(measurements) -> None`, where `measurements` is a sequence aligned with `sensor_configs` whose entries are ranges in metres or `None` for a sensor with no valid echo. Also `MCL._low_variance_resample() -> None`.

- [ ] **Step 1: Write the failing test**

Add these three test functions to `mcl.py` above `_run_tests`, and add the three calls into `_run_tests`:

```python
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
```

- [ ] **Step 2: Run test to verify it fails**

```bash
python mcl.py
```

Expected: `AttributeError: 'MCL' object has no attribute 'update'`

- [ ] **Step 3: Write minimal implementation**

Add `update` and `_low_variance_resample` to the `MCL` class, directly after `predict`:

```python
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
```

- [ ] **Step 4: Run test to verify it passes**

```bash
python mcl.py
```

Expected: `mcl tests passed`

- [ ] **Step 5: Commit**

```bash
git add ros2_ws/src/mcl.py && git commit -m "week6: MCL measurement update + low-variance resampling"
```

---

### Task 4: Convergence test against dead reckoning

**Files:**
- Create: `ros2_ws/src/mcl_test.py`

**Interfaces:**
- Consumes: `MCL`, `SENSOR_CONFIGS`, `default_map`, `default_beam_model` from `mcl.py`.
- Produces: `run_convergence_test() -> None`. Nothing imports this; it is a standalone check.

This is the task that proves the three pieces work together. It mimics the rover's real failure mode — open-loop yaw drift, recorded in PROJECT_STATE.md as -7 to -11 degrees over a straight run — and asserts the filter beats raw integration.

**This task does not follow the red-green cycle**, unlike Tasks 1-3, 5 and 7. It is an integration check written over units that are already implemented and tested, so it is expected to pass on first run. There is no red phase to observe; a failure here means one of Tasks 1-3 is wrong, not that this test is doing its job.

- [ ] **Step 1: Write the integration test**

Create `ros2_ws/src/mcl_test.py`:

```python
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

    assert mcl_error < 0.05, f'MCL off by {mcl_error * 100:.1f}cm'
    assert mcl_error < dead_error, (mcl_error, dead_error)
    print('\nMCL convergence test passed')


if __name__ == '__main__':
    run_convergence_test()
```

- [ ] **Step 2: Run the test and expect it to pass**

```bash
python mcl_test.py
```

Expected: PASS, given Tasks 1-3 are correct. If it fails, the printed errors localize the fault: a large MCL error with a small dead-reckoning error means the measurement update is mis-wired; both large means the motion model is.

This plan's code was run before being written down. The output at the time was:

```
true       x=0.600 y=0.600
dead-reck  x=0.539 y=0.603  error=6.2cm
mcl        x=0.580 y=0.588  error=2.3cm  theta=+12.6deg

MCL convergence test passed
```

**Amended after the final review:** the process-noise floor added in commit `e5a2033` (see Amendments at the top of this plan) engages on the rotation term in this scenario, so the MCL error is now **2.1cm** and the assertions were tightened to `mcl_error < 0.04` and `mcl_error < dead_error / 2`. Dead reckoning is unchanged at 6.2cm.

- [ ] **Step 3: Understand why the MCL error is ~2cm and not ~0**

Do not "fix" this — it is the real behaviour of this sensor arrangement, and recognizing it is the point of the exercise.

The filter ends up believing `theta=+12.6deg` when the truth is 0. That is the injected yaw bias (0.002 rad × 120 steps = 13.8deg) being integrated **completely uncorrected**, and the reason is stronger than "the sensors are noisy":

**Heading is not weakly observed here — it is exactly unobservable.** Both beams reference walls through the same origin corner (right measures to `y_min`, back to `x_min`), so each predicted range reduces to `coord/cos(theta) - 0.09`. That means the entire one-parameter family of poses

```
(x·cos(theta),  y·cos(theta),  theta)
```

predicts **bit-identical** ranges for every theta. Sweeping theta from 0 to 30 degrees changes the predicted pair by between `0` and `1.1e-16` metres — machine epsilon. The two-beam measurement is rank-2 over a 3-DOF state, and this curve is its null direction. No amount of averaging over 120 steps recovers heading, because there is no information there to average.

It is worth being precise that noise is *not* the explanation. If position were pinned at the truth, a 12.6deg rotation would shift each range by 14.8mm against a calibrated sigma of 5.7mm at 0.51m — 2.6 sigma per reading, tens of sigma over the run. That would be crushed, not swamped. The filter tolerates the heading error only because it can slide its *position* along the degenerate curve at zero likelihood cost.

**The position bias runs toward the walls, not away.** Following that curve, the filter reports `x·cos(theta)` — so at 12.6deg it sits about 2.4% *closer* to each measured wall, roughly `0.60 × (1 - cos(12.6deg)) = 1.44cm` per axis (the 90mm sensor offset cancels exactly), giving about 2.0cm combined. The remainder is particle noise. The observed `(0.580, 0.588)` against a true `(0.600, 0.600)` confirms the direction: both below truth.

Position is otherwise fine because x and y *are* directly observable; heading is not. Carry this into Task 8's write-up rather than tuning it away. The fix is a third sensor or the Week 11 IMU, not a parameter — and note *which* third sensor: it must face an **opposing** wall. A front-facing beam measures `(x_max - x)`, which does not rescale the way `x` does, so the common `1/cos` factor no longer cancels and the degeneracy breaks (5.3 sigma separation at 12.6deg). A third beam parallel to an existing one would add nothing.

Copy the printed `dead-reck` and `mcl` figures into the commit message so the improvement is on the record.

- [ ] **Step 4: Re-run to confirm determinism**

```bash
python mcl_test.py
```

Expected: byte-identical output to Step 2. Both seeds are fixed; any variation means a seed was missed.

- [ ] **Step 5: Commit**

```bash
git add ros2_ws/src/mcl_test.py && git commit -m "week6: MCL convergence test vs dead reckoning"
```

---

### Task 5: CSV log format module

**Files:**
- Create: `ros2_ws/src/mcl_log.py`

**Interfaces:**
- Consumes: nothing beyond the standard library. This module must stay numpy- and matplotlib-free so `can_bridge_node.py` can import it cheaply on the Pi.
- Produces: `COLUMNS` (tuple of 8 header strings); `build_row(timestamp, pose, reading) -> tuple`, where `pose` is `(x, y, theta)` and `reading` is the `protocol.decode()` dict for CAN 0x202; `read_log(path) -> list[dict]` with keys `timestamp_s`, `odom_x`, `odom_y`, `odom_theta`, `right_m`, `back_m` (the two range keys are `float` metres or `None`).

- [ ] **Step 1: Write the failing test**

Create `ros2_ws/src/mcl_log.py` with the docstring, imports, and test block only:

```python
"""CSV format shared by the MCL logger (can_bridge_node) and the replay
(mcl_offline), so the two cannot drift apart.

One row per ultrasonic frame (~8Hz), carrying the most recent odometry
pose. One row is therefore exactly one MCL predict+update cycle. Ranges are
stored in millimetres to match the CAN payload and ultrasonic_collect.py;
read_log converts to metres, which is what the filter wants.

Stdlib only on purpose -- the ROS node imports this and should not pull in
numpy or matplotlib to write a CSV.
"""

import csv


COLUMNS = (
    'timestamp_s', 'odom_x', 'odom_y', 'odom_theta',
    'right_mm', 'back_mm', 'right_valid', 'back_valid',
)


def _run_tests():
    import tempfile
    from pathlib import Path

    valid = {'right_mm': 493, 'back_mm': 392,
             'right_valid': True, 'back_valid': True}
    row = build_row(12.5, (0.5825, 0.5825, 0.0), valid)
    assert len(row) == len(COLUMNS), row
    assert row[4] == 493 and row[6] == 1, row

    # right_mm carries the sentinel when the echo timed out; right_valid=0
    # is what read_log keys off, and the sentinel must not leak through.
    invalid_right = {'right_mm': 65535, 'back_mm': 392,
                     'right_valid': False, 'back_valid': True}

    with tempfile.TemporaryDirectory() as folder:
        path = Path(folder) / 'log.csv'
        with path.open('w', newline='', encoding='utf-8') as sink:
            writer = csv.writer(sink)
            writer.writerow(COLUMNS)
            writer.writerow(build_row(12.5, (0.5825, 0.5825, 0.0), valid))
            writer.writerow(build_row(12.6, (0.60, 0.58, 0.10), invalid_right))
        entries = read_log(path)

        empty = Path(folder) / 'empty.csv'
        with empty.open('w', newline='', encoding='utf-8') as sink:
            csv.writer(sink).writerow(COLUMNS)
        try:
            read_log(empty)
        except ValueError:
            pass
        else:
            raise AssertionError('an empty log should raise')

    assert len(entries) == 2, entries
    assert abs(entries[0]['odom_x'] - 0.5825) < 1e-9, entries[0]
    assert abs(entries[0]['right_m'] - 0.493) < 1e-9, entries[0]
    assert abs(entries[0]['back_m'] - 0.392) < 1e-9, entries[0]
    assert entries[1]['right_m'] is None, entries[1]
    assert abs(entries[1]['back_m'] - 0.392) < 1e-9, entries[1]
    assert abs(entries[1]['odom_theta'] - 0.10) < 1e-9, entries[1]
    print('mcl_log tests passed')


if __name__ == '__main__':
    _run_tests()
```

- [ ] **Step 2: Run test to verify it fails**

```bash
python mcl_log.py
```

Expected: `NameError: name 'build_row' is not defined`

- [ ] **Step 3: Write minimal implementation**

Insert both functions into `mcl_log.py` between `COLUMNS` and `_run_tests`:

```python
def build_row(timestamp, pose, reading):
    """Format one log row.

    pose is (x, y, theta) in metres and radians; reading is the dict
    protocol.decode() returns for CAN 0x202.
    """
    x, y, theta = pose
    return (
        f'{timestamp:.6f}',
        f'{x:.6f}',
        f'{y:.6f}',
        f'{theta:.6f}',
        reading['right_mm'],
        reading['back_mm'],
        int(bool(reading['right_valid'])),
        int(bool(reading['back_valid'])),
    )


def read_log(path):
    """Parse a log file into a list of dicts with metre-valued ranges.

    A sensor whose validity bit is clear comes back as None rather than a
    number, so the filter skips it instead of scoring against the 0xFFFF
    timeout sentinel.
    """
    entries = []
    with open(path, newline='', encoding='utf-8') as source:
        reader = csv.DictReader(source)
        if (reader.fieldnames is None
                or not set(COLUMNS).issubset(reader.fieldnames)):
            raise ValueError(f'{path}: missing required columns {COLUMNS}')
        for row in reader:
            entries.append({
                'timestamp_s': float(row['timestamp_s']),
                'odom_x': float(row['odom_x']),
                'odom_y': float(row['odom_y']),
                'odom_theta': float(row['odom_theta']),
                'right_m': (float(row['right_mm']) / 1000.0
                            if int(row['right_valid']) else None),
                'back_m': (float(row['back_mm']) / 1000.0
                           if int(row['back_valid']) else None),
            })
    if not entries:
        raise ValueError(f'{path}: no rows')
    return entries
```

- [ ] **Step 4: Run test to verify it passes**

```bash
python mcl_log.py
```

Expected: `mcl_log tests passed`

- [ ] **Step 5: Commit**

```bash
git add ros2_ws/src/mcl_log.py && git commit -m "week6: shared CSV contract for MCL logging and replay"
```

---

### Task 6: Opt-in MCL logging in the CAN bridge node

**Files:**
- Modify: `ros2_ws/src/can_bridge_node.py`

**Interfaces:**
- Consumes: `mcl_log.COLUMNS` and `mcl_log.build_row` from Task 5.
- Produces: a ROS2 parameter `mcl_log` (string path, `''` disables). No Python API for later tasks — the deliverable is the CSV file.

This task has no automated test: it is IO against a live CAN bus, and the format logic it would test already has one in Task 5. Verification is the manual smoke check in Step 4.

- [ ] **Step 1: Add the import and the parameter**

In `ros2_ws/src/can_bridge_node.py`, add `import csv` immediately above `import math` at line 21, and add the `mcl_log` import after the `from odometry import OdometryEstimator` line:

```python
from odometry import OdometryEstimator
from mcl_log import COLUMNS as MCL_LOG_COLUMNS, build_row as build_mcl_log_row
```

Then, inside `__init__`, immediately after the `self._ultrasonic_update_count = 0` line, insert:

```python
        # ── MCL data logging (opt-in) ──
        # One row per ultrasonic frame (~8Hz) carrying the latest odometry
        # pose: one row is one MCL predict+update cycle, which is exactly
        # what mcl_offline.py replays. Off unless a path is given.
        log_path = self.declare_parameter('mcl_log', '').value
        self._mcl_log_file = None
        self._mcl_log_writer = None
        self._last_pose = None
        if log_path:
            self._mcl_log_file = open(
                log_path, 'w', newline='', encoding='utf-8')
            self._mcl_log_writer = csv.writer(self._mcl_log_file)
            self._mcl_log_writer.writerow(MCL_LOG_COLUMNS)
            self.get_logger().info(f'MCL log -> {log_path}')
```

- [ ] **Step 2: Record the pose and write the rows**

In `_on_encoder_complete`, immediately after the line `x, y, theta, vx, vy, omega = result`, insert:

```python
        self._last_pose = (x, y, theta)
```

In `_on_ultrasonic`, immediately after the `self._publish_range(...)` call for the back sensor and before `self._ultrasonic_update_count += 1`, insert:

```python
        # Skip rows until odometry has produced its first pose, otherwise
        # the replay would start from a pose that was never measured.
        if self._mcl_log_writer is not None and self._last_pose is not None:
            self._mcl_log_writer.writerow(
                build_mcl_log_row(timestamp, self._last_pose, reading))
            self._mcl_log_file.flush()
```

In `destroy_node`, insert before `self._can.shutdown()`:

```python
        if self._mcl_log_file is not None:
            self._mcl_log_file.close()
```

- [ ] **Step 3: Verify it imports cleanly**

```bash
python -c "import ast,sys; ast.parse(open('can_bridge_node.py').read()); print('syntax OK')"
```

Expected: `syntax OK`. A full import needs `rclpy`, which is only present on the Pi, so parse-check here and import-check on the Pi in the next step.

- [ ] **Step 4: Smoke test on the Pi**

Push, pull on the Pi, then with the rover powered and driving under PS2 control:

```bash
source /opt/ros/jazzy/setup.bash && python3 can_bridge_node.py --ros-args -p mcl_log:=/tmp/mcl_run1.csv
```

Drive a slow loop inside the KT-board square for ~60 seconds, note the true starting pose to the millimetre, then Ctrl-C. Confirm the file has the 8-column header, roughly 8 rows per second, `odom_*` values that change as the rover moves, and `right_valid`/`back_valid` mostly 1.

- [ ] **Step 5: Commit**

```bash
git add ros2_ws/src/can_bridge_node.py && git commit -m "week6: opt-in MCL CSV logging in can_bridge_node"
```

---

### Task 7: Offline replay and animation

**Files:**
- Create: `ros2_ws/src/mcl_offline.py`

**Interfaces:**
- Consumes: `read_log` from `mcl_log.py`; `MCL`, `SENSOR_CONFIGS`, `default_map`, `default_beam_model` from `mcl.py`.
- Produces: `body_frame_delta(previous, current) -> (dx, dy, dtheta)`; `odom_to_map(odom_pose, odom_origin, start_pose) -> (x, y, theta)`; `replay(entries, mcl, start_pose) -> list[dict]`; `render(frames, rect_map, output, fps, stride) -> None`; a CLI.

- [ ] **Step 1: Write the failing test**

Create `ros2_ws/src/mcl_offline.py` with the docstring, imports, and test block only:

```python
"""Replay a recorded MCL log through the particle filter and render a GIF.

The log's odometry lives in the odom frame, which starts at (0, 0, 0)
wherever the rover happened to be switched on. The filter never sees that
origin -- it consumes deltas only -- so the true starting pose in the map
has to be supplied on the command line. It is also what the dead-reckoning
trace is anchored to.

Usage:
    python3 mcl_offline.py /tmp/mcl_run1.csv --start 0.30 0.30 0 \\
        --output mcl_run1.gif
"""

import argparse
import math
from pathlib import Path

from mcl import MCL, SENSOR_CONFIGS, default_beam_model, default_map
from mcl_log import read_log


def _run_tests():
    # Facing +x, a world +x move is a straight-ahead body move.
    dx, dy, dtheta = body_frame_delta((0.0, 0.0, 0.0), (0.2, 0.0, 0.0))
    assert abs(dx - 0.2) < 1e-9, dx
    assert abs(dy) < 1e-9, dy
    assert abs(dtheta) < 1e-9, dtheta

    # Same world move while facing +90deg: that is a step to the robot's
    # RIGHT, which is -y in REP-103 body coordinates.
    dx, dy, _ = body_frame_delta((0.0, 0.0, math.pi / 2),
                                 (0.2, 0.0, math.pi / 2))
    assert abs(dx) < 1e-9, dx
    assert abs(dy + 0.2) < 1e-9, dy

    # Heading wrap: +170deg -> -170deg is +20deg, not -340deg.
    _, _, dtheta = body_frame_delta((0.0, 0.0, math.radians(170)),
                                    (0.0, 0.0, math.radians(-170)))
    assert abs(dtheta - math.radians(20)) < 1e-9, math.degrees(dtheta)

    # Odom starts at the origin but the rover actually started at
    # (0.3, 0.4) facing +90deg, so 0.2m of odom +x is 0.2m of map +y.
    mapped = odom_to_map((0.2, 0.0, 0.0), (0.0, 0.0, 0.0),
                         (0.3, 0.4, math.pi / 2))
    assert abs(mapped[0] - 0.3) < 1e-9, mapped
    assert abs(mapped[1] - 0.6) < 1e-9, mapped
    assert abs(mapped[2] - math.pi / 2) < 1e-9, mapped

    print('mcl_offline tests passed')


if __name__ == '__main__':
    main()
```

- [ ] **Step 2: Run test to verify it fails**

```bash
python -c "import mcl_offline; mcl_offline._run_tests()"
```

Expected: `NameError: name 'body_frame_delta' is not defined`

- [ ] **Step 3: Write the frame math**

Insert into `mcl_offline.py` between the imports and `_run_tests`:

```python
def body_frame_delta(previous, current):
    """World-frame pose pair -> the (dx, dy, dtheta) the robot felt.

    Rotating the world displacement by -previous_theta undoes the heading
    it was expressed in, leaving the step in the robot's own frame -- which
    is what MCL.predict wants.
    """
    dx_world = current[0] - previous[0]
    dy_world = current[1] - previous[1]
    cos_t = math.cos(previous[2])
    sin_t = math.sin(previous[2])
    dtheta = current[2] - previous[2]
    return (
        dx_world * cos_t + dy_world * sin_t,
        -dx_world * sin_t + dy_world * cos_t,
        math.atan2(math.sin(dtheta), math.cos(dtheta)),
    )


def odom_to_map(odom_pose, odom_origin, start_pose):
    """Express an odom-frame pose in map coordinates.

    The filter only ever consumes deltas, so the odom origin is arbitrary;
    this exists purely to draw the dead-reckoning trace beside the estimate.
    """
    dx, dy, dtheta = body_frame_delta(odom_origin, odom_pose)
    x0, y0, theta0 = start_pose
    heading = theta0 + dtheta
    return (
        x0 + dx * math.cos(theta0) - dy * math.sin(theta0),
        y0 + dx * math.sin(theta0) + dy * math.cos(theta0),
        math.atan2(math.sin(heading), math.cos(heading)),
    )
```

- [ ] **Step 4: Run test to verify it passes**

```bash
python -c "import mcl_offline; mcl_offline._run_tests()"
```

Expected: `mcl_offline tests passed`

- [ ] **Step 5: Write the replay loop, renderer, and CLI**

Insert into `mcl_offline.py` between `odom_to_map` and `_run_tests`:

```python
def replay(entries, mcl, start_pose):
    """Run the filter over a parsed log. Returns one frame dict per step."""
    mcl.initialize(*start_pose)
    origin = (entries[0]['odom_x'], entries[0]['odom_y'],
              entries[0]['odom_theta'])
    previous = origin
    frames = []

    for entry in entries[1:]:
        current = (entry['odom_x'], entry['odom_y'], entry['odom_theta'])
        measurements = [entry['right_m'], entry['back_m']]
        mcl.predict(*body_frame_delta(previous, current))
        mcl.update(measurements)
        previous = current

        frames.append({
            'particles': mcl.particles.copy(),
            'estimate': mcl.estimate(),
            'dead_reckoning': odom_to_map(current, origin, start_pose),
            'measurements': measurements,
            'timestamp': entry['timestamp_s'],
        })

    return frames


def render(frames, rect_map, output, fps=10, stride=2):
    """Animate particles, estimate, and dead reckoning into a GIF."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter

    shown = frames[::stride]
    figure, axes = plt.subplots(figsize=(6, 6))
    axes.set(
        xlim=(rect_map.x_min - 0.05, rect_map.x_max + 0.05),
        ylim=(rect_map.y_min - 0.05, rect_map.y_max + 0.05),
        xlabel='x (m)', ylabel='y (m)',
    )
    axes.set_aspect('equal')
    axes.plot(
        [rect_map.x_min, rect_map.x_max, rect_map.x_max,
         rect_map.x_min, rect_map.x_min],
        [rect_map.y_min, rect_map.y_min, rect_map.y_max,
         rect_map.y_max, rect_map.y_min],
        color='0.4', linewidth=2)

    cloud = axes.scatter([], [], s=3, alpha=0.25, color='tab:blue',
                         label='particles')
    estimate_dot, = axes.plot([], [], 'o', color='tab:red', markersize=7,
                              label='MCL estimate')
    estimate_heading, = axes.plot([], [], '-', color='tab:red', linewidth=2)
    dead_dot, = axes.plot([], [], 'o', color='tab:green', markersize=6,
                          label='dead reckoning')
    estimate_trail, = axes.plot([], [], '-', color='tab:red', linewidth=1,
                                alpha=0.5)
    dead_trail, = axes.plot([], [], '-', color='tab:green', linewidth=1,
                            alpha=0.5)
    right_ray, = axes.plot([], [], '-', color='tab:orange', linewidth=1,
                           alpha=0.8, label='sensor beams')
    back_ray, = axes.plot([], [], '-', color='tab:orange', linewidth=1,
                          alpha=0.8)
    sensor_rays = (right_ray, back_ray)
    axes.legend(loc='upper right', fontsize=8)

    def draw(index):
        frame = shown[index]
        cloud.set_offsets(frame['particles'][:, :2])

        x, y, theta = frame['estimate']
        estimate_dot.set_data([x], [y])
        estimate_heading.set_data(
            [x, x + 0.08 * math.cos(theta)],
            [y, y + 0.08 * math.sin(theta)])

        dead_x, dead_y, _ = frame['dead_reckoning']
        dead_dot.set_data([dead_x], [dead_y])

        # Beams drawn from the estimated pose out to the measured range.
        # A beam that stops short of the wall means the estimate is too far
        # from that wall; one that overshoots means it is too close.
        for ray, (angle, offset), measured in zip(
                sensor_rays, SENSOR_CONFIGS, frame['measurements']):
            if measured is None:
                ray.set_data([], [])
                continue
            origin_x = x + offset[0] * math.cos(theta) - offset[1] * math.sin(theta)
            origin_y = y + offset[0] * math.sin(theta) + offset[1] * math.cos(theta)
            bearing = theta + angle
            ray.set_data(
                [origin_x, origin_x + measured * math.cos(bearing)],
                [origin_y, origin_y + measured * math.sin(bearing)])

        history = shown[:index + 1]
        estimate_trail.set_data(
            [item['estimate'][0] for item in history],
            [item['estimate'][1] for item in history])
        dead_trail.set_data(
            [item['dead_reckoning'][0] for item in history],
            [item['dead_reckoning'][1] for item in history])

        axes.set_title(f'MCL  t={frame["timestamp"] - shown[0]["timestamp"]:5.1f}s'
                       f'   x={x:.3f} y={y:.3f} '
                       f'theta={math.degrees(theta):+.0f}deg')
        return (cloud, estimate_dot, estimate_heading, dead_dot,
                estimate_trail, dead_trail) + sensor_rays

    animation = FuncAnimation(
        figure, draw, frames=len(shown), interval=1000 // fps, blit=False)
    animation.save(str(output), writer=PillowWriter(fps=fps))
    plt.close(figure)


def parse_args():
    parser = argparse.ArgumentParser(
        description='Replay an MCL log and render a GIF.')
    parser.add_argument('log', type=Path, help='CSV written by can_bridge_node')
    parser.add_argument(
        '--start', nargs=3, type=float, required=True,
        metavar=('X', 'Y', 'THETA_DEG'),
        help='True starting pose in the map: metres, metres, degrees.')
    parser.add_argument('--output', type=Path, default=Path('mcl_replay.gif'))
    parser.add_argument('--particles', type=int, default=500)
    parser.add_argument('--stride', type=int, default=2,
                        help='Render every Nth step (default 2).')
    parser.add_argument('--fps', type=int, default=10)
    parser.add_argument('--seed', type=int, default=0)
    args = parser.parse_args()
    if args.particles < 1:
        parser.error('--particles must be positive')
    if args.stride < 1:
        parser.error('--stride must be positive')
    return args


def main():
    args = parse_args()
    entries = read_log(args.log)
    start_pose = (args.start[0], args.start[1], math.radians(args.start[2]))

    rect_map = default_map()
    mcl = MCL(rect_map, default_beam_model(), SENSOR_CONFIGS,
              num_particles=args.particles, rng_seed=args.seed)
    frames = replay(entries, mcl, start_pose)

    final = frames[-1]
    x, y, theta = final['estimate']
    dead_x, dead_y, dead_theta = final['dead_reckoning']
    print(f'{len(entries)} rows, {len(frames)} filter steps')
    print(f'final MCL       x={x:.3f} y={y:.3f} '
          f'theta={math.degrees(theta):+.1f}deg')
    print(f'final dead-reck x={dead_x:.3f} y={dead_y:.3f} '
          f'theta={math.degrees(dead_theta):+.1f}deg')
    print(f'separation      {math.hypot(x - dead_x, y - dead_y) * 100:.1f}cm')

    render(frames, rect_map, args.output, fps=args.fps, stride=args.stride)
    print(f'GIF written to {args.output.resolve()}')
```

- [ ] **Step 6: Verify the renderer end to end on synthetic data**

The unit tests cover the frame math; this checks that `replay` and `render` actually produce a file. Generate a short synthetic log and run the CLI over it:

```bash
python -c "
import csv, math
from mcl_log import COLUMNS, build_row
from mcl import default_map, SENSOR_CONFIGS
rect_map = default_map()
with open('synthetic_log.csv', 'w', newline='', encoding='utf-8') as sink:
    writer = csv.writer(sink); writer.writerow(COLUMNS)
    for step in range(60):
        odom = (0.005 * step, 0.0, 0.0)
        true_pose = (0.30 + 0.005 * step, 0.30, 0.0)
        right = rect_map.expected_range(true_pose, *SENSOR_CONFIGS[0])
        back = rect_map.expected_range(true_pose, *SENSOR_CONFIGS[1])
        writer.writerow(build_row(step * 0.125, odom, {
            'right_mm': round(right * 1000), 'back_mm': round(back * 1000),
            'right_valid': True, 'back_valid': True}))
print('wrote synthetic_log.csv')
"
python mcl_offline.py synthetic_log.csv --start 0.30 0.30 0 --output synthetic.gif
```

Expected output, verified while writing this plan:

```
60 rows, 59 filter steps
final MCL       x=0.595 y=0.302 theta=+2.1deg
final dead-reck x=0.595 y=0.300 theta=+0.0deg
separation      0.2cm
```

The odometry in this synthetic log is uncorrupted, so MCL and dead reckoning agreeing to 2mm is the correct result — it confirms the filter is not *introducing* error. Open `synthetic.gif` (30 frames at stride 2) and confirm: the red estimate tracks rightward along a constant y, the green marker sits under it, and the two orange beams run from the robot down to `y=0` and left to `x=0`, each terminating on the wall. The particle cloud collapses under the red dot and is largely hidden — expected with noise-free measurements.

- [ ] **Step 7: Clean up the scratch files and commit**

```bash
rm -f synthetic_log.csv synthetic.gif && git add ros2_ws/src/mcl_offline.py && git commit -m "week6: offline MCL replay with GIF animation"
```

---

### Task 8: Replay the real robot run and update project state

**Files:**
- Modify: `PROJECT_STATE.md`

**Interfaces:**
- Consumes: the CSV from Task 6 Step 4 and the CLI from Task 7.
- Produces: no code. The deliverable is a GIF from real data plus the updated project record.

- [ ] **Step 1: Run the replay over the recorded log**

Copy the CSV off the Pi into `data/`, then, substituting the true starting pose noted during recording:

```bash
python mcl_offline.py ../../data/mcl_run1.csv --start 0.30 0.30 0 --output ../../data/mcl_run1.gif
```

- [ ] **Step 2: Inspect the animation against three checks**

Watch the GIF and confirm all three. Any failure is a finding to record, not something to silence:
1. The particle cloud stays inside the walls and contracts within the first few updates instead of scattering.
2. The red MCL trace and the green dead-reckoning trace start together and separate over time, with the red one staying plausible for where the rover actually drove.
3. The reported final separation is non-zero — if it is near zero the filter is not correcting anything, which points at the measurement update being starved (check how many rows had `right_valid`/`back_valid` set).

- [ ] **Step 3: Tune alpha only if the cloud misbehaves**

If the cloud collapses to a point and then cannot follow the rover, raise the translation alphas (first and fourth entries) via a quick edit to the `MCL(...)` call in `main()`. If it stays diffuse and never sharpens, lower them. Re-run Step 1 after each change. Record the final values and the reasoning; leave the defaults in `mcl.py` alone unless the tuned values are clearly better.

- [ ] **Step 4: Update PROJECT_STATE.md**

Add a Week 6 section to the checklist recording: MCL implemented with standard predict/update/low-variance-resample; the alpha values in use and how they were chosen; and the measured MCL-vs-dead-reckoning separation from the real run.

Add to the known-pitfalls section the limitation established in Task 4. Write it as the exact statement, not the loose one — this is going into the project's permanent record:

> With two beams both referencing walls through the same origin corner, **heading is exactly unobservable, not merely weakly observed**. Each predicted range reduces to `coord/cos(theta) - 0.09`, so the whole family `(x·cos(theta), y·cos(theta), theta)` predicts bit-identical ranges (verified to machine epsilon across 0-30deg). The measurement is rank-2 over a 3-DOF state and this curve is its null direction. Sensor noise is *not* the reason — at 12.6deg a rotation would move each range 14.8mm against a 5.7mm sigma if position were held fixed. The filter absorbs the heading error by sliding position along the degenerate curve at zero likelihood cost.
>
> Downstream consequence, which otherwise looks like a bug: an uncorrected heading error `theta` puts the estimate about `1 - cos(theta)` **closer** to each measured wall — 12.6deg gave 1.44cm per axis, ~2cm combined, in the synthetic test. Note the direction: closer, not farther.
>
> The fix is a third sensor or the Week 11 IMU, **not** a parameter to tune — and the third sensor must face an **opposing** wall, since `(x_max - x)` does not rescale the way `x` does and so breaks the degeneracy. A beam parallel to an existing one adds nothing.

Also record the process-noise floor (`SIGMA_TRANS_FLOOR` / `SIGMA_ROT_FLOOR` in `mcl.py`) and why it exists: without it a parked rover injects zero process noise while `update()` keeps resampling, collapsing 500 particles to 1 within ~1.5s. Every recording opens with idle rows, so this is not a corner case.

Add a decision-log row for the holonomic motion model: noise applied to body-frame dx/dy/dtheta rather than the textbook rot1/trans/rot2 decomposition, because that decomposition assumes a differential drive that must turn in order to translate, which this chassis does not.

- [ ] **Step 5: Commit**

```bash
git add PROJECT_STATE.md data/mcl_run1.csv data/mcl_run1.gif && git commit -m "week6: MCL validated on recorded run; update project state"
```

---

## Out of Scope

Deliberately deferred; do not implement:

- `mcl_node.py` online ROS2 integration — comes after offline replay is trusted
- Global localization (uniform particle initialization / kidnapped robot)
- KLD-sampling or any adaptive particle count
- Conditional resampling on an N-effective threshold — a one-line addition if particle depletion shows up, but not before
- Non-rectangular maps, IMU fusion
