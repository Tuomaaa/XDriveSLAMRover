# Week 6 — Monte Carlo Localization

:::{admonition} Chapter status: In progress
:class: status status-progress

The particle filter, convergence test, CSV log contract, and offline replay
tool are implemented and pass all tests on the `week6-mcl` branch. The
implementation and synthetic replay evidence must be merged and reverified
before this chapter can claim completion.
:::

:::{admonition} File guide
:class: tip

**Core algorithm — read this first**

- [`mcl.py`](https://github.com/Tuomaaa/XDriveSLAMRover/blob/week6-mcl/ros2_ws/src/mcl.py) — particle filter: predict (body-frame motion model), update (measurement scoring), low-variance resample, weighted circular-mean estimate

**Recommended — Week 5 dependencies**

- [`measurement_model.py`](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/measurement_model.py) — `BeamModel` that scores each particle
- [`map_model.py`](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/map_model.py) — `RectMap` that computes expected ranges

**Infrastructure — safe to skip**

- `mcl_log.py` — CSV column contract between logger and replay
- `mcl_offline.py` — replay loop and GIF rendering

**Tests — safe to skip**

- `mcl_test.py` — synthetic convergence test (L-path) and heading tracking test (arc)
:::

## From range scoring to pose estimation

Week 5 built a log-likelihood function that scores a candidate pose against
measured ranges and a rectangular map. That function answers one question:
"given these two sensor readings, how plausible is this pose?" It does not
answer the question a moving robot needs: "where am I?"

Monte Carlo Localization (MCL) maintains a cloud of weighted guesses — called
particles — and uses the likelihood function to prune bad guesses each cycle.
The surviving particles concentrate around the true pose over time, even as
encoder drift pushes dead reckoning away from reality.

This chapter implements the standard MCL algorithm from Probabilistic Robotics
Chapter 8, with one structural change: the motion model operates in the
body frame instead of the textbook's rot1/trans/rot2 decomposition. It then
validates the filter against dead reckoning in a synthetic convergence test.

## Prerequisites

- Body-frame odometry increments from Week 3/4
- The calibrated multi-sensor likelihood from Week 5
  (`BeamModel` and `RectMap`)
- A rectangular map and measured sensor extrinsics
- A deterministic random seed for replayable tests

## How to read the repository

| Order | File | What to read |
| ---: | --- | --- |
| 1 | [`mcl.py` constants and defaults](https://github.com/Tuomaaa/XDriveSLAMRover/blob/week6-mcl/ros2_ws/src/mcl.py#L38) | The Week 5 calibration values carried forward: `SIGMA_HIT`, `BEAM_WEIGHTS`, `SENSOR_CONFIGS`, `MAP_BOUNDS`. Read these before the class. |
| 2 | [`MCL.__init__` and `initialize`](https://github.com/Tuomaaa/XDriveSLAMRover/blob/week6-mcl/ros2_ws/src/mcl.py#L87) | Constructor parameters, particle array layout, and the Gaussian scatter. |
| 3 | [`MCL.predict`](https://github.com/Tuomaaa/XDriveSLAMRover/blob/week6-mcl/ros2_ws/src/mcl.py#L117) | Body-frame motion model with noise floor. Compare this to Probabilistic Robotics Table 5.6 and note the differences. |
| 4 | [`MCL.update`](https://github.com/Tuomaaa/XDriveSLAMRover/blob/week6-mcl/ros2_ws/src/mcl.py#L155) | Measurement update: score each particle with the beam model, then resample. |
| 5 | [`MCL._low_variance_resample`](https://github.com/Tuomaaa/XDriveSLAMRover/blob/week6-mcl/ros2_ws/src/mcl.py#L193) | Probabilistic Robotics Table 4.4 implemented in numpy. |
| 6 | [`mcl_test.py`](https://github.com/Tuomaaa/XDriveSLAMRover/blob/week6-mcl/ros2_ws/src/mcl_test.py) | Convergence test: simulated L-path with injected yaw drift, MCL vs dead reckoning. |
| 7 | [`mcl_log.py`](https://github.com/Tuomaaa/XDriveSLAMRover/blob/week6-mcl/ros2_ws/src/mcl_log.py) | The shared CSV contract between the ROS node logger and the offline replay. |
| 8 | [`mcl_offline.py`](https://github.com/Tuomaaa/XDriveSLAMRover/blob/week6-mcl/ros2_ws/src/mcl_offline.py) | Replay a recorded CSV through the filter and render a GIF. |

## Particle state and initialization

The MCL class holds two arrays. `particles` is an $(N, 3)$ numpy array where
each row stores $(x, y, \theta)$. `weights` is an $(N,)$ array that sums to
1.0.

```python
# mcl.py — MCL constructor (abbreviated)
self.particles = np.zeros((self.num_particles, 3))
self.weights = np.full(self.num_particles, 1.0 / self.num_particles)
```

`initialize()` scatters particles as a Gaussian cloud around a known starting
pose. Global localization (uniform scatter over the entire map) is not
implemented. The rover starts at a measured position, so a tight cloud is
appropriate.

```python
# mcl.py — initialize()
self.particles[:, 0] = self._rng.normal(x, xy_spread, count)
self.particles[:, 1] = self._rng.normal(y, xy_spread, count)
self.particles[:, 2] = _wrap_angle(
    self._rng.normal(theta, theta_spread, count))
self._clamp_to_map()
self.weights = np.full(count, 1.0 / count)
```

`_clamp_to_map()` clips every particle to the map rectangle. A particle
outside the walls has no meaningful expected range and lingers on the
`w_rand` floor indefinitely. Clamping keeps every particle scoreable.

### The weighted pose estimate

`estimate()` computes the weighted mean of all particles. The $\theta$
component uses a circular mean:

$$\hat{\theta} = \text{atan2}\!\left(\sum_i w_i \sin\theta_i,\;
\sum_i w_i \cos\theta_i\right)$$

A scalar mean of headings near $\pm\pi$ averages to zero. Two
particles at $+179°$ and $-179°$ must produce an estimate near $180°$.
The circular mean handles this correctly.

```python
# mcl.py — estimate()
sin_sum = float(np.sum(self.weights * np.sin(self.particles[:, 2])))
cos_sum = float(np.sum(self.weights * np.cos(self.particles[:, 2])))
return x, y, math.atan2(sin_sum, cos_sum)
```

After resampling, all weights are uniform ($1/N$). The weighted mean then
becomes an unweighted mean. It also assumes the cloud is unimodal. A cloud
with two separated clusters produces a pose between them.

## Holonomic body-frame motion update

### Why the textbook model does not fit

Probabilistic Robotics Table 5.6 splits each odometry step into three
phases: an initial rotation (rot1), a translation, and a final rotation
(rot2). This decomposition assumes that the robot must turn to face its
direction of travel. An X-drive rover can strafe sideways with zero heading
change. A pure lateral move has zero translation in the rot1/trans/rot2
sense, so the model injects zero noise — a wrong result for a step that
moved 200 mm.

SummerSLAM replaces rot1/trans/rot2 with direct body-frame perturbation.
The filter receives $(dx, dy, d\theta)$ in the robot's own frame and adds
Gaussian noise to each component.

### The predict method

Each call to `predict(dx_body, dy_body, dtheta)` does four things for
every particle:

1. Compute noise standard deviations from the step size.
2. Add Gaussian noise to the body-frame delta.
3. Rotate the noisy delta into the world frame using that particle's heading.
4. Clamp the result inside the map.

```python
# mcl.py — predict() (core)
distance = math.hypot(dx_body, dy_body)
turned = abs(dtheta)
sigma_trans = max(a_trans_trans * distance + a_trans_rot * turned,
                  SIGMA_TRANS_FLOOR)
sigma_rot = max(a_rot_trans * distance + a_rot_rot * turned,
                SIGMA_ROT_FLOOR)

noisy_dx = dx_body + self._rng.normal(0.0, sigma_trans, count)
noisy_dy = dy_body + self._rng.normal(0.0, sigma_trans, count)
noisy_dtheta = dtheta + self._rng.normal(0.0, sigma_rot, count)

heading = self.particles[:, 2].copy()
cos_h = np.cos(heading)
sin_h = np.sin(heading)

self.particles[:, 0] += noisy_dx * cos_h - noisy_dy * sin_h
self.particles[:, 1] += noisy_dx * sin_h + noisy_dy * cos_h
self.particles[:, 2] = _wrap_angle(heading + noisy_dtheta)
```

Step 3 is the critical line. Each particle rotates the noisy delta by *its
own* heading. Particles that disagree about which way they face end up in
different places. The measurement update then prunes the ones that land in
poses inconsistent with the sensor readings.

### The alpha parameters

The four alpha values control how much noise each step injects:

| Alpha | Meaning |
| --- | --- |
| `a_trans_trans` | Translation noise from translation distance |
| `a_trans_rot` | Translation noise from rotation angle |
| `a_rot_trans` | Rotation noise from translation distance |
| `a_rot_rot` | Rotation noise from rotation angle |

The alphas keep the same *role* as Probabilistic Robotics Table 5.6. The
*numbers* are different. The textbook multiplies squared deltas to produce a
variance. This implementation multiplies un-squared deltas to produce a
standard deviation directly. Do not carry Table 5.6 values over.

Default values are `(0.05, 0.01, 0.01, 0.05)`. The convergence test uses
`(0.25, 0.05, 0.05, 0.15)` because its simulated odometry is more corrupt
than the real chassis.

### The noise floor

A strictly proportional model injects zero noise when the rover is parked.
The measurement update resamples every cycle regardless of motion. With no
noise to replenish diversity, 500 particles collapse to a single distinct
particle within about 1.5 seconds of standstill.

Two constants set the floor:

```python
SIGMA_TRANS_FLOOR = 0.001   # metres per step
SIGMA_ROT_FLOOR = 0.010     # radians per step
```

The rotational floor is the more important of the two. Heading is the
weakly-observed dimension (explained in the final section). Resampling
impoverishes it first. Calibrated 2026-07-22 against a 60-second synthetic
wander:

- At `SIGMA_ROT_FLOOR = 0.002`: heading spread collapsed to 0.4° within
  7 seconds. Heading error grew to 28.3° by 52 seconds. Position RMS
  reached 10.2 cm.
- At `SIGMA_ROT_FLOOR = 0.010`: heading spread held near 1°. Heading error
  stayed within 2°. Position RMS dropped to 1.0 cm.

The value 0.010 sits mid-plateau. Behaviour is flat from 0.005 to 0.020
and degrades again by 0.040.

## Measurement update in log space

`update()` reweights every particle against the current sensor readings,
then resamples. The flow has four steps.

### 1. Score each particle

For each particle, the beam model computes a log-likelihood that the
measured ranges came from that pose:

```python
# mcl.py — update() scoring loop
for index in range(self.num_particles):
    pose = (float(self.particles[index, 0]),
            float(self.particles[index, 1]),
            float(self.particles[index, 2]))
    log_weights[index] = self.beam_model.total_log_likelihood(
        measurements, pose, self.rect_map, self.sensor_configs)
```

This is the bridge to Week 5. `total_log_likelihood` computes the expected
range from each sensor at the candidate pose, scores the actual reading
against it using the calibrated beam model, and sums the log-likelihoods.

### 2. Handle None sensors

A sensor with no valid echo this cycle passes as `None`. If both sensors
are `None`, the update is a no-op. With nothing observed, resampling on
uniform weights destroys diversity for no gain.

### 3. Exponentiate safely

The raw log-likelihoods run to several hundred negative. A direct `exp()`
underflows to all zeros. The standard fix is to subtract the maximum
before exponentiating:

```python
shifted = log_weights - np.max(log_weights[finite])
weights = np.exp(shifted)
self.weights = weights / float(np.sum(weights))
```

The highest-scoring particle gets weight `exp(0) = 1`. All others get
values relative to that maximum. The absolute magnitude cancels in
normalization.

### 4. Resample

After normalization, `_low_variance_resample()` rebuilds the particle set.

## Low-variance resampling

The resampler implements Probabilistic Robotics Table 4.4. It draws one
random offset $r \in [0, 1/N)$, then walks through the cumulative weight
distribution in steps of $1/N$.

```python
# mcl.py — _low_variance_resample()
positions = (self._rng.uniform(0.0, 1.0 / count)
             + np.arange(count) / count)

cumulative = np.cumsum(self.weights)
cumulative[-1] = 1.0
indexes = np.searchsorted(cumulative, positions)
np.clip(indexes, 0, count - 1, out=indexes)

self.particles = self.particles[indexes]
self.weights = np.full(count, 1.0 / count)
```

A particle of weight $w$ receives either $\lfloor wN \rfloor$ or
$\lceil wN \rceil$ copies. Multinomial resampling gives a binomial spread
around that number instead. Low-variance resampling is $O(N)$ and
preserves particle diversity better.

After resampling, all weights reset to $1/N$. The cloud's information now
lives in particle density: more copies of a pose means more weight.

## CSV logging and offline replay

### The shared log format

`mcl_log.py` defines the CSV contract that the ROS node writes and the
offline replay reads. One row per ultrasonic frame (~8 Hz), carrying the
latest odometry pose. One row is one MCL predict+update cycle.

```
timestamp_s, odom_x, odom_y, odom_theta, right_mm, back_mm, right_valid, back_valid
```

Ranges are in millimetres in the CSV to match the CAN payload.
`read_log()` converts to metres for the filter. A sensor whose validity bit
is clear becomes `None` so the filter skips it instead of scoring against the
0xFFFF timeout sentinel.

The module uses only the standard library. The ROS node imports it on the
Pi without pulling in numpy or matplotlib.

### Body-frame delta computation

The log stores world-frame odometry poses. The filter consumes body-frame
deltas. `body_frame_delta()` converts between the two:

```python
# mcl_offline.py — body_frame_delta()
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
```

Rotating the world displacement by $-\theta_{\text{prev}}$ undoes the heading
it was expressed in. The result is the step the robot felt in its own frame.

### Replay loop

`replay()` iterates through the log. For each row after the first (which
establishes the odometry origin), it computes the body-frame delta, calls
`predict()`, then calls `update()` with the range readings. Each step
records a frame for the animation.

### GIF animation

`render()` produces a GIF with matplotlib:

- Gray rectangle: map boundary
- Blue dots: particles (the cloud)
- Red dot and arrow: MCL estimated pose and heading
- Green dot: dead-reckoning pose
- Orange lines: sensor beams from the estimate to the measured range

A beam that stops short of the wall means the estimate is too far from
that wall. A beam that overshoots means the estimate is too close.

## Convergence test: MCL vs dead reckoning

`mcl_test.py` runs two hardware-free tests that prove the filter works.

### The L-path test

A simulated rover drives an L-shaped path inside the KT-board map: 300 mm
forward (+x), then 300 mm left (+y), at 5 mm per cycle. The odometry handed
to the filter is corrupted with 3% scale error and a steady yaw bias of
0.002 rad per step. The range readings carry Week 5 calibrated noise.

Results with a fixed random seed:

```
true       x=0.600 y=0.600
dead-reck  x=0.539 y=0.603  error=6.2cm
mcl        x=0.580 y=0.588  error=2.3cm  theta=+12.6deg
```

MCL error is 2.3 cm. Dead-reckoning error is 6.2 cm. The filter beats
raw integration by a factor of 2.7.

### Why MCL error is 2.3 cm instead of zero

The filter ends up at $\theta = +12.6°$ when the truth is $0°$. That is
the injected yaw bias (0.002 rad × 120 steps = 13.8°) passing through
almost uncorrected. The reason is heading observability, explained below.

A wrong heading biases position. The filter reads each range as
$d/\cos(\theta)$ and places itself a fraction short of each wall. At
12.6° of heading error and these distances, that fraction is about
1.2 cm per axis — consistent with the 2.3 cm observed.

### The arc test

A second test drives the rover along a half-circle arc to exercise heading
under sustained rotation. Final heading error stays within 10°. Position
error stays within 3 cm.

## Limits of heading observability

This section explains the filter's sharpest limitation. Position (x, y) is
directly observable through range measurements. Heading ($\theta$) is not.

Two orthogonal beams in a rectangular room measure distances to walls. A
small heading rotation $\theta$ changes an expected range only as
$1/\cos(\theta)$, a second-order effect. At 5° of heading error, the
expected range changes by 0.4%. The HC-SR04's ~3 mm noise swamps that
signal.

The consequence: the filter can correct x and y drift within a centimetre.
It cannot correct heading drift. An uncorrected heading error of $\theta$
makes the filter place itself $d(1 - \cos\theta)$ away from the true wall
distance. At 12.6° and typical distances, that is about 1.2 cm.

The fix is a third sensor at a different angle, or the Week 11 IMU. A
parameter change cannot create heading information that the sensor geometry
does not provide.

### The noise floor keeps the problem bounded

Without the rotational noise floor, the cloud commits to a heading within
seconds and the error grows without bound. The floor (`SIGMA_ROT_FLOOR =
0.010 rad/step`) maintains enough heading diversity that the filter can
slowly track heading changes through the second-order range signal. It
cannot eliminate heading error, but it prevents runaway drift.

(mcl-design-corrections)=
## Design corrections

### Holonomic motion model

The standard rot1/trans/rot2 decomposition (Probabilistic Robotics Table
5.6) assumes a platform that turns to translate. This chassis is holonomic.
SummerSLAM perturbs body-frame $dx$, $dy$, and $d\theta$ directly so that
lateral X-drive motion remains representable.

### Alpha parameter scaling

The textbook alphas multiply *squared* deltas to produce a variance. This
implementation multiplies *un-squared* deltas to produce a standard
deviation. The numerical values are not comparable.

### Noise floor necessity

The textbook does not discuss a noise floor because differential-drive
robots rarely park during a filter cycle. An X-drive rover with two
ultrasonic sensors at 8 Hz has many cycles of near-zero motion. Without
a floor the cloud collapses.

## Source and evidence

- [`mcl.py` on `week6-mcl`](https://github.com/Tuomaaa/XDriveSLAMRover/blob/week6-mcl/ros2_ws/src/mcl.py) — particle filter core
- [`mcl_test.py` on `week6-mcl`](https://github.com/Tuomaaa/XDriveSLAMRover/blob/week6-mcl/ros2_ws/src/mcl_test.py) — convergence and heading tracking tests
- [`mcl_log.py` on `week6-mcl`](https://github.com/Tuomaaa/XDriveSLAMRover/blob/week6-mcl/ros2_ws/src/mcl_log.py) — shared CSV contract
- [`mcl_offline.py` on `week6-mcl`](https://github.com/Tuomaaa/XDriveSLAMRover/blob/week6-mcl/ros2_ws/src/mcl_offline.py) — replay and GIF rendering
- [MCL design specification](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/docs/superpowers/specs/2026-07-22-mcl-particle-filter-design.md) — the original design document
- [MCL implementation plan](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/docs/superpowers/plans/2026-07-22-mcl-particle-filter.md) — task-by-task build plan with expected outputs

## Verification state

| Item | Status |
| --- | --- |
| Unit tests (`mcl.py _run_tests`) | Pass on `week6-mcl`, 10 tests |
| L-path convergence (`mcl_test.py`) | Pass: 2.3 cm MCL vs 6.2 cm dead reckoning |
| Arc heading tracking (`mcl_test.py`) | Pass: heading error < 10°, position error < 3 cm |
| Noise floor calibration | Complete: `SIGMA_ROT_FLOOR = 0.010` verified against synthetic wander |
| Offline replay tests (`mcl_offline.py`) | Pass: body-frame delta and odom-to-map transforms verified |
| CSV log contract (`mcl_log.py`) | Pass: round-trip read/write with valid and invalid sensors |
| Real robot log replay | Not yet captured. Requires Task 6 (CAN bridge logging) and Task 8 (recorded run). |
| Online ROS2 node (`mcl_node.py`) | Deferred until offline replay is validated against real data. |
| Merge to `main` | Pending. |
