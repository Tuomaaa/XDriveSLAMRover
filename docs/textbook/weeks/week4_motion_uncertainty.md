# Week 4 — Ground-Truth Calibration and Motion Uncertainty

:::{admonition} Chapter status: first draft, with partial evidence
:class: status status-progress

We identified, corrected, and verified the missing X-drive translation scale
and the REP-103 frame mirror. The early trial data that motivated those
corrections is preserved. A longer repeatability campaign on the 1 m by 1 m
foam surface is not yet complete. This chapter does not claim a final
translation or heading noise distribution.
:::

:::{admonition} File guide
:class: tip

**Core — read this first**

- [`odometry.py`](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/odometry.py) — `TRANSLATION_SCALE`, `ENCODER_CPR`, forward kinematics with $\sqrt{2}$ projection and `vy`/`omega` negation

**Recommended**

- [`Week4 Trials.xlsx`](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/data/Week4%20Trials.xlsx) — raw ground-truth measurements (forward, strafe, rotation on foam)
- [`PS2_Drive_Test.py`](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/PS2_Drive_Test.py) — D-pad crawl mode for precise positioning during trials
:::

## From kinematics to measured motion

Week 3 produced an encoder odometry estimator that outputs a pose. Week 4
asks how closely that pose matches the physical motion of the rover.

Kinematics and calibration can make the estimator internally consistent. They
do not prove that a 0.5 m forward command moves the rover 0.5 m. That proof
requires a tape measure, a marked start and end point, and a procedure that
separates systematic model errors from random noise.

This chapter records the early ground-truth trials that uncovered two
systematic errors: a missing translation projection and a coordinate-frame
mirror. It also records the observation that the rover yaws during straight
motion. The corrections are in the code. The uncertainty characterization is
not yet complete.

## How to read the repository

| Order | File | What to read |
| ---: | --- | --- |
| 1 | [odometry.py configuration](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/odometry.py#L27) | `TRANSLATION_SCALE`, `ENCODER_CPR`, `MOTOR_MAP`, and `ENCODER_SIGN` as a group. These are the outputs of Week 3 and Week 4 calibration. |
| 2 | [Week4 Trials.xlsx](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/data/Week4%20Trials.xlsx) | Raw trial log with forward, strafe, and rotation measurements on foam. |
| 3 | [PS2_Drive_Test.py D-pad controls](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/PS2_Drive_Test.py#L106) | The fine-adjust crawl mode for ground-truth positioning. |
| 4 | [odometry.py forward kinematics](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/odometry.py#L160) | Where `TRANSLATION_SCALE` and the `vy`/`omega` negation apply. |

## The calibration constants

All four calibration outputs live at the top of
[`odometry.py`](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/odometry.py#L27-L87).
Read them as a group:

```python
# odometry.py L28-L41
WHEEL_RADIUS_M = 0.025       # 50mm diameter / 2
CENTER_TO_WHEEL_M = 0.115    # R: center to wheel contact point
TRANSLATION_SCALE = math.sqrt(2)
ENCODER_CPR = 2779           # CALIBRATED 2026-07-04: 4-wheel avg of 10-rev hand turns
```

The encoder harness map swaps the rear indices. Index 2 reads the RR wheel
and index 3 reads the RL wheel. This is the opposite of the firmware motor
order
([`odometry.py` L62–L67](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/odometry.py#L62-L67)):

```python
MOTOR_MAP = {
    0: "fl",
    1: "fr",
    2: "rr",   # index 2 encoder is physically on the RR wheel
    3: "rl",   # index 3 encoder is physically on the RL wheel
}
```

All four encoder signs are $-1$. A forward drive makes all four raw tick
counts decrease
([`odometry.py` L82–L87](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/odometry.py#L82-L87)):

```python
ENCODER_SIGN = {
    0: -1,
    1: -1,
    2: -1,
    3: -1,
}
```

## Measurement principle

A straight trial often curves because of unequal motor response. If you
compare odometry to a tape measure along the commanded axis, the result
mixes translation error with heading error. I instead compare net
displacement:

$$
s = \frac{\sqrt{x_{actual}^2 + y_{actual}^2}}
         {\sqrt{x_{odom}^2 + y_{odom}^2}}.
$$

This ratio isolates translation scale from path shape. If the rover curves
during a forward trial, $s$ is still meaningful. Both numerator and
denominator measure the same start-to-end straight line.

Heading trials used approximate visual 180-degree and 360-degree references.
They support only a coarse angular scale verification.

## Test setup

### Surface

The X-drive uses omni wheels. On smooth hard floor, the driven rollers slip
and the translation error is too large for calibration. On a foam exercise
mat the rollers grip better, and the errors are small enough for calibration.

I first used a 0.6 m by 0.6 m foam piece. Its short dimension limited the
trial distance. As a result, measurement noise was large relative to the
motion. I later switched to a 1 m by 1 m foam mat. This mat allowed trials of
0.6 to 0.7 m and reduced the relative measurement uncertainty.

### Speed

Trials used medium joystick input:

- Too fast: aggressive acceleration and deceleration cause slip.
- Too slow: weak motors can fail to overcome static friction.
- Medium speed with consistent joystick position keeps each trial comparable.

### D-pad fine adjustment

The PS2 joystick is too coarse for placing the rover on a floor mark. I added
a D-pad crawl mode at
[`PS2_Drive_Test.py` L106–L118](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/PS2_Drive_Test.py#L106-L118)
so the rover can move slowly onto a mark before a trial starts:

```python
# PS2_Drive_Test.py L110-L118
micro = (micro_duty / 100.0)
if ps2.is_pressed(data['btn1'], BTN_UP):
    vx += micro
if ps2.is_pressed(data['btn1'], BTN_DOWN):
    vx -= micro
if ps2.is_pressed(data['btn1'], BTN_LEFT):
    vy += micro
if ps2.is_pressed(data['btn1'], BTN_RIGHT):
    vy -= micro
```

The D-pad adds a small fixed crawl on top of the joystick input. With the
sticks centered, this gives pure slow-speed motion. The body-frame convention
matches the joystick: up is forward, left is left.

The crawl duty is adjustable at runtime with Triangle (+5%) and Cross (-5%)
([`PS2_Drive_Test.py` L97–L104](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/PS2_Drive_Test.py#L97-L104)):

```python
# PS2_Drive_Test.py L97-L104
tri = ps2.is_pressed(data['btn2'], BTN_TRIANGLE)
crs = ps2.is_pressed(data['btn2'], BTN_CROSS)
if tri and not prev_triangle:
    micro_duty = min(micro_duty + 5, 100)
if crs and not prev_cross:
    micro_duty = max(micro_duty - 5, 10)
```

## What the early trials showed

The first foam trials are stored in
[`data/Week4 Trials.xlsx`](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/data/Week4%20Trials.xlsx).
Three patterns were clear:

| Motion | Odometry vs tape | What this tells |
| --- | --- | --- |
| Forward | ~0.74 times the measured distance | Translation scale is wrong |
| Sideways | ~0.74 times the measured distance | Same scale error as forward |
| In-place rotation | ~1.0 times the expected angle | Rotation scale is correct |

Both translation axes share the same error, but rotation is not affected.
This pattern points to a specific missing factor.

(translation-scale-failure)=
## Failure: the missing 45-degree projection

### Symptom

Forward and strafe displacement were both about 0.74 times the tape-measured
distance. In-place rotation remained near the expected scale.

### Diagnosis

The X-drive wheels sit at 45 degrees to the body axes. When a wheel drives
pure $+v_x$, only $\cos(45^\circ) = 1/\sqrt{2} \approx 0.707$ of its surface
speed contributes to $v_x$.

The drive-side inverse kinematics in `PS2_Drive_Test.py` uses the
unnormalized form. Each wheel coefficient is 1 instead of $1/\sqrt{2}$
([`PS2_Drive_Test.py` L120–L124](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/PS2_Drive_Test.py#L120-L124)):

```python
# PS2_Drive_Test.py L120-L124 — inverse kinematics (drive side)
m0 = (vx + vy + omega) * MAX_RPM   # FL
m1 = (vx - vy - omega) * MAX_RPM   # FR
m2 = (vx - vy + omega) * MAX_RPM   # RL
m3 = (vx + vy - omega) * MAX_RPM   # RR
```

The forward kinematics that inverts these equations inherits the same missing
factor. The measured ratio of 0.74 is close to $1/\sqrt{2} = 0.707$. The
remaining difference of about 5 percent is consistent with foam slip, CPR
rounding, and measurement noise.

Rotation is not affected because the angular term uses $\omega R$. This
term does not carry the 45-degree projection.

### Correction

Multiply only the two translation terms by $\sqrt{2}$:

$$
v_x = \sqrt{2}\,\tilde{v}_x,
\qquad
v_y = \sqrt{2}\,\tilde{v}_y.
$$

The angular velocity $\omega$ stays the same. The corrected forward
kinematics in
[`odometry.py` L160–L162](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/odometry.py#L160-L162):

```python
# odometry.py L160-L162
vx = (fl + fr + rl + rr) / 4.0 * TRANSLATION_SCALE
vy = -(fl - fr + rr - rl) / 4.0 * TRANSLATION_SCALE
omega = -(fl - fr - rr + rl) / (4.0 * CENTER_TO_WHEEL_M)
```

This correction belongs to the conventions of this rover. Another X-drive
that uses normalized inverse kinematics does not need this factor. Do not copy the
factor without deriving the wheel equations for the target system.

(frame-mirror-failure)=
## Failure: the coordinate frame was mirrored

### Symptom

After the rear encoder map correction in
{ref}`Week 3 <encoder-map-failure>`, physical left strafe produced negative
$v_y$ and physical counter-clockwise rotation produced negative $\omega$.
Forward $v_x$ was already correct.

$v_x$ is correct, but $v_y$ and $\omega$ are both reversed. This is a
left-right frame mirror.

### Diagnosis

The drive-side inverse kinematics convention has $+v_y$ to the right and
$+\omega$ clockwise (visible in the `PS2_Drive_Test.py` IK above). The
forward kinematics that inverts those equations produces the same convention.
Encoder signs are fully constrained by the forward-motion test. The motor
map is constrained by the physical harness. Neither can absorb this frame
difference.

### Correction

Negate $v_y$ and $\omega$ at the forward-kinematics output to align with
REP-103, where $+y$ is left and positive $\theta$ is counter-clockwise.
Leave $v_x$ the same:

$$
v_y = -\tilde{v}_y,
\qquad
\omega = -\tilde{\omega}.
$$

The negation is on the `vy` and `omega` lines at L161–L162 (the leading
minus signs). `vx` at L160 has no negation. This changes the output
coordinate convention.

## Midpoint pose integration

After the forward kinematics compute `vx`, `vy`, and `omega`, the estimator
integrates them into the world-frame pose with the midpoint method
([`odometry.py` L164–L177](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/odometry.py#L164-L177)):

```python
# odometry.py L164-L177
theta_mid = self.theta + omega * dt / 2.0

dx = (vx * math.cos(theta_mid) - vy * math.sin(theta_mid)) * dt
dy = (vx * math.sin(theta_mid) + vy * math.cos(theta_mid)) * dt

self.x += dx
self.y += dy
self.theta += omega * dt
# Keep theta in (-pi, pi] for sane logging/plotting.
self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
```

The midpoint method evaluates heading at $\theta + \omega\,dt/2$ instead of
$\theta$. This reduces the heading error that dominates accumulated drift.

## Hardware yaw

During straight forward trials, the heading $\theta$ drifted consistently to
about $-7°$ to $-11°$. The rover was also visibly turning.

This is not an odometry bug. The rover yaws because of unequal motor response
in open-loop mode. Different motors produce slightly different torque at the
same PWM duty. The resulting imbalance turns the rover.

The odometry records this physical yaw correctly. The effect is part of the
motion uncertainty that a localization filter must handle. It is not an error
to correct.

To identify the weaker motor, use `encoder_monitor.py`. During a pure forward
command, the channel with fewer ticks per interval is the weaker side. This
test is not yet complete.

## Smoke test

The smoke test at
[`odometry.py` L185–L209](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/odometry.py#L185-L209)
verifies that both corrections (translation scale and frame mirror) are
internally consistent:

```python
# odometry.py L199-L208 — build raw ticks from the expected wheel pattern
STEP = 280
wheel_target = {"fl": -STEP, "fr": +STEP, "rl": +STEP, "rr": -STEP}
raw = {i: ENCODER_SIGN[i] * wheel_target[MOTOR_MAP[i]] for i in MOTOR_MAP}

odo = OdometryEstimator()
odo.update({i: 0 for i in MOTOR_MAP}, timestamp=0.0)
result = odo.update(raw, timestamp=0.02)
x, y, theta, vx, vy, omega = result
assert abs(vx) < 1e-9 and abs(omega) < 1e-9 and vy > 0, \
    "left-strafe sanity check failed"
```

The test builds a physical-left-strafe tick pattern through `ENCODER_SIGN`
and `MOTOR_MAP`, so it stays correct if those constants change. The assertion
verifies three properties of a left strafe: $v_x = 0$, $\omega = 0$, and
$v_y > 0$ (REP-103).

## Test one degree of freedom at a time

The debugging order from Week 3 also serves as the ground-truth validation
order:

1. Forward: verify the $v_x$ scale.
2. Sideways: verify the $v_y$ scale and make sure that no signal leaks into
   $\omega$.
3. Rotation: verify the angular scale at order-of-magnitude level.
4. Combined: run longer paths and measure the accumulated drift.

Each motion isolates a different class of error. A forward trial cannot detect
a frame mirror. A rotation trial cannot detect a translation scale error.
Single trials that combine all degrees of freedom are poor diagnostic tools.

## What Week 4 established

| Result | Evidence |
| --- | --- |
| Translation scale $\sqrt{2}$ | Foam trials: forward and strafe both ~0.74 times tape, rotation ~1.0 |
| REP-103 frame convention | Left strafe and CCW rotation sign verified after negation |
| Hardware yaw exists | Forward trials drift $-7°$ to $-11°$, with visual verification |
| Smooth floor is unusable | X-drive omni wheel slip too large on smooth surfaces |
| Foam mat is adequate | 1 m by 1 m mat supports 0.6 to 0.7 m trials |

The following items are not yet complete:

- More trials on the 1 m foam mat with the $\sqrt{2}$ correction applied, to
  verify that the corrected scale is close to 1.0.
- Translation and heading variance estimates from repeated trials.
- A formal motion-noise model for the particle-filter motion update.
- Weak-motor identification with per-channel encoder rates.

## Reproduction procedure

1. Verify the Week 3 calibration: map, sign, CPR, frame convention.
2. Choose a test surface where omni wheels grip. Push the rover sideways. If
   it slides easily, the surface is too smooth.
3. Mark a start position and a measured endpoint on the surface.
4. Use the D-pad to drive the rover onto the start mark.
5. Command a single-axis motion at moderate speed.
6. Record the odometry pose at the end and the tape-measured displacement.
7. Repeat each axis at least five times before you draw conclusions.
8. Compare the net displacement ratio. This prevents translation and heading
   errors from mixing.
9. Verify the rotation at order-of-magnitude level with visual 180-degree or
   360-degree references.
10. Look for consistent heading drift during straight trials. This drift shows
    motor imbalance.
11. Run `python ros2_ws/src/odometry.py` to verify that the smoke test passes.

## Source index

- [Odometry configuration and forward kinematics](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/odometry.py)
- [Early ground-truth trial data](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/data/Week4%20Trials.xlsx)
- [PS2 D-pad fine-adjust and inverse kinematics](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/PS2_Drive_Test.py)
- [Project decisions and validation state](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/PROJECT_STATE.md)
