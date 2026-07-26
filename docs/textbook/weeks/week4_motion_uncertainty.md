# Week 4 - Ground-Truth Calibration and Motion Uncertainty

:::{admonition} Chapter status: Outline
:class: status status-outline

The missing X-drive translation scale was identified and corrected. The larger
repeatability experiment needed to estimate a robust motion-noise distribution
is still pending.
:::

## Objective

Compare encoder odometry with physical ground truth, separate systematic model
errors from stochastic motion errors, and estimate uncertainty suitable for a
holonomic particle-filter motion update.

## Prerequisites

- Week 3 odometry with fixed mapping, signs, and frame convention
- A measured test surface and repeatable command procedure
- Synchronized final odometry pose and physical end-point measurements

## Measurement principle

Translation scale is compared using net displacement rather than assumed path
length:

$$
s = \frac{\sqrt{x_{actual}^2 + y_{actual}^2}}
         {\sqrt{x_{odom}^2 + y_{odom}^2}}.
$$

This remains useful when motor imbalance bends a nominally straight trial. The
heading trials use approximate visual 180- and 360-degree ground truth, so they
support only a coarse angular scale check.

## Planned chapter sections

- Ground-truth apparatus and measurement uncertainty
- Forward, strafe, rotation, and square-path experiments
- The missing $\sqrt{2}$ translation projection
- Bias, variance, and repeatability metrics
- Holonomic body-frame motion-noise model
- Limits imposed by open-loop motor mismatch and surface friction

(translation-scale-failure)=
## Failure record: translation was consistently too small

Forward and strafe displacement were both about 0.74 times the tape-measured
distance while in-place rotation remained near the expected scale. Because the
two translation axes shared the same error and rotation did not, the evidence
pointed to the omitted $\cos(45^\circ)$ wheel projection rather than CPR or
center-to-wheel radius. Multiplying translation by $\sqrt{2}$ corrected the
model without changing angular velocity.

(frame-mirror-failure)=
## Failure record: lateral and angular axes were mirrored

After the rear encoder map was corrected, left strafe and counter-clockwise
rotation still appeared negative while forward motion was positive. Encoder
signs and physical mapping were already independently constrained, so the
repair flips the kinematic $v_y$ and $\omega$ outputs to REP-103 rather than
retuning encoder signs.

## Source and evidence

- [`data/Week4 Trials.xlsx`](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/data/Week4%20Trials.xlsx)
- [`odometry.py`](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/odometry.py)
- [Ground-truth notes in `PROJECT_STATE.md`](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/PROJECT_STATE.md)

## Verification state

The mapping, frame, and translation scale diagnoses are recorded. A new series
of medium-speed trials on the 1 m by 1 m foam surface is still required before
the project can claim a final translational and heading noise distribution.
