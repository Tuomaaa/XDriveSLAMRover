# Week 6 - Monte Carlo Localization

:::{admonition} Chapter status: In progress
:class: status status-progress

The design and implementation plan are on `main`. The implementation and
synthetic replay evidence are on the `week6-mcl` branch and must be merged and
reverified before this chapter can claim completion.
:::

## Objective

Fuse holonomic encoder motion with the Week 5 ultrasonic likelihood model to
represent the rover pose as a weighted particle set and recover from odometry
drift inside the measured rectangular map.

## Prerequisites

- Body-frame odometry increments from Week 3/4
- The calibrated multi-sensor likelihood from Week 5
- A rectangular map and measured sensor extrinsics
- A deterministic random seed for replayable tests

## Planned chapter sections

- Particle state, initialization, and weighted pose estimate
- Holonomic body-frame motion update
- Ultrasonic measurement update in log space
- Effective sample size and low-variance resampling
- CSV logging and offline replay
- Convergence against dead reckoning
- Limits of heading observability with two ultrasonic sensors

(mcl-design-corrections)=
## Design corrections retained for expansion

- The standard `rot1/trans/rot2` motion decomposition assumes a platform that
  turns to translate. SummerSLAM instead perturbs body-frame $dx$, $dy$, and
  $d\theta$ so lateral X-drive motion remains representable.
- The filter needs a rotational noise floor to maintain particle heading
  diversity. It does not create heading observability where the sensor geometry lacks it.
- The convergence test must compare synthetic evidence against ground truth.
  A shared error can look like success.

## Source and evidence

- [MCL design specification on `main`](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/docs/superpowers/specs/2026-07-22-mcl-particle-filter-design.md)
- [MCL implementation plan on `main`](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/docs/superpowers/plans/2026-07-22-mcl-particle-filter.md)
- [`mcl.py` on `week6-mcl`](https://github.com/Tuomaaa/XDriveSLAMRover/blob/week6-mcl/ros2_ws/src/mcl.py)
- [Offline replay tools on `week6-mcl`](https://github.com/Tuomaaa/XDriveSLAMRover/tree/week6-mcl/ros2_ws/src)

## Verification state

This chapter intentionally reports branch state. Merge, clean test execution,
hardware log replay, and documentation evidence are still required before the
status changes from `In progress`.
