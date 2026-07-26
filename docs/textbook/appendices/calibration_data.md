# Calibration Data

:::{admonition} Appendix status: Initial index
:class: status status-outline

Raw observations are preserved in the repository. Chapter prose must cite the
input file and distinguish raw measurements from derived parameters.
:::

## Encoder and motion calibration

| Artifact | Purpose | Current conclusion |
| --- | --- | --- |
| [`Week4 Trials.xlsx`](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/data/Week4%20Trials.xlsx) | Ground-truth motion trials | Supports the $\sqrt{2}$ translation correction; full uncertainty campaign pending |
| [`odometry.py`](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/odometry.py) | Applied calibration constants | CPR 2779, all signs `-1`, rear encoder swap explicit |

## Ultrasonic calibration

| Artifact | Purpose | Current conclusion |
| --- | --- | --- |
| [`bench_calibration.csv`](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/data/ultrasonic/bench_calibration.csv) | Raw two-sensor samples | 2 sensors x 4 distances x 100 samples |
| [`bench_summary.csv`](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/data/ultrasonic/bench_summary.csv) | Group statistics | Supports $\sigma(d)=0.0017+0.0078d$ m |
| [`map_validation.csv`](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/data/ultrasonic/map_validation.csv) | Map/sensor end-to-end test | True pose outscored shifted poses in all four positions |

## Reproducibility rule

A parameter belongs in a completed chapter only when its acquisition method,
units, sample count, raw input, transformation, and verification result are all
available. Theoretical defaults must remain labeled as theoretical.
