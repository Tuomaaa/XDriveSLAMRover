# Week 5 - Ultrasonic Sensing and Measurement Model

:::{admonition} Chapter status: Outline; engineering milestone verified
:class: status status-verified

The dual-sensor acquisition path, calibration data, range model, map ray
casting, and end-to-end pose discrimination test are complete. The prose and
figures remain to be expanded.
:::

## Objective

Turn two low-cost HC-SR04 range sensors into a calibrated likelihood model that
can score a robot pose inside a rectangular map.

## Prerequisites

- STM32 TIM9 input capture and a shared trigger line
- CAN message `0x202`
- ROS 2 `sensor_msgs/Range` publication
- Known right and rear sensor offsets in the robot frame

## Calibrated model snapshot

The measured range-dependent standard deviation is

$$
\sigma(d) = 0.0017 + 0.0078d \quad \text{meters}.
$$

The four-component beam model uses
$w_{hit}=0.94$, $w_{short}=0.01$, $w_{max}=0.03$, and $w_{rand}=0.02$.
All mixture calculations are performed in log space when combining sensors.

## Planned chapter sections

- TIM9 input-capture timing and sequential ranging
- Range/status encoding and ROS 2 publication
- Bench calibration procedure and fitted noise model
- Probabilistic beam-model components
- Rectangular-map ray casting with sensor offsets
- Map validation and likelihood discrimination

(ultrasonic-invalids)=
## Failure and limitation record

The bench campaign observed an overall invalid rate of about 2.9 percent.
Invalid or saturated readings cannot be silently treated as ordinary Gaussian
hits; the status byte and the `z_max`/random mixture components preserve that
distinction. A small positive bias was measured but intentionally not corrected
because it was minor relative to the current localization resolution.

## Source and evidence

- [`measurement_model.py`](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/measurement_model.py)
- [`map_model.py`](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/map_model.py)
- [`bench_calibration.csv`](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/data/ultrasonic/bench_calibration.csv)
- [`map_validation.csv`](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/data/ultrasonic/map_validation.csv)

## Verification state

Two sensors were measured at four distances with 100 samples per condition.
Eight map-validation channels across four robot poses were within 1.3 mm of
their expected ranges, and every true pose scored above the tested shifted pose.
