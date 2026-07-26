# SummerSLAM

**A hardware-backed X-drive rover for learning localization and SLAM by
building, measuring, and reproducing the complete system.**

:::{admonition} Textbook status: Week 1-6 scaffold
:class: status status-progress

The navigation and evidence map are live. The chapters are intentionally
outlines and will be expanded in order from Week 1. A chapter's engineering
status describes the rover, not the completeness of the written chapter.
:::

SummerSLAM connects an STM32F411 motor-control layer to a Raspberry Pi 4B
running ROS 2 Jazzy. Encoder odometry, calibrated ultrasonic measurements, and
probabilistic localization are added one measurable layer at a time. The
repository keeps firmware, robot-side software, raw data, and the reasoning
behind calibration decisions together.

```{figure} _static/system-architecture.svg
:alt: SummerSLAM system architecture from the PS2 controller and ROS 2 computer through CAN to the STM32, drive base, encoders, and ultrasonic sensors.
:class: architecture-figure
:align: center

The Week 1-6 system boundary. Commands move left to right; telemetry returns
to the ROS 2 computer for estimation.
```

## Six-week roadmap

| Week | Topic | Chapter status | Engineering evidence |
| --- | --- | --- | --- |
| {doc}`1 <weeks/week1_platform>` | Rover platform and hardware bring-up | Outline | Drive electronics, power, and base operation recorded |
| {doc}`2 <weeks/week2_can_ros2>` | STM32, CAN, and ROS 2 control pipeline | Outline | Core STM32-CAN link verified; ROS 2 integration remains ongoing |
| {doc}`3 <weeks/week3_odometry>` | X-drive kinematics and encoder odometry | Outline | Encoder map, signs, CPR, frame convention, and estimator implemented |
| {doc}`4 <weeks/week4_motion_uncertainty>` | Ground-truth calibration and motion uncertainty | Outline | Translation scale identified; full repeatability campaign remains pending |
| {doc}`5 <weeks/week5_ultrasonic>` | Ultrasonic sensing and measurement model | Outline | Bench calibration and map validation completed |
| {doc}`6 <weeks/week6_mcl>` | Monte Carlo Localization | In progress | Design is on `main`; implementation remains on `week6-mcl` |

## How to use this textbook

Read the chapters in order if you are reproducing the rover. Each chapter will
grow toward the same contract:

1. Define the physical or probabilistic model.
2. State the hardware and software prerequisites.
3. Link the exact implementation and input data.
4. Describe the experiment and expected observations.
5. Separate verified results from hypotheses and remaining work.
6. Preserve failures and repairs as reproducible debugging evidence.

The source repository is
[Tuomaaa/XDriveSLAMRover](https://github.com/Tuomaaa/XDriveSLAMRover).
Project status and decisions are tracked in
[`PROJECT_STATE.md`](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/PROJECT_STATE.md).

```{toctree}
:hidden:
:maxdepth: 2
:caption: Weeks 1-6

weeks/week1_platform
weeks/week2_can_ros2
weeks/week3_odometry
weeks/week4_motion_uncertainty
weeks/week5_ultrasonic
weeks/week6_mcl
```

```{toctree}
:hidden:
:maxdepth: 2
:caption: Appendices

appendices/reproduction_environment
appendices/can_protocol
appendices/calibration_data
appendices/debugging_safety
```
