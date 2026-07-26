# SummerSLAM Engineering Notes

**Building an X-Drive SLAM Rover, One Layer at a Time**

:::{admonition} Notes status: Weeks 1-3 first drafts, Weeks 4-6 outlined
:class: status status-progress

The site structure and evidence map are live. Weeks 1-2 form one platform and
programming narrative, and Week 3 begins the concept-focused localization
work. Weeks 4-6 remain outlines and will be expanded in order. A chapter's
engineering status describes the rover, not the completeness of the written
notes.
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
| {doc}`1-2 <weeks/week1_platform>` | Platform bring-up and programming | First draft | Motors, encoders, CAN transport, and PS2 control verified; tutorials and build evidence remain to be added |
| {doc}`3 <weeks/week3_odometry>` | X-drive kinematics and encoder odometry | First draft | Encoder map, signs, CPR, frame convention, and estimator implemented |
| {doc}`4 <weeks/week4_motion_uncertainty>` | Ground-truth calibration and motion uncertainty | Outline | Translation scale identified; full repeatability campaign remains pending |
| {doc}`5 <weeks/week5_ultrasonic>` | Ultrasonic sensing and measurement model | Outline | Bench calibration and map validation completed |
| {doc}`6 <weeks/week6_mcl>` | Monte Carlo Localization | In progress | Design is on `main`; implementation remains on `week6-mcl` |

## How to use these notes

Read the chapters in order if you are reproducing the rover. Each chapter will
grow toward the same contract:

1. Define the physical or probabilistic model.
2. State the hardware and software prerequisites.
3. Link the exact implementation and input data.
4. Describe the experiment and expected observations.
5. Separate verified results from hypotheses and remaining work.
6. Preserve failures and repairs as reproducible debugging evidence.

This is an engineering process journal, not a general robotics textbook. It
explains what I chose, what failed, what I measured, and what I would change so
that a reader can adapt the process instead of treating one parts list as the
only correct build.

The source repository is
[Tuomaaa/XDriveSLAMRover](https://github.com/Tuomaaa/XDriveSLAMRover).
Project status and decisions are tracked in
[`PROJECT_STATE.md`](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/PROJECT_STATE.md).

```{toctree}
:hidden:
:maxdepth: 2
:caption: Weeks 1-6

weeks/week1_platform
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
