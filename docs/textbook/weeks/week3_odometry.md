# Week 3 - X-Drive Kinematics and Encoder Odometry

:::{admonition} Chapter status: Outline
:class: status status-outline

The estimator, encoder calibration, and ROS frame convention are implemented.
Long-distance ground-truth repetition remains part of Week 4.
:::

## Objective

Convert four cumulative encoder counts into a planar pose
$\mathbf{x} = [x, y, \theta]^T$ while keeping physical wheel identity, sign,
scale, and frame conventions explicit.

## Prerequisites

- Working encoder frames `0x200` and `0x201`
- Known motor and encoder harness mapping
- Measured wheel radius, center-to-wheel distance, and encoder CPR
- REP-103 convention: forward $+x$, left $+y$, counter-clockwise $+\theta$

## Kinematic snapshot

For wheel surface velocities ordered as front-left, front-right, rear-left,
and rear-right, the unnormalized inverse convention is

$$
\begin{aligned}
v_{fl} &= v_x + v_y + \omega R, &
v_{fr} &= v_x - v_y - \omega R, \\
v_{rl} &= v_x - v_y + \omega R, &
v_{rr} &= v_x + v_y - \omega R.
\end{aligned}
$$

The estimator uses forward kinematics and midpoint integration. Translational
output is multiplied by $\sqrt{2}$ to restore the physical projection of wheels
mounted at 45 degrees; angular output is not scaled by that factor.

## Planned chapter sections

- X-drive inverse and forward kinematics
- Encoder counter rollover and cumulative tick transport
- Physical motor mapping versus encoder harness mapping
- CPR calibration and per-channel sign correction
- Midpoint pose integration
- ROS 2 `nav_msgs/Odometry` and `odom -> base_link` TF publication

(encoder-map-failure)=
## Failure record: rear encoders were swapped

Pure forward motion looked correct while strafe and rotation channels appeared
to exchange roles. Hand-turning one physical wheel at a time showed that
encoder indices 2 and 3 were wired to rear-right and rear-left respectively,
opposite the motor-drive enum. The repair is an intentional encoder-side
`MOTOR_MAP = {0: fl, 1: fr, 2: rr, 3: rl}`. The two mappings must not be
"cleaned up" into false agreement.

(pid-runaway)=
## Failure record: reversed feedback caused PID runaway

All four raw encoder counts decreased during commanded forward motion. Feeding
those signs directly into the retained PID loop made the controller positive
feedback: motors accelerated immediately and ignored the intended setpoint.
The repair adds an encoder sign of `-1` to each PID feedback channel. PID is
currently compiled out for manual odometry experiments, but the corrected path
is retained for later autonomous velocity control.

(heartbeat-overwrite)=
## Failure record: the emergency stop was overwritten

The heartbeat timeout initially called `motors_stop()`, then the normal 20 ms
control block immediately restored PWM. The visible stop therefore did not
remain active. The repair gates the entire control update on heartbeat health,
keeps PWM at zero while timed out, and clears PID integral state.

## Source and evidence

- [`odometry.py`](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/odometry.py)
- [`encoder_monitor.py`](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/encoder_monitor.py)
- [`can_bridge_node.py`](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/can_bridge_node.py)

## Verification state

The encoder CPR is calibrated to 2779, all four raw signs are known, the rear
encoder swap is represented explicitly, and the output frame is aligned with
REP-103. Direction and scale have bench and short-course evidence; the full
repeatability campaign belongs to Week 4.
