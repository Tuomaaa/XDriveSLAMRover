# Week 3 - X-Drive Kinematics and Encoder Odometry

:::{admonition} Chapter status: implemented, with validation still in progress
:class: status status-progress

The kinematics, encoder calibration, and pose estimator are implemented. I
verified the wheel map, encoder signs, CPR, frame direction, and a left-strafe
smoke test. Longer ground-truth runs, drift measurements, and a complete ROS
and TF test on the Raspberry Pi are not complete, so this chapter does not
claim that the final odometry accuracy is known.
:::

## From a moving rover to a motion estimate

Weeks 1-2 ended with a rover that could move and send encoder counts. Week 3
asks a different question: what do those four counts say about the motion of
the whole rover?

The output I want is a two-dimensional pose:

$$
\mathbf{x} =
\begin{bmatrix}
x & y & \theta
\end{bmatrix}^{T}.
$$

Encoder odometry is a form of dead reckoning. Each new wheel motion is added
to the previous pose. This makes the estimate simple and useful, but it also
means that wheel slip, scale errors, and heading errors accumulate instead of
disappearing.

The conversion path is:

1. Receive four cumulative encoder counts.
2. Subtract the previous counts.
3. Correct the encoder wiring map and count directions.
4. Convert tick changes into wheel velocities.
5. Use X-drive forward kinematics to find body velocity.
6. Rotate that motion into the `odom` frame.
7. Integrate it into the next pose.

The equations were not the most difficult part. Most of my debugging time went
into deciding which physical wheel each number represented, which direction
was positive, and which coordinate convention the controller was using.

## How to read the repository

Follow the data from the STM32 to the final ROS message. Reading the files in
this order makes the boundaries between wiring, protocol, mathematics, and ROS
clear.

| Order | File | What to read |
| ---: | --- | --- |
| 1 | [main.c motor order](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/stm32/Core/Src/main.c#L58) | Start with `MotorPosition`, then read the two encoder CAN frames. This is the motor-side order, not proof of the physical encoder harness. |
| 2 | [protocol.py decoder](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/protocol.py#L72) | See how CAN `0x200` and `0x201` become two pairs of signed 32-bit cumulative counts. |
| 3 | [encoder_monitor.py main loop](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/encoder_monitor.py#L33) | Read the baseline and delta logic. During mapping, turn one wheel at a time and treat the four columns as observed channels rather than trusting a wheel name in advance. |
| 4 | [odometry.py configuration](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/odometry.py#L27) | Read the geometry, translation scale, CPR, `MOTOR_MAP`, and `ENCODER_SIGN` before reading the equations. |
| 5 | [OdometryEstimator.update()](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/odometry.py#L114) | Follow one update from tick subtraction through kinematics and midpoint integration. |
| 6 | [can_bridge_node.py encoder path](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/can_bridge_node.py#L106) | See how the bridge collects both frames, calls the estimator, publishes `odom`, and broadcasts TF. |
| 7 | [PS2_Drive_Test.py inverse kinematics](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/PS2_Drive_Test.py#L120) | Read this last. It explains the command convention that the forward kinematics must invert. |

Inside `odometry.py`, the fastest reading order is:

1. [Physical and calibration constants](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/odometry.py#L27)
2. [Encoder harness map](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/odometry.py#L46)
3. [Encoder sign correction](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/odometry.py#L69)
4. [Tick-to-wheel conversion](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/odometry.py#L90)
5. [Forward kinematics](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/odometry.py#L139)
6. [Midpoint pose integration](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/odometry.py#L164)
7. [Standalone smoke test](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/odometry.py#L185)

This order keeps a wiring correction from looking like part of the kinematic
equation.

## Define one coordinate frame first

I use the ROS REP-103 convention:

- $+x$ points forward.
- $+y$ points left.
- $+z$ points upward.
- Positive $\theta$ is counter-clockwise when viewed from above.

The wheel names are `fl`, `fr`, `rl`, and `rr`. They mean front-left,
front-right, rear-left, and rear-right.

The measured geometry used by the estimator is:

$$
r = 0.025\ \mathrm{m},
\qquad
R = 0.115\ \mathrm{m},
$$

where $r$ is wheel radius and $R$ is the distance from the rover center to a
wheel contact point.

The two rear channels show why position names and numeric indexes must be kept
separate:

| Physical wheel | Motor command index | Encoder channel index |
| --- | ---: | ---: |
| Front-left | 0 | 0 |
| Front-right | 1 | 1 |
| Rear-left | 2 | 3 |
| Rear-right | 3 | 2 |

The motor indexes follow the intended drive wiring. The encoder indexes follow
the harness that was actually built. Both tables are true at the same time.

## Why the STM32 sends cumulative counts

The STM32 extends the encoder timers into cumulative 32-bit counts and sends
them in two CAN frames:

- `0x200`: channels 0 and 1
- `0x201`: channels 2 and 3

The frame construction is visible in
[main.c](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/stm32/Core/Src/main.c#L364).
The Python decoder for the same payload is in
[protocol.py](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/protocol.py#L72).
These two files should be read together whenever the CAN layout changes.

For wheel $i$, the estimator calculates:

$$
\Delta N_i = N_{i,k} - N_{i,k-1}.
$$

Cumulative counts are useful because one lost CAN frame does not permanently
lose its motion. The next received total still includes the missing interval.
With per-frame deltas, that motion would be gone.

The first sample cannot produce a delta, so `update()` stores it as a baseline
and returns no pose result. It also rejects duplicate or out-of-order
timestamps instead of dividing by zero or integrating backward.

## Convert ticks into wheel velocity

Let $C$ be encoder counts per output-shaft revolution. For a measured time gap
$\Delta t$, the wheel-surface velocity is:

$$
u_i =
\frac{2\pi r\Delta N_i}{C\Delta t}.
$$

The implementation is
[ticks_to_wheel_velocity()](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/odometry.py#L90).

The STM32 sends encoder frames near 50 Hz, but Linux and CAN timing do not make
every gap exactly 20 ms. The estimator therefore uses the actual CAN receive
timestamps:

$$
\Delta t = t_k - t_{k-1}.
$$

This prevents scheduling jitter from being treated as a change in wheel
speed. The bridge waits until frame `0x201` completes the four-channel set,
then calls the estimator with that frame's timestamp. Read that handoff in
[can_bridge_node.py](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/can_bridge_node.py#L120).

This pairing is intentionally simple. There is no sequence number, so a rare
lost or reordered half-pair can still combine counts from different instants.
Cumulative counts reduce the damage, but they do not make the pairing perfect.

## Measure the real CPR

The theoretical encoder value is:

$$
C_{theory}
=
7\ \mathrm{PPR}
\times 4
\times 100
=
2800.
$$

The factor of four comes from quadrature decoding. I did not assume that the
motor encoder and gearbox matched their nominal values exactly, so I turned
each output shaft ten revolutions by hand and divided the count change by ten.

| Wheel | Measured counts per output turn |
| --- | ---: |
| Front-left | 2779.5 |
| Front-right | 2778.3 |
| Rear-left | 2778.7 |
| Rear-right | 2777.6 |
| Mean | 2778.5 |

I rounded the shared odometry value to:

$$
C = 2779.
$$

The mean is about 0.8% below the theoretical value, while the spread between
wheels is only about 0.07%. One shared CPR is reasonable for this stage.

These values came from the manual ten-turn test. The compact raw trial log and
shaft-mark photos were not preserved as a publishable artifact, so the table
should not be treated as a full calibration dataset. A careful reproduction
should mark the shaft, record the start and end count for every trial, repeat
the measurement, and keep that raw table in the repository.

The STM32 PID branch still contains `ENCODER_CPR = 2800`, while odometry uses
2779. This does not change the raw counts sent over CAN. PID is currently
disabled, and its constant must be reviewed before closed-loop control is used
again.

## Keep map, sign, frame, and scale separate

I treat four calibration questions as separate variables:

1. **Map:** Which physical wheel produced each channel?
2. **Sign:** Does forward wheel motion increase or decrease the raw count?
3. **Frame:** Do the final $v_y$ and $\omega$ directions follow REP-103?
4. **Scale:** How much physical motion does one tick represent?

Changing one of these values to hide another problem makes later debugging
much harder. For example, an encoder sign can fix one reversed wheel, but it
should not be used to repair a left-right coordinate-frame mirror.

(encoder-map-failure)=
### Failure: the rear encoder channels were swapped

My first motion test had a specific pattern:

- Forward motion changed $x$ as expected.
- Sideways motion appeared as rotation.
- Rotation appeared as sideways motion.

If only one sign were wrong, the channels would mix in a less symmetric way.
The clean exchange between $v_y$ and $\omega$ suggested that two wheel
positions were swapped.

I ran the encoder monitor:

```console
cd ros2_ws/src
sudo python3 encoder_monitor.py
```

Then I turned one physical wheel at a time and watched which column changed.
Encoder channel 2 was connected to the rear-right wheel, while channel 3 was
connected to the rear-left wheel.

The correction belongs in
[`MOTOR_MAP`](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/odometry.py#L62):

```python
MOTOR_MAP = {
    0: "fl",
    1: "fr",
    2: "rr",
    3: "rl",
}
```

I did not change the firmware because the motor harness and encoder harness
are independent. The STM32 motor enum still describes the command wiring. The
Python map describes where the feedback wires actually landed.

## Calibrate encoder signs with one simple motion

After mapping the channels, I drove straight forward. All four physical wheel
velocities should be positive for that command, but all four raw encoder totals
decreased. The measured correction is therefore:

```python
ENCODER_SIGN = {0: -1, 1: -1, 2: -1, 3: -1}
```

This forward test fully determines the four signs. Once all four channels read
positive during physical forward motion, I do not change those signs to repair
sideways or rotational behavior. A remaining error must be in the wheel map,
frame convention, or equations.

(pid-runaway)=
### Failure: reversed feedback caused PID runaway

The same raw sign problem once turned the speed controller into positive
feedback. A positive command produced a negative measured speed, so PID saw a
large error and increased the output. The encoder value then moved farther in
the wrong direction, which made the controller add even more power.

The result was full motor speed with almost no useful controller response.

The retained PID branch now applies the four `ENC_SIGN` corrections before
calculating speed. Read the sign application and safety gate in
[main.c](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/stm32/Core/Src/main.c#L332).
`USE_PID` remains `0`, so this branch has not been revalidated on the current
hardware.

The practical test rule is simple: lift the rover, begin with low output, and
confirm that a positive command produces positive measured feedback before
enabling closed-loop control.

## Separate the physical model from the controller units

For a physical X-drive model, body velocity can be written as wheel-surface
velocity commands with a rotational term proportional to $\omega R$. One
sign convention is:

$$
\begin{aligned}
u_{fl} &= v_x + v_y + \omega R, \\
u_{fr} &= v_x - v_y - \omega R, \\
u_{rl} &= v_x - v_y + \omega R, \\
u_{rr} &= v_x + v_y - \omega R.
\end{aligned}
$$

The manual controller does not work in meters per second and radians per
second. It first converts joystick positions into normalized values, combines
them with the same sign pattern, and then scales the four results to motor RPM.
For that reason,
[PS2_Drive_Test.py](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/PS2_Drive_Test.py#L90)
uses a normalized `omega` term instead of writing `omega * R` directly.

This distinction matters when reproducing the project. The controller code is
good for manual direction commands, but it is not a unit-correct `cmd_vel` to
wheel-RPM conversion. A future autonomous velocity controller should convert
SI units explicitly and include the rover geometry in that boundary.

The controller also scales all four commands together when any one command
exceeds the motor limit. This keeps the requested motion direction while
reducing its magnitude.

## Derive forward kinematics

Forward kinematics reverses the wheel combination. Before the final frame and
scale corrections, the algebraic combinations are:

$$
\tilde{v}_x =
\frac{u_{fl} + u_{fr} + u_{rl} + u_{rr}}{4},
$$

$$
\tilde{v}_y =
\frac{u_{fl} - u_{fr} - u_{rl} + u_{rr}}{4},
$$

$$
\tilde{\omega} =
\frac{u_{fl} - u_{fr} + u_{rl} - u_{rr}}{4R}.
$$

The rover then needs two measured corrections:

$$
v_x = \sqrt{2}\,\tilde{v}_x,
\qquad
v_y = -\sqrt{2}\,\tilde{v}_y,
\qquad
\omega = -\tilde{\omega}.
$$

The actual implementation is the three-line block in
[OdometryEstimator.update()](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/odometry.py#L160).

### Why lateral motion and rotation are negated

After the rear-channel map was fixed, physical left motion produced negative
$v_y$, and a physical counter-clockwise turn produced negative $\omega$.
Forward $v_x$ was already correct.

That pattern means the remaining problem is a left-right frame mirror, not a
free encoder sign. Negating $v_y$ and $\omega$ at the forward-kinematics output
aligns odometry with REP-103 while leaving the already-correct $v_x$ alone.
The detailed debugging record is in
{ref}`REP-103 frame mirror <frame-mirror-failure>`.

### Why translation is multiplied by $\sqrt{2}$

The controller and the first forward-kinematics implementation used an
unnormalized 45-degree wheel convention. In the early ground-truth trials,
forward and sideways odometry measured about 0.74 times the tape-measured
translation, while in-place rotation remained near the correct scale.

That pattern is close to the missing projection
$\cos(45^\circ)=1/\sqrt{2}$. Multiplying only the two translation terms by
$\sqrt{2}$ corrects that convention without changing rotation.

The measurements are stored in
[data/Week4 Trials.xlsx](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/data/Week4%20Trials.xlsx).
They are early tests, not a final accuracy result: translation was measured on
foam, and the actual rotation angles were estimated by eye. The evidence is
strong enough to identify the missing projection factor, but not strong enough
to claim precise long-run odometry performance. See
{ref}`missing 45-degree projection <translation-scale-failure>` for the full
failure trail.

This $\sqrt{2}$ correction belongs to the conventions used by this rover. It
should not be copied into another X-drive implementation without deriving its
wheel equations and checking real motion.

## Integrate body motion into the world frame

The kinematic equations produce velocity in the rover's body frame. The pose
is stored in the world-fixed `odom` frame, so each body displacement must be
rotated by the rover heading.

I use the heading at the middle of the time interval:

$$
\theta_{mid}
=
\theta_k + \frac{\omega\Delta t}{2}.
$$

The pose update is:

$$
\begin{aligned}
x_{k+1}
&=
x_k
+
(v_x\cos\theta_{mid} - v_y\sin\theta_{mid})\Delta t, \\
y_{k+1}
&=
y_k
+
(v_x\sin\theta_{mid} + v_y\cos\theta_{mid})\Delta t, \\
\theta_{k+1}
&=
\theta_k + \omega\Delta t.
\end{aligned}
$$

Midpoint integration uses the average heading during the step instead of the
heading only at its beginning. The difference is small at 50 Hz, but it is
more accurate when the rover translates and rotates at the same time. The code
also wraps $\theta$ into $(-\pi,\pi]$ so logs remain readable.

Read the complete integration block in
[odometry.py](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/odometry.py#L164).
I verified its current sign convention with the standalone left-strafe smoke
test, but I have not published a measured Euler-versus-midpoint path comparison.

## Keep the estimator independent of ROS

`OdometryEstimator` accepts a dictionary of four cumulative counts and one
timestamp. It returns plain numbers:

```text
(x, y, theta, vx, vy, omega)
```

It does not import CAN or ROS libraries. This lets me run the mathematical
smoke test on a laptop with:

```console
python ros2_ws/src/odometry.py
```

The ROS boundary belongs to `can_bridge_node.py`. The node:

1. decodes `0x200` and `0x201`;
2. waits until all four cumulative counts are available;
3. calls `OdometryEstimator.update()`;
4. publishes `nav_msgs/Odometry` on `odom`;
5. broadcasts `odom -> base_link`;
6. prints a reduced-rate pose log for ground-truth work.

Read the estimator handoff in
[can_bridge_node.py](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/can_bridge_node.py#L182),
then read the
[`Odometry` and TF publication](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/can_bridge_node.py#L197).

The publication code exists, but a complete Raspberry Pi record containing
`ros2 topic echo`, topic rate, TF tree, and RViz path has not been captured.
That remains an unverified integration boundary rather than a completed result.

## Test one degree of freedom at a time

The standalone smoke test creates a physical left-strafe wheel pattern and
checks:

$$
v_x = 0,
\qquad
\omega = 0,
\qquad
v_y > 0.
$$

For hardware testing, I use a fixed order because each motion isolates a
different class of mistake:

1. Turn one wheel by hand to identify its encoder channel.
2. Drive forward to fix all four encoder signs.
3. Move left to verify the lateral frame direction.
4. Turn counter-clockwise to verify angular direction.
5. Measure translation and rotation to check scale.
6. Only then run longer paths and drift tests.

The observed error pattern is often more useful than the size of the error:

| Observation | First item to inspect |
| --- | --- |
| One channel has the wrong direction | That encoder sign |
| Sideways motion and rotation exchange roles | Wheel-to-channel map |
| Left and counter-clockwise are both reversed, but forward is correct | Coordinate-frame mirror |
| Forward and sideways share one scale error | CPR or 45-degree projection |
| Only angular scale is wrong | Center-to-wheel distance $R$ |
| Results change strongly with the surface | Wheel slip and contact conditions |

(heartbeat-overwrite)=
### Failure: the heartbeat stop was overwritten

The first timeout logic called `motors_stop()`, but the normal 20 ms control
block ran immediately afterward and wrote a nonzero PWM value again. The stop
command was correct for only a moment.

The fix calculates `hb_ok` and uses it to gate the full control update. When
the heartbeat is missing, PWM stays at zero; the PID branch also clears its
stored error to prevent windup. Encoder accumulation continues so timer wraps
are still handled correctly.

Read the actual gate before trusting this description:
[main.c heartbeat handling](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/stm32/Core/Src/main.c#L308).
This safety behavior should always be tested with the rover lifted before a
floor run.

## What Week 3 established

The estimator now uses:

| Setting | Current value | Evidence state |
| --- | --- | --- |
| Encoder map | `{0: fl, 1: fr, 2: rr, 3: rl}` | One-wheel hand test on the real harness |
| Encoder sign | `-1` for all four channels | Physical forward-motion test |
| Encoder CPR | 2779 | Four ten-turn manual measurements |
| Wheel radius | 0.025 m | Physical geometry |
| Center-to-wheel distance | 0.115 m | Physical geometry |
| Output frame | forward $+x$, left $+y$, CCW $+\theta$ | Direction tests aligned to REP-103 |
| Translation scale | $\sqrt{2}$ | Early foam ground-truth pattern |
| Time step | CAN receive timestamp difference | Implemented in the estimator boundary |
| Pose integration | Midpoint method | Code and standalone smoke test |

This produces one consistent pose estimate. It does not remove wheel slip,
unequal motor response, floor effects, or accumulated heading error. Week 4
measures those effects instead of treating odometry as exact.

## Reproduction procedure

1. Write down the body frame and physical wheel names before changing code.
2. Record motor command order separately from encoder feedback order.
3. Turn each wheel by hand and map every observed encoder channel.
4. Drive one simple forward command and determine every encoder sign.
5. Measure CPR over several output-shaft turns and keep the raw counts.
6. Measure wheel radius $r$ and center-to-wheel distance $R$ on the built rover.
7. Derive forward kinematics from the same convention used by the drive code.
8. Verify forward, left, and counter-clockwise motion separately.
9. Run the standalone estimator smoke test before adding CAN or ROS.
10. Verify `odom`, topic rate, and `odom -> base_link` on the Raspberry Pi.
11. Measure drift on the actual test surface before using odometry in localization.

## Source index

- [Odometry configuration, kinematics, integration, and smoke test](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/odometry.py)
- [Raw encoder monitoring tool](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/encoder_monitor.py)
- [Normalized PS2 X-drive command path](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/PS2_Drive_Test.py)
- [CAN-to-odometry ROS boundary](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/can_bridge_node.py)
- [CAN payload decoder](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/protocol.py)
- [STM32 motor order, encoder frames, PID sign, and heartbeat gate](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/stm32/Core/Src/main.c)
- [Early ground-truth measurements](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/data/Week4%20Trials.xlsx)
- [Project decisions and unresolved validation work](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/PROJECT_STATE.md)
