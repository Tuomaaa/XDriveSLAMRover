# Week 3 - X-Drive Kinematics and Encoder Odometry

:::{admonition} Chapter status: First draft
:class: status status-progress

The main math and code are done. I tested the wheel map, encoder signs, CPR,
frame direction, and pose update. Week 4 will do longer tests.
:::

## From building to modeling

The rover now moves and sends encoder data. The platform build is mostly done.
From this point on, I focus more on ideas and models.

The main question also changes:

> What does each sensor value mean? How can I use it to estimate motion?

In Week 3, I turn four encoder counts into one rover pose:

$$
\mathbf{x} =
\begin{bmatrix}
x & y & \theta
\end{bmatrix}^{T}.
$$

This is called dead reckoning. It adds each new motion to the last pose. Small
errors also get added. They grow over time.

## What I need to solve

The full path is:

1. Read four encoder counts.
2. Find how much each count changed.
3. Convert each change into wheel speed.
4. Find the rover's forward, side, and turn speed.
5. Add this motion to the last pose.

Before doing the math, I must know three things:

- Which encoder belongs to each wheel?
- Which count direction is positive?
- How many counts equal one wheel turn?

## Set the coordinate frame

I use the ROS REP-103 frame:

- $+x$ is forward.
- $+y$ is left.
- $+z$ is up.
- Positive $\theta$ is a counter-clockwise turn.

The wheel names are:

- $fl$: front-left
- $fr$: front-right
- $rl$: rear-left
- $rr$: rear-right

The wheel radius is

$$
r = 0.025\ \mathrm{m}.
$$

The distance from the rover center to a wheel is

$$
R = 0.115\ \mathrm{m}.
$$

I write these rules down first. A formula can be correct but still use the
wrong direction.

:::{admonition} Diagram placeholder - frame and wheels
:class: asset-placeholder

Add a top view of the rover. Show front, $x$, $y$, positive $\theta$, all four
wheel names, roller angles, $r$, and $R$.
:::

## Turn encoder ticks into wheel speed

An encoder gives a count. It does not give speed.

The STM32 sends a running total. This is called a cumulative count. For wheel
$i$, I first find the change:

$$
\Delta N_i = N_{i,k} - N_{i,k-1}.
$$

I also find the time change:

$$
\Delta t = t_k - t_{k-1}.
$$

Let $C$ be the counts for one output-shaft turn. The wheel speed is

$$
u_i =
\frac{2\pi r\Delta N_i}{C\Delta t}.
$$

This code is in
[`ticks_to_wheel_velocity()`](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/odometry.py).

### Why I send a running total

A CAN frame can be lost. A running total helps with this problem.

If one frame is lost, the next count still includes that motion. A per-frame
change would lose it forever.

The first update only saves a start value. Two values are needed to find a
change.

## Measure the real CPR

The motor label gives a theoretical value:

$$
C_{theory}
=
7\ \mathrm{PPR}
\times 4
\times 100
=
2800.
$$

I did not trust this value. I turned each output shaft ten times by hand.

| Wheel | Counts per turn |
| --- | ---: |
| Front-left | 2779.5 |
| Front-right | 2778.3 |
| Rear-left | 2778.7 |
| Rear-right | 2777.6 |
| Mean | 2778.5 |

I use

$$
C = 2779.
$$

The real value is about 0.8% lower than 2800. The four wheels are very close to
each other. One shared value is good enough for this stage.

The old PID code on the STM32 still uses 2800. PID is now off. Raw CAN counts
are not changed by this value. I will update it before I turn PID on again.

:::{admonition} Evidence placeholder - CPR test
:class: asset-placeholder

Add the shaft marks, raw counts, test steps, repeat trials, date, and firmware
commit.
:::

## Keep map, sign, and scale separate

These are three different problems:

1. **Map:** Which wheel made this count?
2. **Sign:** Does forward motion make the count go up or down?
3. **Scale:** How far does one count move the wheel?

One setting should not hide an error in another setting.

### The motor map

The motor output order is:

| Motor index | Wheel | Timer |
| ---: | --- | --- |
| 0 | Front-left | TIM2 |
| 1 | Front-right | TIM3 |
| 2 | Rear-left | TIM4 |
| 3 | Rear-right | TIM5 |

But the rear encoder wires are swapped. The odometry map is:

| Encoder index | Real wheel | Sign |
| ---: | --- | ---: |
| 0 | Front-left | -1 |
| 1 | Front-right | -1 |
| 2 | Rear-right | -1 |
| 3 | Rear-left | -1 |

The code keeps this map in `MOTOR_MAP`. Motor wires and encoder wires are two
different harnesses. Their maps do not need to look the same.

(encoder-map-failure)=
### Failure: the rear encoders were swapped

The first test gave a clear pattern:

- Forward motion changed $x$.
- Side motion looked like a turn.
- A turn looked like side motion.

This did not look like one bad sign. It looked like two wheel names were
swapped.

I ran:

```console
cd ros2_ws/src
sudo python3 encoder_monitor.py
```

I turned one wheel at a time. Index 2 was on the rear-right wheel. Index 3 was
on the rear-left wheel.

I fixed the labels in `MOTOR_MAP`. I did not change the firmware.

:::{admonition} Evidence placeholder - encoder map
:class: asset-placeholder

Add the connector photos, raw monitor output, and the one-wheel test table.
:::

### The encoder signs

I then drove straight forward. All four wheels should give positive forward
motion. All four raw counts went down.

So I use:

```python
ENCODER_SIGN = {0: -1, 1: -1, 2: -1, 3: -1}
```

After this test, I do not change the signs to fix side motion or turning. Those
errors must come from the frame or the wheel math.

(pid-runaway)=
### Failure: the wrong sign caused PID runaway

The same sign error broke the PID loop. A positive motor command gave a
negative speed value. PID saw a large error and added more power. The measured
error then became even larger.

The motors went to full speed. The controller had no useful effect.

The firmware now flips all four signs inside the PID path. PID is still off.
I will test it again before future use.

Always test encoder feedback with the rover lifted. Start with low power.

## Inverse kinematics

Inverse kinematics turns rover motion into four wheel commands.

The drive program uses:

$$
\begin{aligned}
u_{fl} &= v_x + v_y + \omega R, \\
u_{fr} &= v_x - v_y - \omega R, \\
u_{rl} &= v_x - v_y + \omega R, \\
u_{rr} &= v_x + v_y - \omega R.
\end{aligned}
$$

This code is in
[`PS2_Drive_Test.py`](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/PS2_Drive_Test.py).

If one command is too large, I scale all four commands together. This keeps the
same motion direction.

These formulas match my rover and my wheel signs. They are not a rule for every
X-drive. A different build may need different signs.

The formulas also leave out the 45-degree scale factor. I correct that later.

:::{admonition} Diagram placeholder - wheel commands
:class: asset-placeholder

Add wheel arrows for forward motion, left motion, and a counter-clockwise turn.
:::

## Forward kinematics

Forward kinematics does the opposite job. It turns four wheel speeds into rover
motion.

First, I undo the drive formulas:

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

These values still use the drive program's frame and scale.

### Fix the frame and scale

The final code uses:

$$
v_x = \sqrt{2}\,\tilde{v}_x,
$$

$$
v_y = -\sqrt{2}\,\tilde{v}_y,
$$

$$
\omega = -\tilde{\omega}.
$$

The negative signs fix the frame. A real left move first gave negative $v_y$.
A real counter-clockwise turn first gave negative $\omega$. The full test is
in {ref}`REP-103 frame mirror <frame-mirror-failure>`.

The $\sqrt{2}$ fixes translation scale. Early tests gave about 0.74 m of
odometry for 1 m of real motion. This is close to $1/\sqrt{2}$. Rotation did
not have the same error. The full test is in
{ref}`missing 45-degree projection <translation-scale-failure>`.

Do not copy this $\sqrt{2}$ fix without testing your own rover. It belongs to
my wheel setup and my drive formulas.

## The update steps

Each call to `OdometryEstimator.update()` does this:

1. Read four running counts.
2. Subtract the last counts.
3. Apply `ENCODER_SIGN`.
4. Apply `MOTOR_MAP`.
5. Convert ticks to wheel speed.
6. Run forward kinematics.
7. Fix the frame and scale.
8. Add the motion to the pose.

The order is important. A scale value cannot fix a wrong wheel map.

## Use the real time gap

The STM32 sends data near 50 Hz. The time gap should be near 20 ms. It is not
always exact.

I use the CAN receive times:

$$
\Delta t = t_k - t_{k-1}.
$$

This lowers error from CAN delay and Linux timing.

The counts come in CAN frames `0x200` and `0x201`. The bridge updates the
pose after it gets the second frame. A lost first frame can still make one bad
pair. The running counts make the error small, so I did not add a frame number
at this stage.

## Update the pose

Wheel math gives speed in the rover frame. Pose lives in the world `odom`
frame. I must rotate the motion by the rover angle.

I use the angle in the middle of the time step:

$$
\theta_{mid}
=
\theta_k + \frac{\omega\Delta t}{2}.
$$

Then:

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

This is the midpoint method. It is a small step up from plain Euler. It works
better when the rover moves and turns at the same time.

I keep $\theta$ between $-\pi$ and $\pi$.

:::{admonition} Plot placeholder - pose update
:class: asset-placeholder

Add one simple plot that compares Euler, midpoint, and a true circle.
:::

## Keep the math separate from ROS

`OdometryEstimator` only needs counts and time. It returns:

```text
(x, y, theta, vx, vy, omega)
```

It does not know about CAN or ROS. I can test the math on a laptop.

[`can_bridge_node.py`](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/can_bridge_node.py)
handles ROS. It publishes `nav_msgs/Odometry` on `odom`. It also sends the
`odom -> base_link` transform.

The code exists. The full ROS test on the real Pi is still not done.

:::{admonition} Evidence placeholder - ROS output
:class: asset-placeholder

Add `ros2 topic echo /odom`, topic rate, TF tree, launch commands, and an RViz
path.
:::

## Test one motion at a time

The code has a left-move smoke test. It checks:

$$
v_x = 0, \qquad \omega = 0, \qquad v_y > 0.
$$

For the real rover, I use this order:

1. Turn one wheel by hand.
2. Drive forward.
3. Move left.
4. Turn counter-clockwise.
5. Measure distance and angle.

The error pattern often points to the cause:

| What I see | What I check first |
| --- | --- |
| One wheel has the wrong sign | Encoder sign |
| Side motion and turning swap | Wheel map |
| Left and turn are both reversed | Frame direction |
| Both move axes have one scale error | CPR or 45-degree scale |
| Only turn scale is wrong | $R$ |
| Results change with the floor | Wheel slip |

(heartbeat-overwrite)=
### Failure: heartbeat stop did not stay on

The first timeout called `motors_stop()`. The next control update turned PWM
on again.

The fix blocks the full motor update when the heartbeat is missing. PWM stays
at zero. PID memory is also cleared.

I test this with the rover lifted before any floor test.

## Week 3 result

The model now uses:

- Map: `{0: fl, 1: fr, 2: rr, 3: rl}`
- Sign: $-1$ for all four encoders
- CPR: 2779
- Wheel radius: 0.025 m
- Center-to-wheel distance: 0.115 m
- Frame: forward $+x$, left $+y$, CCW $+\theta$
- Move scale: $\sqrt{2}$
- Time: real CAN receive time
- Pose update: midpoint method

This gives one clear pose estimate. It is not perfect. Wheels slip. Motors are
not equal. The floor changes the result. Small errors grow over time.

Week 4 measures these errors.

## Reproduction checklist

- [ ] Draw the frame and wheel directions.
- [ ] Find each encoder index by hand.
- [ ] Test every encoder sign.
- [ ] Measure CPR.
- [ ] Measure $r$ and $R$.
- [ ] Test forward, left, and turn motion.
- [ ] Keep map, sign, scale, and frame settings separate.
- [ ] Test `odom` and `odom -> base_link` on the Pi.
- [ ] Measure drift in Week 4.

## Source files

- [`odometry.py`](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/odometry.py)
- [`encoder_monitor.py`](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/encoder_monitor.py)
- [`PS2_Drive_Test.py`](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/PS2_Drive_Test.py)
- [`can_bridge_node.py`](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/can_bridge_node.py)
- [`main.c`](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/stm32/Core/Src/main.c)

## Verification state

CPR, signs, wheel map, frame direction, move scale, and midpoint code are done.
The left-move smoke test passes.

Long tests, drift data, floor tests, and the full ROS test are not done here.
They belong to Week 4 or later.
