# Week 5 — Ultrasonic Sensing and Measurement Model

:::{admonition} Chapter status: first draft, engineering milestone verified
:class: status status-verified

The dual-sensor acquisition path, bench calibration, beam model, map ray
casting, and end-to-end pose discrimination test are complete. All eight
map-validation channels were within 1.3 mm of the expected value. The true
pose outscored every tested shifted pose.
:::

:::{admonition} File guide
:class: tip

**Core algorithm — read these first**

- [`measurement_model.py`](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/measurement_model.py) — `BeamModel` (4-component mixture: hit/short/max/rand), log-space likelihood
- [`map_model.py`](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/map_model.py) — `RectMap`, 2D ray casting with sensor offsets

**Recommended**

- [`hc_sr04.c`](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/stm32/Core/Src/hc_sr04.c) — TIM9 input-capture state machine, sequential right-then-back ranging

**Infrastructure — safe to skip**

- `main.c` — CAN frame 0x202 construction (five bytes, two ranges plus status)
- `protocol.py` — 0x202 codec (`right_mm`, `back_mm`, validity bits)
- `can_bridge_node.py` — `sensor_msgs/Range` publication

**Data — reference only**

- `data/ultrasonic/bench_calibration.csv` — 800 samples, 2 sensors × 4 distances × 100
- `data/ultrasonic/bench_summary.csv` — group statistics (mean, std, bias, invalid rate)
- `data/ultrasonic/map_validation.csv` — 4 positions × 100 samples × 2 sensors
:::

## From dead reckoning to range sensing

Encoder odometry accumulates error with every step. Week 4 showed that even
a short trial adds several degrees of heading drift from motor imbalance
alone. Without an external reference, this drift is invisible to the
estimator.

Range sensors provide that reference. If the rover knows the map, it can
measure its distance to a wall and score candidate poses. This chapter adds
two HC-SR04 ultrasonic sensors, calibrates their noise, builds a
probabilistic measurement model, and verifies it against a rectangular map.

The output is a log-likelihood function. Given a candidate pose and a map,
this function scores how well the measured ranges agree with the expected
ranges. The function becomes the measurement update in the Week 6 particle
filter.

## How to read the repository

| Order | File | What to read |
| ---: | --- | --- |
| 1 | [hc_sr04.c](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/stm32/Core/Src/hc_sr04.c) | TIM9 input-capture state machine, shared trigger, sequential right-then-back ranging. |
| 2 | [hc_sr04.h](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/stm32/Core/Inc/hc_sr04.h) | `HC_SR04_Result` struct and the `INVALID_MM` sentinel. |
| 3 | [main.c ultrasonic integration](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/stm32/Core/Src/main.c#L384) | CAN frame `0x202` construction inside the 20 ms control loop. |
| 4 | [protocol.py ultrasonic codec](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/protocol.py#L82) | `decode()` for `0x202`: `right_mm`, `back_mm`, and validity bits. |
| 5 | [can_bridge_node.py ultrasonic path](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/can_bridge_node.py#L153) | ROS `sensor_msgs/Range` publication on `/ultrasonic/right` and `/ultrasonic/back`. |
| 6 | [bench_calibration.csv](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/data/ultrasonic/bench_calibration.csv) | 800 valid samples: 2 sensors, 4 distances, 100 per condition. |
| 7 | [bench_summary.csv](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/data/ultrasonic/bench_summary.csv) | Group statistics: mean, std, bias, invalid rate per condition. |
| 8 | [measurement_model.py](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/measurement_model.py) | `BeamModel` and `GaussianModel` with log-space likelihood. |
| 9 | [map_model.py](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/map_model.py) | `RectMap` and 2D ray casting with sensor offsets. |
| 10 | [map_validation.csv](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/data/ultrasonic/map_validation.csv) | 4 positions, 100 samples each, two sensors per position. |

## Part 1 — Hardware acquisition

### Why HC-SR04

The HC-SR04 is inexpensive, widely available, and returns a range in
millimeters. Its main limitations are a narrow beam, a slow update rate, and
about 2 cm minimum range. For a rectangular map with flat walls perpendicular
to the sensor, these limitations are acceptable.

I used two sensors: one faces right and one faces backward. Two sensors in
different directions give the particle filter information about both axes of
the rover's position in a rectangular map.

### TIM9 input capture

The HC-SR04 returns range as a pulse width on its ECHO pin. The STM32
measures this pulse with TIM9 input capture. TIM9 timestamps the rising and
falling edges in hardware.

The pin assignment replaced the USART2 debug serial port:

| Pin | Previous use | New use |
| --- | --- | --- |
| PA2 | USART2 TX | TIM9 CH1, right sensor ECHO |
| PA3 | USART2 RX | TIM9 CH2, back sensor ECHO |
| PA4 | — | TRIG output (shared) |

This was the only way to get two input-capture channels. Motors, encoders,
SPI, and direction GPIO already use all other timers and pins.

### State machine

The driver uses an eight-state enum
([`hc_sr04.c` L24–L33](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/stm32/Core/Src/hc_sr04.c#L24-L33)):

```c
// hc_sr04.c L24-L33
typedef enum {
  HC_STATE_IDLE = 0,
  HC_STATE_TRIG_RIGHT,
  HC_STATE_WAIT_RIGHT_RISING,
  HC_STATE_WAIT_RIGHT_FALLING,
  HC_STATE_TRIG_BACK,
  HC_STATE_WAIT_BACK_RISING,
  HC_STATE_WAIT_BACK_FALLING,
  HC_STATE_DONE,
} HC_SR04_State;
```

### Sequential ranging with a shared trigger

Both sensors share a single TRIG line because no GPIO pins were available for
separate triggers. A single TRIG pulse fires both modules, but the driver
controls which ECHO response it accepts.

The `trigger()` function fires a 12 us pulse and sets the state to wait for
the rising edge on the correct channel
([`hc_sr04.c` L67–L84](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/stm32/Core/Src/hc_sr04.c#L67-L84)):

```c
// hc_sr04.c L67-L84
static void trigger(uint8_t right_sensor)
{
  uint32_t channel = right_sensor ? TIM_CHANNEL_1 : TIM_CHANNEL_2;
  uint32_t interrupt = right_sensor ? TIM_IT_CC1 : TIM_IT_CC2;

  state = right_sensor ? HC_STATE_TRIG_RIGHT : HC_STATE_TRIG_BACK;
  set_rising_polarity(channel);
  __HAL_TIM_CLEAR_IT(&htim9, interrupt);

  uint32_t now = HAL_GetTick();
  echo_deadline_ms = now + HC_SR04_TIMEOUT_MS;
  next_trigger_ms = now + HC_SR04_TRIGGER_SPACING_MS;
  HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);
  delay_us(HC_SR04_TRIGGER_US);
  state = right_sensor ? HC_STATE_WAIT_RIGHT_RISING
                       : HC_STATE_WAIT_BACK_RISING;
  HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);
}
```

The sequence is:

1. Fire TRIG (12 us pulse).
2. Wait for the right sensor's rising edge on TIM9 CH1.
3. Wait for the right sensor's falling edge. Compute the pulse width.
4. Wait 60 ms to avoid crosstalk.
5. Fire TRIG again.
6. Wait for the back sensor's rising edge on TIM9 CH2.
7. Wait for the back sensor's falling edge. Compute the pulse width.
8. Report both results and return to idle.

### Input-capture ISR

The TIM9 capture callback handles both channels
([`hc_sr04.c` L162–L197](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/stm32/Core/Src/hc_sr04.c#L162-L197)):

```c
// hc_sr04.c L170-L182 — right sensor path
if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
    capture = (uint16_t)HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    if (state == HC_STATE_WAIT_RIGHT_RISING) {
      rising_capture = capture;
      __HAL_TIM_SET_CAPTUREPOLARITY(
          htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
      state = HC_STATE_WAIT_RIGHT_FALLING;
    } else if (state == HC_STATE_WAIT_RIGHT_FALLING) {
      right_pulse_us = (uint16_t)(capture - rising_capture);
      result.status |= HC_SR04_STATUS_RIGHT_VALID;
      set_rising_polarity(TIM_CHANNEL_1);
      state = HC_STATE_TRIG_BACK;
    }
}
```

On the rising edge, the ISR saves the counter value and flips the polarity
to falling. On the falling edge, it computes the pulse width as the
difference. Then it advances the state to trigger the back sensor.

### Timeout handling

Each phase has a 40 ms timeout. If no echo arrives, the sensor reports
`HC_SR04_INVALID_MM`. The `HC_SR04_IsComplete()` function handles the timeout
([`hc_sr04.c` L134–L146](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/stm32/Core/Src/hc_sr04.c#L134-L146)):

```c
// hc_sr04.c L134-L146
if ((state == HC_STATE_WAIT_RIGHT_RISING ||
     state == HC_STATE_WAIT_RIGHT_FALLING) &&
    deadline_reached(now, echo_deadline_ms)) {
  result.right_mm = HC_SR04_INVALID_MM;
  set_rising_polarity(TIM_CHANNEL_1);
  state = HC_STATE_TRIG_BACK;
} else if ((state == HC_STATE_WAIT_BACK_RISING ||
            state == HC_STATE_WAIT_BACK_FALLING) &&
           deadline_reached(now, echo_deadline_ms)) {
  result.back_mm = HC_SR04_INVALID_MM;
  set_rising_polarity(TIM_CHANNEL_2);
  state = HC_STATE_DONE;
}
```

### Pulse-to-distance conversion

```c
// hc_sr04.c L54-L60
static uint16_t pulse_to_mm(uint16_t pulse_width_us)
{
  uint32_t distance = ((uint32_t)pulse_width_us * 10U + 29U) / 58U;
  return (distance < HC_SR04_INVALID_MM) ? (uint16_t)distance
                                         : (HC_SR04_INVALID_MM - 1U);
}
```

The conversion divides the round-trip time by 58 to get millimeters, with
rounding. This formula uses the standard speed-of-sound approximation at
room temperature.

### CAN frame 0x202

The STM32 packs both measurements into a single CAN frame inside the 20 ms
control loop
([`main.c` L384–L396](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/stm32/Core/Src/main.c#L384-L396)):

```c
// main.c L384-L396
if (HC_SR04_IsComplete()) {
    HC_SR04_Result ultrasonic = HC_SR04_GetResult();
    can_frame tx_ultrasonic;
    tx_ultrasonic.can_id = CAN_ID_ULTRASONIC;
    tx_ultrasonic.can_dlc = 5;
    tx_ultrasonic.data[0] = (uint8_t)(ultrasonic.right_mm & 0xFFU);
    tx_ultrasonic.data[1] = (uint8_t)(ultrasonic.right_mm >> 8);
    tx_ultrasonic.data[2] = (uint8_t)(ultrasonic.back_mm & 0xFFU);
    tx_ultrasonic.data[3] = (uint8_t)(ultrasonic.back_mm >> 8);
    tx_ultrasonic.data[4] = ultrasonic.status;
    MCP_sendMessage(&tx_ultrasonic);
}
HC_SR04_StartMeasurement();
```

| Byte | Content |
| ---: | --- |
| 0-1 | `right_mm`, uint16 little-endian |
| 2-3 | `back_mm`, uint16 little-endian |
| 4 | Status: bit 0 = right valid, bit 1 = back valid |

The full right-then-back cycle takes more than one loop iteration. The
firmware polls `HC_SR04_IsComplete()` each cycle and sends the frame only
when both readings are ready.

### RPi decode

On the RPi side, `protocol.py` decodes `0x202` into a plain dict
([`protocol.py` L82–L89](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/protocol.py#L82-L89)):

```python
# protocol.py L82-L89
if arbitration_id == MsgId.ULTRASONIC:
    right_mm, back_mm, status = struct.unpack('<HHB', data)
    return {
        "right_mm": right_mm,
        "back_mm": back_mm,
        "right_valid": bool(status & 0x01),
        "back_valid": bool(status & 0x02),
    }
```

### ROS publication

The CAN bridge routes this dict to `_on_ultrasonic`, which publishes two
`sensor_msgs/Range` topics
([`can_bridge_node.py` L153–L180](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/can_bridge_node.py#L153-L180)):

```python
# can_bridge_node.py L153-L160
def _on_ultrasonic(self, reading, timestamp):
    stamp = Time(nanoseconds=int(timestamp * 1e9)).to_msg()
    self._publish_range(
        self._ultrasonic_right_pub, stamp, 'ultrasonic_right',
        reading['right_mm'], reading['right_valid'])
    self._publish_range(
        self._ultrasonic_back_pub, stamp, 'ultrasonic_back',
        reading['back_mm'], reading['back_valid'])
```

The `_publish_range` helper builds the `Range` message
([`can_bridge_node.py` L170–L180](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/can_bridge_node.py#L170-L180)):

```python
# can_bridge_node.py L170-L180
@staticmethod
def _publish_range(publisher, stamp, frame_id, distance_mm, valid):
    msg = Range()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.radiation_type = Range.ULTRASOUND
    msg.field_of_view = 0.26
    msg.min_range = 0.02
    msg.max_range = 4.0
    msg.range = distance_mm / 1000.0 if valid else float('inf')
    publisher.publish(msg)
```

The bridge publishes invalid readings as `float('inf')`. The `field_of_view`
value is 0.26 radians (about 15 degrees). The minimum range is 0.02 m and
the maximum is 4.0 m.

## Part 2 — Bench calibration

### Procedure

I placed each sensor in front of a flat KT foam board at four distances: 100,
300, 600, and 1000 mm. At each distance I collected 100 valid readings. I
discarded invalid returns and recorded their count separately.

The raw data is in
[`bench_calibration.csv`](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/data/ultrasonic/bench_calibration.csv).
The per-condition statistics are in
[`bench_summary.csv`](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/data/ultrasonic/bench_summary.csv).

### Results

| Sensor | Distance (mm) | Mean (mm) | Std (mm) | Bias (mm) | Invalid % |
| --- | ---: | ---: | ---: | ---: | ---: |
| Right | 100 | 102.75 | 2.28 | +2.75 | 2.9 |
| Right | 300 | 304.01 | 3.83 | +4.01 | 1.0 |
| Right | 600 | 605.02 | 6.55 | +5.02 | 2.9 |
| Right | 1000 | 1007.14 | 9.82 | +7.14 | 3.8 |
| Rear | 100 | 102.50 | 2.41 | +2.50 | 4.8 |
| Rear | 300 | 303.80 | 4.10 | +3.80 | 2.0 |
| Rear | 600 | 606.03 | 6.87 | +6.03 | 2.9 |
| Rear | 1000 | 1006.58 | 8.71 | +6.59 | 2.9 |

Two observations guided the model:

1. Standard deviation grows approximately linearly with distance.
2. Bias is small and positive. It grows from about 2.5 mm at 100 mm to about
   7 mm at 1000 mm.

### Noise model

I fitted a linear model to the standard deviation across both sensors:

$$
\sigma(d) = 0.0017 + 0.0078\,d \quad \text{(meters)}.
$$

The `_sigma_at` helper in `measurement_model.py` evaluates this model. It
accepts a constant, a `(sigma_0, sigma_d)` tuple, or a callable
([`measurement_model.py` L24–L35](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/measurement_model.py#L24-L35)):

```python
# measurement_model.py L24-L35
def _sigma_at(sigma_hit, z_expected):
    if callable(sigma_hit):
        sigma = float(sigma_hit(z_expected))
    elif isinstance(sigma_hit, (tuple, list)):
        if len(sigma_hit) != 2:
            raise ValueError('distance-dependent sigma must be (sigma_0, sigma_d)')
        sigma = float(sigma_hit[0]) + float(sigma_hit[1]) * z_expected
    else:
        sigma = float(sigma_hit)
    if not math.isfinite(sigma) or sigma <= 0.0:
        raise ValueError('sigma_hit must remain finite and positive')
    return sigma
```

The two sensors showed nearly identical trends. A per-sensor fit differed by
less than 0.5 mm, so I used a single merged model.

I measured a bias of approximately 2.4 mm plus 0.47 percent of distance, but
I did not correct it. At 1 m the total bias is about 7 mm. This is small
relative to the particle spacing in the Week 6 filter. If localization
accuracy becomes a problem, bias correction is the first item to revisit.

### Invalid readings

The overall invalid rate was about 2.9 percent. The measurement model must
not treat invalid readings as ordinary Gaussian samples. The beam model
handles them through its max-range and random mixture components.

## Part 3 — Probabilistic beam model

### The four-component model

The measurement model follows the beam model from Probabilistic Robotics,
Chapter 6. Given an expected range $z^*$ and a measured range $z$, the
likelihood is a mixture of four components:

$$
p(z \mid z^*) = w_{hit}\,p_{hit} + w_{short}\,p_{short}
              + w_{max}\,p_{max} + w_{rand}\,p_{rand}.
$$

| Component | Meaning | Weight |
| --- | --- | ---: |
| $p_{hit}$ | Truncated Gaussian centered on $z^*$ with $\sigma(z^*)$ | 0.94 |
| $p_{short}$ | Exponential decay for unexpected close obstacles | 0.01 |
| $p_{max}$ | Point mass at max range for invalid or saturated readings | 0.03 |
| $p_{rand}$ | Uniform over $[0, z_{max}]$ for unexplained noise | 0.02 |

I set the weights from the bench observations:

- The data was very clean: no crosstalk, no multipath short readings.
- The invalid rate of about 3 percent maps directly to $w_{max}$.
- $w_{short}$ stays at a minimal value for robustness.
- $w_{rand}$ absorbs readings that do not fit the other three components.

### BeamModel constructor

The constructor validates all weights and the sigma configuration
([`measurement_model.py` L83–L116](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/measurement_model.py#L83-L116)):

```python
# measurement_model.py L83-L105
class BeamModel(_MultiSensorMixin):
    def __init__(
        self,
        z_max,
        sigma_hit,
        lambda_short,
        w_hit,
        w_short,
        w_max,
        w_rand,
    ):
        self.z_max = float(z_max)
        self.sigma_hit = sigma_hit
        self.lambda_short = float(lambda_short)
        self.w_hit = float(w_hit)
        self.w_short = float(w_short)
        self.w_max = float(w_max)
        self.w_rand = float(w_rand)
```

### Hit component: truncated Gaussian

The hit component uses the distance-dependent $\sigma(d)$ from the bench
calibration. The truncated Gaussian is normalized over $[0, z_{max}]$
([`measurement_model.py` L125–L139](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/measurement_model.py#L125-L139)):

```python
# measurement_model.py L125-L139
def _log_hit(self, z, z_expected):
    if z < 0.0 or z > self.z_max:
        return _NEG_INF
    sigma = _sigma_at(self.sigma_hit, z_expected)
    lower = (0.0 - z_expected) / sigma
    upper = (self.z_max - z_expected) / sigma
    normalization = _normal_cdf(upper) - _normal_cdf(lower)
    if normalization <= 0.0:
        return _NEG_INF
    residual = (z - z_expected) / sigma
    return (
        -0.5 * residual * residual
        - math.log(_SQRT_2PI * sigma)
        - math.log(normalization)
    )
```

### Short component: exponential decay

The short component models unexpected close obstacles. It uses a truncated
exponential that drops to zero at the expected range
([`measurement_model.py` L141–L149](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/measurement_model.py#L141-L149)):

```python
# measurement_model.py L141-L149
def _log_short(self, z, z_expected):
    if z_expected <= 0.0 or z < 0.0 or z > z_expected:
        return _NEG_INF
    normalization = -math.expm1(-self.lambda_short * z_expected)
    return (
        math.log(self.lambda_short)
        - self.lambda_short * z
        - math.log(normalization)
    )
```

### Log-space mixture

The code does all mixture calculations in log space to avoid underflow when
it combines multiple sensors
([`measurement_model.py` L157–L174](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/measurement_model.py#L157-L174)):

```python
# measurement_model.py L157-L174
def log_likelihood(self, z_measured, z_expected):
    z_measured = float(z_measured)
    expected = self._expected(float(z_expected))
    if expected is None or math.isnan(z_measured) or z_measured < 0.0:
        return _NEG_INF
    if math.isinf(z_measured) or z_measured >= self.z_max:
        return self._max_range_log_likelihood()

    terms = (
        _weighted_log(
            self.w_hit, self._log_hit(z_measured, expected)),
        _weighted_log(
            self.w_short, self._log_short(z_measured, expected)),
        _NEG_INF,
        _weighted_log(self.w_rand, -math.log(self.z_max)),
    )
    return _logsumexp(terms)
```

When the measured value is at max-range or infinite, the code returns the
combined log-probability of $w_{max}$ and $w_{rand}$
([`measurement_model.py` L151–L155](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/measurement_model.py#L151-L155)):

```python
# measurement_model.py L151-L155
def _max_range_log_likelihood(self):
    return _logsumexp((
        _weighted_log(self.w_max, 0.0),
        _weighted_log(self.w_rand, -math.log(self.z_max)),
    ))
```

### Numerical helpers

`_logsumexp` and `_weighted_log` keep the computation in log space
([`measurement_model.py` L10–L21](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/measurement_model.py#L10-L21)):

```python
# measurement_model.py L10-L21
def _logsumexp(terms):
    finite_terms = [term for term in terms if math.isfinite(term)]
    if not finite_terms:
        return _NEG_INF
    maximum = max(finite_terms)
    return maximum + math.log(
        sum(math.exp(term - maximum) for term in finite_terms))

def _weighted_log(weight, component_log):
    if weight <= 0.0 or component_log == _NEG_INF:
        return _NEG_INF
    return math.log(weight) + component_log
```

## Part 4 — Map and ray casting

### Rectangular map

The test environment is a 116.5 cm KT foam board square, open at the top.
I model it as an axis-aligned rectangle with four walls. The `RectMap`
constructor validates the bounds
([`map_model.py` L9–L20](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/map_model.py#L9-L20)):

```python
# map_model.py L9-L20
class RectMap:
    def __init__(self, x_min, x_max, y_min, y_max):
        bounds = (x_min, x_max, y_min, y_max)
        if not all(math.isfinite(value) for value in bounds):
            raise ValueError('map bounds must be finite')
        if x_min >= x_max or y_min >= y_max:
            raise ValueError('minimum bounds must be smaller than maximum bounds')
        self.x_min = float(x_min)
        self.x_max = float(x_max)
        self.y_min = float(y_min)
        self.y_max = float(y_max)
```

### Ray casting

The `ray_cast` method tests a ray against all four wall segments
([`map_model.py` L22–L48](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/map_model.py#L22-L48)):

```python
# map_model.py L22-L48
def ray_cast(self, x, y, angle):
    dx = math.cos(angle)
    dy = math.sin(angle)
    candidates = []

    if abs(dx) > _EPSILON:
        for wall_x in (self.x_min, self.x_max):
            distance = (wall_x - x) / dx
            hit_y = y + distance * dy
            if (
                distance > _EPSILON
                and self.y_min - _EPSILON <= hit_y <= self.y_max + _EPSILON
            ):
                candidates.append(distance)

    if abs(dy) > _EPSILON:
        for wall_y in (self.y_min, self.y_max):
            distance = (wall_y - y) / dy
            hit_x = x + distance * dx
            if (
                distance > _EPSILON
                and self.x_min - _EPSILON <= hit_x <= self.x_max + _EPSILON
            ):
                candidates.append(distance)

    return min(candidates) if candidates else float('inf')
```

For an axis-aligned rectangle, each wall produces at most one intersection
candidate. Four divisions and a few comparisons are enough.

### Sensor offsets

The sensors are not at the rover center. The right sensor faces $-\pi/2$
(rightward) with an offset of $(0.0, -0.09)$ m from the base_link origin.
The back sensor faces $\pi$ (backward) with an offset of $(-0.09, 0.0)$ m.

The `expected_range` method transforms the sensor position from the robot
frame into the world frame before ray casting
([`map_model.py` L50–L64](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/map_model.py#L50-L64)):

```python
# map_model.py L50-L64
def expected_range(self, pose, sensor_angle_on_robot, sensor_offset=(0.0, 0.0)):
    x, y, theta = pose
    offset_x, offset_y = sensor_offset
    cos_theta = math.cos(theta)
    sin_theta = math.sin(theta)
    sensor_x = x + cos_theta * offset_x - sin_theta * offset_y
    sensor_y = y + sin_theta * offset_x + cos_theta * offset_y
    return self.ray_cast(sensor_x, sensor_y, theta + sensor_angle_on_robot)
```

This is the standard 2D rigid-body transform. The sensor offset rotates with
the rover heading. The beam direction is the sum of the heading and the
sensor's mounting angle.

### Multi-sensor scoring

`total_log_likelihood` sums the log-likelihoods of all valid sensors at a
candidate pose
([`measurement_model.py` L43–L80](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/measurement_model.py#L43-L80)):

```python
# measurement_model.py L67-L80
total = 0.0
used = 0
for observation in observations:
    z_measured, sensor_angle, sensor_offset = observation
    if z_measured is None:
        continue
    z_expected = rect_map.expected_range(
        pose, sensor_angle, sensor_offset)
    score = self.log_likelihood(z_measured, z_expected)
    if score == _NEG_INF:
        return _NEG_INF
    total += score
    used += 1
return total if used else 0.0
```

It skips any `None` measurement. The particle filter calls this function once
per particle per update step.

## Part 5 — Map validation

### Procedure

I placed the rover at four known positions inside the 116.5 cm KT board
square. I collected 100 samples at each position. The positions were:

| Position | Description | Right offset (mm) | Back offset (mm) |
| --- | --- | ---: | ---: |
| P1 | Center | 0 | 0 |
| P2 | Shifted right | +80 | 0 |
| P3 | Shifted back | 0 | +100 |
| P4 | Shifted both | +60 | +70 |

At each position I computed the expected range with `RectMap.expected_range`
and the measured sensor offsets. I then compared it to the mean of the 100
observed readings.

### Results

All eight channels (4 positions times 2 sensors) agreed with the expected
range to within 1.3 mm. This result verifies that the sensor offsets, the map
geometry, and the ray casting are consistent with the physical setup.

### Likelihood discrimination test

At each position I compared the beam model log-likelihood at the true pose
against a shifted pose. The shift was +50 mm or -50 mm in one axis. In all
four positions, the true pose scored higher than every tested shifted pose.

This test does not prove full localization. It proves that the model can tell
the correct pose from a nearby wrong pose. This is the minimum requirement
for a particle filter measurement update.

The data is in
[`map_validation.csv`](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/data/ultrasonic/map_validation.csv).

### Smoke test

The `_run_tests()` function at
[`measurement_model.py` L247–L284](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/measurement_model.py#L247-L284)
verifies the beam model and map integration end-to-end:

```python
# measurement_model.py L249-L283
model = BeamModel(
    z_max=4.0,
    sigma_hit=(0.0017, 0.0078),
    lambda_short=0.5,
    w_hit=0.94,
    w_short=0.01,
    w_max=0.03,
    w_rand=0.02,
)
peak = model.log_likelihood(1.2, 1.2)
far = model.log_likelihood(2.2, 1.2)
assert math.isfinite(peak) and peak > far
assert math.isfinite(model.log_likelihood(float('inf'), 1.2))
assert model.log_likelihood(float('nan'), 1.2) == _NEG_INF

from map_model import RectMap
rect_map = RectMap(-1.0, 1.0, -1.0, 1.0)
score = model.total_log_likelihood(
    [1.0, 1.0],
    (0.0, 0.0, 0.0),
    rect_map,
    [(-math.pi / 2, (0.0, 0.0)), (math.pi, (0.0, 0.0))],
)
assert math.isfinite(score)
```

The test verifies three properties: a measurement at the expected range
scores higher than a far measurement, an infinite reading returns a finite
score, and `NaN` returns $-\infty$.

(ultrasonic-invalids)=
## Failure and limitation record

### Invalid readings

The bench campaign observed an overall invalid rate of about 2.9 percent.
The measurement model must not treat invalid or saturated readings as
ordinary Gaussian hits. The status byte and the $z_{max}$/random mixture
components preserve that distinction. The beam model returns the combined
$w_{max} + w_{rand}$ log-probability for any max-range or infinite reading.

### Bias

I measured a small positive bias but did not correct it. The bias grows from
about 2.5 mm at 100 mm to about 7 mm at 1000 mm. At the current localization
resolution this is minor. If the Week 6 particle filter shows a systematic
offset toward a wall, add bias correction first.

### Beam width and non-perpendicular incidence

The HC-SR04 beam is roughly 15 degrees wide. If the rover is rotated so the
beam hits a wall at a shallow angle, the first return can come from the edge
of the beam rather than its center. This produces a shorter reading than
expected. The rectangular map and perpendicular sensor mounting minimize this
effect. The effect will grow if the model is extended to arbitrary wall
angles.

### Debug serial port removed

The PA2/PA3 pin reassignment removed the only debug serial output. STM32
debugging now uses CAN frame observation and ST-Link breakpoints. This is a
permanent trade-off for this board revision.

## What Week 5 established

| Result | Evidence |
| --- | --- |
| STM32 TIM9 dual-sensor acquisition | Sequential right-then-back, CAN 0x202, 60 ms spacing |
| ROS integration | `/ultrasonic/right` and `/ultrasonic/back` as `sensor_msgs/Range` |
| Noise model | $\sigma(d) = 0.0017 + 0.0078\,d$ from 800 samples across 8 conditions |
| Beam model | $w_{hit}=0.94$, $w_{short}=0.01$, $w_{max}=0.03$, $w_{rand}=0.02$ |
| Map ray casting | `RectMap` with sensor offsets, validated at 4 positions |
| Likelihood discrimination | True pose outscored shifted poses in all 4 test positions |

## Reproduction procedure

1. Wire two HC-SR04 sensors to the STM32: shared TRIG on PA4, right ECHO on
   PA2 (TIM9 CH1), back ECHO on PA3 (TIM9 CH2).
2. Verify that the CubeMX configuration assigns PA2 and PA3 to TIM9 input
   capture.
3. Flash the firmware.
4. Verify that CAN frame `0x202` appears with valid readings. Use
   `candump can0`.
5. Set up a flat target (KT board or similar) at a known distance.
6. Collect at least 100 readings per distance at 3 or more distances across
   the sensor's useful range.
7. Fit a noise model to the standard deviation versus distance.
8. Set beam model weights from the observed invalid rate and the presence or
   absence of short readings.
9. Build a `RectMap` that matches the physical test enclosure.
10. Measure the sensor offsets from the rover center on the built chassis.
11. Place the rover at known positions inside the map. Compare the expected
    ranges with the observed ranges.
12. Run the likelihood discrimination test: the true pose must outscore
    nearby wrong poses.
13. Run `python ros2_ws/src/measurement_model.py` to verify that the smoke
    tests pass.

## Source index

- [HC-SR04 input-capture driver](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/stm32/Core/Src/hc_sr04.c)
- [HC-SR04 header](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/stm32/Core/Inc/hc_sr04.h)
- [STM32 ultrasonic CAN integration](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/stm32/Core/Src/main.c)
- [CAN protocol encoder and decoder](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/protocol.py)
- [ROS ultrasonic bridge](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/can_bridge_node.py)
- [Beam model and Gaussian model](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/measurement_model.py)
- [Rectangular map and ray casting](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/map_model.py)
- [Bench calibration raw data](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/data/ultrasonic/bench_calibration.csv)
- [Bench calibration summary](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/data/ultrasonic/bench_summary.csv)
- [Map validation data](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/data/ultrasonic/map_validation.csv)
