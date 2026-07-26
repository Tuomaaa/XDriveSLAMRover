# Safety and Debugging Index

This appendix makes failure evidence discoverable without hiding it behind the
final implementation. Each entry links to the week where its physical symptom,
diagnosis, and repair belong.

## Power and hardware safety

- {ref}`12 V rail injection <power-rail-incident>` - an oscilloscope probe
  bridged TB6612 `VM` and `VCC`, destroying multiple low-voltage devices.
- Raspberry Pi GPIO is not 5 V tolerant; the PS2 receiver uses 3.3 V logic.
- The TJA1050 module still requires its documented 5 V supply for valid CAN
  transceiver levels even though the MCP2515 logic side is lower voltage.
- High- and low-voltage harnesses must be physically separated before probing.

## Firmware and transport

- {ref}`CAN driver and parser failures <can-stack-failures>` - incorrect
  `TXBnCTRL` addressing, missing `can_frame` definitions, CMake regeneration,
  and Python capture-pattern behavior.
- CubeMX failed to persist TIM3 `ENCODER_MODE_TI12`; the `.ioc` required an
  explicit text correction or one encoder would have one-quarter resolution.
- The ST VS Code extension may report a regenerated project as corrupted;
  opening a clean CubeMX-generated project is the recorded workaround.
- Raspberry Pi Wi-Fi may reach `UP` without a DHCP IPv4 address. CAN and GPIO
  testing remain independent of that failure.

## Motion and control

- {ref}`Rear encoder harness swap <encoder-map-failure>` - forward motion was
  correct while lateral and angular channels exchanged roles.
- {ref}`PID positive-feedback runaway <pid-runaway>` - all raw encoder signs
  opposed commanded forward motion.
- {ref}`Heartbeat stop overwritten <heartbeat-overwrite>` - normal control
  restored PWM immediately after the stop call.
- {ref}`Missing 45-degree projection <translation-scale-failure>` - both
  translation axes measured low while rotation scale remained plausible.
- {ref}`REP-103 frame mirror <frame-mirror-failure>` - lateral and angular
  outputs required an output-frame correction, not retuned encoder signs.
- STM32 still uses theoretical CPR 2800 for the disabled PID branch while ROS
  2 odometry uses calibrated CPR 2779. The mismatch is intentional and should
  not be "fixed" without revisiting that decision.
- Open-loop motor mismatch can create physical yaw during nominal translation;
  ground-truth analysis must separate this behavior from odometry math errors.

## Sensing and localization

- {ref}`Ultrasonic invalid readings <ultrasonic-invalids>` - invalid and
  saturated returns remain explicit mixture-model cases.
- {ref}`MCL design corrections <mcl-design-corrections>` - holonomic motion,
  heading diversity, observability, and ground-truth comparison constraints.

## Documentation rule

Expanded chapters should retain the full evidence chain:
**symptom -> competing hypotheses -> discriminating test -> root cause ->
repair -> regression test**. Removing the failed path would make the hardware
reproduction less reliable.
