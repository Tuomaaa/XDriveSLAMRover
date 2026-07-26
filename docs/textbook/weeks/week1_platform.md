# Week 1 - Rover Platform and Hardware Bring-up

:::{admonition} Chapter status: Outline
:class: status status-outline

The platform exists and its core drive hardware has been exercised. This page
indexes the evidence and safety decisions; the step-by-step build narrative is
the first chapter scheduled for expansion.
:::

## Objective

Build a four-wheel X-drive base whose power, motor direction, encoder wiring,
and emergency-stop path are understood before estimation software is added.

## Prerequisites

- Safe bench practices for mixed 12 V, 5 V, and 3.3 V systems
- An STM32F411CEU6 Black Pill and ST-Link
- Two TB6612 dual motor drivers
- Four GA12-N20 motors with quadrature encoders
- Four 50 mm omni wheels mounted at 45 degrees

## Platform constants

| Property | Value |
| --- | --- |
| Wheel radius | 25 mm |
| Center-to-wheel distance | 115 mm |
| Encoder | 7 PPR, 100:1 gearbox |
| Calibrated encoder CPR | 2779 counts per output revolution |
| Drive geometry | Four-wheel holonomic X-drive |

## Planned chapter sections

- Mechanical X-drive geometry and wheel orientation
- Power tree and common-ground rules
- TB6612 wiring and motor-direction checks
- Encoder and motor harness identification
- Bring-up sequence with current-limited power
- Emergency-stop verification before free driving

(power-rail-incident)=
## Failure record: 12 V injected into the 3.3 V rail

During an oscilloscope measurement on 2026-06-08, the probe bridged adjacent
TB6612 `VM` (12 V) and `VCC` (3.3 V) pins. The 12 V rail propagated through the
shared low-voltage rail and destroyed the STM32 board, one TB6612 module, and
the connected ST-Link. The other TB6612 and the MCP2515/TJA1050 module survived
subsequent checks.

The repair was hardware replacement plus a stricter measurement sequence:

1. Disconnect the high-voltage supply before moving a probe.
2. Route high- and low-voltage wiring in physically separate groups.
3. Attach the probe ground before contacting the signal pin.
4. Reintroduce modules one at a time and check rails for shorts first.

This event is retained because a reproducible hardware build must document how
the system can fail, not only the final wiring diagram.

## Source and evidence

- [`stm32/SummerSLAM.ioc`](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/stm32/SummerSLAM.ioc)
- [`stm32/Core/Src/main.c`](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/stm32/Core/Src/main.c)
- [Current hardware state and wiring notes](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/PROJECT_STATE.md)

## Verification state

PWM drive, encoder capture, the 500 kbit/s CAN link, and the full rover power
scheme have hardware evidence. A polished BOM, mechanical drawing, and a
repeatable photographic assembly sequence are not yet part of this chapter.
