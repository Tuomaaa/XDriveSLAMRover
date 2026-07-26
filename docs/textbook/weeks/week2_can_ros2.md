# Week 2 - STM32, CAN, and ROS 2 Control Pipeline

:::{admonition} Chapter status: Outline
:class: status status-outline

The core transport has been exercised between real hardware endpoints. Several
ROS 2 node features remain on the project checklist, so this chapter does not
claim the complete bridge is finished.
:::

## Objective

Create a deterministic boundary between hard real-time motor and sensor I/O on
the STM32 and estimation software on the Raspberry Pi.

## Prerequisites

- A powered Week 1 rover with a verified emergency stop
- STM32CubeMX/CMake firmware build
- Raspberry Pi 4B with Ubuntu Server 24.04 and ROS 2 Jazzy
- MCP2515 controllers configured for 500 kbit/s

## Planned chapter sections

- Why the STM32 owns motor timing and heartbeat safety
- CAN identifier categories, payloads, and arbitration priorities
- MCP2515 SPI setup on STM32 and Raspberry Pi
- Stateless Python encode/decode boundary
- SocketCAN and ROS 2 message flow
- Hardware-in-the-loop tests and timeout behavior

## Interface snapshot

| CAN ID | Direction | Payload | Role |
| --- | --- | --- | --- |
| `0x100` | RPi to STM32 | 4 x `int16` RPM | Velocity command |
| `0x200`, `0x201` | STM32 to RPi | 2 x `int32` ticks | Encoder feedback |
| `0x202` | STM32 to RPi | ranges + status | Ultrasonic feedback |
| `0x300` | RPi to STM32 | empty or sequence | Heartbeat |
| `0x400`-`0x402` | RPi to STM32 | motor + float | PID tuning |

(can-stack-failures)=
## Failure records retained for expansion

- A vendor MCP2515 driver passed an RTS instruction byte as a register address,
  causing transmit failures. The repair reads the real `TXBnCTRL` address.
- The same library omitted Linux-compatible `can_frame` definitions, requiring
  a local C header with the expected flags and typedefs.
- CubeMX regeneration overwrote manually added CMake source entries; the build
  checklist must verify the MCP2515 source after regeneration.
- A Python `match` statement using a bare constant behaved as a capture pattern.
  The protocol decoder uses explicit `if`/`elif` comparisons instead.
- The Raspberry Pi GPIO is not 5 V tolerant. The PS2 receiver is powered at
  3.3 V, while the TJA1050 CAN transceiver module requires its documented 5 V
  supply for valid bus levels.

## Source and evidence

- [`protocol.py`](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/protocol.py)
- [`can_bridge_node.py`](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/can_bridge_node.py)
- [`mcp2515.c`](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/stm32/Core/Src/mcp2515.c)

## Verification state

SocketCAN loopback and real STM32-to-RPi CAN communication are verified. The
project checklist still tracks ROS 2 command, error, ACK, and tuning behavior;
the expanded chapter must test each path rather than infer completion from the
working encoder path.
