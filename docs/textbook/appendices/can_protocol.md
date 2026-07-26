# CAN Protocol Reference

:::{admonition} Appendix status: Initial reference
:class: status status-outline

This table mirrors the protocol decisions currently recorded in the repository.
The source code remains authoritative until automated protocol-table generation
is introduced.
:::

## Identifier categories

| Priority | Category | Range | Direction | Nominal timing |
| --- | --- | --- | --- | --- |
| 0 | Error | `0x000`-`0x0FF` | STM32 to RPi | Event driven |
| 1 | Velocity command | `0x100` | RPi to STM32 | 50-100 ms |
| 2 | Encoder/range feedback | `0x200`-`0x202` | STM32 to RPi | 20 ms control cycle |
| 3 | Heartbeat | `0x300` | RPi to STM32 | 100 ms |
| 4 | PID tuning | `0x400`-`0x402` | RPi to STM32 | Event driven |
| 5 | ACK/response | `0x500`-`0x5FF` | STM32 to RPi | Event driven |

All multibyte numeric payloads are little-endian. The heartbeat timeout is
200 ms and must gate the entire motor-control update.

## Motor index contract

Motor commands use `0=FL`, `1=FR`, `2=RL`, `3=RR`. Encoder wiring is a separate
physical mapping: the current rear encoder channels are swapped, so estimation
maps encoder index 2 to rear-right and index 3 to rear-left. See
{ref}`encoder-map-failure` before changing either mapping.

## Sources

- [`protocol.py`](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/ros2_ws/src/protocol.py)
- [`main.c`](https://github.com/Tuomaaa/XDriveSLAMRover/blob/main/stm32/Core/Src/main.c)
