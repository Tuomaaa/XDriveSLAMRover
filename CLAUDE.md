# SummerSLAM — X-Drive SLAM Rover

12-week summer research project. Building a hardware-backed holonomic robot to study SLAM: odometry → MCL → EKF-SLAM → visual odometry → sensor fusion.

**Read PROJECT\_STATE.md before making any changes** — it has the current checklist, decision log, known issues, and next steps. Update it at the end of any session with substantive progress.

## Repo layout

* `stm32/` — STM32 CubeMX project (C, CMake via ST VS Code extension). Flash via ST-Link.
* `ros2\_ws/src/can\_bridge/` — main ROS2 Jazzy package (Python). Runs on Raspberry Pi 4B (Ubuntu Server 24.04).
* `PROJECT\_STATE.md` — single source of truth for project status, decisions, and known pitfalls.

## Hardware quick ref

* X-drive: 4x 50mm omni wheels at 45 deg, R=115mm center-to-wheel, r=25mm wheel radius
* Motors: GA12-N20, 7 PPR, 100:1 gear, theoretical CPR=2800 (uncalibrated)
* MCU: STM32F411CEU6, CAN via MCP2515+TJA1050, 500kbps
* SBC: RPi 4B, CAN HAT (MCP2515, 12MHz), PS2 wireless controller

## Motor mapping (must stay in sync everywhere)

motor 0=FL(TIM2), 1=FR(TIM3), 2=RL(TIM4), 3=RR(TIM5)

Enforced by `MotorPosition` enum in `main.c` and `MOTOR\_MAP` in `odometry.py`.

## Deploy to Pi

Pi uses sparse-checkout of this repo. Do not edit code on Pi directly.
Local edit → git push → ssh into Pi → git pull → colcon build → test.

## Style

Chinese responses, English technical terms preserved. Prefer Socratic guidance over handing out answers.

