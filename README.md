# SummerSLAM

SummerSLAM is a 12-week, hardware-backed study of mobile-robot localization
and SLAM on a four-wheel X-drive rover. The current public textbook covers the
first six weeks, from hardware bring-up through Monte Carlo Localization.

**Textbook:** <https://tuomaaa.github.io/XDriveSLAMRover/>

## Repository map

- `stm32/` - STM32F411 firmware, motor control, encoders, CAN, and ultrasonic sensing
- `ros2_ws/src/` - ROS 2 Jazzy CAN bridge, odometry, measurement models, and localization tools
- `data/` - calibration and validation data
- `docs/textbook/` - Sphinx/MyST source for the Week 1-6 textbook
- `PROJECT_STATE.md` - current status, decisions, known issues, and next steps

## Licensing

Original software in this repository is licensed under the [MIT License](LICENSE).
Original textbook content and media are licensed under
[CC BY 4.0](LICENSE-DOCS). Third-party components retain their own licenses.
