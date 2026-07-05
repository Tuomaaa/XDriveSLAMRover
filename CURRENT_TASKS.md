# Week 3-4 当前任务

目标：完成 encoder odometry 的硬件验证，然后进入 drift 测量实验。

## 背景

`odometry.py` 已经写好（forward kinematics + midpoint pose integration），smoke test 通过。但所有参数都还是理论值，encoder 方向也没验证过。这次要在实机上把这些参数定下来，确认 odometry 输出跟真实物理运动一致，然后跑重复轨迹收集 drift 数据。

## 任务（按顺序）

### 1. 心跳问题

STM32 固件有 heartbeat timeout（200ms 无 0x300 帧则停机）。当前 `ps2_drive_test.py` 如果没有单独起一个线程/定时器定期发 0x300，电机不会响应任何速度指令。检查 `ps2_drive_test.py` 是否已包含心跳逻辑，如果没有则加上（100ms 周期发送 0x300，DLC=0）。

### 2. Encoder 方向标定

用 `ps2_drive_test.py` 分别执行：
- 纯 vy 运动（只推左摇杆 X 轴）：观察四路 encoder tick 的符号，对照 `odometry.py` 里 inverse kinematics 的预期（fl=+, fr=-, rl=-, rr=+）
- 纯 omega 运动（只推右摇杆 X 轴）：同理检查

如果某路 encoder 符号跟预期相反，更新 `odometry.py` 里对应的 `ENCODER_SIGN` 为 -1。

需要一个脚本或 `ps2_drive_test.py` 的模式来实时打印四路 encoder 的 raw tick 值（从 CAN 0x200/0x201 读取），方便观察。

### 3. ENCODER_CPR 实测标定

手动转某一个轮子的输出轴精确 10 圈，记录 encoder CNT 差值。实测 CPR = delta_ticks / 10。对四个电机都做一遍（可以不完全一样）。用实测平均值替换 `odometry.py` 和 `firmware/Core/Src/main.c` 里的 `ENCODER_CPR` 宏。

### 4. Ground truth sanity check

用 `ps2_drive_test.py` 手动驾驶机器人走：
- 正方形轨迹（边长约 0.5m），回到起点
- 原地转 360 度

用卷尺量实际终点偏差，对比 `odometry.py` 输出的 (x, y, theta)。这一步目的是验证 kinematics 代码本身没 bug，不是在测 drift——如果偏差离谱（比如方向完全反了），说明 ENCODER_SIGN 或 MOTOR_MAP 还有错，不要急着进 Week4。

### 5. 集成到 can_bridge_node.py

确认 1-4 都没问题后，把 `OdometryEstimator` 接入 `can_bridge_node.py`：
- 收到 0x200+0x201 后调用 `odo.update()`
- 发布 `nav_msgs/Odometry` topic
- 发布 odom→base_link TF

### 6. Week4 drift 实验（前置条件：1-5 全部完成）

设计重复轨迹（直线 1m 来回、正方形、原地转 N 圈），跑 10+ 次，记录每次 odometry 输出的终点 pose。统计 translational drift 和 heading drift 的分布，作为 motion uncertainty 的量化依据。

## 注意

- `ENCODER_CPR=2800` 是理论值，做完任务 3 之前不要拿 drift 数据下结论
- dt 要用 CAN 帧实际到达时间戳算，不要假设固定 20ms
- 所有代码改动完成后更新 PROJECT_STATE.md
