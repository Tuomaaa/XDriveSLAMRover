# SummerSLAM — Project State

> 最后更新：2026-07-04
> 当前阶段：Week 3 → Week 4 过渡 — odometry.py 已标定（ENCODER_SIGN 全 -1，ENCODER_CPR 2779），`can_bridge_node.py` 已接入 `OdometryEstimator` 并发布 odom+TF。发现 STM32 侧 PID runaway（encoder 反向→正反馈），已改为 open-loop（`USE_PID=0`，PID 保留待用），并修复 heartbeat 急停失效。**下一步：重新烧录 STM32 验证不再 runaway → 再做 Ground truth sanity check（正方形 + 原地转 360°）验证 vy/omega 方向 → Week4 drift**

---

## 当前状态

### RPi 系统
- [x] Ubuntu Server 24.04 LTS 已装好
- [x] SSH / 基础配置
- [x] SPI enable
- [x] CAN HAT overlay (Waveshare RS485 CAN HAT, MCP2515, SPI0 CE0, INT GPIO25, 12MHz)
- [x] SocketCAN 验通 (loopback mode, 500kbps)
- [x] ROS2 Jazzy 安装

### PS2 手柄遥控（进行中）
- [x] PS2 协议原理分析（SPI-like, LSB-first, ~25kHz clock）
- [x] RPi 侧 GPIO bit-bang 驱动编写 (ps2_controller.py, /dev/mem + mmap, 零依赖)
- [ ] 驱动实测验证（接收器接线 + analog mode 读取）
- [ ] 摇杆数据 → cmd_vel topic 映射
- 接线：GPIO17→CLK, GPIO27→CMD, GPIO22→DAT, GPIO23→CS, 3.3V供电

### STM32 侧 — CubeMX 配置完成 ✅ / Build 通过 ✅ / 硬件替换完成 ✅
MCU: STM32F411CEU6 (Black Pill)，HSE 25MHz → PLL (M=25, N=200, P=2) → SYSCLK 100MHz。

外设分配：
- **PWM (电机速度)**：TIM1，4 channel (PA8/PA9/PA10/PA11)，Period=4999 → 20kHz
- **Encoder (4 路)**：全部 TIM_ENCODERMODE_TI12，Period 用默认最大值
  - TIM2 (PA15/PB3) — 32-bit, Period=4294967295
  - TIM3 (PB4/PA7) — 16-bit, Period=65535
  - TIM4 (PB6/PB7) — 16-bit, Period=65535
  - TIM5 (PA0/PA1) — 32-bit, Period=4294967295
- **SPI1 (→ MCP2515)**：Master, Mode 0 (CPOL=Low, CPHA=1Edge), 6.25 MBits/s
  - SCK=PA5, MISO=PA6, MOSI=PB5, CS=PB9 (软件控制 GPIO)
- **MCP2515 INT**：PB8, EXTI line 8, falling edge + pull-up, EXTI9_5 IRQ 已 enable
- **USART2 (调试)**：PA2/PA3, 115200 8N1, parity NONE
- **电机方向 GPIO**：PB0/PB1/PB2/PB10/PB12/PB13/PB14/PB15 (8× output push-pull)
- **SWD**：PA13/PA14

Toolchain: CMake (配合 ST 官方 STM32CubeIDE for VS Code extension)。
Build 验证：25 个文件编译+链接通过，RAM 2104B/128KB (1.61%), FLASH 13400B/512KB (2.56%)。

### TB6612 接线（已验证）

TB6612 #1:
- **B 通道 (电机 1)**：PWMB ← PA8 (TIM1_CH1), BIN1 ← PB1 (Motor1_2), BIN2 ← PB0 (Motor1_1)
- **A 通道 (电机 2)**：PWMA ← PA9 (TIM1_CH2), AIN1 ← PB10 (Motor2_2), AIN2 ← PB2 (Motor2_1)
- **电源**：STBY ← 3.3V (常高), VCC ← 3.3V, VM ← 12V (LiPo 直供), GND ← 共地

注意：label 和 TB6612 pin 命名有错位（Motor1_1=BIN2, Motor1_2=BIN1），功能不影响但代码里注意对应关系。

### 硬件事故记录 ⚠️ (2026-06-08)

**事故原因**：用示波器测量 TB6612 VM (12V) 信号时，探针意外同时碰到 VM 和 VCC pin，导致 12V 灌入 3.3V rail。

**传播路径**：12V → VCC (3.3V rail) → 所有共享 3.3V rail 的模块 → STM32 和 TB6612 #2 内部过压击穿。

**损失清单**：
- Black Pill (STM32F411CEU6) — **烧毁** ✗ (5V/3.3V/GND 全短路，多个 GPIO 与 GND 短路)
- TB6612 #2（未直接碰触）— **烧毁** ✗ (VM-GND 短路)
- ST-Link V2 clone — **烧毁** ✗ (灯不亮，12V 通过 SWD 反灌)
- TB6612 #1（探针碰的那块）— **存活** ✓ (12V 通过探针外部旁路分流，芯片内部未承受大电压差)
- MCP2515+TJA1050 模块 — **存活** ✓ (SPI 通信 + loopback 收发验证通过)

**需要补购**：
- Black Pill STM32F411CEU6 × 2（备一块）
- TB6612 模块 × 1
- ST-Link V2 clone × 1

**教训**：
1. 测量高压侧信号前，先断开高压电源
2. 高压 (12V) 和低压 (3.3V/5V) 走线物理隔开，不要混在同一排排针上
3. 探针 GND 夹先夹好，再去碰信号线

### CAN 协议（已定 ✅）

ID 编码：高 3 bits 类别（兼 arbitration 优先级），低 8 bits 子分类。

```
优先级  类别         高3bits   ID范围        方向           频率
──────────────────────────────────────────────────────────────────
0(最高) Error        0b000    0x000~0x0FF   STM32→RPi     event-triggered
1       速度指令      0b001    0x100         RPi→STM32     50~100ms
2       Encoder反馈   0b010    0x200, 0x201  STM32→RPi     20ms (50Hz)
3       Heartbeat    0b011    0x300         RPi→STM32     100ms
4       PID调参       0b100    0x400~0x402   RPi→STM32     event-triggered
5(最低) ACK/Response  0b101    0x5XX         STM32→RPi     event-triggered
```

Payload 格式：
- 0x100 速度指令：4×int16_t 目标RPM (±300RPM, little-endian), 8 bytes
- 0x200 Encoder Frame 0：Motor 0 + Motor 1 各 int32_t 累计ticks, 8 bytes
- 0x201 Encoder Frame 1：Motor 2 + Motor 3 各 int32_t 累计ticks, 8 bytes
- 0x300 Heartbeat：DLC=0 或可选 sequence counter
- 0x400/401/402 PID Kp/Ki/Kd：byte0=motor_index(0~3), byte1+=参数值
- 0x5XX ACK：byte0=原始命令sub-ID, byte1+=可选回传数据

安全机制：
- Heartbeat 100ms 周期，timeout 200ms，超时后 STM32 所有电机 PWM=0
- Error 每种独立 ID，具体 error code 待实现阶段定义
- ACK 提供应用层确认（CAN link-layer ACK ≠ application-layer ACK）

### STM32 固件（进行中）
- [x] CubeMX generate code 后的工程骨架在 VS Code 里 build 通过
- [x] 电机基础驱动验证 (PWM + 方向 GPIO → TB6612 → 电机转)
- [x] Encoder 读取 (TIM2 encoder start + CNT → CAN 0x200 验证通过)
- [x] PWM 输出 + 电机方向 GPIO 控制
- [x] 电机控制：**当前用 open-loop**（`main.c` 的 `#define USE_PID 0`）——摇杆 target RPM 直接映射 PWM duty。**待重新烧录验证**
  - **决策 (2026-07-04)**：遥控 + odometry 阶段不需要闭环，PID 对 odometry 无帮助（odometry 直接读 encoder），且闭环会抹平 Week4 要测的 motion uncertainty。open-loop 还天然免疫下面那个 runaway。
- [x] PID 速度环 (20ms 周期, Kp=8/Ki=2/Kd=0.1)：**代码保留在 `USE_PID` 分支里，当前关闭**，以后 cmd_vel 自主驾驶时翻成 1 恢复
  - **runaway 根因 + 符号修正 (2026-07-04)**：四路 encoder 全反，PID 若用 raw 反向计数当反馈 → 正反馈 runaway（上电即全速、摇杆无效）。已加 `ENC_SIGN[4]={-1,-1,-1,-1}`（仅作用于 PID 反馈，CAN raw ticks 不变，RPi `ENCODER_SIGN=-1` 不用动），随 PID 一起留在 `#if USE_PID` 内。恢复闭环前务必确认 `USE_PID=1` 下不再 runaway
- [x] MCP2515 SPI 驱动 + CAN loopback 收发验证 (thumptech/STM32-MCP2515 库，已修 bug)
- [x] STM32 → RPi 真实 CAN 通信验证 (500kbps, MCP2515 模块需 5V 供电)
- [x] Heartbeat timeout 安全停机 (200ms 内无 0x300 心跳则电机 PWM 清零)
  - **急停失效 bug 修复 (2026-07-04)**：原来 `motors_stop()` 清零后紧随的控制块每 20ms 又把 PWM 顶回去，急停形同虚设。改成用 `hb_ok` 门控整个控制更新（open-loop 与 PID 两种模式都生效），超时时电机保持 0；PID 模式下还清 integral 防 windup。**待重新烧录验证**
- [x] TIM3/TIM4 16-bit encoder overflow 处理 (软件扩展到 int32，与 CAN 协议累计 tick 对齐)

### ROS2 CAN Node（进行中）
- [x] CAN 协议 encode/decode 层 (protocol.py)
- [x] SocketCAN interface 封装 (can_interface.py, python-can)
- [x] can_bridge_node.py 设计完成（代码在 chat 中，未部署到 RPi）
- [ ] Heartbeat publisher (100ms)
- [ ] 速度指令 subscriber (topic → CAN 0x100)
  - cmd_vel → 4路RPM 换算暂留 placeholder
- [ ] Encoder 反馈 listener (CAN 0x200/0x201 → topic)
  - Odometry 计算：kinematics 已定，odometry.py 已写好，待接入 node 并发布 nav_msgs/Odometry + TF
- [ ] Error listener (0x0XX)
- [ ] ACK listener (0x5XX)
- [ ] PID 调参 service (0x400~0x402)

### Week3 Kinematics & Odometry（进行中）

- [x] Inverse kinematics 方程确认，对齐 `ps2_drive_test.py` 的实现：
  `fl = vx+vy+ωR`, `fr = vx-vy-ωR`, `rl = vx-vy+ωR`, `rr = vx+vy-ωR`
- [x] Forward kinematics 消元推导完成：
  `vx = (fl+fr+rl+rr)/4`，`vy = (fl-fr+rr-rl)/4`，`ω = (fl-fr-rr+rl)/(4R)`
- [x] Pose integration 用 midpoint method 实现（`θ_mid = θ + ωdt/2`），而非 plain Euler
- [x] `odometry.py` 模块编写完成，独立于 CAN/ROS，`MOTOR_MAP`/`ENCODER_SIGN`/`ENCODER_CPR` 显式暴露为配置，smoke test（纯 vy 输入 → vx=0, ω=0）通过
- [x] Motor index ↔ 物理轮位映射确认并落地：`main.c` 新增 `MotorPosition` enum (`MOTOR_FL=0, MOTOR_FR=1, MOTOR_RL=2, MOTOR_RR=3`)，`motors[]` 初始化、CAN 收发全部改用 enum 索引，跟 `odometry.py` 的 `MOTOR_MAP` 对齐
- [x] `encoder_monitor.py` 编写完成：实时打印四路 encoder tick（从 baseline 起的 delta），配合 `ps2_drive_test.py` 在第二个终端跑，供 Encoder 方向标定用
- [x] Encoder 方向标定（2026-07-04 实机）：直线前进时四路 encoder tick 全部递减，`ENCODER_SIGN` 四路全设 -1。**注意**：纯 vx 前进测试已完全确定四路符号，vy/omega 方向正确性只能靠 Ground truth test 验证，无法再靠 `ENCODER_SIGN` 修
- [x] `ENCODER_CPR` 实机标定（2026-07-04）：四轮各手转 10 圈，FL 2779.5 / FR 2778.3 / RL 2778.7 / RR 2777.6，平均 2778.5，取整 **2779**（理论值 2800 偏高 ~0.8%，轮间极差仅 0.07%）。已更新 `odometry.py`；`main.c` 的 `#define ENCODER_CPR` 暂留 2800（只影响 PID RPM 反馈精度 ~0.8%，是否为此重烧待定）
- [ ] Ground truth sanity check：正方形轨迹 + 原地转 360°，卷尺量实际终点位置对比代码输出（**待实机**，用 `can_bridge_node.py` 的 pose 日志观察）
- [x] `can_bridge_node.py` 接入 `OdometryEstimator`（2026-07-04）：收齐 0x200+0x201 后调 `update()`（dt 用 CAN 帧时间戳），发布 `nav_msgs/Odometry` on `odom` + `odom→base_link` TF；另加 ~2Hz 节流 pose 日志供 ground-truth 观察。import 改成平铺式（跟全项目一致），`python3 can_bridge_node.py` 直接跑。**代码完成，待实机验证**

---

## 决策记录

| 日期 | 决策 | 理由 |
|------|------|------|
| 2026-06-04 | Ubuntu Server 24.04 而非 RPi OS | ROS2 Jazzy 官方支持，apt 直装 |
| 2026-06-04 | Loopback mode 先行开发 | 无 STM32，MCP2515 自发自收测试 |
| 2026-06-04 | Kinematics 后做 | 先搭 CAN+ROS2 管道，底盘构型独立 |
| 2026-06-04 | 速度指令发目标 RPM 而非 PWM duty | 保证 STM32 侧是闭环 PID 速度控制 |
| 2026-06-04 | Encoder 发累计 ticks 而非 delta | 累计值 idempotent，丢帧不累积误差 |
| 2026-06-04 | Encoder 按电机拆帧而非按字节高低拆 | 每帧 self-contained，parsing 简单 |
| 2026-06-04 | PID Kp/Ki/Kd 每参数一帧 | 调参是低频事件，代码简洁性 > 帧效率 |
| 2026-06-04 | ACK 单独开 0x5XX 类别 | 与调参命令方向相反，混用同 ID 范围会导致 filter 混乱 |
| 2026-06-04 | Heartbeat timeout 200ms (2× 周期) | CAN 可靠性高，2× 已足够；bus traffic 轻 |
| 2026-06-07 | Toolchain 选 CMake | 配合 ST 官方 VS Code extension（基于 CMake 构建系统）|
| 2026-06-07 | 4 路 encoder 统一 TIM_ENCODERMODE_TI12 | 两路边沿都计数，4× 分辨率；四电机一致才能正确算 odometry |
| 2026-06-07 | TIM3 EncoderMode 手动编辑 .ioc 修复 | CubeMX bug：pin 配了 Encoder_Interface 但 timer 参数没写入，GUI 反复操作无效 |
| 2026-06-08 | 电机驱动测试优先于 CAN 通信 | 先验证最底层硬件（PWM+TB6612+电机），再搭通信管道；出问题好定位 |
| 2026-06-12 | protocol.py 为 stateless encode/decode 层 | 不持有状态、不做 buffer，纯翻译器；状态管理交给 can_bridge_node |
| 2026-06-12 | Category 常量（ERROR/ACK/MASK）不放入 MsgId IntEnum | 它们是 bitmask 不是 message ID，混入 IntEnum 会导致 MsgId(0x000) 等误匹配 |
| 2026-06-12 | decode() 用 if/elif 而非 match/case | Python match 对非 dotted 名称会当 capture pattern，bare constant 不做比较 |
| 2026-06-12 | can_interface.py 薄封装 python-can | 只管 bus 生命周期和收发 raw frame，不碰协议逻辑或 ROS2；channel 参数支持 vcan0 测试 |
| 2026-06-13 | MCP2515 驱动用 thumptech/STM32-MCP2515 库 | STM32F4 HAL 兼容，API 干净，只需改 SPI handle + CS pin + 补 can.h |
| 2026-06-13 | MCP2515 模块晶振 8MHz | 模块实测标 "8.000"，初始化用 MCP_setBitrateClock(CAN_500KBPS, MCP_8MHZ) |
| 2026-06-13 | MCP2515+TJA1050 模块供 5V | TJA1050 需 5V 才能输出正确 CAN 电平 (~2.5V idle)；模块有板载 LDO 给 MCP2515 降压 3.3V，SPI 逻辑电平不受影响 |
| 2026-06-14 | PS2 手柄接 RPi 而非 STM32 | STM32 GPIO 已全部用完（PWM×4 + Encoder×8 + SPI+CS+INT + UART + Direction×8 + SWD），RPi 有空闲 GPIO |
| 2026-06-14 | PS2 驱动用 /dev/mem + mmap 而非 RPi.GPIO | RPi 网络不通无法 apt install，mmap 直接操作 BCM2711 寄存器，零外部依赖 |
| 2026-06-14 | PS2 接收器供 3.3V | RPi GPIO 不是 5V tolerant（不像 STM32 F411），PS2 手柄 spec 支持 3V~5V |
| 2026-06-14 | RPi 供电走 GPIO Pin 2 (5V) 而非 USB-C | UBEC 5V 直供，避免杜邦线→USB-C 的压降和接触问题；注意绕过了 polyfuse |
| 2026-07-03 | Motor index 0/1/2/3 直接对应 fl/fr/rl/rr | 跟 `ps2_drive_test.py` 里已验证过的 inverse kinematics 顺序保持一致，不引入第二套编号 |
| 2026-07-03 | dt 用 CAN 帧实际到达时间戳算，而非固定 20ms | 避免 CAN bus jitter / RPi 调度延迟被当成系统性速度偏差，混进 Week4 要测的 motion uncertainty 里 |
| 2026-07-03 | Pose integration 用 midpoint method (`θ_mid = θ + ωdt/2`) | Plain Euler 在原地快速转弯时位移方向系统性偏差；heading error 比 translational error 更致命 |
| 2026-07-03 | `ENCODER_CPR` 暂用理论值 2800，标定列为 TODO 而非阻塞项 | 先把 kinematics 代码跑通，标定可并行/稍后做，但显式记录，防止被误当成已标定值直接用于 Week4 |
| 2026-07-03 | 确认 motor index 物理映射：TIM2(idx0)=FL, TIM3(idx1)=FR, TIM4(idx2)=RL, TIM5(idx3)=RR | review `main.c` 时发现物理映射从未显式写出，只存在于接线事实里；补了 `MotorPosition` enum + 注释，把 main.c、`odometry.py` 的 `MOTOR_MAP`、`ps2_drive_test.py` 的顺序统一成同一份 single source of truth |
| 2026-07-04 | `ENCODER_SIGN` 四路全设 -1 | 实机直线前进时四路 encoder tick 全部递减；纯 vx 前进测试对每路符号是完全约束（每轮必须前进=正），四路一致翻负即可。副作用：vy/omega 也随之翻号，其方向正确性无法再靠 sign 修，只能靠 ground-truth test 验证 |
| 2026-07-04 | `ENCODER_CPR` 用实测 2779 替换理论 2800 | 四轮各手转 10 圈实测：FL 2779.5/FR 2778.3/RL 2778.7/RR 2777.6，均值 2778.5 取整。轮间极差仅 0.07%，用单一全局值足够；理论值偏高 0.8% 属 N20 减速箱正常 |
| 2026-07-04 | `main.c` 的 `ENCODER_CPR` 暂不同步改（留 2800） | STM32 侧该宏只用于 PID 的 RPM 反馈换算，0.8% 误差对速度环无实际影响；改它要再烧一次录，收益不值，待有其他固件改动时顺带更新 |
| 2026-07-04 | odometry 触发时机：收到 0x201（完成 0/1/2/3 全套）时调 update() | STM32 每 20ms 先发 0x200 再发 0x201，以 0x201 为"一组完整"信号最简单；用该帧 `msg.timestamp` 算 dt。若丢 0x200 会把上一帧的 FL/FR 和新 RL/RR 配对，但累计 tick idempotent，单帧误配影响极小，可接受 |
| 2026-07-04 | `can_bridge_node.py` 用平铺 import + `python3` 直跑，不建 colcon 包 | 全项目其它脚本都是平铺 import + 裸跑，无 `package.xml`/`setup.py`；为一个 node 单独搭 package 结构收益低。ground-truth 阶段靠 node 自带 2Hz pose 日志观察，不需要 `ros2 launch` |
| 2026-07-04 | 电机控制先用 open-loop（`USE_PID=0`），PID 保留待 cmd_vel 阶段 | 遥控 + odometry 标定/drift 阶段不需要速度闭环；PID 对 odometry 无帮助（直接读 encoder），且闭环会补偿掉 Week4 要测量的 motion uncertainty。open-loop 也天然免疫 encoder 反向导致的 PID 正反馈 runaway。PID 代码用 `#if USE_PID` 完整保留，翻成 1 即恢复 |
| 2026-07-04 | 急停门控从"清零后被覆盖"改为 `hb_ok` 门控整个控制更新 | 原逻辑 `motors_stop()` 之后控制块又立即重驱 PWM，导致 heartbeat 急停失效（断心跳也停不下来）。现在心跳超时时整个控制更新短路，电机保持 0；encoder_update 仍每 20ms 跑以维持 CAN ticks 与 16-bit overflow 追踪 |

---

## 已知坑 / Workaround

- **CubeMX TIM3 encoder mode 不写入**：pin SH 配置正确 (Encoder_Interface) 但 `TIM3.EncoderMode` 行不生成，GUI 怎么切都没用。默认值是 TIM_ENCODERMODE_TI1（只单路计数），会导致该轮分辨率只有其他轮的 1/4。Workaround：文本编辑器直接在 .ioc 加 `TIM3.EncoderMode=TIM_ENCODERMODE_TI12` 和 `TIM3.IPParameters=EncoderMode`，再 regenerate。
- **Encoder Period 默认值是最大值不是 0**：CubeMX GUI 默认就填了 16-bit=65535 / 32-bit=4294967295，.ioc 里没显式写出来只是因为没改过默认值，generate code 用的是 GUI 显示值。无需手动改。
- **ST VS Code extension import 可能报 "project corrupted"**：clean project 从 CubeMX 重新 generate code 后直接打开可以绕过。
- **12V 和 3.3V 共 rail 风险**：TB6612 的 VM (12V) 和 VCC (3.3V) 引脚相邻，探针误触可烧毁 3.3V rail 上所有模块。测量高压前必须断开高压电源。
- **Python match/case capture pattern 陷阱**：`case ENCODER_0:` 里的 bare name 会被当成新变量捕获任意值，不会比较常量。要么用 dotted name `case MsgId.ENCODER_0:`，要么用 if/elif 代替。
- **thumptech/STM32-MCP2515 库 sendMessageTo bug**：`readRegister(rts_addr)` 用 RTS 指令码 (0x81) 当寄存器地址读，读到垃圾值导致 ERROR_FAILTX。修复：改成 `readRegister((txbn + 3) << 4)` 读真正的 TXBnCTRL (0x30/0x40/0x50)。
- **thumptech/STM32-MCP2515 缺 can.h**：仓库没提供 `can_frame` 定义和 Linux SocketCAN 常量（CAN_EFF_FLAG 等），需自建 can.h，typedef struct 而非 struct（C 兼容）。
- **CMake 手动添加源文件**：CubeMX regenerate 会覆盖 CMakeLists.txt，手动加的 mcp2515.c 条目会丢失，需重新添加。
- **RPi WiFi DHCP 不分配 IPv4**：netplan 配置正确，wlan0 状态 UP 但无 IPv4 地址，只有 IPv6 link-local。`netplan apply` 后偶尔恢复。SSH 通过 IPv6 或缓存 session 维持。不影响 CAN / GPIO 开发。
- **RPi 不是 5V tolerant**：RPi GPIO 输入最大 3.3V，不像 STM32 F411 有 FT pin。PS2 接收器、任何外设模块的 DATA 线输出如果是 5V 会烧 RPi GPIO。
- ~~**`ENCODER_CPR=2800` 是理论计算值**~~ **已标定 (2026-07-04)**：`odometry.py` 用实测 2779（四轮均值）。注意 `main.c` 侧仍是 2800（见决策记录，PID 反馈用，暂不改）；两处值不一致是有意为之，别当成 bug 又改回去。
- ~~**`can_bridge_node.py` import 路径已断**~~ **已修 (2026-07-04)**：改成平铺 import（`from protocol import`、`from can_interface import`、`from odometry import`），跟全项目其它脚本一致。整个 `ros2_ws/src/` 没有 `package.xml`/`setup.py`，本来就不是 colcon 包，全部脚本都靠 `python3 xxx.py` 跑（node 需先 `source /opt/ros/jazzy/setup.bash`）。若将来要 `ros2 launch` / 参数管理再补齐 package 结构。
- **Motor index ≠ 物理轮位，需要显式 MOTOR_MAP**：CAN 协议按 motor index (0~3) 传 encoder 数据，但 forward kinematics 公式用的是物理轮位 (fl/fr/rl/rr)。两者对应关系只能靠接线约定，不能从协议或代码反推。**已解决 (2026-07-03)**：`main.c` 里加了 `MotorPosition` enum (FL=0/FR=1/RL=2/RR=3)，`odometry.py` 的 `MOTOR_MAP` 保持一致，两边都不再用裸数字。

---

## 整车供电方案（已验证）

**电源**：3S 1500mAh 35C LiPo (XT60) → XT60 分线

**12V 直供（从 LiPo）**：
- TB6612 × 2 的 VM（电机电源）

**5V 供电（UBEC 5V 5A）**：
- RPi 4B ← GPIO Pin 2 (5V) + Pin 6 (GND)（绕过板上 polyfuse，注意极性！）
- STM32 Black Pill ← 5V pin
- MCP2515+TJA1050 模块 ← VCC 5V（TJA1050 需要 5V，模块板载 LDO 给 MCP2515 降压 3.3V）
- RPi 和 STM32 从 UBEC **并联取电**，不串联经过对方

**3.3V**：
- TB6612 × 2 的 VCC（逻辑电源）← 从 STM32 3.3V pin 取或单独稳压
- PS2 接收器 ← RPi 3.3V pin（RPi GPIO 不是 5V tolerant，必须 3.3V）

**注意**：所有模块必须共 GND。RPi GPIO Pin 2 供电量到 ~4.8V，偶尔触发欠压警告但可稳定运行。

---

## 下一步

1. **Encoder 方向验证**：单独驱动纯 vy、纯 ω，确认四路 encoder 符号与 forward kinematics 假设一致，据此填 `odometry.py` 的 `ENCODER_SIGN`
2. **`ENCODER_CPR` 实测标定**：手动转固定圈数（如 10 圈），比对 CNT 差值，更新 `odometry.py` 里的 `ENCODER_CPR`
3. **Ground truth sanity check**：跑正方形轨迹 + 原地转 360°，卷尺量实际终点位置，对比 `odometry.py` 输出的 (x, y, θ)
4. **`can_bridge_node.py` 集成 `OdometryEstimator`**：接收 0x200/0x201 → 调用 update() → 发布 nav_msgs/Odometry + TF
5. **PS2 驱动验证**：接线 + 测试 analog mode 读取
6. **PS2 → cmd_vel**：摇杆数据映射到 ROS2 速度指令 topic
7. **RPi → STM32 方向验证**：cansend 发帧，STM32 收并用 GPIO 指示
8. **RPi 网络修复**：WiFi DHCP 不分配 IPv4 地址，需排查（不影响 CAN 和 GPIO 开发）
9. **Week4**：完成 1~3 项验证后，进入重复轨迹统计，测量真实 odometry drift 并与 motion uncertainty 模型对照

---

## 文件结构（规划）

```
summerslam_ws/
├── src/
│   └── can_bridge/          # ROS2 package
│       ├── can_bridge/
│       │   ├── __init__.py
│       │   ├── can_interface.py    # SocketCAN 封装
│       │   ├── can_bridge_node.py  # 主 node
│       │   ├── heartbeat.py        # heartbeat timer
│       │   ├── protocol.py         # CAN ID/payload 定义
│       │   ├── odometry.py         # x-drive forward kinematics + midpoint pose integration
│       │   └── ps2_controller.py   # PS2 手柄 GPIO bit-bang 驱动
│       ├── launch/
│       ├── config/
│       ├── package.xml
│       └── setup.py
```

---

## 参考文档

- `can_protocol_notebook.html` — 协议完整规格（ID/payload/timing/安全机制）
- `can_protocol_process_log.html` — 设计推导过程记录
