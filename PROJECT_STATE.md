# SummerSLAM — Project State

> 最后更新：2026-07-26
> 当前阶段：Week 5 完成 → Week 6。HC-SR04 双超声波全链路跑通（STM32 TIM9 input-capture → CAN 0x202 → RPi sensor_msgs/Range），bench calibration 拟合噪声模型 sigma(d)=0.0017+0.0078d，BeamModel (Prob Robotics Ch.6) + RectMap ray casting 端到端验证通过（4 位置 × 2 传感器，expected vs observed 误差 <1.3mm，log-likelihood true>wrong 全 PASS）。**下一步：Week 6 MCL 粒子滤波定位**

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

### 硬件事故记录 ⚠️（两次，root cause 未确认）

两次 bring-up 期间均发生 destructive short，但目前没有足够证据确认各自的具体短路点和传播路径。旧记录中的“示波器探针桥接 VM/VCC”只能算假设，不能继续写成已确认原因。

**事故 1 损失**：
- ST-Link V2 clone × 1
- STM32 board × 1
- TB6612 module × 2

**事故 2 损失**：
- STM32 board × 1
- TB6612 module × 1

**总损失**：STM32 × 2、TB6612 × 3、ST-Link × 1。Raspberry Pi 因单独处理 power path 保持安全。

**断电后的 quick triage**：
1. 断开 battery、USB、ST-Link 和所有其他电源
2. 用 multimeter 检查 GND 与 3.3V/5V/12V rail 之间的 resistance/continuity
3. 对可疑芯片检查 GPIO 与 GND，但尽量和同型号 good board 对比
4. 区分 persistent short 与 capacitor charging；蜂鸣不等于板子一定损坏
5. ST-Link 单独插 USB 不亮是本次观测到的 failure symptom
6. 之后用 current-limited bench supply 逐模块重新上电

**教训**：
1. continuity test 只能 triage，不能证明 board 完全正常
2. 12V 与 3.3V/5V wiring 必须物理分开，并在断电后改线
3. 加 fuse/current limit/keyed connector，逐 power branch、逐 motor channel bring-up
4. 备 STM32、TB6612、ST-Link 等低成本 spare，避免单次事故中断整个进度
5. isolated DC-DC 可降低 fault propagation，但若 signal/ground 仍相连就不是完整 galvanic isolation，也不能预防所有 short

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
- [x] HC-SR04 双超声波驱动 (hc_sr04.c/h, TIM9 CH1/CH2 input capture, 顺序测距)
  - PA4=TRIG (共享), PA2=Right ECHO (TIM9_CH1), PA3=Back ECHO (TIM9_CH2)
  - **注意**：PA2/PA3 原为 USART2 debug，已让给 TIM9，debug 串口不再可用
  - CAN 0x202 发送 right_mm + back_mm + status (5 bytes)，集成在 20ms 控制循环

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
- [x] **发现并修复 RL/RR encoder 接反（2026-07-06）**：实机方向验证时 vy 和 omega 通道互换（原地转读成 y、横移读成 omega），vx 正常。用 `encoder_monitor.py` 逐个手转物理轮确认是 RL/RR 两路 encoder 物理接反（index 2 在 RR 轮、index 3 在 RL 轮）。修法：`odometry.py` 的 `MOTOR_MAP` 把 index 2↔3 标签对调（`{0:fl,1:fr,2:rr,3:rl}`），只改 odometry 不用重烧。smoke test 顺手重写成按 `ENCODER_SIGN`/`MOTOR_MAP` 反推构造输入（不再硬编码 per-index tick），改配置不会再悄悄失效
- [ ] Ground truth sanity check：正方形轨迹 + 原地转 360°，卷尺量实际终点位置对比代码输出（**待实机**，用 `can_bridge_node.py` 的 pose 日志观察）。先做方向复验：原地转只 theta 变、横移只 y 变、前进只 x 变
- [x] `can_bridge_node.py` 接入 `OdometryEstimator`（2026-07-04）：收齐 0x200+0x201 后调 `update()`（dt 用 CAN 帧时间戳），发布 `nav_msgs/Odometry` on `odom` + `odom→base_link` TF；另加 ~2Hz 节流 pose 日志供 ground-truth 观察。import 改成平铺式（跟全项目一致），`python3 can_bridge_node.py` 直接跑。**代码完成，待实机验证**

### Week5 Ultrasonic Perception（完成 ✅ 2026-07-20）

- [x] HC-SR04 STM32 驱动 (hc_sr04.c/h): TIM9 input-capture, right+back 顺序测距, 共享 TRIG
- [x] CAN 0x202 协议 (protocol.py): encode/decode right_mm + back_mm + status
- [x] can_bridge_node.py 接收 0x202, 发布 `/ultrasonic/right` + `/ultrasonic/back` (sensor_msgs/Range)
- [x] Bench calibration（2026-07-20）: 2 sensors × 4 distances (100/300/600/1000mm) × 100 samples on KT board
  - 噪声模型拟合: `sigma(d) = 0.0017 + 0.0078 * d` (meters)
  - Bias: ~2.4mm + 0.47% of distance (small positive, not corrected)
  - Invalid rate: ~2.9% overall
  - 数据: `data/ultrasonic/bench_calibration.csv`, `data/ultrasonic/bench_summary.csv`
- [x] BeamModel (Prob Robotics Ch.6 四分量 beam model): w_hit=0.94, w_short=0.01, w_max=0.03, w_rand=0.02
- [x] GaussianModel (简化版, 两分量)
- [x] RectMap + 2D ray casting (map_model.py), 支持 sensor offset
- [x] 端到端 map validation（2026-07-20）: 116.5cm KT board 正方形, 4 位置 × 2 传感器
  - Expected vs observed: 全部 8 路误差 <1.3mm
  - Log-likelihood: true pose 得分 > wrong pose (±50mm shift), 全 4 位置 PASS
  - 数据: `data/ultrasonic/map_validation.csv`
- [x] Sensor offset 实测: RIGHT=(-π/2, (0.0, -0.09)m), BACK=(π, (-0.09, 0.0)m)

### Week 1-6 Engineering Notes（Weeks 1-3 first drafts，2026-07-26）

- [x] Sphinx + `sphinx_rtd_theme` + MyST Markdown 文档骨架放在 `docs/textbook/`
- [x] Home、combined Weeks 1-2、Week 3-6、Reproduction/CAN/Calibration/Debugging appendices 接入 `toctree`；旧 Week 2 URL 保留为 orphan compatibility page
- [x] 每章分开标注 chapter status 与 engineering verification state，未完成内容不冒充已验证
- [x] 硬件事故、CAN/firmware 坑、encoder swap、PID runaway、heartbeat 急停覆盖、frame/scale 修正全部建立 cross-reference
- [x] GitHub Actions workflow：PR 只做 strict build，`main` 文档改动 build + deploy 到 GitHub Pages
- [x] 原创 code 使用 MIT，原创 engineering-note content/media 使用 CC BY 4.0，署名 Thomas Pan；第三方 license 保持不变
- [x] Local strict Sphinx build + desktop/mobile visual QA：pinned Sphinx 9.1.0 toolchain 下 `-W --keep-going` 零 warning；1440×900 / 390×844 检查 sidebar、mobile nav、search、公式、架构图、footer license、Prev/Next 均通过，无 horizontal overflow / broken image
- [ ] External linkcheck：当前 remote `main` 尚无本地 ahead 的 Week 5/6 commits，因此对应 GitHub links 在 push 前返回 404；push 后重新执行 linkcheck
- [ ] GitHub repo Settings → Pages → Source 切到 GitHub Actions，并在 push 后验证公开 URL：`https://tuomaaa.github.io/XDriveSLAMRover/`
- [x] Weeks 1-2 combined chapter：完整保留 mechanical/electrical selection、CubeMX/CMake/CubeProgrammer workflow、motor/encoder/CAN/PS2 bring-up 和两次 power incidents；页面不再使用 TODO/placeholder box，未知事故根因仍明确标作 unproven
- [x] Weeks 1-2 media pass：接入 22 张页面图片（rover、layers、parts、SolidWorks、wiring preview）、drive-test MP4 和 full wiring PDF；Fritzing/KiCad source 保留 repo link
- [x] Weeks 1-2 PCB lesson 修订：先用 Dupont 验证并 finalize wiring；不要因 right-angle routing、SI/PI、crosstalk 等高级建议而拖延第一版 PCB；wiring 不再变化后直接画板，并在 kinematics 阶段并行等待制造
- [x] Weeks 1-2 repository reading guide：加入 48 个 repo links，按 `.ioc` → CMake → `main.c` → MCP2515/protocol → monitor/controller tools 给出阅读顺序，并对关键 function 使用 line link 说明读者应观察的 control flow
- [x] Weeks 1-2 clear-English pass：保留 PWM、encoder、CAN、galvanic isolation、SI/PI、ERC、DRC 等必要术语和完整 reasoning；平均约 14.0 words/sentence，heuristic Flesch-Kincaid Grade 约 7.5
- [x] Weeks 1-2 clean strict rebuild：Sphinx 9.1.0 + MyST 5.1.0、`-E -W --keep-going` 零 warning；local HTTP page、22 image assets、drive-test video 和 wiring PDF 均返回 200；两个 debugging anchors 与全部关键 source line targets 已复核
- [x] Week 3 process chapter：完整覆盖 frame convention、cumulative ticks、CPR、map/sign/frame/scale 分离、inverse/forward kinematics、REP-103 与 $\sqrt{2}$ correction、CAN timestamp、midpoint integration、CAN/ROS boundary 和全部 debugging trail；正文不再使用 TODO/evidence/diagram placeholder
- [x] Week 3 unit explanation 修订：区分物理 wheel-velocity 模型中的 `omega*R` 与 `PS2_Drive_Test.py` 的 normalized joystick `omega`；明确当前 manual controller 不是 SI-unit `cmd_vel` → wheel-RPM conversion，避免读者混用两层单位
- [x] Week 3 repository reading guide：加入 35 个 repo/data links，按 STM32 motor order → CAN decoder → encoder monitor → odometry config/update → ROS publication → controller convention 给出阅读顺序；关键 constant/function 使用有效 line link
- [x] Week 3 code walkthrough pass：新增 8 组短 snippet（geometry/calibration constants、tick conversion、two-frame CAN pairing、normalized inverse kinematics、forward kinematics、midpoint integration、ROS estimator handoff、heartbeat safety gate），页面共 13 个 code blocks；每段都紧跟公式/故障解释并与对应 repo implementation 对照
- [x] Week 3 clear-English pass：保留全部 technical terms、公式、实测数字与局限说明；平均约 14.9 words/sentence，heuristic Flesch-Kincaid Grade 约 9.6（technical vocabulary 计入）
- [x] Week 3 clean strict rebuild + code smoke test：Sphinx `-E -W --keep-going` 零 warning；local HTTP Week 3 返回 200；36 个 local repo targets 与 3 个 debugging anchors 均复核；`python ros2_ws/src/odometry.py` 验证 physical-left input 得到 `vy>0` 且 `vx=omega=0`
- [ ] Weeks 1-3 desktop/mobile visual re-QA：本次 browser runtime 因 Windows `CreateProcessWithLogonW 1385` 无法启动；此前 scaffold 的 layout QA 通过，但合并后的长正文和 Week 3 公式页尚未重新截图检查
- [ ] 收集可公开的 BOM/clean CAD drawing/power/toolchain/PCB evidence 后继续补 Weeks 1-2；Week 3 后续补 frame/kinematics diagrams、CPR raw data、encoder-map evidence、Euler-vs-midpoint plot 和 ROS odometry capture；之后扩写 Week 4
- **注意**：Week 6 MCL design/plan 在 `main`，实现仍在 `week6-mcl` branch；合并并重新验证前 notes 必须保持 `In progress`

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
| 2026-07-06 | `MOTOR_MAP` 后两位（index 2/3）故意与 drive 侧不一致 | 实测 RL/RR 的 encoder 线束物理接反，与 motor 线束是两套独立接线。`MOTOR_MAP` 反映的是 **encoder 接线**（index2=rr, index3=rl），不能为"对齐" `main.c` 的 `MotorPosition` enum 而改回去。判据：vy/omega 互换而 vx 正常 = 轮位映射交换，非 sign/公式问题 |
| 2026-07-06 | forward kinematics 的 `vy`/`omega` 输出翻号，对齐 REP-103 | RL/RR map 修好后方向复验：横移左读成 -vy、CCW 读成 -omega，而 vx（前进）正常。「vx 对、vy+omega 同时反」= 左右镜像，说明 drive 侧逆解约定里 +vy 指向右、+omega 是 CW。`ENCODER_SIGN` 被前进测试钉死、`MOTOR_MAP` 被物理接线钉死，这个 frame 镜像只能在输出端修：`odometry.py` 里 `vy`/`omega` 前加负号，vx 不动（本就 REP-103 正确）。改的是**输出坐标约定**，不是 drive 侧命令约定（遥控手感是另一码事，见"不在本次范围"）|
| 2026-07-06 | `odometry.py` 平移补 `√2`（`TRANSLATION_SCALE`），旋转不动 | Task4 海绵 ground-truth（`data/Week4 Trials.xlsx`）：前进+横移 odom 都读实测的 ~0.74，而原地转读 ~1.0。这个"平移偏低、旋转正常"= X-drive 45° 轮的 `cos45=1/√2=0.707` 投影因子在**未归一化逆解**里缺失（drive 侧 `PS2_Drive_Test.py` 与本文件都用系数=1）；旋转项用 `ωR` 不带投影故不受影响。geometry 保证 + 实测 0.74 双重支持。修法：`vx,vy` 各乘 √2，omega 不动。**drive 侧逆解暂不改**（只影响遥控速度标定，不影响方向；cmd_vel 阶段再统一）。剩余 0.74→0.707 的 ~5% 是海绵打滑/CPR 小残差，待硬地或大海绵长距复测再抠 |
| 2026-07-06 | `PS2_Drive_Test.py` 加 D-pad 微调（`MICRO_RPM=50`） | 遥控摇杆太粗，ground-truth 时对不准地面标记。方向键 上/下=前后、左/右=横移，叠加一个固定小 RPM crawl 在摇杆之上（摇杆归中时即纯慢速微调）。body-frame 约定与摇杆一致（+vx 前、+vy 左）。用 `PS2.is_pressed(btn1, BTN_*)` 读，active-low |
| 2026-07-20 | PA2/PA3 从 USART2 改给 TIM9 (HC-SR04 echo) | STM32 GPIO 全部用完，超声波 echo 需要 input-capture timer，PA2/PA3 是 TIM9 唯一可用 pin。USART2 debug 串口牺牲，改用 CAN 帧观察调试 |
| 2026-07-20 | 两个 HC-SR04 共享一根 TRIG 线 + 软件顺序测距 | GPIO 不够分两根 TRIG。hc_sr04.c 用状态机保证先测 right 再测 back，间隔 60ms 避免 crosstalk。实测 invalid 率 ~3%，没有观察到双峰 |
| 2026-07-20 | 噪声模型用两传感器合并拟合而非分别建模 | 两传感器 std 趋势几乎一致（per-sensor fit 差异 <0.5mm），分别建模增加参数但不增加 MCL 区分度。合并后 sigma(d) = 0.0017 + 0.0078*d |
| 2026-07-20 | BeamModel 权重 w_hit=0.94/w_short=0.01/w_max=0.03/w_rand=0.02 | 实测数据非常干净：无 crosstalk、无 multipath 短读数，invalid 率 ~3% 直接映射到 w_max。w_short 保留极小值兜底 |
| 2026-07-20 | Bias (~2.4mm + 0.47% of distance) 不补偿 | bias 绝对值 <8mm@1m，远小于 MCL 粒子间距（通常几十mm），补偿收益不抵引入的复杂度。如果将来 MCL 精度不够再回来加 |
| 2026-07-26 | Engineering Notes 用 Sphinx + `sphinx_rtd_theme` + MyST，GitHub Pages 作为 repo 首页 | 复用 PythonRobotics 的 Read the Docs layout，但内容定位为 first-person reproduction/process journal；首发只展示 Week 1-6 |
| 2026-07-26 | Code 用 MIT、engineering-note content/media 用 CC BY 4.0（Thomas Pan） | 软件与原创内容复用边界清楚；STM32 HAL、CMSIS 等第三方内容继续服从各自 license，不被 root license 重新授权 |
| 2026-07-26 | Week 1 hardware 与 Week 2 programming 合并成一个 Weeks 1-2 chapter | 实际 bring-up workflow 无法把 hardware verification 和 exercising firmware 分开；旧 Week 2 URL 用 orphan compatibility page 保持稳定，sidebar 不重复展示 |

---

## 已知坑 / Workaround

- **CubeMX TIM3 encoder mode 不写入**：pin SH 配置正确 (Encoder_Interface) 但 `TIM3.EncoderMode` 行不生成，GUI 怎么切都没用。默认值是 TIM_ENCODERMODE_TI1（只单路计数），会导致该轮分辨率只有其他轮的 1/4。Workaround：文本编辑器直接在 .ioc 加 `TIM3.EncoderMode=TIM_ENCODERMODE_TI12` 和 `TIM3.IPParameters=EncoderMode`，再 regenerate。
- **Encoder Period 默认值是最大值不是 0**：CubeMX GUI 默认就填了 16-bit=65535 / 32-bit=4294967295，.ioc 里没显式写出来只是因为没改过默认值，generate code 用的是 GUI 显示值。无需手动改。
- **ST VS Code extension import 可能报 "project corrupted"**：clean project 从 CubeMX 重新 generate code 后直接打开可以绕过。
- **12V 与 low-voltage adjacency 风险**：TB6612 的 VM (12V) 和 VCC (3.3V) 引脚相邻，任何 wiring/probing short 都可能把高压带入 low-voltage rail。两次实际事故的具体 root cause 均未确认；测量或改线前必须断开全部电源。
- **Python match/case capture pattern 陷阱**：`case ENCODER_0:` 里的 bare name 会被当成新变量捕获任意值，不会比较常量。要么用 dotted name `case MsgId.ENCODER_0:`，要么用 if/elif 代替。
- **thumptech/STM32-MCP2515 库 sendMessageTo bug**：`readRegister(rts_addr)` 用 RTS 指令码 (0x81) 当寄存器地址读，读到垃圾值导致 ERROR_FAILTX。修复：改成 `readRegister((txbn + 3) << 4)` 读真正的 TXBnCTRL (0x30/0x40/0x50)。
- **thumptech/STM32-MCP2515 缺 can.h**：仓库没提供 `can_frame` 定义和 Linux SocketCAN 常量（CAN_EFF_FLAG 等），需自建 can.h，typedef struct 而非 struct（C 兼容）。
- **CMake 手动添加源文件**：CubeMX regenerate 会覆盖 CMakeLists.txt，手动加的 mcp2515.c 条目会丢失，需重新添加。
- **RPi WiFi DHCP 不分配 IPv4**：netplan 配置正确，wlan0 状态 UP 但无 IPv4 地址，只有 IPv6 link-local。`netplan apply` 后偶尔恢复。SSH 通过 IPv6 或缓存 session 维持。不影响 CAN / GPIO 开发。
- **RPi 不是 5V tolerant**：RPi GPIO 输入最大 3.3V，不像 STM32 F411 有 FT pin。PS2 接收器、任何外设模块的 DATA 线输出如果是 5V 会烧 RPi GPIO。
- ~~**`ENCODER_CPR=2800` 是理论计算值**~~ **已标定 (2026-07-04)**：`odometry.py` 用实测 2779（四轮均值）。注意 `main.c` 侧仍是 2800（见决策记录，PID 反馈用，暂不改）；两处值不一致是有意为之，别当成 bug 又改回去。
- ~~**`can_bridge_node.py` import 路径已断**~~ **已修 (2026-07-04)**：改成平铺 import（`from protocol import`、`from can_interface import`、`from odometry import`），跟全项目其它脚本一致。整个 `ros2_ws/src/` 没有 `package.xml`/`setup.py`，本来就不是 colcon 包，全部脚本都靠 `python3 xxx.py` 跑（node 需先 `source /opt/ros/jazzy/setup.bash`）。若将来要 `ros2 launch` / 参数管理再补齐 package 结构。
- **Motor index ≠ 物理轮位，需要显式 MOTOR_MAP**：CAN 协议按 motor index (0~3) 传 encoder 数据，但 forward kinematics 公式用的是物理轮位 (fl/fr/rl/rr)。两者对应关系只能靠接线约定，不能从协议或代码反推。`main.c` 里加了 `MotorPosition` enum (FL=0/FR=1/RL=2/RR=3) 描述 **motor/drive 接线**。
- **⚠️ encoder 接线 ≠ motor 接线，`MOTOR_MAP` 后两位与 enum 不一致是对的 (2026-07-06)**：实测 RL/RR 的 encoder 物理接反，所以 `odometry.py` 的 `MOTOR_MAP` 是 `{0:fl,1:fr,2:rr,3:rl}`，index 2/3 跟 `main.c` 的 `MotorPosition` enum（motor 侧）**故意相反**。encoder 与 motor 是两套独立线束，不要为"统一"把它改回去。诊断判据：**vy/omega 互换而 vx 正常 = 轮位映射交换**（不是 sign、不是公式）。

---

## 整车供电方案（当前状态；isolation topology 待补图确认）

**电源**：3S 1500mAh 35C LiPo，记录标称 11.4V (XT60) → XT60 分线

**12V 直供（从 LiPo）**：
- TB6612 × 2 的 VM（电机电源）

**最初 5V 供电（shared UBEC 5V 5A）**：
- RPi、STM32 和 MCP2515+TJA1050 最初共用一个 5V/5A UBEC branch

**当前 5V 供电**：
- RPi 4B（8GB RAM，32GB microSD）← 独立的 5V/5A supply branch；具体 power-entry pin 和 protection 需要在 power-tree 图中复核
- STM32 Black Pill ← motor-control 侧 5V branch
- MCP2515+TJA1050 模块 ← motor-control 侧 5V branch（TJA1050 需要 5V，模块板载 LDO 给 MCP2515 降压 3.3V）
- 这样做降低 motor-control fault 传播到 RPi 的概率，但在 ground 和 communication topology 完整确认前，不能称为已验证的 galvanic isolation

**3.3V**：
- TB6612 × 2 的 VCC（逻辑电源）← 从 STM32 3.3V pin 取或单独稳压
- PS2 接收器 ← RPi 3.3V pin（RPi GPIO 不是 5V tolerant，必须 3.3V）

**注意**：补 power-tree 图时必须明确两个 5V converter、ground、CAN signal path、fuse/current limit 和 RPi power-entry。若要 true galvanic isolation，power 与跨域 signal 都要按隔离方案设计；isolated DC-DC 也不能预防所有 12V short。

---

## Engineering Notebook（本 session 记录）

### Session 目标

推进 Week3→Week4 的 ground-truth / drift 前置工作：把 odometry 的方向、尺度、测试流程和现场操作问题（摇杆太粗、地面 slip、硬件偏航）理顺，做到能在实机上稳定收数据。

### 现场观测 / 现象

1. **方向问题（第一轮）**
   - 原地转被读成 y 变化、横移被读成 omega，前进 x 正常。
   - `encoder_monitor.py` 手转物理轮确认：**RL / RR 两路 encoder 物理接反**。

2. **方向问题（第二轮）**
   - 修完 RL/RR map 后，横移左读成 `-vy`，CCW 读成 `-omega`，但前进 `+x` 正常。
   - 结论：不是 sign，也不是 map，而是**输出坐标约定左右镜像**；drive 侧逆解约定里 `+vy` 实际指向右、`+omega` 指向 CW。

3. **尺度问题（foam ground-truth）**
   - `data/Week4 Trials.xlsx`:
     - 前进 / 横移：odom 约为实测的 **0.74×**
     - 原地转：odom 约为实测的 **~1.0×**（但 actual 是 eye test，只能粗看量级）
   - 结论：这是 **X-drive 45° 轮缺 `cos45 = 1/√2` 投影因子** 的经典指纹。平移少了 `1/√2`，旋转不受影响。

4. **硬件偏航**
   - Test1 前进时 `theta` 稳定落在 `-7° ~ -11°`，肉眼也看到明显转向。
   - 结论：这是 **open-loop + 单电机阻力不均** 的物理偏航，odometry 在忠实记录，不是 odom bug。短期内不修，把它当作 Week4 uncertainty 的一部分。

5. **地面选择**
   - 用户实测：**光滑硬地板误差很离谱，可以 ignore**。
   - 进一步澄清：这里不是“硬地一定更准”，而是**这台 X-drive 在光滑地面对 driven 分量抓地差，translation slip 太大**；反而海绵更咬地。
   - 原本海绵的唯一短板是面积太小（0.6m×0.6m，距离短、量测噪声大）；后来用户确认已有 **约 1m×1m 海绵**，这个短板被补掉。

6. **测试操作问题**
   - 用户指出：PS2 摇杆做不了微调，对准地面标记很困难。
   - 结论：需要给遥控加一个**低速 crawl / nudge 模式**，专门给 ground-truth 对位。

### 本 session 的代码改动

#### 1) `ros2_ws/src/odometry.py`

- **修 RL/RR encoder swap**
  - `MOTOR_MAP` 从 drive 顺序改成 encoder 物理顺序：
    - `0: fl`
    - `1: fr`
    - `2: rr`
    - `3: rl`
  - 注释明确写出：encoder 接线与 motor 接线是两套独立线束，后两位与 `main.c` enum 不一致是**故意的**。

- **修 REP-103 方向**
  - forward kinematics 输出端把 `vy` / `omega` 翻号：
    - 左 = `+y`
    - CCW = `+omega`
  - `vx` 保持不变（本来就对）。

- **补平移 `√2` 尺度**
  - 新增 `TRANSLATION_SCALE = math.sqrt(2)`。
  - `vx`、`vy` 各乘 `TRANSLATION_SCALE`；`omega` 不动。
  - 注释记录依据：foam ground-truth 平移 ~0.74×、旋转 ~1.0×，与缺失 `cos45` 完全一致。

- **smoke test 重写并保持稳健**
  - 不再硬编码 per-index ticks，而是通过 `MOTOR_MAP` + `ENCODER_SIGN` 反推 raw ticks。
  - 这样以后再改 map/sign，test 不会悄悄失效。

#### 2) `ros2_ws/src/PS2_Drive_Test.py`

- **加 heartbeat 线程**（本 session 之前已完成，但本 session 继续沿用验证）
- **加 D-pad 微调模式**
  - `MICRO_RPM = 50`
  - D-pad:
    - 上 / 下 = 前进 / 后退
    - 左 / 右 = 左移 / 右移
  - 逻辑上是给 `vx` / `vy` 叠一个固定小 crawl；摇杆归中时就变成纯微调。
  - 目的：让用户在 ground-truth 时可以把车**慢慢蹭到地面标记上**。

### 本 session 的验证

- `python odometry.py` smoke test 通过：
  - `vx = 0`
  - `omega = 0`
  - `vy > 0`
- `python -m py_compile PS2_Drive_Test.py` 通过。
- 代码已 commit & push：
  - `2b015ab` — `odometry: fix swapped RL/RR encoders in MOTOR_MAP`
  - `16bfac0` — `odometry: flip vy/omega sign to REP-103 (left=+y, CCW=+omega)`
  - `c7492f3` — `odometry: bake in sqrt2 translation scale; PS2: add D-pad fine-adjust`

### 本 session 的测试方法 / 实验设计决策

1. **平移尺度怎么量**
   - 不要求车走直线；允许它因为硬件问题拐弯。
   - 真正拿来比的是：
     - 卷尺量的 **起点→终点净位移直线距离**
     - odom 的 `sqrt(x^2 + y^2)`
   - 因为偏航不破坏这个比值，所以即便路径是弧线，平移 scale 仍然可测。

2. **旋转数据怎么用**
   - 用户补充：所有 omega actual 基本都是 **eye test 的大致 180/360**，本身误差不小。
   - 因此：旋转数据只拿来证明“量级对、旋转不需要 √2”，**不拿来抠 `CENTER_TO_WHEEL_M` 的精度**。

3. **地面怎么选**
   - 放弃光滑硬地。
   - 使用 **1m×1m 海绵** 做长距离复测，既保留低 slip，又把距离拉长到 ~0.6–0.7m，降低量测噪声。

4. **速度怎么选**
   - 不推满，不慢挪。
   - 原则是：**中等稳速、每 trial 尽量一致**。
   - 理由：
     - 太快 → 猛起步/急停导致 slip
     - 太慢 → 落入死区，弱电机可能根本不转

5. **样本量建议**
   - 前进：10 次（既测 scale，也估偏航 std）
   - 横移：5–6 次（验证 y 轴 scale 是否一致）
   - 原地转：5 次（只看量级）
   - 正方形：可选，主要看硬件导致的累积偏航，不作为尺度依据

6. **正方形怎么解释**
   - 在这台车上，正方形“闭不闭合”主要反映硬件偏航，不是 odometry 本身。
   - 真正该比的是：**odom 报的终点姿态 vs 实际终点姿态是否一致**。
   - 所以正方形只是附加实验，不是主尺度实验。

### 当前结论（截至本 session 结束）

- odometry 的三件核心事都已理顺：
  1. **map**（RL/RR encoder swap）
  2. **sign / frame convention**（REP-103）
  3. **translation scale**（√2）
- `can_bridge_node.py` 可继续直接用于 ground-truth 读数。
- 目前最大的非理想项不是 odometry 数学，而是：
  - **open-loop 单电机不均导致的偏航**
  - **实际测试地面的抓地特性**
  - **旋转 actual 量测本身比较粗**

### 下一步（从这个 session 接下去）

1. Pi `git pull`，带上：
   - `odometry.py` 的 RL/RR + REP-103 + √2
   - `PS2_Drive_Test.py` 的 D-pad 微调
2. 在 **1m×1m 海绵** 上做新一轮 Task 4：
   - 前进 ~0.6–0.7m × 10
   - 左移 ~0.6m × 5–6
   - 原地转（粗测量级）× 5
3. 把新数据回填到 `data/Week4 Trials.xlsx` 或新表
4. 用新数据：
   - 验证 √2 修正后平移是否接近 1.0× 实测
   - 量化偏航的 mean / std，作为 Week4 motion uncertainty 输入
5. 若需要，再用 `encoder_monitor.py` 做“弱轮定位”（看纯前进时哪列 tick 增速偏少）

---

## 下一步

1. **Week 6 — MCL 粒子滤波定位**：用 encoder odometry 做 motion update，用超声波 BeamModel 做 measurement update，在 KT 板矩形地图内可视化粒子收敛
2. **Ground truth sanity check**：在 1m×1m 海绵上做长距复测（前进 / 横移 / 粗旋转），更新 Week4 数据表（仍待做）
3. **弱轮定位**：用 `encoder_monitor.py` 看纯前进时哪一路 tick 增速明显偏少，确认偏航根因
4. **RPi 网络修复**：WiFi DHCP 不分配 IPv4 地址，需排查（不影响 CAN 和 GPIO 开发）
5. **Engineering Notes Weeks 1-3 review**：审阅 combined platform/programming narrative 与 Week 3 concept draft；补硬件 evidence 和 Week 3 diagrams/data/ROS capture 后开始 Week 4

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
