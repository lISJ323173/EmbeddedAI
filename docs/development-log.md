# 任务四：Embedded AI —— 开发文档

> 在 STM32F407 上部署小型神经网络，用 MPU6050 数据判断人体运动状态（走路 / 跑步 / 静止）。
> 本文档只记录任务四的工作，随开发推进增量更新。

## 1. 任务与交付

### 1.1 任务要求

在 STM32F407 上部署一个小型神经网络；识别 MPU6050 数据并判断人体运动状态，至少区分走路 / 跑步 / 静止三种。需说明：

1. 任务定义、输入维度、采样率、预处理与滑窗方式；
2. 模型结构、参数量与量化方案；
3. 推理引擎（CMSIS-NN 或自写定点算子）、算子实现、内存布局与峰值 RAM 占用；
4. 板上实测表格：单次推理时延、RAM / Flash 占用、准确率、与 PC 浮点结果的误差；
5. 输出可观察（串口 / 屏幕 / 上位机）。

提交物：模型文件与转换脚本、Git 仓库（保留完整 commit 记录）、开发文档（含实测数据）、演示视频。

### 1.2 要求与章节对应

| 要求 | 对应章节 | 状态 |
|---|---|---|
| ① 任务定义 / 输入维度 / 采样率 / 预处理 / 滑窗 | 第 2 节 | 方案已定，待实现 |
| ② 模型结构 / 参数量 / 量化 | 第 7 节 P2 | 未开始 |
| ③ 推理引擎 / 算子 / 内存布局 / 峰值 RAM | 第 7 节 P3 | 未开始 |
| ④ 板上实测表 | 第 5 节（已开始积累） | 进行中 |
| ⑤ 输出可观察 | 4.3 节（串口已通） | 已完成 |
| 提交物 | 第 8 节文件索引 | 进行中 |

## 2. 方案概述

```text
MPU6050 ──I²C(100 kHz)──► STM32F407 ──USART1(115200)──► PC
  6 轴原始值              采集 + 滑窗 + 推理            训练 / 评估 / 采集
```

| 设计项 | 取值 | 依据 |
|---|---|---|
| 采样率 | 50 Hz | 与所选公开数据集一致；步频 2–3 Hz，50 Hz 足够且带宽占用小 |
| 窗口 | 2 s，50% 重叠 | 2 s 可容纳 2–4 个步态周期，重叠避免边界丢样本 |
| 输入维度 | 6 轴 × 统计特征（均值 / 标准差 / 最大 / 最小 / 过零率等） | 特征法比原始波形简单一个数量级，先跑通再考虑 1D-CNN |
| 量化 | int8，对称量化 | Cortex-M4F 无 FPU 加速的整型算子，CMSIS-NN 原生支持 |
| 输出 | 串口一行：状态 + 置信度 | 最省事的可观察输出 |

以上为设计值，实测值以第 5 节为准。

## 3. 硬件与工具链

| 项 | 值 |
|---|---|
| 开发板 | STM32F407ZGT6（Cortex-M4F，192 KB RAM / 1 MB Flash） |
| 传感器 | MPU6050（GY-521 模块），I²C 地址 `0x68`（AD0 接地） |
| 接线 | SCL→PB6，SDA→PB7，VCC→3.3V，GND→GND |
| 调试器 | J-Link，SWD，1 MHz |
| 构建 | CMake preset `Debug`，arm-none-eabi-gcc 14.3.1，Ninja |
| 烧录 | Ozone 工程 `ozone/imu-motion-ai.jdebug` |
| 串口 | USART1（PA9 TX / PA10 RX），115200 8N1 |
| PC | Windows 11，Python 3.14.7 |

固件构建命令：

```bash
cmake --preset Debug && cmake --build --preset Debug
```

## 4. 已完成工作

### 4.1 工程基线（P0.1）

以现有 F407 工程为基线建立独立仓库（保留 Git 历史），在其上新增 IMU 采集与送样任务。

```text
RAM:   21544 B / 128 KB  (16.44%)
FLASH: 67564 B / 1 MB    (6.44%)
产物：  build/Debug/framework.elf
```

### 4.2 传感器身份确认（P0.2a）

启动时对 `0x68`、`0x69` 分别读 `WHO_AM_I`(0x75)，只读不写配置，记录 HAL 状态与错误码：

```text
IMU_DIAG addr=0x68 hal=0 err=0x00000000 id=0x68
IMU_DIAG addr=0x69 hal=1 err=0x00000004 id=0xFF
[INFO] IMU WHO_AM_I=0x68
```

- `0x68`：`HAL_OK`，身份值 `0x68`，与 MPU6050 手册一致；
- `0x69`：`HAL_ERROR` + `0x04`（`HAL_I2C_ERROR_AF`，无应答），该地址无器件，符合预期。

### 4.3 六轴连续输出与姿态响应（P0.2b）

串口格式（当前版本，无时间戳）：

```text
IMU,ax,ay,az,gx,gy,gz
IMU,712,-324,-15832,1895,-290,-741
```

采集方法：模块正面朝上平放 → 缓慢翻转 → 正面朝下平放，连续采集后按 `az` 符号自动切分两段，
用 `tools/check_imu_log.py` 校验六列均为整数且 `az` 变号：

```text
ROWS flat=30 flip=211
AZ flat=+17130 flipped=-15890
FLIP_SPAN ax=3780 ay=17984 az=35120 gx=25321 gy=3824 gz=4194
PASS: six integer columns and az sign reversal
```

结论：

- 六轴同时输出且均为整数；
- 翻转后 `az` 由 `+17130` 变为 `-15890`，符号反转，符合重力方向变化；
- 翻转过程中 `gx` 跨度 25321（约 193 °/s），角速度轴对实际转动有响应。

### 4.4 送样任务的行为

- 采样周期 50 ms（20 Hz，临时值）；
- 传感器缺失或总线异常时不死机：打印 `IMU,ERR,<错误码>`，每秒重试一次初始化，接上即自愈；
- 看门狗改为编译期开关（`APP_USE_IWDG`，默认关闭），避免下载/调试期间被复位。

### 4.5 板端时间戳与采样率标定（P1.1）

每行在读取六轴之前记录 `HAL_GetTick()`，格式改为 `IMU,t_ms,ax,ay,az,gx,gy,gz`。

**第一次测量（保持原调度 `osDelay(50)`）**：

```text
samples 123 / 16 s，相邻间隔全部为 132 ms
→ 7.6 Hz（远低于 20 Hz 设计值）
```

根因：送样任务优先级为 `osPriorityLow`，而基线工程的 `servo_rx_task`（`osPriorityNormal`）
用 10 ms 阻塞式轮询读串口，占用绝大部分 CPU，低优先级任务被饿到约 132 ms 才轮到一次。
间隔完全一致（全部 132 ms）说明是稳定抢占，而不是随机丢样。

**修复**：送样任务优先级提升到 `osPriorityHigh`；延时改为补偿式 `osDelay(50 - 已耗时)`。

**修复后**：

```text
TIMING samples=325 rate=20.02 Hz min=49 ms median=50 ms max=50 ms
PASS: 20 Hz pilot timing
```

### 4.6 公开数据集接入（MotionSense）

走、跑两类若自行采集，需要可穿戴的无线链路；为尽快跑通训练与部署，采用公开数据集
[MotionSense](https://github.com/nicocavalcanti/motion-sense)（IoTDI'19）：

| 项 | 值 |
|---|---|
| 设备与位置 | iPhone 6s，裤前口袋 |
| 采样率 | 50 Hz |
| 受试者 | 24 人 |
| 动作 | `wlk` 走路、`jog` 慢跑、`std`/`sit` 静止 |
| 类别映射 | walk = wlk，run = jog，still = std + sit |

原始数据为 Core Motion 的合并输出（gravity + userAcceleration + rotationRate），
转换时按 `accel = gravity + userAcceleration`（单位 g）、`gyro = rotationRate`（单位 rad/s）
还原，再换算到板端整数域（±2g → 16384 LSB/g，±250 dps → 131 LSB/(°/s)）：

```text
run    rows= 134231 files= 48 subjects=24  (2684.6 s)
still  rows= 645205 files= 96 subjects=24  (12904.1 s)
walk   rows= 344288 files= 72 subjects=24  (6885.8 s)
TOTAL  rows=1123724 files=216
```

换算后自检：静置合加速度 1.00 g，走路均值 1.22 g、跑步 1.71 g，陀螺 RMS 分别为
250 / 19796 / 27846 LSB，量级合理。

固件相应改为 **50 Hz**（周期 20 ms）：

```text
TIMING samples=830 rate=51.34 Hz min=19 ms median=19 ms max=20 ms
PASS: 50 Hz pilot timing
```

> 域差异说明：数据集为裤袋中的手机，板端为手腕模块。板上实时演示时需把模块置于裤袋
> 以复现同一域；若仍戴手腕，应视为跨域测试并在文档中注明。

### 4.7 受试者划分与量程审计（P1.3）

按受试者划分，避免同一人的动作同时出现在训练与测试：

| 划分 | 受试者 | 文件数 | 用途 |
|---|---|---|---|
| train | 1–16 | 144 | 训练与量程评估 |
| val | 17–20 | 36 | 调参 |
| test | 21–24 | 36 | 定稿后一次性评分 |

清单见 `motionsense_split.csv`（216 行，含 split/subject/label/source/rows）。

量程审计（训练集原始数据，任一轴越界即计为超限）：

| 类别 | 行数 | 加速度 > 2 g | 陀螺 > 250 °/s |
|---|---|---|---|
| still | 645,205 | 0.00% | 0.00% |
| walk | 344,288 | **4.89%** | 1.99% |
| run | 134,231 | **23.94%** | **16.26%** |

分位数（训练集）：

| 类别 | \|a\| p99 | \|a\| max | 陀螺 p99 | 陀螺 max |
|---|---|---|---|---|
| still | 1.02 g | 2.72 g | 32 °/s | 367 °/s |
| walk | 2.87 g | 5.64 g | 283 °/s | 521 °/s |
| run | 5.12 g | 8.11 g | 456 °/s | 892 °/s |

结论：默认的 ±2 g / ±250 °/s 会把跑步数据削掉近四分之一，必须改用
**±8 g（4096 LSB/g）/ ±1000 °/s（32.8 LSB/(°/s)）**，此时削顶比例低于 0.01%，
数据集换算系数需同步修改。

### 4.8 量程设置与读回校验（P1.4）

按审计结论把量程调整为 ±8 g / ±1000 °/s，启动时写入并读回校验：

```text
GYRO_CONFIG (0x1B) = 0x10  → FS_SEL=2，±1000 °/s，32.8 LSB/(°/s)
ACCEL_CONFIG(0x1C) = 0x10  → AFS_SEL=2，±8 g，4096 LSB/g
```

实测输出：

```text
IMU_RANGE OK gyro=0x10 accel=0x10
IMU,t_ms,ax,ay,az,gx,gy,gz @50Hz
IMU,234,-1465,1652,-3210,459,-61,-183
```

- 两寄存器的量程位读回均为 `0x10`，写入生效；
- **正面朝上静置实测**：`ax=+74, ay=-25, az=+4310`（标准差 13–21 LSB），
  即 1 g = **4310 LSB**，比标称 4096 高约 5.2%；
- 因此数据集换算采用**实测标定值 4310 LSB/g**（1 g 单点标定），陀螺仍用标称 32.8 LSB/(°/s)；
- 陀螺静止读数 `gx≈+478`（≈14.6 °/s）为固定零偏，后续特征按窗口去均值时可消除。

### 4.9 新量程下的削顶复算与首个窗口（P1.5）

阈值改为 8 g / 1000 °/s 后复算训练集超限比例：

| 类别 | 行数 | 加速度 > 8 g | 陀螺 > 1000 °/s |
|---|---|---|---|
| still | 645,205 | 0.00% | 0.00% |
| walk | 344,288 | 0.00% | 0.00% |
| run | 134,231 | 0.00% | 0.00% |

（run 的峰值 8.11 g 略超标称 8 g，超限行数不足 0.01%，四舍五入后为 0.00%。）

取 `wlk_7/sub_1` 前 100 条（50 Hz，即 2 s）生成首个窗口，逐轴限幅后计算模长：

```text
WINDOW label=walk subject=1 samples=100 channels=2 file=p1_first_window.csv
accel_norm: mean 1.155 g, std 0.565, min 0.252, max 2.921
gyro_norm : mean 121.0 dps, std 62.9, min 12.3, max 421.0
```

该窗口仅含两路模长，用于核对切窗流程；39 维窗口特征在 P2 生成。

### 4.10 窗口特征表（P1.6）

每份录制独立切窗：窗口 100 点（2 s）、步长 50 点（1 s），末尾不足 100 点舍弃；
每个窗口继承原录制的受试者、标签与 train/val/test 划分。

特征共 **8 维**（`motionsense_features.csv`，22,155 行）：

| 通道 | 特征 |
|---|---|
| 加速度模长 | `acc_mean` `acc_std` `acc_max` `acc_mean_delta` |
| 角速度模长 | `gyro_mean` `gyro_std` `gyro_max` `gyro_mean_delta` |

其中 `mean_delta` 为相邻样本差值的绝对值平均，反映信号变化速率。

```text
FEATURES rows=22155 inputs=8
train still=8610 walk=4458 run=1727
val still=2149 walk=1197 run=437
test still=2006 walk=1123 run=448
```

校验：216 个录制源的 `start` 均按 0, 50, 100… 递增（无一处步长异常），
跨录制从 0 重新开始。训练集类别比例约 **5.0 : 2.6 : 1**（still : walk : run），
训练时需按反比加权。

### 4.11 PC 训练环境（P2.1）

板端脚本用系统自带的 Python 3.14.7；训练环境另行使用 **Python 3.12.10**
（TensorFlow 2.20.0 的 Windows 轮子只覆盖到 3.12），两者并存，互不影响。

| 项 | 值 |
|---|---|
| 解释器 | `py -3.12` → `C:\Users\...\Programs\Python\Python312\python.exe` |
| 虚拟环境 | `.venv-train/`（已加入 `.gitignore`） |
| 依赖 | `requirements-train.txt`：`tensorflow==2.20.0`、`numpy==2.1.3` |

验证：

```text
$ ./.venv-train/Scripts/python.exe -m pip check
No broken requirements found.

$ ./.venv-train/Scripts/python.exe -c "..."
TF 2.20.0 NUMPY 2.1.3
MATMUL 23
```

`MATMUL 23` 即 2×4 + 3×5，证明 TensorFlow 能在 CPU 上实际执行计算，而不仅是导入成功。

### 4.12 第一版浮点模型（P2.2）

训练前先核对三类特征均值（8 维，全量窗口统计）：

| 类别 | acc_mean | acc_std | acc_max | acc_mean_delta | gyro_mean | gyro_std | gyro_max | gyro_mean_delta |
|---|---|---|---|---|---|---|---|---|
| still | 1.005 | 0.007 | 1.031 | 0.004 | 3.8 | 2.6 | 11.7 | 0.7 |
| walk | 1.145 | 0.558 | 3.161 | 0.262 | 119.0 | 54.4 | 275.1 | 29.9 |
| run | 1.599 | 1.108 | 5.588 | 0.656 | 194.4 | 92.8 | 486.8 | 63.3 |

三条常识全部成立：`acc_std` 与 `gyro_std` 单调递增（run > walk ≫ still）、
静止的 `gyro_mean` ≈ 0、三类 `acc_mean` 都在 1 g 附近。

模型与训练设置：

| 项 | 取值 |
|---|---|
| 结构 | 8 → 16 (ReLU) → 3 (logits) |
| 参数量 | **195** |
| 标准化 | 用训练集均值/标准差，验证集复用 |
| 类别权重 | 按训练集反比（约 0.57 / 1.11 / 2.86） |
| 训练 | Adam，batch 128，最多 40 轮，早停 patience=5 |

验证集（受试者 17–20，3,783 个窗口）结果：

```text
MODEL params=195 train=14795 val=3783
VAL accuracy=0.9804 labels=['still', 'walk', 'run']
VAL confusion (rows=true, columns=predicted):
[[2149    0    0]
 [   6 1191    0]
 [   0   68  369]]
```

| 类别 | 样本 | 召回率 | 主要错误 |
|---|---|---|---|
| still | 2149 | 1.000 | 无 |
| walk | 1197 | 0.995 | 6 个误判为 still |
| run | 437 | **0.844** | 68 个误判为 walk |

跑步召回率 84.4% 是当前短板，与训练样本最少（1,727 个窗口，约占 12%）一致；
后续可通过过采样、加大权重或增加特征改善。

产物：`artifacts/motion_float.keras`、`artifacts/motion_preprocess.json`（含特征顺序、标签顺序、标准化参数）。

### 4.13 int8 全整数量化（P2.3）

用训练集每类 512 个窗口做代表性校准，输入输出均强制 `int8`：

```text
INT8 bytes=2376 in=(0.0227535, -92) out=(0.1275901, 36)
VAL n=3783 float=0.9804 int8=0.9799 agree=0.9995
RUN float=369/437 int8=368/437
ERROR logit_mae=0.055653 input_clipped_rows=46
```

| 指标 | 浮点 | int8 | 变化 |
|---|---|---|---|
| 验证集准确率 | 0.9804 | **0.9799** | −0.05 个百分点 |
| 跑步召回 | 369/437（84.4%） | **368/437（84.2%）** | 少 1 个 |
| 两版预测一致率 | — | **0.9995** | 3,783 个窗口仅 2 个不同 |
| 模型体积 | 23.6 KB（.keras） | **2,376 B（.tflite）** | 约 1/10 |

- 输入量化参数 `scale=0.0227535, zero_point=-92`，对应可表示范围约 ±2.89 个标准差；
  3,783 个验证窗口中有 **46 行（1.2%）**的标准化特征被截断，对结果影响可忽略；
- logits 平均绝对误差 0.0557，属于低比特量化正常量级；
- 跑步仍是短板（int8 下 368/437），与浮点一致，说明损失来自模型本身而非量化。

产物：`artifacts/motion_int8.tflite`（2,376 B）。

### 4.14 导出模型参数（P3.1）

自写定点推理的前提是把 `.tflite` 里的常量抄成固件可直接引用的数组。
`tools/export_model_header.py` 先校验结构，再生成 `config/motion_model_data.hpp`：

- 单子图、两个算子，均为 `FULLY_CONNECTED`，权重格式为默认、第一层融合 ReLU；
- 张量类型：输入/权重/输出 `int8`，偏置 `int32`；
- 权重为零点对称量化（`zero_point=0`），且按输出通道逐通道给 scale。

```text
DENSE1 8->16 weights=128 bias=16
DENSE2 16->3 weights=48 bias=3
EXPORT OK config/motion_model_data.hpp
ARM HEADER OK
```

导出的量化参数：

| 层 | 输入 scale / zero | 输出 scale / zero | 权重 scale |
|---|---|---|---|
| dense1 | 0.0227535 / −92 | 0.0585463 / −128 | 16 个（逐通道） |
| dense2 | 0.0585463 / −128 | 0.1275901 / 36 | 3 个（逐通道） |

`ARM HEADER OK` 来自 `arm-none-eabi-g++ -std=c++17 -fsyntax-only` 加两条
`static_assert`（dense1 权重 128 字节、dense2 权重 48 字节），确认头文件语法与数组长度正确。

## 5. 实测数据汇总

| 项目 | 实测值 | 备注 |
|---|---|---|
| WHO_AM_I | `0x68` | 冷上电，地址 0x68 |
| 静置 `az`（±8 g，正面朝上） | **+4310** | 1.052 g，标准差 21 LSB |
| 静置合加速度（±8 g） | **4310 LSB = 1.052 g** | 用于 1 g 单点标定 |
| 陀螺零偏（静置，±1000 °/s） | gx 约 +478（≈14.6 °/s） | 窗口去均值可消除 |
| 当前采样率 | **51.34 Hz**（间隔 19/19/20 ms，830 样本 / 16 s） | 改为 50 Hz 后达标 |
| FLASH 占用 | 68088 B / 1 MB（6.49%） | 含诊断与送样代码 |
| RAM 占用 | 21544 B / 128 KB（16.44%） | 静态占用 |

## 6. 问题与经验

1. **调试器暂停会导致 I²C 总线被锁死**：J-Link 连接时 halt CPU，若停在 I²C 传输中途，
   从机会拉住 SDA，`I2C1.SR1.ARLO=1`、`I2C1.SR2.BUSY=1`，此时 MCU 复位无效，必须整板断电。
   这不是硬件故障。
2. **下载中断会写坏镜像**：早期固件启用独立看门狗（约 3 s），下载期间 CPU 停住而 IWDG 继续计数，
   导致 Flash 写一半被复位，表现为 `Not all data bytes were successfully downloaded`。
   现改为编译期开关，默认关闭。
3. **加速度量程偏差**：静置 `az` 比理论值高约 4.5%，属器件灵敏度误差，归一化后可吸收。

## 7. 后续计划

| 阶段 | 内容 | 状态 |
|---|---|---|
| P0 | 采集链路：构建、WHO_AM_I、六轴输出、时间戳 | ✅ 完成 |
| P1 | 采样率标定（50 Hz）、公开数据集接入、受试者划分、量程审计与设置 | ✅ 完成 |
| P2 | 39 维窗口特征、MLP 训练、int8 量化、与浮点对拍 | 进行中 |
| P3 | 模型转 C 数组、CMSIS-NN 推理、arena 与内存布局 | 未开始 |
| P4 | 板上实测：单次推理时延、RAM/Flash、准确率、与 PC 误差 | 未开始 |
| P5 | 演示视频、文档定稿 | 未开始 |

## 8. 文件索引

| 路径 | 说明 |
|---|---|
| `Core/Src/main.cpp` | 板级初始化、诊断、送样任务 |
| `device/src/mpu6050.cpp` | MPU6050 驱动（一次读 14 字节） |
| `ozone/imu-motion-ai.jdebug` | Ozone 调试/烧录工程 |
| `tools/check_imu_log.py` | 六轴日志一致性校验 |
| `p0_flat.txt` / `p0_flip.txt` | 4.3 节的原始日志 |

## 9. 开发约定与提交对照

每个步骤完成时同步做三件事，保证可追溯：

1. 更新本文档对应小节（含实测数字，不写“已实现”这类空话）；
2. 提交代码/脚本/数据，提交信息用 `类型(范围): 动作`，类型取 `feat / fix / test / docs / chore`；
3. 推送并记录提交哈希。

| 步骤 | 主要提交 | 内容 |
|---|---|---|
| P0.1 | `63c2444` | 独立仓库、`.gitignore`、Debug 构建通过 |
| P0.2b | `527a4a6` | 六轴送样（20 Hz） |
| — | `9f0dbfa` | IWDG 改为编译开关，修复下载中断 |
| — | `e8eb39b` | Ozone 工程（1 MHz，指向本仓库 ELF） |
| — | `b8bf68b` | 无传感器时不停机、每秒重试 |
| P0.2a | `56a2723` | 启动读 WHO_AM_I（0x68/0x69）并读回校验 |
| — | `a88732c` | 平放/翻转日志、校验脚本、首版文档 |
| — | `e0c60c2` `1a343fb` `e19c62f` | 文档与 README 收敛到任务四 |
| P1.2 | `a39b634` | 试采转换脚本 + 静止段 |
| P1.1 | `04dd300` `50d7e8e` | 时间戳、20 Hz 优先级饥饿修复 |
| P1.3 | `ae888e7` | MotionSense 接入并统一到 50 Hz |
| P1.3 | `4c843fa` | 受试者划分与削顶审计 |
| P1.4 | `5473729` `ec10d7b` | ±8 g/±1000 °/s、4310 LSB/g 标定 |
| P1.5 | `d18ad33` | 新量程削顶复算、首个 100 点窗口 |
