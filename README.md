# 任务四：Embedded AI —— 基于 MPU6050 的人体运动状态识别

在 STM32F407 上采集 MPU6050 六轴原始数据，部署小型神经网络，区分 **走路 / 跑步 / 静止** 三种状态。

> 本仓库沿用既有 F407 工程骨架作为基线（保留其 Git 历史），任务四的工作在该基线上增量开发。

| 当前阶段 | 状态 |
|---|---|
| P0 采集链路打通（I²C → 固件 → 串口） | ✅ 已完成 |
| P1.1 板端毫秒时间戳与采样率标定 | ✅ 20.02 Hz |
| P1.2 100 Hz 采样与数据集采集 | ⏳ 进行中 |
| P2 模型训练与 int8 量化 | 未开始 |
| P3 板上推理（CMSIS-NN） | 未开始 |
| P4 实测（时延 / RAM / 准确率） | 未开始 |

## 任务要求

1. 说明任务定义、输入维度、采样率、预处理与滑窗方式；
2. 给出模型结构、参数量与量化方案；
3. 推理引擎可使用 CMSIS-NN 或自写定点算子，说明算子实现、内存布局与峰值 RAM 占用；
4. 板上实测表格：单次推理时延、RAM / Flash 占用、准确率、与 PC 浮点结果的误差；
5. 输出可被观察（串口 / 屏幕 / 上位机）。

完整过程与实测数据见 **[开发文档](docs/development-log.md)**。

## 系统方案

```text
MPU6050 ──I²C 100 kHz──► STM32F407 ──USART1 115200──► PC（训练 / 评估）
 6 轴原始值                采集 + 滑窗 + 推理
```

| 设计项 | 取值 | 依据 |
|---|---|---|
| 采样率 | 100 Hz（当前临时 20 Hz） | 步频 2–3 Hz，留 30 倍余量 |
| 窗口 | 2 s，50% 重叠 | 容纳 2–4 个步态周期，重叠避免边界丢样 |
| 输入 | 6 轴 × 统计特征（均值/标准差/极值/过零率） | 先跑通再考虑 1D-CNN |
| 量化 | int8 对称量化 | CMSIS-NN 原生支持 |
| 输出 | 串口一行：状态 + 置信度 | 最直接的可观察输出 |

## 硬件接线

| MPU6050 | STM32F407 | 说明 |
|---|---|---|
| VCC | 3.3V | 不要接 5V |
| GND | GND | 必须共地 |
| SCL | PB6 | I²C1_SCL |
| SDA | PB7 | I²C1_SDA |
| AD0 | GND | 地址 0x68 |
| INT / XDA / XCL | 不接 | — |

串口：USART1（PA9 TX / PA10 RX），**115200 8N1**。

## 快速开始

```bash
# 构建
cmake --preset Debug && cmake --build --preset Debug

# 产物
build/Debug/framework.elf     # Ozone 直接加载
build/Debug/framework.hex     # J-Link / 通用烧录器
```

烧录：Ozone 打开 [`ozone/imu-motion-ai.jdebug`](ozone/imu-motion-ai.jdebug)，点 `Download & Reset Program`。

串口输出（首列为板端毫秒时间戳）：

```text
[INFO] USART1 ready
[INFO] IMU WHO_AM_I=0x68
IMU,t_ms,ax,ay,az,gx,gy,gz @20Hz
IMU,2126,-1256,14744,7440,1824,-328,-601
```

## 已实测结果

| 项目 | 实测值 |
|---|---|
| WHO_AM_I（地址 0x68） | `0x68`，读取 `hal=0 err=0x00000000` |
| 静置 az（正面朝上） | +17130（比理论 16384 高约 4.5%） |
| 翻转后 az（正面朝下） | −15890（符号反转，PASS） |
| 翻转时陀螺跨度 | gx 25321（约 193 °/s） |
| FLASH 占用 | 68088 B / 1 MB（6.49%） |
| RAM 占用 | 21544 B / 128 KB（16.44%） |
| 当前采样率 | **20.02 Hz**（时间戳间隔 49/50/50 ms，325 样本 / 16 s） |

校验脚本：`python tools/check_imu_log.py`（读取 `p0_flat.txt` / `p0_flip.txt`）。

## 目录速览

| 目录 | 与任务四相关的部分 |
|---|---|
| `Core/Src/main.cpp` | 板级初始化、I²C/串口、WHO_AM_I 诊断、IMU 送样任务 |
| `device/src/mpu6050.cpp` | MPU6050 驱动，一次读 14 字节（6 轴 + 温度） |
| `port/stm32f4/` | USART / I²C / PWM 的 HAL 实现 |
| `tools/` | 日志校验脚本 |
| `ozone/` | 调试与烧录工程 |
| `docs/development-log.md` | 任务四开发文档（进度、实测、问题记录） |

## 调试注意事项

用调试器连接会暂停内核。若停在 I²C 传输中途，从机会拉住 SDA，导致 `I2C1.SR2.BUSY=1`，
此时 MCU 复位无效，**必须整板断电**。这是调试副作用，不是硬件故障。
