# 任务四：Embedded AI —— 基于 MPU6050 的人体运动状态识别

在 STM32F407 上采集 MPU6050 六轴原始数据，部署小型神经网络，区分 **走路 / 跑步 / 静止** 三种状态。

> 本仓库沿用既有 F407 工程骨架作为基线（保留其 Git 历史），任务四的工作在该基线上增量开发。

| 当前阶段 | 状态 |
|---|---|
| P0 采集链路（I²C → 固件 → 串口） | ✅ 已完成 |
| P1 采样率 50 Hz、量程 ±8 g/±1000 °/s、数据集 | ✅ 已完成 |
| P2 8 维特征 + MLP（195 参数）+ int8 量化 | ✅ 已完成 |
| P3 自写定点算子，板端两层推理 | ✅ 单样本与 TFLite 完全一致 |
| P4 板上实测（时延 / RAM / 准确率 / 与 PC 误差） | ⏳ 单样本已通，批量评测待做 |

## 任务要求

1. 说明任务定义、输入维度、采样率、预处理与滑窗方式；
2. 给出模型结构、参数量与量化方案；
3. 推理引擎可使用 CMSIS-NN 或自写定点算子，说明算子实现、内存布局与峰值 RAM 占用；
4. 板上实测表格：单次推理时延、RAM / Flash 占用、准确率、与 PC 浮点结果的误差；
5. 输出可被观察（串口 / 屏幕 / 上位机）。

完整过程与实测数据见 **[开发文档](docs/development-log.md)**。

演示视频：[**桌面演示（40 秒）**](docs/media/demo-desktop-20261128.mp4) —— 上电自检 → 静止判定 `still` → 拿起走动判定 `walk`。
（桌面/手持演示，非佩戴实测，详见文档 §4.22）

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
IMU_RANGE OK gyro=0x10 accel=0x10
BOARD_MODEL bytes=252 hash=AA7C1A9E
BOARD_INFER scores=[11,58,27] class=1 max_diff=0
IMU,t_ms,ax,ay,az,gx,gy,gz @50Hz
IMU,2126,-1256,14744,7440,1824,-328,-601
```

## 已实测结果

| 项目 | 实测值 |
|---|---|
| WHO_AM_I（地址 0x68） | `0x68`，读取 `hal=0 err=0x00000000` |
| 量程寄存器 | `GYRO=0x10`（±1000 °/s）、`ACCEL=0x10`（±8 g），读回一致 |
| 静置 az（正面朝上，±8 g） | +4310，即 **1.052 g**，标准差 21 LSB |
| 采样率 | **51.34 Hz**（间隔 19/19/20 ms） |
| 数据集 | MotionSense 24 人 / 22,155 个窗口 / 8 维特征 |
| 浮点验证准确率 | **98.04%**（跑步召回 369/437） |
| int8 验证准确率 | **97.99%**，与浮点一致率 99.95% |
| int8 模型体积 | 2,376 B |
| 板端单样本推理 | `scores=[11,58,27]`，与 TFLite **逐项相同**（max_diff=0） |
| FLASH 占用 | 70612 B / 1 MB（6.73%） |
| RAM 占用 | 21544 B / 128 KB（16.44%） |

校验脚本：`tools/check_imu_log.py`、`tools/check_imu_timing.py`、`tools/audit_motionsense.py`。

## 目录速览

| 目录 | 与任务四相关的部分 |
|---|---|
| `Core/Src/main.cpp` | 板级初始化、I²C/串口、量程设置、参数指纹、启动自检推理 |
| `config/motion_model_data.hpp` | int8 权重/偏置与量化参数（导出生成） |
| `config/motion_infer.hpp` | 自写定点推理（Q31 重定标，两层全连接） |
| `device/src/mpu6050.cpp` | MPU6050 驱动，一次读 14 字节（6 轴 + 温度） |
| `tools/` | 数据转换、特征、训练、量化、导出、校验脚本 |
| `artifacts/` | 浮点模型、int8 模型、预处理参数 |
| `ozone/` | 调试与烧录工程 |
| `docs/development-log.md` | 任务四开发文档（进度、实测、问题记录） |
| `docs/media/` | 演示视频（桌面演示 40 秒） |

## 调试注意事项

用调试器连接会暂停内核。若停在 I²C 传输中途，从机会拉住 SDA，导致 `I2C1.SR2.BUSY=1`，
此时 MCU 复位无效，**必须整板断电**。这是调试副作用，不是硬件故障。
