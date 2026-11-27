# Embedded AI 开发文档（进行中）

> 任务：在 STM32F407 上部署小型神经网络，用 MPU6050 数据判断人体运动状态（走路 / 跑步 / 静止）。
> 本文档随开发推进增量更新，记录每一步的实测数据与结论。

## 0. 环境

| 项 | 值 |
|---|---|
| 开发板 | STM32F407ZGT6 |
| 传感器 | MPU6050（GY-521），I²C 地址 `0x68`（AD0 接地） |
| 接线 | SCL→PB6，SDA→PB7，VCC→3.3V，GND→GND |
| 调试器 | J-Link，SWD，1 MHz |
| 构建 | CMake preset `Debug`，arm-none-eabi-gcc 14.3.1，Ninja |
| 烧录 | Ozone 工程 `ozone/imu-motion-ai.jdebug`（或 J-Link Commander） |
| 串口 | USART1 (PA9/PA10)，115200 8N1 |

## P0.1 独立仓库与构建

从既有 F407 工程建立独立仓库，保留原有提交历史，分支 `main`。

```text
RAM:   21544 B / 128 KB  (16.44%)
FLASH: 67564 B / 1 MB    (6.44%)
产物：  build/Debug/framework.elf
```

常用命令：

```bash
cmake --preset Debug && cmake --build --preset Debug
```

## P0.2a 冷上电读取 WHO_AM_I

固件启动时分别对 `0x68`、`0x69` 读取 `WHO_AM_I`(0x75)，只读不写配置，并记录 HAL 状态与错误码。

实测（冷上电后按 RESET）：

```text
IMU_DIAG addr=0x68 hal=0 err=0x00000000 id=0x68
IMU_DIAG addr=0x69 hal=1 err=0x00000004 id=0xFF
[INFO] IMU WHO_AM_I=0x68
IMU,ax,ay,az,gx,gy,gz @20Hz
```

结论：

- `0x68` 返回 `HAL_OK`，身份值 `0x68`，与 MPU6050 手册一致；
- `0x69` 返回 `HAL_ERROR`，错误码 `0x04` 即 `HAL_I2C_ERROR_AF`（无应答），该地址无器件，符合预期；
- 检测条件无需修改。

### 调试注意事项（重要）

用调试器连接会暂停内核。若恰好停在 I²C 传输中途，从机会拉住 SDA，导致：

```text
I2C1 SR1 = 0x00000200  (ARLO，仲裁丢失)
I2C1 SR2 = 0x00000002  (BUSY，总线被占用)
```

此时 MCU 复位无效，必须**整板断电**才能释放总线。该现象不代表硬件故障，属调试副作用。

## P0.2b 六轴连续输出与姿态响应验证

方法：模块正面朝上平放 → 缓慢翻转 → 正面朝下平放，全程连续采集；按 `az` 符号自动切分两段，
用 `tools/check_imu_log.py` 校验六列均为整数且 `az` 变号。

```text
ROWS flat=30 flip=211
AZ flat=+17130 flipped=-15890
FLIP_SPAN ax=3780 ay=17984 az=35120 gx=25321 gy=3824 gz=4194
PASS: six integer columns and az sign reversal
```

结论：

- 六轴同时输出且均为整数，格式 `IMU,ax,ay,az,gx,gy,gz`；
- 翻转后 `az` 由 `+17130` 变为 `-15890`，符号反转，符合重力方向变化；
- 翻转过程中陀螺仪 `gx` 跨度达 25321（约 193 °/s），说明角速度轴对实际转动有响应；
- 静置时 `az` 约 17130，比理论值 16384 高约 4.5%，属器件量程误差，后续归一化可吸收。

待确认：本次 75 s 采集仅得 624 行（约 8 Hz），低于固件设计的 20 Hz。
P1 引入毫秒时间戳后可判定是采样丢失还是发送节奏偏慢。

## 数据文件

| 文件 | 内容 |
|---|---|
| `p0_flat.txt` | 正面朝上静置 30 行 |
| `p0_flip.txt` | 翻转过程 + 正面朝下静置 211 行 |
| `tools/check_imu_log.py` | 上述两段的一致性校验脚本 |
