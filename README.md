# 嵌入式框架

基于 STM32 的嵌入式框架：在 STM32F407ZGT6 上运行完整的 FreeRTOS 演示，在 STM32F103C8T6 上提供 UART/PWM 移植基线。

## 文档

> 文档语言约定：README 和 `docs/notes.md` 用中文，`docs/` 下的架构/接口/移植/验收文档保留英文，便于与代码接口名、日志和提交记录对应。

- [架构设计](docs/architecture.md)：分层职责、舵机数据流、健康监测/IWDG、任务优先级与栈余量证据。
- [接口说明](docs/interfaces.md)：UART、CAN、I2C、PWM 接口约定，指令协议与 CRC，地址/超时/缓冲区生命周期，以及启动期 UART 日志。
- [移植与构建指南](docs/porting.md)：F407/F103 构建命令、ELF 路径、J-Link 目标、接线和验证范围。
- [验收记录](docs/acceptance.md)：目标板/构建矩阵、实测证据、提交追溯和剩余待验证边界。
- [取舍与踩坑记录](docs/notes.md)：为什么这样拆分、参数取舍、真实调试过程中遇到的坑。

## 目录结构

| 目录 | 职责 |
|---|---|
| `Core/` | STM32F407 启动、时钟、外设初始化、中断向量、`main` 和 IWDG |
| `Drivers/` | ST CMSIS 与 STM32F4 HAL（厂商代码，只读） |
| `Middlewares/` | FreeRTOS 10.3.1 与 CMSIS-RTOS2 |
| `bsp/` | UART、CAN、I2C、PWM 和日志的纯虚接口，以及统一的 `bsp::Error` |
| `port/` | BSP 接口的芯片实现；外设访问的 HAL 适配集中在这里，`Core/` 仍保留启动、时钟、中断和板级 fail-stop 所需的 HAL 调用 |
| `device/` | SG90、MPU6050、指令解析器和带 CRC 校验的解析器 |
| `module/` | CRC-8/SMBUS 与 5 秒舵机指令状态机 |
| `application/` | 舵机接收/输出任务、上下文和命令队列 |
| `config/` | 预留的板级参数表（当前为空） |
| `tests/host/` | 主机侧 UART、MPU6050 与 CRC 测试 |
| `docs/` | 架构、接口、移植和验收文档 |

## 快速开始

### F407 完整演示

```powershell
cd E:\framework
cmake --preset Release
cmake --build --preset Release
```

ELF 路径：`build/Release/framework.elf`；J-Link 目标：`STM32F407ZG`；USB-TTL 使用 115200 8N1。应用只接受带 CRC 的指令帧，例如 `S50*B1\n`；旧的纯文本格式 `S50\n` 会被拒绝。

### F103 UART/PWM 移植

```powershell
cd E:\framework\port\stm32f1
cmake --preset Debug
cmake --build --preset Debug
```

ELF 路径：`port/stm32f1/build/Debug/f103_ref.elf`；J-Link 目标：`STM32F103C8`。

### 主机测试

```powershell
cd E:\framework
cmake -S tests/host -B tests/host/build -G Ninja
cmake --build tests/host/build
ctest --test-dir tests/host/build --output-on-failure
```

预期结果：5/5 测试通过。

## 当前状态

F407 Release 固件最终占用 `31,880 B` Flash、`21,512 B` RAM（加入启动日志后的 `2d730b6`）；加入日志前的基线为 `31,576 B`（`a0c1025`）。F103 阻塞式 UART + TIM3 PWM 移植基线见提交 `7976c73`。

启动期 UART 日志已验证：

```text
[INFO] USART1 ready
[WARN] IMU identity unsupported
```

外部 CAN 总线、真实 MPU6050 的唤醒与采样、HC-05 无线链路，以及运行期并发日志仍待完成。详情见 [验收记录](docs/acceptance.md) 和 [接口说明](docs/interfaces.md)。
