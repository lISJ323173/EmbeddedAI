# framework

STM32 embedded framework with a full FreeRTOS demo on STM32F407ZGT6 and a
UART/PWM porting baseline on STM32F103C8T6.

## Documentation

- [Architecture](docs/architecture.md) - layer responsibilities, servo data flow,
  health monitoring/IWDG, task priorities and stack evidence.
- [Interfaces](docs/interfaces.md) - UART, CAN, I2C and PWM contracts, command
  protocol and CRC, units and buffer lifetimes, plus the startup UART logger.
- [Porting and build guide](docs/porting.md) - F407/F103 build commands, ELF
  paths, J-Link targets, wiring and verification scope.
- [Acceptance record](docs/acceptance.md) - target/build matrix, measured
  evidence, commit traceability and remaining verification boundaries.

## Directory Structure

| Directory | Responsibility |
|---|---|
| `Core/` | STM32F407 startup, clock, peripheral initialization, interrupt vectors, `main` and IWDG |
| `Drivers/` | ST CMSIS and STM32F4 HAL (vendor code, read-only) |
| `Middlewares/` | FreeRTOS 10.3.1 and CMSIS-RTOS2 |
| `bsp/` | Pure-virtual UART, CAN, I2C, PWM and log interfaces; shared `bsp::Error` |
| `port/` | Chip implementations of BSP interfaces; the only code allowed to call peripheral HAL |
| `device/` | SG90, MPU6050, command parsers and checked CRC parser |
| `module/` | CRC-8/SMBUS and the 5 s servo command state |
| `application/` | Servo RX/output tasks, contexts, queue and health policy |
| `config/` | Reserved board parameter tables (currently empty) |
| `tests/host/` | Native UART, MPU6050 and CRC tests |
| `docs/` | Architecture, interface, porting and acceptance records |

## Quick Start

### F407 full demo

```powershell
cd E:\framework
cmake --preset Release
cmake --build --preset Release
```

ELF: `build/Release/framework.elf`; J-Link target `STM32F407ZG`; USB-TTL at
115200 8N1. The application accepts CRC frames such as `S50*B1\n` (the old plain
`S50\n` is rejected).

### F103 UART/PWM port

```powershell
cd E:\framework\port\stm32f1
cmake --preset Debug
cmake --build --preset Debug
```

ELF: `port/stm32f1/build/Debug/f103_ref.elf`; J-Link target `STM32F103C8`.

### Host tests

```powershell
cd E:\framework
cmake -S tests/host -B tests/host/build -G Ninja
cmake --build tests/host/build
ctest --test-dir tests/host/build --output-on-failure
```

Expected: 3/3 tests pass.

## Status

F407 Release `31,880 B` Flash / `21,512 B` RAM was downloaded and regressed; the
acceptance record is commit `a0c1025`. The F103 blocking UART + TIM3 PWM port is
commit `7976c73`. Startup UART logging (`[INFO] USART1 ready`,
`[WARN] IMU identity unsupported`) is verified. External CAN, genuine MPU6050
wake/sample, HC-05 wireless and concurrent runtime logging remain open items; see
[acceptance.md](docs/acceptance.md) and [interfaces.md](docs/interfaces.md).
