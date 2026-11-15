# Porting and Build Guide

## Prerequisites

- CMake 3.22 or newer and Ninja.
- ARM GNU Toolchain 14.3 (`arm-none-eabi-gcc/g++`).
- SEGGER J-Link software and a J-Link probe.
- A 3.3 V USB-TTL adapter for the serial console.
- SG90 with a separate 5 V supply and common ground; the servo signal is 3.3 V
  PWM and the external 5 V rail must not be replaced by the MCU 3.3 V rail.

## STM32F407ZGT6: Full Framework

### Build

From the repository root:

```powershell
cd E:\framework
cmake --preset Debug
cmake --build --preset Debug

# Release
cmake --preset Release
cmake --build --preset Release
```

| Build | ELF | Flash | RAM |
|---|---|---:|---:|
| Debug | `build/Debug/framework.elf` | 61,068 B | 21,520 B |
| Release | `build/Release/framework.elf` | 31,880 B | 21,512 B |

The Release build was downloaded and physically regressed; its evidence is
recorded in `docs/acceptance.md` and commit `a0c1025`. The F407 code path used for
that regression is the CRC-command baseline `4459ea7`.

### Clock and Peripherals

- HSI PLL: 168 MHz SYSCLK, AHB /1, APB1 /4 (42 MHz), APB2 /2 (84 MHz).
- USART1: `PA9` TX / `PA10` RX, 115200 8N1, no flow control.
- TIM3_CH1: `PA6`, 1 us tick (`PSC=83`), 20 ms period (`ARR=19999`), initial
  `CCR1=1500`.
- I2C1: `PB6` SCL / `PB7` SDA, 100 kHz, 7-bit.
- CAN1: `PD0` RX / `PD1` TX, internal loopback only at 500 kbit/s.
- IWDG: `Prescaler=64`, `Reload=1499`; refreshed only by `servo_health_task`.
- Startup logging: `port::UartLog` in `port/stm32f4/include/uart_log.hpp`
  writes two blocking lines before the scheduler starts:
  `[INFO] USART1 ready` and `[WARN] IMU identity unsupported`. Open the serial
  port before reset; each line is sent once. Runtime logging is not integrated
  because USART1 is owned by `servo_rx` after startup.

### J-Link and Serial Wiring

| Signal | F407 |
|---|---|
| J-Link VTref | 3.3 V |
| J-Link GND | GND |
| J-Link SWDIO | PA13 |
| J-Link SWCLK | PA14 |
| J-Link nRESET | NRST (optional) |
| USB-TTL TXD | PA10 (USART1_RX) |
| USB-TTL RXD | PA9 (USART1_TX) |
| USB-TTL GND | GND |
| SG90 signal | PA6 |
| SG90 V+ | external 5 V |
| SG90 GND | common GND |

Use J-Link device `STM32F407ZG` and SWD. Open the PC serial port at 115200 8N1.
Do not connect the USB-TTL VCC to the board when the board is separately powered.

The application accepts CRC frames such as `S50*B1\n`; the old plain `S50\n`
form is rejected.

## STM32F103C8T6: UART/PWM Port

### Build

The F103 project is standalone and has its own CMake preset:

```powershell
cd E:\framework\port\stm32f1
cmake --preset Debug
cmake --build --preset Debug
```

| Build | ELF | Flash | RAM |
|---|---|---:|---:|
| Debug | `port/stm32f1/build/Debug/f103_ref.elf` | 12,216 B | 2,128 B |

The CMake project enables C, C++ and ASM, compiles the port as C++17, and uses
`-mcpu=cortex-m3`. The generated F1 toolchain file supplies the same target flags
to C and C++.

### Clock and Peripherals

- HSI /2 x16: 64 MHz SYSCLK, AHB /1, APB1 /2 (32 MHz), APB2 /1 (64 MHz).
- APB1 timer clock 64 MHz; TIM3 `PSC=63` gives 1 us per count.
- USART1: `PA9` TX / `PA10` RX, 115200 8N1.
- TIM3_CH1: `PA6`, `PSC=63`, `ARR=19999`, initial `CCR1=1500`.
- SWD: `PA13` SWDIO, `PA14` SWCLK, 3.3 V and GND.

### J-Link and Serial Wiring

Use J-Link device `STM32F103C8` and SWD. The board must be separately powered;
connect only VTref (3.3 V sense), GND, SWDIO and SWCLK. Leave J-Link 5 V
unconnected. A typical 20-pin mapping is:

| J-Link 20-pin | F103 |
|---|---|
| 1 VTref | 3.3 V |
| 4 GND | GND |
| 7 SWDIO | PA13 |
| 9 SWCLK | PA14 |
| 15 nRESET | NRST (optional) |

USB-TTL:

```text
TTL TXD -> PA10
TTL RXD -> PA9
TTL GND -> GND
```

Open the serial port **before** pressing RESET: the banner is sent once at boot.
If the port is opened afterwards, the banner will be missed even though the
firmware is working.

### What the F103 Port Reuses

`port/stm32f1/src/f103_servo_demo.cpp` uses the same repository headers as F407:

- `bsp/include/bsp/iuart.hpp` (`F103Uart` implements the blocking interface locally)
- `bsp/include/bsp/ipwm.hpp` and the copied `port/stm32f1/include/stm32_pwm.hpp`
- `device/include/device/checked_servo_command_parser.hpp`
- `device/include/device/sg90.hpp`
- `module/include/module/crc8.hpp` through the checked parser

It does **not** include FreeRTOS, the health task, IWDG, or the 5 s
`module::ServoCommandState`; those remain F407 capabilities. The F103 demo loop
receives bytes, validates CRC, applies the position, and loops.

Measured F103 evidence (commit `7976c73`):

- Banner received on COM7: `F103 BSP READY\r\n`, 16 bytes.
- `TIM3 PSC/ARR/CCR1=63/19999/1500`, `CR1=0x81`, `CCER=0x1`.
- `S0*A3\n -> 1300`, bad CRC `S50*B0\n -> 1300`,
  `S100*25\r\n -> 1700`, `S50*B1\n -> 1500`.

The F103 reference project is also kept at `E:\gen_test\f103_ref` for
comparison, but the reproducible target is `E:\framework\port\stm32f1`.

## Host Tests

```powershell
cd E:\framework
cmake -S tests/host -B tests/host/build -G Ninja
cmake --build tests/host/build
ctest --test-dir tests/host/build --output-on-failure
```

Expected result is 3/3: UART parser, MPU6050 probe and CRC-8. The CRC test uses
compile-time assertions (empty input `0x00`, `123456789 -> 0xF4`, one-bit flip
rejected).

## Current Verification Boundaries

| Area | Verified now | Not verified |
|---|---|---|
| F407 Release | Build, download, CRC frames, timeout center, 10 s health run | Longer soak, external buses |
| F103 | Blocking UART RX/TX, CRC parser, TIM3 PWM, banner | FreeRTOS, 5 s timeout, IWDG |
| CAN | F407 internal loopback polling/FIFO0 IRQ | External transceiver, bus, second node |
| I2C/IMU | F407 blocking/IT reads; `WHO_AM_I=0x74` rejected | Genuine MPU6050 identity/wake/raw sample |
| Phone link | USB-TTL to SG90 | HC-05 wireless/disconnect behavior |
| Logging | `port::UartLog` startup INFO/WARN lines verified | Runtime/concurrent logging; USART1 arbitration after scheduler start |

## Troubleshooting

- **J-Link reports the wrong core**: ensure the probe is on the intended board
  and select `STM32F407ZG` or `STM32F103C8` accordingly.
- **No serial banner**: open the port before reset; verify TTL RXD is on the MCU
  TX pin (PA9 on both boards), TTL TXD is on PA10, and grounds are common.
- **Commands do not move the servo**: use CRC frames, not the old plain format;
  check `TIM3->CCR1` and the 5 V servo supply.
- **Health task resets after a fault test**: restore `inject_output_miss=false`
  and rebuild; the injected build intentionally stops refreshing the IWDG.
