# Framework Acceptance Record

Current F103 gimbal measurements are recorded in [gimbal.md](gimbal.md). Earlier results below describe the framework baseline.

Updated: 2026-11-16

## Targets

| Target | Role | Build |
|---|---|---|
| STM32F407ZGT6 | Full framework demo: FreeRTOS, IWDG, SG90, USB-TTL, CAN, I2C/IMU | Debug, Release |
| STM32F103C8T6 | UART/PWM port evidence, same BSP/device/CRC layers | Debug (C++17) |
| Host (MSYS2 UCRT64) | Parser/CRC/MPU6050 unit tests | Native tests |

## Build Results

| Build | Result | Flash | RAM | Artifact |
|---|---|---:|---:|---|
| F407 Debug | Pass | 61,912 B | 21,520 B | `build/Debug/framework.elf` |
| F407 Release (pre-log baseline) | Pass | 31,576 B | 21,512 B | `build/Release/framework.elf` |
| F407 Release (final, startup log) | Pass | 31,880 B | 21,512 B | `build/Release/framework.elf` |
| F103 Debug | Pass | 12,216 B | 2,128 B | `port/stm32f1/build/Debug/f103_ref.elf` |
| Host CTest | Pass 5/5 | - | - | `tests/host/build` |

The F407 Release ELF was built with `cmake --preset Release`, downloaded with
J-Link, and physically regressed on the F407. The final logging Release is
`2d730b6` (`31,880 B`); `a0c1025` is the pre-log baseline (`31,576 B`). The
release build, memory usage, serial command behavior, timeout center, health
monitor and reset status are recorded below.

## Acceptance Matrix

| Area | Target / Build | Verification | Result | Commit |
|---|---|---|---|---|
| Clock, UART1, TIM3 PWM baseline | F407 Debug | Physical: `CCR1` and UART traffic | Pass | `4d7558d` |
| SG90 position control | F407 Debug | Physical: `S0`/`S50` -> `CCR1=1300/1500` | Pass | `4d7558d` |
| USB-TTL text command chain | F407 Debug | Physical: COM7 at 115200 8N1 | Pass | `9517acc` |
| HAL timebase on TIM6 | F407 Debug | Physical: `uwTick` advancing with SysTick used by FreeRTOS | Pass | `1e9d603` |
| FreeRTOS scheduler | F407 Debug | Physical: tasks run, tick advances | Pass | `43fece2` |
| Dual tasks and command queue | F407 Debug | Physical: RX enqueue/output counters and `CCR1` | Pass | `7ec1516` |
| Health monitor + IWDG | F407 Debug | Physical: windows increment; injected miss resets via IWDG | Pass | `2032d5d` |
| Application task boundary | F407 Debug | Physical: same queue/CCR regression, health clean | Pass | `0f6fa95` |
| 5 s command timeout / center | F407 Debug | Physical: bad CRC does not extend hold; both timeouts center | Pass | `380db49` |
| CRC-8/SMBUS | Host | Compile-time vectors: empty `0x00`, `123456789` -> `0xF4`; bit flip rejected | Pass | `0018f14` |
| Servo parser + smoke | Host | CTest 5/5; parser accepts S0/S100, rejects S101/old format; C++17 smoke | Pass | this change |
| CRC-required command frames | F407 Debug | Physical: valid frames applied; bad CRC/range/old format rejected | Pass | `4459ea7` |
| F103 UART + PWM port | F103 Debug | Physical: `F103 BSP READY\r\n`; frames -> `1300/1300/1700/1500` | Pass | `7976c73` |
| F407 Release regression | F407 Release | Physical: CRC frames, timeout, 10 s health run | Pass | `2d730b6` |
| Startup UART logging | F407 Release | Physical: INFO and WARN each once; S0/S50 and health clean | Pass | `2d730b6` |

## Recorded Evidence

### F407 health and IWDG

- Normal 10 s run: health windows `20`, `servo_health_missing=0`.
- Injected output miss: first missed window reported `0x02`; health task stopped refreshing.
- Before reset: `boot_reset_csr=0x14000000`, `IWDGRSTF=0`.
- After automatic reset: `boot_reset_csr=0x24000000`, `IWDGRSTF=0x20000000`.
- No manual reset was issued between the two observations.

### CRC command frames (F407 Debug)

| Frame | Queue/execution | `CCR1` |
|---|---:|---:|
| `S0*A3\n` | `1/1` | `1300` |
| `S50*B0\n` (bad CRC) | `1/1` | `1300` |
| `S101*22\n` (out of range) | `1/1` | `1300` |
| `S50\n` (old format) | `1/1` | `1300` |
| `S100*25\r\n` | `2/2` | `1700` |
| `S50*B1\n` | `3/3` | `1500` |

A later `S0*A3\n` followed by bad CRC `S50*B0\n` still centered at `1500` about
5 s after `S0`, without increasing the counters.

### F103 port (Release-independent baseline)

- Startup banner: `F103 BSP READY\r\n` on COM7, 115200 8N1.
- `TIM3 PSC/ARR/CCR1 = 63/19999/1500`, `CR1=0x81`, `CCER=0x1`.
- Frame results: `S0*A3 -> 1300`, `S50*B0 -> 1300`, `S100*25 -> 1700`,
  `S50*B1 -> 1500`.
- `HAL_UART_Transmit()` returned `HAL_OK`; a synchronized serial capture was
  required because the banner is sent once at boot.

### F407 Release regression

- Build/download: `31,880 B Flash`, `21,512 B RAM`, ELF downloaded with J-Link.
- Startup logging: COM7 was opened before reset; exactly
  `[INFO] USART1 ready\r\n` and `[WARN] IMU identity unsupported\r\n` were
  received, one each.
- `S0*A3\n` -> `CCR1=1300`, queue/execution `1/1`.
- Bad CRC `S50*B0\n` ~3 s later -> rejected; `CCR1` stayed `1300`.
- About 5 s after `S0`, automatic center -> `CCR1=1500`; counters stayed `1/1`.
- `S100*25\r\n` -> `CCR1=1700`, counters `2/2`.
- `S50*B1\n` -> `CCR1=1500`, counters `3/3`.
- Additional 10 s idle: health windows `3 -> 42`, `servo_health_missing=0`,
  `uwTick=21567`, `CCR1=1500`.
- On the logged build, a fresh reset reached health window `1`; then
  `S0`/`S50` gave `1300/1500` and windows advanced `13 -> 20` with `missing=0`.
- `Error_Handler` and `HardFault_Handler` breakpoints were not hit; no unexpected
  reset was observed.

## Acceptance Boundaries

| Item | Current conclusion |
|---|---|
| CAN | F407 internal-loopback polling and FIFO0 interrupt receive pass; external CAN bus and a second node remain unverified. |
| I2C / IMU | Blocking and interrupt reads were measured. The connected module reports `WHO_AM_I=0x74`, so the device layer rejects it as unsupported. MPU6050 wake and raw-sample paths are covered by host simulation, not by a genuine MPU6050. |
| Phone control | USB-TTL to SG90 command chain is measured. HC-05 wireless link and disconnection behavior remain unverified. |
| FreeRTOS / IWDG | Dual tasks, queue, health monitor and fault-injection reset are measured on F407. The reset evidence is the `IWDGRSTF` transition above. |
| F103 port | Commit `7976c73`; UART blocking RX/TX and TIM3 PWM measured. FreeRTOS and 5 s timeout on F103 are not claimed; those remain F407 results. |
| F407 Release | Build, download and physical regression pass (`31,880 B Flash / 21,512 B RAM`); startup INFO/WARN logs, S0/S50 and health evidence above. |
| Logging | `port::UartLog` over `bsp::IUart` is implemented in `port/stm32f4/include/uart_log.hpp` and called twice before the scheduler. Startup logging is verified; runtime/concurrent logging is not integrated because USART1 belongs to `servo_rx` after startup. |

## Reproduction Commands

```powershell
# F407 Release
cd E:\framework
cmake --preset Release
cmake --build --preset Release

# Host tests
cmake -S tests/host -B tests/host/build -G Ninja
cmake --build tests/host/build
ctest --test-dir tests/host/build --output-on-failure

# F103 port
cd E:\framework\port\stm32f1
cmake --preset Debug
cmake --build --preset Debug
```
