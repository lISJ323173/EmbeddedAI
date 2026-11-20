# Architecture

Current F103 gimbal architecture and progress are documented in [gimbal.md](gimbal.md).

## Purpose

The framework separates chip-specific startup, peripheral access, device behavior,
reusable algorithms and application policy. The same `bsp`, `device` and `module`
code is used by the STM32F407 full demo and by the STM32F103 UART/PWM port.

## Layers

| Path | Responsibility | Allowed dependencies |
|---|---|---|
| `Core/` | STM32F407 startup, clock/pin/peripheral initialization, interrupt vectors, `main.cpp` composition root, HAL timebase and IWDG | CMSIS, HAL, `port`, `bsp`, `device`, `application` |
| `Drivers/` | ST CMSIS and STM32F4 HAL, treated as read-only vendor code | CMSIS |
| `Middlewares/Third_Party/FreeRTOS/` | FreeRTOS 10.3.1 kernel, CMSIS-RTOS2 wrapper and Cortex-M4F port | FreeRTOS/CMSIS |
| `bsp/` | Pure-virtual board-support interfaces and shared error type | C++ standard library only |
| `port/stm32f4/` | STM32F4 implementations of UART, CAN, I2C and PWM; the only F407 code that calls HAL peripherals directly | `bsp`, HAL |
| `port/stm32f1/` | STM32F103 UART/PWM demo and F1 PWM implementation | `bsp`, `device`, `module`, HAL |
| `device/` | Hardware-facing drivers and command parsers: `Sg90`, `Mpu6050`, `ServoCommandParser`, `CheckedServoCommandParser` | `bsp`, `module` |
| `module/` | Chip-independent algorithms/protocol helpers: CRC-8/SMBUS and the 5 s command state | C++ standard library only |
| `application/` | Servo RX/output task bodies, contexts, queue/event wiring | `bsp`, `device`, `module`, CMSIS-RTOS2 |
| `config/` | Reserved for board-level pin/ID/parameter tables; currently empty | - |
| `tests/host/` | Native UART parser, MPU6050 and CRC tests | `bsp`, `device`, `module` |
| `docs/` | Architecture, interface, porting and acceptance records | - |

The dependency direction is:

```text
application -> device -> bsp
      |           |
      +---------> module
Core/port -> bsp + vendor HAL
```

`bsp` has no HAL include. `module` has no HAL or RTOS include. `port` is the
translation layer between those interfaces and STM32 peripherals.

## Servo Command Data Flow

```text
USB-TTL / HC-05 (future)
        |
        | USART1 PA9/PA10, 115200 8N1
        v
port::Stm32Uart                 bsp::IUart (blocking receive)
        |
        | one byte, 10 ms timeout
        v
application::servo_rx_task
        |
        v
device::CheckedServoCommandParser
        |-- split "S50*B1"
        |-- device::ServoCommandParser validates S + 0..100
        |-- module::crc8_smbus("S50") == 0xB1
        |
        | bsp::Error::Ok only
        v
osMessageQueue (4 entries, int32_t position)
        |
        v
application::servo_output_task
        |-- module::ServoCommandState: 5000 ticks hold, then center
        |-- device::Sg90: 0..100 -> 1300..1700 us
        v
port::Stm32PwmCh1              bsp::IPwm
        |
        | TIM3_CH1 / PA6, 1 us tick, 20 ms period
        v
SG90 servo
```

`servo_rx_enqueued` counts successful queue submissions. `servo_output_applied`
counts only commands taken from the queue and applied; a timeout-generated center
does not increment it. The 5 s timeout is measured in RTOS ticks and uses unsigned
tick subtraction, so a single wrap inside the window is harmless.

## Health Monitoring and IWDG

```text
servo_rx_task ---- set rx_alive (bit 0) ----+
                                             |
servo_output_task -- set output_alive (bit 1)-+--> osEventFlags
                                             |
servo_health_task -- every 500 ticks --------+
        |
        | osEventFlagsClear(required_alive)
        | missing = required_alive & ~seen
        v
 missing != 0 ? ---- yes ---> loop osDelay(100), never refresh IWDG
        |
        no
        v
 HAL_IWDG_Refresh(&hiwdg), ++servo_health_windows
```

- `servo_rx_task` reports on a successful byte receive and on a receive timeout.
- `servo_output_task` reports every loop iteration after queue handling, unless
  the test-only `suppress_alive` flag is set.
- `servo_health_task` is the only IWDG refresher. The IWDG is initialized inside
  that task after the scheduler starts, with `Prescaler=IWDG_PRESCALER_64` and
  `Reload=1499`; nominal timeout is about 3 s at LSI 32 kHz.
- If a required bit is absent, the health task deliberately stops refreshing.
  It keeps delaying so lower-priority work can run, but the next IWDG reset is
  unavoidable. A later report cannot clear the already-recorded miss.
- `inject_output_miss` is a compile-time test switch in `Core/Src/main.cpp`; the
  delivered firmware uses `false`.
- Reset evidence: before the injected reset `boot_reset_csr=0x14000000` and
  `IWDGRSTF=0`; after the automatic reset `boot_reset_csr=0x24000000` and
  `IWDGRSTF=0x20000000`.

## Tasks

CMSIS-RTOS2 priorities are shown with their numeric values. Stack sizes are bytes
because `osThreadAttr_t::stack_size` is in bytes.

| Task | Priority | Stack | Wake condition | Work | Measured stack headroom |
|---|---:|---:|---|---|---:|
| `servo_rx` | `osPriorityNormal` (24) | 1024 B | USART1 byte or 10 ms receive timeout | Parse byte, queue valid command, report alive; sample stack on timeout | 764 B free |
| `servo_output` | `osPriorityAboveNormal` (32) | 1024 B | Queue item or 100-tick queue timeout (100 ms) | Apply command, hold/center state, report alive | 824 B free |
| `servo_health` | `osPriorityHigh` (40) | 1024 B | `osDelayUntil` every 500 ticks | Clear alive flags, detect missing tasks, refresh IWDG | Not yet measured |

The 764 B and 824 B values are **minimum free stack** measured on the L8-5
Debug dual-task firmware, not estimates for the current Release build. The
health-task headroom and all current Release stack headroom remain to be measured.

The RX and output task bodies live in `application/src/servo_tasks.cpp`.
`servo_health_task` and IWDG initialization live in the `Core/Src/main.cpp`
composition root because they own board-level fail-stop policy.

Priority rationale:

- `servo_health` is highest so the watchdog decision is not delayed by command
  parsing or PWM updates.
- `servo_output` is above `servo_rx` so a queued position change updates the
  compare register promptly, keeping pulse timing jitter low.
- `servo_rx` is lowest because byte reception is already buffered by the UART and
  a 10 ms timeout still allows timely queueing.

## Startup and Ownership

`Core/Src/main.cpp` is the F407 composition root:

1. Save/clear reset flags, then `HAL_Init()`.
2. Configure the 168 MHz HSI PLL clock, GPIO and peripherals (I2C1, CAN1, TIM3,
   USART1, DMA).
3. Start PWM and command the initial 50% (1500 us) position.
4. Initialize the kernel, event flags and queue.
5. Create `servo_rx`, `servo_output` and `servo_health` with static contexts.
6. Start the scheduler. The health task initializes the IWDG.

The UART, PWM and I2C/CAN handles are static because the task contexts and
peripheral state must outlive `main()`. Queue entries are `int32_t` positions, so
the consumer owns no pointer into the producer's stack.

## Deliberate Scope Limits

- The F407 full demo uses FreeRTOS, the 5 s state machine and IWDG.
- The F103 port demonstrates blocking UART, the CRC parser, `Sg90` and TIM3 PWM;
  it does not claim FreeRTOS or the 5 s timeout.
- CAN is currently internal loopback at the F407; external bus wiring and a second
  node are not part of the acceptance evidence.
- `bsp::ILog` is implemented for startup by `port::UartLog`; it is blocking and
  startup-only, not a concurrent runtime logger. See
  [interfaces.md](interfaces.md#logging-interface) and the personal
  [取舍与踩坑记录](notes.md).
