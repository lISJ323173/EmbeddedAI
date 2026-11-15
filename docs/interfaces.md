# Interfaces

All public interfaces live under `E:\framework\bsp\include\bsp\`. They use
`bsp::Error` and do not expose HAL types. Implementations live under `port/`.

## Common Error Type

`bsp/include/bsp/error.hpp` defines:

```cpp
enum class Error : std::uint8_t {
    Ok, NotInitialized, InvalidArgument, Busy, Timeout,
    ChecksumMismatch, HardwareFailure, Unsupported
};
```

| Error | Meaning at the interface boundary |
|---|---|
| `Ok` | Operation completed as requested |
| `NotInitialized` | Peripheral/handle is not ready |
| `InvalidArgument` | Null/zero/oversized buffer, invalid address, frame or pulse |
| `Busy` | Another transfer/operation owns the resource, or no mailbox |
| `Timeout` | The requested timeout expired |
| `ChecksumMismatch` | Command CRC does not match the frame prefix |
| `HardwareFailure` | HAL or device-level failure |
| `Unsupported` | Valid frame/device form that this port does not support |

The F407 HAL adapters map `HAL_OK -> Ok`, `HAL_BUSY -> Busy`,
`HAL_TIMEOUT -> Timeout`, and any other HAL status to `HardwareFailure`.

## UART

### Interface

`bsp/include/bsp/iuart.hpp`:

```cpp
virtual Error transmit(const std::uint8_t* data, std::size_t size,
                       std::uint32_t timeout_ms) noexcept = 0;
virtual Error receive(std::uint8_t* data, std::size_t size,
                      std::uint32_t timeout_ms) noexcept = 0;
```

- `size` is a byte count; valid range is `1..65535`.
- `timeout_ms` is milliseconds.
- Blocking calls: the caller's buffer must remain valid until the call returns.
- `nullptr`, zero size or size above 65535 returns `InvalidArgument`.
- A reset HAL UART returns `NotInitialized`.

`bsp/include/bsp/iuart_async.hpp` adds interrupt/DMA operation:

```cpp
Error start_transmit(const std::uint8_t* data, std::size_t size,
                     UartTransferMode mode) noexcept;
Error start_receive(std::uint8_t* data, std::size_t size,
                    UartTransferMode mode) noexcept;
void set_callback(UartCallback callback, void* context) noexcept;
```

`UartTransferMode` is `Interrupt` or `Dma`; events are `TxComplete`,
`RxComplete` and `Error`. For asynchronous calls the buffer must remain valid
until the completion callback. A second start while one is active returns `Busy`.
`Stm32Uart::recover()` is a synchronous recovery helper and must be called
outside interrupt context by the sole owner of that UART.

### Implementations

| Target | Implementation | Modes |
|---|---|---|
| F407 | `port/stm32f4/include/stm32_uart.hpp`, `port/stm32f4/src/stm32_uart.cpp` | Blocking, IT and DMA |
| F103 | `port/stm32f1/src/f103_servo_demo.cpp` (`F103Uart`) | Blocking only |

F407 USART1 is on `PA9` TX / `PA10` RX, 115200 8N1, no flow control. DMA is
USART1 TX on DMA2 Stream7/Channel4 and RX on DMA2 Stream2/Channel4. The F103
port uses the same PA9/PA10 pins and 115200 8N1.

## CAN

### Interface

`bsp/include/bsp/ican.hpp` defines `CanFrame` and `ICan`:

```cpp
struct CanFrame {
    std::uint16_t id{};
    std::uint8_t dlc{};
    std::uint8_t data[8]{};
};

virtual Error try_send(const CanFrame& frame) noexcept = 0;
virtual Error receive(CanFrame& frame, std::uint32_t timeout_ms) noexcept = 0;
```

`ICanIrqRx` adds:

```cpp
virtual Error arm_receive_irq() noexcept = 0;
virtual Error wait_receive_irq(CanFrame& frame,
                               std::uint32_t timeout_ms) noexcept = 0;
```

- `id` is a standard 11-bit identifier, range `0..0x7FF`.
- `dlc` is `0..8`; `data` is copied by value.
- `timeout_ms` is milliseconds.
- `try_send` is non-blocking: `Busy` if all TX mailboxes are occupied;
  `NotInitialized` if the peripheral is not listening.
- Polling `receive` accepts standard data frames from FIFO0. Extended/remote
  frames return `Unsupported`; a malformed/DLC failure returns
  `HardwareFailure`; timeout returns `Timeout`.
- `arm_receive_irq` enables the FIFO0 pending notification once. A second arm
  returns `Busy`; `wait_receive_irq` waits for the notification, disables it and
  reads one frame with a zero timeout. Calling wait without arm returns
  `InvalidArgument`.

Implementation: `port/stm32f4/include/stm32_can.hpp` and
`port/stm32f4/src/stm32_can.cpp`. Current F407 firmware configures CAN1 on
`PD0/PD1` in internal loopback at 500 kbit/s (APB1 42 MHz, prescaler 6,
BS1 11 TQ, BS2 2 TQ). External transceiver/bus and a second node are not yet
validated.

## I2C

### Interface

`bsp/include/bsp/ii2c.hpp`:

```cpp
virtual Error read_register(std::uint8_t address_7bit, std::uint8_t reg,
                            std::uint8_t* data, std::size_t size,
                            std::uint32_t timeout_ms) noexcept = 0;
virtual Error write_register(std::uint8_t address_7bit, std::uint8_t reg,
                             std::uint8_t value,
                             std::uint32_t timeout_ms) noexcept = 0;
```

- `address_7bit` is the unshifted 7-bit address, accepted range `0x08..0x77`.
  The implementation shifts it left by one before calling HAL.
- `reg` is an 8-bit register address; reads use `I2C_MEMADD_SIZE_8BIT`.
- `size` is a byte count, `1..65535`; `timeout_ms` is milliseconds.
- Blocking read buffer must remain valid until return.

`II2cAsync` adds:

```cpp
Error start_read_register(std::uint8_t address_7bit, std::uint8_t reg,
                          std::uint8_t* data, std::size_t size) noexcept;
Error poll_read_result() noexcept;
```

`start_read_register` returning `Ok` means the transfer started. The caller's
buffer must stay valid until `poll_read_result()` returns `Ok` or
`HardwareFailure`. `Busy` means still active; `InvalidArgument` means there is no
active transaction.

Implementation: `port/stm32f4/include/stm32_i2c.hpp` and
`port/stm32f4/src/stm32_i2c.cpp`; I2C1 is `PB6/PB7`, 100 kHz, 7-bit addressing.
The connected IMU reports `WHO_AM_I=0x74`; `device/include/device/mpu6050.hpp`
rejects that identity as `Unsupported`, so the hardware acceptance record does
not claim a genuine MPU6050.

## PWM

`bsp/include/bsp/ipwm.hpp`:

```cpp
virtual Error start() noexcept = 0;
virtual Error set_pulse_us(std::uint32_t pulse_us) noexcept = 0;
```

- Pulse width is in microseconds.
- F407 and F103 TIM3 are configured for a 1 us timer tick and a 20 ms period
  (`PSC=83` on F407; `PSC=63` on F103; `ARR=19999` in both).
- `start()` starts channel 1 once; a second start returns `Busy`.
- `set_pulse_us` before start returns `NotInitialized`; a pulse greater than
  `ARR` returns `InvalidArgument`.

Implementations: `port/stm32f4/src/stm32_pwm.cpp` and
`port/stm32f1/src/stm32_pwm.cpp`. Both drive TIM3_CH1 on `PA6`.

## Servo Command Protocol

Frame form:

```text
S<position>*<CRC8><LF>
S<position>*<CRC8><CR><LF>
```

Example:

```text
S50*B1\n
```

- `position` is decimal, `0..100`, at most three digits. The parser rejects
  values above 100, non-digits, and malformed frames.
- CRC is exactly two **uppercase** hexadecimal digits.
- CRC-8/SMBUS parameters: polynomial `0x07`, initial `0x00`, no reflection,
  final XOR `0x00`.
- CRC covers only the ASCII bytes before `*`; for `S50*B1` it covers `S`, `5`,
  `0`. It does not cover `*`, the CRC digits, `CR` or `LF`.
- `LF` and `CRLF` are accepted; a `CR` anywhere except the final position before
  `LF` is invalid.
- An invalid format, out-of-range position or bad CRC is not queued and does not
  refresh the servo hold timer. The next valid frame is still accepted.
- A valid frame resets the 5 s hold timer. With no new valid frame for 5000 ticks
  (5 s at 1000 Hz), the output task commands 50% (1500 us). The check occurs on
  the next 100 ms queue timeout, so the center may appear up to one check late.

Position mapping in `device/include/device/sg90.hpp` is linear
`0..100 -> 1300..1700 us`; 50 maps to 1500 us. The F103 demo uses the same parser, CRC and `Sg90` mapping, but does not
run FreeRTOS or the 5 s state machine.

## Logging Interface

`bsp/include/bsp/ilog.hpp` defines the interface:

```cpp
enum class LogLevel : std::uint8_t { Debug, Info, Warning, Error };

class ILog {
public:
    virtual ~ILog() = default;
    virtual Error write(LogLevel level, const char* message) noexcept = 0;
};
```

**Implementation:** `port/stm32f4/include/uart_log.hpp` defines
`port::UartLog`, which formats one line as `[LEVEL] message\r\n` and submits it
with one blocking `bsp::IUart::transmit(..., 100U)` call. `UartLog` supports
messages up to 80 bytes and rejects null/empty messages, embedded `CR`/`LF`, and
overlong input with `InvalidArgument`; UART errors are returned unchanged.

Current call sites in `Core/Src/main.cpp`:

1. Immediately after `MX_USART1_UART_Init()`:
   `[INFO] USART1 ready`.
2. After the `Mpu6050::probe()` identity check and before `MX_CAN1_Init()`:
   `[WARN] IMU identity unsupported`.

Both calls run **before the scheduler starts**. This is deliberate: the first
version is startup-only, has a single caller, uses blocking output, and does not
race the `servo_rx` task for USART1. A failed `write()` enters `Error_Handler()`.

USART1 is also used by `servo_rx` after the scheduler starts; `UartLog` is not a
concurrent runtime logger. A later runtime log path needs ownership arbitration,
a queue or a dedicated UART/log sink.

Measured on the F407 Release build: the port received exactly
`[INFO] USART1 ready\r\n` followed by `[WARN] IMU identity unsupported\r\n`
(one each) when COM7 was opened before reset. `S0/S50` still produced
`CCR1=1300/1500` and the health window continued with `missing=0`.
