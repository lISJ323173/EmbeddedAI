#include "main.h"
#include "bsp/iuart.hpp"
#include "device/checked_servo_command_parser.hpp"
#include "device/sg90.hpp"
#include "stm32_pwm.hpp"

namespace {

bsp::Error map_status(HAL_StatusTypeDef status) noexcept
{
    switch (status) {
    case HAL_OK:      return bsp::Error::Ok;
    case HAL_BUSY:    return bsp::Error::Busy;
    case HAL_TIMEOUT: return bsp::Error::Timeout;
    default:          return bsp::Error::HardwareFailure;
    }
}

class F103Uart final : public bsp::IUart {
public:
    explicit F103Uart(UART_HandleTypeDef& uart) noexcept : uart_(uart) {}

    bsp::Error transmit(const std::uint8_t* data, std::size_t size,
                        std::uint32_t timeout_ms) noexcept override
    {
        if (data == nullptr || size == 0U || size > 0xFFFFU) {
            return bsp::Error::InvalidArgument;
        }
        if (uart_.gState == HAL_UART_STATE_RESET) {
            return bsp::Error::NotInitialized;
        }
        return map_status(HAL_UART_Transmit(
            &uart_, const_cast<std::uint8_t*>(data),
            static_cast<std::uint16_t>(size), timeout_ms));
    }

    bsp::Error receive(std::uint8_t* data, std::size_t size,
                       std::uint32_t timeout_ms) noexcept override
    {
        if (data == nullptr || size == 0U || size > 0xFFFFU) {
            return bsp::Error::InvalidArgument;
        }
        if (uart_.RxState == HAL_UART_STATE_RESET) {
            return bsp::Error::NotInitialized;
        }
        return map_status(HAL_UART_Receive(
            &uart_, data, static_cast<std::uint16_t>(size), timeout_ms));
    }

private:
    UART_HandleTypeDef& uart_;
};

} // namespace

extern "C" void run_f103_servo_demo(UART_HandleTypeDef* uart_handle,
                                    TIM_HandleTypeDef* timer_handle)
{
    F103Uart uart(*uart_handle);
    port::Stm32PwmCh1 pwm(*timer_handle);
    device::Sg90 servo(pwm);
    device::CheckedServoCommandParser parser;
    unsigned position = 50U;

    if (pwm.start() != bsp::Error::Ok) {
        Error_Handler();
    }

    constexpr std::uint8_t banner[] = "F103 BSP READY\r\n";
    if (uart.transmit(banner, sizeof(banner) - 1U, 100U) != bsp::Error::Ok) {
        Error_Handler();
    }

    for (;;) {
        std::uint8_t byte{};
        const auto status = uart.receive(&byte, 1U, 10U);
        if (status == bsp::Error::Timeout) {
            continue;
        }
        if (status != bsp::Error::Ok) {
            Error_Handler();
        }
        if (parser.push(byte, position) == bsp::Error::Ok &&
            servo.set_position_percent(static_cast<std::int32_t>(position))
                != bsp::Error::Ok) {
            Error_Handler();
        }
    }
}
