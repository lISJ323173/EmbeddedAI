#include "stm32_uart.hpp"

#include <limits>

namespace port {

Stm32Uart::Stm32Uart(UART_HandleTypeDef& handle) noexcept
    : handle_(handle) {}

namespace {

bsp::Error map_status(HAL_StatusTypeDef status) noexcept {
    switch (status) {
    case HAL_OK:      return bsp::Error::Ok;
    case HAL_BUSY:    return bsp::Error::Busy;
    case HAL_TIMEOUT: return bsp::Error::Timeout;
    default:          return bsp::Error::HardwareFailure;
    }
}

bool valid_buffer(const void* data, std::size_t size) noexcept {
    return data != nullptr &&
           size > 0 &&
           size <= std::numeric_limits<std::uint16_t>::max();
}

} // namespace

bsp::Error Stm32Uart::transmit(
    const std::uint8_t* data,
    std::size_t size,
    std::uint32_t timeout_ms) noexcept {
    if (!valid_buffer(data, size))
        return bsp::Error::InvalidArgument;

    if (handle_.gState == HAL_UART_STATE_RESET)
        return bsp::Error::NotInitialized;

    return map_status(HAL_UART_Transmit(
        &handle_, data,
        static_cast<std::uint16_t>(size), timeout_ms));
}

bsp::Error Stm32Uart::receive(
    std::uint8_t* data,
    std::size_t size,
    std::uint32_t timeout_ms) noexcept
{
    if (!valid_buffer(data, size)) {
        return bsp::Error::InvalidArgument;
    }

    if (handle_.RxState == HAL_UART_STATE_RESET) {
        return bsp::Error::NotInitialized;
    }

    return map_status(HAL_UART_Receive(
        &handle_, data,
        static_cast<std::uint16_t>(size), timeout_ms));
}

} // namespace port
