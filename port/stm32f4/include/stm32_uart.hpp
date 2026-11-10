#pragma once

#include "bsp/iuart.hpp"
#include "stm32f4xx_hal.h"

namespace port {

class Stm32Uart final : public bsp::IUart {
public:
    explicit Stm32Uart(UART_HandleTypeDef& handle) noexcept;

    bsp::Error transmit(
        const std::uint8_t* data,
        std::size_t size,
        std::uint32_t timeout_ms
    ) noexcept override;

    bsp::Error receive(
        std::uint8_t* data,
        std::size_t size,
        std::uint32_t timeout_ms
    ) noexcept override;

private:
    UART_HandleTypeDef& handle_;
};

} // namespace port
