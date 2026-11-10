#pragma once

#include "stm32f4xx_hal.h"
#include "bsp/error.hpp"

namespace port {

class Stm32Uart;

namespace uart_router {

bsp::Error register_uart(
    UART_HandleTypeDef* handle,
    Stm32Uart* instance
) noexcept;

bsp::Error unregister_uart(
    UART_HandleTypeDef* handle
) noexcept;

void on_tx_complete(
    UART_HandleTypeDef* handle
) noexcept;

void on_rx_complete(
    UART_HandleTypeDef* handle
) noexcept;

void on_error(
    UART_HandleTypeDef* handle
) noexcept;

} // namespace uart_router

} // namespace port