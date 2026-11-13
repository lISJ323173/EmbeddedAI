#pragma once

#include <cstdint>

enum HAL_StatusTypeDef {
    HAL_OK = 0,
    HAL_ERROR,
    HAL_BUSY,
    HAL_TIMEOUT
};

enum HAL_UART_StateTypeDef {
    HAL_UART_STATE_RESET = 0,
    HAL_UART_STATE_READY = 0x20
};

struct UART_HandleTypeDef {
    HAL_UART_StateTypeDef gState = HAL_UART_STATE_RESET;
    HAL_UART_StateTypeDef RxState = HAL_UART_STATE_RESET;
};

extern "C" {

HAL_StatusTypeDef HAL_UART_Transmit(
    UART_HandleTypeDef* huart,
    const std::uint8_t* data,
    std::uint16_t size,
    std::uint32_t timeout);

HAL_StatusTypeDef HAL_UART_Receive(
    UART_HandleTypeDef* huart,
    std::uint8_t* data,
    std::uint16_t size,
    std::uint32_t timeout);

    HAL_StatusTypeDef HAL_UART_Transmit_IT(
    UART_HandleTypeDef*, const std::uint8_t*, std::uint16_t);

HAL_StatusTypeDef HAL_UART_Receive_IT(
    UART_HandleTypeDef*, std::uint8_t*, std::uint16_t);

HAL_StatusTypeDef HAL_UART_Transmit_DMA(
    UART_HandleTypeDef*, const std::uint8_t*, std::uint16_t);

HAL_StatusTypeDef HAL_UART_Receive_DMA(
    UART_HandleTypeDef*, std::uint8_t*, std::uint16_t);

HAL_StatusTypeDef HAL_UART_Abort(
    UART_HandleTypeDef* huart);
}