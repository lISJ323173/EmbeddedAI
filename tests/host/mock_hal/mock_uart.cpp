#include "stm32f4xx_hal.h"

HAL_StatusTypeDef mock_tx_status = HAL_OK;
HAL_StatusTypeDef mock_rx_status = HAL_OK;
int mock_tx_calls = 0;
int mock_rx_calls = 0;

extern "C" HAL_StatusTypeDef HAL_UART_Transmit(
    UART_HandleTypeDef*,
    const std::uint8_t*,
    std::uint16_t,
    std::uint32_t)
{
    ++mock_tx_calls;
    return mock_tx_status;
}

extern "C" HAL_StatusTypeDef HAL_UART_Receive(
    UART_HandleTypeDef*,
    std::uint8_t*,
    std::uint16_t,
    std::uint32_t)
{
    ++mock_rx_calls;
    return mock_rx_status;
}
HAL_StatusTypeDef mock_async_tx_status = HAL_OK;
HAL_StatusTypeDef mock_async_rx_status = HAL_OK;

bool mock_tx_complete_during_start = false;
bool mock_rx_complete_during_start = false;

extern "C" void HAL_UART_RxCpltCallback(
    UART_HandleTypeDef* huart);

extern "C" void HAL_UART_TxCpltCallback(
    UART_HandleTypeDef* huart);

int mock_tx_it_calls = 0;
int mock_tx_dma_calls = 0;
int mock_rx_it_calls = 0;
int mock_rx_dma_calls = 0;



extern "C" HAL_StatusTypeDef HAL_UART_Transmit_DMA(
    UART_HandleTypeDef*, const std::uint8_t*,
    std::uint16_t)
{
    ++mock_tx_dma_calls;
    return mock_async_tx_status;
}

extern "C" HAL_StatusTypeDef HAL_UART_Transmit_IT(
    UART_HandleTypeDef* huart,
    const std::uint8_t*,
    std::uint16_t)
{
    ++mock_tx_it_calls;

    if (mock_tx_complete_during_start) {
        HAL_UART_TxCpltCallback(huart);
    }

    return mock_async_tx_status;
}
extern "C" HAL_StatusTypeDef HAL_UART_Receive_IT(
    UART_HandleTypeDef* huart,
    std::uint8_t*,
    std::uint16_t)
{
    ++mock_rx_it_calls;

    if(mock_rx_complete_during_start){
        HAL_UART_RxCpltCallback(huart);
    }
    return mock_async_rx_status;
}


extern "C" HAL_StatusTypeDef HAL_UART_Receive_DMA(
    UART_HandleTypeDef*, std::uint8_t*,
    std::uint16_t)
{
    ++mock_rx_dma_calls;
    return mock_async_rx_status;
}