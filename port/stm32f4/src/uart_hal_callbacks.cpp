#include "uart_callback_router.hpp"

extern "C" {

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart)
{
    port::uart_router::on_tx_complete(huart);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
    port::uart_router::on_rx_complete(huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart)
{
    port::uart_router::on_error(huart);
}

}
