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
bsp::Error Stm32Uart::start_transmit(
    const std::uint8_t* data,
    std::size_t size,
    bsp::UartTransferMode mode) noexcept
{
    if (!valid_buffer(data, size))
        return bsp::Error::InvalidArgument;

    if (handle_.gState == HAL_UART_STATE_RESET)
        return bsp::Error::NotInitialized;

    if (tx_state_ == TransferState::Busy){
        return bsp::Error::Busy;
    }

        tx_state_ = TransferState::Busy;

    HAL_StatusTypeDef status =
        mode == bsp::UartTransferMode::Interrupt
        ? HAL_UART_Transmit_IT(&handle_, data, static_cast<std::uint16_t>(size))
        : HAL_UART_Transmit_DMA(&handle_, data, static_cast<std::uint16_t>(size));

    if (status != HAL_OK)
        tx_state_ = TransferState::Idle;

    return map_status(status);
}

bsp::Error Stm32Uart::start_receive(
    std::uint8_t* data,
    std::size_t size,
    bsp::UartTransferMode mode) noexcept
{
    if (!valid_buffer(data, size))
        return bsp::Error::InvalidArgument;

    if (handle_.RxState == HAL_UART_STATE_RESET)
        return bsp::Error::NotInitialized;

    if (rx_state_ == TransferState::Busy){
        return bsp::Error::Busy;
    }
        rx_state_ = TransferState::Busy;
    HAL_StatusTypeDef status =
        mode == bsp::UartTransferMode::Interrupt
        ? HAL_UART_Receive_IT(&handle_, data, static_cast<std::uint16_t>(size))
        : HAL_UART_Receive_DMA(&handle_, data, static_cast<std::uint16_t>(size));

    if (status != HAL_OK){
        rx_state_ = TransferState::Idle;
    }
    return map_status(status);
}

void Stm32Uart::set_callback(
    bsp::UartCallback callback,
    void* context) noexcept
{
    callback_ = callback;
    callback_context_ = context;
}
void Stm32Uart::on_tx_complete() noexcept
{
    if (tx_state_ != TransferState::Busy)
        return;

    tx_state_ = TransferState::Idle;

    if (callback_)
        callback_(callback_context_,
                  bsp::UartEvent::TxComplete,
                  bsp::Error::Ok);
}

void Stm32Uart::on_rx_complete() noexcept
{
    if (rx_state_ != TransferState::Busy)
        return;

    rx_state_ = TransferState::Idle;

    if (callback_)
        callback_(callback_context_,
                  bsp::UartEvent::RxComplete,
                  bsp::Error::Ok);
}

void Stm32Uart::on_error(bsp::Error error) noexcept
{
    // 临时策略：错误发生时结束所有正在进行的传输
    const bool active =
        tx_state_ == TransferState::Busy ||
        rx_state_ == TransferState::Busy;

    tx_state_ = TransferState::Idle;
    rx_state_ = TransferState::Idle;

    if (active && callback_)
        callback_(callback_context_,
                  bsp::UartEvent::Error,
                  error);
}
} // namespace port
