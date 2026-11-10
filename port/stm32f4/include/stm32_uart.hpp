#pragma once

#include "bsp/iuart.hpp"
#include "bsp/iuart_async.hpp"
#include "stm32f4xx_hal.h"

namespace port {

class Stm32Uart final
    : public bsp::IUart,
      public bsp::IUartAsync {
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

        bsp::Error start_transmit(
        const std::uint8_t* data,
        std::size_t size,
        bsp::UartTransferMode mode
    ) noexcept override;

    bsp::Error start_receive(
        std::uint8_t* data,
        std::size_t size,
        bsp::UartTransferMode mode
    ) noexcept override;

    void set_callback(
        bsp::UartCallback callback,
        void* context
    ) noexcept override;
    void on_tx_complete() noexcept;
    void on_rx_complete() noexcept;
    void on_error(bsp::Error error) noexcept;
private:
    UART_HandleTypeDef& handle_;
        enum class TransferState : std::uint8_t {
        Idle,
        Busy
    };

    TransferState tx_state_{TransferState::Idle};
    TransferState rx_state_{TransferState::Idle};

    bsp::UartCallback callback_{nullptr};
    void* callback_context_{nullptr};
};

} // namespace port
