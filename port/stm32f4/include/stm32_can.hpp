#pragma once

#include <atomic>
#include "bsp/ican.hpp"
#include "stm32f4xx_hal.h"

namespace port {

class Stm32Can final : public bsp::ICan, public bsp::ICanIrqRx {
public:
    explicit Stm32Can(CAN_HandleTypeDef& handle) noexcept
        : handle_(handle) {}

    bsp::Error try_send(const bsp::CanFrame& frame) noexcept override;
    bsp::Error receive(bsp::CanFrame& frame,
                       std::uint32_t timeout_ms) noexcept override;

    bsp::Error arm_receive_irq() noexcept override;
    bsp::Error wait_receive_irq(bsp::CanFrame& frame,
                                std::uint32_t timeout_ms) noexcept override;
    void on_rx0_pending(CAN_HandleTypeDef* handle) noexcept;

private:
    enum class RxIrqState : std::uint32_t {
        Idle, Waiting, FramePending, Failed
    };

    std::atomic<RxIrqState> rx_irq_state_{RxIrqState::Idle};
    CAN_HandleTypeDef& handle_;
};

} // namespace port