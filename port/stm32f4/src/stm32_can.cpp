#include "stm32_can.hpp"

namespace port {

bsp::Error Stm32Can::try_send(const bsp::CanFrame& frame) noexcept
{
    if (frame.id > 0x7FFU || frame.dlc > 8U) {
        return bsp::Error::InvalidArgument;
    }
    if (HAL_CAN_GetState(&handle_) != HAL_CAN_STATE_LISTENING) {
        return bsp::Error::NotInitialized;
    }
    if (HAL_CAN_GetTxMailboxesFreeLevel(&handle_) == 0U) {
        return bsp::Error::Busy;
    }

    CAN_TxHeaderTypeDef header{};
    header.StdId = frame.id;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = frame.dlc;

    auto tx_frame = frame;
    std::uint32_t mailbox{};
    const auto status = HAL_CAN_AddTxMessage(
        &handle_, &header, tx_frame.data, &mailbox);
    return status == HAL_OK ? bsp::Error::Ok : bsp::Error::HardwareFailure;
}

bsp::Error Stm32Can::receive(bsp::CanFrame& frame,
                            std::uint32_t timeout_ms) noexcept
{
    if (rx_irq_state_.load() != RxIrqState::Idle) {
        return bsp::Error::Busy;
    }
    if (HAL_CAN_GetState(&handle_) != HAL_CAN_STATE_LISTENING) {
        return bsp::Error::NotInitialized;
    }

    const auto start = HAL_GetTick();
    while (HAL_CAN_GetRxFifoFillLevel(&handle_, CAN_RX_FIFO0) == 0U) {
        if (HAL_GetTick() - start >= timeout_ms) {
            return bsp::Error::Timeout;
        }
    }

    CAN_RxHeaderTypeDef header{};
    bsp::CanFrame received{};
    if (HAL_CAN_GetRxMessage(&handle_, CAN_RX_FIFO0, &header,
                            received.data) != HAL_OK) {
        return bsp::Error::HardwareFailure;
    }
    if (header.IDE != CAN_ID_STD || header.RTR != CAN_RTR_DATA) {
        return bsp::Error::Unsupported;
    }
    if (header.DLC > 8U) {
        return bsp::Error::HardwareFailure;
    }

    received.id = static_cast<std::uint16_t>(header.StdId);
    received.dlc = static_cast<std::uint8_t>(header.DLC);
    frame = received;
    return bsp::Error::Ok;
}

bsp::Error Stm32Can::arm_receive_irq() noexcept
{
    if (HAL_CAN_GetState(&handle_) != HAL_CAN_STATE_LISTENING) {
        return bsp::Error::NotInitialized;
    }
    if (rx_irq_state_.load() != RxIrqState::Idle) {
        return bsp::Error::Busy;
    }

    rx_irq_state_.store(RxIrqState::Waiting);
    if (HAL_CAN_ActivateNotification(&handle_, CAN_IT_RX_FIFO0_MSG_PENDING)
        != HAL_OK) {
        rx_irq_state_.store(RxIrqState::Idle);
        return bsp::Error::HardwareFailure;
    }
    return bsp::Error::Ok;
}

void Stm32Can::on_rx0_pending(CAN_HandleTypeDef* handle) noexcept
{
    if (handle != &handle_ || rx_irq_state_.load() != RxIrqState::Waiting) {
        return;
    }

    const auto status = HAL_CAN_DeactivateNotification(
        &handle_, CAN_IT_RX_FIFO0_MSG_PENDING);
    rx_irq_state_.store(status == HAL_OK ? RxIrqState::FramePending
                                         : RxIrqState::Failed);
}

bsp::Error Stm32Can::wait_receive_irq(bsp::CanFrame& frame,
                                     std::uint32_t timeout_ms) noexcept
{
    if (rx_irq_state_.load() == RxIrqState::Idle) {
        return bsp::Error::InvalidArgument;
    }

    const auto start = HAL_GetTick();
    while (rx_irq_state_.load() == RxIrqState::Waiting) {
        if (HAL_GetTick() - start >= timeout_ms) {
            break;
        }
    }

    // 超时路径也关闭通知，再读取最终状态。
    if (HAL_CAN_DeactivateNotification(&handle_, CAN_IT_RX_FIFO0_MSG_PENDING)
        != HAL_OK) {
        return bsp::Error::HardwareFailure;
    }

    const auto state = rx_irq_state_.exchange(RxIrqState::Idle);
    if (state == RxIrqState::Failed) {
        return bsp::Error::HardwareFailure;
    }
    if (state != RxIrqState::FramePending) {
        return bsp::Error::Timeout;
    }

    return receive(frame, 0U);
}

} // namespace port