#pragma once

#include <cstdint>
#include "bsp/error.hpp"

namespace bsp {

struct CanFrame {
    std::uint16_t id{};
    std::uint8_t dlc{};
    std::uint8_t data[8]{};
};

class ICan {
public:
    virtual ~ICan() = default;

    virtual Error try_send(const CanFrame& frame) noexcept = 0;
    virtual Error receive(CanFrame& frame,
                          std::uint32_t timeout_ms) noexcept = 0;
};

class ICanIrqRx {
public:
    virtual ~ICanIrqRx() = default;

    // 启用一次 FIFO0 接收通知。
    virtual Error arm_receive_irq() noexcept = 0;

    // 在前台等待通知，成功后取出一帧。
    virtual Error wait_receive_irq(CanFrame& frame,
                                   std::uint32_t timeout_ms) noexcept = 0;
};

} // namespace bsp