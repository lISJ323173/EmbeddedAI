#pragma once

#include "bsp/ii2c.hpp"

namespace device {

class Mpu6050 final {
public:
    Mpu6050(bsp::II2c& bus, std::uint8_t address_7bit) noexcept
        : bus_(bus), address_7bit_(address_7bit) {}

    // 读成功时输出身份字节。身份不匹配时仍保留该字节。
    bsp::Error probe(std::uint8_t& who_am_i,
                     std::uint32_t timeout_ms) noexcept;

    bsp::Error wake_up(std::uint32_t timeout_ms) noexcept;

private:
    bsp::II2c& bus_;
    std::uint8_t address_7bit_;
};

} // namespace device
