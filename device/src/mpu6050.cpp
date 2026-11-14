#include "device/mpu6050.hpp"

namespace device {

bsp::Error Mpu6050::probe(std::uint8_t& who_am_i,
                          std::uint32_t timeout_ms) noexcept
{
    constexpr std::uint8_t who_am_i_register = 0x75;
    constexpr std::uint8_t expected_id = 0x68;
    std::uint8_t value{};

    const auto result = bus_.read_register(
        address_7bit_, who_am_i_register, &value, 1U, timeout_ms);
    if (result != bsp::Error::Ok) {
        return result;
    }

    who_am_i = value;
    return value == expected_id ? bsp::Error::Ok : bsp::Error::Unsupported;
}

bsp::Error Mpu6050::wake_up(std::uint32_t timeout_ms) noexcept
{
    std::uint8_t id{};
    const auto result = probe(id, timeout_ms);
    if (result != bsp::Error::Ok) {
        return result;
    }

    constexpr std::uint8_t power_mgmt_1 = 0x6B;
    constexpr std::uint8_t awake_internal_clock = 0x00;
    return bus_.write_register(address_7bit_, power_mgmt_1,
                               awake_internal_clock, timeout_ms);
}

} // namespace device
