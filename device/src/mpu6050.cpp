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
    wake_up_succeeded_ = false;
    std::uint8_t id{};
    const auto result = probe(id, timeout_ms);
    if (result != bsp::Error::Ok) {
        return result;
    }

    constexpr std::uint8_t power_mgmt_1 = 0x6B;
    constexpr std::uint8_t awake_internal_clock = 0x00;
    const auto write_result = bus_.write_register(
        address_7bit_, power_mgmt_1, awake_internal_clock, timeout_ms);
    wake_up_succeeded_ = (write_result == bsp::Error::Ok);
    return write_result;
}

namespace {

std::int16_t decode_signed_be(const std::uint8_t* bytes) noexcept
{
    const std::int32_t value =
        (static_cast<std::int32_t>(bytes[0]) << 8) | bytes[1];
    return static_cast<std::int16_t>(
        value < 0x8000 ? value : value - 0x10000);
}

} // namespace

bsp::Error Mpu6050::read_raw(Mpu6050RawSample& sample,
                             std::uint32_t timeout_ms) noexcept
{
    if (!wake_up_succeeded_) {
        return bsp::Error::NotInitialized;
    }

    constexpr std::uint8_t first_sample_register = 0x3B;
    std::uint8_t bytes[14]{};
    const auto result = bus_.read_register(
        address_7bit_, first_sample_register, bytes, sizeof(bytes), timeout_ms);
    if (result != bsp::Error::Ok) {
        return result;
    }

    sample = {
        decode_signed_be(bytes),
        decode_signed_be(bytes + 2),
        decode_signed_be(bytes + 4),
        decode_signed_be(bytes + 6),
        decode_signed_be(bytes + 8),
        decode_signed_be(bytes + 10),
        decode_signed_be(bytes + 12)
    };
    return bsp::Error::Ok;
}

} // namespace device
