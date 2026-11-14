#pragma once

#include <cstddef>
#include <cstdint>

namespace module {

constexpr std::uint8_t crc8_smbus(const std::uint8_t* data,
                                  std::size_t size) noexcept
{
    std::uint8_t crc = 0U;

    for (std::size_t i = 0; i < size; ++i) {
        crc ^= data[i];
        for (unsigned bit = 0; bit < 8U; ++bit) {
            const bool high_bit = (crc & 0x80U) != 0U;
            crc = static_cast<std::uint8_t>(crc << 1U);
            if (high_bit) {
                crc ^= 0x07U;
            }
        }
    }
    return crc;
}

} // namespace module
