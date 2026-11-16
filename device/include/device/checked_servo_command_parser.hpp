#pragma once

#include <cstddef>
#include <cstdint>
#include "bsp/error.hpp"
#include "device/servo_command_parser.hpp"
#include "module/crc8.hpp"

namespace device {

class CheckedServoCommandParser {
public:
    bsp::Error push(std::uint8_t byte, unsigned& position) noexcept
    {
        if (byte != '\n') {
            if (size_ < sizeof(line_)) {
                line_[size_++] = byte;
            } else {
                overflow_ = true;
            }
            return bsp::Error::Busy;
        }

        const auto result = overflow_
            ? bsp::Error::InvalidArgument : finish(position);
        size_ = 0U;
        overflow_ = false;
        return result;
    }

private:
    static int hex_digit(std::uint8_t byte) noexcept
    {
        if (byte >= '0' && byte <= '9') {
            return byte - '0';
        }
        if (byte >= 'A' && byte <= 'F') {
            return byte - 'A' + 10;
        }
        return -1;
    }

    bsp::Error finish(unsigned& position) const noexcept
    {
        auto length = size_;
        if (length != 0U && line_[length - 1U] == '\r') {
            --length;
        }
        if (length < 5U) {
            return bsp::Error::InvalidArgument;
        }

        const auto separator = length - 3U;
        const auto high = hex_digit(line_[separator + 1U]);
        const auto low = hex_digit(line_[separator + 2U]);
        if (line_[separator] != '*' || high < 0 || low < 0) {
            return bsp::Error::InvalidArgument;
        }

        ServoCommandParser parser;
        unsigned candidate{};
        for (std::size_t i = 0; i < separator; ++i) {
            if (line_[i] == '\r') {
                return bsp::Error::InvalidArgument;
            }
            parser.push(static_cast<char>(line_[i]), candidate);
        }
        if (!parser.push('\n', candidate)) {
            return bsp::Error::InvalidArgument;
        }

        const auto expected = static_cast<std::uint8_t>(high * 16 + low);
        if (module::crc8_smbus(line_, separator) != expected) {
            return bsp::Error::ChecksumMismatch;
        }
        position = candidate;
        return bsp::Error::Ok;
    }

    std::uint8_t line_[8]{}; // S100*25 加可选 CR
    std::size_t size_{};
    bool overflow_{};
};

} // namespace device
