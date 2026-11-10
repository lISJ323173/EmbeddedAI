#pragma once

#include <cstdint>

namespace bsp {

enum class Error : std::uint8_t {
    Ok = 0,
    NotInitialized,
    InvalidArgument,
    Busy,
    Timeout,
    ChecksumMismatch,
    HardwareFailure,
    Unsupported
};

constexpr bool is_ok(Error error) noexcept {
    return error == Error::Ok;
}

} // namespace bsp
