#pragma once

#include <cstddef>
#include <cstdint>
#include "bsp/error.hpp"

namespace bsp {

class IUart {
public:
    virtual ~IUart() = default;

    virtual Error transmit(
        const std::uint8_t* data,
        std::size_t size,
        std::uint32_t timeout_ms
    ) noexcept = 0;

    virtual Error receive(
        std::uint8_t* data,
        std::size_t size,
        std::uint32_t timeout_ms
    ) noexcept = 0;
};

} // namespace bsp