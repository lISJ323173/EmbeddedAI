#pragma once

#include <cstdint>
#include "bsp/error.hpp"

namespace bsp {

class IPwm {
public:
    virtual ~IPwm() = default;

    virtual Error start() noexcept = 0;
    virtual Error set_pulse_us(std::uint32_t pulse_us) noexcept = 0;
};

} // namespace bsp
