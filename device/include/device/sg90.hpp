#pragma once

#include "bsp/ipwm.hpp"

namespace device {

class Sg90 final {
public:
    explicit Sg90(bsp::IPwm& pwm) noexcept : pwm_(pwm) {}

    bsp::Error set_position_percent(std::int32_t percent) noexcept
    {
        if (percent < 0 || percent > 100) {
            return bsp::Error::InvalidArgument;
        }

        constexpr std::uint32_t min_pulse_us = 1300U;
        constexpr std::uint32_t max_pulse_us = 1700U;
        const auto pulse_us = min_pulse_us +
            (max_pulse_us - min_pulse_us) *
                static_cast<std::uint32_t>(percent) / 100U;
        return pwm_.set_pulse_us(pulse_us);
    }

private:
    bsp::IPwm& pwm_;
};

} // namespace device
