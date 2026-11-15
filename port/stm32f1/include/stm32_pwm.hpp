#pragma once

#include "bsp/ipwm.hpp"
#include "stm32f1xx_hal.h"

namespace port {

class Stm32PwmCh1 final : public bsp::IPwm {
public:
    explicit Stm32PwmCh1(TIM_HandleTypeDef& timer) noexcept
        : timer_(timer) {}

    bsp::Error start() noexcept override;
    bsp::Error set_pulse_us(std::uint32_t pulse_us) noexcept override;

private:
    TIM_HandleTypeDef& timer_;
    bool started_{false};
};

} // namespace port
