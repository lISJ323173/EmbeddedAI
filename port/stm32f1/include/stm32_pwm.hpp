#pragma once

#include "bsp/ipwm.hpp"
#include "stm32f1xx_hal.h"

namespace port {

class Stm32PwmCh1 final : public bsp::IPwm {
public:
    explicit Stm32PwmCh1(
        TIM_HandleTypeDef& timer,
        std::uint32_t channel = TIM_CHANNEL_1) noexcept
        : timer_(timer), channel_(channel) {}

    bsp::Error start() noexcept override;
    bsp::Error set_pulse_us(std::uint32_t pulse_us) noexcept override;
    bsp::Error stop() noexcept;

private:
    TIM_HandleTypeDef& timer_;
    std::uint32_t channel_;
    bool started_{false};
};

} // namespace port
