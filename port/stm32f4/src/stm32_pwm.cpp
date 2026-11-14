#include "stm32_pwm.hpp"

namespace port {

bsp::Error Stm32PwmCh1::start() noexcept
{
    if (started_) {
        return bsp::Error::Busy;
    }
    if (timer_.Instance == nullptr) {
        return bsp::Error::NotInitialized;
    }

    const auto status = HAL_TIM_PWM_Start(&timer_, TIM_CHANNEL_1);
    switch (status) {
    case HAL_OK:
        started_ = true;
        return bsp::Error::Ok;
    case HAL_BUSY:
        return bsp::Error::Busy;
    case HAL_TIMEOUT:
        return bsp::Error::Timeout;
    default:
        return bsp::Error::HardwareFailure;
    }
}

bsp::Error Stm32PwmCh1::set_pulse_us(std::uint32_t pulse_us) noexcept
{
    if (!started_) {
        return bsp::Error::NotInitialized;
    }
    if (pulse_us > __HAL_TIM_GET_AUTORELOAD(&timer_)) {
        return bsp::Error::InvalidArgument;
    }

    __HAL_TIM_SET_COMPARE(&timer_, TIM_CHANNEL_1, pulse_us);
    return bsp::Error::Ok;
}

} // namespace port
