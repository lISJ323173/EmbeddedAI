#include "main.h"
#include "stm32_pwm.hpp"
#include "gimbal_control.h"
#include "gimbal_observe.h"
#include "gimbal_transport.hpp"
#include <algorithm>
#include <cmath>

using gimbal::detail::yaw_events;
using gimbal::detail::yaw_start_flag;
using gimbal::detail::yaw_stop_flag;
using gimbal::detail::yaw_hold_flag;

namespace {

constexpr float yaw_kp = 4.0F; // us/deg
constexpr float yaw_ki = 8.0F; // us/(deg*s)
constexpr float yaw_deadband_deg = 1.5F;

float yaw_reference_deg = 0.0F;
float yaw_integral_us = 0.0F;

float wrap_yaw(float angle_deg)
{
    return std::remainder(angle_deg, 360.0F);
}

constexpr uint32_t yaw_pulses[] = {
    1500U, 1470U, 1440U, 1500U,
    1530U, 1560U, 1500U
};

bool yaw_active = false;
unsigned yaw_index = 0U;
uint32_t yaw_hold_ticks = 0U;
bool yaw_reference_pending = false;
uint32_t yaw_reference_wait_ticks = 0U;

void finish_yaw(port::Stm32PwmCh1& pwm, uint32_t state)
{
    if (pwm.stop() != bsp::Error::Ok) {
        ++yaw_pwm_errors;
        state = 4U;
    }
    yaw_active = false;
    yaw_closed_loop_enabled = 0U;
    yaw_reference_pending = false;
    yaw_reference_ready = 0U;
    servo_test_state = state;
}

bool write_yaw(port::Stm32PwmCh1& pwm, uint32_t pulse)
{
    if (pulse < 1440U || pulse > 1560U) {
        finish_yaw(pwm, 4U);
        return false;
    }
    if (pwm.set_pulse_us(pulse) != bsp::Error::Ok) {
        ++yaw_pwm_errors;
        finish_yaw(pwm, 4U);
        return false;
    }
    servo_test_pulse = pulse;
    ++yaw_pwm_updates;
    return true;
}

void update_yaw_pi(port::Stm32PwmCh1& pwm,
                   float feedback_deg, float dt_seconds)
{
    const float error = -feedback_deg;
    yaw_error_mdeg = static_cast<int32_t>(
        std::lround(error * 1000.0F));

    const float control_error =
        std::fabs(error) <= yaw_deadband_deg ? 0.0F : error;

    if (control_error != 0.0F) {
        ++yaw_outside_deadband_samples;
    }

    const float proportional = yaw_kp * control_error;

    const float candidate_integral = std::clamp(
        yaw_integral_us + yaw_ki * control_error * dt_seconds,
        -60.0F, 60.0F);
    const float candidate_output =
        1500.0F + proportional + candidate_integral;

    const bool pushing_upper =
        candidate_output > 1560.0F && control_error > 0.0F;
    const bool pushing_lower =
        candidate_output < 1440.0F && control_error < 0.0F;

    if (!pushing_upper && !pushing_lower) {
        yaw_integral_us = candidate_integral;
    }

    yaw_integral_millius = static_cast<int32_t>(
        std::lround(yaw_integral_us * 1000.0F));

    const float output = std::clamp(
        1500.0F + proportional + yaw_integral_us,
        1440.0F, 1560.0F);
    const uint32_t requested =
        static_cast<uint32_t>(std::lround(output));
    const uint32_t current = servo_test_pulse;
    const uint32_t next = requested > current
        ? std::min<uint32_t>(current + 2U, requested)
        : std::max<uint32_t>(current - 2U, requested);

    if (next != current) {
        (void)write_yaw(pwm, next);
    }
}

} // namespace

extern "C" void gimbal_yaw_tick(void *argument,
                                float yaw_deg,
                                float dt_seconds)
{
    auto& timer = *static_cast<TIM_HandleTypeDef*>(argument);
    static port::Stm32PwmCh1 yaw(timer, TIM_CHANNEL_2);

    uint32_t events = osEventFlagsWait(
        yaw_events,
        yaw_start_flag | yaw_stop_flag | yaw_hold_flag,
        osFlagsWaitAny, 0U);

    if ((events & osFlagsError) != 0U) {
        if (events != osFlagsErrorResource &&
            events != osFlagsErrorTimeout) {
            finish_yaw(yaw, 4U);
            return;
        }
        events = 0U;
    }

    if ((events & yaw_stop_flag) != 0U) {
        finish_yaw(yaw, 0U);
        return;
    }

    const bool start_requested =
        (events & (yaw_start_flag | yaw_hold_flag)) != 0U;
    const bool hold_requested =
        (events & yaw_hold_flag) != 0U;

    const bool permitted = servo_test_allowed &&
        std::isfinite(yaw_deg) &&
        std::isfinite(dt_seconds) &&
        dt_seconds >= 0.005F && dt_seconds <= 0.020F;

    if (!permitted) {
        if (yaw_active) finish_yaw(yaw, 4U);
        else if (start_requested) servo_test_state = 3U;
        return;
    }

    if (!yaw_active && start_requested) {
        const bool pitch_blocks_start =
            pitch_test_state == 1U &&
            (!hold_requested || !pitch_closed_loop_enabled);

        if (pitch_blocks_start ||
            (hold_requested && servo_test_pulse != 1500U)) {
            servo_test_state = 3U;
            return;
        }

        yaw_index = 0U;
        yaw_hold_ticks = 0U;
        yaw_closed_loop_enabled = 0U;

        if (yaw.start() != bsp::Error::Ok) {
            ++yaw_pwm_errors;
            finish_yaw(yaw, 4U);
            return;
        }
        if (!write_yaw(yaw, servo_test_pulse)) return;

        yaw_reference_pending = hold_requested;
        yaw_reference_wait_ticks = 0U;
        yaw_reference_ready = 0U;

        yaw_active = true;
        servo_test_state = 1U;
        return;
    }

    if (!yaw_active) return;

    if (yaw_reference_pending) {
        if (++yaw_reference_wait_ticks >= 100U) {
            yaw_reference_deg = yaw_deg;
            yaw_reference_mdeg = static_cast<int32_t>(
                std::lround(yaw_deg * 1000.0F));

            yaw_integral_us = 0.0F;
            yaw_integral_millius = 0;
            yaw_feedback_mdeg = 0;
            yaw_error_mdeg = 0;
            yaw_feedback_min_mdeg = 0;
            yaw_feedback_max_mdeg = 0;
            yaw_outside_deadband_samples = 0U;

            yaw_reference_pending = false;
            yaw_reference_ready = 1U;
            yaw_closed_loop_enabled = 1U;
        }
        return;
    }

    if (yaw_closed_loop_enabled) {
        const float feedback =
            wrap_yaw(yaw_deg - yaw_reference_deg);

        yaw_feedback_mdeg = static_cast<int32_t>(
            std::lround(feedback * 1000.0F));

        const int32_t feedback_mdeg = yaw_feedback_mdeg;

        if (feedback_mdeg < yaw_feedback_min_mdeg) {
            yaw_feedback_min_mdeg = feedback_mdeg;
        }
        if (feedback_mdeg > yaw_feedback_max_mdeg) {
            yaw_feedback_max_mdeg = feedback_mdeg;
        }

        update_yaw_pi(yaw, feedback, dt_seconds);
        return;
    }

    const uint32_t target = yaw_pulses[yaw_index];
    const uint32_t current = servo_test_pulse;

    if (current != target) {
        const uint32_t next = current < target
            ? std::min<uint32_t>(current + 2U, target)
            : std::max<uint32_t>(current - 2U, target);

        if (write_yaw(yaw, next)) {
            yaw_hold_ticks = 0U;
        }
        return;
    }

    if (++yaw_hold_ticks >= 500U) {
        yaw_hold_ticks = 0U;
        ++yaw_index;

        if (yaw_index >=
            sizeof(yaw_pulses) / sizeof(yaw_pulses[0])) {
            finish_yaw(yaw, 2U);
        }
    }
}
