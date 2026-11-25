#include "main.h"
#include "stm32_pwm.hpp"
#include "gimbal_control.h"
#include "gimbal_observe.h"
#include "gimbal_transport.hpp"
#include <algorithm>
#include <cmath>

using gimbal::detail::pitch_requests;
using gimbal::detail::yaw_events;
using gimbal::detail::pitch_stop_flag;

namespace {

constexpr uint32_t pitch_pulses[] = {
    1500U, 1650U, 1800U, 1950U, 2100U,
    1950U, 1800U, 1650U, 1500U
};

bool pitch_active = false;
unsigned pitch_index = 0U;
uint32_t pitch_hold_ticks = 0U;
float pitch_zero_deg = 0.0F;
constexpr float pitch_target_deg = 20.0F;
constexpr float pitch_kp = 2.0F;  // us/deg
constexpr float pitch_ki = 8.0F;  // us/(deg*s)
constexpr float pitch_deadband_deg = 0.8F;

float pitch_integral_us = 0.0F;

void finish_pitch(
    port::Stm32PwmCh1& pwm,
    uint32_t state,
    uint32_t reason = GIMBAL_FAULT_UNKNOWN)
{
    if (state == GIMBAL_AXIS_FAULT) {
        pitch_last_fault = reason;
    }

    if (pwm.stop() != bsp::Error::Ok) {
        ++pitch_pwm_errors;
        pitch_last_fault = GIMBAL_FAULT_PWM_STOP;
        state = GIMBAL_AXIS_FAULT;
    }

    pitch_active = false;
    pitch_closed_loop_enabled = 0U;
    pitch_test_state = state;
}

bool write_pitch(port::Stm32PwmCh1& pwm, uint32_t pulse)
{
    if (pulse < 1500U || pulse > 2500U) {
        finish_pitch(pwm, GIMBAL_AXIS_FAULT, GIMBAL_FAULT_PULSE_RANGE);
        return false;
    }
    if (pwm.set_pulse_us(pulse) != bsp::Error::Ok) {
        ++pitch_pwm_errors;
        finish_pitch(pwm, GIMBAL_AXIS_FAULT, GIMBAL_FAULT_PWM_WRITE);
        return false;
    }
    pitch_test_pulse = pulse;
    ++pitch_pwm_updates;
    return true;
}

void update_pitch_pi(port::Stm32PwmCh1& pwm,
                     float feedback_deg, float dt_seconds)
{
    const float error = pitch_target_deg - feedback_deg;
    pitch_error_mdeg = static_cast<int32_t>(
        std::lround(error * 1000.0F));

    const float control_error =
        std::fabs(error) <= pitch_deadband_deg ? 0.0F : error;

    const float proportional = pitch_kp * control_error;
    const float candidate_integral = std::clamp(
        pitch_integral_us +
            pitch_ki * control_error * dt_seconds,
        0.0F, 600.0F);

    const float candidate_output =
        1500.0F + proportional + candidate_integral;

    const bool pushing_upper =
        candidate_output > 2100.0F && control_error > 0.0F;
    const bool pushing_lower =
        candidate_output < 1500.0F && control_error < 0.0F;

    if (!pushing_upper && !pushing_lower) {
        pitch_integral_us = candidate_integral;
    }

    const float output = std::clamp(
        1500.0F + proportional + pitch_integral_us,
        1500.0F, 2100.0F);

    const uint32_t requested =
        static_cast<uint32_t>(std::lround(output));
    const uint32_t current = pitch_test_pulse;
    const uint32_t next = requested > current
        ? std::min<uint32_t>(current + 2U, requested)
        : std::max<uint32_t>(current - 2U, requested);

    if (next != current) {
        (void)write_pitch(pwm, next);
    }
}

} // namespace

extern "C" void gimbal_pitch_tick(void *argument,
                                  float roll_deg,
                                  float dt_seconds)
{
    auto& timer = *static_cast<TIM_HandleTypeDef*>(argument);
    static port::Stm32PwmCh1 pitch(timer, TIM_CHANNEL_1);

    uint8_t request = 0U;
    const auto received = osMessageQueueGet(
        pitch_requests, &request, nullptr, 0U);

    if (received != osOK && received != osErrorResource) {
        finish_pitch(
            pitch, GIMBAL_AXIS_FAULT, GIMBAL_FAULT_TRANSPORT);
        return;
    }

    const uint32_t stop_event = osEventFlagsWait(
        yaw_events, pitch_stop_flag, osFlagsWaitAny, 0U);

    if ((stop_event & osFlagsError) != 0U) {
        if (stop_event != osFlagsErrorResource &&
            stop_event != osFlagsErrorTimeout) {
            finish_pitch(
                pitch, GIMBAL_AXIS_FAULT, GIMBAL_FAULT_TRANSPORT);
            return;
        }
    } else if ((stop_event & pitch_stop_flag) != 0U) {
        finish_pitch(pitch, GIMBAL_AXIS_IDLE);
        return;
    }

    if (received == osOK && request == 'X') {
        finish_pitch(pitch, GIMBAL_AXIS_IDLE);
        return;
    }

    if (!std::isfinite(dt_seconds) ||
        dt_seconds < 0.005F || dt_seconds > 0.020F) {
        if (pitch_active) {
            finish_pitch(
                pitch, GIMBAL_AXIS_FAULT,
                GIMBAL_FAULT_INVALID_PERIOD);
        }
        return;
    }

    if (!std::isfinite(roll_deg)) {
        pitch_zero_ready = 0U;
        pitch_zero_status = 3U;
        if (pitch_active) {
            finish_pitch(
                pitch, GIMBAL_AXIS_FAULT,
                GIMBAL_FAULT_INVALID_FEEDBACK);
        }
        return;
    }

    pitch_roll_mdeg = static_cast<int32_t>(
        std::lround(roll_deg * 1000.0F));

    if (received == osOK && request == 'Z') {
        if (!servo_test_allowed || pitch_active ||
            pitch_test_pulse != 1500U) {
            pitch_zero_status = 2U;
        } else {
            pitch_zero_deg = roll_deg;
            pitch_zero_mdeg = pitch_roll_mdeg;
            pitch_zero_ready = 1U;
            pitch_zero_status = 1U;
        }
    }

    if (pitch_zero_ready) {
        pitch_feedback_mdeg = static_cast<int32_t>(
            std::lround((roll_deg - pitch_zero_deg) * 1000.0F));
    }

    if (received == osOK && request == 'Z') return;

    if (!servo_test_allowed) {
        if (pitch_active) {
            finish_pitch(
                pitch, GIMBAL_AXIS_FAULT,
                GIMBAL_FAULT_NOT_PERMITTED);
        } else if (received == osOK) {
            pitch_test_state = GIMBAL_AXIS_REJECTED;
        }
        return;
    }

    if (!pitch_active && received == osOK) {
        if (request != 'P' && request != 'C') return;

        if (request == 'C' && !pitch_zero_ready) {
            pitch_test_state = GIMBAL_AXIS_REJECTED;
            return;
        }

        if (servo_test_state == GIMBAL_AXIS_RUNNING &&
            (request != 'C' ||
             !yaw_closed_loop_enabled ||
             !yaw_reference_ready)) {
            pitch_test_state = GIMBAL_AXIS_REJECTED;
            return;
        }

        pitch_index = 0U;
        pitch_hold_ticks = 0U;
        pitch_closed_loop_enabled = 0U;

        if (pitch.start() != bsp::Error::Ok) {
            ++pitch_pwm_errors;
            finish_pitch(
                pitch, GIMBAL_AXIS_FAULT, GIMBAL_FAULT_PWM_START);
            return;
        }

        if (!write_pitch(pitch, pitch_test_pulse)) return;

        pitch_last_fault = GIMBAL_FAULT_NONE;

        if (request == 'C') {
            const float feedback = roll_deg - pitch_zero_deg;
            const float error = pitch_target_deg - feedback;

            pitch_integral_us = std::clamp(
                static_cast<float>(pitch_test_pulse) - 1500.0F
                    - pitch_kp * error,
                0.0F, 600.0F);

            pitch_target_mdeg = 20000;
            pitch_closed_loop_enabled = 1U;
        }

        pitch_active = true;
        pitch_test_state = GIMBAL_AXIS_RUNNING;
        return;
    }

    if (!pitch_active) return;

    if (pitch_closed_loop_enabled) {
        if (!pitch_zero_ready) {
            finish_pitch(
                pitch, GIMBAL_AXIS_FAULT,
                GIMBAL_FAULT_REFERENCE_MISSING);
            return;
        }

        update_pitch_pi(
            pitch, roll_deg - pitch_zero_deg, dt_seconds);
        return;
    }

    const uint32_t target = pitch_pulses[pitch_index];
    const uint32_t current = pitch_test_pulse;

    if (current != target) {
        const uint32_t next = current < target
            ? std::min<uint32_t>(current + 2U, target)
            : std::max<uint32_t>(current - 2U, target);

        if (write_pitch(pitch, next)) {
            pitch_hold_ticks = 0U;
        }
        return;
    }

    if (++pitch_hold_ticks >= 500U) {
        pitch_hold_ticks = 0U;
        ++pitch_index;

        if (pitch_index >=
            sizeof(pitch_pulses) / sizeof(pitch_pulses[0])) {
            finish_pitch(pitch, GIMBAL_AXIS_COMPLETED);
        }
    }
}
