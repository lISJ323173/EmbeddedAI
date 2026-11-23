#include "main.h"
#include "cmsis_os2.h"
#include "stm32_pwm.hpp"
#include "module/pitch_map.h"
#include <algorithm>
#include <cmath>

extern "C" {
extern UART_HandleTypeDef huart2;
extern volatile uint32_t servo_test_allowed;
extern volatile uint32_t servo_test_state;
extern volatile uint32_t servo_test_pulse;
extern volatile uint32_t gimbal_test_fault;
volatile int32_t pitch_target_mdeg = 20000;
volatile int32_t pitch_error_mdeg = 0;
volatile uint32_t pitch_closed_loop_enabled = 0U;

volatile uint32_t pitch_test_state = 0U;
volatile uint32_t pitch_test_pulse = 1500U;
volatile uint32_t pitch_pwm_updates = 0U;
volatile uint32_t pitch_pwm_errors = 0U;
volatile uint32_t pitch_request_status = 0U;

volatile int32_t pitch_roll_mdeg = 0;
volatile int32_t pitch_zero_mdeg = 0;
volatile int32_t pitch_feedback_mdeg = 0;
volatile uint32_t pitch_zero_ready = 0U;
volatile uint32_t pitch_zero_status = 0U;
}

namespace {

constexpr uint32_t pitch_pulses[] = {
    1500U, 1650U, 1800U, 1950U, 2100U,
    1950U, 1800U, 1650U, 1500U
};

osMessageQueueId_t pitch_requests = nullptr;
bool pitch_active = false;
unsigned pitch_index = 0U;
uint32_t pitch_hold_ticks = 0U;
float pitch_zero_deg = 0.0F;
constexpr float pitch_target_deg = 20.0F;
constexpr float pitch_kp = 2.0F;  // us/deg
constexpr float pitch_ki = 8.0F;  // us/(deg*s)
constexpr float pitch_deadband_deg = 1.5F;

float pitch_integral_us = 0.0F;

void finish_pitch(port::Stm32PwmCh1& pwm, uint32_t state)
{
    if (pwm.stop() != bsp::Error::Ok) {
        ++pitch_pwm_errors;
        state = 4U;
    }
    pitch_active = false;
        pitch_closed_loop_enabled = 0U;
    pitch_test_state = state;
}

bool write_pitch(port::Stm32PwmCh1& pwm, uint32_t pulse)
{
    if (pulse < 1500U || pulse > 2500U) {
        finish_pitch(pwm, 4U);
        return false;
    }
    if (pwm.set_pulse_us(pulse) != bsp::Error::Ok) {
        ++pitch_pwm_errors;
        finish_pitch(pwm, 4U);
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


bool hold_pulse(port::Stm32PwmCh1& pwm, uint32_t pulse)
{
    while (servo_test_pulse != pulse) {
        if (!servo_test_allowed) {
            return false;
        }

        const uint32_t current = servo_test_pulse;
        const uint32_t next = current < pulse
            ? std::min<uint32_t>(current + 2U, pulse)
            : std::max<uint32_t>(current - 2U, pulse);

        if (pwm.set_pulse_us(next) != bsp::Error::Ok) {
            return false;
        }
        servo_test_pulse = next;

        if (osDelay(10U) != osOK) {
            return false;
        }
    }

    for (unsigned i = 0; i < 500U; ++i) {
        if (!servo_test_allowed) {
            return false;
        }
        if (osDelay(10U) != osOK) {
            return false;
        }
    }
    return true;
}

} // namespace

extern "C" int gimbal_pitch_init(void)
{
    pitch_requests = osMessageQueueNew(
        1U, sizeof(uint8_t), nullptr);
    return pitch_requests != nullptr;
}

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
        finish_pitch(pitch, 4U);
        return;
    }

    if (received == osOK && request == 'X') {
        finish_pitch(pitch, 0U);
        return;
    }

    if (!std::isfinite(dt_seconds) ||
        dt_seconds < 0.005F || dt_seconds > 0.020F) {
        if (pitch_active) finish_pitch(pitch, 4U);
        return;
    }

    if (!std::isfinite(roll_deg)) {
        pitch_zero_ready = 0U;
        pitch_zero_status = 3U;
        if (pitch_active) finish_pitch(pitch, 4U);
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
            finish_pitch(pitch, 4U);
        } else if (received == osOK) {
            pitch_test_state = 3U;
        }
        return;
    }

    if (!pitch_active && received == osOK) {
        if (request != 'P' && request != 'C') return;

        if (request == 'C' && !pitch_zero_ready) {
            pitch_test_state = 3U;
            return;
        }

        pitch_index = 0U;
        pitch_hold_ticks = 0U;
        pitch_closed_loop_enabled = 0U;

        if (pitch.start() != bsp::Error::Ok) {
            ++pitch_pwm_errors;
            finish_pitch(pitch, 4U);
            return;
        }

        if (!write_pitch(pitch, pitch_test_pulse)) return;

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
        pitch_test_state = 1U;
        return;
    }

    if (!pitch_active) return;

    if (pitch_closed_loop_enabled) {
        if (!pitch_zero_ready) {
            finish_pitch(pitch, 4U);
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
            finish_pitch(pitch, 2U);
        }
    }
}

extern "C" void gimbal_servo_test_task(void *argument)
{
    auto& timer = *static_cast<TIM_HandleTypeDef*>(argument);
    port::Stm32PwmCh1 yaw(timer, TIM_CHANNEL_2);

    constexpr uint32_t pulses[] = {
        1500U, 1510U, 1530U, 1560U, 1500U
    };

    for (;;) {
        uint8_t command = 0U;
        /* Timeout 0 keeps the HAL UART lock held for only a few
           microseconds; the telemetry task shares this handle and
           would otherwise hit HAL_BUSY. */
        const auto rx = HAL_UART_Receive(
            &huart2, &command, 1U, 0U);

        if (rx == HAL_TIMEOUT) {
            /* Yield so lower-priority work (telemetry) is not starved. */
            (void)osDelay(5U);
            continue;
        }
        if (rx != HAL_OK) {
            servo_test_state = 4U;
            continue;
        }
#if GIMBAL_ENABLE_FAULT_INJECTION
        if (command == 'F' ||
            command == 'T' ||
            command == 'R') {
            gimbal_test_fault = command == 'F' ? 1U
                : command == 'T' ? 2U : 0U;
            continue;
        }
#endif

        if (command != 'P' && command != 'Y' &&
            command != 'Z' && command != 'C' &&
            command != 'X') {
            continue;
        }

        if (command == 'X') {
            const auto sent = osMessageQueuePut(
                pitch_requests, &command, 0U, 0U);

            pitch_request_status = sent == osOK ? 1U
                : sent == osErrorResource ? 2U : 3U;
            continue;
        }

        if (command == 'Y' && pitch_test_state == 1U) {
            servo_test_state = 3U;
            continue;
        }

        if (command == 'P' || command == 'Z' ||
            command == 'C') {
            if (!servo_test_allowed || pitch_test_state == 1U) {
                if (command == 'Z') {
                    pitch_zero_status = 2U;
                } else {
                    pitch_request_status = 2U;
                }
                continue;
            }

            const auto sent = osMessageQueuePut(
                pitch_requests, &command, 0U, 0U);

            if (command == 'Z') {
                pitch_zero_status = sent == osOK ? 0U
                    : sent == osErrorResource ? 2U : 3U;
            } else {
                pitch_request_status = sent == osOK ? 1U
                    : sent == osErrorResource ? 2U : 3U;
            }
            continue;
        }

        /* Assembled gimbal: the P axis only travels upwards from its
           horizontal lower stop. The sequence below stays at or above
           the horizontal position so it never pushes into the stop. */
        if (!servo_test_allowed) {
            servo_test_state = 3U;
            continue;
        }

        servo_test_state = 1U;
        servo_test_pulse =
            __HAL_TIM_GET_COMPARE(&timer, TIM_CHANNEL_2);

        if (yaw.start() != bsp::Error::Ok) {
            servo_test_state = 4U;
            (void)yaw.stop();
            continue;
        }

        bool completed = true;
        for (const uint32_t pulse : pulses) {
            if (!hold_pulse(yaw, pulse)) {
                completed = false;
                break;
            }
        }

        const auto stopped = yaw.stop();
        servo_test_state =
            completed && stopped == bsp::Error::Ok ? 2U : 4U;
    }
}
