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

volatile uint32_t yaw_pwm_updates = 0U;
volatile uint32_t yaw_pwm_errors = 0U;
volatile uint32_t yaw_request_status = 0U;

volatile uint32_t yaw_closed_loop_enabled = 0U;
volatile int32_t yaw_reference_mdeg = 0;
volatile int32_t yaw_feedback_mdeg = 0;
volatile int32_t yaw_error_mdeg = 0;

volatile int32_t yaw_feedback_min_mdeg = 0;
volatile int32_t yaw_feedback_max_mdeg = 0;
volatile int32_t yaw_integral_millius = 0;
volatile uint32_t yaw_outside_deadband_samples = 0U;
volatile uint32_t yaw_reference_ready = 0U;
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


constexpr uint32_t yaw_start_flag = 1U;
constexpr uint32_t yaw_stop_flag = 2U;
constexpr uint32_t yaw_hold_flag = 4U;
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
    1500U, 1440U, 1500U
};

osEventFlagsId_t yaw_events = nullptr;
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

extern "C" int gimbal_pitch_init(void)
{
    pitch_requests = osMessageQueueNew(
        1U, sizeof(uint8_t), nullptr);
    yaw_events = osEventFlagsNew(nullptr);

    return pitch_requests != nullptr && yaw_events != nullptr;
}

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
        if (pitch_test_state == 1U ||
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
        if (write_yaw(yaw, target)) {
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
    (void)argument;

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
            command != 'X' && command != 'H') {
            continue;
        }

        if (command == 'X') {
            const uint32_t flags =
                osEventFlagsSet(yaw_events, yaw_stop_flag);
            yaw_request_status =
                (flags & osFlagsError) != 0U ? 3U : 1U;

            const auto sent = osMessageQueuePut(
                pitch_requests, &command, 0U, 0U);
            pitch_request_status = sent == osOK ? 1U
                : sent == osErrorResource ? 2U : 3U;
            continue;
        }

        if (command == 'Y' || command == 'H') {
            if (!servo_test_allowed ||
                pitch_test_state == 1U ||
                servo_test_state == 1U ||
                (command == 'H' && servo_test_pulse != 1500U)) {
                yaw_request_status = 2U;
                continue;
            }

            const uint32_t requested_flag =
                command == 'H' ? yaw_hold_flag : yaw_start_flag;
            const uint32_t flags =
                osEventFlagsSet(yaw_events, requested_flag);

            yaw_request_status =
                (flags & osFlagsError) != 0U ? 3U : 1U;
            continue;
        }

        if (command == 'P' || command == 'Z' ||
            command == 'C') {
            if (!servo_test_allowed ||
                pitch_test_state == 1U ||
                servo_test_state == 1U) {
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
    }
}
