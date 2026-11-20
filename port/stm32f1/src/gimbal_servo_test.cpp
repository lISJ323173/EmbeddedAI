#include "main.h"
#include "cmsis_os2.h"
#include "stm32_pwm.hpp"

extern "C" {
extern UART_HandleTypeDef huart2;
extern volatile uint32_t servo_test_allowed;
extern volatile uint32_t servo_test_state;
extern volatile uint32_t servo_test_pulse;
}

namespace {

bool hold_pulse(port::Stm32PwmCh1& pwm, uint32_t pulse, uint32_t ticks)
{
    if (pwm.set_pulse_us(pulse) != bsp::Error::Ok) {
        return false;
    }
    servo_test_pulse = pulse;

    for (unsigned i = 0; i < ticks; ++i) {
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

extern "C" void gimbal_servo_test_task(void *argument)
{
    auto& timer = *static_cast<TIM_HandleTypeDef*>(argument);
    port::Stm32PwmCh1 pitch(timer, TIM_CHANNEL_1);
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
        if (command != 'P' && command != 'Y') {
            continue;
        }

        /* Assembled gimbal: the P axis only travels upwards from its
           horizontal lower stop. The sequence below stays at or above
           the horizontal position so it never pushes into the stop. */
        if (!servo_test_allowed) {
            servo_test_state = 3U;
            continue;
        }

        auto& pwm = command == 'P' ? pitch : yaw;
        servo_test_state = 1U;
        servo_test_pulse = 1500U;

        if (pwm.start() != bsp::Error::Ok) {
            servo_test_state = 4U;
            (void)pwm.stop();
            continue;
        }

        bool completed = true;

        if (command == 'P') {
            /* Sweep upwards only: the platform's lower stop sits right
               at 1500 us, anything below that stalls the servo. */
            constexpr uint32_t step_ticks = 200U;

            for (uint32_t p = 1500U; p <= 2500U; p += 100U) {
                if (!hold_pulse(pwm, p, step_ticks)) {
                    completed = false;
                    break;
                }
            }
            if (completed) {
                completed = hold_pulse(pwm, 1500U, step_ticks);
            }
        } else {
            for (const auto pulse : pulses) {
                if (!hold_pulse(pwm, pulse, 300U)) {
                    completed = false;
                    break;
                }
            }
        }

        const auto stopped = pwm.stop();
        servo_test_state =
            completed && stopped == bsp::Error::Ok ? 2U : 4U;
    }
}
