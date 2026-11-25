#include "main.h"
#include "cmsis_os2.h"
#include "gimbal_control.h"
#include "gimbal_observe.h"
#include "gimbal_transport.hpp"

extern "C" {
extern UART_HandleTypeDef huart2;
}

using gimbal::detail::pitch_requests;
using gimbal::detail::yaw_events;
using gimbal::detail::yaw_start_flag;
using gimbal::detail::yaw_stop_flag;
using gimbal::detail::yaw_hold_flag;
using gimbal::detail::pitch_stop_flag;
using gimbal::detail::filter_requests;

extern "C" int gimbal_pitch_init(void)
{
    pitch_requests = osMessageQueueNew(
        1U, sizeof(uint8_t), nullptr);
    yaw_events = osEventFlagsNew(nullptr);
    filter_requests = osMessageQueueNew(
        1U, sizeof(uint32_t), nullptr);

    return pitch_requests != nullptr &&
           yaw_events != nullptr &&
           filter_requests != nullptr;
}

extern "C" int gimbal_filter_take_request(uint32_t *gain_milli)
{
    if (gain_milli == nullptr) return -1;

    const auto result = osMessageQueueGet(
        filter_requests, gain_milli, nullptr, 0U);

    if (result == osOK) return 1;
    if (result == osErrorResource) return 0;
    return -1;
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

        if (rx == HAL_TIMEOUT || rx == HAL_BUSY) {
            if (rx == HAL_BUSY) {
                ++bluetooth_rx_busy_count;
            }
            (void)osDelay(5U);
            continue;
        }

        if (rx != HAL_OK) {
            ++bluetooth_rx_errors;
            bluetooth_rx_last_error =
                static_cast<uint32_t>(rx);
            (void)osDelay(5U);
            continue;
        }

        if (command >= '1' && command <= '3') {
            const uint32_t gain_milli =
                command == '1' ? 500U
                : command == '2' ? 2000U : 5000U;

            const auto result = osMessageQueuePut(
                filter_requests, &gain_milli, 0U, 0U);

            filter_request_status = result == osOK ? 1U
                : result == osErrorResource ? 2U : 3U;
            continue;
        }
#if GIMBAL_ENABLE_FAULT_INJECTION
        if (command == 'F' ||
            command == 'T' ||
            command == 'I' ||
            command == 'R') {
            gimbal_test_fault = command == 'F' ? 1U
                : command == 'T' ? 2U
                : command == 'I' ? 3U : 0U;
            continue;
        }
#endif

        if (command == 'B') {
            if (pitch_test_state == GIMBAL_AXIS_RUNNING ||
                servo_test_state == GIMBAL_AXIS_RUNNING) {
                gyro_recalibration_status = 2U;
            } else {
                gyro_recalibration_status = 1U;
                gyro_recalibrate_request = 1U;
            }
            continue;
        }

        if (command != 'P' && command != 'Y' &&
            command != 'Z' && command != 'C' &&
            command != 'X' && command != 'H' &&
            command != 'S') {
            continue;
        }

        if (command == 'X') {
            const uint32_t flags = osEventFlagsSet(
                yaw_events, yaw_stop_flag | pitch_stop_flag);

            const uint32_t status =
                (flags & osFlagsError) != 0U ? 3U : 1U;

            yaw_request_status = status;
            pitch_request_status = status;
            continue;
        }

        if (command == 'Y' || command == 'H') {
            const bool pitch_blocks_command =
                pitch_test_state == GIMBAL_AXIS_RUNNING &&
                (command != 'H' ||
                 !pitch_closed_loop_enabled ||
                 pitch_step_status == 1U);

            if (!servo_test_allowed ||
                pitch_blocks_command ||
                servo_test_state == GIMBAL_AXIS_RUNNING ||
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
            command == 'C' || command == 'S') {
            const bool yaw_blocks_command =
                servo_test_state == GIMBAL_AXIS_RUNNING &&
                (command != 'C' ||
                 !yaw_closed_loop_enabled ||
                 !yaw_reference_ready);

            if (!servo_test_allowed ||
                pitch_test_state == GIMBAL_AXIS_RUNNING ||
                yaw_blocks_command ||
                (command == 'S' && !pitch_zero_ready)) {
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
