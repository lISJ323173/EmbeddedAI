#pragma once

#include <stdint.h>
#include "gimbal_status.h"

#ifdef __cplusplus
extern "C" {
#endif

extern volatile uint32_t servo_test_allowed;
extern volatile uint32_t servo_test_state;
extern volatile uint32_t servo_test_pulse;
extern volatile uint32_t gimbal_test_fault;
extern volatile uint32_t gyro_recalibrate_request;
extern volatile uint32_t gyro_recalibration_status;

extern volatile int32_t pitch_target_mdeg;
extern volatile int32_t pitch_error_mdeg;
extern volatile uint32_t pitch_closed_loop_enabled;
extern volatile uint32_t pitch_test_state;
extern volatile uint32_t pitch_test_pulse;
extern volatile uint32_t pitch_pwm_updates;
extern volatile uint32_t pitch_pwm_errors;
extern volatile uint32_t pitch_request_status;
extern volatile int32_t pitch_roll_mdeg;
extern volatile int32_t pitch_zero_mdeg;
extern volatile int32_t pitch_feedback_mdeg;
extern volatile uint32_t pitch_zero_ready;
extern volatile uint32_t pitch_zero_status;

extern volatile uint32_t yaw_pwm_updates;
extern volatile uint32_t yaw_pwm_errors;
extern volatile uint32_t yaw_request_status;
extern volatile uint32_t yaw_closed_loop_enabled;
extern volatile int32_t yaw_reference_mdeg;
extern volatile int32_t yaw_feedback_mdeg;
extern volatile int32_t yaw_error_mdeg;
extern volatile int32_t yaw_feedback_min_mdeg;
extern volatile int32_t yaw_feedback_max_mdeg;
extern volatile int32_t yaw_integral_millius;
extern volatile uint32_t yaw_outside_deadband_samples;
extern volatile uint32_t yaw_reference_ready;

extern volatile uint32_t bluetooth_rx_errors;
extern volatile uint32_t bluetooth_rx_busy_count;
extern volatile uint32_t bluetooth_rx_last_error;

extern volatile uint32_t pitch_last_fault;
extern volatile uint32_t yaw_last_fault;

#ifdef __cplusplus
}
#endif
