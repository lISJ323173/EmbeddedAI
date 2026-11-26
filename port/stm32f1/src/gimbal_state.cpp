#include "gimbal_observe.h"
#include "gimbal_transport.hpp"

extern "C" {

volatile uint32_t bluetooth_rx_errors = 0U;
volatile uint32_t bluetooth_rx_busy_count = 0U;
volatile uint32_t bluetooth_rx_last_error = 0U;
volatile int32_t pitch_target_mdeg = 20000;
volatile int32_t pitch_error_mdeg = 0;
volatile uint32_t pitch_closed_loop_enabled = 0U;

volatile uint32_t pitch_test_state = GIMBAL_AXIS_IDLE;
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

volatile int32_t yaw_target_mdeg = 0;
volatile uint32_t yaw_target_apply_count = 0U;
volatile uint32_t yaw_target_apply_ms = 0U;

volatile uint32_t pitch_last_fault = GIMBAL_FAULT_NONE;
volatile uint32_t yaw_last_fault = GIMBAL_FAULT_NONE;

volatile uint32_t filter_request_status = 0U;
volatile uint32_t filter_apply_status = GIMBAL_PARAM_IDLE;
volatile uint32_t filter_apply_count = 0U;

}

namespace gimbal::detail {

osMessageQueueId_t pitch_requests = nullptr;
osEventFlagsId_t yaw_events = nullptr;
osMessageQueueId_t filter_requests = nullptr;

}
