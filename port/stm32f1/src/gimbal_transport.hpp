#pragma once

#include "cmsis_os2.h"

namespace gimbal::detail {

extern osMessageQueueId_t pitch_requests;
extern osEventFlagsId_t yaw_events;
extern osMessageQueueId_t filter_requests;

inline constexpr uint32_t yaw_start_flag = 1U;
inline constexpr uint32_t yaw_stop_flag = 2U;
inline constexpr uint32_t yaw_hold_flag = 4U;
inline constexpr uint32_t pitch_stop_flag = 8U;

inline constexpr uint32_t yaw_target_neg_flag = 16U;
inline constexpr uint32_t yaw_target_zero_flag = 32U;
inline constexpr uint32_t yaw_target_pos_flag = 64U;
inline constexpr uint32_t yaw_target_flags =
    yaw_target_neg_flag | yaw_target_zero_flag | yaw_target_pos_flag;

} // namespace gimbal::detail
