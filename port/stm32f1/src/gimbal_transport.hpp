#pragma once

#include "cmsis_os2.h"

namespace gimbal::detail {

extern osMessageQueueId_t pitch_requests;
extern osEventFlagsId_t yaw_events;

inline constexpr uint32_t yaw_start_flag = 1U;
inline constexpr uint32_t yaw_stop_flag = 2U;
inline constexpr uint32_t yaw_hold_flag = 4U;
inline constexpr uint32_t pitch_stop_flag = 8U;

} // namespace gimbal::detail
