#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int gimbal_pitch_init(void);

/* Returns 1=request, 0=no request, -1=interface error. */
int gimbal_filter_take_request(uint32_t *gain_milli);

void gimbal_pitch_tick(void *argument,
                       float roll_deg,
                       float dt_seconds);

void gimbal_yaw_tick(void *argument,
                     float yaw_deg,
                     float dt_seconds);

void gimbal_servo_test_task(void *argument);

#ifdef __cplusplus
}
#endif
