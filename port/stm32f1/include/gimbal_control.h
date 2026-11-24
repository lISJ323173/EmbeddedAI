#pragma once

#ifdef __cplusplus
extern "C" {
#endif

int gimbal_pitch_init(void);

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
