#pragma once

#include <stdint.h>

typedef struct {
    float q[4];
    float roll;
    float pitch;
    float yaw;
    float correction_gain;
    uint32_t status;
} Attitude;

#ifdef __cplusplus
extern "C" {
#endif

void attitude_update(Attitude *state,
                     const float accel_g[3],
                     const float gyro_dps[3],
                     float dt_seconds);

#ifdef __cplusplus
}
#endif
