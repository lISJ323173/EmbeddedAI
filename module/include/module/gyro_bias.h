#pragma once

#include <stdint.h>

typedef struct {
    uint32_t count;
    uint32_t ready;
    float mean[3];
    float m2[3];
} GyroBias;

#ifdef __cplusplus
extern "C" {
#endif

void gyro_bias_reset(GyroBias *state);
void gyro_bias_update(GyroBias *state,
                      const int32_t accel[3],
                      const int32_t gyro[3]);
int32_t gyro_bias_rate_mdeg(const GyroBias *state,
                            unsigned axis, int32_t raw);

#ifdef __cplusplus
}
#endif
