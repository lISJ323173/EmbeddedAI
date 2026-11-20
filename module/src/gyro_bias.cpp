#include "module/gyro_bias.h"
#include <cmath>

void gyro_bias_reset(GyroBias *state)
{
    *state = {};
}

void gyro_bias_update(GyroBias *state,
                      const int32_t accel[3],
                      const int32_t gyro[3])
{
    if (state->ready) {
        return;
    }

    float accel_squared = 0.0F;
    for (unsigned i = 0; i < 3; ++i) {
        const float value = static_cast<float>(accel[i]);
        accel_squared += value * value;
        if (std::fabs(static_cast<float>(gyro[i])) > 655.0F) {
            gyro_bias_reset(state);
            return;
        }
    }

    constexpr float gravity = 16384.0F;
    if (accel_squared < 0.81F * gravity * gravity ||
        accel_squared > 1.21F * gravity * gravity) {
        gyro_bias_reset(state);
        return;
    }

    ++state->count;
    for (unsigned i = 0; i < 3; ++i) {
        const float delta = gyro[i] - state->mean[i];
        state->mean[i] += delta / state->count;
        state->m2[i] += delta * (gyro[i] - state->mean[i]);
    }

    if (state->count == 300U) {
        for (unsigned i = 0; i < 3; ++i) {
            if (state->m2[i] / 299.0F > 4290.25F) {
                gyro_bias_reset(state);
                return;
            }
        }
        state->ready = 1U;
    }
}

int32_t gyro_bias_rate_mdeg(const GyroBias *state,
                            unsigned axis, int32_t raw)
{
    if (!state->ready || axis >= 3U) {
        return 0;
    }
    return static_cast<int32_t>(
        (raw - state->mean[axis]) * (1000.0F / 131.0F));
}
