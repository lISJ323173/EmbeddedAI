#include "module/attitude.h"
#include <algorithm>
#include <cmath>

namespace {

constexpr float radians_per_degree = 0.01745329252F;
constexpr float degrees_per_radian = 57.29577951F;

void update_angles(Attitude *state)
{
    const float *q = state->q;
    state->roll = std::atan2(
        2.0F * (q[0] * q[1] + q[2] * q[3]),
        1.0F - 2.0F * (q[1] * q[1] + q[2] * q[2]))
        * degrees_per_radian;

    const float sine_pitch = std::clamp(
        2.0F * (q[0] * q[2] - q[3] * q[1]), -1.0F, 1.0F);
    state->pitch = std::asin(sine_pitch) * degrees_per_radian;
    state->yaw = std::atan2(
        2.0F * (q[0] * q[3] + q[1] * q[2]),
        1.0F - 2.0F * (q[2] * q[2] + q[3] * q[3]))
        * degrees_per_radian;
}

void initialize_from_accel(Attitude *state, const float accel[3])
{
    const float roll = std::atan2(accel[1], accel[2]);
    const float pitch = std::atan2(
        -accel[0], std::sqrt(
            accel[1] * accel[1] + accel[2] * accel[2]));

    const float cr = std::cos(roll * 0.5F);
    const float sr = std::sin(roll * 0.5F);
    const float cp = std::cos(pitch * 0.5F);
    const float sp = std::sin(pitch * 0.5F);

    state->q[0] = cr * cp;
    state->q[1] = sr * cp;
    state->q[2] = cr * sp;
    state->q[3] = -sr * sp;
    state->status = 1U;
    update_angles(state);
}

} // namespace

void attitude_update(Attitude *state,
                     const float accel_g[3],
                     const float gyro_dps[3],
                     float dt_seconds)
{
    if (state->status == 2U) {
        return;
    }
    if (!std::isfinite(dt_seconds) ||
        dt_seconds < 0.005F || dt_seconds > 0.020F ||
        !std::isfinite(state->correction_gain) ||
        state->correction_gain < 0.0F ||
        state->correction_gain > 20.0F) {
        state->status = 2U;
        return;
    }

    float accel_squared = 0.0F;
    float omega[3];
    for (unsigned i = 0; i < 3; ++i) {
        if (!std::isfinite(accel_g[i]) ||
            !std::isfinite(gyro_dps[i])) {
            state->status = 2U;
            return;
        }
        accel_squared += accel_g[i] * accel_g[i];
        omega[i] = gyro_dps[i] * radians_per_degree;
    }

    const bool use_accel =
        accel_squared >= 0.81F && accel_squared <= 1.21F;
    if (state->status == 0U) {
        if (use_accel) {
            initialize_from_accel(state, accel_g);
        }
        return;
    }

    const float *q = state->q;
    if (use_accel) {
        const float inverse_norm = 1.0F / std::sqrt(accel_squared);
        const float a[3] = {
            accel_g[0] * inverse_norm,
            accel_g[1] * inverse_norm,
            accel_g[2] * inverse_norm
        };
        const float gravity[3] = {
            2.0F * (q[1] * q[3] - q[0] * q[2]),
            2.0F * (q[0] * q[1] + q[2] * q[3]),
            1.0F - 2.0F * (q[1] * q[1] + q[2] * q[2])
        };
        omega[0] += state->correction_gain *
            (a[1] * gravity[2] - a[2] * gravity[1]);
        omega[1] += state->correction_gain *
            (a[2] * gravity[0] - a[0] * gravity[2]);
        omega[2] += state->correction_gain *
            (a[0] * gravity[1] - a[1] * gravity[0]);
    }

    const float derivative[4] = {
        -q[1] * omega[0] - q[2] * omega[1] - q[3] * omega[2],
         q[0] * omega[0] + q[2] * omega[2] - q[3] * omega[1],
         q[0] * omega[1] - q[1] * omega[2] + q[3] * omega[0],
         q[0] * omega[2] + q[1] * omega[1] - q[2] * omega[0]
    };

    float norm_squared = 0.0F;
    for (unsigned i = 0; i < 4; ++i) {
        state->q[i] += 0.5F * derivative[i] * dt_seconds;
        norm_squared += state->q[i] * state->q[i];
    }

    if (!std::isfinite(norm_squared) || norm_squared < 0.01F) {
        state->status = 2U;
        return;
    }

    const float inverse_norm = 1.0F / std::sqrt(norm_squared);
    for (unsigned i = 0; i < 4; ++i) {
        state->q[i] *= inverse_norm;
    }
    update_angles(state);
}
