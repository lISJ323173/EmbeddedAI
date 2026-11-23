#include "module/pitch_map.h"
#include "bsp/error.hpp"
#include <cmath>

namespace {

struct CalibrationPoint {
    float angle;
    uint32_t pulse;
};

constexpr CalibrationPoint points[] = {
    {0.000F, 1500U},
    {11.100F, 1650U},
    {23.490F, 1800U},
    {35.425F, 1950U},
    {49.770F, 2100U}
};

} // namespace

uint32_t pitch_angle_to_pulse(float angle_deg,
                              uint32_t *pulse_us)
{
    if (pulse_us == nullptr ||
        !std::isfinite(angle_deg) ||
        angle_deg < 5.0F || angle_deg > 45.0F) {
        return static_cast<uint32_t>(
            bsp::Error::InvalidArgument);
    }

    for (unsigned i = 1U; i < 5U; ++i) {
        if (angle_deg <= points[i].angle) {
            const auto& lower = points[i - 1U];
            const auto& upper = points[i];
            const float fraction =
                (angle_deg - lower.angle) /
                (upper.angle - lower.angle);

            const float pulse = lower.pulse +
                fraction * (upper.pulse - lower.pulse);

            *pulse_us = static_cast<uint32_t>(pulse + 0.5F);
            return static_cast<uint32_t>(bsp::Error::Ok);
        }
    }

    return static_cast<uint32_t>(bsp::Error::InvalidArgument);
}
