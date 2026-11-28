#pragma once
#include <cmath>
#include "motion_infer.hpp"
#include "motion_preprocess.hpp"

namespace motion_model {
class LiveWindow {
public:
    bool push(std::uint32_t t, int ax, int ay, int az,
              int gx, int gy, int gz, std::int8_t (&scores)[3])
    {
        if (filled_ != 0 && (t - last_ < 10U || t - last_ > 30U)) {
            next_ = filled_ = since_ = 0;
        }
        last_ = t;
        acc_[next_] = norm(ax, ay, az) / 4096.0f;
        gyro_[next_] = norm(gx, gy, gz) / 32.8f;
        next_ = (next_ + 1U) % 100U;
        if (filled_ < 100U) {
            if (++filled_ < 100U) return false;
        } else if (++since_ < 50U) {
            return false;
        }
        since_ = 0;
        float features[8];
        stats(acc_, features);
        stats(gyro_, features + 4);
        std::int8_t input[8];
        for (unsigned i = 0; i < 8U; ++i) {
            const float z = (features[i] - feature_mean[i]) / feature_std[i];
            const float q = std::nearbyint(z / dense1_input_scale) + dense1_input_zero;
            input[i] = static_cast<std::int8_t>(q < -128.0f ? -128.0f : q > 127.0f ? 127.0f : q);
        }
        infer(input, scores);
        return true;
    }

private:
    static float norm(float x, float y, float z)
    {
        return std::sqrt(x * x + y * y + z * z);
    }

    void stats(const float (&values)[100], float* out) const
    {
        float sum = 0, variance = 0, peak = 0, delta = 0, prev = values[next_];
        for (float value : values) sum += value;
        const float mean = sum / 100.0f;
        for (unsigned i = 0; i < 100U; ++i) {
            const float value = values[(next_ + i) % 100U];
            const float error = value - mean;
            variance += error * error;
            if (value > peak) peak = value;
            delta += std::fabs(value - prev);
            prev = value;
        }
        out[0] = mean;
        out[1] = std::sqrt(variance / 100.0f);
        out[2] = peak;
        out[3] = delta / 99.0f;
    }

    float acc_[100]{}, gyro_[100]{};
    unsigned next_ = 0, filled_ = 0, since_ = 0;
    std::uint32_t last_ = 0;
};
}  // namespace motion_model
