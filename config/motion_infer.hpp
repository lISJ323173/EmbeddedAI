#pragma once
#include "motion_model_data.hpp"

namespace motion_model {

template <unsigned Inputs, unsigned Outputs>
inline void dense(const std::int8_t (&input)[Inputs],
                  const std::int8_t (&weights)[Inputs * Outputs],
                  const std::int32_t (&bias)[Outputs],
                  const std::int32_t (&q31)[Outputs],
                  int input_zero, int output_zero, int lower,
                  std::int8_t (&output)[Outputs])
{
    constexpr std::int64_t unit = std::int64_t{1} << 31;
    for (unsigned o = 0; o < Outputs; ++o) {
        std::int32_t acc = bias[o];
        for (unsigned i = 0; i < Inputs; ++i) {
            acc += (static_cast<int>(input[i]) - input_zero) *
                   static_cast<int>(weights[o * Inputs + i]);
        }
        const std::int64_t product = static_cast<std::int64_t>(acc) * q31[o];
        const auto rounded = (product + (product >= 0 ? unit / 2 : -unit / 2)) / unit;
        const auto scaled = rounded + output_zero;
        output[o] = static_cast<std::int8_t>(
            scaled < lower ? lower : scaled > 127 ? 127 : scaled);
    }
}

inline void infer(const std::int8_t (&input)[8], std::int8_t (&output)[3])
{
    std::int8_t hidden[16];
    dense(input, dense1_weights, dense1_bias, dense1_q31,
          dense1_input_zero, dense1_output_zero, dense1_output_zero, hidden);
    dense(hidden, dense2_weights, dense2_bias, dense2_q31,
          dense2_input_zero, dense2_output_zero, -128, output);
}

}  // namespace motion_model
