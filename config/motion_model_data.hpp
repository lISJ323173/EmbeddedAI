#pragma once
#include <cstdint>

namespace motion_model {
inline constexpr std::int8_t dense1_weights[128] = {
    10, -127, -90, -110, -86, -85, 7, -49, 13, -127, -21, 35,
    50, 74, 48, 84, -79, -39, -78, -18, -80, -127, -68, -50,
    41, -127, -78, -50, -97, -79, -107, -65, -127, -70, 80, -64,
    99, -35, 67, 82, 3, 127, -46, -45, 108, -44, 81, -72,
    97, 127, 48, 71, 51, 95, 43, -45, -54, 28, 46, 123,
    110, 127, -39, -48, -65, 32, -55, -31, 127, 13, -74, 26,
    13, -89, 111, -59, 1, 65, 127, -122, -103, 60, 51, -98,
    122, 112, -25, -127, -47, -44, -52, -127, -124, -7, -62, 29,
    109, 120, 106, 89, 120, 57, 127, 76, 71, 48, -34, 127,
    40, -46, 17, -21, -107, -68, 75, -11, 105, 102, -64, 127,
    86, 127, -87, 73, -52, -3, 31, -30,
};
inline constexpr std::int32_t dense1_bias[16] = {
    1335, 3319, 2093, 322, 5109, 3963, -127, -3795, 7458, -1597, -2045, 1091,
    602, -4889, 8945, -3604,
};
inline constexpr float dense1_weight_scale[16] = {
    6.949836854e-03f, 4.264242016e-03f, 5.960460287e-03f, 6.291471887e-03f, 7.002716884e-03f, 5.154378247e-03f, 6.245652679e-03f, 3.947724123e-03f, 4.376827274e-03f, 3.107317025e-03f, 3.932015039e-03f, 6.062826142e-03f,
    5.324234255e-03f, 5.433988292e-03f, 4.238042980e-03f, 7.608716376e-03f,
};
inline constexpr std::int32_t dense1_q31[16] = {
    5800332, 3558935, 4974599, 5250861, 5844466, 4301843, 5212620, 3294770, 3652899, 2593366, 3281659, 5060033,
    4443605, 4535205, 3537070, 6350233,
};
inline constexpr float dense1_input_scale = 2.275349572e-02f;
inline constexpr int dense1_input_zero = -92;
inline constexpr float dense1_output_scale = 5.854633823e-02f;
inline constexpr int dense1_output_zero = -128;
inline constexpr std::int8_t dense2_weights[48] = {
    81, 13, 49, 8, -47, -127, -60, 28, -60, -20, -15, 69,
    -62, -22, -73, 27, -36, 32, -13, -56, 74, 32, -10, 2,
    86, -39, -55, -69, -40, -79, 67, -127, 1, -51, -108, -30,
    -82, 74, 64, 85, -55, 34, 89, -108, -36, 127, -2, 78,
};
inline constexpr std::int32_t dense2_bias[3] = {
    -299, 649, -937,
};
inline constexpr float dense2_weight_scale[3] = {
    1.097500697e-02f, 1.206095051e-02f, 8.277348243e-03f,
};
inline constexpr std::int32_t dense2_q31[3] = {
    10814773, 11884862, 8156500,
};
inline constexpr float dense2_input_scale = 5.854633823e-02f;
inline constexpr int dense2_input_zero = -128;
inline constexpr float dense2_output_scale = 1.275901049e-01f;
inline constexpr int dense2_output_zero = 36;
}
