#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

uint32_t pitch_angle_to_pulse(float angle_deg,
                              uint32_t *pulse_us);

#ifdef __cplusplus
}
#endif
