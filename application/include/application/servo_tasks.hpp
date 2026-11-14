#pragma once

#include <cstdint>
#include "bsp/iuart.hpp"
#include "cmsis_os2.h"
#include "device/sg90.hpp"

namespace application {

inline constexpr std::uint32_t rx_alive = 1U << 0;
inline constexpr std::uint32_t output_alive = 1U << 1;

struct ServoRxContext {
    bsp::IUart& uart;
    osMessageQueueId_t commands;
    osEventFlagsId_t alive_events;
    void (*on_failure)();
};

struct ServoOutputContext {
    device::Sg90& servo;
    osMessageQueueId_t commands;
    osEventFlagsId_t alive_events;
    void (*on_failure)();
    bool suppress_alive;
};

extern volatile std::uint32_t servo_rx_enqueued;
extern volatile std::uint32_t servo_output_applied;
extern volatile std::uint32_t servo_rx_stack_free_bytes;
extern volatile std::uint32_t servo_output_stack_free_bytes;

void servo_rx_task(void* argument);
void servo_output_task(void* argument);

} // namespace application
