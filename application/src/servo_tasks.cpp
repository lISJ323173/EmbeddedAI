#include "application/servo_tasks.hpp"
#include "device/servo_command_parser.hpp"

namespace application {

volatile std::uint32_t servo_rx_enqueued = 0U;
volatile std::uint32_t servo_output_applied = 0U;
volatile std::uint32_t servo_rx_stack_free_bytes = 0U;
volatile std::uint32_t servo_output_stack_free_bytes = 0U;

static void report_alive(osEventFlagsId_t events, std::uint32_t flag,
                         void (*on_failure)())
{
    if ((osEventFlagsSet(events, flag) & osFlagsError) != 0U) {
        on_failure();
    }
}

void servo_rx_task(void* argument)
{
    auto& context = *static_cast<ServoRxContext*>(argument);
    device::ServoCommandParser parser;
    unsigned position = 50U;

    for (;;) {
        std::uint8_t byte{};
        const auto status = context.uart.receive(&byte, 1U, 10U);
        if (status == bsp::Error::Ok || status == bsp::Error::Timeout) {
            report_alive(context.alive_events, rx_alive, context.on_failure);
        }
        if (status == bsp::Error::Timeout) {
            static std::uint32_t last_sampled_count = 0U;
            const auto count = servo_rx_enqueued;
            if (servo_rx_stack_free_bytes == 0U ||
                count - last_sampled_count >= 4U) {
                servo_rx_stack_free_bytes = osThreadGetStackSpace(osThreadGetId());
                last_sampled_count = count;
            }
            osDelay(1U);
            continue;
        }
        if (status != bsp::Error::Ok) {
            context.on_failure();
        }
        if (parser.push(static_cast<char>(byte), position)) {
            const auto command = static_cast<std::int32_t>(position);
            if (osMessageQueuePut(context.commands, &command, 0U, 0U) != osOK) {
                context.on_failure();
            }
            ++servo_rx_enqueued;
        }
    }
}

void servo_output_task(void* argument)
{
    auto& context = *static_cast<ServoOutputContext*>(argument);

    for (;;) {
        std::int32_t command{};
        const auto status = osMessageQueueGet(
            context.commands, &command, nullptr, 100U);

        if (status == osOK) {
            if (context.servo.set_position_percent(command) != bsp::Error::Ok) {
                context.on_failure();
            }
            ++servo_output_applied;
            servo_output_stack_free_bytes = osThreadGetStackSpace(osThreadGetId());
        } else if (status != osErrorTimeout) {
            context.on_failure();
        }

        if (!context.suppress_alive) {
            report_alive(context.alive_events, output_alive, context.on_failure);
        }
    }
}

} // namespace application
