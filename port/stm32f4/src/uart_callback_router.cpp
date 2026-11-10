#include "uart_callback_router.hpp"
#include "stm32_uart.hpp"
#include <array>

namespace port::uart_router {
namespace {

struct Entry {
    UART_HandleTypeDef* handle = nullptr;
    Stm32Uart* instance = nullptr;
};

std::array<Entry, 4> entries{};

} // namespace

bsp::Error register_uart(
    UART_HandleTypeDef* handle,
    Stm32Uart* instance) noexcept
{
    if (handle == nullptr || instance == nullptr)
        return bsp::Error::InvalidArgument;

    for (const auto& entry : entries) {
        if (entry.handle == handle)
            return bsp::Error::Busy;
    }

    for (auto& entry : entries) {
        if (entry.handle == nullptr) {
            entry.instance = instance;
            entry.handle = handle;
            return bsp::Error::Ok;
        }
    }

    return bsp::Error::Busy;
}

bsp::Error unregister_uart(
    UART_HandleTypeDef* handle) noexcept
{
    if (handle == nullptr)
        return bsp::Error::InvalidArgument;

    for (auto& entry : entries) {
        if (entry.handle == handle) {
            entry.handle = nullptr;
            entry.instance = nullptr;
            return bsp::Error::Ok;
        }
    }

    return bsp::Error::NotInitialized;
}
namespace {

Stm32Uart* find_uart(
    UART_HandleTypeDef* handle) noexcept
{
    if (handle == nullptr) {
        return nullptr;
    }

    for (const auto& entry : entries) {
        if (entry.handle == handle) {
            return entry.instance;
        }
    }

    return nullptr;
}

} // namespace

void on_tx_complete(
    UART_HandleTypeDef* handle) noexcept
{
    if (auto* uart = find_uart(handle)) {
        uart->on_tx_complete();
    }
}

void on_rx_complete(
    UART_HandleTypeDef* handle) noexcept
{
    if (auto* uart = find_uart(handle)) {
        uart->on_rx_complete();
    }
}

void on_error(
    UART_HandleTypeDef* handle) noexcept
{
    if (auto* uart = find_uart(handle)) {
        uart->on_error(bsp::Error::HardwareFailure);
    }
}
} // namespace port::uart_router