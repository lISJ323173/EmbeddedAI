#pragma once

#include <cstddef>
#include <cstdint>
#include "bsp/error.hpp"

namespace bsp {

enum class UartTransferMode : std::uint8_t {
    Interrupt,
    Dma
};

enum class UartEvent : std::uint8_t {
    TxComplete,
    RxComplete,
    Error
};

using UartCallback = void (*)(
    void* context,
    UartEvent event,
    Error result
);

class IUartAsync {
public:
    virtual ~IUartAsync() = default;

    virtual Error start_transmit(
        const std::uint8_t* data,
        std::size_t size,
        UartTransferMode mode
    ) noexcept = 0;

    virtual Error start_receive(
        std::uint8_t* data,
        std::size_t size,
        UartTransferMode mode
    ) noexcept = 0;

    virtual void set_callback(
        UartCallback callback,
        void* context
    ) noexcept = 0;
};

} // namespace bsp