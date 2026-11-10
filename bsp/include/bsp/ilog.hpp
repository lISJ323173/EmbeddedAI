#pragma once

#include <cstdint>
#include "bsp/error.hpp"

namespace bsp {

enum class LogLevel : std::uint8_t {
    Debug,
    Info,
    Warning,
    Error
};

class ILog {
public:
    virtual ~ILog() = default;

    virtual Error write(
        LogLevel level,
        const char* message
    ) noexcept = 0;
};

} // namespace bsp