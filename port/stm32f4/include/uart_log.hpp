#pragma once

#include "bsp/ilog.hpp"
#include "bsp/iuart.hpp"

namespace port {

class UartLog final : public bsp::ILog {
public:
    explicit UartLog(bsp::IUart& uart) noexcept : uart_(uart) {}

    bsp::Error write(bsp::LogLevel level, const char* message) noexcept override
    {
        const char* prefix = nullptr;
        switch (level) {
        case bsp::LogLevel::Debug:   prefix = "[DEBUG] "; break;
        case bsp::LogLevel::Info:    prefix = "[INFO] "; break;
        case bsp::LogLevel::Warning: prefix = "[WARN] "; break;
        case bsp::LogLevel::Error:   prefix = "[ERROR] "; break;
        default: return bsp::Error::InvalidArgument;
        }
        if (message == nullptr || message[0] == '\0') {
            return bsp::Error::InvalidArgument;
        }

        constexpr std::size_t max_message_bytes = 80U;
        std::uint8_t line[8U + max_message_bytes + 2U]{};
        std::size_t used = 0U;

        for (; *prefix != '\0'; ++prefix) {
            line[used++] = static_cast<std::uint8_t>(*prefix);
        }
        for (std::size_t i = 0U; message[i] != '\0'; ++i) {
            if (i == max_message_bytes || message[i] == '\r' || message[i] == '\n') {
                return bsp::Error::InvalidArgument;
            }
            line[used++] = static_cast<std::uint8_t>(message[i]);
        }
        line[used++] = '\r';
        line[used++] = '\n';
        return uart_.transmit(line, used, 100U);
    }

private:
    bsp::IUart& uart_;
};

} // namespace port
