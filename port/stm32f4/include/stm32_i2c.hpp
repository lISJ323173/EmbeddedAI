#pragma once

#include <atomic>
#include "bsp/ii2c.hpp"
#include "stm32f4xx_hal.h"

namespace port {

class Stm32I2c final : public bsp::II2c, public bsp::II2cAsync {
public:
    explicit Stm32I2c(I2C_HandleTypeDef& handle) noexcept
        : handle_(handle) {}

    bsp::Error read_register(std::uint8_t address_7bit,
                             std::uint8_t reg,
                             std::uint8_t* data,
                             std::size_t size,
                             std::uint32_t timeout_ms) noexcept override;

    bsp::Error write_register(std::uint8_t address_7bit,
                              std::uint8_t reg,
                              std::uint8_t value,
                              std::uint32_t timeout_ms) noexcept override;

    bsp::Error start_read_register(std::uint8_t address_7bit,
                                   std::uint8_t reg, std::uint8_t* data,
                                   std::size_t size) noexcept override;
    bsp::Error poll_read_result() noexcept override;
    void on_read_complete(I2C_HandleTypeDef* handle) noexcept;
    void on_read_error(I2C_HandleTypeDef* handle) noexcept;

private:
    enum class ReadState : std::uint32_t { Idle, Active, Complete, Failed };

    static_assert(std::atomic<ReadState>::is_always_lock_free);
    std::atomic<ReadState> read_state_{ReadState::Idle};
    I2C_HandleTypeDef& handle_;
};

} // namespace port
