#include "stm32_i2c.hpp"

#include <limits>

namespace port {

bsp::Error Stm32I2c::read_register(std::uint8_t address_7bit,
                                  std::uint8_t reg,
                                  std::uint8_t* data,
                                  std::size_t size,
                                  std::uint32_t timeout_ms) noexcept
{
    if (address_7bit < 0x08U || address_7bit > 0x77U ||
        data == nullptr || size == 0U ||
        size > std::numeric_limits<std::uint16_t>::max()) {
        return bsp::Error::InvalidArgument;
    }

    if (read_state_.load() != ReadState::Idle) {
        return bsp::Error::Busy;
    }

    if (HAL_I2C_GetState(&handle_) == HAL_I2C_STATE_RESET) {
        return bsp::Error::NotInitialized;
    }

    const auto status = HAL_I2C_Mem_Read(
        &handle_,
        static_cast<std::uint16_t>(address_7bit << 1),
        reg,
        I2C_MEMADD_SIZE_8BIT,
        data,
        static_cast<std::uint16_t>(size),
        timeout_ms);

    switch (status) {
    case HAL_OK:      return bsp::Error::Ok;
    case HAL_BUSY:    return bsp::Error::Busy;
    case HAL_TIMEOUT: return bsp::Error::Timeout;
    default:         return bsp::Error::HardwareFailure;
    }
}

bsp::Error Stm32I2c::write_register(std::uint8_t address_7bit,
                                   std::uint8_t reg,
                                   std::uint8_t value,
                                   std::uint32_t timeout_ms) noexcept
{
    if (address_7bit < 0x08U || address_7bit > 0x77U) {
        return bsp::Error::InvalidArgument;
    }
    if (read_state_.load() != ReadState::Idle) {
        return bsp::Error::Busy;
    }
    if (HAL_I2C_GetState(&handle_) == HAL_I2C_STATE_RESET) {
        return bsp::Error::NotInitialized;
    }

    const auto status = HAL_I2C_Mem_Write(
        &handle_, static_cast<std::uint16_t>(address_7bit << 1),
        reg, I2C_MEMADD_SIZE_8BIT, &value, 1U, timeout_ms);

    switch (status) {
    case HAL_OK:      return bsp::Error::Ok;
    case HAL_BUSY:    return bsp::Error::Busy;
    case HAL_TIMEOUT: return bsp::Error::Timeout;
    default:          return bsp::Error::HardwareFailure;
    }
}

bsp::Error Stm32I2c::start_read_register(std::uint8_t address_7bit,
                                        std::uint8_t reg, std::uint8_t* data,
                                        std::size_t size) noexcept
{
    if (address_7bit < 0x08U || address_7bit > 0x77U ||
        data == nullptr || size == 0U ||
        size > std::numeric_limits<std::uint16_t>::max()) {
        return bsp::Error::InvalidArgument;
    }
    if (read_state_.load() != ReadState::Idle) {
        return bsp::Error::Busy;
    }
    if (HAL_I2C_GetState(&handle_) == HAL_I2C_STATE_RESET) {
        return bsp::Error::NotInitialized;
    }

    read_state_.store(ReadState::Active);
    const auto status = HAL_I2C_Mem_Read_IT(
        &handle_, static_cast<std::uint16_t>(address_7bit << 1),
        reg, I2C_MEMADD_SIZE_8BIT, data, static_cast<std::uint16_t>(size));
    if (status == HAL_OK) {
        return bsp::Error::Ok;
    }

    read_state_.store(ReadState::Idle);
    switch (status) {
    case HAL_BUSY:    return bsp::Error::Busy;
    case HAL_TIMEOUT: return bsp::Error::Timeout;
    default:          return bsp::Error::HardwareFailure;
    }
}

bsp::Error Stm32I2c::poll_read_result() noexcept
{
    switch (read_state_.load()) {
    case ReadState::Active:
        return bsp::Error::Busy;
    case ReadState::Complete:
        read_state_.store(ReadState::Idle);
        return bsp::Error::Ok;
    case ReadState::Failed:
        read_state_.store(ReadState::Idle);
        return bsp::Error::HardwareFailure;
    case ReadState::Idle:
        return bsp::Error::InvalidArgument;
    }
    return bsp::Error::HardwareFailure;
}

void Stm32I2c::on_read_complete(I2C_HandleTypeDef* handle) noexcept
{
    if (handle == &handle_) {
        auto expected = ReadState::Active;
        read_state_.compare_exchange_strong(expected, ReadState::Complete);
    }
}

void Stm32I2c::on_read_error(I2C_HandleTypeDef* handle) noexcept
{
    if (handle == &handle_) {
        auto expected = ReadState::Active;
        read_state_.compare_exchange_strong(expected, ReadState::Failed);
    }
}

} // namespace port
