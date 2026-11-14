#pragma once

#include <cstddef>
#include <cstdint>
#include "bsp/error.hpp"

namespace bsp {

class II2c {
public:
    virtual ~II2c() = default;

    virtual Error read_register(std::uint8_t address_7bit,
                                std::uint8_t reg,
                                std::uint8_t* data,
                                std::size_t size,
                                std::uint32_t timeout_ms) noexcept = 0;
};

class II2cAsync {
public:
    virtual ~II2cAsync() = default;

    // Ok 表示已启动。缓冲区必须保持有效，直到前台取走完成结果。
    virtual Error start_read_register(std::uint8_t address_7bit,
                                      std::uint8_t reg,
                                      std::uint8_t* data,
                                      std::size_t size) noexcept = 0;

    // Busy 表示仍在传输；Ok 或 HardwareFailure 会取走本次结果。
    // 当前没有已启动的事务时返回 InvalidArgument。
    virtual Error poll_read_result() noexcept = 0;
};

} // namespace bsp
