#include <cassert>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include "device/mpu6050.hpp"

namespace {

class FakeI2c final : public bsp::II2c {
public:
    bsp::Error result{bsp::Error::Ok};
    std::uint8_t value{0x68};
    std::uint8_t seen_address{0};
    std::uint8_t seen_reg{0};
    std::size_t seen_size{0};

    bsp::Error read_register(std::uint8_t address_7bit,
                             std::uint8_t reg,
                             std::uint8_t* data,
                             std::size_t size,
                             std::uint32_t) noexcept override
    {
        seen_address = address_7bit;
        seen_reg = reg;
        seen_size = size;

        if (result != bsp::Error::Ok) {
            return result;
        }
        if (data != nullptr && size > 0U) {
            data[0] = value;
        }
        return bsp::Error::Ok;
    }

    bsp::Error write_result{bsp::Error::Ok};
    unsigned write_calls{0};

    bsp::Error write_register(std::uint8_t address_7bit,
                              std::uint8_t reg,
                              std::uint8_t value,
                              std::uint32_t timeout_ms) noexcept override
    {
        ++write_calls;
        assert(address_7bit == 0x68U);
        assert(reg == 0x6BU);
        assert(value == 0x00U);
        assert(timeout_ms == 100U);
        return write_result;
    }
};

} // namespace

int main()
{
    FakeI2c bus;
    device::Mpu6050 imu(bus, 0x68U);
    std::uint8_t id = 0xFF;

    // Expected MPU6050 identity.
    bus.value = 0x68U;
    assert(imu.probe(id, 100U) == bsp::Error::Ok);
    assert(id == 0x68U);
    assert(bus.seen_address == 0x68U);
    assert(bus.seen_reg == 0x75U);
    assert(bus.seen_size == 1U);

    // Known mismatch: raw ID is retained, probe reports Unsupported.
    bus.value = 0x74U;
    id = 0xFF;
    assert(imu.probe(id, 100U) == bsp::Error::Unsupported);
    assert(id == 0x74U);

    // Bus error is passed through and output is not modified.
    bus.result = bsp::Error::Timeout;
    id = 0xFF;
    assert(imu.probe(id, 100U) == bsp::Error::Timeout);
    assert(id == 0xFFU);

    assert(imu.wake_up(100U) == bsp::Error::Timeout);
    assert(bus.write_calls == 0U);

    bus.result = bsp::Error::Ok;
    bus.value = 0x74U;
    assert(imu.wake_up(100U) == bsp::Error::Unsupported);
    assert(bus.write_calls == 0U);

    bus.value = 0x68U;
    assert(imu.wake_up(100U) == bsp::Error::Ok);
    assert(bus.write_calls == 1U);

    bus.write_result = bsp::Error::Timeout;
    assert(imu.wake_up(100U) == bsp::Error::Timeout);
    assert(bus.write_calls == 2U);

    std::cout << "MPU6050 HOST TEST: PASS\n";
    return 0;
}
