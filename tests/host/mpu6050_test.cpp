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
    unsigned calls{0};

    bsp::Error read_register(std::uint8_t address_7bit,
                             std::uint8_t reg,
                             std::uint8_t* data,
                             std::size_t size,
                             std::uint32_t timeout_ms) noexcept override
    {
        ++calls;
        seen_address = address_7bit;
        seen_reg = reg;
        seen_size = size;
        assert(address_7bit == 0x68U);
        assert(data != nullptr);
        assert(timeout_ms == 100U);

        if (reg == 0x75U) {
            assert(size == 1U);
            if (result == bsp::Error::Ok) {
                *data = value;
            }
            return result;
        }

        constexpr std::uint8_t raw_bytes[] = {
            0x12, 0x34, 0xFF, 0xFE, 0x80, 0x00, 0x01, 0x54,
            0x7F, 0xFF, 0xFF, 0xFF, 0x00, 0x02
        };
        assert(reg == 0x3BU);
        assert(size == sizeof(raw_bytes));
        if (result == bsp::Error::Ok) {
            for (std::size_t i = 0; i < sizeof(raw_bytes); ++i) {
                data[i] = raw_bytes[i];
            }
        }
        return result;
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

    device::Mpu6050RawSample sample{};
    const auto reads_before = bus.calls;
    assert(imu.read_raw(sample, 100U) == bsp::Error::NotInitialized);
    assert(bus.calls == reads_before);

    bus.write_result = bsp::Error::Ok;
    assert(imu.wake_up(100U) == bsp::Error::Ok);
    assert(imu.read_raw(sample, 100U) == bsp::Error::Ok);
    assert(sample.accel_x == 0x1234 && sample.accel_y == -2);
    assert(sample.accel_z == -32768 && sample.temperature == 340);
    assert(sample.gyro_x == 32767 && sample.gyro_y == -1 && sample.gyro_z == 2);

    sample.accel_x = 42;
    bus.result = bsp::Error::Timeout;
    assert(imu.read_raw(sample, 100U) == bsp::Error::Timeout);
    assert(sample.accel_x == 42 && sample.accel_y == -2);
    assert(sample.accel_z == -32768 && sample.temperature == 340);
    assert(sample.gyro_x == 32767 && sample.gyro_y == -1 && sample.gyro_z == 2);

    std::cout << "MPU6050 HOST TEST: PASS\n";
    return 0;
}
