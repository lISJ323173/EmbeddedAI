#include "module/crc8.hpp"

constexpr std::uint8_t reference[] = {
    '1', '2', '3', '4', '5', '6', '7', '8', '9'
};

constexpr std::uint8_t corrupted[] = {
    '1', '2', '3', '4', '5', '6', '7', '8', '8'
};

static_assert(module::crc8_smbus(nullptr, 0U) == 0x00U);
static_assert(module::crc8_smbus(reference, sizeof(reference)) == 0xF4U);
static_assert(module::crc8_smbus(corrupted, sizeof(corrupted)) != 0xF4U);

int main() {}
