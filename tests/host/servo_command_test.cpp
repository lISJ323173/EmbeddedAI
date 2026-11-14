#include <cassert>
#include "device/servo_command_parser.hpp"

int main()
{
    device::ServoCommandParser parser;
    unsigned position = 999U;

    const auto feed = [&](const char* bytes) {
        unsigned accepted = 0;
        for (; *bytes != '\0'; ++bytes) {
            accepted += parser.push(*bytes, position) ? 1U : 0U;
        }
        return accepted;
    };

    assert(feed("S5") == 0U && position == 999U);
    assert(feed("0\n") == 1U && position == 50U);
    assert(feed("S101\nS-1\nS\n") == 0U && position == 50U);
    assert(feed("badS50\nS1\r2\n") == 0U && position == 50U);
    assert(feed("S0\n") == 1U && position == 0U);
    assert(feed("S100\r\n") == 1U && position == 100U);
}
