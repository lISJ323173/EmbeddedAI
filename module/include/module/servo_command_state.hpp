#pragma once

#include <cstdint>

namespace module {

class ServoCommandState {
public:
    explicit ServoCommandState(std::uint32_t hold_ticks) noexcept
        : hold_ticks_(hold_ticks) {}

    void accept(std::uint32_t now) noexcept
    {
        last_command_tick_ = now;
        state_ = State::Holding;
    }

    bool should_center(std::uint32_t now) noexcept
    {
        if (state_ != State::Holding ||
            now - last_command_tick_ < hold_ticks_) {
            return false;
        }

        state_ = State::Centered;
        return true;
    }

private:
    enum class State { Centered, Holding };

    std::uint32_t hold_ticks_;
    std::uint32_t last_command_tick_{};
    State state_{State::Centered};
};

} // namespace module
