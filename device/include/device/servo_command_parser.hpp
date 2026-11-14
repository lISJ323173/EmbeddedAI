#pragma once

namespace device {

class ServoCommandParser {
public:
    bool push(char byte, unsigned& position) noexcept
    {
        if (byte == '\n') {
            const bool valid = started_ && !invalid_ && digits_ != 0U;
            if (valid) {
                position = value_;
            }
            *this = {};
            return valid;
        }

        if (invalid_) {
            return false;
        }
        if (!started_ && byte == 'S') {
            started_ = true;
        } else if (started_ && !carriage_return_ && byte == '\r') {
            carriage_return_ = true;
        } else if (started_ && !carriage_return_ &&
                   byte >= '0' && byte <= '9' && digits_ < 3U) {
            value_ = value_ * 10U + static_cast<unsigned>(byte - '0');
            ++digits_;
            invalid_ = value_ > 100U;
        } else {
            invalid_ = true;
        }
        return false;
    }

private:
    unsigned value_{};
    unsigned digits_{};
    bool started_{};
    bool carriage_return_{};
    bool invalid_{};
};

} // namespace device
