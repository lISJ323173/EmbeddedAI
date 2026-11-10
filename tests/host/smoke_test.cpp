#include <iostream>
#include "bsp/error.hpp"

int main() {
    constexpr auto error = bsp::Error::Ok;

    static_assert(bsp::is_ok(error),
                  "Error::Ok must mean success");

    std::cout << "C++17 HOST TEST: PASS\n";
    return 0;
}