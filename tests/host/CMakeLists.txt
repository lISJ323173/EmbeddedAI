cmake_minimum_required(VERSION 3.22)

project(framework_host_tests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

enable_testing()

add_executable(uart_test
    uart_test.cpp
    mock_hal/mock_uart.cpp
    ../../port/stm32f4/src/stm32_uart.cpp
    ../../port/stm32f4/src/uart_callback_router.cpp
    ../../port/stm32f4/src/uart_hal_callbacks.cpp
)

target_include_directories(uart_test PRIVATE
    mock_hal
    ../../bsp/include
    ../../port/stm32f4/include
)

target_compile_options(uart_test PRIVATE
    -Wall -Wextra -Werror -UNDEBUG
)

add_test(NAME uart_error_tests COMMAND uart_test)
