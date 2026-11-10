#include <cassert>
#include <cstdint>
#include <iostream>
#include "stm32_uart.hpp"

extern HAL_StatusTypeDef mock_tx_status;
extern int mock_tx_calls;
extern HAL_StatusTypeDef mock_rx_status;
extern int mock_rx_calls;

int main()
{
    UART_HandleTypeDef handle{};
    port::Stm32Uart uart(handle);
    std::uint8_t data[] = {0x12, 0x34};

    // Test 1: invalid argument
    assert(uart.transmit(nullptr, 2, 100)
           == bsp::Error::InvalidArgument);
    assert(mock_tx_calls == 0);

    // Test 2: not initialized
    assert(uart.transmit(data, 2, 100)
           == bsp::Error::NotInitialized);
    assert(mock_tx_calls == 0);

    // Test 3: normal transmission
    handle.gState = HAL_UART_STATE_READY;
    mock_tx_status = HAL_OK;
    assert(uart.transmit(data, 2, 100)
           == bsp::Error::Ok);
    assert(mock_tx_calls == 1);

    // Test 4: timeout
    mock_tx_status = HAL_TIMEOUT;
    assert(uart.transmit(data, 2, 100)
           == bsp::Error::Timeout);
    assert(mock_tx_calls == 2);

        // Test 5: transmit busy
    mock_tx_status = HAL_BUSY;
    assert(uart.transmit(data, 2, 100) == bsp::Error::Busy);

    // Test 6: transmit hardware failure
    mock_tx_status = HAL_ERROR;
    assert(uart.transmit(data, 2, 100)
           == bsp::Error::HardwareFailure);

    // Test 7: zero-length transmit
    assert(uart.transmit(data, 0, 100)
           == bsp::Error::InvalidArgument);

    // Test 8: oversized transmit
    assert(uart.transmit(data, 65536, 100)
           == bsp::Error::InvalidArgument);

    // Test 9: invalid receive buffer
    assert(uart.receive(nullptr, 2, 100)
           == bsp::Error::InvalidArgument);

    // Test 10: receive not initialized
    assert(uart.receive(data, 2, 100)
           == bsp::Error::NotInitialized);

    // Test 11: receive success
    handle.RxState = HAL_UART_STATE_READY;
    mock_rx_status = HAL_OK;
    assert(uart.receive(data, 2, 100) == bsp::Error::Ok);

    // Test 12: receive timeout
    mock_rx_status = HAL_TIMEOUT;
    assert(uart.receive(data, 2, 100)
           == bsp::Error::Timeout);
               // Test 13: receive busy
    mock_rx_status = HAL_BUSY;
    assert(uart.receive(data, 2, 100)
           == bsp::Error::Busy);

    // Test 14: receive hardware failure
    mock_rx_status = HAL_ERROR;
    assert(uart.receive(data, 2, 100)
           == bsp::Error::HardwareFailure);

    // Test 15: zero-length receive
    const int rx_before = mock_rx_calls;
    assert(uart.receive(data, 0, 100)
           == bsp::Error::InvalidArgument);
    assert(mock_rx_calls == rx_before);

    // Test 16: oversized receive
    assert(uart.receive(data, 65536, 100)
           == bsp::Error::InvalidArgument);
    assert(mock_rx_calls == rx_before);

    // Test 17: invalid TX must not call HAL
    const int tx_before = mock_tx_calls;
    assert(uart.transmit(nullptr, 2, 100)
           == bsp::Error::InvalidArgument);
    assert(mock_tx_calls == tx_before);

    // Test 18: invalid RX must not call HAL
    assert(uart.receive(nullptr, 2, 100)
           == bsp::Error::InvalidArgument);
    assert(mock_rx_calls == rx_before);

    std::cout << "UART HOST TEST: 18 PASS\n";
    return 0;
}