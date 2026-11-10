#include <cassert>
#include <cstdint>
#include <iostream>
#include "stm32_uart.hpp"

extern HAL_StatusTypeDef mock_tx_status;
extern int mock_tx_calls;
extern HAL_StatusTypeDef mock_rx_status;
extern int mock_rx_calls;
extern HAL_StatusTypeDef mock_async_tx_status;
extern HAL_StatusTypeDef mock_async_rx_status;

extern int mock_tx_it_calls;
extern int mock_tx_dma_calls;
extern int mock_rx_it_calls;
extern int mock_rx_dma_calls;
struct CallbackRecord {
    int tx_complete = 0;
    int rx_complete = 0;
    int errors = 0;
    bsp::Error last_error = bsp::Error::Ok;
};

void uart_test_callback(
    void* context,
    bsp::UartEvent event,
    bsp::Error result)
{
    auto& record = *static_cast<CallbackRecord*>(context);

    record.last_error = result;

    switch (event) {
    case bsp::UartEvent::TxComplete:
        ++record.tx_complete;
        break;
    case bsp::UartEvent::RxComplete:
        ++record.rx_complete;
        break;
    case bsp::UartEvent::Error:
        ++record.errors;
        break;
    }
}

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

        using Mode = bsp::UartTransferMode;
    using Error = bsp::Error;

    // 使用独立句柄，避免影响之前18项测试
    UART_HandleTypeDef async_handle{};
    async_handle.gState = HAL_UART_STATE_READY;
    async_handle.RxState = HAL_UART_STATE_READY;
    port::Stm32Uart async_uart(async_handle);

    // 19: 非法发送参数
    assert(async_uart.start_transmit(nullptr, 2, Mode::Interrupt)
           == Error::InvalidArgument);

    // 20: IT发送启动失败
    mock_async_tx_status = HAL_TIMEOUT;
    assert(async_uart.start_transmit(data, 2, Mode::Interrupt)
           == Error::Timeout);

    // 21: IT发送启动成功
    mock_async_tx_status = HAL_OK;
    assert(async_uart.start_transmit(data, 2, Mode::Interrupt)
           == Error::Ok);
    assert(mock_tx_it_calls == 2);

    // 22: TX重复启动
    assert(async_uart.start_transmit(data, 2, Mode::Dma)
           == Error::Busy);
    assert(mock_tx_dma_calls == 0);

    // 23: DMA接收启动失败
    mock_async_rx_status = HAL_BUSY;
    assert(async_uart.start_receive(data, 2, Mode::Dma)
           == Error::Busy);

    // 24: DMA接收启动成功
    mock_async_rx_status = HAL_OK;
    assert(async_uart.start_receive(data, 2, Mode::Dma)
           == Error::Ok);
    assert(mock_rx_dma_calls == 2);

    // 25: RX重复启动
    assert(async_uart.start_receive(data, 2, Mode::Interrupt)
           == Error::Busy);
    assert(mock_rx_it_calls == 0);

    // 26: TX和RX状态独立
    assert(mock_tx_it_calls == 2 && mock_rx_dma_calls == 2);
    CallbackRecord record{};
    async_uart.set_callback(uart_test_callback, &record);

    // 27: TX完成通知
    async_uart.on_tx_complete();
    assert(record.tx_complete == 1);
    assert(record.last_error == Error::Ok);

    // 28: TX重复完成通知不应重复回调
    async_uart.on_tx_complete();
    assert(record.tx_complete == 1);

    // 29: TX完成后允许重新发送
    mock_async_tx_status = HAL_OK;
    assert(async_uart.start_transmit(data, 2, Mode::Dma)
           == Error::Ok);
    assert(mock_tx_dma_calls == 1);

    // 30: RX完成通知
    async_uart.on_rx_complete();
    assert(record.rx_complete == 1);
    assert(record.last_error == Error::Ok);

    // 31: RX重复完成通知不应重复回调
    async_uart.on_rx_complete();
    assert(record.rx_complete == 1);

    // 32: RX完成后允许重新接收
    mock_async_rx_status = HAL_OK;
    assert(async_uart.start_receive(data, 2, Mode::Interrupt)
           == Error::Ok);
    assert(mock_rx_it_calls == 1);
    // 33: 异步通信错误通知
    async_uart.on_error(Error::HardwareFailure);
    assert(record.errors == 1);
    assert(record.last_error == Error::HardwareFailure);

    // 34: 错误后允许重新发送
    mock_async_tx_status = HAL_OK;
    assert(async_uart.start_transmit(data, 2, Mode::Interrupt)
           == Error::Ok);

    // 35: 错误后允许重新接收
    mock_async_rx_status = HAL_OK;
    assert(async_uart.start_receive(data, 2, Mode::Dma)
           == Error::Ok);

    // 36: 再次发生错误
    async_uart.on_error(Error::Timeout);
    assert(record.errors == 2);
    assert(record.last_error == Error::Timeout);

    // 37: 无活动传输时不重复通知
    async_uart.on_error(Error::HardwareFailure);
    assert(record.errors == 2);

    // 38: 错误恢复后可以继续发送
    assert(async_uart.start_transmit(data, 2, Mode::Dma)
           == Error::Ok);

    std::cout << "UART HOST TEST: 38 PASS\n";
    return 0;
}