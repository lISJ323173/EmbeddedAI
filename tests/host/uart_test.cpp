#include <cassert>
#include <cstdint>
#include <iostream>
#include "stm32_uart.hpp"
#include "uart_callback_router.hpp"

extern "C" {

void HAL_UART_TxCpltCallback(
    UART_HandleTypeDef* huart);

void HAL_UART_RxCpltCallback(
    UART_HandleTypeDef* huart);

void HAL_UART_ErrorCallback(
    UART_HandleTypeDef* huart);

}

extern HAL_StatusTypeDef mock_tx_status;
extern int mock_tx_calls;
extern HAL_StatusTypeDef mock_rx_status;
extern int mock_rx_calls;
extern HAL_StatusTypeDef mock_async_tx_status;
extern HAL_StatusTypeDef mock_async_rx_status;

extern bool mock_tx_complete_during_start;
extern bool mock_rx_complete_during_start;

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
    // UART router tests
    namespace router = port::uart_router;

    UART_HandleTypeDef router_handle1{};
    UART_HandleTypeDef router_handle2{};
    UART_HandleTypeDef router_handle3{};
    UART_HandleTypeDef router_handle4{};
    UART_HandleTypeDef router_handle5{};

    port::Stm32Uart uart1(router_handle1);
    port::Stm32Uart uart2(router_handle2);
    port::Stm32Uart uart3(router_handle3);
    port::Stm32Uart uart4(router_handle4);
    port::Stm32Uart uart5(router_handle5);

    // 39: Null handle
    assert(router::register_uart(nullptr, &uart1)
           == Error::InvalidArgument);

    // 40: Null instance
    assert(router::register_uart(&router_handle1, nullptr)
           == Error::InvalidArgument);

    // 41: Successful registration
    assert(router::register_uart(&router_handle1, &uart1)
           == Error::Ok);

    // 42: Duplicate handle
    assert(router::register_uart(&router_handle1, &uart1)
           == Error::Busy);

    // 43: Fill the remaining slots
    assert(router::register_uart(&router_handle2, &uart2) == Error::Ok);
    assert(router::register_uart(&router_handle3, &uart3) == Error::Ok);
    assert(router::register_uart(&router_handle4, &uart4) == Error::Ok);

    // 44: Registry full
    assert(router::register_uart(&router_handle5, &uart5)
           == Error::Busy);

    // 45: Unregister an existing handle
    assert(router::unregister_uart(&router_handle1)
           == Error::Ok);

    // 46: Unregister the same handle again
    assert(router::unregister_uart(&router_handle1)
           == Error::NotInitialized);

    // 47: Reuse the released slot
    assert(router::register_uart(&router_handle5, &uart5)
           == Error::Ok);
               // 48: 未注册句柄不能触发回调
    CallbackRecord router_record{};
    uart1.set_callback(uart_test_callback, &router_record);

    router::on_tx_complete(&router_handle1);
    assert(router_record.tx_complete == 0);

    // 49: 已注册句柄正确转发 TX 完成
    router::on_tx_complete(&router_handle5);
    assert(router_record.tx_complete == 0);

    // 50: 注销后不再转发事件
    assert(router::unregister_uart(&router_handle5)
           == Error::Ok);

    router::on_rx_complete(&router_handle5);
    assert(router_record.rx_complete == 0);
             // 51: 已注册对象接收 TX 完成事件
    CallbackRecord active_record{};
    uart2.set_callback(uart_test_callback, &active_record);

    router_handle2.gState = HAL_UART_STATE_READY;

    mock_async_tx_status = HAL_OK;

    assert(uart2.start_transmit(data, 2, Mode::Interrupt)
           == Error::Ok);

    router::on_tx_complete(&router_handle2);
    assert(active_record.tx_complete == 1);

    // 52: 重复事件不应重复回调
    router::on_tx_complete(&router_handle2);
    assert(active_record.tx_complete == 1);

    // 53: RX 完成事件路由
    router_handle2.RxState = HAL_UART_STATE_READY;

    mock_async_rx_status = HAL_OK;

    assert(uart2.start_receive(data, 2, Mode::Interrupt)
           == Error::Ok);

    router::on_rx_complete(&router_handle2);
    assert(active_record.rx_complete == 1);

    // 54: 错误事件路由
    assert(uart2.start_transmit(data, 2, Mode::Dma)
           == Error::Ok);

    router::on_error(&router_handle2);
    assert(active_record.errors == 1);
    assert(active_record.last_error == Error::HardwareFailure);
         assert(router::unregister_uart(&router_handle2) == Error::Ok);
    assert(router::unregister_uart(&router_handle3) == Error::Ok);
    assert(router::unregister_uart(&router_handle4) == Error::Ok);
    // HAL callback bridge integration tests

    UART_HandleTypeDef bridge_handle{};
    bridge_handle.gState = HAL_UART_STATE_READY;
    bridge_handle.RxState = HAL_UART_STATE_READY;

    port::Stm32Uart bridge_uart(bridge_handle);

    CallbackRecord bridge_record{};

    bridge_uart.set_callback(
        uart_test_callback,
        &bridge_record
    );

    assert(router::register_uart(
        &bridge_handle,
        &bridge_uart
    ) == Error::Ok);

    // 55: HAL TX -> Router -> Stm32Uart -> BSP callback
    mock_async_tx_status = HAL_OK;

    assert(bridge_uart.start_transmit(
        data, 2, Mode::Interrupt
    ) == Error::Ok);

    HAL_UART_TxCpltCallback(&bridge_handle);

    assert(bridge_record.tx_complete == 1);
    assert(bridge_record.last_error == Error::Ok);

    // 56: HAL RX -> Router -> Stm32Uart -> BSP callback
    mock_async_rx_status = HAL_OK;

    assert(bridge_uart.start_receive(
        data, 2, Mode::Interrupt
    ) == Error::Ok);

    HAL_UART_RxCpltCallback(&bridge_handle);

    assert(bridge_record.rx_complete == 1);
    assert(bridge_record.last_error == Error::Ok);

    // 57: HAL Error -> Router -> Stm32Uart -> BSP callback
    assert(bridge_uart.start_transmit(
        data, 2, Mode::Dma
    ) == Error::Ok);

    HAL_UART_ErrorCallback(&bridge_handle);

    assert(bridge_record.errors == 1);
    assert(bridge_record.last_error ==
           Error::HardwareFailure);

    // Clean up the test registration.
    assert(router::unregister_uart(
        &bridge_handle
    ) == Error::Ok);
// 58: Invalid TX transfer mode
UART_HandleTypeDef invalid_mode_handle{};
invalid_mode_handle.gState = HAL_UART_STATE_READY;
invalid_mode_handle.RxState = HAL_UART_STATE_READY;

port::Stm32Uart invalid_mode_uart(invalid_mode_handle);

const auto invalid_mode =
    static_cast<Mode>(255);

assert(invalid_mode_uart.start_transmit(
    data, 2, invalid_mode) == Error::InvalidArgument);

// 59: Invalid RX transfer mode
assert(invalid_mode_uart.start_receive(
    data, 2, invalid_mode) == Error::InvalidArgument);

    // 60–62: TX early-completion fault injection
UART_HandleTypeDef early_handle{};
early_handle.gState = HAL_UART_STATE_READY;

port::Stm32Uart early_uart(early_handle);
CallbackRecord early_record{};

early_uart.set_callback(uart_test_callback, &early_record);

assert(router::register_uart(&early_handle, &early_uart)
       == Error::Ok);

mock_tx_complete_during_start = true;
mock_async_tx_status = HAL_OK;

// 60: Callback before HAL returns success
assert(early_uart.start_transmit(data, 2, Mode::Interrupt)
       == Error::Ok);
assert(early_record.tx_complete == 1);

// 61: Callback before HAL returns failure
mock_async_tx_status = HAL_ERROR;
assert(early_uart.start_transmit(data, 2, Mode::Interrupt)
       == Error::HardwareFailure);
assert(early_record.tx_complete == 1);

// 62: Start again after early completion
mock_async_tx_status = HAL_OK;
assert(early_uart.start_transmit(data, 2, Mode::Interrupt)
       == Error::Ok);
assert(early_record.tx_complete == 2);

mock_tx_complete_during_start = false;

assert(router::unregister_uart(&early_handle)
       == Error::Ok);

       // 63–65: RX early-completion tests
UART_HandleTypeDef early_rx_handle{};
early_rx_handle.RxState = HAL_UART_STATE_READY;

port::Stm32Uart early_rx_uart(early_rx_handle);
CallbackRecord early_rx_record{};

early_rx_uart.set_callback(uart_test_callback, &early_rx_record);

assert(router::register_uart(
    &early_rx_handle, &early_rx_uart) == Error::Ok);

mock_rx_complete_during_start = true;
mock_async_rx_status = HAL_OK;

// 63: RX completes before HAL returns
assert(early_rx_uart.start_receive(
    data, 2, Mode::Interrupt) == Error::Ok);
assert(early_rx_record.rx_complete == 1);

// 64: RX early completion + HAL failure
mock_async_rx_status = HAL_ERROR;
assert(early_rx_uart.start_receive(
    data, 2, Mode::Interrupt) == Error::HardwareFailure);
assert(early_rx_record.rx_complete == 1);

// 65: RX can restart after early completion
mock_async_rx_status = HAL_OK;
assert(early_rx_uart.start_receive(
    data, 2, Mode::Interrupt) == Error::Ok);
assert(early_rx_record.rx_complete == 2);

mock_rx_complete_during_start = false;

assert(router::unregister_uart(
    &early_rx_handle) == Error::Ok);


    std::cout << "UART HOST TEST: 65 PASS\n";
    return 0;
}