/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <cstdio>
#include "stm32_uart.hpp"
#include "uart_callback_router.hpp"
#include "stm32_can.hpp"
#include "stm32_i2c.hpp"
#include "stm32_pwm.hpp"
#include "device/sg90.hpp"
#include "application/servo_tasks.hpp"
#include "device/mpu6050.hpp"
#include "cmsis_os2.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
extern "C" {
    CAN_HandleTypeDef hcan1;
}

extern "C" {
    I2C_HandleTypeDef hi2c1;
}

TIM_HandleTypeDef htim3;

static IWDG_HandleTypeDef hiwdg{};
static volatile std::uint32_t boot_reset_csr = 0U;

static port::Stm32I2c i2c1_port(hi2c1);

static std::uint8_t i2c_it_value = 0xFF;
static bsp::Error i2c_it_result = bsp::Error::Busy;
static unsigned i2c_it_pass_count = 0;

static std::uint8_t mpu_who_am_i = 0xFF;
static bsp::Error i2c_read_result = bsp::Error::HardwareFailure;
static bsp::Error i2c_bad_address_result = bsp::Error::HardwareFailure;

static std::uint8_t imu_probe_id = 0xFF;
static bsp::Error imu_probe_result = bsp::Error::HardwareFailure;

/* Kept at file scope so the debugger can resolve them at the RX check. */
static bsp::CanFrame can_rx_frame{};

extern "C" {
    DMA_HandleTypeDef hdma_usart1_tx;
    DMA_HandleTypeDef hdma_usart1_rx;
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
static void MX_DMA_Init(void);
static void MX_CAN1_Init(void);
static void MX_I2C1_Init(void);
extern "C" void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim);
static void MX_TIM3_Init(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static volatile bool uart_it_done = false;

static void uart_it_callback(
    void*,
    bsp::UartEvent event,
    bsp::Error result)
{
    if (event == bsp::UartEvent::TxComplete &&
        result == bsp::Error::Ok) {
        uart_it_done = true;
    }
}

static volatile bool uart_rx_done = false;
static volatile bool uart_rx_error = false;

static std::uint8_t uart_rx_buffer[8]{};

static void uart_rx_callback(
    void* context,
    bsp::UartEvent event,
    bsp::Error result)
{
    (void)context;

    if (event == bsp::UartEvent::RxComplete &&
        result == bsp::Error::Ok) {
        uart_rx_done = true;
    }

    if (event == bsp::UartEvent::Error ||
        result != bsp::Error::Ok) {
        uart_rx_error = true;
    }
}

static port::Stm32Can can1_port(hcan1);

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
    can1_port.on_rx0_pending(hcan);
}

extern "C" int _write(int file, char *ptr, int len)
{
    (void)file;

    if (ptr == nullptr || len < 0)
    {
        return -1;
    }

    if (len == 0)
    {
        return 0;
    }

    HAL_StatusTypeDef status = HAL_UART_Transmit(
        &huart1,
        reinterpret_cast<uint8_t *>(ptr),
        static_cast<uint16_t>(len),
        1000
    );

    return (status == HAL_OK) ? len : -1;
}

extern "C" void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef* hi2c)
{
    i2c1_port.on_read_complete(hi2c);
}

extern "C" void HAL_I2C_ErrorCallback(I2C_HandleTypeDef* hi2c)
{
    i2c1_port.on_read_error(hi2c);
}

static constexpr std::uint32_t required_alive =
    application::rx_alive | application::output_alive;
static constexpr bool inject_output_miss = false;

static osEventFlagsId_t servo_alive_events = nullptr;
static volatile std::uint32_t servo_health_windows = 0U;
static volatile std::uint32_t servo_health_missing = 0U;

static void servo_health_task(void*)
{
    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
    hiwdg.Init.Reload = 1499U;
    if (HAL_IWDG_Init(&hiwdg) != HAL_OK) {
        Error_Handler();
    }

    constexpr std::uint32_t window_ticks = 500U;
    auto deadline = osKernelGetTickCount();

    for (;;) {
        deadline += window_ticks;
        if (osDelayUntil(deadline) != osOK) {
            Error_Handler();
        }

        const auto seen = osEventFlagsClear(servo_alive_events, required_alive);
        if ((seen & osFlagsError) != 0U) {
            Error_Handler();
        }

        servo_health_missing = required_alive & ~seen;
        if (servo_health_missing != 0U) {
            for (;;) {
                osDelay(100U);
            }
        }

        if (HAL_IWDG_Refresh(&hiwdg) != HAL_OK) {
            Error_Handler();
        }
        ++servo_health_windows;
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  boot_reset_csr = RCC->CSR;
  __HAL_RCC_CLEAR_RESET_FLAGS();
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();

  /* USER CODE BEGIN USART1_PORT_REGISTER */
static port::Stm32Uart uart1_port(huart1);

if (port::uart_router::register_uart(&huart1, &uart1_port)
    != bsp::Error::Ok)
{
    Error_Handler();
}
/* USER CODE END USART1_PORT_REGISTER */

  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  MX_I2C1_Init();
  HAL_Delay(200);

  bsp::II2c& i2c1 = i2c1_port;

  i2c_read_result = i2c1.read_register(0x68, 0x75, &mpu_who_am_i, 1, 100U);
  if (i2c_read_result != bsp::Error::Ok) {
      Error_Handler();
  }

  std::uint8_t scratch{};
  i2c_bad_address_result = i2c1.read_register(0xD0, 0x75, &scratch, 1, 100U);
  if (i2c_bad_address_result != bsp::Error::InvalidArgument) {
      Error_Handler();
  }

  HAL_NVIC_SetPriority(I2C1_EV_IRQn, 5, 0);
  HAL_NVIC_SetPriority(I2C1_ER_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
  HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);

  bsp::II2cAsync& i2c1_async = i2c1_port;
  i2c_it_pass_count = 0;

  for (unsigned attempt = 0; attempt < 2; ++attempt) {
      i2c_it_value = 0xFF;
      if (i2c1_async.start_read_register(0x68, 0x75, &i2c_it_value, 1U)
          != bsp::Error::Ok) {
          Error_Handler();
      }

      std::uint8_t scratch_byte{};
      if (i2c1_async.start_read_register(0x68, 0x75, &scratch_byte, 1U)
          != bsp::Error::Busy ||
          i2c1.read_register(0x68, 0x75, &scratch_byte, 1U, 100U)
          != bsp::Error::Busy) {
          Error_Handler();
      }

      const auto started_at = HAL_GetTick();
      do {
          i2c_it_result = i2c1_async.poll_read_result();
          if (i2c_it_result == bsp::Error::Busy &&
              HAL_GetTick() - started_at >= 100U) {
              Error_Handler();
          }
      } while (i2c_it_result == bsp::Error::Busy);

      if (i2c_it_result != bsp::Error::Ok || i2c_it_value != mpu_who_am_i) {
          Error_Handler();
      }
      ++i2c_it_pass_count;
  }

  device::Mpu6050 imu(i2c1, 0x68U);
  imu_probe_result = imu.probe(imu_probe_id, 100U);

  // 当前模块的设备层验收：读到已知的身份不匹配值。
  if (imu_probe_result != bsp::Error::Unsupported ||
      imu_probe_id != 0x74U) {
      Error_Handler();
  }

  MX_CAN1_Init();

  /* CAN1 internal loopback: transmit once, read the frame back from FIFO0. */
  {
      CAN_FilterTypeDef can_filter{};
      can_filter.FilterBank = 0;
      can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
      can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
      can_filter.FilterIdHigh = 0x0000U;
      can_filter.FilterIdLow = 0x0000U;
      can_filter.FilterMaskIdHigh = 0x0000U;
      can_filter.FilterMaskIdLow = 0x0000U;
      can_filter.FilterFIFOAssignment = CAN_RX_FIFO0;
      can_filter.FilterActivation = ENABLE;
      can_filter.SlaveStartFilterBank = 14;

      if (HAL_CAN_ConfigFilter(&hcan1, &can_filter) != HAL_OK) {
          Error_Handler();
      }

      if (HAL_CAN_Start(&hcan1) != HAL_OK) {
          Error_Handler();
      }

      bsp::ICan& can1 = can1_port;
bsp::ICanIrqRx& can1_irq = can1_port;
const bsp::CanFrame tx_frame{0x123, 4, {0x11, 0x22, 0x33, 0x44}};

HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);
HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);

for (unsigned attempt = 0; attempt < 2; ++attempt) {
    can_rx_frame = {};
    const auto arm_result = can1_irq.arm_receive_irq();
    if (arm_result != bsp::Error::Ok) {
        Error_Handler();
    }

    const auto tx_result = can1.try_send(tx_frame);
    if (tx_result != bsp::Error::Ok) {
        Error_Handler();
    }

    const auto rx_result = can1_irq.wait_receive_irq(can_rx_frame, 100U);
    if (rx_result != bsp::Error::Ok ||
        can_rx_frame.id != tx_frame.id ||
        can_rx_frame.dlc != tx_frame.dlc) {
        Error_Handler();
    }
    for (unsigned i = 0; i < tx_frame.dlc; ++i) {
        if (can_rx_frame.data[i] != tx_frame.data[i]) {
            Error_Handler();
        }
    }

    if (attempt == 0) {
        bsp::CanFrame empty_frame{};
        if (can1_irq.arm_receive_irq() != bsp::Error::Ok) {
            Error_Handler();
        }
        const auto empty_irq_result =
            can1_irq.wait_receive_irq(empty_frame, 10U);
        if (empty_irq_result != bsp::Error::Timeout) {
            Error_Handler();
        }
    }
}
  }

  /* USART1 DMA TX test (disabled after DMA TX acceptance) */
#if 0
static constexpr std::uint8_t dma_tx_message[] =
    "BSP UART DMA TX OK\r\n";

uart_it_done = false;
uart1_port.set_callback(uart_it_callback, nullptr);

const bsp::Error dma_tx_result = uart1_port.start_transmit(
    dma_tx_message,
    sizeof(dma_tx_message) - 1,
    bsp::UartTransferMode::Dma
);

if (dma_tx_result != bsp::Error::Ok) {
    Error_Handler();
}
#endif

//std::printf("Framework boot OK\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
// 暂停阻塞发送，单独验证 IT 模式
/* BSP UART hardware test */
#if 0
static constexpr std::uint8_t test_message[] =
    "BSP UART TX OK\r\n";

const bsp::Error tx_result = uart1_port.transmit(
    test_message,
    sizeof(test_message) - 1,
    100
);

if (tx_result != bsp::Error::Ok) {
    Error_Handler();
}
#endif
/* USER CODE BEGIN UART_IT_TEST */
#if 0
static constexpr std::uint8_t it_message[] =
    "BSP UART IT TX OK\r\n";

uart_it_done = false;
uart1_port.set_callback(uart_it_callback, nullptr);

const bsp::Error it_result = uart1_port.start_transmit(
    it_message,
    sizeof(it_message) - 1,
    bsp::UartTransferMode::Interrupt
);

if (it_result != bsp::Error::Ok) {
    Error_Handler();
}
#endif
/* USART1 DMA receive test */
#if 0
uart_rx_done = false;
uart_rx_error = false;

uart1_port.set_callback(uart_rx_callback, nullptr);

const bsp::Error rx_start_result = uart1_port.start_receive(
    uart_rx_buffer,
    sizeof(uart_rx_buffer),
    bsp::UartTransferMode::Dma
);

if (rx_start_result != bsp::Error::Ok) {
    Error_Handler();
}

#if 0
// 等待接收错误，回调只更新标志。
while (!uart_rx_error) {
}

const bsp::Error recover_result = uart1_port.recover();
if (recover_result != bsp::Error::Ok) {
    Error_Handler();
}

uart_rx_done = false;
uart_rx_error = false;

const bsp::Error rx_restart_result = uart1_port.start_receive(
    uart_rx_buffer,
    sizeof(uart_rx_buffer),
    bsp::UartTransferMode::Dma
);
if (rx_restart_result != bsp::Error::Ok) {
    Error_Handler();
}
#endif
#endif

/* USER CODE END UART_IT_TEST */
// /* BSP UART RX echo test */
// std::uint8_t rx_buffer[8]{};

// const bsp::Error rx_result = uart1_port.receive(
//     rx_buffer,
//     sizeof(rx_buffer),
//     10000
// );

// if (rx_result == bsp::Error::Ok) {
//     static constexpr std::uint8_t prefix[] =
//         "RX ECHO: ";

//     uart1_port.transmit(prefix, sizeof(prefix) - 1, 100);
//     uart1_port.transmit(rx_buffer, sizeof(rx_buffer), 100);

//     static constexpr std::uint8_t newline[] = "\r\n";
//     uart1_port.transmit(newline, sizeof(newline) - 1, 100);
// }

  MX_TIM3_Init();
  static port::Stm32PwmCh1 pwm_port(htim3);
  bsp::IPwm& pwm = pwm_port;

  if (pwm.start() != bsp::Error::Ok) {
      Error_Handler();
  }

  static device::Sg90 servo(pwm);

  if (servo.set_position_percent(50) != bsp::Error::Ok) {
      Error_Handler();
  }

  if (osKernelInitialize() != osOK) {
      Error_Handler();
  }

  servo_alive_events = osEventFlagsNew(nullptr);
  if (servo_alive_events == nullptr) {
      Error_Handler();
  }

  const auto commands = osMessageQueueNew(4U, sizeof(std::int32_t), nullptr);
  if (commands == nullptr) {
      Error_Handler();
  }

  static application::ServoRxContext rx_context{
      uart1_port, commands, servo_alive_events, Error_Handler
  };
  static application::ServoOutputContext output_context{
      servo, commands, servo_alive_events, Error_Handler, inject_output_miss
  };

  osThreadAttr_t rx_attributes{};
  rx_attributes.name = "servo_rx";
  rx_attributes.priority = osPriorityNormal;
  rx_attributes.stack_size = 1024U;

  osThreadAttr_t output_attributes{};
  output_attributes.name = "servo_output";
  output_attributes.priority = osPriorityAboveNormal;
  output_attributes.stack_size = 1024U;

  if (osThreadNew(application::servo_rx_task, &rx_context, &rx_attributes) == nullptr ||
      osThreadNew(application::servo_output_task, &output_context, &output_attributes) == nullptr) {
      Error_Handler();
  }

  osThreadAttr_t health_attributes{};
  health_attributes.name = "servo_health";
  health_attributes.priority = osPriorityHigh;
  health_attributes.stack_size = 1024U;

  if (osThreadNew(servo_health_task, nullptr, &health_attributes) == nullptr) {
      Error_Handler();
  }

  osKernelStart();
  Error_Handler();
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void MX_DMA_Init(void)
{
    __HAL_RCC_DMA2_CLK_ENABLE();

    HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

    HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);
}

static void MX_CAN1_Init(void)
{
    hcan1.Instance = CAN1;
    hcan1.Init.Prescaler = 6;
    hcan1.Init.Mode = CAN_MODE_LOOPBACK;
    hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
    hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
    hcan1.Init.TimeTriggeredMode = DISABLE;
    hcan1.Init.AutoBusOff = DISABLE;
    hcan1.Init.AutoWakeUp = DISABLE;
    hcan1.Init.AutoRetransmission = DISABLE;
    hcan1.Init.ReceiveFifoLocked = DISABLE;
    hcan1.Init.TransmitFifoPriority = DISABLE;

    if (HAL_CAN_Init(&hcan1) != HAL_OK) {
        Error_Handler();
    }
}

static void MX_I2C1_Init(void)
{
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
        Error_Handler();
    }
}

static void MX_TIM3_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 83;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 19999;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 1500;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    HAL_TIM_MspPostInit(&htim3);
}

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    if (htim->Instance == TIM6) {
        HAL_IncTick();
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
