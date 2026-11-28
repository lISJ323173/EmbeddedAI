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
#include "stm32_uart.hpp"
#include "uart_callback_router.hpp"
#include "stm32_can.hpp"
#include "stm32_i2c.hpp"
#include "stm32_pwm.hpp"
#include "uart_log.hpp"
#include "device/sg90.hpp"
#include "application/servo_tasks.hpp"
#include "device/mpu6050.hpp"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include <cstddef>
#include <cstdio>
#include "../../config/motion_model_data.hpp"
#include "../../config/motion_infer.hpp"
#include "../../config/motion_smoke.hpp"
#include "../../config/motion_live.hpp"
#if defined(MOTION_TEST_REPLAY)
#include "../../config/motion_replay.hpp"
#endif
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* 1 = 打开独立看门狗（原演示行为）；0 = 关闭。
   调试/下载阶段必须为 0：IWDG 约 3 s 超时，会在 Ozone/J-Link 写 Flash 期间
   复位芯片，表现为 "Not all data bytes were successfully downloaded"。 */
#ifndef APP_USE_IWDG
#define APP_USE_IWDG 0
#endif

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

#if APP_USE_IWDG
static IWDG_HandleTypeDef hiwdg{};
#endif
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

/* 文件作用域：送样任务和调试器都能直接看到这个实例。 */
static device::Mpu6050 imu(i2c1_port, 0x68U);

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

/* 调试器 halt 可能把 I2C 从机停在半途，从机拉低 SDA 导致总线一直 BUSY。
   这里用 9 个 SCL 脉冲 + 一个 STOP 让从机释放总线，然后重新初始化 I2C。 */
static void i2c_bus_recover()
{
    HAL_I2C_DeInit(&hi2c1);
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef scl{};
    scl.Pin = GPIO_PIN_6;
    scl.Mode = GPIO_MODE_OUTPUT_OD;
    scl.Pull = GPIO_NOPULL;
    scl.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &scl);

    GPIO_InitTypeDef sda_in{};
    sda_in.Pin = GPIO_PIN_7;
    sda_in.Mode = GPIO_MODE_INPUT;
    sda_in.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &sda_in);

    for (int i = 0; i < 9; ++i) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
        HAL_Delay(1);
    }

    GPIO_InitTypeDef sda_out{};
    sda_out.Pin = GPIO_PIN_7;
    sda_out.Mode = GPIO_MODE_OUTPUT_OD;
    sda_out.Pull = GPIO_NOPULL;
    sda_out.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &sda_out);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
    HAL_Delay(1);

    MX_I2C1_Init();
}

static void servo_health_task(void*)
{
#if APP_USE_IWDG
    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
    hiwdg.Init.Reload = 1499U;
    if (HAL_IWDG_Init(&hiwdg) != HAL_OK) {
        Error_Handler();
    }
#endif

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

#if APP_USE_IWDG
        if (HAL_IWDG_Refresh(&hiwdg) != HAL_OK) {
            Error_Handler();
        }
#endif
        ++servo_health_windows;
    }
}

/* 20 Hz 送样：一行 6 个原始值，串口助手直接可读。 */
/* 送样任务的栈水位，供调试器直接读取。 */
volatile std::uint32_t imu_stack_min_free_bytes = 0xFFFFFFFFU;
volatile std::uint32_t imu_stack_live_checks = 0U;

/* 50 Hz 送样（与 MotionSense 数据集一致）：首列是 HAL_GetTick() 毫秒时间。 */
static void imu_telemetry_task(void*)
{
    std::printf("\r\nIMU,t_ms,ax,ay,az,gx,gy,gz @50Hz\r\n");
    device::Mpu6050RawSample sample{};
    unsigned error_streak = 0U;

    for (;;) {
        const std::uint32_t sampled_at = HAL_GetTick();
        auto status = imu.read_raw(sample, 20U);

        if (status != bsp::Error::Ok) {
            /* 每 400 ms 重试一次初始化：传感器晚接上也能自愈。 */
            const bool retry_now = ((error_streak % 20U) == 0U);
            if (retry_now) {
                (void)imu.wake_up(20U);
                status = imu.read_raw(sample, 20U);
            }
            if (status != bsp::Error::Ok) {
                if (retry_now) {
                    std::printf("IMU,ERR,%d\r\n", static_cast<int>(status));
                }
                ++error_streak;
                osDelay(20U);
                continue;
            }
        }

        error_streak = 0U;
        std::printf("IMU,%lu,%d,%d,%d,%d,%d,%d\r\n",
                    static_cast<unsigned long>(sampled_at),
                    static_cast<int>(sample.accel_x),
                    static_cast<int>(sample.accel_y),
                    static_cast<int>(sample.accel_z),
                    static_cast<int>(sample.gyro_x),
                    static_cast<int>(sample.gyro_y),
                    static_cast<int>(sample.gyro_z));

        /* ---- P4.2：实时窗口推理，每 100 点出一次、之后每 50 点一次 ---- */
        static motion_model::LiveWindow motion_window;
        std::int8_t live_scores[3]{};
        if (motion_window.push(sampled_at,
                               sample.accel_x, sample.accel_y, sample.accel_z,
                               sample.gyro_x, sample.gyro_y, sample.gyro_z, live_scores)) {
            int best = 0;
            for (int i = 1; i < 3; ++i) {
                if (live_scores[i] > live_scores[best]) best = i;
            }
            constexpr const char* labels[] = {"still", "walk", "run"};
            char line[96];
            const int length = std::snprintf(
                line, sizeof(line), "LIVE,%lu,%s,%d,%d,%d\r\n",
                static_cast<unsigned long>(sampled_at), labels[best],
                static_cast<int>(live_scores[0]), static_cast<int>(live_scores[1]),
                static_cast<int>(live_scores[2]));
            if (length > 0 && length < static_cast<int>(sizeof(line))) {
                HAL_UART_Transmit(&huart1, reinterpret_cast<uint8_t*>(line),
                                  static_cast<uint16_t>(length), 100U);
            }
            imu_stack_min_free_bytes = static_cast<std::uint32_t>(
                uxTaskGetStackHighWaterMark(nullptr) * sizeof(StackType_t));
            ++imu_stack_live_checks;
        }

        /* 补偿式延时：扣除本次读写耗时，把周期钉在 20 ms（50 Hz）。 */
        constexpr std::uint32_t period_ms = 20U;
        const std::uint32_t spent = HAL_GetTick() - sampled_at;
        osDelay(spent < period_ms ? (period_ms - spent) : 0U);    }
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
  port::UartLog uart_log(uart1_port);
  bsp::ILog& log = uart_log;
  if (log.write(bsp::LogLevel::Info, "USART1 ready") != bsp::Error::Ok) {
      Error_Handler();
  }

  MX_I2C1_Init();
  HAL_Delay(200);

  /* ---- P0.2a：冷上电读取 WHO_AM_I（只读 0x75，不写传感器配置） ---- */
  {
      const unsigned addresses[] = {0x68U, 0x69U};
      for (const unsigned address : addresses) {
          uint8_t id = 0xFFU;
          const auto status = HAL_I2C_Mem_Read(
              &hi2c1, static_cast<uint16_t>(address << 1U),
              0x75U, I2C_MEMADD_SIZE_8BIT, &id, 1U, 100U);
          const auto error = HAL_I2C_GetError(&hi2c1);

          char line[96];
          const int length = std::snprintf(
              line, sizeof(line),
              "IMU_DIAG addr=0x%02X hal=%u err=0x%08lX id=0x%02X\r\n",
              address, static_cast<unsigned>(status),
              static_cast<unsigned long>(error),
              static_cast<unsigned>(id));
          if (length > 0 && length < static_cast<int>(sizeof(line))) {
              HAL_UART_Transmit(&huart1, reinterpret_cast<uint8_t*>(line),
                                static_cast<uint16_t>(length), 100U);
          }
      }
  }

  bsp::II2c& i2c1 = i2c1_port;

  i2c_read_result = i2c1.read_register(0x68, 0x75, &mpu_who_am_i, 1, 100U);
  if (i2c_read_result != bsp::Error::Ok) {
      i2c_bus_recover();
      i2c_read_result = i2c1.read_register(0x68, 0x75, &mpu_who_am_i, 1, 100U);
  }

  /* 没有传感器时不停机：跳过一次性自检继续启动。
     送样任务每秒打印一次 IMU,ERR，方便区分"串口不通"和"传感器不通"。 */
  const bool legacy_selftest = (i2c_read_result == bsp::Error::Ok);
  if (!legacy_selftest) {
      (void)log.write(bsp::LogLevel::Warning, "IMU not found, self-test skipped");
  }

  if (legacy_selftest) {
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

  imu_probe_result = imu.probe(imu_probe_id, 100U);

  // 总线读失败才算故障；WHO_AM_I 不是 0x68 只提示（兼容芯片常见 0x70/0x74）。
  if (imu_probe_result != bsp::Error::Ok &&
      imu_probe_result != bsp::Error::Unsupported) {
      Error_Handler();
  }

  {
      char probe_message[48]{};
      (void)std::snprintf(probe_message, sizeof(probe_message),
                          "IMU WHO_AM_I=0x%02X", static_cast<unsigned>(imu_probe_id));
      if (log.write(bsp::LogLevel::Info, probe_message) != bsp::Error::Ok) {
          Error_Handler();
      }
  }

  // 唤醒失败不阻断启动：送样任务会打印 IMU,ERR 方便定位。
  (void)imu.wake_up(100U);

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
  } // end legacy self-test

  /* ---- P1.4：量程改为 ±8 g / ±1000 °/s，并读回校验 ---- */
  {
      (void)imu.wake_up(100U);

      constexpr uint16_t address = 0x68U << 1U;
      constexpr uint16_t config_regs[] = {0x1BU, 0x1CU};
      constexpr uint8_t range_mask = 0x18U;
      constexpr uint8_t range_bits = 0x10U;
      uint8_t readback[2] = {0xFFU, 0xFFU};
      bool ok = true;

      for (unsigned i = 0; i < 2U; ++i) {
          uint8_t config = 0U;
          if (HAL_I2C_Mem_Read(&hi2c1, address, config_regs[i],
                               I2C_MEMADD_SIZE_8BIT, &config, 1U, 100U) != HAL_OK) {
              ok = false;
              break;
          }

          config = static_cast<uint8_t>((config & 0xE7U) | range_bits);
          if (HAL_I2C_Mem_Write(&hi2c1, address, config_regs[i],
                                I2C_MEMADD_SIZE_8BIT, &config, 1U, 100U) != HAL_OK ||
              HAL_I2C_Mem_Read(&hi2c1, address, config_regs[i],
                               I2C_MEMADD_SIZE_8BIT, &readback[i], 1U, 100U) != HAL_OK ||
              (readback[i] & range_mask) != range_bits) {
              ok = false;
              break;
          }
      }

      char line[80];
      const int length = std::snprintf(
          line, sizeof(line), "IMU_RANGE %s gyro=0x%02X accel=0x%02X\r\n",
          ok ? "OK" : "FAIL", static_cast<unsigned>(readback[0]),
          static_cast<unsigned>(readback[1]));
      if (length > 0 && length < static_cast<int>(sizeof(line))) {
          HAL_UART_Transmit(&huart1, reinterpret_cast<uint8_t*>(line),
                            static_cast<uint16_t>(length), 100U);
      }
      if (!ok) {
          (void)log.write(bsp::LogLevel::Warning, "IMU range config failed");
      }
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  /* ---- P3.2：核对固件内的模型参数与 PC 导出的字节一致 ---- */
  {
      using namespace motion_model;
      std::uint32_t fingerprint = 2166136261U;
      std::size_t total = 0;
      const auto mix = [&](const void* data, std::size_t size) {
          const auto* bytes = static_cast<const volatile unsigned char*>(data);
          for (std::size_t i = 0; i < size; ++i) {
              fingerprint = (fingerprint ^ bytes[i]) * 16777619U;
          }
          total += size;
      };

      mix(dense1_weights, sizeof(dense1_weights));
      mix(dense1_bias, sizeof(dense1_bias));
      mix(dense2_weights, sizeof(dense2_weights));
      mix(dense2_bias, sizeof(dense2_bias));

      char line[64];
      const int length = std::snprintf(
          line, sizeof(line), "BOARD_MODEL bytes=%lu hash=%08lX\r\n",
          static_cast<unsigned long>(total),
          static_cast<unsigned long>(fingerprint));
      if (length > 0 && length < static_cast<int>(sizeof(line))) {
          HAL_UART_Transmit(&huart1, reinterpret_cast<uint8_t*>(line),
                            static_cast<uint16_t>(length), 100U);
      }
  }

  /* ---- P4.1：单样本板端推理对拍 ---- */
  {
      std::int8_t scores[3]{};
      motion_model::infer(smoke_input, scores);
      int best = 0;
      int max_diff = 0;
      for (int i = 0; i < 3; ++i) {
          if (scores[i] > scores[best]) {
              best = i;
          }
          const int diff = static_cast<int>(scores[i]) - smoke_expected[i];
          const int magnitude = diff < 0 ? -diff : diff;
          if (magnitude > max_diff) {
              max_diff = magnitude;
          }
      }

      char line[96];
      const int length = std::snprintf(
          line, sizeof(line), "BOARD_INFER scores=[%d,%d,%d] class=%d max_diff=%d\r\n",
          static_cast<int>(scores[0]), static_cast<int>(scores[1]),
          static_cast<int>(scores[2]), best, max_diff);
      if (length > 0 && length < static_cast<int>(sizeof(line))) {
          HAL_UART_Transmit(&huart1, reinterpret_cast<uint8_t*>(line),
                            static_cast<uint16_t>(length), 100U);
      }
  }

  /* ---- P5.1：DWT 周期计数测 infer() 时延 ---- */
  {
      constexpr unsigned runs = 200;
      std::uint32_t min_cycles = 0xFFFFFFFFU;
      std::uint32_t max_cycles = 0;
      std::uint64_t total_cycles = 0;

      CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
      DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

      for (unsigned run = 0; run < runs; ++run) {
          std::int8_t input[8];
          std::int8_t scores[3];
          for (unsigned i = 0; i < 8; ++i) {
              input[i] = smoke_input[i];
          }

          const std::uint32_t start = DWT->CYCCNT;
          __asm__ volatile("" : "+m"(input) : : "memory");
          motion_model::infer(input, scores);
          __asm__ volatile("" : : "m"(scores) : "memory");
          const std::uint32_t cycles = DWT->CYCCNT - start;

          if (cycles < min_cycles) min_cycles = cycles;
          if (cycles > max_cycles) max_cycles = cycles;
          total_cycles += cycles;
      }

      char line[128];
      const int length = std::snprintf(
          line, sizeof(line),
          "BENCH %s n=%u core_hz=%lu min=%lu avg=%lu max=%lu cycles\r\n",
          min_cycles == 0 ? "FAIL" : "OK", runs,
          static_cast<unsigned long>(SystemCoreClock),
          static_cast<unsigned long>(min_cycles),
          static_cast<unsigned long>((total_cycles + runs / 2) / runs),
          static_cast<unsigned long>(max_cycles));
      if (length > 0 && length < static_cast<int>(sizeof(line))) {
          HAL_UART_Transmit(&huart1, reinterpret_cast<uint8_t*>(line),
                            static_cast<uint16_t>(length), 100U);
      }
  }

#if defined(MOTION_TEST_REPLAY)
  {
      constexpr unsigned count = sizeof(motion_replay) / sizeof(motion_replay[0]);
      unsigned correct[3]{};
      unsigned agree = 0;
      double total_error = 0.0;

      for (const auto& item : motion_replay) {
          std::int8_t scores[3]{};
          motion_model::infer(item.input, scores);
          unsigned board_best = 0;
          unsigned pc_best = 0;
          for (unsigned i = 0; i < 3; ++i) {
              if (scores[i] > scores[board_best]) board_best = i;
              if (item.pc[i] > item.pc[pc_best]) pc_best = i;

              const double board_score =
                  (static_cast<int>(scores[i]) - motion_model::dense2_output_zero) *
                  static_cast<double>(motion_model::dense2_output_scale);
              const double error = board_score - item.pc[i];
              total_error += error < 0.0 ? -error : error;
          }
          correct[item.label] += board_best == item.label;
          agree += board_best == pc_best;
      }

      const auto mae_micro = static_cast<unsigned long>(
          total_error * 1000000.0 / (3U * count) + 0.5);
      char line[128];
      const int length = std::snprintf(
          line, sizeof(line),
          "BOARD_TEST n=%u still=%u walk=%u run=%u agree=%u logit_mae_micro=%lu\r\n",
          count, correct[0], correct[1], correct[2], agree, mae_micro);
      if (length > 0 && length < static_cast<int>(sizeof(line))) {
          HAL_UART_Transmit(&huart1, reinterpret_cast<uint8_t*>(line),
                            static_cast<uint16_t>(length), 100U);
      }
  }
#endif

  MX_TIM3_Init();
  static port::Stm32PwmCh1 pwm_port(htim3);  bsp::IPwm& pwm = pwm_port;

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

  osThreadAttr_t imu_attributes{};
  imu_attributes.name = "imu";
  /* 必须高于 servo_rx：后者用 10 ms 阻塞式轮询占着 CPU，低优先级任务会被饿到 130 ms/次。 */
  imu_attributes.priority = osPriorityHigh;
  imu_attributes.stack_size = 2048U;

  if (osThreadNew(imu_telemetry_task, nullptr, &imu_attributes) == nullptr) {
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
