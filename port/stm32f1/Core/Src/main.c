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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "gimbal_observe.h"
#include "gimbal_status.h"
#include "main.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "module/gyro_bias.h"
#include "module/attitude.h"
#include "gimbal_control.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    uint32_t state;
    uint32_t enabled;
    uint32_t ready;
    int32_t feedback_mdeg;
    uint32_t pulse_us;
    uint32_t last_fault;
} AxisReport;

typedef struct {
    uint32_t count;
    uint32_t errors;
    uint32_t dt_min;
    uint32_t dt_max;
    uint32_t dt_bad_total;
    uint32_t dt_last_bad;
    uint32_t dt_bad_at_sample;
    uint32_t servo_state;
    uint32_t servo_pulse;
    uint32_t missed;
    uint32_t calibration_ready;
    uint32_t calibration_count;
    uint32_t attitude_status;
    int32_t angle_mdeg[3];
    int32_t bias[3];
    int32_t rate_mdeg[3];
    int32_t accel[3];
    int32_t gyro[3];
    AxisReport pitch_axis;
    AxisReport yaw_axis;
    uint32_t correction_gain_milli;
    uint32_t filter_status;
    uint32_t filter_count;
    uint32_t device_sample_ms;
    int32_t pitch_target_mdeg;
    uint32_t pitch_step_status;
    uint32_t pitch_target_apply_count;
    uint32_t pitch_target_apply_ms;
} ImuReport;

typedef struct {
    uint32_t sample_ms;
    int16_t angle_cdeg[3];
    uint16_t sequence;
    uint8_t flags;
} FastReport;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Pitch axis calibration, measured on the assembled gimbal:
   - 1500 us is the level position. The mechanical lower stop sits just
     below it, so never command less than GIMBAL_PITCH_ZERO_US.
   - 2500 us lifts the platform 84 deg; this is the conservative limit
     (the servo's nominal range ends here).
   - Effective gain above the ~100 us linkage slack: ~0.093 deg/us.
   - Servo dead band: ~20 us (~1.9 deg). */
#define GIMBAL_PITCH_ZERO_US  1500U
#define GIMBAL_PITCH_MAX_US   2500U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
extern UART_HandleTypeDef huart1;

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2;

volatile uint8_t imu_id = 0xFFU;
volatile uint8_t imu_power = 0xFFU;
volatile uint32_t imu_init_status = HAL_ERROR;
volatile uint32_t imu_read_status = HAL_ERROR;
volatile uint32_t imu_sample_count = 0U;
volatile uint32_t imu_error_count = 0U;

volatile int32_t imu_ax, imu_ay, imu_az;
volatile int32_t imu_gx, imu_gy, imu_gz;

volatile uint32_t gimbal_heartbeat_count = 0U;
volatile uint32_t gimbal_tick_hz = 0U;

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

volatile uint32_t servo_test_allowed = 0U;
volatile uint32_t servo_test_state = GIMBAL_AXIS_IDLE;
volatile uint32_t servo_test_pulse = 1500U;

static osSemaphoreId_t sample_semaphore;
static QueueHandle_t report_queue;
static QueueHandle_t fast_queue;
volatile uint32_t fast_drop_count = 0U;
volatile uint32_t fast_sent_count = 0U;

static GyroBias gyro_calibration;

static Attitude attitude_state = {
    .correction_gain = 2.0F
};

volatile uint32_t sample_trigger_overflow = 0U;
volatile uint32_t report_drop_count = 0U;
/* Temporary fault injection: 0=none, 1=inhibit, 2=invalid dt. */
volatile uint32_t gimbal_test_fault = 0U;
volatile uint32_t imu_last_i2c_error = 0U;
/* Corrected gyro mean, unit: 0.001 degree/second. */
volatile int32_t gyro_mean_mdps[3] = {0, 0, 0};
volatile uint32_t gyro_mean_window_count = 0U;

volatile uint32_t gyro_recalibrate_request = 0U;
volatile uint32_t gyro_recalibration_status = 0U;

extern volatile uint32_t pitch_test_state;
extern volatile uint32_t pitch_zero_ready;
extern volatile uint32_t yaw_reference_ready;

static uint32_t gyro_mean_restart = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
void run_f103_servo_demo(UART_HandleTypeDef* uart_handle,
                         TIM_HandleTypeDef* timer_handle);
void gimbal_sample_timer_irq(void);
static void GimbalTelemetryTask(void *argument);
void gimbal_servo_test_task(void *argument);
int gimbal_pitch_init(void);
void gimbal_pitch_tick(void *argument,
                       float roll_deg,
                       float dt_seconds);
void gimbal_yaw_tick(void *argument,
                     float yaw_deg,
                     float dt_seconds);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* MPU6050 的原始数据是 16 位有符号、大端 */
static int32_t imu_decode_be(const uint8_t *bytes)
{
    uint32_t value = ((uint32_t)bytes[0] << 8) | bytes[1];

    return value >= 32768U
        ? (int32_t)value - 65536
        : (int32_t)value;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 63;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 63;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void gimbal_sample_timer_irq(void)
{
    if (sample_semaphore != NULL) {
        if (osSemaphoreRelease(sample_semaphore) != osOK) {
            ++sample_trigger_overflow;
        }
    }
}

static void telemetry_send(uint8_t *data, uint16_t length)
{
    HAL_StatusTypeDef tx = HAL_BUSY;
    for (unsigned attempt = 0U; attempt < 5U; ++attempt) {
        tx = HAL_UART_Transmit(&huart2, data, length, 1000U);
        if (tx != HAL_BUSY) {
            break;
        }
        (void)osDelay(5U);
    }
    if (tx != HAL_OK) {
        Error_Handler();
    }
}

static void fast_put_le(uint8_t *out, uint32_t value,
                        unsigned width)
{
    for (unsigned i = 0U; i < width; ++i) {
        out[i] = (uint8_t)(value >> (8U * i));
    }
}

static void telemetry_send_fast(const FastReport *frame)
{
    uint8_t packet[18] = {0xA5U, 0x5AU, 1U};
    packet[3] = frame->flags;
    fast_put_le(packet + 4, frame->sequence, 2U);
    fast_put_le(packet + 6, frame->sample_ms, 4U);

    for (unsigned i = 0U; i < 3U; ++i) {
        fast_put_le(packet + 10U + i * 2U,
                    (uint16_t)frame->angle_cdeg[i], 2U);
    }

    uint16_t crc = 0xFFFFU;
    for (unsigned i = 0U; i < 16U; ++i) {
        crc ^= (uint16_t)((uint16_t)packet[i] << 8);
        for (unsigned bit = 0U; bit < 8U; ++bit) {
            if ((crc & 0x8000U) != 0U) {
                crc = (uint16_t)((crc << 1) ^ 0x1021U);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }

    fast_put_le(packet + 16, crc, 2U);
    telemetry_send(packet, sizeof(packet));
    ++fast_sent_count;
}

static void GimbalTelemetryTask(void *argument)
{
    (void)argument;
    ImuReport report;
    FastReport fast;
    char line[512];

    for (;;) {
        if (xQueueReceive(fast_queue, &fast,
                          portMAX_DELAY) != pdPASS) {
            Error_Handler();
        }
        telemetry_send_fast(&fast);

        if (xQueueReceive(report_queue, &report, 0U) != pdPASS) {
            continue;
        }

        int length = snprintf(
            line, sizeof(line),
            "N=%lu T=%lu ERR=%lu DT=%lu..%lu MISS=%lu "
            "BAD=%lu LAST=%lu AT=%lu "
            "SV=%lu US=%lu CAL=%lu ATT=%lu "
            "RPY=%ld,%ld,%ld "
            "P=%lu,%lu,%lu,%ld,%lu,%lu "
            "Y=%lu,%lu,%lu,%ld,%lu,%lu "
            "K=%lu KS=%lu,%lu PT=%ld PS=%lu,%lu,%lu\r\n",
            (unsigned long)report.count,
            (unsigned long)report.device_sample_ms,
            (unsigned long)report.errors,
            (unsigned long)report.dt_min,
            (unsigned long)report.dt_max,
            (unsigned long)report.missed,
            (unsigned long)report.dt_bad_total,
            (unsigned long)report.dt_last_bad,
            (unsigned long)report.dt_bad_at_sample,
            (unsigned long)report.servo_state,
            (unsigned long)report.servo_pulse,
            (unsigned long)report.calibration_ready,
            (unsigned long)report.attitude_status,
            (long)report.angle_mdeg[0],
            (long)report.angle_mdeg[1],
            (long)report.angle_mdeg[2],
            (unsigned long)report.pitch_axis.state,
            (unsigned long)report.pitch_axis.enabled,
            (unsigned long)report.pitch_axis.ready,
            (long)report.pitch_axis.feedback_mdeg,
            (unsigned long)report.pitch_axis.pulse_us,
            (unsigned long)report.pitch_axis.last_fault,
            (unsigned long)report.yaw_axis.state,
            (unsigned long)report.yaw_axis.enabled,
            (unsigned long)report.yaw_axis.ready,
            (long)report.yaw_axis.feedback_mdeg,
            (unsigned long)report.yaw_axis.pulse_us,
            (unsigned long)report.yaw_axis.last_fault,
            (unsigned long)report.correction_gain_milli,
            (unsigned long)report.filter_status,
            (unsigned long)report.filter_count,
            (long)report.pitch_target_mdeg,
            (unsigned long)report.pitch_step_status,
            (unsigned long)report.pitch_target_apply_count,
            (unsigned long)report.pitch_target_apply_ms);

        if (length <= 0 || length >= (int)sizeof(line)) {
            Error_Handler();
        }
        telemetry_send((uint8_t *)line, (uint16_t)length);
    }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
    (void)argument;
    const uint16_t address = 0x68U << 1;
    uint8_t id = 0xFFU;
    uint8_t power = 0xFFU;
    uint8_t wake = 0x00U;
    uint8_t raw[14];
    uint32_t report_divider = 0U;
    uint32_t sample_time_ms = 0U;
    uint32_t fast_divider = 0U;
    uint16_t fast_sequence = 0U;
    uint32_t previous_tick = 0U;
    uint32_t dt_min = UINT32_MAX;
    uint32_t dt_max = 0U;
    uint32_t dt_bad_total = 0U;
    uint32_t dt_last_bad = 0U;
    uint32_t dt_bad_at_sample = 0U;
    uint8_t have_previous_tick = 0U;
    const uint32_t imu_warmup_start_tick =
        osKernelGetTickCount();

    gimbal_tick_hz = osKernelGetTickFreq();
    if (gimbal_tick_hz != 1000U) {
        Error_Handler();
    }

    osDelay(100U);
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(
        &hi2c1, address, 0x75U, I2C_MEMADD_SIZE_8BIT,
        &id, 1U, 20U);
    imu_id = id;

    if (status == HAL_OK && (id == 0x68U || id == 0x74U)) {
        status = HAL_I2C_Mem_Write(
            &hi2c1, address, 0x6BU, I2C_MEMADD_SIZE_8BIT,
            &wake, 1U, 20U);
        if (status == HAL_OK) {
            osDelay(100U);
            status = HAL_I2C_Mem_Read(
                &hi2c1, address, 0x6BU, I2C_MEMADD_SIZE_8BIT,
                &power, 1U, 20U);
        }
    } else {
        status = HAL_ERROR;
    }

    imu_power = power;
    imu_init_status = status;
    const uint8_t ready =
        status == HAL_OK && (power & 0x40U) == 0U;

    if (!ready) {
        Error_Handler();
    }

    const uint8_t registers[] = {0x1BU, 0x1CU, 0x6CU};

    for (unsigned i = 0U; i < sizeof(registers); ++i) {
        uint8_t value = 0x00U;
        uint8_t readback = 0xFFU;

        if (HAL_I2C_Mem_Write(
                &hi2c1, address, registers[i],
                I2C_MEMADD_SIZE_8BIT, &value, 1U, 20U) != HAL_OK) {
            Error_Handler();
        }

        if (HAL_I2C_Mem_Read(
                &hi2c1, address, registers[i],
                I2C_MEMADD_SIZE_8BIT, &readback, 1U, 20U) != HAL_OK
            || readback != value) {
            Error_Handler();
        }
    }

    osDelay(100U);

    sample_semaphore = osSemaphoreNew(1U, 0U, NULL);
    report_queue = xQueueCreate(2U, sizeof(ImuReport));
    fast_queue = xQueueCreate(16U, sizeof(FastReport));

    if (sample_semaphore == NULL || report_queue == NULL ||
        fast_queue == NULL) {
        Error_Handler();
    }

    const osThreadAttr_t telemetry_attributes = {
        .name = "gimbal_telemetry",
        .stack_size = 1792U,
        .priority = osPriorityLow
    };

    if (osThreadNew(GimbalTelemetryTask, NULL,
                    &telemetry_attributes) == NULL) {
        Error_Handler();
    }

    if (!gimbal_pitch_init()) {
        Error_Handler();
    }

    const osThreadAttr_t servo_test_attributes = {
        .name = "servo_test",
        .stack_size = 1024U,
        /* Must stay below the telemetry task: HAL_UART_Receive busy-waits
           and would otherwise starve it. */
        .priority = osPriorityIdle
    };

    if (osThreadNew(gimbal_servo_test_task, &htim3,
                    &servo_test_attributes) == NULL) {
        Error_Handler();
    }

    __HAL_TIM_SET_COUNTER(&htim2, 0U);
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);

    if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK) {
        Error_Handler();
    }

    for (;;) {
        if (osSemaphoreAcquire(
                sample_semaphore, osWaitForever) != osOK) {
            Error_Handler();
        }

        uint32_t now = osKernelGetTickCount();
        const uint32_t elapsed_ticks =
            have_previous_tick ? now - previous_tick : 10U;
        if (have_previous_tick) {
            uint32_t dt = now - previous_tick;

            if (dt < dt_min) dt_min = dt;
            if (dt > dt_max) dt_max = dt;

            if (dt != 10U) {
                ++dt_bad_total;
                dt_last_bad = dt;
                dt_bad_at_sample = imu_sample_count + 1U;
            }
        }
        previous_tick = now;
        have_previous_tick = 1U;

        if (gyro_recalibrate_request) {
            gyro_recalibrate_request = 0U;

            if (pitch_test_state == GIMBAL_AXIS_RUNNING ||
                servo_test_state == GIMBAL_AXIS_RUNNING) {
                gyro_recalibration_status = 2U;
            } else if ((uint32_t)(
                    osKernelGetTickCount() - imu_warmup_start_tick)
                    < 30U * gimbal_tick_hz) {
                gyro_recalibration_status = 3U;
            } else {
                gyro_bias_reset(&gyro_calibration);
                const float preserved_gain =
                    attitude_state.correction_gain;

                attitude_state =
                    (Attitude){.correction_gain = preserved_gain};

                pitch_zero_ready = 0U;
                yaw_reference_ready = 0U;
                gyro_mean_restart = 1U;
                gyro_mean_window_count = 0U;
                gyro_mean_mdps[0] = 0;
                gyro_mean_mdps[1] = 0;
                gyro_mean_mdps[2] = 0;

                gyro_recalibration_status = 4U;
            }
        }

        uint32_t requested_gain_milli = 0U;
        const int parameter_rx =
            gimbal_filter_take_request(&requested_gain_milli);

        if (parameter_rx < 0) {
            filter_apply_status = GIMBAL_PARAM_TRANSPORT_ERROR;
        } else if (parameter_rx > 0) {
            if (requested_gain_milli > 20000U) {
                filter_apply_status = GIMBAL_PARAM_OUT_OF_RANGE;
            } else if (!gyro_calibration.ready ||
                       attitude_state.status != 1U) {
                filter_apply_status = GIMBAL_PARAM_NOT_READY;
            } else {
                attitude_state.correction_gain =
                    requested_gain_milli / 1000.0F;
                ++filter_apply_count;
                filter_apply_status = GIMBAL_PARAM_APPLIED;
            }
        }

        uint16_t sample_address = address;

#if GIMBAL_ENABLE_FAULT_INJECTION
        if (gimbal_test_fault == 3U) {
            sample_address = 0x67U << 1;
        }
#endif

        const uint32_t read_started_ms = osKernelGetTickCount();

        imu_read_status = HAL_I2C_Mem_Read(
            &hi2c1, sample_address, 0x3BU, I2C_MEMADD_SIZE_8BIT,
            raw, sizeof(raw), 20U);

        if (imu_read_status == HAL_OK) {
            sample_time_ms = read_started_ms;

            imu_ax = imu_decode_be(raw);
            imu_ay = imu_decode_be(raw + 2);
            imu_az = imu_decode_be(raw + 4);
            imu_gx = imu_decode_be(raw + 8);
            imu_gy = imu_decode_be(raw + 10);
            imu_gz = imu_decode_be(raw + 12);
            ++imu_sample_count;

            const int32_t accel[3] = {imu_ax, imu_ay, imu_az};
            const int32_t gyro[3] = {imu_gx, imu_gy, imu_gz};

            gyro_bias_update(&gyro_calibration, accel, gyro);

            if (gyro_calibration.ready) {
                const float accel_g[3] = {
                    imu_ax / 16384.0F,
                    imu_ay / 16384.0F,
                    imu_az / 16384.0F
                };
                const float gyro_dps[3] = {
                    (imu_gx - gyro_calibration.mean[0]) / 131.0F,
                    (imu_gy - gyro_calibration.mean[1]) / 131.0F,
                    (imu_gz - gyro_calibration.mean[2]) / 131.0F
                };

                static float gyro_sum_dps[3] = {0};
                static uint32_t gyro_mean_samples = 0U;

                if (gyro_mean_restart) {
                    gyro_sum_dps[0] = 0.0F;
                    gyro_sum_dps[1] = 0.0F;
                    gyro_sum_dps[2] = 0.0F;
                    gyro_mean_samples = 0U;
                    gyro_mean_restart = 0U;
                }

                for (unsigned axis = 0U; axis < 3U; ++axis) {
                    gyro_sum_dps[axis] += gyro_dps[axis];
                }

                if (++gyro_mean_samples >= 1000U) {
                    for (unsigned axis = 0U; axis < 3U; ++axis) {
                        /* 1000 samples: mean * 1000 = sum. */
                        gyro_mean_mdps[axis] =
                            (int32_t)gyro_sum_dps[axis];
                        gyro_sum_dps[axis] = 0.0F;
                    }
                    gyro_mean_samples = 0U;
                    ++gyro_mean_window_count;
                }

                attitude_update(&attitude_state, accel_g, gyro_dps,
                                elapsed_ticks / (float)gimbal_tick_hz);
            }
        } else {
            ++imu_error_count;
            imu_last_i2c_error = hi2c1.ErrorCode;

            if (!gyro_calibration.ready) {
                gyro_bias_reset(&gyro_calibration);
            }

            if (attitude_state.status == 1U) {
                attitude_state.status = 2U;
            }
        }

        if (gyro_recalibration_status == 4U &&
            gyro_calibration.ready) {
            gyro_recalibration_status = 5U;
        }

        servo_test_allowed =
            gyro_calibration.ready &&
            attitude_state.status == 1U &&
            imu_read_status == HAL_OK &&
            sample_trigger_overflow == 0U;

#if GIMBAL_ENABLE_FAULT_INJECTION
        servo_test_allowed =
            servo_test_allowed && gimbal_test_fault != 1U;
#endif

        /* P-axis PWM has one owner: this 100 Hz sampling task. */
        float control_dt =
            elapsed_ticks / (float)gimbal_tick_hz;

#if GIMBAL_ENABLE_FAULT_INJECTION
        if (gimbal_test_fault == 2U) {
            control_dt = 0.030F;
        }
#endif

        gimbal_pitch_tick(
            &htim3, attitude_state.roll, control_dt);
        gimbal_yaw_tick(
            &htim3, attitude_state.yaw, control_dt);

        if (++fast_divider >= 5U) {
            fast_divider = 0U;

            if (imu_read_status == HAL_OK) {
                const uint8_t valid =
                    gyro_calibration.ready != 0U &&
                    attitude_state.status == 1U &&
                    attitude_state.roll >= -180.0F &&
                    attitude_state.roll <= 180.0F &&
                    attitude_state.pitch >= -90.0F &&
                    attitude_state.pitch <= 90.0F &&
                    attitude_state.yaw >= -180.0F &&
                    attitude_state.yaw <= 180.0F;

                FastReport fast = {
                    .sample_ms = sample_time_ms,
                    .angle_cdeg = {0, 0, 0},
                    .sequence = fast_sequence++,
                    .flags = (uint8_t)(
                        (gyro_calibration.ready ? 1U : 0U) |
                        (valid ? 2U : 0U))
                };

                if (valid) {
                    fast.angle_cdeg[0] =
                        (int16_t)(attitude_state.roll * 100.0F);
                    fast.angle_cdeg[1] =
                        (int16_t)(attitude_state.pitch * 100.0F);
                    fast.angle_cdeg[2] =
                        (int16_t)(attitude_state.yaw * 100.0F);
                }

                if (xQueueSend(fast_queue, &fast, 0U) != pdPASS) {
                    ++fast_drop_count;
                }
            }
        }

        if (++report_divider >= 100U) {
            report_divider = 0U;
            ImuReport report = {
                .count = imu_sample_count,
                .errors = imu_error_count,
                .dt_min = dt_min,
                .dt_max = dt_max,
                .dt_bad_total = dt_bad_total,
                .dt_last_bad = dt_last_bad,
                .dt_bad_at_sample = dt_bad_at_sample,
                .servo_state = servo_test_state,
                .servo_pulse = servo_test_pulse,
                .missed = sample_trigger_overflow,
                .calibration_ready = gyro_calibration.ready,
                .calibration_count = gyro_calibration.count,
                .attitude_status = attitude_state.status,
                .angle_mdeg = {
                    (int32_t)(attitude_state.roll * 1000.0F),
                    (int32_t)(attitude_state.pitch * 1000.0F),
                    (int32_t)(attitude_state.yaw * 1000.0F)
                },
                .bias = {
                    (int32_t)gyro_calibration.mean[0],
                    (int32_t)gyro_calibration.mean[1],
                    (int32_t)gyro_calibration.mean[2]
                },
                .rate_mdeg = {
                    gyro_bias_rate_mdeg(&gyro_calibration, 0U, imu_gx),
                    gyro_bias_rate_mdeg(&gyro_calibration, 1U, imu_gy),
                    gyro_bias_rate_mdeg(&gyro_calibration, 2U, imu_gz)
                },
                .accel = {imu_ax, imu_ay, imu_az},
                .gyro = {imu_gx, imu_gy, imu_gz},
                .pitch_axis = {
                    .state = pitch_test_state,
                    .enabled = pitch_closed_loop_enabled,
                    .ready = pitch_zero_ready,
                    .feedback_mdeg = pitch_feedback_mdeg,
                    .pulse_us = pitch_test_pulse,
                    .last_fault = pitch_last_fault
                },
                .yaw_axis = {
                    .state = servo_test_state,
                    .enabled = yaw_closed_loop_enabled,
                    .ready = yaw_reference_ready,
                    .feedback_mdeg = yaw_feedback_mdeg,
                    .pulse_us = servo_test_pulse,
                    .last_fault = yaw_last_fault
                },
                .correction_gain_milli =
                    attitude_state.correction_gain >= 0.0F &&
                    attitude_state.correction_gain <= 20.0F
                    ? (uint32_t)(
                        attitude_state.correction_gain * 1000.0F
                        + 0.5F)
                    : UINT32_MAX,
                .filter_status = filter_apply_status,
                .filter_count = filter_apply_count,
                .device_sample_ms = sample_time_ms,
                .pitch_target_mdeg = pitch_target_mdeg,
                .pitch_step_status = pitch_step_status,
                .pitch_target_apply_count =
                    pitch_target_apply_count,
                .pitch_target_apply_ms = pitch_target_apply_ms
            };

            if (xQueueSend(report_queue, &report, 0U) != pdPASS) {
                ++report_drop_count;
            }

            dt_min = UINT32_MAX;
            dt_max = 0U;
        }
    }
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim->Instance == TIM2)
  {
    gimbal_sample_timer_irq();
  }

  /* USER CODE END Callback 1 */
}

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
