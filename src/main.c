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
#include <stdbool.h>
#include <stddef.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
///////////////////////////////////////////////////////////////////////
/*
 * 火箭感測器資料快照。
 *
 * 目前還沒有 BMP581、磁力計、IMU 與 ADC 驅動，
 * 所以後面會先填入測試資料。
 */
typedef struct
{
  int32_t pressure_pa;             // 氣壓，單位 Pa
  int16_t temperature_centi_c;     // 溫度，0.01°C
  int16_t mag_x_deci_ut;           // X 軸磁場，0.1 uT
  int16_t mag_y_deci_ut;           // Y 軸磁場，0.1 uT
  int16_t mag_z_deci_ut;           // Z 軸磁場，0.1 uT
  int16_t accel_x_mg;              // X 軸加速度，mg
  int16_t accel_y_mg;              // Y 軸加速度，mg
  int16_t accel_z_mg;              // Z 軸加速度，mg
  uint16_t battery_mv;             // 電池電壓，mV
  uint8_t flight_state;            // 飛行狀態
  uint8_t status_flags;            // 感測器狀態
} SensorSnapshot_t;


/*
 * LoRa 傳送封包。
 *
 * __attribute__((packed)) 表示不讓編譯器在欄位中插入空白，
 * 確保封包固定為 34 bytes。
 */
typedef struct __attribute__((packed))
{
  uint8_t sync_1;                  // 固定 0xA5
  uint8_t sync_2;                  // 固定 0x5A
  uint8_t protocol_version;        // 協定版本
  uint8_t message_type;            // 封包類型

  uint16_t sequence;               // 封包編號
  uint32_t timestamp_ms;           // MCU 啟動後毫秒數

  int32_t pressure_pa;
  int16_t temperature_centi_c;

  int16_t mag_x_deci_ut;
  int16_t mag_y_deci_ut;
  int16_t mag_z_deci_ut;

  int16_t accel_x_mg;
  int16_t accel_y_mg;
  int16_t accel_z_mg;

  uint16_t battery_mv;
  uint8_t flight_state;
  uint8_t status_flags;

  uint16_t crc16;                  // CRC16 校驗
} TelemetryPacket_t;
///////////////////////////////////////////////////////////////////////
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
///////////////////////////////////////////////////////////////////////
#define TELEMETRY_PROTOCOL_VERSION  1U
#define TELEMETRY_MESSAGE_TYPE      1U

/*
 * 感測器每 50 ms 更新一次，也就是 20 Hz。
 *
 * LoRa 每 200 ms 傳一次，也就是 5 Hz。
 * 因為目前 UART 與 E22 都先使用 9600 baud，
 * 不建議直接傳送太高頻率。
 */
#define SENSOR_SAMPLE_PERIOD_MS     50U
#define TELEMETRY_PERIOD_MS         200U

#define LORA_READY_TIMEOUT_MS       1000U

/* 感測器有效旗標 */
#define STATUS_BARO_VALID           (1U << 0)
#define STATUS_MAG_VALID            (1U << 1)
#define STATUS_IMU_VALID            (1U << 2)
#define STATUS_BATTERY_VALID        (1U << 3)

/* 表示目前送的是測試資料 */
#define STATUS_TEST_DATA            (1U << 7)
///////////////////////////////////////////////////////////////////////
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
///////////////////////////////////////////////////////////////////////
static SensorSnapshot_t latest_snapshot;
static uint32_t next_telemetry_ms = 0;
///////////////////////////////////////////////////////////////////////
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
///////////////////////////////////////////////////////////////////////
static bool LORA_WaitReady(uint32_t timeout_ms);
static bool LORA_SetNormalMode(void);

static void Sensors_ReadSnapshot(
    SensorSnapshot_t *sensor);

static void Telemetry_Send(
    const SensorSnapshot_t *sensor);
///////////////////////////////////////////////////////////////////////
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
///////////////////////////////////////////////////////////////////////
/**
 * @brief 等待 LoRa AUX 變成高電位。
 */
static bool LORA_WaitReady(uint32_t timeout_ms)
{
  uint32_t start_time = HAL_GetTick();

  while (HAL_GPIO_ReadPin(
             LORA_AUX_GPIO_Port,
             LORA_AUX_Pin) == GPIO_PIN_RESET)
  {
    if ((HAL_GetTick() - start_time) >= timeout_ms)
    {
      return false;
    }
  }

  return true;
}


/**
 * @brief 將 E22 設定成一般透明傳輸模式。
 *
 * M0 = 0
 * M1 = 0
 */
static bool LORA_SetNormalMode(void)
{
  HAL_GPIO_WritePin(
      LORA_M0_GPIO_Port,
      LORA_M0_Pin,
      GPIO_PIN_RESET);

  HAL_GPIO_WritePin(
      LORA_M1_GPIO_Port,
      LORA_M1_Pin,
      GPIO_PIN_RESET);

  HAL_Delay(10);

  return LORA_WaitReady(1000);
}


/**
 * @brief 讀取感測器資料。
 *
 * 現在先使用測試資料。
 * 等 BMP581、磁力計、IMU 驅動完成後，
 * 再把測試資料換成真實讀值。
 */
static void Sensors_ReadSnapshot(
    SensorSnapshot_t *sensor)
{
  uint32_t time_ms = HAL_GetTick();

  sensor->pressure_pa =
      101325 - (int32_t)((time_ms / 100U) % 500U);

  sensor->temperature_centi_c = 2500;

  sensor->mag_x_deci_ut = 120;
  sensor->mag_y_deci_ut = -35;
  sensor->mag_z_deci_ut = 410;

  sensor->accel_x_mg = 0;
  sensor->accel_y_mg = 0;
  sensor->accel_z_mg = 1000;

  sensor->battery_mv = 7400;

  sensor->flight_state = 0;
  sensor->status_flags = STATUS_TEST_DATA;
}


/**
 * @brief 將遙測資料透過 USART2 傳給 LoRa。
 *
 * 傳送格式：
 *
 * T,時間,氣壓,溫度,磁力X,磁力Y,磁力Z,
 *   加速度X,加速度Y,加速度Z,電池電壓
 */
static void Telemetry_Send(
    const SensorSnapshot_t *sensor)
{
  char tx_buffer[128];

  int length = snprintf(
      tx_buffer,
      sizeof(tx_buffer),
      "T,%lu,%ld,%d,%d,%d,%d,%d,%d,%d,%u\r\n",

      (unsigned long)HAL_GetTick(),

      (long)sensor->pressure_pa,
      (int)sensor->temperature_centi_c,

      (int)sensor->mag_x_deci_ut,
      (int)sensor->mag_y_deci_ut,
      (int)sensor->mag_z_deci_ut,

      (int)sensor->accel_x_mg,
      (int)sensor->accel_y_mg,
      (int)sensor->accel_z_mg,

      (unsigned int)sensor->battery_mv);

  if (length <= 0)
  {
    return;
  }

  /*
   * snprintf 回傳值可能大於 buffer 大小，
   * 因此需要限制最大傳送長度。
   */
  if (length >= (int)sizeof(tx_buffer))
  {
    length = sizeof(tx_buffer) - 1;
  }

  /*
   * 等待 LoRa 可以接收資料。
   */
  if (!LORA_WaitReady(100))
  {
    return;
  }

  /*
   * 使用 USART2 傳送給 E22。
   */
  HAL_UART_Transmit(
      &huart2,
      (uint8_t *)tx_buffer,
      (uint16_t)length,
      200);
}
///////////////////////////////////////////////////////////////////////
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
///////////////////////////////////////////////////////////////////////
if (!LORA_SetNormalMode())
{
  Error_Handler();
}

/* 第一次傳送時間 */
next_telemetry_ms = HAL_GetTick();
///////////////////////////////////////////////////////////////////////
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    uint32_t now_ms = HAL_GetTick();

  /*
   * 每 200 ms 傳送一次，也就是 5 Hz。
   */
  if ((now_ms - next_telemetry_ms) >= 200U)
  {
    next_telemetry_ms = now_ms;

    Sensors_ReadSnapshot(&latest_snapshot);
    Telemetry_Send(&latest_snapshot);
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LORA_M0_Pin|LORA_M2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LORA_M0_Pin LORA_M2_Pin */
  GPIO_InitStruct.Pin = LORA_M0_Pin|LORA_M2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LORA_AUX_Pin */
  GPIO_InitStruct.Pin = LORA_AUX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LORA_AUX_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
