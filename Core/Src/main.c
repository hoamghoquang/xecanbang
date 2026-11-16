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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
  BALANCE_STARTING = 0,
  BALANCE_RUNNING = 1,
  BALANCE_FALLEN = 2,
  BALANCE_MPU_ERROR = 3,
  BALANCE_LOOP_LATE = 4,
  BALANCE_INIT_ERROR = 5
} BalanceStatus;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RAD_TO_DEG          57.2957795f
#define MPU_ADDR_AD0_LOW    (0x68U << 1)
#define MPU_ADDR_AD0_HIGH   (0x69U << 1)
#define MPU_WHO_AM_I        0x75U
#define MPU_PWR_MGMT_1      0x6BU
#define MPU_SMPLRT_DIV      0x19U
#define MPU_CONFIG          0x1AU
#define MPU_GYRO_CONFIG     0x1BU
#define MPU_ACCEL_CONFIG    0x1CU
#define MPU_ACCEL_XOUT_H    0x3BU
#define MPU_GYRO_SCALE      65.5f  // LSB/(deg/s) với thang ±500 deg/s

#define CONTROL_PERIOD_MS   5U
#define CONTROL_MAX_LATE_MS 15U
#define MPU_READ_TIMEOUT_MS 4U
#define FILTER_TAU_S        0.50f
#define MPU_MOUNT_OFFSET_DEG 180.0f
#define FALL_ANGLE_DEG      45.0f

// Đổi dấu nếu cả hai bánh chạy xa phía xe đang ngả.
#define CONTROL_SIGN         1.0f
// Chỉnh riêng nếu hai bánh đẩy xe theo hai hướng ngược nhau.
#define LEFT_MOTOR_SIGN      1
#define RIGHT_MOTOR_SIGN     1

// Giá trị khởi điểm, chỉnh lại theo khung xe, motor và mức pin.
#define REACT_ANGLE_DEG      0.0f
#define BALANCE_TRIM_DEG     0.3f
// Đặt PWM tối đa mỗi khi bộ điều khiển yêu cầu motor quay.
#define MOTOR_LIMIT        1000
#define KP                 38.0f
#define KI                  0.0f
#define KD                  1.5f
#define INTEGRAL_LIMIT     50.0f
#define PWM_PRESCALER        3U
#define PWM_PERIOD         999U
#define MOTOR_INPUT_PINS   (GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15)
#define MOTOR_ENABLE_PINS  (GPIO_PIN_0 | GPIO_PIN_1)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
// Xem các biến này trong Expressions/Live Expressions của CubeIDE.
volatile float gocThoMPU;
volatile float gocTrongLuc;
volatile uint32_t soMauMPU;
volatile uint32_t soLoiMPU;
volatile uint32_t loiI2C;
volatile float gocX = 0.0f;       // Độ, đã bù góc lắp MPU
volatile float tocDoGocX = 0.0f;  // Độ/giây
volatile int16_t lenhMotor = 0;   // Lệnh PWM có dấu, 1000 = 100%
volatile BalanceStatus trangThai = BALANCE_STARTING;

static uint16_t mpuAddress;

static float integralPWM;
static uint32_t lastControlMs;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static float WrapAngle(float angle)
{
  while (angle > 180.0f) {
    angle -= 360.0f;
  }
  while (angle < -180.0f) {
    angle += 360.0f;
  }
  return angle;
}

static void Motor_Stop(void)
{
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0U);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0U);
  // Nạp CCR ngay để ENA/ENB về LOW, kể cả khi CCR dùng preload.
  htim2.Instance->EGR = TIM_EGR_UG;
  HAL_GPIO_WritePin(GPIOB, MOTOR_INPUT_PINS, GPIO_PIN_RESET);
  lenhMotor = 0;
}

static void Motor_Set(int16_t command)
{
  if (command == 0 && lenhMotor == 0) {
    return;
  }
  if (command == 0 || (lenhMotor > 0 && command < 0) ||
      (lenhMotor < 0 && command > 0)) {
    // Khi đảo chiều, ngắt cả hai cầu cho tới lần cập nhật kế tiếp.
    Motor_Stop();
    return;
  }

  if (lenhMotor == 0) {
    uint32_t highPins = command * LEFT_MOTOR_SIGN > 0 ? GPIO_PIN_12 : GPIO_PIN_13;
    highPins |= command * RIGHT_MOTOR_SIGN > 0 ? GPIO_PIN_14 : GPIO_PIN_15;
    uint32_t lowPins = MOTOR_INPUT_PINS & ~highPins;
    GPIOB->BSRR = (lowPins << 16U) | highPins;
  }

  uint32_t magnitude = (uint32_t)(command > 0 ? command : -command);
  uint32_t compare = (magnitude * (PWM_PERIOD + 1U)) / MOTOR_LIMIT;
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, compare);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, compare);
  lenhMotor = command;
}

static bool MPU_Write(uint8_t reg, uint8_t value)
{
  return HAL_I2C_Mem_Write(&hi2c1, mpuAddress, reg, I2C_MEMADD_SIZE_8BIT,
                           &value, 1U, 20U) == HAL_OK;
}

static int16_t ReadSigned16(const uint8_t *bytes)
{
  return (int16_t)(((uint16_t)bytes[0] << 8U) | bytes[1]);
}

static bool MPU_Read(float *angle, float *rate)
{
  uint8_t raw[10];  // ACCEL XYZ, nhiệt độ, GYRO X

  // Bus này chỉ có một master. Nếu bus đã kẹt, dừng ngay.
  if (__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_BUSY) != RESET ||
      HAL_I2C_Mem_Read(&hi2c1, mpuAddress, MPU_ACCEL_XOUT_H,
                       I2C_MEMADD_SIZE_8BIT, raw, sizeof(raw),
                       MPU_READ_TIMEOUT_MS) != HAL_OK) {
    loiI2C = HAL_I2C_GetError(&hi2c1);
    ++soLoiMPU;
    return false;
  }

  int16_t ay = ReadSigned16(&raw[2]);
  int16_t az = ReadSigned16(&raw[4]);
  int16_t gx = ReadSigned16(&raw[8]);

  if (ay == 0 && az == 0) {
    ++soLoiMPU;
    return false;
  }

  gocThoMPU = atan2f((float)ay, (float)az) * RAD_TO_DEG;
  gocTrongLuc = WrapAngle(gocThoMPU - MPU_MOUNT_OFFSET_DEG);
  ++soMauMPU;
  *angle = gocTrongLuc;
  *rate = (float)gx / MPU_GYRO_SCALE;
  return true;
}

static bool MPU_Init(void)
{
  uint8_t id;
  HAL_Delay(100U);

  mpuAddress = MPU_ADDR_AD0_LOW;
  if (HAL_I2C_IsDeviceReady(&hi2c1, mpuAddress, 2U, 10U) != HAL_OK) {
    mpuAddress = MPU_ADDR_AD0_HIGH;
    if (HAL_I2C_IsDeviceReady(&hi2c1, mpuAddress, 2U, 10U) != HAL_OK) {
      return false;
    }
  }

  if (HAL_I2C_Mem_Read(&hi2c1, mpuAddress, MPU_WHO_AM_I,
                       I2C_MEMADD_SIZE_8BIT, &id, 1U, 20U) != HAL_OK ||
      id != 0x68U || !MPU_Write(MPU_PWR_MGMT_1, 0x80U)) {
    return false;
  }
  HAL_Delay(100U);

  // PLL gyro X, DLPF gyro 42 Hz / accel 44 Hz, lấy mẫu 250 Hz.
  if (!MPU_Write(MPU_PWR_MGMT_1, 0x01U) ||
      !MPU_Write(MPU_CONFIG, 0x03U) ||
      !MPU_Write(MPU_SMPLRT_DIV, 0x03U) ||
      !MPU_Write(MPU_GYRO_CONFIG, 0x08U) ||
      !MPU_Write(MPU_ACCEL_CONFIG, 0x00U)) {
    return false;
  }
  HAL_Delay(100U);
  return true;
}

static float Clamp(float value, float limit)
{
  return fmaxf(-limit, fminf(value, limit));
}

static BalanceStatus Balance_Start(void)
{
  float angle;
  float rate;
  if (!MPU_Read(&angle, &rate)) {
    return BALANCE_MPU_ERROR;
  }
  gocX = angle;
  tocDoGocX = rate;
  return fabsf(angle) >= FALL_ANGLE_DEG ? BALANCE_FALLEN : BALANCE_RUNNING;
}

static BalanceStatus Balance_Update(float dt)
{
  float accelAngle;
  float gyroRate;
  if (!MPU_Read(&accelAngle, &gyroRate)) {
    return BALANCE_MPU_ERROR;
  }

  float rate = gyroRate;
  float predicted = WrapAngle(gocX + rate * dt);
  float correction = dt / (FILTER_TAU_S + dt);
  float angle = WrapAngle(predicted + correction * WrapAngle(accelAngle - predicted));
  gocX = angle;
  tocDoGocX = rate;

  if (fabsf(angle) >= FALL_ANGLE_DEG) {
    return BALANCE_FALLEN;
  }

  float gravityError = BALANCE_TRIM_DEG - accelAngle;
  if (fabsf(gravityError) < REACT_ANGLE_DEG) {
    integralPWM = 0.0f;
    Motor_Set(0);
    return BALANCE_RUNNING;
  }

  float error = BALANCE_TRIM_DEG - angle;
  float pd = KP * error - KD * rate;
  float nextIntegral = Clamp(integralPWM + KI * error * dt, INTEGRAL_LIMIT);
  float nextPWM = pd + nextIntegral;
  // Cho phép tích phân giảm khi lệnh đã bão hòa.
  if (fabsf(nextPWM) <= MOTOR_LIMIT || nextPWM * error < 0.0f) {
    integralPWM = nextIntegral;
  }
  float pwm = Clamp(pd + integralPWM, MOTOR_LIMIT);
  int16_t command = (int16_t)lroundf(CONTROL_SIGN * pwm);
  Motor_Set(command);
  return BALANCE_RUNNING;
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

  /* Initialize the peripherals used by the balance controller */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  Motor_Stop();
  if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK ||
      HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2) != HAL_OK) {
    Error_Handler();
  }

  // Dùng mẫu trọng lực đầu tiên làm giá trị ban đầu cho bộ lọc.
  bool mpuReady = MPU_Init();
  trangThai = mpuReady ? Balance_Start() : BALANCE_MPU_ERROR;
  lastControlMs = HAL_GetTick();
  uint32_t lastMPURetryMs = lastControlMs;
  uint32_t lastDiagnosticMs = lastControlMs;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // Sau khi ngắt motor, tiếp tục đọc góc để kiểm tra bằng Watch.
    if (trangThai != BALANCE_RUNNING) {
      uint32_t now = HAL_GetTick();
      if (now - lastDiagnosticMs < 50U) {
        continue;
      }
      lastDiagnosticMs = now;
      // Nếu mất MPU, thử kết nối lại để đọc góc. Motor vẫn khóa đến khi reset.
      if (!mpuReady && now - lastMPURetryMs >= 1000U) {
        lastMPURetryMs = now;
        mpuReady = MPU_Init();
      }

      float angle;
      float rate;
      if (mpuReady && MPU_Read(&angle, &rate)) {
        gocX = angle;
        tocDoGocX = rate;
      } else if (mpuReady) {
        mpuReady = false;
        lastMPURetryMs = HAL_GetTick();
      }
      continue;
    }

    uint32_t now = HAL_GetTick();
    uint32_t elapsed = now - lastControlMs;
    if (elapsed < CONTROL_PERIOD_MS) {
      continue;
    }
    lastControlMs = now;

    BalanceStatus status = BALANCE_LOOP_LATE;
    if (elapsed <= CONTROL_MAX_LATE_MS) {
      status = Balance_Update((float)elapsed * 0.001f);
    }
    if (status != BALANCE_RUNNING) {
      Motor_Stop();
      trangThai = status;
    }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  // I2C1 mặc định: PB6 = SCL, PB7 = SDA, kéo lên 3,3 V bên ngoài.
  __HAL_RCC_I2C1_CLK_ENABLE();
  GPIO_InitTypeDef pins = {0};
  pins.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  pins.Mode = GPIO_MODE_AF_OD;
  pins.Pull = GPIO_NOPULL;
  pins.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &pins);
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  __HAL_RCC_TIM2_CLK_ENABLE();

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = PWM_PRESCALER;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = PWM_PERIOD;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  GPIO_InitTypeDef pins = {0};
  pins.Pin = MOTOR_ENABLE_PINS;
  pins.Mode = GPIO_MODE_AF_PP;
  pins.Pull = GPIO_NOPULL;
  pins.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &pins);

  /* USER CODE END TIM2_Init 2 */
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  __HAL_RCC_AFIO_CLK_ENABLE();
  // I2C1 và TIM2 dùng chân mặc định, giữ SWD trên PA13/PA14.
  MODIFY_REG(AFIO->MAPR,
             AFIO_MAPR_I2C1_REMAP | AFIO_MAPR_TIM2_REMAP | AFIO_MAPR_SWJ_CFG,
             AFIO_MAPR_SWJ_CFG_JTAGDISABLE);

  // Giữ ENA và ENB ở LOW trong lúc cấu hình timer.
  HAL_GPIO_WritePin(GPIOA, MOTOR_ENABLE_PINS, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = MOTOR_ENABLE_PINS;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  HAL_GPIO_WritePin(GPIOA, MOTOR_ENABLE_PINS, GPIO_PIN_RESET);

  GPIO_InitTypeDef pins = {0};
  pins.Pin = MOTOR_ENABLE_PINS;
  pins.Mode = GPIO_MODE_OUTPUT_PP;
  pins.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &pins);

  lenhMotor = 0;
  trangThai = BALANCE_INIT_ERROR;
  __disable_irq();
  while (1) {
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
  (void)file;
  (void)line;
  Error_Handler();
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
