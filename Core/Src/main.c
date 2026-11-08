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
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// AD0 nối GND: 0x68; AD0 nối 3V3: 0x69
#define MPU_ADDR       (0x68U << 1)
#define RAD_TO_DEG     57.2957795f

// Bắt đầu phản ứng khi góc đạt +5 hoặc -5 độ
#define REACT_ANGLE    5.0f

// Dừng khi xe nghiêng quá 35 độ
#define FALL_ANGLE     35.0f

// Đổi thành -1.0f nếu cả hai motor đẩy sai hướng
#define CONTROL_SIGN   1.0f

// PID ban đầu, cần chỉnh theo xe thực tế
#define KP             8.0f
#define KI             0.0f
#define KD             0.25f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

// Có thể xem các biến này bằng debugger ST-LINK
volatile float gocX = 0.0f;
volatile float tocDoGocX = 0.0f;

// Lệnh motor: -255 đến +255
volatile int16_t lenhMotor = 0;

// 0: đang cân chỉnh
// 1: đang hoạt động
// 2: xe nghiêng quá mức
// 3: lỗi MPU/I2C
// 4: vòng điều khiển bị trễ
volatile uint8_t trangThai = 0;

static float angleOffset = 0.0f;
static float gyroBias = 0.0f;
static float integralError = 0.0f;

static uint32_t lastControlMs = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM2_Init(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static float Clamp(float x, float lo, float hi)
{
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static float WrapAngle(float x)
{
  while (x > 180.0f) {
    x -= 360.0f;
  }

  while (x < -180.0f) {
    x += 360.0f;
  }

  return x;
}

static void Motor_Stop(void)
{
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);

  HAL_GPIO_WritePin(
    GPIOB,
    GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15,
    GPIO_PIN_RESET
  );

  lenhMotor = 0;
}

static void Motor_Set(int16_t command)
{
  if (command > 255) command = 255;
  if (command < -255) command = -255;

  // Tạm tắt PWM trước khi đặt chiều
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);

  if (command == 0) {
    Motor_Stop();
    return;
  }

  GPIO_PinState forward =
    command > 0 ? GPIO_PIN_SET : GPIO_PIN_RESET;

  GPIO_PinState reverse =
    command > 0 ? GPIO_PIN_RESET : GPIO_PIN_SET;

  // Motor trái
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, forward); // IN1
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, reverse); // IN2

  // Motor phải: chiều đã sửa bằng phần cứng
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, forward); // IN3
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, reverse); // IN4

  uint32_t magnitude =
    (uint32_t)(command < 0 ? -command : command);

  // Đổi thang lệnh 0..255 sang PWM 0..999
  uint32_t pwm = magnitude * 999U / 255U;

  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm);

  lenhMotor = command;
}

static HAL_StatusTypeDef MPU_Write(uint8_t reg, uint8_t value)
{
  return HAL_I2C_Mem_Write(
    &hi2c2,
    MPU_ADDR,
    reg,
    I2C_MEMADD_SIZE_8BIT,
    &value,
    1,
    10
  );
}

static uint8_t MPU_Read(float *angle, float *rate)
{
  uint8_t raw[14];

  if (HAL_I2C_Mem_Read(
        &hi2c2,
        MPU_ADDR,
        0x3B,
        I2C_MEMADD_SIZE_8BIT,
        raw,
        14,
        10
      ) != HAL_OK) {
    return 0;
  }

  int16_t ay =
    (int16_t)(((uint16_t)raw[2] << 8) | raw[3]);

  int16_t az =
    (int16_t)(((uint16_t)raw[4] << 8) | raw[5]);

  int16_t gx =
    (int16_t)(((uint16_t)raw[8] << 8) | raw[9]);

  if (ay == 0 && az == 0) {
    return 0;
  }

  // Góc quanh trục X
  *angle = atan2f((float)ay, (float)az) * RAD_TO_DEG;

  // Gyro được cấu hình ±500 độ/s
  *rate = (float)gx / 65.5f;

  return 1;
}

static uint8_t MPU_StartAndCalibrate(void)
{
  uint8_t id = 0;

  if (HAL_I2C_IsDeviceReady(
        &hi2c2, MPU_ADDR, 3, 10
      ) != HAL_OK) {
    return 0;
  }

  // Kiểm tra WHO_AM_I
  if (HAL_I2C_Mem_Read(
        &hi2c2,
        MPU_ADDR,
        0x75,
        I2C_MEMADD_SIZE_8BIT,
        &id,
        1,
        10
      ) != HAL_OK) {
    return 0;
  }

  if (id != 0x68) {
    return 0;
  }

  // Reset MPU
  if (MPU_Write(0x6B, 0x80) != HAL_OK) {
    return 0;
  }

  HAL_Delay(100);

  // Đánh thức MPU, chọn clock PLL
  if (MPU_Write(0x6B, 0x01) != HAL_OK) return 0;

  // Bộ lọc khoảng 21 Hz
  if (MPU_Write(0x1A, 0x04) != HAL_OK) return 0;

  // Tần số lấy mẫu: 1000 / (1 + 3) = 250 Hz
  if (MPU_Write(0x19, 0x03) != HAL_OK) return 0;

  // Gyro ±500 độ/s
  if (MPU_Write(0x1B, 0x08) != HAL_OK) return 0;

  // Gia tốc ±2g
  if (MPU_Write(0x1C, 0x00) != HAL_OK) return 0;

  HAL_Delay(200);

  // Giữ xe thẳng và yên khoảng 3 giây
  float sumSin = 0.0f;
  float sumCos = 0.0f;
  float sumRate = 0.0f;

  for (uint16_t i = 0; i < 400; ++i) {
    float angle;
    float rate;

    if (!MPU_Read(&angle, &rate)) {
      return 0;
    }

    sumSin += sinf(angle / RAD_TO_DEG);
    sumCos += cosf(angle / RAD_TO_DEG);
    sumRate += rate;

    HAL_Delay(5);
  }

  angleOffset = atan2f(sumSin, sumCos) * RAD_TO_DEG;
  gyroBias = sumRate / 400.0f;

  gocX = 0.0f;
  tocDoGocX = 0.0f;
  integralError = 0.0f;

  return 1;
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

  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  MX_GPIO_Init();
  MX_I2C2_Init();
  MX_TIM2_Init();

  /* USER CODE BEGIN 2 */

  Motor_Stop();

  if (HAL_TIM_PWM_Start(
        &htim2, TIM_CHANNEL_1
      ) != HAL_OK) {
    Error_Handler();
  }

  if (HAL_TIM_PWM_Start(
        &htim2, TIM_CHANNEL_2
      ) != HAL_OK) {
    Error_Handler();
  }

  if (MPU_StartAndCalibrate()) {
    trangThai = 1;
  } else {
    trangThai = 3;
    Motor_Stop();
  }

  lastControlMs = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // Sau lỗi hoặc ngã: dừng và đợi người dùng RESET
    if (trangThai != 1) {
      Motor_Stop();
      HAL_Delay(1);
      continue;
    }

    uint32_t now = HAL_GetTick();
    uint32_t elapsed = now - lastControlMs;

    // Điều khiển khoảng mỗi 4 ms
    if (elapsed < 4U) {
      continue;
    }

    lastControlMs = now;

    if (elapsed > 20U) {
      trangThai = 4;
      Motor_Stop();
      continue;
    }

    float dt = (float)elapsed * 0.001f;

    float accelAngle;
    float gyroRate;

    if (!MPU_Read(&accelAngle, &gyroRate)) {
      trangThai = 3;
      Motor_Stop();
      continue;
    }

    tocDoGocX = gyroRate - gyroBias;
    accelAngle = WrapAngle(accelAngle - angleOffset);

    // Dự đoán góc bằng gyro
    float prediction =
      WrapAngle(gocX + tocDoGocX * dt);

    // Bộ lọc bổ sung
    float alpha = 0.20f / (0.20f + dt);

    gocX = WrapAngle(
      prediction +
      (1.0f - alpha) * WrapAngle(accelAngle - prediction)
    );

    if (!isfinite(gocX) || !isfinite(tocDoGocX)) {
      trangThai = 3;
      Motor_Stop();
      continue;
    }

    if (fabsf(gocX) > FALL_ANGLE) {
      // Quá 35 độ: dừng, cần RESET để chạy lại
      trangThai = 2;
      integralError = 0.0f;
      Motor_Stop();

    } else if (fabsf(gocX) < REACT_ANGLE) {
      // Góc nhỏ hơn 5 độ: motor dừng
      integralError = 0.0f;
      Motor_Stop();

    } else {
      // Góc từ 5 đến 35 độ: điều khiển motor
      float error = -gocX;

      integralError += error * dt;
      integralError = Clamp(
        integralError, -50.0f, 50.0f
      );

      float output = CONTROL_SIGN * (
        KP * error +
        KI * integralError -
        KD * tocDoGocX
      );

      Motor_Set(
        (int16_t)Clamp(output, -255.0f, 255.0f)
      );
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

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;

  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType =
    RCC_CLOCKTYPE_HCLK |
    RCC_CLOCKTYPE_SYSCLK |
    RCC_CLOCKTYPE_PCLK1 |
    RCC_CLOCKTYPE_PCLK2;

  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(
        &RCC_ClkInitStruct, FLASH_LATENCY_2
      ) != HAL_OK) {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{
  /* USER CODE BEGIN I2C2_Init 0 */

  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_I2C2_CLK_ENABLE();

  GPIO_InitTypeDef pins = {0};

  // I2C2: PB10 = SCL, PB11 = SDA
  // Cần điện trở kéo lên 3V3, thường có sẵn trên module MPU
  pins.Pin = GPIO_PIN_10 | GPIO_PIN_11;
  pins.Mode = GPIO_MODE_AF_OD;
  pins.Pull = GPIO_NOPULL;
  pins.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &pins);

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */

  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

  if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
    Error_Handler();
  }

  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */
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
  htim2.Init.Prescaler = 3;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

  if (HAL_TIMEx_MasterConfigSynchronization(
        &htim2, &sMasterConfig
      ) != HAL_OK) {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

  if (HAL_TIM_PWM_ConfigChannel(
        &htim2, &sConfigOC, TIM_CHANNEL_1
      ) != HAL_OK) {
    Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(
        &htim2, &sConfigOC, TIM_CHANNEL_2
      ) != HAL_OK) {
    Error_Handler();
  }

  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

  HAL_TIM_MspPostInit(&htim2);
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

  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(
    GPIOB,
    GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15,
    GPIO_PIN_RESET
  );

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
  GPIO_InitStruct.Pin =
    GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;

  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  // TIM2 không remap: CH1 = PA0, CH2 = PA1
  __HAL_RCC_AFIO_CLK_ENABLE();
  __HAL_AFIO_REMAP_TIM2_DISABLE();

  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
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

  // Đưa ENA/ENB về LOW nếu có lỗi khởi tạo
  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitTypeDef pins = {0};
  pins.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  pins.Mode = GPIO_MODE_OUTPUT_PP;
  pins.Speed = GPIO_SPEED_FREQ_LOW;

  HAL_GPIO_Init(GPIOA, &pins);

  HAL_GPIO_WritePin(
    GPIOA,
    GPIO_PIN_0 | GPIO_PIN_1,
    GPIO_PIN_RESET
  );

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

  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */