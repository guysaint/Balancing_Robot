/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <math.h>
#define WHO_AM_I_REG 0x75
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

/* USER CODE BEGIN PV */
// --- [1. MPU6050 관련 변수] ---
uint8_t MPU6050_ADDR = 0xD0; // (0x68 << 1) or (0x70 << 1) 확인 필요
int16_t Acc_Y_Raw, Acc_Z_Raw;
int16_t Gyro_X_Raw;
float Acc_Angle, Gyro_Rate;
float Current_Angle = 0.0f;
float Loop_Time = 0.01f; // 10ms (100Hz)

// --- [2. PID 제어 변수] ---
// ★ 튜닝할 때 여기 숫자만 바꾸면 됩니다!
float Kp = 400.0f;   // 비례 항 (우선 이것만 사용)
float Ki = 0.0f;    // 적분 항 (나중에)
float Kd = 0.0f;    // 미분 항 (나중에)

float Target_Angle = -0.55f; // 수직일 때 센서 오차값 (캘리브레이션 값)
float Error, Prev_Error;
float P_Term, I_Term, D_Term;
float PID_Output;

// --- [3. 모터 제어 변수] ---
int Motor_PWM_L, Motor_PWM_R;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Read_MPU6050(void);
void Motor_Control(int speed_L, int speed_R);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// STM32CubeIDE(GCC)에서 printf 출력을 UART2로 연결
int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
    return ch;
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

    printf("Balance Robot Start!\r\n");

    // 1. 통신 및 타이머 시작
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

    // 2. MPU6050 깨우기 (Sleep Mode 해제)
    uint8_t data = 0;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x6B, 1, &data, 1, 100);
    HAL_Delay(100);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	uint32_t current_time;
	uint32_t prev_time = 0;

	while (1)
	{
	  current_time = HAL_GetTick();

	  // --- [10ms 주기 제어 루프 (100Hz)] ---
	  if (current_time - prev_time >= 10)
	  {
		prev_time = current_time;

		// 1. 센서 값 읽기 및 각도 계산 (상보필터)
		Read_MPU6050();

		// 2. PID 계산
		Error = Target_Angle - Current_Angle; // 목표 - 현재

		P_Term = Kp * Error;
		I_Term += Ki * Error * Loop_Time;
		D_Term = Kd * (Error - Prev_Error) / Loop_Time;

		// I항 누적 제한 (Windup 방지 - 필요 시 활성화)
		// if(I_Term > 500) I_Term = 500; if(I_Term < -500) I_Term = -500;

		Prev_Error = Error;

		PID_Output = P_Term + I_Term + D_Term; // 최종 모터 출력값

		// 3. 모터 구동
		// PID 출력이 양수면 앞으로(넘어지려는 쪽으로), 음수면 뒤로
		// PWM 범위 제한 (-999 ~ 999)
		if (PID_Output > 999) PID_Output = 999;
		if (PID_Output < -999) PID_Output = -999;

		// 모터 함수에 입력 (왼쪽, 오른쪽 동일하게 적용)
		Motor_Control((int)PID_Output, (int)PID_Output);

		// 4. 디버깅 (필요할 때만 주석 해제 - 너무 빠르면 렉 걸림)
		// printf("Angle: %.2f | PWM: %d\r\n", Current_Angle, (int)PID_Output);
	  }
	}
  /* USER CODE END 3 */
  }
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
  RCC_OscInitStruct.PLL.PLLN = 100;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void Read_MPU6050(void) {
	uint8_t buffer[6]; // 데이터 읽기용 버퍼

	// 가속도 읽기 (0x3D: ACCEL_YOUT_H 부터 Z까지) - Y, Z축 사용
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x3D, 1, buffer, 4, 100);
	Acc_Y_Raw = (int16_t)(buffer[0] << 8 | buffer[1]);
	Acc_Z_Raw = (int16_t)(buffer[2] << 8 | buffer[3]);

	// 자이로 읽기 (0x43: GYRO_XOUT_H) - X축 회전 사용
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x43, 1, buffer, 2, 100);
	Gyro_X_Raw = (int16_t)(buffer[0] << 8 | buffer[1]);

	// 각도 계산 (상보필터)
	// 1. 가속도 각도 (atan2)
	Acc_Angle = atan2f((float)Acc_Y_Raw, (float)Acc_Z_Raw) * 57.296f;

	// 2. 자이로 각속도
	Gyro_Rate = Gyro_X_Raw / 131.0f;

	// 3. 상보필터 적용 (0.96 : 0.04)
	Current_Angle = 0.96f * (Current_Angle + Gyro_Rate * Loop_Time) + 0.04f * Acc_Angle;
}

void Motor_Control(int speed_L, int speed_R) {
	// --- 왼쪽 모터 제어 ---
	if (speed_L > 0) { // 전진
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);   // L_IN1
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET); // L_IN2
	} else { // 후진
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
		speed_L = -speed_L; // PWM은 양수여야 함
	}

	// --- 오른쪽 모터 제어 ---
	if (speed_R > 0) { // 전진
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);   // R_IN1
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET); // R_IN2
	} else { // 후진
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
		speed_R = -speed_R;
	}

	// 데드존(Deadzone) 보정: 모터가 돌기 시작하는 최소 전압 (약 100~200)
	// 값이 너무 작으면 모터가 웅~ 소리만 내고 안 돕니다.
	if (speed_L > 0 && speed_L < 350) speed_L = 350;
	if (speed_R > 0 && speed_R < 350) speed_R = 350;

	// 최대 속도 제한
	if (speed_L > 999) speed_L = 999;
	if (speed_R > 999) speed_R = 999;

	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, speed_L);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, speed_R);
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
