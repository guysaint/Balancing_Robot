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
#include <stdlib.h>
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
int16_t Acc_X_Raw, Acc_Z_Raw;
int16_t Gyro_Y_Raw;
float Acc_Angle, Gyro_Rate;
float Current_Angle = 0.0f;
float Loop_Time = 0.01f; // 10ms (100Hz)

// --- [2. PID 제어 변수] ---
// ★ 튜닝할 때 여기 숫자만 바꾸면 됩니다!
float Kp = 0.0f;   // 비례 항 - 힘
float Ki = 0.0f;    // 누적 오차 보정 - 적분
float Kd = 0.0f;    // 급발진 방지(진동을 잡아줌) - 미분

float Target_Angle = -1.3f; // 수직일 때 센서 오차값 (캘리브레이션 값)
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

	  if (current_time - prev_time >= 10) // 10ms 주기 (100Hz)
	  {
		prev_time = current_time;

		// 1. 센서 값 읽기
		Read_MPU6050();

		// ★ [안전장치 추가] 40도 이상 기울어지면 모터 끄기!
		if (Current_Angle > 40.0f || Current_Angle < -40.0f)
		{
			Motor_Control(0, 0); // 모터 정지

			// 넘어져 있을 때는 적분항(I)이 계속 쌓여서 폭발하는 것을 방지하기 위해 초기화
			Error = 0;
			Prev_Error = 0;
			I_Term = 0;
			PID_Output = 0;
		}
		else // 정상 범위(서 있을 때)에만 PID 작동
		{
			// 2. PID 계산
			Error = Target_Angle - Current_Angle;

			P_Term = Kp * Error;
			I_Term += Ki * Error * Loop_Time;
			D_Term = Kd * (Error - Prev_Error) / Loop_Time;

			// I항 누적 제한 (안전용)
			if(I_Term > 2000) I_Term = 2000;
			if(I_Term < -2000) I_Term = -2000;

			Prev_Error = Error;

			// 출력 계산 (부호 확인: 아까 반대로 바꾼 것 유지)
			PID_Output = P_Term + I_Term + D_Term;

			// 3. 모터 제한 및 구동 (풀 파워 해제 버전)
			if (PID_Output > 4700) PID_Output = 4700;
			if (PID_Output < -4700) PID_Output = -4700;

			Motor_Control((int)PID_Output, (int)PID_Output);
		}
		// [디버깅용 코드 추가]
		  // i 변수는 static으로 선언해서 값이 유지되게 함
		  //static int print_count = 0;
		  //if(print_count++ > 20) { // 200ms마다 한 번씩 출력
			//  printf("Angle: %.2f\r\n", Current_Angle);
			//  print_count = 0;
		 // }
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

	// 가속도 읽기 (0x3B: ACCEL_YOUT_H 부터 Z까지) - X, Z축 사용
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x3B, 1, buffer, 6, 100);
	Acc_X_Raw = (int16_t)(buffer[0] << 8 | buffer[1]);
	Acc_Z_Raw = (int16_t)(buffer[4] << 8 | buffer[5]);

	// 자이로 읽기 (0x45: GYRO_XOUT_H) - Y축 회전 사용
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x45, 1, buffer, 2, 100);
	Gyro_Y_Raw = (int16_t)(buffer[0] << 8 | buffer[1]);

	// 각도 계산 (상보필터)
	// 1. 가속도 각도 (atan2)
	Acc_Angle = atan2f((float)Acc_X_Raw, (float)Acc_Z_Raw) * 57.296f;

	// 2. 자이로 각속도
	Gyro_Rate = Gyro_Y_Raw / 131.0f;

	// 3. 상보필터 적용 (0.96 : 0.04)
	Current_Angle = 0.96f * (Current_Angle + Gyro_Rate * Loop_Time) + 0.04f * Acc_Angle;
}

void Motor_Control(int speed_L, int speed_R) {
    // ----------------------------------------
    // [1] 방향 제어 (GPIO 설정)
    // ----------------------------------------

    // 왼쪽 모터 방향
    if (speed_L >= 0) {
        // 전진 (포트 설정은 사용자 환경에 맞게 유지)
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);   // IN1
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET); // IN2
    } else {
        // 후진
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
    }

    // 오른쪽 모터 방향
    if (speed_R >= 0) {
        // 전진
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);   // IN1
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET); // IN2
    } else {
        // 후진
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
    }

    // ----------------------------------------
    // [2] 속도값(PWM) 계산 (절대값 변환)
    // ----------------------------------------
    // 방향은 이미 정했으니, 이제 속도(크기)만 남깁니다.
    speed_L = abs(speed_L);
    speed_R = abs(speed_R);

    // ----------------------------------------
    // [3] 데드존(Deadzone) 보정
    // ----------------------------------------
    // 모터가 0이 아니면, 최소 900의 힘은 줘야 바퀴가 구릅니다.
    // (800에서 반응이 약했다면 900으로 올려보세요)
    if (speed_L > 0 && speed_L < 900) speed_L = 900;
    if (speed_R > 0 && speed_R < 900) speed_R = 900;

    // ----------------------------------------
    // [4] 최대 속도 제한
    // ----------------------------------------
    if (speed_L > 4700) speed_L = 4700;
    if (speed_R > 4700) speed_R = 4700;

    // ----------------------------------------
    // [5] 최종 PWM 출력
    // ----------------------------------------
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
