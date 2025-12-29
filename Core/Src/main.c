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
#define MPU6050_ADDR 0xD0 // (0x68 << 1) : 7비트 주소를 8비트로 변환
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
// 엔코더 값을 저장할 변수 (0 ~ 65535)
uint16_t enc_L = 0;
uint16_t enc_R = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  printf("Motor & Encoder Test Start!\r\n");

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // PWM 시작
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); // 엔코더 시작
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

  __HAL_TIM_SET_COUNTER(&htim3, 30000); // 초기값 설정
  __HAL_TIM_SET_COUNTER(&htim4, 30000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  // --- [1단계] 앞으로 가기 (2초) ---
	    printf("Direction: FORWARD\r\n");

	    // 왼쪽 전진 (IN1=High, IN2=Low)
	    HAL_GPIO_WritePin(GPIOC, L_IN1_Pin, GPIO_PIN_SET);
	    HAL_GPIO_WritePin(GPIOC, L_IN2_Pin, GPIO_PIN_RESET);
	    // 오른쪽 전진 (IN1=High, IN2=Low)
	    HAL_GPIO_WritePin(GPIOC, R_IN1_Pin, GPIO_PIN_SET);
	    HAL_GPIO_WritePin(GPIOC, R_IN2_Pin, GPIO_PIN_RESET);

	    // 속도 30% (ARR이 4999이므로 약 1500)
	    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1500);
	    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 1500);

	    HAL_Delay(2000); // 2초 주행

	    // --- [엔코더 값 확인] ---
	    enc_L = __HAL_TIM_GET_COUNTER(&htim3);
	    enc_R = __HAL_TIM_GET_COUNTER(&htim4);
	    printf("Encoder L: %d | R: %d\r\n", enc_L, enc_R);

	    // --- [2단계] 정지 (1초) ---
	    printf("Direction: STOP\r\n");

	    // 정지 (모두 Low)
	    HAL_GPIO_WritePin(GPIOC, L_IN1_Pin, GPIO_PIN_RESET);
	    HAL_GPIO_WritePin(GPIOC, L_IN2_Pin, GPIO_PIN_RESET);
	    HAL_GPIO_WritePin(GPIOC, R_IN1_Pin, GPIO_PIN_RESET);
	    HAL_GPIO_WritePin(GPIOC, R_IN2_Pin, GPIO_PIN_RESET);

	    // 속도 0
	    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
	    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);

	    HAL_Delay(1000);

	    // --- [3단계] 뒤로 가기 (2초) ---
	    printf("Direction: BACKWARD\r\n");

	    // 왼쪽 후진 (IN1=Low, IN2=High)
	    HAL_GPIO_WritePin(GPIOC, L_IN1_Pin, GPIO_PIN_RESET);
	    HAL_GPIO_WritePin(GPIOC, L_IN2_Pin, GPIO_PIN_SET);
	    // 오른쪽 후진 (IN1=Low, IN2=High)
	    HAL_GPIO_WritePin(GPIOC, R_IN1_Pin, GPIO_PIN_RESET);
	    HAL_GPIO_WritePin(GPIOC, R_IN2_Pin, GPIO_PIN_SET);

	    // 속도 30%
	    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1500);
	    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 1500);

	    HAL_Delay(2000);

	    // --- [엔코더 값 확인] ---
	    enc_L = __HAL_TIM_GET_COUNTER(&htim3);
	    enc_R = __HAL_TIM_GET_COUNTER(&htim4);
	    printf("Encoder L: %d | R: %d\r\n", enc_L, enc_R);

	    // --- [4단계] 정지 (1초) ---
	    HAL_GPIO_WritePin(GPIOC, L_IN1_Pin, GPIO_PIN_RESET);
	    HAL_GPIO_WritePin(GPIOC, L_IN2_Pin, GPIO_PIN_RESET);
	    HAL_GPIO_WritePin(GPIOC, R_IN1_Pin, GPIO_PIN_RESET);
	    HAL_GPIO_WritePin(GPIOC, R_IN2_Pin, GPIO_PIN_RESET);
	    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
	    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);

	    HAL_Delay(1000);
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
