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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
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
  /* USER CODE BEGIN 2 */
  // MPU6050/6500 초기화 및 데이터 읽기 변수
  uint8_t check_val = 0;
  uint8_t data[14]; // 가속도(6) + 온도(2) + 자이로(6) 데이터를 한 번에 읽을 버퍼
  int16_t Acc_X_Raw, Acc_Y_Raw, Acc_Z_Raw;
  int16_t Gyro_X_Raw, Gyro_Y_Raw, Gyro_Z_Raw;

  printf("System Init Done!\r\n"); // 초기화 완료 메시지

  // 1. 센서 ID 확인(0x68 또는 0x70)
  // HAL_I2C_Mem_Read(핸들러, 디바이스 주소, 레지스터 주소, 주소크기, 저장할 버퍼, 데이터 길이, 타임아웃)
  HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, WHO_AM_I_REG, 1, &check_val, 1, 100);
  if (check_val == 0x68 || check_val == 0x70)
  {
	  printf("Sensor Connected! (ID: 0x%02X)\r\n", check_val);
	  // 2. 센서 깨우기 (Power Management 1 레지스터 0x6B에 0을 써서 Sleep 해제)
	  uint8_t val = 0;
	  HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x6B, 1, &val, 1, 100);
  }

  else
  {
	  printf("Error: Unknown ID: 0x%02X\r\n", check_val);
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	// 3. 데이터 읽기 (레지스터 0x3B부터 14바이트를 한 번에 읽음)
	// 순서: Accel_X(H,L), Accel_Y, Accel_Z, Temp, Gyro_X, Gyro_Y, Gyro_Z
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x3B, 1, data, 14, 100);

	// 4. 데이터 합치기 (High Byte <<8 | Low Byte)
	Acc_X_Raw = (int16_t)(data[0] << 8 | data[1]);
	Acc_Y_Raw = (int16_t)(data[2] << 8 | data[3]);
	Acc_Z_Raw = (int16_t)(data[4] << 8 | data[5]);

	Gyro_X_Raw = (int16_t)(data[8] << 8 | data[9]);
	Gyro_Y_Raw = (int16_t)(data[10] << 8 | data[11]);
	Gyro_Z_Raw = (int16_t)(data[12] << 8 | data[13]);

	// 5. 시리얼 출력 (Raw Data 확인)
	// 센서를 손으로 기울이면서 숫자가 변하는지 확인해야 함.
	printf("AX:%5d AY:%5d AZ:%5d | GX:%5d GY:%5d GZ:%5d\r\n", Acc_X_Raw, Acc_Y_Raw, Acc_Z_Raw, Gyro_X_Raw, Gyro_Y_Raw, Gyro_Z_Raw);

	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); // 동작 확인용 LED
	HAL_Delay(100); // 0.1초마다 갱신
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
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
