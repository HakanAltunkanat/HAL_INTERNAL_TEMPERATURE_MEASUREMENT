/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
ADC_HandleTypeDef hadc1; // ADC handle for ADC1
TIM_HandleTypeDef htim2; // Timer handle for TIM2

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void); // System Clock configuration function
static void MX_GPIO_Init(void); // GPIO initialization function
static void MX_TIM2_Init(void); // TIM2 initialization function
static void MX_ADC1_Init(void); // ADC1 initialization function
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t seven_config[] = {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F}; // 7-segment display configuration for digits 0-9
uint8_t config; // Configuration variable for 7-segment display
uint16_t digits[4]; // Array to hold the digits to be displayed
uint16_t adc_value[2]; // Array to hold ADC values
#define VREF_INT (uint16_t*)(uint32_t)(0x1FFF7A2A) // Internal reference voltage address
float Vdda = 0, Vsense = 0, avg_slope = 2.5, V_25 = 0.76, Temp = 0, decimal_part; // Variables for temperature calculation
int count = 1, int_part, int_part_decimal; // Variables for digit handling and display count

void adc_read() {
    HAL_ADC_Start(&hadc1); // Start ADC conversion
    if(HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
        adc_value[0] = HAL_ADC_GetValue(&hadc1); // Get the first ADC value (VREF_INT)

        if(HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
            adc_value[1] = HAL_ADC_GetValue(&hadc1); // Get the second ADC value (temperature sensor)
            Vdda = 3.3 * ((float)(*VREF_INT) / adc_value[0]); // Calculate Vdda
            Vsense = Vdda * ((float)adc_value[1] / 4095); // Calculate Vsense
            Temp = (float)(Vsense - V_25) / avg_slope + 25; // Calculate temperature
        }
    }
}

void seven_display(uint8_t digit) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, SET); // Turn off all segments
    config = seven_config[digit]; // Get the configuration for the digit

    if(config & 0x01) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, RESET); // Turn on segment 1 if required
    if(config & 0x02) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, RESET); // Turn on segment 2 if required
    if(config & 0x04) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, RESET); // Turn on segment 3 if required
    if(config & 0x08) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, RESET); // Turn on segment 4 if required
    if(config & 0x10) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, RESET); // Turn on segment 5 if required
    if(config & 0x20) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, RESET); // Turn on segment 6 if required
    if(config & 0x40) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, RESET); // Turn on segment 7 if required
}

void digit_config(float number) {
    int_part = (int) number; // Get the integer part of the number
    decimal_part = (float) number - int_part; // Get the decimal part of the number
    digits[0] = int_part / 10; // Get the tens digit
    digits[1] = int_part % 10; // Get the units digit
    int_part_decimal = (int)(100 * decimal_part); // Get the first two decimal digits
    digits[2] = int_part_decimal / 10; // Get the tenths digit
    digits[3] = int_part_decimal % 10; // Get the hundredths digit
    if(count == 16) count = 1; // Reset the count if it reaches 16
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 , RESET); // Turn off all digit enable pins

    if(count & 0x01) {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, SET); // Enable digit 1
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, SET); // Enable decimal point
        seven_display(digits[3]); // Display the hundredths digit
    }
    if(count & 0x02) {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, SET); // Enable digit 2
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, SET); // Enable decimal point
        seven_display(digits[2]); // Display the tenths digit
    }
    if(count & 0x04) {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, SET); // Enable digit 3
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, RESET); // Disable decimal point
        seven_display(digits[1]); // Display the units digit
    }
    if(count & 0x08) {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, SET); // Enable digit 4
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, SET); // Enable decimal point
        seven_display(digits[0]); // Display the tens digit
    }
    count *= 2; // Double the count for the next digit
}

void TIM2_IRQHandler(void) {
  /* USER CODE BEGIN TIM2_IRQn 0 */
    adc_read(); // Read the ADC values
    digit_config(Temp); // Configure the digits to display the temperature
  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2); // Handle TIM2 interrupt
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init(); // Initialize the HAL Library

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config(); // Configure the system clock

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init(); // Initialize GPIO
  MX_TIM2_Init(); // Initialize TIM2
  MX_ADC1_Init(); // Initialize ADC1
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2); // Start TIM2 with interrupt
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, SET); // Turn on the LEDs on GPIOD pins 12, 13, 14, and 15

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
    Error_Handler();
  }
}

/* ADC1 Initialization Function */
static void MX_ADC1_Init(void) {

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    Error_Handler();
  }

  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/* TIM2 Initialization Function */
static void MX_TIM2_Init(void) {

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 4800 - 1; // Set the prescaler value
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10000 - 1; // Set the period value
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }

  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/* GPIO Initialization Function */
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4
                          | GPIO_PIN_5 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14
                          | GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4
                          | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PD1 PD2 PD3 PD4
                           PD5 PD12 PD13 PD14
                           PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4
                          | GPIO_PIN_5 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14
                          | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA2 PA3 PA4
                           PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4
                          | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add their own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add their own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
