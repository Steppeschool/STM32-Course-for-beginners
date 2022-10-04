/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void gpio_init();
void external_it_init();
void timer_init();
void adc_init();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t timer_value1, timer_value2;
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
  /* USER CODE BEGIN 2 */
  gpio_init();
  external_it_init();
  adc_init();
  timer_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  timer_value1 = TIM1 ->CNT;
	  HAL_Delay(1000);
	  timer_value2 = TIM1 ->CNT;
	  HAL_Delay(1000);
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_10;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void gpio_init()
{
	// GPIOA, Pin 10 and Pin 9, alternate function pins
	RCC ->AHB2ENR |= 0x3;

	LED2_CONTROL_GPIO_Port ->MODER &= ~(0X3 << LED2_CONTROL_Pin_Pos * 2);
	LED2_CONTROL_GPIO_Port ->MODER |= (0x2 << LED2_CONTROL_Pin_Pos * 2);

	LED1_CONTROL_GPIO_Port ->MODER &= ~(0X3 << LED1_CONTROL_Pin_Pos * 2);
	LED1_CONTROL_GPIO_Port ->MODER |= (0x2 << LED1_CONTROL_Pin_Pos * 2);

	//	AF1, pin 9 and 10
	LED1_CONTROL_GPIO_Port ->AFR[1] &= ~(0Xf << 4);
	LED1_CONTROL_GPIO_Port ->AFR[1] |= (0x1 << 4);
	LED1_CONTROL_GPIO_Port ->AFR[1] &= ~(0Xf << 8);
	LED1_CONTROL_GPIO_Port ->AFR[1] |= (0x1 << 8);

	// GPIOB, pin 7
	BUTTON_GPIO_Port ->MODER &= ~(0X3 << BUTTON_Pin_Pos * 2);

	// pull-up resistor, pin 7, PORT B
	BUTTON_GPIO_Port ->PUPDR &= ~(0X3 << BUTTON_Pin_Pos * 2);
	BUTTON_GPIO_Port ->PUPDR |= (0x1 << BUTTON_Pin_Pos * 2);
}

void external_it_init()
{

	RCC -> APB2ENR |= 0x1;
	SYSCFG -> EXTICR[1] &= ~(0x7 << 12);
	SYSCFG -> EXTICR[1] |= (0x1 << 12);

	EXTI -> IMR1 |= 1 << BUTTON_Pin_Pos;
	EXTI -> RTSR1 |= 1 << BUTTON_Pin_Pos;
	EXTI -> FTSR1 |= 1 << BUTTON_Pin_Pos;

	NVIC_SetPriority(EXTI9_5_IRQn, 0);
	NVIC_EnableIRQ(EXTI9_5_IRQn);
}

void timer_init()
{
	RCC ->APB2ENR |= 0x1 << RCC_APB2ENR_TIM1EN_Pos;
	//	32 MHz / (31 + 1) = 1 MHz
	TIM1 -> PSC  = 31;
	//	1 MHz / (999 + 1) = 1 kHz
	TIM1 ->ARR = 999;

	// channel 2 and 3 output
	TIM1 -> CCMR1 &= ~(TIM_CCMR1_CC2S_Msk);
	TIM1 -> CCMR2 &= ~(TIM_CCMR2_CC3S_Msk);
	// channel 2 and 3, pwm mode 1
	TIM1 -> CCMR1 &= ~(TIM_CCMR1_OC2M_Msk);
	TIM1 -> CCMR1 |= (0b110 << TIM_CCMR1_OC2M_Pos);
	TIM1 -> CCMR2 &= ~(TIM_CCMR2_OC3M_Msk);
	TIM1 -> CCMR2 |= (0b110 << TIM_CCMR2_OC3M_Pos);

	// repetition counter
	TIM1 ->RCR = 9;
	// channel 2 and 3 enable
	TIM1 -> CCMR1 |= 1 << TIM_CCMR1_OC2PE_Pos;
	TIM1 -> CCMR2 |= 1 << TIM_CCMR2_OC3PE_Pos;

	// channel 2 and 3 output enable
	TIM1 ->CCER |= 1 << TIM_CCER_CC2E_Pos;
	TIM1 ->CCER |= 1 << TIM_CCER_CC3E_Pos;

	TIM1 ->BDTR |= 1 << TIM_BDTR_MOE_Pos;
	TIM1 ->EGR  |= 1 << TIM_EGR_UG_Pos;

	TIM1 ->DIER |= 1 << TIM_DIER_UIE_Pos;

	TIM1 -> CR1 |= 1;
	NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 0);
	NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);

}

void adc_init()
{
	//	clock enable
	RCC ->AHB2ENR |= (1 << RCC_AHB2ENR_ADCEN_Pos);

	//	system clock
	ADC12_COMMON -> CCR &= ~(ADC_CCR_CKMODE_Msk);

	//	presecaler - 32
	ADC12_COMMON -> CCR &= ~(ADC_CCR_PRESC_Msk);
	ADC12_COMMON -> CCR |= 	(0x8 << ADC_CCR_PRESC_Pos);

	//	Deeppwd = 0, ADVREGEN = 1
	ADC1 -> CR &= ~(1 << ADC_CR_DEEPPWD_Pos);
	ADC1 -> CR |= (1 << ADC_CR_ADVREGEN_Pos);

	//	channels 5,6, and 8 singled-ended
	ADC1 ->DIFSEL &= ~(ADC_DIFSEL_DIFSEL_5);
	ADC1 ->DIFSEL &= ~(ADC_DIFSEL_DIFSEL_6);
	ADC1 ->DIFSEL &= ~(ADC_DIFSEL_DIFSEL_7);

	//	Calibration
	ADC1 -> CR &= ~(ADC_CR_ADCALDIF_Msk);
	ADC1 ->CR |= (1 << ADC_CR_ADCAL_Pos);
	while((ADC1 ->CR & (1 << ADC_CR_ADCAL_Pos)));

	//	Enabling ADC
	ADC1 -> ISR &=~(ADC_CSR_ADRDY_MST_Msk);
	ADC1 -> CR |= (1 << ADC_CR_ADEN_Pos);
	while(!(ADC1 -> ISR & (ADC_CSR_ADRDY_MST_Msk)));

	//	3 conversions,
	ADC1 ->SQR1 &= ~(ADC_SQR1_L_Msk);
	ADC1 ->SQR1 |= (0x3);

	//	channel 5,
	ADC1 ->SQR1 &= ~(ADC_SQR1_SQ1_Msk);
	ADC1 ->SQR1 |= (0x5 << ADC_SQR1_SQ1_Pos);

	//	channel 6,
	ADC1 ->SQR1 &= ~(ADC_SQR1_SQ2_Msk);
	ADC1 ->SQR1 |= (0x6 << ADC_SQR1_SQ2_Pos);

	//	channel 8,
	ADC1 ->SQR1 &= ~(ADC_SQR1_SQ3_Msk);
	ADC1 ->SQR1 |= (0x8 << ADC_SQR1_SQ3_Pos);

	//	12.5 clock cycles: 32 Mhz/ (1000 (presecaler))/ (12.5 + 12.5) = 40 kHz
	ADC1 -> SMPR1 &= ~(ADC_SMPR1_SMP5_Msk);
	ADC1 -> SMPR1 &= ~(ADC_SMPR1_SMP6_Msk);
	ADC1 -> SMPR1 &= ~(ADC_SMPR1_SMP8_Msk);

	ADC1 -> SMPR1 |= (0x2 << ADC_SMPR1_SMP5_Pos);
	ADC1 -> SMPR1 |= (0x2 << ADC_SMPR1_SMP6_Pos);
	ADC1 -> SMPR1 |= (0x2 << ADC_SMPR1_SMP8_Pos);
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

#ifdef  USE_FULL_ASSERT
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

