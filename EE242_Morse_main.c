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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "string.h"
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
TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
static void transmit_decoded_text();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define MAX_TEXT_LENGTH 50
volatile uint8_t currentMorseIndex = 0;
volatile uint8_t currentMorseStartIndex = 0;
volatile char morseInputBuffer[MAX_TEXT_LENGTH*4] = "";
volatile char textOutputBuffer[MAX_TEXT_LENGTH] = "";
volatile char usbOutputBuffer[MAX_TEXT_LENGTH] = "";


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
  MX_TIM1_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  TIM1->CCR1 = 2000;	// Dot time
  TIM1->CCR2 = 4000;// Dash Time
  TIM1->CCR3 = 8000;// Next dot dash time *only valid when SW is low
  TIM1->CCR4 = 12000;// Next character time *only valid when SW is low
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

	  //Transmit button pushed (pull-up)
	  if(!HAL_GPIO_ReadPin(TRANSMIT_Sw_GPIO_Port, TRANSMIT_Sw_Pin)){
		  transmit_decoded_text();
		  uint8_t i = 0;
		  for(i=0;i<sizeof(morseInputBuffer);i++){
			  char c = morseInputBuffer[i];
			  if(c=='.'){
				  HAL_GPIO_WritePin(DOT_Led_GPIO_Port, DOT_Led_Pin, 1);
				  HAL_GPIO_WritePin(DASH_Led_GPIO_Port, DASH_Led_Pin, 0);
				  HAL_Delay(200);
			  }
			  else if(c=='-'){
				  HAL_GPIO_WritePin(DOT_Led_GPIO_Port, DOT_Led_Pin, 0);
				  HAL_GPIO_WritePin(DASH_Led_GPIO_Port, DASH_Led_Pin, 1);
				  HAL_Delay(400);
			  }
			  if(c=='\0'){
				  break;
			  }
		  }
	  }

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 288;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 6;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 36000-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 16000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DOT_Led_Pin|DEBUG_Led_Pin|DASH_Led_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : MORSE_Sw_Pin */
  GPIO_InitStruct.Pin = MORSE_Sw_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MORSE_Sw_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TRANSMIT_Sw_Pin */
  GPIO_InitStruct.Pin = TRANSMIT_Sw_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(TRANSMIT_Sw_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DOT_Led_Pin DEBUG_Led_Pin DASH_Led_Pin */
  GPIO_InitStruct.Pin = DOT_Led_Pin|DEBUG_Led_Pin|DASH_Led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

static void transmit_decoded_text(){
	memcpy(usbOutputBuffer,textOutputBuffer,sizeof(textOutputBuffer));
	strcat(usbOutputBuffer,"\n");
	CDC_Transmit_FS((uint8_t*)usbOutputBuffer, strlen(usbOutputBuffer));
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim){

	HAL_GPIO_WritePin(DOT_Led_GPIO_Port, DOT_Led_Pin, 1);
	HAL_GPIO_WritePin(DASH_Led_GPIO_Port, DASH_Led_Pin, 1);
	HAL_GPIO_WritePin(DEBUG_Led_GPIO_Port, DEBUG_Led_Pin, 1);
	if(currentMorseStartIndex > 0){
		morseInputBuffer[currentMorseStartIndex-1] = 'S';
		uint8_t strPos = strlen(textOutputBuffer);
		if(strPos > 0){
			textOutputBuffer[strPos] = ' ';
		}
	}


	HAL_TIM_Base_Stop_IT(&htim1);
	HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_1);
	HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_2);
	HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_3);
	HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_4);
	__HAL_TIM_SET_COUNTER(&htim1,0);
}

static char decode_morse_character(char* buf,uint8_t len){
	if(buf[0] == '.' && buf[1] == '-' && len == 2){
		return 'A';
	}
	if(buf[0] == '-' && buf[1] == '.'&& buf[2] == '.'&& buf[3] == '.' && len == 4){
		return 'B';
	}
	if(buf[0] == '-' && buf[1] == '.'&& buf[2] == '-'&& buf[3] == '.' && len == 4){
		return 'C';
	}
	if(buf[0] == '-' && buf[1] == '.'&& buf[2] == '.'&& len == 3){
		return 'D';
	}
	if(buf[0] == '.' && len == 1){
		return 'E';
	}
	if(buf[0] == '.' && buf[1] == '.'&& buf[2] == '-'&& buf[3] == '.' && len == 4){
		return 'F';
	}
	if(buf[0] == '-' && buf[1] == '-'&& buf[2] == '.' && len == 3){
		return 'G';
	}
	if(buf[0] == '.' && buf[1] == '.'&& buf[2] == '.'&& buf[3] == '.' && len == 4){
		return 'H';
	}
	if(buf[0] == '.' && buf[1] == '.'&& len == 2){
		return 'I';
	}
	if(buf[0] == '.' && buf[1] == '-'&& buf[2] == '-'&& buf[3] == '-' && len == 4){
		return 'J';
	}
	if(buf[0] == '-' && buf[1] == '.'&& buf[2] == '-' && len == 3){
		return 'K';
	}
	if(buf[0] == '.' && buf[1] == '-'&& buf[2] == '.'&& buf[3] == '.' && len == 4){
		return 'L';
	}
	if(buf[0] == '-' && buf[1] == '-' && len == 2){
		return 'M';
	}
	if(buf[0] == '-' && buf[1] == '.' && len == 2){
		return 'N';
	}
	if(buf[0] == '-' && buf[1] == '-'&& buf[2] == '-' && len == 3){
		return 'O';
	}
	if(buf[0] == '.' && buf[1] == '-'&& buf[2] == '-'&& buf[3] == '.' && len == 4){
		return 'P';
	}
	if(buf[0] == '-' && buf[1] == '-'&& buf[2] == '-'&& buf[3] == '-' && len == 4){
		return 'Q';
	}
	if(buf[0] == '.' && buf[1] == '-'&& buf[2] == '.' && len == 3){
		return 'R';
	}
	if(buf[0] == '.' && buf[1] == '.'&& buf[2] == '.' && len == 3){
		return 'S';
	}
	if(buf[0] == '-' && len == 1){
		return 'T';
	}
	if(buf[0] == '.' && buf[1] == '.'&& buf[2] == '-' && len == 3){
		return 'U';
	}
	if(buf[0] == '.' && buf[1] == '.'&& buf[2] == '.'&& buf[3] == '-' && len == 4){
		return 'V';
	}
	if(buf[0] == '-' && buf[1] == '.'&& buf[2] == '-'&& buf[3] == '-' && len == 4){
		return 'Y';
	}
	if(buf[0] == '-' && buf[1] == '-'&& buf[2] == '.'&& buf[3] == '.' && len == 4){
		return 'Z';
	}
}


void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef* htim)
{

	uint8_t nextMorseIndex = currentMorseIndex+1 ;
	uint8_t currentState = HAL_GPIO_ReadPin(MORSE_Sw_GPIO_Port, MORSE_Sw_Pin);

	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1 && currentState){
		morseInputBuffer[currentMorseIndex] = '.';
		HAL_GPIO_WritePin(DOT_Led_GPIO_Port, DOT_Led_Pin, 1);
		HAL_GPIO_WritePin(DASH_Led_GPIO_Port, DASH_Led_Pin, 0);
		HAL_GPIO_WritePin(DEBUG_Led_GPIO_Port, DEBUG_Led_Pin, 0);
	}
	else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2 && currentState){
		morseInputBuffer[currentMorseIndex] = '-';
		HAL_GPIO_WritePin(DOT_Led_GPIO_Port, DOT_Led_Pin, 0);
		HAL_GPIO_WritePin(DASH_Led_GPIO_Port, DASH_Led_Pin, 1);
		HAL_GPIO_WritePin(DEBUG_Led_GPIO_Port, DEBUG_Led_Pin, 0);
	}
	else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3){
		//Save current dot/dash
		currentMorseIndex = nextMorseIndex;
		HAL_GPIO_WritePin(DOT_Led_GPIO_Port, DOT_Led_Pin, 0);
		HAL_GPIO_WritePin(DASH_Led_GPIO_Port, DASH_Led_Pin, 0);
		HAL_GPIO_WritePin(DEBUG_Led_GPIO_Port, DEBUG_Led_Pin, 1);
	}
	else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4){
		//Next Character
		HAL_GPIO_WritePin(DOT_Led_GPIO_Port, DOT_Led_Pin, 1);
		HAL_GPIO_WritePin(DASH_Led_GPIO_Port, DASH_Led_Pin, 1);
		HAL_GPIO_WritePin(DEBUG_Led_GPIO_Port, DEBUG_Led_Pin, 0);
		if(currentMorseIndex > 0){

			morseInputBuffer[currentMorseIndex] = '|';
			currentMorseIndex = nextMorseIndex;

			char* startPosition = &morseInputBuffer[currentMorseStartIndex];
			char c = decode_morse_character(startPosition, currentMorseIndex - currentMorseStartIndex -1);
			char tempStr[] = {c,'\0'};
			strcat(textOutputBuffer,&tempStr);
			currentMorseStartIndex = currentMorseIndex;

		}
		__HAL_TIM_CLEAR_FLAG(&htim1,TIM_FLAG_UPDATE);
		__HAL_TIM_ENABLE_IT(&htim1,TIM_IT_UPDATE);

	}
	//morseInputBuffer[63] = '\n';
	//CDC_Transmit_FS((uint8_t*)morseInputBuffer, sizeof(morseInputBuffer));

}



void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	//Ignore if not from  our switch.
	if(GPIO_Pin != MORSE_Sw_Pin){
		return;
	}
	uint8_t currentState = HAL_GPIO_ReadPin(MORSE_Sw_GPIO_Port, MORSE_Sw_Pin);
	uint16_t currentTimerValue = __HAL_TIM_GET_COUNTER(&htim1);
	//If falling edge and time span is short < 500*0.1 ms then ignore the event
	if((!currentState && currentTimerValue < 500)){
		return;
	}
	//If rising edge reset and start the timer
	if(currentState){
		//sprintf(textOutputBuffer,"R INPUT: %d | %d\n",currentState,currentTimerValue);
	    //CDC_Transmit_FS((uint8_t*)textOutputBuffer, strlen(textOutputBuffer));
		HAL_GPIO_WritePin(DEBUG_Led_GPIO_Port, DEBUG_Led_Pin, 0);
		HAL_GPIO_WritePin(DOT_Led_GPIO_Port, DOT_Led_Pin, 0);
		HAL_GPIO_WritePin(DASH_Led_GPIO_Port, DASH_Led_Pin, 0);

		__HAL_TIM_DISABLE_IT(&htim1,TIM_IT_UPDATE);
		__HAL_TIM_CLEAR_FLAG(&htim1,TIM_FLAG_UPDATE);

		HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_1);
		HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_2);
		HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_3);
		HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_4);
		__HAL_TIM_SET_COUNTER(&htim1,0);
		HAL_TIM_OC_Start_IT(&htim1,TIM_CHANNEL_1);
		HAL_TIM_OC_Start_IT(&htim1,TIM_CHANNEL_2);
		HAL_TIM_OC_Start_IT(&htim1,TIM_CHANNEL_3);
		HAL_TIM_OC_Start_IT(&htim1,TIM_CHANNEL_4);
	}
	else{
		//sprintf(textOutputBuffer,"F INPUT: %d | %d\n",currentState,currentTimerValue);
	    //CDC_Transmit_FS((uint8_t*)textOutputBuffer, strlen(textOutputBuffer));
		//sprintf(textOutputBuffer,"TIM: %d\n",__HAL_TIM_GET_COUNTER(&htim1));
		//CDC_Transmit_FS((uint8_t*)textOutputBuffer, strlen(textOutputBuffer));

		/*HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_1);
		HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_2);
		HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_3);
		HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_4);
		__HAL_TIM_SET_COUNTER(&htim1,0);
		HAL_TIM_OC_Start_IT(&htim1,TIM_CHANNEL_1);
		HAL_TIM_OC_Start_IT(&htim1,TIM_CHANNEL_2);
		HAL_TIM_OC_Start_IT(&htim1,TIM_CHANNEL_3);
		HAL_TIM_OC_Start_IT(&htim1,TIM_CHANNEL_4);*/
	}

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
