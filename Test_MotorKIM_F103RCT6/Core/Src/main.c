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
#include <stdbool.h>
#include <stepper.h>

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
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
	uint16_t flag_10us = 0;
	
	uint32_t time_tick_ms = 0;

	
	uint16_t cnt_flag_led0 = 0;
	uint16_t cnt_flag_led1 = 0;
	uint16_t time_blink_led0 = 500;
	uint16_t time_blink_led1 = 1000;
	
	uint32_t start_time = 0;
	uint32_t interval = 0;
	
	uint8_t vartest = 0;
	uint8_t run = 0;
	uint16_t period = 80;
	uint32_t PulseCurrent = 0;
	uint32_t PulseTarget = 25600*5;
	
	bool flag_startPul = true;
	
	uint8_t getPin1 = 0, setSP = 0, vSPpos = I;
	
	int32_t target_Pul = 25600;
	volatile uint32_t __s = 0, __e = 0, __i = 0;
	
	// FLASH variables
	float FLASH_degree_I = 10.52f;
	float FLASH_degree_II = 46.51f;
	float FLASH_degree_III = 82.33f;
	float FLASH_degree_IV = 118.11f;
	float FLASH_degree_V = 154.78f;
	uint8_t FLASH_previous_position = III;
	float FLASH_speed = 2.93f;				// rpm
	
	// BLUETOOTH variables
	float BLT_degree = 0;
	uint8_t BLT_Pos_set = 0;
	
	// MCU variables
	float MCU_degree_I = 0;
	float MCU_degree_II = 0;
	float MCU_degree_III = 0;
	float MCU_degree_IV = 0;
	float MCU_degree_V = 0;
	uint8_t MCU_previous_position = 0;
	float MCU_speed_needle = 0;				// rpm
	
	uint8_t MCU_command_toSetDegree = 0;
	uint8_t MCU_command_toZero = 0;
	uint8_t  MCU_command_toRUNFREE = 0;
	uint8_t   MCU_command_toConfirm = 0;
	uint8_t  MCU_command_toAutoRun = 0;
	uint8_t MCU_command_toChangeLevel = 0;
	
	
	uint8_t flag_delay = 0;
	uint8_t an = 1;
	uint8_t result = 0;
	volatile bool flag_delay_motor = true;
	
	uint8_t cnt_motor_process = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
	uint32_t getMicroSecond(void){
		return time_tick_ms*1000 + __HAL_TIM_GetCounter(&htim1);
	}




	void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
		{
			UNUSED(htim);
			//--------------------------------------------------------
			if (htim->Instance == htim1.Instance){ 	// every 1ms
				time_tick_ms++;
				cnt_flag_led0++;
				cnt_flag_led1++;	
			}
			//--------------------------------------------------------
			if (htim->Instance == htim2.Instance){ 	// every 10us
				if(run == 3){
					if(++cnt_motor_process == 2){
						if(MCU_command_toSetDegree == 1){
							MCU_command_toSetDegree = 0;
							Command_toSetDegree(&vMotor, BLT_Pos_set, BLT_degree);
						}
						if(MCU_command_toZero == 1){
							MCU_command_toZero = 0;
							Command_toZERO(&vMotor);
						}
						if(MCU_command_toRUNFREE == 1){
							MCU_command_toRUNFREE = 0;
							Command_toRUNFREE(&vMotor, target_Pul);
						}
						if(MCU_command_toConfirm == 1){
							MCU_command_toConfirm = 0;
							Command_toConfirm(&vMotor);
						}
						if(MCU_command_toAutoRun == 1){
							MCU_command_toAutoRun = 0;
							Command_toAutoRun(&vMotor);
						}
						if(MCU_command_toChangeLevel == 1){
							MCU_command_toChangeLevel = 0;
							Command_toChangeLevel(&vMotor, vSPpos);
						}
						if(setSP == 1){
							SetTargetPos(&vMotor, vSPpos);
							setSP = 0;
						}
						Process_Motor(&vMotor);
						cnt_motor_process = 0;
					}
				}
				
			}
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
		HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
		
		
		/*Get parameter from flash here*/
		MCU_degree_I = FLASH_degree_I;
		MCU_degree_I = FLASH_degree_I;
		MCU_degree_I = FLASH_degree_I;
		MCU_degree_I = FLASH_degree_I;
		MCU_degree_I = FLASH_degree_I;
		MCU_previous_position = FLASH_previous_position;
		MCU_speed_needle = FLASH_speed;
		
		/*Init the parameter*/
		Motor_Init(&vMotor, 25600, MCU_speed_needle, 20, MCU_previous_position);
		Degree_Setup(&vMotor, FLASH_degree_I, FLASH_degree_II, FLASH_degree_III, FLASH_degree_IV, FLASH_degree_V); 		
		
		
		HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
		
		/*Start timer irq*/
		HAL_TIM_Base_Start_IT(&htim1);
		HAL_TIM_Base_Start_IT(&htim2);
		
//		HAL_GPIO_WritePin(ENA_GPIO_Port, ENA_Pin, GPIO_PIN_SET);
//		HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
	
		

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//		HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
//		HAL_Delay(100);
		

		
		
		
		
		//---------------------------------------------------------------------------
		if(cnt_flag_led0 >= time_blink_led0){
			cnt_flag_led0 = 0;
			//HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
		}
		
		if(cnt_flag_led1 >= time_blink_led1){
			cnt_flag_led1 = 0;
			//HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
		}
		
		//---------------------------------------------------------------------------
		if(vartest == 1){
			HAL_GPIO_WritePin(ENA_GPIO_Port, ENA_Pin, GPIO_PIN_RESET);
		}
		else if(vartest == 2){
			HAL_GPIO_WritePin(ENA_GPIO_Port, ENA_Pin, GPIO_PIN_SET);
		}
		//---------------------------------------------------------------------------
		if(run == 1){
			if(flag_startPul){
				start_time = getMicroSecond();
				flag_startPul = false;
			}
			else{
				interval = getMicroSecond() - start_time;
				if(interval <= period/3)
					HAL_GPIO_WritePin(PUL_GPIO_Port, PUL_Pin, GPIO_PIN_SET);
				else{
					if(interval <= period)
						HAL_GPIO_WritePin(PUL_GPIO_Port, PUL_Pin, GPIO_PIN_RESET);
					else{
						flag_startPul = true;
						/*Hoan thanh 1 xung*/
						PulseCurrent++;
						start_time = 0;
						interval = 0;
						
					}
				}
			}
			if(PulseCurrent == PulseTarget){
				run = 0;
				PulseCurrent = 0;
			}
			getPin1 = HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin);
			if(!getPin1){
				run = 0;
				PulseCurrent = 0;
			}
		}
		//---------------------------------------------------------------------------
		if(run == 2){
			if(setSP == 1){
				SetTargetPos(&vMotor, vSPpos);
				setSP = 0;
			}
			Process_Motor(&vMotor);
		}
		//---------------------------------------------------------------------------
//		if(run == 3){
//			if(MCU_command_toSetDegree == 1){
//				MCU_command_toSetDegree = 0;
//				Command_toSetDegree(&vMotor, BLT_Pos_set, BLT_degree);
//			}

//			if(MCU_command_toZero == 1){
//				MCU_command_toZero = 0;
//				Command_toZERO(&vMotor);
//			}
//			
//			if(MCU_command_toRUNFREE == 1){
//				MCU_command_toRUNFREE = 0;
//				Command_toRUNFREE(&vMotor, target_Pul);
//			}
//			
//			if(MCU_command_toConfirm == 1){
//				MCU_command_toConfirm = 0;
//				Command_toConfirm(&vMotor);
//			}
//			
//			if(MCU_command_toAutoRun == 1){
//				MCU_command_toAutoRun = 0;
//				Command_toAutoRun(&vMotor);
//			}
//			if(MCU_command_toChangeLevel == 1){
//				MCU_command_toChangeLevel = 0;
//				Command_toChangeLevel(&vMotor, vSPpos);
//			}
//			
//			if(setSP == 1){
//				SetTargetPos(&vMotor, vSPpos);
//				setSP = 0;
//			}
//			
//			Process_Motor(&vMotor);
//		}
		
		
		//---------------------------------------------------------------------------
		//---------------------------------------------------------------------------
		if(run == 4){		
			Process_Motor(&vMotor);
			if(flag_delay == 0){
				if(an){
					an = 0;
					Command_toZERO(&vMotor);
				}
				if(result == 1){
					result = 0;
					flag_delay = 1;
					if(++MCU_command_toZero > 5){
						MCU_command_toZero = 1;
						
					}		
					an = 1;
				}
			}
			else if(flag_delay == 1){
				if(MCU_command_toZero == 1){
					if(an){
						an = 0;
						Command_toRUNFREE(&vMotor, vMotor.num_Pul_I);
					}			
				}
				else if(MCU_command_toZero == 2){
					if(an){
						an = 0;
						Command_toRUNFREE(&vMotor, vMotor.num_Pul_II);
					}		
				}
				else if(MCU_command_toZero == 3){
					if(an){
						an = 0;
						Command_toRUNFREE(&vMotor, vMotor.num_Pul_III);
					}		
				}
				else if(MCU_command_toZero == 4){
					if(an){
						an = 0;
						Command_toRUNFREE(&vMotor, vMotor.num_Pul_IV);
					}	
				}
				else if(MCU_command_toZero == 5){
					if(an){
						an = 0;
						Command_toRUNFREE(&vMotor, vMotor.num_Pul_V);
					}	
				}
				 
				if(result == 2){
					flag_delay = 2;
					result = 0;
				}
			}
			else if(flag_delay == 2){
				flag_delay = 3;
				start_time = getMicroSecond();
			}
			else if(flag_delay == 3){
				if((getMicroSecond() - start_time) >= 2000000){				
					flag_delay = 0;
					an = 1;		
								
				} 
			}
		}
		
		//---------------------------------------------------------------------------
		
		//			if(BLT_Pos_set != 0){
//				if(BLT_Pos_set == 1){
//					Degree_Setting(&vMotor, I, BLT_degree);
//				}
//				else if(BLT_Pos_set == 2){
//					Degree_Setting(&vMotor, II, BLT_degree);
//				}
//				else if(BLT_Pos_set == 3){
//					Degree_Setting(&vMotor, III, BLT_degree);
//				}
//				else if(BLT_Pos_set == 4){
//					Degree_Setting(&vMotor, IV, BLT_degree);
//				}
//				else if(BLT_Pos_set == 5){
//					Degree_Setting(&vMotor, V, BLT_degree);
//				}
//				else;
//				BLT_Pos_set = 0;
//			}
		
		
		//---------------------------------------------------------------------------
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

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, PUL_Pin|ENA_Pin|DIR_Pin|LED0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : KEY1_Pin */
  GPIO_InitStruct.Pin = KEY1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KEY1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : KEY0_IRQ_Pin */
  GPIO_InitStruct.Pin = KEY0_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(KEY0_IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PUL_Pin ENA_Pin DIR_Pin LED0_Pin */
  GPIO_InitStruct.Pin = PUL_Pin|ENA_Pin|DIR_Pin|LED0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : CTHT_Pin */
  GPIO_InitStruct.Pin = CTHT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CTHT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CTHT_IRQ_Pin */
  GPIO_InitStruct.Pin = CTHT_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CTHT_IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED1_Pin */
  GPIO_InitStruct.Pin = LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

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
