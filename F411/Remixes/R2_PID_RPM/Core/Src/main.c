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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "string.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	GPIO_TypeDef *PORT;
	uint16_t DIR_PIN;
	TIM_HandleTypeDef TIMER;
	uint32_t CHANNEL;
	uint16_t A_PIN;
	uint16_t B_PIN;
	uint8_t DIR;
	uint16_t PWM;
	uint16_t CUR_RPM;
	uint16_t TAR_RPM;
	uint32_t LAST_TICK;
} MOT_TypeDef;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFFER_LENGTH 64
#define SOL_INTERVAL 10000

#define NO_OF_MOTORS 2
#define LEFT_SHIFT 1

#define KP 1
#define KD 1
#define KI 1

#define MAX_PWM 65535
#define MIN_PWM 0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
char buffer[BUFFER_LENGTH];

uint8_t errorFlag = 0;

uint16_t ms = 0;
uint16_t output = 0;

uint32_t currentTime;

int16_t integral = 0;
int16_t previousError = 0;

MOT_TypeDef motor[NO_OF_MOTORS];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void updateTime() {
	currentTime = ms * 1000 + TIM4->CNT;
}

void updateEncoder(uint8_t motIndex) {
	updateTime();

	motor[motIndex].CUR_RPM = 60000000
			/ (currentTime - motor[motIndex].LAST_TICK);
}

void PIDController(float targetRPM, float currentRPM) {
	int16_t error = targetRPM - currentRPM;
	integral += error;
	int16_t derivative = error - previousError;
	output = (KP * error) + (KI * integral) + (KD * derivative);

	previousError = error;
}

int isCleared(char *buffer) {
	for (uint8_t i = 0; i < BUFFER_LENGTH; i++) {
		if (buffer[i] != '\0') {
			return 0;  // Buffer is not cleared
		}
	}
	return 1;  // Buffer is cleared
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */
	uint32_t lastSOL = 0;

	char message[BUFFER_LENGTH + 20];

	motor[0].PORT = GPIOB;
	motor[0].DIR_PIN = GPIO_PIN_12;
	motor[0].TIMER = htim1;
	motor[0].CHANNEL = TIM_CHANNEL_1;
	motor[0].A_PIN = GPIO_PIN_6;
	motor[0].B_PIN = GPIO_PIN_7;

	motor[1].PORT = GPIOB;
	motor[1].DIR_PIN = GPIO_PIN_14;
	motor[1].TIMER = htim1;
	motor[1].CHANNEL = TIM_CHANNEL_3;
	motor[1].A_PIN = GPIO_PIN_8;
	motor[1].B_PIN = GPIO_PIN_9;

	for (uint8_t i = 0; i < NO_OF_MOTORS; i++) {
		motor[i].DIR = 0;
		motor[i].PWM = 0;
		motor[i].TAR_RPM = 0;
		motor[i].CUR_RPM = 0;
	}
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
	MX_TIM4_Init();
	/* USER CODE BEGIN 2 */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

	for (uint8_t i = 0; i < NO_OF_MOTORS; i++) {
		if ((HAL_TIMEx_PWMN_Start(&(motor[i].TIMER), motor[i].CHANNEL) == HAL_OK)) {
			sprintf(message, "Motor %d PWM initialised\n\r", i);
		} else {
			sprintf(message, "Error initialising motor %d PWM\n\r", i);
			errorFlag = 1;
		}

		HAL_GPIO_WritePin(motor[i].PORT, motor[i].DIR_PIN, GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&(motor[i].TIMER), motor[i].CHANNEL, 0);

		CDC_Transmit_FS((uint8_t*) message, strlen(message));
		HAL_Delay(1);
	}

	if (!errorFlag) {
		sprintf(message, "All systems operational\n\r");
		CDC_Transmit_FS((uint8_t*) message, strlen(message));
		HAL_Delay(1);

		sprintf(message, "F411 Black Pill online\n\r");
		CDC_Transmit_FS((uint8_t*) message, strlen(message));
		HAL_Delay(1);
	} else {
		sprintf(message, "STM started with some errors\n\r");
		CDC_Transmit_FS((uint8_t*) message, strlen(message));
		HAL_Delay(1);
	}
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		updateTime();

		if (!isCleared(buffer)) {
			char receivedStr[BUFFER_LENGTH];

			sprintf(message, "String Received: %s\n\r", buffer);
			CDC_Transmit_FS((uint8_t*) message, strlen(message));
			HAL_Delay(1);

			sprintf(receivedStr, "%s", buffer);
			uint8_t receivedStrlen = strlen(receivedStr);

			if (receivedStr[receivedStrlen - 1] == 'A') {
				uint8_t arrIndex = NO_OF_MOTORS - 1;
				uint8_t multiplier = 1;

				for (uint8_t i = 0; i < NO_OF_MOTORS; i++) {
					motor[i].DIR = 0;
					motor[i].TAR_RPM = 0;
				}

				for (int8_t i = receivedStrlen - 2; i >= 0; i--) {
					if (receivedStr[i] >= '0' && receivedStr[i] <= '9') {
						motor[arrIndex].TAR_RPM += (receivedStr[i] - '0')
								* multiplier;
						multiplier *= 10;
					} else if (receivedStr[i] == '-') {
						motor[arrIndex].DIR = 1;
					} else if (receivedStr[i] == ',') {
						arrIndex--;
						multiplier = 1;
					}
				}

				for (uint8_t i = 0; i < NO_OF_MOTORS; i++) {
					HAL_GPIO_WritePin(motor[0].PORT, motor[0].DIR_PIN,
							motor[0].DIR);
				}
			}

			memset(buffer, '\0', BUFFER_LENGTH);

		} else if (currentTime - lastSOL >= SOL_INTERVAL) {
			CDC_Transmit_FS((uint8_t*) "SOL\n\r", strlen("SOL\n\r"));
			lastSOL = currentTime;
		}

		for (uint8_t i = 0; i < NO_OF_MOTORS; i++) {
			PIDController(motor[i].TAR_RPM, motor[i].CUR_RPM);

			if ((motor[i].PWM + output) >= MAX_PWM) {
				motor[i].PWM = MAX_PWM;
			} else if ((motor[i].PWM + output) <= MIN_PWM) {
				motor[i].PWM = MIN_PWM;
			} else {
				motor[i].PWM += output;
			}

			if(motor[i].TAR_RPM == 0){
				motor[i].PWM = 0;
			}

			__HAL_TIM_SET_COMPARE(&(motor[i].TIMER), motor[i].CHANNEL,
					motor[i].PWM << LEFT_SHIFT);

			sprintf(message, "Dir%d: %d Tar_RPM%d: %d Cur_RPM%d: %d PWM%d: %d\n\r", i, motor[i].DIR, i,
					motor[i].TAR_RPM, i, motor[i].CUR_RPM, i, motor[i].PWM);
			CDC_Transmit_FS((uint8_t*) message, strlen(message));
			HAL_Delay(1);	//Doesn't work without this line
		}

		HAL_Delay(1);
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
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

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
	RCC_OscInitStruct.PLL.PLLM = 25;
	RCC_OscInitStruct.PLL.PLLN = 192;
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

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 65535;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */
	HAL_TIM_MspPostInit(&htim1);

}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void) {

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 95;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 1000;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim4) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, MOT1_Dir_Pin | MOT2_Dir_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : LED_Pin */
	GPIO_InitStruct.Pin = LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : MOT1_Dir_Pin MOT2_Dir_Pin */
	GPIO_InitStruct.Pin = MOT1_Dir_Pin | MOT2_Dir_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : MOT1_EXTIA_Pin MOT1_EXTIB_Pin MOT2_EXTIA_Pin MOT2_EXTIB_Pin */
	GPIO_InitStruct.Pin = MOT1_EXTIA_Pin | MOT1_EXTIB_Pin | MOT2_EXTIA_Pin
			| MOT2_EXTIB_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	ms++;
	if (errorFlag) {
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	}

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == motor[0].A_PIN || GPIO_Pin == motor[0].B_PIN) {
		updateEncoder(0);
	} else if (GPIO_Pin == motor[1].A_PIN || GPIO_Pin == motor[1].B_PIN) {
		updateEncoder(1);
	}
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
