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
	uint16_t PIN;
	TIM_HandleTypeDef *TIMER;
	uint32_t CHANNEL;
	uint8_t DIR;
	uint16_t PWM;
} MOTOR_TypeDef;

typedef struct {
	TIM_HandleTypeDef *TIMER;
	uint32_t CHANNEL;
	uint16_t US;
} SERVO_TypeDef;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFFER_LENGTH 64
#define SOL_INTERVAL 10000
#define LEFT_SHIFT

#define LS0_PIN GPIO_PIN_0	//LEAD
#define LS1_PIN GPIO_PIN_1	//LEAD
#define LS2_PIN GPIO_PIN_2	//RACK
#define LS3_PIN GPIO_PIN_10	//RACK

#define RACK_SPEED  19660//3276
#define LEAD_SPEED  45874//19660//3276

#define RACK_COOLDOWN 100
#define LEAD_COOLDOWN 100

#define WRIST_DEFAULT 2000
#define WRIST_ACTIVE 650

#define GRIPPER_DEFAULT 1400
#define GRIPPER_ACTIVE 1100

enum motors {
	RACK, LEAD
};

enum servos {
	WRIST, GRIPPER
};

enum interrupts {
	DISABLED, ENABLED
};

enum rackDirs {
	FORWARDS, BACKWARDS
};

enum leadDirs {
	UPWARDS, DOWNWARDS
};
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
char buffer[BUFFER_LENGTH];

//For rackExti and leadExti:
//0: interrupt disabled
//1: interrupt enabled
volatile uint8_t rackExti = ENABLED; //make sure rack limit switches are NOT pressed at startuo
volatile uint8_t leadExti = ENABLED; //make sure lead limit switches are NOT pressed at startuo

uint8_t errorFlag = 0;

char message[BUFFER_LENGTH + 20];

MOTOR_TypeDef motor[2];
SERVO_TypeDef servo[2];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//void stopMotor(uint8_t index) {
//	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
//	sprintf(message, "Motor %d interrupt\n\r", index);
//	CDC_Transmit_FS((uint8_t*) message, strlen(message));
//}

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
//	uint32_t lastSOL = 0;
	motor[LEAD].PORT = GPIOB;
	motor[LEAD].PIN = GPIO_PIN_14;
	motor[LEAD].TIMER = &htim1;
	motor[LEAD].CHANNEL = TIM_CHANNEL_3;

	motor[RACK].PORT = GPIOB;
	motor[RACK].PIN = GPIO_PIN_12;
	motor[RACK].TIMER = &htim1;
	motor[RACK].CHANNEL = TIM_CHANNEL_1;

	servo[WRIST].TIMER = &htim3;
	servo[WRIST].CHANNEL = TIM_CHANNEL_1;
	servo[WRIST].US = WRIST_DEFAULT;

	servo[GRIPPER].TIMER = &htim3;
	servo[GRIPPER].CHANNEL = TIM_CHANNEL_2;
	servo[GRIPPER].US = GRIPPER_DEFAULT;

	for (uint8_t i = 0; i < 2; i++) {
		motor[i].DIR = 0;
		motor[i].PWM = 0;
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
	MX_TIM3_Init();
	MX_TIM4_Init();
	MX_USB_DEVICE_Init();
	/* USER CODE BEGIN 2 */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

	for (uint8_t i = 0; i < 2; i++) {
		if ((HAL_TIMEx_PWMN_Start(motor[i].TIMER, motor[i].CHANNEL) == HAL_OK)) { //<-This is PWMN
			sprintf(message, "Motor %d PWM initialised\n\r", i);
		} else {
			sprintf(message, "Error initialising motor %d PWM\n\r", i);
		}
		CDC_Transmit_FS((uint8_t*) message, strlen(message));
		HAL_Delay(1);
	}

	for (uint8_t i = 0; i < 2; i++) {
		if (HAL_TIM_PWM_Start(servo[i].TIMER, servo[i].CHANNEL)) { //<-This is PWM
			sprintf(message, "Servo %d PWM initialised\n\r", i);
		} else {
			sprintf(message, "Error initialising servo %d PWM\n\r", i);
		}
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

	__HAL_TIM_SET_COMPARE(servo[WRIST].TIMER, servo[WRIST].CHANNEL,
			WRIST_DEFAULT);
	__HAL_TIM_SET_COMPARE(servo[GRIPPER].TIMER, servo[GRIPPER].CHANNEL,
			GRIPPER_DEFAULT);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
//		uint32_t current_time = HAL_GetTick();

		if (leadExti == DISABLED) {
			if (motor[LEAD].DIR == 0) {
				motor[LEAD].DIR = 1;
			} else if (motor[LEAD].DIR == 1) {
				motor[LEAD].DIR = 0;
			}
			HAL_GPIO_TogglePin(motor[LEAD].PORT, motor[LEAD].PIN);
			HAL_Delay(LEAD_COOLDOWN);
			motor[LEAD].PWM = 0;
			__HAL_TIM_SET_COMPARE(motor[LEAD].TIMER, motor[LEAD].CHANNEL,
					motor[LEAD].PWM);

			leadExti = ENABLED;	//Re-enabling the interrupts
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
		}

		if (rackExti == DISABLED) {
			if (motor[RACK].DIR == 0) {
				motor[RACK].DIR = 1;
			} else if (motor[RACK].DIR == 1) {
				motor[RACK].DIR = 0;
			}
			HAL_GPIO_TogglePin(motor[RACK].PORT, motor[RACK].PIN);
			HAL_Delay(RACK_COOLDOWN);
			motor[RACK].PWM = 0;
			__HAL_TIM_SET_COMPARE(motor[RACK].TIMER, motor[RACK].CHANNEL,
					motor[RACK].PWM);

			rackExti = ENABLED;	//Re-enabling the interrupts
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
		}

		if (!isCleared(buffer)) {
			char receivedStr[BUFFER_LENGTH];

//			sprintf(message, "String Received: %s\n\r", buffer);
//			CDC_Transmit_FS((uint8_t*) message, strlen(message));

			sprintf(receivedStr, "%s", buffer);

			if (receivedStr[0] == '0') {	//RESET
				sprintf(message, "0 \r\n");
				CDC_Transmit_FS((uint8_t*) message, strlen(message));
				HAL_Delay(1);

//				for (uint8_t i = 0; i < 2; i++) {
//					motor[i].DIR = 0;
//					motor[i].PWM = 0;
//				}
//
//				servo[WRIST].US = WRIST_DEFAULT;
//				servo[GRIPPER].US = GRIPPER_DEFAULT;
			} else if (receivedStr[0] == '1') {	//RETRACT
				if (rackExti == ENABLED) {
					sprintf(message, "-1 \r\n");
					CDC_Transmit_FS((uint8_t*) message, strlen(message));
					HAL_Delay(1);

					HAL_GPIO_WritePin(motor[RACK].PORT, motor[RACK].PIN,
							BACKWARDS);
					__HAL_TIM_SET_COMPARE(motor[RACK].TIMER,
							motor[RACK].CHANNEL, RACK_SPEED);

				}
			} else if (receivedStr[0] == '2') {	//EXTEND
				if (rackExti == ENABLED) {
					sprintf(message, "-2 \r\n");
					CDC_Transmit_FS((uint8_t*) message, strlen(message));
					HAL_Delay(1);

					HAL_GPIO_WritePin(motor[RACK].PORT, motor[RACK].PIN,
							FORWARDS);
					__HAL_TIM_SET_COMPARE(motor[RACK].TIMER,
							motor[RACK].CHANNEL, RACK_SPEED);

				}
			} else if (receivedStr[0] == '3') {	//DESCALATE
				if (leadExti == ENABLED) {
					sprintf(message, "-3 \r\n");
					CDC_Transmit_FS((uint8_t*) message, strlen(message));
					HAL_Delay(1);

					HAL_GPIO_WritePin(motor[LEAD].PORT, motor[LEAD].PIN,
							DOWNWARDS);
					__HAL_TIM_SET_COMPARE(motor[LEAD].TIMER,
							motor[LEAD].CHANNEL, LEAD_SPEED);

				}
			} else if (receivedStr[0] == '4') {	//ESCALATE
				if (leadExti == ENABLED) {
					sprintf(message, "-4 \r\n");
					CDC_Transmit_FS((uint8_t*) message, strlen(message));
					HAL_Delay(1);

					HAL_GPIO_WritePin(motor[LEAD].PORT, motor[LEAD].PIN,
							UPWARDS);
					__HAL_TIM_SET_COMPARE(motor[LEAD].TIMER,
							motor[LEAD].CHANNEL, LEAD_SPEED);

				}
			} else if (receivedStr[0] == '5') {	//ARM_DOWN
				sprintf(message, "5 \r\n");
				CDC_Transmit_FS((uint8_t*) message, strlen(message));
				HAL_Delay(1);

				__HAL_TIM_SET_COMPARE(servo[WRIST].TIMER, servo[WRIST].CHANNEL,
						WRIST_ACTIVE);

				HAL_Delay(1000);
				sprintf(message, "-5 \r\n");
				CDC_Transmit_FS((uint8_t*) message, strlen(message));
				HAL_Delay(1);

			} else if (receivedStr[0] == '6') {	//ARM_UP
				sprintf(message, "6 \r\n");
				CDC_Transmit_FS((uint8_t*) message, strlen(message));
				HAL_Delay(1);

				__HAL_TIM_SET_COMPARE(servo[WRIST].TIMER, servo[WRIST].CHANNEL,
						WRIST_DEFAULT);

				HAL_Delay(1000);
				sprintf(message, "-6 \r\n");
				CDC_Transmit_FS((uint8_t*) message, strlen(message));
				HAL_Delay(1);

			} else if (receivedStr[0] == '7') {	//ARM_CLOSE
				sprintf(message, "7 \r\n");
				CDC_Transmit_FS((uint8_t*) message, strlen(message));
				HAL_Delay(1);

				__HAL_TIM_SET_COMPARE(servo[GRIPPER].TIMER,
						servo[GRIPPER].CHANNEL, GRIPPER_ACTIVE);

				HAL_Delay(500);
				sprintf(message, "-7 \r\n");
				CDC_Transmit_FS((uint8_t*) message, strlen(message));
				HAL_Delay(1);

			} else if (receivedStr[0] == '8') {	//ARM_OPEN
				sprintf(message, "8 \r\n");
				CDC_Transmit_FS((uint8_t*) message, strlen(message));
				HAL_Delay(1);

				__HAL_TIM_SET_COMPARE(servo[GRIPPER].TIMER,
						servo[GRIPPER].CHANNEL, GRIPPER_DEFAULT);

				HAL_Delay(500);
				sprintf(message, "-8 \r\n");
				CDC_Transmit_FS((uint8_t*) message, strlen(message));
				HAL_Delay(1);

//			} else if (receivedStr[0] == '9') {	//HALF_DESCALATE: DON'T EVER use this as the 1st command after startup
//				__HAL_TIM_SET_COMPARE(motor[LEAD].TIMER, motor[LEAD].CHANNEL,
//						LEAD_SPEED);
//				HAL_Delay(1000);
//				__HAL_TIM_SET_COMPARE(motor[LEAD].TIMER, motor[LEAD].CHANNEL,
//						0);
//			} else if (receivedStr[0] == 'D') {
//				for (uint8_t i = 0; i < 2; i++) {
//					sprintf(message, "Dir%d: %d PWM%d: %d SER%d: %d\n\r", i,
//							motor[i].DIR, i, motor[i].PWM, i, servo[i].US);
//					CDC_Transmit_FS((uint8_t*) message, strlen(message));
//					HAL_Delay(1);
//				}
			}

//			sprintf(message, "%c \r\n", receivedStr[0]);
//			CDC_Transmit_FS((uint8_t*) message, strlen(message));
//			HAL_Delay(1);

//			for (uint8_t i = 0; i < 2; i++) {
//				sprintf(message, "Dir%d: %d PWM%d: %d SER%d: %d\n\r", i,
//						motor[i].DIR, i, motor[i].PWM, i, servo[i].US);
//				CDC_Transmit_FS((uint8_t*) message, strlen(message));
//				HAL_Delay(1);

//				HAL_GPIO_WritePin(motor[i].PORT, motor[i].PIN, motor[i].DIR);//<-MOTORS HERE
//				__HAL_TIM_SET_COMPARE(motor[i].TIMER, motor[i].CHANNEL,
//						motor[i].PWM);
//				__HAL_TIM_SET_COMPARE(servo[i].TIMER, servo[i].CHANNEL,
//						servo[i].US);
			}

			memset(buffer, '\0', BUFFER_LENGTH);

//		} else if (current_time - lastSOL >= SOL_INTERVAL) {
//			CDC_Transmit_FS((uint8_t*) "SOL\n\r", strlen("SOL\n\r"));
//			lastSOL = current_time;
//		}

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
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 25;
	RCC_OscInitStruct.PLL.PLLN = 144;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 3;
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

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
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
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 71;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 20000;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);

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
	htim4.Init.Prescaler = 71;
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
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, MOT0_Dir_Pin | MOT1_Dir_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : LED_Pin */
	GPIO_InitStruct.Pin = LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LS0_Pin LS1_Pin LS2_Pin LS3_Pin */
	GPIO_InitStruct.Pin = LS0_Pin | LS1_Pin | LS2_Pin | LS3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : MOT0_Dir_Pin MOT1_Dir_Pin */
	GPIO_InitStruct.Pin = MOT0_Dir_Pin | MOT1_Dir_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);

	HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI1_IRQn);

	HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI2_IRQn);

	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (errorFlag) {
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == LS0_PIN || GPIO_Pin == LS1_PIN) {			//for lead
		if (leadExti == ENABLED) {
			leadExti = DISABLED;//disabling the interrupt to prevent bouncing
//			stopMotor(LEAD);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
		}
	} else if (GPIO_Pin == LS2_PIN || GPIO_Pin == LS3_PIN) {	//for rack
		if (rackExti == ENABLED) {
			rackExti = DISABLED;//disabling the interrupt to prevent bouncing
//			stopMotor(RACK);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
		}
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
