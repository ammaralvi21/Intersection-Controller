/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

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
TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart3;

/* Definitions for Traffic_Lights */
osThreadId_t Traffic_LightsHandle;
const osThreadAttr_t Traffic_Lights_attributes = {
  .name = "Traffic_Lights",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 128 * 4
};
/* Definitions for Rx_CLI */
osThreadId_t Rx_CLIHandle;
const osThreadAttr_t Rx_CLI_attributes = {
  .name = "Rx_CLI",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for Status_CLI */
osThreadId_t Status_CLIHandle;
const osThreadAttr_t Status_CLI_attributes = {
  .name = "Status_CLI",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = 128 * 4
};
/* Definitions for CLI_Queue */
osMessageQueueId_t CLI_QueueHandle;
const osMessageQueueAttr_t CLI_Queue_attributes = {
  .name = "CLI_Queue"
};
/* Definitions for Status_Queue */
osMessageQueueId_t Status_QueueHandle;
const osMessageQueueAttr_t Status_Queue_attributes = {
  .name = "Status_Queue"
};
/* Definitions for CLI_Status_Update */
osEventFlagsId_t CLI_Status_UpdateHandle;
const osEventFlagsAttr_t CLI_Status_Update_attributes = {
  .name = "CLI_Status_Update"
};
/* Definitions for New_CMD_Rx */
osEventFlagsId_t New_CMD_RxHandle;
const osEventFlagsAttr_t New_CMD_Rx_attributes = {
  .name = "New_CMD_Rx"
};
/* USER CODE BEGIN PV */
const uint32_t CLI_STATUS_FLAG = 0x0001U;
const uint32_t NEW_CMD_FLAG = 0x0002U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM1_Init(void);
void Traffic_Lights_Task(void *argument);
void Rx_CLI_Task(void *argument);
void Status_CLI_Task(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  Cli_init(&huart3);
  HAL_TIM_Base_Start(&htim1);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of CLI_Queue */
  CLI_QueueHandle = osMessageQueueNew (16, sizeof(CMD_MSG_QUEUE_t), &CLI_Queue_attributes);

  /* creation of Status_Queue */
  Status_QueueHandle = osMessageQueueNew (2, sizeof(Status_MSG_QUEUE_t), &Status_Queue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Traffic_Lights */
  Traffic_LightsHandle = osThreadNew(Traffic_Lights_Task, NULL, &Traffic_Lights_attributes);

  /* creation of Rx_CLI */
  Rx_CLIHandle = osThreadNew(Rx_CLI_Task, NULL, &Rx_CLI_attributes);

  /* creation of Status_CLI */
  Status_CLIHandle = osThreadNew(Status_CLI_Task, NULL, &Status_CLI_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the event(s) */
  /* creation of CLI_Status_Update */
  CLI_Status_UpdateHandle = osEventFlagsNew(&CLI_Status_Update_attributes);

  /* creation of New_CMD_Rx */
  New_CMD_RxHandle = osEventFlagsNew(&New_CMD_Rx_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV8;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV16;

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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 99;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 10000;
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
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 101;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
 // sConfigOC.Pulse = 50;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
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
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA6 PA7 PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void Execute_Cmd(UART_HandleTypeDef * huart, CMD_MSG_QUEUE_t * cliMessage)
{

	  printStringBlock(huart, NEW_LINE_STR);
	  char * arg_substr = NULL;
	  char * rxCommand = strstrip(rxLineBuffer);
	  // The following if statements use strcmp to compare rxLineBuffer string with
	  // ..a specific command string. If equal execute that command.
	  if (strcmp(rxCommand, CMD_FSM_STR) == 0)
	  {
		    cliMessage->code = CMD_FSM_CODE;
			if(osMessageQueuePut(CLI_QueueHandle, cliMessage, 1U, 0U)!= osOK)
			{
			  Error_Handler();
			}
		  osEventFlagsSet(New_CMD_RxHandle, NEW_CMD_FLAG);
		  printStringBlock(huart, MSG_FSM_STR);
	  }
	  else if (strcmp(rxCommand, CMD_SCM_STR) == 0)
	  {
		  cliMessage->code = CMD_SCM_CODE;
	    	if(osMessageQueuePut(CLI_QueueHandle, cliMessage, 1U, 0U)!= osOK)
	        {
	          Error_Handler();
	        }
		  osEventFlagsSet(New_CMD_RxHandle, NEW_CMD_FLAG);
		  printStringBlock(huart, MSG_SCM_STR);
	  }
	  else if ((arg_substr = strstr(rxCommand, CMD_ATM_STR)) != NULL )
	  {

		  arg_substr += strlen(CMD_ATM_STR);
		  uint8_t arg_int = atoi(arg_substr);
		  if((arg_int >= 1) && (arg_int <= 100))
		  {
			  cliMessage->code = CMD_ATM_CODE;
			  cliMessage->multiplier = arg_int;
			  if(osMessageQueuePut(CLI_QueueHandle, cliMessage, 1U, 0U)!= osOK)
			  {
				  Error_Handler();
			  }
			  osEventFlagsSet(New_CMD_RxHandle, NEW_CMD_FLAG);
			  printStringBlock(huart, MSG_ATM_STR);

			  printStringBlock(huart, arg_substr);
			  printStringBlock(huart, " !!");
			  printStringBlock(huart, NEW_LINE_STR);
			  arg_substr = NULL;
		  }
		  else
		  {
			  printStringBlock(huart, MSG_ATM_ERR_STR);

		  }
	  }
	  else if (strcmp(rxCommand, CMD_HELP_STR) == 0)
	  {
		  printStringBlock(huart, MSG_HELP_STR);
	  }
	  else if ((buffCount != 0) && (*rxCommand != '\0'))		//If the rxLineBuffer is empty, don't print help message
	  {
		  printStringBlock(huart, MSG_CMD_ERR_STR);
		  printStringBlock(huart, rxCommand);
		  printStringBlock(huart, MSG_ASK_HELP_STR);
	  }

	  //Transmit command prompt
	  printStringBlock(huart, PROMPT_STR);

}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_Traffic_Lights_Task */
/**
  * @brief  Function implementing the Traffic_Lights thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Traffic_Lights_Task */
void Traffic_Lights_Task(void *argument)
{
  /* USER CODE BEGIN 5 */
	 CMD_MSG_QUEUE_t cliMessage;
	 Status_MSG_QUEUE_t statusMessage;
	volatile uint16_t atmMultiplier = 1;
	osStatus_t status;
	//osStatus_t temp;
	volatile LightScmState currScmState = Primary_G_WK_State;

	 Primary_Red(LIGHT_OFF);
	 Primary_Yellow(LIGHT_OFF);
	 Primary_Green(LIGHT_OFF);
	 Primary_Walk(LIGHT_OFF);

	 Secondary_Red(LIGHT_OFF);
	 Secondary_Yellow(LIGHT_OFF);
	 Secondary_Green(LIGHT_OFF);
	 Secondary_Walk(LIGHT_OFF);

	//
	 HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	 HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_3);


	//Send a status message straight away
	statusMessage.code = CMD_SCM_CODE;
	statusMessage.multiplier = 1;
	statusMessage.LightState = currScmState;
	if(osMessageQueuePut(Status_QueueHandle, &statusMessage, 1U, 0U)!= osOK)
	{
		Error_Handler();
	}
	/* Infinite loop */
	for(;;)
	{

		//check for messages but do not block.
		status = osMessageQueueGet(CLI_QueueHandle, &cliMessage, NULL, 0U );
		if(status == osOK)
		{
			osEventFlagsClear(New_CMD_RxHandle,NEW_CMD_FLAG);
			//This means a message has been received
			if(cliMessage.code == CMD_FSM_CODE)
			{
				Primary_Yellow(LIGHT_OFF);
				Primary_Green(LIGHT_OFF);
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2, 10001);
				Secondary_Yellow(LIGHT_OFF);
				Secondary_Green(LIGHT_OFF);
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3, 10001);

				currScmState = Fail_Safe;
				atmMultiplier = 1;
			}
			else if(cliMessage.code == CMD_ATM_CODE)
			{
				currScmState = Primary_G_WK_State;
				atmMultiplier = cliMessage.multiplier;

			}
			else if((cliMessage.code == CMD_SCM_CODE))
			{
				currScmState = Primary_G_WK_State;
				atmMultiplier = 1;
			}

			statusMessage.code = cliMessage.code;
			statusMessage.multiplier = atmMultiplier;

		}

		statusMessage.LightState = currScmState;

		if (osMessageQueueGetCount(Status_QueueHandle) < 2)
		{
			if(osMessageQueuePut(Status_QueueHandle, &statusMessage, 1U, 0U) != osOK)
			{
				Error_Handler();
			}
		}

		switch (currScmState)
		{
		case Primary_G_WK_State :

			 Primary_G_WK_EventHandler();

			osEventFlagsWait(New_CMD_RxHandle, NEW_CMD_FLAG ,osFlagsWaitAny, (uint32_t) (70500/atmMultiplier));
			currScmState = Primary_G_WW_State;

			break;

		case Primary_G_WW_State :

			Primary_G_WW_EventHandler();

			osEventFlagsWait(New_CMD_RxHandle, NEW_CMD_FLAG ,osFlagsWaitAny, (uint32_t) (14000/atmMultiplier));
			currScmState = Primary_Y_DW_State;
			break;

		case Primary_Y_DW_State :

			Primary_Y_DW_EventHandler();

			osEventFlagsWait(New_CMD_RxHandle, NEW_CMD_FLAG ,osFlagsWaitAny, (uint32_t) (3500/atmMultiplier));

			currScmState = Primary_R_DW_State;
			break;

		case Primary_R_DW_State :

			Primary_R_DW_EventHandler();

			osEventFlagsWait(New_CMD_RxHandle, NEW_CMD_FLAG ,osFlagsWaitAny, (uint32_t) (6000/atmMultiplier));

			currScmState = Secondary_G_WK_State;
			break;

		case Secondary_G_WK_State :

			Secondary_G_WK_EventHandler();

			osEventFlagsWait(New_CMD_RxHandle, NEW_CMD_FLAG ,osFlagsWaitAny, (uint32_t) (20000/atmMultiplier));

			currScmState = Secondary_G_WW_State;
			break;

		case Secondary_G_WW_State :

			Secondary_G_WW_EventHandler();

			osEventFlagsWait(New_CMD_RxHandle, NEW_CMD_FLAG ,osFlagsWaitAny, (uint32_t) (8000/atmMultiplier));

			currScmState = Secondary_Y_WW_State;
			break;

		case Secondary_Y_WW_State :

			Secondary_Y_WW_EventHandler();

			osEventFlagsWait(New_CMD_RxHandle, NEW_CMD_FLAG ,osFlagsWaitAny, (uint32_t) (3500/atmMultiplier));
			currScmState = Primary_G_WK_State;
			break;

		case Fail_Safe:

			Primary_Red(LIGHT_ON);
			Secondary_Red(LIGHT_ON);
			osDelay(1500);
			Primary_Red(LIGHT_OFF);
			Secondary_Red(LIGHT_OFF);
			osDelay(500);

			break;
		}


		//osDelay(period);
	}
  osThreadTerminate(NULL);
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Rx_CLI_Task */
/**
* @brief Function implementing the Rx_CLI thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Rx_CLI_Task */
void Rx_CLI_Task(void *argument)
{
  /* USER CODE BEGIN Rx_CLI_Task */
	CMD_MSG_QUEUE_t cliMessage;
  /* Infinite loop */
  for(;;)
  {
	  if (HAL_UART_Receive(&huart3, (uint8_t *) &rxBuffer, 1, 300) == HAL_OK)
	  {
		if(rxBuffer == '\r')		//If newline was entered
		{

			Execute_Cmd(&huart3, &cliMessage);		//Execute the command line

			//Reset the command line buffer to null
			memset(rxLineBuffer, '\0', sizeof(rxLineBuffer));
			buffCount = 0;


		}
		//If Backspace key was pressed..
		else if ((rxBuffer == '\b') ||	(rxBuffer == 127))
		{
			//Check if buffer count is greater than 0
			 if (buffCount > 0)
			 {
				  buffCount--;								//Decrement command line buffer count
				  rxLineBuffer[buffCount] = '\0';			//Set last character to null indicating
															//..an erase from backspace
				  printStringBlock(&huart3, &rxBuffer);		//Print the output to terminal
			 }

		}
		//If command line buffer count is less than max, and the recieved character is
		//.. within the ascii alphabet scheme.
		else if((buffCount < MAX_RX) && (rxBuffer >= 32) && (rxBuffer <= 126))
		{
			//Push the recieved character into command line buffer
			rxLineBuffer[buffCount] = rxBuffer;
			buffCount++;	//Increment buffer count
			printStringBlock(&huart3, &rxBuffer);	//Output that recieved character to terminal
		}

	  }
	  else
	  {
		  osEventFlagsWait(CLI_Status_UpdateHandle, CLI_STATUS_FLAG, osFlagsWaitAny, 50);
		  //osDelay(50);
	  }
  }
  /* USER CODE END Rx_CLI_Task */
}

/* USER CODE BEGIN Header_Status_CLI_Task */
/**
* @brief Function implementing the Status_CLI thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Status_CLI_Task */
void Status_CLI_Task(void *argument)
{
  /* USER CODE BEGIN Status_CLI_Task */
	Status_MSG_QUEUE_t statusMessage;
	Status_MSG_QUEUE_t displayMSG;
	osStatus_t status;
	char * Init_Region = SAVE_CURSOR CLEAR_STATUS(10) SET_REGION(11) SET_CURSOR(0,0);
	char atmMultiplier[10];
	memset(atmMultiplier, '\0', sizeof(atmMultiplier));

	/* Infinite loop */


	for(;;)
	{
		 status = osMessageQueueGet(Status_QueueHandle, &statusMessage, NULL, 0U );
		 if(status == osOK)
		 {
			 displayMSG = statusMessage;
		 }
		itoa(displayMSG.multiplier, atmMultiplier, 10);
		printStringBlock(&huart3, Init_Region);

		switch (displayMSG.code)
		{
		case CMD_ATM_CODE :
			printStringBlock(&huart3, SET_CURSOR(6,2) "ACCELERATED TEST MODE " SET_CURSOR(7,12) "x");
			printStringBlock(&huart3, atmMultiplier);
			break;

		case CMD_FSM_CODE :
			printStringBlock(&huart3, SET_CURSOR(6,2) "FAIL SAFE MODE");

			break;

		case CMD_SCM_CODE :
			printStringBlock(&huart3, SET_CURSOR(6,2) "STATIC CYCLE MODE");
			break;

		}

		switch (displayMSG.LightState)
		{
			case Primary_G_WK_State:
				Primary_G_WK_StatusUpdate();
			break;

			case Primary_G_WW_State:
				Primary_G_WW_StatusUpdate();
			break;

			case Primary_Y_DW_State:
				Primary_Y_DW_StatusUpdate();
			break;

			case Primary_R_DW_State:
				Primary_R_DW_StatusUpdate();
			break;

			case Secondary_G_WK_State:
				Secondary_G_WK_StatusUpdate();
			break;

			case Secondary_G_WW_State:
				Secondary_G_WW_StatusUpdate();
			break;

			case Secondary_Y_WW_State:
				Secondary_Y_WW_StatusUpdate();
			break;

			case Fail_Safe:
				Fail_Safe_StatusUpdate();
			break;

		}

		printStringBlock(&huart3, RESTORE_CURSOR);
		memset(atmMultiplier, '\0', sizeof(atmMultiplier));

		osEventFlagsSet(CLI_Status_UpdateHandle, CLI_STATUS_FLAG);

	}
  /* USER CODE END Status_CLI_Task */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
