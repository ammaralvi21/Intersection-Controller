/*
 * cli.c
 *
 *  Created on: Oct 6, 2020
 *      Author: ammar
 */

#include "main.h"


void Cli_init(UART_HandleTypeDef * huart)
{
	printStringBlock(huart, CLEAR_SCREEN);
	printStringBlock(huart, SET_CURSOR(10,0));
	printStringBlock(huart, PROMPT_STR);
	//Clearing global counter (int and string) buffer
	memset(s_buff, '\0', sizeof(s_buff));
	memset(rxLineBuffer, '\0', sizeof(rxLineBuffer));
	buffCount = 0;
}

//Interprets and Executes the command line buffer
void Execute_Cmd(UART_HandleTypeDef * huart, uint16_t * cliMessage)
{

	  printStringBlock(huart, NEW_LINE_STR);
	  char * period_substr = NULL;
	  period_substr = strstr(rxLineBuffer, CMD_LED_PERIOD_STR);
	  // The following if statements use strcmp to compare rxLineBuffer string with
	  // ..a specific command string. If equal execute that command.
	  if (strcmp(rxLineBuffer, CMD_LED_ON_STR) == 0)
	  {
		  printStringBlock(huart, MSG_LED_ON_STR);
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
	  }
	  else if (strcmp(rxLineBuffer, CMD_LED_OFF_STR) == 0)
	  {
		  printStringBlock(huart, MSG_LED_OFF_STR);
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

	  }
	  else if (strcmp(rxLineBuffer, CMD_LED_STATE_STR) == 0)
	  {
		  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5))
		  {
			  printStringBlock(huart, MSG_LED_STATE_ON_STR);
		  }
		  else
		  {
			  printStringBlock(huart, MSG_LED_STATE_OFF_STR);
		  }
	  }
	  else if (strcmp(rxLineBuffer, CMD_LED_OFF_STR) == 0)
	  {
		  printStringBlock(huart, MSG_LED_OFF_STR);
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

	  }
	  else if (period_substr != NULL)
	  {
		  period_substr += strlen(CMD_LED_PERIOD_STR);
		  *cliMessage = atoi(period_substr);
		  printStringBlock(huart, MSG_NEW_PERIOD_STR);
		  printStringBlock(huart, period_substr);
		  printStringBlock(huart, NEW_LINE_STR);
		  period_substr = NULL;
	  }
	  else if (strcmp(rxLineBuffer, CMD_HELP_STR) == 0)
	  {

		  printStringBlock(huart, MSG_HELP_STR);
	  }
	  else if (buffCount != 0)		//If the rxLineBuffer is empty, don't print help message
	  {
		  printStringBlock(huart, MSG_ERROR_STR);
	  }

	  //Transmit command prompt
	  printStringBlock(huart, PROMPT_STR);

}


//Print input string in blocking mode

void printStringBlock(UART_HandleTypeDef * huart,const char * message)
{

	HAL_StatusTypeDef st;

	//Make sure the previous uart tx is complete
	while(huart->gState == HAL_UART_STATE_BUSY_TX) {}

	//UART TX in blocking mode
	st = HAL_UART_Transmit(huart,(uint8_t *) message, strlen(message), 2000);
	/* Process Unlocked */

	//If unsuccessfull TX then error
	if(st != HAL_OK)
	{
		Error_Handler();
	}


}




