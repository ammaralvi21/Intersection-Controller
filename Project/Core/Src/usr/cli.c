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

char *strstrip(char *s)
{
    size_t size;
    char *end;

    size = strlen(s);

    if (!size)
        return s;

    end = s + size - 1;
    while (end >= s && isspace(*end))
        end--;
    *(end + 1) = '\0';

    while (*s && isspace(*s))
        s++;

    return s;
}




