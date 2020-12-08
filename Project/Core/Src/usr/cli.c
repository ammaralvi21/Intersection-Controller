/*
 * cli.c
 *
 *  Created on: Oct 6, 2020
 *      Author: ammar
 */

#include "main.h"

extern UART_HandleTypeDef huart3;

void Primary_G_WK_StatusUpdate(void)
{
	printStringBlock(&huart3, SET_CURSOR(1,2) BG_BLUE \
							  SET_CURSOR(2,2) BG_GREEN \
							  SET_CURSOR(3,2) BG_NYELLOW \
							  SET_CURSOR(4,2) BG_NRED \
							  SET_CURSOR(6,24) BG_NRED \
							  SET_CURSOR(7,24) BG_NYELLOW \
							  SET_CURSOR(8,24) BG_GREEN \
							  SET_CURSOR(9,24) BG_BLUE \
							  SET_CURSOR(9,2) BG_NBLUE \
											  BG_NGREEN \
											  BG_NYELLOW \
											  BG_RED \
							 SET_CURSOR(1,15) BG_RED \
											  BG_NYELLOW \
											  BG_NGREEN \
											  BG_NBLUE);
}
void Primary_G_WW_StatusUpdate(void)
{
	printStringBlock(&huart3, SET_CURSOR(1,2) BG_BLUE_W \
							  SET_CURSOR(2,2) BG_GREEN \
							  SET_CURSOR(3,2) BG_NYELLOW \
							  SET_CURSOR(4,2) BG_NRED \
							  SET_CURSOR(6,24) BG_NRED \
							  SET_CURSOR(7,24) BG_NYELLOW \
							  SET_CURSOR(8,24) BG_GREEN \
							  SET_CURSOR(9,24) BG_BLUE_W \
							  SET_CURSOR(9,2) BG_NBLUE \
											  BG_NGREEN \
											  BG_NYELLOW \
											  BG_RED \
							 SET_CURSOR(1,15) BG_RED \
											  BG_NYELLOW \
											  BG_NGREEN \
											  BG_NBLUE);


}

void Primary_Y_DW_StatusUpdate(void)
{
	printStringBlock(&huart3, SET_CURSOR(1,2) BG_NBLUE \
							  SET_CURSOR(2,2) BG_NGREEN \
							  SET_CURSOR(3,2) BG_YELLOW \
							  SET_CURSOR(4,2) BG_NRED \
							  SET_CURSOR(6,24) BG_NRED \
							  SET_CURSOR(7,24) BG_YELLOW \
							  SET_CURSOR(8,24) BG_NGREEN \
							  SET_CURSOR(9,24) BG_NBLUE \
							  SET_CURSOR(9,2) BG_NBLUE \
											  BG_NGREEN \
											  BG_NYELLOW \
											  BG_RED \
							 SET_CURSOR(1,15) BG_RED \
											  BG_NYELLOW \
											  BG_NGREEN \
											  BG_NBLUE);

}

void Primary_R_DW_StatusUpdate(void)
{
	printStringBlock(&huart3, SET_CURSOR(1,2) BG_NBLUE \
							  SET_CURSOR(2,2) BG_NGREEN \
							  SET_CURSOR(3,2) BG_NYELLOW \
							  SET_CURSOR(4,2) BG_RED \
							  SET_CURSOR(6,24) BG_RED \
							  SET_CURSOR(7,24) BG_NYELLOW \
							  SET_CURSOR(8,24) BG_NGREEN \
							  SET_CURSOR(9,24) BG_NBLUE \
							  SET_CURSOR(9,2) BG_NBLUE \
											  BG_NGREEN \
											  BG_NYELLOW \
											  BG_RED \
							 SET_CURSOR(1,15) BG_RED \
											  BG_NYELLOW \
											  BG_NGREEN \
											  BG_NBLUE);
}

void Secondary_G_WK_StatusUpdate(void)
{
	printStringBlock(&huart3, SET_CURSOR(1,2) BG_NBLUE \
							  SET_CURSOR(2,2) BG_NGREEN \
							  SET_CURSOR(3,2) BG_NYELLOW \
							  SET_CURSOR(4,2) BG_RED \
							  SET_CURSOR(6,24) BG_RED \
							  SET_CURSOR(7,24) BG_NYELLOW \
							  SET_CURSOR(8,24) BG_NGREEN \
							  SET_CURSOR(9,24) BG_NBLUE \
							  SET_CURSOR(9,2) BG_BLUE \
											  BG_GREEN \
											  BG_NYELLOW \
											  BG_NRED \
							 SET_CURSOR(1,15) BG_NRED \
											  BG_NYELLOW \
											  BG_GREEN \
											  BG_BLUE);

}

void Secondary_G_WW_StatusUpdate(void)
{
	printStringBlock(&huart3, SET_CURSOR(1,2) BG_NBLUE \
							  SET_CURSOR(2,2) BG_NGREEN \
							  SET_CURSOR(3,2) BG_NYELLOW \
							  SET_CURSOR(4,2) BG_RED \
							  SET_CURSOR(6,24) BG_RED \
							  SET_CURSOR(7,24) BG_NYELLOW \
							  SET_CURSOR(8,24) BG_NGREEN \
							  SET_CURSOR(9,24) BG_NBLUE \
							  SET_CURSOR(9,2) BG_BLUE_W \
											  BG_GREEN \
											  BG_NYELLOW \
											  BG_NRED \
							 SET_CURSOR(1,15) BG_NRED \
											  BG_NYELLOW \
											  BG_GREEN \
											  BG_BLUE_W);

}

void Secondary_Y_WW_StatusUpdate(void)
{
	printStringBlock(&huart3, SET_CURSOR(1,2) BG_NBLUE \
							  SET_CURSOR(2,2) BG_NGREEN \
							  SET_CURSOR(3,2) BG_NYELLOW \
							  SET_CURSOR(4,2) BG_RED \
							  SET_CURSOR(6,24) BG_RED \
							  SET_CURSOR(7,24) BG_NYELLOW \
							  SET_CURSOR(8,24) BG_NGREEN \
							  SET_CURSOR(9,24) BG_NBLUE \
							  SET_CURSOR(9,2) BG_BLUE_W \
											  BG_NGREEN \
											  BG_YELLOW \
											  BG_NRED \
							 SET_CURSOR(1,15) BG_NRED \
											  BG_YELLOW \
											  BG_NGREEN \
											  BG_BLUE_W);

}

void Fail_Safe_StatusUpdate(void)
{
	printStringBlock(&huart3, SET_CURSOR(1,2) BG_NBLUE \
							  SET_CURSOR(2,2) BG_NGREEN \
							  SET_CURSOR(3,2) BG_NYELLOW \
							  SET_CURSOR(4,2) BG_RED \
							  SET_CURSOR(6,24) BG_RED \
							  SET_CURSOR(7,24) BG_NYELLOW \
							  SET_CURSOR(8,24) BG_NGREEN \
							  SET_CURSOR(9,24) BG_NBLUE \
							  SET_CURSOR(9,2) BG_NBLUE \
											  BG_NGREEN \
											  BG_NYELLOW \
											  BG_RED \
							 SET_CURSOR(1,15) BG_RED \
											  BG_NYELLOW \
											  BG_NGREEN \
											  BG_NBLUE);
}

void Cli_init(UART_HandleTypeDef * huart)
{
	printStringBlock(huart, CLEAR_SCREEN);
	printStringBlock(huart,"\x1b[0m" );
	printStringBlock(huart, SET_CURSOR(11,0));
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




