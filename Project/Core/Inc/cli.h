/*
 * cli.h
 *
 *  Created on: Oct 6, 2020
 *      Author: ammar
 */
#ifndef __CLI_H
#define __CLI_H

#include "main.h"

#define MAX_RX 100	//Max rx buffer size

//All the uint8_t * strings used in CLI
#define NEW_LINE_STR (" \r\n\r\n")
#define PROMPT_STR ("UART3-CLI > ")
#define CMD_LED_ON_STR ("led on")
#define CMD_LED_OFF_STR ("led off")
#define CMD_LED_STATE_STR ("led state")
#define CMD_LED_PERIOD_STR ("led period ")
#define CMD_HELP_STR ("help")
#define CMD_TIMING_TEST_STR ("run time test")
#define MSG_LED_ON_STR ( "Turning ON the LED...\r\n\r\n")
#define MSG_LED_OFF_STR ("Turning OFF the LED...\r\n\r\n")
#define MSG_LED_STATE_ON_STR ("!!! The LED state is ON !!!\r\n\r\n")
#define MSG_LED_STATE_OFF_STR ("!!! The LED state is OFF !!!\r\n\r\n")
#define MSG_NEW_PERIOD_STR ("!!! The new LED Period is: ")

#define MSG_HELP_STR ( \
		"HELP MENU:\r\n" \
		"Commands           |    Description\r\n" \
		"-------------------|------------------------\r\n" \
		"''                 |    \r\n" \
		"'help'             |    Show HELP menu\r\n\r\n")

#define MSG_ERROR_STR ("ERROR! Unknown command.\r\nType 'help' for more info\r\n\r\n")

//ANSI Escape codes for terminal manipulation
#define CLEAR_SCREEN "\x1b[2J"
#define SET_CURSOR(X,Y) "\x1b["#X";" #Y "H"
#define SAVE_CURSOR "\x1b[s"
#define RESTORE_CURSOR "\x1b[u"
#define SET_REGION(X) "\x1b["#X";r"
#define CLEAR_STATUS(X) "\x1b["#X";0H\x1b[0K\x1b[1J"



char rxBuffer;				//One byte recieve buffer
char rxLineBuffer[MAX_RX];	//Buffer to store entire command line
volatile uint8_t buffCount;		//Counter for rxLineBuffer
char s_buff[10];			//String when coverting integer to string...
							//..to display in terminal window

void Cli_init(UART_HandleTypeDef * huart);
void Execute_Cmd(UART_HandleTypeDef * huart, uint16_t * cliMessage);
void printStringBlock(UART_HandleTypeDef * huart, const char * message);


#endif
