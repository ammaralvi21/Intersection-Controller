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
#define PROMPT_STR ("ItC-CLI > ")
#define CMD_FSM_STR ("mode fsm")
#define CMD_SCM_STR ("mode scm")
#define CMD_ATM_STR ("atm ")
#define CMD_HELP_STR ("help")
#define MSG_FSM_STR ("!! Switched to Failsafe mode !!\r\n\r\n")
#define MSG_SCM_STR ("!! Switched to Static Cycle mode !!\r\n\r\n")
#define MSG_ATM_STR ("!! Accelerated test mode with multiplication factor: ")
#define MSG_LED_STATE_OFF_STR ("!!! The LED state is OFF !!!\r\n\r\n")
#define MSG_HELP_STR ( \
		"HELP MENU:\r\n" \
		"Commands           |    Description\r\n" \
		"-------------------|------------------------\r\n" \
		"'mode fsm'         |    \r\n" \
		"'mode scm'         |    \r\n" \
		"'atm [x]'          |    \r\n" \
		"'help'             |    Show HELP menu\r\n\r\n")

#define MSG_CMD_ERR_STR ("ERROR! Invalid command: \"")
#define MSG_ASK_HELP_STR ("\"\r\nType 'help' for more info\r\n\r\n")

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
void printStringBlock(UART_HandleTypeDef * huart, const char * message);
char *strstrip(char *s);

#endif
