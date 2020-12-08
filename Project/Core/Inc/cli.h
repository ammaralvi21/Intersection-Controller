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
#define CMD_FSM_CODE ('f')
#define CMD_SCM_CODE ('s')
#define CMD_ATM_CODE ('a')
#define CMD_HELP_STR ("help")
#define MSG_FSM_STR ("!! Switched to Failsafe mode !!\r\n\r\n")
#define MSG_SCM_STR ("!! Switched to Static Cycle mode !!\r\n\r\n")
#define MSG_ATM_STR ("!! Accelerated test mode with multiplication factor: ")
#define MSG_LED_STATE_OFF_STR ("!!! The LED state is OFF !!!\r\n\r\n")
#define MSG_HELP_STR ( \
		"                                 HELP MENU:                              \r\n" \
		"|===================|=====================================================|\r\n" \
		"|{Commands}         |    {Description}                                    |\r\n" \
		"|===================|=====================================================|\r\n" \
		"|'mode fsm'         |    Switch to Failsafe mode                          |\r\n" \
		"|-------------------|-----------------------------------------------------|\r\n" \
		"|'mode scm'         |    Switch to Static Cycle mode                      |\r\n" \
		"|-------------------|-----------------------------------------------------|\r\n" \
		"|'atm [x]'          |    Enter Accelerated test mode with multiplication  |\r\n" \
		"|                   |    factor x where x is between 1 and 100            |\r\n" \
		"|-------------------|-----------------------------------------------------|\r\n" \
		"|'help'             |    Show HELP menu                                   |\r\n" \
        "|-------------------|-----------------------------------------------------|\r\n\r\n" )

#define MSG_ATM_ERR_STR ("ERROR! ATM multiplier should be between 1 and 100\r\n" \
						 "Please try again! Or Type 'help' for more info \r\n\r\n")
#define MSG_CMD_ERR_STR ("ERROR! Invalid command: \"")
#define MSG_ASK_HELP_STR ("\"\r\nType 'help' for more info\r\n\r\n")

//ANSI Escape codes for terminal manipulation
#define CLEAR_SCREEN "\x1b[2J"
#define SET_CURSOR(X,Y) "\x1b["#X";" #Y "H"
#define SAVE_CURSOR "\x1b[s"
#define RESTORE_CURSOR "\x1b[0m\x1b[u"
#define SET_REGION(X) "\x1b["#X";r"
#define CLEAR_STATUS(X) "\x1b["#X";0H\x1b[0K\x1b[1J"

//ANSI Escape SGR codes to display Background color
//.. used to represent traffic light status.
#define BG_RED "\x1b[48;2;255;0;0m R "
#define BG_GREEN "\x1b[48;2;0;255;0m G "
#define BG_YELLOW "\x1b[48;2;255;255;0m Y "
#define BG_BLUE_W "\x1b[48;2;0;0;255m S "
#define BG_BLUE "\x1b[48;2;0;0;255m W "

//Darker color tones to indicate off status
#define BG_NRED "\x1b[48;2;59;0;0m   "
#define BG_NGREEN "\x1b[48;2;0;59;0m   "
#define BG_NYELLOW "\x1b[48;2;48;48;0m   "
#define BG_NBLUE "\x1b[48;2;0;0;48m\x1b[39m\x1b[1m   "


char rxBuffer;				//One byte recieve buffer
char rxLineBuffer[MAX_RX];	//Buffer to store entire command line
volatile uint8_t buffCount;		//Counter for rxLineBuffer
char s_buff[10];			//String when coverting integer to string...
							//..to display in terminal window
void Primary_G_WK_StatusUpdate(void);
void Primary_G_WW_StatusUpdate(void);
void Primary_Y_DW_StatusUpdate(void);
void Primary_R_DW_StatusUpdate(void);
void Secondary_G_WK_StatusUpdate(void);
void Secondary_G_WW_StatusUpdate(void);
void Secondary_Y_WW_StatusUpdate(void);
void Fail_Safe_StatusUpdate();

void Cli_init(UART_HandleTypeDef * huart);
void printStringBlock(UART_HandleTypeDef * huart, const char * message);
char *strstrip(char *s);

#endif
