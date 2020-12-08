/*
 * lights.h
 *
 *  Created on: Dec 1, 2020
 *      Author: ammar
 */

#ifndef INC_LIGHTS_H_
#define INC_LIGHTS_H_

#include "main.h"


typedef enum
{
    Primary_G_WK_State,
	Primary_G_WW_State,
	Primary_Y_DW_State,
	Primary_R_DW_State,
	Secondary_G_WK_State,
	Secondary_G_WW_State,
	Secondary_Y_WW_State,
	Fail_Safe,
} LightScmState;


void Primary_G_WK_EventHandler(void);
void Primary_G_WW_EventHandler(void);
void Primary_Y_DW_EventHandler(void);
void Primary_R_DW_EventHandler(void);
void Secondary_G_WK_EventHandler(void);
void Secondary_G_WW_EventHandler(void);
void Secondary_Y_WW_EventHandler(void);

void Primary_Red(GPIO_PinState state);
void Primary_Yellow(GPIO_PinState state);
void Primary_Green(GPIO_PinState state);
void Primary_Walk(GPIO_PinState state);

void Secondary_Red(GPIO_PinState state);
void Secondary_Yellow(GPIO_PinState state);
void Secondary_Green(GPIO_PinState state);
void Secondary_Walk(GPIO_PinState state);



#endif /* INC_LIGHTS_H_ */
