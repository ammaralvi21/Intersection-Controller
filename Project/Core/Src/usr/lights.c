/*
 * lights.c
 *
 *  Created on: Dec 1, 2020
 *      Author: ammar
 */

#include "main.h"

extern TIM_HandleTypeDef htim1;

void Primary_G_WK_EventHandler(void)
{
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3, 10001);
	Primary_Red(LIGHT_OFF);
	Primary_Yellow(LIGHT_OFF);
	Primary_Green(LIGHT_ON);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2, 0);

	Secondary_Red(LIGHT_ON);
	Secondary_Yellow(LIGHT_OFF);
	Secondary_Green(LIGHT_OFF);

}
void Primary_G_WW_EventHandler(void)
{
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3, 10001);
	Primary_Red(LIGHT_OFF);
	Primary_Yellow(LIGHT_OFF);
	Primary_Green(LIGHT_ON);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2, 5000);

	Secondary_Red(LIGHT_ON);
	Secondary_Yellow(LIGHT_OFF);
	Secondary_Green(LIGHT_OFF);

}

void Primary_Y_DW_EventHandler(void)
{
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2, 10001);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3, 10001);

	Primary_Red(LIGHT_OFF);
	Primary_Yellow(LIGHT_ON);
	Primary_Green(LIGHT_OFF);

	Secondary_Red(LIGHT_ON);
	Secondary_Yellow(LIGHT_OFF);
	Secondary_Green(LIGHT_OFF);

}

void Primary_R_DW_EventHandler(void)
{
	Primary_Red(LIGHT_ON);
	Primary_Yellow(LIGHT_OFF);
	Primary_Green(LIGHT_OFF);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2, 10001);

	Secondary_Red(LIGHT_ON);
	Secondary_Yellow(LIGHT_OFF);
	Secondary_Green(LIGHT_OFF);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3, 10001);

}

void Secondary_G_WK_EventHandler(void)
{
	Primary_Red(LIGHT_ON);
	Primary_Yellow(LIGHT_OFF);
	Primary_Green(LIGHT_OFF);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2, 10001);

	Secondary_Red(LIGHT_OFF);
	Secondary_Yellow(LIGHT_OFF);
	Secondary_Green(LIGHT_ON);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3, 0);


}

void Secondary_G_WW_EventHandler(void)
{
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2, 10001);

	Primary_Red(LIGHT_ON);
	Primary_Yellow(LIGHT_OFF);
	Primary_Green(LIGHT_OFF);


	Secondary_Red(LIGHT_OFF);
	Secondary_Yellow(LIGHT_OFF);
	Secondary_Green(LIGHT_ON);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3, 5000);

}

void Secondary_Y_WW_EventHandler(void)
{
	Primary_Red(LIGHT_ON);
	Primary_Yellow(LIGHT_OFF);
	Primary_Green(LIGHT_OFF);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2, 10001);

	Secondary_Red(LIGHT_OFF);
	Secondary_Yellow(LIGHT_ON);
	Secondary_Green(LIGHT_OFF);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3, 5000);

}


void Primary_Red(GPIO_PinState state)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, !state);
}


void Primary_Yellow(GPIO_PinState state)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, !state);
}

void Primary_Green(GPIO_PinState state)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, !state);
}

void Primary_Walk(GPIO_PinState state)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, !state);
}


void Secondary_Red(GPIO_PinState state)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, !state);
}


void Secondary_Yellow(GPIO_PinState state)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, !state);
}

void Secondary_Green(GPIO_PinState state)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, !state);
}

void Secondary_Walk(GPIO_PinState state)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, !state);
}

