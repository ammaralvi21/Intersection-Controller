/*
 * lights.c
 *
 *  Created on: Dec 1, 2020
 *      Author: ammar
 */

#include "main.h"


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

