/*
 * millis.c
 *
 *  Created on: May 22, 2024
 *      Author: sengu
 */

// millis.c
#include "millis.h"
#include "stm32h7xx_hal.h"

extern TIM_HandleTypeDef htim2;

float millis_(void) {
	return __HAL_TIM_GET_COUNTER(&htim2)/10.0; //timer is set up to measure in resolution of 0.1ms, we just need up to ms
}
