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

float millis(void) {
	return __HAL_TIM_GET_COUNTER(&htim2)/10.0; //timer is set up to measure in resolution of 0.1ms, we just need up to ms
}

//DONT CALL THIS, ITS CALLED IN MAIN
int millisInit(){
	if (htim2.State != HAL_TIM_STATE_READY){
		HAL_TIM_Base_Start(&htim2);
		if (htim2.State == HAL_TIM_STATE_READY){
			return 0; //turned timer on
		}
		else{
			return 1; //timer did not turn on
		}
	}
	else{
		return 2; //timer was already started
	}
}

