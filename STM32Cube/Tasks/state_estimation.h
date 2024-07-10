/*
 * state_estimation.h
 *
 *  Created on: Apr 13, 2024
 *      Author: joedo
 */

#ifndef STATE_ESTIMATION_H_
#define STATE_ESTIMATION_H_

#include "stm32h7xx_hal.h"
#include "Fusion.h"
#include "FreeRTOS.h"
#include "queue.h"

typedef struct
{
	float TimeS; //timestamp in s
	FusionVector gyroscope; //dps
	FusionVector accelerometer; //g
	FusionVector magnetometer; //Gauss
} rawIMUPacked;

extern QueueHandle_t IMUDataHandle;

void stateEstTask(void *arguments);
bool state_est_init();

#endif /* STATE_ESTIMATION_H_ */
