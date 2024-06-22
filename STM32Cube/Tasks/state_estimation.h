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
#include "semphr.h"

typedef struct
{
	uint32_t deltaTimems;
	FusionVector gyroscope;
	FusionVector accelerometer;
	FusionVector magnetometer;
} rawIMUPacked;

extern QueueHandle_t IMUDataHandle;

void stateEstTask(void *arguments);

#endif /* STATE_ESTIMATION_H_ */
