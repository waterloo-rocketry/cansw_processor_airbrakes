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
#include "event_groups.h"

#define RESET_FILTER_FLAG 0x1

typedef struct
{
	uint32_t deltaTimems;
	FusionVector gyroscope;
	FusionVector accelerometer;
	FusionVector magnetometer;
} rawIMUPacked;

extern QueueHandle_t IMUDataHandle;
extern EventGroupHandle_t calibrationEventHandle;

void stateEstTask(void *arguments);
void state_est_init();

#endif /* STATE_ESTIMATION_H_ */
