/*
 * controller.h
 *
 *  Created on: Apr 13, 2024
 *      Author: joedo
 */

#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include "FreeRTOS.h"
#include "queue.h"

#define CONTROLLER_GAIN_P 0.05
#define CONTROLLER_GAIN_I 0.01
#define CONTROLLER_GAIN_D 0.0
#define ALTITUDE_TARGET_FT 20000
#define CONTROLLER_MAX_EXTENSION 1.0
#define CONTROLLER_MIN_EXTENSION 0.0

#define MIN_CONTROLLER_Hz 1
#define CONTROLLER_DELAY_TICKS 1000 / MIN_CONTROLLER_Hz / portTICK_RATE_MS

typedef struct {
	float controller_term_P;
	float controller_term_I;
	float controller_term_D;
	float target_altitude;
	float last_error;
	float error;
	float target_extension;
} controller_t;

extern QueueHandle_t apogeeQueue;

void controllerInit();

void controlTask(void *argument);


#endif /* CONTROLLER_H_ */
