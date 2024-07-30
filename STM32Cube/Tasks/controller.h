/*
 * controller.h
 *
 *  Created on: Apr 13, 2024
 *      Author: joedo
 */

#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include <stdbool.h>

#include "FreeRTOS.h"
#include "queue.h"

#define DEFAULT_TARGET_ALTITUDE_M 6660.0f
#define CONTROLLER_MAX_EXTENSION 1.0
#define CONTROLLER_MIN_EXTENSION 0.0

#define MIN_CONTROLLER_Hz 1
#define CONTROLLER_DELAY_TICKS 1000 / MIN_CONTROLLER_Hz / portTICK_RATE_MS

extern QueueHandle_t apogeeQueue;
extern QueueHandle_t targetQueue;

bool controllerInit();

void controlTask(void* argument);

#endif /* CONTROLLER_H_ */
