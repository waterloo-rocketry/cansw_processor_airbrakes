/*
 * flight_phase.h
 *
 *  Created on: Apr 13, 2024
 *      Author: joedo
 */

#ifndef FLIGHT_PHASE_H_
#define FLIGHT_PHASE_H_

#include <stdbool.h>

#include "FreeRTOS.h"
#include "event_groups.h"

typedef enum {
	PHASE_PRELAUNCH = 0,
	PHASE_BOOST,
	PHASE_COAST,
	PHASE_DESCENT,
	PHASE_LANDED
} FLIGHT_PHASE;

extern EventGroupHandle_t flightPhaseEventsHandle;
#define INJ_OPEN_BIT 0x1
#define RECOVERY_DEPLOYMENT_BIT 0x2
#define RECOVERY_MIN_VELOCITY 30.0


bool extensionAllowed();

void flightPhaseTask(void * argument);
bool flightPhaseInit();

#endif /* FLIGHT_PHASE_H_ */
