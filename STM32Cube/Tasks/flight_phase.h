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
#include "stm32h7xx_hal.h"
#include "cmsis_os.h"

typedef enum FLIGHT_PHASE {
	PHASE_PRELAUNCH = 0,
	PHASE_BOOST,
	PHASE_COAST,
	PHASE_DESCENT,
	PHASE_LANDED
};

extern EventGroupHandle_t flightPhaseEventsHandle;
#define INJ_OPEN_BIT 0x1
#define RECOVERY_DEPLOYMENT_BIT 0x2

bool extensionAllowed();
void flightPhaseTask(void * argument);
void flightPhaseInit();

#endif /* FLIGHT_PHASE_H_ */
