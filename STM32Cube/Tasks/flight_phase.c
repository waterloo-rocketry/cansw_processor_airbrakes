/*
 * flight_phase.c
 *
 *  Created on: May 17, 2024
 *      Author: joedo
 */

#include "flight_phase.h"

#define BOOST_TIME_MS 15000 //time from inj valve open to motor bunout
#define RECOVERY_DELAY_MS 30000 //time from inj valve open to recovery deployment
enum FLIGHT_PHASE flight_state = PHASE_PRELAUNCH;
uint32_t injOpenTime;
uint32_t deploymentTime;

EventGroupHandle_t flightPhaseEventsHandle = NULL;
#define INJ_OPEN (xEventGroupGetBits(flightPhaseEventsHandle) && INJ_OPEN_BIT)
#define RECOVERY_DEPLOYMENT (xEventGroupGetBits(flightPhaseEventsHandle) && RECOVERY_DEPLOYMENT_BIT)


uint32_t millis() //TODO: replace with actual millis() function
{
#warning fake millis() function, replace with actual timing function
	return 0;
}

void flight_phase_task_init()
{
	flightPhaseEventsHandle = xEventGroupCreate();
	xEventGroupClearBits(flightPhaseEventsHandle, 0xFFFF); //clear out the enitire group since API doesn't specify if values are initialized to 0
}

void flightPhaseTask(void * argument)
{
	while(1)
	{
		if(flight_state == PHASE_PRELAUNCH && INJ_OPEN)
		{
			injOpenTime = millis();
			flight_state = 	PHASE_BOOST;
		}

		else if(flight_state == PHASE_BOOST && (millis() - injOpenTime) > BOOST_TIME_MS)
		{
			flight_state = PHASE_COAST;
		}

		else if((flight_state == PHASE_BOOST && (millis() - injOpenTime) > RECOVERY_DELAY_MS) || RECOVERY_DEPLOYMENT)
		{
			flight_state = PHASE_DESCENT;
		}
		osDelayUntil(10);
	}
}

bool extensionAllowed()
{
	return flight_state == PHASE_COAST;
}
