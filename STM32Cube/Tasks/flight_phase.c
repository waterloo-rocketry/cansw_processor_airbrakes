/*
 * flight_phase.c
 *
 *  Created on: May 17, 2024
 *      Author: joedo
 */

#include "flight_phase.h"
#include "millis.h"
#include "printf.h"

#define BOOST_TIME_MS 10000 //time from inj valve open to motor bunout
#define RECOVERY_DELAY_MS 15000 //time from inj valve open to recovery deployment

enum FLIGHT_PHASE flight_state = PHASE_PRELAUNCH;
float injOpenTime;
float deploymentTime;

EventGroupHandle_t flightPhaseEventsHandle = NULL;
#define INJ_OPEN (xEventGroupGetBits(flightPhaseEventsHandle) & INJ_OPEN_BIT)
#define RECOVERY_DEPLOYMENT (xEventGroupGetBits(flightPhaseEventsHandle) & RECOVERY_DEPLOYMENT_BIT)


void flightPhaseInit()
{
	flightPhaseEventsHandle = xEventGroupCreate();
	xEventGroupClearBits(flightPhaseEventsHandle, 0xFFFF); //clear out the enitire group since API doesn't specify if values are initialized to 0
}

void flightPhaseTask(void * argument)
{
	while(1)
	{
		float time = millis_();


		if(flight_state == PHASE_PRELAUNCH && INJ_OPEN)
		{
			injOpenTime = time;
			flight_state = 	PHASE_BOOST;
		}

		else if(flight_state == PHASE_BOOST && (time - injOpenTime) > BOOST_TIME_MS)
		{
			flight_state = PHASE_COAST;
		}

		else if((flight_state == PHASE_COAST && (time - injOpenTime) > RECOVERY_DELAY_MS) || RECOVERY_DEPLOYMENT)
		{
			flight_state = PHASE_DESCENT;
		}
		//printf_("[%f] flight phase: %d\n", time/1000, flight_state);
		osDelay(10);
	}
}

bool extensionAllowed()
{
	return flight_state == PHASE_COAST;
}
