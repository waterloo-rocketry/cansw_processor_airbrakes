/*
 * flight_phase.c
 *
 *  Created on: May 17, 2024
 *      Author: joedo
 */

#include "flight_phase.h"
#include "millis.h"
#include "printf.h"
#include "log.h"
#include "otits.h"

#define BOOST_TIME_MS 9000 //time from inj valve open to motor bunout
#define RECOVERY_DELAY_MS 60000 //time from inj valve open to recovery deployment

static FLIGHT_PHASE flight_state = PHASE_PRELAUNCH;
static float injOpenTime;
static float deploymentTime;

EventGroupHandle_t flightPhaseEventsHandle = NULL;
#define INJ_OPEN (xEventGroupGetBits(flightPhaseEventsHandle) & INJ_OPEN_BIT)
#define RECOVERY_DEPLOYMENT (xEventGroupGetBits(flightPhaseEventsHandle) & RECOVERY_DEPLOYMENT_BIT)

static Otits_Result_t test_flightPhase() {
    Otits_Result_t res;
    float time = millis_();

    // check that millis matches tick count to a reasonable degree TODO: is this how timer works lmao
    if (time - xTaskGetTickCount() > 2) {
        res.info = "millis() value err";
        res.outcome = TEST_OUTCOME_FAILED;
        return res;
    }

    // must be in prelaunch if inj hasn't opened yet
    if (!INJ_OPEN) {
        if (flight_state != PHASE_PRELAUNCH) {
            res.info = "prelaunch not detected";
            res.outcome = TEST_OUTCOME_FAILED;
            return res;
        } else if (time > 30000) { // check inj_open hasn't gotten stuck TODO: what should this number be ??
            res.info = "inj open never happened";
            res.outcome = TEST_OUTCOME_FAILED;
            return res;
        }
    }
    // between the times of inj open and end of boost
    else if (INJ_OPEN && (time - injOpenTime) <= BOOST_TIME_MS) {
        if (flight_state != PHASE_BOOST) {
            res.info = "boost not detected";
            res.outcome = TEST_OUTCOME_FAILED;
            return res;
        }
    }
    // between the times of boost and recovery deployment (and inj is open)
    else if (INJ_OPEN && (time - injOpenTime) <= RECOVERY_DELAY_MS && !RECOVERY_DEPLOYMENT) {
        if (flight_state != PHASE_COAST) {
            res.info = "coast not detected";
            res.outcome = TEST_OUTCOME_FAILED;
            return res;
        }
    }
    // any time after recovery deployment
    else if (INJ_OPEN && (((time - injOpenTime) > RECOVERY_DELAY_MS) || RECOVERY_DEPLOYMENT)) {
        if (flight_state != PHASE_DESCENT) {
            res.info = "descent not detected";
            res.outcome = TEST_OUTCOME_FAILED;
            return res;
        }
    }
    // a time era that is not recognized ???
    else {
        res.info = "flight phase not recognized?!";
        res.outcome = TEST_OUTCOME_FAILED;
        return res;
    }
    // pass if nothing has gone wrong. this test could probably be more explicitly pass than implicit idk
    res.info = "";
    res.outcome = TEST_OUTCOME_PASSED;
    return res;
}

bool flightPhaseInit() {
	flightPhaseEventsHandle = xEventGroupCreate();
	if (flightPhaseEventsHandle == NULL) return false;
	xEventGroupClearBits(flightPhaseEventsHandle, 0xFFFF); //clear out the enitire group since API doesn't specify if values are initialized to 0
	if (!otitsRegister(test_flightPhase, TEST_SOURCE_FLIGHT_PHASE, "phaseCheck")) return false;
	return true;
}

void flightPhaseTask(void *argument) {
    TickType_t LastWakeTime = xTaskGetTickCount();

	while(1) {
		float time = millis_();

		if(flight_state == PHASE_PRELAUNCH && INJ_OPEN) {
			injOpenTime = time;
			flight_state = 	PHASE_BOOST;
			logInfo("FlightPhase", "Entered boost phase at %fms", time);
		}

		else if(flight_state == PHASE_BOOST && (time - injOpenTime) > BOOST_TIME_MS) {
			flight_state = PHASE_COAST;
            logInfo("FlightPhase", "Entered coast phase at %fms", time);
		}

		else if((flight_state == PHASE_COAST && (time - injOpenTime) > RECOVERY_DELAY_MS) || RECOVERY_DEPLOYMENT) {
			flight_state = PHASE_DESCENT;
            logInfo("FlightPhase", "Entered descent phase at %fms", time);
		}
		vTaskDelayUntil(&LastWakeTime, 50);
	}
}

bool extensionAllowed() {
	return flight_state == PHASE_COAST;
}
