/*
 * flight_phase.h
 *
 *  Created on: Apr 13, 2024
 *      Author: joedo
 */

#ifndef FLIGHT_PHASE_H_
#define FLIGHT_PHASE_H_





//init a timer that isn't timer 1
//set the timer to trigger every Xus
//on timer interrupt, increment a counter
//Each time thread executes, check value of counter against predefined thresholds
//Based on threshold values, update internal state
//Based on internal state, update FreeRTOS event flags

typedef enum FLIGHT_PHASE {
	PHASE_PRELAUNCH,
	PHASE_BOOST,
	PHASE_COAST,
	PHASE_DESCENT,
	PHASE_LANDED
};

#endif /* FLIGHT_PHASE_H_ */
