/*
 * controller.c
 *
 *  Created on: Apr 14, 2024
 *      Author: joedo
 */
#include "stm32h7xx_hal.h"

#include <math.h>

#include "printf.h"
#include "millis.h"

#include "controller.h"
#include "flight_phase.h"
#include "can_handler.h"
#include "trajectory.h"
#include "log.h"

QueueHandle_t apogeeQueue;
QueueHandle_t targetQueue;

void controllerInit()
{
	apogeeQueue = xQueueCreate(1, sizeof(float));
	targetQueue = xQueueCreate(1, sizeof(uint16_t));
}

void controlTask(void *argument)
{
	float apogeeEstimate;
	controller_t airbrakesController;
	airbrakesController.last_error = 0;
	airbrakesController.target_altitude = 7000;
	float last_ms = millis_();

  /* Infinite loop */
	for(;;)
	{
		uint16_t updated_target;
		if(xQueueReceive(targetQueue, &updated_target, 0) == pdPASS)
		{
			airbrakesController.target_altitude = (float) updated_target;
			can_msg_t msg;
			build_state_est_calibration_msg((uint32_t) millis_(), 1, updated_target, &msg);
			xQueueSend(busQueue, &msg, 100);
			logInfo("controller", "Controller target updated to %d m", updated_target);
		}

		if(extensionAllowed() && xQueueReceive(apogeeQueue, &apogeeEstimate, 100) == pdTRUE)
		{
			//PID controller update
			airbrakesController.error = airbrakesController.target_altitude - apogeeEstimate;
			airbrakesController.controller_term_P = airbrakesController.error * CONTROLLER_GAIN_P;
			float dt = (millis_() - last_ms) / 1000.0; //time delay in s
			airbrakesController.controller_term_I = airbrakesController.controller_term_I + CONTROLLER_GAIN_I * airbrakesController.error * dt;
			if(airbrakesController.controller_term_I > CONTROLLER_I_SATMAX) airbrakesController.controller_term_I = CONTROLLER_I_SATMAX;
			if(airbrakesController.controller_term_I < -CONTROLLER_I_SATMAX) airbrakesController.controller_term_I = -CONTROLLER_I_SATMAX;

			//prevent divide by 0 errors
			if(dt < 0.0000001) airbrakesController.controller_term_D = 0;
			else airbrakesController.controller_term_D = CONTROLLER_GAIN_D * (airbrakesController.error - airbrakesController.last_error) / dt;

			last_ms = millis_();
			airbrakesController.last_error = airbrakesController.error;

			float output = airbrakesController.controller_term_P + airbrakesController.controller_term_I - airbrakesController.controller_term_D;
			float extension = 0.5 - output; //invert the controller output - if we are undershooting retract, and if we are overshooting, extend

			if(extension > CONTROLLER_MAX_EXTENSION) extension = CONTROLLER_MAX_EXTENSION;
			if(extension < CONTROLLER_MIN_EXTENSION) extension = CONTROLLER_MIN_EXTENSION;

			can_msg_t msg;
			build_actuator_cmd_analog( (uint32_t) millis_(), ACTUATOR_AIRBRAKES_SERVO, (uint8_t) (100 * extension), &msg);
			xQueueSend(busQueue, &msg, 5); //If we are in the coast phase, command the airbrakes servo to the target extension value
			logInfo("controller", "ext CMD: %d", extension * 100);
			printf_("extension: %f\n", extension);

		}
	}
}
