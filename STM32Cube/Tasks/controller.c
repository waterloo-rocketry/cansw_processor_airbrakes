/*
 * controller.c
 *
 *  Created on: Apr 14, 2024
 *      Author: joedo
 */

#include "controller.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "stm32h7xx_hal.h"
#include <math.h>
#include "millis.h"
#include "flight_phase.h"
#include "can_handler.h"
#include "printf.h"
#include "trajectory.h"

QueueHandle_t apogeeQueue;

void controllerInit()
{
	apogeeQueue = xQueueCreate(1, sizeof(float));
}

void controlTask(void *argument)
{
	float apogeeEstimate;
	controller_t airbrakesController;
	airbrakesController.target_altitude = 10000;
	float last_ms = millis_();
	float initial_extension = 0;
	xQueueOverwrite(extQueue, &initial_extension); //put a valid element in the queue so that trajectory prediction can actually run

  /* Infinite loop */
	for(;;)
	{
		if(xQueueReceive(apogeeQueue, &apogeeEstimate, 100) == pdTRUE)
		{
			//PID controller update
			airbrakesController.error = airbrakesController.target_altitude - apogeeEstimate;
			airbrakesController.controller_term_P = airbrakesController.error * CONTROLLER_GAIN_P;
			float dt = (millis_() - last_ms) / 1000.0; //time delay in s
			airbrakesController.controller_term_I = airbrakesController.controller_term_I + CONTROLLER_GAIN_I * airbrakesController.error * dt; //Add some time measure here
			airbrakesController.controller_term_D = CONTROLLER_GAIN_D * (airbrakesController.error - airbrakesController.last_error)/ dt; //here too
			last_ms = millis_();
			airbrakesController.last_error = airbrakesController.error;

			float extension = airbrakesController.controller_term_P + airbrakesController.controller_term_I - airbrakesController.controller_term_D;

			if(extension > CONTROLLER_MAX_EXTENSION) extension = CONTROLLER_MAX_EXTENSION;
			if(extension < CONTROLLER_MIN_EXTENSION) extension = CONTROLLER_MIN_EXTENSION;

			xQueueOverwrite(extQueue, &extension); //make the new extension value available to trajectory prediction

			if(extensionAllowed())
			{
				can_msg_t msg;
				build_actuator_cmd_analog( (uint32_t) millis_(), ACTUATOR_AIRBRAKES_SERVO, extension, &msg);
				xQueueSend(busQueue, &msg, 5); //If we are in the coast phase, command the airbrakes servo to the target extension value
			}
			printf_("extension: %f", extension);
		}
	}
}
