/*
 * controller.c
 *
 *  Created on: Apr 14, 2024
 *      Author: joedo
 */

#include "controller.h"
#include "freertos.h"
#include "queue.h"
#include "cmsis_os.h" //TODO replace with xtaskDelayUntil
#include "stm32h7xx_hal.h"
#include <math.h>
#include "millis.h"
#include "flight_phase.h"
#include "can_handler.h"
#include "printf.h"

extern UART_HandleTypeDef huart4;


void controlTask(void *argument)
{
	float altitudeEstimate;
	controller_t airbrakesController;
	airbrakesController.target_altitude = 10000;
	float last_ticks = millis_();

  /* Infinite loop */
	for(;;)
	{
		//TODO get altitude message from queue
		altitudeEstimate = 1000; //Test code

		//PID controller update
		airbrakesController.error = airbrakesController.target_altitude - altitudeEstimate;
		airbrakesController.controller_term_P = airbrakesController.error * CONTROLLER_GAIN_P;
		float dt = (millis_() - last_ticks); //time delay in ms
		airbrakesController.controller_term_I = airbrakesController.controller_term_I + CONTROLLER_GAIN_I * airbrakesController.error * dt; //Add some time measure here
		airbrakesController.controller_term_D = CONTROLLER_GAIN_D * (airbrakesController.error - airbrakesController.last_error)/ dt; //here too
		last_ticks = millis_();
		airbrakesController.last_error = airbrakesController.error;

		float extension = airbrakesController.controller_term_P + airbrakesController.controller_term_I - airbrakesController.controller_term_D;

		if(extension > CONTROLLER_MAX_EXTENSION) extension = CONTROLLER_MAX_EXTENSION;
		if(extension < CONTROLLER_MIN_EXTENSION) extension = CONTROLLER_MIN_EXTENSION;


		if(extensionAllowed())
		{
			can_msg_t msg;
			build_actuator_cmd_analog( (uint32_t) millis_(), ACTUATOR_AIRBRAKES_SERVO, extension, &msg);
			xQueueSend(busQueue, &msg, 5);
		}
		printf_("extension: %f", extension);
	}
}
