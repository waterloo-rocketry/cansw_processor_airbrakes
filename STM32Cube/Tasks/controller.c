/*
 * controller.c
 *
 *  Created on: Apr 14, 2024
 *      Author: joedo
 */

#include "controller.h"
#include "freertos.h"
#include "cmsis_os.h" //TODO replace with xtaskDelayUntil


void controlTask(void *argument)
{
	float altitudeEstimate;
	controller_t airbrakesController;
	TickType_t last_ticks = xTaskGetTickCount();


  /* Infinite loop */
	for(;;)
	{
		//TODO get altitude message from queue
		altitudeEstimate = 1000; //Test code

		//PID controller update
		airbrakesController.error = airbrakesController.target_altitude - altitudeEstimate;
		airbrakesController.controller_term_P = airbrakesController.error * CONTROLLER_GAIN_P;
		float dt = (float)(xTaskGetTickCount() - last_ticks) / (portTICK_RATE_MS * 1000); //time delay in ms TODO: convert to more accurate source
		airbrakesController.controller_term_I = airbrakesController.controller_term_I + CONTROLLER_GAIN_I * airbrakesController.error * dt; //Add some time measure here
		airbrakesController.controller_term_D = (airbrakesController.error - airbrakesController.last_error) * CONTROLLER_GAIN_D / dt; //here too
		last_ticks = xTaskGetTickCount();

		float extension = airbrakesController.controller_term_P + airbrakesController.controller_term_I - airbrakesController.controller_term_D;

		if(extension > CONTROLLER_MAX_EXTENSION) extension = CONTROLLER_MAX_EXTENSION;
		if(extension < CONTROLLER_MIN_EXTENSION) extension = CONTROLLER_MIN_EXTENSION;

		//TODO log to queue and send to CAN

		//Test Code
		char buffer[16] = {0};
		//sprintf(&buffer, "Target Ext: %f", extension);
		//HAL_UART_Transmit(&buffer, &huart4, &buffer, 16, 10); //Blocking API, 10ms timeout
		osDelayUntil(CONTROLLER_DELAY_TICKS); //Implements periodic behaviour (nominal delay - time spent running the loop code)
	}
  /* USER CODE END controlTask */
}
