/*
 * controller.c
 *
 *  Created on: Apr 14, 2024
 *      Author: joedo
 */


void controlTask(void *argument)
{
  /* USER CODE BEGIN controlTask */
	uint32_t min_controller_frequency = 10; //10 Hz TODO: Make defines for these somewhere more visible
	uint32_t controller_delay_ticks = 1000 / min_controller_frequency / portTICK_RATE_MS; //equivalent FreeRTOS ticks

	altitudeMessage altitudeEstimate;

	//TODO: Put inside of struct
	float controller_term_P = 0;
	float controller_term_I = 0;
	float controller_term_D = 0;
	float target_altitude = ALTITUDE_TARGET_FT;
	float last_error;
	float error;
	float target_extension;
	TickType_t last_ticks = xTaskGetTickCount();

  /* Infinite loop */
	for(;;)
	{
		//xQueueReceive(altitudeQueue, &altitudeEstimate, 10);
		altitudeEstimate.altitude = idx*10; //Test code

		error = target_altitude - altitudeEstimate.altitude;
		controller_term_P = error * CONTROLLER_GAIN_P;
		float dt = (float)(xTaskGetTickCount() - last_ticks) / (portTICK_RATE_MS * 1000); //time delay in ms TODO: convert to more accurate source
		controller_term_I = controller_term_I + CONTROLLER_GAIN_I * error * dt; //Add some time measure here
		controller_term_D = (error - last_error) * CONTROLLER_GAIN_D / dt; //here too
		last_ticks = xTaskGetTickCount();

		float extension = controller_term_P + controller_term_I - controller_term_D;

		if(extension > CONTROLLER_MAX_EXTENSION) extension = CONTROLLER_MAX_EXTENSION;
		if(extension < CONTROLLER_MIN_EXTENSION) extension = CONTROLLER_MIN_EXTENSION;

		//xQueueSend(busQueue);

		//Test Code
		char buffer[16] = {0};
		sprintf(&buffer, "Target Ext: %f", extension);
		HAL_UART_Transmit(&buffer, art4, &buffer, 16, 10); //Blocking API, 10ms timeout
		osDelayUntil(controller_delay_ticks); //Implements periodic behaviour (nominal delay - time spent running the loop code)
	}
  /* USER CODE END controlTask */
}
