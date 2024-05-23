/*
 * controller.c
 *
 *  Created on: Apr 14, 2024
 *      Author: joedo
 */

#include "controller.h"
#include "freertos.h"
#include "cmsis_os.h" //TODO replace with xtaskDelayUntil
#include "stm32h7xx_hal.h"
#include <stdio.h>
#include <math.h>
#include "millis.h"

extern UART_HandleTypeDef huart4;


//needed dependecy for stdio-->printf()
int __io_putchar(int ch)
{
	HAL_UART_Transmit(&huart4,(uint8_t*)&ch,1,HAL_MAX_DELAY);
	return ch;
}



void controlTask(void *argument)
{
	float altitudeEstimate;
	controller_t airbrakesController;
	TickType_t last_ticks = xTaskGetTickCount();

	float startTime = millis();
	osDelay(1000);
	float measuredDelay = millis()-startTime;
	printf("1000ms HAL_Delay is: %lu ms\r\n",(unsigned long)(round(measuredDelay)));

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
		airbrakesController.controller_term_D = CONTROLLER_GAIN_D * (airbrakesController.error - airbrakesController.last_error)/ dt; //here too
		last_ticks = xTaskGetTickCount();

		float extension = airbrakesController.controller_term_P + airbrakesController.controller_term_I - airbrakesController.controller_term_D;

		if(extension > CONTROLLER_MAX_EXTENSION) extension = CONTROLLER_MAX_EXTENSION;
		if(extension < CONTROLLER_MIN_EXTENSION) extension = CONTROLLER_MIN_EXTENSION;

		//TODO log to queue and send to CAN

		//Test Code
		char buffer[16] = {0};
		sprintf(&buffer, "Target Ext: %d\r\n", (int)(extension * 1000));
		//HAL_UART_Transmit(&huart4, &buffer, 16, 10); //Blocking API, 10ms timeout
		osDelayUntil(CONTROLLER_DELAY_TICKS); //Implements periodic behaviour (nominal delay - time spent running the loop code)
	}
  /* USER CODE END controlTask */
}
