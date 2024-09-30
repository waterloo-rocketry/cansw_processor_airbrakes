/*
 * blinky.c
 * Demo task
 *
 */


#include "FreeRTOS.h"
#include "stm32h7xx_hal.h"
#include "task.h"

#include "printf.h"

// FreeRTOS task functions always have the function signature  `void taskname(void *argument)`
void blinkyTask(void *argument) {
	// This loop repeats for infinity and must never exit! (tasks never end)
	for (;;) {
		// Write 0 to the LED pin (turn it off)
		// HAL functions are provided by STM32 to easily interact with hardware
		// LEDs are wired to the STM32's GPIO pins - this one specifically is wired to GPIO_B pin 5
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);

		// FreeRTOS function that puts this task on pause for 500 ticks (0.5 sec)
		// Other tasks will run while this task is paused
		vTaskDelay(500);

		// Write 1 to the LED pin (turn it on)
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1);

		// Pause task for 500 ticks (0.5 sec)
		vTaskDelay(500);
	}
}
