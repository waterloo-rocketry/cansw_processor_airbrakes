/*
 * vn_handler.h
 *
 *  Created on: Mar 24, 2024
 *      Author: joedo
 */

#ifndef VN_HANDLER_H_
#define VN_HANDLER_H_

#include "stm32h7xx_hal.h"
#include "freertos.h"
#include "queue.h"

extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim1;

#define DATA_MSG_LENGTH_MAX 100 //max # of bytes in binary output message

typedef struct {
	uint8_t buffer[DATA_MSG_LENGTH_MAX];
} vn_binary_out;

int vnIMUSetup();

void vnIMUHandler(void *argument);


#endif /* VN_HANDLER_H_ */
