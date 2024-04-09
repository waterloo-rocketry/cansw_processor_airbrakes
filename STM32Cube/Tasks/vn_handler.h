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
#include <stdbool.h>

extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim1;

bool vnIMUSetup();

void vnIMUHandler(void *argument);


#endif /* VN_HANDLER_H_ */
