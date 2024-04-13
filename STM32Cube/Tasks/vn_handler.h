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
#include "semphr.h"
#include "Fusion.h"
#include <stdbool.h>

extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim1;

extern SemaphoreHandle_t vnIMUResultMutex;

bool vnIMUSetup();

void vnIMUHandler(void *argument);

bool writeIMUData(FusionVector *gyroscope, FusionVector *accelerometer, FusionVector *magnetometer);


#endif /* VN_HANDLER_H_ */
