/*
 * vn_handler.h
 *
 *  Created on: Mar 24, 2024
 *      Author: joedo
 */

#ifndef VN_HANDLER_H_
#define VN_HANDLER_H_

#include "stm32h7xx_hal.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "Fusion.h"
#include <stdbool.h>

bool vnIMUSetup();

void vnIMUHandler(void *argument);

void USART1_DMA_Rx_Complete_Callback(UART_HandleTypeDef *huart);


#endif /* VN_HANDLER_H_ */
