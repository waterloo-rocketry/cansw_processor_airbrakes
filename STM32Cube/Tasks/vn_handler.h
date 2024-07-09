/*
 * vn_handler.h
 *
 *  Created on: Mar 24, 2024
 *      Author: joedo
 */

#ifndef VN_HANDLER_H_
#define VN_HANDLER_H_

#include "stm32h7xx_hal.h"
#include "state_estimation.h"
#include "Fusion.h"
#include <stdbool.h>

void vnIMUHandler(void *argument);
void vn_handler_init();

#endif /* VN_HANDLER_H_ */
