/*
 * simulink_handeler.h
 *
 *  Created on: Oct 23, 2024
 *      Author: joedo
 */

#ifndef SIMULINK_HANDELER_H_
#define SIMULINK_HANDELER_H_

#include "stm32h7xx_hal.h"

#include "test_codegen.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "event_groups.h"

//simulink data
//should use getters and setters, externed for now

extern real_T rtU_thisIsAnInput;
extern real_T rtY_thisIsAnOutput;

void updateModelTask(void *arguments);
void updateModelTaskInit();

#endif /* SIMULINK_HANDELER_H_ */
