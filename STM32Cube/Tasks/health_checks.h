/*
 * health_checks.h
 *
 *  Created on: Apr 13, 2024
 *      Author: joedo
 */

#ifndef HEALTH_CHECKS_H_
#define HEALTH_CHECKS_H_

#include "stm32h7xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>

#define ADC1_MAX_COUNTS 65535.0
#define ADC1_VREF 3.3
#define R_SENSE 0.033
#define U3_OPAMP_GAIN 100.0

#define ADC1_VOLTAGE_V(counts) (counts / ADC1_MAX_COUNTS * ADC1_VREF)
#define ADC1_CURR_mA(voltage) (voltage / U3_OPAMP_GAIN / R_SENSE )

#define MAX_CURR_5V_mA 500

void healthCheckTask(void *argument);


#endif /* HEALTH_CHECKS_H_ */
