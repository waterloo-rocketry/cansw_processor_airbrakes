/*
 * health_checks.h
 *
 *  Created on: Apr 13, 2024
 *      Author: joedo
 */

#ifndef HEALTH_CHECKS_H_
#define HEALTH_CHECKS_H_

#define ADC1_MAX_COUNTS 65535.0
#define ADC1_VREF 3.3
#define ADC1_VOLTAGE(counts) (counts / ADC1_MAX_COUNTS * ADC1_VREF)
#define CURR_5V(voltage) (voltage / 0.033)

void healthCheckTask(void *argument);


#endif /* HEALTH_CHECKS_H_ */
