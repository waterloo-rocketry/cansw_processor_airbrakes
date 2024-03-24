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
#include "vectornav.h"

extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim1;

extern xQueueHandle logQueue;

typedef struct {
	uint32 timestamp;
	float accel_x;
	float accel_y;
	float accel_z;
	float gyro_x;
	float gyro_y;
	float gyro_z;
	float mag_x;
	float mag_y;
	float mag_z;
} vn_imu_data_t;

typedef struct {
	uint32 timestamp;
	float gps_lat;
	float gps_lon;
	float gps_alt;
} vn_gps_data_t;

typedef struct {

} vn_full_state_t;

bool vnIMUSetup();

void vnIMUHandler(void *argument);


#endif /* VN_HANDLER_H_ */
