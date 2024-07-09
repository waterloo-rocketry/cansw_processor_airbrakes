/* * vn_handler.c
 *
 *  Created on: Mar 24, 2024
 *      Author: joedo*/

#include "stm32h7xx_hal.h"
#include "vn_handler.h"
#include "vn/protocol/upack.h"
#include "log.h"
#include "state_estimation.h"
#include "printf.h"
#include "can_handler.h"
#include <math.h>
#include <string.h>
#include "stdbool.h"

extern UART_HandleTypeDef huart1;

#define MAX_BINARY_OUTPUT_LENGTH 200
const uint32_t  DMA_RX_TIMEOUT = 300;
const uint8_t MS_WAIT_CAN = 10;
const uint32_t NS_TO_S = 1000000000;
const uint32_t NS_TO_MS = 1000000;
const uint32_t ASCII_METERS = 109;
const bool verbose = false;

uint8_t USART1_Rx_Buffer[MAX_BINARY_OUTPUT_LENGTH];
SemaphoreHandle_t USART1_DMA_Sempahore;

void VN_UASART1_RX_callback(UART_HandleTypeDef *huart, uint16_t Pos)
{
	if(huart == &huart1 && huart->RxEventType == 0x02U) //IDLE event on USART1
		{
			BaseType_t xHigherPriorityTaskWoken = pdFALSE;
			xSemaphoreGiveFromISR(USART1_DMA_Sempahore, &xHigherPriorityTaskWoken);
			portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
		}
}

static uint8_t decimalFromDouble2(double num){
	uint16_t integer_part = (uint16_t)num;
	double fractional_part = num - integer_part;
	if (fractional_part*100 > 254){
		printf_("Error: getDecimalFromDouble2: overflow");
		return 0;
	}
	uint8_t decimal_part = (uint8_t)(fractional_part * 100); // This captures up to two decimal places which is the max safely stored by a 8bit uint, since max size is 255 it cant hold 999 only 99

	return decimal_part;
}

static uint16_t decimalFromDouble4(double num){
	uint16_t integer_part = (uint16_t)num;
	double fractional_part = num - integer_part;
	if (fractional_part*10000 > 65534){
		printf_("Error: getDecimalFromDouble4: overflow");
		return 0;
	}
	uint16_t decimal_part = (uint8_t)(fractional_part * 10000); // This captures up to 4 decimal places which is the max safely stored by a 16bit uint

	return decimal_part;
}

static double minFromDeg(double deg){
	return (deg -  (uint32_t)deg)  * 60;
}

static void send3VectorStateCanMsg_float(uint64_t time, float vector[3], uint8_t firstStateIDOfVector){
	can_msg_t msg;
	for (int i = 0; i < 3; i++) {
		build_state_est_data_msg((uint32_t) time, &vector[i], firstStateIDOfVector + i, &msg);
		xQueueSend(busQueue, &msg, MS_WAIT_CAN);
	}
}

static void send3VectorStateCanMsg_double(uint64_t time, double vector[3], uint8_t firstStateIDOfVector){
	float float_vector[3];
	for (int i = 0; i < 3; i++) {
		float_vector[i] = (float) vector[i];  // Convert double to float
	}
	send3VectorStateCanMsg_float(time, float_vector, firstStateIDOfVector);
}

void vn_handler_init()
{
	HAL_StatusTypeDef status = HAL_UART_RegisterRxEventCallback(&huart1, VN_UASART1_RX_callback);
	USART1_DMA_Sempahore = xSemaphoreCreateBinary();
}


void vnIMUHandler(void *argument)
{
	for(;;)
	{
		//Begin a receive, until we read MAX_BINARY_OUTPUT_LENGTH or the line goes idle, indicating a shorter message
		HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(&huart1, USART1_Rx_Buffer, MAX_BINARY_OUTPUT_LENGTH);
		if(status != HAL_OK)
		{
			//this is not an error, its fine if the code errors here, dont put debug statment here
		}
		else
		{
			printf_("\r\n");
			if(xSemaphoreTake(USART1_DMA_Sempahore, 100) != pdTRUE) //this semaphore is given by the DMA Rx interrupt, indicating we got UART data
			{
				//we timed out, we did not see valid data
			}
			else
			{
				//We got some data. Parse it.
				VnUartPacket packet;
				packet.curExtractLoc = 0;
				packet.data = USART1_Rx_Buffer;
				packet.length = MAX_BINARY_OUTPUT_LENGTH;


					if(VnUartPacket_type(&packet) == PACKETTYPE_BINARY)
					{
						size_t packetLength = VnUartPacket_computeBinaryPacketLength(packet.data);

						if (packetLength>MAX_BINARY_OUTPUT_LENGTH){
							printf_("Memory Overflow!\n\n\r\n");
							continue;
						}


						if (verbose){
							printf_("Data recived: ");
							for(int i = 0; i < MAX_BINARY_OUTPUT_LENGTH; i++) {
								printf_("0x%x ", USART1_Rx_Buffer[i]);
							}
							printf_("\r\n");
						}

						if (packetLength == 53){
							can_msg_t msg;

							uint64_t time_startup = VnUartPacket_extractUint64(&packet)/ NS_TO_MS; //time in ns -> s

							vec3d pos = VnUartPacket_extractVec3d(&packet);
							uint8_t numSatellites = VnUartPacket_extractInt8(&packet);
							vec3f postUncertainty = VnUartPacket_extractVec3f(&packet);

							uint32_t quality_ = sqrt(pow(postUncertainty.c[0], 2) + pow(postUncertainty.c[1], 2) + pow(postUncertainty.c[2], 2)); //the higher the number the worse the quality
							uint8_t quality = (uint8_t) quality_; //cast should truncate

							char msgAsString[300]  = {0};
							sprintf_(msgAsString, "Time: %lli, pos: (Lat: %.3f, Lon: %.3f, Alt: %.3f) +- (%.3f, %.3f, %.3f) using %d satellites \r\n", time_startup, pos.c[0],pos.c[1],pos.c[2], postUncertainty.c[0],postUncertainty.c[1],postUncertainty.c[2], numSatellites);
							printf_(msgAsString);

							//Logging + CAN
							build_gps_lon_msg((uint32_t) time_startup, (uint8_t) pos.c[1], (uint8_t) minFromDeg(pos.c[1]), decimalFromDouble4(minFromDeg(pos.c[1])), (int) (pos.c[1] >= 0) ? 'N' : 'S', &msg);
							xQueueSend(busQueue, &msg, MS_WAIT_CAN);
							build_gps_lat_msg((uint32_t) time_startup, (uint8_t) pos.c[1], (uint8_t) minFromDeg(pos.c[1]), decimalFromDouble4(minFromDeg(pos.c[1])), (int) (pos.c[1] >= 0) ? 'E' : 'W', &msg);
							xQueueSend(busQueue, &msg, MS_WAIT_CAN);
							build_gps_alt_msg((uint32_t) time_startup, (uint16_t)pos.c[2], decimalFromDouble2(pos.c[2]), (uint8_t) 'M', &msg);
							xQueueSend(busQueue, &msg, MS_WAIT_CAN);
							build_gps_info_msg((uint32_t) time_startup, numSatellites, quality, &msg);
							xQueueSend(busQueue, &msg, MS_WAIT_CAN);

							logInfo("VN Group 1", msgAsString);


						}

						//Binary Output #2 92 bytes | Time startup (Common), Angular rate (IMU), Ypr (Attitude), PosEcef (INS), VelEcef (INS), LinAccelEcef (INS)
						else if (packetLength == 92){
							uint64_t time_startup = VnUartPacket_extractUint64(&packet)/ NS_TO_MS; //time in ns -> s

							vec3f angularRate = VnUartPacket_extractVec3f(&packet); //rad/s
							vec3f yprAngles = VnUartPacket_extractVec3f(&packet); //deg

							vec3d posEcef = VnUartPacket_extractVec3d(&packet); //m
							vec3f velEcef = VnUartPacket_extractVec3f(&packet); //m/s
							vec3f linAccelEcef = VnUartPacket_extractVec3f(&packet); //m/s^2 NOT INCLUDING GRAVITY



							char msgAsString[3000] = {0};
							sprintf_(msgAsString,"Time: %lli, Angular Rate: (X: %.3f, Y: %.3f, Z: %.3f), YPR Angles: (Yaw: %.3f, Pitch: %.3f, Roll: %.3f), Pos ECEF: (X: %.3f, Y: %.3f, Z: %.3f), Vel ECEF: (X: %.3f, Y: %.3f, Z: %.3f), Lin Accel ECEF: (X: %.3f, Y: %.3f, Z: %.3f)\r\n",
															time_startup,
															angularRate.c[0], angularRate.c[1], angularRate.c[2],
															yprAngles.c[0], yprAngles.c[1], yprAngles.c[2],
															posEcef.c[0], posEcef.c[1], posEcef.c[2],
															velEcef.c[0], velEcef.c[1], velEcef.c[2],
															linAccelEcef.c[0], linAccelEcef.c[1], linAccelEcef.c[2]);
							printf_(msgAsString);


							//Logging + CAN


							send3VectorStateCanMsg_double(time_startup, posEcef.c,STATE_POS_X); //position
							send3VectorStateCanMsg_float(time_startup, velEcef.c,STATE_VEL_X); //velocity
							send3VectorStateCanMsg_float(time_startup, linAccelEcef.c,STATE_ACC_X); //acceleration
							send3VectorStateCanMsg_float(time_startup, yprAngles.c,STATE_ANGLE_YAW); //angle; TODO: this is also sent by OUR state estimation, so we should expand the enum in canlib to distinguish
							send3VectorStateCanMsg_float(time_startup, angularRate.c,STATE_RATE_YAW); //angle rate in XYZ (TODO: NEED TO CONVERT TO YPR)

							logInfo("VN Group 2", msgAsString);



						}

						//Binary Output #3 42 bytes | UncompMag (IMU). UncompAccel (IMU), UncompGyro (IMU)

						else if (packetLength == 52){
							uint64_t time_startup = VnUartPacket_extractUint64(&packet)/ NS_TO_MS; //time in ns -> s

							vec3f magVec = VnUartPacket_extractVec3f(&packet);
							vec3f accelVec = VnUartPacket_extractVec3f(&packet);
							vec3f gyroVec = VnUartPacket_extractVec3f(&packet);

							printf_("Time: %lli, Mag: (X: %.3f, Y: %.3f, Z: %.3f), Accel: (X: %.3f, Y: %.3f, Z: %.3f), Angles: (X: %.3f, Y: %.3f, Z: %.3f)\r\n", time_startup, magVec.c[0], magVec.c[1], magVec.c[2], accelVec.c[0], accelVec.c[1], accelVec.c[2], gyroVec.c[0], gyroVec.c[1], gyroVec.c[2]);


							rawIMUPacked data;
							memcpy(data.accelerometer.array, accelVec.c, 3 * sizeof(float)); //TODO: z is aparentely the 2nd item, and y is the 3rd???
							memcpy(data.gyroscope.array, gyroVec.c, 3 * sizeof(float));
							memcpy(data.magnetometer.array, magVec.c, 3 * sizeof(float));
							data.deltaTimems = 0; //TODO armaan I need time_startup here

							xQueueOverwrite(IMUDataHandle, &data);

						}

						//unhandled message format
						else{
							printf_("unhandled message format!\r\n");
							//logError(SOURCE_SENSOR, "unhandled message format!");
						}
						//printf_("size: %d\r\n", packetLength);
					}

					else {
						//printf_("Not a valid binary packet\r\n");
						//logError(SOURCE_SENSOR, "Non Binary Packet received");
					}
			}

		}
	}

}
