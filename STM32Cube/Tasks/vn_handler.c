/* * vn_handler.c
 *
 *  Created on: Mar 24, 2024
 *      Author: joedo*/

#include <math.h>
#include <string.h>
#include <stdlib.h>

#include "stm32h7xx_hal.h"

#include "vn/protocol/upack.h"
#include "printf.h"

#include "log.h"
#include "state_estimation.h"
#include "vn_handler.h"
#include "can_handler.h"
#include "millis.h"


extern UART_HandleTypeDef huart1;

#define MAX_BINARY_OUTPUT_LENGTH 200
const uint32_t DMA_RX_TIMEOUT = 250;
const uint8_t MS_WAIT_CAN = 10;
const uint32_t NS_TO_S = 1000000000;
const uint32_t NS_TO_MS = 1000000;
const uint32_t ASCII_METERS = 109;
const float RAD_TO_DEG = 180 / M_PI;
const float g = 9.81;
const bool verbose = false;
const uint8_t SD_RATE_DIVISOR_GROUP1 = 1;
const uint8_t CAN_RATE_DIVISOR_GROUP1 = 3;
const uint8_t CAN_RATE_DIVISOR_GROUP3 = 10;
const uint8_t SD_RATE_DIVISOR_GROUP3 = 1;

uint8_t SDGroup1Counter=0;
uint8_t CANGroup1Counter=0;
uint8_t SDGroup3Counter=0;
uint8_t CANGroup3Counter=0;

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
	double x1=fabs(deg);
	uint32_t x2 = (uint32_t) fabs(deg);
	double result = (x1-x2)*60;

	return result;
}

static void send3VectorStateCanMsg_float(uint32_t time, float vector[3], uint8_t firstStateIDOfVector){
	can_msg_t msg;
	for (int i = 0; i < 3; i++) {
		build_state_est_data_msg(time, &vector[i], firstStateIDOfVector + i, &msg);
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

bool vn_handler_init()
{
	HAL_StatusTypeDef status = HAL_UART_RegisterRxEventCallback(&huart1, VN_UASART1_RX_callback);
	USART1_DMA_Sempahore = xSemaphoreCreateBinary();

	return (status == HAL_OK) && (USART1_DMA_Sempahore != NULL);
}


void vnIMUHandler(void *argument)
{
	for(;;)
	{
		HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(&huart1, USART1_Rx_Buffer, MAX_BINARY_OUTPUT_LENGTH); //Begin a receive, until we read MAX_BINARY_OUTPUT_LENGTH or the line goes idle, indicating a shorter message
		if(status != HAL_OK)
		{
			//this really should not error if we are getting Vn messages at the expected rate
		}
		else
		{
			if(xSemaphoreTake(USART1_DMA_Sempahore, DMA_RX_TIMEOUT) != pdTRUE) //this semaphore is given by the DMA Rx interrupt, indicating we got UART data
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
						    logError("VN", "Mem overflow");
							continue;
						}

						//Group #1: 53 bytes: 53 bytes: Time startup (Common), NumSats (gnss), GnssPosLla (gnss), GnssPosUncertainty (gnss)

						if (packetLength == 53){
							can_msg_t msg;

							uint32_t time_startup = (uint32_t) (VnUartPacket_extractUint64(&packet)/ NS_TO_MS); //time in ns -> s

							uint8_t numSatellites = VnUartPacket_extractInt8(&packet);
							vec3d pos = VnUartPacket_extractVec3d(&packet);
							vec3f postUncertainty = VnUartPacket_extractVec3f(&packet);

							uint32_t quality_ = sqrt(pow(postUncertainty.c[0], 2) + pow(postUncertainty.c[1], 2) + pow(postUncertainty.c[2], 2)); //the higher the number the worse the quality
							uint8_t quality = (uint8_t) quality_; //cast should truncate

							SDGroup1Counter++;
							CANGroup1Counter++;

							if (SDGroup1Counter >= SD_RATE_DIVISOR_GROUP1){
	                            logInfo("VN#1", "%ds, lat/lon/alt (%.3f, %.3f, %.3f) +-(%.3f, %.3f, %.3f), %d sats\n",
	                                    time_startup, pos.c[0],pos.c[1],pos.c[2],
	                                    postUncertainty.c[0],postUncertainty.c[1],postUncertainty.c[2],
	                                    numSatellites);

								SDGroup1Counter = 0;
							}


							if (CANGroup1Counter >= CAN_RATE_DIVISOR_GROUP1){
								//Logging + CAN
								uint8_t degrees_lon = (uint8_t) abs(pos.c[0]);
								uint8_t minutes_lon = (uint8_t) minFromDeg(pos.c[0]);
								uint8_t decimal_min_lon = decimalFromDouble4(minFromDeg(pos.c[0]));

								uint8_t degrees_lat = (uint8_t) abs(pos.c[1]);
								uint8_t minutes_lat = (uint8_t) minFromDeg(pos.c[1]);
								uint8_t decimal_min_lat = decimalFromDouble4(minFromDeg(pos.c[1]));

								uint16_t altitude = (uint16_t) abs(pos.c[2]);
								uint16_t altitude_decimal = decimalFromDouble2(pos.c[2]);


								uint32_t msg_ts = (uint32_t) millis_();
								build_gps_lon_msg(msg_ts, degrees_lon, minutes_lon, decimal_min_lon, (int) (degrees_lon >= 0) ? 'W' : 'E', &msg);
								xQueueSend(busQueue, &msg, MS_WAIT_CAN);
								build_gps_lat_msg(msg_ts, degrees_lat, minutes_lat, decimal_min_lat, (int) (degrees_lat >= 0) ? 'N' : 'S', &msg);
								xQueueSend(busQueue, &msg, MS_WAIT_CAN);
								build_gps_alt_msg(msg_ts, altitude, altitude_decimal , (uint8_t) 'M', &msg);
								xQueueSend(busQueue, &msg, MS_WAIT_CAN);
								build_gps_info_msg(msg_ts, numSatellites, quality, &msg);
								xQueueSend(busQueue, &msg, MS_WAIT_CAN);

								CANGroup1Counter = 0;
							}
						}


						//Binary Output #3 52 bytes | Time startup (Common), UncompMag (IMU). UncompAccel (IMU), UncompGyro (IMU)
						else if (packetLength == 52){
							uint64_t time_startup_ns = VnUartPacket_extractUint64(&packet); //time in ns

							vec3f magVec = VnUartPacket_extractVec3f(&packet); //Gauss
							vec3f accelVec = VnUartPacket_extractVec3f(&packet); //m/s^2
							vec3f gyroVec = VnUartPacket_extractVec3f(&packet); //rad/s

							rawIMUPacked data;
							for(int i = 0; i < 3; i++)
							{
								gyroVec.c[i] = gyroVec.c[i] * RAD_TO_DEG; //rad/s to deg/s
								accelVec.c[i] = accelVec.c[i] / g; //m/s^2 to gs
								//magnetometer units are arbitrary for Fusion
							}

//							printf_(">MAGX:%.4f\n>MAGY:%.4f\n>MAGZ:%.4f\n", magVec.c[0], magVec.c[1], magVec.c[2]);
//							printf_(">ACCX:%.2f\n>ACCY:%.2f\n>ACCZ:%.2f\n", accelVec.c[0], accelVec.c[1], accelVec.c[2]);
//							printf_(">GYRX:%.2f\n>GYRY:%.2f\n>GYRZ:%.2f\n", gyroVec.c[0], gyroVec.c[1], gyroVec.c[2]);

							memcpy(data.accelerometer.array, accelVec.c, 3 * sizeof(float));
							memcpy(data.gyroscope.array, gyroVec.c, 3 * sizeof(float));
							memcpy(data.magnetometer.array, magVec.c, 3 * sizeof(float));
							data.TimeS = time_startup_ns / (float) NS_TO_S;
							xQueueSend(IMUDataHandle, &data, 0); //send to state estimation

							SDGroup3Counter++;
							CANGroup3Counter++;

							if (SDGroup3Counter >= SD_RATE_DIVISOR_GROUP3){
	                            logInfo("VN#3", "%ds, lin: (%.3f, %.3f, %.3f), gyro: (%.3f, %.3f, %.3f)\n",
	                            		time_startup_ns, accelVec.c[0],accelVec.c[1],accelVec.c[2],
										gyroVec.c[0],gyroVec.c[1],gyroVec.c[2]);

	                            SDGroup3Counter = 0;
							}

							if (CANGroup3Counter >= CAN_RATE_DIVISOR_GROUP3){
								//Logging + CAN
								send3VectorStateCanMsg_float((uint32_t) millis_(), accelVec.c,STATE_ACC_X); //Linear Acceleration
								send3VectorStateCanMsg_float((uint32_t) millis_(), gyroVec.c,STATE_RATE_YAW); //Angular Velocity
								CANGroup3Counter = 0;
							}

						}
						else {
							//printf_("unhandled message format!\r\n");
							logError("VN", "unhandled message format!");
						}
					}

					else {
						logError("VN", "Non Binary Packet received");
					}
			}

		}
	}

}
