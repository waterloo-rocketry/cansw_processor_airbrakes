/* * vn_handler.c
 *
 *  Created on: Mar 24, 2024
 *      Author: joedo*/


#include "vn_handler.h"
#include "vn/protocol/upack.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "log.h"
#include "stm32h7xx_hal.h"
#include <stdio.h>
#include "printf.h"
#include "can_handler.h"
#include <math.h>

extern UART_HandleTypeDef huart4;
extern xQueueHandle busQueue;


////RateDivisor sets the output rate as a function of the sensor ImuRate (800 Hz for the VN-200)
//#define CONFIG_MSG_LENGTH 100
//#define VN_200_IMURATE 800 //Hz
//
////Binary Output #1
//#define BINARY1_OUT_RATE 10 //Hz
//#define RATE1_DIVISOR VN_200_IMURATE / BINARY1_OUT_RATE
//
////See "vnenum.h" and the ICD for parameter definitions
//
//#define COMMONGROUP_PARAMS COMMONGROUP_QUATERNION | COMMONGROUP_POSITION | COMMONGROUP_VELOCITY | COMMONGROUP_ACCEL
//#define TIMEGROUP_PARAMS TIMEGROUP_TIMEUTC | TIMEGROUP_TIMEUTC
//#define IMUGROUP_PARAMS IMUGROUP_IMUSTATUS | IMUGROUP_SENSSAT | IMUGROUP_UNCOMPMAG | IMUGROUP_UNCOMPACCEL | IMUGROUP_UNCOMPGYRO | IMUGROUP_TEMP | IMUGROUP_PRES
//#define GPSGROUP_PARAMS GPSGROUP_UTC | GPSGROUP_NUMSATS | GPSGROUP_FIX | GPSGROUP_POSLLA | GPSGROUP_VELECEF
//#define ATTITUDEGROUP_PARAMS ATTITUDEGROUP_NONE
//#define INSGROUP_PARAMS INSGROUP_NONE

//Minimize binary output contents for initial testing
//#define COMMONGROUP_PARAMS COMMONGROUP_NONE
//#define TIMEGROUP_PARAMS TIMEGROUP_TIMEUTC
//#define IMUGROUP_PARAMS IMUGROUP_NONE
//#define GPSGROUP_PARAMS GPSGROUP_NONE
//#define ATTITUDEGROUP_PARAMS ATTITUDEGROUP_NONE
//#define INSGROUP_PARAMS INSGROUP_NONE


#define MAX_BINARY_OUTPUT_LENGTH 100
#define DMA_RX_TIMEOUT 300
const uint8_t MS_WAIT_CAN = 10;
const uint32_t NS_TO_S = 1000000000;

uint8_t USART1_Rx_Buffer[MAX_BINARY_OUTPUT_LENGTH];
SemaphoreHandle_t USART1_DMA_Sempahore;

const uint32_t ASCII_METERS = 109;

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if(huart == &huart4)
		{
			BaseType_t xHigherPriorityTaskWoken = pdFALSE;
			xSemaphoreGiveFromISR(USART1_DMA_Sempahore, &xHigherPriorityTaskWoken);
			portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
		}
}

bool vnIMUSetup(){
	//Setup DMA on Rx
//	HAL_UART_RegisterCallback(&huart1, HAL_UART_RX_COMPLETE_CB_ID, USART1_DMA_Rx_Complete_Callback);
//
//	//TODO: Read the WHOAMI registers or equivalent to verify IMU is connected, if not throw error
//		//Register ID 0x01: Offset 0 Model string[24] – Product model number, maximum length 24 characters.
//
//	//configure binary output for raw IMU data and full state estimate
//	uint8_t buffer[CONFIG_MSG_LENGTH];
//	size_t returnedLength;
//	VnError err = VnUartPacket_genWriteBinaryOutput1(buffer, CONFIG_MSG_LENGTH, VNERRORDETECTIONMODE_CHECKSUM, &returnedLength, ASYNCMODE_BOTH, RATE1_DIVISOR,COMMONGROUP_PARAMS,TIMEGROUP_PARAMS,IMUGROUP_PARAMS,GPSGROUP_PARAMS,ATTITUDEGROUP_PARAMS,INSGROUP_PARAMS, GPSGROUP_NONE);
//	if (err == E_NONE)
//	{
//		return HAL_OK == HAL_UART_Transmit(&huart1, buffer, returnedLength, 10); //Send the command to configure IMU output format
//		//TODO: Log error to logQueue
//	}
//	else
//	{
//		//TODO: Log error to logQueue
//	}
	return false;
}

uint8_t decimalFromDouble2(double num){
	uint16_t integer_part = (uint16_t)num;
	double fractional_part = num - integer_part;
	if (fractional_part*100 > 254){
		printf("Error: getDecimalFromDouble2: overflow");
		return 0;
	}
	uint8_t decimal_part = (uint8_t)(fractional_part * 100); // This captures up to two decimal places which is the max safely stored by a 8bit uint, since max size is 255 it cant hold 999 only 99

	return decimal_part;
}

uint16_t decimalFromDouble4(double num){
	uint16_t integer_part = (uint16_t)num;
	double fractional_part = num - integer_part;
	if (fractional_part*10000 > 65534){
		printf("Error: getDecimalFromDouble4: overflow");
		return 0;
	}
	uint16_t decimal_part = (uint8_t)(fractional_part * 10000); // This captures up to 4 decimal places which is the max safely stored by a 16bit uint

	return decimal_part;
}

double minFromDeg(double deg){
	return (deg -  (uint32_t)deg)  * 60;
}

void send3VectorStateCanMsg_float(uint64_t time, float vector[3], uint8_t firstStateIDOfVector, can_msg_t msg){
	for (int i = 0; i < 3; i++) {
		build_state_est_data_msg((uint32_t) time, &vector[i], firstStateIDOfVector + i, &msg);
		xQueueSend(busQueue, &msg, MS_WAIT_CAN);
	}
}

void send3VectorStateCanMsg_double(uint64_t time, double vector[3], uint8_t firstStateIDOfVector, can_msg_t msg){
	float float_vector[3];
	for (int i = 0; i < 3; i++) {
		float_vector[i] = (float) vector[i];  // Convert double to float
	}
	send3VectorStateCanMsg_float(time, float_vector, firstStateIDOfVector, msg);
}


void vnIMUHandler(void *argument)
{
	//HAL_UART_RegisterCallback(&huart4, HAL_UART_RX_COMPLETE_CB_ID, USART1_DMA_Rx_Complete_Callback);
	USART1_DMA_Sempahore = xSemaphoreCreateBinary();
	printf_("Hello world");
	for(;;)
	{
		//Begin a receive, until we read MAX_BINARY_OUTPUT_LENGTH or the line goes idle, indicating a shorter message
		HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(&huart4, USART1_Rx_Buffer, MAX_BINARY_OUTPUT_LENGTH);
		//HAL_StatusTypeDef status = HAL_UART_Receive_DMA(&huart4, USART1_Rx_Buffer, MAX_BINARY_OUTPUT_LENGTH);
		if(status != HAL_OK)
		{
			//yikes, log an error and try again
		}
		else
		{
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
						can_msg_t msg;
						size_t packetLength = VnUartPacket_computeBinaryPacketLength(packet.data);

						//switch case bad because they all have same scope
						if (packetLength == 14){//TimeStartup only - this packet format will fail the checksum above,
							uint64_t time_startup = VnUartPacket_extractUint64(&packet);
							printf_("%lld\n", time_startup / NS_TO_S);
						}

						else if (packetLength == 38){ //time and raw IMU accel and gyro
							if(VnUartPacket_isValid(&packet)) //if the checksum is good
							{
								uint64_t time_startup = VnUartPacket_extractUint64(&packet); //time in ns -> s
								printf_("%lld\n", time_startup / NS_TO_S);
								float accel_x = VnUartPacket_extractFloat(&packet);
								float accel_y = VnUartPacket_extractFloat(&packet);
								float accel_z = VnUartPacket_extractFloat(&packet);

								float gyro_x = VnUartPacket_extractFloat(&packet);
								float gyro_y = VnUartPacket_extractFloat(&packet);
								float gyro_z = VnUartPacket_extractFloat(&packet);

								printf_("accel: %f %f %f\n", accel_x, accel_y, accel_z);
								printf_("gyro: %f %f %f\n", gyro_x, gyro_y, gyro_z);
							}
							else
							{
								printf_("bad checksum!\n");
							}
						}

						else if (packetLength == 53){
							uint64_t time_startup = VnUartPacket_extractUint64(&packet)/ NS_TO_S; //time in ns -> s

							vec3d pos = VnUartPacket_extractVec3d(&packet);
							uint8_t numSatellites = VnUartPacket_extractInt8(&packet);
							vec3f postUncertainty = VnUartPacket_extractVec3f(&packet);

							uint32_t quality_ = sqrt(pow(postUncertainty.c[0], 2) + pow(postUncertainty.c[1], 2) + pow(postUncertainty.c[2], 2));
							uint8_t quality = quality_; //the higher the number the worse the quality

							if (quality_ > 255){
								quality = 255;
							}


							build_gps_lon_msg((uint32_t) time_startup, (uint8_t) pos.c[1], (uint8_t) minFromDeg(pos.c[1]), decimalFromDouble4(minFromDeg(pos.c[1])), (int) (pos.c[1] >= 0) ? 'N' : 'S', &msg);
							xQueueSend(busQueue, &msg, MS_WAIT_CAN);
							build_gps_lat_msg((uint32_t) time_startup, (uint8_t) pos.c[1], (uint8_t) minFromDeg(pos.c[1]), decimalFromDouble4(minFromDeg(pos.c[1])), (int) (pos.c[1] >= 0) ? 'E' : 'W', &msg);
							xQueueSend(busQueue, &msg, MS_WAIT_CAN);
							build_gps_alt_msg((uint32_t) time_startup, (uint16_t)pos.c[2], decimalFromDouble2(pos.c[2]), (uint8_t) 'M', &msg);
							xQueueSend(busQueue, &msg, MS_WAIT_CAN);
							build_gps_info_msg((uint32_t) time_startup, numSatellites, quality, &msg);
							xQueueSend(busQueue, &msg, MS_WAIT_CAN);

							char msgAsString[300];
							sprintf_(msgAsString, "Time: %lli, pos: (Lat: %.3f, Lon: %.3f, Alt: %.3f) +- (%.3f, %.3f, %.3f) using %d satellites \n", time_startup, pos.c[0],pos.c[1],pos.c[2], postUncertainty.c[0],postUncertainty.c[1],postUncertainty.c[2], numSatellites);
							printf_(msgAsString);

							logInfo(SOURCE_SENSOR, msgAsString);

						}

						else if (packetLength == 92){
							uint64_t time_startup = VnUartPacket_extractUint64(&packet)/ NS_TO_S; //time in ns -> s

							vec3f angularRate = VnUartPacket_extractVec3f(&packet); //rad/s
							vec3f yprAngles = VnUartPacket_extractVec3f(&packet); //deg

							vec3d posEcef = VnUartPacket_extractVec3d(&packet); //m
							vec3f velEcef = VnUartPacket_extractVec3f(&packet); //m/s
							vec3f linAccelEcef = VnUartPacket_extractVec3f(&packet); //m/s^2 NOT INCLUDING GRAVITY


							send3VectorStateCanMsg_double(time_startup, posEcef.c,STATE_POS_X, msg); //position
							send3VectorStateCanMsg_float(time_startup, velEcef.c,STATE_VEL_X, msg); //velocity
							send3VectorStateCanMsg_float(time_startup, linAccelEcef.c,STATE_ACC_X, msg); //acceleration
							send3VectorStateCanMsg_float(time_startup, yprAngles.c,STATE_ANGLE_YAW, msg); //angle
							send3VectorStateCanMsg_float(time_startup, angularRate.c,STATE_RATE_YAW, msg); //angle rate in XYZ (TODO: NEED TO CONVERT TO YPR)



							char msgAsString[900];
							sprintf_(msgAsString,"Time: %lli, Angular Rate: (X: %.3f, Y: %.3f, Z: %.3f), YPR Angles: (Yaw: %.3f, Pitch: %.3f, Roll: %.3f), Pos ECEF: (X: %.3f, Y: %.3f, Z: %.3f), Vel ECEF: (X: %.3f, Y: %.3f, Z: %.3f), Lin Accel ECEF: (X: %.3f, Y: %.3f, Z: %.3f)\n",
								time_startup,
								angularRate.c[0], angularRate.c[1], angularRate.c[2],
								yprAngles.c[0], yprAngles.c[1], yprAngles.c[2],
								posEcef.c[0], posEcef.c[1], posEcef.c[2],
								velEcef.c[0], velEcef.c[1], velEcef.c[2],
								linAccelEcef.c[0], linAccelEcef.c[1], linAccelEcef.c[2]);
							printf_(msgAsString);
							logInfo(SOURCE_SENSOR, msgAsString);

						}



						else if (packetLength == 42){

							vec3f magVec = VnUartPacket_extractVec3f(&packet);
							vec3f accelVec = VnUartPacket_extractVec3f(&packet);
							vec3f angles = VnUartPacket_extractVec3f(&packet);

							printf_("Mag: (X: %.3f, Y: %.3f, Z: %.3f), Accel: (X: %.3f, Y: %.3f, Z: %.3f), Angles: (X: %.3f, Y: %.3f, Z: %.3f)\n", magVec.c[0], magVec.c[1], magVec.c[2], accelVec.c[0], accelVec.c[1], accelVec.c[2], angles.c[0], angles.c[1], angles.c[2]);
						}


						else{
							printf_("unhandled message format!\n");
						}

						printf_("size: %d\n", VnUartPacket_computeBinaryPacketLength(packet.data));
					}
					else{
						printf_("Invalid data");
					}

				//vTaskDelay(10000);
			}

		}
	}

}

