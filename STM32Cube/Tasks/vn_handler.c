/*
 * vn_handler.c
 *
 *  Created on: Mar 24, 2024
 *      Author: joedo
 */

#include <stdio.h>
#include "vn_handler.h"
#include "vn/protocol/upack.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "log.h"
#include "stm32h7xx_hal.h"

//General VN module params
#define CONFIG_MSG_LENGTH 100
#define VN_200_BINARYOUT_RATE 800 //Hz
#define MAX_BINARY_OUTPUT_LENGTH 200
#define DMA_RX_TIMEOUT 300

//See "vnenum.h" and the ICD for parameter definitions
//Binary Output #1 - Testing
#define BINARY1_OUT_RATE 1 //Hz
#define RATE1_DIVISOR VN_200_BINARYOUT_RATE / BINARY1_OUT_RATE //RateDivisor sets the output rate as a function of the sensor base rate (800 Hz for the VN-200)

	//Minimize binary output contents for initial testing
#define COMMONGROUP_PARAMS1 COMMONGROUP_NONE
#define TIMEGROUP_PARAMS1 TIMEGROUP_TIMEUTC
#define IMUGROUP_PARAMS1 IMUGROUP_NONE
#define GPSGROUP_PARAMS1 GPSGROUP_NONE
#define ATTITUDEGROUP_PARAMS1 ATTITUDEGROUP_NONE
#define INSGROUP_PARAMS1 INSGROUP_NONE

//Binary Output #2 - IMU Output
#define BINARY2_OUT_RATE 10 //Hz, this needs to approximately match running frequency of state estimation
#define RATE2_DIVISOR VN_200_BINARYOUT_RATE / BINARY2_OUT_RATE //RateDivisor sets the output rate as a function of the sensor base rate (800 Hz for the VN-200)

#define COMMONGROUP_PARAMS2 COMMONGROUP_NONE
#define TIMEGROUP_PARAMS2 TIMEGROUP_TIMEUTC
#define IMUGROUP_PARAMS2 IMUGROUP_IMUSTATUS | IMUGROUP_SENSSAT | IMUGROUP_UNCOMPMAG | IMUGROUP_UNCOMPACCEL | IMUGROUP_UNCOMPGYRO | IMUGROUP_TEMP | IMUGROUP_PRES | IMUGROUP_SENSSAT
#define GPSGROUP_PARAMS2 GPSGROUP_NONE
#define ATTITUDEGROUP_PARAMS2 ATTITUDEGROUP_NONE
#define INSGROUP_PARAMS2 INSGROUP_NONE


uint8_t USART1_Rx_Buffer[MAX_BINARY_OUTPUT_LENGTH];
SemaphoreHandle_t USART1_DMA_Sempahore;

//IMU Results
SemaphoreHandle_t vnIMUResultMutex;
vec3f accel;
vec3f gyro;
vec3f mag;
float temp;
float pressure;
TimeUtc readTime;
TimeUtc lastReadTime;

void USART1_DMA_Rx_Complete_Callback(UART_HandleTypeDef *huart)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(USART1_DMA_Sempahore, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

uint32_t TimeUTCDiffms(TimeUtc end, TimeUtc start)
{
	return ((uint32_t) end.ms + (uint32_t) end.sec * 1000 + (uint32_t) end.min * 60 * 1000 + (uint32_t) end.hour * 60 * 60 * 1000) - ((uint32_t) start.ms + (uint32_t) start.sec * 1000 + (uint32_t) start.min * 60 * 1000 + (uint32_t) start.hour * 60 * 60 * 1000);
}

bool vnIMUSetup(){
	//Create semaphore for USART1 Rx DMA
	USART1_DMA_Sempahore = xSemaphoreCreateBinary();
	vnIMUResultMutex = xSemaphoreCreateMutex();
	//Setup DMA on Rx
	HAL_UART_RegisterCallback(&huart1, HAL_UART_RX_COMPLETE_CB_ID, USART1_DMA_Rx_Complete_Callback);

	//TODO: Read the WHOAMI registers or equivalent to verify IMU is connected, if not throw error
		//Register ID 0x01: Offset 0 Model string[24] – Product model number, maximum length 24 characters.

	//Send messages to configure VN-300 binary output registers
	uint8_t buffer[CONFIG_MSG_LENGTH];
	size_t returnedLength;

	//configure binary output #1
	VnError err = VnUartPacket_genWriteBinaryOutput1(buffer, CONFIG_MSG_LENGTH, VNERRORDETECTIONMODE_CHECKSUM, &returnedLength, ASYNCMODE_BOTH, RATE1_DIVISOR,
		COMMONGROUP_PARAMS1, TIMEGROUP_PARAMS1, IMUGROUP_PARAMS1, GPSGROUP_PARAMS1, ATTITUDEGROUP_PARAMS1, INSGROUP_PARAMS1, GPSGROUP_NONE);

	if (err == E_NONE)
	{
		return HAL_OK == HAL_UART_Transmit(&huart1, buffer, returnedLength, 10); //Send the command to configure IMU output format
		//TODO: Log error to logQueue
	}
	else
	{
		//TODO: Log error to logQueue
	}

	//configure binary output #2
	err = VnUartPacket_genWriteBinaryOutput1(buffer, CONFIG_MSG_LENGTH, VNERRORDETECTIONMODE_CHECKSUM, &returnedLength, ASYNCMODE_BOTH, RATE2_DIVISOR,
		COMMONGROUP_PARAMS2, TIMEGROUP_PARAMS2, IMUGROUP_PARAMS2, GPSGROUP_PARAMS2,ATTITUDEGROUP_PARAMS2,INSGROUP_PARAMS2, GPSGROUP_NONE);

	if (err == E_NONE)
	{
		return HAL_OK == HAL_UART_Transmit(&huart1, buffer, returnedLength, 10); //Send the command to configure IMU output format
		//TODO: Log error to logQueue
	}
	else
	{
		//TODO: Log error to logQueue
	}


	return false;
}


void vnIMUHandler(void *argument)
{
	for(;;)
	{

		HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(&huart1, USART1_Rx_Buffer, MAX_BINARY_OUTPUT_LENGTH); //TODO no idea if this is the correct way to get data from the DMA
		if(status != HAL_OK)
		{
			//yikes, log an error and try again
		}
		else
		{
			if(xSemaphoreTake(USART1_DMA_Sempahore, 10) != pdTRUE)
			{
				//we timed out, throw an error
			}
			else
			{
				VnUartPacket packet;
				packet.curExtractLoc = 0;
				packet.data = USART1_Rx_Buffer;
				packet.length = MAX_BINARY_OUTPUT_LENGTH;
				uint16_t *dump;
				uint16_t time_group;
				uint16_t imu_group;
				uint16_t gps_group;
				uint16_t ins_group;

				TimeUtc timestamp;

				if(VnUartPacket_type(&packet) == PACKETTYPE_BINARY)
				{
					//TODO: Detect message type properly
					VnUartPacket_parseBinaryOutput(&packet, dump, dump, dump, dump, &time_group, &imu_group, &gps_group, dump, &ins_group, dump);
					if(time_group & TIMEGROUP_TIMEUTC)
					{
						LogData_t msg;
						timestamp = VnUartPacket_extractTimeUtc(&packet);

					}
					if(imu_group != IMUGROUP_NONE)
					{
						//TODO make sure we aren't writing bad data, check IMU status or smth
						if(xSemaphoreTake(vnIMUResultMutex, 10) != pdTRUE)
						{
							//we timed out, throw an error
						}

						VnUartPacket_parseVNIMU(&packet, &mag, &accel, &gyro, &temp, &pressure);
						lastReadTime = readTime;
						readTime = timestamp;

						//Release IMU data mutex
						xSemaphoreGive(vnIMUResultMutex);

					}
					if(gps_group != GPSGROUP_NONE)
					{
						//decode the GPS data and push it to log
						//Every so often, build GPS canlib messages and send them over the bus as well
						//also we have 2 GPSs... no provisions in canlib for THAT
					}
					if(ins_group != INSGROUP_NONE)
					{
						//fancy state estimation results!
						//dump these to the log
					}

				}
				else if(VnUartPacket_type(&packet) == PACKETTYPE_ASCII)
				{
					//TODO: dump ASCII packet contents... to debug log?
				}
			}

		}
	}

}

bool writeIMUData(FusionVector *gyroscope, FusionVector *accelerometer, FusionVector *magnetometer, uint32_t *deltaTimems)
{
	if(xSemaphoreTake(vnIMUResultMutex, 10) != pdTRUE)
	{
		return false; //couldn't get mutex before timeout
	}

	accelerometer->array[0] = accel.c[0]; //X
	accelerometer->array[1] = accel.c[1]; //Y
	accelerometer->array[2] = accel.c[2]; //Z

	gyroscope->array[0] = gyro.c[0]; //X
	gyroscope->array[1] = gyro.c[1]; //Y
	gyroscope->array[2] = gyro.c[2]; //Z

	magnetometer->array[0] = mag.c[0]; //X
	magnetometer->array[1] = mag.c[1]; //Y
	magnetometer->array[2] = mag.c[2]; //Z

	*deltaTimems = TimeUTCDiffms(readTime, lastReadTime);

	//Release IMU data mutex
	xSemaphoreGive(vnIMUResultMutex);

	return true;
}




