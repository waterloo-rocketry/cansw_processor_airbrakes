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

extern UART_HandleTypeDef huart4;


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


#define MAX_BINARY_OUTPUT_LENGTH 50
#define DMA_RX_TIMEOUT 300
uint8_t USART1_Rx_Buffer[MAX_BINARY_OUTPUT_LENGTH];
SemaphoreHandle_t USART1_DMA_Sempahore;

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


void vnIMUHandler(void *argument)
{
	//HAL_UART_RegisterCallback(&huart4, HAL_UART_RX_COMPLETE_CB_ID, USART1_DMA_Rx_Complete_Callback);
	USART1_DMA_Sempahore = xSemaphoreCreateBinary();
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
						uint64_t time_startup;
						switch(VnUartPacket_computeBinaryPacketLength(packet.data)){
						case 14: //TimeStartup only - this packet format will fail the checksum above,
							time_startup = VnUartPacket_extractUint64(&packet);
							printf_("%f\n", time_startup / 1000000000.0);
							break;

						case 38: //time and raw IMU accel and gyro
							if(VnUartPacket_isValid(&packet)) //if the checksum is bad
							{
								time_startup = VnUartPacket_extractUint64(&packet); //time in ns -> s
								printf_("%f\n", time_startup / 1000000000.0);
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
							break;

						default:
							printf_("unhandled message format!\n");
							break;
						}
						printf_("size: %d\n", VnUartPacket_computeBinaryPacketLength(packet.data));
					}

				//vTaskDelay(10000);
			}

		}
	}

}

