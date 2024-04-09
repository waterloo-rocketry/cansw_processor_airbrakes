/*
 * vn_handler.c
 *
 *  Created on: Mar 24, 2024
 *      Author: joedo
 */


#include "vn_handler.h"
#include "vn/protocol/upack.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "fake_logging.h"
#include "stm32h7xx_hal.h"


//RateDivisor sets the output rate as a function of the sensor ImuRate (800 Hz for the VN-200)
#define CONFIG_MSG_LENGTH 100
#define VN_200_IMURATE 800 //Hz

//Binary Output #1
#define BINARY1_OUT_RATE 10 //Hz
#define RATE1_DIVISOR VN_200_IMURATE / BINARY1_OUT_RATE

//See "vnenum.h" and the ICD for parameter definitions
/*
#define COMMONGROUP_PARAMS COMMONGROUP_QUATERNION | COMMONGROUP_POSITION | COMMONGROUP_VELOCITY | COMMONGROUP_ACCEL
#define TIMEGROUP_PARAMS TIMEGROUP_TIMEUTC | TIMEGROUP_TIMEUTC
#define IMUGROUP_PARAMS IMUGROUP_IMUSTATUS | IMUGROUP_SENSSAT | IMUGROUP_UNCOMPMAG | IMUGROUP_UNCOMPACCEL | IMUGROUP_UNCOMPGYRO | IMUGROUP_TEMP | IMUGROUP_PRES
#define GPSGROUP_PARAMS GPSGROUP_UTC | GPSGROUP_NUMSATS | GPSGROUP_FIX | GPSGROUP_POSLLA | GPSGROUP_VELECEF
#define ATTITUDEGROUP_PARAMS ATTITUDEGROUP_NONE
#define INSGROUP_PARAMS INSGROUP_NONE
*/

//Minimize binary output contents for initial testing
#define COMMONGROUP_PARAMS COMMONGROUP_NONE
#define TIMEGROUP_PARAMS TIMEGROUP_TIMEUTC
#define IMUGROUP_PARAMS IMUGROUP_NONE
#define GPSGROUP_PARAMS GPSGROUP_NONE
#define ATTITUDEGROUP_PARAMS ATTITUDEGROUP_NONE
#define INSGROUP_PARAMS INSGROUP_NONE


#define MAX_BINARY_OUTPUT_LENGTH 200
#define DMA_RX_TIMEOUT 300
uint8_t USART1_Rx_Buffer[MAX_BINARY_OUTPUT_LENGTH];
SemaphoreHandle_t USART1_DMA_Sempahore;

void USART1_DMA_Rx_Complete_Callback(UART_HandleTypeDef *huart)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(USART1_DMA_Sempahore, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

int vnIMUSetup(){
	//Setup DMA on Rx
	HAL_UART_RegisterCallback(&huart1, HAL_UART_RX_COMPLETE_CB_ID, USART1_DMA_Rx_Complete_Callback);
	//TODO: Read the WHOAMI registers or equivalent to verify IMU is connected, if not throw error
		//Register ID 0x01: Offset 0 Model string[24] – Product model number, maximum length 24 characters.

	//configure binary output for raw IMU data and full state estimate
	uint8_t buffer[CONFIG_MSG_LENGTH];
	size_t returnedLength;
	VnError err = VnUartPacket_genWriteBinaryOutput1(buffer, CONFIG_MSG_LENGTH, VNERRORDETECTIONMODE_CHECKSUM, &returnedLength, ASYNCMODE_BOTH, RATE1_DIVISOR,COMMONGROUP_PARAMS,TIMEGROUP_PARAMS,IMUGROUP_PARAMS,GPSGROUP_PARAMS,ATTITUDEGROUP_PARAMS,INSGROUP_PARAMS, GPSGROUP_NONE);
	if (err == E_NONE)
	{
		return HAL_UART_Transmit(&huart1, buffer, returnedLength, 10); //Send the command to configure IMU output format
		//TODO: Log error to logQueue
	}
	else
	{
		//TODO: Log error to logQueue
	}
	return -1;
}


void vnIMUHandler(void *argument)
{
	USART1_DMA_Sempahore = xSemaphoreCreateBinary();
	for(;;)
	{
		xSemaphoreTake(USART1_DMA_Sempahore, 0); //Take semaphore with no timeout to ensure state
		HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(&huart1, USART1_Rx_Buffer, MAX_BINARY_OUTPUT_LENGTH);
		if(status != HAL_OK)
		{
			//yikes, log an error and try again
		}
		else
		{
			if(xSemaphoreTake(USART1_DMA_Sempahore, 0) != pdTRUE)
			{
				//we timed out, throw an error
			}
			else //we got data in the buffer! this implies the shared memory region is safe to access
			{
				VnUartPacket packet;
				packet.curExtractLoc = 0;
				packet.data = USART1_Rx_Buffer;
				packet.length = MAX_BINARY_OUTPUT_LENGTH;
				uint16_t *dump;
				uint16_t *time_group;
				uint16_t *imu_group;
				uint16_t *gps_group;
				uint16_t *ins_group;

				if(VnUartPacket_type(&packet) == PACKETTYPE_BINARY)
				{
					//TODO: Detect message type properly
					VnUartPacket_parseBinaryOutput(&packet, dump, dump, dump, dump, time_group, imu_group, gps_group, dump, ins_group, dump);
					if(*time_group & TIMEGROUP_TIMEUTC)
					{
						logMsg_t msg;
						TimeUtc timestamp = VnUartPacket_extractTimeUtc(&packet);
						sprintf(msg.data, "%d:%d:%d", timestamp.hour, timestamp.min, timestamp.sec);
						xQueueSend(logQueue, &msg, 10);
					}
					if(*imu_group != IMUGROUP_NONE)
					{
						//assume this is the IMU type message we expect
						//TODO: Build a valid IMU message for state est and push it to queue
					}
					if(*gps_group != GPSGROUP_NONE)
					{
						//decode the GPS data and push it to log
						//Every so often, build GPS canlib messages and send them over the bus as well
						//also we have 2 GPSs... no provisions in canlib for THAT
					}
					if(*ins_group != INSGROUP_NONE)
					{
						//fancy state estimation results!
						//dump these to the logging queue
					}
				}

			}
		}
	}

}


