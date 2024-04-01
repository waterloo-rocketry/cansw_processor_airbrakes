/*
 * vn_handler.c
 *
 *  Created on: Mar 24, 2024
 *      Author: joedo
 */


#include "vn_handler.h"
#include "vn/protocol/upack.h"

#define CONFIG_MSG_LENGTH 100
#define VN_200_IMURATE 800 //Hz
#define BINARY_OUT_RATE 10 //Hz
#define RATE_DIVISOR VN_200_IMURATE / BINARY_OUT_RATE

//See "vnenum.h" and the ICD for parameter definitions
/*
#define COMMONGROUP_PARAMS COMMONGROUP_QUATERNION | COMMONGROUP_POSITION | COMMONGROUP_VELOCITY | COMMONGROUP_ACCEL
#define TIMEGROUP_PARAMS TIMEGROUP_TIMEUTC | TIMEGROUP_TIMEUTC
#define IMUGROUP_PARAMS IMUGROUP_IMUSTATUS | IMUGROUP_SENSSAT | IMUGROUP_UNCOMPMAG | IMUGROUP_UNCOMPACCEL | IMUGROUP_UNCOMPGYRO | IMUGROUP_TEMP | IMUGROUP_PRES
#define GPSGROUP_PARAMS GPSGROUP_UTC | GPSGROUP_NUMSATS | GPSGROUP_FIX | GPSGROUP_POSLLA | GPSGROUP_VELECEF
#define ATTITUDEGROUP_PARAMS ATTITUDEGROUP_NONE
#define INSGROUP_PARAMS INSGROUP_NONE
*/

//Minimize binary output contents for intial testing
#define COMMONGROUP_PARAMS COMMONGROUP_NONE
#define TIMEGROUP_PARAMS TIMEGROUP_TIMEUTC
#define IMUGROUP_PARAMS IMUGROUP_NONE
#define GPSGROUP_PARAMS GPSGROUP_NONE
#define ATTITUDEGROUP_PARAMS ATTITUDEGROUP_NONE
#define INSGROUP_PARAMS INSGROUP_NONE

xQueueHandle binaryOutQueue;
xQueueHandle rawIMUQueue;
xQueueHandle vnStateQueue;

int vnIMUSetup(){
	//Create the necessary queues
	rawIMUQueue = xQueueCreate(4, sizeof(vn_imu_data_t));
	vnStateQueue = xQueueCreate(4, sizeof(vn_full_state_t));
	binaryOutQueue = xQueueCreate(1, sizeof(vn_binary_out));
	//TODO: Read the WHOAMI registers or equivalent to verify IMU is connected, if not throw error
		//Register ID 0x01: Offset 0 Model string[24] – Product model number, maximum length 24 characters.

	//configure binary output for raw IMU data and full state estimate
		//RateDivisor sets the output rate as a function of the sensor ImuRate (800 Hz for the VN-200). So to achieve a 40 Hz output rate, the RateDivisor is set to 20.
	uint8_t buffer[CONFIG_MSG_LENGTH];
	size_t returnedLength;
	VnError err = VnUartPacket_genWriteBinaryOutput1(buffer, CONFIG_MSG_LENGTH, VNERRORDETECTIONMODE_CHECKSUM, &returnedLength, ASYNCMODE_BOTH, RATE_DIVISOR,COMMONGROUP_PARAMS,TIMEGROUP_PARAMS,IMUGROUP_PARAMS,GPSGROUP_PARAMS,ATTITUDEGROUP_PARAMS,INSGROUP_PARAMS, GPSGROUP_NONE);
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

void vnIMUHandler(void *argument){
	//block on USART1 ready
	for(;;)
	{
		vn_binary_out output;
		if(xQueueReceive(binaryOutQueue, &output, 1) == pdTRUE)
		{
			uint8_t *buffer = output.buffer;
			VnUartPacket packet;
			packet.curExtractLoc = 0;
			packet.data = buffer;
			packet.length = DATA_MSG_LENGTH_MAX;
			//uint16_t *dump;
			//uint16_t *time_group;
			//VnUartPacket_parseBinaryOutput(&packet, dump, dump, dump, dump, time_group, dump, dump, dump, dump, dump);

			TimeUtc timestamp = VnUartPacket_extractTimeUtc(&packet); //grab timestamp from binary output - this should be the only data in there
		}

	}
}

//This method is really bad because we copy a full byte message into local memory, then again into a queue
//We should be using DMA
// TODO: Use DMA for this

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1) //we received UART data from the IMU
	{
		vn_binary_out output;
		HAL_UART_Receive(huart, output.buffer, DATA_MSG_LENGTH_MAX, 10);

		BaseType_t *xHigherPriorityTaskWoken;
		xQueueSendFromISR(binaryOutQueue, &output, xHigherPriorityTaskWoken); //Dump the bytes we received into a queue for decoding
		portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
	}
}
