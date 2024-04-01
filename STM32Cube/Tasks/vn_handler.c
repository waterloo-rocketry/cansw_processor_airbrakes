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

//See "vnenum.h" for paramater definitions
#define COMMONGROUP_PARAMS COMMONGROUP_QUATERNION | COMMONGROUP_POSITION | COMMONGROUP_VELOCITY | COMMONGROUP_ACCEL
#define TIMEGROUP_PARAMS TIMEGROUP_TIMEUTC | TIMEGROUP_TIMEUTC
#define IMUGROUP_PARAMS IMUGROUP_IMUSTATUS | IMUGROUP_SENSSAT | IMUGROUP_UNCOMPMAG | IMUGROUP_UNCOMPACCEL | IMUGROUP_UNCOMPGYRO | IMUGROUP_TEMP | IMUGROUP_PRES
#define GPSGROUP_PARAMS GPSGROUP_UTC | GPSGROUP_NUMSATS | GPSGROUP_FIX | GPSGROUP_POSLLA | GPSGROUP_VELECEF
#define ATTITUDEGROUP_PARAMS ATTITUDEGROUP_NONE
#define INSGROUP_PARAMS INSGROUP_NONE

xQueueHandle rawIMUQueue;
xQueueHandle vnStateQueue;

int vnIMUSetup(){
	//Create the handler task
	//Create the necessary queues

	//Read the WHOAMI registers or equivalent to verify IMU is connected, if not throw error
		//Register ID 0x01: Offset 0 Model string[24] – Product model number, maximum length 24 characters.

	//configure binary output for raw IMU data and full state estimate
		//ateDivisor which sets the output rate as a function of the sensor ImuRate (800 Hz for the VN-200). So to achieve a 40 Hz output rate, the RateDivisor is set to 20.
	uint8_t buffer[CONFIG_MSG_LENGTH];
	size_t returnedLength;
	VnError err = VnUartPacket_genWriteBinaryOutput1(buffer, sizeof(uint8_t)*CONFIG_MSG_LENGTH,VNERRORDETECTIONMODE_CHECKSUM, &returnedLength, ASYNCMODE_BOTH, RATE_DIVISOR,COMMONGROUP_PARAMS,TIMEGROUP_PARAMS,IMUGROUP_PARAMS,GPSGROUP_PARAMS,ATTITUDEGROUP_PARAMS,INSGROUP_PARAMS, GPSGROUP_NONE);
	return err;
}

void vnIMUHandler(void *argument){
	//block on USART1 ready
	//retrieve USART1 data and separate into appropriate queues
	//
}
