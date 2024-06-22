/*
 * state_estimation.c
 *
 *  Created on: Apr 13, 2024
 *      Author: joedo
 */

#define USE_ICM 0 && defined(DEBUG)
#include "state_estimation.h"
#include "ICM-20948.h"
#include "log.h"
#include "can_handler.h"
#if USE_ICM == 1
#include <time.h>
#endif

#define SAMPLE_RATE 100 // replace this with actual sample rate

QueueHandle_t IMUDataHandle;

bool unpackIMUData(FusionVector *gyroscope, FusionVector *accelerometer, FusionVector *magnetometer, uint32_t *deltaTimems)
{
	rawIMUPacked data;
	if(xQueueReceive(IMUDataHandle, &data, 50) == pdTRUE)
	{
		*gyroscope = data.gyroscope;
		*accelerometer = data.accelerometer;
		*magnetometer = data.magnetometer;
		*deltaTimems = data.deltaTimems;
		return true;
	}
	return false;
}

void stateEstTask(void *arguments)
{
		//create queue to store IMU data passed from the VN handler
		IMUDataHandle = xQueueCreate(1, sizeof(rawIMUPacked));

	    // Define calibration (replace with actual calibration data if available)
		//all of these have this missing braces error..... that seems to just be a bug...?
		const FusionMatrix gyroscopeMisalignment = {
				.element = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f}
		};
		const FusionVector gyroscopeSensitivity = {
				.axis = {0.0f, 0.0f, 0.0f}
		};
		const FusionVector gyroscopeOffset = {
				.axis = {0.0f, 0.0f, 0.0f}
		};
		const FusionMatrix accelerometerMisalignment = {
					.element = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f}
			};
		const FusionVector accelerometerSensitivity = {
				.axis = {0.0f, 0.0f, 0.0f}
		};
		const FusionVector accelerometerOffset = {
				.axis = {0.0f, 0.0f, 0.0f}
		};

		// TODO populate with values using VN Control Center's builtin magnetic bias calculator
		const FusionMatrix softIronMatrix = {
				.element = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f}
		};
		const FusionVector hardIronOffset = {
				.axis = {0.0f, 0.0f, 0.0f}
		};

		// Create and Init Fusion objects
		FusionOffset offset;
		FusionAhrs ahrs;

		FusionOffsetInitialise(&offset, SAMPLE_RATE);
		FusionAhrsInitialise(&ahrs);

		// Set AHRS algorithm settings
		 const FusionAhrsSettings settings = {
				 .convention = FusionConventionNwu,
		         .gain = 0.5f,
		         .gyroscopeRange = 2000.0f, // checked, this is correct
		         .accelerationRejection = 10.0f,
		         .magneticRejection = 10.0f,
		         .recoveryTriggerPeriod = 5 * SAMPLE_RATE, /* 5 seconds */
		 };
		 FusionAhrsSetSettings(&ahrs, &settings);

		 #if USE_ICM == 1
		 //if we are using the ICM IMU, initialize it
		 bool res = true;
		 res &= ICM_20948_init();
		 res &= ICM_20948_check_sanity();
		 res &= MAG_Self_Test();
		 if(!res)
		 {
			 //throw an error?
		 }
		#endif

		 FusionVector gyroscope;
		 FusionVector accelerometer;
		 FusionVector magnetometer;

		 // This loop should repeat each time new gyroscope data is available
		 while (true) {
			#if USE_ICM == 1
			 //If using the ICM, do a synchronous (wrt to state estimation) read on all sensors
			 float xData;
			 float yData;
			 float zData;

			 //TODO: wrap this operation in a single function call that populates Fusion Vectors and spits out a single bool
			 // gyroscope data in degrees/s
			 if(ICM_20948_get_gyro_converted(&xData, &yData, &zData)){
				 gyroscope.axis.x = xData;
				 gyroscope.axis.y = yData;
				 gyroscope.axis.z = zData;
			 }

			 //accelerometer data in m/s^2
			 if(ICM_20948_get_accel_converted(&xData, &yData, &zData)) {
				 accelerometer.axis.x = xData;
				 accelerometer.axis.y = yData;
				 accelerometer.axis.z = zData;
			 }

			 // magnetometer data in arbitrary units //TODO Figure out units
			 if(ICM_20948_get_mag_converted(&xData, &yData, &zData)) {
				 magnetometer.axis.x = xData;
				 magnetometer.axis.y = yData;
				 magnetometer.axis.z = zData;
			 }

		 	 // Calculate delta time (in seconds) to account for gyroscope sample clock error
			 //TODO Replace anything to do with time.h with a FreeRTOS or HAL service
			 static clock_t previousTimestamp;
			 const clock_t timestamp = clock(); // replace this with actual gyroscope timestamp
			 const float deltaTime = (float) (timestamp - previousTimestamp) / (float) CLOCKS_PER_SEC;
			 previousTimestamp = timestamp;
			#else
			 uint32_t deltaTimeMS;
			 if(unpackIMUData(&gyroscope, &accelerometer, &magnetometer, &deltaTimeMS) == false)
			 {
				 logError(SOURCE_STATE_EST, "Failed to get raw IMU data");
			 }
			 float deltaTime = (float) deltaTimeMS / 1000.0; //yes I realized deltaTime was a float in s after the fact sue me
			#endif


			 // Apply calibration
			 gyroscope = FusionCalibrationInertial(gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);
		 	 accelerometer = FusionCalibrationInertial(accelerometer, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset);
		 	 magnetometer = FusionCalibrationMagnetic(magnetometer, softIronMatrix, hardIronOffset);

		 	 // Update gyroscope offset correction algorithm
		 	 gyroscope = FusionOffsetUpdate(&offset, gyroscope);

		 	 // Update gyroscope AHRS algorithm
		 	 FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, magnetometer, deltaTime);

		 	 //calculate algorithm outputs
		 	 const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
		 	logInfo(SOURCE_STATE_EST, "EuRoll %d, EuPitch %d, EuYaw %d mdeg", (int) (euler.angle.roll * 1000), (int) (euler.angle.pitch * 1000), (int) (euler.angle.yaw * 1000));

		 	can_msg_t msg;
		 	if(build_state_est_data_msg(69, &euler.angle.roll, STATE_ANGLE_ROLL, &msg)) xQueueSend(busQueue, &msg, 10);
		 	if(build_state_est_data_msg(70, &euler.angle.pitch, STATE_ANGLE_PITCH, &msg)) xQueueSend(busQueue, &msg, 10);
		 	if(build_state_est_data_msg(71, &euler.angle.yaw, STATE_ANGLE_YAW, &msg)) xQueueSend(busQueue, &msg, 10);

		 	//Push accelerationv alues for debugging
		 	const FusionVector earth = FusionAhrsGetEarthAcceleration(&ahrs);
		 	logDebug(SOURCE_STATE_EST, "AccelX %d, AccelY %d, AccelZ %d", earth.axis.x, earth.axis.y, earth.axis.z);

		 	 vTaskDelay(20); //TODO replace this with vTaskDelayUntil
		 }
}
