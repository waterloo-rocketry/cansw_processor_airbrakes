/*
 * state_estimation.c
 *
 *  Created on: Apr 13, 2024
 *      Author: joedo
 */


#include "state_estimation.h"
#include "ICM-20948.h"
#include "Fusion.h"
#include "vn_handler.h"
#include "log.h"
#include "can_handler.h"
#include "millis.h"
#include "printf.h"

#define SAMPLE_RATE 100 // replace this with actual sample rate
#define USE_ICM 1

void stateEstTask(void *arguments)
{
    float previousTimestamp = 0;

	/* USER CODE BEGIN stateEstimationTask */

	    // Define calibration (replace with actual calibration data if available)
		//all of these have this missing braces error..... that seems to just be a bug...?
		const FusionMatrix gyroscopeMisalignment = {
				.element = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f}
		};
		const FusionVector gyroscopeSensitivity = {
				.axis = {1.0f, 1.0f, 1.0f}
		};
		const FusionVector gyroscopeOffset = {
				.axis = {0.0f, 0.0f, 0.0f}
		};
		const FusionMatrix accelerometerMisalignment = {
					.element = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f}
			};
		const FusionVector accelerometerSensitivity = {
				.axis = {1.0f, 1.0f, 1.0f}
		};
		const FusionVector accelerometerOffset = {
				.axis = {0.0f, 0.0f, 0.0f}
		};

		// TODO Figure out how to check soft and hard iron (?)
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
		 res &= ICM_20948_setup();
		 res &= ICM_20948_check_sanity();
		// res &= MAG_Self_Test();
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

			 // magnetometer data in microteslas
			 if(ICM_20948_get_mag_converted(&xData, &yData, &zData)) {
				 magnetometer.axis.x = xData;
				 magnetometer.axis.y = yData;
				 magnetometer.axis.z = zData;
			 }

		 	 // Calculate delta time (in seconds) to account for gyroscope sample clock error
			 const float timestamp = millis_(); // replace this with actual gyroscope timestamp
			 const float deltaTime = (float) (timestamp - previousTimestamp) / 1000;
			 previousTimestamp = timestamp;


			#else
			 //wait for the VN data buffer mutex to be available
			 if(xSemaphoreTake(vnIMUResultMutex, 100) != pdTRUE)
			 {
				 //we timed out, something is holding onto the mutex
			 }

			 uint32_t deltaTimeMS;
			 writeIMUData(&gyroscope, &accelerometer, &magnetometer, &deltaTimeMS);
			 float deltaTime = (float) deltaTimeMS / 1000.0; //yes I realized deltaTime was a float in s after the fact sue me

			 xSemaphoreGive(vnIMUResultMutex);

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
		 	logInfo("stateest", "EuRoll %f, EuPitch %f, EuYaw %f", euler.angle.roll, euler.angle.pitch, euler.angle.yaw);
		 	//printf_("EuRoll %f, EuPitch %f, EuYaw %f\n", euler.angle.roll, euler.angle.pitch, euler.angle.yaw);
		 	can_msg_t msg;
		 	if(build_state_est_data_msg(69, &euler.angle.roll, STATE_ANGLE_ROLL, &msg)) xQueueSend(busQueue, &msg, 10);
		 	if(build_state_est_data_msg(70, &euler.angle.pitch, STATE_ANGLE_PITCH, &msg)) xQueueSend(busQueue, &msg, 10);
		 	if(build_state_est_data_msg(71, &euler.angle.yaw, STATE_ANGLE_YAW, &msg)) xQueueSend(busQueue, &msg, 10);

		 	//Push accelerationv alues for debugging
		 	const FusionVector earth = FusionAhrsGetEarthAcceleration(&ahrs);
		 	logDebug("stateest", "AccelX %d, AccelY %d, AccelZ %d", earth.axis.x, earth.axis.y, earth.axis.z);

		 	 vTaskDelay(20); //TODO replace this with vTaskDelayUntil
		 }
}
