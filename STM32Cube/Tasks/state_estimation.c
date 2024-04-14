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
#include <time.h>

#define SAMPLE_RATE 100 // replace this with actual sample rate
#define USE_ICM 0

void stateEstTask(void *arguments)
{
	/* USER CODE BEGIN stateEstimationTask */

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
			 ICM_20948_get_gyro_converted(&xData, &yData, &zData);
			 gyroscope = {
			 						.axis = {0.0f, 0.0f, 0.0f}
			 				}; // gyroscope data in degrees/s
			 ICM_20948_get_accel_converted(&xData, &yData, &zData);
			 accelerometer = {
			 						.axis = {0.0f, 0.0f, 0.0f}
			 				}; // accelerometer data in g
			 ICM_20948_get_mag_converted(&xData, &yData, &zData);
			 magnetometer = {
			 						.axis = {0.0f, 0.0f, 0.0f}
			 				}; // magnetometer data in arbitrary units //TODO Figure out units

		 	 // Calculate delta time (in seconds) to account for gyroscope sample clock error

			 //TODO Replace anything to do with time.h with a FreeRTOS or HAL service
			 static clock_t previousTimestamp;
			 const clock_t timestamp = clock(); // replace this with actual gyroscope timestamp
			 const float deltaTime = (float) (timestamp - previousTimestamp) / (float) CLOCKS_PER_SEC;
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
		 	 //we only care about attitude says joe but I'm outputting this anyways for now
		 	 const FusionVector earth = FusionAhrsGetEarthAcceleration(&ahrs);

		 	 unsigned char dataStr[100];
		 	 //this has an error even if you change the setting, but it will build
		 	 //int  length = snprintf(dataStr, 100, "Roll %0.1f, Pitch %0.1f, Yaw %0.1f, X %lf, Y %lf, Z %lf\n",
		 	//	               euler.angle.roll, euler.angle.pitch, euler.angle.yaw,
		 	//	               earth.axis.x, earth.axis.y, earth.axis.z) ;
		 	 //HAL_UART_Transmit(&huart4, dataStr, length, 500);


		 	 //put data somewhere, still not clear on what I'm meant to be doing with it tbh
		 	 //sprintf(???, "Roll %0.1f, Pitch %e, Yaw %0.1f, X %0.1f, Y %0.1f, Z %0.1f\n",
		               //euler.angle.roll, euler.angle.pitch, euler.angle.yaw,
		               //earth.axis.x, earth.axis.y, earth.axis.z);

		 	//printf(to_string(euler.angle.roll) + to_string(euler.angle.pitch) + to_string(euler.angle.yaw));
		 	 vTaskDelay(1000); //TODO replace this with vTaskDelayUntil
		 }
}
