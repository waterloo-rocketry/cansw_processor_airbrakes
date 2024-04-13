/*
 * state_estimation.c
 *
 *  Created on: Apr 13, 2024
 *      Author: joedo
 */


#include "state_estimation.h"
#include "Fusion.h"
#include <time.h>

#define SAMPLE_RATE 100 // replace this with actual sample rate

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

		// Figure out how to check soft and hard iron (?)
		const FusionMatrix softIronMatrix = {
				.element = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f}
		};
		const FusionVector hardIronOffset = {
				.axis = {0.0f, 0.0f, 0.0f}
		};

		// Initialise algorithms
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

		 // This loop should repeat each time new gyroscope data is available
		 while (true) {

			 // Acquire latest sensor data
			 // how the hell do I do that again
			 const clock_t timestamp = clock(); // replace this with actual gyroscope timestamp
			 //how to use vn300
			 FusionVector gyroscope = {
						.axis = {0.0f, 0.0f, 0.0f}
				}; // replace this with actual gyroscope data in degrees/s
			 FusionVector accelerometer = {
						.axis = {0.0f, 0.0f, 0.0f}
				}; // replace this with actual accelerometer data in g
			 FusionVector magnetometer = {
						.axis = {0.0f, 0.0f, 0.0f}
				}; // replace this with actual magnetometer data in arbitrary units

			 // Apply calibration
			 gyroscope = FusionCalibrationInertial(gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);
		 	 accelerometer = FusionCalibrationInertial(accelerometer, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset);
		 	 magnetometer = FusionCalibrationMagnetic(magnetometer, softIronMatrix, hardIronOffset);

		 	 // Update gyroscope offset correction algorithm
		 	 gyroscope = FusionOffsetUpdate(&offset, gyroscope);

		 	 // Calculate delta time (in seconds) to account for gyroscope sample clock error
		 	 static clock_t previousTimestamp;
		 	 const float deltaTime = (float) (timestamp - previousTimestamp) / (float) CLOCKS_PER_SEC;
		 	 previousTimestamp = timestamp;

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
		 }
}
