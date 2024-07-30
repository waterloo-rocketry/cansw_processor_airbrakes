/*
 * controller.c
 *
 *  Created on: Apr 14, 2024
 *      Author: joedo
 */
#include "controller.h"

#include <math.h>

#include "can_handler.h"
#include "controller_lib.h"
#include "flight_phase.h"
#include "log.h"
#include "millis.h"
#include "printf.h"
#include "stm32h7xx_hal.h"
#include "trajectory.h"

#define MIN_EXTENSION_CMD 24
#define MAX_EXTENSION_CMD 100

QueueHandle_t apogeeQueue;
QueueHandle_t targetQueue;

bool controllerInit() {
	apogeeQueue = xQueueCreate(1, sizeof(float));
	targetQueue = xQueueCreate(1, sizeof(uint16_t));

	if (apogeeQueue == NULL || targetQueue == NULL) return false;

	return true;
}

void controlTask(void* argument) {
    float apogeeEstimate;
    ControllerState controller_state;
    controllerStateInit(&controller_state);
    float target_altitude = DEFAULT_TARGET_ALTITUDE_M;

    /* Infinite loop */
    for (;;) {
        uint16_t updated_target;
        if (xQueueReceive(targetQueue, &updated_target, 0) == pdPASS) {
            target_altitude = (float)updated_target;
            can_msg_t msg;
            build_state_est_calibration_msg((uint32_t)millis_(), 1,
                                            updated_target, &msg);
            xQueueSend(busQueue, &msg, 100);
            logInfo("controller", "Controller target updated to %d m",
                    updated_target);
        }

        if (xQueueReceive(apogeeQueue, &apogeeEstimate, 100) == pdTRUE &&
            extensionAllowed()) {
            // PID controller update
            float output = updateController(&controller_state,
                                            target_altitude - apogeeEstimate);

            float extension =
                0.5 - output;  // invert the controller output - if we are
                               // undershooting retract, and if we are
                               // overshooting, extend

			if(extension > CONTROLLER_MAX_EXTENSION) extension = CONTROLLER_MAX_EXTENSION;
			if(extension < CONTROLLER_MIN_EXTENSION) extension = CONTROLLER_MIN_EXTENSION;

			uint8_t cmd_extension = extension * (MAX_EXTENSION_CMD - MIN_EXTENSION_CMD) + MIN_EXTENSION_CMD;

			can_msg_t msg;
			build_actuator_cmd_analog( (uint32_t) millis_(), ACTUATOR_AIRBRAKES_SERVO, cmd_extension, &msg);
			xQueueSend(busQueue, &msg, 5); //If we are in the coast phase, command the airbrakes servo to the target extension value
			logInfo("controller", "ext CMD: %d", cmd_extension);
			//printf_("extension: %f\n", extension);

		}
		else if (recoveryPhase())
		{
			can_msg_t msg;
			build_actuator_cmd_analog( (uint32_t) millis_(), ACTUATOR_AIRBRAKES_SERVO, MIN_EXTENSION_CMD, &msg); //close the airbrakes once we have reached the recovery phase
			xQueueSend(busQueue, &msg, 5);
			logInfo("controller", "ext CMD: %d", MIN_EXTENSION_CMD);
		}
	}
}
