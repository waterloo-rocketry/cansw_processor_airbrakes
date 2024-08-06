/*
 * trajectory.c
 *
 *  Created on: May 25, 2024
 *      Author: Jacob Gordon
 */

#include "trajectory.h"

#include <limits.h>
#include <math.h>

#include "Fusion.h"
#include "controller.h"
#include "flight_phase.h"
#include "log.h"
#include "otits.h"
#include "trajectory_lib.h"
#include "printf.h"

xQueueHandle altQueue;
xQueueHandle angleQueue;

/*
 * Test that the apogee queue is not frozen, and values are within reason
 */
Otits_Result_t test_apogeeQueue() {
    Otits_Result_t res;
    float apogee1;
    float apogee2;

    if (xQueuePeek(apogeeQueue, &apogee1, 120) != pdTRUE) {
        res.outcome = TEST_OUTCOME_TIMEOUT;
        res.info = "apogee q peek 1 timeout";
        return res;
    }

    vTaskDelay(120);

    if (xQueuePeek(apogeeQueue, &apogee2, 120) != pdTRUE) {
        res.outcome = TEST_OUTCOME_TIMEOUT;
        res.info = "apogee q peek 2 timeout";
        return res;
    }

    if (apogee1 == apogee2) {
        res.outcome = TEST_OUTCOME_FAILED;
        res.info = "apogee q didnt update?";
        return res;
    }

    if (apogee1 <= 0 || apogee1 > 20000 || apogee2 <= 0 || apogee2 > 20000) {
        res.outcome = TEST_OUTCOME_DATA_ERR;
        res.info = "apogee out of range";
        return res;
    }

    res.outcome = TEST_OUTCOME_PASSED;
    res.info = "";
    return res;
}

void trajectory_task(void* argument) {
    float prev_time = -1;
    int32_t prev_alt = INT_MAX;
    uint32_t recovery_timeout = 0;
    float filter_data[3] = {0.0f}; //storage for lowpass filter data

    for (;;) {
        AltTime altTime;
        FusionEuler angles;
        if (xQueueReceive(altQueue, &altTime, 10) == pdTRUE) {
            if (xQueuePeek(angleQueue, &angles, 100) == pdTRUE) {
                if (prev_alt != INT_MAX) {
                    float vely_raw = (altTime.alt - prev_alt) * 1000.0 /
                                 (altTime.time - prev_time);

                    //float vely = second_order_lowpass_filter(vely_raw, filter_data, VELOCITY_FILTER_ALPHA);
                    float vely = moving_average_filter(vely_raw, filter_data);
                    //printf_(">vely:%d\n", (int) vely);

                    // if we see velocity drop, we know apogee is incoming
                    // regardless to prevent the bit being prematurely set at
                    // startup due to weird numerical stuff, we only check this
                    // condition while in coast phase
                    // Also, to deal with potential noise, the system must see consecutive velocity readings below the threshold
                    if (extensionAllowed())
					{
                    	if (vely < RECOVERY_MIN_VELOCITY)
						{
                    		recovery_timeout++;
                    		if(recovery_timeout >= RECOVERY_TIMEOUT_READINGS)
                    		{
                    			xEventGroupSetBits(flightPhaseEventsHandle, RECOVERY_DEPLOYMENT_BIT);
                    		}
						}
                    	else
                    	{
                    		recovery_timeout = 0;
                    	}
					}

                    float velx =
                        vely /
                        tan(angles.angle.pitch / 180.0 *
                            M_PI);  // state est measures pitch from horizontal
                    float apogee = getMaxAltitude_m(vely, velx, altTime.alt);
                    logInfo("traj", "%fm", apogee);
                    //printf_(">traj:%fm\n", apogee);
                    xQueueOverwrite(apogeeQueue, &apogee);
                }
                prev_alt = altTime.alt;
                prev_time = altTime.time;
            }
        }
    }
}

bool trajectory_init() {
    altQueue = xQueueCreate(1, sizeof(AltTime));
    angleQueue = xQueueCreate(1, sizeof(FusionEuler));

    if (altQueue == NULL || angleQueue == NULL) return false;
    if (!otitsRegister(test_apogeeQueue, TEST_SOURCE_TRAJ, "apogeeQ"))
        return false;

    return true;
}
