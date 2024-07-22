/*
 * trajectory.c
 *
 *  Created on: May 25, 2024
 *      Author: Jacob Gordon
 */

#include "trajectory.h"

#include "Fusion.h"
#include "controller.h"
#include "otits.h"
#include "trajectory_lib.h"

xQueueHandle altQueue;
xQueueHandle angleQueue;
xQueueHandle extQueue;

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
    uint16_t prev_alt = 0xFFFF;

    for (;;) {
        AltTime altTime;
        FusionEuler angles;
        float ext;
        if (xQueueReceive(altQueue, &altTime, 10) == pdTRUE) {
            if (xQueuePeek(extQueue, &ext, 10) == pdTRUE) {
                if (xQueuePeek(angleQueue, &angles, 100) == pdTRUE) {
                    if (prev_alt != 0xFFFF) {
                        float vely = (altTime.alt - prev_alt) * 1000.0 /
                                     (altTime.time - prev_time);
                        float velx = vely * tan(angles.angle.pitch);
                        float apogee = get_max_altitude(
                            vely, velx, altTime.alt, ext, ROCKET_BURNOUT_MASS);
                        xQueueOverwrite(apogeeQueue, &apogee);
                    }
                    prev_alt = altTime.alt;
                    prev_time = altTime.time;
                }
            }
        }
        vTaskDelay(20);  // TODO: for testing so this blocks
    }
}
void trajectory_init() {
    altQueue = xQueueCreate(1, sizeof(AltTime));
    angleQueue = xQueueCreate(1, sizeof(FusionEuler));
    extQueue = xQueueCreate(1, sizeof(float));
    otitsRegister(test_apogeeQueue, TEST_SOURCE_TRAJ, "apogeeQ");
}
