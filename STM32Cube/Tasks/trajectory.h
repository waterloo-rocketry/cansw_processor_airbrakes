/*
 * trajectory.h
 *
 *  Created on: Apr 13, 2024
 *      Author: joedo
 */

#ifndef TRAJECTORY_H_
#define TRAJECTORY_H_

#include "FreeRTOS.h"
#include "queue.h"

extern xQueueHandle altQueue;
extern xQueueHandle angleQueue;
extern xQueueHandle extQueue;

void trajectory_task(void* argument);
void trajectory_init(void);

// Struct for recieving altitude data, could replace with import
typedef struct AltStruct {
    int32_t alt;  // altitude in m
    float time;   // time in ms
} AltTime;

#endif /* TRAJECTORY_H_ */
