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

void trajectory_task(void * argument);
void trajectory_init(void);

//Struct with the data iterated in RK4 method
typedef struct  RK4StateStruct{
    float velY; //m/s
    float velX; //m/s
    float alt; //m
} RK4State;

//Struct with the forces acting on the rocket
typedef struct ForceStruct{
    float Fy; //N
    float Fx; //N
} Forces;

//Struct for recieving altitude data, could replace with import
typedef struct AltStruct{
    int32_t alt; //altitude in m
    float time; //time in ms
} AltTime;


#endif /* TRAJECTORY_H_ */
