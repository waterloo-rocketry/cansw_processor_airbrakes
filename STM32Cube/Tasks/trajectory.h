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
extern xQueueHandle structQueue;

float rocket_area(float extension);
float velocity_derivative(float force, float mass);
float gravitational_acceleration(float altitude);
float air_density(float altitude);
float lookup_drag(float velocity, float altitude, float extension);
float interpolate_drag(float velocity, float altitude, float extension);
float interpolate_cd(float extension, float velocity, float altitude);
RK4Integrals RK4Integrate(RK4Integrals initial, float dt, float mass, float extension);
float get_max_altitude(float velocity, float altitude, float airbrake_ext, float mass);
void trajectoryTask(void);

typedef struct  RK4IntegralsStruct{
    float vel;
    float alt;
} RK4Integrals;

typedef struct  DataStruct{
    float ext;
    float vel;
    float alt;
} Data;





#endif /* TRAJECTORY_H_ */
