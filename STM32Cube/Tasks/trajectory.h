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
RK4State RK4State(RK4State initial, float dt, float mass, float extension);
float get_max_altitude(float velocity, float altitude, float airbrake_ext, float mass);
void trajectoryTask(void);

typedef struct  RK4StateStruct{
    float velY;
    float velX;
    float alt;
} RK4State;

typedef struct  DataStruct{
    float ext;
    float vel;
    float alt;
} Data;

typedef struct ForceStruct{
    float Fy;
    float Fx;
} Forces;

typedef struct AltStruct{
    uint16_t alt;
    float time;
} AltTime;

typedef union {
    float array[3];

    struct {
        float roll;
        float pitch;
        float yaw;
    } angle;
} AnglesUnion;


#endif /* TRAJECTORY_H_ */
