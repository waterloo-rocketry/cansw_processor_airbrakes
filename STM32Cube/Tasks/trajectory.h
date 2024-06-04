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
extern xQueueHandle apogeeQueue;

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

//Struct with the data iterated in RK4 method
typedef struct  RK4StateStruct{
    float velY;
    float velX;
    float alt;
} RK4State;

//Struct with the forces acting on the rocket
typedef struct ForceStruct{
    float Fy;
    float Fx;
} Forces;

//Struct for recieving altitude data, could replace with import
typedef struct AltStruct{
    uint16_t alt;
    float time;
} AltTime;

// Remove this and just import it from Fusion
typedef union {
    float array[3];

    struct {
        float roll;
        float pitch;
        float yaw;
    } angle;
} AnglesUnion;


#endif /* TRAJECTORY_H_ */
