/*
 * trajectory.c
 *
 *  Created on: May 25, 2024
 *      Author: Jacob Gordon
 */

#include "trajectory.h"
#include "controller.h"
#include <math.h>
#include "Fusion.h"

#define GRAV_AT_SEA_LVL 9.80665 //m/s^2
#define EARTH_MEAN_RADIUS 6371009 //m
#define AIRBRAKES_MAX_AREA 0.004993538 //m^2
#define ROCKET_BASE_AREA 0.0182412538 //m^2
#define SIM_ALTITUDE 1000 //All drag sims conducted at 1000m above sea level
#define TOL 0.00001
#define ROCKET_BURNOUT_MASS 39.564 //kg
#define TIME_STEP 0.05 //s

xQueueHandle altQueue;
xQueueHandle angleQueue;
xQueueHandle extQueue;

float rocket_area(float extension);
float velocity_derivative(float force, float mass);
float gravitational_acceleration(float altitude);
float air_density(float altitude);
float lookup_drag(float fixed_extension, float velocity);
float interpolate_drag(float velocity, float altitude, float extension);
float interpolate_cd(float extension, float velocity, float altitude);
RK4State rk4(float h, float mass, float extension, RK4State state);
float get_max_altitude(float velY, float velX, float altitude, float airbrake_ext, float mass);

/**
 * Does not take into account fins
 * @param extension of the airbrakes (0-1)
 * @return rocket's cross-sectional area from airbrake extension
 */
float rocket_area(float extension) {
	return (AIRBRAKES_MAX_AREA * extension) + ROCKET_BASE_AREA;
}



/**
 * @param altitude (m)
 * @return acceleration due to gravity (m/s^2)
 */
float gravitational_acceleration(float altitude) {
	return GRAV_AT_SEA_LVL * pow(EARTH_MEAN_RADIUS / ( EARTH_MEAN_RADIUS + altitude), 2);
}

 /**
 * @param altitude (m)
 * @return air density (kg/m^3)
 * Based on US Standard Atmosphere 1976
 * Same method used for PyAnsys simulations, verified to be correct
 */
float air_density(float altitude) {

	float temperature;
	float pressure;
	float density;

	if (altitude <= 11000) {  // Troposphere
		temperature = 288.15 - 0.00649 * altitude;
		pressure = 101325 * pow((1 - 0.00649 * altitude / 288.15), 5.2561);
	}
	else if (altitude <= 25000) {  // Stratosphere
		temperature = 216.65;
		//pressure = 22632.06 * exp(-0.000157 * (altitude - 11000));
        pressure = 22632.06 * pow(-0.000157 * (altitude - 11000), 5.2561);
	}
	else {  // Mesosphere
		temperature = 273.15 - 0.0028 * altitude;
		pressure = 5474.89 * pow((1 - 0.0028 * altitude / 273.15), 5.2561);
	}

	density = pressure / (287.05 * temperature);

	return density;
}

/**
* @param velocity is used to lookup drag value
* @param fixed_extension MUST BE EITHER 0, 0.5, or 1. Used to determine which function to use and to adjust for rocket area
* @return drag force value based on curves produced by Ansys at fixed extensions
*/
float lookup_drag(float fixed_extension, float velocity) {
    float drag;
    if (fabs(fixed_extension) < TOL) { //0%
        drag = (0.0035 * velocity * velocity) + (0.1317 * velocity) - 5.0119;
    } 
    else if (fabs(fixed_extension - 0.5) < TOL) { //50%
        drag = (0.0045 * velocity * velocity) + (0.1031 * velocity) - 3.8231;
    } 
    else { // 100%
        drag = (0.006 * velocity * velocity) + (0.1038 * velocity) - 4.2522;
    }

    return drag / rocket_area(fixed_extension) / air_density(SIM_ALTITUDE); //adjust simulation drag for altitude and extension amount
}

/**@return drag force acting on rocket
 * @param extension of air brakes, used for adjusting rocket area and iterpolating Ansys sims (0-1)
 * @param velocity used to lookup drag force from Ansys sims
 * @param altitude used to adjust fluid density since Ansys sims were calculated at 1000m
 * */
float interpolate_drag(float extension, float velocity, float altitude) {
    float drag;
    float ext_diff = 0.5;
    float drag_low; //drag at low ext
    float drag_high; //drag at high ext
    float ext_low; //low ext

    if(extension < 0){
        extension = 0;
    } else if(extension > 1){
        extension = 1;
    }

    if (extension > 0.5) {
        ext_low = 0.5;
        drag_low = lookup_drag(0.5, velocity);
        drag_high = lookup_drag(1, velocity);
    }
    else { // extension < 0.5
        ext_low = 0;
        drag_low = lookup_drag(0, velocity);
        drag_high = lookup_drag(0.5, velocity);
    }

    drag = drag_low + (drag_high-drag_low) / (ext_diff) * (extension - ext_low);
    drag = drag * air_density(altitude) * rocket_area(extension); //correct the drag using the actual airbrake extension area and air density
    return drag;
}

/**
* @return Cd value corresponding to interpolated drag force
*/
float interpolate_cd(float extension, float velocity, float altitude){
    float drag_force = interpolate_drag(extension, velocity, altitude);
    return 2 * drag_force / (air_density(altitude) * rocket_area(extension) * (velocity * velocity)); //Cd
}

/**
    * @param airbrake_ext extension of airbrakes, 0-1
    * @param mass of rocket (kg)
    * @param velX, velocity in X direction (m/s)
    * @param velY, velocity in Y direction (m/s)
    * @param alt, altitude (m)
    * @return forces acting on rocket in the X and Y directions (N)
*/
Forces get_forces(float extension, float mass, float velX, float velY, float alt){
    Forces forces;
    float velT = sqrt(velY*velY+ velX*velX);
    float Fd = -interpolate_drag(extension, velT, alt); // force of drag (N)
    float Fg = -gravitational_acceleration(alt) * mass; // force of gravity (N)
    forces.Fy = Fd * velY/velT + Fg;
    forces.Fx = Fd * velX/velT;
    return forces;
}

/**
    * rk4 method to integrate altitude from velocity, and integrate velocity from acceleration (force/mass)
    * @param h time step
    * @param extension of airbrakes, 0-1
    * @param mass of rocket (kg)
    * @param state, including altitude (m) and velocity in X and Y directions (m/s)
    * @return updated altitude and velocity integrals after one rk4 step
*/
RK4State rk4(float h, float mass, float extension, RK4State state) {
    Forces forces;
    //Force / mass = acceleration
    forces = get_forces(extension, mass, state.velX, state.velY, state.alt);
    float ka1 = h * state.velY;
    float kvY1 = h * forces.Fy / mass;
    float kvX1 = h * forces.Fx / mass;

    forces = get_forces(extension, mass, state.velX + kvX1/2, state.velY + kvY1/2, state.alt + ka1/2);
    float ka2 = h * (state.velY + h*kvY1/2);
    float kvY2 = h * forces.Fy / mass;
    float kvX2 = h * forces.Fx / mass;

    forces = get_forces(extension, mass, state.velX + kvX2/2, state.velY + kvY2/2, state.alt + ka2/2);
    float ka3 = h * (state.velY + h*kvY2/2);
    float kvY3 = h * forces.Fy / mass;
    float kvX3 = h * forces.Fx / mass;

    forces = get_forces(extension, mass, state.velX + kvX3, state.velY + kvY3, state.alt + ka3);
    float ka4 = h * (state.velY + h*kvY3);
    float kvY4 = h * forces.Fy / mass;
    float kvX4 = h * forces.Fx / mass;

    RK4State updatedState;
    updatedState.alt = (state.alt + (ka1 + 2*ka2 + 2*ka3 + ka4)/6);
    updatedState.velY = (state.velY + (kvY1 + 2*kvY2 + 2*kvY3 + kvY4)/6);
    updatedState.velX = (state.velX + (kvX1 + 2*kvX2 + 2*kvX3 + kvX4)/6);

    return updatedState;
}

/** @return max apogee
* @param velocity vertical velocity (m/s)
* @param altitude (m)
* @param airbrake_ext extension of airbrakes, 0-1
* @param mass (kg)
*/
float get_max_altitude(float velY, float velX, float altitude, float airbrake_ext, float mass) {

    float prevAlt = 0.0; // variable to store previous altitude
        
    RK4State states;
    
    states.alt = altitude;
    states.velY = velY;
    states.velX = velX;

    while (states.alt >= prevAlt) {
        prevAlt = states.alt; //to check if altitude is decreasing to exit the loop
        states = rk4(TIME_STEP, mass, airbrake_ext, states);  // update velocity and altitude

    }

    return prevAlt;
}

void trajectory_task(void * argument){    
    float prev_time = -1;
    uint16_t prev_alt = 0xFFFF;
    
    for(;;)
    {
        AltTime altTime;
        FusionEuler angles;
        float ext;
        if(xQueueReceive(altQueue, &altTime, 10) == pdTRUE) {
            if(xQueuePeek(extQueue, &ext, 10)== pdTRUE) {
                if(xQueuePeek(angleQueue, &angles, 100) == pdTRUE) {
                    if(prev_alt != 0xFFFF) {
                        float vely = (altTime.alt-prev_alt)*1000.0/(altTime.time-prev_time);
                        float velx = vely*tan(angles.angle.pitch);
                        float apogee = get_max_altitude(vely,velx, altTime.alt, ext, ROCKET_BURNOUT_MASS);
                        printf_("cur alt: %f, pred apogee: %f", altTime.alt, apogee);
                        xQueueOverwrite(apogeeQueue, &apogee);
                    }
                    prev_alt = altTime.alt;
                    prev_time = altTime.time;
                }
            }

        }
    }
}
void trajectory_init(){
    altQueue = xQueueCreate(1, sizeof(AltTime));
    angleQueue = xQueueCreate(1, sizeof(FusionEuler));
    extQueue = xQueueCreate(1, sizeof(float));
}
