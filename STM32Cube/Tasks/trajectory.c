/*
 * trajectory.c
 *
 *  Created on: May 25, 2024
 *      Author: Jacob Gordon
 */

#include "trajectory.h"
#include <math.h>

#define GRAV_AT_SEA_LVL 9.80665 //m/s^2
#define EARTH_MEAN_RADIUS 6371009 //m
#define AIRBRAKES_MAX_AREA 0.004993538 //m^2
#define ROCKET_BASE_AREA 0.0182412538 //m^2
#define SIM_ALTITUDE 1000 //All drag sims conducted at 1000m above sea level
#define TOL 0.00001

xQueueHandle altQueue;
xQueueHandle structQueue;


/**
 * Does not take into account fins
 * @param extension extension of the airbrakes (0-1)
 * @return rocket's cross-sectional area from airbrake extension
 */
float rocket_area(float extension) {
	return (AIRBRAKES_MAX_AREA * extension) + ROCKET_BASE_AREA;
}

/**
 *  @param force (N)
 *  @param mass (kg)
 *  @return acceleration (m/s^2)
 */
float acceleration(float force, float mass) {
    return force/mass;
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
    float angle = atan(velX/velY);
    float Fd = -interpolate_drag(extension, sqrt(velY*velY+ velX*velX), alt); // force of drag (N)
    float Fg = -gravitational_acceleration(alt) * mass; // force of gravity (N)
    forces.Fy = Fd * cos(angle) + Fg;
    forces.Fx = Fd * sin(angle);
    return forces;
}

/**
    * rk4 method to integrate altitude from velocity, and integrate velocity from acceleration (force/mass)
    * @param h time step
    * @param extension extension of airbrakes, 0-1
    * @param mass of rocket (kg)
    * @param state, including altitude (m) and velocity in X and Y directions (m/s)
    * @return updated altitude and velocity integrals after one rk4 step
*/
RK4State rk4(float h, float mass, float extension, RK4State state) {
    Forces forces;

    forces = get_forces(extension, mass, state.velX, state.velY, state.alt);
    float ka1 = h * state.velY;
    float kvY1 = h * acceleration(forces.Fy, mass);
    float kvX1 = h * acceleration(forces.Fx, mass);

    forces = get_forces(extension, mass, state.velX + kvX1/2, state.velY + kvY1/2, state.alt + ka1/2);
    float ka2 = h * (state.velY + h*kvY1/2);
    float kvY2 = h * acceleration(forces.Fy, mass);
    float kvX2 = h * acceleration(forces.Fx, mass);

    forces = get_forces(extension, mass, state.velX + kvX2/2, state.velY + kvY2/2, state.alt + ka2/2);
    float ka3 = h * (state.velY + h*kvY2/2);
    float kvY3 = h * acceleration(forces.Fy, mass);
    float kvX3 = h * acceleration(forces.Fx, mass);

    forces = get_forces(extension, mass, state.velX + kvX3, state.velY + kvY3, state.alt + ka3);
    float ka4 = h * (state.velY + h*kvY3);
    float kvY4 = h * acceleration(forces.Fy, mass);
    float kvX4 = h * acceleration(forces.Fx, mass);

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

    const float h = 0.05; // interval of change for rk4
    float prevAlt = 0.0; // variable to store previous altitude
        
    RK4State states;
    
    states.alt = altitude;
    states.velY = velY;
    states.velX = velX;

    while (states.alt >= prevAlt) {
        prevAlt = states.alt; //to check if altitude is decreasing to exit the loop
        states = rk4(h, mass, airbrake_ext, states);  // update velocity and altitude

    }

    return states.alt;
}

void trajectory_task(){
    structQueue = xQueueCreate(1, sizeof(Data));
    
    altQueue = xQueueCreate(1, sizeof(uint16_t));
    angleQueue = xQueueCreate(1, sizeof(uint16_t));
    int time;
    float prev_alt;
    for(;;)
    {
        uint16_t alt;
        uint16_t angle;
        
        if(xQueueReceive(altQueue, &alt, 10) == pdTRUE) {
            if(xQueueReceive(angleQueue, &angle, 10) == pdTRUE) {
                float vely = (alt-prev_alt)/(time-prev_time);
                float velx = vely*tan(angle);

                prev_alt = alt;
                prev_time = time;
                get_max_altitude(vely,velx, alt, 0.5, 1); //Put this in another queue?
            }
        }
    }
}