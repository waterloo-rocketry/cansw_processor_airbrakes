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
float velocity_derivative(float force, float mass) {
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
    * rk4 method to integrate altitude from velocity, and integrate velocity from acceleration (force/mass)
    * @param h time step
    * @param force sum of forces acting on rocket at given time (N)
    * @param mass of rocket (kg)
    * @param integrals, altitude (m) and velocity (m/s)
    * @return updated altitude and velocity integrals after one rk4 step
*/
RK4Integrals rk4(float h, float force, float mass, RK4Integrals integrals) {
    float ka1 = h * integrals.vel;
    float kv1 = h * velocity_derivative(force, mass);

    float ka2 = h * (integrals.vel + h*ka1/2);
    float kv2 = h * velocity_derivative(force + h*kv1/2, mass);

    float ka3 = h * (integrals.vel + h*ka2/2);
    float kv3 = h * velocity_derivative(force + h*kv2/2, mass);

    float ka4 = h * (integrals.vel + h*ka3);
    float kv4 = h * velocity_derivative(force + h*kv3, mass);

    RK4Integrals updatedIntegrals;
    updatedIntegrals.alt = (integrals.alt + (ka1 + 2*ka2 + 2*ka3 + ka4)/6);
    updatedIntegrals.vel = (integrals.vel + (kv1 + 2*kv2 + 2*kv3 + kv4)/6);
    
    return updatedIntegrals;
}

/** @return max apogee
* @param velocity vertical velocity (m/s)
* @param altitude (m)
* @param airbrake_ext extension of airbrakes, 0-1
* @param mass (kg)
*/
float get_max_altitude(float velocity, float altitude, float airbrake_ext, float mass) {

    float h = 0.05; // interval of change for rk4
    float prevAlt = 0.0; // variable to store previous altitude
        
    RK4Integrals states;
    
    states.alt = altitude;
    states.vel = velocity;
    
    RK4Integrals updatedIntegrals;
    while (states.alt >= prevAlt) {
        // update forces of drag and gravity from new altitude
        float Fg = -gravitational_acceleration(states.alt) * mass; // force of gravity (N)
        float Fd = -interpolate_drag(airbrake_ext, states.vel, states.alt); // force of drag (N)
        // to check if altitude is decreasing to exit the loop
        prevAlt = states.alt;

        // update velocity and altitude
        states = rk4(h, Fg+Fd, mass, states);

        // System.out.println("pred alt: " + states.alt + "m");
    }

    return states.alt;
}

void trajectory_task(){
    altQueue = xQueueCreate(1, sizeof(uint16_t));
    structQueue = xQueueCreate(1, sizeof(Data));
    float prev_alt;
    for(;;)
    {
        uint16_t alt;
        //Block the thread until we see data in the bus queue or 5 ticks elapse
        if(xQueueReceive(busQueue, &alt, 10) == pdTRUE) //Returns pdTRUE if we got a message, pdFALSE if timed out
        {
            float vel = (prev_alt - alt);
        

            prev_alt = alt;
        }
    }
}