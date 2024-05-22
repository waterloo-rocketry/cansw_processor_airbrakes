/*
 * trajectory.c
 *
 *  Created on: May 7, 2024
 *      Author: joedo
 */

#include "trajectory.h"
#include <math.h>

#define GRAV_AT_SEA_LVL 9.80665 //m/s^2
#define EARTH_MEAN_RADIUS 6371009 //m
#define AIRBRAKES_MAX_AREA 0.004993538 //m^2
#define ROCKET_BASE_AREA 0.0182412538 //m^2
#define SIM_ALTITUDE 1000 //All drag sims conducted at 1000m above sea level
#define TOL = 0.00001


 /**
 * @param altitude (m)
 * @return air density (kg/m^3)
 * Based on US Standard Atmosphere 1976
 * Same method used for PyAnsys simulations, verified to be correct
 */
static float air_density(float altitude) {

	float temperature;
	float pressure;
	float density;

	if (altitude <= 11000) {  // Troposphere
		temperature = 288.15 - 0.00649 * altitude;
		pressure = 101325 * pow((1 - 0.00649 * altitude / 288.15), 5.2561);
	}
	else if (altitude <= 25000) {  // Stratosphere
		temperature = 216.65;
		pressure = 22632.06 * pow(-0.000157 * (altitude - 11000), 5.2561); //may have fucked this up in copying
	}
	else {  // Mesosphere
		temperature = 273.15 - 0.0028 * altitude;
		pressure = 5474.89 * pow((1 - 0.0028 * altitude / 273.15), 5.2561);
	}

	density = pressure / (287.05 * temperature);

	return density;
}

/**
 * Does not take into account fins
 * @param extension extension of the airbrakes (0-1)
 * @return rocket's cross-sectional area from airbrake extension
 */
static float rocket_area(float extension) {
	return (AIRBRAKES_MAX_AREA * extension) + ROCKET_BASE_AREA;
}

/**
 * @param altitude (m)
 * @return acceleration due to gravity (m/s^2)
 */
float gravitational_acceleration(float altitude) {
	return GRAV_AT_SEA_LVL * pow(EARTH_MEAN_RADIUS / ( EARTH_MEAN_RADIUS + altitude), 2);
}

