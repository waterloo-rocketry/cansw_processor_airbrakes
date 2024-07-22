#include "trajectory_lib.h"

#include <math.h>

#define GRAV_AT_SEA_LVL 9.80665         // m/s^2
#define EARTH_MEAN_RADIUS 6371009       // m
#define AIRBRAKES_MAX_AREA 0.004993538  // m^2
#define ROCKET_BASE_AREA 0.0182412538   // m^2
#define SIM_ALTITUDE 1000  // All drag sims conducted at 1000m above sea level
#define TOL 0.00001
#define TIME_STEP 0.05              // s

// Struct with the data iterated in RK4 method
typedef struct RK4StateStruct {
    float velY;  // m/s
    float velX;  // m/s
    float alt;   // m
} RK4State;

// Struct with the forces acting on the rocket
typedef struct ForceStruct {
    float Fy;  // N
    float Fx;  // N
} Forces;

/**
 * A degree-3 polynomial in 2 variables.
 */
typedef struct {
    float p00;
    float p10;
    float p01;
    float p20;
    float p11;
    float p02;
    float p30;
    float p21;
    float p12;
    float p03;
} Cubic2VariablePolynomial;

/**
 * Evaluates a cubic 2 variable polynomial at the given coordinates.
 */
float evaluate_cubic_2_variable(Cubic2VariablePolynomial* poly, float x, float y) {
    return poly->p00 + poly->p10 * x + poly->p01 * y + poly->p20 * x * x + poly->p11 * x * y + poly->p02 * y * y + poly->p30 * x * x * x + poly->p21 * x * x * y + poly->p12 * x * y * y + poly->p03 * y * y * y;
}

float interpolate_drag(float extension, float velocity, float altitude) {
    if (extension < 0) {
        extension = 0;
    } else if (extension > 1) {
        extension = 1;
    }

    float x = (velocity - 273.9) / 148.7;
    float y = (altitude - 5000) / 3172;

    Cubic2VariablePolynomial coeffs[11] = {
        { 232.2951, 244.7010, -75.1435, 64.3402, -79.5220, 11.7309, -0.8306, -20.4344, 9.7041, -0.6148 },
        { 235.8993, 249.2100, -76.2767, 65.8251, -81.2931, 12.0289, -0.8408, -21.0236, 9.9787, -0.7853 },
        { 245.6886, 260.2967, -80.2111, 69.3746, -85.4361, 12.5705, -0.6199, -22.1676, 10.4297, -0.6666 },
        { 253.9691, 270.2409, -83.5032, 72.3919, -89.3117, 13.3189, -0.7371, -23.2489, 11.0822, -0.7471 },
        { 263.5127, 280.7695, -86.9338, 75.8929, -93.6884, 14.1591, -0.3771, -24.5324, 11.7445, -0.9399 },
        { 272.4592, 290.5670, -90.1040, 78.8810, -97.0343, 14.5126, -0.1988, -25.6374, 12.0348, -0.7327 },
        { 284.8368, 304.7727, -94.4923, 82.4469, -101.4462, 15.1080, -0.6357, -26.5433, 12.4927, -0.8323 },
        { 296.2638, 317.3919, -98.5746, 86.2809, -106.2663, 15.9556, -0.5224, -28.0106, 13.2557, -0.8541 },
        { 303.1856, 325.1022, -100.9674, 88.7552, -109.3743, 16.5627, -0.4253, -28.9892, 13.8034, -0.9447 },
        { 316.4963, 339.6502, -104.5570, 92.4088, -114.1954, 16.9114, -0.5681, -30.4995, 13.9269, -1.2933 },
        { 340.9146, 367.1520, -114.8622, 101.0439, -123.1214, 18.4047, -0.1879, -31.8071, 15.4422, -1.1456 }
    };

    int index = (int) (extension * 10.0f);
    if (extension * 10.0f == index) {
        return evaluate_cubic_2_variable(&coeffs[index], x, y);
    }
    float first = evaluate_cubic_2_variable(&coeffs[index], x, y);
    float second = evaluate_cubic_2_variable(&coeffs[index + 1], x, y);
    float diff = extension - ((float) ((int) extension));
    return diff * second + (1.0f - diff) * first;
}

/**
 * @param altitude (m)
 * @return acceleration due to gravity (m/s^2)
 */
static float gravitational_acceleration(float altitude) {
    return GRAV_AT_SEA_LVL *
           pow(EARTH_MEAN_RADIUS / (EARTH_MEAN_RADIUS + altitude), 2);
}

/**
 * @param airbrake_ext extension of airbrakes, 0-1
 * @param mass of rocket (kg)
 * @param velX, velocity in X direction (m/s)
 * @param velY, velocity in Y direction (m/s)
 * @param alt, altitude (m)
 * @return forces acting on rocket in the X and Y directions (N)
 */
static Forces get_forces(float extension, float mass, float velX, float velY,
                  float alt) {
    Forces forces;
    float velT = sqrt(velY * velY + velX * velX);
    float Fd = -interpolate_drag(extension, velT, alt);  // force of drag (N)
    float Fg = -gravitational_acceleration(alt) * mass;  // force of gravity (N)
    forces.Fy = Fd * velY / velT + Fg;
    forces.Fx = Fd * velX / velT;
    return forces;
}

/**
 * rk4 method to integrate altitude from velocity, and integrate velocity from
 * acceleration (force/mass)
 * @param h time step
 * @param extension of airbrakes, 0-1
 * @param mass of rocket (kg)
 * @param state, including altitude (m) and velocity in X and Y directions (m/s)
 * @return updated altitude and velocity integrals after one rk4 step
 */
static RK4State rk4(float h, float mass, float extension, RK4State state) {
    Forces forces;
    // Force / mass = acceleration
    forces = get_forces(extension, mass, state.velX, state.velY, state.alt);
    float ka1 = h * state.velY;
    float kvY1 = h * forces.Fy / mass;
    float kvX1 = h * forces.Fx / mass;

    forces = get_forces(extension, mass, state.velX + kvX1 / 2,
                        state.velY + kvY1 / 2, state.alt + ka1 / 2);
    float ka2 = h * (state.velY + h * kvY1 / 2);
    float kvY2 = h * forces.Fy / mass;
    float kvX2 = h * forces.Fx / mass;

    forces = get_forces(extension, mass, state.velX + kvX2 / 2,
                        state.velY + kvY2 / 2, state.alt + ka2 / 2);
    float ka3 = h * (state.velY + h * kvY2 / 2);
    float kvY3 = h * forces.Fy / mass;
    float kvX3 = h * forces.Fx / mass;

    forces = get_forces(extension, mass, state.velX + kvX3, state.velY + kvY3,
                        state.alt + ka3);
    float ka4 = h * (state.velY + h * kvY3);
    float kvY4 = h * forces.Fy / mass;
    float kvX4 = h * forces.Fx / mass;

    RK4State updatedState;
    updatedState.alt = (state.alt + (ka1 + 2 * ka2 + 2 * ka3 + ka4) / 6);
    updatedState.velY = (state.velY + (kvY1 + 2 * kvY2 + 2 * kvY3 + kvY4) / 6);
    updatedState.velX = (state.velX + (kvX1 + 2 * kvX2 + 2 * kvX3 + kvX4) / 6);

    return updatedState;
}

float get_max_altitude(float velY, float velX, float altitude,
                       float airbrake_ext, float mass) {
    float prevAlt = 0.0;  // variable to store previous altitude

    RK4State states;

    states.alt = altitude;
    states.velY = velY;
    states.velX = velX;

    while (states.alt >= prevAlt) {
        prevAlt =
            states.alt;  // to check if altitude is decreasing to exit the loop
        states = rk4(TIME_STEP, mass, airbrake_ext,
                     states);  // update velocity and altitude
    }

    return prevAlt;
}