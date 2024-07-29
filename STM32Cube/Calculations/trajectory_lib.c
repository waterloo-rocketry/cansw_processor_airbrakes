#include "trajectory_lib.h"

#include <math.h>

#define GRAV_AT_SEA_LVL 9.80665    // m/s^2
#define EARTH_MEAN_RADIUS 6371009  // m
#define TOL 0.00001
#define TIME_STEP 0.05  // s

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
float evaluate_cubic_2_variable(Cubic2VariablePolynomial* poly, float x,
                                float y) {
    return poly->p00 + poly->p10 * x + poly->p01 * y + poly->p20 * x * x +
           poly->p11 * x * y + poly->p02 * y * y + poly->p30 * x * x * x +
           poly->p21 * x * x * y + poly->p12 * x * y * y +
           poly->p03 * y * y * y;
}

float interpolate_drag(float extension, float velocity, float altitude) {
    if (extension < 0) {
        extension = 0;
    } else if (extension > 1) {
        extension = 1;
    }

    if (velocity < 34.0f) {
        return 0.0f;
    }

    float x = (velocity - 273.9f) / 148.7f;
    float y = (altitude - 5000.0f) / 3172.0f;

    Cubic2VariablePolynomial coeffs[11] = {
        {232.2951f, 244.7010f, -75.1435f, 64.3402f, -79.5220f, 11.7309f,
         -0.8306f, -20.4344f, 9.7041f, -0.6148f},
        {235.8993f, 249.2100f, -76.2767f, 65.8251f, -81.2931f, 12.0289f,
         -0.8408f, -21.0236f, 9.9787f, -0.7853f},
        {245.6886f, 260.2967f, -80.2111f, 69.3746f, -85.4361f, 12.5705f,
         -0.6199f, -22.1676f, 10.4297f, -0.6666f},
        {253.9691f, 270.2409f, -83.5032f, 72.3919f, -89.3117f, 13.3189f,
         -0.7371f, -23.2489f, 11.0822f, -0.7471f},
        {263.5127f, 280.7695f, -86.9338f, 75.8929f, -93.6884f, 14.1591f,
         -0.3771f, -24.5324f, 11.7445f, -0.9399f},
        {272.4592f, 290.5670f, -90.1040f, 78.8810f, -97.0343f, 14.5126f,
         -0.1988f, -25.6374f, 12.0348f, -0.7327f},
        {284.8368f, 304.7727f, -94.4923f, 82.4469f, -101.4462f, 15.1080f,
         -0.6357f, -26.5433f, 12.4927f, -0.8323f},
        {296.2638f, 317.3919f, -98.5746f, 86.2809f, -106.2663f, 15.9556f,
         -0.5224f, -28.0106f, 13.2557f, -0.8541f},
        {303.1856f, 325.1022f, -100.9674f, 88.7552f, -109.3743f, 16.5627f,
         -0.4253f, -28.9892f, 13.8034f, -0.9447f},
        {316.4963f, 339.6502f, -104.5570f, 92.4088f, -114.1954f, 16.9114f,
         -0.5681f, -30.4995f, 13.9269f, -1.2933f},
        {340.9146f, 367.1520f, -114.8622f, 101.0439f, -123.1214f, 18.4047f,
         -0.1879f, -31.8071f, 15.4422f, -1.1456f}};

    int index = (int)(extension * 10.0f);
    if (extension * 10.0f == index) {
        return evaluate_cubic_2_variable(&coeffs[index], x, y);
    }
    float first = evaluate_cubic_2_variable(&coeffs[index], x, y);
    float second = evaluate_cubic_2_variable(&coeffs[index + 1], x, y);
    float diff = (extension * 10.0f) - ((float)index);
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
