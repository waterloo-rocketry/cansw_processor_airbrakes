#include "trajectory_lib.h"

#include <math.h>

#define GRAV_AT_SEA_LVL 9.80665    // m/s^2
#define EARTH_MEAN_RADIUS 6371009  // m
#define TOL 0.00001
#define TIME_STEP 0.05  // s
#define ROCKET_BURNOUT_MASS_KG 39.609
#define LAUNCH_PAD_ELEVATION 295.0

/**
 * Data iterated in RK4 method.
 */
typedef struct {
    /**
     * Vertical velocity
     */
    float vy_m_s;
    /**
     * Horizontal velocity
     */
    float vx_m_s;
    /**
     * Altitude
     */
    float y_m;
} RK4State;

/**
 * The acceleration acting on the rocket.
 */
typedef struct {
    float ay_m_s2;
    float ax_m_s2;
} Accelerations;

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

// Cubic drag force polynomial coeffs for extensions 0-100% in 10% intervals
static const Cubic2VariablePolynomial DRAG_POLYNOMIAL_COEFFS[11] = {
    {232.2951f, 244.7010f, -75.1435f, 64.3402f, -79.5220f, 11.7309f, -0.8306f,
     -20.4344f, 9.7041f, -0.6148f},  // 0% ext
    {235.8993f, 249.2100f, -76.2767f, 65.8251f, -81.2931f, 12.0289f, -0.8408f,
     -21.0236f, 9.9787f, -0.7853f},  // 10% ext
    {245.6886f, 260.2967f, -80.2111f, 69.3746f, -85.4361f, 12.5705f, -0.6199f,
     -22.1676f, 10.4297f, -0.6666f},  // etc
    {253.9691f, 270.2409f, -83.5032f, 72.3919f, -89.3117f, 13.3189f, -0.7371f,
     -23.2489f, 11.0822f, -0.7471f},
    {263.5127f, 280.7695f, -86.9338f, 75.8929f, -93.6884f, 14.1591f, -0.3771f,
     -24.5324f, 11.7445f, -0.9399f},
    {272.4592f, 290.5670f, -90.1040f, 78.8810f, -97.0343f, 14.5126f, -0.1988f,
     -25.6374f, 12.0348f, -0.7327f},
    {284.8368f, 304.7727f, -94.4923f, 82.4469f, -101.4462f, 15.1080f, -0.6357f,
     -26.5433f, 12.4927f, -0.8323f},
    {296.2638f, 317.3919f, -98.5746f, 86.2809f, -106.2663f, 15.9556f, -0.5224f,
     -28.0106f, 13.2557f, -0.8541f},
    {303.1856f, 325.1022f, -100.9674f, 88.7552f, -109.3743f, 16.5627f, -0.4253f,
     -28.9892f, 13.8034f, -0.9447f},
    {316.4963f, 339.6502f, -104.5570f, 92.4088f, -114.1954f, 16.9114f, -0.5681f,
     -30.4995f, 13.9269f, -1.2933f},
    {340.9146f, 367.1520f, -114.8622f, 101.0439f, -123.1214f, 18.4047f,
     -0.1879f, -31.8071f, 15.4422f, -1.1456f}  // 100% ext
};

/**
 * Evaluates a cubic 2 variable polynomial at the given coordinates.
 */
static float evaluateCubic2Variable(const Cubic2VariablePolynomial* poly,
                                    float x, float y) {
    return poly->p00 + poly->p10 * x + poly->p01 * y + poly->p20 * x * x +
           poly->p11 * x * y + poly->p02 * y * y + poly->p30 * x * x * x +
           poly->p21 * x * x * y + poly->p12 * x * y * y +
           poly->p03 * y * y * y;
}

float dragAccel_m_s2(float extension, float speed_m_s, float altitude_m) {
    // Writing the conditions with inequalities like this allows us to express
    // that we want extension to be in the range [0, 1]. Also handles nan better
    // (hope that doesn't happen)
    if (!(0.0f <= extension)) {
        extension = 0.0f;
    }
    if (!(extension <= 1.0f)) {
        extension = 1.0f;
    }

    // Compute the polynomial for (speed, altitude) -> drag acceleration given
    // the extension.
    // Hopefully, for a fixed extension, the calculation of the polynomial will
    // be optimized out at compile time.
    Cubic2VariablePolynomial poly;

    // Apply a macro to fields of `Cubic2VariablePolynomial`
#define APPLY_TO_FIELDS(M) \
    M(p00) M(p10) M(p01) M(p20) M(p11) M(p02) M(p30) M(p21) M(p12) M(p03)
    float extensionX10 = extension * 10.0f;
    int index = (int)extensionX10;
    if (extensionX10 == index) {
        poly = DRAG_POLYNOMIAL_COEFFS[index];
    } else {
        // Linear interpolation
        const Cubic2VariablePolynomial* p1 = &DRAG_POLYNOMIAL_COEFFS[index];
        const Cubic2VariablePolynomial* p2 = &DRAG_POLYNOMIAL_COEFFS[index + 1];
        float diff = extensionX10 - (float)index;
#define INTERPOLATE_FIELD(field) \
    poly.field = diff * p2->field + (1.0f - diff) * p1->field;
        APPLY_TO_FIELDS(INTERPOLATE_FIELD)
#undef INTERPOLATE_FIELD
    }
    // Divide drag force by mass to get acceleration
#define SCALE_FIELD(field) poly.field /= ROCKET_BURNOUT_MASS_KG;
    APPLY_TO_FIELDS(SCALE_FIELD)
#undef SCALE_FIELD
#undef APPLY_TO_FIELDS

    if (speed_m_s < 34.0f) {
        return 0.0f;
    }

    float x = (speed_m_s - 273.9f) / 148.7f;
    float y = (altitude_m + LAUNCH_PAD_ELEVATION - 5000.0f) / 3172.0f;
    float drag = evaluateCubic2Variable(&poly, x, y);
    if (drag >= 0.0F) {
        return drag;
    } else {
        return 0.0F;
    }
}

/**
 * Computes the acceleration due to gravity.
 */
static float gravitationalAccel_m_s2(float altitude_m) {
    return GRAV_AT_SEA_LVL *
           pow(EARTH_MEAN_RADIUS / (EARTH_MEAN_RADIUS + altitude_m), 2);
}

/**
 * @param airbrake_ext extension of airbrakes, 0-1
 * @param vx_m_s, velocity in X direction (m/s)
 * @param vy_m_s, velocity in Y direction (m/s)
 * @param y_m, altitude (m)
 * @return accels acting on rocket in the X and Y directions (N)
 */
static Accelerations getAccels(float extension, float vx_m_s, float vy_m_s,
                               float y_m) {
    // Total spee
    float speed_m_s = sqrt(vy_m_s * vy_m_s + vx_m_s * vx_m_s);
    // Acceleration due to drag
    float ad_m_s2 = -dragAccel_m_s2(extension, speed_m_s, y_m);
    // Acceleration due to gravity
    float ag_m_s2 = -gravitationalAccel_m_s2(y_m);
    Accelerations accel = {.ay_m_s2 = ad_m_s2 * vy_m_s / speed_m_s + ag_m_s2,
                           .ax_m_s2 = ad_m_s2 * vx_m_s / speed_m_s};
    return accel;
}

/**
 * rk4 method to integrate altitude from velocity, and integrate velocity from
 * acceleration (force/mass)
 * @param h_s time step
 * @param extension of airbrakes, 0-1
 * @param state, including altitude (m) and velocity in X and Y directions (m/s)
 * @return updated altitude and velocity integrals after one rk4 step
 */
static RK4State rk4(float h_s, float extension, RK4State state) {
    Accelerations accels =
        getAccels(extension, state.vx_m_s, state.vy_m_s, state.y_m);
    float ka1 = h_s * state.vy_m_s;
    float kvY1 = h_s * accels.ay_m_s2;
    float kvX1 = h_s * accels.ax_m_s2;

    accels = getAccels(extension, state.vx_m_s + kvX1 / 2,
                       state.vy_m_s + kvY1 / 2, state.y_m + ka1 / 2);
    float ka2 = h_s * (state.vy_m_s + h_s * kvY1 / 2);
    float kvY2 = h_s * accels.ay_m_s2;
    float kvX2 = h_s * accels.ax_m_s2;

    accels = getAccels(extension, state.vx_m_s + kvX2 / 2,
                       state.vy_m_s + kvY2 / 2, state.y_m + ka2 / 2);
    float ka3 = h_s * (state.vy_m_s + h_s * kvY2 / 2);
    float kvY3 = h_s * accels.ay_m_s2;
    float kvX3 = h_s * accels.ax_m_s2;

    accels = getAccels(extension, state.vx_m_s + kvX3, state.vy_m_s + kvY3,
                       state.y_m + ka3);
    float ka4 = h_s * (state.vy_m_s + h_s * kvY3);
    float kvY4 = h_s * accels.ay_m_s2;
    float kvX4 = h_s * accels.ax_m_s2;

    RK4State updatedState;
    updatedState.y_m = (state.y_m + (ka1 + 2 * ka2 + 2 * ka3 + ka4) / 6);
    updatedState.vy_m_s =
        (state.vy_m_s + (kvY1 + 2 * kvY2 + 2 * kvY3 + kvY4) / 6);
    updatedState.vx_m_s =
        (state.vx_m_s + (kvX1 + 2 * kvX2 + 2 * kvX3 + kvX4) / 6);

    return updatedState;
}

float getMaxAltitude_m(float vy_m_s, float vx_m_s, float y_m) {
    float prevAlt = 0.0;  // variable to store previous altitude

    RK4State states;

    states.y_m = y_m;
    states.vy_m_s = vy_m_s;
    states.vx_m_s = vx_m_s;

    while (states.y_m >= prevAlt) {
        prevAlt =
            states.y_m;  // to check if altitude is decreasing to exit the loop
        states = rk4(TIME_STEP, EXTENSION_REFERENCE,
                     states);  // update velocity and altitude
    }

    return prevAlt;
}
