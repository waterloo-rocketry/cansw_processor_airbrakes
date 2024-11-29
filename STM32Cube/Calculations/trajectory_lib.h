#ifndef TRAJECTORY_LIB_H_
#define TRAJECTORY_LIB_H_

#define EXTENSION_REFERENCE 0.53f

/**
 * Returns the acceleration due to drag acting on the rocket.
 * */
float dragAccel_m_s2(float extension, float speed_m_s, float altitude_m);

/**
 * Computes the expected apogee with the reference extension value and burnout
 * mass given the current velocity and altitude.
 */
float getMaxAltitude_m(float vy_m_s, float vx_m_s, float y_m);

float second_order_lowpass_filter(float input, float* values, float alpha);

float moving_average_filter(float input, float* values);

#endif
