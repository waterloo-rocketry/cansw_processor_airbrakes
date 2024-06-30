#ifndef ICM_20948_H
#define	ICM_20948_H

#define ICM_20948_ADDR 0x69
#define AK09916_MAG_ADDR 0x0C

#include <stdint.h>
#include <stdbool.h>

// Initialize mutex, tests, etc. Does not need to be run from a task.
bool ICM_20948_init();

// Set up gyroscope, accelerometer, and magnetometer.
// !!! This must be run from a freertos task !!!
bool ICM_20948_setup();

// Check if the device is alive. Fails if an I2C can't be established
// or if WHO_AM_I registers don't return default values.
bool ICM_20948_check_sanity(void);

// Perform magnetometer self-test procedure according to datasheet
bool MAG_Self_Test(void);

/**
 * Get current accelerometer reading synchronously (blocking)
 * @return (gravities)
*/
bool ICM_20948_get_accel_converted(float *x, float *y, float *z);

/**
 * Get current gyroscope reading synchronously (blocking)
 * @return (deg/sec)
*/
bool ICM_20948_get_gyro_converted(float *x, float *y, float *z);

/**
 * Get current magnetometer reading synchronously (blocking)
 * @return (microteslas)
*/
bool ICM_20948_get_mag_converted(float *x, float *y, float *z);

#endif	/* ICM_20948_H */
