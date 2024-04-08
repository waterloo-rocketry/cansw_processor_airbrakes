#ifndef ICM_20948_H
#define	ICM_20948_H

#define ICM_20948_ADDR 0x69
#define AK09916_MAG_ADDR 0x0C

#include <stdint.h>
#include <stdbool.h>

// Set up gyroscope, accelerometer, and magnetometer
bool ICM_20948_init();

// Check if the device is alive. Fails if an I2C can't be established
// or if WHO_AM_I registers don't return default values.
bool ICM_20948_check_sanity(void);

// Perform magnetometer self-test procedure according to datasheet
bool MAG_Self_Test(void);

// Get IMU data: accelerometer, gyroscope, magnetometer, and temperature

bool ICM_20948_get_accel_raw(int16_t *x, int16_t *y, int16_t *z);

bool ICM_20948_get_gyro_raw(int16_t *x, int16_t *y, int16_t *z);

bool ICM_20948_get_mag_raw(int16_t *x, int16_t *y, int16_t *z);

#endif	/* ICM_20948_H */
