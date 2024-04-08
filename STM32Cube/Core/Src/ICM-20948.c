#include <stdint.h>
#include <stdbool.h>
#include "my2c.h"
#include "ICM-20948.h"
#include "ICM-20948_regmap.h"
#include "cmsis_os.h"

// This driver assumes that I2C is already initialized
bool ICM_20948_init() {
    MY2C_init();

    // Select user bank 0
    MY2C_write1ByteRegister(ICM_20948_ADDR, REG_BANK_SEL, 0x00);
    // Wake device and set clock to internal 20 MHz oscillator
    MY2C_write1ByteRegister(ICM_20948_ADDR, PWR_MGMT_1, 0x06);
    // Enable accelerometer and gyroscope
    MY2C_write1ByteRegister(ICM_20948_ADDR, PWR_MGMT_2, 0x00);
    // Set only I2C master to low power mode
    MY2C_write1ByteRegister(ICM_20948_ADDR, LP_CONFIG, 0b01000000);

    // Disable the i2c master module so that external sensors (mag) don't go through it
    // possibly redundant since its all 0 by default, but might as well confirm it
    MY2C_write1ByteRegister(ICM_20948_ADDR, USER_CTRL, 0x00);
    // Enable BYPASS_EN to set i2c master module pins into bypass mode
    // only takes effect when i2c master module is disabled
    MY2C_write1ByteRegister(ICM_20948_ADDR, INT_PIN_CFG, 0x02);

    // Reset magnetometer
    MY2C_write1ByteRegister(AK09916_MAG_ADDR, CNTL3, 0x01);
    
    // Select user bank 2
    MY2C_write1ByteRegister(ICM_20948_ADDR, REG_BANK_SEL, 0x20);
    // Set gyroscope full scale to +/-2000dps, rate = 9000 Hz
    MY2C_write1ByteRegister(ICM_20948_ADDR, GYRO_CONFIG_1, 0x06);
    // Set ODR for gyroscope to 50Hz. ODR = 1100HZ/(1+GYRO_SMPLRT_DIV[7:0])
    MY2C_write1ByteRegister(ICM_20948_ADDR, GYRO_SMPLRT_DIV, 0x15);

    // Set accelerometer full scale to +/- 16g, rate = 4500 Hz
    MY2C_write1ByteRegister(ICM_20948_ADDR, ACCEL_CONFIG, 0x06);
    // Set ODR for accelerometer to ~49Hz, ODR = 1125Hz/(1+ACCEL_SMPLRT_DIV[11:0])
    MY2C_write1ByteRegister(ICM_20948_ADDR, ACCEL_SMPLRT_DIV_1, 0x00);
    MY2C_write1ByteRegister(ICM_20948_ADDR, ACCEL_SMPLRT_DIV_2, 0x16);

    // Set to continuous measurement mode 3 (50 Hz measurement frequency)
    MY2C_write1ByteRegister(AK09916_MAG_ADDR, CNTL2, 0x06);

    return true;
}

bool ICM_20948_check_sanity(void) {
    // Select user bank 0
    MY2C_write1ByteRegister(ICM_20948_ADDR, REG_BANK_SEL, 0x00);

    uint8_t addr_sanity = MY2C_read1ByteRegister(ICM_20948_ADDR, WHO_AM_I);
    uint8_t mag_addr_sanity = MY2C_read1ByteRegister(AK09916_MAG_ADDR, WIA2);

    // Sanity fails if the "who am i" registers doesn't match
    if (addr_sanity != 0xEA || mag_addr_sanity != 0x09) {
        return false;
    }

    // checks pass
    return true;
}

/**
 * AK09916 self-test procedure
 * possibly not working
*/
bool MAG_Self_Test(void) {
    // reset to power-down mode
    MY2C_write1ByteRegister(AK09916_MAG_ADDR, CNTL2, 0x0);
    osDelay(1000);
    // set self-test mode
    MY2C_write1ByteRegister(AK09916_MAG_ADDR, CNTL2, 0x10);
    
    // bit 1 of this reg indicates whether data is ready
    uint8_t mag_data_status_1 = MY2C_read1ByteRegister(AK09916_MAG_ADDR, ST1);

    // wait until data is ready
    while (!(mag_data_status_1 & 1)) {
        mag_data_status_1 = MY2C_read1ByteRegister(AK09916_MAG_ADDR, ST1);
    }

    // get data
    int16_t x = 0;
    int16_t y = 0;
    int16_t z = 0;

    uint8_t x_h = MY2C_read1ByteRegister(AK09916_MAG_ADDR, HXH);
    uint8_t x_l = MY2C_read1ByteRegister(AK09916_MAG_ADDR, HXL);
    x = (int16_t)((uint16_t)x_h << 8 | x_l);

    uint8_t y_h = MY2C_read1ByteRegister(AK09916_MAG_ADDR, HYH);
    uint8_t y_l = MY2C_read1ByteRegister(AK09916_MAG_ADDR, HYL);
    y = (int16_t)((uint16_t)y_h << 8 | y_l);

    uint8_t z_h = MY2C_read1ByteRegister(AK09916_MAG_ADDR, HZH);
    uint8_t z_l = MY2C_read1ByteRegister(AK09916_MAG_ADDR, HZL);
    z = (int16_t)((uint16_t)z_h << 8 | z_l);

    // Must read ST2 register after measurement, see datasheet register 13.4 ST2
    MY2C_read1ByteRegister(AK09916_MAG_ADDR, ST2);

    // partially validate data according to self-test thresholds
    // view datasheet for full threshold
    if (x <= 200 && y <= 200 && z <= -200) {
        return true;
    }
}

bool ICM_20948_get_accel_raw(int16_t *x, int16_t *y, int16_t *z) {
    if (!x || !y || !z) { return false; }

    // Accelerometer measurement data
    uint8_t x_h = MY2C_read1ByteRegister(ICM_20948_ADDR, ACCEL_XOUT_H);
    uint8_t x_l = MY2C_read1ByteRegister(ICM_20948_ADDR, ACCEL_XOUT_L);
    *x = (int16_t)((uint16_t)x_h << 8 | x_l);

    uint8_t y_h = MY2C_read1ByteRegister(ICM_20948_ADDR, ACCEL_YOUT_H);
    uint8_t y_l = MY2C_read1ByteRegister(ICM_20948_ADDR, ACCEL_YOUT_L);
    *y = (int16_t)((uint16_t)y_h << 8 | y_l);

    uint8_t z_h = MY2C_read1ByteRegister(ICM_20948_ADDR, ACCEL_ZOUT_H);
    uint8_t z_l = MY2C_read1ByteRegister(ICM_20948_ADDR, ACCEL_ZOUT_L);
    *z = (int16_t)((uint16_t)z_h << 8 | z_l);

    return true;
}

bool ICM_20948_get_gyro_raw(int16_t *x, int16_t *y, int16_t *z) {
    if (!x || !y || !z) { return false; }

    uint8_t x_h = MY2C_read1ByteRegister(ICM_20948_ADDR, GYRO_XOUT_H);
    uint8_t x_l = MY2C_read1ByteRegister(ICM_20948_ADDR, GYRO_XOUT_L);
    *x = (int16_t)((uint16_t)x_h << 8 | x_l);

    uint8_t y_h = MY2C_read1ByteRegister(ICM_20948_ADDR, GYRO_YOUT_H);
    uint8_t y_l = MY2C_read1ByteRegister(ICM_20948_ADDR, GYRO_YOUT_L);
    *y = (int16_t)((uint16_t)y_h << 8 | y_l);

    uint8_t z_h = MY2C_read1ByteRegister(ICM_20948_ADDR, GYRO_ZOUT_H);
    uint8_t z_l = MY2C_read1ByteRegister(ICM_20948_ADDR, GYRO_ZOUT_L);
    *z = (int16_t)((uint16_t)z_h << 8 | z_l);

    return true;
}

bool ICM_20948_get_mag_raw(int16_t *x, int16_t *y, int16_t *z) {
    if (!x || !y || !z) { return false; }

     // Check if magnetometer data is ready, fail if it is not
    uint8_t mag_data_status_1 = MY2C_read1ByteRegister(AK09916_MAG_ADDR, ST1);

    // the 1st bit of this reg indicates whether data is ready (1) or not (0)
    if (!(mag_data_status_1 & 1)) {
        return false;
    }

    uint8_t x_h = MY2C_read1ByteRegister(AK09916_MAG_ADDR, HXH);
    uint8_t x_l = MY2C_read1ByteRegister(AK09916_MAG_ADDR, HXL);
    *x = (int16_t)((uint16_t)x_h << 8 | x_l);

    uint8_t y_h = MY2C_read1ByteRegister(AK09916_MAG_ADDR, HYH);
    uint8_t y_l = MY2C_read1ByteRegister(AK09916_MAG_ADDR, HYL);
    *y = (int16_t)((uint16_t)y_h << 8 | y_l);

    uint8_t z_h = MY2C_read1ByteRegister(AK09916_MAG_ADDR, HZH);
    uint8_t z_l = MY2C_read1ByteRegister(AK09916_MAG_ADDR, HZL);
    *z = (int16_t)((uint16_t)z_h << 8 | z_l);

    // Must read ST2 register after measurement, see datasheet register 13.4 ST2
    MY2C_read1ByteRegister(AK09916_MAG_ADDR, ST2);

    return true;
}
