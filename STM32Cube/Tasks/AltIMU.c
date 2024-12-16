struct AccelerometerData {
    int16_t x;
    int16_t y;
    int16_t z;
}

struct GyroscopeData {
    int16_t x;
    int16_t y;
    int16_t z;
}

struct MagnetometerData {
    int16_t x;
    int16_t y;
    int16_t z;
}

struct BarometerData {
    float pressure;
    float temperature;
}

struct IMU_AddressInfo {
    uint8_t acc_addr;
    uint8_t gyr_addr;
    uint8_t mag_addr;
    uint8_t bar_addr;
    uint8_t who_am_i_reg;
}

enum Protocol {
    I2C_PROTOCOL,
    SPI_PROTOCOL,
    UART_PROTOCOL,
    UNKNOWN_PROTOCOL
}

void ALTIMU_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t INT1_CTRL = 0x01;    // Acc data-ready interrupt on INT1
    uint8_t CTRL1_XL = 0x60;     // ODR = 416 Hz, FS = ±2g
    uint8_t CTRL2_G = 0x60;      // ODR = 416 Hz, FS = ±250 dps
    
    uint8_t CTRL_REG1 = 0x70;	//magnetometer control reg 1
    uint8_t CTRL_REG2 = 0x0;	//default full scale config = ±4 gauss
    uint8_t CTRL_REG3 = 0x00; // Continuous-conversion mode
    uint8_t CTRL_REG4 = 0x0c;	//z axis operating mode selection

    uint8_t temp_data = 0;

    // Initialize accelerometer and gyroscope settings
    HAL_I2C_Mem_Write(hi2c, (0b01101010 << 1), 0x10, 1, &CTRL1_XL, 1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Write(hi2c, (0b01101010 << 1), 0x11, 1, &CTRL2_G, 1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Write(hi2c, (0b01101010 << 1), 0x0D, 1, &INT1_CTRL, 1, HAL_MAX_DELAY);
    
    HAL_I2C_Mem_Write(hi2c, (0b0011100 << 1), 0x20, 1, &CTRL_REG1, 1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Write(hi2c, (0b0011100 << 1), 0x21, 1, &CTRL_REG2, 1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Write(hi2c, (0b0011100 << 1), 0x22, 1, &CTRL_REG3, 1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Write(hi2c, (0b0011100 << 1), 0x23, 1, &CTRL_REG4, 1, HAL_MAX_DELAY);


    // Check device communication
    HAL_I2C_IsDeviceReady(hi2c, (0b01101010 << 1), 1, 100);
    HAL_I2C_Master_Transmit(hi2c, (0b01101010 << 1), &temp_data, 1, 100);

    // Read WHO_AM_I register
    HAL_I2C_Mem_Read(hi2c, (0b01101010 << 1) + 1, 0x0F, 1, &temp_data, 1, 100);
    HAL_I2C_Mem_Read(hi2c, (0b0011100 << 1) + 1, 0x0F, 1, &temp_data, 1, 100);

    // Verify device ID
    if (temp_data != 0x6C) {
        Error_Handler();
    }
}

AccelerometerData IMU_Get_ACC_Data() {
    uint8_t accel_data[6]; // Array to store X, Y, Z values
    AccelerometerData acc_data;

    // Read accelerometer data registers (0x28-0x2D for X, Y, Z)
    HAL_I2C_Mem_Read(&hi2c1, (0b01101010 << 1), 0x28, 1, accel_data, 6, HAL_MAX_DELAY);

    // Combine low and high bytes for each axis
    acc_data.x = (int16_t)(accel_data[1] << 8 | accel_data[0]);
    acc_data.y = (int16_t)(accel_data[3] << 8 | accel_data[2]);
    acc_data.z = (int16_t)(accel_data[5] << 8 | accel_data[4]);

    return acc_data;
}

GyroscopeData IMU_Get_GYR_Data() {
    uint8_t gyro_data[6]; // Array to store X, Y, Z values
    GyroscopeData gyr_data;

    // Read gyroscope data registers (0x22-0x27 for X, Y, Z)
    HAL_I2C_Mem_Read(&hi2c1, (0b01101010 << 1), 0x22, 1, gyro_data, 6, HAL_MAX_DELAY);

    // Combine low and high bytes for each axis
    gyr_data.x = (int16_t)(gyro_data[1] << 8 | gyro_data[0]);
    gyr_data.y = (int16_t)(gyro_data[3] << 8 | gyro_data[2]);
    gyr_data.z = (int16_t)(gyro_data[5] << 8 | gyro_data[4]);

    return gyr_data;
}

MagnetometerData IMU_Get_MAG_Data() {
    uint8_t mag_data[6]; // Array to store X, Y, Z values
    MagnetometerData mag_data_struct;

    // Read magnetometer data registers (0x28-0x2D for X, Y, Z)
    // Note the slight difference in I2C address (+1 for read)
    HAL_I2C_Mem_Read(&hi2c1, (0b0011100 << 1) + 1, 0x28, 1, mag_data, 6, HAL_MAX_DELAY);

    // Combine low and high bytes for each axis
    mag_data_struct.x = (int16_t)(mag_data[1] << 8 | mag_data[0]);
    mag_data_struct.y = (int16_t)(mag_data[3] << 8 | mag_data[2]);
    mag_data_struct.z = (int16_t)(mag_data[5] << 8 | mag_data[4]);

    return mag_data_struct;
}

BarometerData IMU_Get_BAR_Data() {
    uint8_t pressure_data[3];
    uint8_t temp_data[2];
    BarometerData bar_data;

    // Read pressure data (assuming 24-bit pressure register)
    HAL_I2C_Mem_Read(&hi2c1, (0b1011100 << 1), 0x28, 1, pressure_data, 3, HAL_MAX_DELAY);

    // Read temperature data (assuming 16-bit temperature register)
    HAL_I2C_Mem_Read(&hi2c1, (0b1011100 << 1), 0x2B, 1, temp_data, 2, HAL_MAX_DELAY);

    // Convert pressure (24-bit to float)
    int32_t raw_pressure = (pressure_data[2] << 16) | (pressure_data[1] << 8) | pressure_data[0];
    bar_data.pressure = raw_pressure / 4096.0f; // Typical conversion for pressure sensors

    // Convert temperature (16-bit to float)
    int16_t raw_temp = (temp_data[1] << 8) | temp_data[0];
    bar_data.temperature = raw_temp / 100.0f; // Typical conversion for temperature

    return bar_data;
}

bool IMU_Check_ACC_Data(AccelerometerData data) { // In g, assuming +16g/-16g range
    if (-16 <= data.x <= 16 && -16 <= data.y <= 16 && -16 <= data.z <= 16) {
        return true;
    }
    return false;
}

bool IMU_Check_GYR_Data(GyroscopeData data) { // In dps, assuming +2000/-2000 dps range
    if (-2000 <= data.x <= 2000 && -2000 <= data.y <= 2000 && -2000 <= data.z <= 2000) {
        return true;
    }
    return false;
}

bool IMU_Check_MAG_Data(MagnetometerData data) { // In Gauss
    if (1 <= data.x <= 3 && 1 <= data.y <= 3 && 0.1 <= data.z <= 1) {
        return true;
    }
    return false;
}

bool IMU_Check_BAR_Data(BarometerData data) { // In hPa and Celsius
    if (260 <= data.pressure <= 1260 && -40 <= data.temperature <= 85) {
        return true;
    }
    return false;
}
 
bool read_sensor_with_timeout(uint8_t reg_addr, void* data, uint32_t timeout_ms) {
    uint32_t start_time = get_current_time_ms();
    
    while ((get_current_time_ms() - start_time) < timeout_ms) {
        if (read_register(reg_addr, data)) {
            return true;
        }
        delay_ms(1);
    }
    
    return false;
}

bool IMU_Check_Sanity() {
    const uint32_t TIMEOUT_MS = 100;
    uint8_t who_am_i_value;
    
    // Read WHO_AM_I register for each sensor
    if (!read_register_with_timeout(WHO_AM_I_REG, &who_am_i_value, TIMEOUT_MS)) {
        return false;
    }
    
    return (who_am_i_value == EXPECTED_WHO_AM_I_VALUE);
}

bool disable_accelerometer() {
    // Disable accelerometer
    uint8_t CTRL1_XL = 0x00;
    return write_register(CTRL1_XL_REG, &CTRL1_XL);
}

bool disable_gyroscope() {
    // Disable gyroscope
    uint8_t CTRL2_G = 0x00;
    return write_register(CTRL2_G_REG, &CTRL2_G);
}

bool disable_magnetometer() {
    // Disable magnetometer
    uint8_t CTRL_REG1_M = 0x00;
    return write_register(CTRL_REG1_M_REG, &CTRL_REG1_M);
}

bool disable_barometer() {
    // Disable barometer
    uint8_t CTRL_REG1 = 0x00;
    return write_register(CTRL_REG1_REG, &CTRL_REG1);
}


bool disable_sensors() {
    // Disable all sensors
    if (!disable_accelerometer()) {
        return false;
    }
    if (!disable_gyroscope()) {
        return false;
    }
    if (!disable_magnetometer()) {
        return false;
    }
    if (!disable_barometer()) {
        return false;
    }
    
    return true;
}

bool reset_communication() {
    // Reset communication interface
    if (!reset_i2c()) {
        return false;
    }
    if (!reset_spi()) {
        return false;
    }
    if (!reset_uart()) {
        return false;
    }
    
    return true;
}

bool IMU_Reset() {
    // Disable all sensors
    if (!disable_sensors()) {
        return false;
    }
    
    // Reset communication interface
    if (!reset_communication()) {
        return false;
    }
    
    // Wait for reset to complete
    delay_ms(100);
    
    // Reinitialize everything
    return ALTIMU_Init(current_protocol);
}

int main(void)
{
    // Initialize hardware and peripherals
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_SPI1_Init();
    MX_USART2_UART_Init();


    // Initialize IMU_AddressInfo structure (placeholders, need to check actual values)
    IMU_AddressInfo imu_info = {
        .acc_addr = 0x6A,
        .gyr_addr = 0x6A,
        .mag_addr = 0x1E,
        .bar_addr = 0x5D,
        .who_am_i_reg = WHO_AM_I_REG
    };

    // Initialize AltIMU
    ALTIMU_Init(&hi2c1);


    // Main loop
    while (1) {
        // Check sanity
        if (!IMU_Check_Sanity()) { 
            // Handle error or try to reset
            if (!IMU_Reset()) {
                Error_Handler();
            }
            continue;
        }

        // Read accelerometer and gyroscope data
        AccelerometerData acc_data = IMU_Get_ACC_Data();
        GyroscopeData gyr_data = IMU_Get_GYR_Data();
	    MagnetometerData mag_data = IMU_Get_MAG_Data();
	    BarometerData bar_data = IMU_Get_BAR_Data();

        // Check data validity
        if (!IMU_Check_ACC_Data(acc_data) || !IMU_Check_GYR_Data(gyr_data) || !IMU_Check_MAG_Data(mag_data) || !IMU_Check_BAR_Data(bar_data)) {
            // Handle error or try to reset
            if (!IMU_Reset()) {
                Error_Handler();  // Placeholder for error handling
            }
            continue;
        }
        // Add a small delay to control the sampling rate
        HAL_Delay(1000);
    }
}
