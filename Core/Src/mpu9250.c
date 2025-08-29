#include "mpu9250.h"

// ================================= Function definitions ============================
// ================================ Private functions ============================
inline static void MPU9250_CS_Low(MPU9250_Handle *hmpu)
{
    HAL_GPIO_WritePin(hmpu->cs_port, hmpu->cs_pin, GPIO_PIN_RESET);
}

inline static void MPU9250_CS_High(MPU9250_Handle *hmpu)
{
    HAL_GPIO_WritePin(hmpu->cs_port, hmpu->cs_pin, GPIO_PIN_SET);
}

static void MPU9250_WriteReg(MPU9250_Handle *hmpu, uint8_t reg, uint8_t data)
{
    uint8_t tx_data[2] = {reg & 0x7F, data}; // Write data to the reg
    MPU9250_CS_Low(hmpu);                    // Set CS low to start communication
    HAL_SPI_Transmit(hmpu->hspi, tx_data, 2, WAIT_SPI_TIMEOUT);
    MPU9250_CS_High(hmpu); // Set CS high to end communication
}

static void MPU9250_ReadReg(MPU9250_Handle *hmpu, uint8_t reg, uint8_t *data, uint8_t len)
{
    reg |= 0x80;                                              // Set the read bit (MSB = 1)
    MPU9250_CS_Low(hmpu);                                     // Set CS low to start communication
    HAL_SPI_Transmit(hmpu->hspi, &reg, 1, WAIT_SPI_TIMEOUT);  // Send the register address
    HAL_SPI_Receive(hmpu->hspi, data, len, WAIT_SPI_TIMEOUT); // Receive the data from the register
    MPU9250_CS_High(hmpu);                                    // Set CS high to end communication
}

static void MPU9250_WriteMagReg(MPU9250_Handle *hmpu, uint8_t reg, uint8_t data)
{
    MPU9250_WriteReg(hmpu, AK8963_I2C_SLV0_ADDR, AK8963_ADDRESS << 1); // I2C_SLV0_ADDR (write)
    MPU9250_WriteReg(hmpu, AK8963_I2C_SLV0_REG, reg);                  // I2C_SLV0_REG
    MPU9250_WriteReg(hmpu, AK8963_I2C_SLV0_DO, data);                  // I2C_SLV0_DO
    MPU9250_WriteReg(hmpu, AK8963_I2C_SLV0_CTRL, 0x81); // I2C_SLV0_CTRL (enable, read 1 byte)
    HAL_Delay(20); // Wait for the magnetometer to process the command
}

// ================================== Public functions ============================
void MPU9250_Init(MPU9250_Handle *hmpu)
{
    // Initialize the MPU9250
    MPU9250_CS_High(hmpu); // Ensure CS is high before starting
    HAL_Delay(100);        // Wait for the MPU9250 to stabilize

    // Wake up the MPU9250
    MPU9250_WriteReg(hmpu, MPU9250_PWR_MGMT_1, 0x00);
    HAL_Delay(50); // Wait for the MPU9250 to wake up

    uint8_t cfg; // Read the current configuration

    /*
     Set the accelerometer and gyroscope full-scale range
     + accelerometer: ±4g
     + gyroscope: ±1000 dps
    */
    MPU9250_ReadReg(hmpu, MPU9250_ACCEL_CONFIG, &cfg, 1);
    cfg &= ~0x18; // 0x18 = 0b00011000, clear bits 3 and 4
    cfg |= 0x08;  // Set to ±4g
    MPU9250_WriteReg(hmpu, MPU9250_ACCEL_CONFIG, cfg);

    MPU9250_ReadReg(hmpu, MPU9250_GYRO_CONFIG, &cfg, 1);
    cfg &= ~0x18; // Clear bits 3 and 4
    cfg |= 0x10;  // Set to ±1000 dps
    MPU9250_WriteReg(hmpu, MPU9250_GYRO_CONFIG, cfg);

    MPU9250_InitMagnetometer(hmpu); // Initialize the magnetometer
}

void MPU9250_ReadAccel(MPU9250_Handle *hmpu, float *ax, float *ay, float *az)
{
    uint8_t data[6];                                      // Buffer to hold accelerometer data
    MPU9250_ReadReg(hmpu, MPU9250_ACCEL_XOUT_H, data, 6); // Read 6 bytes of accelerometer data

    // Combine high and low bytes to form 16-bit values
    int16_t raw_ax = (int16_t)((data[0] << 8) | data[1]);
    int16_t raw_ay = (int16_t)((data[2] << 8) | data[3]);
    int16_t raw_az = (int16_t)((data[4] << 8) | data[5]);

    // Divide by 8192.0 to convert to g
    *ax = (float)raw_ax / 8192.0f;
    *ay = (float)raw_ay / 8192.0f;
    *az = (float)raw_az / 8192.0f;
}

void MPU9250_ReadGyro(MPU9250_Handle *hmpu, float *gx, float *gy, float *gz)
{
    uint8_t data[6];                                     // Buffer to hold gyroscope data
    MPU9250_ReadReg(hmpu, MPU9250_GYRO_XOUT_H, data, 6); // Read 6 bytes of gyroscope data

    // Combine high and low bytes to form 16-bit values
    int16_t raw_gx = (int16_t)((data[0] << 8) | data[1]);
    int16_t raw_gy = (int16_t)((data[2] << 8) | data[3]);
    int16_t raw_gz = (int16_t)((data[4] << 8) | data[5]);

    // Divide by 32.8 to convert to degrees per second
    *gx = (float)raw_gx / 32.8f;
    *gy = (float)raw_gy / 32.8f;
    *gz = (float)raw_gz / 32.8f;
}

void MPU9250_InitMagnetometer(MPU9250_Handle *hmpu)
{
    // Initialize the AK8963 magnetometer
    // Enable I2C Master mode
    MPU9250_WriteReg(hmpu, MPU9250_USER_CTRL, 0x20);   // Enable I2C Master mode
    MPU9250_WriteReg(hmpu, MPU9250_I2C_MAS_CTR, 0x0D); // Set I2C Master clock to 400 kHz

    // Reset AK8963
    MPU9250_WriteMagReg(hmpu, AK8963_CNTL1, 0x00); // Reset the AK8963_CNTL1 register
    HAL_Delay(20);                                 // Wait for the magnetometer to reset

    // Set AK8963 to continuous measurement mode: 16-bit, continuous mode 2 100Hz (0x16)
    MPU9250_WriteMagReg(hmpu, AK8963_CNTL1, 0x16);
    HAL_Delay(20); // Wait for the magnetometer to set up

    // Configure the I2C slave to read magnetometer data from AK8963 (read 6 bytes)
    MPU9250_WriteReg(
        hmpu, AK8963_I2C_SLV0_ADDR, (AK8963_ADDRESS << 1) | 0x01); // I2C_SLV0_ADDR (read)
    MPU9250_WriteReg(hmpu, AK8963_I2C_SLV0_REG, AK8963_HXL);       // I2C_SLV0_REG (start address)
    MPU9250_WriteReg(hmpu, AK8963_I2C_SLV0_CTRL, 0x86); // I2C_SLV0_CTRL (enable, read 6 bytes)
}

void MPU9250_ReadMagnetometer(MPU9250_Handle *hmpu, float *mx, float *my, float *mz)
{
    uint8_t data[6];                                         // Buffer to hold magnetometer data
    MPU9250_ReadReg(hmpu, AK8963_EXT_SENS_DATA_00, data, 6); // EXT_SENS_DATA_00..05

    // Combine high and low bytes to form 16-bit values
    int16_t raw_mx = (int16_t)((data[1] << 8) | data[0]);
    int16_t raw_my = (int16_t)((data[3] << 8) | data[2]);
    int16_t raw_mz = (int16_t)((data[5] << 8) | data[4]);

    // Convert to microteslas (uT)
    *mx = (float)raw_mx * 0.15f; // Scale factor for AK8963 (LSB = 0.15 µT ở 16-bit mode)
    *my = (float)raw_my * 0.15f;
    *mz = (float)raw_mz * 0.15f;
}

void MPU9250_ReadAll(MPU9250_Handle *hmpu, MPU9250_Data *data)
{
    // Read accelerometer data
    MPU9250_ReadAccel(hmpu, &data->ax, &data->ay, &data->az);

    // Read gyroscope data
    MPU9250_ReadGyro(hmpu, &data->gx, &data->gy, &data->gz);

    // Read magnetometer data
    MPU9250_ReadMagnetometer(hmpu, &data->mx, &data->my, &data->mz);
}
