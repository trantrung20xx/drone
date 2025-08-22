#ifndef INC_MPU9250_H_
#define INC_MPU9250_H_

#include "main.h"
#include <stdint.h>

// Waiting time for SPI communication
#define WAIT_SPI_TIMEOUT 100 // ms

// ================================= MPU9250 SPI Addresses ============================
#define MPU9250_WHO_AM_I     0x75
#define MPU9250_PWR_MGMT_1   0x6B
#define MPU9250_ACCEL_CONFIG 0x1C
#define MPU9250_GYRO_CONFIG  0x1B
#define MPU9250_ACCEL_XOUT_H 0x3B
#define MPU9250_GYRO_XOUT_H  0x43
#define MPU9250_USER_CTRL    0x6A
#define MPU9250_I2C_MAS_CTR  0x24

// AK8963 Magnetometer addresses
#define AK8963_ADDRESS          0x0C // I2C address for AK8963
#define AK8963_WHO_AM_I         0x00
#define AK8963_I2C_SLV0_ADDR    0x25
#define AK8963_I2C_SLV0_REG     0x26
#define AK8963_I2C_SLV0_CTRL    0x27
#define AK8963_I2C_SLV0_DO      0x63
#define AK8963_CNTL1            0x0A
#define AK8963_EXT_SENS_DATA_00 0x49
#define AK8963_HXL              0x03

// ================================= Struct management ============================
typedef struct {
    SPI_HandleTypeDef* hspi;    // Pointer to the SPI handle
    GPIO_TypeDef*      cs_port; // Chip select port
    uint8_t            cs_pin;  // Chip select pin
} MPU9250_Handle;

typedef struct {
    float ax, ay, az; // Accelerometer data
    float gx, gy, gz; // Gyroscope data
    float mx, my, mz; // Magnetometer data
} MPU9250_Data;

// ================================= Function prototypes ============================
void MPU9250_Init(MPU9250_Handle* hmpu);
void MPU9250_ReadAccel(MPU9250_Handle* hmpu, float* ax, float* ay, float* az);
void MPU9250_ReadGyro(MPU9250_Handle* hmpu, float* gx, float* gy, float* gz);
void MPU9250_InitMagnetometer(MPU9250_Handle* hmpu);
void MPU9250_ReadMagnetometer(MPU9250_Handle* hmpu, float* mx, float* my, float* mz);
void MPU9250_ReadAll(MPU9250_Handle* hmpu, MPU9250_Data* data);

#endif /* INC_MPU9250_H_ */
