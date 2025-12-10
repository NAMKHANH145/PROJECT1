#pragma once
#include "driver/i2c.h"
#define MPU6050_I2C_ADDR    0x68
#define MPU6050_PWR_MGMT_1  0x6B
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_GYRO_XOUT_H 0x43
#define MPU6050_WHO_AM_I    0x75
typedef struct {
    int16_t accel_x, accel_y, accel_z;
    int16_t gyro_x, gyro_y, gyro_z;
} mpu6050_raw_data_t;
esp_err_t mpu6050_init(i2c_port_t i2c_bus_num);
esp_err_t mpu6050_read_raw_data(i2c_port_t i2c_bus_num, mpu6050_raw_data_t *data);