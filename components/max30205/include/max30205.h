#pragma once

#include "driver/i2c.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// MAX30205 I2C Address
// Address is determined by A0, A1, A2 pins:
// Address = 0x48 + (A2 << 2) + (A1 << 1) + A0
// - A0=GND, A1=GND, A2=GND: 0x48 (DEFAULT - most common)
// - A0=VCC, A1=GND, A2=GND: 0x49
// - A0=GND, A1=VCC, A2=GND: 0x4A
// - A0=VCC, A1=VCC, A2=GND: 0x4B
// - A0=GND, A1=GND, A2=VCC: 0x4C
// - A0=VCC, A1=GND, A2=VCC: 0x4D
// - A0=GND, A1=VCC, A2=VCC: 0x4E
// - A0=VCC, A1=VCC, A2=VCC: 0x4F
// OS pin: Overtemperature Shutdown (active low). Should be pulled high or left floating.
// If sensor not found, try scanning I2C bus to find correct address
#define MAX30205_I2C_ADDR           0x48

// Registers
#define MAX30205_REG_TEMPERATURE    0x00
#define MAX30205_REG_CONFIGURATION  0x01
#define MAX30205_REG_THYST          0x02
#define MAX30205_REG_TOS            0x03

/**
 * @brief Initialize the MAX30205 temperature sensor
 *
 * @param i2c_bus_num I2C bus number (e.g., I2C_NUM_0)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t max30205_init(i2c_port_t i2c_bus_num);

/**
 * @brief Read temperature in Celsius
 *
 * @param i2c_bus_num I2C bus number
 * @param temperature Pointer to float to store temperature
 * @return esp_err_t ESP_OK on success
 */
esp_err_t max30205_read_temperature(i2c_port_t i2c_bus_num, float *temperature);
  
#ifdef __cplusplus
}
#endif