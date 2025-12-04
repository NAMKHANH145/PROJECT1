#pragma once

 #include "driver/i2c.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif
    
  // MAX30205 I2C Address (A0=0, A1=0, A2=0)
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