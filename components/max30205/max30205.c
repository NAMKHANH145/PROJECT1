    #include "max30205.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "MAX30205";

/**
 * @brief Write a single byte to MAX30205 register
 */
static esp_err_t max30205_write_reg(i2c_port_t i2c_bus_num, uint8_t reg, uint8_t value) {
    uint8_t write_buf[2] = {reg, value};
    return i2c_master_write_to_device(i2c_bus_num, MAX30205_I2C_ADDR,
                                     write_buf, sizeof(write_buf),
                                     pdMS_TO_TICKS(1000));
}
   
/**
 * @brief Read N bytes from MAX30205 register
 */
static esp_err_t max30205_read_reg(i2c_port_t i2c_bus_num, uint8_t reg,
                                    uint8_t *data, size_t len) {
    return i2c_master_write_read_device(i2c_bus_num, MAX30205_I2C_ADDR,
                                       &reg, 1, data, len,
                                       pdMS_TO_TICKS(1000));
}
   
esp_err_t max30205_init(i2c_port_t i2c_bus_num) {
    esp_err_t err;

    ESP_LOGI(TAG, "Initializing MAX30205 on Bus %d...", i2c_bus_num);

    // Configure for continuous conversion mode (default)
    // Shutdown bit = 0, Continuous conversion
    err = max30205_write_reg(i2c_bus_num, MAX30205_REG_CONFIGURATION, 0x00);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Bus %d: Failed to write configuration register", i2c_bus_num);
        return err;
    }

    // Wait for first conversion (typically 100ms)
    vTaskDelay(pdMS_TO_TICKS(100));

    // Test read to verify sensor is responding
    float test_temp;
    err = max30205_read_temperature(i2c_bus_num, &test_temp);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Bus %d: Sensor not responding, check connections", i2c_bus_num);
        return err;
    }
    // Sanity check: body temperature should be between 10-50°C for ambient/body
    if (test_temp < 10.0f || test_temp > 60.0f) {
        ESP_LOGW(TAG, "Bus %d: Temperature reading (%.2f°C) outside usual range",
                 i2c_bus_num, test_temp);
    }

    ESP_LOGI(TAG, "Bus %d: MAX30205 initialized successfully (%.2f°C)",
             i2c_bus_num, test_temp);
    return ESP_OK;
}
   
esp_err_t max30205_read_temperature(i2c_port_t i2c_bus_num, float *temperature) {
    uint8_t data[2];
    esp_err_t err;
    // Read 16-bit temperature register
    err = max30205_read_reg(i2c_bus_num, MAX30205_REG_TEMPERATURE, data, 2);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Bus %d: Failed to read temperature register", i2c_bus_num);
        return err;
    }

    // Combine MSB and LSB (Big-Endian format)
    int16_t raw_temp = (int16_t)((data[0] << 8) | data[1]);
    // Convert to Celsius (resolution: 0.00390625°C per LSB)
    // The raw value is left-aligned in 16 bits, with 8 fractional bits
    *temperature = (float)raw_temp * 0.00390625f;

    return ESP_OK;
   }