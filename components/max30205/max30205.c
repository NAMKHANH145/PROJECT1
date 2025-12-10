#include "max30205.h"
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"

static const char *TAG = "MAX30205";

/**
 * @brief Scan I2C bus for MAX30205 at common addresses
 * @return I2C address if found, 0 if not found
 */
static uint8_t max30205_scan_bus(i2c_port_t i2c_bus_num) {
    // MAX30205 possible addresses: 0x48-0x4F
    uint8_t possible_addrs[] = {0x48, 0x49, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E, 0x4F};
    
    ESP_LOGI(TAG, "Bus %d: Scanning for MAX30205...", i2c_bus_num);
    
    for (int i = 0; i < sizeof(possible_addrs); i++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (possible_addrs[i] << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t err = i2c_master_cmd_begin(i2c_bus_num, cmd, pdMS_TO_TICKS(50));
        i2c_cmd_link_delete(cmd);
        
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Bus %d: Found device at 0x%02X (might be MAX30205)", i2c_bus_num, possible_addrs[i]);
            return possible_addrs[i];
        }
    }
    
    ESP_LOGW(TAG, "Bus %d: No MAX30205 found at any address (0x48-0x4F)", i2c_bus_num);
    return 0;
}

/**
 * @brief Attempt to recover I2C bus by sending clock pulses
 * This helps clear stuck I2C bus states
 */
static void max30205_recover_i2c_bus(i2c_port_t i2c_bus_num) {
    // Try to clear I2C bus by toggling SCL
    // Note: This is a best-effort recovery, may not work if hardware is faulty
    ESP_LOGW(TAG, "Bus %d: Attempting I2C bus recovery...", i2c_bus_num);
    
    // Small delay to let bus settle
    vTaskDelay(pdMS_TO_TICKS(50));
}

/**
 * @brief Write a single byte to MAX30205 register
 */
static esp_err_t max30205_write_reg(i2c_port_t i2c_bus_num, uint8_t reg, uint8_t value) {
    uint8_t write_buf[2] = {reg, value};
    esp_err_t err = i2c_master_write_to_device(i2c_bus_num, MAX30205_I2C_ADDR,
                                               write_buf, sizeof(write_buf),
                                               pdMS_TO_TICKS(1000));
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Bus %d: Write reg 0x%02X failed (err=%d, %s)", 
                 i2c_bus_num, reg, err, esp_err_to_name(err));
    }
    return err;
}
   
// static esp_err_t max30205_read_reg removed (unused)
   
esp_err_t max30205_init(i2c_port_t i2c_bus_num) {
    esp_err_t err;

    ESP_LOGI(TAG, "Initializing MAX30205 on Bus %d (Expected Address 0x%02X)...", i2c_bus_num, MAX30205_I2C_ADDR);

    // First, probe the device to see if it's present at expected address
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MAX30205_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    esp_err_t probe_err = i2c_master_cmd_begin(i2c_bus_num, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    if (probe_err != ESP_OK) {
        ESP_LOGW(TAG, "Bus %d: MAX30205 not found at expected address 0x%02X. Scanning bus...", 
                 i2c_bus_num, MAX30205_I2C_ADDR);
        
        // Scan for MAX30205 at other possible addresses
        uint8_t found_addr = max30205_scan_bus(i2c_bus_num);
        if (found_addr == 0) {
            ESP_LOGE(TAG, "Bus %d: MAX30205 not found at any address (0x48-0x4F).", i2c_bus_num);
            ESP_LOGE(TAG, "Bus %d: Hardware checklist:", i2c_bus_num);
            ESP_LOGE(TAG, "  1) Power: VCC (2.7-3.3V) and GND connected?");
            ESP_LOGE(TAG, "  2) I2C: SDA and SCL connected? Pull-up resistors (4.7kΩ) present?");
            ESP_LOGE(TAG, "  3) Address pins A0, A1, A2: If floating, address may be unstable!");
            ESP_LOGE(TAG, "     -> Connect A0, A1, A2 to GND for address 0x48 (recommended)");
            ESP_LOGE(TAG, "  4) OS pin: Should be HIGH (VCC) or floating (not GND)");
            ESP_LOGE(TAG, "  5) Verify sensor is on same I2C bus as MPU6050 (Bus 1)");
            return probe_err;
        } else {
            ESP_LOGW(TAG, "Bus %d: MAX30205 found at 0x%02X instead of 0x%02X!", 
                     i2c_bus_num, found_addr, MAX30205_I2C_ADDR);
            ESP_LOGW(TAG, "Bus %d: Update MAX30205_I2C_ADDR to 0x%02X in components/max30205/include/max30205.h", 
                     i2c_bus_num, found_addr);
            // Note: We can't change the address at runtime, user needs to update header file
            return ESP_ERR_NOT_FOUND;
        }
    }
    
    ESP_LOGI(TAG, "Bus %d: MAX30205 detected at 0x%02X", i2c_bus_num, MAX30205_I2C_ADDR);

    // Configure for continuous conversion mode (default)
    // Shutdown bit = 0, Continuous conversion
    // Also clear any error flags by writing 0x00
    err = max30205_write_reg(i2c_bus_num, MAX30205_REG_CONFIGURATION, 0x00);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Bus %d: Failed to write configuration register (err=%d, %s)", 
                 i2c_bus_num, err, esp_err_to_name(err));
        return err;
    }

    // Wait for first conversion (MAX30205 needs ~100ms for first conversion)
    // Additional delay to ensure sensor is ready
    vTaskDelay(pdMS_TO_TICKS(200)); // Increased delay for first conversion

    // Test read to verify sensor is responding
    float test_temp;
    err = max30205_read_temperature(i2c_bus_num, &test_temp);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Bus %d: Sensor not responding after init, check connections", i2c_bus_num);
        return err;
    }
    
    // Sanity check: body temperature should be between 10-50°C for ambient/body
    if (test_temp < 10.0f || test_temp > 60.0f) {
        ESP_LOGW(TAG, "Bus %d: Temperature reading (%.2f°C) outside usual range (10-60°C)",
                 i2c_bus_num, test_temp);
    }

    ESP_LOGI(TAG, "Bus %d: MAX30205 initialized successfully (%.2f°C)",
             i2c_bus_num, test_temp);
    return ESP_OK;
}
   
esp_err_t max30205_read_temperature(i2c_port_t i2c_bus_num, float *temperature) {
    if (temperature == NULL) {
        ESP_LOGE(TAG, "Bus %d: NULL temperature pointer", i2c_bus_num);
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t reg_addr = MAX30205_REG_TEMPERATURE;
    uint8_t data[2];
    esp_err_t err = ESP_FAIL;

    // Try two different methods: write_read_device (faster) and separate write+read (more reliable)
    for (int attempt = 0; attempt < 3; attempt++) {
        // Method 1: Try write_read_device first (Repeated Start - faster)
        err = i2c_master_write_read_device(i2c_bus_num, MAX30205_I2C_ADDR,
                                           &reg_addr, 1,
                                           data, 2,
                                           pdMS_TO_TICKS(1000));
        
        // If that fails, try separate write and read (more reliable for some sensors)
        if (err != ESP_OK && attempt > 0) {
            // Write pointer register
            esp_err_t write_err = i2c_master_write_to_device(i2c_bus_num, MAX30205_I2C_ADDR,
                                                            &reg_addr, 1,
                                                            pdMS_TO_TICKS(100));
            if (write_err == ESP_OK) {
                // Small delay before read
                vTaskDelay(pdMS_TO_TICKS(5));
                // Read 2 bytes
                err = i2c_master_read_from_device(i2c_bus_num, MAX30205_I2C_ADDR,
                                                  data, 2,
                                                  pdMS_TO_TICKS(1000));
            } else {
                err = write_err;
            }
        }
        
        if (err == ESP_OK) break;
        
        // Log detailed error information
        const char* err_name = esp_err_to_name(err);
        ESP_LOGW(TAG, "Bus %d: Temp read failed (err=%d, %s) attempt %d", 
                 i2c_bus_num, err, err_name, attempt+1);
        
        if (attempt == 1) {
            // Attempt bus recovery before reconfiguration
            max30205_recover_i2c_bus(i2c_bus_num);
            
            // Reconfigure to exit any transient shutdown state
            esp_err_t write_err = max30205_write_reg(i2c_bus_num, MAX30205_REG_CONFIGURATION, 0x00);
            if (write_err != ESP_OK) {
                ESP_LOGW(TAG, "Bus %d: Failed to reconfigure (err=%d, %s). Sensor may be disconnected.", 
                         i2c_bus_num, write_err, esp_err_to_name(write_err));
            }
            vTaskDelay(pdMS_TO_TICKS(100)); // Longer delay after reconfiguration and recovery
        } else {
            vTaskDelay(pdMS_TO_TICKS(30)); // Increased delay between attempts
        }
    }

    if (err != ESP_OK) {
        const char* err_name = esp_err_to_name(err);
        ESP_LOGE(TAG, "Bus %d: Failed to read temp after 3 attempts (I2C Error %d: %s)", 
                 i2c_bus_num, err, err_name);
        
        // Attempt I2C bus recovery before final probe
        max30205_recover_i2c_bus(i2c_bus_num);
        
        // Additional diagnostic: Try to detect if device is present
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (MAX30205_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t probe_err = i2c_master_cmd_begin(i2c_bus_num, cmd, pdMS_TO_TICKS(100));
        i2c_cmd_link_delete(cmd);
        
        if (probe_err == ESP_OK) {
            ESP_LOGW(TAG, "Bus %d: Device at 0x%02X responds to probe, but read fails. Possible timing issue.", 
                     i2c_bus_num, MAX30205_I2C_ADDR);
        } else {
            ESP_LOGE(TAG, "Bus %d: Device at 0x%02X not responding to probe (err=%d, %s).", 
                     i2c_bus_num, MAX30205_I2C_ADDR, probe_err, esp_err_to_name(probe_err));
            ESP_LOGE(TAG, "Bus %d: Possible causes: 1) Wiring issue (SDA/SCL), 2) Wrong I2C address, 3) Sensor not powered, 4) I2C bus stuck", 
                     i2c_bus_num);
        }
        
        return err;
    }

    // Combine MSB and LSB (Big-Endian format)
    int16_t raw_temp = (int16_t)((data[0] << 8) | data[1]);
    *temperature = (float)raw_temp * 0.00390625f;
    return ESP_OK;
}