#include "max30102.h"
#include "esp_log.h" 
#include "driver/i2c.h"

static const char *TAG = "MAX30102";

// Hàm nội bộ: Ghi 1 byte
static esp_err_t max30102_write_reg(i2c_port_t i2c_bus_num, uint8_t reg, uint8_t value) {
    uint8_t write_buf[2] = {reg, value};
    return i2c_master_write_to_device(i2c_bus_num, MAX30102_I2C_ADDR, write_buf, sizeof(write_buf), pdMS_TO_TICKS(1000));
}

// Hàm nội bộ: Đọc N byte
static esp_err_t max30102_read_reg(i2c_port_t i2c_bus_num, uint8_t reg, uint8_t *data, size_t len) {
    return i2c_master_write_read_device(i2c_bus_num, MAX30102_I2C_ADDR, &reg, 1, data, len, pdMS_TO_TICKS(1000));
}

esp_err_t max30102_init(i2c_port_t i2c_bus_num) {
    esp_err_t err;
    vTaskDelay(pdMS_TO_TICKS(100)); 

    // 1. Check ID
    uint8_t part_id = 0;
    max30102_read_reg(i2c_bus_num, MAX30102_PART_ID, &part_id, 1); // Dummy read
    vTaskDelay(pdMS_TO_TICKS(10));
    err = max30102_read_reg(i2c_bus_num, MAX30102_PART_ID, &part_id, 1);
    
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Bus %d: I2C Error reading Part ID", i2c_bus_num);
        return err;
    }

    if (part_id != 0x15) {
        ESP_LOGW(TAG, "Bus %d: Unknown Part ID (0x%02X). Continuing...", i2c_bus_num, part_id);
    } else {
        ESP_LOGI(TAG, "Bus %d: Found MAX30102 (ID: 0x%02X)", i2c_bus_num, part_id);
    }

    // 2. Reset
    max30102_write_reg(i2c_bus_num, MAX30102_MODE_CONFIG, 0x40);
    vTaskDelay(pdMS_TO_TICKS(100)); 

    // 3. Config cơ bản (Mức thấp để tiết kiệm điện lúc đầu)
    max30102_write_reg(i2c_bus_num, MAX30102_FIFO_CONFIG, 0x5F); 
    max30102_write_reg(i2c_bus_num, MAX30102_MODE_CONFIG, 0x03); 
    max30102_write_reg(i2c_bus_num, MAX30102_SPO2_CONFIG, 0x27);
    max30102_write_reg(i2c_bus_num, MAX30102_LED_PULSE_AMP_1, 0x24); // ~7mA
    max30102_write_reg(i2c_bus_num, MAX30102_LED_PULSE_AMP_2, 0x24); // ~7mA
    max30102_write_reg(i2c_bus_num, MAX30102_FIFO_WR_PTR, 0x00);
    max30102_write_reg(i2c_bus_num, MAX30102_FIFO_RD_PTR, 0x00);

    return ESP_OK;
}

// [MỚI] Hàm Boost LED chuyên dụng
// [TUNED V2] Giảm dòng LED xuống ~9mA (0x2F) để tránh bão hòa hoàn toàn
esp_err_t max30102_boost_leds(i2c_port_t i2c_bus_num) {
    esp_err_t ret = ESP_OK;
    ESP_LOGW(TAG, "⚡ Tuning LED Current to ~9mA (Low-Medium Sensitivity)...");

    // Thử mức 0x2F (~9mA) hoặc 0x3F (~12mA)
    // Mức cũ 0x4F (~15mA) vẫn gây bão hòa ở một số người dùng da mỏng
    uint8_t current_val = 0x2F; 

    // Reg 0x0C (LED1 - Red)
    ret |= max30102_write_reg(i2c_bus_num, MAX30102_LED_PULSE_AMP_1, current_val); 
    // Reg 0x0D (LED2 - IR)
    ret |= max30102_write_reg(i2c_bus_num, MAX30102_LED_PULSE_AMP_2, current_val); 
    
    // Reg 0x0A (SpO2 Config): 
    // ADC Range = 4096nA (01), Sample Rate = 100Hz (001), Pulse Width = 411us (11) -> 0x27
    // Giữ nguyên cấu hình này để lấy độ phân giải cao nhất
    ret |= max30102_write_reg(i2c_bus_num, MAX30102_SPO2_CONFIG, 0x27);

    return ret;
}

esp_err_t max30102_read_fifo(i2c_port_t i2c_bus_num, max30102_data_t *data) {
    uint8_t ptrs[2];
    if (i2c_master_write_read_device(i2c_bus_num, MAX30102_I2C_ADDR, 
        (uint8_t[]){MAX30102_FIFO_WR_PTR}, 1, ptrs, 2, pdMS_TO_TICKS(100)) != ESP_OK) {
        return ESP_FAIL;
    }

    // Nếu WrPtr == RdPtr => Không có dữ liệu
    if (ptrs[0] == ptrs[1]) return ESP_ERR_NOT_FOUND;

    uint8_t fifo_data[6];
    esp_err_t err = max30102_read_reg(i2c_bus_num, MAX30102_FIFO_DATA, fifo_data, 6);
    if (err != ESP_OK) return err;

    data->red = ((uint32_t)fifo_data[0] << 16) | ((uint32_t)fifo_data[1] << 8) | fifo_data[2];
    data->ir  = ((uint32_t)fifo_data[3] << 16) | ((uint32_t)fifo_data[4] << 8) | fifo_data[5];

    data->red &= 0x03FFFF;
    data->ir  &= 0x03FFFF;

    return ESP_OK;
}