#include "max30102.h"
#include "esp_log.h" // Dùng "log/log.h" (thay vì "esp_log.h") để khớp với "log" trong CMakeLists
#include "driver/i2c.h"

static const char *TAG = "MAX30102";

// Hàm nội bộ: Ghi 1 byte vào thanh ghi
static esp_err_t max30102_write_reg(i2c_port_t i2c_bus_num, uint8_t reg, uint8_t value) {
    uint8_t write_buf[2] = {reg, value};
    // Dùng hàm I2C chuẩn của ESP-IDF
    return i2c_master_write_to_device(i2c_bus_num, MAX30102_I2C_ADDR, write_buf, sizeof(write_buf), pdMS_TO_TICKS(1000));
}

// Hàm nội bộ: Đọc N byte từ thanh ghi
static esp_err_t max30102_read_reg(i2c_port_t i2c_bus_num, uint8_t reg, uint8_t *data, size_t len) {
    // Dùng hàm I2C chuẩn của ESP-IDF
    return i2c_master_write_read_device(i2c_bus_num, MAX30102_I2C_ADDR, &reg, 1, data, len, pdMS_TO_TICKS(1000));
}

// Hàm khởi tạo (public) - ĐÃ SỬA LỖI
esp_err_t max30102_init(i2c_port_t i2c_bus_num) {
    esp_err_t err;

    // 1. Kiểm tra ID thiết bị
    uint8_t part_id;
    err = max30102_read_reg(i2c_bus_num, MAX30102_PART_ID, &part_id, 1);
    
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Bus %d: Lỗi đọc Part ID, kiểm tra dây cắm!", i2c_bus_num);
        return err; // Lỗi này nghiêm trọng, nên dừng lại
    }

    // --- SỬA LỖI BẮT ĐẦU TỪ ĐÂY ---
    if (part_id != 0x15) {
        // Chỉ cảnh báo (W), không phải lỗi (E)
        // Không 'return ESP_FAIL' nữa, vẫn tiếp tục chạy
        ESP_LOGW(TAG, "Bus %d: Part ID không chuẩn (0x%02X), lẽ ra là 0x15. Có thể là chip clone, tiếp tục...", i2c_bus_num, part_id);
    } else {
        ESP_LOGI(TAG, "Bus %d: Đã tìm thấy MAX30102 (ID: 0x%02X)", i2c_bus_num, part_id);
    }
    // --- KẾT THÚC SỬA LỖI ---

    // 2. Reset cảm biến
    ESP_LOGI(TAG, "Bus %d: Đang reset...", i2c_bus_num);
    max30102_write_reg(i2c_bus_num, MAX30102_MODE_CONFIG, 0x40);
    vTaskDelay(pdMS_TO_TICKS(100)); // Chờ reset

    // 3. Cấu hình FIFO (Lấy trung bình 4 mẫu, bật rollover)
    max30102_write_reg(i2c_bus_num, MAX30102_FIFO_CONFIG, 0x5F); 

    // 4. Cấu hình Mode (Chế độ SpO2: Red + IR)
    max30102_write_reg(i2c_bus_num, MAX30102_MODE_CONFIG, 0x03); 

    // 5. Cấu hình SpO2 (ADC 4096nA, 100 mẫu/giây, Bề rộng xung 411us)
    max30102_write_reg(i2c_bus_num, MAX30102_SPO2_CONFIG, 0x27);

    // 6. Cấu hình cường độ 2 LED (mức 0x24 ~7mA, là mức khởi đầu tốt)
    max30102_write_reg(i2c_bus_num, MAX30102_LED_PULSE_AMP_1, 0x24); // Red
    max30102_write_reg(i2c_bus_num, MAX30102_LED_PULSE_AMP_2, 0x24); // IR

    // 7. Xóa con trỏ FIFO
    max30102_write_reg(i2c_bus_num, MAX30102_FIFO_WR_PTR, 0x00);
    max30102_write_reg(i2c_bus_num, MAX30102_FIFO_RD_PTR, 0x00);

    ESP_LOGI(TAG, "Bus %d: MAX30102 đã khởi tạo xong.", i2c_bus_num);
    return ESP_OK;
}

// Hàm đọc FIFO (public)
esp_err_t max30102_read_fifo(i2c_port_t i2c_bus_num, max30102_data_t *data) {
    uint8_t read_ptr = 0;
    uint8_t write_ptr = 0;
    
    // Đọc con trỏ FIFO
    max30102_read_reg(i2c_bus_num, MAX30102_FIFO_WR_PTR, &write_ptr, 1);
    max30102_read_reg(i2c_bus_num, MAX30102_FIFO_RD_PTR, &read_ptr, 1);

    // Kiểm tra xem có dữ liệu mới không
    if (read_ptr == write_ptr) {
        return ESP_ERR_NOT_FOUND; // FIFO rỗng, không có dữ liệu
    }

    // Đọc 1 mẫu (6 bytes: 3 cho Red, 3 cho IR)
    uint8_t fifo_data[6];
    esp_err_t err = max30102_read_reg(i2c_bus_num, MAX30102_FIFO_DATA, fifo_data, 6);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Bus %d: Lỗi đọc FIFO data", i2c_bus_num);
        return err;
    }

    // Ghép 3 byte thành 1 số 32-bit (dữ liệu thực tế là 18-bit)
    data->red = ((uint32_t)fifo_data[0] << 16) | ((uint32_t)fifo_data[1] << 8) | fifo_data[2];
    data->ir  = ((uint32_t)fifo_data[3] << 16) | ((uint32_t)fifo_data[4] << 8) | fifo_data[5];

    // Lọc lấy 18 bit dữ liệu
    data->red &= 0x03FFFF;
    data->ir  &= 0x03FFFF;

    return ESP_OK;
}