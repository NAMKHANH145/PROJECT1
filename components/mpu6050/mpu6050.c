#include "mpu6050.h"
#include "esp_log.h"
#include "driver/i2c.h"

static const char *TAG = "MPU6050";

#define I2C_MASTER_ACK_EN   true    /*!< Enable ACK check for master */
#define I2C_MASTER_ACK_DIS  false   /*!< Disable ACK check for master */
#define ACK_VAL             0x00    /*!< I2C ack value */
#define NACK_VAL            0x01    /*!< I2C nack value */

// --- HÀM NỘI BỘ: Ghi 1 byte (Viết lại dùng cmd_link để ổn định hơn) ---
static esp_err_t mpu6050_write_reg(i2c_port_t i2c_bus_num, uint8_t reg, uint8_t value) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    // Gửi địa chỉ thiết bị + bit WRITE (0)
    i2c_master_write_byte(cmd, (MPU6050_I2C_ADDR << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK_EN);
    // Gửi địa chỉ thanh ghi
    i2c_master_write_byte(cmd, reg, I2C_MASTER_ACK_EN);
    // Gửi giá trị
    i2c_master_write_byte(cmd, value, I2C_MASTER_ACK_EN);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(i2c_bus_num, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

// --- HÀM NỘI BỘ: Đọc N byte (Viết lại dùng cmd_link) ---
static esp_err_t mpu6050_read_reg(i2c_port_t i2c_bus_num, uint8_t reg, uint8_t *data, size_t len) {
    if (len == 0) return ESP_OK;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    // 1. Gửi địa chỉ ghi để chọn thanh ghi
    i2c_master_write_byte(cmd, (MPU6050_I2C_ADDR << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK_EN);
    i2c_master_write_byte(cmd, reg, I2C_MASTER_ACK_EN);
    
    // 2. Restart để chuyển sang chế độ đọc
    i2c_master_start(cmd);
    // Gửi địa chỉ với bit READ (1)
    i2c_master_write_byte(cmd, (MPU6050_I2C_ADDR << 1) | I2C_MASTER_READ, I2C_MASTER_ACK_EN);
    
    // 3. Đọc dữ liệu
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, ACK_VAL); // ACK tất cả các byte trừ byte cuối
    }
    i2c_master_read_byte(cmd, data + len - 1, NACK_VAL); // NACK byte cuối cùng để báo kết thúc

    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(i2c_bus_num, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

// --- HÀM KHỞI TẠO (Đã thêm Reset) ---
esp_err_t mpu6050_init(i2c_port_t i2c_bus_num) {
    esp_err_t err;

    ESP_LOGI(TAG, "Đang khởi tạo MPU6050 trên Bus %d...", i2c_bus_num);

    // 0. RESET MPU6050 trước khi làm gì (Ghi bit 7 vào PWR_MGMT_1)
    // Giúp xóa các trạng thái treo nếu có
    mpu6050_write_reg(i2c_bus_num, MPU6050_PWR_MGMT_1, 0x80);
    vTaskDelay(pdMS_TO_TICKS(100)); // Đợi reset xong

    // 1. Kiểm tra ID thiết bị (WHO_AM_I)
    uint8_t who_am_i;
    err = mpu6050_read_reg(i2c_bus_num, MPU6050_WHO_AM_I, &who_am_i, 1);
    
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Bus %d: Lỗi giao tiếp I2C khi đọc WHO_AM_I. Kiểm tra dây nối (Pull-up)!", i2c_bus_num);
        return err;
    }

    if (who_am_i != 0x68) { 
        ESP_LOGW(TAG, "Bus %d: ID không khớp (0x%02X), nhưng vẫn tiếp tục khởi tạo...", i2c_bus_num, who_am_i);
    } else {
        ESP_LOGI(TAG, "Bus %d: Đã tìm thấy MPU6050 (ID: 0x%02X)", i2c_bus_num, who_am_i);
    }

    // 2. Đánh thức cảm biến (Ghi 0x00 vào PWR_MGMT_1)
    err = mpu6050_write_reg(i2c_bus_num, MPU6050_PWR_MGMT_1, 0x00);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Bus %d: Lỗi đánh thức MPU6050", i2c_bus_num);
        return err;
    }

    // 3. (Tùy chọn) Cấu hình bộ lọc DLPF để giảm nhiễu (DLPF_CFG = 3 -> ~44Hz bandwidth)
    // Giúp dữ liệu đọc ra ổn định hơn
    mpu6050_write_reg(i2c_bus_num, 0x1A, 0x03); 

    return ESP_OK;
}

// --- HÀM ĐỌC DỮ LIỆU ---
esp_err_t mpu6050_read_raw_data(i2c_port_t i2c_bus_num, mpu6050_raw_data_t *data) {
    uint8_t buffer[14]; 

    // Đọc 14 byte dữ liệu liên tục từ 0x3B
    esp_err_t err = mpu6050_read_reg(i2c_bus_num, MPU6050_ACCEL_XOUT_H, buffer, 14);
    if (err != ESP_OK) {
        // Không log lỗi quá nhiều ở đây để tránh spam console nếu đứt dây
        return err;
    }

    // Chuyển đổi Big-Endian sang int16
    data->accel_x = (int16_t)((buffer[0] << 8) | buffer[1]);
    data->accel_y = (int16_t)((buffer[2] << 8) | buffer[3]);
    data->accel_z = (int16_t)((buffer[4] << 8) | buffer[5]);
    
    data->gyro_x  = (int16_t)((buffer[8] << 8) | buffer[9]);
    data->gyro_y  = (int16_t)((buffer[10] << 8) | buffer[11]);
    data->gyro_z  = (int16_t)((buffer[12] << 8) | buffer[13]);

    return ESP_OK;
}