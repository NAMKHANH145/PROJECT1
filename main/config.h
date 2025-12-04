#pragma once

#include "driver/gpio.h"
#include "driver/i2c.h"

/**
 * ============================================================
 * ESP32-S3 WEARABLE HEALTH MONITOR - HARDWARE CONFIGURATION
 * ============================================================
 */

// ============================================================
// I2C BUS CONFIGURATION (DUAL BUS ARCHITECTURE)
// ============================================================

// Bus 0: Dedicated to MAX30102 (High Speed Streaming)
#define I2C_BUS_PPG             I2C_NUM_0
#define I2C_BUS_PPG_SDA         GPIO_NUM_10
#define I2C_BUS_PPG_SCL         GPIO_NUM_9
#define I2C_PPG_FREQ_HZ         100000      // 100kHz for stability

// Bus 1: Shared by MPU6050 + MAX30205
#define I2C_BUS_AUX             I2C_NUM_1
#define I2C_BUS_AUX_SDA         GPIO_NUM_11
#define I2C_BUS_AUX_SCL         GPIO_NUM_12
#define I2C_AUX_FREQ_HZ         100000      // 100kHz for stability  

// ============================================================
// SPI DISPLAY CONFIGURATION (ST7789 1.47")
// ============================================================

#define SPI_DISPLAY_HOST        SPI2_HOST
#define SPI_DISPLAY_MOSI_PIN    GPIO_NUM_6  // Super Mini
#define SPI_DISPLAY_SCK_PIN     GPIO_NUM_4  // Super Mini
#define SPI_DISPLAY_DC_PIN      GPIO_NUM_8  // Super Mini
#define SPI_DISPLAY_RST_PIN     GPIO_NUM_5  // Super Mini
#define SPI_DISPLAY_CS_PIN      GPIO_NUM_7  // Super Mini

// Display resolution (ST7789 1.47" 172x320)
#define DISPLAY_WIDTH           172
#define DISPLAY_HEIGHT          320

// ============================================================
// BUTTON INPUT CONFIGURATION
// ============================================================

#define BUTTON_NEXT_PIN         GPIO_NUM_2   // Button 1
#define BUTTON_PREV_PIN         GPIO_NUM_3   // Button 2
#define BUTTON_DEBOUNCE_MS      50

// ============================================================
// SENSOR SAMPLING CONFIGURATION
// ============================================================

#define PPG_SAMPLE_RATE_HZ      100          // MAX30102 sampling rate
#define IMU_SAMPLE_RATE_HZ      100          // MPU6050 sampling rate
#define WINDOW_SIZE_SAMPLES     1000         // 10 seconds @ 100Hz
#define WINDOW_SIZE_SECONDS     10

// Temperature sampling (slower, non-critical)
#define TEMP_SAMPLE_INTERVAL_MS 5000         // 5 seconds

// ============================================================
// FREERTOS WATCHDOG (CƠ CHẾ BẢO VỆ)
// ============================================================

// Thời gian chờ tối đa trước khi reset chip (tăng lên 10s cho môi trường test nhiễu)
#define TWDT_TIMEOUT_S          10      

// ============================================================
// SENSOR SAMPLING CONFIGURATION
// ============================================================

#define PPG_SAMPLE_RATE_HZ      100          // Tốc độ lấy mẫu: 100 mẫu/giây
#define WINDOW_SIZE_SAMPLES     1000         // Cửa sổ xử lý: 1000 mẫu (10 giây)
#define WINDOW_SIZE_SECONDS     10           // Độ dài cửa sổ tính bằng giây

// ============================================================
// FREERTOS TASK PRIORITIES (ĐỘ ƯU TIÊN)
// ============================================================
// Giá trị càng cao -> Ưu tiên càng lớn. Task quan trọng sẽ chiếm CPU trước.

#define TASK_PRIORITY_BUTTON            6    // Cao nhất: Phản hồi nút nhấn tức thì
#define TASK_PRIORITY_SENSOR            5    // Rất cao: Đọc cảm biến phải đều đặn
#define TASK_PRIORITY_PROCESSING        4    // Cao: Xử lý AI nặng
#define TASK_PRIORITY_LVGL              3    // Trung bình: Vẽ màn hình (20FPS là đủ)
#define TASK_PRIORITY_TEMPERATURE       3    // Thấp: Nhiệt độ thay đổi chậm

// ============================================================
// FREERTOS STACK SIZES (BỘ NHỚ NGĂN XẾP)
// ============================================================
// Tăng dư dả cho môi trường Test để tránh Stack Overflow

#define STACK_SIZE_SENSOR           4096     // 4KB
#define STACK_SIZE_PROCESSING       12288    // 12KB (Vì chạy AI và FFT cần nhiều RAM cục bộ)
#define STACK_SIZE_TEMPERATURE      2048     // 2KB
#define STACK_SIZE_LVGL             8192     // 8KB (Driver màn hình màu cần nhiều stack)
#define STACK_SIZE_BUTTON           2048     // 2KB

// ============================================================
// FREERTOS QUEUE SIZES
// ============================================================

#define QUEUE_SIZE_SENSOR_DATA      2        // 2 x 10-second windows
#define QUEUE_SIZE_HEALTH_METRICS   5        // 5 metric updates
#define QUEUE_SIZE_TEMPERATURE      2        // 2 temperature readings
#define QUEUE_SIZE_BUTTON_EVENTS    10       // 10 button events

// ============================================================
// SIGNAL PROCESSING PARAMETERS
// ============================================================

// SpO2 Calculation
#define SPO2_DC_ALPHA               0.95f    // DC component low-pass filter
#define SPO2_VALID_RANGE_MIN        70       // Minimum valid SpO2 %
#define SPO2_VALID_RANGE_MAX        100      // Maximum valid SpO2 %

// Heart Rate
#define HR_VALID_RANGE_MIN          40       // Minimum valid HR (BPM)
#define HR_VALID_RANGE_MAX          200      // Maximum valid HR (BPM)

// Respiration Rate
#define RR_VALID_RANGE_MIN          8        // Minimum valid RR (breaths/min)
#define RR_VALID_RANGE_MAX          30       // Maximum valid RR (breaths/min)

// Motion Detection (MPU6050)
#define MOTION_THRESHOLD            2000     // Acceleration threshold for artifact detection

// ============================================================
// LVGL CONFIGURATION
// ============================================================

#define LVGL_TICK_PERIOD_MS         10       // LVGL tick period
#define LVGL_TASK_DELAY_MS          50       // 20Hz UI update rate

// ============================================================
// UI SCREEN IDS
// ============================================================

typedef enum {
    SCREEN_HOME = 0,        // HR, SpO2, Temperature
    SCREEN_RESPIRATION,     // Respiration rate + waveform
    SCREEN_STRESS,          // Stress level + HRV
    SCREEN_SETTINGS,        // System settings
    SCREEN_COUNT            // Total number of screens
} ui_screen_t;

// ============================================================
// SYSTEM CONSTANTS
// ============================================================

#define SYSTEM_NAME             "HealthWatch"
#define FIRMWARE_VERSION        "1.0.0"
