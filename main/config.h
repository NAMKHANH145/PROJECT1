#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "driver/gpio.h"
#include "driver/i2c.h"

/**
 * ============================================================
 * ESP32-S3 WEARABLE HEALTH MONITOR - SYSTEM CONFIGURATION
 * ============================================================
 */

// ============================================================
// I2C BUS CONFIGURATION (DUAL BUS ARCHITECTURE)
// ============================================================

// Bus 0: Dedicated to MAX30102 (High Speed Streaming)
#define I2C_BUS_PPG             I2C_NUM_0
// WARNING: GPIO 9/10 are default USB-JTAG pins. 
// Ensure "USB Serial/JTAG Controller" is DISABLED in menuconfig.
#define I2C_BUS_PPG_SDA         GPIO_NUM_10 
#define I2C_BUS_PPG_SCL         GPIO_NUM_9
#define I2C_PPG_FREQ_HZ         400000      // Increased to 400kHz for smoother PPG streaming

// Bus 1: Shared by MPU6050 + MAX30205
#define I2C_BUS_AUX             I2C_NUM_1
#define I2C_BUS_AUX_SDA         GPIO_NUM_11
#define I2C_BUS_AUX_SCL         GPIO_NUM_12
#define I2C_AUX_FREQ_HZ         100000      // 100kHz standard

// ============================================================
// SPI DISPLAY CONFIGURATION (ST7789 1.47")
// ============================================================

#define SPI_DISPLAY_HOST        SPI2_HOST
#define SPI_DISPLAY_MOSI_PIN    GPIO_NUM_6 
#define SPI_DISPLAY_SCK_PIN     GPIO_NUM_4 
#define SPI_DISPLAY_DC_PIN      GPIO_NUM_8 
#define SPI_DISPLAY_RST_PIN     GPIO_NUM_5 
#define SPI_DISPLAY_CS_PIN      GPIO_NUM_7 
#define SPI_DISPLAY_BLK_PIN     GPIO_NUM_13 // [CHANGED] Moved BLK to IO13 to free IO1 for Battery

// Battery Monitoring
#define BATTERY_ADC_PIN         GPIO_NUM_1  // ADC1_CH0

// Resolution (ST7789 1.47" 172x320)
#define DISPLAY_WIDTH           172
#define DISPLAY_HEIGHT          320

// --- BUTTONS ---
// GPIO 2: Next Screen | GPIO 3: Previous Screen
#define BUTTON_NEXT_PIN         GPIO_NUM_2 
#define BUTTON_PREV_PIN         GPIO_NUM_3 
#define BUTTON_DEBOUNCE_MS      250 // Tăng lên để chống rung tốt hơn

// ============================================================
// SENSOR SAMPLING CONFIGURATION
// ============================================================

#define PPG_SAMPLE_RATE_HZ      100          // MAX30102 sampling rate
#define IMU_SAMPLE_RATE_HZ      100          // MPU6050 sampling rate
#define WINDOW_SIZE_SAMPLES     1000         // 10 seconds @ 100Hz
#define WINDOW_SIZE_SECONDS     10

// Temperature sampling
#define TEMP_SAMPLE_INTERVAL_MS 5000         // 5 seconds

// ============================================================
// SYSTEM & FREERTOS CONFIG
// ============================================================

#define TWDT_TIMEOUT_S          10           // Watchdog Timeout

// Task Priorities (Higher = More Critical)
#define TASK_PRIORITY_BUTTON    6 
#define TASK_PRIORITY_SENSOR    5 
#define TASK_PRIORITY_PROCESSING 4 
#define TASK_PRIORITY_LVGL      10  // Highest priority to prevent screen tearing/stripes 
#define TASK_PRIORITY_TEMPERATURE 3 

// Stack Sizes (Bytes)
#define STACK_SIZE_SENSOR       4096 
#define STACK_SIZE_PROCESSING   16384        // Increased to 16KB for TFLite Arena overhead
#define STACK_SIZE_TEMPERATURE  4096 
#define STACK_SIZE_LVGL         8192 
#define STACK_SIZE_BUTTON       2048 

// Queue Sizes
#define QUEUE_SIZE_SENSOR_DATA      2 
#define QUEUE_SIZE_HEALTH_METRICS   5 
#define QUEUE_SIZE_TEMPERATURE      2 
#define QUEUE_SIZE_BUTTON_EVENTS    10 

// ============================================================
// SIGNAL PROCESSING PARAMETERS
// ============================================================

#define SPO2_DC_ALPHA           0.95f 
#define SPO2_VALID_RANGE_MIN    70 
#define SPO2_VALID_RANGE_MAX    100 

#define HR_VALID_RANGE_MIN      40 
#define HR_VALID_RANGE_MAX      200 

#define RR_VALID_RANGE_MIN      8 
#define RR_VALID_RANGE_MAX      30 

#define MOTION_THRESHOLD        2000 // Raw acceleration units

// ============================================================
// LVGL & UI CONFIGURATION
// ============================================================

#define LVGL_TICK_PERIOD_MS     20 
#define LVGL_TASK_DELAY_MS      20  // Refresh faster (50fps) for smoother scrolling

typedef enum {
    SCREEN_HOME = 0,
    SCREEN_RESPIRATION,
    SCREEN_STRESS,
    SCREEN_SETTINGS,
    SCREEN_COUNT
} ui_screen_t;

// ============================================================
// [ADDED] SHARED DATA TYPES (CRITICAL FOR COMPILATION)
// ============================================================

/**
 * @brief Structure holding all calculated health metrics.
 * Passed from Processing Task -> UI Task.
 */
typedef enum {
    STATUS_SCANNING = 0,
    STATUS_NO_FINGER,
    STATUS_CONNECTED,
    STATUS_MEASURING,
    STATUS_ERROR
} system_status_t;

/**
 * @brief Structure holding all calculated health metrics.
 * Passed from Processing Task -> UI Task.
 */
typedef struct {
    int heart_rate_bpm;         // BPM (e.g., 72)
    float confidence_hr;        // % Confidence
    int spo2_percent;           // % (e.g., 98), -1 if invalid
    float temperature_celsius;  // Deg C (e.g., 36.5)
    int respiration_rate;       // Breaths/min (e.g., 16)
    float confidence_rr;        // % Confidence
    int stress_level;           // 0 (Low) or 1 (High)
    float confidence_stress;    // % Confidence
    float hrv_sdnn;             // SDNN in ms
    bool motion_detected;       // True if user is moving too much
    bool data_valid;            // True if sensor connection is good
    system_status_t system_status; // Current system state
    int battery_percent;        // [NEW] Battery Percentage (0-100)
    bool low_battery;           // Low Battery Warning
    bool ai_error;              // AI Model Failure
    bool i2c_error;             // Sensor Connection Failure
} health_metrics_t;

/**
 * @brief Button events passed to UI Task
 */
typedef enum {
    BUTTON_EVENT_NEXT,      // Chuyển tiếp
    BUTTON_EVENT_PREV       // Quay lại
} button_event_t;

// ============================================================
// SYSTEM CONSTANTS
// ============================================================

#define SYSTEM_NAME             "HealthWatch"
#define FIRMWARE_VERSION        "1.0.0"