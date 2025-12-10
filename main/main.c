/**
 * ============================================================ 
 * ESP32-S3 WEARABLE HEALTH MONITORING SYSTEM
 * ============================================================ 
 *
 * Multi-sensor health monitoring device with TinyML inference
 * ENGINEERED FOR STABILITY - MEMORY POOL & HARDWARE TIMER EDITION
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

// FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h" 

// ESP-IDF
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"     
#include "esp_task_wdt.h"       
#include "esp_pm.h"         
#include "dsps_fft2r.h"     
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_psram.h" 
#include "lvgl.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_adc/adc_oneshot.h" // [NEW] ADC Driver

// Project Headers
#include "config.h"
#include "max30102.h"
#include "mpu6050.h"
#include "max30205.h"
#include "algorithms.h"     
#include "signal_filters.h" 
#include "tinyml_manager.h" 
#include "ui.h"
#include "ble_server.h" // [NEW] BLE Server
#include "nvs_flash.h"  // Required for BLE

static const char *TAG = "MAIN";

// ============================================================ 
// 1. DATA STRUCTURES & CONFIG
// ============================================================ 

// Cấu trúc gói dữ liệu cảm biến thô
typedef struct {
    uint32_t red_buffer[WINDOW_SIZE_SAMPLES];
    uint32_t ir_buffer[WINDOW_SIZE_SAMPLES];
    int16_t accel_x[WINDOW_SIZE_SAMPLES];
    int16_t accel_y[WINDOW_SIZE_SAMPLES];
    int16_t accel_z[WINDOW_SIZE_SAMPLES];
    uint32_t timestamp_ms;
    bool data_valid;
    bool i2c_error;
} sensor_data_package_t;

// Số lượng buffer trong Memory Pool
#define POOL_SIZE  3 
#define SENSOR_TIMER_FREQ_HZ 100 // Tần số lấy mẫu 100Hz

// ============================================================ 
// 2. GLOBAL HANDLES
// ============================================================ 

// Queues
static QueueHandle_t sensor_data_queue = NULL;      
static QueueHandle_t free_buf_queue = NULL;         
static QueueHandle_t health_metrics_queue = NULL;
static QueueHandle_t temperature_queue = NULL;
// Lưu ý: button_event_queue không dùng trong Polling Mode, ta gọi thẳng UI

// Mutex cho I2C
static SemaphoreHandle_t i2c_ppg_mutex = NULL;
static SemaphoreHandle_t i2c_aux_mutex = NULL;

// Task Handles
static TaskHandle_t task_handle_sensor = NULL;
static TaskHandle_t task_handle_processing = NULL;
static TaskHandle_t task_handle_temperature = NULL;
static TaskHandle_t task_handle_lvgl = NULL;
static TaskHandle_t task_handle_button = NULL;

// Global Raw Buffers (Dùng để tích lũy mẫu trước khi đóng gói)
static uint32_t *g_red_buffer = NULL;
static uint32_t *g_ir_buffer = NULL;
static int16_t *g_accel_x = NULL;
static int16_t *g_accel_y = NULL;
static int16_t *g_accel_z = NULL;

// Power Management
static esp_pm_lock_handle_t s_cpu_lock = NULL;
static volatile int64_t g_last_activity_time = 0; // Timestamp of last user interaction
static bool g_is_sleeping = false; // System Sleep State
volatile int g_current_battery = 0; // [NEW] Shared Battery Value
volatile float g_last_temp = 0.0f; // [NEW] Shared Temperature Value

// ============================================================ 
// 3. HARDWARE TIMER CALLBACK (ISR)
// ============================================================ 
static bool IRAM_ATTR on_timer_alarm_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx) {
    BaseType_t high_task_awoken = pdFALSE;
    TaskHandle_t task_to_notify = (TaskHandle_t)user_ctx;
    // Đánh thức Task Sensor dậy để đọc dữ liệu
    vTaskNotifyGiveFromISR(task_to_notify, &high_task_awoken);
    return (high_task_awoken == pdTRUE);
}

// ============================================================ 
// 4. POWER MANAGEMENT (Moved Up)
// ============================================================ 
static void power_manager_init(void) {
    esp_pm_lock_create(ESP_PM_CPU_FREQ_MAX, 0, "cpu_max", &s_cpu_lock);
}

static void set_power_mode(bool active) {
    if (s_cpu_lock) {
        if (active) esp_pm_lock_acquire(s_cpu_lock);
        else esp_pm_lock_release(s_cpu_lock);
    }
}

// ============================================================ 
// 5. I2C INITIALIZATION
// ============================================================ 
static void i2c_setup_buses(void) {
    ESP_LOGI(TAG, "Initializing Dual I2C Buses...");

    // Bus 0: PPG (Fast Speed)
    i2c_config_t conf_ppg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_BUS_PPG_SDA,
        .scl_io_num = I2C_BUS_PPG_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_PPG_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_BUS_PPG, &conf_ppg));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_BUS_PPG, conf_ppg.mode, 0, 0, 0));

    // Bus 1: AUX (Standard Speed)
    i2c_config_t conf_aux = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_BUS_AUX_SDA,
        .scl_io_num = I2C_BUS_AUX_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_AUX_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_BUS_AUX, &conf_aux));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_BUS_AUX, conf_aux.mode, 0, 0, 0));

    ESP_LOGI(TAG, "Dual I2C Initialized.");
}

// ============================================================ 
// 5. BUTTON HANDLING (POLLING MODE - AN TOÀN NHẤT)
// ============================================================ 

static void button_gpio_init(void) {
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE, // Tắt ngắt để tránh xung đột
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << BUTTON_NEXT_PIN) | (1ULL << BUTTON_PREV_PIN),
        .pull_up_en = GPIO_PULLUP_ENABLE, // Kéo lên nguồn 3.3V (Bấm nút sẽ kéo xuống GND)
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    
    ESP_LOGI(TAG, "Buttons Init (Polling): NEXT(IO%d), PREV(IO%d)", BUTTON_NEXT_PIN, BUTTON_PREV_PIN);
}

static void button_task(void *pvParameters) {
    int last_next = 1;
    int last_prev = 1;

    ESP_LOGI(TAG, "Button Task Started");

    while (1) {
        // Đọc mức logic (1 = Nhả, 0 = Nhấn)
        int curr_next = gpio_get_level(BUTTON_NEXT_PIN);
        int curr_prev = gpio_get_level(BUTTON_PREV_PIN);

        // Phát hiện cạnh xuống (1 -> 0) của nút NEXT
        if (last_next == 1 && curr_next == 0) {
            ESP_LOGI(TAG, ">>> Button NEXT Pressed! <<<");
            g_last_activity_time = esp_timer_get_time(); // Reset Sleep Timer
            ui_wake_up(); // Sáng màn hình
            ui_handle_button(BUTTON_EVENT_NEXT); // Chuyển màn hình
            vTaskDelay(pdMS_TO_TICKS(250)); // Chống rung (Debounce)
        }

        // Phát hiện cạnh xuống (1 -> 0) của nút PREV
        if (last_prev == 1 && curr_prev == 0) {
            ESP_LOGI(TAG, ">>> Button PREV Pressed! <<<");
            g_last_activity_time = esp_timer_get_time(); // Reset Sleep Timer
            ui_wake_up();
            ui_handle_button(BUTTON_EVENT_PREV);
            vTaskDelay(pdMS_TO_TICKS(250)); // Chống rung
        }

        last_next = curr_next;
        last_prev = curr_prev;

        vTaskDelay(pdMS_TO_TICKS(50)); // Check mỗi 50ms
    }
}

// ============================================================ 
// 6. SENSOR ACQUISITION TASK (ĐỌC CẢM BIẾN)
// ============================================================ 
static void sensor_acquisition_task(void *pvParameters) {
    ESP_LOGI(TAG, "Sensor Task Started");
    esp_task_wdt_add(NULL); // Đăng ký Watchdog

    // Cấu hình Hardware Timer để đọc mẫu chính xác 100Hz
    gptimer_handle_t gptimer = NULL;
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, 
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));

    gptimer_alarm_config_t alarm_config = {
        .alarm_count = 1000000 / SENSOR_TIMER_FREQ_HZ, 
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config));

    gptimer_event_callbacks_t cbs = { .on_alarm = on_timer_alarm_cb };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, (void*)xTaskGetCurrentTaskHandle()));
    ESP_ERROR_CHECK(gptimer_enable(gptimer));
    ESP_ERROR_CHECK(gptimer_start(gptimer));

    int sample_count = 0;
    sensor_data_package_t *current_pkg = NULL;
    bool pkg_i2c_error = false;
    int consecutive_i2c_errors = 0;

    while (1) {
        // Chờ Timer báo hiệu (mỗi 10ms)
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20));
        esp_task_wdt_reset();

        bool ppg_ok = false;
        // Đọc MAX30102
        if (xSemaphoreTake(i2c_ppg_mutex, pdMS_TO_TICKS(15)) == pdTRUE) {
            max30102_data_t ppg_data;
            esp_err_t ret = max30102_read_fifo(I2C_BUS_PPG, &ppg_data);
            if (ret == ESP_OK) {
                g_red_buffer[sample_count] = ppg_data.red;
                g_ir_buffer[sample_count] = ppg_data.ir;
                ppg_ok = true;
                
                // [WAKE UP CHECK] Finger Detected?
                if (ppg_data.ir > 50000) { 
                    g_last_activity_time = esp_timer_get_time();
                }

                consecutive_i2c_errors = 0; // Reset error counter
            } else if (ret != ESP_ERR_NOT_FOUND) {
                pkg_i2c_error = true;
                consecutive_i2c_errors++;
            }
            xSemaphoreGive(i2c_ppg_mutex);
        }
        
        // Attempt Recovery if too many errors
        if (consecutive_i2c_errors >= 50) { // ~0.5 seconds of failures
            ESP_LOGE(TAG, "MAX30102 Critical Failure! Attempting Reset...");
            if (xSemaphoreTake(i2c_ppg_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                max30102_init(I2C_BUS_PPG);
                max30102_boost_leds(I2C_BUS_PPG);
                xSemaphoreGive(i2c_ppg_mutex);
                consecutive_i2c_errors = 0;
            }
        }
        
        if (!ppg_ok) { // Nếu lỗi đọc, điền 0
             g_red_buffer[sample_count] = 0;
             g_ir_buffer[sample_count]  = 0;
        }

        // Đọc MPU6050
        if (xSemaphoreTake(i2c_aux_mutex, pdMS_TO_TICKS(15)) == pdTRUE) {
            mpu6050_raw_data_t imu_data;
            if (mpu6050_read_raw_data(I2C_BUS_AUX, &imu_data) == ESP_OK) {
                g_accel_x[sample_count] = imu_data.accel_x;
                g_accel_y[sample_count] = imu_data.accel_y;
                g_accel_z[sample_count] = imu_data.accel_z;

                // [WAKE UP CHECK] Motion Detected? (Threshold ~1.5G or sudden jerk)
                if (abs(imu_data.accel_x) > 15000 || abs(imu_data.accel_y) > 15000) {
                     g_last_activity_time = esp_timer_get_time();
                }

            } else {
                // Giữ giá trị cũ nếu lỗi
                 g_accel_x[sample_count] = (sample_count > 0) ? g_accel_x[sample_count-1] : 0;
                 g_accel_y[sample_count] = (sample_count > 0) ? g_accel_y[sample_count-1] : 0;
                 g_accel_z[sample_count] = (sample_count > 0) ? g_accel_z[sample_count-1] : 0;
            }
            xSemaphoreGive(i2c_aux_mutex);
        }

        // --- SLEEP MODE LOGIC ---
        int64_t now = esp_timer_get_time();
        if ((now - g_last_activity_time) > 10000000) { // 10 seconds timeout
            if (!g_is_sleeping) {
                ESP_LOGI(TAG, "Entering Sleep Mode (No Activity)...");
                ui_enter_sleep(); // Turn off screen
                set_power_mode(false); // Reduce CPU clock
                g_is_sleeping = true;
            }
        } else {
            if (g_is_sleeping) {
                ESP_LOGI(TAG, "Waking Up!");
                ui_wake_up(); // Turn on screen
                set_power_mode(true); // Boost CPU
                g_is_sleeping = false;
            }
        }

        sample_count++;

        // Khi đủ mẫu (WINDOW_SIZE), đóng gói gửi sang AI
        // [MODIFICATION] Only send data if NOT sleeping
        if (sample_count >= WINDOW_SIZE_SAMPLES && !g_is_sleeping) {
            if (xQueueReceive(free_buf_queue, &current_pkg, 0) == pdTRUE) {
                memcpy(current_pkg->red_buffer, g_red_buffer, sizeof(uint32_t) * WINDOW_SIZE_SAMPLES);
                memcpy(current_pkg->ir_buffer, g_ir_buffer, sizeof(uint32_t) * WINDOW_SIZE_SAMPLES);
                memcpy(current_pkg->accel_x, g_accel_x, sizeof(int16_t) * WINDOW_SIZE_SAMPLES);
                memcpy(current_pkg->accel_y, g_accel_y, sizeof(int16_t) * WINDOW_SIZE_SAMPLES);
                memcpy(current_pkg->accel_z, g_accel_z, sizeof(int16_t) * WINDOW_SIZE_SAMPLES);
                
                current_pkg->timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000);
                current_pkg->data_valid = true;
                current_pkg->i2c_error = pkg_i2c_error;

                if (xQueueSend(sensor_data_queue, &current_pkg, 0) != pdTRUE) {
                    ESP_LOGW(TAG, "Queue Full! Dropping Frame.");
                    xQueueSend(free_buf_queue, &current_pkg, 0); // Trả lại buffer
                }
            } else {
                ESP_LOGW(TAG, "Pool Empty! Frame Dropped.");
            }
            
            // --- SLIDING WINDOW IMPLEMENTATION (Overlapping) ---
            // Thay vì reset về 0 (chờ 10s), ta dịch chuyển dữ liệu cũ sang trái
            // Giữ lại 800 mẫu cũ (8s), chừa chỗ cho 200 mẫu mới (2s)
            // => Latency giảm xuống còn 2s mà vẫn có đủ 10s dữ liệu cho AI
            int shift_amount = 200; // 2 seconds @ 100Hz
            int keep_amount = WINDOW_SIZE_SAMPLES - shift_amount;

            // Shift buffers left
            memmove(g_red_buffer, &g_red_buffer[shift_amount], keep_amount * sizeof(uint32_t));
            memmove(g_ir_buffer, &g_ir_buffer[shift_amount], keep_amount * sizeof(uint32_t));
            memmove(g_accel_x, &g_accel_x[shift_amount], keep_amount * sizeof(int16_t));
            memmove(g_accel_y, &g_accel_y[shift_amount], keep_amount * sizeof(int16_t));
            memmove(g_accel_z, &g_accel_z[shift_amount], keep_amount * sizeof(int16_t));

            sample_count = keep_amount; // Bắt đầu điền tiếp từ vị trí 800
            pkg_i2c_error = false; // Reset error flag
        } else if (sample_count >= WINDOW_SIZE_SAMPLES && g_is_sleeping) {
             // If sleeping, just shift the buffer to keep checking for wake-up events
             // but DO NOT send data to AI.
            int shift_amount = 200; 
            int keep_amount = WINDOW_SIZE_SAMPLES - shift_amount;
            
            memmove(g_red_buffer, &g_red_buffer[shift_amount], keep_amount * sizeof(uint32_t));
            memmove(g_ir_buffer, &g_ir_buffer[shift_amount], keep_amount * sizeof(uint32_t));
            memmove(g_accel_x, &g_accel_x[shift_amount], keep_amount * sizeof(int16_t));
            memmove(g_accel_y, &g_accel_y[shift_amount], keep_amount * sizeof(int16_t));
            memmove(g_accel_z, &g_accel_z[shift_amount], keep_amount * sizeof(int16_t));

            sample_count = keep_amount;
        }
    }
}

// ============================================================ 
// 8. SIGNAL PROCESSING & AI TASK
// ============================================================ 
static void signal_processing_task(void *pvParameters) {
    sensor_data_package_t *sensor_pkg;
    
    // Ngưỡng chuyển động
  //  const float MOTION_STATIONARY = 200.0f;
    const float MOTION_SHAKE = 3000.0f;
    
    // Cấp phát bộ nhớ cho DSP (Ưu tiên RAM nội để nhanh)
    uint32_t caps = MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT; 
    
    float *ppg_float = heap_caps_malloc(sizeof(float) * WINDOW_SIZE_SAMPLES, caps);
    float *rr_intervals = heap_caps_malloc(sizeof(float) * 30, caps);
    #define RR_HISTORY_SIZE 32
    float *rr_history = heap_caps_malloc(sizeof(float) * RR_HISTORY_SIZE, caps);
    
    // Kiểm tra cấp phát
    if (!ppg_float || !rr_intervals || !rr_history) {
        ESP_LOGE(TAG, "CRITICAL: Proc Buffers Alloc Failed!"); 
        vTaskSuspend(NULL);
    }
    
    // Init Filters
    ppg_filter_t ppg_filter; ppg_filter_init(&ppg_filter);
    kalman_filter_t kf_ax, kf_ay, kf_az;
    kalman_filter_init(&kf_ax, 0.01f, 0.1f, 0.0f); 
    kalman_filter_init(&kf_ay, 0.01f, 0.1f, 0.0f); 
    kalman_filter_init(&kf_az, 0.01f, 0.1f, 0.0f);
    lms_filter_t lms_filter; lms_filter_init(&lms_filter, 0.1f); 
    
    // power_manager_init(); // Already called in app_main
    bool is_power_active = false;
   

    while (1) {
        // Chờ gói dữ liệu từ Sensor Task
        if (xQueueReceive(sensor_data_queue, &sensor_pkg, portMAX_DELAY) == pdTRUE) {
            
            // Tăng xung nhịp CPU để xử lý AI nhanh
            if (!is_power_active) { set_power_mode(true); is_power_active = true; }

            // [DEBUG DIAGNOSTICS]
            ESP_LOGI(TAG, "================ [ PROCESSING START ] ================");
            ESP_LOGI(TAG, "Buffer Address: %p", sensor_pkg);
            ESP_LOGI(TAG, "Memory Location: %s", esp_ptr_in_dram(sensor_pkg) ? "Internal RAM" : (esp_ptr_external_ram(sensor_pkg) ? "PSRAM" : "Unknown"));
            ESP_LOGI(TAG, "Free Heap (Internal): %zu bytes", heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
            ESP_LOGI(TAG, "Free Heap (PSRAM): %zu bytes", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
            
            // Check raw signal stats
            long long ir_sum = 0;
            for(int i=0; i<WINDOW_SIZE_SAMPLES; i++) ir_sum += sensor_pkg->ir_buffer[i];
            float ir_mean = (float)ir_sum / WINDOW_SIZE_SAMPLES;
            ESP_LOGI(TAG, "Raw IR Mean: %.2f (Finger Threshold: 50000.0)", ir_mean);

            // 1. Lọc tín hiệu & Khử nhiễu
            float acc_var = 0;
            for(int i=0; i<WINDOW_SIZE_SAMPLES; i++) {
                // Kalman cho IMU
                sensor_pkg->accel_x[i] = (int16_t)kalman_filter_update(&kf_ax, (float)sensor_pkg->accel_x[i]);
                sensor_pkg->accel_y[i] = (int16_t)kalman_filter_update(&kf_ay, (float)sensor_pkg->accel_y[i]);
                sensor_pkg->accel_z[i] = (int16_t)kalman_filter_update(&kf_az, (float)sensor_pkg->accel_z[i]);
                
                // Tính độ biến thiên để phát hiện chuyển động
                float diff = (float)sensor_pkg->accel_z[i] - (float)sensor_pkg->accel_z[0];
                acc_var += diff * diff;

                // Lọc PPG
                float bp_out = ppg_filter_process(&ppg_filter, (float)sensor_pkg->ir_buffer[i]);
                float noise_ref = (float)sensor_pkg->accel_z[i] / 16384.0f;
                static float avg_acc = 0; avg_acc = 0.95f * avg_acc + 0.05f * noise_ref; noise_ref -= avg_acc;
                ppg_float[i] = lms_filter_update(&lms_filter, bp_out, noise_ref);
            }
            acc_var /= WINDOW_SIZE_SAMPLES;
            float motion_score = sqrtf(acc_var);
            ESP_LOGI(TAG, "Motion Score: %.2f (Shake Thresh: %.2f)", motion_score, MOTION_SHAKE);

            // [AUTO-CORRECTION] Check for Inverted Signal (Negative Skewness)
            // PPG peaks should be positive. If sensor is upside down or scattering changes, it might be inverted.
            float mean_val = calculate_mean(ppg_float, WINDOW_SIZE_SAMPLES);
            float std_val = calculate_std(ppg_float, WINDOW_SIZE_SAMPLES);
            double sum_skew = 0;
            if (std_val > 0.0001f) {
                for(int i=0; i<WINDOW_SIZE_SAMPLES; i++) {
                    float z = (ppg_float[i] - mean_val) / std_val;
                    sum_skew += z*z*z;
                }
                float skewness = (float)(sum_skew / WINDOW_SIZE_SAMPLES);
                ESP_LOGI(TAG, "Signal Skewness: %.2f", skewness);
                
                if (skewness < -0.5f) { // Strong negative skew -> Invert signal
                    ESP_LOGW(TAG, "Signal Inverted -> Correcting...");
                    for(int i=0; i<WINDOW_SIZE_SAMPLES; i++) ppg_float[i] = -ppg_float[i];
                }
            }

            // 2. Chạy TinyML Models
            float sqi = calculate_signal_quality_index(ppg_float, WINDOW_SIZE_SAMPLES);
            z_score_normalize(ppg_float, WINDOW_SIZE_SAMPLES);
            
            ESP_LOGI(TAG, "Signal Quality Index (SQI): %.2f", sqi);

            int hr_bpm = tinyml_predict_bpm(ppg_float);
            int rr_bpm = tinyml_predict_respiration(ppg_float);
            int stress_prob = tinyml_predict_stress(ppg_float, sensor_pkg->accel_x, sensor_pkg->accel_y, sensor_pkg->accel_z);
            int stress_class = (stress_prob > 50) ? 1 : 0;
            
            ESP_LOGI(TAG, ">>> AI RESULTS >>> HR: %d bpm, RR: %d brpm, Stress: %d%% (%s)", 
                     hr_bpm, rr_bpm, stress_prob, stress_class ? "HIGH" : "LOW");

            // 3. Tính toán SpO2 (Thuật toán cổ điển)
            int spo2 = calculate_spo2(sensor_pkg->red_buffer, sensor_pkg->ir_buffer, WINDOW_SIZE_SAMPLES);
            ESP_LOGI(TAG, ">>> ALG RESULT >>> SpO2: %d%%", spo2);

            // 4. Gửi kết quả sang UI
            health_metrics_t metrics = {0}; // Init to zero to avoid garbage
            metrics.heart_rate_bpm = hr_bpm;
            metrics.spo2_percent = spo2;
            metrics.respiration_rate = rr_bpm;
            metrics.stress_level = stress_class;
            
            // Check Finger Presence (IR Threshold)
            bool is_finger = (ir_mean > 50000.0f); // Threshold for finger detection
            if (!is_finger) ESP_LOGW(TAG, "Status: NO FINGER DETECTED (IR < 50000)");

            // Tăng ngưỡng SQI để loại bỏ nhiễu (Randomness fix)
            // motion_score thấp = ít chuyển động. motion_score cao = chuyển động.
            bool is_stable = (motion_score < MOTION_SHAKE);
            metrics.motion_detected = !is_stable;
            
            // [DEBUG] Tạm thời bỏ check SQI (> 0.0f) để dữ liệu đi qua dễ dàng
            metrics.data_valid = (spo2 != -1 && sqi > 10.0f); // Lenient threshold
            if (metrics.data_valid) ESP_LOGI(TAG, "Status: DATA VALID");
            else ESP_LOGW(TAG, "Status: DATA INVALID (Low SQI or SpO2 fail)");
            
            // --- NEW METRICS POPULATION ---
            metrics.i2c_error = sensor_pkg->i2c_error;
            metrics.ai_error = false; 
            metrics.low_battery = (g_current_battery < 20); 
            metrics.battery_percent = g_current_battery;
            
            // Confidence Estimation
            metrics.confidence_hr = sqi; 
            metrics.confidence_rr = (sqi > 20.0f) ? (sqi - 10.0f) : sqi/2; 
            if(metrics.confidence_rr > 100) metrics.confidence_rr = 100;
            metrics.confidence_stress = (float)stress_prob;

            // [NEW] Update BLE
            ble_update_data(hr_bpm, spo2, (int)g_last_temp, g_current_battery);

            // System Status Logic (Deterministic)
            if (metrics.i2c_error) {
                metrics.system_status = STATUS_ERROR;
            } else if (!is_finger) {
                metrics.system_status = STATUS_NO_FINGER;
                metrics.data_valid = false; // Force invalid if no finger
            } else if (!metrics.data_valid) {
                // Finger present but signal weak/noisy
                metrics.system_status = STATUS_SCANNING;
            } else {
                // Data Valid
                if (metrics.motion_detected) {
                    metrics.system_status = STATUS_MEASURING; // Đang đo nhưng rung
                } else {
                    metrics.system_status = STATUS_CONNECTED; // Ổn định
                }
            }
            
            ESP_LOGI(TAG, "Sending Packet to UI Queue...");
            ESP_LOGI(TAG, "================ [ PROCESSING END ] ================");

            xQueueSend(health_metrics_queue, &metrics, 0);
            
            // Trả buffer về Pool
            xQueueSend(free_buf_queue, &sensor_pkg, 0);
        }
    }
}

// ============================================================ 
// 9. SLOW MAINTENANCE TASK (Temp + Battery)
// ============================================================ 
static adc_oneshot_unit_handle_t adc1_handle = NULL;

static void init_adc(void) {
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12, // Measure up to ~3.1V (with divider -> 6.2V max)
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_0, &config)); // GPIO 1 is ADC1_CH0
}

static int read_battery_percentage(void) {
    if (!adc1_handle) return 0;
    int raw = 0;
    if (adc_oneshot_read(adc1_handle, ADC_CHANNEL_0, &raw) == ESP_OK) {
        // Raw (12-bit) -> Voltage
        // V_pin = raw * 3.3 / 4095
        // V_bat = V_pin * 2 (Divider)
        float v_bat = (raw * 3.3f / 4095.0f) * 2.0f;
        
        // Simple Linear Mapping for LiPo (3.0V - 4.2V)
        int pct = (int)((v_bat - 3.0f) / (4.2f - 3.0f) * 100.0f);
        if (pct > 100) pct = 100;
        if (pct < 0) pct = 0;
        return pct;
    }
    return 0;
}

static void slow_maintenance_task(void *pvParameters) {
    temp_filter_t temp_filter;
    temp_filter_init(&temp_filter);
    
    while (1) {
        // 1. Temperature
        float temperature = 0.0f;
        esp_err_t ret = ESP_FAIL;
        
        if (xSemaphoreTake(i2c_aux_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            ret = max30205_read_temperature(I2C_BUS_AUX, &temperature);
            if (ret == ESP_OK) {
                float smoothed = temp_filter_process(&temp_filter, temperature);
                g_last_temp = smoothed; // Update global
                // Send Temp Update (reuse float queue for simple values)
                xQueueSend(temperature_queue, &smoothed, 0); 
                ESP_LOGI(TAG, "Temp: %.2f C", smoothed);
            }
            xSemaphoreGive(i2c_aux_mutex);
        }

        // 2. Battery
        int bat = read_battery_percentage();
        // We need a way to send Battery to UI. 
        // Hack: Send a negative value to temperature_queue to indicate battery? 
        // Better: Just update a global or rely on the metrics queue update cycle.
        // For stability, let's just log it here and let UI pick it up via a global variable hack 
        // OR better: Update the health_metrics_t queue with a "Partial Update" flag.
        
        // SIMPLEST APPROACH: Update a global atomic variable that the main loop picks up
        // But for now, let's just print it. To display it, we need to pass it to UI.
        // Let's modify the UI Task to handle a special "Battery Update" logic or just
        // include it in the next sensor packet?
        // Let's use a static global for battery that UI reads directly or passed via metrics.
        // Actually, let's just make 'bat' available to the sensor task or UI task via a shared volatile.
        
        extern volatile int g_current_battery; // Forward declaration
        g_current_battery = bat;

        ESP_LOGI(TAG, "Battery: %d%%", bat);

        vTaskDelay(pdMS_TO_TICKS(TEMP_SAMPLE_INTERVAL_MS));
    }
}

// ============================================================ 
// 10. UI TASK (LVGL)
// ============================================================ 
static void lvgl_task(void *pvParameters) {
    health_metrics_t metrics = {0};
    float temperature = 0.0f;
    float last_known_temp = 0.0f;

    while (1) {
        ui_tick_handler();
        
        // Nhận dữ liệu từ Queue và cập nhật UI
        if (xQueueReceive(health_metrics_queue, &metrics, 0) == pdTRUE) {
            ESP_LOGI(TAG, "[UI] Received Metrics! HR: %d, Status: %d", metrics.heart_rate_bpm, metrics.system_status);
            metrics.temperature_celsius = last_known_temp; // Restore last valid temp
            ui_update_metrics(&metrics);
        }
        if (xQueueReceive(temperature_queue, &temperature, 0) == pdTRUE) {
            // Cập nhật riêng nhiệt độ (vì nó chậm hơn các chỉ số khác)
            last_known_temp = temperature;
            health_metrics_t t = metrics; 
            t.temperature_celsius = temperature;
            ui_update_metrics(&t);
        }
        
        ui_task_handler(); // LVGL render
        vTaskDelay(pdMS_TO_TICKS(LVGL_TASK_DELAY_MS));
    }
}

// ============================================================
// 11. DEBUG INFO
// ============================================================
void print_debug_info(void) {
    ESP_LOGW(TAG, "Waiting 3 seconds for Serial Monitor...");
    vTaskDelay(pdMS_TO_TICKS(3000));

    ESP_LOGI(TAG, "=== SYSTEM DIAGNOSTICS ===");
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    ESP_LOGI(TAG, "Model: %s (Cores: %d)", (chip_info.model == CHIP_ESP32S3) ? "ESP32-S3" : "Unknown", chip_info.cores);
    
    size_t free_internal = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    ESP_LOGI(TAG, "Free Internal RAM: %d KB", free_internal/1024);
    ESP_LOGI(TAG, "==========================");
}

// ============================================================ 
// 12. APP MAIN
// ============================================================ 
void app_main(void) {
    print_debug_info(); 
    ESP_LOGI(TAG, "Booting Health Watch...");

    // 1. Khởi tạo Mutex
    i2c_ppg_mutex = xSemaphoreCreateMutex();
    i2c_aux_mutex = xSemaphoreCreateMutex();
    
    // 2. Cấp phát Memory Pool (Ưu tiên RAM nội để an toàn)
    // Dùng MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT để đảm bảo tương thích DMA
    uint32_t caps = MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT;
    #define SAFE_ALLOC(size) heap_caps_malloc(size, caps)
    
    sensor_data_queue = xQueueCreate(POOL_SIZE, sizeof(sensor_data_package_t*));
    free_buf_queue = xQueueCreate(POOL_SIZE, sizeof(sensor_data_package_t*));
    
    for (int i = 0; i < POOL_SIZE; i++) {
        void *pkg = SAFE_ALLOC(sizeof(sensor_data_package_t));
        if (pkg) xQueueSend(free_buf_queue, &pkg, 0);
        else ESP_LOGE(TAG, "Alloc Fail Buffer %d", i);
    }

    // 3. Khởi tạo Queues khác
    health_metrics_queue = xQueueCreate(QUEUE_SIZE_HEALTH_METRICS, sizeof(health_metrics_t));
    temperature_queue = xQueueCreate(QUEUE_SIZE_TEMPERATURE, sizeof(float));
    // button_event_queue bỏ vì dùng polling

    // 4. Cấp phát Buffer thô
    g_red_buffer = SAFE_ALLOC(sizeof(uint32_t) * WINDOW_SIZE_SAMPLES);
    g_ir_buffer = SAFE_ALLOC(sizeof(uint32_t) * WINDOW_SIZE_SAMPLES);
    g_accel_x = SAFE_ALLOC(sizeof(int16_t) * WINDOW_SIZE_SAMPLES);
    g_accel_y = SAFE_ALLOC(sizeof(int16_t) * WINDOW_SIZE_SAMPLES);
    g_accel_z = SAFE_ALLOC(sizeof(int16_t) * WINDOW_SIZE_SAMPLES);
    
    if(!g_red_buffer) { ESP_LOGE(TAG, "Global Buffer Alloc Fail!"); return; }

    // 5. Khởi tạo Phần cứng
    if (!ui_init()) ESP_LOGE(TAG, "UI Init Fail");
    
    // Chạy UI Task ngay để hiện Logo
    // Pin to Core 1 to avoid I2C interrupts from Core 0
    xTaskCreatePinnedToCore(lvgl_task, "ui", STACK_SIZE_LVGL, NULL, TASK_PRIORITY_LVGL, &task_handle_lvgl, 1);
    
    i2c_setup_buses();
    button_gpio_init(); // Init nút bấm (Polling mode)
    power_manager_init(); // Init Power Management Lock
    ble_server_init(); // [NEW] Init BLE Server

    // 6. Init Sensors
    if (mpu6050_init(I2C_BUS_AUX) != ESP_OK) ui_show_error("IMU Fail");
    if (max30205_init(I2C_BUS_AUX) != ESP_OK) ESP_LOGW(TAG, "Temp Fail");
    
    if (max30102_init(I2C_BUS_PPG) != ESP_OK) {
        ui_show_error("PPG Fail");
    } else {
        max30102_boost_leds(I2C_BUS_PPG);
    }

    esp_err_t dsp_ret = dsps_fft2r_init_fc32(NULL, 1024);
    if (dsp_ret != ESP_OK) ESP_LOGE(TAG, "DSP Init Failed");

    // 7. Init AI
    if (tinyml_init()) {
        ESP_LOGI(TAG, "AI Engine: OK");
    } else {
        ui_show_error("AI Error");
    }

    // 8. Chạy các Task còn lại
    init_adc(); // [NEW] Init Battery ADC
    xTaskCreatePinnedToCore(sensor_acquisition_task, "sensor", STACK_SIZE_SENSOR, NULL, TASK_PRIORITY_SENSOR, &task_handle_sensor, 0);
    xTaskCreatePinnedToCore(slow_maintenance_task, "maint", STACK_SIZE_TEMPERATURE, NULL, TASK_PRIORITY_TEMPERATURE, &task_handle_temperature, 0);
    xTaskCreatePinnedToCore(signal_processing_task, "proc", STACK_SIZE_PROCESSING, NULL, TASK_PRIORITY_PROCESSING, &task_handle_processing, 1);
    
    // Button Task (Polling Mode)
    xTaskCreatePinnedToCore(button_task, "btn", STACK_SIZE_BUTTON, NULL, TASK_PRIORITY_BUTTON, &task_handle_button, 0);

    ESP_LOGI(TAG, "System Ready.");
}