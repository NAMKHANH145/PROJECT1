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

// ESP-IDF
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"     // Hardware Timer
#include "esp_task_wdt.h"       // Watchdog

// Project Headers
#include "config.h"
#include "max30102.h"
#include "mpu6050.h"
#include "max30205.h"
#include "algorithms.h"
#include "signal_filters.h"
#include "tinyml_manager.h" 
#include "ui.h"

static const char *TAG = "MAIN";

// ============================================================ 
// CONFIGURATION CONSTANTS
// ============================================================ 
#define SENSOR_TIMER_FREQ_HZ    100     // 100Hz Exact Sampling
#define POOL_SIZE               3       // Number of pre-allocated buffers
// Watchdog timeout set to 10s in config.h

// ============================================================ 
// DATA STRUCTURES
// ============================================================ 

typedef struct {
    uint32_t red_buffer[WINDOW_SIZE_SAMPLES];
    uint32_t ir_buffer[WINDOW_SIZE_SAMPLES];
    int16_t accel_x[WINDOW_SIZE_SAMPLES];
    int16_t accel_y[WINDOW_SIZE_SAMPLES];
    int16_t accel_z[WINDOW_SIZE_SAMPLES];
    uint32_t timestamp_ms;
    bool data_valid;
} sensor_data_package_t;

// ============================================================ 
// GLOBAL VARIABLES & HANDLES
// ============================================================ 

static QueueHandle_t sensor_data_queue = NULL;      // Ready data for processing
static QueueHandle_t free_buf_queue = NULL;         // Memory Pool: Available buffers
static QueueHandle_t health_metrics_queue = NULL;
static QueueHandle_t temperature_queue = NULL;
static QueueHandle_t button_event_queue = NULL;

// Separate Mutex for each Bus (Mutex = Chìa khóa, ai có chìa khóa mới được dùng Bus)
static SemaphoreHandle_t i2c_ppg_mutex = NULL;
static SemaphoreHandle_t i2c_aux_mutex = NULL;

static TaskHandle_t task_handle_sensor = NULL;
static TaskHandle_t task_handle_processing = NULL;
static TaskHandle_t task_handle_temperature = NULL;
static TaskHandle_t task_handle_lvgl = NULL;
static TaskHandle_t task_handle_button = NULL;

// Raw buffers for current window accumulation
static uint32_t *g_red_buffer = NULL;
static uint32_t *g_ir_buffer = NULL;
static int16_t *g_accel_x = NULL;
static int16_t *g_accel_y = NULL;
static int16_t *g_accel_z = NULL;

// ============================================================ 
// HARDWARE TIMER CALLBACK (ISR)
// ============================================================ 
// Hàm này được gọi chính xác 100 lần/giây từ phần cứng (Hardware Timer)
static bool IRAM_ATTR on_timer_alarm_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx) {
    BaseType_t high_task_awoken = pdFALSE;
    TaskHandle_t task_to_notify = (TaskHandle_t)user_ctx;
    
    // Đánh thức Task Sensor dậy làm việc
    vTaskNotifyGiveFromISR(task_to_notify, &high_task_awoken);
    
    return (high_task_awoken == pdTRUE);
}

// ============================================================ 
// I2C INITIALIZATION
// ============================================================ 

static void i2c_setup_buses(void) {
    ESP_LOGI(TAG, "Initializing Dual I2C Buses...");

    // 1. Init Bus PPG (Port 0) - MAX30102
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

    // 2. Init Bus AUX (Port 1) - MPU6050 + MAX30205
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

    ESP_LOGI(TAG, "Dual I2C Initialized: PPG(Port%d) AUX(Port%d)", I2C_BUS_PPG, I2C_BUS_AUX);
}

// ============================================================ 
// GPIO BUTTON INTERRUPT HANDLER
// ============================================================ 

static void IRAM_ATTR button_isr_handler(void* arg) {
    button_event_t event = (button_event_t)(int)arg;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(button_event_queue, &event, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR();
}

static void button_gpio_init(void) {
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_NEGEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << BUTTON_NEXT_PIN) | (1ULL << BUTTON_PREV_PIN),
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_NEXT_PIN, button_isr_handler, (void*)BUTTON_EVENT_NEXT);
    gpio_isr_handler_add(BUTTON_PREV_PIN, button_isr_handler, (void*)BUTTON_EVENT_PREV);
}

// ============================================================ 
// TASKS
// ============================================================ 

static void sensor_acquisition_task(void *pvParameters) {
    ESP_LOGI(TAG, "Sensor Task Started (High Precision Mode)");
    
    // 1. Init Watchdog (Giám sát treo hệ thống)
    esp_task_wdt_add(NULL);

    // 2. Setup Hardware Timer (Bộ định thời phần cứng)
    gptimer_handle_t gptimer = NULL;
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, // 1MHz, 1 tick = 1us
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));

    gptimer_alarm_config_t alarm_config = {
        .alarm_count = 1000000 / SENSOR_TIMER_FREQ_HZ, // 10000 ticks = 10ms
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config));

    gptimer_event_callbacks_t cbs = {
        .on_alarm = on_timer_alarm_cb,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, (void*)xTaskGetCurrentTaskHandle()));
    ESP_ERROR_CHECK(gptimer_enable(gptimer));
    ESP_ERROR_CHECK(gptimer_start(gptimer));

    int sample_count = 0;
    sensor_data_package_t *current_pkg = NULL;
    int error_counter_ppg = 0;

    while (1) {
        // WAIT for Timer Notification (Chờ đúng 10ms)
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20));

        // Reset Watchdog (Báo cáo "Tôi vẫn sống")
        esp_task_wdt_reset();

        // --- READ PPG (Bus 0) ---
        // Lấy quyền truy cập Bus 0
        if (xSemaphoreTake(i2c_ppg_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            max30102_data_t ppg_data;
            // Đọc dữ liệu thật
            if (max30102_read_fifo(I2C_BUS_PPG, &ppg_data) == ESP_OK) {
                g_red_buffer[sample_count] = ppg_data.red;
                g_ir_buffer[sample_count] = ppg_data.ir;
                error_counter_ppg = 0;
            } else {
                 // Nếu lỗi: Dùng lại giá trị cũ để giữ đồ thị không bị đứt đoạn
                 g_red_buffer[sample_count] = (sample_count > 0) ? g_red_buffer[sample_count-1] : 0;
                 g_ir_buffer[sample_count] = (sample_count > 0) ? g_ir_buffer[sample_count-1] : 0;
                 
                 error_counter_ppg++;
                 if (error_counter_ppg > 50) {
                     // TEST CASE: Hot-Fix Logic
                     // Nếu lỗi quá 50 lần (0.5s), thử init lại sensor
                     ESP_LOGE(TAG, "CRITICAL: PPG Sensor Lost! Attempting Re-Init...");
                     max30102_init(I2C_BUS_PPG); // Thử cứu vãn
                     error_counter_ppg = 0;
                 }
            }
            xSemaphoreGive(i2c_ppg_mutex); // Trả quyền truy cập
        }

        // --- READ IMU (Bus 1) ---
        if (xSemaphoreTake(i2c_aux_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            mpu6050_raw_data_t imu_data;
            if (mpu6050_read_raw_data(I2C_BUS_AUX, &imu_data) == ESP_OK) {
                g_accel_x[sample_count] = imu_data.accel_x;
                g_accel_y[sample_count] = imu_data.accel_y;
                g_accel_z[sample_count] = imu_data.accel_z;
            } else {
                 // Lỗi thì giữ giá trị cũ
                 g_accel_x[sample_count] = (sample_count > 0) ? g_accel_x[sample_count-1] : 0;
                 g_accel_y[sample_count] = (sample_count > 0) ? g_accel_y[sample_count-1] : 0;
                 g_accel_z[sample_count] = (sample_count > 0) ? g_accel_z[sample_count-1] : 0;
            }
            xSemaphoreGive(i2c_aux_mutex);
        }

        // --- DEBUG: SERIAL PLOTTER (20Hz) ---
        // Chỉ in 1 mẫu mỗi 5 mẫu (100Hz / 5 = 20Hz) để không tắc nghẽn UART
        // Format này dùng được cho "Serial Plotter" của Arduino IDE hoặc Teleplot
        if (sample_count % 5 == 0) {
            printf(">RED:%lu\n>IR:%lu\n>ACC_Z:%d\n", 
                   g_red_buffer[sample_count], 
                   g_ir_buffer[sample_count],
                   g_accel_z[sample_count]);
        }

        sample_count++;

        // Window Complete? (Đủ 10 giây dữ liệu chưa?)
        if (sample_count >= WINDOW_SIZE_SAMPLES) {
            
            // MEMORY POOL PATTERN: Lấy 1 cái hộp rỗng từ kho
            if (xQueueReceive(free_buf_queue, &current_pkg, 0) == pdTRUE) {
                
                // Copy dữ liệu vào hộp
                memcpy(current_pkg->red_buffer, g_red_buffer, sizeof(uint32_t) * WINDOW_SIZE_SAMPLES);
                memcpy(current_pkg->ir_buffer, g_ir_buffer, sizeof(uint32_t) * WINDOW_SIZE_SAMPLES);
                memcpy(current_pkg->accel_x, g_accel_x, sizeof(int16_t) * WINDOW_SIZE_SAMPLES);
                memcpy(current_pkg->accel_y, g_accel_y, sizeof(int16_t) * WINDOW_SIZE_SAMPLES);
                memcpy(current_pkg->accel_z, g_accel_z, sizeof(int16_t) * WINDOW_SIZE_SAMPLES);
                
                current_pkg->timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000);
                current_pkg->data_valid = true;

                // Gửi hộp đi cho Task AI xử lý
                if (xQueueSend(sensor_data_queue, &current_pkg, 0) != pdTRUE) {
                    ESP_LOGE(TAG, "Processing too slow! Dropping Frame.");
                    // Nếu AI bận quá, trả hộp về kho ngay để không mất hộp
                    xQueueSend(free_buf_queue, &current_pkg, 0);
                }
            } else {
                ESP_LOGW(TAG, "Memory Pool Empty! Frame Dropped.");
            }
            
            sample_count = 0;
        }
    }
}

#include "esp_pm.h" // Power Management
#include "dsps_fft2r.h" // DSP Library

// ... (Previous includes)

// ============================================================ 
// POWER MANAGEMENT (BREAKTHROUGH 2)
// ============================================================ 
static esp_pm_lock_handle_t s_cpu_lock = NULL;

static void power_manager_init(void) {
    // Lock CPU frequency to max when active (requires CONFIG_PM_ENABLE)
    // If not enabled in menuconfig, this will just return ESP_ERR_NOT_SUPPORTED (safe)
    esp_pm_lock_create(ESP_PM_CPU_FREQ_MAX, 0, "cpu_max", &s_cpu_lock);
}

static void set_power_mode(bool active) {
    if (s_cpu_lock) {
        if (active) {
            esp_pm_lock_acquire(s_cpu_lock);
        } else {
            esp_pm_lock_release(s_cpu_lock);
        }
    }
}

static void signal_processing_task(void *pvParameters) {
    sensor_data_package_t *sensor_pkg;
    
    health_metrics_t metrics = {
        .heart_rate_bpm = -1,
        .spo2_percent = -1,
        .respiration_rate = -1,
        .stress_level = -1,
        .data_valid = false
    };
    
    xQueueSend(health_metrics_queue, &metrics, 0);
    
    // Processing buffers
    float *ppg_float = heap_caps_malloc(sizeof(float) * WINDOW_SIZE_SAMPLES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    float *rr_intervals = heap_caps_malloc(sizeof(float) * 30, MALLOC_CAP_8BIT);
    
    // BREAKTHROUGH 3: FFT BUFFERS (Need history)
    // Circular buffer for RR history (store last ~64 intervals)
    #define RR_HISTORY_SIZE 64
    float *rr_history = heap_caps_malloc(sizeof(float) * RR_HISTORY_SIZE, MALLOC_CAP_8BIT);
    int rr_hist_idx = 0;
    int rr_hist_count = 0;
    float *fft_resample_buf = heap_caps_malloc(sizeof(float) * FFT_SIZE, MALLOC_CAP_8BIT);

    if (!ppg_float || !rr_intervals || !rr_history || !fft_resample_buf) {
        ESP_LOGE(TAG, "CRITICAL: Failed to allocate buffers!");
        vTaskSuspend(NULL);
    }
    
    // Initialize Filters
    ppg_filter_t ppg_filter;
    ppg_filter_init(&ppg_filter);

    kalman_filter_t kf_ax, kf_ay, kf_az;
    kalman_filter_init(&kf_ax, 0.01f, 0.1f, 0.0f); 
    kalman_filter_init(&kf_ay, 0.01f, 0.1f, 0.0f);
    kalman_filter_init(&kf_az, 0.01f, 0.1f, 0.0f);

    // BREAKTHROUGH 1: LMS Adaptive Filter
    lms_filter_t lms_filter;
    lms_filter_init(&lms_filter, 0.05f); 

    // Initialize Power Manager
    power_manager_init();
    bool is_power_active = false;

    while (1) {
        if (xQueueReceive(sensor_data_queue, &sensor_pkg, portMAX_DELAY) == pdTRUE) {
            
            // --- START PERFORMANCE METERING ---
            int64_t t_start = esp_timer_get_time();

            // POWER MODE: ACTIVE (We are processing)
            if (!is_power_active) {
                set_power_mode(true);
                is_power_active = true;
            }

            // 1. Kalman Filter (Accel) & Signal Chain (LMS)
            for (int i = 0; i < WINDOW_SIZE_SAMPLES; i++) {
                    float ax_smooth = kalman_filter_update(&kf_ax, (float)sensor_pkg->accel_x[i]);
                    float ay_smooth = kalman_filter_update(&kf_ay, (float)sensor_pkg->accel_y[i]);
                    float az_smooth = kalman_filter_update(&kf_az, (float)sensor_pkg->accel_z[i]);
                    
                    sensor_pkg->accel_x[i] = (int16_t)ax_smooth;
                    sensor_pkg->accel_y[i] = (int16_t)ay_smooth;
                    sensor_pkg->accel_z[i] = (int16_t)az_smooth;

                    float bp_out = ppg_filter_process(&ppg_filter, (float)sensor_pkg->ir_buffer[i]);
                    
                    float noise_ref = (float)sensor_pkg->accel_z[i] / 16384.0f;
                    static float avg_acc = 0;
                    avg_acc = 0.95f * avg_acc + 0.05f * noise_ref;
                    noise_ref -= avg_acc;

                    float clean_ppg = lms_filter_update(&lms_filter, bp_out, noise_ref);
                    ppg_float[i] = clean_ppg;
            }

            int64_t t_filter_done = esp_timer_get_time();

            bool motion = detect_motion_artifact(sensor_pkg->accel_x, sensor_pkg->accel_y,
                                               sensor_pkg->accel_z, WINDOW_SIZE_SAMPLES, MOTION_THRESHOLD);
            metrics.motion_detected = motion;
            
            // 3. Normalize & AI
            z_score_normalize(ppg_float, WINDOW_SIZE_SAMPLES);
            int hr_bpm = tinyml_predict_bpm(ppg_float);
            metrics.heart_rate_bpm = hr_bpm;

            // 4. SpO2 & Power Logic
            int spo2 = calculate_spo2(sensor_pkg->red_buffer, sensor_pkg->ir_buffer, WINDOW_SIZE_SAMPLES);
            
            static int off_wrist_counter = 0;
            if (spo2 == -1) {
                off_wrist_counter++;
                if (off_wrist_counter >= 3) {
                        metrics.spo2_percent = -1;
                        metrics.heart_rate_bpm = -1;
                        
                        // POWER MODE: IDLE (Off wrist)
                        if (is_power_active) {
                            set_power_mode(false); // Release CPU lock -> DFS drops freq
                            is_power_active = false;
                            ESP_LOGI(TAG, "Entering Low Power Mode (DFS)");
                        }
                } else {
                    metrics.spo2_percent = -1;
                }
            } else {
                off_wrist_counter = 0;
                metrics.spo2_percent = spo2;
            }

            // 5. Respiration & Basic Stress
            int rr = tinyml_predict_respiration(ppg_float);
            metrics.respiration_rate = rr;
            
            // 6. BREAKTHROUGH 3: FFT HRV Analysis
            int new_rr_count = extract_rr_intervals(ppg_float, hr_bpm, rr_intervals, 30);
            
            // Add to history
            for (int i = 0; i < new_rr_count; i++) {
                rr_history[rr_hist_idx] = rr_intervals[i];
                rr_hist_idx = (rr_hist_idx + 1) % RR_HISTORY_SIZE;
                if (rr_hist_count < RR_HISTORY_SIZE) rr_hist_count++;
            }

            float lf_hf_ratio = 0.0f;
            // Compute FFT every 5 windows (approx 5-10 seconds) to save CPU
            static int fft_timer = 0;
            fft_timer++;
            if (fft_timer >= 5 && rr_hist_count >= 32) { // Need at least 32 points
                fft_timer = 0;
                
                // Unroll circular buffer for FFT function
                float *temp_rr = heap_caps_malloc(sizeof(float) * rr_hist_count, MALLOC_CAP_8BIT);
                if (temp_rr) {
                    for(int k=0; k<rr_hist_count; k++) {
                         // Correct unwinding:
                         int idx = (rr_hist_idx - rr_hist_count + k + RR_HISTORY_SIZE) % RR_HISTORY_SIZE;
                         temp_rr[k] = rr_history[idx];
                    }
                    
                    if (resample_rr_intervals(temp_rr, rr_hist_count, fft_resample_buf)) {
                         lf_hf_ratio = calculate_lf_hf_ratio(fft_resample_buf);
                    }
                    free(temp_rr);
                }
            }

            // Map LF/HF to Stress (0-100)
            int fft_stress = (int)(lf_hf_ratio * 30.0f);
            if (fft_stress > 100) fft_stress = 100;
            
            // Fusion Stress: Combine AI Stress (Motion robust) + FFT Stress (Precision)
            if (metrics.motion_detected) {
                 metrics.stress_level = tinyml_predict_stress(ppg_float, sensor_pkg->accel_x, sensor_pkg->accel_y, sensor_pkg->accel_z);
            } else {
                 metrics.stress_level = (metrics.stress_level + fft_stress) / 2; // Average
            }
            
            int64_t t_algo_done = esp_timer_get_time();

            // SHADOW MONITOR (Aggregated Metrics)
            // Format: METRICS,BPM,SpO2,RR,Stress,LF_HF
            printf("METRICS,%d,%d,%d,%d,%.2f\n", hr_bpm, spo2, rr, metrics.stress_level, lf_hf_ratio);

            // --- PERFORMANCE LOGGING ---
            float t_filter_ms = (float)(t_filter_done - t_start) / 1000.0f;
            float t_algo_ms   = (float)(t_algo_done - t_filter_done) / 1000.0f;
            float t_total_ms  = (float)(t_algo_done - t_start) / 1000.0f;
            
            // Cửa sổ lấy mẫu là 10s = 10000ms. Nếu t_total > 1000ms (1s) là bắt đầu hơi chậm, nhưng vẫn OK.
            // Nếu t_total > 9000ms là hệ thống quá tải.
            ESP_LOGI(TAG, "[PERF] Filter: %.1fms | AI/Math: %.1fms | Total: %.1fms | StackFree: %d bytes", 
                     t_filter_ms, t_algo_ms, t_total_ms, uxTaskGetStackHighWaterMark(NULL));

            xQueueSend(health_metrics_queue, &metrics, 0);
            
            // Return buffer
            if (xQueueSend(free_buf_queue, &sensor_pkg, 0) != pdTRUE) {
                ESP_LOGE(TAG, "Free Queue Full? Logic Error.");
            }
        }
    }
}
static void temperature_task(void *pvParameters) {
    temp_filter_t temp_filter;
    temp_filter_init(&temp_filter);
    
    while (1) {
        float temperature = 0.0f;
        if (xSemaphoreTake(i2c_aux_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            if (max30205_read_temperature(I2C_BUS_AUX, &temperature) == ESP_OK) {
                float smoothed_temp = temp_filter_process(&temp_filter, temperature);
                xQueueSend(temperature_queue, &smoothed_temp, 0);
            }
            xSemaphoreGive(i2c_aux_mutex);
        }

        // --- SYSTEM HEALTH MONITOR (Every 5s) ---
        // TEST CASE: Kiểm tra xem có bị tràn Stack (bộ nhớ cục bộ) không
        // "High Water Mark" = Số byte THẤP NHẤT từng còn dư trong Stack.
        // Nếu số này tiệm cận 0 -> Nguy hiểm (Stack Overflow).
        ESP_LOGI(TAG, "--- SYSTEM HEALTH ---");
        if (task_handle_sensor) 
            ESP_LOGI(TAG, "Sensor Task Stack Free: %d bytes", uxTaskGetStackHighWaterMark(task_handle_sensor));
        if (task_handle_processing) 
            ESP_LOGI(TAG, "Proc Task Stack Free: %d bytes", uxTaskGetStackHighWaterMark(task_handle_processing));
        if (task_handle_lvgl) 
            ESP_LOGI(TAG, "UI Task Stack Free: %d bytes", uxTaskGetStackHighWaterMark(task_handle_lvgl));
        
        size_t free_heap = heap_caps_get_free_size(MALLOC_CAP_8BIT);
        ESP_LOGI(TAG, "Total Heap Free: %d bytes", free_heap);
        ESP_LOGI(TAG, "---------------------");

        vTaskDelay(pdMS_TO_TICKS(TEMP_SAMPLE_INTERVAL_MS));
    }
}

static void lvgl_task(void *pvParameters) {
    health_metrics_t metrics;
    float temperature;
    // ui_init(); // MOVED TO APP_MAIN to prevent Race Condition

    while (1) {
        if (xQueueReceive(health_metrics_queue, &metrics, 0) == pdTRUE) {
            ui_update_metrics(&metrics);
        }
        if (xQueueReceive(temperature_queue, &temperature, 0) == pdTRUE) {
            health_metrics_t temp_update;
            memcpy(&temp_update, &metrics, sizeof(health_metrics_t));
            temp_update.temperature_celsius = temperature;
            ui_update_metrics(&temp_update);
        }
        ui_task_handler();
        vTaskDelay(pdMS_TO_TICKS(LVGL_TASK_DELAY_MS));
    }
}

static void button_task(void *pvParameters) {
    button_event_t event;
    TickType_t last_press_time = 0;
    while (1) {
        if (xQueueReceive(button_event_queue, &event, portMAX_DELAY) == pdTRUE) {
            TickType_t current_time = xTaskGetTickCount();
            if ((current_time - last_press_time) > pdMS_TO_TICKS(BUTTON_DEBOUNCE_MS)) {
                ui_wake_up();
                ui_handle_button(event);
                last_press_time = current_time;
            }
        }
    }
}

void app_main(void) {
    // SAFETY DELAY (Chờ 2 giây để nguồn điện ổn định)
    vTaskDelay(pdMS_TO_TICKS(2000));

    ESP_LOGI(TAG, "Starting Health Watch - Senior Edition");
    
    // In thông tin bộ nhớ để Debug
    size_t free_ram = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    size_t free_psram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    ESP_LOGI(TAG, "Free Internal RAM: %d bytes", free_ram);
    ESP_LOGI(TAG, "Free PSRAM: %d bytes", free_psram);

    gpio_reset_pin(GPIO_NUM_3);

    i2c_ppg_mutex = xSemaphoreCreateMutex();
    i2c_aux_mutex = xSemaphoreCreateMutex();

    // 1. MEMORY POOL INITIALIZATION
    sensor_data_queue = xQueueCreate(POOL_SIZE, sizeof(sensor_data_package_t*));
    free_buf_queue = xQueueCreate(POOL_SIZE, sizeof(sensor_data_package_t*));
    
    for (int i = 0; i < POOL_SIZE; i++) {
        sensor_data_package_t *pkg = heap_caps_malloc(sizeof(sensor_data_package_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (pkg == NULL) {
             ESP_LOGE(TAG, "Fatal: PSRAM Alloc Failed");
             abort(); 
        }
        xQueueSend(free_buf_queue, &pkg, 0);
    }

    g_red_buffer = heap_caps_malloc(sizeof(uint32_t) * WINDOW_SIZE_SAMPLES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    g_ir_buffer = heap_caps_malloc(sizeof(uint32_t) * WINDOW_SIZE_SAMPLES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    g_accel_x = heap_caps_malloc(sizeof(int16_t) * WINDOW_SIZE_SAMPLES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    g_accel_y = heap_caps_malloc(sizeof(int16_t) * WINDOW_SIZE_SAMPLES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    g_accel_z = heap_caps_malloc(sizeof(int16_t) * WINDOW_SIZE_SAMPLES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

    i2c_setup_buses();
    button_gpio_init();
    
    health_metrics_queue = xQueueCreate(QUEUE_SIZE_HEALTH_METRICS, sizeof(health_metrics_t));
    temperature_queue = xQueueCreate(QUEUE_SIZE_TEMPERATURE, sizeof(float));
    button_event_queue = xQueueCreate(QUEUE_SIZE_BUTTON_EVENTS, sizeof(button_event_t));

    esp_task_wdt_config_t twdt_config = {
        .timeout_ms = TWDT_TIMEOUT_S * 1000, 
        .idle_core_mask = (1 << 0) | (1 << 1),
        .trigger_panic = false, // Đổi thành FALSE: Chỉ in lỗi, không reset chip              
    };
    ESP_ERROR_CHECK(esp_task_wdt_init(&twdt_config));

    // === CRITICAL FIX: INIT UI FIRST ===
    // Đảm bảo màn hình hoạt động dù cảm biến chết
    if (!ui_init()) {
        ESP_LOGE(TAG, "UI Init Failed!");
    }
    
    // Start UI Task (Handles refreshing)
    xTaskCreatePinnedToCore(lvgl_task, "lvgl_task", STACK_SIZE_LVGL, NULL, TASK_PRIORITY_LVGL, &task_handle_lvgl, 0);
    vTaskDelay(pdMS_TO_TICKS(200));

    // Kiểm tra cảm biến (Chỉ để báo log, không dừng hệ thống)
    bool sensors_ok = true;
    char error_msg[32] = {0};
    
    if (mpu6050_init(I2C_BUS_AUX) != ESP_OK) {
        sensors_ok = false;
        snprintf(error_msg, 32, "IMU Init Failed!");
        ESP_LOGE(TAG, "%s", error_msg);
    } 
    
    if (max30205_init(I2C_BUS_AUX) != ESP_OK) {
        sensors_ok = false;
        snprintf(error_msg, 32, "Temp Sensor Failed!");
        ESP_LOGE(TAG, "%s", error_msg);
    }
    
    if (max30102_init(I2C_BUS_PPG) != ESP_OK) {
        sensors_ok = false;
        snprintf(error_msg, 32, "PPG Sensor Failed!");
        ESP_LOGE(TAG, "%s", error_msg);
    }
    
    if (!sensors_ok) {
         ui_show_error(error_msg);
         ESP_LOGE(TAG, "Hardware Error Detected. Entering ROBUST MODE...");
    } else {
         ESP_LOGI(TAG, "All Sensors OK");
    }

     esp_err_t dsp_ret = dsps_fft2r_init_fc32(NULL, 1024);
     if (dsp_ret != ESP_OK) ESP_LOGE(TAG, "DSP Init Failed");

     if (!tinyml_init()) ui_show_error("AI Init Failed");

    // LUÔN LUÔN khởi tạo task cảm biến để hệ thống chạy, dù đọc ra toàn số 0
    xTaskCreatePinnedToCore(sensor_acquisition_task, "sensor_task", STACK_SIZE_SENSOR, NULL, TASK_PRIORITY_SENSOR, &task_handle_sensor, 0);
    xTaskCreatePinnedToCore(temperature_task, "temp_task", STACK_SIZE_TEMPERATURE, NULL, TASK_PRIORITY_TEMPERATURE, &task_handle_temperature, 0);
    xTaskCreatePinnedToCore(signal_processing_task, "proc_task", STACK_SIZE_PROCESSING, NULL, TASK_PRIORITY_PROCESSING, &task_handle_processing, 1);
    
    xTaskCreatePinnedToCore(button_task, "btn_task", STACK_SIZE_BUTTON, NULL, TASK_PRIORITY_BUTTON, &task_handle_button, 0);
  
    ESP_LOGI(TAG, "System Running (Debug Mode Enabled).");
}
