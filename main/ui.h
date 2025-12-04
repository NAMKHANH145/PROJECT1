#pragma once

#include "config.h"
#include <stdbool.h>

/**
 * ============================================================
 * USER INTERFACE MODULE (LVGL-based)
 * ============================================================
 * 
 * This module manages the OLED display using LVGL.
 * 
 * NOTE: To enable LVGL support:
 * 1. Uncomment "lvgl/lvgl: ^8.3.0" in main/idf_component.yml
 * 2. Set USE_LVGL_DISPLAY to 1 below
 * 3. Configure lv_conf.h for your display
 * 4. Implement display driver in components/ssd1306_lvgl/
 */

// ============================================================
// CONFIGURATION
// ============================================================

// Set to 1 to enable LVGL display, 0 to disable (for testing without hardware)
#define USE_LVGL_DISPLAY 1

// ============================================================
// DATA STRUCTURES
// ============================================================

/**
 * Health metrics to be displayed on UI
 */
typedef struct {
    int heart_rate_bpm;         // Heart rate in BPM
    int spo2_percent;           // SpO2 percentage
    float temperature_celsius;  // Body temperature in Celsius
    int respiration_rate;       // Respiration rate in breaths/min
    int stress_level;           // Stress level (0=baseline, 1=stress)
    float hrv_sdnn;            // HRV SDNN metric
    bool motion_detected;       // Motion artifact flag
    bool data_valid;           // Overall data validity
} health_metrics_t;

/**
 * Button event types
 */
typedef enum {
    BUTTON_EVENT_NEXT,
    BUTTON_EVENT_PREV
} button_event_t;

// ============================================================
// PUBLIC API
// ============================================================

/**
 * @brief Initialize the UI system
 * 
 * Sets up LVGL, display driver, and creates initial screens.
 * 
 * @return true if initialization successful, false otherwise
 */
bool ui_init(void);

/**
 * @brief Update displayed health metrics
 * 
 * This should be called whenever new health data is available.
 * Thread-safe.
 * 
 * @param metrics Pointer to health metrics structure
 */
void ui_update_metrics(const health_metrics_t *metrics);

/**
 * @brief Handle button event (screen navigation)
 * 
 * @param event Button event (next/prev screen)
 */
void ui_handle_button(button_event_t event);

/**
 * @brief Get current active screen
 * 
 * @return Current screen ID
 */
ui_screen_t ui_get_current_screen(void);

/**
 * @brief LVGL tick handler (must be called periodically)
 * 
 * Call this every 10ms from a timer or task.
 */
void ui_tick_handler(void);

/**
 * @brief Main UI task handler
 * 
 * Call this periodically (e.g., every 50ms) to update display.
 * Processes LVGL events and redraws screen.
 */
void ui_task_handler(void);

/**
 * @brief Display a critical system error
 * 
 * @param msg Error message string
 */
void ui_show_error(const char *msg);

/**
 * @brief Enter low power UI mode (Turn off display)
 */
void ui_enter_sleep(void);

/**
 * @brief Wake up UI (Turn on display)
 */
void ui_wake_up(void);
