#pragma once

#include <stdbool.h>
#include "lvgl.h"
#include "config.h" // Provides health_metrics_t and BUTTON defines

// ============================================================
// PUBLIC API
// ============================================================

/**
 * @brief Initialize the UI (LVGL + Display Driver)
 * Mirrors the setup in lcd_test.c
 */
bool ui_init(void);

/**
 * @brief Update the UI with new sensor data.
 * Replaces ui_update_data() from lcd_test.c but takes the project struct.
 */
void ui_update_metrics(const health_metrics_t *metrics);

/**
 * @brief Handle button events (Next/Prev screen)
 */
void ui_handle_button(button_event_t event);

/**
 * @brief Required for LVGL timekeeping
 */
void ui_tick_handler(void);
void ui_task_handler(void);

/**
 * @brief Control display power
 */
void ui_wake_up(void);
void ui_enter_sleep(void);

/**
 * @brief Show specific error message on status line
 */
void ui_show_error(const char *msg);
