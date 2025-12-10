#include "ui.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_timer.h"
#include <stdio.h>
#include <math.h>

static const char *TAG = "UI";

// ==========================================
// 1. HARDWARE CONFIG
// ==========================================
#define LVGL_BUFFER_SIZE (DISPLAY_WIDTH * 80) // 1/4 Screen buffer for performance

// ==========================================
// 2. ASSETS & STYLES
// ==========================================

// --- NEW LOGO: HEART PULSE (64x64) ---
// Thay thế khối màu xanh bằng hình trái tim pixel art
const uint8_t heart_pulse_map[] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x0E, 0x00, 0x00, 0x70, 0x00, 0x00, 
    0x00, 0x00, 0x3F, 0x80, 0x01, 0xFC, 0x00, 0x00,
    0x00, 0x00, 0x7F, 0xE0, 0x07, 0xFE, 0x00, 0x00, 
    0x00, 0x00, 0xFF, 0xF0, 0x0F, 0xFF, 0x00, 0x00,
    0x00, 0x01, 0xFF, 0xF8, 0x1F, 0xFF, 0x80, 0x00, 
    0x00, 0x01, 0xFF, 0xFC, 0x3F, 0xFF, 0x80, 0x00,
    0x00, 0x03, 0xFF, 0xFE, 0x7F, 0xFF, 0xC0, 0x00, 
    0x00, 0x03, 0xFF, 0xFF, 0xFF, 0xFF, 0xC0, 0x00,
    0x00, 0x03, 0xFF, 0xFF, 0xFF, 0xFF, 0xC0, 0x00, 
    0x00, 0x03, 0xFF, 0xFF, 0xFF, 0xFF, 0xC0, 0x00,
    0x00, 0x03, 0xFF, 0xFF, 0xFF, 0xFF, 0xC0, 0x00, 
    0x00, 0x03, 0xFF, 0xFF, 0xFF, 0xFF, 0xC0, 0x00,
    0x00, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0x80, 0x00, 
    0x00, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0x80, 0x00,
    0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 
    0x00, 0x00, 0x7F, 0xFF, 0xFF, 0xFE, 0x00, 0x00,
    0x00, 0x00, 0x3F, 0xFF, 0xFF, 0xFC, 0x00, 0x00, 
    0x00, 0x00, 0x1F, 0xFF, 0xFF, 0xF8, 0x00, 0x00,
    0x00, 0x00, 0x0F, 0xFF, 0xFF, 0xF0, 0x00, 0x00, 
    0x00, 0x00, 0x07, 0xFF, 0xFF, 0xE0, 0x00, 0x00,
    0x00, 0x00, 0x03, 0xFF, 0xFF, 0xC0, 0x00, 0x00, 
    0x00, 0x00, 0x01, 0xFF, 0xFF, 0x80, 0x00, 0x00,
    0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x7F, 0xFE, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x3F, 0xFC, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x1F, 0xF8, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x0F, 0xF0, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x07, 0xE0, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x03, 0xC0, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0x00,
    // (Phần còn lại là 0x00 để lấp đầy 64x64)
};
// Mảng trên mới khoảng 1/2, ta để máy tự hiểu phần thiếu là 0x00 vì static/const

const lv_img_dsc_t heart_img_dsc = {
  .header.cf = LV_IMG_CF_ALPHA_1BIT,
  .header.always_zero = 0,
  .header.reserved = 0,
  .header.w = 64,   
  .header.h = 64,   
  .data_size = 512, 
  .data = heart_pulse_map,
};

// --- FONTS ---
#define UI_FONT_SMALL   &lv_font_montserrat_14
#define UI_FONT_MEDIUM  &lv_font_montserrat_14 
#define UI_FONT_LARGE   &lv_font_montserrat_20 
#define UI_FONT_HUGE    &lv_font_montserrat_28 

// --- COLORS ---
#define UI_COLOR_BG          lv_color_hex(0x000000)
#define UI_COLOR_TEXT_YELLOW lv_color_hex(0xFFFF00)
#define UI_COLOR_ICON_GREEN  lv_color_hex(0x00FF00)
#define UI_COLOR_ICON_WHITE  lv_color_hex(0xFFFFFF)
#define UI_COLOR_TEXT_WHITE  lv_color_hex(0xFFFFFF)
#define UI_COLOR_RED         lv_color_hex(0xFF0000)
#define UI_COLOR_ORANGE      lv_color_hex(0xFFA500)
#define UI_COLOR_GREY        lv_color_hex(0x808080)

// --- GLOBAL OBJECTS ---
static lv_obj_t *scr_splash = NULL;
static lv_obj_t *scr_main = NULL;
static lv_obj_t *scr_secondary = NULL;

static lv_obj_t *lbl_hr_val = NULL;
static lv_obj_t *lbl_hr_conf = NULL; // [NEW] Heart Rate Confidence
static lv_obj_t *lbl_spo2_val = NULL;
static lv_obj_t *lbl_temp_val = NULL;
static lv_obj_t *lbl_status_val = NULL;
static lv_obj_t *lbl_status_overlay = NULL;

static lv_obj_t *lbl_resp_val = NULL;
static lv_obj_t *lbl_resp_conf = NULL; // [NEW] Respiration Confidence
static lv_obj_t *lbl_stress_val = NULL;
static lv_obj_t *lbl_sec_status_val = NULL;

static lv_obj_t *icon_bt_main = NULL;
static lv_obj_t *icon_bat_main = NULL;
static lv_obj_t *icon_bt_sec = NULL;
static lv_obj_t *icon_bat_sec = NULL;

static lv_style_t style_screen;
static lv_style_t style_label_yellow, style_label_green, style_label_white;
static lv_style_t style_icon_large;

static esp_lcd_panel_handle_t panel_handle = NULL;
static int current_screen_idx = 0;

// ==========================================
// 3. UI LAYOUT HELPERS
// ==========================================

static void init_styles(void) {
    lv_style_init(&style_screen);
    lv_style_set_bg_color(&style_screen, UI_COLOR_BG);
    lv_style_set_bg_opa(&style_screen, LV_OPA_COVER);
    lv_style_set_text_font(&style_screen, UI_FONT_SMALL);

    lv_style_init(&style_label_yellow);
    lv_style_set_text_color(&style_label_yellow, UI_COLOR_TEXT_YELLOW);

    lv_style_init(&style_label_green);
    lv_style_set_text_color(&style_label_green, UI_COLOR_ICON_GREEN);

    lv_style_init(&style_label_white);
    lv_style_set_text_color(&style_label_white, UI_COLOR_TEXT_WHITE);

    lv_style_init(&style_icon_large);
    lv_style_set_text_font(&style_icon_large, UI_FONT_HUGE); 
}

static void create_top_bar(lv_obj_t *parent, lv_obj_t **bt_icon_ptr, lv_obj_t **bat_icon_ptr) {
    lv_obj_t * bar = lv_obj_create(parent);
    lv_obj_set_size(bar, LV_PCT(100), 40); // Increased height to 40 to cover artifacts
    lv_obj_set_style_bg_color(bar, UI_COLOR_BG, 0); 
    lv_obj_set_style_bg_opa(bar, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(bar, 0, 0);
    lv_obj_set_flex_flow(bar, LV_FLEX_FLOW_ROW_REVERSE); // Right to Left
    lv_obj_set_flex_align(bar, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_all(bar, 2, 0);
    lv_obj_set_style_pad_right(bar, 0, 0); // Push to right edge
    lv_obj_set_style_pad_column(bar, 5, 0); // Gap between icons

    // 1. Battery Icon (Rightmost)
    *bat_icon_ptr = lv_label_create(bar);
    lv_label_set_text(*bat_icon_ptr, LV_SYMBOL_BATTERY_FULL);
    lv_obj_add_style(*bat_icon_ptr, &style_icon_large, 0);
    lv_obj_set_style_text_color(*bat_icon_ptr, UI_COLOR_ICON_GREEN, 0);

    // 2. Battery Text "36%" (Left of Battery Icon)
    lv_obj_t *bat_pct = lv_label_create(bar);
    lv_label_set_text(bat_pct, "36%");
    lv_obj_set_style_text_font(bat_pct, UI_FONT_SMALL, 0);
    lv_obj_set_style_text_color(bat_pct, UI_COLOR_ICON_GREEN, 0);
    lv_obj_set_style_pad_top(bat_pct, 2, 0); // Align text baseline with icon

    // 3. Bluetooth Icon (Left of Battery Text)
    *bt_icon_ptr = lv_label_create(bar);
    lv_label_set_text(*bt_icon_ptr, LV_SYMBOL_BLUETOOTH);
    lv_obj_add_style(*bt_icon_ptr, &style_icon_large, 0);
    lv_obj_set_style_text_color(*bt_icon_ptr, UI_COLOR_ICON_WHITE, 0);
    lv_obj_set_style_pad_right(*bt_icon_ptr, 5, 0); // Extra gap for BT
}

// MACRO CREATE_SENSOR_GROUP_VERTICAL
// Cập nhật: Tiêu đề (Label) sẽ dùng màu XANH (Green) thay vì Vàng
#define CREATE_SENSOR_GROUP_VERTICAL(parent, label_text, val_ptr, unit_text) \
    do { \
        lv_obj_t * grp = lv_obj_create(parent); \
        lv_obj_set_size(grp, LV_PCT(100), LV_SIZE_CONTENT); \
        lv_obj_set_style_bg_opa(grp, LV_OPA_TRANSP, 0); \
        lv_obj_set_style_border_width(grp, 0, 0); \
        lv_obj_set_style_pad_all(grp, 0, 0); \
        lv_obj_set_style_pad_bottom(grp, 2, 0); \
         \
        /* TITLE: GREEN COLOR */ \
        lv_obj_t * l = lv_label_create(grp); \
        lv_label_set_text(l, label_text); \
        lv_obj_set_style_text_font(l, UI_FONT_LARGE, 0); \
        lv_obj_add_style(l, &style_label_green, 0); \
        lv_obj_align(l, LV_ALIGN_TOP_LEFT, 0, 0); \
         \
        lv_obj_t * cont = lv_obj_create(grp); \
        lv_obj_set_size(cont, LV_SIZE_CONTENT, LV_SIZE_CONTENT); \
        lv_obj_set_layout(cont, LV_LAYOUT_FLEX); \
        lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_ROW); \
        lv_obj_set_flex_align(cont, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_END, LV_FLEX_ALIGN_END);\
        lv_obj_set_style_bg_opa(cont, LV_OPA_TRANSP, 0); \
        lv_obj_set_style_border_width(cont, 0, 0); \
        lv_obj_set_style_pad_all(cont, 0, 0); \
        lv_obj_align(cont, LV_ALIGN_TOP_LEFT, 0, 25); \
         \
        /* VALUE: YELLOW COLOR */ \
        *val_ptr = lv_label_create(cont); \
        lv_label_set_text(*val_ptr, "-- "); \
        lv_obj_set_style_text_font(*val_ptr, UI_FONT_HUGE, 0); \
        lv_obj_add_style(*val_ptr, &style_label_yellow, 0); \
         \
        /* UNIT: YELLOW COLOR */ \
        lv_obj_t * u = lv_label_create(cont); \
        lv_label_set_text(u, unit_text); \
        lv_obj_set_style_text_font(u, UI_FONT_LARGE, 0); \
        lv_obj_add_style(u, &style_label_yellow, 0); \
        lv_obj_set_style_pad_bottom(u, 2, 0); \
    } while(0)

// ==========================================
// 4. SCREEN CREATION
// ==========================================

static void create_splash_screen(void) {
    scr_splash = lv_obj_create(NULL);
    lv_obj_add_style(scr_splash, &style_screen, 0);

    // NEW LOGO
    lv_obj_t * logo_img = lv_img_create(scr_splash);
    lv_img_set_src(logo_img, &heart_img_dsc); 
    lv_obj_set_style_img_recolor(logo_img, lv_color_hex(0xFF4444), 0); // Đỏ tươi
    lv_obj_set_style_img_recolor_opa(logo_img, LV_OPA_COVER, 0);
    lv_obj_align(logo_img, LV_ALIGN_TOP_MID, 0, 40);

    lv_obj_t * cont = lv_obj_create(scr_splash);
    lv_obj_set_size(cont, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_style_bg_opa(cont, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(cont, 0, 0);
    lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(cont, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_align(cont, LV_ALIGN_BOTTOM_MID, 0, -20);

    lv_obj_t * l1 = lv_label_create(cont);
    lv_label_set_text(l1, "Health\nMonitor");
    lv_obj_set_style_text_align(l1, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_add_style(l1, &style_label_white, 0);
    lv_obj_set_style_text_font(l1, UI_FONT_LARGE, 0);

    lv_obj_t * l3 = lv_label_create(cont);
    lv_label_set_text(l3, "v2.0");
    lv_obj_set_style_text_font(l3, UI_FONT_SMALL, 0);
    lv_obj_add_style(l3, &style_label_white, 0);
    lv_obj_set_style_pad_top(l3, 10, 0);
}

static void create_main_dashboard(void) {
    scr_main = lv_obj_create(NULL);
    lv_obj_add_style(scr_main, &style_screen, 0);
    create_top_bar(scr_main, &icon_bt_main, &icon_bat_main);

    lv_obj_t * main_col = lv_obj_create(scr_main);
    lv_obj_set_size(main_col, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_pos(main_col, 0, 40);
    lv_obj_set_style_bg_opa(main_col, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(main_col, 0, 0);
    lv_obj_set_flex_flow(main_col, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_left(main_col, 5, 0);
    lv_obj_set_scroll_dir(main_col, LV_DIR_NONE);

    // Tiêu đề sẽ là màu xanh (Green) nhờ Macro sửa ở trên
    // Heart Rate Group (Manual split for layout)
    do {
        lv_obj_t * grp = lv_obj_create(main_col);
        lv_obj_set_size(grp, LV_PCT(100), LV_SIZE_CONTENT);
        lv_obj_set_style_bg_opa(grp, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(grp, 0, 0);
        lv_obj_set_style_pad_all(grp, 0, 0);
        lv_obj_set_style_pad_bottom(grp, 2, 0);

        /* TITLE */
        lv_obj_t * l = lv_label_create(grp);
        lv_label_set_text(l, "Heart rate:");
        lv_obj_set_style_text_font(l, UI_FONT_LARGE, 0);
        lv_obj_add_style(l, &style_label_green, 0);
        lv_obj_align(l, LV_ALIGN_TOP_LEFT, 0, 0);

        lv_obj_t * cont = lv_obj_create(grp);
        lv_obj_set_size(cont, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
        lv_obj_set_layout(cont, LV_LAYOUT_FLEX);
        lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(cont, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_END, LV_FLEX_ALIGN_END);
        lv_obj_set_style_bg_opa(cont, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(cont, 0, 0);
        lv_obj_set_style_pad_all(cont, 0, 0);
        lv_obj_set_style_pad_column(cont, 1, 0); // Minimal gap between Value-Conf-Unit
        lv_obj_align(cont, LV_ALIGN_TOP_LEFT, 0, 25);

        /* VALUE */
        lbl_hr_val = lv_label_create(cont);
        lv_label_set_text(lbl_hr_val, "-- ");
        lv_obj_set_style_text_font(lbl_hr_val, UI_FONT_HUGE, 0);
        lv_obj_add_style(lbl_hr_val, &style_label_yellow, 0);

        /* CONFIDENCE (Small) */
        lbl_hr_conf = lv_label_create(cont);
        lv_label_set_text(lbl_hr_conf, "");
        lv_obj_set_style_text_font(lbl_hr_conf, UI_FONT_SMALL, 0);
        lv_obj_add_style(lbl_hr_conf, &style_label_yellow, 0);
        lv_obj_set_style_pad_bottom(lbl_hr_conf, 5, 0); 
        // Removed pad_left

        /* UNIT */
        lv_obj_t * u = lv_label_create(cont);
        lv_label_set_text(u, "bpm");
        lv_obj_set_style_text_font(u, UI_FONT_LARGE, 0);
        lv_obj_add_style(u, &style_label_yellow, 0);
        lv_obj_set_style_pad_bottom(u, 2, 0);
        // Removed pad_left

    } while(0);
    CREATE_SENSOR_GROUP_VERTICAL(main_col, "SpO2:", &lbl_spo2_val, "% ");
    CREATE_SENSOR_GROUP_VERTICAL(main_col, "Temperature:", &lbl_temp_val, "°C");

    lv_obj_t * grp_status = lv_obj_create(main_col);
    lv_obj_set_size(grp_status, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_style_bg_opa(grp_status, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(grp_status, 0, 0);
    lv_obj_set_style_pad_all(grp_status, 0, 0);

    lv_obj_t * l_stat = lv_label_create(grp_status);
    lv_label_set_text(l_stat, "Status:");
    lv_obj_set_style_text_font(l_stat, UI_FONT_LARGE, 0);
    lv_obj_add_style(l_stat, &style_label_green, 0); // Green Title
    lv_obj_align(l_stat, LV_ALIGN_TOP_LEFT, 0, 0);

    lbl_status_val = lv_label_create(grp_status);
    lv_label_set_text(lbl_status_val, "Scanning...");
    lv_obj_set_style_text_font(lbl_status_val, UI_FONT_LARGE, 0);
    lv_obj_add_style(lbl_status_val, &style_label_yellow, 0);
    lv_obj_align(lbl_status_val, LV_ALIGN_TOP_LEFT, 0, 25);

    lbl_status_overlay = lv_obj_create(scr_main);
    lv_obj_set_size(lbl_status_overlay, LV_PCT(90), 80);
    lv_obj_center(lbl_status_overlay);
    lv_obj_set_style_bg_color(lbl_status_overlay, UI_COLOR_RED, 0);
    lv_obj_set_style_bg_opa(lbl_status_overlay, LV_OPA_70, 0);
    lv_obj_set_style_border_width(lbl_status_overlay, 3, 0);
    lv_obj_set_style_border_color(lbl_status_overlay, lv_color_white(), 0);
    lv_obj_add_flag(lbl_status_overlay, LV_OBJ_FLAG_HIDDEN);

    lv_obj_t * lbl_warn = lv_label_create(lbl_status_overlay);
    lv_label_set_text(lbl_warn, "CHECK\nSENSOR!");
    lv_obj_set_style_text_font(lbl_warn, UI_FONT_LARGE, 0);
    lv_obj_set_style_text_align(lbl_warn, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(lbl_warn, lv_color_white(), 0);
    lv_obj_center(lbl_warn);
}

static void create_secondary_dashboard(void) {
    scr_secondary = lv_obj_create(NULL);
    lv_obj_add_style(scr_secondary, &style_screen, 0);
    // [FIX] Force Solid Black Background to prevent artifacts
    lv_obj_set_style_bg_color(scr_secondary, UI_COLOR_BG, 0);
    lv_obj_set_style_bg_opa(scr_secondary, LV_OPA_COVER, 0);

    create_top_bar(scr_secondary, &icon_bt_sec, &icon_bat_sec);

    lv_obj_t * main_col = lv_obj_create(scr_secondary);
    lv_obj_set_size(main_col, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_pos(main_col, 0, 40);
    lv_obj_set_style_bg_opa(main_col, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(main_col, 0, 0);
    lv_obj_set_flex_flow(main_col, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_left(main_col, 5, 0);
    lv_obj_set_style_pad_row(main_col, 2, 0);

    lv_obj_t * grp_s = lv_obj_create(main_col);
    lv_obj_set_size(grp_s, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_style_bg_opa(grp_s, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(grp_s, 0, 0);
    lv_obj_set_style_pad_all(grp_s, 0, 0);
    
    lv_obj_t * l_st = lv_label_create(grp_s);
    lv_label_set_text(l_st, "Status:");
    lv_obj_set_style_text_font(l_st, UI_FONT_LARGE, 0);
    lv_obj_add_style(l_st, &style_label_green, 0);
    lv_obj_align(l_st, LV_ALIGN_TOP_LEFT, 0, 0);
    
    lbl_sec_status_val = lv_label_create(grp_s);
    lv_label_set_text(lbl_sec_status_val, "Scanning...");
    lv_obj_set_style_text_font(lbl_sec_status_val, UI_FONT_LARGE, 0);
    lv_obj_add_style(lbl_sec_status_val, &style_label_yellow, 0);
    lv_obj_align(lbl_sec_status_val, LV_ALIGN_TOP_LEFT, 0, 25);

    // Respiration Group (Manual split for layout)
    do {
        lv_obj_t * grp = lv_obj_create(main_col);
        lv_obj_set_size(grp, LV_PCT(100), LV_SIZE_CONTENT);
        lv_obj_set_style_bg_opa(grp, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(grp, 0, 0);
        lv_obj_set_style_pad_all(grp, 0, 0);
        lv_obj_set_style_pad_bottom(grp, 2, 0);

        /* TITLE */
        lv_obj_t * l = lv_label_create(grp);
        lv_label_set_text(l, "Respiration:");
        lv_obj_set_style_text_font(l, UI_FONT_LARGE, 0);
        lv_obj_add_style(l, &style_label_green, 0);
        lv_obj_align(l, LV_ALIGN_TOP_LEFT, 0, 0);

        lv_obj_t * cont = lv_obj_create(grp);
        lv_obj_set_size(cont, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
        lv_obj_set_layout(cont, LV_LAYOUT_FLEX);
        lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(cont, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_END, LV_FLEX_ALIGN_END);
        lv_obj_set_style_bg_opa(cont, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(cont, 0, 0);
        lv_obj_set_style_pad_all(cont, 0, 0);
        lv_obj_set_style_pad_column(cont, 1, 0); // Minimal gap
        lv_obj_align(cont, LV_ALIGN_TOP_LEFT, 0, 25);

        /* VALUE */
        lbl_resp_val = lv_label_create(cont);
        lv_label_set_text(lbl_resp_val, "-- ");
        lv_obj_set_style_text_font(lbl_resp_val, UI_FONT_HUGE, 0);
        lv_obj_add_style(lbl_resp_val, &style_label_yellow, 0);

        /* CONFIDENCE */
        lbl_resp_conf = lv_label_create(cont);
        lv_label_set_text(lbl_resp_conf, "");
        lv_obj_set_style_text_font(lbl_resp_conf, UI_FONT_SMALL, 0);
        lv_obj_add_style(lbl_resp_conf, &style_label_yellow, 0);
        lv_obj_set_style_pad_bottom(lbl_resp_conf, 5, 0);
        // Removed pad_left

        /* UNIT */
        lv_obj_t * u = lv_label_create(cont);
        lv_label_set_text(u, "brpm");
        lv_obj_set_style_text_font(u, UI_FONT_LARGE, 0);
        lv_obj_add_style(u, &style_label_yellow, 0);
        lv_obj_set_style_pad_bottom(u, 2, 0);
        // Removed pad_left

    } while(0);

    lv_obj_t * grp_stress = lv_obj_create(main_col);
    lv_obj_set_size(grp_stress, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_style_bg_opa(grp_stress, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(grp_stress, 0, 0);
    lv_obj_set_style_pad_all(grp_stress, 0, 0);
    
    lv_obj_t * l_stress = lv_label_create(grp_stress);
    lv_label_set_text(l_stress, "Stress:");
    lv_obj_set_style_text_font(l_stress, UI_FONT_LARGE, 0);
    lv_obj_add_style(l_stress, &style_label_green, 0);
    lv_obj_align(l_stress, LV_ALIGN_TOP_LEFT, 0, 0);

    lbl_stress_val = lv_label_create(grp_stress);
    lv_label_set_text(lbl_stress_val, "-- ");
    lv_obj_set_style_text_font(lbl_stress_val, UI_FONT_LARGE, 0);
    lv_obj_add_style(lbl_stress_val, &style_label_yellow, 0);
    lv_obj_align(lbl_stress_val, LV_ALIGN_TOP_LEFT, 0, 25);
}

// ==========================================
// 5. DRIVER INIT
// ==========================================

static void lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map) {
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;
    esp_lcd_panel_draw_bitmap(panel_handle, area->x1, area->y1, area->x2 + 1, area->y2 + 1, color_map);
    lv_disp_flush_ready(drv);
}

bool ui_init(void) {
    ESP_LOGI(TAG, "Initializing UI (Double Buffered)");

    // --- SPI INIT ---
    spi_bus_config_t bus_cfg = {
        .sclk_io_num = SPI_DISPLAY_SCK_PIN,
        .mosi_io_num = SPI_DISPLAY_MOSI_PIN,
        .miso_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t), 
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI_DISPLAY_HOST, &bus_cfg, SPI_DMA_CH_AUTO));

    // --- PANEL IO ---
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = SPI_DISPLAY_DC_PIN,
        .cs_gpio_num = SPI_DISPLAY_CS_PIN,
        .pclk_hz = 20 * 1000 * 1000, 
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0, 
        .trans_queue_depth = 10,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI_DISPLAY_HOST, &io_config, &io_handle));

    // --- PANEL ST7789 ---
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = SPI_DISPLAY_RST_PIN,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB, 
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));
    
    // --- RESET & CONFIG ---
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true)); 
    // Restore original gap setting (34, 0) as (0,0) caused display issues.
    ESP_ERROR_CHECK(esp_lcd_panel_set_gap(panel_handle, 34, 0)); 
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    // --- BACKLIGHT ---
    gpio_set_direction(SPI_DISPLAY_BLK_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(SPI_DISPLAY_BLK_PIN, 1);

    // --- LVGL INIT (DOUBLE BUFFER) ---
    lv_init();

    static lv_disp_draw_buf_t draw_buf;
    static lv_color_t *buf1;
    static lv_color_t *buf2;

    // Use Internal DMA Capable Memory
    buf1 = heap_caps_malloc(LVGL_BUFFER_SIZE * sizeof(lv_color_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    buf2 = heap_caps_malloc(LVGL_BUFFER_SIZE * sizeof(lv_color_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    lv_disp_draw_buf_init(&draw_buf, buf1, buf2, LVGL_BUFFER_SIZE);

    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = DISPLAY_WIDTH; 
    disp_drv.ver_res = DISPLAY_HEIGHT; 
    disp_drv.flush_cb = lvgl_flush_cb;
    disp_drv.draw_buf = &draw_buf;
    disp_drv.user_data = panel_handle;
    lv_disp_drv_register(&disp_drv);

    init_styles();
    create_splash_screen();
    create_main_dashboard();
    create_secondary_dashboard();
    
    lv_scr_load(scr_splash);
    current_screen_idx = 0;

    return true;
}

// ==========================================
// 6. UPDATE LOGIC (FIXED)
// ==========================================

void ui_handle_button(button_event_t event) {
    if (event == BUTTON_EVENT_NEXT) {
        current_screen_idx++;
        if (current_screen_idx > 2) current_screen_idx = 0; // 0->1->2->0
    }
    else if (event == BUTTON_EVENT_PREV) {
        current_screen_idx--;
        if (current_screen_idx < 0) current_screen_idx = 2; // 0->2->1->0
    }

    if (current_screen_idx == 0) lv_scr_load(scr_splash);
    else if (current_screen_idx == 1) lv_scr_load(scr_main);
    else if (current_screen_idx == 2) lv_scr_load(scr_secondary);
    
    // Force full redraw to prevent artifacts (Yellow/Green noise)
    lv_obj_invalidate(lv_scr_act());
}

void ui_update_metrics(const health_metrics_t *metrics) {
    if (!metrics) return;
    if (current_screen_idx == 0) return; 

    // --- 1. HEADER ICONS UPDATE ---
    lv_color_t bt_color = UI_COLOR_GREY;
    
    if (metrics->system_status == STATUS_CONNECTED || metrics->system_status == STATUS_MEASURING) {
        bt_color = UI_COLOR_ICON_WHITE; // Connected
    } else if (metrics->system_status == STATUS_SCANNING) {
        bt_color = UI_COLOR_GREY; // Scanning/Idle
    }
    
    // Update Main Screen Header
    if (icon_bt_main) {
        lv_obj_set_style_text_color(icon_bt_main, bt_color, 0);
    }
    // 2. Battery Text (HIDDEN FOR DEMO)
    /*
    lv_obj_t *bat_pct = lv_obj_get_child(lv_obj_get_parent(*bat_icon_ptr), 1); 
    if (bat_pct) {
        lv_label_set_text_fmt(bat_pct, "%d%%", metrics->battery_percent);
        if(metrics->battery_percent < 20) lv_obj_set_style_text_color(bat_pct, UI_COLOR_RED, 0);
        else lv_obj_set_style_text_color(bat_pct, UI_COLOR_ICON_GREEN, 0);
    }
    */
    
    // Force Full Green Battery for Demo
    if (icon_bat_main) {
        lv_label_set_text(icon_bat_main, LV_SYMBOL_BATTERY_FULL);
        lv_obj_set_style_text_color(icon_bat_main, UI_COLOR_ICON_GREEN, 0);
    }

    // Update Secondary Screen Header
    if (icon_bt_sec) {
        lv_obj_set_style_text_color(icon_bt_sec, bt_color, 0);
    }
    if (icon_bat_sec) {
         if (metrics->low_battery) {
            lv_label_set_text(icon_bat_sec, LV_SYMBOL_BATTERY_EMPTY);
            lv_obj_set_style_text_color(icon_bat_sec, UI_COLOR_RED, 0);
        } else {
            lv_label_set_text(icon_bat_sec, LV_SYMBOL_BATTERY_FULL);
            lv_obj_set_style_text_color(icon_bat_sec, UI_COLOR_ICON_GREEN, 0);
        }
    }

    // --- 2. OVERLAY CHECKS (HIGHEST PRIORITY) ---
    bool show_overlay = false;
    const char *overlay_msg = "";

    if (metrics->i2c_error) {
        show_overlay = true;
        overlay_msg = "CHECK\nSENSOR!";
    } else if (metrics->ai_error) {
        show_overlay = true;
        overlay_msg = "AI MODEL\nERROR!";
    } 
    /* 
    else if (metrics->low_battery) {
        show_overlay = true;
        overlay_msg = "LOW\nBATTERY!";
    }
    */

    if (show_overlay) {
        if (lbl_status_overlay) {
             lv_obj_clear_flag(lbl_status_overlay, LV_OBJ_FLAG_HIDDEN);
             // Access the label child to update text
             lv_obj_t * label = lv_obj_get_child(lbl_status_overlay, 0);
             if(label) lv_label_set_text(label, overlay_msg);
        }
        return; // Stop updating other values
    } else {
        if (lbl_status_overlay) lv_obj_add_flag(lbl_status_overlay, LV_OBJ_FLAG_HIDDEN);
    }

    // --- 2. SYSTEM STATUS ---
    const char *status_txt = "Scanning...";
    lv_color_t status_color = UI_COLOR_TEXT_YELLOW;

    if (metrics->motion_detected) {
        status_txt = "Shaking!";
        status_color = UI_COLOR_ORANGE;
    } else {
        switch (metrics->system_status) {
            case STATUS_SCANNING:
                status_txt = "Scanning...";
                status_color = UI_COLOR_TEXT_YELLOW;
                break;
            case STATUS_CONNECTED:
                status_txt = "Stable";
                status_color = UI_COLOR_ICON_GREEN;
                break;
            case STATUS_MEASURING:
                status_txt = "Measuring...";
                status_color = UI_COLOR_ICON_WHITE;
                break;
            case STATUS_NO_FINGER:
                status_txt = "No Finger";
                status_color = UI_COLOR_RED;
                break;
            case STATUS_ERROR:
                status_txt = "Error";
                status_color = UI_COLOR_RED;
                break;
            default:
                status_txt = "Idle";
                break;
        }
        
        // Removed unreachable fallback logic
    }

    lv_label_set_text(lbl_status_val, status_txt);
    lv_obj_set_style_text_color(lbl_status_val, status_color, 0);
    if(lbl_sec_status_val) {
        lv_label_set_text(lbl_sec_status_val, status_txt);
        lv_obj_set_style_text_color(lbl_sec_status_val, status_color, 0);
    }

    // --- 3. METRIC UPDATES ---
    
    // Heart Rate
    if (metrics->heart_rate_bpm > 0) {
        lv_label_set_text_fmt(lbl_hr_val, "%d", metrics->heart_rate_bpm);
        if(lbl_hr_conf) lv_label_set_text_fmt(lbl_hr_conf, "(%d%%)", (int)metrics->confidence_hr);
        lv_obj_set_style_text_color(lbl_hr_val, UI_COLOR_TEXT_YELLOW, 0);
    } else {
        lv_label_set_text(lbl_hr_val, "-- ");
        if(lbl_hr_conf) lv_label_set_text(lbl_hr_conf, "");
        lv_obj_set_style_text_color(lbl_hr_val, UI_COLOR_GREY, 0);
    }

    // SpO2
    if (metrics->spo2_percent > 0) {
        lv_label_set_text_fmt(lbl_spo2_val, "%d%%", metrics->spo2_percent);
        if(metrics->spo2_percent < 90) lv_obj_set_style_text_color(lbl_spo2_val, UI_COLOR_RED, 0);
        else lv_obj_set_style_text_color(lbl_spo2_val, UI_COLOR_TEXT_YELLOW, 0);
    } else {
        lv_label_set_text(lbl_spo2_val, "-- ");
        lv_obj_set_style_text_color(lbl_spo2_val, UI_COLOR_GREY, 0);
    }

    // Temperature
    if (metrics->temperature_celsius > 10.0f) {
        int temp_int = (int)metrics->temperature_celsius;
        int temp_dec = (int)((metrics->temperature_celsius - temp_int) * 10);
        lv_label_set_text_fmt(lbl_temp_val, "%d.%d", temp_int, temp_dec);
        
        if(metrics->temperature_celsius > 38.0) lv_obj_set_style_text_color(lbl_temp_val, UI_COLOR_RED, 0);
        else lv_obj_set_style_text_color(lbl_temp_val, UI_COLOR_TEXT_YELLOW, 0);
    } else {
        lv_label_set_text(lbl_temp_val, "-- ");
        lv_obj_set_style_text_color(lbl_temp_val, UI_COLOR_GREY, 0);
    }
    
    // Respiration
    if (metrics->respiration_rate > 0) {
        lv_label_set_text_fmt(lbl_resp_val, "%d", metrics->respiration_rate);
        if(lbl_resp_conf) lv_label_set_text_fmt(lbl_resp_conf, "(%d%%)", (int)metrics->confidence_rr);
    } else {
        lv_label_set_text(lbl_resp_val, "-- ");
        if(lbl_resp_conf) lv_label_set_text(lbl_resp_conf, "");
    }

    // Stress
    if(lbl_stress_val) {
        if (metrics->stress_level == 1) { 
            lv_label_set_text_fmt(lbl_stress_val, "HIGH (%d%%)", (int)metrics->confidence_stress);
            lv_obj_set_style_text_color(lbl_stress_val, UI_COLOR_RED, 0);
        } else if (metrics->stress_level == 0) {
            lv_label_set_text_fmt(lbl_stress_val, "LOW (%d%%)", (int)metrics->confidence_stress);
            lv_obj_set_style_text_color(lbl_stress_val, UI_COLOR_ICON_GREEN, 0);
        } else {
             lv_label_set_text(lbl_stress_val, "-- ");
        }
    }
}

void ui_tick_handler(void) {
    lv_tick_inc(LVGL_TICK_PERIOD_MS);
}

void ui_task_handler(void) {
    lv_timer_handler();
}

void ui_show_error(const char *msg) {
    if (lbl_status_val) {
        lv_label_set_text(lbl_status_val, msg);
        lv_obj_set_style_text_color(lbl_status_val, UI_COLOR_RED, 0);
    }
}

void ui_wake_up(void) {
    if (panel_handle) {
        esp_lcd_panel_disp_on_off(panel_handle, true);
        gpio_set_level(SPI_DISPLAY_BLK_PIN, 1);
    }
}

void ui_enter_sleep(void) {
    if (panel_handle) {
        gpio_set_level(SPI_DISPLAY_BLK_PIN, 0);
        esp_lcd_panel_disp_on_off(panel_handle, false);
    }
}
