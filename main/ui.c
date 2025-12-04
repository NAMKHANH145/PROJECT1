#include "ui.h"
#include "config.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include <string.h>
     
      // LVGL & Display Drivers
      #include "lvgl.h"
    #include "esp_lcd_panel_io.h"
    #include "esp_lcd_panel_vendor.h"
    #include "esp_lcd_panel_ops.h"
   
    static const char *TAG = "UI";
    
    // ============================================================ 
    // GLOBAL STATE
    // ============================================================ 
    
    static health_metrics_t current_metrics = {0};
    static ui_screen_t current_screen = SCREEN_HOME;
    static bool ui_initialized = false;
    static esp_lcd_panel_handle_t panel_handle = NULL;
    
    // LVGL objects
    static lv_obj_t *screen_home = NULL;
    static lv_obj_t *screen_respiration = NULL;
    static lv_obj_t *screen_stress = NULL;
    static lv_obj_t *screen_settings = NULL;
    
    static lv_obj_t *label_hr = NULL;
    static lv_obj_t *label_spo2 = NULL;
    static lv_obj_t *label_temp = NULL;
    static lv_obj_t *label_rr = NULL;
    static lv_obj_t *label_stress = NULL;
    static lv_obj_t *label_hrv = NULL;
    
    // ============================================================ 
    // LVGL FLUSH CALLBACK
    // ============================================================ 
   
    static void lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map) {
        esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;
        int offsetx1 = area->x1;
        int offsetx2 = area->x2;
        int offsety1 = area->y1;
        int offsety2 = area->y2;
    
        // Copy a buffer's content to a specific area of the display
        esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);

        // Inform the graphics library that you are ready with the flushing
        lv_disp_flush_ready(drv);
    }
   
    // ============================================================ 
    // SCREEN CREATION
    // ============================================================ 
   
    static void create_home_screen(void) {
        screen_home = lv_obj_create(NULL);

        lv_obj_t *title = lv_label_create(screen_home);
        lv_label_set_text(title, SYSTEM_NAME);
        lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 0);

        label_hr = lv_label_create(screen_home);
        lv_label_set_text(label_hr, "HR: --");
        lv_obj_align(label_hr, LV_ALIGN_LEFT_MID, 5, -8);

        label_spo2 = lv_label_create(screen_home);
        lv_label_set_text(label_spo2, "SpO2: --%");
        lv_obj_align(label_spo2, LV_ALIGN_LEFT_MID, 5, 8);
        label_temp = lv_label_create(screen_home);
        lv_label_set_text(label_temp, "Temp: --.-");
        lv_obj_align(label_temp, LV_ALIGN_LEFT_MID, 5, 24);
    }
    static void create_respiration_screen(void) {
        screen_respiration = lv_obj_create(NULL);

        lv_obj_t *title = lv_label_create(screen_respiration);
        lv_label_set_text(title, "Respiration");
        lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 0);

        label_rr = lv_label_create(screen_respiration);
        lv_label_set_text(label_rr, "Rate: --");
        lv_obj_align(label_rr, LV_ALIGN_CENTER, 0, 0);
    }
   
    static void create_stress_screen(void) {
        screen_stress = lv_obj_create(NULL);

        lv_obj_t *title = lv_label_create(screen_stress);
        lv_label_set_text(title, "Stress Level");
        lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 0);

        label_stress = lv_label_create(screen_stress);
        lv_label_set_text(label_stress, "Sts: --");
        lv_obj_align(label_stress, LV_ALIGN_CENTER, 0, -8);

        label_hrv = lv_label_create(screen_stress);
        lv_label_set_text(label_hrv, "HRV: --");
        lv_obj_align(label_hrv, LV_ALIGN_CENTER, 0, 10);
    }
   
    static void create_settings_screen(void) {
        screen_settings = lv_obj_create(NULL);

        lv_obj_t *title = lv_label_create(screen_settings);
        lv_label_set_text(title, "System Info");
        lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 0);
        lv_obj_t *ver = lv_label_create(screen_settings);
        lv_label_set_text_fmt(ver, "Ver: %s", FIRMWARE_VERSION);
        lv_obj_align(ver, LV_ALIGN_CENTER, 0, 0);
    }
   
    // ============================================================ 
    // PUBLIC API
    // ============================================================ 
   
    bool ui_init(void) {
        ESP_LOGI(TAG, "Initializing UI and Display (ST7789 1.47\")...");
   
        // 1. Initialize SPI Bus
        spi_bus_config_t bus_cfg = {
            .sclk_io_num = SPI_DISPLAY_SCK_PIN,
            .mosi_io_num = SPI_DISPLAY_MOSI_PIN,
            .miso_io_num = -1,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
            .max_transfer_sz = DISPLAY_WIDTH * 40 * sizeof(uint16_t), // Buffer size aligned
        };
        ESP_ERROR_CHECK(spi_bus_initialize(SPI_DISPLAY_HOST, &bus_cfg, SPI_DMA_CH_AUTO));

        // 2. Initialize Panel IO
        esp_lcd_panel_io_handle_t io_handle = NULL;
        esp_lcd_panel_io_spi_config_t io_config = {
            .dc_gpio_num = SPI_DISPLAY_DC_PIN,
            .cs_gpio_num = SPI_DISPLAY_CS_PIN,
            .pclk_hz = 20 * 1000 * 1000, // Reduced to 20MHz for stability
            .lcd_cmd_bits = 8,
            .lcd_param_bits = 8,
            .spi_mode = 2, // ST7789 usually works with Mode 2 or 3
            .trans_queue_depth = 10,
        };
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI_DISPLAY_HOST, &io_config, &io_handle));
        
        // 3. Initialize Panel Driver (ST7789)
        esp_lcd_panel_dev_config_t panel_config = {
            .reset_gpio_num = SPI_DISPLAY_RST_PIN,
            .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
            .bits_per_pixel = 16, 
        };
        ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));
        ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
        ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
        
        // ST7789 1.47" Specifics (172x320 in 240x320 Controller)
        // Offset X = (240 - 172) / 2 = 34
        ESP_ERROR_CHECK(esp_lcd_panel_set_gap(panel_handle, 34, 0)); 
        ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));
        ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, true)); // Landscape/Portrait adjustments if needed
        ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, false, false));

        ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
        
        // 4. Initialize LVGL
        lv_init();

        // 5. Allocate Draw Buffer (Partial Buffer Strategy)
        // 20 lines * 172 width * 2 bytes/pixel
        #define BUF_HEIGHT 40
        static lv_disp_draw_buf_t draw_buf;
        static lv_color_t *buf1;
        buf1 = heap_caps_malloc(DISPLAY_WIDTH * BUF_HEIGHT * sizeof(lv_color_t), MALLOC_CAP_DMA);
        if (buf1 == NULL) {
            ESP_LOGE(TAG, "Critical: Failed to allocate LVGL draw buffer in DMA memory!");
            return false;
        }
        lv_disp_draw_buf_init(&draw_buf, buf1, NULL, DISPLAY_WIDTH * BUF_HEIGHT);

        // 6. Register Display Driver
        static lv_disp_drv_t disp_drv;
        lv_disp_drv_init(&disp_drv);
        disp_drv.hor_res = DISPLAY_WIDTH;
        disp_drv.ver_res = DISPLAY_HEIGHT;
        disp_drv.flush_cb = lvgl_flush_cb;
        disp_drv.draw_buf = &draw_buf;
        disp_drv.user_data = panel_handle;
   
        lv_disp_drv_register(&disp_drv);

        // 7. Create UI
        create_home_screen();
        create_respiration_screen();
        create_stress_screen();
        create_settings_screen();
        lv_scr_load(screen_home);

        ui_initialized = true;
        ESP_LOGI(TAG, "UI Initialized");
        return true;
    }
   
 void ui_update_metrics(const health_metrics_t *metrics) {
     if (!ui_initialized || !metrics) return;
     memcpy(&current_metrics, metrics, sizeof(health_metrics_t));

     // Only update if objects exist (safe check)
     // Handle "Measuring..." state (flagged by -1)
     if (label_hr) {
         if (metrics->heart_rate_bpm == -1) lv_label_set_text(label_hr, "HR: ...");
         else lv_label_set_text_fmt(label_hr, "HR: %d", metrics->heart_rate_bpm);
     }
     
     if (label_spo2) {
         if (metrics->spo2_percent == -1) lv_label_set_text(label_spo2, "SpO2: ...");
         else lv_label_set_text_fmt(label_spo2, "SpO2: %d%%", metrics->spo2_percent);
     }

     if (label_temp) lv_label_set_text_fmt(label_temp, "Temp: %.1f", metrics->temperature_celsius);
     
     if (label_rr) {
         if (metrics->respiration_rate == -1) lv_label_set_text(label_rr, "Rate: ...");
         else lv_label_set_text_fmt(label_rr, "Rate: %d", metrics->respiration_rate);
     }

     if (label_stress) {
         if (metrics->stress_level == -1) lv_label_set_text(label_stress, "Sts: ...");
         else lv_label_set_text_fmt(label_stress, "Sts: %d", metrics->stress_level);
     }

     if (label_hrv) lv_label_set_text_fmt(label_hrv, "HRV: %.0f", metrics->hrv_sdnn);
    }

   void ui_show_error(const char *msg) {
       if (!ui_initialized) return;
       
       // Create a simple error popup or screen
       lv_obj_t * mbox1 = lv_msgbox_create(NULL, "System Error", msg, NULL, true);
       lv_obj_center(mbox1);
       
       // Force update
       lv_task_handler();
   }

   void ui_enter_sleep(void) {
       if (panel_handle) {
           esp_lcd_panel_disp_on_off(panel_handle, false);
       }
   }

   void ui_wake_up(void) {
       if (panel_handle) {
           esp_lcd_panel_disp_on_off(panel_handle, true);
           lv_scr_load(screen_home); // Reset to home
       }
   }
   
   void ui_handle_button(button_event_t event) {
       if (!ui_initialized) return;

       if (event == BUTTON_EVENT_NEXT) {
           current_screen = (current_screen + 1) % SCREEN_COUNT;
       } else {
           current_screen = (current_screen - 1 + SCREEN_COUNT) % SCREEN_COUNT;
       }

       switch (current_screen) {
         case SCREEN_HOME: lv_scr_load(screen_home); break;
         case SCREEN_RESPIRATION: lv_scr_load(screen_respiration); break;
         case SCREEN_STRESS: lv_scr_load(screen_stress); break;
         case SCREEN_SETTINGS: lv_scr_load(screen_settings); break;
         default: break;
       }
   }
   
   ui_screen_t ui_get_current_screen(void) {
       return current_screen;
   }
   
   void ui_tick_handler(void) {
       lv_tick_inc(LVGL_TICK_PERIOD_MS);
   }
   
   void ui_task_handler(void) {
       lv_task_handler();
   }