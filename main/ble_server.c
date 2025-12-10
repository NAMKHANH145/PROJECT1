#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"

#define TAG_BLE "BLE_SRV"

// UUIDs
#define SERVICE_UUID        0x00FF 
#define CHAR_UUID_DATA      0xFF01

static uint16_t g_gatts_if = ESP_GATT_IF_NONE;
static uint16_t g_conn_id = 0xFFFF;
static uint16_t g_attr_handle = 0;
static bool g_connected = false;

// Advertising Data
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = false,
    .min_interval = 0x0006, 
    .max_interval = 0x0010, 
    .appearance = 0x00,
    .manufacturer_len = 0, 
    .p_manufacturer_data =  NULL, 
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = 0,
    .p_service_uuid = NULL,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min        = 0x320, // [OPTIMIZED] 500ms (0x320 * 0.625ms)
    .adv_int_max        = 0x640, // [OPTIMIZED] 1000ms (0x640 * 0.625ms)
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

// GAP Handler
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&adv_params);
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(TAG_BLE, "Advertising start failed");
        }
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
        ESP_LOGI(TAG_BLE, "Connection params updated: min=%d, max=%d, latency=%d", 
                 param->update_conn_params.min_int, 
                 param->update_conn_params.max_int, 
                 param->update_conn_params.latency);
        break;
    default:
        break;
    }
}

// GATTS Handler
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    switch (event) {
    case ESP_GATTS_REG_EVT:
        g_gatts_if = gatts_if;
        esp_ble_gap_set_device_name("HealthWatch");
        esp_ble_gap_config_adv_data(&adv_data);
        
        esp_gatt_srvc_id_t service_id;
        service_id.is_primary = true;
        service_id.id.inst_id = 0x00;
        service_id.id.uuid.len = ESP_UUID_LEN_16;
        service_id.id.uuid.uuid.uuid16 = SERVICE_UUID;

        esp_ble_gatts_create_service(gatts_if, &service_id, 4);
        break;

    case ESP_GATTS_CREATE_EVT:
        esp_ble_gatts_start_service(param->create.service_handle);
        
        esp_bt_uuid_t char_uuid;
        char_uuid.len = ESP_UUID_LEN_16;
        char_uuid.uuid.uuid16 = CHAR_UUID_DATA;

        esp_gatt_char_prop_t a_property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;

        esp_ble_gatts_add_char(param->create.service_handle, &char_uuid,
                               ESP_GATT_PERM_READ, a_property, NULL, NULL);
        break;

    case ESP_GATTS_ADD_CHAR_EVT:
        g_attr_handle = param->add_char.attr_handle;
        break;

    case ESP_GATTS_CONNECT_EVT:
        g_conn_id = param->connect.conn_id;
        g_connected = true;
        ESP_LOGI(TAG_BLE, "Device Connected");
        
        // [OPTIMIZED] Request looser connection parameters to save CPU & Power
        // Min Interval: 100ms (0x50), Max Interval: 200ms (0xA0)
        // Latency: 0, Timeout: 4000ms (0x190)
        esp_ble_conn_update_params_t conn_params = {0};
        memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
        conn_params.min_int = 0x50; 
        conn_params.max_int = 0xA0; 
        conn_params.latency = 0;
        conn_params.timeout = 400; 
        esp_ble_gap_update_conn_params(&conn_params);
        break;

    case ESP_GATTS_DISCONNECT_EVT:
        g_connected = false;
        esp_ble_gap_start_advertising(&adv_params);
        ESP_LOGI(TAG_BLE, "Device Disconnected");
        break;

    default:
        break;
    }
}

// Public API
void ble_server_init(void) {
    esp_err_t ret;

    // Initialize NVS (Required for BT)
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) { ESP_LOGE(TAG_BLE, "Controller init failed"); return; }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) { ESP_LOGE(TAG_BLE, "Controller enable failed"); return; }

    ret = esp_bluedroid_init();
    if (ret) { ESP_LOGE(TAG_BLE, "Bluedroid init failed"); return; }

    ret = esp_bluedroid_enable();
    if (ret) { ESP_LOGE(TAG_BLE, "Bluedroid enable failed"); return; }

    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret) { ESP_LOGE(TAG_BLE, "GATTS reg failed"); return; }

    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret) { ESP_LOGE(TAG_BLE, "GAP reg failed"); return; }

    ret = esp_ble_gatts_app_register(0);
    if (ret) { ESP_LOGE(TAG_BLE, "App reg failed"); return; }
    
    ESP_LOGI(TAG_BLE, "BLE Server Initialized");
}

void ble_update_data(int hr, int spo2, int temp_int, int bat) {
    // Chỉ làm gì đó nếu ĐÃ KẾT NỐI
    if (g_connected && g_attr_handle != 0) {
        // Chuẩn bị dữ liệu gọn nhẹ
        char buffer[32];
        // Format: "H:72,S:98,T:36,B:85" (Viết tắt để gói tin nhỏ nhất có thể)
        int len = snprintf(buffer, sizeof(buffer), "H:%d,S:%d,T:%d,B:%d", hr, spo2, temp_int, bat);
        
        // Gửi dạng Indication/Notify
        esp_ble_gatts_send_indicate(g_gatts_if, g_conn_id, g_attr_handle, len, (uint8_t *)buffer, false);
    } 
    // Nếu chưa kết nối -> Không làm gì cả (Tiết kiệm CPU)
}