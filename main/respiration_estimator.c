#include "respiration_estimator.h"
#include "config.h"
#include "tinyml_manager.h"
#include "esp_log.h"

static const char *TAG = "RESP_EST";

void respiration_model_init(void) {
    // Model initialization is handled centrally by tinyml_manager.c
    ESP_LOGI(TAG, "Respiration model init delegated to TinyML Manager");
}

int respiration_model_predict(float *ppg_signal, float *workspace_buffer) {
    if (ppg_signal == NULL) {
        return -1;
    }
    
    // Delegate inference to TinyML Manager which holds the TFLite interpreter
    // and implements the specific DSP post-processing (Moving Average, Prominence Check)
    int rr = tinyml_predict_respiration(ppg_signal);
    
    // Validate range (sanity check)
    if (rr < RR_VALID_RANGE_MIN) rr = RR_VALID_RANGE_MIN;
    if (rr > RR_VALID_RANGE_MAX) rr = RR_VALID_RANGE_MAX;
    
    return rr;
}