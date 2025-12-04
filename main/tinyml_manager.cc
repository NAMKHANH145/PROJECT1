#include "tinyml_manager.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include <math.h>
#include <string.h>

// TensorFlow Lite Micro Headers
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/system_setup.h"
#include "tensorflow/lite/schema/schema_generated.h"

// Model Headers
#include "denoise_model.h"     // Heart Rate
#include "stress_model.h"      // Stress
#include "respiration_model.h" // Respiration

static const char *TAG = "TinyML";

// ============================================================
// CONFIGURATION
// ============================================================
// Increased Arena size for U-Net models
#define TENSOR_ARENA_SIZE (90 * 1024) 

static uint8_t *tensor_arena = nullptr;

// Interpreters
static tflite::MicroInterpreter *interpreter_hr = nullptr;
static tflite::MicroInterpreter *interpreter_stress = nullptr;
static tflite::MicroInterpreter *interpreter_resp = nullptr;

// Models
const tflite::Model* model_hr = nullptr;
const tflite::Model* model_stress = nullptr;
const tflite::Model* model_resp = nullptr;

// Resolver
static tflite::MicroMutableOpResolver<20> *resolver = nullptr; 

// ============================================================
// HELPER FUNCTIONS
// ============================================================

static void smooth_signal(float *input, float *output, int length, int window) {
    // Simple Moving Average
    float sum = 0;
    int half_window = window / 2;
    
    // Initial window fill
    for(int i=0; i<window && i<length; i++) {
        sum += input[i];
    }
    
    for (int i = 0; i < length; i++) {
        if (i >= half_window && i + half_window < length) {
             output[i] = sum / window;
             // Slide window
             sum -= input[i - half_window];
             sum += input[i + half_window + 1];
        } else {
            output[i] = input[i]; // Edge case: copy raw
        }
    }
}

static int count_peaks(float *signal, int length, float min_height, int min_distance) {
    int peaks = 0;
    int last_peak = -min_distance;
    
    for (int i = 1; i < length - 1; i++) {
        if (signal[i] > signal[i-1] && signal[i] > signal[i+1]) {
            if (signal[i] > min_height && (i - last_peak) >= min_distance) {
                peaks++;
                last_peak = i;
            }
        }
    }
    return peaks;
}

// ============================================================
// INITIALIZATION
// ============================================================

bool tinyml_init(void) {
    ESP_LOGI(TAG, "Initializing TinyML Models...");

    // 1. Allocate Tensor Arena
    tensor_arena = (uint8_t*)heap_caps_malloc(TENSOR_ARENA_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (tensor_arena == nullptr) {
        tensor_arena = (uint8_t*)heap_caps_malloc(TENSOR_ARENA_SIZE, MALLOC_CAP_8BIT);
    }
    if (tensor_arena == nullptr) {
        ESP_LOGE(TAG, "Failed to allocate Tensor Arena!");
        return false;
    }

    // 2. Load Models
    // PLEASE VERIFY VARIABLE NAMES IN YOUR .H FILES
    model_hr = tflite::GetModel(g_denoise_model_data); 
    model_stress = tflite::GetModel(g_stress_model_data);
    model_resp = tflite::GetModel(g_respiration_model_data);

    if (model_hr->version() != TFLITE_SCHEMA_VERSION) {
        ESP_LOGE(TAG, "HR Model schema mismatch!"); return false;
    }
    if (model_stress->version() != TFLITE_SCHEMA_VERSION) {
        ESP_LOGE(TAG, "Stress Model schema mismatch!"); return false;
    }
    if (model_resp->version() != TFLITE_SCHEMA_VERSION) {
        ESP_LOGE(TAG, "Resp Model schema mismatch!"); return false;
    }

    // 3. Setup Resolver
    static tflite::MicroMutableOpResolver<20> local_resolver;
    // local_resolver.AddConv1D(); // Not available in this version, mapped to Conv2D often or just not needed if model is converted right
    local_resolver.AddConv2D();
    local_resolver.AddFullyConnected();
    local_resolver.AddSoftmax();
    local_resolver.AddReshape();
    local_resolver.AddMaxPool2D();
    local_resolver.AddAveragePool2D();
    local_resolver.AddRelu();
    local_resolver.AddConcatenation(); 
    local_resolver.AddResizeNearestNeighbor(); 
    local_resolver.AddTransposeConv();
    local_resolver.AddQuantize();
    local_resolver.AddDequantize();
    local_resolver.AddLogistic(); 
    local_resolver.AddAdd(); 
    local_resolver.AddPad();
    local_resolver.AddMean();

    resolver = &local_resolver;

    // 4. Build Interpreters (Static allocation objects)
    static tflite::MicroInterpreter static_interpreter_hr(model_hr, *resolver, tensor_arena, TENSOR_ARENA_SIZE);
    interpreter_hr = &static_interpreter_hr;

    static tflite::MicroInterpreter static_interpreter_stress(model_stress, *resolver, tensor_arena, TENSOR_ARENA_SIZE);
    interpreter_stress = &static_interpreter_stress;
    
    static tflite::MicroInterpreter static_interpreter_resp(model_resp, *resolver, tensor_arena, TENSOR_ARENA_SIZE);
    interpreter_resp = &static_interpreter_resp;

    ESP_LOGI(TAG, "TinyML Initialized.");
    return true;
}

// ============================================================
// INFERENCE
// ============================================================

int tinyml_predict_bpm(float *ppg_input) {
    if (interpreter_hr->AllocateTensors() != kTfLiteOk) return 0;

    TfLiteTensor* input = interpreter_hr->input(0);
    TfLiteTensor* output = interpreter_hr->output(0);

    // Pre-process & Quantize
    for (int i = 0; i < 1000; i++) {
        float val = ppg_input[i];
        int32_t q_val = val / input->params.scale + input->params.zero_point;
        if (q_val < -128) q_val = -128;
        if (q_val > 127) q_val = 127;
        input->data.int8[i] = (int8_t)q_val;
    }

    if (interpreter_hr->Invoke() != kTfLiteOk) return 0;

    // Post-process
    float max_val = 0.0f;
    float mask_prob[1000];
    
    for (int i = 0; i < 1000; i++) {
        float val = (output->data.int8[i] - output->params.zero_point) * output->params.scale;
        mask_prob[i] = val;
        if (val > max_val) max_val = val;
    }

    float threshold = (max_val * 0.3f > 0.1f) ? max_val * 0.3f : 0.1f;
    int peaks = count_peaks(mask_prob, 1000, threshold, 30);
    return peaks * 6;
}

int tinyml_predict_stress(float *ppg, int16_t *ax, int16_t *ay, int16_t *az) {
    if (interpreter_stress->AllocateTensors() != kTfLiteOk) return 0;

    TfLiteTensor* input = interpreter_stress->input(0);
    TfLiteTensor* output = interpreter_stress->output(0);

    // Input: (1, 1000, 4)
    int idx = 0;
    for (int i = 0; i < 1000; i++) {
        float f_ax = (float)ax[i] / 16384.0f; // Approx normalize
        float f_ay = (float)ay[i] / 16384.0f;
        float f_az = (float)az[i] / 16384.0f;
        
        // Order: PPG, X, Y, Z (Check your training notebook for order!)
        // Assuming PPG, X, Y, Z
        float vals[4] = {ppg[i], f_ax, f_ay, f_az};
        
        for(int c=0; c<4; c++){
             int32_t q = vals[c] / input->params.scale + input->params.zero_point;
             if (q < -128) {
                 q = -128;
             } 
             if (q > 127) {
                 q = 127;
             }
             input->data.int8[idx++] = (int8_t)q;
        }
    }

    if (interpreter_stress->Invoke() != kTfLiteOk) return 0;

    float p_stress = (output->data.int8[1] - output->params.zero_point) * output->params.scale;
    return (int)(p_stress * 100.0f);
}



static int calculate_rr_bpm(float *signal, int length, float fs) {



    // 1. Smoothing (Moving Average - Window 150 ~ 1.5s)



    // FIX TASK 4: Remove static, use PSRAM to ensure thread safety and avoid stack overflow



    float *smoothed = (float*)heap_caps_malloc(length * sizeof(float), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);



    if (smoothed == nullptr) return 0;







    smooth_signal(signal, smoothed, length, 150);







    // 2. Calculate Prominence Threshold (Range * 0.4)



    float min_val = 10000.0f;



    float max_val = -10000.0f;



    



    for (int i = 0; i < length; i++) {



        if (smoothed[i] < min_val) min_val = smoothed[i];



        if (smoothed[i] > max_val) max_val = smoothed[i];



    }



    



    float range_val = max_val - min_val;



    float prominence_threshold = range_val * 0.4f;



    // Approximate prominence check: Peak must be in the top 60% of the signal range



    float threshold = min_val + prominence_threshold;







    // 3. Find Peaks



    int peak_indices[50];



    int peak_count = 0;



    int min_distance = (int)(fs * 0.4f); // 0.4s = 40 samples @ 100Hz



    int last_peak = -min_distance;



    



    for (int i = 1; i < length - 1; i++) {



        // Local Maxima



        if (smoothed[i] > smoothed[i-1] && smoothed[i] > smoothed[i+1]) {



            // Prominence/Height Check



            if (smoothed[i] > threshold) {



                // Distance Check



                if ((i - last_peak) >= min_distance) {



                    if (peak_count < 50) {



                        peak_indices[peak_count++] = i;



                        last_peak = i;



                    }



                } else {



                    // If too close, keep the higher one



                    if (peak_count > 0) {



                        int prev_idx = peak_indices[peak_count - 1];



                        if (smoothed[i] > smoothed[prev_idx]) {



                            peak_indices[peak_count - 1] = i;



                            last_peak = i;



                        }



                    }



                }



            }



        }



    }



    



    free(smoothed); // Clean up







    if (peak_count < 2) return 0;



    



    // 4. Calculate BPM from Average Peak Distance



    float sum_dist = 0;



    for (int i = 0; i < peak_count - 1; i++) {



        sum_dist += (peak_indices[i+1] - peak_indices[i]);



    }



    float avg_dist = sum_dist / (peak_count - 1);



    



    if (avg_dist <= 0) return 0;



    



    return (int)((60.0f * fs) / avg_dist);



}



int tinyml_predict_respiration(float *ppg_input) {

    if (interpreter_resp->AllocateTensors() != kTfLiteOk) return 0;

    TfLiteTensor* input = interpreter_resp->input(0);

    TfLiteTensor* output = interpreter_resp->output(0);

    

    // Model expects 1500 samples, we have 1000. Zero-pad the rest.

    int model_len = input->dims->data[1]; 

    if (model_len == 0) model_len = 1500; // Fallback if dim not read

    

    for (int i = 0; i < model_len; i++) {

        float val = (i < 1000) ? ppg_input[i] : 0.0f; // Zero-pad

        int32_t q = val / input->params.scale + input->params.zero_point;

        if (q < -128) q = -128;

        if (q > 127) q = 127;

        input->data.int8[i] = (int8_t)q;

    }

    

    if (interpreter_resp->Invoke() != kTfLiteOk) return 0;



    float resp_wave[1000];

    // Only extract the first 1000 samples corresponding to real data

    for (int i = 0; i < 1000; i++) {

        resp_wave[i] = (output->data.int8[i] - output->params.zero_point) * output->params.scale;

    }

    

    // Use the new robust calculation function

    return calculate_rr_bpm(resp_wave, 1000, 100.0f);

}
