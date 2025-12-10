#include "tinyml_manager.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include <math.h>
#include <string.h>
#include <inttypes.h>

// TensorFlow Lite Micro Headers
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/system_setup.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/micro/micro_op_resolver.h"

// Kernel gốc
#include "tensorflow/lite/micro/kernels/micro_ops.h" 

// Headers xử lý tín hiệu
#include "signal_filters.h" 

// Model Headers
#include "denoise_model.h"     // Heart Rate
#include "stress_model.h"      // Stress
#include "respiration_model.h" // Respiration

static const char *TAG = "TinyML";

// ============================================================
// CONFIGURATION (MEMORY ARENA SIZES)
// ============================================================
// [UPDATED] Tăng size dựa trên yêu cầu thực tế từ Log lỗi
// HR cần ~141KB -> Cấp 160KB cho an toàn
#define ARENA_SIZE_HR     (160 * 1024)  
// Stress model nhẹ -> Cấp 25KB
#define ARENA_SIZE_STRESS (25 * 1024)  
// Resp cần ~212KB -> Cấp 240KB cho an toàn
#define ARENA_SIZE_RESP   (240 * 1024)  

static uint8_t *tensor_arena_hr = nullptr;
static uint8_t *tensor_arena_stress = nullptr;
static uint8_t *tensor_arena_resp = nullptr;

// Interpreters
static tflite::MicroInterpreter *interpreter_hr = nullptr;
static tflite::MicroInterpreter *interpreter_stress = nullptr;
static tflite::MicroInterpreter *interpreter_resp = nullptr;

// Models
const tflite::Model* model_hr = nullptr;
const tflite::Model* model_stress = nullptr;
const tflite::Model* model_resp = nullptr;

// Resolver
static tflite::MicroOpResolver *resolver = nullptr; 

// ============================================================
// HELPER: MEMORY ALLOCATOR (Smart Alloc)
// ============================================================
static uint8_t* allocate_arena(size_t size) {
    // 1. Thử cấp phát trong PSRAM (RAM ngoài) trước vì Arena rất lớn
    uint8_t* ptr = (uint8_t*)heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    
    if (ptr) {
        ESP_LOGI(TAG, "Allocated %d bytes in PSRAM", size);
        return ptr;
    }

    // 2. Nếu không có PSRAM, buộc dùng RAM nội (Internal SRAM)
    ESP_LOGW(TAG, "PSRAM not available/full. Trying Internal RAM...");
    ptr = (uint8_t*)heap_caps_malloc(size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    
    if (ptr) {
        ESP_LOGI(TAG, "Allocated %d bytes in Internal RAM", size);
    } else {
        ESP_LOGE(TAG, "FAILED to allocate %d bytes!", size);
    }
    return ptr;
}

static void log_model_ops(const tflite::Model* model, const char* name) {
    if (!model || !model->subgraphs() || model->subgraphs()->size() == 0) return;
    // Chỉ in số lượng để đỡ rác log
    auto* sub = model->subgraphs()->Get(0);
    auto* ops = sub->operators();
    ESP_LOGI(TAG, "%s Model: %d operators", name, int(ops ? ops->size() : 0));
}

// ============================================================
// INITIALIZATION
// ============================================================

bool tinyml_init(void) {
    ESP_LOGI(TAG, "Initializing TinyML Models...");
    tflite::InitializeTarget();

    // 1. Allocate Arenas
    tensor_arena_hr = allocate_arena(ARENA_SIZE_HR);
    tensor_arena_stress = allocate_arena(ARENA_SIZE_STRESS);
    tensor_arena_resp = allocate_arena(ARENA_SIZE_RESP);

    // Nếu thiếu bất kỳ bộ nhớ nào -> Dừng ngay
    if (!tensor_arena_hr || !tensor_arena_stress || !tensor_arena_resp) {
        ESP_LOGE(TAG, "Critical Memory Error: Cannot allocate Tensor Arenas");
        return false;
    }

    // 2. Load Models
    model_hr = tflite::GetModel(g_denoise_model_data);
    model_stress = tflite::GetModel(g_stress_model_data);
    model_resp = tflite::GetModel(g_respiration_model_data);
    
    if (!model_hr || !model_stress || !model_resp) { ESP_LOGE(TAG, "Model buffer invalid"); return false; }
    if (model_hr->version() != TFLITE_SCHEMA_VERSION) { ESP_LOGE(TAG, "HR schema mismatch"); return false; }
    if (model_stress->version() != TFLITE_SCHEMA_VERSION) { ESP_LOGE(TAG, "Stress schema mismatch"); return false; }
    if (model_resp->version() != TFLITE_SCHEMA_VERSION) { ESP_LOGE(TAG, "Resp schema mismatch"); return false; }
    
    log_model_ops(model_hr, "HR");
    log_model_ops(model_stress, "Stress");
    log_model_ops(model_resp, "Resp");

    // 3. Setup Resolver
    static tflite::MicroMutableOpResolver<45> local_resolver; 
    
    // Critical Ops for U-Net
    local_resolver.AddPad();            
    local_resolver.AddTransposeConv();  
    local_resolver.AddShape();          
    local_resolver.AddReshape();
    local_resolver.AddConcatenation();
    local_resolver.AddConv2D();
    local_resolver.AddStridedSlice();
    local_resolver.AddPack();
    local_resolver.AddUnpack();
    
    // Standard Ops
    local_resolver.AddAdd();
    local_resolver.AddSub();
    local_resolver.AddMul();
    local_resolver.AddDiv();
    local_resolver.AddMean();
    local_resolver.AddFullyConnected();
    local_resolver.AddSoftmax();
    local_resolver.AddLogistic(); // Sigmoid
    local_resolver.AddRelu();
    local_resolver.AddMaxPool2D();
    local_resolver.AddAveragePool2D();
    local_resolver.AddDepthwiseConv2D();
    local_resolver.AddQuantize();
    local_resolver.AddDequantize();
    local_resolver.AddExpandDims();
    local_resolver.AddSqueeze();
    local_resolver.AddSplit();
    local_resolver.AddSplitV();
    local_resolver.AddResizeNearestNeighbor();

    resolver = &local_resolver;

    // 4. Build Interpreters
    static tflite::MicroInterpreter static_interpreter_hr(model_hr, *resolver, tensor_arena_hr, ARENA_SIZE_HR);
    interpreter_hr = &static_interpreter_hr;

    static tflite::MicroInterpreter static_interpreter_stress(model_stress, *resolver, tensor_arena_stress, ARENA_SIZE_STRESS);
    interpreter_stress = &static_interpreter_stress;
    
    static tflite::MicroInterpreter static_interpreter_resp(model_resp, *resolver, tensor_arena_resp, ARENA_SIZE_RESP);
    interpreter_resp = &static_interpreter_resp;
    
    // 5. Allocate Tensors (Và kiểm tra kỹ lưỡng)
    bool init_success = true;

    if (interpreter_hr->AllocateTensors() != kTfLiteOk) { 
        ESP_LOGE(TAG, "HR Model: Alloc Failed! (Arena too small?)");
        interpreter_hr = nullptr;
        init_success = false; 
    } else {
        ESP_LOGI(TAG, "HR Model: Ready");
    }

    if (interpreter_stress->AllocateTensors() != kTfLiteOk) { 
        ESP_LOGE(TAG, "Stress Model: Alloc Failed!"); 
        interpreter_stress = nullptr;
        init_success = false;
    } else {
        ESP_LOGI(TAG, "Stress Model: Ready");
    }

    if (interpreter_resp->AllocateTensors() != kTfLiteOk) { 
        ESP_LOGE(TAG, "Resp Model: Alloc Failed! (Arena too small?)"); 
        interpreter_resp = nullptr;
        init_success = false;
    } else {
        ESP_LOGI(TAG, "Resp Model: Ready");
    }

    // Nếu có bất kỳ lỗi nào, trả về false để Main báo lỗi
    if (!init_success) {
        ESP_LOGE(TAG, "TinyML Init FAILED due to memory allocation errors.");
        return false;
    }

    ESP_LOGI(TAG, "TinyML Init Success. Heap free: %lu bytes", (unsigned long)heap_caps_get_free_size(MALLOC_CAP_8BIT));
    return true;
}

// ============================================================
// INFERENCE FUNCTIONS
// ============================================================

// 1. HR Prediction
int tinyml_predict_bpm(float *ppg_input) {
    if (!interpreter_hr) return 0;
    TfLiteTensor* input = interpreter_hr->input(0);
    TfLiteTensor* output = interpreter_hr->output(0);

    for (int i = 0; i < 1000; i++) {
        float val = ppg_input[i];
        int32_t q_val = val / input->params.scale + input->params.zero_point;
        if (q_val < -128) q_val = -128; else if (q_val > 127) q_val = 127;
        input->data.int8[i] = (int8_t)q_val;
    }

    if (interpreter_hr->Invoke() != kTfLiteOk) return 0;

    // Use shared buffer or allocate small temp
    float max_val = 0.0f;
    // Không allocate mảng 1000 float trên stack (4KB) để tránh tràn stack
    // Dùng static hoặc xử lý trực tiếp
    int count = (output->bytes > 1000) ? 1000 : output->bytes;
    
    // [Optimization] Tính toán trực tiếp không cần mảng trung gian
    // Dùng hàm từ signal_filters.h thì cần mảng, ở đây ta làm đơn giản
    // Hoặc cấp phát heap
    float *mask_prob = (float*)heap_caps_malloc(count * sizeof(float), MALLOC_CAP_INTERNAL);
    if (!mask_prob) return 0;

    for (int i = 0; i < count; i++) {
        float val = (output->data.int8[i] - output->params.zero_point) * output->params.scale;
        mask_prob[i] = val;
        if (val > max_val) max_val = val;
    }
    
    float threshold = (max_val * 0.3f > 0.1f) ? max_val * 0.3f : 0.1f;
    int peaks = signal_utils_count_peaks(mask_prob, count, threshold, 30);
    
    free(mask_prob);
    return peaks * 6;
}

// 2. Stress Prediction
int tinyml_predict_stress(float *ppg, int16_t *ax, int16_t *ay, int16_t *az) {
    if (!interpreter_stress) return 0;
    TfLiteTensor* input = interpreter_stress->input(0);
    TfLiteTensor* output = interpreter_stress->output(0);

    int idx = 0;
    for (int i = 0; i < 1000; i++) {
        float f_ax = (float)ax[i] / 16384.0f; 
        float f_ay = (float)ay[i] / 16384.0f;
        float f_az = (float)az[i] / 16384.0f;
        
        float vals[4] = {ppg[i], f_ax, f_ay, f_az};
        for(int c=0; c<4; c++){
             int32_t q = vals[c] / input->params.scale + input->params.zero_point;
             if (q < -128) q = -128; else if (q > 127) q = 127;
             input->data.int8[idx++] = (int8_t)q;
        }
    }

    if (interpreter_stress->Invoke() != kTfLiteOk) return 0;

    float p_stress = (output->data.int8[1] - output->params.zero_point) * output->params.scale;
    return (int)(p_stress * 100.0f);
}

// 3. Respiration Prediction
// Helper tính BPM cho Resp
static int calculate_resp_bpm(float *signal, int length, float fs) {
    float *smoothed = (float*)heap_caps_malloc(length * sizeof(float), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!smoothed) return 0;

    signal_utils_smooth(signal, smoothed, length, 150); 

    float min_val = 10000.0f, max_val = -10000.0f;
    for (int i = 0; i < length; i++) {
        if (smoothed[i] < min_val) min_val = smoothed[i];
        if (smoothed[i] > max_val) max_val = smoothed[i];
    }
    
    float threshold = min_val + (max_val - min_val) * 0.4f;
    int peak_count = signal_utils_count_peaks(smoothed, length, threshold, (int)(fs * 0.4f));
    
    free(smoothed);
    
    if (peak_count < 2) return 0;
    float duration_min = (float)length / fs / 60.0f;
    return (int)(peak_count / duration_min);
}

int tinyml_predict_respiration(float *ppg_input) {
    if (!interpreter_resp) return 0;
    TfLiteTensor* input = interpreter_resp->input(0);
    TfLiteTensor* output = interpreter_resp->output(0);

    // Dynamic Input Shape
    int model_len = 1500;
    if (input->dims->size >= 2) model_len = input->dims->data[1];
    
    for (int i = 0; i < model_len; i++) {
        float val = (i < 1000) ? ppg_input[i] : 0.0f;
        int32_t q = val / input->params.scale + input->params.zero_point;
        if (q < -128) q = -128; else if (q > 127) q = 127;
        input->data.int8[i] = (int8_t)q;
    }

    if (interpreter_resp->Invoke() != kTfLiteOk) return 0;

    float *resp_wave = (float*)heap_caps_malloc(model_len * sizeof(float), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!resp_wave) return 0;

    for (int i = 0; i < model_len; i++) {
        resp_wave[i] = (output->data.int8[i] - output->params.zero_point) * output->params.scale;
    }
    
    int rr = calculate_resp_bpm(resp_wave, model_len, 100.0f);
    free(resp_wave);
    
    return rr;
}