#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================
// TINYML MODEL MANAGEMENT
// ============================================================

/**
 * @brief Initialize all TinyML models (Denoise, Stress, Respiration)
 * Allocates Tensor Arena and sets up Interpreters.
 * @return true if successful, false otherwise
 */
bool tinyml_init(void);

/**
 * @brief Run Denoise Model Inference (Heart Rate Estimation)
 * @param ppg_input Normalized PPG signal (1000 samples)
 * @return Estimated Heart Rate (BPM)
 */
int tinyml_predict_bpm(float *ppg_input);

/**
 * @brief Run Stress Model Inference (CNN)
 * @param ppg Normalized PPG signal (1000 samples)
 * @param ax Accelerometer X (1000 samples)
 * @param ay Accelerometer Y (1000 samples)
 * @param az Accelerometer Z (1000 samples)
 * @return Stress Level Probability (0-100)
 */
int tinyml_predict_stress(float *ppg, int16_t *ax, int16_t *ay, int16_t *az);

/**
 * @brief Run Respiration Model Inference
 * @param ppg_input Normalized PPG signal (1000 samples)
 * @return Respiration Rate (BPM)
 */
int tinyml_predict_respiration(float *ppg_input);

#ifdef __cplusplus
}
#endif
