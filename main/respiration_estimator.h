#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the respiration rate estimation model
 * 
 * This function should be called once during system initialization.
 * Currently uses a signal processing approach, but can be replaced
 * with TinyML model when available.
 */
void respiration_model_init(void);

/**
 * @brief Estimate respiration rate from PPG signal
 * 
 * This function analyzes the baseline wander of the PPG signal
 * (typically 0.1-0.5 Hz) to estimate breathing rate.
 * 
 * @param ppg_signal PPG signal buffer (1000 samples @ 100Hz, normalized)
 * @param workspace_buffer Pre-allocated scratch buffer of size WINDOW_SIZE_SAMPLES (float)
 * @return Respiration rate in breaths/min (8-30 range), or -1 if invalid
 */
int respiration_model_predict(float *ppg_signal, float *workspace_buffer);

#ifdef __cplusplus
}
#endif
