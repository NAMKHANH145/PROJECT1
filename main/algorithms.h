#pragma once

#include <stdint.h>
#include <stdbool.h>

/**
 * ============================================================
 * SIGNAL PROCESSING ALGORITHMS FOR HEALTH MONITORING
 * ============================================================
 */

// ============================================================
// SpO2 CALCULATION
// ============================================================
int calculate_spo2(const uint32_t *red_buffer, const uint32_t *ir_buffer, int length);

// ============================================================
// HRV ANALYSIS
// ============================================================
float calculate_sdnn(const float *rr_intervals, int count);
float calculate_rmssd(const float *rr_intervals, int count);
int extract_rr_intervals(const float *ppg_signal, int hr_bpm, 
                         float *rr_intervals, int max_count);
int estimate_stress(float sdnn);

// ============================================================
// FREQUENCY DOMAIN ANALYSIS (FFT)
// ============================================================
// [CHANGED] Increased to 64 for better resolution (Bin width = 0.0625 Hz)
// 64 * 4 bytes = 256 bytes (Negligible RAM usage)
#define FFT_SIZE 64       
#define HRV_SAMPLING_HZ 4 

bool resample_rr_intervals(const float *rr_intervals, int count, float *output_buffer);
float calculate_lf_hf_ratio(float *signal_4hz);

// ============================================================
// MOTION & QUALITY
// ============================================================
bool detect_motion_artifact(const int16_t *accel_x, const int16_t *accel_y, 
                            const int16_t *accel_z, int length, int threshold);

float calculate_signal_quality_index(const float *buffer, int length);

// ============================================================
// UTILITY
// ============================================================
void z_score_normalize(float *buffer, int length);
void moving_average_filter(const float *input, float *output, int length, int window_size);
float calculate_mean(const float *buffer, int length);
float calculate_std(const float *buffer, int length);