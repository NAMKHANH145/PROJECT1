#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#ifdef __cplusplus
extern "C" {
#endif
// ============================================================
// IIR BANDPASS FILTER (For PPG - Butterworth 2nd Order)
// ============================================================
// Fs = 100Hz, Low Cut = 0.5Hz, High Cut = 4.0Hz

typedef struct {
    float b0, b1, b2;
    float a1, a2;
    float w1, w2; // Delay lines (Direct Form II)
} iir_section_t;

typedef struct {
    iir_section_t section1; // High-pass section
    iir_section_t section2; // Low-pass section
    float gain;
} ppg_filter_t;

void ppg_filter_init(ppg_filter_t *filter);
float ppg_filter_process(ppg_filter_t *filter, float input);

// ============================================================
// KALMAN FILTER (1D - For Accelerometer Smoothing)
// ============================================================

typedef struct {
    float q; // Process noise
    float r; // Measurement noise
    float x; // Value
    float p; // Error covariance
    float k; // Kalman gain
} kalman_filter_t;

void kalman_filter_init(kalman_filter_t *kf, float q, float r, float initial_value);
float kalman_filter_update(kalman_filter_t *kf, float measurement);

// ============================================================
// MOVING AVERAGE FILTER (For Temperature)
// ============================================================

#define TEMP_FILTER_WINDOW 10

typedef struct {
    float buffer[TEMP_FILTER_WINDOW];
    int index;
    int count;
    float sum;
} temp_filter_t;

void temp_filter_init(temp_filter_t *filter);
float temp_filter_process(temp_filter_t *filter, float input);

// ============================================================
// LMS ADAPTIVE FILTER (Noise Cancellation)
// ============================================================

#define LMS_FILTER_TAPS 32 

typedef struct {
    float weights[LMS_FILTER_TAPS]; 
    float buffer[LMS_FILTER_TAPS];  
    float mu;                       
    int head;                       
} lms_filter_t;

void lms_filter_init(lms_filter_t *lms, float learning_rate);
float lms_filter_update(lms_filter_t *lms, float input_signal, float noise_ref);

// ============================================================
// [NEW] SIGNAL UTILITIES (Post-processing for TinyML)
// ============================================================

/**
 * @brief Smooth an array of signal data using Moving Average
 * @param input Input array
 * @param output Output array (can be same as input)
 * @param length Array length
 * @param window Window size
 */
void signal_utils_smooth(float *input, float *output, int length, int window);

/**
 * @brief Count peaks in a signal array
 * @param signal Input array
 * @param length Array length
 * @param min_height Minimum height threshold
 * @param min_distance Minimum distance between peaks (in samples)
 * @return Number of peaks found
 */
int signal_utils_count_peaks(float *signal, int length, float min_height, int min_distance);
// [QUAN TRỌNG] Đóng ngoặc extern "C"
#ifdef __cplusplus
}
#endif