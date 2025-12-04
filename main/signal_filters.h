#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

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
    iir_section_t section1; // High-pass section (approx)
    iir_section_t section2; // Low-pass section (approx)
    float gain;
} ppg_filter_t;

void ppg_filter_init(ppg_filter_t *filter);
float ppg_filter_process(ppg_filter_t *filter, float input);

// ============================================================
// KALMAN FILTER (1D - For Accelerometer Smoothing)
// ============================================================

typedef struct {
    float q; // Process noise covariance
    float r; // Measurement noise covariance
    float x; // Value
    float p; // Estimation error covariance
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
// LMS ADAPTIVE FILTER (Breakthrough 1: Noise Cancellation)
// ============================================================
// Uses Accelerometer Z-axis as noise reference to clean PPG

#define LMS_FILTER_TAPS 32 

typedef struct {
    float weights[LMS_FILTER_TAPS]; // Learned weights
    float buffer[LMS_FILTER_TAPS];  // Noise reference history
    float mu;                       // Learning rate
    int head;                       // Circular buffer index
} lms_filter_t;

/**
 * @brief Initialize LMS Adaptive Filter
 * @param learning_rate Step size (mu). Try 0.01 to 0.05
 */
void lms_filter_init(lms_filter_t *lms, float learning_rate);

/**
 * @brief Update LMS Filter
 * @param input_signal The noisy signal (PPG Raw)
 * @param noise_ref The reference noise (Accel Z)
 * @return The CLEANED signal (Error term)
 */
float lms_filter_update(lms_filter_t *lms, float input_signal, float noise_ref);