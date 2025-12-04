#pragma once

#include <stdint.h>
#include <stdbool.h>

/**
 * ============================================================
 * SIGNAL PROCESSING ALGORITHMS FOR HEALTH MONITORING
 * ============================================================
 */

// ============================================================
// SpO2 CALCULATION (Ratio of Ratios Method)
// ============================================================

/**
 * @brief Calculate SpO2 from Red and IR PPG signals
 * 
 * Uses the ratio-of-ratios method:
 * R = (AC_red/DC_red) / (AC_ir/DC_ir)
 * SpO2 = 110 - 25*R (empirical calibration)
 * 
 * @param red_buffer Array of Red LED samples (1000 samples @ 100Hz)
 * @param ir_buffer Array of IR LED samples (1000 samples @ 100Hz)
 * @param length Number of samples in each buffer
 * @return SpO2 percentage (70-100), or -1 if invalid
 */
int calculate_spo2(const uint32_t *red_buffer, const uint32_t *ir_buffer, int length);

// ============================================================
// HRV (HEART RATE VARIABILITY) ANALYSIS
// ============================================================

/**
 * @brief Calculate SDNN (Standard Deviation of NN intervals)
 * 
 * SDNN is a time-domain measure of HRV. Higher SDNN indicates
 * better cardiovascular health and lower stress.
 * 
 * @param rr_intervals Array of RR intervals in milliseconds
 * @param count Number of RR intervals
 * @return SDNN in milliseconds, or -1.0f if invalid
 */
float calculate_sdnn(const float *rr_intervals, int count);

/**
 * @brief Calculate RMSSD (Root Mean Square of Successive Differences)
 * 
 * RMSSD is a time-domain measure of short-term HRV variability.
 * 
 * @param rr_intervals Array of RR intervals in milliseconds
 * @param count Number of RR intervals
 * @return RMSSD in milliseconds, or -1.0f if invalid
 */
float calculate_rmssd(const float *rr_intervals, int count);

/**
 * @brief Extract RR intervals from HR peaks detected by TinyML model
 * 
 * This function should be called after HR estimation to extract
 * RR intervals for HRV analysis.
 * 
 * @param ppg_signal PPG signal buffer (1000 samples)
 * @param hr_bpm Detected heart rate in BPM
 * @param rr_intervals Output array for RR intervals (max 20 intervals)
 * @param max_count Maximum number of intervals to extract
 * @return Number of RR intervals extracted
 */
int extract_rr_intervals(const float *ppg_signal, int hr_bpm, 
                        float *rr_intervals, int max_count);

/**
 * @brief Estimate stress level based on HRV (SDNN)
 * 
 * @param sdnn Standard Deviation of NN intervals (ms)
 * @return Stress level (0-100), where higher values indicate higher stress
 */
int estimate_stress(float sdnn);

// ============================================================
// BREAKTHROUGH 3: FREQUENCY DOMAIN ANALYSIS (FFT)
// ============================================================
#define FFT_SIZE 64       // Power of 2 (Need ~16 seconds of data at 4Hz)
#define HRV_SAMPLING_HZ 4 // Standard for HRV interpolation

/**
 * @brief Resample irregular RR intervals to fixed sampling rate for FFT
 * @param rr_intervals Array of RR intervals in ms
 * @param count Number of intervals
 * @param output_buffer Buffer to store resampled data (size must be FFT_SIZE)
 * @return true if successful
 */
bool resample_rr_intervals(const float *rr_intervals, int count, float *output_buffer);

/**
 * @brief Perform FFT and calculate LF/HF Ratio
 * LF (Low Freq): 0.04Hz - 0.15Hz (Stress/Sympathetic)
 * HF (High Freq): 0.15Hz - 0.4Hz (Relax/Parasympathetic)
 * @param signal_4hz Fixed rate signal (4Hz)
 * @return LF/HF Ratio (Higher = More Stress)
 */
float calculate_lf_hf_ratio(float *signal_4hz);

// ============================================================
// MOTION ARTIFACT DETECTION
// ============================================================

/**
 * @brief Detect motion artifacts from accelerometer data
 * 
 * Returns true if significant motion is detected, which indicates
 * that the current PPG reading may be unreliable.
 * 
 * @param accel_x Array of X-axis acceleration samples
 * @param accel_y Array of Y-axis acceleration samples
 * @param accel_z Array of Z-axis acceleration samples
 * @param length Number of samples
 * @param threshold Motion detection threshold
 * @return true if motion detected, false otherwise
 */
bool detect_motion_artifact(const int16_t *accel_x, const int16_t *accel_y, 
                           const int16_t *accel_z, int length, int threshold);

// ============================================================
// UTILITY FUNCTIONS
// ============================================================

/**
 * @brief Z-score normalization for signal preprocessing
 * 
 * Normalizes buffer to have zero mean and unit variance.
 * 
 * @param buffer Input/output signal buffer
 * @param length Buffer length
 */
void z_score_normalize(float *buffer, int length);

/**
 * @brief Simple moving average filter for DC component extraction
 * 
 * @param input Input signal
 * @param output Output filtered signal
 * @param length Signal length
 * @param window_size Moving average window size
 */
void moving_average_filter(const float *input, float *output, int length, int window_size);

/**
 * @brief Calculate mean of a signal
 */
float calculate_mean(const float *buffer, int length);

/**
 * @brief Calculate standard deviation of a signal
 */
float calculate_std(const float *buffer, int length);
