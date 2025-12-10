#include "algorithms.h"
#include "config.h"
#include "esp_log.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

// Fallback macros if config.h misses them
#ifndef PPG_SAMPLE_RATE_HZ
#define PPG_SAMPLE_RATE_HZ 100
#endif
#ifndef WINDOW_SIZE_SECONDS
#define WINDOW_SIZE_SECONDS 10
#endif

// ============================================================
// UTILITY FUNCTIONS
// ============================================================

void moving_average_filter(const float *input, float *output, int length, int window_size) {
    if (length <= 0 || window_size <= 0) return;

    for (int i = 0; i < length; i++) {
        float sum = 0.0f;
        int count = 0;
        
        for (int j = 0; j < window_size; j++) {
            if (i - j >= 0) {
                sum += input[i - j];
                count++;
            }
        }
        output[i] = (count > 0) ? (sum / count) : 0.0f;
    }
}

float calculate_mean(const float *buffer, int length) {
    if (length <= 0) return 0.0f;
    double sum = 0.0; // Use double for accumulation precision
    for (int i = 0; i < length; i++) {
        sum += buffer[i];
    }
    return (float)(sum / length);
}
    
float calculate_std(const float *buffer, int length) {
    if (length <= 1) return 0.0f;

    float mean = calculate_mean(buffer, length);
    double sum_sq = 0.0; // Use double
    
    for (int i = 0; i < length; i++) {
        float diff = buffer[i] - mean;
        sum_sq += diff * diff;
    }
    
    return (float)sqrt(sum_sq / length);
}
    
void z_score_normalize(float *buffer, int length) {
    if (length <= 0) return;

    float mean = calculate_mean(buffer, length);
    float std = calculate_std(buffer, length);

    if (std < 1e-6f) std = 1.0f; // Prevent div by zero

    for (int i = 0; i < length; i++) {
        buffer[i] = (buffer[i] - mean) / std;
    }
}
    
// ============================================================
// SpO2 CALCULATION
// ============================================================
   
int calculate_spo2(const uint32_t *red_buffer, const uint32_t *ir_buffer, int length) {
    if (length < 100) return -1;

    // Step 1: Calculate DC components
    double sum_red = 0.0, sum_ir = 0.0; // Double to prevent overflow
    for (int i = 0; i < length; i++) {
        sum_red += red_buffer[i];
        sum_ir += ir_buffer[i];
    }
    float dc_red = (float)(sum_red / length);
    float dc_ir = (float)(sum_ir / length);

    // Check for valid signal (finger detected?)
    if (dc_red < 10000.0f || dc_ir < 10000.0f) {
        return -1; 
    }
    
    // Step 2: Calculate AC components (RMS)
    double ac_red_sq = 0.0, ac_ir_sq = 0.0;
    for (int i = 0; i < length; i++) {
        float diff_red = (float)red_buffer[i] - dc_red;
        float diff_ir = (float)ir_buffer[i] - dc_ir;
        ac_red_sq += diff_red * diff_red;
        ac_ir_sq += diff_ir * diff_ir;
    }

    float ac_red = sqrtf((float)(ac_red_sq / length));
    float ac_ir = sqrtf((float)(ac_ir_sq / length));

    // Step 3: Ratio of Ratios
    if (dc_red < 1.0f || dc_ir < 1.0f || ac_ir < 0.1f) return -1;

    float R = (ac_red / dc_red) / (ac_ir / dc_ir);

    // Step 4: Calibration (Standard Maxim/TI approximation)
    // Formula: 110 - 25 * R
    float spo2_f = 110.0f - 25.0f * R;
    int spo2 = (int)spo2_f;

    // Clamp
    if (spo2 < 0) spo2 = 0; // Invalid
    if (spo2 > 100) spo2 = 100;
    if (spo2 < 50) return -1; // Usually noise if calculated < 50%

    return spo2;
}

// ============================================================
// HRV & STRESS ANALYSIS
// ============================================================
   
float calculate_sdnn(const float *rr_intervals, int count) {
    if (count < 2) return -1.0f;
    return calculate_std(rr_intervals, count);
}

float calculate_rmssd(const float *rr_intervals, int count) {
    if (count < 2) return -1.0f;
    
    double sum_sq_diff = 0.0;
    for (int i = 0; i < count - 1; i++) {
        float diff = rr_intervals[i+1] - rr_intervals[i];
        sum_sq_diff += diff * diff;
    }
    
    return (float)sqrt(sum_sq_diff / (count - 1));
}
   
int extract_rr_intervals(const float *ppg_signal, int hr_bpm,
                        float *rr_intervals, int max_count) {
    // Sanity check inputs
    if (hr_bpm < 40 || hr_bpm > 200) return 0;

    // Minimum distance between peaks in samples
    // E.g. 200 BPM -> 3.33 peaks/sec -> distance ~30 samples @ 100Hz
    int min_distance = (PPG_SAMPLE_RATE_HZ * 60) / 220; 

    // Find peaks
    int peak_indices[40]; 
    int peak_count = 0;
    float threshold = 0.5f; // Z-score normalized signal, peaks usually > 1.0
    int last_peak = -min_distance;

    for (int i = 1; i < WINDOW_SIZE_SAMPLES - 1 && peak_count < 40; i++) {
        if (ppg_signal[i] > ppg_signal[i-1] &&
            ppg_signal[i] > ppg_signal[i+1] &&
            ppg_signal[i] > threshold &&
            (i - last_peak) >= min_distance) {

            peak_indices[peak_count++] = i;
            last_peak = i;
        }
    }

    // Convert peaks to RR intervals (ms)
    int rr_count = 0;
    for (int i = 0; i < peak_count - 1 && rr_count < max_count; i++) {
        int dist_samples = peak_indices[i + 1] - peak_indices[i];
        float rr_ms = ((float)dist_samples / PPG_SAMPLE_RATE_HZ) * 1000.0f;

        // Filter physiological impossibilities (300ms = 200bpm, 1500ms = 40bpm)
        if (rr_ms >= 300.0f && rr_ms <= 1500.0f) {
            rr_intervals[rr_count++] = rr_ms;
        }
    }

    return rr_count;
}

int estimate_stress(float sdnn) {
    if (sdnn < 0.0f) return 0;
    // Simple heuristic mapping
    if (sdnn >= 100.0f) return 5;  // Very Relaxed
    if (sdnn <= 20.0f) return 95;  // High Stress

    // Linear mapping between 20ms(95) and 100ms(5)
    // Slope = (5 - 95) / (100 - 20) = -90 / 80 = -1.125
    int stress = 95 - (int)((sdnn - 20.0f) * 1.125f);
    
    if (stress < 0) stress = 0;
    if (stress > 100) stress = 100;
    return stress;
}
  
// ============================================================
// MOTION ARTIFACT DETECTION
// ============================================================
 
bool detect_motion_artifact(const int16_t *accel_x, const int16_t *accel_y,
                           const int16_t *accel_z, int length, int threshold) {
   if (length < 10) return false;
   
   // 1. Calculate Mean Magnitude
   double sum_mag = 0.0;
   float mags[length]; // VLA is okay for small length, but better static if large
   
   for (int i = 0; i < length; i++) {
       mags[i] = sqrtf((float)accel_x[i]*accel_x[i] + 
                       (float)accel_y[i]*accel_y[i] + 
                       (float)accel_z[i]*accel_z[i]);
       sum_mag += mags[i];
   }
   float mean_mag = (float)(sum_mag / length);

   // 2. Calculate Variance of Magnitude
   double sum_sq_diff = 0.0;
   for (int i = 0; i < length; i++) {
       float diff = mags[i] - mean_mag;
       sum_sq_diff += diff * diff;
   }
   float std_dev = sqrtf((float)(sum_sq_diff / length));

   return (std_dev > (float)threshold);
}

// ============================================================
// FREQUENCY DOMAIN ANALYSIS IMPLEMENTATION (ESP-DSP)
// ============================================================
#include "dsps_fft2r.h"
#include "dsps_wind.h"

bool resample_rr_intervals(const float *rr_intervals, int count, float *output_buffer) {
    if (count < 2) return false;
    // Note: FFT_SIZE is defined in header (now 64)
    if (count > FFT_SIZE) count = FFT_SIZE; 

    // Create time axis for RR intervals
    float time_axis[64]; // Match FFT_SIZE manually or use malloc
    time_axis[0] = 0.0f;
    for (int i = 1; i < count; i++) {
        time_axis[i] = time_axis[i-1] + rr_intervals[i-1];
    }

    float total_duration = time_axis[count-1];
    float sampling_interval = 1000.0f / HRV_SAMPLING_HZ; 

    // Linear Interpolation
    int rr_idx = 0;
    for (int i = 0; i < FFT_SIZE; i++) {
        float target_time = i * sampling_interval;
        
        if (target_time > total_duration) {
            output_buffer[i] = 0.0f; 
            continue;
        }

        while (rr_idx < count - 1 && time_axis[rr_idx+1] < target_time) {
            rr_idx++;
        }

        if (rr_idx < count - 1) {
            float t0 = time_axis[rr_idx];
            float t1 = time_axis[rr_idx+1];
            float v0 = rr_intervals[rr_idx];
            float v1 = rr_intervals[rr_idx+1];
            
            if (t1 - t0 > 0.001f) {
                float slope = (v1 - v0) / (t1 - t0);
                output_buffer[i] = v0 + slope * (target_time - t0);
            } else {
                output_buffer[i] = v0;
            }
        } else {
            output_buffer[i] = rr_intervals[count-1];
        }
    }
    
    // Detrend (Remove DC bias to focus on variations)
    float mean = calculate_mean(output_buffer, FFT_SIZE);
    for(int i=0; i<FFT_SIZE; i++) output_buffer[i] -= mean;

    return true;
}

float calculate_lf_hf_ratio(float *signal_4hz) {
    // FFT Buffer: Real + Imaginary interleaved
    float fft_buffer[FFT_SIZE * 2]; 
    
    // 1. Windowing
    float wind[FFT_SIZE];
    dsps_wind_hann_f32(wind, FFT_SIZE);

    for (int i = 0; i < FFT_SIZE; i++) {
        fft_buffer[i * 2 + 0] = signal_4hz[i] * wind[i]; // Real
        fft_buffer[i * 2 + 1] = 0.0f;                    // Imag
    }

    // 2. Compute FFT
    dsps_fft2r_fc32(fft_buffer, FFT_SIZE);
    dsps_bit_rev_fc32(fft_buffer, FFT_SIZE);

    // 3. Power Spectrum Integration
    float bin_width = (float)HRV_SAMPLING_HZ / FFT_SIZE; // 4.0 / 64 = 0.0625 Hz
    float lf_power = 0.0f;
    float hf_power = 0.0f;

    // Start from i=1 to skip DC
    for (int i = 1; i < FFT_SIZE / 2; i++) {
        float r = fft_buffer[i * 2 + 0];
        float im = fft_buffer[i * 2 + 1];
        float power = r * r + im * im;
        float freq = i * bin_width;

        if (freq >= 0.04f && freq < 0.15f) {
            lf_power += power;
        } else if (freq >= 0.15f && freq < 0.4f) {
            hf_power += power;
        }
    }

    if (hf_power < 0.00001f) return 1.0f; // Avoid Div/0
    return lf_power / hf_power;
}

// ============================================================
// SIGNAL QUALITY ESTIMATION
// ============================================================
float calculate_signal_quality_index(const float *buffer, int length) {
    if (length < 10) return 0.0f;

    // 1. Calculate Standard Deviation (Signal Strength)
    float mean = calculate_mean(buffer, length);
    float std = calculate_std(buffer, length);

    // Flatline Check: If signal is extremely flat, it's noise or sensor off
    // Threshold lowered to 0.0001 to be very permissive
    if (std < 0.0001f) return 0.0f; 

    // 2. Calculate Skewness (Signal Asymmetry)
    // PPG pulses are asymmetric. Noise is often symmetric (Gaussian).
    double sum_skew = 0.0;
    for (int i = 0; i < length; i++) {
        float z = (buffer[i] - mean) / std;
        sum_skew += z * z * z;
    }
    float skewness = (float)(sum_skew / length);
    
    // [FIX] Use Absolute Skewness. 
    // Inverted sensor -> Negative Skew. Normal -> Positive Skew. Both are valid.
    float abs_skew = fabsf(skewness);

    // 3. Scoring Logic
    float score = 0.0f;
    
    // Amplitude Score (0-50): Is the pulse strong enough?
    // Typical normalized PPG has std dev around 1.0 (due to z-score), 
    // but before normalization it varies. Assuming input is LMS output (small amplitude).
    // Let's assume input is somewhat normalized or reasonable range.
    if (std > 0.01f) score += 40.0f;
    else score += (std * 4000.0f); // Linear ramp up for weak signals

    // Skewness Score (0-50): Is it shaped like a pulse?
    // Good PPG usually has |skew| > 0.5
    if (abs_skew > 0.8f) {
        score += 50.0f;
    } else if (abs_skew > 0.2f) {
        score += (abs_skew * 50.0f); // 0.2 -> 10pts, 0.8 -> 40pts
    } else {
        score += 0.0f; // Very Gaussian noise
    }
    
    if (score > 100.0f) score = 100.0f;
    return score;
}