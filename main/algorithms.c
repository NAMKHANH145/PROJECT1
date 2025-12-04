 #include "algorithms.h"
      #include "config.h"
      #include <math.h>
      #include <string.h>
      #include <stdlib.h>
     
      // ============================================================
    // UTILITY FUNCTIONS
    // ============================================================

    void moving_average_filter(const float *input, float *output, int length, int window_size) {
        if (length <= 0 || window_size <= 0) return;

        for (int i = 0; i < length; i++) {
            float sum = 0.0f;
            int count = 0;
            
            // Causal moving average: sum current and previous (window_size - 1) samples
            for (int j = 0; j < window_size; j++) {
                if (i - j >= 0) {
                    sum += input[i - j];
                    count++;
                }
            }
            
            if (count > 0) {
                output[i] = sum / count;
            } else {
                output[i] = 0.0f;
            }
        }
    }

    float calculate_mean(const float *buffer, int length) {
        if (length <= 0) return 0.0f;

        float sum = 0.0f;
        for (int i = 0; i < length; i++) {
            sum += buffer[i];
        }
        return sum / length;
    }
    
    float calculate_std(const float *buffer, int length) {
        if (length <= 1) return 0.0f;

        float mean = calculate_mean(buffer, length);
        float sum_sq = 0.0f;
    
        for (int i = 0; i < length; i++) {
            float diff = buffer[i] - mean;
            sum_sq += diff * diff;
        }
    
        return sqrtf(sum_sq / length);
    }
    
    void z_score_normalize(float *buffer, int length) {
        if (length <= 0) return;

        float mean = calculate_mean(buffer, length);
        float std = calculate_std(buffer, length);

        // Prevent division by zero
        if (std < 1e-6f) std = 1.0f;

        for (int i = 0; i < length; i++) {
            buffer[i] = (buffer[i] - mean) / std;
        }
    }
    
    // ============================================================
    // SpO2 CALCULATION
    // ============================================================
   
    int calculate_spo2(const uint32_t *red_buffer, const uint32_t *ir_buffer, int length) {
        if (length < 100) return -1; // Need at least 100 samples

        // Step 1: Calculate DC components (mean)
        float dc_red = 0.0f, dc_ir = 0.0f;
        for (int i = 0; i < length; i++) {
            dc_red += (float)red_buffer[i];
            dc_ir += (float)ir_buffer[i];
        }
        dc_red /= length;
        dc_ir /= length;
    
        // Check for valid DC values
        if (dc_red < 10000.0f || dc_ir < 10000.0f) {
            return -1; // Signal too weak
        }
        
         // Step 2: Calculate AC components (standard deviation)
         float ac_red_sq = 0.0f, ac_ir_sq = 0.0f;
         for (int i = 0; i < length; i++) {
             float diff_red = (float)red_buffer[i] - dc_red;
             float diff_ir = (float)ir_buffer[i] - dc_ir;
             ac_red_sq += diff_red * diff_red;
             ac_ir_sq += diff_ir * diff_ir;
         }
 
         float ac_red = sqrtf(ac_red_sq / length);
         float ac_ir = sqrtf(ac_ir_sq / length);
 
         // Step 3: Calculate ratio of ratios (R)
         // R = (AC_red/DC_red) / (AC_ir/DC_ir)
         if (dc_red < 1.0f || dc_ir < 1.0f || ac_ir < 1.0f) {
             return -1; // Avoid division by zero
         }
 
         float ratio_red = ac_red / dc_red;
         float ratio_ir = ac_ir / dc_ir;
         if (ratio_ir < 0.001f) {
             return -1; // Invalid ratio
         }
 
         float R = ratio_red / ratio_ir;
 
         // Step 4: Apply empirical calibration formula
         // SpO2 = 110 - 25*R (typical calibration, may need adjustment)
         int spo2 = (int)(110.0f - 25.0f * R);
         // Step 5: Clamp to valid range
         if (spo2 < SPO2_VALID_RANGE_MIN) spo2 = SPO2_VALID_RANGE_MIN;
         if (spo2 > SPO2_VALID_RANGE_MAX) spo2 = SPO2_VALID_RANGE_MAX;

         return spo2;
    }

    // ============================================================
    // HRV & STRESS ANALYSIS
    // ============================================================
   
    float calculate_sdnn(const float *rr_intervals, int count) {
        if (count < 2) return -1.0f;
   
        float mean = calculate_mean(rr_intervals, count);
        float sum_sq = 0.0f;
   
        for (int i = 0; i < count; i++) {
            float diff = rr_intervals[i] - mean;
            sum_sq += diff * diff;
        }
 
        return sqrtf(sum_sq / count);
    }
   
    int extract_rr_intervals(const float *ppg_signal, int hr_bpm,
                            float *rr_intervals, int max_count) {
        if (hr_bpm < HR_VALID_RANGE_MIN || hr_bpm > HR_VALID_RANGE_MAX) {
            return 0; // Invalid HR
        }

        // Expected number of peaks in 10 seconds
        int expected_peaks = (hr_bpm * WINDOW_SIZE_SECONDS) / 60;
        if (expected_peaks < 2) return 0;

        // Simple peak detection on normalized PPG signal
        int peak_indices[30]; // Max 30 peaks in 10 seconds
        int peak_count = 0;
        int min_distance = (PPG_SAMPLE_RATE_HZ * 60) / (HR_VALID_RANGE_MAX * 2);

        float threshold = 0.3f; // Threshold for peak detection
        int last_peak = -min_distance;
        for (int i = 1; i < WINDOW_SIZE_SAMPLES - 1 && peak_count < 30; i++) {
            if (ppg_signal[i] > ppg_signal[i-1] &&
                ppg_signal[i] > ppg_signal[i+1] &&
                ppg_signal[i] > threshold &&
                (i - last_peak) >= min_distance) {
 
                peak_indices[peak_count++] = i;
                last_peak = i;
            }
        }
 
        // Calculate RR intervals
        int rr_count = 0;
        for (int i = 0; i < peak_count - 1 && rr_count < max_count; i++) {
            int distance = peak_indices[i + 1] - peak_indices[i];
            float rr_ms = ((float)distance / PPG_SAMPLE_RATE_HZ) * 1000.0f;

            if (rr_ms >= 300.0f && rr_ms <= 1500.0f) {
                rr_intervals[rr_count++] = rr_ms;
            }
        }
 
        return rr_count;
    }
    /**
   170  * @brief Placeholder stress estimation based on HRV (SDNN)
   171  * Lower HRV (SDNN) typically correlates with higher stress.
   172  * Range: 0 (Relaxed) to 100 (High Stress)
   173  */
    int estimate_stress(float sdnn) {
        if (sdnn < 0.0f) return 0;
   
        // Heuristic mapping:
        // SDNN > 100ms -> Low stress
        // SDNN < 30ms  -> High stress
 
         if (sdnn >= 100.0f) return 10; // Very relaxed
         if (sdnn <= 20.0f) return 90;  // Very stressed
 
         // Linear interpolation roughly
         // Stress = 100 - (SDNN * Factor)
         // 100 - 30 = 70 (High stress)
         // 100 - 90 = 10 (Low stress)
 
         int stress = 110 - (int)sdnn;
 
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
   
        // Calculate variance of acceleration magnitude
       float mean_mag = 0.0f;
       for (int i = 0; i < length; i++) {
           float mag = sqrtf((float)accel_x[i] * accel_x[i] +
                            (float)accel_y[i] * accel_y[i] +
                            (float)accel_z[i] * accel_z[i]);
       mean_mag += mag;
   }
       mean_mag /= length;

       float variance = 0.0f;
       for (int i = 0; i < length; i++) {
           float mag = sqrtf((float)accel_x[i] * accel_x[i] +
                            (float)accel_y[i] * accel_y[i] +
                            (float)accel_z[i] * accel_z[i]);
           float diff = mag - mean_mag;
           variance += diff * diff;
       }
       variance /= length;

       float std_dev = sqrtf(variance);

       return (std_dev > (float)threshold);
   }

// ============================================================
// FREQUENCY DOMAIN ANALYSIS IMPLEMENTATION (ESP-DSP)
// ============================================================
#include "dsps_fft2r.h"
#include "dsps_wind.h"
#include "esp_log.h"

bool resample_rr_intervals(const float *rr_intervals, int count, float *output_buffer) {
    if (count < 2) return false;
    if (count > FFT_SIZE) count = FFT_SIZE; // Safety clamp

    // Stack allocation is safe here (64 * 4 = 256 bytes)
    // Much safer/faster than malloc in a loop
    float time_axis[FFT_SIZE]; 

    time_axis[0] = 0.0f;
    for (int i = 1; i < count; i++) {
        time_axis[i] = time_axis[i-1] + rr_intervals[i-1]; // Cumulative sum
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
    
    // Detrend (Remove DC)
    float mean = calculate_mean(output_buffer, FFT_SIZE);
    for(int i=0; i<FFT_SIZE; i++) output_buffer[i] -= mean;

    return true;
}

float calculate_lf_hf_ratio(float *signal_4hz) {
    // esp-dsp expects buffer size 2*N for Complex input (Real, Imag interleaved)
    // We reuse signal_4hz as Real part, but need space for Imag
    // To save memory, we allocate a combined buffer on stack
    float fft_buffer[FFT_SIZE * 2]; 
    
    // 1. Apply Hanning Window & Interleave to Complex Format
    // esp-dsp window function
    float wind[FFT_SIZE];
    dsps_wind_hann_f32(wind, FFT_SIZE);

    for (int i = 0; i < FFT_SIZE; i++) {
        fft_buffer[i * 2 + 0] = signal_4hz[i] * wind[i]; // Real
        fft_buffer[i * 2 + 1] = 0.0f;                    // Imag
    }

    // 2. Perform FFT
    // Initialize is done in main, but safe to call if needed (or assume main calls it)
    // dsps_fft2r_init_fc32(NULL, CONFIG_DSP_MAX_FFT_SIZE); // Usually handled globally
    
    dsps_fft2r_fc32(fft_buffer, FFT_SIZE);
    
    // 3. Bit Reversal (Standard FFT step)
    dsps_bit_rev_fc32(fft_buffer, FFT_SIZE);

    // 4. Compute Power & Integrate
    float bin_width = (float)HRV_SAMPLING_HZ / FFT_SIZE;
    float lf_power = 0.0f;
    float hf_power = 0.0f;

    // Start from bin 1 to skip DC
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

    if (hf_power < 0.0001f) return 1.0f;
    return lf_power / hf_power;
}