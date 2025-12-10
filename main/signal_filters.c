#include "signal_filters.h"
#include <string.h>

// ============================================================
// PPG FILTER IMPLEMENTATION
// ============================================================

void ppg_filter_init(ppg_filter_t *filter) {
    memset(filter, 0, sizeof(ppg_filter_t));

    // HPF (0.5 Hz)
    filter->section1.b0 = 0.97803f;
    filter->section1.b1 = -1.95606f;
    filter->section1.b2 = 0.97803f;
    filter->section1.a1 = -1.95558f;
    filter->section1.a2 = 0.95654f;
// [TUNING] Thay đổi hệ số bộ lọc LPF (Section 2)
// Cutoff ~4.0 Hz (Cho phép nhịp tim nhanh tới 240bpm)
filter->section2.b0 = 0.01336f;
filter->section2.b1 = 0.02672f;
filter->section2.b2 = 0.01336f;
filter->section2.a1 = -1.64746f;
filter->section2.a2 = 0.70090f;
    
    filter->gain = 1.0f;
}

static float section_process(iir_section_t *s, float input) {
    float w = input - (s->a1 * s->w1) - (s->a2 * s->w2);
    float y = (s->b0 * w) + (s->b1 * s->w1) + (s->b2 * s->w2);
    s->w2 = s->w1;
    s->w1 = w;
    return y;
}

float ppg_filter_process(ppg_filter_t *filter, float input) {
    float stage1 = section_process(&filter->section1, input);
    float stage2 = section_process(&filter->section2, stage1);
    return stage2 * filter->gain;
}


// ============================================================
// KALMAN FILTER IMPLEMENTATION
// ============================================================

void kalman_filter_init(kalman_filter_t *kf, float q, float r, float initial_value) {
    kf->q = q;
    kf->r = r;
    kf->x = initial_value;
    kf->p = 1.0f;
    kf->k = 0.0f;
}

float kalman_filter_update(kalman_filter_t *kf, float measurement) {
    kf->p = kf->p + kf->q;
    kf->k = kf->p / (kf->p + kf->r);
    kf->x = kf->x + kf->k * (measurement - kf->x);
    kf->p = (1.0f - kf->k) * kf->p;
    return kf->x;
}


// ============================================================
// TEMP FILTER IMPLEMENTATION
// ============================================================

void temp_filter_init(temp_filter_t *filter) {
    memset(filter, 0, sizeof(temp_filter_t));
}

float temp_filter_process(temp_filter_t *filter, float input) {
    filter->sum -= filter->buffer[filter->index];
    filter->buffer[filter->index] = input;
    filter->sum += input;
    
    filter->index++;
    if (filter->index >= TEMP_FILTER_WINDOW) {
        filter->index = 0;
    }
    
    if (filter->count < TEMP_FILTER_WINDOW) {
        filter->count++;
    }
    
    return filter->sum / filter->count;
}

// ============================================================
// LMS ADAPTIVE FILTER IMPLEMENTATION
// ============================================================

void lms_filter_init(lms_filter_t *lms, float learning_rate) {
    memset(lms->weights, 0, sizeof(lms->weights));
    memset(lms->buffer, 0, sizeof(lms->buffer));
    lms->mu = learning_rate;
    lms->head = 0;
}

float lms_filter_update(lms_filter_t *lms, float input_signal, float noise_ref) {
    lms->head = (lms->head - 1 + LMS_FILTER_TAPS) % LMS_FILTER_TAPS;
    lms->buffer[lms->head] = noise_ref;

    float y = 0.0f;
    for (int i = 0; i < LMS_FILTER_TAPS; i++) {
        int idx = (lms->head + i) % LMS_FILTER_TAPS;
        y += lms->weights[i] * lms->buffer[idx];
    }

    float e = input_signal - y;

    for (int i = 0; i < LMS_FILTER_TAPS; i++) {
        int idx = (lms->head + i) % LMS_FILTER_TAPS;
        float delta = 2.0f * lms->mu * e * lms->buffer[idx];
        lms->weights[i] += delta;
        
        if (lms->weights[i] > 10.0f) lms->weights[i] = 10.0f;
        if (lms->weights[i] < -10.0f) lms->weights[i] = -10.0f;
    }

    return e;
}

// ============================================================
// [NEW] SIGNAL UTILITIES IMPLEMENTATION
// ============================================================

void signal_utils_smooth(float *input, float *output, int length, int window) {
    float sum = 0;
    int half_window = window / 2;
    
    // Initial fill
    for(int i=0; i<window && i<length; i++) {
        sum += input[i];
    }
    
    for (int i = 0; i < length; i++) {
        if (i >= half_window && i + half_window < length) {
             output[i] = sum / window;
             // Slide window
             sum -= input[i - half_window];
             sum += input[i + half_window];
        } else {
            output[i] = input[i]; // Copy edges
        }
    }
}

int signal_utils_count_peaks(float *signal, int length, float min_height, int min_distance) {
    int peaks = 0;
    int last_peak = -min_distance;
    
    for (int i = 1; i < length - 1; i++) {
        // Local maxima check
        if (signal[i] > signal[i-1] && signal[i] > signal[i+1]) {
            // Threshold and distance check
            if (signal[i] > min_height && (i - last_peak) >= min_distance) {
                peaks++;
                last_peak = i;
            }
        }
    }
    return peaks;
}