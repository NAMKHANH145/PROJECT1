#include "signal_filters.h"
#include <string.h>

// ============================================================
// PPG FILTER IMPLEMENTATION
// ============================================================
// Designed using digital filter design tools (Butterworth)
// Fs = 100 Hz

void ppg_filter_init(ppg_filter_t *filter) {
    memset(filter, 0, sizeof(ppg_filter_t));

    // --------------------------------------------------------
    // SECTION 1: High-pass Filter (Cutoff ~0.5 Hz)
    // --------------------------------------------------------
    // Reduces baseline wander
    // b = [0.97803, -1.95606, 0.97803]
    // a = [1.00000, -1.95558, 0.95654]
    filter->section1.b0 = 0.97803f;
    filter->section1.b1 = -1.95606f;
    filter->section1.b2 = 0.97803f;
    filter->section1.a1 = -1.95558f;
    filter->section1.a2 = 0.95654f;

    // --------------------------------------------------------
    // SECTION 2: Low-pass Filter (Cutoff ~4.0 Hz)
    // --------------------------------------------------------
    // Removes high freq noise
    // b = [0.01336, 0.02672, 0.01336]
    // a = [1.00000, -1.64746, 0.70090]
    filter->section2.b0 = 0.01336f;
    filter->section2.b1 = 0.02672f;
    filter->section2.b2 = 0.01336f;
    filter->section2.a1 = -1.64746f;
    filter->section2.a2 = 0.70090f;
    
    filter->gain = 1.0f;
}

static float section_process(iir_section_t *s, float input) {
    // Direct Form II implementation
    // w[n] = x[n] - a1*w[n-1] - a2*w[n-2]
    // y[n] = b0*w[n] + b1*w[n-1] + b2*w[n-2]
    
    float w = input - (s->a1 * s->w1) - (s->a2 * s->w2);
    float y = (s->b0 * w) + (s->b1 * s->w1) + (s->b2 * s->w2);
    
    // Shift delay line
    s->w2 = s->w1;
    s->w1 = w;
    
    return y;
}

float ppg_filter_process(ppg_filter_t *filter, float input) {
    // Cascade processing: Input -> HPF -> LPF -> Output
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
    kf->p = 1.0f; // Initial uncertainty
    kf->k = 0.0f;
}

float kalman_filter_update(kalman_filter_t *kf, float measurement) {
    // 1. Prediction Update
    // x = x (No control input)
    // p = p + q
    kf->p = kf->p + kf->q;

    // 2. Measurement Update
    // k = p / (p + r)
    kf->k = kf->p / (kf->p + kf->r);
    
    // x = x + k * (measurement - x)
    kf->x = kf->x + kf->k * (measurement - kf->x);
    
    // p = (1 - k) * p
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
    // Subtract oldest value
    filter->sum -= filter->buffer[filter->index];
    
    // Overwrite with new value
    filter->buffer[filter->index] = input;
    filter->sum += input;
    
    // Move index
    filter->index++;
    if (filter->index >= TEMP_FILTER_WINDOW) {
        filter->index = 0;
    }
    
    // Increment count until window is full
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
    // 1. Update Reference Buffer (Circular Buffer)
    lms->head = (lms->head - 1 + LMS_FILTER_TAPS) % LMS_FILTER_TAPS;
    lms->buffer[lms->head] = noise_ref;

    // 2. Filter (Dot Product: y = w * x)
    float y = 0.0f;
    for (int i = 0; i < LMS_FILTER_TAPS; i++) {
        int idx = (lms->head + i) % LMS_FILTER_TAPS;
        y += lms->weights[i] * lms->buffer[idx];
    }

    // 3. Calculate Error (Clean Signal: e = d - y)
    // d = input_signal (Primary + Noise)
    // y = Estimated Noise
    float e = input_signal - y;

    // 4. Update Weights (LMS Rule: w = w + 2 * mu * e * x)
    // Clamp weights to prevent explosion if mu is too high
    for (int i = 0; i < LMS_FILTER_TAPS; i++) {
        int idx = (lms->head + i) % LMS_FILTER_TAPS;
        float delta = 2.0f * lms->mu * e * lms->buffer[idx];
        lms->weights[i] += delta;
        
        // Safety Clamp
        if (lms->weights[i] > 10.0f) lms->weights[i] = 10.0f;
        if (lms->weights[i] < -10.0f) lms->weights[i] = -10.0f;
    }

    return e; // The error IS the clean signal!
}
