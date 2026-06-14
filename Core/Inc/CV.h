// Inc/CV.h
#ifndef MLDS_G4_CV_H
#define MLDS_G4_CV_H

#include <stdint.h>

#define MAX_WINDOW_MS 1000   // Maximum possible sliding window size (1000ms = 1 seconds, takes ~12KB RAM)
#define BUF_SIZE 2000
#define HALF_BUF_SIZE (BUFFER_SIZE/2) // Size of DMA half-buffer

// Exponential Component (Low ADC / Noise Floor)
#define PARAM_A 0.2260f // Exponential amplitude
#define PARAM_B -0.3752f // Exponential decay rate

// Inverse Power Law Component (High ADC / Linear RF Region)
#define PARAM_C 0.0340f  // Power law amplitude
#define PARAM_D -0.8608f // Power law exponent

// System Baseline Offset
#define PARAM_E -0.1684f // DC drift and minimum threshold floor

#define MAX_ADC_VAL 4095.0f
#define CV_LUT_SIZE 4095



typedef struct {
    // Sliding Window Memory (Takes only ~1.2 KB of RAM!)
    uint32_t sum_history[MAX_WINDOW_MS];
    uint64_t sq_sum_history[MAX_WINDOW_MS];
    // Running Totals
    uint64_t total_sum;
    uint64_t total_sq_sum;
    uint16_t head_idx;
    uint16_t active_chunks;
    // Results
    float current_cv;
    float current_mean;
} sliding_cv_t;

void calc_sliding_cv(uint16_t* new_buf, uint16_t size, uint16_t target_window_ms, sliding_cv_t* scv);
void generate_cv_threshold_lut(float* buf);

#endif // MLDS_G4_CV_H