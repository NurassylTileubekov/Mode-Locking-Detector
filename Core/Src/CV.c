// Src/CV.c
#include "CV.h"
#include <math.h>
#include <string.h>

void calc_sliding_cv(uint16_t* new_buf, uint16_t size, uint16_t target_window_ms, sliding_cv_t* scv) {
    uint32_t chunk_sum = 0;
    uint64_t chunk_sq_sum = 0;
    static uint16_t old_target_window_ms = 0;
    
    // 1. Calculate statistics for ONLY the newest 1ms chunk
    for (int i = 0; i < size; i++) {
        uint32_t val = new_buf[i];
        chunk_sum += val;
        chunk_sq_sum += (val * val);
    }
    
    if (target_window_ms > MAX_WINDOW_MS) {
        target_window_ms = MAX_WINDOW_MS;
    }
    if (target_window_ms == 0) {
        target_window_ms = 1;
    }
    
    if (old_target_window_ms != target_window_ms) {
        // If the window shrank and we have more active chunks than the new window size,
        // we must remove the oldest excess chunks from our running totals.
        while (scv->active_chunks > target_window_ms) {
            uint16_t tail_idx = (scv->head_idx + MAX_WINDOW_MS - scv->active_chunks) % MAX_WINDOW_MS;
            scv->total_sum -= scv->sum_history[tail_idx];
            scv->total_sq_sum -= scv->sq_sum_history[tail_idx];
            scv->active_chunks--;
        }
        old_target_window_ms = target_window_ms;
    }

    // 2. Remove the oldest 1ms chunk from the running totals if window is full
    if (scv->active_chunks >= target_window_ms) {
        // Find the index of the oldest chunk
        uint16_t tail_idx = (scv->head_idx + MAX_WINDOW_MS - target_window_ms) % MAX_WINDOW_MS;
        scv->total_sum -= scv->sum_history[tail_idx];
        scv->total_sq_sum -= scv->sq_sum_history[tail_idx];
    } else {
        scv->active_chunks++; // We are still filling the initial window
    }

    // 3. Add the newest 1ms chunk to running totals
    scv->total_sum += chunk_sum;
    scv->total_sq_sum += chunk_sq_sum;

    // 4. Overwrite the oldest data in the circular buffer with the new data
    scv->sum_history[scv->head_idx] = chunk_sum;
    scv->sq_sum_history[scv->head_idx] = chunk_sq_sum;

    // Advance the head pointer
    scv->head_idx = (scv->head_idx + 1) % MAX_WINDOW_MS;

    // 5. Calculate the CV from the running totals
    uint32_t n_samples = scv->active_chunks * size;
    if (n_samples > 1) {
        // Calculate Mean
        scv->current_mean = (float)scv->total_sum / (float)n_samples;

        // Calculate Variance using integer math to prevent catastrophic cancellation
        // We use algebraic expansion to prevent uint64_t overflow on large windows:
        // (S^2)/N = q^2*N + 2*q*r + r^2/N  (where q = S/N, r = S%N)
        uint64_t q = scv->total_sum / n_samples;
        uint64_t r = scv->total_sum % n_samples;
        uint64_t mean_of_sum_squared = (q * q * n_samples) + (2 * q * r) + ((r * r) / n_samples);

        uint64_t variance_numerator = 0;
        
        if (scv->total_sq_sum > mean_of_sum_squared) {
            variance_numerator = scv->total_sq_sum - mean_of_sum_squared;
        }
        
        float variance = (float)variance_numerator / (float)(n_samples - 1);
        float std_deviation = sqrtf(variance);

        // Calculate final CV percentage
        if (scv->current_mean != 0.0f) {
            scv->current_cv = (std_deviation / scv->current_mean) * 100.0f;
        } else {
            scv->current_cv = 0.0f;
        }
    }
}

// Generate LUT CV threshold
void generate_cv_threshold_lut(float* buf) {
    for (int i = 0; i < CV_LUT_SIZE; i++) {
        // 1. Calculate corresponding ADC value for the current array index
        // This ensures the size of the array spans the full 12-bit range
        float adc_val = (float)i * (MAX_ADC_VAL / (float)(CV_LUT_SIZE - 1));
        
        // 2. Normalize ADC value to 0.0 - 1.0 matching the regression model
        float x_norm = adc_val / MAX_ADC_VAL;
        
        // 3. Prevent domain error (division by zero) for the inverse power law
        // The x_norm value is clamped to a minimum floor
        if (x_norm < 0.001f) {
            x_norm = 0.001f;
        }
        
        // 4. Execute single-precision composite math
        // y = a*exp(b*x) + c*(x^d) + e
        float exp_term = PARAM_A * expf(PARAM_B * x_norm);
        float pow_term = PARAM_C * powf(x_norm, PARAM_D);
        
        // 5. Store the calculated geometric threshold in the array
        buf[i] = exp_term + pow_term + PARAM_E;
    }
}