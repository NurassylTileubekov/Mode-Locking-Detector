#include "CV.h"
#include <math.h>
#include <string.h>

void calc_sliding_cv(uint16_t* new_buf, uint16_t size, uint16_t target_window_ms, sliding_cv_t* scv) {
    uint32_t chunk_sum    = 0;
    uint64_t chunk_sq_sum = 0;

    // Accumulate stats for the newest 1ms chunk (unrolled x4).
    int i = 0;
    for (; i <= size - 4; i += 4) {
        uint32_t v0 = new_buf[i];
        uint32_t v1 = new_buf[i+1];
        uint32_t v2 = new_buf[i+2];
        uint32_t v3 = new_buf[i+3];

        chunk_sum    += v0 + v1 + v2 + v3;
        chunk_sq_sum += (uint64_t)(v0 * v0) +
                        (uint64_t)(v1 * v1) +
                        (uint64_t)(v2 * v2) +
                        (uint64_t)(v3 * v3);
    }

    for (; i < size; i++) {
        uint32_t val  = new_buf[i];
        chunk_sum    += val;
        chunk_sq_sum += (uint64_t)(val * val);
    }

    if (target_window_ms > MAX_WINDOW_MS) target_window_ms = MAX_WINDOW_MS;
    if (target_window_ms == 0)            target_window_ms = 1;
    if (scv->old_target_window_ms != target_window_ms) {
        while (scv->active_chunks > target_window_ms) {
            uint16_t tail_idx = scv->head_idx + MAX_WINDOW_MS - scv->active_chunks;
            if (tail_idx >= MAX_WINDOW_MS) tail_idx -= MAX_WINDOW_MS;
            scv->total_sum        -= scv->sum_history[tail_idx];
            scv->total_sq_sum     -= scv->sq_sum_history[tail_idx];
            scv->active_chunks--;
        }
        scv->old_target_window_ms = target_window_ms;
    }

    // Evict the oldest chunk if the window is full.
    if (scv->active_chunks >= target_window_ms) {
        uint16_t tail_idx = scv->head_idx + MAX_WINDOW_MS - target_window_ms;
        if (tail_idx >= MAX_WINDOW_MS) tail_idx -= MAX_WINDOW_MS;

        scv->total_sum    -= scv->sum_history[tail_idx];
        scv->total_sq_sum -= scv->sq_sum_history[tail_idx];
    } else {
        scv->active_chunks++;
    }

    scv->total_sum    += chunk_sum;
    scv->total_sq_sum += chunk_sq_sum;

    scv->sum_history[scv->head_idx]    = chunk_sum;
    scv->sq_sum_history[scv->head_idx] = chunk_sq_sum;
    scv->head_idx++;
    if (scv->head_idx >= MAX_WINDOW_MS) scv->head_idx = 0;

    uint64_t n_samples = (uint64_t)scv->active_chunks * size;
    if (n_samples > 1) {
        float raw_mean = (float)scv->total_sum / (float)n_samples;

        // Variance via Var = E[x^2] - (E[x])^2, kept in integer arithmetic to
        // avoid catastrophic cancellation. With S = q*N + r:
        //   S^2/N = q^2*N + 2*q*r + r^2/N   (first two terms exact integers)
        uint64_t q, r;
        // Fast-path the common 32-bit case (64-bit division is slow on M4).
        if (scv->total_sum <= 0xFFFFFFFFULL) {
            uint32_t ts32 = (uint32_t)scv->total_sum;
            uint32_t ns32 = (uint32_t)n_samples;
            q = ts32 / ns32;
            r = ts32 % ns32;
        } else {
            q = scv->total_sum / n_samples;
            r = scv->total_sum % n_samples;
        }

        uint64_t mean_sq_int  = (q * q * n_samples) + (2 * q * r);

        float fr = (float)r;
        float mean_sq_frac = (fr * fr) / (float)n_samples;

        // Bessel-corrected sample variance, split into integer and float parts.
        float variance = 0.0f;
        if (scv->total_sq_sum >= mean_sq_int) {
            uint64_t var_num_int = scv->total_sq_sum - mean_sq_int;
            variance = ((float)var_num_int - mean_sq_frac) / (float)(n_samples - 1);
            if (variance < 0.0f) variance = 0.0f;
        }

        float std_dev = sqrtf(variance);

        scv->current_cv = (raw_mean != 0.0f)
                        ? (std_dev / raw_mean) * 100.0f
                        : 0.0f;

        scv->current_mean = raw_mean * (ADC_VREF_MV / MAX_ADC_VAL);
    }
}

// Populate a LUT mapping ADC value -> CV threshold. Call at startup and after fitting.
void generate_cv_threshold_lut(float* buf, cv_model_params_t* params) {
    if (!params) return;
    for (int i = 0; i < CV_LUT_SIZE; i++) {
        float adc_val = (float)i * (MAX_ADC_VAL / (float)(CV_LUT_SIZE - 1));
        float x_norm = adc_val / MAX_ADC_VAL;
        if (x_norm < 0.001f) x_norm = 0.001f;  // avoid div-by-zero in the power term

        float exp_term = params->a * expf(params->b * x_norm);
        float pow_term = params->c * powf(x_norm, params->d);

        buf[i] = exp_term + pow_term + params->e;
    }
}
