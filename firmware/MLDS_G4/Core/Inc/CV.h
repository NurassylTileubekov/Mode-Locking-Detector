// Inc/CV.h
#ifndef MLDS_G4_CV_H
#define MLDS_G4_CV_H

#include <stdint.h>

#define MAX_WINDOW_MS 1000   // max sliding window (~12KB RAM)
#define BUF_SIZE 2000
#define HALF_BUF_SIZE (BUFFER_SIZE/2)

// CV curve fit model: y = a*exp(b*x) + c*x^d + e
typedef struct {
    float a;
    float b;
    float c;
    float d;
    float e;
} cv_model_params_t;

#define DEFAULT_CV_PARAMS (cv_model_params_t){-0.1646f, -16.1240f, 0.0232f, -0.9721f, -0.0049f}

#define MAX_ADC_VAL 4095.0f
#define ADC_VREF_MV 3300.0f
#define CV_LUT_SIZE     4096

#define CV_CAPTURE_RESOLUTION 256

#define CV_CAPTURE_MODE_MIN 0
#define CV_CAPTURE_MODE_AVG 1

// MIN tracks the stable mode-locked lower envelope, robust to transient CV spikes.
#define CV_CAPTURE_MODE CV_CAPTURE_MODE_MIN

typedef struct {
    uint32_t sum_history[MAX_WINDOW_MS];
    uint64_t sq_sum_history[MAX_WINDOW_MS];
    uint64_t total_sum;
    uint64_t total_sq_sum;
    uint16_t head_idx;
    uint16_t active_chunks;
    float current_cv;
    float current_mean;
    float old_target_window_ms;
} sliding_cv_t;

void calc_sliding_cv(uint16_t* new_buf, uint16_t size, uint16_t target_window_ms, sliding_cv_t* scv);
void generate_cv_threshold_lut(float* buf, cv_model_params_t* params);

#endif // MLDS_G4_CV_H
