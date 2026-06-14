// Src/laser.c
#include "laser.h"
#include <math.h>


float lerp(float a, float b, float t) {
    return a + (b - a) * t;
}
void process_laser_logic(uint16_t* buf, uint16_t size, sliding_cv_t* scv, float* cv_threshold_lut,
    laser_config_t* config, uint16_t adc2_val, laser_state_t* state) {
    calc_sliding_cv(buf, size, config->target_window_ms, scv);
    // Calculate exact float index
    float float_index = (scv->current_mean / (float)MAX_ADC_VAL) * (CV_LUT_SIZE - 1);
    // Extract base integer index and fractional part (t) for lerp
    int i = (int)float_index;
    // Safety Clamp!
    if (i < 0) i = 0;
    if (i >= CV_LUT_SIZE) i = CV_LUT_SIZE - 1;

    //Uncomment if LUT is less than 4095
    //float t = float_index - (float)i;
    // Clamp bounds to prevent out-of-bounds on array access
    // if (i < 0) {
    //     i = 0;
    //     t = 0.0f;
    // } else if (i >= CV_LUT_SIZE - 1) {
    //     i = CV_LUT_SIZE - 2;
    //     t = 1.0f;
    // }
    state->current_cv_threshold = cv_threshold_lut[i] * config->threshold_offset;
    //Uncomment if LUT is less than 4095
    //state->current_cv_threshold = lerp(cv_threshold_lut[i], cv_threshold_lut[i + 1], t);

    state->cw_mv= ((float)adc2_val * 3300.0f) / MAX_ADC_VAL;
    state->ml_rms_mv = (scv->current_mean * 3300.0f) / MAX_ADC_VAL;

    // Optimized Control Flow
    if (state->ml_rms_mv >= config->ml_threshold_high || state->cw_mv >= config->cw_threshold_high) {
        state->status = SATURATED;
    }
    else {
        if (state->ml_rms_mv < config->ml_threshold_low) {
            if (state->cw_mv < config->ml_threshold_low) {
                state->status = NO_SIGNAL;
            } else {
                state->status = CW;
            }
        } else {
            if (scv->current_cv < state->current_cv_threshold) {
                state->status = MODE_LOCKED;
            } else {
                state->status = UNSTABLE;
            }
        }
    }
}