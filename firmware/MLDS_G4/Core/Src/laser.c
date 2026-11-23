// Src/laser.c
#include "laser.h"
#include <math.h>

void process_laser_logic(uint16_t* buf, uint16_t size, sliding_cv_t* scv, float* cv_threshold_lut,
    laser_config_t* config, cw_calibration_t* cal, uint16_t adc2_val, laser_state_t* state) {
    uint8_t has_seen_cw = state->has_seen_cw;

    calc_sliding_cv(buf, size, config->target_window_ms, scv);

    // Map the mean signal (mV) to a LUT index.
    float float_index = (scv->current_mean / ADC_VREF_MV) * (CV_LUT_SIZE - 1);
    int i = (int)float_index;
    if (i < 0) i = 0;
    if (i >= CV_LUT_SIZE) i = CV_LUT_SIZE - 1;

    state->current_cv_threshold = cv_threshold_lut[i] + config->threshold_offset;

    state->cw_mv = ((float)adc2_val * ADC_VREF_MV) / MAX_ADC_VAL;
    state->ml_rms_mv = scv->current_mean;

    float ml_high = ML_THRESHOLD_HIGH_MV * config->saturation_percent;
    float cw_high = cal->cw_saturation   * config->saturation_percent;

    if (state->ml_rms_mv < ML_THRESHOLD_LOW_MV) {
        // CW regime
        if (state->cw_mv >= cw_high) {
            state->status = SATURATED;
        } else if (state->cw_mv < cal->cw_threshold_low) {
            state->status = LOW_SIGNAL;
        } else {
            state->status = CW;
            has_seen_cw = 1;
        }
    } else {
        // Mode-locked regime
        if (state->ml_rms_mv >= ml_high) {
            state->status = SATURATED;
        } else if (scv->current_cv < state->current_cv_threshold) {
            state->status = MODE_LOCKED;
            has_seen_cw = 0;
        } else if (has_seen_cw) {
            state->status = INITIATING;
        } else {
            state->status = UNSTABLE;
        }
    }

    state->has_seen_cw = has_seen_cw;
}
