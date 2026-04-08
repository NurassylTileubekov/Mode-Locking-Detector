//
// Created by nurassyl on 7.04.2026.
//
#include "CV.h"
#include <math.h>
#include <stdint.h>
#include "arm_math.h"


void calcCV(uint16_t* buf, int size, laser_par_t* par, laser_state_t* state) {
    if (size > 1) {
        float f_buf[size];
        float avg = 0.0f;
        float std = 0.0f;
        for (int i = 0; i < size; i++ ) {
            f_buf[i] = (float)buf[i];
        }
        arm_mean_f32(f_buf, size, &avg);
        state->avg_mV_rms = (avg * 3300.0f)/30712.5f;
        arm_std_f32(f_buf, size, &std);
        if (avg > 0.0f) {
            state->current_cv = (std / avg) * 100.0f;
            if (state->current_cv < par->threshold_cv) {
                state->cv_status = STABLE;
                if (state->laser_status == NOT_MODE_LOCKED || state->laser_status == Q_SWITCHING) {
                    state->laser_status = Q_SWITCHING;
                    state->stability_counter++;
                }
            }
            else {
                state->cv_status = UNSTABLE;
                state->laser_status = NOT_MODE_LOCKED;
                state->stability_counter = 0;
            }
            if (state->stability_counter >= par->stability_window) {
                state->laser_status = MODE_LOCKED;
            }
        }
        else {
            state->current_cv = 0.0f;
            state->cv_status = UNSTABLE;
            state->laser_status = NOT_MODE_LOCKED;
            state->stability_counter = 0;
            buf[0] = 0;
        }
    }
    else {
        state->current_cv = 0.0f;
        state->cv_status = UNSTABLE;
        state->laser_status = NOT_MODE_LOCKED;
        state->stability_counter = 0;
    }
}