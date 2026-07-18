#ifndef FIT_H
#define FIT_H

#include <stdint.h>
#include "CV.h"

// Fit the CV curve model. captured_data (size CV_LUT_SIZE) uses <= 0 for empty
// bins; params_in_out is both the initial guess and the optimized result.
void fit_cv_curve(float* captured_data, cv_model_params_t* params_in_out, float* error_out, uint32_t* epochs_run, float* r_squared_out);

#endif // FIT_H
