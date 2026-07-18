#include "fit.h"
#include <math.h>

// Levenberg-Marquardt nonlinear least squares (same method as scipy.curve_fit /
// MINPACK lmdif). Handles the very different parameter magnitudes (b ~ -16 vs
// e ~ -0.005) that made the previous Adam optimizer oscillate.
#define NPARAM            5
#define LM_MAX_ITERS      200
#define LM_LAMBDA_INIT    1e-3f
#define LM_LAMBDA_FACTOR  10.0f
#define LM_LAMBDA_MAX     1e12f
#define LM_FTOL           1e-9f
#define LM_INNER_TRIES    40

static float valid_x[CV_LUT_SIZE];
static float valid_log_x[CV_LUT_SIZE];

// Model: y = a*exp(b*x) + c*x^d + e
static inline float model_eval(const float p[NPARAM], float x) {
    return p[0] * expf(p[1] * x) + p[2] * powf(x, p[3]) + p[4];
}

// Sum of squared residuals; +inf if the model blows up (rejects divergent steps).
static float compute_sse(const float p[NPARAM], const float* y, const float* xs, uint32_t n) {
    float sse = 0.0f;
    for (uint32_t i = 0; i < n; i++) {
        float r = model_eval(p, xs[i]) - y[i];
        sse += r * r;
    }
    return isfinite(sse) ? sse : INFINITY;
}

// Solve A*out = rhs (5x5) via Gaussian elimination with partial pivoting.
// A and rhs are modified in place. Returns 0 if (near-)singular.
static int solve5(float A[NPARAM][NPARAM], float rhs[NPARAM], float out[NPARAM]) {
    for (int col = 0; col < NPARAM; col++) {
        int piv = col;
        float best = fabsf(A[col][col]);
        for (int r = col + 1; r < NPARAM; r++) {
            float v = fabsf(A[r][col]);
            if (v > best) { best = v; piv = r; }
        }
        if (best < 1e-20f) return 0;
        if (piv != col) {
            for (int j = 0; j < NPARAM; j++) { float t = A[col][j]; A[col][j] = A[piv][j]; A[piv][j] = t; }
            float t = rhs[col]; rhs[col] = rhs[piv]; rhs[piv] = t;
        }
        for (int r = col + 1; r < NPARAM; r++) {
            float f = A[r][col] / A[col][col];
            if (f != 0.0f) {
                for (int j = col; j < NPARAM; j++) A[r][j] -= f * A[col][j];
                rhs[r] -= f * rhs[col];
            }
        }
    }
    for (int i = NPARAM - 1; i >= 0; i--) {
        float s = rhs[i];
        for (int j = i + 1; j < NPARAM; j++) s -= A[i][j] * out[j];
        out[i] = s / A[i][i];
    }
    return 1;
}

void fit_cv_curve(float* captured_data, cv_model_params_t* params_in_out, float* error_out, uint32_t* epochs_run, float* r_squared_out) {
    if (!captured_data || !params_in_out) return;

    // Seed from the caller's guess; left untouched until a finite result is accepted.
    float p[NPARAM] = {
        params_in_out->a, params_in_out->b, params_in_out->c,
        params_in_out->d, params_in_out->e
    };

    // Compact valid points to the front, caching x and log(x).
    uint32_t n = 0;
    for (int i = 0; i < CV_LUT_SIZE; i++) {
        if (captured_data[i] > 0.0f) {
            float x = (float)i / MAX_ADC_VAL;
            if (x < 0.001f) x = 0.001f;
            captured_data[n] = captured_data[i];
            valid_x[n]       = x;
            valid_log_x[n]   = logf(x);
            n++;
        }
    }

    if (n < 10) {
        if (error_out)     *error_out = -1.0f;
        if (epochs_run)    *epochs_run = 0;
        if (r_squared_out) *r_squared_out = 0.0f;
        return;
    }
    const float* y = captured_data;

    float mean_y = 0.0f;
    for (uint32_t i = 0; i < n; i++) mean_y += y[i];
    mean_y /= (float)n;
    float ss_tot = 0.0f;
    for (uint32_t i = 0; i < n; i++) { float dm = y[i] - mean_y; ss_tot += dm * dm; }

    float cost   = compute_sse(p, y, valid_x, n);
    float lambda = LM_LAMBDA_INIT;
    uint32_t iter = 0;

    while (iter < LM_MAX_ITERS) {
        iter++;

        // Accumulate JtJ (5x5 symmetric) and Jtr = J^T * residual.
        float JtJ[NPARAM][NPARAM] = {{0}};
        float Jtr[NPARAM] = {0};
        for (uint32_t i = 0; i < n; i++) {
            float x  = valid_x[i];
            float ev = expf(p[1] * x);
            float pv = powf(x, p[3]);
            float yp = p[0] * ev + p[2] * pv + p[4];
            float r  = yp - y[i];

            float J[NPARAM];
            J[0] = ev;
            J[1] = p[0] * x * ev;
            J[2] = pv;
            J[3] = p[2] * pv * valid_log_x[i];
            J[4] = 1.0f;

            for (int a = 0; a < NPARAM; a++) {
                Jtr[a] += J[a] * r;
                for (int b = a; b < NPARAM; b++) JtJ[a][b] += J[a] * J[b];
            }
        }
        for (int a = 0; a < NPARAM; a++)
            for (int b = 0; b < a; b++) JtJ[a][b] = JtJ[b][a];

        // Raise damping until a step reduces the cost.
        int accepted = 0;
        for (int tries = 0; tries < LM_INNER_TRIES; tries++) {
            float A[NPARAM][NPARAM];
            float rhs[NPARAM];
            float delta[NPARAM];
            for (int a = 0; a < NPARAM; a++) {
                for (int b = 0; b < NPARAM; b++) A[a][b] = JtJ[a][b];
                A[a][a] += lambda * JtJ[a][a];
                rhs[a]   = -Jtr[a];
            }

            if (!solve5(A, rhs, delta)) {
                lambda *= LM_LAMBDA_FACTOR;
                if (lambda > LM_LAMBDA_MAX) goto done;
                continue;
            }

            float p_new[NPARAM];
            for (int a = 0; a < NPARAM; a++) p_new[a] = p[a] + delta[a];
            float new_cost = compute_sse(p_new, y, valid_x, n);

            if (new_cost < cost) {
                float rel = (cost - new_cost) / (cost > 0.0f ? cost : 1.0f);
                for (int a = 0; a < NPARAM; a++) p[a] = p_new[a];
                cost   = new_cost;
                lambda /= LM_LAMBDA_FACTOR;
                accepted = 1;
                if (rel < LM_FTOL) goto done;
                break;
            } else {
                lambda *= LM_LAMBDA_FACTOR;
                if (lambda > LM_LAMBDA_MAX) goto done;
            }
        }
        if (!accepted) goto done;
    }
done:;

    // Never persist a non-finite fit.
    for (int a = 0; a < NPARAM; a++) {
        if (!isfinite(p[a])) {
            if (error_out)     *error_out = -1.0f;
            if (epochs_run)    *epochs_run = iter;
            if (r_squared_out) *r_squared_out = 0.0f;
            return;
        }
    }

    params_in_out->a = p[0];
    params_in_out->b = p[1];
    params_in_out->c = p[2];
    params_in_out->d = p[3];
    params_in_out->e = p[4];

    if (error_out)     *error_out  = cost / (float)n;
    if (epochs_run)    *epochs_run = iter;
    if (r_squared_out) *r_squared_out = (ss_tot > 0.0f) ? (1.0f - cost / ss_tot) : 0.0f;
}
