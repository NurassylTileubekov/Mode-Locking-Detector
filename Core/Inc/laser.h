// Inc/laser.h
#ifndef LASER_H
#define LASER_H

#include <stdint.h>
#include "CV.h"

// ML detection thresholds (mV). High threshold is scaled by saturation_percent.
#define ML_THRESHOLD_LOW_MV   300.0f
#define ML_THRESHOLD_HIGH_MV  3300.0f

#define CW_THRESHOLD_LOW_MV 350.0f
#define CW_THRESHOLD_HIGH_MV 3250.0f

// Defaults: target_window_ms, threshold_offset, saturation_percent
#define LASER_CONFIG_DEFAULT (laser_config_t){0xAA, 25, 0.05f, 0.9f, 0xBB}

enum laser_status {
    LOW_SIGNAL,
    CW,
    UNSTABLE,
    MODE_LOCKED,
    SATURATED,
    INITIATING,
};

// Configuration received from PC
typedef struct __attribute__((packed)) {
    uint8_t frame_start;
    uint16_t target_window_ms;
    float threshold_offset;
    float saturation_percent;   // 0.0 - 1.0, scales the saturation thresholds
    uint8_t frame_end;
} laser_config_t;

// CW thresholds (mV) produced by calibration; stored in flash with the CV model.
typedef struct {
    float cw_threshold_low;
    float cw_saturation;
} cw_calibration_t;
#define CW_CALIBRATION_DEFAULT (cw_calibration_t){100.0f, 3200.0f}

// Combined calibration written to flash in a single page-erase.
typedef struct {
    cv_model_params_t cv;
    cw_calibration_t  cw;
} laser_calibration_t;

typedef struct {
    float ml_rms_mv;
    float cw_mv;
    float current_cv;
    float current_cv_threshold;
    enum laser_status status;
    uint8_t has_seen_cw;
} laser_state_t;

// Telemetry frame sent to PC
typedef struct __attribute__((packed)) {
    uint8_t frame_start;
    float ml_rms_mv;
    float cw_mv;
    float current_cv;
    float current_cv_threshold;
    uint8_t status;
    uint8_t frame_end;
} laser_frame_t;

void process_laser_logic(uint16_t* buf, uint16_t size, sliding_cv_t* scv, float* cv_threshold_lut, laser_config_t* config, cw_calibration_t* cal, uint16_t adc2_val, laser_state_t* state);

#endif // LASER_H
