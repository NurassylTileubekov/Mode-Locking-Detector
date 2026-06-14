// Inc/laser.h
#ifndef LASER_H
#define LASER_H

#include <stdint.h>
#include "CV.h"

enum laser_status {
    NO_SIGNAL,
    CW,
    UNSTABLE,
    MODE_LOCKED,
    SATURATED
};

// Configuration received from PC
typedef struct __attribute__((packed)) {
    uint8_t frame_start;
    uint16_t target_window_ms;
    float ml_threshold_low;
    float ml_threshold_high;
    float cw_threshold_low;
    float cw_threshold_high;
    float threshold_offset;
    uint8_t frame_end;
} laser_config_t;

// Output state of the laser logic
typedef struct {
    float ml_rms_mv;
    float cw_mv;
    float current_cv;
    float current_cv_threshold;
    enum laser_status status;
} laser_state_t;

// Telemetry frame sent to PC
typedef struct __attribute__((packed)) {
    uint8_t frame_start;
    float ml_rms_mv;
    float cw_mv;
    float current_cv;
    float current_cv_threshold;
    enum laser_status status;
    uint8_t frame_end;
} laser_frame_t;

void process_laser_logic(uint16_t* buf, uint16_t size, sliding_cv_t* scv, float* cv_threshold_lut, laser_config_t* config, uint16_t adc2_val, laser_state_t* state);

#endif // LASER_H