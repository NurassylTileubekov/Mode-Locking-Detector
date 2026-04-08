//
// Created by nurassyl on 7.04.2026.
//

#ifndef MLDS_G4_CV_H
#define MLDS_G4_CV_H
#include <stdint.h>
enum laser_status{
    NOT_MODE_LOCKED,
    Q_SWITCHING,
    MODE_LOCKED
  };
enum cv_status{
    UNSTABLE,
    STABLE
  };
typedef struct {
    float threshold_cv;
    uint16_t stability_window;
}laser_par_t;

typedef struct {
    float current_cv;
    float avg_mV_rms;
    uint16_t stability_counter;
    enum laser_status laser_status;
    enum cv_status cv_status;
}laser_state_t;

typedef struct {

}laser_features_t;
void calcCV(uint16_t* buf, int size, laser_par_t* par, laser_state_t* state);
#endif //MLDS_G4_CV_H