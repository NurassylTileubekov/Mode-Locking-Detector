//
// Register-mapped read-only I2C2 slave interface.
//

#ifndef MLDS_G4_I2C_SLAVE_H
#define MLDS_G4_I2C_SLAVE_H

#include <stdint.h>
#include "stm32g4xx_hal.h"
#include "laser.h"
#include "CV.h"

//Registers
typedef struct __attribute__((packed)) {
                                    // Start Addresses
    uint8_t status;                 // 0x00
    float   ml_rms_mv;              // 0x01
    float   cw_mv;                  // 0x05
    float   current_cv;             // 0x09
    float   current_cv_threshold;   // 0x0D
    float   mse;                    // 0x11
    float   r_squared;              // 0x15
} i2c_reg_map_t;

//Type punning for 8-bit chunk access
typedef union {
    i2c_reg_map_t fields;
    uint8_t       bytes[sizeof(i2c_reg_map_t)];
} i2c_reg_union_t;

#define I2C_REG_TOTAL_SIZE ((uint8_t)sizeof(i2c_reg_map_t))

void i2c_slave_init(I2C_HandleTypeDef *hi2c, const laser_calibration_t *cal);
void i2c_slave_sync_ro(const laser_state_t *state, const sliding_cv_t *scv,
                       const laser_calibration_t *cal);

#endif // MLDS_G4_I2C_SLAVE_H
