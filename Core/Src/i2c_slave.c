//
// Interrupt-driven, strictly read-only register-mapped I2C2 slave.
//

#include "i2c_slave.h"
#include "main.h"
#include <string.h>

static i2c_reg_union_t live_regs;
static i2c_reg_union_t tx_snapshot;

static uint8_t reg_ptr = 0;
static uint8_t reg_offset = 0;
static uint8_t rx_len  = 0;
static uint8_t rx_byte = 0;
static uint8_t tx_byte = 0;

static uint8_t i2c_slave_next_tx_byte(void)
{
    uint16_t idx = (uint16_t)reg_ptr + reg_offset;
    if (idx < I2C_REG_TOTAL_SIZE) {
        reg_offset++;
        return tx_snapshot.bytes[idx];
    }
    // Clamp past-end reads to 0x00 without incrementing reg_offset so uint8_t cannot wrap.
    return 0x00;
}

void i2c_slave_init(I2C_HandleTypeDef *hi2c, const laser_calibration_t *cal)
{
    memset(&live_regs, 0, sizeof(live_regs));
    memset(&tx_snapshot, 0, sizeof(tx_snapshot));
    reg_ptr = 0;
    reg_offset = 0;
    rx_len  = 0;
    rx_byte = 0;
    tx_byte = 0;

    live_regs.fields.mse       = cal->mse;
    live_regs.fields.r_squared = cal->r_squared;

    if (HAL_I2C_EnableListen_IT(hi2c) != HAL_OK) {
        Error_Handler();
    }
}

void i2c_slave_sync_ro(const laser_state_t *state, const sliding_cv_t *scv,
                       const laser_calibration_t *cal)
{
    // Mask IRQs briefly so floats cannot tear if read by AddrCallback mid-update.
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    live_regs.fields.status               = (uint8_t)state->status;
    live_regs.fields.ml_rms_mv            = state->ml_rms_mv;
    live_regs.fields.cw_mv                = state->cw_mv;
    live_regs.fields.current_cv           = scv->current_cv;
    live_regs.fields.current_cv_threshold = state->current_cv_threshold;
    live_regs.fields.mse                  = cal->mse;
    live_regs.fields.r_squared            = cal->r_squared;

    if (!primask) {
        __enable_irq();
    }
}

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
    if (TransferDirection == I2C_DIRECTION_TRANSMIT) {
        rx_len = 0;
        HAL_I2C_Slave_Seq_Receive_IT(hi2c, &rx_byte, 1, I2C_FIRST_FRAME);
    } else {
        reg_offset = 0;
        tx_snapshot = live_regs;
        tx_byte = i2c_slave_next_tx_byte();
        HAL_I2C_Slave_Seq_Transmit_IT(hi2c, &tx_byte, 1, I2C_FIRST_FRAME);
    }
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    // Only the first byte sets the register pointer, extra write bytes are ignored.
    if (rx_len == 0) {
        reg_offset = 0;
        reg_ptr = rx_byte;
        rx_len  = 1;
    }

    HAL_I2C_Slave_Seq_Receive_IT(hi2c, &rx_byte, 1, I2C_NEXT_FRAME);
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    tx_byte = i2c_slave_next_tx_byte();
    HAL_I2C_Slave_Seq_Transmit_IT(hi2c, &tx_byte, 1, I2C_NEXT_FRAME);
}

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
    rx_len = 0;
    HAL_I2C_EnableListen_IT(hi2c);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->ErrorCode == HAL_I2C_ERROR_AF) {
        hi2c->ErrorCode = HAL_I2C_ERROR_NONE;
    }

    if (hi2c->State == HAL_I2C_STATE_READY) {
        rx_len = 0;
        HAL_I2C_EnableListen_IT(hi2c);
    }
}
