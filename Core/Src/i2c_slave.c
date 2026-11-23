//
// Interrupt-driven, strictly read-only register-mapped I2C2 slave.
//

#include "i2c_slave.h"
#include "main.h"
#include <string.h>

static i2c_reg_union_t live_regs;
static i2c_reg_union_t tx_snapshot;

volatile uint8_t reg_ptr = 0;
volatile uint32_t i2c_slave_bus_err_count = 0;
volatile uint32_t i2c_slave_last_error = 0;
static uint8_t rx_byte = 0;
static uint8_t dummy_byte = 0x00;

void i2c_slave_init(I2C_HandleTypeDef *hi2c, const laser_calibration_t *cal)
{
    memset(&live_regs, 0, sizeof(live_regs));
    reg_ptr = 0;

    live_regs.fields.mse       = cal->mse;
    live_regs.fields.r_squared = cal->r_squared;

    if (HAL_I2C_EnableListen_IT(hi2c) != HAL_OK) {
        Error_Handler();
    }
}

void i2c_slave_sync_ro(const laser_state_t *state, const sliding_cv_t *scv,
                       const laser_calibration_t *cal)
{
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
        HAL_I2C_Slave_Seq_Receive_IT(hi2c, &rx_byte, 1, I2C_FIRST_FRAME);
    } else {
        uint8_t ptr = reg_ptr;
        tx_snapshot = live_regs;
        if (ptr < I2C_REG_TOTAL_SIZE) {
            uint16_t bytes_to_send = (uint16_t)I2C_REG_TOTAL_SIZE - ptr;
            HAL_I2C_Slave_Seq_Transmit_IT(hi2c, &tx_snapshot.bytes[ptr], bytes_to_send, I2C_FIRST_FRAME);
        } else {
            HAL_I2C_Slave_Seq_Transmit_IT(hi2c, &dummy_byte, 1, I2C_FIRST_FRAME);
        }
    }
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    (void)hi2c;
    reg_ptr = rx_byte;
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->State == HAL_I2C_STATE_READY) {
        HAL_I2C_EnableListen_IT(hi2c);
        return;
    }

    if (hi2c->XferISR != NULL) {
        HAL_I2C_Slave_Seq_Transmit_IT(hi2c, &dummy_byte, 1, I2C_NEXT_FRAME);
    }
}

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
    HAL_I2C_EnableListen_IT(hi2c);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    uint32_t err = hi2c->ErrorCode;

    if ((err & ~HAL_I2C_ERROR_AF) != 0U) {
        i2c_slave_bus_err_count++;
        i2c_slave_last_error = err;
        if (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_TXE) == RESET) {
            __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_TXE);
        }
    }

    hi2c->ErrorCode = HAL_I2C_ERROR_NONE;

    if (hi2c->State == HAL_I2C_STATE_READY) {
        HAL_I2C_EnableListen_IT(hi2c);
    }
}
