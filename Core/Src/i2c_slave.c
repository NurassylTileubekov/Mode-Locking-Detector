//
// Created by nurassyl on 8.11.2026.
//
#include "main.h"
#include  "i2c_slave.h"
#include <stdio.h>
#include "string.h"

extern I2C_HandleTypeDef hi2c2;
extern UART_HandleTypeDef huart1;


#define RX_SIZE 6
uint8_t rx_data[RX_SIZE];
uint8_t rx_count = 0;

int is_first_received = 0;
int count_addr = 0;
int count_rx_cplt = 0;
int count_error = 0;

void process_data(void);

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c) {
    HAL_I2C_EnableListen_IT(hi2c);
}

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode) {
    if (TransferDirection == I2C_DIRECTION_TRANSMIT) {
        if (is_first_received == 0) {
            rx_count = 0;
            count_addr++;
            HAL_I2C_Slave_Seq_Receive_IT(hi2c, rx_data, RX_SIZE, I2C_FIRST_FRAME);
        }
    }
    else {
        Error_Handler();
    }
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c) {

    if (is_first_received == 0) {
        rx_count++;
        is_first_received = 1;
        HAL_I2C_Slave_Seq_Receive_IT(hi2c, rx_data + rx_count, rx_data[0], I2C_LAST_FRAME);
    }
    else {
        rx_count += rx_data[0];
        is_first_received = 0;
        process_data();
    }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
    count_error++;
    uint32_t error_code = HAL_I2C_GetError(hi2c);
    if (error_code == 4) { // Acknowledgement Failure Error
        process_data();
    }


    HAL_I2C_EnableListen_IT(hi2c);
}

void process_data(void) {

}