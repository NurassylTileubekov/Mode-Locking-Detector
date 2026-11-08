//
// Created by nurassyl on 8.11.2026.
//
#include "main.h"
#include  "i2c_slave.h"
#include <stdio.h>
#include "string.h"

extern I2C_HandleTypeDef hi2c2;
extern UART_HandleTypeDef huart1;


#define RxSIZE 6
uint8_t RxData[RxSIZE];

int count = 0;

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c) {
    HAL_I2C_EnableListen_IT(hi2c);
}

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode) {
    if (TransferDirection == I2C_DIRECTION_TRANSMIT) {
        HAL_I2C_Slave_Sequential_Receive_IT(hi2c, RxData, RxSIZE, I2C_FIRST_AND_LAST_FRAME);
    }
    else {
        Error_Handler();
    }
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    count++;
    static char message[50];
    sprintf(message, "I2C message recieved\r\n");
    HAL_UART_Transmit_DMA(&huart1, (uint8_t*)message, strlen(message));
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
    HAL_I2C_EnableListen_IT(hi2c);
}