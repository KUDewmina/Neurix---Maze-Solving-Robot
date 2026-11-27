/*
 * uart_transmit.h
 *
 *  Created on: Dec 30, 2025
 *      Author: lenovo
 */

#ifndef INC_UART_TRANSMIT_H_
#define INC_UART_TRANSMIT_H_


#include "stm32f4xx_hal.h" // change to your STM32 series if needed

// Functions to transmit numbers and strings via UART
void UART_SendString(UART_HandleTypeDef *huart, const char *str, uint8_t newline);
void UART_SendNumber(UART_HandleTypeDef *huart, int num, uint8_t newline);
void UART_SendFloat(UART_HandleTypeDef *huart, float num, uint8_t newline);
void float_to_str(float num, char *str, int decimals);
void int_to_str(int num, char *str);

#endif // UART_TRANSMIT_H



