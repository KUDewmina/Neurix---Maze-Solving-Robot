#include "uart_transmit.h"
#include <stdio.h>
#include <string.h>

extern int _printf_float(void);
int use_printf_float = (int)_printf_float;

void UART_SendString(UART_HandleTypeDef *huart, const char *str, uint8_t newline)
{
    HAL_UART_Transmit(huart, (uint8_t*)str, strlen(str), 100);
    if(newline)
    {
        char end[] = "\r\n";
        HAL_UART_Transmit(huart, (uint8_t*)end, 2, 100);
    }
}

void UART_SendNumber(UART_HandleTypeDef *huart, int num, uint8_t newline)
{
    char buffer[12];
    sprintf(buffer, "%d", num);
    HAL_UART_Transmit(huart, (uint8_t*)buffer, strlen(buffer), 100);
    if(newline)
    {
        char end[] = "\r\n";
        HAL_UART_Transmit(huart, (uint8_t*)end, 2, 100);
    }
}

void int_to_str(int num, char *str)
{
    int i = 0;
    int is_negative = 0;

    if (num == 0)
    {
        str[i++] = '0';
        str[i] = '\0';
        return;
    }

    if (num < 0)
    {
        is_negative = 1;
        num = -num;
    }

    while (num != 0)
    {
        str[i++] = (num % 10) + '0';
        num /= 10;
    }

    if (is_negative)
        str[i++] = '-';

    str[i] = '\0';

    // reverse string
    for (int j = 0; j < i / 2; j++)
    {
        char temp = str[j];
        str[j] = str[i - j - 1];
        str[i - j - 1] = temp;
    }
}


void float_to_str(float num, char *str, int decimals)
{
    int int_part;
    float frac_part;
    char int_str[12];
    int i = 0;

    if (num < 0)
    {
        str[i++] = '-';
        num = -num;
    }

    int_part = (int)num;
    frac_part = num - int_part;

    // integer part
    int_to_str(int_part, int_str);
    for (int j = 0; int_str[j] != '\0'; j++)
        str[i++] = int_str[j];

    // decimal point
    str[i++] = '.';

    // fractional part
    for (int j = 0; j < decimals; j++)
    {
        frac_part *= 10;
        int digit = (int)frac_part;
        str[i++] = digit + '0';
        frac_part -= digit;
    }

    str[i] = '\0';
}

void UART_SendFloat(UART_HandleTypeDef *huart, float num, uint8_t newline)
{
    char buffer[12];
    float_to_str(num, buffer, 2);
    HAL_UART_Transmit(huart, (uint8_t*)buffer, strlen(buffer), 100);
    if(newline)
    {
        char end[] = "\r\n";
        HAL_UART_Transmit(huart, (uint8_t*)end, 2, 100);
    }
}

