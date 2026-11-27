#ifndef __SSD1306_H
#define __SSD1306_H

#include "stm32f4xx_hal.h"   // Adjust for your MCU family (e.g., stm32f1xx_hal.h)
#include <stdint.h>
#include <stddef.h>

/* Display dimensions for SSD1306 */
#define SSD1306_WIDTH   128
#define SSD1306_HEIGHT  64

/* I2C Address (0x3C shifted left for HAL = 0x78) */
#define SSD1306_ADDR    (0x3C << 1)

/* Public API */
void SSD1306_Init(I2C_HandleTypeDef *hi2c);
void SSD1306_Clear(void);
void SSD1306_UpdateScreen(void);
void SSD1306_SetCursor(uint8_t x, uint8_t y);
void SSD1306_WriteChar(char ch);
void SSD1306_WriteString(char *str);
void SSD1306_WriteChar_12x16(char ch);
void SSD1306_WriteString_12x16(char *str);

#endif /* __SSD1306_H */
