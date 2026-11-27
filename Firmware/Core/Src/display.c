/*
 * display.c
 *
 *  Created on: Jul 8, 2026
 *      Author: ASUS
 */
#include "display.h"
#include "ssd1306.h"

void DISPLAY(char *First,
             char *Second);

void DISPLAY(char *First, char *Second)
{
    if (First == NULL || Second == NULL) return;

    SSD1306_Clear();
    SSD1306_SetCursor(0, 0);
    SSD1306_WriteString_12x16(First);
    SSD1306_SetCursor(0, 2);
    SSD1306_WriteString_12x16(Second);
    SSD1306_UpdateScreen();

}
/*
 * display.c
 *
 *  Created on: Jul 31, 2026
 *      Author: ASUS
 */


