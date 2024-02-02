//
// Created by yue on 2024/2/2.
//

#ifndef LCD_PRINTF_LCD_PRINTF_H
#define LCD_PRINTF_LCD_PRINTF_H

#include <stdint.h>

void lcd_printf(const char* format, ...);
void lcd_color_char_set(uint16_t color);

#endif //LCD_PRINTF_LCD_PRINTF_H
