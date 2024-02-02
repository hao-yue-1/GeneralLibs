//
// Created by yue on 2024/2/2.
//

#include "lcd_printf.h"
#include "lcd.h"

#include <stdio.h>
#include <stdarg.h>
#include <string.h>

extern uint16_t lcd_color_char;     // 字符颜色
extern uint16_t lcd_color_back;    // 背景颜色

/* 默认字符大小为 8*16(w*h) 根据字符大小将屏幕划分为30*8的网格 在网格内绘制字符 */
static uint8_t lcd_x_arr[30] = {0, 8, 16, 24, 32, 40, 48, 56, 64,
                       72, 80, 88, 96, 104, 112, 120, 128,
                       136, 144, 152, 160, 168, 176, 184,
                       192, 200, 208, 216, 224, 232};
static uint8_t lcd_y_arr[8] = {0, 16, 32, 48, 64, 80, 96, 112};

/* 网格屏幕字符绘制坐标 */
static uint8_t lcd_x_p = 0;
static uint8_t lcd_y_p = 0;

static uint8_t flag_point = 0;  // 在当前输出行尾部绘制一个标记 (0:当前为新行,标记未绘制 - 1:当前为旧行,标记已绘制)

static void lcd_clear_line(uint8_t y)
{
    lcd_fill(0, lcd_y_arr[y], LCD_W - 1, lcd_y_arr[y] + 16, lcd_color_back);
}

/**
 * 显示一个字符并更新坐标
 * @param val
 */
static void lcd_putchar_update_point(char val)
{
    lcd_print_char(lcd_x_arr[lcd_x_p], lcd_y_arr[lcd_y_p], val, 0, lcd_color_char);
    /* 左移准备下一个字符 */
    lcd_x_p++;
    /* 行尾 换行 */
    if (lcd_x_p > 27)   // 预留尾部两个字符的位置用于绘制当前行标记
    {
        lcd_x_p = 0;
        lcd_y_p++;
        flag_point = 0;
        lcd_clear_line(lcd_y_p);
    }
    /* 页面底部 重新开始 */
    if (lcd_y_p > 7)
    {
        lcd_y_p = 0;
        flag_point = 0;
        lcd_clear_line(lcd_y_p);
    }
}

/**
 * 屏幕铺满后重新从顶部开始显示 当前显示的行行尾会有标记
 * @param format
 * @param ...
 */
void lcd_printf(const char* format, ...)
{
    /* 绘制当前行行尾标记 */
    if (flag_point == 0)
    {
        if (lcd_y_p == 0)   // 当前为第一行 清除最底部一行标记
            lcd_fill(224, lcd_y_arr[7], 239, lcd_y_arr[7]+16, lcd_color_back);
        else                // 清除上一行标记
            lcd_fill(224, lcd_y_arr[lcd_y_p]-16, 239, lcd_y_arr[lcd_y_p], lcd_color_back);

        // 绘制当前行标记
        lcd_fill(224+4, lcd_y_arr[lcd_y_p]+4, 224+11, lcd_y_arr[lcd_y_p]+11, lcd_color_char);
        flag_point = 1;
    }

    va_list args;
    va_start(args, format);

    for (int i = 0; format[i] != '\0'; ++i)
    {
        if (format[i] == '%')
        {
            switch (format[++i])
            {
                case 'd':
                {
                    int d = va_arg(args, int);
                    char d_str[16] = {0};
                    sprintf(d_str, "%d", d);
                    unsigned int len = strlen(d_str);
                    for (int j = 0; j < len; ++j)
                    {
                        lcd_putchar_update_point(d_str[j]);
                    }
                    break;
                }
                case 'x':
                {
                    int x = va_arg(args, int);
                    char x_str[16] = {0};
                    sprintf(x_str, "%x", x);
                    unsigned int len = strlen(x_str);
                    for (int j = 0; j < len; ++j)
                    {
                        lcd_putchar_update_point(x_str[j]);
                    }
                    break;
                }
                case 'f':
                {
                    double f = va_arg(args, double);
                    printf("f = %f\r\n", f);
                    char f_str[16] = {0};
                    sprintf(f_str, "%f", f);
                    unsigned int len = strlen(f_str);
                    for (int j = 0; j < len; ++j)
                    {
                        lcd_putchar_update_point(f_str[j]);
                    }
                    break;
                }
                case 'c':
                {
                    int c = va_arg(args, int);
                    lcd_putchar_update_point(c);
                    break;
                }
                case 's':
                {
                    char* p = va_arg(args, char*);
                    unsigned int len = strlen(p);
                    for (int j = 0; j < len; ++j)
                    {
                        lcd_putchar_update_point(p[j]);
                    }
                    break;
                }
                default:
                {
                    lcd_putchar_update_point(format[i]);
                    break;
                }
            }
        }
        else if (format[i] == '\r') // 回到当前行开头位置
        {
            lcd_x_p = 0;
        }
        else if (format[i] == '\n') // 切换到下一行
        {
            lcd_y_p++;
            if (lcd_y_p > 7)
            {
                lcd_y_p = 0;
            }
            flag_point = 0;
        }
        else
        {
            lcd_putchar_update_point(format[i]);
        }
    }

    va_end(args);
}

/**
 * 设置字符颜色
 * @param color
 */
void lcd_color_char_set(uint16_t color)
{
    lcd_color_char = color;
}
