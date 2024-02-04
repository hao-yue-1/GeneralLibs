//
// Created by yue on 2024/2/3.
//

#ifndef LCD_PRINTF_UART_RECV_H
#define LCD_PRINTF_UART_RECV_H

#include <stdint.h>

/**
 * README
 * 该模块旨在提供一个数据接收的环形缓冲区的接口，实际上是循环队列的改版
 * 使用该模块之前，需要在合适的地方（中断or遍历）调用uart_recv_write将串口接收到的数据写入缓冲区
 * 除uart_recv.new_data外，不建议直接访问结构体中的其他变量，务必通过API进行数据读写和判断
 */

/**
 * 环形缓冲区长度
 * 由于该模块的循环队列采用：少用一个元素空间，约定read在write的下一位置作为队列满状态
 * 因此缓冲区的实际大小为UART_REV_LEN - 1
 */
#define UART_REV_LEN 8

struct Uart_Recv_t
{
    uint8_t buf[UART_REV_LEN];
    uint16_t r;
    uint16_t w;
    uint8_t new_flag;   // 当有新数据写入时=1, 用户若主动访问该数据后务必将其=0, 建议通过uart_recv_new进行访问判断
    uint8_t new_data;   // 保存最新获取的数据 主要用于适配STM32 HAL库
};

extern struct Uart_Recv_t uart_recv;

void uart_recv_init(void);
uint8_t uart_recv_read(uint8_t *val);
uint8_t uart_recv_write(uint8_t val);
uint8_t uart_recv_new(void);

#endif //LCD_PRINTF_UART_RECV_H
