//
// Created by yue on 2024/2/3.
//

#include "uart_recv.h"
#include <string.h>

struct Uart_Recv_t uart_recv;

/**
 * 初始化 - 务必在开始接收之前调用
 */
void uart_recv_init(void)
{
    uart_recv.r = 0;
    uart_recv.w = 0;
    uart_recv.new_flag = 0;
    uart_recv.new_data = 0;
    memset(uart_recv.buf, 0, UART_REV_LEN);
}

/**
 * 从环形缓冲区中读出一个8位数据
 * @param val
 * @return 0:成功 - 1:缓冲区为空
 */
uint8_t uart_recv_read(uint8_t *val)
{
    if (uart_recv.r == uart_recv.w) // 判断是否为空
        return 1;

    *val = uart_recv.buf[uart_recv.r];  // 读

    uart_recv.r = (uart_recv.r + 1) % UART_REV_LEN; // 移动读指针

    return 0;
}

/**
 * 将一个8位数据写入环形缓冲区
 * @param val
 * @return 0:成功 - 1:缓冲区已满
 */
uint8_t uart_recv_write(uint8_t val)
{
    if(((uart_recv.w + 1) % UART_REV_LEN) == uart_recv.r)   // 判断是否已满
        return 1;

    uart_recv.buf[uart_recv.w] = val;   // 写

    uart_recv.w = (uart_recv.w + 1) % UART_REV_LEN; // 移动写指针

    return 0;
}

/**
 * 判断是否有接收到新的数据
 * @return 0:接收到新数据 - 1:没有接收到新数据
 */
uint8_t uart_recv_new(void)
{
    if (uart_recv.new_flag == 0)
        return 1;

    uart_recv.new_flag = 0; // 将标志=0

    return 0;
}
