//
// Created by yue on 2024/2/3.
//

#ifndef LCD_PRINTF_UART_RECV_H
#define LCD_PRINTF_UART_RECV_H

#include <stdint.h>

/**
 * README
 * ��ģ��ּ���ṩһ�����ݽ��յĻ��λ������Ľӿڣ�ʵ������ѭ�����еĸİ�
 * ʹ�ø�ģ��֮ǰ����Ҫ�ں��ʵĵط����ж�or����������uart_recv_write�����ڽ��յ�������д�뻺����
 * ��uart_recv.new_data�⣬������ֱ�ӷ��ʽṹ���е��������������ͨ��API�������ݶ�д���ж�
 */

/**
 * ���λ���������
 * ���ڸ�ģ���ѭ�����в��ã�����һ��Ԫ�ؿռ䣬Լ��read��write����һλ����Ϊ������״̬
 * ��˻�������ʵ�ʴ�СΪUART_REV_LEN - 1
 */
#define UART_REV_LEN 8

struct Uart_Recv_t
{
    uint8_t buf[UART_REV_LEN];
    uint16_t r;
    uint16_t w;
    uint8_t new_flag;   // ����������д��ʱ=1, �û����������ʸ����ݺ���ؽ���=0, ����ͨ��uart_recv_new���з����ж�
    uint8_t new_data;   // �������»�ȡ������ ��Ҫ��������STM32 HAL��
};

extern struct Uart_Recv_t uart_recv;

void uart_recv_init(void);
uint8_t uart_recv_read(uint8_t *val);
uint8_t uart_recv_write(uint8_t val);
uint8_t uart_recv_new(void);

#endif //LCD_PRINTF_UART_RECV_H
