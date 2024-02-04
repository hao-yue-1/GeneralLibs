//
// Created by yue on 2024/2/3.
//

#include "uart_recv.h"
#include <string.h>

struct Uart_Recv_t uart_recv;

/**
 * ��ʼ�� - ����ڿ�ʼ����֮ǰ����
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
 * �ӻ��λ������ж���һ��8λ����
 * @param val
 * @return 0:�ɹ� - 1:������Ϊ��
 */
uint8_t uart_recv_read(uint8_t *val)
{
    if (uart_recv.r == uart_recv.w) // �ж��Ƿ�Ϊ��
        return 1;

    *val = uart_recv.buf[uart_recv.r];  // ��

    uart_recv.r = (uart_recv.r + 1) % UART_REV_LEN; // �ƶ���ָ��

    return 0;
}

/**
 * ��һ��8λ����д�뻷�λ�����
 * @param val
 * @return 0:�ɹ� - 1:����������
 */
uint8_t uart_recv_write(uint8_t val)
{
    if(((uart_recv.w + 1) % UART_REV_LEN) == uart_recv.r)   // �ж��Ƿ�����
        return 1;

    uart_recv.buf[uart_recv.w] = val;   // д

    uart_recv.w = (uart_recv.w + 1) % UART_REV_LEN; // �ƶ�дָ��

    return 0;
}

/**
 * �ж��Ƿ��н��յ��µ�����
 * @return 0:���յ������� - 1:û�н��յ�������
 */
uint8_t uart_recv_new(void)
{
    if (uart_recv.new_flag == 0)
        return 1;

    uart_recv.new_flag = 0; // ����־=0

    return 0;
}
