//
// Created by yue on 2023/12/5.
//

#include "oled.h"
#include "oled_font.h"
#include "i2c.h"

/**
 * OLED - ��ʼ������ ���������ֲ���д
 */
const unsigned char oled_init_cmd[25]=
{
    0xAE,//�ر���ʾ
    0xD5,//����ʱ�ӷ�Ƶ����,��Ƶ��
    0x80,  //[3:0],��Ƶ����;[7:4],��Ƶ��
    0xA8,//��������·��
    0X3F,//Ĭ��0X3F(1/64)
    0xD3,//������ʾƫ��
    0X00,//Ĭ��Ϊ0
    0x40,//������ʾ��ʼ�� [5:0],����.
    0x8D,//��ɱ�����
    0x14,//bit2������/�ر�
    0x20,//�����ڴ��ַģʽ
    0x02,//[1:0],00���е�ַģʽ;01���е�ַģʽ;10,ҳ��ַģʽ;Ĭ��10;
    0xA1,//���ض�������,bit0:0,0->0;1,0->127;
    0xC8,//����COMɨ�跽��;bit3:0,��ͨģʽ;1,�ض���ģʽ COM[N-1]->COM0;N:����·��
    0xDA,//����COMӲ����������
    0x12,//[5:4]����
    0x81,//�Աȶ�����
    0xEF,//1~255;Ĭ��0X7F (��������,Խ��Խ��)
    0xD9,//����Ԥ�������
    0xf1,//[3:0],PHASE 1;[7:4],PHASE 2;
    0xDB,//����VCOMH ��ѹ����
    0x30,//[6:4] 000,0.65*vcc;001,0.77*vcc;011,0.83*vcc;
    0xA4,//ȫ����ʾ����;bit0:1,����;0,�ر�;(����/����)
    0xA6,//������ʾ��ʽ;bit0:1,������ʾ;0,������ʾ
    0xAF,//������ʾ
};

/**
 * IIC - ��OLEDд����
 * @param cmd
 * @author WBN
 */
void OLED_SendCmd(unsigned char cmd)
{
    HAL_I2C_Mem_Write(&hi2c2 ,0x78,0x00,I2C_MEMADD_SIZE_8BIT,&cmd,1,0x100);
}

/**
 * IIC - ��OLEDд����
 * @param data ����
 * @author WBN
 */
void OLED_SendData(unsigned char data)
{
    HAL_I2C_Mem_Write(&hi2c2 ,0x78,0x40,I2C_MEMADD_SIZE_8BIT,&data,1,0x100);
}

/**
 * OLED - ��ʼ��
 * @author WBN
 */
void OLED_Init(void)
{
    for(int i=0; i<25; i++)
    {
        OLED_SendCmd(oled_init_cmd[i]);
    }
}

/**
 * OLED - �����е�ַ 0-127
 * @param column
 * @author WBN
 */
void OLED_SetColumn(unsigned char column)
{
    OLED_SendCmd(0x10|(column>>4));    //�����е�ַ��λ
    OLED_SendCmd(0x00|(column&0x0f));  //�����е�ַ��λ
}

/**
 * OLED - ����ҳ��ַ 0-7
 * @param page
 * @author WBN
 */
void OLED_SetPage(unsigned char page)
{
    OLED_SendCmd(0xb0+page);
}

/**
 * OLED - ���� - ��ɫ�ڰ׿�ѡ
 * @param color 0: ��ɫ - 1: ��ɫ
 * @author WBN
 */
void OLED_Clear(unsigned char color)
{
    if (color == 0) // ��ɫ
    {
        color = 0x00;
    }
    else            // ��ɫ
    {
        color = 0xff;
    }

    for(int page=0; page<8; page++)             //page loop
    {
        OLED_SetPage(page);
        OLED_SetColumn(0);

        for(int column=0; column<128; column++)	//column loop
        {
            OLED_SendData(color);
        }
    }
}

/**
 * OLED - ��ʾͼƬ - ����(128*64)
 * @param ptr_pic ͼƬ����
 * @author WBN
 */
void OLED_PrintPicture(const unsigned char *ptr_pic)
{
    for(int page=0; page<(64/8); page++)        //page loop
    {
        OLED_SetPage(page);
        OLED_SetColumn(0);

        for(int column=0; column<128; column++)	//column loop
        {
            OLED_SendData(*ptr_pic++);
        }
    }
}

/**
 * OLED - ���ù��λ��
 * @param x
 * @param y
 * @author WBN
 */
void OLED_SetPos(unsigned char x, unsigned char y)
{
    OLED_SendCmd(0xb0+y);
    OLED_SendCmd(((x&0xf0)>>4)|0x10);
    OLED_SendCmd((x&0x0f)|0x01);
}

/*
*********************************************************************************************************
*	�� �� ��: OLED_ShowStr
*	����˵��: ��ʾ�ַ��������ֿ��е��ַ���
*	��    ��: 1. x,y����ʼ��ʾ����		//y=0,2,4,6��TextSize=2��  y=0,1,2,3,4,5,6,7��TextSize=1��
			  2. ch[]���ַ���
			  3. TextSize���ַ���С��1:6*8  2:8*16��
*	�� �� ֵ: ��
*********************************************************************************************************
*/

/**
 * OLED - ��ʾ�ַ���
 * @param x ��ʼ���� (0-127)
 * @param y ��ʼ���� (text_size=1: 0-7 - text_size=2: 0-6)
 * @param data �ַ���
 * @param text_size �ַ���С (0: 6*8 - 1: 8*16)
 */
void OLED_PrintStr(unsigned char x, unsigned char y, unsigned char* data, unsigned char text_size)
{
    unsigned char c = 0, i = 0, j = 0;
    switch(text_size)
    {
        case 0:
        {
            while(data[j] != '\0')
            {
                c = data[j] - 32;
                if(x > 126)
                {
                    x = 0;
                    y++;
                }
                OLED_SetPos(x,y);
                for(i=0;i<6;i++)
                    OLED_SendData(F6x8[c][i]);
                x += 6;
                j++;
            }
            break;
        }
        case 1:
        {
            while(data[j] != '\0')
            {
                c = data[j] - 32;
                if(x > 120)
                {
                    x = 0;
                    y++;
                }
                OLED_SetPos(x,y);
                for(i=0;i<8;i++)
                    OLED_SendData(F8X16[c*16+i]);
                OLED_SetPos(x,y+1);
                for(i=0;i<8;i++)
                    OLED_SendData(F8X16[c*16+i+8]);
                x += 8;
                j++;
            }
            break;
        }
        default: break;
    }
}
