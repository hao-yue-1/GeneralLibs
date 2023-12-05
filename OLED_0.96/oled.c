//
// Created by yue on 2023/12/5.
//

#include "oled.h"
#include "oled_font.h"
#include "i2c.h"

/**
 * OLED - 初始化命令 根据数据手册书写
 */
const unsigned char oled_init_cmd[25]=
{
    0xAE,//关闭显示
    0xD5,//设置时钟分频因子,震荡频率
    0x80,  //[3:0],分频因子;[7:4],震荡频率
    0xA8,//设置驱动路数
    0X3F,//默认0X3F(1/64)
    0xD3,//设置显示偏移
    0X00,//默认为0
    0x40,//设置显示开始行 [5:0],行数.
    0x8D,//电荷泵设置
    0x14,//bit2，开启/关闭
    0x20,//设置内存地址模式
    0x02,//[1:0],00，列地址模式;01，行地址模式;10,页地址模式;默认10;
    0xA1,//段重定义设置,bit0:0,0->0;1,0->127;
    0xC8,//设置COM扫描方向;bit3:0,普通模式;1,重定义模式 COM[N-1]->COM0;N:驱动路数
    0xDA,//设置COM硬件引脚配置
    0x12,//[5:4]配置
    0x81,//对比度设置
    0xEF,//1~255;默认0X7F (亮度设置,越大越亮)
    0xD9,//设置预充电周期
    0xf1,//[3:0],PHASE 1;[7:4],PHASE 2;
    0xDB,//设置VCOMH 电压倍率
    0x30,//[6:4] 000,0.65*vcc;001,0.77*vcc;011,0.83*vcc;
    0xA4,//全局显示开启;bit0:1,开启;0,关闭;(白屏/黑屏)
    0xA6,//设置显示方式;bit0:1,反相显示;0,正常显示
    0xAF,//开启显示
};

/**
 * IIC - 向OLED写命令
 * @param cmd
 * @author WBN
 */
void OLED_SendCmd(unsigned char cmd)
{
    HAL_I2C_Mem_Write(&hi2c2 ,0x78,0x00,I2C_MEMADD_SIZE_8BIT,&cmd,1,0x100);
}

/**
 * IIC - 向OLED写数据
 * @param data 命令
 * @author WBN
 */
void OLED_SendData(unsigned char data)
{
    HAL_I2C_Mem_Write(&hi2c2 ,0x78,0x40,I2C_MEMADD_SIZE_8BIT,&data,1,0x100);
}

/**
 * OLED - 初始化
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
 * OLED - 设置列地址 0-127
 * @param column
 * @author WBN
 */
void OLED_SetColumn(unsigned char column)
{
    OLED_SendCmd(0x10|(column>>4));    //设置列地址高位
    OLED_SendCmd(0x00|(column&0x0f));  //设置列地址低位
}

/**
 * OLED - 设置页地址 0-7
 * @param page
 * @author WBN
 */
void OLED_SetPage(unsigned char page)
{
    OLED_SendCmd(0xb0+page);
}

/**
 * OLED - 清屏 - 颜色黑白可选
 * @param color 0: 黑色 - 1: 白色
 * @author WBN
 */
void OLED_Clear(unsigned char color)
{
    if (color == 0) // 黑色
    {
        color = 0x00;
    }
    else            // 白色
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
 * OLED - 显示图片 - 满屏(128*64)
 * @param ptr_pic 图片数组
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
 * OLED - 设置光标位置
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
*	函 数 名: OLED_ShowStr
*	功能说明: 显示字符串（在字库中的字符）
*	形    参: 1. x,y：起始显示坐标		//y=0,2,4,6（TextSize=2）  y=0,1,2,3,4,5,6,7（TextSize=1）
			  2. ch[]：字符串
			  3. TextSize：字符大小（1:6*8  2:8*16）
*	返 回 值: 无
*********************************************************************************************************
*/

/**
 * OLED - 显示字符串
 * @param x 起始坐标 (0-127)
 * @param y 起始坐标 (text_size=1: 0-7 - text_size=2: 0-6)
 * @param data 字符串
 * @param text_size 字符大小 (0: 6*8 - 1: 8*16)
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
