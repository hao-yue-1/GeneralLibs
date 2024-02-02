/*
* 简介: ICM20602 IMU 姿态传感器
        该芯片支持直接读出六轴数据 三轴陀螺仪三轴加速度计
		支持IIC、SPI通信
* 作者: WBN
* 时间: 2022/11/8
*/

#include "ICM20602.h"
#include "SEEKFREE_IIC.h"
#include "stm32f1xx_hal.h"	//HAL_Delay
#include "main.h"			//CS引脚宏
#include "spi.h"			//SPI通信

/* 下面是用软件IIC通信 */

/*
 *******************************************************************************************
 ** 函数功能: ICM20602 软件IIC写寄存器
 ** 参    数: 1. dev_add: 设备地址 低七位地址
			  2. reg: 寄存器地址
			  3. dat: 写入的数据
 ** 返 回 值: 无
 ** 作    者: WBN
 ********************************************************************************************
 */ 
void ICMWriteRegByte_IIC(uint8_t dev_add, uint8_t reg, uint8_t dat)
{	
	simiic_start();
    send_ch( (dev_add<<1) | 0x00);   //发送器件地址加写位
	send_ch( reg );   				 //发送从机寄存器地址
	send_ch( dat );   				 //发送需要写入的数据
	simiic_stop();
}

/*
 *******************************************************************************************
 ** 函数功能: ICM20602 软件IIC读寄存器 单字节
 ** 参    数: 1. dev_add: 设备地址 低七位地址
			  2. reg:     寄存器地址
 ** 返 回 值: 寄存器的数据
 ** 作    者: WBN
 ********************************************************************************************
 */
uint8_t ICMReadRegByte_IIC(uint8_t dev_add, uint8_t reg)
{	
	uint8_t dat;
	simiic_start();
    send_ch( (dev_add<<1) | 0x00);  //发送器件地址加写位
	send_ch( reg );   				//发送从机寄存器地址
	
	simiic_start();
	send_ch( (dev_add<<1) | 0x01);  //发送器件地址加读位
	dat = read_ch(0);   				//读取数据
	simiic_stop();
	
	return dat;
}

/*
 *******************************************************************************************
 ** 函数功能: ICM20602 软件IIC读寄存器 多字节
 ** 参    数: 1. dev_add:  设备地址 低七位地址
			  2. reg:      寄存器地址
			  3. *dat_add: 读取数据保存的地址指针
			  4. num:	   读取字节数量
 ** 返 回 值: 寄存器的数据
 ** 作    者: WBN
 ********************************************************************************************
 */
void ICMReadRegByteS_IIC(uint8_t dev_add, uint8_t reg, uint8_t *dat_add, uint8_t num)
{
	simiic_start();
    send_ch( (dev_add<<1) | 0x00);  //发送器件地址加写位
	send_ch( reg );   				//发送从机寄存器地址
	
	simiic_start();
	send_ch( (dev_add<<1) | 0x01);  //发送器件地址加读位
    while(--num)
    {
        *dat_add = read_ch(1); //读取数据
        dat_add++;
    }
    *dat_add = read_ch(0); //读取数据
	simiic_stop();
}

/*
 *******************************************************************************************
 ** 函数功能: ICM20602自检函数
 ** 参    数: 无
 ** 返 回 值: 无
 ** 作    者: WBN
 ********************************************************************************************
 */ 
void ICM20602Self_IIC(void)
{
    uint8_t dat=0;
    while(dat != 0x12)   //读取ICM20602 ID
    {
        dat = ICMReadRegByte_IIC(ICM20602_DEV_ADDR,ICM20602_WHO_AM_I);
        HAL_Delay(10);
    }
}

/*
 *******************************************************************************************
 ** 函数功能: 软件IIC，初始化ICM20602
 ** 参    数: 无
 ** 返 回 值: 无
 ** 作    者: WBN
 ********************************************************************************************
 */ 
void ICM20602Init_IIC(void)
{
    HAL_Delay(10);  		//上电延时
    ICM20602Self_IIC();		//检测
    //复位
    ICMWriteRegByte_IIC(ICM20602_DEV_ADDR,ICM20602_PWR_MGMT_1,0x80);	//复位设备
    HAL_Delay(2);	//延时
    while(0x80 & ICMReadRegByte_IIC(ICM20602_DEV_ADDR,ICM20602_PWR_MGMT_1));//等待复位完成
    //配置参数
    ICMWriteRegByte_IIC(ICM20602_DEV_ADDR,ICM20602_PWR_MGMT_1,0x01);               //时钟设置
    ICMWriteRegByte_IIC(ICM20602_DEV_ADDR,ICM20602_PWR_MGMT_2,0x00);               //开启陀螺仪和加速度计
    ICMWriteRegByte_IIC(ICM20602_DEV_ADDR,ICM20602_CONFIG,0x01);                   //176HZ 1KHZ
    ICMWriteRegByte_IIC(ICM20602_DEV_ADDR,ICM20602_SMPLRT_DIV,0x07);               //采样速率 SAMPLE_RATE = INTERNAL_SAMPLE_RATE / (1 + SMPLRT_DIV)
    ICMWriteRegByte_IIC(ICM20602_DEV_ADDR,ICM20602_GYRO_CONFIG,0x18);              //±2000 dps
    ICMWriteRegByte_IIC(ICM20602_DEV_ADDR,ICM20602_ACCEL_CONFIG,0x10);             //±8g
    ICMWriteRegByte_IIC(ICM20602_DEV_ADDR,ICM20602_ACCEL_CONFIG_2,0x03);           //Average 4 samples   44.8HZ   //0x23 Average 16 samples
	//ICM20602_GYRO_CONFIG寄存器
    //设置为:0x00 陀螺仪量程为:±250 dps     获取到的陀螺仪数据除以131           可以转化为带物理单位的数据， 单位为：°/s
    //设置为:0x08 陀螺仪量程为:±500 dps     获取到的陀螺仪数据除以65.5          可以转化为带物理单位的数据，单位为：°/s
    //设置为:0x10 陀螺仪量程为:±1000dps     获取到的陀螺仪数据除以32.8          可以转化为带物理单位的数据，单位为：°/s
    //设置为:0x18 陀螺仪量程为:±2000dps     获取到的陀螺仪数据除以16.4          可以转化为带物理单位的数据，单位为：°/s
    //ICM20602_ACCEL_CONFIG寄存器
    //设置为:0x00 加速度计量程为:±2g          获取到的加速度计数据 除以16384      可以转化为带物理单位的数据，单位：g(m/s^2)
    //设置为:0x08 加速度计量程为:±4g          获取到的加速度计数据 除以8192       可以转化为带物理单位的数据，单位：g(m/s^2)
    //设置为:0x10 加速度计量程为:±8g          获取到的加速度计数据 除以4096       可以转化为带物理单位的数据，单位：g(m/s^2)
    //设置为:0x18 加速度计量程为:±16g         获取到的加速度计数据 除以2048       可以转化为带物理单位的数据，单位：g(m/s^2)
}

/*
 *******************************************************************************************
 ** 函数功能: 获取ICM20602加速度计数据
 ** 参    数: 1. x: 加速度计X轴数据
			  2. y: 加速度计Z轴数据
			  3. z: 加速度计Y轴数据
 ** 返 回 值: 无
 ** 作    者: WBN
 ********************************************************************************************
 */ 
void ICM20602GetAcc_IIC(int16_t *x,int16_t *y,int16_t *z)
{
    uint8_t dat[6];
    
    ICMReadRegByteS_IIC(ICM20602_DEV_ADDR, ICM20602_ACCEL_XOUT_H, dat, 6);  
    *x = (int16_t)(((uint16_t)dat[0]<<8 | dat[1]));
    *y = (int16_t)(((uint16_t)dat[2]<<8 | dat[3]));
    *z = (int16_t)(((uint16_t)dat[4]<<8 | dat[5]));
}

/*
 *******************************************************************************************
 ** 函数功能: 获取ICM20602陀螺仪数据
 ** 参    数: 1. x: 陀螺仪X轴数据
			  2. y: 陀螺仪Z轴数据
			  3. z: 陀螺仪Y轴数据
 ** 返 回 值: 无
 ** 作    者: WBN
 ********************************************************************************************
 */ 
void ICM20602GetGyro_IIC(int16_t *x,int16_t *y,int16_t *z)
{
    uint8_t dat[6];

    ICMReadRegByteS_IIC(ICM20602_DEV_ADDR, ICM20602_GYRO_XOUT_H, dat, 6);  
    *x = (int16_t)(((uint16_t)dat[0]<<8 | dat[1]));
    *y = (int16_t)(((uint16_t)dat[2]<<8 | dat[3]));
    *z = (int16_t)(((uint16_t)dat[4]<<8 | dat[5]));
}

/*
 *******************************************************************************************
 ** 函数功能: 初始化ICM20602陀螺仪零漂
 ** 参    数: 1. *handle: 零漂数据
			  2. dt:      采样时间间隔 单位ms
              3. mode:    通信方式 0.软件IIC 1.SPI
 ** 返 回 值: 无
 ** 作    者: WBN
 ********************************************************************************************
 */ 
void ICMGyroErrorInit(ICMGyroError *handle,int dt,uint8_t mode)
{
	handle->gyro_x=0;
	handle->gyro_y=0;
	handle->gyro_z=0;
	
	int16_t gyro_x,gyro_y,gyro_z;
	for(int i=0;i<100;i++)
	{
		if(mode==0)			//IIC
		{
			ICM20602GetGyro_IIC(&gyro_x,&gyro_y,&gyro_z);
		}
		else if(mode==1)	//SPI
		{
			ICM20602GetGyro_SPI(&gyro_x,&gyro_y,&gyro_z);
		}
		else				//ERROR
		{
			break;
		}
		handle->gyro_x+=gyro_x;
		handle->gyro_y+=gyro_y;
		handle->gyro_z+=gyro_z;
		HAL_Delay(dt);
	}
	
	handle->gyro_x/=100;
	handle->gyro_y/=100;
	handle->gyro_z/=100;
}

/* 下面是用SPI通信 */

/*
 *******************************************************************************************
 ** 函数功能: ICM20602 SPI写寄存器 单字节
 ** 参    数: 1. cmd: 寄存器地址
			  2. val: 写入的数据
 ** 返 回 值: 无
 ** 作    者: WBN
 ********************************************************************************************
 */ 
void ICMWriteRegByte_SPI(uint8_t cmd, uint8_t val)
{	
    uint8_t dat[2];
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin,GPIO_PIN_RESET);
	
    dat[0] = cmd | ICM20602_SPI_W;
    dat[1] = val;
    HAL_SPI_Transmit(&hspi1,dat,2,500);

	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin,GPIO_PIN_SET);
}

/*
 *******************************************************************************************
 ** 函数功能: ICM20602 SPI读寄存器 单字节
 ** 参    数: 1. cmd:  寄存器地址
			  2. *val: 接收数据的地址
 ** 返 回 值: 无
 ** 作    者: WBN
 ********************************************************************************************
 */ 
void ICMReadRegByte_SPI(uint8_t cmd, uint8_t *val)
{	
    uint8_t dat[2];

	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin,GPIO_PIN_RESET);
    dat[0] = cmd | ICM20602_SPI_R;
    dat[1] = *val;
	HAL_SPI_TransmitReceive(&hspi1,dat,dat,2,500);
	
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin,GPIO_PIN_SET);

    *val = dat[1];
}

/*
 *******************************************************************************************
 ** 函数功能: ICM20602 SPI读寄存器 多字节
 ** 参    数: 1. *val: 接收数据的地址
			  2. num: 读取数据数量
 ** 返 回 值: 无
 ** 作    者: WBN
 ********************************************************************************************
 */ 
void ICMReadRegByteS_SPI(uint8_t * val, uint8_t num)
{	
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin,GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1,val,val,num,500);
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin,GPIO_PIN_SET);
}

/*
 *******************************************************************************************
 ** 函数功能: ICM20602自检函数
 ** 参    数: 无
 ** 返 回 值: 无
 ** 作    者: WBN
 ********************************************************************************************
 */ 
void ICM20602Self_SPI(void)
{
    uint8_t dat=0;
    while(0x12 != dat)   //读取ICM20602 ID
    {
        ICMReadRegByte_SPI(ICM20602_WHO_AM_I,&dat);
        HAL_Delay(10);
    }
}
     
/*
 *******************************************************************************************
 ** 函数功能: 初始化ICM20602
 ** 参    数: 无
 ** 返 回 值: 无
 ** 作    者: WBN
 ********************************************************************************************
 */ 
void ICM20602Init_SPI(void)
{
    uint8_t val = 0x0;

    HAL_Delay(10);  	//上电延时
    ICM20602Self_SPI();	//检测
    ICMWriteRegByte_SPI(ICM20602_PWR_MGMT_1,0x80);//复位设备
    HAL_Delay(2);
    do	//等待复位成功
    {
        ICMReadRegByte_SPI(ICM20602_PWR_MGMT_1,&val);
    }while(val != 0x41);
    
    ICMWriteRegByte_SPI(ICM20602_PWR_MGMT_1,     0x01);            //时钟设置
    ICMWriteRegByte_SPI(ICM20602_PWR_MGMT_2,     0x00);            //开启陀螺仪和加速度计
    ICMWriteRegByte_SPI(ICM20602_CONFIG,         0x01);            //176HZ 1KHZ
    ICMWriteRegByte_SPI(ICM20602_SMPLRT_DIV,     0x07);            //采样速率 SAMPLE_RATE = INTERNAL_SAMPLE_RATE / (1 + SMPLRT_DIV)
    ICMWriteRegByte_SPI(ICM20602_GYRO_CONFIG,    0x18);            //±2000 dps
    ICMWriteRegByte_SPI(ICM20602_ACCEL_CONFIG,   0x10);            //±8g
    ICMWriteRegByte_SPI(ICM20602_ACCEL_CONFIG_2, 0x03);            //Average 4 samples   44.8HZ   //0x23 Average 16 samples
	//ICM20602_GYRO_CONFIG寄存器
    //设置为:0x00 陀螺仪量程为:±250 dps     获取到的陀螺仪数据除以131           可以转化为带物理单位的数据， 单位为：°/s
    //设置为:0x08 陀螺仪量程为:±500 dps     获取到的陀螺仪数据除以65.5          可以转化为带物理单位的数据，单位为：°/s
    //设置为:0x10 陀螺仪量程为:±1000dps     获取到的陀螺仪数据除以32.8          可以转化为带物理单位的数据，单位为：°/s
    //设置为:0x18 陀螺仪量程为:±2000dps     获取到的陀螺仪数据除以16.4          可以转化为带物理单位的数据，单位为：°/s
    //ICM20602_ACCEL_CONFIG寄存器
    //设置为:0x00 加速度计量程为:±2g          获取到的加速度计数据 除以16384      可以转化为带物理单位的数据，单位：g(m/s^2)
    //设置为:0x08 加速度计量程为:±4g          获取到的加速度计数据 除以8192       可以转化为带物理单位的数据，单位：g(m/s^2)
    //设置为:0x10 加速度计量程为:±8g          获取到的加速度计数据 除以4096       可以转化为带物理单位的数据，单位：g(m/s^2)
    //设置为:0x18 加速度计量程为:±16g         获取到的加速度计数据 除以2048       可以转化为带物理单位的数据，单位：g(m/s^2)
}

/*
 *******************************************************************************************
 ** 函数功能: 获取ICM20602加速度计数据
 ** 参    数: 1. x: 加速度计X轴数据
			  2. y: 加速度计Y轴数据
			  3. z: 加速度计Z轴数据
 ** 返 回 值: 无
 ** 作    者: WBN
 ********************************************************************************************
 */ 
void ICM20602GetAcc_SPI(int16_t *x,int16_t *y,int16_t *z)
{
    struct
    {
        uint8_t reg;
        uint8_t dat[6];
    }buf;

    buf.reg = ICM20602_ACCEL_XOUT_H | ICM20602_SPI_R;
    
    ICMReadRegByteS_SPI(&buf.reg, 7);
    *x = (int16_t)(((uint16_t)buf.dat[0]<<8 | buf.dat[1]));
    *y = (int16_t)(((uint16_t)buf.dat[2]<<8 | buf.dat[3]));
    *z = (int16_t)(((uint16_t)buf.dat[4]<<8 | buf.dat[5]));
}

/*
 *******************************************************************************************
 ** 函数功能: 获取ICM20602陀螺仪数据
 ** 参    数: 1. x: 陀螺仪X轴数据
			  2. y: 陀螺仪Y轴数据
			  3. z: 陀螺仪Z轴数据
 ** 返 回 值: 无
 ** 作    者: WBN
 ********************************************************************************************
 */ 
void ICM20602GetGyro_SPI(int16_t *x,int16_t *y,int16_t *z)
{
    struct
    {
        uint8_t reg;
        uint8_t dat[6];
    }buf;

    buf.reg = ICM20602_GYRO_XOUT_H | ICM20602_SPI_R;
    
    ICMReadRegByteS_SPI(&buf.reg, 7);
    *x = (int16_t)(((uint16_t)buf.dat[0]<<8 | buf.dat[1]));
    *y = (int16_t)(((uint16_t)buf.dat[2]<<8 | buf.dat[3]));
    *z = (int16_t)(((uint16_t)buf.dat[4]<<8 | buf.dat[5]));
}
