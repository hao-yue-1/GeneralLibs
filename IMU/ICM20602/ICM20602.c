/*
* ���: ICM20602 IMU ��̬������
        ��оƬ֧��ֱ�Ӷ����������� ����������������ٶȼ�
		֧��IIC��SPIͨ��
* ����: WBN
* ʱ��: 2022/11/8
*/

#include "ICM20602.h"
#include "SEEKFREE_IIC.h"
#include "stm32f1xx_hal.h"	//HAL_Delay
#include "main.h"			//CS���ź�
#include "spi.h"			//SPIͨ��

/* �����������IICͨ�� */

/*
 *******************************************************************************************
 ** ��������: ICM20602 ���IICд�Ĵ���
 ** ��    ��: 1. dev_add: �豸��ַ ����λ��ַ
			  2. reg: �Ĵ�����ַ
			  3. dat: д�������
 ** �� �� ֵ: ��
 ** ��    ��: WBN
 ********************************************************************************************
 */ 
void ICMWriteRegByte_IIC(uint8_t dev_add, uint8_t reg, uint8_t dat)
{	
	simiic_start();
    send_ch( (dev_add<<1) | 0x00);   //����������ַ��дλ
	send_ch( reg );   				 //���ʹӻ��Ĵ�����ַ
	send_ch( dat );   				 //������Ҫд�������
	simiic_stop();
}

/*
 *******************************************************************************************
 ** ��������: ICM20602 ���IIC���Ĵ��� ���ֽ�
 ** ��    ��: 1. dev_add: �豸��ַ ����λ��ַ
			  2. reg:     �Ĵ�����ַ
 ** �� �� ֵ: �Ĵ���������
 ** ��    ��: WBN
 ********************************************************************************************
 */
uint8_t ICMReadRegByte_IIC(uint8_t dev_add, uint8_t reg)
{	
	uint8_t dat;
	simiic_start();
    send_ch( (dev_add<<1) | 0x00);  //����������ַ��дλ
	send_ch( reg );   				//���ʹӻ��Ĵ�����ַ
	
	simiic_start();
	send_ch( (dev_add<<1) | 0x01);  //����������ַ�Ӷ�λ
	dat = read_ch(0);   				//��ȡ����
	simiic_stop();
	
	return dat;
}

/*
 *******************************************************************************************
 ** ��������: ICM20602 ���IIC���Ĵ��� ���ֽ�
 ** ��    ��: 1. dev_add:  �豸��ַ ����λ��ַ
			  2. reg:      �Ĵ�����ַ
			  3. *dat_add: ��ȡ���ݱ���ĵ�ַָ��
			  4. num:	   ��ȡ�ֽ�����
 ** �� �� ֵ: �Ĵ���������
 ** ��    ��: WBN
 ********************************************************************************************
 */
void ICMReadRegByteS_IIC(uint8_t dev_add, uint8_t reg, uint8_t *dat_add, uint8_t num)
{
	simiic_start();
    send_ch( (dev_add<<1) | 0x00);  //����������ַ��дλ
	send_ch( reg );   				//���ʹӻ��Ĵ�����ַ
	
	simiic_start();
	send_ch( (dev_add<<1) | 0x01);  //����������ַ�Ӷ�λ
    while(--num)
    {
        *dat_add = read_ch(1); //��ȡ����
        dat_add++;
    }
    *dat_add = read_ch(0); //��ȡ����
	simiic_stop();
}

/*
 *******************************************************************************************
 ** ��������: ICM20602�Լ캯��
 ** ��    ��: ��
 ** �� �� ֵ: ��
 ** ��    ��: WBN
 ********************************************************************************************
 */ 
void ICM20602Self_IIC(void)
{
    uint8_t dat=0;
    while(dat != 0x12)   //��ȡICM20602 ID
    {
        dat = ICMReadRegByte_IIC(ICM20602_DEV_ADDR,ICM20602_WHO_AM_I);
        HAL_Delay(10);
    }
}

/*
 *******************************************************************************************
 ** ��������: ���IIC����ʼ��ICM20602
 ** ��    ��: ��
 ** �� �� ֵ: ��
 ** ��    ��: WBN
 ********************************************************************************************
 */ 
void ICM20602Init_IIC(void)
{
    HAL_Delay(10);  		//�ϵ���ʱ
    ICM20602Self_IIC();		//���
    //��λ
    ICMWriteRegByte_IIC(ICM20602_DEV_ADDR,ICM20602_PWR_MGMT_1,0x80);	//��λ�豸
    HAL_Delay(2);	//��ʱ
    while(0x80 & ICMReadRegByte_IIC(ICM20602_DEV_ADDR,ICM20602_PWR_MGMT_1));//�ȴ���λ���
    //���ò���
    ICMWriteRegByte_IIC(ICM20602_DEV_ADDR,ICM20602_PWR_MGMT_1,0x01);               //ʱ������
    ICMWriteRegByte_IIC(ICM20602_DEV_ADDR,ICM20602_PWR_MGMT_2,0x00);               //���������Ǻͼ��ٶȼ�
    ICMWriteRegByte_IIC(ICM20602_DEV_ADDR,ICM20602_CONFIG,0x01);                   //176HZ 1KHZ
    ICMWriteRegByte_IIC(ICM20602_DEV_ADDR,ICM20602_SMPLRT_DIV,0x07);               //�������� SAMPLE_RATE = INTERNAL_SAMPLE_RATE / (1 + SMPLRT_DIV)
    ICMWriteRegByte_IIC(ICM20602_DEV_ADDR,ICM20602_GYRO_CONFIG,0x18);              //��2000 dps
    ICMWriteRegByte_IIC(ICM20602_DEV_ADDR,ICM20602_ACCEL_CONFIG,0x10);             //��8g
    ICMWriteRegByte_IIC(ICM20602_DEV_ADDR,ICM20602_ACCEL_CONFIG_2,0x03);           //Average 4 samples   44.8HZ   //0x23 Average 16 samples
	//ICM20602_GYRO_CONFIG�Ĵ���
    //����Ϊ:0x00 ����������Ϊ:��250 dps     ��ȡ�������������ݳ���131           ����ת��Ϊ������λ�����ݣ� ��λΪ����/s
    //����Ϊ:0x08 ����������Ϊ:��500 dps     ��ȡ�������������ݳ���65.5          ����ת��Ϊ������λ�����ݣ���λΪ����/s
    //����Ϊ:0x10 ����������Ϊ:��1000dps     ��ȡ�������������ݳ���32.8          ����ת��Ϊ������λ�����ݣ���λΪ����/s
    //����Ϊ:0x18 ����������Ϊ:��2000dps     ��ȡ�������������ݳ���16.4          ����ת��Ϊ������λ�����ݣ���λΪ����/s
    //ICM20602_ACCEL_CONFIG�Ĵ���
    //����Ϊ:0x00 ���ٶȼ�����Ϊ:��2g          ��ȡ���ļ��ٶȼ����� ����16384      ����ת��Ϊ������λ�����ݣ���λ��g(m/s^2)
    //����Ϊ:0x08 ���ٶȼ�����Ϊ:��4g          ��ȡ���ļ��ٶȼ����� ����8192       ����ת��Ϊ������λ�����ݣ���λ��g(m/s^2)
    //����Ϊ:0x10 ���ٶȼ�����Ϊ:��8g          ��ȡ���ļ��ٶȼ����� ����4096       ����ת��Ϊ������λ�����ݣ���λ��g(m/s^2)
    //����Ϊ:0x18 ���ٶȼ�����Ϊ:��16g         ��ȡ���ļ��ٶȼ����� ����2048       ����ת��Ϊ������λ�����ݣ���λ��g(m/s^2)
}

/*
 *******************************************************************************************
 ** ��������: ��ȡICM20602���ٶȼ�����
 ** ��    ��: 1. x: ���ٶȼ�X������
			  2. y: ���ٶȼ�Z������
			  3. z: ���ٶȼ�Y������
 ** �� �� ֵ: ��
 ** ��    ��: WBN
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
 ** ��������: ��ȡICM20602����������
 ** ��    ��: 1. x: ������X������
			  2. y: ������Z������
			  3. z: ������Y������
 ** �� �� ֵ: ��
 ** ��    ��: WBN
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
 ** ��������: ��ʼ��ICM20602��������Ư
 ** ��    ��: 1. *handle: ��Ư����
			  2. dt:      ����ʱ���� ��λms
              3. mode:    ͨ�ŷ�ʽ 0.���IIC 1.SPI
 ** �� �� ֵ: ��
 ** ��    ��: WBN
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

/* ��������SPIͨ�� */

/*
 *******************************************************************************************
 ** ��������: ICM20602 SPIд�Ĵ��� ���ֽ�
 ** ��    ��: 1. cmd: �Ĵ�����ַ
			  2. val: д�������
 ** �� �� ֵ: ��
 ** ��    ��: WBN
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
 ** ��������: ICM20602 SPI���Ĵ��� ���ֽ�
 ** ��    ��: 1. cmd:  �Ĵ�����ַ
			  2. *val: �������ݵĵ�ַ
 ** �� �� ֵ: ��
 ** ��    ��: WBN
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
 ** ��������: ICM20602 SPI���Ĵ��� ���ֽ�
 ** ��    ��: 1. *val: �������ݵĵ�ַ
			  2. num: ��ȡ��������
 ** �� �� ֵ: ��
 ** ��    ��: WBN
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
 ** ��������: ICM20602�Լ캯��
 ** ��    ��: ��
 ** �� �� ֵ: ��
 ** ��    ��: WBN
 ********************************************************************************************
 */ 
void ICM20602Self_SPI(void)
{
    uint8_t dat=0;
    while(0x12 != dat)   //��ȡICM20602 ID
    {
        ICMReadRegByte_SPI(ICM20602_WHO_AM_I,&dat);
        HAL_Delay(10);
    }
}
     
/*
 *******************************************************************************************
 ** ��������: ��ʼ��ICM20602
 ** ��    ��: ��
 ** �� �� ֵ: ��
 ** ��    ��: WBN
 ********************************************************************************************
 */ 
void ICM20602Init_SPI(void)
{
    uint8_t val = 0x0;

    HAL_Delay(10);  	//�ϵ���ʱ
    ICM20602Self_SPI();	//���
    ICMWriteRegByte_SPI(ICM20602_PWR_MGMT_1,0x80);//��λ�豸
    HAL_Delay(2);
    do	//�ȴ���λ�ɹ�
    {
        ICMReadRegByte_SPI(ICM20602_PWR_MGMT_1,&val);
    }while(val != 0x41);
    
    ICMWriteRegByte_SPI(ICM20602_PWR_MGMT_1,     0x01);            //ʱ������
    ICMWriteRegByte_SPI(ICM20602_PWR_MGMT_2,     0x00);            //���������Ǻͼ��ٶȼ�
    ICMWriteRegByte_SPI(ICM20602_CONFIG,         0x01);            //176HZ 1KHZ
    ICMWriteRegByte_SPI(ICM20602_SMPLRT_DIV,     0x07);            //�������� SAMPLE_RATE = INTERNAL_SAMPLE_RATE / (1 + SMPLRT_DIV)
    ICMWriteRegByte_SPI(ICM20602_GYRO_CONFIG,    0x18);            //��2000 dps
    ICMWriteRegByte_SPI(ICM20602_ACCEL_CONFIG,   0x10);            //��8g
    ICMWriteRegByte_SPI(ICM20602_ACCEL_CONFIG_2, 0x03);            //Average 4 samples   44.8HZ   //0x23 Average 16 samples
	//ICM20602_GYRO_CONFIG�Ĵ���
    //����Ϊ:0x00 ����������Ϊ:��250 dps     ��ȡ�������������ݳ���131           ����ת��Ϊ������λ�����ݣ� ��λΪ����/s
    //����Ϊ:0x08 ����������Ϊ:��500 dps     ��ȡ�������������ݳ���65.5          ����ת��Ϊ������λ�����ݣ���λΪ����/s
    //����Ϊ:0x10 ����������Ϊ:��1000dps     ��ȡ�������������ݳ���32.8          ����ת��Ϊ������λ�����ݣ���λΪ����/s
    //����Ϊ:0x18 ����������Ϊ:��2000dps     ��ȡ�������������ݳ���16.4          ����ת��Ϊ������λ�����ݣ���λΪ����/s
    //ICM20602_ACCEL_CONFIG�Ĵ���
    //����Ϊ:0x00 ���ٶȼ�����Ϊ:��2g          ��ȡ���ļ��ٶȼ����� ����16384      ����ת��Ϊ������λ�����ݣ���λ��g(m/s^2)
    //����Ϊ:0x08 ���ٶȼ�����Ϊ:��4g          ��ȡ���ļ��ٶȼ����� ����8192       ����ת��Ϊ������λ�����ݣ���λ��g(m/s^2)
    //����Ϊ:0x10 ���ٶȼ�����Ϊ:��8g          ��ȡ���ļ��ٶȼ����� ����4096       ����ת��Ϊ������λ�����ݣ���λ��g(m/s^2)
    //����Ϊ:0x18 ���ٶȼ�����Ϊ:��16g         ��ȡ���ļ��ٶȼ����� ����2048       ����ת��Ϊ������λ�����ݣ���λ��g(m/s^2)
}

/*
 *******************************************************************************************
 ** ��������: ��ȡICM20602���ٶȼ�����
 ** ��    ��: 1. x: ���ٶȼ�X������
			  2. y: ���ٶȼ�Y������
			  3. z: ���ٶȼ�Z������
 ** �� �� ֵ: ��
 ** ��    ��: WBN
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
 ** ��������: ��ȡICM20602����������
 ** ��    ��: 1. x: ������X������
			  2. y: ������Y������
			  3. z: ������Z������
 ** �� �� ֵ: ��
 ** ��    ��: WBN
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
