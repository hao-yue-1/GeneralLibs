/*
* ���: һ�׿������˲��㷨
* ����: WBN
* ʱ��: 2022/11/7
*/

#include "Kalman1.h"

/*
 *******************************************************************************************
 ** ��������: ��ʼ��һ�׿������ṹ��Ĳ���
 ** ��    ��: 1. *handle: һ�׿������˲����ṹ��
			  2. _Q:	  Q����
			  3. _R:	  R����
 ** �� �� ֵ: ��
 ** ��    ��: WBN
 ********************************************************************************************
 */ 
void Kalman1Config(Kalman1* handle, float _Q, float _R)
{
	handle->x = 0;
	handle->x_ = 0;
	handle->p = 0;
	handle->p_ = 0;
	handle->K = 1;
	handle->A = 1;
	handle->H = 1;
	handle->Q = _Q;
	handle->R = _R;
}

/*
 *******************************************************************************************
 ** ��������: һ�׿������˲���
 ** ��    ��: 1. *handle: һ�׿������˲����ṹ��
			  2. value:   ��Ҫ�����˲���ֵ
 ** �� �� ֵ: �˲����ֵ
 ** ��    ��: WBN
 ********************************************************************************************
 */ 
float Kalman1Filter(Kalman1 *handle,float value)
{
	//���㣺�������
	handle->x_ = handle->A * handle->x_;	
	//���㣺�������Э����
	handle->p_ = handle->A * handle->A * handle->p + handle->Q;
	//���㣺����������
	handle->K = (handle->p_ * handle->H) / (handle->H * handle->p_ * handle->H + handle->R);
	//���㣺�������
	handle->x = handle->x_ + handle->K * (value - handle->H * handle->x_);
	//���£��������Э����
	handle->p = (1 - handle->K * handle->H) * handle->p_;
	
	return handle->x;
}
