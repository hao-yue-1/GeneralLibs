/*
* ���: һ�׿������˲��㷨
* ����: WBN
* ʱ��: 2022/11/7
*/

#ifndef KALMAN_H
#define KALMAN_H

typedef struct
{
	float x_;	//�������
	float x;	//�������
	float p_;	//�������Э����
	float p;	//�������Э����
	float K;	//����������
	float A;	//����ϵ��=1
	float Q;	//������������
	float R;	//������������
	float H;	//����ϵ��=1

}Kalman1;

void Kalman1Config(Kalman1* handle, float _Q, float _R);
float Kalman1Filter(Kalman1 *handle,float value);

#endif
