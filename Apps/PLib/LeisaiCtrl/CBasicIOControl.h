#pragma once

#include ".\OpenClass\FileOP\ini\opini.h"
#include "ServoMotorDriver.h"



class CBasicIOControl
{
public:
	CBasicIOControl(CServoMotorDriver *pServoMotorDriver);

	//��������(bIONo,nChannelNo:4λ����ǧλ��ʾ���ƿ������ţ���λ��ʾ�ڵ�ţ���λʮλ��ʾIO/Channel��)
	//IO����
	int ReadInbit(int nIONo, WORD &nStatus); //nStatus:�͵�ƽON=0,�ߵ�ƽOFF=1
	int ReadOutbit(int nIONo, WORD &nStatus);
	int WriteOutbit(int nIONo, WORD nStatus);

	int ReadInbit(T_IO_PARAM tIOParam);
	int ReadOutbit(T_IO_PARAM tIOParam);
	int WriteOutbit(T_IO_PARAM tIOParam, WORD on_off);

	//AD/DA����
	int SetADMode(int nChannelNo, int nMode);//nMode:����ģʽ��0����ѹģʽ��1������ģʽ
	int SetDAMode(int nChannelNo, int nMode);
	int SetDAChannel(int nChannelNo, double dValue);//dValuev:�������ֵ����λ��mV/mA
	int GetDAChannel(int nChannelNo, double &dValue);
	int GetADChannel(int nChannelNo, double &dValue);



	CServoMotorDriver *m_pServoMotorDriver = NULL;
};

