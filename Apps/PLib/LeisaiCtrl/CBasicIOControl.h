#pragma once

#include ".\OpenClass\FileOP\ini\opini.h"
#include "ServoMotorDriver.h"



class CBasicIOControl
{
public:
	CBasicIOControl(CServoMotorDriver *pServoMotorDriver);

	//基础函数(bIONo,nChannelNo:4位数，千位表示控制卡软件编号，百位表示节点号，个位十位表示IO/Channel号)
	//IO功能
	int ReadInbit(int nIONo, WORD &nStatus); //nStatus:低电平ON=0,高电平OFF=1
	int ReadOutbit(int nIONo, WORD &nStatus);
	int WriteOutbit(int nIONo, WORD nStatus);

	int ReadInbit(T_IO_PARAM tIOParam);
	int ReadOutbit(T_IO_PARAM tIOParam);
	int WriteOutbit(T_IO_PARAM tIOParam, WORD on_off);

	//AD/DA功能
	int SetADMode(int nChannelNo, int nMode);//nMode:输入模式，0：电压模式，1：电流模式
	int SetDAMode(int nChannelNo, int nMode);
	int SetDAChannel(int nChannelNo, double dValue);//dValuev:输入输出值，单位：mV/mA
	int GetDAChannel(int nChannelNo, double &dValue);
	int GetADChannel(int nChannelNo, double &dValue);



	CServoMotorDriver *m_pServoMotorDriver = NULL;
};

