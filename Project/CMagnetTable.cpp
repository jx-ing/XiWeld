#include "stdafx.h"
#include ".\Project\CMagnetTable.h"


CMagnetTable::CMagnetTable()
{
}


CMagnetTable::~CMagnetTable()
{
}

uint8_t1 Up4Value[31] = { 0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09
,0x0a,0x0b,0x0c,0x0d,0x0e,0x0f,0x10,0x11,0x12,0x13
,0x14,0x15,0x16,0x17,0x18,0x19,0x1a,0x1b,0x1c,0x1d };
uint8_t1 Up5Value[31] = { 0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01
,0x01,0x01,0x01,0x01,0x01,0x02,0x02,0x02,0x02,0x02
,0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x02 };
uint8_t1 Down4Value[31] = { 0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09
,0x0a,0x0b,0x0c,0x0d,0x0e,0x0f,0x10,0x11,0x12,0x13
,0x14,0x15,0x16,0x17,0x18,0x19,0x1a,0x1b,0x1c,0x1d };
uint8_t1 Down5Value[31] = { 0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01
,0x01,0x01,0x01,0x01,0x01,0x02,0x02,0x02,0x02,0x02
,0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x02 };
uint8_t1 Up4ValueSupport[15] = { 0x22,0x23,0x24,0x25,0x26,0x27,0x28,0x2d,0x2e,0x2f,0x30,0x31,0x32,0x33 };
uint8_t1 Up5ValueSupport[15] = { 0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x06,0x06,0x06,0x06,0x06,0x06,0x06 };
/*
, 0x1e, 0x1f, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27
, 0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x2f*/
unsigned int CMagnetTable::Crc_Count(unsigned char pbuf[], unsigned char num)
{
	int i, j; unsigned int wcrc = 0xffff;
	for (i = 0; i < num; i++)
	{
		wcrc ^= (unsigned int)(pbuf[i]);
		for (j = 0; j < 8; j++)
		{
			if (wcrc & 0x0001)
			{
				wcrc >>= 1; wcrc ^= 0xa001;
			}
			else
				wcrc >>= 1;
		}
	}
	return wcrc;
}

void CMagnetTable::Crc_Count(std::vector<uint8_t1> &vcbuf)
{
	int num = vcbuf.size();
	int i, j; unsigned int wcrc = 0xffff;
	for (i = 0; i < num; i++)
	{
		wcrc ^= (unsigned int)(vcbuf[i]);
		for (j = 0; j < 8; j++)
		{
			if (wcrc & 0x0001)
			{
				wcrc >>= 1; wcrc ^= 0xa001;
			}
			else
				wcrc >>= 1;
		}
	}
	uint8_t1 ucLow = wcrc & 0xff;
	uint8_t1 ucHigh = (wcrc >> 8) & 0xff;
	vcbuf.push_back(ucLow);
	vcbuf.push_back(ucHigh);
}

bool CMagnetTable::CheckCrc_Count(std::vector<uint8_t1> vcbuf)
{
	int num = vcbuf.size() - 2;
	if (num < 1)
	{
		return false;
	}
	int i, j; unsigned int wcrc = 0xffff;
	for (i = 0; i < num; i++)
	{
		wcrc ^= (unsigned int)(vcbuf[i]);
		for (j = 0; j < 8; j++)
		{
			if (wcrc & 0x0001)
			{
				wcrc >>= 1; wcrc ^= 0xa001;
			}
			else
				wcrc >>= 1;
		}
	}
	uint8_t1 ucLow = wcrc & 0xff;
	uint8_t1 ucHigh = (wcrc >> 8) & 0xff;
	if (ucHigh == vcbuf.back() && ucLow == vcbuf[vcbuf.size() - 2])
	{
		return true;
	}
	return false;
}

std::vector<uint8_t1> CMagnetTable::SendData(uint8_t1 Adr, uint8_t1 Fun, uint8_t1 StartAdrH, uint8_t1 StartAdrL, uint8_t1 DataH, uint8_t1 DataL)
{
	vector<uint8_t1> vTemp;
	vTemp.push_back(Adr);
	vTemp.push_back(Fun);
	vTemp.push_back(StartAdrH);
	vTemp.push_back(StartAdrL);
	vTemp.push_back(DataH);
	vTemp.push_back(DataL);
	Crc_Count(vTemp);
	//XiMessageBoxOk(NULL,"%x  %x", vTemp[vTemp.size() - 2], vTemp.back());
	WriteToPort(vTemp);
	return vTemp;
}

bool CMagnetTable::RcvData(std::vector<uint8_t1> vcSendData, std::vector<uint8_t1> &vcData)
{
	vector<uint8_t1> vHeader;
	ReadToPort(vHeader, 3);
	if (vHeader[0] != vcSendData[0] || vHeader[1] != vcSendData[1])
	{
		return false;
	}
	int nDataSize = 0;
	if (vHeader[1] == 0x03)
	{
		nDataSize = vHeader[2] + 2;
	}
	else
	{
		nDataSize = vcSendData.size() - 3;
	}
	vector<uint8_t1> vData;
	ReadToPort(vData, nDataSize);
	vcData.clear();
	vcData = vHeader;
	vcData.insert(vcData.end(), vData.begin(), vData.end());

	if (CheckCrc_Count(vcData))
	{
		return true;
	}
	return false;
}

long CMagnetTable::GetDecimal(std::vector<uint8_t1> vData)
{
	long lSum = 0;
	reverse(vData.begin(), vData.end());
	for (int n = vData.size() - 1; n >= 0; n--)
	{
		long lTemp = GetMultipleSquare(16, n * 2);
		long lLow = vData[n];
		lLow = lLow & 0x0f;
		lLow *= lTemp;
		long lHigh = vData[n];
		lHigh = (lHigh >> 4) & 0x0f;
		lHigh *= lTemp * 16;
		lSum += lLow + lHigh;
	}
	return lSum;
}

long CMagnetTable::GetMultipleSquare(long lData, int m)
{
	if (m < 1)
	{
		return 1;
	}
	long lSum = lData;
	for (int n = 1; n < m; n++)
	{
		lSum = lSum * lData;
	}
	return lSum;
}


bool CMagnetTable::SetMagnetTableUP(int iMagnetNo)
{

	if (Up5Value[iMagnetNo] < 20 && 0x01 == 0)
	{
		return false;
	}
	std::vector<uint8_t1> vSendData = SendData(0x01, 0x06, 0x00, Up4Value[iMagnetNo], Up5Value[iMagnetNo], 0x10);
	std::vector<uint8_t1> vRcvData;
	if (!RcvData(vSendData, vRcvData))
	{
		return false;
	}
	//if (vSendData != vRcvData)
	//{
	//	return false;
	//}
	return true;
}
bool CMagnetTable::SetMagnetTableDown(int iMagnetNo)
{

	if (Down5Value[iMagnetNo] < 20 && 0x01 == 0)
	{
		return false;
	}
	std::vector<uint8_t1> vSendData = SendData(0x01, 0x06, 0x00, Down4Value[iMagnetNo], Down5Value[iMagnetNo], 0x01);
	std::vector<uint8_t1> vRcvData;
	if (!RcvData(vSendData, vRcvData))
	{
		return false;
	}
	//if (vSendData != vRcvData)
	//{
	//	return false;
	//}
	return true;
}
bool CMagnetTable::SetSupportUP(int iSupportNo)
{

	if (Up5ValueSupport[iSupportNo] < 20 && 0x01 == 0)
	{
		return false;
	}
	std::vector<uint8_t1> vSendData = SendData(0x01, 0x06, 0x00, Up4ValueSupport[iSupportNo], Up5ValueSupport[iSupportNo], 0x10);
	std::vector<uint8_t1> vRcvData;
	if (!RcvData(vSendData, vRcvData))
	{
		return false;
	}
	//if (vSendData != vRcvData)
	//{
	//	return false;
	//}
	return true;
}

bool CMagnetTable::SetSupportDown(int iSupportNo)
{

	if (Up5ValueSupport[iSupportNo] < 20 && 0x01 == 0)
	{
		return false;
	}
	std::vector<uint8_t1> vSendData = SendData(0x01, 0x06, 0x00, Up4ValueSupport[iSupportNo], Up5ValueSupport[iSupportNo], 0x01);
	std::vector<uint8_t1> vRcvData;
	if (!RcvData(vSendData, vRcvData))
	{
		return false;
	}
	//if (vSendData != vRcvData)
	//{
	//	return false;
	//}
	return true;
}

bool CMagnetTable::CheckMagnetNo(int iSupportNo)
{

	if (Up5ValueSupport[iSupportNo] < 20 && 0x01 == 0)
	{
		return false;
	}
	std::vector<uint8_t1> vSendData = SendData(0x01, 0x04, 0x00, 0x44, 0x00, 0x01);
	std::vector<uint8_t1> vRcvData;
	if (!RcvData(vSendData, vRcvData))
	{
		return false;
	}
	if (vSendData != vRcvData)
	{
		return false;
	}
	return true;
}