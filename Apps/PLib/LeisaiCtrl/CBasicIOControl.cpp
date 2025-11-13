#include "stdafx.h"
#include "CBasicIOControl.h"

CBasicIOControl::CBasicIOControl(CServoMotorDriver *pServoMotorDriver)
	:m_pServoMotorDriver(pServoMotorDriver)
{

}
 
int CBasicIOControl::ReadInbit(int nIONo, WORD &nStatus)
{
	if (m_pServoMotorDriver == NULL)
	{
		return -1;
	}
	int nSoftCtrlCardNo = nIONo / 1000;
	int nNoteNo = (nIONo % 1000) / 100;
	int nBitNo = nIONo % 100;
	return m_pServoMotorDriver->ReadInbit(nSoftCtrlCardNo, nBitNo, nStatus, nNoteNo);
}

int CBasicIOControl::ReadOutbit(int nIONo, WORD &nStatus)
{
	if (m_pServoMotorDriver == NULL)
	{
		return -1;
	}
	int nSoftCtrlCardNo = nIONo / 1000;
	int nNoteNo = (nIONo % 1000) / 100;
	int nBitNo = nIONo % 100;
	return m_pServoMotorDriver->ReadOutbit(nSoftCtrlCardNo, nBitNo, nStatus, nNoteNo);
}

int CBasicIOControl::WriteOutbit(int nIONo, WORD nStatus)
{
	if (m_pServoMotorDriver == NULL)
	{
		return -1;
	}
	int nSoftCtrlCardNo = nIONo / 1000;
	int nNoteNo = (nIONo % 1000) / 100;
	int nBitNo = nIONo % 100;
	return m_pServoMotorDriver->WriteOutbit(nSoftCtrlCardNo, nBitNo, nStatus, nNoteNo);
}

int CBasicIOControl::ReadInbit(T_IO_PARAM tIOParam)
{
	WORD wCardNo = tIOParam.nCardNo;
	WORD wNoteID = tIOParam.nCardNodeNo;
	WORD wBitNo = tIOParam.nBit;

	if (0 == wNoteID)
	{
		return dmc_read_inbit(wCardNo, wBitNo);
	}
	else
	{
		WORD wReturn = OFF;
		nmc_read_inbit(wCardNo, wNoteID, wBitNo, &wReturn); 
		return wReturn;
	}
}

int CBasicIOControl::ReadOutbit(T_IO_PARAM tIOParam)
{
	WORD wCardNo = tIOParam.nCardNo;
	WORD wNoteID = tIOParam.nCardNodeNo;
	WORD wBitNo = tIOParam.nBit;

	if (0 == wNoteID)
	{
		return dmc_read_outbit(wCardNo, wBitNo);
	}
	else
	{
		WORD wReturn = OFF;
		nmc_read_outbit(wCardNo, wNoteID, wBitNo, &wReturn);
		return wReturn;
	}
}

int CBasicIOControl::WriteOutbit(T_IO_PARAM tIOParam, WORD on_off)
{
	WORD wCardNo = tIOParam.nCardNo;
	WORD wNoteID = tIOParam.nCardNodeNo;
	WORD wBitNo = tIOParam.nBit;

	if (0 == wNoteID)
	{
		return dmc_write_outbit(wCardNo, wBitNo, on_off);
	}
	else
	{
		return nmc_write_outbit(wCardNo, wNoteID, wBitNo, on_off);
	}
}

int CBasicIOControl::SetADMode(int nChannelNo, int nMode)
{
	if (m_pServoMotorDriver == NULL)
	{
		return -1;
	}
	int nSoftCtrlCardNo = nChannelNo / 1000;
	int nNoteNo = (nChannelNo % 1000) / 100;
	int nBitNo = nChannelNo % 100;
	return m_pServoMotorDriver->SetADMode(nSoftCtrlCardNo, nNoteNo, nBitNo, nMode);
}

int CBasicIOControl::SetDAMode(int nChannelNo, int nMode)
{
	if (m_pServoMotorDriver == NULL)
	{
		return -1;
	}
	int nSoftCtrlCardNo = nChannelNo / 1000;
	int nNoteNo = (nChannelNo % 1000) / 100;
	int nBitNo = nChannelNo % 100;
	return m_pServoMotorDriver->SetDAMode(nSoftCtrlCardNo, nNoteNo, nBitNo, nMode);
}

int CBasicIOControl::SetDAChannel(int nChannelNo, double dValue)
{
	if (m_pServoMotorDriver == NULL)
	{
		return -1;
	}
	int nSoftCtrlCardNo = nChannelNo / 1000;
	int nNoteNo = (nChannelNo % 1000) / 100;
	int nBitNo = nChannelNo % 100;
	return m_pServoMotorDriver->SetDAChannel(nSoftCtrlCardNo, nNoteNo, nBitNo, dValue);
}

int CBasicIOControl::GetDAChannel(int nChannelNo, double &dValue)
{
	if (m_pServoMotorDriver == NULL)
	{
		return -1;
	}
	int nSoftCtrlCardNo = nChannelNo / 1000;
	int nNoteNo = (nChannelNo % 1000) / 100;
	int nBitNo = nChannelNo % 100;
	return m_pServoMotorDriver->GetDAChannel(nSoftCtrlCardNo, nNoteNo, nBitNo, dValue);
}

int CBasicIOControl::GetADChannel(int nChannelNo, double &dValue)
{
	if (m_pServoMotorDriver == NULL)
	{
		return -1;
	}
	int nSoftCtrlCardNo = nChannelNo / 1000;
	int nNoteNo = (nChannelNo % 1000) / 100;
	int nBitNo = nChannelNo % 100;
	return m_pServoMotorDriver->GetADChannel(nSoftCtrlCardNo, nNoteNo, nBitNo, dValue);
}

