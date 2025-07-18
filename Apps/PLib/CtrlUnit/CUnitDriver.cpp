#include "stdafx.h"
#include "CUnitDriver.h"

CUnitDriver::CUnitDriver(T_MOTOR_PARAM tMotorParam, CServoMotorDriver *pServoMotorDriver)
	:m_nSoftAxisNo(tMotorParam.nSoftAxisNo),m_tMotorParam(tMotorParam), m_pServoMotorDriver(pServoMotorDriver)
{

}

CUnitDriver::~CUnitDriver()
{

}

T_MOTOR_PARAM CUnitDriver::GetMotorParam()
{
	return m_tMotorParam;
}

int CUnitDriver::SetSevon(bool bEnable)
{
	return m_pServoMotorDriver->SetSevon(m_nSoftAxisNo, bEnable);
}

bool CUnitDriver::GetSevon(bool &bEnable)
{
	return m_pServoMotorDriver->GetSevon(m_nSoftAxisNo, bEnable);
}

bool CUnitDriver::GetSevonRdy(bool &bSevonRdy)
{
	return m_pServoMotorDriver->GetSevonRdy(m_nSoftAxisNo, bSevonRdy);
}

int CUnitDriver::PosMove(long lDistance, int nMode, double dSpeed, double dAcc, double dDec, double dSParam /*= 0.1*/)
{
	return m_pServoMotorDriver->PosMove(m_nSoftAxisNo, lDistance, nMode, dSpeed, dAcc, dDec, dSParam);
}

int CUnitDriver::PosMove(double dDistance, int nMode, double dSpeed, double dAcc, double dDec, double dSParam /*= 0.1*/)
{
	return m_pServoMotorDriver->PosMove(m_nSoftAxisNo, dDistance, nMode, dSpeed, dAcc, dDec, dSParam);
}

int CUnitDriver::ContiMove(int nDir, double dSpeed, double dAcc)
{
	return m_pServoMotorDriver->ContiMove(m_nSoftAxisNo, nDir, dSpeed, dAcc);
}

bool CUnitDriver::CheckAxisRun(int *pnError /*= NULL*/)
{
	return m_pServoMotorDriver->CheckAxisRun(m_nSoftAxisNo, pnError);
}

void CUnitDriver::CheckAxisDone()
{
	if (GetLocalDebugMark()) return;
	while (1)
	{
		Sleep(50);
		DoEvent();
		if (!CheckAxisRun())
		{
			Sleep(30); 
			if (!CheckAxisRun())
			{
				break;
			}
		}
	}
}

bool CUnitDriver::CheckAxisDone(double dDistance, double dError)
{
	CheckAxisDone();
	double dCurrentPosition;
	int nRtn = GetCurrentPosition(dCurrentPosition);
	if (!IsEqual(dDistance, dCurrentPosition, dError))
	{
		WRITE_LOG(GetStr("%lf:%lf---%lf", dDistance, dCurrentPosition, dError));
		return false;
	}
	return true;
}

bool CUnitDriver::CheckAxisDone(long lDistance, long lError)
{
	CheckAxisDone();
	long lCurrentPosition;
	int nRtn = GetCurrentPosition(lCurrentPosition);
	if (!IsEqual(lDistance, lCurrentPosition, lError))
	{
		WRITE_LOG(GetStr("%d:%d---%d", lDistance, lCurrentPosition, lError));
		return false;
	}
	return true;
}

int CUnitDriver::EmgStopAxis()
{
	return m_pServoMotorDriver->EmgStopAxis(m_nSoftAxisNo);
}

int CUnitDriver::SetDecelStopTime(double dDec)
{
	return m_pServoMotorDriver->SetDecelStopTime(m_nSoftAxisNo, dDec);
}

int CUnitDriver::DecelStop()
{
	return m_pServoMotorDriver->DecelStop(m_nSoftAxisNo);
}

int CUnitDriver::GetCurrentPosition(long &lPosition)
{
	return m_pServoMotorDriver->GetCurrentPosition(m_nSoftAxisNo, lPosition);
}

int CUnitDriver::GetCurrentPosition(double &dPosition)
{
	return m_pServoMotorDriver->GetCurrentPosition(m_nSoftAxisNo, dPosition);
}

int CUnitDriver::SetCurrentPosition(long lPosition)
{
	return m_pServoMotorDriver->SetCurrentPosition(m_nSoftAxisNo, lPosition);
}

int CUnitDriver::SetCurrentPosition(double dPosition)
{
	return m_pServoMotorDriver->SetCurrentPosition(m_nSoftAxisNo, dPosition);
}

long CUnitDriver::CoorChange(double dCoor)
{
	double dPulseEquivalent;
	m_pServoMotorDriver->GetDataPulseEquivalent(m_nSoftAxisNo, dPulseEquivalent);
	if (dPulseEquivalent < 1)
	{
		dPulseEquivalent = 1 / dPulseEquivalent;
	}
	return long(dCoor * dPulseEquivalent + 0.5);
}

double CUnitDriver::CoorChange(long lCoor)
{
	double dPulseEquivalent;
	m_pServoMotorDriver->GetDataPulseEquivalent(m_nSoftAxisNo, dPulseEquivalent);
	if (dPulseEquivalent < 1)
	{
		dPulseEquivalent = 1 / dPulseEquivalent;
	}
	return long(lCoor / dPulseEquivalent);
}

double CUnitDriver::GetPulseEquivalent()
{
	double dPulseEquivalent;
	m_pServoMotorDriver->GetDataPulseEquivalent(m_nSoftAxisNo, dPulseEquivalent);
	if (dPulseEquivalent < 1)
	{
		dPulseEquivalent = /*1 / */dPulseEquivalent; // 脉冲当量统一使用 一个脉冲表示的距离
	}
	return dPulseEquivalent;
}

int CUnitDriver::GetTargetPosition(long &lPosition)
{
	return m_pServoMotorDriver->GetTargetPosition(m_nSoftAxisNo, lPosition);
}

int CUnitDriver::GetTargetPosition(double &dPosition)
{
	return m_pServoMotorDriver->GetTargetPosition(m_nSoftAxisNo, dPosition);
}

int CUnitDriver::SetTargetPosition(long lPosition, bool bupdate /*= false*/)
{
	return m_pServoMotorDriver->SetTargetPosition(m_nSoftAxisNo, lPosition, bupdate);
}

int CUnitDriver::SetTargetPosition(double dPosition, bool bupdate /*= false*/)
{
	return m_pServoMotorDriver->SetTargetPosition(m_nSoftAxisNo, dPosition, bupdate);
}

int CUnitDriver::GetSpeed(long &lSpeed)
{
	return m_pServoMotorDriver->GetSpeed(m_nSoftAxisNo, lSpeed);
}

int CUnitDriver::GetSpeed(double &dSpeed)
{
	return m_pServoMotorDriver->GetSpeed(m_nSoftAxisNo, dSpeed);
}

int CUnitDriver::ChangeSpeed(long lSpeed, double dChangeTime /*= 0.1*/)
{
	return m_pServoMotorDriver->ChangeSpeed(m_nSoftAxisNo, lSpeed, dChangeTime);
}

int CUnitDriver::ChangeSpeed(double dSpeed, double dChangeTime /*= 0.1*/)
{
	return m_pServoMotorDriver->ChangeSpeed(m_nSoftAxisNo, dSpeed, dChangeTime);
}

double CUnitDriver::GetMaxSpeed(int nSoftAxisNo)
{
	return m_pServoMotorDriver->GetMaxSpeed(nSoftAxisNo);
}

int CUnitDriver::OpenAbsEncoder()
{
	return m_pServoMotorDriver->OpenAbsEncoder(m_nSoftAxisNo);
}

int CUnitDriver::CloseAbsEncoder()
{
	return m_pServoMotorDriver->CloseAbsEncoder(m_nSoftAxisNo);
}

int CUnitDriver::GetAbsData(double &dAbsData)
{
	return m_pServoMotorDriver->GetAbsData(m_nSoftAxisNo, dAbsData);
}

int CUnitDriver::ResetAbsEncoder()
{
	int nRtn = m_pServoMotorDriver->ClearManyLapData(m_nSoftAxisNo);
	if (nRtn != 0)
	{
		return nRtn;
	}
	return m_pServoMotorDriver->SetInitLapData(m_nSoftAxisNo);
}
