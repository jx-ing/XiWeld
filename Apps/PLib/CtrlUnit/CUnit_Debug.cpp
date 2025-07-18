#include "stdafx.h"
#include ".\Apps\PLib\CtrlUnit\CUnit_Debug.h"

#if ENABLE_UNIT_DEBUG

CUnit_Debug::CUnit_Debug(T_CONTRAL_UNIT tCtrlUnitInfo, CServoMotorDriver* pServoMotorDriver /*= NULL*/)
	:CUnit(tCtrlUnitInfo, pServoMotorDriver)
{
	m_bStartDebug = GetLocalDebugMark() == TRUE;
	CreateDatabaseTable();
}

CUnit_Debug::~CUnit_Debug()
{
}
bool CUnit_Debug::GetEmgStopState()
{
	return m_bEmgStop;
}
void CUnit_Debug::EmgStop()
{
	m_bEmgStop = true;
}
void CUnit_Debug::Rest()
{
	m_bEmgStop = false;
}
BOOL CUnit_Debug::GetRobotTool(int nToolNo, T_ROBOT_COORS& tTool)
{
	if (!m_bStartDebug)
	{
		return CContralUnit::GetRobotTool(nToolNo, tTool);
	}
	return GetRobotTool_Database(nToolNo, tTool);
}

T_ROBOT_COORS CUnit_Debug::GetRobotTool(int nToolNo)
{
	if (!m_bStartDebug)
	{
		return CContralUnit::GetRobotTool(nToolNo);
	}
	return GetRobotTool_Database(nToolNo);
}

T_ROBOT_COORS CUnit_Debug::GetRobotTool_Database(int nToolNo)
{
	std::map< int, std::vector<double>> mvdRobotTool;
	if (FALSE == m_cDatabaseCtrl.ReadData_RobotTool(GetUnitName(), mvdRobotTool))
	{
		return T_ROBOT_COORS();
	}
	if (mvdRobotTool.find(nToolNo) == mvdRobotTool.end())
	{
		double adTool[6] = {0};
		m_cDatabaseCtrl.InsertData_RobotTool(GetUnitName(), nToolNo, adTool);
		return T_ROBOT_COORS();
	}
	return T_ROBOT_COORS(mvdRobotTool[nToolNo][0], mvdRobotTool[nToolNo][1], mvdRobotTool[nToolNo][2],
		mvdRobotTool[nToolNo][3], mvdRobotTool[nToolNo][4], mvdRobotTool[nToolNo][5], 0, 0, 0);
}

BOOL CUnit_Debug::GetRobotTool_Database(int nToolNo, T_ROBOT_COORS& tTool)
{
	std::map< int, std::vector<double>> mvdRobotTool;
	if (FALSE == m_cDatabaseCtrl.ReadData_RobotTool(GetUnitName(), mvdRobotTool))
	{
		return FALSE;
	}
	if (mvdRobotTool.find(nToolNo) == mvdRobotTool.end())
	{
		double adTool[6] = { tTool.dX, tTool.dY, tTool.dZ, tTool.dRX, tTool.dRY, tTool.dRZ };
		m_cDatabaseCtrl.InsertData_RobotTool(GetUnitName(), nToolNo, adTool);
		return TRUE;
	}
	tTool = T_ROBOT_COORS(mvdRobotTool[nToolNo][0], mvdRobotTool[nToolNo][1], mvdRobotTool[nToolNo][2],
		mvdRobotTool[nToolNo][3], mvdRobotTool[nToolNo][4], mvdRobotTool[nToolNo][5], 0, 0, 0);
	return TRUE;
}

BOOL CUnit_Debug::RobotCheckRunning_Database()
{
	T_DEVICE_STATE tRobotState;
	tRobotState.strUnitName = GetUnitName();
	tRobotState.strDeviceType = "robot";
	tRobotState.strDeviceNo = "0";
	m_cDatabaseCtrl.ReadData_DeviceState(tRobotState);
	return tRobotState.bMoveState;
}

BOOL CUnit_Debug::RobotCheckRunning()
{
	if (!m_bStartDebug)
	{
		return CContralUnit::RobotCheckRunning();
	}
	return RobotCheckRunning_Database();
}

BOOL CUnit_Debug::RobotCheckDone(int nDelayTime)
{
	if (!m_bStartDebug)
	{
		return CContralUnit::RobotCheckDone(nDelayTime);
	}
	return RobotCheckDone_Database(nDelayTime);
}

BOOL CUnit_Debug::RobotCheckDone(T_ROBOT_COORS tAimCoor, T_ROBOT_COORS tCompareLimit /* = T_ROBOT_COORS(1, 1, 1, 1, 1, 1, 1, 1, 1)*/, int nTool /*= -1*/)
{
	CHECK_BOOL_RTN_UNIT(RobotCheckDone(), GetStr("运动至:%s 失败", GetStr(tAimCoor)));
	return GetRobotCtrl()->CompareCoord(tAimCoor, GetRobotPos(nTool), tCompareLimit);
}

BOOL CUnit_Debug::RobotCheckDone(T_ANGLE_PULSE tAimPulse, T_ANGLE_PULSE tCompareLimit /*= T_ANGLE_PULSE(10, 10, 10, 10, 10, 10, 10, 10, 10)*/)
{
	CHECK_BOOL_RTN_UNIT(RobotCheckDone(), GetStr("运动至:%s 失败", GetStr(tAimPulse)));
	return GetRobotCtrl()->ComparePulse(tAimPulse, GetRobotPulse(), tCompareLimit);
}

BOOL CUnit_Debug::RobotCheckDone_Database(int nDelayTime)
{
	long long nStartTime = XI_clock();
	while (RobotCheckRunning_Database() == FALSE)
	{
		if ((XI_clock() - nStartTime) > nDelayTime)
		{
			break;
		}
		Sleep(10);
	}
	while (1)
	{
		if (RobotCheckRunning_Database() == FALSE)
		{
			Sleep(200);
			if (RobotCheckRunning_Database() == FALSE)
			{
				break;
			}
		}
		Sleep(10);
	}
	return TRUE;
}

T_ROBOT_COORS CUnit_Debug::GetRobotPos(int nToolNo /*= -1*/)
{
	if (!m_bStartDebug)
	{
		T_ROBOT_COORS tCurrentPos = CContralUnit::GetRobotPos(nToolNo);
		return tCurrentPos;
	}

	if (nToolNo == -1)
	{
		T_ROBOT_COORS tCurrentPos = GetRobotCoor();
		GetCurrentPosition(0, tCurrentPos.dBX);
		return tCurrentPos;
	}
	else if (nToolNo >= 0 && nToolNo <= 10)
	{
		T_ROBOT_COORS tTool, tCurrentPos;
		if (FALSE == GetRobotTool(nToolNo, tTool))
		{
			return T_ROBOT_COORS();
		}
		GetRobotCtrl()->RobotKinematics(GetRobotPulse(), tTool, tCurrentPos); 
		GetCurrentPosition(0, tCurrentPos.dBX);
		return tCurrentPos;
	}
	else
	{
		return T_ROBOT_COORS();
	}
	
}

T_ANGLE_PULSE CUnit_Debug::GetCurBaseAxisPulse(T_ANGLE_PULSE tPulse)
{
	T_ANGLE_PULSE temp = GetRobotPulse();
	tPulse.lBXPulse = temp.lBXPulse;
	tPulse.lBYPulse = temp.lBYPulse;
	tPulse.lBZPulse = temp.lBZPulse;
	return tPulse;
}

T_ROBOT_COORS CUnit_Debug::GetCurBaseAxisPos(T_ROBOT_COORS tCoor)
{
	T_ROBOT_COORS temp = GetRobotPos();
	tCoor.dBX = temp.dBX;
	tCoor.dBY = temp.dBY;
	tCoor.dBZ = temp.dBZ;
	return tCoor;
}

bool CUnit_Debug::SetMoveValue(std::vector<T_ROBOT_MOVE_INFO> vtMoveInfo)
{
	if (!m_bStartDebug)
	{
		return CUnit::SetMoveValue(vtMoveInfo);
	}
	return SetMoveValue_Database(vtMoveInfo);
}

bool CUnit_Debug::SetMoveValue(CRobotMove cMoveInfo)
{
	if (!m_bStartDebug)
	{
		return CUnit::SetMoveValue(cMoveInfo);
	}
	return false;
}

int CUnit_Debug::GetMoveStep()
{
	if (!m_bStartDebug)
	{
		return CUnit::GetMoveStep();
	}
	T_DEVICE_STATE tRobotState;	
	tRobotState.strUnitName = GetUnitName();
	tRobotState.strDeviceType = "robot";
	tRobotState.strDeviceNo = "0";
	m_cDatabaseCtrl.ReadData_DeviceState(tRobotState);
	return tRobotState.nMoveStep;
}

int CUnit_Debug::CleanMoveStep()
{
	if (!m_bStartDebug)
	{
		return CUnit::CleanMoveStep();
	}
	m_cDatabaseCtrl.UpdateData_DeviceState_MoveStep(GetUnitName(), "robot", "0", 0);
	return 0;
}

bool CUnit_Debug::SetMoveValue_Database(std::vector<T_ROBOT_MOVE_INFO> vtMoveInfo)
{
	if (vtMoveInfo.size() > 10)
	{
		return false;
	}
	std::vector<T_MOVE_VALUE> vtMoveValue;
	m_cDatabaseCtrl.ReadData_MoveValue(GetUnitName(), "robot", "0", vtMoveValue);
	m_cDatabaseCtrl.DeleteData_MoveValue(GetUnitName(), "robot", "0", vtMoveValue);
	T_MOVE_VALUE tMoveValue;
	tMoveValue.strUnitName = GetUnitName();
	tMoveValue.strMoveDevice = "robot";
	tMoveValue.strDeviceNo = "0";
	for (size_t i = 0; i < vtMoveInfo.size(); i++)
	{
		if (vtMoveInfo[i].nMoveType == MOVL) tMoveValue.strMoveType = "MOVL";
		else if (vtMoveInfo[i].nMoveType == MOVC) tMoveValue.strMoveType = "MOVC";
		else if (vtMoveInfo[i].nMoveType == MOVJ) tMoveValue.strMoveType = "MOVJ";
		else if (vtMoveInfo[i].nMoveType == AngleSpdforMOVL) tMoveValue.strMoveType = "AngleSpdforMOVL";
		tMoveValue.nStepNo = 20 + i;
		tMoveValue.nRobotTool = vtMoveInfo[i].tCoord.val.p.tool_no;
		if (vtMoveInfo[i].tCoord.val.p.dtype == MP_ROBO_DTYPE)
		{
			tMoveValue.nValueType = MP_ROBO_DTYPE;
			tMoveValue.adCoor[0] = (double)vtMoveInfo[i].tCoord.val.p.data[0] / 1000.0;
			tMoveValue.adCoor[1] = (double)vtMoveInfo[i].tCoord.val.p.data[1] / 1000.0;
			tMoveValue.adCoor[2] = (double)vtMoveInfo[i].tCoord.val.p.data[2] / 1000.0;
			tMoveValue.adCoor[3] = (double)vtMoveInfo[i].tCoord.val.p.data[3] / 10000.0;
			tMoveValue.adCoor[4] = (double)vtMoveInfo[i].tCoord.val.p.data[4] / 10000.0;
			tMoveValue.adCoor[5] = (double)vtMoveInfo[i].tCoord.val.p.data[5] / 10000.0;
		}
		else
		{
			tMoveValue.nValueType = MP_PULSE_DTYPE;
			tMoveValue.alPulse[0] = vtMoveInfo[i].tCoord.val.p.data[0];
			tMoveValue.alPulse[1] = vtMoveInfo[i].tCoord.val.p.data[1];
			tMoveValue.alPulse[2] = vtMoveInfo[i].tCoord.val.p.data[2];
			tMoveValue.alPulse[3] = vtMoveInfo[i].tCoord.val.p.data[3];
			tMoveValue.alPulse[4] = vtMoveInfo[i].tCoord.val.p.data[4];
			tMoveValue.alPulse[5] = vtMoveInfo[i].tCoord.val.p.data[5];
		}
		tMoveValue.dSpeed = vtMoveInfo[i].tSpeed.dSpeed;
		if (FALSE == m_cDatabaseCtrl.InsertData_MoveValue(tMoveValue))
		{
			return false;
		}
	}
	return true;
}

T_ANGLE_PULSE CUnit_Debug::GetRobotPulse()
{
	if (!m_bStartDebug)
	{
		return CContralUnit::GetRobotPulse();
	}
	T_DEVICE_STATE tRobotState;
	tRobotState.strUnitName = GetUnitName();
	tRobotState.strDeviceType = "robot";
	tRobotState.strDeviceNo = "0";
	if (FALSE == m_cDatabaseCtrl.ReadData_DeviceState(tRobotState))
	{
		return T_ANGLE_PULSE();
	}
	T_ANGLE_PULSE tPulse;
	tPulse = TransforPulse(tRobotState.alPulse);
	tPulse.lBXPulse = tRobotState.alPulse[6];
	tPulse.lBYPulse = tRobotState.alPulse[7];
	tPulse.lBZPulse = tRobotState.alPulse[8];
	return tPulse;
}

T_ROBOT_COORS CUnit_Debug::GetRobotCoor()
{
	T_DEVICE_STATE tRobotState;
	tRobotState.strUnitName = GetUnitName();
	tRobotState.strDeviceType = "robot";
	tRobotState.strDeviceNo = "0";
	if (FALSE == m_cDatabaseCtrl.ReadData_DeviceState(tRobotState))
	{
		return T_ROBOT_COORS();
	}
	T_ROBOT_COORS tCoor;
	tCoor = TransforPos(tRobotState.adCoor);
	tCoor.dBX = tRobotState.adCoor[6];
	tCoor.dBY = tRobotState.adCoor[7];
	tCoor.dBZ = tRobotState.adCoor[8];
	return tCoor;
}

void CUnit_Debug::CreateDatabaseTable()
{
	T_DEVICE_STATE tRobotState;
	m_cDatabaseCtrl.CreatTable_MoveValue();
	m_cDatabaseCtrl.CreatTable_RobotTool();
	m_cDatabaseCtrl.CreatTable_DeviceState();

	tRobotState.strUnitName = GetUnitName();
	std::vector<T_MOVE_VALUE> vtMoveValue;
	m_cDatabaseCtrl.ReadData_MoveValue("", "", "0", vtMoveValue);
	m_cDatabaseCtrl.DeleteData_MoveValue("", "", "0", vtMoveValue);
	
	m_cDatabaseCtrl.ReadData_DeviceState(tRobotState);

}

void CUnit_Debug::CallJob(CString sJobName, int nExternalType)
{
	if (!m_bStartDebug)
	{
		CUnit::CallJob(sJobName, nExternalType);
		return;
	}
	if ("CONTIMOVANY" == sJobName)
	{
		AfxBeginThread(ThreadCallJob_Database, this);
	}
}

UINT CUnit_Debug::ThreadCallJob_Database(void* pParam)
{
	CUnit_Debug* pcMyObj = (CUnit_Debug*)pParam;
	pcMyObj->CallJob_Database();
	return 0;
}

void CUnit_Debug::CallJob_Database()
{
	if (m_bEmgStop)
	{
		return;
	}
	std::vector<T_MOVE_VALUE> vtMoveValue;
	m_cDatabaseCtrl.ReadData_MoveValue(GetUnitName(), "robot", "0", vtMoveValue);

	m_cDatabaseCtrl.UpdateData_DeviceState_MoveState(GetUnitName(), "robot", "0", TRUE);
	m_cDatabaseCtrl.UpdateData_DeviceState_MoveStep(GetUnitName(), "robot", "0", 0);

	int nRtn = 0;
	size_t i = 0;
	for (i = 0; i < vtMoveValue.size(); i++)
	{
		if (vtMoveValue[i].strMoveType == "MOVL")
		{
			nRtn = CallJob_Database_MOVL(vtMoveValue[i]);
			if (nRtn < 0)
			{
				break;
			}
		}
		else if (vtMoveValue[i].strMoveType == "MOVJ")
		{
			nRtn = CallJob_Database_MOVJ(vtMoveValue[i]);
			if (nRtn < 0)
			{
				break;
			}
		}
		m_cDatabaseCtrl.UpdateData_DeviceState_MoveStep(GetUnitName(), "robot", "0", i + 1);
	}
	if (i == vtMoveValue.size())
	{
		m_cDatabaseCtrl.UpdateData_DeviceState_MoveStep(GetUnitName(), "robot", "0", 0);
	}


	m_cDatabaseCtrl.DeleteData_MoveValue(GetUnitName(), "robot", "0", vtMoveValue);
	m_cDatabaseCtrl.UpdateData_DeviceState_MoveState(GetUnitName(), "robot", "0", FALSE);

}

int CUnit_Debug::CallJob_Database_MOVL(T_MOVE_VALUE tMoveValue)
{
	T_ROBOT_COORS tCurrentCoor = GetRobotPos(tMoveValue.nRobotTool);
	T_ROBOT_COORS tRobotCoor = TransforPos(tMoveValue.adCoor);

	if (tMoveValue.nValueType == MP_PULSE_DTYPE)
	{
		GetRobotCtrl()->RobotKinematics(TransforPulse(tMoveValue.alPulse), GetRobotTool(tMoveValue.nRobotTool), tRobotCoor);
	}

	double dDistance = GetDistance(tCurrentCoor, tRobotCoor);
	if (dDistance < 0.5)
	{
		return 1;
	}
	double dTime = dDistance / (m_dDebugSpeedRatio * tMoveValue.dSpeed / 10.0);
	double dStepDistance[9];
	dStepDistance[0] = (tRobotCoor.dX - tCurrentCoor.dX) / dTime / 10.0;
	dStepDistance[1] = (tRobotCoor.dY - tCurrentCoor.dY) / dTime / 10.0;
	dStepDistance[2] = (tRobotCoor.dZ - tCurrentCoor.dZ) / dTime / 10.0;
	dStepDistance[3] = (tRobotCoor.dRX - tCurrentCoor.dRX) / dTime / 10.0;
	dStepDistance[4] = (tRobotCoor.dRY - tCurrentCoor.dRY) / dTime / 10.0;
	dStepDistance[5] = (tRobotCoor.dRZ - tCurrentCoor.dRZ) / dTime / 10.0;
	dStepDistance[6] = (tRobotCoor.dBX - tCurrentCoor.dBX) / dTime / 10.0;
	dStepDistance[7] = (tRobotCoor.dBY - tCurrentCoor.dBY) / dTime / 10.0;
	dStepDistance[8] = (tRobotCoor.dBZ - tCurrentCoor.dBZ) / dTime / 10.0;

	double adMoveCoor[9] = { 0 };
	long alMovePulse[9] = { 0 };
	T_ROBOT_COORS tMoveCoor = tCurrentCoor;
	int nMoveNum = (int)(dTime * 10.0 + 0.5);
	for (int i = 0; i < nMoveNum-1; i++)
	{
		if (m_bEmgStop)
		{
			return -10;
		}

		long long lStartTime = XI_clock();
		tMoveCoor.dX += dStepDistance[0];
		tMoveCoor.dY += dStepDistance[1];
		tMoveCoor.dZ += dStepDistance[2];
		tMoveCoor.dRX += dStepDistance[3];
		tMoveCoor.dRY += dStepDistance[4];
		tMoveCoor.dRZ += dStepDistance[5];
		tMoveCoor.dBX += dStepDistance[6];
		tMoveCoor.dBY += dStepDistance[7];
		tMoveCoor.dBZ += dStepDistance[8];

		//if (GetDistance(tCurrentCoor, tMoveCoor) > dDistance)
		//{
		//	break;
		//}


		T_ANGLE_PULSE tMovePulse;
		if (false == GetRobotCtrl()->RobotInverseKinematics(tMoveCoor, GetRobotPulse(), GetRobotTool(tMoveValue.nRobotTool), tMovePulse))
		{
			WRITE_LOG(GetStr("转换失败：Coor=%s, Tool=%s", GetStr(tMoveCoor), GetStr(GetRobotTool(tMoveValue.nRobotTool))));
			tMovePulse = GetRobotPulse();
		}
		TransforArray(tMoveCoor, adMoveCoor);
		TransforArray(tMovePulse, alMovePulse);
		//			WRITE_LOG(GetStr("tMoveCoor=%s, tMovePulse=%s", GetStr(tMoveCoor), GetStr(tMovePulse)));
		m_cDatabaseCtrl.UpdateData_DeviceState_Coor(GetUnitName(), "robot", "0", adMoveCoor, alMovePulse);


		long long lUsedTime = XI_clock() - lStartTime;
		if (lUsedTime < 100)
		{
			Sleep(100 - lUsedTime);
		}
	}
	T_ANGLE_PULSE tMovePulse;
	GetRobotCtrl()->RobotInverseKinematics(tRobotCoor, GetRobotPulse(), GetRobotTool(tMoveValue.nRobotTool), tMovePulse);
	TransforArray(tRobotCoor, adMoveCoor);
	TransforArray(tMovePulse, alMovePulse);
//	WRITE_LOG(GetStr("最终：tMoveCoor=%s, tMovePulse=%s", GetStr(tRobotCoor), GetStr(tMovePulse)));
	m_cDatabaseCtrl.UpdateData_DeviceState_Coor(GetUnitName(), "robot", "0", adMoveCoor, alMovePulse);
	return 0;
}

int CUnit_Debug::CallJob_Database_MOVJ(T_MOVE_VALUE tMoveValue)
{
	T_ANGLE_PULSE tCurrentPulse = GetRobotPulse();
	T_ANGLE_PULSE tRobotPulse = TransforPulse(tMoveValue.alPulse);

	if (tMoveValue.nValueType == MP_ROBO_DTYPE)
	{
		if (!GetRobotCtrl()->RobotInverseKinematics(TransforPos(tMoveValue.adCoor), tCurrentPulse, GetRobotTool(tMoveValue.nRobotTool), tRobotPulse))
		{
			return -1;
		}
	}

	T_ROBOT_COORS tCurrentCoor;
	GetRobotCtrl()->RobotKinematics(tCurrentPulse, GetRobotTool(tMoveValue.nRobotTool), tCurrentCoor);
	T_ROBOT_COORS tRobotCoor;
	GetRobotCtrl()->RobotKinematics(tRobotPulse, GetRobotTool(tMoveValue.nRobotTool), tRobotCoor);

	//double dDistance = GetDistance(tCurrentCoor, tRobotCoor);
	//if (dDistance < 0.5)
	//{
	//	return 1;
	//}

	T_ANGLE_PULSE tDistancePulse = GetAxisDiff(tRobotPulse, tCurrentPulse);
	long lDistance = GetMaxPulse(tDistancePulse);
	if (tDistancePulse < 10)
	{
		return 1;
	}
	double dTime = lDistance / (m_dDebugSpeedRatio * (tMoveValue.dSpeed / 10000.0) * 90000);
	double dStepDistance[9];
	dStepDistance[0] = (tRobotPulse.nSPulse  - tCurrentPulse.nSPulse ) / dTime / 10.0;
	dStepDistance[1] = (tRobotPulse.nLPulse  - tCurrentPulse.nLPulse ) / dTime / 10.0;
	dStepDistance[2] = (tRobotPulse.nUPulse  - tCurrentPulse.nUPulse ) / dTime / 10.0;
	dStepDistance[3] = (tRobotPulse.nRPulse  - tCurrentPulse.nRPulse ) / dTime / 10.0;
	dStepDistance[4] = (tRobotPulse.nBPulse  - tCurrentPulse.nBPulse ) / dTime / 10.0;
	dStepDistance[5] = (tRobotPulse.nTPulse  - tCurrentPulse.nTPulse ) / dTime / 10.0;
	dStepDistance[6] = (tRobotPulse.lBXPulse - tCurrentPulse.lBXPulse) / dTime / 10.0;
	dStepDistance[7] = (tRobotPulse.lBYPulse - tCurrentPulse.lBYPulse) / dTime / 10.0;
	dStepDistance[8] = (tRobotPulse.lBZPulse - tCurrentPulse.lBZPulse) / dTime / 10.0;

	double adMoveCoor[9] = { 0 };
	long alMovePulse[9] = { 0 };
	T_ANGLE_PULSE tMovePulse = tCurrentPulse;
	int nMoveNum = (int)(dTime * 10.0 + 0.5);
	for (int i = 0; i < nMoveNum - 1; i++)
	{
		if (m_bEmgStop)
		{
			return -10;
		}

		long long lStartTime = XI_clock();
		tMovePulse.nSPulse  += dStepDistance[0];
		tMovePulse.nLPulse  += dStepDistance[1];
		tMovePulse.nUPulse  += dStepDistance[2];
		tMovePulse.nRPulse  += dStepDistance[3];
		tMovePulse.nBPulse  += dStepDistance[4];
		tMovePulse.nTPulse  += dStepDistance[5];
		tMovePulse.lBXPulse += dStepDistance[6];
		tMovePulse.lBYPulse += dStepDistance[7];
		tMovePulse.lBZPulse += dStepDistance[8];


		T_ROBOT_COORS tMoveCoor;
		GetRobotCtrl()->RobotKinematics(tMovePulse, GetRobotTool(tMoveValue.nRobotTool), tMoveCoor);
		//if (GetDistance(tCurrentCoor, tMoveCoor) > dDistance)
		//{
		//	break;
		//}

		TransforArray(tMoveCoor, adMoveCoor);
		TransforArray(tMovePulse, alMovePulse);
		m_cDatabaseCtrl.UpdateData_DeviceState_Coor(GetUnitName(), "robot", "0", adMoveCoor, alMovePulse);


		long long lUsedTime = XI_clock() - lStartTime;
		if (lUsedTime < 100)
		{
			Sleep(100 - lUsedTime);
		}
	}
	TransforArray(tRobotCoor, adMoveCoor);
	TransforArray(tRobotPulse, alMovePulse);
//	WRITE_LOG(GetStr("最终：tMoveCoor=%s, tMovePulse=%s", GetStr(tRobotCoor), GetStr(tRobotPulse)));
	m_cDatabaseCtrl.UpdateData_DeviceState_Coor(GetUnitName(), "robot", "0", adMoveCoor, tMoveValue.alPulse);
	return 0;
}

int CUnit_Debug::AbsPosMove(int nUnitMotorNo, double dAbsPosture, double dSpeed, double dAcc, double dDec, double dSParam)
{
	if (!m_bStartDebug)
	{
		return CContralUnit::AbsPosMove(nUnitMotorNo, dAbsPosture, dSpeed, dAcc, dDec, dSParam);
	}
	return AbsPosMove_Database(nUnitMotorNo, dAbsPosture, dSpeed, dAcc, dDec, dSParam);
}

int CUnit_Debug::AbsPosMove_Database(int nUnitMotorNo, double dAbsPosture, double dSpeed, double dAcc, double dDec, double dSParam)
{
	T_MOVE_VALUE tMoveValue;
	tMoveValue.strUnitName = GetUnitName();
	tMoveValue.strMoveDevice = "motor";
	tMoveValue.strMoveType = "MOVL";
	tMoveValue.strDeviceNo.Format("%d", nUnitMotorNo);
	tMoveValue.dSpeed = dSpeed;
	tMoveValue.adCoor[0] = dAbsPosture;
	std::vector<T_MOVE_VALUE> vtMoveValue;
	m_cDatabaseCtrl.ReadData_MoveValue(GetUnitName(), "motor", tMoveValue.strDeviceNo, vtMoveValue);
	m_cDatabaseCtrl.DeleteData_MoveValue(GetUnitName(), "motor", tMoveValue.strDeviceNo, vtMoveValue);
	if (FALSE == m_cDatabaseCtrl.InsertData_MoveValue(tMoveValue))
	{
		return -1;
	}
	T_MOTOR_MOVE* ptMotorMove;
	ptMotorMove = new T_MOTOR_MOVE;
	ptMotorMove->pFather = this;
	ptMotorMove->strDeviceNo.Format("%d", nUnitMotorNo);
	AfxBeginThread(ThreadPosMove_Database, ptMotorMove);
	return 0;
}
UINT CUnit_Debug::ThreadPosMove_Database(void* pParam)
{
	T_MOTOR_MOVE cMyObj = *((T_MOTOR_MOVE*)pParam);
	delete pParam;
	cMyObj.pFather->PosMove_Database(cMyObj.strDeviceNo);
	return 0;
}

void CUnit_Debug::PosMove_Database(CString strDeviceNo)
{
	if (m_bEmgStop)
	{
		return;
	}
	std::vector<T_MOVE_VALUE> vtMoveValue;
	m_cDatabaseCtrl.ReadData_MoveValue(GetUnitName(), "motor", strDeviceNo, vtMoveValue);
	if (vtMoveValue.empty() || strDeviceNo.IsEmpty())
	{
		return;
	}

	T_MOVE_VALUE tMoveValue = vtMoveValue.back();
	int nDeviceNo = atoi(strDeviceNo);
	double dCurrentCoor;
	GetCurrentPosition(nDeviceNo, dCurrentCoor);
	double dAimCoor = tMoveValue.adCoor[0];

	if (tMoveValue.strMoveType == "MOVJ")
	{
		dAimCoor = GetMotorCtrl(nDeviceNo)->CoorChange(tMoveValue.alPulse[0]);
	}

	double dDistance = fabs(dAimCoor - dCurrentCoor);
	if (dDistance < 0.5)
	{
		return;
	}

	m_cDatabaseCtrl.UpdateData_DeviceState_MoveState(GetUnitName(), "motor", strDeviceNo, TRUE);

	double dTime = dDistance / (m_dDebugSpeedRatio * tMoveValue.dSpeed);
	double dStepDistance = (dAimCoor - dCurrentCoor) / dTime / 10.0;

	double adMoveCoor[9] = { 0 };
	long alMovePulse[9] = { 0 };
	double dMoveCoor = dCurrentCoor;
	int nMoveNum = (int)(dTime * 10.0 + 0.5);
	for (int i = 0; i < nMoveNum - 1; i++)
//	while (true)
	{
		if (m_bEmgStop)
		{
			m_cDatabaseCtrl.DeleteData_MoveValue(GetUnitName(), "motor", strDeviceNo, vtMoveValue);
			m_cDatabaseCtrl.UpdateData_DeviceState_MoveState(GetUnitName(), "motor", strDeviceNo, FALSE);
			return;
		}
		long long lStartTime = XI_clock();
		dMoveCoor += dStepDistance;

		//if (fabs(dCurrentCoor - dMoveCoor) > dDistance)
		//{
		//	break;
		//}
		adMoveCoor[0] = dMoveCoor;
		alMovePulse[0] = GetMotorCtrl(nDeviceNo)->CoorChange(adMoveCoor[0]);
		m_cDatabaseCtrl.UpdateData_DeviceState_Coor(GetUnitName(), "motor", strDeviceNo, adMoveCoor, alMovePulse);

		long long lUsedTime = XI_clock() - lStartTime;
		if (lUsedTime < 100)
		{
			Sleep(100 - lUsedTime);
		}
	}
	adMoveCoor[0] = dAimCoor;
	alMovePulse[0] = GetMotorCtrl(nDeviceNo)->CoorChange(adMoveCoor[0]);
	m_cDatabaseCtrl.UpdateData_DeviceState_Coor(GetUnitName(), "motor", strDeviceNo, adMoveCoor, alMovePulse);

	m_cDatabaseCtrl.DeleteData_MoveValue(GetUnitName(), "motor", strDeviceNo, vtMoveValue);
	m_cDatabaseCtrl.UpdateData_DeviceState_MoveState(GetUnitName(), "motor", strDeviceNo, FALSE);

}

BOOL CUnit_Debug::MotorCheckRunning(int nUnitMotorNo)
{
	if (!m_bStartDebug)
	{
		WORD wState;
		int nRtn = GetRobotServoSignal(wState);
		if (nRtn != 0 || wState == OFF)
		{
			Sleep(100);
			nRtn = GetRobotServoSignal(wState);
			if (nRtn != 0 || wState == OFF)
			{
				WRITE_LOG_UNIT("机器人伺服掉电导致外部轴急停！");
			}
		}
		return CContralUnit::MotorCheckRunning(nUnitMotorNo);
	}
	return MotorCheckRunning_Database(nUnitMotorNo);
}

BOOL CUnit_Debug::MotorCheckRunning_Database(int nUnitMotorNo)
{
	T_DEVICE_STATE tRobotState;
	tRobotState.strUnitName = GetUnitName();
	tRobotState.strDeviceType = "motor";
	tRobotState.strDeviceNo.Format("%d", nUnitMotorNo);
	m_cDatabaseCtrl.ReadData_DeviceState(tRobotState);
	return tRobotState.bMoveState;
}

BOOL CUnit_Debug::MotorCheckDone(int nUnitMotorNo, double dAbsPosture)
{
	if (!m_bStartDebug)
	{
		return CContralUnit::MotorCheckDone(nUnitMotorNo, dAbsPosture);
	}
	return MotorCheckDone_Database(nUnitMotorNo, dAbsPosture);
}

BOOL CUnit_Debug::MotorCheckNoWait(int nUnitMotorNo, double dAbsPosture)
{
	if (MotorCheckRunning(nUnitMotorNo) == TRUE)
	{
		return TRUE;
	}
	double dPosition;
	int nRtn = GetCurrentPosition(nUnitMotorNo, dPosition);
	if (!IsEqual(dAbsPosture, dPosition, 0.5))
	{
		return FALSE;
	}
	return TRUE;
}

BOOL CUnit_Debug::MotorCheckDone(int nUnitMotorNo, double dAbsPosture, double dEarlyEndDistance)
{
	if (!m_bStartDebug)
	{
		return CContralUnit::MotorCheckDone_CheckRobot(nUnitMotorNo, dAbsPosture, dEarlyEndDistance);
	}
	return MotorCheckDone_Database(nUnitMotorNo, dAbsPosture, dEarlyEndDistance);
}

BOOL CUnit_Debug::MotorCheckDone_Database(int nUnitMotorNo, double dAbsPosture)
{
	while (1)
	{
		Sleep(50);
		DoEvent();
		if (!MotorCheckRunning_Database(nUnitMotorNo))
		{
			Sleep(30);
			if (!MotorCheckRunning_Database(nUnitMotorNo))
			{
				break;
			}
		}
	}
	double dCurrentPosition;
	int nRtn = GetCurrentPosition_Database(nUnitMotorNo, dCurrentPosition);
	if (!IsEqual(dAbsPosture, dCurrentPosition, 0.5))
	{
		WRITE_LOG(GetStr("%lf, %lf", dAbsPosture, dCurrentPosition));
		return FALSE;
	}
	return TRUE;
}

BOOL CUnit_Debug::MotorCheckDone_Database(int nUnitMotorNo, double dAbsPosture, double dEarlyEndDistance)
{
	long long lTime = XI_clock();
	while (!MotorCheckRunning_Database(nUnitMotorNo))
	{
		Sleep(10);
		DoEvent();
		if (XI_clock() - lTime)
		{
			WRITE_LOG_UNIT(GetStr("MotorCheckDone未运动，轴:%d", nUnitMotorNo));
			return FALSE;
		}
	}
	while (MotorCheckRunning_Database(nUnitMotorNo))
	{
		double dCurrentPosition;
		GetCurrentPosition_Database(nUnitMotorNo, dCurrentPosition);
		if (fabs(dAbsPosture - dCurrentPosition) < dEarlyEndDistance)
		{
			Sleep(100);
			GetCurrentPosition_Database(nUnitMotorNo, dCurrentPosition);
			if (fabs(dAbsPosture - dCurrentPosition) < dEarlyEndDistance)
			{
				break;
			}
		}
	}
	if (!MotorCheckRunning_Database(nUnitMotorNo))
	{
		return MotorCheckDone_Database(nUnitMotorNo, dAbsPosture);
	}
	return TRUE;
}

int CUnit_Debug::GetCurrentPosition(int nUnitMotorNo, double& dPosition)
{
	if (!m_bStartDebug)
	{
		return CContralUnit::GetCurrentPosition(nUnitMotorNo, dPosition);
	}
	return GetCurrentPosition_Database(nUnitMotorNo, dPosition);
}

int CUnit_Debug::GetCurrentPosition_Database(int nUnitMotorNo, double& dPosition)
{
	T_DEVICE_STATE tRobotState;
	tRobotState.strUnitName = GetUnitName();
	tRobotState.strDeviceType = "motor";
	tRobotState.strDeviceNo.Format("%d", nUnitMotorNo);
	if (FALSE == m_cDatabaseCtrl.ReadData_DeviceState(tRobotState))
	{
		return -1;
	}
	dPosition = tRobotState.adCoor[0];
	return 0;
}


BOOL CUnit_Debug::RobotBackHome(T_ROBOT_MOVE_SPEED tSpeed)
{
	if (!m_bStartDebug)
	{
		return CUnit::RobotBackHome();
	}

	double adMoveCoor[9] = { 0 };
	long alMovePulse[9] = { 0 };
	T_ROBOT_COORS tRobotCoor;
	GetRobotCtrl()->RobotKinematics(GetRobotCtrl()->m_tHomePulse, GetRobotTool(1), tRobotCoor);
	TransforArray(tRobotCoor, adMoveCoor);
	TransforArray(GetRobotCtrl()->m_tHomePulse, alMovePulse);
	m_cDatabaseCtrl.UpdateData_DeviceState_Coor(GetUnitName(), "robot", "0", adMoveCoor, alMovePulse);
	return TRUE;
}

#endif