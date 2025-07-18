#include "stdafx.h"
#include "CUnit.h"
#include ".\LocalFiles\ExLib\ALGO\include\TrackFilterHandle_C_DLL.h"
#include ".\LocalFiles\ExLib\ALGO\include\TrackSmoothHandle_C_DLL.h"
#include "Apps/RealTimeTrack/RealTimeTrack.h"

CUnit::CUnit(T_CONTRAL_UNIT tCtrlUnitInfo, CServoMotorDriver *pServoMotorDriver)
	:CUnitDataManagement(tCtrlUnitInfo, pServoMotorDriver)
	, m_dExAxisXPos(0.0)
	, m_dExAxisYPos(0.0)
	, m_dExAxisZPos(0.0)
{
	CalculateTransRelation(m_GantryTransRobotMat, m_RobotTransGantryMat, m_dRobotBaseToGantryCtrDis, m_dGantryRobotOriginDis);
	SetAbsEncoderCoor();
	LoadExAixsFun();
	CreatTrackFilteObject();
}

bool CUnit::SetMoveValue(std::vector<T_ROBOT_MOVE_INFO> vtMoveInfo)
{
	return GetRobotCtrl()->SetMoveValue(vtMoveInfo);
}

bool CUnit::SetMoveValue(CRobotMove cMoveInfo)
{
	return GetRobotCtrl()->SetMoveValue(cMoveInfo);
}

int CUnit::GetMoveStep()
{
	return GetRobotCtrl()->GetMoveStep();
}

int CUnit::CleanMoveStep()
{
	return GetRobotCtrl()->CleanMoveStep();
}

void CUnit::CallJob(CString sJobName, int nExternalType)
{
	GetRobotCtrl()->CallJob(sJobName, nExternalType);
}



bool CUnit::ComparePulse(T_ANGLE_PULSE tPulse1, T_ANGLE_PULSE tPulse2, long lLimit /*= 30*/)
{
	if (fabs((double)(tPulse1.nSPulse - tPulse2.nSPulse)) >= lLimit
		|| fabs((double)(tPulse1.nLPulse - tPulse2.nLPulse)) >= lLimit
		|| fabs((double)(tPulse1.nUPulse - tPulse2.nUPulse)) >= lLimit
		|| fabs((double)(tPulse1.nRPulse - tPulse2.nRPulse)) >= lLimit
		|| fabs((double)(tPulse1.nBPulse - tPulse2.nBPulse)) >= lLimit
		|| fabs((double)(tPulse1.nTPulse - tPulse2.nTPulse)) >= lLimit)
	{
		return false;
	}
	return true;
}

bool CUnit::ComparePulse_Base(T_ANGLE_PULSE tPulse1, double dLimit)
{
	CRobotDriverAdaptor* pHandlingRobot = GetRobotCtrl();
	T_ANGLE_PULSE tPulse2 = GetRobotPulse();
	bool bRtn = true;
	int nType = pHandlingRobot->m_nExternalAxleType;
	double dBX = pHandlingRobot->m_tExternalAxle[0].dPulse * fabs(tPulse1.lBXPulse - tPulse2.lBXPulse);
	double dBY = pHandlingRobot->m_tExternalAxle[1].dPulse * fabs(tPulse1.lBYPulse - tPulse2.lBYPulse);
	double dBZ = pHandlingRobot->m_tExternalAxle[2].dPulse * fabs(tPulse1.lBZPulse - tPulse2.lBZPulse);
	if (((dBX >= dLimit) && (((nType) & 0x01) == 1))
		|| ((dBY >= dLimit) && (((nType >> 1) & 0x01) == 1))
		|| ((dBZ >= dLimit) && (((nType >> 2) & 0x01) == 1)))
	{
		bRtn = false;
	}
	return bRtn;
}

void CUnit::UnitEmgStop()
{
	if (GetMotorEnableState())
	{
		for (size_t i = 0; i < GetMotorNum(); i++)
		{
			GetMotorCtrl(i)->EmgStopAxis();
		}
	}
	if (GetRobotEnableState())
	{
		GetRobotCtrl()->HoldOn();
		GetRobotCtrl()->ServoOff();
		GetRobotCtrl()->HoldOff();
	}
}

bool CUnit::RobotEmg(bool bPopup)
{
	switch (GetRobotCtrl()->m_eRobotBrand) {
	case ROBOT_BRAND_YASKAWA:		//安川
	{
		CString strRobotName = GetRobotCtrl()->m_strRobotName;
		bool bStopArc = false;
		//依赖项↓
		if (false == CheckInIO("Servo")) {
			Sleep(100);
			if (false == CheckInIO("Servo")) {
				Sleep(100);
				if (false == CheckInIO("Servo")) {
					GetRobotCtrl()->m_eThreadStatus = INCISEHEAD_THREAD_STATUS_STOPPED;
					GetRobotCtrl()->HoldOn();
					if (bPopup)
					{
						//已修改
						XUI::MesBox::PopInfo("{0}Warning:RobotEmg():机械臂已急停！", strRobotName.GetBuffer());
						//SwitchIO("alarmlight", true);
						//XiMessageBox("%s Warning:RobotEmg():机械臂已急停！", strRobotName);
					}
					return true;
				}
			}
		}
	}
	break;
	case ROBOT_BRAND_ESTUN:			//埃斯顿
		break;
	case ROBOT_BRAND_CRP:				//卡诺普
	{
		// 急停或未伺服上电 返回false 否则返回true
		if (GetRobotCtrl()->m_pCrpRobotCtrl->Read_M_Number_status(20) == 0) {
			return true;
		}
		return false;
	}
	break;
	default:
		break;
	}
	return false;
}

bool CUnit::RobotBackHome(T_ROBOT_MOVE_SPEED tSpeed)
{
	if (GetRobotCtrl()->ComparePulse(GetRobotCtrl()->m_tHomePulse, GetRobotPulse(), T_ANGLE_PULSE(10, 10, 10, 10, 10, 10, -1, -1, -1)))
	{
		return true;
	}
	//BOOL bRtn;
	//std::vector<T_ROBOT_MOVE_INFO> vtRobotMoveTrack;
	//CRobotMove cMoveInfo;
	//T_ROBOT_COORS tTenp = GetRobotPos(1);
	//tTenp.dZ += 100;
	//T_ANGLE_PULSE tPulse = GetRobotCtrl()->m_tHomePulse;
	//tPulse.lBXPulse = GetRobotPulse().lBXPulse;
	//cMoveInfo.InsertCoor(tTenp, tSpeed, 1, MOVJ, 1);
	//cMoveInfo.InsertCoor(tPulse, tSpeed, 1, MOVJ, 1);
	//bRtn = SetMoveValue(cMoveInfo);
	//CHECK_BOOL_RTN_UNIT(bRtn, "设置失败");
	//CallJob("CONTIMOVANY", GetRobotCtrl()->m_nExternalAxleType);
	//bRtn = RobotCheckDone(GetRobotCtrl()->m_tHomePulse, T_ANGLE_PULSE(10, 10, 10, 10, 10, 10, -1, -1, -1));
	//CHECK_BOOL_RTN_UNIT(bRtn, "运动失败");

	CRobotDriverAdaptor* pRobotDriver = GetRobotCtrl();
	int nRobotNo = pRobotDriver->m_nRobotNo;
	CHECK_BOOL_RETURN(CheckIsReadyRun());

	// 正座机器人 or 倒挂机器人 ？
	double dRobotInstallMode = pRobotDriver->m_nRobotInstallDir; // 正装：1.0   倒装：-1.0
	double dMinUpMoveDis = 100.0;
	double dBackHeightThreshold = 0.0; // 400.0;
	T_ANGLE_PULSE tCurPulse;
	T_ROBOT_COORS tCurCoord;
	T_ROBOT_COORS tTransCoord;
	T_ROBOT_COORS tTempCoord;
	T_ROBOT_COORS tHomeCoord;
	T_ROBOT_MOVE_INFO tRobotMoveInfo;
	vector<T_ROBOT_MOVE_INFO> vtRobotMoveInfo(0);

	MoveExAxleToSafe(); // 国焊临时

	tCurCoord = pRobotDriver->GetCurrentPos();
	tCurPulse = pRobotDriver->GetCurrentPulse();
	pRobotDriver->RobotKinematics(pRobotDriver->m_tHomePulse, pRobotDriver->m_tTools.tGunTool, tHomeCoord);
	pRobotDriver->RobotKinematics(tCurPulse, pRobotDriver->m_tTools.tGunTool, tTransCoord);

	// 读取直角坐标和读取关节坐标转换出来结果不同时，不能运动
	if (false == pRobotDriver->CompareCoords(tTransCoord, tCurCoord))
	{
		WriteLog("MoveToSafeHeight:读取坐标和计算坐标误差过大");
		//已修改
		XUI::MesBox::PopInfo("读取坐标和计算坐标不同，无法自动回安全位置！");
		//XiMessageBoxOk("读取坐标和计算坐标不同，无法自动回安全位置！");
		return false;
	}

	double dMinBackGunHeight = tHomeCoord.dZ - (dBackHeightThreshold * dRobotInstallMode);
	// 高度小于dMinBackGunHeight时 执行操作：1、RY恢复标准45  2、最少抬高100mm（MOVL距离太小速度非常快）
	tTempCoord = tCurCoord;
	//tTempCoord.dRX = tHomeCoord.dRX;
	//tTempCoord.dRY = tHomeCoord.dRY;
	if ((dRobotInstallMode > 0.0 && tCurCoord.dZ < dMinBackGunHeight) ||
		(dRobotInstallMode < 0.0 && tCurCoord.dZ > dMinBackGunHeight))
	{
		if (fabs(tCurCoord.dZ - dMinBackGunHeight) < dMinUpMoveDis)
		{
			tTempCoord.dZ += (dMinUpMoveDis * dRobotInstallMode);
		}
		else
		{
			tTempCoord.dZ = dMinBackGunHeight;
		}
		tRobotMoveInfo = pRobotDriver->PVarToRobotMoveInfo(0, tTempCoord, pRobotDriver->m_tCoordHighSpeed, MOVL);
		vtRobotMoveInfo.push_back(tRobotMoveInfo);
	}
	tRobotMoveInfo = pRobotDriver->PVarToRobotMoveInfo(1, pRobotDriver->m_tHomePulse, pRobotDriver->m_tBackHomeSpeed, MOVJ);
	vtRobotMoveInfo.push_back(tRobotMoveInfo);
	pRobotDriver->SetMoveValue(vtRobotMoveInfo);
	pRobotDriver->CallJob("CONTIMOVANY");
	RobotCheckDone();

	if (false == pRobotDriver->ComparePulse(pRobotDriver->m_tHomePulse))
	{
		//SetHintInfo("回安全位置失败！");
		return false;
	}
	//SetHintInfo("回安全位置完成！");
	return true;
}

bool CUnit::MoveExAxleToSafe(T_ANGLE_PULSE* ptDstPulse/* = NULL*/)
{
	// 运动外部轴到安全位置
	T_ROBOT_MOVE_INFO tRobotMoveInfo;
	vector<T_ROBOT_MOVE_INFO> vtRobotMoveInfo(0);
	T_ROBOT_COORS tCoord = GetRobotCtrl()->GetCurrentPos();
	if (tCoord.dBY < -1500.0) tCoord.dBY = -1500;
	if (tCoord.dBY > 1500.0) tCoord.dBY = 1500;
	if (tCoord.dBZ < 500.0) tCoord.dBZ = 500;
	if (tCoord.dBZ > 2000.0) tCoord.dBZ = 2000;

	if (NULL != ptDstPulse)
	{
		T_ROBOT_COORS tDstCoord;
		GetRobotCtrl()->RobotKinematics(*ptDstPulse, GetRobotCtrl()->m_tTools.tGunTool, tDstCoord);
		tCoord.dBY = tDstCoord.dBY;
		if (tCoord.dBY < -1500.0) tCoord.dBY = -1500;
		if (tCoord.dBY > 1500.0) tCoord.dBY = 1500;
	}

	tRobotMoveInfo = GetRobotCtrl()->PVarToRobotMoveInfo(0, tCoord, GetRobotCtrl()->m_tExAxlePulseSpeed, MOVJ);
	vtRobotMoveInfo.push_back(tRobotMoveInfo);
	GetRobotCtrl()->SetMoveValue(vtRobotMoveInfo, false, true);
	GetRobotCtrl()->CallJob("CONTIMOVANY-BP");
	RobotCheckDone();
	return true;
}

T_RUNNING_RTN_RESULT CUnit::SetAbsEncoderCoor()
{
	T_RUNNING_RTN_RESULT tRtnResult;
	int nRtn = 0;
	for (size_t i = 0; i < GetMotorNum(); i++)
	{
		nRtn = GetMotorCtrl(i)->OpenAbsEncoder();
		if (nRtn != 0)
		{
			RTN_STATE_RESULT(tRtnResult, RTN_STATUS_DERVICE_ERROR);
		}
		double dPosition;
		nRtn = GetMotorCtrl(i)->GetAbsData(dPosition);
		if (nRtn != 0)
		{
			RTN_STATE_RESULT(tRtnResult, RTN_STATUS_DERVICE_ERROR);
		}
		nRtn = GetMotorCtrl(i)->SetCurrentPosition(dPosition);
		if (nRtn != 0)
		{
			RTN_STATE_RESULT(tRtnResult, RTN_STATUS_DERVICE_ERROR);
		}
	}
	RTN_STATE_RESULT(tRtnResult, RTN_STATUS_SUCCEED);
}

CString CUnit::GetTrackTipDataFileName(int nCameraNo)
{
	CString sFileName = DATA_PATH + GetRobotCtrl()->m_strRobotName + "\\TrackDipParam_" + GetCameraParam(nCameraNo).strCameraNameEN + ".ini";
	return sFileName;
}

double CUnit::GetPositionDis()
{
	if (ROBOT_BRAND_ESTUN == GetRobotCtrl()->m_eRobotBrand)
	{
		GetRobotCtrl()->GetPositionDis();
	}
	double dCurPos = 0.0;
	GetRobotCtrl()->m_pvpMotorDriver->at(0)->GetCurrentPosition(dCurPos);
	return dCurPos;
}

//T_ROBOT_COORS CUnit::GetCurrentPosWorld()
//{
//	T_ROBOT_COORS tCurCoordWorld;
//	return tCurCoordWorld;
//}
//
//T_ANGLE_PULSE CUnit::GetCurrentPulseWorld()
//{
//	T_ANGLE_PULSE tCurPulseWorld;
//	return tCurPulseWorld;
//}

int CUnit::WorldMoveByJob(T_ANGLE_PULSE tPulse, T_ROBOT_MOVE_SPEED tPulseMove, CString JobName)
{
	if (ROBOT_BRAND_ESTUN == GetRobotCtrl()->m_eRobotBrand)
	{
		GetRobotCtrl()->MoveToLineScanPos(tPulse, tPulseMove);
		return 0;
	}

	// 非机器人扩展轴此处调用单轴插补
	bool bMoveExAxle = false;
	double dExAxisPos = 0.0;
	int nExternalAxleType = GetRobotCtrl()->m_nExternalAxleType;
	if (((nExternalAxleType & 0x01) == 1) && (((nExternalAxleType >> 6) & 0x01) == 1))
	{
		dExAxisPos = tPulse.lBXPulse * GetRobotCtrl()->m_tExternalAxle[0].dPulse;
		bMoveExAxle = true;
	}
	else if ((((nExternalAxleType >> 1) & 0x01) == 1) && (((nExternalAxleType >> 7) & 0x01) == 1))
	{
		dExAxisPos = tPulse.lBYPulse * GetRobotCtrl()->m_tExternalAxle[1].dPulse;
		bMoveExAxle = true;
	}
	else if ((((nExternalAxleType >> 2) & 0x01) == 1) && (((nExternalAxleType >> 8) & 0x01) == 1))
	{
		dExAxisPos = tPulse.lBZPulse * GetRobotCtrl()->m_tExternalAxle[2].dPulse;
		bMoveExAxle = true;
	}
	if (true == bMoveExAxle)
	{
		double dMaxSpeed = GetMotorCtrl()->GetMaxSpeed(0); // 临时
		//if (0 != MoveExAxisForLineScan(dExAxisPos, 6000.0)) {
		if (0 != MoveExAxisForLineScan(dExAxisPos, dMaxSpeed)) {
			return -1;
		}
	}
	GetRobotCtrl()->MoveByJob(tPulse, tPulseMove, GetRobotCtrl()->m_nExternalAxleType, JobName);
	return 0;
}

bool CUnit::WorldIsRunning()
{
	bool bRunning = false;
	if (ROBOT_BRAND_ESTUN == GetRobotCtrl()->m_eRobotBrand)
	{
		bRunning = GetRobotCtrl()->WorldIsRunning();
		return bRunning;
	}
	bRunning |= CheckInIO("Running");
	for (int i = 0; i < GetRobotCtrl()->m_pvpMotorDriver->size(); i++)
	{
		bRunning |= GetRobotCtrl()->m_pvpMotorDriver->at(i)->CheckAxisRun();
	}
	return bRunning;
}


void CUnit::WorldCheckRobotDone(int nDelayTime/* = 1200*/)
{
	if (ROBOT_BRAND_ESTUN == GetRobotCtrl()->m_eRobotBrand)
	{
		GetRobotCtrl()->WorldCheckRobotDone(nDelayTime);
		return;
	}
	RobotCheckDone();
	int nExAxisType = GetRobotCtrl()->m_nExternalAxleType;
	if (nExAxisType > 0 && 
		((((nExAxisType >> 6) & 0x01) == 1) || (((nExAxisType >> 7) & 0x01) == 1) || (((nExAxisType >> 8) & 0x01) == 1)))
	{
		for (int i = 0; i < GetRobotCtrl()->m_pvpMotorDriver->size(); i++)
		{
			GetRobotCtrl()->m_pvpMotorDriver->at(i)->CheckAxisDone();
		}
	}
}
bool  CUnit::GetExAxisType(int nMoveXYZAxisNo)
{
	bool bIsMoveYasExAxis = false;
	CRobotDriverAdaptor* pRobotDriver = GetRobotCtrl();
	int nExternalAxleBrand = pRobotDriver->m_nExternalAxleType;

	if (1 == (nExternalAxleBrand & 0x01) && 1 == nMoveXYZAxisNo) {
		if (1 == (nExternalAxleBrand >> 3 & 0x01)) // 安川
		{
			bIsMoveYasExAxis = true;
		}
	}
	else if (1 == (nExternalAxleBrand >> 1 & 0x01) && 2 == nMoveXYZAxisNo) {
		if (1 == (nExternalAxleBrand >> 4 & 0x01)) // 安川
		{
			bIsMoveYasExAxis = true;
		}
	}
	else if (1 == (nExternalAxleBrand >> 2 & 0x01) && 3 == nMoveXYZAxisNo) {
		if (1 == (nExternalAxleBrand >> 5 & 0x01)) // 安川
		{
			bIsMoveYasExAxis = true;
		}
	}
	return bIsMoveYasExAxis;
}

double CUnit::GetExPositionDis(int nMoveXYZAxisNo)
{
	double dCurPos = 0.0;
	if (ROBOT_BRAND_ESTUN == GetRobotCtrl()->m_eRobotBrand)
	{
		dCurPos = GetRobotCtrl()->GetExPositionDis(nMoveXYZAxisNo);
		return dCurPos;
	}
	CRobotDriverAdaptor* pRobotDriver = GetRobotCtrl();
	if (!GetExAxisType(nMoveXYZAxisNo))
	{
		CUnitDriver* pUnitDriver;
		int nAxisDir = -1;
		nAxisDir = 1 == labs(nMoveXYZAxisNo) ? 1 : nAxisDir;
		nAxisDir = 2 == labs(nMoveXYZAxisNo) ? 2 : nAxisDir;
		nAxisDir = 3 == labs(nMoveXYZAxisNo) ? 3 : nAxisDir;

		for (int nAxis = 0; nAxis < pRobotDriver->m_pvpMotorDriver->size(); nAxis++) 
		{
			pUnitDriver = pRobotDriver->m_pvpMotorDriver->at(nAxis);
			if (nAxisDir != labs(pUnitDriver->GetMotorParam().nToRobotDir))
			{
				if (nAxis == pRobotDriver->m_pvpMotorDriver->size() - 1) 
		{
			//XiMessageBoxOk("外部轴配置错误! 运动失败");
			//已修改
			XUI::MesBox::PopInfo("外部轴配置错误!运动失败");

			return -1;
		}
			}
			else
			{
		pUnitDriver->GetCurrentPosition(dCurPos);
				break;
			}
		}
	}
	else
	{
		T_ROBOT_COORS tRobotCoors = pRobotDriver->GetCurrentPos();
		if (1 == nMoveXYZAxisNo) {
			dCurPos = tRobotCoors.dBX;
		}
		else if (2 == nMoveXYZAxisNo) {
			dCurPos = tRobotCoors.dBY;
		}
		else if (3 == nMoveXYZAxisNo) {
			dCurPos = tRobotCoors.dBZ;
		}
		else
		{
			//已修改
			//XiMessageBoxOk("外部轴未就绪，请检查！");
			XUI::MesBox::PopInfo("外部轴未就绪，请检查！");

			return -1;
		}
	}
	return dCurPos;
}

double CUnit::GetExPulseEquivalent(int nMoveXYZAxisNo)
{
	double PulseEquivalent = 0.0;
	CRobotDriverAdaptor* pRobotDriver = GetRobotCtrl();
	if (!GetExAxisType(nMoveXYZAxisNo))
	{
		CUnitDriver* pUnitDriver;
		int nAxisDir = -1;
		nAxisDir = 1 == labs(nMoveXYZAxisNo) ? 1 : nAxisDir;
		nAxisDir = 2 == labs(nMoveXYZAxisNo) ? 2 : nAxisDir;
		nAxisDir = 3 == labs(nMoveXYZAxisNo) ? 3 : nAxisDir;

		for (int nAxis = 0; nAxis < pRobotDriver->m_pvpMotorDriver->size(); nAxis++)
		{
			pUnitDriver = pRobotDriver->m_pvpMotorDriver->at(nAxis);
			if (nAxisDir != labs(pUnitDriver->GetMotorParam().nToRobotDir))
			{
				if (nAxis == pRobotDriver->m_pvpMotorDriver->size() - 1)
				{
					//已修改
					XUI::MesBox::PopInfo("外部轴配置错误!运动失败");
					//XiMessageBoxOk("外部轴配置错误! 运动失败");
					return -1;
				}
			}
			else
			{
				PulseEquivalent = pUnitDriver->GetPulseEquivalent();
				break;
			}
		}
	}
	else
	{
		if (1 == nMoveXYZAxisNo) {
			PulseEquivalent = pRobotDriver->m_tExternalAxle[0].dPulse;
		}
		else if (2 == nMoveXYZAxisNo) {
			PulseEquivalent = pRobotDriver->m_tExternalAxle[1].dPulse;
		}
		else if (3 == nMoveXYZAxisNo) {
			PulseEquivalent = pRobotDriver->m_tExternalAxle[2].dPulse;
		}
		else
		{
			//已修改
			XUI::MesBox::PopInfo("外部轴未就绪，请检查！");
			//XiMessageBoxOk("外部轴未就绪，请检查！");
			return -1;
		}
	}
	return PulseEquivalent;
}

int CUnit::MoveExAxisFun(double dDist, int nSpeed, int nMoveXYZAxisNo, double dMaxExSpeed/* = -1*/)
{
	if (ROBOT_BRAND_ESTUN == GetRobotCtrl()->m_eRobotBrand)
	{
		GetRobotCtrl()->MoveExAxisFun(dDist, nSpeed, nMoveXYZAxisNo, dMaxExSpeed);
		return 0;
	}
	if (dMaxExSpeed < 0.0)
	{
		dMaxExSpeed = nSpeed;
	}
	if (nMoveXYZAxisNo > 3 || nMoveXYZAxisNo < 1)
	{
		//已修改
		XUI::MesBox::PopInfo("外部轴配置错误!运动失败");
		//XiMessageBoxOk("外部轴配置错误! 运动失败");
		return -1;
	}
	CRobotDriverAdaptor* pRobotDriver = GetRobotCtrl();
	double dMaxExAxlePos = (double)pRobotDriver->m_tExternalAxle[nMoveXYZAxisNo - 1].lMaxPulseNum * pRobotDriver->m_tExternalAxle[nMoveXYZAxisNo - 1].dPulse;
	double dMinExAxlePos = (double)pRobotDriver->m_tExternalAxle[nMoveXYZAxisNo - 1].lMinPulseNum * pRobotDriver->m_tExternalAxle[nMoveXYZAxisNo - 1].dPulse;
	if (dDist > dMaxExAxlePos || dDist < dMinExAxlePos)
	{
		//已修改
		XUI::MesBox::PopInfo("外部轴{0}目标位置{1:.3f}超出设定极限{2:.3f}-{3:.3f}!", nMoveXYZAxisNo, dDist, dMinExAxlePos, dMaxExAxlePos);
		//XiMessageBoxOk("外部轴%d 目标位置%.3lf 超出设定极限%.3lf - %.3lf!", nMoveXYZAxisNo, dDist, dMinExAxlePos, dMaxExAxlePos);
		return -1;
	}
	if (!GetExAxisType(nMoveXYZAxisNo))
	{
		CUnitDriver* pUnitDriver;
		//int nAxisNo = nMoveAxisNo; // 需要通过XYZ三个方向获取对应的软件轴号
		int nAxisDir = -1;
		nAxisDir = 1 == labs(nMoveXYZAxisNo) ? 1 : nAxisDir;
		nAxisDir = 2 == labs(nMoveXYZAxisNo) ? 2 : nAxisDir;
		nAxisDir = 3 == labs(nMoveXYZAxisNo) ? 3 : nAxisDir;



		for (int nAxis = 0; nAxis < pRobotDriver->m_pvpMotorDriver->size(); nAxis++)
		{
			pUnitDriver = pRobotDriver->m_pvpMotorDriver->at(nAxis);
			if (nAxisDir != labs(pUnitDriver->GetMotorParam().nToRobotDir))
			{
				if (nAxis == pRobotDriver->m_pvpMotorDriver->size() - 1)
				{
					//已修改
					XUI::MesBox::PopInfo("外部轴配置错误!运动失败");
					//XiMessageBoxOk("外部轴配置错误! 运动失败");
					return -1;
				}
			}
			else
			{
				break;
			}


			bool bServoRdy = false;
			pUnitDriver->GetSevonRdy(bServoRdy);
			if (!bServoRdy)
			{
				//已修改
				XUI::MesBox::PopInfo("外部轴未就绪，请检查！");
				//XiMessageBoxOk("外部轴未就绪，请检查！");
				return -1;
			}
		}
		pUnitDriver->PosMove(dDist, 1, dMaxExSpeed / 60.0, 0.5, 0.5);
		return 0;
	}
	else
	{
		T_ROBOT_COORS tRobotCoors = pRobotDriver->GetCurrentPos();

		if (1 == nMoveXYZAxisNo) {
			tRobotCoors.dBX = dDist;
		}
		else if (2 == nMoveXYZAxisNo) {
			tRobotCoors.dBY = dDist;
		}
		else if (3 == nMoveXYZAxisNo) {
			tRobotCoors.dBZ = dDist;
		}
		else
		{
			//已修改
			XUI::MesBox::PopInfo("外部轴未就绪，请检查！");
			//XiMessageBoxOk("外部轴未就绪，请检查！");
			return -1;
		}
		T_ROBOT_MOVE_SPEED tPulseMove(nSpeed / 6, 20, 20);
		pRobotDriver->MoveByJob(tRobotCoors, tPulseMove, GetRobotCtrl()->m_nExternalAxleType, "MOVL");
		return 0;
	}
	return 0;
}

int CUnit::XYZAxisNoToSoftAxisNo(int nMoveXYZAxisNo)
{
	if (ROBOT_BRAND_ESTUN == GetRobotCtrl()->m_eRobotBrand)
	{
		int nSoftAxisNo = GetRobotCtrl()->XYZAxisNoToSoftAxisNo(nMoveXYZAxisNo);
		return nSoftAxisNo;
	}
	int nSoftAxisNo = -1;
	for (int nAxis = 0; nAxis < GetRobotCtrl()->m_pvpMotorDriver->size(); nAxis++)
	{
		CUnitDriver* pUnitDriver = GetRobotCtrl()->m_pvpMotorDriver->at(nAxis);
		if (nMoveXYZAxisNo != labs(pUnitDriver->GetMotorParam().nToRobotDir))
		{
			if (nAxis == GetRobotCtrl()->m_pvpMotorDriver->size() - 1)
			{
				//已修改
				XUI::MesBox::PopInfo("外部轴配置错误!运动失败");
				//XiMessageBoxOk("外部轴配置错误! 运动失败");
				return -1;
			}
		}
		else
		{
			nSoftAxisNo = pUnitDriver->GetMotorParam().nSoftAxisNo;
			break;
		}
	}
	return nSoftAxisNo;
}

int CUnit::MoveExAxisForLineScan(double dDist, int nSpeed, bool bIsMoveYasExAxis)
{
	if (ROBOT_BRAND_ESTUN == GetRobotCtrl()->m_eRobotBrand)
	{
		GetRobotCtrl()->MoveExAxisForLineScan(dDist, nSpeed);
	}
	int nDriverNo = 0;
	CRobotDriverAdaptor* pRobotDriver = GetRobotCtrl();
	int nExternalAxleBrand = pRobotDriver->m_nExternalAxleType;
	if (0 != nExternalAxleBrand&& !bIsMoveYasExAxis)
	{
		CUnitDriver* pUnitDriver = pRobotDriver->m_pvpMotorDriver->at(nDriverNo);
		int nToRobotDir = pUnitDriver->GetMotorParam().nToRobotDir;
		int nAxisNo = -1;
		nAxisNo = (labs(nToRobotDir) == 1) ? 0 : nAxisNo; // X轴
		nAxisNo = (labs(nToRobotDir) == 2) ? 1 : nAxisNo; // Y轴
		nAxisNo = (labs(nToRobotDir) == 3) ? 2 : nAxisNo; // Z轴
		if (nAxisNo > 2 || nAxisNo < 0)
		{
			//已修改
			XUI::MesBox::PopInfo("外部轴配置错误!运动失败");
			//XiMessageBoxOk("外部轴配置错误! 运动失败");
			return -1;
		}
		double dMaxExAxlePos = (double)pRobotDriver->m_tExternalAxle[nAxisNo].lMaxPulseNum * pRobotDriver->m_tExternalAxle[nAxisNo].dPulse;
		double dMinExAxlePos = (double)pRobotDriver->m_tExternalAxle[nAxisNo].lMinPulseNum * pRobotDriver->m_tExternalAxle[nAxisNo].dPulse;
		if (dDist > dMaxExAxlePos || dDist < dMinExAxlePos)
		{
			//已修改
			XUI::MesBox::PopInfo("外部轴{0}目标位置{1:.3f}超出设定极限{2:.3f}-{3:.3f}!", nAxisNo, dDist, dMinExAxlePos, dMaxExAxlePos);
			//XiMessageBoxOk("外部轴%d 目标位置%.3lf 超出设定极限%.3lf - %.3lf!", nAxisNo, dDist, dMinExAxlePos, dMaxExAxlePos);
			return -1;
		}
		bool bServoRdy = false;
		pUnitDriver->GetSevonRdy(bServoRdy);
		if (!bServoRdy)
		{
			//已修改
			XUI::MesBox::PopInfo("外部轴未就绪，请检查！");
			//XiMessageBoxOk("外部轴未就绪，请检查！");
			return -1;
		}

		pUnitDriver->PosMove(dDist, 1, (double)nSpeed / 60.0, 0.5, 0.5); 

		return 0;
	}
	else
	{
		T_ROBOT_COORS tRobotCoors = pRobotDriver->GetCurrentPos();
#ifdef SINGLE_ROBOT
		tRobotCoors.dBY = dDist;
#else
		tRobotCoors.dBX = dDist;
#endif // SINGLE_ROBOT
		T_ROBOT_MOVE_SPEED tPulseMove(nSpeed / 6, 20, 20);
		pRobotDriver->MoveByJob(tRobotCoors, tPulseMove, GetRobotCtrl()->m_nExternalAxleType, "MOVL");
		return 0;
	}
	return 0;
}

bool CUnit::CheckIsReadyRun()
{
	if (g_bLocalDebugMark) return true;

	if (ROBOT_BRAND_ESTUN == GetRobotCtrl()->m_eRobotBrand)
	{
		bool bServo = GetRobotCtrl()->CheckIsReadyRun();
		return bServo;
	}

	bool bServo = CheckInIO("Servo");
	if (true == bServo) {
		return true;
	}
	if (true == CheckInIO("RobotEmg")) {
		//已修改
		XUI::MesBox::PopInfo("请将[{0}]机器人急停复位后操作", GetRobotCtrl()->m_strCustomName.GetBuffer());
		//SwitchIO("alarmlight", true);
		//XiMessageBoxOk("请将[%s]机器人急停复位后操作", GetRobotCtrl()->m_strCustomName);
		return false;
	}
	if (true == CheckInIO("TotalEmg")) {
		//已修改
		XUI::MesBox::PopInfo("请将[{0}]总急停复位后操作", GetRobotCtrl()->m_strCustomName.GetBuffer());
		//SwitchIO("alarmlight", true);
		//XiMessageBoxOk("请将[%s]总急停复位后操作", GetRobotCtrl()->m_strCustomName);
		return false;
	}
	Sleep(50);
	GetRobotCtrl()->ServoOn();
	Sleep(50);

	for (int i = 0; i < 30; i++) {
		if (CheckInIO("Servo")) {
			break;
		}
		Sleep(100);
	}
	bServo = CheckInIO("Servo");
	if (false == bServo) {
		//已修改
		//SwitchIO("alarmlight", true);
		XUI::MesBox::PopInfo("{0}机器人上电失败，可能是示教器模式错误!", GetRobotCtrl()->m_strCustomName.GetBuffer());
		//XiMessageBoxOk("[%s]机器人上电失败，可能是示教器模式错误!", GetRobotCtrl()->m_strCustomName);
	}
	return bServo;
}
////（问题：未完成CheckInIO()定义：该函数依赖CheckInIO()  该函数已完成定义：注释依赖项）
//E_ERROR_STATE CUnit::CheckRobotDone(int nDelayTime)
//{
//	{
//		int nInitialTime = XI_clock();
//		//依赖项↓
//		//while (true != CheckInIO("Running")) {
//		//	int nTime = XI_clock() - nInitialTime;
//		//	if (nTime > nDelayTime) {
//		//		break;
//		//	}
//		//	Sleep(10);
//		//}
//		int i = 0;
//		//依赖项↓
//		//bool bRunning = CheckInIO("Running");
//		//while (true == bRunning) {
//		//	Sleep(10);
//		//	bRunning = CheckInIO("Running");
//		//	if (false == bRunning) {
//		//		i++;
//		//		bRunning = true;
//		//		if (i > 18)//18*10ms时刚好达到完全停住的状态（目测），为了稳定性，多延迟70ms
//		//		{
//		//			break;
//		//		}
//		//	}
//		//	else {
//		//		i = 0;
//		//	}
//		//}
//		return RUNNING_STATUS_SUCCESS;
//	}
//	
//}

bool CUnit::CheckRobotDone(T_ROBOT_COORS tAimCoor, CString sInfo/* = ""*/)
{
	if (g_bLocalDebugMark)
	{
		return true;
	}
	RobotCheckDone();
	T_ROBOT_COORS tCoor = GetRobotCtrl()->GetCurrentPos();
	bool bRst = GetRobotCtrl()->CompareCoords(tAimCoor, tCoor, 2.0, 1);
	if (false == bRst && sInfo != "")
	{
		XiMessageBoxOk(sInfo.GetBuffer());
	}
	return bRst;
}

bool CUnit::CheckRobotDone(T_ANGLE_PULSE tAimPulse, CString sInfo/* = ""*/)
{
	if (g_bLocalDebugMark)
	{
		return true;
	}
	RobotCheckDone();
	bool bRst = ComparePulse(tAimPulse, GetRobotCtrl()->GetCurrentPulse());
	if (false == bRst && sInfo != "")
	{
		XiMessageBoxOk(sInfo.GetBuffer());
	}
	return bRst;
}

//bool CUnit::MoveToSafeHeight()
//{
//	if (g_bLocalDebugMark) return true;
//	double dRobotInstallMode = GetRobotCtrl()->m_nRobotInstallDir;
//	double dMinUpMoveDis = 100.0;
//	double dBackHeightThreshold = 400.0;
//	T_ANGLE_PULSE tCurPulse;
//	T_ROBOT_COORS tCurCoord;
//	T_ROBOT_COORS tTransCoord;
//	T_ROBOT_COORS tTempCoord;
//	T_ROBOT_COORS tHomeCoord;
//	T_ROBOT_MOVE_INFO tRobotMoveInfo;
//	vector<T_ROBOT_MOVE_INFO> vtRobotMoveInfo;
//	vtRobotMoveInfo.clear();
//
//	tCurPulse = GetRobotCtrl()->GetCurrentPulse();
//	tCurCoord = GetRobotCtrl()->GetCurrentPos();
//	GetRobotCtrl()->RobotKinematics(GetRobotCtrl()->m_tHomePulse, GetRobotCtrl()->m_tTools.tGunTool, tHomeCoord);
//	GetRobotCtrl()->RobotKinematics(tCurPulse, GetRobotCtrl()->m_tTools.tGunTool, tTransCoord);
//	// 读取直角坐标和读取关节坐标转换出来结果不同时，不能运动
//	if (false == GetRobotCtrl()->CompareCoords(tTransCoord, tCurCoord)) {
//		WriteLog("MoveToSafeHeight:读取坐标和计算坐标误差过大");
//		XiMessageBoxOk("读取坐标和计算坐标不同，无法自动抬枪！");
//		return false;
//	}
//
//	double dMinBackGunHeight = tHomeCoord.dZ - (dBackHeightThreshold * dRobotInstallMode);
//	// 高度小于dMinBackGunHeight时 执行操作：1、RY恢复标准45  2、最少抬高100mm（MOVL距离太小速度非常快）
//	tTempCoord = tCurCoord;
//	tTempCoord.dRX = tHomeCoord.dRX;
//	tTempCoord.dRY = tHomeCoord.dRY;
//	if ((dRobotInstallMode > 0.0 && tCurCoord.dZ < dMinBackGunHeight) ||
//		(dRobotInstallMode < 0.0 && tCurCoord.dZ > dMinBackGunHeight)) {
//		if (fabs(tCurCoord.dZ - dMinBackGunHeight) < dMinUpMoveDis) {
//			tTempCoord.dZ += (dMinUpMoveDis * dRobotInstallMode);
//		}
//		else {
//			tTempCoord.dZ = dMinBackGunHeight;
//		}
//		tRobotMoveInfo = GetRobotCtrl()->PVarToRobotMoveInfo(0, tTempCoord, GetRobotCtrl()->m_tCoordHighSpeed, MOVL);
//		vtRobotMoveInfo.push_back(tRobotMoveInfo);
//	}
//	if (vtRobotMoveInfo.size() > 0) {
//		SetMoveValue(vtRobotMoveInfo);
//		GetRobotCtrl()->CallJob("CONTIMOVANY");
//		RobotCheckDone();
//		return GetRobotCtrl()->CompareCoords(tTempCoord);
//	}
//	else {
//		return true;
//	}
//}

bool CUnit::SendTeachMoveData(const std::vector<T_ANGLE_PULSE>& vtMeasurePulse, const std::vector<int>& vnMeasureType, int nDownSpeed, int nTeachSpeed, int nUpSpeed, int nTrigSigTime)
{
	// 发送测量轨迹数据（示教点数 轨迹坐标 速度 触发信号时间）
	// P010 - P069 示教位置
	// I010 - I069 点类型 1示教点 0过渡点
	// I001 总示教点数 包括收下枪过渡点和中间测量过渡点
	// I003 下枪运动速度 MOVJ
	// I004 示教运动速度 MOVL
	// I005 收枪运动速度 MOVL
	// I006 触发信号持续时间 单位ms
	CRobotDriverAdaptor* pRobotDriver = GetRobotCtrl();
	vector<MP_USR_VAR_INFO> vtVarInfo;
	vtVarInfo.clear();
	int nTotalPosNum = 0;

	for (int i = 0; i < vtMeasurePulse.size(); i++) {
		if (vnMeasureType[i] & E_TRANSITION_POINT &&
			vnMeasureType[i] & E_DOUBLE_LONG_LINE &&
			vnMeasureType[i] & E_LS_RL_FLIP &&
			vnMeasureType[i] & E_LL_RS_FLIP) {
			continue;
		}
		int nPtnType = E_TRANSITION_POINT == vnMeasureType[i] ? 0 : 1;
		vtVarInfo.push_back(pRobotDriver->PrepareValData(i + 10, vtMeasurePulse[i]));
		vtVarInfo.push_back(pRobotDriver->PrepareValData(i + 10, nPtnType));
		nTotalPosNum++;
	}

	vtVarInfo.push_back(pRobotDriver->PrepareValData(1, nTotalPosNum));
	vtVarInfo.push_back(pRobotDriver->PrepareValData(3, nDownSpeed));
	vtVarInfo.push_back(pRobotDriver->PrepareValData(4, nTeachSpeed));
	vtVarInfo.push_back(pRobotDriver->PrepareValData(5, nUpSpeed));
	vtVarInfo.push_back(pRobotDriver->PrepareValData(6, nTrigSigTime));

	// 变量高速混发到机器人
	pRobotDriver->SetMultiVar_H(vtVarInfo);
	return true;
}

bool CUnit::TeachMove(const std::vector<T_ANGLE_PULSE>& vtMeasurePulse, double dExAxlePos, const std::vector<int>& vnMeasureType, int nTrigSigTime)
{
	
	if (ROBOT_BRAND_ESTUN == GetRobotCtrl()->m_eRobotBrand)
	{
		GetRobotCtrl()->TeachMove(vtMeasurePulse, dExAxlePos, vnMeasureType, nTrigSigTime);
		return true;
	}
	CRobotDriverAdaptor* pRobotDriver = GetRobotCtrl();
	int nAxisNo = m_nMeasureAxisNo;
	if (0 != MoveExAxisFun(dExAxlePos, 9000, nAxisNo))return false;
	WorldCheckRobotDone();
	// 需要更改获取轴数据，临时更改
	double dCurExPos = GetExPositionDis(nAxisNo);// GetRobotCtrl()->GetCurrentPos(ROBOT_AXIS_BPY);
	if (fabs(dExAxlePos - dCurExPos) > 5.0) // 与目标位置相差超过阈值 判断为运动失败
	{
		//已修改
		XUI::MesBox::PopInfo("自动示教:外部轴未运动到指定位置");
		//XiMessageBox("自动示教:外部轴未运动到指定位置");
		return false;
	}

	T_ROBOT_MOVE_INFO tRobotMoveInfo;
	vector<T_ROBOT_MOVE_INFO> vtRobotMoveInfo(0);
	// 下枪运动到扫描起点位置 MOVJ MOVJ
	tRobotMoveInfo = pRobotDriver->PVarToRobotMoveInfo(0, vtMeasurePulse[0], pRobotDriver->m_tPulseHighSpeed, MOVJ);
	vtRobotMoveInfo.push_back(tRobotMoveInfo);
	pRobotDriver->SetMoveValue(vtRobotMoveInfo);
	pRobotDriver->CallJob("CONTIMOVANY");
	CheckRobotDone(vtMeasurePulse[0]);
	SendTeachMoveData(vtMeasurePulse, vnMeasureType, 
		pRobotDriver->m_tPulseHighSpeed.dSpeed, pRobotDriver->m_tTeachSpeed.dSpeed, pRobotDriver->m_tPulseLowSpeed.dSpeed, nTrigSigTime);
	pRobotDriver->CallJob("TEACH-J");
	return true;
}

int CUnit::WeldMove_Scan(const std::vector<T_ROBOT_COORS>& vtWeldPathPoints, const std::vector<int>& vnPtnType, const T_WELD_PARA& tWeldPara, double dExAxlePos, E_WELD_SEAM_TYPE eWeldSeamType, bool bIsArcOn, int ToolNum)
{
	if (ROBOT_BRAND_ESTUN == GetRobotCtrl()->m_eRobotBrand)
	{
		GetRobotCtrl()->WeldMove(vtWeldPathPoints, vnPtnType, tWeldPara, dExAxlePos, eWeldSeamType, bIsArcOn);
		return 0;
	}

	CRobotDriverAdaptor* pRobotDriver = GetRobotCtrl();
	int nCursor = 0; // 记录分段查找获取轨迹索引
	vector<T_ROBOT_COORS> vtWeldCoord; // 保存分段轨迹
	int nPtnType; // vtWeldCoord的轨迹点类型
	bool bIsCallJob = false;

	//// 设置工艺参数 是否起弧：I058 起弧包角焊接停弧电流电压：I080-I087 停弧停留时间:I60  复位I071 I072
	CHECK_BOOL_RETURN(SendWeldParam(bIsArcOn, tWeldPara));
	
	WriteLog("SendWeldParam Done!");

	//// 获取起弧轨迹 设置起弧Job数据
	vtWeldCoord.clear();
	while (true) {
		if (E_WELD_ARC_ON & vnPtnType[nCursor]) {
			vtWeldCoord.push_back(vtWeldPathPoints[nCursor]);
			nCursor++;
		}
		else {
			break;
		}
		if (vtWeldCoord.size() > 7) // 起弧轨迹最多点数为7
		{
			//已修改	
			XUI::MesBox::PopInfo("焊接起弧轨迹点数过多！");
			//XiMessageBox("焊接起弧轨迹点数过多！");
			return false;
		}
	}
	CHECK_BOOL_RETURN(SendArcOnData(vtWeldCoord, tWeldPara, ToolNum));
	WriteLog("SendArcOnData Done!");

	// 循环获取下一段轨迹（包角？ 焊接？）发送包角或焊接轨迹 (焊接轨迹需要循环发送)
	int nTrackSectionNo = 0;
	while (true == GetNextSectionTrack(vtWeldPathPoints, vnPtnType, nCursor, vtWeldCoord, nPtnType)) // 获取到轨迹
	{	// I070 0表示在GENERALWELD JOB内 
		//		1表示在ARCON	   JOB内
		//		2表示在WRAP		   JOB内
		//		3表示在WELD		   JOB内
		//		4表示在ARCOFF	   JOB内
		INT _flag = pRobotDriver->m_pYaskawaRobotCtrl->GetIntVar(70);
		WriteLog("获取下一段轨迹成功第%d段", ++nTrackSectionNo);
		if (E_WELD_WRAP & nPtnType) {
			//未发送完一个包角轨迹时禁止发送新的包角轨迹
			while (_flag == 2) {
				Sleep(10);
				_flag = pRobotDriver->m_pYaskawaRobotCtrl->GetIntVar(70);
			}
			SendWrapData(vtWeldCoord, tWeldPara/*, tWeldInfo*/);
			WriteLog("SendWrapData Done!");
		}
		else if (E_WELD_TRACK & nPtnType) {
			//未发送完一个焊接轨迹时禁止发送新的焊接轨迹
			while (_flag == 3) {
				Sleep(10);
				_flag = pRobotDriver->m_pYaskawaRobotCtrl->GetIntVar(70);
			}
			SendWeldData(vtWeldCoord, tWeldPara, eWeldSeamType, bIsCallJob, ToolNum); // 内部调用且只调用一次CallJob
			WriteLog("SendWeldData Done!");
		}
	}
	return 0;
}

int CUnit::WeldMove(const std::vector<T_ROBOT_COORS>& vtWeldPathPoints, const std::vector<int>& vnPtnType, const T_WELD_PARA& tWeldPara, double dExAxlePos, E_WELD_SEAM_TYPE eWeldSeamType, bool bIsArcOn, int ToolNum)
{
	//E_DHGIGE_ACQUISITION_MODE eCameraMode = E_ACQUISITION_MODE_SOURCE_SOFTWARE;
	//E_DHGIGE_CALL_BACK eCallBack = E_CALL_BACK_MODE_OFF;
	//SwitchDHCamera(m_nTrackCameraNo, true, true, eCameraMode, eCallBack); // 下枪时 开相机 激光
	//m_vpImageCapture[m_nTrackCameraNo]->StartAcquisition();
	if (ROBOT_BRAND_ESTUN == GetRobotCtrl()->m_eRobotBrand)
	{
		GetRobotCtrl()->WeldMove(vtWeldPathPoints, vnPtnType, tWeldPara, dExAxlePos, eWeldSeamType, bIsArcOn);
		return 0;
	}

	CRobotDriverAdaptor *pRobotDriver = GetRobotCtrl();
	int nCursor = 0; // 记录分段查找获取轨迹索引
	vector<T_ROBOT_COORS> vtWeldCoord; // 保存分段轨迹
	int nPtnType; // vtWeldCoord的轨迹点类型
	bool bIsCallJob = false;

	//// 设置工艺参数 是否起弧：I058 起弧包角焊接停弧电流电压：I080-I087 停弧停留时间:I60  复位I071 I072
	CHECK_BOOL_RETURN(SendWeldParam(bIsArcOn, tWeldPara));
	WriteLog("SendWeldParam Done!");

	//// 获取起弧轨迹 设置起弧Job数据
	vtWeldCoord.clear();
	while (true) {
		if (E_WELD_ARC_ON & vnPtnType[nCursor]) {
			vtWeldCoord.push_back(vtWeldPathPoints[nCursor]);
			nCursor++;
		}
		else {
			break;
		}
		if (vtWeldCoord.size() > 7) // 起弧轨迹最多点数为7
		{
			//已修改
			XUI::MesBox::PopInfo("焊接起弧轨迹点数过多！");
			//XiMessageBox("焊接起弧轨迹点数过多！");
			return false;
		}
	}
	CHECK_BOOL_RETURN(SendArcOnData(vtWeldCoord, tWeldPara, ToolNum));
	WriteLog("SendArcOnData Done!");

	// 循环获取下一段轨迹（包角？ 焊接？）发送包角或焊接轨迹 (焊接轨迹需要循环发送)
	int nTrackSectionNo = 0;
	while (true == GetNextSectionTrack(vtWeldPathPoints, vnPtnType, nCursor, vtWeldCoord, nPtnType)) // 获取到轨迹
	{	// I070 0表示在GENERALWELD JOB内 
		//		1表示在ARCON	   JOB内
		//		2表示在WRAP		   JOB内
		//		3表示在WELD		   JOB内
		//		4表示在ARCOFF	   JOB内
		INT _flag = pRobotDriver->m_pYaskawaRobotCtrl->GetIntVar(70);
		WriteLog("获取下一段轨迹成功第%d段", ++nTrackSectionNo);
		if (E_WELD_WRAP & nPtnType) {
			//未发送完一个包角轨迹时禁止发送新的包角轨迹
			while (_flag == 2) {
				Sleep(10);
				_flag = pRobotDriver->m_pYaskawaRobotCtrl->GetIntVar(70);
			}
			SendWrapData(vtWeldCoord, tWeldPara/*, tWeldInfo*/);
			WriteLog("SendWrapData Done!");
		}
		else if (E_WELD_TRACK & nPtnType) {
			//未发送完一个焊接轨迹时禁止发送新的焊接轨迹
			//未发送完一个焊接轨迹时禁止发送新的焊接轨迹

			while (_flag == 3) {
				Sleep(10);
				_flag = pRobotDriver->m_pYaskawaRobotCtrl->GetIntVar(70);
			}
			if (!m_ScanTrackingWeldEnable || eWeldSeamType == E_STAND_SEAM)
			{
				SendWeldData(vtWeldCoord, tWeldPara, eWeldSeamType, bIsCallJob, ToolNum); // 内部调用且只调用一次CallJob
			}
			else
			{
				//jwq 初始化焊接轨迹类
				m_cRealTimeTrack.m_cAdjustTrack.loadPara();
				if (m_bBreakPointContinue)
				{
					m_cRealTimeTrack.m_cAdjustTrack.setUnit(*this);
					m_cWeldTrack.loadTrack(m_cRealTimeTrack.m_cAdjustTrack.getDataFileRootFolder() + "AdjustTrack//LastestAdjustTrack.txt");
					int nFirstNo = m_cWeldTrack.getNearestCoordNoFast(vtWeldCoord[0], 20.0, 0);
					if (nFirstNo < 0)
					{
						//已修改
						XUI::MesBox::PopInfo("新轨迹和之前记录的轨迹偏差较大，无法断点续焊！");
						//XiMessageBox("新轨迹和之前记录的轨迹偏差较大，无法断点续焊！");
						return false;
					}
					m_cWeldTrack.erase(0, nFirstNo);
				}
				else
				{
					m_cWeldTrack.initTrack(vtWeldCoord);
				}

				//jwq 初始化新跟踪
				m_cRealTimeTrack.initTrack(m_cWeldTrack);

				//jwq 新跟踪锁定
				if (!LockForNewRTTA(NULL))
				{
					//已修改
					XUI::MesBox::PopInfo("初始化新跟踪锁定失败！");
					//XiMessageBox("初始化新跟踪锁定失败！");
					return -1;
				}

				//jwq 开始新跟踪
				if (!m_cRealTimeTrack.initBeforeRun())
				{
					//已修改
					XUI::MesBox::PopInfo("初始化新跟踪失败！");
					//XiMessageBox("初始化新跟踪失败！");
					return -1;
				}
				getMainDlg()->KillTimer(2);
				m_cRealTimeTrack.startRun();

				//SendWeldData(vtWeldCoord, tWeldPara, eWeldSeamType, bIsCallJob, ToolNum); // 内部调用且只调用一次CallJob
				auto file = fopen("ActualWeldTrack.txt", "w");
				SendWeldDataNew(file, tWeldPara, eWeldSeamType, bIsCallJob, ToolNum); // 内部调用且只调用一次CallJob
				fclose(file);
				//jwq 结束新跟踪
				m_cRealTimeTrack.endRun();
				m_cRealTimeTrack.waitUntilEnd();
				getMainDlg()->SetTimer(2, 200, NULL);
			}
			WriteLog("SendWeldData Done!");
		}
	}
	return 0;
}

int CUnit::WeldMove_BP(const std::vector<T_ROBOT_COORS>& vtWeldPathPoints, const std::vector<int>& vnPtnType, const T_WELD_PARA& tWeldPara, E_WELD_SEAM_TYPE eWeldSeamType, bool bIsArcOn)
{
	double dExAxlePos = 0.0;
	if (ROBOT_BRAND_ESTUN == GetRobotCtrl()->m_eRobotBrand)
	{
		GetRobotCtrl()->WeldMove(vtWeldPathPoints, vnPtnType, tWeldPara, dExAxlePos, eWeldSeamType, bIsArcOn);
		return 0;
	}

	CRobotDriverAdaptor* pRobotDriver = GetRobotCtrl();
	int nCursor = 0; // 记录分段查找获取轨迹索引
	vector<T_ROBOT_COORS> vtWeldCoord; // 保存分段轨迹
	int nPtnType; // vtWeldCoord的轨迹点类型
	bool bIsCallJob = false;

	//// 设置工艺参数 是否起弧：I058 起弧包角焊接停弧电流电压：I080-I087 停弧停留时间:I60  复位I071 I072
	CHECK_BOOL_RETURN(SendWeldParam(bIsArcOn, tWeldPara));
	GetRobotCtrl()->m_cLog->Write("SendWeldParam Done!");

	//// 获取起弧轨迹 设置起弧Job数据
	vtWeldCoord.clear();
	while (true) {
		if (E_WELD_ARC_ON & vnPtnType[nCursor]) {
			vtWeldCoord.push_back(vtWeldPathPoints[nCursor]);
			nCursor++;
		}
		else {
			break;
		}
		if (vtWeldCoord.size() > 7) // 起弧轨迹最多点数为7
		{
			//已修改
			XUI::MesBox::PopInfo("焊接起弧轨迹点数过多！");
			//XiMessageBox("焊接起弧轨迹点数过多！");
			return false;
		}
	}
	CHECK_BOOL_RETURN(SendArcOnData(vtWeldCoord, tWeldPara));
	GetRobotCtrl()->m_cLog->Write("SendArcOnData Done!");

	// 循环获取下一段轨迹（包角？ 焊接？）发送包角或焊接轨迹 (焊接轨迹需要循环发送)
	int nTrackSectionNo = 0;
	while (true == GetNextSectionTrack(vtWeldPathPoints, vnPtnType, nCursor, vtWeldCoord, nPtnType)) // 获取到轨迹
	{	// I070 0表示在GENERALWELD JOB内 
		//		1表示在ARCON	   JOB内
		//		2表示在WRAP		   JOB内
		//		3表示在WELD		   JOB内
		//		4表示在ARCOFF	   JOB内
		INT _flag = pRobotDriver->m_pYaskawaRobotCtrl->GetIntVar(70);
		GetRobotCtrl()->m_cLog->Write("获取下一段轨迹成功第%d段", ++nTrackSectionNo);
		if (E_WELD_WRAP & nPtnType) {
			//未发送完一个包角轨迹时禁止发送新的包角轨迹
			while (_flag == 2) {
				Sleep(10);
				_flag = pRobotDriver->m_pYaskawaRobotCtrl->GetIntVar(70);
			}
			SendWrapData(vtWeldCoord, tWeldPara/*, tWeldInfo*/);
			GetRobotCtrl()->m_cLog->Write("SendWrapData Done!");
		}
		else if (E_WELD_TRACK & nPtnType) {
			//未发送完一个焊接轨迹时禁止发送新的焊接轨迹
			while (_flag == 3) {
				Sleep(10);
				_flag = pRobotDriver->m_pYaskawaRobotCtrl->GetIntVar(70);
			}
			SendWeldData_BP(vtWeldCoord, tWeldPara, eWeldSeamType, bIsCallJob); // 内部调用且只调用一次CallJob
			GetRobotCtrl()->m_cLog->Write("SendWeldData Done!");
		}
	}
	return 0;
}

bool CUnit::GetNextSectionTrack(const std::vector<T_ROBOT_COORS>& vtSrcCoord, const std::vector<int>& vnPtnType, int& nCursor, std::vector<T_ROBOT_COORS>& vtWeldCoord, int& nPtnType)
{
	vtWeldCoord.clear();
	if (nCursor >= vtSrcCoord.size()) {
		return false;
	}
	nPtnType = vnPtnType[nCursor];
	while ((vnPtnType.size() > nCursor) && (nPtnType == vnPtnType[nCursor])) {
		vtWeldCoord.push_back(vtSrcCoord[nCursor]);
		nCursor++;
	}
	return true;
}

bool CUnit::SendWeldParam(BOOL bIsArcOn, const T_WELD_PARA& tWeldParam)
{
	if (ROBOT_BRAND_YASKAWA != GetRobotCtrl()->m_eRobotBrand)
	{
		return true;
	}
	// 发送设置工艺参数 是否起弧：I058 起弧包角焊接停弧电流电压：I080-I087 停弧停留时间:I70
	// I1:变量复位 I74: 1脉冲分别 0直流分别
	CRobotDriverAdaptor *pRobotDriver = GetRobotCtrl();
	MP_USR_VAR_INFO tUsrVarInfo;
	vector<MP_USR_VAR_INFO> vtUsrVarInfo;
	vtUsrVarInfo.clear();
	tUsrVarInfo = pRobotDriver->PrepareValData(58, (TRUE == bIsArcOn ? 1 : 0));
	vtUsrVarInfo.push_back(tUsrVarInfo);
	tUsrVarInfo = pRobotDriver->PrepareValData(80, (int)tWeldParam.dStartArcCurrent);
	vtUsrVarInfo.push_back(tUsrVarInfo);
	tUsrVarInfo = pRobotDriver->PrepareValData(81, (int)tWeldParam.dStartArcVoltage * 10);


	vtUsrVarInfo.push_back(tUsrVarInfo);
	tUsrVarInfo = pRobotDriver->PrepareValData(82, (int)tWeldParam.dWrapCurrentt1);
	vtUsrVarInfo.push_back(tUsrVarInfo);
	tUsrVarInfo = pRobotDriver->PrepareValData(83, (int)tWeldParam.dWrapVoltage1 * 10);
	vtUsrVarInfo.push_back(tUsrVarInfo);
	tUsrVarInfo = pRobotDriver->PrepareValData(62, (int)(tWeldParam.dWrapWaitTime1 * 100.0)); // Timer 单位换算
	vtUsrVarInfo.push_back(tUsrVarInfo);

	//终点包角参数
	tUsrVarInfo = pRobotDriver->PrepareValData(75, (int)tWeldParam.dWrapCurrentt2);
	vtUsrVarInfo.push_back(tUsrVarInfo);
	tUsrVarInfo = pRobotDriver->PrepareValData(76, (int)tWeldParam.dWrapVoltage2 * 10);
	vtUsrVarInfo.push_back(tUsrVarInfo);
	tUsrVarInfo = pRobotDriver->PrepareValData(77, (int)tWeldParam.dWrapWaitTime2 * 100.0);
	vtUsrVarInfo.push_back(tUsrVarInfo);

	
	tUsrVarInfo = pRobotDriver->PrepareValData(84, (int)tWeldParam.dTrackCurrent);
	vtUsrVarInfo.push_back(tUsrVarInfo);
	tUsrVarInfo = pRobotDriver->PrepareValData(85, (int)tWeldParam.dTrackVoltage * 10);
	vtUsrVarInfo.push_back(tUsrVarInfo);
	tUsrVarInfo = pRobotDriver->PrepareValData(86, (int)tWeldParam.dStopArcCurrent);
	vtUsrVarInfo.push_back(tUsrVarInfo);
	tUsrVarInfo = pRobotDriver->PrepareValData(87, (int)tWeldParam.dStopArcVoltage * 10);
	vtUsrVarInfo.push_back(tUsrVarInfo);

	tUsrVarInfo = pRobotDriver->PrepareValData(61, (int)(tWeldParam.dStartWaitTime * 100.0)); // Timer 单位换算
	vtUsrVarInfo.push_back(tUsrVarInfo);
	tUsrVarInfo = pRobotDriver->PrepareValData(60, (int)(tWeldParam.dStopWaitTime * 100.0)); // Timer 单位换算
	vtUsrVarInfo.push_back(tUsrVarInfo);
	
	tUsrVarInfo = pRobotDriver->PrepareValData(88, (int)36); // 江南包角段临时测试速度
	vtUsrVarInfo.push_back(tUsrVarInfo);


	tUsrVarInfo = pRobotDriver->PrepareValData(70, 0);
	vtUsrVarInfo.push_back(tUsrVarInfo);
	tUsrVarInfo = pRobotDriver->PrepareValData(71, 0);
	vtUsrVarInfo.push_back(tUsrVarInfo);
	tUsrVarInfo = pRobotDriver->PrepareValData(72, 0);
	vtUsrVarInfo.push_back(tUsrVarInfo);
	tUsrVarInfo = pRobotDriver->PrepareValData(1, 0); // 复位
	vtUsrVarInfo.push_back(tUsrVarInfo);
	tUsrVarInfo = pRobotDriver->PrepareValData(74, tWeldParam.nWeldMethod); // 复位
	vtUsrVarInfo.push_back(tUsrVarInfo);

	pRobotDriver->SetMultiVar_H(vtUsrVarInfo); // 发送
	return true;
}

bool CUnit::SendGrooveParam(BOOL bIsArcOn, const T_WELD_PARA& tWeldParam)
{
	// 发送设置工艺参数 是否起弧：I058 起弧包角焊接停弧电流电压：I080-I087 停弧停留时间:I70
		// I1:变量复位 I74: 1脉冲分别 0直流分别
	CRobotDriverAdaptor* pRobotDriver = GetRobotCtrl();
	MP_USR_VAR_INFO tUsrVarInfo;
	vector<MP_USR_VAR_INFO> vtUsrVarInfo;
	vtUsrVarInfo.clear();
	tUsrVarInfo = pRobotDriver->PrepareValData(58, (TRUE == bIsArcOn ? 1 : 0));
	vtUsrVarInfo.push_back(tUsrVarInfo);
	tUsrVarInfo = pRobotDriver->PrepareValData(80, (int)tWeldParam.dStartArcCurrent);
	vtUsrVarInfo.push_back(tUsrVarInfo);
	tUsrVarInfo = pRobotDriver->PrepareValData(81, (int)tWeldParam.dStartArcVoltage * 10);
	vtUsrVarInfo.push_back(tUsrVarInfo);
	tUsrVarInfo = pRobotDriver->PrepareValData(82, (int)tWeldParam.dWrapCurrentt1);
	vtUsrVarInfo.push_back(tUsrVarInfo);
	tUsrVarInfo = pRobotDriver->PrepareValData(83, (int)tWeldParam.dWrapVoltage1 * 10);
	vtUsrVarInfo.push_back(tUsrVarInfo);
	tUsrVarInfo = pRobotDriver->PrepareValData(84, (int)tWeldParam.dTrackCurrent);
	vtUsrVarInfo.push_back(tUsrVarInfo);
	tUsrVarInfo = pRobotDriver->PrepareValData(85, (int)tWeldParam.dTrackVoltage * 10);
	vtUsrVarInfo.push_back(tUsrVarInfo);
	tUsrVarInfo = pRobotDriver->PrepareValData(86, (int)tWeldParam.dStopArcCurrent);
	vtUsrVarInfo.push_back(tUsrVarInfo);
	tUsrVarInfo = pRobotDriver->PrepareValData(87, (int)tWeldParam.dStopArcVoltage * 10);
	vtUsrVarInfo.push_back(tUsrVarInfo);
	tUsrVarInfo = pRobotDriver->PrepareValData(60, (int)(tWeldParam.dStopWaitTime * 100.0)); // Timer 单位换算
	vtUsrVarInfo.push_back(tUsrVarInfo);
	tUsrVarInfo = pRobotDriver->PrepareValData(70, 0);
	vtUsrVarInfo.push_back(tUsrVarInfo);
	tUsrVarInfo = pRobotDriver->PrepareValData(71, 0);
	vtUsrVarInfo.push_back(tUsrVarInfo);
	tUsrVarInfo = pRobotDriver->PrepareValData(72, 0);
	vtUsrVarInfo.push_back(tUsrVarInfo);
	tUsrVarInfo = pRobotDriver->PrepareValData(1, 0); // 复位
	vtUsrVarInfo.push_back(tUsrVarInfo);
	tUsrVarInfo = pRobotDriver->PrepareValData(74, tWeldParam.nWeldMethod); // 复位
	vtUsrVarInfo.push_back(tUsrVarInfo);
	tUsrVarInfo = pRobotDriver->PrepareValData(9, (int)(tWeldParam.dStartWaitTime * 100.0)); // 起弧停留时间
	vtUsrVarInfo.push_back(tUsrVarInfo);

	pRobotDriver->SetMultiVar_H(vtUsrVarInfo); // 发送
	return true;
}

int CUnit::GrooveWeldMove(const std::vector<T_ROBOT_COORS>& vtWeldPathPoints, const T_WELD_PARA& tWeldPara, double dExAxlePos, bool bIsArcOn, T_WAVE_PARA tWavePara, vector<double> weldSpeedRate, vector<double> vdWaveWaitTime)
{
	int nCursor = 0; // 记录分段查找获取轨迹索引
	bool bIsCallJob = false;
	//SendGrooveArcOnData(tWeldPara);
	SendGrooveParam(bIsArcOn, tWeldPara);
	if (false == SendGrooveWeldData(bIsArcOn, vtWeldPathPoints, tWeldPara, bIsCallJob, tWavePara, weldSpeedRate, vdWaveWaitTime))
	{
		return -1;
	}
	return 0;
}

bool CUnit::SendGrooveWeldData(BOOL bIsArcOn, const std::vector<T_ROBOT_COORS>& vtWeldCoord, const T_WELD_PARA& tWeldParam, bool& bIsCallJob, T_WAVE_PARA tWavePara, vector<double> weldSpeedRate, vector<double> vdWaveWaitTime)
{
	CRobotDriverAdaptor* pRobotDriver = GetRobotCtrl();
	MP_USR_VAR_INFO tUsrVarInfo;
	vector<MP_USR_VAR_INFO> vtUsrVarInfo;
	vtUsrVarInfo.clear();
	int nJobPtnNum = 0;
	if (1 == tWavePara.waveType)
	{
		nJobPtnNum = 39;
	}
	else
	{
		nJobPtnNum = 40;
	}
	int nTotalNum = vtWeldCoord.size();
	int nSendNum = 0;
	int nPreMovePtnNum = 0; // 记录上次发送轨迹时运动走过点数
	int nMovePtnNum = 0; // 焊接Job当前走过点数
	double dLeftWaveWaitTime = vdWaveWaitTime.size() == weldSpeedRate.size() ? vdWaveWaitTime[0] : tWavePara.dLeftWaitTime;
	double dRightWaveWaitTime = vdWaveWaitTime.size() == weldSpeedRate.size() ? vdWaveWaitTime[0] : tWavePara.dRightWaitTime;
	tUsrVarInfo = pRobotDriver->PrepareValData(11, nTotalNum);
	vtUsrVarInfo.push_back(tUsrVarInfo);
	tUsrVarInfo = pRobotDriver->PrepareValData(19, tWeldParam.WeldVelocity / 6);
	vtUsrVarInfo.push_back(tUsrVarInfo);
	tUsrVarInfo = pRobotDriver->PrepareValData(72, 1);
	vtUsrVarInfo.push_back(tUsrVarInfo);
	//tUsrVarInfo = pRobotDriver->PrepareValData(79, (int)(tWavePara.dMidWaitTime * 100));
	tUsrVarInfo = pRobotDriver->PrepareValData(79, (int)(0.0 * 100));
	vtUsrVarInfo.push_back(tUsrVarInfo);
	//tUsrVarInfo = pRobotDriver->PrepareValData(92, (int)(tWavePara.dLeftWaitTime * 100));
	tUsrVarInfo = pRobotDriver->PrepareValData(92, (int)(dLeftWaveWaitTime * 100));
	vtUsrVarInfo.push_back(tUsrVarInfo);
	//tUsrVarInfo = pRobotDriver->PrepareValData(91, (int)(tWavePara.dRightWaitTime * 100));
	tUsrVarInfo = pRobotDriver->PrepareValData(91, (int)(dRightWaveWaitTime * 100));
	vtUsrVarInfo.push_back(tUsrVarInfo);
	pRobotDriver->m_vtWeldLineInWorldPoints.clear();//焊接轨迹
	pRobotDriver->m_vtWeldLineInWorldPoints.insert(pRobotDriver->m_vtWeldLineInWorldPoints.end(), vtWeldCoord.begin(), vtWeldCoord.end());
	while (nSendNum < nJobPtnNum && nSendNum < nTotalNum)
	{
		tUsrVarInfo = pRobotDriver->PrepareValData((nSendNum % nJobPtnNum) + 20, vtWeldCoord[nSendNum]);
		vtUsrVarInfo.push_back(tUsrVarInfo);
		tUsrVarInfo = pRobotDriver->PrepareValData((nSendNum % nJobPtnNum) + 15, weldSpeedRate[nSendNum] * tWeldParam.WeldVelocity / 6);
		vtUsrVarInfo.push_back(tUsrVarInfo);
		double adBasePosVar[3] = { vtWeldCoord[nSendNum].dBX,vtWeldCoord[nSendNum].dBY,vtWeldCoord[nSendNum].dBZ };
		double adSendBasePosVar[3] = { 0.0 };
		pRobotDriver->RobotTransCoordToBase(adBasePosVar, adSendBasePosVar);
		pRobotDriver->SetBasePosVar((nSendNum % nJobPtnNum) + 20, adSendBasePosVar, 0);
		nSendNum++;
	}
	pRobotDriver->SetMultiVar_H(vtUsrVarInfo);
	if (false == bIsCallJob) // 有多端焊接轨迹时 只CallJob一次即可
	{
		bIsCallJob = true;
		if (0 == tWavePara.waveType)
		{
			if (tWavePara.bStandWeld)
			{
				pRobotDriver->CallJob("WELDGROOVE");
			}
			else
			{
				pRobotDriver->CallJob("WELDGROOVE-BP");
			}
		}
		else if (1 == tWavePara.waveType)
		{
			if (tWavePara.bStandWeld)
			{
				pRobotDriver->CallJob("WELDTRIGROOVE");
			}
			else
			{
				pRobotDriver->CallJob("WELDTRIGROOVE-BP");
			}
		}
		else
		{
			//已修改
			XUI::MesBox::PopInfo("摆动设置错误！");
			//XiMessageBox("摆动设置错误！");
		}
		Sleep(1000); // 确保CallJob运动起来
	}
	// 循环发送轨迹
	while (true/*nSendNum < nTotalNum*/) // 有点？
	{
		if (false == WorldIsRunning()) // 运动中？
		{
			WriteLog("焊接过程中异常停止");
			return false;
		}
		nMovePtnNum = pRobotDriver->GetIntVar(1);
		pRobotDriver->m_cLog->Write("Read I001 = %d", nMovePtnNum);
		if (nMovePtnNum <= nPreMovePtnNum) // 没有新走过的轨迹点
		{
			Sleep(200);
			continue;
		}

		int nTemp = pRobotDriver->m_vtWeldLineInWorldPoints.size();
		if ((nMovePtnNum > nTemp) || (nMovePtnNum > nPreMovePtnNum + 20))
		{
			pRobotDriver->HoldOn();
			Sleep(500);
			pRobotDriver->HoldOff();
			//已修改
			XUI::MesBox::PopInfo("焊接轨迹数量检测异常:{0}I001={1}Pre{2}", nTemp, nMovePtnNum, nPreMovePtnNum);
			//XiMessageBoxOk("焊接轨迹数量检测异常:%d I001=%d Pre%d", nTemp, nMovePtnNum, nPreMovePtnNum);
			break;
		}

		if (nMovePtnNum >= nTemp - 1)
		{
			pRobotDriver->m_cLog->Write("正常结束加劲板数据发送流程:%d I001=%d", nTemp, nMovePtnNum);
			break;
		}
		if (INCISEHEAD_THREAD_STATUS_STOPPED == pRobotDriver->m_eThreadStatus)
		{
			pRobotDriver->m_cLog->Write("急停结束加劲板数据发送流程:%d I001=%d", nTemp, nMovePtnNum);
			break;
		}
		if (nSendNum >= nTemp)
		{
			pRobotDriver->m_cLog->Write("焊接数据发送流程结束:Send:%d Total:%d", nSendNum, nTemp);
			break;
		}

		// 准备并发送轨迹
		vtUsrVarInfo.clear();
		int nNeedSendNum = nMovePtnNum - nPreMovePtnNum;
		int nEIdx = (nSendNum + nNeedSendNum > nTemp/*nTotalNum*/ ? nTemp/*nTotalNum*/ : nSendNum + nNeedSendNum);
		for (; nSendNum < nEIdx; nSendNum++)
		{
			pRobotDriver->m_cLog->Write("linshi nSendNum = %d nTotalNum = %d", nSendNum, nTemp);
			tUsrVarInfo = pRobotDriver->PrepareValData((nSendNum % nJobPtnNum) + 20, pRobotDriver->m_vtWeldLineInWorldPoints[nSendNum]/*vtWeldCoord[nSendNum]*/);
			vtUsrVarInfo.push_back(tUsrVarInfo);
			tUsrVarInfo = pRobotDriver->PrepareValData((nSendNum % nJobPtnNum) + 15, weldSpeedRate[nSendNum] * tWeldParam.WeldVelocity / 6);
			vtUsrVarInfo.push_back(tUsrVarInfo);

			double adBasePosVar[3] = {
				pRobotDriver->m_vtWeldLineInWorldPoints[nSendNum].dBX,
				pRobotDriver->m_vtWeldLineInWorldPoints[nSendNum].dBY,
				pRobotDriver->m_vtWeldLineInWorldPoints[nSendNum].dBZ };
			double adSendBasePosVar[3] = { 0.0 };
			pRobotDriver->RobotTransCoordToBase(adBasePosVar, adSendBasePosVar);
			pRobotDriver->m_pYaskawaRobotCtrl->SetBasePosVar((nSendNum % nJobPtnNum) + 20, adSendBasePosVar, 0);
		}
		if (vdWaveWaitTime.size() == weldSpeedRate.size()) // 摆动停留时间渐变
		{
			tUsrVarInfo = pRobotDriver->PrepareValData(91, (int)(vdWaveWaitTime[nMovePtnNum] * 100.0));
			vtUsrVarInfo.push_back(tUsrVarInfo);
			tUsrVarInfo = pRobotDriver->PrepareValData(92, (int)(vdWaveWaitTime[nMovePtnNum] * 100.0));
			vtUsrVarInfo.push_back(tUsrVarInfo);
		}
		pRobotDriver->m_cLog->Write("nPreMovePtnNum = %d nMovePtnNum = %d nNeedSendNum = %d nSendNum = %d nTotalNum = %d %d",
			nPreMovePtnNum, nMovePtnNum, nNeedSendNum, nSendNum, nTotalNum, pRobotDriver->m_vtWeldLineInWorldPoints.size());
		pRobotDriver->SetMultiVar_H(vtUsrVarInfo);
		nPreMovePtnNum = nMovePtnNum;

		// 发送轨迹后：不跟踪 && 轨迹点发完 ？ 退出
		if (false == pRobotDriver->m_bIsOpenTrack && nSendNum >= nTotalNum)
		{
			pRobotDriver->m_cLog->Write("不跟踪轨迹发送完成 退出！");
			break;
		}
		DoEvent();
		Sleep(100);
	}
	pRobotDriver->m_cLog->Write("结束数据发送:%d", pRobotDriver->m_vtWeldLineInWorldPoints.size());

	if (TRUE == bIsArcOn)
	{
		// 临时处理焊接轨迹发送完成后发送一个收弧轨迹点，新收弧方法：切换收弧电流电压 -> 停留 -> 回退10mm -> 停留 -> 收弧
		// 对m_vtWeldLineInWorldPoints.size()中最后一个坐标操作 外部轴相同，机器人xyz向焊接起点偏10mm（立焊可能不同）
		// 发送道P变量，修改机器人收弧Job 配合使用
		double dStopArcDis = 10.0;
		T_ROBOT_COORS tStartCoord = pRobotDriver->m_vtWeldLineInWorldPoints.front();
		T_ROBOT_COORS tEndCoord = pRobotDriver->m_vtWeldLineInWorldPoints.back();
		T_ROBOT_COORS tStopCoord = tEndCoord;
		double dDis = TwoPointDis(
			tStartCoord.dX + tStartCoord.dBX, tStartCoord.dY + tStartCoord.dBY, tStartCoord.dZ + tStartCoord.dBZ,
			tEndCoord.dX + tEndCoord.dBX, tEndCoord.dY + tEndCoord.dBY, tEndCoord.dZ + tEndCoord.dBZ);
		double dDisX = (tStartCoord.dX + tStartCoord.dBX) - (tEndCoord.dX + tEndCoord.dBX);
		double dDisY = (tStartCoord.dY + tStartCoord.dBY) - (tEndCoord.dY + tEndCoord.dBY);
		double dDisZ = (tStartCoord.dZ + tStartCoord.dBZ) - (tEndCoord.dZ + tEndCoord.dBZ);
		tStopCoord.dX += (dStopArcDis * dDisX / dDis);
		tStopCoord.dY += (dStopArcDis * dDisY / dDis);
		tStopCoord.dZ += (dStopArcDis * dDisZ / dDis);
		pRobotDriver->m_vtWeldLineInWorldPoints.push_back(tStopCoord);
		vtUsrVarInfo.clear();
		tUsrVarInfo = pRobotDriver->PrepareValData(61, tWeldParam.WeldVelocity / 6);
		vtUsrVarInfo.push_back(tUsrVarInfo);
		tUsrVarInfo = pRobotDriver->PrepareValData(61, pRobotDriver->m_vtWeldLineInWorldPoints.back());
		vtUsrVarInfo.push_back(tUsrVarInfo);
		pRobotDriver->SetMultiVar_H(vtUsrVarInfo);
		pRobotDriver->m_cLog->Write("结束收弧轨迹点发送:%d", pRobotDriver->m_vtWeldLineInWorldPoints.size());
	}
	return true;
}

bool CUnit::SendArcOnData(const std::vector<T_ROBOT_COORS>& vtWeldCoord, const T_WELD_PARA& tWeldParam, int ToolNum)
{
	CRobotDriverAdaptor *pRobotDriver = GetRobotCtrl();
	// 点总数：I001 速度：I004 起弧停留时间：I009 位置：P001-P007
	MP_USR_VAR_INFO tUsrVarInfo;
	vector<MP_USR_VAR_INFO> vtUsrVarInfo;
	vtUsrVarInfo.clear();
	int nPtnNum = vtWeldCoord.size();
	if (nPtnNum > 7) // 点数0-7
	{
		//已修改
		XUI::MesBox::PopInfo("起弧焊接运动轨迹点数量{0}错误", nPtnNum);
		//XiMessageBox("起弧焊接运动轨迹点数量%d错误", nPtnNum);
		return false;
	}

	tUsrVarInfo = pRobotDriver->PrepareValData(20, nPtnNum);
	vtUsrVarInfo.push_back(tUsrVarInfo);

	tUsrVarInfo = pRobotDriver->PrepareValData(4, tWeldParam.WeldVelocity / 6);
	vtUsrVarInfo.push_back(tUsrVarInfo);
	tUsrVarInfo = pRobotDriver->PrepareValData(9, (int)(tWeldParam.dStartWaitTime * 100.0)); // TIMER
	vtUsrVarInfo.push_back(tUsrVarInfo);
	for (int i = 0; i < nPtnNum; i++) {
		tUsrVarInfo = pRobotDriver->PrepareValData(i + 1, vtWeldCoord[i], ToolNum);
		vtUsrVarInfo.push_back(tUsrVarInfo);
	}
	pRobotDriver->SetMultiVar_H(vtUsrVarInfo);
	return true;
}

bool CUnit::SendWrapData(const std::vector<T_ROBOT_COORS>& vtWeldCoord, const T_WELD_PARA& tWeldParam/*, const WELD_WRAP_ANGLE& tWrapParam*/)
{
	WriteLog("SendWrapData Start!");
	CRobotDriverAdaptor *pRobotDriver = GetRobotCtrl();
	// 发送设置工艺参数 点数：I011 速度：I014  位置：P011-P018 需要包角：I071
	MP_USR_VAR_INFO tUsrVarInfo;
	vector<MP_USR_VAR_INFO> vtUsrVarInfo;
	vtUsrVarInfo.clear();
	int nPtnNum = vtWeldCoord.size();
	if (nPtnNum > 8) // 包角轨迹最多8个点
	{
		//已修改
		XUI::MesBox::PopInfo("包角运动轨迹点数量{0}错误", nPtnNum);
		//XiMessageBox("包角运动轨迹点数量%d错误", nPtnNum);
		return false;
	}
	tUsrVarInfo = pRobotDriver->PrepareValData(11, nPtnNum);
	vtUsrVarInfo.push_back(tUsrVarInfo);
	tUsrVarInfo = pRobotDriver->PrepareValData(14, tWeldParam.WeldVelocity / 6 /*tWrapParam.m_dWrapdVelocity / 6*/);
	vtUsrVarInfo.push_back(tUsrVarInfo);
	tUsrVarInfo = pRobotDriver->PrepareValData(71, 1);
	vtUsrVarInfo.push_back(tUsrVarInfo);
	for (int i = 0; i < nPtnNum; i++) {
		tUsrVarInfo = pRobotDriver->PrepareValData(i + 11, vtWeldCoord[i]); // 位置：P011-P018
		vtUsrVarInfo.push_back(tUsrVarInfo);
	}
	pRobotDriver->SetMultiVar_H(vtUsrVarInfo);
	return true;
}
//（问题：未完成WorldIsRunning()定义：该函数依赖WorldIsRunning()  该函数已完成定义：注释依赖项
bool CUnit::SendWeldData(std::vector<T_ROBOT_COORS>& vtWeldCoord, const T_WELD_PARA& tWeldParam, E_WELD_SEAM_TYPE eWeldSeamType, bool& bIsCallJob, int ToolNum)
{
	WriteLog("SendWeldData Start!");
	CRobotDriverAdaptor *pRobotDriver = GetRobotCtrl();
	MP_USR_VAR_INFO tUsrVarInfo;
	vector<MP_USR_VAR_INFO> vtUsrVarInfo;
	vtUsrVarInfo.clear();
	int nJobPtnNum = 40;
	int nTotalNum = vtWeldCoord.size();
	int nSendNum = 0;
	int nPreMovePtnNum = 0; // 记录上次发送轨迹时运动走过点数
	int nMovePtnNum = 0; // 焊接Job当前走过点数

	tUsrVarInfo = pRobotDriver->PrepareValData(21, nTotalNum);
	vtUsrVarInfo.push_back(tUsrVarInfo);
	tUsrVarInfo = pRobotDriver->PrepareValData(24, tWeldParam.WeldVelocity / 6);
	vtUsrVarInfo.push_back(tUsrVarInfo);
	tUsrVarInfo = pRobotDriver->PrepareValData(72, 1);
	vtUsrVarInfo.push_back(tUsrVarInfo);
	if (tWeldParam.nWrapConditionNo > 0) // 摆动条件号有0 ？ 
	{
		tUsrVarInfo = pRobotDriver->PrepareValData(8, tWeldParam.nWrapConditionNo);
		vtUsrVarInfo.push_back(tUsrVarInfo);
		tUsrVarInfo = pRobotDriver->PrepareValData(59, 1);
		vtUsrVarInfo.push_back(tUsrVarInfo);

		//tUsrVarInfo = PrepareValData(20, 0); // 焊接点数复位，防止其他运动设置20影响焊接运动计数
		//vtUsrVarInfo.push_back(tUsrVarInfo);

		// 起点焊接方向反向0.5mm 摆动焊接接近点
		//T_ROBOT_COORS tTempCoord = RobotCoordPosOffset(vtWeldCoord[0], vtWeldCoord[1], vtWeldCoord[0], 0.5);
		double dOffsetDis = 0.5;
		T_ROBOT_COORS tTempCoord = vtWeldCoord[0];
		T_ROBOT_COORS tDirSPtn = vtWeldCoord[1];
		T_ROBOT_COORS tDirEPtn = vtWeldCoord[0];
		double dDis = TwoPointDis(tDirSPtn.dX, tDirSPtn.dY, tDirSPtn.dZ, tDirEPtn.dX, tDirEPtn.dY, tDirEPtn.dZ);
		double dDisX = tDirEPtn.dX - tDirSPtn.dX;
		double dDisY = tDirEPtn.dY - tDirSPtn.dY;
		double dDisZ = tDirEPtn.dZ - tDirSPtn.dZ;
		tTempCoord.dX += (dDisX * dOffsetDis / dDis);
		tTempCoord.dY += (dDisY * dOffsetDis / dDis);
		tTempCoord.dZ += (dDisZ * dOffsetDis / dDis);

		tUsrVarInfo = pRobotDriver->PrepareValData(19, tTempCoord, ToolNum); // 摆焊接近点
		vtUsrVarInfo.push_back(tUsrVarInfo);

		T_ROBOT_COORS tRefp1 = vtWeldCoord[0];
		T_ROBOT_COORS tRefp2 = vtWeldCoord[0];
		CalcSwayRefpCoord(vtWeldCoord[0], eWeldSeamType, tRefp1, tRefp2);
		tUsrVarInfo = pRobotDriver->PrepareValData(60, tRefp1, ToolNum);
		vtUsrVarInfo.push_back(tUsrVarInfo);
		tUsrVarInfo = pRobotDriver->PrepareValData(61, tRefp2, ToolNum);
		vtUsrVarInfo.push_back(tUsrVarInfo);
	}
	else {
		tUsrVarInfo = pRobotDriver->PrepareValData(59, 0);
		vtUsrVarInfo.push_back(tUsrVarInfo);
	}
	// 跟踪截取数据

	pRobotDriver->m_vtWeldLineInWorldPoints.clear();//焊接轨迹
	pRobotDriver->m_vtWeldLineInWorldPoints.insert(pRobotDriver->m_vtWeldLineInWorldPoints.end(),
		vtWeldCoord.begin(), vtWeldCoord.end());

	// 焊接运动第一圈轨迹
	while (nSendNum < nJobPtnNum && nSendNum < nTotalNum) {
		tUsrVarInfo = pRobotDriver->PrepareValData((nSendNum % nJobPtnNum) + 20, vtWeldCoord[nSendNum], ToolNum);
		vtUsrVarInfo.push_back(tUsrVarInfo);
		/*tUsrVarInfo = pRobotDriver->PrepareValDataEx((nSendNum % nJobPtnNum) + 20, vtWeldCoord[nSendNum], ToolNum);
		vtUsrVarInfo.push_back(tUsrVarInfo);*/
		nSendNum++;
	}
	WriteLog("SendWeldData Fitst Time Start!");
	pRobotDriver->SetMultiVar_H(vtUsrVarInfo);
	WriteLog("SendWeldData Fitst Time Done!");
	if (false == bIsCallJob) // 有多端焊接轨迹时 只CallJob一次即可
	{
		bIsCallJob = true;
		//jwq 平焊立焊调用不同JOB 
		if (E_FLAT_SEAM == eWeldSeamType)
		{
			pRobotDriver->CallJob("GENERALWELD");

		}
		else if (E_STAND_SEAM == eWeldSeamType)
		{
			//pRobotDriver->CallJob("GENERALWELD-L");
			pRobotDriver->CallJob("GENERALWELD");
		}
		else
		{
			XiMessageBox("不存在的焊接类型！");
			return false;
		}
		Sleep(1000); // 确保CallJob运动起来
	}
	// 循环发送轨迹
	while (true/*nSendNum < nTotalNum*/) // 有点？
	{
		if (!g_bLocalDebugMark) {
			if (false == WorldIsRunning()) // 运动中？
			{
				WriteLog("焊接过程中异常停止");
				return false;
			}
		}
		nMovePtnNum = pRobotDriver->GetIntVar(1);
		//WriteLog("Read I001 = %d", nMovePtnNum);
		if (nMovePtnNum <= nPreMovePtnNum) // 没有新走过的轨迹点
		{
			Sleep(200);
			continue;
		}

		int nTemp = pRobotDriver->m_vtWeldLineInWorldPoints.size();
		if ((nMovePtnNum > nTemp) || (nMovePtnNum > nPreMovePtnNum + 20)) {
			pRobotDriver->HoldOn();
			Sleep(500);
			pRobotDriver->HoldOff();
			//已修改
			XUI::MesBox::PopInfo("焊接轨迹数量检测异常:{0}I001={1}Pre{2}", nTemp, nMovePtnNum, nPreMovePtnNum);
			//XiMessageBoxOk("焊接轨迹数量检测异常:%d I001=%d Pre%d", nTemp, nMovePtnNum, nPreMovePtnNum);
			break;
		}

		if (nMovePtnNum >= nTemp - 1) {
			pRobotDriver->m_cLog->Write("正常结束加劲板数据发送流程:%d I001=%d", nTemp, nMovePtnNum);
			break;
		}
		if (INCISEHEAD_THREAD_STATUS_STOPPED == pRobotDriver->m_eThreadStatus) {
			pRobotDriver->m_cLog->Write("急停结束加劲板数据发送流程:%d I001=%d", nTemp, nMovePtnNum);
			break;
		}
		if (nSendNum >= nTemp) {
			pRobotDriver->m_cLog->Write("焊接数据发送流程结束:Send:%d Total:%d", nSendNum, nTemp);
			break;
		}

		// 准备并发送轨迹
		vtUsrVarInfo.clear();
		int nNeedSendNum = nMovePtnNum - nPreMovePtnNum;
		int nEIdx = (nSendNum + nNeedSendNum > nTemp/*nTotalNum*/ ? nTemp/*nTotalNum*/ : nSendNum + nNeedSendNum);
		for (; nSendNum < nEIdx; nSendNum++) {
			pRobotDriver->m_cLog->Write("linshi nSendNum = %d nTotalNum = %d", nSendNum, nTemp);
			tUsrVarInfo = pRobotDriver->PrepareValData((nSendNum % nJobPtnNum) + 20, pRobotDriver->m_vtWeldLineInWorldPoints[nSendNum]/*vtWeldCoord[nSendNum]*/, ToolNum);
			vtUsrVarInfo.push_back(tUsrVarInfo);
			//tUsrVarInfo = pRobotDriver->PrepareValDataEx((nSendNum % nJobPtnNum) + 20, pRobotDriver->m_vtWeldLineInWorldPoints[nSendNum], ToolNum);
			//vtUsrVarInfo.push_back(tUsrVarInfo);
		}
		pRobotDriver->m_cLog->Write("nPreMovePtnNum = %d nMovePtnNum = %d nNeedSendNum = %d nSendNum = %d nTotalNum = %d %d",
			nPreMovePtnNum, nMovePtnNum, nNeedSendNum, nSendNum, nTotalNum, pRobotDriver->m_vtWeldLineInWorldPoints.size());
		pRobotDriver->SetMultiVar_H(vtUsrVarInfo);
		nPreMovePtnNum = nMovePtnNum;

		// 发送轨迹后：不跟踪 && 轨迹点发完 ？ 退出
		if (nSendNum >= nTotalNum) {
			pRobotDriver->m_cLog->Write("不跟踪轨迹发送完成 退出！");
			break;
		}
		DoEvent();
		Sleep(100);
	}
	pRobotDriver->m_cLog->Write("结束数据发送:%d", pRobotDriver->m_vtWeldLineInWorldPoints.size());
	return true;
}

bool CUnit::SendWeldData_BP(std::vector<T_ROBOT_COORS>& vtWeldCoord, const T_WELD_PARA& tWeldParam, E_WELD_SEAM_TYPE eWeldSeamType, bool& bIsCallJob)
{
	GetRobotCtrl()->m_cLog->Write("SendWeldData Start!");
	CRobotDriverAdaptor* pRobotDriver = GetRobotCtrl();
	MP_USR_VAR_INFO tUsrVarInfo;
	vector<MP_USR_VAR_INFO> vtUsrVarInfo;
	vtUsrVarInfo.clear();
	double adExCoord[40][3] = { 0.0 };
	double adSendBasePosVar[3] = { 0.0 };
	int nJobPtnNum = 40;
	int nTotalNum = vtWeldCoord.size();
	int nSendNum = 0;
	int nPreMovePtnNum = 0; // 记录上次发送轨迹时运动走过点数
	int nMovePtnNum = 0; // 焊接Job当前走过点数

	tUsrVarInfo = pRobotDriver->PrepareValData(21, nTotalNum);
	vtUsrVarInfo.push_back(tUsrVarInfo);
	tUsrVarInfo = pRobotDriver->PrepareValData(24, tWeldParam.WeldVelocity / 6);
	vtUsrVarInfo.push_back(tUsrVarInfo);
	tUsrVarInfo = pRobotDriver->PrepareValData(72, 1);
	vtUsrVarInfo.push_back(tUsrVarInfo);
	if (tWeldParam.nWrapConditionNo > 0) // 摆动条件号有0 ？ 
	{
		tUsrVarInfo = pRobotDriver->PrepareValData(8, tWeldParam.nWrapConditionNo);
		vtUsrVarInfo.push_back(tUsrVarInfo);
		tUsrVarInfo = pRobotDriver->PrepareValData(59, 1);
		vtUsrVarInfo.push_back(tUsrVarInfo);

		// 起点焊接方向反向0.5mm 摆动焊接接近点
		//T_ROBOT_COORS tTempCoord = RobotCoordPosOffset(vtWeldCoord[0], vtWeldCoord[1], vtWeldCoord[0], 0.5);
		double dOffsetDis = 0.5;
		T_ROBOT_COORS tTempCoord = vtWeldCoord[0];
		T_ROBOT_COORS tDirSPtn = vtWeldCoord[1];
		T_ROBOT_COORS tDirEPtn = vtWeldCoord[0];
		double dDis = TwoPointDis(tDirSPtn.dX, tDirSPtn.dY, tDirSPtn.dZ, tDirEPtn.dX, tDirEPtn.dY, tDirEPtn.dZ);
		double dDisX = tDirEPtn.dX - tDirSPtn.dX;
		double dDisY = tDirEPtn.dY - tDirSPtn.dY;
		double dDisZ = tDirEPtn.dZ - tDirSPtn.dZ;
		tTempCoord.dX += (dDisX * dOffsetDis / dDis);
		tTempCoord.dY += (dDisY * dOffsetDis / dDis);
		tTempCoord.dZ += (dDisZ * dOffsetDis / dDis);

		tUsrVarInfo = pRobotDriver->PrepareValData(19, tTempCoord); // 摆焊接近点
		vtUsrVarInfo.push_back(tUsrVarInfo);

		T_ROBOT_COORS tRefp1 = vtWeldCoord[0];
		T_ROBOT_COORS tRefp2 = vtWeldCoord[0];
		CalcSwayRefpCoord(vtWeldCoord[0], eWeldSeamType, tRefp1, tRefp2);
		tUsrVarInfo = pRobotDriver->PrepareValData(60, tRefp1);
		vtUsrVarInfo.push_back(tUsrVarInfo);
		tUsrVarInfo = pRobotDriver->PrepareValData(61, tRefp2);
		vtUsrVarInfo.push_back(tUsrVarInfo);
	}
	else {
		tUsrVarInfo = pRobotDriver->PrepareValData(59, 0);
		vtUsrVarInfo.push_back(tUsrVarInfo);
	}

	// 跟踪截取数据
	pRobotDriver->m_vtWeldLineInWorldPoints.clear();//焊接轨迹
	pRobotDriver->m_vtWeldLineInWorldPoints.insert(pRobotDriver->m_vtWeldLineInWorldPoints.end(),
		vtWeldCoord.begin(), vtWeldCoord.end());

	// 焊接运动第一圈轨迹
	while (nSendNum < nJobPtnNum && nSendNum < nTotalNum) {
		tUsrVarInfo = pRobotDriver->PrepareValData((nSendNum % nJobPtnNum) + 20, vtWeldCoord[nSendNum]);
		adExCoord[nSendNum][0] = vtWeldCoord[nSendNum].dBX;
		adExCoord[nSendNum][1] = vtWeldCoord[nSendNum].dBY;
		adExCoord[nSendNum][2] = vtWeldCoord[nSendNum].dBZ;
		vtUsrVarInfo.push_back(tUsrVarInfo);
		nSendNum++;
	}
	GetRobotCtrl()->m_cLog->Write("SendWeldData Fitst Time Start!");
	pRobotDriver->SetMultiVar_H(vtUsrVarInfo);
	for (int i = 0; i < nSendNum; i++) // 发送BP变量
	{
		GetRobotCtrl()->RobotTransCoordToBase(adExCoord[i], adSendBasePosVar);
		GetRobotCtrl()->SetBasePosVar(20 + i, adSendBasePosVar, 0);
		GetRobotCtrl()->m_cLog->Write("Init Send BasePosVar:%11.3lf%11.3lf%11.3lf",
			adSendBasePosVar[0], adSendBasePosVar[1], adSendBasePosVar[2]);
	}
	GetRobotCtrl()->m_cLog->Write("SendWeldData Fitst Time Done!");
	if (false == bIsCallJob) // 有多端焊接轨迹时 只CallJob一次即可
	{
		bIsCallJob = true;
		pRobotDriver->CallJob("GENERALWELD-BP");
		Sleep(1000); // 确保CallJob运动起来
	}
	// 循环发送轨迹
	while (true/*nSendNum < nTotalNum*/) // 有点？
	{
		if (!g_bLocalDebugMark) {
			if (false == WorldIsRunning()) // 运动中？
			{
				GetRobotCtrl()->m_cLog->Write("焊接过程中异常停止");
				return false;
			}
		}
		nMovePtnNum = pRobotDriver->GetIntVar(1);
		//GetRobotCtrl()->m_cLog->Write("Read I001 = %d", nMovePtnNum);
		if (nMovePtnNum <= nPreMovePtnNum) // 没有新走过的轨迹点
		{
			Sleep(200);
			continue;
		}

		int nTemp = pRobotDriver->m_vtWeldLineInWorldPoints.size();
		if ((nMovePtnNum > nTemp) || (nMovePtnNum > nPreMovePtnNum + 20)) {
			pRobotDriver->HoldOn();
			Sleep(500);
			pRobotDriver->HoldOff();
			//已修改
			XUI::MesBox::PopInfo("焊接轨迹数量检测异常:{0}I001={1}Pre{2}", nTemp, nMovePtnNum, nPreMovePtnNum);
			//XiMessageBoxOk("焊接轨迹数量检测异常:%d I001=%d Pre%d", nTemp, nMovePtnNum, nPreMovePtnNum);
			break;
		}

		if (nMovePtnNum >= nTemp - 1) {
			pRobotDriver->m_cLog->Write("正常结束加劲板数据发送流程:%d I001=%d", nTemp, nMovePtnNum);
			break;
		}
		if (INCISEHEAD_THREAD_STATUS_STOPPED == pRobotDriver->m_eThreadStatus) {
			pRobotDriver->m_cLog->Write("急停结束加劲板数据发送流程:%d I001=%d", nTemp, nMovePtnNum);
			break;
		}
		if (nSendNum >= nTemp) {
			pRobotDriver->m_cLog->Write("焊接数据发送流程结束:Send:%d Total:%d", nSendNum, nTemp);
			break;
		}

		// 准备并发送轨迹
		vtUsrVarInfo.clear();
		int nNeedSendNum = nMovePtnNum - nPreMovePtnNum;
		int nEIdx = (nSendNum + nNeedSendNum > nTemp/*nTotalNum*/ ? nTemp/*nTotalNum*/ : nSendNum + nNeedSendNum);
		for (; nSendNum < nEIdx; nSendNum++) {
			pRobotDriver->m_cLog->Write("linshi nSendNum = %d nTotalNum = %d", nSendNum, nTemp);
			tUsrVarInfo = pRobotDriver->PrepareValData((nSendNum % nJobPtnNum) + 20, pRobotDriver->m_vtWeldLineInWorldPoints[nSendNum]/*vtWeldCoord[nSendNum]*/);
			vtUsrVarInfo.push_back(tUsrVarInfo);

			adExCoord[0][0] = pRobotDriver->m_vtWeldLineInWorldPoints[nSendNum].dBX;
			adExCoord[0][1] = pRobotDriver->m_vtWeldLineInWorldPoints[nSendNum].dBY;
			adExCoord[0][2] = pRobotDriver->m_vtWeldLineInWorldPoints[nSendNum].dBZ;
			GetRobotCtrl()->RobotTransCoordToBase(adExCoord[0], adSendBasePosVar);
			GetRobotCtrl()->SetBasePosVar((nSendNum% nJobPtnNum) + 20, adSendBasePosVar, 0); // 发送BP变量
			GetRobotCtrl()->m_cLog->Write("RealTime Send BasePosVar:%11.3lf%11.3lf%11.3lf",
				adSendBasePosVar[0], adSendBasePosVar[1], adSendBasePosVar[2]);
		}
		pRobotDriver->m_cLog->Write("nPreMovePtnNum = %d nMovePtnNum = %d nNeedSendNum = %d nSendNum = %d nTotalNum = %d %d",
			nPreMovePtnNum, nMovePtnNum, nNeedSendNum, nSendNum, nTotalNum, pRobotDriver->m_vtWeldLineInWorldPoints.size());
		pRobotDriver->SetMultiVar_H(vtUsrVarInfo);
		nPreMovePtnNum = nMovePtnNum;

		// 发送轨迹后：不跟踪 && 轨迹点发完 ？ 退出
		if (nSendNum >= nTotalNum) {
			pRobotDriver->m_cLog->Write("不跟踪轨迹发送完成 退出！");
			break;
		}
		DoEvent();
		Sleep(100);
	}
	pRobotDriver->m_cLog->Write("结束数据发送:%d", pRobotDriver->m_vtWeldLineInWorldPoints.size());
	return true;
}

void CUnit::CalcSwayRefpCoord(T_ROBOT_COORS tStartCoord, E_WELD_SEAM_TYPE eWeldSeamType, T_ROBOT_COORS& tRefp1, T_ROBOT_COORS& tRefp2)
{
	CRobotDriverAdaptor *pRobotDriver = GetRobotCtrl();
	double dDis = 6.0;
	tRefp1 = tStartCoord;
	tRefp2 = tStartCoord;
	double dWeldDirAngle = pRobotDriver->RzToDirAngle(tStartCoord.dRZ);
	switch (eWeldSeamType) {
	case E_FLAT_SEAM:
		tRefp1.dX += (dDis * CosD(dWeldDirAngle));
		tRefp1.dY += (dDis * SinD(dWeldDirAngle));
		tRefp2.dZ += (dDis * (double)pRobotDriver->m_nRobotInstallDir);
		break;
	case E_STAND_SEAM:
		tRefp1.dX += (dDis * CosD(dWeldDirAngle + 45.0));
		tRefp1.dY += (dDis * SinD(dWeldDirAngle + 45.0));
		tRefp2.dX += (dDis * CosD(dWeldDirAngle - 45.0));
		tRefp2.dY += (dDis * SinD(dWeldDirAngle - 45.0));
		break;
	case E_PLAT_GROOVE:
		tRefp1.dX += (dDis * CosD(dWeldDirAngle + 90.0));
		tRefp1.dY += (dDis * SinD(dWeldDirAngle + 90.0));
		tRefp1.dZ += dDis;
		tRefp2.dX += (dDis * CosD(dWeldDirAngle - 90.0));
		tRefp2.dY += (dDis * SinD(dWeldDirAngle - 90.0));
		tRefp2.dZ += (dDis * (double)pRobotDriver->m_nRobotInstallDir);
		break;
	}
}




//计算转换矩阵
//vGantryTransRobotMat 计算得到的龙门转机器人坐标的转换矩阵
//vRobotTransGantryMat 计算得到的机器人转龙门坐标的转换矩阵
int CUnit::CaliTranMat(std::vector<cv::Mat>& vGantryTransRobotMat, std::vector<cv::Mat>& vRobotTransGantryMat, bool save /*=false*/)
{
	//author: wanglong 20231129
	char str[50];
	char* cstr = "";
	CString path;
	for (int i = 0; i < 4; i++) {
		std::vector<CvPoint3D64f> gantryPoints;
		FILE* pf_gantryCoor;
		path.Format(".\\Data\\标定参数\\龙门转换矩阵\\GantryCoor_%d.txt", i);
		XI_fopen_s(&pf_gantryCoor, path, "r");
		for (int j = 0; j < 4; j++)	//4个数据
		{
			//读取一行
			memset(str, '\0', 50);
			fgets(str, 50, pf_gantryCoor);
			CvPoint3D64f a;
			cstr = str;
			a.x = GetDouble(&cstr, 50);
			a.y = GetDouble(&cstr, 50);
			a.z = GetDouble(&cstr, 50);
			gantryPoints.push_back(a);
		}
		fclose(pf_gantryCoor);

		std::vector<CvPoint3D64f> robotPoints;
		FILE* pf_robotCoor;
		path.Format(".\\Data\\标定参数\\龙门转换矩阵\\RobotCoor_%d.txt", i);
		XI_fopen_s(&pf_robotCoor, path, "r");
		for (int j = 0; j < 4; j++)	//4个数据
		{
			//读取一行
			memset(str, '\0', 50);
			fgets(str, 50, pf_robotCoor);
			CvPoint3D64f a;
			cstr = str;
			a.x = GetDouble(&cstr, 50);
			a.y = GetDouble(&cstr, 50);
			a.z = GetDouble(&cstr, 50);
			robotPoints.push_back(a);
		}
		fclose(pf_robotCoor);

		cv::Mat mat1 = CalculateTransformationMatrix(gantryPoints, robotPoints);
		if (vGantryTransRobotMat.size() == 0) {
			vGantryTransRobotMat.push_back(mat1);
		}
		else {
			vGantryTransRobotMat[i] = mat1;
		}

		cv::Mat mat2 = CalculateTransformationMatrix(robotPoints, gantryPoints);
		if (vRobotTransGantryMat.size() == 0) {
			vRobotTransGantryMat.push_back(mat2);
		}
		else {
			vRobotTransGantryMat[i] = mat2;
		}
		if (save) {
			FILE* pf_mat_gantry;
			path.Format(".\\Data\\Mat\\MatWorldToRobot_%d.txt", i);
			XI_fopen_s(&pf_mat_gantry, path, "w");
			for (int j = 0; j < vGantryTransRobotMat[i].rows; j++)//行
			{
				for (int k = 0; k < vGantryTransRobotMat[i].cols; k++)//列
				{
					fprintf_s(pf_mat_gantry, "%5.9f\t", vGantryTransRobotMat[i].at<double>(j, k));
				}
				fprintf_s(pf_mat_gantry, "\n");
			}
			fclose(pf_mat_gantry);

			FILE* pf_mat_robot;
			path.Format(".\\Data\\Mat\\MatRobotToWorld_%d.txt", i);
			XI_fopen_s(&pf_mat_robot, path, "w");
			for (int j = 0; j < vRobotTransGantryMat[i].rows; j++)//行
			{
				for (int k = 0; k < vRobotTransGantryMat[i].cols; k++)//列
				{
					fprintf_s(pf_mat_robot, "%5.9f\t", vRobotTransGantryMat[i].at<double>(j, k));
				}
				fprintf_s(pf_mat_robot, "\n");
			}
			fclose(pf_mat_robot);
		}
	}

	COPini MatIni;
	MatIni.SetFileName(".\\Data\\Mat\\MatGantryRobot.ini");
	CString SectionName;
	CString KeyName;
	for (int i = 0; i < 4; i++)
	{
		SectionName.Format("MAT_GANTRY_TO_ROBOT_%d", i);
		MatIni.SetSectionName(SectionName);
		for (int j = 0; j < vGantryTransRobotMat[i].rows; j++)//行
		{
			for (int k = 0; k < vGantryTransRobotMat[i].cols; k++)//列
			{
				KeyName.Format("MAT_%d_%d", j + 1, k + 1);
				MatIni.WriteString(KeyName, vGantryTransRobotMat[i].at<double>(j, k), 9);
			}
		}

		SectionName.Format("MAT_ROBOT_TO_GANTRY_%d", i);
		MatIni.SetSectionName(SectionName);
		for (int j = 0; j < vGantryTransRobotMat[i].rows; j++)//行
		{
			for (int k = 0; k < vGantryTransRobotMat[i].cols; k++)//列
			{
				KeyName.Format("MAT_%d_%d", j + 1, k + 1);
				MatIni.WriteString(KeyName, vRobotTransGantryMat[i].at<double>(j, k), 9);
			}
		}
	}

	MatIni.SetSectionName("GANTRY_ROBOT_ORIGINAL_POINT_DISTANCE");
	CvPoint3D64f point;
	point.x = 0;
	point.y = 0;
	point.z = 0;
	std::vector< CvPoint3D64f> points;
	points.push_back(point);
	for (int i = 0; i < 4; i++) {
		KeyName.Format("ROBOT_%d", i);
		double distance = CoordinateTransformate(points, vGantryTransRobotMat[i])[0].x;	//取决于机器人的哪个坐标方向和轨道方向平行
		MatIni.WriteString(KeyName, distance);
	}

	return 0;
}

int CUnit::LoadTranMat(std::vector<cv::Mat>& vGantryTransRobotMat, std::vector<cv::Mat>& vRobotTransGantryMat, CString IniFileName)
{
	COPini MatIni;
	MatIni.SetFileName(IniFileName);
	CString SectionName;
	CString KeyName;
	double value = 0;
	for (int i = 0; i < 4; i++)
	{
		SectionName.Format("MAT_GANTRY_TO_ROBOT_%d", i);
		MatIni.SetSectionName(SectionName);
		cv::Mat mat(4, 4, CV_64F);
		for (int j = 0; j < 4; j++)//行
		{
			for (int k = 0; k < 4; k++)//列
			{
				value = 0;
				KeyName.Format("MAT_%d_%d", j + 1, k + 1);
				MatIni.ReadString(KeyName, &value);
				mat.at<double>(j, k) = value;
			}
		}
		vGantryTransRobotMat.push_back(mat);

		SectionName.Format("MAT_ROBOT_TO_GANTRY_%d", i);
		MatIni.SetSectionName(SectionName);
		cv::Mat mat2(4, 4, CV_64F);
		for (int j = 0; j < 4; j++)//行
		{
			for (int k = 0; k < 4; k++)//列
			{
				value = 0;
				KeyName.Format("MAT_%d_%d", j + 1, k + 1);
				MatIni.ReadString(KeyName, &value);
				mat2.at<double>(j, k) = value;
			}
		}
		vRobotTransGantryMat.push_back(mat2);
	}
	return 0;
}

int CUnit::LoadOriginDis(std::vector<double>& vGantryRobotOriginDis, CString IniFileName)
{
	COPini MatIni;
	MatIni.SetFileName(IniFileName);
	MatIni.SetSectionName("GANTRY_ROBOT_ORIGINAL_POINT_DISTANCE");
	CString KeyName;
	double value = 0;
	for (int i = 0; i < 4; i++)
	{
		KeyName.Format("ROBOT_%d", i);
		MatIni.ReadString(KeyName, &value);
		if (vGantryRobotOriginDis.size() < i + 1) {
			vGantryRobotOriginDis.push_back(value);
		}
		else {
			vGantryRobotOriginDis[i] = value;
		}
	}
	return 0;
}

bool CUnit::CalculateTransRelation(cv::Mat& GantryTransRobotMat, cv::Mat& RobotTransGantryMat,double &dBaseDis, double& OriginDis)
{
	COPini opini;
	CString strFile = DATA_PATH + GetRobotCtrl()->m_strRobotName + ROBOT_TRANS_RELATION;
	opini.SetFileName(strFile);
	opini.SetSectionName("RobotRelationPos"); 
	int nIsEnable = 0;
	opini.ReadString("IsEnable", &nIsEnable);
	if (1 != nIsEnable)
	{
		// 不使用转换矩阵时，初始化为恒等矩阵
		GantryTransRobotMat = cv::Mat::eye(4, 4, CV_64F);
		RobotTransGantryMat = cv::Mat::eye(4, 4, CV_64F);
		return false;
	}
	vector<T_ROBOT_COORS> vtRovotPos,vtGantryPos;
	vtRovotPos.resize(4);
	vtGantryPos.resize(4);
	opini.ReadString("FirstRobotFirstPoint.d", "", vtRovotPos[0]);
	opini.ReadString("FirstRobotSecondPoint.d", "", vtRovotPos[1]);
	opini.ReadString("FirstRobotThirdPoint.d", "", vtRovotPos[2]);
	opini.ReadString("FirstRobotFourthPoint.d", "", vtRovotPos[3]);

	opini.SetSectionName("GantryRelationPos");
	opini.ReadString("FirstRobotFirstPoint.d", "", vtGantryPos[0]);
	opini.ReadString("FirstRobotSecondPoint.d", "", vtGantryPos[1]);
	opini.ReadString("FirstRobotThirdPoint.d", "", vtGantryPos[2]);
	opini.ReadString("FirstRobotFourthPoint.d", "", vtGantryPos[3]);
	opini.SetSectionName("BaseDis");
	opini.ReadString("RobotBaseToGantryCtrDis", &dBaseDis);
	opini.ReadString("GantryRobotOriginDis", &OriginDis);
	

	std::vector<CvPoint3D64f> BasePoints;
	std::vector<CvPoint3D64f> TargetPoints;
	for (int i = 0; i < vtRovotPos.size(); i++)
	{
		CvPoint3D64f tRt = { vtRovotPos.at(i).dX,vtRovotPos.at(i).dY,vtRovotPos.at(i).dZ };
		CvPoint3D64f tGy = { vtGantryPos.at(i).dX,vtGantryPos.at(i).dY,vtGantryPos.at(i).dZ };
		BasePoints.push_back(tRt);
		TargetPoints.push_back(tGy);
	}
	GantryTransRobotMat = CalculateTransformationMatrix(TargetPoints, BasePoints);
	RobotTransGantryMat = CalculateTransformationMatrix(BasePoints, TargetPoints);
	return  true;
}
std::vector<CvPoint3D64f> CUnit::TransCoor_Gantry2RobotNew(std::vector<CvPoint3D64f>& BasePoints)
{
	int nDir = 0 == GetRobotCtrl()->m_nRobotNo % 2 ? -1 : 1;
	for (int i = 0; i < BasePoints.size(); i++)
	{
		double dAxisY = BasePoints[i].y;
		BasePoints[i].y = 0.0;
		CvPoint3D64f rePoint = TransCoor_Gantry2Robot(BasePoints[i]);
		rePoint.x += dAxisY * nDir;
		BasePoints[i] = rePoint;
	}
	return BasePoints;
}

std::vector<CvPoint3D64f> CUnit::TransCoor_Gantry2Robot(std::vector<CvPoint3D64f>& BasePoints)
{
	return CoordinateTransformate(BasePoints, m_GantryTransRobotMat);
}

std::vector<CvPoint3D64f> CUnit::TransCoor_Robot2Gantry(std::vector<CvPoint3D64f>& BasePoints)
{
	return CoordinateTransformate(BasePoints, m_RobotTransGantryMat);
}

CvPoint3D64f CUnit::TransCoor_Gantry2RobotNew(CvPoint3D64f& BasePoint)
{
	int nDir = 0 == GetRobotCtrl()->m_nRobotNo % 2 ? -1 : 1;
	double dAxisY = BasePoint.y;
	//BasePoint.y = 0;
	std::vector<CvPoint3D64f> tempPoints;
	tempPoints.push_back(BasePoint);
	std::vector<CvPoint3D64f> resultPoints = CoordinateTransformate(tempPoints, m_GantryTransRobotMat);
	//resultPoints[0].x += dAxisY * nDir;
	return resultPoints[0];
}

CvPoint3D64f CUnit::TransCoor_Gantry2Robot(CvPoint3D64f& BasePoint)
{
	std::vector<CvPoint3D64f> tempPoints;
	tempPoints.push_back(BasePoint);
	std::vector<CvPoint3D64f> resultPoints = CoordinateTransformate(tempPoints, m_GantryTransRobotMat);
	return resultPoints[0];
}

CvPoint3D64f CUnit::TransCoor_Robot2Gantry(CvPoint3D64f& BasePoint)
{
	std::vector<CvPoint3D64f> tempPoints;
	tempPoints.push_back(BasePoint);
	std::vector<CvPoint3D64f> resultPoints = CoordinateTransformate(tempPoints, m_RobotTransGantryMat);
	return resultPoints[0];
}

std::vector<CvPoint3D64f> CUnit::TransNorVector_Gantry2Robot(std::vector<CvPoint3D64f>& BasePoints)
{
	return CoordinateTransformateNormal(BasePoints, m_GantryTransRobotMat);
}

std::vector<CvPoint3D64f> CUnit::TransNorVector_Robot2Gantry(std::vector<CvPoint3D64f>& BasePoints)
{
	return CoordinateTransformateNormal(BasePoints, m_RobotTransGantryMat);
}

CvPoint3D64f CUnit::TransNorVector_Gantry2Robot(CvPoint3D64f& BasePoint)
{
	std::vector<CvPoint3D64f> tempPoints;
	tempPoints.push_back(BasePoint);
	std::vector<CvPoint3D64f> resultPoints = CoordinateTransformateNormal(tempPoints, m_GantryTransRobotMat);
	return resultPoints[0];
}

CvPoint3D64f CUnit::TransNorVector_Robot2Gantry(CvPoint3D64f& BasePoint)
{
	std::vector<CvPoint3D64f> tempPoints;
	tempPoints.push_back(BasePoint);
	std::vector<CvPoint3D64f> resultPoints = CoordinateTransformateNormal(tempPoints, m_RobotTransGantryMat);
	return resultPoints[0];
}

double CUnit::TransNorAngle_Gantry2Robot(double dNorAngle)
{
	CvPoint3D64f BasePointNor = { CosD(dNorAngle),SinD(dNorAngle) ,0.0 };
	CvPoint3D64f TargetPointNor = TransNorVector_Gantry2Robot(BasePointNor);
	dNorAngle = atan2(TargetPointNor.y, TargetPointNor.x) * 180 / 3.1415926;
	return dNorAngle;
}

void CUnit::LoadExAixsFun()
{
	m_nLinedScanAxisNo = -1;
	m_nMeasureAxisNo = -1;
	m_nTrackAxisNo = -1;
	m_nMeasureAxisNo_up = -1;
	COPini opini;
	CString strFile = DATA_PATH + GetRobotCtrl()->m_strRobotName + ROBOT_PARA_INI;
	opini.SetFileName(strFile);
	opini.SetSectionName("ExternalAxleFuncation");
	opini.ReadString("LineScanAxis", &m_nLinedScanAxisNo);
	opini.ReadString("MeasureAxis",  &m_nMeasureAxisNo);
	opini.ReadString("MeasureAxis_up", &m_nMeasureAxisNo_up);
	opini.ReadString("TrackingAxis", &m_nTrackAxisNo);
}

void CUnit::CreatTrackFilteObject()
{
	int RobotHandle = MoveHandle_MakeObject();
	int RobotSmooth = TrackSmooth_MakeObject();
	m_nRobotSmooth = TrackFilter_MakeObject();
	//! 设置配置参数
	TrackFilter_SafetyParameters TrackFilterParam;
	TrackFilterParam.toImpPntMinSafetyDis = 25.0;// 输入点间距最大阈值
	TrackFilterParam.looseSafetyZ = 10.0; // 宽松条件下点间Z最大阈值
	TrackFilterParam.strictSafetyZ = 8.0; // 严格条件下点间Z最大阈值
	TrackFilter_SetSafetyParameters(m_nRobotSmooth, TrackFilterParam);
}

bool CUnit::SwingTracking(CRobotDriverAdaptor* pRobotDriver, std::vector<T_ROBOT_COORS> vtRobotCoors, double dSpeed)
{
	if (!m_bSwingState) {
		return true;
	}
	//1.准备参数 
	// 先分离机器人坐标和外部轴坐标 然后需要将外部轴和机器人分别走到起点 分解焊接速度确定外部轴和机器人的速度 合并参数到焊接结构体中
	// 外部轴由于一直匀速运动所以不用合并参数 但是需要记录世界坐标的外部轴坐标(测试和错误处理使用) 
	std::vector<double> vdAxisRealCoors; //外部轴坐标
	std::vector<T_ROBOT_COORS> vtRobotRealCoors; //机器人坐标
	//m_pRobotDriver = pRobotDriver;
	if (vtRobotCoors.size() == 0) {
		//已修改
		XUI::MesBox::PopInfo("SwingTracking接收到的轨迹为空！！！");
		//XiMessageBox("SwingTracking接收到的轨迹为空！！！");
		WriteLog("SwingTracking接收到的轨迹为空！！！");
		return false;
	}
		
	//(1) 分离坐标
	for (int i = 0; i < vtRobotCoors.size(); i++) {
		T_ROBOT_COORS tempRobotCoors = vtRobotCoors[i];
#ifdef SINGLE_ROBOT
		vdAxisRealCoors.push_back(vtRobotCoors[i].dBY);
		tempRobotCoors.dBY = 0;
		vtRobotRealCoors.push_back(tempRobotCoors);
#else

		vdAxisRealCoors.push_back(vtRobotCoors[i].dBX);
		tempRobotCoors.dBX = 0;
		vtRobotRealCoors.push_back(tempRobotCoors);
#endif		
	}
	//(2) 获取起点终点
	SaveSEPtn(vtRobotCoors[0], vtRobotCoors[vtRobotCoors.size()-1]);
	//(3) 分解速度
	double dRobotSpeed = 0;
	double dAxisSpeed = 0;
	DecomposeSpeed(m_swingTrackWeldParam.startPtn, m_swingTrackWeldParam.endPtn, dSpeed, dAxisSpeed, dRobotSpeed);
	//1. 计算速度 v=2π×频率×振幅 mm/s（暂定）
	double dSwingSpeed = (m_swingTrackWeldParam.dSwingRightAmplitude + m_swingTrackWeldParam.dSwingLeftAmplitude) * 2 * m_swingTrackWeldParam.dSwingFrequency * 60;
	//2. 转换单位 mm/min
	dSwingSpeed = (dSwingSpeed / 6);
	if (0 == dSwingSpeed)
	{
		dSwingSpeed = 100;
	}
	m_swingTrackWeldParam.dRobotWeldSpeed = dSwingSpeed;

	//(4)初始化部分焊接结构体参数
	m_swingTrackWeldParam.vtWorldCoors = vtRobotCoors; //合并未处理的焊道世界坐标
	m_swingTrackWeldParam.vtWeldRealCoors = vtRobotRealCoors; //合并机器人焊道
	m_swingTrackWeldParam.vdAxisCoors = vdAxisRealCoors; //合并外部轴焊道
														 //m_swingTrackWeldParam.dRobotWeldSpeed = dRobotSpeed; //合并机器人速度
	m_swingTrackWeldParam.dAxisWeldSpeed = dSpeed / 60; //合并外部轴速度 （大车单抽运动 速度单位 mm/s 此处除60）
	m_swingTrackWeldParam.vtWeldPara;  //合并工艺参数
									   //WriteSwingParam();			//写入频率 左摆幅 右摆幅
									   //2. 启动控制线程
	clock_t testTime = clock();
	m_bSwingState = false;
	AfxBeginThread(ThreadMainContData, this);
	WriteLog("\n已经启动主控线程 时间:%d\n", clock() - testTime);
	return true;
}
void CUnit::SeparateAndSaveWeldingPath(std::vector<T_ROBOT_COORS> vtWorldCoors)
{
	//1. 定义单一元素
	T_ROBOT_COORS tRobotCoors; // 机器人分离坐标
	double dAxisCoors;  //外部轴坐标
	std::vector<T_ROBOT_COORS> vtRobotCoors; //分离出的机器人轨迹
	std::vector<double> vdAxisCoors; //分离出的外部轴轨迹

									 //2. 看运动方向分离坐标
#ifdef SINGLE_ROBOT
	for (int i = 0; i < vtWorldCoors.size(); i++) {
		dAxisCoors = vtWorldCoors[i].dY;
		vtWorldCoors[i].dY = 0;
		tRobotCoors = vtWorldCoors[i];
		vtRobotCoors.push_back(tRobotCoors);
		vdAxisCoors.push_back(dAxisCoors);
	}
#else
	for (int i = 0; i < vtWorldCoors.size(); i++) {
		dAxisCoors = vtWorldCoors[i].dBX;
		vtWorldCoors[i].dBX = 0;
		tRobotCoors = vtWorldCoors[i];
		vtRobotCoors.push_back(tRobotCoors);
		vdAxisCoors.push_back(dAxisCoors);
	}

#endif		
	//3. 保存分离坐标
	m_swingTrackWeldParam.vdAxisCoors.insert(m_swingTrackWeldParam.vdAxisCoors.end(), vdAxisCoors.begin(), vdAxisCoors.end());
	m_swingTrackWeldParam.vtWeldRealCoors.insert(m_swingTrackWeldParam.vtWeldRealCoors.end(), vtRobotCoors.begin(), vtRobotCoors.end());
}
void CUnit::SeparateAndSaveWeldingPath(T_ROBOT_COORS tWorldCoors)
{
	//1. 定义单一元素
	T_ROBOT_COORS tRobotCoors; // 机器人分离坐标
	double dAxisCoors;  //外部轴坐标
						//2. 看运动方向分离坐标
#ifdef SINGLE_ROBOT
	dAxisCoors = tWorldCoors.dY;
	tWorldCoors.dY = 0;
	tRobotCoors = tWorldCoors;
#else
	dAxisCoors = tWorldCoors.dBX;
	tWorldCoors.dX = 0;
	tRobotCoors = tWorldCoors;
#endif
	//3. 保存分离坐标
	m_swingTrackWeldParam.vdAxisCoors.push_back(dAxisCoors);
	m_swingTrackWeldParam.vtWeldRealCoors.push_back(tRobotCoors);
}
void CUnit::SaveSEPtn(T_ROBOT_COORS tStartPtn, T_ROBOT_COORS tEndPtn)
{
	m_swingTrackWeldParam.startPtn = tStartPtn;
	m_swingTrackWeldParam.endPtn = tEndPtn;
}
bool CUnit::SendWeldData()
{
	/*
	I51记录已走过点数

	I2-I10记录焊接信息\
	焊接速度
	起弧电流
	起弧电压
	焊接电流
	焊接电压
	收弧电流
	收弧电压
	*/
	CRobotDriverAdaptor* pRobotDriver = GetRobotCtrl();
	//高速发送准备
	MP_USR_VAR_INFO tUsrVarInfo;
	vector<MP_USR_VAR_INFO> vtUsrVarInfo;
	//准备速度
	tUsrVarInfo = pRobotDriver->PrepareValData(99, m_swingTrackWeldParam.dRobotWeldSpeed);
	vtUsrVarInfo.push_back(tUsrVarInfo);
	//准备焊接参数（先不写）

	//发送终点(给机器人一个开始的运动方向)
	//准备摆动坐标结构体
	WriteSwing(m_swingTrackWeldParam.vtWeldRealCoors[0]);

	// 临时 区分测量时不起弧 不停留 不摆动
	if ((0 == m_swingTrackWeldParam.dSwingLeftAmplitude) && (0 == m_swingTrackWeldParam.dSwingRightAmplitude)&&(0== m_swingTrackWeldParam.SwingNumber))// 测量
	{
		//是否起弧
		tUsrVarInfo = pRobotDriver->PrepareValData(58, 0);
		vtUsrVarInfo.push_back(tUsrVarInfo);
		// 摆动停留时间
		tUsrVarInfo = pRobotDriver->PrepareValData(61, 0);
		vtUsrVarInfo.push_back(tUsrVarInfo);
		// 发送点
		tUsrVarInfo = pRobotDriver->PrepareValData(101, m_swingTrackWeldParam.swingTrackPoint.tLeftPoint);
		vtUsrVarInfo.push_back(tUsrVarInfo);
		tUsrVarInfo = pRobotDriver->PrepareValData(102, m_swingTrackWeldParam.swingTrackPoint.tMiddlePoint);
		vtUsrVarInfo.push_back(tUsrVarInfo);
		tUsrVarInfo = pRobotDriver->PrepareValData(103, m_swingTrackWeldParam.swingTrackPoint.tRightPoint);
		vtUsrVarInfo.push_back(tUsrVarInfo);
		pRobotDriver->SetMultiVar_H(vtUsrVarInfo);
	}
	else // 焊接
	{
		// 临时 不起弧
		tUsrVarInfo = pRobotDriver->PrepareValData(58, 0);
		vtUsrVarInfo.push_back(tUsrVarInfo);
		// 摆动停留时间
		tUsrVarInfo = pRobotDriver->PrepareValData(61, m_swingTrackWeldParam.dSwingtime*100);
		vtUsrVarInfo.push_back(tUsrVarInfo);
		// 发送点
		tUsrVarInfo = pRobotDriver->PrepareValData(101, m_swingTrackWeldParam.swingTrackPoint.tLeftPoint);
		vtUsrVarInfo.push_back(tUsrVarInfo);
		tUsrVarInfo = pRobotDriver->PrepareValData(102, m_swingTrackWeldParam.swingTrackPoint.tMiddlePoint);
		vtUsrVarInfo.push_back(tUsrVarInfo);
		tUsrVarInfo = pRobotDriver->PrepareValData(103, m_swingTrackWeldParam.swingTrackPoint.tRightPoint);
		vtUsrVarInfo.push_back(tUsrVarInfo);
		pRobotDriver->SetMultiVar_H(vtUsrVarInfo);
	}
	// *********** 加载焊接工艺参数 ***********
	/*vector<T_WELD_PARA> vtWeldPara;
	T_WELD_PARA tWeldPara;
	E_WELD_SEAM_TYPE eWeldSeamType = E_FLAT_SEAM;
	if (false == GetCurWeldParam(eWeldSeamType, vtWeldPara))
	{
		XiMessageBox("跟踪：加载焊接工艺参数失败！");
		return false;
	}
	SendWeldParam(0, vtWeldPara[0]);*/
	return true;
}

void CUnit::LoadWeldParam(int SwingNumber, int nLayerNo)
{
	SwingTrackWeldParam tWeldPara;

	//int SwingNumber;                     //摆动条件号
	//int nLayerNo;                        //层号
	//double dSwingFrequency;              //摆动频率
	//double dSwingLeftAmplitude;          //摆动左振幅
	//double dSwingRightAmplitude;         //摆动右振幅
	//double dSwingtime;                   //摆动停留时间

	//double dSwingOffsetLayerNo;          //摆动各层偏移量 暂时不用
	//bool bSwingMode;                     //摆动方式       暂时不用

	// 平焊参数
	CString sFileName = DATA_PATH + "RobotA" + WELD_GROOVE_FILE;
	if (0 == CheckFileExists(sFileName, true))
	{
		//已修改
		XUI::MesBox::PopInfo("{0}文件不存在,请检查", sFileName.GetBuffer());
		//XiMessageBoxGroup(1, "%s文件不存在,请检查", sFileName.GetBuffer());
	}
	FILE* ReadWeldGrooveFlat;
	ReadWeldGrooveFlat = fopen(sFileName.GetBuffer(), "r");
	int nReadRst = -1;
	int nDataNum = 6; // 数量个数
	while (EOF != (nReadRst = fscanf(ReadWeldGrooveFlat, "%d %d %lf %lf %lf %lf",
		&tWeldPara.SwingNumber, &tWeldPara.nLayerNo, &tWeldPara.dSwingFrequency,
		&tWeldPara.dSwingLeftAmplitude, &tWeldPara.dSwingRightAmplitude,&tWeldPara.dSwingtime)))
	{
		if (nDataNum != nReadRst)
		{
			//已修改
			XUI::MesBox::PopInfo("检测到不完整坡口焊工艺参数，加载失败！");
			//XiMessageBox("检测到不完整坡口焊工艺参数，加载失败！");
			return;
		}
		if ((SwingNumber == tWeldPara.SwingNumber) && (nLayerNo == tWeldPara.nLayerNo))
		{
			m_swingTrackWeldParam.dSwingFrequency= tWeldPara.dSwingFrequency;
			m_swingTrackWeldParam.dSwingLeftAmplitude = tWeldPara.dSwingLeftAmplitude;
			m_swingTrackWeldParam.dSwingRightAmplitude = tWeldPara.dSwingRightAmplitude;
			m_swingTrackWeldParam.dSwingtime = tWeldPara.dSwingtime;
		}
	}
	fclose(ReadWeldGrooveFlat);
	return;
}
void CUnit::SendCoors()
{
	//获取外部轴坐标
	CRobotDriverAdaptor* pRobotDriver = GetRobotCtrl();
	double newPos = GetPositionDis();
	//寻找对应的机器人坐标的下标
	int index = 0;
	int nDir = 0;
	double dDis = m_swingTrackWeldParam.vdAxisCoors[m_swingTrackWeldParam.vdAxisCoors.size() - 1] - m_swingTrackWeldParam.vdAxisCoors[0];
	if (dDis > 0) {
		nDir = -1;
	}
	else {
		nDir = 1;
	}
	for (int i = 0; i < m_swingTrackWeldParam.vdAxisCoors.size(); i++) {
		if ((newPos*nDir) >(m_swingTrackWeldParam.vdAxisCoors[i] * nDir)) {
			index = i;
			break;
		}
	}
	//准备摆动坐标结构体
	WriteSwing(m_swingTrackWeldParam.vtWeldRealCoors[index]);

	//高速发送准备
	MP_USR_VAR_INFO tUsrVarInfo;
	vector<MP_USR_VAR_INFO> vtUsrVarInfo;
	//准备坐标
	tUsrVarInfo = pRobotDriver->PrepareValData(101, m_swingTrackWeldParam.swingTrackPoint.tLeftPoint);
	vtUsrVarInfo.push_back(tUsrVarInfo);
	tUsrVarInfo = pRobotDriver->PrepareValData(102, m_swingTrackWeldParam.swingTrackPoint.tMiddlePoint);
	vtUsrVarInfo.push_back(tUsrVarInfo);
	tUsrVarInfo = pRobotDriver->PrepareValData(103, m_swingTrackWeldParam.swingTrackPoint.tRightPoint);
	vtUsrVarInfo.push_back(tUsrVarInfo);
	//********** 用于先测后焊记录步号 ***********
	//m_pTraceModel->m_nWeldStepNo = index;
										 //pRobotDriver->m_nCurWeldStepNo = index; // 断点续焊使用

	WriteLog("正在发送第%d个坐标 发送的中间点坐标为X:%lf Y:%lf Z:%lf", index,
		m_swingTrackWeldParam.swingTrackPoint.tMiddlePoint.dX,
		m_swingTrackWeldParam.swingTrackPoint.tMiddlePoint.dY,
		m_swingTrackWeldParam.swingTrackPoint.tMiddlePoint.dZ);
	//准备机器人速度
	//tUsrVarInfo = pRobotDriver->PrepareValData(2, m_swingTrackWeldParam.dRobotWeldSpeed);
	//vtUsrVarInfo.push_back(tUsrVarInfo);
	pRobotDriver->SetMultiVar_H(vtUsrVarInfo);

	m_swingTrackWeldParam.dRobotWeldSpeed = 0;
}
void CUnit::WriteSwing(T_ROBOT_COORS tRobotCoors)
{
	//1. 计算速度 v=2π×频率×振幅 mm/s（暂定）
	double dSwingSpeed = (m_swingTrackWeldParam.dSwingRightAmplitude + m_swingTrackWeldParam.dSwingLeftAmplitude) * 2 * m_swingTrackWeldParam.dSwingFrequency * 60;
	//2. 转换单位 mm/min
	dSwingSpeed = dSwingSpeed / 6;
	//3. 计算各个分量需要加减的值
	//左点
	double dXLeftDis = m_swingTrackWeldParam.dSwingLeftAmplitude * m_swingTrackWeldParam.norm[0];
	double dYLeftDis = m_swingTrackWeldParam.dSwingLeftAmplitude * m_swingTrackWeldParam.norm[1];
	double dZLeftDis = m_swingTrackWeldParam.dSwingLeftAmplitude * m_swingTrackWeldParam.norm[2];
	//右点
	double dXRightDis = m_swingTrackWeldParam.dSwingRightAmplitude * m_swingTrackWeldParam.norm[0];
	double dYRightDis = m_swingTrackWeldParam.dSwingRightAmplitude * m_swingTrackWeldParam.norm[1];
	double dZRightDis = m_swingTrackWeldParam.dSwingRightAmplitude * m_swingTrackWeldParam.norm[2];
	//4. 增加偏移量
	//COPini opini;
	//opini.SetFileName("./Data/WeldPoint");
	//double FlatWeldHorComp0 = 0;
	//double FlatWeldHeightComp0 = 0;
	//double FlatWeldHorComp90 = 0;
	//double FlatWeldHeightComp90 = 0;
	//double FlatWeldHorComp180 = 0;
	//double FlatWeldHeightComp180 = 0;
	//double FlatWeldHorComp270 = 0;
	//double FlatWeldHeightComp270 = 0;


	//opini.SetSectionName("WeldCompVa0");
	//opini.ReadString("FlatWeldHorComp", &FlatWeldHorComp0);
	//opini.ReadString("FlatWeldHeightComp", &FlatWeldHeightComp0);
	//opini.SetSectionName("WeldCompVa90");
	//opini.ReadString("FlatWeldHorComp", &FlatWeldHorComp90);
	//opini.ReadString("FlatWeldHeightComp", &FlatWeldHeightComp90);
	//opini.SetSectionName("WeldCompVa180");
	//opini.ReadString("FlatWeldHorComp", &FlatWeldHorComp180);
	//opini.ReadString("FlatWeldHeightComp", &FlatWeldHeightComp180);
	//opini.SetSectionName("WeldCompVa270");
	//opini.ReadString("FlatWeldHorComp", &FlatWeldHorComp0);
	//opini.ReadString("FlatWeldHeightComp", &FlatWeldHeightComp270);


	//4. 计算摆动运动结构体
	T_ROBOT_COORS tSwingLeftPtn;
	tSwingLeftPtn.dX = tRobotCoors.dX - dXLeftDis;
	tSwingLeftPtn.dY = tRobotCoors.dY - dYLeftDis;
	tSwingLeftPtn.dZ = tRobotCoors.dZ;// + dZLeftDis;
	tSwingLeftPtn.dRX = tRobotCoors.dRX;
	tSwingLeftPtn.dRY = tRobotCoors.dRY;
	tSwingLeftPtn.dRZ = tRobotCoors.dRZ;

	T_ROBOT_COORS tSwingMiddlePtn;
	tSwingMiddlePtn = tRobotCoors;

	T_ROBOT_COORS tSwingRightPtn;
	tSwingRightPtn.dX = tRobotCoors.dX + dXRightDis;
	tSwingRightPtn.dY = tRobotCoors.dY + dYRightDis;
	tSwingRightPtn.dZ = tRobotCoors.dZ;// -dZRightDis;
	tSwingRightPtn.dRX = tRobotCoors.dRX;
	tSwingRightPtn.dRY = tRobotCoors.dRY;
	tSwingRightPtn.dRZ = tRobotCoors.dRZ;

	m_swingTrackWeldParam.swingTrackPoint.tLeftPoint = tSwingLeftPtn;
	m_swingTrackWeldParam.swingTrackPoint.tMiddlePoint = tSwingMiddlePtn;
	m_swingTrackWeldParam.swingTrackPoint.tRightPoint = tSwingRightPtn;
	//5. 计算速度 

	//m_swingTrackWeldParam.dRobotWeldSpeed = std::sqrt(m_swingTrackWeldParam.dRobotWeldSpeed * m_swingTrackWeldParam.dRobotWeldSpeed + dSwingSpeed * dSwingSpeed);
	m_swingTrackWeldParam.dRobotWeldSpeed = dSwingSpeed;
}
void CUnit::DecomposeSpeed(T_ROBOT_COORS tStartPytn, T_ROBOT_COORS tEnd, double nSpeed, double& dMainSpeed, double& dSecSpeed)
{
	//计算向量准备
	cv::Vec6d twoPoint(
		tStartPytn.dX + tStartPytn.dBX, tStartPytn.dY + tStartPytn.dBY, tStartPytn.dZ + tStartPytn.dBZ,
		tEnd.dX + tEnd.dBX, tEnd.dY + tEnd.dBY, tEnd.dZ + tEnd.dBZ);
#ifdef SINGLE_ROBOT
	cv::Vec3d norm(
		twoPoint[1] - twoPoint[4],
		twoPoint[0] - twoPoint[3],
		twoPoint[2] - twoPoint[5]
	);
#else
	cv::Vec3d norm(

		twoPoint[0] - twoPoint[3],
		twoPoint[1] - twoPoint[4],
		twoPoint[2] - twoPoint[5]
	);
#endif
	if (nSpeed == 0) {
		//已修改
		XUI::MesBox::PopInfo("输入的速度为0， 请检查输入速度！！！");
		//XiMessageBox("输入的速度为0， 请检查输入速度！！！");
		return;
	}
	//保存所计算出的向量
	double dTempDot = std::sqrt(norm[0] * norm[0] + norm[1] * norm[1] + norm[2] * norm[2]);
	m_swingTrackWeldParam.norm = norm;
	m_swingTrackWeldParam.norm[0] = fabs(m_swingTrackWeldParam.norm[0] / dTempDot);
	m_swingTrackWeldParam.norm[1] = fabs(m_swingTrackWeldParam.norm[1] / dTempDot);
	m_swingTrackWeldParam.norm[2] = fabs(m_swingTrackWeldParam.norm[2] / dTempDot);

	//计算速度的实际值
	double dot = std::sqrt(norm[0] * norm[0] + norm[1] * norm[1] + norm[2] * norm[2]) / nSpeed;
	norm[0] = fabs(norm[0] / dot);
	norm[1] = fabs(norm[1] / dot);
	norm[2] = fabs(norm[2] / dot);


	if (norm[0] < norm[1]) {
		dMainSpeed = norm[1];
		dSecSpeed = std::sqrt(norm[0] * norm[0] + norm[2] * norm[2]);
	}
	else {
		dMainSpeed = norm[0];
		dSecSpeed = std::sqrt(norm[1] * norm[1] + norm[2] * norm[2]);
	}
	return;
}
bool CUnit::isAxisToEnd()
{
	CRobotDriverAdaptor* pRobotDriver = GetRobotCtrl();
	double newPos = GetPositionDis();
	double dStart = m_swingTrackWeldParam.vdAxisCoors[0];
	double dEnd = m_swingTrackWeldParam.vdAxisCoors[m_swingTrackWeldParam.vdAxisCoors.size() - 1];
	double dWeldDir = dEnd > dStart ? 1.0 : -1.0;
	if ((newPos * dWeldDir) >= (dEnd * dWeldDir)) {
		return true;
	}
	return false;
}
void CUnit::UpdateRealRobotCoors()
{
	// 记录 m_btRealRobotCoors 的大小
	CRobotDriverAdaptor* pRobotDriver = GetRobotCtrl();
	m_swingTrackWeldParam.nFlat = m_swingTrackWeldParam.vtWorldCoors.size();
	WriteLog("结构体中世界坐标大小：%d    总世界坐标大小:%d", m_swingTrackWeldParam.vtWorldCoors.size(), pRobotDriver->m_vtWeldLineInWorldPoints.size());

	// 判断 m_vtweldlneTnWor1dPoints 的大小是否大于 nFalt
	if (pRobotDriver->m_vtWeldLineInWorldPoints.size() > m_swingTrackWeldParam.nFlat) {
		// 获取迭代器指向的位置
		auto it = pRobotDriver->m_vtWeldLineInWorldPoints.begin() + m_swingTrackWeldParam.nFlat;

		// 将大于的部分赋值 给 m_btRealRobotCoors
		m_swingTrackWeldParam.vtWorldCoors.insert(m_swingTrackWeldParam.vtWorldCoors.end(), it, pRobotDriver->m_vtWeldLineInWorldPoints.end());

		//分解新添加的世界坐标
		std::vector<T_ROBOT_COORS> temp(m_swingTrackWeldParam.vtWorldCoors.begin() + m_swingTrackWeldParam.nFlat, m_swingTrackWeldParam.vtWorldCoors.end());

		for (int i = 0; i < temp.size(); i++) {
			double dAxis = temp[i].dBY;
			temp[i].dBY = 0;
			T_ROBOT_COORS tRobotCoors = temp[i];

			m_swingTrackWeldParam.vtWeldRealCoors.push_back(tRobotCoors);
			m_swingTrackWeldParam.vdAxisCoors.push_back(dAxis);
		}
		WriteLog("整合完：结构体中世界坐标大小：%d    总世界坐标大小:%d", m_swingTrackWeldParam.vtWorldCoors.size(), pRobotDriver->m_vtWeldLineInWorldPoints.size());
	}


}
UINT CUnit::ThreadCheckSwingConsistency(void* pParam)
{
	CUnit* pDiaphragmWeld = (CUnit*)pParam;
	CRobotDriverAdaptor* pRobotDriver = pDiaphragmWeld->GetRobotCtrl();
	while (!pDiaphragmWeld->m_bSwingState) {
		pDiaphragmWeld->UpdateRealRobotCoors();
		Sleep(500);
	}
	return 0;
}
void CUnit::WriteSwingParam(double dSwingFrequency, double dSwingLeftAmplitude, double dSwingRightAmplitude)
{
	m_swingTrackWeldParam.SwingNumber=0;
	m_swingTrackWeldParam.dSwingFrequency = dSwingFrequency;
	m_swingTrackWeldParam.dSwingLeftAmplitude = dSwingLeftAmplitude;
	m_swingTrackWeldParam.dSwingRightAmplitude = dSwingRightAmplitude;
}
UINT CUnit::ThreadMainContData(void* pParam)
{
	CUnit* pDiaphragmWeld = (CUnit*)pParam;
	CRobotDriverAdaptor* pRobotDriver = pDiaphragmWeld->GetRobotCtrl();
	pRobotDriver->SetIntVar(12, 0);
	//记录时间
	clock_t startTime = clock();

	//1. 发送焊接信息
	if (false == pDiaphragmWeld->SendWeldData())
	{
		//已修改
		XUI::MesBox::PopInfo("焊接参数设置失败！");
		//XiMessageBox("焊接参数设置失败！");
		return 0;
	}

	//2. 开启外部轴
	CUnitDriver* pUnitDriver = pRobotDriver->m_pvpMotorDriver->at(0);/*pDiaphragmWeld->m_vpUnit[0]->GetRobotCtrl()->m_pvpMotorDriver->at(0)*/;
	double dDis = pDiaphragmWeld->m_swingTrackWeldParam.vdAxisCoors[pDiaphragmWeld->m_swingTrackWeldParam.vdAxisCoors.size() - 1] - pDiaphragmWeld->m_swingTrackWeldParam.vdAxisCoors[0];
	int nDir = dDis > 0.0 ? 1 : 0;
	//pDiaphragmWeld->pUnitDriver->ContiMove(nDir, pDiaphragmWeld->m_swingTrackWeldParam.dAxisWeldSpeed, 0.5);
	pUnitDriver->ContiMove(nDir, pDiaphragmWeld->m_swingTrackWeldParam.dAxisWeldSpeed, 0.5);

	//3. 开启机器人
	pRobotDriver->CallJob("SWINGTRACKING");
	int nMaxWaitTime = 3000;
	int nCurWaitTime = 0;
	while (((/*TRUE != pDiaphragmWeld->m_ptUnit->CheckInIO("Running")*/TRUE != pDiaphragmWeld->WorldIsRunning()) /*|| (pDiaphragmWeld->m_ptUnit->CheckRobotDone)*//*||*/ /*(0 != pRobotDriver->m_pMoveCtrl->CheckDone(pRobotDriver->m_nAXIS_X))*/) &&
		(nCurWaitTime < nMaxWaitTime))
	{
		nCurWaitTime += 50;
		Sleep(50);
	}

	WriteLog("\n机器人线程开始成功 时间:%d\n", clock() - pDiaphragmWeld->testTime);
	//4. 开启实施更新线程
	CWinThread* pUpdataThread = AfxBeginThread(ThreadCheckSwingConsistency, pDiaphragmWeld);
	while (1) {
		//如果没有达到终点 则发送下一个机器人坐标
		if (!pDiaphragmWeld->isAxisToEnd() &&
			(/*TRUE == pDiaphragmWeld->m_ptUnit->CheckInIO("Running")*/TRUE == pDiaphragmWeld->WorldIsRunning()) /*&&
			(0 == pRobotDriver->m_pMoveCtrl->CheckDone(pRobotDriver->m_nAXIS_X))*/) {
			pDiaphragmWeld->SendCoors();
		}
		else {
			//pDiaphragmWeld->m_pTraceModel->m_bCameraFindWeldEnd = true; // 退出跟踪线程
			pUnitDriver->DecelStop();
			pRobotDriver->SetIntVar(12, 1);
			pRobotDriver->HoldOn();
			Sleep(200);
			pRobotDriver->HoldOff();
			WriteLog("ThreadMainContData Exit!");
			break;
		}
		Sleep(200);
	}
	clock_t endTime = clock() - startTime;
	//5. 记录数据(外部轴移动的距离为焊道长度)
	dDis = pDiaphragmWeld->m_swingTrackWeldParam.vdAxisCoors[pDiaphragmWeld->m_swingTrackWeldParam.vdAxisCoors.size() - 1] - pDiaphragmWeld->m_swingTrackWeldParam.vdAxisCoors[0];
	/*CStatisticalData* pStatisticalData = CStatisticalData::getInstance();
	dDis = dDis / 1000;
	if (dDis > 0) {
	pStatisticalData->UpdateWeldData(E_FLAT_SEAM, dDis, endTime);
	}
	else {
	pStatisticalData->UpdateWeldData(E_FLAT_SEAM, -dDis, endTime);
	}

	pStatisticalData->UpDateStatisticsData();*/

	//6. 清枪
	/*if (false == pRobotDriver->CleanGunH())
	{
	XiMessageBox("清枪失败");
	}
	else
	{
	XiMessageBox("清枪完成");
	}	*/
	pDiaphragmWeld->m_bSwingState = true;
	return 0;
}

bool CUnit::SendWeldDataNew(FILE* file, const T_WELD_PARA& tWeldParam, E_WELD_SEAM_TYPE eWeldSeamType, bool& bIsCallJob, int ToolNum)
{
	WriteLog("SendWeldData Start!");
	CRobotDriverAdaptor* pRobotDriver = GetRobotCtrl();
	MP_USR_VAR_INFO tUsrVarInfo;
	vector<MP_USR_VAR_INFO> vtUsrVarInfo;
	vtUsrVarInfo.clear();
	int nJobPtnNum = 40;
	int nTotalNum = m_cWeldTrack.getCount();
	int nSendNum = 0;
	int nPreMovePtnNum = 0; // 记录上次发送轨迹时运动走过点数
	int nMovePtnNum = 0; // 焊接Job当前走过点数

	tUsrVarInfo = pRobotDriver->PrepareValData(21, nTotalNum);
	vtUsrVarInfo.push_back(tUsrVarInfo);
	tUsrVarInfo = pRobotDriver->PrepareValData(24, tWeldParam.WeldVelocity / 6);
	vtUsrVarInfo.push_back(tUsrVarInfo);
	tUsrVarInfo = pRobotDriver->PrepareValData(72, 1);
	vtUsrVarInfo.push_back(tUsrVarInfo);

	// 起点焊接方向反向0.5mm 摆动焊接接近点,再引弧功能需要在起弧指令前后添加焊道方向移动指令
	//T_ROBOT_COORS tTempCoord = RobotCoordPosOffset(vtWeldCoord[0], vtWeldCoord[1], vtWeldCoord[0], 0.5);
	double dOffsetDis = 0.5;
	T_ROBOT_COORS tTempCoord = m_cWeldTrack.getPoint(0);
	T_ROBOT_COORS tDirSPtn = m_cWeldTrack.getPoint(1);
	T_ROBOT_COORS tDirEPtn = tTempCoord;
	double dDis = TwoPointDis(tDirSPtn.dX, tDirSPtn.dY, tDirSPtn.dZ, tDirEPtn.dX, tDirEPtn.dY, tDirEPtn.dZ);
	double dDisX = tDirEPtn.dX - tDirSPtn.dX;
	double dDisY = tDirEPtn.dY - tDirSPtn.dY;
	double dDisZ = tDirEPtn.dZ - tDirSPtn.dZ;
	tTempCoord.dX += (dDisX * dOffsetDis / dDis);
	tTempCoord.dY += (dDisY * dOffsetDis / dDis);
	tTempCoord.dZ += (dDisZ * dOffsetDis / dDis);

	tUsrVarInfo = pRobotDriver->PrepareValData(19, tTempCoord, ToolNum); // 摆焊接近点
	vtUsrVarInfo.push_back(tUsrVarInfo);

	// 在引弧Arc前后方向接近点
	tTempCoord.dX += (dDisX * dOffsetDis / dDis);
	tTempCoord.dY += (dDisY * dOffsetDis / dDis);
	tTempCoord.dZ += (dDisZ * dOffsetDis / dDis);
	tUsrVarInfo = pRobotDriver->PrepareValData(18, tTempCoord, ToolNum);
	vtUsrVarInfo.push_back(tUsrVarInfo);

	if (tWeldParam.nWrapConditionNo > 0) // 摆动条件号有0 ？ 
	{
		tUsrVarInfo = pRobotDriver->PrepareValData(8, tWeldParam.nWrapConditionNo);
		vtUsrVarInfo.push_back(tUsrVarInfo);
		tUsrVarInfo = pRobotDriver->PrepareValData(59, 1);
		vtUsrVarInfo.push_back(tUsrVarInfo);

		//tUsrVarInfo = PrepareValData(20, 0); // 焊接点数复位，防止其他运动设置20影响焊接运动计数
		//vtUsrVarInfo.push_back(tUsrVarInfo);

		T_ROBOT_COORS tRefp1;
		T_ROBOT_COORS tRefp2;
		CalcSwayRefpCoord(m_cWeldTrack.getPoint(0), eWeldSeamType, tRefp1, tRefp2);
		tUsrVarInfo = pRobotDriver->PrepareValData(60, tRefp1, ToolNum);
		vtUsrVarInfo.push_back(tUsrVarInfo);
		tUsrVarInfo = pRobotDriver->PrepareValData(61, tRefp2, ToolNum);
		vtUsrVarInfo.push_back(tUsrVarInfo);
	}
	else {
		tUsrVarInfo = pRobotDriver->PrepareValData(59, 0);
		vtUsrVarInfo.push_back(tUsrVarInfo);
	}

	// 跟踪截取数据
	{
		auto vtWeldCoord = m_cWeldTrack.getTrack();
		pRobotDriver->m_vtWeldLineInWorldPoints.clear();//焊接轨迹
		pRobotDriver->m_vtWeldLineInWorldPoints.insert(pRobotDriver->m_vtWeldLineInWorldPoints.end(),
			vtWeldCoord.begin(), vtWeldCoord.end());
	}

	// 焊接运动第一圈轨迹
	long lBPValue[3];
	auto nLastSendNo = nJobPtnNum > nTotalNum ? nTotalNum : nJobPtnNum;
	if (nLastSendNo > nSendNum)
	{
		auto vtSendTrack = m_cWeldTrack.getTrack(nSendNum, nLastSendNo - 1);
		for (size_t i = 0; i < vtSendTrack.size(); i++)
		{
			tUsrVarInfo = pRobotDriver->PrepareValData((nSendNum % nJobPtnNum) + 20, vtSendTrack[i], ToolNum);
			vtUsrVarInfo.push_back(tUsrVarInfo);
			//if (m_bWeldAfterMeasureMoveExAxis)
			{
				lBPValue[0] = vtSendTrack[i].dBX * 1000;
				lBPValue[1] = vtSendTrack[i].dBY * 1000;
				lBPValue[2] = vtSendTrack[i].dBZ * 1000;
				tUsrVarInfo = pRobotDriver->PrepareValData((nSendNum % nJobPtnNum) + 20, lBPValue, ToolNum);
				vtUsrVarInfo.push_back(tUsrVarInfo);
			}
			nSendNum++;
			m_cRealTimeTrack.m_cAdjustTrack.setLastSendPnt(vtSendTrack.back(), nSendNum - 1);
		}
	}

	//while (nSendNum < nJobPtnNum && nSendNum < nTotalNum) {
	//	tUsrVarInfo = pRobotDriver->PrepareValData((nSendNum % nJobPtnNum) + 20, vtWeldCoord[nSendNum], ToolNum);
	//	vtUsrVarInfo.push_back(tUsrVarInfo);
	//	// 添加外部轴移动
	//	if (m_bWeldAfterMeasureMoveExAxis) {
	//		lBPValue[0] = vtWeldCoord[nSendNum].dBX * 1000;
	//		lBPValue[1] = vtWeldCoord[nSendNum].dBY * 1000;
	//		lBPValue[2] = vtWeldCoord[nSendNum].dBZ * 1000;
	//		tUsrVarInfo = pRobotDriver->PrepareValData((nSendNum % nJobPtnNum) + 20, lBPValue, ToolNum);
	//		vtUsrVarInfo.push_back(tUsrVarInfo);
	//	}
	//	nSendNum++;
	//}
	WriteLog("SendWeldData Fitst Time Start!");
	pRobotDriver->SetMultiVar_H(vtUsrVarInfo);
	WriteLog("SendWeldData Fitst Time Done!");
	if (false == bIsCallJob) // 有多端焊接轨迹时 只CallJob一次即可
	{
		bIsCallJob = true;

		if (E_FLAT_SEAM == eWeldSeamType)
		{
			if (m_bWeldAfterMeasureMoveExAxis) {
				pRobotDriver->CallJob("GENERALWELD");
			}
			else {
				pRobotDriver->CallJob("GENERALWELD");
			}
		}
		else
		{
			pRobotDriver->CallJob("GENERALWELD-L");
		}

		Sleep(1000); // 确保CallJob运动起来
	}

	// 循环发送轨迹
	while (true/*nSendNum < nTotalNum*/) // 有点？
	{
		auto tCurCoord = pRobotDriver->GetCurrentPos();
		fprintf_s(file, "%.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf \n",
			tCurCoord.dX, tCurCoord.dY, tCurCoord.dZ,
			tCurCoord.dRX, tCurCoord.dRY, tCurCoord.dRZ,
			tCurCoord.dBX, tCurCoord.dBY, tCurCoord.dBZ);
		if (!g_bLocalDebugMark) {
			if (false == WorldIsRunning()) // 运动中？
			{
				WriteLog("焊接过程中异常停止");
				return false;
			}
		}
		nMovePtnNum = pRobotDriver->GetIntVar(1);
		WriteLog("Read I001 = %d", nMovePtnNum);
		if (nMovePtnNum <= nPreMovePtnNum) // 没有新走过的轨迹点
		{
			Sleep(200);
			continue;
		}

		int nTemp = pRobotDriver->m_vtWeldLineInWorldPoints.size();
		if ((nMovePtnNum > nTemp) || (nMovePtnNum > nPreMovePtnNum + 20)) {
			pRobotDriver->HoldOn();
			Sleep(500);
			pRobotDriver->HoldOff();
			//已修改
			XUI::MesBox::PopInfo("焊接轨迹数量检测异常:{0}I001={1}Pre{2}", nTemp, nMovePtnNum, nPreMovePtnNum);
			//XiMessageBoxOk("焊接轨迹数量检测异常:%d I001=%d Pre%d", nTemp, nMovePtnNum, nPreMovePtnNum);
			break;
		}

		if (nMovePtnNum >= nTemp - 1) {
			pRobotDriver->m_cLog->Write("正常结束加劲板数据发送流程:%d I001=%d", nTemp, nMovePtnNum);
			break;
		}
		if (INCISEHEAD_THREAD_STATUS_STOPPED == pRobotDriver->m_eThreadStatus) {
			pRobotDriver->m_cLog->Write("急停结束加劲板数据发送流程:%d I001=%d", nTemp, nMovePtnNum);
			break;
		}
		if (nSendNum >= nTemp) {
			pRobotDriver->m_cLog->Write("焊接数据发送流程结束:Send:%d Total:%d", nSendNum, nTemp);
			break;
		}

		// 准备并发送轨迹
		vtUsrVarInfo.clear();
		int nNeedSendNum = nMovePtnNum - nPreMovePtnNum;
		int nEIdx = (nSendNum + nNeedSendNum > nTemp/*nTotalNum*/ ? nTemp/*nTotalNum*/ : nSendNum + nNeedSendNum);
		if (nSendNum < nEIdx)
		{
			auto vtSendTrack = m_cWeldTrack.getTrack(nSendNum, nEIdx - 1);
			for (size_t i = 0; i < vtSendTrack.size(); i++)
			{
				tUsrVarInfo = pRobotDriver->PrepareValData((nSendNum % nJobPtnNum) + 20, vtSendTrack[i], ToolNum);
				vtUsrVarInfo.push_back(tUsrVarInfo);
				//if (m_bWeldAfterMeasureMoveExAxis)
				{
					lBPValue[0] = vtSendTrack[i].dBX * 1000;
					lBPValue[1] = vtSendTrack[i].dBY * 1000;
					lBPValue[2] = vtSendTrack[i].dBZ * 1000;
					tUsrVarInfo = pRobotDriver->PrepareValData((nSendNum % nJobPtnNum) + 20, lBPValue, ToolNum);
					vtUsrVarInfo.push_back(tUsrVarInfo);
				}
				nSendNum++;
				m_cRealTimeTrack.m_cAdjustTrack.setLastSendPnt(vtSendTrack.back(), nSendNum - 1);
			}
		}

		//for (; nSendNum < nEIdx; nSendNum++) {
		//	pRobotDriver->m_cLog->Write("linshi nSendNum = %d nTotalNum = %d", nSendNum, nTemp);
		//	tUsrVarInfo = pRobotDriver->PrepareValData((nSendNum % nJobPtnNum) + 20, pRobotDriver->m_vtWeldLineInWorldPoints[nSendNum]/*vtWeldCoord[nSendNum]*/, ToolNum);
		//	vtUsrVarInfo.push_back(tUsrVarInfo);
		//	// 添加外部轴移动
		//	if (m_bWeldAfterMeasureMoveExAxis) {
		//		lBPValue[0] = vtWeldCoord[nSendNum].dBX * 1000;
		//		lBPValue[1] = vtWeldCoord[nSendNum].dBY * 1000;
		//		lBPValue[2] = vtWeldCoord[nSendNum].dBZ * 1000;
		//		tUsrVarInfo = pRobotDriver->PrepareValData((nSendNum % nJobPtnNum) + 20, lBPValue, ToolNum);
		//		vtUsrVarInfo.push_back(tUsrVarInfo);
		//	}

		//}
		pRobotDriver->m_cLog->Write("nPreMovePtnNum = %d nMovePtnNum = %d nNeedSendNum = %d nSendNum = %d nTotalNum = %d %d",
			nPreMovePtnNum, nMovePtnNum, nNeedSendNum, nSendNum, nTotalNum, pRobotDriver->m_vtWeldLineInWorldPoints.size());
		pRobotDriver->SetMultiVar_H(vtUsrVarInfo);
		nPreMovePtnNum = nMovePtnNum;

		// 发送轨迹后：不跟踪 && 轨迹点发完 ？ 退出
		if (nSendNum >= nTotalNum) {
			pRobotDriver->m_cLog->Write("不跟踪轨迹发送完成 退出！");
			break;
		}
		DoEvent();
		Sleep(100);
	}

	pRobotDriver->m_cLog->Write("结束数据发送:%d ", pRobotDriver->m_vtWeldLineInWorldPoints.size());
	return true;
}

bool CUnit::LockForNewRTTA(IplImage* pShowImage)
{
	//只设置初始值，斜率在跟踪时初始化
	WidgetLaserInfo* pWidgetLaserInfoNew = new WidgetLaserInfo;
	pWidgetLaserInfoNew->crossFilp = false;
	pWidgetLaserInfoNew->assembleFilp = false;
	m_cRealTimeTrack.setLaserLockResult(*pWidgetLaserInfoNew);//更新锁定参数
	delete pWidgetLaserInfoNew;
	return true;

	//采图
	auto pDHCam = (CDHGigeImageCapture*)GetCameraCtrl(m_nTrackCameraNo);
	if (!pDHCam->CaptureImage(1, 2000))
	{
		pDHCam->ShowErrorString();
		return false;
	}

	CString strImg;
	strImg.Format("D:\\XiRobotSW\\LocalFiles\\OutputFiles\\RobotA\\Pic\\lock.bin");
	char sImgFile[255];
	strcpy(sImgFile, strImg);
	BinaryImgSaveAsynchronous(sImgFile, pDHCam->m_tLatestImage.pImage, 0);

	//获取当前位置
	auto tCurRobotCoord = m_pYasakawaRobotDriver->GetCurrentPos();
	auto tCurRobotPulse = m_pYasakawaRobotDriver->GetCurrentPulse();

	//锁定
	CvPoint cpMidKeyPoint;//交点
	WidgetLaserInfo* pWidgetLaserInfo = new WidgetLaserInfo[10];
	int nRst = WidgetLaserLock(pDHCam->m_tLatestImage.pImage, pWidgetLaserInfo, false, false, "WidgetLaserLock");

	//寻找符合要求的交点
	int nFindNo = -1;//符合要求的交点
	double dMinDis = 20.0;//最近点的距离
	for (int i = 0; i < nRst; i++)
	{
		//计算交点三维坐标
		T_ABS_POS_IN_BASE tPointAbsCoordInBase;
		T_ROBOT_COORS tAimCoord = TranImageToBase(
			m_nTrackCameraNo, pWidgetLaserInfo[i].crossPoint,
			tCurRobotCoord, tCurRobotPulse,
			&tPointAbsCoordInBase);

		//找到最近的坐标
		T_ROBOT_COORS tNearestCoord;
		if (m_cWeldTrack.getNearestCoordFast(tNearestCoord, tAimCoord, dMinDis, 0))
		{
			double dTemp = SQUARE(tNearestCoord.dX + tNearestCoord.dBX - tAimCoord.dX - tAimCoord.dBX)
				+ SQUARE(tNearestCoord.dY + tNearestCoord.dBY - tAimCoord.dY - tAimCoord.dBY)
				+ SQUARE(tNearestCoord.dZ + tNearestCoord.dBZ - tAimCoord.dZ - tAimCoord.dBZ);
			dMinDis = sqrt(dTemp);//更新最近距离
			cpMidKeyPoint = pWidgetLaserInfo[i].crossPoint;//更新交点
			m_cRealTimeTrack.setLaserLockResult(pWidgetLaserInfo[i]);//更新锁定参数
			nFindNo = i;//更新序号
		}
	}
	DELETE_POINTER_ARRAY(pWidgetLaserInfo);

	//没找到合适的交点
	if (nRst <= 0 || nFindNo < 0)
	{
		//已修改
		XUI::MesBox::PopInfo("初始对点！(错误代码：找不到关键点");
		//XiMessageBoxOk("初始对点！(错误代码：找不到关键点)");
		CTime cTime;
		cTime = CTime::GetCurrentTime();
		int nCurrentHour = cTime.GetHour();
		int nCurrentMinute = cTime.GetMinute();
		int nCurrentSecond = cTime.GetSecond();
		CString strFile;
		strFile.Format("%s%s\\LockError\\%d-%d-%d-%d-%d-%d.jpg", OUTPUT_PATH, GetRobotCtrl()->m_strRobotName, cTime.GetYear(), cTime.GetMonth(), cTime.GetDay(), nCurrentHour, nCurrentMinute, nCurrentSecond);
		SaveImage(pDHCam->m_tLatestImage.pImage, strFile.GetBuffer());
		return false;
	}

	//保存锁定数据
	SaveImage(pDHCam->m_tLatestImage.pImage, OUTPUT_PATH + GetUnitName() + SEARCH_SCANLOCK_IMG);

	//显示图片
	if (pShowImage)
	{
		cvCvtColor(pDHCam->m_tLatestImage.pImage, pShowImage, CV_GRAY2RGB);
		cvCircle(pShowImage, cpMidKeyPoint, 8, CV_RGB(255, 0, 0), 4);
	}

	return true;
}
