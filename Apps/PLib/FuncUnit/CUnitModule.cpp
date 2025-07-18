#include "stdafx.h"
#include "CUnitModule.h"

CUnitModule::CUnitModule(T_CONTRAL_UNIT tCtrlUnitInfo, CServoMotorDriver* pServoMotorDriver /*= NULL*/)
#if ENABLE_UNIT_DEBUG
	:CUnit_Debug(tCtrlUnitInfo, pServoMotorDriver)
#else
	:CUnit(tCtrlUnitInfo, pServoMotorDriver)
#endif // ENABLE_UNIT_DEBUG
{
	InitialTransMatrix(m_dBaseToExternalTransMatrix, m_dExternalToBaseTransMatrix);
	m_vvtRecognitionParaA.resize(7);
}

CUnitModule::~CUnitModule()
{
}

CString CUnitModule::GetNode()
{
	return m_tContralUnit.strChineseName;
}

void GetBaseToWorldTransMatrix(double dXExternalAxisPos, double dYExternalAxisPos, double dZExternalAxisPos, double dBaseToExternalTransMatrix[][4], double dBaseToWorldTransMatrix[][4])
{
	dBaseToWorldTransMatrix[0][0] = dBaseToExternalTransMatrix[0][0]; dBaseToWorldTransMatrix[0][1] = dBaseToExternalTransMatrix[0][1]; dBaseToWorldTransMatrix[0][2] = dBaseToExternalTransMatrix[0][2]; dBaseToWorldTransMatrix[0][3] = dXExternalAxisPos;
	dBaseToWorldTransMatrix[1][0] = dBaseToExternalTransMatrix[1][0]; dBaseToWorldTransMatrix[1][1] = dBaseToExternalTransMatrix[1][1]; dBaseToWorldTransMatrix[1][2] = dBaseToExternalTransMatrix[1][2]; dBaseToWorldTransMatrix[1][3] = dYExternalAxisPos;
	dBaseToWorldTransMatrix[2][0] = dBaseToExternalTransMatrix[2][0]; dBaseToWorldTransMatrix[2][1] = dBaseToExternalTransMatrix[2][1]; dBaseToWorldTransMatrix[2][2] = dBaseToExternalTransMatrix[2][2]; dBaseToWorldTransMatrix[2][3] = dZExternalAxisPos;
	dBaseToWorldTransMatrix[3][0] = dBaseToExternalTransMatrix[3][0]; dBaseToWorldTransMatrix[3][1] = dBaseToExternalTransMatrix[3][1]; dBaseToWorldTransMatrix[3][2] = dBaseToExternalTransMatrix[3][2]; dBaseToWorldTransMatrix[3][3] = dBaseToExternalTransMatrix[3][3];
}
void GetWorldToBaseTransMatrix(double dXExternalAxisPos, double dYExternalAxisPos, double dZExternalAxisPos, double dExternalToBaseTransMatrix[][4], double dWorldToBaseTransMatrix[][4])
{
	dWorldToBaseTransMatrix[0][0] = dExternalToBaseTransMatrix[0][0]; dWorldToBaseTransMatrix[0][1] = dExternalToBaseTransMatrix[0][1]; dWorldToBaseTransMatrix[0][2] = dExternalToBaseTransMatrix[0][2]; dWorldToBaseTransMatrix[0][3] = -dXExternalAxisPos;
	dWorldToBaseTransMatrix[1][0] = dExternalToBaseTransMatrix[1][0]; dWorldToBaseTransMatrix[1][1] = dExternalToBaseTransMatrix[1][1]; dWorldToBaseTransMatrix[1][2] = dExternalToBaseTransMatrix[1][2]; dWorldToBaseTransMatrix[1][3] = -dYExternalAxisPos;
	dWorldToBaseTransMatrix[2][0] = dExternalToBaseTransMatrix[2][0]; dWorldToBaseTransMatrix[2][1] = dExternalToBaseTransMatrix[2][1]; dWorldToBaseTransMatrix[2][2] = dExternalToBaseTransMatrix[2][2]; dWorldToBaseTransMatrix[2][3] = -dZExternalAxisPos;
	dWorldToBaseTransMatrix[3][0] = dExternalToBaseTransMatrix[3][0]; dWorldToBaseTransMatrix[3][1] = dExternalToBaseTransMatrix[3][1]; dWorldToBaseTransMatrix[3][2] = dExternalToBaseTransMatrix[3][2]; dWorldToBaseTransMatrix[3][3] = dExternalToBaseTransMatrix[3][3];
}
void ProcessExternalAxisValue(T_ROBOT_COORS_IN_WORLD tRobotCoorsInWorld, int nExternalType, double& dExternalX, double& dExternalY, double& dExternalZ)
{
	dExternalX = (nExternalType & 0x01) * (tRobotCoorsInWorld.dX);
	dExternalY = ((nExternalType >> 1) & 0x01) * (tRobotCoorsInWorld.dY);
	dExternalZ = ((nExternalType >> 2) & 0x01) * (tRobotCoorsInWorld.dZ);
}

void CUnitModule::InitialTransMatrix(double dBaseToExternalTransMatrix[][4], double dExternalToBaseTransMatrix[][4])
{
	dBaseToExternalTransMatrix[0][0] = 1.0; dBaseToExternalTransMatrix[0][1] = 0.0; dBaseToExternalTransMatrix[0][2] = 0.0; dBaseToExternalTransMatrix[0][3] = 0.0;
	dBaseToExternalTransMatrix[1][0] = 0.0; dBaseToExternalTransMatrix[1][1] = 1.0; dBaseToExternalTransMatrix[1][2] = 0.0; dBaseToExternalTransMatrix[1][3] = 0.0;
	dBaseToExternalTransMatrix[2][0] = 0.0; dBaseToExternalTransMatrix[2][1] = 0.0; dBaseToExternalTransMatrix[2][2] = 1.0; dBaseToExternalTransMatrix[2][3] = 0.0;
	dBaseToExternalTransMatrix[3][0] = 0.0; dBaseToExternalTransMatrix[3][1] = 0.0; dBaseToExternalTransMatrix[3][2] = 0.0; dBaseToExternalTransMatrix[3][3] = 1.0;

	dExternalToBaseTransMatrix[0][0] = 1.0; dExternalToBaseTransMatrix[0][1] = 0.0; dExternalToBaseTransMatrix[0][2] = 0.0; dExternalToBaseTransMatrix[0][3] = 0.0;
	dExternalToBaseTransMatrix[1][0] = 0.0; dExternalToBaseTransMatrix[1][1] = 1.0; dExternalToBaseTransMatrix[1][2] = 0.0; dExternalToBaseTransMatrix[1][3] = 0.0;
	dExternalToBaseTransMatrix[2][0] = 0.0; dExternalToBaseTransMatrix[2][1] = 0.0; dExternalToBaseTransMatrix[2][2] = 1.0; dExternalToBaseTransMatrix[2][3] = 0.0;
	dExternalToBaseTransMatrix[3][0] = 0.0; dExternalToBaseTransMatrix[3][1] = 0.0; dExternalToBaseTransMatrix[3][2] = 0.0; dExternalToBaseTransMatrix[3][3] = 1.0;
}

void CUnitModule::CoorTransform(std::vector<T_POINT_3D>& vtPoint, T_POINT_3D tCenter, double dRotAngle)
{
	int nIdx = 0;
	for (nIdx = 0; nIdx < vtPoint.size(); nIdx++)
	{
		vtPoint[nIdx].x += tCenter.x;
		vtPoint[nIdx].y += tCenter.y;
		vtPoint[nIdx].z += tCenter.z;
	}

	// Rotation
	for (nIdx = 0; nIdx < vtPoint.size(); nIdx++)
	{
		T_POINT_3D tPoint;
		tPoint.x = (vtPoint[nIdx].x * cos(dRotAngle * PI / 180.0)) -
			(vtPoint[nIdx].y * sin(dRotAngle * PI / 180.0));
		tPoint.y = (vtPoint[nIdx].x * sin(dRotAngle * PI / 180.0)) +
			(vtPoint[nIdx].y * cos(dRotAngle * PI / 180.0));
		tPoint.z = vtPoint[nIdx].z;

		vtPoint[nIdx] = tPoint;
	}
}

bool CUnitModule::LaserScanGetPointCouid(T_ROBOT_COORS tDiffCoor, CString strTableName, CString strTemplateNo, CString strPath, CString& strPointCouldPath, std::vector<CvPoint3D64f>& vpntPointCould, T_ROBOT_MOVE_SPEED tPulseMove)
{
	//CRobotDriverAdaptor* pRobotDriver = GetRobotCtrl();
	////XiMessageBoxPopup(NULL, "开始线扫识别", m_bNaturalPop);

	////先关闭IO
	//pRobotDriver->CallJob("CLOSEIO");

	////初始化,参数
	//CString strFilePath = GetFolderPath() + "\\LineScan";
	//CLineScan cLineScan(this, 0, strFilePath);

	////设定线扫位置高度调整量
	//cLineScan.SetHeightAdjust();

	////参数
	//T_ANGLE_PULSE tMovePulse;
	//if (!cLineScan.LoadLineScanPara(tDiffCoor, strTableName,  strTemplateNo,  strPath, tMovePulse))
	//{
	//	XiMessageBox(cLineScan.GetLastErrorInfo());
	//	return false;
	//}
	//m_bCanRotate = FALSE;

	////等待机器人运动结束
	//while (1)
	//{
	//	if (!RobotCheckRunning())
	//	{
	//		Sleep(50);
	//		if (!RobotCheckRunning())
	//		{
	//			break;
	//		}
	//	}
	//	Sleep(50);
	//}
	//BOOL bRtn;
	//T_ANGLE_PULSE tCurPulse;
	//if (CheckSafePos())
	//{
	//	tCurPulse = GetRobotPulse();
	//	tCurPulse.lBXPulse = tMovePulse.lBXPulse;
	//	GetRobotCtrl()->MoveByJob(tCurPulse, tPulseMove, 1, "MOVL");
	//	bRtn = CheckRtnResult(CheckMoveTrack(tCurPulse));
	//	if (bRtn == FALSE)
	//	{
	//		MESSAGE_BOX(GetStr("线扫运动失败！"));
	//		return false;
	//	}
	//}
	////开往起始点
	//GetRobotCtrl()->MoveByJob(tMovePulse, tPulseMove, 1, "MOVJ");

	////初始化相机，设置运动变量
	//if (!cLineScan.BeforLineScan(-1))
	//{
	//	XiMessageBox(cLineScan.GetLastErrorInfo());
	//	return false;
	//}

	////检测到位
	//tCurPulse = GetRobotPulse();
	//if (!ComparePulse(tCurPulse, tMovePulse)
	//	|| !ComparePulse_Base(tMovePulse, 3.0))
	//{
	//	//未到位则检测停止再检测到位
	//	WriteLog("未到位则检测停止再检测到位");
	//	RobotCheckDone();
	//	tCurPulse = GetRobotPulse();
	//	if (!ComparePulse(tCurPulse, tMovePulse)
	//		|| !ComparePulse_Base(tMovePulse, 3.0))
	//	{
	//		XiMessageBox("ERROR: 未到达线扫起始位置！");
	//		return false;
	//	}
	//}

	//if (!cLineScan.LineScan())
	//{
	//	XiMessageBox(cLineScan.GetLastErrorInfo());
	//	return false;
	//}
	//strPointCouldPath = cLineScan.GetPointCouldPath();
	//vpntPointCould.clear();
	//vpntPointCould.assign(cLineScan.m_vpntPointCould.begin(), cLineScan.m_vpntPointCould.end());
	return true;
}


bool CUnitModule::LaserScanGetPointCouid(CString strTableName, CString& strPointCouldPath, std::vector<CvPoint3D64f>& vpntPointCould)
{
	//CRobotDriverAdaptor* pRobotDriver = GetRobotCtrl();
	//XiMessageBoxPopup(NULL, "开始线扫识别", m_bNaturalPop);

	////先关闭IO
	//pRobotDriver->CallJob("CLOSEIO");

	////初始化,参数
	//CString strFilePath = GetFolderPath() + "\\LineScan";
	//CLineScan cLineScan(this, 0, strFilePath);

	////设定线扫位置高度调整量
	//cLineScan.SetHeightAdjust();

	////参数
	//T_ANGLE_PULSE tMovePulse;
	//if (!cLineScan.LoadLineScanPara(strTableName, tMovePulse))
	//{
	//	XiMessageBox(cLineScan.GetLastErrorInfo());
	//	return false;
	//}

	////开往起始点
	//T_ROBOT_MOVE_SPEED tPulseMove = { 1000,100,100 };
	//pRobotDriver->MoveByJob(tMovePulse, tPulseMove, 0);
	//if (FALSE == RobotCheckDone(tMovePulse))
	//{
	//	return false;
	//}

	////
	//if (!cLineScan.BeforLineScan(-1))
	//{
	//	XiMessageBox(cLineScan.GetLastErrorInfo());
	//	return false;
	//}

	////检测到位
	//T_ANGLE_PULSE tCurPulse = GetRobotPulse();
	//if (!ComparePulse(tCurPulse, tMovePulse)
	//	|| !ComparePulse_Base(tMovePulse, 3.0))
	//{
	//	//未到位则检测停止再检测到位
	//	WriteLog("未到位则检测停止再检测到位");
	//	RobotCheckDone();
	//	tCurPulse = GetRobotPulse();
	//	if (!ComparePulse(tCurPulse, tMovePulse)
	//		|| !ComparePulse_Base(tMovePulse, 3.0))
	//	{
	//		XiMessageBox("ERROR: 未到达线扫起始位置！");
	//		return false;
	//	}
	//}

	//if (!cLineScan.LineScan())
	//{
	//	XiMessageBox(cLineScan.GetLastErrorInfo());
	//	return false;
	//}
	//strPointCouldPath = cLineScan.GetPointCouldPath();
	//vpntPointCould.clear();
	//vpntPointCould.assign(cLineScan.m_vpntPointCould.begin(), cLineScan.m_vpntPointCould.end());
	return true;
}

bool CUnitModule::LaserScanRuning(CString strTableName,int nCurRecognitionTableNo)
{
//	CRobotDriverAdaptor* pRobotDriver = GetRobotCtrl();
//	XiMessageBoxPopup(NULL, "开始线扫识别", m_bNaturalPop);
	//
//	//先关闭IO
//	pRobotDriver->CallJob("CLOSEIO");
//
//	//初始化,参数
//	CString strFilePath = GetFolderPath() + "\\LineScan";
//	CLineScan cLineScan(this, 0, strFilePath);
//
//	//设定线扫位置高度调整量
//	if (1)//第一次按照默认高度
//	{
//		cLineScan.SetHeightAdjust();
//	}
//	else//根据上次工件高度、相机手眼关系计算新高度
//	{
//		//设定新的线扫位置高度
////		double dNewHeight;
////		cLineScan.SetLineScanHeight(dNewHeight);
//	}
//
//	//参数
//	T_ANGLE_PULSE tMovePulse;
//	if (!cLineScan.LoadLineScanPara(strTableName, tMovePulse))
//	{
//		XiMessageBox(cLineScan.GetLastErrorInfo());
//		return false;
//	}
//
//	//开往起始点
//	T_ROBOT_MOVE_SPEED tPulseMove = { 1000,100,100 };
//	pRobotDriver->MoveByJob(tMovePulse, tPulseMove, 0);
//	if (FALSE == RobotCheckDone(tMovePulse))
//	{
//		return false;
//	}
//
//	//
//	if (!cLineScan.BeforLineScan(nCurRecognitionTableNo))
//	{
//		XiMessageBox(cLineScan.GetLastErrorInfo());
//		return false;
//	}
//
//	//检测到位
//	T_ANGLE_PULSE tCurPulse = GetRobotPulse();
//	if (!ComparePulse(tCurPulse, tMovePulse)
//		|| !ComparePulse_Base(tMovePulse, 3.0))
//	{
//		//未到位则检测停止再检测到位
//		WriteLog("未到位则检测停止再检测到位");
//		RobotCheckDone();
//		tCurPulse = GetRobotPulse();
//		if (!ComparePulse(tCurPulse, tMovePulse)
//			|| !ComparePulse_Base(tMovePulse, 3.0))
//		{
//			XiMessageBox("ERROR: 未到达线扫起始位置！");
//			return false;
//		}
//	}
//
//	if (!cLineScan.LineScan())
//	{
//		XiMessageBox(cLineScan.GetLastErrorInfo());
//		return false;
//	}
//
//
//	//识别匹配函数
//	bool bRtn = cLineScan.Recognize(false,nCurRecognitionTableNo);
//	CString strCurRecognitionTableNo;
//	strCurRecognitionTableNo.Format("%d", nCurRecognitionTableNo);
//	if (bRtn)
//	{
//		LoadMatchInfo(m_vtRecognitionPara);
//		LoadMatchInfo(m_vvtRecognitionParaA[nCurRecognitionTableNo-1]);
//		LoadGraspAndPlaceInfo(m_vtGraspAndPlaceInfo);
//		LoadGraspAndPlaceInfo(m_vtGraspAndPlaceInfo, ".\\Data\\SortRobotA\\LineScan\\FeedingTray" + strCurRecognitionTableNo + "\\GraphData\\GraspAndPlaceInfo.txt");
//		m_vtTheoryGraphCenter.clear();
//		m_vtTheoryGraphCenter = LoadTheoryGraphCenter();
//	}
//	else
//	{
//		XiMessageBox(cLineScan.GetLastErrorInfo());
//	}
//
//	if (!cLineScan.CheckRobotLineScanDone())
//	{
//		WriteLog("还未到位就检测停止");
//		RobotCheckDone();
//	}
//	if (bRtn)
//	{
		return true;
//	}
//	else
//	{
//		return false;
//	}
}

T_RUNNING_RTN_RESULT CUnitModule::CalcGrabPos(int nPartNo, T_ANGLE_PULSE tReferencePulse, T_ROBOT_COORS tRobotTool,T_ROBOT_COORS& tGrabPos)
{
	T_RUNNING_RTN_RESULT tRtnResult;
	CHECK_RTN_RESULT(tRtnResult, CalcGrabPos(m_vtRecognitionPara[nPartNo], tReferencePulse, tRobotTool, tGrabPos));
	return T_RUNNING_RTN_RESULT();
}

T_RUNNING_RTN_RESULT CUnitModule::CalcGrabPos(T_RECOGNITION_PARA tRecognitionPara, T_ANGLE_PULSE tReferencePulse, T_ROBOT_COORS tRobotTool, T_ROBOT_COORS& tGrabPos)
{
	CRobotDriverAdaptor* pRobotDriver = GetRobotCtrl();
	T_RUNNING_RTN_RESULT tRtnResult;
	int nChartNo = tRecognitionPara.nTemplateNo;
	T_GRASPANDPLACE_INFO tGraspAndPlaceInfo = m_vtGraspAndPlaceInfo[nChartNo];

	T_POINT_3D tGraspPoint;
	tGraspPoint.x = tGraspAndPlaceInfo.tMagnetCenter.x;
	tGraspPoint.y = tGraspAndPlaceInfo.tMagnetCenter.y;
	tGraspPoint.z = 0.0;

	double dIdealRotateAngle = tGraspAndPlaceInfo.dMagnetRotateAngle;

	std::vector<T_POINT_3D> vtGraspPoint;
	vtGraspPoint.clear();
	vtGraspPoint.push_back(tGraspPoint);

	T_POINT_3D tTheoryCenter;
	tTheoryCenter.x = -m_vtTheoryGraphCenter[nChartNo].dCenterX;
	tTheoryCenter.y = -m_vtTheoryGraphCenter[nChartNo].dCenterY;
	tTheoryCenter.z = 0.0;
	double dRotateChangle = tRecognitionPara.dAngleRotate;
	CoorTransform(vtGraspPoint, tTheoryCenter, dRotateChangle);
	double dRealRotateAngle = dIdealRotateAngle + dRotateChangle;

	T_POINT_3D tRealCenter;
	tRealCenter.x = tRecognitionPara.dTranslateX;
	tRealCenter.y = tRecognitionPara.dTranslateY;
	tRealCenter.z = 0.0;

	CoorTransform(vtGraspPoint, tRealCenter, 0.0);

	T_POINT_3D tRobotTeachCenterPoint;
	tRobotTeachCenterPoint.x = tRecognitionPara.dTranslateX;
	tRobotTeachCenterPoint.y = tRecognitionPara.dTranslateY;
	tRobotTeachCenterPoint.z = tRecognitionPara.dPartHeight;


	T_ROBOT_COORS tGraspCoors, tGraspInitalCoors;

	double dGraspPointMatrixInBase[4][4];
	int nRows = 4, nCols = 4;


	double dRotateX = 180.0;
	double dMatrixRotateX[4][4];
	double dMatrixRotateZ[4][4];

	pRobotDriver->m_cXiRobotAbsCoorsTrans->GetRotateZMatrix(dRealRotateAngle, dMatrixRotateZ);
	pRobotDriver->m_cXiRobotAbsCoorsTrans->GetRotateXMatrix(dRotateX, dMatrixRotateX);
	pRobotDriver->m_cXiRobotAbsCoorsTrans->Trmul(dMatrixRotateZ, nRows, nCols, dMatrixRotateX, nRows, nCols, dGraspPointMatrixInBase, nRows, nCols);


	dGraspPointMatrixInBase[0][3] = vtGraspPoint[0].x;
	dGraspPointMatrixInBase[1][3] = vtGraspPoint[0].y;
	dGraspPointMatrixInBase[2][3] = tRobotTeachCenterPoint.z;

	/////////外部轴抓取/////////
	double dBaseToWorldTransMatrix[4][4];
	double dGraspPointMatrixInWorld[4][4];
	//GetBaseToWorldTransMatrix(tCurrentRobotPulses.dYExternalPos, m_dBaseToExternalTransMatrix, dBaseToWorldTransMatrix);
	double dCurrentExternalX = 0.0, dCurrentExternalY = 0.0, dCurrentExternalZ = 0.0;


	GetBaseToWorldTransMatrix(dCurrentExternalX, dCurrentExternalY, dCurrentExternalZ, m_dBaseToExternalTransMatrix, dBaseToWorldTransMatrix);
	pRobotDriver->m_cXiRobotAbsCoorsTrans->Trmul(dBaseToWorldTransMatrix, nRows, nCols, dGraspPointMatrixInBase, nRows, nCols, dGraspPointMatrixInWorld, nRows, nCols);

	T_ROBOT_COORS_IN_WORLD tGraspPointInWorld;
	tGraspPointInWorld.dX = dGraspPointMatrixInWorld[0][3];
	tGraspPointInWorld.dY = dGraspPointMatrixInWorld[1][3];
	tGraspPointInWorld.dZ = dGraspPointMatrixInWorld[2][3];
	tGraspPointInWorld.dRX = 0.0;
	tGraspPointInWorld.dRY = 0.0;
	tGraspPointInWorld.dRZ = 0.0;

	double dGraspExternalAxisXPos = 0.0;
	double dGraspExternalAxisYPos = 0.0;
	double dGraspExternalAxisZPos = 0.0;

	ProcessExternalAxisValue(tGraspPointInWorld, pRobotDriver->m_nExternalAxleType, dGraspExternalAxisXPos, dGraspExternalAxisYPos, dGraspExternalAxisZPos);

	double dWorldToBaseTransMatrix[4][4];
	//GetWorldToBaseTransMatrix(dGraspExternalAxisPos, m_dExternalToBaseTransMatrix, dWorldToBaseTransMatrix);
	GetWorldToBaseTransMatrix(dGraspExternalAxisXPos, dGraspExternalAxisYPos, dGraspExternalAxisZPos, m_dExternalToBaseTransMatrix, dWorldToBaseTransMatrix);
	pRobotDriver->m_cXiRobotAbsCoorsTrans->Trmul(dWorldToBaseTransMatrix, nRows, nCols, dGraspPointMatrixInWorld, nRows, nCols, dGraspPointMatrixInBase, nRows, nCols);



	tGraspCoors.dX = dGraspPointMatrixInBase[0][3];
	tGraspCoors.dY = dGraspPointMatrixInBase[1][3];
	tGraspCoors.dZ = dGraspPointMatrixInBase[2][3];
	tGraspCoors.dRX = 0.0;
	tGraspCoors.dRY = 0.0;
	tGraspCoors.dRZ = 0.0;
	pRobotDriver->m_cXiRobotAbsCoorsTrans->GetRotateDecompose(dGraspPointMatrixInBase, tGraspCoors.dRX, tGraspCoors.dRY, tGraspCoors.dRZ);

	T_ANGLE_PULSE tPulse;
	if (!pRobotDriver->RobotInverseKinematics(tGraspCoors, tReferencePulse, tRobotTool, tPulse, pRobotDriver->m_nExternalAxleType))
	{
		MESSAGE_BOX_UNIT("抓取位置超限！");
		RTN_STATE_RESULT(tRtnResult, RTN_STATUS_MOVE_PULSE_LIMIT);
	}
	tGrabPos = tGraspCoors;
	WRITE_LOG_UNIT(GetStr("tGraspCoors is tGraspCoors.dX = %lf %lf %lf %lf %lf %lf %lf %lf %lf",
		tGraspCoors.dX,
		tGraspCoors.dY,
		tGraspCoors.dZ,
		tGraspCoors.dRX,
		tGraspCoors.dRY,
		tGraspCoors.dRZ,
		tGraspCoors.dBX,
		tGraspCoors.dBY,
		tGraspCoors.dBZ
	));
	RTN_STATE_RESULT(tRtnResult, RTN_STATUS_SUCCEED);
}

bool CUnitModule::LoadMatchInfo(std::vector<T_RECOGNITION_PARA>& vtMatchInfo, CString sRealResultFileName)
{
	vtMatchInfo.clear();
	char acTmpArr[1000];
	FILE* pfRecognition;
	if (0 != XI_fopen_s(&pfRecognition, sRealResultFileName, "r"))
	{
		return false;
	}
	fgets(acTmpArr, 1000, pfRecognition);

	int nCameraNo = 0;
	int nCameraPos = 0;
	int nTemplateNo = 0;
	double dPartHeight = 0.0;
	double dAngleRotate = 0.0;
	double dTranslateX = 0.0;
	double dTranslateY = 0.0;
	int nLayerNo = 0;
	int nGroupNo = 0;
	T_RECOGNITION_PARA tReconnitionPara;

	while (fscanf_s(pfRecognition, "%d%d%d%lf%lf%lf%lf%d%d", &nTemplateNo, &nCameraNo, &nCameraPos, &dTranslateX, &dTranslateY, &dPartHeight, &dAngleRotate, &nLayerNo, &nGroupNo) > 0)
	{
		tReconnitionPara.nCameraNo = nCameraNo;
		tReconnitionPara.nCameraPos = nCameraPos;
		tReconnitionPara.nTemplateNo = nTemplateNo;
		tReconnitionPara.dPartHeight = dPartHeight;
		tReconnitionPara.dAngleRotate = dAngleRotate;
		tReconnitionPara.dTranslateX = dTranslateX;
		tReconnitionPara.dTranslateY = dTranslateY;
		tReconnitionPara.nLayerNo = nLayerNo;
		tReconnitionPara.nGroupNo = nGroupNo;

		vtMatchInfo.push_back(tReconnitionPara);
	}
	fclose(pfRecognition);

	return true;
}

bool CUnitModule::SaveMatchInfo(std::vector<T_RECOGNITION_PARA> vtMatchInfo, CString sRealResultFileName)
{
	FILE* pfRecognition;
	if (0 != XI_fopen_s(&pfRecognition, sRealResultFileName, "w"))
	{
		return false;
	}
	fprintf_s(pfRecognition, "模板号：  工件轮廓中心点所在相机号：  工件轮廓中心所在相机位置：  工件轮廓中心点X:   工件轮廓中心点Y:   工件轮廓中心点Z:   顺时针选择角度：   工件编号：   摞号：  \n");
	for (int nIdx = 0; nIdx < vtMatchInfo.size(); nIdx++)
	{
		fprintf_s(pfRecognition, "%d   %d   %d   %lf   %lf   %lf   %lf   %d   %d\n", vtMatchInfo[nIdx].nTemplateNo, vtMatchInfo[nIdx].nCameraNo,
			vtMatchInfo[nIdx].nCameraPos, vtMatchInfo[nIdx].dTranslateX, vtMatchInfo[nIdx].dTranslateY, vtMatchInfo[nIdx].dPartHeight,
			vtMatchInfo[nIdx].dAngleRotate, vtMatchInfo[nIdx].nLayerNo, vtMatchInfo[nIdx].nGroupNo);
	}
	fclose(pfRecognition);
	return true;
}

bool CUnitModule::LoadGraspAndPlaceInfo(std::vector<T_GRASPANDPLACE_INFO>& vtGraspAndPlaceInfo, CString sRealResultFileName)
{
	FILE* posfile;
	if (0 != XI_fopen_s(&posfile, sRealResultFileName, "r"))
	{
		return false;
	}
	vtGraspAndPlaceInfo.clear();
	T_GRASPANDPLACE_INFO tGraspAndPlaceInfo;
	int n;
	while (fscanf_s(posfile, "%d%lf%lf%lf%lf%lf%lf%lf",
		&n, &tGraspAndPlaceInfo.tCuttingTableCenter.x, &tGraspAndPlaceInfo.tCuttingTableCenter.y, &tGraspAndPlaceInfo.dCuttingTableRotateAngle,
		&tGraspAndPlaceInfo.tMagnetCenter.x, &tGraspAndPlaceInfo.tMagnetCenter.y, &tGraspAndPlaceInfo.dMagnetRotateAngle,
		&tGraspAndPlaceInfo.dThickness) > 0)
	{
		vtGraspAndPlaceInfo.push_back(tGraspAndPlaceInfo);
	}
	fclose(posfile);
	return false;
}

std::vector<T_Template_PARA> CUnitModule::LoadTheoryGraphCenter(CString sIdealResultFileName)
{
	FILE* pfTemplate;
	XI_fopen_s(&pfTemplate, sIdealResultFileName, "r");
	std::vector<T_Template_PARA> vtTemplatePara;
	T_Template_PARA tTemplatePara;
	int nTemplateNo = 0;
	double dCenterX = 0;
	double dCenterY = 0;
	while (fscanf_s(pfTemplate, "%d%lf%lf", &nTemplateNo, &dCenterX, &dCenterY) > 0)
	{
		tTemplatePara.nTemplateNo = nTemplateNo;
		tTemplatePara.dCenterX = dCenterX;
		tTemplatePara.dCenterY = dCenterY;

		vtTemplatePara.push_back(tTemplatePara);
	}
	fclose(pfTemplate);

	return vtTemplatePara;
}

int CUnitModule::GetPartNoFromTemplate(T_RECOGNITION_PARA &tRecognitionPara, int nTemplateNo, std::vector<T_RECOGNITION_PARA> vtRecognitionPara)
{
	std::vector<T_RECOGNITION_PARA> vtTemp;
	sort(vtRecognitionPara.begin(), vtRecognitionPara.end(), cmpRecognitionParaHeight_FromLargeToSmall);
	for (size_t i = 0; i < vtRecognitionPara.size(); i++)
	{
		if (vtRecognitionPara[i].nTemplateNo == nTemplateNo)
		{
			vtTemp.push_back(vtRecognitionPara[i]);
		}
	}
	if (vtTemp.empty())
	{
		return -1;
	}
	tRecognitionPara = vtTemp[0];
	return vtTemp.size() - 1;
}

bool CUnitModule::DeletePartNoFromTemplate(T_RECOGNITION_PARA tRecognitionPara, std::vector<T_RECOGNITION_PARA>& vtRecognitionPara)
{
	for (size_t i = 0; i < vtRecognitionPara.size(); i++)
	{
		if (tRecognitionPara.nTemplateNo == vtRecognitionPara[i].nTemplateNo &&
			tRecognitionPara.nGroupNo == vtRecognitionPara[i].nGroupNo &&
			tRecognitionPara.nLayerNo == vtRecognitionPara[i].nLayerNo &&
			IsEqual(tRecognitionPara.dTranslateX, vtRecognitionPara[i].dTranslateX, 0.1) &&
			IsEqual(tRecognitionPara.dTranslateY, vtRecognitionPara[i].dTranslateY, 0.1) &&
			IsEqual(tRecognitionPara.dPartHeight, vtRecognitionPara[i].dPartHeight, 0.1) &&
			IsEqual(tRecognitionPara.dAngleRotate, vtRecognitionPara[i].dAngleRotate, 0.1))
		{
			vtRecognitionPara.erase(vtRecognitionPara.begin() + i, vtRecognitionPara.begin() + i + 1);
			return true;
		}
	}
	return false;
}

T_RUNNING_RTN_RESULT CUnitModule::SetTrack(std::vector<T_ROBOT_MOVE_INFO>& tRobotMoveTrack, int nContinueStep)
{
	T_RUNNING_RTN_RESULT tRtnResult;
	if (nContinueStep >= tRobotMoveTrack.size() || nContinueStep < 0)
	{
		RTN_STATE_RESULT(tRtnResult, RTN_STATUS_MOVE_VAR_ERROR);
	}
	tRobotMoveTrack.erase(tRobotMoveTrack.begin(), tRobotMoveTrack.begin() + nContinueStep);
	if (!SetMoveValue(tRobotMoveTrack))
	{
		RTN_STATE_RESULT(tRtnResult, RTN_STATUS_MOVE_VAR_ERROR);
	}

	RTN_STATE_RESULT(tRtnResult, RTN_STATUS_SUCCEED);
}

T_RUNNING_RTN_RESULT CUnitModule::SetTrack(CRobotMove cMoveInfo, int nContinueStep)
{
	T_RUNNING_RTN_RESULT tRtnResult;
	if (!cMoveInfo.DeleteTrack(nContinueStep))
	{
		RTN_STATE_RESULT(tRtnResult, RTN_STATUS_MOVE_VAR_ERROR);
	}
	if (!SetMoveValue(cMoveInfo))
	{
		RTN_STATE_RESULT(tRtnResult, RTN_STATUS_MOVE_VAR_ERROR);
	}

	RTN_STATE_RESULT(tRtnResult, RTN_STATUS_SUCCEED);
}

T_RUNNING_RTN_RESULT CUnitModule::StartMove()
{
	T_RUNNING_RTN_RESULT tRtnResult;
	CallJob("CONTIMOVANY", GetRobotCtrl()->m_nExternalAxleType);
	RTN_STATE_RESULT(tRtnResult, RTN_STATUS_SUCCEED);
}

T_RUNNING_RTN_RESULT CUnitModule::CheckMoveTrack(T_ROBOT_COORS tRobotCoor)
{
	T_RUNNING_RTN_RESULT tRtnResult;
	if (FALSE == RobotCheckDone(tRobotCoor, T_ROBOT_COORS(1, 1, 1, -1, -1, -1, -1, -1, -1)))
	{
		WORD wState;
		int nRtn = GetRobotServoSignal(wState);
		if (nRtn != 0 || wState == OFF)
		{
			Sleep(100);
			nRtn = GetRobotServoSignal(wState);
			if (nRtn != 0 || wState == OFF)
			{
				WRITE_LOG_UNIT("机器人伺服掉电,急停外部轴！");
				UnitEmgStop();
				RTN_STATE_RESULT(tRtnResult, RTN_STATUS_MOVE_PLACE_ERROR);
			}
		}
		RTN_STATE_RESULT(tRtnResult, RTN_STATUS_MOVE_PLACE_ERROR);
	}
	CleanMoveStep();
	WriteLog("CheckMoveTrack_End");
	RTN_STATE_RESULT(tRtnResult, RTN_STATUS_SUCCEED);
}

T_RUNNING_RTN_RESULT CUnitModule::CheckMoveTrack(T_ANGLE_PULSE tRobotPulse)
{
	T_RUNNING_RTN_RESULT tRtnResult;

	if (FALSE == RobotCheckDone(tRobotPulse))
	{
		WORD wState;
		int nRtn = GetRobotServoSignal(wState);
		if (nRtn != 0 || wState == OFF)
		{
			Sleep(100);
			nRtn = GetRobotServoSignal(wState);
			if (nRtn != 0 || wState == OFF)
			{
				WRITE_LOG_UNIT("机器人伺服掉电,急停外部轴！");
				UnitEmgStop();
				RTN_STATE_RESULT(tRtnResult, RTN_STATUS_MOVE_PLACE_ERROR);
			}
		}
		RTN_STATE_RESULT(tRtnResult, RTN_STATUS_MOVE_PLACE_ERROR);
	}
	CleanMoveStep();
	RTN_STATE_RESULT(tRtnResult, RTN_STATUS_SUCCEED);
}

T_RUNNING_RTN_RESULT CUnitModule::RobotPosMove(T_ROBOT_COORS tRobotCoor, T_ROBOT_MOVE_SPEED tRobotSpeed, int nMoveType /*= MOVJ*/, UINT unToolNum /*= 1*/, bool bWait)
{
	T_RUNNING_RTN_RESULT tRtnResult;
	CRobotMove cRobotMove;
	cRobotMove.Clear();
	cRobotMove.InsertCoor(tRobotCoor, tRobotSpeed, 1, nMoveType, unToolNum);
	CHECK_RTN_RESULT(tRtnResult, SetTrack(cRobotMove));
	CHECK_RTN_RESULT(tRtnResult, StartMove());
	if (bWait)
	{
		CHECK_RTN_RESULT(tRtnResult, CheckMoveTrack(tRobotCoor));
	}
	RTN_STATE_RESULT(tRtnResult, RTN_STATUS_SUCCEED);
}

T_RUNNING_RTN_RESULT CUnitModule::RobotPosMove(T_ANGLE_PULSE tRobotPulse, T_ROBOT_MOVE_SPEED tRobotSpeed, int nMoveType /*= MOVJ*/, UINT unToolNum /*= 1*/, bool bWait)
{
	T_RUNNING_RTN_RESULT tRtnResult;
	CRobotMove cRobotMove;
	cRobotMove.Clear();
	cRobotMove.InsertCoor(tRobotPulse, tRobotSpeed, 1, nMoveType, unToolNum);
	CHECK_RTN_RESULT(tRtnResult, SetTrack(cRobotMove));
	CHECK_RTN_RESULT(tRtnResult, StartMove());
	if (bWait)
	{
		CHECK_RTN_RESULT(tRtnResult, CheckMoveTrack(tRobotPulse));
	}
	RTN_STATE_RESULT(tRtnResult, RTN_STATUS_SUCCEED);
}

bool CUnitModule::CheckSafePos()
{
	return GetRobotCtrl()->ComparePulse(GetRobotCtrl()->m_tHomePulse, GetRobotPulse(), T_ANGLE_PULSE(10, 10, 10, 10, 10, 10, -1, -1, -1));
}