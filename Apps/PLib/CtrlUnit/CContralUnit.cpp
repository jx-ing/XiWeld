#include "stdafx.h"
#include "CContralUnit.h"

#include <fstream>
CContralUnit::CContralUnit(T_CONTRAL_UNIT tCtrlUnitInfo, CServoMotorDriver *pServoMotorDriver)
: m_tContralUnit(tCtrlUnitInfo), m_pServoMotorDriver(pServoMotorDriver)
{
	InitContralResource();
}

CContralUnit::~CContralUnit()
{
	for (int n = 0; n < m_pContralCard.size(); n++)
	{
		DELETE_UNIT(m_pContralCard[n]);
	}
	for (int n = 0; n < m_vpImageCapture.size(); n++)
	{
		DELETE_UNIT(m_vpImageCapture[n]);
	}
	for (int n = 0; n < m_vpKinectControl.size(); n++)
	{
		DELETE_UNIT(m_vpKinectControl[n]);
	}
#if ENABLE_MECH_EYE
	for (int n = 0; n < m_vpMechMindDevice.size(); n++)
	{
		DELETE_UNIT(m_vpMechMindDevice[n]);
	}
#endif
	DELETE_UNIT(m_pYasakawaRobotDriver);
	DELETE_UNIT(m_cUnitLog);
}

BOOL CContralUnit::LoadBaseIOParam(CString strUnitName, T_COMMON_IO &m_tCommonIO)
{
	BOOL bRtn = 0;
	int nValNum = 0;
	COPini opini;
	CString strFileName = DATA_PATH + strUnitName + COMMON_IO_PARAM_INI;
	opini.SetFileName(strFileName);
	//气路
	opini.SetSectionName("GasPath");
	bRtn = bRtn && opini.ReadAddString("AirPressureSignalIO", &m_tCommonIO.tGasIO.nAirPressureSignalIO, -1);
	bRtn = bRtn && opini.ReadAddString("AirSolenoidIO", &m_tCommonIO.tGasIO.nAirSolenoidIO, -1);

	//警示灯
	bRtn = opini.SetSectionName("CautionLight");
	bRtn = bRtn && opini.ReadAddString("CautionLightGreenIO", &m_tCommonIO.tWarningLightIO.nCautionLightGreenIO_OUT, -1);
	bRtn = bRtn && opini.ReadAddString("CautionLightYellowIO", &m_tCommonIO.tWarningLightIO.nCautionLightYellowIO_OUT, -1);
	bRtn = bRtn && opini.ReadAddString("CautionLightRedIO", &m_tCommonIO.tWarningLightIO.nCautionLightRedIO_OUT, -1);

	bRtn = opini.SetSectionName("AllRobotIO_IN");//机械臂输出
	bRtn = bRtn && opini.ReadAddString("AllEmgStopIO", &m_tCommonIO.tEmgStopIO.nAllEmgStopIO, -1);

	bRtn = opini.SetSectionName("RobotPowerIO_OUT");//机械控制柜电源
	bRtn = bRtn && opini.ReadAddString("RobotPowerIO", &m_tCommonIO.tRobtPowerIO.nRobotExternalPowerControl, -1);

	//机器人控制柜电源

	return TRUE;
}

BOOL CContralUnit::LoadRobotIOParam(CString strUnitName, T_YASKAWA_ROBOT_IO & m_tRobotIO)
{
	BOOL bRtn = TRUE;
	int nValNum = 0;
	COPini opini;
	CString strFileName = DATA_PATH + strUnitName + COMMON_IO_PARAM_INI;
	opini.SetFileName(strFileName);

	opini.SetSectionName("YaskawaRobotIO");
	bRtn = bRtn && opini.ReadAddString("RobotRunningIO", &m_tCommonIO.tYaskawaRobotIO.tRobotInputIO.nRobotRunningIO, -1);
	bRtn = bRtn && opini.ReadAddString("RobotServoOnIO", &m_tCommonIO.tYaskawaRobotIO.tRobotInputIO.nRobotServoOnIO, -1);
	bRtn = bRtn && opini.ReadAddString("ErrorsIO", &m_tCommonIO.tYaskawaRobotIO.tRobotInputIO.nErrorsIO, -1);
	bRtn = bRtn && opini.ReadAddString("EmgStopIO", &m_tCommonIO.tYaskawaRobotIO.tRobotInputIO.nEmgStopIO, -1);
	bRtn = bRtn && opini.ReadAddString("BatteryWarningIO", &m_tCommonIO.tYaskawaRobotIO.tRobotInputIO.nBatteryWarningIO, -1);
	bRtn = bRtn && opini.ReadAddString("OperationalOriginIO", &m_tCommonIO.tYaskawaRobotIO.tRobotInputIO.nOperationalOriginIO, -1);
	bRtn = bRtn && opini.ReadAddString("MidwayStartIO", &m_tCommonIO.tYaskawaRobotIO.tRobotInputIO.nMidwayStartIO, -1);
	bRtn = bRtn && opini.ReadAddString("ChoiceLongRangeModeIO", &m_tCommonIO.tYaskawaRobotIO.tRobotInputIO.nChoiceLongRangeModeIO, -1);
	bRtn = bRtn && opini.ReadAddString("ChoiceManualModeIO", &m_tCommonIO.tYaskawaRobotIO.tRobotInputIO.nChoiceManualModeIO, -1);
	bRtn = bRtn && opini.ReadAddString("ChoiceTeachModeIO", &m_tCommonIO.tYaskawaRobotIO.tRobotInputIO.nChoiceTeachModeIO, -1);
	bRtn = bRtn && opini.ReadAddString("RobotInitIO", &m_tCommonIO.tYaskawaRobotIO.tRobotOutputIO.nRobotInitIO, -1);
	bRtn = bRtn && opini.ReadAddString("RobotPauseIO", &m_tCommonIO.tYaskawaRobotIO.tRobotOutputIO.nRobotPauseIO, -1);
	bRtn = bRtn && opini.ReadAddString("CallJobIO", &m_tCommonIO.tYaskawaRobotIO.tRobotOutputIO.nCallJobIO, -1);
	bRtn = bRtn && opini.ReadAddString("ResetErrorsIO", &m_tCommonIO.tYaskawaRobotIO.tRobotOutputIO.nResetErrorsIO, -1);
	bRtn = bRtn && opini.ReadAddString("ExRobotEmgIO", &m_tCommonIO.tYaskawaRobotIO.tRobotOutputIO.nRobotExEmgIO, -1);
	bRtn = bRtn && opini.ReadAddString("ServoOnIO", &m_tCommonIO.tYaskawaRobotIO.tRobotOutputIO.nServoOnIO, -1);
	bRtn = bRtn && opini.ReadAddString("ProhibitOperationIO", &m_tCommonIO.tYaskawaRobotIO.tRobotOutputIO.nProhibitOperationIO, -1);

	return bRtn;
}

BOOL CContralUnit::LoadMotorParam(CString strUnitName, std::vector< T_MOTOR_PARAM> &m_vtMotorPara)
{
	m_vtMotorPara.clear();

	T_MOTOR_PARAM tMotorPara;
	BOOL bRtn = TRUE;
	CString strFileName = DATA_PATH + strUnitName + MOTOR_PARAM_INI;
	
	int nMotorNum;
	COPini opini;
	bRtn = bRtn & opini.SetFileName(false, strFileName);
	bRtn = bRtn & opini.SetSectionName("Base");
	bRtn = bRtn & opini.ReadAddString("MotorNum", &nMotorNum, 0);
	for (int n = 0; n < nMotorNum; n++)
	{
		CString strSectionName;
		strSectionName.Format("Motor%d", n);
		bRtn = bRtn & opini.SetSectionName(strSectionName);
		bRtn = bRtn & opini.ReadAddString("SoftAxisNo", &tMotorPara.nSoftAxisNo, -1);
		bRtn = bRtn & opini.ReadAddString("ToRobotDir", &tMotorPara.nToRobotDir, -1);
		m_vtMotorPara.push_back(tMotorPara);
	}
	return bRtn;
}

BOOL CContralUnit::LoadDeviceIO()
{
	T_IO_PARAM tIOParam;
	int nNo = 0;
	bool bRet = TRUE;
	COPini opini;
	opini.SetFileName(IO_PARA_DEVICE);
	CString sKeyName;
	sKeyName.Format("%dNameCN", nNo);
	while (opini.CheckExists(IO_PARA_DEVICE, "INPUT", sKeyName)) // INPUT
	{
		opini.SetSectionName("INPUT");
		tIOParam.bReadOnly = true;
		sKeyName.Format("%dNameCN", nNo);
		bRet = bRet && opini.ReadString(sKeyName, tIOParam.sNameCN);
		tIOParam.sNameCN = Utf8ToGBK(tIOParam.sNameCN);
		sKeyName.Format("%dNameEN", nNo);
		bRet = bRet && opini.ReadString(sKeyName, tIOParam.sNameEN);
		int nValue = 0;
		sKeyName.Format("%dTypeInfo", nNo);
		bRet = bRet && opini.ReadString(sKeyName, &nValue);
		tIOParam.nCardNo = nValue / 1000;
		tIOParam.nCardNodeNo = (nValue - (tIOParam.nCardNo * 1000)) / 100;
		tIOParam.nType = (nValue - (tIOParam.nCardNo * 1000) - (tIOParam.nCardNodeNo * 100)) / 10;
		tIOParam.nDefaultVal = nValue % 10;
		sKeyName.Format("%dBit", nNo);
		bRet = bRet && opini.ReadString(sKeyName, &tIOParam.nBit);
		m_mtDeviceIO[tIOParam.sNameEN] = tIOParam;
		sKeyName.Format("%dNameCN", ++nNo);
	}

	nNo = 0;
	sKeyName.Format("%dNameCN", nNo);
	while (opini.CheckExists(IO_PARA_DEVICE, "OUTPUT", sKeyName)) // OUTPUT
	{
		opini.SetSectionName("OUTPUT");
		tIOParam.bReadOnly = false;
		sKeyName.Format("%dNameCN", nNo);
		bRet = bRet && opini.ReadString(sKeyName, tIOParam.sNameCN);
		tIOParam.sNameCN = Utf8ToGBK(tIOParam.sNameCN);
		sKeyName.Format("%dNameEN", nNo);
		bRet = bRet && opini.ReadString(sKeyName, tIOParam.sNameEN);
		int nValue = 0;
		sKeyName.Format("%dTypeInfo", nNo);
		bRet = bRet && opini.ReadString(sKeyName, &nValue);
		tIOParam.nCardNo = nValue / 1000;
		tIOParam.nCardNodeNo = (nValue - (tIOParam.nCardNo * 1000)) / 100;
		tIOParam.nType = (nValue - (tIOParam.nCardNo * 1000) - (tIOParam.nCardNodeNo * 100)) / 10;
		tIOParam.nDefaultVal = nValue % 10;
		sKeyName.Format("%dBit", nNo);
		bRet = bRet && opini.ReadString(sKeyName, &tIOParam.nBit);
		m_mtDeviceIO[tIOParam.sNameEN] = tIOParam;
		sKeyName.Format("%dNameCN", ++nNo);
	}
	return bRet;
}

BOOL CContralUnit::LoadRobotIOParam()
{
	m_mtRobotIO.clear();
	//加载IO
	int nNo = 0;
	T_IO_PARAM tIOParam;
	CString sKeyName;
	COPini opini;
	BOOL bRet = TRUE;
	CString sFileName = DATA_PATH + m_tContralUnit.strUnitName + IO_PARA_ROBOT;
	opini.SetFileName(sFileName);

	nNo = 0;
	sKeyName.Format("%dNameCN", nNo);
	while (opini.CheckExists(sFileName, "INPUT", sKeyName))	// INPUT
	{
		opini.SetSectionName("INPUT");
		tIOParam.bReadOnly = true;
		sKeyName.Format("%dNameCN", nNo);
		bRet = bRet && opini.ReadString(sKeyName, tIOParam.sNameCN);
		tIOParam.sNameCN = Utf8ToGBK(tIOParam.sNameCN);
		sKeyName.Format("%dNameEN", nNo);
		bRet = bRet && opini.ReadString(sKeyName, tIOParam.sNameEN);
		int nValue = 0;
		sKeyName.Format("%dTypeInfo", nNo);
		bRet = bRet && opini.ReadString(sKeyName, &nValue);
		tIOParam.nCardNo = nValue / 1000;
		tIOParam.nCardNodeNo = (nValue - (tIOParam.nCardNo * 1000)) / 100;
		tIOParam.nType = (nValue - (tIOParam.nCardNo * 1000) - (tIOParam.nCardNodeNo * 100)) / 10;
		tIOParam.nDefaultVal = nValue % 10;
		sKeyName.Format("%dBit", nNo);
		bRet = bRet && opini.ReadString(sKeyName, &tIOParam.nBit);
		m_mtRobotIO[tIOParam.sNameEN] = tIOParam;
		sKeyName.Format("%dNameCN", ++nNo);
	}

	
	nNo = 0;
	sKeyName.Format("%dNameCN", nNo);
	while (opini.CheckExists(sFileName, "OUTPUT", sKeyName)) // OUTPUT
	{
		opini.SetSectionName("OUTPUT");
		tIOParam.bReadOnly = false;
		sKeyName.Format("%dNameCN", nNo);
		bRet = bRet && opini.ReadString(sKeyName, tIOParam.sNameCN);
		tIOParam.sNameCN = Utf8ToGBK(tIOParam.sNameCN);
		sKeyName.Format("%dNameEN", nNo);
		bRet = bRet && opini.ReadString(sKeyName, tIOParam.sNameEN);
		int nValue = 0;
		sKeyName.Format("%dTypeInfo", nNo);
		bRet = bRet && opini.ReadString(sKeyName, &nValue);
		tIOParam.nCardNo = nValue / 1000;
		tIOParam.nCardNodeNo = (nValue - (tIOParam.nCardNo * 1000)) / 100;
		tIOParam.nType = (nValue - (tIOParam.nCardNo * 1000) - (tIOParam.nCardNodeNo * 100)) / 10;
		tIOParam.nDefaultVal = nValue % 10;
		sKeyName.Format("%dBit", nNo);
		bRet = bRet && opini.ReadString(sKeyName, &tIOParam.nBit);
		m_mtRobotIO[tIOParam.sNameEN] = tIOParam;
		sKeyName.Format("%dNameCN", ++nNo);
	}
	return bRet;
}

DWORD CContralUnit::SwitchIO(CString sName, bool bOpen)
{
	std::map<CString, T_IO_PARAM>::iterator iter = m_mtDeviceIO.find(sName);
	if (iter == m_mtDeviceIO.end())
	{
		iter = m_mtRobotIO.find(sName);
		if (iter == m_mtRobotIO.end())
		{
			return -1; // IO不存在
		}
	}
	T_IO_PARAM tIOParam = iter->second;
	int on_off = bOpen ? tIOParam.nDefaultVal : (0 == tIOParam.nDefaultVal ? 1 : 0);
	if (E_YASKAWA_IO == tIOParam.nType)
	{
		if (g_bLocalDebugMark) return 0;
		
		m_pYasakawaRobotDriver->m_pYaskawaRobotCtrl->WriteIO(tIOParam.nBit, on_off);
		return 0;
	}
	else if (E_ESTUN_IO == tIOParam.nType)
	{
		if (g_bLocalDebugMark) return 0;
		GetRobotCtrl()->SetOutIOVar(tIOParam.nBit, on_off);
		return 0;
	}
	else
	{
		return m_pBasicIOControl->WriteOutbit(tIOParam, on_off);
	}
}

bool CContralUnit::CheckCO2IO(CString sName)
{
	std::map<CString, T_IO_PARAM>::iterator iter = m_mtDeviceIO.find(sName);
	if (iter == m_mtDeviceIO.end())
	{
		iter = m_mtRobotIO.find(sName);
		if (iter == m_mtRobotIO.end())
		{
			return false; // IO不存在
		}
	}
	T_IO_PARAM tIOParam = iter->second;
	int nStatus = 0;
	if (E_LS_IO == tIOParam.nType)
	{
		nStatus = m_pBasicIOControl->ReadInbit(tIOParam);
	}
	else if (E_ESTUN_IO == tIOParam.nType)
	{
		if (g_bLocalDebugMark) return 0;
		GetRobotCtrl()->GetInputIOVar(tIOParam.nBit, nStatus);
	}
	else if (E_YASKAWA_IO == tIOParam.nType)
	{
		unsigned short int nRst = 0;
		m_pYasakawaRobotDriver->m_pYaskawaRobotCtrl->ReadRapidIO(tIOParam.nBit, nRst);
		//m_pYasakawaRobotDriver->m_pYaskawaRobotCtrl->ReadIO(tIOParam.nBit, nRst);
		nStatus = nRst;
	}
	return nStatus == tIOParam.nDefaultVal;
}

bool CContralUnit::CheckInIO(CString sName)
{
	std::map<CString, T_IO_PARAM>::iterator iter = m_mtDeviceIO.find(sName);
	if (iter == m_mtDeviceIO.end())
	{
		iter = m_mtRobotIO.find(sName);
		if (iter == m_mtRobotIO.end())
		{
			return false; // IO不存在
		}
	}
	T_IO_PARAM tIOParam = iter->second;
	int nStatus = 0;
	if (E_LS_IO == tIOParam.nType)
	{
		nStatus = m_pBasicIOControl->ReadInbit(tIOParam);
	}
	else if (E_ESTUN_IO == tIOParam.nType)
	{
		if (g_bLocalDebugMark) return 0;
		GetRobotCtrl()->GetInputIOVar(tIOParam.nBit, nStatus);
	}
	else if(E_YASKAWA_IO == tIOParam.nType)
	{
		unsigned short int nRst = 0;
		m_pYasakawaRobotDriver->m_pYaskawaRobotCtrl->ReadRapidIO(tIOParam.nBit, nRst);
		//m_pYasakawaRobotDriver->m_pYaskawaRobotCtrl->ReadIO(tIOParam.nBit, nRst);
		nStatus = nRst;
	}
	return nStatus == tIOParam.nDefaultVal;
}

bool CContralUnit::CheckOutIO(CString sName)
{
	std::map<CString, T_IO_PARAM>::iterator iter = m_mtDeviceIO.find(sName);
	if (iter == m_mtDeviceIO.end())
	{
		iter = m_mtRobotIO.find(sName);
		if (iter == m_mtRobotIO.end())
		{
			return false; // IO不存在
		}
	}
	T_IO_PARAM tIOParam = iter->second;
	int nStatus = 0;
	if (E_LS_IO == tIOParam.nType)
	{
		nStatus = m_pBasicIOControl->ReadOutbit(tIOParam);
	}
	else
	{
		unsigned short int nRst = 0;
		//m_pYasakawaRobotDriver->m_pYaskawaRobotCtrl->ReadRapidIO(tIOParam.nBit, nRst);
		m_pYasakawaRobotDriver->m_pYaskawaRobotCtrl->ReadIO(tIOParam.nBit, nRst);
		nStatus = nRst;
	}
	return nStatus == tIOParam.nDefaultVal;
}

BOOL CContralUnit::LoadCameraParam(CString strUnitName, std::vector<T_CAMREA_PARAM> &vtCameraPara)
{
	vtCameraPara.clear();

	BOOL bRtn = TRUE;
	int nCameraNo = 0;
	int nCameraType = 0;
	CString strFileName = DATA_PATH + strUnitName + CAMERA_PARAM_INI;
	CString strSectionName;

	COPini opini;
	opini.SetFileName(false, strFileName);
	int nCameraNum = 0;
	opini.SetSectionName("Base");
	opini.ReadAddString("CameraNum", &nCameraNum, 0);
	opini.ReadAddString("MeasureCameraNo", &m_nMeasureCameraNo, -1);
	opini.ReadAddString("TrackCameraNo", &m_nTrackCameraNo, -1);
	opini.ReadAddString("LineScanCameraNo", &m_nLineScanCameraNo, -1);


	for(int n = 0; n < nCameraNum; n++)
	{
		T_CAMREA_PARAM tCameraPara;
		strSectionName.Format("CAMERA%d", n);
		bRtn = bRtn && opini.SetSectionName(strSectionName);

		bRtn = bRtn && opini.ReadAddString("CameraName", tCameraPara.strCameraName, "NULL");
		tCameraPara.strCameraName = Utf8ToGBK(tCameraPara.strCameraName);
		bRtn = bRtn && opini.ReadAddString("CameraNameEN", tCameraPara.strCameraNameEN, "NULL");
		bRtn = bRtn && opini.ReadAddString("CameraType", &nCameraType, 1);
		if (nCameraType >= (int)E_CAMERA_TYPE_MAX_NUM || nCameraType < 0)
		{
			WRITE_LOG_UNIT(GetStr("相机 %d 类型错误", nCameraType));
		}
		tCameraPara.eCameraType = (E_CAMERA_TYPE)nCameraType;

		switch (tCameraPara.eCameraType)
		{
		case E_DH_CAMERA:
			bRtn = bRtn && LoadDHCameraParam(opini, tCameraPara.bAutoOpen,tCameraPara.nInstallPos,tCameraPara.eFlipMode, tCameraPara.tCameraTool);
			bRtn = bRtn && LoadDHCameraParam(opini, tCameraPara.tDHCameraDriverPara);
			tCameraPara.tHandEyeCaliPara.tCameraInnerPara.nWidth = tCameraPara.tDHCameraDriverPara.nRoiWidth;
			tCameraPara.tHandEyeCaliPara.tCameraInnerPara.nHeight = tCameraPara.tDHCameraDriverPara.nRoiHeight;
			bRtn = bRtn && LoadCameraBaseParam(opini, tCameraPara.tHandEyeCaliPara);
			bRtn = bRtn && LoadCameraHandEyeCaliParam(opini, tCameraPara.tHandEyeCaliPara);
			bRtn = bRtn && LoadCameraDistortionParam(opini, tCameraPara.tCameraDistortion);
			bRtn = bRtn && LoadLaserPlanEquation(opini, tCameraPara.tLaserPlanEquation);
			bRtn = bRtn && LoadCameraTransfomParam(opini, tCameraPara.tCameraTransfomPara);
			bRtn = bRtn && LoadDHCompen(opini, tCameraPara.tPointCloudCompen);
			break;
		case E_KINECT_CAMERA:
			bRtn = bRtn && LoadDepthCameraParam(opini, tCameraPara.tPANOCameraDriverPara);
			bRtn = bRtn && LoadCameraHandEyeCaliParam(opini, tCameraPara.tHandEyeCaliPara);
			bRtn = bRtn && LoadCameraDistortionParam(opini, tCameraPara.tCameraDistortion);
			break;
		case E_MECHEYE_CAMERA:
			bRtn = bRtn && LoadMecheyeCameraParam(opini, tCameraPara.tMecheyeCameraDriverPara);
			bRtn = bRtn && LoadCameraTransfomParam(opini, tCameraPara.tCameraTransfomPara);
			break;
		case E_VZENSE_CAMERA:
			bRtn = bRtn && LoadDHCameraParam(opini, tCameraPara.bAutoOpen, tCameraPara.nInstallPos, tCameraPara.eFlipMode, tCameraPara.tCameraTool);
			bRtn = bRtn && LoadCameraTransfomParam(opini, tCameraPara.tCameraTransfomPara);
			break;
		default:
			break;
		}

		bRtn = bRtn && LoadCameraIOParam(opini, tCameraPara.tCameraIO);

		vtCameraPara.push_back(tCameraPara);
	}
	return bRtn;
}

BOOL CContralUnit::LoadCameraBaseParam(COPini& opini, T_HAND_EYE_CALI_PARAM& tHandEyeCaliPara)
{
	BOOL bRtn = TRUE;
	bRtn = bRtn && opini.ReadAddString("KEY_DX", &tHandEyeCaliPara.tCameraInnerPara.dPixelX, 0);
	bRtn = bRtn && opini.ReadAddString("KEY_DY", &tHandEyeCaliPara.tCameraInnerPara.dPixelY, 0);
	bRtn = bRtn && opini.ReadAddString("KEY_F", &tHandEyeCaliPara.tCameraInnerPara.dFocal, 0);
	bRtn = bRtn && opini.ReadAddString("KEY_LEN", &tHandEyeCaliPara.tCameraInnerPara.dBaseLineLength, 0);
	bRtn = bRtn && opini.ReadAddString("KEY_CTAN", &tHandEyeCaliPara.tCameraInnerPara.dCtanBaseLineAngle, 0);
	return bRtn;
}
BOOL CContralUnit::LoadCameraHandEyeCaliParam(COPini& opini, T_HAND_EYE_CALI_PARAM& tHandEyeCaliPara)
{
	BOOL bRtn = TRUE;

	bRtn = bRtn && opini.ReadString("KEY_CAMERA_READY_ROBOT_POS_D", "", tHandEyeCaliPara.tCameraReadyRobotCoors, T_ROBOT_COORS(1, 1, 1, 1, 1, 1, -1, -1, -1));
	bRtn = bRtn && opini.ReadString("KEY_CAMERA_READY_ROBOT_", "_PULSE", tHandEyeCaliPara.tCameraReadyRobotPulse, T_ANGLE_PULSE(1, 1, 1, 1, 1, 1, -1, -1, -1));
	tHandEyeCaliPara.tCameraReadyRobotTheta = GetRobotCtrl()->PulseToTheta(tHandEyeCaliPara.tCameraReadyRobotPulse);
	bRtn = bRtn && opini.ReadString("KEY_MOVE_TO_CAMERA_READY_ROBOT_POS_D", "", tHandEyeCaliPara.tGunMoveToCameraCenterRobotCoors, T_ROBOT_COORS(1, 1, 1, 1, 1, 1, -1, -1, -1));
	
	bRtn = bRtn && opini.ReadAddString("KEY_N_CAM_X_AXIS_DIR_IN_ROBOT_BASE", &tHandEyeCaliPara.nCameraXAxisInRobotCoordinate, 0);
	bRtn = bRtn && opini.ReadAddString("KEY_N_CAM_Y_AXIS_DIR_IN_ROBOT_BASE", &tHandEyeCaliPara.nCameraYAxisInRobotCoordinate, 0);
	bRtn = bRtn && opini.ReadAddString("KEY_N_CAM_Z_AXIS_DIR_IN_ROBOT_BASE", &tHandEyeCaliPara.nCameraZAxisInRobotCoordinate, 0);

	tHandEyeCaliPara.eManipulatorType = GetRobotCtrl()->m_eManipulatorType;
	tHandEyeCaliPara.nExternalAxisType = GetRobotCtrl()->m_nExternalAxleType;
	return bRtn;
}
BOOL CContralUnit::LoadCameraTransfomParam(COPini& opini, T_CAMERA_TRANSFORM_PARAM &tCameraTransfomPara)
{
	BOOL bRtn = TRUE;
	bRtn = bRtn && opini.ReadAddString("RotateDegree", &tCameraTransfomPara.dRotateDegree, 0);
	bRtn = bRtn && opini.ReadAddString("DirX", &tCameraTransfomPara.dDirX, 0);
	bRtn = bRtn && opini.ReadAddString("DirY", &tCameraTransfomPara.dDirY, 0);
	bRtn = bRtn && opini.ReadAddString("DirZ", &tCameraTransfomPara.dDirZ, 0);
	bRtn = bRtn && opini.ReadAddString("X", &tCameraTransfomPara.dX, 0);
	bRtn = bRtn && opini.ReadAddString("Y", &tCameraTransfomPara.dY, 0);
	bRtn = bRtn && opini.ReadAddString("Z", &tCameraTransfomPara.dZ, 0);
	for (size_t i = 0; i < 4; i++)
	{
		for (size_t j = 0; j < 4; j++)
		{
			bRtn = bRtn && opini.ReadAddString(GetStr("TransMatrix%d_%d", i, j), &tCameraTransfomPara.dMatrix[i][j], 0);
		}
	}
	return bRtn;
}
BOOL CContralUnit::LoadCameraDistortionParam(COPini& opini, T_CAMERA_DISTORTION& tCameraDistortion)
{
	BOOL bRtn = TRUE;
	double dPara = 0.0;
	tCameraDistortion.cvCameraMatrix = cv::Mat::eye(3, 3, CV_64F);
	tCameraDistortion.cvDistCoeffs = cv::Mat::zeros(5, 1, CV_64F);
	bRtn = bRtn && opini.ReadAddString("EnableUndistort", &tCameraDistortion.bEnableUndistort, 0);
	bRtn = bRtn && opini.ReadAddString("CameraMatrix0_0", &dPara, 0);
	tCameraDistortion.cvCameraMatrix.at<double>(0, 0) = dPara;
	bRtn = bRtn && opini.ReadAddString("CameraMatrix0_1", &dPara, 0);
	tCameraDistortion.cvCameraMatrix.at<double>(0, 1) = dPara;
	bRtn = bRtn && opini.ReadAddString("CameraMatrix0_2", &dPara, 0);
	tCameraDistortion.cvCameraMatrix.at<double>(0, 2) = dPara;
	bRtn = bRtn && opini.ReadAddString("CameraMatrix1_0", &dPara, 0);
	tCameraDistortion.cvCameraMatrix.at<double>(1, 0) = dPara;
	bRtn = bRtn && opini.ReadAddString("CameraMatrix1_1", &dPara, 0);
	tCameraDistortion.cvCameraMatrix.at<double>(1, 1) = dPara;
	bRtn = bRtn && opini.ReadAddString("CameraMatrix1_2", &dPara, 0);
	tCameraDistortion.cvCameraMatrix.at<double>(1, 2) = dPara;
	bRtn = bRtn && opini.ReadAddString("CameraMatrix2_0", &dPara, 0);
	tCameraDistortion.cvCameraMatrix.at<double>(2, 0) = dPara;
	bRtn = bRtn && opini.ReadAddString("CameraMatrix2_1", &dPara, 0);
	tCameraDistortion.cvCameraMatrix.at<double>(2, 1) = dPara;
	bRtn = bRtn && opini.ReadAddString("CameraMatrix2_2", &dPara, 0);
	tCameraDistortion.cvCameraMatrix.at<double>(2, 2) = dPara;
	bRtn = bRtn && opini.ReadAddString("DistCoeffs0_0", &dPara, 0);
	tCameraDistortion.cvDistCoeffs.at<double>(0, 0) = dPara;
	bRtn = bRtn && opini.ReadAddString("DistCoeffs1_0", &dPara, 0);
	tCameraDistortion.cvDistCoeffs.at<double>(1, 0) = dPara;
	bRtn = bRtn && opini.ReadAddString("DistCoeffs2_0", &dPara, 0);
	tCameraDistortion.cvDistCoeffs.at<double>(2, 0) = dPara;
	bRtn = bRtn && opini.ReadAddString("DistCoeffs3_0", &dPara, 0);
	tCameraDistortion.cvDistCoeffs.at<double>(3, 0) = dPara;
	bRtn = bRtn && opini.ReadAddString("DistCoeffs4_0", &dPara, 0);
	tCameraDistortion.cvDistCoeffs.at<double>(4, 0) = dPara;
	return bRtn;
}
BOOL CContralUnit::LoadLaserPlanEquation(COPini& opini, T_LASER_PLAN_EQUATION& tLaserPlanEquation)
{
	BOOL bRtn = TRUE;
	tLaserPlanEquation.vdLineLaserCalibrationResult.clear();
	for (size_t i = 0; i < 4; i++)
	{
		double dPara;
		bRtn = bRtn && opini.ReadAddString(GetStr("LineLaserCalibrationResult%d", i), &dPara, 0);
		tLaserPlanEquation.vdLineLaserCalibrationResult.push_back(dPara);
	}
	return bRtn;
}

BOOL CContralUnit::LoadCameraIOParam(COPini& opini, T_CAMERA_IO& tCameraIO)
{
	BOOL bRtn = TRUE;
	int nLeaserNum = 0;
	bRtn = bRtn && opini.ReadAddString("SupLEDIO", &tCameraIO.tOutputIO.nSupLEDIO, -1);
	bRtn = bRtn && opini.ReadAddString("SupLED1IO", &tCameraIO.tOutputIO.nSupLED1IO, -1);
	bRtn = bRtn && opini.ReadAddString("SupLED2IO", &tCameraIO.tOutputIO.nSupLED2IO, -1);
	bRtn = bRtn && opini.ReadAddString("AirSolenoidValueIO", &tCameraIO.tOutputIO.nAirSolenoidValueIO, -1);
	bRtn = bRtn && opini.ReadAddString("AntisplashIO", &tCameraIO.tOutputIO.nAntisplashIO, -1);
	bRtn = bRtn && opini.ReadAddString("AntisplashOpenedIO", &tCameraIO.tInputIO.nAntisplashOpenedIO, -1);
	bRtn = bRtn && opini.ReadAddString("AntisplashClosedIO", &tCameraIO.tInputIO.nAntisplashClosedIO, -1);
	bRtn = bRtn && opini.ReadAddString("CameraPowerIO", &tCameraIO.tOutputIO.m_nCameraPowerIO, -1);
	bRtn = bRtn && opini.ReadAddString("LeaserLineNum", &nLeaserNum, 0);
	tCameraIO.vsLaserIOName.clear();
	for (int n = 0; n < nLeaserNum; n++)
	{
		int nTemp;
		bRtn = bRtn && opini.ReadAddString(GetStr("LeaserLine%d", n), &nTemp, -1);
		tCameraIO.tOutputIO.vnLeaseLineIO.push_back(nTemp);
		CString sLaserIOName = "";
		bRtn = bRtn && opini.ReadAddString(GetStr("LaserIOName%d", n), sLaserIOName, "null");
		tCameraIO.vsLaserIOName.push_back(sLaserIOName);
	}
	return bRtn;
}
CLog* CContralUnit::GetLog()
{
	if (m_cUnitLog != NULL)
	{
		return m_cUnitLog;
	}
	return ReturnCLog();
}
CString CContralUnit::GetFolderPath()
{
	return DATA_PATH + m_tContralUnit.strUnitName;
}
CString CContralUnit::GetUnitName()
{
	return m_tContralUnit.strUnitName;
}

bool CContralUnit::LoadDHCompen(COPini& opini, XI_POINT &tPoint)
{
	BOOL bRtn = TRUE;
	bRtn = bRtn && opini.ReadAddString("PointCloudCompenX ", &tPoint.x, 0);
	bRtn = bRtn && opini.ReadAddString("PointCloudCompenY ", &tPoint.y, 0);
	bRtn = bRtn && opini.ReadAddString("PointCloudCompenZ ", &tPoint.z, 0);
	return bRtn == TRUE;
}

bool CContralUnit::LoadDHCompenNew(COPini& opini, double ppdHandEyeCompen[][4])
{
	BOOL bRtn = TRUE;
	for (size_t i = 0; i < 4; i++)
	{
		for (size_t j = 0; j < 4; j++)
		{
			CString sKey;
			sKey.Format("HandEyeCompen%d_%d", i, j);
			bRtn = bRtn && opini.ReadAddString(sKey, &(ppdHandEyeCompen[i][j]), i == j ? 1 : 0);
		}
	}
	return bRtn == TRUE;
}

bool CContralUnit::LoadDHCameraParam(COPini& opini, bool& bAutoOpen, int &nInstallPos, E_FLIP_MODE& eFilpMode, T_ROBOT_COORS& tCameraTool)
{
	BOOL bRtn = TRUE;
	int nFlipMode = 0;
	bRtn = bRtn && opini.ReadAddString("AutoOpen", &bAutoOpen, 0);
	bRtn = bRtn && opini.ReadAddString("InstallPos", &nInstallPos, 0);
	bRtn = bRtn && opini.ReadAddString("ImageFlipMode", &nFlipMode, 0);
	eFilpMode = (E_FLIP_MODE)nFlipMode;
	bRtn = bRtn && opini.ReadAddString("CameraTool_dX ", &tCameraTool.dX , 0);
	bRtn = bRtn && opini.ReadAddString("CameraTool_dY ", &tCameraTool.dY , 0);
	bRtn = bRtn && opini.ReadAddString("CameraTool_dZ ", &tCameraTool.dZ , 0);
	bRtn = bRtn && opini.ReadAddString("CameraTool_dRX", &tCameraTool.dRX, 0);
	bRtn = bRtn && opini.ReadAddString("CameraTool_dRY", &tCameraTool.dRY, 0);
	bRtn = bRtn && opini.ReadAddString("CameraTool_dRZ", &tCameraTool.dRZ, 0);
	if (nInstallPos < -1 || nInstallPos > 5)
	{
		nInstallPos = 5;
		//已修改
		XUI::MesBox::PopInfo("相机安装位置参数错误!已使用默认安装到法兰上!");
		//XiMessageBoxOk("相机安装位置参数错误!已使用默认安装到法兰上!");
	}
	return bRtn == TRUE;
}

BOOL CContralUnit::LoadDHCameraParam(COPini& opini, T_DH_CAMREA_DRIVER_PARAM& tDHCameraDriverPara)
{
	BOOL bRtn = TRUE;
	bRtn = bRtn && opini.ReadAddString("CameraAcquisitionMode", &tDHCameraDriverPara.nCameraAcquisitionMode, 0);
	if (tDHCameraDriverPara.nCameraAcquisitionMode >= (int)E_DHGIGE_ACQUISITION_MODE_MAX_NUM || tDHCameraDriverPara.nCameraAcquisitionMode < 0)
	{
		WRITE_LOG_UNIT(GetStr("相机 %d 类型错误", tDHCameraDriverPara.nCameraAcquisitionMode));
		tDHCameraDriverPara.nCameraAcquisitionMode = 0;
	}
	bRtn = bRtn && opini.ReadAddString("CallBackMode", &tDHCameraDriverPara.nCallBackMode, 0);
	if (tDHCameraDriverPara.nCallBackMode >= (int)E_DHGIGE_CALL_BACK_MAX_NUM || tDHCameraDriverPara.nCallBackMode < 0)
	{
		WRITE_LOG_UNIT(GetStr("相机 %d 类型错误", tDHCameraDriverPara.nCallBackMode));
		tDHCameraDriverPara.nCallBackMode = 0;
	}
	bRtn = bRtn && opini.ReadAddString("DeviceAddress", tDHCameraDriverPara.strDeviceAddress, "127.0.0.1");
	bRtn = bRtn && opini.ReadAddString("ExposureTime", &tDHCameraDriverPara.dExposureTime, 0);
	bRtn = bRtn && opini.ReadAddString("GainLevel", &tDHCameraDriverPara.dGainLevel, 0);
	bRtn = bRtn && opini.ReadAddString("RoiWidth", &tDHCameraDriverPara.nRoiWidth, 0);
	bRtn = bRtn && opini.ReadAddString("RoiHeight", &tDHCameraDriverPara.nRoiHeight, 0);
	bRtn = bRtn && opini.ReadAddString("MaxWidth", &tDHCameraDriverPara.nMaxWidth, 0);
	bRtn = bRtn && opini.ReadAddString("MaxHeight", &tDHCameraDriverPara.nMaxHeight, 0);
	bRtn = bRtn && opini.ReadAddString("RoiOffsetX", &tDHCameraDriverPara.nRoiOffsetX, 0);
	bRtn = bRtn && opini.ReadAddString("RoiOffsetY", &tDHCameraDriverPara.nRoiOffsetY, 0);
	bRtn = bRtn && opini.ReadAddString("StrobeLine1", &tDHCameraDriverPara.nStrobeLine1, 0);
	bRtn = bRtn && opini.ReadAddString("StrobeLine2", &tDHCameraDriverPara.nStrobeLine2, 0);
	return bRtn;
}

BOOL CContralUnit::LoadDepthCameraParam(COPini& opini, T_DEPTH_CAMREA_DRIVER_PARAM& tPANOCameraDriverPara)
{
	BOOL bRtn = TRUE;
	bRtn = bRtn && opini.ReadAddString("DepthCameraType", &tPANOCameraDriverPara.nCameraType, 0);
	bRtn = bRtn && opini.ReadAddString("DepthWidth", &tPANOCameraDriverPara.nDepthWidth, 0);
	bRtn = bRtn && opini.ReadAddString("DepthHeight", &tPANOCameraDriverPara.nDepthHeight, 0);
	bRtn = bRtn && opini.ReadAddString("ColorWidth", &tPANOCameraDriverPara.nColorWidth, 0);
	bRtn = bRtn && opini.ReadAddString("ColorHeight", &tPANOCameraDriverPara.nColorHeight, 0);
	bRtn = bRtn && opini.ReadAddString("InfraredWidth", &tPANOCameraDriverPara.nInfraredWidth, 0);
	bRtn = bRtn && opini.ReadAddString("InfraredHeight", &tPANOCameraDriverPara.nInfraredHeight, 0);
	return bRtn;
}

BOOL CContralUnit::LoadMecheyeCameraParam(COPini& opini, T_MECHEYE_CAMREA_DRIVER_PARAM& tMecheyeCameraDriverPara)
{
	BOOL bRtn = TRUE;
	bRtn = bRtn && opini.ReadAddString("DeviceAddress", tMecheyeCameraDriverPara.strDeviceAddress, "127.0.0.1");
	bRtn = bRtn && opini.ReadAddString("DevicePort", &tMecheyeCameraDriverPara.nPort, 0);
	return bRtn;
}

int CContralUnit::InitContralResource()
{
	//初始化日志
	CString sLogPath = "Monitor\\" + m_tContralUnit.strUnitName + "\\";
	m_cUnitLog = new CLog(sLogPath);

	BOOL bRtn = TRUE;
	int nRtn = 0;

	if (m_tContralUnit.strUnitType.Find("N") != -1)
	{
		return 0;
	}


	if (m_pServoMotorDriver == NULL)
	{
		if (!g_bLocalDebugMark)
		{
			MESSAGE_BOX_UNIT("未初始化控制卡驱动，IO将无效");
		}
	}
	m_pBasicIOControl = new CBasicIOControl(m_pServoMotorDriver);

	bRtn = bRtn && LoadBaseIOParam(m_tContralUnit.strUnitName, m_tCommonIO);
	CHECK_BOOL_BOX_RTN_UNIT(bRtn, "加载基础IO失败");

	bRtn = bRtn && LoadDeviceIO();
	CHECK_BOOL_BOX_RTN_UNIT(bRtn, "加载设备IO失败");

	bRtn = bRtn && LoadRobotIOParam();
	CHECK_BOOL_BOX_RTN_UNIT(bRtn, "加载控制单元" + m_tContralUnit.strUnitName + "IO参数失败");

	if (m_tContralUnit.strUnitType.Find("O") != -1)
	{
		return 0;
	}

	//初始化控制
	if (m_tContralUnit.strUnitType.Find("S") != -1)
	{
		bRtn = bRtn && LoadMotorParam(m_tContralUnit.strUnitName, m_vtMotorPara);
		CHECK_BOOL_BOX_RTN_UNIT(bRtn, "加载电机参数失败");
		if (m_vtMotorPara.empty())
		{
			MESSAGE_BOX_UNIT("选择了伺服控制，但没有配置相应参数");
		}
		else if (m_pServoMotorDriver == NULL)
		{
			MESSAGE_BOX_UNIT("选择了伺服控制，但没有初始化相应驱动");
		}
		else
		{
			for (int n = 0; n < m_vtMotorPara.size(); n++)
			{
				CUnitDriver *temp;
				temp = new CUnitDriver(m_vtMotorPara[n], m_pServoMotorDriver);
				temp->SetSevon(true);
				m_pContralCard.push_back(temp);
			}
		}
	}
	if (m_tContralUnit.strUnitType.Find("R") != -1)
	{
		bRtn = bRtn && LoadRobotIOParam(m_tContralUnit.strUnitName, m_tCommonIO.tYaskawaRobotIO);
		CHECK_BOOL_BOX_RTN_UNIT(bRtn, "加载机器人IO失败");
		//m_pYasakawaRobotDriver = new CRobotDriverAdaptor(m_tContralUnit.strUnitName, m_cUnitLog, &m_pContralCard);
		m_pYasakawaRobotDriver = InitRobot(m_tContralUnit.strUnitName, m_cUnitLog, &m_pContralCard);
	}

	//加载参数
	bRtn = bRtn && LoadCameraParam(m_tContralUnit.strUnitName, m_vtCameraPara);
	CHECK_BOOL_BOX_RTN_UNIT(bRtn, "加载相机参数失败");

	//初始化相机
	nRtn = InitCamera();
	CHECK_INT_BOX_RTN_UNIT(nRtn, "初始化相机失败");
	return 0;
}

CRobotDriverAdaptor* CContralUnit::InitRobot(CString strUnitName, CLog* cLog, std::vector<CUnitDriver*>* ppUnitDriver)
{
	int nMaxRobotNo = 0;
	int nRobotBrand = 0;
	COPini opini;
	COPini cIni;
	CString strRobotName;
	cIni.SetFileName(DATA_PATH + strUnitName + ROBOT_PARA_INI);
	cIni.SetSectionName("BaseParam");
	cIni.ReadString("RobotBrand", &nRobotBrand);

	CRobotDriverAdaptor* pRobotDriver = NULL;
	switch (nRobotBrand)
	{
	case 1: pRobotDriver = new CRobotDriverAdaptor(strUnitName, cLog, ppUnitDriver);
		break;
	case 2: pRobotDriver = new EstunAdaptor(strUnitName, cLog, ppUnitDriver);
		break;
	default: XiMessageBox("机器人品牌错误!");
		break;
	}
	return pRobotDriver;
}

int CContralUnit::InitCamera()
{
	int nDHCameraNum = 0;
	int nDepthCameraNum = 0;
	int nMecheyeCameraNum = 0;
	for (size_t i = 0; i < m_vtCameraPara.size(); i++)
	{
		switch (m_vtCameraPara[i].eCameraType)
		{
		case E_DH_CAMERA:
			CHECK_INT_BOX_RTN_UNIT(InitDHCamera(m_vtCameraPara[i]), GetStr("初始化相机 %d 失败", i));
			m_vtCameraPara[i].nSameTypeNo = nDHCameraNum++;
			break;
		case E_KINECT_CAMERA:
			CHECK_INT_BOX_RTN_UNIT(InitDepthCamera(m_vtCameraPara[i].tPANOCameraDriverPara), GetStr("初始化相机 %d 失败", i));
			m_vtCameraPara[i].nSameTypeNo = nDepthCameraNum++;
			break;
		case E_MECHEYE_CAMERA:
			CHECK_INT_BOX_RTN_UNIT(InitMecheyeCamera(m_vtCameraPara[i].tMecheyeCameraDriverPara), GetStr("初始化相机 %d 失败", i));
			m_vtCameraPara[i].nSameTypeNo = nMecheyeCameraNum++;
			break;
		default:
			break;
		}
	}
	return 0;
}

int CContralUnit::InitDHCamera(T_CAMREA_PARAM tCameraPara)
{
	CDHGigeImageCapture* pDHCamera = NULL;
	pDHCamera = new CDHGigeImageCapture(m_cUnitLog, tCameraPara);
	if (!GetLocalDebugMark() && tCameraPara.bAutoOpen)
	{
		T_DH_CAMREA_DRIVER_PARAM tDHCameraDriverPara = tCameraPara.tDHCameraDriverPara;
		if (tDHCameraDriverPara.nCameraAcquisitionMode >= 0 && tDHCameraDriverPara.nCameraAcquisitionMode <= E_DHGIGE_ACQUISITION_MODE_MAX_NUM &&
			tDHCameraDriverPara.nCallBackMode >= 0 && tDHCameraDriverPara.nCallBackMode <= E_DHGIGE_CALL_BACK_MAX_NUM)
		{
			pDHCamera->InitCam(
				(E_DHGIGE_ACQUISITION_MODE)tDHCameraDriverPara.nCameraAcquisitionMode,
				(E_DHGIGE_CALL_BACK)tDHCameraDriverPara.nCallBackMode);
		}
		else
		{
			MESSAGE_BOX_UNIT(GetStr("相机参数异常：CameraAcquisitionMode:%d, CallBackMode:%d",
				tDHCameraDriverPara.nCameraAcquisitionMode,
				tDHCameraDriverPara.nCallBackMode));
			return -1;
		}
	}
	m_vpImageCapture.push_back(pDHCamera);
	return 0;
}

int CContralUnit::SwitchDHCamera(int nCameraNo, bool bOpen, bool bSwitchLaser, E_DHGIGE_ACQUISITION_MODE eCaptureMode, E_DHGIGE_CALL_BACK eCallBackMode)
{
	if (nCameraNo < 0 || nCameraNo >= m_vtCameraPara.size() || nCameraNo >= m_vpImageCapture.size())
	{
		return -1;
	}
	// 获取相机参数 和 大恒相机指针
	T_CAMREA_PARAM tDHCameraPara = m_vtCameraPara[nCameraNo];
	CDHGigeImageCapture* pDHCamera = m_vpImageCapture[nCameraNo];
	if (!GetLocalDebugMark())
	{
		
		if ((bOpen) && 
			(eCaptureMode >= 0) && (eCaptureMode <= E_DHGIGE_ACQUISITION_MODE_MAX_NUM) &&
			(eCallBackMode >= 0) && (eCallBackMode <= E_DHGIGE_CALL_BACK_MAX_NUM))
		{
			pDHCamera->InitCam(eCaptureMode, eCallBackMode);
		}
		else if (!bOpen)
		{
			pDHCamera->UnInitCam();
		}
		else
		{
			MESSAGE_BOX_UNIT(GetStr("相机参数异常：CameraAcquisitionMode:%d, CallBackMode:%d", eCaptureMode, eCallBackMode));
			return -2;
		}
		for (int i = 0; bSwitchLaser && (i < tDHCameraPara.tCameraIO.vsLaserIOName.size()); i++)
		{
			SwitchIO(tDHCameraPara.tCameraIO.vsLaserIOName[i], bOpen);
		}
	}
	return 0;
}

int CContralUnit::InitDepthCamera(T_DEPTH_CAMREA_DRIVER_PARAM tDepthCameraPara)
{
	CKinectControl* pKinectControl;
	pKinectControl = new CKinectControl(tDepthCameraPara);
	if (!GetLocalDebugMark())
	{
		if (tDepthCameraPara.nCameraType <= 3 && tDepthCameraPara.nCameraType >= 2)
		{
			pKinectControl->OpenKinect();
		}
		else
		{
			MESSAGE_BOX_UNIT(GetStr("相机参数异常：CameraType:%d", tDepthCameraPara.nCameraType));
			return -1;
		}
	}
	m_vpKinectControl.push_back(pKinectControl);
	return 0;
}

int CContralUnit::InitMecheyeCamera(T_MECHEYE_CAMREA_DRIVER_PARAM& tMecheyeCameraDriverPara)
{
#if ENABLE_MECH_EYE
	CMechEyeCtrl* pMechMindDevice;
	pMechMindDevice = new CMechEyeCtrl(tMecheyeCameraDriverPara);
	if (!GetLocalDebugMark())
	{
		if (0 != pMechMindDevice->InitMechEyeCamera())
		{
			MESSAGE_BOX_UNIT(GetStr("相机参数异常：DeviceAddress:%s,Port：%d", tMecheyeCameraDriverPara.strDeviceAddress, tMecheyeCameraDriverPara.nPort));
			return -1;
		}
	}
	m_vpMechMindDevice.push_back(pMechMindDevice);
#else
	MESSAGE_BOX_UNIT("MechEye 已经被裁剪，无法初始化！");
#endif
	return 0;
}

bool CContralUnit::GetRobotEnableState()
{
	if (m_pYasakawaRobotDriver != NULL)
	{
		return true;
	}
	return false;
}

bool CContralUnit::GetMotorEnableState()
{
	if (!m_pContralCard.empty())
	{
		return true;
	}
	return false;
}

int CContralUnit::ReadInbit(int nIONo, WORD& wState)
{
	CHECK_IO_RTN_UNIT(nIONo);
	CHECK_CTRL_CARD_RTN_UNIT;

	return m_pBasicIOControl->ReadInbit(nIONo, wState);
}

int CContralUnit::ReadOutbit(int nIONo, WORD& wState)
{
	CHECK_IO_RTN_UNIT(nIONo);
	CHECK_CTRL_CARD_RTN_UNIT;

	return m_pBasicIOControl->ReadOutbit(nIONo, wState);
}

int CContralUnit::WriteOutbit(int nIONo, WORD wState)
{
	CHECK_IO_RTN_UNIT(nIONo);
	CHECK_CTRL_CARD_RTN_UNIT;

	return m_pBasicIOControl->WriteOutbit(nIONo, wState);
}

int CContralUnit::GetAllEmgStopState(WORD& wState)
{
	CHECK_IO_RTN_UNIT(m_tCommonIO.tEmgStopIO.nAllEmgStopIO);
	CHECK_CTRL_CARD_RTN_UNIT;

	return m_pBasicIOControl->ReadInbit(m_tCommonIO.tEmgStopIO.nAllEmgStopIO, wState);
}

int CContralUnit::OpenAirSolenoidValue()
{
	CHECK_IO_RTN_UNIT(m_tCommonIO.tGasIO.nAirSolenoidIO);
	CHECK_CTRL_CARD_RTN_UNIT;

	return m_pBasicIOControl->WriteOutbit(m_tCommonIO.tGasIO.nAirSolenoidIO, ON);
}

int CContralUnit::CloseAirSolenoidValue()
{
	CHECK_IO_RTN_UNIT(m_tCommonIO.tGasIO.nAirSolenoidIO);
	CHECK_CTRL_CARD_RTN_UNIT;

	return m_pBasicIOControl->WriteOutbit(m_tCommonIO.tGasIO.nAirSolenoidIO, OFF);
}

int CContralUnit::GetAirPressureSignal(WORD &wState)
{
	CHECK_IO_RTN_UNIT(m_tCommonIO.tGasIO.nAirPressureSignalIO);
	CHECK_CTRL_CARD_RTN_UNIT;

	return m_pBasicIOControl->ReadInbit(m_tCommonIO.tGasIO.nAirPressureSignalIO, wState);
}

int CContralUnit::OpenRobotControlCabinetPower()
{
	CHECK_IO_RTN_UNIT(m_tCommonIO.tRobtPowerIO.nRobotExternalPowerControl);
	CHECK_CTRL_CARD_RTN_UNIT;

	WORD wState;
	ReadOutbit(m_tCommonIO.tRobtPowerIO.nRobotExternalPowerControl, wState);
	if (ON == wState)
	{
		return true;
	}

	return m_pBasicIOControl->WriteOutbit(m_tCommonIO.tRobtPowerIO.nRobotExternalPowerControl, ON);
}

int CContralUnit::SetCautionLightGreen(WORD on_off)
{
	return m_pBasicIOControl->WriteOutbit(m_tCommonIO.tWarningLightIO.nCautionLightGreenIO_OUT, on_off);
}

int CContralUnit::SetCautionLightYellow(WORD on_off)
{
	return m_pBasicIOControl->WriteOutbit(m_tCommonIO.tWarningLightIO.nCautionLightYellowIO_OUT, on_off);
}

int CContralUnit::SetCautionLightRed(WORD on_off)
{
	return m_pBasicIOControl->WriteOutbit(m_tCommonIO.tWarningLightIO.nCautionLightRedIO_OUT, on_off);
}

int CContralUnit::OpenCameraPower(int nCameraNo)
{
	CHECK_IO_RTN_UNIT(m_vtCameraPara[nCameraNo].tCameraIO.tOutputIO.m_nCameraPowerIO);
	CHECK_CTRL_CARD_RTN_UNIT;

	return m_pBasicIOControl->WriteOutbit(m_vtCameraPara[nCameraNo].tCameraIO.tOutputIO.m_nCameraPowerIO, ON);
}

int CContralUnit::CloseCameraPower(int nCameraNo)
{
	CHECK_IO_RTN_UNIT(m_vtCameraPara[nCameraNo].tCameraIO.tOutputIO.m_nCameraPowerIO);
	CHECK_CTRL_CARD_RTN_UNIT;
	return m_pBasicIOControl->WriteOutbit(m_vtCameraPara[nCameraNo].tCameraIO.tOutputIO.m_nCameraPowerIO, OFF);
}

int CContralUnit::OpenSupLED(int nCameraNo)
{
	CHECK_IO_RTN_UNIT(m_vtCameraPara[nCameraNo].tCameraIO.tOutputIO.nSupLEDIO);
	CHECK_CTRL_CARD_RTN_UNIT;
	return m_pBasicIOControl->WriteOutbit(m_vtCameraPara[nCameraNo].tCameraIO.tOutputIO.nSupLEDIO, ON);
}

int CContralUnit::CloseSupLED(int nCameraNo)
{
	CHECK_IO_RTN_UNIT(m_vtCameraPara[nCameraNo].tCameraIO.tOutputIO.nSupLEDIO);
	CHECK_CTRL_CARD_RTN_UNIT;
	return m_pBasicIOControl->WriteOutbit(m_vtCameraPara[nCameraNo].tCameraIO.tOutputIO.nSupLEDIO, OFF);
}

int CContralUnit::OpenSupLEDCtrlSignalPower(int nCameraNo)
{
	CHECK_IO_RTN_UNIT(m_vtCameraPara[nCameraNo].tCameraIO.tOutputIO.nSupLED1IO);
	CHECK_IO_RTN_UNIT(m_vtCameraPara[nCameraNo].tCameraIO.tOutputIO.nSupLED2IO);
	CHECK_CTRL_CARD_RTN_UNIT;

	return (m_pBasicIOControl->WriteOutbit(m_vtCameraPara[nCameraNo].tCameraIO.tOutputIO.nSupLED1IO, ON) | m_pBasicIOControl->WriteOutbit(m_vtCameraPara[nCameraNo].tCameraIO.tOutputIO.nSupLED1IO, ON));
}

int CContralUnit::CloseSupLEDCtrlSignalPower(int nCameraNo)
{
	CHECK_IO_RTN_UNIT(m_vtCameraPara[nCameraNo].tCameraIO.tOutputIO.nSupLED1IO);
	CHECK_IO_RTN_UNIT(m_vtCameraPara[nCameraNo].tCameraIO.tOutputIO.nSupLED2IO);
	CHECK_CTRL_CARD_RTN_UNIT;

	return (m_pBasicIOControl->WriteOutbit(m_vtCameraPara[nCameraNo].tCameraIO.tOutputIO.nSupLED1IO, OFF) | m_pBasicIOControl->WriteOutbit(m_vtCameraPara[nCameraNo].tCameraIO.tOutputIO.nSupLED1IO, OFF));
}

int CContralUnit::OpenLensCap(int nCameraNo)
{
	CHECK_IO_RTN_UNIT(m_vtCameraPara[nCameraNo].tCameraIO.tOutputIO.nAntisplashIO);
	CHECK_CTRL_CARD_RTN_UNIT;
	return m_pBasicIOControl->WriteOutbit(m_vtCameraPara[nCameraNo].tCameraIO.tOutputIO.nAntisplashIO, ON);
}

int CContralUnit::CloseLensCap(int nCameraNo)
{
	CHECK_IO_RTN_UNIT(m_vtCameraPara[nCameraNo].tCameraIO.tOutputIO.nAntisplashIO);
	CHECK_CTRL_CARD_RTN_UNIT;
	return m_pBasicIOControl->WriteOutbit(m_vtCameraPara[nCameraNo].tCameraIO.tOutputIO.nAntisplashIO, OFF);
}

int CContralUnit::GetLensCapState(WORD &wState, int nCameraNo)
{
	CHECK_IO_RTN_UNIT(m_vtCameraPara[nCameraNo].tCameraIO.tInputIO.nAntisplashClosedIO);
	CHECK_IO_RTN_UNIT(m_vtCameraPara[nCameraNo].tCameraIO.tInputIO.nAntisplashOpenedIO);
	CHECK_CTRL_CARD_RTN_UNIT;

	WORD wCloseState, wOpenState;
	int nRtn = m_pBasicIOControl->ReadInbit(m_vtCameraPara[nCameraNo].tCameraIO.tInputIO.nAntisplashClosedIO, wCloseState);
	CHECK_INT_BOX_RTN_UNIT(nRtn, "获取防飞溅挡板关闭信号失败");
	nRtn = m_pBasicIOControl->ReadInbit(m_vtCameraPara[nCameraNo].tCameraIO.tInputIO.nAntisplashOpenedIO, wOpenState);
	CHECK_INT_BOX_RTN_UNIT(nRtn, "获取防飞溅挡板打开信号失败");
	if (wCloseState == ON && wOpenState == OFF)
	{
		wState = -1;
	}
	else if (wCloseState == OFF && wOpenState == ON)
	{
		wState = 1;
	}
	else
	{
		wState = 0;
	}

	return wState;
}

int CContralUnit::GetLaserNum(int nCameraNo)
{
	return m_vtCameraPara[nCameraNo].tCameraIO.tOutputIO.vnLeaseLineIO.size();
}

int CContralUnit::OpenLaser(int nCameraNo, int nLeaserLineNo /*= 0*/)
{
	if (m_vtCameraPara[nCameraNo].tCameraIO.tOutputIO.vnLeaseLineIO.size() <= nLeaserLineNo || nLeaserLineNo < 0)
	{
		return -1;
	}
	CHECK_IO_RTN_UNIT(m_vtCameraPara[nCameraNo].tCameraIO.tOutputIO.vnLeaseLineIO[nLeaserLineNo]);
	CHECK_CTRL_CARD_RTN_UNIT;
	return m_pBasicIOControl->WriteOutbit(m_vtCameraPara[nCameraNo].tCameraIO.tOutputIO.vnLeaseLineIO[nLeaserLineNo], ON);
}

int CContralUnit::CloseLaser(int nCameraNo, int nLeaserLineNo /*= 0*/)
{
	if (m_vtCameraPara[nCameraNo].tCameraIO.tOutputIO.vnLeaseLineIO.size() <= nLeaserLineNo || nLeaserLineNo < 0)
	{
		return -1;
	}
	CHECK_IO_RTN_UNIT(m_vtCameraPara[nCameraNo].tCameraIO.tOutputIO.vnLeaseLineIO[nLeaserLineNo]);
	CHECK_CTRL_CARD_RTN_UNIT;
	return m_pBasicIOControl->WriteOutbit(m_vtCameraPara[nCameraNo].tCameraIO.tOutputIO.vnLeaseLineIO[nLeaserLineNo], OFF);
}

//int CContralUnit::OpenRobot()
//{
//	CHECK_IO_RTN_UNIT(m_tCommonIO.tYaskawaRobotIO.tRobotOutputIO.nRobotInitIO);
//	CHECK_CTRL_CARD_RTN_UNIT;
//
//	return m_pBasicIOControl->WriteOutbit(m_tCommonIO.tYaskawaRobotIO.tRobotOutputIO.nRobotInitIO, ON);
//}
//
//int CContralUnit::CloseRobot()
//{
//	CHECK_IO_RTN_UNIT(m_tCommonIO.tYaskawaRobotIO.tRobotOutputIO.nRobotInitIO);
//	CHECK_CTRL_CARD_RTN_UNIT;
//
//	return m_pBasicIOControl->WriteOutbit(m_tCommonIO.tYaskawaRobotIO.tRobotOutputIO.nRobotInitIO, OFF);
//}
//
//int CContralUnit::OpenCallJob()
//{
//	CHECK_IO_RTN_UNIT(m_tCommonIO.tYaskawaRobotIO.tRobotOutputIO.nCallJobIO);
//	CHECK_CTRL_CARD_RTN_UNIT;
//
//	return m_pBasicIOControl->WriteOutbit(m_tCommonIO.tYaskawaRobotIO.tRobotOutputIO.nCallJobIO, ON);
//}
//
//int CContralUnit::CloseCallJob()
//{
//	CHECK_IO_RTN_UNIT(m_tCommonIO.tYaskawaRobotIO.tRobotOutputIO.nCallJobIO);
//	CHECK_CTRL_CARD_RTN_UNIT;
//
//	return m_pBasicIOControl->WriteOutbit(m_tCommonIO.tYaskawaRobotIO.tRobotOutputIO.nCallJobIO, OFF);
//}
//
//int CContralUnit::OpenRobotPause()
//{
//	CHECK_IO_RTN_UNIT(m_tCommonIO.tYaskawaRobotIO.tRobotOutputIO.nRobotPauseIO);
//	CHECK_CTRL_CARD_RTN_UNIT;
//
//	return m_pBasicIOControl->WriteOutbit(m_tCommonIO.tYaskawaRobotIO.tRobotOutputIO.nRobotPauseIO, ON);
//}
//
//int CContralUnit::CloseRobotPause()
//{
//	CHECK_IO_RTN_UNIT(m_tCommonIO.tYaskawaRobotIO.tRobotOutputIO.nRobotPauseIO);
//	CHECK_CTRL_CARD_RTN_UNIT;
//
//	return m_pBasicIOControl->WriteOutbit(m_tCommonIO.tYaskawaRobotIO.tRobotOutputIO.nRobotPauseIO, OFF);
//}
//
//int CContralUnit::OpenExRobotEmg()
//{
//	CHECK_IO_RTN_UNIT(m_tCommonIO.tYaskawaRobotIO.tRobotOutputIO.nRobotExEmgIO);
//	CHECK_CTRL_CARD_RTN_UNIT;
//
//	return m_pBasicIOControl->WriteOutbit(m_tCommonIO.tYaskawaRobotIO.tRobotOutputIO.nRobotExEmgIO, ON);
//}
//
//int CContralUnit::CloseExRobotEmg()
//{
//	CHECK_IO_RTN_UNIT(m_tCommonIO.tYaskawaRobotIO.tRobotOutputIO.nRobotExEmgIO);
//	CHECK_CTRL_CARD_RTN_UNIT;
//
//	return m_pBasicIOControl->WriteOutbit(m_tCommonIO.tYaskawaRobotIO.tRobotOutputIO.nRobotExEmgIO, OFF);
//}
//
//int CContralUnit::OpenResetErrors()
//{
//	CHECK_IO_RTN_UNIT(m_tCommonIO.tYaskawaRobotIO.tRobotOutputIO.nResetErrorsIO);
//	CHECK_CTRL_CARD_RTN_UNIT;
//
//	return m_pBasicIOControl->WriteOutbit(m_tCommonIO.tYaskawaRobotIO.tRobotOutputIO.nResetErrorsIO, ON);
//}
//
//int CContralUnit::CloseResetErrors()
//{
//	CHECK_IO_RTN_UNIT(m_tCommonIO.tYaskawaRobotIO.tRobotOutputIO.nResetErrorsIO);
//	CHECK_CTRL_CARD_RTN_UNIT;
//
//	return m_pBasicIOControl->WriteOutbit(m_tCommonIO.tYaskawaRobotIO.tRobotOutputIO.nResetErrorsIO, OFF);
//}

//int CContralUnit::GetRobotRunningSignal(WORD &wState)
//{
//	//CHECK_IO_RTN_UNIT(m_tCommonIO.tYaskawaRobotIO.tRobotInputIO.nRobotRunningIO);
//	//CHECK_CTRL_CARD_RTN_UNIT;
//	//return m_pBasicIOControl->ReadInbit(m_tCommonIO.tYaskawaRobotIO.tRobotInputIO.nRobotRunningIO, wState);
//	return CheckInIO("Running");
//}

int CContralUnit::GetRobotServoSignal(WORD & wState)
{
	//CHECK_IO_RTN_UNIT(m_tCommonIO.tYaskawaRobotIO.tRobotInputIO.nRobotServoOnIO);
	//CHECK_CTRL_CARD_RTN_UNIT;
	//return m_pBasicIOControl->ReadInbit(m_tCommonIO.tYaskawaRobotIO.tRobotInputIO.nRobotServoOnIO, wState);
	wState = !(CheckInIO("Servo"));
	return true;
}

//int CContralUnit::GetRobotErrorsSignal(WORD & wState)
//{
//	CHECK_IO_RTN_UNIT(m_tCommonIO.tYaskawaRobotIO.tRobotInputIO.nErrorsIO);
//	CHECK_CTRL_CARD_RTN_UNIT;
//
//	return m_pBasicIOControl->ReadInbit(m_tCommonIO.tYaskawaRobotIO.tRobotInputIO.nErrorsIO, wState);
//}
//
//int CContralUnit::GetRobotBatteryWarningSignal(WORD &wState)
//{
//	CHECK_IO_RTN_UNIT(m_tCommonIO.tYaskawaRobotIO.tRobotInputIO.nBatteryWarningIO);
//	CHECK_CTRL_CARD_RTN_UNIT;
//
//	return m_pBasicIOControl->ReadInbit(m_tCommonIO.tYaskawaRobotIO.tRobotInputIO.nBatteryWarningIO, wState);
//}
//
//int CContralUnit::GetRobotOperationalOriginSignal(WORD &wState)
//{
//	CHECK_IO_RTN_UNIT(m_tCommonIO.tYaskawaRobotIO.tRobotInputIO.nOperationalOriginIO);
//	CHECK_CTRL_CARD_RTN_UNIT;
//
//	return m_pBasicIOControl->ReadInbit(m_tCommonIO.tYaskawaRobotIO.tRobotInputIO.nOperationalOriginIO, wState);
//}
//
//int CContralUnit::GetRobotMidwayStartSignal(WORD &wState)
//{
//	CHECK_IO_RTN_UNIT(m_tCommonIO.tYaskawaRobotIO.tRobotInputIO.nMidwayStartIO);
//	CHECK_CTRL_CARD_RTN_UNIT;
//
//	return m_pBasicIOControl->ReadInbit(m_tCommonIO.tYaskawaRobotIO.tRobotInputIO.nMidwayStartIO, wState);
//}
//
//int CContralUnit::GetRobotEmgStopSignal(WORD &wState)
//{
//	CHECK_IO_RTN_UNIT(m_tCommonIO.tYaskawaRobotIO.tRobotInputIO.nEmgStopIO);
//	CHECK_CTRL_CARD_RTN_UNIT;
//
//	return m_pBasicIOControl->ReadInbit(m_tCommonIO.tYaskawaRobotIO.tRobotInputIO.nEmgStopIO, wState);
//}
//
//int CContralUnit::GetRobotLongRangeModeSignal(WORD &wState)
//{
//	CHECK_IO_RTN_UNIT(m_tCommonIO.tYaskawaRobotIO.tRobotInputIO.nChoiceLongRangeModeIO);
//	CHECK_CTRL_CARD_RTN_UNIT;
//
//	return m_pBasicIOControl->ReadInbit(m_tCommonIO.tYaskawaRobotIO.tRobotInputIO.nChoiceLongRangeModeIO, wState);
//}
//
//int CContralUnit::GetRobotManualModeSignal(WORD &wState)
//{
//	CHECK_IO_RTN_UNIT(m_tCommonIO.tYaskawaRobotIO.tRobotInputIO.nChoiceManualModeIO);
//	CHECK_CTRL_CARD_RTN_UNIT;
//
//	return m_pBasicIOControl->ReadInbit(m_tCommonIO.tYaskawaRobotIO.tRobotInputIO.nChoiceManualModeIO, wState);
//}
//
//int CContralUnit::GetRobotTeachModeSignal(WORD &wState)
//{
//	CHECK_IO_RTN_UNIT(m_tCommonIO.tYaskawaRobotIO.tRobotInputIO.nChoiceTeachModeIO);
//	CHECK_CTRL_CARD_RTN_UNIT;
//
//	return m_pBasicIOControl->ReadInbit(m_tCommonIO.tYaskawaRobotIO.tRobotInputIO.nChoiceTeachModeIO, wState);
//}


CRobotDriverAdaptor * CContralUnit::GetRobotCtrl()
{
	return m_pYasakawaRobotDriver;
}

CUnitDriver * CContralUnit::GetMotorCtrl(int nnUnitMotorNoMotorNo)
{
	if (m_pContralCard.size() <= nnUnitMotorNoMotorNo)
	{
		MESSAGE_BOX_UNIT(GetStr("输入电机编号%d超出最大电机编号%d", nnUnitMotorNoMotorNo, m_pContralCard.size() - 1));
		return NULL;
	}
	return m_pContralCard[nnUnitMotorNoMotorNo];
}

int CContralUnit::AbsPosMove(int nUnitMotorNo, double dAbsPosture, double dSpeed, double dAcc, double dDec, double dSParam)
{
	CUnitDriver* pMotorCtrl = GetMotorCtrl(nUnitMotorNo);
	if (pMotorCtrl == NULL)
	{
		return -1;
	}
	return pMotorCtrl->PosMove(dAbsPosture, 1, dSpeed, dAcc, dDec, dSParam);
}

int CContralUnit::RelaPosMove(int nUnitMotorNo, double dRelaPosture, double dSpeed, double dAcc, double dDec, double dSParam)
{
	CUnitDriver* pMotorCtrl = GetMotorCtrl(nUnitMotorNo);
	if (pMotorCtrl == NULL)
	{
		return -1;
	}
	return pMotorCtrl->PosMove(dRelaPosture, 0, dSpeed, dAcc, dDec, dSParam);
}

BOOL CContralUnit::MotorCheckRunning(int nUnitMotorNo)
{
	CUnitDriver* pMotorCtrl = GetMotorCtrl(nUnitMotorNo);
	if (pMotorCtrl == NULL)
	{
		return FALSE;
	}
	return pMotorCtrl->CheckAxisRun();
}

BOOL CContralUnit::MotorCheckDone_CheckRobot(int nUnitMotorNo, double dAbsPosture)
{
	CUnitDriver* pMotorCtrl = GetMotorCtrl(nUnitMotorNo);
	if (pMotorCtrl == NULL)
	{
		return FALSE;
	}
	while (1)
	{
		Sleep(50);
		DoEvent();
		WORD wState;
		int nRtn = GetRobotServoSignal(wState);
		if (nRtn != 0 || wState == OFF)
		{
			Sleep(100);
			nRtn = GetRobotServoSignal(wState);
			if (nRtn != 0 || wState == OFF)
			{
				WRITE_LOG_UNIT("机器人伺服掉电导致外部轴急停！");
				pMotorCtrl->EmgStopAxis();
				return FALSE;
			}
		}
		if (!MotorCheckRunning(nUnitMotorNo))
		{
			Sleep(30);
			if (!MotorCheckRunning(nUnitMotorNo))
			{
				break;
			}
		}
	}
	double dCurrentPosition;
	int nRtn = pMotorCtrl->GetCurrentPosition(dCurrentPosition);
	if (!IsEqual(dAbsPosture, dCurrentPosition, 0.5))
	{
		return FALSE;
	}
	return TRUE;
}

BOOL CContralUnit::MotorCheckDone(int nUnitMotorNo, double dAbsPosture)
{
	CUnitDriver* pMotorCtrl = GetMotorCtrl(nUnitMotorNo);
	if (pMotorCtrl == NULL)
	{
		return FALSE;
	}
	return pMotorCtrl->CheckAxisDone(dAbsPosture);
}

BOOL CContralUnit::MotorCheckDone_CheckRobot(int nUnitMotorNo, double dAbsPosture, double dEarlyEndDistance)
{
	CUnitDriver* pMotorCtrl = GetMotorCtrl(nUnitMotorNo);
	if (pMotorCtrl == NULL)
	{
		return FALSE;
	}
	long long lTime = XI_clock();
	while (!pMotorCtrl->CheckAxisRun())
	{
		Sleep(10);
		DoEvent();
		if (XI_clock() - lTime > 500)
		{
			WRITE_LOG_UNIT(GetStr("MotorCheckDone未运动，轴:%d", nUnitMotorNo));
			return FALSE;
		}
	}
	while (pMotorCtrl->CheckAxisRun())
	{
		Sleep(50);
		DoEvent();
		WORD wState;
		int nRtn = GetRobotServoSignal(wState);
		if (nRtn != 0 || wState == OFF)
		{
			Sleep(100);
			nRtn = GetRobotServoSignal(wState);
			if (nRtn != 0 || wState == OFF)
			{
				WRITE_LOG_UNIT("机器人伺服掉电导致外部轴急停！");
				pMotorCtrl->EmgStopAxis();
				return FALSE;
			}
		}
		double dCurrentPosition;
		pMotorCtrl->GetCurrentPosition(dCurrentPosition);
		if (fabs(dAbsPosture - dCurrentPosition) < dEarlyEndDistance)
		{
			Sleep(100);
			pMotorCtrl->GetCurrentPosition(dCurrentPosition);
			if (fabs(dAbsPosture - dCurrentPosition) < dEarlyEndDistance)
			{
				break;
			}
		}
	}
	if (!pMotorCtrl->CheckAxisRun())
	{
		return pMotorCtrl->CheckAxisDone(dAbsPosture);
	}
	return TRUE;
}

BOOL CContralUnit::MotorCheckDone(int nUnitMotorNo, double dAbsPosture, double dEarlyEndDistance)
{
	CUnitDriver* pMotorCtrl = GetMotorCtrl(nUnitMotorNo);
	if (pMotorCtrl == NULL)
	{
		return FALSE;
	}
	long long lTime = XI_clock();
	while (!pMotorCtrl->CheckAxisRun())
	{
		Sleep(10);
		DoEvent();
		if (XI_clock() - lTime > 500)
		{
			WRITE_LOG_UNIT(GetStr("MotorCheckDone未运动，轴:%d", nUnitMotorNo));
			return FALSE;
		}
	}
	while (pMotorCtrl->CheckAxisRun())
	{
		double dCurrentPosition;
		pMotorCtrl->GetCurrentPosition(dCurrentPosition);
		if (fabs(dAbsPosture - dCurrentPosition) < dEarlyEndDistance)
		{
			Sleep(100);
			pMotorCtrl->GetCurrentPosition(dCurrentPosition);
			if (fabs(dAbsPosture - dCurrentPosition) < dEarlyEndDistance)
			{
				break;
			}
		}
	}
	if (!pMotorCtrl->CheckAxisRun())
	{
		return pMotorCtrl->CheckAxisDone(dAbsPosture);
	}
	return TRUE;
}

int CContralUnit::GetMotorNum()
{
	return m_pContralCard.size();
}

int CContralUnit::GetCurrentPosition(int nUnitMotorNo, double& dPosition)
{
	return GetMotorCtrl(nUnitMotorNo)->GetCurrentPosition(dPosition);
}

void* CContralUnit::GetCameraCtrl(int nCameraNo)
{
	if (m_vtCameraPara.size() <= nCameraNo)
	{
		MESSAGE_BOX_UNIT(GetStr("输入相机编号%d超出最大相机编号%d", nCameraNo, m_vtCameraPara.size() - 1));
		return nullptr;
	}
	int nSameTypeNo = m_vtCameraPara[nCameraNo].nSameTypeNo;
	switch (m_vtCameraPara[nCameraNo].eCameraType)
	{
	case E_DH_CAMERA:
		return m_vpImageCapture[nSameTypeNo];
		break;
	case E_KINECT_CAMERA:
		return m_vpKinectControl[nSameTypeNo];
		break;
	case E_MECHEYE_CAMERA:
#if ENABLE_MECH_EYE
		return m_vpMechMindDevice[nSameTypeNo];
#else
		MESSAGE_BOX_UNIT("MechEye 已经被裁剪，无法调用！"); 
		return nullptr;
#endif
		break;
	default:
		break;
	}
	return nullptr;
}

T_CAMREA_PARAM CContralUnit::GetCameraParam(int nCameraNo)
{
	return m_vtCameraPara[nCameraNo];
}

CDHGigeImageCapture * CContralUnit::GetDHCameraCtrl(int nSameTypeNo /*= 0*/)
{
	if (m_vpImageCapture.size() <= nSameTypeNo)
	{
		MESSAGE_BOX_UNIT(GetStr("输入大恒相机编号%d超出最大大恒相机编号%d", nSameTypeNo, m_vpImageCapture.size() - 1));
		return NULL;
	}
	return m_vpImageCapture[nSameTypeNo];
}

CKinectControl * CContralUnit::GetKinectCameraCtrl(int nSameTypeNo /*= 0*/)
{
	if (m_vpKinectControl.size() <= nSameTypeNo)
	{
		MESSAGE_BOX_UNIT(GetStr("输入全景相机编号%d超出最大全景相机编号%d", nSameTypeNo, m_vpKinectControl.size() - 1));
		return NULL;
	}
	return m_vpKinectControl[nSameTypeNo];
}

#if ENABLE_MECH_EYE
CMechEyeCtrl* CContralUnit::GetMechEyeCameraCtrl(int nSameTypeNo)
{
	if (m_vpMechMindDevice.size() <= nSameTypeNo)
	{
		MESSAGE_BOX_UNIT(GetStr("输入全景相机编号%d超出最大全景相机编号%d", nSameTypeNo, m_vpMechMindDevice.size() - 1));
		return NULL;
	}
	return m_vpMechMindDevice[nSameTypeNo];
}
#endif

BOOL CContralUnit::CaptureImage(IplImage * Image, int nSameTypeNo /*= 0*/)
{
	if (Image == NULL)
	{
		return FALSE;
	}
	return GetDHCameraCtrl(nSameTypeNo)->CaptureImage(Image, 3);
	//for (int i = 3; i > 0; i--)//三次采图，以防万一
	//{
	//	cvCopyImage(GetDHCameraCtrl(nSameTypeNo)->CaptureImage(false), Image);
	//	if (GetDHCameraCtrl(nSameTypeNo)->GetCaptureImageState())
	//	{
	//		break;
	//	}
	//	else if (i == 1)
	//	{
	//		return FALSE;
	//	}
	//	Sleep(100);
	//}
	//return TRUE;
}

BOOL CContralUnit::GetRobotConnectState()
{
	int nState;
	return GetRobotCtrl()->GetXiRobotCtrl()->GetServoPower(nState, 500);
}

BOOL CContralUnit::GetRobotTool(int nToolNo, T_ROBOT_COORS &tTool)
{
	double adRobotToolData[6] = {0};
	BOOL bRtn = GetRobotCtrl()->GetToolData(nToolNo, adRobotToolData);
	tTool = T_ROBOT_COORS(adRobotToolData[0], adRobotToolData[1], adRobotToolData[2], adRobotToolData[3], adRobotToolData[4], adRobotToolData[5],0,0,0);
	return bRtn;
}

T_ROBOT_COORS CContralUnit::GetRobotPos(int nToolNo /*= -1*/)
{
	if (nToolNo == -1)
	{
		return GetRobotCtrl()->GetCurrentPos();
	}
	else if (nToolNo >= 0 && nToolNo <= 10)
	{
		T_ROBOT_COORS tTool, tCurrentPos;
		if (FALSE == GetRobotTool(nToolNo, tTool))
		{
			return T_ROBOT_COORS();
		}
		GetRobotCtrl()->RobotKinematics(GetRobotPulse(), tTool, tCurrentPos);
		return tCurrentPos;
	}
	else
	{
		return T_ROBOT_COORS();
	}
}

T_ANGLE_PULSE CContralUnit::GetRobotPulse()
{
	return GetRobotCtrl()->GetCurrentPulse();
}

T_ROBOT_COORS CContralUnit::GetRobotTool(int nToolNo)
{
	double adRobotToolData[6];
	GetRobotCtrl()->GetToolData(nToolNo, adRobotToolData);
	return T_ROBOT_COORS(adRobotToolData[0], adRobotToolData[1], adRobotToolData[2], adRobotToolData[3], adRobotToolData[4], adRobotToolData[5], 0, 0, 0);
}

BOOL CContralUnit::RobotCheckRunning()
{
	if (ROBOT_BRAND_ESTUN == GetRobotCtrl()->m_eRobotBrand)
	{
		return GetRobotCtrl()->WorldIsRunning();
	}
	else
	{
		return CheckInIO("Running");
	}
	//WORD wState = OFF;
	//int nRtn = GetRobotRunningSignal(wState);
	//if (nRtn != 0)
	//{
	//	WRITE_LOG_UNIT(GetStr("获取IO信号失败，错误代码：%d", nRtn));
	//}
	//if (wState == ON)
	//{
	//	return TRUE;
	//}
	//return FALSE;
}

BOOL CContralUnit::RobotCheckDone(int nDelayTime /*= 1000*/)
{
	if (g_bLocalDebugMark)
	{
		return TRUE;
	}
	long long nStartTime = XI_clock();
	while (RobotCheckRunning() == FALSE)
	{
		if ((XI_clock() - nStartTime) > nDelayTime)
		{
			break;
		}
		Sleep(10);
	}
	while (1)
	{
		if (RobotCheckRunning() == FALSE)
		{
			Sleep(500);
			if (RobotCheckRunning() == FALSE)
			{
				break;
			}
		}
		Sleep(150);
	}
	return TRUE;
}

BOOL CContralUnit::RobotCheckDone(T_ROBOT_COORS tAimCoor, T_ROBOT_COORS tCompareLimit /* = T_ROBOT_COORS(1, 1, 1, 1, 1, 1, 1, 1, 1)*/, int nTool /*= -1*/)
{
	CHECK_BOOL_RTN_UNIT(RobotCheckDone(), GetStr("运动至:%s 失败", GetStr(tAimCoor)));
	return GetRobotCtrl()->CompareCoord(tAimCoor, GetRobotPos(nTool), tCompareLimit);
}

BOOL CContralUnit::RobotCheckDone(T_ANGLE_PULSE tAimPulse, T_ANGLE_PULSE tCompareLimit /*= T_ANGLE_PULSE(10, 10, 10, 10, 10, 10, 10, 10, 10)*/)
{
	CHECK_BOOL_RTN_UNIT(RobotCheckDone(), GetStr("运动至:%s 失败", GetStr(tAimPulse)));
	return GetRobotCtrl()->ComparePulse(tAimPulse, GetRobotPulse(), tCompareLimit);
}


