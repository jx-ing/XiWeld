// RobotDriverAdaptor.cpp: implementation of the CRobotDriverAdaptor class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include ".\Project\XiGrooveRobot.h"
#include ".\Apps\PLib\YaskawaRobot\RobotDriverAdaptor.h"

#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#define new DEBUG_NEW
#endif

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CRobotDriverAdaptor::CRobotDriverAdaptor(CString strUnitName, CLog* cLog, std::vector<CUnitDriver*>* ppUnitDriver)
{
	m_pvpMotorDriver = ppUnitDriver;
	InitRobotDriver(strUnitName, cLog);
}

CRobotDriverAdaptor::~CRobotDriverAdaptor()
{
	Close();
	m_cLog = NULL;
	m_pvpMotorDriver = NULL;
}

BOOL CRobotDriverAdaptor::InitRobotDriver(CString strUnitName, CLog* cLog)
{
	m_cLog = NULL;
	int nSocketPort;
	CString strRobotNo;
	strRobotNo.Format("Robot%d", m_nRobotNo);
	int nMaxNo = 0;
	T_ROBOT_COORS tZeroTool;
	tZeroTool.dX = 0;
	tZeroTool.dY = 0;
	tZeroTool.dZ = 0;
	tZeroTool.dRX = 0;
	tZeroTool.dRY = 0;
	tZeroTool.dRZ = 0;
	m_tFirstTool = tZeroTool;

	m_cLog = cLog;
	COPini cIni;
	cIni.SetFileName(DATA_PATH + strUnitName + ROBOT_PARA_INI);
	cIni.SetSectionName("BaseParam");
	cIni.ReadString("RobotName", m_strRobotName);
	cIni.ReadString("CustomName", m_strCustomName);
	m_strCustomName = Utf8ToGBK(m_strCustomName);
	cIni.ReadString("RobotType", &m_nRobotType);
	cIni.ReadString("RobotBrand", (INT*)&m_eRobotBrand);

	CString strManipulatorType;
	cIni.ReadString("ManipulatorType", strManipulatorType);
	if (!GetManipulatorType(strManipulatorType, m_eManipulatorType))
	{
	}

	cIni.ReadString("SocketPort", &nSocketPort);

	cIni.SetSectionName("Tool");
	cIni.ReadString("PolisherTool_d", "", m_tTools.tPolisherTool, T_ROBOT_COORS(1, 1, 1, 1, 1, 1, -1, -1, -1));
	cIni.ReadString("MagnetTool_d", "", m_tTools.tMagnetTool, T_ROBOT_COORS(1, 1, 1, 1, 1, 1, -1, -1, -1));
	cIni.ReadString("GunTool_d", "", m_tTools.tGunTool, T_ROBOT_COORS(1, 1, 1, 1, 1, 1, -1, -1, -1));
	cIni.ReadString("CameraTool_d", "", m_tTools.tCameraTool, T_ROBOT_COORS(1, 1, 1, 1, 1, 1, -1, -1, -1));
	m_tFirstTool = m_tTools.tGunTool;

	cIni.SetSectionName("HomePulse");
	cIni.ReadString("", "Pulse", m_tHomePulse);

	CString sSafePulse = "SafePulse0";
	int nSafePulseNo = 0;
	cIni.SetSectionName(sSafePulse);
	while (cIni.CheckExists("SPulse"))
	{
		T_ANGLE_PULSE tSafePulse;
		bool bEnable = false;
		cIni.ReadString("Enable", &bEnable);
		if (true == bEnable)
		{
			cIni.ReadString("", "Pulse", tSafePulse);
			m_vtSafePulse.push_back(tSafePulse);
		}
		nSafePulseNo++;
		sSafePulse.Format("SafePulse%d", nSafePulseNo);
		cIni.SetSectionName(sSafePulse);
	}

	cIni.SetSectionName("ExternalAxle");
	cIni.ReadString("ExternalAxleType", &m_nExternalAxleType);
	if (((m_nExternalAxleType >> 2) & 0x01) == 1)
	{
		m_tExternalAxle[2].bEnable = true;
		cIni.ReadString("BZPulse", &m_tExternalAxle[2].dPulse);
		cIni.ReadString("BZMaxPulseNum", &m_tExternalAxle[2].lMaxPulseNum);
		cIni.ReadString("BZMinPulseNum", &m_tExternalAxle[2].lMinPulseNum);
	}
	else
	{
		m_tExternalAxle[2].bEnable = false;
		m_tExternalAxle[2].dPulse = 1;
		m_tExternalAxle[2].lMaxPulseNum = 0;
		m_tExternalAxle[2].lMinPulseNum = 0;
	}
	if (((m_nExternalAxleType >> 1) & 0x01) == 1)
	{
		m_tExternalAxle[1].bEnable = true;
		cIni.ReadString("BYPulse", &m_tExternalAxle[1].dPulse);
		cIni.ReadString("BYMaxPulseNum", &m_tExternalAxle[1].lMaxPulseNum);
		cIni.ReadString("BYMinPulseNum", &m_tExternalAxle[1].lMinPulseNum);
	}
	else
	{
		m_tExternalAxle[1].bEnable = false;
		m_tExternalAxle[1].dPulse = 1;
		m_tExternalAxle[1].lMaxPulseNum = 0;
		m_tExternalAxle[1].lMinPulseNum = 0;
	}
	if ((m_nExternalAxleType & 0x01) == 1)
	{
		m_tExternalAxle[0].bEnable = true;
		cIni.ReadString("BXPulse", &m_tExternalAxle[0].dPulse);
		cIni.ReadString("BXMaxPulseNum", &m_tExternalAxle[0].lMaxPulseNum);
		cIni.ReadString("BXMinPulseNum", &m_tExternalAxle[0].lMinPulseNum);
	}
	else
	{
		m_tExternalAxle[0].bEnable = false;
		m_tExternalAxle[0].dPulse = 1;
		m_tExternalAxle[0].lMaxPulseNum = 0;
		m_tExternalAxle[0].lMinPulseNum = 0;
	}

	if (54 == m_nExternalAxleType)
	{
		cIni.SetSectionName("GrooveWeld");
		cIni.ReadString("FlatGrooveScan_d", "", m_tFlatGrooveScanPosture, T_ROBOT_COORS(1, 1, 1, 1, 1, 1, -1, -1, -1));
		cIni.ReadString("FlatGrooveWeld_d", "", m_tFlatGrooveWeldPosture, T_ROBOT_COORS(1, 1, 1, 1, 1, 1, -1, -1, -1));
		cIni.ReadString("StandDownGrooveScan_d", "", m_tStandDownGrooveScanPosture, T_ROBOT_COORS(1, 1, 1, 1, 1, 1, -1, -1, -1));
		cIni.ReadString("StandDownGrooveWeld_d", "", m_tStandDownGrooveWeldPosture, T_ROBOT_COORS(1, 1, 1, 1, 1, 1, -1, -1, -1));
		cIni.ReadString("StandUpGrooveScan_d", "", m_tStandUpGrooveScanPosture, T_ROBOT_COORS(1, 1, 1, 1, 1, 1, -1, -1, -1));
		cIni.ReadString("StandUpGrooveWeld_d", "", m_tStandUpGrooveWeldPosture, T_ROBOT_COORS(1, 1, 1, 1, 1, 1, -1, -1, -1));
	}

	cIni.SetFileName(DATA_PATH + m_strRobotName + SYETEM_PARAM_FILE);
	cIni.SetSectionName("EquipmentParam");
	cIni.ReadString("WeldNorAngleInHome", &m_dWeldNorAngleInHome);

	//算法相关
	CString strSectionName;
	LoadRobotPara(strUnitName, m_tKinematics, m_tAxisUnit, m_tAxisLimitAngle);
	m_cXiRobotAbsCoorsTrans = new GROUP_ROBOT_ABS_COORS_TRANS::XiRobotAbsCoorsTrans(m_tKinematics, m_tAxisUnit, m_tAxisLimitAngle);
	RobotKinematics(m_tHomePulse, m_tTools.tGunTool, m_tRobotHomeCoors);
	//初始化机器人
	m_pYaskawaRobotCtrl = NULL;
	m_pCrpRobotCtrl = NULL;
	if (ROBOT_BRAND_YASKAWA == m_eRobotBrand)
	{
		m_pYaskawaRobotCtrl = new CXiRobotCtrl(GetManipulatorType(m_eManipulatorType));
		if (!GetLocalDebugMark())
		{
			Open(nSocketPort);
		}
	}
	LoadSpeedAndSafeHeight();
	return TRUE;
}
void CRobotDriverAdaptor::SetKinematicsParam(T_KINEMATICS tKinematics, T_AXISUNIT tAxisUnit, T_AXISLIMITANGLE tAxisLimitAngle)
{
	m_tKinematics = tKinematics;
	m_tAxisUnit = tAxisUnit;
	m_tAxisLimitAngle = tAxisLimitAngle;
	m_cXiRobotAbsCoorsTrans = new GROUP_ROBOT_ABS_COORS_TRANS::XiRobotAbsCoorsTrans(tKinematics, tAxisUnit, tAxisLimitAngle);
	RobotKinematics(m_tHomePulse, m_tTools.tGunTool, m_tRobotHomeCoors);
}

void CRobotDriverAdaptor::LoadSpeedAndSafeHeight()
{
	COPini opini;
	SYSTEM_PARA_INI;
	opini.SetFileName(DATA_PATH + m_strRobotName + SYETEM_PARAM_FILE);
	opini.SetSectionName("BackHome");
	opini.ReadString("BackHomeSpeed", &m_tBackHomeSpeed.dSpeed);
	opini.ReadString("BackHomeACC", &m_tBackHomeSpeed.dACC);
	opini.ReadString("BackHomeDEC", &m_tBackHomeSpeed.dDEC);
	opini.SetSectionName("Teach");
	opini.ReadString("TeachSpeed", &m_tTeachSpeed.dSpeed);
	opini.ReadString("TeachACC ", &m_tTeachSpeed.dACC);
	opini.ReadString("TeachDEC", &m_tTeachSpeed.dDEC);
	opini.SetSectionName("ExAxlePulseSpeed");
	opini.ReadString("Speed", &m_tExAxlePulseSpeed.dSpeed);
	opini.ReadString("Acc", &m_tExAxlePulseSpeed.dACC);
	opini.ReadString("Dec", &m_tExAxlePulseSpeed.dDEC);
	opini.SetSectionName("PulseHighSpeed");
	opini.ReadString("Speed", &m_tPulseHighSpeed.dSpeed);
	opini.ReadString("Acc", &m_tPulseHighSpeed.dACC);
	opini.ReadString("Dec", &m_tPulseHighSpeed.dDEC);
	opini.SetSectionName("PulseLowSpeed");
	opini.ReadString("Speed", &m_tPulseLowSpeed.dSpeed);
	opini.ReadString("Acc", &m_tPulseLowSpeed.dACC);
	opini.ReadString("Dec", &m_tPulseLowSpeed.dDEC);
	opini.SetSectionName("CoordHighSpeed");
	opini.ReadString("Speed", &m_tCoordHighSpeed.dSpeed);
	opini.ReadString("Acc", &m_tCoordHighSpeed.dACC);
	opini.ReadString("Dec", &m_tCoordHighSpeed.dDEC);
	opini.SetSectionName("CoordLowSpeed");
	opini.ReadString("Speed", &m_tCoordLowSpeed.dSpeed);
	opini.ReadString("Acc", &m_tCoordLowSpeed.dACC);
	opini.ReadString("Dec", &m_tCoordLowSpeed.dDEC);
}


E_ROBOT_MODEL CRobotDriverAdaptor::GetManipulatorType(E_MANIPULATOR_TYPE eManipulatorType)
{
	switch (eManipulatorType)
	{
	case E_ROBOT_GP12:
	{
		return E_GP12;
	}
	case E_ROBOT_MH24:
	{
		return E_MH24;
	}
	case E_ROBOT_GP25:
	{
		return E_GP25;
	}
	case E_ROBOT_GP180:
	{
		return E_GP180;
	}
	case E_ROBOT_GP225:
	{
		return E_GP225;
	}
	case E_ROBOT_AR2010:
	{
		return E_AR2010;
	}
	default:
		return E_GP180;
		break;
	}
}
void CRobotDriverAdaptor::Open(int nSocketPort)
{
	// 初始化机械臂端口
	COPini cOpini;
	GetModulePath(XI_ROBOT_CTRL_INI);
	cOpini.SetFileName(gcModuleWholePath);
	cOpini.SetSectionName("XiRobotCtrl");
	cOpini.WriteString("SocketPort", nSocketPort);
	m_pYaskawaRobotCtrl->Init();

	Sleep(200);//500改200

	// 伺服使能上电
	int nIfUseRobotCtrl = 1;
	if (nIfUseRobotCtrl)
	{
		//设置背面，正肘，仰姿态
		unsigned int nRobotPosture;
		nRobotPosture = 0;
		BYTE ucMASK1 = 0;//0000 0001
		nRobotPosture |= ucMASK1; //打开第一位背面1，关闭第一位正面0
		BYTE ucMASK2 = 2;//0000 0010
		nRobotPosture &= ~ucMASK2; //打开第二位负肘1，关闭第二位正肘0
		BYTE ucMASK3 = 4;//0000 0100
		nRobotPosture |= ucMASK3; //打开第三位仰1，关闭第三位俯0
		m_pYaskawaRobotCtrl->ConfigRobotPosture(nRobotPosture);
		Sleep(200);//4000改200
	}
}

void CRobotDriverAdaptor::Close()
{
	m_pYaskawaRobotCtrl->Close();
}


void ReloadBP(double dBPVar[3], int nExternalAxleType)
{
	if (g_bLocalDebugMark) return;
	std::vector<double> vdExternalAxleVal;
	vdExternalAxleVal.clear();

	if (((nExternalAxleType) & 0x01) == 1 && ((nExternalAxleType >> 3) & 0x01) == 1) {
		vdExternalAxleVal.push_back(dBPVar[0]);
	}
	if (((nExternalAxleType >> 1) & 0x01) == 1 && ((nExternalAxleType >> 4) & 0x01) == 1) {
		vdExternalAxleVal.push_back(dBPVar[1]);
		}
	if (((nExternalAxleType >> 2) & 0x01) == 1 && ((nExternalAxleType >> 5) & 0x01) == 1) {
		vdExternalAxleVal.push_back(dBPVar[2]);
	}

	while (vdExternalAxleVal.size() < 3) {
		vdExternalAxleVal.push_back(0);
	}

	dBPVar[0] = vdExternalAxleVal[0];
	dBPVar[1] = vdExternalAxleVal[1];
	dBPVar[2] = vdExternalAxleVal[2];
}

void ReloadBP(long lBPVar[3], int nExternalAxleType)
{
	if (g_bLocalDebugMark) return;
	std::vector<long> vlExternalAxleVal;
	vlExternalAxleVal.clear();

	if (((nExternalAxleType) & 0x01) == 1 && ((nExternalAxleType >> 3) & 0x01) == 1) {
		vlExternalAxleVal.push_back(lBPVar[0]);
	}
	if (((nExternalAxleType >> 1) & 0x01) == 1 && ((nExternalAxleType >> 4) & 0x01) == 1) {
		vlExternalAxleVal.push_back(lBPVar[1]);
		}
	if (((nExternalAxleType >> 2) & 0x01) == 1 && ((nExternalAxleType >> 5) & 0x01) == 1) {
		vlExternalAxleVal.push_back(lBPVar[2]);
	}

	while (vlExternalAxleVal.size() < 3) {
		vlExternalAxleVal.push_back(0);
	}

	lBPVar[0] = vlExternalAxleVal[0];
	lBPVar[1] = vlExternalAxleVal[1];
	lBPVar[2] = vlExternalAxleVal[2];
}

int ReadBPAxisNo(int nAxisNo, int nExternalAxleType)
{
	if (g_bLocalDebugMark) return -1;
	int renAxisNo = -1;
	std::vector<int> vlExternalAxleVal;
	vlExternalAxleVal.clear();
	vlExternalAxleVal.resize(3);
	int nAxisNum = 0;
	if (((nExternalAxleType) & 0x01) == 1 && ((nExternalAxleType >> 3) & 0x01) == 1) {
		vlExternalAxleVal[0] = 0;
		nAxisNum++;
	}
	else
	{
		vlExternalAxleVal[0] = -1;
	}
	if (((nExternalAxleType >> 1) & 0x01) == 1 && ((nExternalAxleType >> 4) & 0x01) == 1) {
		vlExternalAxleVal[1] = 0;
		if (1 == nAxisNum){
			vlExternalAxleVal[1] = 1;
		}			
		nAxisNum++;
	}
	else
	{
		vlExternalAxleVal[1] = -1;
	}
	if (((nExternalAxleType >> 2) & 0x01) == 1 && ((nExternalAxleType >> 5) & 0x01) == 1) {
		vlExternalAxleVal[2] = 0;
		if (1 == nAxisNum){
			vlExternalAxleVal[2] = 1;
		}
		else if (2 == nAxisNum){
			vlExternalAxleVal[2] = 2;
		}
	}
	else
	{
		vlExternalAxleVal[2] = -1;
	}

	switch(nAxisNo)
	{
	case ROBOT_AXIS_BPX:
		renAxisNo = vlExternalAxleVal[0];
		break;
	case ROBOT_AXIS_BPY:
		renAxisNo = vlExternalAxleVal[1];
		break;
	case ROBOT_AXIS_BPZ:
		renAxisNo = vlExternalAxleVal[2];
		break;
	}
	return renAxisNo;
}

T_ROBOT_MOVE_INFO CRobotDriverAdaptor::PVarToRobotMoveInfo(int nVarNo, T_ANGLE_PULSE tPulse, T_ROBOT_MOVE_SPEED tSpeed, int nMoveType /*= MOVJ*/, UINT unToolNum /*= 1*/, UINT unUserNo /*= 0*/, UINT unPosture /*= 4*/)
{
	T_ROBOT_MOVE_INFO tRtnVal;
	tRtnVal.tCoord.var_type = MP_VAR_P;
	tRtnVal.tCoord.var_no = nVarNo;
	tRtnVal.tCoord.val.p.dtype = MP_PULSE_DTYPE;
	tRtnVal.tCoord.val.p.uf_no = unUserNo;
	tRtnVal.tCoord.val.p.tool_no = unToolNum;
	tRtnVal.tCoord.val.p.fig_ctrl = unPosture;
	tRtnVal.tCoord.val.p.data[0] = tPulse.nSPulse;
	tRtnVal.tCoord.val.p.data[1] = tPulse.nLPulse;
	tRtnVal.tCoord.val.p.data[2] = tPulse.nUPulse;
	tRtnVal.tCoord.val.p.data[3] = tPulse.nRPulse;
	tRtnVal.tCoord.val.p.data[4] = tPulse.nBPulse;
	tRtnVal.tCoord.val.p.data[5] = tPulse.nTPulse;
	tRtnVal.tCoord.val.p.data[6] = 0;
	tRtnVal.tCoord.val.p.data[7] = 0;
	tRtnVal.nMoveType = nMoveType;
	tRtnVal.tSpeed = tSpeed;
	tRtnVal.adBasePosVar[0] = tPulse.lBXPulse;
	tRtnVal.adBasePosVar[1] = tPulse.lBYPulse;
	tRtnVal.adBasePosVar[2] = tPulse.lBZPulse;
	return tRtnVal;
}

T_ROBOT_MOVE_INFO CRobotDriverAdaptor::PVarToRobotMoveInfo(int nVarNo, T_ROBOT_COORS tCoord, T_ROBOT_MOVE_SPEED tSpeed, int nMoveType /*= MOVJ*/, UINT unToolNum /*= 1*/, UINT unUserNo /*= 0*/, UINT unPosture /*= 4*/)
{
	T_ROBOT_MOVE_INFO tRtnVal;
	tRtnVal.tCoord.var_type = MP_VAR_P;
	tRtnVal.tCoord.var_no = nVarNo;
	tRtnVal.tCoord.val.p.dtype = MP_ROBO_DTYPE;
	tRtnVal.tCoord.val.p.uf_no = unUserNo;
	tRtnVal.tCoord.val.p.tool_no = unToolNum;
	tRtnVal.tCoord.val.p.fig_ctrl = unPosture;
	tRtnVal.tCoord.val.p.data[0] = long(tCoord.dX * 1000);
	tRtnVal.tCoord.val.p.data[1] = long(tCoord.dY * 1000);
	tRtnVal.tCoord.val.p.data[2] = long(tCoord.dZ * 1000);
	tRtnVal.tCoord.val.p.data[3] = long(tCoord.dRX * 10000);
	tRtnVal.tCoord.val.p.data[4] = long(tCoord.dRY * 10000);
	tRtnVal.tCoord.val.p.data[5] = long(tCoord.dRZ * 10000);
	tRtnVal.tCoord.val.p.data[6] = 0;
	tRtnVal.tCoord.val.p.data[7] = 0;
	tRtnVal.nMoveType = nMoveType;
	tRtnVal.tSpeed = tSpeed;
	tRtnVal.adBasePosVar[0] = tCoord.dBX;
	tRtnVal.adBasePosVar[1] = tCoord.dBY;
	tRtnVal.adBasePosVar[2] = tCoord.dBZ;
	return tRtnVal;
}

void CRobotDriverAdaptor::ThroughTranPointInfo(T_ANGLE_PULSE& tStartPulse, T_ANGLE_PULSE& tEndPulse, vector<T_ROBOT_MOVE_INFO>& tAllPulse)
{
	tAllPulse.clear();
	int nRobotNo = 0;
	vector<T_ANGLE_PULSE>temp;
	T_ROBOT_MOVE_INFO tRobotInfo;
	for (int i = 0; i < m_vtSafePulse.size(); i++)
	{
		if (((m_vtSafePulse[i].nSPulse < tStartPulse.nSPulse) && (m_vtSafePulse[i].nSPulse > tEndPulse.nSPulse))
			|| ((m_vtSafePulse[i].nSPulse > tStartPulse.nSPulse) && (m_vtSafePulse[i].nSPulse < tEndPulse.nSPulse)))
		{
			temp.push_back(m_vtSafePulse[i]);
		}
	}
	bool bMinToMax = tStartPulse.nSPulse < tEndPulse.nSPulse;
	
	sort(temp.begin(), temp.end(), [bMinToMax](const T_ANGLE_PULSE& t1, const T_ANGLE_PULSE& t2) {
		return bMinToMax ? t1.nSPulse < t2.nSPulse : t1.nSPulse > t2.nSPulse;});

	for (int i = 0; i < temp.size(); i++)
	{
		tRobotInfo = PVarToRobotMoveInfo(i, temp[i], m_tBackHomeSpeed, MOVJ);
		tAllPulse.push_back(tRobotInfo);
	}
}

bool CRobotDriverAdaptor::TransPulse(T_ROBOT_MOVE_INFO& tPoint, T_ANGLE_PULSE& tPulse)
{
	T_ROBOT_COORS tTranCoord;
	bool flag;
	int nRobotNo = 0;
	if (tPoint.tCoord.val.p.dtype == MP_ROBO_DTYPE)
	{
		tTranCoord.dX = tPoint.tCoord.val.p.data[0] / 1000.0;
		tTranCoord.dY = tPoint.tCoord.val.p.data[1] / 1000.0;
		tTranCoord.dZ = tPoint.tCoord.val.p.data[2] / 1000.0;
		tTranCoord.dRX = tPoint.tCoord.val.p.data[3] / 10000.0;
		tTranCoord.dRY = tPoint.tCoord.val.p.data[4] / 10000.0;
		tTranCoord.dRZ = tPoint.tCoord.val.p.data[5] / 10000.0;
		flag = RobotInverseKinematics(tTranCoord, m_tHomePulse, m_tTools.tGunTool, tPulse);
		if (flag == 0)
		{
			XUI::MesBox::PopError("转换失败");
			return false;
		}
	}
	else
	{
		tPulse.nSPulse = tPoint.tCoord.val.p.data[0];
		tPulse.nLPulse = tPoint.tCoord.val.p.data[1];
		tPulse.nUPulse = tPoint.tCoord.val.p.data[2];
		tPulse.nRPulse = tPoint.tCoord.val.p.data[3];
		tPulse.nBPulse = tPoint.tCoord.val.p.data[4];
		tPulse.nTPulse = tPoint.tCoord.val.p.data[5];
	}
	return true;
}

bool CRobotDriverAdaptor::TheAnswer(vector<T_ROBOT_MOVE_INFO>& vtRobotMoveInfo)
{
	vector<T_ROBOT_MOVE_INFO> vTemp(0);
	vector<T_ANGLE_PULSE> vtPulse(0); // 与vtRobotMoveInfo对应的关节坐标 多一个当前位置的首元素
	T_ANGLE_PULSE tCurPulse;

	tCurPulse = GetCurrentPulse();
	vtPulse.push_back(tCurPulse); // 当前位置关节坐标
	for (int i = 0; i < vtRobotMoveInfo.size(); i++)
	{
		T_ANGLE_PULSE tPulse;
		CHECK_BOOL_RETURN(TransPulse(vtRobotMoveInfo[i], tPulse));
		vtPulse.push_back(tPulse);
	}

	vector<T_ROBOT_MOVE_INFO> vtInfo(vtRobotMoveInfo); 
	vtRobotMoveInfo.clear();
	if (vtPulse.size() > 1)
	{
		for (int i = 0; i < vtPulse.size() - 1; i++)
		{
			int second = i + 1;
			ThroughTranPointInfo(vtPulse[i], vtPulse[second], vTemp);
			for (int j = 0; j < vTemp.size(); j++)
			{
				//if (54 == m_nExternalAxleType && i > 0) // 国焊坡口临时
				//{
				//	break;
				//}
				vTemp[j].adBasePosVar[0] = vtInfo[i].adBasePosVar[0];
				vTemp[j].adBasePosVar[1] = vtInfo[i].adBasePosVar[1];
				vTemp[j].adBasePosVar[2] = vtInfo[i].adBasePosVar[2];
				if (0 == vtRobotMoveInfo.size() &&  // 国焊提速 保证第一个点高速
					0.1 < fabs(vtInfo[i].tSpeed.dSpeed - vTemp[j].tSpeed.dSpeed) &&
					0.1 > fabs(vtInfo[i].tSpeed.dSpeed - m_tExAxlePulseSpeed.dSpeed))
				{
					swap(vtInfo[i].tSpeed, vTemp[j].tSpeed);
				}
				vtRobotMoveInfo.push_back(vTemp[j]);
			}
			vtRobotMoveInfo.push_back(vtInfo[i]);
		}
	}
	return true;
}

bool CRobotDriverAdaptor::SetMoveValue(CRobotMove cMoveInfo)
{
	//P20~P29作为移动坐标
	//I9作为坐标数量，
	//I8和I10用于统计（不需要设置），
	//I20~I29用于设置移动方式
	//I40~I69用于设置移动速度参数
	if (cMoveInfo.GetTrackNum() > 10)
	{
		return false;
	}

	std::vector<T_ROBOT_MOVE_INFO> vtMoveInfo = cMoveInfo.GetTrack();

	//获取P变量
	int nVarNo = 0;
	MP_USR_VAR_INFO pVarInfo[62];
	for (int i = 0; i < vtMoveInfo.size(); i++)
	{
		pVarInfo[nVarNo] = vtMoveInfo[i].tCoord;
		nVarNo++;
	}

	//获取I9
	pVarInfo[nVarNo].var_type = MP_VAR_I;
	pVarInfo[nVarNo].var_no = 9;
	pVarInfo[nVarNo].val.i = short(cMoveInfo.GetTrackNum());
	nVarNo++;

	//获取I20~I29
	for (int i = 0; i < vtMoveInfo.size(); i++)
	{
		if (vtMoveInfo[i].nMoveDevice != 0)
		{
			continue;
		}
		pVarInfo[nVarNo].var_type = MP_VAR_I;
		pVarInfo[nVarNo].var_no = 20 + i;
		pVarInfo[nVarNo].val.i = vtMoveInfo[i].nMoveType;
		nVarNo++;
	}

	//获取I40~I69
	int nFirstNo = nVarNo;
	for (int i = 0; i < vtMoveInfo.size(); i++)
	{
		if (vtMoveInfo[i].nMoveDevice != 0)
		{
			continue;
		}
		pVarInfo[nVarNo].var_type = MP_VAR_I;
		pVarInfo[nVarNo].var_no = 40 + nVarNo - nFirstNo;
		pVarInfo[nVarNo].val.i = vtMoveInfo[i].tSpeed.dSpeed;
		nVarNo++;
		pVarInfo[nVarNo].var_type = MP_VAR_I;
		pVarInfo[nVarNo].var_no = 40 + nVarNo - nFirstNo;
		pVarInfo[nVarNo].val.i = vtMoveInfo[i].tSpeed.dACC;
		nVarNo++;
		pVarInfo[nVarNo].var_type = MP_VAR_I;
		pVarInfo[nVarNo].var_no = 40 + nVarNo - nFirstNo;
		pVarInfo[nVarNo].val.i = vtMoveInfo[i].tSpeed.dDEC;
		nVarNo++;
	}

	//设置变量
	Sleep(30);
	if (TRUE != SetMultiVar_H(nVarNo, pVarInfo))
	{
		return false;
	}
	Sleep(30);
	return true;
}
bool CRobotDriverAdaptor::SetMoveValue(std::vector<T_ROBOT_MOVE_INFO> vtMoveInfo, bool bIsSafeMove, bool bUsePB)
{
	if (bIsSafeMove)
	{
		TheAnswer(vtMoveInfo);
	}
	if (ROBOT_BRAND_YASKAWA != m_eRobotBrand) // 安川专用
	{
		return false;
	}
	//P20~P29作为移动坐标
	//I9作为坐标数量，
	//I8和I10用于统计（不需要设置），
	//I20~I29用于设置移动方式
	//I40~I69用于设置移动速度参数
	if (vtMoveInfo.size() > 10) {
		return false;
	}

	//获取P变量
	int nVarNo = 0;
	MP_USR_VAR_INFO(*pVarInfo) = new MP_USR_VAR_INFO[vtMoveInfo.size() * 5 + 1];
	for (int i = 0; i < vtMoveInfo.size(); i++) {
		vtMoveInfo[i].tCoord.var_no = 20 + i;
		pVarInfo[nVarNo] = vtMoveInfo[i].tCoord;
		nVarNo++;
	}

	//获取I9
	pVarInfo[nVarNo].var_type = MP_VAR_I;
	pVarInfo[nVarNo].var_no = 9;
	pVarInfo[nVarNo].val.i = short(vtMoveInfo.size());
	nVarNo++;

	//获取I20~I29
	for (int i = 0; i < vtMoveInfo.size(); i++) {
		pVarInfo[nVarNo].var_type = MP_VAR_I;
		pVarInfo[nVarNo].var_no = 20 + i;
		pVarInfo[nVarNo].val.i = vtMoveInfo[i].nMoveType;
		nVarNo++;
	}

	//获取I40~I69
	int nFirstNo = nVarNo;
	for (int i = 0; i < vtMoveInfo.size(); i++) {
		pVarInfo[nVarNo].var_type = MP_VAR_I;
		pVarInfo[nVarNo].var_no = 40 + nVarNo - nFirstNo;
		pVarInfo[nVarNo].val.i = vtMoveInfo[i].tSpeed.dSpeed;
		nVarNo++;
		pVarInfo[nVarNo].var_type = MP_VAR_I;
		pVarInfo[nVarNo].var_no = 40 + nVarNo - nFirstNo;
		pVarInfo[nVarNo].val.i = vtMoveInfo[i].tSpeed.dACC;
		nVarNo++;
		pVarInfo[nVarNo].var_type = MP_VAR_I;
		pVarInfo[nVarNo].var_no = 40 + nVarNo - nFirstNo;
		pVarInfo[nVarNo].val.i = vtMoveInfo[i].tSpeed.dDEC;
		nVarNo++;
	}

	//设置变量
	SetMultiVar_H(nVarNo, pVarInfo);
	delete pVarInfo;
	pVarInfo = NULL;

	if (bUsePB) // 发送BP变量
	{
		for (int i = 0; i < vtMoveInfo.size(); i++) 
		{
			int nBasePosVarType = MP_ROBO_DTYPE == vtMoveInfo[i].tCoord.val.p.dtype ? 0 : 1; // 0基座 1关节
			double adSendBasePosVar[3] = { 0.0 };
			RobotTransCoordToBase(vtMoveInfo[i].adBasePosVar, adSendBasePosVar);
			m_pYaskawaRobotCtrl->SetBasePosVar(20 + i, adSendBasePosVar, nBasePosVarType);
		}
	}
	return true;
}

int CRobotDriverAdaptor::GetMoveStep()
{
	if (ROBOT_BRAND_YASKAWA != m_eRobotBrand) // 安川专用
	{
		return FALSE;
	}
	return GetIntVar(10);
}

int CRobotDriverAdaptor::CleanMoveStep()
{
	if (ROBOT_BRAND_YASKAWA != m_eRobotBrand) // 安川专用
	{
		return FALSE;
	}
	SetIntVar(10, 0);
	return 0;
}

void CRobotDriverAdaptor::RobotTransCoordToBase(double *adCoord, double *adBasePosVar)
{
	memset(adBasePosVar, 0, sizeof(double) * 3);
	int nIdx = 0;
	if ((1 == (m_nExternalAxleType >> 0 & 1)) && ((1 == (m_nExternalAxleType >> 3 & 1))))
	{
		adBasePosVar[nIdx++] = adCoord[0];
	}

	if ((1 == (m_nExternalAxleType >> 1 & 1)) && ((1 == (m_nExternalAxleType >> 4 & 1))))
	{
		adBasePosVar[nIdx++] = adCoord[1];
	}

	if ((1 == (m_nExternalAxleType >> 2 & 1)) && ((1 == (m_nExternalAxleType >> 5 & 1))))
	{
		adBasePosVar[nIdx++] = adCoord[2];
	}
}

void CRobotDriverAdaptor::RobotTransBaseToCoord(double *adBasePosVar, double *adCoord)
{
	memset(adCoord, 0, sizeof(double) * 3);
	int nIdx = 0;
	if ((1 == (m_nExternalAxleType >> 0 & 1)) && ((1 == (m_nExternalAxleType >> 3 & 1))))
	{
		adCoord[0] = adBasePosVar[nIdx++];
	}

	if ((1 == (m_nExternalAxleType >> 1 & 1)) && ((1 == (m_nExternalAxleType >> 4 & 1))))
	{
		adCoord[1] = adBasePosVar[nIdx++];
	}

	if ((1 == (m_nExternalAxleType >> 2 & 1)) && ((1 == (m_nExternalAxleType >> 5 & 1))))
	{
		adCoord[2] = adBasePosVar[nIdx++];
	}
}

bool CRobotDriverAdaptor::ReadSyncCoord(T_ANGLE_PULSE& tPulse, int nSyncId)
{
	double adPos[12] = { 0 };
	long alPulse[9] = { 0 };
	long alExPulse[3] = { 0 };
	//m_pYaskawaRobotCtrl->ReadSyncCoord(adPos, alPulse, alExPulse, nSyncId);
	//if (
	long vxTime; 
	long rcvIdx; 
	long sendIdx; 
	long winTime;
	m_pYaskawaRobotCtrl->ReadSyncCoord(adPos, alPulse, alExPulse, nSyncId, vxTime, rcvIdx, sendIdx, winTime);
	//{
		tPulse.nSPulse = alPulse[0]; // 关节坐标
		tPulse.nLPulse = alPulse[1];
		tPulse.nUPulse = alPulse[2];
		tPulse.nRPulse = alPulse[3];
		tPulse.nBPulse = alPulse[4];
		tPulse.nTPulse = alPulse[5];
		tPulse.lBXPulse = alExPulse[2];
		tPulse.lBYPulse = alExPulse[0]; // jwq临时 Y方向单外部轴 只用0, 12无效
		tPulse.lBZPulse = alExPulse[2];
		return true;
	//}
	//return false;
}
bool CRobotDriverAdaptor::GetManipulatorType(CString strManipulatorType, E_MANIPULATOR_TYPE& eManipulatorType)
{
	std::map<CString, E_MANIPULATOR_TYPE> mapManipulatorType;
	mapManipulatorType["GP12"] = E_ROBOT_GP12;
	mapManipulatorType["GP25"] = E_ROBOT_GP25;
	mapManipulatorType["GP180"] = E_ROBOT_GP180;
	mapManipulatorType["GP225"] = E_ROBOT_GP225;
	mapManipulatorType["GP280L"] = E_ROBOT_GP280L;
	mapManipulatorType["MH12"] = E_ROBOT_MH12;
	mapManipulatorType["MH24"] = E_ROBOT_MH24;
	mapManipulatorType["MH50_20"] = E_ROBOT_MH50_20;
	mapManipulatorType["MH225"] = E_ROBOT_MH225;
	mapManipulatorType["MA2010"] = E_ROBOT_MA2010;
	mapManipulatorType["AR2010"] = E_ROBOT_AR2010;
	mapManipulatorType["ESTUN"] = E_ROBOT_ESTUN;
	try
	{
		eManipulatorType = mapManipulatorType.at(strManipulatorType);
	}
	catch (...)
	{
		XUI::MesBox::PopError("Error: 未知的机械臂型号！");
		eManipulatorType = E_ROBOT_UNKOWN;
		return false;
	}
	return true;
}
//（问题：老框架无此定义	）
bool CRobotDriverAdaptor::IsCuttingRobot()
{
	return (m_nRobotType & CUTTING_ROBOT) == CUTTING_ROBOT;
}
//（问题：老框架无此定义	）
bool CRobotDriverAdaptor::IsHandlingRobot()
{
	return (m_nRobotType & HANDLING_ROBOT) == HANDLING_ROBOT;
}
//（问题：老框架无此定义	）
bool CRobotDriverAdaptor::IsPolishingRobot()
{
	return (m_nRobotType & POLISHING_ROBOT) == POLISHING_ROBOT;
}

//（问题：使用宏ROBOT_PAUSE_INI（目录：./Data/_RobotName_/RobotPause.ini）该函数已完成定义 新框架无此目录结构 老框架无此定义）
bool CRobotDriverAdaptor::SavePausePara(CString sKey, int nValue)
{
	BOOL bRet = TRUE;
	COPini cIni;
	cIni.SetFileName(DATA_PATH + m_strRobotName + ROBOT_PAUSE_INI);
	cIni.SetSectionName("Pause");
	if (TRUE != cIni.WriteString(sKey, nValue))
	{
		return false;
	}
	m_cLog->Write("SavePausePara:%s,%d", sKey, nValue);
	return true;
}
//（问题：使用宏ROBOT_PAUSE_INI（目录：./Data/_RobotName_/RobotPause.ini）该函数已完成定义 新框架无此目录结构 老框架无此定义）
bool CRobotDriverAdaptor::SavePausePara(CString sKey, bool bValue)
{
	BOOL bRet = TRUE;
	COPini cIni;
	cIni.SetFileName(DATA_PATH + m_strRobotName + ROBOT_PAUSE_INI);
	cIni.SetSectionName("Pause");
	if (TRUE != cIni.WriteString(sKey, bValue))
	{
		return false;
	}
	m_cLog->Write("SavePausePara:%s,%d", sKey, (int)bValue);
	return true;
}
//（问题：使用宏ROBOT_PAUSE_INI（目录：./Data/_RobotName_/RobotPause.ini）该函数已完成定义 新框架无此目录结构 老框架无此定义）
bool CRobotDriverAdaptor::SavePausePara(CString sKey, double dValue)
{
	BOOL bRet = TRUE;
	COPini cIni;
	cIni.SetFileName(DATA_PATH + m_strRobotName + ROBOT_PAUSE_INI);
	cIni.SetSectionName("Pause");
	if (TRUE != cIni.WriteString(sKey, dValue))
	{
		return false;
	}
	m_cLog->Write("SavePausePara:%s,%lf", sKey, dValue);
	return true;
}
//（问题：使用宏ROBOT_PAUSE_INI（目录：./Data/_RobotName_/RobotPause.ini）该函数已完成定义 新框架无此目录结构 老框架无此定义）
bool CRobotDriverAdaptor::SavePausePara(CString sKey, CString sValue)
{
	BOOL bRet = TRUE;
	COPini cIni;
	cIni.SetFileName(DATA_PATH + m_strRobotName + ROBOT_PAUSE_INI);
	cIni.SetSectionName("Pause");
	if (TRUE != cIni.WriteString(sKey, sValue))
	{
		return false;
	}
	m_cLog->Write("SavePausePara:%s,%s", sKey, sValue);
	return true;
}
//（问题：使用宏ROBOT_PAUSE_INI（目录：./Data/_RobotName_/RobotPause.ini）该函数已完成定义 新框架无此目录结构 老框架无此定义）
bool CRobotDriverAdaptor::LoadPausePara(CString sKey, int& nValue)
{
	BOOL bRet = TRUE;
	COPini cIni;
	cIni.SetFileName(DATA_PATH + m_strRobotName + ROBOT_PAUSE_INI);
	cIni.SetSectionName("Pause");
	if (TRUE != cIni.ReadString(sKey, &nValue))
	{
		return false;
	}
	return true;
}
//（问题：使用宏ROBOT_PAUSE_INI（目录：./Data/_RobotName_/RobotPause.ini）该函数已完成定义 新框架无此目录结构 老框架无此定义）
bool CRobotDriverAdaptor::LoadPausePara(CString sKey, bool& bValue)
{
	BOOL bRet = TRUE;
	COPini cIni;
	cIni.SetFileName(DATA_PATH + m_strRobotName + ROBOT_PAUSE_INI);
	cIni.SetSectionName("Pause");
	if (TRUE != cIni.ReadString(sKey, &bValue))
	{
		return false;
	}
	return true;
}
//（问题：使用宏ROBOT_PAUSE_INI（目录：./Data/_RobotName_/RobotPause.ini）该函数已完成定义 新框架无此目录结构 老框架无此定义）
bool CRobotDriverAdaptor::LoadPausePara(CString sKey, double& dValue)
{
	BOOL bRet = TRUE;
	COPini cIni;
	cIni.SetFileName(DATA_PATH + m_strRobotName + ROBOT_PAUSE_INI);
	cIni.SetSectionName("Pause");
	if (TRUE != cIni.ReadString(sKey, &dValue))
	{
		return false;
	}
	return true;
}
//（问题：使用宏ROBOT_PAUSE_INI（目录：./Data/_RobotName_/RobotPause.ini）该函数已完成定义 新框架无此目录结构 老框架无此定义）
bool CRobotDriverAdaptor::LoadPausePara(CString sKey, CString& sValue)
{
	BOOL bRet = TRUE;
	COPini cIni;
	cIni.SetFileName(DATA_PATH + m_strRobotName + ROBOT_PAUSE_INI);
	cIni.SetSectionName("Pause");
	if (TRUE != cIni.ReadString(sKey, sValue))
	{
		return false;
	}
	return true;
}
//（问题：使用宏ROBOT_PAUSE_INI（目录：./Data/_RobotName_/RobotPause.ini）该函数已完成定义 新框架无此目录结构 老框架无此定义）
bool CRobotDriverAdaptor::SetAimPulse(T_ANGLE_PULSE tAimPulse)
{
	m_tAimPulse = tAimPulse;
	COPini cIni;
	cIni.SetFileName(DATA_PATH + m_strRobotName + ROBOT_PAUSE_INI);
	cIni.SetSectionName("Pause");
	if (TRUE != cIni.WriteString("AimPulse_", "", tAimPulse))
	{
		return false;
	}
	return true;
}
//（问题：使用宏ROBOT_PAUSE_INI（目录：./Data/_RobotName_/RobotPause.ini）该函数已完成定义 新框架无此目录结构 老框架无此定义）
bool CRobotDriverAdaptor::SetAimCoord(T_ROBOT_COORS tAimCoord)
{
	m_tAimCoord = tAimCoord;
	COPini cIni;
	cIni.SetFileName(DATA_PATH + m_strRobotName + ROBOT_PAUSE_INI);
	cIni.SetSectionName("Pause");
	if (TRUE != cIni.WriteString("AimCoord_", "", tAimCoord))
	{
		return false;
	}
	return true;
}
//（问题：使用宏ROBOT_PAUSE_INI（目录：./Data/_RobotName_/RobotPause.ini）该函数已完成定义 新框架无此目录结构 老框架无此定义）
bool CRobotDriverAdaptor::CheckIsAimPulse(T_ANGLE_PULSE tCompareLimit)
{
	COPini cIni;
	cIni.SetFileName(DATA_PATH + m_strRobotName + ROBOT_PAUSE_INI);
	cIni.SetSectionName("Pause");
	if (TRUE != cIni.ReadString("AimPulse_", "", m_tAimPulse))
	{
		return false;
	}
	return ComparePulse(GetCurrentPulse(), m_tAimPulse, tCompareLimit);
}
//（问题：使用宏ROBOT_PAUSE_INI（目录：./Data/_RobotName_/RobotPause.ini）该函数已完成定义 新框架无此目录结构 老框架无此定义）
bool CRobotDriverAdaptor::CheckIsAimCoord(T_ROBOT_COORS tCompareLimit)
{
	COPini cIni;
	cIni.SetFileName(DATA_PATH + m_strRobotName + ROBOT_PAUSE_INI);
	cIni.SetSectionName("Pause");
	if (TRUE != cIni.ReadString("AimCoord_", "", m_tAimCoord))
	{
		return false;
	}
	return CompareCoord(GetCurrentPos(), m_tAimCoord, tCompareLimit);
}

bool CRobotDriverAdaptor::CompareCurCoord(T_ROBOT_COORS tCoord, T_ROBOT_COORS tCompareLimit)
{
	return CompareCoord(GetCurrentPos(), tCoord, tCompareLimit);
}

bool CRobotDriverAdaptor::CompareCurPulse(T_ANGLE_PULSE tPulse, T_ANGLE_PULSE tCompareLimit)
{
	return ComparePulse(GetCurrentPulse(), tPulse, tCompareLimit);
}

bool CRobotDriverAdaptor::CompareCoord(T_ROBOT_COORS tCoord1, T_ROBOT_COORS tCoord2, T_ROBOT_COORS tCompareLimit)
{
	if ((tCompareLimit.dX >= 0 && fabs((tCoord1.dX - tCoord2.dX)) >= tCompareLimit.dX)
		|| (tCompareLimit.dY >= 0 && fabs((tCoord1.dY - tCoord2.dY)) >= tCompareLimit.dY)
		|| (tCompareLimit.dZ >= 0 && fabs((tCoord1.dZ - tCoord2.dZ)) >= tCompareLimit.dZ)
		|| (tCompareLimit.dRX >= 0 && fabs((tCoord1.dRX - tCoord2.dRX)) >= tCompareLimit.dRX)
		|| (tCompareLimit.dRY >= 0 && fabs((tCoord1.dRY - tCoord2.dRY)) >= tCompareLimit.dRY)
		|| (tCompareLimit.dRZ >= 0 && fabs((tCoord1.dRZ - tCoord2.dRZ)) >= tCompareLimit.dRZ)
		|| (m_tExternalAxle[0].bEnable && tCompareLimit.dBX >= 0 && fabs((tCoord1.dBX - tCoord2.dBX)) >= tCompareLimit.dBX)
		|| (m_tExternalAxle[1].bEnable && tCompareLimit.dBY >= 0 && fabs((tCoord1.dBY - tCoord2.dBY)) >= tCompareLimit.dBY)
		|| (m_tExternalAxle[2].bEnable && tCompareLimit.dBZ >= 0 && fabs((tCoord1.dBZ - tCoord2.dBZ)) >= tCompareLimit.dBZ))
	{
		WriteLog("%s : %s", GetStr(tCoord1), GetStr(tCoord2));
		return false;
	}
	return true;
}

bool CRobotDriverAdaptor::ComparePulse(T_ANGLE_PULSE tPulse1, T_ANGLE_PULSE tPulse2, T_ANGLE_PULSE tCompareLimit)
{
	if ((tCompareLimit.nSPulse >= 0 && abs((tPulse1.nSPulse - tPulse2.nSPulse)) >= tCompareLimit.nSPulse)
		|| (tCompareLimit.nLPulse >= 0 && abs((tPulse1.nLPulse - tPulse2.nLPulse)) >= tCompareLimit.nLPulse)
		|| (tCompareLimit.nUPulse >= 0 && abs((tPulse1.nUPulse - tPulse2.nUPulse)) >= tCompareLimit.nUPulse)
		|| (tCompareLimit.nRPulse >= 0 && abs((tPulse1.nRPulse - tPulse2.nRPulse)) >= tCompareLimit.nRPulse)
		|| (tCompareLimit.nBPulse >= 0 && abs((tPulse1.nBPulse - tPulse2.nBPulse)) >= tCompareLimit.nBPulse)
		|| (tCompareLimit.nTPulse >= 0 && abs((tPulse1.nTPulse - tPulse2.nTPulse)) >= tCompareLimit.nTPulse)
		|| (m_tExternalAxle[0].bEnable && tCompareLimit.lBXPulse >= 0 && abs((tPulse1.lBXPulse - tPulse2.lBXPulse)) >= tCompareLimit.lBXPulse)
		|| (m_tExternalAxle[1].bEnable && tCompareLimit.lBYPulse >= 0 && abs((tPulse1.lBYPulse - tPulse2.lBYPulse)) >= tCompareLimit.lBYPulse)
		|| (m_tExternalAxle[2].bEnable && tCompareLimit.lBZPulse >= 0 && abs((tPulse1.lBZPulse - tPulse2.lBZPulse)) >= tCompareLimit.lBZPulse))
	{
		WriteLog("%s : %s", GetStr(tPulse1), GetStr(tPulse2));
		return false;
	}
	return true;
}

bool CRobotDriverAdaptor::RobotInverseKinematics(T_ROBOT_COORS tRobotCoors, T_ROBOT_COORS tToolCoors, std::vector<T_ANGLE_PULSE>& vtResultPulse)
{
	//转化为关节坐标
	std::vector<T_ANGLE_THETA> vtIKResultTheta;
	m_cXiRobotAbsCoorsTrans->RobotInverseKinematics(tRobotCoors, tToolCoors, vtResultPulse, vtIKResultTheta, m_eManipulatorType);
	//if (m_eRobotBrand == ROBOT_BRAND_ESTUN || m_eRobotBrand == ROBOT_BRAND_CRP)
	//{
	//	vtResultPulse.resize(vtIKResultTheta.size());	
	//	for (int i = 0; i < vtIKResultTheta.size(); i++)
	//	{
	//		vtResultPulse[i] = ThetaToPulse(vtIKResultTheta[i]);
	//	}
	//}
	for (int i = 0; i < vtResultPulse.size(); i++)
	{
		if (((m_nExternalAxleType >> 2) & 0x01) == 1)
		{
			vtResultPulse[i].lBZPulse = (long)(tRobotCoors.dBZ / m_tExternalAxle[2].dPulse);
		}
		else
		{
			vtResultPulse[i].lBZPulse = 0;
		}
		if (((m_nExternalAxleType >> 1) & 0x01) == 1)
		{
			vtResultPulse[i].lBYPulse = (long)(tRobotCoors.dBY / m_tExternalAxle[1].dPulse);
		}
		else
		{
			vtResultPulse[i].lBYPulse = 0;
		}
		if ((m_nExternalAxleType & 0x01) == 1)
		{
			vtResultPulse[i].lBXPulse = (long)(tRobotCoors.dBX / m_tExternalAxle[0].dPulse);
		}
		else
		{
			vtResultPulse[i].lBXPulse = 0;
		}
	}
	return vtResultPulse.size() > 0;
}

bool CRobotDriverAdaptor::RobotInverseKinematics(T_ROBOT_COORS tRobotCoors, T_ANGLE_PULSE tReferencePulse, T_ROBOT_COORS tToolCoors, T_ANGLE_PULSE& tResultPulse, int nExternalAxleType /*= -1*/)
{
	//转化为关节坐标
	std::vector<T_ANGLE_PULSE> vtIKResultPulse;
	std::vector<T_ANGLE_THETA> vtIKResultTheta;
	m_cXiRobotAbsCoorsTrans->RobotInverseKinematics(tRobotCoors, tToolCoors, vtIKResultPulse, vtIKResultTheta, m_eManipulatorType);
	//if (m_eRobotBrand == ROBOT_BRAND_ESTUN || m_eRobotBrand == ROBOT_BRAND_CRP)
	//{
	//	vtIKResultPulse.resize(vtIKResultTheta.size());
	//	for (int i = 0; i < vtIKResultPulse.size(); i++)
	//	{
	//		vtIKResultPulse[i] = ThetaToPulse(vtIKResultTheta[i]);
	//	}
	//}
	//找出合适的关节坐标
	double dAnglePulseDis = 0.0;
	double dMin = 999999.0;
	int nMinNo = -1;
	for (int nResultNo = 0; nResultNo < vtIKResultPulse.size(); nResultNo++)
	{
		dAnglePulseDis = m_cXiRobotAbsCoorsTrans->GetAnglePulseDisNew(vtIKResultPulse[nResultNo], tReferencePulse);

		if (dAnglePulseDis < dMin)
		{
			dMin = dAnglePulseDis;
			nMinNo = nResultNo;
		}
	}
	if (nMinNo < 0)
	{
		WriteLog(GetStr(tToolCoors));
		return false;
	}
	tResultPulse = vtIKResultPulse[nMinNo];

	if (nExternalAxleType < 0)
	{
		nExternalAxleType = m_nExternalAxleType;
	}

	if (((nExternalAxleType >> 2) & 0x01) == 1)
	{
		tResultPulse.lBZPulse = tRobotCoors.dBZ / m_tExternalAxle[2].dPulse;
	}
	else
	{
		tResultPulse.lBZPulse = 0;
	}
	if (((nExternalAxleType >> 1) & 0x01) == 1)
	{
		tResultPulse.lBYPulse = tRobotCoors.dBY / m_tExternalAxle[1].dPulse;
	}
	else
	{
		tResultPulse.lBYPulse = 0;
	}
	if ((nExternalAxleType & 0x01) == 1)
	{
		tResultPulse.lBXPulse = tRobotCoors.dBX / m_tExternalAxle[0].dPulse;
	}
	else
	{
		tResultPulse.lBXPulse = 0;
	}

	return true;
}

void CRobotDriverAdaptor::RobotKinematics(T_ANGLE_PULSE tCurrentPulse, T_ROBOT_COORS tToolCoors, T_ROBOT_COORS& tCurrentRobotCoors, int nExternalAxleType /*= -1*/)
{
	T_ANGLE_THETA tCurrentTheta = PulseToTheta(tCurrentPulse);
	m_cXiRobotAbsCoorsTrans->RobotKinematics(tCurrentPulse, tCurrentTheta, tToolCoors, tCurrentRobotCoors, m_eManipulatorType);

	if (nExternalAxleType < 0)
	{
		nExternalAxleType = m_nExternalAxleType;
	}

	if (((nExternalAxleType >> 2) & 0x01) == 1)
	{
		tCurrentRobotCoors.dBZ = tCurrentPulse.lBZPulse * m_tExternalAxle[2].dPulse;
	}
	else
	{
		tCurrentRobotCoors.dBZ = 0;
	}
	if (((nExternalAxleType >> 1) & 0x01) == 1)
	{
		tCurrentRobotCoors.dBY = tCurrentPulse.lBYPulse * m_tExternalAxle[1].dPulse;
	}
	else
	{
		tCurrentRobotCoors.dBY = 0;
	}
	if ((nExternalAxleType & 0x01) == 1)
	{
		tCurrentRobotCoors.dBX = tCurrentPulse.lBXPulse * m_tExternalAxle[0].dPulse;
	}
	else
	{
		tCurrentRobotCoors.dBX = 0;
	}
}

bool CRobotDriverAdaptor::MoveToolByWeldGun(T_ROBOT_COORS tWeldGunCoors, T_ROBOT_COORS tWeldGunTools, T_ROBOT_COORS tMagneticCoors, T_ROBOT_COORS tMagneticTools, T_ROBOT_COORS& tGunCoorsMoveMagnetic)
{
	tMagneticCoors.dX = tWeldGunCoors.dX;
	tMagneticCoors.dY = tWeldGunCoors.dY;
	tMagneticCoors.dZ = tWeldGunCoors.dZ;
	tMagneticCoors.dBX = tWeldGunCoors.dBX;
	tMagneticCoors.dBY = tWeldGunCoors.dBY;
	tMagneticCoors.dBZ = tWeldGunCoors.dBZ;

	tMagneticCoors.dRX = fmod(tMagneticCoors.dRX, 360.0);
	tMagneticCoors.dRY = fmod(tMagneticCoors.dRY, 360.0);
	tMagneticCoors.dRZ = fmod(tMagneticCoors.dRZ, 360.0);
	tMagneticCoors.dRX = tMagneticCoors.dRX > 180.0 ? tMagneticCoors.dRX - 360.0 : tMagneticCoors.dRX;
	tMagneticCoors.dRY = tMagneticCoors.dRY > 180.0 ? tMagneticCoors.dRY - 360.0 : tMagneticCoors.dRY;
	tMagneticCoors.dRZ = tMagneticCoors.dRZ > 180.0 ? tMagneticCoors.dRZ - 360.0 : tMagneticCoors.dRZ;
	tMagneticCoors.dRX = tMagneticCoors.dRX < -180.0 ? tMagneticCoors.dRX + 360.0 : tMagneticCoors.dRX;
	tMagneticCoors.dRY = tMagneticCoors.dRY < -180.0 ? tMagneticCoors.dRY + 360.0 : tMagneticCoors.dRY;
	tMagneticCoors.dRZ = tMagneticCoors.dRZ < -180.0 ? tMagneticCoors.dRZ + 360.0 : tMagneticCoors.dRZ;

	std::vector<T_ANGLE_PULSE> vtIKResultPulse;
	RobotInverseKinematics(tMagneticCoors, tMagneticTools, vtIKResultPulse);

	if (vtIKResultPulse.size() > 0) {
		RobotKinematics(vtIKResultPulse[0], tWeldGunTools, tGunCoorsMoveMagnetic);
	}
	else {
		tGunCoorsMoveMagnetic.dX = 0.0;
		tGunCoorsMoveMagnetic.dY = 0.0;
		tGunCoorsMoveMagnetic.dZ = 0.0;
		tGunCoorsMoveMagnetic.dRX = 0.0;
		tGunCoorsMoveMagnetic.dRY = 0.0;
		tGunCoorsMoveMagnetic.dRZ = 0.0;
		tGunCoorsMoveMagnetic.dBX = 0.0;
		tGunCoorsMoveMagnetic.dBY = 0.0;
		tGunCoorsMoveMagnetic.dBZ = 0.0;
		WriteLog("Trans Error!");
		return false;
	}
	return true;
}

T_ANGLE_PULSE CRobotDriverAdaptor::ThetaToPulse(T_ANGLE_THETA tTheta)
{
	T_ANGLE_PULSE tPulse;
	tPulse.nSPulse = (long)((double)tTheta.dThetaS / m_tAxisUnit.dSPulse);
	tPulse.nLPulse = (long)((double)tTheta.dThetaL / m_tAxisUnit.dLPulse);
	tPulse.nUPulse = (long)((double)tTheta.dThetaU / m_tAxisUnit.dUPulse);
	tPulse.nRPulse = (long)((double)tTheta.dThetaR / m_tAxisUnit.dRPulse);
	tPulse.nBPulse = (long)((double)tTheta.dThetaB / m_tAxisUnit.dBPulse);
	tPulse.nTPulse = (long)((double)tTheta.dThetaT / m_tAxisUnit.dTPulse);
	tPulse.lBXPulse = 0;
	tPulse.lBYPulse = 0;
	tPulse.lBZPulse = 0;
	return tPulse;
}

T_ANGLE_THETA CRobotDriverAdaptor::PulseToTheta(T_ANGLE_PULSE tPulse)
{
	T_ANGLE_THETA tTheta;
	tTheta.dThetaS = (double)tPulse.nSPulse * m_tAxisUnit.dSPulse;
	tTheta.dThetaL = (double)tPulse.nLPulse * m_tAxisUnit.dLPulse;
	tTheta.dThetaU = (double)tPulse.nUPulse * m_tAxisUnit.dUPulse;
	tTheta.dThetaR = (double)tPulse.nRPulse * m_tAxisUnit.dRPulse;
	tTheta.dThetaB = (double)tPulse.nBPulse * m_tAxisUnit.dBPulse;
	tTheta.dThetaT = (double)tPulse.nTPulse * m_tAxisUnit.dTPulse;
	return tTheta;
}

double CRobotDriverAdaptor::DirAngleToRz(double dDirAngle)
{
	double dRz = (dDirAngle - m_dWeldNorAngleInHome) + m_tRobotHomeCoors.dRZ;
	if (dRz > 180) {
		dRz -= 360;
	}
	if (dRz < -180) {
		dRz += 360;
	}
	return dRz;
}

double CRobotDriverAdaptor::DirAngleToRz(double dBaseRz, double dBaseDirAngle, double dChangeDir, double dDirAngle)
{
	dDirAngle = fmod(dDirAngle, 360.0);
	return (dChangeDir * (dDirAngle - dBaseDirAngle) + dBaseRz);
}

double CRobotDriverAdaptor::RzToDirAngle(double dRz)
{
	double dDirAngle = (dRz - m_tRobotHomeCoors.dRZ) + m_dWeldNorAngleInHome;
	dDirAngle = fmod(dDirAngle, 360.0);
	dDirAngle = dDirAngle < 0.0 ? dDirAngle + 360.0 : dDirAngle;
	return dDirAngle;
}

double CRobotDriverAdaptor::RzToDirAngle(double dBaseRz, double dBaseDirAngle, double dChangeDir, double dRz)
{
	dRz = fmod(dRz, 360.0);
	dRz = dRz > 180.0 ? (dRz -= 360.0) : dRz;
	dRz = dRz < -180.0 ? (dRz += 360.0) : dRz;
	return (dChangeDir * (dRz - dBaseRz) + dBaseDirAngle);
}

double CRobotDriverAdaptor::GetCurrentPos(int nAxisNo)  //查询机器人当前坐标
{
	if (TRUE == g_bLocalDebugMark) return 0.0;
	if (nAxisNo < 6) 
	{
		return (m_pYaskawaRobotCtrl->GetCurrentPos(nAxisNo));
	}
	else 
	{
		double dCurPos = 0.0;
		if (6 == nAxisNo && (1 == (m_nExternalAxleType >> 6 & 0x01)))
		{
			GetPanasonicExPos(E_EX_X, dCurPos);
		}
		else if (7 == nAxisNo && (1 == (m_nExternalAxleType >> 7 & 0x01)))
		{
			GetPanasonicExPos(E_EX_Y, dCurPos);
		}
		else if (8 == nAxisNo && (1 == (m_nExternalAxleType >> 8 & 0x01)))
		{
			GetPanasonicExPos(E_EX_Z, dCurPos);
		}
		else
		{
			dCurPos = m_pYaskawaRobotCtrl->GetCurrentBasePos(nAxisNo - 6);
		}
		return dCurPos;
	}
}

bool CRobotDriverAdaptor::GetInputIOVar(int name, int value)
{
	return false;
}

bool CRobotDriverAdaptor::SetOutIOVar(int name, int value)
{
	return false;
}

T_ROBOT_COORS CRobotDriverAdaptor::GetCurrentPos_ESTUN()
{
	T_ROBOT_COORS tRobotCurCoord(0, 0, 0, 0, 0, 0, 0, 0, 0);
	return tRobotCurCoord;
}

T_ROBOT_COORS CRobotDriverAdaptor::GetCurrentPos()
{
	T_ROBOT_COORS tRobotCurCoord;
	if (TRUE == g_bLocalDebugMark) return tRobotCurCoord;

	tRobotCurCoord.dX = GetCurrentPos(ROBOT_AXIS_X);
	tRobotCurCoord.dY = GetCurrentPos(ROBOT_AXIS_Y);
	tRobotCurCoord.dZ = GetCurrentPos(ROBOT_AXIS_Z);
	tRobotCurCoord.dRX = GetCurrentPos(ROBOT_AXIS_RX);
	tRobotCurCoord.dRY = GetCurrentPos(ROBOT_AXIS_RY);
	tRobotCurCoord.dRZ = GetCurrentPos(ROBOT_AXIS_RZ);
	tRobotCurCoord.dBX = GetCurrentPos(ROBOT_AXIS_BPX);
	tRobotCurCoord.dBY = GetCurrentPos(ROBOT_AXIS_BPY);
	tRobotCurCoord.dBZ = GetCurrentPos(ROBOT_AXIS_BPZ);

	if ((((m_nExternalAxleType >> 3) & 0x01) == 1))
	{
		tRobotCurCoord.dX -= tRobotCurCoord.dBX;
	}	
	if ((((m_nExternalAxleType >> 4) & 0x01) == 1))
	{
		tRobotCurCoord.dY -= tRobotCurCoord.dBY;
	}	
	if ((((m_nExternalAxleType >> 5) & 0x01) == 1))
	{
		tRobotCurCoord.dZ -= tRobotCurCoord.dBZ;
	}	
	return tRobotCurCoord;
}

long CRobotDriverAdaptor::GetCurrentPulse(int nAxisNo)  //查询机器人当前坐标, 获取外部轴位置逻辑不正确
{
	if (TRUE == g_bLocalDebugMark) return 0;
	if (nAxisNo < 6) 
	{
		return (m_pYaskawaRobotCtrl->GetCurrentPulse(nAxisNo));
	}
	else 
	{
		long lCurPos = 0.0;
		if (6 == nAxisNo && (1 == (m_nExternalAxleType>>6  & 0x01)))
		{
			GetPanasonicExPos(E_EX_X, lCurPos);		
		}
		else if (7 == nAxisNo && (1 == (m_nExternalAxleType >> 7 & 0x01)))
		{
			GetPanasonicExPos(E_EX_Y, lCurPos);
		}
		else if (8 == nAxisNo && (1 == (m_nExternalAxleType >> 8 & 0x01)))
		{
			GetPanasonicExPos(E_EX_Z, lCurPos);
		}
		else
		{
			int reAxisNo = ReadBPAxisNo(nAxisNo, m_nExternalAxleType);
			if (-1 != reAxisNo)
			{
				lCurPos = m_pYaskawaRobotCtrl->GetCurrentBasePulse(reAxisNo);
			}
			
			//lCurPos = m_pYaskawaRobotCtrl->GetCurrentBasePulse(nAxisNo - 6);
		}
		return lCurPos;
	}
}

T_ANGLE_PULSE CRobotDriverAdaptor::GetCurrentPulse()
{
	T_ANGLE_PULSE tRobotCurPulses;
	tRobotCurPulses.nSPulse = GetCurrentPulse(SAxis);
	tRobotCurPulses.nLPulse = GetCurrentPulse(LAxis);
	tRobotCurPulses.nUPulse = GetCurrentPulse(UAxis);
	tRobotCurPulses.nRPulse = GetCurrentPulse(RAxis);
	tRobotCurPulses.nBPulse = GetCurrentPulse(BAxis);
	tRobotCurPulses.nTPulse = GetCurrentPulse(TAxis);
	tRobotCurPulses.lBXPulse = GetCurrentPulse(BPXAxis);
	tRobotCurPulses.lBYPulse = GetCurrentPulse(BPYAxis);
	tRobotCurPulses.lBZPulse = GetCurrentPulse(BPZAxis);

	return tRobotCurPulses;
}

double CRobotDriverAdaptor::GetPositionDis()
{
	double dCurPos;
	m_pvpMotorDriver->at(0)->GetCurrentPosition(dCurPos);
	return dCurPos;
	/*return 0.0;*/
}

int CRobotDriverAdaptor::MoveExAxisForLineScan(double dDist, int nSpeed)
{
	return 0;
}

int CRobotDriverAdaptor::MoveToLineScanPos(T_ANGLE_PULSE tPulse, T_ROBOT_MOVE_SPEED tPulseMove)
{
	return 0;
}

// 埃斯顿专用: 请查看EstunAdaptor类实现
bool CRobotDriverAdaptor::WorldIsRunning()
{
	return false;
}

bool CRobotDriverAdaptor::TeachMove(const std::vector<T_ANGLE_PULSE>& vtMeasurePulse, double dExAxlePos, const std::vector<int>& vnMeasureType, int nTrigSigTime)
{
	return false;
}

int CRobotDriverAdaptor::ContiMoveAny(const std::vector<T_ROBOT_MOVE_INFO>& vtRobotMoveInfo)
{
	SetMoveValue(vtRobotMoveInfo);
	CallJob("CONTIMOVANY");
	return 0;
}

int CRobotDriverAdaptor::WeldMove(const std::vector<T_ROBOT_COORS>& vtWeldPathPoints, const std::vector<int>& vnPtnType, const T_WELD_PARA& tWeldPara, double dExAxlePos, E_WELD_SEAM_TYPE eWeldSeamType, bool bIsArcOn)
{
	return 0;
}

//（问题：①使用目录 ./Data/RobotAndCar.ini 新框架无此目录结构 ②未完成SwitchIO()定义：该函数依赖SwitchIO()  该函数已完成定义：注释依赖项）
//bool CRobotDriverAdaptor::CleanGunH(int RobotNum)
//{
//	if (g_bLocalDebugMark) return true;
//	//位置过低时先抬枪再回到安全位置
//	CHECK_BOOL_RETURN(CheckIsReadyRun());
//	double NowZ = GetCurrentPos(ROBOT_AXIS_Z);
//	if (NowZ < m_tRobotLimitation.drTableZ + 550) {
//		PosMove(ROBOT_AXIS_Z, m_tRobotLimitation.drTableZ + 550.0, 500, COORD_ABS);
//		CheckRobotDone();
//		NowZ = GetCurrentPos(ROBOT_AXIS_Z);
//		if (NowZ != m_tRobotLimitation.drTableZ + 550.0) {
//			XiMessageBox("当前位置过低，自动抬枪失败，请手动抬枪");
//			return FALSE;
//		}
//	}
//	vector<T_ROBOT_COORS> vtRobotCoor;
//	MP_USR_VAR_INFO tUsrVarInfo;
//	vector<MP_USR_VAR_INFO> vtUsrVarInfo;
//	COPini opini;
//	opini.SetFileName("./data/RobotAndCar.ini");
//	opini.SetSectionName("ClearGunParam");
//
//	T_ANGLE_PULSE tCleanInitPlus;//清枪标志位(关节)
//	T_ROBOT_COORS tCleanInitCoord; //清枪标志位(直角) 
//	if (RobotNum == 0) {
//		opini.ReadString("CleanInitPlus0.nSPulse", &tCleanInitPlus.nSPulse);
//		opini.ReadString("CleanInitPlus0.nLPulse", &tCleanInitPlus.nLPulse);
//		opini.ReadString("CleanInitPlus0.nUPulse", &tCleanInitPlus.nUPulse);
//		opini.ReadString("CleanInitPlus0.nRPulse", &tCleanInitPlus.nRPulse);
//		opini.ReadString("CleanInitPlus0.nBPulse", &tCleanInitPlus.nBPulse);
//		opini.ReadString("CleanInitPlus0.nTPulse", &tCleanInitPlus.nTPulse);
//	}
//	else {
//		opini.ReadString("CleanInitPlus1.nSPulse", &tCleanInitPlus.nSPulse);
//		opini.ReadString("CleanInitPlus1.nLPulse", &tCleanInitPlus.nLPulse);
//		opini.ReadString("CleanInitPlus1.nUPulse", &tCleanInitPlus.nUPulse);
//		opini.ReadString("CleanInitPlus1.nRPulse", &tCleanInitPlus.nRPulse);
//		opini.ReadString("CleanInitPlus1.nBPulse", &tCleanInitPlus.nBPulse);
//		opini.ReadString("CleanInitPlus1.nTPulse", &tCleanInitPlus.nTPulse);
//
//	}
//	RobotKinematics(tCleanInitPlus, m_tTools.tGunTool, tCleanInitCoord);
//
//	T_ROBOT_COORS tClearShifting;   //清枪偏移距离
//	T_ROBOT_COORS tCutSilkShifting; //
//	T_ROBOT_COORS tOilingShifting;
//
//	if (RobotNum == 0) {
//		opini.ReadString("ClearShifting0.dX", &tClearShifting.dX);
//		opini.ReadString("ClearShifting0.dY", &tClearShifting.dY);
//		opini.ReadString("ClearShifting0.dZ", &tClearShifting.dZ);
//		opini.ReadString("CutSilkShifting0.dX", &tCutSilkShifting.dX);
//		opini.ReadString("CutSilkShifting0.dY", &tCutSilkShifting.dY);
//		opini.ReadString("CutSilkShifting0.dZ", &tCutSilkShifting.dZ);
//		opini.ReadString("Oiling0.dX", &tOilingShifting.dX);
//		opini.ReadString("Oiling0.dY", &tOilingShifting.dY);
//		opini.ReadString("Oiling0.dZ", &tOilingShifting.dZ);
//
//	}
//	else {
//		opini.ReadString("ClearShifting1.dX", &tClearShifting.dX);
//		opini.ReadString("ClearShifting1.dY", &tClearShifting.dY);
//		opini.ReadString("ClearShifting1.dZ", &tClearShifting.dZ);
//		opini.ReadString("CutSilkShifting1.dX", &tCutSilkShifting.dX);
//		opini.ReadString("CutSilkShifting1.dY", &tCutSilkShifting.dY);
//		opini.ReadString("CutSilkShifting1.dZ", &tCutSilkShifting.dZ);
//		opini.ReadString("Oiling1.dX", &tOilingShifting.dX);
//		opini.ReadString("Oiling1.dY", &tOilingShifting.dY);
//		opini.ReadString("Oiling1.dZ", &tOilingShifting.dZ);
//	}
//
//
//	double realCleanerAngle;		//清枪器实际安装角度
//	if (RobotNum == 0) {
//		opini.ReadString("RealCleanerAngle0", &realCleanerAngle);
//	}
//	else {
//		opini.ReadString("RealCleanerAngle1", &realCleanerAngle);
//	}
//
//	//清枪点
//	T_ROBOT_COORS tClearGunPoint(tCleanInitCoord);
//	tClearGunPoint.dX += tClearShifting.dX;
//	tClearGunPoint.dY += tClearShifting.dY * SinD(realCleanerAngle) * m_nRobotInstallDir;
//	tClearGunPoint.dZ += tClearShifting.dZ * m_nRobotInstallDir;
//
//	//剪丝点
//	T_ROBOT_COORS tCutSilkPoint(tCleanInitCoord);
//	tCutSilkPoint.dX = tCutSilkPoint.dX + tCutSilkShifting.dX;
//	tCutSilkPoint.dY = tCutSilkPoint.dY + tCutSilkShifting.dY * SinD(realCleanerAngle) * m_nRobotInstallDir;
//	tCutSilkPoint.dZ = tCutSilkPoint.dZ + tCutSilkShifting.dZ * m_nRobotInstallDir;
//
//	//喷油点
//	T_ROBOT_COORS tOilingPoint(tCleanInitCoord);
//	tOilingPoint.dX = tOilingPoint.dX + tOilingShifting.dX;
//	tOilingPoint.dY = tOilingPoint.dY + tOilingShifting.dY * SinD(realCleanerAngle) * m_nRobotInstallDir;
//	tOilingPoint.dZ = tOilingPoint.dZ + tOilingShifting.dZ * m_nRobotInstallDir;
//
//	//清枪过渡点(上)
//	T_ROBOT_COORS tClearGunExcessivePointBefor(tClearGunPoint);
//	tClearGunExcessivePointBefor.dX += 200.0;
//	tClearGunExcessivePointBefor.dZ -= 300.0;
//
//	//清枪过渡点(下）
//	T_ROBOT_COORS tClearGunExcessivePoint(tClearGunPoint);
//	tClearGunExcessivePoint.dZ -= 300;
//
//	//剪丝过渡点
//	T_ROBOT_COORS tCutSilkExcssivePoint(tCutSilkPoint);
//	tCutSilkExcssivePoint.dZ += 100;
//
//	//喷油过渡点
//	T_ROBOT_COORS tOilingExcssivePoint(tOilingPoint);
//	tOilingExcssivePoint.dZ += 100;
//	/****************************/
//	T_ROBOT_MOVE_SPEED clean(m_tBackHomeSpeed);
//	clean.dSpeed = 1000;
//	MoveByJob(m_tHomePulse, m_tBackHomeSpeed, 0); //回安全位置
//
//	MoveByJob(tClearGunExcessivePointBefor, clean, 0, "MOVJ");
//	CheckRobotDone(200);
//	MoveByJob(tClearGunExcessivePoint, clean, 0, "MOVJ"); //回清枪过渡点
//	clean.dSpeed = 200;
//	CheckRobotDone(200);
//	//依赖项↓
//	//SwitchIO("clampOrCut", true); //剪丝(松开)
//	Sleep(200);
//	MoveByJob(tClearGunPoint, clean, 0, "MOVL"); //回清枪点
//	CheckRobotDone(200);
//	clean.dSpeed = 400;
//	//依赖项↓
//	//SwitchIO("clampOrCut", false); //松开剪丝(加紧)
//	Sleep(200);
//	//依赖项↓
//	//SwitchIO("motorRaise", true); //马达上升
//	Sleep(200);
//	//依赖项↓
//	//SwitchIO("motorSpin", true); //马达旋转
//	Sleep(2500);
//	//依赖项↓
//	//SwitchIO("motorSpin", false); //马达停止旋转
//	Sleep(10);
//	//依赖项↓
//	//SwitchIO("motorRaise", false); //马达下降
//	Sleep(10);
//	//依赖项↓
//	//SwitchIO("clampOrCut", true); //剪丝(松开)
//	Sleep(200);
//	MoveByJob(tClearGunExcessivePoint, m_tBackHomeSpeed, 0, "MOVL"); //回清枪过渡点
//	CheckRobotDone(200);
//
//	MoveByJob(tCutSilkExcssivePoint, clean, 0, "MOVL"); //回剪丝过渡点
//	CheckRobotDone(200);
//	//依赖项↓
//	//SwitchIO("clampOrCut", false); //松开剪丝(加紧)
//	Sleep(200);
//	MoveByJob(tCutSilkPoint, clean, 0, "MOVL"); //回剪丝点
//	CheckRobotDone(200);
//	//依赖项↓
//	//SwitchIO("clampOrCut", true); //剪丝(松开)
//	Sleep(200);
//	//依赖项↓
//	//SwitchIO("clampOrCut", false); //松开剪丝(加紧)
//	Sleep(200);
//	MoveByJob(tCutSilkExcssivePoint, m_tBackHomeSpeed, 0, "MOVL"); //回剪丝过渡点
//	CheckRobotDone(200);
//	MoveByJob(tOilingExcssivePoint, m_tBackHomeSpeed, 0, "MOVL"); //回喷油过渡点
//	CheckRobotDone(200);
//	MoveByJob(tOilingPoint, m_tBackHomeSpeed, 0, "MOVL"); //回喷油点
//	CheckRobotDone(200);
//	MoveByJob(tOilingExcssivePoint, m_tBackHomeSpeed, 0, "MOVL"); //回喷油过渡点
//	CheckRobotDone(200);
//	MoveByJob(m_tHomePulse, m_tBackHomeSpeed, 0); //回安全位置
//	CheckRobotDone(200);
//	XiMessageBox("清枪完成！");
//	return true;
//}

double CRobotDriverAdaptor::GetCurrentDegree(int nAxisNo)
{
	if (g_bLocalDebugMark) return 0.0;
	switch (m_eRobotBrand) {
	case ROBOT_BRAND_YASKAWA:		//安川
	{
		if (nAxisNo < 6) {
			double dAngle = m_pYaskawaRobotCtrl->GetCurrentDegree(nAxisNo);
			dAngle = dAngle / 1000.0;
			return dAngle;
		}
		else {
			return 0.0;
		}
	}
	break;
	case ROBOT_BRAND_ESTUN:			//埃斯顿
		break;
	case ROBOT_BRAND_CRP:				//卡诺普
	{
		double pos[10] = { 0.0 };
		m_pCrpRobotCtrl->GetCurrAxis(pos);
		return pos[nAxisNo];
	}
	break;
	default:
		break;
	}
	return 0.0;
}

void CRobotDriverAdaptor::PosMove(int nAxisNo, double dDist, long lRobotSpd, WORD wCoorType, int nToolNo, long lCoordFrm)
{
	if (g_bLocalDebugMark) return;
	switch (m_eRobotBrand) {
	case ROBOT_BRAND_YASKAWA:		//安川
	{
		if (nAxisNo < 6) {
			m_pYaskawaRobotCtrl->PosMove(nAxisNo, dDist, wCoorType, nToolNo, lRobotSpd, lCoordFrm);
		}
		else {
			return;
		}
	}
	break;
	case ROBOT_BRAND_ESTUN:			//埃斯顿
		break;
	case ROBOT_BRAND_CRP:				//卡诺普
	{
		lRobotSpd = lRobotSpd / 60;
		double pos[10] = { 0 };
		int result = m_pCrpRobotCtrl->GetCurrPos(pos);
		pos[nAxisNo] = pos[nAxisNo] + dDist;
		result = m_pCrpRobotCtrl->MoveByJob_L(pos, lRobotSpd, nToolNo, lCoordFrm, 0, 0);
		if (result >= 0) {
			m_pCrpRobotCtrl->XiRobot_CallJob(1);
		}
	}
	break;
	default:
		break;
	}
	return;
}

void CRobotDriverAdaptor::PosMove(double adRobotCoord[10], long lRobotSpd, WORD wCoorType, int nToolNo, long lCoordFrm)
{
	if (g_bLocalDebugMark) return;
	switch (m_eRobotBrand) {
	case ROBOT_BRAND_YASKAWA:		//安川
		m_pYaskawaRobotCtrl->PosMove(adRobotCoord, wCoorType, nToolNo, lRobotSpd, lCoordFrm);
		break;
	case ROBOT_BRAND_ESTUN:			//埃斯顿
		break;
	case ROBOT_BRAND_CRP:				//卡诺普
	{
		double pos[10] = { 0.0 };
		m_pCrpRobotCtrl->GetAppointToolPos(pos, 1, 0); // 默认工具1 用户0
		pos[0] = adRobotCoord[0];
		pos[1] = adRobotCoord[1];
		pos[2] = adRobotCoord[2];
		pos[3] = adRobotCoord[3];
		pos[4] = adRobotCoord[4];
		pos[5] = adRobotCoord[5];
		int result = m_pCrpRobotCtrl->MoveByJob_L(pos, lRobotSpd / 60, nToolNo, lCoordFrm, 0, 0);
		if (result >= 0) {
			m_pCrpRobotCtrl->XiRobot_CallJob(1);
		}
	}
	break;
	default:
		break;
	}
}


int CRobotDriverAdaptor::AxisPulseMove(int nAxisNo, long lPulse, long lRobotSpd, WORD wCoorType, int nToolNo)
{
	if (g_bLocalDebugMark) return TRUE;
	switch (m_eRobotBrand) {
	case ROBOT_BRAND_YASKAWA:		//安川
		m_pYaskawaRobotCtrl->AxisPulseMove(nAxisNo, lPulse, wCoorType, nToolNo, lRobotSpd);
		break;
	case ROBOT_BRAND_ESTUN:			//埃斯顿
		break;
	case ROBOT_BRAND_CRP:				//卡诺普
	{
		double cur[10] = { 0.0 };
		if (m_pCrpRobotCtrl->GetCurrAxisByAngle(cur) >= 0) {
			switch (nAxisNo) {
			case 0: cur[nAxisNo] = lPulse * m_tAxisUnit.dSPulse; break;
			case 1: cur[nAxisNo] = lPulse * m_tAxisUnit.dLPulse; break;
			case 2: cur[nAxisNo] = lPulse * m_tAxisUnit.dUPulse; break;
			case 3: cur[nAxisNo] = lPulse * m_tAxisUnit.dRPulse; break;
			case 4: cur[nAxisNo] = lPulse * m_tAxisUnit.dBPulse; break;
			case 5: cur[nAxisNo] = lPulse * m_tAxisUnit.dTPulse; break;
			case 6: cur[nAxisNo] = lPulse * m_tAxisUnit.dTPulse; break;
			case 7: cur[nAxisNo] = lPulse * m_tAxisUnit.dTPulse; break;
			default:
				break;
			}
			if (m_pCrpRobotCtrl->MoveByJob_J_Axis(cur, lRobotSpd / 100, 1, 0, 0, 0) >= 0) {
				m_pCrpRobotCtrl->XiRobot_CallJob(2);
				return TRUE;
			}
		}
	}
	break;
	default:
		break;
	}
	return FALSE;
}

void CRobotDriverAdaptor::MoveToAbsPluse(long alAbsPulse[6], long lSpeedRatio)
{
	if (g_bLocalDebugMark) return;
	switch (m_eRobotBrand) {
	case ROBOT_BRAND_YASKAWA:		//安川
		m_pYaskawaRobotCtrl->MoveToAbsPluse(alAbsPulse, 1, lSpeedRatio);
		break;
	case ROBOT_BRAND_ESTUN:			//埃斯顿
		break;
	case ROBOT_BRAND_CRP:				//卡诺普
	{
		double pos[10] = { 0.0 };
		m_pCrpRobotCtrl->GetCurrAxisByAngle(pos);

		pos[0] = alAbsPulse[0] * m_tAxisUnit.dSPulse;
		pos[1] = alAbsPulse[1] * m_tAxisUnit.dLPulse;
		pos[2] = alAbsPulse[2] * m_tAxisUnit.dUPulse;
		pos[3] = alAbsPulse[3] * m_tAxisUnit.dRPulse;
		pos[4] = alAbsPulse[4] * m_tAxisUnit.dBPulse;
		pos[5] = alAbsPulse[5] * m_tAxisUnit.dTPulse;
		int result = m_pCrpRobotCtrl->MoveByJob_J_Axis(pos, lSpeedRatio / 100, 1, 0, 0, 0); //工具号和用户号固定为1和0
		if (result >= 0) {
			m_pCrpRobotCtrl->XiRobot_CallJob(2);
		}
	}
	break;
	default:
		break;
	}
}


void CRobotDriverAdaptor::MoveByJob(double* dRobotJointCoord, T_ROBOT_MOVE_SPEED tPulseMove, int nExternalAxleType, int nPVarType, CString JobName)
{
	if (g_bLocalDebugMark) return;
	switch (m_eRobotBrand) {
	case ROBOT_BRAND_YASKAWA:		//安川
	{
		int nMaxSpeed = 20000;
		if (JobName == "MOVJ") {
			nMaxSpeed = 10000;
		}
		if (tPulseMove.dSpeed < 1) {
			tPulseMove.dSpeed = 1;
		}
		else if (tPulseMove.dSpeed > nMaxSpeed) {
			tPulseMove.dSpeed = nMaxSpeed;
		}
		if (tPulseMove.dACC < 20) {
			tPulseMove.dACC = 20;
		}
		else if (tPulseMove.dACC > 100) {
			tPulseMove.dACC = 100;
		}
		if (tPulseMove.dDEC < 20) {
			tPulseMove.dDEC = 20;
		}
		else if (tPulseMove.dDEC > 100) {
			tPulseMove.dDEC = 100;
		}
		long lSpeed[3];
		lSpeed[0] = tPulseMove.dSpeed;
		lSpeed[1] = tPulseMove.dACC;
		lSpeed[2] = tPulseMove.dDEC;
		m_pYaskawaRobotCtrl->SetMultiVar(3, 5, lSpeed);
		m_pYaskawaRobotCtrl->SetPosVar(0, dRobotJointCoord, nPVarType);

		if (54 != nExternalAxleType) // 国焊立焊示教临时
		{
			////设置BP
			if (nExternalAxleType > 0)
			{
				if ((((nExternalAxleType >> 3) & 0x01) == 1) ||
					(((nExternalAxleType >> 4) & 0x01) == 1) ||
					(((nExternalAxleType >> 5) & 0x01) == 1)) {
					double dBasePos[3];
					dBasePos[0] = dRobotJointCoord[6];
					dBasePos[1] = dRobotJointCoord[7];
					dBasePos[2] = dRobotJointCoord[8];
					ReloadBP(dBasePos, nExternalAxleType);
					m_pYaskawaRobotCtrl->SetBasePosVar(0, dBasePos, nPVarType);
					JobName += "-BP";
				}
			}
		}
		CallJob(JobName);
	}
	break;
	case ROBOT_BRAND_ESTUN:			//埃斯顿
		break;
	case ROBOT_BRAND_CRP:				//卡诺普
		break;
	default:
		break;
	}

}

void CRobotDriverAdaptor::MoveByJob(T_ROBOT_COORS tRobotJointCoord, T_ROBOT_MOVE_SPEED tPulseMove, int nExternalAxleType, CString JobName /*= "MOVJ"*/)
{
	if (g_bLocalDebugMark) return;
	switch (m_eRobotBrand) {
	case ROBOT_BRAND_YASKAWA:		//安川
	{
		double dPos[9] = { 0,0,0,0,0,0,0,0,0 };
		dPos[0] = tRobotJointCoord.dX;
		dPos[1] = tRobotJointCoord.dY;
		dPos[2] = tRobotJointCoord.dZ;
		dPos[3] = tRobotJointCoord.dRX;
		dPos[4] = tRobotJointCoord.dRY;
		dPos[5] = tRobotJointCoord.dRZ;
		dPos[6] = tRobotJointCoord.dBX;
		dPos[7] = tRobotJointCoord.dBY;
		dPos[8] = tRobotJointCoord.dBZ;
		MoveByJob(dPos, tPulseMove, nExternalAxleType, ROBOT_COORD, JobName);
	}
	break;
	case ROBOT_BRAND_ESTUN:			//埃斯顿
		break;
	case ROBOT_BRAND_CRP:				//卡诺普
	{
		if (JobName == "MOVL") {
			double dPos[10] = { 0 };
			dPos[0] = tRobotJointCoord.dX;
			dPos[1] = tRobotJointCoord.dY;
			dPos[2] = tRobotJointCoord.dZ;
			dPos[3] = tRobotJointCoord.dRX;
			dPos[4] = tRobotJointCoord.dRY;
			dPos[5] = tRobotJointCoord.dRZ;
			dPos[6] = tRobotJointCoord.dBX;
			dPos[7] = tRobotJointCoord.dBY;
			double cur[10] = { 0 };
			//m_pCrpRobotCtrl->GetCurrPos(cur);
			m_pCrpRobotCtrl->GetAppointToolPos(cur, 1, 0); // 默认工具1 用户0  ！！！！+
			dPos[8] = cur[8];
			dPos[9] = cur[9];
			int speed = tPulseMove.dSpeed / 60;
			m_pCrpRobotCtrl->MoveByJob_L(dPos, speed, cur[8], cur[9], tPulseMove.dACC, tPulseMove.dDEC);
			m_pCrpRobotCtrl->XiRobot_CallJob(1);
		}
		else if (JobName == "MOVJ") {
			double dPos[10] = { 0 };
			dPos[0] = tRobotJointCoord.dX * m_tAxisUnit.dSPulse;
			dPos[1] = tRobotJointCoord.dY * m_tAxisUnit.dLPulse;
			dPos[2] = tRobotJointCoord.dZ * m_tAxisUnit.dUPulse;
			dPos[3] = tRobotJointCoord.dRX * m_tAxisUnit.dRPulse;
			dPos[4] = tRobotJointCoord.dRY * m_tAxisUnit.dBPulse;
			dPos[5] = tRobotJointCoord.dRZ * m_tAxisUnit.dTPulse;
			dPos[6] = tRobotJointCoord.dBX * m_tAxisUnit.dTPulse;
			dPos[7] = tRobotJointCoord.dBY * m_tAxisUnit.dTPulse;
			double cur[10] = { 0 };
			m_pCrpRobotCtrl->GetCurrAxisByAngle(cur); // 默认工具1 用户0  ！！！！+
			dPos[8] = cur[6];
			dPos[9] = cur[7];
			int speed = tPulseMove.dSpeed / 100;
			m_pCrpRobotCtrl->MoveByJob_J_Axis(dPos, speed, cur[8], cur[9], tPulseMove.dACC, tPulseMove.dDEC);
			m_pCrpRobotCtrl->XiRobot_CallJob(2);
		}
	}
	break;
	default:
		break;
	}

}

void CRobotDriverAdaptor::MoveByJob(T_ANGLE_PULSE tRobotJointCoord, T_ROBOT_MOVE_SPEED tPulseMove, int nExternalAxleType, CString JobName)
{
	if (g_bLocalDebugMark) return;
	switch (m_eRobotBrand) {
	case ROBOT_BRAND_YASKAWA:		//安川
	{
		double dPos[9] = { 0,0,0,0,0,0,0,0,0 };
		dPos[0] = tRobotJointCoord.nSPulse;
		dPos[1] = tRobotJointCoord.nLPulse;
		dPos[2] = tRobotJointCoord.nUPulse;
		dPos[3] = tRobotJointCoord.nRPulse;
		dPos[4] = tRobotJointCoord.nBPulse;
		dPos[5] = tRobotJointCoord.nTPulse;
		dPos[6] = tRobotJointCoord.lBXPulse;
		dPos[7] = tRobotJointCoord.lBYPulse;
		dPos[8] = tRobotJointCoord.lBZPulse;
		MoveByJob(dPos, tPulseMove, nExternalAxleType, PULSE_COORD, JobName);
	}
	break;
	case ROBOT_BRAND_ESTUN:			//埃斯顿
		break;
	case ROBOT_BRAND_CRP:				//卡诺普
	{
		if (JobName == "MOVJ") {
			double dPos[8] = { 0 };
			dPos[0] = tRobotJointCoord.nSPulse * m_tAxisUnit.dSPulse; // ！！！！+
			dPos[1] = tRobotJointCoord.nLPulse * m_tAxisUnit.dLPulse;
			dPos[2] = tRobotJointCoord.nUPulse * m_tAxisUnit.dUPulse;
			dPos[3] = tRobotJointCoord.nRPulse * m_tAxisUnit.dRPulse;
			dPos[4] = tRobotJointCoord.nBPulse * m_tAxisUnit.dBPulse;
			dPos[5] = tRobotJointCoord.nTPulse * m_tAxisUnit.dTPulse;
			dPos[6] = tRobotJointCoord.lBXPulse * m_tAxisUnit.dTPulse;
			dPos[7] = 0;

			double speed = tPulseMove.dSpeed / 100;
			double acc = tPulseMove.dACC;
			double dec = tPulseMove.dDEC;
			m_pCrpRobotCtrl->MoveByJob_J_Axis(dPos, speed, 1, 0, acc, dec);
			m_pCrpRobotCtrl->XiRobot_CallJob(2);
		}
		else if (JobName == "MOVL") {
			double dPos[8] = { 0 };
			dPos[0] = tRobotJointCoord.nSPulse; // ！！！！+
			dPos[1] = tRobotJointCoord.nLPulse;
			dPos[2] = tRobotJointCoord.nUPulse;
			dPos[3] = tRobotJointCoord.nRPulse;
			dPos[4] = tRobotJointCoord.nBPulse;
			dPos[5] = tRobotJointCoord.nTPulse;
			dPos[6] = tRobotJointCoord.lBXPulse;
			dPos[7] = 0;

			double speed = tPulseMove.dSpeed / 60;
			double acc = tPulseMove.dACC;
			double dec = tPulseMove.dDEC;
			m_pCrpRobotCtrl->MoveByJob_L(dPos, speed, 1, 0, acc, dec);
			m_pCrpRobotCtrl->XiRobot_CallJob(1);
		}
	}
	break;
	default:
		break;
	}

}

void CRobotDriverAdaptor::MoveByJob(long* alRobotJointCoord, T_ROBOT_MOVE_SPEED tPulseMove, int nExternalAxleType, CString JobName)
{
	if (g_bLocalDebugMark) return;
	switch (m_eRobotBrand) {
	case ROBOT_BRAND_YASKAWA:		//安川
	{
		double dPos[9] = { 0,0,0,0,0,0,0,0,0 };
		for (int i = 0; i < 9; i++) {
			dPos[i] = alRobotJointCoord[i];
		}
		MoveByJob(dPos, tPulseMove, nExternalAxleType, PULSE_COORD, JobName);
	}
	break;
	case ROBOT_BRAND_ESTUN:			//埃斯顿
		break;
	case ROBOT_BRAND_CRP:				//卡诺普
	{
		if (JobName == "MOVJ") {
			double dPos[8] = { 0 };
			dPos[0] = alRobotJointCoord[0] * m_tAxisUnit.dSPulse; // 脉冲转角度 ！！！！+
			dPos[1] = alRobotJointCoord[1] * m_tAxisUnit.dLPulse;
			dPos[2] = alRobotJointCoord[2] * m_tAxisUnit.dUPulse;
			dPos[3] = alRobotJointCoord[3] * m_tAxisUnit.dRPulse;
			dPos[4] = alRobotJointCoord[4] * m_tAxisUnit.dBPulse;
			dPos[5] = alRobotJointCoord[5] * m_tAxisUnit.dTPulse;
			dPos[6] = alRobotJointCoord[6] * m_tAxisUnit.dTPulse;
			dPos[7] = alRobotJointCoord[7] * m_tAxisUnit.dTPulse;

			double speed = tPulseMove.dSpeed / 100;
			double acc = tPulseMove.dACC;
			double dec = tPulseMove.dDEC;
			m_pCrpRobotCtrl->MoveByJob_J_Axis(dPos, speed, 1, 0, acc, dec);
			m_pCrpRobotCtrl->XiRobot_CallJob(2);
		}
		else if (JobName == "MOVL") {
			double dPos[10] = { 0 };
			dPos[0] = alRobotJointCoord[0] / 1000;
			dPos[1] = alRobotJointCoord[0] / 1000;
			dPos[2] = alRobotJointCoord[0] / 1000;
			dPos[3] = alRobotJointCoord[0] / 10000;
			dPos[4] = alRobotJointCoord[0] / 10000;
			dPos[5] = alRobotJointCoord[0] / 10000;
			dPos[6] = alRobotJointCoord[0] / 10000;
			dPos[7] = 0;
			dPos[8] = 1;
			dPos[9] = 0;

			int speed = tPulseMove.dSpeed / 60;
			m_pCrpRobotCtrl->MoveByJob_L(dPos, speed, 1, 0, tPulseMove.dACC, tPulseMove.dDEC);
			m_pCrpRobotCtrl->XiRobot_CallJob(1);
		}
	}
	break;
	default:
		break;
	}

}

bool CRobotDriverAdaptor::DropGun(std::vector<double*> pos, std::vector<T_ROBOT_MOVE_SPEED> myspeed, std::vector<int> movType)
{
	if (g_bLocalDebugMark) return true;
	switch (m_eRobotBrand) {
	case ROBOT_BRAND_YASKAWA:		//安川
		break;
	case ROBOT_BRAND_ESTUN:			//埃斯顿
		break;
	case ROBOT_BRAND_CRP:				//卡诺普
	{
		string changeTool_str; //存入文件的工具字符串
		string changeUser_str; //存入文件的用户字符串
		string end;            //最后一行结束的字符串
		string mEnd, mStrat;
		int lineNum = 1;       //行号
		m_pCrpRobotCtrl->MakeMout(500, true, lineNum++, &mStrat);
		m_pCrpRobotCtrl->MakeChangeTool(1, lineNum++, changeTool_str);
		m_pCrpRobotCtrl->MakeChangeUse(0, lineNum++, changeUser_str);
		std::ofstream RecordFile("Move_L"); //用记事本的方式记录下来
		RecordFile << mStrat << "\n";
		RecordFile << changeTool_str << "\n";
		RecordFile << changeUser_str << "\n";

		int size = pos.size();
		if (pos.size() != myspeed.size() || pos.size() != movType.size()) {
			return false;
		}
		for (int i = 0; i < size; i++) {
			if (movType[i] == MOVJ) {
				string tmp;
				double Axis[10] = { pos[i][0] * m_tAxisUnit.dSPulse, pos[i][1] * m_tAxisUnit.dLPulse, pos[i][2] * m_tAxisUnit.dUPulse,
									pos[i][3] * m_tAxisUnit.dRPulse, pos[i][4] * m_tAxisUnit.dBPulse, pos[i][5] * m_tAxisUnit.dTPulse, pos[i][6] * m_tAxisUnit.dTPulse, 0, 1,0 };
				m_pCrpRobotCtrl->MakeMoveJ_Axis(Axis, myspeed[i].dSpeed / 100, 1, 0, 0, 0, 9, lineNum++, &tmp);
				RecordFile << tmp << "\n";
			}
			else if (movType[i] == MOVL) {
				string tmp;
				m_pCrpRobotCtrl->MakeMoveL(pos[i], myspeed[i].dSpeed / 60, 1, 0, 0, 0, 9, lineNum++, &tmp);
				RecordFile << tmp << "\n";
			}
		}
		m_pCrpRobotCtrl->MakeMout(500, false, lineNum++, &mEnd);
		m_pCrpRobotCtrl->programerEnd(lineNum, &end);
		RecordFile << mEnd << "\n";
		RecordFile << end << "\n";

		RecordFile.close();
		//发送文件
		if (!m_pCrpRobotCtrl->Obj_FTP_cmd->FTP_if_load) return false;
		bool result = m_pCrpRobotCtrl->Obj_FTP_cmd->FTP_SendFile("Move_L");
		if (!result) return false;

		m_pCrpRobotCtrl->XiRobot_CallJob(1);
	}
	break;
	default:
		break;
	}
	return true;
}

void CRobotDriverAdaptor::MoveByJob(double* dRobotJointCoord, T_ROBOT_MOVE_SPEED tPulseMove, int nExternalAxleType, int nPVarType, CString JobName, int config[7])
{
	XUI::MesBox::PopError("非埃斯顿不可使用");
}

void CRobotDriverAdaptor::MoveByJob(T_ROBOT_COORS tRobotJointCoord, T_ROBOT_MOVE_SPEED tPulseMove, int nExternalAxleType, CString JobName, int config[7])
{
	XUI::MesBox::PopError("非埃斯顿不可使用");
}

void CRobotDriverAdaptor::HoldOn()
{
	switch (m_eRobotBrand) {
	case ROBOT_BRAND_YASKAWA:		//安川
		m_pYaskawaRobotCtrl->HoldOn();
		break;
	case ROBOT_BRAND_ESTUN:			//埃斯顿
		break;
	case ROBOT_BRAND_CRP:				//卡诺普
	{
		int res = m_pCrpRobotCtrl->StopProgamer();
	}
	break;
	default:
		break;
	}
}

int CRobotDriverAdaptor::MoveExAxisFun(double dDist, int nSpeed, int nMoveXYZAxisNo, double dMaxExSpeed)
{
	return 0;
}

int CRobotDriverAdaptor::XYZAxisNoToSoftAxisNo(int nMoveXYZAxisNo)
{
	return 0;
}

double CRobotDriverAdaptor::GetExPositionDis(int nMoveXYZAxisNo)
{
	return 0.0;
}

void CRobotDriverAdaptor::CalModeValue(T_ANGLE_PULSE tReferPluse, const char* RobotMode, int config[7])
{
	return;
}

bool CRobotDriverAdaptor::EstunTrackInit(const T_ROBOT_COORS& startPos, const CString& IP, const vector<int>& Cfg)
{
	return false;
}

bool CRobotDriverAdaptor::EstunClearWeldBuffer()
{
	return false;
}

bool CRobotDriverAdaptor::EstunSendTrackData(const vector<T_ROBOT_COORS>& WeldPathPoint, const vector<vector<int>>& Cfg)
{
	return false;
}

bool CRobotDriverAdaptor::EstunTrackStop()
{
	return false;
}

void CRobotDriverAdaptor::HoldOff()
{
	if (GetLocalDebugMark())
	{
		return;
	}
	switch (m_eRobotBrand) {
	case ROBOT_BRAND_YASKAWA:		//安川
		m_pYaskawaRobotCtrl->HoldOff();
		break;
	case ROBOT_BRAND_ESTUN:			//埃斯顿
		break;
	case ROBOT_BRAND_CRP:				//卡诺普
		break;
	default:
		break;
	}
}

void CRobotDriverAdaptor::ServoOn()
{
	if (g_bLocalDebugMark){
		return;
	}
	switch (m_eRobotBrand) {
	case ROBOT_BRAND_YASKAWA:		//安川
		m_pYaskawaRobotCtrl->ServoOn();
		break;
	case ROBOT_BRAND_ESTUN:			//埃斯顿
		break;
	case ROBOT_BRAND_CRP:				//卡诺普
	{
		int state = m_pCrpRobotCtrl->Read_M_Number_status(20); //判断当前状态是否已上电
		if (state == 0) {
			m_pCrpRobotCtrl->Enable();
		}
	}
	break;
	default:
		break;
	}
}

void CRobotDriverAdaptor::CleanAlarm()
{
	switch (m_eRobotBrand) {
	case ROBOT_BRAND_YASKAWA:		//安川
		m_pYaskawaRobotCtrl->ServoOn();
		break;
	case ROBOT_BRAND_ESTUN:			//埃斯顿
		break;
	case ROBOT_BRAND_CRP:				//卡诺普
		if (m_pCrpRobotCtrl->Obj_Modbus->m_connected) {
			int number = 512; //M512
			m_pCrpRobotCtrl->Write_M_Number_status(number, true);
			Sleep(200);
			m_pCrpRobotCtrl->Write_M_Number_status(number, false);
			Sleep(200);
			m_pCrpRobotCtrl->Write_M_Number_status(number, true);
			Sleep(200);
			m_pCrpRobotCtrl->Write_M_Number_status(number, false);
			Sleep(200);
		}
		break;
	default:
		break;
	}
}

void CRobotDriverAdaptor::ServoOff()
{
	switch (m_eRobotBrand) {
	case ROBOT_BRAND_YASKAWA:			//安川
		m_pYaskawaRobotCtrl->ServoOff();
		break;
	case ROBOT_BRAND_ESTUN:			//埃斯顿
		break;
	case ROBOT_BRAND_CRP:				//卡诺普
	{
		int result = m_pCrpRobotCtrl->Disable();
	}
	break;
	default:
		break;
	}
}

void CRobotDriverAdaptor::ConfigRobotPosture(unsigned int nRobotPosture)
{
	switch (m_eRobotBrand) {
	case ROBOT_BRAND_YASKAWA:			//安川
		m_pYaskawaRobotCtrl->ConfigRobotPosture(nRobotPosture);
		break;
	case ROBOT_BRAND_ESTUN:			//埃斯顿
		break;
	case ROBOT_BRAND_CRP:				//卡诺普
		break;
	default:
		break;
	}
}

BOOL CRobotDriverAdaptor::SetMultiVar_H(UINT unCount, MP_USR_VAR_INFO* pVarInfo)
{
	if (g_bLocalDebugMark) return TRUE;
	if (unCount <= 0) return FALSE;
	switch (m_eRobotBrand) {
	case ROBOT_BRAND_YASKAWA:		//安川
	{
		long long stratr = XI_clock();
		while (true == m_bInsterPVal) {
			if ((XI_clock() - stratr) > 400) {
				break;
			}
			DoEvent();
			Sleep(5);
		}
		m_bInsterPVal = true;

		Sleep(50);
		UINT i = 0;
		if (unCount > MAXPOSVARNO_H) {
			for (; i < unCount - MAXPOSVARNO_H; i = i + MAXPOSVARNO_H) {
				m_pYaskawaRobotCtrl->SetMultiVars_H(MAXPOSVARNO_H, &pVarInfo[i]);
			}
		}
		if (i < unCount) {
			m_pYaskawaRobotCtrl->SetMultiVars_H(unCount - i, &pVarInfo[i]);
		}
		m_bInsterPVal = false;
		return m_bInsterPVal;
	}
	break;
	case ROBOT_BRAND_ESTUN:			//埃斯顿
		break;
	case ROBOT_BRAND_CRP:				//卡诺普
		break;
	default:
		break;
	}
	return FALSE;
}

void CRobotDriverAdaptor::SetMultiVar_H(const std::vector<MP_USR_VAR_INFO> vtVarInfo)
{
	if (g_bLocalDebugMark) return;
	switch (m_eRobotBrand) {
	case ROBOT_BRAND_YASKAWA:		//安川
	{
		MP_USR_VAR_INFO* pVarInfo = (MP_USR_VAR_INFO*)malloc(sizeof(MP_USR_VAR_INFO) * vtVarInfo.size());
		for (int i = 0; i < vtVarInfo.size(); i++) {
			pVarInfo[i] = vtVarInfo[i];
		}
		SetMultiVar_H(vtVarInfo.size(), pVarInfo);
		free(pVarInfo);
		return;
	}
	break;
	case ROBOT_BRAND_ESTUN:			//埃斯顿
		break;
	case ROBOT_BRAND_CRP:				//卡诺普
		break;
	default:
		return;
	}
}

void CRobotDriverAdaptor::SetMultiVar_H_WriteSyncIo(int nSyncId, int nSyncIoNo, UINT unCount, MP_USR_VAR_INFO* pVarInfo)
{
	if (g_bLocalDebugMark) return;
	switch (m_eRobotBrand) {
	case ROBOT_BRAND_YASKAWA:		//安川
	{
		if (unCount <= 0)
		{
			return;
		}
		UINT i = 0;
		if (unCount > MAXPOSVARNO_H)
		{
			for (; i < unCount - MAXPOSVARNO_H; i = i + MAXPOSVARNO_H)
			{
				m_pYaskawaRobotCtrl->SetMultiVars_H(MAXPOSVARNO_H, &pVarInfo[i]);
			}
		}
		if (i < unCount)
		{
			WriteLog("SetMultiVar_H_WriteSyncIo nSyncId:%d nSyncIoNo:%d unCount - i:%d", nSyncId, nSyncIoNo, unCount - i);
			m_pYaskawaRobotCtrl->SetMultiVar_H_WriteSyncIo(nSyncId, nSyncIoNo, unCount - i, &pVarInfo[i]);
			WriteLog("SetMultiVar_H_WriteSyncIo.");
		}
		break;
	}
	case ROBOT_BRAND_ESTUN:			//埃斯顿
		break;
	case ROBOT_BRAND_CRP:				//卡诺普
		break;
	default:
		return;
	}
	return;
}

void CRobotDriverAdaptor::SetMultiBasePosVar(UINT unCount, UINT unIndex, long lRobotPulse[][3], int nExternalAxleType)
{
	if (g_bLocalDebugMark) return;
	switch (m_eRobotBrand) {
	case ROBOT_BRAND_YASKAWA:		//安川
	{
		T_ROBOT_POSVAR_DATA* pVarData = new T_ROBOT_POSVAR_DATA[unCount];
		for (UINT i = 0; i < unCount; i++)
		{
			//P变量赋值
			pVarData[i].usPorBPorExVar = MP_RESTYPE_VAR_BASE;
			pVarData[i].usIndex = unIndex + i;
			pVarData[i].usPulseOrCart = MP_PULSE_COORD;
			pVarData[i].usPosture = 4;
			pVarData[i].usToolNum = 1;
			pVarData[i].usUserCoordNum = 0;//该参数无意义		
			ReloadBP(lRobotPulse[i], nExternalAxleType);
			pVarData[i].ulValue[0] = lRobotPulse[i][0];
			pVarData[i].ulValue[1] = lRobotPulse[i][1];
			pVarData[i].ulValue[2] = lRobotPulse[i][2];
			pVarData[i].ulValue[3] = 0;//该参数无意义
			pVarData[i].ulValue[4] = 0;//该参数无意义
			pVarData[i].ulValue[5] = 0;//该参数无意义
			pVarData[i].ulValue[6] = 0;//该参数无意义
			pVarData[i].ulValue[7] = 0;//该参数无意义
		}
		SetMultiPosVar(unCount, pVarData);
		DELETE_POINTER_ARRAY(pVarData);
	}
	break;
	case ROBOT_BRAND_ESTUN:			//埃斯顿
		break;
	case ROBOT_BRAND_CRP:				//卡诺普
		break;
	default:
		break;
	}

}

void CRobotDriverAdaptor::SetMultiBasePosVar(UINT unCount, UINT unIndex, double dPosCoord[][3], int nExternalAxleType, int nPVarType)
{
	if (g_bLocalDebugMark) return;
	switch (m_eRobotBrand) {
	case ROBOT_BRAND_YASKAWA:		//安川
	{
		int nRatio = 1;
		if (nPVarType == MP_BASE_COORD) {
			nRatio = 1000;
		}
		else if (nPVarType == MP_PULSE_COORD) {
			nRatio = 1;
		}
		else {
			XUI::MesBox::PopError("错误的BP变量类型！");
			return;
		}
		T_ROBOT_POSVAR_DATA* pVarData = new T_ROBOT_POSVAR_DATA[unCount];
		for (UINT i = 0; i < unCount; i++) {
			//P变量赋值
			pVarData[i].usPorBPorExVar = MP_RESTYPE_VAR_BASE;
			pVarData[i].usIndex = unIndex + i;
			pVarData[i].usPulseOrCart = nPVarType;
			pVarData[i].usPosture = 4;
			pVarData[i].usToolNum = 1;
			pVarData[i].usUserCoordNum = 0;//该参数无意义
			ReloadBP(dPosCoord[i], nExternalAxleType);
			pVarData[i].ulValue[0] = long(dPosCoord[i][0] * nRatio);
			pVarData[i].ulValue[1] = long(dPosCoord[i][1] * nRatio);
			pVarData[i].ulValue[2] = long(dPosCoord[i][2] * nRatio);
			pVarData[i].ulValue[3] = 0;//该参数无意义
			pVarData[i].ulValue[4] = 0;//该参数无意义
			pVarData[i].ulValue[5] = 0;//该参数无意义
			pVarData[i].ulValue[6] = 0;//该参数无意义
			pVarData[i].ulValue[7] = 0;//该参数无意义
		}
		SetMultiPosVar(unCount, pVarData);
		DELETE_POINTER_ARRAY(pVarData);
	}
	break;
	case ROBOT_BRAND_ESTUN:			//埃斯顿
		break;
	case ROBOT_BRAND_CRP:				//卡诺普
		break;
	default:
		break;
	}
}

void CRobotDriverAdaptor::SetMultiPosVar(UINT unCount, UINT unIndex, long lRobotPulse[][6], UINT unToolNum, UINT unUserNo, UINT unPosture)
{
	if (g_bLocalDebugMark) return;
	if (unCount <= 0) return;
	switch (m_eRobotBrand) {
	case ROBOT_BRAND_YASKAWA:		//安川
	{
		UINT i = 0;
		if (unCount > MAXPOSVARNO)
		{
			for (; i < unCount - MAXPOSVARNO; i = i + MAXPOSVARNO)
			{
				m_pYaskawaRobotCtrl->SetMultiPosVar(MAXPOSVARNO, unIndex + i, &lRobotPulse[i], unToolNum, unUserNo, unPosture);
			}
		}
		if (i < unCount)
		{
			m_pYaskawaRobotCtrl->SetMultiPosVar(unCount - i, unIndex + i, &lRobotPulse[i], unToolNum, unUserNo, unPosture);
		}
	}
	break;
	case ROBOT_BRAND_ESTUN:			//埃斯顿
		break;
	case ROBOT_BRAND_CRP:				//卡诺普
		break;
	default:
		break;
	}
}

void CRobotDriverAdaptor::SetMultiPosVar(UINT unCount, UINT unIndex, double adPosCoord[][6], UINT unToolNum/*=1*/, UINT unUserNo/*=1*/, UINT unPosture/*=4*/)
{
	if (g_bLocalDebugMark) return;
	if (unCount <= 0) return;
	switch (m_eRobotBrand) {
	case ROBOT_BRAND_YASKAWA:		//安川
	{
		UINT i = 0;
		if (unCount > MAXPOSVARNO)
		{
			for (; i < unCount - MAXPOSVARNO; i = i + MAXPOSVARNO)
			{
				m_pYaskawaRobotCtrl->SetMultiPosVar(MAXPOSVARNO, unIndex + i, &adPosCoord[i], unToolNum, unUserNo, unPosture);
			}
		}
		if (i < unCount)
		{
			m_pYaskawaRobotCtrl->SetMultiPosVar(unCount - i, unIndex + i, &adPosCoord[i], unToolNum, unUserNo, unPosture);
		}
	}
	break;
	case ROBOT_BRAND_ESTUN:			//埃斯顿
		break;
	case ROBOT_BRAND_CRP:				//卡诺普
		break;
	default:
		break;
	}
}

void CRobotDriverAdaptor::SetMultiPosVar(UINT unCount, T_ROBOT_POSVAR_DATA* pPosVarData)
{
	if (g_bLocalDebugMark) return;
	if (unCount <= 0) return;
	switch (m_eRobotBrand) {
	case ROBOT_BRAND_YASKAWA:		//安川
	{
		UINT i = 0;
		if (unCount > MAXPOSVARNO)
		{
			for (; i < unCount - MAXPOSVARNO; i = i + MAXPOSVARNO)
			{
				m_pYaskawaRobotCtrl->SetMultiPosVar(MAXPOSVARNO, &pPosVarData[i]);
			}
		}
		if (i < unCount)
		{
			m_pYaskawaRobotCtrl->SetMultiPosVar(unCount - i, &pPosVarData[i]);
		}
	}
	break;
	case ROBOT_BRAND_ESTUN:			//埃斯顿
		break;
	case ROBOT_BRAND_CRP:				//卡诺普
		break;
	default:
		break;
	}
}

void CRobotDriverAdaptor::SetMultiPosVar(UINT unIndex, std::vector<T_ANGLE_PULSE> vtRobotPulse, UINT unToolNum /*= 1*/, UINT unUserNo /*= 1*/, UINT unPosture /*= 4*/)
{
	if (g_bLocalDebugMark) return;
	switch (m_eRobotBrand) {
	case ROBOT_BRAND_YASKAWA:		//安川
	{
		UINT unCount = vtRobotPulse.size();
		//转换
		long(*lRobotPulse)[6] = new long[unCount][6];
		for (UINT i = 0; i < unCount; i++)
		{
			lRobotPulse[i][0] = vtRobotPulse[i].nSPulse;
			lRobotPulse[i][1] = vtRobotPulse[i].nLPulse;
			lRobotPulse[i][2] = vtRobotPulse[i].nUPulse;
			lRobotPulse[i][3] = vtRobotPulse[i].nRPulse;
			lRobotPulse[i][4] = vtRobotPulse[i].nBPulse;
			lRobotPulse[i][5] = vtRobotPulse[i].nTPulse;
		}
		//赋值
		SetMultiPosVar(unCount, unIndex, lRobotPulse, unToolNum, unUserNo, unPosture);
		//清理
		delete[] lRobotPulse;
		lRobotPulse = NULL;
	}
	break;
	case ROBOT_BRAND_ESTUN:			//埃斯顿
		break;
	case ROBOT_BRAND_CRP:				//卡诺普
		break;
	default:
		break;
	}
}
//（问题：新框架无此定义 老框架未正确定义）
bool CRobotDriverAdaptor::SetMultiPosVar(UINT unIndex, std::vector<T_ROBOT_COORS> vtRobotJointCoord, T_ROBOT_MOVE_SPEED tPosMove, int config[7], UINT unToolNum)
{
	return false;
	if (g_bLocalDebugMark) return false;
	switch (m_eRobotBrand) {
	case ROBOT_BRAND_YASKAWA:		//安川
	{

	}
	break;
	case ROBOT_BRAND_ESTUN:			//埃斯顿
		break;
	case ROBOT_BRAND_CRP:				//卡诺普
		break;
	default:
		break;
	}
}
//（问题：新框架无此重载 老框架未正确定义）
bool CRobotDriverAdaptor::SetMultiPosVar(UINT unIndex, std::vector<T_ROBOT_COORS> vtRobotJointCoord, T_ROBOT_MOVE_SPEED tPosMove, CString Program_name, int config[7], UINT unToolNum)
{
	return false;
	if (g_bLocalDebugMark) return false;
	switch (m_eRobotBrand) {
	case ROBOT_BRAND_YASKAWA:		//安川
	{

	}
	break;
	case ROBOT_BRAND_ESTUN:			//埃斯顿
		break;
	case ROBOT_BRAND_CRP:				//卡诺普
		break;
	default:
		break;
	}
}

void CRobotDriverAdaptor::GetMultiPosVar(UINT unCount, UINT unIndex, double adRobotPos[][6], UINT* pToolNo /*= NULL*/, UINT* pUserNo /*= NULL*/, UINT* pPosture /*= NULL*/, UINT32 unTimeout /*= 2000*/)
{
	if (g_bLocalDebugMark) return;
	if (unCount <= 0)  return;
	switch (m_eRobotBrand) {
	case ROBOT_BRAND_YASKAWA:		//安川
	{
		UINT nToolNo = 1;
		UINT nUserNo = 1;
		UINT nPosture = 4;
		if (pToolNo == NULL) {
			pToolNo = &nToolNo;
		}
		if (pUserNo == NULL) {
			pUserNo = &nUserNo;
		}
		if (pPosture == NULL) {
			pPosture = &nPosture;
		}
		UINT i = 0;
		if (unCount > MAXPOSVARNO) {
			for (; i < unCount - MAXPOSVARNO; i = i + MAXPOSVARNO) {
				Sleep(50);
				m_pYaskawaRobotCtrl->GetMultiPosVar(MAXPOSVARNO, unIndex + i, &adRobotPos[i], pToolNo, pUserNo, pPosture, unTimeout);
			}
		}
		if (i < unCount) {
			Sleep(50);
			m_pYaskawaRobotCtrl->GetMultiPosVar(unCount - i, unIndex + i, &adRobotPos[i], pToolNo, pUserNo, pPosture, unTimeout);
		}
	}
	break;
	case ROBOT_BRAND_ESTUN:			//埃斯顿
		break;
	case ROBOT_BRAND_CRP:				//卡诺普
	{
		UINT nToolNo = 1;
		UINT nUserNo = 0;
		double date[10];
		for (UINT i = 0; i < unCount; i++) {
			memset(date, 0, sizeof(date));//将数组清空
			int result = m_pCrpRobotCtrl->Read_GP_Number_status((unIndex + i), date);
			//m_pCrpRobotCtrl->GetAppointToolPos(date, nToolNo, nUserNo); //将此GP读到的数据转换成目标工具号和用户号下
			adRobotPos[i][0] = date[0];
			adRobotPos[i][1] = date[1];
			adRobotPos[i][2] = date[2];
			adRobotPos[i][3] = date[3];
			adRobotPos[i][4] = date[4];
			adRobotPos[i][5] = date[5];
		}
	}
	break;
	default:
		return;
	}

}

void CRobotDriverAdaptor::GetMultiPosVar(UINT unCount, MP_VAR_INFO* mpVarInfo, LONG* pPosVarData, UINT32 unTimeout /*= 2000*/)
{
	if (g_bLocalDebugMark) return;
	if (unCount <= 0) return;
	switch (m_eRobotBrand) {
	case ROBOT_BRAND_YASKAWA:		//安川
	{
		UINT i = 0;
		if (unCount > MAXPOSVARNO) {
			for (; i < unCount - MAXPOSVARNO; i = i + MAXPOSVARNO) {
				m_pYaskawaRobotCtrl->GetMultiPosVar(MAXPOSVARNO, &mpVarInfo[i], &pPosVarData[i * 10], unTimeout);
			}
		}
		if (i < unCount) {
			m_pYaskawaRobotCtrl->GetMultiPosVar(unCount - i, &mpVarInfo[i], &pPosVarData[i * 10], unTimeout);
		}
	}
	break;
	case ROBOT_BRAND_ESTUN:			//埃斯顿
		break;
	case ROBOT_BRAND_CRP:				//卡诺普
	{
		UINT nToolNo = 1;
		UINT nUserNo = 0;

		int itemNum = 0;
		for (UINT i = 0; i < unCount; i++) {
			double date[10] = { 0 };
			int result = m_pCrpRobotCtrl->Read_GP_Number_status((mpVarInfo[i].usIndex), date);
			//m_pCrpRobotCtrl->GetAppointToolPos(date, nToolNo, nUserNo); //将此GP读到的数据转换成目标工具号和用户号下
			for (int j = 0; j < 6; j++) {
				pPosVarData[itemNum + j] = date[j]; // 直角坐标xyz*1000 RxRyRz*10000  角度转脉冲 ！！！！ 2
			}
			itemNum += 6;
		}
	}
	break;
	default:
		return;
	}
}

void CRobotDriverAdaptor::SetMultiVar(UINT unCount, unsigned unIndex, LONG lRobotValue[], UINT unType /*= MP_RESTYPE_VAR_I*/)
{
	if (g_bLocalDebugMark) return;
	if (unCount <= 0) return;
	switch (m_eRobotBrand) {
	case ROBOT_BRAND_YASKAWA:		//安川
		m_pYaskawaRobotCtrl->SetMultiVar(unCount, unIndex, lRobotValue, unType);
		break;
	case ROBOT_BRAND_ESTUN:			//埃斯顿
		break;
	case ROBOT_BRAND_CRP:				//卡诺普
	{
		if (unType == MP_RESTYPE_VAR_B) {}
		else if (unType == MP_RESTYPE_VAR_I) {
			for (UINT i = 0; i < unCount; i++) {
				m_pCrpRobotCtrl->Write_GI_Number_status(unIndex + i, lRobotValue[i]);
			}
		}
		else if (unType == MP_RESTYPE_VAR_D) {}
		else if (unType == MP_RESTYPE_VAR_R) {}
	}
	break;
	default:
		break;
	}
}

void CRobotDriverAdaptor::SetMultiVar(UINT unCount, MP_VAR_DATA* pVarData)
{
	if (g_bLocalDebugMark) return;
	if (unCount <= 0) return;
	switch (m_eRobotBrand) {
	case ROBOT_BRAND_YASKAWA:		//安川
		m_pYaskawaRobotCtrl->SetMultiVar(unCount, pVarData);
		break;
	case ROBOT_BRAND_ESTUN:			//埃斯顿
		break;
	case ROBOT_BRAND_CRP:				//卡诺普
	{
		for (UINT i = 0; i < unCount; i++) {
			switch (pVarData[i].usType) {
			case MP_RESTYPE_VAR_B: {};
			case MP_RESTYPE_VAR_I: {
				m_pCrpRobotCtrl->Write_GI_Number_status(pVarData[i].usIndex, pVarData[i].ulValue);
				break;
			};
			case MP_RESTYPE_VAR_D: {};
			case MP_RESTYPE_VAR_R: {};
			}
		}
	}
	break;
	default:
		break;
	}
}

//（问题：老框架无此重载	）（2023/12/21安川独占）
void CRobotDriverAdaptor::SetMultiBasePosVar(UINT unIndex, std::vector<T_ANGLE_PULSE> vtRobotPulse, int nExternalAxleType)
{
	if (g_bLocalDebugMark) return;
	switch (m_eRobotBrand) {
	case ROBOT_BRAND_YASKAWA:		//安川
		break;
	case ROBOT_BRAND_ESTUN:			//埃斯顿
		break;
	case ROBOT_BRAND_CRP:				//卡诺普
		break;
	default:
		break;
	}
	UINT unCount = vtRobotPulse.size();
	long(*lRobotPulse)[3] = new long[unCount][3];
	for (UINT i = 0; i < unCount; i++)
	{
		lRobotPulse[i][0] = vtRobotPulse[i].lBXPulse;
		lRobotPulse[i][1] = vtRobotPulse[i].lBYPulse;
		lRobotPulse[i][2] = vtRobotPulse[i].lBZPulse;
	}
	SetMultiBasePosVar(unCount, unIndex, lRobotPulse, nExternalAxleType);
	delete[] lRobotPulse;
}
//（问题：老框架无此定义	）（2023/12/21安川独占）
void CRobotDriverAdaptor::SetBasePosVar(int nIndex, double adBasePosVar[3], int nCoordType)
{
	if (g_bLocalDebugMark) return;
	switch (m_eRobotBrand) {
	case ROBOT_BRAND_YASKAWA:		//安川
		m_pYaskawaRobotCtrl->SetBasePosVar(nIndex, adBasePosVar, nCoordType);
		break;
	case ROBOT_BRAND_ESTUN:			//埃斯顿
		break;
	case ROBOT_BRAND_CRP:				//卡诺普
		break;
	default:
		break;
	}
}
//（问题：老框架无此定义	）（2023/12/21安川独占）
int CRobotDriverAdaptor::GetBasePosVar(long lPvarIndex, double* array, UINT32 unTimeout /*= 2000*/)
{
	if (g_bLocalDebugMark) return 0;
	switch (m_eRobotBrand) {
	case ROBOT_BRAND_YASKAWA:		//安川
		return m_pYaskawaRobotCtrl->GetBasePosVar(lPvarIndex, array, unTimeout);
		break;
	case ROBOT_BRAND_ESTUN:			//埃斯顿
		break;
	case ROBOT_BRAND_CRP:				//卡诺普
		break;
	default:
		return -1;
	}
	return FALSE;
}

void CRobotDriverAdaptor::SetPosVar(int nIndex, double adPosVar[6], int nPVarType)
{
	if (g_bLocalDebugMark) return;
	long long stratr = XI_clock();
	while (true == m_bInsterPVal)  // 等待一定时间间隔再发送 
	{
		if ((XI_clock() - stratr) > 400) {
			break;
		}
		DoEvent();
	}

	switch (m_eRobotBrand) {
	case ROBOT_BRAND_YASKAWA:		//安川
	{
		m_bInsterPVal = true;
		m_pYaskawaRobotCtrl->SetPosVar(nIndex, adPosVar, nPVarType);
		m_bInsterPVal = false;
	}
	break;
	case ROBOT_BRAND_ESTUN:			//埃斯顿
		break;
	case ROBOT_BRAND_CRP:				//卡诺普
	{
		double pos[10] = { 0 };		//工具号1，用户号0
		m_pCrpRobotCtrl->GetAppointToolPos(pos, 1, 0);
		double axios[6] = { 0.0 };
		double FlangePos[6] = { 0.0 };
		for (int i = 0; i < 6; i++) {
			axios[i] = adPosVar[i];
		}
		m_pCrpRobotCtrl->Tool_PosToFlange(axios, 1, 0, FlangePos);
		for (int i = 0; i < 6; i++) {
			pos[i] = FlangePos[i];
		}
		int result = m_pCrpRobotCtrl->Write_GP_Number_single(nIndex, pos);
	}
	break;
	default:
		return;
	}
}
void CRobotDriverAdaptor::SetPosVar(int nIndex, long alPosVar[6], int nPVarType)
{
	double adPosVar[6];
	adPosVar[0] = alPosVar[0];
	adPosVar[1] = alPosVar[1];
	adPosVar[2] = alPosVar[2];
	adPosVar[3] = alPosVar[3];
	adPosVar[4] = alPosVar[4];
	adPosVar[5] = alPosVar[5];
	SetPosVar(nIndex, adPosVar, nPVarType);
}
//（问题：老框架无此重载	）（2023/12/21安川独占）
void CRobotDriverAdaptor::SetPosVar(int nIndex, long lPosVar[6])
{
	if (g_bLocalDebugMark) return;
	switch (m_eRobotBrand) {
	case ROBOT_BRAND_YASKAWA:		//安川
	{
		double* adPosVar;
		adPosVar = reinterpret_cast<double*>(lPosVar);
		m_pYaskawaRobotCtrl->SetPosVar(nIndex, adPosVar, 1);
		return;
	}
	break;
	case ROBOT_BRAND_ESTUN:			//埃斯顿
		break;
	case ROBOT_BRAND_CRP:				//卡诺普
		break;
	default:
		return;
	}

}

void CRobotDriverAdaptor::SetPosVar(int nIndex, T_ANGLE_PULSE tRobotPulse)
{
	if (g_bLocalDebugMark) return;
	switch (m_eRobotBrand) {
	case ROBOT_BRAND_YASKAWA:		//安川
	{
		long adPosVar[6] = { tRobotPulse.nSPulse,tRobotPulse.nLPulse, tRobotPulse.nUPulse,
							   tRobotPulse.nRPulse,tRobotPulse.nBPulse, tRobotPulse.nTPulse };
		SetPosVar(nIndex, adPosVar, 1);
	}
	break;
	case ROBOT_BRAND_ESTUN:			//埃斯顿
		break;
	case ROBOT_BRAND_CRP:				//卡诺普
	{
		long adPosVar[6] = { tRobotPulse.nSPulse,tRobotPulse.nLPulse, tRobotPulse.nUPulse,
			tRobotPulse.nRPulse,tRobotPulse.nBPulse, tRobotPulse.nTPulse };
		T_ROBOT_COORS tCurrentRobotCoors, tToolCoors;
		tToolCoors.dX = m_pCrpRobotCtrl->ToolArray[1][0];
		tToolCoors.dY = m_pCrpRobotCtrl->ToolArray[1][1];
		tToolCoors.dZ = m_pCrpRobotCtrl->ToolArray[1][2];
		tToolCoors.dRX = m_pCrpRobotCtrl->ToolArray[1][3];
		tToolCoors.dRY = m_pCrpRobotCtrl->ToolArray[1][4];
		tToolCoors.dRZ = m_pCrpRobotCtrl->ToolArray[1][5];
		tToolCoors.dBX = 0;
		tToolCoors.dBY = 0;
		tToolCoors.dBZ = 0;

		RobotKinematics(tRobotPulse, tToolCoors, tCurrentRobotCoors);// 1. 将输入的关节坐标转换成工具坐标
		double axios[6] = { 0.0 };
		double FlangePos[6] = { 0.0 };
		axios[0] = tCurrentRobotCoors.dX;
		axios[1] = tCurrentRobotCoors.dY;
		axios[2] = tCurrentRobotCoors.dZ;
		axios[3] = tCurrentRobotCoors.dRX;
		axios[4] = tCurrentRobotCoors.dRY;
		axios[5] = tCurrentRobotCoors.dRZ;
		m_pCrpRobotCtrl->Tool_PosToFlange(axios, 1, 0, FlangePos); //2.将1.得到的工具坐标转换成对应的法兰坐标

		double pos[10] = { 0 };
		m_pCrpRobotCtrl->GetAppointToolPos(pos, 1, 0); //获取当前完整数据包（当前工具坐标 + 外部轴 + 工具信息）
		for (int i = 0; i < 6; i++) {
			pos[i] = FlangePos[i];  // 3. 将2.得到的法兰坐标填到上面获得的数据包里
		}
		int result = m_pCrpRobotCtrl->Write_GP_Number_single(nIndex, pos);
	}
	break;
	default:
		break;
	}
}

void CRobotDriverAdaptor::SetPosVar(int nIndex, T_ROBOT_COORS tRobotCoors)
{
	if (g_bLocalDebugMark) return;
	switch (m_eRobotBrand) {
	case ROBOT_BRAND_YASKAWA:		//安川
	{
		double adPosVar[6] = { tRobotCoors.dX, tRobotCoors.dY, tRobotCoors.dZ, tRobotCoors.dRX, tRobotCoors.dRY, tRobotCoors.dRZ };
		SetPosVar(nIndex, adPosVar, 0);
	}
	break;
	case ROBOT_BRAND_ESTUN:			//埃斯顿
		break;
	case ROBOT_BRAND_CRP:				//卡诺普
	{
		double Tpos[6] = { tRobotCoors.dX,  tRobotCoors.dY,  tRobotCoors.dZ,
			tRobotCoors.dRX, tRobotCoors.dRY, tRobotCoors.dRZ };
		double Fpos[6];
		m_pCrpRobotCtrl->Tool_PosToFlange(Tpos, 1, 0, Fpos);
		double pos[10] = { Fpos[0],Fpos[1],Fpos[2],Fpos[3],Fpos[4],Fpos[5],
			tRobotCoors.dBX, tRobotCoors.dBY, 1.0, 0.0 };		//工具号1，用户号0
		int result = m_pCrpRobotCtrl->Write_GP_Number_single(nIndex, pos);
	}
	break;
	default:
		return;
	}
}
//（问题：老框架无此定义	）（2023/12/21安川独占）
int CRobotDriverAdaptor::GetPosVar(long lPvarIndex, double array[6])
{
	if (g_bLocalDebugMark) return 0;
	switch (m_eRobotBrand) {
	case ROBOT_BRAND_YASKAWA:		//安川
		return (m_pYaskawaRobotCtrl->GetPosVar(lPvarIndex, array));
		break;
	case ROBOT_BRAND_ESTUN:			//埃斯顿
		break;
	case ROBOT_BRAND_CRP:				//卡诺普
		break;
	default:
		break;
	}
	return FALSE;
}
int CRobotDriverAdaptor::GetIntVar(int nIndex)
{
	if (g_bLocalDebugMark) return 0;
	switch (m_eRobotBrand) {
	case ROBOT_BRAND_YASKAWA:		//安川
		return m_pYaskawaRobotCtrl->GetIntVar(nIndex);
		break;
	case ROBOT_BRAND_ESTUN:			//埃斯顿
		break;
	case ROBOT_BRAND_CRP:				//卡诺普
	{
		int result = m_pCrpRobotCtrl->Read_GI_Number_status(nIndex);
		if (result < 0) {
			XiMessageBox("读取失败！");
			return -1;
		}
		else {
			result += 1;
			return result;
		}
	}
	break;
	default:
		return -1;
	}
	return FALSE;
}

void CRobotDriverAdaptor::SetIntVar(int nIndex, int nValue)
{
	if (g_bLocalDebugMark) return;
	switch (m_eRobotBrand) {
	case ROBOT_BRAND_YASKAWA:		//安川
		m_pYaskawaRobotCtrl->SetIntVar(nIndex, nValue);
		break;
	case ROBOT_BRAND_ESTUN:			//埃斯顿
		break;
	case ROBOT_BRAND_CRP:				//卡诺普
	{
		int result = m_pCrpRobotCtrl->Write_GI_Number_status(nIndex, nValue);
		if (result <= 0) {
			XiMessageBox("写入失败!");
		}
		else {
			XiMessageBox("写入成功!");
		}
	}
	break;
	default:
		break;
	}
}

void CRobotDriverAdaptor::SetIntVar_H(int nIndex, int nValue)
{
	if (g_bLocalDebugMark) return;
	switch (m_eRobotBrand) {
	case ROBOT_BRAND_YASKAWA:		//安川
	{
		MP_USR_VAR_INFO tVarInfoTrack[5];
		tVarInfoTrack[0].var_type = MP_VAR_I;
		tVarInfoTrack[0].var_no = nIndex;
		tVarInfoTrack[0].val.i = nValue;
		SetMultiVar_H(1, tVarInfoTrack);
	}
	break;
	case ROBOT_BRAND_ESTUN:			//埃斯顿
		break;
	case ROBOT_BRAND_CRP:				//卡诺普
	{
		int result = m_pCrpRobotCtrl->Write_GI_Number_status(nIndex, nValue);
		if (result <= 0) {
			XiMessageBox("写入失败!");
		}
		else {
			XiMessageBox("写入成功!");
		}
	}
	break;
	default:
		return;
	}
}
int CRobotDriverAdaptor::ConvAxesToCartPos(long alPulse[6], UINT* unFig_ctrl, double adCoord[6], int nGrpNo /*= 0*/, UINT unTool_no /*= 1*/)
{
	switch (m_eRobotBrand) {
	case ROBOT_BRAND_YASKAWA:		//安川
		return m_pYaskawaRobotCtrl->ConvAxesToCartPos(alPulse, unFig_ctrl, adCoord, nGrpNo, unTool_no);
		break;
	case ROBOT_BRAND_ESTUN:			//埃斯顿
		break;
	case ROBOT_BRAND_CRP:				//卡诺普
		break;
	default:
		break;
	}
	return -1;
}

int CRobotDriverAdaptor::ConvCartToAxesPos(double adCoord[6], long alPulse[6], UINT unFig_ctrl /*= 4*/, int nGrpNo /*= 0*/, UINT unTool_no /*= 1*/, UINT unKinema_type /*= 0*/)
{
	switch (m_eRobotBrand) {
	case ROBOT_BRAND_YASKAWA:		//安川
		return m_pYaskawaRobotCtrl->ConvCartToAxesPos(adCoord, alPulse, unFig_ctrl, nGrpNo, unTool_no, unKinema_type);
		break;
	case ROBOT_BRAND_ESTUN:			//埃斯顿
		break;
	case ROBOT_BRAND_CRP:				//卡诺普
		break;
	default:
		break;
	}
	return -1;
}

BOOL CRobotDriverAdaptor::GetToolData(UINT unToolNo, double adRobotToolData[6], UINT32 unTimeout /*= 2000*/)
{
	switch (m_eRobotBrand) {
	case ROBOT_BRAND_YASKAWA:		//安川
		return m_pYaskawaRobotCtrl->GetToolData(unToolNo, adRobotToolData, unTimeout);
		break;
	case ROBOT_BRAND_ESTUN:			//埃斯顿
		break;
	case ROBOT_BRAND_CRP:				//卡诺普
		adRobotToolData[0] = m_pCrpRobotCtrl->ToolArray[1][0];
		adRobotToolData[1] = m_pCrpRobotCtrl->ToolArray[1][1];
		adRobotToolData[2] = m_pCrpRobotCtrl->ToolArray[1][2];
		adRobotToolData[3] = m_pCrpRobotCtrl->ToolArray[1][3];
		adRobotToolData[4] = m_pCrpRobotCtrl->ToolArray[1][4];
		adRobotToolData[5] = m_pCrpRobotCtrl->ToolArray[1][5];
		break;
	default:
		break;
	}
	return TRUE;
}

BOOL CRobotDriverAdaptor::GetAlarmCode(int& nErrorNo, int& ErrorData, int& AlarmNum, MP_ALARM_DATA* pData, UINT32 unTimeout /*= 2000*/)
{
	switch (m_eRobotBrand) {
	case ROBOT_BRAND_YASKAWA:		//安川
		return m_pYaskawaRobotCtrl->GetAlarmCode(nErrorNo, ErrorData, AlarmNum, pData, unTimeout);
		break;
	case ROBOT_BRAND_ESTUN:			//埃斯顿
		break;
	case ROBOT_BRAND_CRP:				//卡诺普
		break;
	default:
		break;
	}
	return FALSE;
}

CString CRobotDriverAdaptor::GetWarningMessage()
{
	switch (m_eRobotBrand) {
	case ROBOT_BRAND_YASKAWA:		//安川
		return m_pYaskawaRobotCtrl->GetWarningMessage();
		break;
	case ROBOT_BRAND_ESTUN:			//埃斯顿
		break;
	case ROBOT_BRAND_CRP:				//卡诺普
		break;
	default:
		break;
	}
	return "";
	//	return m_pYaskawaRobotCtrl->GetWarningMessage();

}

//（问题：老框架无此重载）
void CRobotDriverAdaptor::CallJob(CString sJobName, int nExternalType)
{
	//if (nExternalType > 0)
	//{
	//	sJobName += "-BP";
	//} 
	m_pYaskawaRobotCtrl->CallJob(sJobName.GetBuffer());
}

int CRobotDriverAdaptor::CallJob(char JobName[24])
{
	switch (m_eRobotBrand) {
	case ROBOT_BRAND_YASKAWA:		//安川
		m_pYaskawaRobotCtrl->CallJob(JobName);
		break;
	case ROBOT_BRAND_ESTUN:			//埃斯顿
		break;
	case ROBOT_BRAND_CRP:				//卡诺普
	{
		std::map<std::string, int> XiJobCase;
		string str = JobName;
		int JobIndex = XiJobCase.find(str)->second; //根据输入的程序名转换成卡诺普的case信息
		XiJobCase.insert(pair<string, int>("Move_L", 1));
		XiJobCase.insert(pair<string, int>("Move_J_A", 2));
		XiJobCase.insert(pair<string, int>("MultiPos", 3));
		XiJobCase.insert(pair<string, int>("ChangeTool", 4));
		XiJobCase.insert(pair<string, int>("ChangeUser", 5));

		m_pCrpRobotCtrl->XiRobot_CallJob(JobIndex);
		break;
	}
	default:
		m_pYaskawaRobotCtrl->CallJob(JobName);
	}
	return 0;
}

int CRobotDriverAdaptor::CallJob(CString sJobName)
{
	switch (m_eRobotBrand) {
	case ROBOT_BRAND_YASKAWA:		//安川
	{
		char JobName[24];
		int nLength = sJobName.GetLength();
		int i = 0;
		for (; i < nLength; i++) {
			JobName[i] = sJobName[i];
		}
		JobName[i] = '\0';
		m_pYaskawaRobotCtrl->CallJob(JobName);
		break;
	}
	case ROBOT_BRAND_ESTUN:			//埃斯顿
		break;
	case ROBOT_BRAND_CRP:				//卡诺普
	{
		/*将XiRobot所有分支都添加到XiJobCase中，再根据输入的JobName来匹配分支*/
		std::map<std::string, int> XiJobCase;
		XiJobCase.insert(pair<string, int>("Move_L", 1));
		XiJobCase.insert(pair<string, int>("Move_J_A", 2));
		XiJobCase.insert(pair<string, int>("MultiPos", 3));
		XiJobCase.insert(pair<string, int>("ChangeTool", 4));
		XiJobCase.insert(pair<string, int>("ChangeUser", 5));
		int JobIndex = XiJobCase.find(sJobName.GetBuffer())->second;
		m_pCrpRobotCtrl->XiRobot_CallJob(JobIndex);
		break;
	}
	default:
		char JobName[24];
		int nLength = sJobName.GetLength();
		int i = 0;
		for (; i < nLength; i++) {
			JobName[i] = sJobName[i];
		}
		JobName[i] = '\0';
		m_pYaskawaRobotCtrl->CallJob(JobName);
		break;
	}
	return 0;
}

void CRobotDriverAdaptor::WorldCheckRobotDone(int nDelayTime)
{
}

bool CRobotDriverAdaptor::CheckIsReadyRun()
{
	return false;
}

void CRobotDriverAdaptor::SetRobotToolNo(int nToolNo)
{
	switch (m_eRobotBrand) {
	case ROBOT_BRAND_YASKAWA:		//安川
		m_pYaskawaRobotCtrl->SetRobotToolNo(nToolNo);
		break;
	case ROBOT_BRAND_ESTUN:			//埃斯顿
		break;
	case ROBOT_BRAND_CRP:				//卡诺普
		m_pCrpRobotCtrl->ChangeToolNum(nToolNo); // 更改工具号
		break;
	default:
		break;
	}
}

CXiRobotCtrl* CRobotDriverAdaptor::GetXiRobotCtrl()
{
	return m_pYaskawaRobotCtrl;
}

void CRobotMove::Clear()
{
	m_vRobotMoveInfo.clear();
	m_nTrackNum = 0;
}

bool CRobotMove::InsertCoor(T_ANGLE_PULSE tPulse, T_ROBOT_MOVE_SPEED tSpeed, int nExternalAxleType, int nMoveType, UINT unToolNum, UINT unUserNo, UINT unPosture)
{
	if (m_nTrackNum >= MAXMOVENUM)
	{
		return false;
	}
	T_ROBOT_MOVE_INFO tRtnVal;
	tRtnVal.nMoveDevice = 0;
	tRtnVal.nTrackNo = m_nTrackNum;
	tRtnVal.tCoord.var_type = MP_VAR_P;
	tRtnVal.tCoord.var_no = 20 + m_nTrackNum;
	tRtnVal.tCoord.val.p.dtype = MP_PULSE_DTYPE;
	tRtnVal.tCoord.val.p.uf_no = unUserNo;
	tRtnVal.tCoord.val.p.tool_no = unToolNum;
	tRtnVal.tCoord.val.p.fig_ctrl = unPosture;
	tRtnVal.tCoord.val.p.data[0] = tPulse.nSPulse;
	tRtnVal.tCoord.val.p.data[1] = tPulse.nLPulse;
	tRtnVal.tCoord.val.p.data[2] = tPulse.nUPulse;
	tRtnVal.tCoord.val.p.data[3] = tPulse.nRPulse;
	tRtnVal.tCoord.val.p.data[4] = tPulse.nBPulse;
	tRtnVal.tCoord.val.p.data[5] = tPulse.nTPulse;
	tRtnVal.tCoord.val.p.data[6] = 0;
	tRtnVal.tCoord.val.p.data[7] = 0;
	tRtnVal.nMoveType = nMoveType;
	tRtnVal.tSpeed = tSpeed;
	m_vRobotMoveInfo.push_back(tRtnVal);

	if (nExternalAxleType > 0)
	{
		long lBasePulse[3];
		lBasePulse[0] = tPulse.lBXPulse;
		lBasePulse[1] = tPulse.lBYPulse;
		lBasePulse[2] = tPulse.lBZPulse;
		ReloadBP(lBasePulse, nExternalAxleType);
		T_ROBOT_MOVE_INFO tRtnVal;
		tRtnVal.nMoveDevice = 1;
		tRtnVal.nTrackNo = m_nTrackNum;
		tRtnVal.tCoord.var_type = MP_VAR_BP;
		tRtnVal.tCoord.var_no = 20 + m_nTrackNum;
		tRtnVal.tCoord.val.bp.dtype = MP_PULSE_DTYPE;
		tRtnVal.tCoord.val.bp.uf_no = unUserNo;
		tRtnVal.tCoord.val.bp.tool_no = unToolNum;
		tRtnVal.tCoord.val.bp.fig_ctrl = unPosture;
		tRtnVal.tCoord.val.bp.data[0] = lBasePulse[0];
		tRtnVal.tCoord.val.bp.data[1] = lBasePulse[1];
		tRtnVal.tCoord.val.bp.data[2] = lBasePulse[2];
		tRtnVal.tCoord.val.bp.data[3] = 0;
		tRtnVal.tCoord.val.bp.data[4] = 0;
		tRtnVal.tCoord.val.bp.data[5] = 0;
		tRtnVal.tCoord.val.bp.data[6] = 0;
		tRtnVal.tCoord.val.bp.data[7] = 0;
		tRtnVal.nMoveType = nMoveType;
		tRtnVal.tSpeed = tSpeed;
		m_vRobotMoveInfo.push_back(tRtnVal);
	}
	m_nTrackNum++;
	return true;
}

bool CRobotMove::InsertCoor(T_ROBOT_COORS tCoord, T_ROBOT_MOVE_SPEED tSpeed, int nExternalAxleType, int nMoveType, UINT unToolNum, UINT unUserNo, UINT unPosture)
{
	if (m_nTrackNum >= MAXMOVENUM)
	{
		return false;
	}
	T_ROBOT_MOVE_INFO tRtnVal;
	tRtnVal.nMoveDevice = 0;
	tRtnVal.nTrackNo = m_nTrackNum;
	tRtnVal.tCoord.var_type = MP_VAR_P;
	tRtnVal.tCoord.var_no = 20 + m_nTrackNum;
	tRtnVal.tCoord.val.p.dtype = MP_ROBO_DTYPE;
	tRtnVal.tCoord.val.p.uf_no = unUserNo;
	tRtnVal.tCoord.val.p.tool_no = unToolNum;
	tRtnVal.tCoord.val.p.fig_ctrl = unPosture;
	tRtnVal.tCoord.val.p.data[0] = long(tCoord.dX * 1000);
	tRtnVal.tCoord.val.p.data[1] = long(tCoord.dY * 1000);
	tRtnVal.tCoord.val.p.data[2] = long(tCoord.dZ * 1000);
	tRtnVal.tCoord.val.p.data[3] = long(tCoord.dRX * 10000);
	tRtnVal.tCoord.val.p.data[4] = long(tCoord.dRY * 10000);
	tRtnVal.tCoord.val.p.data[5] = long(tCoord.dRZ * 10000);
	tRtnVal.tCoord.val.p.data[6] = 0;
	tRtnVal.tCoord.val.p.data[7] = 0;
	tRtnVal.nMoveType = nMoveType;
	tRtnVal.tSpeed = tSpeed;
	m_vRobotMoveInfo.push_back(tRtnVal);

	if (nExternalAxleType > 0)
	{
		double dBasePos[3];
		dBasePos[0] = tCoord.dBX;
		dBasePos[1] = tCoord.dBY;
		dBasePos[2] = tCoord.dBZ;
		ReloadBP(dBasePos, nExternalAxleType);
		T_ROBOT_MOVE_INFO tRtnVal;
		tRtnVal.nMoveDevice = 1;
		tRtnVal.nTrackNo = m_nTrackNum;
		tRtnVal.tCoord.var_type = MP_VAR_BP;
		tRtnVal.tCoord.var_no = 20 + m_nTrackNum;
		tRtnVal.tCoord.val.bp.dtype = MP_BASE_DTYPE;
		tRtnVal.tCoord.val.bp.uf_no = unUserNo;
		tRtnVal.tCoord.val.bp.tool_no = unToolNum;
		tRtnVal.tCoord.val.bp.fig_ctrl = unPosture;
		tRtnVal.tCoord.val.bp.data[0] = long(dBasePos[0] * 1000);
		tRtnVal.tCoord.val.bp.data[1] = long(dBasePos[1] * 1000);
		tRtnVal.tCoord.val.bp.data[2] = long(dBasePos[2] * 1000);
		tRtnVal.tCoord.val.bp.data[3] = 0;
		tRtnVal.tCoord.val.bp.data[4] = 0;
		tRtnVal.tCoord.val.bp.data[5] = 0;
		tRtnVal.tCoord.val.bp.data[6] = 0;
		tRtnVal.tCoord.val.bp.data[7] = 0;
		tRtnVal.nMoveType = nMoveType;
		tRtnVal.tSpeed = tSpeed;
		m_vRobotMoveInfo.push_back(tRtnVal);
	}
	m_nTrackNum++;
	return true;
}

bool CRobotMove::DeleteTrack(int nNum)
{
	if (nNum >= m_nTrackNum || nNum < 0)
	{
		return false;
	}
	if (nNum == 0)
	{
		return true;
	}
	for (size_t j = 0; j < nNum; j++)
	{
		for (size_t i = 0; i < m_vRobotMoveInfo.size(); i++)
		{
			if (j == m_vRobotMoveInfo[i].nTrackNo)
			{
				m_vRobotMoveInfo.erase(m_vRobotMoveInfo.begin() + i, m_vRobotMoveInfo.begin() + i + 1);
				i--;
			}
		}
	}
	for (size_t i = 0; i < m_vRobotMoveInfo.size(); i++)
	{
		m_vRobotMoveInfo[i].nTrackNo -= nNum;
		m_vRobotMoveInfo[i].tCoord.var_no -= nNum;
	}
	m_nTrackNum -= nNum;
	return true;
}

std::vector<T_ROBOT_MOVE_INFO> CRobotMove::GetTrack()
{
	return m_vRobotMoveInfo;
}

int CRobotMove::GetTrackNum()
{
	return m_nTrackNum;
}

bool CRobotDriverAdaptor::ContiMoveByJob(std::vector<T_ANGLE_PULSE> vtPulseMove, T_ROBOT_MOVE_SPEED tPulseMove, bool bPulseOrder, CString strJobName)
{
	switch (m_eRobotBrand)
	{
	case ROBOT_BRAND_YASKAWA:
	{
		int nSize = vtPulseMove.size();
		//阈值警告
		if (vtPulseMove.size() > 15) {
			XiMessageBox("Error:连续运动分段过多！");
			return false;
		}
		if (g_bLocalDebugMark) {
			return true;
		}
		//重新排序
		std::vector<T_ANGLE_PULSE> vtRealPulses;
		if (!bPulseOrder)//与JOB顺序相反需要倒一下
		{
			for (int i = vtPulseMove.size() - 1; i >= 0; i--) {
				vtRealPulses.push_back(vtPulseMove[i]);
			}
		}
		else {
			for (int i = 0; i < vtPulseMove.size(); i++) {
				vtRealPulses.push_back(vtPulseMove[i]);
			}
		}

		//赋值
		if ((((m_nExternalAxleType >> 3) & 0x01) == 1) ||
			(((m_nExternalAxleType >> 4) & 0x01) == 1) ||
			(((m_nExternalAxleType >> 5) & 0x01) == 1)) {
			long lRealBasePulse[10][3];
			for (int i = 0; i < vtRealPulses.size(); i++) {
				lRealBasePulse[i][0] = vtRealPulses[i].lBXPulse;
				lRealBasePulse[i][1] = vtRealPulses[i].lBYPulse;
				lRealBasePulse[i][2] = vtRealPulses[i].lBZPulse;
			}
			SetMultiBasePosVar(vtRealPulses.size(), 1, lRealBasePulse, m_nExternalAxleType);
		}
		SetMultiPosVar(1, vtRealPulses);
		if (tPulseMove.dSpeed < 1) {
			tPulseMove.dSpeed = 1;
		}
		else if (tPulseMove.dSpeed > 20000) {
			tPulseMove.dSpeed = 20000;
		}
		if (tPulseMove.dACC < 20) {
			tPulseMove.dACC = 20;
		}
		else if (tPulseMove.dACC > 100) {
			tPulseMove.dACC = 100;
		}
		if (tPulseMove.dDEC < 20) {
			tPulseMove.dDEC = 20;
		}
		else if (tPulseMove.dDEC > 100) {
			tPulseMove.dDEC = 100;
		}
		long lIntVar[5];
		lIntVar[0] = tPulseMove.dSpeed;
		lIntVar[1] = tPulseMove.dACC;
		lIntVar[2] = tPulseMove.dDEC;
		lIntVar[3] = 0;
		lIntVar[4] = vtRealPulses.size();
		SetMultiVar(5, 5, lIntVar);

		//运动
		if ((((m_nExternalAxleType >> 3) & 0x01) == 1) ||
			(((m_nExternalAxleType >> 4) & 0x01) == 1) ||
			(((m_nExternalAxleType >> 5) & 0x01) == 1)) {
			CallJob(strJobName + "-BP");
		}
		else {
			CallJob(strJobName);
		}
		return true;
	}
	break;
	case ROBOT_BRAND_ESTUN:
		return false;
		break;
	case ROBOT_BRAND_CRP:
	{
		// vtPulseMove若干个脉冲关节坐标 依次运动 仅机器人 (需要转换为角度关节坐标)
		// tPulseMove 运动 速度 加速度 减速度
		// bPulseOrder ：true正常顺序运动 false按vtPulseMove倒序运动
		// strJobName MOVL或MOVJ
		int pointNum = vtPulseMove.size();
		string changeTool_str;
		string changeUser_str;
		string end;
		string mStrat, mEnd;
		string MoveL_str;
		int  lineNum = 1;
		double pos[10]; //当前待处理的点，数组形式
		T_ANGLE_PULSE tmp; //容器中当前待处理的点

		m_pCrpRobotCtrl->GetCurrPos(pos);
		m_pCrpRobotCtrl->MakeMout(500, true, lineNum++, &mStrat);
		m_pCrpRobotCtrl->MakeChangeTool(1, lineNum++, changeTool_str);
		m_pCrpRobotCtrl->MakeChangeUse(0, lineNum++, changeUser_str);
		if (bPulseOrder) { //正常顺序运动 
			// 只有MOVJ关节插补
			std::ofstream RecordFile("Move_J_A"); //用记事本的方式记录下来job程序
			RecordFile << mStrat << "\n";
			RecordFile << changeTool_str << "\n";
			RecordFile << changeUser_str << "\n";
			int i = 0;
			for (; i < pointNum; i++) {
				tmp = vtPulseMove[i];
				pos[0] = tmp.nSPulse * m_tAxisUnit.dSPulse;
				pos[1] = tmp.nLPulse * m_tAxisUnit.dLPulse;
				pos[2] = tmp.nUPulse * m_tAxisUnit.dUPulse;
				pos[3] = tmp.nRPulse * m_tAxisUnit.dRPulse;
				pos[4] = tmp.nBPulse * m_tAxisUnit.dBPulse;
				pos[5] = tmp.nTPulse * m_tAxisUnit.dTPulse;
				m_pCrpRobotCtrl->MakeMoveJ_Axis(pos, tPulseMove.dSpeed / 100, 1, 0, tPulseMove.dACC, tPulseMove.dDEC, 0, lineNum++, &MoveL_str);
				RecordFile << MoveL_str << "\n";
			}
			m_pCrpRobotCtrl->MakeMout(500, false, lineNum++, &mEnd);
			m_pCrpRobotCtrl->programerEnd(lineNum++, &end);
			RecordFile << mEnd << "\n";
			RecordFile << end << "\n";
			RecordFile.close();

			if (!m_pCrpRobotCtrl->Obj_FTP_cmd->FTP_if_load) return false;
			bool result = m_pCrpRobotCtrl->Obj_FTP_cmd->FTP_SendFile("Move_J_A");
			if (!result) return false;
			m_pCrpRobotCtrl->XiRobot_CallJob(2);
			return true;
		}
		else { //倒序运动 
			// 只有MOVJ关节插补
			std::ofstream RecordFile("Move_J_A"); //用记事本的方式记录下来job程序
			RecordFile << mStrat << "\n";
			RecordFile << changeTool_str << "\n";
			RecordFile << changeUser_str << "\n";
			int i = pointNum - 1;
			for (; i >= 0; i--) {
				tmp = vtPulseMove[i];
				pos[0] = tmp.nSPulse * m_tAxisUnit.dSPulse;
				pos[1] = tmp.nLPulse * m_tAxisUnit.dLPulse;
				pos[2] = tmp.nUPulse * m_tAxisUnit.dUPulse;
				pos[3] = tmp.nRPulse * m_tAxisUnit.dRPulse;
				pos[4] = tmp.nBPulse * m_tAxisUnit.dBPulse;
				pos[5] = tmp.nTPulse * m_tAxisUnit.dTPulse;
				m_pCrpRobotCtrl->MakeMoveJ_Axis(pos, tPulseMove.dSpeed / 100, 1, 0, tPulseMove.dACC, tPulseMove.dDEC, 0, lineNum++, &MoveL_str);
				RecordFile << MoveL_str << "\n";
			}
			m_pCrpRobotCtrl->MakeMout(500, false, lineNum++, &mEnd);
			m_pCrpRobotCtrl->programerEnd(lineNum++, &end);
			RecordFile << mEnd << "\n";
			RecordFile << end << "\n";
			RecordFile.close();

			if (!m_pCrpRobotCtrl->Obj_FTP_cmd->FTP_if_load) return false;
			bool result = m_pCrpRobotCtrl->Obj_FTP_cmd->FTP_SendFile("Move_J_A");
			if (!result) return false;
			m_pCrpRobotCtrl->XiRobot_CallJob(2);
			return true;
		}
	}
	break;
	}
	return true;
}

void CRobotDriverAdaptor::SetIntValFun(int nIntVal, int nVal) {
	if (ROBOT_BRAND_YASKAWA != m_eRobotBrand) return;		// 安川专用 
	int nWhileNum = 0;
	int nIval = 0;
	SetIntVar_H(nIntVal, nVal); Sleep(10);
	nIval = GetIntVar(nIntVal);
	while (nIval != nVal) {
		if (m_eThreadStatus == INCISEHEAD_THREAD_STATUS_STOPPED) {
			break;
		}
		DoEvent();
		SetIntVar_H(nIntVal, nVal); Sleep(10);
		nIval = GetIntVar(nIntVal);
		if (nVal == nIval) {
			m_cLog->Write("I变量;%d 设置值:%d", nIntVal, nIval);
			break;
		}
		if (nWhileNum > 20) {
			XUI::MesBox::PopError("I变量：{0} 参数：{1} 设置失败", nIntVal, nVal);
			break;
		}
		nWhileNum++;
	}
}

bool CRobotDriverAdaptor::CompareCoords(T_ROBOT_COORS tCoords1, T_ROBOT_COORS tCoords2, double dCoordsLimit, int nCheckType, double dAngleLimit) {
	tCoords1.dRX = fmod(tCoords1.dRX, 360.0);
	tCoords1.dRY = fmod(tCoords1.dRY, 360.0);
	tCoords1.dRZ = fmod(tCoords1.dRZ, 360.0);

	tCoords1.dRX = tCoords1.dRX < 0.0 ? tCoords1.dRX + 360.0 : tCoords1.dRX;  // 0 - 360
	tCoords1.dRY = tCoords1.dRY < 0.0 ? tCoords1.dRY + 360.0 : tCoords1.dRY;
	tCoords1.dRZ = tCoords1.dRZ < 0.0 ? tCoords1.dRZ + 360.0 : tCoords1.dRZ;

	tCoords2.dRX = fmod(tCoords2.dRX, 360.0);
	tCoords2.dRY = fmod(tCoords2.dRY, 360.0);
	tCoords2.dRZ = fmod(tCoords2.dRZ, 360.0);

	tCoords2.dRX = tCoords2.dRX < 0.0 ? tCoords2.dRX + 360.0 : tCoords2.dRX;  // 0 - 360
	tCoords2.dRY = tCoords2.dRY < 0.0 ? tCoords2.dRY + 360.0 : tCoords2.dRY;
	tCoords2.dRZ = tCoords2.dRZ < 0.0 ? tCoords2.dRZ + 360.0 : tCoords2.dRZ;

	double dErrRx = fabs(tCoords1.dRX - tCoords2.dRX);
	double dErrRy = fabs(tCoords1.dRY - tCoords2.dRY);
	double dErrRz = fabs(tCoords1.dRZ - tCoords2.dRZ);

	dErrRx = dErrRx > 180.0 ? dErrRx - 360.0 : dErrRx;
	dErrRy = dErrRy > 180.0 ? dErrRy - 360.0 : dErrRy;
	dErrRz = dErrRz > 180.0 ? dErrRz - 360.0 : dErrRz;

	if (nCheckType == 1 || nCheckType == 3) {
		if (fabs(tCoords1.dX - tCoords2.dX) >= dCoordsLimit ||
			fabs(tCoords1.dY - tCoords2.dY) >= dCoordsLimit ||
			fabs(tCoords1.dZ - tCoords2.dZ) >= dCoordsLimit) {
			//WriteLog("CompareCoords xyz Err %.3lf %.3lf %.3lf",
			//	fabs(tCoords1.dX - tCoords2.dX),
			//	fabs(tCoords1.dY - tCoords2.dY),
			//	fabs(tCoords1.dZ - tCoords2.dZ));
			return false;
		}
	}
	if (nCheckType == 2 || nCheckType == 3) {
		if (fabs(dErrRx) >= dAngleLimit ||
			fabs(dErrRy) >= dAngleLimit ||
			fabs(dErrRz) >= dAngleLimit) {
			//WriteLog("CompareCoords rxryrz Err %.3lf %.3lf %.3lf", dErrRx, dErrRy, dErrRz);
			return false;
		}
	}
	return true;
}

bool CRobotDriverAdaptor::CompareCoords(T_ROBOT_COORS tCoords1, double dCoordsLimit, int nCheckType, double dAngleLimit) {
	tCoords1.dRX = fmod(tCoords1.dRX, 360.0);
	tCoords1.dRY = fmod(tCoords1.dRY, 360.0);
	tCoords1.dRZ = fmod(tCoords1.dRZ, 360.0);

	tCoords1.dRX = tCoords1.dRX < 0.0 ? tCoords1.dRX + 360.0 : tCoords1.dRX; // 0 - 360
	tCoords1.dRY = tCoords1.dRY < 0.0 ? tCoords1.dRY + 360.0 : tCoords1.dRY;
	tCoords1.dRZ = tCoords1.dRZ < 0.0 ? tCoords1.dRZ + 360.0 : tCoords1.dRZ;

	T_ROBOT_COORS tCoords2 = GetCurrentPos();
	tCoords2.dRX = tCoords2.dRX < 0.0 ? tCoords2.dRX + 360.0 : tCoords2.dRX;  // 0 - 360
	tCoords2.dRY = tCoords2.dRY < 0.0 ? tCoords2.dRY + 360.0 : tCoords2.dRY;
	tCoords2.dRZ = tCoords2.dRZ < 0.0 ? tCoords2.dRZ + 360.0 : tCoords2.dRZ;

	double dErrRx = fabs(tCoords1.dRX - tCoords2.dRX);
	double dErrRy = fabs(tCoords1.dRY - tCoords2.dRY);
	double dErrRz = fabs(tCoords1.dRZ - tCoords2.dRZ);

	dErrRx = dErrRx > 180.0 ? dErrRx - 360.0 : dErrRx;
	dErrRy = dErrRy > 180.0 ? dErrRy - 360.0 : dErrRy;
	dErrRz = dErrRz > 180.0 ? dErrRz - 360.0 : dErrRz;


	if (nCheckType == 1 || nCheckType == 3) {
		if (fabs(tCoords1.dX - tCoords2.dX) >= dCoordsLimit ||
			fabs(tCoords1.dY - tCoords2.dY) >= dCoordsLimit ||
			fabs(tCoords1.dZ - tCoords2.dZ) >= dCoordsLimit) {
			WriteLog("CompareCoords Err3: %.3lf %.3lf %.3lf",
				tCoords1.dX - tCoords2.dX,
				tCoords1.dY - tCoords2.dY,
				tCoords1.dZ - tCoords2.dZ);
			return false;
		}
	}
	if (nCheckType == 2 || nCheckType == 3) {
		if (fabs(dErrRx) >= dAngleLimit ||
			fabs(dErrRy) >= dAngleLimit ||
			fabs(dErrRz) >= dAngleLimit) {
			WriteLog("CompareCoords Err4: %.3lf %.3lf %.3lf", dErrRx, dErrRy, dErrRz);
			return false;
		}
	}
	return true;
}

bool CRobotDriverAdaptor::CompareXY(T_ROBOT_COORS tCoords1, T_ROBOT_COORS tCoords2, double dCoordsLimit) {
	if (fabs(tCoords1.dX - tCoords2.dX) >= dCoordsLimit
		|| fabs(tCoords1.dY - tCoords2.dY) >= dCoordsLimit) {
		return false;
	}
	return true;
}

bool CRobotDriverAdaptor::ComparePulse(T_ANGLE_PULSE tPulse1, T_ANGLE_PULSE tPulse2, long lLimit /*= 30*/) {
	Sleep(200);
	if (fabs((double)(tPulse1.nSPulse - tPulse2.nSPulse)) >= lLimit
		|| fabs((double)(tPulse1.nLPulse - tPulse2.nLPulse)) >= lLimit
		|| fabs((double)(tPulse1.nUPulse - tPulse2.nUPulse)) >= lLimit
		|| fabs((double)(tPulse1.nRPulse - tPulse2.nRPulse)) >= lLimit
		|| fabs((double)(tPulse1.nBPulse - tPulse2.nBPulse)) >= lLimit
		|| fabs((double)(tPulse1.nTPulse - tPulse2.nTPulse)) >= lLimit) {
		WriteLog("ComparePulse Err1: %d %d %d %d %d %d",
			tPulse1.nSPulse - tPulse2.nSPulse,
			tPulse1.nLPulse - tPulse2.nLPulse,
			tPulse1.nUPulse - tPulse2.nUPulse,
			tPulse1.nRPulse - tPulse2.nRPulse,
			tPulse1.nBPulse - tPulse2.nBPulse,
			tPulse1.nTPulse - tPulse2.nTPulse);
		return false;
	}
	return true;
}

bool CRobotDriverAdaptor::ComparePulse(T_ANGLE_PULSE tPulse1, long lLimit /*= 30*/) {
	T_ANGLE_PULSE tPulse2 = GetCurrentPulse();
	tPulse2 = GetCurrentPulse();
	if (fabs((double)(tPulse1.nSPulse - tPulse2.nSPulse)) >= lLimit
		|| fabs((double)(tPulse1.nLPulse - tPulse2.nLPulse)) >= lLimit
		|| fabs((double)(tPulse1.nUPulse - tPulse2.nUPulse)) >= lLimit
		|| fabs((double)(tPulse1.nRPulse - tPulse2.nRPulse)) >= lLimit
		|| fabs((double)(tPulse1.nBPulse - tPulse2.nBPulse)) >= lLimit
		|| fabs((double)(tPulse1.nTPulse - tPulse2.nTPulse)) >= lLimit) {
		WriteLog("ComparePulse Err2: %d %d %d %d %d %d",
			tPulse1.nSPulse - tPulse2.nSPulse,
			tPulse1.nLPulse - tPulse2.nLPulse,
			tPulse1.nUPulse - tPulse2.nUPulse,
			tPulse1.nRPulse - tPulse2.nRPulse,
			tPulse1.nBPulse - tPulse2.nBPulse,
			tPulse1.nTPulse - tPulse2.nTPulse);
		return false;
	}
	return true;
}

int CRobotDriverAdaptor::Welding(std::vector<T_ROBOT_COORS> vtWeldCOORS, double dRealWeldExPos, std::vector<int> vnPtnType, T_WELD_PARA Para, int weldType, bool arcOn)
{
	if (g_bLocalDebugMark) return 0;
	switch (m_eRobotBrand) {
	case ROBOT_BRAND_YASKAWA:		//安川
		break;
	case ROBOT_BRAND_ESTUN:			//埃斯顿
		break;
	case ROBOT_BRAND_CRP:				//卡诺普
	{
		/********************************** 1.变量声明************************************/
		vector<CRP_CPOS> vtCrp_CPos; //运动点数据
		vector<string> pro;          //存入文件的运动的字符串
		string changeTool_str;       //存入文件的工具字符串
		string changeUser_str;       //存入文件的用户字符串
		string mStrat, mEnd;         //存入程序运行状态
		string end;                  //最后一行结束的字符串
		string arc_str, arcEnd_str;  //起弧，停弧
		string waveStart, waveEnd;   //立焊的摆焊
		int lineNum = 1; //记录行数
		/****************** 2.对于焊缝点位进行提取，抽离出关键点位**********************/
		CRP_CPOS Crp_CPos;
		Crp_CPos.X = vtWeldCOORS[0].dX;
		Crp_CPos.Y = vtWeldCOORS[0].dY;
		Crp_CPos.Z = vtWeldCOORS[0].dZ;
		Crp_CPos.A = vtWeldCOORS[0].dRX;
		Crp_CPos.B = vtWeldCOORS[0].dRY;
		Crp_CPos.C = vtWeldCOORS[0].dRZ;
		Crp_CPos.A_7 = dRealWeldExPos;
		Crp_CPos.A_8 = vtWeldCOORS[0].dBY;
		vtCrp_CPos.push_back(Crp_CPos); //Ⅰ 焊缝下枪点（立焊的话到达后要开启起弧+摆焊；平焊的话到达后要开启起弧）

		int transformEnd = vtWeldCOORS.size(), weldEnd = vtWeldCOORS.size();
		for (int i = 1; i < vtWeldCOORS.size() - 1; i++) {

			//立焊： 立焊终点没有变姿态处理（但是有包角）
			if (weldType == 0) {

				//Ⅱ 开始段，姿态变化的终点
				if (vtWeldCOORS[i + 1].dRY == vtWeldCOORS[i].dRY && i < transformEnd) {
					CRP_CPOS Crp_CPos;
					Crp_CPos.X = vtWeldCOORS[i].dX;
					Crp_CPos.Y = vtWeldCOORS[i].dY;
					Crp_CPos.Z = vtWeldCOORS[i].dZ;
					Crp_CPos.A = vtWeldCOORS[i].dRX;
					Crp_CPos.B = vtWeldCOORS[i].dRY;
					Crp_CPos.C = vtWeldCOORS[i].dRZ;
					Crp_CPos.A_7 = dRealWeldExPos;
					Crp_CPos.A_8 = vtWeldCOORS[i].dBY;
					vtCrp_CPos.push_back(Crp_CPos);
					transformEnd = i;
				}

				//Ⅲ 焊接轨迹的终点(到达后停止摆焊）
				if (vtWeldCOORS[i + 1].dRY != vtWeldCOORS[i].dRY && i > transformEnd) {
					CRP_CPOS Crp_CPos;
					Crp_CPos.X = vtWeldCOORS[i].dX;
					Crp_CPos.Y = vtWeldCOORS[i].dY;
					Crp_CPos.Z = vtWeldCOORS[i].dZ;
					Crp_CPos.A = vtWeldCOORS[i].dRX;
					Crp_CPos.B = vtWeldCOORS[i].dRY;
					Crp_CPos.C = vtWeldCOORS[i].dRZ;
					Crp_CPos.A_7 = dRealWeldExPos;
					Crp_CPos.A_8 = vtWeldCOORS[i].dBY;
					vtCrp_CPos.push_back(Crp_CPos);
					weldEnd = i;
				}

				//Ⅳ 所有的包角点（到达最后一个点后，停弧）
				if (i > weldEnd) {
					CRP_CPOS Crp_CPos;
					Crp_CPos.X = vtWeldCOORS[i].dX;
					Crp_CPos.Y = vtWeldCOORS[i].dY;
					Crp_CPos.Z = vtWeldCOORS[i].dZ;
					Crp_CPos.A = vtWeldCOORS[i].dRX;
					Crp_CPos.B = vtWeldCOORS[i].dRY;
					Crp_CPos.C = vtWeldCOORS[i].dRZ;
					Crp_CPos.A_7 = dRealWeldExPos;
					Crp_CPos.A_8 = vtWeldCOORS[i].dBY;
					vtCrp_CPos.push_back(Crp_CPos);
				}
			}

			//平焊： 平焊终点需要变姿态处理 （但是无包角）
			else if (weldType == 1) {
				int transformEnd = vtWeldCOORS.size();

				//Ⅱ 起始段，变姿态的终点
				if (vtWeldCOORS[i + 1].dRZ == vtWeldCOORS[i].dRZ && i < transformEnd) {
					CRP_CPOS Crp_CPos;
					Crp_CPos.X = vtWeldCOORS[i].dX;
					Crp_CPos.Y = vtWeldCOORS[i].dY;
					Crp_CPos.Z = vtWeldCOORS[i].dZ;
					Crp_CPos.A = vtWeldCOORS[i].dRX;
					Crp_CPos.B = vtWeldCOORS[i].dRY;
					Crp_CPos.C = vtWeldCOORS[i].dRZ;
					Crp_CPos.A_7 = dRealWeldExPos;
					Crp_CPos.A_8 = vtWeldCOORS[i].dBY;
					vtCrp_CPos.push_back(Crp_CPos);
					transformEnd = i;
				}

				//Ⅲ 结尾段，变姿态的起点
				if (vtWeldCOORS[i + 1].dRZ != vtWeldCOORS[i].dRZ && i > transformEnd) {
					CRP_CPOS Crp_CPos;
					Crp_CPos.X = vtWeldCOORS[i].dX;
					Crp_CPos.Y = vtWeldCOORS[i].dY;
					Crp_CPos.Z = vtWeldCOORS[i].dZ;
					Crp_CPos.A = vtWeldCOORS[i].dRX;
					Crp_CPos.B = vtWeldCOORS[i].dRY;
					Crp_CPos.C = vtWeldCOORS[i].dRZ;
					Crp_CPos.A_7 = dRealWeldExPos;
					Crp_CPos.A_8 = vtWeldCOORS[i].dBY;
					vtCrp_CPos.push_back(Crp_CPos);
				}
			}
		}

		//Ⅳ 结尾段，变换姿态的终点（到达后停弧）
		Crp_CPos.X = vtWeldCOORS[vtWeldCOORS.size() - 1].dX;
		Crp_CPos.Y = vtWeldCOORS[vtWeldCOORS.size() - 1].dY;
		Crp_CPos.Z = vtWeldCOORS[vtWeldCOORS.size() - 1].dZ;
		Crp_CPos.A = vtWeldCOORS[vtWeldCOORS.size() - 1].dRX;
		Crp_CPos.B = vtWeldCOORS[vtWeldCOORS.size() - 1].dRY;
		Crp_CPos.C = vtWeldCOORS[vtWeldCOORS.size() - 1].dRZ;
		Crp_CPos.A_7 = dRealWeldExPos;
		Crp_CPos.A_8 = vtWeldCOORS[vtWeldCOORS.size() - 1].dBY;
		vtCrp_CPos.push_back(Crp_CPos);
		/************************ 3.下载焊接工艺文件**********************************/
		const char name[] = "ArcPara_Record.txt";
		char weldPara[200];
		memset(weldPara, '\0', sizeof(char) * 200);
		string tmp;

		if (0 != m_pCrpRobotCtrl->Obj_FTP_cmd->FTP_DownloadFileByCoverLocal("/file/Arc/ArcPara_Record.txt", name)) {
			return -1;
		}
		ifstream ReadFile; //读到下载到焊接文件
		ReadFile.open("./file/ArcPara_Record.txt");
		if (ReadFile.fail()) {
			return -2;
		}
		/************************ 4.修改焊接工艺文件并上传**********************************/
		//立焊
		if (weldType == 0) {
			sprintf(weldPara, "0 程序0  %.6f %.6f %.6f %.6f 140.000000 20.000000 %.6f 1.000000 0 %.6f %.6f %.6f 0.000000 0.000000 0.000000 0 ",
				Para.dTrackCurrent, Para.dTrackVoltage, Para.dStopArcCurrent, Para.dStopArcVoltage, Para.dStopWaitTime, Para.dStartArcCurrent, Para.dStartArcVoltage, Para.dStartWaitTime);
		}
		//平焊
		else if (weldType == 1) {
			sprintf(weldPara, "1 程序1  %.6f %.6f %.6f %.6f 140.000000 20.000000 %.6f 1.000000 0 %.6f %.6f %.6f 0.000000 0.000000 0.000000 0 ",
				Para.dTrackCurrent, Para.dTrackVoltage, Para.dStopArcCurrent, Para.dStopArcVoltage, Para.dStopWaitTime, Para.dStartArcCurrent, Para.dStartArcVoltage, Para.dStartWaitTime);
		}

		char line[1024] = { '\0' };
		string lineText;
		int curLineNum = 0; //当前行数
		string tempStr;

		//修改焊接工艺文件
		while (ReadFile.getline(line, sizeof(line))) {
			curLineNum++;
			if (weldType + 1 != curLineNum) {
				lineText = line;
				tempStr += lineText;
			}
			else {
				lineText = weldPara;
				tempStr += lineText;
			}
			tempStr += '\n';
		}
		ReadFile.close();
		ofstream out;
		out.open("./file/ArcPara_Record.txt");
		if (out.fail()) {
			return -3;
		}
		out.flush();
		out << tempStr; //写入修改后的内容
		out.close();
		//上传修改后的焊接文件
		if (!m_pCrpRobotCtrl->Obj_FTP_cmd->FTP_SendFileToAddress("./file/ArcPara_Record.txt", "./file/Arc/")) {
			return -4;
		}
		/************************ 5.生成指令字符串**********************************/
		m_pCrpRobotCtrl->MakeMout(500, true, lineNum++, &mStrat);
		m_pCrpRobotCtrl->MakeChangeTool(1, lineNum++, changeTool_str); //第一个参数工具1 
		m_pCrpRobotCtrl->MakeChangeUse(0, lineNum++, changeUser_str); //第一个参数用户0


		//准备程序文件运动的字符串
		string temp;
		double LPos[10] = { 0.0 };
		for (int i = 0; i < vtCrp_CPos.size(); i++) {
			LPos[0] = vtCrp_CPos[i].X;
			LPos[1] = vtCrp_CPos[i].Y;
			LPos[2] = vtCrp_CPos[i].Z;
			LPos[3] = vtCrp_CPos[i].A;
			LPos[4] = vtCrp_CPos[i].B;
			LPos[5] = vtCrp_CPos[i].C;
			LPos[6] = vtCrp_CPos[i].A_7;
			LPos[7] = 0;
			LPos[8] = 1;
			LPos[9] = 0;

			//立焊
			if (weldType == 0) {
				switch (i) {
				case 0: { //焊接起点（开起弧、开摆焊）
					m_pCrpRobotCtrl->MakeMoveL(LPos, 15, 1, 0, 1, 1, 9, lineNum++, &temp);
					if (arcOn) {
						m_pCrpRobotCtrl->MakeArc(weldType, 0, 0, Para.WeldVelocity / 60, 0, lineNum++, &arc_str);
					}
					m_pCrpRobotCtrl->MakeWaveStart(0, lineNum++, &waveStart);
					break;
				}
				case 2: { //焊接终点(停止摆焊)
					m_pCrpRobotCtrl->MakeMoveL(LPos, Para.WeldVelocity / 60, 1, 0, 1, 1, 0, lineNum++, &temp);
					m_pCrpRobotCtrl->MakeWaveEnd(lineNum++, &waveEnd);
					break;
				}
				default: {
					m_pCrpRobotCtrl->MakeMoveL(LPos, Para.WeldVelocity / 60, 1, 0, 1, 1, 0, lineNum++, &temp);
					break;
				}
				}
			}

			//平焊
			if (weldType == 1) {
				if (i == 0) {
					m_pCrpRobotCtrl->MakeMoveL(LPos, 20, 1, 0, 1, 1, 0, lineNum++, &temp);
					if (arcOn) {
						m_pCrpRobotCtrl->MakeArc(weldType, 0, 0, Para.WeldVelocity / 60, 0, lineNum++, &arc_str);
					}
				}
				else {
					m_pCrpRobotCtrl->MakeMoveL(LPos, Para.WeldVelocity / 60, 1, 0, 1, 1, 0, lineNum++, &temp);
				}
			}

			// 所有点走完，停弧
			if (arcOn) {
				m_pCrpRobotCtrl->MakeArcEnd(weldType, lineNum++, &arcEnd_str);
			}
			pro.push_back(temp);
		}
		/*********************** 6.创建示教程序文件，将2所得指令写入文件*************/
		std::ofstream RecordFile("Move_J_A");
		RecordFile << mStrat << "\n";
		RecordFile << changeTool_str << "\n";
		RecordFile << changeUser_str << "\n";

		//写入运动部分
		for (int i = 0; i < pro.size(); i++) {
			switch (i) {
				//焊接起点（立焊需要额外添加摆焊）
			case 0: {
				RecordFile << pro[i] << "\n";
				if (arcOn) {
					RecordFile << arc_str << "\n";
				}
				if (weldType == 0) { //立焊后加摆焊
					RecordFile << waveStart << "\n";
				}
				break;
			}
				  //立焊包角点起点/平焊结尾变换姿态的起点（立焊需要额外添加停止摆动)
			case 2: {
				if (weldType == 0) { //立焊结束后加停止摆焊
					RecordFile << pro[i] << "\n";
					RecordFile << waveEnd << "\n";
				}
				else if (weldType == 1) {
					RecordFile << pro[i] << "\n";
				}
				break;
			}
			default: {
				RecordFile << pro[i] << "\n";
				break;
			}
			}
		}
		if (arcOn) {
			RecordFile << arcEnd_str << "\n";
		}
		m_pCrpRobotCtrl->MakeMout(500, false, lineNum++, &mEnd);
		m_pCrpRobotCtrl->programerEnd(lineNum, &end);

		RecordFile << mEnd << "\n";
		RecordFile << end << "\n";
		RecordFile.close();
		/************************ 7.发送文件到示教器**********************************/
		if (!m_pCrpRobotCtrl->Obj_FTP_cmd->FTP_if_load) return -1;
		bool result = m_pCrpRobotCtrl->Obj_FTP_cmd->FTP_SendFile("Move_J_A");
		if (!result) return -1;
		Sleep(100);
		m_pCrpRobotCtrl->XiRobot_CallJob(2); //会焊接很吓人！
	}
	break;
	default:
		break;
	}
	return 0;
}

int CRobotDriverAdaptor::SendTeachMoveData_L(std::vector<T_ROBOT_COORS> vtMeasureCOORS, std::vector<bool> veMeasureType, int nDowdSpeed, int nTeachSpeed, int nUpSpeed, int nTrigSigTime)
{
	if (g_bLocalDebugMark) return 0;
	switch (m_eRobotBrand) {
	case ROBOT_BRAND_YASKAWA:		//安川
		break;
	case ROBOT_BRAND_ESTUN:			//埃斯顿
		break;
	case ROBOT_BRAND_CRP:				//卡诺普
	{
		//准备角度
		vector<CRP_CPOS> vtCrp_CPos;
		for (int i = 0; i < vtMeasureCOORS.size(); i++) {
			CRP_CPOS Crp_CPos;
			Crp_CPos.X = vtMeasureCOORS[i].dX;
			Crp_CPos.Y = vtMeasureCOORS[i].dY;
			Crp_CPos.Z = vtMeasureCOORS[i].dZ;
			Crp_CPos.A = vtMeasureCOORS[i].dRX;
			Crp_CPos.B = vtMeasureCOORS[i].dRY;
			Crp_CPos.C = vtMeasureCOORS[i].dRZ;
			Crp_CPos.A_7 = 0;
			Crp_CPos.A_8 = 0;
			vtCrp_CPos.push_back(Crp_CPos);
		}
		//准备存入文件的字符串
		vector<string> pro;  //存入文件的运动的字符串
		string changeTool_str; //存入文件的工具字符串
		string changeUser_str; //存入文件的用户字符串
		string OpenTrackLaser; //程序开始打开激光的字符串
		string CloseCrm; //程序开始关闭相机的字符串
		string end; //最后一行结束的字符串
		int dACC = 20;
		int dDEC = 20;
		//程序文件前4行
		m_pCrpRobotCtrl->MakeChangeTool(1, 1, changeTool_str); //第一个参数工具1 
		m_pCrpRobotCtrl->MakeChangeUse(0, 2, changeUser_str); //第一个参数用户0
		m_pCrpRobotCtrl->MakeDout(12, false, 3, &CloseCrm);
		m_pCrpRobotCtrl->MakeDout(10, true, 4, &OpenTrackLaser);
		//准备程序文件运动的字符串
		int lineNum = 5; //记录行数
		for (int i = 0; i < vtCrp_CPos.size(); i++) {
			string temp;
			//判断是否是过渡点
			if (veMeasureType[i]) {
				//判断是否是下枪点 收枪点
				if (i == 0) {
					m_pCrpRobotCtrl->MakeMoveL(vtCrp_CPos[i], nDowdSpeed / 60, 1, 0, dACC, dDEC, 0, lineNum, &temp);
					pro.push_back(temp);
				}
				else if (i == vtCrp_CPos.size()) {
					m_pCrpRobotCtrl->MakeMoveL(vtCrp_CPos[i], nUpSpeed / 60, 1, 0, dACC, dDEC, 0, lineNum, &temp);
					pro.push_back(temp);
				}
				else {
					m_pCrpRobotCtrl->MakeMoveL(vtCrp_CPos[i], nTeachSpeed / 60, 1, 0, dACC, dDEC, 0, lineNum, &temp);
					pro.push_back(temp);
				}
				lineNum++;
				//采图
				m_pCrpRobotCtrl->MakeDout(12, 1, lineNum, &temp);//打开相机
				pro.push_back(temp);
				lineNum++;
				MakeSleep(nTrigSigTime, lineNum, temp);
				pro.push_back(temp);
				lineNum++;
				m_pCrpRobotCtrl->MakeDout(12, 0, lineNum, &temp);//关闭相机
				pro.push_back(temp);
				lineNum++;
			}
			else {
				//采图
				if (i == 0) {
					m_pCrpRobotCtrl->MakeMoveL(vtCrp_CPos[i], nDowdSpeed / 60, 1, 0, dACC, dDEC, 9, lineNum, &temp);
					pro.push_back(temp);
				}
				else if (i == vtCrp_CPos.size()) {
					m_pCrpRobotCtrl->MakeMoveL(vtCrp_CPos[i], nUpSpeed / 60, 1, 0, dACC, dDEC, 9, lineNum, &temp);
					pro.push_back(temp);
				}
				else {
					m_pCrpRobotCtrl->MakeMoveL(vtCrp_CPos[i], nTeachSpeed / 60, 1, 0, dACC, dDEC, 9, lineNum, &temp);
					pro.push_back(temp);
				}
				lineNum++;
			}
		}
		//创建文件Move_L
		std::ofstream RecordFile("Move_L"); //用记事本的方式记录下来
		//写入头4行
		RecordFile << changeTool_str << "\n";
		RecordFile << changeUser_str << "\n";
		RecordFile << CloseCrm << "\n";
		RecordFile << OpenTrackLaser << "\n";
		//写入运动部分
		for (int i = 0; i < pro.size(); i++) {
			RecordFile << pro[i] << "\n";
		}
		//写入结尾
		RecordFile << end << "\n";
		RecordFile.close();
		//发送文件
		if (!m_pCrpRobotCtrl->Obj_FTP_cmd->FTP_if_load) return -1;
		bool result = m_pCrpRobotCtrl->Obj_FTP_cmd->FTP_SendFile("Move_L");
		if (!result) return -1;
	}
	break;
	default:
		break;
	}
	return 0;
}



int CRobotDriverAdaptor::SendTeachMoveData_J(std::vector<T_ANGLE_PULSE> vtMeasurePulse, double ExternalAxle, const std::vector<int> veMeasureType, int nDowdSpeed, int nTeachSpeed, int nUpSpeed, int nTrigSigTime)
{
	if (g_bLocalDebugMark) return 0;
	switch (m_eRobotBrand) {
	case ROBOT_BRAND_YASKAWA:		//安川
		break;
	case ROBOT_BRAND_ESTUN:			//埃斯顿
		break;
	case ROBOT_BRAND_CRP:				//卡诺普
	{
		//准备坐标点
		vector<CRP_APOS> vtCrp_APos;
		for (int i = 0; i < vtMeasurePulse.size(); i++) {
			CRP_APOS Crp_APos;
			Crp_APos.A1 = vtMeasurePulse[i].nSPulse * m_tAxisUnit.dSPulse;
			Crp_APos.A2 = vtMeasurePulse[i].nLPulse * m_tAxisUnit.dLPulse;
			Crp_APos.A3 = vtMeasurePulse[i].nUPulse * m_tAxisUnit.dUPulse;
			Crp_APos.A4 = vtMeasurePulse[i].nRPulse * m_tAxisUnit.dRPulse;
			Crp_APos.A5 = vtMeasurePulse[i].nBPulse * m_tAxisUnit.dBPulse;
			Crp_APos.A6 = vtMeasurePulse[i].nTPulse * m_tAxisUnit.dTPulse;
			Crp_APos.A7 = ExternalAxle;
			Crp_APos.A8 = 0;
			vtCrp_APos.push_back(Crp_APos);
		}
		//准备存入文件的字符串
		vector<string> pro;  //存入文件的运动的字符串
		string changeTool_str; //存入文件的工具字符串
		string changeUser_str; //存入文件的用户字符串
		string OpenTrackLaser, CloseTrackLaser; //程序开始打开激光的字符串
		string CloseCrm; //程序开始关闭相机的字符串
		string end; //最后一行结束的字符串
		string Extern; //外部轴移动
		int dACC = 20;
		int dDEC = 20;
		//程序文件前5行
		m_pCrpRobotCtrl->MakeChangeTool(1, 1, changeTool_str); //第一个参数工具1 
		m_pCrpRobotCtrl->MakeChangeUse(0, 2, changeUser_str); //第一个参数用户0
		m_pCrpRobotCtrl->MakeDout(12, false, 3, &CloseCrm);
		m_pCrpRobotCtrl->MakeDout(10, true, 4, &OpenTrackLaser);
		double axis[10] = { 0.0 };
		if (0 > m_pCrpRobotCtrl->GetCurrAxisByAngle(axis)) {
			XiMessageBox("获取失败");
			return -1;
		}
		CRP_APOS extrnPos = { axis[0],axis[1], axis[2], axis[3], axis[4], axis[5], ExternalAxle, axis[7] };
		m_pCrpRobotCtrl->MakeMoveJ_Axis(extrnPos, nDowdSpeed / 100, 1, 0, 1, 1, 9, 5, &Extern);

		//准备程序文件运动的字符串
		int lineNum = 6; //记录行数
		for (int i = 0; i < vtCrp_APos.size(); i++) {
			string temp;
			//判断是否是过渡点(枚举使用)？？？？？？
			if (veMeasureType[i]) {
				//判断是否是下枪点 收枪点
				if (veMeasureType[i] == 1) {
					m_pCrpRobotCtrl->MakeMoveJ_Axis(vtCrp_APos[i], nDowdSpeed / 100, 1, 0, 1, 1, 9, lineNum++, &temp);
					pro.push_back(temp);
				}
				else {
					m_pCrpRobotCtrl->MakeMoveJ_Axis(vtCrp_APos[i], nTeachSpeed / 100, 1, 0, 1, 1, 0, lineNum++, &temp);
					pro.push_back(temp);

					//采图
					m_pCrpRobotCtrl->MakeDout(12, 1, lineNum++, &temp);//打开相机
					pro.push_back(temp);
					lineNum++;
					MakeSleep(nTrigSigTime, lineNum++, temp);
					pro.push_back(temp);
					lineNum++;
					m_pCrpRobotCtrl->MakeDout(12, 0, lineNum++, &temp);//关闭相机
					pro.push_back(temp);
					lineNum++;
				}
			}
			else {
				//采图
				if (i == 0) {
					m_pCrpRobotCtrl->MakeMoveJ_Axis(vtCrp_APos[i], nDowdSpeed / 100, 1, 0, 1, 1, 0, lineNum, &temp);
					pro.push_back(temp);
				}
				else if (i == vtCrp_APos.size()) {
					m_pCrpRobotCtrl->MakeMoveJ_Axis(vtCrp_APos[i], nUpSpeed / 100, 1, 0, 1, 1, 0, lineNum, &temp);
					pro.push_back(temp);
				}
				else {
					m_pCrpRobotCtrl->MakeMoveJ_Axis(vtCrp_APos[i], nTeachSpeed / 100, 1, 0, 1, 1, 0, lineNum, &temp);
					pro.push_back(temp);
				}
				lineNum++;
			}
		}
		m_pCrpRobotCtrl->MakeDout(10, false, lineNum++, &CloseTrackLaser);
		//创建文件Move_L
		std::ofstream RecordFile("Move_J_A"); //用记事本的方式记录下来
		//写入头4行
		RecordFile << changeTool_str << "\n";
		RecordFile << changeUser_str << "\n";
		RecordFile << CloseCrm << "\n";
		RecordFile << OpenTrackLaser << "\n";
		RecordFile << Extern << "\n";
		//写入运动部分
		for (int i = 0; i < pro.size(); i++) {
			RecordFile << pro[i] << "\n";
		}
		//写入关闭激光
		RecordFile << CloseTrackLaser << "\n";
		//写入结尾
		m_pCrpRobotCtrl->programerEnd(lineNum, &end);
		RecordFile << end << "\n";
		RecordFile.close();

		//发送文件
		if (!m_pCrpRobotCtrl->Obj_FTP_cmd->FTP_if_load) return -1;
		bool result = m_pCrpRobotCtrl->Obj_FTP_cmd->FTP_SendFile("Move_J_A");
		if (!result) return -1;
		m_pCrpRobotCtrl->XiRobot_CallJob(2);
	}
	break;
	default:
		break;
	}
	return 0;
}

int CRobotDriverAdaptor::Read_M(int M_Num)
{
	if (g_bLocalDebugMark) return TRUE;
	int result = m_pCrpRobotCtrl->Read_M_Number_status(M_Num);
	return result;
}

int CRobotDriverAdaptor::Write_M(int M_Num, int Status)
{
	if (g_bLocalDebugMark) return TRUE;
	int result = m_pCrpRobotCtrl->Write_M_Number_status(M_Num, Status > 0);
	return result;
}

int CRobotDriverAdaptor::Read_Y(int Y_num)
{
	if (g_bLocalDebugMark) return TRUE;
	int result = m_pCrpRobotCtrl->Read_Y_Number_status(Y_num);
	return result;
}

int CRobotDriverAdaptor::Write_Y(int Y_num, int Status)
{
	if (g_bLocalDebugMark) return TRUE;
	int result = m_pCrpRobotCtrl->Write_Y_Number_status(Y_num, Status > 0);
	return result;

}

void CRobotDriverAdaptor::MakeDout(int M_num, bool state, int lineNum, std::string* Dout_str)
{
	if (g_bLocalDebugMark) return;
	char Dout[50];
	memset(Dout, '\0', 50);
	if (state) {
		sprintf(Dout, "DOUT M#(%d)=ON N%d ", M_num, lineNum);
	}
	else {
		sprintf(Dout, "DOUT M#(%d)=OFF N%d ", M_num, lineNum);
	}
	*Dout_str = Dout;
}

void CRobotDriverAdaptor::MakeSleep(int SleepTime, int LineNumber, string& Sleep_Str)
{
	if (g_bLocalDebugMark) return;
	char Sleep[1024];
	sprintf(Sleep, "WAIT M#(0)==ON DT=%d CT=0", SleepTime);
	Sleep_Str = Sleep;
}

E_ERROR_STATE CRobotDriverAdaptor::CheckRobotDone_CRP(int nDelayTime)
{
	int nMinDelayTime = 1000; // 卡诺普相对与安川延时时间普遍较大
	nDelayTime = nDelayTime < nMinDelayTime ? nMinDelayTime : nDelayTime;

	//获取当前时间
	long long nInitialTime = XI_clock();
	//定义M500状态（M500是在子程序开始的时候设置的，目的是在程序运行的时候监视程序状态， 程序运行中：1，程序停止：0）
	int nStopStatus = 0;
	//循环读取机器人M21的状态 直到ntime大于等待时间或者机器人退出停止状态（可能是急停），则看情况选择返回值
	while ((nStopStatus == this->Read_M(500)) &&  //行健自定义的运动状态检测寄存器
		(nStopStatus == this->Read_M(21))     //卡诺普自带的运动状态检测寄存器
		) {
		//定义该函数的运行时间
		long long ntime = XI_clock() - nInitialTime;
		if (ntime > nDelayTime) {
			break;
		}
		Sleep(100);
	}

	/*确保两种方法都判断出运动已停止才返回运动状态已停止*/
	while ((nStopStatus != this->Read_M(500)) &&   //行健自定义的运动状态检测寄存器
		(nStopStatus != this->Read_M(21))       //卡诺普自带的运动状态检测寄存器
		) {
		Sleep(100);
	}
	return RUNNING_STATUS_SUCCESS;
}

MP_USR_VAR_INFO CRobotDriverAdaptor::PrepareValData(int nIntValIdx, int nValue)
{
	switch (m_eRobotBrand) {
	case ROBOT_BRAND_YASKAWA:		//安川
	{
		MP_USR_VAR_INFO tValInfo;
		tValInfo.var_type = MP_VAR_I;
		tValInfo.var_no = nIntValIdx;
		tValInfo.val.i = nValue;
		return tValInfo;
	}
	break;
	case ROBOT_BRAND_ESTUN:			//埃斯顿
		break;
	case ROBOT_BRAND_CRP:				//卡诺普
		break;
	default:
		break;
	}
	return MP_USR_VAR_INFO();
}

MP_USR_VAR_INFO CRobotDriverAdaptor::PrepareValData(int nPosVarIdx, long lBPValue[3], UINT unToolNum, UINT unUserNo, UINT unPosture)
{
	switch (m_eRobotBrand) {
	case ROBOT_BRAND_YASKAWA:		//安川
	{
		ReloadBP(lBPValue, m_nExternalAxleType);
		MP_USR_VAR_INFO tValInfo;
		tValInfo.var_type = MP_VAR_BP;
		tValInfo.var_no = nPosVarIdx;
		tValInfo.val.p.dtype = MP_BASE_COORD;
		tValInfo.val.p.uf_no = unUserNo;
		tValInfo.val.p.tool_no = unToolNum;
		tValInfo.val.p.fig_ctrl = unPosture;
		tValInfo.val.p.data[0] = lBPValue[0];
		tValInfo.val.p.data[1] = lBPValue[1];
		tValInfo.val.p.data[2] = lBPValue[2];
		tValInfo.val.p.data[3] = 0;
		tValInfo.val.p.data[4] = 0;
		tValInfo.val.p.data[5] = 0;
		tValInfo.val.p.data[6] = 0;
		tValInfo.val.p.data[7] = 0;
		return tValInfo;
	}
	break;
	case ROBOT_BRAND_ESTUN:			//埃斯顿
		break;
	case ROBOT_BRAND_CRP:				//卡诺普
		break;
	default:
		break;
	}
	return MP_USR_VAR_INFO();
}

MP_USR_VAR_INFO CRobotDriverAdaptor::PrepareValDataEx(int nPosVarIdx, T_ANGLE_PULSE tPulse, UINT unToolNum, UINT unUserNo, UINT unPosture)
{
	long lBPValue[3];
	lBPValue[0] = tPulse.lBXPulse;
	lBPValue[1] = tPulse.lBYPulse;
	lBPValue[2] = tPulse.lBZPulse;

	switch (m_eRobotBrand) {
	case ROBOT_BRAND_YASKAWA:		//安川
	{
		ReloadBP(lBPValue, m_nExternalAxleType);
		MP_USR_VAR_INFO tValInfo;
		tValInfo.var_type = MP_VAR_BP;
		tValInfo.var_no = nPosVarIdx;
		tValInfo.val.p.dtype = MP_PULSE_COORD;
		tValInfo.val.p.uf_no = unUserNo;
		tValInfo.val.p.tool_no = unToolNum;
		tValInfo.val.p.fig_ctrl = unPosture;
		tValInfo.val.p.data[0] = lBPValue[0];
		tValInfo.val.p.data[1] = lBPValue[1];
		tValInfo.val.p.data[2] = lBPValue[2];
		tValInfo.val.p.data[3] = 0;
		tValInfo.val.p.data[4] = 0;
		tValInfo.val.p.data[5] = 0;
		tValInfo.val.p.data[6] = 0;
		tValInfo.val.p.data[7] = 0;
		return tValInfo;
	}
	break;
	case ROBOT_BRAND_ESTUN:			//埃斯顿
		break;
	case ROBOT_BRAND_CRP:				//卡诺普
		break;
	default:
		break;
	}
	return MP_USR_VAR_INFO();
}

MP_USR_VAR_INFO CRobotDriverAdaptor::PrepareValDataEx(int nPosVarIdx, T_ROBOT_COORS tPulse, UINT unToolNum, UINT unUserNo, UINT unPosture)
{
	long lBPValue[3];
	lBPValue[0] = tPulse.dBX * 1000;
	lBPValue[1] = tPulse.dBY * 1000;
	lBPValue[2] = tPulse.dBZ * 1000;

	switch (m_eRobotBrand) {
	case ROBOT_BRAND_YASKAWA:		//安川
	{
		ReloadBP(lBPValue, m_nExternalAxleType);
		MP_USR_VAR_INFO tValInfo;
		tValInfo.var_type = MP_VAR_BP;
		tValInfo.var_no = nPosVarIdx;
		tValInfo.val.p.dtype = MP_BASE_COORD;
		tValInfo.val.p.uf_no = unUserNo;
		tValInfo.val.p.tool_no = unToolNum;
		tValInfo.val.p.fig_ctrl = unPosture;
		tValInfo.val.p.data[0] = lBPValue[0];
		tValInfo.val.p.data[1] = lBPValue[1];
		tValInfo.val.p.data[2] = lBPValue[2];
		tValInfo.val.p.data[3] = 0;
		tValInfo.val.p.data[4] = 0;
		tValInfo.val.p.data[5] = 0;
		tValInfo.val.p.data[6] = 0;
		tValInfo.val.p.data[7] = 0;
		return tValInfo;
	}
	break;
	case ROBOT_BRAND_ESTUN:			//埃斯顿
		break;
	case ROBOT_BRAND_CRP:				//卡诺普
		break;
	default:
		break;
	}
	return MP_USR_VAR_INFO();
}

MP_USR_VAR_INFO CRobotDriverAdaptor::PrepareValData(int nPosVarIdx, T_ANGLE_PULSE tPulse, UINT unToolNum, UINT unUserNo, UINT unPosture)
{
	switch (m_eRobotBrand) {
	case ROBOT_BRAND_YASKAWA:		//安川
	{
		MP_USR_VAR_INFO tValInfo;
		tValInfo.var_type = MP_VAR_P;
		tValInfo.var_no = nPosVarIdx;
		tValInfo.val.p.dtype = MP_PULSE_COORD;
		tValInfo.val.p.uf_no = unUserNo;
		tValInfo.val.p.tool_no = unToolNum;
		tValInfo.val.p.fig_ctrl = unPosture;
		tValInfo.val.p.data[0] = tPulse.nSPulse;
		tValInfo.val.p.data[1] = tPulse.nLPulse;
		tValInfo.val.p.data[2] = tPulse.nUPulse;
		tValInfo.val.p.data[3] = tPulse.nRPulse;
		tValInfo.val.p.data[4] = tPulse.nBPulse;
		tValInfo.val.p.data[5] = tPulse.nTPulse;
		tValInfo.val.p.data[6] = 0;
		tValInfo.val.p.data[7] = 0;
		return tValInfo;
	}
	break;
	case ROBOT_BRAND_ESTUN:			//埃斯顿
		break;
	case ROBOT_BRAND_CRP:				//卡诺普
		break;
	default:
		break;
	}
	return MP_USR_VAR_INFO();
}

MP_USR_VAR_INFO CRobotDriverAdaptor::PrepareValData(int nPosVarIdx, T_ROBOT_COORS tCoord, UINT unToolNum, UINT unUserNo, UINT unPosture)
{
	switch (m_eRobotBrand) {
	case ROBOT_BRAND_YASKAWA:		//安川
	{
		MP_USR_VAR_INFO tValInfo;
		tValInfo.var_type = MP_VAR_P;
		tValInfo.var_no = nPosVarIdx;
		tValInfo.val.p.dtype = MP_ROBO_COORD;
		tValInfo.val.p.uf_no = unUserNo;
		tValInfo.val.p.tool_no = unToolNum;
		tValInfo.val.p.fig_ctrl = unPosture;
		tValInfo.val.p.data[0] = long(tCoord.dX * 1000);
		tValInfo.val.p.data[1] = long(tCoord.dY * 1000);
		tValInfo.val.p.data[2] = long(tCoord.dZ * 1000);
		tValInfo.val.p.data[3] = long(tCoord.dRX * 10000);
		tValInfo.val.p.data[4] = long(tCoord.dRY * 10000);
		tValInfo.val.p.data[5] = long(tCoord.dRZ * 10000);
		tValInfo.val.p.data[6] = 0;
		tValInfo.val.p.data[7] = 0;
		return tValInfo;
	}
	break;
	case ROBOT_BRAND_ESTUN:			//埃斯顿
		break;
	case ROBOT_BRAND_CRP:				//卡诺普
		break;
	default:
		break;
	}
	return MP_USR_VAR_INFO();
}

int CRobotDriverAdaptor::GetPanasonicExDir(E_EXTERNAL_AXLE_TYPE eExAxisType)
{
	//获取龙门坐标失败
	double dReadPos = 0.0;
	int nToRobotDir = 0;
	double dDir = 1.0;
	if (eExAxisType == E_EX_X) nToRobotDir = 1;
	if (eExAxisType == E_EX_Y) nToRobotDir = 2;
	if (eExAxisType == E_EX_Z) nToRobotDir = 3;
	CUnitDriver* pMotorDirver = NULL;
	for (int i = 0; i < m_pvpMotorDriver->size(); i++)
	{
		if (nToRobotDir == labs(m_pvpMotorDriver->at(i)->GetMotorParam().nToRobotDir))
		{
			pMotorDirver = m_pvpMotorDriver->at(i);
			dDir = m_pvpMotorDriver->at(i)->GetMotorParam().nToRobotDir > 0 ? 1.0 : -1.0;
		}
	}

	return dDir;
}

int CRobotDriverAdaptor::GetPanasonicExPos(E_EXTERNAL_AXLE_TYPE eExAxisType, double &dExAxisPos)
{
	double dReadPos = 0.0;
	int nToRobotDir = 0;
	double dDir = 1.0;
	if (eExAxisType == E_EX_X) nToRobotDir = 1;
	if (eExAxisType == E_EX_Y) nToRobotDir = 2;
	if (eExAxisType == E_EX_Z) nToRobotDir = 3;
	CUnitDriver* pMotorDirver = NULL;
	for (int i = 0; i < m_pvpMotorDriver->size(); i++)
	{
		if (nToRobotDir == labs(m_pvpMotorDriver->at(i)->GetMotorParam().nToRobotDir))
		{
			pMotorDirver = m_pvpMotorDriver->at(i);
			dDir = m_pvpMotorDriver->at(i)->GetMotorParam().nToRobotDir > 0 ? 1.0 : -1.0;
		}
	}
	if (NULL == pMotorDirver) return dReadPos;
	pMotorDirver->GetCurrentPosition(dReadPos);
	dExAxisPos = dReadPos * dDir;
	return 0;
}

int CRobotDriverAdaptor::GetPanasonicExPos(E_EXTERNAL_AXLE_TYPE eExAxisType, long& lExAxisPos)
{
	long lReadPos = 0;
	int nToRobotDir = 0;
	double dDir = 1.0;
	if (eExAxisType == E_EX_X) nToRobotDir = 1;
	if (eExAxisType == E_EX_Y) nToRobotDir = 2;
	if (eExAxisType == E_EX_Z) nToRobotDir = 3;
	CUnitDriver* pMotorDirver = NULL;
	for (int i = 0; i < m_pvpMotorDriver->size(); i++)
	{
		if (nToRobotDir == labs(m_pvpMotorDriver->at(i)->GetMotorParam().nToRobotDir))
		{
			pMotorDirver = m_pvpMotorDriver->at(i);
			dDir = m_pvpMotorDriver->at(i)->GetMotorParam().nToRobotDir > 0 ? 1.0 : -1.0;
		}
	}
	if (NULL == pMotorDirver) return lReadPos;
	pMotorDirver->GetCurrentPosition(lReadPos);
	lExAxisPos = lReadPos * dDir;
	return 0;
}