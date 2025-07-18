
#include "stdafx.h"
#include "EstunAdaptor.h"
#include ".\Project\XiGrooveRobot.h"


EstunAdaptor::EstunAdaptor(CString strUnitName, CLog* cLog, std::vector<CUnitDriver*>* ppUnitDriver)
	:CRobotDriverAdaptor(strUnitName, cLog, ppUnitDriver)
{
	int nSocketPort;
	CString strIP;
	COPini cIni;
	cIni.SetFileName(DATA_PATH + strUnitName + ROBOT_PARA_INI);
	cIni.SetSectionName("BaseParam");
	cIni.ReadString("RobotMode", m_sRobotMode);
	cIni.ReadString("SocketPort", &nSocketPort);
	cIni.ReadString("FTPIP", strIP);
	m_pEstunRobot = new ESTUNRobotCtrl();
	if (!GetLocalDebugMark())
	{
		m_pEstunRobot->initSocket("192.168.60.2", nSocketPort, false);
		m_pEstunRobot->IfRecordInformation(true, true, false);
		m_pEstunRobot->ConfigFTP(strIP, "XiRobot", "123456");
	}
	api.connectToServer("192.168.60.63");
	Sleep(1000); // ��֤���ӳɹ� ����IO��������Ч 
	COPini opini;
	opini.SetFileName(DATA_PATH + "RobotA" + SYETEM_PARAM_FILE);
	opini.SetSectionName("EquipmentParam");

	opini.ReadString("FlatWeldMode", &m_nFlatWelderMode);
	opini.ReadString("FlatNormalWelderMode_W", m_sFlatNormalWelderMode_W);
	opini.ReadString("FlatNormalWelderMode_MM", m_sFlatNormalWelderMode_MM);
	opini.ReadString("FlatPulseWelderMode_W", m_sFlatPulseWelderMode_W);
	opini.ReadString("FlatPulseWelderMode_MM", m_sFlatPulseWelderMode_MM);

	opini.ReadString("VerWeldMode ", &m_nVerWelderMode);
	opini.ReadString("VerNormalWelderMode_W", m_sVerNormalWelderMode_W);
	opini.ReadString("VerNormalWelderMode_MM", m_sVerNormalWelderMode_MM);
	opini.ReadString("VerPulseWelderMode_W", m_sVerPulseWelderMode_W);
	opini.ReadString("VerPulseWelderMode_MM", m_sVerPulseWelderMode_MM);

	//opini.ReadString("WelderType", &g_bWelderType);
}

EstunAdaptor::~EstunAdaptor()
{
}

bool EstunAdaptor::getWelderMode(E_WELD_SEAM_TYPE eWeldSeamType, CString& sWelderMode_W, CString& sWelderMode_MM)
{
	switch (eWeldSeamType)
	{
	case E_FLAT_SEAM:
	case E_PLAT_MULTIPLE:
	case E_PLAT_GROOVE:
		switch (m_nFlatWelderMode)
		{
		case 0:
			sWelderMode_W = m_sFlatNormalWelderMode_W;
			sWelderMode_MM = m_sFlatNormalWelderMode_MM;
			return true;
		case 1:
			sWelderMode_W = m_sFlatPulseWelderMode_W;
			sWelderMode_MM = m_sFlatPulseWelderMode_MM;
			return true;
		}
		XUI::MesBox::PopError("�����ڵ�ƽ������ģʽ��{0}", m_nFlatWelderMode);
		return false;
	case E_STAND_SEAM:
	case E_STAND_MULTIPLE:
	case E_STAND_GROOVE:
		switch (m_nVerWelderMode)
		{
		case 0:
			sWelderMode_W = m_sVerNormalWelderMode_W;
			sWelderMode_MM = m_sVerNormalWelderMode_MM;
			return true;
		case 1:
			sWelderMode_W = m_sVerPulseWelderMode_W;
			sWelderMode_MM = m_sVerPulseWelderMode_MM;
			return true;
		}
		XUI::MesBox::PopError("�����ڵ���������ģʽ��{0}", m_nVerWelderMode);
		return false;
	}
	XUI::MesBox::PopError("�����ڵĺ������ͣ�{0}", int(eWeldSeamType));
	return false;
}

//���������Ϣ+
void  EstunAdaptor::cleanAlarm()
{
	int sRt;
	sRt = m_pEstunRobot->ErrClear_Py();
	if (sRt < 0)
	{
		Sleep(500);
		m_pEstunRobot->ErrClear_Py();
		//XiMessageBox("�������ʧ��");
	}
}

void EstunAdaptor::SetSysMode(int mode)
{
	int sRt;
	sRt = m_pEstunRobot->SetSysMode(mode);
	if (sRt < 0)
	{
		//XiMessageBox("����ϵͳģʽʧ��");
		Sleep(200);
		m_pEstunRobot->SetSysMode(mode);
	}
}

void EstunAdaptor::SetTpSpeed(int speed)//����TPʾ�����ϵ��ٶ�%
{
	int sRt;
	sRt = m_pEstunRobot->SetTpSpeed_Py(speed);
}
void EstunAdaptor::LoadUserProgramer(const char* projName, const char* progName)
{
	m_pEstunRobot->LoadUserProgramer_Py(projName, progName);
}
//���õ�ǰ���� !+
bool EstunAdaptor::SetRobotToolNo(int nToolNo)
{
	CString ToolName;
	ToolName.Format("TOOL%d", nToolNo);
	//�������˹���Ϊ1ʱ���ù���1ʧ�� 
	if (m_pEstunRobot->SetToolNum("TOOL99", 1) != 0 || m_pEstunRobot->SetToolNum(ToolName, 1) != 0)
	{
		XiMessageBox("��˹�ٻ��������ù���ʧ��");
		return false;
	}
	return true;
}

//��ѯ�����˵�ǰ����Ƕ�(�����ⲿ��) +
double EstunAdaptor::GetCurrentDegree(int nAxisNo)
{
	double adPos[10] = { 0.0 };
	m_pEstunRobot->GetCurAxis_Fast(adPos);
	return adPos[nAxisNo];
}
//��ȡ������Ϣ	+		
BOOL EstunAdaptor::GetToolData(UINT unToolNo, double adRobotToolData[6])
{
	char ToolName[20];
	memset(ToolName, '\0', 20 * sizeof(char));
	sprintf_s(ToolName, "TOOL%d", unToolNo);
	if (0 != m_pEstunRobot->GetToolDate(ToolName, adRobotToolData))
	{
		return FALSE;
	}
	return TRUE;
}

void EstunAdaptor::SetPosVar(int nIndex, T_ROBOT_COORS tRobotCoors, int config[7], int scoper)
{
	int cnfg[7] = { 0 };
	double adPosVar[8] = { tRobotCoors.dX,tRobotCoors.dY,tRobotCoors.dZ,tRobotCoors.dRX,tRobotCoors.dRY,tRobotCoors.dRZ,tRobotCoors.dBX,tRobotCoors.dBY };
	double PosVar[8] = { adPosVar[0] ,adPosVar[1],adPosVar[2],adPosVar[5],adPosVar[4],adPosVar[3],adPosVar[6],adPosVar[7] };
	if (m_pEstunRobot->LoadUserProgramer_Py("Xi_Robot", "MOVL") != 0 || m_pEstunRobot->SetPosVar_Py(nIndex, PosVar, cnfg, scoper) != 0)
	{
		XiMessageBox("��˹�ٻ���������P����ʧ��");
		//return false;
	}
	//return true;
}
void EstunAdaptor::SetPosVar(int nIndex, T_ANGLE_PULSE tRobotPulse, int scoper)
{
	double PosVar[8] = {
		tRobotPulse.nSPulse * m_tAxisUnit.dSPulse,tRobotPulse.nLPulse * m_tAxisUnit.dLPulse,
		tRobotPulse.nUPulse * m_tAxisUnit.dUPulse,tRobotPulse.nRPulse * m_tAxisUnit.dRPulse,
		tRobotPulse.nBPulse * m_tAxisUnit.dBPulse,tRobotPulse.nTPulse * m_tAxisUnit.dTPulse,0,0 };
	int config[7] = { 0 };
	if (m_pEstunRobot->LoadUserProgramer_Py("Xi_Robot", "MOVL") != 0 || m_pEstunRobot->SetPosVar_Py(nIndex, PosVar, config, scoper, 1) != 0)
	{
		XUI::MesBox::PopOkCancel("��˹�ٻ�����{0}����P����ʧ��", m_nRobotNo);
		//return false;
	}
}

// ��˹�ٻ�����ר��: �ٶȱ�����д
void EstunAdaptor::SetMultiSpeed(UINT index, UINT count, double** speedVar)//���ô����ٶ�
{
	m_pEstunRobot->SetMultiSpeed(index, count, speedVar);
}
void EstunAdaptor::SetSpeed(int nIndex, double adSpeed[5]) // ���õ����ٶ�
{
	m_pEstunRobot->SetSpeed(nIndex, adSpeed);
}
void EstunAdaptor::SetWeaveDate(const char* name, ESTUN_WeaveDate WeaveDate, int scope)
{
	int SRt = m_pEstunRobot->SetWeaveDate(name, WeaveDate, scope);
	if (SRt < 0)
	{
		Sleep(200);
		m_pEstunRobot->SetWeaveDate(name, WeaveDate, scope);
	}
}
int EstunAdaptor::GetIntVar(int nIndex, char* cStrPreFix)
{
	int nValue;
	char cIntVarName[100] = { 0 };
	sprintf(cIntVarName, "%s%d", cStrPreFix, nIndex);
	//m_pRobotCtrl_Estun->GetIntVar(cIntVarName, nValue);
	m_pEstunRobot->GetIntVar_Py(cIntVarName, nValue);
	return nValue;
}

void EstunAdaptor::SetIntVar(int nIndex, int nValue, int score, char* cStrPreFix)
{
	char cIntVarName[100] = { 0 };
	sprintf(cIntVarName, "%s%d", cStrPreFix, nIndex);
	//m_pRobotCtrl_Estun->GetIntVar(cIntVarName, nValue);
	m_pEstunRobot->SetIntVar_Py(cIntVarName, nValue, score);
}

//����Real����
bool EstunAdaptor::SetRealVar(int nIndex, double value, char* cStrPreFix, int score)
{
	char cIntVarName[100] = { 0 };
	sprintf(cIntVarName, "%s%d", cStrPreFix, nIndex);
	//SetRealVar_Py(const char* name, double value, int score = 2);
	int sRt = m_pEstunRobot->SetRealVar_Py(cIntVarName, value, score);
	if (sRt < 0)
	{
		//XiMessageBox("���õ�����ѹ����");
		return false;
	}
	return true;
}

int EstunAdaptor::AxisPulseMove(int nAxisNo, long lDist, long lRobotSpd, WORD wCoorType, int nToolNo, long lCoordFrm)
{
	double dDist = lDist;
	dDist /= 2000; //����ת��Ϊ����
	if (lCoordFrm == 0)
	{
		lRobotSpd = lRobotSpd / 60;
	}
	else
	{
		lRobotSpd = lRobotSpd / 100;
	}
	if (nAxisNo < 8)
	{
		int config[7] = { 0 };//Ĭ��mode cfֵΪ��
		//m_pRobotCtrl_Estun->SetRobotToolNo(nToolNo,2);
		int sRt = m_pEstunRobot->MoveByJob(nAxisNo + 1, dDist, config, lRobotSpd, wCoorType, lCoordFrm);//lCoordFrm == 0 ֱ��
		if (sRt < 0)
		{
			XiMessageBox("MoveByJobʧ��");
			return -1;
		}
		return 1;
	}
	return -1;
}

void EstunAdaptor::PosMove(int nAxisNo, double dDist, long lRobotSpd, WORD wCoorType, int nToolNo, long lCoordFrm)
{
	int config[7]{0};
	bool bSelfCreate = false;
	if (NULL == config)
	{
		bSelfCreate = true;
		//config = new int[7];
		for (int i = 0; i < 7; i++) {
			config[i] = 0;
		}

	}
	if (lCoordFrm == 0)
	{
		lRobotSpd = lRobotSpd / 60;
	}
	else
	{
		lRobotSpd = lRobotSpd / 100;
	}
	if (nAxisNo < 8)
	{
		int sRt = m_pEstunRobot->MoveByJob(nAxisNo + 1, dDist, config, lRobotSpd, wCoorType, lCoordFrm);//lCoordFrm == 0 ֱ��
		if (sRt < 0)
		{
			XiMessageBox("MoveByJobʧ��");
		}
	}
	if (bSelfCreate)
	{
		//delete[] config;
	}
}

//ͨ���ƶ�����
void EstunAdaptor::MoveByJob(double* dRobotJointCoord, T_ROBOT_MOVE_SPEED tPulseMove, int nExternalAxleType, int nPVarType, CString JobName, int config[7])
{
	if (NULL == config)
	{
		for (int i = 0; i < 7; i++)
		{
			config[i] = 0;
		}
	}
	double dPos[9] = { 0,0,0,0,0,0,0,0,0 };
	dPos[0] = dRobotJointCoord[0];
	dPos[1] = dRobotJointCoord[1];
	dPos[2] = dRobotJointCoord[2];
	dPos[3] = dRobotJointCoord[5]; // ��˹�ٻ����� Rx��Rz ��д�ߵ�
	dPos[4] = dRobotJointCoord[4];
	dPos[5] = dRobotJointCoord[3];

	// ֱ�߲岹 N mm/min <=> N / 60 mm / s  ����:  6000mm/min  <=> 100mm/s	�� Tcp = 100
	// �ؽڲ岹 1000 <=> 10 <=> 10%			����: �ؽ��ٶ�3000 <=> 30%		�� Per = 30
	if (nPVarType == 0)
	{
		tPulseMove.dSpeed = tPulseMove.dSpeed / 60;
	}
	else if (nPVarType == 1)
	{
		tPulseMove.dSpeed = tPulseMove.dSpeed / 100;
	}

	double speed = tPulseMove.dSpeed;
	int nMoveType = PULSE_COORD;
	int sRt = m_pEstunRobot->MoveByJob(dPos, config, speed, 1, nMoveType);
	if (sRt < 0)
	{
		XiMessageBox("MoveByJobʧ��");
	}
}

bool EstunAdaptor::SetOutIOVar(int name, int value)
{
	int sRt = m_pEstunRobot->Set_O_Var_Py(name, value);
	if (0 > sRt)
	{
		return false;
	}
	return true;
}

bool EstunAdaptor::GetInputIOVar(int name, int& value)
{
	m_pEstunRobot->Get_I_Var_Py(name, value);
	return value != 0;
}



// ��˹�ٽǶ�����ר�ú�����ֻ�ܹؽڲ岹��
void EstunAdaptor::MoveByJobForAngle(double* adRobotAngleCoord, T_ROBOT_MOVE_SPEED tPulseMove, CString JobName)
{
	double dPos[9] = { 0,0,0,0,0,0,0,0,0 };
	//double dPos[8] = { 0,0,0,0,0,0,0,0};
	int config[7] = { 0 };
	dPos[0] = adRobotAngleCoord[0]; // ����Ƕ�
	dPos[1] = adRobotAngleCoord[1];
	dPos[2] = adRobotAngleCoord[2];
	dPos[3] = adRobotAngleCoord[3];
	dPos[4] = adRobotAngleCoord[4];
	dPos[5] = adRobotAngleCoord[5];
	dPos[6] = adRobotAngleCoord[6];
	dPos[7] = adRobotAngleCoord[7];

	// ֱ�߲岹 N mm/min <=> N / 60 mm / s  ����:  6000mm/min  <=> 100mm/s	�� Tcp = 100
	// �ؽڲ岹 1000 <=> 10 <=> 10%			����: �ؽ��ٶ�3000 <=> 30%		�� Per = 30
	double nSpeed = tPulseMove.dSpeed / 100;
	int nMoveType = PULSE_COORD;
	int sRt = m_pEstunRobot->MoveByJob(dPos, config, nSpeed, 1, nMoveType);
	if (sRt < 0)
	{
		Sleep(100);
		WriteLog("MoveByJobʧ��1");
		sRt = m_pEstunRobot->MoveByJob(dPos, config, nSpeed, 1, nMoveType);
		if (sRt < 0)
		{
			XiMessageBox("MOVByJobʧ��");
		}
	}
}


bool EstunAdaptor::EstunSendTrackData(const vector<T_ROBOT_COORS>& WeldPathPoint, const vector<vector<int>>& Cfg)
{
	int size = WeldPathPoint.size();
	E_ROB_POS* tmp = new E_ROB_POS[size]();
	int i = 0;
	for (; i < size; i++)
	{
		int cfg_Array[7];
		double cpos_Array[16];
		cfg_Array[0] = Cfg[i][0];
		cfg_Array[1] = Cfg[i][1];
		cfg_Array[2] = Cfg[i][2];
		cfg_Array[3] = Cfg[i][3];
		cfg_Array[4] = Cfg[i][4];
		cfg_Array[5] = Cfg[i][5];
		cfg_Array[6] = Cfg[i][6];
		cpos_Array[0] = WeldPathPoint.at(i).dX;
		cpos_Array[1] = WeldPathPoint.at(i).dY;
		cpos_Array[2] = WeldPathPoint.at(i).dZ;
		cpos_Array[3] = WeldPathPoint.at(i).dRZ;
		cpos_Array[4] = WeldPathPoint.at(i).dRY;
		cpos_Array[5] = WeldPathPoint.at(i).dRX;
		cpos_Array[6] = WeldPathPoint.at(i).dBX;
		cpos_Array[7] = WeldPathPoint.at(i).dBY;
		cpos_Array[8] = 0;
		cpos_Array[9] = 0;
		cpos_Array[10] = 0;
		cpos_Array[11] = 0;
		cpos_Array[12] = 0;
		cpos_Array[13] = 0;
		cpos_Array[14] = 0;
		cpos_Array[15] = 0;
		//E_ROB_POS tmpERP(cfg_Array, cpos_Array);
		std::cout << "cfg:"<< cfg_Array[0]<<"," << cfg_Array[1] << "," << cfg_Array[2] << "," << cfg_Array[3] << "," << cfg_Array[4] << "," << cfg_Array[5] << "," << cfg_Array[6] << "," << std::endl;
		
		E_ROB_POSCFG cfgtmp11(cfg_Array[0], cfg_Array[1], cfg_Array[2], cfg_Array[3], cfg_Array[4], cfg_Array[5], cfg_Array[6]);
		E_ROB_POS tmpERP(cfgtmp11, WeldPathPoint[i].dX, WeldPathPoint[i].dY, WeldPathPoint[i].dZ, WeldPathPoint[i].dRZ,
			WeldPathPoint[i].dRY, WeldPathPoint[i].dRX, WeldPathPoint[i].dBX, WeldPathPoint[i].dBY);

		/*tmp[i] = (Cfg[i], WeldPathPoint[i].dX, WeldPathPoint[i].dY, WeldPathPoint[i].dZ, WeldPathPoint[i].dRX,
			WeldPathPoint[i].dRY, WeldPathPoint[i].dRZ, WeldPathPoint[i].dBX, WeldPathPoint[i].dBY, 0, 0, 0, 0, 0, 0, 0, 0);
		E_ROB_POS(const E_ROB_POSCFG& cfg, double x, double y, double z, double a, double b, double c,
			double a7 = 0, double a8 = 0, double a9 = 0, double a10 = 0, double a11 = 0, double a12 = 0,
			double a13 = 0, double a14 = 0, double a15 = 0, double a16 = 0);	
			
			*/
		tmp[i] = tmpERP;

	}
	bool flag = api.E_setCPosToWeldBuffer(tmp, size); //�����������
	if (!flag)
	{
		Sleep(10);
		flag = api.E_setCPosToWeldBuffer(tmp, size); //�ٷ�һ��
		if (!flag) {
			Sleep(10);
			flag = api.E_setCPosToWeldBuffer(tmp, size); //�ٷ�������
			if (!flag) {
				std::cout << "error" << std::endl;
				XiMessageBox("���ݷ���ʧ��");
				delete[](tmp);
				return false;
			}
		}
		
	}
	delete[](tmp);
	return true;
}

bool EstunAdaptor::EstunTrackInit( const T_ROBOT_COORS& startPos,const CString& IP, const vector<int>& Cfg)
{
	ESString myIp(IP);
	bool state = api.connectToServer(myIp, true);
	while (false == state) {
		Sleep(25);
		state = api.connectToServer(myIp, true);
	}
	if (state == false) {
		return false;
	}
	api.E_clearWeldBuffer();

	if (Cfg.size() < 7)
	{
		return false;
	}
	E_ROB_POSCFG cfgtmp22(Cfg[0], Cfg[1], Cfg[2], Cfg[3], Cfg[4], Cfg[5], Cfg[6]);

	E_ROB_POS EstartPos(cfgtmp22, startPos.dX, startPos.dY, startPos.dZ, startPos.dRZ, startPos.dRY, startPos.dRX, startPos.dBX, startPos.dBY, 0, 0, 0, 0, 0, 0, 0, 0);
	
	while (true) {
		//��ѯ�������Ƿ�׼���ý�����ʼ��
		if (1 == api.E_getFirstCposReadyFlag())
		{
			//������ʼ��
			state = api.E_setFirstCPosToWeld(EstartPos);
			break;
		}
		else {
			Sleep(25);
		}
	}
	return state;
}

bool EstunAdaptor::EstunTrackStop()
{
	bool res = api.E_setFinishFlagToWeld(); // �в����汾���Բ�����111
	if (res != true) {
		return false;
	}
	return res;
}

bool EstunAdaptor::EstunClearWeldBuffer()
{
	return api.E_clearWeldBuffer();
}


//�����ֲٺг��� 
int EstunAdaptor::CallJob(char JobName[24])
{
	if (0 > m_pEstunRobot->LoadUserProgramer_Py("Xi_Robot", JobName)) {
		return -2; //���ز��ɹ�
	};
	int sRt = m_pEstunRobot->Prog_startRun_Py();
	if (sRt < 0)
	{
		int sRt = m_pEstunRobot->Prog_startRun_Py();
		if (sRt < 0)
		{
			XUI::MesBox::PopOkCancel("���μ��س��� {0} ʧ��1", JobName);
			return -1; //���ò��ɹ�
		}
	}
	return 0;
}

int EstunAdaptor::CallJob(CString sJobName)
{
	if (0 > m_pEstunRobot->LoadUserProgramer_Py("Xi_Robot", sJobName)) {
		return -2; //���ز��ɹ�
	}
	if (0 > m_pEstunRobot->Prog_startRun_Py()) {
		return -1; //���ò��ɹ�
	}
	return 0;
}

//ֱ�������˶�����
void EstunAdaptor::MoveByJob(T_ROBOT_COORS tRobotJointCoord, T_ROBOT_MOVE_SPEED tPulseMove, int nExternalAxleType, CString JobName, int config[7])////Ĭ�� modeֵCfֵΪ��
{
	if (NULL == config)
	{
		int cnfg[7] = { 0 };
		config = cnfg;
	}
	double dPos[9] = { 0,0,0,0,0,0,0,0,0 };
	dPos[0] = tRobotJointCoord.dX;
	dPos[1] = tRobotJointCoord.dY;
	dPos[2] = tRobotJointCoord.dZ;
	dPos[3] = tRobotJointCoord.dRZ; // ��˹�ٻ����� Rx��Rz ��д�ߵ�
	dPos[4] = tRobotJointCoord.dRY;
	dPos[5] = tRobotJointCoord.dRX;
	dPos[6] = tRobotJointCoord.dBX;
	dPos[7] = tRobotJointCoord.dBY;
	dPos[8] = tRobotJointCoord.dBZ;

	double speed = tPulseMove.dSpeed / 60;
	int nMoveType = ROBOT_COORD;
	int sRt = m_pEstunRobot->MoveByJob(dPos, config, speed, 1, nMoveType);
	if (sRt < 0)
	{
		Sleep(100);
		WriteLog("MoveByJobʧ��");
		sRt = m_pEstunRobot->MoveByJob(dPos, config, speed, 1, nMoveType);
		if (sRt < 0)
		{
			XiMessageBox("MoveByJobʧ��");
		}
	}
}
//�ؽ������˶�����
void EstunAdaptor::MoveByJob(T_ANGLE_PULSE tRobotJointCoord, T_ROBOT_MOVE_SPEED tPulseMove, int nExternalAxleType, CString JobName)
{
	// Pulse => Angle
	double adRobotAngle[9] = { 0.0 };
	adRobotAngle[0] = tRobotJointCoord.nSPulse * m_tAxisUnit.dSPulse;
	adRobotAngle[1] = tRobotJointCoord.nLPulse * m_tAxisUnit.dLPulse;
	adRobotAngle[2] = tRobotJointCoord.nUPulse * m_tAxisUnit.dUPulse;
	adRobotAngle[3] = tRobotJointCoord.nRPulse * m_tAxisUnit.dRPulse;
	adRobotAngle[4] = tRobotJointCoord.nBPulse * m_tAxisUnit.dBPulse;
	adRobotAngle[5] = tRobotJointCoord.nTPulse * m_tAxisUnit.dTPulse;
	adRobotAngle[6] = GetPositionDis();//tRobotJointCoord.lBXPulse * m_tAxisUnit.dTPulse;
	adRobotAngle[7] = tRobotJointCoord.lBYPulse * m_tAxisUnit.dTPulse;
	MoveByJobForAngle(adRobotAngle, tPulseMove, JobName);
}

T_ROBOT_MOVE_INFO EstunAdaptor::PVarToRobotMoveInfo(int nVarNo, T_ANGLE_PULSE tPulse, T_ROBOT_MOVE_SPEED tSpeed, int nMoveType, UINT unToolNum, UINT unUserNo, UINT unPosture)
{
	T_ROBOT_MOVE_INFO tRtnVal;
	tRtnVal.tCoord.var_type = MP_VAR_P;
	tRtnVal.tCoord.var_no = nVarNo;
	tRtnVal.tCoord.val.p.dtype = MP_PULSE_COORD;
	tRtnVal.tCoord.val.p.uf_no = unUserNo;
	tRtnVal.tCoord.val.p.tool_no = unToolNum;
	tRtnVal.tCoord.val.p.fig_ctrl = unPosture;
	tRtnVal.tCoord.val.p.data[0] = tPulse.nSPulse;
	tRtnVal.tCoord.val.p.data[1] = tPulse.nLPulse;
	tRtnVal.tCoord.val.p.data[2] = tPulse.nUPulse;
	tRtnVal.tCoord.val.p.data[3] = tPulse.nRPulse;
	tRtnVal.tCoord.val.p.data[4] = tPulse.nBPulse;
	tRtnVal.tCoord.val.p.data[5] = tPulse.nTPulse;
	tRtnVal.tCoord.val.p.data[6] = GetPositionDis();// 0;
	tRtnVal.tCoord.val.p.data[7] = 0;
	tRtnVal.nMoveType = nMoveType;
	tRtnVal.tSpeed = tSpeed;
	return tRtnVal;
}

T_ROBOT_MOVE_INFO EstunAdaptor::PVarToRobotMoveInfo(int nVarNo, T_ROBOT_COORS tCoord, T_ROBOT_MOVE_SPEED tSpeed, int nMoveType, UINT unToolNum, UINT unUserNo, UINT unPosture)
{
	T_ANGLE_PULSE tRefPulse = T_ANGLE_PULSE();
	T_ROBOT_MOVE_INFO tRtnVal;
	tRtnVal.tCoord.var_type = MP_VAR_P;
	tRtnVal.tCoord.var_no = nVarNo;
	tRtnVal.tCoord.val.p.dtype = MP_ROBO_COORD;
	tRtnVal.tCoord.val.p.uf_no = unUserNo;
	tRtnVal.tCoord.val.p.tool_no = unToolNum;
	tRtnVal.tCoord.val.p.fig_ctrl = unPosture;
	tRtnVal.tCoord.val.p.data[0] = long(tCoord.dX * 1000);
	tRtnVal.tCoord.val.p.data[1] = long(tCoord.dY * 1000);
	tRtnVal.tCoord.val.p.data[2] = long(tCoord.dZ * 1000);
	tRtnVal.tCoord.val.p.data[3] = long(tCoord.dRX * 10000);
	tRtnVal.tCoord.val.p.data[4] = long(tCoord.dRY * 10000);
	tRtnVal.tCoord.val.p.data[5] = long(tCoord.dRZ * 10000);
	tRtnVal.tCoord.val.p.data[6] = GetPositionDis();// 0;
	tRtnVal.tCoord.val.p.data[7] = 0;
	tRtnVal.nMoveType = nMoveType;
	tRtnVal.tSpeed = tSpeed;
	//if (m_nRobotBrand == 1)
	{
		tRtnVal.tPulse.var_type = MP_VAR_P;
		tRtnVal.tPulse.var_no = nVarNo;
		tRtnVal.tPulse.val.p.dtype = MP_PULSE_COORD;
 		tRtnVal.tPulse.val.p.uf_no = unUserNo;
		tRtnVal.tPulse.val.p.tool_no = unToolNum;
		tRtnVal.tPulse.val.p.fig_ctrl = unPosture;
		tRtnVal.tPulse.val.p.data[0] = tRefPulse.nSPulse;
		tRtnVal.tPulse.val.p.data[1] = tRefPulse.nLPulse;
		tRtnVal.tPulse.val.p.data[2] = tRefPulse.nUPulse;
		tRtnVal.tPulse.val.p.data[3] = tRefPulse.nRPulse;
		tRtnVal.tPulse.val.p.data[4] = tRefPulse.nBPulse;
		tRtnVal.tPulse.val.p.data[5] = tRefPulse.nTPulse;
		tRtnVal.tPulse.val.p.data[6] = 0;
		tRtnVal.tPulse.val.p.data[7] = 0;
	}
	return tRtnVal;
}

//��˹�ٻ����� ����modeֵ,�������ͺţ�50B��20 ������
void EstunAdaptor::CalModeValue(T_ANGLE_PULSE tReferPluse, const char* RobotMode, int config[7])
{
	double Axis[6];
	Axis[0] = tReferPluse.nSPulse * m_tAxisUnit.dSPulse;
	Axis[1] = tReferPluse.nLPulse * m_tAxisUnit.dLPulse;
	Axis[2] = tReferPluse.nUPulse * m_tAxisUnit.dUPulse;
	Axis[3] = tReferPluse.nRPulse * m_tAxisUnit.dRPulse;
	Axis[4] = tReferPluse.nBPulse * m_tAxisUnit.dBPulse;
	Axis[5] = tReferPluse.nTPulse * m_tAxisUnit.dTPulse;
	m_pEstunRobot->ModeValue(Axis, RobotMode, config);
}


bool EstunAdaptor::SetMoveValue(std::vector<T_ROBOT_MOVE_INFO> vtMoveInfo, bool bSafeMove, bool bUsePB)
{
	//��˹�ٻ����ˣ�����CONTIMOVANY1
	//	INT20	 		:	�˶��ܵ���
	//	S1    -   S10	:	�ٶȱ���
	//	INT1  -  INT10	:	�˶���ʽ 0 MOVL 1 MOVJ
	//	INT30 -  INT39	:	�������� 0 mP	1 mJP
	//	mP1   -  mP10	:	�˶�ֱ����������
	//	mJP1  -  mJP10	:	�˶��ؽ���������
	T_ROBOT_MOVE_INFO tRobotMoveInfo;
	ESTUN_CPOS tEstunCPos;
	ESTUN_APOS tEstunAPos;
	ESTUN_SPEED tEstunSpeed;
	int nMaxTrackPtnSum = 10;
	int nTrackPtnSum = vtMoveInfo.size();

	if ((nTrackPtnSum > nMaxTrackPtnSum)) // �ǰ�˹�ٻ����� �������� ����false
	{
		return false;
	}

	T_MIX_VAR tMixVar;
	memset(&tMixVar, 0, sizeof(tMixVar));
	tMixVar.INT_Name = (int*)malloc(sizeof(int) * (nMaxTrackPtnSum * 2 + 1)); // �������� �� �˶����� ����INT���� ���ܵ���
	tMixVar.INT_Value = (int*)malloc(sizeof(int) * (nMaxTrackPtnSum * 2 + 1)); // �������� �� �˶����� ����INT���� ���ܵ���
	tMixVar.CPOS_Name = (int*)malloc(sizeof(int) * nMaxTrackPtnSum);
	tMixVar.CPOS_Value = (ESTUN_CPOS*)malloc(sizeof(ESTUN_CPOS) * nMaxTrackPtnSum);
	tMixVar.APOS_Name = (int*)malloc(sizeof(int) * nMaxTrackPtnSum);
	tMixVar.APOS_Value = (ESTUN_APOS*)malloc(sizeof(ESTUN_APOS) * nMaxTrackPtnSum);
	tMixVar.SPEED_Name = (int*)malloc(sizeof(int) * nMaxTrackPtnSum);
	tMixVar.SPEED_Value = (ESTUN_SPEED*)malloc(sizeof(ESTUN_SPEED) * nMaxTrackPtnSum);

	// INT20:�˶��ܵ���
	tMixVar.INT_Name[tMixVar.INT_Count] = 20;
	tMixVar.INT_Value[tMixVar.INT_Count] = nTrackPtnSum;
	tMixVar.INT_Count++;

	for (int i = 0; i < nTrackPtnSum; i++) // ��˹�ٻ����˱�����Ŵ�1
	{
		tRobotMoveInfo = vtMoveInfo[i];
		// INT1-INT10:�˶���ʽ 0 MOVL 1 MOVJ
		int nMoveType = tRobotMoveInfo.nMoveType;
		tMixVar.INT_Name[tMixVar.INT_Count] = i + 1;
		tMixVar.INT_Value[tMixVar.INT_Count] = nMoveType;
		tMixVar.INT_Count++;

		// INT30-INT39:�������� 0 mP	1 mJP
		int nCoordType = (MP_ROBO_COORD == tRobotMoveInfo.tCoord.val.p.dtype) ? 0 : 1;
		tMixVar.INT_Name[tMixVar.INT_Count] = i + 30;
		tMixVar.INT_Value[tMixVar.INT_Count] = nCoordType;
		tMixVar.INT_Count++;

		if (MP_ROBO_COORD == tRobotMoveInfo.tCoord.val.p.dtype)
		{
			// mP1-mP10���˶�ֱ����������
			memset(&tEstunCPos, 0, sizeof(tEstunCPos));
			double Axis[6];
			Axis[0] = (double)tRobotMoveInfo.tPulse.val.p.data[0] * m_tAxisUnit.dSPulse;
			Axis[1] = (double)tRobotMoveInfo.tPulse.val.p.data[1] * m_tAxisUnit.dLPulse;
			Axis[2] = (double)tRobotMoveInfo.tPulse.val.p.data[2] * m_tAxisUnit.dUPulse;
			Axis[3] = (double)tRobotMoveInfo.tPulse.val.p.data[3] * m_tAxisUnit.dRPulse;
			Axis[4] = (double)tRobotMoveInfo.tPulse.val.p.data[4] * m_tAxisUnit.dBPulse;
			Axis[5] = (double)tRobotMoveInfo.tPulse.val.p.data[5] * m_tAxisUnit.dTPulse;
			int Config[7];
			m_pEstunRobot->ModeValue(Axis, m_sRobotMode, Config);
			tEstunCPos.mode = Config[0];
			tEstunCPos.cf1 = Config[1];
			tEstunCPos.cf2 = Config[2];
			tEstunCPos.cf3 = Config[3];
			tEstunCPos.cf4 = Config[4];
			tEstunCPos.cf5 = Config[5];
			tEstunCPos.cf6 = Config[6];
			tEstunCPos.X = (double)tRobotMoveInfo.tCoord.val.p.data[0] / 1000.0;
			tEstunCPos.Y = (double)tRobotMoveInfo.tCoord.val.p.data[1] / 1000.0;
			tEstunCPos.Z = (double)tRobotMoveInfo.tCoord.val.p.data[2] / 1000.0;
			tEstunCPos.A = (double)tRobotMoveInfo.tCoord.val.p.data[5] / 10000.0;
			tEstunCPos.B = (double)tRobotMoveInfo.tCoord.val.p.data[4] / 10000.0;
			tEstunCPos.C = (double)tRobotMoveInfo.tCoord.val.p.data[3] / 10000.0;
			tEstunCPos.Axis_7 = tRobotMoveInfo.tCoord.val.p.data[6];
			tEstunCPos.Axis_8 = tRobotMoveInfo.tCoord.val.p.data[7];
			tMixVar.CPOS_Name[tMixVar.CPOS_Count] = i + 1;
			tMixVar.CPOS_Value[tMixVar.CPOS_Count] = tEstunCPos;
			tMixVar.CPOS_Count++;
		}
		else
		{
			// mJP1-mJP10:�˶��ؽ���������
			tEstunAPos.A1 = (double)tRobotMoveInfo.tCoord.val.p.data[0] * m_tAxisUnit.dSPulse;
			tEstunAPos.A2 = (double)tRobotMoveInfo.tCoord.val.p.data[1] * m_tAxisUnit.dLPulse;
			tEstunAPos.A3 = (double)tRobotMoveInfo.tCoord.val.p.data[2] * m_tAxisUnit.dUPulse;
			tEstunAPos.A4 = (double)tRobotMoveInfo.tCoord.val.p.data[3] * m_tAxisUnit.dRPulse;
			tEstunAPos.A5 = (double)tRobotMoveInfo.tCoord.val.p.data[4] * m_tAxisUnit.dBPulse;
			tEstunAPos.A6 = (double)tRobotMoveInfo.tCoord.val.p.data[5] * m_tAxisUnit.dTPulse;
			tEstunAPos.A7 = (double)tRobotMoveInfo.tCoord.val.p.data[6];
			tEstunAPos.A8 = (double)tRobotMoveInfo.tCoord.val.p.data[7];
			tMixVar.APOS_Name[tMixVar.APOS_Count] = i + 1;
			tMixVar.APOS_Value[tMixVar.APOS_Count] = tEstunAPos;
			tMixVar.APOS_Count++;
		}

		// S1 - S10:	�ٶȱ���
		memset(&tEstunSpeed, 1, sizeof(tEstunSpeed));
		if (0 == nMoveType) // MOVL
		{
			tEstunSpeed.tcp = (double)tRobotMoveInfo.tSpeed.dSpeed / 60.0; // mm/min -> mm/s
			tEstunSpeed.per = 1;
		}
		else
		{
			tEstunSpeed.per = tRobotMoveInfo.tSpeed.dSpeed / 100; // 0.01% -> 1%	1000 => 10%
			tEstunSpeed.tcp = 1;
		}
		tEstunSpeed.exp_7 = 360.0;
		tEstunSpeed.exp_8 = 180.0;
		tEstunSpeed.ori = 360;
		tMixVar.SPEED_Name[tMixVar.SPEED_Count] = i + 1;
		tMixVar.SPEED_Value[tMixVar.SPEED_Count] = tEstunSpeed;
		tMixVar.SPEED_Count++;
	}

	// ���س��� ��д�����
	int nRst = m_pEstunRobot->LoadUserProgramer_Py("Xi_Robot", "CONTIMOVANY");
	if (nRst < 0)
	{
		WriteLog("һ�μ��س��� CONTIMOVANY ʧ��3");
		Sleep(500);
		nRst = m_pEstunRobot->LoadUserProgramer_Py("Xi_Robot", "CONTIMOVANY");
		//if (nRst < 0)
		int nFalt = 1;
		while (nRst < 0)
		{
			Sleep(1000);
			nRst = m_pEstunRobot->LoadUserProgramer_Py("Xi_Robot", "CONTIMOVANY");
			if (nRst < 0)
			{
				WriteLog("����CONTIMOVANY����ʧ�� ����%d", ++nFalt);
				//XiMessageBox("���μ��س��� CONTIMOVANY ʧ��3");
				//return false;
			}
		}
	}

	nRst = m_pEstunRobot->SetMultiVar_H(tMixVar);
	if (nRst < 0)
	{
		XiMessageBox("ESTUN:���ٻ췢����ʧ��!");
		return false;
	}

	free(tMixVar.INT_Name);
	free(tMixVar.INT_Value);
	free(tMixVar.CPOS_Name);
	free(tMixVar.CPOS_Value);
	free(tMixVar.APOS_Name);
	free(tMixVar.APOS_Value);
	free(tMixVar.SPEED_Name);
	free(tMixVar.SPEED_Value);
	return true;
}

int EstunAdaptor::MoveExAxisForLineScan(double dDist, int nSpeed) {
	int config[7] = { 0 };
	T_ANGLE_PULSE tReferPluse = GetCurrentPulse(); //��ѯ�����˵�ǰ�ؽ�����
	CalModeValue(tReferPluse, "8", config);
	PosMove(6, dDist, nSpeed, COORD_ABS); //COORD_ABS ����
	return 0;
}

bool EstunAdaptor::MoveToSafeHeight() {
	double dRobotInstallMode = m_nRobotInstallDir;// ��װ��1.0   ��װ��-1.0
	double dMinUpMoveDis = 100.0;
	double dBackHeightThreshold = 100.0;
	T_ANGLE_PULSE tCurPulse;
	T_ROBOT_COORS tCurCoord;
	T_ROBOT_COORS tTransCoord;
	T_ROBOT_COORS tTempCoord;
	BOOL isChanged = false; //tTempCoord��û���޸Ĺ�
	T_ROBOT_COORS tHomeCoord;
	T_ROBOT_MOVE_SPEED tPulseMove;

	tPulseMove.dSpeed = 1500;
	tCurPulse = GetCurrentPulse(); //��ǰ������
	tCurCoord = GetCurrentPos(); //��ǰֱ������
	RobotKinematics(m_tHomePulse, m_tTools.tGunTool, tHomeCoord); //��ȫλ�õ�����תΪֱ������
	RobotKinematics(tCurPulse, m_tTools.tGunTool, tTransCoord); //��ǰ����תΪֱ������
	// ��ȡֱ������Ͷ�ȡ�ؽ�����ת�����������ͬʱ�������˶�
	if (false == CompareCoords(tTransCoord, tCurCoord))
	{
		WriteLog("MoveToSafeHeight:��ȡ����ͼ�������������");
		XiMessageBoxOk("��ȡ����ͼ������겻ͬ���޷��Զ�̧ǹ��");
		return false;
	}

	double dMinBackGunHeight = tHomeCoord.dZ - (dBackHeightThreshold * dRobotInstallMode);
	// �߶�С��dMinBackGunHeightʱ ִ�в�����1��RY�ָ���׼45  2������̧��100mm��MOVL����̫С�ٶȷǳ��죩
	tTempCoord = tCurCoord;
	tTempCoord.dRX = tHomeCoord.dRX;
	tTempCoord.dRY = tHomeCoord.dRY;
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
		isChanged = true;
	}
	if (isChanged == TRUE) {
		int cnfg[7] = { 0 };
		CalModeValue(tCurPulse, "8", cnfg);
		MoveByJob(tTempCoord, tPulseMove, 0, "MOVL", cnfg);////Ĭ�� modeֵCfֵΪ��	
		CheckRobotDone();
		return CompareCoords(tTempCoord);
	}
	else {
		return true;
	}
}

bool EstunAdaptor::TeachMove(const std::vector<T_ANGLE_PULSE>& vtMeasurePulse, double dExAxlePos, const  std::vector<int>& vnMeasureType, int nTrigSigTime)
{

	//#ifndef SINGLE_ROBOT
	// �ⲿ���˶�
	if (0 != MoveExAxisForLineScan(dExAxlePos, 9000))
	{
		return false;
	}
	WorldCheckRobotDone();
	double dCurExPos = GetPositionDis();
	if (fabs(dExAxlePos - dCurExPos) > 5.0) // ��Ŀ��λ��������ֵ �ж�Ϊ�˶�ʧ��
	{
		XiMessageBox("�Զ�ʾ��:�ⲿ��δ�˶���ָ��λ��");
		return false;
	}
	//#endif // !SINGLE_ROBOT

	SendTeachMoveData(vtMeasurePulse, vnMeasureType, m_tPulseHighSpeed.dSpeed, m_tTeachSpeed.dSpeed, m_tPulseLowSpeed.dSpeed, nTrigSigTime);
	CallJob("TEACH");
	return true;
}
//bool EstunAdaptor::CleanGunH()
//{
//	//λ�ù���ʱ��̧ǹ�ٻص���ȫλ��
//	CHECK_BOOL_RETURN(CheckIsReadyRun());
//	double NowZ = GetCurrentPos(ROBOT_AXIS_Z);
//	if (NowZ < m_tRobotLimitation.drTableZ + 550)
//	{
//		PosMove(ROBOT_AXIS_Z, m_tRobotLimitation.drTableZ + 550.0, 500, COORD_ABS);
//		CheckRobotDone();
//		NowZ = GetCurrentPos(ROBOT_AXIS_Z);
//		if (NowZ != m_tRobotLimitation.drTableZ + 550.0)
//		{
//			XiMessageBox("��ǰλ�ù��ͣ��Զ�̧ǹʧ�ܣ����ֶ�̧ǹ");
//			return FALSE;
//		}
//	}
//	vector<T_ROBOT_COORS> vtRobotCoor;
//	MP_USR_VAR_INFO tUsrVarInfo;
//	vector<MP_USR_VAR_INFO> vtUsrVarInfo;
//	vector<T_ROBOT_MOVE_INFO> vtMoveInfo;
//	T_ROBOT_MOVE_INFO tRobotMoveInfo;
//	COPini opini;
//	opini.SetFileName("./data/RobotAndCar.ini");
//	opini.SetSectionName("ClearGunParam");
//
//	T_ANGLE_PULSE tCleanInitPlus;//��ǹ��־λ(�ؽ�)
//	T_ROBOT_COORS tCleanInitCoord; //��ǹ��־λ(ֱ��) 
//	opini.ReadString("CleanInitPlus.nSPulse", &tCleanInitPlus.nSPulse);
//	opini.ReadString("CleanInitPlus.nLPulse", &tCleanInitPlus.nLPulse);
//	opini.ReadString("CleanInitPlus.nUPulse", &tCleanInitPlus.nUPulse);
//	opini.ReadString("CleanInitPlus.nRPulse", &tCleanInitPlus.nRPulse);
//	opini.ReadString("CleanInitPlus.nBPulse", &tCleanInitPlus.nBPulse);
//	opini.ReadString("CleanInitPlus.nTPulse", &tCleanInitPlus.nTPulse);
//	RobotKinematics(tCleanInitPlus, m_tFirstTool, tCleanInitCoord);
//
//	T_ROBOT_COORS tClearShifting;   //��ǹƫ�ƾ���
//	T_ROBOT_COORS tCutSilkShifting; //��˿ƫ�ƾ���
//	opini.ReadString("ClearShifting.dX", &tClearShifting.dX);
//	opini.ReadString("ClearShifting.dY", &tClearShifting.dY);
//	opini.ReadString("ClearShifting.dZ", &tClearShifting.dZ);
//	opini.ReadString("CutSilkShifting.dX", &tCutSilkShifting.dX);
//	opini.ReadString("CutSilkShifting.dY", &tCutSilkShifting.dY);
//	opini.ReadString("CutSilkShifting.dZ", &tCutSilkShifting.dZ);
//
//	double realCleanerAngle;		//��ǹ��ʵ�ʰ�װ�Ƕ�
//	opini.ReadString("RealCleanerAngle", &realCleanerAngle);
//
//	//��ǹ��
//	T_ROBOT_COORS tClearGunPoint(tCleanInitCoord);
//	tClearGunPoint.dX = tClearGunPoint.dX + tClearShifting.dX;
//	tClearGunPoint.dY = tClearGunPoint.dY + tClearShifting.dY * SinD(realCleanerAngle) * m_nRobotInstallDir;
//	tClearGunPoint.dZ = tClearGunPoint.dZ + tClearShifting.dZ * m_nRobotInstallDir;
//
//	//��˿��
//	T_ROBOT_COORS tCutSilkPoint(tCleanInitCoord);
//	tCutSilkPoint.dX = tCutSilkPoint.dX + tCutSilkShifting.dX;
//	tCutSilkPoint.dY = tCutSilkPoint.dY + tCutSilkShifting.dY * SinD(realCleanerAngle) * m_nRobotInstallDir;
//	tCutSilkPoint.dZ = tCutSilkPoint.dZ + tCutSilkShifting.dZ * m_nRobotInstallDir;
//
//	//��ǹ���ɵ�(��)
//	T_ROBOT_COORS tClearGunExcessivePointBefor(tClearGunPoint);
//	tClearGunExcessivePointBefor.dX -= 4.0;
//
//	//��ǹ���ɵ�(��)
//	T_ROBOT_COORS tClearGunExcessivePoint(tClearGunExcessivePointBefor);
//	tClearGunExcessivePoint.dZ += 100;
//
//	//��˿���ɵ�
//	T_ROBOT_COORS tCutSilkExcssivePoint(tCutSilkPoint);
//	tCutSilkExcssivePoint.dZ += 100;
//
//	//��ǹ���ɵ�(��)
//	vtRobotCoor.push_back(tClearGunExcessivePoint);
//	//��ǹ���ɵ�(��)
//	vtRobotCoor.push_back(tClearGunExcessivePointBefor);
//	//��ǹ��
//	vtRobotCoor.push_back(tClearGunPoint);
//	//��˿���ɵ�
//	vtRobotCoor.push_back(tCutSilkExcssivePoint);
//	//��˿��
//	vtRobotCoor.push_back(tCutSilkPoint);
//	//��ȫλ��
//	tRobotMoveInfo = PVarToRobotMoveInfo(0, m_tHomePulse, m_tPulseHighSpeed, MOVJ);
//	vtMoveInfo.push_back(tRobotMoveInfo);
//	for (int i = 0; i < vtRobotCoor.size(); i++)
//	{
//		tRobotMoveInfo = PVarToRobotMoveInfo(i + 1, vtRobotCoor[i], m_tPulseHighSpeed, MOVL);
//		vtMoveInfo.push_back(tRobotMoveInfo);
//	}
//	SetMoveValue(vtMoveInfo);
//	CallJob("Clear");
//	CheckRobotDone();
//	return true;
//}
int EstunAdaptor::MoveToLineScanPos(T_ANGLE_PULSE tPulse, T_ROBOT_MOVE_SPEED tPulseMove)
{
	MoveByJob(tPulse, tPulseMove, m_nExternalAxleType, "MOVJ");
	CheckRobotDone();
	MoveExAxisForLineScan((double)tPulse.lBXPulse * m_tExternalAxle[0].dPulse, 12000);
	CheckRobotDone();
	return 0;
}
int EstunAdaptor::ContiMoveAny(const std::vector<T_ROBOT_MOVE_INFO>& vtRobotMoveInfo)
{
	SetMoveValue(vtRobotMoveInfo);
	CallJob("CONTIMOVANY");
	return 0;
}

bool EstunAdaptor::CalcWeaveWeldTrack(std::vector<T_ROBOT_COORS>& vtWeldTrack, const T_WELD_PARA& tWeldPara, E_WELD_SEAM_TYPE eWeldSeamType, double& dRealWeldSpeed)
{
	if (vtWeldTrack.size() < 2)
	{
		XUI::MesBox::PopInfo("���ӹ켣����{0} ����ں��켣ʧ�ܣ�");
		return false;
	}
	// �ްڶ�������˵�һ��
	if (0 >= tWeldPara.nWrapConditionNo || 1 == tWeldPara.tWeaveParam.nType)
	{
		dRealWeldSpeed = tWeldPara.WeldVelocity;
		return true;
	}
	int nOneLoopPtnNum = 2 == tWeldPara.tWeaveParam.nType ? 3 : 4; // һ�ΰڶ�ѭ���˶�����
	double dFreq = tWeldPara.tWeaveParam.dFreq;
	double dAmpL = tWeldPara.tWeaveParam.dAmpL;
	double dAmpR = tWeldPara.tWeaveParam.dAmpR;
	double dWeldDisEverySecond = tWeldPara.WeldVelocity / 60.0; // ÿ�뺸�ӷ����ƶ�����
	double dSingleMoveDis = dWeldDisEverySecond / ((double)nOneLoopPtnNum * dFreq);
	double dOneLoopMoveDis = 0.0;
//	CvPoint3D64f tWeaveOffsetDirL;
//	CvPoint3D64f tWeaveOffsetDirR;
	double dDisTeackInterval = TwoPointDis(vtWeldTrack[0].dX, vtWeldTrack[0].dY, vtWeldTrack[0].dZ, vtWeldTrack[1].dX, vtWeldTrack[1].dY, vtWeldTrack[1].dZ);
	int nSamplingInterval = dDisTeackInterval > dSingleMoveDis ? 1 : (int)(dSingleMoveDis / dDisTeackInterval) + 1;

	// �ڶ�ʵ���˶��ٶ�
	if (2 == tWeldPara.tWeaveParam.nType) // ���ǰ�
	{
		dOneLoopMoveDis = (sqrt(pow(dSingleMoveDis, 2) + pow(dAmpL, 2))) + (sqrt(pow(dSingleMoveDis, 2) + pow(dAmpR, 2))) +
			(sqrt(pow(dSingleMoveDis, 2) + pow(sqrt(pow(dAmpL, 2) + pow(dAmpR, 2)), 2)));
	}
	else if (3 == tWeldPara.tWeaveParam.nType) // L��
	{
		dOneLoopMoveDis = (sqrt(pow(dSingleMoveDis, 2) + pow(dAmpL, 2)) * 2.0) + (sqrt(pow(dSingleMoveDis, 2) + pow(dAmpR, 2)) * 2.0);
	}
	else
	{
		XUI::MesBox::PopInfo("�ڶ���ʽ{0}����ڶ�ѭ���˶�����ʧ�ܣ�", tWeldPara.tWeaveParam.nType);
		return false;
	}
	dRealWeldSpeed = dOneLoopMoveDis * 60.0;  // mm/min

	// �ڶ��켣���ֵ
	int nOrgNum = vtWeldTrack.size();
	std::vector<T_ROBOT_COORS> vtCoors(vtWeldTrack);
	vtWeldTrack.clear();
	int nCurIdx = 0;
	int nNextIdx = nSamplingInterval;
	T_ROBOT_COORS tCurCoord = vtCoors[nCurIdx];
	T_ROBOT_COORS tNextCoord = vtCoors[nNextIdx];
	T_ROBOT_COORS tNewCoord = tCurCoord;
	double dDis, dDisX, dDisY, dDisZ, dDisRx, dDisRy, dDisRz;

	vtWeldTrack.push_back(tNewCoord); // ��һ������
	while (true)
	{
		dDis = TwoPointDis(tCurCoord.dX, tCurCoord.dY, tCurCoord.dZ, tNextCoord.dX, tNextCoord.dY, tNextCoord.dZ);
		dDisX = tNextCoord.dX - tCurCoord.dX;
		dDisY = tNextCoord.dY - tCurCoord.dY;
		dDisZ = tNextCoord.dZ - tCurCoord.dZ;
		dDisRx = tNextCoord.dRX - tCurCoord.dRX;
		dDisRy = tNextCoord.dRY - tCurCoord.dRY;
		dDisRz = tNextCoord.dRZ - tCurCoord.dRZ;
		int nNewPtnNum = dDis / dSingleMoveDis;
		for (int i = 1; i <= nNewPtnNum; i++)
		{
			tNewCoord.dX = tCurCoord.dX + dDisX * dSingleMoveDis * (double)i / dDis;
			tNewCoord.dY = tCurCoord.dY + dDisY * dSingleMoveDis * (double)i / dDis;
			tNewCoord.dZ = tCurCoord.dZ + dDisZ * dSingleMoveDis * (double)i / dDis;
			tNewCoord.dRX = CalcPosture(tCurCoord.dRX, tNextCoord.dRX, dSingleMoveDis * (double)i / dDis);
			tNewCoord.dRY = CalcPosture(tCurCoord.dRY, tNextCoord.dRY, dSingleMoveDis * (double)i / dDis);
			tNewCoord.dRZ = CalcPosture(tCurCoord.dRZ, tNextCoord.dRZ, dSingleMoveDis * (double)i / dDis);
			vtWeldTrack.push_back(tNewCoord); // �ڶ������Ժ�����
		}
		nNextIdx += nSamplingInterval;
		if (nNextIdx >= nOrgNum)
		{
			break;
		}
		tCurCoord = tNewCoord;
		tNextCoord = vtCoors[nNextIdx];
	}
	vtWeldTrack.push_back(tNextCoord); // ���һ������

	// ��һ�� �� ���һ�� ��������ƫ��
	// ���ǰ� L R C ����
	// L��    L C R C ����
	int nOffsetPtnIdx = 0;
	for (int i = 1; i < vtWeldTrack.size() - 1; i++)
	{
		int nOffsetType = nOffsetPtnIdx % nOneLoopPtnNum;
		if (0 == nOffsetType) // ���ڶ�ƫ��
		{
			if (E_FLAT_SEAM == eWeldSeamType) // ƽ��
			{
				double dDirAngle = RzToDirAngle(vtWeldTrack[vtWeldTrack.size() / 2.0].dRZ); // ����ǰ�����̬Ӱ�캸�취�����(�������㷨�����δ����)
				vtWeldTrack[i].dX += dAmpL * CosD(dDirAngle);
				vtWeldTrack[i].dY += dAmpL * SinD(dDirAngle);
			}
			else if (E_STAND_SEAM == eWeldSeamType) // ����
			{
				double dDirAngle = RzToDirAngle(vtWeldTrack[0].dRZ); // ����ʹ�÷���Ǽ�������ƫ�Ʒ���
				double dOffsetAngle = dDirAngle + (45.0 * (double)m_nRobotInstallDir);
				vtWeldTrack[i].dX += dAmpL * CosD(dOffsetAngle);
				vtWeldTrack[i].dY += dAmpL * SinD(dOffsetAngle);
			}
		}
		else if (((2 == tWeldPara.tWeaveParam.nType) && (1 == nOffsetType)) ||	// ���ǰ��Ҳ�ƫ��
			((3 == tWeldPara.tWeaveParam.nType) && (2 == nOffsetType)))    // L���Ҳ�ƫ��
		{
			if (E_FLAT_SEAM == eWeldSeamType) // ƽ��
			{
				vtWeldTrack[i].dZ += dAmpR;
			}
			else if (E_STAND_SEAM == eWeldSeamType) // ����
			{
				double dDirAngle = RzToDirAngle(vtWeldTrack[0].dRZ); // ����ʹ�÷���Ǽ�������ƫ�Ʒ���
				double dOffsetAngle = dDirAngle - (45.0 * (double)m_nRobotInstallDir);
				vtWeldTrack[i].dX += dAmpR * CosD(dOffsetAngle);
				vtWeldTrack[i].dY += dAmpR * SinD(dOffsetAngle);
			}
		}
		nOffsetPtnIdx++;
	}

	FILE* pf = fopen("AA_Test.txt", "w");
	for (int i = 0; i < vtWeldTrack.size(); i++)
	{
		fprintf(pf, "%d%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf\n",
			i, vtWeldTrack[i].dX, vtWeldTrack[i].dY, vtWeldTrack[i].dZ, vtWeldTrack[i].dRX, vtWeldTrack[i].dRY, vtWeldTrack[i].dRZ);
	}
	fclose(pf);
	return true;
}

int EstunAdaptor::WeldMoveRobotWeave(const std::vector<T_ROBOT_COORS>& vtWeldPathPoints, const  std::vector<int>& vnPtnType, const T_WELD_PARA& tWeldPara, double dExAxlePos, E_WELD_SEAM_TYPE eWeldSeamType, bool bIsArcOn)
{
	// ���̱�����INT1 INT2 INT3 INT4 INT5 : �Ƿ��� �Ƿ񺸽� �Ƿ���� ��ͣ�� �ջ�ͣ��
	// ���������Real0 - Real7 : �� ���� ���� �ջ� ������ѹ��ע��˳��
	// ���̱�����WRAVE1 : �ڶ��ļ���
	// ���������S1 : �����˶��ٶ�
	// ���������mP1 - mP100 : ���ӹ켣����
	// ���������mP101 mP102 : ���ǹ켣����

	// ����100����Ĺ켣�����һ���켣�㲻ȫ������100����Ľ�ֹ����
	// �ް��Ƕ����İ��ǹ켣����Ϊ���һ��������
	bool bHasWrap = false;
	bool bHasWeld = false;
	std::vector<T_ROBOT_COORS> vtAllCoord(vtWeldPathPoints);
	std::vector<T_ROBOT_COORS> vtWrapCoord(0);
	for (int i = vtAllCoord.size() - 1; i >= 0; i--)
	{
		vtAllCoord[i].dBX = dExAxlePos;
		if (E_WELD_WRAP & vnPtnType[i])
		{
			vtWrapCoord.push_back(vtAllCoord[i]);
			vtAllCoord.pop_back(); //vtAllCoord.erase(vtAllCoord.end() - 1);
		}
	}
	int nWeldPtnNum = vtAllCoord.size();
	int nWrapPtnNum = vtWrapCoord.size();
	bHasWeld = nWeldPtnNum > 0 ? true : false;
	bHasWrap = nWrapPtnNum > 0 ? true : false;

	if (nWeldPtnNum < 0 || nWeldPtnNum > WELD_TRACK_MAX_NUM)
	{
		XUI::MesBox::PopInfo("���ӹ켣���쳣 {0}", nWeldPtnNum);
		return -1;
	}
	if (0 != nWrapPtnNum && WRAP_TRACK_MAX_NUM != nWrapPtnNum)
	{
		XUI::MesBox::PopInfo("���ǹ켣���쳣 {0}", nWeldPtnNum);
		return -1;
	}
	// ���ɲ����ͺ����˶�����
	if (false == SendWeldProgram(vtAllCoord.size(), eWeldSeamType))
	{
		XUI::MesBox::PopInfo("���ӳ�����ʧ�ܣ� ������{0}", nWeldPtnNum);
		return -1;
	}
	T_ROBOT_COORS tLastCoord = vtAllCoord[nWeldPtnNum - 1]; // ֻ�а��ǹ켣ʱ��������
	if (0 == nWrapPtnNum)
	{
		vtWrapCoord.push_back(tLastCoord);
		vtWrapCoord.push_back(tLastCoord);
	}
	else
	{
		swap(vtWrapCoord[0], vtWrapCoord[1]);
	}
	for (int i = vtAllCoord.size(); i < WELD_TRACK_MAX_NUM; i++)
	{
		vtAllCoord.push_back(tLastCoord);
	}
	int nAllPtnNum = vtAllCoord.size();
	vtAllCoord.insert(vtAllCoord.end(), vtWrapCoord.begin(), vtWrapCoord.end());
	nAllPtnNum = vtAllCoord.size();

	T_ROBOT_MOVE_SPEED tPosMove;
	tPosMove.dSpeed = tWeldPara.WeldVelocity;
	tPosMove.dACC = 100;
	tPosMove.dDEC = 100;

	SetIntVar(1, bIsArcOn, 2);
	SetIntVar(2, bHasWeld, 2);
	SetIntVar(3, bHasWrap, 2);
	SetIntVar(4, tWeldPara.dStartWaitTime * 1000, 2); // s -> ms
	SetIntVar(5, tWeldPara.dStopWaitTime * 1000, 2);  // s -> ms
	if (E_STAND_SEAM == eWeldSeamType)
	{
		SetIntVar(6, 0);
	}
	else
	{
		SetIntVar(6, m_nFlatWelderMode/*tWeldPara.nWeldMethod*/);
	}


	SetRealVar(0, tWeldPara.dStartArcCurrent, "REAL", 1);
	SetRealVar(1, tWeldPara.dStartArcVoltage, "REAL", 1);
	SetRealVar(2, tWeldPara.dTrackCurrent, "REAL", 1);
	SetRealVar(3, tWeldPara.dTrackVoltage, "REAL", 1);
	SetRealVar(4, tWeldPara.dWrapCurrentt1, "REAL", 1);
	SetRealVar(5, tWeldPara.dWrapVoltage1, "REAL", 1);
	SetRealVar(6, tWeldPara.dStopArcCurrent, "REAL", 1);
	SetRealVar(7, tWeldPara.dStopArcVoltage, "REAL", 1);

	ESTUN_WeaveDate tWeaveDate;
	tWeaveDate.Type = tWeldPara.tWeaveParam.nType;
	tWeaveDate.Freq = tWeldPara.tWeaveParam.dFreq;
	tWeaveDate.Amp_L = tWeldPara.tWeaveParam.dAmpL;
	tWeaveDate.Amp_R = tWeldPara.tWeaveParam.dAmpR;
	tWeaveDate.StopTime_L = tWeldPara.tWeaveParam.dTimeL * 1000;
	tWeaveDate.StopTime_C = tWeldPara.tWeaveParam.dTimeC * 1000;
	tWeaveDate.StopTime_R = tWeldPara.tWeaveParam.dTimeR * 1000;
	tWeaveDate.RotAngle_X = tWeldPara.tWeaveParam.dRotAngleX;
	tWeaveDate.RotAngle_Z = tWeldPara.tWeaveParam.dRotAngleZ;
	tWeaveDate.DelayType_L = tWeldPara.tWeaveParam.nDelateTypeL;
	tWeaveDate.DelayType_C = tWeldPara.tWeaveParam.nDelateTypeC;
	tWeaveDate.DelayType_R = tWeldPara.tWeaveParam.nDelateTypeR;
	SetWeaveDate("WEAVE1", tWeaveDate, 2);

	int config[7] = { 0 };
	if (false == SetMultiPosVar(1, vtAllCoord, tPosMove, "WELD", config))
	{
		XiMessageBoxOk("SetMultiPosVar���ӹ켣����ʧ�ܣ�");
		return -1;
	}

	CallJob("WELD");
	return 0;
}

bool EstunAdaptor::SendWeldProgram(int nWeldTrackNum, E_WELD_SEAM_TYPE eWeldSeamType)
{
	CString sWelderMode_W;		//����ģʽ����W
	CString sWelderMode_MM;		//����ģʽ����MM
	if (!getWelderMode(eWeldSeamType, sWelderMode_W, sWelderMode_MM))
		return false;

	CString sLocalFile;
	CString sRemoteFile = ".\\WELD.erp";
	sLocalFile.Format("LocalFiles\\OutputFiles\\%s\\ESTUN_FILE\\WELD.erp", m_strRobotName);
	FILE* pf = fopen(sLocalFile.GetBuffer(), "w");
	if (NULL == pf) return false;

	fprintf(pf, "// InstParseVersion: V1.02\n");
	fprintf(pf, "Start:\n");
	fprintf(pf, "SetWelderMode{" + sWelderMode_W + "," + sWelderMode_MM + "}\n");
	fprintf(pf, "Wait{T = 50}\n");
	fprintf(pf, "SetWelderMode{" + sWelderMode_W + "," + sWelderMode_MM + "}\n");
	fprintf(pf, "MovL{P=t_l.P1,V=t_l.S1}\n");
	fprintf(pf, "IF t_p.INT1.value == 1 THEN\n");
	fprintf(pf, "    ARC_SET{A=t_g.REAL0,V=t_g.REAL1,H=0}\n");
	fprintf(pf, "    ARC_ON{}\n");
	fprintf(pf, "    Wait{T=t_p.INT4}\n");
	fprintf(pf, "ENDIF\n");
	fprintf(pf, "/*Weld*/\n");
	fprintf(pf, "IF t_p.INT2.value == 1 THEN\n");
	fprintf(pf, "    ARC_SET{A=t_g.REAL2,V=t_g.REAL3,H=0}\n");
	for (int i = 0; i < nWeldTrackNum; i++)
	{
		fprintf(pf, "    ARC_LW{P=t_l.P%d,V=t_l.S1,B=\"RELATIVE\",C=t_s.C10,WEAVE=t_p.WEAVE1}\n", i + 1);

	}
	fprintf(pf, "ENDIF\n");
	fprintf(pf, "IF t_p.INT3.value == 1 THEN\n");
	fprintf(pf, "    ARC_SET{A=t_g.REAL4,V=t_g.REAL5,H=0}\n");
	fprintf(pf, "    SetCartDyn{Acc=100,Dec=100,Jerk=100,OriAcc=3,OriDec=3,OriJerk=100,Torq=100}\n");
	fprintf(pf, "    ARC_Line{P=t_l.P499,V=t_l.S1,B=\"FINE\"}\n");
	fprintf(pf, "    ARC_Line{P=t_l.P500,V=t_l.S1,B=\"FINE\"}\n");
	fprintf(pf, "    SetCartDyn{Acc=100,Dec=100,Jerk=100,OriAcc=100,OriDec=100,OriJerk=100,Torq=100}\n");
	fprintf(pf, "ENDIF\n");
	fprintf(pf, "/*ArcOff*/\n");
	fprintf(pf, "IF t_p.INT1.value == 1 THEN\n");
	fprintf(pf, "    ARC_SET{A=t_g.REAL6,V=t_g.REAL7,H=0}\n");
	fprintf(pf, "    Wait{T=t_p.INT5}\n");
	fprintf(pf, "    ARC_OFF{}\n");
	fprintf(pf, "ENDIF\n");
	//fprintf(pf, "MovL{P=t_l.P500,V=t_l.S1}\n");
	fprintf(pf, "End;\n");
	fclose(pf);

	// �ϴ������ļ�
	int sRt = m_pEstunRobot->ConnectFtp();
	if (sRt < 0) return false;
	sRt = m_pEstunRobot->UploadFile(sRemoteFile, sLocalFile);
	if (sRt < 0)
	{
		sRt = m_pEstunRobot->DisConnectFtp();
		XUI::MesBox::PopInfo("�ϴ������ļ�:{0} ʧ�ܣ�", sLocalFile);
		return false;
	}
	sRt = m_pEstunRobot->DisConnectFtp();
	if (sRt < 0) return false;

	return true;
}

int EstunAdaptor::WeldMoveTriangleWeave(const std::vector<T_ROBOT_COORS>& vtWeldPathPoints, const  std::vector<int>& vnPtnType, const T_WELD_PARA& tWeldPara, double dExAxlePos, E_WELD_SEAM_TYPE eWeldSeamType, bool bIsArcOn)
{
	// ���̱�����INT1 INT2 INT3 INT4 INT5 : �Ƿ��� �Ƿ񺸽� �Ƿ���� ��ͣ�� �ջ�ͣ��
	// ���̱�����INT6 INT7 INT8	: ���ǰ� �м� ��� �Ҳ�ͣ��ʱ�䵥λms
	// ���������Real0 - Real7 : �� ���� ���� �ջ� ������ѹ��ע��˳��
	// ���������S1 : �����˶��ٶ�
	// ���������mP1 - mP298 : ���ӹ켣����
	// ���������mP299 mP300 : ���ǹ켣����

	// !!! ����298����Ĺ켣�����һ���켣�㲻ȫ������298����Ľ�ֹ���� !!!
	// !!! �ް��Ƕ����İ��ǹ켣����Ϊ���һ��������						!!!

	// ���� ���ӹ켣 �� ���ǹ켣
	bool bHasWrap = false;
	bool bHasWeld = false;
	std::vector<T_ROBOT_COORS> vtAllCoord(vtWeldPathPoints);
	std::vector<T_ROBOT_COORS> vtWrapCoord(0);
	for (int i = vtAllCoord.size() - 1; i >= 0; i--)
	{
		vtAllCoord[i].dBX = dExAxlePos;
		if (E_WELD_WRAP & vnPtnType[i])
		{
			vtWrapCoord.push_back(vtAllCoord[i]);
			vtAllCoord.pop_back(); //vtAllCoord.erase(vtAllCoord.end() - 1);
		}
	}

	double dRealWeldSpeed = tWeldPara.WeldVelocity;
	if (false == CalcWeaveWeldTrack(vtAllCoord, tWeldPara, eWeldSeamType, dRealWeldSpeed))
	{
		return -1;
	}

	int nWeldPtnNum = vtAllCoord.size();
	int nWrapPtnNum = vtWrapCoord.size();
	bHasWeld = nWeldPtnNum > 0 ? true : false;
	bHasWrap = nWrapPtnNum > 0 ? true : false;

	if (nWeldPtnNum < 0 || nWeldPtnNum > WELD_TRACK_MAX_NUM)
	{
		XUI::MesBox::PopInfo("���ӹ켣���쳣 {0}", nWeldPtnNum);
		return -1;
	}
	if (0 != nWrapPtnNum && WRAP_TRACK_MAX_NUM != nWrapPtnNum)
	{
		XUI::MesBox::PopInfo("���ǹ켣���쳣 {0}", nWeldPtnNum);
		return -1;
	}

	// ���ɲ����ͺ����˶�����
	if (false == SendWeldTriangleWeaveProgram(vtAllCoord.size(), eWeldSeamType))
	{
		XUI::MesBox::PopInfo("���ӳ�����ʧ�ܣ� ������{0}", nWeldPtnNum);
		return -1;
	}

	// ��ȫ��ָ����ǹ켣
	T_ROBOT_COORS tLastCoord = vtAllCoord[nWeldPtnNum - 1]; // ֻ�а��ǹ켣ʱ��������
	if (0 == nWrapPtnNum)
	{
		vtWrapCoord.push_back(tLastCoord);
		vtWrapCoord.push_back(tLastCoord);
	}
	else
	{
		swap(vtWrapCoord[0], vtWrapCoord[1]);
	}

	// ʹ�����һ�����ӹ켣�㣬��ȫ��󺸽ӹ켣����
	for (int i = vtAllCoord.size(); i < WELD_TRACK_MAX_NUM; i++)
	{
		vtAllCoord.push_back(tLastCoord);
	}
	int nAllPtnNum = vtAllCoord.size();
	vtAllCoord.insert(vtAllCoord.end(), vtWrapCoord.begin(), vtWrapCoord.end());
	nAllPtnNum = vtAllCoord.size();

	T_ROBOT_MOVE_SPEED tPosMove;
	tPosMove.dSpeed = dRealWeldSpeed;
	tPosMove.dACC = 100;
	tPosMove.dDEC = 100;

	SetIntVar(1, bIsArcOn, 2);
	SetIntVar(2, bHasWeld, 2);
	SetIntVar(3, bHasWrap, 2);
	SetIntVar(4, tWeldPara.dStartWaitTime * 1000, 2); // s -> ms
	SetIntVar(5, tWeldPara.dStopWaitTime * 1000, 2);  // s -> ms
	SetIntVar(6, tWeldPara.tWeaveParam.dTimeC * 1000, 2);  // s -> ms
	SetIntVar(7, tWeldPara.tWeaveParam.dTimeL * 1000, 2);  // s -> ms
	SetIntVar(8, tWeldPara.tWeaveParam.dTimeR * 1000, 2);  // s -> ms

	SetRealVar(0, tWeldPara.dStartArcCurrent, "REAL", 1);
	SetRealVar(1, tWeldPara.dStartArcVoltage, "REAL", 1);
	SetRealVar(2, tWeldPara.dTrackCurrent, "REAL", 1);
	SetRealVar(3, tWeldPara.dTrackVoltage, "REAL", 1);
	SetRealVar(4, tWeldPara.dWrapCurrentt1, "REAL", 1);
	SetRealVar(5, tWeldPara.dWrapVoltage1, "REAL", 1);
	SetRealVar(6, tWeldPara.dStopArcCurrent, "REAL", 1);
	SetRealVar(7, tWeldPara.dStopArcVoltage, "REAL", 1);

	// ���ͺ��ӹ켣�����ļ�
	int config[7] = { 0 };
	if (false == SetMultiPosVar(1, vtAllCoord, tPosMove, "WELD_TRIANGLE", config))
	{
		XiMessageBoxOk("SetMultiPosVar���ӹ켣����ʧ�ܣ�");
		return -1;
	}

	CallJob("WELD_TRIANGLE");
	return 0;
}

int EstunAdaptor::WeldMoveLWeave(const std::vector<T_ROBOT_COORS>& vtWeldPathPoints, const  std::vector<int>& vnPtnType, const T_WELD_PARA& tWeldPara, double dExAxlePos, E_WELD_SEAM_TYPE eWeldSeamType, bool bIsArcOn)
{
	// ���̱�����INT1 INT2 INT3 INT4 INT5 : �Ƿ��� �Ƿ񺸽� �Ƿ���� ��ͣ�� �ջ�ͣ��
	// ���̱�����INT6 INT7 INT8 INT9 : L�� �м� ��� �м� �Ҳ�ͣ��ʱ�䵥λms
	// ���������Real0 - Real7 : �� ���� ���� �ջ� ������ѹ��ע��˳��
	// ���������S1 : �����˶��ٶ�
	// ���������mP1 - mP298 : ���ӹ켣����
	// ���������mP299 mP300 : ���ǹ켣����

	// !!! ����298����Ĺ켣�����һ���켣�㲻ȫ������298����Ľ�ֹ���� !!!
	// !!! �ް��Ƕ����İ��ǹ켣����Ϊ���һ��������						!!!

	// ���� ���ӹ켣 �� ���ǹ켣
	bool bHasWrap = false;
	bool bHasWeld = false;
	std::vector<T_ROBOT_COORS> vtAllCoord(vtWeldPathPoints);
	std::vector<T_ROBOT_COORS> vtWrapCoord(0);
	for (int i = vtAllCoord.size() - 1; i >= 0; i--)
	{
		vtAllCoord[i].dBX = dExAxlePos;
		if (E_WELD_WRAP & vnPtnType[i])
		{
			vtWrapCoord.push_back(vtAllCoord[i]);
			vtAllCoord.pop_back();
		}
	}

	double dRealWeldSpeed = tWeldPara.WeldVelocity;
	if (false == CalcWeaveWeldTrack(vtAllCoord, tWeldPara, eWeldSeamType, dRealWeldSpeed))
	{
		return -1;
	}

	int nWeldPtnNum = vtAllCoord.size();
	int nWrapPtnNum = vtWrapCoord.size();
	bHasWeld = nWeldPtnNum > 0 ? true : false;
	bHasWrap = nWrapPtnNum > 0 ? true : false;


	if (nWeldPtnNum < 0 || nWeldPtnNum > WELD_TRACK_MAX_NUM)
	{
		XUI::MesBox::PopInfo("���ӹ켣���쳣 {0}", nWeldPtnNum);
		return -1;
	}
	if (0 != nWrapPtnNum && WRAP_TRACK_MAX_NUM != nWrapPtnNum)
	{
		XUI::MesBox::PopInfo("���ǹ켣���쳣 {0}", nWeldPtnNum);
		return -1;
	}

	// ���ɲ����ͺ����˶�����
	if (false == SendWeldLWeaveProgram(vtAllCoord.size(), eWeldSeamType))
	{
		XUI::MesBox::PopInfo("���ӳ�����ʧ�ܣ� ������{0}", nWeldPtnNum);
		return -1;
	}

	// ��ȫ��ָ����ǹ켣
	T_ROBOT_COORS tLastCoord = vtAllCoord[nWeldPtnNum - 1]; // ֻ�а��ǹ켣ʱ��������
	if (0 == nWrapPtnNum)
	{
		vtWrapCoord.push_back(tLastCoord);
		vtWrapCoord.push_back(tLastCoord);
	}
	else
	{
		swap(vtWrapCoord[0], vtWrapCoord[1]);
	}

	// ʹ�����һ�����ӹ켣�㣬��ȫ��󺸽ӹ켣����
	for (int i = vtAllCoord.size(); i < WELD_TRACK_MAX_NUM; i++)
	{
		vtAllCoord.push_back(tLastCoord);
	}
	vtAllCoord.insert(vtAllCoord.end(), vtWrapCoord.begin(), vtWrapCoord.end());

	T_ROBOT_MOVE_SPEED tPosMove;
	tPosMove.dSpeed = dRealWeldSpeed;
	tPosMove.dACC = 100;
	tPosMove.dDEC = 100;

	SetIntVar(1, bIsArcOn, 2);
	SetIntVar(2, bHasWeld, 2);
	SetIntVar(3, bHasWrap, 2);
	SetIntVar(4, tWeldPara.dStartWaitTime * 1000, 2); // s -> ms
	SetIntVar(5, tWeldPara.dStopWaitTime * 1000, 2);  // s -> ms
	SetIntVar(6, tWeldPara.tWeaveParam.dTimeC * 1000, 2);  // s -> ms
	SetIntVar(7, tWeldPara.tWeaveParam.dTimeL * 1000, 2);  // s -> ms
	SetIntVar(8, tWeldPara.tWeaveParam.dTimeC * 1000, 2);  // s -> ms
	SetIntVar(9, tWeldPara.tWeaveParam.dTimeR * 1000, 2);  // s -> ms

	SetRealVar(0, tWeldPara.dStartArcCurrent, "REAL", 1);
	SetRealVar(1, tWeldPara.dStartArcVoltage, "REAL", 1);
	SetRealVar(2, tWeldPara.dTrackCurrent, "REAL", 1);
	SetRealVar(3, tWeldPara.dTrackVoltage, "REAL", 1);
	SetRealVar(4, tWeldPara.dWrapCurrentt1, "REAL", 1);
	SetRealVar(5, tWeldPara.dWrapVoltage1, "REAL", 1);
	SetRealVar(6, tWeldPara.dStopArcCurrent, "REAL", 1);
	SetRealVar(7, tWeldPara.dStopArcVoltage, "REAL", 1);

	// ���ͺ��ӹ켣�����ļ�
	int config[7] = { 0 };
	if (false == SetMultiPosVar(1, vtAllCoord, tPosMove, "WELD_L_WEAVE", config))
	{
		XiMessageBoxOk("SetMultiPosVar���ӹ켣����ʧ�ܣ�");
		return -1;
	}

	CallJob("WELD_L_WEAVE");
	return 0;
}

int EstunAdaptor::WeldMove(const std::vector<T_ROBOT_COORS>& vtWeldPathPoints, const  std::vector<int>& vnPtnType, const T_WELD_PARA& tWeldPara, double dExAxlePos, E_WELD_SEAM_TYPE eWeldSeamType, bool bIsArcOn)
{
	if ((0 >= tWeldPara.nWrapConditionNo) || (1 == tWeldPara.tWeaveParam.nType)) // �ްڶ� �� �����˵�һ��
	{
		return WeldMoveRobotWeave(vtWeldPathPoints, vnPtnType, tWeldPara, dExAxlePos, eWeldSeamType, bIsArcOn);
	}
	else if ((0 < tWeldPara.nWrapConditionNo) && (2 == tWeldPara.tWeaveParam.nType)) // �ǻ��������ǰ�
	{
		return WeldMoveTriangleWeave(vtWeldPathPoints, vnPtnType, tWeldPara, dExAxlePos, eWeldSeamType, bIsArcOn);
	}
	else if ((0 < tWeldPara.nWrapConditionNo) && (3 == tWeldPara.tWeaveParam.nType)) // �ǻ�����L��
	{
		return WeldMoveLWeave(vtWeldPathPoints, vnPtnType, tWeldPara, dExAxlePos, eWeldSeamType, bIsArcOn);
	}
	else
	{
		XUI::MesBox::PopInfo("�ڶ�������{0}(-1-255) �ڶ�����{1}(1-3) ����!", tWeldPara.nWrapConditionNo, tWeldPara.tWeaveParam.nType);
		return -1;
	}
}

double EstunAdaptor::CalcPosture(double dPosture1, double dPosture2, double dRatio)
{
	dPosture1 = fmod(dPosture1, 360.0);
	dPosture2 = fmod(dPosture2, 360.0);
	dPosture1 = dPosture1 > 180.0 ? dPosture1 - 360.0 : dPosture1;
	dPosture1 = dPosture1 < -180.0 ? dPosture1 + 360.0 : dPosture1;
	dPosture2 = dPosture2 > 180.0 ? dPosture2 - 360.0 : dPosture2;
	dPosture2 = dPosture2 < -180.0 ? dPosture2 + 360.0 : dPosture2;

	double dErr = dPosture2 - dPosture1;
	dErr = dErr > 180.0 ? dPosture2 - 360.0 - dPosture1 : dErr;
	dErr = dErr < -180.0 ? dPosture2 + 360.0 - dPosture1 : dErr;

	double dPosture = dPosture1 + dErr * dRatio;
	dPosture = dPosture > 180.0 ? dPosture - 360.0 : dPosture;
	dPosture = dPosture < -180.0 ? dPosture + 360.0 : dPosture;

	return dPosture;
}

bool EstunAdaptor::SendWeldTriangleWeaveProgram(int nWeldTrackNum, E_WELD_SEAM_TYPE eWeldSeamType)
{
	CString sWelderMode_W;		//����ģʽ����W
	CString sWelderMode_MM;		//����ģʽ����MM
	if (!getWelderMode(eWeldSeamType, sWelderMode_W, sWelderMode_MM))
		return false;

	CString sLocalFile;
	CString sRemoteFile = ".\\WELD_TRIANGLE.erp";
	sLocalFile.Format("LocalFiles\\OutputFiles\\%s\\ESTUN_FILE\\WELD_TRIANGLE.erp", m_strRobotName);
	FILE* pf = fopen(sLocalFile.GetBuffer(), "w");
	if (NULL == pf) return false;

	fprintf(pf, "// InstParseVersion: V1.02\n");
	fprintf(pf, "Start:\n");
	fprintf(pf, "SetWelderMode{" + sWelderMode_W + "," + sWelderMode_MM + "}\n");
	fprintf(pf, "Wait{T = 50}\n");
	fprintf(pf, "SetWelderMode{" + sWelderMode_W + "," + sWelderMode_MM + "}\n");
	fprintf(pf, "/*ArcOn*/\n");
	fprintf(pf, "MovL{P=t_l.P1,V=t_l.S1}\n");
	fprintf(pf, "IF t_p.INT1.value == 1 THEN\n");
	fprintf(pf, "    ARC_SET{A=t_g.REAL0,V=t_g.REAL1,H=0}\n");
	fprintf(pf, "    ARC_ON{}\n");
	fprintf(pf, "    Wait{T=t_p.INT4}\n");
	fprintf(pf, "ENDIF\n");
	fprintf(pf, "/*Weld*/\n");
	fprintf(pf, "IF t_p.INT2.value == 1 THEN\n");
	fprintf(pf, "    ARC_SET{A=t_g.REAL2,V=t_g.REAL3,H=0}\n");
	for (int i = 0; i < nWeldTrackNum; i++)
	{
		fprintf(pf, "    ARC_Line{P=t_l.P%d,V=t_l.S1,B=\"FINE\"}\n", i + 1);
		fprintf(pf, "    Wait{T=t_p.INT%d}\n", i % 3 + 6); // ͣ��ʱ�����ͱ������� 6 7 8
	}
	fprintf(pf, "ENDIF\n");
	fprintf(pf, "/*Wrap*/\n");
	fprintf(pf, "IF t_p.INT3.value == 1 THEN\n");
	fprintf(pf, "    ARC_SET{A=t_g.REAL4,V=t_g.REAL5,H=0}\n");
	fprintf(pf, "    SetCartDyn{Acc=100,Dec=100,Jerk=100,OriAcc=3,OriDec=3,OriJerk=100,Torq=100}\n");
	fprintf(pf, "    ARC_Line{P=t_l.P499,V=t_l.S1,B=\"FINE\"}\n");
	fprintf(pf, "    ARC_Line{P=t_l.P500,V=t_l.S1,B=\"FINE\"}\n");
	fprintf(pf, "    SetCartDyn{Acc=100,Dec=100,Jerk=100,OriAcc=100,OriDec=100,OriJerk=100,Torq=100}\n");
	fprintf(pf, "ENDIF\n");
	fprintf(pf, "/*ArcOff*/\n");
	fprintf(pf, "IF t_p.INT1.value == 1 THEN\n");
	fprintf(pf, "    ARC_SET{A=t_g.REAL6,V=t_g.REAL7,H=0}\n");
	fprintf(pf, "    Wait{T=t_p.INT5}\n");
	fprintf(pf, "    ARC_OFF{}\n");
	fprintf(pf, "ENDIF\n");
	fprintf(pf, "MovL{P=t_l.P300,V=t_l.S1}\n");
	fprintf(pf, "End;\n");
	fclose(pf);

	// �ϴ������ļ�
	int sRt = m_pEstunRobot->ConnectFtp();
	if (sRt < 0) return false;
	sRt = m_pEstunRobot->UploadFile(sRemoteFile, sLocalFile);
	if (sRt < 0)
	{
		sRt = m_pEstunRobot->DisConnectFtp();
		XUI::MesBox::PopInfo("�ϴ������ļ�:{0} ʧ�ܣ�", sLocalFile);
		return false;
	}
	sRt = m_pEstunRobot->DisConnectFtp();
	if (sRt < 0) return false;
	return true;
}

bool EstunAdaptor::SendWeldLWeaveProgram(int nWeldTrackNum, E_WELD_SEAM_TYPE eWeldSeamType)
{
	CString sWelderMode_W;		//����ģʽ����W
	CString sWelderMode_MM;		//����ģʽ����MM
	if (!getWelderMode(eWeldSeamType, sWelderMode_W, sWelderMode_MM))
		return false;

	CString sLocalFile;
	CString sRemoteFile = ".\\WELD_L_WEAVE.erp";
	sLocalFile.Format("LocalFiles\\OutputFiles\\%s\\ESTUN_FILE\\WELD_TRIANGLE.erp", m_strRobotName);
	FILE* pf = fopen(sLocalFile.GetBuffer(), "w");
	if (NULL == pf) return false;

	fprintf(pf, "// InstParseVersion: V1.02\n");
	fprintf(pf, "Start:\n");
	fprintf(pf, "SetWelderMode{" + sWelderMode_W + "," + sWelderMode_MM + "}\n");
	fprintf(pf, "Wait{T = 50}\n");
	fprintf(pf, "SetWelderMode{" + sWelderMode_W + "," + sWelderMode_MM + "}\n");
	fprintf(pf, "/*ArcOn*/\n");
	fprintf(pf, "MovL{P=t_l.P1,V=t_l.S1}\n");
	fprintf(pf, "IF t_p.INT1.value == 1 THEN\n");
	fprintf(pf, "    ARC_SET{A=t_g.REAL0,V=t_g.REAL1,H=0}\n");
	fprintf(pf, "    ARC_ON{}\n");
	fprintf(pf, "    Wait{T=t_p.INT4}\n");
	fprintf(pf, "ENDIF\n");
	fprintf(pf, "/*Weld*/\n");
	fprintf(pf, "IF t_p.INT2.value == 1 THEN\n");
	fprintf(pf, "    ARC_SET{A=t_g.REAL2,V=t_g.REAL3,H=0}\n");
	for (int i = 0; i < nWeldTrackNum; i++)
	{
		fprintf(pf, "    ARC_Line{P=t_l.P%d,V=t_l.S1,B=\"FINE\"}\n", i + 1);
		fprintf(pf, "    Wait{T=t_p.INT%d}\n", i % 4 + 6); // ͣ��ʱ�����ͱ������� 6 7 8 9
	}
	fprintf(pf, "ENDIF\n");
	fprintf(pf, "/*Wrap*/\n");
	fprintf(pf, "IF t_p.INT3.value == 1 THEN\n");
	fprintf(pf, "    ARC_SET{A=t_g.REAL4,V=t_g.REAL5,H=0}\n");
	fprintf(pf, "    SetCartDyn{Acc=100,Dec=100,Jerk=100,OriAcc=3,OriDec=3,OriJerk=100,Torq=100}\n");
	fprintf(pf, "    ARC_Line{P=t_l.P499,V=t_l.S1,B=\"FINE\"}\n");
	fprintf(pf, "    ARC_Line{P=t_l.P500,V=t_l.S1,B=\"FINE\"}\n");
	fprintf(pf, "    SetCartDyn{Acc=100,Dec=100,Jerk=100,OriAcc=100,OriDec=100,OriJerk=100,Torq=100}\n");
	fprintf(pf, "ENDIF\n");
	fprintf(pf, "/*ArcOff*/\n");
	fprintf(pf, "IF t_p.INT1.value == 1 THEN\n");
	fprintf(pf, "    ARC_SET{A=t_g.REAL6,V=t_g.REAL7,H=0}\n");
	fprintf(pf, "    Wait{T=t_p.INT5}\n");
	fprintf(pf, "    ARC_OFF{}\n");
	fprintf(pf, "ENDIF\n");
	fprintf(pf, "MovL{P=t_l.P500,V=t_l.S1}\n");
	fprintf(pf, "End;\n");
	fclose(pf);

	// �ϴ������ļ�
	int sRt = m_pEstunRobot->ConnectFtp();
	if (sRt < 0) return false;
	sRt = m_pEstunRobot->UploadFile(sRemoteFile, sLocalFile);
	if (sRt < 0)
	{
		sRt = m_pEstunRobot->DisConnectFtp();
		XUI::MesBox::PopInfo("�ϴ������ļ�:{0} ʧ�ܣ�", sLocalFile);
		return false;
	}
	sRt = m_pEstunRobot->DisConnectFtp();
	if (sRt < 0) return false;
	return true;
}

int EstunAdaptor::MoveExAxisFun(double dDist, int nSpeed, int nMoveXYZAxisNo, double dMaxExSpeed)
{
	if (dMaxExSpeed < 0.0)
	{
		dMaxExSpeed = nSpeed;
	}
	if (nMoveXYZAxisNo > 3 || nMoveXYZAxisNo < 1)
	{
		XiMessageBoxOk("�ⲿ�����ô���! �˶�ʧ��");
		return -1;
	}
	double dMaxExAxlePos = (double)m_tExternalAxle[nMoveXYZAxisNo - 1].lMaxPulseNum * m_tExternalAxle[nMoveXYZAxisNo - 1].dPulse;
	double dMinExAxlePos = (double)m_tExternalAxle[nMoveXYZAxisNo - 1].lMinPulseNum * m_tExternalAxle[nMoveXYZAxisNo - 1].dPulse;
	if (dDist > dMaxExAxlePos || dDist < dMinExAxlePos)
	{
		XUI::MesBox::PopInfo("�ⲿ��{0} Ŀ��λ��{1} �����趨����{2} - {3}!", nMoveXYZAxisNo, dDist, dMinExAxlePos, dMaxExAxlePos);
		return -1;
	}

	T_ROBOT_COORS tRobotCoors = GetCurrentPos();

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
		XiMessageBoxOk("�ⲿ��δ���������飡");
		return -1;
	}
	int config[7] = { 0,0,0,0,0,0,0 };
	T_ROBOT_MOVE_SPEED tPulseMove(nSpeed/* / 60*/, 20, 20);
	MoveByJob(tRobotCoors, tPulseMove, m_nExternalAxleType, "MOVL", config);
	return 0;
}

int EstunAdaptor::XYZAxisNoToSoftAxisNo(int nMoveXYZAxisNo)
{
	//int nSoftAxisNo = -1;
	//for (int nAxis = 0; nAxis < m_pvpMotorDriver->size(); nAxis++)
	//{
	//	CUnitDriver* pUnitDriver = m_pvpMotorDriver->at(nAxis);
	//	if (nMoveXYZAxisNo != labs(pUnitDriver->GetMotorParam().nToRobotDir))
	//	{
	//		if (nAxis == m_pvpMotorDriver->size() - 1)
	//		{
	//			XiMessageBoxOk("�ⲿ�����ô���! �˶�ʧ��");
	//			return -1;
	//		}
	//	}
	//	else
	//	{
	//		nSoftAxisNo = pUnitDriver->GetMotorParam().nSoftAxisNo;
	//		break;
	//	}
	//}
	int nSoftAxisNo = nMoveXYZAxisNo;
	return nSoftAxisNo;
}


bool EstunAdaptor::CheckIsReadyRun()
{
	/*if (true == CheckInIO("RobotEmg"))
	{
		XiMessageBoxOk("�뽫�����˼�ͣ��λ�����");
		return false;
	}
	if (true == CheckInIO("TotalEmg"))
	{
		XiMessageBoxOk("�뽫�ܼ�ͣ��λ�����");
		return false;
	}
	m_pEstunRobot->ErrClear_Py();
	m_pEstunRobot->SetSysMode(2);

	for (int i = 0; i < 30; i++) {
		if (1 == m_pEstunRobot->GetServoSts()) {
			break;
		}
		Sleep(100);
	}
	if (1 != m_pEstunRobot->GetServoSts())
	{
		XiMessageBoxOk("ESTUN�������ϵ�ʧ�ܣ�");
	}*/
	return true;
}

double EstunAdaptor::GetExPositionDis(int nMoveXYZAxisNo)
{
	double dCurPos = 0.0;
	T_ROBOT_COORS tRobotCoors = GetCurrentPos();
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
		XiMessageBoxOk("�ⲿ��δ���������飡");
		return -1;
	}
	return dCurPos;
}

E_ERROR_STATE EstunAdaptor::CheckRobotDone(int nDelayTime)
{
	int retValue = 0;
	while (1)
	{
		retValue = m_pEstunRobot->CheckDone();
		if (1 == retValue)
		{
			Sleep(200);
			retValue = m_pEstunRobot->CheckDone();
		}
		if (1 != retValue)
		{
			Sleep(200);
			retValue = m_pEstunRobot->CheckDone();
			if (1 != retValue)
			{
				return RUNNING_STATUS_UNKNOWN_ERROR;
			}
		}
	}
	return RUNNING_STATUS_SUCCESS;
}

bool EstunAdaptor::WorldIsRunning() {
	int nRst = m_pEstunRobot->CheckDone_Fast();
	if (1 != nRst) {
		WriteLog("WorldIsRunning: %d", nRst);
		return false;
	}
	return true;
}

void EstunAdaptor::WorldCheckRobotDone(int nDelayTime)
{
	CheckRobotDone();
}

void EstunAdaptor::HoldOn()
{
	m_pEstunRobot->Stop();
}

void EstunAdaptor::HoldOff()
{
}


//��˹�ٻ�����һ�����ô������� ֱ��
bool EstunAdaptor::SetMultiPosVar(UINT unIndex, std::vector<T_ROBOT_COORS> vtRobotJointCoord, T_ROBOT_MOVE_SPEED tPosMove, int config[7], UINT unToolNum)
{
	if (NULL == config)
	{
		for (int i = 0; i < 7; i++)
		{
			config[i] = 0;
		}
	}
	UINT unCount = vtRobotJointCoord.size();
	UINT size = unCount;
	if (size < WELD_TRACK_MAX_NUM)
	{
		size = WELD_TRACK_MAX_NUM;
	}
	int sRt;
	T_ROBOT_POSVAR* var = new T_ROBOT_POSVAR[size];
	for (UINT i = 0; i < unCount; i++)
	{
		var[i].p[0] = vtRobotJointCoord[i].dX;
		var[i].p[1] = vtRobotJointCoord[i].dY;
		var[i].p[2] = vtRobotJointCoord[i].dZ;
		var[i].p[3] = vtRobotJointCoord[i].dRZ;
		var[i].p[4] = vtRobotJointCoord[i].dRY;
		var[i].p[5] = vtRobotJointCoord[i].dRX;
		var[i].ExP[0] = 0;
		var[i].ExP[1] = 0;
		var[i].speed = tPosMove.dSpeed / 60;
		var[i].Ex_7_speed = 1;
		var[i].Ex_8_speed = 1;
		var[i].mode = config[0];
		var[i].cf1 = config[1];
		var[i].cf2 = config[2];
		var[i].cf3 = config[3];
		var[i].cf4 = config[4];
		var[i].cf5 = config[5];
		var[i].cf6 = config[6];
	};
	if (unCount < WELD_TRACK_MAX_NUM)
	{
		//unCount = WELD_TRACK_MAX_NUM;
		for (int i = unCount; i < WELD_TRACK_MAX_NUM; i++)
		{
			var[i].p[0] = vtRobotJointCoord[unCount - 1].dX;
			var[i].p[1] = vtRobotJointCoord[unCount - 1].dY;
			var[i].p[2] = vtRobotJointCoord[unCount - 1].dZ;
			var[i].p[3] = vtRobotJointCoord[unCount - 1].dRZ;
			var[i].p[4] = vtRobotJointCoord[unCount - 1].dRY;
			var[i].p[5] = vtRobotJointCoord[unCount - 1].dRX;
			var[i].ExP[0] = 0;
			var[i].ExP[1] = 0;
			var[i].speed = tPosMove.dSpeed / 60;
			var[i].Ex_7_speed = 1;
			var[i].Ex_8_speed = 1;
			var[i].mode = config[0];
			var[i].cf1 = config[1];
			var[i].cf2 = config[2];
			var[i].cf3 = config[3];
			var[i].cf4 = config[4];
			var[i].cf5 = config[5];
			var[i].cf6 = config[6];
		}
	}
	sRt = m_pEstunRobot->SetMultiPosVar_M(unIndex, size, var, "MultiPos_Mv", 0, unToolNum, 1);//��ʼλ�ã�������0-ֱ�ǣ�1-�ؽ�
	if (sRt < 0)
	{
		XiMessageBox("���ö���� SetMultiPosVar_Mʧ��");
		return false;
	}
	if (tPosMove.dSpeed / 60 > 8 && tPosMove.dSpeed / 60 < 11)
	{
		sRt = m_pEstunRobot->SetIntVar_Py("endPos", unCount - 2, 2);
		return false;
	}
	if (tPosMove.dSpeed / 60 <= 8)
	{
		sRt = m_pEstunRobot->SetIntVar_Py("endPos", unCount - 1, 2);
		return false;
	}
	if (sRt < 0)
	{
		XiMessageBox("���ö����ֹͣendPosʧ�� ");
		return false;
	}
	delete[] var;
	return true;
}
bool EstunAdaptor::SetMultiPosVar(UINT unIndex, std::vector<T_ROBOT_COORS> vtRobotJointCoord, T_ROBOT_MOVE_SPEED tPosMove, CString Program_name, int config[7], UINT unToolNum)
{
	if (NULL == config)
	{
		for (int i = 0; i < 7; i++)
		{
			config[i] = 0;
		}
	}
	//CString szFilePath = GetModuleFilePath();
	//szFilePath.Delete(szFilePath.Find("."), 3);
	//szFilePath = "D:\\" + szFilePath;

	CString FilePath = "LocalFiles\\OutputFiles\\" + m_strRobotName + "\\ESTUN_FILE\\" + Program_name + ".erd";
	CString FilePathInI = "LocalFiles\\OutputFiles\\" + m_strRobotName + "\\ESTUN_FILE\\" + Program_name + "INI.erd";
	XiBase::GetSystemPath(FilePath);
	XiBase::GetSystemPath(FilePathInI);
	CString DownloadFile = ".\\" + Program_name + ".erd";
	CString strP;
	CString strS;
	//�����ļ�
	WriteLog(FilePath);
	int sRt = m_pEstunRobot->ConnectFtp();
	if (sRt < 0)
	{
		XiMessageBox("ConnectFtp1ʧ��");
		return false;
	}
	sRt = m_pEstunRobot->DownloadFile(DownloadFile, FilePath);
	if (sRt < 0)
	{
		XiMessageBox("�����ļ�ʧ��");
		return false;
	}
	sRt = m_pEstunRobot->DisConnectFtp();
	if (sRt < 0)
	{
		XiMessageBox("DisConnectFtp1ʧ��");
		return false;
	}
	//ת���ļ���ʽ
	sRt = m_pEstunRobot->ErdFile2IniFile(FilePath, FilePathInI);
	if (sRt < 0)
	{
		XiMessageBox("ErdFile2IniFileʧ��");
		return false;
	}
	CString ErdSpeed;
	ESTUN_SPEED speed;
	ESTUN_CPOS CPOS;
	CString erdCPos;
	UINT unCount = vtRobotJointCoord.size();
	UINT size = unCount;
	if (unCount < WELD_TRACK_MAX_NUM)
	{
		size = WELD_TRACK_MAX_NUM;
	}
	/*if (0 == m_nRobotBrand)
	{
		return false;
	}*/
	for (UINT i = 0; i < unCount; i++)
	{
		strP.Format("P%d", i + 1);
		//strS.Format("S%d", i + 1);
		strS.Format("S%d", 1);
		for (size_t i = 0; i < 7; i++)
		{
			*(&CPOS.mode + i) = config[i];
		}
		CPOS.X = vtRobotJointCoord[i].dX;
		CPOS.Y = vtRobotJointCoord[i].dY;
		CPOS.Z = vtRobotJointCoord[i].dZ;
		CPOS.A = vtRobotJointCoord[i].dRZ;
		CPOS.B = vtRobotJointCoord[i].dRY;
		CPOS.C = vtRobotJointCoord[i].dRX;
		CPOS.Axis_7 = vtRobotJointCoord[i].dBX;
		CPOS.Axis_8 = vtRobotJointCoord[i].dBX;
		sRt = m_pEstunRobot->CPos2ErdCPos(CPOS, erdCPos);
		sRt = m_pEstunRobot->WriteIniFileParameter(FilePathInI, strP, erdCPos);

		speed.tcp = (double)tPosMove.dSpeed / 60.0;
		speed.per = 10;
		speed.ori = 360;
		speed.exp_7 = 1;
		speed.exp_8 = 1;
		sRt = m_pEstunRobot->Speed2ErdSpeed(speed, ErdSpeed);
		if (sRt < 0)
		{
			XiMessageBox("Speed2ErdSpeedʧ��");
			return false;
		}
		sRt = m_pEstunRobot->WriteIniFileParameter(FilePathInI, strS, ErdSpeed);
		if (sRt < 0)
		{
			XiMessageBox("WriteIniFileParameterʧ��");
			return false;
		}
		//if (unCount < 300)
		//{
		//	for (int i = unCount; i < size; i++)
		//	{
		//		strP.Format("P%d", i + 1);
		//		strS.Format("S%d", i + 1);
		//		for (size_t i = 0; i < 7; i++)
		//		{
		//			*(&CPOS.mode + i) = config[i];
		//		}
		//		CPOS.X = vtRobotJointCoord[unCount - 1].dX;
		//		CPOS.Y = vtRobotJointCoord[unCount - 1].dY;
		//		CPOS.Z = vtRobotJointCoord[unCount - 1].dZ;
		//		CPOS.A = vtRobotJointCoord[unCount - 1].dRZ;
		//		CPOS.B = vtRobotJointCoord[unCount - 1].dRY;
		//		CPOS.C = vtRobotJointCoord[unCount - 1].dRX;
		//		CPOS.Axis_7 = vtRobotJointCoord[i].dBX;
		//		CPOS.Axis_8 = vtRobotJointCoord[i].dBY;
		//		sRt = m_pEstunRobot->CPos2ErdCPos(CPOS, erdCPos);
		//		if (sRt < 0)
		//		{
		//			return false;
		//		}
		//		sRt = m_pEstunRobot->WriteIniFileParameter(FilePathInI, strP, erdCPos);
		//		if (sRt < 0)
		//		{
		//			return false;
		//		}
		//		speed.tcp = tPosMove.nSpeed / 60;
		//		speed.per = 10;
		//		speed.ori = 360;
		//		speed.exp_7 = 1;
		//		speed.exp_8 = 1;
		//		sRt = m_pEstunRobot->Speed2ErdSpeed(speed, ErdSpeed);
		//		if (sRt < 0)
		//		{
		//			return false;
		//		}
		//		sRt = m_pEstunRobot->WriteIniFileParameter(FilePathInI, strS, ErdSpeed);
		//		if (sRt < 0)
		//		{
		//			return false;
		//		}
		//	}
		//}
	}
	//���ļ���ʽת������
	sRt = m_pEstunRobot->IniFile2ErdFile(FilePathInI, FilePath);
	if (sRt < 0)
	{
		XiMessageBox("���ļ���ʽת������ʧ��");
		return false;
	}
	//�ϴ��ļ�
	sRt = m_pEstunRobot->ConnectFtp();
	if (sRt < 0)
	{
		XiMessageBox("ConnectFtpʧ��");
		return false;
	}
	sRt = m_pEstunRobot->UploadFile(DownloadFile, FilePath);
	if (sRt < 0)
	{
		XiMessageBox("�ϴ��ļ�ʧ��");
		return false;
	}
	sRt = m_pEstunRobot->DisConnectFtp();
	if (sRt < 0)
	{
		XiMessageBox("DisConnectFtpʧ��");
		return false;
	}

	return true;
}

// ����Ӳ����ʾ���˶����� nDownSpeed�ؽڲ岹�ٶ� nTeachSpeed �� nUpSpeed ֱ�߲岹�ٶ�
bool EstunAdaptor::SendTeachMoveData(const std::vector<T_ANGLE_PULSE>& vtMeasurePulse, const std::vector<int>& vnMeasureType, int nDownSpeed, int nTeachSpeed, int nUpSpeed, int nTrigSigTime)
{
	// ���̱���
	// mJP1 - mJP20 : �����ؽ�����
	// INT1 - INT20 : �ߵ�ÿ�������Ƿ񴥷���ͼ�ź�
	// S1 : �����˶��ٶ�
	ESTUN_APOS tEstunAPos;
	ESTUN_SPEED tEstunSpeed;
	int nMaxTrackPtnSum = 40;
	int nTrackPtnSum = vtMeasurePulse.size();
	double dCurExAxisPos = GetPositionDis();
	std::vector<T_ANGLE_PULSE> vtSendPulse(vtMeasurePulse);
	std::vector<int> vnSendType(vnMeasureType);

	T_ANGLE_PULSE tLastPulse = vtSendPulse[nTrackPtnSum - 1];
	for (int n = 0; vtSendPulse.size() < nMaxTrackPtnSum; n++)
	{
		vtSendPulse.push_back(tLastPulse);
		vnSendType.push_back(E_TRANSITION_POINT);
	}

	if ((nTrackPtnSum > nMaxTrackPtnSum)) // �ǰ�˹�ٻ����� �������� ����false
	{
		XiMessageBoxOk("�����켣������࣡");
		return false;
	}

	T_MIX_VAR tMixVar;
	memset(&tMixVar, 0, sizeof(tMixVar));
	tMixVar.INT_Name = (int*)malloc(sizeof(int) * (nMaxTrackPtnSum)); // �������� �� �˶����� ����INT���� ���ܵ���
	tMixVar.INT_Value = (int*)malloc(sizeof(int) * (nMaxTrackPtnSum)); // �������� �� �˶����� ����INT���� ���ܵ���
	tMixVar.CPOS_Name = (int*)malloc(sizeof(int) * nMaxTrackPtnSum);
	tMixVar.CPOS_Value = (ESTUN_CPOS*)malloc(sizeof(ESTUN_CPOS) * nMaxTrackPtnSum);
	tMixVar.APOS_Name = (int*)malloc(sizeof(int) * nMaxTrackPtnSum);
	tMixVar.APOS_Value = (ESTUN_APOS*)malloc(sizeof(ESTUN_APOS) * nMaxTrackPtnSum);
	tMixVar.SPEED_Name = (int*)malloc(sizeof(int) * nMaxTrackPtnSum);
	tMixVar.SPEED_Value = (ESTUN_SPEED*)malloc(sizeof(ESTUN_SPEED) * nMaxTrackPtnSum);

	// S1 : �ٶȱ���
	memset(&tEstunSpeed, 1, sizeof(tEstunSpeed));
	tEstunSpeed.per = nTeachSpeed / 100; // 0.01% -> 1%	1000 => 10%
	tEstunSpeed.tcp = 1;
	tEstunSpeed.ori = 360;
	tEstunSpeed.exp_7 = 360.0;
	tEstunSpeed.exp_8 = 180.0;
	tMixVar.SPEED_Name[tMixVar.SPEED_Count] = 1;
	tMixVar.SPEED_Value[tMixVar.SPEED_Count] = tEstunSpeed;
	tMixVar.SPEED_Count++;

	for (int i = 0; i < nMaxTrackPtnSum; i++) // ��˹�ٻ����˱�����Ŵ�1
	{
		// INT1-INT20 : �Ƿ���� ������ͼ�ź�
		tMixVar.INT_Name[tMixVar.INT_Count] = i + 1;
		tMixVar.INT_Value[tMixVar.INT_Count] = (1 == (E_TRANSITION_POINT & vnSendType[i]) ? 0 : 1);
		tMixVar.INT_Count++;

		// mJP1-mJP20 : �˶��ؽ���������
		tEstunAPos.A1 = (double)vtSendPulse[i].nSPulse * m_tAxisUnit.dSPulse;
		tEstunAPos.A2 = (double)vtSendPulse[i].nLPulse * m_tAxisUnit.dLPulse;
		tEstunAPos.A3 = (double)vtSendPulse[i].nUPulse * m_tAxisUnit.dUPulse;
		tEstunAPos.A4 = (double)vtSendPulse[i].nRPulse * m_tAxisUnit.dRPulse;
		tEstunAPos.A5 = (double)vtSendPulse[i].nBPulse * m_tAxisUnit.dBPulse;
		tEstunAPos.A6 = (double)vtSendPulse[i].nTPulse * m_tAxisUnit.dTPulse;
		tEstunAPos.A7 = dCurExAxisPos;
		tEstunAPos.A8 = dCurExAxisPos;
		tMixVar.APOS_Name[tMixVar.APOS_Count] = i + 1;
		tMixVar.APOS_Value[tMixVar.APOS_Count] = tEstunAPos;
		tMixVar.APOS_Count++;
	}

	// ���س��� ��д�����
	int nRst = m_pEstunRobot->LoadUserProgramer_Py("Xi_Robot", "TEACH");
	if (nRst < 0)
	{
		XiMessageBox("���μ��س��� TEACH ʧ��2");
		return false;
	}

	nRst = m_pEstunRobot->SetMultiVar_H(tMixVar);
	if (nRst < 0)
	{
		XiMessageBox("ESTUN:���ٻ췢����ʧ��!");
		return false;
	}

	free(tMixVar.INT_Name);
	free(tMixVar.INT_Value);
	free(tMixVar.CPOS_Name);
	free(tMixVar.CPOS_Value);
	free(tMixVar.APOS_Name);
	free(tMixVar.APOS_Value);
	free(tMixVar.SPEED_Name);
	free(tMixVar.SPEED_Value);
	return true;
}


double EstunAdaptor::GetCurrentPos(int nAxisNo)
{
	double adPos[10] = { 0.0 };
	m_pEstunRobot->GetCurPos_Fast(adPos);
	if (nAxisNo < 8)
	{
		switch (nAxisNo)
		{
		case 0:return adPos[0];
		case 1:return adPos[1];
		case 2:return adPos[2];
		case 3:return adPos[5]; // ABC <=> Rz Ry Rx
		case 4:return adPos[4];
		case 5:return adPos[3];
		case 6:return adPos[6];
		case 7:return adPos[7];
		default:return 0.0;
		}
	}
	return 0.0;
}
T_ROBOT_COORS EstunAdaptor::GetCurrentPos_ESTUN()
{
	//ʹ��EstunApi��ȡ��ǰλ��
	T_ROBOT_COORS tRobotCurCoord;
	E_ROB_POS ePos = api.E_GetCurWPos();

	tRobotCurCoord.dX = ePos.getX();
	tRobotCurCoord.dY = ePos.getY();
	tRobotCurCoord.dZ = ePos.getZ();
	tRobotCurCoord.dRZ = ePos.getA(); // ABC <=> Rz Ry Rx
	tRobotCurCoord.dRY = ePos.getB();
	tRobotCurCoord.dRX = ePos.getC();
	tRobotCurCoord.dBX = ePos.getExValByIndex(7);
	tRobotCurCoord.dBY = ePos.getExValByIndex(8);
	tRobotCurCoord.dBZ = ePos.getExValByIndex(9);

	return tRobotCurCoord;
}

T_ROBOT_COORS EstunAdaptor::GetCurrentPos()
{
	T_ROBOT_COORS tRobotCurCoord;
	memset(&tRobotCurCoord, 0, sizeof(tRobotCurCoord));

	double adPos[10] = { 0 };
	int sRt = m_pEstunRobot->GetCurPos_Fast(adPos);
	if (sRt < 0)
	{
		//	XiMessageBox("GetCurPos_Fastʧ��");
	}
	tRobotCurCoord.dX = adPos[0];
	tRobotCurCoord.dY = adPos[1];
	tRobotCurCoord.dZ = adPos[2];
	tRobotCurCoord.dRX = adPos[5]; // ABC <=> Rz Ry Rx
	tRobotCurCoord.dRY = adPos[4];
	tRobotCurCoord.dRZ = adPos[3];
	tRobotCurCoord.dBX = adPos[6];
	tRobotCurCoord.dBY = adPos[7];
	tRobotCurCoord.dBZ = adPos[8];
	return tRobotCurCoord;
}

long EstunAdaptor::GetCurrentPulse(int nAxisNo)
{
	double adPos[10] = { 0.0 };
	m_pEstunRobot->GetCurAxis_Fast(adPos);
	double adAxisUnit[6] = { m_tAxisUnit.dSPulse,m_tAxisUnit.dLPulse ,m_tAxisUnit.dUPulse ,m_tAxisUnit.dRPulse ,m_tAxisUnit.dBPulse ,m_tAxisUnit.dTPulse };
	return (long)(adPos[nAxisNo] / adAxisUnit[nAxisNo]);
}

T_ANGLE_PULSE EstunAdaptor::GetCurrentPulse()
{
	T_ANGLE_PULSE tRobotCurPulses;
	double adPos[10] = { 0.0 };
	m_pEstunRobot->GetCurAxis_Fast(adPos);
	tRobotCurPulses.nSPulse = (long)(adPos[0] / m_tAxisUnit.dSPulse);
	tRobotCurPulses.nLPulse = (long)(adPos[1] / m_tAxisUnit.dLPulse);
	tRobotCurPulses.nUPulse = (long)(adPos[2] / m_tAxisUnit.dUPulse);
	tRobotCurPulses.nRPulse = (long)(adPos[3] / m_tAxisUnit.dRPulse);
	tRobotCurPulses.nBPulse = (long)(adPos[4] / m_tAxisUnit.dBPulse);
	tRobotCurPulses.nTPulse = (long)(adPos[5] / m_tAxisUnit.dTPulse);
	tRobotCurPulses.lBXPulse = adPos[6];
	tRobotCurPulses.lBYPulse = adPos[7];
	tRobotCurPulses.lBZPulse = adPos[8];
	return tRobotCurPulses;
}

double EstunAdaptor::GetPositionDis() 
{
	double adPos[10] = { 0.0 };
	m_pEstunRobot->GetCurPos_Fast(adPos);
	return adPos[6];
}





