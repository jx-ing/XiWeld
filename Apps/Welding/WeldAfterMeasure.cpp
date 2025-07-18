#include "StdAfx.h"
#include "WeldAfterMeasure.h"
#include "WeldDataJson.h"
#include <GFPGJointLib.h>
//#include "small_parts_convertor.h"

//#include "AssemblyWeld.h"

using namespace WAM;

// �Ȳ�󺸻���
WeldAfterMeasure::WeldAfterMeasure(CUnit* ptUnit, E_WORKPIECE_TYPE eWorkPieceType) :
	m_ptUnit(ptUnit),
	m_pRobotDriver(ptUnit->GetRobotCtrl()),
	m_eWorkPieceType(eWorkPieceType),
	m_bIsLocalDebug(false),
	m_WeldSeamGroupAngle(0),
	m_bWorking(false),
	m_bDeleteSeam(TRUE),
	m_dZPallet(0.0),
	m_dSideBoardThick(0.0),
	m_dBoardLen(0.0),
	m_dWeldHoleSize(0.0),
	m_dEndBoardSupportLen(0.0),
	m_dPurlinSupportLen(0.0),
	m_nExPartType(1),
	m_nWeldMode(1),
	m_dScanEndpointOffset(0.0),
	m_pUnWarpBoundWeld(false),
	//m_pUnTracePointCloudProcess(false),
	m_nPauseGroupNo(0),
	m_nPauseWeldNo(0),
	m_nPauseLayerNo(0)
{
	m_pRobotDriver->RobotKinematics(m_pRobotDriver->m_tHomePulse, m_pRobotDriver->m_tTools.tGunTool, m_tRobotHomeCoors);
	LoadCompParam();
	LoadEquipmentParam();
	SetFilePath();
	//InitCheckCollide(DATA_PATH + m_ptUnit->m_tContralUnit.strUnitName + "\\new_head.step");
}

WeldAfterMeasure::~WeldAfterMeasure()
{
	m_ptUnit = NULL;
	m_pRobotDriver = NULL;
	m_pIsArcOn = NULL;
	m_pIsNaturalPop = NULL;
	m_pColorImg = NULL;
}

void WeldAfterMeasure::AppendTeachData(vector<T_TEACH_DATA>& vtTeachData, vector<T_TEACH_DATA> vtNewTeachData)
{
	for (int i = 0; i < vtNewTeachData.size(); i++)
	{
		vtTeachData.push_back(vtNewTeachData[i]);
	}
}

void WeldAfterMeasure::DecomposeExAxle(T_ROBOT_COORS& tRobotCoord, double dExAxlePos)
{
#ifdef SINGLE_ROBOT
	tRobotCoord.dY -= dExAxlePos;
#else
	tRobotCoord.dX -= dExAxlePos;
#endif // SINGLE_ROBOT
}

void WeldAfterMeasure::DecomposeExAxle(T_ROBOT_COORS& tRobotCoord, double dExAxlePos, int nExAxleNo)
{
	switch (nExAxleNo)
	{
	case 1:tRobotCoord.dX -= dExAxlePos; tRobotCoord.dBX = dExAxlePos; break;
	case 2:tRobotCoord.dY -= dExAxlePos; tRobotCoord.dBY = dExAxlePos; break;
	case 3:tRobotCoord.dZ -= dExAxlePos; tRobotCoord.dBZ = dExAxlePos; break;
	default:XiMessageBoxOk("DecomposeExAxle ��������!"); break;
	}
}

int WeldAfterMeasure::CheckTeachCoord(const T_ROBOT_COORS& tTeachCoord, const vector<T_ROBOT_COORS>& vtTotalTeachCoord, T_ROBOT_COORS tThreshold)
{
	int nRst = -1;
	for (int nIdx = 0; nIdx < vtTotalTeachCoord.size(); nIdx++)
	{
		if (true == m_pRobotDriver->CompareCoords(tTeachCoord, vtTotalTeachCoord[nIdx], tThreshold.dX, 1, tThreshold.dRX))
		{
			nRst = nIdx;
			break;
		}
	}
	return nRst;
}

CString WeldAfterMeasure::GetTeachDataString(const T_TEACH_DATA& tTeachData)
{
	CString sRst;
	CString sImageType;
	CString sCalcType;
	CString sSeamType;
	CString sAttribute;
	CString sEndpointType;

	if (E_TRANSITION_POINT & tTeachData.nMeasureType) sImageType = "����";
	else if (E_DOUBLE_LONG_LINE & tTeachData.nMeasureType) sImageType = "˫��";
	else if (E_LS_RL_FLIP & tTeachData.nMeasureType) sImageType = "�̳�";
	else if (E_LL_RS_FLIP & tTeachData.nMeasureType) sImageType = "����";
	else if (E_L_LINE_POINT & tTeachData.nMeasureType) sImageType = "����";
	else if (E_R_LINE_POINT & tTeachData.nMeasureType) sImageType = "����";
	else sImageType = "δ֪";

	if (E_TRANSITION_POINT & tTeachData.nMeasureType) sCalcType = "����";
	else if (E_TEACH_POINT & tTeachData.nMeasureType) sCalcType = "ʾ��";
	else if (E_ADJUST_POINT & tTeachData.nMeasureType) sCalcType = "����";
	else if (E_SEARCH_POINT & tTeachData.nMeasureType) sCalcType = "����";
	else if (E_ARRIS_POINT & tTeachData.nMeasureType) sCalcType = "���";
	else sCalcType = "δ֪";

	if (E_FLAT_SEAM == tTeachData.eWeldSeamType) sSeamType = "ƽ��";
	else if (E_STAND_SEAM == tTeachData.eWeldSeamType) sSeamType = "����";
	else sSeamType = "δ֪";

	if (E_BELONG_START == tTeachData.eAttribute) sAttribute = "���";
	else if (E_BELONG_MIDDLE == tTeachData.eAttribute) sAttribute = "�м�";
	else if (E_BELONG_END == tTeachData.eAttribute) sAttribute = "�յ�";
	else sAttribute = "δ֪";

	if (E_INTERFERE_POINT == tTeachData.eEndpointType) sEndpointType = "����";
	else if (E_FREE_POINT == tTeachData.eEndpointType) sEndpointType = "����";
	else sEndpointType = "δ֪";

	sRst.Format("%s%s %s %s %s", sCalcType, sImageType, sSeamType, sAttribute, sEndpointType);
	return sRst;
}


long WeldAfterMeasure::GetMoveExAxisPos(T_ANGLE_PULSE tPulse, int nAxisNo)
{
	long lExAxisPos = 0.0;
	switch (nAxisNo)
	{
	case 1:lExAxisPos = tPulse.lBXPulse; break;
	case 2:lExAxisPos = tPulse.lBYPulse; break;
	case 3:lExAxisPos = tPulse.lBZPulse; break;
	default: break;
	}
	return lExAxisPos;
}

double WeldAfterMeasure::GetMoveExAxisPos(T_ROBOT_COORS tCoord, int nAxisNo)
{
	double dExAxisPos = 0.0;
	switch (nAxisNo)
	{
	case 1:dExAxisPos = tCoord.dBX; break;
	case 2:dExAxisPos = tCoord.dBY; break;
	case 3:dExAxisPos = tCoord.dBZ; break;
	default: break;
	}
	return dExAxisPos;
}

void WeldAfterMeasure::SaveWorkpieceType(CRobotDriverAdaptor* pRobotCtrl)
{
	int nNo = 0;
	CString str;
	str.Format(".\\Data\\%sPause.ini", pRobotCtrl->m_strRobotName);
	COPini opini;
	opini.SetFileName(str);
	opini.SetSectionName(pRobotCtrl->m_strRobotName);
	//nNo = /*(int)pRobotCtrl->*/m_tChoseWorkPieceType;
	opini.WriteString("WorkPieceType", nNo);
}

void WeldAfterMeasure::LoadWorkpieceType(CRobotDriverAdaptor* pRobotCtrl, std::map<int, CString> &nsPartType)
{
	nsPartType.clear();
	COPini opini;
	int nNo = 0;
	int nTotalTypeNum = 0;
	int nTypeNum = 0;
	CString sTypeName = "";
	CString str;
	opini.SetFileName(SYSTEM_PARA_INI);
	opini.SetSectionName("TotalPartType");
	opini.ReadString("TotalNumber", &nTotalTypeNum);
	for (int i = 0; i < nTotalTypeNum; i++)
	{
		str.Format("Name%d", i);
		opini.ReadString(str, sTypeName);
		sTypeName = Utf8ToGBK(sTypeName);
		str.Format("Number%d", i);
		opini.ReadString(str, &nTypeNum);
		nsPartType[nTypeNum] = sTypeName;
	}
	if (nsPartType.size() != nTotalTypeNum)
	{
		XiMessageBox("�������ͼ���ʧ�ܣ�");
	}

	str.Format(".\\Data\\%sPause.ini", pRobotCtrl->m_strRobotName);
	opini.SetFileName(str);
	opini.SetSectionName(pRobotCtrl->m_strRobotName);
	opini.ReadString("WorkPieceType", &nNo);
	//m_tChoseWorkPieceType = (E_WORKPIECE_TYPE)nNo;
}

void WeldAfterMeasure::LoadEquipmentParam()
{
	COPini opini;
	opini.SetFileName(DATA_PATH + m_ptUnit->m_tContralUnit.strUnitName + SYETEM_PARAM_FILE);
	opini.SetSectionName("EquipmentParam");
	opini.ReadString("RobotInstallDir"		, &m_nRobotInstallDir);
	opini.ReadString("GunAngle"				, &m_dGunAngle);
	opini.ReadString("GunLaserAngle"		, &m_dGunLaserAngle);
	opini.ReadString("GunCameraAngle"		, &m_dGunCameraAngle);
	opini.ReadString("RotateToCamRxDir"		, &m_dRotateToCamRxDir);
	opini.ReadString("HandEyeDis"			, &m_dHandEyeDis);
	opini.ReadString("TrackCamHandEyeDis"	, &m_dTrackCamHandEyeDis);
	opini.ReadString("MeasureDisThreshold"	, &m_dMeasureDisThreshold);
	opini.ReadString("FlatWeldRx"			, &m_dPlatWeldRx);
	opini.ReadString("FlatWeldRy"			, &m_dPlatWeldRy);
	opini.ReadString("NormalWeldRx"			, &m_dNormalWeldRx);
	opini.ReadString("NormalWeldRy"			, &m_dNormalWeldRy);
	opini.ReadString("StandWeldRx"			, &m_dStandWeldRx);
	opini.ReadString("StandWeldRy"			, &m_dStandWeldRy);
	opini.ReadString("TransitionsRx"		, &m_dTransitionsRx);
	opini.ReadString("TransitionsRy"		, &m_dTransitionsRy);
	opini.ReadString("StandWeldScanFreeRx"	, &m_dStandWeldScanRx);
	opini.ReadString("StandWeldScanFreeRy"	, &m_dStandWeldScanRy);
	opini.ReadString("StandWeldScanOffsetRz", &m_dStandWeldScanOffsetRz);
	opini.ReadString("GrooveScanRx"			, &m_dGrooveScanRx);
	opini.ReadString("GrooveScanRy"			, &m_dGrooveScanRy);
	opini.ReadString("GrooveScanOffsetRz"	, &m_dGrooveScanOffsetRz);
	opini.ReadString("GrooveWeldRx"			, &m_dGrooveWeldRx);
	opini.ReadString("GrooveWeldRy"			, &m_dGrooveWeldRy);
	opini.ReadString("GrooveWeldRz"			, &m_dGrooveWeldRz);
	opini.ReadString("WeldNorAngleInHome"	, &m_dWeldNorAngleInHome);
	opini.ReadString("EndpointSearchDis"	, &m_dEndpointSearchDis);
	opini.ReadString("GunDownBackSafeDis"	, &m_dGunDownBackSafeDis);
	opini.ReadString("ShortSeamThreshold"	, &m_dShortSeamThreshold);
	opini.ReadString("LengthSeamThreshold"	, &m_dLengthSeamThreshold);
	opini.ReadString("PointSpacing"			, &m_dPointSpacing);
	opini.ReadString("CleanGunDis"			, &m_dCleanGunDis);
	opini.ReadString("FlatWeldContinue"		, &m_bFlatWeldContinue);
	opini.ReadString("CheckCollideEnable"	, &m_bCheckCollide);
	opini.ReadString("TrackingWeldEnable"	, &m_bTrackingEnable);
	opini.ReadString("TrackingLenThreshold"	, &m_dTrackingLengthThreshold);
	opini.ReadString("AutoCalcWrapDirStand"	, &m_bAutoCalcWrapDirStand);
	opini.ReadString("ScanTrackingWeldEnable ", &m_ptUnit->m_ScanTrackingWeldEnable);

	m_pRobotDriver->m_nRobotInstallDir = m_nRobotInstallDir;
}

void WeldAfterMeasure::LoadCompParam()
{
	// ���ز�������
	m_mdFlatHorComp.clear();
	m_mdFlatHeightComp.clear();
	m_mdStandLenComp.clear();
	m_mdStandVerComp.clear();
	CString sSectionName;
	COPini opini;
	opini.SetFileName(DATA_PATH + m_ptUnit->m_tContralUnit.strUnitName + WELD_PARAM_FILE);

	double dFlatHorComp = 0.0;
	double dFlatHeightComp = 0.0;
	// 0 45 90 135 180 215 270 315
	for (int i = 0; i < 360; i += 45)
	{
		sSectionName.Format("WeldCompVal%d", i);
		opini.SetSectionName(sSectionName);
		opini.ReadString("FlatWeldHorComp", &dFlatHorComp);
		opini.ReadString("FlatWeldHeightComp", &dFlatHeightComp);
		m_mdFlatHorComp[i] = dFlatHorComp;
		m_mdFlatHeightComp[i] = dFlatHeightComp;
	}

	sSectionName.Format("WeldCompVal%d", 405);
	opini.SetSectionName(sSectionName);
	opini.ReadString("FlatWeldHorComp", &dFlatHorComp);
	opini.ReadString("FlatWeldHeightComp", &dFlatHeightComp);
	m_mdFlatHorComp[405] = dFlatHorComp;
	m_mdFlatHeightComp[405] = dFlatHeightComp;

	double dStandLenComp = 0.0;
	double dStandVerComp = 0.0;
	// 0 45 90 135 180 215 270 315
	for (int i = 0; i < 360; i += 45)
	{
		sSectionName.Format("StandWeldComp%d", i);
		opini.SetSectionName(sSectionName);
		opini.ReadString("StandWeldLenComp", &dStandLenComp);
		opini.ReadString("StandWeldVerComp", &dStandVerComp);
		m_mdStandLenComp[i] = dStandLenComp;
		m_mdStandVerComp[i] = dStandVerComp;
	}
}

void WeldAfterMeasure::SetFilePath()
{
	m_sDataSavePath = OUTPUT_PATH + m_ptUnit->m_tContralUnit.strUnitName + RECOGNITION_FOLDER;
	m_sPointCloudFileName = OUTPUT_PATH + m_ptUnit->m_tContralUnit.strUnitName + "\\" + POINT_CLOUD_FILE;
	CheckFolder(m_sDataSavePath);
}

void WeldAfterMeasure::SetWeldParam(BOOL* pIsArcOn, BOOL* pIsNaturalPop, BOOL* pIsTeachPop, BOOL* bNeedWrap)
{
	m_pIsArcOn = pIsArcOn;
	m_pIsNaturalPop = pIsNaturalPop;
	m_pIsTeachPop = pIsTeachPop;
	m_bNeedWrap = bNeedWrap;
	//if (-1 == m_nRobotInstallDir)
	//{
	//	m_nRobotHangPos = HANGING_UPSIDE_DOWN;
	//}
	//else if (1 == m_nRobotInstallDir)
	//{
	//	m_nRobotHangPos = HANGING_UP;
	//}
	//else
	//{
	//	XiMessageBox("�������������ͳ���");
	//}
}

void WeldAfterMeasure::SetHardware(CScanInitModule* pScanInit)
{
	m_pScanInit = pScanInit;
	return;
}

void WeldAfterMeasure::LoadContourData(CRobotDriverAdaptor* pRobotDriver, CString cFileName, CvPoint3D64f* pPointCloud, int& nPtnNum)
{
	int nRobotNo = pRobotDriver->m_nRobotNo;
	int nWeldLineNo = 0;

	HANDLE hFile = CreateFile(cFileName, GENERIC_READ, FILE_SHARE_READ, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
	if (hFile == INVALID_HANDLE_VALUE) {
		std::cerr << "�޷����ļ�" << std::endl;
		return;
	}

	// �����ļ�ӳ��
	HANDLE hMapping = CreateFileMapping(hFile, NULL, PAGE_READONLY, 0, 0, NULL);
	if (hMapping == NULL) {
		std::cerr << "�޷������ļ�ӳ��" << std::endl;
		CloseHandle(hFile);
		return;
	}

	// ӳ����ͼ
	LPVOID pBuffer = MapViewOfFile(hMapping, FILE_MAP_READ, 0, 0, 0);
	if (pBuffer == NULL) {
		std::cerr << "�޷�ӳ����ͼ" << std::endl;
		CloseHandle(hMapping);
		CloseHandle(hFile);
		return;
	}

	// ʹ��ӳ����ڴ�
	char* data = static_cast<char*>(pBuffer);
	std::cout << "�ļ�����: " << data << std::endl;

	// ������Դ
	UnmapViewOfFile(pBuffer);
	CloseHandle(hMapping);
	CloseHandle(hFile);

	return;


	if (false == CheckFileExists(cFileName, false))
	{
		return;
	}
	FILE* pf = fopen(cFileName, "r");
	double nNo = 0, nNo2 = 0, nNo3 = 0;
	nPtnNum = 0;
	while (EOF != fscanf(pf, "%lf%lf%lf%lf%lf%lf",
		&nNo, &pPointCloud[nPtnNum].x, &pPointCloud[nPtnNum].y, &pPointCloud[nPtnNum].z, &nNo2, &nNo3))
	{
		nPtnNum++;
	}
	fclose(pf);
	return;
}

void WeldAfterMeasure::LoadContourData(CRobotDriverAdaptor* pRobotDriver, CString cFileName, vector<CvPoint3D64f>& vtPointCloud)
{
	vtPointCloud.reserve(3000000);
	if (false == CheckFileExists((CString)cFileName, false))
	{
		return;
	}

	FILE* pf = fopen(cFileName, "r");
	int nNo = 0;
	CvPoint3D64f tTempPtn;
	double refuse_1 = 0;
	double refuse_2 = 0;

	while (EOF != fscanf(pf, "%d%lf%lf%lf", &nNo, &tTempPtn.x, &tTempPtn.y, &tTempPtn.z/*,&refuse_1, &refuse_2*/))
	{
		vtPointCloud.push_back(tTempPtn);
		m_vtImageNum.push_back(nNo);
	}
	fclose(pf);
	vtPointCloud.shrink_to_fit();

	return;
}

void WeldAfterMeasure::LoadContourData3D(CRobotDriverAdaptor* pRobotDriver, CString cFileName, vector<CvPoint3D64f>& vtPointCloud)
{
	vtPointCloud.reserve(3000000);
	if (false == CheckFileExists((CString)cFileName, false))
	{
		return;
	}

	FILE* pf = fopen(cFileName, "r");
	int nNo = 0;
	CvPoint3D64f tTempPtn;
	double refuse_1 = 0;
	double refuse_2 = 0;

	while (EOF != fscanf(pf, "%d%lf%lf%lf", &nNo, &tTempPtn.x, &tTempPtn.y, &tTempPtn.z))
	{
		vtPointCloud.push_back(tTempPtn);
		m_vtImageNum.push_back(nNo);
	}
	fclose(pf);
	vtPointCloud.shrink_to_fit();

	return;
}

void WeldAfterMeasure::SaveContourData()
{
	FILE* pfOut = NULL;
	if (0 != XI_fopen_s(&pfOut, ".\\LocalFiles\\OutputFiles\\RobotA\\Recognition\\PointCloud.txt", "w"))
	{
		fclose(pfOut);
		return;
	}

	CString str1 = ".\\LocalFiles\\OutputFiles\\RobotA\\Recognition\\PointCloud_0.txt";
	for (int n = 0; n < LINESCAN_CAMERA_NUM; n++)
	{
		str1.Format(".\\LocalFiles\\OutputFiles\\RobotA\\Recognition\\PointCloud_%d.txt", n);
		FILE* pf = fopen(str1, "r");
		char* cstr1 = (char*)malloc(sizeof(char) * 128);
		memset(cstr1, '/0', 128);
		while (NULL != fgets(cstr1, 128, pf))
		{
			fprintf(pfOut, "%s\n", cstr1);
			memset(cstr1, '/0', 128);
		}
		fclose(pf);
	}
	fclose(pfOut);
	return;
}

bool WeldAfterMeasure::SavePointCloudProcessResult(LineOrCircularArcWeldingSeam* pWeldSeams, int nWeldLineNumber)
{
	vector<LineOrCircularArcWeldingLine> vtRecoWeldLine(nWeldLineNumber);
	for (int i = 0; i < nWeldLineNumber; i++)
	{
		vtRecoWeldLine[i].IsArc = pWeldSeams[i].IsArc;
		vtRecoWeldLine[i].isClockwise = pWeldSeams[i].bClockwise;
		vtRecoWeldLine[i].ZSide = pWeldSeams[i].ZSide;
		if (((pWeldSeams[i].StartNormalVector.x > 0) && (pWeldSeams[i].StartNormalVector.y > 0)) ||
			((pWeldSeams[i].StartNormalVector.x < 0) && (pWeldSeams[i].StartNormalVector.y < 0)))
		{
			vtRecoWeldLine[i].isLeft = true;
		}
		else
		{
			vtRecoWeldLine[i].isLeft = false;
		}
		vtRecoWeldLine[i].CenterPoint = pWeldSeams[i].CenterPoint;
		vtRecoWeldLine[i].StartPoint = pWeldSeams[i].StartPoint;
		vtRecoWeldLine[i].EndPoint = pWeldSeams[i].EndPoint;
		vtRecoWeldLine[i].StartNormalVector = pWeldSeams[i].StartNormalVector;
		vtRecoWeldLine[i].EndNormalVector = pWeldSeams[i].EndNormalVector;
		vtRecoWeldLine[i].StartPointType = pWeldSeams[i].StartPointType;
		vtRecoWeldLine[i].EndPointType = pWeldSeams[i].EndPointType;
	}
	return SavePointCloudProcessResult(vtRecoWeldLine.data(), nWeldLineNumber);
}

bool WeldAfterMeasure::SavePointCloudProcessResult(LineOrCircularArcWeldingLine* pWeldSeams, int nWeldLineNumber)
{
	Welding_Info tWeldInfo;

	// LineOrCircularArcWeldingLine -> Welding_Info 
	// �Ӿ��ⶨ�庸����Ϣ�ṹ�� Welding_Info ���£�
	//Welding_Track* tracks;			// �켣����
	//unsigned int tracks_size;		// �켣���鳤��
	tWeldInfo.tracks_size = nWeldLineNumber;
	tWeldInfo.tracks = new Welding_Track[nWeldLineNumber];
	for (int i = 0; i < nWeldLineNumber; i++)
	{
		// �Ӿ���ṹ�� Welding_Track �������£�
		//Three_DPoint* points;			// �켣������
		//Three_DPoint* points_normal;	// �켣�㷨������
		//Three_DPoint* points_upper;		// �켣���Ӧ���ص����飬����Ϊ NULL
		//unsigned int points_size;		// �켣�����鳤��
		//int start_point_type;			// ������ͣ�0�����ɶˣ�1������ˣ�3�����
		//int end_point_type;				// �յ����ͣ�ͬ��
		//int track_type;					// �켣���ͣ�0��ֱ�ߺ��죻1�����ߺ��죻
		//Three_DPoint center;			// ���ߺ���Բ�����ֱ꣬�ߺ���Ϊ { 0, 0, 0 }
		//double ZSide;					// ����߶�
		//bool isLeft;					// �����������Ƿ�����ƽ��Ϊ false
		//int direction;					// ���췽��0��ƽ����1������
		tWeldInfo.tracks[i].points_size = 2;														// ֻ������յ�
		tWeldInfo.tracks[i].start_point_type = pWeldSeams[i].StartPointType;						// ������ͣ�0�����ɶˣ�1������ˣ�3�����
		tWeldInfo.tracks[i].end_point_type = pWeldSeams[i].EndPointType;							// �յ����ͣ�ͬ��
		tWeldInfo.tracks[i].track_type = pWeldSeams[i].IsArc;										// �켣���ͣ�0��ֱ�ߺ��죻1�����ߺ���
		Trans3D(pWeldSeams[i].CenterPoint, tWeldInfo.tracks[i].center);								// ���ߺ���Բ�����ֱ꣬�ߺ���Ϊ { 0, 0, 0 }
		tWeldInfo.tracks[i].ZSide = pWeldSeams[i].ZSide;											// ����߶�
		tWeldInfo.tracks[i].isLeft = pWeldSeams[i].isLeft;											// �����������Ƿ�����ƽ��Ϊ false
		tWeldInfo.tracks[i].direction = IsStandWeldSeam(pWeldSeams[i].StartNormalVector) ? 1 : 0;	// ���췽��0��ƽ����1������
		tWeldInfo.tracks[i].points = new Three_DPoint[2];											// �켣������
		Trans3D(pWeldSeams[i].StartPoint, tWeldInfo.tracks[i].points[0]);
		Trans3D(pWeldSeams[i].EndPoint, tWeldInfo.tracks[i].points[1]);
		tWeldInfo.tracks[i].points_normal = new Three_DPoint[2];									// �켣�㷨������
		Trans3D(pWeldSeams[i].StartNormalVector, tWeldInfo.tracks[i].points_normal[0]);
		Trans3D(pWeldSeams[i].EndNormalVector, tWeldInfo.tracks[i].points_normal[1]);
		if (0 == tWeldInfo.tracks[i].direction)
		{
			tWeldInfo.tracks[i].points_upper = new Three_DPoint[2];									// ƽ�����켣���Ӧ���ص�����
			Trans3D(pWeldSeams[i].StartPoint, tWeldInfo.tracks[i].points_upper[0]);
			Trans3D(pWeldSeams[i].EndPoint, tWeldInfo.tracks[i].points_upper[1]);
			tWeldInfo.tracks[i].points_upper[0].z = pWeldSeams[i].ZSide;
			tWeldInfo.tracks[i].points_upper[1].z = pWeldSeams[i].ZSide;
		}
		else
		{
			tWeldInfo.tracks[i].points_upper = NULL;												// ����Ϊ NULL
		}
	}

	// ����
	bool bRst = SavePointCloudProcessResult(tWeldInfo);

	// tWeldInfo �ͷ�
	for (int i = 0; i < nWeldLineNumber; i++)
	{
		DELETE_POINTER_ARRAY(tWeldInfo.tracks[i].points);
		DELETE_POINTER_ARRAY(tWeldInfo.tracks[i].points_normal);
		DELETE_POINTER_ARRAY(tWeldInfo.tracks[i].points_upper);
	}
	DELETE_POINTER_ARRAY(tWeldInfo.tracks);

	return bRst;
}

bool WeldAfterMeasure::SavePointCloudProcessResult(const Welding_Info& tWeldInfo)
{
	WeldInfoToJson(tWeldInfo); // ����Json��ʽ������Ϣ

	m_vtWeldSeamData.clear();
	CString sOutFileName2 = OUTPUT_PATH + m_ptUnit->m_tContralUnit.strUnitName + "\\" + POINT_CLOUD_IDENTIFY_RESULT;
	FILE* pf2 = fopen(sOutFileName2.GetBuffer(), "w");
	double dAdjustZ = 0;

	std::vector<T_WELD_PARA> vtWeldParaFlat;
	std::vector<T_WELD_PARA> vtWeldParaStand;
	CHECK_BOOL_RETURN(GetCurWeldParam(E_FLAT_SEAM, vtWeldParaFlat));
	CHECK_BOOL_RETURN(GetCurWeldParam(E_STAND_SEAM, vtWeldParaStand));
	for (unsigned int i = 0; i < tWeldInfo.tracks_size; i++)
	{
		LineOrCircularArcWeldingLine tWeld;
		memset(&tWeld, 0, sizeof(tWeld));
		Welding_Track tTrack = tWeldInfo.tracks[i];

		// ����켣����
		CString sFileName = OUTPUT_PATH + m_ptUnit->m_tContralUnit.strUnitName + "\\" + POINT_CLOUD_IDENTIFY_RESULT;
		sFileName.Format("%s00_PointCloudRecoTrack%d.txt",
			OUTPUT_PATH + m_ptUnit->m_tContralUnit.strUnitName + RECOGNITION_FOLDER, i);
		FILE* pf = fopen(sFileName, "w");
		for (unsigned int nPtnNo = 0; nPtnNo < tTrack.points_size; nPtnNo++)
		{
			fprintf(pf, "%d%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf\n", nPtnNo,
				tTrack.points[nPtnNo].x,
				tTrack.points[nPtnNo].y,
				tTrack.points[nPtnNo].z,
				tTrack.points_normal[nPtnNo].x,
				tTrack.points_normal[nPtnNo].y,
				tTrack.points_normal[nPtnNo].z);
		}
		fclose(pf);

		tWeld.StartPoint.x = tTrack.points[0].x;
		tWeld.StartPoint.y = tTrack.points[0].y;
		tWeld.StartPoint.z = tTrack.points[0].z;
		tWeld.StartNormalVector.x = tTrack.points_normal[0].x;
		tWeld.StartNormalVector.y = tTrack.points_normal[0].y;
		tWeld.StartNormalVector.z = tTrack.points_normal[0].z;

		tWeld.EndPoint.x = tTrack.points[tTrack.points_size - 1].x;
		tWeld.EndPoint.y = tTrack.points[tTrack.points_size - 1].y;
		tWeld.EndPoint.z = tTrack.points[tTrack.points_size - 1].z;
		tWeld.EndNormalVector.x = tTrack.points_normal[tTrack.points_size - 1].x;
		tWeld.EndNormalVector.y = tTrack.points_normal[tTrack.points_size - 1].y;
		tWeld.EndNormalVector.z = tTrack.points_normal[tTrack.points_size - 1].z;

		double dStartToEndDis = TwoPointDis(tWeld.StartPoint.x, tWeld.StartPoint.y, tWeld.StartPoint.z,
			tWeld.EndPoint.x, tWeld.EndPoint.y, tWeld.EndPoint.z);
		if (dStartToEndDis < 1)
		{
			tWeld.EndPoint = tWeld.StartPoint;
		}
		//if (dStartToEndDis < 0.1)
		//{
		//	tWeld.EndPoint.x = tTrack.points[tTrack.points_size / 2].x;
		//	tWeld.EndPoint.y = tTrack.points[tTrack.points_size / 2].y;
		//	tWeld.EndPoint.z = tTrack.points[tTrack.points_size / 2].z;
		//	tWeld.EndNormalVector.x = tTrack.points_normal[tTrack.points_size / 2].x;
		//	tWeld.EndNormalVector.y = tTrack.points_normal[tTrack.points_size / 2].y;
		//	tWeld.EndNormalVector.z = tTrack.points_normal[tTrack.points_size / 2].z;
		//}

		tWeld.StartPointType = tTrack.start_point_type;
		tWeld.EndPointType = tTrack.end_point_type;
		tWeld.IsArc = tTrack.track_type;
		tWeld.ZSide = tTrack.ZSide;
		tWeld.isLeft = tTrack.isLeft;

		bool bIsStandWeld = IsStandWeldSeam(tWeld.StartNormalVector);
		int nWeldSize = bIsStandWeld ? vtWeldParaStand[0].nWeldAngleSize : vtWeldParaFlat[0].nWeldAngleSize;
		double dStartWeldHoleSize = false == tWeld.StartPointType ? 0.0 : m_dWeldHoleSize;
		double dEndWeldHoleSize = false == tWeld.EndPointType ? 0.0 : m_dWeldHoleSize;
		fprintf(pf2, "%d %4d%4d%11.3lf%4d%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%4d%4d 0 0 0 %4d%11.3lf%11.3lf 0 0.0 0.0\n",
			i, tWeld.IsArc, tWeld.isClockwise, tWeld.ZSide, tWeld.isLeft,
			tTrack.center.x, tTrack.center.y, tTrack.center.z - dAdjustZ,
			//tWeld.CenterPoint.x, tWeld.CenterPoint.y, tWeld.CenterPoint.z - dAdjustZ,
			tWeld.StartPoint.x, tWeld.StartPoint.y, tWeld.StartPoint.z - dAdjustZ,
			tWeld.EndPoint.x, tWeld.EndPoint.y, tWeld.EndPoint.z - dAdjustZ,
			tWeld.StartNormalVector.x, tWeld.StartNormalVector.y, tWeld.StartNormalVector.z,
			tWeld.EndNormalVector.x, tWeld.EndNormalVector.y, tWeld.EndNormalVector.z,
			tWeld.StartPointType, tWeld.EndPointType, nWeldSize, dStartWeldHoleSize, dEndWeldHoleSize);
		m_vtWeldSeamData.push_back(tWeld);
	}
	fclose(pf2);
	return true;
}

bool WeldAfterMeasure::SaveSplitAfterPointCloudProcessResult(std::vector<WeldLineInfo> vtWeldSeamInfo, CString sFileName/* = ""*/ , bool IsArc)
{
	m_vtWeldSeamData.clear();
	CString sOutFileName = m_sDataSavePath + "00-SteelSeam.txt";
	FILE* pf = fopen(sOutFileName.GetBuffer(), "w");
	//CString sOutFileName2 = "GraphData\\PointCloudIdentifyReault.txt";
	FILE* pf2 = fopen(sFileName.GetBuffer(), "w");
	double dAdjustZ = 0;

	for (int i = 0; i < vtWeldSeamInfo.size(); i++)
	{
		LineOrCircularArcWeldingLine tWeld = vtWeldSeamInfo[i].tWeldLine;
		WeldSeamAtrribute tAtrribute = vtWeldSeamInfo[i].tAtrribute;

		fprintf(pf, "%d%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%4d%4d%4d%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%4d%4d%4d%4d%4d%11.3lf%11.3lf%11.3lf%4d\n",
			i, tWeld.StartPoint.x, tWeld.StartPoint.y, tWeld.StartPoint.z,
			tWeld.EndPoint.x, tWeld.EndPoint.y, tWeld.EndPoint.z,
			tWeld.ZSide, tWeld.isLeft, tWeld.isClockwise, tWeld.IsArc, tWeld.CenterPoint.x, tWeld.CenterPoint.y, tWeld.CenterPoint.z,
			tWeld.StartNormalVector.x, tWeld.StartNormalVector.y, tWeld.StartNormalVector.z,
			tWeld.EndNormalVector.x, tWeld.EndNormalVector.y, tWeld.EndNormalVector.z,
			tWeld.StartPointType, tWeld.EndPointType,
			tAtrribute.nIsDoubleWelding, tAtrribute.nStartWrapType, tAtrribute.nEndWrapType,
			tAtrribute.nWeldAngleSize, tAtrribute.dStartHoleSize, tAtrribute.dEndHoleSize, tAtrribute.nRobotSelete
		);
		//ͬ�⴦���ļ�
		if (IsArc)
		{// ����ӿڹ���־��ʹ��
			fprintf(pf2, "%d %4d%4d%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%4d%4d%4d%4d%4d\n",
				i, tWeld.IsArc, tWeld.isClockwise, tWeld.ZSide, /*tWeld.isLeft,*/
				tWeld.CenterPoint.x, tWeld.CenterPoint.y, tWeld.CenterPoint.z,
				tWeld.StartPoint.x, tWeld.StartPoint.y, tWeld.StartPoint.z,
				tWeld.EndPoint.x, tWeld.EndPoint.y, tWeld.EndPoint.z,
				tWeld.StartNormalVector.x, tWeld.StartNormalVector.y, tWeld.StartNormalVector.z,
				tWeld.EndNormalVector.x, tWeld.EndNormalVector.y, tWeld.EndNormalVector.z,
				tWeld.StartPointType, tWeld.EndPointType, tAtrribute.nIsDoubleWelding,
				tAtrribute.nStartWrapType, tAtrribute.nEndWrapType);
		}
		else
		{
			fprintf(pf2, "%d %4d%4d%11.3lf%4d%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%4d%4d%4d%4d%4d%11.3lf%11.3lf%11.3lf%4d\n",
				i, tWeld.IsArc, tWeld.isClockwise, tWeld.ZSide, tWeld.isLeft,
				tWeld.CenterPoint.x, tWeld.CenterPoint.y, tWeld.CenterPoint.z - dAdjustZ,
				tWeld.StartPoint.x, tWeld.StartPoint.y, tWeld.StartPoint.z - dAdjustZ,
				tWeld.EndPoint.x, tWeld.EndPoint.y, tWeld.EndPoint.z - dAdjustZ,
				tWeld.StartNormalVector.x, tWeld.StartNormalVector.y, tWeld.StartNormalVector.z,
				tWeld.EndNormalVector.x, tWeld.EndNormalVector.y, tWeld.EndNormalVector.z,
				tWeld.StartPointType, tWeld.EndPointType,
				tAtrribute.nIsDoubleWelding, tAtrribute.nStartWrapType, tAtrribute.nEndWrapType,
				tAtrribute.nWeldAngleSize, tAtrribute.dStartHoleSize, tAtrribute.dEndHoleSize, tAtrribute.nRobotSelete);
		}
		m_vtWeldSeamData.push_back(tWeld);
	}
	fclose(pf);
	fclose(pf2);

	return true;
}
bool WeldAfterMeasure::LoadCloudProcessResultNew(CString sFileName/* = ""*/)
{
    CString sOutFileName =  OUTPUT_PATH + m_ptUnit->m_tContralUnit.strUnitName + "\\" + IDENTIFY_RESULT_SAVE;
	CString sOutFileNameOLd = OUTPUT_PATH + m_ptUnit->m_tContralUnit.strUnitName + "\\" + IDENTIFY_RESULT_SAVE_OLD;
	if ("" != sFileName)
	{
		sOutFileName = sFileName;
	}
    FILE *pf = fopen(sOutFileName.GetBuffer(), "r");
	//FILE* pf2 = fopen(sOutFileNameOLd.GetBuffer(), "w");
    if (NULL == pf)
    {
		XUI::MesBox::PopOkCancel("���ص��ƴ������ļ� {0} ��ʧ��", sOutFileName);
        return false;
    }
    m_vtWeldSeamData.clear();
	m_vtWeldSeamInfo.clear();

    int nIdx = 0;
    bool m = 0;
    LineOrCircularArcWeldingLine tWeld;
	WeldLineInfo tWeldInfo;
	int i = 0;
	double tmp;
	while (EOF != fscanf(pf, "%d %d %d %lf %d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %d %d %d %d %d %lf %lf %lf %d %lf %lf %lf %lf",
		&nIdx, &tWeld.IsArc,
		(int*)(void*)&tWeld.isClockwise,
		&tWeld.ZSide,
		(int*)(void*)&tWeld.isLeft,
        &tWeld.CenterPoint.x, &tWeld.CenterPoint.y, &tWeld.CenterPoint.z,
        &tWeld.StartPoint.x, &tWeld.StartPoint.y, &tWeld.StartPoint.z,
        &tWeld.EndPoint.x, &tWeld.EndPoint.y, &tWeld.EndPoint.z,          
        &tWeld.StartNormalVector.x, &tWeld.StartNormalVector.y, &tWeld.StartNormalVector.z,
        &tWeld.EndNormalVector.x, &tWeld.EndNormalVector.y, &tWeld.EndNormalVector.z,
        &tWeld.StartPointType, &tWeld.EndPointType,
		&tWeldInfo.tAtrribute.nIsDoubleWelding, 
		&tWeldInfo.tAtrribute.nStartWrapType, 
		&tWeldInfo.tAtrribute.nEndWrapType,
		&tWeldInfo.tAtrribute.nWeldAngleSize, 
		&tWeldInfo.tAtrribute.dStartHoleSize, 
		&tWeldInfo.tAtrribute.dEndHoleSize, 
		&tWeldInfo.tAtrribute.nRobotSelete,
		&tmp, &tmp, &tmp, &tmp))
    {
		tWeldInfo.tWeldLine = tWeld;
        m_vtWeldSeamData.push_back(tWeld);
		m_vtWeldSeamInfo.push_back(tWeldInfo);
		//// ����ӿڹ���־��ʹ��
		//fprintf(pf2, "%d %4d%4d%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%4d%4d%4d%4d%4d\n",
		//	i, tWeld.IsArc, tWeld.isClockwise, tWeld.ZSide,
		//	tWeld.CenterPoint.x, tWeld.CenterPoint.y, tWeld.CenterPoint.z,
		//	tWeld.StartPoint.x, tWeld.StartPoint.y, tWeld.StartPoint.z,
		//	tWeld.EndPoint.x, tWeld.EndPoint.y, tWeld.EndPoint.z,
		//	tWeld.StartNormalVector.x, tWeld.StartNormalVector.y, tWeld.StartNormalVector.z,
		//	tWeld.EndNormalVector.x, tWeld.EndNormalVector.y, tWeld.EndNormalVector.z,
		//	tWeld.StartPointType, tWeld.EndPointType, tWeldInfo.tAtrribute.nIsDoubleWelding,
		//	tWeldInfo.tAtrribute.nStartWrapType, tWeldInfo.tAtrribute.nEndWrapType);
		i++;
    }
    fclose(pf);
	//fclose(pf2);
    return true;
}

bool WeldAfterMeasure::LoadCloudProcessResult(CString sFileName, vector<WeldLineInfo>& vtWeldLineInfo)
{
	vtWeldLineInfo.clear();
	LineOrCircularArcWeldingLine tWeld;
	WeldLineInfo tWeldInfo;
	int nIdx = 0;

	FILE* pf = fopen(sFileName.GetBuffer(), "r");
	if (NULL == pf)
	{
		XUI::MesBox::PopOkCancel("���ص��ƴ������ļ� {0} ��ʧ��", sFileName);
		return false;
	}

	while (EOF != fscanf(pf, "%d%d%d%lf %d %lf%lf%lf %lf%lf%lf %lf%lf%lf %lf%lf%lf %lf%lf%lf %d%d %d%d%d %lf%lf%lf%d%lf",
		&nIdx, &tWeld.IsArc,
		(int*)(void*)&tWeld.isClockwise,
		&tWeld.ZSide,
		(int*)(void*)&tWeld.isLeft,
		&tWeld.CenterPoint.x, &tWeld.CenterPoint.y, &tWeld.CenterPoint.z,
		&tWeld.StartPoint.x, &tWeld.StartPoint.y, &tWeld.StartPoint.z,
		&tWeld.EndPoint.x, &tWeld.EndPoint.y, &tWeld.EndPoint.z,
		&tWeld.StartNormalVector.x, &tWeld.StartNormalVector.y, &tWeld.StartNormalVector.z,
		&tWeld.EndNormalVector.x, &tWeld.EndNormalVector.y, &tWeld.EndNormalVector.z,
		&tWeld.StartPointType, &tWeld.EndPointType,
		&tWeldInfo.tAtrribute.nIsDoubleWelding,
		&tWeldInfo.tAtrribute.nStartWrapType,
		&tWeldInfo.tAtrribute.nEndWrapType,
		&tWeldInfo.tAtrribute.nWeldAngleSize,
		&tWeldInfo.tAtrribute.dStartHoleSize,
		&tWeldInfo.tAtrribute.dEndHoleSize,
		&tWeldInfo.tAtrribute.nRobotSelete,
		&tWeldInfo.tAtrribute.dThoeryLength))
	{
		tWeldInfo.tWeldLine = tWeld;
		vtWeldLineInfo.push_back(tWeldInfo);
	}
	fclose(pf);
	return vtWeldLineInfo.size() > 0 ? true : false;
}

bool WeldAfterMeasure::SplitCloudProcessResult(CString sFileName/* = ""*/)
{
	CString saveFileName = OUTPUT_PATH + sFileName + "\\" + IDENTIFY_RESULT_SAVE;
	CString saveFileNameOld = OUTPUT_PATH + sFileName + "\\" + IDENTIFY_RESULT_SAVE_OLD;
	FILE* pf = fopen(saveFileName.GetBuffer(), "r");
	if (NULL == pf)
	{
		XUI::MesBox::PopOkCancel("���ص��ƴ������ļ� {0} ��ʧ��", saveFileName);
		return false;
	}

	std::vector<LineOrCircularArcWeldingLine> vtWeldSeamData, vtWeldSeamDataLine; // ���ƴ���õ��ĺ�����Ϣ
	std::vector<WeldLineInfo> vtWeldSeamInfo, vtWeldSeamInfoLine; // ����ʶ���������ŵ��޷�ʶ���������Ϣ
	vtWeldSeamData.clear();
	vtWeldSeamInfo.clear();
	vtWeldSeamDataLine.clear();
	vtWeldSeamInfoLine.clear();

	int nIdx = 0;
	bool m = 0;
	double tmp;
	LineOrCircularArcWeldingLine tWeld;
	WeldLineInfo tWeldInfo;
	while (EOF != fscanf(pf, "%d%d%d%lf %d %lf%lf%lf %lf%lf%lf %lf%lf%lf %lf%lf%lf %lf%lf%lf %d%d %d%d%d %lf%lf%lf%d%lf%lf%lf%lf",
		&nIdx, &tWeld.IsArc, (int*)(void*)&tWeld.isClockwise, &tWeld.ZSide, (int*)(void*)&tWeld.isLeft,
		&tWeld.CenterPoint.x, &tWeld.CenterPoint.y, &tWeld.CenterPoint.z,
		&tWeld.StartPoint.x, &tWeld.StartPoint.y, &tWeld.StartPoint.z,
		&tWeld.EndPoint.x, &tWeld.EndPoint.y, &tWeld.EndPoint.z,
		&tWeld.StartNormalVector.x, &tWeld.StartNormalVector.y, &tWeld.StartNormalVector.z,
		&tWeld.EndNormalVector.x, &tWeld.EndNormalVector.y, &tWeld.EndNormalVector.z,
		&tWeld.StartPointType, &tWeld.EndPointType,
		&tWeldInfo.tAtrribute.nIsDoubleWelding, &tWeldInfo.tAtrribute.nStartWrapType, &tWeldInfo.tAtrribute.nEndWrapType,
		&tWeldInfo.tAtrribute.nWeldAngleSize, &tWeldInfo.tAtrribute.dStartHoleSize, &tWeldInfo.tAtrribute.dEndHoleSize, &tWeldInfo.tAtrribute.nRobotSelete, &tmp, &tmp, &tmp, &tmp))
		//&nDoubleWeld, &nWrapTypeS, &nWrapTypeE,
		//&nWeldSize, &dWeldHoleSizeS, &dWeldHoleSizeE, &nBelongRobot))
	{
		if (WeldingLineIsArc(tWeld))
		{
			tWeldInfo.tWeldLine = tWeld;
			vtWeldSeamData.push_back(tWeld);
			vtWeldSeamInfo.push_back(tWeldInfo);
		}	
		else
		{
			tWeldInfo.tWeldLine = tWeld;
			vtWeldSeamDataLine.push_back(tWeld);
			vtWeldSeamInfoLine.push_back(tWeldInfo);
		}
	}
	fclose(pf);
	SaveSplitAfterPointCloudProcessResult(vtWeldSeamInfo, saveFileNameOld,true);
	SaveSplitAfterPointCloudProcessResult(vtWeldSeamInfoLine, saveFileName);
	return true;
}

#if 0
void WeldAfterMeasure::DetermineWeldingMode()
{
	int nGroupNumber = m_vvtWeldSeamGroup.size();
	for (int nGroupNo = 0; nGroupNo < nGroupNumber; nGroupNo++)
	{
		for (int nWeldSeamNo = 0; nWeldSeamNo < m_vvtWeldSeamGroup[nGroupNo].size(); nWeldSeamNo++)
		{
			//�жϷ�������ݺ��ӷ�ʽ������/�Ȳ�󺸣���
			//�ж��������ݶ���������1��ֱ�쳤�ȴ���200mm���Ǳպϻ��ߣ��պ�Բ���뾶����200mm���������١�
			// ������/���ɣ�������ʼ�ε��ƻ�ȡ��ʼ������۹켣���м�㶨��������ȡ��ʼ�켣��
			// �յ����ɣ��ݶ����ٹ������������жϽ�β�㣩������/�м��ʹ�����۳����Ƶ���β����
			//
			m_vvtWeldLineInfoGroup.at(nGroupNo).at(nWeldSeamNo).tAtrribute.bWeldMode = false;
			double dLength = TwoPointDis(m_vvtWeldSeamGroup[nGroupNo][nWeldSeamNo].StartPoint.x, m_vvtWeldSeamGroup[nGroupNo][nWeldSeamNo].StartPoint.y, m_vvtWeldSeamGroup[nGroupNo][nWeldSeamNo].StartPoint.z,
				m_vvtWeldSeamGroup[nGroupNo][nWeldSeamNo].EndPoint.x, m_vvtWeldSeamGroup[nGroupNo][nWeldSeamNo].EndPoint.y, m_vvtWeldSeamGroup[nGroupNo][nWeldSeamNo].EndPoint.z);
					
			if (m_vvtWeldSeamGroup[nGroupNo][nWeldSeamNo].IsArc) // Բ��
			{
				m_vvtWeldLineInfoGroup.at(nGroupNo).at(nWeldSeamNo).tAtrribute.bWeldMode = true;
				double dRadius = TwoPointDis(m_vvtWeldSeamGroup[nGroupNo][nWeldSeamNo].StartPoint.x, m_vvtWeldSeamGroup[nGroupNo][nWeldSeamNo].StartPoint.y, m_vvtWeldSeamGroup[nGroupNo][nWeldSeamNo].StartPoint.z,
					m_vvtWeldSeamGroup[nGroupNo][nWeldSeamNo].CenterPoint.x, m_vvtWeldSeamGroup[nGroupNo][nWeldSeamNo].CenterPoint.y, m_vvtWeldSeamGroup[nGroupNo][nWeldSeamNo].CenterPoint.z);
				if (dLength < 1.0 && dRadius < 800.0)
				{
					m_vvtWeldLineInfoGroup.at(nGroupNo).at(nWeldSeamNo).tAtrribute.bWeldMode = false;
				}
				if (dLength>1.0)
				{
					m_vvtWeldLineInfoGroup.at(nGroupNo).at(nWeldSeamNo).tAtrribute.dThoeryLength -= 6;
				}
			}
			else // ���ɳ�ֱ��
			{
				if (E_STAND_SEAM == GetWeldSeamType(m_vvtWeldSeamGroup[nGroupNo][nWeldSeamNo]))
				{
					m_vvtWeldLineInfoGroup.at(nGroupNo).at(nWeldSeamNo).tAtrribute.bWeldMode = false;
					m_vvtWeldLineInfoGroup.at(nGroupNo).at(nWeldSeamNo).tAtrribute.nStartWrapType = 10;
					continue;
				}
				if (dLength > 300.0)
				{
					m_vvtWeldLineInfoGroup.at(nGroupNo).at(nWeldSeamNo).tAtrribute.bWeldMode = true;
				}
			}

			m_vvtWeldLineInfoGroup.at(nGroupNo).at(nWeldSeamNo).tAtrribute.dThoeryLength = 500.0/*dLength*/;
			// ������/���ɣ�������ʼ�ε��ƻ�ȡ��ʼ������۹켣���м�㶨��������ȡ��ʼ�켣��
			//m_vvtWeldLineInfoGroup.at(nGroupNo).at(nWeldSeamNo).tAtrribute.bStartFixScan = 2 == m_vvtWeldSeamGroup[nGroupNo][nWeldSeamNo].StartPointType ? true : false;
			// �յ����ɣ��ݶ����ٹ������������жϽ�β�㣩������/�м��ʹ�����۳����Ƶ���β����
			//m_vvtWeldLineInfoGroup.at(nGroupNo).at(nWeldSeamNo).tAtrribute.bEndFixScan = 0 == m_vvtWeldSeamGroup[nGroupNo][nWeldSeamNo].EndPointType ? false : true;
			// ������/���ɣ�������ʼ�ε��ƻ�ȡ��ʼ������۹켣���м�㶨��������ȡ��ʼ�켣��
			m_vvtWeldLineInfoGroup.at(nGroupNo).at(nWeldSeamNo).tAtrribute.bStartFixScan = (false ==  m_vvtWeldSeamGroup[nGroupNo][nWeldSeamNo].StartPointType) ? false : true;
			// �յ����ɣ��ݶ����ٹ������������жϽ�β�㣩������/�м��ʹ�����۳����Ƶ���β����
			m_vvtWeldLineInfoGroup.at(nGroupNo).at(nWeldSeamNo).tAtrribute.bEndFixScan = false == m_vvtWeldSeamGroup[nGroupNo][nWeldSeamNo].EndPointType ? false : true;

			// ������Ϊ�м�㣨��Ӹ���򻺺������յ��������ɣ���������㣬��ǰ���յ㣨����������������
		}
	}
}
#endif // 0
void WeldAfterMeasure::DetermineWeldingMode()
{
	int nGroupNumber = m_vvtWeldSeamGroup.size();
	for (int nGroupNo = 0; nGroupNo < nGroupNumber; nGroupNo++)
	{
		for (int nWeldSeamNo = 0; nWeldSeamNo < m_vvtWeldSeamGroup[nGroupNo].size(); nWeldSeamNo++)
		{
			//�жϷ�������ݺ��ӷ�ʽ������/�Ȳ�󺸣���
			//�ж��������ݶ���������1�����ɳ�ֱ�쳤�ȴ���1000mm���Ǳպϻ��ߣ��պ�Բ���뾶����200mm��
			m_vvtWeldLineInfoGroup.at(nGroupNo).at(nWeldSeamNo).tAtrribute.bWeldMode = false;
			double dLength = TwoPointDis(m_vvtWeldSeamGroup[nGroupNo][nWeldSeamNo].StartPoint.x, m_vvtWeldSeamGroup[nGroupNo][nWeldSeamNo].StartPoint.y, m_vvtWeldSeamGroup[nGroupNo][nWeldSeamNo].StartPoint.z,
				m_vvtWeldSeamGroup[nGroupNo][nWeldSeamNo].EndPoint.x, m_vvtWeldSeamGroup[nGroupNo][nWeldSeamNo].EndPoint.y, m_vvtWeldSeamGroup[nGroupNo][nWeldSeamNo].EndPoint.z);
					
			if (WeldingLineIsArc(m_vvtWeldSeamGroup[nGroupNo][nWeldSeamNo])) // Բ��
			{
				m_vvtWeldLineInfoGroup.at(nGroupNo).at(nWeldSeamNo).tAtrribute.bWeldMode = true;
				double dRadius = TwoPointDis(m_vvtWeldSeamGroup[nGroupNo][nWeldSeamNo].StartPoint.x, m_vvtWeldSeamGroup[nGroupNo][nWeldSeamNo].StartPoint.y, m_vvtWeldSeamGroup[nGroupNo][nWeldSeamNo].StartPoint.z,
					m_vvtWeldSeamGroup[nGroupNo][nWeldSeamNo].CenterPoint.x, m_vvtWeldSeamGroup[nGroupNo][nWeldSeamNo].CenterPoint.y, m_vvtWeldSeamGroup[nGroupNo][nWeldSeamNo].CenterPoint.z);
				if (dLength < 1.0 && dRadius < 200.0)
				{
					m_vvtWeldLineInfoGroup.at(nGroupNo).at(nWeldSeamNo).tAtrribute.bWeldMode = false;
				}
				
			}
			else // ���ɳ�ֱ��
			{
				if (E_STAND_SEAM == GetWeldSeamType(m_vvtWeldSeamGroup[nGroupNo][nWeldSeamNo]))
				{
					m_vvtWeldLineInfoGroup.at(nGroupNo).at(nWeldSeamNo).tAtrribute.bWeldMode = false;
					m_vvtWeldLineInfoGroup.at(nGroupNo).at(nWeldSeamNo).tAtrribute.nStartWrapType = 10;
					continue;
				}			
				if (dLength > 300.0)
				{
					m_vvtWeldLineInfoGroup.at(nGroupNo).at(nWeldSeamNo).tAtrribute.bWeldMode = true;
				}
			}

			////jwq ��ʱ���ж��Ƿ��Ǳպ�Բ��
			//double dDis = TwoPointDis(m_vvtWeldLineInfoGroup.at(nGroupNo).at(nWeldSeamNo).tWeldLine.StartPoint.x,
			//	m_vvtWeldLineInfoGroup.at(nGroupNo).at(nWeldSeamNo).tWeldLine.StartPoint.y,
			//	m_vvtWeldLineInfoGroup.at(nGroupNo).at(nWeldSeamNo).tWeldLine.StartPoint.z,
			//	m_vvtWeldLineInfoGroup.at(nGroupNo).at(nWeldSeamNo).tWeldLine.EndPoint.x,
			//	m_vvtWeldLineInfoGroup.at(nGroupNo).at(nWeldSeamNo).tWeldLine.EndPoint.y,
			//	m_vvtWeldLineInfoGroup.at(nGroupNo).at(nWeldSeamNo).tWeldLine.EndPoint.z);

			//// ���ҹ���ȫ���Ȳ��,����ɾ��
			//if (dDis > 1.0)
			//{
			//	m_vvtWeldLineInfoGroup.at(nGroupNo).at(nWeldSeamNo).tAtrribute.bWeldMode = false;
			//}
			//else
			//{
			//	m_vvtWeldLineInfoGroup.at(nGroupNo).at(nWeldSeamNo).tAtrribute.bWeldMode = true;
			//}
			//-------------------------------------------------------------------------------
			m_vvtWeldLineInfoGroup.at(nGroupNo).at(nWeldSeamNo).tAtrribute.dThoeryLength = dLength;
			// ������/���ɣ�������ʼ�ε��ƻ�ȡ��ʼ������۹켣���м�㶨��������ȡ��ʼ�켣��
			m_vvtWeldLineInfoGroup.at(nGroupNo).at(nWeldSeamNo).tAtrribute.bStartFixScan = (false == m_vvtWeldSeamGroup[nGroupNo][nWeldSeamNo].StartPointType) ? false : true;
			// �յ����ɣ��ݶ����ٹ������������жϽ�β�㣩������/�м��ʹ�����۳����Ƶ���β����
			m_vvtWeldLineInfoGroup.at(nGroupNo).at(nWeldSeamNo).tAtrribute.bEndFixScan = false == m_vvtWeldSeamGroup[nGroupNo][nWeldSeamNo].EndPointType ? false : true;

			// ������Ϊ�м�㣨��Ӹ���򻺺������յ��������ɣ���������㣬��ǰ���յ㣨����������������
		}
	}
}

void WeldAfterMeasure::SaveWeldSeamGroupInfo()
{
	m_vtWeldSeamData.clear();
	CString sOutFileName = m_sDataSavePath + "00-WeldSeamGroup.txt";
	FILE* pf = fopen(sOutFileName.GetBuffer(), "w");
	int nGroupNumber = m_vvtWeldSeamGroup.size();
	for (int nGroupNo = 0; nGroupNo < nGroupNumber; nGroupNo++)
	{
		for (int nWeldSeamNo = 0; nWeldSeamNo < m_vvtWeldSeamGroup[nGroupNo].size(); nWeldSeamNo++)
		{
			LineOrCircularArcWeldingLine tWeld = m_vvtWeldSeamGroup[nGroupNo][nWeldSeamNo];
			fprintf(pf, "%d%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%4d%4d%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%4d%4d%4d\n",
				nGroupNo, tWeld.StartPoint.x, tWeld.StartPoint.y, tWeld.StartPoint.z,
				tWeld.EndPoint.x, tWeld.EndPoint.y, tWeld.EndPoint.z,
				tWeld.ZSide, tWeld.isLeft, tWeld.IsArc, tWeld.CenterPoint.x, tWeld.CenterPoint.y, tWeld.CenterPoint.z,
				tWeld.StartNormalVector.x, tWeld.StartNormalVector.y, tWeld.StartNormalVector.z,
				tWeld.EndNormalVector.x, tWeld.EndNormalVector.y, tWeld.EndNormalVector.z,
				tWeld.StartPointType, tWeld.EndPointType, m_vvtWeldLineInfoGroup.at(nGroupNo).at(nWeldSeamNo).tAtrribute.bWeldMode);
		}
		fprintf(pf, "\n");
	}
	fclose(pf);
}

void WeldAfterMeasure::SaveTeachResult(int nGroupNo)
{
	CString sFileName;
	sFileName.Format("%s%d-TeachResult.txt", m_sDataSavePath, nGroupNo);
	FILE* pf = fopen(sFileName.GetBuffer(), "w");
	for (int i = 0; i < m_vtTeachResult.size(); i++)
	{
		T_TEACH_RESULT tTeachResult = m_vtTeachResult[i];
		// ʾ�̵�� ������������ ��ά�� ��ά�� ��ͼֱ�� ��ͼ�ؽ� ��ͼ�ⲿ��
		// ���� 
		fprintf(pf, "%d%4d%11.3lf%11.3lf%11.3lf%6d%6d%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%10d%10d%10d%10d%10d%10d%11.3lf\n",
			i, 0, tTeachResult.tKeyPtn3D.x, tTeachResult.tKeyPtn3D.y, tTeachResult.tKeyPtn3D.z, tTeachResult.tKeyPtn2D.x, tTeachResult.tKeyPtn2D.y,
			tTeachResult.tRobotCoors.dX, tTeachResult.tRobotCoors.dY, tTeachResult.tRobotCoors.dZ,
			tTeachResult.tRobotCoors.dRX, tTeachResult.tRobotCoors.dRY, tTeachResult.tRobotCoors.dRZ,
			tTeachResult.tRobotPulse.nSPulse, tTeachResult.tRobotPulse.nLPulse, tTeachResult.tRobotPulse.nUPulse,
			tTeachResult.tRobotPulse.nRPulse, tTeachResult.tRobotPulse.nBPulse, tTeachResult.tRobotPulse.nTPulse, tTeachResult.dExAxlePos);
		// ����
		for (int n = 0; n < tTeachResult.vtLeftPtns2D.size(); n++)
		{
			fprintf(pf, "%d%4d%11.3lf%11.3lf%11.3lf%6d%6d%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%10d%10d%10d%10d%10d%10d%11.3lf\n",
				i, 1, tTeachResult.vtLeftPtns3D[n].x, tTeachResult.vtLeftPtns3D[n].y, tTeachResult.vtLeftPtns3D[n].z, 
				tTeachResult.vtLeftPtns2D[n].x, tTeachResult.vtLeftPtns2D[n].y,
				tTeachResult.tRobotCoors.dX, tTeachResult.tRobotCoors.dY, tTeachResult.tRobotCoors.dZ,
				tTeachResult.tRobotCoors.dRX, tTeachResult.tRobotCoors.dRY, tTeachResult.tRobotCoors.dRZ,
				tTeachResult.tRobotPulse.nSPulse, tTeachResult.tRobotPulse.nLPulse, tTeachResult.tRobotPulse.nUPulse,
				tTeachResult.tRobotPulse.nRPulse, tTeachResult.tRobotPulse.nBPulse, tTeachResult.tRobotPulse.nTPulse, tTeachResult.dExAxlePos);
		}
		// ����
		for (int n = 0; n < tTeachResult.vtRightPtns2D.size(); n++)
		{
			fprintf(pf, "%d%4d%11.3lf%11.3lf%11.3lf%6d%6d%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%10d%10d%10d%10d%10d%10d%11.3lf\n",
				i, 2, tTeachResult.vtRightPtns3D[n].x, tTeachResult.vtRightPtns3D[n].y, tTeachResult.vtRightPtns3D[n].z,
				tTeachResult.vtRightPtns2D[n].x, tTeachResult.vtRightPtns2D[n].y,
				tTeachResult.tRobotCoors.dX, tTeachResult.tRobotCoors.dY, tTeachResult.tRobotCoors.dZ,
				tTeachResult.tRobotCoors.dRX, tTeachResult.tRobotCoors.dRY, tTeachResult.tRobotCoors.dRZ,
				tTeachResult.tRobotPulse.nSPulse, tTeachResult.tRobotPulse.nLPulse, tTeachResult.tRobotPulse.nUPulse,
				tTeachResult.tRobotPulse.nRPulse, tTeachResult.tRobotPulse.nBPulse, tTeachResult.tRobotPulse.nTPulse, tTeachResult.dExAxlePos);
		}
	}
	fclose(pf);
}

void WAM::WeldAfterMeasure::SaveTeachResultWorld(int nGroupNo)
{
	CString sFileName;
	sFileName.Format("%s%d-TeachResultWorld.txt", m_sDataSavePath, nGroupNo);
	FILE* pf = fopen(sFileName.GetBuffer(), "w");
	for (int i = 0; i < m_vtTeachResult.size(); i++)
	{
		T_TEACH_RESULT tTeachResult = m_vtTeachResult[i];
		// ���� 
		fprintf(pf, "K%d %11.3lf %11.3lf %11.3lf\n",
			i, 
			tTeachResult.tKeyPtn3D.x + tTeachResult.tRobotCoors.dBX,
			tTeachResult.tKeyPtn3D.y + tTeachResult.tRobotCoors.dBY,
			tTeachResult.tKeyPtn3D.z + tTeachResult.tRobotCoors.dBZ);
		// ����
		for (int n = 0; n < tTeachResult.vtLeftPtns2D.size(); n++)
		{
			fprintf(pf, "L%d %11.3lf %11.3lf %11.3lf\n",
				i,
				tTeachResult.vtLeftPtns3D[n].x + tTeachResult.tRobotCoors.dBX,
				tTeachResult.vtLeftPtns3D[n].y + tTeachResult.tRobotCoors.dBY,
				tTeachResult.vtLeftPtns3D[n].z + tTeachResult.tRobotCoors.dBZ);
		}
		// ����
		for (int n = 0; n < tTeachResult.vtRightPtns2D.size(); n++)
		{
			fprintf(pf, "R%d %11.3lf %11.3lf %11.3lf\n",
				i,
				tTeachResult.vtRightPtns3D[n].x + tTeachResult.tRobotCoors.dBX,
				tTeachResult.vtRightPtns3D[n].y + tTeachResult.tRobotCoors.dBY,
				tTeachResult.vtRightPtns3D[n].z + tTeachResult.tRobotCoors.dBZ);
		}
	}
	fclose(pf);
}

bool WeldAfterMeasure::LoadTeachResult(int nGroupNo, int nLayerNo/* = -1*/)
{
	m_vtTeachResult.clear();
	int nDataColNum = 20;
	CString sFileName;
	if (nLayerNo < 0)
	{
		sFileName.Format("%s%d-TeachResult.txt", m_sDataSavePath, nGroupNo);
	}
	else
	{
		sFileName.Format("%s%d-%d-TeachResult.txt", m_sDataSavePath, nLayerNo, nGroupNo);
	}
	FILE* pf = fopen(sFileName.GetBuffer(), "r");
	if (NULL == pf)
	{
		XUI::MesBox::PopOkCancel("���ص�%d��ʾ�̽���ļ� {0} ��ʧ�ܣ�", nGroupNo, sFileName);
		return false;
	}

	T_TEACH_RESULT tTeachResult; 
	T_TEACH_RESULT tTempTeachResult;
	int nPreReadIdx = -1;
	int nCurReadIdx = 0;
	int nPtnType = 0; // 0���� 1���� 2����
	CvPoint tKeyPtn2D;
	XI_POINT tKeyPtn3D;
	int nRst = fscanf(pf, "%d%d%lf%lf%lf%d%d%lf%lf%lf%lf%lf%lf%d%d%d%d%d%d%lf",
		&nCurReadIdx, &nPtnType, &tKeyPtn3D.x, &tKeyPtn3D.y, &tKeyPtn3D.z, &tKeyPtn2D.x, &tKeyPtn2D.y,
		&tTeachResult.tRobotCoors.dX, &tTeachResult.tRobotCoors.dY, &tTeachResult.tRobotCoors.dZ,
		&tTeachResult.tRobotCoors.dRX, &tTeachResult.tRobotCoors.dRY, &tTeachResult.tRobotCoors.dRZ,
		&tTeachResult.tRobotPulse.nSPulse, &tTeachResult.tRobotPulse.nLPulse, &tTeachResult.tRobotPulse.nUPulse,
		&tTeachResult.tRobotPulse.nRPulse, &tTeachResult.tRobotPulse.nBPulse, &tTeachResult.tRobotPulse.nTPulse, 
		&tTeachResult.dExAxlePos);
	while (nDataColNum == nRst)
	{
		if (nPreReadIdx != nCurReadIdx)
		{
			if (nPreReadIdx >= 0)
			{
				m_vtTeachResult.push_back(tTeachResult);
			}
			tTeachResult.vtLeftPtns2D.clear();
			tTeachResult.vtLeftPtns3D.clear();
			tTeachResult.vtRightPtns2D.clear();
			tTeachResult.vtRightPtns3D.clear();
			nPreReadIdx = nCurReadIdx;
		}
		switch (nPtnType)
		{
		case 0: 
			tTeachResult.tKeyPtn2D = tKeyPtn2D; 
			tTeachResult.tKeyPtn3D = tKeyPtn3D; 
			break;
		case 1: 
			tTeachResult.vtLeftPtns2D.push_back(tKeyPtn2D);
			tTeachResult.vtLeftPtns3D.push_back(tKeyPtn3D);
			break;
		case 2:
			tTeachResult.vtRightPtns2D.push_back(tKeyPtn2D);
			tTeachResult.vtRightPtns3D.push_back(tKeyPtn3D);
			break;
		default:
			XUI::MesBox::PopOkCancel("���ص�{0}��ʾ�̽���ļ� {1} ��⴦�������������", nGroupNo, sFileName);
			fclose(pf);
			return false;
		}

		nRst = fscanf(pf, "%d%d%lf%lf%lf%d%d%lf%lf%lf%lf%lf%lf%d%d%d%d%d%d%lf",
			&nCurReadIdx, &nPtnType, &tKeyPtn3D.x, &tKeyPtn3D.y, &tKeyPtn3D.z, &tKeyPtn2D.x, &tKeyPtn2D.y,
			&tTempTeachResult.tRobotCoors.dX, &tTempTeachResult.tRobotCoors.dY, &tTempTeachResult.tRobotCoors.dZ,
			&tTempTeachResult.tRobotCoors.dRX, &tTempTeachResult.tRobotCoors.dRY, &tTempTeachResult.tRobotCoors.dRZ,
			&tTempTeachResult.tRobotPulse.nSPulse, &tTempTeachResult.tRobotPulse.nLPulse, &tTempTeachResult.tRobotPulse.nUPulse,
			&tTempTeachResult.tRobotPulse.nRPulse, &tTempTeachResult.tRobotPulse.nBPulse, &tTempTeachResult.tRobotPulse.nTPulse,
			&tTempTeachResult.dExAxlePos);
		if (-1 == nCurReadIdx) // ����ɨ�赽�Ķ˵����� �� ��1������λ��tEndPtn3D��
		{
			m_vtTeachResult[0].tEndPtn3D = tKeyPtn3D;
			break;
		}
		tTeachResult.tRobotCoors.dX = tTempTeachResult.tRobotCoors.dX;
		tTeachResult.tRobotCoors.dY = tTempTeachResult.tRobotCoors.dY;
		tTeachResult.tRobotCoors.dZ = tTempTeachResult.tRobotCoors.dZ;
		tTeachResult.tRobotCoors.dRX = tTempTeachResult.tRobotCoors.dRX;
		tTeachResult.tRobotCoors.dRY = tTempTeachResult.tRobotCoors.dRY;
		tTeachResult.tRobotCoors.dRZ = tTempTeachResult.tRobotCoors.dRZ;
		tTeachResult.tRobotPulse.nSPulse = tTempTeachResult.tRobotPulse.nSPulse;
		tTeachResult.tRobotPulse.nLPulse = tTempTeachResult.tRobotPulse.nLPulse;
		tTeachResult.tRobotPulse.nUPulse = tTempTeachResult.tRobotPulse.nUPulse;
		tTeachResult.tRobotPulse.nRPulse = tTempTeachResult.tRobotPulse.nRPulse;
		tTeachResult.tRobotPulse.nBPulse = tTempTeachResult.tRobotPulse.nBPulse;
		tTeachResult.tRobotPulse.nTPulse = tTempTeachResult.tRobotPulse.nTPulse;
		tTeachResult.dExAxlePos = tTempTeachResult.dExAxlePos;
	}
	fclose(pf);
	m_vtTeachResult.push_back(tTeachResult); // �������һ��ʾ��λ�ý��

	if ((EOF != nRst) && (nDataColNum != nRst))
	{
		XUI::MesBox::PopOkCancel("���ص�{0}��ʾ�̽�������ļ� {1} ��⵽���������ݣ�", nGroupNo, sFileName);
		m_vtTeachResult.clear();
		return false;
	}
	return true;
}

bool WeldAfterMeasure::LoadTeachResult(int nGroupNo, int nLayerNo, std::vector<T_TEACH_RESULT>& vtTeachResult)
{
	vtTeachResult.clear();
	int nDataColNum = 20;
	CString sFileName;
	sFileName.Format("%s%d-%d-TeachResult.txt", m_sDataSavePath, nLayerNo, nGroupNo);

	FILE* pf = fopen(sFileName.GetBuffer(), "r");
	if (NULL == pf)
	{
		XUI::MesBox::PopOkCancel("���ص�{0}��ʾ�̽���ļ� {1} ��ʧ�ܣ�", nGroupNo, sFileName);
		return false;
	}

	T_TEACH_RESULT tTeachResult;
	T_TEACH_RESULT tTempTeachResult;
	int nPreReadIdx = -1;
	int nCurReadIdx = 0;
	int nPtnType = 0; // 0���� 1���� 2����
	CvPoint tKeyPtn2D;
	XI_POINT tKeyPtn3D;
	int nRst = fscanf(pf, "%d%d%lf%lf%lf%d%d%lf%lf%lf%lf%lf%lf%d%d%d%d%d%d%lf",
		&nCurReadIdx, &nPtnType, &tKeyPtn3D.x, &tKeyPtn3D.y, &tKeyPtn3D.z, &tKeyPtn2D.x, &tKeyPtn2D.y,
		&tTeachResult.tRobotCoors.dX, &tTeachResult.tRobotCoors.dY, &tTeachResult.tRobotCoors.dZ,
		&tTeachResult.tRobotCoors.dRX, &tTeachResult.tRobotCoors.dRY, &tTeachResult.tRobotCoors.dRZ,
		&tTeachResult.tRobotPulse.nSPulse, &tTeachResult.tRobotPulse.nLPulse, &tTeachResult.tRobotPulse.nUPulse,
		&tTeachResult.tRobotPulse.nRPulse, &tTeachResult.tRobotPulse.nBPulse, &tTeachResult.tRobotPulse.nTPulse,
		&tTeachResult.dExAxlePos);
	while (nDataColNum == nRst)
	{
		if (nPreReadIdx != nCurReadIdx)
		{
			if (nPreReadIdx >= 0)
			{
				vtTeachResult.push_back(tTeachResult);
			}
			tTeachResult.vtLeftPtns2D.clear();
			tTeachResult.vtLeftPtns3D.clear();
			tTeachResult.vtRightPtns2D.clear();
			tTeachResult.vtRightPtns3D.clear();
			nPreReadIdx = nCurReadIdx;
		}
		switch (nPtnType)
		{
		case 0:
			tTeachResult.tKeyPtn2D = tKeyPtn2D;
			tTeachResult.tKeyPtn3D = tKeyPtn3D;
			break;
		case 1:
			tTeachResult.vtLeftPtns2D.push_back(tKeyPtn2D);
			tTeachResult.vtLeftPtns3D.push_back(tKeyPtn3D);
			break;
		case 2:
			tTeachResult.vtRightPtns2D.push_back(tKeyPtn2D);
			tTeachResult.vtRightPtns3D.push_back(tKeyPtn3D);
			break;
		default:
			XUI::MesBox::PopOkCancel("���ص�{0}��ʾ�̽���ļ� {1} ��⴦�������������", nGroupNo, sFileName);
			fclose(pf);
			return false;
		}

		nRst = fscanf(pf, "%d%d%lf%lf%lf%d%d%lf%lf%lf%lf%lf%lf%d%d%d%d%d%d%lf",
			&nCurReadIdx, &nPtnType, &tKeyPtn3D.x, &tKeyPtn3D.y, &tKeyPtn3D.z, &tKeyPtn2D.x, &tKeyPtn2D.y,
			&tTempTeachResult.tRobotCoors.dX, &tTempTeachResult.tRobotCoors.dY, &tTempTeachResult.tRobotCoors.dZ,
			&tTempTeachResult.tRobotCoors.dRX, &tTempTeachResult.tRobotCoors.dRY, &tTempTeachResult.tRobotCoors.dRZ,
			&tTempTeachResult.tRobotPulse.nSPulse, &tTempTeachResult.tRobotPulse.nLPulse, &tTempTeachResult.tRobotPulse.nUPulse,
			&tTempTeachResult.tRobotPulse.nRPulse, &tTempTeachResult.tRobotPulse.nBPulse, &tTempTeachResult.tRobotPulse.nTPulse,
			&tTempTeachResult.dExAxlePos);
		if (-1 == nCurReadIdx) // ����ɨ�赽�Ķ˵����� �� ��1������λ��tEndPtn3D��
		{
			vtTeachResult[0].tEndPtn3D = tKeyPtn3D;
			break;
		}
		tTeachResult.tRobotCoors.dX = tTempTeachResult.tRobotCoors.dX;
		tTeachResult.tRobotCoors.dY = tTempTeachResult.tRobotCoors.dY;
		tTeachResult.tRobotCoors.dZ = tTempTeachResult.tRobotCoors.dZ;
		tTeachResult.tRobotCoors.dRX = tTempTeachResult.tRobotCoors.dRX;
		tTeachResult.tRobotCoors.dRY = tTempTeachResult.tRobotCoors.dRY;
		tTeachResult.tRobotCoors.dRZ = tTempTeachResult.tRobotCoors.dRZ;
		tTeachResult.tRobotPulse.nSPulse = tTempTeachResult.tRobotPulse.nSPulse;
		tTeachResult.tRobotPulse.nLPulse = tTempTeachResult.tRobotPulse.nLPulse;
		tTeachResult.tRobotPulse.nUPulse = tTempTeachResult.tRobotPulse.nUPulse;
		tTeachResult.tRobotPulse.nRPulse = tTempTeachResult.tRobotPulse.nRPulse;
		tTeachResult.tRobotPulse.nBPulse = tTempTeachResult.tRobotPulse.nBPulse;
		tTeachResult.tRobotPulse.nTPulse = tTempTeachResult.tRobotPulse.nTPulse;
		tTeachResult.dExAxlePos = tTempTeachResult.dExAxlePos;
	}
	fclose(pf);
	vtTeachResult.push_back(tTeachResult); // �������һ��ʾ��λ�ý��

	if ((EOF != nRst) && (nDataColNum != nRst))
	{
		XUI::MesBox::PopOkCancel("���ص�{0]��ʾ�̽�������ļ� {1} ��⵽���������ݣ�", nGroupNo, sFileName);
		vtTeachResult.clear();
		return false;
	}
	return true;
}

bool WeldAfterMeasure::LoadRealWeldTrack(int nGroupNo, int nWeldNo, E_WELD_SEAM_TYPE& eWeldSeamType, double& dExAxlePos, vector<T_ROBOT_COORS>& vtRealWeldTrack, vector<int>& vnPtnType)
{
	double dExAxlePos_y;
	int nDataColNum = 11;
	vtRealWeldTrack.clear();
	vnPtnType.clear();
	CString sFileName;
	sFileName.Format("%s%d_%d_RealWeldCoord.txt", m_sDataSavePath.GetBuffer(), nGroupNo, nWeldNo);
	FILE* pf = fopen(sFileName.GetBuffer(), "r");
	if (NULL == pf)
	{
		//XiMessageBox("���ص�%d�� ��%d������켣ʧ��", nGroupNo, nWeldNo);
		return false;
	}
	// ��� ֱ������ �ⲿ������ ��������
	int nIdx = 0;
	int nWeldSeamType = 0;
	T_ROBOT_COORS tCoord;
	int nPtnType = 0;
	int nRst = fscanf(pf, "%d%lf%lf%lf%lf%lf%lf%lf%lf%d%d", &nIdx,
		&tCoord.dX, &tCoord.dY, &tCoord.dZ,
		&tCoord.dRX, &tCoord.dRY, &tCoord.dRZ, &dExAxlePos, &dExAxlePos_y, &nWeldSeamType, &nPtnType);
#ifdef SINGLE_ROBOT
	tCoord.dBY = dExAxlePos;
#else
	tCoord.dBX = dExAxlePos;
	tCoord.dBY = dExAxlePos_y;
#endif
	eWeldSeamType = (E_WELD_SEAM_TYPE)nWeldSeamType;
	while (nDataColNum == nRst)
	{
		vtRealWeldTrack.push_back(tCoord);
		vnPtnType.push_back(nPtnType);
		nRst = fscanf(pf, "%d%lf%lf%lf%lf%lf%lf%lf%lf%d%d", &nIdx,
			&tCoord.dX, &tCoord.dY, &tCoord.dZ,
			&tCoord.dRX, &tCoord.dRY, &tCoord.dRZ, &dExAxlePos, &dExAxlePos_y , &nWeldSeamType, &nPtnType);
#ifdef SINGLE_ROBOT
		tCoord.dBY = dExAxlePos;
#else
		tCoord.dBX = dExAxlePos;
		tCoord.dBY = dExAxlePos_y;
#endif
	}
	if ((EOF != nRst) && (nDataColNum != nRst))
	{
		XUI::MesBox::PopOkCancel("���ص�{0}�� ��{1}������ ��⵽����������", nGroupNo, nWeldNo);
		return false;
	}
	fclose(pf);

	dExAxlePos = vtRealWeldTrack[0].dBX;

	return true;
}

bool WeldAfterMeasure::CheckFlangeToolCoord(int nGroupNo, int nWeldNo, vector<T_ANGLE_PULSE> vtRobotPulse)
{
	bool bRst = true;
	double dDis = 0.0;
	double dDisThreshold = 200.0;
	CString sFileName;
	sFileName.Format(".\\SteelStructure\\%d_%d_FlangeCoord.txt", nGroupNo, nWeldNo);
	FILE* pf = fopen(sFileName.GetBuffer(), "w");
	T_ROBOT_COORS tFlangeTool(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	T_ROBOT_COORS tCoord;
	for (int i = 0; i < vtRobotPulse.size(); i++)
	{
		m_pRobotDriver->RobotKinematics(vtRobotPulse[i], tFlangeTool, tCoord);
		dDis = TwoPointDis(0.0, 0.0, tCoord.dX, tCoord.dY);
		bRst &= (dDis > dDisThreshold);
		fprintf(pf, "%d%11.3lf%11.3lf%11.3lf Dis:%11.3lf Threshole:%11.3lf\n", i, tCoord.dX, tCoord.dY, tCoord.dZ, dDis, dDisThreshold);
	}
	fclose(pf);
	return bRst;
}

bool WAM::WeldAfterMeasure::AddMeasurePointsToLongSeam(int nWeldNo, std::vector<std::vector<T_TEACH_DATA>> vvtTeachData, LineOrCircularArcWeldingLine tWeldSeam,
														double dPtnInterval, double dPostureRx, double dPostureRy, double dWeldDirAngle, 
														E_WELD_SEAM_TYPE eWeldSeamType, vector<T_ROBOT_COORS>& vtWeldCoord)
{
	std::vector<T_TEACH_DATA> tTempTeachData(vvtTeachData[nWeldNo]);
	std::vector<T_TEACH_DATA> tTeachData;
	std::vector<XI_POINT> vtKeyPtn3D;
	XI_POINT tStartPoint;
	tStartPoint.x = tWeldSeam.StartPoint.x;
	tStartPoint.y = tWeldSeam.StartPoint.y;
	tStartPoint.z = tWeldSeam.StartPoint.z;
	for (int No = 0; No < tTempTeachData.size(); No++) {
		if (tTempTeachData[No].eAttribute == E_BELONG_MIDDLE) {
			tTeachData.push_back(tTempTeachData[No]);
		}
	}
	if (tTeachData.size() > 0 && m_vtTeachResult.size() > 0) {
		// ��ȡtTeachData��Ӧ�����в����������
		for (int No = 0; No < tTeachData.size(); No++)
		{
			if (m_vtTeachResult[tTeachData[No].nMeasurePtnNo].tKeyPtn3D.x > 900000.0 ||
				m_vtTeachResult[tTeachData[No].nMeasurePtnNo].tKeyPtn3D.y > 900000.0 ||
				m_vtTeachResult[tTeachData[No].nMeasurePtnNo].tKeyPtn3D.z > 900000.0)
			{
				continue;
			}
			vtKeyPtn3D.push_back(m_vtTeachResult[tTeachData[No].nMeasurePtnNo].tKeyPtn3D);
		}

		bool flag = false;
		CvPoint3D64f tmpStartPoint;
		CvPoint3D64f tmpEndPoint;
		T_ROBOT_COORS tTempCoors = vtWeldCoord.back();
		std::vector<T_ROBOT_COORS> vtRobotCoors;
		tmpStartPoint.x = vtWeldCoord[0].dX;
		tmpStartPoint.y = vtWeldCoord[0].dY;
		tmpStartPoint.z = vtWeldCoord[0].dZ;
		vtWeldCoord.clear();
		sort(vtKeyPtn3D.begin(), vtKeyPtn3D.end(),
			[tStartPoint](const XI_POINT& tPtn1, const XI_POINT& tPtn2) -> bool
		{
			double dDis1 = TwoPointDis(tStartPoint.x, tStartPoint.y, tStartPoint.z, tPtn1.x, tPtn1.y, tPtn1.z);
			double dDis2 = TwoPointDis(tStartPoint.x, tStartPoint.y, tStartPoint.z, tPtn2.x, tPtn2.y, tPtn2.z);
			return (dDis1 < dDis2);
		});

		for (int No = 0; No < vtKeyPtn3D.size(); No++) {

			tmpEndPoint.x = vtKeyPtn3D[No].x;
			tmpEndPoint.y = vtKeyPtn3D[No].y;
			tmpEndPoint.z = vtKeyPtn3D[No].z;
			if (false == GenerateWeldLineData(tmpStartPoint, tmpEndPoint, dPtnInterval, dPostureRx, dPostureRy, dWeldDirAngle, eWeldSeamType, vtRobotCoors))
			{
				XiMessageBox("������յ����꼰��̬����ֱ�����꺸��켣����ʧ��");
				return false;
			}
			tmpStartPoint.x = vtKeyPtn3D[No].x;
			tmpStartPoint.y = vtKeyPtn3D[No].y;
			tmpStartPoint.z = vtKeyPtn3D[No].z;
			if (No > 0) {
				vtRobotCoors.erase(vtRobotCoors.begin());
			}
			vtWeldCoord.insert(vtWeldCoord.end(), vtRobotCoors.begin(), vtRobotCoors.end());
			vtRobotCoors.clear();
		}
		tmpEndPoint.x = tTempCoors.dX;
		tmpEndPoint.y = tTempCoors.dY;
		tmpEndPoint.z = tTempCoors.dZ;
		if (false == GenerateWeldLineData(tmpStartPoint, tmpEndPoint, dPtnInterval, dPostureRx, dPostureRy, dWeldDirAngle, eWeldSeamType, vtRobotCoors))
		{
			XiMessageBox("������յ����꼰��̬����ֱ�����꺸��켣����ʧ��");
			return false;
		}
		vtRobotCoors.erase(vtRobotCoors.begin());
		vtWeldCoord.insert(vtWeldCoord.end(), vtRobotCoors.begin(), vtRobotCoors.end());
		vtRobotCoors.clear();

		CString sFileName;
		sFileName.Format("temp.txt");
		FILE* pf = fopen(sFileName, "w");
		for (int nPtnIdx = 0; nPtnIdx < vtWeldCoord.size(); nPtnIdx++)
		{
			T_ROBOT_COORS& tCoord = vtWeldCoord[nPtnIdx];
			fprintf(pf, "%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf\n", vtWeldCoord[nPtnIdx].dX, vtWeldCoord[nPtnIdx].dY, vtWeldCoord[nPtnIdx].dZ, vtWeldCoord[nPtnIdx].dRX, vtWeldCoord[nPtnIdx].dRY, vtWeldCoord[nPtnIdx].dRZ);
		}
		fclose(pf);
	}
	return false;
}

void WeldAfterMeasure::SaveWeldTrack(int nGroupNo, int nWeldNo, int nLayerNo, E_WELD_SEAM_TYPE eWeldSeamType, double dExAxlePos, vector<T_ROBOT_COORS> vtWeldCoord, vector<int> vnPtnType)
{
	// ���� ��� ֱ������ �ⲿ������ ��������
	CString sFileName;
	CString sFileName2;
	sFileName.Format("%s%d_%d_%d_AdjustRealWeldCoord.txt", m_sDataSavePath, nGroupNo, nWeldNo, nLayerNo);
	sFileName2.Format("%s%d_%d_%d_AdjustRealWeldCoordWorld.txt", m_sDataSavePath, nGroupNo, nWeldNo, nLayerNo);
	FILE* pf = fopen(sFileName, "w");
	FILE* pf2 = fopen(sFileName2, "w");
	for (int nPtnIdx = 0; nPtnIdx < vtWeldCoord.size(); nPtnIdx++)
	{
		T_ROBOT_COORS& tCoord = vtWeldCoord[nPtnIdx];
		fprintf(pf, "%d%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%4d%4d\n", nPtnIdx,
			tCoord.dX, tCoord.dY, tCoord.dZ, tCoord.dRX, tCoord.dRY, tCoord.dRZ, 
			tCoord.dBX, tCoord.dBY, tCoord.dBZ, m_dTeachExAxlePos, eWeldSeamType, vnPtnType[nPtnIdx]);

		fprintf(pf2, "%d%11.3lf%11.3lf%11.3lf%4d%4d\n", nPtnIdx,
			tCoord.dX + tCoord.dBX, tCoord.dY + tCoord.dBY, tCoord.dZ + tCoord.dBZ, eWeldSeamType, vnPtnType[nPtnIdx]);
	}

	fclose(pf);
	fclose(pf2);
}

bool WeldAfterMeasure::JudgeDirAngle(double dDir1, double dDir2, double dThreshold) const
{
	dDir1 = fmod(dDir1, 360.0);
	dDir2 = fmod(dDir2, 360.0);
	dDir1 = dDir1 < 0.0 ? dDir1 + 360.0 : dDir1;
	dDir2 = dDir2 < 0.0 ? dDir2 + 360.0 : dDir2;
	double dDiff = fabs(dDir1 - dDir2);
	if (dDiff > 180.0)
	{
		dDiff -= 360.0;
	}
	return (fabs(dDiff) > fabs(dThreshold) ? false : true);
}

bool WeldAfterMeasure::JudgeAngle(double dPositiveLimit, double dNegativeLimit, double dAngle1, double dAngle2, double dThreshold) const
{
	dAngle1 = fmod(dAngle1, 360.0);
	dAngle2 = fmod(dAngle2, 360.0);
	dAngle1 = dAngle1 > dPositiveLimit ? dAngle1 - 360.0 : dAngle1;  // dNegativeLimit - dPositiveLimit
	dAngle1 = dAngle1 < dNegativeLimit ? dAngle1 + 360.0 : dAngle1;
	dAngle2 = dAngle2 > dPositiveLimit ? dAngle2 - 360.0 : dAngle2;
	dAngle2 = dAngle2 < dNegativeLimit ? dAngle2 + 360.0 : dAngle2;
	double dDiff = fabs(dAngle1 - dAngle2);
	if (dDiff > 180.0)
	{
		dDiff -= 360.0;
	}
	return (fabs(dDiff) > fabs(dThreshold) ? false : true);
}

double WeldAfterMeasure::TwoDirAngleMedian(double dDirAngle1, double dDirAngle2)
{
	XiAlgorithm alg;
	return alg.CalcArcAngle(CosD(dDirAngle1) + CosD(dDirAngle2), SinD(dDirAngle1) + SinD(dDirAngle2));
}

double WeldAfterMeasure::TwoAngleMedian(double dPositiveLimit, double dNegativeLimit, double dAngle1, double dAngle2)
{
	dAngle1 = fmod(dAngle1, 360.0);
	dAngle2 = fmod(dAngle2, 360.0);
	dAngle1 = dAngle1 > dPositiveLimit ? dAngle1 - 360.0 : dAngle1;  // dNegativeLimit - dPositiveLimit
	dAngle1 = dAngle1 < dNegativeLimit ? dAngle1 + 360.0 : dAngle1;
	dAngle2 = dAngle2 > dPositiveLimit ? dAngle2 - 360.0 : dAngle2;
	dAngle2 = dAngle2 < dNegativeLimit ? dAngle2 + 360.0 : dAngle2;
	double dDiff = dAngle1 - dAngle2;
	double dDir = dDiff > 0.0 ? 1.0 : -1.0;
	if (fabs(dDiff) > 180.0)
	{
		return dAngle1 + dDir * (360.0 - fabs(dDiff)) / 2.0;
	}
	return (dAngle1 + dAngle2) / 2.0;

}


double WeldAfterMeasure::DirAngleToRz(double dDirAngle)
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

double WeldAfterMeasure::DirAngleToRz(double dBaseRz, double dBaseDirAngle, double dChangeDir, double dDirAngle)
{
	dDirAngle = fmod(dDirAngle, 360.0);
	return (dChangeDir * (dDirAngle - dBaseDirAngle) + dBaseRz);
}

double WeldAfterMeasure::RzToDirAngle(double dRz)
{
	double dDirAngle = (dRz - m_tRobotHomeCoors.dRZ) + m_dWeldNorAngleInHome;
	dDirAngle = fmod(dDirAngle, 360.0);
	dDirAngle = dDirAngle < 0.0 ? dDirAngle + 360.0 : dDirAngle;
	return dDirAngle;
}

double WeldAfterMeasure::RzToDirAngle(double dBaseRz, double dBaseDirAngle, double dChangeDir, double dRz)
{
	dRz = fmod(dRz, 360.0);
	dRz = dRz > 180.0 ? (dRz -= 360.0) : dRz;
	dRz = dRz < -180.0 ? (dRz += 360.0) : dRz;
	return (dChangeDir * (dRz - dBaseRz) + dBaseDirAngle);
}

T_ROBOT_COORS WeldAfterMeasure::GenerateRobotCoord(CvPoint3D64f tSrcPtn3D, double dRx, double dRy, double dRz)
{
	T_ROBOT_COORS tRobotCoors;
	memset(&tRobotCoors, 0, sizeof(tRobotCoors));
	tRobotCoors.dX = tSrcPtn3D.x;
	tRobotCoors.dY = tSrcPtn3D.y;
	tRobotCoors.dZ = tSrcPtn3D.z;
	tRobotCoors.dRX = dRx;
	tRobotCoors.dRY = dRy;
	tRobotCoors.dRZ = dRz;
	return tRobotCoors;
}

T_ROBOT_COORS WeldAfterMeasure::GenerateRobotCoord(T_ALGORITHM_POINT tAlgPtn3D, double dRx, double dRy, double dRz)
{
	T_ROBOT_COORS tRobotCoors;
	memset(&tRobotCoors, 0, sizeof(tRobotCoors));
	tRobotCoors.dX = tAlgPtn3D.dCoorX;
	tRobotCoors.dY = tAlgPtn3D.dCoorY;
	tRobotCoors.dZ = tAlgPtn3D.dCoorZ;
	tRobotCoors.dRX = dRx;
	tRobotCoors.dRY = dRy;
	tRobotCoors.dRZ = dRz;
	return tRobotCoors;
}

void WeldAfterMeasure::GenerateMultipleCoord(T_ROBOT_COORS tStartCoord, T_ROBOT_COORS tEndCoord, vector<T_ROBOT_COORS>& vtCoords, double dInterval)
{
	vtCoords.clear();
	double dDis = TwoPointDis(tStartCoord.dX, tStartCoord.dY, tStartCoord.dZ, tEndCoord.dX, tEndCoord.dY, tEndCoord.dZ);
	double dDisX = tEndCoord.dX - tStartCoord.dX;
	double dDisY = tEndCoord.dY - tStartCoord.dY;
	double dDisZ = tEndCoord.dZ - tStartCoord.dZ;
	int nPtnNum = dDis / dInterval;
	dInterval = dDis / (double)nPtnNum;

	T_ROBOT_COORS tCoord = tStartCoord;
	for (int i = 0; i < nPtnNum; i++)
	{
		tCoord.dX = tStartCoord.dX + dDisX * (double)i / (double)nPtnNum;
		tCoord.dY = tStartCoord.dY + dDisY * (double)i / (double)nPtnNum;
		tCoord.dZ = tStartCoord.dZ + dDisZ * (double)i / (double)nPtnNum;
		vtCoords.push_back(tCoord);
	}
}

void WeldAfterMeasure::GenerateMultipleCoord(XI_POINT tStartCoord, XI_POINT tEndCoord, vector<XI_POINT>& vtCoords, double dInterval)
{
	vtCoords.clear();
	double dDis = TwoPointDis(tStartCoord.x, tStartCoord.y, tStartCoord.z, tEndCoord.x, tEndCoord.y, tEndCoord.z);
	double dDisX = tEndCoord.x - tStartCoord.x;
	double dDisY = tEndCoord.y - tStartCoord.y;
	double dDisZ = tEndCoord.z - tStartCoord.z;
	int nPtnNum = dDis / dInterval;
	dInterval = dDis / (double)nPtnNum;

	XI_POINT tCoord;
	for (int i = 0; i < nPtnNum; i++)
	{
		tCoord.x = tStartCoord.x + dDisX * (double)i / (double)nPtnNum;
		tCoord.y = tStartCoord.y + dDisY * (double)i / (double)nPtnNum;
		tCoord.z = tStartCoord.z + dDisZ * (double)i / (double)nPtnNum;
		vtCoords.push_back(tCoord);
	}
}


T_ROBOT_COORS WeldAfterMeasure::RobotCoordPosOffset(T_ROBOT_COORS tSrcPtn, T_LINE_PARA tLineParam, double dOffsetDis)
{
	tSrcPtn.dX += (dOffsetDis * tLineParam.dDirX);
	tSrcPtn.dY += (dOffsetDis * tLineParam.dDirY);
	tSrcPtn.dZ += (dOffsetDis * tLineParam.dDirZ);
	return tSrcPtn;
}

T_ROBOT_COORS WeldAfterMeasure::RobotCoordPosOffset(T_ROBOT_COORS tSrcPtn, T_ROBOT_COORS tDirSPtn, T_ROBOT_COORS tDirEPtn, double dOffsetDis)
{
	double dDis = TwoPointDis(tDirSPtn.dX, tDirSPtn.dY, tDirSPtn.dZ,tDirEPtn.dX, tDirEPtn.dY, tDirEPtn.dZ);
	double dDisX = tDirEPtn.dX - tDirSPtn.dX;
	double dDisY = tDirEPtn.dY - tDirSPtn.dY;
	double dDisZ = tDirEPtn.dZ - tDirSPtn.dZ;
	tSrcPtn.dX += (dDisX * dOffsetDis / dDis);
	tSrcPtn.dY += (dDisY * dOffsetDis / dDis);
	tSrcPtn.dZ += (dDisZ * dOffsetDis / dDis);
	return tSrcPtn;
}

T_ROBOT_COORS WeldAfterMeasure::RobotCoordPosOffset(T_ROBOT_COORS tSrcPtn, CvPoint3D64f tDirSPtn, CvPoint3D64f tDirEPtn, double dOffsetDis)
{
	double dDis = TwoPointDis(tDirSPtn.x, tDirSPtn.y, tDirSPtn.z, tDirEPtn.x, tDirEPtn.y, tDirEPtn.z);
	double dDisX = tDirEPtn.x - tDirSPtn.x;
	double dDisY = tDirEPtn.y - tDirSPtn.y;
	double dDisZ = tDirEPtn.z - tDirSPtn.z;
	tSrcPtn.dX += (dDisX * dOffsetDis / dDis);
	tSrcPtn.dY += (dDisY * dOffsetDis / dDis);
	tSrcPtn.dZ += (dDisZ * dOffsetDis / dDis);
	return tSrcPtn;
}

XI_POINT WeldAfterMeasure::RobotCoordPosOffset(XI_POINT tSrcPtn, XI_POINT tDirSPtn, XI_POINT tDirEPtn, double dOffsetDis)
{
	double dDis = TwoPointDis(tDirSPtn.x, tDirSPtn.y, tDirSPtn.z, tDirEPtn.x, tDirEPtn.y, tDirEPtn.z);
	double dDisX = tDirEPtn.x - tDirSPtn.x;
	double dDisY = tDirEPtn.y - tDirSPtn.y;
	double dDisZ = tDirEPtn.z - tDirSPtn.z;
	tSrcPtn.x += (dDisX * dOffsetDis / dDis);
	tSrcPtn.y += (dDisY * dOffsetDis / dDis);
	tSrcPtn.z += (dDisZ * dOffsetDis / dDis);
	return tSrcPtn;
}

T_ALGORITHM_POINT WeldAfterMeasure::RobotCoordPosOffset(T_ALGORITHM_POINT tSrcPtn, T_ALGORITHM_POINT tDirSPtn, T_ALGORITHM_POINT tDirEPtn, double dOffsetDis)
{
	double dDis = TwoPointDis(tDirSPtn.dCoorX, tDirSPtn.dCoorY, tDirSPtn.dCoorZ, tDirEPtn.dCoorX, tDirEPtn.dCoorY, tDirEPtn.dCoorZ);
	double dDisX = tDirEPtn.dCoorX - tDirSPtn.dCoorX;
	double dDisY = tDirEPtn.dCoorY - tDirSPtn.dCoorY;
	double dDisZ = tDirEPtn.dCoorZ - tDirSPtn.dCoorZ;
	tSrcPtn.dCoorX += (dDisX * dOffsetDis / dDis);
	tSrcPtn.dCoorY += (dDisY * dOffsetDis / dDis);
	tSrcPtn.dCoorZ += (dDisZ * dOffsetDis / dDis);
	return tSrcPtn;
}

void WeldAfterMeasure::RobotCoordPosOffset(T_ROBOT_COORS& tSrcCoord, double dOffsetDir, double dOffsetDis, double dHeightOffsetDis)
{
	tSrcCoord.dX += (dOffsetDis * CosD(dOffsetDir));
	tSrcCoord.dY += (dOffsetDis * SinD(dOffsetDir));
	tSrcCoord.dZ += dHeightOffsetDis;
}

T_ALGORITHM_POINT WeldAfterMeasure::TransPtn3D(XI_POINT tPtn)
{
	T_ALGORITHM_POINT tNewPtn;
	tNewPtn.dCoorX = tPtn.x;
	tNewPtn.dCoorY = tPtn.y;
	tNewPtn.dCoorZ = tPtn.z;
	return tNewPtn;
}

XI_POINT WeldAfterMeasure::TtansPtn3D(T_ALGORITHM_POINT tPtn)
{
	XI_POINT tNewPtn;
	tNewPtn.x = tPtn.dCoorX;
	tNewPtn.y = tPtn.dCoorY;
	tNewPtn.z = tPtn.dCoorZ;
	return tNewPtn;
}

bool WeldAfterMeasure::CalcContinuePulse(vector<T_ROBOT_COORS>& vtRobotCoors, vector<T_ANGLE_PULSE>& vtRobotPulse)
{
	int nCoordNum = vtRobotCoors.size();
	if (nCoordNum <= 0)
	{
		return false;
	}
	vtRobotPulse.clear();
	vtRobotPulse.resize(nCoordNum);

	// Rz��Χת������180.0����
	for (int i = 0; i < nCoordNum; i++)
	{
		vtRobotCoors[i].dRZ = fmod(vtRobotCoors[i].dRZ, 360.0);
		vtRobotCoors[i].dRZ = vtRobotCoors[i].dRZ > 180.0 ? vtRobotCoors[i].dRZ - 360.0 : vtRobotCoors[i].dRZ;
		vtRobotCoors[i].dRZ = vtRobotCoors[i].dRZ < -180.0 ? vtRobotCoors[i].dRZ + 360.0 : vtRobotCoors[i].dRZ;
	}

	// ʹRz���ڱ仯��С��180��
	vector<double> vdRz;
	vdRz.resize(nCoordNum);
	vdRz[0] = vtRobotCoors[0].dRZ;
	double dMaxRz = vdRz[0];
	double dMinRz = vdRz[0];
	for (int i = 1; i < nCoordNum; i++)
	{
		if (fabs(vtRobotCoors[i].dRZ - vdRz[i - 1]) <= 180.0)
		{
			vdRz[i] = vtRobotCoors[i].dRZ;
		}
		else if (vtRobotCoors[i].dRZ > vdRz[i - 1])
		{
			vdRz[i] = vtRobotCoors[i].dRZ - 360.0;
		}
		else
		{
			vdRz[i] = vtRobotCoors[i].dRZ + 360.0;
		}
		dMaxRz = dMaxRz < vdRz[i] ? vdRz[i] : dMaxRz;
		dMinRz = dMinRz > vdRz[i] ? vdRz[i] : dMinRz;
	}

	// ��С������Rz��Χ
	double dMaxMinAverageRz = (dMaxRz + dMinRz) / 2.0;
	if (fabs(dMaxMinAverageRz) > 180.0)
	{
		double dOffsetDir = dMaxMinAverageRz > 0.0 ? -1.0 : 1.0;
		int nOffsetTimes = fabs(dMaxMinAverageRz) / 360;
		double dOffsetDis = (fabs(dMaxMinAverageRz) - (double)nOffsetTimes * 360.0) > 180.0 ? 360.0 : 0.0;
		for (int i = 0; i < nCoordNum; i++)
		{
			vdRz[i] += (dOffsetDir * (double)nOffsetTimes * 360.0 + dOffsetDir * dOffsetDis);
		}
	}

	// ����ת���ο��ؽ����� �� ����ֱ�������Ӧ�Ĺؽ�����
	double dAxisUnitT = m_pRobotDriver->m_tAxisUnit.dTPulse; // T�����嵱��

	T_ROBOT_COORS tStandardRefCoord;
	m_pRobotDriver->RobotKinematics(m_pRobotDriver->m_tHomePulse, m_pRobotDriver->m_tTools.tGunTool, tStandardRefCoord);
	for (int i = 0; i < nCoordNum; i++)
	{
		T_ANGLE_PULSE tRefRobot = m_pRobotDriver->m_tHomePulse;
		tRefRobot.nTPulse += ((double)m_nRobotInstallDir * (vdRz[i] - tStandardRefCoord.dRZ) / dAxisUnitT); // ����T������ͷ���Ǳ仯�����Ƿ���ͬ
		bool bRst = m_pRobotDriver->RobotInverseKinematics(vtRobotCoors[i], tRefRobot, m_pRobotDriver->m_tTools.tGunTool, vtRobotPulse[i]);
		if (false == bRst)
		{
			vtRobotPulse.clear();
			XUI::MesBox::PopOkCancel("ת����{0}��ֱ������ʧ��!", i);
			return false;
		}
	}
	return true;
}

bool WeldAfterMeasure::CalcContinuePulseForWeld(vector<T_ROBOT_COORS>& vtRobotCoors, vector<T_ANGLE_PULSE>& vtRobotPulse, bool bWeldTrack)
{
	vtRobotPulse.clear();
	vector<vector<T_ANGLE_PULSE>> vvtRobotPulse;	// ÿ��ֱ������ ��Ӧ�Ĺؽ����꼯��
	vector<vector<T_ANGLE_PULSE>> vvtTrackPulse;	// ����������˶��Ĺؽ�����켣
	vector<T_ANGLE_PULSE> vtPulse;					// ��ʱ����
	vector<T_ANGLE_PULSE> vtTotalPulse;						// ÿ�������˶��켣�����ۼ�����
	vector<T_ANGLE_PULSE> vtAveragePulse;					// ÿ�������˶��켣����ƽ������
	T_AXISUNIT tAxleUnit = m_pRobotDriver->m_tAxisUnit;
	long nMinLPulse = -1 == m_nRobotInstallDir ? (long)(-70.0 / tAxleUnit.dLPulse) : (long)(-180.0 / tAxleUnit.dLPulse); // ��ʱ
	long nMaxLPulse = -1 == m_nRobotInstallDir ? (long)(55.0 / tAxleUnit.dLPulse) : (long)(180.0 / tAxleUnit.dLPulse);
	T_ANGLE_THETA tThetaThreshold = true == bWeldTrack ?
		T_ANGLE_THETA(90.0, 45.0, 45.0, 50.0, 90.0, 180.0) :
		T_ANGLE_THETA(135.0, 90.0, 90.0, 90.0, 90.0, 135.0);
	T_ANGLE_PULSE tAngleThreshold((long)tThetaThreshold.dThetaS / tAxleUnit.dSPulse, (long)tThetaThreshold.dThetaL / tAxleUnit.dLPulse,
		(long)tThetaThreshold.dThetaU / tAxleUnit.dUPulse,(long)tThetaThreshold.dThetaR / tAxleUnit.dRPulse,
		(long)tThetaThreshold.dThetaB / tAxleUnit.dBPulse, (long)tThetaThreshold.dThetaT / tAxleUnit.dTPulse, 0, 0, 0);
	vvtRobotPulse.clear();
	vvtTrackPulse.clear();
	vtPulse.clear();
	vtTotalPulse.clear();
	vtAveragePulse.clear();

	FILE* pf = fopen("A_Test.txt", "w");
	fprintf(pf, "%6d%6d%10d%10d%10d%10d%10d%10d\n", -1, -1, tAngleThreshold.nSPulse, tAngleThreshold.nLPulse,
		tAngleThreshold.nUPulse, tAngleThreshold.nRPulse, tAngleThreshold.nBPulse, tAngleThreshold.nTPulse);
	int nPulseMinRstNum = 99; 
	for (int i = 0; i < vtRobotCoors.size(); i++)
	{
		m_pRobotDriver->RobotInverseKinematics(vtRobotCoors[i], m_pRobotDriver->m_tTools.tGunTool, vtPulse);
		if (vtPulse.size() == 0)
		{
			//XiMessageBox("ת��ʧ��");
			return false;
		}
		vvtRobotPulse.push_back(vtPulse);
		nPulseMinRstNum = nPulseMinRstNum > vtPulse.size() ? vtPulse.size() : nPulseMinRstNum;
		for (int nCoordNo = 0; nCoordNo < vtPulse.size(); nCoordNo++)
		{
			fprintf(pf, "%6d%6d%10d%10d%10d%10d%10d%10d\n",i, nCoordNo, vtPulse[nCoordNo].nSPulse, vtPulse[nCoordNo].nLPulse,
				vtPulse[nCoordNo].nUPulse, vtPulse[nCoordNo].nRPulse, vtPulse[nCoordNo].nBPulse, vtPulse[nCoordNo].nTPulse);
		}
	}
	fclose(pf);

	vector<vector<double>> vvtMaxAngleErr;	// ����������˶��Ĺؽ�����켣
	vector<double> vdMaxAngleErr;
	vdMaxAngleErr.clear();
	for (int nTrackNo = 0; nTrackNo < vvtRobotPulse[0].size(); nTrackNo++) // ÿ��켣��һ����
	{
		vector<T_ANGLE_PULSE> vtTrackPulse(vvtRobotPulse.size());
		vtTrackPulse[0] = vvtRobotPulse[0][nTrackNo];
		vvtTrackPulse.push_back(vtTrackPulse);
		vtTotalPulse.push_back(vvtRobotPulse[0][nTrackNo]);
		vdMaxAngleErr.resize(vvtRobotPulse.size());
		vvtMaxAngleErr.push_back(vdMaxAngleErr);
	}

	int nFoundIdx = 0;
	// ʱ�临�Ӷ� С�� 100 * N
	for (int i = 1; i < vvtRobotPulse.size(); i++) // ����ÿ��켣 �ڶ������Ժ�ĵ� N
	{
		vector<T_ANGLE_PULSE>& vtTempPulse = vvtRobotPulse[i];
		//if(nFoundIdx < )
		for (int nTrackNo = 0; nTrackNo < vvtTrackPulse.size(); ) // 10
		{
			const T_ANGLE_PULSE& tPulse = vvtTrackPulse[nTrackNo][nFoundIdx];
			int nMatchNum = 0;
			for (int nTempPulseIdx = 0; nTempPulseIdx < vtTempPulse.size(); nTempPulseIdx++) // 10
			{
				const T_ANGLE_PULSE& tNewPulse = vtTempPulse[nTempPulseIdx];

				if ((labs(tPulse.nSPulse - tNewPulse.nSPulse) < tAngleThreshold.nSPulse) &&
					(labs(tPulse.nLPulse - tNewPulse.nLPulse) < tAngleThreshold.nLPulse) &&
					(labs(tPulse.nUPulse - tNewPulse.nUPulse) < tAngleThreshold.nUPulse) &&
					(labs(tPulse.nRPulse - tNewPulse.nRPulse) < tAngleThreshold.nRPulse) &&
					(labs(tPulse.nBPulse - tNewPulse.nBPulse) < tAngleThreshold.nBPulse) &&
					(labs(tPulse.nTPulse - tNewPulse.nTPulse) < tAngleThreshold.nTPulse) &&
					(tNewPulse.nLPulse > nMinLPulse) && 
					(tNewPulse.nLPulse < nMaxLPulse))
				{
					if (nMatchNum >= 1)//���������ʱѡȡ�����ֵ��С��һ��
					{
						long lDiffe = labs(tPulse.nSPulse - tNewPulse.nSPulse) +
							labs(tPulse.nLPulse - tNewPulse.nLPulse) +
							labs(tPulse.nUPulse - tNewPulse.nUPulse) +
							labs(tPulse.nRPulse - tNewPulse.nRPulse) +
							labs(tPulse.nBPulse - tNewPulse.nBPulse) +
							labs(tPulse.nTPulse - tNewPulse.nTPulse);

						long lDiffe2 = labs(tPulse.nSPulse - vvtTrackPulse[nTrackNo][nFoundIdx + 1].nSPulse) +
							labs(tPulse.nLPulse - vvtTrackPulse[nTrackNo][nFoundIdx + 1].nLPulse) +
							labs(tPulse.nUPulse - vvtTrackPulse[nTrackNo][nFoundIdx + 1].nUPulse) +
							labs(tPulse.nRPulse - vvtTrackPulse[nTrackNo][nFoundIdx + 1].nRPulse) +
							labs(tPulse.nBPulse - vvtTrackPulse[nTrackNo][nFoundIdx + 1].nBPulse) +
							labs(tPulse.nTPulse - vvtTrackPulse[nTrackNo][nFoundIdx + 1].nTPulse);
						if (lDiffe2 < lDiffe)
						{
							continue;
						}
						WriteLog("�켣%d ��%d���� ����һ��������%d �������ѡ�������ֵ��С�Ľ����", nTrackNo, nFoundIdx, nMatchNum);
					}
					vvtTrackPulse[nTrackNo][nFoundIdx + 1] = tNewPulse;
					nMatchNum++;
					vtTotalPulse[nTrackNo].nSPulse += tNewPulse.nSPulse; // ������ҵ������� �˴�������
					vtTotalPulse[nTrackNo].nLPulse += tNewPulse.nLPulse;
					vtTotalPulse[nTrackNo].nUPulse += tNewPulse.nUPulse;
					vtTotalPulse[nTrackNo].nRPulse += tNewPulse.nRPulse;
					vtTotalPulse[nTrackNo].nBPulse += tNewPulse.nBPulse;
					vtTotalPulse[nTrackNo].nTPulse += tNewPulse.nTPulse;
				}
			}
			if (nMatchNum < 1)
			{ 
				WriteLog("�켣%d ��%d���� ����һ��������%d ��������ù켣ɾ����", nTrackNo, nFoundIdx, nMatchNum);
				vvtTrackPulse.erase(vvtTrackPulse.begin() + nTrackNo); 
				vtTotalPulse.erase(vtTotalPulse.begin() + nTrackNo);
				continue; // ɾ���켣�� ����nTrackNo����
			}
			if (nMatchNum > 1) // ����1����ôѡ��
			{
				XUI::MesBox::PopOkCancel("�켣{0} ��{1}���� ����һ��������{2} �����������ʧ�ܣ�", nTrackNo, nFoundIdx, nMatchNum);
				return false;
			}
			nTrackNo++;
		}
		nFoundIdx++;
	}
	if (vvtTrackPulse.size() < 1)
	{
		WriteLog(" �����˶��켣����ʧ��!");
		//XiMessageBox("�����˶��켣����ʧ��!");
		return false;
	}

	vtAveragePulse.resize(vtTotalPulse.size());
	for (int nTrackNo = 0; nTrackNo < vvtTrackPulse.size(); nTrackNo++)
	{
		CString sJobName;
		sJobName.Format("MOVL_PULSE-%d", nTrackNo);
		//GenerateJobLocalVariable(vvtTrackPulse[nTrackNo], MOVL, sJobName);
		long lPtnNum = vvtTrackPulse[nTrackNo].size();
		vtAveragePulse[nTrackNo].nSPulse = vtTotalPulse[nTrackNo].nSPulse / lPtnNum;
		vtAveragePulse[nTrackNo].nLPulse = vtTotalPulse[nTrackNo].nLPulse / lPtnNum;
		vtAveragePulse[nTrackNo].nUPulse = vtTotalPulse[nTrackNo].nUPulse / lPtnNum;
		vtAveragePulse[nTrackNo].nRPulse = vtTotalPulse[nTrackNo].nRPulse / lPtnNum;
		vtAveragePulse[nTrackNo].nBPulse = vtTotalPulse[nTrackNo].nBPulse / lPtnNum;
		vtAveragePulse[nTrackNo].nTPulse = vtTotalPulse[nTrackNo].nTPulse / lPtnNum;
	}

	int nChooseIdx = 0;
	//// ȡ S R T �������С�켣
	//long lMinPulseSum = labs(vtAveragePulse[0].nSPulse) + labs(vtAveragePulse[0].nRPulse) + labs(vtAveragePulse[0].nTPulse);
	//for (int i = 1; i < vtAveragePulse.size(); i++)
	//{
	//	long lCurMinPulseSum = labs(vtAveragePulse[i].nSPulse) + labs(vtAveragePulse[i].nRPulse) + labs(vtAveragePulse[i].nTPulse);
	//	if (lMinPulseSum > lCurMinPulseSum)
	//	{
	//		lMinPulseSum = lCurMinPulseSum;
	//		nChooseIdx = i;
	//	}
	//}
	//// ȡ ��R�� 0 +180 -180 ˳��ѡ��
	nChooseIdx = -1;
	double dCurAngleR = 999.0;
	double dCurAngleT = 999.0;
	for (int i = 0; i < vtAveragePulse.size(); i++) // ����R��ӽ�0�Ĺ켣����
	{
		double dAngleR = vtAveragePulse[i].nRPulse * tAxleUnit.dRPulse;
		double dAngleT = vtAveragePulse[i].nTPulse * tAxleUnit.dTPulse;
		// R�᲻��ת(С��90) �� T����С
		if ((fabs(dAngleR) < 90.0)/* && (fabs(dAngleR) < fabs(dCurAngleR) */ && fabs(dAngleT) < fabs(dCurAngleT))
		{
			nChooseIdx = i;
			dCurAngleR = dAngleR;
			dCurAngleT = dAngleT;
		}
	}
	if (nChooseIdx < 0 && -1 == m_nRobotInstallDir) // ����R�����켣���� (��װר�� ��װR�᲻��ת)
	{
		dCurAngleR = -999.0;
		for (int i = 0; i < vtAveragePulse.size(); i++)
		{
			double dAngleR = vtAveragePulse[i].nRPulse * tAxleUnit.dRPulse;
			if (dAngleR > dCurAngleR)
			{
				nChooseIdx = i;
				dCurAngleR = dAngleR;
			}
		}
	}
	if (nChooseIdx < 0)
	{
		WriteLog("�����˶��켣ѡ��ʧ��!");
		return false;
	}

	vtRobotPulse.reserve(vvtTrackPulse[nChooseIdx].size());
	for (int i = 0; i < vvtTrackPulse[nChooseIdx].size(); i++)
	{
		vtRobotPulse.push_back(vvtTrackPulse[nChooseIdx][i]);
	}
	return true;
}

bool WeldAfterMeasure::CalcContinuePulseForMeasure(vector<T_ROBOT_COORS>& vtRobotCoors, vector<T_ANGLE_PULSE>& vtRobotPulse, bool bWeldTrack)
{
	vtRobotPulse.clear();
	vector<vector<T_ANGLE_PULSE>> vvtRobotPulse;	// ÿ��ֱ������ ��Ӧ�Ĺؽ����꼯��
	vector<vector<T_ANGLE_PULSE>> vvtTrackPulse;	// ����������˶��Ĺؽ�����켣
	vector<T_ANGLE_PULSE> vtPulse;					// ��ʱ����
	vector<T_ANGLE_PULSE> vtTotalPulse;						// ÿ�������˶��켣�����ۼ�����
	vector<T_ANGLE_PULSE> vtAveragePulse;					// ÿ�������˶��켣����ƽ������
	T_AXISUNIT tAxleUnit = m_pRobotDriver->m_tAxisUnit;
	long nMinLPulse = -1 == m_nRobotInstallDir ? (long)(-50.0 / tAxleUnit.dLPulse) : (long)(-180.0 / tAxleUnit.dLPulse); // ��ʱ
	long nMaxLPulse = -1 == m_nRobotInstallDir ? (long)(35.0 / tAxleUnit.dLPulse) : (long)(180.0 / tAxleUnit.dLPulse);
	T_ANGLE_THETA tThetaThreshold = true == bWeldTrack ?
		T_ANGLE_THETA(90.0, 45.0, 45.0, 50.0, 90.0, 100.0) :
		T_ANGLE_THETA(135.0, 90.0, 90.0, 90.0, 90.0, 135.0);
	T_ANGLE_PULSE tAngleThreshold((long)tThetaThreshold.dThetaS / tAxleUnit.dSPulse, (long)tThetaThreshold.dThetaL / tAxleUnit.dLPulse,
		(long)tThetaThreshold.dThetaU / tAxleUnit.dUPulse, (long)tThetaThreshold.dThetaR / tAxleUnit.dRPulse,
		(long)tThetaThreshold.dThetaB / tAxleUnit.dBPulse, (long)tThetaThreshold.dThetaT / tAxleUnit.dTPulse, 0, 0, 0);
	vvtRobotPulse.clear();
	vvtTrackPulse.clear();
	vtPulse.clear();
	vtTotalPulse.clear();
	vtAveragePulse.clear();

	FILE* pf = fopen("A_Test.txt", "w");
	fprintf(pf, "%6d%6d%10d%10d%10d%10d%10d%10d\n", -1, -1, tAngleThreshold.nSPulse, tAngleThreshold.nLPulse,
		tAngleThreshold.nUPulse, tAngleThreshold.nRPulse, tAngleThreshold.nBPulse, tAngleThreshold.nTPulse);
	int nPulseMinRstNum = 99;
	for (int i = 0; i < vtRobotCoors.size(); i++)
	{
		m_pRobotDriver->RobotInverseKinematics(vtRobotCoors[i], m_pRobotDriver->m_tTools.tGunTool, vtPulse);
		vvtRobotPulse.push_back(vtPulse);
		nPulseMinRstNum = nPulseMinRstNum > vtPulse.size() ? vtPulse.size() : nPulseMinRstNum;
		for (int nCoordNo = 0; nCoordNo < vtPulse.size(); nCoordNo++)
		{
			fprintf(pf, "%6d%6d%10d%10d%10d%10d%10d%10d\n", i, nCoordNo, vtPulse[nCoordNo].nSPulse, vtPulse[nCoordNo].nLPulse,
				vtPulse[nCoordNo].nUPulse, vtPulse[nCoordNo].nRPulse, vtPulse[nCoordNo].nBPulse, vtPulse[nCoordNo].nTPulse);
		}
	}
	fclose(pf);

	vector<vector<double>> vvtMaxAngleErr;	// ����������˶��Ĺؽ�����켣
	vector<double> vdMaxAngleErr;
	vdMaxAngleErr.clear();
	for (int nTrackNo = 0; nTrackNo < vvtRobotPulse[0].size(); nTrackNo++) // ÿ��켣��һ����
	{
		vector<T_ANGLE_PULSE> vtTrackPulse(vvtRobotPulse.size());
		vtTrackPulse[0] = vvtRobotPulse[0][nTrackNo];
		vvtTrackPulse.push_back(vtTrackPulse);
		vtTotalPulse.push_back(vvtRobotPulse[0][nTrackNo]);
		vdMaxAngleErr.resize(vvtRobotPulse.size());
		vvtMaxAngleErr.push_back(vdMaxAngleErr);
	}

	int nFoundIdx = 0;
	// ʱ�临�Ӷ� С�� 100 * N
	for (int i = 1; i < vvtRobotPulse.size(); i++) // ����ÿ��켣 �ڶ������Ժ�ĵ� N
	{
		vector<T_ANGLE_PULSE>& vtTempPulse = vvtRobotPulse[i];
		//if(nFoundIdx < )
		for (int nTrackNo = 0; nTrackNo < vvtTrackPulse.size(); ) // 10
		{
			const T_ANGLE_PULSE& tPulse = vvtTrackPulse[nTrackNo][nFoundIdx];
			int nMatchNum = 0;
			for (int nTempPulseIdx = 0; nTempPulseIdx < vtTempPulse.size(); nTempPulseIdx++) // 10
			{
				const T_ANGLE_PULSE& tNewPulse = vtTempPulse[nTempPulseIdx];

				if ((labs(tPulse.nSPulse - tNewPulse.nSPulse) < tAngleThreshold.nSPulse) &&
					(labs(tPulse.nLPulse - tNewPulse.nLPulse) < tAngleThreshold.nLPulse) &&
					(labs(tPulse.nUPulse - tNewPulse.nUPulse) < tAngleThreshold.nUPulse) &&
					(labs(tPulse.nRPulse - tNewPulse.nRPulse) < tAngleThreshold.nRPulse) &&
					(labs(tPulse.nBPulse - tNewPulse.nBPulse) < tAngleThreshold.nBPulse) &&
					(labs(tPulse.nTPulse - tNewPulse.nTPulse) < tAngleThreshold.nTPulse) &&
					(tNewPulse.nLPulse > nMinLPulse) &&
					(tNewPulse.nLPulse < nMaxLPulse))
				{
					if (nMatchNum >= 1)//���������ʱѡȡ�����ֵ��С��һ��
					{
						long lDiffe = labs(tPulse.nSPulse - tNewPulse.nSPulse) +
							labs(tPulse.nLPulse - tNewPulse.nLPulse) +
							labs(tPulse.nUPulse - tNewPulse.nUPulse) +
							labs(tPulse.nRPulse - tNewPulse.nRPulse) +
							labs(tPulse.nBPulse - tNewPulse.nBPulse) +
							labs(tPulse.nTPulse - tNewPulse.nTPulse);

						long lDiffe2 = labs(tPulse.nSPulse - vvtTrackPulse[nTrackNo][nFoundIdx + 1].nSPulse) +
							labs(tPulse.nLPulse - vvtTrackPulse[nTrackNo][nFoundIdx + 1].nLPulse) +
							labs(tPulse.nUPulse - vvtTrackPulse[nTrackNo][nFoundIdx + 1].nUPulse) +
							labs(tPulse.nRPulse - vvtTrackPulse[nTrackNo][nFoundIdx + 1].nRPulse) +
							labs(tPulse.nBPulse - vvtTrackPulse[nTrackNo][nFoundIdx + 1].nBPulse) +
							labs(tPulse.nTPulse - vvtTrackPulse[nTrackNo][nFoundIdx + 1].nTPulse);
						if (lDiffe2 < lDiffe)
						{
							continue;
						}
						WriteLog("�켣%d ��%d���� ����һ��������%d �������ѡ�������ֵ��С�Ľ����", nTrackNo, nFoundIdx, nMatchNum);
					}
					vvtTrackPulse[nTrackNo][nFoundIdx + 1] = tNewPulse;
					nMatchNum++;
					vtTotalPulse[nTrackNo].nSPulse += tNewPulse.nSPulse; // ������ҵ������� �˴�������
					vtTotalPulse[nTrackNo].nLPulse += tNewPulse.nLPulse;
					vtTotalPulse[nTrackNo].nUPulse += tNewPulse.nUPulse;
					vtTotalPulse[nTrackNo].nRPulse += tNewPulse.nRPulse;
					vtTotalPulse[nTrackNo].nBPulse += tNewPulse.nBPulse;
					vtTotalPulse[nTrackNo].nTPulse += tNewPulse.nTPulse;
				}
			}
			if (nMatchNum < 1)
			{
				WriteLog("�켣%d ��%d���� ����һ��������%d ��������ù켣ɾ����", nTrackNo, nFoundIdx, nMatchNum);
				vvtTrackPulse.erase(vvtTrackPulse.begin() + nTrackNo);
				vtTotalPulse.erase(vtTotalPulse.begin() + nTrackNo);
				continue; // ɾ���켣�� ����nTrackNo����
			}
			if (nMatchNum > 1) // ����1����ôѡ��
			{
				XUI::MesBox::PopOkCancel("�켣{0} ��{1}���� ����һ��������{2} �����������ʧ�ܣ�", nTrackNo, nFoundIdx, nMatchNum);
				return false;
			}
			nTrackNo++;
		}
		nFoundIdx++;
	}
	if (vvtTrackPulse.size() < 1)
	{
		WriteLog(" �����˶��켣����ʧ��!");
		//XiMessageBox("�����˶��켣����ʧ��!");
		return false;
	}

	vtAveragePulse.resize(vtTotalPulse.size());
	for (int nTrackNo = 0; nTrackNo < vvtTrackPulse.size(); nTrackNo++)
	{
		CString sJobName;
		sJobName.Format("MOVL_PULSE-%d", nTrackNo);
		//GenerateJobLocalVariable(vvtTrackPulse[nTrackNo], MOVL, sJobName);
		long lPtnNum = vvtTrackPulse[nTrackNo].size();
		vtAveragePulse[nTrackNo].nSPulse = vtTotalPulse[nTrackNo].nSPulse / lPtnNum;
		vtAveragePulse[nTrackNo].nLPulse = vtTotalPulse[nTrackNo].nLPulse / lPtnNum;
		vtAveragePulse[nTrackNo].nUPulse = vtTotalPulse[nTrackNo].nUPulse / lPtnNum;
		vtAveragePulse[nTrackNo].nRPulse = vtTotalPulse[nTrackNo].nRPulse / lPtnNum;
		vtAveragePulse[nTrackNo].nBPulse = vtTotalPulse[nTrackNo].nBPulse / lPtnNum;
		vtAveragePulse[nTrackNo].nTPulse = vtTotalPulse[nTrackNo].nTPulse / lPtnNum;
	}

	int nChooseIdx = 0;
	//// ȡ S R T �������С�켣
	//long lMinPulseSum = labs(vtAveragePulse[0].nSPulse) + labs(vtAveragePulse[0].nRPulse) + labs(vtAveragePulse[0].nTPulse);
	//for (int i = 1; i < vtAveragePulse.size(); i++)
	//{
	//	long lCurMinPulseSum = labs(vtAveragePulse[i].nSPulse) + labs(vtAveragePulse[i].nRPulse) + labs(vtAveragePulse[i].nTPulse);
	//	if (lMinPulseSum > lCurMinPulseSum)
	//	{
	//		lMinPulseSum = lCurMinPulseSum;
	//		nChooseIdx = i;
	//	}
	//}
	//// ȡ ��R�� 0 +180 -180 ˳��ѡ��
	nChooseIdx = -1;
	double dCurAngleR = 999.0;
	double dCurAngleT = 999.0;
	for (int i = 0; i < vtAveragePulse.size(); i++) // ����R��ӽ�0�Ĺ켣����
	{
		double dAngleR = vtAveragePulse[i].nRPulse * tAxleUnit.dRPulse;
		double dAngleT = vtAveragePulse[i].nTPulse * tAxleUnit.dTPulse;
		// R�᲻��ת(С��90) �� T����С
		if ((fabs(dAngleR) < 90.0)/* && (fabs(dAngleR) < fabs(dCurAngleR) */ && fabs(dAngleT) < fabs(dCurAngleT))
		{
			nChooseIdx = i;
			dCurAngleR = dAngleR;
			dCurAngleT = dAngleT;
		}
	}
	if (nChooseIdx < 0 && -1 == m_nRobotInstallDir) // ����R�����켣���� (��װר�� ��װR�᲻��ת)
	{
		dCurAngleR = -999.0;
		for (int i = 0; i < vtAveragePulse.size(); i++)
		{
			double dAngleR = vtAveragePulse[i].nRPulse * tAxleUnit.dRPulse;
			if (dAngleR > dCurAngleR)
			{
				nChooseIdx = i;
				dCurAngleR = dAngleR;
			}
		}
	}
	if (nChooseIdx < 0)
	{
		WriteLog("�����˶��켣ѡ��ʧ��!");
		return false;
	}

	vtRobotPulse.reserve(vvtTrackPulse[nChooseIdx].size());
	for (int i = 0; i < vvtTrackPulse[nChooseIdx].size(); i++)
	{
		vtRobotPulse.push_back(vvtTrackPulse[nChooseIdx][i]);
	}
	return true;
}


bool WeldAfterMeasure::JudgeTheoryTrackContinueMove(int nGroupNo, /*double dCarPos,*/ double dExAxlePos, double yPos) {
	// ɾ���� nGroupNo �麸��켣�ļ� 
	m_dTeachExAxlePos = dExAxlePos;
	m_dTeachExAxlePos_y = yPos;

	int nMaxWeldNo = 8;
	int nMaxLayerNo = 6;
	CString sFileName;
	for (int nDelWeldNo = 0; nDelWeldNo < nMaxWeldNo; nDelWeldNo++) {
		sFileName.Format("%d_%d_RealWeldCoord.txt", nGroupNo, nDelWeldNo);
		DelFiles(m_sDataSavePath, sFileName);
		for (int nLayerNo = 0; nLayerNo < nMaxLayerNo; nLayerNo++) {
			sFileName.Format("%d_%d_%d_AdjustRealWeldCoord.txt", nGroupNo, nDelWeldNo, nLayerNo);
			DelFiles(m_sDataSavePath, sFileName);
		}
	}

	// ���ɺ��ӹ켣�ļ�
	CHECK_BOOL_RETURN(GenerateWeldLineData(nGroupNo, m_dWeldHoleSize, 0.0, true));

	// ����Ƿ�������˶�
	CHECK_BOOL_RETURN(CheckContinueMove(nGroupNo, /*dCarPos,*/ dExAxlePos, yPos));

	return true;
}

bool WeldAfterMeasure::CheckContinueMove(int nGroupNo, /*double dCarPos,*/ double dExAxisPos, double yPos) {
	CleaeJobBuffer(); // �������������������Job �Ļ�����
	double dFileExAxlePos = 0.0;
	E_WELD_SEAM_TYPE eWeldSeamType = E_FLAT_SEAM;
	vector<T_ROBOT_COORS> vtRealWeldCoord;
	vector<T_ANGLE_PULSE> vtRealWeldPulse;
	vector<int> vnPtnType;

	vector<LineOrCircularArcWeldingLine>& vtWeldSeam = m_vvtWeldSeamGroupAdjust[nGroupNo];
	vector<int> vnWeldOrder(0);
	vector<int> vnStandWeldNoIdx(0);
	vector<int> vnFlatWeldNoIdx(0);
	for (int i = 0; i < vtWeldSeam.size(); i++) {
		eWeldSeamType = GetWeldSeamType(vtWeldSeam[i]);
		if (E_FLAT_SEAM == eWeldSeamType) {
			vnFlatWeldNoIdx.push_back(i);
		}
		else if (E_STAND_SEAM == eWeldSeamType) {
			vnStandWeldNoIdx.push_back(i);
		}
	}
	vnWeldOrder.insert(vnWeldOrder.end(), vnStandWeldNoIdx.begin(), vnStandWeldNoIdx.end());
	vnWeldOrder.insert(vnWeldOrder.end(), vnFlatWeldNoIdx.begin(), vnFlatWeldNoIdx.end());

	bool bIsCollide = false;
	for (int i = 0; i < 8; i++) {
		if (i >= vnWeldOrder.size()) break;
		int nWeldNo = vnWeldOrder[i];
		// ���ع켣
		if (false == LoadRealWeldTrack(nGroupNo, i, eWeldSeamType, dFileExAxlePos, vtRealWeldCoord, vnPtnType)) {
			break;
		}

		// ���С������ �� �ⲿ������ ƫ�ƺ�ʵ�ʻ���������
		for (int i = 0; i < vtRealWeldCoord.size(); i++) {
			//vtRealWeldCoord[i].dX += dCarPos;
#ifdef SINGLE_ROBOT
			vtRealWeldCoord[i].dY -= yPos;
#else
			//vtRealWeldCoord[i].dX -= dExAxisPos;
			vtRealWeldCoord[i].dY -= yPos;
#endif // SINGLE_ROBOT

		}


		vector<T_ROBOT_COORS> vtAdjustRealWeldCoord(vtRealWeldCoord);

		T_ROBOT_COORS tDownCoord; // ��ǹ����
		T_ROBOT_COORS tBackCoord; // ��ǹ����		
		double dMaxHeight = CalcBoardMaxHeight(nGroupNo);
		tDownCoord = vtAdjustRealWeldCoord[0];
		if (E_PLAT_GROOVE != eWeldSeamType) {
			tDownCoord.dX += (m_dGunDownBackSafeDis / 1.2 * CosD(m_pRobotDriver->RzToDirAngle(tDownCoord.dRZ)));
			tDownCoord.dY += (m_dGunDownBackSafeDis / 1.2 * SinD(m_pRobotDriver->RzToDirAngle(tDownCoord.dRZ)));
			tDownCoord.dRY = m_dPlatWeldRy; // ���ɵ� Ry 45 ��ȫ
		}
		tDownCoord.dZ = (dMaxHeight + (m_dGunDownBackSafeDis * 2.0) * ((double)m_nRobotInstallDir));
		vtAdjustRealWeldCoord.insert(vtAdjustRealWeldCoord.begin(), tDownCoord);// ��ǹ

		// !!!!!! ��������ǹ���ɵ����ͺ�������ǹ���ɵ�һ�£������ⲿ��λ�ÿ���ͣ��λ�ò�ͬ
		tBackCoord = vtAdjustRealWeldCoord[vtAdjustRealWeldCoord.size() - 1];
		tBackCoord.dX += (m_dGunDownBackSafeDis / 1.2 * CosD(m_pRobotDriver->RzToDirAngle(tBackCoord.dRZ)));
		tBackCoord.dY += (m_dGunDownBackSafeDis / 1.2 * SinD(m_pRobotDriver->RzToDirAngle(tBackCoord.dRZ)));
		tBackCoord.dRY = m_dPlatWeldRy; // ���ɵ� Ry 45 ��ȫ
		tBackCoord.dZ = (dMaxHeight + (m_dGunDownBackSafeDis * 2.0) * ((double)m_nRobotInstallDir));
		vtAdjustRealWeldCoord.push_back(tBackCoord);// ��ǹ

		SaveCoordToFile(vtAdjustRealWeldCoord, "AAA_CheckContinueMove.txt");
		CHECK_BOOL_RETURN(CalcContinuePulseForWeld(vtAdjustRealWeldCoord, vtRealWeldPulse, true));
		if (false == CheckFlangeToolCoord(vtRealWeldPulse))
		{
			m_pRobotDriver->m_cLog->Write("���ۺ��ӹ켣�����˶���飺�������ľ�������ԭ�����");
			return false;
		}
		//GenerateFilePLYPlane(dExAxisPos);
		//SetCheckCollidePlane();
		//bIsCollide |= CheckIsCollide(vtAdjustRealWeldCoord, dExAxisPos, true);

		SaveToJobBuffer(vtRealWeldPulse);
	}
	GenerateJobForContinueWeld(nGroupNo);
	CleaeJobBuffer();
	if (bIsCollide)
	{
		XiMessageBoxOk("���ۺ��ӹ켣���棬�޷�����");
		return false;
	}
	return true;
}

bool WeldAfterMeasure::CheckContinueMove_rAngle(int nGroupNo, /*double dCarPos,*/ double dExAxisPos, double yPos, double ry, double rx) {
	CleaeJobBuffer(); // �������������������Job �Ļ�����
	double dFileExAxlePos = 0.0;
	E_WELD_SEAM_TYPE eWeldSeamType = E_FLAT_SEAM;
	vector<T_ROBOT_COORS> vtRealWeldCoord;
	vector<T_ANGLE_PULSE> vtRealWeldPulse;
	vector<int> vnPtnType;

	vector<LineOrCircularArcWeldingLine>& vtWeldSeam = m_vvtWeldSeamGroupAdjust[nGroupNo];
	vector<int> vnWeldOrder(0);
	vector<int> vnStandWeldNoIdx(0);
	vector<int> vnFlatWeldNoIdx(0);
	for (int i = 0; i < vtWeldSeam.size(); i++) {
		eWeldSeamType = GetWeldSeamType(vtWeldSeam[i]);
		if (E_FLAT_SEAM == eWeldSeamType) {
			vnFlatWeldNoIdx.push_back(i);
		}
		else if (E_STAND_SEAM == eWeldSeamType) {
			vnStandWeldNoIdx.push_back(i);
		}
	}
	vnWeldOrder.insert(vnWeldOrder.end(), vnStandWeldNoIdx.begin(), vnStandWeldNoIdx.end());
	vnWeldOrder.insert(vnWeldOrder.end(), vnFlatWeldNoIdx.begin(), vnFlatWeldNoIdx.end());

	bool bIsCollide = false;
	for (int i = 0; i < 8; i++) {
		if (i >= vnWeldOrder.size()) break;
		int nWeldNo = vnWeldOrder[i];
		// ���ع켣
		if (false == LoadRealWeldTrack(nGroupNo, i, eWeldSeamType, dFileExAxlePos, vtRealWeldCoord, vnPtnType)) {
			break;
		}

		//		// ���С������ �� �ⲿ������ ƫ�ƺ�ʵ�ʻ���������
		//		for (int i = 0; i < vtRealWeldCoord.size(); i++) {
		//			//vtRealWeldCoord[i].dX += dCarPos;
		//#ifdef SINGLE_ROBOT
		//			vtRealWeldCoord[i].dY -= dExAxisPos;
		//#else
		//			vtRealWeldCoord[i].dX -= dExAxisPos;
		//			vtRealWeldCoord[i].dY -= yPos;
		//#endif // SINGLE_ROBOT
		//
		//		}


		vector<T_ROBOT_COORS> vtAdjustRealWeldCoord(vtRealWeldCoord);

		for (int i = 0; i < vtAdjustRealWeldCoord.size(); i++)
		{
			vtAdjustRealWeldCoord[i].dRY = ry;
			vtAdjustRealWeldCoord[i].dRX = rx;
		}

		T_ROBOT_COORS tDownCoord; // ��ǹ����
		T_ROBOT_COORS tBackCoord; // ��ǹ����		
		double dMaxHeight = CalcBoardMaxHeight(nGroupNo);
		tDownCoord = vtAdjustRealWeldCoord[0];
		if (E_PLAT_GROOVE != eWeldSeamType) {
			tDownCoord.dX += (m_dGunDownBackSafeDis / 1.2 * CosD(m_pRobotDriver->RzToDirAngle(tDownCoord.dRZ)));
			tDownCoord.dY += (m_dGunDownBackSafeDis / 1.2 * SinD(m_pRobotDriver->RzToDirAngle(tDownCoord.dRZ)));
			tDownCoord.dRY = m_dPlatWeldRy; // ���ɵ� Ry 45 ��ȫ
		}
		tDownCoord.dZ = (dMaxHeight + (m_dGunDownBackSafeDis * 2.0) * ((double)m_nRobotInstallDir));
		vtAdjustRealWeldCoord.insert(vtAdjustRealWeldCoord.begin(), tDownCoord);// ��ǹ

		// !!!!!! ��������ǹ���ɵ����ͺ�������ǹ���ɵ�һ�£������ⲿ��λ�ÿ���ͣ��λ�ò�ͬ
		tBackCoord = vtAdjustRealWeldCoord[vtAdjustRealWeldCoord.size() - 1];
		tBackCoord.dX += (m_dGunDownBackSafeDis / 1.2 * CosD(m_pRobotDriver->RzToDirAngle(tBackCoord.dRZ)));
		tBackCoord.dY += (m_dGunDownBackSafeDis / 1.2 * SinD(m_pRobotDriver->RzToDirAngle(tBackCoord.dRZ)));
		tBackCoord.dRY = m_dPlatWeldRy; // ���ɵ� Ry 45 ��ȫ
		tBackCoord.dZ = (dMaxHeight + (m_dGunDownBackSafeDis * 2.0) * ((double)m_nRobotInstallDir));
		vtAdjustRealWeldCoord.push_back(tBackCoord);// ��ǹ

		SaveCoordToFile(vtAdjustRealWeldCoord, "AAA_CheckContinueMove.txt");
		CHECK_BOOL_RETURN(CalcContinuePulseForMeasure(vtAdjustRealWeldCoord, vtRealWeldPulse, true));
		if (false == CheckFlangeToolCoord(vtRealWeldPulse))
		{
			m_pRobotDriver->m_cLog->Write("���ۺ��ӹ켣�����˶���飺�������ľ�������ԭ�����");
			return false;
		}
		//GenerateFilePLYPlane(dExAxisPos);
		//SetCheckCollidePlane();
		//bIsCollide |= CheckIsCollide(vtAdjustRealWeldCoord, dExAxisPos, true);

		SaveToJobBuffer(vtRealWeldPulse);
	}
	GenerateJobForContinueWeld(nGroupNo);
	CleaeJobBuffer();
	if (bIsCollide)
	{
		XiMessageBoxOk("���ۺ��ӹ켣���棬�޷�����");
		return false;
	}
	return true;
}

bool WeldAfterMeasure::CompareExAxlePos(CMoveCtrlModule* pMoveCtrl, double dDstPos, double dLimit)
{
	double dCurExPos = pMoveCtrl->GetPositionDis(m_ptUnit->m_vtMotorPara[0].nSoftAxisNo); // �����
	return (fabs(dCurExPos - dDstPos) > dLimit ? false : true);
}

bool WeldAfterMeasure::TransCoordByCamera(std::vector<T_ROBOT_COORS>& vtCoord, T_ROBOT_COORS tSrcTool, T_ROBOT_COORS tDstTool)
{
	// �������ת��ǹ����
	for (int i = 0; i < vtCoord.size(); i++)
	{
		if (false == m_pRobotDriver->MoveToolByWeldGun(vtCoord[i], tSrcTool, vtCoord[i], tDstTool, vtCoord[i]))
		{
			//XiMessageBoxOk("����%d,ת�����������ʧ��", i);
			return false;
		}
	}
	return true;
}

bool WeldAfterMeasure::ProcessTeachImage(int nTeachPtnNo, GROUP_STAND_DIP_NS::CImageProcess* pImgProcess, IplImage* pImg, T_ANGLE_PULSE tCapPulse, int nMeasureType, T_TEACH_RESULT &tTeachResult, E_FLIP_MODE eFlipMode)
{
	int nLinePtnNum = 5;
	T_ROBOT_COORS CameraCoors;
	vector<T_ROBOT_COORS> vtCameraCoors;
	//E_CAM_ID eCamId = E_LEFT_ROBOT_LEFT_CAM_H;
	// ��������ͳ�ʼ��
	tTeachResult.vtLeftPtns2D.clear();
	tTeachResult.vtRightPtns2D.clear();
	tTeachResult.vtLeftPtns3D.clear();
	tTeachResult.vtRightPtns3D.clear();
	tTeachResult.dExAxlePos = m_dTeachExAxlePos;
	tTeachResult.tRobotPulse = tCapPulse;
	m_pRobotDriver->RobotKinematics(tTeachResult.tRobotPulse, m_pRobotDriver->m_tTools.tGunTool, tTeachResult.tRobotCoors);
	m_pRobotDriver->RobotKinematics(tTeachResult.tRobotPulse, m_pRobotDriver->m_tTools.tCameraTool, CameraCoors);
	//vtCameraCoors.push_back(CameraCoors);
	//TransCoordByCamera(vtCameraCoors, m_pRobotDriver->m_tTools.tCameraTool, m_pRobotDriver->m_tTools.tGunTool);
	//CameraCoors = vtCameraCoors[0];
	FlipImage(pImg, eFlipMode); // ����������ò��� ��תͼƬ

	// ��ȡ��ά��
	CvPoint tKeyPoint;
	CvPoint tBesideKeyPoint;
	vector<CvPoint> vLeftPtn;
	vector<CvPoint> vRightPtn;
	WidgetLaserInfo* LaserInfo = new WidgetLaserInfo[10];
	char* LogName = "WidgetLaserLock";
	long long tTime = XI_clock();
	if (nMeasureType & (E_DOUBLE_LONG_LINE))
	{
		int PtnNum = WidgetLaserLock(pImg, LaserInfo, true, false, LogName);
		//bool state = WidgetLaserTrack(pImg, LaserInfo,"WidgetLaserTrack-corner");
		if (PtnNum <= 0)
		{
			XUI::MesBox::PopInfo("ʾ��ʧ��FF�����鼤��ͼƬ{0}�Ƿ�����", nTeachPtnNo);
		}
		else if (PtnNum == 1)
		{
			tKeyPoint = LaserInfo[0].crossPoint;
			for (int i = 0; i < LaserInfo[0].samPntNum; i++)
			{
				// ����ͼ��༤���� ����
				vLeftPtn.push_back(LaserInfo[0].bottomPlateLineSamPnt[i]);//verticalPlateLineSamPnt
				// ����ͼ�Ҳ༤���� �װ�
				vRightPtn.push_back(LaserInfo[0].verticalPlateLineSamPnt[i]);
			}
		}
		else if (PtnNum > 1)
		{
			CvPoint tmpPoint;
			T_ROBOT_COORS tmpKeyPoint;
			double wDis;
			double zDis;
			vector<double> vtWorldDis;
			vector<double> vtHeightDis;
			vector<double> vtZ;
			WriteLog("������ǹλ��:%11.3lf %11.3lf %11.3lf", tTeachResult.tRobotCoors.dX, tTeachResult.tRobotCoors.dY, tTeachResult.tRobotCoors.dZ);
			WriteLog("�������λ��:%11.3lf %11.3lf %11.3lf", CameraCoors.dX, CameraCoors.dY, CameraCoors.dZ);
			for (int keyNum = 0; keyNum < PtnNum; keyNum++)
			{
				tmpPoint = LaserInfo[keyNum].crossPoint;
				ResumeImagePoint2D(pImg->width, pImg->height, tmpPoint, eFlipMode);
				tmpKeyPoint = m_ptUnit->TranImageToBase(m_ptUnit->m_nMeasureCameraNo, tmpPoint, tTeachResult.tRobotCoors, tTeachResult.tRobotPulse);
				wDis = TwoPointDis(tmpKeyPoint.dX, tmpKeyPoint.dY, tmpKeyPoint.dZ,
					CameraCoors.dX, CameraCoors.dY, CameraCoors.dZ);
				zDis = fabs(tmpKeyPoint.dZ - CameraCoors.dZ);
				vtHeightDis.push_back(zDis);
				vtWorldDis.push_back(wDis);
				vtZ.push_back(tmpKeyPoint.dZ);
				WriteLog("����������:%11.3lf %11.3lf %11.3lf", tmpKeyPoint.dX, tmpKeyPoint.dY, tmpKeyPoint.dZ);
			}

			int realKey = 0;
			auto minHeightPosition = min_element(vtHeightDis.begin(), vtHeightDis.end());
			auto maxHeightPosition = max_element(vtHeightDis.begin(), vtHeightDis.end());
			auto minWorldPosition = min_element(vtWorldDis.begin(), vtWorldDis.end());


			/*if (fabs(*maxHeightPosition - *minHeightPosition) > 5)
			{
				auto minPtn = max_element(vtZ.begin(), vtZ.end());
				realKey = minPtn - vtZ.begin();
			}
			else
			{*/
				realKey = minWorldPosition - vtWorldDis.begin();
			//}

			tKeyPoint = LaserInfo[realKey].crossPoint;
			for (int i = 0; i < LaserInfo[realKey].samPntNum; i++)
			{
				// ����ͼ��༤���� ����
				vLeftPtn.push_back(LaserInfo[realKey].bottomPlateLineSamPnt[i]);//verticalPlateLineSamPnt
				// ����ͼ�Ҳ༤���� �װ�
				vRightPtn.push_back(LaserInfo[realKey].verticalPlateLineSamPnt[i]);
			}
		}

		/*pImgProcess->GetBesideKeyPoints(pImg, tKeyPoint, tBesideKeyPoint, vLeftPtn, vRightPtn, false,
			500, 500,
			300, 300,
			20, 20);*/
		WriteLog("Process%d E_DOUBLE_LONG_LINE %d ʾ�̴���ʱ�� %d eFlipMode:%d", nTeachPtnNo, nMeasureType, XI_clock()- tTime, eFlipMode);
	}
	else if (nMeasureType & (E_LS_RL_FLIP))
	{
		int PtnNum = WidgetLaserLock(pImg, LaserInfo, true, true, "WidgetLaserLock_HeightMeasure");
		if (PtnNum <= 0)
		{
			XUI::MesBox::PopInfo("ʾ��ʧ��TT�����鼤��ͼƬ{0}�Ƿ�����", nTeachPtnNo);
		}
		else if (PtnNum == 1)
		{
			tKeyPoint = LaserInfo[0].crossPoint;
			for (int i = 0; i < LaserInfo[0].samPntNum; i++)
			{
				// ����ͼ��༤���� ����
				vLeftPtn.push_back(LaserInfo[0].bottomPlateLineSamPnt[i]);//verticalPlateLineSamPnt
				// ����ͼ�Ҳ༤���� �װ�
				vRightPtn.push_back(LaserInfo[0].verticalPlateLineSamPnt[i]);
			}
		}
		else if (PtnNum > 1)
		{
			CvPoint tmpPoint;
			T_ROBOT_COORS tmpKeyPoint;
			double dis;
			vector<double> vtDis;
			for (int keyNum = 0; keyNum < PtnNum; keyNum++)
			{
				tmpPoint = LaserInfo[keyNum].crossPoint;
				ResumeImagePoint2D(pImg->width, pImg->height, tmpPoint, eFlipMode);
				tmpKeyPoint = m_ptUnit->TranImageToBase(m_ptUnit->m_nMeasureCameraNo, tmpPoint, tTeachResult.tRobotCoors, tTeachResult.tRobotPulse);
				/*dis = TwoPointDis(tmpKeyPoint.dX, tmpKeyPoint.dY, tmpKeyPoint.dZ,
					tTeachResult.tRobotCoors.dX, tTeachResult.tRobotCoors.dY, tTeachResult.tRobotCoors.dZ);*/
				dis = fabs(tmpKeyPoint.dZ - CameraCoors.dZ);
			
				vtDis.push_back(dis);
			}
			auto minPosition = min_element(vtDis.begin(), vtDis.end());
			int realKey = minPosition - vtDis.begin();
			tKeyPoint = LaserInfo[realKey].crossPoint;
			for (int i = 0; i < LaserInfo[realKey].samPntNum; i++)
			{
				// ����ͼ��༤���� ����
				vLeftPtn.push_back(LaserInfo[realKey].bottomPlateLineSamPnt[i]);//verticalPlateLineSamPnt
				// ����ͼ�Ҳ༤���� �װ�
				vRightPtn.push_back(LaserInfo[realKey].verticalPlateLineSamPnt[i]);
			}
		}
		// ��ת
		/*cvFlip(pImg, pImg, 0);
		pImgProcess->GetBesideKeyPoints(pImg, tKeyPoint, tBesideKeyPoint, vLeftPtn, vRightPtn, false,
			500, 500,
			300, 300,
			20, 20, true, 0);*/
		WriteLog("Process%d E_LS_RL_FLIP %d ʾ�̴���ʱ�� %d", nTeachPtnNo, nMeasureType, XI_clock() - tTime);
		// ��ά������ת
		/*int nCenterY = pImg->height / 2;
		tKeyPoint.y = nCenterY - (tKeyPoint.y - nCenterY);
		tBesideKeyPoint.y = nCenterY - (tBesideKeyPoint.y - nCenterY);
		for (int i = 0; i < vLeftPtn.size(); i++)
		{
			vLeftPtn[i].y = nCenterY - (vLeftPtn[i].y - nCenterY);
		}
		for (int i = 0; i < vRightPtn.size(); i++)
		{
			vRightPtn[i].y = nCenterY - (vRightPtn[i].y - nCenterY);
		}*/
	}
	else if (nMeasureType & (E_LL_RS_FLIP))
	{
		int PtnNum = WidgetLaserLock(pImg, LaserInfo, false, true, LogName);
		if (PtnNum <= 0)
		{
			XUI::MesBox::PopInfo("ʾ��ʧ��FT�����鼤��ͼƬ{0}�Ƿ�����", nTeachPtnNo);
		}
		else if (PtnNum == 1)
		{
			tKeyPoint = LaserInfo[0].crossPoint;
			for (int i = 0; i < LaserInfo[0].samPntNum; i++)
			{
				// ����ͼ��༤���� ����
				vLeftPtn.push_back(LaserInfo[0].bottomPlateLineSamPnt[i]);//verticalPlateLineSamPnt
				// ����ͼ�Ҳ༤���� �װ�
				vRightPtn.push_back(LaserInfo[0].verticalPlateLineSamPnt[i]);
			}
		}
		else if (PtnNum > 1)
		{
			CvPoint tmpPoint;
			T_ROBOT_COORS tmpKeyPoint;
			double dis;
			vector<double> vtDis;
			for (int keyNum = 0; keyNum < PtnNum; keyNum++)
			{
				tmpPoint = LaserInfo[keyNum].crossPoint;
				ResumeImagePoint2D(pImg->width, pImg->height, tmpPoint, eFlipMode);
				tmpKeyPoint = m_ptUnit->TranImageToBase(m_ptUnit->m_nMeasureCameraNo, tmpPoint, tTeachResult.tRobotCoors, tTeachResult.tRobotPulse);
				/*dis = TwoPointDis(tmpKeyPoint.dX, tmpKeyPoint.dY, tmpKeyPoint.dZ,
					tTeachResult.tRobotCoors.dX, tTeachResult.tRobotCoors.dY, tTeachResult.tRobotCoors.dZ);*/
				dis = fabs(tmpKeyPoint.dZ - CameraCoors.dZ);
				/*dis = TwoPointDis(tmpKeyPoint.dX, tmpKeyPoint.dY, tmpKeyPoint.dZ,
					CameraCoors.dX, CameraCoors.dY, CameraCoors.dZ);*/
				vtDis.push_back(dis);
			}
			auto minPosition = min_element(vtDis.begin(), vtDis.end());
			int realKey = minPosition - vtDis.begin();
			tKeyPoint = LaserInfo[realKey].crossPoint;
			for (int i = 0; i < LaserInfo[realKey].samPntNum; i++)
			{
				// ����ͼ��༤���� ����
				vLeftPtn.push_back(LaserInfo[realKey].bottomPlateLineSamPnt[i]);//verticalPlateLineSamPnt
				// ����ͼ�Ҳ༤���� �װ�
				vRightPtn.push_back(LaserInfo[realKey].verticalPlateLineSamPnt[i]);
			}
		}
		// ��ת
		/*cvFlip(pImg, pImg, 0);
		pImgProcess->GetBesideKeyPoints(pImg, tKeyPoint, tBesideKeyPoint, vLeftPtn, vRightPtn, false,
			500, 500,
			300, 300,
			20, 20, true, 1);*/
		WriteLog("Process%d E_LL_RS_FLIP %d ʾ�̴���ʱ�� %d", nTeachPtnNo, nMeasureType, XI_clock() - tTime);
		// ��ά������ת
		/*int nCenterY = pImg->height / 2;
		tKeyPoint.y = nCenterY - (tKeyPoint.y - nCenterY);
		tBesideKeyPoint.y = nCenterY - (tBesideKeyPoint.y - nCenterY);
		for (int i = 0; i < vLeftPtn.size(); i++)
		{
			vLeftPtn[i].y = nCenterY - (vLeftPtn[i].y - nCenterY);
		}
		for (int i = 0; i < vRightPtn.size(); i++)
		{
			vRightPtn[i].y = nCenterY - (vRightPtn[i].y - nCenterY);
		}*/
	}
	else if (nMeasureType & (E_L_LINE_POINT))
	{
		pImgProcess->GetBesideKeyPointsExt(pImg, tKeyPoint, tBesideKeyPoint, vLeftPtn, vRightPtn, false,
			500, 500,
			300, 300,
			20, 20);
		WriteLog("Process%d E_L_LINE_POINT %d ʾ�̴���ʱ�� %d", nTeachPtnNo, nMeasureType, XI_clock() - tTime);
	}
	else if (nMeasureType & (E_R_LINE_POINT))
	{
		pImgProcess->GetBesideKeyPointsExt(pImg, tKeyPoint, tBesideKeyPoint, vLeftPtn, vRightPtn, false,
			500, 500,
			300, 300,
			20, 20);
		WriteLog("Process%d E_R_LINE_POINT %d ʾ�̴���ʱ�� %d", nTeachPtnNo, nMeasureType, XI_clock() - tTime);
	}

	if (0 >= vLeftPtn.size() && 0 >= vRightPtn.size()) // �� �� �� ����E_L_LINE_POINT��E_R_LINE_POINT
	{
		SYSTEMTIME tTime;
		GetLocalTime(&tTime);
		CString sSavePath;
		sSavePath.Format("%sPicture\\Err\\Err_Point_%04d%02d%02d%02d%02d%02d_Image%d.jpg",
			m_sDataSavePath.GetBuffer(), tTime.wYear, tTime.wMonth, tTime.wDay,tTime.wHour, tTime.wMinute, tTime.wSecond, nTeachPtnNo);
		SaveImage(pImg, sSavePath);
		cvCvtColor(pImg, *m_pColorImg, CV_GRAY2RGB);
		WriteLog("ʾ�̲�����%d %d ", nTeachPtnNo, nMeasureType);
		return false;
	}

	//if (false == CheckImgProPtn2D(nMeasureType, tKeyPoint, vLeftPtn, vRightPtn))
	//{
	//	SYSTEMTIME tTime;
	//	GetLocalTime(&tTime);
	//	CString sSavePath;
	//	sSavePath.Format("%sPicture\\Err\\Err_Check_%04d%02d%02d%02d%02d%02d_Image%d.jpg",
	//		m_sDataSavePath.GetBuffer(), tTime.wYear, tTime.wMonth, tTime.wDay, tTime.wHour, tTime.wMinute, tTime.wSecond, nTeachPtnNo);
	//	SaveImage(pImg, sSavePath);
	//	cvCvtColor(pImg, *m_pColorImg, CV_GRAY2RGB);
	//	WriteLog("����ά�㴦��ʧ�ܣ�");
	//	return false;
	//}
	// ���� ��ά��ת��ά����(����������)
	ResumeImagePoint2D(pImg->width, pImg->height, tKeyPoint, eFlipMode);

	tTeachResult.tKeyPtn2D = tKeyPoint;
	T_ROBOT_COORS tRetCoord = m_ptUnit->TranImageToBase(m_ptUnit->m_nMeasureCameraNo, tTeachResult.tKeyPtn2D, tTeachResult.tRobotCoors, tTeachResult.tRobotPulse);
	tTeachResult.tKeyPtn3D = P2P(tRetCoord);

	// ���� ��ά��ת��ά����(����������)
	int nStep = vLeftPtn.size() / nLinePtnNum;
	for (int i = 0; i < vLeftPtn.size(); i += nStep)
	{
		ResumeImagePoint2D(pImg->width, pImg->height, vLeftPtn[i], eFlipMode);
		tTeachResult.vtLeftPtns2D.push_back(vLeftPtn[i]);
		tRetCoord = m_ptUnit->TranImageToBase(m_ptUnit->m_nMeasureCameraNo, vLeftPtn[i],tTeachResult.tRobotCoors, tTeachResult.tRobotPulse);
		tTeachResult.vtLeftPtns3D.push_back(P2P(tRetCoord));
	}

	// ���� �����ά��ת��ά����(����������)
	nStep = vRightPtn.size() / nLinePtnNum;
	for (int i = 0; i < vRightPtn.size(); i += nStep)
	{
		ResumeImagePoint2D(pImg->width, pImg->height, vRightPtn[i], eFlipMode);
		tTeachResult.vtRightPtns2D.push_back(vRightPtn[i]);
		tRetCoord = m_ptUnit->TranImageToBase(m_ptUnit->m_nMeasureCameraNo, vRightPtn[i], tTeachResult.tRobotCoors, tTeachResult.tRobotPulse);
		tTeachResult.vtRightPtns3D.push_back(P2P(tRetCoord));
	}
	return true;
}

bool WeldAfterMeasure::CheckImgProPtn2D(int nMeasureType, CvPoint tKeyPoint, vector<CvPoint> vLeftPtn, vector<CvPoint> vRightPtn)
{
	bool bRst = false;
	CvPoint tLeftPtn = E_R_LINE_POINT & nMeasureType ? cvPoint(0, 99999) : vLeftPtn[vLeftPtn.size() - 1];
	CvPoint tRightPtn = E_L_LINE_POINT & nMeasureType ? cvPoint(0, 99999) : vRightPtn[vRightPtn.size() - 1];

	if (((E_LS_RL_FLIP & nMeasureType) || (E_LS_RL_FLIP & nMeasureType)) &&
		(tKeyPoint.y + 10 > tLeftPtn.y) && 
		(tKeyPoint.y + 10 > tRightPtn.y))
	{
		bRst = true;
	}
	else if ((tKeyPoint.y - 10 < tLeftPtn.y) && (tKeyPoint.y - 10 < tRightPtn.y))
	{
		bRst = true;
	}
	return bRst;
}

void WeldAfterMeasure::XiPointAddToAlgPtn(const vector<XI_POINT>&vtXiPoint, vector<T_ALGORITHM_POINT>&vtAlgPtns)
{
	T_ALGORITHM_POINT tAlgPtn;
	for (int i = 0; i < vtXiPoint.size(); i++)
	{
		tAlgPtn.dCoorX = vtXiPoint[i].x;
		tAlgPtn.dCoorY = vtXiPoint[i].y;
		tAlgPtn.dCoorZ = vtXiPoint[i].z;
		vtAlgPtns.push_back(tAlgPtn);
	}
}

T_ALGORITHM_LINE_PARA WeldAfterMeasure::PlanePlaneInterLine(T_ALGORITHM_PLANE_PARAM tPlane1, T_ALGORITHM_PLANE_PARAM tPlane2)
{
	T_ALGORITHM_LINE_PARA tInterLine;
	tInterLine.tLineDir.dDirX = tPlane1.b * tPlane2.c - tPlane2.b * tPlane1.c;
	tInterLine.tLineDir.dDirY = tPlane2.a * tPlane1.c - tPlane1.a * tPlane2.c;
	tInterLine.tLineDir.dDirZ = tPlane1.a * tPlane2.b - tPlane2.a * tPlane1.b;

	tInterLine.tPoint.dCoorX = (tPlane1.b * tPlane2.d - tPlane2.b * tPlane1.d) / (tPlane1.a * tPlane2.b - tPlane2.a * tPlane1.b);
	tInterLine.tPoint.dCoorY = (tPlane1.a * tPlane2.d - tPlane2.a * tPlane1.d) / (tPlane2.a * tPlane1.b - tPlane1.a * tPlane2.b);
	tInterLine.tPoint.dCoorZ = 0.0;
	return tInterLine;
}

bool WeldAfterMeasure::AdjsutWeldSeam(LineOrCircularArcWeldingLine& tWeldSeam,
	T_ALGORITHM_POINT tAlgStartPtn, T_ALGORITHM_POINT tAlgEndPtn, double dNorAngle, double dZSide)
{
	// ������ʱ�뺸��ԭ���жϺ��ӷ���ͺ��ӷ����Ƿ�ƥ��
	XiAlgorithm alg;
	double dWeldDirAngle = alg.CalcArcAngle(tAlgEndPtn.dCoorX - tAlgStartPtn.dCoorX, tAlgEndPtn.dCoorY - tAlgStartPtn.dCoorY);

	// ��ʶ�����Ա�
	double dRecoTeachDisErrThreshold = 150.0;
#ifdef SINGLE_ROBOT
	double dPtnDisS = TwoPointDis(
		tWeldSeam.StartPoint.x, tWeldSeam.StartPoint.y, tWeldSeam.StartPoint.z,
		tAlgStartPtn.dCoorX + m_ptUnit->m_dExAxisXPos, tAlgStartPtn.dCoorY + m_dTeachExAxlePos, tAlgStartPtn.dCoorZ + m_ptUnit->m_dExAxisZPos);
	double dPtnDisE = TwoPointDis(
		tWeldSeam.EndPoint.x, tWeldSeam.EndPoint.y, tWeldSeam.EndPoint.z,
		tAlgEndPtn.dCoorX + m_ptUnit->m_dExAxisXPos, tAlgEndPtn.dCoorY + m_dTeachExAxlePos, tAlgEndPtn.dCoorZ + m_ptUnit->m_dExAxisZPos);
#else

	double dPtnDisS = TwoPointDis(
		tWeldSeam.StartPoint.x, tWeldSeam.StartPoint.y, tWeldSeam.StartPoint.z,
		tAlgStartPtn.dCoorX + m_dTeachExAxlePos, tAlgStartPtn.dCoorY+ m_dTeachExAxlePos_y,
		tAlgStartPtn.dCoorZ+ m_ptUnit->m_dExAxisZPos);
	double dPtnDisE = TwoPointDis(
		tWeldSeam.EndPoint.x, tWeldSeam.EndPoint.y, tWeldSeam.EndPoint.z,
		tAlgEndPtn.dCoorX + m_dTeachExAxlePos, tAlgEndPtn.dCoorY + m_dTeachExAxlePos_y,
		tAlgEndPtn.dCoorZ + m_ptUnit->m_dExAxisZPos);
#endif // SINGLE_ROBOT

	if (dPtnDisS > dRecoTeachDisErrThreshold || dPtnDisE > dRecoTeachDisErrThreshold)
	{
		XUI::MesBox::PopInfo("��ȷ���������ʶ���������� S:{0} E:{1} ��ֵ:{2}",
			dPtnDisS, dPtnDisE, dRecoTeachDisErrThreshold);
		return false;
	}

	// ƽ������Zֵ ��Ϊ0
	if (!IsStandWeldSeam(tWeldSeam.StartNormalVector) && true != JudgeDirAngle(dWeldDirAngle - (90.0 * (double)m_nRobotInstallDir), dNorAngle)) // ƽ�����
	{
		XiMessageBox("ƽ�����ӷ���ͷ���ǲ�������");
		return false;
	}
	tWeldSeam.ZSide = dZSide;
	tWeldSeam.StartPoint.x = tAlgStartPtn.dCoorX;
	tWeldSeam.StartPoint.y = tAlgStartPtn.dCoorY;
	tWeldSeam.StartPoint.z = tAlgStartPtn.dCoorZ;
	tWeldSeam.EndPoint.x = tAlgEndPtn.dCoorX;
	tWeldSeam.EndPoint.y = tAlgEndPtn.dCoorY;
	tWeldSeam.EndPoint.z = tAlgEndPtn.dCoorZ;

	double dVectorDis = sqrt(SQUARE(tWeldSeam.StartNormalVector.x) + SQUARE(tWeldSeam.StartNormalVector.y) + SQUARE(tWeldSeam.StartNormalVector.z));
	tWeldSeam.StartNormalVector.x = CosD(dNorAngle);
	tWeldSeam.StartNormalVector.y = SinD(dNorAngle);
	tWeldSeam.StartNormalVector.z = 1.0 * tWeldSeam.StartNormalVector.z / dVectorDis;

	dVectorDis = sqrt(SQUARE(tWeldSeam.EndNormalVector.x) + SQUARE(tWeldSeam.EndNormalVector.y) + SQUARE(tWeldSeam.EndNormalVector.z));
	tWeldSeam.EndNormalVector.x = CosD(dNorAngle);
	tWeldSeam.EndNormalVector.y = SinD(dNorAngle);
	tWeldSeam.EndNormalVector.z = 1.0 * tWeldSeam.EndNormalVector.z / dVectorDis;
	return true;
}

bool WeldAfterMeasure::AdjustScanWeldSeam(int nGroupNo, int nWeldNo, std::vector<T_TEACH_DATA>& tTeachData)
{
	std::vector<T_TEACH_RESULT> vtTeachResult(0);
	LineOrCircularArcWeldingLine& tWeldSeam = m_vvtWeldSeamGroupAdjust[nGroupNo][nWeldNo];

	if ((0 >= tTeachData.size()))
	{
		//���޸�
		XUI::MesBox::PopInfo("������{0}����{1}��������{2}�쳣��", nGroupNo, nWeldNo, tTeachData.size());
		//XiMessageBox("������%d ����%d ��������%d �쳣��", nGroupNo, nWeldNo, tTeachData.size());
		return false;
	}
	T_ALGORITHM_POINT tAlgPtn;
	vector<T_ALGORITHM_POINT> vtAlgPtns(0);
	// ��ȡtTeachDataS��Ӧ�����в����������
	for (int i = 0; i < tTeachData.size(); i++)
	{
		vtTeachResult.push_back(m_vtTeachResult[tTeachData[i].nMeasurePtnNo]);
		tAlgPtn.dCoorX = m_vtTeachResult[tTeachData[i].nMeasurePtnNo].tKeyPtn3D.x;
		tAlgPtn.dCoorY = m_vtTeachResult[tTeachData[i].nMeasurePtnNo].tKeyPtn3D.y;
		tAlgPtn.dCoorZ = m_vtTeachResult[tTeachData[i].nMeasurePtnNo].tKeyPtn3D.z;
		vtAlgPtns.push_back(tAlgPtn);
	}

	// ��ʶ�����Ա�
	T_ALGORITHM_POINT tAlgStartPtn = vtAlgPtns[0];
	T_ALGORITHM_POINT tAlgEndPtn = vtAlgPtns[1];
	double dRecoTeachDisErrThreshold = 150.0;

#ifdef SINGLE_ROBOT
	double dPtnDisS = TwoPointDis(
		tWeldSeam.StartPoint.x, tWeldSeam.StartPoint.y, tWeldSeam.StartPoint.z,
		tAlgStartPtn.dCoorX + m_ptUnit->m_dExAxisXPos, tAlgStartPtn.dCoorY + m_dTeachExAxlePos, tAlgStartPtn.dCoorZ + m_ptUnit->m_dExAxisZPos);
	double dPtnDisE = TwoPointDis(
		tWeldSeam.EndPoint.x, tWeldSeam.EndPoint.y, tWeldSeam.EndPoint.z,
		tAlgEndPtn.dCoorX + m_ptUnit->m_dExAxisXPos, tAlgEndPtn.dCoorY + m_dTeachExAxlePos, tAlgEndPtn.dCoorZ + m_ptUnit->m_dExAxisZPos);
#else
	double dPtnDisS = TwoPointDis(
		tWeldSeam.StartPoint.x, tWeldSeam.StartPoint.y, tWeldSeam.StartPoint.z,
		tAlgStartPtn.dCoorX + m_dTeachExAxlePos, tAlgStartPtn.dCoorY + m_dTeachExAxlePos_y, tAlgStartPtn.dCoorZ + m_ptUnit->m_dExAxisZPos);
	double dPtnDisE = TwoPointDis(
		tWeldSeam.EndPoint.x, tWeldSeam.EndPoint.y, tWeldSeam.EndPoint.z,
		tAlgEndPtn.dCoorX + m_dTeachExAxlePos, tAlgEndPtn.dCoorY + m_dTeachExAxlePos_y, tAlgEndPtn.dCoorZ + m_ptUnit->m_dExAxisZPos);
#endif // SINGLE_ROBOT


	if (dPtnDisS > dRecoTeachDisErrThreshold || dPtnDisE > dRecoTeachDisErrThreshold)
	{
		XUI::MesBox::PopInfo("ɨ�辫ȷ���������ʶ���������� S:{0} E:{1} ��ֵ:{2}",
			dPtnDisS, dPtnDisE, dRecoTeachDisErrThreshold);
		return false;
	}

	tWeldSeam.StartPoint.x = tAlgStartPtn.dCoorX;
	tWeldSeam.StartPoint.y = tAlgStartPtn.dCoorY;
	tWeldSeam.StartPoint.z = tAlgStartPtn.dCoorZ;
	tWeldSeam.EndPoint.x = tAlgEndPtn.dCoorX;
	tWeldSeam.EndPoint.y = tAlgEndPtn.dCoorY;
	tWeldSeam.EndPoint.z = tAlgEndPtn.dCoorZ;

	return true;
}

bool WeldAfterMeasure::AdjustWeldSeamArc(int nGroupNo, int nWeldNo, std::vector<T_TEACH_DATA>& tTeachData)
{
	XiAlgorithm alg;
	std::vector<T_TEACH_RESULT> vtTeachResult(0);
	std::vector<T_TEACH_DATA> tTeachDataS(0);
	std::vector<T_TEACH_DATA> tTeachDataE(0);
	LineOrCircularArcWeldingLine &tRefSeamLine = m_vvtWeldSeamGroupAdjust[nGroupNo][nWeldNo];

	if (!WeldingLineIsArc(tRefSeamLine))
	{
		XiMessageBoxOk("��Բ������켣����ʧ�ܣ�");
		return false;
	}

	for (int nDataNo = 0; nDataNo < tTeachData.size(); nDataNo++)
	{
		if (tTeachData[nDataNo].eAttribute == E_BELONG_START)
			tTeachDataS.push_back(tTeachData[nDataNo]);
		else if (tTeachData[nDataNo].eAttribute == E_BELONG_END)
			tTeachDataE.push_back(tTeachData[nDataNo]);
	}

	if ((0 >= tTeachDataS.size())/* || (0 >= tTeachDataE.size())*/)
	{
		//���޸�
		XUI::MesBox::PopInfo("������{0}����{1}��������{2} {3}�쳣��", nGroupNo, nWeldNo, tTeachDataS.size(), tTeachDataE.size());
		//XiMessageBox("������%d ����%d ��������%d %d�쳣��", nGroupNo, nWeldNo, tTeachDataS.size(), tTeachDataE.size());
		return false;
	}


	T_ALGORITHM_POINT tAlgPtn;
	vector<T_ALGORITHM_POINT> vtAlgPtns(0);
	// ��ȡtTeachDataS��Ӧ�����в����������
	//FILE* pf2 = fopen("A_Test.txt", "w");
	for (int i = 0; i < tTeachDataS.size(); i++)
	{
		vtTeachResult.push_back(m_vtTeachResult[tTeachDataS[i].nMeasurePtnNo]); 
		tAlgPtn.dCoorX = m_vtTeachResult[tTeachDataS[i].nMeasurePtnNo].tKeyPtn3D.x;
		tAlgPtn.dCoorY = m_vtTeachResult[tTeachDataS[i].nMeasurePtnNo].tKeyPtn3D.y;
		tAlgPtn.dCoorZ = m_vtTeachResult[tTeachDataS[i].nMeasurePtnNo].tKeyPtn3D.z;
		vtAlgPtns.push_back(tAlgPtn);
		//fprintf(pf2, "%d%11.3lf%11.3lf%11.3lf\n", i, tAlgPtn.dCoorX, tAlgPtn.dCoorY, tAlgPtn.dCoorZ);
	}
	//fclose(pf2);

	double dMin = 0.0;
	vector<T_ALGORITHM_POINT> vtOutputPoints(0);
	T_SPACE_CIRCLE_PARAM tSpaceCircleParam = alg.FitCircleUsing3PointsNew(vtAlgPtns, 0.9, dMin/*, 36, vtOutputPoints*/);

	double dPtnInterval = 2.0;
	double dRadius = tSpaceCircleParam.dRadius;
	int nPtnNum = 2.0 * PI * dRadius / 2.0;
	double dAngleInterval = 360.0 / (double)nPtnNum;

	tSpaceCircleParam = alg.FitCircleUsing3PointsNew(vtAlgPtns, 0.9, dMin, nPtnNum, vtOutputPoints);

	tRefSeamLine.CenterPoint.x = tSpaceCircleParam.tCenterPoint.dCoorX;
	tRefSeamLine.CenterPoint.y = tSpaceCircleParam.tCenterPoint.dCoorY;
	tRefSeamLine.CenterPoint.z = tSpaceCircleParam.tCenterPoint.dCoorZ;

	tRefSeamLine.StartPoint.x = vtOutputPoints[0].dCoorX;
	tRefSeamLine.StartPoint.y = vtOutputPoints[0].dCoorY;
	tRefSeamLine.StartPoint.z = vtOutputPoints[0].dCoorZ;

	tRefSeamLine.EndPoint.x = vtOutputPoints[0].dCoorX;
	tRefSeamLine.EndPoint.y = vtOutputPoints[0].dCoorY;
	tRefSeamLine.EndPoint.z = vtOutputPoints[0].dCoorZ;

	//tRefSeamLine.StartNormalVector.x = tSpaceCircleParam.tCircleNormalDir.dDirX; // ��㷨���б��溸��ռ�Բ��ƽ�淨��
	//tRefSeamLine.StartNormalVector.y = tSpaceCircleParam.tCircleNormalDir.dDirY;
	//tRefSeamLine.StartNormalVector.z = tSpaceCircleParam.tCircleNormalDir.dDirZ;

	// �ж���Ȧ��Ȧ����
	double dStartNorDirAngle = alg.CalcArcAngle(tRefSeamLine.StartNormalVector.x, tRefSeamLine.StartNormalVector.y);
	double dCenterToStartDirAngle = alg.CalcArcAngle(tRefSeamLine.StartPoint.x - tRefSeamLine.CenterPoint.x, tRefSeamLine.StartPoint.y - tRefSeamLine.CenterPoint.y);
	bool bOutsideCircle = JudgeDirAngle(dStartNorDirAngle, dCenterToStartDirAngle, 90.0);
	//bOutsideCircle = false;
	// ���� ��� ֱ������ �ⲿ������ ��������
	double dStartDirAngle = m_dWeldNorAngleInHome + 180.0;
	CString sFileName;
	sFileName.Format("%s%d_%d_RealWeldCoord.txt", m_sDataSavePath, nGroupNo, nWeldNo);
	FILE* pf = fopen(sFileName, "w");
	//for (int nPtnIdx = 0; nPtnIdx < vtOutputPoints.size(); nPtnIdx++)
	for (int nPtnIdx = 0; nPtnIdx < nPtnNum; nPtnIdx++)
	{
		//double dDirAngle = alg.CalcArcAngle(vtOutputPoints[nPtnIdx].dCoorX - tSpaceCircleParam.tCenterPoint.dCoorX, vtOutputPoints[nPtnIdx].dCoorY - tSpaceCircleParam.tCenterPoint.dCoorY);
		//T_ROBOT_COORS tCoord = GenerateRobotCoord(vtOutputPoints[nPtnIdx], m_dPlatWeldRx, m_dPlatWeldRy, m_pRobotDriver->DirAngleToRz(dDirAngle));
		double dCurDirAngle = dStartDirAngle + (double)nPtnIdx * dAngleInterval;
		T_ROBOT_COORS tCoord = GenerateRobotCoord(tSpaceCircleParam.tCenterPoint, m_dPlatWeldRx, m_dPlatWeldRy, DirAngleToRz(dCurDirAngle));
		double dOffsetDirAngle = bOutsideCircle ? dCurDirAngle : dCurDirAngle + 180.0; // ��Բ ��Բ����
		RobotCoordPosOffset(tCoord, dOffsetDirAngle, dRadius);
		
		fprintf(pf, "%d%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%4d%10d\n", nPtnIdx,
			tCoord.dX, tCoord.dY, tCoord.dZ, tCoord.dRX, tCoord.dRY, tCoord.dRZ, m_dTeachExAxlePos, m_dTeachExAxlePos_y, E_FLAT_SEAM, E_WELD_TRACK);
	}
	fclose(pf);
	return true;
}

E_WELD_SEAM_TYPE WeldAfterMeasure::GetWeldSeamType(const LineOrCircularArcWeldingLine& tWeldSeam)
{
	if (!IsStandWeldSeam(tWeldSeam.StartNormalVector))
	{
		return E_FLAT_SEAM;
	}
	else
	{
		return E_STAND_SEAM;
	}
}

bool WeldAfterMeasure::GetCurWeldParam(E_WELD_SEAM_TYPE eWeldSeamType, vector<T_WELD_PARA>& vtWeldPara)
{
	CWeldParamProcess cWeldParamProcess(m_ptUnit->m_tContralUnit.strUnitName);
	bool bRst = false;
	switch (eWeldSeamType)
	{
	case E_FLAT_SEAM: bRst = cWeldParamProcess.LoadCurUseParam(vtWeldPara, true);
		break;
	case E_STAND_SEAM: bRst = cWeldParamProcess.LoadCurUseParam(vtWeldPara, false);
		break;
	case E_PLAT_MULTIPLE:
		break;
	case E_STAND_MULTIPLE:
		break;
	case E_PLAT_GROOVE: bRst = cWeldParamProcess.LoadCurUseParam(vtWeldPara, true);
		break;
	case E_STAND_GROOVE:
		break;
	default:
		break;
	}
	// ���ָ�ƽ�� ʹ��ֱ������
	if (eWeldSeamType == E_FLAT_SEAM && m_eWorkPieceType == E_LINELLAE)
	{
		for (int i = 0; i < vtWeldPara.size(); i++)
		{
			vtWeldPara[i].nWeldMethod = 0;
		}
	}
	if (bRst == false)
	{
		XiMessageBox("�����˲����ڵĺ��ӹ���");
	}
	return bRst;
}

bool WeldAfterMeasure::GetWeldParam(E_WELD_SEAM_TYPE eWeldSeamType, int nWeldAngleSize, vector<T_WELD_PARA>& vtWeldPara)
{
	CWeldParamProcess cWeldParamProcess(m_ptUnit->m_tContralUnit.strUnitName);
	bool bRst = false;
	switch (eWeldSeamType)
	{
	case E_FLAT_SEAM: bRst = cWeldParamProcess.LoadWeldParam(true, nWeldAngleSize, vtWeldPara);
		break;
	case E_STAND_SEAM: bRst = cWeldParamProcess.LoadWeldParam(false, nWeldAngleSize, vtWeldPara);
		break;
	case E_PLAT_MULTIPLE:
		break;
	case E_STAND_MULTIPLE:
		break;
	case E_PLAT_GROOVE: bRst = cWeldParamProcess.LoadCurUseParam(vtWeldPara, true);
		break;
	case E_STAND_GROOVE:bRst = cWeldParamProcess.LoadCurUseParam(vtWeldPara, false);
		break;
	default:
		break;
	}
	// ���ָ�ƽ�� ʹ��ֱ������
	if (eWeldSeamType == E_FLAT_SEAM && m_eWorkPieceType == E_LINELLAE)
	{
		for (int i = 0; i < vtWeldPara.size(); i++)
		{
			vtWeldPara[i].nWeldMethod = m_nWeldMode;
		}
	}
	return bRst;
}

bool WeldAfterMeasure::CalcWrapTrack(WeldLineInfo tWeldLineInfo, E_WELD_SEAM_TYPE eWeldSeamType, double dStandWeldWrapLen, int nStandWeldDir,
	vector<T_ROBOT_COORS>& vtWeldCoord, vector<int>& vnPtnType)
{
	if (dStandWeldWrapLen > 5.0)
	{
		dStandWeldWrapLen = 5.0;
	}
	if (dStandWeldWrapLen < 0.0)
	{
		dStandWeldWrapLen = 0.0;
	}
	double dHeightDiff = tWeldLineInfo.tWeldLine.ZSide - tWeldLineInfo.tWeldLine.EndPoint.z;
	double dStandWrapCompHeight = 0.0; // �����յ�߶Ȳ��� ���϶ຸ���� �����ٺ���С (��ʱ�����ȵ��µ������߶Ȳ�׼)
	double dWrapEndPostureRy = m_dPlatWeldRy/*45.0*/; // �������ǻָ���ƽ����̬
	bool bRst = false; 
	T_ROBOT_COORS tRobotCoord;
	double dOffsetDirAngle = 0.0;
	vnPtnType.clear();
	switch (eWeldSeamType)
	{
	case E_FLAT_SEAM:
		for (int i = 0; i < vtWeldCoord.size(); i++)
		{
			vnPtnType.push_back(E_WELD_TRACK);
		}
		break;
	case E_STAND_SEAM:
		for (int i = 0; i < vtWeldCoord.size(); i++)
		{
			vnPtnType.push_back(E_WELD_TRACK);
		}
		tRobotCoord = vtWeldCoord[vtWeldCoord.size() - 1];
		tRobotCoord.dZ += dStandWrapCompHeight * ((double)m_nRobotInstallDir);
		tRobotCoord.dRY = dWrapEndPostureRy;
		if ((TRUE == *m_bNeedWrap) && (1 == nStandWeldDir)) // ѡ����� �� �����Ϻ��� �Ű���
		{
			vtWeldCoord.push_back(tRobotCoord); // ���ǹ켣��1
			vnPtnType.push_back(E_WELD_WRAP);
		}
		dOffsetDirAngle = RzToDirAngle(tRobotCoord.dRZ) + 180.0;

		// δ�����Զ�������Ƿ��� �� �ߵͲ�С��5mm  ʹ��Ĭ�ϰ��Ƿ�ʽ
		if (false == m_bAutoCalcWrapDirStand || fabs(dHeightDiff) < 5.0)
		{
			#ifdef SINGLE_ROBOT
					tRobotCoord.dY += (dStandWeldWrapLen * fabs(SinD(dOffsetDirAngle)) / SinD(dOffsetDirAngle));
			#else
					tRobotCoord.dX += (dStandWeldWrapLen * fabs(CosD(dOffsetDirAngle)) / CosD(dOffsetDirAngle));
			#endif // SINGLE_ROBOT
			//tRobotCoord.dX += (dStandWeldWrapLen * fabs(CosD(dOffsetDirAngle)) / CosD(dOffsetDirAngle));
		}
		else // ���ݷ���� + �߰��Ƿ����� �Զ�������Ƿ���ȱ����Ϣ������ȫͨ�ã�
		{
			AutoCalcStandWrapOffsetCoord(dStandWeldWrapLen, dOffsetDirAngle, tWeldLineInfo.tWeldLine.isLeft, tRobotCoord);
		}

		if ((TRUE == *m_bNeedWrap) && (1 == nStandWeldDir)) // ѡ����� �� �����Ϻ��� �Ű���
		{
			vtWeldCoord.push_back(tRobotCoord); // ���ǹ켣��2
			vnPtnType.push_back(E_WELD_WRAP);
		}
		break;
	default: 
		return false;
	}
	bRst = (vtWeldCoord.size() == vnPtnType.size() ? true : false);
	return bRst;
}

void WeldAfterMeasure::AutoCalcStandWrapOffsetCoord(double dDis, double dDirAngle, bool bHeightBoardIsLeft, T_ROBOT_COORS& tRobotCoord)
{
	double dWrapDirChange = 45.0;
	double dRobotDir = m_nRobotInstallDir > 0.0 ? 1.0 : -1.0;
	double dLeftRightDir = bHeightBoardIsLeft ? -1.0 : 1.0;
	double dOffsetDir = dDirAngle + dWrapDirChange * dRobotDir * dLeftRightDir;
	RobotCoordPosOffset(tRobotCoord, dOffsetDir, dDis);
}

void WeldAfterMeasure::TrackComp(int nGroupNo, E_WELD_SEAM_TYPE eWeldSeamType, const vector<T_ROBOT_COORS>& vtSrcWeldCoord, const T_WELD_PARA& tWeldPara, vector<T_ROBOT_COORS>& vtAdjustWeldCoord)
{
	bool bOnlyProcessComp = m_bFlatWeldContinue; //false;
	vtAdjustWeldCoord.clear();
	vtAdjustWeldCoord.resize(vtSrcWeldCoord.size());
	double dRyAngle = 0.0;
	double dDirAngle = 0.0;
	double dMultiLayerComp1 = tWeldPara.CrosswiseOffset;
	double dMultiLayerComp2 = tWeldPara.verticalOffset;
	double dWeldAngleComp = tWeldPara.dWeldAngle; // ���ղ�����˿�͵װ���������н�

	if (((E_FLAT_SEAM == eWeldSeamType) && (dWeldAngleComp < 30.0 || dWeldAngleComp > 60.0)) ||
		((E_STAND_SEAM == eWeldSeamType) && (dWeldAngleComp < 20.0 || dWeldAngleComp > 70.0)))
	{
		XUI::MesBox::PopOkCancel("{0}�� ����{1} ��{2}�����Ӳ��� ���ӽǶ�{3}�� ����ʹ��Ĭ��45.0��",
			E_FLAT_SEAM == eWeldSeamType ? "ƽ" : "��", tWeldPara.nWeldAngleSize, tWeldPara.nLayerNo + 1, dWeldAngleComp);
		dWeldAngleComp = m_dPlatWeldRy;
	}
	if ((E_PLAT_GROOVE == eWeldSeamType) && (dWeldAngleComp < -21.0 || dWeldAngleComp > 30.0))
	{
		XUI::MesBox::PopOkCancel("ƽ���¿� ��{0}�����Ӳ��� ���ӽǶ�{1}�� ����ʹ��Ĭ��0��(��ֱ����)", tWeldPara.nLayerNo + 1, dWeldAngleComp);
		dWeldAngleComp = 0.0;
	}

	if (E_FLAT_SEAM == eWeldSeamType) // ƽ������
	{
		if (E_FLAT_SEAM == eWeldSeamType&&vtSrcWeldCoord.size() > 300)//ƽ���ҳ��ȳ���300�Ӹ���
		{
			m_pRobotDriver->m_bIsOpenTrack = false;
		}
		else
		{
			m_pRobotDriver->m_bIsOpenTrack = false;
		}
		for (int nPtnNo = 0; nPtnNo < vtSrcWeldCoord.size(); nPtnNo++)
		{
			double dFlatHorComp = 0.0;
			double dFlatHeightComp = 0.0;
			if (bOnlyProcessComp)
			{
				dDirAngle = RzToDirAngle(vtSrcWeldCoord[nPtnNo].dRZ); // ȡ�е�Rz���㷽���
				dFlatHorComp = dMultiLayerComp1;
				dFlatHeightComp = dMultiLayerComp2;
			}
			else
			{
				//if (nPtnNo < m_ChangeDisPoint)
				//{
				//	dDirAngle = RzToDirAngle(vtSrcWeldCoord[m_ChangeDisPoint].dRZ); // ȡ�е�Rz���㷽���
				//}
				//else if (nPtnNo > vtSrcWeldCoord.size() - m_ChangeDisPoint)
				//{
				//	dDirAngle = RzToDirAngle(vtSrcWeldCoord[vtSrcWeldCoord.size() - m_ChangeDisPoint - 1].dRZ); // ȡ�е�Rz���㷽���
				//}
				if (E_SCAN_WELD_LINE == m_eWorkPieceType)
				{
					dDirAngle = RzToDirAngle(vtSrcWeldCoord[nPtnNo/*vtSrcWeldCoord.size() / 2*/].dRZ); // ȡ�е�Rz���㷽���
				}
				else
				{
					dDirAngle = RzToDirAngle(vtSrcWeldCoord[vtSrcWeldCoord.size() / 2].dRZ); // ȡ�е�Rz���㷽���
				}
				int n1 = dDirAngle / 45.0;
				int n2 = (fmod(dDirAngle, 45.0) / 45.0 > 0.5) ? 45 : 0;
				int nCompDirAngleIdx = (n1 * 45 + n2) % 360;
				dFlatHorComp = m_mdFlatHorComp[nCompDirAngleIdx] + dMultiLayerComp1;
				dFlatHeightComp = m_mdFlatHeightComp[nCompDirAngleIdx] + dMultiLayerComp2;
				if (m_pRobotDriver->m_bIsOpenTrack)
				{
					dFlatHorComp = 0;
					dFlatHeightComp = 0;
				}
			}

			vtAdjustWeldCoord[nPtnNo] = vtSrcWeldCoord[nPtnNo];
			vtAdjustWeldCoord[nPtnNo].dX += (dFlatHorComp * CosD(dDirAngle));
			vtAdjustWeldCoord[nPtnNo].dY += (dFlatHorComp * SinD(dDirAngle));
			vtAdjustWeldCoord[nPtnNo].dZ += dFlatHeightComp * (double)m_nRobotInstallDir;
			vtAdjustWeldCoord[nPtnNo].dRY = dWeldAngleComp * (double)m_nRobotInstallDir;
			vtAdjustWeldCoord[nPtnNo].dRZ -= tWeldPara.dWeldDipAngle;
		}
	}
	else if (E_STAND_SEAM == eWeldSeamType) // ��������
	{	
		for (int nPtnNo = 0; nPtnNo < vtSrcWeldCoord.size(); nPtnNo++)
		{
 			dRyAngle = 90.0 * m_nRobotInstallDir - vtSrcWeldCoord[nPtnNo].dRY; // ��Ry90��ˮƽ����Ϊ��׼
			dDirAngle = RzToDirAngle(vtSrcWeldCoord[nPtnNo].dRZ);

			// ��ͬ��������������ͬ
			int n1 = dDirAngle / 45.0;
			int n2 = (fmod(dDirAngle, 45.0) / 45.0 > 0.5) ? 45 : 0;
			int nCompDirAngleIdx = (n1 * 45 + n2) % 360;
			double dStandLenComp = m_mdStandLenComp[nCompDirAngleIdx] + dMultiLayerComp1;
			double dStandVerComp = m_mdStandVerComp[nCompDirAngleIdx] + dMultiLayerComp2;

			vtAdjustWeldCoord[nPtnNo] = vtSrcWeldCoord[nPtnNo];
			vtAdjustWeldCoord[nPtnNo].dX += (dStandLenComp * CosD(dRyAngle) * CosD(dDirAngle)); // ���쳤�Ȳ���
			vtAdjustWeldCoord[nPtnNo].dY += (dStandLenComp * CosD(dRyAngle) * SinD(dDirAngle));
			vtAdjustWeldCoord[nPtnNo].dZ += (dStandLenComp * SinD(dRyAngle));
			vtAdjustWeldCoord[nPtnNo].dX += (dStandVerComp * CosD(dDirAngle + 90.0)); // ��˿��ֱ���򲹳�
			vtAdjustWeldCoord[nPtnNo].dY += (dStandVerComp * SinD(dDirAngle + 90.0));
			vtAdjustWeldCoord[nPtnNo].dRZ += ((dWeldAngleComp - 45.0) * m_nRobotInstallDir);
		}
	}
	else if(E_PLAT_GROOVE == eWeldSeamType) // ƽ���¿�
	{
		XiAlgorithm alg;
		dDirAngle = alg.CalcArcAngle(
			m_vvtWeldLineInfoGroup[nGroupNo][0].tWeldLine.StartNormalVector.x, 
			m_vvtWeldLineInfoGroup[nGroupNo][0].tWeldLine.StartNormalVector.y);
		double dRxOffsetDir = dDirAngle > RzToDirAngle(m_dGrooveWeldRz) ? -1.0 : 1.0;
		int n1 = dDirAngle / 45.0;
		int n2 = (fmod(dDirAngle, 45.0) / 45.0 > 0.5) ? 45 : 0;
		int nCompDirAngleIdx = (n1 * 45 + n2) % 360;
		double dFlatHorComp = m_mdFlatHorComp[nCompDirAngleIdx] + dMultiLayerComp1;
		double dFlatHeightComp = m_mdFlatHeightComp[nCompDirAngleIdx] + dMultiLayerComp2;
		for (int nPtnNo = 0; nPtnNo < vtSrcWeldCoord.size(); nPtnNo++)
		{
			//vtAdjustWeldCoord[nPtnNo] = vtSrcWeldCoord[nPtnNo];
			//vtAdjustWeldCoord[nPtnNo].dX += (dFlatHorComp * CosD(dDirAngle));
			//vtAdjustWeldCoord[nPtnNo].dY += (dFlatHorComp * SinD(dDirAngle));
			//vtAdjustWeldCoord[nPtnNo].dZ += dFlatHeightComp * m_nRobotInstallDir;
			//vtAdjustWeldCoord[nPtnNo].dRX += (dWeldAngleComp * m_nRobotInstallDir * dRxOffsetDir);

			vtAdjustWeldCoord[nPtnNo] = vtSrcWeldCoord[nPtnNo];
			vtAdjustWeldCoord[nPtnNo].dX += dFlatHorComp;
			vtAdjustWeldCoord[nPtnNo].dZ += dFlatHeightComp * (double)m_nRobotInstallDir;
		}
	}
	else if (E_STAND_GROOVE == eWeldSeamType) // �����¿� ������ʱ
	{
		double dDirAngle = 0.0;
		if ( fabs(vtSrcWeldCoord.front().dRX - m_pRobotDriver->m_tStandUpGrooveWeldPosture.dRX) < 5.0) // �Ϸ�����λ�ò���ʹ��ƽ��90��
		{
			dDirAngle = 90.0;
		}
		//double dRxOffsetDir = dDirAngle > RzToDirAngle(m_dGrooveWeldRz) ? -1.0 : 1.0;
		int n1 = dDirAngle / 45.0;
		int n2 = (fmod(dDirAngle, 45.0) / 45.0 > 0.5) ? 45 : 0;
		int nCompDirAngleIdx = (n1 * 45 + n2) % 360;
		double dFlatHorComp = m_mdFlatHorComp[nCompDirAngleIdx] + dMultiLayerComp1;
		double dFlatHeightComp = m_mdFlatHeightComp[nCompDirAngleIdx] + dMultiLayerComp2;
		for (int nPtnNo = 0; nPtnNo < vtSrcWeldCoord.size(); nPtnNo++)
		{
			T_ROBOT_COORS tS = vtSrcWeldCoord.front();
			T_ROBOT_COORS tE = vtSrcWeldCoord.back();
			double dDis = TwoPointDis(tS.dX + tS.dBX, tS.dY + tS.dBY, tS.dZ + tS.dBZ, tE.dX + tE.dBX, tE.dY + tE.dBY, tE.dZ + tE.dBZ);
			double dDisY = (tE.dY + tE.dBY) - (tS.dY + tS.dBY);
			double dDisZ = (tE.dZ + tE.dBZ) - (tS.dZ + tS.dBZ);
			double dVerY = dDisY / dDis;
			double dVerZ = dDisZ / dDis;

			vtAdjustWeldCoord[nPtnNo] = vtSrcWeldCoord[nPtnNo];
			vtAdjustWeldCoord[nPtnNo].dX += dFlatHorComp;

			vtAdjustWeldCoord[nPtnNo].dY += dFlatHeightComp * (-dVerZ);
			vtAdjustWeldCoord[nPtnNo].dZ += dFlatHeightComp * dVerY;
		}
	}
}

double WeldAfterMeasure::CalcBoardMaxHeight(int nGroupNo)
{
	const vector<LineOrCircularArcWeldingLine>& vtWeldLineSeam = m_vvtWeldSeamGroup[nGroupNo];
	double dMaxHeight = -99999.0; // ��¼������ߵ�Zֵ
	for (int nWeldSeamNo = 0; nWeldSeamNo < vtWeldLineSeam.size(); nWeldSeamNo++)
	{
		dMaxHeight = dMaxHeight < vtWeldLineSeam[nWeldSeamNo].StartPoint.z * m_nRobotInstallDir ? vtWeldLineSeam[nWeldSeamNo].StartPoint.z * m_nRobotInstallDir : dMaxHeight;
		dMaxHeight = dMaxHeight < vtWeldLineSeam[nWeldSeamNo].EndPoint.z * m_nRobotInstallDir ? vtWeldLineSeam[nWeldSeamNo].EndPoint.z * m_nRobotInstallDir : dMaxHeight;
		dMaxHeight = dMaxHeight < vtWeldLineSeam[nWeldSeamNo].ZSide * m_nRobotInstallDir ? vtWeldLineSeam[nWeldSeamNo].ZSide * m_nRobotInstallDir : dMaxHeight;
	}
	return dMaxHeight * m_nRobotInstallDir;
}

bool WeldAfterMeasure::IsStandWeldSeam(CvPoint3D64f tVector)
{
	double dVectorDis = sqrt((double)(tVector.x*tVector.x + tVector.y*tVector.y + tVector.z*tVector.z));
	double dZDis = fabs(tVector.z);
	double dAngle = asin(dZDis / dVectorDis)*180.0 / PI;
	return(dAngle < 15.0);
}

bool WeldAfterMeasure::ResetWeldSeamGroup(int nGroupNo, vector<LineOrCircularArcWeldingLine>& vtWeldLineSeam)
{
	double dDefaultBoardThick = 15.0;
	int nWeldNum = vtWeldLineSeam.size();
	if (nWeldNum > 3) // �������
	{
		return true;
	}
	if (nWeldNum <= 0)
	{
		XiMessageBox("��ϰ庸�������쳣!");
		return false;
	}
	// ƽ���� �� �����ɶ˵� �쳣
	if ((false == IsStandWeldSeam(vtWeldLineSeam[0].StartNormalVector)) && 
		((false == vtWeldLineSeam[0].StartPointType) || (false == vtWeldLineSeam[0].EndPointType)))
	{
		XiMessageBox("�޷�Ϊ�������ɶ˵ķ�ϰ岹ȫ������Ϣ!");
		return false;
	}

	XiAlgorithm alg;
	LineOrCircularArcWeldingLine tWeldSeam;
	vector<LineOrCircularArcWeldingLine> vtWeldSeamBuffer = vtWeldLineSeam;
	vtWeldLineSeam.clear();
	double dFlatNorAngle = alg.CalcArcAngle(vtWeldSeamBuffer[0].StartNormalVector.x, vtWeldSeamBuffer[0].StartNormalVector.y);
	double dPtnOffsetDir = dFlatNorAngle + 180.0;
	double dOffsetX = (dDefaultBoardThick * CosD(dPtnOffsetDir));
	double dOffsetY = (dDefaultBoardThick * SinD(dPtnOffsetDir));

	vtWeldLineSeam.push_back(vtWeldSeamBuffer[0]); // ƽ��1
	tWeldSeam = vtWeldSeamBuffer[0];
	swap(tWeldSeam.StartPoint, tWeldSeam.EndPoint);
	tWeldSeam.StartPoint.x += dOffsetX;
	tWeldSeam.StartPoint.y += dOffsetY;
	tWeldSeam.EndPoint.x += dOffsetX;
	tWeldSeam.EndPoint.y += dOffsetY;
	tWeldSeam.StartNormalVector.x *= -1.0;
	tWeldSeam.StartNormalVector.y *= -1.0;
	tWeldSeam.EndNormalVector = tWeldSeam.StartNormalVector;
	vtWeldLineSeam.push_back(tWeldSeam); // ƽ��2
	vtWeldLineSeam.push_back(vtWeldSeamBuffer[1]); // ����1
	vtWeldLineSeam.push_back(vtWeldSeamBuffer[2]); // ����2
	tWeldSeam = vtWeldSeamBuffer[2];
	tWeldSeam.StartPoint.x += dOffsetX;
	tWeldSeam.StartPoint.y += dOffsetY;
	tWeldSeam.EndPoint.x += dOffsetX;
	tWeldSeam.EndPoint.y += dOffsetY;
	double dNorVectorXYDis = sqrt(SQUARE(tWeldSeam.StartNormalVector.x) + SQUARE(tWeldSeam.StartNormalVector.y));
	double dStandWeldNorAngle = alg.CalcArcAngle(tWeldSeam.StartNormalVector.x, tWeldSeam.StartNormalVector.y) - 90.0;
	tWeldSeam.StartNormalVector.x = dNorVectorXYDis * CosD(dStandWeldNorAngle);
	tWeldSeam.StartNormalVector.y = dNorVectorXYDis * SinD(dStandWeldNorAngle);
	tWeldSeam.EndNormalVector = tWeldSeam.StartNormalVector;
	vtWeldLineSeam.push_back(tWeldSeam); // ƽ��3
	tWeldSeam = vtWeldSeamBuffer[1];
	tWeldSeam.StartPoint.x += dOffsetX;
	tWeldSeam.StartPoint.y += dOffsetY;
	tWeldSeam.EndPoint.x += dOffsetX;
	tWeldSeam.EndPoint.y += dOffsetY;
	dNorVectorXYDis = sqrt(SQUARE(tWeldSeam.StartNormalVector.x) + SQUARE(tWeldSeam.StartNormalVector.y));
	dStandWeldNorAngle = alg.CalcArcAngle(tWeldSeam.StartNormalVector.x, tWeldSeam.StartNormalVector.y) + 90.0;
	tWeldSeam.StartNormalVector.x = dNorVectorXYDis * CosD(dStandWeldNorAngle);
	tWeldSeam.StartNormalVector.y = dNorVectorXYDis * SinD(dStandWeldNorAngle);
	tWeldSeam.EndNormalVector = tWeldSeam.StartNormalVector;
	vtWeldLineSeam.push_back(tWeldSeam); // ƽ��4
	// ����ָ��ĺ�����Ϣ
	CString sFileName;
	sFileName.Format("%s%d_ResetWeldSeamGroup.txt", m_sDataSavePath, nGroupNo);
	FILE* pf = fopen(sFileName.GetBuffer(), "w");
	for (int nWeldSeamNo = 0; nWeldSeamNo < vtWeldLineSeam.size(); nWeldSeamNo++)
	{
		LineOrCircularArcWeldingLine tWeld = vtWeldLineSeam[nWeldSeamNo];
		fprintf(pf, "%d%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%4d%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%4d%4d\n",
			nGroupNo, tWeld.StartPoint.x, tWeld.StartPoint.y, tWeld.StartPoint.z,
			tWeld.EndPoint.x, tWeld.EndPoint.y, tWeld.EndPoint.z,
			tWeld.ZSide, tWeld.IsArc, tWeld.CenterPoint.x, tWeld.CenterPoint.y, tWeld.CenterPoint.z,
			tWeld.StartNormalVector.x, tWeld.StartNormalVector.y, tWeld.StartNormalVector.z,
			tWeld.EndNormalVector.x, tWeld.EndNormalVector.y, tWeld.EndNormalVector.z,
			tWeld.StartPointType, tWeld.EndPointType);
	}
	fclose(pf);
	return true;
}

bool WeldAfterMeasure::JudgeCollision(double dRy, double dAngle, double dBackH, double dLH, double dRH,
	double dFrontBackDis, double dLRDis, double dGunSafeDis, double dSafeDis)
{
	// dRy : ������Ryֵ(�������н�) ���磺70��ʾ��˿������н�70�� ��װ�н�90��- 70��
	// dAngle : ��˿�������ļнǣ�����ʱ90�㣬����ʱ45�㣩
	// dBackH : ��˿��ָ�����߶� 
	// dLH dRH : �����泯��˿ָ���򣬺�ǹ�����Ҳ��߶�
	// dFrontBackDis : �غ�˿�������������
	// dLRDis : ���Ӵ�ֱ��˿�����������������
	// dGunSafeDis : �����������(���۾���)�ͺ���(����0)��ǹ��������İ�ȫ���루�ͻ�е�ṹ�йأ�
	// dSafeDis : ���沿����С��ȫ����

	// �жϺ�ǹ�����Ƿ����

	// �ж�����Ƿ����

	return true;
}

bool WeldAfterMeasure::AdjustLineSeamTrack(E_WELD_SEAM_TYPE eWeldSeamType, const vector<XI_POINT>& vtScanPtns, vector<T_ROBOT_COORS>& vtWeldTrack)
{
	XiAlgorithm alg;
	double dMaxTotalDisErr = 30.0;
	double dMaxAdjacentDisErr = 10.0;
	int nTeachResultNum = vtScanPtns.size();
	int nTotalPtnNum = nTeachResultNum + 2;
	int nWeldTrackNum = vtWeldTrack.size();
	T_ROBOT_COORS tStartCoord = vtWeldTrack[0];
	XI_POINT tStartPtn = { vtWeldTrack[0].dX,vtWeldTrack[0].dY,vtWeldTrack[0].dZ };
	XI_POINT tEndPtn = { vtWeldTrack[nWeldTrackNum - 1].dX,vtWeldTrack[nWeldTrackNum - 1].dY,vtWeldTrack[nWeldTrackNum - 1].dZ };
	vector<XI_POINT> vtKeyPtn3D;
	vtKeyPtn3D.resize(nTotalPtnNum);
	vector<T_ALGORITHM_POINT> vtAlgPtns;
	vtAlgPtns.resize(nTotalPtnNum);

	if (nTotalPtnNum < 2)
	{
		XiMessageBox("ɨ�����������ݹ��٣�");
		return false;
	}

	if (nTeachResultNum >= 2) // ������յ��������������ϵ�ɨ�������� ���������
	{
		XI_POINT tDir;
		tDir.x = vtScanPtns[0].x - vtScanPtns[1].x;
		tDir.y = vtScanPtns[0].y - vtScanPtns[1].y;
		tDir.z = vtScanPtns[0].z - vtScanPtns[1].z;
		double dBaseDis = TwoPointDis(vtScanPtns[0].x, vtScanPtns[0].y, vtScanPtns[0].z, vtScanPtns[1].x, vtScanPtns[1].y, vtScanPtns[1].z);
		double dOffsetDis = TwoPointDis(tStartPtn.x, tStartPtn.y, tStartPtn.z, vtScanPtns[0].x, vtScanPtns[0].y, vtScanPtns[0].z);
		tStartPtn.x = vtScanPtns[0].x + dOffsetDis * tDir.x / dBaseDis;
		tStartPtn.y = vtScanPtns[0].y + dOffsetDis * tDir.y / dBaseDis;
		tStartPtn.z = vtScanPtns[0].z + dOffsetDis * tDir.z / dBaseDis;
	}
	vtKeyPtn3D[0] = tStartPtn;
	vtAlgPtns[0].dCoorX = tStartPtn.x;
	vtAlgPtns[0].dCoorY = tStartPtn.y;
	vtAlgPtns[0].dCoorZ = tStartPtn.z;
	for (int i = 0; i < nTeachResultNum; i++)
	{
		vtKeyPtn3D[i + 1] = vtScanPtns[i];
		vtAlgPtns[i + 1].dCoorX = vtScanPtns[i].x;
		vtAlgPtns[i + 1].dCoorY = vtScanPtns[i].y;
		vtAlgPtns[i + 1].dCoorZ = vtScanPtns[i].z;
	}
	// ʹ�ó��յ��������������������յ�
	XI_POINT tDir;
	tDir.x = vtKeyPtn3D[nTotalPtnNum - 2].x - vtKeyPtn3D[nTotalPtnNum - 3].x;
	tDir.y = vtKeyPtn3D[nTotalPtnNum - 2].y - vtKeyPtn3D[nTotalPtnNum - 3].y;
	tDir.z = vtKeyPtn3D[nTotalPtnNum - 2].z - vtKeyPtn3D[nTotalPtnNum - 3].z;
	double dBaseDis = TwoPointDis(vtKeyPtn3D[nTotalPtnNum - 2].x, vtKeyPtn3D[nTotalPtnNum - 2].y, vtKeyPtn3D[nTotalPtnNum - 2].z,
		vtKeyPtn3D[nTotalPtnNum - 3].x, vtKeyPtn3D[nTotalPtnNum - 3].y, vtKeyPtn3D[nTotalPtnNum - 3].z);
	double dOffsetDis = TwoPointDis(tEndPtn.x, tEndPtn.y, tEndPtn.z,
		vtKeyPtn3D[nTotalPtnNum - 2].x, vtKeyPtn3D[nTotalPtnNum - 2].y, vtKeyPtn3D[nTotalPtnNum - 2].z);
	tEndPtn.x = vtKeyPtn3D[nTotalPtnNum - 2].x + dOffsetDis * tDir.x / dBaseDis;
	tEndPtn.y = vtKeyPtn3D[nTotalPtnNum - 2].y + dOffsetDis * tDir.y / dBaseDis;
	tEndPtn.z = vtKeyPtn3D[nTotalPtnNum - 2].z + dOffsetDis * tDir.z / dBaseDis;

	vtKeyPtn3D[nTotalPtnNum - 1] = tEndPtn;
	vtAlgPtns[nTotalPtnNum - 1].dCoorX = tEndPtn.x;
	vtAlgPtns[nTotalPtnNum - 1].dCoorY = tEndPtn.y;
	vtAlgPtns[nTotalPtnNum - 1].dCoorZ = tEndPtn.z;

	T_ALGORITHM_LINE_PARA tAlgLine = alg.CalcLineParamRansac(vtAlgPtns, 0.8);
	T_ALGORITHM_POINT tSegmentStartPoint = tAlgLine.tPoint;
	T_ALGORITHM_POINT tSegmentEndPoint = {
		tAlgLine.tPoint.dCoorX + tAlgLine.tLineDir.dDirX,
		tAlgLine.tPoint.dCoorY + tAlgLine.tLineDir.dDirY,
		tAlgLine.tPoint.dCoorZ + tAlgLine.tLineDir.dDirZ };

	sort(vtKeyPtn3D.begin(), vtKeyPtn3D.end(),
		[tStartPtn](const XI_POINT& tPtn1, const XI_POINT& tPtn2) -> bool
		{
			double dDis1 = TwoPointDis(tStartPtn.x, tStartPtn.y, tStartPtn.z, tPtn1.x, tPtn1.y, tPtn1.z);
			double dDis2 = TwoPointDis(tStartPtn.x, tStartPtn.y, tStartPtn.z, tPtn2.x, tPtn2.y, tPtn2.z);
			return (dDis1 < dDis2);
		});

	vtWeldTrack.clear();
	vtWeldTrack.reserve(nWeldTrackNum + 20); // ������������ռ�
	vtWeldTrack.push_back(tStartCoord);
	double dDirAngle = RzToDirAngle(tStartCoord.dRZ);
	vector<T_ROBOT_COORS> vtCoord;
	T_ALGORITHM_POINT tPtn;
	tPtn.dCoorX = vtKeyPtn3D[0].x;
	tPtn.dCoorY = vtKeyPtn3D[0].y;
	tPtn.dCoorZ = vtKeyPtn3D[0].z;

	double dPreErrDis = 0.0;
	double dCurErrDis = alg.CalDisPointToLine(tPtn, tSegmentStartPoint, tSegmentEndPoint);
	if (dCurErrDis > dMaxTotalDisErr/*dMaxAdjacentDisErr*/)
	{
		XUI::MesBox::PopOkCancel("������1���{0}����!", dCurErrDis);
		return false;
	}
	dPreErrDis = dCurErrDis;
	for (int nPtnNo = 1; nPtnNo < vtKeyPtn3D.size(); nPtnNo++)
	{
		tPtn.dCoorX = vtKeyPtn3D[nPtnNo].x;
		tPtn.dCoorY = vtKeyPtn3D[nPtnNo].y;
		tPtn.dCoorZ = vtKeyPtn3D[nPtnNo].z;
		dCurErrDis = alg.CalDisPointToLine(tPtn, tSegmentStartPoint, tSegmentEndPoint);
		if (dCurErrDis > dMaxTotalDisErr || fabs(dCurErrDis - dPreErrDis) > dMaxAdjacentDisErr)
		{
			//���޸�
			XUI::MesBox::PopInfo("������{0}���{1:.3f}ǰһ�������{2:.3f}������!", nPtnNo + 1, dCurErrDis, dPreErrDis);
			//XiMessageBox("������%d ���%.3lf ǰһ�������%.3lf ������!", nPtnNo + 1, dCurErrDis, dPreErrDis);
			return false;
		}
		dPreErrDis = dCurErrDis;
		CHECK_BOOL_RETURN(GenerateWeldLineData(vtKeyPtn3D[nPtnNo - 1], vtKeyPtn3D[nPtnNo], 1.0, tStartCoord.dRX, tStartCoord.dRY, dDirAngle, eWeldSeamType, vtCoord));
		for (int nTrackPtnNo = 1; nTrackPtnNo < vtCoord.size(); nTrackPtnNo++)
		{
			vtWeldTrack.push_back(vtCoord[nTrackPtnNo]);
		}
	}
	vtWeldTrack.shrink_to_fit();
	return true;
}

bool WeldAfterMeasure::CalcTwoPulseTempPtn(T_ANGLE_PULSE& tTempPulse, T_ANGLE_PULSE tStartPulse, T_ANGLE_PULSE tEndPulse, double dLevelOffsetDis, double dHeightOffset)
{
	double dMinDisThreshold = 10.0;
	T_ROBOT_COORS tStartCoord;
	T_ROBOT_COORS tEndCoord;

	m_pRobotDriver->RobotKinematics(tStartPulse, m_pRobotDriver->m_tTools.tGunTool, tStartCoord);
	m_pRobotDriver->RobotKinematics(tEndPulse, m_pRobotDriver->m_tTools.tGunTool, tEndCoord);

	double dDis = TwoPointDis(tStartCoord.dX, tStartCoord.dY, tStartCoord.dZ, tEndCoord.dX, tEndCoord.dY, tEndCoord.dZ);
	if (dDis < dMinDisThreshold)
	{
		XiMessageBox("���ؽ����������̣�������ɵ�ʧ��!");
		return false;
	}

	T_ROBOT_COORS tMidCoord;
	T_ANGLE_PULSE tMidPulse;
	tMidPulse.nSPulse = (tStartPulse.nSPulse + tEndPulse.nSPulse) / 2;
	tMidPulse.nLPulse = (tStartPulse.nLPulse + tEndPulse.nLPulse) / 2;
	tMidPulse.nUPulse = (tStartPulse.nUPulse + tEndPulse.nUPulse) / 2;
	tMidPulse.nRPulse = (tStartPulse.nRPulse + tEndPulse.nRPulse) / 2;
	tMidPulse.nBPulse = (tStartPulse.nBPulse + tEndPulse.nBPulse) / 2;
	tMidPulse.nTPulse = (tStartPulse.nTPulse + tEndPulse.nTPulse) / 2;
	m_pRobotDriver->RobotKinematics(tMidPulse, m_pRobotDriver->m_tTools.tGunTool, tMidCoord);

	tMidCoord.dX = (tStartCoord.dX + tEndCoord.dX) / 2.0;
	tMidCoord.dY = (tStartCoord.dY + tEndCoord.dY) / 2.0;
	tMidCoord.dZ = (tStartCoord.dZ + tEndCoord.dZ) / 2.0 + dHeightOffset;

	XiAlgorithm alg;
	double dMoveDirAngle = alg.CalcArcAngle(tEndCoord.dX - tStartCoord.dX, tEndCoord.dY - tStartCoord.dY);
	tMidCoord.dX += (dLevelOffsetDis * CosD(dMoveDirAngle - 90.0));
	tMidCoord.dY += (dLevelOffsetDis * SinD(dMoveDirAngle - 90.0));

	return m_pRobotDriver->RobotInverseKinematics(tMidCoord, tMidPulse, m_pRobotDriver->m_tTools.tGunTool, tTempPulse);
}

bool WeldAfterMeasure::JudgeOutsideWeld(const vector<LineOrCircularArcWeldingLine>& vtWeldSeam)
{
	vector<LineOrCircularArcWeldingLine> tFlatWeldSeam(0);
	for (int i = 0; i < vtWeldSeam.size(); i++)
	{
		if (!IsStandWeldSeam(vtWeldSeam[i].StartNormalVector))
		{
			tFlatWeldSeam.push_back(vtWeldSeam[i]);
		}
	}
	if (tFlatWeldSeam.size() <= 1)
	{
		return false;
	}
	LineOrCircularArcWeldingLine tFirstWeldLine = tFlatWeldSeam[0];
	LineOrCircularArcWeldingLine tSecondWeldLine = tFlatWeldSeam[1];
	CvPoint3D64f tFirstMidPtn = cvPoint3D64f(
		(tFirstWeldLine.StartPoint.x + tFirstWeldLine.EndPoint.x) / 2.0,
		(tFirstWeldLine.StartPoint.y + tFirstWeldLine.EndPoint.y) / 2.0,
		(tFirstWeldLine.StartPoint.z + tFirstWeldLine.EndPoint.z) / 2.0);
	CvPoint3D64f tSecondMidPtn = cvPoint3D64f(
		(tSecondWeldLine.StartPoint.x + tSecondWeldLine.EndPoint.x) / 2.0,
		(tSecondWeldLine.StartPoint.y + tSecondWeldLine.EndPoint.y) / 2.0,
		(tSecondWeldLine.StartPoint.z + tSecondWeldLine.EndPoint.z) / 2.0);
	XiAlgorithm alg;
	double dDirAngle1 = alg.CalcArcAngle(tFirstWeldLine.StartNormalVector.x, tFirstWeldLine.StartNormalVector.y);
	double dDirAngle2 = alg.CalcArcAngle(tSecondMidPtn.x - tFirstMidPtn.x, tSecondMidPtn.y - tFirstMidPtn.y);
	bool bIsOutsideWeldLine = (!JudgeDirAngle(dDirAngle1, dDirAngle2, 90.0));
	return bIsOutsideWeldLine;
}

bool WeldAfterMeasure::GetContiueWeldOrderFlat(const vector<LineOrCircularArcWeldingLine>& vtWeldSeam, vector<int>& vnFlatWeldNoIdx)
{
	vnFlatWeldNoIdx.clear();
	double dGroupDisThreshold = 3.0; // mm
	struct SortData
	{
		int nWeldNo;
		double dWeldNorAngle;
		LineOrCircularArcWeldingLine tWeldLine;
	};
	SortData tSortData;
	vector<SortData> vtSortData(0);

	double dStartDirAngle = m_dWeldNorAngleInHome + 180.0;
	//// �ຸ����������� ���취��仯����
	//double dWeldChangeDir = 1 == m_nRobotInstallDir ? 1 : -1.0;
	
	XiAlgorithm alg;
	for (int i = 0; i < vtWeldSeam.size(); i++)
	{
		if (IsStandWeldSeam(vtWeldSeam[i].StartNormalVector))
		{
			continue;
		}
		tSortData.nWeldNo = i;
		tSortData.dWeldNorAngle = alg.CalcArcAngle(vtWeldSeam[i].StartNormalVector.x, vtWeldSeam[i].StartNormalVector.y);
		tSortData.tWeldLine = vtWeldSeam[i];
		vtSortData.push_back(tSortData);
	}
	if (0 == vtSortData.size()) // ��ƽ������
	{
		return true;
	}

	// ���ҵ�һ������(�������˵��������ͬ�����޴˺��죬ѡ������dStartDirAngle������ĺ���)
	int nFirstIdx = -1;
	int nMinIdx = -1;
	for (int i = 0; i < vtSortData.size(); i++)
	{
		vector<SortData> vtTempSortData(vtSortData);
		SortData tFirstData = vtSortData[i];
		if (JudgeDirAngle(dStartDirAngle,alg.CalcArcAngle(tFirstData.tWeldLine.StartNormalVector.x, tFirstData.tWeldLine.StartNormalVector.y), 45.0))
		{
			nMinIdx = i;
		}
		vtTempSortData.erase(vtTempSortData.begin() + i);
		bool bExistSameStartPtn = false;
		for (int j = 0; (false == bExistSameStartPtn) && (j < vtTempSortData.size()); j++)
		{
			if (dGroupDisThreshold > TwoPointDis(
				tFirstData.tWeldLine.StartPoint.x, tFirstData.tWeldLine.StartPoint.y, tFirstData.tWeldLine.StartPoint.z,
				vtTempSortData[j].tWeldLine.StartPoint.x, vtTempSortData[j].tWeldLine.StartPoint.y, vtTempSortData[j].tWeldLine.StartPoint.z))
			{
				bExistSameStartPtn = true;
			}
			if (dGroupDisThreshold > TwoPointDis(
				tFirstData.tWeldLine.StartPoint.x, tFirstData.tWeldLine.StartPoint.y, tFirstData.tWeldLine.StartPoint.z,
				vtTempSortData[j].tWeldLine.EndPoint.x, vtTempSortData[j].tWeldLine.EndPoint.y, vtTempSortData[j].tWeldLine.EndPoint.z))
			{
				bExistSameStartPtn = true;
			}
		}
		if (false == bExistSameStartPtn)
		{
			if (-1 != nFirstIdx)
			{
				XiMessageBoxOk("GetContiueWeldOrderFlat������������ƽ���죬�������ӹ켣����ʧ�ܣ�");
				return false;
			}
			nFirstIdx = i;
		}
	}

	if (-1 == nFirstIdx && -1 == nMinIdx)
	{
		XiMessageBoxOk("GetContiueWeldOrderFlat:����ʧ�ܣ�");
		return false;
	}
	nFirstIdx = (-1 == nFirstIdx) ? nMinIdx : nFirstIdx;
	vector<SortData> vtTempSortData(vtSortData);
	vtSortData.clear();
	vtSortData.push_back(vtTempSortData[nFirstIdx]);
	vtTempSortData.erase(vtTempSortData.begin() + nFirstIdx);
	while (vtTempSortData.size() > 0)
	{
		for (int i = 0; i < vtTempSortData.size(); i++)
		{
			LineOrCircularArcWeldingLine tLastLine = vtSortData[vtSortData.size() - 1].tWeldLine;
			if (dGroupDisThreshold > TwoPointDis(tLastLine.EndPoint.x, tLastLine.EndPoint.y, tLastLine.EndPoint.z,
				vtTempSortData[i].tWeldLine.StartPoint.x, vtTempSortData[i].tWeldLine.StartPoint.y, vtTempSortData[i].tWeldLine.StartPoint.z))
			{
				vtSortData.push_back(vtTempSortData[i]);
				vtTempSortData.erase(vtTempSortData.begin() + i);
				break;
			}
		}
	}
	for (int i = 0; i < vtSortData.size(); i++)
	{
		vnFlatWeldNoIdx.push_back(vtSortData[i].nWeldNo);
	}
	return true;
}

bool WeldAfterMeasure::WaitAndCheckThreadExit(CWinThread* pWinThread, CString sThreadName)
{
	DWORD nExitCode = 1;
	while (0 != nExitCode)
	{
		BOOL bGetResult = GetExitCodeThread(pWinThread->m_hThread, &nExitCode);
		if ((FALSE == bGetResult) ||
			((0 != nExitCode) && (STILL_ACTIVE != nExitCode)))
		{
			WriteLog("�߳�[%s] ʧ���˳� ExitCode:%d", sThreadName, nExitCode);
			DELETE_POINTER(pWinThread);
			return false;
		}
		Sleep(50);
		DoEvent();
	}
	DELETE_POINTER(pWinThread);
	WriteLog("�߳�[%s]�˳� ExitCode:%d", sThreadName, nExitCode);
	return (0 == nExitCode) ? true : false;
}

UINT WeldAfterMeasure::ThreadTeachProcess(void* pParam)
{
	WeldAfterMeasure* pObj = (WeldAfterMeasure*)pParam;
	bool bTeachSuccess = pObj->TeachProcess();
	return (true == bTeachSuccess ? 0 : -1);
}

bool WeldAfterMeasure::TeachProcess(int nTimeOut)
{
	m_vtTeachResult.clear();
	int nCurWaitTime = 0; // ��λ ms
	int nWaitIntervalTime = 50; // ��λ ms

	CString sTeachOrgPath = OUTPUT_PATH + m_ptUnit->m_tContralUnit.strUnitName + TEACH_SRC_PATH;
	CString sTeachProPath = OUTPUT_PATH + m_ptUnit->m_tContralUnit.strUnitName + TEACH_PRO_PATH;
	CheckFolder(sTeachOrgPath);
	CheckFolder(sTeachProPath);
	CDHGigeImageCapture* pDHCamera = (CDHGigeImageCapture*)m_ptUnit->GetCameraCtrl(m_ptUnit->m_nMeasureCameraNo);
	T_CAMREA_PARAM tCameraParam = m_ptUnit->GetCameraParam(m_ptUnit->m_nMeasureCameraNo);
	IplImage* pBufferImg = cvCreateImage(cvSize(tCameraParam.tDHCameraDriverPara.nMaxWidth, tCameraParam.tDHCameraDriverPara.nMaxHeight), IPL_DEPTH_8U, 1);
	IplImage* pColorImg = cvCreateImage(cvSize(pBufferImg->width, pBufferImg->height), IPL_DEPTH_8U, 3);
	CString sIniFile = m_ptUnit->GetTrackTipDataFileName(m_ptUnit->m_nMeasureCameraNo);
	GROUP_STAND_DIP_NS::CImageProcess* pImgProcess = new GROUP_STAND_DIP_NS::CImageProcess(pBufferImg->width, pBufferImg->height, m_pRobotDriver->m_nRobotNo, sIniFile);

	vector<int> vnMeasurePtnType(m_vnMeasurePtnType); // ���� ������ �����в���������
	vector<T_ANGLE_PULSE> vtTeachPulse(m_vtTeachPulse);
	vector<T_ANGLE_PULSE>::iterator iterPulse = vtTeachPulse.begin();
	for (vector<int>::iterator iter = vnMeasurePtnType.begin();iter != vnMeasurePtnType.end();)
	{
		if (*iter & E_SEARCH_POINT)
		{
			iter = vnMeasurePtnType.erase(iter);
			iterPulse = vtTeachPulse.erase(iterPulse);
			continue;
		}
		iter++;
		iterPulse++;
	}
	int nTeachPtnTypeNum = vnMeasurePtnType.size();
	m_pRobotDriver->m_cLog->Write("����Ӧ�ò�ͼ������%d", nTeachPtnTypeNum);
	// �����ص���ͼ
	pDHCamera->ReadCapInfo(false);
	m_ptUnit->SwitchDHCamera(m_ptUnit->m_nMeasureCameraNo, true, true, E_ACQUISITION_MODE_SOURCE_HARD_0, E_CALL_BACK_MODE_SAVE_IMAGE);
	BOOL bStatus = pDHCamera->StartAcquisition();
	if (TRUE != bStatus)
	{
		XiMessageBox("ʾ�̿����ص���ͼʧ�ܣ�");
		delete pImgProcess;
		return false;
	}
	
	// ѭ���ȴ�����������ʾ��ͼ��
	int nProIdx = 0;
	int nCapNum = 0;
	int nErrorNumber = 0;
	while (true)
	{
		nCapNum = pDHCamera->m_vtCallBackImage.size();
		// �ȴ���ͼ���� ��ʱ�˳�
		if (nCapNum <= nProIdx)
		{
			Sleep(nWaitIntervalTime);
			nCurWaitTime += nWaitIntervalTime;
			if ((nCurWaitTime > nTimeOut) /*||(false == m_ptUnit->CheckInIO("Servo"))*/)
			{
				//���޸�
				XUI::MesBox::PopInfo("�ȴ���{0}��ʾ��ͼƬ��ʱ {1} {2}", nProIdx, nCurWaitTime, nTimeOut);
				//XiMessageBox("�ȴ���%d��ʾ��ͼƬ��ʱ %d %d", nProIdx, nCurWaitTime, nTimeOut);
				break;
			}
			continue;
		}

		// ��λ�ȴ�ʱ��
		nCurWaitTime = 0; 

		// ���� �� �ͷ� ��������е�ͼ��
		T_GET_CB_IMAGE* ptGetCBImage = pDHCamera->m_vtCallBackImage[nProIdx];
		cvCopyImage(ptGetCBImage->pImage, pBufferImg);
		cvReleaseImage(&(ptGetCBImage->pImage));

		// ��ԭͼ
		CString sSavePath;
		sSavePath.Format("%s\\Src_%d.jpg", sTeachOrgPath.GetBuffer(), nProIdx);
		SaveImage(pBufferImg, sSavePath);

		// ת��ɫ�ռ�(תRGB��ɫͼ)
		cvCvtColor(pBufferImg, pColorImg, CV_GRAY2RGB);

		// ͼ���� ���ݲ�ͬ�������� ����ͼ����õ���ά��
		T_TEACH_RESULT tTeachReault;

		if (false == ProcessTeachImage(nProIdx, pImgProcess, pBufferImg, vtTeachPulse[nProIdx],
			vnMeasurePtnType[nProIdx], tTeachReault, tCameraParam.eFlipMode))
		{
			nErrorNumber++;
			if (vnMeasurePtnType[nProIdx] & E_ADJUST_POINT)
			{
				tTeachReault.tKeyPtn3D.x = 999999.0;
				tTeachReault.tKeyPtn3D.y = 999999.0;
				tTeachReault.tKeyPtn3D.z = 999999.0;
				m_vtTeachResult.push_back(tTeachReault);
				WriteLog("ʾ�̵�%d��ͼ����ʧ�ܣ�", nProIdx); 
				nProIdx++;
				if (nProIdx >= nTeachPtnTypeNum)
				{
					break;
				}
				continue;
			}
			else
			{
				//���޸�
				XUI::MesBox::PopInfo("ʾ�̵�{0}��ͼ����ʧ�ܣ�", nProIdx);
				//XiMessageBox("ʾ�̵�%d��ͼ����ʧ�ܣ�", nProIdx);
				break;
			}
		}
		m_vtTeachResult.push_back(tTeachReault);

		// ������ά��
		for (int i = 0; i < tTeachReault.vtLeftPtns2D.size(); i++)
		{
			cvCircle(pColorImg, tTeachReault.vtLeftPtns2D[i], 6, CV_RGB(0, 255, 0), 2); // ����
		}
		for (int i = 0; i < tTeachReault.vtRightPtns2D.size(); i++)
		{
			cvCircle(pColorImg, tTeachReault.vtRightPtns2D[i], 6, CV_RGB(0, 0, 255), 2); // ����
		}
		cvCircle(pColorImg, tTeachReault.tKeyPtn2D, 6, CV_RGB(255, 0, 0), 2); // ����
		
		// �洦��ͼ
		sSavePath.Format("%s\\Pro_%d.jpg", sTeachProPath.GetBuffer(), nProIdx);
		SaveImage(pColorImg, sSavePath); 
		cvCopyImage(pColorImg, *m_pColorImg);
		// �ж��Ƿ���������ʾ��ͼƬ
		nProIdx++;
		if (nProIdx >= nTeachPtnTypeNum)
		{
			break;
		}
	}
	cvReleaseImage(&pColorImg);

	// ��ȡ�ɼ�֡�� ֹͣ�ɼ� �رռ�������
	nCapNum = pDHCamera->ReadCapInfo(false); // ����ڲ�ͳ��֡��
	pDHCamera->StopAcquisition();
	m_ptUnit->SwitchDHCamera(m_ptUnit->m_nMeasureCameraNo, false);

	if ((nCapNum != nTeachPtnTypeNum) || (nProIdx != nTeachPtnTypeNum))
	{
		//���޸�
		XUI::MesBox::PopInfo("����ͼ��:{0}ʵ��ͼ��:{1}����ͼ��:{2}ʾ��ʧ�ܣ�", nTeachPtnTypeNum, nCapNum, nProIdx);
		//XiMessageBox("����ͼ��:%d ʵ��ͼ��:%d ����ͼ��:%d ʾ��ʧ�ܣ�", nTeachPtnTypeNum, nCapNum, nProIdx);
		delete pImgProcess;
		return false;
	}

	if (nErrorNumber >= 2)
	{
		//���޸�
		XUI::MesBox::PopInfo("����ʧ��ͼ��{0},ʾ��ʧ�ܣ�", nErrorNumber);
		//XiMessageBox("����ʧ��ͼ��%d,ʾ��ʧ�ܣ�", nErrorNumber);
		delete pImgProcess;
		return false;
	}
	delete pImgProcess;
	if (NULL == pBufferImg)
	{
		cvReleaseImage(&pBufferImg);
		pBufferImg = NULL;
	}
	return true;
}

double WeldAfterMeasure::RecordRunTime(CRobotDriverAdaptor* pRobotCtrl, long ltimeBegin, CString str)
{
	long long dEnd;
	dEnd = XI_clock();
	double time = (double)(dEnd - ltimeBegin) / CLOCKS_PER_SEC;
	pRobotCtrl->m_cLog->Write("%s ʱ��ͳ�ƣ�%s ʱ��:%11.3lf", pRobotCtrl->m_strRobotName, str, time);
	return time;
}

bool WeldAfterMeasure::WeldTrackLock(CRobotDriverAdaptor* pRobotCtrl, T_ROBOT_COORS tStartP, T_ROBOT_COORS tEndP, bool bIsTrack)
{
//	if (!bIsTrack)
//	{
//		return true;
//	}
//	double dLen = TwoPointDis(tStartP.dX, tStartP.dY + tStartP.dBY, tStartP.dZ, tEndP.dX,
//		tEndP.dY + tEndP.dBY, tEndP.dZ);
//	double ddirX = (tEndP.dX - tStartP.dX) / dLen;
//	double ddirY = ((tEndP.dY + tEndP.dBY) - (tStartP.dY + tStartP.dBY)) / dLen;
//	double ddirZ = (tEndP.dZ - tStartP.dZ) / dLen;
//	double dNormal = atan2(ddirY, ddirX) * 180 / 3.1415926 - 90 * pRobotCtrl->m_nRobotInstallDir;
//
//	tStartP.dRZ = m_pRobotDriver->DirAngleToRz(dNormal);
//	RobotCoordPosOffset(tStartP, dNormal + 90 * pRobotCtrl->m_nRobotInstallDir, 50., 0. * pRobotCtrl->m_nRobotInstallDir);
//	RobotCoordPosOffset(tStartP, dNormal, 50., 50. * pRobotCtrl->m_nRobotInstallDir);
//	pRobotCtrl->MoveByJob(tStartP, m_pRobotDriver->m_tCoordLowSpeed, pRobotCtrl->m_nExternalAxleType, "MOVL");
//	if (!m_ptUnit->CheckRobotDone(tStartP))
//	{
//		XiMessageBox("δ�ƶ��İ�ȫλ��");
//		return false;
//	}
//	RobotCoordPosOffset(tStartP, dNormal, -50., -50. * pRobotCtrl->m_nRobotInstallDir);
//	pRobotCtrl->MoveByJob(tStartP, m_pRobotDriver->m_tCoordLowSpeed, pRobotCtrl->m_nExternalAxleType, "MOVL");
//	if (!m_ptUnit->CheckRobotDone(tStartP))
//	{
//		XiMessageBox("δ�ƶ��İ�ȫλ��");
//		return false;
//	}
//	m_pScanInit->RealTimeTrackLockArcBefor(pRobotCtrl);
//	return true;
//}
//
//bool WeldAfterMeasure::WeldTrackLock(CRobotDriverAdaptor* pRobotCtrl, vector<T_ROBOT_COORS> tCoors, bool bIsTrack)
//{
//	if (!bIsTrack)
//	{
//		return true;
//	}
//	if (tCoors.size() < 10)
//	{
//		XiMessageBox("��������");
//		return false;
//	}
//	std::vector<XI_POINT> vtPoint;
//	for (int i = 0; i < tCoors.size(); i++)
//	{
//		XI_POINT tp = { tCoors[i].dX,tCoors[i].dY + tCoors[i].dBY,tCoors[i].dZ };
//		vtPoint.push_back(tp);
//	}
//	T_LINE_PARA	tLine = m_pScanInit->CalcLineParamRansac(vtPoint, 0.7);
//	double dNormal = atan2(tLine.dDirY, tLine.dDirX) * 180 / 3.1415926 - 90 * pRobotCtrl->m_nRobotInstallDir;
//
//	T_ROBOT_COORS tStartP = tCoors.at(0);
//	if (pRobotCtrl->m_nOpenTrackingPos > 0)
//	{
//		tStartP = tCoors.at(30);
//	}
//	tStartP.dRZ = m_pRobotDriver->DirAngleToRz(dNormal);
//	RobotCoordPosOffset(tStartP, dNormal, 30., 30. * pRobotCtrl->m_nRobotInstallDir);
//	//T_PULSE_MOVE tMove(500, 20, 20);
//	pRobotCtrl->MoveByJob(tStartP, pRobotCtrl->m_tCoordLowSpeed, pRobotCtrl->m_nExternalAxleType, "MOVL");
//	if (!m_ptUnit->CheckRobotDone(tStartP))
//	{
//		XiMessageBox("δ�ƶ��İ�ȫλ��");
//		return false;
//	}
//	RobotCoordPosOffset(tStartP, dNormal, -30., -30. * pRobotCtrl->m_nRobotInstallDir);
//	pRobotCtrl->MoveByJob(tStartP, pRobotCtrl->m_tCoordLowSpeed, pRobotCtrl->m_nExternalAxleType, "MOVL");
//	if (!m_ptUnit->CheckRobotDone(tStartP))
//	{
//		XiMessageBox("δ�ƶ��İ�ȫλ��");
//		return false;
//	}
//	m_pScanInit->RealTimeTrackLockArcBefor(pRobotCtrl);
//	//�ƶ�����ʼλ��
//	if (pRobotCtrl->m_nOpenTrackingPos > 0)
//	{
//		RobotCoordPosOffset(tStartP, dNormal, 30., 30. * pRobotCtrl->m_nRobotInstallDir);
//		pRobotCtrl->MoveByJob(tStartP, pRobotCtrl->m_tCoordLowSpeed, pRobotCtrl->m_nExternalAxleType, "MOVL");
//		if (!m_ptUnit->CheckRobotDone(tStartP))
//		{
//			XiMessageBox("δ�ƶ��İ�ȫλ��");
//			return false;
//		}
//		pRobotCtrl->MoveByJob(tCoors.at(0), pRobotCtrl->m_tCoordLowSpeed, pRobotCtrl->m_nExternalAxleType, "MOVL");
//		if (!m_ptUnit->CheckRobotDone(tCoors.at(0)))
//		{
//			XiMessageBox("δ�ƶ��İ�ȫλ��");
//			return false;
//		}
//	}
	return true;
}

bool WeldAfterMeasure::IsWorking()
{
	return m_bWorking;
}

bool WAM::WeldAfterMeasure::FreeWaveGrooveWeld(vector<vector<T_ROBOT_COORS>> vvtGrooveWavePath, vector<T_WAVE_PARA> vtTWavePara, vector<vector<double>> vWeldSpeedRate)
{
	E_WELD_SEAM_TYPE eWeldSeamType = E_STAND_GROOVE;
	if ((fabs(vvtGrooveWavePath[0].front().dZ - vvtGrooveWavePath[0].back().dZ) < 100.0))
	{
		eWeldSeamType = E_PLAT_GROOVE;
	}

	vector<T_WELD_PARA> vtWeldPara;
	if (false == GetWeldParam(eWeldSeamType, 116, vtWeldPara))
	{
		XiMessageBox("���غ��ӹ��ղ���ʧ�ܣ�");
		return false;
	}
	for (int nLayerNo = 0; nLayerNo < vvtGrooveWavePath.size(); nLayerNo++)
	{
		T_WELD_PARA tWeldPara = vtWeldPara[nLayerNo];
		WriteLog("���ղ���%s����%d�� ƫ����%.3lf %.3lf",
			tWeldPara.strWorkPeace, tWeldPara.nLayerNo, tWeldPara.CrosswiseOffset, tWeldPara.verticalOffset);
		//���޸�
		if (MB_OKCANCEL != XUI::MesBox::PopOkCancel("�Ƿ񺸽ӵ�{0}�����죿", tWeldPara.nLayerNo + 1))
		//if (MB_OKCANCEL != XiMessageBox("�Ƿ񺸽ӵ�%d�����죿", tWeldPara.nLayerNo + 1))
		{
			continue;
		}
		vtTWavePara[nLayerNo].bStandWeld = true;

		if ((eWeldSeamType == E_PLAT_GROOVE) && (nLayerNo > 0))
		{
			tWeldPara.nWeldMethod = 1;
		}

		if (false == DoFreeGrooveWelding(vvtGrooveWavePath[nLayerNo], vtTWavePara[nLayerNo], tWeldPara, vWeldSpeedRate[nLayerNo]))
		{
			return false;
		}
	}
	return true;
}

bool WAM::WeldAfterMeasure::DoFreeGrooveWelding(std::vector<T_ROBOT_COORS>& vtWeldPathPoints, T_WAVE_PARA tWavePara, const T_WELD_PARA& tWeldPara, vector<double> weldSpeedRate)
{
	double disEndToStart = sqrt(pow((vtWeldPathPoints[0].dX - vtWeldPathPoints.back().dX), 2) +
		pow((vtWeldPathPoints[0].dY - vtWeldPathPoints.back().dY), 2) + pow((vtWeldPathPoints[0].dZ - vtWeldPathPoints.back().dZ), 2));
	double vecZ = (vtWeldPathPoints[vtWeldPathPoints.size() - 1].dZ - vtWeldPathPoints[0].dZ) / disEndToStart;
	T_ROBOT_COORS tmpRobotCoord;
	T_ROBOT_MOVE_SPEED tPulseMove = { 500,20,20 };
	double dCurExAxlePos = m_ptUnit->GetPositionDis();
	//���Ϊ�������޸ĺ�����̬
	/*if (vecZ > 0.9)
	{
		double disAngleVer = (m_dStandWeldRy - m_dNormalWeldRy) / 20;
		for (int i = 0; i < vtWeldPathPoints.size(); i++)
		{
			if (i < 20)
			{
				vtWeldPathPoints[i].dRY = m_dNormalWeldRy + disAngleVer * i;
			}
			else
			{
				vtWeldPathPoints[i].dRY = m_dStandWeldRy;
			}
			vtWeldPathPoints[i].dRZ = tWavePara.dVerticalAngle;
			vtWeldPathPoints[i].dRX = m_dStandWeldRx;
		}
	}*/

	//������ǹ��
	tmpRobotCoord = vtWeldPathPoints[0];
	tmpRobotCoord.dZ = tmpRobotCoord.dZ + 100 * m_pRobotDriver->m_nRobotInstallDir;
	//�ṩ�ο�ֵ
	double startSpeed = (tWavePara.dWaveDistance / 2) * tWeldPara.WeldVelocity / sqrt(pow(tWavePara.dWaveDistance / 2, 2) + pow(tWavePara.dStartWave, 2));
	double endSpeed = (tWavePara.dWaveDistance / 2) * tWeldPara.WeldVelocity / sqrt(pow(tWavePara.dWaveDistance / 2, 2) + pow(tWavePara.dEndWave, 2));
	//�ṩ�ο�ֵ
	if (abs(vecZ) > 0.5)
	{
		tmpRobotCoord.dX = tmpRobotCoord.dX + 100 * cos(PI * tmpRobotCoord.dRZ / 180);
		tmpRobotCoord.dY = tmpRobotCoord.dY + 100 * sin(PI * tmpRobotCoord.dRZ / 180);
		/*tmpRobotCoord.dRX = m_dNormalWeldRx;
		tmpRobotCoord.dRY = m_dNormalWeldRy;*/
		//if (1 == tWavePara.waveType)
		//{
		//	//�ṩ�ο�ֵ
		//	double startHypotenuse = sqrt(pow(tWavePara.dWaveDistance / 2, 2) + pow(tWavePara.dWaveHeight, 2) + pow(tWavePara.dStartWave / 2, 2));
		//	double startBase = sqrt(pow(tWavePara.dStartWave / 2, 2) + pow(tWavePara.dWaveDistance / 2, 2));
		//	double endHypotenuse = sqrt(pow(tWavePara.dWaveDistance / 2, 2) + pow(tWavePara.dWaveHeight, 2) + pow(tWavePara.dEndWave / 2, 2));
		//	double endBase = sqrt(pow(tWavePara.dEndWave / 2, 2) + pow(tWavePara.dWaveDistance / 2, 2));
		//	startSpeed = tWavePara.dWaveDistance * tWeldPara.WeldVelocity / (startHypotenuse + startBase);
		//	endSpeed = tWavePara.dWaveDistance * tWeldPara.WeldVelocity / (endHypotenuse + endBase);
		//}
	}
	XUI::MesBox::PopOkCancel("��ʼ�ٶ�Ϊ{0}mm/min,�յ��ٶ�Ϊ{1}mm/min", startSpeed, endSpeed);

	//��ǹ
	m_pRobotDriver->MoveByJob(tmpRobotCoord, tPulseMove, m_pRobotDriver->m_nExternalAxleType, "MOVL");
	m_ptUnit->WorldCheckRobotDone();
	m_pRobotDriver->MoveByJob(vtWeldPathPoints[0], tPulseMove, m_pRobotDriver->m_nExternalAxleType, "MOVL");
	m_ptUnit->WorldCheckRobotDone();
	//ִ�к���
	if (false == m_bIsLocalDebug) // ����
	{
		if (0 > m_ptUnit->GrooveWeldMove(vtWeldPathPoints, tWeldPara, dCurExAxlePos, TRUE==*m_pIsArcOn, tWavePara, weldSpeedRate))
		{
			return false;
		}
		m_ptUnit->WorldCheckRobotDone();
	}
	Sleep(2000);//�ӳ�����

	//������ǹ
	tmpRobotCoord = vtWeldPathPoints.back();
	tmpRobotCoord.dZ = tmpRobotCoord.dZ + 100 * m_pRobotDriver->m_nRobotInstallDir;
	if (abs(vecZ) > 0.5)
	{
		tmpRobotCoord.dX = tmpRobotCoord.dX + 100 * cos(PI * tmpRobotCoord.dRZ / 180);
		tmpRobotCoord.dY = tmpRobotCoord.dY + 100 * sin(PI * tmpRobotCoord.dRZ / 180);
		/*	tmpRobotCoord.dRX = m_dNormalWeldRx;
			tmpRobotCoord.dRY = m_dNormalWeldRy;*/
	}
	//��ǹ
	m_pRobotDriver->MoveByJob(tmpRobotCoord, tPulseMove, m_pRobotDriver->m_nExternalAxleType, "MOVL");
	m_ptUnit->WorldCheckRobotDone();
	return true;
}

void WeldAfterMeasure::SetTeachData(const std::vector<T_ANGLE_PULSE>& vtMeasurePulse, const vector<int>& vnMeasureType, double dExAxlePos, double& dExAxlePos_y)
{
	m_vnTeachIdxWithTemp.clear();
	m_vnMeasurePtnType.clear();
	m_vtTeachPulse.clear();
	m_dTeachExAxlePos = dExAxlePos;
	dExAxlePos_y = m_dTeachExAxlePos_y;
	for (int i = 0; i < vnMeasureType.size(); i++)
	{
		if ((vnMeasureType[i] & E_DOUBLE_LONG_LINE) || 
			(vnMeasureType[i] & E_LS_RL_FLIP) ||
			(vnMeasureType[i] & E_LL_RS_FLIP) ||
			(vnMeasureType[i] & E_L_LINE_POINT) ||
			(vnMeasureType[i] & E_R_LINE_POINT)) // ��ҪӲ����������ͼ�Ĳ����켣
		{
			m_vnMeasurePtnType.push_back(vnMeasureType[i]);
			m_vtTeachPulse.push_back(vtMeasurePulse[i]);
			m_vnTeachIdxWithTemp.push_back(i);
		}
	}
	m_nTeachPtnNum = m_vnMeasurePtnType.size();
}

bool WeldAfterMeasure::SpuriousTriggerTeach(int nGroupNo, const std::vector<T_ANGLE_PULSE>& vtMeasurePulse, const vector<int>& vnMeasureType, double dExAxlePos)
{

	if (vtMeasurePulse.size()<1)
	{
		return false;
	}
	//m_vtTeachResult.clear();
	//E_CAM_ID eCamId = E_LEFT_ROBOT_LEFT_CAM_H;
	//int nRobotId = eCamId / 4;
	//int nCamId = eCamId % 4 / 2;

	T_CAMREA_PARAM tCameraParam = m_ptUnit->GetCameraParam(m_ptUnit->m_nMeasureCameraNo);
	CDHGigeImageCapture* pDHCamera = (CDHGigeImageCapture*)m_ptUnit->GetCameraCtrl(m_ptUnit->m_nMeasureCameraNo);
	//IplImage* pBufferImg = pDHCamera->m_pImageBuff;
	IplImage* pBufferImg = cvCreateImage(cvSize(2448, 2048), IPL_DEPTH_8U, 1);
	IplImage* pColorImg = cvCreateImage(cvSize(pBufferImg->width, pBufferImg->height), IPL_DEPTH_8U, 3);
	CString sIniFile;
	sIniFile.Format("%s%s\\TrackDipParam_TrackCam.ini", DATA_PATH, m_ptUnit->m_tContralUnit.strUnitName, m_ptUnit->m_nMeasureCameraNo);
	GROUP_STAND_DIP_NS::CImageProcess* pImgProcess = new GROUP_STAND_DIP_NS::CImageProcess(pBufferImg->width, pBufferImg->height, m_pRobotDriver->m_nRobotNo, sIniFile);
	
//#ifndef SINGLE_ROBOT
	// �ⲿ���˶�
	int nAxisNo = m_ptUnit->m_nMeasureAxisNo;
	if (0 != m_ptUnit->MoveExAxisFun(dExAxlePos, 6000, nAxisNo))
	//if (0 != m_ptUnit->MoveExAxisForLineScan(dExAxlePos, m_pRobotDriver->m_tPulseHighSpeed.dSpeed,true))
	{
		cvReleaseImage(&pColorImg); 
		delete pImgProcess;
		return false;
	}
	m_ptUnit->WorldCheckRobotDone();
	double dCurExPos = 0.0;
	// ��Ҫ���Ļ�ȡ�����ݣ���ʱ����
	dCurExPos = m_ptUnit->GetExPositionDis(nAxisNo);//m_pRobotDriver->GetCurrentPos(ROBOT_AXIS_BPY);
	if (fabs(dExAxlePos - dCurExPos) > 5.0) // ��Ŀ��λ��������ֵ �ж�Ϊ�˶�ʧ��
	{
		XiMessageBox("�Զ�ʾ��:�ⲿ��δ�˶���ָ��λ��");
		delete pImgProcess;
		return false;
	}
//#endif // SINGLE_ROBOT
	m_pRobotDriver->MoveByJob(vtMeasurePulse[0], m_pRobotDriver->m_tPulseLowSpeed, m_pRobotDriver->m_nExternalAxleType, "MOVJ");

	E_DHGIGE_ACQUISITION_MODE eCameraMode = E_ACQUISITION_MODE_SOURCE_SOFTWARE;//E_ACQUISITION_MODE_CONTINUE;
	E_DHGIGE_CALL_BACK eCallBack = E_CALL_BACK_MODE_OFF;//E_CALL_BACK_MODE_WAIT_IMAGE;
	
	m_ptUnit->SwitchDHCamera(m_ptUnit->m_nMeasureCameraNo, true, true, eCameraMode, eCallBack); // ��ǹ������ ����� ����
	m_ptUnit->m_vpImageCapture[m_ptUnit->m_nMeasureCameraNo]->StartAcquisition();
	m_ptUnit->CheckRobotDone(vtMeasurePulse[0]);
	if (!m_pRobotDriver->ComparePulse(vtMeasurePulse[0], 100))
	{
		XiMessageBox("������δ���е�λ");
		delete pImgProcess;
		return false;
	}
	int nProIndex = 0;
	for (int n = 0;n<vtMeasurePulse.size();n++)//0 6 7 10 11 15
	{
		m_pRobotDriver->MoveByJob(vtMeasurePulse[n], m_pRobotDriver->m_tTeachSpeed, m_pRobotDriver->m_nExternalAxleType, "MOVJ");
		if (!m_ptUnit->CheckRobotDone(vtMeasurePulse[n]))
		{
			XiMessageBox("������δ���е�λ");
			cvReleaseImage(&pColorImg);
			delete pImgProcess;
			return false;
		}	
		if (E_TRANSITION_POINT & vnMeasureType[n])
		{
			continue;
		}		
		//cvCopyImage(pDHCamera->CaptureImage(false), pBufferImg);
		if (!pDHCamera->CaptureImage(pBufferImg, 1))
		{
			//pDHCamera->ShowErrorString();
		}

		// ��ԭͼ
		CString sSavePath;
		sSavePath.Format("%sPicture\\Org\\Src_%d.jpg", m_sDataSavePath.GetBuffer(), nProIndex);
		SaveImage(pBufferImg, sSavePath);
		
		// ת��ɫ�ռ�(תRGB��ɫͼ)
		cvCvtColor(pBufferImg, pColorImg, CV_GRAY2RGB);

		// ͼ���� ���ݲ�ͬ�������� ����ͼ����õ���ά��
		T_TEACH_RESULT tTeachReault;
		if (false == ProcessTeachImage(nProIndex, pImgProcess, pBufferImg, vtMeasurePulse[n], vnMeasureType[n], tTeachReault, tCameraParam.eFlipMode))
		{
			//���޸�
			XUI::MesBox::PopInfo("ʾ�̵�{0}��ͼ����ʧ�ܣ�n = {1} nProIdx = {2} {3} {4}");
			//XiMessageBox("ʾ�̵�%d��ͼ����ʧ�ܣ�n = %d nProIdx = %d %d %d", 
			//	nProIndex, n, nProIndex, vnMeasureType[n], m_vnMeasurePtnType[nProIndex]);
			break;
		}
		m_vtTeachResult.push_back(tTeachReault);

		// ������ά��
		for (int i = 0; i < tTeachReault.vtLeftPtns2D.size(); i++)
		{
			cvCircle(pColorImg, tTeachReault.vtLeftPtns2D[i], 6, CV_RGB(0, 255, 0), 2); // ����
		}
		for (int i = 0; i < tTeachReault.vtRightPtns2D.size(); i++)
		{
			cvCircle(pColorImg, tTeachReault.vtRightPtns2D[i], 6, CV_RGB(0, 0, 255), 2); // ����
		}
		cvCircle(pColorImg, tTeachReault.tKeyPtn2D, 6, CV_RGB(255, 0, 0), 2); // ����

																			  // �洦��ͼ
		sSavePath.Format("%sPicture\\Pro\\Pro_%d.jpg", m_sDataSavePath.GetBuffer(), nProIndex);
		SaveImage(pColorImg, sSavePath);
		cvCopyImage(pColorImg, *m_pColorImg);

		nProIndex++;
		//// �ж��Ƿ���������ʾ��ͼƬ
		//if (nProIndex >= m_nTeachPtnNum)
		//{
		//	break;
		//}	
	}
	m_ptUnit->SwitchDHCamera(m_ptUnit->m_nMeasureCameraNo, false);
	cvReleaseImage(&pColorImg);
	delete pImgProcess;
	
	return true;
}

bool WeldAfterMeasure::DoTeach(int nGroupNo, const std::vector<T_ANGLE_PULSE>& vtSrcMeasurePulse, const vector<int>& vnSrcMeasureType, double dExAxlePos, int nLayerNo/* = 0*/)
{
	m_nSmallGroupNoForTeach = -1;

	double dExAxlePos_y;
	m_vvtWeldSeamGroupAdjust;
	int nTrigSigTime = 15; //  x 0.01 x 1000 ms // I���� 15 * 0.01s = 0.15s = 150ms

	// ���ݼ��
	if (vtSrcMeasurePulse.size() < 3 || vtSrcMeasurePulse.size() != vnSrcMeasureType.size())
	{
		XiMessageBox("ʾ�����ݴ���");
		return false;
	}
	
	// ���ݼ�¼����Ա���������̺߳���ʹ�ã�
	SetTeachData(vtSrcMeasurePulse, vnSrcMeasureType, dExAxlePos, dExAxlePos_y);


	// ���������������ʾ�̲����� �������ֺ������˳��
	vector<T_ANGLE_PULSE> vtSearchPulse; // ���������� ����ͼ������� �ڲ��ǰ�켣�е�����
	vector<int> vnSearchMeasureType;
	vector<int> vnSearchIdx;
	vector<T_ANGLE_PULSE> vtMeasurePulse(vtSrcMeasurePulse); // ʾ�̵����� ����ͼ������� �ڲ��ǰ�켣�е�����
	T_ANGLE_PULSE tPulse = m_pRobotDriver->GetCurrentPulse();
	tPulse.nSPulse = vtMeasurePulse[0].nSPulse;
	vector<int> vnMeasureType(vnSrcMeasureType);
	vector<int> vnMeasureIdx;
	vtSearchPulse.clear();
	vnSearchMeasureType.clear();
	vnSearchIdx.clear();
	vnMeasureIdx.clear();
	for (int i = 0; i < vtSrcMeasurePulse.size(); i++)
	{
		vnMeasureIdx.push_back(i);
	}
	std::map<int, int> mnnIdx;
	mnnIdx.clear();
	// ��ȡɨ����������ɵ����� m_vnMeasurePtnTypeɨ������ �� m_vnTeachTrackOrder��ŵ���
	for (int i = 0; i < m_vnMeasurePtnType.size(); i++)
	{
		if (m_vnMeasurePtnType[i] & E_SEARCH_POINT)
		{
			mnnIdx[m_vnTeachTrackOrder[i]] = m_vnTeachIdxWithTemp[i];
		}
	}
	if (0 != mnnIdx.size() % 2)
	{
		XUI::MesBox::PopOkCancel("��������������{0}����!", mnnIdx.size());
		return false;
	}
	m_vnMeasurePtnType;   // �˴�������
	m_vnTeachTrackOrder;  // �˴�������
	m_vnTeachIdxWithTemp; // �˴�������
	std::map<int, int>::iterator iter;
	vector<int> vnDeleteIdx;
	vnDeleteIdx.clear();
	for (iter = mnnIdx.begin(); iter != mnnIdx.end(); iter++)
	{
		int nIdx = iter->second + 1;
		vtSearchPulse.push_back(vtMeasurePulse[nIdx]); // ���ɵ�
		vnSearchMeasureType.push_back(vnMeasureType[nIdx]);
		vnSearchIdx.push_back(nIdx);
		vnDeleteIdx.push_back(nIdx);

		nIdx = iter->second;
		vtSearchPulse.push_back(vtMeasurePulse[nIdx]);
		vnSearchMeasureType.push_back(vnMeasureType[nIdx]);
		vnSearchIdx.push_back(nIdx);
		vnDeleteIdx.push_back(nIdx);
		if (0 == vtSearchPulse.size() % 4) // ��֤ �켣˳��������ǹ->�������->�����յ�->������ǹ
		{
			int n1 = vtSearchPulse.size() - 1;
			int n2 = n1 - 1;
			swap(vtSearchPulse[n1], vtSearchPulse[n2]);
			swap(vnSearchMeasureType[n1], vnSearchMeasureType[n2]);
			swap(vnSearchIdx[n1], vnSearchIdx[n2]);
		}

	}
	sort(vnDeleteIdx.begin(), vnDeleteIdx.end(), [](const int& n1, const int& n2) -> bool {return n1 > n2; });
	for (int i = 0; i < vnDeleteIdx.size(); i++) // ����ɾ��
	{
		vtMeasurePulse.erase(vtMeasurePulse.begin() + vnDeleteIdx[i]);
		vnMeasureType.erase(vnMeasureType.begin() + vnDeleteIdx[i]);
		vnMeasureIdx.erase(vnMeasureIdx.begin() + vnDeleteIdx[i]);
	}
	// ����ʹ��
	vector<T_ANGLE_PULSE> vtPulse(vtMeasurePulse);
	vtPulse.insert(vtPulse.end(), vtSearchPulse.begin(), vtSearchPulse.end());
	CString sJobName;
	sJobName.Format("MOVJTEACHBOARD%d", nGroupNo);
	GenerateJobLocalVariable(vtPulse, MOVJ, sJobName);

	//���ƶ�������-y����

	if (-1 != m_ptUnit->m_nMeasureAxisNo_up)
	{
		if (0 != m_ptUnit->MoveExAxisFun(dExAxlePos_y, 9000, 2))return false;
		m_ptUnit->WorldCheckRobotDone();
		// ��Ҫ���Ļ�ȡ�����ݣ���ʱ����
		double dCurExPos = m_ptUnit->GetExPositionDis(2);// GetRobotCtrl()->GetCurrentPos(ROBOT_AXIS_BPY);
		if (fabs(dExAxlePos_y - dCurExPos) > 5.0) // ��Ŀ��λ��������ֵ �ж�Ϊ�˶�ʧ��
		{
			XiMessageBox("�Զ�ʾ��:�ⲿ��δ�˶���ָ��λ��");
			return false;
		}
	}

	//���������ȫ�ǹ��ȵ㣬����Ϊû�в�����
	bool bMeasure = false;
	for (size_t i = 0; i < vnMeasureType.size(); i++)
	{
		if (vnMeasureType[i] != E_TRANSITION_POINT)
		{
			bMeasure = true;
			break;
		}
	}

	m_pRobotDriver->MoveByJob(tPulse, m_pRobotDriver->m_tPulseHighSpeed, m_pRobotDriver->m_nExternalAxleType, "MOVJ");
	m_ptUnit->CheckRobotDone(tPulse);
	// ���Ͳ����켣���ݣ�ʾ�̵��� �켣���� �ٶ� �����ź�ʱ�䣩
	if (bMeasure && vtMeasurePulse.size() > 0)
	{
		m_nSmallGroupNoForTeach++;
		bool bTeachSuccess = false;
		if (1)//����
		{
			//ʾ�̲�ͼ
			bTeachSuccess = SpuriousTriggerTeach(nGroupNo, vtMeasurePulse, vnMeasureType, dExAxlePos);
		}
		else//Ӳ����
		{
			// ʾ���˶�
						// ʾ����ǹ������ ����� ����  ������ɺ�ر� ��ʡʱ��
			m_ptUnit->SwitchDHCamera(m_ptUnit->m_nMeasureCameraNo, true, true, E_ACQUISITION_MODE_SOURCE_HARD_0, E_CALL_BACK_MODE_SAVE_IMAGE);
			m_ptUnit->m_vpImageCapture[m_ptUnit->m_nMeasureCameraNo]->StartAcquisition();
			if (false == m_ptUnit->TeachMove(vtMeasurePulse, dExAxlePos, vnMeasureType, nTrigSigTime))
			{
				XiMessageBox("��ȷ�����˶�ʧ��!");
				return false;
			}
			// ����ͼ�����߳�
			CWinThread* pThread = AfxBeginThread(ThreadTeachProcess, this);
			pThread->m_bAutoDelete = false;
			// �ȴ������߳��˳�
			bTeachSuccess = WaitAndCheckThreadExit(pThread, "ʾ���߳�");
		}
		if (false == bTeachSuccess)
		{
			m_pRobotDriver->HoldOn();
			Sleep(500);
			m_pRobotDriver->HoldOff();
			return false;
		}
		if (!m_pRobotDriver->ComparePulse(vtMeasurePulse[vtMeasurePulse.size() - 1], m_pRobotDriver->GetCurrentPulse()))
		{
			m_ptUnit->RobotCheckDone(1400);
		}
		//CHECK_PULSE_RETURN_BOOL(m_pRobotDriver, vtMeasurePulse[vtMeasurePulse.size() - 1]);
		long long llTime = XI_clock();
		while (false == m_pRobotDriver->ComparePulse(vtMeasurePulse[vtMeasurePulse.size() - 1], m_pRobotDriver->GetCurrentPulse()))
		{ 
			Sleep(50);
			if (XI_clock() - llTime > 2000)
			{
				WriteLog("Check Pulse Err2");
				return false;
			}
		}
	}

	map<int, T_TEACH_RESULT> mtSearchResult;
	mtSearchResult.clear();
	if (vtSearchPulse.size() > 0)
	{
		int nEndpointNo = vtSearchPulse.size() / 4 - 1;
		int nSearchPointNum = 4;
		for (int i = vtSearchPulse.size(); i > 0; i -= nSearchPointNum)
		{
			m_nSmallGroupNoForTeach++;
			int nIndex = i - nSearchPointNum;
			vector<T_ANGLE_PULSE> vtPulse(vtSearchPulse.begin() + nIndex, vtSearchPulse.begin() + nIndex + nSearchPointNum);
			vector<int> vnType(vnSearchMeasureType.begin() + nIndex, vnSearchMeasureType.begin() + nIndex + nSearchPointNum);
			vector<T_TEACH_RESULT> vtTeachResult;
			// �ⲿ���˶�
			int nAxisNo = m_ptUnit->m_nMeasureAxisNo;
			int nUpAxisNo = m_ptUnit->m_nMeasureAxisNo_up;
			double dExPos = 0.0;
			double dExPos_y = 0.0;
#ifdef SINGLE_ROBOT
			dExPos = vtPulse[0].lBYPulse * m_pRobotDriver->m_tExternalAxle[1].dPulse;
#else
			dExPos = vtPulse[0].lBXPulse * m_pRobotDriver->m_tExternalAxle[0].dPulse;
			dExPos_y = vtPulse[0].lBYPulse * m_pRobotDriver->m_tExternalAxle[1].dPulse;
#endif // SINGLE_ROBOT
			double dCurExPos = m_ptUnit->GetExPositionDis(nAxisNo);
			if (fabs(dExPos - dCurExPos) > 5.0)
			{
				if (0 != m_ptUnit->MoveExAxisFun(dExPos, m_pRobotDriver->m_tPulseHighSpeed.dSpeed * 5, nAxisNo))
				{
					return false;
				}
				m_ptUnit->WorldCheckRobotDone();
				// ��Ҫ���Ļ�ȡ�����ݣ���ʱ����
				dCurExPos = m_ptUnit->GetExPositionDis(nAxisNo);
				if (fabs(dExPos - dCurExPos) > 5.0) // ��Ŀ��λ��������ֵ �ж�Ϊ�˶�ʧ��
				{
					XiMessageBox("�Զ�ʾ��:�ⲿ��δ�˶���ָ��λ��");
					return false;

				}
			}

			if (-1 != m_ptUnit->m_nMeasureAxisNo_up)
			{
				double dCurExPos_y = m_ptUnit->GetExPositionDis(nUpAxisNo);
				if (fabs(dExPos_y - dCurExPos_y) > 5.0)
				{
					if (0 != m_ptUnit->MoveExAxisFun(dExPos_y, m_pRobotDriver->m_tPulseHighSpeed.dSpeed * 5, nUpAxisNo))
					{
						return false;
					}
					m_ptUnit->WorldCheckRobotDone();
					// ��Ҫ���Ļ�ȡ�����ݣ���ʱ����
					dCurExPos_y = m_ptUnit->GetExPositionDis(nUpAxisNo);
					if (fabs(dExPos_y - dCurExPos_y) > 5.0) // ��Ŀ��λ��������ֵ �ж�Ϊ�˶�ʧ��
					{
						XiMessageBox("�Զ�ʾ��:�ⲿ��δ�˶���ָ��λ��");
						return false;

					}
				}
			}

			CHECK_BOOL_RETURN(ScanEndpoint(nGroupNo,m_ptUnit->m_nMeasureCameraNo, vtPulse, vnType, vtTeachResult));

			int nRstNo = 0;
			for (int n = nIndex; n < nIndex + nSearchPointNum; n++)
			{
				mtSearchResult[vnSearchIdx[n]] = vtTeachResult[nRstNo++];
			}
			// ��¼�������ݣ���������
			m_pScanInit->SaveEndpointData(m_pRobotDriver->m_strRobotName, nGroupNo, nEndpointNo, vtTeachResult[3].tKeyPtn3D, m_bWorkpieceShape);
			nEndpointNo--;
		}
	}
	// ʾ���˶�������浽 m_vtTeachResult ������������ ������������������
	vector<T_TEACH_RESULT> vtTeachResult(vnMeasureIdx.size());
	T_TEACH_RESULT tNullTeachResult;
	memset(&tNullTeachResult, 0, sizeof(tNullTeachResult));
	int nTranstionNum = 0;
	for (int i = 0; i < vtTeachResult.size(); i++)
	{
		if (E_TRANSITION_POINT & vnMeasureType[i])
		{
			nTranstionNum++;
			vtTeachResult[i] = tNullTeachResult;
			continue;
		}
		vtTeachResult[i] = m_vtTeachResult[i - nTranstionNum];
	}

	// ���ܽ��
	m_vtTeachResult.resize(vtSrcMeasurePulse.size(), tNullTeachResult);
	for (int i = 0; i < vnMeasureIdx.size(); i++) // ����ʾ�̲������
	{
		m_vtTeachResult[vnMeasureIdx[i]] = vtTeachResult[i]; 
	}
	map<int, T_TEACH_RESULT>::iterator iter2 = mtSearchResult.begin();
	for (; iter2 != mtSearchResult.end(); iter2++) // ���������������
	{
		m_vtTeachResult[iter2->first] = iter2->second;
	}

	for (int i = m_vtTeachResult.size() - 1; i >= 0; i--) // ɾ�����ɵ��������
	{
		if (vnSrcMeasureType[i] & E_TRANSITION_POINT)
		{
			m_vtTeachResult.erase(m_vtTeachResult.begin() + i);
		}
	}
	// ����ʾ�̽��(ʾ�̼��������)
	SaveTeachResult(nGroupNo);
	SaveTeachResultWorld(nGroupNo);

	m_vvtWeldSeamGroupAdjust;
	return true;
}

bool WeldAfterMeasure::Weld(int nGroupNo)
{
	// ���غ��ӹ켣����������
	double dCurrentWeldLen = 0.0;// ��ǰ���ӳ���
	CString strWeld;
	double dExAxlePos = 0.0;
	E_WELD_SEAM_TYPE eWeldSeamType = E_FLAT_SEAM;
	vector<T_ROBOT_COORS> vtAdjustRealWeldCoord;
	vector<T_ROBOT_COORS> vtRealWeldCoord;
	vector<T_ANGLE_PULSE> vtRealWeldPulse;
	vector<int> vnPtnType;
	int nFlatWeldNo = 0;
	int nStandWeldNo = 0;
	CleaeJobBuffer(); // �������������������Job �Ļ�����

	vector<LineOrCircularArcWeldingLine>& vtWeldSeam = m_vvtWeldSeamGroupAdjust[nGroupNo];
	vector<int> vnWeldOrder(0);
	vector<int> vnStandWeldNoIdx(0);
	vector<int> vnFlatWeldNoIdx(0);
	for (int i = 0; i < vtWeldSeam.size(); i++)
	{
		eWeldSeamType = GetWeldSeamType(vtWeldSeam[i]);
		if (E_FLAT_SEAM == eWeldSeamType)
		{
			vnFlatWeldNoIdx.push_back(i);
		}
		else if (E_STAND_SEAM == eWeldSeamType)
		{
			vnStandWeldNoIdx.push_back(i);
		}
	}
	vnWeldOrder.insert(vnWeldOrder.end(), vnStandWeldNoIdx.begin(), vnStandWeldNoIdx.end());
	vnWeldOrder.insert(vnWeldOrder.end(), vnFlatWeldNoIdx.begin(), vnFlatWeldNoIdx.end());

	for (int i = m_nPauseWeldNo; i < 8; i++)
	{
		if (i >= vnWeldOrder.size())
		{
			break;
		}
		int nWeldNo = vnWeldOrder[i];
		// ���ع켣
		if (false == LoadRealWeldTrack(nGroupNo, i, eWeldSeamType, dExAxlePos, vtRealWeldCoord, vnPtnType))
		{
			break;
		}		
		// ���ع��ղ���
		vector<T_WELD_PARA> vtWeldPara;
		//if (false == GetCurWeldParam(eWeldSeamType, vtWeldPara))
		int nWeldAngleSize = m_vvtWeldLineInfoGroup[nGroupNo][nWeldNo].tAtrribute.nWeldAngleSize;
		if (false == GetWeldParam(eWeldSeamType, nWeldAngleSize, vtWeldPara))
		{
			XiMessageBox("���غ��ӹ��ղ���ʧ�ܣ�");
			return false;
		}
		
		double dDirAngle = RzToDirAngle(vtRealWeldCoord[vtRealWeldCoord.size() / 2].dRZ); // ȡ�е�Rz���㷽���
		int n1 = dDirAngle / 45.0;
		int n2 = (fmod(dDirAngle, 45.0) / 45.0 > 0.5) ? 45 : 0;
		int nDirAngle = (n1 * 45 + n2) % 360;// ƽ������ǣ�360-rz��

		CString sWeldName;
		E_FLAT_SEAM == eWeldSeamType ? nFlatWeldNo++ : nStandWeldNo++;
		sWeldName.Format("%s%d", E_FLAT_SEAM == eWeldSeamType ? "ƽ��" : "����", E_FLAT_SEAM == eWeldSeamType ? nFlatWeldNo : nStandWeldNo);
		//���޸�
		CString str = E_FLAT_SEAM == eWeldSeamType ? "ƽ��" : "����";
		str = XUI::Languge::GetInstance().translate(str.GetBuffer());
		sWeldName = XUI::Languge::GetInstance().translate("{0}{1}", str.GetBuffer(), E_FLAT_SEAM == eWeldSeamType ? nFlatWeldNo : nStandWeldNo);

		WriteLog("��������������ţ�%d,����ţ�%d %s ����:%d ��������:%.3lf �յ������:%.3lf", nGroupNo, nWeldNo,
			E_FLAT_SEAM == eWeldSeamType ? "ƽ��" : "����",
			m_vvtWeldLineInfoGroup[nGroupNo][nWeldNo].tAtrribute.nWeldAngleSize,
			m_vvtWeldLineInfoGroup[nGroupNo][nWeldNo].tAtrribute.dStartHoleSize,
			m_vvtWeldLineInfoGroup[nGroupNo][nWeldNo].tAtrribute.dEndHoleSize);


		//���޸�
		if (TRUE == (*m_pIsNaturalPop) && IDOK != XUI::MesBox::PopOkCancel("����{0}���ӽǶ�{1}��", sWeldName.GetBuffer(), nDirAngle) /*XiMessageBox("���� %s ���ӽǶ�%d ��", sWeldName, nDirAngle)*/)

		//if (TRUE == (*m_pIsNaturalPop) && IDOK != XiMessageBox("���� %s ���ӽǶ�%d ��", sWeldName, nDirAngle))
		{
			continue;
		}

		// ���������(������������������)
		for (int nLayerNo = m_nPauseLayerNo; nLayerNo < vtWeldPara.size(); nLayerNo++)
		{
			T_WELD_PARA tWeldPara = vtWeldPara[nLayerNo];
			WriteLog("���ղ���%s����%d�� ƫ����%.3lf %.3lf",
				tWeldPara.strWorkPeace, tWeldPara.nLayerNo, tWeldPara.CrosswiseOffset, tWeldPara.verticalOffset);
			//���޸�
			if (TRUE == (*m_pIsNaturalPop) && IDOK != XUI::MesBox::PopOkCancel("{0}����{1}��{2}�� ���ӽǶ�{3}��", sWeldName.GetBuffer(), tWeldPara.nWeldAngleSize, tWeldPara.nLayerNo + 1, nDirAngle) /*XiMessageBox("%s ����%d ��%d�� ���ӽǶ�%d ��",
				sWeldName, tWeldPara.nWeldAngleSize, tWeldPara.nLayerNo + 1, nDirAngle*/)
			//if (TRUE == (*m_pIsNaturalPop) && IDOK != XiMessageBox("%s ����%d ��%d�� ���ӽǶ�%d ��",
				//sWeldName, tWeldPara.nWeldAngleSize, tWeldPara.nLayerNo + 1, nDirAngle))
			{
				continue;
			}
			TrackComp(nGroupNo, eWeldSeamType, vtRealWeldCoord, tWeldPara, vtAdjustRealWeldCoord);
			SaveWeldTrack(nGroupNo, i, tWeldPara.nLayerNo, eWeldSeamType, dExAxlePos, vtAdjustRealWeldCoord, vnPtnType);
			GetPauseWeldTrack(vtAdjustRealWeldCoord, vnPtnType, eWeldSeamType);
			bool bDoWeldSuccess = DoWelding(nGroupNo, nWeldNo, eWeldSeamType, vtAdjustRealWeldCoord, vnPtnType, tWeldPara);
			SavePauseInfo(nGroupNo, i, nLayerNo); // ��ͣ����ʹ��
			if (false == bDoWeldSuccess)
			{
				return false;
			}
		}
	}
	GenerateJobForContinueWeld(nGroupNo, true);
	CleaeJobBuffer();
	return true;
}

bool WeldAfterMeasure::DoWelding(int nGroupNo, int nWeldNo, E_WELD_SEAM_TYPE eWeldSeamType, std::vector<T_ROBOT_COORS>& vtWeldPathPoints,
	const vector<int>& vnPtnType, const T_WELD_PARA& tWeldPara)
{
	long long lTimeS = XI_clock();
	vector<T_ANGLE_PULSE> vtAccurateWeldPulse;
	vector<T_ROBOT_COORS> vtAccurateWeldCoord; // ���ӹ켣 + ��ǹ��ǹ����
	T_ANGLE_PULSE tDownGunPulse;
	T_ANGLE_PULSE tBackGunPulse;
	T_ROBOT_COORS tDownCoord; // ��ǹ����
	T_ROBOT_COORS tBackCoord; // ��ǹ����
	T_ROBOT_MOVE_INFO tRobotMoveInfo;
	vector<T_ROBOT_MOVE_INFO> vtRobotMoveInfo;

	CRobotDriverAdaptor* pRobotDriver = m_pRobotDriver;
	int nRobotNo = pRobotDriver->m_nRobotNo;
	int nTotalPointNum = vtWeldPathPoints.size();
	if (nTotalPointNum < 10)
	{
		XUI::MesBox::PopError("���ӹ켣���ݵ���{0} ���٣�����ʧ�ܣ�", nTotalPointNum);
		return false;
	}
	// ��ǹ
	double dMaxHeight = CalcBoardMaxHeight(nGroupNo);
	// ������ʱ���
	//dMaxHeight -= m_ptUnit->m_dExAxisZPos;
	//-----------------------------------------
	tDownCoord = vtWeldPathPoints[0];
	if (E_PLAT_GROOVE != eWeldSeamType)
	{
		if (E_STAND_SEAM == eWeldSeamType&&m_bIsSlope==true)
		{
			tDownCoord.dX += (1.2 / 1.2 * CosD(RzToDirAngle(tDownCoord.dRZ)));
			tDownCoord.dY += (1.2 / 1.2 * SinD(RzToDirAngle(tDownCoord.dRZ)));
			tDownCoord.dRY = 45.0; // ���ɵ� Ry 45 ��ȫ
		}
		else {
			tDownCoord.dX += (m_dGunDownBackSafeDis / 1.2 * CosD(RzToDirAngle(tDownCoord.dRZ)));
			tDownCoord.dY += (m_dGunDownBackSafeDis / 1.2 * SinD(RzToDirAngle(tDownCoord.dRZ)));
			tDownCoord.dRY = 45.0; // ���ɵ� Ry 45 ��ȫ
		}
	}
	tDownCoord.dZ = (dMaxHeight + m_dGunDownBackSafeDis * ((double)m_nRobotInstallDir));
	vtAccurateWeldCoord.push_back(tDownCoord);
	vtAccurateWeldCoord.insert(vtAccurateWeldCoord.end(), vtWeldPathPoints.begin(), vtWeldPathPoints.end());
	tBackCoord = vtWeldPathPoints[vtWeldPathPoints.size() - 1]; 
	if (E_PLAT_GROOVE != eWeldSeamType)
	{
		tBackCoord.dX += (m_dGunDownBackSafeDis / 1.2 * CosD(RzToDirAngle(tBackCoord.dRZ)));
		tBackCoord.dY += (m_dGunDownBackSafeDis / 1.2 * SinD(RzToDirAngle(tBackCoord.dRZ)));
		tBackCoord.dRY = 45.0; // ���ɵ� Ry 45 ��ȫ
	}
	tBackCoord.dZ = (dMaxHeight + m_dGunDownBackSafeDis * ((double)m_nRobotInstallDir));
	vtAccurateWeldCoord.push_back(tBackCoord);

	// ������ǹ�ؽ����꼰���������˶��ĺ��ӹ켣�ؽ�����켣
	double dRealWeldExPos = m_dTeachExAxlePos;
	vector<double> vdExAxleChangeDis(24);
	double dExAxleChangeDisThreshold = 1200;
	double dMinExAxleChangeDis = -dExAxleChangeDisThreshold;
	double dMaxExAxleChangeDis = dExAxleChangeDisThreshold;

	SaveCoordToFile(vtAccurateWeldCoord, "AAA_DoWelding.txt");
	if(false == CalcContinuePulseForWeld(vtAccurateWeldCoord, vtAccurateWeldPulse))
	{
		XiMessageBox("���ⲿ��λ�ã�������ǹ���ɵ����ʧ�ܣ�");
		return false;

		bool bCalcRst = false;
		bool bAddorSubtract = false;//true��false��
		int nIndex = 1;
		//double dExAxleChangeDis = dMinExAxleChangeDis;
		double dExAxleChangeDis = 0;
		while (dMinExAxleChangeDis <= dExAxleChangeDis && dExAxleChangeDis <= dMaxExAxleChangeDis) {
			dRealWeldExPos = m_dTeachExAxlePos;
			vector<T_ROBOT_COORS> vtTempWeldCoord(vtAccurateWeldCoord); // ���ӹ켣 + ��ǹ��ǹ����
			dRealWeldExPos += dExAxleChangeDis;

			double dMaxExAxlePos = (double)m_ptUnit->GetRobotCtrl()->m_tExternalAxle[m_ptUnit->m_nMeasureAxisNo - 1].lMaxPulseNum * m_ptUnit->GetRobotCtrl()->m_tExternalAxle[m_ptUnit->m_nMeasureAxisNo - 1].dPulse;
			double dMinExAxlePos = (double)m_ptUnit->GetRobotCtrl()->m_tExternalAxle[m_ptUnit->m_nMeasureAxisNo - 1].lMinPulseNum * m_ptUnit->GetRobotCtrl()->m_tExternalAxle[m_ptUnit->m_nMeasureAxisNo - 1].dPulse;
			if (dRealWeldExPos > dMaxExAxlePos || dRealWeldExPos < dMinExAxlePos)
			{
				WriteLog("�ⲿ��%d Ŀ��λ��%.3lf �����趨����%.3lf - %.3lf!", m_ptUnit->m_nMeasureAxisNo, dRealWeldExPos, dMinExAxlePos, dMaxExAxlePos);
				continue;
			}

			for (int nPtnNo = 0; nPtnNo < vtTempWeldCoord.size(); nPtnNo++)
			{
#ifdef SINGLE_ROBOT
				vtTempWeldCoord[nPtnNo].dY -= dExAxleChangeDis;
				vtTempWeldCoord[nPtnNo].dBY = dRealWeldExPos;
#else
				vtTempWeldCoord[nPtnNo].dX -= dExAxleChangeDis;
				vtTempWeldCoord[nPtnNo].dBX = dRealWeldExPos;
#endif // SINGLE_ROBOT
			}
			bCalcRst = CalcContinuePulseForWeld(vtTempWeldCoord, vtAccurateWeldPulse);
			if (true == bCalcRst)
			{
				vtWeldPathPoints.clear();
				vtWeldPathPoints.insert(vtWeldPathPoints.begin(), vtTempWeldCoord.begin() + 1, vtTempWeldCoord.end() - 1);
				tBackCoord = vtTempWeldCoord[vtTempWeldCoord.size() - 1];
				break;
			}
			dExAxleChangeDis = false == bAddorSubtract ? 0 * nIndex : 0 * nIndex;
			nIndex = true == bAddorSubtract ? nIndex + 1 : nIndex;
			bAddorSubtract = false == bAddorSubtract ? true : false;
		}
		if (false == bCalcRst)
		{
			XiMessageBox("���ⲿ��λ�ã�������ǹ���ɵ����ʧ�ܣ�");
			return false;
		}
	}
//#ifndef SINGLE_ROBOT
	// ����ⲿ���Ƿ���dRealWeldExPos
	// ������ھ��˶���dRealWeldExPos������Ƿ��˶���λ
	// �Ѿ�dRealWeldExPosֱ����������
	// 
	//���ƶ�������-y����
		double dExAxlePos_y = m_dTeachExAxlePos_y;
		if (-1 != m_ptUnit->m_nMeasureAxisNo_up)
		{
			if (0 != m_ptUnit->MoveExAxisFun(dExAxlePos_y, 9000, 2))return false;
			m_ptUnit->WorldCheckRobotDone();
			// ��Ҫ���Ļ�ȡ�����ݣ���ʱ����
			double dCurExPos = m_ptUnit->GetExPositionDis(2);// GetRobotCtrl()->GetCurrentPos(ROBOT_AXIS_BPY);
			if (fabs(dExAxlePos_y - dCurExPos) > 5.0) // ��Ŀ��λ��������ֵ �ж�Ϊ�˶�ʧ��
			{
				XiMessageBox("�Զ�ʾ��:�ⲿ��δ�˶���ָ��λ��");
				return false;
			}
		}
		
	int nAxisNo = m_ptUnit->m_nMeasureAxisNo;
	dRealWeldExPos = 1 == nAxisNo ? dRealWeldExPos : m_dTeachExAxlePos_y;
	double dCurExAxlePos = 0.0;
	dCurExAxlePos = m_ptUnit->GetExPositionDis(nAxisNo);
	if ((false == m_bIsLocalDebug) && (fabs(dCurExAxlePos - dRealWeldExPos) > 5.0)) // ��Ҫ�˶��ⲿ��
	{
		WriteLog("�ⲿ�Ὺʼ�˶���ָ��λ��");		
		if (0 != m_ptUnit->MoveExAxisFun(dRealWeldExPos, pRobotDriver->m_tPulseHighSpeed.dSpeed*5, nAxisNo))
		{
			return false;
		}
		m_ptUnit->WorldCheckRobotDone();
		double dCurExAxlePos = 0.0;
		dCurExAxlePos = m_ptUnit->GetExPositionDis(nAxisNo);
		if (fabs(dRealWeldExPos - dCurExAxlePos) > 5.0) // �ⲿ���˶�ʧ��
		{
			XiMessageBox("DoTeah:�ⲿ��δ�˶���ָ��λ��");
			return false;
		}
	}
//#endif // !SINGLE_ROBOT

	tDownGunPulse = vtAccurateWeldPulse[0];
	vtRobotMoveInfo.clear();
	T_ROBOT_COORS tDownGunPos = pRobotDriver->GetCurrentPos();
	pRobotDriver->RobotKinematics(tDownGunPulse, pRobotDriver->m_tTools.tGunTool, tDownGunPos);
	if (false == m_bIsLocalDebug) // �ӵ�ǰλ����ǹR��仯������Ҫ���Ӱ�ȫ���ɵ�
	{
		T_ANGLE_PULSE tCurAnglePulse = pRobotDriver->GetCurrentPulse();
		T_ROBOT_COORS tCurCoorsPos = pRobotDriver->GetCurrentPos();
		double dAngleErrAxisR = (double)(tDownGunPulse.nRPulse - tCurAnglePulse.nRPulse) * m_pRobotDriver->m_tAxisUnit.dRPulse;
		double length = TwoPointDis(tCurCoorsPos.dX + tCurCoorsPos.dBX, tCurCoorsPos.dY + tCurCoorsPos.dBY, tCurCoorsPos.dZ + tCurCoorsPos.dBZ, 
			tDownGunPos.dX + tCurCoorsPos.dBX, tDownGunPos.dY + tCurCoorsPos.dBY, tDownGunPos.dZ + tCurCoorsPos.dBZ);
		if (fabs(dAngleErrAxisR) > 90.0 || length > 500.0)
		{
			T_ANGLE_PULSE tSafeDownGunPulse(
				tCurAnglePulse.nSPulse,
				m_pRobotDriver->m_tHomePulse.nLPulse,
				m_pRobotDriver->m_tHomePulse.nUPulse,
				(tDownGunPulse.nRPulse + tCurAnglePulse.nRPulse) / 2,
				m_pRobotDriver->m_tHomePulse.nBPulse,
				(tDownGunPulse.nTPulse + tCurAnglePulse.nTPulse) / 2, 0, 0, 0);
			tRobotMoveInfo = m_pRobotDriver->PVarToRobotMoveInfo(0, tSafeDownGunPulse, m_pRobotDriver->m_tPulseHighSpeed, MOVJ);
			vtRobotMoveInfo.push_back(tRobotMoveInfo);
			vtAccurateWeldPulse.insert(vtAccurateWeldPulse.begin(), tSafeDownGunPulse);
		}
	}

	tRobotMoveInfo = m_pRobotDriver->PVarToRobotMoveInfo(0, tDownGunPulse, m_pRobotDriver->m_tPulseHighSpeed, MOVJ);
	vtRobotMoveInfo.push_back(tRobotMoveInfo);

	tRobotMoveInfo = m_pRobotDriver->PVarToRobotMoveInfo(1, vtWeldPathPoints[0], m_pRobotDriver->m_tCoordLowSpeed, MOVL);
	vtRobotMoveInfo.push_back(tRobotMoveInfo);
	
	if (false == m_bIsLocalDebug) // ��ǹ
	{
		// ��ǹ�˶� ��������
		m_pRobotDriver->SetMoveValue(vtRobotMoveInfo,false);
		m_pRobotDriver->CallJob("CONTIMOVANY");
		//m_pRobotDriver->ContiMoveAny(vtRobotMoveInfo, dRealWeldExPos);
		m_ptUnit->RobotCheckDone();
		if (false == m_pRobotDriver->CompareCoords(vtWeldPathPoints[0], m_pRobotDriver->GetCurrentPos())) 
		{
			WriteLog("Check Coors Err1"); 
			return false;
		}; // �����һ����ǹ�����й�
	}
	WriteLog("��ǹ���");
	if (false == m_bIsLocalDebug) // ����
	{
		if (0 > m_ptUnit->WeldMove(vtWeldPathPoints, vnPtnType, tWeldPara, dRealWeldExPos, eWeldSeamType, TRUE == *m_pIsArcOn))
		{
			return false;
		}
		m_ptUnit->RobotCheckDone();
		if (!m_ptUnit->m_ScanTrackingWeldEnable || eWeldSeamType == E_STAND_SEAM)
		{
			CHECK_COORS_RETURN_BOOL(m_pRobotDriver, vtWeldPathPoints[vtWeldPathPoints.size() - 1]);
		}
		else
		{
			CHECK_COORS_RETURN_BOOL(m_pRobotDriver, m_ptUnit->m_cWeldTrack.back());
		}
	}
	WriteLog("�������");

	vtRobotMoveInfo.clear();
	tRobotMoveInfo = m_pRobotDriver->PVarToRobotMoveInfo(0, tBackCoord, m_pRobotDriver->m_tCoordLowSpeed, MOVL);
	vtRobotMoveInfo.push_back(tRobotMoveInfo);
	if (false == m_bIsLocalDebug) // ��ǹ
	{
		m_pRobotDriver->SetMoveValue(vtRobotMoveInfo,false);
		m_pRobotDriver->CallJob("CONTIMOVANY");
		//m_pRobotDriver->ContiMoveAny(vtRobotMoveInfo, dRealWeldExPos);
		m_ptUnit->RobotCheckDone();
		CHECK_COORS_RETURN_BOOL(m_pRobotDriver, tBackCoord);
	}
	WriteLog("��ǹ���");
	long long lWeldTime = XI_clock() - lTimeS; //һ��������ʱ��
	double dWeldLenOld = (double)nTotalPointNum / 1000.0; // ��λ��
	LineOrCircularArcWeldingLine tWeld = m_vvtWeldSeamGroupAdjust[nGroupNo][nWeldNo];
	//һ�������ĳ���
	double dWeldLen = TwoPointDis(tWeld.StartPoint.x, tWeld.StartPoint.y, tWeld.StartPoint.z, tWeld.EndPoint.x, tWeld.EndPoint.y, tWeld.EndPoint.z) / 1000.0;
	dWeldLen= (double)nTotalPointNum / 1000.0;
	CTime time = CTime::GetCurrentTime();
	CString sProductDataFileName;
	sProductDataFileName.Format("Monitor\\%s\\ProductData_%4d.%.2d.%.2d.csv", m_pRobotDriver->m_strRobotName, time.GetYear(), time.GetMonth(), time.GetDay());
	FILE* RecordTheoryData;
	OpenProductLog(&RecordTheoryData, sProductDataFileName);
	SaveProductData(RecordTheoryData, dWeldLen, lWeldTime, nGroupNo);
	CloseProductLog(RecordTheoryData);
	//E_FLAT_SEAM == eWeldSeamType ? m_lFlatWeldTime += lWeldTime : m_lStandWeldTime += lWeldTime; // ����ʱ���¼
	//E_FLAT_SEAM == eWeldSeamType ? m_dFlatWeldLen += dWeldLen : m_dStandWeldLen += dWeldLen;
	
	//��¼����ʱ��ͳ���
	//CStatisticalData* pStatisticalData = CStatisticalData::getInstance();
	//pStatisticalData->UpdateWeldData(eWeldSeamType, dWeldLen, lWeldTime);
	//pStatisticalData->UpDateStatisticsData();
	WriteLog("���쳤��ͳ�ƣ�Old:%.3lf New:%.3lf", dWeldLenOld, dWeldLen);

	dWeldLen = vtWeldPathPoints.size();
	m_dCurWeldLenBeforeCleanGun += dWeldLen;
	//if (m_dCurWeldLenBeforeCleanGun > m_dCleanGunDis) // ����3�׽���һ����ǹ
	//{
	//	BackHome();
	//	CHECK_BOOL_RETURN(m_ptUnit->CheckIsReadyRun());
	//	m_pRobotDriver->CallJob("CLEAR");
	//	m_ptUnit->RobotCheckDone();
	//	BackHome();
	//	m_dCurWeldLenBeforeCleanGun = 0.0;
	//}

	// ���ɺ��ӹ켣JOB ������
	CString sJobName;
	sJobName.Format("DOWELDING%d-%d-%d", nGroupNo, nWeldNo, tWeldPara.nLayerNo);
	GenerateJobLocalVariable(vtAccurateWeldPulse, MOVL, sJobName);
	//GenerateJobCoord(vtAccurateWeldCoord, MOVL, sJobName);

	return true;
}

bool WeldAfterMeasure::ScanEndpoint(int nGroupNo, int nCameraNo, vector<T_ANGLE_PULSE> vtPulse, vector<int> vnType, vector<T_TEACH_RESULT>& vtTeachResult)
{
	vtTeachResult.clear();
	if (4 != vtPulse.size())
	{
		return false;
	}
	// �ж������ģʽ
	bool bScanMode = (E_SCAN_POINT & vnType[1]) == E_SCAN_POINT;
	E_SCANMODE eScanMode = E_SCANMODE_2DPOINT;
	if (E_SCAN_POINT & vnType[1])
	{
		eScanMode = E_SCANMODE_KINEMATIC;
	}
	else if (E_SEARCH_POINT_FIX & vnType[1])
	{
		eScanMode = E_SCANMODE_FIXEDSCAN;
	}

	bool bSinglePointMeasure = false;
	T_ROBOT_COORS tCoord;
	vector<T_ROBOT_COORS> vtCoord(0);
	T_ROBOT_MOVE_INFO tRobotMoveInfo;
	vector<T_ROBOT_MOVE_INFO> vtRobotMoveInfo(0);
	for (int i = 0; i < vtPulse.size(); i++)
	{
		m_pRobotDriver->RobotKinematics(vtPulse[i], m_pRobotDriver->m_tTools.tGunTool, tCoord);
		vtCoord.push_back(tCoord);
	}
	// ��ǹ�˶���ɨ�����λ�� MOVJ MOVJ
	tRobotMoveInfo = m_pRobotDriver->PVarToRobotMoveInfo(0, vtPulse[0], m_pRobotDriver->m_tPulseHighSpeed, MOVJ);
	vtRobotMoveInfo.push_back(tRobotMoveInfo);
	tRobotMoveInfo = m_pRobotDriver->PVarToRobotMoveInfo(1, vtPulse[1], m_pRobotDriver->m_tPulseLowSpeed, MOVJ);
	vtRobotMoveInfo.push_back(tRobotMoveInfo);
	m_pRobotDriver->SetMoveValue(vtRobotMoveInfo);
	m_pRobotDriver->CallJob("CONTIMOVANY");
	E_DHGIGE_ACQUISITION_MODE eCameraMode = E_ACQUISITION_MODE_SOURCE_SOFTWARE;
	E_DHGIGE_CALL_BACK eCallBack = E_CALL_BACK_MODE_OFF;
	m_ptUnit->SwitchDHCamera(m_ptUnit->m_nMeasureCameraNo, true, true, eCameraMode, eCallBack); // ��ǹʱ ����� ����
	m_ptUnit->m_vpImageCapture[m_ptUnit->m_nMeasureCameraNo]->StartAcquisition();
	m_ptUnit->RobotCheckDone();
	CHECK_PULSE_RETURN_BOOL(m_pRobotDriver, vtPulse[1]);

	double dDis = TwoPointDis(vtCoord[1].dX, vtCoord[1].dY, vtCoord[1].dZ, vtCoord[2].dX, vtCoord[2].dY, vtCoord[2].dZ);
	bSinglePointMeasure = dDis < 0.1;
	if (!bSinglePointMeasure) // �˶�ɨ�������˵�
	{
		double dScanSpeed = 500 / 6; // ����������MOVLʹ�����α�����Ϊ�ٶ���Ҫ����6����Ҫ�ĵ��ײ㣩
		if (bScanMode)
		{
			dScanSpeed = 400;
		}
		if (ROBOT_BRAND_ESTUN == m_pRobotDriver->m_eRobotBrand)
		{
			dScanSpeed = 280; // ��˹�ٻ�����MOVLʹ�����α�����Ϊ�ٶȲ�����6����Ҫ�ĵ��ײ㣩
			if (bScanMode)
			{
				dScanSpeed = 102;
			}
		}
		vtRobotMoveInfo.clear();
		T_ROBOT_MOVE_SPEED tScanSpeed(dScanSpeed / 6, 100, 100);
		if (ROBOT_BRAND_ESTUN == m_pRobotDriver->m_eRobotBrand)
		{
			tScanSpeed.dSpeed = dScanSpeed;
		}
		else
		{
			tScanSpeed.dSpeed = dScanSpeed/6;
		}
		tRobotMoveInfo = m_pRobotDriver->PVarToRobotMoveInfo(0, vtCoord[1], tScanSpeed, MOVL);
		vtRobotMoveInfo.push_back(tRobotMoveInfo);
		tRobotMoveInfo = m_pRobotDriver->PVarToRobotMoveInfo(1, vtCoord[2], tScanSpeed, MOVL);
		vtRobotMoveInfo.push_back(tRobotMoveInfo);

		// ������ɨ������յ���� Start
		double dWeldStartZ = m_vvtWeldLineInfoGroup[nGroupNo][0].tWeldLine.StartPoint.z;
		double dWeldEndZ = m_vvtWeldLineInfoGroup[nGroupNo][0].tWeldLine.EndPoint.z;
		if (bScanMode && fabs(dWeldEndZ - dWeldStartZ) > 120.0) // ɨ������յ�
		{
			double dCamGunToolErrZ = fabs(
				m_ptUnit->GetCameraParam(m_ptUnit->m_nMeasureCameraNo).tCameraTool.dZ - m_pRobotDriver->m_tTools.tGunTool.dZ);
			double dScanStartZ = dWeldStartZ + ((dCamGunToolErrZ + 20.0) * m_pRobotDriver->m_nRobotInstallDir);
			double dScanEndZ = dWeldEndZ + ((dCamGunToolErrZ - 50.0) * m_pRobotDriver->m_nRobotInstallDir);
			//XiMessageBoxOk("dScanStartZ %.3lf dScanEndZ %.3lf", dScanStartZ, dScanEndZ);
			vtRobotMoveInfo.clear();
			T_ROBOT_COORS tScanCoord = vtCoord[1];
			tScanCoord.dZ = dScanStartZ;
			//tScanCoord.dRX = m_dStandWeldScanRx;
			//tScanCoord.dRY = m_dStandWeldScanRy;
			//tScanCoord.dRX = 15.0;
			tRobotMoveInfo = m_pRobotDriver->PVarToRobotMoveInfo(0, tScanCoord, m_pRobotDriver->m_tCoordHighSpeed, MOVL);
			vtRobotMoveInfo.push_back(tRobotMoveInfo);

			tScanCoord = vtCoord[2];
			tScanCoord.dZ = dScanStartZ;
			//tScanCoord.dRX = m_dStandWeldScanRx;
			//tScanCoord.dRY = m_dStandWeldScanRy;
			//tScanCoord.dRX = 15.0;
			tRobotMoveInfo = m_pRobotDriver->PVarToRobotMoveInfo(1, tScanCoord, tScanSpeed, MOVL);
			vtRobotMoveInfo.push_back(tRobotMoveInfo);

			tScanCoord = vtCoord[1];
			tScanCoord.dZ = dScanEndZ;
			//tScanCoord.dRX = m_dStandWeldScanRx;
			//tScanCoord.dRY = m_dStandWeldScanRy;
			//tScanCoord.dRX = 15.0;
			tRobotMoveInfo = m_pRobotDriver->PVarToRobotMoveInfo(2, tScanCoord, m_pRobotDriver->m_tCoordHighSpeed, MOVL);
			vtRobotMoveInfo.push_back(tRobotMoveInfo);

			tScanCoord = vtCoord[2];
			tScanCoord.dZ = dScanEndZ;
			//tScanCoord.dRX = m_dStandWeldScanRx;
			//tScanCoord.dRY = m_dStandWeldScanRy;
			//tScanCoord.dRX = 15.0;
			tRobotMoveInfo = m_pRobotDriver->PVarToRobotMoveInfo(3, tScanCoord, tScanSpeed, MOVL);
			vtRobotMoveInfo.push_back(tRobotMoveInfo);
		}
		// ������ɨ������յ���� End

		m_pRobotDriver->SetMoveValue(vtRobotMoveInfo, false);

		E_FLIP_MODE eFlipMode = m_ptUnit->GetCameraParam(m_ptUnit->m_nMeasureCameraNo).eFlipMode;
		m_pScanInit->SetScanParam(TRUE == *m_pIsNaturalPop, eFlipMode, eScanMode, "CONTIMOVANY");

		CTime cTime;
		cTime = CTime::GetCurrentTime();

		int nCurrentHour = cTime.GetHour();
		int nCurrentMinute = cTime.GetMinute();
		int nCurrentSecond = cTime.GetSecond();
		CString strFile;
		strFile.Format("%s%s%s\\%d-%d-%d-AA_PointCloud.txt", OUTPUT_PATH, m_ptUnit->GetRobotCtrl()->m_strRobotName, WELDDATA_PATH, nCurrentHour, nCurrentMinute, nCurrentSecond);
		E_POINT_CLOUD_PROC_MOMENT eProcMoment = E_POINT_CLOUD_PROC_MOMENT_DYNAMIC;//jwq 20250416
		E_IMAGE_PROC_METHOD eImgProcMethod = E_IMAGE_PROC_METHOD_EEEEEInTrack;
		E_POINT_CLOUD_PROC_METHOD ePointCloudProcMethod = E_POINT_CLOUD_PROC_METHOD_Groove;
		if (bScanMode)
		{
			eProcMoment = E_POINT_CLOUD_PROC_MOMENT_FINAL;
			eImgProcMethod = E_IMAGE_PROC_METHOD_EEEEE;
		}

		auto tDeltaCoord = vtCoord[2] - vtCoord[1];
		CvPoint3D64f tDeltaDis{ tDeltaCoord.dX + tDeltaCoord.dBX, tDeltaCoord.dY + tDeltaCoord.dBY, tDeltaCoord.dZ + tDeltaCoord.dBZ };
		if (!m_pScanInit->InitDynamicCapture_H_M(
			eProcMoment, eImgProcMethod, ePointCloudProcMethod, m_ptUnit->m_nMeasureCameraNo,
			OUTPUT_PATH + m_pRobotDriver->m_strRobotName + WELDDATA_PATH, "AA_PointCloud.txt", 80, 
			IsStandWeldSeam(tDeltaDis) ? E_STAND_SEAM : E_FLAT_SEAM, 0, 0, true, true))
		{
			return false;
		}
		m_pScanInit->InitRealTimeTracking(E_JUDGE_END_PROCESS_SEARCH);
		//ƽ��������Ҫ��������������Ҫ
		if (eImgProcMethod == E_IMAGE_PROC_METHOD_EEEEEInTrack
			&& !m_pScanInit->lockLaser(m_pRobotDriver, m_ptUnit->m_nMeasureCameraNo))
		{
			return false;
		}
		m_pScanInit->DynamicCaptureNew(m_pRobotDriver, m_ptUnit->m_nMeasureCameraNo, strFile);
	}
	else // �����˶�����
	{
		if (!m_pScanInit->SinglePointMeasure(m_pRobotDriver))
		{
			return false;
		};
	}

	// ��ǹ�˶� MOVL
	vtRobotMoveInfo.clear();
	tRobotMoveInfo = m_pRobotDriver->PVarToRobotMoveInfo(0, vtPulse[0], m_pRobotDriver->m_tPulseLowSpeed, MOVJ);
	vtRobotMoveInfo.push_back(tRobotMoveInfo);
	m_pRobotDriver->SetMoveValue(vtRobotMoveInfo,false);
	m_pRobotDriver->CallJob("CONTIMOVANY");
	m_ptUnit->SwitchDHCamera(m_ptUnit->m_nMeasureCameraNo, false); // ��ǹʱ�ر�����ͼ���
	m_ptUnit->RobotCheckDone();
	CHECK_PULSE_RETURN_BOOL(m_pRobotDriver, vtPulse[0]);

	if (INCISEHEAD_THREAD_STATUS_STOPPED == m_pRobotDriver->m_eThreadStatus)
	{
		return false;
	}
	if (bSinglePointMeasure || bScanMode)
	{
		CHECK_BOOL_RETURN(GetScanProcResult(nGroupNo, bScanMode, bSinglePointMeasure, vtCoord, vtTeachResult));
	}
	else
	{
		if (m_pScanInit->m_pTraceModel->vtPointCloudEndPoints.size() <= 0)
		{
			XiMessageBoxOk("�˵�ɨ��ʧ�ܣ�");
			return false;
		}
		double dExPos = m_ptUnit->GetExPositionDis(m_ptUnit->m_nMeasureAxisNo);
		double dExPos_y = 0/*m_ptUnit->GetExPositionDis(m_ptUnit->m_nMeasureAxisNo_up)*/;
//		for (int nPt = 0;nPt<m_pScanInit->m_pTraceModel->vtPointCloudEndPoints.size();nPt++)
//		{
//#ifdef SINGLE_ROBOT
//			m_pScanInit->m_pTraceModel->vtPointCloudEndPoints[nPt].y -= dExPos;
//#else
//			m_pScanInit->m_pTraceModel->vtPointCloudEndPoints[nPt].x -= dExPos;
//			m_pScanInit->m_pTraceModel->vtPointCloudEndPoints[nPt].y -= dExPos_y;
//#endif // SINGLE_ROBOT
//		}
		T_TEACH_RESULT tTeachResult;
		tTeachResult.tKeyPtn3D.x = m_pScanInit->m_pTraceModel->vtPointCloudEndPoints.back().x;
		tTeachResult.tKeyPtn3D.y = m_pScanInit->m_pTraceModel->vtPointCloudEndPoints.back().y;
		tTeachResult.tKeyPtn3D.z = m_pScanInit->m_pTraceModel->vtPointCloudEndPoints.back().z;
		vtTeachResult.push_back(tTeachResult);
		vtTeachResult.push_back(tTeachResult);
		tTeachResult.tKeyPtn3D.x = m_pScanInit->m_pTraceModel->vtPointCloudEndPoints.back().x;
		tTeachResult.tKeyPtn3D.y = m_pScanInit->m_pTraceModel->vtPointCloudEndPoints.back().y;
		tTeachResult.tKeyPtn3D.z = m_pScanInit->m_pTraceModel->vtPointCloudEndPoints.back().z;
		vtTeachResult.push_back(tTeachResult);
		vtTeachResult.push_back(tTeachResult);
	}

	// ȥ���ⲿ�ᣬ����֮ǰ����һ��,��ʱ
	double dExPos = m_ptUnit->GetExPositionDis(m_ptUnit->m_nMeasureAxisNo);
	double dExPos_y = 0/*m_ptUnit->GetExPositionDis(m_ptUnit->m_nMeasureAxisNo_up)*/;
	for (int i = 0; i < vtTeachResult.size(); i++)
	{
#ifdef SINGLE_ROBOT
		vtTeachResult[i].tKeyPtn3D.y -= dExPos;
#else
		vtTeachResult[i].tKeyPtn3D.x -= dExPos;
		vtTeachResult[i].tKeyPtn3D.y -= dExPos_y;
#endif // SINGLE_ROBOT
	}
	return true;
}

bool WeldAfterMeasure::CheckContinueMove_Offset(int nGroupNo, /*double dCarPos,*/ double dExAxisPos, double yPos, double offSet, double ry, double rx)
{
	CleaeJobBuffer(); // �������������������Job �Ļ�����
	double dFileExAxlePos = 0.0;
	E_WELD_SEAM_TYPE eWeldSeamType = E_FLAT_SEAM;
	vector<T_ROBOT_COORS> vtRealWeldCoord;
	vector<T_ANGLE_PULSE> vtRealWeldPulse;
	vector<int> vnPtnType;

	vector<LineOrCircularArcWeldingLine>& vtWeldSeam = m_vvtWeldSeamGroupAdjust[nGroupNo];
	vector<int> vnWeldOrder(0);
	vector<int> vnStandWeldNoIdx(0);
	vector<int> vnFlatWeldNoIdx(0);
	for (int i = 0; i < vtWeldSeam.size(); i++) {
		eWeldSeamType = GetWeldSeamType(vtWeldSeam[i]);
		if (E_FLAT_SEAM == eWeldSeamType) {
			vnFlatWeldNoIdx.push_back(i);
		}
		else if (E_STAND_SEAM == eWeldSeamType) {
			vnStandWeldNoIdx.push_back(i);
		}
	}
	vnWeldOrder.insert(vnWeldOrder.end(), vnStandWeldNoIdx.begin(), vnStandWeldNoIdx.end());
	vnWeldOrder.insert(vnWeldOrder.end(), vnFlatWeldNoIdx.begin(), vnFlatWeldNoIdx.end());

	bool bIsCollide = false;
	for (int i = 0; i < 8; i++) {
		if (i >= vnWeldOrder.size()) break;
		int nWeldNo = vnWeldOrder[i];
		// ���ع켣
		if (false == LoadRealWeldTrack(nGroupNo, i, eWeldSeamType, dFileExAxlePos, vtRealWeldCoord, vnPtnType)) {
			break;
		}

		vector<T_ROBOT_COORS> vtAdjustRealWeldCoord(vtRealWeldCoord);

		for (int ptnNo = 0; ptnNo < vtAdjustRealWeldCoord.size(); ptnNo++)
		{
			vtAdjustRealWeldCoord[ptnNo].dX += (offSet * CosD(m_pRobotDriver->RzToDirAngle(vtAdjustRealWeldCoord[ptnNo].dRZ)));
			vtAdjustRealWeldCoord[ptnNo].dY += (offSet * SinD(m_pRobotDriver->RzToDirAngle(vtAdjustRealWeldCoord[ptnNo].dRZ)));
			vtAdjustRealWeldCoord.at(ptnNo).dBX = rx;
			vtAdjustRealWeldCoord.at(ptnNo).dBY = ry;
		}

		T_ROBOT_COORS tDownCoord; // ��ǹ����
		T_ROBOT_COORS tBackCoord; // ��ǹ����		
		double dMaxHeight = CalcBoardMaxHeight(nGroupNo);
		tDownCoord = vtAdjustRealWeldCoord[0];
		if (E_PLAT_GROOVE != eWeldSeamType) {
			tDownCoord.dX += (m_dGunDownBackSafeDis / 1.2 * CosD(m_pRobotDriver->RzToDirAngle(tDownCoord.dRZ)));
			tDownCoord.dY += (m_dGunDownBackSafeDis / 1.2 * SinD(m_pRobotDriver->RzToDirAngle(tDownCoord.dRZ)));
			tDownCoord.dRY = m_dPlatWeldRy; // ���ɵ� Ry 45 ��ȫ
		}
		tDownCoord.dZ = (dMaxHeight + (m_dGunDownBackSafeDis * 2.0) * ((double)m_nRobotInstallDir));
		vtAdjustRealWeldCoord.insert(vtAdjustRealWeldCoord.begin(), tDownCoord);// ��ǹ

		// !!!!!! ��������ǹ���ɵ����ͺ�������ǹ���ɵ�һ�£������ⲿ��λ�ÿ���ͣ��λ�ò�ͬ
		tBackCoord = vtAdjustRealWeldCoord[vtAdjustRealWeldCoord.size() - 1];
		tBackCoord.dX += (m_dGunDownBackSafeDis / 1.2 * CosD(m_pRobotDriver->RzToDirAngle(tBackCoord.dRZ)));
		tBackCoord.dY += (m_dGunDownBackSafeDis / 1.2 * SinD(m_pRobotDriver->RzToDirAngle(tBackCoord.dRZ)));
		tBackCoord.dRY = m_dPlatWeldRy; // ���ɵ� Ry 45 ��ȫ
		tBackCoord.dZ = (dMaxHeight + (m_dGunDownBackSafeDis * 2.0) * ((double)m_nRobotInstallDir));
		vtAdjustRealWeldCoord.push_back(tBackCoord);// ��ǹ

		SaveCoordToFile(vtAdjustRealWeldCoord, "AAA_CheckContinueMove.txt");
		CHECK_BOOL_RETURN(CalcContinuePulseForMeasure(vtAdjustRealWeldCoord, vtRealWeldPulse, true));
		if (false == CheckFlangeToolCoord(vtRealWeldPulse))
		{
			m_pRobotDriver->m_cLog->Write("���ۺ��ӹ켣�����˶���飺�������ľ�������ԭ�����");
			return false;
		}
		//GenerateFilePLYPlane(dExAxisPos);
		//SetCheckCollidePlane();
		//bIsCollide |= CheckIsCollide(vtAdjustRealWeldCoord, dExAxisPos, true);

		SaveToJobBuffer(vtRealWeldPulse);
	}
	GenerateJobForContinueWeld(nGroupNo);
	CleaeJobBuffer();
	if (bIsCollide)
	{
		XiMessageBoxOk("���ۺ��ӹ켣���棬�޷�����");
		return false;
	}
	return true;
}

bool WeldAfterMeasure::GetScanProcResult(int nGroupNo, bool bIsScanMode, bool bSinglePointMeasure, vector<T_ROBOT_COORS> vtCoord, vector<T_TEACH_RESULT>& vtTeachResult)
{
	vtTeachResult.clear();
	T_TEACH_RESULT tTeachResult;
	memset(&tTeachResult, 0, sizeof(tTeachResult));
	if (bSinglePointMeasure)
	{
		tTeachResult.tKeyPtn3D = m_pScanInit->GetDynamicScanEndPoint();	// ��ȡ�˵�
		if (!bSinglePointMeasure) // �����˵㲹�� ��������޲���
		{
			double dDis = TwoPointDis(vtCoord[1].dX, vtCoord[1].dY, vtCoord[1].dZ,	// �˵��������� ����ڼ�
				vtCoord[2].dX, vtCoord[2].dY, vtCoord[2].dZ);
			double dDisX = vtCoord[2].dX - vtCoord[1].dX;
			double dDisY = vtCoord[2].dY - vtCoord[1].dY;
			double dDisZ = vtCoord[2].dZ - vtCoord[1].dZ;
			double dEndPointComp = m_dScanEndpointOffset;
			tTeachResult.tKeyPtn3D.x += (dDisX * dEndPointComp / dDis);
			tTeachResult.tKeyPtn3D.y += (dDisY * dEndPointComp / dDis);
			tTeachResult.tKeyPtn3D.z += (dDisZ * dEndPointComp / dDis);
		}
		vtTeachResult.resize(vtCoord.size(), tTeachResult);
	}
	else
	{
		CString sPointCloudFile;
		CString sRecoResultFile;
		CString sSaveResultFile;
		CString sInputFile;
		sPointCloudFile = m_pScanInit->GetPointCloudFileName();
		int nIdx = sPointCloudFile.ReverseFind('.');
		CString s1 = sPointCloudFile.Left(nIdx);
		sRecoResultFile.Format("%s_Rst.txt", (s1.GetBuffer()));
		sSaveResultFile.Format("%s_Save.txt", (s1.GetBuffer()));
		sInputFile.Format("%s_Input.txt", (s1.GetBuffer()));

		RiserEndPointInfo tWeldInfo[3] = { 0 };
		CvPoint3D32f tRefPtn[2] = { 0.0 };
		T_ROBOT_COORS tPtnS;
		T_ROBOT_COORS tPtnE;
		T_ROBOT_COORS tCamTool;
		m_ptUnit->m_nMeasureCameraNo;
		tCamTool = m_ptUnit->GetCameraParam(m_ptUnit->m_nMeasureCameraNo).tCameraTool;
		bool bRst = true;

		//auto tDeltaCoord = vtCoord[2] - vtCoord[1];
		//CvPoint3D64f tDeltaDis{ tDeltaCoord.dX + tDeltaCoord.dBX, tDeltaCoord.dY + tDeltaCoord.dBY, tDeltaCoord.dZ + tDeltaCoord.dBZ };
		double dDirAngle = RzToDirAngle(vtCoord[1].dRZ);
		double dDirAngle2 = RzToDirAngle(vtCoord[2].dRZ);
		dDirAngle = TwoDirAngleMedian(dDirAngle, dDirAngle2);

		if (bIsScanMode)
		{
			XiAlgorithm alg;
			LineOrCircularArcWeldingLine tSeam = m_vvtWeldLineInfoGroup[nGroupNo][0].tWeldLine;
			double dNorAngle = alg.CalcArcAngle(tSeam.StartNormalVector.x, tSeam.StartNormalVector.y);
			dDirAngle = m_pRobotDriver->DirAngleToRz(dNorAngle) + m_dStandWeldScanOffsetRz;
		}

		E_WELD_SEAM_TYPE eWeldSeamType = bIsScanMode ? E_STAND_SEAM : E_FLAT_SEAM;

		CvPoint3D32f tCameraNorm = cvPoint3D32f(-CosD(dDirAngle), -SinD(dDirAngle), m_nRobotInstallDir > 0.0 ? -1.0 : 1.0);
		bool bFound = false;
		int nRealSmallGroupNoForTeach = m_nSmallGroupNoForTeach;
		if (bIsScanMode)//abs(vtCoord[1].dRZ - vtCoord[2].dRZ) > 45) 
		{
			//ͨ�������켣���㺸������λ�ûᵼ�´�����ϣ��޷��޸Ĳ����켣
			//bRst &= m_pRobotDriver->MoveToolByWeldGun( vtCoord[0], tCamTool, vtCoord[0], m_pRobotDriver->m_tTools.tGunTool, tPtnS);
			//bRst &= m_pRobotDriver->MoveToolByWeldGun( vtCoord[1], tCamTool, vtCoord[1], m_pRobotDriver->m_tTools.tGunTool, tPtnE);
			
			//�����켣��ǰ�����򣬵���ʵ��˳�����غ���˳�����ﴦ��һ��
			//bRst &= m_pRobotDriver->MoveToolByWeldGun(vtCoord[1], tCamTool, vtCoord[1], m_pRobotDriver->m_tTools.tGunTool, tPtnS);
			//for (size_t i = 0; i < m_vtTeachData.size(); i++)
			//{
			//	double dDis = square(m_vtTeachData[i].tMeasureCoordGunTool.dX - tPtnS.dX)
			//		+ square(m_vtTeachData[i].tMeasureCoordGunTool.dY - tPtnS.dY)
			//		+ square(m_vtTeachData[i].tMeasureCoordGunTool.dZ - tPtnS.dZ);
			//	if (dDis < 1.0)
			//	{
			//		nRealSmallGroupNoForTeach = m_vtTeachData[i].nWeldNo;
			//	}
			//}
			//double dMinDis = 99999.9;
			//for (size_t i = 0; i < m_vvtWeldLineInfoGroup[m_nGroupNo].size(); i++)
			//{
			//	if (!IsStandWeldSeam(m_vvtWeldLineInfoGroup[m_nGroupNo][i].tWeldLine.StartNormalVector))
			//	{
			//		continue;
			//	}
			//	auto& tStart = m_vvtWeldLineInfoGroup[m_nGroupNo][i].tWeldLine.StartPoint;
			//	double dDis = square(tStart.x - tPtnS.dX - tPtnS.dBX)
			//		+ square(tStart.y - tPtnS.dY - tPtnS.dBY);
			//		//+ square(tStart.z - tPtnS.dZ - tPtnS.dBZ);
			//	if (dDis < dMinDis)
			//	{
			//		nRealSmallGroupNoForTeach = i;
			//		dMinDis = dDis;
			//		bFound = true;
			//	}
			//}		
		}
		if (bIsScanMode)
		{
			//�޸�Ϊֱ��ʹ������������
			tPtnS = T_ROBOT_COORS();
			tPtnE = T_ROBOT_COORS();
			tPtnS.dX = m_vvtWeldLineInfoGroup[nGroupNo][0].tWeldLine.StartPoint.x;
			tPtnS.dY = m_vvtWeldLineInfoGroup[nGroupNo][0].tWeldLine.StartPoint.y;
			tPtnS.dZ = m_vvtWeldLineInfoGroup[nGroupNo][0].tWeldLine.StartPoint.z;
			tPtnE.dX = m_vvtWeldLineInfoGroup[nGroupNo][0].tWeldLine.EndPoint.x;
			tPtnE.dY = m_vvtWeldLineInfoGroup[nGroupNo][0].tWeldLine.EndPoint.y;
			tPtnE.dZ = m_vvtWeldLineInfoGroup[nGroupNo][0].tWeldLine.EndPoint.z;
			//tCameraNorm.x = -m_vvtWeldLineInfoGroup[m_nGroupNo][nRealSmallGroupNoForTeach].tWeldLine.StartNormalVector.x;
			//tCameraNorm.y = -m_vvtWeldLineInfoGroup[m_nGroupNo][nRealSmallGroupNoForTeach].tWeldLine.StartNormalVector.y;
			//tCameraNorm.z = -m_vvtWeldLineInfoGroup[m_nGroupNo][nRealSmallGroupNoForTeach].tWeldLine.StartNormalVector.z;
			//tCameraNorm.z = m_pRobotDriver->m_nRobotInstallDir > 0 ? -1.0 : 1.0;
		}
		else
		{
			m_pRobotDriver->m_cLog->Write("����δ�ҵ���Ӧ���뺸�������ܻ�Ӱ��������ȡ");
			bRst &= m_pRobotDriver->MoveToolByWeldGun(vtCoord[1], tCamTool, vtCoord[1], m_pRobotDriver->m_tTools.tGunTool, tPtnS);
			bRst &= m_pRobotDriver->MoveToolByWeldGun(vtCoord[2], tCamTool, vtCoord[2], m_pRobotDriver->m_tTools.tGunTool, tPtnE);
			//tPtnS.dZ -= 80.0 * m_nRobotInstallDir;
			//tPtnE.dZ -= 80.0 * m_nRobotInstallDir;
		}
		if (false == bRst)
		{
			XiMessageBoxOk("��ȡ���� ����ת������!");
			return false;
		}

		T_ROBOT_COORS tRobotVer = vtCoord[1];
		tRobotVer.dX = (tPtnS.dX + tPtnS.dBX + tPtnE.dX + tPtnE.dBX) / 2; // ����������� ��������ο���ʹ����������
		tRobotVer.dY = (tPtnS.dY + tPtnS.dBY + tPtnE.dY + tPtnE.dBY) / 2;
		tRobotVer.dZ = (tPtnS.dZ + tPtnS.dBZ + tPtnE.dZ + tPtnE.dBZ) / 2;
		if (eWeldSeamType == E_STAND_SEAM)
		{
			if (true)
			{
				tRefPtn[0] = cvPoint3D32f(tPtnS.dX + tPtnS.dBX, tPtnS.dY + tPtnS.dBY, tPtnS.dZ);
				tRefPtn[1] = cvPoint3D32f(tPtnE.dX + tPtnE.dBX, tPtnE.dY + tPtnE.dBY, tPtnE.dZ);
			}
			else
			{
				//RobotCoordPosOffset(tPtnS, 0.0, 0.0, 150.0 * m_nRobotInstallDir);
				tPtnS = tRobotVer;
				tPtnE = tRobotVer;
				RobotCoordPosOffset(tPtnE, 0.0, 0.0, 100.0 * m_nRobotInstallDir);
				tRefPtn[0] = cvPoint3D32f(tPtnS.dX, tPtnS.dY, tPtnS.dZ);
				tRefPtn[1] = cvPoint3D32f(tPtnE.dX, tPtnE.dY, tPtnE.dZ);
			}
			//---------------------------------
		}
		else
		{
			RobotCoordPosOffset(tPtnS, 0.0, 0.0, 100.0 * m_nRobotInstallDir);
			RobotCoordPosOffset(tPtnE, 0.0, 0.0, 100.0 * m_nRobotInstallDir);
			//---------------------------------
			tRefPtn[0] = cvPoint3D32f(tPtnS.dX + tPtnS.dBX, tPtnS.dY + tPtnS.dBY, tPtnS.dZ);
			tRefPtn[1] = cvPoint3D32f(tPtnE.dX + tPtnE.dBX, tPtnE.dY + tPtnE.dBY, tPtnE.dZ);
		}


		CvPoint3D32f tPlaneHNorm = cvPoint3D32f(0.0, 0.0, m_nRobotInstallDir > 0.0 ? 1.0 : -1.0);

		FILE* pfInput = fopen(sInputFile.GetBuffer(), "w");
		fprintf(pfInput, "%11.3lf%11.3lf%11.3lf\n", tCameraNorm.x, tCameraNorm.y, tCameraNorm.z);
		fprintf(pfInput, "%11.3lf%11.3lf%11.3lf\n", tPlaneHNorm.x, tPlaneHNorm.y, tPlaneHNorm.z);
		fprintf(pfInput, "%11.3lf%11.3lf%11.3lf\n", tRefPtn[0].x, tRefPtn[0].y, tRefPtn[0].z);
		fprintf(pfInput, "%11.3lf%11.3lf%11.3lf\n", tRefPtn[1].x, tRefPtn[1].y, tRefPtn[1].z);
		fclose(pfInput);

		//int nWeldInfoNum = GetRiserEndPoint(sPointCloudFile.GetBuffer(), tWeldInfo, tCameraNorm, tPlaneHNorm, tRefPtn, 2, "GetRiserEndPoint_Stand");
		CvPoint3D64f tmpCloudPt;
		int x;
		vector<CvPoint3D64f> vtCloudPt;
		FILE* pf = fopen(sPointCloudFile, "r");
		while (~fscanf(pf, "%d%lf%lf%lf", &x, &tmpCloudPt.x, &tmpCloudPt.y, &tmpCloudPt.z))
		{
			vtCloudPt.push_back(tmpCloudPt);
		}
		fclose(pf);
		int nWeldInfoNum = GetRiserEndPoint(vtCloudPt.data(), vtCloudPt.size(), tWeldInfo, tCameraNorm, tPlaneHNorm,
			tRefPtn, 2, eWeldSeamType == E_STAND_SEAM ? "GetRiserEndPoint_Stand" : "GetRiserEndPoint");

		pf = fopen(sRecoResultFile.GetBuffer(), "w");
		for (int i = 0; i < nWeldInfoNum; i++)
		{				
			if (eWeldSeamType == E_FLAT_SEAM)
			{
				CvPoint3D32f tTemp = tWeldInfo[i].staPnt;
				tWeldInfo[i].staPnt = tWeldInfo[i].endPnt;
				tWeldInfo[i].endPnt = tTemp;
			}
			else
			{
				if (((tWeldInfo[i].endPnt.z > tWeldInfo[i].staPnt.z) && (m_nRobotInstallDir < 0)) ||
					((tWeldInfo[i].endPnt.z < tWeldInfo[i].staPnt.z) && (m_nRobotInstallDir > 0)))
				{
					CvPoint3D32f tTemp = tWeldInfo[i].staPnt;
					tWeldInfo[i].staPnt = tWeldInfo[i].endPnt;
					tWeldInfo[i].endPnt = tTemp;
				}
			}
			fprintf(pf, "%d %4d%4d%11.3lf%4d%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%4d%4d 0 0 0 %4d%11.3lf%11.3lf 0 0.0\n",
				i, tWeldInfo[i].GrooveID, 0, tWeldInfo[i].GrooveWidth, -1 == tWeldInfo[i].GrooveType ? true : false,
				0.0, 0.0, 0.0,
				tWeldInfo[i].staPnt.x, tWeldInfo[i].staPnt.y, tWeldInfo[i].staPnt.z,
				tWeldInfo[i].endPnt.x, tWeldInfo[i].endPnt.y, tWeldInfo[i].endPnt.z,
				tWeldInfo[i].normal.x, tWeldInfo[i].normal.y, tWeldInfo[i].normal.z,
				tWeldInfo[i].normal.x, tWeldInfo[i].normal.y, tWeldInfo[i].normal.z,
				tWeldInfo[i].staPntWeldType, tWeldInfo[i].endPntWeldType,
				0, tWeldInfo[i].staPntRA_Radius, tWeldInfo[i].endPntRA_Radius);
		}
		fclose(pf);

		if (1 != nWeldInfoNum)
		{
			XUI::MesBox::PopInfo("��ȡ��������{0}����!", nWeldInfoNum);
			return false;
		}

		CopyFile((LPCTSTR)sRecoResultFile, (LPCTSTR)sSaveResultFile, FALSE);
		if (*m_pIsNaturalPop)
		{
			RunInteractiveWindow(sPointCloudFile, sSaveResultFile, sRecoResultFile);
		}
		// sSaveResultFile�ж�ȡ����յ�
		vector<WeldLineInfo> vtWeldLineInfo;
		if (false == LoadCloudProcessResult(sSaveResultFile, vtWeldLineInfo))
		{
			XiMessageBoxOk("��ȡ���� ����ʶ����ʧ��!");
			return false;
		}

		vtTeachResult.clear();
		tTeachResult.tKeyPtn3D.x = vtWeldLineInfo[0].tWeldLine.StartPoint.x;
		tTeachResult.tKeyPtn3D.y = vtWeldLineInfo[0].tWeldLine.StartPoint.y;
		tTeachResult.tKeyPtn3D.z = vtWeldLineInfo[0].tWeldLine.StartPoint.z;
		vtTeachResult.push_back(tTeachResult);
		vtTeachResult.push_back(tTeachResult);
		tTeachResult.tKeyPtn3D.x = vtWeldLineInfo[0].tWeldLine.EndPoint.x;
		tTeachResult.tKeyPtn3D.y = vtWeldLineInfo[0].tWeldLine.EndPoint.y;
		tTeachResult.tKeyPtn3D.z = vtWeldLineInfo[0].tWeldLine.EndPoint.z;
		vtTeachResult.push_back(tTeachResult);
		vtTeachResult.push_back(tTeachResult);
	}
	return true;
}

void WeldAfterMeasure::GetAdjoinSeam(int nSeamNo, vector<LineOrCircularArcWeldingLine>& vtSeamGroup, LineOrCircularArcWeldingLine** ptAdjoinSeamS, LineOrCircularArcWeldingLine** ptAdjoinSeamE)
{
	if (0 >= vtSeamGroup.size())
	{
		return;
	}
	LineOrCircularArcWeldingLine tCurSeam = vtSeamGroup[nSeamNo];
	GetAdjoinSeam(tCurSeam, vtSeamGroup, ptAdjoinSeamS, ptAdjoinSeamE);
	return;
}

void WeldAfterMeasure::GetAdjoinSeam(LineOrCircularArcWeldingLine tSeam, vector<LineOrCircularArcWeldingLine>& vtSeamGroup, LineOrCircularArcWeldingLine** ptAdjoinSeamS, LineOrCircularArcWeldingLine** ptAdjoinSeamE)
{
	*ptAdjoinSeamS = NULL;
	*ptAdjoinSeamE = NULL;
	if (0 >= vtSeamGroup.size())
	{
		return;
	}
	double dEndPointDisThreshold = 3.0; // ����
	LineOrCircularArcWeldingLine tCurSeam = tSeam;
	bool bIsStandSeam = IsStandWeldSeam(tCurSeam.StartNormalVector);
	if (bIsStandSeam)
	{
		for (int nOtherIdx = 0; nOtherIdx < vtSeamGroup.size(); nOtherIdx++) // ���������������ڵ�ƽ����
		{
			LineOrCircularArcWeldingLine tOtherSeam = vtSeamGroup[nOtherIdx];
			if (IsStandWeldSeam(tOtherSeam.StartNormalVector))
			{
				continue;
			}
			if (dEndPointDisThreshold > TwoPointDis(tCurSeam.StartPoint.x, tCurSeam.StartPoint.y, tCurSeam.StartPoint.z,
				tOtherSeam.StartPoint.x, tOtherSeam.StartPoint.y, tOtherSeam.StartPoint.z))
			{
				*ptAdjoinSeamS = &vtSeamGroup[nOtherIdx];
				break;
			}
		}
		for (int nOtherIdx = 0; nOtherIdx < vtSeamGroup.size(); nOtherIdx++) // �յ�������������ڵ�ƽ����
		{
			LineOrCircularArcWeldingLine tOtherSeam = vtSeamGroup[nOtherIdx];
			if (IsStandWeldSeam(tOtherSeam.StartNormalVector))
			{
				continue;
			}
			if (dEndPointDisThreshold > TwoPointDis(tCurSeam.StartPoint.x, tCurSeam.StartPoint.y, tCurSeam.StartPoint.z,
				tOtherSeam.EndPoint.x, tOtherSeam.EndPoint.y, tOtherSeam.EndPoint.z))
			{
				*ptAdjoinSeamE = &vtSeamGroup[nOtherIdx];
				break;
			}
		}
	}
	else
	{
		for (int nOtherIdx = 0; nOtherIdx < vtSeamGroup.size(); nOtherIdx++) // ƽ���������ƽ����
		{
			LineOrCircularArcWeldingLine tOtherSeam = vtSeamGroup[nOtherIdx];
			if (IsStandWeldSeam(tOtherSeam.StartNormalVector))
			{
				continue;
			}
			if (dEndPointDisThreshold > TwoPointDis(tCurSeam.StartPoint.x, tCurSeam.StartPoint.y, tCurSeam.StartPoint.z,
				tOtherSeam.EndPoint.x, tOtherSeam.EndPoint.y, tOtherSeam.EndPoint.z))
			{
				*ptAdjoinSeamS = &vtSeamGroup[nOtherIdx];
				break;
			}
		}
		for (int nOtherIdx = 0; nOtherIdx < vtSeamGroup.size(); nOtherIdx++) // ƽ���յ�����ƽ����
		{
			LineOrCircularArcWeldingLine tOtherSeam = vtSeamGroup[nOtherIdx];
			if (IsStandWeldSeam(tOtherSeam.StartNormalVector))
			{
				continue;
			}
			if (dEndPointDisThreshold > TwoPointDis(tCurSeam.EndPoint.x, tCurSeam.EndPoint.y, tCurSeam.EndPoint.z,
				tOtherSeam.StartPoint.x, tOtherSeam.StartPoint.y, tOtherSeam.StartPoint.z))
			{
				*ptAdjoinSeamE = &vtSeamGroup[nOtherIdx];
				break;
			}
		}
	}
	return;
}

void WeldAfterMeasure::CalcInterfereInfo(int nSeamNo, vector<LineOrCircularArcWeldingLine> vtWeldSeam, vector<T_ROBOT_COORS>& vtWeldCoord,
	double& dChangeDisS, double& dChangeDisE, int& nChangePtnNum, int& nDelPtnNum, double& dWeldHoleSizeS, double& dWeldHoleSizeE)
{
	// ����ι켣ֱ��ɾ�� ɾ������ɾ�������ײ���(����ο��ܰ����ڹ�������)
	// ����δ��ڹ����� �����׵���0�� С�ڹ����״�С��ȥ����γ���
	// �и����ʱ�������̬���룬����̬�������dChangeDisʱ ����dChangeDis
	// dWeldHoleDis  dChangeDis
	double dWeldSilkLen = 20.0;
	double dWeldGunRadius = 10.0;
	double dAcuteThreshold = 85.0;
	double dGunWidth = fabs(m_pRobotDriver->m_tTools.tGunTool.dX);
	XiAlgorithm alg;
	LineOrCircularArcWeldingLine tSeam = vtWeldSeam[nSeamNo];
	LineOrCircularArcWeldingLine* ptAdjoinSeamS = NULL;
	LineOrCircularArcWeldingLine* ptAdjoinSeamE = NULL;
	GetAdjoinSeam(tSeam, vtWeldSeam, &ptAdjoinSeamS, &ptAdjoinSeamE);
	double dStoEDir = alg.CalcArcAngle(tSeam.EndPoint.x - tSeam.StartPoint.x, tSeam.EndPoint.y - tSeam.StartPoint.y);
	double dEtoSDir = dStoEDir + 180.0;
	if (NULL != ptAdjoinSeamS)
	{
		double dAngle = alg.VectorsInnerAngle(
			ptAdjoinSeamS->StartPoint.x - ptAdjoinSeamS->EndPoint.x,
			ptAdjoinSeamS->StartPoint.y - ptAdjoinSeamS->EndPoint.y, 0.0,
			tSeam.EndPoint.x - tSeam.StartPoint.x,
			tSeam.EndPoint.y - tSeam.StartPoint.y, 0.0);
		if (fabs(dAngle) < dAcuteThreshold)
		{
			double dInterfereDisS = (dWeldSilkLen + dWeldGunRadius) / 1.414 / TanD(dAngle);
			vtWeldCoord.erase(vtWeldCoord.begin(), vtWeldCoord.begin() + (int)dInterfereDisS);
			dWeldHoleSizeS = dInterfereDisS > dWeldHoleSizeS ? 0.0: dWeldHoleSizeS - dInterfereDisS;
			double dRealChangeDis = dGunWidth / TanD(dAngle) + 25.0; // ���Ǻ�ǹǹ�ܰ뾶
			dChangeDisS = dChangeDisS < dRealChangeDis - dInterfereDisS ? dRealChangeDis - dInterfereDisS : dChangeDisS;
		}
	}
	if (NULL != ptAdjoinSeamE)
	{
		double dAngle = alg.VectorsInnerAngle(
			ptAdjoinSeamE->EndPoint.x - ptAdjoinSeamE->StartPoint.x,
			ptAdjoinSeamE->EndPoint.y - ptAdjoinSeamE->StartPoint.y, 0.0,
			tSeam.StartPoint.x - tSeam.EndPoint.x,
			tSeam.StartPoint.y - tSeam.EndPoint.y, 0.0);
		if (fabs(dAngle) < dAcuteThreshold)
		{
			double dInterfereDisE = (dWeldSilkLen + dWeldGunRadius) / 1.414 / TanD(dAngle);
			vtWeldCoord.erase(vtWeldCoord.end() - (int)dInterfereDisE, vtWeldCoord.end());
			dWeldHoleSizeE = dInterfereDisE > dWeldHoleSizeE ? 0.0 : dWeldHoleSizeE - dInterfereDisE;
			double dRealChangeDis = dGunWidth / TanD(dAngle) + 25.0; // ���Ǻ�ǹǹ�ܰ뾶
			dChangeDisE = dRealChangeDis - dInterfereDisE;
			dChangeDisE = dChangeDisE < dRealChangeDis - dInterfereDisE ? dRealChangeDis - dInterfereDisE : dChangeDisE;
		}
	}
}

bool WeldAfterMeasure::IsAcuteSeamStand(LineOrCircularArcWeldingLine tSeam, double dAcuteAngleThreshold)
{
	LineOrCircularArcWeldingLine* ptAdjoinSeamS = NULL;
	LineOrCircularArcWeldingLine* ptAdjoinSeamE = NULL;
	GetAdjoinSeam(tSeam, m_vtWeldSeamData, &ptAdjoinSeamS, &ptAdjoinSeamE);
	if (NULL == ptAdjoinSeamS || NULL == ptAdjoinSeamE)
	{
		return false;
	}
	XiAlgorithm alg;
	double dAngle = alg.VectorsInnerAngle(
		ptAdjoinSeamS->EndPoint.x - ptAdjoinSeamS->StartPoint.x,
		ptAdjoinSeamS->EndPoint.y - ptAdjoinSeamS->StartPoint.y, 0.0,
		ptAdjoinSeamE->StartPoint.x - ptAdjoinSeamE->EndPoint.x,
		ptAdjoinSeamE->StartPoint.y - ptAdjoinSeamE->EndPoint.y, 0.0);
	return fabs(dAngle) < dAcuteAngleThreshold;
}

XI_POINT WeldAfterMeasure::GetPieceHeight(int nGroupNo, double zOffeset)
{
	XI_POINT backPoint;
	vector<LineOrCircularArcWeldingLine> vtSeamGroup;
	vector<double> vtPieceHeight;
	vector<LineOrCircularArcWeldingLine> vtGroup;
	for (int nGroup = 0; nGroup < m_vvtWeldSeamGroup.size(); nGroup++)
	{
		for (int nSeam = 0; nSeam < m_vvtWeldSeamGroup[nGroup].size(); nSeam++)
		{
			//ȫ��������Ϣ
			vtSeamGroup.push_back(m_vvtWeldSeamGroup[nGroup][nSeam]);
		}
		if (nGroup == nGroupNo)
		{
			//��ǰ���麸����Ϣ
			vtGroup = m_vvtWeldSeamGroup[nGroup];
		}
	}
	//��ǰ�������ĵ�
	vector<double> xAv;
	vector<double> yAv;
	for (int n = 0; n < vtGroup.size(); n++)
	{
		xAv.push_back(vtGroup[n].StartPoint.x);
		xAv.push_back(vtGroup[n].EndPoint.x);
		yAv.push_back(vtGroup[n].StartPoint.y);
		yAv.push_back(vtGroup[n].EndPoint.y);
	}
	double xMax = *max_element(xAv.begin(), xAv.end());
	double xMin = *min_element(xAv.begin(), xAv.end());
	double yMax = *max_element(yAv.begin(), yAv.end());
	double yMin = *min_element(yAv.begin(), yAv.end());
	backPoint.x = (xMax + xMin) / 2;
	backPoint.y = (yMax + yMin) / 2;
	//��ȡ����ȫ���߶�
	for (int i = 0; i < vtSeamGroup.size(); i++)
	{
		vtPieceHeight.push_back(vtSeamGroup[i].EndPoint.z);
		vtPieceHeight.push_back(vtSeamGroup[i].StartPoint.z);
		vtPieceHeight.push_back(vtSeamGroup[i].ZSide);
	}
	////�����߶�
	//double maxHeight = vtPieceHeight[0];
	//for (int j = 0; j < vtPieceHeight.size(); j++)
	//{
	//	if (maxHeight <= vtPieceHeight[j])
	//	{
	//		swap(maxHeight, vtPieceHeight[j]);
	//	}
	//}
	//backPoint.z = maxHeight + zOffeset;
	sort(vtPieceHeight.begin(), vtPieceHeight.end(), [](const double& d1, const double& d2)->bool { return d1 < d2; });
	double maxHeight = 1 == m_nRobotInstallDir ? vtPieceHeight[vtPieceHeight.size() - 1] : vtPieceHeight[0];
	backPoint.z = maxHeight + (zOffeset * (double)m_nRobotInstallDir);
	// �������
	backPoint.z -= m_ptUnit->m_dExAxisZPos;
	return backPoint;
}

bool WAM::WeldAfterMeasure::PointCloudProcessWithModel()
{
	if (Matching(m_eWorkPieceType))
	{
		XiMessageBoxOk("ģ��ƥ�����");
		return true;
	}
	else
	{
		XiMessageBoxOk("ģ��ƥ��ʧ��");
		return false;
	}
}

bool WeldAfterMeasure::GetWorkPieceName(E_WORKPIECE_TYPE eWorkPieceType, CString& sWorkPieceName, CString& sWorkPieceTypeName)
{
	bool bRst = true;
	switch (eWorkPieceType)
	{
	case E_DIAPHRAGM: // ����
		sWorkPieceTypeName.Format("E_DIAPHRAGM"); 
		sWorkPieceName.Format("ʵʱ����");
		break;
	case E_LINELLAE: // ���� (С����)
		sWorkPieceTypeName.Format("E_LINELLAE"); 
		sWorkPieceName.Format("���ָ�");
		break;
	case SMALL_PIECE: // Сɢ��
		sWorkPieceTypeName.Format("SMALL_PIECE");
		sWorkPieceName.Format("С��");
		break;
	case STIFFEN_PLATE: // �Ӿ���
		sWorkPieceTypeName.Format("STIFFEN_PLATE");
		sWorkPieceName.Format("������");
		break;
	case E_PURLIN_HANGER: // ����
		sWorkPieceTypeName.Format("E_PURLIN_HANGER");
		sWorkPieceName.Format("����");
		break;
	case E_END_PLATE: // �˰� ������
		sWorkPieceTypeName.Format("E_END_PLATE");
		sWorkPieceName.Format("������");
		break;
	case E_CORBEL: // ţ��
		sWorkPieceTypeName.Format("E_CORBEL");
		sWorkPieceName.Format("ţ��");
		break;
	case E_CAKE: // �����
		sWorkPieceTypeName.Format("E_CAKE");
		sWorkPieceName.Format("�����");
		break;
	case E_BEVEL: // �¿�
		sWorkPieceTypeName.Format("E_BEVEL");
		sWorkPieceName.Format("�¿�");
		break;
	case E_SINGLE_BOARD: // ���嵥��
		sWorkPieceTypeName.Format("E_SINGLE_BOARD");
		sWorkPieceName.Format("���嵥��");
		break;
	case E_SCAN_WELD_LINE: // ɨ�����
		sWorkPieceTypeName.Format("E_SCAN_WELD_LINE");
		sWorkPieceName.Format("ɨ�����");
		break;
	default:
		bRst = false;
		sWorkPieceTypeName.Format("unknown");
		sWorkPieceName.Format("δ֪");
		//���޸�
		XUI::MesBox::PopInfo("��������{0}����", 1);
		//XiMessageBox("��������%d����", eWorkPieceType);
	}
	return bRst;
}

void WeldAfterMeasure::InputParaWindow(CString sUnitName, E_WORKPIECE_TYPE eWorkPieceType)
{
	CString sWorkPieceName;
	CString sWorkPieceTypeName;
	GetWorkPieceName(eWorkPieceType, sWorkPieceName, sWorkPieceTypeName);
	
	//���޸�
	sWorkPieceName = XUI::Languge::GetInstance().translate(sWorkPieceName.GetBuffer());
	CString str1 = "�������";
	str1 = XUI::Languge::GetInstance().translate(str1.GetBuffer());
	CString cStrTitle = sWorkPieceName + str1;
	//CString cStrTitle = sWorkPieceName + "�������";
	int nParamNum;
	COPini opini;
	opini.SetFileName(DATA_PATH + sUnitName + SYETEM_PARAM_FILE);
	opini.SetSectionName(sWorkPieceTypeName);
	// ��ȡ��ǰ������Ҫ����Ĳ�������
	opini.ReadString("TotalNumber", &nParamNum);
	// ��������������
	std::vector<CString> vsInputName(nParamNum);
	// ʵ�ʲ�����ֵ
	std::vector<double> vnInputData(nParamNum);
	for (int i = 0; i < nParamNum; i++)
	{
		CString cStrName;
		cStrName.Format("ParamName%d", i);
		CString cStrValue;
		cStrValue.Format("ParamValue%d", i);

		opini.ReadString(cStrName, vsInputName[i]);
		vsInputName[i] = Utf8ToGBK(vsInputName[i]);
		opini.ReadString(cStrValue, &vnInputData[i]);
	}
	// �޸�ʶ���������
	ParamInput cParamDlg(cStrTitle, vsInputName, &vnInputData);
	int nRst = cParamDlg.DoModal();
	for (int i = 0; i < nParamNum; i++)
	{
		CString cStrValue;
		cStrValue.Format("ParamValue%d", i);
		opini.WriteString(cStrValue, vnInputData[i], 3);
	}
}

bool WeldAfterMeasure::GenerateWeldLineData(int nGroupNo, double dWeldHoleSize, double dBoardThick, bool bTheoryCheck/* = false*/)
{
	bool bFlatWeldContinue = m_bFlatWeldContinue;
	if (nGroupNo < 0 || nGroupNo > m_vvtWeldSeamGroupAdjust.size())
	{
		return false;
	}

	vector<LineOrCircularArcWeldingLine>& vtWeldSeam = m_vvtWeldSeamGroupAdjust[nGroupNo];
	bool bIsOutsideWeld = JudgeOutsideWeld(vtWeldSeam); // �Ƿ�Ϊ��ຸ��
	double dRzChangeDir = bIsOutsideWeld ? -1.0 : 1.0; // ��ຸ��Rz�仯���� ���ڲ��෴
	XiAlgorithm alg;
	double dWeldDirAngle;
	E_WELD_SEAM_TYPE eWeldSeamType;
	vector<T_ROBOT_COORS> vtAllWeldCoord(0);
	vector<int> vnAllPointType(0);
	vector<T_ROBOT_COORS> vtWeldCoord;
	LineOrCircularArcWeldingLine tWeldSeam;

	vector<int> vnWeldOrder;
	vector<int> vnStandWeldNoIdx;
	vector<int> vnFlatWeldNoIdx;
	vnWeldOrder.clear();
	vnStandWeldNoIdx.clear();
	vnFlatWeldNoIdx.clear();
	for (int i = 0; i < vtWeldSeam.size(); i++)
	{
		eWeldSeamType = GetWeldSeamType(vtWeldSeam[i]);
		if (E_FLAT_SEAM == eWeldSeamType)
		{
			vnFlatWeldNoIdx.push_back(i);
		}
		else if (E_STAND_SEAM == eWeldSeamType)
		{
			vnStandWeldNoIdx.push_back(i);
		}
	}

	// ��������������ƽ�������ţ�������������˳���������(��֤������ӹ켣˳�����Ӻ󼴿���������)
	if (bFlatWeldContinue)
	{
		GetContiueWeldOrderFlat(vtWeldSeam, vnFlatWeldNoIdx);
	}

	vnWeldOrder.insert(vnWeldOrder.end(), vnStandWeldNoIdx.begin(), vnStandWeldNoIdx.end());
	vnWeldOrder.insert(vnWeldOrder.end(), vnFlatWeldNoIdx.begin(), vnFlatWeldNoIdx.end());


	//��ȡʾ�̽�� 
	std::vector<std::vector<T_TEACH_DATA>> vvtTeachData(vtWeldSeam.size(), std::vector<T_TEACH_DATA>(0)); // ÿ���������֣�����1����� ����2�������
	// ���ʾ�̽������������Ƿ�����m_vtTeachData������ �����ҵ�ǰ������ÿ������Ĳ�������
	for (int nDataNo = 0; nDataNo < m_vtTeachData.size(); nDataNo++)
	{
		vvtTeachData[m_vtTeachData[nDataNo].nWeldNo].push_back(m_vtTeachData[nDataNo]);
	}
	int nWeldNo = 0;
	for (int i = 0; i < vnWeldOrder.size(); i++)
	{
		nWeldNo = vnWeldOrder[i];
		tWeldSeam = vtWeldSeam[nWeldNo];

		if (WeldingLineIsArc(tWeldSeam))
		{
			CHECK_BOOL_RETURN(GenerateWeldTrackArc(tWeldSeam));
			continue;
		}

		double dWeldHoleDisS = m_vvtWeldLineInfoGroup[nGroupNo][nWeldNo].tAtrribute.dStartHoleSize; // dWeldHoleSize; // �����׳ߴ�
		double dWeldHoleDisE = m_vvtWeldLineInfoGroup[nGroupNo][nWeldNo].tAtrribute.dEndHoleSize; // dWeldHoleSize; // �����׳ߴ�
		double dChangeDisS = 45.0;	// ����̬����
		double dChangeDisE = 45.0;	// ����̬����
		double dChangeAngle = 30.0; // ����̬�Ƕ�
		double dPtnInterval = 2.0;  // �켣����
		// ��ȡ�������� ���ɺ��ӹ켣
		eWeldSeamType = GetWeldSeamType(tWeldSeam);
		double dTempVar = dChangeDisS;
		if (E_STAND_SEAM == eWeldSeamType) // ���� ������㲻����̬
		{
			dChangeDisS = 0.0;
		}
		else
		{
			dChangeDisS = dTempVar;
		}

		if (E_STAND_SEAM == eWeldSeamType)
		{
			dChangeDisS = 20;
			dChangeDisE = 20;
			dChangeAngle = 55.0;
		}

		double dPostureRx = E_FLAT_SEAM == eWeldSeamType ? m_dPlatWeldRx : m_dStandWeldRx;
		double dPostureRy = E_FLAT_SEAM == eWeldSeamType ? m_dPlatWeldRy : m_dStandWeldRy;
		dWeldDirAngle = alg.CalcArcAngle(tWeldSeam.StartNormalVector.x, tWeldSeam.StartNormalVector.y);
		if (false == GenerateWeldLineData(tWeldSeam.StartPoint, tWeldSeam.EndPoint, dPtnInterval, dPostureRx, dPostureRy, dWeldDirAngle, eWeldSeamType, vtWeldCoord))
		{
			XiMessageBox("������յ����꼰��̬����ֱ�����꺸��켣����ʧ��");
			return false;
		}
		
		int nChangePtnNum = (int)(dChangeDisS / dPtnInterval); // ����̬����
		m_ChangeDisPoint = nChangePtnNum;
		int nDelPtnNum = (int)(dWeldHoleDisS / dPtnInterval); // ������ɾ������
		double dStepChangeAngle = (double)m_nRobotInstallDir * dChangeAngle / (double)nChangePtnNum; // ���ڵ���̬�仯�Ƕ�

		if (E_FLAT_SEAM == eWeldSeamType)
		{
			CalcInterfereInfo(nWeldNo, vtWeldSeam, vtWeldCoord, 
				dChangeDisS, dChangeDisE, nChangePtnNum, nDelPtnNum,dWeldHoleDisS, dWeldHoleDisE);
		}

		int nChangePtnNumS = (int)(dChangeDisS / dPtnInterval); // ����̬����
		int nChangePtnNumE = (int)(dChangeDisE / dPtnInterval); // ����̬����
		int nDelPtnNumS = (int)(dWeldHoleDisS / dPtnInterval); // ������ɾ������
		int nDelPtnNumE = (int)(dWeldHoleDisE / dPtnInterval); // ������ɾ������
		double dStepChangeAngleS = (double)m_nRobotInstallDir * dChangeAngle / (double)nChangePtnNumS; // ���ڵ���̬�仯�Ƕ�
		double dStepChangeAngleE = (double)m_nRobotInstallDir * dChangeAngle / (double)nChangePtnNumE; // ���ڵ���̬�仯�Ƕ�

		AddMeasurePointsToLongSeam(nWeldNo,vvtTeachData,tWeldSeam,dPtnInterval,dPostureRx,dPostureRy,dWeldDirAngle,eWeldSeamType,vtWeldCoord);
		
		int nWeldTrackPtnNum = vtWeldCoord.size();
		if (nWeldTrackPtnNum < 10 || 
			nWeldTrackPtnNum < nChangePtnNumS || 
			nWeldTrackPtnNum < nChangePtnNumE )
		{
			//���޸�
			//XiMessageBox("����켣����%d���� �޷����ӣ�", nWeldTrackPtnNum);
			XUI::MesBox::PopInfo("����켣����{0}���� �޷����ӣ�", nWeldTrackPtnNum);
			return false;
		}

		// ����˵����̬  ɾ�������׹켣
		if (E_FLAT_SEAM == eWeldSeamType && 0 < tWeldSeam.EndPointType) // ƽ�������յ� 
		{
			int nBaseIndex = nWeldTrackPtnNum - 1 - nChangePtnNumE;
			double dSrcRz = vtWeldCoord[nBaseIndex].dRZ;
			for (int n = nWeldTrackPtnNum - nChangePtnNumE; n < nWeldTrackPtnNum; n++)
			{
				vtWeldCoord[n].dRZ = dSrcRz - ((n - nBaseIndex) * dStepChangeAngleE * dRzChangeDir); // ��̬��С(���������Ұ�װ��ʽ���)(֧������ຸ��)
			}
			vtWeldCoord.erase(vtWeldCoord.end() - nDelPtnNumE, vtWeldCoord.end()); // ɾ������
		}
		if (E_FLAT_SEAM == eWeldSeamType && 0 < tWeldSeam.StartPointType) // ƽ���������
		{
			double dSrcRz = vtWeldCoord[nChangePtnNumS].dRZ;
			for (int n = nChangePtnNumS - 1; n >= 0; n--)
			{
				vtWeldCoord[n].dRZ = dSrcRz + ((nChangePtnNumS - n) * dStepChangeAngleS * dRzChangeDir); // ��̬����(���������Ұ�װ��ʽ���)(֧������ຸ��)
			}
			vtWeldCoord.erase(vtWeldCoord.begin(), vtWeldCoord.begin() + nDelPtnNumS); // ɾ������
		}
		if (E_STAND_SEAM == eWeldSeamType)
		{
			double dSrcRy = vtWeldCoord[0].dRY;
			double dDstRy = m_dPlatWeldRy;
			double dStepChangeAngleStandSeam = (dDstRy - dSrcRy) / (double)nChangePtnNum;
			double dVectorDis = sqrt((double)(tWeldSeam.StartNormalVector.x * tWeldSeam.StartNormalVector.x + 
				tWeldSeam.StartNormalVector.y * tWeldSeam.StartNormalVector.y + tWeldSeam.StartNormalVector.z * tWeldSeam.StartNormalVector.z));
			double dZDis = fabs(tWeldSeam.StartNormalVector.z);
			double dAngle = asin(dZDis / dVectorDis) * 180.0 / PI;
			if (dAngle < 15)
			{
				for (int n = nChangePtnNum - 1; n >= 0; n--)
				{
					vtWeldCoord[n].dRY = dSrcRy + ((nChangePtnNum - n) * dStepChangeAngleStandSeam);
				}
				m_bIsSlope = false;
			}
			else
			{
				m_bIsSlope = true;
				for (int m = 0;m < vtWeldCoord.size();m++)
				{
					vtWeldCoord[m].dRY = 45;
					if (tWeldSeam.StartNormalVector.y > 0)
					{
						vtWeldCoord[m].dRZ += 15;
					}
					else
					{
						vtWeldCoord[m].dRZ -= 15;
					}
				}
			}
			vtWeldCoord.erase(vtWeldCoord.begin(), vtWeldCoord.begin() + nDelPtnNum); // ɾ������
		}

		// ������Ӱ��Ƕ���(��ʹ��ZSide�ж��Ƿ��ܰ��ǣ�
		vector<int> vnPointType;
		std::vector<T_WELD_PARA> vtWeldPara;
		int nWeldAngleSize = m_vvtWeldLineInfoGroup[nGroupNo][nWeldNo].tAtrribute.nWeldAngleSize;
		if (false == GetWeldParam(eWeldSeamType, nWeldAngleSize, vtWeldPara))
		{
			XiMessageBox("���غ��ӹ��ղ���ʧ�ܣ�");
			return false;
		}
		if (false == CalcWrapTrack(m_vvtWeldLineInfoGroup[nGroupNo][nWeldNo], eWeldSeamType, nWeldAngleSize / 2.0, vtWeldPara[0].nStandWeldDir, vtWeldCoord, vnPointType))
		{
			XiMessageBox("���ǹ켣����ʧ��");
			return false;
		}

		// ���ƽ�����ɶ˼���ˮ�״����ǣ�ƽ�����ǣ�
		if (E_FLAT_SEAM == eWeldSeamType && (TRUE == *m_bNeedWrap)) // ƽ�� ѡ����ǲŰ���
		{
			T_ROBOT_COORS tDirCoor, tCoor, tWrapPoint;
			/*
				nStartWrapType
				0:��㲻��ƴ�ӵ㣬�յ㲻��ƴ�ӵ�
				1:�����ƴ�ӵ㣬�յ㲻��ƴ�ӵ�
				2:��㲻��ƴ�ӵ㣬�յ���ƴ�ӵ�
				3:�����ƴ�ӵ㣬�յ���ƴ�ӵ�
			*/
			// �յ㲻��ƴ�ӵ�
			if (m_vvtWeldLineInfoGroup[nGroupNo][nWeldNo].tAtrribute.nStartWrapType == 0
				|| m_vvtWeldLineInfoGroup[nGroupNo][nWeldNo].tAtrribute.nStartWrapType == 1)
			{
				/*
					���� �й��� ��
					���� �޹��� ��
					���� �й��� ��
					���� �޹��� ����
				*/
				//jwq��ʱ��ɾ������
				if(false)
				//���ɶ� || ����˲��ҹ����׳ߴ����20
				//if ((!tWeldSeam.EndPointType) || (dWeldHoleDisE > 30.0 && tWeldSeam.EndPointType))
				{
					tDirCoor = vtWeldCoord.at(vtWeldCoord.size() - 5);
					tCoor = vtWeldCoord.at(vtWeldCoord.size() - 1);//��β��
					tWrapPoint = RobotCoordPosOffset(tCoor, tDirCoor, tCoor, 5);
					if (m_vvtWeldLineInfoGroup[nGroupNo][nWeldNo].tAtrribute.nEndWrapType == 0 ||
						m_vvtWeldLineInfoGroup[nGroupNo][nWeldNo].tAtrribute.nEndWrapType == 1)
					{
						vtWeldCoord.push_back(tWrapPoint);
						vnPointType.push_back(E_WELD_WRAP);
					}
					tWrapPoint.dX -= 7 * CosD(tWrapPoint.dRZ);
					tWrapPoint.dY -= 7 * SinD(tWrapPoint.dRZ);
					if (!tWeldSeam.EndPointType) {	//�����յ�
						/*
							nStartWrapType:
							0�������ǣ��յ����
							1����㲻���ǣ��յ����
							2�������ǣ��յ㲻����
							3����㲻���ǣ��յ㲻����
						*/
						if (m_vvtWeldLineInfoGroup[nGroupNo][nWeldNo].tAtrribute.nEndWrapType == 0 ||
							m_vvtWeldLineInfoGroup[nGroupNo][nWeldNo].tAtrribute.nEndWrapType == 1)
						{
							tWrapPoint.dRZ += 3.0 * m_nRobotInstallDir;
							vtWeldCoord.push_back(tWrapPoint);
							vnPointType.push_back(E_WELD_WRAP);
						}
					}
					else {	//�����յ�
						vtWeldCoord.push_back(tWrapPoint);
						vnPointType.push_back(E_WELD_WRAP);
					}
				}
			}

			/*
				nStartWrapType
				0:��㲻��ƴ�ӵ㣬�յ㲻��ƴ�ӵ�
				1:�����ƴ�ӵ㣬�յ㲻��ƴ�ӵ�
				2:��㲻��ƴ�ӵ㣬�յ���ƴ�ӵ�
				3:�����ƴ�ӵ㣬�յ���ƴ�ӵ�
			*/
			// ��㲻��ƴ�ӵ�
			if (m_vvtWeldLineInfoGroup[nGroupNo][nWeldNo].tAtrribute.nStartWrapType == 0
				|| m_vvtWeldLineInfoGroup[nGroupNo][nWeldNo].tAtrribute.nStartWrapType == 2)
			{
				//jwq��ʱ��ɾ������
				if (false)
				//���ɶ� || ����˲��ҹ����׳ߴ����20
				//if ((!tWeldSeam.StartPointType) || (dWeldHoleDisS > 30.0 && tWeldSeam.StartPointType))
				{
					tDirCoor = vtWeldCoord.at(5);
					tCoor = vtWeldCoord.at(0);//��ʼ��
					tWrapPoint = RobotCoordPosOffset(tCoor, tDirCoor, tCoor, 3);
					if (m_vvtWeldLineInfoGroup[nGroupNo][nWeldNo].tAtrribute.nEndWrapType == 0 ||
						m_vvtWeldLineInfoGroup[nGroupNo][nWeldNo].tAtrribute.nEndWrapType == 2)
					{
						vtWeldCoord.insert(vtWeldCoord.begin(), tWrapPoint);
						vnPointType.insert(vnPointType.begin(), E_WELD_WRAP);
					}
					tWrapPoint.dX -= 7 * CosD(tWrapPoint.dRZ);
					tWrapPoint.dY -= 7 * SinD(tWrapPoint.dRZ);
					if (!tWeldSeam.StartPointType) {	//�������
						/*
							nStartWrapType:
							0�������ǣ��յ����
							1����㲻���ǣ��յ����
							2�������ǣ��յ㲻����
							3����㲻���ǣ��յ㲻����
						*/
						if (m_vvtWeldLineInfoGroup[nGroupNo][nWeldNo].tAtrribute.nEndWrapType == 0 ||
							m_vvtWeldLineInfoGroup[nGroupNo][nWeldNo].tAtrribute.nEndWrapType == 2)
						{
							tWrapPoint.dRZ -= 3.0 * m_nRobotInstallDir;
							vtWeldCoord.insert(vtWeldCoord.begin(), tWrapPoint);
							vnPointType.insert(vnPointType.begin(), E_WELD_WRAP);
						}
					}
					else {	//�������
						vtWeldCoord.insert(vtWeldCoord.begin(), tWrapPoint);
						vnPointType.insert(vnPointType.begin(), E_WELD_WRAP);
					}
				}
			}
		}

		// ������������ߵ��켣��
		// 1.ƽ���Ҹ���(���)���������(�����)�յ� 
		// 2.���������º���  
		// 3.���ָ�ƽ�����x�����յ�x 
		bool bTrackReverse = false;
		if (((E_FLAT_SEAM == eWeldSeamType) && (1 == vtWeldSeam[nWeldNo].StartPointType) && (false == vtWeldSeam[nWeldNo].EndPointType)) ||  // ������ʱ
			((E_STAND_SEAM == eWeldSeamType) && (0 == vtWeldPara[0].nStandWeldDir)) ||
			((E_FLAT_SEAM == eWeldSeamType) && (E_LINELLAE == m_eWorkPieceType) && (vtWeldCoord[0].dX > vtWeldCoord[vtWeldCoord.size() - 1].dX)))
		{
			bTrackReverse = true;
			int nWeldTrackPtnNum = vtWeldCoord.size();
			for (int nPtnNo = 0; nPtnNo < nWeldTrackPtnNum / 2; nPtnNo++)
			{
				swap(vtWeldCoord[nPtnNo], vtWeldCoord[nWeldTrackPtnNum - 1 - nPtnNo]);
				swap(vnPointType[nPtnNo], vnPointType[nWeldTrackPtnNum - 1 - nPtnNo]);
			}
		}
		
		// ��ӹ�����Ǳ���̬����֧�ֹ��ָ�ƽ����
		if (E_LINELLAE == m_eWorkPieceType)
		{
			double dWeldDipAngle = false == bTrackReverse ? vtWeldPara[0].dWeldDipAngle : -vtWeldPara[0].dWeldDipAngle;
			for (int nPtnNo = 0; nPtnNo < vtWeldCoord.size(); nPtnNo++)
			{
				vtWeldCoord[nPtnNo].dRZ += (dWeldDipAngle * (double)m_nRobotInstallDir);
			}
		}

		if ((E_FLAT_SEAM != eWeldSeamType) || !bFlatWeldContinue)
		{
			// ���� ��� ֱ������ �ⲿ������ ��������
			CString sFileName;
			sFileName.Format("%s%d_%d_RealWeldCoord.txt", m_sDataSavePath, nGroupNo, i);
			FILE* pf = fopen(sFileName, "w");
			for (int nPtnIdx = 0; nPtnIdx < vtWeldCoord.size(); nPtnIdx++)
			{
				T_ROBOT_COORS& tCoord = vtWeldCoord[nPtnIdx];
				if (bTheoryCheck) 
				{
#ifdef SINGLE_ROBOT
					tCoord.dY -= m_dTeachExAxlePos;
					m_dTeachExAxlePos_y = m_dTeachExAxlePos;
#else
					tCoord.dX -= m_dTeachExAxlePos;
					tCoord.dY -= m_dTeachExAxlePos_y;
#endif
				}
				fprintf(pf, "%d%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%4d%10d\n", nPtnIdx,
					tCoord.dX, tCoord.dY, tCoord.dZ, tCoord.dRX, tCoord.dRY, tCoord.dRZ, m_dTeachExAxlePos, m_dTeachExAxlePos_y, eWeldSeamType, vnPointType[nPtnIdx]);
			}
			fclose(pf);
		}
		else
		{
			vtAllWeldCoord.insert(vtAllWeldCoord.end(), vtWeldCoord.begin(), vtWeldCoord.end());
			vnAllPointType.insert(vnAllPointType.end(), vnPointType.begin(), vnPointType.end());
		}
	}
	if (bFlatWeldContinue && (vtAllWeldCoord.size() > 0) && (vtAllWeldCoord.size() == vnAllPointType.size()))
	{
		CString sFileName;
		sFileName.Format("%s%d_%d_RealWeldCoord.txt", m_sDataSavePath, nGroupNo, vnStandWeldNoIdx.size());
		FILE* pf = fopen(sFileName, "w");
		for (int nPtnIdx = 0; nPtnIdx < vtAllWeldCoord.size(); nPtnIdx++)
		{
			T_ROBOT_COORS& tCoord = vtAllWeldCoord[nPtnIdx];
			if (bTheoryCheck)
			{
#ifdef SINGLE_ROBOT
				tCoord.dY -= m_dTeachExAxlePos;
#else
				tCoord.dX -= m_dTeachExAxlePos;
				tCoord.dX -= m_dTeachExAxlePos_y;
#endif
			}
			fprintf(pf, "%d%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%4d%10d\n", nPtnIdx,
				tCoord.dX, tCoord.dY, tCoord.dZ, tCoord.dRX, tCoord.dRY, tCoord.dRZ, m_dTeachExAxlePos, m_dTeachExAxlePos_y, eWeldSeamType, vnAllPointType[nPtnIdx]);
		}
		fclose(pf);
	}
	return true;
}

bool WeldAfterMeasure::GenerateWeldLineData(const LineOrCircularArcWeldingLine& tWeldSeam, int nGroupNo, int nWeldNo, double dWeldHoleSize, double dBoardThick, std::vector<XI_POINT> vtScanPtns)
{
	double dChangeDis = 20.0;	// ����̬����
	double dChangeAngle = 30.0; // ����̬�Ƕ�
	double dPtnInterval = 1.0;  // �켣����
	int nChangePtnNum = (int)(dChangeDis / dPtnInterval); // ����̬����
	double dStepChangeAngle = dChangeAngle / (double)nChangePtnNum; // ���ڵ���̬�仯�Ƕ�
	int nDelPtnNum = (int)(dWeldHoleSize / dPtnInterval); // ������ɾ������

	XiAlgorithm alg;
	double dWeldDirAngle;
	E_WELD_SEAM_TYPE eWeldSeamType;
	vector<T_ROBOT_COORS> vtWeldCoord;

	// ��ȡ�������� ���ɺ��ӹ켣
	eWeldSeamType = GetWeldSeamType(tWeldSeam);
	double dPostureRx = E_FLAT_SEAM == eWeldSeamType ? m_dPlatWeldRx : m_dStandWeldRx;
	double dPostureRy = E_FLAT_SEAM == eWeldSeamType ? m_dPlatWeldRy : m_dStandWeldRy;
	dWeldDirAngle = alg.CalcArcAngle(tWeldSeam.StartNormalVector.x, tWeldSeam.StartNormalVector.y);
	if (false == GenerateWeldLineData(tWeldSeam.StartPoint, tWeldSeam.EndPoint, dPtnInterval, dPostureRx, dPostureRy, dWeldDirAngle, eWeldSeamType, vtWeldCoord))
	{
		return false;
	}

	int nWeldTrackPtnNum = vtWeldCoord.size();
	if (nWeldTrackPtnNum < 10 || nWeldTrackPtnNum < (nChangePtnNum/* * 2.0*/))
	{
		//���޸�
		XUI::MesBox::PopInfo("����켣����{0}���� �޷����ӣ�", nWeldTrackPtnNum);
		//XiMessageBox("����켣����%d���� �޷����ӣ�", nWeldTrackPtnNum);
		return false;
	}

	if (vtScanPtns.size() > 0) // ��ɨ������ ��������
	{
		CHECK_BOOL_RETURN(AdjustLineSeamTrack(eWeldSeamType, vtScanPtns, vtWeldCoord));
		nWeldTrackPtnNum = vtWeldCoord.size();
	}


	// ����˵����̬  ɾ�������׹켣
	if (E_FLAT_SEAM == eWeldSeamType && 0 < tWeldSeam.EndPointType) // ƽ�������յ� 
	{
		int nBaseIndex = nWeldTrackPtnNum - 1 - nChangePtnNum;
		double dSrcRz = vtWeldCoord[nBaseIndex].dRZ;
		for (int n = nWeldTrackPtnNum - nChangePtnNum; n < nWeldTrackPtnNum; n++)
		{
			vtWeldCoord[n].dRZ = dSrcRz - ((n - nBaseIndex) * dStepChangeAngle); // ��̬��С(���������Ұ�װ��ʽ���)
		}
		vtWeldCoord.erase(vtWeldCoord.end() - nDelPtnNum, vtWeldCoord.end()); // ɾ������
	}
	if (E_FLAT_SEAM == eWeldSeamType && 0 < tWeldSeam.StartPointType) // ƽ���������
	{
		double dSrcRz = vtWeldCoord[nChangePtnNum].dRZ;
		for (int n = nChangePtnNum - 1; n >= 0; n--)
		{
			vtWeldCoord[n].dRZ = dSrcRz + ((nChangePtnNum - n) * dStepChangeAngle); // ��̬����(���������Ұ�װ��ʽ���)
		}
		vtWeldCoord.erase(vtWeldCoord.begin(), vtWeldCoord.begin() + nDelPtnNum); // ɾ������
	}
	if (E_STAND_SEAM == eWeldSeamType)
	{
		double dSrcRy = vtWeldCoord[0].dRY;
		double dDstRy = 45.0;
		dStepChangeAngle = (dDstRy - dSrcRy) / (double)nChangePtnNum;
		for (int n = nChangePtnNum - 1; n >= 0; n--)
		{
			vtWeldCoord[n].dRY = dSrcRy + ((nChangePtnNum - n) * dStepChangeAngle);
		}
		vtWeldCoord.erase(vtWeldCoord.begin(), vtWeldCoord.begin() + nDelPtnNum); // ɾ������

	}

	// ������Ӱ��Ƕ���(��ʹ��ZSide�ж��Ƿ��ܰ��ǣ�
	vector<int> vnPointType;
	std::vector<T_WELD_PARA> vtWeldPara;
	int nWeldAngleSize = m_vvtWeldLineInfoGroup[nGroupNo][nWeldNo].tAtrribute.nWeldAngleSize;
	if (false == GetWeldParam(eWeldSeamType, nWeldAngleSize, vtWeldPara))
	{
		XiMessageBox("���غ��ӹ��ղ���ʧ�ܣ�");
		return false;
	}
	if (false == CalcWrapTrack(m_vvtWeldLineInfoGroup[nGroupNo][nWeldNo], eWeldSeamType, nWeldAngleSize / 2, vtWeldPara[0].nStandWeldDir, vtWeldCoord, vnPointType))
	{
		return false;
	}

	// ƽ�� �� ����(���)��� �� ����(�����)�յ� �� �ߵ����ӹ켣
	if ((E_FLAT_SEAM == eWeldSeamType) && (0 < tWeldSeam.StartPointType) && (false == tWeldSeam.EndPointType))
	{
		int nWeldTrackPtnNum = vtWeldCoord.size();
		for (int nPtnNo = 0; nPtnNo < nWeldTrackPtnNum / 2; nPtnNo++)
		{
			swap(vtWeldCoord[nPtnNo], vtWeldCoord[nWeldTrackPtnNum - 1 - nPtnNo]);
			swap(vnPointType[nPtnNo], vnPointType[nWeldTrackPtnNum - 1 - nPtnNo]);
		}
	}

	// ���� ��� ֱ������ �ⲿ������ ��������
	CString sFileName;
	sFileName.Format("%s%d_%d_RealWeldCoord.txt", m_sDataSavePath, nGroupNo, nWeldNo);
	FILE* pf = fopen(sFileName, "w");
	for (int nPtnIdx = 0; nPtnIdx < vtWeldCoord.size(); nPtnIdx++)
	{
		T_ROBOT_COORS& tCoord = vtWeldCoord[nPtnIdx];
		fprintf(pf, "%d%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%4d%10d\n", nPtnIdx,
			tCoord.dX, tCoord.dY, tCoord.dZ, tCoord.dRX, tCoord.dRY, tCoord.dRZ, m_dTeachExAxlePos, m_dTeachExAxlePos_y,eWeldSeamType, vnPointType[nPtnIdx]);
	}
	fclose(pf);

	return true;
}

bool WeldAfterMeasure::GenerateWeldLineData(CvPoint3D64f tStartPtn, CvPoint3D64f tEndPtn, double dInterval,
	double dRx, double dRy, double dWeldNorAngle, E_WELD_SEAM_TYPE eWeldSeamType, std::vector<T_ROBOT_COORS>& vtRobotCoors)
{
	vtRobotCoors.clear();
	T_ROBOT_COORS tRobotCoord;
	double dDisX = tEndPtn.x - tStartPtn.x;
	double dDisY = tEndPtn.y - tStartPtn.y;
	double dDisZ = tEndPtn.z - tStartPtn.z;
	// ������ʱ�뺸��ԭ���жϺ��ӷ���ͺ��ӷ����Ƿ�ƥ��
	XiAlgorithm alg;
	double dWeldDirAngle = alg.CalcArcAngle(dDisX, dDisY);
	if ((E_SCAN_WELD_LINE != m_eWorkPieceType))
	{
		if (E_FLAT_SEAM == eWeldSeamType && true != JudgeDirAngle(dWeldDirAngle - (90.0 * (double)m_nRobotInstallDir), dWeldNorAngle)) // ƽ�����
		{
			XiMessageBox("ƽ�����ӷ���ͷ���ǲ�������");
			vtRobotCoors.clear();
			return false;
		}
	}
	double dDis = TwoPointDis(tStartPtn.x, tStartPtn.y, tStartPtn.z, tEndPtn.x, tEndPtn.y, tEndPtn.z);

	int nPtnNum = dDis / dInterval;
	for (int i = 0; i <= nPtnNum; i++)
	{
		tRobotCoord.dX = tStartPtn.x + (dDisX * ((double)i / (double)nPtnNum));
		tRobotCoord.dY = tStartPtn.y + (dDisY * ((double)i / (double)nPtnNum));
		tRobotCoord.dZ = tStartPtn.z + (dDisZ * ((double)i / (double)nPtnNum));
		tRobotCoord.dRX = dRx;
		tRobotCoord.dRY = dRy;
		tRobotCoord.dRZ = DirAngleToRz(dWeldNorAngle);
		vtRobotCoors.push_back(tRobotCoord);
	}
	return true;
}

bool WeldAfterMeasure::GenerateWeldLineData(XI_POINT tStartPtn, XI_POINT tEndPtn, double dInterval,
	double dRx, double dRy, double dWeldNorAngle, E_WELD_SEAM_TYPE eWeldSeamType, std::vector<T_ROBOT_COORS>& vtRobotCoors)
{
	CvPoint3D64f tSPtn = { tStartPtn.x, tStartPtn.y, tStartPtn.z };
	CvPoint3D64f tEPtn = { tEndPtn.x, tEndPtn.y, tEndPtn.z };
	return GenerateWeldLineData(tSPtn, tEPtn, dInterval, dRx, dRy, dWeldNorAngle, eWeldSeamType, vtRobotCoors);
}

bool WeldAfterMeasure::GenerateWeldTrackArc(LineOrCircularArcWeldingLine tWeldLine)
{
	return true;
	//double dPtnInterval = 2.0;
	//double dRadius = TwoPointDis(tWeldLine.CenterPoint.x, tWeldLine.CenterPoint.y, tWeldLine.CenterPoint.z, tWeldLine.StartPoint.x, tWeldLine.StartPoint.y, tWeldLine.StartPoint.z);
	//int nPtnNum = 2.0 * PI * dRadius / 2.0;
	//double dAngleInterval = 360.0 / (double)nPtnNum;

	//T_SPACE_CIRCLE_PARAM tSpaceCircleParam;
	//tSpaceCircleParam.dRadius = dRadius;
	//tSpaceCircleParam.tCenterPoint.dCoorX = tWeldLine.CenterPoint.x;
	//tSpaceCircleParam.tCenterPoint.dCoorY = tWeldLine.CenterPoint.y;
	//tSpaceCircleParam.tCenterPoint.dCoorZ = tWeldLine.CenterPoint.z;
	//tSpaceCircleParam.tCircleNormalDir.dDirX = tWeldLine.StartNormalVector.x; // ��㷨���б��溸��ռ�Բ��ƽ�淨��
	//tSpaceCircleParam.tCircleNormalDir.dDirY = tWeldLine.StartNormalVector.y;
	//tSpaceCircleParam.tCircleNormalDir.dDirZ = tWeldLine.StartNormalVector.z;

}

void WeldAfterMeasure::SaveCoordToFile(std::vector<T_ROBOT_COORS> vtCoord, CString sFileName)
{
	FILE* pf = fopen(sFileName.GetBuffer(), "w");
	if (NULL == pf)
	{
		return;
	}
	for (int i = 0; i < vtCoord.size(); i++)
	{
		fprintf(pf, "%d%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf\n", i,
			vtCoord[i].dX, vtCoord[i].dY, vtCoord[i].dZ, vtCoord[i].dRX, vtCoord[i].dRY, vtCoord[i].dRZ);
	}
	fclose(pf);
}

bool WeldAfterMeasure::GeneralTeachResult(int nGroupNo, std::vector<T_ROBOT_COORS>& vtMeasureCoord, std::vector<T_ANGLE_PULSE>& vtMeasurePulse, vector<int>& vnMeasureType, double& dExAxlePos)
{
	if (E_BEVEL == m_eWorkPieceType)
	{
		return true;
	}
	double dOffsetDir = 0.0;
	double dStepDis = 5.0;
	int nOffsetPtnNum = 5;
	T_ROBOT_COORS tCoord;
	CString sFileName;
	sFileName.Format("%s%d-TeachResult.txt", m_sDataSavePath, nGroupNo);
	FILE* pf = fopen(sFileName.GetBuffer(), "w");
	int nTeachPtnNo = 0;
	for (int i = 0; i < vnMeasureType.size(); i++)
	{
		tCoord = vtMeasureCoord[i];
		if (E_TRANSITION_POINT & vnMeasureType[i])
		{
			continue;
		}
		T_ROBOT_COORS tKeyPtnCoord;
		if (false == m_pRobotDriver->MoveToolByWeldGun(tCoord, m_pRobotDriver->m_tTools.tCameraTool,
			tCoord, m_pRobotDriver->m_tTools.tGunTool, tKeyPtnCoord))
		{
			XiMessageBoxOk("�����������ת��ǹ����ʧ�ܣ�");
			return false;
		}
		// �ǵ�
		fprintf(pf, "%d%4d%11.3lf%11.3lf%11.3lf 0 0 0.0 0.0 0.0 0.0 0.0 0.0 0 0 0 0 0 0 0.0\n",
			nTeachPtnNo, 0, tKeyPtnCoord.dX, tKeyPtnCoord.dY, tKeyPtnCoord.dZ);

		if (E_LS_RL_FLIP & vnMeasureType[i]) // �������� �������
		{
			tCoord = tKeyPtnCoord;
			dOffsetDir = RzToDirAngle(tCoord.dRZ) + 180.0;
			for (int nPtnNo = 0; nPtnNo < nOffsetPtnNum; nPtnNo++)
			{
				RobotCoordPosOffset(tCoord, dOffsetDir, dStepDis);
				fprintf(pf, "%d%4d%11.3lf%11.3lf%11.3lf 0 0 0.0 0.0 0.0 0.0 0.0 0.0 0 0 0 0 0 0 0.0\n",
					nTeachPtnNo, 1, tCoord.dX, tCoord.dY, tCoord.dZ);
			}
			tCoord = tKeyPtnCoord;
			for (int nPtnNo = 0; nPtnNo < nOffsetPtnNum; nPtnNo++)
			{
				RobotCoordPosOffset(tCoord, dOffsetDir, 0.0, (double)(m_nRobotInstallDir) * (-dStepDis));
				fprintf(pf, "%d%4d%11.3lf%11.3lf%11.3lf 0 0 0.0 0.0 0.0 0.0 0.0 0.0 0 0 0 0 0 0 0.0\n",
					nTeachPtnNo, 1, tCoord.dX, tCoord.dY, tCoord.dZ);
			}

		}
		
		if (E_L_LINE_POINT & vnMeasureType[i] || E_DOUBLE_LONG_LINE & vnMeasureType[i]) // ��������
		{
			tCoord = tKeyPtnCoord;
			for (int nPtnNo = 0; nPtnNo < nOffsetPtnNum; nPtnNo++)
			{
				RobotCoordPosOffset(tCoord, dOffsetDir, 0.0, (double)(m_nRobotInstallDir) * dStepDis);
				fprintf(pf, "%d%4d%11.3lf%11.3lf%11.3lf 0 0 0.0 0.0 0.0 0.0 0.0 0.0 0 0 0 0 0 0 0.0\n",
					nTeachPtnNo, 1, tCoord.dX, tCoord.dY, tCoord.dZ);
			}
		}

		if (E_R_LINE_POINT & vnMeasureType[i] || E_DOUBLE_LONG_LINE & vnMeasureType[i]) // �װ��
		{
			tCoord = tKeyPtnCoord;
			dOffsetDir = RzToDirAngle(tCoord.dRZ);
			for (int nPtnNo = 0; nPtnNo < nOffsetPtnNum; nPtnNo++)
			{
				RobotCoordPosOffset(tCoord, dOffsetDir, dStepDis);
				fprintf(pf, "%d%4d%11.3lf%11.3lf%11.3lf 0 0 0.0 0.0 0.0 0.0 0.0 0.0 0 0 0 0 0 0 0.0\n",
					nTeachPtnNo, 2, tCoord.dX, tCoord.dY, tCoord.dZ);
			}
		}
		nTeachPtnNo++;
	}
	fclose(pf);
	return true;
}

void WeldAfterMeasure::Test()
{
	*m_pColorImg = cvLoadImage("./SteelStructure/Picture/Pro/1.jpg", 1);
}

void WeldAfterMeasure::GenerateJobSpotWeld(vector<T_ANGLE_PULSE> vtRobotPulse, int nMoveType, CString sJobName)
{
	int nPtnNum = vtRobotPulse.size();
	if (nPtnNum <= 0)
	{
		return;
	}

	CString sMoveCommand = nMoveType == MOVL ? "MOVL" : "MOVJ";
	CString sSpeedName = nMoveType == MOVL ? "V" : "VJ";
	CString sSavePathName;
	sSavePathName.Format(OUTPUT_PATH + m_ptUnit->GetUnitName() + "\\JOB\\%s.JBI", sJobName);
	FILE* pf = fopen(sSavePathName, "w");

	fprintf(pf, "/JOB\n");
	fprintf(pf, "//NAME " + sJobName + "\n");
	fprintf(pf, "//POS\n");
	fprintf(pf, "///NPOS %d,0,0,1,0,0\n", nPtnNum);
	fprintf(pf, "///TOOL 1\n");
	fprintf(pf, "///POSTYPE PULSE\n");
	fprintf(pf, "///PULSE\n");
	for (int i = 0; i < nPtnNum; i++)
	{
		fprintf(pf, "C%05d=%ld,%ld,%ld,%ld,%ld,%ld\n", i,
			vtRobotPulse[i].nSPulse, vtRobotPulse[i].nLPulse, vtRobotPulse[i].nUPulse,
			vtRobotPulse[i].nRPulse, vtRobotPulse[i].nBPulse, vtRobotPulse[i].nTPulse);
	}
	fprintf(pf, "//INST\n");
	fprintf(pf, "///DATE 2021/03/10 17:07\n");
	fprintf(pf, "///ATTR SC,RW\n");
	fprintf(pf, "///GROUP1 RB1\n");
	fprintf(pf, "NOP\n");
	fprintf(pf, "SET I003 0\n");
	for (int i = 0; i < nPtnNum; i++)
	{
		fprintf(pf, sMoveCommand + " C%05d " + sSpeedName + "=I004\n", i);
		fprintf(pf, "IFTHENEXP I003 = 1\n"); 
		fprintf(pf, "	 ARCON AC=I005 AV=I006\n"); 
		fprintf(pf, "	 TIMER T=I007\n"); 
		fprintf(pf, "	 ARCOF\n"); 
		fprintf(pf, "ELSE\n"); 
		fprintf(pf, "	 TIMER T=I007\n"); 
		fprintf(pf, "ENDIF\n"); 
	}
	fprintf(pf, "END\n");
	fclose(pf);
	SaveToJobBuffer(vtRobotPulse);
}

void WeldAfterMeasure::GenerateJobLocalVariable(vector<T_ANGLE_PULSE> vtRobotPulse, int nMoveType, CString sJobName, int nStep/* = 10*/)
{
	double dPulseLBY = 0.00766967710464; // ���Է���ʹ��ת��
	int nExAxleType = m_pRobotDriver->m_nExternalAxleType;

	if (nExAxleType)
	{

	}
	// ����
	int nReserveNum = 25;
	//int nStep =  10;
	int nSIdx = nReserveNum;
	int nEIdx = vtRobotPulse.size() - nReserveNum;
	if (vtRobotPulse.size() > 60)
	{
		vector<T_ANGLE_PULSE> vtPulse(vtRobotPulse.begin(), vtRobotPulse.end());
		vtRobotPulse.clear();
		vtRobotPulse.insert(vtRobotPulse.end(), vtPulse.begin(), vtPulse.begin() + nReserveNum);
		for (int i = nSIdx; i < nEIdx; i += nStep)
		{
			vtRobotPulse.push_back(vtPulse[i]);
		}
		vtRobotPulse.insert(vtRobotPulse.end(), vtPulse.end() - nReserveNum, vtPulse.end());
	}
	if (MOVL == nMoveType)
	{
		if (vtRobotPulse.size() < 1)
		{
			return;
		}
		T_ANGLE_PULSE tPulse = vtRobotPulse[0];
		vtRobotPulse.insert(vtRobotPulse.begin(), tPulse);
	}
	int nPtnNum = vtRobotPulse.size();
	if (nPtnNum <= 0)
	{
		return;
	}

	CString sMoveCommand = nMoveType == MOVL ? "MOVL" : "MOVJ";
	CString sSpeedName = nMoveType == MOVL ? "V" : "VJ";
	CString sSavePathName;
	sSavePathName.Format(OUTPUT_PATH + m_ptUnit->GetUnitName() + "\\JOB\\%s.JBI", sJobName);
	FILE* pf = fopen(sSavePathName, "w");

	fprintf(pf, "/JOB\n");
	fprintf(pf, "//NAME " + sJobName + "\n");
	fprintf(pf, "//POS\n");
	fprintf(pf, "///NPOS %d,%d,0,1,0,0\n", nPtnNum, nExAxleType > 0 ? nPtnNum : 0);
	fprintf(pf, "///TOOL 1\n");
	fprintf(pf, "///POSTYPE PULSE\n");
	fprintf(pf, "///PULSE\n");
	for (int i = 0; i < nPtnNum; i++)
	{
		fprintf(pf, "C%05d=%ld,%ld,%ld,%ld,%ld,%ld\n", i,
			vtRobotPulse[i].nSPulse, vtRobotPulse[i].nLPulse, vtRobotPulse[i].nUPulse,
			vtRobotPulse[i].nRPulse, vtRobotPulse[i].nBPulse, vtRobotPulse[i].nTPulse);
	}

	if (nExAxleType > 0) // �˴���Ҫ�޸ģ�֧��ͬ��ʽ�ⲿ������
	{
		fprintf(pf, "///TOOL 0\n");
		for (int i = 0; i < nPtnNum; i++)
		{
			fprintf(pf, "BC%05d=%d,%d\n", i,
				(long)((double)vtRobotPulse[i].lBXPulse * m_pRobotDriver->m_tExternalAxle[0].dPulse / dPulseLBY),
				(long)((double)vtRobotPulse[i].lBYPulse * m_pRobotDriver->m_tExternalAxle[1].dPulse / dPulseLBY));
		}
	}

	fprintf(pf, "//INST\n");
	fprintf(pf, "///DATE 2021/03/10 17:07\n");
	fprintf(pf, "///ATTR SC,RW\n");
	fprintf(pf, "///GROUP1 RB1%s\n", nExAxleType > 0 ? ",BS1" : "");
	fprintf(pf, "NOP\n");
	for (int i = 0; i < nPtnNum; i++)
	{
		if (0 == i && MOVL == nMoveType)
		{
			if (nExAxleType > 0)
			{
				fprintf(pf, "MOVJ C%05d BC%05d VJ=I005\n", i, i++);
			}
			else
			{
				fprintf(pf, "MOVJ C%05d VJ=I005\n", i++);
			}
		}
		if (nExAxleType > 0)
		{
			fprintf(pf, sMoveCommand + " C%05d BC%05d " + sSpeedName + "=I005\n", i, i);
		}
		else
		{
			fprintf(pf, sMoveCommand + " C%05d " + sSpeedName + "=I005\n", i);
		}
	}
	fprintf(pf, "END\n");
	fclose(pf);
	SaveToJobBuffer(vtRobotPulse);
}

void WeldAfterMeasure::GenerateJobCoord(vector<T_ROBOT_COORS> vtRobotCoord, int nMoveType, CString sJobName)
{
	int nMaxPosVar = 60;
	int nReserveNum = 21;
	int nStep = 0;
	int nPtnNum = vtRobotCoord.size();
	if (nPtnNum <= 0)
	{
		return;
	}
	vector<int> vnPosIdx;
	vnPosIdx.clear(); 
	for (int i = 0; i < nPtnNum; )
	{
		vnPosIdx.push_back(i);
		if (nPtnNum > nMaxPosVar && i > nReserveNum && i < nPtnNum - nReserveNum)
		{
			nStep = (nPtnNum - (2 * nReserveNum)) / (nMaxPosVar - (2 * nReserveNum));
			i = (i + nStep < nPtnNum - nReserveNum) ? i + nStep : i + 1;
		}
		else
		{
			i++;
		}
	}
	nPtnNum = vnPosIdx.size();

	CString sMoveCommand = nMoveType == MOVL ? "MOVL" : "MOVJ";
	CString sSpeedName = nMoveType == MOVL ? "V" : "VJ";
	CString sSavePathName;
	sSavePathName.Format(OUTPUT_PATH + m_ptUnit->GetUnitName() + "\\JOB\\%s.JBI", sJobName);
	FILE* pf = fopen(sSavePathName, "w");

	fprintf(pf, "/JOB\n");
	fprintf(pf, "//NAME " + sJobName + "\n");
	fprintf(pf, "//POS\n");
	fprintf(pf, "///NPOS 0,0,0,%d,0,0\n", nPtnNum);
	fprintf(pf, "///TOOL 1\n");
	fprintf(pf, "///POSTYPE ROBOT\n");
	fprintf(pf, "///RECTAN\n");
	fprintf(pf, "///RCONF 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0\n");
	for (int i = 0; i < nPtnNum; i++)
	{
		fprintf(pf, "P%05d=%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf\n", i,
			vtRobotCoord[vnPosIdx[i]].dX, vtRobotCoord[vnPosIdx[i]].dY, vtRobotCoord[vnPosIdx[i]].dZ,
			vtRobotCoord[vnPosIdx[i]].dRX, vtRobotCoord[vnPosIdx[i]].dRY, vtRobotCoord[vnPosIdx[i]].dRZ);
	}
	fprintf(pf, "//INST\n");
	fprintf(pf, "///DATE 2021/03/10 17:07\n");
	fprintf(pf, "///ATTR SC,RW\n");
	fprintf(pf, "///GROUP1 RB1\n");
	fprintf(pf, "NOP\n");
	for (int i = 0; i < nPtnNum; i++)
	{
		fprintf(pf, sMoveCommand + " P%03d " + sSpeedName + "=I005\n", i);
	}
	fprintf(pf, "END\n");
	fclose(pf);
}

void WeldAfterMeasure::GenerateJobForContinueWeld(int nGroupNo, bool bUsedExAxle/* = false*/)
{
	double dPulseLBY = 0.00766967710464; // ���Է���ʹ��ת��
	double dPulseLBZ = 0.00766967685128; // ���Է���ʹ��ת��
	int nTrackNum = m_vvtJobPulse.size();
	int nPtnNum = 0;
	for (int i = 0; i < nTrackNum; i++)
	{
		nPtnNum += m_vvtJobPulse[i].size(); // ÿ��켣��һ���� ��MOVJ ��MOVL
	}

	if (nPtnNum <= 0)
	{
		return;
	}

	CString sJobName;
	sJobName.Format("CONTINUE-WELD-%d%s", nGroupNo, bUsedExAxle ? "-BP" : "");
	CString sSavePathName = OUTPUT_PATH + m_ptUnit->GetUnitName() + "\\JOB\\" + sJobName + ".JBI";
	FILE* pf = fopen(sSavePathName, "w");

	fprintf(pf, "/JOB\n");
	fprintf(pf, "//NAME " + sJobName + "\n");
	fprintf(pf, "//POS\n");
	fprintf(pf, "///NPOS %d,%d,0,1,0,0\n", nPtnNum + 2, bUsedExAxle ? nPtnNum + 2 : 0); // �����ǹ����ǹ��������
	fprintf(pf, "///TOOL 1\n");
	fprintf(pf, "///POSTYPE PULSE\n");
	fprintf(pf, "///PULSE\n");

	int nTotalPtnNo = 0;
	T_ANGLE_PULSE tPulse = m_pRobotDriver->m_tHomePulse;
	fprintf(pf, "C%05d=%ld,%ld,%ld,%ld,%ld,%ld\n", nTotalPtnNo++,
		tPulse.nSPulse, tPulse.nLPulse, tPulse.nUPulse,
		tPulse.nRPulse, tPulse.nBPulse, tPulse.nTPulse); // ��ǹ��ȫλ��
	for (int nTrackNo = 0; nTrackNo < nTrackNum; nTrackNo++)
	{
		for (int nPtnNo = 0; nPtnNo < m_vvtJobPulse[nTrackNo].size(); nPtnNo++)
		{
			tPulse = m_vvtJobPulse[nTrackNo][nPtnNo];
			fprintf(pf, "C%05d=%ld,%ld,%ld,%ld,%ld,%ld\n", nTotalPtnNo++,
				tPulse.nSPulse, tPulse.nLPulse, tPulse.nUPulse,
				tPulse.nRPulse, tPulse.nBPulse, tPulse.nTPulse);
		}
	}
	tPulse = m_pRobotDriver->m_tHomePulse;
	fprintf(pf, "C%05d=%ld,%ld,%ld,%ld,%ld,%ld\n", nTotalPtnNo++,
		tPulse.nSPulse, tPulse.nLPulse, tPulse.nUPulse,
		tPulse.nRPulse, tPulse.nBPulse, tPulse.nTPulse); // ��ǹ��ȫλ��

	if (bUsedExAxle)
	{

		int nExIdx = 0;
		fprintf(pf, "///TOOL 0\n");
		fprintf(pf, "BC%05d=%d,%d\n", nExIdx++,
			(long)((double)m_vvtJobPulse.front().front().lBXPulse * m_pRobotDriver->m_tExternalAxle[0].dPulse / dPulseLBY),
			(long)((double)m_vvtJobPulse.front().front().lBYPulse * m_pRobotDriver->m_tExternalAxle[1].dPulse / dPulseLBY));
		for (int nTrackNo = 0; nTrackNo < nTrackNum; nTrackNo++)
		{
			for (int nPtnNo = 0; nPtnNo < m_vvtJobPulse[nTrackNo].size(); nPtnNo++)
			{
				tPulse = m_vvtJobPulse[nTrackNo][nPtnNo];
				fprintf(pf, "BC%05d=%d,%d\n", nExIdx++,
					(long)((double)tPulse.lBXPulse * m_pRobotDriver->m_tExternalAxle[0].dPulse / dPulseLBY),
					(long)((double)tPulse.lBYPulse * m_pRobotDriver->m_tExternalAxle[1].dPulse / dPulseLBY));
			}
		}
		fprintf(pf, "BC%05d=%d,%d\n", nExIdx++,
			(long)((double)m_vvtJobPulse.back().back().lBXPulse * m_pRobotDriver->m_tExternalAxle[0].dPulse / dPulseLBY),
			(long)((double)m_vvtJobPulse.back().back().lBYPulse * m_pRobotDriver->m_tExternalAxle[1].dPulse / dPulseLBY));
	}

	fprintf(pf, "//INST\n");
	fprintf(pf, "///DATE 2021/03/10 17:07\n");
	fprintf(pf, "///ATTR SC,RW\n");
	fprintf(pf, "///GROUP1 RB1%s\n", bUsedExAxle ? ",BS1" : "");
	fprintf(pf, "NOP\n");
	nTotalPtnNo = 0;
	//fprintf(pf, "MOVJ C%05d VJ=I005\n", nTotalPtnNo++); // ��ǹǰ�ذ�ȫλ��
	if (bUsedExAxle)
	{
		fprintf(pf, "MOVJ C%05d BC%05d VJ=I005\n", nTotalPtnNo, nTotalPtnNo++); // ��ǹǰ�ذ�ȫλ��
	}
	else
	{
		fprintf(pf, "MOVJ C%05d VJ=I005\n", nTotalPtnNo++); // ��ǹǰ�ذ�ȫλ��
	}

	for (int nTrackNo = 0; nTrackNo < nTrackNum; nTrackNo++)
	{
		for (int nPtnNo = 0; nPtnNo < m_vvtJobPulse[nTrackNo].size(); nPtnNo++)
		{
			tPulse = m_vvtJobPulse[nTrackNo][nPtnNo];
			if (0 == nPtnNo || 1 == nPtnNo || 2 == nPtnNo)
			{
				//fprintf(pf, "MOVJ C%05d VJ=I005\n", nTotalPtnNo++);
				if (bUsedExAxle)
				{
					fprintf(pf, "MOVJ C%05d BC%05d VJ=I005\n", nTotalPtnNo, nTotalPtnNo++);
				}
				else
				{
					fprintf(pf, "MOVJ C%05d VJ=I005\n", nTotalPtnNo++);
				}
			}
			else
			{
				if (bUsedExAxle)
				{
					fprintf(pf, "MOVL C%05d BC%05d V=I005\n", nTotalPtnNo, nTotalPtnNo++);
				}
				else
				{
					fprintf(pf, "MOVL C%05d V=I005\n", nTotalPtnNo++);
				}
			}
		}
	}
	//fprintf(pf, "MOVJ C%05d VJ=I005\n", nTotalPtnNo++); // ��ǹ�ذ�ȫλ��
	if (bUsedExAxle)
	{
		fprintf(pf, "MOVJ C%05d BC%05d VJ=I005\n", nTotalPtnNo, nTotalPtnNo++); // ��ǹ�ذ�ȫλ��
	}
	else
	{
		fprintf(pf, "MOVJ C%05d VJ=I005\n", nTotalPtnNo++); // ��ǹ�ذ�ȫλ��
	}
	fprintf(pf, "END\n");
	fclose(pf);
}

//void WeldAfterMeasure::GenerateFilePLYPlane(double dExAxisPos, CString sName/* = "AllWeldSeamPlane"*/)
//{
//	WeldLineInfo tWeldLineInfo;
//	std::vector<WeldLineInfo> vtWeldSeamPlane(0);
//	for (int i = 0; i < m_vtWeldSeamInfo.size(); i++)
//	{
//		if (!IsStandWeldSeam(m_vtWeldSeamInfo[i].tWeldLine.StartNormalVector))
//		{
//			tWeldLineInfo = m_vtWeldSeamInfo[i];
//#ifdef SINGLE_ROBOT
//			tWeldLineInfo.tWeldLine.StartPoint.y -= dExAxisPos;
//			tWeldLineInfo.tWeldLine.EndPoint.y -= dExAxisPos;
//#else
//			tWeldLineInfo.tWeldLine.StartPoint.x -= dExAxisPos;
//			tWeldLineInfo.tWeldLine.EndPoint.x -= dExAxisPos;
//#endif // SINGLE_ROBOT
//			vtWeldSeamPlane.push_back(tWeldLineInfo);
//		}
//	}
//
//	int nWeldNum = vtWeldSeamPlane.size();
//	CvPoint3D64f tPtn;
//	vector<CvPoint3D64f> vtPtns(0);
//	vector<vector<CvPoint3D64f>> vvtPtns(0);
//	m_vtPartPlaneRects.resize(nWeldNum);
//	for (int nWeldNo = 0; nWeldNo < nWeldNum; nWeldNo++)
//	{
//		vtPtns.clear();
//		vtPtns.push_back(vtWeldSeamPlane[nWeldNo].tWeldLine.StartPoint);
//		m_vtPartPlaneRects[nWeldNo].tLeftDown.dCoorX = vtWeldSeamPlane[nWeldNo].tWeldLine.StartPoint.x;
//		m_vtPartPlaneRects[nWeldNo].tLeftDown.dCoorY = vtWeldSeamPlane[nWeldNo].tWeldLine.StartPoint.y;
//		m_vtPartPlaneRects[nWeldNo].tLeftDown.dCoorZ = vtWeldSeamPlane[nWeldNo].tWeldLine.StartPoint.z;
//		vtPtns.push_back(vtWeldSeamPlane[nWeldNo].tWeldLine.EndPoint);
//		m_vtPartPlaneRects[nWeldNo].tRightDown.dCoorX = vtWeldSeamPlane[nWeldNo].tWeldLine.EndPoint.x;
//		m_vtPartPlaneRects[nWeldNo].tRightDown.dCoorY = vtWeldSeamPlane[nWeldNo].tWeldLine.EndPoint.y;
//		m_vtPartPlaneRects[nWeldNo].tRightDown.dCoorZ = vtWeldSeamPlane[nWeldNo].tWeldLine.EndPoint.z;
//		tPtn = vtWeldSeamPlane[nWeldNo].tWeldLine.EndPoint;
//		tPtn.z = vtWeldSeamPlane[nWeldNo].tWeldLine.ZSide;
//		vtPtns.push_back(tPtn);
//		m_vtPartPlaneRects[nWeldNo].tRightUp.dCoorX = tPtn.x;
//		m_vtPartPlaneRects[nWeldNo].tRightUp.dCoorY = tPtn.y;
//		m_vtPartPlaneRects[nWeldNo].tRightUp.dCoorZ = tPtn.z;
//		tPtn = vtWeldSeamPlane[nWeldNo].tWeldLine.StartPoint;
//		tPtn.z = vtWeldSeamPlane[nWeldNo].tWeldLine.ZSide;
//		vtPtns.push_back(tPtn);
//		m_vtPartPlaneRects[nWeldNo].tLeftUp.dCoorX = tPtn.x;
//		m_vtPartPlaneRects[nWeldNo].tLeftUp.dCoorY = tPtn.y;
//		m_vtPartPlaneRects[nWeldNo].tLeftUp.dCoorZ = tPtn.z;
//		vvtPtns.push_back(vtPtns);
//	}
//
//	// ÿ��ƽ����Ӧһ��ƽ�� �ĸ��˵� �ĸ�ƽ��
//	int nPlaneNum = vvtPtns.size();
//	CString sFileName;
//	CheckFolder(OUTPUT_PATH + m_ptUnit->m_tContralUnit.strUnitName + "\\Collide\\");
//	DelFiles(OUTPUT_PATH + m_ptUnit->m_tContralUnit.strUnitName + "\\Collide\\");
//	sFileName.Format(OUTPUT_PATH + m_ptUnit->m_tContralUnit.strUnitName + "\\Collide\\AA_%s.ply", sName);
//	FILE* pf = fopen(sFileName.GetBuffer(), "w");
//	fprintf(pf, "ply\n");
//	fprintf(pf, "format ascii 1.0\n");
//	fprintf(pf, "comment zipper output\n");
//	fprintf(pf, "element vertex %d\n", nPlaneNum * 4);
//	fprintf(pf, "property float x\n");
//	fprintf(pf, "property float y\n");
//	fprintf(pf, "property float z\n");
//	fprintf(pf, "property float confidence\n");
//	fprintf(pf, "property float intensity\n");
//	fprintf(pf, "element face %d\n", nPlaneNum * 4);
//	fprintf(pf, "property list uchar int vertex_indices\n");
//	fprintf(pf, "end_header\n");
//
//	// Ptn
//	for (int nPlaneNo = 0; nPlaneNo < nPlaneNum; nPlaneNo++)
//	{
//		vtPtns = vvtPtns[nPlaneNo];
//		for (int nPtnNo = 0; nPtnNo < vtPtns.size(); nPtnNo++)
//		{
//			fprintf(pf, "%.3lf %.3lf %.3lf 0.1 0.1\n", vtPtns[nPtnNo].x, vtPtns[nPtnNo].y, vtPtns[nPtnNo].z);
//		}
//	}
//	//plane
//
//	for (int i = 0; i < vvtPtns.size(); i++)
//	{
//		fprintf(pf, "3 %d %d %d\n", i * 4 + 0, i * 4 + 1, i * 4 + 2);
//		fprintf(pf, "3 %d %d %d\n", i * 4 + 1, i * 4 + 2, i * 4 + 3);
//		fprintf(pf, "3 %d %d %d\n", i * 4 + 2, i * 4 + 3, i * 4 + 0);
//		fprintf(pf, "3 %d %d %d\n", i * 4 + 3, i * 4 + 0, i * 4 + 1);
//	}
//	fclose(pf);
//}

void WeldAfterMeasure::GenerateFilePLYPlane(vector<WeldLineInfo> vtWeldSeamPlane, CString sName/* = "AllWeldSeamPlane"*/)
{
	m_vvtWeldSeamGroup;
	m_vvtWeldSeamGroupAdjust;
	CvPoint3D64f tPtn;
	vector<CvPoint3D64f> vtPtns(0);
	vector<vector<CvPoint3D64f>> vvtPtns(0);
	for (int nWeldNo = 0; nWeldNo < vtWeldSeamPlane.size(); nWeldNo++)
	{
		vtPtns.clear();
		vtPtns.push_back(vtWeldSeamPlane[nWeldNo].tWeldLine.StartPoint);
		vtPtns.push_back(vtWeldSeamPlane[nWeldNo].tWeldLine.EndPoint);
		tPtn = vtWeldSeamPlane[nWeldNo].tWeldLine.EndPoint;
		tPtn.z = vtWeldSeamPlane[nWeldNo].tWeldLine.ZSide;
		vtPtns.push_back(tPtn);
		tPtn = vtWeldSeamPlane[nWeldNo].tWeldLine.StartPoint;
		tPtn.z = vtWeldSeamPlane[nWeldNo].tWeldLine.ZSide;
		vtPtns.push_back(tPtn);
		vvtPtns.push_back(vtPtns);
	}

	// ÿ��ƽ����Ӧһ��ƽ�� �ĸ��˵� �ĸ�ƽ��
	int nPlaneNum = vvtPtns.size();
	CString sFileName;
	CheckFolder(OUTPUT_PATH + m_ptUnit->GetUnitName() + "\\JOB");
	sFileName.Format(OUTPUT_PATH + m_ptUnit->GetUnitName() + "\\JOB\\AA_%s.ply", sName);
	FILE* pf = fopen(sFileName.GetBuffer(), "w");
	fprintf(pf, "ply\n");
	fprintf(pf, "format ascii 1.0\n");
	fprintf(pf, "comment zipper output\n");
	fprintf(pf, "element vertex %d\n", nPlaneNum * 4);
	fprintf(pf, "property float x\n");
	fprintf(pf, "property float y\n");
	fprintf(pf, "property float z\n");
	fprintf(pf, "property float confidence\n");
	fprintf(pf, "property float intensity\n");
	fprintf(pf, "element face %d\n", nPlaneNum * 4);
	fprintf(pf, "property list uchar int vertex_indices\n");
	fprintf(pf, "end_header\n");

	// Ptn
	for (int nPlaneNo = 0; nPlaneNo < nPlaneNum; nPlaneNo++)
	{
		vtPtns = vvtPtns[nPlaneNo];
		for (int nPtnNo = 0; nPtnNo < vtPtns.size(); nPtnNo++)
		{
			fprintf(pf, "%.3lf %.3lf %.3lf 0.1 0.1\n", vtPtns[nPtnNo].x, vtPtns[nPtnNo].y, vtPtns[nPtnNo].z);
		}
	}
	//plane

	for (int i = 0; i < vvtPtns.size(); i++)
	{
		fprintf(pf, "3 %d %d %d\n", i * 4 + 0, i * 4 + 1, i * 4 + 2);
		fprintf(pf, "3 %d %d %d\n", i * 4 + 1, i * 4 + 2, i * 4 + 3);
		fprintf(pf, "3 %d %d %d\n", i * 4 + 2, i * 4 + 3, i * 4 + 0);
		fprintf(pf, "3 %d %d %d\n", i * 4 + 3, i * 4 + 0, i * 4 + 1);
	}
	fclose(pf);
}

void WeldAfterMeasure::GenerateFilePLY(int nGroupNo, double dExAxlePos/* = 0.0*/, CString sName/* = "Data\\JOB\\PointCloud.ply"*/)
{
	m_vvtWeldSeamGroup;
	m_vvtWeldSeamGroupAdjust; 
	CvPoint3D64f tPtn;
	vector<CvPoint3D64f> vtPtns;
	vtPtns.clear();
	for (int nGroupNo = 0; nGroupNo < m_vvtWeldSeamGroup.size(); nGroupNo++)
	{
		for (int nWeldNo = 0; nWeldNo < m_vvtWeldSeamGroup[nGroupNo].size(); nWeldNo++)
		{
			vtPtns.push_back(m_vvtWeldSeamGroup[nGroupNo][nWeldNo].StartPoint);
			vtPtns.push_back(m_vvtWeldSeamGroup[nGroupNo][nWeldNo].EndPoint);
			tPtn = m_vvtWeldSeamGroup[nGroupNo][nWeldNo].EndPoint;
			tPtn.x += 0.1;
			tPtn.y += 0.1;
			tPtn.z += 0.1;
			vtPtns.push_back(tPtn);
		}
	}

	int nNum = vtPtns.size();
	CString sFileName;
	sFileName.Format(OUTPUT_PATH + m_ptUnit->GetUnitName() + "\\JOB\\Group_%d_AllWeldSeam.ply", nGroupNo);
	FILE* pf = fopen(sFileName.GetBuffer(), "w");
	fprintf(pf, "ply\n");
	fprintf(pf, "format ascii 1.0\n");
	fprintf(pf, "comment zipper output\n");
	fprintf(pf, "element vertex %d\n", nNum);
	fprintf(pf, "property float x\n");
	fprintf(pf, "property float y\n");
	fprintf(pf, "property float z\n");
	fprintf(pf, "property float confidence\n");
	fprintf(pf, "property float intensity\n");
	fprintf(pf, "element face %d\n", nNum / 3);
	fprintf(pf, "property list uchar int vertex_indices\n");
	fprintf(pf, "end_header\n");
	for (int i = 0; i < vtPtns.size(); i++)
	{
#ifdef SINGLE_ROBOT
		fprintf(pf, "%.3lf %.3lf %.3lf 0.1 0.1\n", vtPtns[i].x, vtPtns[i].y - dExAxlePos, vtPtns[i].z);
#else
		fprintf(pf, "%.3lf %.3lf %.3lf 0.1 0.1\n", vtPtns[i].x - dExAxlePos, vtPtns[i].y, vtPtns[i].z);
#endif // SINGLE_ROBOT
	}
	for (int i = 0; i < vtPtns.size(); i += 3)
	{
		fprintf(pf, "3 %d %d %d\n", i, i + 1, i + 2);
	}
	fclose(pf);
}

void WeldAfterMeasure::GenerateFilePLY(CvPoint3D64f* pPointCloud, int PointCloudSize, CString sFileName, 
	int nSample/* = 2*/, double dExAxlePos/* = 0.0*/)
{
	sFileName = OUTPUT_PATH + m_ptUnit->GetUnitName() + "\\JOB\\PointCloud.ply";
	int nNum = PointCloudSize / nSample;
	FILE* pf = fopen(sFileName.GetBuffer(), "w");
	fprintf(pf, "ply\n");
	fprintf(pf, "format ascii 1.0\n");
	fprintf(pf, "comment zipper output\n");
	fprintf(pf, "element vertex %d\n", nNum);
	fprintf(pf, "property float x\n");
	fprintf(pf, "property float y\n");
	fprintf(pf, "property float z\n");
	fprintf(pf, "property float confidence\n");
	fprintf(pf, "property float intensity\n");
	fprintf(pf, "element face 1\n");
	fprintf(pf, "property list uchar int vertex_indices\n");
	fprintf(pf, "end_header\n");
	for (int i = 0; i < nNum; i++)
	{
		fprintf(pf, "%.3lf %.3lf %.3lf 0.1 0.1\n", pPointCloud[i * nSample].x, pPointCloud[i * nSample].y - dExAxlePos, pPointCloud[i * nSample].z);
	}
	fprintf(pf, "3 0 %d %d\n", nNum / 2, nNum - 1);
	fclose(pf);
}

void WeldAfterMeasure::SaveToJobBuffer(const vector<T_ANGLE_PULSE>& vtPulse)
{
	m_vvtJobPulse.push_back(vtPulse);
}

void WeldAfterMeasure::CleaeJobBuffer()
{
	m_vvtJobPulse.clear();
}

void WeldAfterMeasure::PointCloudRotate180(vector<CvPoint3D64f> &vtPtns, double dCenX, double dCenY)
{
	int nPtnNum = vtPtns.size();
	if (nPtnNum < 2000)
	{
		return;
	}

	CvPoint3D64f tPtn;
	FILE* pf = fopen("GraphData\\LeftPointAbsCoordInBase-Rotate.txt", "w");
	for (int nPtnNo = 0; nPtnNo < nPtnNum; nPtnNo++)
	{
		tPtn = vtPtns[nPtnNo];
		tPtn.x = dCenX + (dCenX - tPtn.x);
		tPtn.y = dCenY + (dCenY - tPtn.y);
		vtPtns[nPtnNo] = tPtn;
		fprintf(pf, "0 %11.3lf%11.3lf%11.3lf 0 0\n", tPtn.x, tPtn.y, tPtn.z);
	}
	fclose(pf);
}

void WeldAfterMeasure::TeachResultRotate180(double dCenX, double dCenY)
{
	vector<T_TEACH_RESULT>& vtTeachResult = m_vtTeachResult;
	int nTeachPosNum = vtTeachResult.size();
	if (nTeachPosNum < 1)
	{
		return;
	}
	for (int n = 0; n < nTeachPosNum; n++)
	{
		T_TEACH_RESULT &tTeachResult = vtTeachResult[n];
		XI_POINT &tPtn = tTeachResult.tKeyPtn3D;
		tPtn.x = dCenX + (dCenX - tPtn.x);
		tPtn.y = dCenY + (dCenY - tPtn.y);
		for (int i = 0; i < tTeachResult.vtLeftPtns3D.size(); i++)
		{
			XI_POINT& tPtn = tTeachResult.vtLeftPtns3D[i];
			tPtn.x = dCenX + (dCenX - tPtn.x);
			tPtn.y = dCenY + (dCenY - tPtn.y);
		}
		for (int i = 0; i < tTeachResult.vtRightPtns3D.size(); i++)
		{
			XI_POINT& tPtn = tTeachResult.vtRightPtns3D[i];
			tPtn.x = dCenX + (dCenX - tPtn.x);
			tPtn.y = dCenY + (dCenY - tPtn.y);
		}
	}
}

bool WeldAfterMeasure::GenerateTemplate()
{
	// ��ʾ�����ģ������ ��׼����ģ������ + _��׼��.ini
	// �򿪽��������ʾ���Ƽ�������Ϣ �� �򿪻�׼���ļ�.ini (�޸ĺ���뱣��)

	TCHAR szFilePath[MAX_PATH + 1];
	GetModuleFileName(NULL, szFilePath, MAX_PATH);
	(_tcsrchr(szFilePath, _T('\\')))[1 - 3] = 0; //ɾ���ļ�����ֻ���·��
	CString sDefaultPath = CString(szFilePath) + OUTPUT_PATH + m_ptUnit->m_tContralUnit.strUnitName + "\\";
	CString sTemplateFile = SaveFileDlg(NULL, "txt", "����ģ��", sDefaultPath, "��ѡ�񱣴�ģ��·�����ļ���");
	CString sTemplateIniFile = sTemplateFile.Left(sTemplateFile.ReverseFind('.')) + "_��׼��.ini";
	if (sTemplateFile.IsEmpty()) return false;
	if (!CheckFileExists(sTemplateIniFile, true))
	{
		COPini opini;
		opini.SetFileName(sTemplateIniFile);
		opini.SetSectionName("BasePoints");
		opini.WriteString("PtnX0", 0.0);
		opini.WriteString("PtnY0", 0.0);
		opini.WriteString("PtnZ0", 0.0);
		opini.WriteString("PtnX1", 0.0);
		opini.WriteString("PtnY1", 0.0);
		opini.WriteString("PtnZ1", 0.0);
		opini.WriteString("PtnX2", 0.0);
		opini.WriteString("PtnY2", 0.0);
		opini.WriteString("PtnZ2", 0.0);
		opini.WriteString("PtnX3", 0.0);
		opini.WriteString("PtnY3", 0.0);
		opini.WriteString("PtnZ3", 0.0);
		opini.SetSectionName("TargetPoints");
		opini.WriteString("PtnX0", 0.0);
		opini.WriteString("PtnY0", 0.0);
		opini.WriteString("PtnZ0", 0.0);
		opini.WriteString("PtnX1", 0.0);
		opini.WriteString("PtnY1", 0.0);
		opini.WriteString("PtnZ1", 0.0);
		opini.WriteString("PtnX2", 0.0);
		opini.WriteString("PtnY2", 0.0);
		opini.WriteString("PtnZ2", 0.0);
		opini.WriteString("PtnX3", 0.0);
		opini.WriteString("PtnY3", 0.0);
		opini.WriteString("PtnZ3", 0.0);
	}

	CString sCommand = "C:\\Windows\\SysWOW64\\notepad.exe " + sTemplateIniFile;
	WinExec(sCommand.GetBuffer(), SW_SHOWNORMAL);
	RunInteractiveWindow(m_ptUnit->m_tContralUnit.strUnitName);

	CString saveFileName = OUTPUT_PATH + m_ptUnit->m_tContralUnit.strUnitName + "\\" + IDENTIFY_RESULT_SAVE;
	CopyFile(saveFileName, sTemplateFile, false);
	XUI::MesBox::PopInfo("ģ��������ɣ�ע����ȷд��ͱ����ļ�:\n{0}", sTemplateIniFile);
	return true;
}

void WeldAfterMeasure::TemplateMatch()
{
	TCHAR szFilePath[MAX_PATH + 1];
	GetModuleFileName(NULL, szFilePath, MAX_PATH);
	(_tcsrchr(szFilePath, _T('\\')))[1 - 3] = 0; //ɾ���ļ�����ֻ���·��

	CString sTemplateFile;
	CString sTemplateIniFile;
	CString sDefaultPath = CString(szFilePath)+ OUTPUT_PATH + m_ptUnit->m_tContralUnit.strUnitName + "\\";
	sTemplateFile = OpenFileDlg(NULL, _T("txt"), sDefaultPath, "��ģ���ļ�");
	sTemplateIniFile = sTemplateFile.Left(sTemplateFile.ReverseFind('.')) + "_��׼��.ini";

	if (!CheckFileExists(sTemplateFile) || !CheckFileExists(sTemplateIniFile))
	{
		XiMessageBoxOk("ģ����������ݲ����ڣ�");
		return;
	}

	CString sCommand = "C:\\Windows\\SysWOW64\\notepad.exe " + sTemplateIniFile;
	WinExec(sCommand.GetBuffer(), SW_SHOWNORMAL);
	RunInteractiveWindow(m_ptUnit->m_tContralUnit.strUnitName);
	if (1 != XiMessageBox("ȷ���ο�����д�룬���ڱ���ģ��ʱ���λ�ü�˳����ͬ����ʼƥ�䣿"))
	{
		XiMessageBoxOk("����ģ��ƥ����ֹ��");
		return;
	}

	std::vector<CvPoint3D64f> vtBasePoints(4);
	std::vector<CvPoint3D64f> vtTargetPoints(4);
	COPini opini;
	opini.SetFileName(sTemplateIniFile);
	opini.SetSectionName("BasePoints");
	opini.ReadString("PtnX0", &vtBasePoints[0].x);
	opini.ReadString("PtnY0", &vtBasePoints[0].y);
	opini.ReadString("PtnZ0", &vtBasePoints[0].z);
	opini.ReadString("PtnX1", &vtBasePoints[1].x);
	opini.ReadString("PtnY1", &vtBasePoints[1].y);
	opini.ReadString("PtnZ1", &vtBasePoints[1].z);
	opini.ReadString("PtnX2", &vtBasePoints[2].x);
	opini.ReadString("PtnY2", &vtBasePoints[2].y);
	opini.ReadString("PtnZ2", &vtBasePoints[2].z);
	opini.ReadString("PtnX3", &vtBasePoints[3].x);
	opini.ReadString("PtnY3", &vtBasePoints[3].y);
	opini.ReadString("PtnZ3", &vtBasePoints[3].z);
	opini.SetSectionName("TargetPoints");
	opini.ReadString("PtnX0", &vtTargetPoints[0].x);
	opini.ReadString("PtnY0", &vtTargetPoints[0].y);
	opini.ReadString("PtnZ0", &vtTargetPoints[0].z);
	opini.ReadString("PtnX1", &vtTargetPoints[1].x);
	opini.ReadString("PtnY1", &vtTargetPoints[1].y);
	opini.ReadString("PtnZ1", &vtTargetPoints[1].z);
	opini.ReadString("PtnX2", &vtTargetPoints[2].x);
	opini.ReadString("PtnY2", &vtTargetPoints[2].y);
	opini.ReadString("PtnZ2", &vtTargetPoints[2].z);
	opini.ReadString("PtnX3", &vtTargetPoints[3].x);
	opini.ReadString("PtnY3", &vtTargetPoints[3].y);
	opini.ReadString("PtnZ3", &vtTargetPoints[3].z);

	cv::Mat mat = CalculateTransformationMatrix(vtBasePoints, vtTargetPoints);
	LoadCloudProcessResultNew(sTemplateFile);
	int nPtnNum = m_vtWeldSeamData.size();
//	LineOrCircularArcWeldingLine tWeldLine;
	std::vector<CvPoint3D64f> vtStartPtn(nPtnNum);
	std::vector<CvPoint3D64f> vtEndPtn(nPtnNum);
	std::vector<CvPoint3D64f> vtNormal(nPtnNum);
	std::vector<CvPoint3D64f> vtCenter(nPtnNum);
	for (int i = 0; i < nPtnNum; i++)
	{
		vtStartPtn[i] = m_vtWeldSeamData[i].StartPoint;
		vtEndPtn[i] = m_vtWeldSeamData[i].EndPoint;
		vtCenter[i] = m_vtWeldSeamData[i].CenterPoint;
		vtNormal[i].x = m_vtWeldSeamData[i].StartPoint.x + m_vtWeldSeamData[i].StartNormalVector.x * 1000.0;
		vtNormal[i].y = m_vtWeldSeamData[i].StartPoint.y + m_vtWeldSeamData[i].StartNormalVector.y * 1000.0;
		vtNormal[i].z = m_vtWeldSeamData[i].StartPoint.z + m_vtWeldSeamData[i].StartNormalVector.z * 1000.0;
	}

	std::vector<CvPoint3D64f> vtNewStartPtn = CoordinateTransformate(vtStartPtn, mat);
	std::vector<CvPoint3D64f> vtNewEndPtn = CoordinateTransformate(vtEndPtn, mat);
	std::vector<CvPoint3D64f> vtNewNormal = CoordinateTransformate(vtNormal, mat);
	std::vector<CvPoint3D64f> vtNewCenter = CoordinateTransformate(vtCenter, mat);

	for (int i = 0; i < nPtnNum; i++)
	{
		m_vtWeldSeamData[i].StartPoint = vtNewStartPtn[i];
		m_vtWeldSeamData[i].EndPoint = vtNewEndPtn[i];
		if (m_vtWeldSeamData[i].IsArc)m_vtWeldSeamData[i].CenterPoint = vtNewCenter[i];
		m_vtWeldSeamData[i].StartNormalVector.x = (vtNewNormal[i].x - m_vtWeldSeamData[i].StartPoint.x) / 1000.0;
		m_vtWeldSeamData[i].StartNormalVector.y = (vtNewNormal[i].y - m_vtWeldSeamData[i].StartPoint.y) / 1000.0;
		m_vtWeldSeamData[i].StartNormalVector.z = (vtNewNormal[i].z - m_vtWeldSeamData[i].StartPoint.z) / 1000.0;
		m_vtWeldSeamData[i].EndNormalVector = m_vtWeldSeamData[i].StartNormalVector;
	}

	CString sOutFileName = OUTPUT_PATH + m_ptUnit->m_tContralUnit.strUnitName + "\\" + POINT_CLOUD_IDENTIFY_RESULT;
	CString sSaveFileName = OUTPUT_PATH + m_ptUnit->m_tContralUnit.strUnitName + "\\" + IDENTIFY_RESULT_SAVE;
	FILE* pf = fopen(sOutFileName.GetBuffer(), "w");
	for (int i = 0; i < nPtnNum; i++)
	{
		LineOrCircularArcWeldingLine tWeld = m_vtWeldSeamData[i];
		WeldLineInfo tWeldInfo = m_vtWeldSeamInfo[i];
		fprintf(pf, "%d%4d%4d%11.3lf %4d %11.3lf%11.3lf%11.3lf %11.3lf%11.3lf%11.3lf %11.3lf%11.3lf%11.3lf %11.3lf%11.3lf%11.3lf %11.3lf%11.3lf%11.3lf %4d%4d %4d%4d%4d %11.3lf%11.3lf%11.3lf%4d 0.0 0.0 0.0 0.0\n",
			i, tWeld.IsArc, tWeld.isClockwise, tWeld.ZSide, tWeld.isLeft,
			tWeld.CenterPoint.x, tWeld.CenterPoint.y, tWeld.CenterPoint.z,
			tWeld.StartPoint.x, tWeld.StartPoint.y, tWeld.StartPoint.z,
			tWeld.EndPoint.x, tWeld.EndPoint.y, tWeld.EndPoint.z,
			tWeld.StartNormalVector.x, tWeld.StartNormalVector.y, tWeld.StartNormalVector.z,
			tWeld.EndNormalVector.x, tWeld.EndNormalVector.y, tWeld.EndNormalVector.z,
			tWeld.StartPointType, tWeld.EndPointType,
			tWeldInfo.tAtrribute.nIsDoubleWelding, tWeldInfo.tAtrribute.nStartWrapType, tWeldInfo.tAtrribute.nEndWrapType,
			tWeldInfo.tAtrribute.nWeldAngleSize, tWeldInfo.tAtrribute.dStartHoleSize, tWeldInfo.tAtrribute.dEndHoleSize, tWeldInfo.tAtrribute.nRobotSelete);
	}
	fclose(pf);
	CopyFile((LPCTSTR)sOutFileName, (LPCTSTR)sSaveFileName, FALSE);
	XiMessageBoxOk("����ģ��ƥ����ɣ�");
	RunInteractiveWindow(m_ptUnit->m_tContralUnit.strUnitName);
	return;
}

//����ֵ:��ͬһ����ϵ�´�ԭʼλ�õ�Ŀ��λ�õ�����任����
//BasePoints:ԭʼλ���ĸ���Ӧ��������ϵ�µ�����
//TargetPoints:Ŀ��λ���ĸ���Ӧ��������ϵ�µ�����
/*���ĸ��㹲��,�����BaseCoordinateMatrix������,������󲻴���,�ڼ���BaseCoordinateMatrix_1ʱ���õ�����Ľ��*/
cv::Mat WeldAfterMeasure::CalculateTransformationMatrix(std::vector<CvPoint3D64f>& BasePoints, std::vector<CvPoint3D64f>& TargetPoints)
{
	if ((BasePoints.size() != 4) || (TargetPoints.size() != 4))
		return cv::Mat();
	cv::Mat BaseCoordinateMatrix = cv::Mat::zeros(4, 4, CV_64F);
	for (int i = 0; i < 4; i++)
	{
		BaseCoordinateMatrix.ptr<double>(0)[i] = BasePoints[i].x;
		BaseCoordinateMatrix.ptr<double>(1)[i] = BasePoints[i].y;
		BaseCoordinateMatrix.ptr<double>(2)[i] = BasePoints[i].z;
		BaseCoordinateMatrix.ptr<double>(3)[i] = 1.0;
	}
	cv::Mat TargetCoordinateMatrix = cv::Mat::zeros(4, 4, CV_64F);
	for (int i = 0; i < 4; i++)
	{
		TargetCoordinateMatrix.ptr<double>(0)[i] = TargetPoints[i].x;
		TargetCoordinateMatrix.ptr<double>(1)[i] = TargetPoints[i].y;
		TargetCoordinateMatrix.ptr<double>(2)[i] = TargetPoints[i].z;
		TargetCoordinateMatrix.ptr<double>(3)[i] = 1.0;
	}
	cv::Mat BaseCoordinateMatrix_1;
	cv::invert(BaseCoordinateMatrix, BaseCoordinateMatrix_1, cv::DECOMP_LU);
	cv::Mat TransformationMatrix = TargetCoordinateMatrix * BaseCoordinateMatrix_1;
	return TransformationMatrix;
}

//����ֵ:ԭʼλ�õ�任��Ŀ��λ�ú�Ľ������ͬһ����ϵ�µ�����
//BasePoints:ԭʼλ�õ�����
//TransformationMatrix:����任����
std::vector<CvPoint3D64f> WeldAfterMeasure::CoordinateTransformate(std::vector<CvPoint3D64f>& BasePoints, cv::Mat TransformationMatrix)
{
	if ((TransformationMatrix.rows != 4) || (TransformationMatrix.cols != 4) || (TransformationMatrix.type() != CV_64F))
		return std::vector<CvPoint3D64f>();
	cv::Mat BaseCoordinateMatrix = cv::Mat::zeros(4, int(BasePoints.size()), CV_64F);
	for (size_t i = 0; i < BasePoints.size(); i++)
	{
		BaseCoordinateMatrix.ptr<double>(0)[i] = BasePoints[i].x;
		BaseCoordinateMatrix.ptr<double>(1)[i] = BasePoints[i].y;
		BaseCoordinateMatrix.ptr<double>(2)[i] = BasePoints[i].z;
		BaseCoordinateMatrix.ptr<double>(3)[i] = 1.0;
	}
	cv::Mat TargetCoordinateMatrix = TransformationMatrix * BaseCoordinateMatrix;
	std::vector<CvPoint3D64f> TargetPoints(BasePoints.size());
	for (size_t i = 0; i < TargetPoints.size(); i++)
	{
		TargetPoints[i].x = TargetCoordinateMatrix.ptr<double>(0)[i];
		TargetPoints[i].y = TargetCoordinateMatrix.ptr<double>(1)[i];
		TargetPoints[i].z = TargetCoordinateMatrix.ptr<double>(2)[i];
	}
	return TargetPoints;
}

CvPoint3D64f WeldAfterMeasure::RotatePoint3D(CvPoint3D64f Point, CvPoint3D64f& RotationCenter, CvPoint3D64f& RotationAxis, double RotationAngle)
{
	CvPoint3D64f Vector_OP = cvPoint3D64f(Point.x - RotationCenter.x, Point.y - RotationCenter.y, Point.z - RotationCenter.z);
	double Vector_OP_Length = sqrt(Vector_OP.x * Vector_OP.x + Vector_OP.y * Vector_OP.y + Vector_OP.z * Vector_OP.z);
	double AxisLength = sqrt(RotationAxis.x * RotationAxis.x + RotationAxis.y * RotationAxis.y + RotationAxis.z * RotationAxis.z);
	CvPoint3D64f n = cvPoint3D64f(RotationAxis.x / AxisLength, RotationAxis.y / AxisLength, RotationAxis.z / AxisLength);
	double cos_angle_OP_n = (Vector_OP.x * n.x + Vector_OP.y * n.y + Vector_OP.z * n.z) / Vector_OP_Length;
	double Vector_OA_Length = Vector_OP_Length * cos_angle_OP_n;
	CvPoint3D64f Vector_OA = cvPoint3D64f(n.x * Vector_OA_Length, n.y * Vector_OA_Length, n.z * Vector_OA_Length);
	CvPoint3D64f Vector_AP = cvPoint3D64f(Vector_OP.x - Vector_OA.x, Vector_OP.y - Vector_OA.y, Vector_OP.z - Vector_OA.z);
	double Vector_AP_Length = sqrt(Vector_AP.x * Vector_AP.x + Vector_AP.y * Vector_AP.y + Vector_AP.z * Vector_AP.z);
	CvPoint3D64f ap = cvPoint3D64f(Vector_AP.x / Vector_AP_Length, Vector_AP.y / Vector_AP_Length, Vector_AP.z / Vector_AP_Length);
	CvPoint3D64f ap_n = cvPoint3D64f(ap.y * n.z - ap.z * n.y, ap.z * n.x - ap.x * n.z, ap.x * n.y - ap.y * n.x);
	CvPoint3D64f Vector_AP_;
	Vector_AP_.x = -Vector_AP_Length * sin(RotationAngle) * ap_n.x + cos(RotationAngle) * Vector_AP.x;
	Vector_AP_.y = -Vector_AP_Length * sin(RotationAngle) * ap_n.y + cos(RotationAngle) * Vector_AP.y;
	Vector_AP_.z = -Vector_AP_Length * sin(RotationAngle) * ap_n.z + cos(RotationAngle) * Vector_AP.z;
	CvPoint3D64f NewPoint = cvPoint3D64f(Vector_OA.x + Vector_AP_.x + RotationCenter.x, Vector_OA.y + Vector_AP_.y + RotationCenter.y, Vector_OA.z + Vector_AP_.z + RotationCenter.z);

	return NewPoint;
}

XI_POINT WeldAfterMeasure::RotatePoint3D(XI_POINT Point, XI_POINT& RotationCenter, XI_POINT& RotationAxis, double RotationAngle)
{
	XI_POINT Vector_OP = CreateXI_POINT(Point.x - RotationCenter.x, Point.y - RotationCenter.y, Point.z - RotationCenter.z);
	double Vector_OP_Length = sqrt(Vector_OP.x * Vector_OP.x + Vector_OP.y * Vector_OP.y + Vector_OP.z * Vector_OP.z);
	double AxisLength = sqrt(RotationAxis.x * RotationAxis.x + RotationAxis.y * RotationAxis.y + RotationAxis.z * RotationAxis.z);
	XI_POINT n = CreateXI_POINT(RotationAxis.x / AxisLength, RotationAxis.y / AxisLength, RotationAxis.z / AxisLength);
	double cos_angle_OP_n = (Vector_OP.x * n.x + Vector_OP.y * n.y + Vector_OP.z * n.z) / Vector_OP_Length;
	double Vector_OA_Length = Vector_OP_Length * cos_angle_OP_n;
	XI_POINT Vector_OA = CreateXI_POINT(n.x * Vector_OA_Length, n.y * Vector_OA_Length, n.z * Vector_OA_Length);
	XI_POINT Vector_AP = CreateXI_POINT(Vector_OP.x - Vector_OA.x, Vector_OP.y - Vector_OA.y, Vector_OP.z - Vector_OA.z);
	double Vector_AP_Length = sqrt(Vector_AP.x * Vector_AP.x + Vector_AP.y * Vector_AP.y + Vector_AP.z * Vector_AP.z);
	XI_POINT ap = CreateXI_POINT(Vector_AP.x / Vector_AP_Length, Vector_AP.y / Vector_AP_Length, Vector_AP.z / Vector_AP_Length);
	XI_POINT ap_n = CreateXI_POINT(ap.y * n.z - ap.z * n.y, ap.z * n.x - ap.x * n.z, ap.x * n.y - ap.y * n.x);
	XI_POINT Vector_AP_;
	Vector_AP_.x = -Vector_AP_Length * sin(RotationAngle) * ap_n.x + cos(RotationAngle) * Vector_AP.x;
	Vector_AP_.y = -Vector_AP_Length * sin(RotationAngle) * ap_n.y + cos(RotationAngle) * Vector_AP.y;
	Vector_AP_.z = -Vector_AP_Length * sin(RotationAngle) * ap_n.z + cos(RotationAngle) * Vector_AP.z;
	XI_POINT NewPoint = CreateXI_POINT(Vector_OA.x + Vector_AP_.x + RotationCenter.x, Vector_OA.y + Vector_AP_.y + RotationCenter.y, Vector_OA.z + Vector_AP_.z + RotationCenter.z);

	return NewPoint;
}

void WeldAfterMeasure::GetChangePosturePare(vector<T_ROBOT_COORS> tLeftTrackCoors, XI_POINT tLeftCenter, double dLeftNormal, bool bLeftIsArc, vector<T_ROBOT_COORS> tRightTrackCoors, XI_POINT tRightCenter, double dRightNormal, bool bRightIsArc, bool bStartOrEnd,
	int& nStartStepNo, double& dStChangeAngle, double dChangePostureThresVal)
{
	T_ALGORITHM_POINT_2D tFirstIntersectionPoint, tSecondIntersectionPoint, tPoint;
	XI_POINT intersection;
	double dLeftNormalAngle = dLeftNormal - 90 * m_nRobotInstallDir;
	double dRightNormalAngle = dRightNormal + 90 * m_nRobotInstallDir;
	double dDirx = CosD(dLeftNormalAngle);
	double dDiry = SinD(dLeftNormalAngle);
	double dLeftDirx = CosD(dRightNormalAngle);
	double dLeftDiry = SinD(dRightNormalAngle);
	XI_POINT tVertor1 = { dDirx ,dDiry ,0 };
	XI_POINT tVertor2 = { dLeftDirx ,dLeftDiry ,0 };
	double dAngle = calculateAngle(tVertor1, tVertor2), dStartDis;
	dStChangeAngle = 90 - dAngle / 2;
	if (dAngle < 85) //�н�С��85��
	{
		bool bIsLineLeft = bLeftIsArc;
		bool bIsLineRight = bRightIsArc;
		if (!bStartOrEnd) // ��βʱ��෴ת����
		{
			reverse(tLeftTrackCoors.begin(), tLeftTrackCoors.end());
		}
		int nSize = tLeftTrackCoors.size() < tRightTrackCoors.size() ? tLeftTrackCoors.size() : tRightTrackCoors.size();
		for (int n = 0; n < nSize - 1; n++)
		{
			if (bStartOrEnd) // ��ʼ��
			{
				// ֱ�� ֱ�� // ֱ�� Բ��
				if (!bIsLineLeft && !bIsLineRight || !bIsLineLeft && bIsLineRight)
				{
					CalcLineLineIntersection(tRightTrackCoors.at(n), RzToDirAngle(tRightTrackCoors.at(n).dRZ),
						tLeftTrackCoors.at(0), RzToDirAngle(tLeftTrackCoors.at(0).dRZ - 90 * m_nRobotInstallDir), intersection);

				}
				else if (bIsLineLeft && !bIsLineRight)// Բ�� ֱ��
				{
					XiAlgorithm xiAlg;
					T_ALGORITHM_POINT_2D tPointStart = { tRightTrackCoors.at(n).dX,tRightTrackCoors.at(n).dY };
					T_ALGORITHM_POINT_2D tPointEnd = { tRightTrackCoors.at(n).dX + 50 * CosD(RzToDirAngle(tRightTrackCoors.at(n).dRZ)),
													   tRightTrackCoors.at(n).dY + 50 * SinD(RzToDirAngle(tRightTrackCoors.at(n).dRZ)) };
					T_ALGORITHM_POINT_2D tCenter = { tLeftCenter.x,tLeftCenter.y };
					double dRadius = TwoPointDis(tCenter.x, tCenter.y, tLeftTrackCoors.at(0).dX, tLeftTrackCoors.at(0).dY);
					tPoint = tPointStart;

					xiAlg.CalLineArcTwoIntersectionPoint(tPointStart, tPointEnd, tCenter, dRadius,
						tFirstIntersectionPoint, tSecondIntersectionPoint);
					XI_POINT tP1 = { tFirstIntersectionPoint.x,tFirstIntersectionPoint.y,tRightTrackCoors.at(n).dZ };
					XI_POINT tP2 = { tSecondIntersectionPoint.x,tSecondIntersectionPoint.y,tRightTrackCoors.at(n).dZ };

					double dDisInter1 = TwoPointDis(tFirstIntersectionPoint.x, tFirstIntersectionPoint.y, tPoint.x, tPoint.y);
					double dDisInter2 = TwoPointDis(tSecondIntersectionPoint.x, tSecondIntersectionPoint.y, tPoint.x, tPoint.y);

					intersection = dDisInter1 < dDisInter2 ? tP1 : tP2;
				}

				dStartDis = TwoPointDis(intersection.x, intersection.y, intersection.z,
					tRightTrackCoors.at(n).dX, tRightTrackCoors.at(n).dY, tRightTrackCoors.at(n).dZ);
			}
			else //��β��
			{
				// ֱ�� ֱ�� // Բ�� ֱ�� 
				if (!bIsLineLeft && !bIsLineRight || bIsLineLeft && !bIsLineRight)
				{
					CalcLineLineIntersection(tLeftTrackCoors.at(n), RzToDirAngle(tLeftTrackCoors.at(n).dRZ),
						tRightTrackCoors.at(0), RzToDirAngle(tRightTrackCoors.at(0).dRZ - 90 * m_nRobotInstallDir), intersection);

				}
				else if (!bIsLineLeft && bIsLineRight)// ֱ�� Բ�� 
				{
					XiAlgorithm xiAlg;
					T_ALGORITHM_POINT_2D tPointStart = { tLeftTrackCoors.at(n).dX,tLeftTrackCoors.at(n).dY };
					T_ALGORITHM_POINT_2D tPointEnd = { tLeftTrackCoors.at(n).dX + 50 * CosD(RzToDirAngle(tLeftTrackCoors.at(n).dRZ)),
													   tLeftTrackCoors.at(n).dY + 50 * SinD(RzToDirAngle(tLeftTrackCoors.at(n).dRZ)) };
					T_ALGORITHM_POINT_2D tCenter = { tRightCenter.x,tRightCenter.y };
					double dRadius = TwoPointDis(tCenter.x, tCenter.y, tRightTrackCoors.at(0).dX, tRightTrackCoors.at(0).dY);
					tPoint = tPointStart;

					xiAlg.CalLineArcTwoIntersectionPoint(tPointStart, tPointEnd, tCenter, dRadius,
						tFirstIntersectionPoint, tSecondIntersectionPoint);
					XI_POINT tP1 = { tFirstIntersectionPoint.x,tFirstIntersectionPoint.y,tLeftTrackCoors.at(n).dZ };
					XI_POINT tP2 = { tSecondIntersectionPoint.x,tSecondIntersectionPoint.y,tLeftTrackCoors.at(n).dZ };

					double dDisInter1 = TwoPointDis(tFirstIntersectionPoint.x, tFirstIntersectionPoint.y, tPoint.x, tPoint.y);
					double dDisInter2 = TwoPointDis(tSecondIntersectionPoint.x, tSecondIntersectionPoint.y, tPoint.x, tPoint.y);

					intersection = dDisInter1 < dDisInter2 ? tP1 : tP2;
				}

				dStartDis = TwoPointDis(intersection.x, intersection.y, intersection.z,
					tLeftTrackCoors.at(n).dX, tLeftTrackCoors.at(n).dY, tLeftTrackCoors.at(n).dZ);
			}

			if (dStartDis > fabs(dChangePostureThresVal))
			{
				nStartStepNo = n;
				break;
			}
		}
	}
	if (nStartStepNo < 45)
	{
		nStartStepNo = 45;
	}
}

// ����н�
double WeldAfterMeasure::calculateAngle(XI_POINT vector1, XI_POINT vector2) {
	double dotProduct = dot_product(vector1, vector2);
	double magnitude1 = std::sqrt(vector1.x * vector1.x + vector1.y * vector1.y + vector1.z * vector1.z);
	double magnitude2 = std::sqrt(vector2.x * vector2.x + vector2.y * vector2.y + vector2.z * vector2.z);
	double cosTheta = dotProduct / (magnitude1 * magnitude2);
	double angle = std::acos(cosTheta);
	return angle * 180 / 3.1415926;
}

bool WeldAfterMeasure::CalcLineLineIntersection(T_ROBOT_COORS tStart, double dStartAngle, T_ROBOT_COORS tEnd, double dEndAngle, XI_POINT& intersection)
{
	double slope1 = TanD(dStartAngle), slope2 = TanD(dEndAngle);
	if (fabs(slope1) == fabs(slope2))
	{
		XiMessageBox("��ֱ��ƽ�У��޽���");
		return false;
	}
	// ���㽻��� x ����
	intersection.x = (tEnd.dY - tStart.dY + slope1 * tStart.dX - slope2 * tEnd.dX) / (slope1 - slope2);
	// ʹ������һ��ֱ�ߵķ��̼��㽻��� y ����
	intersection.y = tStart.dY + slope1 * (intersection.x - tStart.dX);
	intersection.z = (tStart.dZ + tEnd.dZ) / 2;
	return true;
}

void WeldAfterMeasure::WeldSeamTransCoor_Gantry2Robot(std::vector<LineOrCircularArcWeldingLine>& vtWeldSeamData, std::vector<WeldLineInfo>& vtWeldSeamInfo)
{
	std::vector<CvPoint3D64f> tCenterPoint;
	std::vector<CvPoint3D64f> tStartPoint;
	std::vector<CvPoint3D64f> tEndPoint;
	std::vector<CvPoint3D64f> tStartNormalVector;
	std::vector<CvPoint3D64f> tEndNormalVector;
	std::vector<CvPoint3D64f> vtTrackPos;
	CvPoint3D64f tTrackPos = { 0 };
	for (int num = 0; num < vtWeldSeamData.size(); num++)
	{
		tCenterPoint.push_back(vtWeldSeamData[num].CenterPoint);
		tStartPoint.push_back(vtWeldSeamData[num].StartPoint);
		tEndPoint.push_back(vtWeldSeamData[num].EndPoint);
		tStartNormalVector.push_back(vtWeldSeamData[num].StartNormalVector);
		tEndNormalVector.push_back(vtWeldSeamData[num].EndNormalVector);
	}
	
	std::vector<CvPoint3D64f> reCenterPoint = m_ptUnit->TransCoor_Gantry2RobotNew(tCenterPoint);
	std::vector<CvPoint3D64f> reStartPoint = m_ptUnit->TransCoor_Gantry2RobotNew(tStartPoint);
	std::vector<CvPoint3D64f> reEndPoint = m_ptUnit->TransCoor_Gantry2RobotNew(tEndPoint);
	std::vector<CvPoint3D64f> reStartNormalVector = m_ptUnit->TransNorVector_Gantry2Robot(tStartNormalVector);
	std::vector<CvPoint3D64f> reEndNormalVector = m_ptUnit->TransNorVector_Gantry2Robot(tEndNormalVector);

	for (int num1 = 0; num1 < vtWeldSeamData.size(); num1++)
	{
		vtWeldSeamData[num1].CenterPoint = reCenterPoint[num1];
		vtWeldSeamData[num1].StartPoint = reStartPoint[num1];
		vtWeldSeamData[num1].EndPoint = reEndPoint[num1];
		vtWeldSeamData[num1].StartNormalVector = reStartNormalVector[num1];
		vtWeldSeamData[num1].EndNormalVector = reEndNormalVector[num1];
		vtWeldSeamInfo[num1].tWeldLine = vtWeldSeamData[num1];
	}
}

bool WeldAfterMeasure::WeldSeamGroupingAuto(int& nWeldGroupNum)
{
	m_vvtWeldSeamGroup.clear();
	m_vvtWeldLineInfoGroup.clear();
	WeldLineInfo tWeldSeam;
	vector<WeldLineInfo> vtWeldSeamPlane(0);
	vtWeldSeamPlane.clear();
	for (int nWeldNo = 0; nWeldNo < m_vtWeldSeamInfo.size(); nWeldNo++)
	{
		tWeldSeam = m_vtWeldSeamInfo.at(nWeldNo);
		// ��麸���Ƿ��������з���
		int nGroupNo = 0;
		for (; nGroupNo < m_vvtWeldLineInfoGroup.size(); nGroupNo++)
		{
			if (tWeldSeam.tAtrribute.nGroupNo == m_vvtWeldLineInfoGroup.at(nGroupNo).at(0).tAtrribute.nGroupNo)
			{
				m_vvtWeldLineInfoGroup.at(nGroupNo).push_back(tWeldSeam);
				break;
			}
		}
		// ���û�����ݻ�ǰ����û�����Է�������Ϊͬһ�������һ��
		if (m_vvtWeldLineInfoGroup.size() < 1 || nGroupNo == m_vvtWeldLineInfoGroup.size())
		{
			vtWeldSeamPlane.clear();
			vtWeldSeamPlane.push_back(tWeldSeam);
			m_vvtWeldLineInfoGroup.push_back(vtWeldSeamPlane);
			vtWeldSeamPlane.clear();
			continue;
		}
	}
	// ���㷨��������
	sort(m_vvtWeldLineInfoGroup.begin(), m_vvtWeldLineInfoGroup.end(),
		[](const std::vector<WeldLineInfo>& vtSeam1, const std::vector<WeldLineInfo>& vtSeam2)->bool 
		{ 
			int nGroup1 = 99999;
			int nGroup2 = 99999;
			if (vtSeam1.size()> 0)
			{
				nGroup1 = vtSeam1.at(0).tAtrribute.nGroupNo;
			}
			if (vtSeam2.size() > 0)
			{
				nGroup2 = vtSeam2.at(0).tAtrribute.nGroupNo;
			}
			return nGroup1 < nGroup2;
		}
	);

	vector<LineOrCircularArcWeldingLine> vtWeldSeam(0);
	for (int nGroupNo = 0; nGroupNo < m_vvtWeldLineInfoGroup.size(); nGroupNo++)
	{
		vtWeldSeam.clear();
		for (int nSeamNo = 0; nSeamNo < m_vvtWeldLineInfoGroup[nGroupNo].size(); nSeamNo++)
		{
			vtWeldSeam.push_back(m_vvtWeldLineInfoGroup[nGroupNo][nSeamNo].tWeldLine);
		}
		m_vvtWeldSeamGroup.push_back(vtWeldSeam);
	}

	nWeldGroupNum = m_vvtWeldSeamGroup.size();
	m_vvtWeldSeamGroupAdjust = m_vvtWeldSeamGroup;
	m_vvtCowWeldSeamGroupAdjust = m_vvtWeldSeamGroup;
	// ���㺸������ģʽ������/�Ȳ��
	DetermineWeldingMode();
	SaveWeldSeamGroupInfo();

	return true;
}


//void WeldAfterMeasure::InitCheckCollide(CString sModelFilePath)
//{
//	if (!m_bCheckCollide) return;
//	CheckFolder(OUTPUT_PATH + m_ptUnit->m_tContralUnit.strUnitName + "\\Collide\\");
//	m_pCollide = xi::CreatRobotCollideClosedPlaneInterfaceObj();
//	//m_pCollide->setRobotPath(DATA_PATH + m_ptUnit->m_tContralUnit.strUnitName + "\\new_head.step");
//}
//
//void WeldAfterMeasure::SetCheckCollidePlane()
//{
//	if (!m_bCheckCollide) return;
//	m_pCollide->clearAllPlanes();
//	const vector<T_RECT_POINT>& vtRects = m_vtPartPlaneRects;
//	for (int i = 0; i < vtRects.size(); i++)
//	{
//		m_pCollide->addRecFace(
//			xi::Point(vtRects[i].tLeftDown.dCoorX,  vtRects[i].tLeftDown.dCoorY , vtRects[i].tLeftDown.dCoorZ),
//			xi::Point(vtRects[i].tRightDown.dCoorX, vtRects[i].tRightDown.dCoorY, vtRects[i].tRightDown.dCoorZ),
//			xi::Point(vtRects[i].tRightUp.dCoorX,   vtRects[i].tRightUp.dCoorY  , vtRects[i].tRightUp.dCoorZ),
//			xi::Point(vtRects[i].tLeftUp.dCoorX,    vtRects[i].tLeftUp.dCoorY   , vtRects[i].tLeftUp.dCoorZ));
//	}
//}
//
//bool WeldAfterMeasure::CheckIsCollide(T_ROBOT_COORS tCoord, bool bOutputStep, double dCheckDis)
//{
//	bool bCollide = false;
//	vector<T_ANGLE_PULSE> vtPulse(0);
//	T_ROBOT_COORS tFlaneTool(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
//	m_pRobotDriver->RobotInverseKinematics(tCoord, m_pRobotDriver->m_tTools.tGunTool, vtPulse);
//	m_pRobotDriver->RobotKinematics(vtPulse[0], tFlaneTool, tCoord);
//	CString sOutPutPath;
//	sOutPutPath.Format("%s\\Collide\\", OUTPUT_PATH + m_ptUnit->m_tContralUnit.strUnitName);
//
//	if (!m_bCheckCollide) return false;
//
//	double dMat[4][4] = { 0 };
//	m_pRobotDriver->m_cXiRobotAbsCoorsTrans->GenerateToolToBaseMatrix(tCoord.dX, tCoord.dY, tCoord.dZ, tCoord.dRX, tCoord.dRY, tCoord.dRZ, dMat);
//	float afRobotPose[16] = { // ��ʼ��̬
//		dMat[0][0], dMat[0][1], dMat[0][2], dMat[0][3],
//		dMat[1][0], dMat[1][1], dMat[1][2], dMat[1][3],
//		dMat[2][0], dMat[2][1], dMat[2][2], dMat[2][3],
//		dMat[3][0], dMat[3][1], dMat[3][2], dMat[3][3]
//	};
//	m_pCollide->setRobotPose(afRobotPose);
//	bCollide = m_pCollide->collide(bOutputStep, sOutPutPath.GetBuffer(), dCheckDis);
//
//	SYSTEMTIME tTime;
//	GetLocalTime(&tTime);
//	CString sSrcFilePath = sOutPutPath + "result.step";
//	CString sDstFilePath;
//	sDstFilePath.Format("%sresult_%.2d-%.2d-%.2d-%.3d.step", sOutPutPath, tTime.wHour, tTime.wMinute, tTime.wSecond, tTime.wMilliseconds);
//	CopyFile((LPCTSTR)sSrcFilePath, (LPCTSTR)sDstFilePath, FALSE);
//	return bCollide;
//}
//
//bool WeldAfterMeasure::CheckIsCollide(T_ANGLE_PULSE tPulse, bool bOutputStep, double dCheckDis)
//{
//	if (!m_bCheckCollide) return false;
//	T_ROBOT_COORS tCoord;
//	m_pRobotDriver->RobotKinematics(tPulse, m_pRobotDriver->m_tTools.tGunTool, tCoord);
//	return CheckIsCollide(tCoord, bOutputStep, dCheckDis);
//}
//
//bool WeldAfterMeasure::CheckIsCollide(vector<T_ROBOT_COORS> vtCoord, double dExAxis, bool bOutputStep, double dCheckDis)
//{
//	if (!m_bCheckCollide) return false;
//	int dStepDisThreshold = 15.0; // ��������������ľ�����ֵ
//	int dStepAngleThreshold = 30.0; // ������̬�任������ֵ������
//	vector<T_ROBOT_COORS> vtCheckCoord(1, vtCoord[0]);
//	for (int i = 1; i < vtCoord.size(); i++)
//	{
//		if (m_pRobotDriver->CompareCoords(vtCoord[i], vtCheckCoord[vtCheckCoord.size() - 1], dStepDisThreshold, 3, dStepAngleThreshold))
//		{
//			continue;
//		}
//		vtCheckCoord.push_back(vtCoord[i]);
//	}
//
//	long lStartTime = XI_clock();
//	bool bIsCollide = false;
//	T_ROBOT_COORS tCoord;
//	vector<T_ANGLE_PULSE> vtPulse(0);
//	T_ROBOT_COORS tFlaneTool(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
//	FILE* pf = fopen(OUTPUT_PATH + m_ptUnit->m_tContralUnit.strUnitName + "\\Collide\\CheckCollideMatrix.txt", "w");
//
//	T_ROBOT_COORS tStandardGunTool(-115.0, 0.0, 679.0, 0.0, -45.0, 0.0, 0.0, 0.0, 0.0);
//
//	for (int i = 0; i < vtCheckCoord.size(); i++)
//	{
//		tCoord = vtCheckCoord[i];
//		m_pRobotDriver->RobotInverseKinematics(tCoord, m_pRobotDriver->m_tTools.tGunTool, vtPulse);
//		m_pRobotDriver->RobotKinematics(vtPulse[0], tFlaneTool, tCoord);
//
//		double dMat[4][4] = { 0 };
//		m_pRobotDriver->m_cXiRobotAbsCoorsTrans->GenerateToolToBaseMatrix(tCoord.dX, tCoord.dY, tCoord.dZ, tCoord.dRX, tCoord.dRY, tCoord.dRZ, dMat);
//		fprintf(pf, "����%d%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf\n", i + 1, tCoord.dX, tCoord.dY, tCoord.dZ, tCoord.dRX, tCoord.dRY, tCoord.dRZ);
//		fprintf(pf, "%11.3lf%11.3lf%11.3lf%11.3lf\n", dMat[0][0], dMat[0][1], dMat[0][2], dMat[0][3]);
//		fprintf(pf, "%11.3lf%11.3lf%11.3lf%11.3lf\n", dMat[1][0], dMat[1][1], dMat[1][2], dMat[1][3]);
//		fprintf(pf, "%11.3lf%11.3lf%11.3lf%11.3lf\n", dMat[2][0], dMat[2][1], dMat[2][2], dMat[2][3]);
//		fprintf(pf, "%11.3lf%11.3lf%11.3lf%11.3lf\n", dMat[3][0], dMat[3][1], dMat[3][2], dMat[3][3]);
//		fprintf(pf, "\n");
//		fflush(pf);
//
//		bool bRst = CheckIsCollide(vtCheckCoord[i], bOutputStep, dCheckDis);
//		if (true == bRst)
//		{
//			bIsCollide = bRst;
//		}
//	}
//	WriteLog("[��ײ���]:��ʱ%d", XI_clock() - lStartTime);
//	return bIsCollide;
//}

bool WeldAfterMeasure::GetPauseWeldTrack(vector<T_ROBOT_COORS>& vtCoord, vector<int> &vnPtnType, E_WELD_SEAM_TYPE eSeamType)
{
	if (!m_ptUnit->m_bBreakPointContinue)
	{
		return true;
	}
	int nPtnNum = vtCoord.size();
	if (nPtnNum < 2) return false;
	double dBackDis = E_FLAT_SEAM == eSeamType ? 15.0 : 6.0;
	T_ROBOT_COORS tPauseCoord;
	//T_ROBOT_COORS tStandardGunTool(-115.0, 0.0, 679.0, 0.0, -45.0, 0.0, 0.0, 0.0, 0.0);
	m_pRobotDriver->RobotKinematics(m_tPausePulse, /*tStandardGunTool*/m_pRobotDriver->m_tTools.tGunTool, tPauseCoord);
	double dTrackStepDis = TwoPointDis(
		vtCoord[0].dX + vtCoord[0].dBX, vtCoord[0].dY + vtCoord[0].dBY, vtCoord[0].dZ + vtCoord[0].dBZ, 
		vtCoord[1].dX + vtCoord[1].dBX, vtCoord[1].dY + vtCoord[1].dBY, vtCoord[1].dZ + vtCoord[1].dBZ);
	double dMinDis = TwoPointDis(
		vtCoord[0].dX + vtCoord[0].dBX, vtCoord[0].dY + vtCoord[0].dBY, vtCoord[0].dZ + vtCoord[0].dBZ,
		tPauseCoord.dX + tPauseCoord.dBX, tPauseCoord.dY + tPauseCoord.dBY, tPauseCoord.dZ + tPauseCoord.dBZ);
	int nMinDisIdx = 0;
	int nBackPtnNum = 1; //dBackDis / dTrackStepDis;
	int nTruncationIdx = 0;
	for (int i = 1; i < vtCoord.size(); i++)
	{
		double dDis = TwoPointDis(
			vtCoord[i].dX + vtCoord[i].dBX, vtCoord[i].dY + vtCoord[i].dBY, vtCoord[i].dZ + vtCoord[i].dBZ,
			tPauseCoord.dX + tPauseCoord.dBX, tPauseCoord.dY + tPauseCoord.dBY, tPauseCoord.dZ + tPauseCoord.dBZ);
		if (dDis < dMinDis)
		{
			dMinDis = dDis;
			nMinDisIdx = i;
		}
	}
	nTruncationIdx = nMinDisIdx - nBackPtnNum;
	nTruncationIdx = nTruncationIdx < 0 ? 0 : nTruncationIdx;
	vtCoord.erase(vtCoord.begin(), vtCoord.begin() + nTruncationIdx);
	vnPtnType.erase(vnPtnType.begin(), vnPtnType.begin() + nTruncationIdx);

	if (E_SCAN_WELD_LINE == m_eWorkPieceType)
	{
		m_dTeachExAxlePos = m_dPauseExAxlePos;
		m_dTeachExAxlePos_y = m_dPauseExAxlePos_y;
	}

	return true;
}

void WeldAfterMeasure::SavePauseInfo(int nGroupNo, int nWeldNo, int nLayerNo)
{
	// �ָ�������ʹ�õ���ͣ״̬ ��Ӱ����һ���������
	m_ptUnit->m_bBreakPointContinue = false;
	m_nPauseGroupNo = 0;
	m_nPauseWeldNo = 0;
	m_nPauseLayerNo = 0;

	COPini opini;
	opini.SetFileName(DATA_PATH + m_ptUnit->m_tContralUnit.strUnitName + "\\Pause.ini");

	opini.SetSectionName("PauseInfo");
	opini.WriteString("GroupNo", nGroupNo);
	opini.WriteString("WeldNo", nWeldNo);
	opini.WriteString("LayerNo", nLayerNo);

	T_ANGLE_PULSE tPausePulse = m_pRobotDriver->GetCurrentPulse();
	opini.SetSectionName("PauseRobotPulse");
	opini.WriteString("SPulse", tPausePulse.nSPulse);
	opini.WriteString("LPulse", tPausePulse.nLPulse);
	opini.WriteString("UPulse", tPausePulse.nUPulse);
	opini.WriteString("RPulse", tPausePulse.nRPulse);
	opini.WriteString("BPulse", tPausePulse.nBPulse);
	opini.WriteString("TPulse", tPausePulse.nTPulse);
	opini.WriteString("BXPulse", tPausePulse.lBXPulse);
	opini.WriteString("BYPulse", tPausePulse.lBYPulse);
	opini.WriteString("BZPulse", tPausePulse.lBZPulse);
}

void WeldAfterMeasure::LoadPauseInfo()
{
	if (!m_ptUnit->m_bBreakPointContinue)
	{
		return;
	}
	COPini opini;
	opini.SetFileName(DATA_PATH + m_ptUnit->m_tContralUnit.strUnitName + "\\Pause.ini");

	opini.SetSectionName("PauseInfo");
	opini.ReadString("GroupNo", &m_nPauseGroupNo);
	opini.ReadString("WeldNo", &m_nPauseWeldNo);
	opini.ReadString("LayerNo", &m_nPauseLayerNo);

	T_ANGLE_PULSE tPausePulse = m_pRobotDriver->GetCurrentPulse();
	opini.SetSectionName("PauseRobotPulse");
	opini.ReadString("SPulse", &m_tPausePulse.nSPulse);
	opini.ReadString("LPulse", &m_tPausePulse.nLPulse);
	opini.ReadString("UPulse", &m_tPausePulse.nUPulse);
	opini.ReadString("RPulse", &m_tPausePulse.nRPulse);
	opini.ReadString("BPulse", &m_tPausePulse.nBPulse);
	opini.ReadString("TPulse", &m_tPausePulse.nTPulse);
	opini.ReadString("BXPulse", &m_tPausePulse.lBXPulse);
	opini.ReadString("BYPulse", &m_tPausePulse.lBYPulse);
	opini.ReadString("BZPulse", &m_tPausePulse.lBZPulse);
}

bool WeldAfterMeasure::CalcRobotPostureCoors(vector<XI_POINT> vtOutPoints, double dRX, double dRY, double dStepDis, double dStChangePostureDis, double dStChangeAngle,
	double dEdChangePostureDis, double dEdChangeAngle, bool bStartType, bool bEndType, double dStartHoleSize,double dEndHoleSize, int& nStartChangeStepNo, int& nEndChangeStepNo, vector<XI_POINT>& vtPosture)
{
	if (vtOutPoints.size() < 3)
	{
		XiMessageBox("CalcRobotPostureCoors ��������");
		return false;
	}
	//���������̬
	vtPosture.clear();
	int nDir = 1;//��̬�仯����
	bool bStartChangeAngle = true;//˳�����
	int nChangePtnNumSt = dStChangePostureDis / dStepDis; // ���ٸ��������̬
	int nChangePtnNumEd = dEdChangePostureDis / dStepDis; // ���ٸ��������̬
	double dChangeAngleUnitSt = nChangePtnNumEd == 0 ? 0 : dStChangeAngle / (double)nChangePtnNumSt; // ���ڵ��Ƕȱ仯����
	double dChangeAngleUnitEd = nChangePtnNumEd == 0 ? 0 : dEdChangeAngle / (double)nChangePtnNumEd; // ���ڵ��Ƕȱ仯����
	int nDeleteStepNumStart = dStartHoleSize / dStepDis;//�����ٱ���̬����
	int nDeleteStepNumEnd = dEndHoleSize / dStepDis;//�յ���ٱ���̬����

	nChangePtnNumSt -= nDeleteStepNumStart;
	nChangePtnNumEd -= nDeleteStepNumEnd;
	nChangePtnNumSt = nChangePtnNumSt < 0 ? 0 : nChangePtnNumSt;
	nChangePtnNumEd = nChangePtnNumEd < 0 ? 0 : nChangePtnNumEd;
	if (true == bStartType)
	{
		nStartChangeStepNo = nChangePtnNumSt;
	}
	if (true == bEndType)
	{
		nEndChangeStepNo = nChangePtnNumEd;
	}

	for (int i = 0; i < vtOutPoints.size() - 1; i++)
	{
		XI_POINT tEndp = vtOutPoints.at(i + 1);
		XI_POINT tStartp = vtOutPoints.at(i);
		double dLength = TwoPointDis(tEndp.x, tEndp.y, tEndp.z, tStartp.x, tStartp.y, tStartp.z);
		/*XI_POINT tDir = {
			(tEndp.x - tStartp.x) / dLength,
			(tEndp.y - tStartp.y) / dLength ,
			(tEndp.z - tStartp.z) / dLength };
		double dDireAngle = atan2(tDir.y, tDir.x) * 180 / 3.1415926;*/
		// ����ʹ�ø�������������Rz
		vector<XI_POINT> vtDirPoints;
		for (size_t n = 0; n < 5; n++)
		{
			if ((i + n)< vtOutPoints.size())
			{
				vtDirPoints.push_back(vtOutPoints[i + n]);
			}
			else
			{
				vtDirPoints.clear();
				for (size_t j = 5; j > 0; j--)
				{
					vtDirPoints.push_back(vtOutPoints[i - j]);
				}
				break;
			}
			
		}
		T_LINE_PARA tBestLineParam;
		if (!CalcLineParamRansac(tBestLineParam, vtDirPoints, 0.7)) { return false; }
		double dDireAngle = atan2(tBestLineParam.dDirY, tBestLineParam.dDirX) * 180 / 3.1415926;
		double dNormal = dDireAngle - 90 * m_nRobotInstallDir;

		if (true == bStartType && i <= nChangePtnNumSt) // ��㴦
		{
			dNormal += (double)(nChangePtnNumSt - i) * dChangeAngleUnitSt * m_nRobotInstallDir;
		}
		if (true == bEndType && i > (vtOutPoints.size() - nChangePtnNumEd - 2)) // �յ㴦
		{
			dNormal -= (double)(i - (vtOutPoints.size() - nChangePtnNumEd - 2)) * dChangeAngleUnitEd * m_nRobotInstallDir;
		}

		XI_POINT tPosture = { dRX,dRY,m_pRobotDriver->DirAngleToRz(dNormal) };
		vtPosture.push_back(tPosture);
	}
	//������һ������̬
	vtPosture.push_back(vtPosture.at(vtPosture.size() - 1));
	if (vtPosture.size() != vtOutPoints.size())
	{
		XiMessageBox("�����̬������ƥ��");
		return false;
	}
	return true;
}

bool WeldAfterMeasure::CalcRobotPostureCoors(vector<XI_POINT> vtOutPoints, double dRX, double dRY, double dStepDis, double dStChangePostureDis, double dStChangeAngle,
	double dEdChangePostureDis, double dEdChangeAngle, bool bStartType, bool bEndType, double dStartHoleSize, double dEndHoleSize, int& nStartChangeStepNo, int& nEndChangeStepNo, vector<T_ROBOT_COORS>& vtCoutCoors)
{
	vtCoutCoors.clear();
	vector<XI_POINT> vtPosture;
	CHECK_BOOL_RETURN(CalcRobotPostureCoors(vtOutPoints, dRX, dRY, dStepDis, dStChangePostureDis, dStChangeAngle,
		dEdChangePostureDis, dEdChangeAngle, bStartType, bEndType, dStartHoleSize, dEndHoleSize, nStartChangeStepNo, nEndChangeStepNo, vtPosture));
	for (int n = 0; n < vtPosture.size(); n++)
	{
		T_ROBOT_COORS tCoors(vtOutPoints.at(n).x, vtOutPoints.at(n).y, vtOutPoints.at(n).z
			, vtPosture.at(n).x, vtPosture.at(n).y, vtPosture.at(n).z, 0, 0, 0);
		vtCoutCoors.push_back(tCoors);
	}
	return  true;
}

//bool WeldAfterMeasure::ModelMatching(double dCorasrMatrix[][4], std::string ModelFileName, int nWeldPartNo, std::string SaveRoute_weldline_point, double dFineMatrix[][4])
//{
//	XiAlgorithm* m_XiAlgorithm;
//	m_XiAlgorithm = new XiAlgorithm;
//
//	T_ALGORITHM_LINESEGMENT tLineSegment;
//	std::vector<T_ALGORITHM_LINESEGMENT> vtAllIdealLineSegments;
//	std::vector < std::vector<T_ALGORITHM_LINESEGMENT>> vvtAllIdealLineSegments;
//	std::vector<T_ALGORITHM_LINESEGMENT> vtAllTransIdealLineSegments;
//	std::vector<T_ALGORITHM_LINESEGMENT> vtIdealLineSegments;
//	std::vector <std::vector<T_ALGORITHM_LINESEGMENT>> vvtIdealLineSegments;
//	std::vector<T_ALGORITHM_LINESEGMENT> vtRealLineSegments;
//	std::vector<VTLINESEGMENT> vvtRealLineSegments;
//	std::vector<T_ALGORITHM_POINT> vtRealLineSegmentPoints;
//	std::vector<VTPOINT> vvtRealLineSegmentPoints;
//	std::vector<VVTPOINT> vvvtRealLineSegmentPoints;
//
////	double dTransMatrix[4][4];////��ΰ����Ĵ�ƥ����� ��Ϊ��������������
//
//	// 	dTransMatrix[0][0] = 0.99973;      dTransMatrix[0][1] = -0.0220447;  dTransMatrix[0][2] = -0.00736546; dTransMatrix[0][3] = 4567.21;
//	//  dTransMatrix[1][0] = -0.0220953;   dTransMatrix[1][1] = -0.999733;   dTransMatrix[1][2] = -0.00686115; dTransMatrix[1][3] = 1801.35;
//	// 	dTransMatrix[2][0] = -0.00721223;  dTransMatrix[2][1] = 0.00702203;  dTransMatrix[2][2] = -0.99995;    dTransMatrix[2][3] = 1977.68;
//	// 	dTransMatrix[3][0] = 0;            dTransMatrix[3][1] = 0;           dTransMatrix[3][2] = 0;           dTransMatrix[3][3] = 1;
//	//     
//	//dTransMatrix[0][0] = 0.996301;      dTransMatrix[0][1] = -0.0846703;  dTransMatrix[0][2] = -0.0146789; dTransMatrix[0][3] = 1774.06;
//	//dTransMatrix[1][0] = -0.0846435;   dTransMatrix[1][1] = -0.996409;   dTransMatrix[1][2] = 0.00240201; dTransMatrix[1][3] = 1897.72;
//	//dTransMatrix[2][0] = -0.0148296;  dTransMatrix[2][1] = -0.00115066;  dTransMatrix[2][2] = -0.99989;    dTransMatrix[2][3] = 1998.26;
//	//dTransMatrix[3][0] = 0;            dTransMatrix[3][1] = 0;           dTransMatrix[3][2] = 0;           dTransMatrix[3][3] = 1;
//
//	int nPlateNo = 0;
//	int nWeldLineNo = 0;
//	int nIsArc = 0;
//	int nClockWise = 0;
//	double dZSide = 0.0;
//	int nIsLeft = 0;
//	T_ALGORITHM_POINT tCenterPoint;
//	T_ALGORITHM_POINT tStartPoint, tEndPoint;
//	T_ALGORITHM_LINE_DIR tStartVerLineDir, tEndVerLineDir;
//	int nStartPointType, nEndPointType;
//	int nIsDoubleWelding;
//	int nStartCornerType;
//	int nEndCornerType;
//	int nWeldSize;
//	double dStartHoleRadiusSize;
//	double dEndHoleRadiusSize;
//	int nRobotSelect;
//	double dTheoryLength;
//
//	vtIdealLineSegments.clear();
//	FILE* pfInput = fopen(ModelFileName.c_str(), "r");//////�㷨��ȡ����ģ������  ��Ҫ����ǰ������һ��(��ǿ�������������) ���������Ǹ����� nPlateNo
//	while (EOF != fscanf(pfInput, "%d%d%d %lf%d%lf%lf%lf %lf%lf%lf %lf%lf%lf %lf%lf%lf %lf%lf%lf %d%d%d%d %d%d%lf%lf %d%lf",
//		&nPlateNo, &nIsArc, &nClockWise, &dZSide, &nIsLeft, &tCenterPoint.dCoorX, &tCenterPoint.dCoorY, &tCenterPoint.dCoorZ, &tStartPoint.dCoorX,
//		&tStartPoint.dCoorY, &tStartPoint.dCoorZ, &tEndPoint.dCoorX, &tEndPoint.dCoorY, &tEndPoint.dCoorZ, &tStartVerLineDir.dDirX, &tStartVerLineDir.dDirY,
//		&tStartVerLineDir.dDirZ, &tEndVerLineDir.dDirX, &tEndVerLineDir.dDirY, &tEndVerLineDir.dDirZ, &nStartPointType,
//		&nEndPointType, &nIsDoubleWelding, &nStartCornerType, &nEndCornerType, &nWeldSize, &dStartHoleRadiusSize, &dEndHoleRadiusSize,
//		&nRobotSelect, &dTheoryLength))
//	{
//		double dIncludeAngle = m_XiAlgorithm->VectorsInnerAngle(tStartVerLineDir.dDirX, tStartVerLineDir.dDirY, tStartVerLineDir.dDirZ, 0.0, 0.0, 1.0);
//
//		tLineSegment.tStartPoint.dCoorX = tStartPoint.dCoorX;
//		tLineSegment.tStartPoint.dCoorY = tStartPoint.dCoorY;
//		tLineSegment.tStartPoint.dCoorZ = tStartPoint.dCoorZ;
//		tLineSegment.tEndPoint.dCoorX = tEndPoint.dCoorX;
//		tLineSegment.tEndPoint.dCoorY = tEndPoint.dCoorY;
//		tLineSegment.tEndPoint.dCoorZ = tEndPoint.dCoorZ;
//		tLineSegment.nPartNo = nPlateNo;
//		vtAllIdealLineSegments.push_back(tLineSegment);
//
//		if (fabs(tEndPoint.dCoorZ - tStartPoint.dCoorZ) < 10.0 && nIsArc == 0 && fabs(dIncludeAngle - 45.0) < 5.0)
//		{
//			vtIdealLineSegments.push_back(tLineSegment);
//		}
//	}
//	fclose(pfInput);
//	
//	int nPartNo = 0;
//	int nLastPartNo = 0;
//	int nLastWeldLineNo = 0;
//	T_ALGORITHM_POINT tPoint;
//
//	pfInput = fopen(SaveRoute_weldline_point.c_str(), "r");/////��ΰ���ɺ�������������Ϣ��Ϊ����
//	while (fscanf(pfInput, "%d%d%lf%lf%lf", &nPartNo, &nWeldLineNo, &tPoint.dCoorX, &tPoint.dCoorY, &tPoint.dCoorZ) > 0)
//	{
//		if (nPartNo == nLastPartNo && nLastWeldLineNo == nWeldLineNo)
//		{
//			vtRealLineSegmentPoints.push_back(tPoint);
//			nLastWeldLineNo = nWeldLineNo;
//			nLastPartNo = nPartNo;
//		}
//		else if (nPartNo == nLastPartNo && nLastWeldLineNo != nWeldLineNo)
//		{
//			vvtRealLineSegmentPoints.push_back(vtRealLineSegmentPoints);
//
//			vtRealLineSegmentPoints.clear();
//			nLastWeldLineNo = nWeldLineNo;
//			nLastPartNo = nPartNo;
//		}
//		else if (nPartNo != nLastPartNo)
//		{
//			vvtRealLineSegmentPoints.push_back(vtRealLineSegmentPoints);
//			vvvtRealLineSegmentPoints.push_back(vvtRealLineSegmentPoints);
//
//			vtRealLineSegmentPoints.clear();
//			vvtRealLineSegmentPoints.clear();
//
//			vtRealLineSegmentPoints.push_back(tPoint);
//			nLastWeldLineNo = nWeldLineNo;
//			nLastPartNo = nPartNo;
//		}
//	}
//
//	vvtRealLineSegmentPoints.push_back(vtRealLineSegmentPoints);
//	vvvtRealLineSegmentPoints.push_back(vvtRealLineSegmentPoints);
//	fclose(pfInput);
//
//	std::vector<T_ALGORITHM_LINESEGMENT> vtRealSegment;
//	vtRealSegment.clear();
//	T_ALGORITHM_LINESEGMENT tRealLineSegment;
//	///ʶ�𹤼���  Ŀǰ1���Ǵ�ԲȦ���� 0���ǲ���ԲȦ����
//
//	for (nWeldLineNo = 0; nWeldLineNo < vvvtRealLineSegmentPoints[nWeldPartNo].size(); nWeldLineNo++)
//	{
//		tRealLineSegment.tStartPoint.dCoorX = vvvtRealLineSegmentPoints[nWeldPartNo][nWeldLineNo][0].dCoorX;
//		tRealLineSegment.tStartPoint.dCoorY = vvvtRealLineSegmentPoints[nWeldPartNo][nWeldLineNo][0].dCoorY;
//		tRealLineSegment.tStartPoint.dCoorZ = vvvtRealLineSegmentPoints[nWeldPartNo][nWeldLineNo][0].dCoorZ;
//
//		tRealLineSegment.tEndPoint.dCoorX = vvvtRealLineSegmentPoints[nWeldPartNo][nWeldLineNo][vvvtRealLineSegmentPoints[nWeldPartNo][nWeldLineNo].size() - 1].dCoorX;
//		tRealLineSegment.tEndPoint.dCoorY = vvvtRealLineSegmentPoints[nWeldPartNo][nWeldLineNo][vvvtRealLineSegmentPoints[nWeldPartNo][nWeldLineNo].size() - 1].dCoorY;
//		tRealLineSegment.tEndPoint.dCoorZ = vvvtRealLineSegmentPoints[nWeldPartNo][nWeldLineNo][vvvtRealLineSegmentPoints[nWeldPartNo][nWeldLineNo].size() - 1].dCoorZ;
//
//		vtRealSegment.push_back(tRealLineSegment);
//	}
//
//	CString strName1, strName2, strName3;
//	strName1.Format("RealSegment_%d.txt", nWeldPartNo);
//	strName2.Format("TransAllIdealMergeSegment_%d.txt", nWeldPartNo);
//	strName3.Format("TestAllIdealMergeSegment_%d.txt", nWeldPartNo);
//	FILE* pf1 = fopen(strName1, "w");
//	for (int nId = 0; nId < vtRealSegment.size(); nId++)
//	{
//		fprintf(pf1, "%4d %4d %11.3lf%11.3lf%11.3lf\n", nId, vtRealSegment[nId].nPartNo, vtRealSegment[nId].tStartPoint.dCoorX, vtRealSegment[nId].tStartPoint.dCoorY, vtRealSegment[nId].tStartPoint.dCoorZ);
//		fprintf(pf1, "%4d %4d %11.3lf%11.3lf%11.3lf\n", nId, vtRealSegment[nId].nPartNo, vtRealSegment[nId].tEndPoint.dCoorX, vtRealSegment[nId].tEndPoint.dCoorY, vtRealSegment[nId].tEndPoint.dCoorZ);
//	}
//	fclose(pf1);
//
//	std::vector<T_ALGORITHM_LINESEGMENT> vtTestTransIdealLineSegments;
//	vtTestTransIdealLineSegments.clear();
//	m_XiAlgorithm->TransLineSegments(vtIdealLineSegments, dCorasrMatrix, vtTestTransIdealLineSegments);///������ģ������ת��ʵ�ʵ����� �鿴���
//
//	pf1 = fopen(strName3, "w");
//	for (int nId = 0; nId < vtTestTransIdealLineSegments.size(); nId++)
//	{
//		fprintf(pf1, "%4d %4d %11.3lf%11.3lf%11.3lf\n", nId, vtTestTransIdealLineSegments[nId].nPartNo, vtTestTransIdealLineSegments[nId].tStartPoint.dCoorX, vtTestTransIdealLineSegments[nId].tStartPoint.dCoorY, vtTestTransIdealLineSegments[nId].tStartPoint.dCoorZ);
//		fprintf(pf1, "%4d %4d %11.3lf%11.3lf%11.3lf\n", nId, vtTestTransIdealLineSegments[nId].nPartNo, vtTestTransIdealLineSegments[nId].tEndPoint.dCoorX, vtTestTransIdealLineSegments[nId].tEndPoint.dCoorY, vtTestTransIdealLineSegments[nId].tEndPoint.dCoorZ);
//	}
//	fclose(pf1);
//
//	//double dFineMatrix[4][4];
//	std::vector<T_ALGORITHM_LINESEGMENT> vtIdealTransSegment;
//	double dMatchError;
//	dMatchError = m_XiAlgorithm->GetCoraseToFineMatch(vtIdealLineSegments, vtRealSegment, dCorasrMatrix, dFineMatrix, vtIdealTransSegment);////���þ�ƥ�亯�� ��dTransMatrix ��ΰ��ƥ����� ��dFineMatrix ��ƥ�����   
//	m_XiAlgorithm->TransLineSegments(vtAllIdealLineSegments, dFineMatrix, vtAllTransIdealLineSegments);///������ģ������ת��ʵ�ʵ����� �鿴���
//
//	if (dMatchError>15.0)
//	{
//		XiMessageBox("����������");
//	}
//
//	FILE* pf = fopen(strName2, "w");
//	for (int nId = 0; nId < vtAllTransIdealLineSegments.size(); nId++)
//	{
//		fprintf(pf, "%4d %4d %11.3lf%11.3lf%11.3lf\n", nId, vtAllTransIdealLineSegments[nId].nPartNo, vtAllTransIdealLineSegments[nId].tStartPoint.dCoorX, vtAllTransIdealLineSegments[nId].tStartPoint.dCoorY, vtAllTransIdealLineSegments[nId].tStartPoint.dCoorZ);
//		fprintf(pf, "%4d %4d %11.3lf%11.3lf%11.3lf\n", nId, vtAllTransIdealLineSegments[nId].nPartNo, vtAllTransIdealLineSegments[nId].tEndPoint.dCoorX, vtAllTransIdealLineSegments[nId].tEndPoint.dCoorY, vtAllTransIdealLineSegments[nId].tEndPoint.dCoorZ);
//	}
//	fclose(pf);
//
//	return true;
//}

//bool WeldAfterMeasure::ModelMatching(TransformMatrix* TransMat, std::string ModelFileName1, std::string ModelFileName2, std::string SaveRoute_weldline_point)
//{
//	XiAlgorithm* m_XiAlgorithm;
//	m_XiAlgorithm = new XiAlgorithm;
//
//	T_ALGORITHM_LINESEGMENT tLineSegment;
//	std::vector<T_ALGORITHM_LINESEGMENT> vtAllIdealLineSegments;
//	std::vector < std::vector<T_ALGORITHM_LINESEGMENT>> vvtAllIdealLineSegments;
//	std::vector<T_ALGORITHM_LINESEGMENT> vtAllTransIdealLineSegments;
//	std::vector<T_ALGORITHM_LINESEGMENT> vtIdealLineSegments;
//	std::vector <std::vector<T_ALGORITHM_LINESEGMENT>> vvtIdealLineSegments;
//	std::vector<T_ALGORITHM_LINESEGMENT> vtRealLineSegments;
//	std::vector<VTLINESEGMENT> vvtRealLineSegments;
//	std::vector<T_ALGORITHM_POINT> vtRealLineSegmentPoints;
//	std::vector<VTPOINT> vvtRealLineSegmentPoints;
//	std::vector<VVTPOINT> vvvtRealLineSegmentPoints;
//
//	double dTransMatrix[4][4];////��ΰ����Ĵ�ƥ����� ��Ϊ��������������
//
//	
//
//	// 	dTransMatrix[0][0] = 0.99973;      dTransMatrix[0][1] = -0.0220447;  dTransMatrix[0][2] = -0.00736546; dTransMatrix[0][3] = 4567.21;
//	//  dTransMatrix[1][0] = -0.0220953;   dTransMatrix[1][1] = -0.999733;   dTransMatrix[1][2] = -0.00686115; dTransMatrix[1][3] = 1801.35;
//	// 	dTransMatrix[2][0] = -0.00721223;  dTransMatrix[2][1] = 0.00702203;  dTransMatrix[2][2] = -0.99995;    dTransMatrix[2][3] = 1977.68;
//	// 	dTransMatrix[3][0] = 0;            dTransMatrix[3][1] = 0;           dTransMatrix[3][2] = 0;           dTransMatrix[3][3] = 1;
//	//     
//	dTransMatrix[0][0] = 0.996301;      dTransMatrix[0][1] = -0.0846703;  dTransMatrix[0][2] = -0.0146789; dTransMatrix[0][3] = 1774.06;
//	dTransMatrix[1][0] = -0.0846435;   dTransMatrix[1][1] = -0.996409;   dTransMatrix[1][2] = 0.00240201; dTransMatrix[1][3] = 1897.72;
//	dTransMatrix[2][0] = -0.0148296;  dTransMatrix[2][1] = -0.00115066;  dTransMatrix[2][2] = -0.99989;    dTransMatrix[2][3] = 1998.26;
//	dTransMatrix[3][0] = 0;            dTransMatrix[3][1] = 0;           dTransMatrix[3][2] = 0;           dTransMatrix[3][3] = 1;
//
//	int nPlateNo = 0;
//	int nWeldLineNo = 0;
//	int nIsArc = 0;
//	int nClockWise = 0;
//	double dZSide = 0.0;
//	int nIsLeft = 0;
//	T_ALGORITHM_POINT tCenterPoint;
//	T_ALGORITHM_POINT tStartPoint, tEndPoint;
//	T_ALGORITHM_LINE_DIR tStartVerLineDir, tEndVerLineDir;
//	int nStartPointType, nEndPointType;
//	int nIsDoubleWelding;
//	int nStartCornerType;
//	int nEndCornerType;
//	int nWeldSize;
//	int nStartHoleRadiusSize;
//	int nEndHoleRadiusSize;
//	int nRobotSelect;
//	double nTheoryLength;
//
//	vtIdealLineSegments.clear();
//	FILE* pfInput = fopen(ModelFileName1.c_str(), "r");//////�㷨��ȡ����ģ������  ��Ҫ����ǰ������һ��(��ǿ�������������) ���������Ǹ����� nPlateNo
//	while (fscanf(pfInput, "%d%d%d %lf%d%lf%lf%lf %lf%lf%lf %lf%lf%lf %lf%lf%lf %lf%lf%lf %d%d%d%d %d%d%d%d %d%lf",
//		&nPlateNo, &nIsArc, &nClockWise, &dZSide, &nIsLeft, &tCenterPoint.dCoorX, &tCenterPoint.dCoorY, &tCenterPoint.dCoorZ, &tStartPoint.dCoorX,
//		&tStartPoint.dCoorY, &tStartPoint.dCoorZ, &tEndPoint.dCoorX, &tEndPoint.dCoorY, &tEndPoint.dCoorZ, &tStartVerLineDir.dDirX, &tStartVerLineDir.dDirY,
//		&tStartVerLineDir.dDirZ, &tEndVerLineDir.dDirX, &tEndVerLineDir.dDirY, &tEndVerLineDir.dDirZ, &nStartPointType,
//		&nEndPointType, &nIsDoubleWelding, &nStartCornerType, &nEndCornerType, &nWeldSize, &nStartHoleRadiusSize, &nEndHoleRadiusSize,
//		&nRobotSelect,&nTheoryLength) > 0)
//	{
//		double dIncludeAngle = m_XiAlgorithm->VectorsInnerAngle(tStartVerLineDir.dDirX, tStartVerLineDir.dDirY, tStartVerLineDir.dDirZ, 0.0, 0.0, 1.0);
//
//		tLineSegment.tStartPoint.dCoorX = tStartPoint.dCoorX;
//		tLineSegment.tStartPoint.dCoorY = tStartPoint.dCoorY;
//		tLineSegment.tStartPoint.dCoorZ = tStartPoint.dCoorZ;
//		tLineSegment.tEndPoint.dCoorX = tEndPoint.dCoorX;
//		tLineSegment.tEndPoint.dCoorY = tEndPoint.dCoorY;
//		tLineSegment.tEndPoint.dCoorZ = tEndPoint.dCoorZ;
//		tLineSegment.nPartNo = nPlateNo;
//		vtAllIdealLineSegments.push_back(tLineSegment);
//
//		if (fabs(tEndPoint.dCoorZ - tStartPoint.dCoorZ) < 10.0 && nIsArc == 0 && fabs(dIncludeAngle - 45.0) < 5.0)
//		{
//			vtIdealLineSegments.push_back(tLineSegment);
//		}
//	}
//	fclose(pfInput);
//	vvtIdealLineSegments.push_back(vtIdealLineSegments);
//
//	vtIdealLineSegments.clear();
//	FILE* pfInput2 = fopen(ModelFileName2.c_str(), "r");//////�㷨��ȡ����ģ������  ��Ҫ����ǰ������һ��(��ǿ�������������) ���������Ǹ����� nPlateNo
//	while (fscanf(pfInput2, "%d%d%d%lf%d%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%d%d%d%d%d%d%d%d%d%lf",
//		&nPlateNo, &nIsArc, &nClockWise, &dZSide, &nIsLeft, &tCenterPoint.dCoorX, &tCenterPoint.dCoorY, &tCenterPoint.dCoorZ, &tStartPoint.dCoorX,
//		&tStartPoint.dCoorY, &tStartPoint.dCoorZ, &tEndPoint.dCoorX, &tEndPoint.dCoorY, &tEndPoint.dCoorZ, &tStartVerLineDir.dDirX, &tStartVerLineDir.dDirY,
//		&tStartVerLineDir.dDirZ, &tEndVerLineDir.dDirX, &tEndVerLineDir.dDirY, &tEndVerLineDir.dDirZ, &nStartPointType,
//		&nEndPointType, &nIsDoubleWelding, &nStartCornerType, &nEndCornerType, &nWeldSize, &nStartHoleRadiusSize, &nEndHoleRadiusSize,
//		&nRobotSelect, &nTheoryLength) > 0)
//	{
//		double dIncludeAngle = m_XiAlgorithm->VectorsInnerAngle(tStartVerLineDir.dDirX, tStartVerLineDir.dDirY, tStartVerLineDir.dDirZ, 0.0, 0.0, 1.0);
//
//		tLineSegment.tStartPoint.dCoorX = tStartPoint.dCoorX;
//		tLineSegment.tStartPoint.dCoorY = tStartPoint.dCoorY;
//		tLineSegment.tStartPoint.dCoorZ = tStartPoint.dCoorZ;
//		tLineSegment.tEndPoint.dCoorX = tEndPoint.dCoorX;
//		tLineSegment.tEndPoint.dCoorY = tEndPoint.dCoorY;
//		tLineSegment.tEndPoint.dCoorZ = tEndPoint.dCoorZ;
//		tLineSegment.nPartNo = nPlateNo;
//		vtAllIdealLineSegments.push_back(tLineSegment);
//
//		if (fabs(tEndPoint.dCoorZ - tStartPoint.dCoorZ) < 10.0 && nIsArc == 0 && fabs(dIncludeAngle - 45.0) < 5.0)
//		{
//			vtIdealLineSegments.push_back(tLineSegment);
//		}
//	}
//	fclose(pfInput2);
//
//	vvtIdealLineSegments.push_back(vtIdealLineSegments);
//
//	int nPartNo = 0;
//	int nLastPartNo = 0;
//	int nLastWeldLineNo = 0;
//	T_ALGORITHM_POINT tPoint;
//
//	pfInput = fopen(SaveRoute_weldline_point.c_str(), "r");/////��ΰ���ɺ�������������Ϣ��Ϊ����
//	while (fscanf(pfInput, "%d%d%lf%lf%lf", &nPartNo, &nWeldLineNo, &tPoint.dCoorX, &tPoint.dCoorY, &tPoint.dCoorZ) > 0)
//	{
//		if (nPartNo == nLastPartNo && nLastWeldLineNo == nWeldLineNo)
//		{
//			vtRealLineSegmentPoints.push_back(tPoint);
//			nLastWeldLineNo = nWeldLineNo;
//			nLastPartNo = nPartNo;
//		}
//		else if (nPartNo == nLastPartNo && nLastWeldLineNo != nWeldLineNo)
//		{
//			vvtRealLineSegmentPoints.push_back(vtRealLineSegmentPoints);
//
//			vtRealLineSegmentPoints.clear();
//			nLastWeldLineNo = nWeldLineNo;
//			nLastPartNo = nPartNo;
//		}
//		else if (nPartNo != nLastPartNo)
//		{
//			vvtRealLineSegmentPoints.push_back(vtRealLineSegmentPoints);
//			vvvtRealLineSegmentPoints.push_back(vvtRealLineSegmentPoints);
//
//			vtRealLineSegmentPoints.clear();
//			vvtRealLineSegmentPoints.clear();
//
//			vtRealLineSegmentPoints.push_back(tPoint);
//			nLastWeldLineNo = nWeldLineNo;
//			nLastPartNo = nPartNo;
//		}
//	}
//
//	vvtRealLineSegmentPoints.push_back(vtRealLineSegmentPoints);
//	vvvtRealLineSegmentPoints.push_back(vvtRealLineSegmentPoints);
//	fclose(pfInput);
//
//	for (int PartNo = 0; PartNo < 2; PartNo++)
//	{
//		std::vector<T_ALGORITHM_LINESEGMENT> vtRealSegment;
//		vtRealSegment.clear();
//		T_ALGORITHM_LINESEGMENT tRealLineSegment;
//		/*const */int nWeldPartNo = PartNo;///ʶ�𹤼���  Ŀǰ1���Ǵ�ԲȦ���� 0���ǲ���ԲȦ����
//
//		for (size_t i = 0; i < 4; i++)
//		{
//			for (size_t j = 0; j < 4; j++)
//			{
//				if (0 == PartNo)
//				{
//					//dTransMatrix[i][j] = TransMat->Transform_Matrix_1[i][j];
//				}
//				else
//				{
//					//dTransMatrix[i][j] = TransMat->Transform_Matrix_2[i][j];
//				}
//
//			}
//		}
//		
//		for (nWeldLineNo = 0; nWeldLineNo < vvvtRealLineSegmentPoints[nWeldPartNo].size(); nWeldLineNo++)
//		{
//			tRealLineSegment.tStartPoint.dCoorX = vvvtRealLineSegmentPoints[nWeldPartNo][nWeldLineNo][0].dCoorX;
//			tRealLineSegment.tStartPoint.dCoorY = vvvtRealLineSegmentPoints[nWeldPartNo][nWeldLineNo][0].dCoorY;
//			tRealLineSegment.tStartPoint.dCoorZ = vvvtRealLineSegmentPoints[nWeldPartNo][nWeldLineNo][0].dCoorZ;
//
//			tRealLineSegment.tEndPoint.dCoorX = vvvtRealLineSegmentPoints[nWeldPartNo][nWeldLineNo][vvvtRealLineSegmentPoints[nWeldPartNo][nWeldLineNo].size() - 1].dCoorX;
//			tRealLineSegment.tEndPoint.dCoorY = vvvtRealLineSegmentPoints[nWeldPartNo][nWeldLineNo][vvvtRealLineSegmentPoints[nWeldPartNo][nWeldLineNo].size() - 1].dCoorY;
//			tRealLineSegment.tEndPoint.dCoorZ = vvvtRealLineSegmentPoints[nWeldPartNo][nWeldLineNo][vvvtRealLineSegmentPoints[nWeldPartNo][nWeldLineNo].size() - 1].dCoorZ;
//
//			vtRealSegment.push_back(tRealLineSegment);
//		}
//		CString strName1, strName2;
//		strName1.Format("RealSegment_%d.txt", nWeldPartNo);
//		strName2.Format("TransAllIdealMergeSegment_%d.txt", nWeldPartNo);
//		FILE* pf1 = fopen(strName1, "w");
//		for (int nId = 0; nId < vtRealSegment.size(); nId++)
//		{
//			fprintf(pf1, "%4d%4d%11.3lf%11.3lf%11.3lf\n", nId, vtRealSegment[nId].nPartNo, vtRealSegment[nId].tStartPoint.dCoorX, vtRealSegment[nId].tStartPoint.dCoorY, vtRealSegment[nId].tStartPoint.dCoorZ);
//			fprintf(pf1, "%4d%4d%11.3lf%11.3lf%11.3lf\n", nId, vtRealSegment[nId].nPartNo, vtRealSegment[nId].tEndPoint.dCoorX, vtRealSegment[nId].tEndPoint.dCoorY, vtRealSegment[nId].tEndPoint.dCoorZ);
//		}
//		fclose(pf1);
//
//		double dFineMatrix[4][4];
//		std::vector<T_ALGORITHM_LINESEGMENT> vtIdealTransSegment;
//		double dMatchError;
//		dMatchError = m_XiAlgorithm->GetCoraseToFineMatch(vvtIdealLineSegments[nWeldPartNo], vtRealSegment, dTransMatrix, dFineMatrix, vtIdealTransSegment);////���þ�ƥ�亯�� ��dTransMatrix ��ΰ��ƥ����� ��dFineMatrix ��ƥ�����   
//		m_XiAlgorithm->TransLineSegments(vtAllIdealLineSegments, dFineMatrix, vtAllTransIdealLineSegments);///������ģ������ת��ʵ�ʵ����� �鿴���
//
//		FILE* pf = fopen(strName2, "w");
//		for (int nId = 0; nId < vtAllTransIdealLineSegments.size(); nId++)
//		{
//			fprintf(pf, "%4d%4d%11.3lf%11.3lf%11.3lf\n", nId, vtAllTransIdealLineSegments[nId].nPartNo, vtAllTransIdealLineSegments[nId].tStartPoint.dCoorX, vtAllTransIdealLineSegments[nId].tStartPoint.dCoorY, vtAllTransIdealLineSegments[nId].tStartPoint.dCoorZ);
//			fprintf(pf, "%4d%4d%11.3lf%11.3lf%11.3lf\n", nId, vtAllTransIdealLineSegments[nId].nPartNo, vtAllTransIdealLineSegments[nId].tEndPoint.dCoorX, vtAllTransIdealLineSegments[nId].tEndPoint.dCoorY, vtAllTransIdealLineSegments[nId].tEndPoint.dCoorZ);
//		}
//		fclose(pf);
//
//		for (size_t i = 0; i < 4; i++)
//		{
//			for (size_t j = 0; j < 4; j++)
//			{
//				if (0 == PartNo)
//				{
//					TransMat->Transform_Matrix_1[i][j] = dFineMatrix[i][j];
//				}
//				else
//				{
//					TransMat->Transform_Matrix_2[i][j] = dFineMatrix[i][j];
//				}
//
//			}
//		}
//	}	
//	return true;
//}

bool WeldAfterMeasure::CalcJumpWarpTrack(int nWarpNum, double dBoardChick, double dGunToEyeCompen, double dFlatHorComp, std::vector<T_ROBOT_COORS> vtWeldLineTrack, double dWorkpieceHight,
	std::vector<T_ROBOT_COORS>& vtWarpCoors, std::vector<int>& vtWarpPtType)
{
	// !!!!!!!!���ɰ��ǹ켣------------------------------
			// ���һ���������� p1 
			// ���һ���켣������ӳ�30mm p2 
			// �����෴����ƫ�ư�� p3 
			// p2 �� p3 ������ǹ�켣
			// p1��Բ�ƫ��p4 
			// ������100mm�ĵ���Բ�ƫ����Ϊ�����ʼ�ι켣 
			// �켣��m_pTraceModel->m_vvtTrackWarpCoors[0]
			// ���ͣ�m_pTraceModel->m_vvnTrackWarpPtType[0]
	/* dBoardChick += fabs(m_pScanInit->m_pTraceModel->m_dGunToEyeCompenX)*/; // ���
	T_ROBOT_COORS P1 = vtWeldLineTrack[vtWeldLineTrack.size() - 1];
	T_ROBOT_COORS tDirEnd = P1;
	T_ROBOT_COORS tDirStart;
	T_ROBOT_COORS P2, P3, P4;
	vector<XI_POINT> vtDirPoints(0);// ���ڼ��㷽��
	int nIdx = 0;

	tDirEnd.dX += tDirEnd.dBX;
	tDirEnd.dY += tDirEnd.dBY;
	vtWarpCoors.clear();
	vtWarpPtType.clear();
	T_ROBOT_COORS P;
	for (nIdx = vtWeldLineTrack.size() - 1; nIdx >= 0; nIdx--)
	{
		P = vtWeldLineTrack[nIdx];
		double dDis = TwoPointDis(P.dX + P.dBX, P.dY + P.dBY, P1.dX + P1.dBX, P1.dY + P1.dBY);
		if (dDis > m_dHandEyeDis)
		{
			tDirStart = P;
			tDirStart.dX += tDirStart.dBX;
			tDirStart.dY += tDirStart.dBY;
			break;
		}
		XI_POINT tPt = { P.dX + P.dBX ,P.dY + P.dBY ,P.dZ + P.dBZ };
		vtDirPoints.push_back(tPt);
	}
	// �����β�η���
	T_LINE_PARA tBestLineParamRever ; // ������
	if (!CalcLineParamRansac(tBestLineParamRever,vtDirPoints, 0.7))
	{
		return false;
	}
	reverse(vtDirPoints.begin(), vtDirPoints.end());
	T_LINE_PARA tBestLineParam; // ����
	if (!CalcLineParamRansac(tBestLineParam, vtDirPoints, 0.7))
	{
		return false;
	}
	double dWorkDir = atan2(tBestLineParam.dDirY, tBestLineParam.dDirX) * 180.0 / 3.1415926;

	//jwq���ҳ��ο��˵�
	XI_POINT tDealEnd = {0.0};// { 60.327, 3106.450, 1989.990 };
	if (m_vtTrackTheoryTrack.size() < 2)
	{
		return false;
	}
	double dPStartDis = TwoPointDis(P1.dX + P1.dBX, P1.dY + P1.dBY, m_vtTrackTheoryTrack[0].x, m_vtTrackTheoryTrack[0].y);
	double dPEndDis = TwoPointDis(P1.dX + P1.dBX, P1.dY + P1.dBY, m_vtTrackTheoryTrack.back().x, m_vtTrackTheoryTrack.back().y);
	switch (nWarpNum)
	{
	case 0:
	case 1:
		tDealEnd = dPStartDis > dPEndDis ? m_vtTrackTheoryTrack.back() : m_vtTrackTheoryTrack[0];
		break;
	default:
		return true;//�ر�أ���Ŀ����1��Ϊ����Ҫ����
	}

	//jwq���켣Ԥ��
	if (!TrajectoryLineTrack(vtWarpCoors, tBestLineParam, P1, tDealEnd, 16, 10.0))
	{
		return false;
	}
	for (size_t i = 0; i < vtWarpCoors.size(); i++)
	{
		vtWarpPtType.push_back(E_WELD_WRAP);
	}

	//��ʵ���ǹ켣�����ܳ�
	double dRealDis = TwoPointDis(P1.dX + P1.dBX, P1.dY + P1.dBY, P1.dZ + P1.dBZ, 
		tDealEnd.x, tDealEnd.y, tDealEnd.z);

	//double dMaxDis = 2.0;//������ʵ��ƫ������
	////���ֱ�ߵ���㣨�������꣩
	//T_ROBOT_COORS tRansacLineStart = T_ROBOT_COORS(tBestLineParam.dPointX, tBestLineParam.dDirY, tBestLineParam.dDirZ, 0, 0, 0, 0, 0, 0);
	////��Ϻ����ǹ������㣨�������꣩
	//T_ROBOT_COORS tRansacJumpStart;
	////ƽ��������Ҫ�ľ���
	//double dAdjustLength = 10.0 > dRealDis ? dRealDis : 10.0;
	////ƽ��ƫ�Ƶ�λ����
	//T_ROBOT_COORS tAdjustVec = -tRansacJumpStart;
	//tAdjustVec = tAdjustVec * (1.0 / dRealDis);
	////����
	////ƽ��������
	//double dFirstRealOffsetDis = dRealDis / 16.0;//2.0
	//T_ROBOT_COORS tStartPoint = RobotCoordPosOffset(tRansacLineStart, tBestLineParam, dFirstRealOffsetDis);
	////ʵ�ʰ������
	//P2 = P1;
	//for (int i = 0; i < 16; i++) // ��һ�ΰ���
	//{
	//	P2 = RobotCoordPosOffset(P2, tBestLineParam, dFirstRealOffsetDis);
	//	vtWarpCoors.push_back(P2);
	//	vtWarpPtType.push_back(E_WELD_WRAP);
	//}

	int nFirstNum = vtWarpCoors.size();
	P2 = vtWarpCoors.back();
	P2 = RobotCoordPosOffset(P2, tBestLineParam, 3.5);//jwq��ʱ�����Ǵ�����ƫ����
	//vtWarpCoors.push_back(P2);
	//vtWarpPtType.push_back(E_WELD_WRAP);

	P3 = P2;

	//jwq��ʱ�ĽǶȣ���֤�Գ�
	//P3.dRZ += 180.0;
	//if (P3.dRZ > 180.0)
	//{
	//	P3.dRZ -= 360.0;
	//}
	P3.dRZ = DirAngleToRz(dWorkDir + 90.0 * m_nRobotInstallDir);

	RobotCoordPosOffset(P3, RzToDirAngle(P3.dRZ), dBoardChick + dGunToEyeCompen*2.0);
	//RobotCoordPosOffset(P3, RzToDirAngle(P3.dRZ) + 90.0 * m_nRobotInstallDir, 2.0);

	double dInsertInDis = 0.7 * dBoardChick;//��ǹ���������ľ���
	double dInsertOutDis = 0.5 * dBoardChick;//��ǹ���������ľ���
	dInsertInDis = dInsertInDis > 8.0 ? 8.0 : dInsertInDis;
	dInsertOutDis = dInsertOutDis > 8.0 ? 8.0 : dInsertOutDis;
	dInsertInDis += dGunToEyeCompen;
	dInsertInDis -= dFlatHorComp;
	dInsertOutDis += dGunToEyeCompen;
	dInsertOutDis -= dFlatHorComp;

	vector<T_ROBOT_COORS> vtTmpCoord(0);
	vector<int> vtTmpType(0);
	double dBaseRobotY = vtWeldLineTrack[0].dY; // �ָ�������X�̶�ֵ			
	P2.dBY = P2.dBY + P2.dY - dBaseRobotY;
	P2.dY = dBaseRobotY;
	P3.dBY = P3.dBY + P3.dY - dBaseRobotY;
	P3.dY = dBaseRobotY;
	// ���ǳ�ʼλ��
	vtTmpCoord.clear();
	vtTmpType.clear();
	vtTmpCoord.push_back(P2);
	vtTmpType.push_back(E_WELD_WRAP);
	P2.dRZ = DirAngleToRz(dWorkDir - 90.0 * m_nRobotInstallDir);//jwq��ʱ�����Ǵ��ǶȰ�����
	GenerateWrapBoundTrack(P2, P3, vtTmpCoord, vtTmpType, dWorkpieceHight, dInsertInDis, dInsertOutDis, 6.0, 8.0);
	double dSecondRealOffsetDis = dRealDis / 15.0;//2.0
	for (int i = 0; i < 15; i++) // �ڶ��ΰ���
	{
		P3 = RobotCoordPosOffset(P3, tBestLineParamRever, dSecondRealOffsetDis);
		vtWarpCoors.push_back(P3);
		vtWarpPtType.push_back(E_WELD_WRAP);
	}
	for (int i = vtWeldLineTrack.size() - 2; i >= nIdx; i--) // �����ʼ�ι켣 
	{
		P4 = vtWeldLineTrack[i];
		P4.dRZ = P3.dRZ;
		RobotCoordPosOffset(P4, RzToDirAngle(P4.dRZ), dBoardChick + dGunToEyeCompen*2.0 - m_pTraceModel->m_dGunToEyeCompenX, -1 * m_pTraceModel->m_dGunToEyeCompenZ);
		vtWarpCoors.push_back(P4);
		if (i == vtWeldLineTrack.size() - 2)
		{
			vtWarpPtType.push_back(E_WELD_OPEN_TRACKING);
		}
		else
		{
			vtWarpPtType.push_back(E_WELD_TRACK);
		}
	}
	for (int i = 0; i < vtWarpCoors.size(); i++)
	{
		vtWarpCoors[i].dBY = vtWarpCoors[i].dBY + vtWarpCoors[i].dY - dBaseRobotY;
		vtWarpCoors[i].dY = dBaseRobotY;
	}
	// ������ǹ����
	vtWarpCoors.insert(vtWarpCoors.begin() + nFirstNum, vtTmpCoord.begin(), vtTmpCoord.end());
	vtWarpPtType.insert(vtWarpPtType.begin() + nFirstNum, vtTmpType.begin(), vtTmpType.end());
	// ����Rz�Ƕ�
	for (int i = 0; i < vtWarpCoors.size(); i++)
	{
		if (vtWarpCoors[i].dRZ > 180.0)
		{
			vtWarpCoors[i].dRZ -= 360.0;
		}
		if (vtWarpCoors[i].dRZ < -180.0)
		{
			vtWarpCoors[i].dRZ += 360.0;
		}
	}
	if (vtWarpCoors.size() != vtWarpPtType.size() || vtWarpCoors.size() < 1 || vtWarpPtType.size() < 1)
	{
		return false;
	}
	return true;

}


void WeldAfterMeasure::GenerateWrapBoundTrack(T_ROBOT_COORS tWrapStart, T_ROBOT_COORS tWrapEnd, vector<T_ROBOT_COORS>& vtRobotCoor, vector<int>& vtPtType, 
	double dWorkpieceHight,double dInsertInDis, double dInsertOutDis, double dBackGunPercent, double dDirRotation)
{
	// ��ǹ�յ�󳵺ͻ�е��Y�Ĳ���
	double dDeltaY = tWrapEnd.dBY - tWrapStart.dBY;
	//vtRobotCoor.clear();
	//vtPtType.clear();
	T_ROBOT_COORS tInsertCoord;
	XiAlgorithm tAlg;
	double dDirWrap, dBoardThick;
	dDirRotation = fabs(dDirRotation);
	if (dDirRotation > 90 || dDirRotation < 0)
	{
		dDirRotation = 0.0;
	}
	dBoardThick = TwoPointDis(tWrapStart.dX + tWrapStart.dBX, tWrapStart.dY + tWrapStart.dBY, tWrapStart.dZ,
		tWrapEnd.dX + tWrapEnd.dBX, tWrapEnd.dY + tWrapEnd.dBY, tWrapEnd.dZ);
	dDirWrap = tAlg.CalcArcAngle((tWrapEnd.dX + tWrapEnd.dBX) - (tWrapStart.dX + tWrapStart.dBX), (tWrapEnd.dY + tWrapEnd.dBY) - (tWrapStart.dY + tWrapStart.dBY));

	// ���ǳ�ʼλ��
	tInsertCoord = tWrapStart;
	//vtRobotCoor.push_back(tInsertCoord);
	//vtPtType.push_back(E_WELD_WRAP);
	// ��ʼԭ����ת,��λ���˴����ͣ��
	tInsertCoord.dRZ += dDirRotation * m_nRobotInstallDir;
	vtRobotCoor.push_back(tInsertCoord);
	vtPtType.push_back(E_TRANSITION_ARCOFF_FAST_POINT);
	// ����������յ�ƫ��dInPercentStart�����
	// ����ת����̬������ǣ������ջ�
	RobotCoordPosOffset(tInsertCoord, dDirWrap, dInsertInDis);
	//tInsertCoord.dRZ += dDirRotation * m_nRobotInstallDir;
	vtRobotCoor.push_back(tInsertCoord);
	//vtRobotCoor.push_back(tInsertCoord);
	//vtPtType.push_back(E_WELD_WRAP);
	vtPtType.push_back(E_TRANSITION_ARCOFF_FAST_POINT);
	/* ��ʼ������ɵ� */
	// ̧���������λ��+20 תǹ
	tInsertCoord = tWrapStart;
	RobotCoordPosOffset(tInsertCoord, dDirWrap + 180.0, dBoardThick * dBackGunPercent);
	//20Ӧ��Ϊ����,��δ����
	tInsertCoord.dZ = dWorkpieceHight + m_nRobotInstallDir * 20;
	vtRobotCoor.push_back(tInsertCoord);
	vtPtType.push_back(E_TRANSITION_ARCOFF_FAST_POINT);
	//180��ֳ�2��90����ʱ����ת
	/*
	*	(0��)
			��
	*			(90��)
			��
	*	(180��)
	*/
	RobotCoordPosOffset(tInsertCoord, dDirWrap, dBoardThick * (dBackGunPercent + 0.5));
	RobotCoordPosOffset(tInsertCoord, dDirWrap + 90.0 * m_nRobotInstallDir, fabs(m_pRobotDriver->m_tTools.tGunTool.dX) + 20.0/*m_dGunHeadToGunBarrel*/);
	tInsertCoord.dRZ += 90.0 * m_nRobotInstallDir;
	vtRobotCoor.push_back(tInsertCoord);
	vtPtType.push_back(E_TRANSITION_ARCOFF_FAST_POINT);
	tInsertCoord.dRZ += 90.0 * m_nRobotInstallDir;
	RobotCoordPosOffset(tInsertCoord, dDirWrap, dBoardThick * (dBackGunPercent + 0.5));
	RobotCoordPosOffset(tInsertCoord, dDirWrap - 90.0 * m_nRobotInstallDir, fabs(m_pRobotDriver->m_tTools.tGunTool.dX) + 20.0/*m_dGunHeadToGunBarrel*/);
	vtRobotCoor.push_back(tInsertCoord);
	vtPtType.push_back(E_TRANSITION_ARCOFF_FAST_POINT);
	//��ǹ��
	tInsertCoord.dZ = tWrapEnd.dZ + m_nRobotInstallDir * 20;
	tInsertCoord.dRZ -= dDirRotation * m_nRobotInstallDir;
	vtRobotCoor.push_back(tInsertCoord);
	vtPtType.push_back(E_TRANSITION_ARCOFF_FAST_POINT);
	//��ǹ�� �����Ƕ�ǹ�� Ҳ�Ǻ��ӵ㣩dInPercentEnd�����
	tInsertCoord = tWrapEnd;
	// ��λ���˴���
	RobotCoordPosOffset(tInsertCoord, dDirWrap + 180.0, dInsertOutDis);
	tInsertCoord.dRZ -= dDirRotation * m_nRobotInstallDir;
	//tInsertCoord.dBY -= dDeltaY;
	//tInsertCoord.dY += dDeltaY;
	vtRobotCoor.push_back(tInsertCoord);
	vtPtType.push_back(E_TRANSITION_ARCOFF_FAST_POINT);
	// ����ת����̬����
	tInsertCoord = tWrapEnd;
	// �������ƫ�ƾ���
	//RobotCoordPosOffset(tInsertCoord, RzToDirAngle(tWrapEnd.dRZ), 1.5);
	RobotCoordPosOffset(tInsertCoord, dDirWrap - 90.0 * m_nRobotInstallDir, 1.5);//����λ������ƫ��1mm
	tInsertCoord.dRZ -= dDirRotation * m_nRobotInstallDir;
	//tInsertCoord.dBY -= dDeltaY;
	//tInsertCoord.dY += dDeltaY;
	vtRobotCoor.push_back(tInsertCoord);
	vtPtType.push_back(E_TRANSITION_ARCOFF_FAST_POINT);

	//ԭ��ת�Ƕ�
	tInsertCoord.dRZ += dDirRotation * m_nRobotInstallDir;
	vtRobotCoor.push_back(tInsertCoord);
	vtPtType.push_back(E_TRANSITION_ARCOFF_FAST_POINT);
	return;
}

bool WeldAfterMeasure::TrajectoryLineTrack(std::vector<T_ROBOT_COORS>& vtOutputTrack, T_LINE_PARA tRansacLine, T_ROBOT_COORS tRealStart, XI_POINT tDealEnd, int nPointCount, double dAdjustDis)
{
	vtOutputTrack.clear();

	//��������յ��ͶӰ��
	T_ALGORITHM_POINT tRealStartWorld = { tRealStart.dX + tRealStart.dBX, tRealStart.dY + tRealStart.dBY, tRealStart.dZ + tRealStart.dBZ };//ʵ�ʹ켣���
	T_ALGORITHM_POINT tDealEndWorld = { tDealEnd.x, tDealEnd.y, tDealEnd.z };//����켣�յ�
	T_ALGORITHM_POINT tRansacStart = { 0.0 };//��Ϻ�Ĺ켣���
	T_ALGORITHM_POINT tRansacEnd = { 0.0 };//��Ϻ�Ĺ켣�ص�
	if (!PointtoLineProjection(tRansacLine, tRealStartWorld, tRansacStart)
		|| !PointtoLineProjection(tRansacLine, tDealEndWorld, tRansacEnd))
	{
		return false;
	}

	//�켣�ܳ�
	double dTrackLength = TwoPointDis(tRansacStart.dCoorX, tRansacStart.dCoorY, tRansacStart.dCoorZ, tRansacEnd.dCoorX, tRansacEnd.dCoorY, tRansacEnd.dCoorZ);

	//���·���ƽ�����룬��ֹ����
	dAdjustDis = dAdjustDis > dTrackLength ? dTrackLength : dAdjustDis;

	//����ƽ���յ�
	T_ALGORITHM_POINT tAdjustEnd;
	tAdjustEnd.dCoorX = tRansacStart.dCoorX + (dAdjustDis * tRansacLine.dDirX);
	tAdjustEnd.dCoorY = tRansacStart.dCoorY + (dAdjustDis * tRansacLine.dDirY);
	tAdjustEnd.dCoorZ = tRansacStart.dCoorZ + (dAdjustDis * tRansacLine.dDirZ);

	//ƽ����Ҫ�ĵ���
	int nAdjustCount = dAdjustDis / dTrackLength * nPointCount;
	nAdjustCount = nAdjustCount > 0 ? nAdjustCount : 1;

	//ƽ���켣��������ÿһ��֮�䣩
	T_ALGORITHM_POINT tAdjustVec = { (tAdjustEnd.dCoorX - tRealStartWorld.dCoorX) / (nAdjustCount * 1.0),
		(tAdjustEnd.dCoorY - tRealStartWorld.dCoorY) / (nAdjustCount * 1.0),
		(tAdjustEnd.dCoorZ - tRealStartWorld.dCoorZ) / (nAdjustCount * 1.0) };

	//ƽ����켣��������ÿһ��֮�䣩
	T_ALGORITHM_POINT tAfterAdjustVec = { (tRansacEnd.dCoorX - tAdjustEnd.dCoorX) / ((nPointCount - nAdjustCount) * 1.0),
		(tRansacEnd.dCoorY - tAdjustEnd.dCoorY) / ((nPointCount - nAdjustCount) * 1.0),
		(tRansacEnd.dCoorZ - tAdjustEnd.dCoorZ) / ((nPointCount - nAdjustCount) * 1.0) };

	//����ƽ���켣���ⲿ�᲻�䣩
	T_ROBOT_COORS tRealPos = tRealStart;
	for (size_t i = 0; i < nAdjustCount; i++)
	{
		tRealPos.dX += tAdjustVec.dCoorX;
		tRealPos.dY += tAdjustVec.dCoorY;
		tRealPos.dZ += tAdjustVec.dCoorZ;
		vtOutputTrack.push_back(tRealPos);
	}

	//����ƽ����켣���ⲿ�᲻�䣩
	for (size_t i = 0; i < nPointCount - nAdjustCount; i++)
	{
		tRealPos.dX += tAfterAdjustVec.dCoorX;
		tRealPos.dY += tAfterAdjustVec.dCoorY;
		tRealPos.dZ += tAfterAdjustVec.dCoorZ;
		vtOutputTrack.push_back(tRealPos);
	}

	return int(vtOutputTrack.size()) == nPointCount;
}

bool WAM::WeldAfterMeasure::CalcScanTrack(LineOrCircularArcWeldingLine SeamData, std::vector<T_ROBOT_COORS>& vtMeasureCoord, vector<T_ANGLE_PULSE>& vtMeasurePulse, double& dExAxlePos)
{
	T_ROBOT_COORS tmpCoor;
	double gunAngle = 0;
	vector<T_ROBOT_COORS> vtTmpCoors;
	vtTmpCoors.clear();
	vtMeasureCoord.clear(); // ת����������ߺ�
	vtMeasurePulse.clear();
	CvPoint3D64f tTotalValue = { 0 }; // ͳ��XYZֵ ȷ���ⲿ��λ��
	CvPoint3D64f tTotalDirValue = { 0 };
	double dPartMaxHeight = -99999.0; // ������¼������ߵ�Zֵ ���Ҽ�¼������͵�
	double dPartMinHeight = 99999.0; // ������¼������׵�Zֵ ���Ҽ�¼������ߵ�

	tTotalValue.x += SeamData.StartPoint.x;
	tTotalValue.x += SeamData.EndPoint.x;
	tTotalValue.y += SeamData.StartPoint.y;
	tTotalValue.y += SeamData.EndPoint.y;
	tTotalValue.z += SeamData.StartPoint.z;
	tTotalValue.z += SeamData.EndPoint.z;

	dExAxlePos = tTotalValue.y / 2;
	//����뾶
	double dRadius = sqrt(pow((SeamData.StartPoint.x - SeamData.CenterPoint.x), 2) + pow((SeamData.StartPoint.y - SeamData.CenterPoint.y), 2) + pow((SeamData.StartPoint.z - SeamData.CenterPoint.z), 2));
	//������㷨��
	SeamData.StartNormalVector.x = (SeamData.StartPoint.x - SeamData.CenterPoint.x) / dRadius;
	SeamData.StartNormalVector.y = (SeamData.StartPoint.y - SeamData.CenterPoint.y) / dRadius;
	SeamData.StartNormalVector.z = (SeamData.StartPoint.z - SeamData.CenterPoint.z) / dRadius;
	//�����յ㷨��
	SeamData.EndNormalVector.x = (SeamData.EndPoint.x - SeamData.CenterPoint.x) / dRadius;
	SeamData.EndNormalVector.y = (SeamData.EndPoint.y - SeamData.CenterPoint.y) / dRadius;
	SeamData.EndNormalVector.z = (SeamData.EndPoint.z - SeamData.CenterPoint.z) / dRadius;
	//�������յ��Z����
	double disStartToEnd = sqrt(pow((SeamData.StartPoint.x - SeamData.EndPoint.x), 2) + pow((SeamData.StartPoint.y - SeamData.EndPoint.y), 2) + pow((SeamData.StartPoint.z - SeamData.EndPoint.z), 2));
	double StartToEndVectorDz = (SeamData.EndPoint.z - SeamData.StartPoint.z) / disStartToEnd;
	//�������յ���������
	double startToEndRadian = atan2(SeamData.EndNormalVector.y, SeamData.EndNormalVector.x) - atan2(SeamData.StartNormalVector.y, SeamData.StartNormalVector.x);
	if (startToEndRadian < 0)
	{
		startToEndRadian += 2 * PI;
	}
	//���㻡���ܳ�
	double disRadian = startToEndRadian * dRadius;
	//���������㣬��ֵΪ10mm��������ʱ�뷽�򺸽ӣ�
	tmpCoor.dX = SeamData.StartPoint.x + SeamData.StartNormalVector.x * 10;
	tmpCoor.dY = SeamData.StartPoint.y + SeamData.StartNormalVector.y * 10;
	tmpCoor.dZ = SeamData.StartPoint.z + SeamData.StartNormalVector.z * 10;
	tmpCoor.dRX = m_dNormalWeldRx;
	tmpCoor.dBY = m_dNormalWeldRy;
	//��㷨��ת�����淨��
	double vecStartSideX = SeamData.StartNormalVector.x * cos(PI / 2) - SeamData.StartNormalVector.y * sin(PI / 2);
	double vecStartSideY = SeamData.StartNormalVector.y * cos(PI / 2) + SeamData.StartNormalVector.x * sin(PI / 2);
	gunAngle = atan2(vecStartSideY, vecStartSideX) * 180 / PI;
	tmpCoor.dBZ = DirAngleToRz(gunAngle);
	vtTmpCoors.push_back(tmpCoor);
	//����25mmΪת�ǹ������򣬼��㱾��ת�Ǻ������
	double sumRunRadian = 25;
	double disRunRadian = sumRunRadian * startToEndRadian / disRadian;
	vecStartSideX = SeamData.StartNormalVector.x * cos(disRunRadian) - SeamData.StartNormalVector.y * sin(disRunRadian);
	vecStartSideY = SeamData.StartNormalVector.y * cos(disRunRadian) + SeamData.StartNormalVector.x * sin(disRunRadian);
	tmpCoor.dX = dRadius * vecStartSideX;
	tmpCoor.dY = dRadius * vecStartSideY;
	tmpCoor.dZ = tmpCoor.dZ + StartToEndVectorDz * (SeamData.EndPoint.z - SeamData.StartPoint.z) * sumRunRadian / disRadian;
	gunAngle = atan2(vecStartSideY, vecStartSideX) * 180 / PI;
	tmpCoor.dBZ = DirAngleToRz(gunAngle);
	vtMeasureCoord.push_back(tmpCoor);
	//����2mmһ����
	disRunRadian = 0;
	while (sumRunRadian < (disRadian - 25))
	{
		sumRunRadian += 2;
		disRunRadian = sumRunRadian * startToEndRadian / disRadian;
		vecStartSideX = SeamData.StartNormalVector.x * cos(disRunRadian) - SeamData.StartNormalVector.y * sin(disRunRadian);
		vecStartSideY = SeamData.StartNormalVector.y * cos(disRunRadian) + SeamData.StartNormalVector.x * sin(disRunRadian);
		tmpCoor.dX = dRadius * vecStartSideX;
		tmpCoor.dY = dRadius * vecStartSideY;
		tmpCoor.dZ = tmpCoor.dZ + StartToEndVectorDz * (SeamData.EndPoint.z - SeamData.StartPoint.z) * sumRunRadian / disRadian;
		tmpCoor.dRX = m_dNormalWeldRx;
		tmpCoor.dBY = m_dNormalWeldRy;
		gunAngle = atan2(vecStartSideY, vecStartSideX) * 180 / PI;
		tmpCoor.dBZ = DirAngleToRz(gunAngle);
		vtTmpCoors.push_back(tmpCoor);
	}
	//�������һ���յ�����
	tmpCoor.dX = SeamData.EndPoint.x + SeamData.EndNormalVector.x * 10;
	tmpCoor.dY = SeamData.EndPoint.y + SeamData.EndNormalVector.y * 10;
	tmpCoor.dZ = SeamData.EndPoint.z + SeamData.EndNormalVector.z * 10;
	tmpCoor.dRX = m_dNormalWeldRx;
	tmpCoor.dBY = m_dNormalWeldRy;
	vecStartSideX = SeamData.EndNormalVector.x * cos(-PI / 2) - SeamData.EndNormalVector.y * sin(-PI / 2);
	vecStartSideY = SeamData.EndNormalVector.y * cos(-PI / 2) + SeamData.EndNormalVector.x * sin(-PI / 2);
	gunAngle = atan2(vecStartSideY, vecStartSideX) * 180 / PI;
	tmpCoor.dBZ = DirAngleToRz(gunAngle);
	vtTmpCoors.push_back(tmpCoor);

	//��ǹ����ת�������
	for (int i = 0; i < vtTmpCoors.size(); i++)
	{
		if (!m_pRobotDriver->MoveToolByWeldGun(vtTmpCoors[i], m_pRobotDriver->m_tTools.tGunTool, vtTmpCoors[i], m_pRobotDriver->m_tTools.tCameraTool, tmpCoor))
		{
			return false;
		}
		vtMeasureCoord.push_back(tmpCoor);
	}

	//תΪ��������
	return CalcContinuePulseForWeld(vtMeasureCoord, vtMeasurePulse, false);
}

std::vector<T_ROBOT_COORS> WAM::WeldAfterMeasure::CalLineDividePoints(T_ROBOT_COORS tStart, T_ROBOT_COORS tEnd, double dStep)
{
	//��㵽�յ������
	T_ROBOT_COORS dStartToEnd = tEnd - tStart;

	//����
	double dLength = sqrt(dStartToEnd.dX * dStartToEnd.dX + dStartToEnd.dY * dStartToEnd.dY);

	//����
	int nPiece = dLength / dStep + 0.5;
	dStep = dLength / (nPiece * 1.0);
	T_ROBOT_COORS tEveryVect = dStartToEnd * (1.0 / nPiece);

	//�ȷ�
	std::vector<T_ROBOT_COORS> vtPoint;
	vtPoint.push_back(tStart);
	for (int i = 1; i < nPiece; i++)
	{
		vtPoint.push_back(tStart + tEveryVect * i);
	}
	vtPoint.push_back(tEnd);
	return vtPoint;
}

std::vector<T_ROBOT_COORS> WAM::WeldAfterMeasure::CalArcDividePoints(T_ROBOT_COORS tStart, T_ROBOT_COORS tEnd, T_ROBOT_COORS tCenter, double dStep)
{
	//Բ�ĵ���������
	T_ROBOT_COORS dCenterToStart = tStart - tCenter;

	//Բ�ĵ��յ������
	T_ROBOT_COORS dCenterToEnd = tEnd - tCenter;

	//�뾶
	double dRadius1 = sqrt(dCenterToStart.dX * dCenterToStart.dX + dCenterToStart.dY * dCenterToStart.dY);
	double dRadius2 = sqrt(dCenterToEnd.dX * dCenterToEnd.dX + dCenterToEnd.dY * dCenterToEnd.dY);
	double dRadius = dRadius1 > dRadius2 ? dRadius1 : dRadius2;
	dRadius += 0.001;

	//Բ�ĵ��е������
	T_ROBOT_COORS tCenterToMid = (dCenterToStart + dCenterToEnd) * 0.5;

	//�Ҹߣ����޲����Բ��Բ����
	double dMidHeight = dRadius - sqrt(tCenterToMid.dX * tCenterToMid.dX + tCenterToMid.dY * tCenterToMid.dY);

	//�����Ƕ�
	double dStartAngle = dCenterToStart.dX / dRadius;
	//dStartAngle = dStartAngle > 1.0 ? 1.0 : dStartAngle;
	dStartAngle = acos(dStartAngle);
	if (-dCenterToStart.dY > 0)
	{
		dStartAngle = 2 * PI - dStartAngle;
	}

	//���յ�Ƕ�
	double dEndAngle = dCenterToEnd.dX / dRadius;
	//dEndAngle = dEndAngle > 1.0 ? 1.0 : dEndAngle;
	dEndAngle = acos(dEndAngle);
	if (-dCenterToEnd.dY > 0)
	{
		dEndAngle = 2 * PI - dEndAngle;
	}

	//Բ����Ӧ�Ļ���
	bool bCrossZero = true;//�Ƿ�Խ�����
	double dTotalAngle = dEndAngle - dStartAngle;
	bool bValIsMoreThanPI = fabs(dTotalAngle) > PI;
	bool bAngleIsMoreThanPI = dMidHeight > dRadius;
	if (bValIsMoreThanPI && bAngleIsMoreThanPI
		|| (!bValIsMoreThanPI) && (!bAngleIsMoreThanPI))//Բ����������
	{
		bCrossZero = false;
	}
	else if (dTotalAngle > 0)//������ȣ�dAngle2 > dAngle1
	{
		dTotalAngle = -2 * PI + dTotalAngle;
	}
	else//������ȣ�dAngle1 > dAngle2
	{
		dTotalAngle = 2 * PI + dTotalAngle;
	}

	//���Ȳ���
	double dEveryAngle = dStep / dRadius;

	//�ȷַ���
	int nPiece = fabs(dTotalAngle / dEveryAngle + 0.5);

	//���Ȳ�������
	dEveryAngle = dTotalAngle / (nPiece * 1.0);

	//�߶Ȳ���
	double dHeight = (tEnd.dZ - tStart.dZ) / (nPiece * 1.0);

	//�ȷ�
	std::vector<T_ROBOT_COORS> vtPoint;
	for (int i = 0; i < nPiece; i++)
	{
		T_ROBOT_COORS tPoint;
		double dAngle = dStartAngle + dEveryAngle * i;
		tPoint.dX = tCenter.dX + dRadius * cos(dAngle);
		tPoint.dY = tCenter.dY + dRadius * sin(dAngle);
		tPoint.dZ = tStart.dZ + dHeight * i;
		vtPoint.push_back(tPoint);
	}
	vtPoint.push_back(tEnd);
	return vtPoint;
}
bool WAM::WeldAfterMeasure::CalcArcMoveTrackBySeamData(int nRobotNo, LineOrCircularArcWeldingLine SeamData, std::vector<T_ROBOT_COORS>& vtMeasureCoord, vector<T_ANGLE_PULSE>& vtMeasurePulse, double dStep, int& nStartPointCount, int& nEndPointCount)
{
	//// �������Ų�ͬ
	double dCurCarPos = 0.0; m_ptUnit->GetExPositionDis(m_ptUnit->m_nLinedScanAxisNo);
	double dCurExPos = m_ptUnit->GetExPositionDis(m_ptUnit->m_nMeasureAxisNo);
	SeamData.StartPoint.x += dCurCarPos;
	SeamData.EndPoint.x += dCurCarPos;
	SeamData.CenterPoint.x += dCurCarPos;
	SeamData.StartPoint.y -= dCurExPos;
	SeamData.EndPoint.y -= dCurExPos;
	SeamData.CenterPoint.y -= dCurExPos;
	auto StartPoint = /*m_ptUnit->TransCoor_Gantry2RobotNew*/(SeamData.StartPoint);
	auto EndPoint = /*m_ptUnit->TransCoor_Gantry2RobotNew*/(SeamData.EndPoint);
	auto CenterPoint = /*m_ptUnit->TransCoor_Gantry2RobotNew*/(SeamData.CenterPoint);
	T_ROBOT_COORS tStart;
	tStart.dX = StartPoint.x;
	tStart.dY = StartPoint.y;
	tStart.dZ = StartPoint.z;
	T_ROBOT_COORS tEnd;
	tEnd.dX = EndPoint.x;
	tEnd.dY = EndPoint.y;
	tEnd.dZ = EndPoint.z;
	T_ROBOT_COORS tCenter;
	tCenter.dX = CenterPoint.x;
	tCenter.dY = CenterPoint.y;
	tCenter.dZ = CenterPoint.z;

	double dStartSectionLength = 50.0;
	double dArcRotateAngle = 55.0;
	double dLineRotateAngle = 0.0;
	vtMeasurePulse.clear();

	//Բ�ĵ���������
	T_ROBOT_COORS dCenterToStart = tStart - tCenter;

	//Բ�ĵ��յ������
	T_ROBOT_COORS dCenterToEnd = tEnd - tCenter;

	//�뾶
	double dRadius1 = sqrt(dCenterToStart.dX * dCenterToStart.dX + dCenterToStart.dY * dCenterToStart.dY);
	double dRadius2 = sqrt(dCenterToEnd.dX * dCenterToEnd.dX + dCenterToEnd.dY * dCenterToEnd.dY);
	double dRadius = dRadius1 > dRadius2 ? dRadius1 : dRadius2;

	//���ɵȼ���켣
	std::vector<T_ROBOT_COORS> vtPoint = CalArcDividePoints(tStart, tEnd, tCenter, dStep);

	//������ʼ��
	T_ROBOT_COORS tLineStartPoint = dCenterToStart * (dStartSectionLength / dRadius) + tStart;
	std::vector<T_ROBOT_COORS> vtStartLinePoint = CalLineDividePoints(tLineStartPoint, tStart, dStep);

	//������ֹ��
	T_ROBOT_COORS tLineEndPoint = dCenterToEnd * (dStartSectionLength / dRadius) + tEnd;
	std::vector<T_ROBOT_COORS> vtEndPoint = CalLineDividePoints(tEnd, tLineEndPoint, dStep);

	//RZ�仯����
	int nStepCount = dStartSectionLength / dStep + 0.5;
	int nArcStepCount = nStepCount;
	//if (nArcStepCount >= vtPoint.size() - nArcStepCount)//Բ��������
	{
		nArcStepCount = vtPoint.size() / 2.0;
	}
	nStartPointCount = nStepCount;
	nEndPointCount = nStepCount;

	//RZ�仯������ֱ��Բ�����Գе�dRotateAngle/2��
	double dLineRZStep = dLineRotateAngle / (nStepCount * 1.0);
	double dArcRZStep = dArcRotateAngle / (nArcStepCount * 1.0);

	//�Ƕȸ�ֵ
	double dStartGunAngle = atan2(dCenterToStart.dY, dCenterToStart.dX) * 180 / PI - dLineRotateAngle - dArcRotateAngle;
	double dEndGunAngle = atan2(dCenterToEnd.dY, dCenterToEnd.dX) * 180 / PI + dLineRotateAngle + dArcRotateAngle;
	//��ʼ��ֱ�߽Ƕ�
	for (size_t i = 0; i < vtStartLinePoint.size(); i++)
	{
		vtStartLinePoint[i].dRX = 0;
		vtStartLinePoint[i].dRY = -45;
		vtStartLinePoint[i].dRZ = DirAngleToRz(dStartGunAngle) + i * dLineRZStep;
	}
	//������ʼ�νǶ�
	for (size_t i = 0; i < nArcStepCount; i++)
	{
		double dGunAngle = atan2(vtPoint[i].dY - tCenter.dY, vtPoint[i].dX - tCenter.dX) * 180 / PI;
		vtPoint[i].dRX = 0;
		vtPoint[i].dRY = -45;
		vtPoint[i].dRZ = DirAngleToRz(dGunAngle) - (nArcStepCount - i) * dArcRZStep;
	}
	//�����м�νǶȣ�����ǰ���غϣ�
	for (size_t i = nArcStepCount; i < vtPoint.size() - nArcStepCount; i++)
	{
		double dGunAngle = atan2(vtPoint[i].dY - tCenter.dY, vtPoint[i].dX - tCenter.dX) * 180 / PI;
		vtPoint[i].dRX = 0;
		vtPoint[i].dRY = -45;
		vtPoint[i].dRZ = DirAngleToRz(dGunAngle);
	}
	//���߽����νǶ�
	for (size_t i = vtPoint.size() - 1 - nArcStepCount; i < vtPoint.size(); i++)
	{
		double dGunAngle = atan2(vtPoint[i].dY - tCenter.dY, vtPoint[i].dX - tCenter.dX) * 180 / PI;
		vtPoint[i].dRX = 0;
		vtPoint[i].dRY = -45;
		vtPoint[i].dRZ = DirAngleToRz(dGunAngle) + (i + 1 + nArcStepCount - vtPoint.size()) * dArcRZStep;
	}
	//������ֱ�߽Ƕ�
	for (size_t i = 0; i < vtEndPoint.size(); i++)
	{
		vtEndPoint[i].dRX = 0;
		vtEndPoint[i].dRY = -45;
		vtEndPoint[i].dRZ = DirAngleToRz(dEndGunAngle) - (vtEndPoint.size() - i) * dLineRZStep;
	}

	//�ϲ�����
	vtMeasureCoord = vtStartLinePoint;
	vtMeasureCoord.insert(vtMeasureCoord.end(), vtPoint.begin() + 1, vtPoint.end());
	vtMeasureCoord.insert(vtMeasureCoord.end(), vtEndPoint.begin() + 1, vtEndPoint.end());

	//ȷ���ⲿ��
	//double dMaxBY = -99999;
	//double dMinBY = 99999;
	//for (size_t i = 0; i < vtMeasureCoord.size(); i++)
	//{
	//	if (vtMeasureCoord[i].dY > dMaxBY)
	//	{
	//		dMaxBY = vtMeasureCoord[i].dY;
	//	}
	//	if (vtMeasureCoord[i].dY < dMinBY)
	//	{
	//		dMinBY = vtMeasureCoord[i].dY;
	//	}
	//}
	//double dBY = (dMaxBY + dMinBY) / 2.0;
	for (size_t i = 0; i < vtMeasureCoord.size(); i++)
	{
		if (vtMeasureCoord[i].dRZ > 180.0)
		{
			vtMeasureCoord[i].dRZ -= 360.0;
		}
		if (vtMeasureCoord[i].dRZ < -180.0)
		{
			vtMeasureCoord[i].dRZ += 360.0;
		}
		//// ������ⲿ��
		//vtMeasureCoord[i].dBX = 0.0;
		//vtMeasureCoord[i].dBY = vtMeasureCoord[i].dY - 500.0 + dCurExPos;
		//vtMeasureCoord[i].dBZ = 0.0;
		//vtMeasureCoord[i].dY = 500.0;
		////���������᲻���ⲿ��
		vtMeasureCoord[i].dBX = 0.0;
		vtMeasureCoord[i].dBY = dCurExPos;
		vtMeasureCoord[i].dBZ = 0.0;
	}

	//��ǹ����ת�������
	XiBase::CXiOpenFile file;
	file.XI_fopen(".\\ceshi.txt", "w");
	for (int i = 0; i < vtMeasureCoord.size(); i++)
	{
		file.XI_fprintf("%12.4lf %12.4lf %12.4lf %12.4lf %12.4lf %12.4lf %12.4lf %12.4lf %12.4lf\n",
			vtMeasureCoord[i].dX, vtMeasureCoord[i].dY, vtMeasureCoord[i].dZ,
			vtMeasureCoord[i].dRX, vtMeasureCoord[i].dRY, vtMeasureCoord[i].dRZ,
			vtMeasureCoord[i].dBX, vtMeasureCoord[i].dBY, vtMeasureCoord[i].dBZ);
		T_ROBOT_COORS tmpCoor;
		if (!m_pRobotDriver->MoveToolByWeldGun(vtMeasureCoord[i], m_pRobotDriver->m_tTools.tGunTool, vtMeasureCoord[i], m_pRobotDriver->m_tTools.tCameraTool, tmpCoor))
		{
			return false;
		}
		vtMeasureCoord[i] = tmpCoor;
	}
	file.XI_fclose();

	//תΪ��������
	if (!CalcContinuePulseForWeld(vtMeasureCoord, vtMeasurePulse, false))
	{
		return false;
	}
	return true;
}
#if 0
bool WAM::WeldAfterMeasure::CalcArcMoveTrackBySeamData(int nRobotNo, LineOrCircularArcWeldingLine SeamData, std::vector<T_ROBOT_COORS>& vtMeasureCoord, vector<T_ANGLE_PULSE>& vtMeasurePulse, double dStep, int& nStartPointCount, int& nEndPointCount)
{
	double dCurCarPos = m_ptUnit->GetExPositionDis(m_ptUnit->m_nLinedScanAxisNo);
	double dCurExPos = m_ptUnit->GetExPositionDis(m_ptUnit->m_nMeasureAxisNo);
	SeamData.StartPoint.x += dCurCarPos;
	SeamData.EndPoint.x += dCurCarPos;
	SeamData.CenterPoint.x += dCurCarPos;
	SeamData.StartPoint.y -= dCurExPos;
	SeamData.EndPoint.y -= dCurExPos;
	SeamData.CenterPoint.y -= dCurExPos;
	auto StartPoint = /*m_ptUnit->TransCoor_Gantry2RobotNew*/(SeamData.StartPoint);
	auto EndPoint = /*m_ptUnit->TransCoor_Gantry2RobotNew*/(SeamData.EndPoint);
	auto CenterPoint = /*m_ptUnit->TransCoor_Gantry2RobotNew*/(SeamData.CenterPoint);
	T_ROBOT_COORS tStart;
	tStart.dX = StartPoint.x;
	tStart.dY = StartPoint.y;
	tStart.dZ = StartPoint.z;
	T_ROBOT_COORS tEnd;
	tEnd.dX = EndPoint.x;
	tEnd.dY = EndPoint.y;
	tEnd.dZ = EndPoint.z;
	T_ROBOT_COORS tCenter;
	tCenter.dX = CenterPoint.x;
	tCenter.dY = CenterPoint.y;
	tCenter.dZ = CenterPoint.z;

	double dStartSectionLength = 60.0;
	double dRotateAngle = 60.0;
	vtMeasurePulse.clear();

	//Բ�ĵ���������
	T_ROBOT_COORS dCenterToStart = tStart - tCenter;

	//Բ�ĵ��յ������
	T_ROBOT_COORS dCenterToEnd = tEnd - tCenter;

	//�뾶
	double dRadius = sqrt(dCenterToStart.dX * dCenterToStart.dX + dCenterToStart.dY * dCenterToStart.dY);

	//���ɵȼ���켣
	std::vector<T_ROBOT_COORS> vtPoint = CalArcDividePoints(tStart, tEnd, tCenter, dStep);

	//������ʼ��
	T_ROBOT_COORS tLineStartPoint = dCenterToStart * (dStartSectionLength / dRadius) + tStart;
	std::vector<T_ROBOT_COORS> vtStartLinePoint = CalLineDividePoints(tLineStartPoint, tStart, dStep);

	//������ֹ��
	T_ROBOT_COORS tLineEndPoint = dCenterToEnd * (dStartSectionLength / dRadius) + tEnd;
	std::vector<T_ROBOT_COORS> vtEndPoint = CalLineDividePoints(tEnd, tLineEndPoint, dStep);

	//RZ�仯����
	int nStepCount = dStartSectionLength / dStep + 0.5;
	nStartPointCount = nStepCount;
	nEndPointCount = nStepCount;

	//RZ�仯������ֱ��Բ�����Գе�20��
	double dRZStep = dRotateAngle / (nStepCount * 1.0) / 2;

	//�Ƕȸ�ֵ
	double dStartGunAngle = atan2(dCenterToStart.dY, dCenterToStart.dX) * 180 / PI - dRotateAngle;
	double dEndGunAngle = atan2(dCenterToEnd.dY, dCenterToEnd.dX) * 180 / PI + dRotateAngle;
	for (size_t i = 0; i < vtStartLinePoint.size(); i++)
	{
		vtStartLinePoint[i].dRX = 0;
		vtStartLinePoint[i].dRY = -45;
		vtStartLinePoint[i].dRZ = DirAngleToRz(dStartGunAngle) + i * dRZStep;
	}
	for (size_t i = 0; i < nStepCount; i++)
	{
		double dGunAngle = atan2(vtPoint[i].dY - tCenter.dY, vtPoint[i].dX - tCenter.dX) * 180 / PI;
		vtPoint[i].dRX = 0;
		vtPoint[i].dRY = -45;
		vtPoint[i].dRZ = DirAngleToRz(dGunAngle) - (nStepCount - i) * dRZStep;
	}
	for (size_t i = nStepCount; i < vtPoint.size() - nStepCount; i++)
	{
		double dGunAngle = atan2(vtPoint[i].dY - tCenter.dY, vtPoint[i].dX - tCenter.dX) * 180 / PI;
		vtPoint[i].dRX = 0;
		vtPoint[i].dRY = -45;
		vtPoint[i].dRZ = DirAngleToRz(dGunAngle);
	}
	for (size_t i = vtPoint.size() - 1 - nStepCount; i < vtPoint.size(); i++)
	{
		double dGunAngle = atan2(vtPoint[i].dY - tCenter.dY, vtPoint[i].dX - tCenter.dX) * 180 / PI;
		vtPoint[i].dRX = 0;
		vtPoint[i].dRY = -45;
		vtPoint[i].dRZ = DirAngleToRz(dGunAngle) + (i + 1 + nStepCount - vtPoint.size())*dRZStep;
	}
	for (size_t i = 0; i < vtEndPoint.size(); i++)
	{
		vtEndPoint[i].dRX = 0;
		vtEndPoint[i].dRY = -45;
		vtEndPoint[i].dRZ = DirAngleToRz(dEndGunAngle) - (vtEndPoint.size() - i) * dRZStep;
	}

	//�ϲ�����
	vtMeasureCoord = vtStartLinePoint;
	vtMeasureCoord.insert(vtMeasureCoord.end(), vtPoint.begin() + 1, vtPoint.end());
	vtMeasureCoord.insert(vtMeasureCoord.end(), vtEndPoint.begin() + 1, vtEndPoint.end());

	//ȷ���ⲿ��
	//double dMaxBY = -99999;
	//double dMinBY = 99999;
	//for (size_t i = 0; i < vtMeasureCoord.size(); i++)
	//{
	//	if (vtMeasureCoord[i].dY > dMaxBY)
	//	{
	//		dMaxBY = vtMeasureCoord[i].dY;
	//	}
	//	if (vtMeasureCoord[i].dY < dMinBY)
	//	{
	//		dMinBY = vtMeasureCoord[i].dY;
	//	}
	//}
	//double dBY = (dMaxBY + dMinBY) / 2.0;
	for (size_t i = 0; i < vtMeasureCoord.size(); i++)
	{
		if (vtMeasureCoord[i].dRZ > 180.0)
		{
			vtMeasureCoord[i].dRZ -= 360.0;
		}
		if (vtMeasureCoord[i].dRZ < -180.0)
		{
			vtMeasureCoord[i].dRZ += 360.0;
		}
		vtMeasureCoord[i].dBX = 0.0;
		vtMeasureCoord[i].dBY = vtMeasureCoord[i].dY - 500.0 + dCurExPos;
		vtMeasureCoord[i].dBZ = 0.0;
		vtMeasureCoord[i].dY = 500.0;
	}

	//��ǹ����ת�������
	XiBase::CXiOpenFile file;
	file.XI_fopen(".\\ceshi.txt", "w");
	for (int i = 0; i < vtMeasureCoord.size(); i++)
	{
		file.XI_fprintf("%12.4lf %12.4lf %12.4lf %12.4lf %12.4lf %12.4lf %12.4lf %12.4lf %12.4lf\n",
			vtMeasureCoord[i].dX, vtMeasureCoord[i].dY, vtMeasureCoord[i].dZ, 
			vtMeasureCoord[i].dRX, vtMeasureCoord[i].dRY, vtMeasureCoord[i].dRZ, 
			vtMeasureCoord[i].dBX, vtMeasureCoord[i].dBY, vtMeasureCoord[i].dBZ);
		T_ROBOT_COORS tmpCoor;
		if (!m_pRobotDriver->MoveToolByWeldGun(vtMeasureCoord[i], m_pRobotDriver->m_tTools.tGunTool, vtMeasureCoord[i], m_pRobotDriver->m_tTools.tCameraTool, tmpCoor))
		{
			return false;
		}
		vtMeasureCoord[i] = tmpCoor;
	}
	file.XI_fclose();

	//תΪ��������
	if (!CalcContinuePulseForWeld(vtMeasureCoord, vtMeasurePulse, false))
	{
		return false;
	}
	return true;
}


bool WAM::WeldAfterMeasure::CalcLineMoveTrackBySeamData(int nRobotNo, LineOrCircularArcWeldingLine SeamData, std::vector<T_ROBOT_COORS>& vtMeasureCoord, vector<T_ANGLE_PULSE>& vtMeasurePulse, double dStep)
{
	double dCurCarPos = m_ptUnit->GetExPositionDis(m_ptUnit->m_nLinedScanAxisNo);
	double dCurExPos = m_ptUnit->GetExPositionDis(m_ptUnit->m_nMeasureAxisNo);
	SeamData.StartPoint.x += dCurCarPos;
	SeamData.EndPoint.x += dCurCarPos;
	//SeamData.CenterPoint.x += dCurCarPos;
	SeamData.StartPoint.y -= dCurExPos;
	SeamData.EndPoint.y -= dCurExPos;
	//SeamData.CenterPoint.y -= dCurExPos;
	auto StartPoint = /*m_ptUnit->TransCoor_Gantry2RobotNew*/(SeamData.StartPoint);
	auto EndPoint = /*m_ptUnit->TransCoor_Gantry2RobotNew*/(SeamData.EndPoint);
	//auto CenterPoint = /*m_ptUnit->TransCoor_Gantry2RobotNew*/(SeamData.CenterPoint);
	T_ROBOT_COORS tStart;
	tStart.dX = StartPoint.x;
	tStart.dY = StartPoint.y;
	tStart.dZ = StartPoint.z;
	T_ROBOT_COORS tEnd;
	tEnd.dX = EndPoint.x;
	tEnd.dY = EndPoint.y;
	tEnd.dZ = EndPoint.z;
	//T_ROBOT_COORS tCenter;
	//tCenter.dX = CenterPoint.x;
	//tCenter.dY = CenterPoint.y;
	//tCenter.dZ = CenterPoint.z;

	double dStartSectionLength = 60.0;
	double dRotateAngle = 60.0;
	vtMeasurePulse.clear();

	//Բ�ĵ���������
	//T_ROBOT_COORS dCenterToStart = tStart - tCenter;

	//Բ�ĵ��յ������
	//T_ROBOT_COORS dCenterToEnd = tEnd - tCenter;

	//�뾶
	//double dRadius = sqrt(dCenterToStart.dX * dCenterToStart.dX + dCenterToStart.dY * dCenterToStart.dY);

	//ֱ�߷���
	T_ROBOT_COORS tDir = tEnd - tStart;

	//���ɵȼ���켣
	std::vector<T_ROBOT_COORS> vtPoint = CalLineDividePoints(tStart, tEnd, dStep);

	//������ʼ��
	T_ROBOT_COORS tLineStartPoint = dCenterToStart * (dStartSectionLength / dRadius) + tStart;
	std::vector<T_ROBOT_COORS> vtStartLinePoint = CalLineDividePoints(tLineStartPoint, tStart, dStep);

	//������ֹ��
	T_ROBOT_COORS tLineEndPoint = dCenterToEnd * (dStartSectionLength / dRadius) + tEnd;
	std::vector<T_ROBOT_COORS> vtEndPoint = CalLineDividePoints(tEnd, tLineEndPoint, dStep);

	//RZ�仯����
	int nStepCount = dStartSectionLength / dStep + 0.5;

	//RZ�仯������ֱ��Բ�����Գе�20��
	double dRZStep = dRotateAngle / (nStepCount * 1.0) / 2;

	//�Ƕȸ�ֵ
	double dStartGunAngle = atan2(dCenterToStart.dY, dCenterToStart.dX) * 180 / PI - dRotateAngle;
	double dEndGunAngle = atan2(dCenterToEnd.dY, dCenterToEnd.dX) * 180 / PI + dRotateAngle;
	for (size_t i = 0; i < vtStartLinePoint.size(); i++)
	{
		vtStartLinePoint[i].dRX = 0;
		vtStartLinePoint[i].dRY = -45;
		vtStartLinePoint[i].dRZ = DirAngleToRz(dStartGunAngle) + i * dRZStep;
	}
	for (size_t i = 0; i < nStepCount; i++)
	{
		double dGunAngle = atan2(vtPoint[i].dY - tCenter.dY, vtPoint[i].dX - tCenter.dX) * 180 / PI;
		vtPoint[i].dRX = 0;
		vtPoint[i].dRY = -45;
		vtPoint[i].dRZ = DirAngleToRz(dGunAngle) - (nStepCount - i) * dRZStep;
	}
	for (size_t i = nStepCount; i < vtPoint.size() - nStepCount; i++)
	{
		double dGunAngle = atan2(vtPoint[i].dY - tCenter.dY, vtPoint[i].dX - tCenter.dX) * 180 / PI;
		vtPoint[i].dRX = 0;
		vtPoint[i].dRY = -45;
		vtPoint[i].dRZ = DirAngleToRz(dGunAngle);
	}
	for (size_t i = vtPoint.size() - 1 - nStepCount; i < vtPoint.size(); i++)
	{
		double dGunAngle = atan2(vtPoint[i].dY - tCenter.dY, vtPoint[i].dX - tCenter.dX) * 180 / PI;
		vtPoint[i].dRX = 0;
		vtPoint[i].dRY = -45;
		vtPoint[i].dRZ = DirAngleToRz(dGunAngle) + (i + 1 + nStepCount - vtPoint.size()) * dRZStep;
	}
	for (size_t i = 0; i < vtEndPoint.size(); i++)
	{
		vtEndPoint[i].dRX = 0;
		vtEndPoint[i].dRY = -45;
		vtEndPoint[i].dRZ = DirAngleToRz(dEndGunAngle) - (vtEndPoint.size() - i) * dRZStep;
	}

	//�ϲ�����
	vtMeasureCoord = vtStartLinePoint;
	vtMeasureCoord.insert(vtMeasureCoord.end(), vtPoint.begin(), vtPoint.end());
	vtMeasureCoord.insert(vtMeasureCoord.end(), vtEndPoint.begin(), vtEndPoint.end());

	//ȷ���ⲿ��
	//double dMaxBY = -99999;
	//double dMinBY = 99999;
	//for (size_t i = 0; i < vtMeasureCoord.size(); i++)
	//{
	//	if (vtMeasureCoord[i].dY > dMaxBY)
	//	{
	//		dMaxBY = vtMeasureCoord[i].dY;
	//	}
	//	if (vtMeasureCoord[i].dY < dMinBY)
	//	{
	//		dMinBY = vtMeasureCoord[i].dY;
	//	}
	//}
	//double dBY = (dMaxBY + dMinBY) / 2.0;
	for (size_t i = 0; i < vtMeasureCoord.size(); i++)
	{
		if (vtMeasureCoord[i].dRZ > 180.0)
		{
			vtMeasureCoord[i].dRZ -= 360.0;
		}
		if (vtMeasureCoord[i].dRZ < -180.0)
		{
			vtMeasureCoord[i].dRZ += 360.0;
		}
		vtMeasureCoord[i].dBX = 0.0;
		vtMeasureCoord[i].dBY = vtMeasureCoord[i].dY - 500.0 + dCurExPos;
		vtMeasureCoord[i].dBZ = 0.0;
		vtMeasureCoord[i].dY = 500.0;
	}

	//��ǹ����ת�������
	XiBase::CXiOpenFile file;
	file.XI_fopen(".\\ceshi.txt", "w");
	for (int i = 0; i < vtMeasureCoord.size(); i++)
	{
		file.XI_fprintf("%12.4lf %12.4lf %12.4lf %12.4lf %12.4lf %12.4lf %12.4lf %12.4lf %12.4lf\n",
			vtMeasureCoord[i].dX, vtMeasureCoord[i].dY, vtMeasureCoord[i].dZ,
			vtMeasureCoord[i].dRX, vtMeasureCoord[i].dRY, vtMeasureCoord[i].dRZ,
			vtMeasureCoord[i].dBX, vtMeasureCoord[i].dBY, vtMeasureCoord[i].dBZ);
		T_ROBOT_COORS tmpCoor;
		if (!m_pRobotDriver->MoveToolByWeldGun(vtMeasureCoord[i], m_pRobotDriver->m_tTools.tGunTool, vtMeasureCoord[i], m_pRobotDriver->m_tTools.tCameraTool, tmpCoor))
		{
			return false;
		}
		vtMeasureCoord[i] = tmpCoor;
	}
	file.XI_fclose();

	//תΪ��������
	if (!CalcContinuePulseForWeld(vtMeasureCoord, vtMeasurePulse, false))
	{
		return false;
	}
	return true;
}

#endif

bool WeldAfterMeasure::CalcRealWeldTrack(WeldLineInfo tWeldSeam, vector<T_ROBOT_COORS>& vtWeldTrackRobot)
{
	double dWeldHoleDisS = tWeldSeam.tAtrribute.dStartHoleSize; // �����׳ߴ�
	double dWeldHoleDisE = tWeldSeam.tAtrribute.dEndHoleSize; // dWeldHoleSize; // �����׳ߴ�
	double dChangeDisS = 45.0;	// ����̬����
	double dChangeDisE = 45.0;	// ����̬����
	double dChangeAngle = 30.0; // ����̬�Ƕ�
	double dPtnInterval = 3.0;  // �켣����

	// ��ȡ�������� ���ɺ��ӹ켣
	//bool bIsOutsideWeld = JudgeOutsideWeld(vtWeldSeam); // �Ƿ�Ϊ��ຸ��
	//double dRzChangeDir = bIsOutsideWeld ? -1.0 : 1.0; // ��ຸ��Rz�仯���� ���ڲ��෴
	E_WELD_SEAM_TYPE eWeldSeamType = GetWeldSeamType(tWeldSeam.tWeldLine);
	double dPostureRx = E_FLAT_SEAM == eWeldSeamType ? m_dPlatWeldRx : m_dStandWeldRx;
	double dPostureRy = E_FLAT_SEAM == eWeldSeamType ? m_dPlatWeldRy : m_dStandWeldRy;


	int nChangePtnNum = (int)(dChangeDisS / dPtnInterval); // ����̬����
	int nDelPtnNum = (int)(dWeldHoleDisS / dPtnInterval); // ������ɾ������
	double dStepChangeAngle = (double)m_nRobotInstallDir * dChangeAngle / (double)nChangePtnNum; // ���ڵ���̬�仯�Ƕ�

	int nChangePtnNumS = (int)(dChangeDisS / dPtnInterval); // ����̬����
	int nChangePtnNumE = (int)(dChangeDisE / dPtnInterval); // ����̬����
	int nDelPtnNumS = (int)(dWeldHoleDisS / dPtnInterval); // ������ɾ������
	int nDelPtnNumE = (int)(dWeldHoleDisE / dPtnInterval); // ������ɾ������
	double dStepChangeAngleS = (double)m_nRobotInstallDir * dChangeAngle / (double)nChangePtnNumS; // ���ڵ���̬�仯�Ƕ�
	double dStepChangeAngleE = (double)m_nRobotInstallDir * dChangeAngle / (double)nChangePtnNumE; // ���ڵ���̬�仯�Ƕ�


	int nWeldTrackPtnNum = vtWeldTrackRobot.size();
	if (nWeldTrackPtnNum < 10 ||
		nWeldTrackPtnNum < nChangePtnNumS ||
		nWeldTrackPtnNum < nChangePtnNumE)
	{
		//���޸�
		XUI::MesBox::PopInfo("����켣����{0}���� �޷����ӣ�", nWeldTrackPtnNum);
		//XiMessageBox("����켣����%d���� �޷����ӣ�", nWeldTrackPtnNum);
		return false;
	}


	// ����˵����̬  ɾ�������׹켣
	if (E_FLAT_SEAM == eWeldSeamType && 0 < tWeldSeam.tWeldLine.EndPointType) // ƽ�������յ� 
	{
		int nBaseIndex = nWeldTrackPtnNum - 1 - nChangePtnNumE;
		double dSrcRz = vtWeldTrackRobot[nBaseIndex].dRZ;
		for (int n = nWeldTrackPtnNum - nChangePtnNumE; n < nWeldTrackPtnNum; n++)
		{
			vtWeldTrackRobot[n].dRZ = dSrcRz - ((n - nBaseIndex) * dStepChangeAngleE /** dRzChangeDir*/); // ��̬��С(���������Ұ�װ��ʽ���)(֧������ຸ��)
		}
		vtWeldTrackRobot.erase(vtWeldTrackRobot.end() - nDelPtnNumE, vtWeldTrackRobot.end()); // ɾ������
	}
	if (E_FLAT_SEAM == eWeldSeamType && 0 < tWeldSeam.tWeldLine.StartPointType) // ƽ���������
	{
		double dSrcRz = vtWeldTrackRobot[nChangePtnNumS].dRZ;
		for (int n = nChangePtnNumS - 1; n >= 0; n--)
		{
			vtWeldTrackRobot[n].dRZ = dSrcRz + ((nChangePtnNumS - n) * dStepChangeAngleS /** dRzChangeDir*/); // ��̬����(���������Ұ�װ��ʽ���)(֧������ຸ��)
		}
		vtWeldTrackRobot.erase(vtWeldTrackRobot.begin(), vtWeldTrackRobot.begin() + nDelPtnNumS); // ɾ������
	}
	return true;
}

bool WeldAfterMeasure::SplineFiltering(double dExAxisPos, vector<XI_POINT> vtWeldTrack, vector<T_ROBOT_COORS>& vtWeldTrackRobot)
{
	double dStepDis = 3.0;
	TrackFilter_Init(m_ptUnit->m_nRobotSmooth, dStepDis);
	std::vector<TrackFilter_Node> CoutTrackPoint;
	vtWeldTrackRobot.clear();
	// �������ҷ��ط�����һ��
	bool bNormalDir = false;
	if (-1 == m_pRobotDriver->m_nRobotInstallDir)
	{
		bNormalDir = TRUE;
	}
	// �����˲�
	for (int index = 0; index < vtWeldTrack.size(); index++)
	{
		CoutTrackPoint.clear();
		CoutTrackPoint = TrackFilter_FilterCurvePointInPointOut(m_ptUnit->m_nRobotSmooth, vtWeldTrack[index].x,
			vtWeldTrack[index].y, vtWeldTrack[index].z, bNormalDir);

		m_pRobotDriver->m_cLog->Write("��ʼ���˲�:%d %d", index, CoutTrackPoint.size());
		for (int nSize = 0; nSize < CoutTrackPoint.size(); nSize++)
		{
			// ���㷨��
			double dTrackVerDegree = atan2(CoutTrackPoint[nSize].normalDir_.y_, CoutTrackPoint[nSize].normalDir_.x_) * 180.0 / PI;

			T_ROBOT_COORS tRobotPoint(
				CoutTrackPoint[nSize].pnt_.x_, CoutTrackPoint[nSize].pnt_.y_, CoutTrackPoint[nSize].pnt_.z_,
				m_dPlatWeldRx, m_dPlatWeldRy, m_pRobotDriver->DirAngleToRz(dTrackVerDegree), 0.0, 0.0, 0.0);
			vtWeldTrackRobot.push_back(tRobotPoint);
		}
	}
	// ��ȡ��β
	TrackSmooth_PntArray tTrackPtnArray;
	TrackSmooth_Init(m_ptUnit->m_nRobotSmooth, dStepDis, vtWeldTrackRobot.back().dX,
		vtWeldTrackRobot.back().dY, vtWeldTrackRobot.back().dZ);
	// ��ȡ��β������
	TrackSmooth_PushNextPoint(m_ptUnit->m_nRobotSmooth, vtWeldTrack.back().x,
		vtWeldTrack.back().y, vtWeldTrack.back().z, &tTrackPtnArray);
	for (int n = 0; n < tTrackPtnArray.arraySize_; n++)
	{
		T_ROBOT_COORS tRobotPoint(
			tTrackPtnArray.pnts_[n].x_, tTrackPtnArray.pnts_[n].y_, tTrackPtnArray.pnts_[n].z_,
			m_dPlatWeldRx, m_dPlatWeldRy, vtWeldTrackRobot.back().dRZ, 0.0, 0.0, 0.0);
		vtWeldTrackRobot.push_back(tRobotPoint);
	}
	TrackSmooth_FreePntArray(&tTrackPtnArray);

	//��ȡ��β�㣬����βʱ���ã�����һ��ľ���С�� moveMinDis 
	TrackSmooth_GetEndPoint(m_ptUnit->m_nRobotSmooth, &tTrackPtnArray);
	for (int n = 0; n < tTrackPtnArray.arraySize_; n++)
	{
		if (m_pTraceModel->m_bIfOpenExternal)
		{
			T_ROBOT_COORS tRobotPoint(
				tTrackPtnArray.pnts_[n].x_, tTrackPtnArray.pnts_[n].y_, tTrackPtnArray.pnts_[n].z_,
				m_dPlatWeldRx, m_dPlatWeldRy, vtWeldTrackRobot.back().dRZ, 0.0, 0.0, 0.0);
			vtWeldTrackRobot.push_back(tRobotPoint);
		}
	}
	TrackSmooth_FreePntArray(&tTrackPtnArray);

	//jwq ����Ƕ�
	for (size_t i = 1; i < vtWeldTrackRobot.size() - 1; i++)
	{
		double dTrackVerDegree = atan2(vtWeldTrackRobot[i + 1].dY - vtWeldTrackRobot[i - 1].dY, vtWeldTrackRobot[i + 1].dX - vtWeldTrackRobot[i - 1].dX) * 180.0 / PI + 90;
		vtWeldTrackRobot[i].dRZ = m_pRobotDriver->DirAngleToRz(dTrackVerDegree);
	}
	vtWeldTrackRobot[0].dRZ = vtWeldTrackRobot[1].dRZ;
	vtWeldTrackRobot.back().dRZ = vtWeldTrackRobot[vtWeldTrackRobot.size() - 2].dRZ;

	// ժ���ⲿ������
	for (size_t i = 0; i < vtWeldTrackRobot.size(); i++)
	{
		T_ROBOT_COORS tRobotPoint = vtWeldTrackRobot[i];

		tRobotPoint.dX = 1 == labs(m_ptUnit->m_nTrackAxisNo) ? vtWeldTrackRobot[i].dX + vtWeldTrackRobot[i].dBX - dExAxisPos : vtWeldTrackRobot[i].dX;
		tRobotPoint.dY = 2 == labs(m_ptUnit->m_nTrackAxisNo) ? vtWeldTrackRobot[i].dY + vtWeldTrackRobot[i].dBY - dExAxisPos : vtWeldTrackRobot[i].dY;
		tRobotPoint.dZ = 3 == labs(m_ptUnit->m_nTrackAxisNo) ? vtWeldTrackRobot[i].dZ + vtWeldTrackRobot[i].dBZ - dExAxisPos : vtWeldTrackRobot[i].dZ;
		tRobotPoint.dBX = 1 == labs(m_ptUnit->m_nTrackAxisNo) ? dExAxisPos : vtWeldTrackRobot[i].dBX;
		tRobotPoint.dBY = 2 == labs(m_ptUnit->m_nTrackAxisNo) ? dExAxisPos : vtWeldTrackRobot[i].dBY;
		tRobotPoint.dBZ = 3 == labs(m_ptUnit->m_nTrackAxisNo) ? dExAxisPos : vtWeldTrackRobot[i].dBZ;

		vtWeldTrackRobot[i] = tRobotPoint;
	}
	return true;
}

int WeldAfterMeasure::DetermineWarpMode(int nGroupNo)
{
	int nWarpNum = 0;
	for (int nWeldNo = 0; nWeldNo < m_vvtWeldLineInfoGroup[nGroupNo].size(); nWeldNo++)
	{
		if (nWeldNo > 0)
		{
			break;
		}
		if (m_vvtWeldLineInfoGroup[nGroupNo][nWeldNo].tAtrribute.nStartWrapType > 0 && 
			m_vvtWeldLineInfoGroup[nGroupNo][nWeldNo].tAtrribute.nStartWrapType < 99)
		{
			nWarpNum++;
		}
		if (m_vvtWeldLineInfoGroup[nGroupNo][nWeldNo].tAtrribute.nEndWrapType > 0 &&
			m_vvtWeldLineInfoGroup[nGroupNo][nWeldNo].tAtrribute.nEndWrapType < 99)
		{
			nWarpNum++;
		}
	}
	return nWarpNum;
}

bool WeldAfterMeasure::CalcOnceWarpTrack(int nWarpNum, double dBoardChick, double dGunToEyeCompen, double dFlatHorComp, std::vector<T_ROBOT_COORS> vtWeldLineTrack, double dWorkpieceHight,
	std::vector<T_ROBOT_COORS>& vtWarpCoors, std::vector<int>& vtWarpPtType, E_WRAPANGLE_PARAM tWarpParam)
{
	m_pRobotDriver->m_cLog->Write("һ�ΰ��ǲ���:%.3lf %.3lf %.3lf", tWarpParam.dEndpointOffsetDis, tWarpParam.dStartVerOffset, tWarpParam.dStartBackVerOffset);
	// !!!!!!!!���ɰ��ǹ켣------------------------------
	// ���һ���������� p1 
	// ���һ���켣������ӳ�30mm p2 
	// �����෴����ƫ�ư�� p3 
	// p2 �� p3 ������ǹ�켣
	// p1��Բ�ƫ��p4 
	// ������100mm�ĵ���Բ�ƫ����Ϊ�����ʼ�ι켣 
	// �켣��m_pTraceModel->m_vvtTrackWarpCoors[0]
	// ���ͣ�m_pTraceModel->m_vvnTrackWarpPtType[0]
	/* dBoardChick += fabs(m_pScanInit->m_pTraceModel->m_dGunToEyeCompenX)*/; // ���
	//int nWrapPtnNum = 10;
	T_ROBOT_COORS First = vtWeldLineTrack[vtWeldLineTrack.size() - 1/* - nWrapPtnNum*/];
	T_ROBOT_COORS P1 = vtWeldLineTrack[vtWeldLineTrack.size() - 1/* - nWrapPtnNum*/];
	T_ROBOT_COORS tDirEnd = P1; // ���� ����ǰ���ӵķ�����
	T_ROBOT_COORS tDirStart;
	T_ROBOT_COORS P2, P3, P4;
	vector<XI_POINT> vtDirPoints(0);// �������һ�ι켣 ���ڼ��㷽��
	int nIdx = 0;

	tDirEnd.dX += tDirEnd.dBX;
	tDirEnd.dY += tDirEnd.dBY;
	vtWarpCoors.clear();
	vtWarpPtType.clear();
	T_ROBOT_COORS P;
	for (nIdx = vtWeldLineTrack.size() - 1/* - nWrapPtnNum*/; nIdx >= 0; nIdx--)
	{
		P = vtWeldLineTrack[nIdx];
		double dDis = TwoPointDis(P.dX + P.dBX, P.dY + P.dBY, P1.dX + P1.dBX, P1.dY + P1.dBY);
		if (dDis > m_dHandEyeDis)
		{
			tDirStart = P;
			tDirStart.dX += tDirStart.dBX;
			tDirStart.dY += tDirStart.dBY;
			break;
		}
		XI_POINT tPt = { P.dX + P.dBX ,P.dY + P.dBY ,P.dZ + P.dBZ };
		vtDirPoints.push_back(tPt);
	}
	// �����β�η���
	T_LINE_PARA tBestLineParamRever; // ������
	if (!CalcLineParamRansac(tBestLineParamRever, vtDirPoints, 0.7))
	{
		return false;
	}
	reverse(vtDirPoints.begin(), vtDirPoints.end());
	T_LINE_PARA tBestLineParam; // ����
	if (!CalcLineParamRansac(tBestLineParam, vtDirPoints, 0.7))
	{
		return false;
	}
	double dWorkDir = atan2(tBestLineParam.dDirY, tBestLineParam.dDirX) * 180.0 / 3.1415926;
	//dWorkDir -= 90 * m_nRobotInstallDir;
	//jwq���ҳ��ο��˵�
	XI_POINT tDealEnd = { 0.0 };// { 60.327, 3106.450, 1989.990 };
	/*if (m_vtTrackTheoryTrack.size() < 2)
	{
		return false;
	}*/
	/*double dPStartDis = TwoPointDis(P1.dX + P1.dBX, P1.dY + P1.dBY, m_vtTrackTheoryTrack[0].x, m_vtTrackTheoryTrack[0].y);
	double dPEndDis = TwoPointDis(P1.dX + P1.dBX, P1.dY + P1.dBY, m_vtTrackTheoryTrack.back().x, m_vtTrackTheoryTrack.back().y);*/
	/*double dPStartDis = TwoPointDis(P1.dX + P1.dBX, P1.dY + P1.dBY, m_vtTrackTheoryTrack[0].x, m_vtTrackTheoryTrack[0].y);
	double dPEndDis = TwoPointDis(P1.dX + P1.dBX, P1.dY + P1.dBY, m_vtTrackTheoryTrack.back().x, m_vtTrackTheoryTrack.back().y);*/
	switch (nWarpNum)
	{
	case 0:
		break;
	case 1:
		//////tDealEnd = dPStartDis > dPEndDis ? m_vtTrackTheoryTrack.back() : m_vtTrackTheoryTrack[0];
		//tDealEnd.x = vtWeldLineTrack[vtWeldLineTrack.size() - 1].dX+ vtWeldLineTrack[vtWeldLineTrack.size() - 1].dBX;
		//tDealEnd.y = vtWeldLineTrack[vtWeldLineTrack.size() - 1].dY;
		//tDealEnd.z = vtWeldLineTrack[vtWeldLineTrack.size() - 1].dZ;
		tDealEnd.x = m_pTraceModel->m_tRealEndpointCoor.dX + m_pTraceModel->m_tRealEndpointCoor.dBX;
		tDealEnd.y = m_pTraceModel->m_tRealEndpointCoor.dY + m_pTraceModel->m_tRealEndpointCoor.dBY;
		tDealEnd.z = m_pTraceModel->m_tRealEndpointCoor.dZ + m_pTraceModel->m_tRealEndpointCoor.dBZ;
		break;
	default:
		return true;//�ر�أ���Ŀ����1��Ϊ����Ҫ����
	}

	//jwq���켣Ԥ��
	if (!TrajectoryLineTrack(vtWarpCoors, tBestLineParam, P1, tDealEnd, 16, 10.0))
	{
		return false;
	}
	for (size_t i = 0; i < vtWarpCoors.size(); i++)
	{
		RobotCoordPosOffset(vtWarpCoors[i], dWorkDir - 90.0 * m_nRobotInstallDir, tWarpParam.dStartVerOffset);
		vtWarpPtType.push_back(E_WELD_WRAP);
	}

	//��ʵ���ǹ켣�����ܳ�
	double dRealDis = TwoPointDis(P1.dX + P1.dBX, P1.dY + P1.dBY, P1.dZ + P1.dBZ,
		tDealEnd.x, tDealEnd.y, tDealEnd.z);

	int nFirstNum = vtWarpCoors.size();
	P2 = vtWarpCoors.back();
	P2 = RobotCoordPosOffset(P2, tBestLineParam, tWarpParam.dEndpointOffsetDis);//jwq��ʱ�����Ǵ�����ƫ����
	P3 = P2;
	//jwq��ʱ�ĽǶȣ���֤�Գ�
	P3.dRZ = DirAngleToRz(dWorkDir + 90.0 * m_nRobotInstallDir);

	// ��ӷ��油��
	double dNorCompe = tWarpParam.dStartBackVerOffset + tWarpParam.dStartVerOffset;


	RobotCoordPosOffset(P3, RzToDirAngle(P3.dRZ), dNorCompe + dBoardChick + (dGunToEyeCompen - m_pTraceModel->m_dFlatHorComp) * 2.0);
	//RobotCoordPosOffset(P3, RzToDirAngle(P3.dRZ), dBoardChick + dGunToEyeCompen * 2.0);

	// һ�ΰ��ǹ켣
	vector<T_ROBOT_COORS> vOnceWarpTrack;
	vector<int> vOnceWarpType(0);
	T_ROBOT_COORS tWrapStart = P2;
	T_ROBOT_COORS tWrapEnd = P3;
	T_ROBOT_COORS tInsertCoord;
	XiAlgorithm tAlg;
	double dDirWrap, dBoardThick;
	int nStepNum = 0;// ����
	double dStepDis = 2.0; // ����
	double dStepAngle = 0.0; // �����仯�Ƕ�
	double dChangeAngle = 180.0; // �仯�ܵĽǶ�
	dBoardThick = TwoPointDis(tWrapStart.dX + tWrapStart.dBX, tWrapStart.dY + tWrapStart.dBY, tWrapStart.dZ,
		tWrapEnd.dX + tWrapEnd.dBX, tWrapEnd.dY + tWrapEnd.dBY, tWrapEnd.dZ);
	dDirWrap = tAlg.CalcArcAngle((tWrapEnd.dX + tWrapEnd.dBX) - (tWrapStart.dX + tWrapStart.dBX), (tWrapEnd.dY + tWrapEnd.dBY) - (tWrapStart.dY + tWrapStart.dBY));

	nStepNum = (int)(dBoardThick / dStepDis + 0.5);
	dStepAngle = dChangeAngle / (double)nStepNum;

	// ��Ӱ��ǹ켣��
	// ��ʼ�켣
	tWrapStart.dRZ += 45 * m_pRobotDriver->m_nRobotInstallDir;

	vOnceWarpTrack.push_back(tWrapStart);
	vOnceWarpType.push_back(E_WELD_WRAP);
	// �м�켣
	tWrapStart.dRZ += 45 * m_pRobotDriver->m_nRobotInstallDir;
	for (int i = 1; i < nStepNum; i++)
	{
		RobotCoordPosOffset(tWrapStart, dDirWrap, dStepDis);
		vOnceWarpTrack.push_back(tWrapStart);
		vOnceWarpType.push_back(E_WELD_WRAP);
	}
	P3.dRZ = tWrapStart.dRZ + 45 * m_pRobotDriver->m_nRobotInstallDir;
	vOnceWarpTrack.push_back(P3);
	vOnceWarpType.push_back(E_WELD_WRAP);
	//��β�켣
	P3.dRZ = tWrapStart.dRZ + 90.0 * m_pRobotDriver->m_nRobotInstallDir;

	// �ڶ��ΰ��ǹ켣����
	double dSecondRealOffsetDis = dRealDis / 15.0;//2.0
	for (int i = 1; i < 15; i++) // �ڶ��ΰ���
	{
		P3 = RobotCoordPosOffset(P3, tBestLineParamRever, dSecondRealOffsetDis);
		vtWarpCoors.push_back(P3);
		vtWarpPtType.push_back(E_WELD_WRAP);
	}
	for (int i = vtWeldLineTrack.size() - 1/* - nWrapPtnNum*/; i >= nIdx; i--) // �����ʼ�ι켣 
	{
		P4 = vtWeldLineTrack[i];
		P4.dRZ = P3.dRZ;
		RobotCoordPosOffset(P4, dWorkDir + 90.0 * m_nRobotInstallDir, tWarpParam.dStartBackVerOffset + dBoardChick + (dGunToEyeCompen - m_pTraceModel->m_dFlatHorComp) * 2.0 - m_pTraceModel->m_dGunToEyeCompenX, -1 * m_pTraceModel->m_dGunToEyeCompenZ);
		//RobotCoordPosOffset(P4, dWorkDir + 90.0 * m_nRobotInstallDir/*RzToDirAngle(P4.dRZ)*/, dBoardChick + dGunToEyeCompen+ tWarpParam.dStartBackVerOffset, -1 * m_pTraceModel->m_dGunToEyeCompenZ);
		vtWarpCoors.push_back(P4);
		if (i == vtWeldLineTrack.size() - 1/* - nWrapPtnNum*/)
		{
			vtWarpPtType.push_back(E_WELD_OPEN_TRACKING);
		}
		else
		{
			vtWarpPtType.push_back(E_WELD_TRACK);
		}
	}
	// ������ǹ����
	vtWarpCoors.insert(vtWarpCoors.begin() + nFirstNum, vOnceWarpTrack.begin(), vOnceWarpTrack.end());
	vtWarpPtType.insert(vtWarpPtType.begin() + nFirstNum, vOnceWarpType.begin(), vOnceWarpType.end());

	// ���ƻ����˺��ⲿ��ƽ���ᱣ�ֲ���
	for (size_t i = 0; i < vtWarpCoors.size(); i++)
	{
		if (1 == m_ptUnit->m_nTrackAxisNo)
		{
			vtWarpCoors[i].dBX = vtWarpCoors[i].dX + vtWarpCoors[i].dBX - First.dX;
			vtWarpCoors[i].dX = First.dX;
		}
		else if (2 == m_ptUnit->m_nTrackAxisNo)
		{
			vtWarpCoors[i].dBY = vtWarpCoors[i].dY + vtWarpCoors[i].dBY - First.dY;
			vtWarpCoors[i].dY = First.dY;
		}

	}
	// ����Rz�Ƕ�
	for (int i = 0; i < vtWarpCoors.size(); i++)
	{
		if (vtWarpCoors[i].dRZ > 180.0)
		{
			vtWarpCoors[i].dRZ -= 360.0;
		}
		if (vtWarpCoors[i].dRZ < -180.0)
		{
			vtWarpCoors[i].dRZ += 360.0;
		}
	}
	vector<T_ANGLE_PULSE> vtRealWeldPulse;
	if (!CalcContinuePulseForWeld(vtWarpCoors, vtRealWeldPulse, TRUE))
	{
		XiMessageBox("һ�ΰ��Ǽ����������������У�������ײ��ͣ����");
		return false;
	}
	if (vtWarpCoors.size() != vtWarpPtType.size() || vtWarpCoors.size() < 1 || vtWarpPtType.size() < 1)
	{
		return false;
	}
	return true;
}

E_WRAPANGLE_PARAM WeldAfterMeasure::LoadWarpParam(CString strRobotName, E_WRAPANGLE_TYPE eWarpType, int nWarpNo)
{
	bool bRst = true;
	E_WRAPANGLE_PARAM tWarpParam;
	CString strPath = DATA_PATH + strRobotName + "\\WarpAngleParam.ini";
	COPini opini;
	opini.SetFileName(strPath);
	switch (eWarpType)
	{
	case E_WRAPANGLE_JUMP:
		bRst = bRst && opini.SetSectionName("JumpParam");
		if (0 == nWarpNo)
		{
			bRst = bRst && opini.ReadString("StartOffsetDis", &tWarpParam.dEndpointOffsetDis);
			bRst = bRst && opini.ReadString("StartVerOffset", &tWarpParam.dStartVerOffset);
			bRst = bRst && opini.ReadString("StartBackVerOffset", &tWarpParam.dStartBackVerOffset);
			bRst = bRst && opini.ReadString("WarpSpeed", &tWarpParam.dWarpSpeed);
			bRst = bRst && opini.ReadString("StartWarpLength ", &tWarpParam.dWarpLength);
			bRst = bRst && opini.ReadString("StartRotateSpeed", &tWarpParam.dRotateSpeed);
			bRst = bRst && opini.ReadString("StartRotateAngle", &tWarpParam.dRotateAngle);
			bRst = bRst && opini.ReadString("StartRotateStpTime", &tWarpParam.dRotateStpTime);
			bRst = bRst && opini.ReadString("StartInputSpeed ", &tWarpParam.dInputSpeed);
			bRst = bRst && opini.ReadString("StartInputRatoi ", &tWarpParam.dInputRatoi);
			bRst = bRst && opini.ReadString("StartInputMaxDis", &tWarpParam.dInputMaxDis);
			bRst = bRst && opini.ReadString("StartBackRotateSpeed ", &tWarpParam.dBackRotateSpeed);
			bRst = bRst && opini.ReadString("StartBackRotateAngle ", &tWarpParam.dBackRotateAngle);
			bRst = bRst && opini.ReadString("StartBackRotateStpTime", &tWarpParam.dBackRotateStpTime);
			bRst = bRst && opini.ReadString("StartBackInputSpeed ", &tWarpParam.dBackInputSpeed);
			bRst = bRst && opini.ReadString("StartBackInputRatoi ", &tWarpParam.dBackInputRatoi);
			bRst = bRst && opini.ReadString("StartBackInputMaxDis", &tWarpParam.dBackInputMaxDis);
			bRst = bRst && opini.ReadString("JumpSafeHight", &tWarpParam.dJumpSafeHight);
		}
		else
		{
			bRst = bRst && opini.ReadString("EndOffsetDis", &tWarpParam.dEndpointOffsetDis);
			bRst = bRst && opini.ReadString("EndVerOffset", &tWarpParam.dStartVerOffset);
			bRst = bRst && opini.ReadString("EndBackVerOffset", &tWarpParam.dStartBackVerOffset);
			bRst = bRst && opini.ReadString("WarpSpeed", &tWarpParam.dWarpSpeed);
			bRst = bRst && opini.ReadString("EndWarpLength ", &tWarpParam.dWarpLength);
			bRst = bRst && opini.ReadString("EndRotateSpeed", &tWarpParam.dRotateSpeed);
			bRst = bRst && opini.ReadString("EndRotateAngle", &tWarpParam.dRotateAngle);
			bRst = bRst && opini.ReadString("EndRotateStpTime", &tWarpParam.dRotateStpTime);
			bRst = bRst && opini.ReadString("EndInputSpeed ", &tWarpParam.dInputSpeed);
			bRst = bRst && opini.ReadString("EndInputRatoi ", &tWarpParam.dInputRatoi);
			bRst = bRst && opini.ReadString("EndInputMaxDis", &tWarpParam.dInputMaxDis);
			bRst = bRst && opini.ReadString("EndBackRotateSpeed ", &tWarpParam.dBackRotateSpeed);
			bRst = bRst && opini.ReadString("EndBackRotateAngle ", &tWarpParam.dBackRotateAngle);
			bRst = bRst && opini.ReadString("EndBackRotateStpTime", &tWarpParam.dBackRotateStpTime);
			bRst = bRst && opini.ReadString("EndBackInputSpeed ", &tWarpParam.dBackInputSpeed);
			bRst = bRst && opini.ReadString("EndBackInputRatoi ", &tWarpParam.dBackInputRatoi);
			bRst = bRst && opini.ReadString("EndBackInputMaxDis", &tWarpParam.dBackInputMaxDis);
			bRst = bRst && opini.ReadString("JumpSafeHight", &tWarpParam.dJumpSafeHight);
		}

		break;
	case E_WRAPANGLE_ONCE:
		opini.SetSectionName("OnceParam");
		if (0 == nWarpNo)
		{
			bRst = bRst && opini.ReadString("StartOffsetDis", &tWarpParam.dEndpointOffsetDis);
			bRst = bRst && opini.ReadString("StartVerOffset", &tWarpParam.dStartVerOffset);
			bRst = bRst && opini.ReadString("StartBackVerOffset", &tWarpParam.dStartBackVerOffset);
			bRst = bRst && opini.ReadString("WarpSpeed", &tWarpParam.dWarpSpeed);
			bRst = bRst && opini.ReadString("StartWarpLength ", &tWarpParam.dWarpLength);
		}
		else
		{
			bRst = bRst && opini.ReadString("EndOffsetDis", &tWarpParam.dEndpointOffsetDis);
			bRst = bRst && opini.ReadString("EndVerOffset", &tWarpParam.dStartVerOffset);
			bRst = bRst && opini.ReadString("EndBackVerOffset", &tWarpParam.dStartBackVerOffset);
			bRst = bRst && opini.ReadString("WarpSpeed", &tWarpParam.dWarpSpeed);
			bRst = bRst && opini.ReadString("EndWarpLength ", &tWarpParam.dWarpLength);
		}
		break;
	case E_WRAPANGLE_TWO_SINGLE:
		opini.SetSectionName("TwoParam");

		if (0 == nWarpNo)
		{
			bRst = bRst && opini.ReadString("StartOffsetDis", &tWarpParam.dEndpointOffsetDis);
			bRst = bRst && opini.ReadString("StartVerOffset", &tWarpParam.dStartVerOffset);
			bRst = bRst && opini.ReadString("StartBackVerOffset", &tWarpParam.dStartBackVerOffset);
			bRst = bRst && opini.ReadString("WarpSpeed", &tWarpParam.dWarpSpeed);
			bRst = bRst && opini.ReadString("StartRotateSpeed", &tWarpParam.dRotateSpeed);
			bRst = bRst && opini.ReadString("StartRotateAngle", &tWarpParam.dRotateAngle);
			bRst = bRst && opini.ReadString("StartRotateStpTime", &tWarpParam.dRotateStpTime);
			bRst = bRst && opini.ReadString("StartInputSpeed ", &tWarpParam.dInputSpeed);
			bRst = bRst && opini.ReadString("StartInputRatoi ", &tWarpParam.dInputRatoi);
			bRst = bRst && opini.ReadString("StartInputMaxDis", &tWarpParam.dInputMaxDis);
		}
		else
		{
			bRst = bRst && opini.ReadString("EndOffsetDis", &tWarpParam.dEndpointOffsetDis);
			bRst = bRst && opini.ReadString("EndVerOffset", &tWarpParam.dStartVerOffset);
			bRst = bRst && opini.ReadString("EndBackVerOffset", &tWarpParam.dStartBackVerOffset);
			bRst = bRst && opini.ReadString("WarpSpeed", &tWarpParam.dWarpSpeed);
			bRst = bRst && opini.ReadString("EndRotateSpeed", &tWarpParam.dRotateSpeed);
			bRst = bRst && opini.ReadString("EndRotateAngle", &tWarpParam.dRotateAngle);
			bRst = bRst && opini.ReadString("EndRotateStpTime", &tWarpParam.dRotateStpTime);
			bRst = bRst && opini.ReadString("EndInputSpeed ", &tWarpParam.dInputSpeed);
			bRst = bRst && opini.ReadString("EndInputRatoi ", &tWarpParam.dInputRatoi);
			bRst = bRst && opini.ReadString("EndInputMaxDis", &tWarpParam.dInputMaxDis);
		}
		break;
	default:
		break;
	}
	return tWarpParam;
}

bool WeldAfterMeasure::CheckFlangeToolCoord(const T_ROBOT_COORS& tRobotCoord, double dDisThreshold/* = 200.0*/)
{
	bool bRst = true;
	T_ROBOT_COORS tFlangeTool(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	T_ROBOT_COORS tCoord;
	std::vector<T_ANGLE_PULSE> vtPulse;
	bRst &= m_pRobotDriver->RobotInverseKinematics(tRobotCoord, m_pRobotDriver->m_tTools.tGunTool, vtPulse);
	if (false == bRst) return bRst;
	m_pRobotDriver->RobotKinematics(vtPulse[0], tFlangeTool, tCoord);
	double dDis = TwoPointDis(0.0, 0.0, tCoord.dX, tCoord.dY);
	bRst &= (dDis > dDisThreshold);
	return bRst;
}

bool WeldAfterMeasure::CheckFlangeToolCoord(const vector<T_ROBOT_COORS>& vtRobotCoord, double dDisThreshold/* = 200.0*/)
{
	bool bRst = true;
	T_ROBOT_COORS tFlangeTool(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	T_ROBOT_COORS tCoord;
	for (int i = 0; i < vtRobotCoord.size(); i++)
	{
		std::vector<T_ANGLE_PULSE> vtPulse;
		bRst &= m_pRobotDriver->RobotInverseKinematics(vtRobotCoord[i], m_pRobotDriver->m_tTools.tGunTool, vtPulse);
		if (false == bRst) break;
		m_pRobotDriver->RobotKinematics(vtPulse[0], tFlangeTool, tCoord);
		double dDis = TwoPointDis(0.0, 0.0, tCoord.dX, tCoord.dY);
		bRst &= (dDis > dDisThreshold);
		if (false == bRst) break;
	}
	return bRst;
}

bool WeldAfterMeasure::CheckFlangeToolCoord(const T_ANGLE_PULSE& tRobotPulse, double dDisThreshold/* = 200.0*/)
{
	bool bRst = true;
	T_ROBOT_COORS tFlangeTool(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	T_ROBOT_COORS tCoord;
	m_pRobotDriver->RobotKinematics(tRobotPulse, tFlangeTool, tCoord);
	double dDis = TwoPointDis(0.0, 0.0, tCoord.dX, tCoord.dY);
	bRst &= (dDis > dDisThreshold);
	return bRst;
}

bool WeldAfterMeasure::CheckFlangeToolCoord(const vector<T_ANGLE_PULSE> &vtRobotPulse, double dDisThreshold/* = 200.0*/)
{
	bool bRst = true;
	double dDis = 0.0;
	T_ROBOT_COORS tFlangeTool(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	T_ROBOT_COORS tCoord;
	for (int i = 0; i < vtRobotPulse.size(); i++)
	{
		m_pRobotDriver->RobotKinematics(vtRobotPulse[i], tFlangeTool, tCoord);
		dDis = TwoPointDis(0.0, 0.0, tCoord.dX, tCoord.dY);
		bRst &= (dDis > dDisThreshold);
		if (false == bRst) break;
	}
	return bRst;
}

//void WeldAfterMeasure::WeldInfoToJson(Welding_Info tWeldInfo)
//{
//	Welding_Track tracks;			// �켣����
//
//	// ����Json�ĵ�
//	xrc::weld::WeldDataDocJson weldDoc;
//
//	// ��������, Ĭ������, �޸���������
//	xrc::weld::WeldProperty property;
//
//	// ����˵��������
//	property.startWrapCorner = 0.0;
//	property.endWrapCorner = 0.0;
//
//	// �����״�С
//	property.sHoleSize = m_dWeldHoleSize;
//	property.eHoleSize = m_dWeldHoleSize;
//
//	// �Ƿ�˫�溸��
//	property.doubleWelding = 0;
//
//	// ���ӳ��� Ĭ����Ч
//	property.weldingLength = -1;
//
//	std::vector<T_WELD_PARA> vtWeldParaFlat;
//	GetCurWeldParam(E_FLAT_SEAM, vtWeldParaFlat);
//
//	for (size_t i = 0; i < tWeldInfo.tracks_size; i++)
//	{
//		tracks = tWeldInfo.tracks[i];
//
//		// ������, ���һ��Բ��
//		xrc::weld::WeldWire weldWire;
//
//		for (size_t j = 0; j < tracks.points_size - 1; j++)
//		{
//			xrc::weld::GeoXYZ sPoint{ tracks.points[j].x, tracks.points[j].y, tracks.points[j].z };
//			xrc::weld::GeoXYZ ePoint{ tracks.points[j + 1].x, tracks.points[j + 1].y, tracks.points[j + 1].z };
//
//			// ����һ��ֱ��
//			xrc::weld::ThreePointLine line{ sPoint, ePoint };
//
//
//			xrc::weld::GeoXYZ sNormal{ tracks.points_normal[j].x , tracks.points_normal[j].y , tracks.points_normal[j].z };
//			xrc::weld::GeoXYZ eNormal{ tracks.points_normal[j + 1].x , tracks.points_normal[j + 1].y ,tracks.points_normal[j + 1].z };
//
//			weldWire.addBack(line, sNormal, eNormal);
//		}
//
//		// ��������
//		property.index = i;
//
//		// �Ƿ�������������ڰ�
//		property.isleft = tracks.isLeft;
//
//		// ��������Ļ����˱��
//		property.robotSelect = 0.0;
//
//		// ���캸�ųߴ�
//		property.weldAngel = vtWeldParaFlat[0].nWeldAngleSize;
//
//		// 
//		property.zSide = tracks.ZSide;
//
//		// ���յ㷨��
//		property.sNDir._x = tracks.points_normal[0].x;
//		property.sNDir._y = tracks.points_normal[0].y;
//		property.sNDir._z = tracks.points_normal[0].z;
//
//		property.eNDir._x = tracks.points_normal[tracks.points_size - 1].x;
//		property.eNDir._y = tracks.points_normal[tracks.points_size - 1].y;
//		property.eNDir._z = tracks.points_normal[tracks.points_size - 1].z;
//
//		// ����˵�����
//		property.startPointType = tracks.start_point_type;
//		property.endPointType = tracks.end_point_type;
//
//
//		// һ������
//		xrc::weld::WeldSeam ws;
//		ws.setWeldWire(weldWire);     // ���ú�����
//		ws.setWeldProperty(property); // ���ú�������
//
//		weldDoc.weldInfos.push_back(ws);  // ��Ӻ���
//
//
//		// ���浽�ļ�
//
//			CString output_weldsinfo_path = OUTPUT_PATH + m_ptUnit->m_tContralUnit.strUnitName + "\\" + POINT_CLOUD_IDENTIFY_RESULT_JSON;
//			bool bRst = xrc::weld::writeWeldDataDocJson(weldDoc, output_weldsinfo_path);
//	}
//}

void WeldAfterMeasure::WeldInfoToJson(Welding_Info tWeldInfo)
{
	CString output_weldsinfo_path = OUTPUT_PATH + m_ptUnit->m_tContralUnit.strUnitName + "\\" + POINT_CLOUD_IDENTIFY_RESULT_JSON;
	DelFiles(output_weldsinfo_path);

	Welding_Track tracks;			// �켣����

	// ����Json�ĵ�
	xrc::weld::WeldDataDocJson weldDoc;

	// һ������
	xrc::weld::WeldingWorkpiece tWeldWorkpiece;

	// ��������, Ĭ������, �޸���������
	xrc::weld::WeldProperty property;

	// ����˵��������
	property.startWrapCorner = 0.0;
	property.endWrapCorner = 0.0;

	// �����״�С
	property.sHoleSize = m_dWeldHoleSize;
	property.eHoleSize = m_dWeldHoleSize;

	// �Ƿ�˫�溸��
	property.doubleWelding = 0;

	// ���ӳ��� Ĭ����Ч
	property.weldingLength = -1;

	std::vector<T_WELD_PARA> vtWeldParaFlat;
	GetCurWeldParam(E_FLAT_SEAM, vtWeldParaFlat);

	for (size_t i = 0; i < tWeldInfo.tracks_size; i++)
	{
		tracks = tWeldInfo.tracks[i];

		// ������, ���һ��Բ��
		xrc::weld::WeldWire weldWire;

		if(tracks.points_size > 0)
		{
			for (int j = 0; j < (int)tracks.points_size - 1; j++)
			{
				xrc::weld::GeoXYZ sPoint{ tracks.points[j].x, tracks.points[j].y, tracks.points[j].z };
				xrc::weld::GeoXYZ ePoint{ tracks.points[j + 1].x, tracks.points[j + 1].y, tracks.points[j + 1].z };

				// ����һ��ֱ��
				xrc::weld::ThreePointLine line{ sPoint, ePoint };


				xrc::weld::GeoXYZ sNormal{ tracks.points_normal[j].x , tracks.points_normal[j].y , tracks.points_normal[j].z };
				xrc::weld::GeoXYZ eNormal{ tracks.points_normal[j + 1].x , tracks.points_normal[j + 1].y ,tracks.points_normal[j + 1].z };

				weldWire.addBack(line, sNormal, eNormal);
			}
		}

		// ��������
		property.index = i;

		// �Ƿ�������������ڰ�
		property.isleft = tracks.isLeft;

		// ��������Ļ����˱��
		property.robotSelect = 0.0;

		// ���캸�ųߴ�
		property.weldAngel = vtWeldParaFlat[0].nWeldAngleSize;

		// 
		property.zSide = tracks.ZSide;

		// ���յ㷨��
		property.sNDir._x = tracks.points_normal[0].x;
		property.sNDir._y = tracks.points_normal[0].y;
		property.sNDir._z = tracks.points_normal[0].z;

		property.eNDir._x = tracks.points_normal[tracks.points_size - 1].x;
		property.eNDir._y = tracks.points_normal[tracks.points_size - 1].y;
		property.eNDir._z = tracks.points_normal[tracks.points_size - 1].z;

		// ����˵�����
		property.startPointType = tracks.start_point_type;
		property.endPointType = tracks.end_point_type;


		// һ������
		xrc::weld::WeldSeam ws;
		ws.setWeldWire(weldWire);     // ���ú�����
		ws.setWeldProperty(property); // ���ú�������

		tWeldWorkpiece.welds_.push_back(ws);
	}
	weldDoc.weldingWorks_.push_back(tWeldWorkpiece);  // ��Ӻ���

	// ���浽�ļ�
	bool bRst = xrc::weld::WeldDataDocJson::write(weldDoc, output_weldsinfo_path);
}

//void WeldAfterMeasure::LoadWeldInfoToJson(Welding_Info& tWeldInfo, int nWeldIndex, std::vector<T_ROBOT_COORS>& vtContour)
//{
//	Welding_Track tracks;			// �켣����
//
//
//	//// ����Json�ĵ�
//	xrc::weld::WeldDataDocJson weldDoc;
//	//CString save_weldsinfo_path = OUTPUT_PATH + m_ptUnit->m_tContralUnit.strUnitName + "\\" + IDENTIFY_RESULT_SAVE_JSON;
//
//	//bool bRst = xrc::weld::readWeldDataDocJson(save_weldsinfo_path, weldDoc);
//
//	//for (int seamPtnNum = 0; seamPtnNum < weldDoc.weldInfos[nWeldIndex].getWeldWire().lineSize(); seamPtnNum++)
//	//{
//	//	T_ROBOT_COORS Ptn;
//
//	//	Ptn.dX = weldDoc.weldInfos[nWeldIndex].getWeldWire().getSegments().at(seamPtnNum).endPoint_._x;
//
//	//	Ptn.dX = weldDoc.weldInfos[nWeldIndex].getWeldWire().getSegments().at(seamPtnNum).midPoint_._x;
//
//	//	Ptn.dX = weldDoc.weldInfos[nWeldIndex].getWeldWire().getStartPnt()._x;
//	//	Ptn.dY = weldDoc.weldInfos[nWeldIndex].getWeldWire().getStartPnt()._y;
//	//	Ptn.dZ = weldDoc.weldInfos[nWeldIndex].getWeldWire().getStartPnt()._z;
//	//}
//
//	// ��������, Ĭ������, �޸���������
//	xrc::weld::WeldProperty property;
//
//	// ����˵��������
//	property.startWrapCorner = 0.0;
//	property.endWrapCorner = 0.0;
//
//	// �����״�С
//	property.sHoleSize = m_dWeldHoleSize;
//	property.eHoleSize = m_dWeldHoleSize;
//
//	// �Ƿ�˫�溸��
//	property.doubleWelding = 0;
//
//	// ���ӳ��� Ĭ����Ч
//	property.weldingLength = -1;
//
//	std::vector<T_WELD_PARA> vtWeldParaFlat;
//	GetCurWeldParam(E_FLAT_SEAM, vtWeldParaFlat);
//
//	for (size_t i = 0; i < tWeldInfo.tracks_size; i++)
//	{
//		tracks = tWeldInfo.tracks[i];
//
//		// ������, ���һ��Բ��
//		xrc::weld::WeldWire weldWire;
//
//		for (size_t j = 0; j < tracks.points_size - 1; j++)
//		{
//			xrc::weld::GeoXYZ sPoint{ tracks.points[j].x, tracks.points[j].y, tracks.points[j].z };
//			xrc::weld::GeoXYZ ePoint{ tracks.points[j + 1].x, tracks.points[j + 1].y, tracks.points[j + 1].z };
//
//			// ����һ��ֱ��
//			xrc::weld::ThreePointLine line{ sPoint, ePoint };
//
//
//			xrc::weld::GeoXYZ sNormal{ tracks.points_normal[j].x , tracks.points_normal[j].y , tracks.points_normal[j].z };
//			xrc::weld::GeoXYZ eNormal{ tracks.points_normal[j + 1].x , tracks.points_normal[j + 1].y ,tracks.points_normal[j + 1].z };
//
//			weldWire.addBack(line, sNormal, eNormal);
//		}
//
//		// ��������
//		property.index = i;
//
//		// �Ƿ�������������ڰ�
//		property.isleft = tracks.isLeft;
//
//		// ��������Ļ����˱��
//		property.robotSelect = 0.0;
//
//		// ���캸�ųߴ�
//		property.weldAngel = vtWeldParaFlat[0].nWeldAngleSize;
//
//		// 
//		property.zSide = tracks.ZSide;
//
//		// ���յ㷨��
//		property.sNDir._x = tracks.points_normal[0].x;
//		property.sNDir._y = tracks.points_normal[0].y;
//		property.sNDir._z = tracks.points_normal[0].z;
//
//		property.eNDir._x = tracks.points_normal[tracks.points_size - 1].x;
//		property.eNDir._y = tracks.points_normal[tracks.points_size - 1].y;
//		property.eNDir._z = tracks.points_normal[tracks.points_size - 1].z;
//
//		// ����˵�����
//		property.startPointType = tracks.start_point_type;
//		property.endPointType = tracks.end_point_type;
//
//
//		// һ������
//		xrc::weld::WeldSeam ws;
//		ws.setWeldWire(weldWire);     // ���ú�����
//		ws.setWeldProperty(property); // ���ú�������
//
//		weldDoc.weldInfos.push_back(ws);  // ��Ӻ���
//
//
//		// ���浽�ļ�
//
//		CString output_weldsinfo_path = OUTPUT_PATH + m_ptUnit->m_tContralUnit.strUnitName + "\\" + POINT_CLOUD_IDENTIFY_RESULT_JSON;
//		bool bRst = xrc::weld::writeWeldDataDocJson(weldDoc, output_weldsinfo_path);
//	}
//}

void WeldAfterMeasure::BackHome()
{
	//int nRobotNo = 1 == XiMessageBox("ȷ��������1,ȡ��������2") ? 0 : 1;
	int nRobotNo = 0;
	CUnit* pUnit = m_ptUnit;
	CRobotDriverAdaptor* pRobotDriver = m_pRobotDriver;
	CHECK_BOOL(pUnit->CheckIsReadyRun());

	// ���������� or ���һ����� ��
	double dRobotInstallMode = pRobotDriver->m_nRobotInstallDir; // ��װ��1.0   ��װ��-1.0
	double dMinUpMoveDis = 100.0;
	double dBackHeightThreshold = 400.0;
	T_ANGLE_PULSE tCurPulse;
	T_ROBOT_COORS tCurCoord;
	T_ROBOT_COORS tTransCoord;
	T_ROBOT_COORS tTempCoord;
	T_ROBOT_COORS tHomeCoord;
	T_ROBOT_MOVE_INFO tRobotMoveInfo;
	vector<T_ROBOT_MOVE_INFO> vtRobotMoveInfo;
	vtRobotMoveInfo.clear();

	tCurCoord = pRobotDriver->GetCurrentPos();
	tCurPulse = pRobotDriver->GetCurrentPulse();
	pRobotDriver->RobotKinematics(pRobotDriver->m_tHomePulse, pRobotDriver->m_tTools.tGunTool, tHomeCoord);
	pRobotDriver->RobotKinematics(tCurPulse, pRobotDriver->m_tTools.tGunTool, tTransCoord);

	// ��ȡֱ������Ͷ�ȡ�ؽ�����ת�����������ͬʱ�������˶�
	if (false == pRobotDriver->CompareCoords(tTransCoord, tCurCoord))
	{
		WriteLog("MoveToSafeHeight:��ȡ����ͼ�������������");
		XiMessageBoxOk("��ȡ����ͼ������겻ͬ���޷��Զ��ذ�ȫλ�ã�");
		return;
	}

	double dMinBackGunHeight = tHomeCoord.dZ - (dBackHeightThreshold * dRobotInstallMode);//��С�����߶Ȱ�ȫλ�ü�����ֵ
	// �߶�С��dMinBackGunHeightʱ ִ�в�����1��RY�ָ���׼45  2������̧��100mm��MOVL����̫С�ٶȷǳ��죩
	tTempCoord = tCurCoord;
	tTempCoord.dRX = tHomeCoord.dRX;
	tTempCoord.dRY = tHomeCoord.dRY;
	if ((dRobotInstallMode > 0.0 && tCurCoord.dZ < dMinBackGunHeight) ||
		(dRobotInstallMode < 0.0 && tCurCoord.dZ > dMinBackGunHeight))//�жϣ���ǰλ���������С�߶ȱȽ�
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
	pUnit->RobotCheckDone();

	if (false == pRobotDriver->ComparePulse(pRobotDriver->m_tHomePulse))
	{
		//SetHintInfo("�ذ�ȫλ��ʧ�ܣ�");
		return;
	}
	//SetHintInfo("�ذ�ȫλ����ɣ�");

	//���ƶ�������-y����
	if (-1 != pUnit->m_nMeasureAxisNo_up)
	{
		if (0 != pUnit->MoveExAxisFun(0, 9000, 2))return;
		pUnit->WorldCheckRobotDone();
		//��Ҫ���Ļ�ȡ�����ݣ���ʱ����
		double dCurExPos = pUnit->GetExPositionDis(2);
		if (fabs(0 - dCurExPos) > 5.0)
		{
			XiMessageBox("�Զ�ʾ�̣��ⲿ��δ�˶���ָ��λ��");
			return;
		}
	}
	//if ((1 == pRobotDriver->m_nRobotInstallDir) && (IDOK == XiMessageBox("�Ƿ�����ϰ�ȫλ�ã�")))
	//{
	   // T_ANGLE_PULSE pSafePoint;
	   // double dAxisUnitT = pRobotDriver->m_tAxisUnit.dSPulse; // T�����嵱��
	   // pSafePoint.nSPulse = pRobotDriver->m_tHomePulse.nSPulse + (90.0 / dAxisUnitT);
	   // pSafePoint.nLPulse = pRobotDriver->m_tHomePulse.nLPulse;
	   // pSafePoint.nUPulse = pRobotDriver->m_tHomePulse.nUPulse;
	   // pSafePoint.nRPulse = pRobotDriver->m_tHomePulse.nRPulse;
	   // pSafePoint.nBPulse = pRobotDriver->m_tHomePulse.nBPulse;
	   // pSafePoint.nTPulse = pRobotDriver->m_tHomePulse.nTPulse;
	   // pRobotDriver->MoveByJob(pSafePoint, pRobotDriver->m_tBackHomeSpeed, pRobotDriver->m_nExternalAxleType, "MOVJ");
	   // pRobotDriver->CheckRobotDone();
	   // SetHintInfo("�����ϰ�ȫλ����ɣ�");
	//}

	return;
}