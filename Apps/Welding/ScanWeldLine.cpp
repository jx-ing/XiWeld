#include "stdafx.h"
#include "ScanWeldLine.h"
#include "WeldDataJson.h"
#include "numeric"


ScanWeldLine::ScanWeldLine(CUnit* ptUnit, E_WORKPIECE_TYPE eWorkPieceType) :
	WeldAfterMeasure(ptUnit, eWorkPieceType)
{
}

ScanWeldLine::~ScanWeldLine()
{

}

void ScanWeldLine::SetRecoParam()
{
	COPini opini;
	opini.SetFileName(DATA_PATH + m_ptUnit->m_tContralUnit.strUnitName + SYETEM_PARAM_FILE);

	opini.SetSectionName("E_SCAN_WELD_LINE");
	opini.ReadString("ParamValue0", &m_dZPallet);
	opini.ReadString("ParamValue1", &m_dScanSpeed);
	opini.ReadString("ParamValue2", &m_dMaxLength);
	opini.ReadString("ParamValue3", &m_VisionType);
	opini.ReadString("ParamValue4", &m_LiRon);
	opini.ReadString("ParamValue5", &m_bIsCake);
	opini.ReadString("ParamValue6", &m_nScanNum);
	opini.ReadString("ParamValue7", &m_nWeldHole);

	opini.SetSectionName("EquipmentParam");
	opini.ReadString("ScanTrackingWeldEnable", &m_nScanTrackingWeldEnable);

}

bool ScanWeldLine::PointCloudProcess(bool bUseModel, CvPoint3D64f* pPointCloud/* = NULL*/, int PointCloudSize/* = 0*/)
{
	if (bUseModel)
		return PointCloudProcessWithModel();

//	bool bRst;
	Welding_Info tWeldInfo;
	vector<CvPoint3D64f> vtPointCloud;
	//m_sPointCloudFileName = "LineScan\\Gantry\Scan5D.txt";
	int nCurUseTable = 0;
	COPini opini2;

	opini2.SetFileName(DATA_PATH + m_ptUnit->m_tContralUnit.strUnitName + LINE_SCAN_PARAM);
	opini2.SetSectionName("CurUseTableNo");
	opini2.ReadString("CurUseTableNo", &nCurUseTable);

	//��ɨ
	std::string FileName = "LineScan\\GantryLeft\\PointCloud\\Scan5D.txt";
	if (nCurUseTable != 0)
		std::string FileName = "LineScan\\GantryRight\\PointCloud\\Scan5D.txt";
	try
	{
		//LoadContourData(m_pRobotDriver, m_sPointCloudFileName, vtPointCloud);
		//pPointCloud = (CvPoint3D64f*)vtPointCloud.data();
		//PointCloudSize = vtPointCloud.size();

		//std::string FileName = ".\\LineScan\\Gantry\\PointCloud\\Scan5D.txt";
		std::string FileNameSave = ".\\LineScan\\Gantry\\PointCloud\\Scan5D-Background.bin";
		if (NULL == pPointCloud)
		{

			int maxPntNum = 60000000;
			int CoverNum = 1;
			int threadnum = 32;

			vtPointCloud.resize(maxPntNum);
			clock_t tTime = clock();
			int nCloudSize = TXTCloudFileRead((char*)FileName.c_str(), vtPointCloud.data(), maxPntNum, CoverNum, threadnum);
			vtPointCloud.resize(nCloudSize);
			//XiMessageBox("%d", clock() - tTime);
			pPointCloud = (CvPoint3D64f*)vtPointCloud.data();
			PointCloudSize = vtPointCloud.size();
		}

		std::string func = "LocalFiles\\ExLib\\Vision\\ConfigFiles\\Get_Welding_Info.ini";


		// Τ����������������ӿ�
		if (m_bIsCake == 1)
		{
			std::string workbench_height_str = std::to_string(m_dZPallet);
			std::string disconnect_long_track_length_str = std::to_string(m_dMaxLength);
			//const char* workbench_height = std::to_string(m_dZPallet).c_str();
			SetPara(func.c_str(), "Get_Welding_Info", "debug_mode", "false");
			SetPara(func.c_str(), "Get_Welding_Info", "is_upright", m_nRobotInstallDir > 0 ? "true" : "false");
			SetPara(func.c_str(), "Get_Welding_Info", "workbench_height", workbench_height_str.c_str());
			SetPara(func.c_str(), "Get_Welding_Info", "disconnect_long_track_length", disconnect_long_track_length_str.c_str());
			SetPara(func.c_str(), "Get_Welding_Info", "thread_nums", "4");
			SetPara(func.c_str(), "Get_Welding_Info", "debug_path", "D:\\XiRobotSW\\LocalFiles\\ExLib\\Vision\\ConfigFiles\\test");
			long long l1 = XI_clock();

			bool bRst = Get_Welding_Info(
				(Three_DPoint*)pPointCloud, PointCloudSize, tWeldInfo,
				func.c_str());
			long long l2 = XI_clock();
			WriteLog("ȫɨ�������ȡ������ʱ��%dms", l2 - l1);

			//WeldInfoToJson(tWeldInfo);
			l1 = XI_clock();
			SavePointCloudProcessResult(tWeldInfo);
			l2 = XI_clock();
			WriteLog("���溸����ʱ��%dms", l2 - l1);
			//���޸�
			CString str = 0 ? "�ɹ�" : "ʧ��";
			str = XUI::Languge::GetInstance().translate(str.GetBuffer());
			//XiMessageBox("����%s ��������Ϊ:%d", bRst ? "�ɹ�" : "ʧ��", tWeldInfo.tracks_size);
			XUI::MesBox::PopInfo("����{0}��������Ϊ:{1}", str.GetBuffer(), tWeldInfo.tracks_size);	
		}
		else if (m_bIsCake == 0)
		{

			m_vtWeldSeamData.clear();
			int nWeldingSeamNumber = 0;
			bool bInstallDir = 1 == m_nRobotInstallDir ? true : false;
			vector<CvPoint3D64f> vtPointCloud;
			LineOrCircularArcWeldingLine* pLineOrCircularArcWeldingLine;

			
			std::string func = "LocalFiles\\ExLib\\Vision\\ConfigFiles\\Get_LineOrCircularArcWeldingLine_JiaMuSi_Prefab_01.ini";
			std::string welding_line_segment_length = std::to_string(m_dMaxLength);
			SetPara(func.c_str(), "Get_LineOrCircularArcWeldingLine_JiaMuSi_Prefab_01", "debug_mode", "false");
			SetPara(func.c_str(), "Get_LineOrCircularArcWeldingLine_JiaMuSi_Prefab_01", "is_upright", m_nRobotInstallDir > 0 ? "true" : "false");
			SetPara(func.c_str(), "Get_LineOrCircularArcWeldingLine_JiaMuSi_Prefab_01", "welding_line_segment_length", welding_line_segment_length.c_str());
			SetPara(func.c_str(), "Get_LineOrCircularArcWeldingLine_JiaMuSi_Prefab_01", "thread_nums", "4");
			SetPara(func.c_str(), "Get_LineOrCircularArcWeldingLine_JiaMuSi_Prefab_01", "debug_path", "D:\\XiRobotSW\\LocalFiles\\ExLib\\Vision\\ConfigFiles\\test");
			bool bRst = Get_LineOrCircularArcWeldingLine_JiaMuSi_Prefab_01(
				(Three_DPoint*)pPointCloud, PointCloudSize, pLineOrCircularArcWeldingLine, nWeldingSeamNumber,
				func.c_str());
			//����ת��
			// LineOrCircularArcWeldingLine -> Welding_Info 
			// �Ӿ��ⶨ�庸����Ϣ�ṹ�� Welding_Info ���£�
			//Welding_Track* tracks;			// �켣����
			//unsigned int tracks_size;		// �켣���鳤��
			tWeldInfo.tracks_size = nWeldingSeamNumber;
			tWeldInfo.tracks = new Welding_Track[nWeldingSeamNumber];
			SavePointCloudProcessResult((LineOrCircularArcWeldingLine*)pLineOrCircularArcWeldingLine, nWeldingSeamNumber);
			//���޸�
			//XiMessageBox("����%s ��������Ϊ:%d", bRst ? "�ɹ�" : "ʧ��", nWeldingSeamNumber);
			CString str = 0 ? "�ɹ�" : "ʧ��";
			str = XUI::Languge::GetInstance().translate(str.GetBuffer());
			XUI::MesBox::PopInfo("����{0}��������Ϊ:{1}", str.GetBuffer(), nWeldingSeamNumber);

		}

		return true;
	}
	catch (...)
	{
		XiMessageBox("������ȡ�������������쳣");
		return false;
	}
	return true;
}

Local_Welding_Info ScanWeldLine::PartPointCloudProcess(vector<T_ROBOT_COORS>& vtCoord, int nGroupNo)
{
	int* pImageNum;
	vector<int> vtImageNum;
	Local_Welding_Info tLocalWeldingTrackInfo;
	vector<CvPoint3D64f> vtPointCloud;
	CvPoint3D64f* pPointCloud = NULL;
	int PointCloudSize = 0;
	CvPoint3D64f* tRefPointCloud = NULL;
	int RefPointCloudSize = 0;
	T_ROBOT_COORS coor;

	m_sPointCloudFileName = "LocalFiles\\OutputFiles\\RobotA\\Recognition\\0_ScanWeldLinePointCloud.txt";

	WeldLineInfo tWeldInfo = m_vvtWeldLineInfoGroup[nGroupNo][0];
	LineOrCircularArcWeldingLine tWeldLine = tWeldInfo.tWeldLine;
	int nWeldIndex = tWeldInfo.tAtrribute.nWeldSeamIdx;
	CString RefPointCloudFileName;
	RefPointCloudFileName.Format("%s00_PointCloudRecoTrack%d.txt", OUTPUT_PATH + m_ptUnit->m_tContralUnit.strUnitName + RECOGNITION_FOLDER, nWeldIndex);
	try
	{
		//���ص�������
		vector<CvPoint3D64f> vtPointCloud;
		LoadContourData3D(m_pRobotDriver, m_sPointCloudFileName, vtPointCloud);

		pPointCloud = (CvPoint3D64f*)vtPointCloud.data();
		PointCloudSize = vtPointCloud.size();

		// ����ȫ�������ȡ�Ķ�Ӧ����
		vector<CvPoint3D64f> vtRefPointCloud;
//		CvPoint3D64f tmp3DPoint;

		vector<CvPoint3D64f> vtTmpPointCloud;
		int TmpPointCloudSize = 0;
		CvPoint3D64f* tTmpPointCloud = NULL;
		tTmpPointCloud = (CvPoint3D64f*)vtTmpPointCloud.data();

		FILE* pf1 = fopen(RefPointCloudFileName, "r");
		int nNo = 0;
		CvPoint3D64f tTempPtn;
		double x;
		int pcImageNum = 0;

		while (EOF != fscanf(pf1, "%d%lf%lf%lf%lf%lf%lf", &nNo, &tTempPtn.x, &tTempPtn.y, &tTempPtn.z, &x, &x, &x))
		{
			vtRefPointCloud.push_back(tTempPtn);
		}
		fclose(pf1);

		tRefPointCloud = (CvPoint3D64f*)vtRefPointCloud.data();
		RefPointCloudSize = vtRefPointCloud.size();
		pImageNum = (int*)m_vtImageNum.data();
		if (m_bIsCake == 1)
		{
			// Τ����������������ӿ�
			SetPara("LocalFiles\\ExLib\\Vision\\ConfigFiles", "Rebuild_Local_Point_Cloud", "debug_mode", "true");
			SetPara("LocalFiles\\ExLib\\Vision\\ConfigFiles", "Rebuild_Local_Point_Cloud", "debug_path", "D:\\XiRobotSW\\LocalFiles\\ExLib\\Vision\\ConfigFiles\\test");
			SetPara("LocalFiles\\ExLib\\Vision\\ConfigFiles", "Rebuild_Local_Point_Cloud", "thread_nums", "4");
			SetPara("LocalFiles\\ExLib\\Vision\\ConfigFiles", "Get_Local_Welding_Info", "debug_mode", "true");
			SetPara("LocalFiles\\ExLib\\Vision\\ConfigFiles", "Get_Local_Welding_Info", "debug_path", "D:\\XiRobotSW\\LocalFiles\\ExLib\\Vision\\ConfigFiles\\test");
			bool bRst1 = Rebuild_Local_Point_Cloud(pImageNum, (Three_DPoint*)pPointCloud, PointCloudSize, (Three_DPoint*&)tTmpPointCloud, TmpPointCloudSize, "LocalFiles\\ExLib\\Vision\\ConfigFiles");
			bool bRst2 = Get_Local_Welding_Info((Three_DPoint*)pPointCloud, PointCloudSize, (Three_DPoint*)tRefPointCloud, RefPointCloudSize, tLocalWeldingTrackInfo, "LocalFiles\\ExLib\\Vision\\ConfigFiles");
		}
		else if (m_bIsCake == 0)
		{
			// Τ����������������ӿ�
			SetPara("LocalFiles\\ExLib\\Vision\\ConfigFiles", "Rebuild_Local_Point_Cloud", "debug_mode", "true");
			SetPara("LocalFiles\\ExLib\\Vision\\ConfigFiles", "Rebuild_Local_Point_Cloud", "debug_path", "D:\\XiRobotSW\\LocalFiles\\ExLib\\Vision\\ConfigFiles\\test");
			SetPara("LocalFiles\\ExLib\\Vision\\ConfigFiles", "Rebuild_Local_Point_Cloud", "thread_nums", "4");
			SetPara("LocalFiles\\ExLib\\Vision\\ConfigFiles", "Get_Local_Welding_Info_��ľ˹���Ӽ�", "debug_mode", "true");
			SetPara("LocalFiles\\ExLib\\Vision\\ConfigFiles", "Get_Local_Welding_Info_��ľ˹���Ӽ�", "debug_path", "D:\\XiRobotSW\\LocalFiles\\ExLib\\Vision\\ConfigFiles\\test");
			//bool bRst1 = Rebuild_Local_Point_Cloud(pImageNum, (Three_DPoint*)pPointCloud, PointCloudSize, (Three_DPoint*&)tTmpPointCloud, TmpPointCloudSize, "LocalFiles\\ExLib\\Vision\\ConfigFiles");
			bool bRst2 = Get_Local_Welding_Info((Three_DPoint*)pPointCloud, PointCloudSize, (Three_DPoint*)tRefPointCloud,
				RefPointCloudSize, tLocalWeldingTrackInfo, "D:\\XiRobotSW\\LocalFiles\\ExLib\\Vision\\ConfigFiles\\Get_Local_Welding_Info_��ľ˹���Ӽ�.ini");
		}
		FILE* pf = fopen("LocalFiles\\OutputFiles\\0_ScanWeldLine.txt", "w");
		for (unsigned int i = 0; i < tLocalWeldingTrackInfo.points_size; i++)
		{
			fprintf(pf, "%d %lf %lf %lf\n", i, tLocalWeldingTrackInfo.points[i].x, tLocalWeldingTrackInfo.points[i].y, tLocalWeldingTrackInfo.points[i].z);
			coor.dX = tLocalWeldingTrackInfo.points[i].x;
			coor.dY = tLocalWeldingTrackInfo.points[i].y;
			coor.dZ = tLocalWeldingTrackInfo.points[i].z;

			vtCoord.push_back(coor);
		}



		fclose(pf);

		//if (fabs(vtCoord[0].dZ - vtRefPointCloud[0].z) > 15.0)
		//{
		//	XiMessageBox("z����ʶ����������10mm������ʶ����");
		//	vtCoord.clear();
		//	return tLocalWeldingTrackInfo;
		//}

		if (TRUE == m_ptUnit->m_bNaturalPop)
		{
			/*XiMessageBox("����%s ��������Ϊ:%d", bRst2 ? "�ɹ�" : "ʧ��", tLocalWeldingTrackInfo.points_size);*/
		}
		return tLocalWeldingTrackInfo;
	}
	catch (...)
	{
		XiMessageBox("������ȡ�������������쳣");
		return tLocalWeldingTrackInfo;
	}

}

bool ScanWeldLine::WeldSeamGrouping(int& nWeldGroupNum)
{
	// �Զ�����
	if (g_bAutoGroupingMark)
	{
		bool bRst = WeldSeamGroupingAuto(nWeldGroupNum);
		return bRst;
	}
	//----------------------------------
	nWeldGroupNum = 0;
	double dGroupDisThreshold = 3.0; // mm
	double dMinFlatSeamLen = m_dHandEyeDis > 0.0 ? m_dHandEyeDis + m_dMeasureDisThreshold * 2.0 : 40.0; // ��Сƽ������
	double dMinStandSeamLen = 40.0; // ��С��������
	m_vvtWeldSeamGroup.clear();
	// ����ƽ������ �� ƽ������յ�����
	vector<WeldLineInfo> vtWeldSeamPlane(0);
	vector<WeldLineInfo> vtWeldLineStandard(0);
	vector<WeldLineInfo> vtWeldLineArc(0);
	vector<WeldLineInfo> vtWeldLinePlaneOverLength(0);
	vector<CvPoint3D64f> vtPtn(0); // ƽ������ʹ��(�ϲ��غϵĵ�)
	vector<vector<int>> vvnWeldNo(0); // ƽ������ʹ��(ÿ���ϲ����������Ķ�������ź�)
	XiAlgorithm alg;
	for (int nWeldNo = 0; nWeldNo < m_vtWeldSeamInfo.size(); nWeldNo++)
	{
		WeldLineInfo& tLineSeamInfo = m_vtWeldSeamInfo[nWeldNo];
		LineOrCircularArcWeldingLine& tLineSeam = tLineSeamInfo.tWeldLine;
		double dWeldSeamLen = TwoPointDis(
			tLineSeam.StartPoint.x, tLineSeam.StartPoint.y, tLineSeam.StartPoint.z,
			tLineSeam.EndPoint.x, tLineSeam.EndPoint.y, tLineSeam.EndPoint.z);
		tLineSeamInfo.tAtrribute.bWeldMode = false; // Ĭ�ϲ�����
		if (IsStandWeldSeam(tLineSeam.StartNormalVector)) // ��������յ㷨����ZֵΪ0
		{
			if (dWeldSeamLen < dMinStandSeamLen)
			{
				XUI::MesBox::PopInfo("��������{0} ����С��{1},�Զ�ɾ����", nWeldNo, dMinStandSeamLen);
				continue;
			}
			vtWeldLineStandard.push_back(tLineSeamInfo);
		}
		else
		{
			if (true == m_bTrackingEnable && dWeldSeamLen > m_dTrackingLengthThreshold) // �������������
			{
				tLineSeamInfo.tAtrribute.bWeldMode = true;
			}
			// Բ�����쵥������
			if (WeldingLineIsArc(tLineSeam))
			{
				vtWeldLineArc.push_back(tLineSeamInfo);
				continue;
			}
			// ����ƽ���쵥������
			if (dWeldSeamLen > m_dLengthSeamThreshold)
			{
				double dNorAngle = alg.CalcArcAngle(tLineSeam.StartNormalVector.x, tLineSeam.StartNormalVector.y);
				double dWeldSeamDir = alg.CalcArcAngle(tLineSeam.EndPoint.x - tLineSeam.StartPoint.x, tLineSeam.EndPoint.y - tLineSeam.StartPoint.y);
				if (false == JudgeDirAngle(dNorAngle + (90.0 * m_nRobotInstallDir), dWeldSeamDir)) // ����յ㷽��Ǹ��ٺ��ӷ���
				{
					swap(tLineSeam.StartPoint, tLineSeam.EndPoint); // ��������յ�������Ϣ
					swap(tLineSeam.StartNormalVector, tLineSeam.EndNormalVector);
					swap(tLineSeam.StartPointType, tLineSeam.EndPointType);
					swap(tLineSeamInfo.tAtrribute.dStartHoleSize, tLineSeamInfo.tAtrribute.dEndHoleSize);
					swap(tLineSeamInfo.tAtrribute.nStartWrapType, tLineSeamInfo.tAtrribute.nEndWrapType);
					m_vtWeldSeamData[nWeldNo] = tLineSeam;
					m_vtWeldSeamInfo[nWeldNo].tWeldLine = tLineSeam;
					m_vtWeldSeamInfo[nWeldNo].tAtrribute = tLineSeamInfo.tAtrribute;
				}
				vtWeldLinePlaneOverLength.push_back(tLineSeamInfo);
				continue;
			}
			if (dWeldSeamLen < dMinFlatSeamLen && dWeldSeamLen > 15.0)
			{
				XUI::MesBox::PopInfo("ƽ������{0} ����С��{1},�Զ�ɾ����", nWeldNo, dMinFlatSeamLen);
				continue;
			}
			if (dWeldSeamLen < m_dShortSeamThreshold &&
				0 < tLineSeam.StartPointType &&
				0 < tLineSeam.EndPointType)
			{
				XUI::MesBox::PopInfo("˫�����ƽ������{0} ����С��{1},�Զ�ɾ����", nWeldNo, m_dShortSeamThreshold);
				continue;
			}
			double dNorAngle = alg.CalcArcAngle(tLineSeam.StartNormalVector.x, tLineSeam.StartNormalVector.y);
			double dWeldSeamDir = alg.CalcArcAngle(tLineSeam.EndPoint.x - tLineSeam.StartPoint.x, tLineSeam.EndPoint.y - tLineSeam.StartPoint.y);
			if (false == JudgeDirAngle(dNorAngle + (90.0 * m_nRobotInstallDir), dWeldSeamDir)) // ����յ㷽��Ǹ��ٺ��ӷ���
			{
				swap(tLineSeam.StartPoint, tLineSeam.EndPoint); // ��������յ�������Ϣ
				swap(tLineSeam.StartNormalVector, tLineSeam.EndNormalVector);
				swap(tLineSeam.StartPointType, tLineSeam.EndPointType);
				swap(tLineSeamInfo.tAtrribute.dStartHoleSize, tLineSeamInfo.tAtrribute.dEndHoleSize);
				swap(tLineSeamInfo.tAtrribute.nStartWrapType, tLineSeamInfo.tAtrribute.nEndWrapType);
				m_vtWeldSeamData[nWeldNo] = tLineSeam;
				m_vtWeldSeamInfo[nWeldNo].tWeldLine = tLineSeam;
				m_vtWeldSeamInfo[nWeldNo].tAtrribute = tLineSeamInfo.tAtrribute;
			}
			vtWeldSeamPlane.push_back(tLineSeamInfo);
			// vtPtn�в����Ƿ���������յ�����ĵ� �� ��Ӧ��vnWeldNo���һ�������ţ�û�����һ���㵽vtPtn����¼�µĺ�����
			bool bFoundS = false;
			bool bFoundE = false;
			for (int nPtnNo = 0; nPtnNo < vtPtn.size(); nPtnNo++)
			{
				if (dGroupDisThreshold > TwoPointDis(vtPtn[nPtnNo].x, vtPtn[nPtnNo].y, vtPtn[nPtnNo].z,
					tLineSeam.StartPoint.x, tLineSeam.StartPoint.y, tLineSeam.StartPoint.z)) // S 
				{
					bFoundS = true;
					vvnWeldNo[nPtnNo].push_back(nWeldNo);
				}
				if (dGroupDisThreshold > TwoPointDis(vtPtn[nPtnNo].x, vtPtn[nPtnNo].y, vtPtn[nPtnNo].z,
					tLineSeam.EndPoint.x, tLineSeam.EndPoint.y, tLineSeam.EndPoint.z)) // E
				{
					bFoundE = true;
					vvnWeldNo[nPtnNo].push_back(nWeldNo);
				}
				if (vvnWeldNo[nPtnNo].size() > 2)
				{
					XiMessageBox("����������ƽ���ཻ��һ�㣬����ʶ������");
					return false;
				}
			}
			if (false == bFoundS)
			{
				vtPtn.push_back(tLineSeam.StartPoint);
				vector<int> vtWeldNo(0);
				vtWeldNo.push_back(nWeldNo);
				vvnWeldNo.push_back(vtWeldNo);
			}
			if (false == bFoundE)
			{
				vtPtn.push_back(tLineSeam.EndPoint);
				vector<int> vtWeldNo(0);
				vtWeldNo.push_back(nWeldNo);
				vvnWeldNo.push_back(vtWeldNo);
			}
		}
	}

	GenerateFilePLYPlane(vtWeldSeamPlane);

	// ƽ������ (ÿ����β����ƽ�����������)
	sort(vvnWeldNo.begin(), vvnWeldNo.end(), [](const vector<int>& vn1, const vector<int>& vn2) -> bool { return vn1.size() > vn2.size(); });
	vector<vector<int>> vvnEachGroupIndex(0);
	while (vvnWeldNo.size() > 0) // ÿ���˵�
	{
		vector<int> vnWeldNo(vvnWeldNo[0]); // ȡʣ���еĵ�һ���㣬���Ҹõ����ں����������ƽ��
		bool bFound = true;
		while (true == bFound && vvnWeldNo.size() > 0)
		{
			bFound = false;
			for (int nPtnNo = 0; (!bFound) && (nPtnNo < vvnWeldNo.size()); nPtnNo++) // δ�����ÿ��������
			{
				for (int nWeldNo = 0; (!bFound) && (nWeldNo < vvnWeldNo[nPtnNo].size()); nWeldNo++) // ÿ��������Ķ�������
				{
					for (int nNo = 0; (!bFound) && (nNo < vnWeldNo.size()); nNo++) // �ѷ����е�ÿ�������
					{
						if (vnWeldNo[nNo] == vvnWeldNo[nPtnNo][nWeldNo])
						{
							bFound = true;
							for (int nNewWeldNo = 0; nNewWeldNo < vvnWeldNo[nPtnNo].size(); nNewWeldNo++) // ֻ��¼��ǰδ��¼�����
							{
								vector<int>::iterator iter = find(vnWeldNo.begin(), vnWeldNo.end(), vvnWeldNo[nPtnNo][nNewWeldNo]);
								if (iter != vnWeldNo.end()) continue;
								vnWeldNo.push_back(vvnWeldNo[nPtnNo][nNewWeldNo]);
							}
							vvnWeldNo.erase(vvnWeldNo.begin() + nPtnNo);
						}
					}
				}
			}
		}
		vvnEachGroupIndex.push_back(vnWeldNo);
	}

	vector<WeldLineInfo> vtWeldLineInfo(0);
	m_vvtWeldLineInfoGroup.resize(vvnEachGroupIndex.size(), vtWeldLineInfo);
	for (int nGroupNo = 0; nGroupNo < vvnEachGroupIndex.size(); nGroupNo++) // ����ÿ�麸��
	{
		for (int nWeldNo = 0; nWeldNo < vvnEachGroupIndex[nGroupNo].size(); nWeldNo++)
		{
			m_vvtWeldLineInfoGroup[nGroupNo].push_back(m_vtWeldSeamInfo[vvnEachGroupIndex[nGroupNo][nWeldNo]]);
		}
	}

	// ƽ����������
	double dPlaneStandardDisThreshold = 2.0; // mm
	for (int nGroupNo = 0; nGroupNo < m_vvtWeldLineInfoGroup.size(); nGroupNo++)
	{
		vector<WeldLineInfo> vtGroup = m_vvtWeldLineInfoGroup[nGroupNo];
		for (int nSeamNo = 0; nSeamNo < vtGroup.size(); nSeamNo++)
		{
			WeldLineInfo& tSeamPlane = vtGroup[nSeamNo];
			if (0 < tSeamPlane.tWeldLine.StartPointType) // ���������
			{
				for (int nStardWeldNo = 0; nStardWeldNo < vtWeldLineStandard.size(); nStardWeldNo++)
				{
					LineOrCircularArcWeldingLine& tSeamStandard = vtWeldLineStandard[nStardWeldNo].tWeldLine;
					double dDis = TwoPointDis(tSeamPlane.tWeldLine.StartPoint.x, tSeamPlane.tWeldLine.StartPoint.y, tSeamPlane.tWeldLine.StartPoint.z,
						tSeamStandard.StartPoint.x, tSeamStandard.StartPoint.y, tSeamStandard.StartPoint.z);
					if (fabs(dDis) < dPlaneStandardDisThreshold)
					{
						if (false == IsAcuteSeamStand(tSeamStandard, 80.0)) // ������������浽���� ����Ҫɾ��
						{
							m_vvtWeldLineInfoGroup[nGroupNo].push_back(vtWeldLineStandard[nStardWeldNo]);
						}
						else
						{
							WriteLog("������������Զ�ɾ��1");
						}
						vtWeldLineStandard.erase(vtWeldLineStandard.begin() + nStardWeldNo);
						break;
					}
				}
			}

			if (0 < tSeamPlane.tWeldLine.EndPointType) // �յ�������
			{
				for (int nStardWeldNo = 0; nStardWeldNo < vtWeldLineStandard.size(); nStardWeldNo++)
				{
					LineOrCircularArcWeldingLine& tSeamStandard = vtWeldLineStandard[nStardWeldNo].tWeldLine;
					double dDis = TwoPointDis(tSeamPlane.tWeldLine.EndPoint.x, tSeamPlane.tWeldLine.EndPoint.y, tSeamPlane.tWeldLine.EndPoint.z,
						tSeamStandard.StartPoint.x, tSeamStandard.StartPoint.y, tSeamStandard.StartPoint.z);
					if (fabs(dDis) < dPlaneStandardDisThreshold)
					{
						if (false == IsAcuteSeamStand(tSeamStandard, 80.0)) // ������������浽���� ����Ҫɾ��
						{
							m_vvtWeldLineInfoGroup[nGroupNo].push_back(vtWeldLineStandard[nStardWeldNo]);
						}
						else
						{
							WriteLog("������������Զ�ɾ��2");
						}
						vtWeldLineStandard.erase(vtWeldLineStandard.begin() + nStardWeldNo);
						break;
					}
				}
			}
		}
	}

	// ��ƽ���޹����ĵ����������� ����Ϊ����һ��
	for (int nAloneStandWeldNo = 0; nAloneStandWeldNo < vtWeldLineStandard.size(); nAloneStandWeldNo++)
	{
		vtWeldLineInfo.clear();
		vtWeldLineInfo.resize(1, vtWeldLineStandard[nAloneStandWeldNo]);
		m_vvtWeldLineInfoGroup.push_back(vtWeldLineInfo);
	}

	// ƽ���������챣��Ϊ����һ��
	for (int nWeldArcNo = 0; nWeldArcNo < vtWeldLinePlaneOverLength.size(); nWeldArcNo++)
	{
		vtWeldLineInfo.clear();
		vtWeldLineInfo.resize(1, vtWeldLinePlaneOverLength[nWeldArcNo]);
		m_vvtWeldLineInfoGroup.push_back(vtWeldLineInfo);
	}

	// Բ�����챣��Ϊ����һ��
	for (int nWeldArcNo = 0; nWeldArcNo < vtWeldLineArc.size(); nWeldArcNo++)
	{
		vtWeldLineInfo.clear();
		vtWeldLineInfo.resize(1, vtWeldLineArc[nWeldArcNo]);
		m_vvtWeldLineInfoGroup.push_back(vtWeldLineInfo);
	}

	// ���麸������
	sort(m_vvtWeldLineInfoGroup.begin(), m_vvtWeldLineInfoGroup.end(),
		[](const vector<WeldLineInfo>& seam1, const vector<WeldLineInfo>& seam2)-> bool
		{
#ifdef SINGLE_ROBOT
			double yDis1 = 0;
			double yDis2 = 0;
			for (int i = 0; i < seam1.size(); i++)
			{
				yDis1 += seam1[i].tWeldLine.StartPoint.y;
				yDis1 += seam1[i].tWeldLine.EndPoint.y;
			}
			for (int j = 0; j < seam2.size(); j++)
			{
				yDis2 += seam2[j].tWeldLine.StartPoint.y;
				yDis2 += seam2[j].tWeldLine.EndPoint.y;
			}
			yDis1 = yDis1 / 2.0 / seam1.size();
			yDis2 = yDis2 / 2.0 / seam2.size();
			return (yDis1 > yDis2);
#else
			double xDis1 = 0;
			double xDis2 = 0;
			for (int i = 0; i < seam1.size(); i++)
			{
				xDis1 += seam1[i].tWeldLine.StartPoint.x;
				xDis1 += seam1[i].tWeldLine.EndPoint.x;
			}
			for (int j = 0; j < seam2.size(); j++)
			{
				xDis2 += seam2[j].tWeldLine.StartPoint.x;
				xDis2 += seam2[j].tWeldLine.EndPoint.x;
			}
			xDis1 = xDis1 / 2.0 / seam1.size();
			xDis2 = xDis2 / 2.0 / seam2.size();
			return (xDis1 > xDis2);
#endif // SINGLE_ROBOT
		});

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
	SaveWeldSeamGroupInfo();
	return true;
}

bool ScanWeldLine::CalcMeasureTrack(int nGroupNo, std::vector<T_ROBOT_COORS>& vtMeasureCoord, std::vector<T_ANGLE_PULSE>& vtMeasurePulse, vector<int>& vnMeasureType, double& dExAxlePos, double& dSafeHeight)
{
	
	std::vector<T_ROBOT_COORS> vtContour;
	T_ROBOT_COORS tTotalCoord;
	m_vvtWeldSeamGroupAdjust = m_vvtWeldSeamGroup;
	// ��ȡ��nGroupNo�麸�����Ϣ ���ݺ���������ȷ�����������ļ�
	WeldLineInfo tWeldInfo = m_vvtWeldLineInfoGroup[nGroupNo][0];
	LineOrCircularArcWeldingLine tWeldLine = tWeldInfo.tWeldLine;
	int nWeldIndex = tWeldInfo.tAtrribute.nWeldSeamIdx;
	CString sContourFile = OUTPUT_PATH + m_ptUnit->m_tContralUnit.strUnitName + "\\" + IDENTIFY_RESULT_SAVE_JSON;
	//sContourFile.Format("%s00_PointCloudRecoTrack%d.txt", OUTPUT_PATH + m_ptUnit->m_tContralUnit.strUnitName + RECOGNITION_FOLDER, nWeldIndex);

	// ��ȡ�����ļ� �˲� ����ֱ������
	//CHECK_BOOL_RETURN(LoadContour(nWeldIndex, sContourFile, vtContour));
	CHECK_BOOL_RETURN(LoadContourByJson(nWeldIndex, nGroupNo, sContourFile, vtContour));

	double dWeldSeamLen = vtContour.size() * 5.0;
	m_iscircle = 0;
	m_isOneWled = 0;
	if (m_pRobotDriver->CompareXY(vtContour.at(0), vtContour.at(vtContour.size() - 1)))
	{
		m_iscircle = 1;
		m_isOneWled = 2;
	}

	if (false == CalcMeasureTrack_Normal(nGroupNo, vtMeasureCoord, vtMeasurePulse, vnMeasureType, dExAxlePos, dSafeHeight))
	{
		return false;
	}

	return true;
}

bool ScanWeldLine::CalcMeasureTrack_Normal(int nGroupNo, std::vector<T_ROBOT_COORS>& vtMeasureCoord, std::vector<T_ANGLE_PULSE>& vtMeasurePulse, vector<int>& vnMeasureType, double& dExAxlePos, double& dSafeHeight)
{
	std::vector<T_ROBOT_COORS> vtContour;
	T_ROBOT_COORS tTotalCoord;

	// ��ȡ��nGroupNo�麸�����Ϣ ���ݺ���������ȷ�����������ļ�
	WeldLineInfo tWeldInfo = m_vvtWeldLineInfoGroup[nGroupNo][0];
	LineOrCircularArcWeldingLine tWeldLine = tWeldInfo.tWeldLine;
	int nWeldIndex = tWeldInfo.tAtrribute.nWeldSeamIdx;
	CString sContourFile = OUTPUT_PATH + m_ptUnit->m_tContralUnit.strUnitName + "\\" + IDENTIFY_RESULT_SAVE_JSON;
	//sContourFile.Format("%s00_PointCloudRecoTrack%d.txt", OUTPUT_PATH + m_ptUnit->m_tContralUnit.strUnitName + RECOGNITION_FOLDER, nWeldIndex);

	// ��ȡ�����ļ� �˲� ����ֱ������
	//CHECK_BOOL_RETURN(LoadContour(nWeldIndex, sContourFile, vtContour));
	CHECK_BOOL_RETURN(LoadContourByJson(nWeldIndex, nGroupNo, sContourFile, vtContour));

	for (int num = 0; num < vtContour.size(); num++)
	{

		tTotalCoord.dX += vtContour[num].dX;
		tTotalCoord.dY += vtContour[num].dY;
	}

	// �����ⲿ��ƫ��
#ifdef SINGLE_ROBOT
	dExAxlePos = 0.0;
	double dExAxlePos_y = tTotalCoord.dY / vtContour.size();
#else
	dExAxlePos = tTotalCoord.dX / vtContour.size();
	double dExAxlePos_y = 0.0;// tTotalCoord.dY / vtContour.size();
#endif // SINGLE_ROBOT
	// X����ƫ�Ʒ�Χ

	vector<double> vdExAxleOffset = { dExAxlePos };
	if (1 == m_ptUnit->m_nMeasureAxisNo)
	{
		for (int i = 1; i < 10; i++)
		{
			vdExAxleOffset.push_back(dExAxlePos + (100.0 * i));
			vdExAxleOffset.push_back(dExAxlePos + (-100.0 * i));
		}
	}
	// Y����ƫ�Ʒ�Χ
	vector<double> vdExAxleOffset_y = { dExAxlePos_y };
	if (2 == m_ptUnit->m_nMeasureAxisNo)
	{
		for (int i = 1; i < 10; i++)
		{
			vdExAxleOffset_y.push_back(dExAxlePos_y + 100.0 * i);
			vdExAxleOffset_y.push_back(dExAxlePos_y + (-100.0 * i));
		}
	}

	bool bCalcSuccess = false;
	for (int nOffsetIdx = 0; !bCalcSuccess && nOffsetIdx < vdExAxleOffset.size(); nOffsetIdx++)
	{
		// �ֽ�X�����ⲿ������
		std::vector<T_ROBOT_COORS> vtCoor_ori = vtContour;
		if(1 == m_ptUnit->m_nMeasureAxisNo)
		DecomposeExAxle(vtCoor_ori, dExAxlePos, vdExAxleOffset[nOffsetIdx], m_ptUnit->m_nMeasureAxisNo);
	

		for (int nOffsetIdx_y = 0; !bCalcSuccess && nOffsetIdx_y < vdExAxleOffset_y.size(); nOffsetIdx_y++)
		{

			vtMeasureCoord.clear();
			vnMeasureType.clear();
			vtMeasurePulse.clear();
			std::vector<T_ROBOT_COORS> vtCoor = vtCoor_ori;


			// �ֽ�Y�����ⲿ������

			if (-1 != m_ptUnit->m_nMeasureAxisNo_up)
			{
				DecomposeExAxle(vtCoor, dExAxlePos, vdExAxleOffset_y[nOffsetIdx_y], m_ptUnit->m_nMeasureAxisNo_up);
			}
			else if(2 == m_ptUnit->m_nMeasureAxisNo)//X��ֽ���˲��طֽ�
			{
				DecomposeExAxle(vtCoor, dExAxlePos, vdExAxleOffset_y[nOffsetIdx_y], m_ptUnit->m_nMeasureAxisNo);
			}
			dExAxlePos = vdExAxleOffset[nOffsetIdx];
			dExAxlePos_y = vdExAxleOffset_y[nOffsetIdx_y];
			m_dTeachExAxlePos = dExAxlePos;
			m_dTeachExAxlePos_y = vdExAxleOffset_y[nOffsetIdx_y];

			if (vdExAxleOffset[nOffsetIdx] > 3300.0 || vdExAxleOffset[nOffsetIdx] < -4200.0)
			{
				WriteLog("�ⲿ��%d Ŀ��λ��%.3lf �����趨����%.3lf - %.3lf!", m_ptUnit->m_nMeasureAxisNo, vdExAxleOffset[nOffsetIdx], 3300.0, -4200.0);
				continue;
			}

			if (vdExAxleOffset_y[nOffsetIdx_y] > 2500.0 || vdExAxleOffset_y[nOffsetIdx_y] < -2500.0)
			{
				WriteLog("�ⲿ��%d Ŀ��λ��%.3lf �����趨����%.3lf - %.3lf!", m_ptUnit->m_nMeasureAxisNo_up, vdExAxleOffset_y[nOffsetIdx_y], 250.0, -1450.0);
				continue;
			}

			if (1 == m_LiRon)
			{
				//����˱���̬ɨ��
				if (1 == tWeldInfo.tWeldLine.StartPointType || 1 == tWeldInfo.tWeldLine.EndPointType)
				{
					CurveScanSide(tWeldInfo, dExAxlePos, dExAxlePos_y, vtCoor, 80.0, 90.0);
				}
	
			}
			
			//// �˲��������̬
			CHECK_BOOL_RETURN(TrackFilter(vtCoor, vtMeasureCoord));

			for (int i = 0;i < vtMeasureCoord.size();i++)
			{
				vtMeasureCoord[i].dZ += 20;
			}
			// ����Ӿ�ʶ�������λ�߶ȵ�����
			double evaZ = std::accumulate(vtMeasureCoord.begin(), vtMeasureCoord.end(), 0.0, [](double sum, const T_ROBOT_COORS& coor) { return sum + coor.dZ; }) / vtMeasureCoord.size();
			for (auto coor : vtMeasureCoord)
			{
				if (coor.dZ > evaZ)
				{
					coor.dZ = evaZ;
				}
			}
			
			bool isCon = false;
			int segmentNum = 20;
			int stepNum = vtMeasureCoord.size() / segmentNum;
			vector<T_ROBOT_COORS> tt = vtMeasureCoord;
			for (int nOffsetIdx = 0;!isCon && nOffsetIdx < stepNum; nOffsetIdx++)
			{

				// �����㲻����λ �������
				if (m_pRobotDriver->CompareXY(vtContour.at(0), vtContour.at(vtContour.size() - 1)))
				{
					vector<T_ROBOT_COORS> tmprealCoord;
					vector<T_ROBOT_COORS> tmpCoord = tt;
					int zeroRz = nOffsetIdx * segmentNum;
					double minRZdiff = 9999;

					tmprealCoord.assign(tmpCoord.begin() + zeroRz, tmpCoord.end());
					tmprealCoord.insert(tmprealCoord.end(), tmpCoord.begin(), tmpCoord.begin() + zeroRz);
					vtMeasureCoord = tmprealCoord;
				}
			/*	if (m_iscircle == 1)
				{
					vtMeasureCoord.push_back(vtMeasureCoord[0]);
					vtMeasureCoord.push_back(vtMeasureCoord[1]);
					vtMeasureCoord.push_back(vtMeasureCoord[2]);
					vtMeasureCoord.push_back(vtMeasureCoord[3]);
					vtMeasureCoord.push_back(vtMeasureCoord[4]);
				}*/
				m_vtContour = vtMeasureCoord;

				// �����������й켣
				if (!CalcContinuePulseForWeld(vtMeasureCoord, vtMeasurePulse, true))
				{
					continue;
				}
			}

			// ��������켣��ǹ��������
			SaveMeasureTrackGunTool(nGroupNo, vtCoor);

			// ��ʼ���켣������
			vnMeasureType.resize(vtMeasureCoord.size(), E_WELD_TRACK);

			for (auto& tCoord : vtMeasureCoord)
			{
				if (!m_pRobotDriver->MoveToolByWeldGun(
					tCoord,
					m_pRobotDriver->m_tTools.tGunTool,
					tCoord,
					m_pRobotDriver->m_tTools.tCameraTool,
					tCoord))
				{
					continue;
				}
			}
			// �������ǹ���ɵ�
			AddSafeCoord(vtMeasureCoord, vnMeasureType, tWeldLine.ZSide);			// �������ǹ���ɵ�

			
			// �����������й켣
			if (!CalcContinuePulseForWeld(vtMeasureCoord, vtMeasurePulse, true))
			{
				WriteLog("ExAxleOffset_y:%3lf CalcContinuePulseForWeld Fail!", vdExAxleOffset_y[nOffsetIdx_y]);
				continue;
			}

			if (!CheckFlangeToolCoord(vtMeasurePulse)) // ������������켣�� ���뷨�������Ƿ��㹻��
			{
				WriteLog("ExAxleOffset:%3lf CalcContinuePulseForWeld Fail!", vdExAxleOffset[nOffsetIdx]);
				WriteLog("ExAxleOffset_y:%3lf CalcContinuePulseForWeld Fail!", vdExAxleOffset_y[nOffsetIdx_y]);
				continue;
			}

			// ������ʶ�������ɴ����ۺ�����Ϣ �� �����Ƿ������������
			// ��֤����ʱ�ͺ������ⲿ�ἰС��λ����ͬ (����ʱ�Կ��������ƶ�) �����켣�ͺ��ӹ켣��������������ʱ�ųɹ�
			if (m_iscircle != 1 && !JudgeTheoryTrackContinueMove(nGroupNo, m_dTeachExAxlePos, m_dTeachExAxlePos_y)) {
				WriteLog("ExAxleOffset:%3lf JudgeTheoryTrackContinueMove Fail!", vdExAxleOffset[nOffsetIdx]);
				WriteLog("ExAxleOffset_y:%3lf JudgeTheoryTrackContinueMove Fail!", vdExAxleOffset_y[nOffsetIdx_y]);
				continue;
			}

			bCalcSuccess = true;
			m_conRz = vtMeasureCoord[0].dRZ;

			dExAxlePos = vtMeasureCoord[0].dBX;
			m_dTeachExAxlePos = vtMeasureCoord[0].dBX;
			m_dTeachExAxlePos_y = vtMeasureCoord[0].dBY;

			// ��������켣�����������
			SaveMeasureTrackCamTool(nGroupNo, vtMeasureCoord, vtMeasurePulse, vnMeasureType);
		}
	}
	if (false == bCalcSuccess)
	{
		XiMessageBox("��λ�ü������������켣ʧ�ܣ�");
		return false;
	}

	// ���ߵ���ʹ��
	CString sJobName;
	sJobName.Format("MOVJTEACHBOARD%d", nGroupNo);
	GenerateJobLocalVariable(vtMeasurePulse, MOVJ, sJobName, 1);
	return true;
}

bool ScanWeldLine::CalcMeasureTrack_Circle(int nGroupNo, std::vector<T_ROBOT_COORS>& vtMeasureCoord, std::vector<T_ANGLE_PULSE>& vtMeasurePulse, vector<int>& vnMeasureType, double& dExAxlePos, double& dSafeHeight)
{
	// ��ȡ��nGroupNo�麸�����Ϣ ���ݺ���������ȷ�����������ļ�
	WeldLineInfo tWeldInfo = m_vvtWeldLineInfoGroup[nGroupNo][0];
	LineOrCircularArcWeldingLine tWeldLine = tWeldInfo.tWeldLine;
	int nWeldIndex = tWeldInfo.tAtrribute.nWeldSeamIdx;
	CString sContourFile = OUTPUT_PATH + m_ptUnit->m_tContralUnit.strUnitName + "\\" + IDENTIFY_RESULT_SAVE_JSON;
	//sContourFile.Format("%s00_PointCloudRecoTrack%d.txt", OUTPUT_PATH + m_ptUnit->m_tContralUnit.strUnitName + RECOGNITION_FOLDER, nWeldIndex);

	// ��ȡ�����ļ� �˲� ����ֱ������
	std::vector<T_ROBOT_COORS> vtContour;
	//CHECK_BOOL_RETURN(LoadContour(nWeldIndex, sContourFile, vtContour));
	CHECK_BOOL_RETURN(LoadContourByJson(nWeldIndex, nGroupNo, sContourFile, vtContour));

	std::vector<T_ROBOT_COORS> vtCoor = vtContour;

	// ��ȡ���귶Χ �ж��Ƿ���
	double minX_val, maxX_val, minY_val, maxY_val;
	for (const auto& coor : vtContour)
	{
		minX_val = std::min(minX_val, coor.dX);
		maxX_val = std::max(maxX_val, coor.dX);
		minY_val = std::min(minY_val, coor.dY);
		maxY_val = std::max(maxY_val, coor.dY);
	}

	if (minX_val > 3300.0 || maxX_val < -4200.0)
	{
		double limit_exceeded = maxX_val > 3300.0 ? maxX_val : minX_val;
		//���޸�
		XUI::MesBox::PopInfo("�ع�(X)�����ⲿ�ᳬ�� Ŀ��λ��{0:.3f} �����趨����{1:.3f} - {2:.3f}!", limit_exceeded, 3300.0, -4200.0);
		//XiMessageBox("�ع�(X)�����ⲿ�ᳬ�� Ŀ��λ��%.3lf �����趨����%.3lf - %.3lf!", limit_exceeded, 3300.0, -4200.0);
		return false;
	}

	if (maxY_val > 2500.0 || minY_val < -2500.0)
	{
		double limit_exceeded = maxY_val > 2500.0 ? maxY_val : minY_val;
		//���޸�
		XUI::MesBox::PopInfo("����(Y)�����ⲿ�ᳬ�� Ŀ��λ��{0:.3f} �����趨����{1:.3f} - {2:.3f}!", limit_exceeded, 2500.0, -2500.0);
		//XiMessageBox("����(Y)�����ⲿ�ᳬ�� Ŀ��λ��%.3lf �����趨����%.3lf - %.3lf!", limit_exceeded, 2500.0, -2500.0);
		return false;
	}

	

	//����˱���̬ɨ��
	if (1 == tWeldInfo.tWeldLine.StartPointType || 1 == tWeldInfo.tWeldLine.EndPointType)
	{
		CurveScanSide(tWeldInfo, 0.0, 0.0, vtCoor, 60.0, 90.0);
	}

	// �˲��������̬
	CHECK_BOOL_RETURN(TrackFilter(vtCoor, vtMeasureCoord, 12));

	// �ֽ��ⲿ��

	//�Ի���������չתΪ�������꣬��һ���в���
	std::vector<double> vdXValue;
	vdXValue.push_back(550.0);
	int nMaxXCount = (2000.0 + 550.0) / 50.0;
	for (int i = 1; i < nMaxXCount; i++)
	{
		double dDis1 = i * 50.0 + 550.0;
		double dDis2 = -i * 50.0 + 550.0;
		if (dDis1 < 2000.0)
			vdXValue.push_back(dDis1);
		if (dDis2 > -2000.0)
			vdXValue.push_back(dDis2);
	}


	std::vector<double> vdYValue;
	vdYValue.push_back(0.0);
	for (int i = 1; i < 41; i++)
	{
		vdYValue.push_back(i * 50.0);
		vdYValue.push_back(-i * 50.0);
	}

	std::vector<T_ROBOT_COORS> vtMeasureCoordNew(vtMeasureCoord.size());
	std::vector<bool> vbIsSuccess(vtMeasureCoord.size());
	for (size_t i = 0; i < vbIsSuccess.size(); i++)
	{
		vbIsSuccess[i] = false;
	}
	for (size_t i = 0; i < vdXValue.size(); i++)
	{
		for (size_t j = 0; j < vdYValue.size(); j++)
		{
			//�����ⲿ��
			std::vector<T_ROBOT_COORS> vtTeachDataNew = vtMeasureCoord;
			for (size_t k = 0; k < vtTeachDataNew.size(); k++)
			{
				auto& tCoord = vtTeachDataNew[k];
				tCoord.dBX = tCoord.dBX + tCoord.dX - vdXValue[i];
				tCoord.dX = vdXValue[i];
				tCoord.dBY = tCoord.dBY + tCoord.dY - vdYValue[j];
				tCoord.dY = vdYValue[j];

				if (!vbIsSuccess[k]
					&& m_pRobotDriver->MoveToolByWeldGun(
						tCoord,
						m_pRobotDriver->m_tTools.tGunTool,
						tCoord,
						m_pRobotDriver->m_tTools.tCameraTool,
						tCoord))
				{
					vbIsSuccess[k] = true;
					tCoord.dX = tCoord.dX + tCoord.dBX;
					tCoord.dY = tCoord.dY + tCoord.dBY;
					tCoord.dZ = tCoord.dZ + tCoord.dBZ;
					tCoord.dBX = 0.0;
					tCoord.dBY = 0.0;
					tCoord.dBZ = 0.0;
					vtMeasureCoordNew[k] = tCoord;
				}
			}
		}
	}
	for (size_t i = 0; i < vbIsSuccess.size(); i++)
	{
		if (!vbIsSuccess[i])
		{
			return false;
		}
	}

	for (size_t i = 0; i < vdXValue.size(); i++)
	{
		for (size_t j = 0; j < vdYValue.size(); j++)
		{
			//�����ⲿ��
			bool bSuccess = true;
			std::vector<T_ROBOT_COORS> vtTeachDataNew = vtMeasureCoordNew;
			std::vector<T_ROBOT_COORS> vtWeldDataNew = vtMeasureCoordNew;
			for (size_t k = 0; k < vtTeachDataNew.size(); k++)
			{
				auto& tCoord = vtTeachDataNew[k];
				tCoord.dBX = tCoord.dBX + tCoord.dX - vdXValue[i];
				tCoord.dX = vdXValue[i];
				tCoord.dBY = tCoord.dBY + tCoord.dY - vdYValue[j];
				tCoord.dY = vdYValue[j];

				if (tCoord.dBX < -5000.0
					|| tCoord.dBX > 5000.0
					|| tCoord.dBY < -1100.0
					|| tCoord.dBY > 1100.0)
				{
					bSuccess = false;
				}
			}
			if (!bSuccess)
				continue;

			// �������ǹ���ɵ�
			AddSafeCoord(vtTeachDataNew, vnMeasureType, tWeldLine.ZSide + 120);			// �������ǹ���ɵ�
			vtTeachDataNew[0].dZ = vtTeachDataNew[1].dZ;
			vtTeachDataNew.back().dZ = vtTeachDataNew[vtTeachDataNew.size() - 2].dZ;

			// �����������й켣
			if (!CalcContinuePulseForWeld(vtTeachDataNew, vtMeasurePulse, true))
			{
				continue;
			}

			vtMeasureCoord = vtTeachDataNew;

			m_conRz = vtMeasureCoord[0].dRZ;

			dExAxlePos = vtMeasureCoord[0].dBX;
			m_dTeachExAxlePos = vtMeasureCoord[0].dBX;
			m_dTeachExAxlePos_y = vtMeasureCoord[0].dBY;

			// ��ʼ���켣������
			vnMeasureType.resize(vtMeasureCoord.size(), E_WELD_TRACK);
			return true;
		}
	}
	return false;

	double ScanX = (m_nRobotInstallDir == -1) ? 800.0 : 0.0;
	double ScanY = 0.0;
	for (auto& coor : vtMeasureCoord)
	{
//		double bx, by;
		if (-1 != m_ptUnit->m_nMeasureAxisNo_up)
		{
			adjustCoordinate(coor.dBY, coor.dY, -1450.0, 450.0, ScanY);
			adjustCoordinate(coor.dBX, coor.dX, -4000.0, 3300.0, ScanX);
		}
		else
		{
			if (1 == m_ptUnit->m_nMeasureAxisNo) 
			{
				adjustCoordinate(coor.dBX, coor.dX, -4000.0, 3300.0, ScanX);
			}
			else if (2 == m_ptUnit->m_nMeasureAxisNo) 
			{
				adjustCoordinate(coor.dBY, coor.dY, -2400.0, 2400.0, ScanY);
			}
		}
	}

	bool isCon = false;
	int segmentNum = 20;
	int stepNum = vtMeasureCoord.size() / segmentNum;
	vector<T_ROBOT_COORS> tt = vtMeasureCoord;
	for(int nOffsetIdx = 0;!isCon && nOffsetIdx < stepNum; nOffsetIdx++)
	{

		// �����㲻����λ �������
		if (m_pRobotDriver->CompareXY(vtContour.at(0), vtContour.at(vtContour.size() - 1)))
		{
			vector<T_ROBOT_COORS> tmprealCoord;
			vector<T_ROBOT_COORS> tmpCoord = tt;
			int zeroRz = nOffsetIdx * segmentNum;
			double minRZdiff = 9999;

			tmprealCoord.assign(tmpCoord.begin() + zeroRz, tmpCoord.end());
			tmprealCoord.insert(tmprealCoord.end(), tmpCoord.begin(), tmpCoord.begin() + zeroRz);
			vtMeasureCoord = tmprealCoord;
		}
		if (m_iscircle == 1)
		{
			vtMeasureCoord.push_back(vtMeasureCoord[0]);
			vtMeasureCoord.push_back(vtMeasureCoord[1]);
			vtMeasureCoord.push_back(vtMeasureCoord[2]);
			vtMeasureCoord.push_back(vtMeasureCoord[3]);
			vtMeasureCoord.push_back(vtMeasureCoord[4]);
		}
		m_vtContour = vtMeasureCoord;

		SaveMeasureTrackGunTool(nGroupNo, vtMeasureCoord);

		// ��ʼ���켣������
		vnMeasureType.resize(vtMeasureCoord.size(), E_WELD_TRACK);

		// �������ǹ���ɵ�
		AddSafeCoord(vtMeasureCoord, vnMeasureType, tWeldLine.ZSide);			// �������ǹ���ɵ�

		// �����������й켣
		if (!CalcContinuePulseForWeld(vtMeasureCoord, vtMeasurePulse, true))
		{
			continue;
		}

		isCon = true;
		m_conRz = vtMeasureCoord[0].dRZ;

		dExAxlePos = vtMeasureCoord[0].dBX;
		m_dTeachExAxlePos = vtMeasureCoord[0].dBX;
		m_dTeachExAxlePos_y = vtMeasureCoord[0].dBY;

		// ��������켣�����������
		SaveMeasureTrackCamTool(nGroupNo, vtMeasureCoord, vtMeasurePulse, vnMeasureType);
	}

	if (false == isCon)
	{
		XiMessageBox("��λ�ü������������켣ʧ�ܣ�");
		return false;
	}

	// ���ߵ���ʹ��
	CString sJobName;
	sJobName.Format("MOVJTEACHBOARD%d", nGroupNo);
	GenerateJobLocalVariable(vtMeasurePulse, MOVJ, sJobName, 1);

	return true;
}

void ScanWeldLine::adjustCoordinate(double& coord, double& axisCoord, double lowerLimit, double upperLimit, double scanValue) {
	coord = axisCoord - scanValue;
	axisCoord = scanValue;

	if (coord > upperLimit || coord < lowerLimit) {
		axisCoord = axisCoord + coord;
		if (coord > 0) {
			coord = upperLimit;
			axisCoord = axisCoord - coord;
		}
		else if (coord < 0) {
			coord = lowerLimit;
			axisCoord = axisCoord - coord;
		}
	}
}

void ScanWeldLine::CurveScanSide(WeldLineInfo tWeldInfo, double dExAxlePos, double dExAxlePos_y, std::vector<T_ROBOT_COORS>& vtMeasureCoord, double pExLinepExLine, double angle)
{
	XiAlgorithm alg;
	T_ROBOT_COORS tTempCoors;
	T_ROBOT_COORS tStartTempCoors;
	T_ROBOT_COORS tEndTempCoors;
	//double pExLine = 60.0;

	//lambda����������ƫ��
	auto updateCoord = [&](const auto& point, const auto& normalVector, double angleOffset)
		{
			double dDirAngle = alg.CalcArcAngle(normalVector.x, normalVector.y);
			double rz = DirAngleToRz(dDirAngle);

			tTempCoors.dX = point.x + pExLinepExLine * CosD(rz + angleOffset);
			tTempCoors.dY = point.y + pExLinepExLine * SinD(rz + angleOffset);
			tTempCoors.dZ = point.z;

			tTempCoors.dBX = dExAxlePos;
			tTempCoors.dBY = dExAxlePos_y;
			tTempCoors.dBZ = 0.0;

			return tTempCoors;
		};

	int eraseCount = static_cast<int>(pExLinepExLine / 8);

	vector<T_ROBOT_COORS> vtTempCoors;

	if (0 != tWeldInfo.tWeldLine.StartPointType)
	{
		CvPoint3D64f StartPoint;
		StartPoint.x = vtMeasureCoord[0].dX;
		StartPoint.y = vtMeasureCoord[0].dY;
		StartPoint.z = vtMeasureCoord[0].dZ;

		CvPoint3D64f StartNormalVector;
		StartNormalVector.x = vtMeasureCoord[0].dRX;
		StartNormalVector.y = vtMeasureCoord[0].dRY;
		StartNormalVector.z = vtMeasureCoord[0].dRZ;

		//StartNormalVector.x = tWeldInfo.tWeldLine.EndNormalVector.x;
		//StartNormalVector.y = tWeldInfo.tWeldLine.EndNormalVector.y;
		//StartNormalVector.z = tWeldInfo.tWeldLine.EndNormalVector.z;

		tStartTempCoors = updateCoord(StartPoint, StartNormalVector, - m_nRobotInstallDir * (angle+20));

		//CalcCorvrAngle(true, false, true, tStartTempCoors, 60);
	}

	if (0 != tWeldInfo.tWeldLine.EndPointType)
	{
		CvPoint3D64f EndPoint;
		EndPoint.x = vtMeasureCoord.back().dX;
		EndPoint.y = vtMeasureCoord.back().dY;
		EndPoint.z = vtMeasureCoord.back().dZ;

		CvPoint3D64f EndNormalVector;
		EndNormalVector.x = vtMeasureCoord.back().dRX;
		EndNormalVector.y = vtMeasureCoord.back().dRY;
		EndNormalVector.z = vtMeasureCoord.back().dRZ;

		//EndNormalVector.x = vtMeasureCoord.back().dRX;
		//EndNormalVector.y = vtMeasureCoord.back().dRY;
		//EndNormalVector.z = vtMeasureCoord.back().dRZ;

		tEndTempCoors = updateCoord(EndPoint, EndNormalVector, angle-90);

		//CalcCorvrAngle(false, false, true, tEndTempCoors, 60);
	}

	if (0 != tWeldInfo.tWeldLine.EndPointType && 0 == tWeldInfo.tWeldLine.StartPointType)
	{
		CvPoint3D64f StartPoint;
		StartPoint.x = vtMeasureCoord[0].dX;
		StartPoint.y = vtMeasureCoord[0].dY;
		StartPoint.z = vtMeasureCoord[0].dZ;

		CvPoint3D64f StartNormalVector;
		StartNormalVector.x = vtMeasureCoord[0].dRX;
		StartNormalVector.y = vtMeasureCoord[0].dRY;
		StartNormalVector.z = vtMeasureCoord[0].dRZ;

		//StartNormalVector.x = tWeldInfo.tWeldLine.EndNormalVector.x;
		//StartNormalVector.y = tWeldInfo.tWeldLine.EndNormalVector.y;
		//StartNormalVector.z = tWeldInfo.tWeldLine.EndNormalVector.z;

		tStartTempCoors = updateCoord(StartPoint, StartNormalVector, -m_nRobotInstallDir * (angle + 20));
	}
	if (0 != tWeldInfo.tWeldLine.StartPointType)
	{
		vtMeasureCoord.insert(vtMeasureCoord.begin(), tStartTempCoors);
	}
	if (0 != tWeldInfo.tWeldLine.EndPointType && 0 == tWeldInfo.tWeldLine.StartPointType)
	{
		vtMeasureCoord.insert(vtMeasureCoord.begin(), tStartTempCoors);
	}
	if (0 != tWeldInfo.tWeldLine.EndPointType)
	{
		vtMeasureCoord.push_back(tEndTempCoors);
	}

}

void ScanWeldLine::CalcCorvrAngle(bool bBelongStart, bool bBelongPtnFree, bool bOtherPtnFree, T_ROBOT_COORS& tCoord, double dSeamLen/* = 0*/)
{
	// ������������㸽�������յ㸽�� �� 
	// ����������������(true:����  false:����)
	// ����������������ĵ�����(true:����  false:����)
	// ���쳤�� Ĭ��0��ʾ �����㲻����ĳ������ �Ǹ��ݸ���˵�������ĺ���

	// ˫���ɶ� ������̬
	// ���������� ���ȴ� ������̬
	// ���������� �� ��һ��˵���� �� ����С �� !bBelongStart �������̬
	// ��������� ��bBelongStart �������̬
	if (bBelongPtnFree && (!bOtherPtnFree) && (dSeamLen < m_dShortSeamThreshold))
	{
		bBelongStart = !bBelongStart;
	}
	//double dChangeAngle = bBelongStart ? m_dGunLaserAngle : m_dGunCameraAngle; // ��㸽���ڵ����� �յ㸽���ڵ����
	double dChangeAngle = bBelongStart ? m_dGunCameraAngle : m_dGunLaserAngle; // ��㸽���ڵ����� �յ㸽���ڵ����
	double dChangeDir = bBelongStart ? m_dRotateToCamRxDir : -m_dRotateToCamRxDir; // ������ת����� �����ת�򼤹�
	tCoord.dRX += (dChangeAngle * dChangeDir);
	double dRZAdjust = 0.0;
	//if (m_pRobotDriver->m_nRobotNo > 1)
	{
		//dRZAdjust = 10.0;
	}
	//tCoord.dRZ += ((dChangeAngle+ dRZAdjust) * (dChangeDir));// ���Ϲ����ݺύ������Rzת��
}


// ���ɹ켣�ĺ���  
vector<T_ROBOT_COORS> ScanWeldLine::generateTrajectory(double startX, double startY, double startZ, double endX, double endY, double endZ)
{
	std::vector<T_ROBOT_COORS> trajectory;

	// ����ÿ��ά�ȵĲ���  
	double dx = (endX - startX) / std::max(1.0, std::fabs(endX - startX));
	double dy = (endY - startY) / std::max(1.0, std::fabs(endY - startY));
	double dz = (endZ - startZ) / std::max(1.0, std::fabs(endZ - startZ));

	// ���ڼ����1mm�������ܲ�������������������Ҫѭ��ֱ���ӽ��յ�  
	double currentX = startX;
	double currentY = startY;
	double currentZ = startZ;

	// ��ֹ����ѭ���������ϲ��ᣬ������˼�����ȫ��  
	int maxIterations = 1000000;
	int iterations = 0;



	while (std::fabs(currentX - endX) > 0.0005 || std::fabs(currentY - endY) > 0.0005 || std::fabs(currentZ - endZ) > 0.0005 && iterations < maxIterations)
	{
		T_ROBOT_COORS tmpPont;

		tmpPont.dX = currentX;
		tmpPont.dY = currentY;
		tmpPont.dZ = currentZ;

		trajectory.push_back(tmpPont);

		// ���µ�ǰλ��  
		currentX += dx;
		currentY += dy;
		currentZ += dz;

		// ���ӵ�������  
		iterations++;
	}

	// ����յ㣨��Ϊ��������������һ����ܲ���ȫ�����յ㣩  
	T_ROBOT_COORS tmpPont;

	tmpPont.dX = endX;
	tmpPont.dY = endY;
	tmpPont.dZ = endZ;

	trajectory.push_back(tmpPont);

	return trajectory;
}

bool ScanWeldLine::DoTeach(int nGroupNo, const std::vector<T_ANGLE_PULSE>& vtMeasurePulse, const vector<int>& vnMeasureType, double dExAxlePos, int nLayerNo/* = 0*/)
{
	if (GetLocalDebugMark())
	{
		return true;
	}

	BackHome();
	int nNO=0;
	std::vector<T_ROBOT_COORS>vtFileCoord;
	std::vector<T_ANGLE_PULSE>vtPul(0);
	std::vector<int>vnPul(0);
	std::vector<int>vnRealType(0);
	std::vector<T_ANGLE_PULSE>vtRealPulse(0);
	vnRealType = vnMeasureType;
	vtRealPulse = vtMeasurePulse;
	//reverse(vnRealType.begin(), vnRealType.end());
	//reverse(vtRealPulse.begin(), vtRealPulse.end());
	/*std::vector<T_ANGLE_PULSE>vtRealPulse;
	std::vector<int>vnRealType;
	vtRealPulse.insert(vtRealPulse.end(), vtPul.begin(), vtPul.end());
	reverse(vtPul.begin(), vtPul.end());
	vtRealPulse.insert(vtRealPulse.end(), vtPul.begin(), vtPul.end());
	
	vnRealType.insert(vnRealType.end(), vnPul.begin(), vnPul.end());
	reverse(vnPul.begin(), vnPul.end());
	vnRealType.insert(vnRealType.end(), vnPul.begin(), vnPul.end());*/

	isMeasureMove = false;
	double dExAxlePos_y = m_dTeachExAxlePos_y;
	//double dExAxlePos_y = vtMeasurePulse[0].lBYPulse * m_pRobotDriver->m_tExternalAxle[1].dPulse;
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

	// �˶��ⲿ�ᵽָ��λ��
	int nAxisNo = m_ptUnit->m_nMeasureAxisNo;
	CHECK_BOOL_RETURN(MoveExAxleToDst(1 == nAxisNo ? dExAxlePos: dExAxlePos_y, nAxisNo, m_pRobotDriver->m_tExAxlePulseSpeed.dSpeed));

	//// ��ȡ��ǹ�켣 �˶� MOVJ MOVL
	T_ROBOT_MOVE_INFO tRobotMoveInfo;
	vector<T_ROBOT_MOVE_INFO> vtRobotMoveInfo(0);
	tRobotMoveInfo = m_pRobotDriver->PVarToRobotMoveInfo(0, vtRealPulse[0], m_pRobotDriver->m_tPulseHighSpeed, MOVJ, 1);
	vtRobotMoveInfo.push_back(tRobotMoveInfo);
	tRobotMoveInfo = m_pRobotDriver->PVarToRobotMoveInfo(1, vtRealPulse[0], m_pRobotDriver->m_tPulseHighSpeed, MOVJ, 2);
	vtRobotMoveInfo.push_back(tRobotMoveInfo);
	tRobotMoveInfo = m_pRobotDriver->PVarToRobotMoveInfo(2, vtRealPulse[0], m_pRobotDriver->m_tCoordLowSpeed, MOVL, 2);
	vtRobotMoveInfo.push_back(tRobotMoveInfo);
	m_pRobotDriver->SetMoveValue(vtRobotMoveInfo);
	m_pRobotDriver->CallJob("CONTIMOVANY");
	E_DHGIGE_ACQUISITION_MODE eCameraMode = E_ACQUISITION_MODE_SOURCE_SOFTWARE;
	E_DHGIGE_CALL_BACK eCallBack = E_CALL_BACK_MODE_OFF;
	m_ptUnit->SwitchDHCamera(m_ptUnit->m_nMeasureCameraNo, true, true, eCameraMode, eCallBack); // ��ǹʱ ����� ����
	m_ptUnit->m_vpImageCapture[m_ptUnit->m_nMeasureCameraNo]->StartAcquisition();
	//m_ptUnit->SwitchDHCamera(m_ptUnit->m_nTrackCameraNo, true, true, eCameraMode, eCallBack); // ��ǹʱ ����� ����
	//m_ptUnit->m_vpImageCapture[m_ptUnit->m_nTrackCameraNo]->StartAcquisition();
	m_ptUnit->RobotCheckDone();
	CHECK_PULSE_RETURN_BOOL(m_pRobotDriver, vtRealPulse[0]);
	m_pRobotDriver->m_nScanNum = 1;
	for (int i = 1;i <= m_nScanNum;i++)
		// ����ɨ��ɼ�����
	{
		AfxBeginThread(ThreadMeasure, this); // �ɼ���������

		// ��ȡɨ��켣 ɨ��
		E_WELD_SEAM_TYPE eSeamType = E_FLAT_SEAM; // ƽ��
		T_ROBOT_COORS tCoord;
		std::vector<T_ROBOT_COORS> vtCoord(0);
		std::vector<T_ROBOT_COORS> vtRealCoord(0);
		std::vector<int> vnType(0);
		std::vector<T_WELD_PARA> vtWeldPara;
		GetCurWeldParam(eSeamType, vtWeldPara);
		vtWeldPara[0].WeldVelocity = m_dScanSpeed; // �����˶�ɨ���ٶ�
		vtWeldPara[0].nWrapConditionNo = 0; // ɨ���˶�ʹ�ú����˶� ȥ���ڶ�
		for (int i = 0; i < vnRealType.size(); i++)
		{
			if (E_WELD_TRACK == vnRealType[i])
			{
				m_pRobotDriver->RobotKinematics(vtRealPulse[i], m_pRobotDriver->m_tTools.tGunTool, tCoord);
				if (m_pRobotDriver->m_nScanNum == 1)
				{
					/*tCoord.dRZ=10;*/
					
				}
				else if(m_pRobotDriver->m_nScanNum == 2)
				{
					tCoord.dRZ += 45;
				}
				vtCoord.push_back(tCoord);
				vnType.push_back(vnRealType[i]);
				//vnType.push_back(vnMeasureType[i]);
			}
		}
		std::vector<T_ROBOT_COORS>vtRobotCoor;
		std::vector<int>vnRobotCoor;
		//if (m_bIsCake == 0)
		//{
		//	for (int i = 0;i < 50;i++)
		//	{
		//		tCoord = vtCoord[0];
		//		tCoord.dRZ += i * 1;
		//		tCoord.dX += SinD(vtCoord[0].dRZ)*i;
		//		tCoord.dY -=2*i;
		//		vtRobotCoor.push_back(tCoord);
		//		vnRobotCoor.push_back(i);
		//	}
		//	reverse(vtRobotCoor.begin(), vtRobotCoor.end());
		//	vtCoord.insert(vtCoord.begin(), vtRobotCoor.begin(), vtRobotCoor.end());
		//	vnType.insert(vnType.end(), vnRobotCoor.begin(), vnRobotCoor.end());
		//	reverse(vtRobotCoor.begin(), vtRobotCoor.end());
		//	vtCoord.insert(vtCoord.begin(), vtRobotCoor.begin(), vtRobotCoor.end());
		//	vnType.insert(vnType.end(), vnRobotCoor.begin(), vnRobotCoor.end());
		//}
	
		m_ptUnit->WeldMove_Scan(vtCoord, vnType, vtWeldPara[0], dExAxlePos, eSeamType, false, 2);
		m_ptUnit->RobotCheckDone();
		if (m_pRobotDriver->m_nScanNum == 1)
		{
			reverse(vtRealPulse.begin(), vtRealPulse.end());
			reverse(vnRealType.begin(), vnRealType.end());
		}
		CString strFile;
		strFile.Format("%s%s%s\\%d_ScanWeldLinePointCloud%d.txt", OUTPUT_PATH,
			m_ptUnit->GetRobotCtrl()->m_strRobotName, RECOGNITION_FOLDER, 0, m_pRobotDriver->m_nScanNum);
		FILE* pf = fopen(strFile.GetBuffer(), "r");
		while (EOF != fscanf(pf, "%d%lf%lf%lf", &nNO,
			&tCoord.dX, &tCoord.dY, &tCoord.dZ))
		{
			vtFileCoord.push_back(tCoord);
		}
		fclose(pf);
		m_pRobotDriver->m_nScanNum++;
	}
	CString str;
	str.Format("%s%s%s\\%d_ScanWeldLinePointCloud.txt", OUTPUT_PATH,
		m_ptUnit->GetRobotCtrl()->m_strRobotName, RECOGNITION_FOLDER, 0);
	FILE* pf1 = fopen(str.GetBuffer(), "w");
	for (int i = 0;i < vtFileCoord.size();i++)
	{
		fprintf(pf1, "%d%11.3lf%11.3lf%11.3lf\n", nNO, vtFileCoord[i].dX, vtFileCoord[i].dY, vtFileCoord[i].dZ);
	}
	fclose(pf1);
	// ��ȡ��ǹ�켣 �˶� MOVL MOVJ
	vtRobotMoveInfo.clear();
	WriteLog("��ǹ��movl:%d %d %d %d %d %d", 
		vtRealPulse[vtRealPulse.size() - 1].nSPulse, 
		vtRealPulse[vtRealPulse.size() - 1].nLPulse,
		vtRealPulse[vtRealPulse.size() - 1].nUPulse,
		vtRealPulse[vtRealPulse.size() - 1].nRPulse,
		vtRealPulse[vtRealPulse.size() - 1].nBPulse,
		vtRealPulse[vtRealPulse.size() - 1].nTPulse);
	WriteLog("��ǹ��movj:%d %d %d %d %d %d",
		vtRealPulse.back().nSPulse,
		vtRealPulse.back().nLPulse,
		vtRealPulse.back().nUPulse,
		vtRealPulse.back().nRPulse,
		vtRealPulse.back().nBPulse,
		vtRealPulse.back().nTPulse);
	/*tRobotMoveInfo = m_pRobotDriver->PVarToRobotMoveInfo(0, vtRealPulse[vtRealPulse.size() - 1], m_pRobotDriver->m_tCoordLowSpeed, MOVL, 2);
	vtRobotMoveInfo.push_back(tRobotMoveInfo);
	tRobotMoveInfo = m_pRobotDriver->PVarToRobotMoveInfo(1, vtRealPulse.back(), m_pRobotDriver->m_tPulseHighSpeed, MOVJ, 2);
	vtRobotMoveInfo.push_back(tRobotMoveInfo);
	tRobotMoveInfo = m_pRobotDriver->PVarToRobotMoveInfo(2, vtRealPulse.back(), m_pRobotDriver->m_tPulseHighSpeed, MOVJ, 1);
	vtRobotMoveInfo.push_back(tRobotMoveInfo);
	m_pRobotDriver->SetMoveValue(vtRobotMoveInfo);
	m_pRobotDriver->CallJob("CONTIMOVANY");*/
	m_ptUnit->SwitchDHCamera(m_ptUnit->m_nMeasureCameraNo, false); // ����� ����
	//m_ptUnit->SwitchDHCamera(m_ptUnit->m_nTrackCameraNo, false); // ����� ����
	m_ptUnit->RobotCheckDone();
	//CHECK_PULSE_RETURN_BOOL(m_pRobotDriver, vtRealPulse.back());

	// ��ȡɨ�躸�������� m_pTraceModel->vtConnerWorldCoords;
	//CHECK_BOOL_RETURN(GetScanWeldLineResult());

	//BackHome();

	// ����ʾ�̽��
	SaveTeachResult(nGroupNo);
	return true;
}

bool ScanWeldLine::CalcWeldTrack(int nGroupNo)
{
	if (!m_ptUnit->m_bBreakPointContinue)
	{
		// ɾ���� nGroupNo �麸��켣�ļ� 
		int nMaxWeldNo = 8;
		int nMaxLayerNo = 6;
		CString sFileName;
		for (int nDelWeldNo = 0; nDelWeldNo < nMaxWeldNo; nDelWeldNo++)
		{
			sFileName.Format("%d_%d_RealWeldCoord.txt", nGroupNo, nDelWeldNo);
			DelFiles(m_sDataSavePath, sFileName);
			for (int nLayerNo = 0; nLayerNo < nMaxLayerNo; nLayerNo++)
			{
				sFileName.Format("%d_%d_%d_AdjustRealWeldCoord.txt", nGroupNo, nDelWeldNo, nLayerNo);
				DelFiles(m_sDataSavePath, sFileName);
			}
		}

		//XiAlgorithm alg;
		//std::vector<int> vnPointType(0);
		std::vector<T_ROBOT_COORS> vtWeldCoord(0);
		std::vector<T_ROBOT_COORS> tmpcoor(0);
		std::vector<T_ROBOT_COORS> vtRealWeldCoord(0);

		if (GetLocalDebugMark()) // ����ʹ����ɨ���ƴ������������켣
		{
			WeldLineInfo tWeldInfo = m_vvtWeldLineInfoGroup[nGroupNo][0];
			int nWeldIndex = tWeldInfo.tAtrribute.nWeldSeamIdx;
			CString sContourFile = OUTPUT_PATH + m_ptUnit->m_tContralUnit.strUnitName + "\\" + IDENTIFY_RESULT_SAVE_JSON;
			//sContourFile.Format("%s00_PointCloudRecoTrack%d.txt", OUTPUT_PATH + m_ptUnit->m_tContralUnit.strUnitName + RECOGNITION_FOLDER, nWeldIndex);
			// ��ȡ�����ļ� �˲� ����ֱ������
			//CHECK_BOOL_RETURN(LoadContour(nWeldIndex, sContourFile, vtWeldCoord));
			CHECK_BOOL_RETURN(LoadContourByJson(nWeldIndex, nGroupNo, sContourFile, vtWeldCoord));
		}
		else
		{
			PartPointCloudProcess(vtWeldCoord, nGroupNo);
		}

		if (vtWeldCoord.size() == 0)
		{
			XiMessageBox("���ӹ켣����ʧ��");
			return false;
		}

		if (m_iscircle == 1)
		{
			if (!m_pRobotDriver->CompareXY(vtWeldCoord.at(0), vtWeldCoord.at(vtWeldCoord.size() - 1), 5.0))
			{
				XiMessageBox("�պ�Բ���켣�Ͽ���������ʶ���������");
				return false;
			}
		}


		// �ֽ��ⲿ������
		for (int i = 0; i < vtWeldCoord.size(); i++)
		{
			if (-1 != m_ptUnit->m_nMeasureAxisNo_up)
			{
				WeldAfterMeasure::DecomposeExAxle(vtWeldCoord[i], m_dTeachExAxlePos, m_ptUnit->m_nTrackAxisNo);
				WeldAfterMeasure::DecomposeExAxle(vtWeldCoord[i], m_dTeachExAxlePos_y, m_ptUnit->m_nMeasureAxisNo_up);
			}
			else
			{
#ifdef SINGLE_ROBOT
				WeldAfterMeasure::DecomposeExAxle(vtWeldCoord[i], m_dTeachExAxlePos_y, m_ptUnit->m_nTrackAxisNo);
#else
				WeldAfterMeasure::DecomposeExAxle(vtWeldCoord[i], m_dTeachExAxlePos, m_ptUnit->m_nTrackAxisNo);
#endif
			}
		}

		// �˲��������̬
		if (m_nScanTrackingWeldEnable)
		{
			CHECK_BOOL_RETURN(TrackFilter(vtWeldCoord, vtRealWeldCoord, 1, 30)); // ���ӹ켣���3mm 3x10mm���ȹ켣ȷ��Rz
		}
		else
		{
			CHECK_BOOL_RETURN(TrackFilter(vtWeldCoord, vtRealWeldCoord, 3, 10)); // ���ӹ켣���3mm 3x10mm���ȹ켣ȷ��Rz
		}

		bool isCon = false;
		int stepNum = vtRealWeldCoord.size() / 20 - 100.0;
		stepNum = stepNum < 50 ? 50 : stepNum;
		vector<T_ROBOT_COORS> tt = vtRealWeldCoord;
		vector<T_ANGLE_PULSE> vtMeasurePulse;
		for (int nOffsetIdx = 0; !isCon && nOffsetIdx < stepNum; nOffsetIdx++)
		{

			// �����㲻����λ �������
			if (m_pRobotDriver->CompareXY(vtWeldCoord.at(0), vtWeldCoord.at(vtWeldCoord.size() - 1)))
			{
				vector<T_ROBOT_COORS> tmprealCoord;
				vector<T_ROBOT_COORS> tmpCoord = tt;
				int zeroRz = stepNum;
				double minRZdiff = 9999;

				tmprealCoord.assign(tmpCoord.begin() + zeroRz, tmpCoord.end());
				tmprealCoord.insert(tmprealCoord.end(), tmpCoord.begin(), tmpCoord.begin() + zeroRz);
				vtRealWeldCoord = tmprealCoord;
			}
			// �����������й켣
			if (!CalcContinuePulseForWeld(vtRealWeldCoord, vtMeasurePulse, true))
			{
				continue;
			}

			/*if (m_iscircle == 1)
			{
				vtRealWeldCoord.push_back(vtRealWeldCoord[0]);
				vtRealWeldCoord.push_back(vtRealWeldCoord[1]);
				vtRealWeldCoord.push_back(vtRealWeldCoord[2]);
				vtRealWeldCoord.push_back(vtRealWeldCoord[3]);
				vtRealWeldCoord.push_back(vtRealWeldCoord[4]);
			}*/

			isCon = true;
			m_dTeachExAxlePos = vtRealWeldCoord[0].dBX;
			m_dTeachExAxlePos_y = vtRealWeldCoord[0].dBY;
		}

		double dWeldHoleDisS = 0; // dWeldHoleSize; // �����׳ߴ�
		double dWeldHoleDisE = 0; // dWeldHoleSize; // �����׳ߴ�
		double dChangeDisS = 45.0;	// ����̬����
		double dChangeDisE = 45.0;	// ����̬����
		if (m_bIsCake == 0)
		{
			dChangeDisS = 80.0;	// ����̬����
			dChangeDisE = 80.0;	// ����̬����
			dWeldHoleDisS = m_nWeldHole;
			dWeldHoleDisE = m_nWeldHole;
		}
		double dChangeAngle = 30.0; // ����̬�Ƕ�
		double dPtnInterval = 2.0;  // �켣����
		// ��ȡ�������� ���ɺ��ӹ켣;
		double dTempVar = dChangeDisS;

		XiAlgorithm alg;

		int nChangePtnNum = (int)(dChangeDisS / dPtnInterval); // ����̬����
		int nDelPtnNum = (int)(dWeldHoleDisS / dPtnInterval); // ������ɾ������
		double dStepChangeAngle = (double)m_nRobotInstallDir * dChangeAngle / (double)nChangePtnNum; // ���ڵ���̬�仯�Ƕ�


		int nChangePtnNumS = (int)(dChangeDisS / dPtnInterval); // ����̬����
		int nChangePtnNumE = (int)(dChangeDisE / dPtnInterval); // ����̬����
		int nDelPtnNumS = (int)(dWeldHoleDisS / dPtnInterval); // ������ɾ������
		int nDelPtnNumE = (int)(dWeldHoleDisE / dPtnInterval); // ������ɾ������
		double dStepChangeAngleS = (double)m_nRobotInstallDir * dChangeAngle / (double)nChangePtnNumS; // ���ڵ���̬�仯�Ƕ�
		double dStepChangeAngleE = (double)m_nRobotInstallDir * dChangeAngle / (double)nChangePtnNumE; // ���ڵ���̬�仯�Ƕ�


		int nWeldTrackPtnNum = vtRealWeldCoord.size();
		if (nWeldTrackPtnNum < 10 ||
			nWeldTrackPtnNum < nChangePtnNumS ||
			nWeldTrackPtnNum < nChangePtnNumE)
		{
			//���޸�
			XUI::MesBox::PopInfo("����켣����{0}���� �޷����ӣ�", nWeldTrackPtnNum);
			//XiMessageBox("����켣����%d���� �޷����ӣ�", nWeldTrackPtnNum);
			return false;
		}

		int nBaseIndex = nWeldTrackPtnNum - 1 - nChangePtnNumE;
		double dSrcRz;
		dSrcRz = vtRealWeldCoord[nBaseIndex].dRZ;
		for (int n = nWeldTrackPtnNum - nChangePtnNumE; n < nWeldTrackPtnNum; n++)
		{
			vtRealWeldCoord[n].dRZ = dSrcRz - ((n - nBaseIndex) * dStepChangeAngleE); // ��̬��С(���������Ұ�װ��ʽ���)(֧������ຸ��)
		}

		dSrcRz = vtRealWeldCoord[nChangePtnNumS].dRZ;
		for (int n = nChangePtnNumS - 1; n >= 0; n--)
		{
			vtRealWeldCoord[n].dRZ = dSrcRz + ((nChangePtnNumS - n) * dStepChangeAngleS); // ��̬����(���������Ұ�װ��ʽ���)(֧������ຸ��)
		}
		vtRealWeldCoord.erase(vtRealWeldCoord.begin(), vtRealWeldCoord.begin() + nDelPtnNumS); // ɾ������

		/*if (!m_pRobotDriver->CompareXY(vtWeldCoord.at(0), vtWeldCoord.at(vtWeldCoord.size() - 1)) && 0 == m_LiRon)
		{
			vtRealWeldCoord.erase(vtRealWeldCoord.begin(), vtRealWeldCoord.begin() + 10);
			vtRealWeldCoord.erase(vtRealWeldCoord.end() - 10, vtRealWeldCoord.end());
		}*/
		
	//  �պϽṹ������
		if (m_bIsCake == 1
			&& m_pRobotDriver->CompareXY(vtWeldCoord.at(0), vtWeldCoord.at(vtWeldCoord.size() - 1)))
		{
			T_ROBOT_COORS tBuffCoor;
			tBuffCoor = vtRealWeldCoord[10];
			tBuffCoor.dRZ = vtRealWeldCoord.back().dRZ;
			vtRealWeldCoord.push_back(tBuffCoor);
		}
		// ���溸�ӹ켣�ļ� ��� ֱ������ �ⲿ������ ��������
		int nWeldNo = 0;
		E_WELD_SEAM_TYPE eWeldSeamType = E_FLAT_SEAM;
		sFileName.Format("%s%d_%d_RealWeldCoord.txt", m_sDataSavePath, nGroupNo, nWeldNo);
		FILE* pf = fopen(sFileName, "w");
		for (int nPtnIdx = 0; nPtnIdx < vtRealWeldCoord.size(); nPtnIdx++)
		{
			T_ROBOT_COORS& tCoord = vtRealWeldCoord[nPtnIdx];
			fprintf(pf, "%d%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%4d%10d\n", nPtnIdx,
				tCoord.dX, tCoord.dY, tCoord.dZ, tCoord.dRX, tCoord.dRY, tCoord.dRZ, m_dTeachExAxlePos, m_dTeachExAxlePos_y, eWeldSeamType, E_WELD_TRACK);
		}
		fclose(pf);
	}
	if (m_ptUnit->m_bBreakPointContinue)
	{
		T_ROBOT_COORS tPauseCoord;
		m_pRobotDriver->RobotKinematics(m_tPausePulse, m_pRobotDriver->m_tTools.tGunTool, tPauseCoord);
		m_dTeachExAxlePos = tPauseCoord.dBX;
		m_dTeachExAxlePos_y = tPauseCoord.dBY;

		m_dPauseExAxlePos = tPauseCoord.dBX;
		m_dPauseExAxlePos_y = tPauseCoord.dBY;
	}

	return true;
}

bool ScanWeldLine::TrackFilter_EX(const std::vector<T_ROBOT_COORS>& vtSrcCoord, std::vector<T_ROBOT_COORS>& vtRstCoord, int nFilterOutInterval/* = 8*/, int nCalcPosturePtnStep/* = 8*/)
{
	vtRstCoord.clear();
	std::vector<T_ROBOT_COORS> vtTempCoord(0);
	T_ROBOT_COORS tTempCoors;
	TrackSmooth_PntArray tTrackPtnArray;
	tTrackPtnArray.arraySize_ = 0;
	tTrackPtnArray.pnts_ = NULL;
	// ��ʼ����β���˲�����
	TrackSmooth_Init(m_ptUnit->m_nRobotSmooth, nFilterOutInterval, vtSrcCoord[0].dBX, vtSrcCoord[0].dBY, vtSrcCoord[0].dZ); // �˲�����3mm

	for (int i = 1; i < vtSrcCoord.size(); i++)
	{
		TrackSmooth_PushNextPoint(m_ptUnit->m_nRobotSmooth, vtSrcCoord[i].dBX, vtSrcCoord[i].dBY, vtSrcCoord[i].dZ, &tTrackPtnArray);
		for (int n = 0; n < tTrackPtnArray.arraySize_; n++)
		{
			tTempCoors = vtSrcCoord[i];
			tTempCoors.dBX = tTrackPtnArray.pnts_[n].x_;
			tTempCoors.dBY = tTrackPtnArray.pnts_[n].y_;
			tTempCoors.dZ = tTrackPtnArray.pnts_[n].z_;
			vtTempCoord.push_back(tTempCoors);
		}
		TrackSmooth_FreePntArray(&tTrackPtnArray);
	}

	//��ȡ��β�㣬����βʱ����
	TrackSmooth_GetEndPoint(m_ptUnit->m_nRobotSmooth, &tTrackPtnArray);
	for (int n = 0; n < tTrackPtnArray.arraySize_; n++)
	{
		tTempCoors = vtSrcCoord.back();
		tTempCoors.dBX = tTrackPtnArray.pnts_[n].x_;
		tTempCoors.dBY = tTrackPtnArray.pnts_[n].y_;
		tTempCoors.dZ = tTrackPtnArray.pnts_[n].z_;
		vtTempCoord.push_back(tTempCoors);
	}
	TrackSmooth_FreePntArray(&tTrackPtnArray);

	SaveCoordToFile(vtSrcCoord, "A_FilterIn.txt");
	SaveCoordToFile(vtTempCoord, "A_FilterOut.txt");

	// ������̬
	if (vtTempCoord.size() < (int)nCalcPosturePtnStep)
	{
		XiMessageBoxOk("�켣�˲�ʧ��");
		return false;
	}
	int nPtnNum = vtTempCoord.size();
	int nIdxS = 0;
	int nIdxE = 0;
	XiAlgorithm alg;
	for (int i = 0; i < nPtnNum; i++)
	{
		nIdxS = i - (nCalcPosturePtnStep / 2);
		nIdxE = i + (nCalcPosturePtnStep / 2);
		if (nIdxS < 0)
		{
			nIdxS = 0;
			nIdxE = nIdxS + nCalcPosturePtnStep;
		}
		if (nIdxE >= nPtnNum)
		{
			nIdxE = nPtnNum - 1;
			nIdxS = nIdxE - nCalcPosturePtnStep;
		}
		double dDirAngle = alg.CalcArcAngle(
			vtTempCoord[nIdxE].dX + vtTempCoord[nIdxE].dBX - vtTempCoord[nIdxS].dX - vtTempCoord[nIdxS].dBX,
			vtTempCoord[nIdxE].dY + vtTempCoord[nIdxE].dBY - vtTempCoord[nIdxS].dY - vtTempCoord[nIdxS].dBY);
		double dNorAngle = dDirAngle - (90.0 * (double)m_nRobotInstallDir);
		tTempCoors = vtTempCoord[i];
		tTempCoors.dRX = 0.0;
		tTempCoors.dRY = m_dStandWeldScanRy;
		tTempCoors.dRZ = DirAngleToRz(dNorAngle);
		vtRstCoord.push_back(tTempCoors);
	}
	SaveCoordToFile(vtRstCoord, "A_FilterResult.txt");
	return true;
}


bool ScanWeldLine::LoadContour(int nWeldIndex, CString sContourFile, std::vector<T_ROBOT_COORS>& vtContour)
{
	int nContourPtnNo = 0;
	T_ROBOT_COORS tCoord;
	FILE* pf = fopen(sContourFile, "r");
	if (NULL == pf)
	{
		//���޸�
		XUI::MesBox::PopInfo("�����ļ���{0}������!", sContourFile.GetBuffer());
		//XiMessageBoxOk("�����ļ���%s ������!", sContourFile);
		return false;
	}
	while (EOF != fscanf(pf, "%d%lf%lf%lf%lf%lf%lf", &nContourPtnNo,
		&tCoord.dX, &tCoord.dY, &tCoord.dZ, &tCoord.dRX, &tCoord.dRY, &tCoord.dRZ))
	{
		vtContour.push_back(tCoord);
	}
	fclose(pf);
	return true;
}

bool ScanWeldLine::LoadContourByJson(int nWeldIndex, int nGroupNo, CString sContourFile, std::vector<T_ROBOT_COORS>& vtContour)
{
	vtContour.clear();
	XiBase::GetSystemPath(sContourFile);

	xrc::weld::WeldDataDocJson weldDoc;
	bool bRst = xrc::weld::WeldDataDocJson::read(sContourFile.GetBuffer(), weldDoc);

	if (false == bRst)
	{
		//���޸�
		XUI::MesBox::PopInfo("���ص��ƴ������ļ�{0}��ʧ��", sContourFile.GetBuffer());
		//XiMessageBox("���ص��ƴ������ļ� %s ��ʧ��", sContourFile);
		return false;
	}

	LineOrCircularArcWeldingLine tWeld{};
	WeldLineInfo tWeldInfo;
	xrc::weld::WeldSeam tWeldSeam;
	int nWeldingWorksNum = weldDoc.weldingWorks_.size();

	//С�����������ҽ���һ������
	nWeldingWorksNum = nWeldingWorksNum > 1 ? 1 : nWeldingWorksNum;
	//CheckFolder(OUTPUT_PATH + m_pRobotDriver->m_strRobotName + RECOGNITION_FOLDER + "WeldContour");

	for (int i = 0; i < nWeldingWorksNum; i++)
	{
		for (int nWeldSeamNo = 0; nWeldSeamNo < weldDoc.weldingWorks_[i].welds_.size(); nWeldSeamNo++)
		{
			if (weldDoc.weldingWorks_[i].welds_[nWeldSeamNo].property_.index != nWeldIndex)
			{
				continue;
			}
			tWeldSeam = weldDoc.weldingWorks_[i].welds_[nWeldSeamNo];
			//tWeld.IsArc = false;// ???
			tWeld.IsArc = tWeldSeam.wire_.hasArc();
			tWeld.isClockwise = false; // ???
			tWeld.ZSide = tWeldSeam.property_.zSide;
			tWeld.isLeft = 0 != tWeldSeam.property_.isleft;

			tWeld.CenterPoint.x = 0.0;
			tWeld.CenterPoint.y = 0.0;
			tWeld.CenterPoint.z = 0.0;

			tWeld.StartPoint.x = tWeldSeam.getStartPoint().x();
			tWeld.StartPoint.y = tWeldSeam.getStartPoint().y();
			tWeld.StartPoint.z = tWeldSeam.getStartPoint().z();
			tWeld.EndPoint.x = tWeldSeam.getEndPoint().x();
			tWeld.EndPoint.y = tWeldSeam.getEndPoint().y();
			tWeld.EndPoint.z = tWeldSeam.getEndPoint().z();

			tWeld.StartNormalVector.x = tWeldSeam.getStartWeldDir().x();
			tWeld.StartNormalVector.y = tWeldSeam.getStartWeldDir().y();
			tWeld.StartNormalVector.z = tWeldSeam.getStartWeldDir().z();
			tWeld.EndNormalVector.x = tWeldSeam.getEndWeldDir().x();
			tWeld.EndNormalVector.y = tWeldSeam.getEndWeldDir().y();
			tWeld.EndNormalVector.z = tWeldSeam.getEndWeldDir().z();

			tWeld.StartPointType = tWeldSeam.property_.startPointType;
			tWeld.EndPointType = tWeldSeam.property_.endPointType;

			tWeldInfo.tAtrribute.nWeldSeamIdx = tWeldSeam.property_.index;
			tWeldInfo.tAtrribute.nIsDoubleWelding = tWeldSeam.property_.doubleWelding;
			tWeldInfo.tAtrribute.nStartWrapType = tWeldSeam.property_.startWrapCorner;
			tWeldInfo.tAtrribute.nEndWrapType = tWeldSeam.property_.endWrapCorner;
			tWeldInfo.tAtrribute.dStartHoleSize = tWeldSeam.property_.sHoleSize;
			tWeldInfo.tAtrribute.dEndHoleSize = tWeldSeam.property_.eHoleSize;
			tWeldInfo.tAtrribute.nRobotSelete = tWeldSeam.property_.robotSelect;
			tWeldInfo.tAtrribute.dThoeryLength = tWeldSeam.property_.weldingLength;

			tWeldInfo.tAtrribute.dGroupTrackPos = tWeldSeam.property_.groupTrackPos;
			tWeldInfo.tAtrribute.nGroupNo = tWeldSeam.property_.groupNo;
			tWeldInfo.tAtrribute.nBigGroupNo = tWeldSeam.property_.bigGroupNo;

			// ����Ż�ȡ����������
			int nLineSize = tWeldSeam.wire_.lineSize();
			if (false)//nLineSize > 10) // �߶�ʹ�û�ȡ��ķ�ʽ�������찴�����ȡ�������ʱ̫����
			{
				int nPtnNum = nLineSize + 1;
				//tWeldInfo.tAtrribute.vtContourPtns.clear();
				//tWeldInfo.tAtrribute.vtContourNormal.clear();
				//tWeldInfo.tAtrribute.vtContourPtns.reserve(nPtnNum);
				//tWeldInfo.tAtrribute.vtContourNormal.reserve(nPtnNum);
				for (int i = 0; i < nLineSize; i++)
				{
					xrc::weld::GeoXYZ tGeoPtn = tWeldSeam.wire_.getLine(i).startPnt_;
					xrc::weld::GeoXYZ tmidPtn = tWeldSeam.wire_.getLine(i).midPnt_;
					xrc::weld::GeoXYZ tEndPtn = tWeldSeam.wire_.getLine(i).endPnt_;
					//xrc::weld::GeoXYZ tGeoNormal = tWeldSeam.wire_.getLineStartNDir(i);
					bool bIsArc = tWeldSeam.wire_.getLine(i).isArc();

					//tWeldInfo.tAtrribute.vtContourPtns.push_back(cvPoint3D64f(tGeoPtn._x, tGeoPtn._y, tGeoPtn._z));
					//tWeldInfo.tAtrribute.vtContourNormal.push_back(cvPoint3D64f(tGeoNormal._x, tGeoNormal._y, tGeoNormal._z));
					if (bIsArc)
					{
						//tWeldInfo.tAtrribute.vtContourPtns.push_back(cvPoint3D64f(tmidPtn._x, tmidPtn._y, tmidPtn._z));
						//tWeldInfo.tAtrribute.vtContourNormal.push_back(cvPoint3D64f(tGeoNormal._x, tGeoNormal._y, tGeoNormal._z));
					}
				}
				//tWeldInfo.tAtrribute.vtContourPtns.push_back(tWeld.EndPoint);
				//tWeldInfo.tAtrribute.vtContourNormal.push_back(tWeld.EndNormalVector);
			}
			else
			{
				// �������ȡ����������
				T_ROBOT_COORS tCoord;
				double dWeldContourLen = tWeldSeam.wire_.length();
				double dContourPtnDis = 5.0;
				vtContour.clear();
				//tWeldInfo.tAtrribute.vtContourNormal.clear();
				for (double dDis = 0; dDis < dWeldContourLen; dDis += dContourPtnDis)
				{
					xrc::weld::GeoXYZ tGeoPtn = tWeldSeam.wire_.getPoint(dDis);
					xrc::weld::GeoXYZ tGeoNormal = tWeldSeam.wire_.getTangentDir(dDis);
					//vtContour.push_back(cvPoint3D64f(tGeoPtn._x, tGeoPtn._y, tGeoPtn._z));
					//tWeldInfo.tAtrribute.vtContourNormal.push_back(cvPoint3D64f(tGeoNormal._x, tGeoNormal._y, tGeoNormal._z));

					tCoord.dX = tGeoPtn._x;
					tCoord.dY = tGeoPtn._y;
					tCoord.dZ = tGeoPtn._z;
					tCoord.dRX = tGeoNormal._x;
					tCoord.dRY = tGeoNormal._y;
					tCoord.dRZ = tGeoNormal._z;
					vtContour.push_back(tCoord);
				}
				//vtContour.push_back(tWeld.EndPoint);
				//tWeldInfo.tAtrribute.vtContourNormal.push_back(tWeld.EndNormalVector);

				tCoord.dX = tWeld.EndPoint.x;
				tCoord.dY = tWeld.EndPoint.y;
				tCoord.dZ = tWeld.EndPoint.z;
				tCoord.dRX = tWeld.EndNormalVector.x;
				tCoord.dRY = tWeld.EndNormalVector.y;
				tCoord.dRZ = tWeld.EndNormalVector.z;
				vtContour.push_back(tCoord);
			}

			//if (GetLocalDebugMark()) // �鿴���ɵĺ���������Ϣ�Ƿ���ȷ
			//{
			//	CString sFileName;
			//	sFileName.Format("%s\\Part%d_Weld%d_IsArc%d.txt",
			//		OUTPUT_PATH + m_pRobotDriver->m_strRobotName + RECOGNITION_FOLDER + "WeldContour",
			//		i, nWeldSeamNo, (int)tWeld.IsArc);
			//	FILE* pf = fopen(sFileName.GetBuffer(), "w");
			//	if (NULL != pf)
			//	{
			//		CvPoint3D64f tPtn;
			//		CvPoint3D64f tNormal;
			//		for (int nPtnNo = 0; nPtnNo < tWeldInfo.tAtrribute.vtContourPtns.size(); nPtnNo++)
			//		{
			//			tPtn = tWeldInfo.tAtrribute.vtContourPtns[nPtnNo];
			//			tNormal = tWeldInfo.tAtrribute.vtContourNormal[nPtnNo];
			//			fprintf(pf, "%d%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf\n",
			//				nPtnNo, tPtn.x, tPtn.y, tPtn.z, tNormal.x, tNormal.y, tNormal.z);
			//		}
			//		fclose(pf);
			//	}
			//}

			tWeldInfo.tWeldLine = tWeld;
			//vtWeldLineInfo.push_back(tWeldInfo);
		}
	}
	return true;
}

void ScanWeldLine::DecomposeExAxle(std::vector<T_ROBOT_COORS>& vtCoord, double& dExAxlePos, double& dExAxlePos_n, int trackAxisNo)
{
	int nCoordNum = vtCoord.size();
	T_ROBOT_COORS tTotalCoord;
	for (int i = 0; i < nCoordNum; i++)
	{
		tTotalCoord = tTotalCoord + vtCoord[i];
	}


	// �ֽ��ⲿ������
	for (int i = 0; i < nCoordNum; i++)
	{
		WeldAfterMeasure::DecomposeExAxle(vtCoord[i], dExAxlePos_n, trackAxisNo);
	}
}

bool ScanWeldLine::TrackFilter(const std::vector<T_ROBOT_COORS>& vtSrcCoord, std::vector<T_ROBOT_COORS>& vtRstCoord, int nFilterOutInterval/* = 8*/, int nCalcPosturePtnStep/* = 8*/)
{
	vtRstCoord.clear();
	std::vector<T_ROBOT_COORS> vtTempCoord(0);
	T_ROBOT_COORS tTempCoors;
	TrackSmooth_PntArray tTrackPtnArray;
	tTrackPtnArray.arraySize_ = 0;
	tTrackPtnArray.pnts_ = NULL;
	// ��ʼ����β���˲�����
	TrackSmooth_Init(m_ptUnit->m_nRobotSmooth, nFilterOutInterval, vtSrcCoord[0].dX, vtSrcCoord[0].dY, vtSrcCoord[0].dZ); // �˲�����3mm

	for (int i = 1; i < vtSrcCoord.size(); i++)
	{
		TrackSmooth_PushNextPoint(m_ptUnit->m_nRobotSmooth, vtSrcCoord[i].dX, vtSrcCoord[i].dY, vtSrcCoord[i].dZ, &tTrackPtnArray);
		for (int n = 0; n < tTrackPtnArray.arraySize_; n++)
		{
			tTempCoors = vtSrcCoord[i];
			tTempCoors.dX = tTrackPtnArray.pnts_[n].x_;
			tTempCoors.dY = tTrackPtnArray.pnts_[n].y_;
			tTempCoors.dZ = tTrackPtnArray.pnts_[n].z_;
			vtTempCoord.push_back(tTempCoors);
		}
		TrackSmooth_FreePntArray(&tTrackPtnArray);
	}

	//��ȡ��β�㣬����βʱ����
	TrackSmooth_GetEndPoint(m_ptUnit->m_nRobotSmooth, &tTrackPtnArray);
	for (int n = 0; n < tTrackPtnArray.arraySize_; n++)
	{
		tTempCoors = vtSrcCoord.back();
		tTempCoors.dX = tTrackPtnArray.pnts_[n].x_;
		tTempCoors.dY = tTrackPtnArray.pnts_[n].y_;
		tTempCoors.dZ = tTrackPtnArray.pnts_[n].z_;
		vtTempCoord.push_back(tTempCoors);
	}
	TrackSmooth_FreePntArray(&tTrackPtnArray);

	SaveCoordToFile(vtSrcCoord, "A_FilterIn.txt");
	SaveCoordToFile(vtTempCoord, "A_FilterOut.txt");

	// ������̬
	if (vtTempCoord.size() < (int)nCalcPosturePtnStep)
	{
		XiMessageBoxOk("�켣�˲�ʧ��");
		return false;
	}
	int nPtnNum = vtTempCoord.size();
	int nIdxS = 0;
	int nIdxE = 0;
	XiAlgorithm alg;
	for (int i = 0; i < nPtnNum; i++)
	{
		nIdxS = i - (nCalcPosturePtnStep / 2);
		nIdxE = i + (nCalcPosturePtnStep / 2);
		if (nIdxS < 0)
		{
			nIdxS = 0;
			nIdxE = nIdxS + nCalcPosturePtnStep;
		}
		if (nIdxE >= nPtnNum)
		{
			nIdxE = nPtnNum - 1;
			nIdxS = nIdxE - nCalcPosturePtnStep;
		}
		double dDeltaX = vtTempCoord[nIdxE].dX + vtTempCoord[nIdxE].dBX - vtTempCoord[nIdxS].dX - vtTempCoord[nIdxS].dBX;
		double dDeltaY = vtTempCoord[nIdxE].dY + vtTempCoord[nIdxE].dBY - vtTempCoord[nIdxS].dY - vtTempCoord[nIdxS].dBY;
		double dDirAngle = alg.CalcArcAngle(dDeltaX, dDeltaY);
		double dNorAngle = dDirAngle - (90.0 * (double)m_nRobotInstallDir);
		tTempCoors = vtTempCoord[i];
		tTempCoors.dRX = m_dStandWeldScanRx;
		tTempCoors.dRY = m_dStandWeldScanRy;
		tTempCoors.dRZ = DirAngleToRz(dNorAngle);
		vtRstCoord.push_back(tTempCoors);
	}
	//�������RZ�仯̫��
	int nNo = 0;
	std::vector<int>vnNo;
	T_ROBOT_COORS tCoor;
	std::vector<T_ROBOT_COORS>vtCoor;
	int nSize = vtRstCoord.size();
	std::vector<T_ROBOT_COORS>vtSmoothCoor;
	for (size_t i = 1; i < nSize; i++)
	{
		if (fabs(vtRstCoord[i].dRZ - vtRstCoord[i - 1].dRZ) > 10.0)
		{
			//XiMessageBox("���������۹켣��RZ�仯̫��");
		}
		if (m_bIsCake == 1)//�ǽ��ߺ���ı�Rz�Ƕ���ת�ǳ�����Ӽ�����
		{
			if (fabs(vtRstCoord[i].dRZ - vtRstCoord[i - 1].dRZ) > 1)
			{
				if (i > 1 && i < vtRstCoord.size() - 1 && fabs(vtRstCoord[i - 2].dRZ - vtRstCoord[i - 1].dRZ) > 1 && fabs(vtRstCoord[i].dRZ - vtRstCoord[i + 1].dRZ) > 1)
				{
					nNo++;
					vnNo.push_back(i);
					for (int k = 1;k < vnNo.size();k++)//�ж��Ƿ������仯�Ƕ�
					{
						if ((vnNo[k] - vnNo[k - 1]) > 1)
						{
							nNo = 0;
							vnNo.clear();
							break;
						}
					}
					if (nNo >= 5)//�������α仯�Ƕ������м�ֽ�һ�ε�
					{
						for (int j = 1;j < vnNo.size();j++)
						{
							tCoor.dX = (vtRstCoord[vnNo[j]].dX + vtRstCoord[vnNo[j - 1]].dX) / 2;
							tCoor.dY = (vtRstCoord[vnNo[j]].dY + vtRstCoord[vnNo[j - 1]].dY) / 2;
							tCoor.dZ = (vtRstCoord[vnNo[j]].dZ + vtRstCoord[vnNo[j - 1]].dZ) / 2;
							tCoor.dRX = (vtRstCoord[vnNo[j]].dRX + vtRstCoord[vnNo[j - 1]].dRX) / 2;
							tCoor.dRY = (vtRstCoord[vnNo[j]].dRY + vtRstCoord[vnNo[j - 1]].dRY) / 2;
							tCoor.dRZ = (vtRstCoord[vnNo[j]].dRZ + vtRstCoord[vnNo[j - 1]].dRZ) / 2;
							tCoor.dBX = (vtRstCoord[vnNo[j]].dBX + vtRstCoord[vnNo[j - 1]].dBX) / 2;
							tCoor.dBY = (vtRstCoord[vnNo[j]].dBY + vtRstCoord[vnNo[j - 1]].dBY) / 2;
							vtCoor.push_back(vtRstCoord[vnNo[j - 1]]);
							vtCoor.push_back(tCoor);
						}
						size_t start = vnNo[0] - 1;
						size_t end = vnNo.back();
						vtRstCoord.erase(vtRstCoord.begin() + start, vtRstCoord.begin() + end);
						vtRstCoord.insert(vtRstCoord.begin() + start, vtCoor.begin(), vtCoor.end());
						vnNo.clear();
						vtCoor.clear();
						nNo = 0;
					}
				}
			}
		}
	}
	SaveCoordToFile(vtRstCoord, "A_FilterResult.txt");
	return true;
}

bool ScanWeldLine::TrackFilter_weld(const std::vector<T_ROBOT_COORS>& vtSrcCoord, std::vector<T_ROBOT_COORS>& vtRstCoord, int nFilterOutInterval/* = 8*/, int nCalcPosturePtnStep/* = 8*/)
{
	vtRstCoord.clear();
	std::vector<T_ROBOT_COORS> vtTempCoord(0);
	T_ROBOT_COORS tTempCoors;
	TrackSmooth_PntArray tTrackPtnArray;
	tTrackPtnArray.arraySize_ = 0;
	tTrackPtnArray.pnts_ = NULL;

	TrackFilter_Init(m_ptUnit->m_nRobotSmooth, nFilterOutInterval);
	TrackFilter_SafetyParameters pam;
	pam.toOutputPntMaxRotationAngle = 0.5;
	pam.toBeforePntMaxRotationAngle = 0.6;
	TrackFilter_SetSafetyParameters(m_ptUnit->m_nRobotSmooth, pam);
	// ��ʼ����β���˲�����
	//TrackSmooth_Init(m_ptUnit->m_nRobotSmooth, nFilterOutInterval, vtSrcCoord[0].dX, vtSrcCoord[0].dY, vtSrcCoord[0].dZ); // �˲�����3mm

	std::vector<T_ROBOT_COORS> vttc;
	for (int j = 0; j < vtSrcCoord.size() / 150; j++)
	{
		vttc.push_back(vtSrcCoord[150 * j]);
	}

	for (int i = 0; i < vttc.size(); i++)
	{
		//TrackSmooth_PushNextPoint(m_ptUnit->m_nRobotSmooth, vtSrcCoord[i].dX, vtSrcCoord[i].dY, vtSrcCoord[i].dZ, &tTrackPtnArray);
		std::vector<TrackFilter_Node> track = TrackFilter_FilterCurvePointInPointOut(m_ptUnit->m_nRobotSmooth, vttc[i].dX, vttc[i].dY, vttc[i].dZ, false);



		double length = 0;
		bool doneMove = TrackFilter_DoneMove(m_ptUnit->m_nRobotSmooth, &length);

		TrackFilter_Answer answer = TrackFilter_GetToEndPnts(m_ptUnit->m_nRobotSmooth, false);
		TrackFilter_Free(answer);

		std::vector<TrackFilter_XYZ> cachePnts = TrackFilter_GetCachePntsPlus(m_ptUnit->m_nRobotSmooth);

		for (int n = 0; n < track.size(); n++)
		{
			tTempCoors = vttc[i];
			tTempCoors.dX = track[n].pnt_.x_;
			tTempCoors.dY = track[n].pnt_.y_;
			tTempCoors.dZ = track[n].pnt_.z_;
			vtTempCoord.push_back(tTempCoors);
		}
		//TrackSmooth_FreePntArray(&tTrackPtnArray);
	}

	for (int n = 6; n < 0; n++)
	{
		tTempCoors = vttc.back();
		tTempCoors.dX = vttc[n].dX;
		tTempCoors.dY = vttc[n].dY;
		tTempCoors.dZ = vttc[n].dZ;
		vtTempCoord.insert(vtTempCoord.begin(), tTempCoors);
	}
	TrackSmooth_FreePntArray(&tTrackPtnArray);

	SaveCoordToFile(vttc, "A_FilterIn.txt");
	SaveCoordToFile(vtTempCoord, "A_FilterOut.txt");

	// ������̬
	if (vtTempCoord.size() < (int)nCalcPosturePtnStep)
	{
		XiMessageBoxOk("�켣�˲�ʧ��");
		return false;
	}
	int nPtnNum = vtTempCoord.size();
	int nIdxS = 0;
	int nIdxE = 0;
	XiAlgorithm alg;
	for (int i = 0; i < nPtnNum; i++)
	{
		nIdxS = i - (nCalcPosturePtnStep / 2);
		nIdxE = i + (nCalcPosturePtnStep / 2);
		if (nIdxS < 0)
		{
			nIdxS = 0;
			nIdxE = nIdxS + nCalcPosturePtnStep;
		}
		if (nIdxE >= nPtnNum)
		{
			nIdxE = nPtnNum - 1;
			nIdxS = nIdxE - nCalcPosturePtnStep;
		}
		double dDirAngle = alg.CalcArcAngle(
			vtTempCoord[nIdxE].dX + vtTempCoord[nIdxE].dBX - vtTempCoord[nIdxS].dX - vtTempCoord[nIdxS].dBX,
			vtTempCoord[nIdxE].dY + vtTempCoord[nIdxE].dBY - vtTempCoord[nIdxS].dY - vtTempCoord[nIdxS].dBY);
		double dNorAngle = dDirAngle - (90.0 * (double)m_nRobotInstallDir);
		tTempCoors = vtTempCoord[i];
		tTempCoors.dRX = 0.0;
		tTempCoors.dRY = m_dStandWeldScanRy;
		tTempCoors.dRZ = DirAngleToRz(dNorAngle);
		vtRstCoord.push_back(tTempCoors);
	}
	SaveCoordToFile(vtRstCoord, "A_FilterResult.txt");
	return true;
}

void ScanWeldLine::SaveMeasureTrackGunTool(int nGroupNo, const std::vector<T_ROBOT_COORS>& vtCoord)
{
	int nCoordNum = vtCoord.size();
	if (nCoordNum <= 0)
	{
		return;
	}
	CString sFileName;
	sFileName.Format("%s%d-MeasureTrackGunTool.txt", m_sDataSavePath, nGroupNo);
	FILE* pf = fopen(sFileName.GetBuffer(), "w");
	for (int i = 0; i < vtCoord.size(); i++)
	{
		fprintf(pf, "%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf\n",
			vtCoord[i].dX + vtCoord[i].dBX,
			vtCoord[i].dY + vtCoord[i].dBY,
			vtCoord[i].dZ + vtCoord[i].dBZ,
			vtCoord[i].dX, vtCoord[i].dY, vtCoord[i].dZ,
			vtCoord[i].dRX, vtCoord[i].dRY, vtCoord[i].dRZ,
			vtCoord[i].dBX, vtCoord[i].dBY, vtCoord[i].dBZ);
	}
	fclose(pf);

	CString sFileName1;
	sFileName1.Format("%s%d-RefTrackGunTool.txt", m_sDataSavePath, nGroupNo);
	FILE* pf1 = fopen(sFileName1.GetBuffer(), "w");
	for (int i = 0; i < vtCoord.size(); i++)
	{
		fprintf(pf1, "%d%11.3lf%11.3lf%11.3lf\n",
			i,
			vtCoord[i].dX + vtCoord[i].dBX,
			vtCoord[i].dY + vtCoord[i].dBY,
			vtCoord[i].dZ + vtCoord[i].dBZ);
	}
	fclose(pf1);
}

void ScanWeldLine::SaveMeasureTrackCamTool(int nGroupNo, const std::vector<T_ROBOT_COORS>& vtCoord, const std::vector<T_ANGLE_PULSE>& vtPulse, const std::vector<int>& vtPtnType)
{
	int nCoordNum = vtCoord.size();
	int nPulseNum = vtPulse.size();
	int nTypeNum = vtPtnType.size();
	if (nCoordNum != nPulseNum || nCoordNum != nTypeNum || nCoordNum <= 0)
	{
		XiMessageBoxOk("������������쳣������ʧ��");
		return;
	}
	CString sFileName;
	sFileName.Format("%s%d-MeasureTrackRealCoordPulseEx.txt", m_sDataSavePath, nGroupNo);
	FILE* pf = fopen(sFileName.GetBuffer(), "w");
	for (int i = 0; i < vtCoord.size(); i++)
	{
		fprintf(pf, "%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%10d%10d%10d%10d%10d%10d\n",
			vtCoord[i].dX + vtCoord[i].dBX,
			vtCoord[i].dY + vtCoord[i].dBY,
			vtCoord[i].dZ + vtCoord[i].dBZ,
			vtCoord[i].dX, vtCoord[i].dY, vtCoord[i].dZ,
			vtCoord[i].dRX, vtCoord[i].dRY, vtCoord[i].dRZ,
			vtPulse[i].nSPulse, vtPulse[i].nLPulse, vtPulse[i].nUPulse,
			vtPulse[i].nRPulse, vtPulse[i].nBPulse, vtPulse[i].nTPulse);
	}
	fclose(pf);
}

bool ScanWeldLine::TransCoordByTool(std::vector<T_ROBOT_COORS>& vtCoord, T_ROBOT_COORS tSrcTool, T_ROBOT_COORS tDstTool)
{
	// ��ǹ����ת�������
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

void ScanWeldLine::AddSafeCoord(std::vector<T_ROBOT_COORS>& vtCoord, vector<int>& vnType, double dMaxPartHeightZ)
{
	// ��ǹ ��ȫ��
	T_ROBOT_COORS tCoord = vtCoord.front();
	double dOffsetDirAngle = m_pRobotDriver->RzToDirAngle(tCoord.dRZ);
	tCoord.dX += (m_dGunDownBackSafeDis / 2.0 * CosD(dOffsetDirAngle));
	tCoord.dY += (m_dGunDownBackSafeDis / 2.0 * SinD(dOffsetDirAngle));
	tCoord.dZ = dMaxPartHeightZ + (m_dGunDownBackSafeDis * (double)m_nRobotInstallDir);
	tCoord.dRX = m_dPlatWeldRx;
	tCoord.dRY = m_dPlatWeldRy;
	vtCoord.insert(vtCoord.begin(), tCoord);
	vnType.insert(vnType.begin(), E_TRANSITION_POINT);

	// ��ǹ ��ȫ��
	tCoord = vtCoord.back();
	dOffsetDirAngle = m_pRobotDriver->RzToDirAngle(tCoord.dRZ);
	tCoord.dX += (m_dGunDownBackSafeDis / 2.0 * CosD(dOffsetDirAngle));
	tCoord.dY += (m_dGunDownBackSafeDis / 2.0 * SinD(dOffsetDirAngle));
	tCoord.dZ = dMaxPartHeightZ + (m_dGunDownBackSafeDis * (double)m_nRobotInstallDir);
	tCoord.dRX = m_dPlatWeldRx;
	tCoord.dRY = m_dPlatWeldRy;
	vtCoord.push_back(tCoord);
	vnType.push_back(E_TRANSITION_POINT);
}


bool ScanWeldLine::MoveExAxleToDst(double dPos, int nAxisNo, double dSpeed)
{
	if (GetLocalDebugMark())
	{
		return true;
	}
	double dCurExPos = m_ptUnit->GetExPositionDis(nAxisNo);
	if (fabs(dPos - dCurExPos) > 5.0)
	{
		if (0 != m_ptUnit->MoveExAxisFun(dPos, dSpeed, nAxisNo))
		{
			return false;
		}
		m_ptUnit->WorldCheckRobotDone();
		dCurExPos = m_ptUnit->GetExPositionDis(nAxisNo);
		if (fabs(dPos - dCurExPos) > 5.0) // ��Ŀ��λ��������ֵ �ж�Ϊ�˶�ʧ��
		{
			XiMessageBox("�Զ�ʾ��:�ⲿ��δ�˶���ָ��λ��");
			return false;
		}
	}
	return true;
}

bool ScanWeldLine::GetScanWeldLineResult()
{
	m_vtTeachResult.clear();
	int nPtnNum = m_pScanInit->m_pTraceModel->vtConnerWorldCoords.size();
	if (0 >= nPtnNum)
	{
		XiMessageBoxOk("ɨ����Ϊ�գ�ɨ��ʧ�ܣ�");
		return false;
	}
	T_TEACH_RESULT tTeachResult;
	//memset(&tTeachResult, 0, sizeof(tTeachResult));
	CvPoint tPtn2D;
	tPtn2D.x = 0;
	tPtn2D.y = 0;
	tTeachResult.tKeyPtn2D = tPtn2D;
	for (int i = 0; i < nPtnNum; i++)
	{
		XI_POINT tPtn;
		tPtn.x = m_pScanInit->m_pTraceModel->vtConnerWorldCoords[i].dX;
		tPtn.y = m_pScanInit->m_pTraceModel->vtConnerWorldCoords[i].dY;
		tPtn.z = m_pScanInit->m_pTraceModel->vtConnerWorldCoords[i].dZ;

		tTeachResult.vtLeftPtns3D.push_back(tPtn);
		tTeachResult.vtLeftPtns2D.push_back(tPtn2D);
	}
	m_vtTeachResult.push_back(tTeachResult);
	return true;
}


UINT ScanWeldLine::ThreadMeasure(void* pParam)
{
	try
	{

		ScanWeldLine* pObj = (ScanWeldLine*)pParam;
		// ����ɨ��ɼ�����
		CString strFile;
		strFile.Format("%s%s%s\\%d_ScanWeldLinePointCloud%d.txt", OUTPUT_PATH,
			pObj->m_ptUnit->GetRobotCtrl()->m_strRobotName, RECOGNITION_FOLDER, 0, pObj->m_pRobotDriver->m_nScanNum);
		E_POINT_CLOUD_PROC_MOMENT eProcMoment = E_POINT_CLOUD_PROC_MOMENT_FINAL;
		E_IMAGE_PROC_METHOD eImgProcMethod = E_IMAGE_PROC_METHOD_EEEEE;
		E_POINT_CLOUD_PROC_METHOD ePointCloudProcMethod = E_POINT_CLOUD_PROC_METHOD_GetRiserEndPoint;
		pObj->m_pScanInit->InitDynamicCapture_H_M(
			eProcMoment, eImgProcMethod, ePointCloudProcMethod, pObj->m_ptUnit->m_nMeasureCameraNo,
			OUTPUT_PATH + pObj->m_pRobotDriver->m_strRobotName + WELDDATA_PATH, "AA_PointCloud.txt", 80, E_FLAT_SEAM, 0, 0, false);
		//pObj->m_pScanInit->InitDynamicCapture_H_M(
		//	eProcMoment, eImgProcMethod, ePointCloudProcMethod, pObj->m_ptUnit->m_nTrackCameraNo,
		//	OUTPUT_PATH + pObj->m_pRobotDriver->m_strRobotName + WELDDATA_PATH, "AA_PointCloud.txt", 80, E_FLAT_SEAM, 0, 0, false);
		//m_pScanInit->InitRealTimeTracking(E_JUDGE_END_PROCESS_SEARCH); // eProcMoment ʵʱ����ʱ��Ч
		pObj->m_pScanInit->DynamicCaptureNew_ScanWeldLine(pObj->m_pRobotDriver, pObj->m_ptUnit->m_nMeasureCameraNo, strFile, 10000);
		//pObj->m_pScanInit->DynamicCaptureNew_ScanWeldLine(pObj->m_pRobotDriver, pObj->m_ptUnit->m_nTrackCameraNo, strFile, 10000);
	}
	catch (...)
	{
		XiMessageBox("ȫɨ�账���쳣");
		return false;
	}
	return 0;
}

//void ScanWeldLine::WeldInfoToJson(Welding_Info tWeldInfo)
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
//		// ���浽�ļ�
//		CString output_weldsinfo_path = OUTPUT_PATH + m_ptUnit->m_tContralUnit.strUnitName + RECOGNITION_FOLDER + "weld.json";
//		xrc::weld::writeWeldDataDocJson(weldDoc, output_weldsinfo_path);
//	}
//}

void ScanWeldLine::WeldInfoToJson(Welding_Info tWeldInfo)
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

		for (size_t j = 0; j < tracks.points_size - 1; j++)
		{
			xrc::weld::GeoXYZ sPoint{ tracks.points[j].x, tracks.points[j].y, tracks.points[j].z };
			xrc::weld::GeoXYZ ePoint{ tracks.points[j + 1].x, tracks.points[j + 1].y, tracks.points[j + 1].z };

			// ����һ��ֱ��
			xrc::weld::ThreePointLine line{ sPoint, ePoint };


			xrc::weld::GeoXYZ sNormal{ tracks.points_normal[j].x , tracks.points_normal[j].y , tracks.points_normal[j].z };
			xrc::weld::GeoXYZ eNormal{ tracks.points_normal[j + 1].x , tracks.points_normal[j + 1].y ,tracks.points_normal[j + 1].z };

			weldWire.addBack(line, sNormal, eNormal);
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

void ScanWeldLine::BackHome()
{
	CHECK_BOOL(m_ptUnit->CheckIsReadyRun());

	// ���������� or ���һ����� ��
	double dRobotInstallMode = m_pRobotDriver->m_nRobotInstallDir; // ��װ��1.0   ��װ��-1.0
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

	tCurCoord = m_pRobotDriver->GetCurrentPos();
	tCurPulse = m_pRobotDriver->GetCurrentPulse();
	m_pRobotDriver->RobotKinematics(m_pRobotDriver->m_tHomePulse, m_pRobotDriver->m_tTools.tGunTool, tHomeCoord);
	m_pRobotDriver->RobotKinematics(tCurPulse, m_pRobotDriver->m_tTools.tGunTool, tTransCoord);

	// ��ȡֱ������Ͷ�ȡ�ؽ�����ת�����������ͬʱ�������˶�
	//if (false == m_pRobotDriver->CompareCoords(tTransCoord, tCurCoord))
	//{
	//	WriteLog("MoveToSafeHeight:��ȡ����ͼ�������������");
	//	XiMessageBoxOk("��ȡ����ͼ������겻ͬ���޷��Զ��ذ�ȫλ�ã�");
	//	return;
	//}

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
		tRobotMoveInfo = m_pRobotDriver->PVarToRobotMoveInfo(0, tTempCoord, m_pRobotDriver->m_tCoordHighSpeed, MOVL);
		vtRobotMoveInfo.push_back(tRobotMoveInfo);
	}
	tRobotMoveInfo = m_pRobotDriver->PVarToRobotMoveInfo(1, m_pRobotDriver->m_tHomePulse, m_pRobotDriver->m_tBackHomeSpeed, MOVJ);
	vtRobotMoveInfo.push_back(tRobotMoveInfo);
	m_pRobotDriver->SetMoveValue(vtRobotMoveInfo);
	m_pRobotDriver->CallJob("CONTIMOVANY");
	m_ptUnit->RobotCheckDone();

	if (false == m_pRobotDriver->ComparePulse(m_pRobotDriver->m_tHomePulse))
	{
		WriteLog("�ذ�ȫλ��ʧ��! ");
		return;
	}
	WriteLog("�ذ�ȫλ�����! ");

	return;
}