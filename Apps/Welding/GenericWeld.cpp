#include "StdAfx.h"
#include "GenericWeld.h"
#include "Apps/RealTimeTrack/AdjustTrack.h"

GenericWeld::GenericWeld(CUnit* ptUnit, E_WORKPIECE_TYPE eWorkPieceType) :
	WeldAfterMeasure(ptUnit, eWorkPieceType)
{
}

GenericWeld::~GenericWeld()
{

}

void GenericWeld::SetRecoParam()
{
	COPini opini;
	opini.SetFileName(DATA_PATH + m_ptUnit->m_tContralUnit.strUnitName + SYETEM_PARAM_FILE);
	switch (m_eWorkPieceType)
	{
	case E_LINELLAE:
		opini.SetSectionName("E_LINELLAE");
		opini.ReadString("ParamValue0", &m_dZPallet);
		opini.ReadString("ParamValue1", &m_dMaxWeldLen);
		opini.ReadString("ParamValue2", &m_dJointLen);
		opini.ReadString("ParamValue3", &m_dScanEndpointOffset);
		opini.ReadString("ParamValue4", &m_nExPartType);
		opini.ReadString("ParamValue5", &m_nWeldMode);
		break;
	case SMALL_PIECE: 
		opini.SetSectionName("SMALL_PIECE");
		opini.ReadString("ParamValue0", &m_dZPallet);
		opini.ReadString("ParamValue1", &m_dWeldHoleSize);
		opini.ReadString("ParamValue2", &m_dScanEndpointOffset);
		break;
	case STIFFEN_PLATE:
		opini.SetSectionName("STIFFEN_PLATE");
		opini.ReadString("ParamValue0", &m_dZPallet);
		opini.ReadString("ParamValue1", &m_dSideBoardThick);
		opini.ReadString("ParamValue2", &m_dWeldHoleSize);
		opini.ReadString("ParamValue3", &m_dBoardLen);
		opini.ReadString("ParamValue4", &m_dScanEndpointOffset);
		break;
	case E_CORBEL:
		opini.SetSectionName("E_CORBEL");
		opini.ReadString("ParamValue0", &m_dZPallet);
		opini.ReadString("ParamValue1", &m_dWeldHoleSize);
		opini.ReadString("ParamValue2", &m_dScanEndpointOffset);
		break;
	case E_END_PLATE:
		opini.SetSectionName("E_END_PLATE");
		opini.ReadString("ParamValue0", &m_dZPallet);
		opini.ReadString("ParamValue1", &m_bDeleteSeam);
		opini.ReadString("ParamValue2", &m_dWeldHoleSize);
		opini.ReadString("ParamValue3", &m_dEndBoardSupportLen);
		opini.ReadString("ParamValue4", &m_dPurlinSupportLen);
		opini.ReadString("ParamValue5", &m_dPurlinLen);
		opini.ReadString("ParamValue6", &m_dEndBoardLen);
		opini.ReadString("ParamValue7", &m_nExPartType);
		opini.ReadString("ParamValue8", &m_dScanEndpointOffset);
		break;
	case E_CAKE:
		opini.SetSectionName("E_CAKE");
		opini.ReadString("ParamValue0", &m_dZPallet);
		break;
	case E_BEVEL:
		opini.SetSectionName("E_BEVEL");
		opini.ReadString("ParamValue0", &m_dZPallet);
		opini.ReadString("ParamValue1", &m_dSideBoardThick);
		opini.ReadString("ParamValue2", &m_dScanEndpointOffset);
		break;
	
	//���޸�
	default: XUI::MesBox::PopInfo("GenericWeld��֧�ֹ�������!")/*XiMessageBox("GenericWeld��֧�ֹ�������%d!", m_eWorkPieceType)*/; break;
	//default: XiMessageBox("GenericWeld��֧�ֹ�������%d!", m_eWorkPieceType); break;
	}
	opini.SetSectionName("E_LINELLAE");
	opini.ReadString("ParamValue1", &m_dMaxWeldLen);
	opini.ReadString("ParamValue2", &m_dJointLen);
}

bool GenericWeld::PointCloudProcess(bool bUseModel, CvPoint3D64f* pPointCloud/* = NULL*/, int PointCloudSize/* = 0*/)
{
	if (bUseModel)
		return PointCloudProcessWithModel();

	m_vtWeldSeamData.clear();
	int nWeldingSeamNumber = 0;
	bool bInstallDir = 1 == m_nRobotInstallDir ? true : false;
	vector<CvPoint3D64f> vtPointCloud;
	LineOrCircularArcWeldingLine* pLineOrCircularArcWeldingLine; // ��ΰ
	//m_sPointCloudFileName = "LocalFiles\\OutputFiles\\RobotA\\Recognition\\PointCloud_0.txt";
	int nCurUseTable = 0;
	COPini opini2;
	
	opini2.SetFileName(DATA_PATH + m_ptUnit->m_tContralUnit.strUnitName + LINE_SCAN_PARAM);
	opini2.SetSectionName("CurUseTableNo");
	opini2.ReadString("CurUseTableNo", &nCurUseTable);

	//��ɨ
	CString FileName = "LineScan\\GantryLeft\\PointCloud\\Scan5D.txt";
	if (nCurUseTable != 0)
		 FileName = "LineScan\\GantryRight\\PointCloud\\Scan5D.txt";
	try
	{
		if (NULL == pPointCloud)
		{
			LoadContourData(m_pRobotDriver, FileName, vtPointCloud);
			pPointCloud = (CvPoint3D64f*)vtPointCloud.data();
			PointCloudSize = vtPointCloud.size();
		}

		//std::string FileName = ".\\LineScan\\Gantry\\PointCloud\\Scan5D.txt";
		std::string FileNameSave = ".\\LineScan\\Gantry\\PointCloud\\Scan5D-Background.bin";
		//if (NULL == pPointCloud)
		//{

		//	int maxPntNum = 60000000;
		//	int CoverNum = 1;
		//	int threadnum = 32;

		//	vtPointCloud.resize(maxPntNum);
		//	clock_t tTime = clock();
		//	int nCloudSize = TXTCloudFileRead((char*)FileName.c_str(), vtPointCloud.data(), maxPntNum, CoverNum, threadnum);
		//	vtPointCloud.resize(nCloudSize);
		//	//XiMessageBox("%d", clock() - tTime);
		//	pPointCloud = (CvPoint3D64f*)vtPointCloud.data();
		//	PointCloudSize = vtPointCloud.size();
		//}


		//SaveContourData();	//�����е������ݱ��浽һ���ļ���
		if (E_LINELLAE == m_eWorkPieceType)
		{
			if (!Get_LineOrCircularArcWeldingLine_JiaMuSi_Prefab_01((Three_DPoint*)pPointCloud, PointCloudSize,
				pLineOrCircularArcWeldingLine, nWeldingSeamNumber,
				"LocalFiles\\ExLib\\Vision\\ConfigFiles\\Get_LineOrCircularArcWeldingLine_JiaMuSi_Prefab_01.ini"))
			{
				nWeldingSeamNumber = 0;
			}
			//if (0 == m_nExPartType)
			//{
			//	// ���ָ� С���ӿ� 
			//	pLineOrCircularArcWeldingLine = SmallSamplePointCloudToWeldingLines(
			//		(Three_DPoint*)pPointCloud, PointCloudSize, &nWeldingSeamNumber, bInstallDir, m_dZPallet, 5.0, 25, 60,80,  false, false, 8/*, true, false, 0, false*/);
			//}
			//else 
			//{
			//	//// ���ָ� ţ�Ƚӿ�
			//	//pLineOrCircularArcWeldingLine = BracketPointCloudToWeldingLines(
			//	//	(Three_DPoint*)pPointCloud, PointCloudSize, &nWeldingSeamNumber, bInstallDir, m_dZPallet, 5.0, false);
			//}
			SavePointCloudProcessResult((LineOrCircularArcWeldingLine*)pLineOrCircularArcWeldingLine, nWeldingSeamNumber);
			ReleaseWeldingLines(&pLineOrCircularArcWeldingLine); 
			LoadCloudProcessResultNew(OUTPUT_PATH + m_ptUnit->m_tContralUnit.strUnitName + "\\" + POINT_CLOUD_IDENTIFY_RESULT);
			Segmentation();
		}
		else if (SMALL_PIECE == m_eWorkPieceType)
		{
			//// Բ��
			/*pLineOrCircularArcWeldingLine = ArcPlatePointCloudToWeldingLinesExtraction(
				(Three_DPoint*)pPointCloud, PointCloudSize, &nWeldingSeamNumber,
				bInstallDir, true, m_dZPallet, 5.0, false, true, false, 15, 4, 1);*/
				// ���ָ� С���ӿ� 
			pLineOrCircularArcWeldingLine = SmallSamplePointCloudToWeldingLines(
				(Three_DPoint*)pPointCloud, PointCloudSize, &nWeldingSeamNumber, bInstallDir, m_dZPallet, 5.0, 12, 60, 80, false, false, 8/*, true, false, 0, false*/);
			SavePointCloudProcessResult((LineOrCircularArcWeldingLine*)pLineOrCircularArcWeldingLine, nWeldingSeamNumber);
			ReleaseWeldingLines(&pLineOrCircularArcWeldingLine);
			/*LoadCloudProcessResultNew(OUTPUT_PATH + m_ptUnit->m_tContralUnit.strUnitName + "\\" + POINT_CLOUD_IDENTIFY_RESULT);
			Segmentation();*/
#if 0
			std::string ModelFileName1 = OUTPUT_PATH + "ModelMatch1\\����202240409_2.txt";// ԭʼ����ģ��
			std::string AllModelFileRoute = OUTPUT_PATH + "ModelMatch1";
			std::string SaveRoute_weldline_point = ".\\LocalFiles\\OutputFiles\\RobotA\\Recognition\\SaveRoute_weldline_point.txt";
			//FigureAllTemplateTransformMatrix
			int nMatrixNum = 0;
			TemplateTransformMatrix* pTemplateTransMat = FigureAllTemplateTransformMatrix((Three_DPoint*)pPointCloud, PointCloudSize,
				&nMatrixNum, bInstallDir, true, m_dZPallet, 5.0, 5, true, 20.0, AllModelFileRoute.c_str(), SaveRoute_weldline_point.c_str(), 4);

			double dCoarseMatrix[4][4];
			double dFineMatrix[4][4];
			//FILE* pf = fopen("TemplateTransformMatrix.txt", "w");
			// ԭʼ����ֵ
			for (int i = 0; i < 4; i++)
			{
				for (int j = 0; j < 4; j++)
				{
					dCoarseMatrix[i][j] = pTemplateTransMat->Transform_Matrix[i][j];
				}
				//fprintf(pf, "%.3lf %.3lf %.3lf %.3lf\n", dCoarseMatrix[i][0], dCoarseMatrix[i][1], dCoarseMatrix[i][2], dCoarseMatrix[i][3]);
			}
			//fclose(pf);

			ModelMatching(dCoarseMatrix, ModelFileName1, 0, SaveRoute_weldline_point, dFineMatrix);

			// ��ȷ����ֵ
			for (int i = 0; i < 4; i++)
			{
				for (int j = 0; j < 4; j++)
				{
					pTemplateTransMat[0].Transform_Matrix[i][j] = dFineMatrix[i][j];
				}
			}

			std::string RealModelFileName1 = OUTPUT_PATH + "ModelMatch2"; // ��ʵҪ����ģ��
			std::string SaveRoute_AllWeldingLine  = ".\\LocalFiles\\OutputFiles\\RobotA\\Recognition\\PointCloudIdentifyReault.txt"; //������·��
			TransformAllTemplatesToReal(RealModelFileName1.c_str(), pTemplateTransMat, nMatrixNum, SaveRoute_AllWeldingLine.c_str());
			return true;
#endif // 0
		}
		else if (STIFFEN_PLATE == m_eWorkPieceType)
		{
			//// ����������
			//LineOrCircularArcWeldingSeam* pLineOrCircularArcWeldingSeam;
			//WeldingSeamGroup* pWeldingSeamGroup;
			//pWeldingSeamGroup = RecognizeWeldingSeamsAccordingToRule1(
			//	pPointCloud, PointCloudSize, &nWeldingSeamNumber, m_dSideBoardThick,
			//	m_dZPallet, true, 8, 1000);
			//SavePointCloudProcessResult(pLineOrCircularArcWeldingSeam, nWeldingSeamNumber);
			//ReleaseLineOrCircularArcWeldingSeamsPointer(&pLineOrCircularArcWeldingSeam);
			// ��ΰ������
			pLineOrCircularArcWeldingLine = BeamPointCloudToWeldingLines((Three_DPoint*)pPointCloud, PointCloudSize, &nWeldingSeamNumber, bInstallDir, m_dZPallet, 5.0);
			SavePointCloudProcessResult((LineOrCircularArcWeldingLine*)pLineOrCircularArcWeldingLine, nWeldingSeamNumber);
			ReleaseWeldingLines(&pLineOrCircularArcWeldingLine);
		}
		else if (E_CORBEL == m_eWorkPieceType)
		{
			// ��ΰʶ��ӿ� ţ��
			//pLineOrCircularArcWeldingLine = BracketPointCloudToWeldingLines(
			//	(Three_DPoint*)pPointCloud, PointCloudSize, &nWeldingSeamNumber, bInstallDir, m_dZPallet, 5.0, false);
			//SavePointCloudProcessResult((LineOrCircularArcWeldingLine*)pLineOrCircularArcWeldingLine, nWeldingSeamNumber);
			//ReleaseWeldingLines(&pLineOrCircularArcWeldingLine);
		}
		else if (E_END_PLATE == m_eWorkPieceType)
		{
			// ��������ֵΪ������Ϣ�ṹ���ָ��
			// ��ΰʶ��ӿ� �˰�  ������:��� ���� �˰� ���
			pLineOrCircularArcWeldingLine = PurlinSupportPointCloudToWeldingLines(
				(Three_DPoint*)pPointCloud, PointCloudSize, &nWeldingSeamNumber, bInstallDir, 
				m_dZPallet, m_bDeleteSeam, 5.0, 3600, m_dEndBoardSupportLen, m_dPurlinSupportLen, m_dPurlinLen, m_dEndBoardLen, m_nExPartType,false,50,25,40);

			auto pf2 = fopen(".\\test123.txt", "w");
			for (size_t i = 0; i < nWeldingSeamNumber; i++)
			{
				LineOrCircularArcWeldingLine tWeld = pLineOrCircularArcWeldingLine[i];
				fprintf(pf2, "%zd %4d%4d%11.3lf%4d%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%4d%4d 0 0 0 %4d%11.3lf%11.3lf 0 0.0 0.0\n",
					i, tWeld.IsArc, tWeld.isClockwise, tWeld.ZSide, tWeld.isLeft,
					tWeld.CenterPoint.x, tWeld.CenterPoint.y, tWeld.CenterPoint.z - 0.0,
					tWeld.StartPoint.x, tWeld.StartPoint.y, tWeld.StartPoint.z - 0.0,
					tWeld.EndPoint.x, tWeld.EndPoint.y, tWeld.EndPoint.z - 0.0,
					tWeld.StartNormalVector.x, tWeld.StartNormalVector.y, tWeld.StartNormalVector.z,
					tWeld.EndNormalVector.x, tWeld.EndNormalVector.y, tWeld.EndNormalVector.z,
					tWeld.StartPointType, tWeld.EndPointType, 6, 0.0, 0.0);
			}
			fclose(pf2);

			SavePointCloudProcessResult((LineOrCircularArcWeldingLine*)pLineOrCircularArcWeldingLine, nWeldingSeamNumber);
			ReleaseWeldingLines(&pLineOrCircularArcWeldingLine);
		}
		else if (E_CAKE == m_eWorkPieceType)
		{
			pLineOrCircularArcWeldingLine = SquareOutsideCircleInsideWeldingLine(
				(Three_DPoint*)pPointCloud, PointCloudSize, &nWeldingSeamNumber, bInstallDir, m_dZPallet, 5.0);
			SavePointCloudProcessResult((LineOrCircularArcWeldingLine*)pLineOrCircularArcWeldingLine, nWeldingSeamNumber);
			ReleaseWeldingLines(&pLineOrCircularArcWeldingLine);
		}
		else if (E_BEVEL == m_eWorkPieceType)
		{
			//((CButton*)GetDlgItem(IDC_BUTTON3))->EnableWindow(TRUE);
			//((CButton*)GetDlgItem(��ťID))->EnableWindow(TRUE);
			pLineOrCircularArcWeldingLine = GroovePointCloudToWeldingLines(
				(Three_DPoint*)pPointCloud, PointCloudSize, &nWeldingSeamNumber, bInstallDir, m_dZPallet, m_dSideBoardThick);
			SavePointCloudProcessResult((LineOrCircularArcWeldingLine*)pLineOrCircularArcWeldingLine, nWeldingSeamNumber);
			ReleaseWeldingLines(&pLineOrCircularArcWeldingLine);
		}
		else
		{
			XiMessageBox("ͨ���ࣺ���ʹ��� ����ʧ�ܣ�");
			return false;
		}
	}
	catch (...)
	{
		XiMessageBox("�ӽ�崦���쳣");
		return false;
	}

	//���޸�
	XUI::MesBox::PopInfo("��������Ϊ:{0}", nWeldingSeamNumber);
	//XiMessageBox("��������Ϊ:%d", nWeldingSeamNumber);
	if (nWeldingSeamNumber <= 0)
	{
		return false;
	}	
	return true;
}

bool GenericWeld::WeldSeamGrouping(int& nWeldGroupNum)
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
	double dMinStandSeamLen = 10.0; // ��С��������
	m_vvtWeldSeamGroup.clear();
	// ����ƽ������ �� ƽ������յ�����
	vector<WeldLineInfo> vtWeldSeamPlane(0);
	vector<WeldLineInfo> vtWeldLineStandard(0);
	vector<WeldLineInfo> vtWeldLineArc(0);
	vector<WeldLineInfo> vtWeldLinePlaneOverLength(0);
	vector<CvPoint3D64f> vtPtn(0); // ƽ������ʹ��(�ϲ��غϵĵ�)
	vector<vector<int>> vvnWeldNo(0); // ƽ������ʹ��(ÿ���ϲ����������Ķ�������ź�)
	XiAlgorithm alg;
	CheckSeamDataEndpointType(dGroupDisThreshold); // ���˵������Ƿ���ȷ(�������ʶ��Ϊ����)
	for (int nWeldNo = 0; nWeldNo < m_vtWeldSeamInfo.size(); nWeldNo++)
	{
		WeldLineInfo &tLineSeamInfo = m_vtWeldSeamInfo[nWeldNo];
		LineOrCircularArcWeldingLine &tLineSeam = tLineSeamInfo.tWeldLine;
		double dWeldSeamLen = TwoPointDis(
			tLineSeam.StartPoint.x, tLineSeam.StartPoint.y, tLineSeam.StartPoint.z,
			tLineSeam.EndPoint.x, tLineSeam.EndPoint.y, tLineSeam.EndPoint.z);
		tLineSeamInfo.tAtrribute.bWeldMode = false; // Ĭ�ϲ�����
		if (IsStandWeldSeam(tLineSeam.StartNormalVector)) // ��������յ㷨����ZֵΪ0
		{
			if (dWeldSeamLen < dMinStandSeamLen)
			{
				//���޸�
				XUI::MesBox::PopInfo("��������{0} ����С��{1:.1f},�Զ�ɾ����", nWeldNo, dMinStandSeamLen);
				//XiMessageBoxOk("��������%d ����С��%.1lf,�Զ�ɾ����", nWeldNo, dMinStandSeamLen);
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
				vtWeldLinePlaneOverLength.push_back(tLineSeamInfo);
				continue;
			}
			if (dWeldSeamLen < dMinFlatSeamLen)
			{
				//���޸�
				XUI::MesBox::PopInfo("ƽ������{0} ����С��{1:.1f},�Զ�ɾ����", nWeldNo, dMinFlatSeamLen);
				//XiMessageBoxOk("ƽ������%d ����С��%.1lf,�Զ�ɾ����", nWeldNo, dMinFlatSeamLen);
				continue;
			}
			if (dWeldSeamLen < m_dShortSeamThreshold &&
				0 < tLineSeam.StartPointType &&
				0 < tLineSeam.EndPointType)
			{
				//���޸�
				XUI::MesBox::PopInfo("˫�����ƽ������{0} ����С��{1:.1f},�Զ�ɾ����", nWeldNo, m_dShortSeamThreshold);
				//XiMessageBoxOk("˫�����ƽ������%d ����С��%.1lf,�Զ�ɾ����", nWeldNo, m_dShortSeamThreshold);
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

	Sleep(50); // ȷ������ WinExec("del_pic.bat", SW_SHOW);��
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
	m_vvtCowWeldSeamGroupAdjust = m_vvtWeldSeamGroup;
	SaveWeldSeamGroupInfo();
	return true;
}

bool GenericWeld::CalcMeasureTrack(int nGroupNo, std::vector<T_ROBOT_COORS>& vtMeasureCoord, std::vector<T_ANGLE_PULSE>& vtMeasurePulse, vector<int>& vnMeasureType, double& dExAxlePos, double& dSafeHeight)
{
	double dExAxlePos_y;

	T_ROBOT_COORS tThersholdCoord(10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 0.0, 0.0, 0.0); // ��ͬ�������ж���ֵ
	vtMeasureCoord.clear(); // ת����������ߺ�
	vtMeasurePulse.clear();
	vnMeasureType.clear();
	m_vtTeachData.clear();
	m_vnTeachTrackOrder.clear();
	m_vtTeachResult.clear();
	T_TEACH_DATA tTeachData;
	vector<T_TEACH_DATA> vtTeachData;
	vector<LineOrCircularArcWeldingLine> vtSeamGroup = m_vvtWeldSeamGroup[nGroupNo];

	//double dTotalValueY = 0.0; // ͳ��Yֵ ȷ���ⲿ��λ��
	CvPoint3D64f tTotalValue = {0}; // ͳ��XYZֵ ȷ���ⲿ��λ��
	CvPoint3D64f tTotalDirValue = {0};
	double dPartMaxHeight = -99999.0; // ������¼������ߵ�Zֵ ���Ҽ�¼������͵�
	double dPartMinHeight = 99999.0; // ������¼������׵�Zֵ ���Ҽ�¼������ߵ�
	for (int nWeldSeamNo = 0; nWeldSeamNo < vtSeamGroup.size(); nWeldSeamNo++)
	{
		tTotalValue.x += (vtSeamGroup[nWeldSeamNo].StartPoint.x);
		tTotalValue.x += (vtSeamGroup[nWeldSeamNo].EndPoint.x);
		tTotalValue.y += (vtSeamGroup[nWeldSeamNo].StartPoint.y);
		tTotalValue.y += (vtSeamGroup[nWeldSeamNo].EndPoint.y);
		tTotalValue.z += (vtSeamGroup[nWeldSeamNo].StartPoint.z);
		tTotalValue.z += (vtSeamGroup[nWeldSeamNo].EndPoint.z);

		tTotalDirValue.x += vtSeamGroup[nWeldSeamNo].StartNormalVector.x;
		tTotalDirValue.x += vtSeamGroup[nWeldSeamNo].EndNormalVector.x;
		tTotalDirValue.y += vtSeamGroup[nWeldSeamNo].StartNormalVector.y;
		tTotalDirValue.y += vtSeamGroup[nWeldSeamNo].EndNormalVector.y;

		dPartMaxHeight = dPartMaxHeight < vtSeamGroup[nWeldSeamNo].StartPoint.z * m_nRobotInstallDir ? vtSeamGroup[nWeldSeamNo].StartPoint.z * m_nRobotInstallDir : dPartMaxHeight;
		dPartMaxHeight = dPartMaxHeight < vtSeamGroup[nWeldSeamNo].EndPoint.z * m_nRobotInstallDir ? vtSeamGroup[nWeldSeamNo].EndPoint.z * m_nRobotInstallDir : dPartMaxHeight;
		dPartMaxHeight = dPartMaxHeight < vtSeamGroup[nWeldSeamNo].ZSide * m_nRobotInstallDir ? vtSeamGroup[nWeldSeamNo].ZSide * m_nRobotInstallDir : dPartMaxHeight;

		dPartMinHeight = dPartMinHeight > vtSeamGroup[nWeldSeamNo].StartPoint.z * m_nRobotInstallDir ? vtSeamGroup[nWeldSeamNo].StartPoint.z * m_nRobotInstallDir : dPartMinHeight;
		dPartMinHeight = dPartMinHeight > vtSeamGroup[nWeldSeamNo].EndPoint.z * m_nRobotInstallDir ? vtSeamGroup[nWeldSeamNo].EndPoint.z * m_nRobotInstallDir : dPartMinHeight;
		dPartMinHeight = dPartMinHeight > vtSeamGroup[nWeldSeamNo].ZSide * m_nRobotInstallDir ? vtSeamGroup[nWeldSeamNo].ZSide * m_nRobotInstallDir : dPartMinHeight;
	}
	dPartMaxHeight *= m_nRobotInstallDir;
	dPartMinHeight *= m_nRobotInstallDir;
#ifdef SINGLE_ROBOT
	dExAxlePos = tTotalValue.y / (vtSeamGroup.size() * 2); // �ⲿ��λ��:ע����ⲿ��λ�ÿ��ܱ������������
#else
	dExAxlePos = tTotalValue.x / (vtSeamGroup.size() * 2); // �ⲿ��λ��:ע����ⲿ��λ�ÿ��ܱ������������
	dExAxlePos_y = 0.0;
#endif // SINGLE_ROBOT

	// ��ȡ��������

	// ���������麸������xֵ �ܺ�
	double dAllWeldTotalX = 0.0;   ///С����û�� С������
	int nAllWeldNum = 0;
	for (int nGroup = 0; nGroup < m_vvtWeldSeamGroup.size(); nGroup++)
	{
		vector<LineOrCircularArcWeldingLine> vtGroup = m_vvtWeldSeamGroup[nGroup];
		for (int nWeld = 0; nWeld < vtGroup.size(); nWeld++)
		{
			nAllWeldNum++;
			dAllWeldTotalX += vtGroup[nWeld].StartPoint.x;
			dAllWeldTotalX += vtGroup[nWeld].EndPoint.x;
		}
	}
	tTotalValue.x = dAllWeldTotalX * vtSeamGroup.size() / nAllWeldNum;
	
	if (g_bGantryEnableMark)
	{
		// ��ȡ��������
		CHECK_BOOL_RETURN(GetGantryPos(vtSeamGroup, dExAxlePos, dPartMaxHeight,
			m_vvtWeldLineInfoGroup[nGroupNo].at(0).tAtrribute.dGroupTrackPos, dPartMinHeight, tTotalValue, tTotalDirValue));
	}

	// ����ʾ�������ȵ�������
	vector<T_TEACH_DATA> vtSearchTeachData;
	vtSearchTeachData.clear();
	std::vector<T_ROBOT_COORS> vtMeasureCoordGunTool(0); // �ϲ���ĺ�ǹ��������
 	for (int nSeamNo = 0; nSeamNo < vtSeamGroup.size(); nSeamNo++)
	{
		vtTeachData.clear();
		//��������� CalcSeamTeachData ��������ͨ��ƽ��������յ�ĳ�������Ӷ��ٸ�������
 		CHECK_BOOL_RETURN(CalcSeamTeachData(nGroupNo, nSeamNo, dExAxlePos, vtSeamGroup, vtTeachData));
		//SaveTeachDataEverySeam(nGroupNo, nSeamNo, dExAxlePos, vtTeachData); // ����ʹ��		
		for (int nNewTeachPtnNo = 0; nNewTeachPtnNo < vtTeachData.size(); nNewTeachPtnNo++) // ��¼ÿ����������Ϣ
		{
			tTeachData = vtTeachData[nNewTeachPtnNo];
			// �����˵��ȵ�������
			if (E_SEARCH_POINT & tTeachData.nMeasureType)
			{
				vtSearchTeachData.push_back(tTeachData);
				continue;
			}
			int nIdx = CheckTeachCoord(tTeachData.tMeasureCoordGunTool, vtMeasureCoordGunTool, tThersholdCoord);
			// �����ڸò������� �� ������������������ �� �������ĵ�nIdx������������ ����²�������
			if ((-1 == nIdx))
			{
				tTeachData.nMeasurePtnNo = vtMeasureCoord.size();
				vtMeasureCoordGunTool.push_back(tTeachData.tMeasureCoordGunTool);
				//vtMeasureCoord.push_back(tTeachData.tMeasureCoordCamTool);
				vtMeasureCoord.push_back(tTeachData.tMeasureCoordGunTool); // ��ǹ���� �������������켣��ת�������
				vnMeasureType.push_back(tTeachData.nMeasureType);
			}
			else  // ���ڸò�������
			{
				tTeachData.nMeasurePtnNo = nIdx;
				tTeachData.tMeasureCoordCamTool = vtMeasureCoord[nIdx];
			}
			m_vtTeachData.push_back(tTeachData);
		}


		
	}
 	int m_vtTeachDataSize = m_vtTeachData.size();
	int nMeasureTypeSize = vnMeasureType.size();
	// ����������������
	for (int nNo = 0; nNo < vtSearchTeachData.size(); nNo++)
	{
		tTeachData = vtSearchTeachData[nNo];
		tTeachData.nMeasurePtnNo = vtMeasureCoord.size();
		vtMeasureCoordGunTool.push_back(tTeachData.tMeasureCoordGunTool);
		//vtMeasureCoord.push_back(tTeachData.tMeasureCoordCamTool);
		vtMeasureCoord.push_back(tTeachData.tMeasureCoordGunTool); // ��ǹ���� �������������켣��ת�������
		vnMeasureType.push_back(tTeachData.nMeasureType);
		m_vtTeachData.push_back(tTeachData);
	}

	// ��������켣��ǹ��������
	CString sFileName;
	sFileName.Format("%s%d-MeasureTrackGunTool.txt", m_sDataSavePath, nGroupNo);
	FILE* pf = fopen(sFileName.GetBuffer(), "w");
	for (int i = 0; i < m_vtTeachData.size(); i++)
	{
#ifdef SINGLE_ROBOT
		fprintf(pf, "%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%4d%4d " + GetTeachDataString(m_vtTeachData[i]) + "\n",
			m_vtTeachData[i].tMeasureCoordGunTool.dX, m_vtTeachData[i].tMeasureCoordGunTool.dY + dExAxlePos, m_vtTeachData[i].tMeasureCoordGunTool.dZ,
			m_vtTeachData[i].tMeasureCoordGunTool.dX, m_vtTeachData[i].tMeasureCoordGunTool.dY, m_vtTeachData[i].tMeasureCoordGunTool.dZ,
			m_vtTeachData[i].tMeasureCoordGunTool.dRX, m_vtTeachData[i].tMeasureCoordGunTool.dRY, m_vtTeachData[i].tMeasureCoordGunTool.dRZ,
			dExAxlePos, m_vtTeachData[i].nWeldNo, m_vtTeachData[i].nMeasurePtnNo);
#else
		fprintf(pf, "%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%4d%4d " + GetTeachDataString(m_vtTeachData[i]) + "\n",
			m_vtTeachData[i].tMeasureCoordGunTool.dX + dExAxlePos, m_vtTeachData[i].tMeasureCoordGunTool.dY, m_vtTeachData[i].tMeasureCoordGunTool.dZ,
			m_vtTeachData[i].tMeasureCoordGunTool.dX, m_vtTeachData[i].tMeasureCoordGunTool.dY, m_vtTeachData[i].tMeasureCoordGunTool.dZ,
			m_vtTeachData[i].tMeasureCoordGunTool.dRX, m_vtTeachData[i].tMeasureCoordGunTool.dRY, m_vtTeachData[i].tMeasureCoordGunTool.dRZ,
			dExAxlePos, m_vtTeachData[i].nWeldNo, m_vtTeachData[i].nMeasurePtnNo);
#endif // SINGLE_ROBOT
	}
	fclose(pf);


	// �ⲿ��λ��ƫ�� : 0.0  ��100.0  ��200.0  ��300.0 ����
	vector<double> vdExAxleOffset;
	vector<double> vdExAxleOffset_y;
	vdExAxleOffset.clear();
	vdExAxleOffset.push_back(0.0);
	vdExAxleOffset_y.clear();
	vdExAxleOffset_y.push_back(0.0);
		for (int i = 1; i < 10; i++)
	{
		vdExAxleOffset.push_back(100.0 * i);
		//vdExAxleOffset.push_back(-100.0 * i);
	}
	for (int i = 1; i < 10; i++)
	{
		//vdExAxleOffset.push_back(100.0 * i);
		vdExAxleOffset.push_back(-100.0 * i);
	}
	if (-1 != m_ptUnit->m_nMeasureAxisNo_up)
	{
		for (int i = 1; i < 10; i++)
		{
			vdExAxleOffset_y.push_back(-140.0 * i);
		}

		for (int i = 1; i < 10; i++)
		{
			vdExAxleOffset_y.push_back(22.0 * i);
		}
	}
	int continuePC = 0;
	bool bCalcSuccess = false;
	for (int nOffsetIdx = 0; (false == bCalcSuccess) && (nOffsetIdx < vdExAxleOffset.size()); nOffsetIdx++)
	{
		for (int nOffsetIdx_y = 0; (false == bCalcSuccess) && (nOffsetIdx_y < vdExAxleOffset_y.size()); nOffsetIdx_y++)
		{
			double dTempExAxlePos = dExAxlePos;
			double dTempExAxlePos_y = 0.0;
			std::vector<T_ROBOT_COORS> vtTempCoord(vtMeasureCoord);
			std::vector<T_ANGLE_PULSE> vtTempPulse(vtMeasurePulse);
			vector<int> vnTempType(vnMeasureType);

			double dMaxExAxlePos = (double)m_ptUnit->GetRobotCtrl()->m_tExternalAxle[m_ptUnit->m_nMeasureAxisNo - 1].lMaxPulseNum * m_ptUnit->GetRobotCtrl()->m_tExternalAxle[m_ptUnit->m_nMeasureAxisNo - 1].dPulse;
			double dMinExAxlePos = (double)m_ptUnit->GetRobotCtrl()->m_tExternalAxle[m_ptUnit->m_nMeasureAxisNo - 1].lMinPulseNum * m_ptUnit->GetRobotCtrl()->m_tExternalAxle[m_ptUnit->m_nMeasureAxisNo - 1].dPulse;

			double dMaxExAxlePos_y = (double)m_ptUnit->GetRobotCtrl()->m_tExternalAxle[m_ptUnit->m_nMeasureAxisNo_up - 1].lMaxPulseNum * m_ptUnit->GetRobotCtrl()->m_tExternalAxle[m_ptUnit->m_nMeasureAxisNo_up - 1].dPulse;
			double dMinExAxlePos_y = (double)m_ptUnit->GetRobotCtrl()->m_tExternalAxle[m_ptUnit->m_nMeasureAxisNo_up - 1].lMinPulseNum * m_ptUnit->GetRobotCtrl()->m_tExternalAxle[m_ptUnit->m_nMeasureAxisNo_up - 1].dPulse;
			if (vdExAxleOffset[nOffsetIdx] + dTempExAxlePos > dMaxExAxlePos || vdExAxleOffset[nOffsetIdx] + dTempExAxlePos < dMinExAxlePos)
			{
				WriteLog("�ⲿ��%d Ŀ��λ��%.3lf �����趨����%.3lf - %.3lf!", m_ptUnit->m_nMeasureAxisNo, vdExAxleOffset[nOffsetIdx], dMinExAxlePos, dMaxExAxlePos);
				continue;
			}

			if (-1 != m_ptUnit->m_nMeasureAxisNo_up)
			{
				if (vdExAxleOffset_y[nOffsetIdx_y] + dTempExAxlePos_y > dMaxExAxlePos_y || vdExAxleOffset_y[nOffsetIdx_y] + dTempExAxlePos_y < dMinExAxlePos_y)
				{
					WriteLog("�ⲿ��%d Ŀ��λ��%.3lf �����趨����%.3lf - %.3lf!", m_ptUnit->m_nMeasureAxisNo_up, vdExAxleOffset_y[nOffsetIdx_y], dMinExAxlePos_y, dMaxExAxlePos_y);
					continue;
				}
			}

			sFileName.Format("%s%d_Offset%.0lf_SortMeasureCoordBefore.txt", m_sDataSavePath, nGroupNo, vdExAxleOffset[nOffsetIdx]);
			//SaveCoordToFile(vtTempCoord, sFileName);
			if (-1 != m_ptUnit->m_nMeasureAxisNo_up)
			{
				if (!WeldingLineIsArc(vtSeamGroup[0]) && false == SortMeasureCoordNew_y(dPartMaxHeight, 0.0,
					vdExAxleOffset_y[nOffsetIdx_y], dTempExAxlePos_y, vtTempCoord, vnTempType, m_vnTeachTrackOrder))
				{
					WriteLog("ExAxleOffset_y:%3lf SortMeasureCoord Fail!", vdExAxleOffset[nOffsetIdx]);
					continue;
				}
			}
			if (!WeldingLineIsArc(vtSeamGroup[0]) && false == SortMeasureCoordNew(dPartMaxHeight, 0.0,
				vdExAxleOffset[nOffsetIdx], dTempExAxlePos, vtTempCoord, vnTempType, m_vnTeachTrackOrder))
			{
				WriteLog("ExAxleOffset:%3lf SortMeasureCoord Fail!", vdExAxleOffset[nOffsetIdx]);
				continue;
			}
			else if (WeldingLineIsArc(vtSeamGroup[0]) && false == SortMeasureCoordForArc(dPartMaxHeight, 0.0,
				vdExAxleOffset[nOffsetIdx], dTempExAxlePos, vtTempCoord, vnTempType, m_vnTeachTrackOrder))
			{
				WriteLog("ExAxleOffset:%3lf SortMeasureCoordForArc Fail!", vdExAxleOffset[nOffsetIdx]);
				continue;
			}

			// ���������������Ұ���ϼ��
			RTT::AdjustTrack t;
			std::vector<T_ROBOT_COORS_IN_WORLD> vtMeasureRobotPoints;

			vector<T_ROBOT_COORS> rawCoor;
			for (int num = 0; num < vtTempCoord.size(); num++)
			{
				T_ROBOT_COORS a;
				m_pRobotDriver->MoveToolByWeldGun(vtTempCoord[num], m_pRobotDriver->m_tTools.tCameraTool,
					vtTempCoord[num], m_pRobotDriver->m_tTools.tGunTool, a);
				rawCoor.push_back(a);
			}

			for (auto& c : rawCoor)
			{
				T_ROBOT_COORS_IN_WORLD rCoor;
				rCoor.dX = c.dX + c.dBX + dTempExAxlePos + vdExAxleOffset[nOffsetIdx];
				rCoor.dY = c.dY + c.dBY + dTempExAxlePos_y + vdExAxleOffset_y[nOffsetIdx_y];
				rCoor.dZ = c.dZ + c.dBZ;
				rCoor.dRX = c.dRX;
				rCoor.dRY = c.dRY;
				rCoor.dRZ = c.dRZ;
				vtMeasureRobotPoints.push_back(rCoor);
			}
			T_LINE_DIR_3D tWeldLineDir;
			XI_POINT startPoint, endPoint;
			startPoint.x = m_vvtWeldLineInfoGroup[nGroupNo][0].tWeldLine.StartPoint.x;
			startPoint.y = m_vvtWeldLineInfoGroup[nGroupNo][0].tWeldLine.StartPoint.y;
			startPoint.z = m_vvtWeldLineInfoGroup[nGroupNo][0].tWeldLine.StartPoint.z;

			endPoint.x = m_vvtWeldLineInfoGroup[nGroupNo][0].tWeldLine.EndPoint.x;
			endPoint.y = m_vvtWeldLineInfoGroup[nGroupNo][0].tWeldLine.EndPoint.y;
			endPoint.z = m_vvtWeldLineInfoGroup[nGroupNo][0].tWeldLine.EndPoint.z;


			T_SPACE_LINE_DIR lineDir = CalLineDir(startPoint, endPoint);

			tWeldLineDir.dDirX = lineDir.dDirX;
			tWeldLineDir.dDirY = lineDir.dDirY;
			tWeldLineDir.dDirZ = lineDir.dDirZ;

			vector<T_LINE_DIR_3D> vtWeldLineDir;

			GetWeldLineDir(rawCoor, vtWeldLineDir);

			E_CAM_ID eCamId = E_LEFT_ROBOT_BACKWARD_CAM;
			std::vector<T_XIGEO_WELDLINE> vtCheckBlockPlate;
			GetBlockPlate(vtCheckBlockPlate);
			std::vector<T_ROBOT_COORS_IN_WORLD> vtAdjustMeasureRobotPoints;

			t.AdjustMeasureCoors(*m_ptUnit, vtMeasureRobotPoints, vtWeldLineDir, eCamId, vtCheckBlockPlate, vtAdjustMeasureRobotPoints);

			vector<T_ROBOT_COORS> tmp = vtTempCoord;
			if (vtTempCoord.size() == vtAdjustMeasureRobotPoints.size())
			{
				for (int i = 0; i < vtAdjustMeasureRobotPoints.size(); i++)
				{
					T_ROBOT_COORS a;
					a.dX = vtAdjustMeasureRobotPoints[i].dX - dTempExAxlePos - vdExAxleOffset[nOffsetIdx];
					a.dY = vtAdjustMeasureRobotPoints[i].dY - dTempExAxlePos_y - vdExAxleOffset_y[nOffsetIdx_y];
					a.dZ = vtAdjustMeasureRobotPoints[i].dZ;
					a.dRX = vtAdjustMeasureRobotPoints[i].dRX;
					a.dRY = vtAdjustMeasureRobotPoints[i].dRY;
					a.dRZ = vtAdjustMeasureRobotPoints[i].dRZ;
					m_pRobotDriver->MoveToolByWeldGun(a, m_pRobotDriver->m_tTools.tGunTool,
						a, m_pRobotDriver->m_tTools.tCameraTool, a);
					if (!IsStandWeldSeam(vtSeamGroup[0].StartNormalVector))
					{
						vtTempCoord[i].dX = a.dX;
						vtTempCoord[i].dY = a.dY;
						vtTempCoord[i].dZ = a.dZ;
						vtTempCoord[i].dRX = a.dRX;
						vtTempCoord[i].dRY = a.dRY;
						vtTempCoord[i].dRZ = a.dRZ;
					}
				}
			}

			sFileName.Format("%s%d_Offset%.0lf_SortMeasureCoord.txt", m_sDataSavePath, nGroupNo, vdExAxleOffset[nOffsetIdx]);
			//SaveCoordToFile(vtTempCoord, sFileName);
			// �����ǹ ��ǹ����
			dSafeHeight = dPartMaxHeight + (m_dGunDownBackSafeDis * (double)m_nRobotInstallDir);
			if (false == AddSafeCoord(vtTempCoord, vnTempType, dPartMaxHeight, m_dGunDownBackSafeDis))
			{
				WriteLog("ExAxleOffset:%3lf AddSafeCoord Fail!", vdExAxleOffset[nOffsetIdx]);
				WriteLog("ExAxleOffset_y:%3lf AddSafeCoord Fail!", vdExAxleOffset_y[nOffsetIdx_y]);
				continue;
			}
			sFileName.Format("%s%d_Offset%.0lf_AddSafeCoord.txt", m_sDataSavePath, nGroupNo, vdExAxleOffset[nOffsetIdx]);
			//SaveCoordToFile(vtTempCoord, sFileName);
			// vtTempCoord -> vtTempPulse
			if (false == CalcContinuePulseForWeld(vtTempCoord, vtTempPulse, false))
			{
				WriteLog("ExAxleOffset:%3lf CalcContinuePulseForWeld Fail!", vdExAxleOffset[nOffsetIdx]);
				WriteLog("ExAxleOffset_y:%3lf CalcContinuePulseForWeld Fail!", vdExAxleOffset_y[nOffsetIdx_y]);
				continue;
			}
			if (false == CheckFlangeToolCoord(vtTempPulse)) // ������������켣�� ���뷨�������Ƿ��㹻��
			{
				WriteLog("ExAxleOffset:%3lf CalcContinuePulseForWeld Fail!", vdExAxleOffset[nOffsetIdx]);
				WriteLog("ExAxleOffset_y:%3lf CalcContinuePulseForWeld Fail!", vdExAxleOffset_y[nOffsetIdx_y]);
				continue;
			}

			// ������ʶ�������ɴ����ۺ�����Ϣ �� �����Ƿ������������
			// ��֤����ʱ�ͺ������ⲿ�ἰС��λ����ͬ (����ʱ�Կ��������ƶ�) �����켣�ͺ��ӹ켣��������������ʱ�ųɹ�
			if (false == JudgeTheoryTrackContinueMove(nGroupNo, dTempExAxlePos, dTempExAxlePos_y)) {
				WriteLog("ExAxleOffset:%3lf JudgeTheoryTrackContinueMove Fail!", vdExAxleOffset[nOffsetIdx]);
				WriteLog("ExAxleOffset_y:%3lf JudgeTheoryTrackContinueMove Fail!", vdExAxleOffset_y[nOffsetIdx_y]);
				continue;
			}
			continuePC++;
			if (continuePC >= 1)
			{
				bCalcSuccess = true;
			}
			else
			{
				continue;
			}
			dExAxlePos = dTempExAxlePos;
			dExAxlePos_y = dTempExAxlePos_y;
			vtMeasureCoord.clear();
			vtMeasurePulse.clear();
			vnMeasureType.clear();
			vtMeasureCoord.assign(vtTempCoord.begin(), vtTempCoord.end());
			vtMeasurePulse.assign(vtTempPulse.begin(), vtTempPulse.end());
			vnMeasureType.assign(vnTempType.begin(), vnTempType.end());

			// ����ɹ�����鵱ǰλ�úͲ����켣��һ������R���ֵ��������ֵ�����һ�����ɵ�
			if (false == m_bIsLocalDebug)
			{
				T_ANGLE_PULSE tCurAnglePulse = m_pRobotDriver->GetCurrentPulse();
				double dAngleErrAxisR = (double)(vtMeasurePulse[0].nRPulse - tCurAnglePulse.nRPulse) * m_pRobotDriver->m_tAxisUnit.dRPulse;
				if (fabs(dAngleErrAxisR) > 90.0)
				{
					T_ROBOT_COORS tSafeDownCoord;
					T_ANGLE_PULSE tSafeDownGunPulse(
						tCurAnglePulse.nSPulse, //(vtMeasurePulse[0].nSPulse - tCurAnglePulse.nSPulse) / 2
						m_pRobotDriver->m_tHomePulse.nLPulse,
						m_pRobotDriver->m_tHomePulse.nUPulse,
						tCurAnglePulse.nRPulse,
						0,
						(vtMeasurePulse[0].nTPulse - tCurAnglePulse.nTPulse) / 2, 0, 0, 0);
					m_pRobotDriver->RobotKinematics(tSafeDownGunPulse, m_pRobotDriver->m_tTools.tGunTool, tSafeDownCoord);
					vtMeasurePulse.insert(vtMeasurePulse.begin(), tSafeDownGunPulse);
					vtMeasureCoord.insert(vtMeasureCoord.begin(), tSafeDownCoord);
					vnMeasureType.insert(vnMeasureType.begin(), E_TRANSITION_POINT);
				}
			}
			for (int i = 0; i < vtMeasureCoord.size(); i++) // ���ⲿ������ͻ��������� ���
			{
#ifdef SINGLE_ROBOT
				vtMeasureCoord[i].dBY += dExAxlePos;
				vtMeasurePulse[i].lBYPulse = /*dExAxlePos*/vtMeasureCoord[i].dBY / m_pRobotDriver->m_tExternalAxle[1].dPulse;
#else
				vtMeasureCoord[i].dBX += dExAxlePos;
				vtMeasurePulse[i].lBXPulse = /*dExAxlePos*/vtMeasureCoord[i].dBX / m_pRobotDriver->m_tExternalAxle[0].dPulse;
				vtMeasureCoord[i].dBY += dExAxlePos_y;
				vtMeasurePulse[i].lBYPulse = /*dExAxlePos*/vtMeasureCoord[i].dBY / m_pRobotDriver->m_tExternalAxle[1].dPulse;
#endif // SINGLE_ROBOT
			}
			WriteLog("ExAxleOffset:%3lf �������������˶��켣�ɹ�", vdExAxleOffset[nOffsetIdx]);
		}
	}
	WriteLog("����������%d\n", continuePC);
	if (false == bCalcSuccess)
	{
		XiMessageBox("��λ�ü������������켣ʧ�ܣ�");
		return false;
	}


	// ����ʵ�ʲ������й켣 �ؽ����� ֱ������ �ⲿ�� 
	sFileName.Format("%s%d-MeasureTrackRealCoordPulseEx.txt", m_sDataSavePath, nGroupNo);
	pf = fopen(sFileName.GetBuffer(), "w");
	for (int i = 0; i < vtMeasureCoord.size(); i++)
	{
#ifdef SINGLE_ROBOT
		fprintf(pf, "%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%10d%10d%10d%10d%10d%10d%11.3lf\n",
			vtMeasureCoord[i].dX, vtMeasureCoord[i].dY + dExAxlePos, vtMeasureCoord[i].dZ,
			vtMeasureCoord[i].dRX, vtMeasureCoord[i].dRY, vtMeasureCoord[i].dRZ,
			vtMeasurePulse[i].nSPulse, vtMeasurePulse[i].nLPulse, vtMeasurePulse[i].nUPulse,
			vtMeasurePulse[i].nRPulse, vtMeasurePulse[i].nBPulse, vtMeasurePulse[i].nTPulse, dExAxlePos);
#else
		fprintf(pf, "%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%10d%10d%10d%10d%10d%10d%11.3lf\n",
			vtMeasureCoord[i].dX + dExAxlePos, vtMeasureCoord[i].dY + dExAxlePos_y, vtMeasureCoord[i].dZ,
			vtMeasureCoord[i].dRX, vtMeasureCoord[i].dRY, vtMeasureCoord[i].dRZ,
			vtMeasurePulse[i].nSPulse, vtMeasurePulse[i].nLPulse, vtMeasurePulse[i].nUPulse,
			vtMeasurePulse[i].nRPulse, vtMeasurePulse[i].nBPulse, vtMeasurePulse[i].nTPulse, dExAxlePos);
#endif // SINGLE_ROBOT
	}
	fclose(pf);

	// ���ߵ���ʹ��
	CString sJobName;
	sJobName.Format("MOVJTEACHBOARD%d", nGroupNo);
	GenerateJobLocalVariable(vtMeasurePulse, MOVJ, sJobName);
	//GenerateFilePLY(nGroupNo, dExAxlePos); // ����ʹ��
	return true;
}


void GenericWeld::GetWeldLineDir(std::vector<T_ROBOT_COORS> vtMeasureCoord, std::vector<T_LINE_DIR_3D>& vtWeldLineDir)
{
	T_LINE_DIR_3D tWeldLineDir;

	// ljx ��ʱ
	XI_POINT tStartPoint;
	XI_POINT tEndPoint;

	tStartPoint.x = 339.119;
	tStartPoint.y = -1507.650;
	tStartPoint.z = 96.397;


	tEndPoint.x = 340.092;
	tEndPoint.y = -1547.623;
	tEndPoint.z = 96.387;
	T_SPACE_LINE_DIR lineDir = CalLineDir(tStartPoint, tEndPoint);

	tWeldLineDir.dDirX = lineDir.dDirX;
	tWeldLineDir.dDirY = lineDir.dDirY;
	tWeldLineDir.dDirZ = lineDir.dDirZ;


	vector<int> angleNo;

	for (int i = 0; i < vtMeasureCoord.size(); i++)
	{
		vtWeldLineDir.push_back(tWeldLineDir);
		if (i < vtMeasureCoord.size() - 1)
		{
			if (vtMeasureCoord[i].dRZ == vtMeasureCoord[i + 1].dRZ)
			{
				
			}
		}
	}
	
	/*for (auto& plate : lineDir)
	{
		plateNo++;
		CString sFileName;
		sFileName.Format("Avoid\\BlockPlate\\CheckBlockPlate_%d.txt", plateNo);
		FILE* pf = fopen(sFileName, "w");
		fprintf(pf,
			"%11.3lf %11.3lf %11.3lf\n%11.3lf %11.3lf %11.3lf\n%11.3lf %11.3lf %11.3lf\n%11.3lf %11.3lf %11.3lf\n",
			plate.tDownStartPoint.x, plate.tDownStartPoint.y, plate.tDownStartPoint.z,
			plate.tDownEndPoint.x, plate.tDownEndPoint.y, plate.tDownEndPoint.z,
			plate.tUpStartPoint.x, plate.tUpStartPoint.y, plate.tUpStartPoint.z,
			plate.tUpEndPoint.x, plate.tUpEndPoint.y, plate.tUpEndPoint.z);
		fclose(pf);
	}*/
}

void GenericWeld::GetBlockPlate(std::vector<T_XIGEO_WELDLINE>& vtCheckBlockPlate)
{
	T_XIGEO_WELDLINE blockPlate;
	XiAlgorithm alg;
	for (int nGroupNo = 0; nGroupNo < m_vvtWeldLineInfoGroup.size(); nGroupNo++)
	{
		for (int nSeamNo = 0; nSeamNo < m_vvtWeldLineInfoGroup[nGroupNo].size(); nSeamNo++)
		{
			LineOrCircularArcWeldingLine tSeam = m_vvtWeldLineInfoGroup[nGroupNo][nSeamNo].tWeldLine;
			if (!IsStandWeldSeam(tSeam.StartNormalVector))
			{
				blockPlate.tDownEndPoint.x = tSeam.EndPoint.x;
				blockPlate.tDownEndPoint.y = tSeam.EndPoint.y;
				blockPlate.tDownEndPoint.z = tSeam.EndPoint.z;

				blockPlate.tDownStartPoint.x = tSeam.StartPoint.x;
				blockPlate.tDownStartPoint.y = tSeam.StartPoint.y;
				blockPlate.tDownStartPoint.z = tSeam.StartPoint.z;

				blockPlate.tUpEndPoint.x = tSeam.EndPoint.x;
				blockPlate.tUpEndPoint.y = tSeam.EndPoint.y;
				blockPlate.tUpEndPoint.z = tSeam.ZSide;

				blockPlate.tUpStartPoint.x = tSeam.StartPoint.x;
				blockPlate.tUpStartPoint.y = tSeam.StartPoint.y;
				blockPlate.tUpStartPoint.z = tSeam.ZSide;

				vtCheckBlockPlate.push_back(blockPlate);

				// ���
				double boardThick = 20.0;

				// �����
				double startAngleForBoard = RzToDirAngle(alg.CalcArcAngle(
					tSeam.StartNormalVector.x,
					tSeam.StartNormalVector.y)) - 180.0 * m_nRobotInstallDir;
				double endAngleForBoard = RzToDirAngle(alg.CalcArcAngle(
					tSeam.EndNormalVector.x,
					tSeam.EndNormalVector.y)) - 180.0 * m_nRobotInstallDir;

				// ��Ӱ��
				blockPlate.tDownEndPoint.x += boardThick * CosD(endAngleForBoard);
				blockPlate.tDownEndPoint.y += boardThick * SinD(endAngleForBoard);

				blockPlate.tDownStartPoint.x += boardThick * CosD(startAngleForBoard);
				blockPlate.tDownStartPoint.y += boardThick * SinD(startAngleForBoard);

				blockPlate.tUpEndPoint.x += boardThick * CosD(endAngleForBoard);
				blockPlate.tUpEndPoint.y += boardThick * SinD(endAngleForBoard);

				blockPlate.tUpStartPoint.x += boardThick * CosD(startAngleForBoard);
				blockPlate.tUpStartPoint.y += boardThick * SinD(startAngleForBoard);

				vtCheckBlockPlate.push_back(blockPlate);
			}
			else
			{
				blockPlate.tUpStartPoint.x = tSeam.EndPoint.x;
				blockPlate.tUpStartPoint.y = tSeam.EndPoint.y;
				blockPlate.tUpStartPoint.z = tSeam.EndPoint.z;

				blockPlate.tDownStartPoint.x = tSeam.StartPoint.x;
				blockPlate.tDownStartPoint.y = tSeam.StartPoint.y;
				blockPlate.tDownStartPoint.z = tSeam.StartPoint.z;

				XiAlgorithm alg;
				T_ROBOT_COORS tCoord;
				double dAdjoinSeam1Height = true == 
					tSeam.isLeft ? tSeam.ZSide : tSeam.EndPoint.z; // ZSide ����?
				double dAdjoinSeam2Height = true == 
					tSeam.isLeft ? tSeam.EndPoint.z : tSeam.ZSide;
				double dRobotChangeDir = 1 == m_nRobotInstallDir ? 1.0 : -1.0;
				double dAdjoinOffsetDis = 250.0;
				double dNorAngle = alg.CalcArcAngle(
					tSeam.StartNormalVector.x, 
					tSeam.StartNormalVector.y);
				CvPoint3D64f tPtn = tSeam.StartPoint;
				double dAdjoinSeamNorAngle = dNorAngle + (45.0 * dRobotChangeDir);	// ������rz
				double dOffsetDir = dNorAngle - (45.0 * dRobotChangeDir);			// ���������ƫ�ƽǶ�
				tCoord = GenerateRobotCoord(tPtn, m_dPlatWeldRx, m_dPlatWeldRy, DirAngleToRz(dAdjoinSeamNorAngle));
				RobotCoordPosOffset(tCoord, dOffsetDir, dAdjoinOffsetDis); // �յ������������ӵ�ƽ���ϲ���λ����������ƫ��

				blockPlate.tDownEndPoint.x = tCoord.dX;
				blockPlate.tDownEndPoint.y = tCoord.dY;
				blockPlate.tDownEndPoint.z = tCoord.dZ;

				blockPlate.tUpEndPoint.x = tCoord.dX;
				blockPlate.tUpEndPoint.y = tCoord.dY;
				blockPlate.tUpEndPoint.z = dAdjoinSeam1Height;

				vtCheckBlockPlate.push_back(blockPlate);

				// ���
				double boardThick = 20.0;

				// �����
				double startAngleForBoard = dOffsetDir - 90.0 * dRobotChangeDir;
				double endAngleForBoard = dOffsetDir - 90.0 * dRobotChangeDir;

				// ��Ӱ��
				blockPlate.tDownEndPoint.x += boardThick * CosD(endAngleForBoard);
				blockPlate.tDownEndPoint.y += boardThick * SinD(endAngleForBoard);

				blockPlate.tDownStartPoint.x += boardThick * CosD(startAngleForBoard);
				blockPlate.tDownStartPoint.y += boardThick * SinD(startAngleForBoard);

				blockPlate.tUpEndPoint.x += boardThick * CosD(endAngleForBoard);
				blockPlate.tUpEndPoint.y += boardThick * SinD(endAngleForBoard);

				blockPlate.tUpStartPoint.x += boardThick * CosD(startAngleForBoard);
				blockPlate.tUpStartPoint.y += boardThick * SinD(startAngleForBoard);

				vtCheckBlockPlate.push_back(blockPlate);

				blockPlate.tUpStartPoint.x = tSeam.EndPoint.x;
				blockPlate.tUpStartPoint.y = tSeam.EndPoint.y;
				blockPlate.tUpStartPoint.z = tSeam.EndPoint.z;

				blockPlate.tDownStartPoint.x = tSeam.StartPoint.x;
				blockPlate.tDownStartPoint.y = tSeam.StartPoint.y;
				blockPlate.tDownStartPoint.z = tSeam.StartPoint.z;

				tPtn = tSeam.StartPoint;
				dAdjoinSeamNorAngle = dNorAngle - (45.0 * dRobotChangeDir);
				dOffsetDir = dNorAngle + (45.0 * dRobotChangeDir);
				tCoord = GenerateRobotCoord(tPtn, m_dPlatWeldRx, m_dPlatWeldRy, DirAngleToRz(dAdjoinSeamNorAngle));
				RobotCoordPosOffset(tCoord, dOffsetDir, dAdjoinOffsetDis + m_dHandEyeDis); // ������������������ƽ���ϲ�������Ҫ�������ƫ�ƾ���


				blockPlate.tDownEndPoint.x = tCoord.dX;
				blockPlate.tDownEndPoint.y = tCoord.dY;
				blockPlate.tDownEndPoint.z = tCoord.dZ;

				blockPlate.tUpEndPoint.x = tCoord.dX;
				blockPlate.tUpEndPoint.y = tCoord.dY;
				blockPlate.tUpEndPoint.z = dAdjoinSeam2Height;

				vtCheckBlockPlate.push_back(blockPlate);

				// ���
				boardThick = 20.0;

				// �����
				startAngleForBoard = dOffsetDir - 90.0 * dRobotChangeDir;
				endAngleForBoard = dOffsetDir - 90.0 * dRobotChangeDir;

				// ��Ӱ��
				blockPlate.tDownEndPoint.x += boardThick * CosD(endAngleForBoard);
				blockPlate.tDownEndPoint.y += boardThick * SinD(endAngleForBoard);

				blockPlate.tDownStartPoint.x += boardThick * CosD(startAngleForBoard);
				blockPlate.tDownStartPoint.y += boardThick * SinD(startAngleForBoard);

				blockPlate.tUpEndPoint.x += boardThick * CosD(endAngleForBoard);
				blockPlate.tUpEndPoint.y += boardThick * SinD(endAngleForBoard);

				blockPlate.tUpStartPoint.x += boardThick * CosD(startAngleForBoard);
				blockPlate.tUpStartPoint.y += boardThick * SinD(startAngleForBoard);

				vtCheckBlockPlate.push_back(blockPlate);
			}
		}
	}
	int plateNo = 0;
	for (auto& plate : vtCheckBlockPlate)
	{
		plateNo++;
		CString sFileName;
		sFileName.Format("Avoid\\BlockPlate\\CheckBlockPlate_%d.txt", plateNo);
		FILE* pf = fopen(sFileName, "w");
		fprintf(pf, 
			"%11.3lf %11.3lf %11.3lf\n%11.3lf %11.3lf %11.3lf\n%11.3lf %11.3lf %11.3lf\n%11.3lf %11.3lf %11.3lf\n",
			plate.tDownStartPoint.x, plate.tDownStartPoint.y, plate.tDownStartPoint.z, 
			plate.tDownEndPoint.x, plate.tDownEndPoint.y, plate.tDownEndPoint.z,
			plate.tUpStartPoint.x, plate.tUpStartPoint.y, plate.tUpStartPoint.z,
			plate.tUpEndPoint.x, plate.tUpEndPoint.y, plate.tUpEndPoint.z);
		fclose(pf);
	}
}

bool GenericWeld::CalcWeldTrack(int nGroupNo)
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

	// ���ʾ�̽������
	int nTeachResultNum = m_vtTeachResult.size();
	if (nTeachResultNum != m_nTeachPtnNum)
	{
		XiMessageBox("ʾ�̵�������ͬ");
		return false;
	}
	// ��ʾ���й�������һ�º����������۹켣
	if (DetermineGroupMode(nGroupNo))
	{
		// ���ڴ��ڸ��ٺ���,��������麸��켣
		CHECK_BOOL_RETURN(CalcAndTrackWeldingTry(nGroupNo));
		return true;
	}
	// �� m_vtTeachResult �� m_vnTeachTrackOrder �ָ�˳��
	CHECK_BOOL_RETURN(RestoreOrderTeachResult());

	// ʹ�ò�����������������Ϣ(��ʶ�����ݶԱȣ��������ܺ���)
	CHECK_BOOL_RETURN(WeldSeamAdjust(nGroupNo));

	// ���ɺ��ӹ켣�ļ�
	CHECK_BOOL_RETURN(GenerateWeldLineData(nGroupNo, m_dWeldHoleSize, 0.0));

	return true;
}

bool GenericWeld::SortMeasureCoord(double dPartHeight, double dFirstSeamNorAngle, double dExAxleOffsetDis, 
	double &dExAxlePos, vector<T_ROBOT_COORS>& vtMeasureCoord, vector<int> &vnMeasureType, vector<int>& vtSortIdx)
{
	// ע�⣺����˳��Ϊ�������ź�����ʱ�뷽��
	//	����������������ϵ����ʱ�� Rz��С
	//  ���ң�����������ϵ��˳ʱ�� Rz����

	vtSortIdx.clear();
	int nCoordNum = vtMeasureCoord.size();
	int nTypeNum = vnMeasureType.size();
	if (nCoordNum != nTypeNum || nCoordNum < 1)
	{
		XiMessageBox("�����������Ͳ�������������ƥ�䣬����ʧ��");
		return false;
	}
	// ���������������ʾ�̲�����
	int nRzChangeDir = 1 == m_nRobotInstallDir ? -1 : 1; // -1:�����˳�����Rz��С  1:Rz����
	double dStartDirAngle = dFirstSeamNorAngle; // ��������ǶȾ������ĸ����쿪ʼ����
	double dEndDirAngle = dStartDirAngle + 360.0;
	double dHomePulseRz = 0.0;

	dExAxlePos += dExAxleOffsetDis; // ʹ���ⲿ��ƫ�� �޸��ⲿ������
	for (int nIdx = 0; nIdx < vtMeasureCoord.size(); nIdx++)
	{
		vtMeasureCoord[nIdx].dY -= dExAxleOffsetDis; // ʹ���ⲿ��ƫ�� �޸Ļ���������
		// Rzת���� dStartDirAngle - dEndDirAngle ��Χ�� ��������ʹ��
		vtMeasureCoord[nIdx].dRZ = fmod(vtMeasureCoord[nIdx].dRZ, 360.0);
		vtMeasureCoord[nIdx].dRZ = vtMeasureCoord[nIdx].dRZ > (dStartDirAngle) ? vtMeasureCoord[nIdx].dRZ - 360.0 : vtMeasureCoord[nIdx].dRZ;
		vtMeasureCoord[nIdx].dRZ = vtMeasureCoord[nIdx].dRZ < (dEndDirAngle) ? vtMeasureCoord[nIdx].dRZ + 360.0 : vtMeasureCoord[nIdx].dRZ;
	}

	// ��vtMeasureCoord����Rz�������� �� ��ȡ����� �������
	TempStruct tTempStruct;
	vector<TempStruct> vtTempStruct;
	vtTempStruct.clear();
	for (int i = 0; i < vtMeasureCoord.size(); i++)
	{
		tTempStruct.nIdx = i;
		tTempStruct.tCoord = vtMeasureCoord[i];
		vtTempStruct.push_back(tTempStruct);
	}
	sort(vtTempStruct.begin(), vtTempStruct.end(), [nRzChangeDir](const TempStruct& tTS1, const TempStruct& tTS2) -> bool
		{
			return (tTS1.tCoord.dRZ < tTS2.tCoord.dRZ) * nRzChangeDir > 0;
		});

	vector<int> vtTempType(vnMeasureType);
	for (int nIdx = 0; nIdx < vtTempStruct.size(); nIdx++)
	{
		int nTypeIdx = vtTempStruct[nIdx].nIdx;
		vtSortIdx.push_back(nTypeIdx);
		vtMeasureCoord[nIdx] = vtTempStruct[nIdx].tCoord;
		vnMeasureType[nIdx] = vtTempType[nTypeIdx];
	}

	vector<T_ROBOT_COORS> vtTempCoord(vtMeasureCoord);
	vector<int> vnTempType(vnMeasureType);
	vtMeasureCoord.clear();
	vnMeasureType.clear();
	vtMeasureCoord.push_back(vtTempCoord[0]);
	vnMeasureType.push_back(vnTempType[0]);
	T_ROBOT_COORS tPreCoord;
	T_ROBOT_COORS tCurCoord;
	for (int i = 1; i < vtTempCoord.size(); i++)
	{
		tPreCoord = vtTempCoord[i - 1];
		tCurCoord = vtTempCoord[i];
		if (fabs(tPreCoord.dRZ - tCurCoord.dRZ) > 100.0) // ���ڲ�����Rz����100��
		{
			T_ROBOT_COORS tTempCoord = tPreCoord;
			tTempCoord.dX = (tPreCoord.dX + tCurCoord.dX) / 2.0;
			tTempCoord.dY = (tPreCoord.dY + tCurCoord.dY) / 2.0;
			tTempCoord.dZ = dPartHeight + m_dGunDownBackSafeDis;// (tPreCoord.dZ + tCurCoord.dZ) / 2.0;
			tTempCoord.dRX = m_dPlatWeldRx;
			tTempCoord.dRY = m_dPlatWeldRy;
			tTempCoord.dRZ = (tPreCoord.dRZ + tCurCoord.dRZ) / 2.0;
			vtMeasureCoord.push_back(tTempCoord);
			vnMeasureType.push_back(E_TRANSITION_POINT);
		}
		vtMeasureCoord.push_back(vtTempCoord[i]);
		vnMeasureType.push_back(vnTempType[i]);
	}

	return true;
}

bool GenericWeld::SortMeasureCoordNew_y(double dPartHeight, double dFirstSeamNorAngle, double dExAxleOffsetDis, 
	double& dExAxlePos, vector<T_ROBOT_COORS>& vtMeasureCoord, vector<int>& vnMeasureType, vector<int>& vtSortIdx)
{
	// ע�⣺����˳��Ϊ�������ź�����ʱ�뷽��
	//	����������������ϵ����ʱ�� Rz��С
	//  ���ң�����������ϵ��˳ʱ�� Rz����
	vtSortIdx.clear();
	int nCoordNum = vtMeasureCoord.size();
	int nTypeNum = vnMeasureType.size();
	if (nCoordNum != nTypeNum || nCoordNum < 1)
	{
		XiMessageBox("�����������Ͳ�������������ƥ�䣬����ʧ��");
		return false;
	}
	// ���������������ʾ�̲�����
	double dRzChangeDir = 1 == m_nRobotInstallDir ? 1.0 : -1.0;

	// �����갴�շ����Ϊ����
	int nGroupNum = 4;
	vector<vector<TempStruct>> vvtCoord(nGroupNum, vector<TempStruct>(0));
	vector<TempStruct> vtTempStruct(0);
	dExAxlePos += dExAxleOffsetDis; // ʹ���ⲿ��ƫ�� �޸��ⲿ������
	for (int nIdx = 0; nIdx < vtMeasureCoord.size(); nIdx++)
	{
#ifdef SINGLE_ROBOT
		vtMeasureCoord[nIdx].dX -= dExAxleOffsetDis; // ʹ���ⲿ��ƫ�� �޸Ļ���������
#else
		vtMeasureCoord[nIdx].dY -= dExAxleOffsetDis; // ʹ���ⲿ��ƫ�� �޸Ļ���������
#endif // SINGLE_ROBOT

		//// ��ǹ���� ת �����������
		//if (false == m_pRobotDriver->MoveToolByWeldGun(vtMeasureCoord[nIdx], m_pRobotDriver->m_tTools.tGunTool,
		//	vtMeasureCoord[nIdx], m_pRobotDriver->m_tTools.tCameraTool, vtMeasureCoord[nIdx]))
		//{
		//	m_pRobotDriver->m_cLog->Write("Robot %d ��%d�������� ת��ʧ�ܣ�", m_pRobotDriver->m_nRobotNo, nIdx);
		//	return false;
		//}

		double dDirAngle = m_pRobotDriver->RzToDirAngle(vtMeasureCoord[nIdx].dRZ);
		for (int nAngleInx = 0; nAngleInx < nGroupNum; nAngleInx++) // �����������(Rz) ����
		{
			double dBaseDirAngle = dFirstSeamNorAngle - (nAngleInx * 90.0 * m_pRobotDriver->m_nRobotInstallDir); // �������� �尴�����С����
			if (true == JudgeDirAngle(dDirAngle, dBaseDirAngle, 45.0))
			{
				TempStruct tTempStruct;
				tTempStruct.nIdx = nIdx;
				tTempStruct.tCoord = vtMeasureCoord[nIdx];
				vvtCoord[nAngleInx].push_back(tTempStruct);
				break;
			}
		}
	}
	// ����������ϰ���������������򣬸���90������������У�ɾ������
	while (true)
	{
		int nFirstNullNo = -1;
		int nFirstGroupNo = -1;
		for (int i = 0; i < vvtCoord.size(); i++)
		{
			if ((0 == vvtCoord[i].size()) && (-1 == nFirstNullNo))
			{
				nFirstNullNo = i;
			}
			if ((0 != vvtCoord[i].size()) && (-1 == nFirstGroupNo))
			{
				nFirstGroupNo = i;
			}
			if ((-1 != nFirstNullNo) && (-1 != nFirstGroupNo))
			{
				break;
			}
		}
		if (-1 == nFirstNullNo) // �޿���
		{
			break;
		}
		else if (0 == nFirstNullNo) // ��һ���ǿ���
		{
			vvtCoord.erase(vvtCoord.begin());
		}
		else
		{
			vvtCoord.push_back(vvtCoord[0]);
			vvtCoord.erase(vvtCoord.begin());
		}
	}
	nGroupNum = vvtCoord.size();

	//// ��Ȧ������������
	//bool bOutsideWeld = false;
	//if (nGroupNum > 1)
	//{
	//	XiAlgorithm alg;
	//	double dDirAngle1 = RzToDirAngle(vvtCoord[0][0].tCoord.dRZ);
	//	T_ROBOT_COORS tCoord1;
	//	T_ROBOT_COORS tCoord2;
	//	T_ROBOT_COORS tCamTool = m_ptUnit->GetCameraParam(m_ptUnit->m_nMeasureCameraNo).tCameraTool;
	//	bool bRst = true;
	//	bRst &= m_pRobotDriver->MoveToolByWeldGun(vvtCoord[0][0].tCoord, tCamTool, vvtCoord[0][0].tCoord, m_pRobotDriver->m_tTools.tGunTool, tCoord1);
	//	bRst &= m_pRobotDriver->MoveToolByWeldGun(vvtCoord[1][0].tCoord, tCamTool, vvtCoord[1][0].tCoord, m_pRobotDriver->m_tTools.tGunTool, tCoord2);
	//	if (false == bRst)
	//	{
	//		XiMessageBoxOk("����ת��ʧ��bOutsideWeld");
	//		return false;
	//	}
	//	double dDirAngle2 = alg.CalcArcAngle(tCoord2.dX - tCoord1.dX, tCoord2.dY - tCoord1.dY);
	//	if (!JudgeDirAngle(dDirAngle1, dDirAngle2, 95.0)) // ��Ȧ���� ���ǶȲ�����ߵ�
	//	{
	//		bOutsideWeld = true;
	//		for (int i = 0; i < vvtCoord.size() / 2; i++)
	//		{
	//			swap(vvtCoord[i], vvtCoord[vvtCoord.size() - 1 - i]);
	//		}
	//	}
	//}

	//// ÿ��·������
	//for (int nGroupNo = 0; nGroupNo < nGroupNum; nGroupNo++)
	//{
	//	vector<TempStruct> vtCoord(vvtCoord[nGroupNo]);
	//	if (vtCoord.size() > 0)
	//	{
	//		double dDirAngle = m_pRobotDriver->RzToDirAngle(vtCoord[0].tCoord.dRZ);
	//		double dMoveDirAngle = dDirAngle + (90.0 * dRzChangeDir);
	//		XI_POINT tPtn;
	//		int nInstallDir = m_nRobotInstallDir;
	//		sort(vtCoord.begin(), vtCoord.end(), [nInstallDir](const TempStruct& t1, const TempStruct& t2) -> bool {
	//			return t1.tCoord.dZ * nInstallDir < t2.tCoord.dZ * nInstallDir;
	//			});
	//		tPtn.x = vtCoord[0].tCoord.dX + 999999.0 * CosD(dMoveDirAngle); // ������������
	//		tPtn.y = vtCoord[0].tCoord.dY + 999999.0 * SinD(dMoveDirAngle);
	//		tPtn.z = vtCoord[0].tCoord.dZ;
	//		sort(vtCoord.begin(), vtCoord.end(), [tPtn](const TempStruct& t1, const TempStruct& t2) -> bool {
	//			double dDis1 = TwoPointDis(t1.tCoord.dX, t1.tCoord.dY, t1.tCoord.dZ, tPtn.x, tPtn.y, tPtn.z);
	//			double dDis2 = TwoPointDis(t2.tCoord.dX, t2.tCoord.dY, t2.tCoord.dZ, tPtn.x, tPtn.y, tPtn.z);
	//			return dDis1 > dDis2; });

	//		vtTempStruct.push_back(vtCoord[0]);
	//		vtCoord.erase(vtCoord.begin());
	//		int nCoordNum = vtCoord.size();
	//		while (vtCoord.size() > 0)
	//		{
	//			int nMinDisIdx = -1;
	//			int nMinLevelDisIdx = -1;
	//			double dMinDis = 99999.0;
	//			double dMinLevelDis = 99999.0;
	//			double dCurDis = 0.0;
	//			double dCurLevelDis = 0.0;
	//			T_ROBOT_COORS tPreCoord = vtTempStruct[vtTempStruct.size() - 1].tCoord;
	//			nCoordNum = vtCoord.size();
	//			for (int i = 0; i < nCoordNum; i++)
	//			{
	//				dCurDis = TwoPointDis(tPreCoord.dX, tPreCoord.dY, tPreCoord.dZ, vtCoord[i].tCoord.dX, vtCoord[i].tCoord.dY, vtCoord[i].tCoord.dZ);
	//				dCurLevelDis = TwoPointDis(tPreCoord.dX, tPreCoord.dY, vtCoord[i].tCoord.dX, vtCoord[i].tCoord.dY);
	//				if (dCurDis < dMinDis)
	//				{
	//					dMinDis = dCurDis;
	//					nMinDisIdx = i;
	//				}
	//				if (dCurLevelDis < dMinLevelDis && dCurLevelDis < 20.0) // ����̬������û�����Ϸ�����
	//				{
	//					dMinLevelDis = dCurLevelDis;
	//					nMinLevelDisIdx = i;
	//				}
	//			}
	//			if (nMinLevelDisIdx >= 0 && nMinLevelDisIdx < nCoordNum)
	//			{
	//				vtTempStruct.push_back(vtCoord[nMinLevelDisIdx]);
	//				vtCoord.erase(vtCoord.begin() + nMinLevelDisIdx);
	//			}
	//			else if (nMinDisIdx >= 0 && nMinDisIdx < nCoordNum)
	//			{
	//				vtTempStruct.push_back(vtCoord[nMinDisIdx]);
	//				vtCoord.erase(vtCoord.begin() + nMinDisIdx);
	//			}
	//		}
	//	}
	//}

	//vector<int> vtTempType(vnMeasureType);
	//for (int nIdx = 0; nIdx < vtTempStruct.size(); nIdx++)
	//{
	//	int nTypeIdx = vtTempStruct[nIdx].nIdx;
	//	vtSortIdx.push_back(nTypeIdx);
	//	vtMeasureCoord[nIdx] = vtTempStruct[nIdx].tCoord;
	//	vnMeasureType[nIdx] = vtTempType[nTypeIdx];
	//}

	//// ���ڲ���λ����̬�仯������ӹ��ɵ�
	//vector<T_ROBOT_COORS> vtTempCoord(vtMeasureCoord);
	//vector<int> vnTempType(vnMeasureType);
	//vtMeasureCoord.clear();
	//vnMeasureType.clear();
	//vtMeasureCoord.push_back(vtTempCoord[0]);
	//vnMeasureType.push_back(vnTempType[0]);
	//T_ROBOT_COORS tPreCoord;
	//T_ROBOT_COORS tCurCoord;
	//for (int i = 1; i < vtTempCoord.size(); i++)
	//{
	//	tPreCoord = vtTempCoord[i - 1];
	//	tCurCoord = vtTempCoord[i];
	//	double dDis = TwoPointDis(
	//		tPreCoord.dX + tPreCoord.dBX, tPreCoord.dY + tPreCoord.dBY,
	//		tCurCoord.dX + tCurCoord.dBX, tCurCoord.dY + tCurCoord.dBY);
	//	if ((true == bOutsideWeld) &&
	//		(false == JudgeAngle(180.0, -180.0, tPreCoord.dRZ, tCurCoord.dRZ, 45.0))) // ��Ȧ���� Rz�仯����45��
	//	{
	//		T_ROBOT_COORS tTempCoord = tPreCoord;
	//		tTempCoord.dX = (tPreCoord.dX + tCurCoord.dX) / 2.0;
	//		tTempCoord.dY = (tPreCoord.dY + tCurCoord.dY) / 2.0;
	//		tTempCoord.dRX = m_dPlatWeldRx;
	//		tTempCoord.dRY = m_dPlatWeldRy;
	//		tTempCoord.dRZ = TwoAngleMedian(180.0, -180.0, tPreCoord.dRZ, tCurCoord.dRZ);
	//		RobotCoordPosOffset(tTempCoord, RzToDirAngle(tTempCoord.dRZ), m_dMeasureDisThreshold * 2.0);
	//		vtMeasureCoord.push_back(tTempCoord);
	//		vnMeasureType.push_back(E_TRANSITION_POINT);
	//	}
	//	else if (false == JudgeAngle(180.0, -180.0, tPreCoord.dRZ, tCurCoord.dRZ, 100.0) ||
	//		(dDis > 200.0))// ���ڲ�����Rz����100��
	//	{
	//		T_ROBOT_COORS tTempCoord = tPreCoord;
	//		tTempCoord.dX = (tPreCoord.dX + tCurCoord.dX) / 2.0;
	//		tTempCoord.dY = (tPreCoord.dY + tCurCoord.dY) / 2.0;
	//		tTempCoord.dZ = dPartHeight + (m_dGunDownBackSafeDis * m_nRobotInstallDir); // (tPreCoord.dZ + tCurCoord.dZ) / 2.0;
	//		tTempCoord.dRX = m_dPlatWeldRx;
	//		tTempCoord.dRY = m_dPlatWeldRy;
	//		tTempCoord.dRZ = TwoAngleMedian(180.0, -180.0, tPreCoord.dRZ, tCurCoord.dRZ);
	//		vtMeasureCoord.push_back(tTempCoord);
	//		vnMeasureType.push_back(E_TRANSITION_POINT);
	//	}
	//	vtMeasureCoord.push_back(vtTempCoord[i]);
	//	vnMeasureType.push_back(vnTempType[i]);
	//}

	return true;
}

bool GenericWeld::SortMeasureCoordNew(double dPartHeight, double dFirstSeamNorAngle, double dExAxleOffsetDis, 
	double& dExAxlePos, vector<T_ROBOT_COORS>& vtMeasureCoord, vector<int>& vnMeasureType, vector<int>& vtSortIdx)
{
	// ע�⣺����˳��Ϊ�������ź�����ʱ�뷽��
	//	����������������ϵ����ʱ�� Rz��С
	//  ���ң�����������ϵ��˳ʱ�� Rz����
	vtSortIdx.clear();
	int nCoordNum = vtMeasureCoord.size();
	int nTypeNum = vnMeasureType.size();
	if (nCoordNum != nTypeNum || nCoordNum < 1)
	{
		XiMessageBox("�����������Ͳ�������������ƥ�䣬����ʧ��");
		return false;
	}
	// ���������������ʾ�̲�����
	double dRzChangeDir = 1 == m_nRobotInstallDir ? 1.0 : -1.0;

	// �����갴�շ����Ϊ����
	int nGroupNum = 4;
	vector<vector<TempStruct>> vvtCoord(nGroupNum, vector<TempStruct>(0));
	vector<TempStruct> vtTempStruct(0);
	dExAxlePos += dExAxleOffsetDis; // ʹ���ⲿ��ƫ�� �޸��ⲿ������
	for (int nIdx = 0; nIdx < vtMeasureCoord.size(); nIdx++)
	{
#ifdef SINGLE_ROBOT
		vtMeasureCoord[nIdx].dY -= dExAxleOffsetDis; // ʹ���ⲿ��ƫ�� �޸Ļ���������
#else
		vtMeasureCoord[nIdx].dX -= dExAxleOffsetDis; // ʹ���ⲿ��ƫ�� �޸Ļ���������
#endif // SINGLE_ROBOT

		// ��ǹ���� ת �����������
		if (false == m_pRobotDriver->MoveToolByWeldGun(vtMeasureCoord[nIdx], m_pRobotDriver->m_tTools.tGunTool,
			vtMeasureCoord[nIdx], m_pRobotDriver->m_tTools.tCameraTool, vtMeasureCoord[nIdx]))
		{
			m_pRobotDriver->m_cLog->Write("Robot %d ��%d�������� ת��ʧ�ܣ�", m_pRobotDriver->m_nRobotNo, nIdx);
			return false;
		}

		double dDirAngle = m_pRobotDriver->RzToDirAngle(vtMeasureCoord[nIdx].dRZ);
		for (int nAngleInx = 0; nAngleInx < nGroupNum; nAngleInx++) // �����������(Rz) ����
		{
			double dBaseDirAngle = dFirstSeamNorAngle - (nAngleInx * 90.0 * m_pRobotDriver->m_nRobotInstallDir); // �������� �尴�����С����
			if (true == JudgeDirAngle(dDirAngle, dBaseDirAngle, 45.0))
			{
				TempStruct tTempStruct;
				tTempStruct.nIdx = nIdx;
				tTempStruct.tCoord = vtMeasureCoord[nIdx];
				vvtCoord[nAngleInx].push_back(tTempStruct);
				break;
			}
		}
	}
	// ����������ϰ���������������򣬸���90������������У�ɾ������
	while (true)
	{
		int nFirstNullNo = -1;
		int nFirstGroupNo = -1;
		for (int i = 0; i < vvtCoord.size(); i++)
		{
			if ((0 == vvtCoord[i].size()) && (-1 == nFirstNullNo))
			{
				nFirstNullNo = i;
			}
			if ((0 != vvtCoord[i].size()) && (-1 == nFirstGroupNo))
			{
				nFirstGroupNo = i;
			}
			if ((-1 != nFirstNullNo) && (-1 != nFirstGroupNo))
			{
				break;
			}
		}
		if (-1 == nFirstNullNo) // �޿���
		{
			break;
		}
		else if (0 == nFirstNullNo) // ��һ���ǿ���
		{
			vvtCoord.erase(vvtCoord.begin());
		}
		else
		{
			vvtCoord.push_back(vvtCoord[0]);
			vvtCoord.erase(vvtCoord.begin());
		}
	}
	nGroupNum = vvtCoord.size();

	// ��Ȧ������������
	bool bOutsideWeld = false;
	if (nGroupNum > 1)
	{
		XiAlgorithm alg;
		double dDirAngle1 = RzToDirAngle(vvtCoord[0][0].tCoord.dRZ);
		T_ROBOT_COORS tCoord1;
		T_ROBOT_COORS tCoord2;
		T_ROBOT_COORS tCamTool = m_ptUnit->GetCameraParam(m_ptUnit->m_nMeasureCameraNo).tCameraTool;
		bool bRst = true;
		bRst &= m_pRobotDriver->MoveToolByWeldGun(vvtCoord[0][0].tCoord, tCamTool, vvtCoord[0][0].tCoord, m_pRobotDriver->m_tTools.tGunTool, tCoord1);
		bRst &= m_pRobotDriver->MoveToolByWeldGun(vvtCoord[1][0].tCoord, tCamTool, vvtCoord[1][0].tCoord, m_pRobotDriver->m_tTools.tGunTool, tCoord2);
		if (false == bRst)
		{
			XiMessageBoxOk("����ת��ʧ��bOutsideWeld");
			return false;
		}
		double dDirAngle2 = alg.CalcArcAngle(tCoord2.dX - tCoord1.dX, tCoord2.dY - tCoord1.dY);
		if (!JudgeDirAngle(dDirAngle1, dDirAngle2, 95.0)) // ��Ȧ���� ���ǶȲ�����ߵ�
		{
			bOutsideWeld = true;
			for (int i = 0; i < vvtCoord.size() / 2; i++)
			{
				swap(vvtCoord[i], vvtCoord[vvtCoord.size() - 1 - i]);
			}
		}
	}

	// ÿ��·������
	for (int nGroupNo = 0; nGroupNo < nGroupNum; nGroupNo++)
	{
		vector<TempStruct> vtCoord(vvtCoord[nGroupNo]);
		if (vtCoord.size() > 0)
		{
			double dDirAngle = m_pRobotDriver->RzToDirAngle(vtCoord[0].tCoord.dRZ);
			double dMoveDirAngle = dDirAngle + (90.0 * dRzChangeDir);
			XI_POINT tPtn;
			int nInstallDir = m_nRobotInstallDir;
			sort(vtCoord.begin(), vtCoord.end(), [nInstallDir](const TempStruct& t1, const TempStruct& t2) -> bool {
				return t1.tCoord.dZ * nInstallDir < t2.tCoord.dZ * nInstallDir;
				});
			tPtn.x = vtCoord[0].tCoord.dX + 999999.0 * CosD(dMoveDirAngle); // ������������
			tPtn.y = vtCoord[0].tCoord.dY + 999999.0 * SinD(dMoveDirAngle);
			tPtn.z = vtCoord[0].tCoord.dZ;
			sort(vtCoord.begin(), vtCoord.end(), [tPtn](const TempStruct& t1, const TempStruct& t2) -> bool {
				double dDis1 = TwoPointDis(t1.tCoord.dX, t1.tCoord.dY, t1.tCoord.dZ, tPtn.x, tPtn.y, tPtn.z);
				double dDis2 = TwoPointDis(t2.tCoord.dX, t2.tCoord.dY, t2.tCoord.dZ, tPtn.x, tPtn.y, tPtn.z);
				return dDis1 > dDis2; });

			vtTempStruct.push_back(vtCoord[0]);
			vtCoord.erase(vtCoord.begin());
			int nCoordNum = vtCoord.size();
			while (vtCoord.size() > 0)
			{
				int nMinDisIdx = -1;
				int nMinLevelDisIdx = -1;
				double dMinDis = 99999.0;
				double dMinLevelDis = 99999.0;
				double dCurDis = 0.0;
				double dCurLevelDis = 0.0;
				T_ROBOT_COORS tPreCoord = vtTempStruct[vtTempStruct.size() - 1].tCoord;
				nCoordNum = vtCoord.size();
				for (int i = 0; i < nCoordNum; i++)
				{
					dCurDis = TwoPointDis(tPreCoord.dX, tPreCoord.dY, tPreCoord.dZ, vtCoord[i].tCoord.dX, vtCoord[i].tCoord.dY, vtCoord[i].tCoord.dZ);
					dCurLevelDis = TwoPointDis(tPreCoord.dX, tPreCoord.dY, vtCoord[i].tCoord.dX, vtCoord[i].tCoord.dY);
					if (dCurDis < dMinDis)
					{
						dMinDis = dCurDis;
						nMinDisIdx = i;
					}
					if (dCurLevelDis < dMinLevelDis && dCurLevelDis < 20.0) // ����̬������û�����Ϸ�����
					{
						dMinLevelDis = dCurLevelDis; 
						nMinLevelDisIdx = i;
					}
				}
				if (nMinLevelDisIdx >= 0 && nMinLevelDisIdx < nCoordNum)
				{
					vtTempStruct.push_back(vtCoord[nMinLevelDisIdx]);
					vtCoord.erase(vtCoord.begin() + nMinLevelDisIdx);
				}
				else if (nMinDisIdx >= 0 && nMinDisIdx < nCoordNum)
				{
					vtTempStruct.push_back(vtCoord[nMinDisIdx]);
					vtCoord.erase(vtCoord.begin() + nMinDisIdx);
				}
			}
		}
	}

	vector<int> vtTempType(vnMeasureType);
	for (int nIdx = 0; nIdx < vtTempStruct.size(); nIdx++)
	{
		int nTypeIdx = vtTempStruct[nIdx].nIdx;
		vtSortIdx.push_back(nTypeIdx);
		vtMeasureCoord[nIdx] = vtTempStruct[nIdx].tCoord;
		vnMeasureType[nIdx] = vtTempType[nTypeIdx];
	}

	// ���ڲ���λ����̬�仯������ӹ��ɵ�
	vector<T_ROBOT_COORS> vtTempCoord(vtMeasureCoord);
	vector<int> vnTempType(vnMeasureType);
	vtMeasureCoord.clear();
	vnMeasureType.clear();
	vtMeasureCoord.push_back(vtTempCoord[0]);
	vnMeasureType.push_back(vnTempType[0]);
	T_ROBOT_COORS tPreCoord;
	T_ROBOT_COORS tCurCoord;
	for (int i = 1; i < vtTempCoord.size(); i++)
	{
		tPreCoord = vtTempCoord[i - 1];
		tCurCoord = vtTempCoord[i];
		double dDis = TwoPointDis(
			tPreCoord.dX + tPreCoord.dBX, tPreCoord.dY + tPreCoord.dBY,
			tCurCoord.dX + tCurCoord.dBX, tCurCoord.dY + tCurCoord.dBY);
		if ((true == bOutsideWeld) && 
			(false == JudgeAngle(180.0, -180.0, tPreCoord.dRZ, tCurCoord.dRZ, 45.0))) // ��Ȧ���� Rz�仯����45��
		{
			T_ROBOT_COORS tTempCoord = tPreCoord;
			tTempCoord.dX = (tPreCoord.dX + tCurCoord.dX) / 2.0;
			tTempCoord.dY = (tPreCoord.dY + tCurCoord.dY) / 2.0;
			tTempCoord.dRX = m_dPlatWeldRx;
			tTempCoord.dRY = m_dPlatWeldRy;
			tTempCoord.dRZ = TwoAngleMedian(180.0, -180.0, tPreCoord.dRZ, tCurCoord.dRZ);
			RobotCoordPosOffset(tTempCoord, RzToDirAngle(tTempCoord.dRZ), m_dMeasureDisThreshold * 2.0);
			vtMeasureCoord.push_back(tTempCoord);
			vnMeasureType.push_back(E_TRANSITION_POINT);
		}
		else if (false == JudgeAngle(180.0, -180.0, tPreCoord.dRZ, tCurCoord.dRZ, 100.0) ||
			(dDis > 200.0))// ���ڲ�����Rz����100��
		{
			T_ROBOT_COORS tTempCoord = tPreCoord;
			tTempCoord.dX = (tPreCoord.dX + tCurCoord.dX) / 2.0;
			tTempCoord.dY = (tPreCoord.dY + tCurCoord.dY) / 2.0;
			tTempCoord.dZ = dPartHeight + (m_dGunDownBackSafeDis * m_nRobotInstallDir); // (tPreCoord.dZ + tCurCoord.dZ) / 2.0;
			tTempCoord.dRX = m_dPlatWeldRx;
			tTempCoord.dRY = m_dPlatWeldRy;
			tTempCoord.dRZ = TwoAngleMedian(180.0, -180.0, tPreCoord.dRZ, tCurCoord.dRZ);
			vtMeasureCoord.push_back(tTempCoord);
			vnMeasureType.push_back(E_TRANSITION_POINT);
		}
		vtMeasureCoord.push_back(vtTempCoord[i]);
		vnMeasureType.push_back(vnTempType[i]);
	}

	return true;
}

bool GenericWeld::SortMeasureCoordForArc(double dPartHeight, double dFirstSeamNorAngle, double dExAxleOffsetDis,
	double& dExAxlePos, vector<T_ROBOT_COORS>& vtMeasureCoord, vector<int>& vnMeasureType, vector<int>& vtSortIdx)
{
	vtSortIdx.clear();
	int nCoordNum = vtMeasureCoord.size();
	int nTypeNum = vnMeasureType.size();
	if (nCoordNum != nTypeNum || nCoordNum < 1)
	{
		XiMessageBox("�����������Ͳ�������������ƥ�䣬����ʧ��");
		return false;
	}

	dExAxlePos += dExAxleOffsetDis; // ʹ���ⲿ��ƫ�� �޸��ⲿ������
	for (int nIdx = 0; nIdx < vtMeasureCoord.size(); nIdx++)
	{
		vtSortIdx.push_back(nIdx);
#ifdef SINGLE_ROBOT
		vtMeasureCoord[nIdx].dY -= dExAxleOffsetDis; // ʹ���ⲿ��ƫ�� �޸Ļ���������
#else
		vtMeasureCoord[nIdx].dX -= dExAxleOffsetDis; // ʹ���ⲿ��ƫ�� �޸Ļ���������
#endif // SINGLE_ROBOT
													 // ��ǹ���� ת �����������
		if (false == m_pRobotDriver->MoveToolByWeldGun(vtMeasureCoord[nIdx], m_pRobotDriver->m_tTools.tGunTool,
			vtMeasureCoord[nIdx], m_pRobotDriver->m_tTools.tCameraTool, vtMeasureCoord[nIdx]))
		{
			m_pRobotDriver->m_cLog->Write("Robot %d ��%d�������� ת��ʧ�ܣ�", m_pRobotDriver->m_nRobotNo, nIdx);
			return false;
		}
	}
	return true;
}

bool GenericWeld::RestoreOrderTeachResult()
{
	int nTeachResultNum = m_vtTeachResult.size();
	int nOrderIdxNum = m_vnTeachTrackOrder.size();
	if (nTeachResultNum != nOrderIdxNum)
	{
		XUI::MesBox::PopOkCancel("��������Ͳ���λ������������{0} {1}", nTeachResultNum, nOrderIdxNum);
		return false;
	}
	vector<T_TEACH_RESULT> vtTeachResult(m_vtTeachResult);
	m_vtTeachResult.resize(nTeachResultNum);
	for (int nIdx = 0; nIdx < nTeachResultNum; nIdx++)
	{
		m_vtTeachResult[m_vnTeachTrackOrder[nIdx]] = vtTeachResult[nIdx];
	}
	return true;
}

bool GenericWeld::AddSafeCoord(vector<T_ROBOT_COORS>& vtMeasureCoord, vector<int>& vnMeasureType,
	double dMaxPartHeight, double dSafeDis)
{
	// 1��Ϊ��һ�������������һ����ǹ���ɵ㵽��һ������ǰ 
	// 2��Ϊ���һ�������������һ����ǹ���ɵ㵽���һ�������
	// 3��Ϊÿ��������������һ�����ɵ�

	int nFirstIdx = 9999;
	int nLastIdx = -1;
	vector<int> vnSearchPtnIdx(0);
	int nSrcPtnNum = vtMeasureCoord.size();
	// ���� ��һ���������� ���� ���һ�������������� ��ÿ������������
	for (int i = 0; i < vnMeasureType.size(); i++)
	{
		if ((i < nFirstIdx) && !(E_SEARCH_POINT & vnMeasureType[i]))
		{
			nFirstIdx = i;
		}
		if ((i > nLastIdx) && !(E_SEARCH_POINT & vnMeasureType[i]))
		{
			nLastIdx = i;
		}
		if (E_SEARCH_POINT & vnMeasureType[i])
		{
			vnSearchPtnIdx.push_back(i);
		}
	}

	T_ROBOT_COORS tFirstCoord;
	if (nFirstIdx >= 0 && nFirstIdx < vtMeasureCoord.size())
	{
		tFirstCoord = vtMeasureCoord[nFirstIdx];
	}
	T_ROBOT_COORS tMeasureCoord;
	double dOffsetDirAngle;
	// ���ʾ����ǹ���ɵ�
	if ((nLastIdx >= 0) && (nLastIdx < nSrcPtnNum))
	{
		tMeasureCoord = vtMeasureCoord[nLastIdx];
		dOffsetDirAngle = m_pRobotDriver->RzToDirAngle(tMeasureCoord.dRZ);
		tMeasureCoord.dX += (dSafeDis / 2.0 * CosD(dOffsetDirAngle));
		tMeasureCoord.dY += (dSafeDis / 2.0 * SinD(dOffsetDirAngle));
		tMeasureCoord.dZ = dMaxPartHeight + (dSafeDis * (double)m_nRobotInstallDir);
		//tMeasureCoord.dRX = m_dPlatWeldRx;
		//tMeasureCoord.dRY = m_dPlatWeldRy;
		vtMeasureCoord.push_back(tMeasureCoord); // ��ǹ
		vnMeasureType.push_back(E_TRANSITION_POINT);
	}

	// �������������ǹ���ɵ�
	for (int i = vnSearchPtnIdx.size() - 1; i >= 0; i--)
	{
		int nIdx = vnSearchPtnIdx[i];
		tMeasureCoord = vtMeasureCoord[nIdx];
		dOffsetDirAngle = m_pRobotDriver->RzToDirAngle(tMeasureCoord.dRZ);
		//tMeasureCoord.dX += (dSafeDis / 2.0 * CosD(dOffsetDirAngle));
		//tMeasureCoord.dY += (dSafeDis / 2.0 * SinD(dOffsetDirAngle));
		tMeasureCoord.dZ = dMaxPartHeight + (dSafeDis * (double)m_nRobotInstallDir);
		//tMeasureCoord.dRX = m_dPlatWeldRx;
		//tMeasureCoord.dRY = m_dPlatWeldRy;
		vtMeasureCoord.insert(vtMeasureCoord.begin() + nIdx + 1, tMeasureCoord);
		vnMeasureType.insert(vnMeasureType.begin() + nIdx + 1, E_TRANSITION_POINT);
	}

	// ���ʾ����ǹ���ɵ�
	if ((nFirstIdx >= 0) && (nFirstIdx < nSrcPtnNum))
	{
		tMeasureCoord = tFirstCoord;
		dOffsetDirAngle = m_pRobotDriver->RzToDirAngle(tMeasureCoord.dRZ);
		tMeasureCoord.dX += (dSafeDis / 2.0 * CosD(dOffsetDirAngle));
		tMeasureCoord.dY += (dSafeDis / 2.0 * SinD(dOffsetDirAngle));
		tMeasureCoord.dZ = dMaxPartHeight + (dSafeDis * (double)m_nRobotInstallDir);
		//tMeasureCoord.dRX = m_dPlatWeldRx;
		//tMeasureCoord.dRY = m_dPlatWeldRy;
		vtMeasureCoord.insert(vtMeasureCoord.begin(), tMeasureCoord); // ��ǹ
		vnMeasureType.insert(vnMeasureType.begin(), E_TRANSITION_POINT);
	}
	return true;
}

bool GenericWeld::CalcSeamMeasureCoord(LineOrCircularArcWeldingLine tSeam, double dOfficeDis, double dRobotChangeDir, vector<T_ROBOT_COORS>& vtMeasureCoord, LineOrCircularArcWeldingLine* ptAdjoinSeamS, LineOrCircularArcWeldingLine* ptAdjoinSeamE, bool bOffsetHandEyeDis/* = true*/)
{
	vtMeasureCoord.clear();
	double dMaxAcuteAngle = 80.0;
	double dMinAcuteAngle = 58.0;
	XiAlgorithm alg;
	T_ROBOT_COORS tCoord;
	double dNorAngleS = alg.CalcArcAngle(tSeam.StartNormalVector.x, tSeam.StartNormalVector.y);
	double dNorAngleE = alg.CalcArcAngle(tSeam.EndNormalVector.x, tSeam.EndNormalVector.y);
	double dOffsetDirS = dNorAngleS + (90.0 * dRobotChangeDir);
	double dOffsetDirE = dNorAngleE - (90.0 * dRobotChangeDir);
	double dSeamLen = TwoPointDis(
		tSeam.StartPoint.x, tSeam.StartPoint.y, tSeam.StartPoint.z,
		tSeam.EndPoint.x, tSeam.EndPoint.y, tSeam.EndPoint.z);
	double dEtoSDir = alg.CalcArcAngle(
		tSeam.StartPoint.x - tSeam.EndPoint.x,
		tSeam.StartPoint.y - tSeam.EndPoint.y);
	double dStoEDir = dEtoSDir + 180.0;
	if (NULL != ptAdjoinSeamS) // ������̬Rz��������ƽ���Ƕ� �������
	{
		double dAdjoinSeamDir = alg.CalcArcAngle(
			ptAdjoinSeamS->StartPoint.x - ptAdjoinSeamS->EndPoint.x,
			ptAdjoinSeamS->StartPoint.y - ptAdjoinSeamS->EndPoint.y);
		if (JudgeAngle(360.0, 0.0, dAdjoinSeamDir, dStoEDir, dMinAcuteAngle))
		{
			//���޸�
			XUI::MesBox::PopInfo("��������С��{0:.3f}�㣬�޷����ӣ�", dMinAcuteAngle);
			//XiMessageBox("��������С��%.3lf�㣬�޷����ӣ�", dMinAcuteAngle);
			return false;
		}
		if (JudgeAngle(360.0, 0.0, dAdjoinSeamDir, dStoEDir, dMaxAcuteAngle)) // ��Ǵ�������ǹRz�����ں��췽��Ϊ׼
		{
			dNorAngleS = dAdjoinSeamDir;
		}
	}
	if (NULL != ptAdjoinSeamE)
	{
		double dAdjoinSeamDir = alg.CalcArcAngle(
			ptAdjoinSeamE->EndPoint.x - ptAdjoinSeamE->StartPoint.x,
			ptAdjoinSeamE->EndPoint.y - ptAdjoinSeamE->StartPoint.y);
		if (JudgeAngle(360.0, 0.0, dAdjoinSeamDir, dEtoSDir, dMinAcuteAngle))
		{
			//���޸�
			XUI::MesBox::PopInfo("��������С��{0:.3f}�㣬�޷����ӣ�", dMinAcuteAngle);
			//XiMessageBox("��������С��%.3lf�㣬�޷����ӣ�", dMinAcuteAngle);
			return false;
		}
		if (JudgeAngle(360.0, 0.0, dAdjoinSeamDir, dEtoSDir, dMaxAcuteAngle)) // ��Ǵ�������ǹRz�����ں��췽��Ϊ׼
		{
			dNorAngleE = dAdjoinSeamDir;
			WriteLog("��������С��%.3lf�㣬�޷����ӣ�", dMinAcuteAngle);
		}
	}
	double dHandEyeDis = true == bOffsetHandEyeDis ? m_dHandEyeDis : 0.0;
	tCoord = GenerateRobotCoord(tSeam.StartPoint, m_dPlatWeldRx, m_dPlatWeldRy, DirAngleToRz(dNorAngleS));
	RobotCoordPosOffset(tCoord, dOffsetDirS, dOfficeDis + dHandEyeDis);
	CalcCorvrAngle(true, !tSeam.StartPointType, !tSeam.EndPointType, tCoord, dSeamLen);
	vtMeasureCoord.push_back(tCoord);

	tCoord = GenerateRobotCoord(tSeam.EndPoint, m_dPlatWeldRx, m_dPlatWeldRy, DirAngleToRz(dNorAngleE));
	RobotCoordPosOffset(tCoord, dOffsetDirE, dOfficeDis);
	CalcCorvrAngle(false, !tSeam.EndPointType, !tSeam.StartPointType, tCoord, dSeamLen);
	vtMeasureCoord.push_back(tCoord);
	return true;
}
bool GenericWeld::CalcSeamMeasureCoordFix(LineOrCircularArcWeldingLine tSeam, double dOfficeDis, double dRobotChangeDir, vector<T_ROBOT_COORS>& vtMeasureCoord, LineOrCircularArcWeldingLine* ptAdjoinSeamS, LineOrCircularArcWeldingLine* ptAdjoinSeamE,bool bIsStart, bool bOffsetHandEyeDis/* = true*/)
{
	vtMeasureCoord.clear();
	double dMaxAcuteAngle = 80.0;
	double dMinAcuteAngle = 58.0;
	XiAlgorithm alg;
	T_ROBOT_COORS tCoord;
	double dNorAngleS = alg.CalcArcAngle(tSeam.StartNormalVector.x, tSeam.StartNormalVector.y);
	double dNorAngleE = alg.CalcArcAngle(tSeam.EndNormalVector.x, tSeam.EndNormalVector.y);
	double dOffsetDirS = dNorAngleS + (90.0 * dRobotChangeDir);
	double dOffsetDirE = dNorAngleE - (90.0 * dRobotChangeDir);
	double dSeamLen = TwoPointDis(
		tSeam.StartPoint.x, tSeam.StartPoint.y, tSeam.StartPoint.z,
		tSeam.EndPoint.x, tSeam.EndPoint.y, tSeam.EndPoint.z);
#if 0
	double dEtoSDir = alg.CalcArcAngle(
		tSeam.StartPoint.x - tSeam.EndPoint.x,
		tSeam.StartPoint.y - tSeam.EndPoint.y);
	double dStoEDir = dEtoSDir + 180.0;
	if (NULL != ptAdjoinSeamS) // ������̬Rz��������ƽ���Ƕ� �������
	{
		double dAdjoinSeamDir = alg.CalcArcAngle(
			ptAdjoinSeamS->StartPoint.x - ptAdjoinSeamS->EndPoint.x,
			ptAdjoinSeamS->StartPoint.y - ptAdjoinSeamS->EndPoint.y);
		if (JudgeAngle(360.0, 0.0, dAdjoinSeamDir, dStoEDir, dMinAcuteAngle))
		{
			//���޸�
			XUI::MesBox::PopInfo("��������С��{0:.3f}�㣬�޷����ӣ�", dMinAcuteAngle);
			//XiMessageBox("��������С��%.3lf�㣬�޷����ӣ�", dMinAcuteAngle);
			return false;
		}
		if (JudgeAngle(360.0, 0.0, dAdjoinSeamDir, dStoEDir, dMaxAcuteAngle)) // ��Ǵ�������ǹRz�����ں��췽��Ϊ׼
		{
			dNorAngleS = dAdjoinSeamDir;
		}
	}
	if (NULL != ptAdjoinSeamE)
	{
		double dAdjoinSeamDir = alg.CalcArcAngle(
			ptAdjoinSeamE->EndPoint.x - ptAdjoinSeamE->StartPoint.x,
			ptAdjoinSeamE->EndPoint.y - ptAdjoinSeamE->StartPoint.y);
		if (JudgeAngle(360.0, 0.0, dAdjoinSeamDir, dEtoSDir, dMinAcuteAngle))
		{
			//���޸�
			XUI::MesBox::PopInfo("��������С��{0:.3f}�㣬�޷����ӣ�", dMinAcuteAngle);
			//XiMessageBox("��������С��%.3lf�㣬�޷����ӣ�", dMinAcuteAngle);
			return false;
		}
		if (JudgeAngle(360.0, 0.0, dAdjoinSeamDir, dEtoSDir, dMaxAcuteAngle)) // ��Ǵ�������ǹRz�����ں��췽��Ϊ׼
		{
			dNorAngleE = dAdjoinSeamDir;
			WriteLog("��������С��%.3lf�㣬�޷����ӣ�", dMinAcuteAngle);
		}
#endif // 0
	double dHandEyeDis = true == bOffsetHandEyeDis ? m_dHandEyeDis : 0.0;
	if (bIsStart)
	{
		// Բ����������㣬���Ż�
		tCoord = GenerateRobotCoord(tSeam.StartPoint, m_dPlatWeldRx, m_dPlatWeldRy, DirAngleToRz(dNorAngleS));
		RobotCoordPosOffset(tCoord, dOffsetDirS, dOfficeDis * 1.0 + dHandEyeDis);//��������
		CalcCorvrAngle(true, !tSeam.StartPointType, !tSeam.EndPointType, tCoord, dSeamLen);
		vtMeasureCoord.push_back(tCoord);

		tCoord = GenerateRobotCoord(tSeam.StartPoint, m_dPlatWeldRx, m_dPlatWeldRy, DirAngleToRz(dNorAngleS));
		RobotCoordPosOffset(tCoord, dOffsetDirS, -10.0 + dHandEyeDis);//��������
		CalcCorvrAngle(true, !tSeam.StartPointType, !tSeam.EndPointType, tCoord, dSeamLen);
		vtMeasureCoord.push_back(tCoord);
	}
	else
	{
		tCoord = GenerateRobotCoord(tSeam.EndPoint, m_dPlatWeldRx, m_dPlatWeldRy, DirAngleToRz(dNorAngleE));
		RobotCoordPosOffset(tCoord, dOffsetDirE, dOfficeDis * 1/*dOfficeDis * 4*/);
		CalcCorvrAngle(false, !tSeam.EndPointType, !tSeam.StartPointType, tCoord, dSeamLen);
		vtMeasureCoord.push_back(tCoord);

		tCoord = GenerateRobotCoord(tSeam.EndPoint, m_dPlatWeldRx, m_dPlatWeldRy, DirAngleToRz(dNorAngleE));
		RobotCoordPosOffset(tCoord, dOffsetDirE, /*dOfficeDis*/-10.0);//��������
		CalcCorvrAngle(false, !tSeam.EndPointType, !tSeam.StartPointType, tCoord, dSeamLen);
		vtMeasureCoord.push_back(tCoord);
		
	}
	return true;
}

bool GenericWeld::CalcSeamTeachData(int nGroupNo, int nSeamNo, double dExAxlePos, vector<LineOrCircularArcWeldingLine> vtSeamGroup, vector<T_TEACH_DATA>& vtTeachData)
{
	/*
	 * ��ͬ�˵������Ϊһ�¼�����ʽ��
	 * 1��ƽ�� ��� ���ɶ˵� (����������ǣ�)
	 * 2��ƽ�� ��� ����˵�
	 * 3��ƽ�� �յ� ���ɶ˵� (����������ǣ�)
	 * 4��ƽ�� �յ� ����˵�
	 * 5������ ��� ���ɶ˵� (���е��ų����� �ݲ�����)
	 * 6������ ��� ����˵�
	 * 7������ �յ� ����˵�
	 * 8�����Բ����
	 * 9��Բ�� ��� �յ� �м����
	 */

	//if (IsStandWeldSeam(vtSeamGroup[nSeamNo].StartNormalVector))
	//{
	//	double sZside = 0;
	//	double eZside = 0;
	//	double sMeasureDis = 0;
	//	double eMeasureDis = 0;


	//}

	if (10 == m_vvtWeldLineInfoGroup[nGroupNo][nSeamNo].tAtrribute.nStartWrapType) // ɨ��ֲ�������ȡ���죬��֧�ֵ�������
	{
		return CalcScanMeasureTeachData(nGroupNo, nSeamNo, dExAxlePos, vtSeamGroup, vtTeachData);
	}
	else if (DetermineGroupMode(nGroupNo) || WeldingLineIsArc(vtSeamGroup[nSeamNo])) // ԲȦ�Ȳ��,�պ�Բ�����٣��������߸���
	{
		return CalcSeamArcTeachData(nGroupNo, nSeamNo, dExAxlePos, vtSeamGroup, vtTeachData);
	}
	double dSearchDis = m_dEndpointSearchDis; // ���ɶ��������Ⱦ���
	vtTeachData.clear();
	XiAlgorithm alg;
	double dAngleChangeDir = 1 == m_nRobotInstallDir ? 1.0 : -1.0;///�������λ����������
	//T_TEACH_DATA tTeachData;
	vector<T_TEACH_DATA> vtTempTeachData;
	LineOrCircularArcWeldingLine tSeam = vtSeamGroup[nSeamNo];
	bool bIsFlatWeld = !IsStandWeldSeam(tSeam.StartNormalVector);  //trueΪƽ�� falseΪ����
	bool bFreeStartPoint = !tSeam.StartPointType;
	bool bFreeEndPoint = !tSeam.EndPointType;
	double dNorAngleS = alg.CalcArcAngle(tSeam.StartNormalVector.x, tSeam.StartNormalVector.y);
	double dNorAngleE = alg.CalcArcAngle(tSeam.EndNormalVector.x, tSeam.EndNormalVector.y);
	LineOrCircularArcWeldingLine *ptAdjoinSeamS = NULL; //&tAdjoinSeamS; // ƽ��������ڵ�ƽ���� �� ���������������ڵ�ƽ����
	LineOrCircularArcWeldingLine *ptAdjoinSeamE = NULL; //&tAdjoinSeamE; // ƽ���յ����ڵ�ƽ���� �� �յ�������������ڵ�ƽ����
	GetAdjoinSeam(nSeamNo, vtSeamGroup, &ptAdjoinSeamS, &ptAdjoinSeamE);


	double dShortSeamThreshold = m_dShortSeamThreshold; //��������յ�Ĳ������ƫ�ƾ���
	if (bIsFlatWeld) {
		CalcFlatSeamMeasurementEnhancement(nSeamNo, bIsFlatWeld, true, tSeam, dNorAngleS, dExAxlePos, ptAdjoinSeamS, vtTempTeachData);
	}
	AppendTeachData(vtTeachData, vtTempTeachData);
	// ��������������
	if (bIsFlatWeld && bFreeStartPoint)			// ��� ƽ�� ���ɶ˵�
	{
		CHECK_BOOL_RETURN(CalcFreeTeachDataFlat(nGroupNo, nSeamNo, bIsFlatWeld, true, tSeam, dNorAngleS, dSearchDis, dExAxlePos, vtTempTeachData));
	}
	else if (bIsFlatWeld && !bFreeStartPoint)	// ��� ƽ�� ����˵�
	{
		// ���� �����Ȳ�󺸸���λ�ȡ�˵㷽ʽ
		//CHECK_BOOL_RETURN(CalcInterfereTeachDataFlatFix(nSeamNo, bIsFlatWeld, true, vtSeamGroup, dNorAngleS, dExAxlePos, vtTempTeachData));
		// ԭ������
		CHECK_BOOL_RETURN(CalcInterfereTeachDataFlat(nSeamNo, bIsFlatWeld, true, vtSeamGroup, dNorAngleS, dExAxlePos, vtTempTeachData));
	}
	else if (!bIsFlatWeld && !bFreeStartPoint)	// ��� ���� ����˵�
	{
		CHECK_BOOL_RETURN(CalcTeachDataStand(nSeamNo, bIsFlatWeld, true, vtSeamGroup, dExAxlePos, vtTempTeachData));
	}
	else if (!bIsFlatWeld && bFreeStartPoint)	// ��� ���� ���ɶ˵�
	{
		CHECK_BOOL_RETURN(CalcTeachDataStandFreeStart(nSeamNo, bIsFlatWeld, true, tSeam, dExAxlePos, ptAdjoinSeamS, ptAdjoinSeamE, vtTempTeachData));
	}
	else 
	{
		//���޸�
		XUI::MesBox::PopInfo("������λ�ü���ʧ�ܣ���������{0} �յ�����{1}", bIsFlatWeld, bFreeStartPoint);
		//XiMessageBox("������λ�ü���ʧ�ܣ���������%d �������%d", bIsFlatWeld, bFreeStartPoint);
		return false;
	}
	AppendTeachData(vtTeachData, vtTempTeachData);

	// �����յ��������
	if (bIsFlatWeld && bFreeEndPoint)			// �յ� ƽ�� ���ɶ˵�
	{
		CHECK_BOOL_RETURN(CalcFreeTeachDataFlat(nGroupNo, nSeamNo, bIsFlatWeld, false, tSeam, dNorAngleE, dSearchDis, dExAxlePos, vtTempTeachData));
	}
	else if (bIsFlatWeld && !bFreeEndPoint)		// �յ� ƽ�� ����˵�
	{
		// ���� �����Ȳ�󺸸���λ�ȡ�˵㷽ʽ
		//CHECK_BOOL_RETURN(CalcInterfereTeachDataFlatFix(nSeamNo, bIsFlatWeld, false, vtSeamGroup, dNorAngleS, dExAxlePos, vtTempTeachData));
		// ԭ������
		CHECK_BOOL_RETURN(CalcInterfereTeachDataFlat(nSeamNo, bIsFlatWeld, false, vtSeamGroup, dNorAngleE, dExAxlePos, vtTempTeachData));
	}
	else if (!bIsFlatWeld && bFreeEndPoint)		// �յ� ���� ���ɶ˵�
	{
		CHECK_BOOL_RETURN(CalcTeachDataStand(nSeamNo, bIsFlatWeld, false, vtSeamGroup, dExAxlePos, vtTempTeachData));
	}
	else
	{
		//���޸�
		XUI::MesBox::PopInfo("�յ����λ�ü���ʧ�ܣ���������{0} �յ�����{1}", bIsFlatWeld, bFreeEndPoint);
		//XiMessageBox("������λ�ü���ʧ�ܣ���������%d �յ�����%d", bIsFlatWeld, bFreeEndPoint);
		return false;
	}
	AppendTeachData(vtTeachData, vtTempTeachData);

	//for (int i = 0; i < vtTeachData.size(); i++)
	//{
	//	vtTeachData[i].tMeasureCoordGunTool.dRY -= 10.0;
	//}

	//// ����ʹ��
	//CString sFileName;
	//sFileName.Format("%s%d-%d-MeasureTrackGunTool.txt", m_sDataSavePath, nGroupNo, nSeamNo);
	//FILE *CoutFile = fopen(sFileName.GetBuffer(0), "w");
	//for (int nNo = 0; nNo < vtTeachData.size(); nNo++)
	//{
	//	//SaveRobotCoor(CoutFile, vtTeachData[nNo].tMeasureCoordGunTool);
	//}
	//fclose(CoutFile);

	//// ��ǹ���� ת ����������� �ƶ����������������˶��켣ǰת��
	//for (int nNo = 0; nNo < vtTeachData.size(); nNo++)
	//{
	//	if (false == m_pRobotDriver->MoveToolByWeldGun(vtTeachData[nNo].tMeasureCoordGunTool, m_pRobotDriver->m_tTools.tGunTool,
	//		vtTeachData[nNo].tMeasureCoordGunTool, m_pRobotDriver->m_tTools.tCameraTool, vtTeachData[nNo].tMeasureCoordCamTool))
	//	{
	//		XiMessageBox("����%d ��%d�������� ת��ʧ�ܣ�", nSeamNo, nNo);
	//		return false;
	//	}
	//}
	return true;
}

bool GenericWeld::CalcScanMeasureTeachData(int nGroupNo, int nSeamNo, double dExAxlePos, vector<LineOrCircularArcWeldingLine> vtSeamGroup, vector<T_TEACH_DATA>& vtTeachData)
{
	vtTeachData.clear();
	XiAlgorithm alg;
	LineOrCircularArcWeldingLine tSeam = vtSeamGroup[nSeamNo];
	bool bIsFlatSeam = !IsStandWeldSeam(tSeam.StartNormalVector);
	double dNorAngle = alg.CalcArcAngle(tSeam.StartNormalVector.x, tSeam.StartNormalVector.y);

	T_ROBOT_COORS tCoord;
	T_TEACH_DATA tTeachData; // nMeasurePtnNo��tMeasureCoordCamTool�ⲿ�ϲ��������Ǹ�ֵ
	tTeachData.nWeldNo = nSeamNo;
	tTeachData.nMeasurePtnNo = 0;
	tTeachData.nMeasureType = E_DOUBLE_LONG_LINE | E_SEARCH_POINT | E_SCAN_POINT;
	tTeachData.eWeldSeamType = bIsFlatSeam ? E_FLAT_SEAM : E_STAND_SEAM;
	tTeachData.eAttribute = E_BELONG_START;
	tTeachData.eEndpointType = E_FREE_POINT;
	CvPoint3D64f tStartPtn = tSeam.StartPoint;
	CvPoint3D64f tEndPtn = tSeam.EndPoint;
	double dRobotChangeDir = 1 == m_nRobotInstallDir ? 1.0 : -1.0;

	double dScanRz = m_pRobotDriver->DirAngleToRz(dNorAngle) + m_dStandWeldScanOffsetRz;
	tCoord = GenerateRobotCoord(tStartPtn, m_dStandWeldScanRx, m_dStandWeldScanRy, dScanRz);
	//RobotCoordPosOffset(tCoord, dNorAngle, m_dEndpointSearchDis / 3.0, (-1.0) * m_dEndpointSearchDis / 3.0 * dRobotChangeDir);
	double dOffsetDis = m_dEndpointSearchDis / 2.0;
	if (0 == m_pRobotDriver->m_nRobotNo)
	{
		RobotCoordPosOffset(tCoord, dNorAngle, dOffsetDis, 0.0 * m_nRobotInstallDir);
	}
	else
	{
		RobotCoordPosOffset(tCoord, dNorAngle, dOffsetDis, 50.0 * m_nRobotInstallDir);
	}
	// ��ֱ����ƫ��
	RobotCoordPosOffset(tCoord, dNorAngle + 90.0, dOffsetDis, 0.0);
	tCoord.dRZ -= 25.0;
	DecomposeExAxle(tCoord, dExAxlePos);
	tTeachData.tMeasureCoordGunTool = tCoord;
	if (0 != m_pRobotDriver->m_nRobotNo)
	{
		tTeachData.tMeasureCoordGunTool.dRX = 0.0;
	}
	vtTeachData.push_back(tTeachData);

	// ��ֱ����ƫ��
	tCoord.dRZ += 50.0;
	RobotCoordPosOffset(tCoord, dNorAngle - 90.0, dOffsetDis * 2, 0.0);
	//tCoord.dZ = tEndPtn.z;
	//RobotCoordPosOffset(tCoord, dNorAngle, 0.0, m_dEndpointSearchDis * dRobotChangeDir);
	tTeachData.tMeasureCoordGunTool = tCoord;
	vtTeachData.push_back(tTeachData);

	/*for (int i = 0; i < vtTeachData.size(); i++)
	{
		if (!m_pRobotDriver->MoveToolByWeldGun(vtTeachData[i].tMeasureCoordGunTool, m_pRobotDriver->m_tTools.tGunTool,
			vtTeachData[i].tMeasureCoordGunTool, m_pRobotDriver->m_tTools.tCameraTool, vtTeachData[i].tMeasureCoordCamTool))
		{
			XiMessageBox("����������۹켣ת��ʧ��");
			return false;
		}
	}*/
	return true;
/*
	vtTeachData.clear();
	XiAlgorithm alg;
	LineOrCircularArcWeldingLine tSeam = vtSeamGroup[nSeamNo];
	bool bIsFlatSeam = !IsStandWeldSeam(tSeam.StartNormalVector);
	double dVectorDis = sqrt((double)(tSeam.StartNormalVector.x * tSeam.StartNormalVector.x + tSeam.StartNormalVector.y * tSeam.StartNormalVector.y + tSeam.StartNormalVector.z * tSeam.StartNormalVector.z));
	double dZDis = fabs(tSeam.StartNormalVector.z);
	double dAngle = asin(dZDis / dVectorDis) * 180.0 / PI;
	double dNorAngle = alg.CalcArcAngle(tSeam.StartNormalVector.x, tSeam.StartNormalVector.y);

	if (dAngle < 15)
	{
		T_ROBOT_COORS tCoord;
		T_TEACH_DATA tTeachData; // nMeasurePtnNo��tMeasureCoordCamTool�ⲿ�ϲ��������Ǹ�ֵ
		tTeachData.nWeldNo = nSeamNo;
		tTeachData.nMeasurePtnNo = 0;
		tTeachData.nMeasureType = E_DOUBLE_LONG_LINE | E_SEARCH_POINT | E_SCAN_POINT;
		tTeachData.eWeldSeamType = bIsFlatSeam ? E_FLAT_SEAM : E_STAND_SEAM;
		tTeachData.eAttribute = E_BELONG_START;
		tTeachData.eEndpointType = E_FREE_POINT;
		CvPoint3D64f tStartPtn = tSeam.StartPoint;
		CvPoint3D64f tEndPtn = tSeam.EndPoint;
		double dRobotChangeDir = 1 == m_nRobotInstallDir ? 1.0 : -1.0;

		double dScanRz = m_pRobotDriver->DirAngleToRz(dNorAngle) + m_dStandWeldScanOffsetRz;
		tCoord = GenerateRobotCoord(tStartPtn, m_dStandWeldScanRx, m_dStandWeldScanRy, dScanRz);
		//RobotCoordPosOffset(tCoord, dNorAngle, m_dEndpointSearchDis / 3.0, (-1.0) * m_dEndpointSearchDis / 3.0 * dRobotChangeDir);

		//����ƫ����Ӧ����һ�£�������������������
		//��������켣ƫ�ˣ�Ӧ�ȼ�����뺸���λ���ǲ��ǶԵģ������������޸�ƫ����
		double dOffsetDis = 50.0;
		if (0 == m_pRobotDriver->m_nRobotNo)
		{
			RobotCoordPosOffset(tCoord, dNorAngle, dOffsetDis, 70.0 * m_nRobotInstallDir);
		}
		else
		{
			RobotCoordPosOffset(tCoord, dNorAngle, dOffsetDis, 50.0 * m_nRobotInstallDir);
		}
		// ��ֱ����ƫ��
		RobotCoordPosOffset(tCoord, dNorAngle + 90.0, dOffsetDis, 0.0);
		tCoord.dRZ -= 50.0;
		DecomposeExAxle(tCoord, dExAxlePos);
		tTeachData.tMeasureCoordGunTool = tCoord;
		if (0 != m_pRobotDriver->m_nRobotNo)
		{
			tTeachData.tMeasureCoordGunTool.dRX = 0.0;
		}
		vtTeachData.push_back(tTeachData);

		// ��ֱ����ƫ��
		tCoord.dRZ += 70.0;
		RobotCoordPosOffset(tCoord, dNorAngle - 90.0, dOffsetDis * 2.0, 0.0);
		//tCoord.dZ = tEndPtn.z;
		//RobotCoordPosOffset(tCoord, dNorAngle, 0.0, m_dEndpointSearchDis * dRobotChangeDir);
		tTeachData.tMeasureCoordGunTool = tCoord;
		vtTeachData.push_back(tTeachData);
		//if (tSeam.StartNormalVector.y > 0)
		//{
		//	for (int i = 0;i < vtTeachData.size();i++)
		//	{
		//		vtTeachData[i].tMeasureCoordGunTool.dY -= 50;
		//	}
		//}

	}
	else//�����������
	{
		T_ROBOT_COORS tCoord;
		T_TEACH_DATA tTeachData; // nMeasurePtnNo��tMeasureCoordCamTool�ⲿ�ϲ��������Ǹ�ֵ
		tTeachData.nWeldNo = nSeamNo;
		tTeachData.nMeasurePtnNo = 0;
		tTeachData.nMeasureType = E_DOUBLE_LONG_LINE | E_SEARCH_POINT | E_SCAN_POINT;
		tTeachData.eWeldSeamType = bIsFlatSeam ? E_FLAT_SEAM : E_STAND_SEAM;
		tTeachData.eAttribute = E_BELONG_START;
		tTeachData.eEndpointType = E_FREE_POINT;
		CvPoint3D64f tStartPtn = tSeam.StartPoint;
		CvPoint3D64f tEndPtn = tSeam.EndPoint;
		double dRobotChangeDir = 1 == m_nRobotInstallDir ? 1.0 : -1.0;

		double dScanRz = m_pRobotDriver->DirAngleToRz(dNorAngle) + m_dStandWeldScanOffsetRz+50;
		if (tSeam.StartNormalVector.y > 0)
		{
			dScanRz = 108;
		}
		else
		{
			dScanRz = -78;
		}
		tCoord = GenerateRobotCoord(tStartPtn, m_dStandWeldScanRx, m_dStandWeldScanRy, dScanRz);
		//RobotCoordPosOffset(tCoord, dNorAngle, m_dEndpointSearchDis / 3.0, (-1.0) * m_dEndpointSearchDis / 3.0 * dRobotChangeDir);

		if (0 == m_pRobotDriver->m_nRobotNo)
		{
			RobotCoordPosOffset(tCoord, dNorAngle, 40.0, 70.0 * m_nRobotInstallDir);
		}
		else
		{
			RobotCoordPosOffset(tCoord, dNorAngle, 40.0, 50.0 * m_nRobotInstallDir);
		}
		// ��ֱ����ƫ��
		RobotCoordPosOffset(tCoord, dNorAngle + 90.0, m_dEndpointSearchDis / 2, 10.0);
		double a = CosD(dNorAngle + 90.0);
		tCoord.dX -= a * 70;
		double b = SinD(dNorAngle + 90.0);
		//tCoord.dRZ -= 50.0;
		DecomposeExAxle(tCoord, dExAxlePos);
		tTeachData.tMeasureCoordGunTool = tCoord;
		if (0 != m_pRobotDriver->m_nRobotNo)
		{
			tTeachData.tMeasureCoordGunTool.dRX = 0.0;
		}
		vtTeachData.push_back(tTeachData);

		// ��ֱ����ƫ��
		//tCoord.dRZ += 70.0;
		//tCoord = GenerateRobotCoord(tEndPtn, m_dStandWeldScanRx, m_dStandWeldScanRy, dScanRz);
		//RobotCoordPosOffset(tCoord, dNorAngle - 90.0, m_dEndpointSearchDis * 1.2, 10.0);
		//tCoord.dZ = tEndPtn.z;
		tCoord = GenerateRobotCoord(tEndPtn, m_dStandWeldScanRx, m_dStandWeldScanRy, dScanRz);
		if (tSeam.StartNormalVector.y < 0)
		{
			tCoord.dX -= a * 70;
		}
	//	tCoord.dX -= a * 20;
		//RobotCoordPosOffset(tCoord, dNorAngle, 0.0, m_dEndpointSearchDis * dRobotChangeDir);
		DecomposeExAxle(tCoord, dExAxlePos);
		tTeachData.tMeasureCoordGunTool = tCoord;
		vtTeachData.push_back(tTeachData);
	}
	//for (int i = 0; i < vtTeachData.size(); i++)
	//{
	//	if (!m_pRobotDriver->MoveToolByWeldGun(vtTeachData[i].tMeasureCoordGunTool, m_pRobotDriver->m_tTools.tGunTool,
	//		vtTeachData[i].tMeasureCoordGunTool, m_pRobotDriver->m_tTools.tCameraTool, vtTeachData[i].tMeasureCoordCamTool))
	//	{
	//		XiMessageBox("����������۹켣ת��ʧ��");
	//		return false;
	//	}
	//}
	return true;
	*/
}

//bool GenericWeld::CalcScanMeasureTeachData(int nGroupNo, int nSeamNo, double dExAxlePos, vector<LineOrCircularArcWeldingLine> vtSeamGroup, vector<T_TEACH_DATA>& vtTeachData)
//{
//	vtTeachData.clear();
//	XiAlgorithm alg;
//	LineOrCircularArcWeldingLine tSeam = vtSeamGroup[nSeamNo];
//	bool bIsFlatSeam = !IsStandWeldSeam(tSeam.StartNormalVector);
//	double dNorAngle = alg.CalcArcAngle(tSeam.StartNormalVector.x, tSeam.StartNormalVector.y);
//
//	T_ROBOT_COORS tCoord;
//	T_TEACH_DATA tTeachData; // nMeasurePtnNo��tMeasureCoordCamTool�ⲿ�ϲ��������Ǹ�ֵ
//	tTeachData.nWeldNo = nSeamNo;
//	tTeachData.nMeasurePtnNo = 0;
//	tTeachData.nMeasureType = E_DOUBLE_LONG_LINE | E_SEARCH_POINT | E_SCAN_POINT;
//	tTeachData.eWeldSeamType = bIsFlatSeam ? E_FLAT_SEAM : E_STAND_SEAM;
//	tTeachData.eAttribute = E_BELONG_START;
//	tTeachData.eEndpointType = E_FREE_POINT;
//	CvPoint3D64f tStartPtn = tSeam.StartPoint;
//	CvPoint3D64f tEndPtn = tSeam.EndPoint;
//	double dRobotChangeDir = 1 == m_nRobotInstallDir ? 1.0 : -1.0;
//
//	double dScanRz = m_pRobotDriver->DirAngleToRz(dNorAngle) + m_dStandWeldScanOffsetRz;
//	tCoord = GenerateRobotCoord(tStartPtn, m_dStandWeldScanRx, m_dStandWeldScanRy, dScanRz);
//	//RobotCoordPosOffset(tCoord, dNorAngle, m_dEndpointSearchDis / 3.0, (-1.0) * m_dEndpointSearchDis / 3.0 * dRobotChangeDir);
//	RobotCoordPosOffset(tCoord, dNorAngle, 0.0);
//	DecomposeExAxle(tCoord, dExAxlePos);
//	tTeachData.tMeasureCoordGunTool = tCoord;
//	vtTeachData.push_back(tTeachData);
//
//	tCoord.dZ = tEndPtn.z;
//	RobotCoordPosOffset(tCoord, dNorAngle, 0.0, m_dEndpointSearchDis * dRobotChangeDir);
//	tTeachData.tMeasureCoordGunTool = tCoord;
//	vtTeachData.push_back(tTeachData);
//
//	for (int i = 0; i < vtTeachData.size(); i++)
//	{
//		if (!m_pRobotDriver->MoveToolByWeldGun(vtTeachData[i].tMeasureCoordGunTool, m_pRobotDriver->m_tTools.tGunTool,
//			vtTeachData[i].tMeasureCoordGunTool, m_pRobotDriver->m_tTools.tCameraTool, vtTeachData[i].tMeasureCoordCamTool))
//		{
//			XiMessageBox("����������۹켣ת��ʧ��");
//			return false;
//		}
//	}
//	return true;
//}

bool GenericWeld::CalcSeamArcTeachData(int nGroupNo, int nSeamNo, double dExAxlePos, vector<LineOrCircularArcWeldingLine> vtSeamGroup, vector<T_TEACH_DATA>& vtTeachData)
{
	LineOrCircularArcWeldingLine tSeamLine = vtSeamGroup[nSeamNo];
	double dStartEndDis = TwoPointDis(
		tSeamLine.StartPoint.x, tSeamLine.StartPoint.y, tSeamLine.StartPoint.z,
		tSeamLine.EndPoint.x, tSeamLine.EndPoint.y, tSeamLine.EndPoint.z);
	if (dStartEndDis < 1.0 && !m_vvtWeldLineInfoGroup[nGroupNo][nSeamNo].tAtrribute.bWeldMode) // �պ�Բ�κ���
	{
		CHECK_BOOL_RETURN(CalcTeachDataCircle(nGroupNo, nSeamNo, dExAxlePos, vtSeamGroup, vtTeachData));
	}
	else // ���ɺ���
	{
		CHECK_BOOL_RETURN(CalcTeachDataFreeCurve(nGroupNo, nSeamNo, dExAxlePos, vtSeamGroup, vtTeachData));
	}
	for (int i = 0; i < vtTeachData.size(); i++)
	{
		if (!m_pRobotDriver->MoveToolByWeldGun(vtTeachData[i].tMeasureCoordGunTool, m_pRobotDriver->m_tTools.tGunTool,
			vtTeachData[i].tMeasureCoordGunTool, m_pRobotDriver->m_tTools.tCameraTool, vtTeachData[i].tMeasureCoordCamTool))
		{
			//XiMessageBox("Բ������%d ��%d�������� ת��ʧ�ܣ�", nSeamNo, i);
			//���޸�
			XUI::MesBox::PopInfo("Բ������{0} ��{1}�������� ת��ʧ�ܣ�", nSeamNo, i);
			return false;
		}
	}
	return true;
}

bool GenericWeld::CalcTeachDataCircle(int nGroupNo, int nSeamNo, double dExAxlePos, vector<LineOrCircularArcWeldingLine> vtSeamGroup, vector<T_TEACH_DATA>& vtTeachData)
{
	/*
	 * int nWeldNo;							// ������������������
	 * int nMeasurePtnNo;					// ��������(ͬһ������������ڼ���������˵�)
	 * int nMeasureType;					// ���������ͣ� �Ȳ��ʾ�� �������� �����˵� ����ͶӰ
	 * E_WELD_STAM_TYPE eWeldSeamType;		// ����������������ĺ�������: ƽ�� ���� ����
	 * E_ATTRIBUTE eAttribute;				// �����������ԣ����ڲ������ �յ� �򺸷��м�����
	 * E_ENDPOINT_TYPE eEndpointType;		// �������������˵�Ķ˵�����: ���ɶ˵� ����˵�
	 * T_ROBOT_COORS tMeasureCoordGunTool;	// ������λ��ֱ������ ��ǹ����
	 * T_ROBOT_COORS tMeasureCoordCamTool;	// ����λ��ֱ������ �������
	 */
	vtTeachData.clear();
	T_ROBOT_COORS tCoord;
	T_TEACH_DATA tTeachData; // nMeasurePtnNo��tMeasureCoordCamTool�ⲿ�ϲ��������Ǹ�ֵ
	tTeachData.nWeldNo = nSeamNo;
	tTeachData.nMeasurePtnNo = 0;
	tTeachData.nMeasureType = E_DOUBLE_LONG_LINE | E_TEACH_POINT;
	tTeachData.eWeldSeamType = E_FLAT_SEAM;
	tTeachData.eAttribute = E_BELONG_START;
	tTeachData.eEndpointType = E_FREE_POINT;

	// �������� ˳��ʱ�� ��ʼ��������� �����Ƕȼ��(�ȽǶȻ�Ⱦ���) 
	XiAlgorithm alg;
	LineOrCircularArcWeldingLine tSeamLine = vtSeamGroup[nSeamNo];
	double dRobotDir = 1 == m_nRobotInstallDir ? 1.0 : -1.0; 
	double dFirstDirAngle = m_dWeldNorAngleInHome + 180.0;
	double dCenterToStartDirAngle = alg.CalcArcAngle(tSeamLine.StartPoint.x - tSeamLine.CenterPoint.x, tSeamLine.StartPoint.y - tSeamLine.CenterPoint.y);
	double dStartDirAngle = alg.CalcArcAngle(tSeamLine.StartNormalVector.x, tSeamLine.StartNormalVector.y);
	bool bOutsideCircle = JudgeDirAngle(dCenterToStartDirAngle, dStartDirAngle, 45.0) ? true : false;
	double dStartOffsetDir = bOutsideCircle ? dFirstDirAngle : dFirstDirAngle + 180.0;
	double dRadius = TwoPointDis(tSeamLine.StartPoint.x, tSeamLine.StartPoint.y, tSeamLine.CenterPoint.x, tSeamLine.CenterPoint.y);
	int nMeasurePtnNum = 6;
	double dIntervalMinDis = 100.0;
	double dAngleInterval = 360.0 / nMeasurePtnNum;
	double dIntervalDis = 2.0 * PI * dRadius / nMeasurePtnNum;
	if (dIntervalDis > dIntervalMinDis)
	{
		nMeasurePtnNum = 2.0 * PI * dRadius / (double)dIntervalMinDis;
		dAngleInterval = 360.0 / nMeasurePtnNum;
	}

	for (int nNo = 0; nNo < nMeasurePtnNum; nNo++)
	{
		tCoord = GenerateRobotCoord(tSeamLine.CenterPoint, m_dPlatWeldRx, m_dPlatWeldRy, dFirstDirAngle + (nNo * dAngleInterval * dRobotDir));
		DecomposeExAxle(tCoord, dExAxlePos);
		RobotCoordPosOffset(tCoord, dStartOffsetDir + (nNo * dAngleInterval * dRobotDir), dRadius);
		tTeachData.tMeasureCoordGunTool = tCoord;
		vtTeachData.push_back(tTeachData);
	}

	return true;
}

bool GenericWeld::CalcTeachDataFreeCurve(int nGroupNo, int nSeamNo, double dExAxlePos, vector<LineOrCircularArcWeldingLine> vtSeamGroup, vector<T_TEACH_DATA>& vtTeachData)
{
	m_pTraceModel->m_tWorkPieceType = m_eWorkPieceType; // ��ʼ����������
	double dSearchDis = m_dEndpointSearchDis; // ���ɶ��������Ⱦ���
	vtTeachData.clear();
	XiAlgorithm alg;
	vector<T_TEACH_DATA> vtTempTeachData;
	LineOrCircularArcWeldingLine tSeam = vtSeamGroup[nSeamNo];
	bool bIsFlatWeld = !IsStandWeldSeam(tSeam.StartNormalVector);  //trueΪƽ�� falseΪ����
	bool bFreeStartPoint = !tSeam.StartPointType;
	bool bFreeEndPoint = !tSeam.EndPointType;
	double dNorAngleS = alg.CalcArcAngle(tSeam.StartNormalVector.x, tSeam.StartNormalVector.y);
	double dNorAngleE = alg.CalcArcAngle(tSeam.EndNormalVector.x, tSeam.EndNormalVector.y);
	double dStartEndDis = TwoPointDis(
		tSeam.StartPoint.x, tSeam.StartPoint.y, tSeam.StartPoint.z,
		tSeam.EndPoint.x, tSeam.EndPoint.y, tSeam.EndPoint.z);
	bool IsCircle = dStartEndDis < 1.0 ? TRUE : FALSE; // �պ�Բ��
	// ��ʼ��β�Ƿ񶨳�����
	bool bFixScanStart = m_vvtWeldLineInfoGroup.at(nGroupNo).at(nSeamNo).tAtrribute.bStartFixScan;
	bool bFixScanEnd = m_vvtWeldLineInfoGroup.at(nGroupNo).at(nSeamNo).tAtrribute.bEndFixScan;
	// �Ƿ����
	bool bTrack = m_vvtWeldLineInfoGroup.at(nGroupNo).at(nSeamNo).tAtrribute.bWeldMode;
	// Ψһ��Ҫ������β���������Ϊ�м�������������:2,/ ���Ϊ��Ӹ��棺3����β���ɻ����
	int nStartType = m_vvtWeldLineInfoGroup.at(nGroupNo).at(nSeamNo).tWeldLine.StartPointType;

	if (IsCircle) // �պ�Բ��
	{
		m_pScanInit->m_pTraceModel->m_tWorkPieceType = E_CLOSEDARC;
		CHECK_BOOL_RETURN(CalcInterfereTeachDataFlatFix(nSeamNo, bIsFlatWeld, true, vtSeamGroup, dNorAngleS, dExAxlePos, vtTempTeachData));
	}
	else if (bIsFlatWeld && bFreeStartPoint&& !bFixScanStart)			// ��� ƽ�� ���ɶ˵�
	{
		double dSearchLegth = dSearchDis;
		if (WeldingLineIsArc(m_vvtWeldLineInfoGroup.at(nGroupNo).at(nSeamNo).tWeldLine))
		{
			dSearchLegth = dSearchDis * 2;
		}
		CHECK_BOOL_RETURN(CalcFreeTeachDataFlat(nGroupNo, nSeamNo, bIsFlatWeld, true, tSeam, dNorAngleS, dSearchLegth, dExAxlePos, vtTempTeachData));
	}
	else if (bIsFlatWeld && !bFreeStartPoint&& bFixScanStart)	// ��� ƽ�� ����˵�
	{
		CHECK_BOOL_RETURN(CalcInterfereTeachDataFlatFix(nSeamNo, bIsFlatWeld, true, vtSeamGroup, dNorAngleS, dExAxlePos, vtTempTeachData));
	}
	else
	{
		//���޸�
		XUI::MesBox::PopInfo("�յ����λ�ü���ʧ�ܣ���������{0} �������{1}", bIsFlatWeld, bFreeEndPoint);
		//XiMessageBox("������λ�ü���ʧ�ܣ���������%d �������%d", bIsFlatWeld, bFreeStartPoint);
		return false;
	}
	AppendTeachData(vtTeachData, vtTempTeachData);
	// �����˵����
	m_vvtWeldLineInfoGroup.at(nGroupNo).at(nSeamNo).tAtrribute.nScanEndpointNum++;

	// �����յ��������
	//if (/*2 != */nStartType < 2/*IsCircle|| bFixScanEnd|| !bTrack*/) // �պ�Բ��
	//{
	//	vtTempTeachData.clear();
	//}
	if (IsCircle) // �պ�Բ��
	{
		vtTempTeachData.clear();
	}
	else if (bIsFlatWeld && bFreeEndPoint && !bFixScanEnd)			// �յ� ƽ�� ���ɶ˵�
	{
		CHECK_BOOL_RETURN(CalcFreeTeachDataFlat(nGroupNo, nSeamNo, bIsFlatWeld, false, tSeam, dNorAngleE, dSearchDis, dExAxlePos, vtTempTeachData));
	}
	else if (bIsFlatWeld && !bFreeEndPoint && bFixScanEnd)		// �յ� ƽ�� ����˵�
	{
		CHECK_BOOL_RETURN(CalcInterfereTeachDataFlatFix(nSeamNo, bIsFlatWeld, false, vtSeamGroup, dNorAngleE, dExAxlePos, vtTempTeachData));
	}
	else
	{
		//���޸�
		XUI::MesBox::PopInfo("������λ�ü���ʧ�ܣ���������{0} �յ�����{1}", bIsFlatWeld, bFreeStartPoint);
		//XiMessageBox("������λ�ü���ʧ�ܣ���������%d �յ�����%d", bIsFlatWeld, bFreeEndPoint);
		return false;
	}
	AppendTeachData(vtTeachData, vtTempTeachData);
	if (vtTempTeachData.size() > 0)
	{
		// �����˵����
		m_vvtWeldLineInfoGroup.at(nGroupNo).at(nSeamNo).tAtrribute.nScanEndpointNum++;
	}
	// ����ⲿ��ͻ���������,���Ż�
	bool bFirstExPos = false;
	double dMachinePos = 0.0;
	for (int i = 0; i < vtTeachData.size(); i++)
	{
		double dMachineCarMovePos = 0.0;
		double dRobotOffsetPos = -500.0;//�����˹̶�λ��	
		if (1 == m_pRobotDriver->m_nRobotInstallDir) dRobotOffsetPos = 0.0;
		T_ROBOT_COORS WorldCoors = vtTeachData[i].tMeasureCoordGunTool;
#ifdef SINGLE_ROBOT

		dMachineCarMovePos = WorldCoors.dY + WorldCoors.dBY - dRobotOffsetPos;
		WorldCoors.dY = dRobotOffsetPos;
		WorldCoors.dBY = dMachineCarMovePos;
#else
		dMachineCarMovePos = WorldCoors.dX + WorldCoors.dBX - dRobotOffsetPos;
		WorldCoors.dX = dRobotOffsetPos;
		WorldCoors.dBX = dMachineCarMovePos;
#endif
		if (bFirstExPos)
		{
#ifdef SINGLE_ROBOT
			dRobotOffsetPos = WorldCoors.dY + WorldCoors.dBY - dMachinePos;
			WorldCoors.dY = dRobotOffsetPos;
			WorldCoors.dBY = dMachinePos;
#else
			dRobotOffsetPos = WorldCoors.dX + WorldCoors.dBX - dMachinePos;
			WorldCoors.dX = dRobotOffsetPos;
			WorldCoors.dBX = dMachinePos;
#endif
		}
		else
		{
			dMachinePos = dMachineCarMovePos;
		}
		vtTeachData[i].tMeasureCoordGunTool = WorldCoors;
		if (i > 0 && 0 == (i + 1) % 2)
		{
			bFirstExPos = false;
		}
		else
		{
			bFirstExPos = true;
		}
	}
	return true;
}

bool GenericWeld::CalcFreeTeachDataFlat(int nGroupNo, int nSeamNo, bool bIsFlatSeam, bool bIsStartPtn, LineOrCircularArcWeldingLine tSeam, double dNorAngle, double dSearchDis, double dExAxlePos, vector<T_TEACH_DATA>& vtTeachData)
{
	/*
		int nWeldNo;						// ������������������
		int nMeasurePtnNo;					// ��������(ͬһ������������ڼ���������˵�)
		int nMeasureType;					// ���������ͣ� �Ȳ��ʾ�� �������� �����˵� ����ͶӰ
		E_WELD_STAM_TYPE eWeldSeamType;		// ����������������ĺ�������: ƽ�� ���� ����
		E_ATTRIBUTE eAttribute;				// �����������ԣ����ڲ������ �յ� �򺸷��м�����
		E_ENDPOINT_TYPE eEndpointType;		// �������������˵�Ķ˵�����: ���ɶ˵� ����˵�
		T_ROBOT_COORS tMeasureCoordGunTool;	// ������λ��ֱ������ ��ǹ����
		T_ROBOT_COORS tMeasureCoordCamTool;	// ����λ��ֱ������ �������
	*/
	// ������˳�򣺴Ӻ������򺸷�������
	vtTeachData.clear();
	if (!bIsFlatSeam)
	{
		XiMessageBox("��ƽ���޷�����ƽ�����ɶ˵�������ݣ�");
		return false;
	}
	T_ROBOT_COORS tCoord;
	T_TEACH_DATA tTeachData; // nMeasurePtnNo��tMeasureCoordCamTool�ⲿ�ϲ��������Ǹ�ֵ
	tTeachData.nWeldNo = nSeamNo; 
	tTeachData.nMeasurePtnNo = 0;
	tTeachData.nMeasureType = E_DOUBLE_LONG_LINE | E_SEARCH_POINT;
	tTeachData.eWeldSeamType = bIsFlatSeam ? E_FLAT_SEAM : E_STAND_SEAM;
	tTeachData.eAttribute = bIsStartPtn ? E_BELONG_START : E_BELONG_END;
	tTeachData.eEndpointType = E_FREE_POINT;
	CvPoint3D64f tPtn = bIsStartPtn ? tSeam.StartPoint : tSeam.EndPoint;
	double dSeamLen = TwoPointDis(
		tSeam.StartPoint.x, tSeam.StartPoint.y, tSeam.StartPoint.z,
		tSeam.EndPoint.x, tSeam.EndPoint.y, tSeam.EndPoint.z);

	double dRobotChangeDir = 1 == m_nRobotInstallDir ? 1.0 : -1.0;
	double dSEChangeDir = true == bIsStartPtn ? 1.0 : -1.0;
	double dOffsetDirS = dNorAngle + (90.0 * dSEChangeDir * dRobotChangeDir);
	double dOffsetDirE = dNorAngle - (90.0 * dSEChangeDir * dRobotChangeDir);
	bool bMeasureBelongPtnFree = bIsStartPtn ? !tSeam.StartPointType : !tSeam.EndPointType;
	bool bOtherPtnFree = bIsStartPtn ? !tSeam.EndPointType : !tSeam.StartPointType;

	WeldSeamAtrribute tAtrribute = m_vvtWeldLineInfoGroup[nGroupNo][nSeamNo].tAtrribute;
	LineOrCircularArcWeldingLine tWeldLine = m_vvtWeldLineInfoGroup[nGroupNo][nSeamNo].tWeldLine;
	//bool bMeasureNeedOffset = true; // !!!! �ֶκ��� �����ΰ����������ó�ͻ
	//bool bMeasureNeedOffset = (bIsStartPtn && (3 != tWeldLine.StartPointType)) || (!bIsStartPtn && (3 != tWeldLine.EndPointType));
	bool bMeasureNeedOffset = (bIsStartPtn && (99 != tAtrribute.nStartWrapType)) || (!bIsStartPtn && (99 != tAtrribute.nEndWrapType));

	tCoord = GenerateRobotCoord(tPtn, m_dNormalWeldRx, m_dNormalWeldRy, DirAngleToRz(dNorAngle));

	if (bMeasureNeedOffset)
	{
		RobotCoordPosOffset(tCoord, dOffsetDirS, dSearchDis);
	}
	DecomposeExAxle(tCoord, dExAxlePos);
	CalcCorvrAngle(bIsStartPtn, bMeasureBelongPtnFree, bOtherPtnFree, tCoord, dSeamLen);
	tTeachData.tMeasureCoordGunTool = tCoord;
	vtTeachData.push_back(tTeachData);

	tCoord = GenerateRobotCoord(tPtn, m_dNormalWeldRx, m_dNormalWeldRy, DirAngleToRz(dNorAngle));
	if (bMeasureNeedOffset)
	{
		RobotCoordPosOffset(tCoord, dOffsetDirE, dSearchDis);
	}
	DecomposeExAxle(tCoord, dExAxlePos);
	CalcCorvrAngle(bIsStartPtn, bMeasureBelongPtnFree, bOtherPtnFree, tCoord, dSeamLen);
	tTeachData.tMeasureCoordGunTool = tCoord;
	vtTeachData.push_back(tTeachData);
	return true;
}

bool GenericWeld::CalcInterfereTeachDataFlat(int nSeamNo, bool bIsFlatSeam, bool bIsStartPtn, vector<LineOrCircularArcWeldingLine> vtSeam, double dNorAngle, double dExAxlePos, vector<T_TEACH_DATA>& vtTeachData)
{
	// ������˳�򣺵�ǰ����������λ�� ���ڱ߲�������λ��(�Ƿ������ں������λ�ò�ͬ)
	vtTeachData.clear();
	if (!bIsFlatSeam)
	{
		XiMessageBox("��ƽ���޷�����ƽ������˵�������ݣ�");
		return false;
	}
	LineOrCircularArcWeldingLine tSeam = vtSeam[nSeamNo];
	double dShortSeamThreshold = m_dShortSeamThreshold;
	T_ROBOT_COORS tCoord; 
	vector<T_ROBOT_COORS> vtMeasureCoord;
	T_TEACH_DATA tTeachData; // nMeasurePtnNo��tMeasureCoordCamTool�ⲿ�ϲ��������Ǹ�ֵ
	tTeachData.nWeldNo = nSeamNo;
	tTeachData.nMeasurePtnNo = 0;
	tTeachData.nMeasureType = E_DOUBLE_LONG_LINE | E_TEACH_POINT;
	tTeachData.eWeldSeamType = bIsFlatSeam ? E_FLAT_SEAM : E_STAND_SEAM;
	tTeachData.eAttribute = bIsStartPtn ? E_BELONG_START : E_BELONG_END;
	tTeachData.eEndpointType = E_INTERFERE_POINT;
	double dCurSeamOffsetDis = m_dMeasureDisThreshold;
	double dAdjoinOffsetDis = m_dMeasureDisThreshold;
	double dCurSeamLen = TwoPointDis(tSeam.StartPoint.x, tSeam.StartPoint.y, tSeam.StartPoint.z, tSeam.EndPoint.x, tSeam.EndPoint.y, tSeam.EndPoint.z);
	double dRobotChangeDir = 1 == m_nRobotInstallDir ? 1.0 : -1.0;
	double dSEChangeDir = true == bIsStartPtn ? 1.0 : -1.0;

	// ��ǰ�����ϵĲ�����	
	LineOrCircularArcWeldingLine* ptAdjoinSeamS = NULL; //&tAdjoinSeamS; // ƽ��������ڵ�ƽ���� �� ���������������ڵ�ƽ����
	LineOrCircularArcWeldingLine* ptAdjoinSeamE = NULL; //&tAdjoinSeamE; // ƽ���յ����ڵ�ƽ���� �� �յ�������������ڵ�ƽ����
	GetAdjoinSeam(nSeamNo, vtSeam, &ptAdjoinSeamS, &ptAdjoinSeamE);
	dCurSeamOffsetDis = dCurSeamLen < dShortSeamThreshold ? dCurSeamLen / 3.0 : dCurSeamOffsetDis;
	CHECK_BOOL_RETURN(CalcSeamMeasureCoord(tSeam, dCurSeamOffsetDis, dRobotChangeDir, vtMeasureCoord, ptAdjoinSeamS, ptAdjoinSeamE));
	for (int nIdx = 0; nIdx < vtMeasureCoord.size(); nIdx++)
	{
		DecomposeExAxle(vtMeasureCoord[nIdx], dExAxlePos);
		tTeachData.tMeasureCoordGunTool = vtMeasureCoord[nIdx];
		vtTeachData.push_back(tTeachData);
	}
	// ���ں����ϵĲ�����
	double dAdjoinSeamLen = 0.0;
	LineOrCircularArcWeldingLine* ptAdjoinSeam = bIsStartPtn ? ptAdjoinSeamS : ptAdjoinSeamE;
	if (NULL != ptAdjoinSeam) // ����˵����ڱ��к���
	{
		dAdjoinSeamLen = TwoPointDis(ptAdjoinSeam->StartPoint.x, ptAdjoinSeam->StartPoint.y, ptAdjoinSeam->StartPoint.z,
			ptAdjoinSeam->EndPoint.x, ptAdjoinSeam->EndPoint.y, ptAdjoinSeam->EndPoint.z);
		dAdjoinOffsetDis = dAdjoinSeamLen < dShortSeamThreshold ? dAdjoinSeamLen / 3.0 : dAdjoinOffsetDis;
		GetAdjoinSeam(*ptAdjoinSeam, vtSeam, &ptAdjoinSeamS, &ptAdjoinSeamE);
		CHECK_BOOL_RETURN(CalcSeamMeasureCoord(*ptAdjoinSeam, dAdjoinOffsetDis, dRobotChangeDir, vtMeasureCoord, ptAdjoinSeamS, ptAdjoinSeamE));
		for (int nIdx = 0; nIdx < vtMeasureCoord.size(); nIdx++)
		{
			DecomposeExAxle(vtMeasureCoord[nIdx], dExAxlePos);
			tTeachData.tMeasureCoordGunTool = vtMeasureCoord[nIdx];
			vtTeachData.push_back(tTeachData);
		}
	}
	else // ����˵����ڱ��޺���
	{
		CvPoint3D64f tPtn = bIsStartPtn ? tSeam.StartPoint : tSeam.EndPoint;
		double dHandEyeOffsetDis = !bIsStartPtn ? m_dHandEyeDis : 0.0; // ƽ���յ�����������ں���ʱ ����������һ���������� ��Ҫ�������ƫ��
		double dAdjoinSeamNorAngle = dNorAngle + (90.0 * dSEChangeDir * dRobotChangeDir);
		tCoord = GenerateRobotCoord(tPtn, m_dPlatWeldRx, m_dPlatWeldRy, DirAngleToRz(dAdjoinSeamNorAngle));
		RobotCoordPosOffset(tCoord, dNorAngle, dAdjoinOffsetDis + dHandEyeOffsetDis);
		DecomposeExAxle(tCoord, dExAxlePos);
		CalcCorvrAngle(!bIsStartPtn, false, false, tCoord);
		tTeachData.tMeasureCoordGunTool = tCoord;
		vtTeachData.push_back(tTeachData);

		tCoord = GenerateRobotCoord(tPtn, m_dPlatWeldRx, m_dPlatWeldRy, DirAngleToRz(dAdjoinSeamNorAngle));
		RobotCoordPosOffset(tCoord, dNorAngle, dAdjoinOffsetDis * 2.0 + dHandEyeOffsetDis);
		DecomposeExAxle(tCoord, dExAxlePos);
		CalcCorvrAngle(!bIsStartPtn, false, false, tCoord); // ȥ����ֹ�ڵ�
		tTeachData.tMeasureCoordGunTool = tCoord;
		vtTeachData.push_back(tTeachData);
	}
	return true;
}

bool GenericWeld::CalcInterfereTeachDataFlatFix(int nSeamNo, bool bIsFlatSeam, bool bIsStartPtn, vector<LineOrCircularArcWeldingLine> vtSeam, double dNorAngle, double dExAxlePos, vector<T_TEACH_DATA>& vtTeachData)
{
	// ������˳�򣺵�ǰ����������λ�� ���ڱ߲�������λ��(�Ƿ������ں������λ�ò�ͬ)
	vtTeachData.clear();
	if (!bIsFlatSeam)
	{
		XiMessageBox("��ƽ���޷�����ƽ������˵�������ݣ�");
		return false;
	}
	LineOrCircularArcWeldingLine tSeam = vtSeam[nSeamNo];
	double dShortSeamThreshold = m_dShortSeamThreshold;
	T_ROBOT_COORS tCoord;
	vector<T_ROBOT_COORS> vtMeasureCoord;
	T_TEACH_DATA tTeachData; // nMeasurePtnNo��tMeasureCoordCamTool�ⲿ�ϲ��������Ǹ�ֵ
	tTeachData.nWeldNo = nSeamNo;
	tTeachData.nMeasurePtnNo = 0;
	tTeachData.nMeasureType = E_DOUBLE_LONG_LINE | E_SEARCH_POINT | E_SEARCH_POINT_FIX;
	tTeachData.eWeldSeamType = bIsFlatSeam ? E_FLAT_SEAM : E_STAND_SEAM;
	tTeachData.eAttribute = bIsStartPtn ? E_BELONG_START : E_BELONG_END;
	tTeachData.eEndpointType = E_INTERFERE_POINT;
	double dCurSeamOffsetDis = m_dEndpointSearchDis;// m_dMeasureDisThreshold;//��������
	double dAdjoinOffsetDis = m_dEndpointSearchDis;//m_dMeasureDisThreshold;//��������
	double dCurSeamLen = TwoPointDis(tSeam.StartPoint.x, tSeam.StartPoint.y, tSeam.StartPoint.z, tSeam.EndPoint.x, tSeam.EndPoint.y, tSeam.EndPoint.z);
	double dRobotChangeDir = 1 == m_nRobotInstallDir ? 1.0 : -1.0;
	double dSEChangeDir = true == bIsStartPtn ? 1.0 : -1.0;

	// ��ǰ�����ϵĲ�����	
	LineOrCircularArcWeldingLine* ptAdjoinSeamS = NULL; //&tAdjoinSeamS; // ƽ��������ڵ�ƽ���� �� ���������������ڵ�ƽ����
	LineOrCircularArcWeldingLine* ptAdjoinSeamE = NULL; //&tAdjoinSeamE; // ƽ���յ����ڵ�ƽ���� �� �յ�������������ڵ�ƽ����
	GetAdjoinSeam(nSeamNo, vtSeam, &ptAdjoinSeamS, &ptAdjoinSeamE);
	//dCurSeamOffsetDis = dCurSeamLen < dShortSeamThreshold ? dCurSeamLen / 3.0 : dCurSeamOffsetDis;
	// Բ����������	
	CHECK_BOOL_RETURN(CalcSeamMeasureCoordFix(tSeam, dCurSeamOffsetDis, dRobotChangeDir, vtMeasureCoord, ptAdjoinSeamS, ptAdjoinSeamE, bIsStartPtn));

	for (int nIdx = 0; nIdx < vtMeasureCoord.size(); nIdx++)
	{
		DecomposeExAxle(vtMeasureCoord[nIdx], dExAxlePos);
		tTeachData.tMeasureCoordGunTool = vtMeasureCoord[nIdx];
		vtTeachData.push_back(tTeachData);
	}
	return true;
}


bool GenericWeld::CalcTeachDataStand(int nSeamNo, bool bIsFlatSeam, bool bIsStartPtn, 
	vector<LineOrCircularArcWeldingLine> vtSeam, double dExAxlePos, vector<T_TEACH_DATA>& vtTeachData)
{
	// ������˳��˳ʱ�����ں������ ��ʱ�����ں������
	vtTeachData.clear();
	if (bIsFlatSeam)
	{
		XiMessageBox("�������޷�����������������ݣ�");
		return false;
	}
	LineOrCircularArcWeldingLine tSeam = vtSeam[nSeamNo];
	double dShortSeamThreshold = m_dShortSeamThreshold;
	XiAlgorithm alg;
	T_ROBOT_COORS tCoord;
	vector<T_ROBOT_COORS> vtMeasureCoord;
	E_ENDPOINT_TYPE eEndpointTypeS = 0 <  tSeam.StartPointType ? E_INTERFERE_POINT : E_FREE_POINT;
	E_ENDPOINT_TYPE eEndpointTypeE = 0 <  tSeam.EndPointType ? E_INTERFERE_POINT : E_FREE_POINT;
	vector<T_TEACH_DATA> vtTeachDataStandUpper; // �����߶Ȳ�����
	vtTeachDataStandUpper.clear();
	T_TEACH_DATA tTeachData; // nMeasurePtnNo��tMeasureCoordCamTool�ⲿ�ϲ��������Ǹ�ֵ
	tTeachData.nWeldNo = nSeamNo;
	tTeachData.nMeasurePtnNo = 0;
	tTeachData.nMeasureType = E_INTERFERE_POINT == eEndpointTypeS ? E_DOUBLE_LONG_LINE | E_TEACH_POINT : E_L_LINE_POINT | E_TEACH_POINT; // �����յ�����϶���������Ͳ�һ��
	tTeachData.eWeldSeamType = bIsFlatSeam ? E_FLAT_SEAM : E_STAND_SEAM;
	tTeachData.eAttribute = bIsStartPtn ? E_BELONG_START : E_BELONG_END;
	tTeachData.eEndpointType = true == bIsStartPtn ? eEndpointTypeS : eEndpointTypeE;
	double dAdjoinOffsetDis = m_dMeasureDisThreshold;
	double dCurSeamLen = TwoPointDis(tSeam.StartPoint.x, tSeam.StartPoint.y, tSeam.StartPoint.z, tSeam.EndPoint.x, tSeam.EndPoint.y, tSeam.EndPoint.z);
	double dRobotChangeDir = 1 == m_nRobotInstallDir ? 1.0 : -1.0;
	double dNorAngle = alg.CalcArcAngle(tSeam.StartNormalVector.x, tSeam.StartNormalVector.y);

	// �����յ������������ȷ�������� ˳ʱ�� ��ʱ�� �� �������������߶� tSeam.EndPoint.z tSeam.ZSide �Ĺ�ϵ
	// ����������������߶�
	double dAdjoinSeam1Height = true == tSeam.isLeft ? tSeam.ZSide : tSeam.EndPoint.z; // ZSide ����?
	double dAdjoinSeam2Height = true == tSeam.isLeft ? tSeam.EndPoint.z : tSeam.ZSide;
	LineOrCircularArcWeldingLine* ptAdjoinSeamS = NULL;
	LineOrCircularArcWeldingLine* ptAdjoinSeamE = NULL;
	GetAdjoinSeam(tSeam, vtSeam, &ptAdjoinSeamS, &ptAdjoinSeamE);
	LineOrCircularArcWeldingLine* ptAdjoinSeam1 = ptAdjoinSeamE; // ��ǹ�Ƕ��������ƽ��
	LineOrCircularArcWeldingLine* ptAdjoinSeam2 = ptAdjoinSeamS; // ��ǹ�Ƕ������Ҳ�ƽ��

	if (NULL != ptAdjoinSeam1)	// ����˳ʱ�뷽��������ں���
	{
		double dAdjoinSeamLen = TwoPointDis(ptAdjoinSeam1->StartPoint.x, ptAdjoinSeam1->StartPoint.y, ptAdjoinSeam1->StartPoint.z,
			ptAdjoinSeam1->EndPoint.x, ptAdjoinSeam1->EndPoint.y, ptAdjoinSeam1->EndPoint.z);
		dAdjoinOffsetDis = dAdjoinSeamLen < dShortSeamThreshold ? dAdjoinSeamLen / 3.0 : dAdjoinOffsetDis;
		GetAdjoinSeam(*ptAdjoinSeam1, vtSeam, &ptAdjoinSeamS, &ptAdjoinSeamE);
		CHECK_BOOL_RETURN(CalcSeamMeasureCoord(*ptAdjoinSeam1, dAdjoinOffsetDis, dRobotChangeDir, vtMeasureCoord, ptAdjoinSeamS, ptAdjoinSeamE));
		for (int nIdx = 0; nIdx < vtMeasureCoord.size(); nIdx++)
		{
			DecomposeExAxle(vtMeasureCoord[nIdx], dExAxlePos);
			tTeachData.tMeasureCoordGunTool = vtMeasureCoord[nIdx];
			vtTeachData.push_back(tTeachData);
		}
		if (false == bIsStartPtn) // �����յ�߶Ȳ�����
		{
			//tTeachData.tMeasureCoordGunTool.dRX = m_dPlatWeldRx;
			//tTeachData.tMeasureCoordGunTool.dRY = m_dPlatWeldRy;
			//tTeachData.tMeasureCoordGunTool = RobotCoordPosOffset(tTeachData.tMeasureCoordGunTool, ptAdjoinSeam1->StartPoint, ptAdjoinSeam1->EndPoint, 18.0);
			tTeachData.tMeasureCoordGunTool.dZ = dAdjoinSeam1Height;
			vtTeachDataStandUpper.push_back(tTeachData);
		}
	}
	else						// ����˳ʱ�뷽�򲻴������ں���
	{
		CvPoint3D64f tPtn = tSeam.StartPoint;
		double dAdjoinSeamNorAngle = dNorAngle + (45.0 * dRobotChangeDir);
		double dOffsetDir = dNorAngle - (45.0 * dRobotChangeDir);
		tCoord = GenerateRobotCoord(tPtn, m_dPlatWeldRx, m_dPlatWeldRy, DirAngleToRz(dAdjoinSeamNorAngle));
		RobotCoordPosOffset(tCoord, dOffsetDir, dAdjoinOffsetDis); // �յ������������ӵ�ƽ���ϲ���λ����������ƫ��
		DecomposeExAxle(tCoord, dExAxlePos);
		CalcCorvrAngle(false, false, false, tCoord); // ��ptAdjoinSeamE���
		tTeachData.tMeasureCoordGunTool = tCoord;
		vtTeachData.push_back(tTeachData);

		if (false == bIsStartPtn) // �����յ�߶Ȳ�����
		{
			//tTeachData.tMeasureCoordGunTool.dRX = m_dPlatWeldRx;
			//tTeachData.tMeasureCoordGunTool.dRY = m_dPlatWeldRy;
			tTeachData.tMeasureCoordGunTool.dZ = dAdjoinSeam1Height;
			vtTeachDataStandUpper.push_back(tTeachData);
		}

		tCoord = GenerateRobotCoord(tPtn, m_dPlatWeldRx, m_dPlatWeldRy, DirAngleToRz(dAdjoinSeamNorAngle));
		RobotCoordPosOffset(tCoord, dOffsetDir, dAdjoinOffsetDis * 2.0/* - 10.0*/);  // ��·��ʱ-10
		DecomposeExAxle(tCoord, dExAxlePos);
		CalcCorvrAngle(false, false, false, tCoord); // ��ptAdjoinSeamE��� // ȥ����ֹ�ڵ�
		tTeachData.tMeasureCoordGunTool = tCoord;
		vtTeachData.push_back(tTeachData);
	}

	dAdjoinOffsetDis = m_dMeasureDisThreshold;
	if (NULL != ptAdjoinSeam2)	// ������ʱ�뷽��������ں���
	{
		double dAdjoinSeamLen = TwoPointDis(ptAdjoinSeam2->StartPoint.x, ptAdjoinSeam2->StartPoint.y, ptAdjoinSeam2->StartPoint.z,
			ptAdjoinSeam2->EndPoint.x, ptAdjoinSeam2->EndPoint.y, ptAdjoinSeam2->EndPoint.z);
		dAdjoinOffsetDis = dAdjoinSeamLen < dShortSeamThreshold ? dAdjoinSeamLen / 3.0 : dAdjoinOffsetDis;
		GetAdjoinSeam(*ptAdjoinSeam2, vtSeam, &ptAdjoinSeamS, &ptAdjoinSeamE);
		CHECK_BOOL_RETURN(CalcSeamMeasureCoord(*ptAdjoinSeam2, dAdjoinOffsetDis, dRobotChangeDir, vtMeasureCoord, ptAdjoinSeamS, ptAdjoinSeamE));
		for (int nIdx = 0; nIdx < vtMeasureCoord.size(); nIdx++)
		{
			DecomposeExAxle(vtMeasureCoord[nIdx], dExAxlePos);
			tTeachData.tMeasureCoordGunTool = vtMeasureCoord[nIdx];
			vtTeachData.push_back(tTeachData);
		}
		if (false == bIsStartPtn) // �����յ�߶Ȳ�����
		{
			tTeachData.tMeasureCoordGunTool = vtMeasureCoord[0];
			//tTeachData.tMeasureCoordGunTool.dRX = m_dPlatWeldRx;
			//tTeachData.tMeasureCoordGunTool.dRY = m_dPlatWeldRy;
			//tTeachData.tMeasureCoordGunTool = RobotCoordPosOffset(tTeachData.tMeasureCoordGunTool, ptAdjoinSeam2->EndPoint, ptAdjoinSeam2->StartPoint, 18.0);
			tTeachData.tMeasureCoordGunTool.dZ = dAdjoinSeam2Height;
			vtTeachDataStandUpper.push_back(tTeachData);
		}
	}
	else						// ������ʱ�뷽�򲻴������ں���
	{
		CvPoint3D64f tPtn = tSeam.StartPoint;
		double dAdjoinSeamNorAngle = dNorAngle - (45.0 * dRobotChangeDir);
		double dOffsetDir = dNorAngle + (45.0 * dRobotChangeDir);
		tCoord = GenerateRobotCoord(tPtn, m_dPlatWeldRx, m_dPlatWeldRy, DirAngleToRz(dAdjoinSeamNorAngle));
		RobotCoordPosOffset(tCoord, dOffsetDir, dAdjoinOffsetDis + m_dHandEyeDis); // ������������������ƽ���ϲ�������Ҫ�������ƫ�ƾ���
		DecomposeExAxle(tCoord, dExAxlePos);
		CalcCorvrAngle(true, false, false, tCoord); // ��ptAdjoinSeamS���
		tTeachData.tMeasureCoordGunTool = tCoord;
		vtTeachData.push_back(tTeachData);

		if (false == bIsStartPtn) // �����յ�߶Ȳ�����
		{
			//tTeachData.tMeasureCoordGunTool.dRX = m_dPlatWeldRx;
			//tTeachData.tMeasureCoordGunTool.dRY = m_dPlatWeldRy;
			tTeachData.tMeasureCoordGunTool.dZ = dAdjoinSeam2Height;
			vtTeachDataStandUpper.push_back(tTeachData);
		}

		tCoord = GenerateRobotCoord(tPtn, m_dPlatWeldRx, m_dPlatWeldRy, DirAngleToRz(dAdjoinSeamNorAngle));
		RobotCoordPosOffset(tCoord, dOffsetDir, dAdjoinOffsetDis * 2.0/* - 10.0*/ + m_dHandEyeDis); // ��·��ʱ-10 // ������������������ƽ���ϲ�������Ҫ�������ƫ�ƾ���
		DecomposeExAxle(tCoord, dExAxlePos);
		CalcCorvrAngle(true, false, false, tCoord); // ��ptAdjoinSeamS���  // ȥ����ֹ�ڵ�
		tTeachData.tMeasureCoordGunTool = tCoord;
		vtTeachData.push_back(tTeachData);
	}
	for (int i = 0; i < vtTeachDataStandUpper.size(); i++)
	{
		vtTeachDataStandUpper[i].nMeasureType = E_LS_RL_FLIP | E_TEACH_POINT;
	}
	AppendTeachData(vtTeachData, vtTeachDataStandUpper); // ׷�������յ�߶Ȳ�������

	return true;
}

bool GenericWeld::CalcFlatSeamMeasurementEnhancement(int nSeamNo, bool bIsFlatSeam, bool bIsStartPtn, LineOrCircularArcWeldingLine tSeam, double dNorAngle, double dExAxlePos, LineOrCircularArcWeldingLine* ptAdjoinSeam, vector<T_TEACH_DATA>& vtTeachData)
{
	//������������
	double dPointSpacing = m_dPointSpacing;
	//ƽ�����ϲ������ľ���
	double dShortSeamThreshold = m_dShortSeamThreshold;
	//��ʱ����ʹ�õĺ�ǹ����
	T_ROBOT_COORS tCoord;
	vector<T_ROBOT_COORS> vtTempCoord;
	vector<T_ROBOT_COORS> vtMeasureCoord;
	T_TEACH_DATA tTeachData; // nMeasurePtnNo��tMeasureCoordCamTool�ⲿ�ϲ��������Ǹ�ֵ
	//���ò��������Ĳ���
	tTeachData.nWeldNo = nSeamNo;
	tTeachData.nMeasurePtnNo = 0;
	tTeachData.nMeasureType = E_DOUBLE_LONG_LINE | E_ADJUST_POINT;
	tTeachData.eWeldSeamType = E_FLAT_SEAM;
	tTeachData.eAttribute =  E_BELONG_MIDDLE;
	tTeachData.eEndpointType = E_INTERFERE_POINT;
	double dCurSeamOffsetDis = m_dMeasureDisThreshold;
	double dAdjoinOffsetDis = m_dMeasureDisThreshold;
	//�����յ�֮��ľ���
	double dCurSeamLen = TwoPointDis(tSeam.StartPoint.x, tSeam.StartPoint.y, tSeam.StartPoint.z, tSeam.EndPoint.x, tSeam.EndPoint.y, tSeam.EndPoint.z);
	//�жϻ����˰�װ����
	double dRobotChangeDir = 1 == m_nRobotInstallDir ? 1.0 : -1.0;
	double dSEChangeDir = true == bIsStartPtn ? 1.0 : -1.0;
	//�㷨��
	XiAlgorithm alg;
	double NorAngle = alg.CalcArcAngle(tSeam.StartNormalVector.x, tSeam.StartNormalVector.y);
	//�����ж���ƽ���������ɵ�����������֮��ľ����Ƿ���ڲ����������ֵ�����ľ���
	//1. ����ƽ����������������
		// ��ǰ�����ϵĲ�����	
	dCurSeamOffsetDis = dCurSeamLen < dShortSeamThreshold ? dCurSeamLen / 3.0 : dCurSeamOffsetDis;
	CHECK_BOOL_RETURN(CalcSeamMeasureCoord(tSeam, dCurSeamOffsetDis, dRobotChangeDir, vtTempCoord, NULL, NULL));
	//2. �ж�����ƽ����֮��ľ����Ƿ���ڲ������ϵ���ֵ �������2����ֵ�� ������ɲ����
	int temp = TwoPointDis(vtTempCoord[0].dX, vtTempCoord[0].dY, vtTempCoord[0].dZ, vtTempCoord[1].dX, vtTempCoord[1].dY, vtTempCoord[1].dZ);
	if (temp > (2 * dPointSpacing)) {
		int flag = temp / dPointSpacing;
		while (flag >= 2) {
			if (flag == 2) {
				tCoord.dX  = (vtTempCoord[0].dX  + vtTempCoord[1].dX )/2;
				tCoord.dY  = (vtTempCoord[0].dY  + vtTempCoord[1].dY )/2;
				tCoord.dZ  = (vtTempCoord[0].dZ  + vtTempCoord[1].dZ )/2;
				tCoord.dRX = m_dNormalWeldRx;
				tCoord.dRY = m_dNormalWeldRy;
				tCoord.dRZ = DirAngleToRz(NorAngle);
				vtMeasureCoord.push_back(tCoord);
				break;// return true;
			}
			else {
				tSeam.StartPoint.x = vtTempCoord[0].dX;
				tSeam.StartPoint.y = vtTempCoord[0].dY;
				tSeam.StartPoint.z = vtTempCoord[0].dZ;
				tSeam.EndPoint.x = vtTempCoord[1].dX;
				tSeam.EndPoint.y = vtTempCoord[1].dY;
				tSeam.EndPoint.z = vtTempCoord[1].dZ;
				vtTempCoord.clear();
				CalcSeamMeasureCoord(tSeam, m_dPointSpacing, dRobotChangeDir, vtTempCoord, NULL, NULL, false);
				vtMeasureCoord.push_back(vtTempCoord[0]);
				vtMeasureCoord.push_back(vtTempCoord[1]);
				flag -= 2;
			}
		}
	}
	else {
		return false;
	}
	for (int i = 0; i < vtMeasureCoord.size(); i++) {
		DecomposeExAxle(vtMeasureCoord[i], dExAxlePos);
		vtMeasureCoord[i].dRX = m_dNormalWeldRx;
		tTeachData.tMeasureCoordGunTool = vtMeasureCoord[i];
		vtTeachData.push_back(tTeachData);
	}
	return true;
}

bool GenericWeld::CalcTeachDataStandFreeStart(int nSeamNo, bool bIsFlatSeam, bool bIsStartPtn, LineOrCircularArcWeldingLine tSeam, double dExAxlePos,
	LineOrCircularArcWeldingLine* ptAdjoinSeamS, LineOrCircularArcWeldingLine* ptAdjoinSeamE, vector<T_TEACH_DATA>& vtTeachData)
{
	// ������˳��˳ʱ�����ں������ ��ʱ�����ں������
	vtTeachData.clear();
	if (bIsFlatSeam || !bIsStartPtn)
	{
		XiMessageBox("���������ɶ˵� �����������ʧ�ܣ�");
		return false;
	}
	double dShortSeamThreshold = m_dShortSeamThreshold;
	XiAlgorithm alg;
	T_ROBOT_COORS tCoord;
	vector<T_ROBOT_COORS> vtMeasureCoord;
	E_ENDPOINT_TYPE eEndpointTypeS = 0 <  tSeam.StartPointType ? E_INTERFERE_POINT : E_FREE_POINT;
	E_ENDPOINT_TYPE eEndpointTypeE = 0 <  tSeam.EndPointType ? E_INTERFERE_POINT : E_FREE_POINT;
	vector<T_TEACH_DATA> vtTeachDataStandUpper; // �����߶Ȳ�����
	vtTeachDataStandUpper.clear();
	T_TEACH_DATA tTeachData; 
	tTeachData.nWeldNo = nSeamNo;
	tTeachData.nMeasurePtnNo = 0;
	tTeachData.nMeasureType = E_L_LINE_POINT | E_TEACH_POINT; // �����յ�����϶���������Ͳ�һ��
	tTeachData.eWeldSeamType = bIsFlatSeam ? E_FLAT_SEAM : E_STAND_SEAM;
	tTeachData.eAttribute = bIsStartPtn ? E_BELONG_START : E_BELONG_END;
	tTeachData.eEndpointType = true == bIsStartPtn ? eEndpointTypeS : eEndpointTypeE;
	double dAdjoinOffsetDis = m_dMeasureDisThreshold;
	double dRobotChangeDir = 1 == m_nRobotInstallDir ? 1.0 : -1.0;
	double dNorAngle = alg.CalcArcAngle(tSeam.StartNormalVector.x, tSeam.StartNormalVector.y);

	// ƽ���յ������������� ����İ��� �Ĳ�����
	CvPoint3D64f tPtn = tSeam.StartPoint;
	double dAdjoinSeamNorAngle = dNorAngle + (45.0 * dRobotChangeDir);
	double dOffsetDir = dNorAngle - (45.0 * dRobotChangeDir);
	tCoord = GenerateRobotCoord(tPtn, m_dPlatWeldRx, m_dPlatWeldRy, DirAngleToRz(dAdjoinSeamNorAngle));
	RobotCoordPosOffset(tCoord, dOffsetDir, dAdjoinOffsetDis); // �յ������������ӵ�ƽ���ϲ���λ����������ƫ��
	DecomposeExAxle(tCoord, dExAxlePos);
	CalcCorvrAngle(false, false, false, tCoord);
	tTeachData.tMeasureCoordGunTool = tCoord;
	vtTeachData.push_back(tTeachData);

	tCoord = GenerateRobotCoord(tPtn, m_dPlatWeldRx, m_dPlatWeldRy, DirAngleToRz(dAdjoinSeamNorAngle));
	RobotCoordPosOffset(tCoord, dOffsetDir, dAdjoinOffsetDis * 2.0);
	DecomposeExAxle(tCoord, dExAxlePos);
	//tCoord.dRX += CalcCorvrAngle(false, false, false); // ȥ����ֹ�ڵ�
	tTeachData.tMeasureCoordGunTool = tCoord;
	vtTeachData.push_back(tTeachData);

	dAdjoinOffsetDis = m_dMeasureDisThreshold;
	// ƽ����������������� ����İ��� �Ĳ�����			
	{
		CvPoint3D64f tPtn = tSeam.StartPoint;
		double dAdjoinSeamNorAngle = dNorAngle - (45.0 * dRobotChangeDir);
		double dOffsetDir = dNorAngle + (45.0 * dRobotChangeDir);
		tCoord = GenerateRobotCoord(tPtn, m_dPlatWeldRx, m_dPlatWeldRy, DirAngleToRz(dAdjoinSeamNorAngle));
		RobotCoordPosOffset(tCoord, dOffsetDir, dAdjoinOffsetDis + m_dHandEyeDis); // ������������������ƽ���ϲ�������Ҫ�������ƫ�ƾ���
		DecomposeExAxle(tCoord, dExAxlePos);
		CalcCorvrAngle(true, false, false, tCoord);
		tTeachData.tMeasureCoordGunTool = tCoord;
		vtTeachData.push_back(tTeachData);

		tCoord = GenerateRobotCoord(tPtn, m_dPlatWeldRx, m_dPlatWeldRy, DirAngleToRz(dAdjoinSeamNorAngle));
		RobotCoordPosOffset(tCoord, dOffsetDir, dAdjoinOffsetDis * 2.0 + m_dHandEyeDis); // ������������������ƽ���ϲ�������Ҫ�������ƫ�ƾ���
		DecomposeExAxle(tCoord, dExAxlePos);
		//tCoord.dRX += CalcCorvrAngle(true, false, false);  // ȥ����ֹ�ڵ�
		tTeachData.tMeasureCoordGunTool = tCoord;
		vtTeachData.push_back(tTeachData);
	}

	return true;
}

void GenericWeld::CalcCorvrAngle(bool bBelongStart, bool bBelongPtnFree, bool bOtherPtnFree, T_ROBOT_COORS& tCoord, double dSeamLen/* = 0*/)
{
	// ������������㸽�������յ㸽�� �� 
	// ����������������(true:����  false:����)
	// ����������������ĵ�����(true:����  false:����)
	// ���쳤�� Ĭ��0��ʾ �����㲻����ĳ������ �Ǹ��ݸ���˵�������ĺ���
	
	// ˫���ɶ� ������̬
	// ���������� ���ȴ� ������̬
	// ���������� �� ��һ��˵���� �� ����С �� !bBelongStart �������̬
	// ��������� ��bBelongStart �������̬

	/*if ((bBelongPtnFree && bOtherPtnFree) ||
		(bBelongPtnFree && (dSeamLen > m_dShortSeamThreshold)))
	{
		return;
	}
	else */
	bool isBelongStart = bBelongStart;
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

bool GenericWeld::WeldSeamAdjust(int nGroupNo)
{
	double dZSide = 0.0;
	double dNorAngle = 0.0;
	T_ALGORITHM_POINT tAlgPtnS;
	T_ALGORITHM_POINT tAlgPtnE;
	std::vector<T_TEACH_DATA> vtTeachData;
	std::map<int, int> mnnMeasureNo;
	vtTeachData.clear();
	mnnMeasureNo.clear();
	std::vector<LineOrCircularArcWeldingLine> tSeamGroup(m_vvtWeldSeamGroupAdjust[nGroupNo]);
	std::vector<std::vector<T_TEACH_DATA>> vvtTeachData(tSeamGroup.size(), vtTeachData); // ÿ���������֣�����1����� ����2�������

	// ���ʾ�̽������������Ƿ�����m_vtTeachData������ �����ҵ�ǰ������ÿ������Ĳ�������
	for (int nDataNo = 0; nDataNo < m_vtTeachData.size(); nDataNo++)
	{
		mnnMeasureNo[m_vtTeachData[nDataNo].nMeasurePtnNo] = 0;
		vvtTeachData[m_vtTeachData[nDataNo].nWeldNo].push_back(m_vtTeachData[nDataNo]);
	}
	if (m_vtTeachResult.size() != mnnMeasureNo.size())
	{
		//���޸�
		XUI::MesBox::PopInfo("�������ݴ���! �������ʧ��! {0} {1}", mnnMeasureNo.size(), m_vtTeachResult.size());
		//XiMessageBox("�������ݴ���! �������ʧ��! %d %d", mnnMeasureNo.size(), m_vtTeachResult.size());
		return false;
	}

	// ����������nGroup��ÿ������
	for (int nWeldNo = 0; nWeldNo < tSeamGroup.size(); nWeldNo++)
	{
		// ��ȡ��������в�������
		std::vector<T_TEACH_DATA>& tData = vvtTeachData[nWeldNo];
		if (10 == m_vvtWeldLineInfoGroup[nGroupNo][nWeldNo].tAtrribute.nStartWrapType)
		{
			CHECK_BOOL_RETURN(AdjustScanWeldSeam(nGroupNo, nWeldNo, tData));
			continue;
		}
		else if (WeldingLineIsArc(tSeamGroup[nWeldNo])) // Բ�� ���� ����յ�
		{
			CHECK_BOOL_RETURN(AdjustWeldSeamArc(nGroupNo, nWeldNo, tData));
			continue;
		}
		// ���� ƽ��/����  ���/�յ�  ����/���� �����Լ���׼ȷ�˵�����
		CHECK_BOOL_RETURN(CalcAccurateEndpoint(nGroupNo, nWeldNo, tData, tAlgPtnS, tAlgPtnE, dNorAngle, dZSide));

		// ����һ������
		m_vvtWeldSeamGroupAdjust[nGroupNo][nWeldNo] = m_vvtWeldSeamGroup[nGroupNo][nWeldNo]; // �ظ�����ͬһ�麸��ָ�����ǰʶ������
		CHECK_BOOL_RETURN(AdjsutWeldSeam(m_vvtWeldSeamGroupAdjust[nGroupNo][nWeldNo], tAlgPtnS, tAlgPtnE, dNorAngle, dZSide));
	}
	return true;
}

bool GenericWeld::CalcAccurateEndpoint(int nGroupNo, int nWeldNo, std::vector<T_TEACH_DATA>& tData,
	T_ALGORITHM_POINT& tS, T_ALGORITHM_POINT& tE, double& dNorAngle, double& dZSide)
{
	// ��� ƽ�� ���ɶ˵�	��������λ�ã�ȡ�˵�
	// �յ� ƽ�� ���ɶ˵�	��������λ�ã�ȡ�˵�
	// ��� ƽ�� ����˵�	�ĸ�����λ�ã������ཻ
	// ��� ���� ����˵�	�ĸ�����λ�ã������ཻ
	// �յ� ƽ�� ����˵�	�ĸ�����λ�ã������ཻ
	// �յ� ���� ���ɶ˵�	��������λ�ã����������ཻ�Ͳ��

	// ��ȡÿ���˵�����в������� ���ж���Ч��
	XiAlgorithm alg;
	std::vector<T_TEACH_DATA> tTeachDataS;
	std::vector<T_TEACH_DATA> tTeachDataE;
	tTeachDataS.clear();
	tTeachDataE.clear();

	for (int nDataNo = 0; nDataNo < tData.size(); nDataNo++)
	{
		if (tData[nDataNo].eAttribute == E_BELONG_START)
			tTeachDataS.push_back(tData[nDataNo]);
		else if (tData[nDataNo].eAttribute == E_BELONG_END)
			tTeachDataE.push_back(tData[nDataNo]);
	}
	if ((0 >= tTeachDataS.size()) || (0 >= tTeachDataE.size()))
	{
		//���޸�
		XUI::MesBox::PopInfo("������{0} ����{1} ��������{2} {3}�쳣��", nGroupNo, nWeldNo, tTeachDataS.size(), tTeachDataE.size());
		//XiMessageBox("������%d ����%d ��������%d %d�쳣��", nGroupNo, nWeldNo, tTeachDataS.size(), tTeachDataE.size());
		return false;
	}
	bool bIsFlatWeld = E_FLAT_SEAM == tTeachDataS[0].eWeldSeamType;
	bool bFreePointS = E_FREE_POINT == tTeachDataS[0].eEndpointType;
	bool bFreePointE = E_FREE_POINT == tTeachDataE[0].eEndpointType;

	// ���Ҹ���ζ�����������ʱ���Ķ˵�Ϊ������ȡ�˵�
	//if (bIsFlatWeld)
	//{
	//	bFreePointS = true;
	//	bFreePointE = true;
	//}
	//-------------------------

	CHECK_BOOL_RETURN(CheckTeachDataNumValid(tTeachDataS.size(), bIsFlatWeld, true, bFreePointS));
	CHECK_BOOL_RETURN(CheckTeachDataNumValid(tTeachDataE.size(), bIsFlatWeld, false, bFreePointE));

	// ���㾫ȷ���
	if (bIsFlatWeld && bFreePointS)			// ��� ƽ�� ���ɶ˵� 1
	{
		CHECK_BOOL_RETURN(CalcFreePoint(tTeachDataS, tS));
	}
	else if (bIsFlatWeld && !bFreePointS)	// ��� ƽ�� ����˵� 2
	{
		CHECK_BOOL_RETURN(CalcInterferePoint(tTeachDataS, tS));
	}
	else if (!bIsFlatWeld && !bFreePointS)	// ��� ���� ����˵� 2
	{
		CHECK_BOOL_RETURN(CalcInterferePointStand(tTeachDataS, tS));
	}
	else if (!bIsFlatWeld && bFreePointS)	// ��� ���� ���ɶ˵� 4
	{
		CHECK_BOOL_RETURN(CalcFreeStartPointStand(tTeachDataS, tS));
	}
	else
	{
		//���޸�
		XUI::MesBox::PopInfo("������{0} ����{1} �������ʧ�ܣ���������{2} �������{3}", nGroupNo, nWeldNo, bIsFlatWeld, bFreePointS);
		//XiMessageBox("������%d ����%d �������ʧ�ܣ���������%d �������%d", nGroupNo, nWeldNo, bIsFlatWeld, bFreePointS);
		return false;
	}

	// ���㾫ȷ�յ�
	if (bIsFlatWeld && bFreePointE)			// �յ� ƽ�� ���ɶ˵� 1
	{
		CHECK_BOOL_RETURN(CalcFreePoint(tTeachDataE, tE));
	}
	else if (bIsFlatWeld && !bFreePointE)		// �յ� ƽ�� ����˵� 2
	{
		CHECK_BOOL_RETURN(CalcInterferePoint(tTeachDataE, tE));
	}
	else if (!bIsFlatWeld && bFreePointE)		// �յ� ���� ���ɶ˵� 3
	{
		CHECK_BOOL_RETURN(CalcFreeEndPointStand(tTeachDataE, tE));
	}
	else
	{
		//���޸�
		XUI::MesBox::PopInfo("������{0} ����{1} �����յ�ʧ�ܣ���������{2} �������{3}", nGroupNo, nWeldNo, bIsFlatWeld, bFreePointE);
		//XiMessageBox("������%d ����%d �����յ�ʧ�ܣ���������%d �������%d", nGroupNo, nWeldNo, bIsFlatWeld, bFreePointE);
		return false;
	}

	// ���㺸�취��
	if (bIsFlatWeld)
	{
		double dWeldDir = alg.CalcArcAngle(tE.dCoorX - tS.dCoorX, tE.dCoorY - tS.dCoorY);
		dNorAngle = dWeldDir - (90.0 * (double)m_nRobotInstallDir);
	}
	else
	{
		int nIdx1 = tTeachDataS[0].nMeasurePtnNo;
		int nIdx2 = tTeachDataS[1].nMeasurePtnNo;
		int nIdx3 = tTeachDataS[2].nMeasurePtnNo;
		int nIdx4 = tTeachDataS[3].nMeasurePtnNo;
		double dWeldDir1 = alg.CalcArcAngle(
			m_vtTeachResult[nIdx1].tKeyPtn3D.x - m_vtTeachResult[nIdx2].tKeyPtn3D.x,
			m_vtTeachResult[nIdx1].tKeyPtn3D.y - m_vtTeachResult[nIdx2].tKeyPtn3D.y);
		double dWeldDir2 = alg.CalcArcAngle(
			m_vtTeachResult[nIdx3].tKeyPtn3D.x - m_vtTeachResult[nIdx4].tKeyPtn3D.x,
			m_vtTeachResult[nIdx3].tKeyPtn3D.y - m_vtTeachResult[nIdx4].tKeyPtn3D.y);
		double dBaseNorAngle = alg.CalcArcAngle(
			m_vvtWeldSeamGroupAdjust[nGroupNo][nWeldNo].StartNormalVector.x,
			m_vvtWeldSeamGroupAdjust[nGroupNo][nWeldNo].StartNormalVector.y);
		dWeldDir1 = true == JudgeDirAngle(dBaseNorAngle, dWeldDir1, 90.0) ? dWeldDir1 : dWeldDir1 += 180.0;
		dWeldDir2 = true == JudgeDirAngle(dBaseNorAngle, dWeldDir2, 90.0) ? dWeldDir2 : dWeldDir2 += 180.0;
		dNorAngle = TwoDirAngleMedian(dWeldDir1, dWeldDir2);
	}

	// dZSide
	dZSide = m_vvtWeldSeamGroupAdjust[nGroupNo][nWeldNo].ZSide;

	return true;
}

bool GenericWeld::CheckTeachDataNumValid(int nNumber, bool bIsFlatSeam, bool bIsStartPoint, bool bIsFreePoint)
{
	// ��� ƽ�� ���ɶ˵�	��������λ�ã�ȡ�˵�
	// �յ� ƽ�� ���ɶ˵�	��������λ�ã�ȡ�˵�
	// ��� ƽ�� ����˵�	�ĸ�����λ�ã������ཻ
	// ��� ���� ����˵�	�ĸ�����λ�ã������ཻ
	// �յ� ƽ�� ����˵�	�ĸ�����λ�ã������ཻ
	// �յ� ���� ���ɶ˵�	��������λ�ã����������ཻ�Ͳ��
	int nFlatSeamFreeNum = 2;
	int nThreePlaneEndPointNum = 4;
	int nStandSeamFreeEndNum = 6;
	int nStandSeamFreeStartNum = 4;
	bool bIsValid = false;
	if (bIsFlatSeam && bIsFreePoint) // ƽ�� ���ɶ˵�
	{
		bIsValid = nFlatSeamFreeNum == nNumber ? true : false;
	}
	else if (!bIsFreePoint)			// ����˵�
	{
		bIsValid = nThreePlaneEndPointNum == nNumber ? true : false;
	}
	else if (!bIsStartPoint && !bIsFlatSeam && bIsFreePoint)	// �յ� ���� ���ɶ˵�
	{
		bIsValid = nStandSeamFreeEndNum == nNumber ? true : false;
	}
	else if (bIsStartPoint && !bIsFlatSeam && bIsFreePoint)
	{
		bIsValid = nStandSeamFreeStartNum == nNumber ? true : false;
	}
	else
	{
		XUI::MesBox::PopOkCancel("�������󣬼������������ʧ��! {0} {1} {2} {3}", nNumber, bIsFlatSeam, bIsStartPoint, bIsFreePoint);
	}
	return bIsValid;
}

bool GenericWeld::CalcFreePoint(const std::vector<T_TEACH_DATA>& tTeachData, T_ALGORITHM_POINT& tStartPtn)
{
	tStartPtn.dCoorX = m_vtTeachResult[tTeachData[0].nMeasurePtnNo].tKeyPtn3D.x;
	tStartPtn.dCoorY = m_vtTeachResult[tTeachData[0].nMeasurePtnNo].tKeyPtn3D.y;
	tStartPtn.dCoorZ = m_vtTeachResult[tTeachData[0].nMeasurePtnNo].tKeyPtn3D.z;
	return true;
}

bool GenericWeld::CalcInterferePoint(const std::vector<T_TEACH_DATA>& tTeachData, T_ALGORITHM_POINT& tAlgPoint)
{
	XiAlgorithm alg;
	T_ALGORITHM_LINE_PARA tAlgLine;
	T_ALGORITHM_PLANE_PARAM tPlane1;
	T_ALGORITHM_PLANE_PARAM tPlane2;
	T_ALGORITHM_PLANE_PARAM tPlane3;
	std::vector<T_ALGORITHM_POINT> vtFirstPoints(0);
	std::vector<T_ALGORITHM_POINT> vtSecondPoints(0);
	std::vector<T_ALGORITHM_POINT> vtThirdPoints(0);
	std::vector<T_TEACH_RESULT> vtTeachResult(0);

	// ��ȡtTeachData��Ӧ�����в����������
	for (int i = 0; i < tTeachData.size(); i++)
	{
		vtTeachResult.push_back(m_vtTeachResult[tTeachData[i].nMeasurePtnNo]);
	}

	// ��ȡ�������ϵĵ�
	//XiPointAddToAlgPtn(vtTeachResult[0].vtLeftPtns3D, vtFirstPoints);
	//XiPointAddToAlgPtn(vtTeachResult[1].vtLeftPtns3D, vtFirstPoints);
	//XiPointAddToAlgPtn(vtTeachResult[2].vtLeftPtns3D, vtSecondPoints);
	//XiPointAddToAlgPtn(vtTeachResult[3].vtLeftPtns3D, vtSecondPoints);
	////XiPointAddToAlgPtn(vtTeachResult[0].vtRightPtns3D, vtThirdPoints);
	//XiPointAddToAlgPtn(vtTeachResult[1].vtRightPtns3D, vtThirdPoints);
	//XiPointAddToAlgPtn(vtTeachResult[2].vtRightPtns3D, vtThirdPoints);
	////XiPointAddToAlgPtn(vtTeachResult[3].vtRightPtns3D, vtThirdPoints);

	XiPointAddToAlgPtn(vtTeachResult[0].vtRightPtns3D, vtFirstPoints);
	XiPointAddToAlgPtn(vtTeachResult[1].vtRightPtns3D, vtFirstPoints);
	XiPointAddToAlgPtn(vtTeachResult[2].vtRightPtns3D, vtSecondPoints);
	XiPointAddToAlgPtn(vtTeachResult[3].vtRightPtns3D, vtSecondPoints);
	XiPointAddToAlgPtn(vtTeachResult[0].vtLeftPtns3D, vtThirdPoints);
	XiPointAddToAlgPtn(vtTeachResult[2].vtLeftPtns3D, vtThirdPoints);

	// �������ƽ��
	tPlane1 = alg.CalcPlaneParamRansac(vtFirstPoints, 0.8);
	tPlane2 = alg.CalcPlaneParamRansac(vtSecondPoints, 0.8);
	tPlane3 = alg.CalcPlaneParamRansac(vtThirdPoints, 0.8);

	// �����ཻ�õ�����
	tAlgLine = PlanePlaneInterLine(tPlane1, tPlane2);

	// ���ߺ͵������ཻ�ýǵ�
	if (false == alg.LinePlateInterSection(tAlgLine, tPlane3, tAlgPoint))
	{
		XiMessageBox("����˵�: �����ཻ����ʧ��!");
		return false;
	}

	return true;
}

bool GenericWeld::CalcInterferePointStand(const std::vector<T_TEACH_DATA>& tTeachData, T_ALGORITHM_POINT& tAlgPoint)
{
	XiAlgorithm alg;
	T_ALGORITHM_LINE_PARA tAlgLine;
	T_ALGORITHM_PLANE_PARAM tPlane1;
	T_ALGORITHM_PLANE_PARAM tPlane2;
	T_ALGORITHM_PLANE_PARAM tPlane3;
	std::vector<T_ALGORITHM_POINT> vtFirstPoints(0);
	std::vector<T_ALGORITHM_POINT> vtSecondPoints(0);
	std::vector<T_ALGORITHM_POINT> vtThirdPoints(0);
	std::vector<T_TEACH_RESULT> vtTeachResult(0);

	// ��ȡtTeachData��Ӧ�����в����������
	for (int i = 0; i < tTeachData.size(); i++)
	{
		vtTeachResult.push_back(m_vtTeachResult[tTeachData[i].nMeasurePtnNo]);
	}

	// ��ȡ�������ϵĵ�
	//XiPointAddToAlgPtn(vtTeachResult[0].vtLeftPtns3D, vtFirstPoints);
	//XiPointAddToAlgPtn(vtTeachResult[1].vtLeftPtns3D, vtFirstPoints);
	//XiPointAddToAlgPtn(vtTeachResult[2].vtLeftPtns3D, vtSecondPoints);
	//XiPointAddToAlgPtn(vtTeachResult[3].vtLeftPtns3D, vtSecondPoints);
	////XiPointAddToAlgPtn(vtTeachResult[0].vtRightPtns3D, vtThirdPoints);
	//XiPointAddToAlgPtn(vtTeachResult[1].vtRightPtns3D, vtThirdPoints);
	//XiPointAddToAlgPtn(vtTeachResult[2].vtRightPtns3D, vtThirdPoints);
	////XiPointAddToAlgPtn(vtTeachResult[3].vtRightPtns3D, vtThirdPoints);

	XiPointAddToAlgPtn(vtTeachResult[0].vtRightPtns3D, vtFirstPoints);
	XiPointAddToAlgPtn(vtTeachResult[1].vtRightPtns3D, vtFirstPoints);
	XiPointAddToAlgPtn(vtTeachResult[2].vtRightPtns3D, vtSecondPoints);
	XiPointAddToAlgPtn(vtTeachResult[3].vtRightPtns3D, vtSecondPoints);
	XiPointAddToAlgPtn(vtTeachResult[0].vtLeftPtns3D, vtThirdPoints);
	XiPointAddToAlgPtn(vtTeachResult[2].vtLeftPtns3D, vtThirdPoints);

	// �������ƽ��
	tPlane1 = alg.CalcPlaneParamRansac(vtFirstPoints, 0.8);
	tPlane2 = alg.CalcPlaneParamRansac(vtSecondPoints, 0.8);
	tPlane3 = alg.CalcPlaneParamRansac(vtThirdPoints, 0.8);

	// �����ཻ�õ�����
	tAlgLine = PlanePlaneInterLine(tPlane1, tPlane2);

	// ���ߺ͵������ཻ�ýǵ�
	if (false == alg.LinePlateInterSection(tAlgLine, tPlane3, tAlgPoint))
	{
		XiMessageBox("����˵�: �����ཻ����ʧ��!");
		return false;
	}

	return true;
}


bool GenericWeld::CalcFreeStartPointStand(const std::vector<T_TEACH_DATA>& tTeachData, T_ALGORITHM_POINT& tAlgPoint)
{
	XiAlgorithm alg;
	T_ALGORITHM_LINE_PARA tAlgLine;
	T_ALGORITHM_PLANE_PARAM tPlane;
	T_ALGORITHM_PLANE_PARAM tPlane1;
	T_ALGORITHM_PLANE_PARAM tPlane2;
	std::vector<T_ALGORITHM_POINT> vtFirstPoints(0);
	std::vector<T_ALGORITHM_POINT> vtSecondPoints(0);
	std::vector<T_TEACH_RESULT> vtTeachResult(0);
	double dMaxHeight = -99999.0;

	// ��ȡtTeachData��Ӧ�����в����������
	for (int i = 0; i < tTeachData.size(); i++)
	{
		vtTeachResult.push_back(m_vtTeachResult[tTeachData[i].nMeasurePtnNo]);
	}

	// ��ȡ���������ϵĵ� (���������ཻ ȡ��ߵ� )
	XiPointAddToAlgPtn(vtTeachResult[0].vtLeftPtns3D, vtFirstPoints);
	XiPointAddToAlgPtn(vtTeachResult[1].vtLeftPtns3D, vtFirstPoints);
	XiPointAddToAlgPtn(vtTeachResult[2].vtLeftPtns3D, vtSecondPoints);
	XiPointAddToAlgPtn(vtTeachResult[3].vtLeftPtns3D, vtSecondPoints);

	// �����������ƽ��
	tPlane1 = alg.CalcPlaneParamRansac(vtFirstPoints, 0.8);
	tPlane2 = alg.CalcPlaneParamRansac(vtSecondPoints, 0.8);

	// �����ཻ�õ�����
	tAlgLine = PlanePlaneInterLine(tPlane1, tPlane2);

	// ��ȡ�������߶� ȡ�ߵ�
	dMaxHeight = vtTeachResult[0].tKeyPtn3D.z * (double)m_nRobotInstallDir > dMaxHeight ? vtTeachResult[0].tKeyPtn3D.z * (double)m_nRobotInstallDir : dMaxHeight;
	dMaxHeight = vtTeachResult[1].tKeyPtn3D.z * (double)m_nRobotInstallDir > dMaxHeight ? vtTeachResult[1].tKeyPtn3D.z * (double)m_nRobotInstallDir : dMaxHeight;
	dMaxHeight = vtTeachResult[2].tKeyPtn3D.z * (double)m_nRobotInstallDir > dMaxHeight ? vtTeachResult[2].tKeyPtn3D.z * (double)m_nRobotInstallDir : dMaxHeight;
	dMaxHeight = vtTeachResult[3].tKeyPtn3D.z * (double)m_nRobotInstallDir > dMaxHeight ? vtTeachResult[3].tKeyPtn3D.z * (double)m_nRobotInstallDir : dMaxHeight;
	dMaxHeight *= (double)m_nRobotInstallDir;

	// ������ƽ��z=height�ཻ �ö˵�
	memset(&tPlane, 0, sizeof(tPlane));
	tPlane.c = 1;
	tPlane.d = -dMaxHeight;
	if (false == alg.LinePlateInterSection(tAlgLine, tPlane, tAlgPoint))
	{
		XiMessageBox("�����������: �����ཻ����ʧ��!");
		return false;
	}

	return true;
}

bool GenericWeld::CalcFreeEndPointStand(const std::vector<T_TEACH_DATA>& tTeachData, T_ALGORITHM_POINT& tAlgPoint)
{
	bool bBasedOnHeight = false; // true:�����߶�ȡ�ߵ� false:�����߶�ȡ�͵� (���Ϊ���������� ֱ����ƽ�潻�� ȡ�ϵͽ���)
	XiAlgorithm alg;
	T_ALGORITHM_LINE_PARA tAlgLine;
	T_ALGORITHM_PLANE_PARAM tPlane;
	T_ALGORITHM_PLANE_PARAM tPlane1;
	T_ALGORITHM_PLANE_PARAM tPlane2;
	std::vector<T_ALGORITHM_POINT> vtFirstPoints(0);
	std::vector<T_ALGORITHM_POINT> vtSecondPoints(0);
	std::vector<T_TEACH_RESULT> vtTeachResult(0);
	double dMinHeight = 0.0;
	double dMaxHeight = 0.0;
	double dEndHeight = 0.0;

	// ��ȡtTeachData��Ӧ�����в����������
	for (int i = 0; i < tTeachData.size(); i++)
	{
		vtTeachResult.push_back(m_vtTeachResult[tTeachData[i].nMeasurePtnNo]);
	}

	// ��ȡ���������ϵĵ� ȡƽ���������������ϵ㣬��ȥ��ԭ��ÿ����������Զ��ǵ�����������������㣬�����ƽ�棬��׼ȷ
	XiPointAddToAlgPtn(vtTeachResult[0].vtRightPtns3D, vtFirstPoints);
	vtFirstPoints.pop_back();
	vtFirstPoints.pop_back();
	XiPointAddToAlgPtn(vtTeachResult[1].vtRightPtns3D, vtFirstPoints);
	vtFirstPoints.pop_back();
	vtFirstPoints.pop_back();
	XiPointAddToAlgPtn(vtTeachResult[4].vtLeftPtns3D, vtFirstPoints);
	vtFirstPoints.pop_back();
	vtFirstPoints.pop_back();

	XiPointAddToAlgPtn(vtTeachResult[2].vtRightPtns3D, vtSecondPoints);
	vtSecondPoints.pop_back();
	vtSecondPoints.pop_back();
	XiPointAddToAlgPtn(vtTeachResult[3].vtRightPtns3D, vtSecondPoints);
	vtSecondPoints.pop_back();
	vtSecondPoints.pop_back();
	XiPointAddToAlgPtn(vtTeachResult[5].vtLeftPtns3D, vtSecondPoints);
	vtSecondPoints.pop_back();
	vtSecondPoints.pop_back();
	//// ��ȡ���������ϵĵ�
	//XiPointAddToAlgPtn(vtTeachResult[0].vtLeftPtns3D, vtFirstPoints);
	//XiPointAddToAlgPtn(vtTeachResult[1].vtLeftPtns3D, vtFirstPoints);
	//XiPointAddToAlgPtn(vtTeachResult[2].vtLeftPtns3D, vtSecondPoints);
	//XiPointAddToAlgPtn(vtTeachResult[3].vtLeftPtns3D, vtSecondPoints);

	// �����������ƽ��
	tPlane1 = alg.CalcPlaneParamRansac(vtFirstPoints, 0.8);
	tPlane2 = alg.CalcPlaneParamRansac(vtSecondPoints, 0.8);

	// �����ཻ�õ�����
	tAlgLine = PlanePlaneInterLine(tPlane1, tPlane2);

	// ��ȡ�����߶�
	dMinHeight = vtTeachResult[4].tKeyPtn3D.z * (double)m_nRobotInstallDir;
	dMaxHeight = vtTeachResult[5].tKeyPtn3D.z * (double)m_nRobotInstallDir;
	if (dMinHeight > dMaxHeight)
	{
		swap(dMinHeight, dMaxHeight);
	}
	dEndHeight = true == bBasedOnHeight ? dMaxHeight : dMinHeight;
	dEndHeight *= (double)m_nRobotInstallDir;

	// ������ƽ��z=height�ཻ �ö˵�
	memset(&tPlane, 0, sizeof(tPlane));
	tPlane.c = 1;
	tPlane.d = -dEndHeight;
	if (false == alg.LinePlateInterSection(tAlgLine, tPlane, tAlgPoint))
	{
		XiMessageBox("���������յ�: �����ཻ����ʧ��!");
		return false;
	}

	return true;
}

void GenericWeld::CheckSeamDataEndpointType(double dDisThreshold)
{
	int nSeamNum = m_vtWeldSeamInfo.size();
	vector<int> vnStartIdx(nSeamNum); // ���������
	vector<int> vnEndIdx(nSeamNum); // �յ�������
	vector<CvPoint3D64f> vtPtns(0); // �㼯 ������غ�
	vector<int> vnPtnNum(0); // �㼯�и��������

	//// �ƶ���ʶ���
	////������ֽ�Ϊ�̺���
	//double dMaxWeldLenThreshold = 1500.0;
	//std::vector<WeldLineInfo> vtWeldSeamInfo(m_vtWeldSeamInfo);
	//m_vtWeldSeamInfo.clear();
	//for (int nSeamNo = 0; nSeamNo < nSeamNum; nSeamNo++)
	//{
	//	std::vector<WeldLineInfo> vtInfos;
	//	WeldSeamSegmentation(vtWeldSeamInfo[nSeamNo], dMaxWeldLenThreshold, vtInfos);
	//	m_vtWeldSeamInfo.insert(m_vtWeldSeamInfo.end(), vtInfos.begin(), vtInfos.end());
	//}
	//m_vtWeldSeamData.clear();
	//for (int i = 0; i < m_vtWeldSeamInfo.size(); i++)
	//{
	//	m_vtWeldSeamData.push_back(m_vtWeldSeamInfo[i].tWeldLine);
	//}

	//// ����ֶκ�����
	//CString sOutFileName = "GraphData\\PointCloudIdentifyReault.txt";
	//FILE* pf = fopen(sOutFileName.GetBuffer(), "w");
	//for (int i = 0; i < m_vtWeldSeamData.size(); i++)
	//{
	//	int n1 = m_vtWeldSeamData.size();
	//	int n2 = m_vtWeldSeamInfo.size();
	//	LineOrCircularArcWeldingLine tWeld = m_vtWeldSeamData[i];
	//	WeldLineInfo tWeldInfo = m_vtWeldSeamInfo[i];
	//	fprintf(pf, "%d%4d%4d%11.3lf %4d %11.3lf%11.3lf%11.3lf %11.3lf%11.3lf%11.3lf %11.3lf%11.3lf%11.3lf %11.3lf%11.3lf%11.3lf %11.3lf%11.3lf%11.3lf %4d%4d %4d%4d%4d %4d%11.3lf%11.3lf%4d\n",
	//		i, tWeld.IsArc, tWeld.isClockwise, tWeld.ZSide, tWeld.isLeft,
	//		tWeld.CenterPoint.x, tWeld.CenterPoint.y, tWeld.CenterPoint.z,
	//		tWeld.StartPoint.x, tWeld.StartPoint.y, tWeld.StartPoint.z,
	//		tWeld.EndPoint.x, tWeld.EndPoint.y, tWeld.EndPoint.z,
	//		tWeld.StartNormalVector.x, tWeld.StartNormalVector.y, tWeld.StartNormalVector.z,
	//		tWeld.EndNormalVector.x, tWeld.EndNormalVector.y, tWeld.EndNormalVector.z,
	//		tWeld.StartPointType, tWeld.EndPointType,
	//		tWeldInfo.tAtrribute.nIsDoubleWelding, tWeldInfo.tAtrribute.nStartWrapType, tWeldInfo.tAtrribute.nEndWrapType,
	//		tWeldInfo.tAtrribute.nWeldAngleSize, tWeldInfo.tAtrribute.dStartHoleSize, tWeldInfo.tAtrribute.dEndHoleSize, tWeldInfo.tAtrribute.nRobotSelete);
	//}
	//fclose(pf);


	for (int nSeamNo = 0; nSeamNo < nSeamNum; nSeamNo++)
	{
		LineOrCircularArcWeldingLine tSeam = m_vtWeldSeamData[nSeamNo];
		bool bExistStart = false;
		bool bExistEnd = false;
		for (int nPtnNo = 0; nPtnNo < vtPtns.size(); nPtnNo++)
		{
			double dDisS = TwoPointDis(vtPtns[nPtnNo].x, vtPtns[nPtnNo].y, vtPtns[nPtnNo].z, tSeam.StartPoint.x, tSeam.StartPoint.y, tSeam.StartPoint.z);
			double dDisE = TwoPointDis(vtPtns[nPtnNo].x, vtPtns[nPtnNo].y, vtPtns[nPtnNo].z, tSeam.EndPoint.x, tSeam.EndPoint.y, tSeam.EndPoint.z);
			if (false == bExistStart && dDisS < dDisThreshold)
			{
				bExistStart = true;
				vnPtnNum[nPtnNo] += 1; 
				vnStartIdx[nSeamNo] = nPtnNo;
			}			
			if (false == bExistEnd && dDisE < dDisThreshold)
			{
				bExistEnd = true;
				vnPtnNum[nPtnNo] += 1;
				vnEndIdx[nSeamNo] = nPtnNo;
			}
			if (bExistStart && bExistEnd) break;
		}
		if (!bExistStart)
		{
			vnStartIdx[nSeamNo] = vtPtns.size();
			vtPtns.push_back(tSeam.StartPoint); 
			vnPtnNum.push_back(1);
		}
		if (!bExistEnd)
		{
			vnEndIdx[nSeamNo] = vtPtns.size();
			vtPtns.push_back(tSeam.EndPoint);
			vnPtnNum.push_back(1);
		}
	}
	for (int nSeamNo = 0; nSeamNo < nSeamNum; nSeamNo++)
	{
		if (vnPtnNum[vnStartIdx[nSeamNo]] > 1 && false == m_vtWeldSeamData[nSeamNo].StartPointType) // �����������ϵĵ���ͬΪ����˵�
		{
			//���޸�
			XUI::MesBox::PopInfo("����{0}������ʶ��Ϊ���ɣ��Զ�����", nSeamNo);
			//XiMessageBox("����%d������ʶ��Ϊ���ɣ��Զ�����", nSeamNo);
			m_vtWeldSeamData[nSeamNo].StartPointType = true;
			m_vtWeldSeamInfo[nSeamNo].tWeldLine.StartPointType = true;
		}
		if (vnPtnNum[vnEndIdx[nSeamNo]] > 1 && false == m_vtWeldSeamData[nSeamNo].EndPointType) // �����������ϵĵ���ͬΪ����˵�
		{
			//���޸�
			XUI::MesBox::PopInfo("����{0}�յ����ʶ��Ϊ���ɣ��Զ�����", nSeamNo);
			//XiMessageBox("����%d�յ����ʶ��Ϊ���ɣ��Զ�����", nSeamNo);
			m_vtWeldSeamData[nSeamNo].EndPointType = true;
			m_vtWeldSeamInfo[nSeamNo].tWeldLine.EndPointType = true;
		}
	}
}

void GenericWeld::Segmentation()
{
	//������ֽ�Ϊ�̺���
	int nSeamNum = m_vtWeldSeamInfo.size();
	std::vector<WeldLineInfo> vtWeldSeamInfo(m_vtWeldSeamInfo);
	m_vtWeldSeamInfo.clear();
	for (int nSeamNo = 0; nSeamNo < nSeamNum; nSeamNo++)
	{
		std::vector<WeldLineInfo> vtInfos;
		WeldSeamSegmentation(vtWeldSeamInfo[nSeamNo], m_dMaxWeldLen, vtInfos);
		m_vtWeldSeamInfo.insert(m_vtWeldSeamInfo.end(), vtInfos.begin(), vtInfos.end());
	}
	m_vtWeldSeamData.clear();
	for (int i = 0; i < m_vtWeldSeamInfo.size(); i++)
	{
		m_vtWeldSeamData.push_back(m_vtWeldSeamInfo[i].tWeldLine);
	}

	// ����ֶκ�����
	CString sOutFileName = OUTPUT_PATH + m_ptUnit->m_tContralUnit.strUnitName + "\\" + POINT_CLOUD_IDENTIFY_RESULT;;
	FILE* pf = fopen(sOutFileName.GetBuffer(), "w");
	for (int i = 0; i < m_vtWeldSeamData.size(); i++)
	{
		int n1 = m_vtWeldSeamData.size();
		int n2 = m_vtWeldSeamInfo.size();
		LineOrCircularArcWeldingLine tWeld = m_vtWeldSeamData[i];
		WeldLineInfo tWeldInfo = m_vtWeldSeamInfo[i];
		fprintf(pf, "%d%4d%4d%11.3lf %4d %11.3lf%11.3lf%11.3lf %11.3lf%11.3lf%11.3lf %11.3lf%11.3lf%11.3lf %11.3lf%11.3lf%11.3lf %11.3lf%11.3lf%11.3lf %4d%4d %4d%4d%4d %11.3lf%11.3lf%11.3lf%4d 0 0\n",
			i, tWeld.IsArc, tWeld.isClockwise, tWeld.ZSide, tWeld.isLeft,
			tWeld.CenterPoint.x, tWeld.CenterPoint.y, tWeld.CenterPoint.z,
			tWeld.StartPoint.x, tWeld.StartPoint.y, tWeld.StartPoint.z,
			tWeld.EndPoint.x, tWeld.EndPoint.y, tWeld.EndPoint.z,
			tWeld.StartNormalVector.x, tWeld.StartNormalVector.y, tWeld.StartNormalVector.z,
			tWeld.EndNormalVector.x, tWeld.EndNormalVector.y, tWeld.EndNormalVector.z,
			tWeld.StartPointType, tWeld.EndPointType,
			tWeldInfo.tAtrribute.nIsDoubleWelding, tWeldInfo.tAtrribute.nStartWrapType, tWeldInfo.tAtrribute.nEndWrapType,
			tWeldInfo.tAtrribute.nWeldAngleSize, tWeldInfo.tAtrribute.dStartHoleSize, tWeldInfo.tAtrribute.dEndHoleSize, 
			tWeldInfo.tAtrribute.nRobotSelete);

	}
	fclose(pf);
}

//void GenericWeld::WeldSeamSegmentation(WeldLineInfo tWeldSeamInfo, double dLenThreshold, std::vector<WeldLineInfo>& vtWeldSeams)
//{
//	double dOffset = m_dJointLen;
//	vtWeldSeams.clear();
//	WeldLineInfo tSeamInfo = tWeldSeamInfo;
//	double dWeldSeamLen = TwoPointDis(
//		tSeamInfo.tWeldLine.StartPoint.x, tSeamInfo.tWeldLine.StartPoint.y, tSeamInfo.tWeldLine.StartPoint.z,
//		tSeamInfo.tWeldLine.EndPoint.x, tSeamInfo.tWeldLine.EndPoint.y, tSeamInfo.tWeldLine.EndPoint.z);
//	if (m_dHandEyeDis > 1.0 || dWeldSeamLen < dLenThreshold)
//	{
//		vtWeldSeams.push_back(tWeldSeamInfo);
//		return;
//	}
//	else
//	{
//		int nSegmNum = dWeldSeamLen / dLenThreshold + 1;
//		double dSegmLen = dWeldSeamLen / (double)nSegmNum;
//		double dDisX = tSeamInfo.tWeldLine.EndPoint.x - tSeamInfo.tWeldLine.StartPoint.x;
//		double dDisY = tSeamInfo.tWeldLine.EndPoint.y - tSeamInfo.tWeldLine.StartPoint.y;
//		double dDisZ = tSeamInfo.tWeldLine.EndPoint.z - tSeamInfo.tWeldLine.StartPoint.z;
//		// ��һ��
//		tSeamInfo = tWeldSeamInfo;
//		tSeamInfo.tWeldLine.EndPoint.x = tSeamInfo.tWeldLine.StartPoint.x + (dSegmLen - m_dEndpointSearchDis + dOffset) * dDisX / dWeldSeamLen;
//		tSeamInfo.tWeldLine.EndPoint.y = tSeamInfo.tWeldLine.StartPoint.y + (dSegmLen - m_dEndpointSearchDis + dOffset) * dDisY / dWeldSeamLen;
//		tSeamInfo.tWeldLine.EndPoint.z = tSeamInfo.tWeldLine.StartPoint.z + (dSegmLen - m_dEndpointSearchDis + dOffset) * dDisZ / dWeldSeamLen;
//		tSeamInfo.tWeldLine.EndPointType = false; // �����
//		tSeamInfo.tAtrribute.dEndHoleSize = 0; // 
//		vtWeldSeams.push_back(tSeamInfo);
//
//		// �м��
//		for (int i = 1; i < nSegmNum - 1; i++)
//		{
//			tSeamInfo = tWeldSeamInfo;
//			tSeamInfo.tWeldLine.EndPoint.x = tSeamInfo.tWeldLine.StartPoint.x + ((dSegmLen * (i + 1)) - m_dEndpointSearchDis + dOffset) * dDisX / dWeldSeamLen;
//			tSeamInfo.tWeldLine.EndPoint.y = tSeamInfo.tWeldLine.StartPoint.y + ((dSegmLen * (i + 1)) - m_dEndpointSearchDis + dOffset) * dDisY / dWeldSeamLen;
//			tSeamInfo.tWeldLine.EndPoint.z = tSeamInfo.tWeldLine.StartPoint.z + ((dSegmLen * (i + 1)) - m_dEndpointSearchDis + dOffset) * dDisZ / dWeldSeamLen;
//			tSeamInfo.tWeldLine.EndPointType = false; // �����
//			tSeamInfo.tAtrribute.dEndHoleSize = 0; // 
//
//			tSeamInfo.tWeldLine.StartPoint.x = tSeamInfo.tWeldLine.StartPoint.x + ((dSegmLen * i) + m_dEndpointSearchDis - dOffset) * dDisX / dWeldSeamLen;
//			tSeamInfo.tWeldLine.StartPoint.y = tSeamInfo.tWeldLine.StartPoint.y + ((dSegmLen * i) + m_dEndpointSearchDis - dOffset) * dDisY / dWeldSeamLen;
//			tSeamInfo.tWeldLine.StartPoint.z = tSeamInfo.tWeldLine.StartPoint.z + ((dSegmLen * i) + m_dEndpointSearchDis - dOffset) * dDisZ / dWeldSeamLen;
//			tSeamInfo.tWeldLine.StartPointType = false; // �����
//			tSeamInfo.tAtrribute.dStartHoleSize = 0; // 
//
//			vtWeldSeams.push_back(tSeamInfo);
//		}
//
//		// ���һ��
//		tSeamInfo = tWeldSeamInfo;
//		tSeamInfo.tWeldLine.StartPoint.x = tSeamInfo.tWeldLine.StartPoint.x + ((dSegmLen * (nSegmNum - 1)) + m_dEndpointSearchDis - dOffset) * dDisX / dWeldSeamLen;
//		tSeamInfo.tWeldLine.StartPoint.y = tSeamInfo.tWeldLine.StartPoint.y + ((dSegmLen * (nSegmNum - 1)) + m_dEndpointSearchDis - dOffset) * dDisY / dWeldSeamLen;
//		tSeamInfo.tWeldLine.StartPoint.z = tSeamInfo.tWeldLine.StartPoint.z + ((dSegmLen * (nSegmNum - 1)) + m_dEndpointSearchDis - dOffset) * dDisZ / dWeldSeamLen;
//		tSeamInfo.tWeldLine.StartPointType = false; // �����
//		tSeamInfo.tAtrribute.dStartHoleSize = 0; // 
//
//		vtWeldSeams.push_back(tSeamInfo);
//	}
//}


void GenericWeld::WeldSeamSegmentation(WeldLineInfo tWeldSeamInfo, double dLenThreshold, std::vector<WeldLineInfo>& vtWeldSeams)
{
	double dOffset = m_dJointLen / 2.0;
	vtWeldSeams.clear();
	WeldLineInfo tSeamInfo = tWeldSeamInfo;
	double dWeldSeamLen = TwoPointDis(
		tSeamInfo.tWeldLine.StartPoint.x, tSeamInfo.tWeldLine.StartPoint.y, tSeamInfo.tWeldLine.StartPoint.z,
		tSeamInfo.tWeldLine.EndPoint.x, tSeamInfo.tWeldLine.EndPoint.y, tSeamInfo.tWeldLine.EndPoint.z);
	if (m_dHandEyeDis > 1.0 || dWeldSeamLen < dLenThreshold || WeldingLineIsArc(tWeldSeamInfo.tWeldLine))
	{
		vtWeldSeams.push_back(tWeldSeamInfo);
		return;
	}
	else
	{
		int nSegmNum = dWeldSeamLen / dLenThreshold + 1;
		double dSegmLen = dWeldSeamLen / (double)nSegmNum;
		double dDisX = tSeamInfo.tWeldLine.EndPoint.x - tSeamInfo.tWeldLine.StartPoint.x;
		double dDisY = tSeamInfo.tWeldLine.EndPoint.y - tSeamInfo.tWeldLine.StartPoint.y;
		double dDisZ = tSeamInfo.tWeldLine.EndPoint.z - tSeamInfo.tWeldLine.StartPoint.z;
		// ��һ��
		tSeamInfo = tWeldSeamInfo;
		tSeamInfo.tWeldLine.EndPoint.x = tSeamInfo.tWeldLine.StartPoint.x + (dSegmLen + dOffset) * dDisX / dWeldSeamLen;
		tSeamInfo.tWeldLine.EndPoint.y = tSeamInfo.tWeldLine.StartPoint.y + (dSegmLen + dOffset) * dDisY / dWeldSeamLen;
		tSeamInfo.tWeldLine.EndPoint.z = tSeamInfo.tWeldLine.StartPoint.z + (dSegmLen + dOffset) * dDisZ / dWeldSeamLen;
		tSeamInfo.tWeldLine.EndPointType = 0; // �ֶδ�϶˵�����
		tSeamInfo.tAtrribute.dEndHoleSize = 0; 
		tSeamInfo.tAtrribute.nEndWrapType = 0; // ��϶˵㲻����
		vtWeldSeams.push_back(tSeamInfo);

		// �м��
		for (int i = 1; i < nSegmNum - 1; i++)
		{
			tSeamInfo = tWeldSeamInfo;
			tSeamInfo.tWeldLine.EndPoint.x = tSeamInfo.tWeldLine.StartPoint.x + ((dSegmLen * (i + 1)) + dOffset) * dDisX / dWeldSeamLen;
			tSeamInfo.tWeldLine.EndPoint.y = tSeamInfo.tWeldLine.StartPoint.y + ((dSegmLen * (i + 1)) + dOffset) * dDisY / dWeldSeamLen;
			tSeamInfo.tWeldLine.EndPoint.z = tSeamInfo.tWeldLine.StartPoint.z + ((dSegmLen * (i + 1)) + dOffset) * dDisZ / dWeldSeamLen;
			tSeamInfo.tWeldLine.EndPointType = 0; // �ֶδ�϶˵�����
			tSeamInfo.tAtrribute.dEndHoleSize = 0;
			tSeamInfo.tAtrribute.nEndWrapType = 0; // ��϶˵㲻����

			tSeamInfo.tWeldLine.StartPoint.x = tSeamInfo.tWeldLine.StartPoint.x + ((dSegmLen * i) - dOffset) * dDisX / dWeldSeamLen;
			tSeamInfo.tWeldLine.StartPoint.y = tSeamInfo.tWeldLine.StartPoint.y + ((dSegmLen * i) - dOffset) * dDisY / dWeldSeamLen;
			tSeamInfo.tWeldLine.StartPoint.z = tSeamInfo.tWeldLine.StartPoint.z + ((dSegmLen * i) - dOffset) * dDisZ / dWeldSeamLen;
			tSeamInfo.tWeldLine.StartPointType = 0; // �ֶδ�϶˵�����
			tSeamInfo.tAtrribute.dStartHoleSize = 0;
			tSeamInfo.tAtrribute.nStartWrapType = 0; // ��϶˵㲻����

			vtWeldSeams.push_back(tSeamInfo);
		}

		// ���һ��
		tSeamInfo = tWeldSeamInfo;
		tSeamInfo.tWeldLine.StartPoint.x = tSeamInfo.tWeldLine.StartPoint.x + ((dSegmLen * (nSegmNum - 1)) - dOffset) * dDisX / dWeldSeamLen;
		tSeamInfo.tWeldLine.StartPoint.y = tSeamInfo.tWeldLine.StartPoint.y + ((dSegmLen * (nSegmNum - 1)) - dOffset) * dDisY / dWeldSeamLen;
		tSeamInfo.tWeldLine.StartPoint.z = tSeamInfo.tWeldLine.StartPoint.z + ((dSegmLen * (nSegmNum - 1)) - dOffset) * dDisZ / dWeldSeamLen;
		tSeamInfo.tWeldLine.StartPointType = 0; // �ֶδ�϶˵�����
		tSeamInfo.tAtrribute.dStartHoleSize = 0;
		tSeamInfo.tAtrribute.nStartWrapType = 0; // ��϶˵㲻����

		vtWeldSeams.push_back(tSeamInfo);
	}
}

void GenericWeld::SaveTeachDataEverySeam(int nGroupNo, int nSeamNo, double dExAxlePos, vector<T_TEACH_DATA>& vtTeachData)
{
	CString sFileName;
	sFileName.Format("%sGroup%d_Seam%d_TeachData.txt", m_sDataSavePath, nGroupNo, nSeamNo);
	FILE* pf = fopen(sFileName.GetBuffer(), "w");
	for (int i = 0; i < vtTeachData.size(); i++)
	{
#ifdef SINGLE_ROBOT
		fprintf(pf, "%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%4d%4d " + GetTeachDataString(vtTeachData[i]) + "\n",
			vtTeachData[i].tMeasureCoordGunTool.dX, vtTeachData[i].tMeasureCoordGunTool.dY + dExAxlePos, vtTeachData[i].tMeasureCoordGunTool.dZ,
			vtTeachData[i].tMeasureCoordGunTool.dRX, vtTeachData[i].tMeasureCoordGunTool.dRY, vtTeachData[i].tMeasureCoordGunTool.dRZ,
			vtTeachData[i].nWeldNo, vtTeachData[i].nMeasurePtnNo);
#else
		fprintf(pf, "%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%4d%4d " + GetTeachDataString(vtTeachData[i]) + "\n",
			vtTeachData[i].tMeasureCoordGunTool.dX + dExAxlePos, vtTeachData[i].tMeasureCoordGunTool.dY, vtTeachData[i].tMeasureCoordGunTool.dZ,
			vtTeachData[i].tMeasureCoordGunTool.dRX, vtTeachData[i].tMeasureCoordGunTool.dRY, vtTeachData[i].tMeasureCoordGunTool.dRZ,
			vtTeachData[i].nWeldNo, vtTeachData[i].nMeasurePtnNo);
#endif // SINGLE_ROBOT

	}
	fclose(pf);
}

bool GenericWeld::GetGantryPos(vector<LineOrCircularArcWeldingLine>& vtSeamGroup, double& dExAxlePos, double& dPartMaxHeight, double dTrackPos, double dPartMinHeight,
	CvPoint3D64f tTotalValue, CvPoint3D64f tTotalDirValue)
{
	// ����������Ӵ�ͣ��λ�ã�С������ʱ�ⲻ�᲻�˶���
/* *********************************��������
*					*
*					*
*					*
*					*
*					*(����ͣ��λ��)
*/
// ��֤��е�ۺ͹���֮��ʼ�ձ��������̬��������Zֵ
	double dExAxisZPos = dPartMinHeight - 2000.0;
	// û���ⲿZ��,����Z��ʱע�� //dExAxisZPos = 0.0;
	dExAxisZPos = 0.0;
	// ------------------
	dPartMaxHeight -= dExAxisZPos;
	// ����ƫ�Ʒ���
	double dAdjustDir = atan2(tTotalDirValue.y, tTotalDirValue.x) * 180 / 3.1415926;
	// �ж��Ƿ�Ϊ���������ң����жϷ������ܲ�������Ҫ�Ż�
	if (fabs(tTotalDirValue.x) < 0.5 && fabs(tTotalDirValue.y) < 0.5)
	{
		dAdjustDir = 180;
	}
	// �ȸ��ݷ������һ���̶�ƫ�ƵĴ����꣬���ݸ�λ�ü���������˶��Ƿ��ޣ��ٴμ����λ��
	XI_POINT tMachineCoor;
	tMachineCoor.x = tTotalValue.x / (vtSeamGroup.size() * 2);
	tMachineCoor.y = tTotalValue.y / (vtSeamGroup.size() * 2);
	tMachineCoor.z = dExAxisZPos;
	// ���������豸�ݶ���е�������ⲿ��ΪX���������򣩣�Y���������Ϊ������,����ʱȥ���������
	double dExAxisPosTrack = tMachineCoor.y;//tTotalValue.y / (vtSeamGroup.size() * 2);
	dExAxlePos = tMachineCoor.x;

	vector<double> vdExAxleOffset;
	vdExAxleOffset.clear();
	vdExAxleOffset.push_back(0.0);

	int i = 0;
	bool bIsContinueRun = true;
	for (i = 0; i < vdExAxleOffset.size(); i++)
	{
		XI_POINT tMachineCoor2 = tMachineCoor;
		double dAxisXAdjustDis = 600.0+ vdExAxleOffset.at(i);		
		
		if (!m_ptUnit->m_bSingleRobotWork)// ���
		{
			//0�Ż�е�ۣ� ������XҪ�ߵ����� = �����Ļ���������ϵX+ԭʼ��������������Yֵ+�������ĵ��������ĵľ���
			//1�Ż�е�ۣ� ������XҪ�ߵ����� = �����Ļ���������ϵX-ԭʼ��������������Yֵ-�������ĵ��������ĵľ���

			dAxisXAdjustDis = 600.0 + vdExAxleOffset.at(i);
#ifdef SINGLE_ROBOT
			if (0 == (m_pRobotDriver->m_nRobotNo%2))
			{
				dExAxisPosTrack = -1*(dTrackPos + m_ptUnit->m_dRobotBaseToGantryCtrDis);
			}
			else
			{
				dExAxisPosTrack = dTrackPos + m_ptUnit->m_dRobotBaseToGantryCtrDis;
			}
			dExAxlePos = tMachineCoor2.y;
#else
			dExAxisPosTrack = dTrackPos/*tMachineCoor2.y*/;
			dExAxlePos = tMachineCoor2.x /*- dAxisXAdjustDis*/;
#endif // SINGLE_ROBOT
			
		}
		else //����
		{

			double dDir = dAdjustDir;
			//dAxisXAdjustDis = 300.0 + vdExAxleOffset.at(i);
			dAxisXAdjustDis = 0.0 + vdExAxleOffset.at(i);
			tMachineCoor2.x += dAxisXAdjustDis * CosD(dDir);
			tMachineCoor2.y += dAxisXAdjustDis * SinD(dDir);
#ifdef SINGLE_ROBOT

			/*if (0 == (m_pRobotDriver->m_nRobotNo % 2))
			{
				dExAxisPosTrack = -1 * (dTrackPos + m_ptUnit->m_dGantryRobotOriginDis);
			}
			else
			{
				dExAxisPosTrack = dTrackPos + m_ptUnit->m_dGantryRobotOriginDis;
			}*/

			dExAxisPosTrack = tMachineCoor2.x;
			dExAxlePos = tMachineCoor2.y;
#else
			dExAxisPosTrack = tMachineCoor2.y;
			dExAxlePos = tMachineCoor2.x;
#endif // SINGLE_ROBOT		
		}
		//��������ʱʹ��   xia 20231117
		//dExAxisPosTrack = 0;
		//*******************
		double dMaxExAxlePos, dMinExAxlePos, dMaxTrackPos, dMinTraclPos;
#ifdef SINGLE_ROBOT
		dMaxExAxlePos = (double)m_pRobotDriver->m_tExternalAxle[1].lMaxPulseNum * m_pRobotDriver->m_tExternalAxle[1].dPulse;
		dMinExAxlePos = (double)m_pRobotDriver->m_tExternalAxle[1].lMinPulseNum * m_pRobotDriver->m_tExternalAxle[1].dPulse;

		dMaxTrackPos = (double)m_pRobotDriver->m_tExternalAxle[0].lMaxPulseNum * m_pRobotDriver->m_tExternalAxle[0].dPulse;
		dMinTraclPos = (double)m_pRobotDriver->m_tExternalAxle[0].lMinPulseNum * m_pRobotDriver->m_tExternalAxle[0].dPulse;
#else		
		dMaxExAxlePos = (double)m_pRobotDriver->m_tExternalAxle[0].lMaxPulseNum * m_pRobotDriver->m_tExternalAxle[0].dPulse;
		dMinExAxlePos = (double)m_pRobotDriver->m_tExternalAxle[0].lMinPulseNum * m_pRobotDriver->m_tExternalAxle[0].dPulse;

		dMaxTrackPos = (double)m_pRobotDriver->m_tExternalAxle[1].lMaxPulseNum * m_pRobotDriver->m_tExternalAxle[1].dPulse;	
		dMinTraclPos = (double)m_pRobotDriver->m_tExternalAxle[1].lMinPulseNum * m_pRobotDriver->m_tExternalAxle[1].dPulse;
#endif
		//xia
		/*if (dExAxlePos > dMaxExAxlePos || dExAxlePos < dMinExAxlePos)
		{
			bIsContinueRun = false;
			XiMessageBoxOk("������%d �ⲿ��%d Ŀ��λ��%.3lf �����趨����%.3lf - %.3lf!", m_pRobotDriver->m_nRobotNo, dExAxlePos, dMinExAxlePos, dMaxExAxlePos);
			continue;
		}

		if (dExAxisPosTrack > dMaxTrackPos || dExAxisPosTrack < dMinTraclPos)
		{
			bIsContinueRun = false;
			XiMessageBoxOk("������%d �ⲿ��%d Ŀ��λ��%.3lf �����趨����%.3lf - %.3lf!", m_pRobotDriver->m_nRobotNo, dExAxisPosTrack, dMinTraclPos, dMaxTrackPos);
			continue;
		}*/

		std::vector<T_ROBOT_COORS> vtRobotCoors, vtTotalRobotCoors;
		// ȥ���ⲿY���ⲿZֵ
		bIsContinueRun = true;
#ifdef SINGLE_ROBOT
		double dExAxisX = dExAxisPosTrack;
		double dExAxisY = dExAxlePos;
#else
		double dExAxisX = dExAxlePos;
		double dExAxisY = dExAxisPosTrack;
#endif // SINGLE_ROBOT

		if (!bIsContinueRun)
		{
			continue;
		}

		for (int nSm = 0; nSm < vtSeamGroup.size(); nSm++)
		{
#ifdef SINGLE_ROBOT
			vtSeamGroup[nSm].StartPoint.x -= dExAxisX;
			vtSeamGroup[nSm].EndPoint.x -= dExAxisX;
			vtSeamGroup[nSm].CenterPoint.x -= dExAxisX;
#else
			vtSeamGroup[nSm].StartPoint.y -= dExAxisY;
			vtSeamGroup[nSm].EndPoint.y -= dExAxisY;
			vtSeamGroup[nSm].CenterPoint.y -= dExAxisY;
#endif // SINGLE_ROBOT
			
			vtSeamGroup[nSm].StartPoint.z -= dExAxisZPos;
			vtSeamGroup[nSm].EndPoint.z -= dExAxisZPos;
			vtSeamGroup[nSm].ZSide -= dExAxisZPos;
		}
		break;
	}
	if (!bIsContinueRun)
	{
		XUI::MesBox::PopOkCancel("{0}�Ż����˼����λ��ʧ��", m_pRobotDriver->m_nRobotNo);
		return false;
	}
#ifdef SINGLE_ROBOT
	m_ptUnit->m_dExAxisYPos = dExAxlePos;
	m_ptUnit->m_dExAxisXPos = dExAxisPosTrack;
#else
	m_ptUnit->m_dExAxisYPos = dExAxisPosTrack;
	m_ptUnit->m_dExAxisXPos = dExAxlePos;
#endif
	m_ptUnit->m_dExAxisZPos = dExAxisZPos;
	return true;
}

bool GenericWeld::DetermineGroupMode(int nGroupNo)
{
	// ��ʱ�պ�Բ�������첻������ټ���
	for (int nWeldNo = 0; nWeldNo < m_vvtWeldLineInfoGroup[nGroupNo].size(); nWeldNo++)
	{
		if (10 == m_vvtWeldLineInfoGroup[nGroupNo][nWeldNo].tAtrribute.nStartWrapType)// ����
		{
			return false;
		}
		if (m_vvtWeldLineInfoGroup[nGroupNo][nWeldNo].tAtrribute.bWeldMode)
		{
			return true;
		}
		double dDis = TwoPointDis(m_vvtWeldLineInfoGroup[nGroupNo][nWeldNo].tWeldLine.StartPoint.x, m_vvtWeldLineInfoGroup[nGroupNo][nWeldNo].tWeldLine.StartPoint.y,
			m_vvtWeldLineInfoGroup[nGroupNo][nWeldNo].tWeldLine.StartPoint.z, m_vvtWeldLineInfoGroup[nGroupNo][nWeldNo].tWeldLine.EndPoint.x,
			m_vvtWeldLineInfoGroup[nGroupNo][nWeldNo].tWeldLine.EndPoint.y, m_vvtWeldLineInfoGroup[nGroupNo][nWeldNo].tWeldLine.EndPoint.z);
		if (dDis < 0.1 && WeldingLineIsArc(m_vvtWeldLineInfoGroup[nGroupNo][nWeldNo].tWeldLine))
		{
			return false;
		}
	}
	return false;
}

//int CScanInitModule::JudgeFindWeldEndpointMode(int nMethod)
//{
//	// �жϷ���: 1.��֪�յ�(�����յ㡢��ǰ������ʼ��β��), 2.�����۳���,����ʼλ������ʱҲ�������ζ˵㣬 3.���ӹ������ж�(�����յ�)
//	// �����жϷ���ȷ�����������м����������ν�β��
//	int bRst = 1;
//	switch (nMethod)
//	{
//	case 1: bRst = 2; break;
//	case 2: bRst = 1; break;
//	case 3: bRst = 1; break;
//	default:XiMessageBoxOk("�����ڸ����յ��жϷ���%d!"); break;
//	}
//	return bRst;
//}

bool GenericWeld::CalcAndTrackWeldingTry(int nGroupNo)
{
	// ���ٺ����������ݣ�����/�����õ���ʼ�����ݣ���С�ڸ������۳��ȣ�����β��
	double dStepDis = 3.0;
	vector<T_ROBOT_COORS> vtWeldCoors; // ����켣
	vector<T_ROBOT_COORS> vtReverseCoors; //����켣
	vector<int> vtWeldCoorsType;
	E_WELD_SEAM_TYPE eWeldSeamType;
	double dExPos = 0.0;
	
	
	// ����������nGroup��ÿ������
	for (int nWeldNo = 0; nWeldNo < m_vvtWeldLineInfoGroup[nGroupNo].size(); nWeldNo++)
	{
		// ��ʾֻȡһ���ڵ�һ�����죬�˴η�����Ǻ����Ϊһ�飬����������죬���ຸ�쵥����Ϊһ��
		
		if (nWeldNo > 0&& DetermineWarpMode(nGroupNo)>0)
		{
			break;
		}
		// ----------------------------
		m_pScanInit->m_pTraceModel->m_eStartWrapAngleType = E_WRAPANGLE_EMPTY_SINGLE;
		// �����˵����ͣ���������ǰ������ʼ��β���ݣ���֪�յ����ͣ�������ֻ������ʼ���ݣ����ٹ���������β�㡢���۳��Ⱥ��ӣ�
		// �����жϽ�β���;������ؼ��ν�β����
		vector<XI_POINT> vtpoints;
		vector<XI_POINT> vtCoutPoints;		
		vector<T_ANGLE_PULSE> vtWeldPulse;
		vector<T_ROBOT_COORS> vtCoutRobotCoors;
		m_vtTrackTheoryTrack.clear();
		int nScanEndpointNum = m_vvtWeldLineInfoGroup.at(nGroupNo).at(nWeldNo).tAtrribute.nScanEndpointNum;
		for (int i = 0; i < nScanEndpointNum; i++)
		{
			vtpoints.clear();
			CString strFileName;
			strFileName.Format("%s%s%sEndpointCoors-%d-%d.txt", OUTPUT_PATH, m_pRobotDriver->m_strRobotName, RECOGNITION_FOLDER, nGroupNo, i);
			if (!LoadPointsData(vtpoints, strFileName)) {
				//���޸�
				XUI::MesBox::PopInfo("{0}�ļ���������ʧ��", strFileName.GetBuffer());
				//XiMessageBox("%s �ļ���������ʧ��", strFileName);
				return false;
			}
			if (vtpoints.size() > 10) {
				// �Գ�ʼ�켣���������˲�
				CHECK_BOOL_RETURN(m_pScanInit->DataPointsProcess(m_ptUnit->GetRobotCtrl(), vtpoints, vtCoutPoints, vtCoutRobotCoors));
			}
			else {
				vtCoutPoints.insert(vtCoutPoints.end(), vtpoints.begin(), vtpoints.end());
				// ֱ�� �������۳���
				double dLength = TwoPointDis(
					vtCoutPoints[0].x, vtCoutPoints[0].y, vtCoutPoints[0].z,
					vtCoutPoints[vtCoutPoints.size() - 1].x, vtCoutPoints[vtCoutPoints.size() - 1].y, vtCoutPoints[vtCoutPoints.size() - 1].z);

				m_vvtWeldLineInfoGroup[nGroupNo][nWeldNo].tAtrribute.dThoeryLength = dLength;
			}
		}
		//��¼������������
		m_vtTrackTheoryTrack.insert(m_vtTrackTheoryTrack.end(), vtCoutPoints.begin(), vtCoutPoints.end());
		if (false) //���۳��ȡ����ٹ���������β��
		{
			XI_POINT tEndPoint = { m_vvtWeldLineInfoGroup.at(nGroupNo).at(nWeldNo).tWeldLine.EndPoint.x - m_ptUnit->m_dExAxisXPos,
				m_vvtWeldLineInfoGroup.at(nGroupNo).at(nWeldNo).tWeldLine.EndPoint.y,
			m_vvtWeldLineInfoGroup.at(nGroupNo).at(nWeldNo).tWeldLine.EndPoint.z };
			vtCoutPoints.push_back(tEndPoint);
		}

		dExPos = 0.0;
		vtWeldCoorsType.clear();
		vtWeldCoors.clear();
		double dStartHoleSize = m_vvtWeldLineInfoGroup.at(nGroupNo).at(nWeldNo).tAtrribute.dStartHoleSize;
		double dEndHoleSize = m_vvtWeldLineInfoGroup.at(nGroupNo).at(nWeldNo).tAtrribute.dEndHoleSize;
		bool bFixStart = m_vvtWeldLineInfoGroup.at(nGroupNo).at(nWeldNo).tWeldLine.StartPointType > 0;
		//bool bFixScanEnd = m_vvtWeldLineInfoGroup.at(nGroupNo).at(nWeldNo).tAtrribute.bEndFixScan;
		// �������Ϊ2��3ʱ�����յ㶼Ҫ����
		int nStartType = m_vvtWeldLineInfoGroup.at(nGroupNo).at(nWeldNo).tWeldLine.StartPointType;
		LineOrCircularArcWeldingLine tSeam = m_vvtWeldLineInfoGroup[nGroupNo][nWeldNo].tWeldLine;
		m_pScanInit->m_pTraceModel->m_vtRealEndpointCoor.clear();
		m_pScanInit->m_pTraceModel->m_vtEndPointType.clear();
		if (!m_vvtWeldLineInfoGroup[nGroupNo][nWeldNo].tAtrribute.bWeldMode)
		{
			T_LINE_PARA tLineParam;
			if (!CalcLineParamRansac(tLineParam, vtCoutPoints, 0.7))
			{
				return false;
			}
			double dNormal = atan2(tLineParam.dDirY, tLineParam.dDirX) * 180 / 3.1415926 - 90 * m_nRobotInstallDir;;
			double dChangeDisS = 45.0;	// ����̬����
			double dChangeDisE = 45.0;	// ����̬����
			double dChangeAngle = 30.0; // ����̬�Ƕ�
			double dPtnInterval = 2.0;  // �켣����

			for (size_t i = 0; i < m_vvtWeldLineInfoGroup[nGroupNo][nWeldNo].tAtrribute.dThoeryLength / dPtnInterval; i++)
			{
				T_ROBOT_COORS tPoint;
				tPoint.dX = vtCoutPoints[0].x + i * tLineParam.dDirX * dPtnInterval;
				tPoint.dY = vtCoutPoints[0].y + i * tLineParam.dDirY * dPtnInterval;
				tPoint.dZ = vtCoutPoints[0].z + i * tLineParam.dDirZ * dPtnInterval;
				tPoint.dRX = m_dPlatWeldRx;
				tPoint.dRY = m_dPlatWeldRy;
				tPoint.dRZ = DirAngleToRz(dNormal);
				vtWeldCoors.push_back(tPoint);
			}
			// ��ȡ�������� ���ɺ��ӹ켣
			eWeldSeamType = GetWeldSeamType(tSeam);
			//double dTempVar = dChangeDisS;

			int nChangePtnNum = (int)(dChangeDisS / dPtnInterval); // ����̬����

			double dStepChangeAngle = (double)m_nRobotInstallDir * dChangeAngle / (double)nChangePtnNum; // ���ڵ���̬�仯�Ƕ�
			int nChangePtnNumS = (int)(dChangeDisS / dPtnInterval); // ����̬����
			int nChangePtnNumE = (int)(dChangeDisE / dPtnInterval); // ����̬����
			double dStepChangeAngleS = (double)m_nRobotInstallDir * dChangeAngle / (double)nChangePtnNumS; // ���ڵ���̬�仯�Ƕ�
			double dStepChangeAngleE = (double)m_nRobotInstallDir * dChangeAngle / (double)nChangePtnNumE; // ���ڵ���̬�仯�Ƕ�

			int nWeldTrackPtnNum = vtWeldCoors.size();

			// ����˵����̬  ɾ�������׹켣
			if (E_FLAT_SEAM == eWeldSeamType && 0 < tSeam.EndPointType) // ƽ�������յ� 
			{
				int nBaseIndex = nWeldTrackPtnNum - 1 - nChangePtnNumE;
				double dSrcRz = vtWeldCoors[nBaseIndex].dRZ;
				for (int n = nWeldTrackPtnNum - nChangePtnNumE; n < nWeldTrackPtnNum; n++)
				{
					vtWeldCoors[n].dRZ = dSrcRz - ((n - nBaseIndex) * dStepChangeAngleE); // ��̬��С(���������Ұ�װ��ʽ���)(֧������ຸ��)
				}
			}
			if (E_FLAT_SEAM == eWeldSeamType && 0 < tSeam.StartPointType) // ƽ���������
			{
				double dSrcRz = vtWeldCoors[nChangePtnNumS].dRZ;
				for (int n = nChangePtnNumS - 1; n >= 0; n--)
				{
					vtWeldCoors[n].dRZ = dSrcRz + ((nChangePtnNumS - n) * dStepChangeAngleS); // ��̬����(���������Ұ�װ��ʽ���)(֧������ຸ��)
				}
			}
			// ���������ⲿ��
			dExPos = (vtWeldCoors[0].dY + vtWeldCoors[vtWeldCoors.size() - 1].dY) / 2;
			for (size_t nStepNo = 0; nStepNo < vtWeldCoors.size(); nStepNo++)
			{
				vtWeldCoors[nStepNo].dY -= dExPos;
				vtWeldCoorsType.push_back(E_WELD_TRACK);
			}
			m_dTeachExAxlePos = dExPos;
			//continue;
		}
		else
		{
			eWeldSeamType = GetWeldSeamType(tSeam);
			double dStartEndDis = TwoPointDis(
				tSeam.StartPoint.x, tSeam.StartPoint.y, tSeam.StartPoint.z,
				tSeam.EndPoint.x, tSeam.EndPoint.y, tSeam.EndPoint.z);
			bool IsCircle = dStartEndDis < 1.0 ? TRUE : FALSE; // �պ�Բ��	
			// �������������������侫�ȵ���������ʱ����
			double dNormalDirOffset = 0.0, dZValDirOffset = 0.0;
			if (0 == m_pRobotDriver->m_nRobotNo)
			{
				dNormalDirOffset = 0.0; //-1.5;
				dZValDirOffset = 0.0; //3.0;
			}
			else if (1 == m_pRobotDriver->m_nRobotNo)
			{
				dNormalDirOffset = 0.0; //-1.5;
				dZValDirOffset = 0.0; //-0.5;
			}
			else if (2 == m_pRobotDriver->m_nRobotNo)
			{
				dNormalDirOffset = 0.0; //2.0;
				dZValDirOffset = 0.0; //0.5;
			}
			else if (3 == m_pRobotDriver->m_nRobotNo)
			{
				dNormalDirOffset = 0.0; //2.0;
				dZValDirOffset = 0.0; //1.5;
			}
			//---------------------------------------------------
			T_LINE_PARA tLineParam;
			if (!CalcLineParamRansac(tLineParam, vtCoutPoints, 0.7))
			{
				return false;
			}
			double dNormal = atan2(tLineParam.dDirY, tLineParam.dDirX) * 180 / 3.1415926 - 90 * m_nRobotInstallDir;;
			if (DetermineWarpMode(nGroupNo) > 0) // ����
			{	
				// ��������
				m_pScanInit->m_pTraceModel->m_eStartWrapAngleType = E_WRAPANGLE_JUMP;
				// �������			
				
				double dChangeDisS = 35.0;	// ����̬����
				double dChangeDisE = 35.0;	// ����̬����
				double dChangeAngle = 30.0; // ����̬�Ƕ�
				double dPtnInterval = 2.0;  // �켣����
				int nChangePtnNum = (int)(dChangeDisS / dPtnInterval); // ����̬����
				double dStepChangeAngle = (double)m_nRobotInstallDir * dChangeAngle / (double)nChangePtnNum; // ���ڵ���̬�仯�Ƕ�
				int nChangePtnNumS = (int)(dChangeDisS / dPtnInterval); // ����̬����
				int nChangePtnNumE = (int)(dChangeDisE / dPtnInterval); // ����̬����
				double dStepChangeAngleS = (double)m_nRobotInstallDir * dChangeAngle / (double)nChangePtnNumS; // ���ڵ���̬�仯�Ƕ�
				double dStepChangeAngleE = (double)m_nRobotInstallDir * dChangeAngle / (double)nChangePtnNumE; // ���ڵ���̬�仯�Ƕ�
				int nWeldTrackPtnNum = vtWeldCoors.size();
				// ���°�����ʱ���ã���ʱʵ�ְ��ǹ��ܣ��ڴ��ж�ͬ���ڰ��Ǵ������˴���ʾ����ֻ����һͷ������ͷ��
				//double dThink = 12.0;// ���
				//----------------------------------------------------------------------------------------------------------
				//�����Ҫ����:-ͷ������������ת����ͷ������ͷ��������ͷ����ת��һͷ��תһͷ��
				//���ݳ�ʼ�ι켣�����۳���(��ʵ�ʽ�β��)�����������������ݣ��������ٹ켣�����ǹ켣����ǹ�켣
				m_pScanInit->m_pTraceModel->dBoardThink = 10.0;
				vtWeldCoors.clear();
				vtReverseCoors.clear();
				// ��β��ǹ,���һͷ���ǻ����������β��:

				//e----------------------------
				//								|
				//s----------------------------e
				// �����ͷ���ǻ����������β��:
				// e--------------------------s
				// |						  |
				// s-----------------e s------e

				// ��һ��
				double dThoeryLen = m_vvtWeldLineInfoGroup[nGroupNo][nWeldNo].tAtrribute.dThoeryLength;

				dThoeryLen = TwoPointDis(vtCoutPoints[0].x, vtCoutPoints[0].y, vtCoutPoints[0].z,
					vtCoutPoints[vtCoutPoints.size() - 1].x, vtCoutPoints[vtCoutPoints.size() - 1].y, vtCoutPoints[vtCoutPoints.size() - 1].z);
				size_t i = 0;
				if (2 == DetermineWarpMode(nGroupNo))
				{
					i = (24 / dPtnInterval) + 0;// 1:��ʼ������ƫ��һ��//���ǳ���
				}
				else if (0 != m_vvtWeldLineInfoGroup[nGroupNo][nWeldNo].tWeldLine.StartPointType)
				{
					i = 5;
				}
				
				int n1 = dThoeryLen / dPtnInterval;
				int n2 = 24 / dPtnInterval;//���ǳ���
				if (n1 * 2 < dThoeryLen)
				{
					n1 += 4; //��β������Ӽ���
				}
				for ( ; i < (n1 - n2); i++)
				{
					T_ROBOT_COORS tPoint;
					tPoint.dX = vtCoutPoints[0].x + i * tLineParam.dDirX * dPtnInterval;
					tPoint.dY = vtCoutPoints[0].y + i * tLineParam.dDirY * dPtnInterval;
					tPoint.dZ = vtCoutPoints[0].z + i * tLineParam.dDirZ * dPtnInterval;
					tPoint.dRX = m_dPlatWeldRx;
					tPoint.dRY = m_dPlatWeldRy;
					tPoint.dRZ = DirAngleToRz(dNormal);
					RobotCoordPosOffset(tPoint, dNormal, dNormalDirOffset, dZValDirOffset);
					vtWeldCoors.push_back(tPoint);
					// �����������
					RobotCoordPosOffset(tPoint, dNormal + 180.0, m_pScanInit->m_pTraceModel->dBoardThink /*+ dNormalDirOffset*/);
					tPoint.dRZ = DirAngleToRz(dNormal + 180.0);
					//tPoint.dZ += dZValDirOffset;
					vtReverseCoors.push_back(tPoint);
				}
				// ���������ݷ����Ų�
				reverse(vtReverseCoors.begin(), vtReverseCoors.end());
				CString strFileAuto = OUTPUT_PATH + m_ptUnit->m_tContralUnit.strUnitName + "\\" + RECOGNITION_FOLDER;
				if (1 == DetermineWarpMode(nGroupNo))
				{
					// ��β�����ͣ�����������ν�β���ɰ��ǹ������ͣ������β�����̬
					m_pScanInit->m_pTraceModel->m_vtEndPointType.push_back(m_vvtWeldLineInfoGroup[nGroupNo][nWeldNo].tWeldLine.EndPointType);
					m_pScanInit->m_pTraceModel->m_vtEndPointType.push_back(m_vvtWeldLineInfoGroup[nGroupNo][nWeldNo].tWeldLine.StartPointType);
					m_pScanInit->m_pTraceModel->m_vtRealEndpointCoor.push_back(vtWeldCoors[vtWeldCoors.size() - 1]);
					m_pScanInit->m_pTraceModel->m_vtRealEndpointCoor.push_back(vtReverseCoors[vtReverseCoors.size() - 1]);				

					if (E_FLAT_SEAM == eWeldSeamType && 0 < tSeam.StartPointType)// ƽ���������
					{
						double dSrcRz = vtWeldCoors[nChangePtnNumS].dRZ;
						for (int n = nChangePtnNumS - 1; n >= 0; n--)
						{
							vtWeldCoors[n].dRZ = dSrcRz + ((nChangePtnNumS - n) * dStepChangeAngleS); // ��̬����(���������Ұ�װ��ʽ���)(֧������ຸ��)
						}
					}
					else
					{
						nChangePtnNum = 0;
					}

					// ��������켣
					// ��ӹ��ȵ�
					T_ROBOT_COORS tRobotTemp = vtWeldCoors[vtWeldCoors.size() - 1];
					tRobotTemp.dRZ += 90 * m_nRobotInstallDir;
					tRobotTemp.dX = (vtWeldCoors[vtWeldCoors.size() - 1].dX + vtReverseCoors[0].dX) / 2;
					tRobotTemp.dY = (vtWeldCoors[vtWeldCoors.size() - 1].dY + vtReverseCoors[0].dY) / 2;
					tRobotTemp.dZ = (vtWeldCoors[vtWeldCoors.size() - 1].dZ + vtReverseCoors[0].dZ) / 2;
					vtWeldCoors[vtWeldCoors.size() - 1].dRZ += 45 * m_nRobotInstallDir;
					vtWeldCoors.push_back(tRobotTemp);
					vtReverseCoors[0].dRZ -= 45 * m_nRobotInstallDir;
					vtWeldCoors.insert(vtWeldCoors.end(), vtReverseCoors.begin(), vtReverseCoors.end());

				}
				SaveCoordToFile(vtWeldCoors, strFileAuto + "AAAAA-vtWeldCoors.txt");
				SaveCoordToFile(vtReverseCoors, strFileAuto + "AAAAA-vtReverseCoors.txt");
				// ������
				if (2 == DetermineWarpMode(nGroupNo))
				{
					// ������ǲ�����̬
					nChangePtnNum = 0;
					m_pScanInit->m_pTraceModel->m_vtEndPointType.push_back(0);
					m_pScanInit->m_pTraceModel->m_vtEndPointType.push_back(0);
					m_pScanInit->m_pTraceModel->m_vtEndPointType.push_back(0);

					m_pScanInit->m_pTraceModel->m_vtRealEndpointCoor.clear();
					m_pScanInit->m_pTraceModel->m_vtRealEndpointCoor.push_back(vtReverseCoors[vtReverseCoors.size() - 1]);
					m_pScanInit->m_pTraceModel->m_vtRealEndpointCoor.push_back(vtWeldCoors[vtWeldCoors.size() - 1]);


					vector<T_ROBOT_COORS> vtWeldCoorsTemp(vtWeldCoors); // ����켣
					vtWeldCoors.clear();
					vtWeldCoors.insert(vtWeldCoors.end(), vtReverseCoors.end() - 100, vtReverseCoors.end());
					// ��������켣
					// ��ӹ��ȵ�
					T_ROBOT_COORS tRobotTemp = vtReverseCoors[vtReverseCoors.size() - 1];
					tRobotTemp.dRZ += 90 * m_nRobotInstallDir;
					tRobotTemp.dX = (vtReverseCoors[vtReverseCoors.size() - 1].dX + vtWeldCoorsTemp[0].dX) / 2;
					tRobotTemp.dY = (vtReverseCoors[vtReverseCoors.size() - 1].dY + vtWeldCoorsTemp[0].dY) / 2;
					tRobotTemp.dZ = (vtReverseCoors[vtReverseCoors.size() - 1].dZ + vtWeldCoorsTemp[0].dZ) / 2;
					vtWeldCoors[vtWeldCoors.size()-1].dRZ+= 45 * m_nRobotInstallDir;
					vtWeldCoors.push_back(tRobotTemp);
					vtWeldCoorsTemp[0].dRZ -= 45 * m_nRobotInstallDir;
					vtWeldCoors.insert(vtWeldCoors.end(), vtWeldCoorsTemp.begin(), vtWeldCoorsTemp.end());

					tRobotTemp = vtWeldCoorsTemp[vtWeldCoorsTemp.size() - 1];
					tRobotTemp.dRZ += 90 * m_nRobotInstallDir;
					tRobotTemp.dX = (vtReverseCoors[0].dX + vtWeldCoorsTemp[vtWeldCoorsTemp.size() - 1].dX) / 2;
					tRobotTemp.dY = (vtReverseCoors[0].dY + vtWeldCoorsTemp[vtWeldCoorsTemp.size() - 1].dY) / 2;
					tRobotTemp.dZ = (vtReverseCoors[0].dZ + vtWeldCoorsTemp[vtWeldCoorsTemp.size() - 1].dZ) / 2;
					vtWeldCoors[vtWeldCoors.size() - 1].dRZ += 45 * m_nRobotInstallDir;
					vtWeldCoors.push_back(tRobotTemp);
					vtReverseCoors[0].dRZ -= 45 * m_nRobotInstallDir;
					vtWeldCoors.insert(vtWeldCoors.end(), vtReverseCoors.begin(), vtReverseCoors.end()-100);
					// ------------
					m_pScanInit->m_pTraceModel->m_vtRealEndpointCoor.push_back(vtWeldCoors[0]);
					SaveCoordToFile(vtWeldCoors, strFileAuto + "AAAAA-vtWeldCoors2.txt");
				}
				// ���������켣
				for (int i = 0; i < vtWeldCoors.size(); i++)
				{
					vtWeldCoorsType.push_back(E_WELD_TRACK);
				}
				for (int i = 0; i < nChangePtnNum; i++)
				{
					vtWeldCoorsType[i] = E_WELD_TRACK | E_WELD_TRACK_CHANGE_POSTURE;
				}
				//----------------------------------------------------------------------------------
			}
			else
			{
				if (!IsCircle)
				{
					// ���������켣
					for (int i = 0; i < vtCoutPoints.size(); i++)
					{
						vtWeldCoorsType.push_back(E_WELD_TRACK);
					}

					int nStartChangeStepNo = 0, nEndChangeStepNo = 0;
					CalcRobotPostureCoors(vtCoutPoints, m_dPlatWeldRx, m_dPlatWeldRy, dStepDis, 35.0, 35.0, 35.0, 35.0, /*1*/ 0 != m_vvtWeldLineInfoGroup[nGroupNo][nWeldNo].tWeldLine.StartPointType, 
						0, dStartHoleSize, dEndHoleSize, nStartChangeStepNo, nEndChangeStepNo, vtWeldCoors);
					// ������ʼ���ݺ͸������һ��
					for (size_t i = 0; i < vtWeldCoors.size(); i++)
					{
						RobotCoordPosOffset(vtWeldCoors[i], dNormal, dNormalDirOffset, dZValDirOffset);
					}
					//---------------------------------------------------------------------------------

					// ����ƽ���β����̬
					//m_vvtWeldLineInfoGroup[nGroupNo][nWeldNo].tAtrribute.bEndFixScan = true;

					for (int i = 0; i < nStartChangeStepNo; i++)
					{
						vtWeldCoorsType[i] = E_WELD_TRACK | E_WELD_TRACK_CHANGE_POSTURE;
					}

					vector<XI_POINT> vtTrack;
					XI_POINT tSartPoint = vtCoutPoints.at(vtCoutPoints.size() - 2), tEndpoint = vtCoutPoints.at(vtCoutPoints.size() - 1);
					// ȥ����β����
					vtWeldCoors.pop_back();
					vtWeldCoors.pop_back();
					// �����������۹켣
					GenerateMultipleCoord(tSartPoint, tEndpoint, vtTrack, dStepDis);
					double dRZ = vtWeldCoors.at(vtWeldCoors.size() - 1).dRZ;
					for (size_t nTemp = 0; nTemp < vtTrack.size(); nTemp++)
					{
						T_ROBOT_COORS tRoobt(vtTrack.at(nTemp).x, vtTrack.at(nTemp).y, vtTrack.at(nTemp).z, m_dPlatWeldRx, m_dPlatWeldRy, dRZ, 0, 0, 0);
						vtWeldCoorsType.push_back(E_TRANSITION_POINT);
						vtWeldCoors.push_back(tRoobt);
					}


				}
				else
				{
					vtpoints.clear();
					CString strFileName;
					strFileName.Format("%s%s%sEndpointCoors-%d-0.txt", OUTPUT_PATH, m_pRobotDriver->m_strRobotName, RECOGNITION_FOLDER, nGroupNo);
					if (!LoadPointsData(vtpoints, strFileName)) {
						//���޸�
						XUI::MesBox::PopInfo("{0}�ļ���������ʧ��", strFileName.GetBuffer());
						//XiMessageBox("%s �ļ���������ʧ��", strFileName);
						return false;
					}
					CHECK_BOOL_RETURN(m_pScanInit->DataPointsProcess(m_ptUnit->GetRobotCtrl(), vtpoints, vtCoutPoints, vtCoutRobotCoors));
					int nStartChangeStepNo, nEndChangeStepNo;
					CalcRobotPostureCoors(vtCoutPoints, m_dPlatWeldRx, m_dPlatWeldRy, dStepDis, 45.0, 45.0, 45.0, 45.0, 0, 0, 0, 0,
						nStartChangeStepNo, nEndChangeStepNo, vtWeldCoors);
					// ���������켣
					for (int i = 0; i < vtWeldCoors.size(); i++)
					{
						vtWeldCoorsType.push_back(E_WELD_TRACK);
					}
				}
			}
		}

		// ���� ��� ֱ������ �ⲿ������ ��������
		CString sFileName;
		sFileName.Format("%s%d_%d_RealWeldCoord.txt", m_sDataSavePath, nGroupNo, nWeldNo);
		FILE* pf = fopen(sFileName, "w");
		for (int nPtnIdx = 0; nPtnIdx < vtWeldCoors.size(); nPtnIdx++)
		{
			T_ROBOT_COORS tCoord = vtWeldCoors[nPtnIdx];
			fprintf(pf, "%d %11.3lf %11.3lf %11.3lf %11.3lf %11.3lf %11.3lf %11.3lf %4d %4d\n", nPtnIdx,
				tCoord.dX, tCoord.dY, tCoord.dZ, tCoord.dRX, tCoord.dRY, tCoord.dRZ, dExPos, eWeldSeamType, vtWeldCoorsType[nPtnIdx]);
		}
		fclose(pf);
	}
	return true;
}

//bool GenericWeld::CalcAndTrackWeldingTry(int nGroupNo)
//{
//	// ���ٺ����������ݣ�����/�����õ���ʼ�����ݣ���С�ڸ������۳��ȣ�����β��
//	double dStepDis = 3.0;
//	vector<T_ROBOT_COORS> vtWeldCoors; // ����켣
//	vector<T_ROBOT_COORS> vtReverseCoors; //����켣
//	vector<int> vtWeldCoorsType;
//	E_WELD_SEAM_TYPE eWeldSeamType;
//	double dExPos = 0.0;
//	// ����������nGroup��ÿ������
//	for (int nWeldNo = 0; nWeldNo < m_vvtWeldLineInfoGroup[nGroupNo].size(); nWeldNo++)
//	{
//		// ��ʾֻȡһ���ڵ�һ�����죬�˴η�����Ǻ����Ϊһ�飬����������죬���ຸ�쵥����Ϊһ��
//		if (nWeldNo > 0)
//		{
//			break;
//		}
//		// ----------------------------
//		dExPos = 0.0;
//		vtWeldCoorsType.clear();
//		vtWeldCoors.clear();
//		double dStartHoleSize = m_vvtWeldLineInfoGroup.at(nGroupNo).at(nWeldNo).tAtrribute.dStartHoleSize;
//		double dEndHoleSize = m_vvtWeldLineInfoGroup.at(nGroupNo).at(nWeldNo).tAtrribute.dEndHoleSize;
//		bool bFixStart = m_vvtWeldLineInfoGroup.at(nGroupNo).at(nWeldNo).tWeldLine.StartPointType;
//		//bool bFixScanEnd = m_vvtWeldLineInfoGroup.at(nGroupNo).at(nWeldNo).tAtrribute.bEndFixScan;
//
//		// �������Ϊ2��3ʱ�����յ㶼Ҫ����
//		int nStartType = m_vvtWeldLineInfoGroup.at(nGroupNo).at(nWeldNo).tWeldLine.StartPointType;
//
//		LineOrCircularArcWeldingLine tSeam = m_vvtWeldLineInfoGroup[nGroupNo][nWeldNo].tWeldLine;
//		if (!m_vvtWeldLineInfoGroup[nGroupNo][nWeldNo].tAtrribute.bWeldMode)
//		{
//			vector<XI_POINT> vtpoints;
//			CString strFileName;
//			strFileName.Format("%s%s%sEndpointCoors-%d-0.txt", OUTPUT_PATH, m_pRobotDriver->m_strRobotName, RECOGNITION_FOLDER, nGroupNo);
//			if (!LoadPointsData(vtpoints, strFileName)) {
//				XiMessageBox("%s �ļ���������ʧ��", strFileName);
//				return false;
//			}
//			T_LINE_PARA tLineParam = CalcLineParamRansac(vtpoints, 0.7);
//			double dNormal = atan2(tLineParam.dDirY, tLineParam.dDirX) * 180 / 3.1415926 - 90 * m_nRobotInstallDir;;
//			double dChangeDisS = 45.0;	// ����̬����
//			double dChangeDisE = 45.0;	// ����̬����
//			double dChangeAngle = 45.0; // ����̬�Ƕ�
//			double dPtnInterval = 2.0;  // �켣����
//			
//			for (size_t i = 0; i < m_vvtWeldLineInfoGroup[nGroupNo][nWeldNo].tAtrribute.dThoeryLength / dPtnInterval; i++)
//			{
//				T_ROBOT_COORS tPoint;
//				tPoint.dX = vtpoints[0].x + i * tLineParam.dDirX * dPtnInterval;
//				tPoint.dY = vtpoints[0].y + i * tLineParam.dDirY * dPtnInterval;
//				tPoint.dZ = vtpoints[0].z + i * tLineParam.dDirZ * dPtnInterval;
//				tPoint.dRX = m_dPlatWeldRx;
//				tPoint.dRY = m_dPlatWeldRy;
//				tPoint.dRZ = DirAngleToRz(dNormal);
//				vtWeldCoors.push_back(tPoint);
//			}
//			// ��ȡ�������� ���ɺ��ӹ켣
//			eWeldSeamType = GetWeldSeamType(tSeam);
//			double dTempVar = dChangeDisS;
//
//			int nChangePtnNum = (int)(dChangeDisS / dPtnInterval); // ����̬����
//			double dStepChangeAngle = (double)m_nRobotInstallDir * dChangeAngle / (double)nChangePtnNum; // ���ڵ���̬�仯�Ƕ�
//			int nChangePtnNumS = (int)(dChangeDisS / dPtnInterval); // ����̬����
//			int nChangePtnNumE = (int)(dChangeDisE / dPtnInterval); // ����̬����
//			double dStepChangeAngleS = (double)m_nRobotInstallDir * dChangeAngle / (double)nChangePtnNumS; // ���ڵ���̬�仯�Ƕ�
//			double dStepChangeAngleE = (double)m_nRobotInstallDir * dChangeAngle / (double)nChangePtnNumE; // ���ڵ���̬�仯�Ƕ�
//
//			int nWeldTrackPtnNum = vtWeldCoors.size();
//
//			// ����˵����̬  ɾ�������׹켣
//			if (E_FLAT_SEAM == eWeldSeamType && true == tSeam.EndPointType) // ƽ�������յ� 
//			{
//				int nBaseIndex = nWeldTrackPtnNum - 1 - nChangePtnNumE;
//				double dSrcRz = vtWeldCoors[nBaseIndex].dRZ;
//				for (int n = nWeldTrackPtnNum - nChangePtnNumE; n < nWeldTrackPtnNum; n++)
//				{
//					vtWeldCoors[n].dRZ = dSrcRz - ((n - nBaseIndex) * dStepChangeAngleE ); // ��̬��С(���������Ұ�װ��ʽ���)(֧������ຸ��)
//				}
//			}
//			if (E_FLAT_SEAM == eWeldSeamType && true == tSeam.StartPointType) // ƽ���������
//			{
//				double dSrcRz = vtWeldCoors[nChangePtnNumS].dRZ;
//				for (int n = nChangePtnNumS - 1; n >= 0; n--)
//				{
//					vtWeldCoors[n].dRZ = dSrcRz + ((nChangePtnNumS - n) * dStepChangeAngleS); // ��̬����(���������Ұ�װ��ʽ���)(֧������ຸ��)
//				}
//			}
//			// ���������ⲿ��
//			dExPos = (vtWeldCoors[0].dY + vtWeldCoors[vtWeldCoors.size() - 1].dY) / 2;
//			for (size_t nStepNo = 0; nStepNo < vtWeldCoors.size(); nStepNo++)
//			{
//				vtWeldCoors[nStepNo].dY -= dExPos;
//				vtWeldCoorsType.push_back(E_WELD_TRACK);
//			}
//			m_dTeachExAxlePos = dExPos;
//			//continue;
//		}
//		else
//		{
//			eWeldSeamType = GetWeldSeamType(tSeam);
//			double dStartEndDis = TwoPointDis(
//				tSeam.StartPoint.x, tSeam.StartPoint.y, tSeam.StartPoint.z,
//				tSeam.EndPoint.x, tSeam.EndPoint.y, tSeam.EndPoint.z);
//			bool IsCircle = dStartEndDis < 1.0 ? TRUE : FALSE; // �պ�Բ��
//			// �պ�Բ��
//			vector<XI_POINT> vtpoints, vtCoutPoints;
//			vector<T_ROBOT_COORS> vtCoutRobotCoors;
//			vector<T_ANGLE_PULSE> vtWeldPulse;
//			if (DetermineWarpMode(nGroupNo) > 0) // ����
//			{
//				// ���س�ʼ������
//				CString strFileName;
//				strFileName.Format("%s%s%sEndpointCoors-%d-0.txt", OUTPUT_PATH, m_pRobotDriver->m_strRobotName, RECOGNITION_FOLDER, nGroupNo);
//				if (!LoadPointsData(vtpoints, strFileName)) {
//					XiMessageBox("%s �ļ���������ʧ��", strFileName);
//					return false;
//				}
//				// �������
//				T_LINE_PARA tLineParam = CalcLineParamRansac(vtpoints, 0.7);
//				double dNormal = atan2(tLineParam.dDirY, tLineParam.dDirX) * 180 / 3.1415926 - 90 * m_nRobotInstallDir;;
//				double dChangeDisS = 45.0;	// ����̬����
//				double dChangeDisE = 45.0;	// ����̬����
//				double dChangeAngle = 45.0; // ����̬�Ƕ�
//				double dPtnInterval = 2.0;  // �켣����
//				m_pScanInit->m_pTraceModel->m_vtRealEndpointCoor.clear();
//				// ���°�����ʱ���ã���ʱʵ�ְ��ǹ��ܣ��ڴ��ж�ͬ���ڰ��Ǵ������˴���ʾ����ֻ����һͷ������ͷ��
//				double dThink = 10.0;// ���
//				//----------------------------------------------------------------------------------------------------------
//				//�����Ҫ����:-ͷ������������ת����ͷ������ͷ��������ͷ����ת��һͷ��תһͷ��
//				//���ݳ�ʼ�ι켣�����۳���(��ʵ�ʽ�β��)�����������������ݣ��������ٹ켣�����ǹ켣����ǹ�켣
//				vtWeldCoors.clear();
//				vtReverseCoors.clear();
//				// ��β��ǹ,���һͷ���ǻ����������β��:
//
//				//e----------------------------
//				//								|
//				//s----------------------------e
//				// �����ͷ���ǻ����������β��:
//				// e--------------------------s
//				// |						  |
//				// s-----------------e s------e
//
//				// ��һ��
//				for (size_t i = (30 / dPtnInterval-1); i < (m_vvtWeldLineInfoGroup[nGroupNo][nWeldNo].tAtrribute.dThoeryLength / dPtnInterval - 30 / dPtnInterval); i++)
//				{
//					T_ROBOT_COORS tPoint;
//					tPoint.dX = vtpoints[0].x + i * tLineParam.dDirX * dPtnInterval;
//					tPoint.dY = vtpoints[0].y + i * tLineParam.dDirY * dPtnInterval;
//					tPoint.dZ = vtpoints[0].z + i * tLineParam.dDirZ * dPtnInterval;
//					tPoint.dRX = m_dPlatWeldRx;
//					tPoint.dRY = m_dPlatWeldRy;
//					tPoint.dRZ = DirAngleToRz(dNormal);
//					vtWeldCoors.push_back(tPoint);
//					// �����������
//					RobotCoordPosOffset(tPoint, dNormal + 180.0, dThink + 2.0);
//					tPoint.dRZ = DirAngleToRz(dNormal + 180.0);
//					tPoint.dZ += 1.5;
//					vtReverseCoors.push_back(tPoint);
//				}
//				// ���������ݷ����Ų�
//				reverse(vtReverseCoors.begin(), vtReverseCoors.end());
//				if (1 == DetermineWarpMode(nGroupNo))
//				{
//					m_pScanInit->m_pTraceModel->m_vtRealEndpointCoor.push_back(vtWeldCoors[vtWeldCoors.size() - 1]);
//					m_pScanInit->m_pTraceModel->m_vtRealEndpointCoor.push_back(vtReverseCoors[vtReverseCoors.size() - 1]);
//				}
//				SaveCoordToFile(vtWeldCoors, "AAAAA-vtWeldCoors.txt");
//				SaveCoordToFile(vtReverseCoors, "AAAAA-vtReverseCoors.txt");
//				// ������
//				if (2 == DetermineWarpMode(nGroupNo))
//				{
//					m_pScanInit->m_pTraceModel->m_vtRealEndpointCoor.clear();
//					m_pScanInit->m_pTraceModel->m_vtRealEndpointCoor.push_back(vtReverseCoors[vtReverseCoors.size() - 1]);
//					m_pScanInit->m_pTraceModel->m_vtRealEndpointCoor.push_back(vtWeldCoors[vtWeldCoors.size() - 1]);
//					vtWeldCoors.clear();
//					vtWeldCoors.insert(vtWeldCoors.end(), vtReverseCoors.end() - 100, vtReverseCoors.end());
//					m_pScanInit->m_pTraceModel->m_vtRealEndpointCoor.push_back(vtWeldCoors[0]);
//					SaveCoordToFile(vtWeldCoors, "AAAAA-vtWeldCoors2.txt");
//				}
//				// ���������켣
//				for (int i = 0; i < vtWeldCoors.size(); i++)
//				{
//					vtWeldCoorsType.push_back(E_WELD_TRACK);
//				}
//				//----------------------------------------------------------------------------------
//			}
//			else
//			{
//				if (!IsCircle)
//				{
//					// �Ǳպ�Բ�������ι켣����ʼ���ݺͽ�β��		
//					for (int i = 0; i < 2; i++)
//					{
//						vtpoints.clear();
//						if (i == 0 || nStartType > 1)
//						{
//							// ��ʾ��������ǰ�����յ㣬���ɶ��յ�����ʹ����������
//							CString strFileName;
//							strFileName.Format("%s%s%sEndpointCoors-%d-%d.txt", OUTPUT_PATH, m_pRobotDriver->m_strRobotName, RECOGNITION_FOLDER, nGroupNo, i);
//							if (!LoadPointsData(vtpoints, strFileName)) {
//								XiMessageBox("%s �ļ���������ʧ��", strFileName);
//								return false;
//							}
//						}
//						if (vtpoints.size() > 10) {
//							// �Գ�ʼ�켣���������˲�
//							CHECK_BOOL_RETURN(m_pScanInit->DataPointsProcess(m_ptUnit->GetRobotCtrl(), vtpoints, vtCoutPoints, vtCoutRobotCoors));
//						}
//						else {
//							if (/*2 == */nStartType > 1)
//							{
//								vtCoutPoints.insert(vtCoutPoints.end(), vtpoints.begin(), vtpoints.end());
//
//								// �������۳���
//								double dLength = TwoPointDis(
//									vtCoutPoints[0].x, vtCoutPoints[0].y, vtCoutPoints[0].z,
//									vtCoutPoints[vtCoutPoints.size() - 1].x, vtCoutPoints[vtCoutPoints.size() - 1].y, vtCoutPoints[vtCoutPoints.size() - 1].z);
//
//								m_vvtWeldLineInfoGroup[nGroupNo][nWeldNo].tAtrribute.dThoeryLength = dLength;
//							}
//							else
//							{
//								XI_POINT tEndPoint = { m_vvtWeldLineInfoGroup.at(nGroupNo).at(nWeldNo).tWeldLine.EndPoint.x - m_ptUnit->m_dExAxisXPos,
//								m_vvtWeldLineInfoGroup.at(nGroupNo).at(nWeldNo).tWeldLine.EndPoint.y,
//							m_vvtWeldLineInfoGroup.at(nGroupNo).at(nWeldNo).tWeldLine.EndPoint.z };
//								vtCoutPoints.push_back(tEndPoint);
//							}
//						}
//					}
//
//					// ���������켣
//					for (int i = 0; i < vtCoutPoints.size(); i++)
//					{
//						vtWeldCoorsType.push_back(E_WELD_TRACK);
//					}
//
//					int nStartChangeStepNo = 0, nEndChangeStepNo = 0;
//					CalcRobotPostureCoors(vtCoutPoints, m_dPlatWeldRx, m_dPlatWeldRy, dStepDis, 45.0, 45.0, 45.0, 45.0, m_vvtWeldLineInfoGroup[nGroupNo][nWeldNo].tWeldLine.StartPointType,
//						0, dStartHoleSize, dEndHoleSize, nStartChangeStepNo, nEndChangeStepNo, vtWeldCoors);
//
//					for (int i = 0; i < nStartChangeStepNo; i++)
//					{
//						vtWeldCoorsType[i] = E_WELD_TRACK | E_WELD_TRACK_CHANGE_POSTURE;
//					}
//					//if (!bFixScanEnd)
//					{
//						vector<XI_POINT> vtTrack;
//						XI_POINT tSartPoint = vtCoutPoints.at(vtCoutPoints.size() - 2), tEndpoint = vtCoutPoints.at(vtCoutPoints.size() - 1);
//						// �����������۹켣
//						GenerateMultipleCoord(tSartPoint, tEndpoint, vtTrack, dStepDis);
//						double dRZ = vtWeldCoors.at(vtWeldCoors.size() - 1).dRZ;
//						for (size_t nTemp = 0; nTemp < vtTrack.size(); nTemp++)
//						{
//							T_ROBOT_COORS tRoobt(vtTrack.at(nTemp).x, vtTrack.at(nTemp).y, vtTrack.at(nTemp).z, m_dPlatWeldRx, m_dPlatWeldRy, dRZ, 0, 0, 0);
//							vtWeldCoorsType.push_back(E_TRANSITION_POINT);
//							vtWeldCoors.push_back(tRoobt);
//						}
//					}
//
//
//				}
//				else
//				{
//					vtpoints.clear();
//					CString strFileName;
//					strFileName.Format("%s%s%sEndpointCoors-%d-0.txt", OUTPUT_PATH, m_pRobotDriver->m_strRobotName, RECOGNITION_FOLDER, nGroupNo);
//					if (!LoadPointsData(vtpoints, strFileName)) {
//						XiMessageBox("%s �ļ���������ʧ��", strFileName);
//						return false;
//					}
//					CHECK_BOOL_RETURN(m_pScanInit->DataPointsProcess(m_ptUnit->GetRobotCtrl(), vtpoints, vtCoutPoints, vtCoutRobotCoors));
//					int nStartChangeStepNo, nEndChangeStepNo;
//					CalcRobotPostureCoors(vtCoutPoints, m_dPlatWeldRx, m_dPlatWeldRy, dStepDis, 45.0, 45.0, 45.0, 45.0, 0, 0, 0, 0,
//						nStartChangeStepNo, nEndChangeStepNo, vtWeldCoors);
//					// ���������켣
//					for (int i = 0; i < vtWeldCoors.size(); i++)
//					{
//						vtWeldCoorsType.push_back(E_WELD_TRACK);
//					}
//				}
//			}					
//		}
//
//		// ���� ��� ֱ������ �ⲿ������ ��������
//		CString sFileName;
//		sFileName.Format("%s%d_%d_RealWeldCoord.txt", m_sDataSavePath, nGroupNo, nWeldNo);
//		FILE* pf = fopen(sFileName, "w");
//		for (int nPtnIdx = 0; nPtnIdx < vtWeldCoors.size(); nPtnIdx++)
//		{
//			T_ROBOT_COORS tCoord = vtWeldCoors[nPtnIdx];
//			fprintf(pf, "%d %11.3lf %11.3lf %11.3lf %11.3lf %11.3lf %11.3lf %11.3lf %4d %4d\n", nPtnIdx,
//				tCoord.dX, tCoord.dY, tCoord.dZ, tCoord.dRX, tCoord.dRY, tCoord.dRZ, dExPos, eWeldSeamType, vtWeldCoorsType[nPtnIdx]);
//		}
//		fclose(pf);
//	}
//	return true;
//}








