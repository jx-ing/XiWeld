#include "StdAfx.h"
#include "SingleBoard.h"

SingleBoard::SingleBoard(CUnit* ptUnit, E_WORKPIECE_TYPE eWorkPieceType) :
	WeldAfterMeasure(ptUnit, eWorkPieceType)
{
}

SingleBoard::~SingleBoard()
{

}
void SingleBoard::SetRecoParam()
{
	COPini opini;
	opini.SetFileName(DATA_PATH + m_ptUnit->m_tContralUnit.strUnitName + SYETEM_PARAM_FILE);

	opini.SetSectionName("E_SINGLE_BOARD");
	opini.ReadString("ParamValue0", &m_dZPallet);
	opini.ReadString("ParamValue1", &m_dSideBoardThick);
	opini.ReadString("ParamValue2", &m_bAutoMeasureThick);
}

bool SingleBoard::PointCloudProcess(bool bUseModel, CvPoint3D64f* pPointCloud/* = NULL*/, int PointCloudSize/* = 0*/)
{
	if (bUseModel)
		return PointCloudProcessWithModel();

	m_vtWeldSeamData.clear();
	int nWeldingSeamNumber = 0;
	bool bInstallDir = 1 == m_nRobotInstallDir ? true : false;
	vector<CvPoint3D64f> vtPointCloud;
	LineOrCircularArcWeldingLine* pLineOrCircularArcWeldingLine;
	try
	{
		if (NULL == pPointCloud)
		{
			LoadContourData(m_pRobotDriver, m_sPointCloudFileName, vtPointCloud);
			pPointCloud = (CvPoint3D64f*)vtPointCloud.data();
			PointCloudSize = vtPointCloud.size();
		}
		// ���ָ� С���ӿ� 
		pLineOrCircularArcWeldingLine = SmallSamplePointCloudToWeldingLines(
			(Three_DPoint*)pPointCloud, PointCloudSize, &nWeldingSeamNumber, 
			bInstallDir, m_dZPallet, 5.0, false, false, 0.0, false, false, 2);
		//// ���ָ� ţ�Ƚӿ�
		//pLineOrCircularArcWeldingLine = BracketPointCloudToWeldingLines(
		//	(Three_DPoint*)pPointCloud, PointCloudSize, &nWeldingSeamNumber, bInstallDir, m_dZPallet, 5.0, false);
		//// Բ��
		//pLineOrCircularArcWeldingLine = ArcPlatePointCloudToWeldingLinesExtraction(
		//	(Three_DPoint*)pPointCloud, PointCloudSize, &nWeldingSeamNumber,
		//	bInstallDir, true, m_dZPallet, 5.0, false, true, false, 15, 4);
		SavePointCloudProcessResult((LineOrCircularArcWeldingLine*)pLineOrCircularArcWeldingLine, nWeldingSeamNumber);
		ReleaseWeldingLines(&pLineOrCircularArcWeldingLine);
		LoadCloudProcessResultNew(OUTPUT_PATH + m_ptUnit->m_tContralUnit.strUnitName + "\\" + POINT_CLOUD_IDENTIFY_RESULT);
	}
	catch (...)
	{
		XiMessageBox("����崦���쳣");
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

bool SingleBoard::WeldSeamGrouping(int& nWeldGroupNum)
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
				vtWeldLinePlaneOverLength.push_back(tLineSeamInfo);
				continue;
			}
			if (dWeldSeamLen < dMinFlatSeamLen)
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

bool SingleBoard::CalcMeasureTrack(int nGroupNo, std::vector<T_ROBOT_COORS>& vtMeasureCoord, std::vector<T_ANGLE_PULSE>& vtMeasurePulse, vector<int>& vnMeasureType, double& dExAxlePos, double& dSafeHeight)
{
	InitParam();

	// ��������� ��������� �����һ����
	T_ROBOT_COORS tCoord;
	T_ANGLE_PULSE tPulse;
	T_ROBOT_COORS tTempCoord;
	T_ANGLE_PULSE tTempPulse;
	T_TEACH_DATA tTeachData;
	tTeachData.nWeldNo = 0;
	tTeachData.nMeasurePtnNo = 0;
	tTeachData.nMeasureType = E_DOUBLE_LONG_LINE | E_TEACH_POINT;
	tTeachData.eWeldSeamType = E_FLAT_SEAM;
	tTeachData.eAttribute = E_BELONG_START;
	tTeachData.eEndpointType = E_FREE_POINT;
	LineOrCircularArcWeldingLine tSeam = m_vvtWeldLineInfoGroup[nGroupNo][0].tWeldLine;
	WeldSeamAtrribute tAtrribute = m_vvtWeldLineInfoGroup[nGroupNo][0].tAtrribute;
	T_CAMREA_PARAM tCameraParam = m_ptUnit->GetCameraParam(m_ptUnit->m_nTrackCameraNo);

	CvPoint3D64f tPtn;
	tPtn.x = (tSeam.StartPoint.x + tSeam.EndPoint.x) / 2.0;
	tPtn.y = (tSeam.StartPoint.y + tSeam.EndPoint.y) / 2.0;
	tPtn.z = (tSeam.StartPoint.z + tSeam.EndPoint.z) / 2.0;

	double dPartMaxHeight = tSeam.StartPoint.z * (double)m_nRobotInstallDir > tSeam.EndPoint.z * (double)m_nRobotInstallDir ? tSeam.StartPoint.z : tSeam.EndPoint.z;
	dPartMaxHeight = dPartMaxHeight * (double)m_nRobotInstallDir > tSeam.ZSide * (double)m_nRobotInstallDir ? dPartMaxHeight : tSeam.ZSide;
	//dPartMaxHeight *= (double)m_nRobotInstallDir;
	dSafeHeight = dPartMaxHeight + (m_dGunDownBackSafeDis * (double)m_nRobotInstallDir);

#ifdef SINGLE_ROBOT
	dExAxlePos = tPtn.y + m_dExAxleOffsetDis; 
#else
	dExAxlePos = tPtn.x + m_dExAxleOffsetDis;
#endif // SINGLE_ROBOT

	XiAlgorithm alg;
	double dNorAngle = alg.CalcArcAngle(tSeam.StartNormalVector.x, tSeam.StartNormalVector.y);
	double dWeldNorAngle = alg.CalcArcAngle(tSeam.EndPoint.x - tSeam.StartPoint.x, tSeam.EndPoint.y - tSeam.StartPoint.y);

	if (m_bAutoMeasureThick)
	{
		tCoord = GenerateRobotCoord(tPtn, m_dPlatWeldRx, m_dPlatWeldRy, DirAngleToRz(dNorAngle + 180.0));
		RobotCoordPosOffset(tCoord, RzToDirAngle(tCoord.dRZ), 10.0);	
		DecomposeExAxle(tCoord, dExAxlePos, m_ptUnit->m_nMeasureAxisNo);
		tTeachData.tMeasureCoordGunTool = tCoord;
		m_vtTeachData.push_back(tTeachData);

		// ���ɵ�
		tTempCoord = tCoord; 
		RobotCoordPosOffset(tTempCoord, RzToDirAngle(tTempCoord.dRZ), m_dGunDownBackSafeDis);
		tTempCoord.dZ = dSafeHeight;
		m_pRobotDriver->RobotInverseKinematics(tTempCoord, m_tThickMeasureRefPulse, m_pRobotDriver->m_tTools.tGunTool/*tCameraParam.tCameraTool*/, tTempPulse);

		vtMeasureCoord.push_back(tTempCoord);
		vtMeasurePulse.push_back(tTempPulse);
		vnMeasureType.push_back(E_TRANSITION_POINT);

		// ��������
		m_pRobotDriver->RobotInverseKinematics(tCoord, m_tThickMeasureRefPulse, m_pRobotDriver->m_tTools.tGunTool/*tCameraParam.tCameraTool*/, tPulse);
		vtMeasureCoord.push_back(tCoord);
		vtMeasurePulse.push_back(tPulse);
		vnMeasureType.push_back(E_DOUBLE_LONG_LINE);
		// ���ɵ�	
		vtMeasureCoord.push_back(tTempCoord);
		vtMeasurePulse.push_back(tTempPulse);
		vnMeasureType.push_back(E_TRANSITION_POINT);
	}

	tCoord = GenerateRobotCoord(tPtn, m_dPlatWeldRx, m_dPlatWeldRy, DirAngleToRz(dNorAngle));
	DecomposeExAxle(tCoord, dExAxlePos, m_ptUnit->m_nMeasureAxisNo);
	tTeachData.tMeasureCoordGunTool = tCoord;
	m_vtTeachData.push_back(tTeachData);

	// ���ɵ�
	tTempCoord = tCoord; 
	RobotCoordPosOffset(tTempCoord, RzToDirAngle(tTempCoord.dRZ), m_dGunDownBackSafeDis);
	tTempCoord.dZ = dSafeHeight;
	m_pRobotDriver->RobotInverseKinematics(tTempCoord, m_tStartMeasureRefPulse, m_pRobotDriver->m_tTools.tGunTool/*tCameraParam.tCameraTool*/, tTempPulse);
	vtMeasureCoord.push_back(tTempCoord);
	vtMeasurePulse.push_back(tTempPulse);
	vnMeasureType.push_back(E_TRANSITION_POINT);

	// ��������
	m_pRobotDriver->RobotInverseKinematics(tCoord, m_tStartMeasureRefPulse, m_pRobotDriver->m_tTools.tGunTool/*tCameraParam.tCameraTool*/, tPulse);
	vtMeasureCoord.push_back(tCoord);
	vtMeasurePulse.push_back(tPulse);
	vnMeasureType.push_back(E_DOUBLE_LONG_LINE);

	// !!! �����������޹��ɵ� ֱ���˶������������λ��׼������
	
	// ��������켣��ǹ��������
	CString sFileName;
	sFileName.Format("%s%d-MeasureTrackGunTool.txt", m_sDataSavePath, nGroupNo);
	FILE* pf = fopen(sFileName.GetBuffer(), "w");
	for (int i = 0; i < vtMeasureCoord.size(); i++)
	{
		fprintf(pf, "%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%4d\n",
			vtMeasureCoord[i].dX + vtMeasureCoord[i].dBX,
			vtMeasureCoord[i].dY + vtMeasureCoord[i].dBY,
			vtMeasureCoord[i].dZ + vtMeasureCoord[i].dBZ,
			vtMeasureCoord[i].dX, vtMeasureCoord[i].dY, vtMeasureCoord[i].dZ,
			vtMeasureCoord[i].dRX, vtMeasureCoord[i].dRY, vtMeasureCoord[i].dRZ,
			dExAxlePos, vnMeasureType[i]);
	}
	fclose(pf);

	// ���ߵ���ʹ��
	CString sJobName;
	sJobName.Format("MOVJTEACHBOARD%d", nGroupNo);
	GenerateJobLocalVariable(vtMeasurePulse, MOVJ, sJobName);

	// ��ǹ����ת�������
	for (int i = 0; i < vtMeasureCoord.size(); i++)
	{
		if (false == m_pRobotDriver->MoveToolByWeldGun(vtMeasureCoord[i], m_pRobotDriver->m_tTools.tGunTool,
			vtMeasureCoord[i], tCameraParam.tCameraTool, vtMeasureCoord[i]))
		{
			XUI::MesBox::PopInfo("�����켣{0},ת�����������ʧ��", i);
			return false;
		}
		if (false == m_pRobotDriver->RobotInverseKinematics(vtMeasureCoord[i], vtMeasurePulse[i],
			m_pRobotDriver->m_tTools.tGunTool, vtMeasurePulse[i]))
		{
			XUI::MesBox::PopInfo("�����켣{0},ת�ؽ�����ʧ��", i);
			return false;
		}
	}
	return true;
}

bool SingleBoard::DoTeach(int nGroupNo, const std::vector<T_ANGLE_PULSE>& vtMeasurePulse, const vector<int>& vnMeasureType, double dExAxlePos, int nLayerNo/* = 0*/)
{
	// ִ�в��� DoTeach()  ���ɵ�+������ -> ���ɵ�+������  ->  ���ɵ�+������ ������������켣
	// ִ�в�����ǹ
	// ���ȣ�[��ͼ ���� ̧ǹ���µ�������λ��]
	// ����㣺��ͼ ���� ����
	// �˶���������λ��
	// ����������ͼ ����

	int nTrigSigTime = 15; //  x 0.01 x 1000 ms // I���� 15 * 0.01s = 0.15s = 150ms

	// ���ݼ��
	if (/*vtMeasurePulse.size() < 3 ||*/ vtMeasurePulse.size() != vtMeasurePulse.size())
	{
		XiMessageBox("ʾ�����ݴ���");
		return false;
	}
	double y;
	// ���ݼ�¼����Ա���������̺߳���ʹ�ã�
	SetTeachData(vtMeasurePulse, vnMeasureType, dExAxlePos, y);

	// ���������������ʾ�̲����� �������ֺ������˳��

	vector<int> vnMeasureIdx;
	vnMeasureIdx.clear();
	for (int i = 0; i < vtMeasurePulse.size(); i++)
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

	// ���Ͳ����켣���ݣ�ʾ�̵��� �켣���� �ٶ� �����ź�ʱ�䣩
	bool bTeachSuccess = false;
	if (vtMeasurePulse.size() > 0)
	{
		if (true)//����
		{
			//ʾ�̲�ͼ
			m_ptUnit->SwitchDHCamera(m_ptUnit->m_nTrackCameraNo, true, true, E_ACQUISITION_MODE_SOURCE_SOFTWARE, E_CALL_BACK_MODE_OFF);
			m_ptUnit->m_vpImageCapture[m_ptUnit->m_nTrackCameraNo]->StartAcquisition();
			bTeachSuccess = SpuriousTriggerTeach(nGroupNo, vtMeasurePulse, vnMeasureType, dExAxlePos, false);
		}
		else//Ӳ����
		{
			// ʾ���˶�
			// ʾ����ǹ������ ����� ����  ������ɺ�ر� ��ʡʱ��
			m_ptUnit->SwitchDHCamera(m_ptUnit->m_nTrackCameraNo, true, true, E_ACQUISITION_MODE_SOURCE_HARD_0, E_CALL_BACK_MODE_SAVE_IMAGE);
			m_ptUnit->m_vpImageCapture[m_ptUnit->m_nTrackCameraNo]->StartAcquisition();
		
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

	/*vector<T_ANGLE_PULSE> GetNextTeach;
	while (true)
	{

	}*/

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
	m_vtTeachResult.resize(vtMeasurePulse.size(), tNullTeachResult);
	for (int i = 0; i < vnMeasureIdx.size(); i++) // ����ʾ�̲������
	{
		m_vtTeachResult[vnMeasureIdx[i]] = vtTeachResult[i];
	}

	for (int i = m_vtTeachResult.size() - 1; i >= 0; i--) // ɾ�����ɵ��������
	{
		if (vnMeasureType[i] & E_TRANSITION_POINT)
		{
			m_vtTeachResult.erase(m_vtTeachResult.begin() + i);
		}
	}
	//T_ROBOT_COORS tSafeWeldCoors;
	T_ROBOT_COORS tStarWeldCoors;
	T_ANGLE_PULSE tStarWeldPulse;
	std::vector<T_ANGLE_PULSE> vtStarWeldPulse;
	vector<int> vnStarWeldType;
	vtStarWeldPulse.clear();
	vnStarWeldType.clear();
	m_pRobotDriver->RobotKinematics(vtMeasurePulse[vtMeasurePulse.size() - 1], m_pRobotDriver->m_tTools.tGunTool, tStarWeldCoors);
	// ***�ƶ������һ��˫���߲���λ�ò��ٴβ���һ��˫����***
	//m_vtTeachResult.push_back(m_tLastTeachResult);
	tStarWeldCoors.dX = m_vtTeachResult[m_vtTeachResult.size() - 1].tKeyPtn3D.x;
	tStarWeldCoors.dY = m_vtTeachResult[m_vtTeachResult.size() - 1].tKeyPtn3D.y;
	tStarWeldCoors.dZ = m_vtTeachResult[m_vtTeachResult.size() - 1].tKeyPtn3D.z;
	m_pRobotDriver->RobotInverseKinematics(tStarWeldCoors, m_tStartMeasureRefPulse, m_pRobotDriver->m_tTools.tGunTool, tStarWeldPulse);
	vtStarWeldPulse.push_back(tStarWeldPulse);
	vnStarWeldType.push_back(E_DOUBLE_LONG_LINE);
	//ʾ�̲�ͼ
	//m_ptUnit->SwitchDHCamera(m_ptUnit->m_nTrackCameraNo, true, true, E_ACQUISITION_MODE_SOURCE_SOFTWARE, E_CALL_BACK_MODE_OFF);
	//m_ptUnit->m_vpImageCapture[m_ptUnit->m_nTrackCameraNo]->StartAcquisition();
	T_ROBOT_COORS tCurRobotCoord = m_pRobotDriver->GetCurrentPos();
	m_pRobotDriver->m_cLog->Write("��ǰλ�� X:%.2lf Y:%.2lf Z:%.2lf,Ŀ��λ��X:%.2lf Y:%.2lf Z:%.2lf", tCurRobotCoord.dX, tCurRobotCoord.dY, tCurRobotCoord.dZ, tStarWeldCoors.dX, tStarWeldCoors.dY, tStarWeldCoors.dZ);
	double dDis = TwoPointDis(
		tCurRobotCoord.dX, tCurRobotCoord.dY, tCurRobotCoord.dZ,
		tStarWeldCoors.dX, tStarWeldCoors.dY, tStarWeldCoors.dZ);
	if (fabs(dDis) > 200.0 ||
		fabs(tCurRobotCoord.dX - tStarWeldCoors.dX) > 25.0 ||
		fabs(tCurRobotCoord.dZ - tStarWeldCoors.dZ) > 45.0)
	{
		XiMessageBox("�������������������");
		return false;
	}
	bTeachSuccess =  SpuriousTriggerTeach(nGroupNo, vtStarWeldPulse, vnStarWeldType, dExAxlePos , true);

	if (false == bTeachSuccess)
	{
		m_pRobotDriver->HoldOn();
		Sleep(500);
		m_pRobotDriver->HoldOff();
		XiMessageBox("��ͼʧ�ܣ�");
		m_ptUnit->SwitchDHCamera(m_ptUnit->m_nTrackCameraNo, false);
		return false;
	}
	//m_ptUnit->SwitchDHCamera(m_ptUnit->m_nTrackCameraNo, false);
	// ����ʾ�̽��(ʾ�̼��������)
	SaveTeachResult(nGroupNo);
	m_vtTeachResult;
	
	m_vvtWeldSeamGroupAdjust;
	return true;
}

bool SingleBoard::CalcWeldTrack(int nGroupNo)
{
	// ִ�в��� DoTeach()  ���ɵ�+������ -> ���ɵ�+������  ->  ���ɵ�+������ ������������켣
	// ִ�в�����ǹ
	// ���ȣ�[��ͼ ���� ̧ǹ���µ�������λ��]
	// ����㣺��ͼ ���� ����
	// �˶���������λ��
	// ����������ͼ ����

	//***��������˫���߽�������һ�ΰ��ǳ�ʼ�ι켣***

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
	if (nTeachResultNum != m_nTeachPtnNum + 1)
	{
		XiMessageBox("ʾ�̵�������ͬ");
		return false;
	}

	//bool bFlatWeldContinue = m_bFlatWeldContinue;
	vector<LineOrCircularArcWeldingLine>& vtWeldSeam = m_vvtWeldSeamGroupAdjust[nGroupNo];
	vector<int> vnWeldOrder;
	vector<int> vnFlatWeldNoIdx;
	vnWeldOrder.clear();
	vnFlatWeldNoIdx.clear();
	E_WELD_SEAM_TYPE eWeldSeamType;
	for (int i = 0; i < vtWeldSeam.size(); i++)
	{
		eWeldSeamType = GetWeldSeamType(vtWeldSeam[i]);
		if (E_FLAT_SEAM == eWeldSeamType)
		{
			vnFlatWeldNoIdx.push_back(i);
		}
	}

	std::vector<T_ROBOT_COORS> vtWeldCoord;
	vtWeldCoord.clear();
	vnWeldOrder.insert(vnWeldOrder.end(), vnFlatWeldNoIdx.begin(), vnFlatWeldNoIdx.end());
	XiAlgorithm alg;
	double dWeldDirAngle = alg.CalcArcAngle(m_vtTeachResult[m_vtTeachResult.size() - 1].tKeyPtn3D.x - m_vtTeachResult[m_vtTeachResult.size() - 2].tKeyPtn3D.x, m_vtTeachResult[m_vtTeachResult.size() - 1].tKeyPtn3D.y - m_vtTeachResult[m_vtTeachResult.size() - 2].tKeyPtn3D.y);
	dWeldDirAngle -= 90 * m_nRobotInstallDir;
	if (false == GenerateWeldLineData(m_vtTeachResult[m_vtTeachResult.size() - 2].tKeyPtn3D, m_vtTeachResult[m_vtTeachResult.size() - 1].tKeyPtn3D, 3, m_dPlatWeldRx, m_dPlatWeldRy, dWeldDirAngle, eWeldSeamType, vtWeldCoord))
	{
		XiMessageBox("������յ����꼰��̬����ֱ�����꺸��켣����ʧ��");
		return false;
	}
	vector<int> vnPointType;
	vnPointType.clear();
	for (int i = 0; i < vtWeldCoord.size(); i++)
	{
		vnPointType.push_back(E_WELD_TRACK);
	}
	// ���ɺ��ӹ켣�ļ�
	// ���� ��� ֱ������ �ⲿ������ ��������
	for (int i = 0; i < vnWeldOrder.size(); i++)
	{
		CString sFileName;
		sFileName.Format("%s%d_%d_RealWeldCoord.txt", m_sDataSavePath, nGroupNo, i);
		FILE* pf = fopen(sFileName, "w");
		for (int nPtnIdx = 0; nPtnIdx < vtWeldCoord.size(); nPtnIdx++)
		{
			T_ROBOT_COORS& tCoord = vtWeldCoord[nPtnIdx];
			fprintf(pf, "%d%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%4d%10d\n", nPtnIdx,
				tCoord.dX, tCoord.dY, tCoord.dZ, tCoord.dRX, tCoord.dRY, tCoord.dRZ, m_dTeachExAxlePos, eWeldSeamType, vnPointType[nPtnIdx]);
		}
		fclose(pf);
	}

	//������
	if (m_bAutoMeasureThick)
	{
		T_ALGORITHM_POINT tPtn;
		tPtn.dCoorX = m_vtTeachResult[m_vtTeachResult.size() - 3].tKeyPtn3D.x;
		tPtn.dCoorY = m_vtTeachResult[m_vtTeachResult.size() - 3].tKeyPtn3D.y;
		tPtn.dCoorZ = m_vtTeachResult[m_vtTeachResult.size() - 3].tKeyPtn3D.z;
		T_ALGORITHM_POINT tSegmentStartPoint;
		tSegmentStartPoint.dCoorX = m_vtTeachResult[m_vtTeachResult.size() - 2].tKeyPtn3D.x;
		tSegmentStartPoint.dCoorY = m_vtTeachResult[m_vtTeachResult.size() - 2].tKeyPtn3D.y;
		tSegmentStartPoint.dCoorZ = m_vtTeachResult[m_vtTeachResult.size() - 2].tKeyPtn3D.z;
		T_ALGORITHM_POINT tSegmentEndPoint;
		tSegmentEndPoint.dCoorX = m_vtTeachResult[m_vtTeachResult.size() - 1].tKeyPtn3D.x;
		tSegmentEndPoint.dCoorY = m_vtTeachResult[m_vtTeachResult.size() - 1].tKeyPtn3D.y;
		tSegmentEndPoint.dCoorZ = m_vtTeachResult[m_vtTeachResult.size() - 1].tKeyPtn3D.z;
		double dBoardThick = alg.CalDisPointToLine(tPtn, tSegmentStartPoint, tSegmentEndPoint);
		COPini opini;
		opini.SetFileName(DATA_PATH + m_ptUnit->m_tContralUnit.strUnitName + SYETEM_PARAM_FILE);
		opini.SetSectionName("E_SINGLE_BOARD");
		opini.WriteString("ParamValue1", dBoardThick);
	}
	return true;
}

bool SingleBoard::DoWelding(int nGroupNo, int nWeldNo, E_WELD_SEAM_TYPE eWeldSeamType, std::vector<T_ROBOT_COORS>& vtWeldPathPoints,
	const vector<int>& vnPtnType, const T_WELD_PARA& tWeldPara)
{
	// ִ�и��ٺ��� ���ں������λ��ʱ��ִ�к�����ǹ�˶�

	return true;
}

void SingleBoard::InitParam()
{
	m_dExAxleOffsetDis = 0.0;
	T_ANGLE_PULSE tThickMeasurHomePulse = { -99741,-5651,-54014,68,-57359,-93216,0,0,0 };
	T_ANGLE_PULSE tStartMeasureHomePulse = { -99741,-5651,-54014,68,-57359,-93216,0,0,0 };
	m_tThickMeasureRefPulse = m_pRobotDriver->m_tHomePulse;
	m_tStartMeasureRefPulse = tStartMeasureHomePulse;
	/*m_tThickMeasureRefPulse = m_pRobotDriver->m_tHomePulse;
	m_tStartMeasureRefPulse = m_pRobotDriver->m_tHomePulse;*/
}

bool SingleBoard::SingleGroupWeld(int nGroupNo)
{
	// ������ʼ��
	InitParam();

	// ��������� ��������� �����һ����
	double dSafeHeight;
	std::vector<T_ROBOT_COORS> vtMeasureCoord(0);
	std::vector<T_ANGLE_PULSE> vtMeasurePulse(0);
	std::vector<int> vnMeasureType(0);
	CHECK_BOOL_RETURN(CalcMeasureTrack(nGroupNo, vtMeasureCoord, vtMeasurePulse, vnMeasureType, m_dTeachExAxlePos, dSafeHeight));

	// ִ�в��� DoTeach()  ���ɵ�+������ -> ���ɵ�+������  ->  ���ɵ�+������ ������������켣
	// ִ�в�����ǹ
	// ���ȣ�[��ͼ ���� ̧ǹ���µ�������λ��]
	// ����㣺��ͼ ���� ����
	// �˶���������λ��
	// ����������ͼ ����
	CHECK_BOOL_RETURN(DoTeach(nGroupNo, vtMeasurePulse, vnMeasureType, m_dTeachExAxlePos));

	// �������ݴ��� CalcWeldTrack()
	// ������ ��ʼ�ι켣
	CHECK_BOOL_RETURN(CalcWeldTrack(nGroupNo));

	// ִ�и��ٺ���
	std::vector<T_ROBOT_COORS> vtWeldPathPoints(0);
	std::vector<int> vnWeldPtnType(0);
	T_WELD_PARA	tWeldPara;
	CHECK_BOOL_RETURN(DoWelding(nGroupNo, 0, E_FLAT_SEAM, vtWeldPathPoints, vnWeldPtnType, tWeldPara));
	return true;
}

bool SingleBoard::SetSingleBoardJobState(bool bState)
{
	if(bState == true)
		m_pRobotDriver->SetIntVar(67, 1);
	else 
		m_pRobotDriver->SetIntVar(67, 0);
	Sleep(1000);
	return false;
}

bool SingleBoard::SpuriousTriggerTeach(int nGroupNo, const std::vector<T_ANGLE_PULSE>& vtMeasurePulse, const vector<int>& vnMeasureType, double dExAxlePos, bool bIsMeasureLastPoint)
{
	if (vtMeasurePulse.size() < 1)
	{
		return false;
	}
	//1. ��ȡ�������
	T_CAMREA_PARAM tCameraParam = m_ptUnit->GetCameraParam(m_ptUnit->m_nTrackCameraNo);
	CDHGigeImageCapture* pDHCamera = (CDHGigeImageCapture*)m_ptUnit->GetCameraCtrl(m_ptUnit->m_nTrackCameraNo);
	//IplImage* pBufferImg = pDHCamera->m_pImageBuff;
	IplImage* pBufferImg = cvCreateImage(cvSize(2448, 2048), IPL_DEPTH_8U, 1);
	IplImage* pColorImg = cvCreateImage(cvSize(pBufferImg->width, pBufferImg->height), IPL_DEPTH_8U, 3);
	CString sIniFile;
	sIniFile.Format("%s%s\\TrackDipParam_TrackCam.ini", DATA_PATH, m_ptUnit->m_tContralUnit.strUnitName, m_ptUnit->m_nTrackCameraNo);
	GROUP_STAND_DIP_NS::CImageProcess* pImgProcess = new GROUP_STAND_DIP_NS::CImageProcess(pBufferImg->width, pBufferImg->height, m_pRobotDriver->m_nRobotNo, sIniFile);

	//2. �ⲿ���˶�������λ��
	int nAxisNo = m_ptUnit->m_nMeasureAxisNo;
	if (0 != m_ptUnit->MoveExAxisFun(dExAxlePos, m_pRobotDriver->m_tPulseHighSpeed.dSpeed * 2, nAxisNo))
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
		m_ptUnit->SwitchDHCamera(m_ptUnit->m_nTrackCameraNo, false);
		delete pImgProcess;
		return false;
	}
	
	//3. ���Ͳ���λ�ã��Զ����������5��������  �ֶ�����İ�������������㣩
	vector<MP_USR_VAR_INFO> vtVarInfo;
	int nPos = 40;
	for (auto& it : vtMeasurePulse) {
		vtVarInfo.push_back(m_pRobotDriver->PrepareValData(nPos, it));
		nPos++;
	}
	m_pRobotDriver->SetMultiVar_H(vtVarInfo);

	////4. ��ʼ�����
	//E_DHGIGE_ACQUISITION_MODE eCameraMode = E_ACQUISITION_MODE_CONTINUE;
	//E_DHGIGE_CALL_BACK eCallBack = E_CALL_BACK_MODE_WAIT_IMAGE;
	//m_ptUnit->SwitchDHCamera(m_ptUnit->m_nTrackCameraNo, true, true, eCameraMode, eCallBack); // ��ǹ������ ����� ����
	//m_ptUnit->m_vpImageCapture[m_ptUnit->m_nTrackCameraNo]->StartAcquisition();

	//5. ��ʼ�����˶� ���嵥����в���job 
	int nProIndex = 0;  //����ͼƬ������
	//�Ƿ��Զ������������ֵ�������1 2job 

	if (!bIsMeasureLastPoint) {
		int nIndex = 0;
		if (!m_bAutoMeasureThick) {
			CString cstr;
			cstr.Format("%d", nIndex + 1);
			nIndex++;
			m_pRobotDriver->MoveByJob(vtMeasurePulse[0], m_pRobotDriver->m_tPulseLowSpeed, m_pRobotDriver->m_nExternalAxleType, cstr.GetBuffer());
			m_ptUnit->CheckRobotDone(vtMeasurePulse.back());
			//��ȡͼ��
			pDHCamera->CaptureImage(pBufferImg, 1);
			// ��ԭͼ
			CString sSavePath;
			sSavePath.Format("%sPicture\\Org\\Src_%d.jpg", m_sDataSavePath.GetBuffer(), nProIndex);
			SaveImage(pBufferImg, sSavePath);

			// ת��ɫ�ռ�(תRGB��ɫͼ)
			cvCvtColor(pBufferImg, pColorImg, CV_GRAY2RGB);

			// ͼ���� ���ݲ�ͬ�������� ����ͼ����õ���ά��
			T_TEACH_RESULT tTeachReault;
			T_ANGLE_PULSE tAnglePulse = m_pRobotDriver->GetCurrentPulse();
			if (false == ProcessTeachImage(nProIndex, pImgProcess, pBufferImg, tAnglePulse, E_DOUBLE_LONG_LINE, tTeachReault, tCameraParam.eFlipMode))
			{
				//���޸�
				XUI::MesBox::PopInfo("ʾ�̵�{0}��ͼ����ʧ�ܣ�n = {1} nProIdx = {2} {3} {4}", nProIndex, 0, nProIndex, vnMeasureType[1], m_vnMeasurePtnType[nProIndex]);
				//XiMessageBox("ʾ�̵�%d��ͼ����ʧ�ܣ�n = %d nProIdx = %d %d %d",
				//	nProIndex, 0, nProIndex, vnMeasureType[1], m_vnMeasurePtnType[nProIndex]);
				m_ptUnit->SwitchDHCamera(m_ptUnit->m_nTrackCameraNo, false);
				return false;
			}
			//���洦����
			m_vtTeachResult.push_back(tTeachReault);
			if (nIndex == 1) {
				m_tLastTeachResult = tTeachReault;
			}
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
			/*SetSingleBoardJobState(true);
			Sleep(100);
			SetSingleBoardJobState(false);*/

			nProIndex++;
			//break;
		}
		else {
			for (int i = 0; i < 2; i++) {
				CString cstr;
				cstr.Format("%d", i + 1);
				m_pRobotDriver->MoveByJob(vtMeasurePulse[0], m_pRobotDriver->m_tPulseLowSpeed, m_pRobotDriver->m_nExternalAxleType, cstr.GetBuffer());
				m_ptUnit->CheckRobotDone(vtMeasurePulse.back());
				//��ȡͼ��
				pDHCamera->CaptureImage(pBufferImg, 1);
				// ��ԭͼ
				CString sSavePath;
				sSavePath.Format("%sPicture\\Org\\Src_%d.jpg", m_sDataSavePath.GetBuffer(), nProIndex);
				SaveImage(pBufferImg, sSavePath);

				// ת��ɫ�ռ�(תRGB��ɫͼ)
				cvCvtColor(pBufferImg, pColorImg, CV_GRAY2RGB);

				// ͼ���� ���ݲ�ͬ�������� ����ͼ����õ���ά��
				T_TEACH_RESULT tTeachReault;
				T_ANGLE_PULSE tAnglePulse = m_pRobotDriver->GetCurrentPulse();
				if (false == ProcessTeachImage(nProIndex, pImgProcess, pBufferImg, tAnglePulse, E_DOUBLE_LONG_LINE, tTeachReault, tCameraParam.eFlipMode))
				{
					//���޸�
					XUI::MesBox::PopInfo("ʾ�̵�{0}��ͼ����ʧ�ܣ�n = {1} nProIdx = {2} {3} {4}", nProIndex, 0, nProIndex, vnMeasureType[1], m_vnMeasurePtnType[nProIndex]);
					//XiMessageBox("ʾ�̵�%d��ͼ����ʧ�ܣ�n = %d nProIdx = %d %d %d",
					//	nProIndex, 0, nProIndex, vnMeasureType[1], m_vnMeasurePtnType[nProIndex]);
					m_ptUnit->SwitchDHCamera(m_ptUnit->m_nTrackCameraNo, false);
					return false;
				}
				//���洦����
				m_vtTeachResult.push_back(tTeachReault);
				if (i == 1) {
					m_tLastTeachResult = tTeachReault;
				}
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
				/*SetSingleBoardJobState(true);
				Sleep(100);
				SetSingleBoardJobState(false);*/

				nProIndex++;
				//break;

			}
		}
	}
	else {
		for (int n = 0; n < vtMeasurePulse.size(); n++)//0 6 7 10 11 15
		{
			m_pRobotDriver->MoveByJob(vtMeasurePulse[n], m_pRobotDriver->m_tTeachSpeed, m_pRobotDriver->m_nExternalAxleType, "MOVJ");
			if (!m_ptUnit->CheckRobotDone(vtMeasurePulse[n]))
			{
				XiMessageBox("������δ���е�λ");
				m_ptUnit->SwitchDHCamera(m_ptUnit->m_nTrackCameraNo, false);
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
			long long dStart = XI_clock();

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
				XUI::MesBox::PopInfo("ʾ�̵�{0}��ͼ����ʧ�ܣ�n = {1} nProIdx = {2} {3} {4}", nProIndex, 0, nProIndex, vnMeasureType[1], m_vnMeasurePtnType[nProIndex]);
				//XiMessageBox("ʾ�̵�%d��ͼ����ʧ�ܣ�n = %d nProIdx = %d %d %d",
				//	nProIndex, n, nProIndex, vnMeasureType[n], m_vnMeasurePtnType[nProIndex]);
				m_ptUnit->SwitchDHCamera(m_ptUnit->m_nTrackCameraNo, false);
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
			long long dEnd = XI_clock() - dStart;
		}
	}

	//m_ptUnit->SwitchDHCamera(m_ptUnit->m_nTrackCameraNo, false);
	cvReleaseImage(&pColorImg);
	delete pImgProcess;

	return true;
	
}

bool SingleBoard::SavePointCloudProcessResult(LineOrCircularArcWeldingLine * pWeldSeams, int nWeldLineNumber)
{
	m_vtWeldSeamData.clear();
	CString sOutFileName2 = OUTPUT_PATH + m_ptUnit->m_tContralUnit.strUnitName + "\\" + POINT_CLOUD_IDENTIFY_RESULT;
	FILE* pf2 = fopen(sOutFileName2.GetBuffer(), "w");
	double dAdjustZ = 0;

	std::vector<T_WELD_PARA> vtWeldParaFlat;
	std::vector<T_WELD_PARA> vtWeldParaStand;
	CHECK_BOOL_RETURN(GetCurWeldParam(E_FLAT_SEAM, vtWeldParaFlat));
	CHECK_BOOL_RETURN(GetCurWeldParam(E_STAND_SEAM, vtWeldParaStand));
	for (int i = 0; i < nWeldLineNumber; i++)
	{
		LineOrCircularArcWeldingLine tWeld = pWeldSeams[i];

		bool bIsStandWeld = IsStandWeldSeam(tWeld.StartNormalVector);
		int nWeldSize = bIsStandWeld ? vtWeldParaStand[0].nWeldAngleSize : vtWeldParaFlat[0].nWeldAngleSize;
		double dStartWeldHoleSize = false == tWeld.StartPointType ? 0.0 : m_dWeldHoleSize;
		double dEndWeldHoleSize = false == tWeld.EndPointType ? 0.0 : m_dWeldHoleSize;
		fprintf(pf2, "%d %4d%4d%11.3lf%4d%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%4d%4d 1 4 4 %4d%11.3lf%11.3lf 0\n",
			i, tWeld.IsArc, tWeld.isClockwise, tWeld.ZSide, tWeld.isLeft,
			tWeld.CenterPoint.x, tWeld.CenterPoint.y, tWeld.CenterPoint.z - dAdjustZ,
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

/*
bool SingleBoard::CalcSeamTeachData(int nGroupNo, int nSeamNo, double dExAxlePos, vector<LineOrCircularArcWeldingLine> vtSeamGroup, vector<T_TEACH_DATA>& vtTeachData)
{
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
	
	// ************�˴���Ҫ���� ���յ��Ƿ���а��Ǽ��������� ���ֲ�ͬ�Ĳ����켣���㺯��

	// �����������
	CHECK_BOOL_RETURN(CalcThickMeasureData(m_bAutoMeasureThick, nGroupNo, nSeamNo, tSeam, dExAxlePos, vtTempTeachData));
	AppendTeachData(vtTeachData, vtTempTeachData);
	
	// �������������
	CHECK_BOOL_RETURN(CalcWeldMidTeachData(nGroupNo, nSeamNo, tSeam, dExAxlePos, vtTempTeachData));
	AppendTeachData(vtTeachData, vtTempTeachData);

	return true;
}

bool SingleBoard::CalcThickMeasureData(bool bNeedMeasureThick, int nGroupNo, int nSeamNo, LineOrCircularArcWeldingLine tSeam, double dExAxlePos, vector<T_TEACH_DATA>& vtTeachData)
{
	vtTeachData.clear();
	if (false == bNeedMeasureThick) return true;
}

bool SingleBoard::CalcWeldMidTeachData(int nGroupNo, int nSeamNo, LineOrCircularArcWeldingLine tSeam, double dExAxlePos, vector<T_TEACH_DATA>& vtTeachData)
{
	vtTeachData.clear();
	T_ROBOT_COORS tCoord;
	T_TEACH_DATA tTeachData;
	tTeachData.nWeldNo = nSeamNo;
	tTeachData.nMeasurePtnNo = 0;
	tTeachData.nMeasureType = E_DOUBLE_LONG_LINE | E_TEACH_POINT;
	tTeachData.eWeldSeamType =  E_FLAT_SEAM;
	tTeachData.eAttribute = E_BELONG_START;
	tTeachData.eEndpointType = E_FREE_POINT;

	CvPoint3D64f tPtn;
	tPtn.x = (tSeam.StartPoint.x + tSeam.EndPoint.x) / 2.0;
	tPtn.y = (tSeam.StartPoint.y + tSeam.EndPoint.y) / 2.0;
	tPtn.z = (tSeam.StartPoint.z + tSeam.EndPoint.z) / 2.0;

	XiAlgorithm alg;
	double dNorAngle = alg.CalcArcAngle(tSeam.StartNormalVector.x, tSeam.StartNormalVector.y);
	double dWeldNorAngle = alg.CalcArcAngle(tSeam.EndPoint.x - tSeam.StartPoint.x, tSeam.EndPoint.y - tSeam.StartPoint.y);

	tCoord = GenerateRobotCoord(tPtn, m_dPlatWeldRx, m_dPlatWeldRy, DirAngleToRz(dNorAngle));
	DecomposeExAxle(tCoord, dExAxlePos);
	tTeachData.tMeasureCoordGunTool = tCoord;
	vtTeachData.push_back(tTeachData);

	RobotCoordPosOffset(tCoord, dWeldNorAngle, 100.0);
	DecomposeExAxle(tCoord, dExAxlePos);
	tTeachData.tMeasureCoordGunTool = tCoord;
	vtTeachData.push_back(tTeachData);

	return true;
}

bool SingleBoard::SortMeasureCoord(double dPartHeight, double dFirstSeamNorAngle, double dExAxleOffsetDis,
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
		else if (false == JudgeAngle(180.0, -180.0, tPreCoord.dRZ, tCurCoord.dRZ, 100.0)) // ���ڲ�����Rz����100��
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

bool SingleBoard::AddSafeCoord(vector<T_ROBOT_COORS>& vtMeasureCoord, vector<int>& vnMeasureType,
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
*/