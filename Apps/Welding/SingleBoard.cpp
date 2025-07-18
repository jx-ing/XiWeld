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
		// 工字钢 小件接口 
		pLineOrCircularArcWeldingLine = SmallSamplePointCloudToWeldingLines(
			(Three_DPoint*)pPointCloud, PointCloudSize, &nWeldingSeamNumber, 
			bInstallDir, m_dZPallet, 5.0, false, false, 0.0, false, false, 2);
		//// 工字钢 牛腿接口
		//pLineOrCircularArcWeldingLine = BracketPointCloudToWeldingLines(
		//	(Three_DPoint*)pPointCloud, PointCloudSize, &nWeldingSeamNumber, bInstallDir, m_dZPallet, 5.0, false);
		//// 圆弧
		//pLineOrCircularArcWeldingLine = ArcPlatePointCloudToWeldingLinesExtraction(
		//	(Three_DPoint*)pPointCloud, PointCloudSize, &nWeldingSeamNumber,
		//	bInstallDir, true, m_dZPallet, 5.0, false, true, false, 15, 4);
		SavePointCloudProcessResult((LineOrCircularArcWeldingLine*)pLineOrCircularArcWeldingLine, nWeldingSeamNumber);
		ReleaseWeldingLines(&pLineOrCircularArcWeldingLine);
		LoadCloudProcessResultNew(OUTPUT_PATH + m_ptUnit->m_tContralUnit.strUnitName + "\\" + POINT_CLOUD_IDENTIFY_RESULT);
	}
	catch (...)
	{
		XiMessageBox("单筋板处理异常");
		return false;
	}

	//已修改
	XUI::MesBox::PopInfo("工件数量为:{0}", nWeldingSeamNumber);
	//XiMessageBox("工件数量为:%d", nWeldingSeamNumber);
	if (nWeldingSeamNumber <= 0)
	{
		return false;
	}
	return true;
}

bool SingleBoard::WeldSeamGrouping(int& nWeldGroupNum)
{
	// 自动分组
	if (g_bAutoGroupingMark)
	{
		bool bRst = WeldSeamGroupingAuto(nWeldGroupNum);
		return bRst;
	}
	//----------------------------------
	nWeldGroupNum = 0;
	double dGroupDisThreshold = 3.0; // mm
	double dMinFlatSeamLen = m_dHandEyeDis > 0.0 ? m_dHandEyeDis + m_dMeasureDisThreshold * 2.0 : 40.0; // 最小平焊长度
	double dMinStandSeamLen = 40.0; // 最小立焊长度
	m_vvtWeldSeamGroup.clear();
	// 区分平焊立焊 及 平焊起点终点排序
	vector<WeldLineInfo> vtWeldSeamPlane(0);
	vector<WeldLineInfo> vtWeldLineStandard(0);
	vector<WeldLineInfo> vtWeldLineArc(0);
	vector<WeldLineInfo> vtWeldLinePlaneOverLength(0);
	vector<CvPoint3D64f> vtPtn(0); // 平焊分组使用(合并重合的点)
	vector<vector<int>> vvnWeldNo(0); // 平焊分组使用(每个合并点所关联的多个焊缝编号号)
	XiAlgorithm alg;
	for (int nWeldNo = 0; nWeldNo < m_vtWeldSeamInfo.size(); nWeldNo++)
	{
		WeldLineInfo& tLineSeamInfo = m_vtWeldSeamInfo[nWeldNo];
		LineOrCircularArcWeldingLine& tLineSeam = tLineSeamInfo.tWeldLine;
		double dWeldSeamLen = TwoPointDis(
			tLineSeam.StartPoint.x, tLineSeam.StartPoint.y, tLineSeam.StartPoint.z,
			tLineSeam.EndPoint.x, tLineSeam.EndPoint.y, tLineSeam.EndPoint.z);
		tLineSeamInfo.tAtrribute.bWeldMode = false; // 默认不跟踪
		if (IsStandWeldSeam(tLineSeam.StartNormalVector)) // 立焊起点终点法向量Z值为0
		{
			if (dWeldSeamLen < dMinStandSeamLen)
			{
				XUI::MesBox::PopInfo("立焊焊缝{0} 长度小于{1},自动删除！", nWeldNo, dMinStandSeamLen);
				continue;
			}
			vtWeldLineStandard.push_back(tLineSeamInfo);
		}
		else
		{
			if (true == m_bTrackingEnable && dWeldSeamLen > m_dTrackingLengthThreshold) // 满足跟踪条件？
			{
				tLineSeamInfo.tAtrribute.bWeldMode = true;
			}
			// 圆弧焊缝单独保存
			if (WeldingLineIsArc(tLineSeam))
			{
				vtWeldLineArc.push_back(tLineSeamInfo);
				continue;
			}
			// 超长平焊缝单独保存
			if (dWeldSeamLen > m_dLengthSeamThreshold)
			{
				vtWeldLinePlaneOverLength.push_back(tLineSeamInfo);
				continue;
			}
			if (dWeldSeamLen < dMinFlatSeamLen)
			{
				XUI::MesBox::PopInfo("平焊焊缝{0} 长度小于{1},自动删除！", nWeldNo, dMinFlatSeamLen);
				continue;
			}
			if (dWeldSeamLen < m_dShortSeamThreshold &&
				0 < tLineSeam.StartPointType &&
				0 < tLineSeam.EndPointType)
			{
				XUI::MesBox::PopInfo("双侧干涉平焊焊缝{0} 长度小于{1},自动删除！", nWeldNo, m_dShortSeamThreshold);
				continue;
			}
			double dNorAngle = alg.CalcArcAngle(tLineSeam.StartNormalVector.x, tLineSeam.StartNormalVector.y);
			double dWeldSeamDir = alg.CalcArcAngle(tLineSeam.EndPoint.x - tLineSeam.StartPoint.x, tLineSeam.EndPoint.y - tLineSeam.StartPoint.y);
			if (false == JudgeDirAngle(dNorAngle + (90.0 * m_nRobotInstallDir), dWeldSeamDir)) // 起点终点方向非跟踪焊接方向
			{
				swap(tLineSeam.StartPoint, tLineSeam.EndPoint); // 交换起点终点数据信息
				swap(tLineSeam.StartNormalVector, tLineSeam.EndNormalVector);
				swap(tLineSeam.StartPointType, tLineSeam.EndPointType);
				swap(tLineSeamInfo.tAtrribute.dStartHoleSize, tLineSeamInfo.tAtrribute.dEndHoleSize);
				swap(tLineSeamInfo.tAtrribute.nStartWrapType, tLineSeamInfo.tAtrribute.nEndWrapType);
				m_vtWeldSeamData[nWeldNo] = tLineSeam;
				m_vtWeldSeamInfo[nWeldNo].tWeldLine = tLineSeam;
				m_vtWeldSeamInfo[nWeldNo].tAtrribute = tLineSeamInfo.tAtrribute;
			}
			vtWeldSeamPlane.push_back(tLineSeamInfo);
			// vtPtn中查找是否有与起点终点相近的点 有 对应的vnWeldNo添加一个焊缝编号，没有添加一个点到vtPtn并记录新的焊缝编号
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
					XiMessageBox("有两个以上平焊相交于一点，请检查识别结果！");
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

	// 平焊分组 (每组首尾相连平焊焊缝的索引)
	sort(vvnWeldNo.begin(), vvnWeldNo.end(), [](const vector<int>& vn1, const vector<int>& vn2) -> bool { return vn1.size() > vn2.size(); });
	vector<vector<int>> vvnEachGroupIndex(0);
	while (vvnWeldNo.size() > 0) // 每个端点
	{
		vector<int> vnWeldNo(vvnWeldNo[0]); // 取剩余中的第一个点，查找该点所在焊缝组的所有平焊
		bool bFound = true;
		while (true == bFound && vvnWeldNo.size() > 0)
		{
			bFound = false;
			for (int nPtnNo = 0; (!bFound) && (nPtnNo < vvnWeldNo.size()); nPtnNo++) // 未分组的每个公共点
			{
				for (int nWeldNo = 0; (!bFound) && (nWeldNo < vvnWeldNo[nPtnNo].size()); nWeldNo++) // 每个公共点的多个焊缝号
				{
					for (int nNo = 0; (!bFound) && (nNo < vnWeldNo.size()); nNo++) // 已分组中的每个焊缝号
					{
						if (vnWeldNo[nNo] == vvnWeldNo[nPtnNo][nWeldNo])
						{
							bFound = true;
							for (int nNewWeldNo = 0; nNewWeldNo < vvnWeldNo[nPtnNo].size(); nNewWeldNo++) // 只记录当前未记录焊缝号
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
	for (int nGroupNo = 0; nGroupNo < vvnEachGroupIndex.size(); nGroupNo++) // 区分每组焊缝
	{
		for (int nWeldNo = 0; nWeldNo < vvnEachGroupIndex[nGroupNo].size(); nWeldNo++)
		{
			m_vvtWeldLineInfoGroup[nGroupNo].push_back(m_vtWeldSeamInfo[vvnEachGroupIndex[nGroupNo][nWeldNo]]);
		}
	}

	// 平焊查找立焊
	double dPlaneStandardDisThreshold = 2.0; // mm
	for (int nGroupNo = 0; nGroupNo < m_vvtWeldLineInfoGroup.size(); nGroupNo++)
	{
		vector<WeldLineInfo> vtGroup = m_vvtWeldLineInfoGroup[nGroupNo];
		for (int nSeamNo = 0; nSeamNo < vtGroup.size(); nSeamNo++)
		{
			WeldLineInfo& tSeamPlane = vtGroup[nSeamNo];
			if (0 < tSeamPlane.tWeldLine.StartPointType) // 起点有立焊
			{
				for (int nStardWeldNo = 0; nStardWeldNo < vtWeldLineStandard.size(); nStardWeldNo++)
				{
					LineOrCircularArcWeldingLine& tSeamStandard = vtWeldLineStandard[nStardWeldNo].tWeldLine;
					double dDis = TwoPointDis(tSeamPlane.tWeldLine.StartPoint.x, tSeamPlane.tWeldLine.StartPoint.y, tSeamPlane.tWeldLine.StartPoint.z,
						tSeamStandard.StartPoint.x, tSeamStandard.StartPoint.y, tSeamStandard.StartPoint.z);
					if (fabs(dDis) < dPlaneStandardDisThreshold)
					{
						if (false == IsAcuteSeamStand(tSeamStandard, 80.0)) // 锐角立焊不保存到分组 但需要删除
						{
							m_vvtWeldLineInfoGroup[nGroupNo].push_back(vtWeldLineStandard[nStardWeldNo]);
						}
						else
						{
							WriteLog("锐角立焊焊缝自动删除1");
						}
						vtWeldLineStandard.erase(vtWeldLineStandard.begin() + nStardWeldNo);
						break;
					}
				}
			}

			if (0 < tSeamPlane.tWeldLine.EndPointType) // 终点有立焊
			{
				for (int nStardWeldNo = 0; nStardWeldNo < vtWeldLineStandard.size(); nStardWeldNo++)
				{
					LineOrCircularArcWeldingLine& tSeamStandard = vtWeldLineStandard[nStardWeldNo].tWeldLine;
					double dDis = TwoPointDis(tSeamPlane.tWeldLine.EndPoint.x, tSeamPlane.tWeldLine.EndPoint.y, tSeamPlane.tWeldLine.EndPoint.z,
						tSeamStandard.StartPoint.x, tSeamStandard.StartPoint.y, tSeamStandard.StartPoint.z);
					if (fabs(dDis) < dPlaneStandardDisThreshold)
					{
						if (false == IsAcuteSeamStand(tSeamStandard, 80.0)) // 锐角立焊不保存到分组 但需要删除
						{
							m_vvtWeldLineInfoGroup[nGroupNo].push_back(vtWeldLineStandard[nStardWeldNo]);
						}
						else
						{
							WriteLog("锐角立焊焊缝自动删除2");
						}
						vtWeldLineStandard.erase(vtWeldLineStandard.begin() + nStardWeldNo);
						break;
					}
				}
			}
		}
	}

	// 与平焊无关联的单独立焊焊缝 保存为单独一组
	for (int nAloneStandWeldNo = 0; nAloneStandWeldNo < vtWeldLineStandard.size(); nAloneStandWeldNo++)
	{
		vtWeldLineInfo.clear();
		vtWeldLineInfo.resize(1, vtWeldLineStandard[nAloneStandWeldNo]);
		m_vvtWeldLineInfoGroup.push_back(vtWeldLineInfo);
	}

	// 平焊超长焊缝保存为独立一组
	for (int nWeldArcNo = 0; nWeldArcNo < vtWeldLinePlaneOverLength.size(); nWeldArcNo++)
	{
		vtWeldLineInfo.clear();
		vtWeldLineInfo.resize(1, vtWeldLinePlaneOverLength[nWeldArcNo]);
		m_vvtWeldLineInfoGroup.push_back(vtWeldLineInfo);
	}

	// 圆弧焊缝保存为独立一组
	for (int nWeldArcNo = 0; nWeldArcNo < vtWeldLineArc.size(); nWeldArcNo++)
	{
		vtWeldLineInfo.clear();
		vtWeldLineInfo.resize(1, vtWeldLineArc[nWeldArcNo]);
		m_vvtWeldLineInfoGroup.push_back(vtWeldLineInfo);
	}

	// 各组焊缝排序
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

	// 计算测量点 测厚两个点 不测厚一个点
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

		// 过渡点
		tTempCoord = tCoord; 
		RobotCoordPosOffset(tTempCoord, RzToDirAngle(tTempCoord.dRZ), m_dGunDownBackSafeDis);
		tTempCoord.dZ = dSafeHeight;
		m_pRobotDriver->RobotInverseKinematics(tTempCoord, m_tThickMeasureRefPulse, m_pRobotDriver->m_tTools.tGunTool/*tCameraParam.tCameraTool*/, tTempPulse);

		vtMeasureCoord.push_back(tTempCoord);
		vtMeasurePulse.push_back(tTempPulse);
		vnMeasureType.push_back(E_TRANSITION_POINT);

		// 测厚测量点
		m_pRobotDriver->RobotInverseKinematics(tCoord, m_tThickMeasureRefPulse, m_pRobotDriver->m_tTools.tGunTool/*tCameraParam.tCameraTool*/, tPulse);
		vtMeasureCoord.push_back(tCoord);
		vtMeasurePulse.push_back(tPulse);
		vnMeasureType.push_back(E_DOUBLE_LONG_LINE);
		// 过渡点	
		vtMeasureCoord.push_back(tTempCoord);
		vtMeasurePulse.push_back(tTempPulse);
		vnMeasureType.push_back(E_TRANSITION_POINT);
	}

	tCoord = GenerateRobotCoord(tPtn, m_dPlatWeldRx, m_dPlatWeldRy, DirAngleToRz(dNorAngle));
	DecomposeExAxle(tCoord, dExAxlePos, m_ptUnit->m_nMeasureAxisNo);
	tTeachData.tMeasureCoordGunTool = tCoord;
	m_vtTeachData.push_back(tTeachData);

	// 过渡点
	tTempCoord = tCoord; 
	RobotCoordPosOffset(tTempCoord, RzToDirAngle(tTempCoord.dRZ), m_dGunDownBackSafeDis);
	tTempCoord.dZ = dSafeHeight;
	m_pRobotDriver->RobotInverseKinematics(tTempCoord, m_tStartMeasureRefPulse, m_pRobotDriver->m_tTools.tGunTool/*tCameraParam.tCameraTool*/, tTempPulse);
	vtMeasureCoord.push_back(tTempCoord);
	vtMeasurePulse.push_back(tTempPulse);
	vnMeasureType.push_back(E_TRANSITION_POINT);

	// 起点测量点
	m_pRobotDriver->RobotInverseKinematics(tCoord, m_tStartMeasureRefPulse, m_pRobotDriver->m_tTools.tGunTool/*tCameraParam.tCameraTool*/, tPulse);
	vtMeasureCoord.push_back(tCoord);
	vtMeasurePulse.push_back(tPulse);
	vnMeasureType.push_back(E_DOUBLE_LONG_LINE);

	// !!! 起点测量结束无过渡点 直接运动到起点测量结果位置准备焊接
	
	// 保存测量轨迹焊枪工具坐标
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

	// 离线调试使用
	CString sJobName;
	sJobName.Format("MOVJTEACHBOARD%d", nGroupNo);
	GenerateJobLocalVariable(vtMeasurePulse, MOVJ, sJobName);

	// 焊枪工具转相机工具
	for (int i = 0; i < vtMeasureCoord.size(); i++)
	{
		if (false == m_pRobotDriver->MoveToolByWeldGun(vtMeasureCoord[i], m_pRobotDriver->m_tTools.tGunTool,
			vtMeasureCoord[i], tCameraParam.tCameraTool, vtMeasureCoord[i]))
		{
			XUI::MesBox::PopInfo("测量轨迹{0},转相机工具坐标失败", i);
			return false;
		}
		if (false == m_pRobotDriver->RobotInverseKinematics(vtMeasureCoord[i], vtMeasurePulse[i],
			m_pRobotDriver->m_tTools.tGunTool, vtMeasurePulse[i]))
		{
			XUI::MesBox::PopInfo("测量轨迹{0},转关节坐标失败", i);
			return false;
		}
	}
	return true;
}

bool SingleBoard::DoTeach(int nGroupNo, const std::vector<T_ANGLE_PULSE>& vtMeasurePulse, const vector<int>& vnMeasureType, double dExAxlePos, int nLayerNo/* = 0*/)
{
	// 执行测量 DoTeach()  过渡点+测量点 -> 过渡点+测量点  ->  过渡点+测量点 最多三个测量轨迹
	// 执行测量下枪
	// 测厚度：[采图 处理 抬枪并下到起点测量位置]
	// 测起点：采图 处理 测量
	// 运动到起点测量位置
	// 测锁定：采图 处理

	int nTrigSigTime = 15; //  x 0.01 x 1000 ms // I变量 15 * 0.01s = 0.15s = 150ms

	// 数据检查
	if (/*vtMeasurePulse.size() < 3 ||*/ vtMeasurePulse.size() != vtMeasurePulse.size())
	{
		XiMessageBox("示教数据错误！");
		return false;
	}
	double y;
	// 数据记录到成员变量（供线程函数使用）
	SetTeachData(vtMeasurePulse, vnMeasureType, dExAxlePos, y);

	// 区分搜索测量点和示教测量点 并保存拆分后的索引顺序

	vector<int> vnMeasureIdx;
	vnMeasureIdx.clear();
	for (int i = 0; i < vtMeasurePulse.size(); i++)
	{
		vnMeasureIdx.push_back(i);
	}
	std::map<int, int> mnnIdx;
	mnnIdx.clear();
	// 获取扫描测量及过渡点坐标 m_vnMeasurePtnType扫描类型 且 m_vnTeachTrackOrder序号递增
	for (int i = 0; i < m_vnMeasurePtnType.size(); i++)
	{
		if (m_vnMeasurePtnType[i] & E_SEARCH_POINT)
		{
			mnnIdx[m_vnTeachTrackOrder[i]] = m_vnTeachIdxWithTemp[i];
		}
	}
	if (0 != mnnIdx.size() % 2)
	{
		XUI::MesBox::PopOkCancel("搜索测量点数量{0}错误!", mnnIdx.size());
		return false;
	}
	m_vnMeasurePtnType;   // 此处调试用
	m_vnTeachTrackOrder;  // 此处调试用
	m_vnTeachIdxWithTemp; // 此处调试用

	// 发送测量轨迹数据（示教点数 轨迹坐标 速度 触发信号时间）
	bool bTeachSuccess = false;
	if (vtMeasurePulse.size() > 0)
	{
		if (true)//软触发
		{
			//示教采图
			m_ptUnit->SwitchDHCamera(m_ptUnit->m_nTrackCameraNo, true, true, E_ACQUISITION_MODE_SOURCE_SOFTWARE, E_CALL_BACK_MODE_OFF);
			m_ptUnit->m_vpImageCapture[m_ptUnit->m_nTrackCameraNo]->StartAcquisition();
			bTeachSuccess = SpuriousTriggerTeach(nGroupNo, vtMeasurePulse, vnMeasureType, dExAxlePos, false);
		}
		else//硬触发
		{
			// 示教运动
			// 示教下枪过程中 开相机 激光  处理完成后关闭 节省时间
			m_ptUnit->SwitchDHCamera(m_ptUnit->m_nTrackCameraNo, true, true, E_ACQUISITION_MODE_SOURCE_HARD_0, E_CALL_BACK_MODE_SAVE_IMAGE);
			m_ptUnit->m_vpImageCapture[m_ptUnit->m_nTrackCameraNo]->StartAcquisition();
		
			if (false == m_ptUnit->TeachMove(vtMeasurePulse, dExAxlePos, vnMeasureType, nTrigSigTime))
			{
				XiMessageBox("精确测量运动失败!");
				return false;
			}
			// 开启图像处理线程
			CWinThread* pThread = AfxBeginThread(ThreadTeachProcess, this);
			pThread->m_bAutoDelete = false;
			// 等待处理线程退出
			bTeachSuccess = WaitAndCheckThreadExit(pThread, "示教线程");
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

	// 示教运动结果保存到 m_vtTeachResult 单独保存后清除 保存后续搜索测量结果
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

	// 汇总结果
	m_vtTeachResult.resize(vtMeasurePulse.size(), tNullTeachResult);
	for (int i = 0; i < vnMeasureIdx.size(); i++) // 保存示教测量结果
	{
		m_vtTeachResult[vnMeasureIdx[i]] = vtTeachResult[i];
	}

	for (int i = m_vtTeachResult.size() - 1; i >= 0; i--) // 删除过渡点多余数据
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
	// ***移动到最后一个双长边测量位置并再次测量一次双长边***
	//m_vtTeachResult.push_back(m_tLastTeachResult);
	tStarWeldCoors.dX = m_vtTeachResult[m_vtTeachResult.size() - 1].tKeyPtn3D.x;
	tStarWeldCoors.dY = m_vtTeachResult[m_vtTeachResult.size() - 1].tKeyPtn3D.y;
	tStarWeldCoors.dZ = m_vtTeachResult[m_vtTeachResult.size() - 1].tKeyPtn3D.z;
	m_pRobotDriver->RobotInverseKinematics(tStarWeldCoors, m_tStartMeasureRefPulse, m_pRobotDriver->m_tTools.tGunTool, tStarWeldPulse);
	vtStarWeldPulse.push_back(tStarWeldPulse);
	vnStarWeldType.push_back(E_DOUBLE_LONG_LINE);
	//示教采图
	//m_ptUnit->SwitchDHCamera(m_ptUnit->m_nTrackCameraNo, true, true, E_ACQUISITION_MODE_SOURCE_SOFTWARE, E_CALL_BACK_MODE_OFF);
	//m_ptUnit->m_vpImageCapture[m_ptUnit->m_nTrackCameraNo]->StartAcquisition();
	T_ROBOT_COORS tCurRobotCoord = m_pRobotDriver->GetCurrentPos();
	m_pRobotDriver->m_cLog->Write("当前位置 X:%.2lf Y:%.2lf Z:%.2lf,目标位置X:%.2lf Y:%.2lf Z:%.2lf", tCurRobotCoord.dX, tCurRobotCoord.dY, tCurRobotCoord.dZ, tStarWeldCoors.dX, tStarWeldCoors.dY, tStarWeldCoors.dZ);
	double dDis = TwoPointDis(
		tCurRobotCoord.dX, tCurRobotCoord.dY, tCurRobotCoord.dZ,
		tStarWeldCoors.dX, tStarWeldCoors.dY, tStarWeldCoors.dZ);
	if (fabs(dDis) > 200.0 ||
		fabs(tCurRobotCoord.dX - tStarWeldCoors.dX) > 25.0 ||
		fabs(tCurRobotCoord.dZ - tStarWeldCoors.dZ) > 45.0)
	{
		XiMessageBox("测量误差与理论误差过大！");
		return false;
	}
	bTeachSuccess =  SpuriousTriggerTeach(nGroupNo, vtStarWeldPulse, vnStarWeldType, dExAxlePos , true);

	if (false == bTeachSuccess)
	{
		m_pRobotDriver->HoldOn();
		Sleep(500);
		m_pRobotDriver->HoldOff();
		XiMessageBox("采图失败！");
		m_ptUnit->SwitchDHCamera(m_ptUnit->m_nTrackCameraNo, false);
		return false;
	}
	//m_ptUnit->SwitchDHCamera(m_ptUnit->m_nTrackCameraNo, false);
	// 保存示教结果(示教及搜索结果)
	SaveTeachResult(nGroupNo);
	m_vtTeachResult;
	
	m_vvtWeldSeamGroupAdjust;
	return true;
}

bool SingleBoard::CalcWeldTrack(int nGroupNo)
{
	// 执行测量 DoTeach()  过渡点+测量点 -> 过渡点+测量点  ->  过渡点+测量点 最多三个测量轨迹
	// 执行测量下枪
	// 测厚度：[采图 处理 抬枪并下到起点测量位置]
	// 测起点：采图 处理 测量
	// 运动到起点测量位置
	// 测锁定：采图 处理

	//***根据两个双长边结果，拟合一次包角初始段轨迹***

	// 删除第 nGroupNo 组焊缝轨迹文件 
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

	// 检查示教结果数量
	int nTeachResultNum = m_vtTeachResult.size();
	if (nTeachResultNum != m_nTeachPtnNum + 1)
	{
		XiMessageBox("示教点数不相同");
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
		XiMessageBox("由起点终点坐标及姿态生成直角坐标焊缝轨迹点云失败");
		return false;
	}
	vector<int> vnPointType;
	vnPointType.clear();
	for (int i = 0; i < vtWeldCoord.size(); i++)
	{
		vnPointType.push_back(E_WELD_TRACK);
	}
	// 生成焊接轨迹文件
	// 保存 序号 直角坐标 外部轴坐标 焊接类型
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

	//计算厚度
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
	// 执行跟踪焊接 不在焊接起点位置时先执行焊接下枪运动

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
	// 参数初始化
	InitParam();

	// 计算测量点 测厚两个点 不测厚一个点
	double dSafeHeight;
	std::vector<T_ROBOT_COORS> vtMeasureCoord(0);
	std::vector<T_ANGLE_PULSE> vtMeasurePulse(0);
	std::vector<int> vnMeasureType(0);
	CHECK_BOOL_RETURN(CalcMeasureTrack(nGroupNo, vtMeasureCoord, vtMeasurePulse, vnMeasureType, m_dTeachExAxlePos, dSafeHeight));

	// 执行测量 DoTeach()  过渡点+测量点 -> 过渡点+测量点  ->  过渡点+测量点 最多三个测量轨迹
	// 执行测量下枪
	// 测厚度：[采图 处理 抬枪并下到起点测量位置]
	// 测起点：采图 处理 测量
	// 运动到起点测量位置
	// 测锁定：采图 处理
	CHECK_BOOL_RETURN(DoTeach(nGroupNo, vtMeasurePulse, vnMeasureType, m_dTeachExAxlePos));

	// 测量数据处理 CalcWeldTrack()
	// 计算板厚 初始段轨迹
	CHECK_BOOL_RETURN(CalcWeldTrack(nGroupNo));

	// 执行跟踪焊接
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
	//1. 获取相机参数
	T_CAMREA_PARAM tCameraParam = m_ptUnit->GetCameraParam(m_ptUnit->m_nTrackCameraNo);
	CDHGigeImageCapture* pDHCamera = (CDHGigeImageCapture*)m_ptUnit->GetCameraCtrl(m_ptUnit->m_nTrackCameraNo);
	//IplImage* pBufferImg = pDHCamera->m_pImageBuff;
	IplImage* pBufferImg = cvCreateImage(cvSize(2448, 2048), IPL_DEPTH_8U, 1);
	IplImage* pColorImg = cvCreateImage(cvSize(pBufferImg->width, pBufferImg->height), IPL_DEPTH_8U, 3);
	CString sIniFile;
	sIniFile.Format("%s%s\\TrackDipParam_TrackCam.ini", DATA_PATH, m_ptUnit->m_tContralUnit.strUnitName, m_ptUnit->m_nTrackCameraNo);
	GROUP_STAND_DIP_NS::CImageProcess* pImgProcess = new GROUP_STAND_DIP_NS::CImageProcess(pBufferImg->width, pBufferImg->height, m_pRobotDriver->m_nRobotNo, sIniFile);

	//2. 外部轴运动到测量位置
	int nAxisNo = m_ptUnit->m_nMeasureAxisNo;
	if (0 != m_ptUnit->MoveExAxisFun(dExAxlePos, m_pRobotDriver->m_tPulseHighSpeed.dSpeed * 2, nAxisNo))
	{
		cvReleaseImage(&pColorImg);
		delete pImgProcess;
		return false;
	}
	m_ptUnit->WorldCheckRobotDone();
	double dCurExPos = 0.0;
	// 需要更改获取轴数据，临时更改
	dCurExPos = m_ptUnit->GetExPositionDis(nAxisNo);//m_pRobotDriver->GetCurrentPos(ROBOT_AXIS_BPY);
	if (fabs(dExAxlePos - dCurExPos) > 5.0) // 与目标位置相差超过阈值 判断为运动失败
	{
		XiMessageBox("自动示教:外部轴未运动到指定位置");
		m_ptUnit->SwitchDHCamera(m_ptUnit->m_nTrackCameraNo, false);
		delete pImgProcess;
		return false;
	}
	
	//3. 发送测量位置（自动测量板厚是5个测量点  手动输入的板厚是两个测量点）
	vector<MP_USR_VAR_INFO> vtVarInfo;
	int nPos = 40;
	for (auto& it : vtMeasurePulse) {
		vtVarInfo.push_back(m_pRobotDriver->PrepareValData(nPos, it));
		nPos++;
	}
	m_pRobotDriver->SetMultiVar_H(vtVarInfo);

	////4. 初始化相机
	//E_DHGIGE_ACQUISITION_MODE eCameraMode = E_ACQUISITION_MODE_CONTINUE;
	//E_DHGIGE_CALL_BACK eCallBack = E_CALL_BACK_MODE_WAIT_IMAGE;
	//m_ptUnit->SwitchDHCamera(m_ptUnit->m_nTrackCameraNo, true, true, eCameraMode, eCallBack); // 下枪过程中 开相机 激光
	//m_ptUnit->m_vpImageCapture[m_ptUnit->m_nTrackCameraNo]->StartAcquisition();

	//5. 开始测量运动 单板单筋独有测量job 
	int nProIndex = 0;  //测量图片的数量
	//是否自动测量调用区分单独调用1 2job 

	if (!bIsMeasureLastPoint) {
		int nIndex = 0;
		if (!m_bAutoMeasureThick) {
			CString cstr;
			cstr.Format("%d", nIndex + 1);
			nIndex++;
			m_pRobotDriver->MoveByJob(vtMeasurePulse[0], m_pRobotDriver->m_tPulseLowSpeed, m_pRobotDriver->m_nExternalAxleType, cstr.GetBuffer());
			m_ptUnit->CheckRobotDone(vtMeasurePulse.back());
			//获取图像
			pDHCamera->CaptureImage(pBufferImg, 1);
			// 存原图
			CString sSavePath;
			sSavePath.Format("%sPicture\\Org\\Src_%d.jpg", m_sDataSavePath.GetBuffer(), nProIndex);
			SaveImage(pBufferImg, sSavePath);

			// 转颜色空间(转RGB彩色图)
			cvCvtColor(pBufferImg, pColorImg, CV_GRAY2RGB);

			// 图像处理 根据不同处理类型 调用图像处理得到二维点
			T_TEACH_RESULT tTeachReault;
			T_ANGLE_PULSE tAnglePulse = m_pRobotDriver->GetCurrentPulse();
			if (false == ProcessTeachImage(nProIndex, pImgProcess, pBufferImg, tAnglePulse, E_DOUBLE_LONG_LINE, tTeachReault, tCameraParam.eFlipMode))
			{
				//已修改
				XUI::MesBox::PopInfo("示教第{0}张图处理失败！n = {1} nProIdx = {2} {3} {4}", nProIndex, 0, nProIndex, vnMeasureType[1], m_vnMeasurePtnType[nProIndex]);
				//XiMessageBox("示教第%d张图处理失败！n = %d nProIdx = %d %d %d",
				//	nProIndex, 0, nProIndex, vnMeasureType[1], m_vnMeasurePtnType[nProIndex]);
				m_ptUnit->SwitchDHCamera(m_ptUnit->m_nTrackCameraNo, false);
				return false;
			}
			//保存处理结果
			m_vtTeachResult.push_back(tTeachReault);
			if (nIndex == 1) {
				m_tLastTeachResult = tTeachReault;
			}
			// 画出二维点
			for (int i = 0; i < tTeachReault.vtLeftPtns2D.size(); i++)
			{
				cvCircle(pColorImg, tTeachReault.vtLeftPtns2D[i], 6, CV_RGB(0, 255, 0), 2); // 左线
			}
			for (int i = 0; i < tTeachReault.vtRightPtns2D.size(); i++)
			{
				cvCircle(pColorImg, tTeachReault.vtRightPtns2D[i], 6, CV_RGB(0, 0, 255), 2); // 右线
			}
			cvCircle(pColorImg, tTeachReault.tKeyPtn2D, 6, CV_RGB(255, 0, 0), 2); // 交点

			// 存处理图
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
				//获取图像
				pDHCamera->CaptureImage(pBufferImg, 1);
				// 存原图
				CString sSavePath;
				sSavePath.Format("%sPicture\\Org\\Src_%d.jpg", m_sDataSavePath.GetBuffer(), nProIndex);
				SaveImage(pBufferImg, sSavePath);

				// 转颜色空间(转RGB彩色图)
				cvCvtColor(pBufferImg, pColorImg, CV_GRAY2RGB);

				// 图像处理 根据不同处理类型 调用图像处理得到二维点
				T_TEACH_RESULT tTeachReault;
				T_ANGLE_PULSE tAnglePulse = m_pRobotDriver->GetCurrentPulse();
				if (false == ProcessTeachImage(nProIndex, pImgProcess, pBufferImg, tAnglePulse, E_DOUBLE_LONG_LINE, tTeachReault, tCameraParam.eFlipMode))
				{
					//已修改
					XUI::MesBox::PopInfo("示教第{0}张图处理失败！n = {1} nProIdx = {2} {3} {4}", nProIndex, 0, nProIndex, vnMeasureType[1], m_vnMeasurePtnType[nProIndex]);
					//XiMessageBox("示教第%d张图处理失败！n = %d nProIdx = %d %d %d",
					//	nProIndex, 0, nProIndex, vnMeasureType[1], m_vnMeasurePtnType[nProIndex]);
					m_ptUnit->SwitchDHCamera(m_ptUnit->m_nTrackCameraNo, false);
					return false;
				}
				//保存处理结果
				m_vtTeachResult.push_back(tTeachReault);
				if (i == 1) {
					m_tLastTeachResult = tTeachReault;
				}
				// 画出二维点
				for (int i = 0; i < tTeachReault.vtLeftPtns2D.size(); i++)
				{
					cvCircle(pColorImg, tTeachReault.vtLeftPtns2D[i], 6, CV_RGB(0, 255, 0), 2); // 左线
				}
				for (int i = 0; i < tTeachReault.vtRightPtns2D.size(); i++)
				{
					cvCircle(pColorImg, tTeachReault.vtRightPtns2D[i], 6, CV_RGB(0, 0, 255), 2); // 右线
				}
				cvCircle(pColorImg, tTeachReault.tKeyPtn2D, 6, CV_RGB(255, 0, 0), 2); // 交点

																					  // 存处理图
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
				XiMessageBox("机器人未运行到位");
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

			// 存原图
			CString sSavePath;
			sSavePath.Format("%sPicture\\Org\\Src_%d.jpg", m_sDataSavePath.GetBuffer(), nProIndex);
			SaveImage(pBufferImg, sSavePath);

			// 转颜色空间(转RGB彩色图)
			cvCvtColor(pBufferImg, pColorImg, CV_GRAY2RGB);

			// 图像处理 根据不同处理类型 调用图像处理得到二维点
			T_TEACH_RESULT tTeachReault;
			if (false == ProcessTeachImage(nProIndex, pImgProcess, pBufferImg, vtMeasurePulse[n], vnMeasureType[n], tTeachReault, tCameraParam.eFlipMode))
			{
				//已修改
				XUI::MesBox::PopInfo("示教第{0}张图处理失败！n = {1} nProIdx = {2} {3} {4}", nProIndex, 0, nProIndex, vnMeasureType[1], m_vnMeasurePtnType[nProIndex]);
				//XiMessageBox("示教第%d张图处理失败！n = %d nProIdx = %d %d %d",
				//	nProIndex, n, nProIndex, vnMeasureType[n], m_vnMeasurePtnType[nProIndex]);
				m_ptUnit->SwitchDHCamera(m_ptUnit->m_nTrackCameraNo, false);
				break;
			}
			m_vtTeachResult.push_back(tTeachReault);

			// 画出二维点
			for (int i = 0; i < tTeachReault.vtLeftPtns2D.size(); i++)
			{
				cvCircle(pColorImg, tTeachReault.vtLeftPtns2D[i], 6, CV_RGB(0, 255, 0), 2); // 左线
			}
			for (int i = 0; i < tTeachReault.vtRightPtns2D.size(); i++)
			{
				cvCircle(pColorImg, tTeachReault.vtRightPtns2D[i], 6, CV_RGB(0, 0, 255), 2); // 右线
			}
			cvCircle(pColorImg, tTeachReault.tKeyPtn2D, 6, CV_RGB(255, 0, 0), 2); // 交点

																				  // 存处理图
			sSavePath.Format("%sPicture\\Pro\\Pro_%d.jpg", m_sDataSavePath.GetBuffer(), nProIndex);
			SaveImage(pColorImg, sSavePath);
			cvCopyImage(pColorImg, *m_pColorImg);

			nProIndex++;
			//// 判断是否处理完所有示教图片
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
	double dSearchDis = m_dEndpointSearchDis; // 自由端搜索长度距离
	vtTeachData.clear();
	XiAlgorithm alg;
	vector<T_TEACH_DATA> vtTempTeachData;
	LineOrCircularArcWeldingLine tSeam = vtSeamGroup[nSeamNo];
	bool bIsFlatWeld = !IsStandWeldSeam(tSeam.StartNormalVector);  //true为平焊 false为立焊
	bool bFreeStartPoint = !tSeam.StartPointType;
	bool bFreeEndPoint = !tSeam.EndPointType;
	double dNorAngleS = alg.CalcArcAngle(tSeam.StartNormalVector.x, tSeam.StartNormalVector.y);
	double dNorAngleE = alg.CalcArcAngle(tSeam.EndNormalVector.x, tSeam.EndNormalVector.y);
	
	// ************此处需要根据 起终点是否进行包角及包角类型 区分不同的测量轨迹计算函数

	// 计算测厚测量点
	CHECK_BOOL_RETURN(CalcThickMeasureData(m_bAutoMeasureThick, nGroupNo, nSeamNo, tSeam, dExAxlePos, vtTempTeachData));
	AppendTeachData(vtTeachData, vtTempTeachData);
	
	// 计算正面测量点
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
	// 注意：测量顺序为俯视沿着焊缝逆时针方向；
	//	正座：机器人坐标系中逆时针 Rz减小
	//  倒挂：机器人坐标系中顺时针 Rz增加
	vtSortIdx.clear();
	int nCoordNum = vtMeasureCoord.size();
	int nTypeNum = vnMeasureType.size();
	if (nCoordNum != nTypeNum || nCoordNum < 1)
	{
		XiMessageBox("测量点数量和测量类型数量不匹配，排序失败");
		return false;
	}
	// 区分搜索测量点和示教测量点
	double dRzChangeDir = 1 == m_nRobotInstallDir ? 1.0 : -1.0;

	// 将坐标按照法相分为四组
	int nGroupNum = 4;
	vector<vector<TempStruct>> vvtCoord(nGroupNum, vector<TempStruct>(0));
	vector<TempStruct> vtTempStruct(0);
	dExAxlePos += dExAxleOffsetDis; // 使用外部轴偏移 修改外部轴坐标
	for (int nIdx = 0; nIdx < vtMeasureCoord.size(); nIdx++)
	{
#ifdef SINGLE_ROBOT
		vtMeasureCoord[nIdx].dY -= dExAxleOffsetDis; // 使用外部轴偏移 修改机器人坐标
#else
		vtMeasureCoord[nIdx].dX -= dExAxleOffsetDis; // 使用外部轴偏移 修改机器人坐标
#endif // SINGLE_ROBOT

		// 焊枪工具 转 跟踪相机工具
		if (false == m_pRobotDriver->MoveToolByWeldGun(vtMeasureCoord[nIdx], m_pRobotDriver->m_tTools.tGunTool,
			vtMeasureCoord[nIdx], m_pRobotDriver->m_tTools.tCameraTool, vtMeasureCoord[nIdx]))
		{
			m_pRobotDriver->m_cLog->Write("Robot %d 第%d个测量点 转换失败！", m_pRobotDriver->m_nRobotNo, nIdx);
			return false;
		}

		double dDirAngle = m_pRobotDriver->RzToDirAngle(vtMeasureCoord[nIdx].dRZ);
		for (int nAngleInx = 0; nAngleInx < nGroupNum; nAngleInx++) // 按测量方向角(Rz) 分组
		{
			double dBaseDirAngle = dFirstSeamNorAngle - (nAngleInx * 90.0 * m_pRobotDriver->m_nRobotInstallDir); // 正座测量 板按法相减小排序
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
	// 组间排序：以上按方向分组结果的排序，各组90°过渡连续运行，删除空组
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
		if (-1 == nFirstNullNo) // 无空组
		{
			break;
		}
		else if (0 == nFirstNullNo) // 第一个是空组
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

	// 外圈焊缝重新排序
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
			XiMessageBoxOk("坐标转换失败bOutsideWeld");
			return false;
		}
		double dDirAngle2 = alg.CalcArcAngle(tCoord2.dX - tCoord1.dX, tCoord2.dY - tCoord1.dY);
		if (!JudgeDirAngle(dDirAngle1, dDirAngle2, 95.0)) // 外圈焊缝 各角度测量组颠倒
		{
			bOutsideWeld = true;
			for (int i = 0; i < vvtCoord.size() / 2; i++)
			{
				swap(vvtCoord[i], vvtCoord[vvtCoord.size() - 1 - i]);
			}
		}
	}

	// 每组路径排序
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
			tPtn.x = vtCoord[0].tCoord.dX + 999999.0 * CosD(dMoveDirAngle); // 生成排序辅助点
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
					if (dCurLevelDis < dMinLevelDis && dCurLevelDis < 20.0) // 变姿态，导致没有正上方坐标
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

	// 相邻测量位置姿态变化过大添加过渡点
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
			(false == JudgeAngle(180.0, -180.0, tPreCoord.dRZ, tCurCoord.dRZ, 45.0))) // 外圈焊缝 Rz变化超过45°
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
		else if (false == JudgeAngle(180.0, -180.0, tPreCoord.dRZ, tCurCoord.dRZ, 100.0)) // 相邻测量点Rz相差超过100°
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
	// 1、为第一个非搜索点添加一个下枪过渡点到第一个坐标前 
	// 2、为最后一个非搜索点添加一个收枪过渡点到最后一个坐标后
	// 3、为每个搜索点后面添加一个过渡点

	int nFirstIdx = 9999;
	int nLastIdx = -1;
	vector<int> vnSearchPtnIdx(0);
	int nSrcPtnNum = vtMeasureCoord.size();
	// 查找 第一个非搜索点 索引 最后一个非搜索点索引 和每个搜索点索引
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
	// 添加示教收枪过渡点
	if ((nLastIdx >= 0) && (nLastIdx < nSrcPtnNum))
	{
		tMeasureCoord = vtMeasureCoord[nLastIdx];
		dOffsetDirAngle = m_pRobotDriver->RzToDirAngle(tMeasureCoord.dRZ);
		tMeasureCoord.dX += (dSafeDis / 2.0 * CosD(dOffsetDirAngle));
		tMeasureCoord.dY += (dSafeDis / 2.0 * SinD(dOffsetDirAngle));
		tMeasureCoord.dZ = dMaxPartHeight + (dSafeDis * (double)m_nRobotInstallDir);
		//tMeasureCoord.dRX = m_dPlatWeldRx;
		//tMeasureCoord.dRY = m_dPlatWeldRy;
		vtMeasureCoord.push_back(tMeasureCoord); // 收枪
		vnMeasureType.push_back(E_TRANSITION_POINT);
	}

	// 添加搜索点收下枪过渡点
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

	// 添加示教下枪过渡点
	if ((nFirstIdx >= 0) && (nFirstIdx < nSrcPtnNum))
	{
		tMeasureCoord = tFirstCoord;
		dOffsetDirAngle = m_pRobotDriver->RzToDirAngle(tMeasureCoord.dRZ);
		tMeasureCoord.dX += (dSafeDis / 2.0 * CosD(dOffsetDirAngle));
		tMeasureCoord.dY += (dSafeDis / 2.0 * SinD(dOffsetDirAngle));
		tMeasureCoord.dZ = dMaxPartHeight + (dSafeDis * (double)m_nRobotInstallDir);
		//tMeasureCoord.dRX = m_dPlatWeldRx;
		//tMeasureCoord.dRY = m_dPlatWeldRy;
		vtMeasureCoord.insert(vtMeasureCoord.begin(), tMeasureCoord); // 下枪
		vnMeasureType.insert(vnMeasureType.begin(), E_TRANSITION_POINT);
	}
	return true;
}
*/