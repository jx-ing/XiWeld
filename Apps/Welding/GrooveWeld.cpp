#include "stdafx.h"
#include "GrooveWeld.h"

GrooveWeld::GrooveWeld(CUnit* ptUnit, E_WORKPIECE_TYPE eWorkPieceType) :
	WeldAfterMeasure(ptUnit, eWorkPieceType)
{
}

GrooveWeld::~GrooveWeld()
{

}


void GrooveWeld::SetRecoParam()
{
	COPini opini;
	opini.SetFileName(DATA_PATH + m_ptUnit->m_tContralUnit.strUnitName + SYETEM_PARAM_FILE);
	switch (m_eWorkPieceType)
	{
	case E_BEVEL:
		opini.SetSectionName("E_BEVEL");
		opini.ReadString("ParamValue0", &m_dZPallet);
		opini.ReadString("ParamValue1", &m_dSideBoardThick);
		opini.ReadString("ParamValue2", &m_dScanEndpointOffset);
		break;
	default: XUI::MesBox::PopOkCancel("GrooveWeld不支持工件类型{0}!", (int)m_eWorkPieceType); break;
	}
}

bool GrooveWeld::PointCloudProcess(bool bUseModel, CvPoint3D64f* pPointCloud/* = NULL*/, int PointCloudSize/* = 0*/)
{
	if (bUseModel)
		return PointCloudProcessWithModel();

	m_vtWeldSeamData.clear();
	int nWeldingSeamNumber = 0;
	bool bInstallDir = 1 == m_nRobotInstallDir ? true : false;
	vector<CvPoint3D64f> vtPointCloud;
	LineOrCircularArcWeldingLine* pLineOrCircularArcWeldingLine; // 李伟
	try
	{
		if (NULL == pPointCloud)
		{
			LoadContourData(m_pRobotDriver, m_sPointCloudFileName, vtPointCloud);
			pPointCloud = (CvPoint3D64f*)vtPointCloud.data();
			PointCloudSize = vtPointCloud.size();
		}
		//SaveContourData();	//将所有点云数据保存到一个文件中
		if (E_BEVEL == m_eWorkPieceType)
		{
			pLineOrCircularArcWeldingLine = GroovePointCloudToWeldingLines(
				(Three_DPoint*)pPointCloud, PointCloudSize, &nWeldingSeamNumber, bInstallDir, m_dZPallet, m_dSideBoardThick);
			SavePointCloudProcessResult((LineOrCircularArcWeldingLine*)pLineOrCircularArcWeldingLine, nWeldingSeamNumber);
			ReleaseWeldingLines(&pLineOrCircularArcWeldingLine);
		}
		else
		{
			XiMessageBox("坡口焊接类：类型错误 处理失败！");
			return false;
		}
	}
	catch (...)
	{
		XiMessageBox("加筋板处理异常");
		return false;
	}

	//已修改
	XUI::MesBox::PopInfo("焊道数量为:{0}", nWeldingSeamNumber);
	//XiMessageBox("焊道数量为:%d", nWeldingSeamNumber);
	if (nWeldingSeamNumber <= 0)
	{
		return false;
	}
	return true;
}

bool GrooveWeld::WeldSeamGrouping(int& nWeldGroupNum)
{
	// 自动分组
	if (g_bAutoGroupingMark)
	{
		bool bRst = WeldSeamGroupingAuto(nWeldGroupNum);
		return bRst;
	}
	nWeldGroupNum = 0;
	m_vvtWeldSeamGroup.clear();
	std::vector<WeldLineInfo> vtWeldLineInfo(0);
	for (int nWeldNo = 0; nWeldNo < m_vtWeldSeamInfo.size(); nWeldNo++)
	{
		vtWeldLineInfo.clear();
		WeldLineInfo& tLineSeamInfo = m_vtWeldSeamInfo[nWeldNo];
		vtWeldLineInfo.resize(1, tLineSeamInfo);
		m_vvtWeldLineInfoGroup.push_back(vtWeldLineInfo);
	}

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

bool GrooveWeld::CalcMeasureTrack(int nGroupNo, std::vector<T_ROBOT_COORS>& vtMeasureCoord, std::vector<T_ANGLE_PULSE>& vtMeasurePulse, vector<int>& vnMeasureType, double& dExAxlePos, double& dSafeHeight)
{
	//return CalcMeasureTrack(nGroupNo, vtMeasureCoord, vtMeasurePulse, vnMeasureType, dSafeHeight);
	return CalcMeasureTrack_G(nGroupNo, vtMeasureCoord, vtMeasurePulse, vnMeasureType, dSafeHeight);

//	T_ROBOT_COORS tThersholdCoord(10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 0.0, 0.0, 0.0); // 相同测量点判断阈值
//	vtMeasureCoord.clear(); // 转跟踪相机工具后
//	vtMeasurePulse.clear();
//	vnMeasureType.clear();
//	m_vtTeachData.clear();
//	m_vnTeachTrackOrder.clear();
//	m_vtTeachResult.clear();
//	T_TEACH_DATA tTeachData;
//	vector<T_TEACH_DATA> vtTeachData;
//	vector<LineOrCircularArcWeldingLine> vtSeamGroup = m_vvtWeldSeamGroup[nGroupNo];
//	
//	CvPoint3D64f tTotalValue = { 0 }; // 统计XYZ值 确定外部轴位置
//	double dPartMaxHeight = -99999.0; // 正座记录工件最高点Z值 倒挂记录工件最低点
//	for (int nWeldSeamNo = 0; nWeldSeamNo < vtSeamGroup.size(); nWeldSeamNo++)
//	{
//		tTotalValue.x += (vtSeamGroup[nWeldSeamNo].StartPoint.x);
//		tTotalValue.x += (vtSeamGroup[nWeldSeamNo].EndPoint.x);
//		tTotalValue.y += (vtSeamGroup[nWeldSeamNo].StartPoint.y);
//		tTotalValue.y += (vtSeamGroup[nWeldSeamNo].EndPoint.y);
//		tTotalValue.z += (vtSeamGroup[nWeldSeamNo].StartPoint.z);
//		tTotalValue.z += (vtSeamGroup[nWeldSeamNo].EndPoint.z);
//
//		dPartMaxHeight = dPartMaxHeight < vtSeamGroup[nWeldSeamNo].StartPoint.z * m_nRobotInstallDir ? vtSeamGroup[nWeldSeamNo].StartPoint.z * m_nRobotInstallDir : dPartMaxHeight;
//		dPartMaxHeight = dPartMaxHeight < vtSeamGroup[nWeldSeamNo].EndPoint.z * m_nRobotInstallDir ? vtSeamGroup[nWeldSeamNo].EndPoint.z * m_nRobotInstallDir : dPartMaxHeight;
//		//dPartMaxHeight = dPartMaxHeight < vtSeamGroup[nWeldSeamNo].ZSide * m_nRobotInstallDir ? vtSeamGroup[nWeldSeamNo].ZSide * m_nRobotInstallDir : dPartMaxHeight;
//	}
//	dPartMaxHeight *= m_nRobotInstallDir;
//#ifdef SINGLE_ROBOT
//	dExAxlePos = tTotalValue.y / (vtSeamGroup.size() * 2); // 外部轴位置:注意此外部轴位置可能被后续计算更新
//#else
//	dExAxlePos = tTotalValue.x / (vtSeamGroup.size() * 2); // 外部轴位置:注意此外部轴位置可能被后续计算更新
//#endif // SINGLE_ROBOT
//
//	std::vector<T_ROBOT_COORS> vtMeasureCoordGunTool(0); // 合并后的焊枪工具坐标
//	for (int nSeamNo = 0; nSeamNo < vtSeamGroup.size(); nSeamNo++)
//	{
//		CHECK_BOOL_RETURN(CalcGrooveScanTrack(nGroupNo, nSeamNo, dExAxlePos, vtSeamGroup, vtTeachData));
//		for (int nNo = 0; nNo < vtTeachData.size(); nNo++)
//		{
//			tTeachData = vtTeachData[nNo];
//			tTeachData.nMeasurePtnNo = vtMeasureCoord.size();
//			vtMeasureCoordGunTool.push_back(tTeachData.tMeasureCoordGunTool);
//			vtMeasureCoord.push_back(tTeachData.tMeasureCoordGunTool);
//			vnMeasureType.push_back(tTeachData.nMeasureType);
//			m_vtTeachData.push_back(tTeachData);
//		}
//	}
//
//	// 保存测量轨迹焊枪工具坐标
//	CString sFileName;
//	sFileName.Format("%s%d-MeasureTrackGunTool.txt", m_sDataSavePath, nGroupNo);
//	FILE* pf = fopen(sFileName.GetBuffer(), "w");
//	for (int i = 0; i < m_vtTeachData.size(); i++)
//	{
//#ifdef SINGLE_ROBOT
//		fprintf(pf, "%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%4d%4d " + GetTeachDataString(m_vtTeachData[i]) + "\n",
//			m_vtTeachData[i].tMeasureCoordGunTool.dX, m_vtTeachData[i].tMeasureCoordGunTool.dY + dExAxlePos, m_vtTeachData[i].tMeasureCoordGunTool.dZ,
//			m_vtTeachData[i].tMeasureCoordGunTool.dX, m_vtTeachData[i].tMeasureCoordGunTool.dY, m_vtTeachData[i].tMeasureCoordGunTool.dZ,
//			m_vtTeachData[i].tMeasureCoordGunTool.dRX, m_vtTeachData[i].tMeasureCoordGunTool.dRY, m_vtTeachData[i].tMeasureCoordGunTool.dRZ,
//			dExAxlePos, m_vtTeachData[i].nWeldNo, m_vtTeachData[i].nMeasurePtnNo);
//#else
//		fprintf(pf, "%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%4d%4d " + GetTeachDataString(m_vtTeachData[i]) + "\n",
//			m_vtTeachData[i].tMeasureCoordGunTool.dX + dExAxlePos, m_vtTeachData[i].tMeasureCoordGunTool.dY, m_vtTeachData[i].tMeasureCoordGunTool.dZ,
//			m_vtTeachData[i].tMeasureCoordGunTool.dX, m_vtTeachData[i].tMeasureCoordGunTool.dY, m_vtTeachData[i].tMeasureCoordGunTool.dZ,
//			m_vtTeachData[i].tMeasureCoordGunTool.dRX, m_vtTeachData[i].tMeasureCoordGunTool.dRY, m_vtTeachData[i].tMeasureCoordGunTool.dRZ,
//			dExAxlePos, m_vtTeachData[i].nWeldNo, m_vtTeachData[i].nMeasurePtnNo);
//#endif // SINGLE_ROBOT
//	}
//	fclose(pf);
//
//	// 外部轴位置偏移 : 0.0  ±100.0  ±200.0  ±300.0 ……
//	vector<double> vdExAxleOffset;
//	vdExAxleOffset.clear();
//	vdExAxleOffset.push_back(0.0);
//	for (int i = -1000; i < 1001; i += 100)
//	{
//		vdExAxleOffset.push_back((double)i);
//	}
//
//	bool bCalcSuccess = false;
//	for (int nOffsetIdx = 0; (false == bCalcSuccess) && (nOffsetIdx < vdExAxleOffset.size()); nOffsetIdx++)
//	{
//		double dTempExAxlePos = dExAxlePos;
//		std::vector<T_ROBOT_COORS> vtTempCoord(vtMeasureCoord);
//		std::vector<T_ANGLE_PULSE> vtTempPulse(vtMeasurePulse);
//		vector<int> vnTempType(vnMeasureType);
//
//		double dMaxExAxlePos = (double)m_ptUnit->GetRobotCtrl()->m_tExternalAxle[m_ptUnit->m_nMeasureAxisNo - 1].lMaxPulseNum * m_ptUnit->GetRobotCtrl()->m_tExternalAxle[m_ptUnit->m_nMeasureAxisNo - 1].dPulse;
//		double dMinExAxlePos = (double)m_ptUnit->GetRobotCtrl()->m_tExternalAxle[m_ptUnit->m_nMeasureAxisNo - 1].lMinPulseNum * m_ptUnit->GetRobotCtrl()->m_tExternalAxle[m_ptUnit->m_nMeasureAxisNo - 1].dPulse;
//		if (vdExAxleOffset[nOffsetIdx] + dTempExAxlePos > dMaxExAxlePos || vdExAxleOffset[nOffsetIdx] + dTempExAxlePos < dMinExAxlePos)
//		{
//			WriteLog("外部轴%d 目标位置%.3lf 超出设定极限%.3lf - %.3lf!", m_ptUnit->m_nMeasureAxisNo, vdExAxleOffset[nOffsetIdx], dMinExAxlePos, dMaxExAxlePos);
//			continue;
//		}
//
//		sFileName.Format("%s%d_Offset%.0lf_SortMeasureCoordBefore.txt", m_sDataSavePath, nGroupNo, vdExAxleOffset[nOffsetIdx]);
//		if (false == SortMeasureCoordNew(dPartMaxHeight, 0.0,
//			vdExAxleOffset[nOffsetIdx], dTempExAxlePos, vtTempCoord, vnTempType, m_vnTeachTrackOrder))
//		{
//			WriteLog("ExAxleOffset:%3lf SortMeasureCoord Fail!", vdExAxleOffset[nOffsetIdx]);
//			continue;
//		}
//		sFileName.Format("%s%d_Offset%.0lf_SortMeasureCoord.txt", m_sDataSavePath, nGroupNo, vdExAxleOffset[nOffsetIdx]);
//		// 添加收枪 下枪坐标
//		dSafeHeight = dPartMaxHeight + (m_dGunDownBackSafeDis * (double)m_nRobotInstallDir);
//		if (false == AddSafeCoord(vtTempCoord, vnTempType, dPartMaxHeight, m_dGunDownBackSafeDis))
//		{
//			WriteLog("ExAxleOffset:%3lf AddSafeCoord Fail!", vdExAxleOffset[nOffsetIdx]);
//			continue;
//		}
//		sFileName.Format("%s%d_Offset%.0lf_AddSafeCoord.txt", m_sDataSavePath, nGroupNo, vdExAxleOffset[nOffsetIdx]);
//		//SaveCoordToFile(vtTempCoord, sFileName);
//		// vtTempCoord -> vtTempPulse
//		if (false == CalcContinuePulseForWeld(vtTempCoord, vtTempPulse, false))
//		{
//			WriteLog("ExAxleOffset:%3lf CalcContinuePulseForWeld Fail!", vdExAxleOffset[nOffsetIdx]);
//			continue;
//		}
//
//		bCalcSuccess = true;
//		dExAxlePos = dTempExAxlePos;
//		vtMeasureCoord.clear();
//		vtMeasurePulse.clear();
//		vnMeasureType.clear();
//		vtMeasureCoord.assign(vtTempCoord.begin(), vtTempCoord.end());
//		vtMeasurePulse.assign(vtTempPulse.begin(), vtTempPulse.end());
//		vnMeasureType.assign(vnTempType.begin(), vnTempType.end());
//
//		// 计算成功，检查当前位置和测量轨迹第一个坐标R轴差值，超过阈值再添加一个过渡点
//		if (false == m_bIsLocalDebug)
//		{
//			T_ANGLE_PULSE tCurAnglePulse = m_pRobotDriver->GetCurrentPulse();
//			double dAngleErrAxisR = (double)(vtMeasurePulse[0].nRPulse - tCurAnglePulse.nRPulse) * m_pRobotDriver->m_tAxisUnit.dRPulse;
//			if (fabs(dAngleErrAxisR) > 90.0)
//			{
//				T_ROBOT_COORS tSafeDownCoord;
//				T_ANGLE_PULSE tSafeDownGunPulse(
//					tCurAnglePulse.nSPulse, //(vtMeasurePulse[0].nSPulse - tCurAnglePulse.nSPulse) / 2
//					m_pRobotDriver->m_tHomePulse.nLPulse,
//					m_pRobotDriver->m_tHomePulse.nUPulse,
//					tCurAnglePulse.nRPulse,
//					0,
//					(vtMeasurePulse[0].nTPulse - tCurAnglePulse.nTPulse) / 2, 0, 0, 0);
//				m_pRobotDriver->RobotKinematics(tSafeDownGunPulse, m_pRobotDriver->m_tTools.tGunTool, tSafeDownCoord);
//				vtMeasurePulse.insert(vtMeasurePulse.begin(), tSafeDownGunPulse);
//				vtMeasureCoord.insert(vtMeasureCoord.begin(), tSafeDownCoord);
//				vnMeasureType.insert(vnMeasureType.begin(), E_TRANSITION_POINT);
//			}
//		}
//		for (int i = 0; i < vtMeasureCoord.size(); i++) // 将外部轴坐标和机器人坐标 结合
//		{
//#ifdef SINGLE_ROBOT
//			vtMeasureCoord[i].dBY += dExAxlePos;
//			vtMeasurePulse[i].lBYPulse = dExAxlePos / m_pRobotDriver->m_tExternalAxle[1].dPulse;
//#else
//			vtMeasureCoord[i].dBX = dExAxlePos;
//			vtMeasurePulse[i].lBXPulse = dExAxlePos / m_pRobotDriver->m_tExternalAxle[0].dPulse;
//#endif // SINGLE_ROBOT
//		}
//		WriteLog("ExAxleOffset:%3lf 计算连续测量运动轨迹成功", vdExAxleOffset[nOffsetIdx]);
//	}
//	if (false == bCalcSuccess)
//	{
//		XiMessageBox("多位置计算连续测量轨迹失败！");
//		return false;
//	}
//
//	// 保存实际测量运行轨迹 关节坐标 直角坐标 外部轴 
//	sFileName.Format("%s%d-MeasureTrackRealCoordPulseEx.txt", m_sDataSavePath, nGroupNo);
//	pf = fopen(sFileName.GetBuffer(), "w");
//	for (int i = 0; i < vtMeasureCoord.size(); i++)
//	{
//#ifdef SINGLE_ROBOT
//		fprintf(pf, "%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%10d%10d%10d%10d%10d%10d%11.3lf\n",
//			vtMeasureCoord[i].dX, vtMeasureCoord[i].dY + dExAxlePos, vtMeasureCoord[i].dZ,
//			vtMeasureCoord[i].dRX, vtMeasureCoord[i].dRY, vtMeasureCoord[i].dRZ,
//			vtMeasurePulse[i].nSPulse, vtMeasurePulse[i].nLPulse, vtMeasurePulse[i].nUPulse,
//			vtMeasurePulse[i].nRPulse, vtMeasurePulse[i].nBPulse, vtMeasurePulse[i].nTPulse, dExAxlePos);
//#else
//		fprintf(pf, "%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%10d%10d%10d%10d%10d%10d%11.3lf\n",
//			vtMeasureCoord[i].dX + dExAxlePos, vtMeasureCoord[i].dY, vtMeasureCoord[i].dZ,
//			vtMeasureCoord[i].dRX, vtMeasureCoord[i].dRY, vtMeasureCoord[i].dRZ,
//			vtMeasurePulse[i].nSPulse, vtMeasurePulse[i].nLPulse, vtMeasurePulse[i].nUPulse,
//			vtMeasurePulse[i].nRPulse, vtMeasurePulse[i].nBPulse, vtMeasurePulse[i].nTPulse, dExAxlePos);
//#endif // SINGLE_ROBOT
//	}
//	fclose(pf);
//
//	// 离线调试使用
//	CString sJobName;
//	sJobName.Format("MOVJTEACHBOARD%d", nGroupNo);
//	GenerateJobLocalVariable(vtMeasurePulse, MOVJ, sJobName);
//	GenerateFilePLY(nGroupNo, dExAxlePos); // 调试使用
//
//	//if (m_bIsLocalDebug)
//	//{
//	//	m_pScanInit->InitDynamicCapture_H_M(E_POINT_CLOUD_PROC_MOMENT_FINAL, E_IMAGE_PROC_METHOD_EEEEE, E_POINT_CLOUD_PROC_METHOD_Groove,
//	//		m_ptUnit->m_nMeasureCameraNo, OUTPUT_PATH + m_pRobotDriver->m_strRobotName + GROOVE_DATA_PATH, 80);
//	//	m_pScanInit->FinalProc_Groove(); // 调试使用
//	//	GetScanProcResult(vtMeasureCoord, m_vtTeachResult);
//	//	SaveTeachResult(nGroupNo);
//	//}
//	return true;
}

bool GrooveWeld::DoTeach(int nGroupNo, const std::vector<T_ANGLE_PULSE>& vtMeasurePulse, const vector<int>& vnMeasureType, double dExAxlePos111, int nLayerNo/* = 0*/)
{
	DelFiles(OUTPUT_PATH + m_pRobotDriver->m_strRobotName + "\\Groove");
	DelFiles(OUTPUT_PATH + m_pRobotDriver->m_strRobotName + "\\Track");
	// 数据检查
	if (vtMeasurePulse.size() < 3 || vtMeasurePulse.size() != vnMeasureType.size())
	{
		XiMessageBox("示教数据错误！");
		return false;
	}
	double y;
	// 数据记录到成员变量（供线程函数使用）
	SetTeachData(vtMeasurePulse, vnMeasureType, 0.0, y);

	vector<T_TEACH_RESULT> vtTeachResult(0);
	if (vtMeasurePulse.size() > 0)
	{
		CHECK_BOOL_RETURN(ScanEndpoint_G(nGroupNo, nLayerNo, m_ptUnit->m_nMeasureCameraNo, vtMeasurePulse, vnMeasureType, vtTeachResult));
	}

	// 汇总结果
	m_vtTeachResult.clear();
	m_vtTeachResult.assign(vtTeachResult.begin(), vtTeachResult.end());

	// 保存示教结果(示教及搜索结果)
	SaveTeachResult_G(nGroupNo, nLayerNo);
	return true;
}

bool GrooveWeld::ScanEndpoint(int nGroupNo, int nCameraNo, vector<T_ANGLE_PULSE> vtPulse, vector<int> vnType, vector<T_TEACH_RESULT>& vtTeachResult)
{
	vtTeachResult.clear();
	m_pRobotDriver->m_vtWeldLineInWorldPoints.clear();
	E_SCANMODE eScanMode = E_SCANMODE_2DPOINT;
	T_ROBOT_COORS tCoord;
	vector<T_ROBOT_COORS> vtCoord(0);
	T_ROBOT_MOVE_INFO tRobotMoveInfo;
	vector<T_ROBOT_MOVE_INFO> vtRobotMoveInfo(0);
	for (int i = 0; i < vtPulse.size(); i++)
	{
		m_pRobotDriver->RobotKinematics(vtPulse[i], m_pRobotDriver->m_tTools.tGunTool, tCoord);
		vtCoord.push_back(tCoord);
		
		if (E_TRANSITION_POINT != vnType[i])
		{
			m_pRobotDriver->m_vtWeldLineInWorldPoints.push_back(tCoord);
		}
	}

	// 下枪运动到扫描起点位置 MOVJ MOVJ
	int i = 0;
	for (i = 0; i < vnType.size(); i++)
	{
		if (E_TRANSITION_POINT != vnType[i])
		{
			tRobotMoveInfo = m_pRobotDriver->PVarToRobotMoveInfo(0, vtPulse[i + 1], m_pRobotDriver->m_tPulseHighSpeed, MOVJ);
			vtRobotMoveInfo.push_back(tRobotMoveInfo);
			break;
		}
		tRobotMoveInfo = m_pRobotDriver->PVarToRobotMoveInfo(0, vtPulse[i], m_pRobotDriver->m_tPulseHighSpeed, MOVJ);
		vtRobotMoveInfo.push_back(tRobotMoveInfo);
	}
	vtRobotMoveInfo[vtRobotMoveInfo.size() - 1].tSpeed = m_pRobotDriver->m_tPulseLowSpeed;
	m_pRobotDriver->SetMoveValue(vtRobotMoveInfo);
	m_pRobotDriver->CallJob("CONTIMOVANY");

	E_DHGIGE_ACQUISITION_MODE eCameraMode = E_ACQUISITION_MODE_SOURCE_SOFTWARE;
	E_DHGIGE_CALL_BACK eCallBack = E_CALL_BACK_MODE_OFF;
	m_ptUnit->SwitchDHCamera(m_ptUnit->m_nMeasureCameraNo, true, true, eCameraMode, eCallBack); // 下枪时 开相机 激光
	m_ptUnit->m_vpImageCapture[m_ptUnit->m_nMeasureCameraNo]->StartAcquisition();
	m_ptUnit->RobotCheckDone();
	CHECK_PULSE_RETURN_BOOL(m_pRobotDriver, vtPulse[i]);

	E_FLIP_MODE eFlipMode = m_ptUnit->GetCameraParam(m_ptUnit->m_nMeasureCameraNo).eFlipMode;
	m_pScanInit->SetScanParam(TRUE == *m_pIsNaturalPop, eFlipMode, eScanMode, "CONTIMOVANY");

	if (!m_pScanInit->InitDynamicCapture_H_M(E_POINT_CLOUD_PROC_MOMENT_DYNAMIC, E_IMAGE_PROC_METHOD_EEEEE, E_POINT_CLOUD_PROC_METHOD_Groove,
		m_ptUnit->m_nMeasureCameraNo, OUTPUT_PATH + m_pRobotDriver->m_strRobotName + GROOVE_DATA_PATH, "AA_PointCloud.txt", 80, m_eWeldSeamType, m_nGroupNo, m_nLayerNo))
	{
		return false;
	}

	m_pScanInit->DynamicCaptureNew_H_M(m_pRobotDriver, m_ptUnit->m_nMeasureCameraNo, 4, 50);

	// 收枪运动 MOVL
	vtRobotMoveInfo.clear();
	tRobotMoveInfo = m_pRobotDriver->PVarToRobotMoveInfo(0, vtPulse.back(), m_pRobotDriver->m_tPulseLowSpeed, MOVJ);
	vtRobotMoveInfo.push_back(tRobotMoveInfo);	
	m_pRobotDriver->SetMoveValue(vtRobotMoveInfo, false);
	m_pRobotDriver->CallJob("CONTIMOVANY");

	m_ptUnit->SwitchDHCamera(m_ptUnit->m_nMeasureCameraNo, false); // 收枪时关闭相机和激光
	m_ptUnit->RobotCheckDone();
	CHECK_PULSE_RETURN_BOOL(m_pRobotDriver, vtPulse.back());
	CHECK_BOOL_RETURN(GetScanProcResult(vtCoord, vtTeachResult));
	return true;
}

bool GrooveWeld::CalcWeldTrack(int nGroupNo)
{
	// 第一次测量删除所有第 nGroupNo 组焊缝轨迹文件 
	if (0 == m_nLayerNo)
	{
		int nMaxWeldNo = 20;
		int nMaxLayerNo = 20;
		CString sFileName;
		for (int nDelWeldNo = 0; nDelWeldNo < nMaxWeldNo; nDelWeldNo++)
		{
			sFileName.Format("%d_%d_RealWeldCoord.txt", nGroupNo, nDelWeldNo);
			DelFiles(m_sDataSavePath, sFileName);
			sFileName.Format("%d_%d_RealWeldCoord_SrcBackUp.txt", nGroupNo, nDelWeldNo);
			DelFiles(m_sDataSavePath, sFileName);
			for (int nLayerNo = 0; nLayerNo < nMaxLayerNo; nLayerNo++)
			{
				sFileName.Format("%d_%d_%d_AdjustRealWeldCoord.txt", nGroupNo, nDelWeldNo, nLayerNo);
				DelFiles(m_sDataSavePath, sFileName);
				sFileName.Format("%d_%d_%d_AdjustRealWeldCoordWorld.txt", nGroupNo, nDelWeldNo, nLayerNo);
				DelFiles(m_sDataSavePath, sFileName);
			}
		}
	}

	// 检查示教结果数量
	if ((1 != m_vtTeachResult.size()) ||
		(0 != (m_vtTeachResult[0].vtLeftPtns3D.size() % 5)) ||
		(10 > m_vtTeachResult[0].vtLeftPtns3D.size()))
	{
		XUI::MesBox::PopOkCancel("坡口测量结果异常 {0} {1}", m_vtTeachResult.size(), m_vtTeachResult[0].vtLeftPtns3D.size());
		return false;
	}

	// 生成焊接轨迹
	if (0 == m_nLayerNo)
	{
		CHECK_BOOL_RETURN(GeneralGrooveWeldTrack_G_First(nGroupNo)); // 首次测量生成所有焊缝
		//CHECK_BOOL_RETURN(GeneralGrooveWeldTrack_G(nGroupNo)); // 纯直线焊接 弃用
	}
	else
	{
		CHECK_BOOL_RETURN(GeneralGrooveWeldTrack_G_Other(nGroupNo, m_nLayerNo)); // 非首次测量仅修正当前焊缝
	}
	
	return true;
}

bool GrooveWeld::LoadRealWeldTrack(int nGroupNo, int nWeldNo, E_WELD_SEAM_TYPE& eWeldSeamType, double& dExAxlePos, vector<T_ROBOT_COORS>& vtRealWeldTrack, vector<int>& vnPtnType)
{
	int nDataColNum = 13;
	m_vdGrooveWeldSpeedRate.clear();
	m_vdWaveWaitTime.clear();
	vtRealWeldTrack.clear();
	vnPtnType.clear();
	CString sFileName;
	sFileName.Format("%s%d_%d_RealWeldCoord.txt", m_sDataSavePath.GetBuffer(), nGroupNo, nWeldNo);
	FILE* pf = fopen(sFileName.GetBuffer(), "r");
	if (NULL == pf)
	{
		//XiMessageBox("加载第%d组 第%d个焊缝轨迹失败", nGroupNo, nWeldNo);
		return false;
	}
	// 序号 直角坐标 外部轴坐标 焊接类型
	int nIdx = 0;
	int nWeldSeamType = 0;
	T_ROBOT_COORS tCoord;
	double dWaveWaitTime = 0;
	double dSpeedRate = 0.0;
	int nRst = fscanf(pf, "%d%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%d%lf", &nIdx,
		&tCoord.dX, &tCoord.dY, &tCoord.dZ, &tCoord.dRX, &tCoord.dRY, &tCoord.dRZ,
		&tCoord.dBX, &tCoord.dBY, &tCoord.dBZ, &dSpeedRate, &nWeldSeamType, &dWaveWaitTime);
	while (nDataColNum == nRst)
	{
		m_vdGrooveWeldSpeedRate.push_back(dSpeedRate);
		m_vdWaveWaitTime.push_back(dWaveWaitTime);
		vtRealWeldTrack.push_back(tCoord);
		vnPtnType.push_back(E_WELD_TRACK);
		nRst = fscanf(pf, "%d%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%d%lf", &nIdx,
			&tCoord.dX, &tCoord.dY, &tCoord.dZ, &tCoord.dRX, &tCoord.dRY, &tCoord.dRZ,
			&tCoord.dBX, &tCoord.dBY, &tCoord.dBZ, &dSpeedRate, &nWeldSeamType, &dWaveWaitTime);
	}
	T_ROBOT_COORS tS = vtRealWeldTrack.front();
	T_ROBOT_COORS tE = vtRealWeldTrack.back();
	double dDis = TwoPointDis(tS.dX + tS.dBX, tS.dY + tS.dBY, tS.dZ + tS.dBZ, tE.dX + tE.dBX, tE.dY + tE.dBY, tE.dZ + tE.dBZ);
	double dDisZ = fabs(tS.dZ + tS.dBZ - tE.dZ - tE.dBZ);
	eWeldSeamType = dDisZ / dDis > 0.5 ? E_STAND_GROOVE : E_PLAT_GROOVE;

	if ((EOF != nRst) && (nDataColNum != nRst))
	{
		XUI::MesBox::PopOkCancel("加载第{0}组 第{1}个焊缝 检测到不完整数据", nGroupNo, nWeldNo);
		return false;
	}
	fclose(pf);
	return true;
}

bool GrooveWeld::DoWelding(int nGroupNo, int nWeldNo, E_WELD_SEAM_TYPE eWeldSeamType, std::vector<T_ROBOT_COORS>& vtWeldPathPoints,
	const vector<int>& vnPtnType, const T_WELD_PARA& tWeldPara)
{
	return DoWelding_G(nGroupNo, nWeldNo, eWeldSeamType, vtWeldPathPoints, vnPtnType, tWeldPara);

	long long lTimeS = XI_clock();
	vector<T_ANGLE_PULSE> vtAccurateWeldPulse;
	vector<T_ROBOT_COORS> vtAccurateWeldCoord; // 焊接轨迹 + 下枪收枪坐标
	T_ANGLE_PULSE tDownGunPulse;
	T_ANGLE_PULSE tBackGunPulse;
	T_ROBOT_COORS tDownCoord; // 下枪坐标
	T_ROBOT_COORS tBackCoord; // 收枪坐标
	T_ROBOT_MOVE_INFO tRobotMoveInfo;
	vector<T_ROBOT_MOVE_INFO> vtRobotMoveInfo;

	CRobotDriverAdaptor* pRobotDriver = m_pRobotDriver;
	int nRobotNo = pRobotDriver->m_nRobotNo;
	int nTotalPointNum = vtWeldPathPoints.size();
	if (nTotalPointNum < 10)
	{
		XUI::MesBox::PopError("焊接轨迹数据点数{0} 过少，焊接失败！", nTotalPointNum);
		return false;
	}
	// 下枪
	XiAlgorithm alg;
	LineOrCircularArcWeldingLine tWeldLine = m_vvtWeldLineInfoGroup[nGroupNo][nWeldNo].tWeldLine;
	double dNorAngle = alg.CalcArcAngle(tWeldLine.StartNormalVector.x, tWeldLine.StartNormalVector.y);
	//double dMaxHeight = CalcBoardMaxHeight(nGroupNo);
	double dH1 = tWeldLine.StartPoint.z * m_nRobotInstallDir;
	double dH2 = tWeldLine.EndPoint.z * m_nRobotInstallDir;
	double dMaxHeight = dH1 > dH2 ? dH1 * m_nRobotInstallDir : dH2 * m_nRobotInstallDir;

	tDownCoord = vtWeldPathPoints[0];
	tDownCoord.dX += (m_dGunDownBackSafeDis / 2 * CosD(dNorAngle/*RzToDirAngle(tDownCoord.dRZ)*/));
	tDownCoord.dY += (m_dGunDownBackSafeDis / 2 * SinD(dNorAngle/*RzToDirAngle(tDownCoord.dRZ)*/));
	//tDownCoord.dRY = m_dPlatWeldRy; // 过渡点 Ry 45 安全
	tDownCoord.dZ = (dMaxHeight + m_dGunDownBackSafeDis * ((double)m_nRobotInstallDir));
	vtAccurateWeldCoord.push_back(tDownCoord);

	vtAccurateWeldCoord.insert(vtAccurateWeldCoord.end(), vtWeldPathPoints.begin(), vtWeldPathPoints.end());

	tBackCoord = vtWeldPathPoints[vtWeldPathPoints.size() - 1];
	tBackCoord.dX += (m_dGunDownBackSafeDis / 2 * CosD(dNorAngle/*RzToDirAngle(tBackCoord.dRZ)*/));
	tBackCoord.dY += (m_dGunDownBackSafeDis / 2 * SinD(dNorAngle/*RzToDirAngle(tBackCoord.dRZ)*/));
	//tBackCoord.dRY = m_dPlatWeldRy; // 过渡点 Ry 45 安全
	tBackCoord.dZ = (dMaxHeight + m_dGunDownBackSafeDis * ((double)m_nRobotInstallDir));
	vtAccurateWeldCoord.push_back(tBackCoord);

	// 计算下枪关节坐标及可以连续运动的焊接轨迹关节坐标轨迹
	double dRealWeldExPos = GetMoveExAxisPos(vtAccurateWeldCoord.front(), m_ptUnit->m_nMeasureAxisNo);//m_dTeachExAxlePos;
	double dExAxleChangeDisThreshold = 0;
	double dMinExAxleChangeDis = -dExAxleChangeDisThreshold;
	double dMaxExAxleChangeDis = dExAxleChangeDisThreshold;

	if (false == CalcContinuePulseForWeld(vtAccurateWeldCoord, vtAccurateWeldPulse))
	{
		bool bCalcRst = false;
		bool bAddorSubtract = false;//true加false减
		int nIndex = 1;
		double dExAxleChangeDis = 0;
		while (dMinExAxleChangeDis <= dExAxleChangeDis && dExAxleChangeDis <= dMaxExAxleChangeDis) {
			dRealWeldExPos = m_dTeachExAxlePos;
			vector<T_ROBOT_COORS> vtTempWeldCoord(vtAccurateWeldCoord); // 焊接轨迹 + 下枪收枪坐标
			dRealWeldExPos += dExAxleChangeDis;

			double dMaxExAxlePos = (double)m_ptUnit->GetRobotCtrl()->m_tExternalAxle[m_ptUnit->m_nMeasureAxisNo - 1].lMaxPulseNum * m_ptUnit->GetRobotCtrl()->m_tExternalAxle[m_ptUnit->m_nMeasureAxisNo - 1].dPulse;
			double dMinExAxlePos = (double)m_ptUnit->GetRobotCtrl()->m_tExternalAxle[m_ptUnit->m_nMeasureAxisNo - 1].lMinPulseNum * m_ptUnit->GetRobotCtrl()->m_tExternalAxle[m_ptUnit->m_nMeasureAxisNo - 1].dPulse;
			if (dRealWeldExPos > dMaxExAxlePos || dRealWeldExPos < dMinExAxlePos)
			{
				WriteLog("外部轴%d 目标位置%.3lf 超出设定极限%.3lf - %.3lf!", m_ptUnit->m_nMeasureAxisNo, dRealWeldExPos, dMinExAxlePos, dMaxExAxlePos);
				continue;
			}

			for (int nPtnNo = 0; nPtnNo < vtTempWeldCoord.size(); nPtnNo++)
			{
#ifdef SINGLE_ROBOT
				vtTempWeldCoord[nPtnNo].dY -= dExAxleChangeDis;
#else
				vtTempWeldCoord[nPtnNo].dX -= dExAxleChangeDis;
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
			dExAxleChangeDis = false == bAddorSubtract ? -50 * nIndex : 50 * nIndex;
			nIndex = true == bAddorSubtract ? nIndex + 1 : nIndex;
			bAddorSubtract = false == bAddorSubtract ? true : false;
		}
		if (false == bCalcRst)
		{
			XiMessageBox("多外部轴位置，焊接下枪过渡点计算失败！");
			return false;
		}
	}
	//#ifndef SINGLE_ROBOT
		// 检查外部轴是否在dRealWeldExPos
		// 如果不在就运动到dRealWeldExPos并检查是否运动到位
		// 已经dRealWeldExPos直接向下运行
	int nAxisNo = m_ptUnit->m_nMeasureAxisNo;
	double dCurExAxlePos = 0.0;
	dCurExAxlePos = m_ptUnit->GetExPositionDis(nAxisNo);
	if ((false == m_bIsLocalDebug) && (fabs(dCurExAxlePos - dRealWeldExPos) > 5.0)) // 需要运动外部轴
	{
		WriteLog("外部轴开始运动到指定位置");
		if (0 != m_ptUnit->MoveExAxisFun(dRealWeldExPos, pRobotDriver->m_tPulseHighSpeed.dSpeed * 5, nAxisNo))
		{
			return false;
		}
		m_ptUnit->WorldCheckRobotDone();
		double dCurExAxlePos = 0.0;
		dCurExAxlePos = m_ptUnit->GetExPositionDis(nAxisNo);
		if (fabs(dRealWeldExPos - dCurExAxlePos) > 5.0) // 外部轴运动失败
		{
			XiMessageBox("DoTeah:外部轴未运动到指定位置");
			return false;
		}
	}
	//#endif // !SINGLE_ROBOT

	tDownGunPulse = vtAccurateWeldPulse[0];
	vtRobotMoveInfo.clear();

	if (false == m_bIsLocalDebug) // 从当前位置下枪R轴变化过大，需要增加安全过渡点
	{
		T_ANGLE_PULSE tCurAnglePulse = pRobotDriver->GetCurrentPulse();
		double dAngleErrAxisR = (double)(tDownGunPulse.nRPulse - tCurAnglePulse.nRPulse) * m_pRobotDriver->m_tAxisUnit.dRPulse;
		if (fabs(dAngleErrAxisR) > 90.0)
		{
			T_ANGLE_PULSE tSafeDownGunPulse(
				tCurAnglePulse.nSPulse,
				m_pRobotDriver->m_tHomePulse.nLPulse,
				m_pRobotDriver->m_tHomePulse.nUPulse,
				(tCurAnglePulse.nRPulse + tCurAnglePulse.nRPulse) / 2,
				0,
				(tDownGunPulse.nTPulse + tCurAnglePulse.nTPulse) / 2, 
				tDownGunPulse.lBXPulse, tDownGunPulse.lBYPulse, tDownGunPulse.lBZPulse);
			tRobotMoveInfo = m_pRobotDriver->PVarToRobotMoveInfo(0, tSafeDownGunPulse, m_pRobotDriver->m_tPulseHighSpeed, MOVJ);
			vtRobotMoveInfo.push_back(tRobotMoveInfo);
			vtAccurateWeldPulse.insert(vtAccurateWeldPulse.begin(), tSafeDownGunPulse);
		}
	}

	tRobotMoveInfo = m_pRobotDriver->PVarToRobotMoveInfo(0, tDownGunPulse, m_pRobotDriver->m_tPulseHighSpeed, MOVJ);
	vtRobotMoveInfo.push_back(tRobotMoveInfo);

	tRobotMoveInfo = m_pRobotDriver->PVarToRobotMoveInfo(1, vtWeldPathPoints[0], m_pRobotDriver->m_tCoordLowSpeed, MOVL);
	vtRobotMoveInfo.push_back(tRobotMoveInfo);

	if (false == m_bIsLocalDebug) // 下枪
	{
		// 下枪运动 ！！！！
		m_pRobotDriver->SetMoveValue(vtRobotMoveInfo, false);
		m_pRobotDriver->CallJob("CONTIMOVANY");
		//m_pRobotDriver->ContiMoveAny(vtRobotMoveInfo, dRealWeldExPos);
		m_ptUnit->RobotCheckDone();
		if (false == m_pRobotDriver->CompareCoords(vtWeldPathPoints[0], m_pRobotDriver->GetCurrentPos()))
		{
			WriteLog("Check Coors Err1");
			return false;
		}; // 与最后一个下枪坐标有关
	}
	WriteLog("下枪完成");
	if (false == m_bIsLocalDebug) // 焊接
	{
		// 改为联动焊接
		//if (0 > m_ptUnit->WeldMove(vtWeldPathPoints, vnPtnType, tWeldPara, dRealWeldExPos, eWeldSeamType, TRUE == *m_pIsArcOn))
		//{
		//	return false;
		//}
		double dTotalVal = tWeldPara.WeldVelocity/* 800*/; //焊接速度 mm/min
		m_ptUnit->m_bSwingState = true;
		int SwingNumber = tWeldPara.nWrapConditionNo/*666*/;
		int nLayerNo = tWeldPara.nLayerNo;
		m_ptUnit->LoadWeldParam(SwingNumber, nLayerNo);
		//m_ptUnit->WriteSwingParam(2, 1, 1);
		m_ptUnit->SwingTracking(pRobotDriver, vtWeldPathPoints, dTotalVal);

		m_ptUnit->RobotCheckDone();
		CHECK_COORS_RETURN_BOOL(m_pRobotDriver, vtWeldPathPoints[vtWeldPathPoints.size() - 1]);
	}
	WriteLog("焊接完成");

	vtRobotMoveInfo.clear();
	tRobotMoveInfo = m_pRobotDriver->PVarToRobotMoveInfo(0, tBackCoord, m_pRobotDriver->m_tCoordLowSpeed, MOVL);
	vtRobotMoveInfo.push_back(tRobotMoveInfo);
	if (false == m_bIsLocalDebug) // 收枪
	{
		m_pRobotDriver->SetMoveValue(vtRobotMoveInfo, false);
		m_pRobotDriver->CallJob("CONTIMOVANY");
		//m_pRobotDriver->ContiMoveAny(vtRobotMoveInfo, dRealWeldExPos);
		m_ptUnit->RobotCheckDone();
		CHECK_COORS_RETURN_BOOL(m_pRobotDriver, tBackCoord);
	}
	WriteLog("收枪完成");
	long long lWeldTime = XI_clock() - lTimeS; //一条焊道的时间
	double dWeldLenOld = (double)nTotalPointNum / 1000.0; // 单位米
	LineOrCircularArcWeldingLine tWeld = m_vvtWeldSeamGroupAdjust[nGroupNo][nWeldNo];
	//一条焊道的长度
	double dWeldLen = TwoPointDis(tWeld.StartPoint.x, tWeld.StartPoint.y, tWeld.StartPoint.z, tWeld.EndPoint.x, tWeld.EndPoint.y, tWeld.EndPoint.z) / 1000.0;
	dWeldLen = (double)nTotalPointNum / 1000.0;
	CTime time = CTime::GetCurrentTime();
	CString sProductDataFileName;
	sProductDataFileName.Format("Monitor\\%s\\ProductData_%4d.%.2d.%.2d.csv", m_pRobotDriver->m_strRobotName, time.GetYear(), time.GetMonth(), time.GetDay());
	FILE* RecordTheoryData;
	OpenProductLog(&RecordTheoryData, sProductDataFileName);
	SaveProductData(RecordTheoryData, dWeldLen, lWeldTime, nGroupNo);
	CloseProductLog(RecordTheoryData);
	WriteLog("焊缝长度统计：Old:%.3lf New:%.3lf", dWeldLenOld, dWeldLen);

	m_dCurWeldLenBeforeCleanGun += dWeldLen;
	if (m_dCurWeldLenBeforeCleanGun > m_dCleanGunDis) // 大于3米进行一次清枪
	{
		//CHECK_BOOL_RETURN(m_pRobotDriver->CleanGunH());
		m_dCurWeldLenBeforeCleanGun = 0.0;
	}

	// 生成焊接轨迹JOB 测试用
	CString sJobName;
	sJobName.Format("DOWELDING%d-%d-%d", nGroupNo, nWeldNo, tWeldPara.nLayerNo);
	GenerateJobLocalVariable(vtAccurateWeldPulse, MOVL, sJobName);
	//GenerateJobCoord(vtAccurateWeldCoord, MOVL, sJobName);

	return true;
}

bool GrooveWeld::CalcGrooveScanTrack(int nGroupNo, int nSeamNo, double dExAxlePos, vector<LineOrCircularArcWeldingLine> vtSeamGroup, vector<T_TEACH_DATA>& vtTeachData)
{
	vtTeachData.clear();
	XiAlgorithm alg;
	T_ROBOT_COORS tCoord;
	T_TEACH_DATA tTeachData;
	LineOrCircularArcWeldingLine tSeam = vtSeamGroup[nSeamNo];
	double dNorAngle = alg.CalcArcAngle(tSeam.StartNormalVector.x, tSeam.StartNormalVector.y);
	tTeachData.nWeldNo = nSeamNo;
	tTeachData.nMeasurePtnNo = 0;
	tTeachData.nMeasureType = E_SEARCH_POINT | E_SCAN_POINT;
	tTeachData.eWeldSeamType = E_FLAT_SEAM;
	tTeachData.eAttribute = E_BELONG_START;
	tTeachData.eEndpointType = E_FREE_POINT;

	double dRobotChangeDir = 1 == m_nRobotInstallDir ? 1.0 : -1.0;
	double dOffsetDirS = dNorAngle - (90.0 * dRobotChangeDir);
	double dOffsetDirE = dNorAngle + (90.0 * dRobotChangeDir);

	tCoord = GenerateRobotCoord(tSeam.StartPoint, m_dGrooveScanRx, m_dGrooveScanRy, DirAngleToRz(dNorAngle) + m_dGrooveScanOffsetRz);
	RobotCoordPosOffset(tCoord, dOffsetDirS, 0.0/*m_dEndpointSearchDis*/);
	DecomposeExAxle(tCoord, dExAxlePos);
	tTeachData.tMeasureCoordGunTool = tCoord;
	vtTeachData.push_back(tTeachData);

	tCoord = GenerateRobotCoord(tSeam.EndPoint, m_dGrooveScanRx, m_dGrooveScanRy, DirAngleToRz(dNorAngle) + m_dGrooveScanOffsetRz);
	RobotCoordPosOffset(tCoord, dOffsetDirE, 0.0/*m_dEndpointSearchDis*/);
	DecomposeExAxle(tCoord, dExAxlePos);
	tTeachData.tMeasureCoordGunTool = tCoord;
	vtTeachData.push_back(tTeachData);
	return true;
}

bool GrooveWeld::SortMeasureCoordNew(double dPartHeight, double dFirstSeamNorAngle, double dExAxleOffsetDis,
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

bool GrooveWeld::AddSafeCoord(vector<T_ROBOT_COORS>& vtMeasureCoord, vector<int>& vnMeasureType,
	double dMaxPartHeight, double dSafeDis)
{
	double dPointInterval = 5.0;
	T_ROBOT_COORS tMeasureCoord;
	T_ROBOT_COORS tCoordS = vtMeasureCoord.front();
	T_ROBOT_COORS tCoordE = vtMeasureCoord.back();
	int nType = vnMeasureType.front();
	vtMeasureCoord.clear();
	vnMeasureType.clear();
	
	// 添加搜索点下枪过渡点
	tMeasureCoord = tCoordS;
	tMeasureCoord.dZ = dMaxPartHeight + (dSafeDis * (double)m_nRobotInstallDir);
	vtMeasureCoord.push_back(tMeasureCoord);
	vnMeasureType.push_back(E_TRANSITION_POINT);

	// 测量轨迹
	double dDis = TwoPointDis(
		tCoordS.dX + tCoordS.dBX, tCoordS.dY + tCoordS.dBY, tCoordS.dZ + tCoordS.dBZ,
		tCoordE.dX + tCoordE.dBX, tCoordE.dY + tCoordE.dBY, tCoordE.dZ + tCoordE.dBZ);
	double dDisX = tCoordE.dX - tCoordS.dX;
	double dDisY = tCoordE.dY - tCoordS.dY;
	double dDisZ = tCoordE.dZ - tCoordS.dZ;
	double dDisBX = tCoordE.dBX - tCoordS.dBX;
	double dDisBY = tCoordE.dBY - tCoordS.dBY;
	double dDisBZ = tCoordE.dBZ - tCoordS.dBZ;
	int nPtnNum = dDis / dPointInterval;
	double dRealInterval = dDis / (double)nPtnNum;
	for (int i = 0; i <= nPtnNum; i++)
	{
		tMeasureCoord = tCoordS;
		tMeasureCoord.dX += (dDisX * (double)i / (double)nPtnNum);
		tMeasureCoord.dY += (dDisY * (double)i / (double)nPtnNum);
		tMeasureCoord.dZ += (dDisZ * (double)i / (double)nPtnNum);
		tMeasureCoord.dBX += (dDisBX * (double)i / (double)nPtnNum);
		tMeasureCoord.dBY += (dDisBY * (double)i / (double)nPtnNum);
		tMeasureCoord.dBZ += (dDisBZ * (double)i / (double)nPtnNum);
		vtMeasureCoord.push_back(tMeasureCoord);
		vnMeasureType.push_back(nType);
	}


	// 添加搜索点下枪过渡点
	tMeasureCoord = tCoordE;
	tMeasureCoord.dZ = dMaxPartHeight + (dSafeDis * (double)m_nRobotInstallDir);
	vtMeasureCoord.push_back(tMeasureCoord);
	vnMeasureType.push_back(E_TRANSITION_POINT);

	return true;
}

bool GrooveWeld::GetScanProcResult(vector<T_ROBOT_COORS> vtCoord, vector<T_TEACH_RESULT>& vtTeachResult)
{
	m_pRobotDriver->m_cLog->Write("GrooveWeld::GetScanProcResult Start");
	CString sPointCloudFile;
	CString sRecoResultFile;
	CString sSaveResultFile;
	CString sInputFile;
	sPointCloudFile = OUTPUT_PATH + m_pRobotDriver->m_strRobotName + GROOVE_DATA_PATH + "AA_PointCloud.txt";
	int nIdx = sPointCloudFile.ReverseFind('.');
	CString s1 = sPointCloudFile.Left(nIdx);
	sRecoResultFile.Format("%s_Rst.txt", /*UnicodeToUtf8*/(s1.GetBuffer()));
	sSaveResultFile.Format("%s_Save.txt", /*UnicodeToUtf8*/(s1.GetBuffer()));
	sInputFile.Format("%s_Input.txt", /*UnicodeToUtf8*/(s1.GetBuffer()));

	FILE* pf = fopen(sRecoResultFile.GetBuffer(), "w");
	for (int i = 5; i < m_pScanInit->m_pTraceModel->vtPointCloudEndPoints.size(); i++)
	{
		Three_DPoint tPtnS = m_pScanInit->m_pTraceModel->vtPointCloudEndPoints[i - 5];
		Three_DPoint tPtnE = m_pScanInit->m_pTraceModel->vtPointCloudEndPoints[i];
		fprintf(pf, "%d %4d%4d%11.3lf%4d%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%4d%4d 0 0 0 %4d%11.3lf%11.3lf 0 0.0\n",
			0, 0, 0, 0.0, 0,
			0.0, 0.0, 0.0,
			tPtnS.x, tPtnS.y, tPtnS.z,
			tPtnE.x, tPtnE.y, tPtnE.z,
			0.0, 0.0, 0.0,
			0.0, 0.0, 0.0,
			0, 0,
			0, 0.0, 0.0);
	}
	fclose(pf);

	CopyFile((LPCTSTR)sRecoResultFile, (LPCTSTR)sSaveResultFile, FALSE);
	if (*m_pIsNaturalPop)
	{
		m_pRobotDriver->m_cLog->Write("GrooveWeld::GetScanProcResult RunInteractiveWindow Start");
		RunInteractiveWindow(sPointCloudFile, sSaveResultFile, sRecoResultFile);
		m_pRobotDriver->m_cLog->Write("GrooveWeld::GetScanProcResult RunInteractiveWindow End");
	}
	vector<WeldLineInfo> vtWeldLineInfo;
	if (false == LoadCloudProcessResult(sSaveResultFile, vtWeldLineInfo))
	{
		XiMessageBoxOk("提取焊缝 加载识别结果失败!");
		return false;
	}

	vtTeachResult.clear();
	T_TEACH_RESULT tTeachResult;
	memset(&tTeachResult, 0, sizeof(tTeachResult));
	XI_POINT tXiPoint;
	for (int i = 4; i < vtWeldLineInfo.size(); i += 5)
	{
		if (4 == i)
		{
			for (int j = i - 4; j <= i; j++)
			{
				tXiPoint.x = vtWeldLineInfo[j].tWeldLine.StartPoint.x;
				tXiPoint.y = vtWeldLineInfo[j].tWeldLine.StartPoint.y;
				tXiPoint.z = vtWeldLineInfo[j].tWeldLine.StartPoint.z;
				tTeachResult.vtLeftPtns3D.push_back(tXiPoint);
				tTeachResult.vtLeftPtns2D.push_back(cvPoint(0, 0));
			}
		}
		for (int j = i - 4; j <= i; j++)
		{
			tXiPoint.x = vtWeldLineInfo[j].tWeldLine.EndPoint.x;
			tXiPoint.y = vtWeldLineInfo[j].tWeldLine.EndPoint.y;
			tXiPoint.z = vtWeldLineInfo[j].tWeldLine.EndPoint.z;
			tTeachResult.vtLeftPtns3D.push_back(tXiPoint);
			tTeachResult.vtLeftPtns2D.push_back(cvPoint(0, 0));
		}
	}
	vtTeachResult.push_back(tTeachResult);
	m_pRobotDriver->m_cLog->Write("GrooveWeld::GetScanProcResult End");
	return true;
}

bool GrooveWeld::GetScanProcResult_G(int nGroupNo, int nLayerNo, vector<T_ROBOT_COORS> vtCoord, vector<T_TEACH_RESULT>& vtTeachResult)
{
	m_pRobotDriver->m_cLog->Write("GrooveWeld::GetScanProcResult Start");
	CString sPointCloudFile;
	CString sRecoResultFile;
	CString sSaveResultFile;
	CString sInputFile;
	CString sName;
	sName.Format("Layer%d_Group%d_PointCloud.txt", nLayerNo, nGroupNo);
	sPointCloudFile = OUTPUT_PATH + m_pRobotDriver->m_strRobotName + GROOVE_DATA_PATH + sName;
	int nIdx = sPointCloudFile.ReverseFind('.');
	CString s1 = sPointCloudFile.Left(nIdx);
	sRecoResultFile.Format("%s_Rst.txt", /*UnicodeToUtf8*/(s1.GetBuffer()));
	sSaveResultFile.Format("%s_Save.txt", /*UnicodeToUtf8*/(s1.GetBuffer()));
	sInputFile.Format("%s_Input.txt", /*UnicodeToUtf8*/(s1.GetBuffer()));

	FILE* pf = fopen(sRecoResultFile.GetBuffer(), "w");
	for (int i = 5; i < m_pScanInit->m_pTraceModel->vtPointCloudEndPoints.size(); i++)
	{
		Three_DPoint tPtnS = m_pScanInit->m_pTraceModel->vtPointCloudEndPoints[i - 5];
		Three_DPoint tPtnE = m_pScanInit->m_pTraceModel->vtPointCloudEndPoints[i];
		fprintf(pf, "%d %4d%4d%11.3lf%4d%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%4d%4d 0 0 0 %4d%11.3lf%11.3lf 0 0.0\n",
			0, 0, 0, 0.0, 0,
			0.0, 0.0, 0.0,
			tPtnS.x, tPtnS.y, tPtnS.z,
			tPtnE.x, tPtnE.y, tPtnE.z,
			0.0, 0.0, 0.0,
			0.0, 0.0, 0.0,
			0, 0,
			0, 0.0, 0.0);
	}
	fclose(pf);

	CopyFile((LPCTSTR)sRecoResultFile, (LPCTSTR)sSaveResultFile, FALSE);
	if (*m_pIsNaturalPop)
	{
		m_pRobotDriver->m_cLog->Write("GrooveWeld::GetScanProcResult RunInteractiveWindow Start");
		RunInteractiveWindow(sPointCloudFile, sSaveResultFile, sRecoResultFile);
		m_pRobotDriver->m_cLog->Write("GrooveWeld::GetScanProcResult RunInteractiveWindow End");
	}
	vector<WeldLineInfo> vtWeldLineInfo;
	if (false == LoadCloudProcessResult(sSaveResultFile, vtWeldLineInfo))
	{
		XiMessageBoxOk("提取焊缝 加载识别结果失败!");
		return false;
	}

	vtTeachResult.clear();
	T_TEACH_RESULT tTeachResult;
	memset(&tTeachResult, 0, sizeof(tTeachResult));
	XI_POINT tXiPoint;
	for (int i = 4; i < vtWeldLineInfo.size(); i += 5)
	{
		if (4 == i)
		{
			for (int j = i - 4; j <= i; j++)
			{
				tXiPoint.x = vtWeldLineInfo[j].tWeldLine.StartPoint.x;
				tXiPoint.y = vtWeldLineInfo[j].tWeldLine.StartPoint.y;
				tXiPoint.z = vtWeldLineInfo[j].tWeldLine.StartPoint.z;
				tTeachResult.vtLeftPtns3D.push_back(tXiPoint);
				tTeachResult.vtLeftPtns2D.push_back(cvPoint(0, 0));
			}
		}
		for (int j = i - 4; j <= i; j++)
		{
			tXiPoint.x = vtWeldLineInfo[j].tWeldLine.EndPoint.x;
			tXiPoint.y = vtWeldLineInfo[j].tWeldLine.EndPoint.y;
			tXiPoint.z = vtWeldLineInfo[j].tWeldLine.EndPoint.z;
			tTeachResult.vtLeftPtns3D.push_back(tXiPoint);
			tTeachResult.vtLeftPtns2D.push_back(cvPoint(0, 0));
		}
	}
	vtTeachResult.push_back(tTeachResult);
	m_pRobotDriver->m_cLog->Write("GrooveWeld::GetScanProcResult End");
	return true;
}

void GrooveWeld::SaveTeachResult(int nGroupNo)
{
	m_pRobotDriver->m_cLog->Write("GrooveWeld::SaveTeachResult!");
	CString sFileName;
	sFileName.Format("%s%d-TeachResult.txt", m_sDataSavePath, nGroupNo);
	FILE* pf = fopen(sFileName.GetBuffer(), "w");
	for (int i = 0; i < m_vtTeachResult.size(); i++)
	{
		T_TEACH_RESULT tTeachResult = m_vtTeachResult[i];
		// 示教点号 交点左线右线 三维点 二维点 采图直角 采图关节 采图外部轴
		// 左线
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
		// 右线
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

void GrooveWeld::SaveTeachResult_G(int nGroupNo, int nLayerNo)
{
	m_pRobotDriver->m_cLog->Write("GrooveWeld::SaveTeachResult!");
	CString sFileName;
	sFileName.Format("%s%d-%d-TeachResult.txt", m_sDataSavePath, nLayerNo, nGroupNo);
	FILE* pf = fopen(sFileName.GetBuffer(), "w");
	for (int i = 0; i < m_vtTeachResult.size(); i++)
	{
		T_TEACH_RESULT tTeachResult = m_vtTeachResult[i];
		// 示教点号 交点左线右线 三维点 二维点 采图直角 采图关节 采图外部轴
		// 左线
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
		// 右线
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

bool GrooveWeld::GeneralGrooveWeldTrack(int nGroupNo)
{
	return false;
}

bool GrooveWeld::CalcMeasureTrack(int nGroupNo, std::vector<T_ROBOT_COORS>& vtMeasureCoord, std::vector<T_ANGLE_PULSE>& vtMeasurePulse, vector<int>& vnMeasureType, double& dSafeHeight)
{
	//double dExAxlePos = 0.0;
	T_ROBOT_COORS tFixCoord(0.0, -1000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0); // 表示机器人Y固定1000，外部轴运动 都是0表示外部轴固定
	T_ROBOT_COORS tThersholdCoord(10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 0.0, 0.0, 0.0); // 相同测量点判断阈值
	vtMeasureCoord.clear(); // 转跟踪相机工具后
	vtMeasurePulse.clear();
	vnMeasureType.clear();
	m_vtTeachData.clear();
	m_vnTeachTrackOrder.clear();
	m_vtTeachResult.clear();
	T_TEACH_DATA tTeachData;
	vector<T_TEACH_DATA> vtTeachData;
	vector<LineOrCircularArcWeldingLine> vtSeamGroup = m_vvtWeldSeamGroup[nGroupNo];

	CvPoint3D64f tTotalValue = { 0 }; // 统计XYZ值 确定外部轴位置
	double dPartMaxHeight = -99999.0; // 正座记录工件最高点Z值 倒挂记录工件最低点
	for (int nWeldSeamNo = 0; nWeldSeamNo < vtSeamGroup.size(); nWeldSeamNo++)
	{
		tTotalValue.x += (vtSeamGroup[nWeldSeamNo].StartPoint.x);
		tTotalValue.x += (vtSeamGroup[nWeldSeamNo].EndPoint.x);
		tTotalValue.y += (vtSeamGroup[nWeldSeamNo].StartPoint.y);
		tTotalValue.y += (vtSeamGroup[nWeldSeamNo].EndPoint.y);
		tTotalValue.z += (vtSeamGroup[nWeldSeamNo].StartPoint.z);
		tTotalValue.z += (vtSeamGroup[nWeldSeamNo].EndPoint.z);

		dPartMaxHeight = dPartMaxHeight < vtSeamGroup[nWeldSeamNo].StartPoint.z * m_nRobotInstallDir ? vtSeamGroup[nWeldSeamNo].StartPoint.z * m_nRobotInstallDir : dPartMaxHeight;
		dPartMaxHeight = dPartMaxHeight < vtSeamGroup[nWeldSeamNo].EndPoint.z * m_nRobotInstallDir ? vtSeamGroup[nWeldSeamNo].EndPoint.z * m_nRobotInstallDir : dPartMaxHeight;
	}
	dPartMaxHeight *= m_nRobotInstallDir;

	std::vector<T_ROBOT_COORS> vtMeasureCoordGunTool(0); // 合并后的焊枪工具坐标
	for (int nSeamNo = 0; nSeamNo < vtSeamGroup.size(); nSeamNo++)
	{
		CHECK_BOOL_RETURN(CalcGrooveScanTrack(nGroupNo, nSeamNo, 0.0, vtSeamGroup, vtTeachData));
		for (int nNo = 0; nNo < vtTeachData.size(); nNo++)
		{
			tTeachData = vtTeachData[nNo];
			tTeachData.nMeasurePtnNo = vtMeasureCoord.size();
			vtMeasureCoordGunTool.push_back(tTeachData.tMeasureCoordGunTool);
			vtMeasureCoord.push_back(tTeachData.tMeasureCoordGunTool);
			vnMeasureType.push_back(tTeachData.nMeasureType);
			m_vtTeachData.push_back(tTeachData);
		}
	}

	// 保存测量轨迹焊枪工具坐标（xyz保存世界坐标）
	CString sFileName;
	sFileName.Format("%s%d-MeasureTrackGunTool.txt", m_sDataSavePath, nGroupNo);
	FILE* pf = fopen(sFileName.GetBuffer(), "w");
	for (int i = 0; i < m_vtTeachData.size(); i++)
	{
		fprintf(pf, "%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%4d%4d " + GetTeachDataString(m_vtTeachData[i]) + "\n",
			m_vtTeachData[i].tMeasureCoordGunTool.dX, m_vtTeachData[i].tMeasureCoordGunTool.dY, m_vtTeachData[i].tMeasureCoordGunTool.dZ,
			m_vtTeachData[i].tMeasureCoordGunTool.dRX, m_vtTeachData[i].tMeasureCoordGunTool.dRY, m_vtTeachData[i].tMeasureCoordGunTool.dRZ,
			m_vtTeachData[i].tMeasureCoordGunTool.dBX, m_vtTeachData[i].tMeasureCoordGunTool.dBY, m_vtTeachData[i].tMeasureCoordGunTool.dBZ,
			m_vtTeachData[i].nWeldNo, m_vtTeachData[i].nMeasurePtnNo);
	}
	fclose(pf);

	// 外部轴位置偏移 : 0.0  ±100.0  ±200.0  ±300.0 ……
	vector<double> vdExAxleOffset;
	vdExAxleOffset.clear();
	vdExAxleOffset.push_back(0.0);
	for (int i = 100; i < 501; i += 100)
	{
		vdExAxleOffset.push_back((double)i);
	}

	bool bCalcSuccess = false;
	for (int nOffsetIdx = 0; (false == bCalcSuccess) && (nOffsetIdx < vdExAxleOffset.size()); nOffsetIdx++)
	{
		std::vector<T_ROBOT_COORS> vtTempCoord(vtMeasureCoord);
		std::vector<T_ANGLE_PULSE> vtTempPulse(vtMeasurePulse);
		vector<int> vnTempType(vnMeasureType);

		m_tFixCoord = tFixCoord;
		m_tFixCoord.dY += vdExAxleOffset[nOffsetIdx];

		//sFileName.Format("%s%d_Offset%.0lf_SortMeasureCoordBefore.txt", m_sDataSavePath, nGroupNo, vdExAxleOffset[nOffsetIdx]);
		//if (false == SortMeasureCoordNew(dPartMaxHeight, 0.0,
		//	vdExAxleOffset[nOffsetIdx], dTempExAxlePos, vtTempCoord, vnTempType, m_vnTeachTrackOrder))
		//{
		//	WriteLog("ExAxleOffset:%3lf SortMeasureCoord Fail!", vdExAxleOffset[nOffsetIdx]);
		//	continue;
		//}

		sFileName.Format("%s%d_Offset%.0lf_SortMeasureCoord.txt", m_sDataSavePath, nGroupNo, vdExAxleOffset[nOffsetIdx]);
		// 添加收枪 下枪坐标
		dSafeHeight = dPartMaxHeight + (m_dGunDownBackSafeDis * (double)m_nRobotInstallDir);
		if (false == AddSafeCoord(vtTempCoord, vnTempType, dPartMaxHeight, m_dGunDownBackSafeDis))
		{
			WriteLog("ExAxleOffset:%3lf AddSafeCoord Fail!", vdExAxleOffset[nOffsetIdx]);
			continue;
		}
		sFileName.Format("%s%d_Offset%.0lf_AddSafeCoord.txt", m_sDataSavePath, nGroupNo, vdExAxleOffset[nOffsetIdx]);
		//SaveCoordToFile(vtTempCoord, sFileName);

		// 分解世界坐标xyz 到 机器人xyz + 外部轴BxByBz
		if (false == DecomposeCoordinate(vtTempCoord, m_tFixCoord, m_ptUnit->m_nMeasureAxisNo))
		{
			WriteLog("DecomposeCoordinate Fail!");
			continue;
		}

		// 转相机工具坐标
		if (false == TransCameraToolCoord(vtTempCoord, m_ptUnit->m_nMeasureCameraNo))
		{
			WriteLog("TransCameraToolCoord Fail!");
			continue;
		}

		if (false == CalcContinuePulseForWeld(vtTempCoord, vtTempPulse, false))
		{
			WriteLog("ExAxleOffset:%3lf CalcContinuePulseForWeld Fail!", vdExAxleOffset[nOffsetIdx]);
			continue;
		}

		bCalcSuccess = true;
		vtMeasureCoord.clear();
		vtMeasurePulse.clear();
		vnMeasureType.clear();
		vtMeasureCoord.assign(vtTempCoord.begin(), vtTempCoord.end());
		vtMeasurePulse.assign(vtTempPulse.begin(), vtTempPulse.end());
		vnMeasureType.assign(vnTempType.begin(), vnTempType.end());

		// 计算成功，检查当前位置和测量轨迹第一个坐标R轴差值，超过阈值再添加一个过渡点
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
					(vtMeasurePulse[0].nTPulse - tCurAnglePulse.nTPulse) / 2,
					vtMeasurePulse[0].lBXPulse, vtMeasurePulse[0].lBYPulse, vtMeasurePulse[0].lBZPulse);
				m_pRobotDriver->RobotKinematics(tSafeDownGunPulse, m_pRobotDriver->m_tTools.tGunTool, tSafeDownCoord);
				vtMeasurePulse.insert(vtMeasurePulse.begin(), tSafeDownGunPulse);
				vtMeasureCoord.insert(vtMeasureCoord.begin(), tSafeDownCoord);
				vnMeasureType.insert(vnMeasureType.begin(), E_TRANSITION_POINT);
			}
		}

		WriteLog("ExAxleOffset:%3lf 计算连续测量运动轨迹成功", vdExAxleOffset[nOffsetIdx]);
	}
	if (false == bCalcSuccess)
	{
		XiMessageBox("多位置计算连续测量轨迹失败！");
		return false;
	}

	// 保存实际测量运行轨迹 关节坐标 直角坐标 外部轴 
	sFileName.Format("%s%d-MeasureTrackRealCoordPulseEx.txt", m_sDataSavePath, nGroupNo);
	pf = fopen(sFileName.GetBuffer(), "w");
	for (int i = 0; i < vtMeasureCoord.size(); i++)
	{
		fprintf(pf, "%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%10d%10d%10d%10d%10d%10d%10d%10d%10d\n",
			vtMeasureCoord[i].dX, vtMeasureCoord[i].dY, vtMeasureCoord[i].dZ,
			vtMeasureCoord[i].dRX, vtMeasureCoord[i].dRY, vtMeasureCoord[i].dRZ,
			vtMeasureCoord[i].dBX, vtMeasureCoord[i].dBY, vtMeasureCoord[i].dBZ,
			vtMeasurePulse[i].nSPulse, vtMeasurePulse[i].nLPulse, vtMeasurePulse[i].nUPulse,
			vtMeasurePulse[i].nRPulse, vtMeasurePulse[i].nBPulse, vtMeasurePulse[i].nTPulse,
			vtMeasurePulse[i].lBXPulse, vtMeasurePulse[i].lBYPulse, vtMeasurePulse[i].lBZPulse);
	}
	fclose(pf);

	// 离线调试使用
 	CString sJobName;
	sJobName.Format("MOVJTEACHBOARD%d", nGroupNo);
	GenerateJob(vtMeasurePulse, MOVJ, sJobName);
	GenerateFilePLY(nGroupNo, 0.0); // 调试使用

	//if (m_bIsLocalDebug)
	//{
	//	m_pScanInit->InitDynamicCapture_H_M(E_POINT_CLOUD_PROC_MOMENT_FINAL, E_IMAGE_PROC_METHOD_EEEEE, E_POINT_CLOUD_PROC_METHOD_Groove,
	//		m_ptUnit->m_nMeasureCameraNo, OUTPUT_PATH + m_pRobotDriver->m_strRobotName + GROOVE_DATA_PATH, 80);
	//	m_pScanInit->FinalProc_Groove(); // 调试使用
	//	GetScanProcResult(vtMeasureCoord, m_vtTeachResult);
	//	SaveTeachResult(nGroupNo);
	//}
	return true;
}

bool GrooveWeld::DecomposeCoordinate(T_ROBOT_COORS& tRobotCoors, T_ROBOT_COORS tFixCoord, int nDecomposeAxisNo)
{
	if (fabs(tFixCoord.dX) < 0.0001)
	{
		tRobotCoors.dX += tRobotCoors.dBX; 
		tRobotCoors.dBX = 0.0;
	}
	else
	{
		tRobotCoors.dBX = tRobotCoors.dX + tRobotCoors.dBX - tFixCoord.dX;
		tRobotCoors.dX = tFixCoord.dX;
	}

	if (fabs(tFixCoord.dY) < 0.0001)
	{
		tRobotCoors.dY += tRobotCoors.dBY;
		tRobotCoors.dBY = 0.0;
	}
	else
	{
		tRobotCoors.dBY = tRobotCoors.dY + tRobotCoors.dBY - tFixCoord.dY;
		tRobotCoors.dY = tFixCoord.dY;
	}
	if (fabs(tFixCoord.dZ) < 0.0001)
	{
		tRobotCoors.dZ += tRobotCoors.dBZ;
		tRobotCoors.dBZ = 0.0;
	}
	else
	{
		tRobotCoors.dBZ = tRobotCoors.dZ + tRobotCoors.dBZ - tFixCoord.dZ;
		tRobotCoors.dZ = tFixCoord.dZ;
	}
	return true;
}

bool GrooveWeld::DecomposeCoordinate(std::vector<T_ROBOT_COORS>& vtRobotCoors, T_ROBOT_COORS tFixCoord, int nDecomposeAxisNo)
{
	bool bRst = true;
	for (int i = 0; i < vtRobotCoors.size(); i++)
	{
		bRst = bRst && DecomposeCoordinate(vtRobotCoors[i], tFixCoord, nDecomposeAxisNo);
	}
	return bRst;
}

bool GrooveWeld::TransCameraToolCoord(std::vector<T_ROBOT_COORS>& vtCoord, int nCameraNo)
{
	T_ROBOT_COORS tCameraTool = m_ptUnit->GetCameraParam(nCameraNo).tCameraTool;
	for (int i = 0; i < vtCoord.size(); i++)
	{
		// 焊枪工具 转 跟踪相机工具
		if (false == m_pRobotDriver->MoveToolByWeldGun(vtCoord[i], m_pRobotDriver->m_tTools.tGunTool, vtCoord[i], tCameraTool, vtCoord[i]))
		{
			m_pRobotDriver->m_cLog->Write("Robot %d 第%d个测量点 转换失败！", m_pRobotDriver->m_nRobotNo, i);
			return false;
		}
	}
	return true;
}

//bool GrooveWeld::DoTeach_G(int nGroupNo,int nLayerNo, const std::vector<T_ANGLE_PULSE>& vtMeasurePulse, const vector<int>& vnMeasureType, double dExAxlePos111)
//{
//	// 数据检查
//	if (vtMeasurePulse.size() < 3 || vtMeasurePulse.size() != vnMeasureType.size())
//	{
//		XiMessageBox("示教数据错误！");
//		return false;
//	}
//
//	// 数据记录到成员变量（供线程函数使用）
//	SetTeachData(vtMeasurePulse, vnMeasureType, 0.0);
//
//	vector<T_TEACH_RESULT> vtTeachResult(0);
//	if (vtMeasurePulse.size() > 0)
//	{
//		//// 外部轴运动
//		//int nAxisNo = m_ptUnit->m_nMeasureAxisNo;
//		//T_ROBOT_COORS tFirstCoord;
//		//m_pRobotDriver->RobotKinematics(vtMeasurePulse.front(), m_pRobotDriver->m_tTools.tGunTool, tFirstCoord);
//		//double dExPos = GetMoveExAxisPos(tFirstCoord, nAxisNo);
//		//if (0 != m_ptUnit->MoveExAxisFun(dExPos, m_pRobotDriver->m_tPulseHighSpeed.dSpeed * 5, nAxisNo))
//		//{
//		//	return false;
//		//}
//		//m_ptUnit->WorldCheckRobotDone();
//		//double dCurExPos = 0.0;
//		//// 需要更改获取轴数据，临时更改
//		//dCurExPos = m_ptUnit->GetExPositionDis(nAxisNo);
//		//if (!GetLocalDebugMark() && fabs(dExPos - dCurExPos) > 5.0) // 与目标位置相差超过阈值 判断为运动失败
//		//{
//		//	XiMessageBox("自动示教:外部轴未运动到指定位置");
//		//	return false;
//		//}
//		CHECK_BOOL_RETURN(ScanEndpoint_G(nGroupNo,nLayerNo, m_ptUnit->m_nMeasureCameraNo, vtMeasurePulse, vnMeasureType, vtTeachResult));
//	}
//
//	// 汇总结果
//	m_vtTeachResult.clear();
//	m_vtTeachResult.assign(vtTeachResult.begin(), vtTeachResult.end());
//
//	// 保存示教结果(示教及搜索结果)
//	SaveTeachResult(nGroupNo);
//	return true;
//}

bool GrooveWeld::CalcGrooveScanTrack_G(int nGroupNo, int nSeamNo, double dExAxlePos, vector<LineOrCircularArcWeldingLine> vtSeamGroup, vector<T_TEACH_DATA>& vtTeachData)
{
	vtTeachData.clear();
	XiAlgorithm alg;
	T_ROBOT_COORS tCoord;
	T_TEACH_DATA tTeachData;
	LineOrCircularArcWeldingLine tSeam = vtSeamGroup[nSeamNo];
	tTeachData.nWeldNo = nSeamNo;
	tTeachData.nMeasurePtnNo = 0;
	tTeachData.nMeasureType = E_SEARCH_POINT | E_SCAN_POINT;
	tTeachData.eWeldSeamType = E_FLAT_SEAM;
	tTeachData.eAttribute = E_BELONG_START;
	tTeachData.eEndpointType = E_FREE_POINT;

	double dRobotChangeDir = 1 == m_nRobotInstallDir ? 1.0 : -1.0;

	tCoord = GenerateRobotCoord(tSeam.StartPoint, m_dGrooveScanRx, m_dGrooveScanRy, m_dGrooveScanOffsetRz);
	RobotCoordPosOffset(tCoord, tSeam.EndPoint, tSeam.StartPoint, 0.0/*m_dEndpointSearchDis*/);
	DecomposeExAxle(tCoord, dExAxlePos);
	tTeachData.tMeasureCoordGunTool = tCoord;
	vtTeachData.push_back(tTeachData);

	tCoord = GenerateRobotCoord(tSeam.EndPoint, m_dGrooveScanRx, m_dGrooveScanRy, m_dGrooveScanOffsetRz);
	RobotCoordPosOffset(tCoord, tSeam.StartPoint, tSeam.EndPoint, 0.0/*m_dEndpointSearchDis*/);
	DecomposeExAxle(tCoord, dExAxlePos);
	tTeachData.tMeasureCoordGunTool = tCoord;
	vtTeachData.push_back(tTeachData);
	return true;
}

bool GrooveWeld::AddSafeCoord_G(vector<T_ROBOT_COORS>& vtMeasureCoord, vector<int>& vnMeasureType, double dMaxPartHeight, double dSafeDis, E_WELD_SEAM_TYPE eWeldSeamType/* = E_PLAT_GROOVE*/)
{
	double dPointInterval = 5.0;
	T_ROBOT_COORS tMeasureCoord;
	T_ROBOT_COORS tCoordS = vtMeasureCoord.front();
	T_ROBOT_COORS tCoordE = vtMeasureCoord.back();
	int nType = vnMeasureType.front();
	vtMeasureCoord.clear();
	vnMeasureType.clear();

	// 添加搜索点下枪过渡点
	tMeasureCoord = tCoordS;
	if (E_STAND_GROOVE == eWeldSeamType)  tMeasureCoord.dY += dSafeDis;
	tMeasureCoord.dZ += /*dMaxPartHeight + */(dSafeDis * (double)m_nRobotInstallDir);
	vtMeasureCoord.push_back(tMeasureCoord);
	vnMeasureType.push_back(E_TRANSITION_POINT);

	//// 测量轨迹
	vtMeasureCoord.push_back(tCoordS);
	vnMeasureType.push_back(nType);
	vtMeasureCoord.push_back(tCoordE);
	vnMeasureType.push_back(nType);

	// 添加搜索点下枪过渡点
	tMeasureCoord = tCoordE;
	if (E_STAND_GROOVE == eWeldSeamType)  tMeasureCoord.dY += dSafeDis;
	tMeasureCoord.dZ += /*dMaxPartHeight + */(dSafeDis * (double)m_nRobotInstallDir);
	vtMeasureCoord.push_back(tMeasureCoord);
	vnMeasureType.push_back(E_TRANSITION_POINT);

	return true;
}

bool GrooveWeld::CalcMeasureTrack_G(int nGroupNo, std::vector<T_ROBOT_COORS>& vtMeasureCoord, std::vector<T_ANGLE_PULSE>& vtMeasurePulse, vector<int>& vnMeasureType, double& dSafeHeight)
{
	// 平焊？ 爬坡焊下？ 爬坡焊上？
	CvPoint3D64f pS = m_vvtWeldSeamGroup[nGroupNo][0].StartPoint;
	CvPoint3D64f pE = m_vvtWeldSeamGroup[nGroupNo][0].EndPoint;
	double dDis = TwoPointDis(pS.x, pS.y, pS.z, pE.x, pE.y, pE.z);
	double dDisZ = fabs(pS.z - pE.z);

	m_eWeldSeamType = E_PLAT_GROOVE;
	T_ROBOT_COORS tFixCoord = m_pRobotDriver->m_tFlatGrooveScanPosture; // 更新分解固定坐标
	m_dGrooveScanRx = m_pRobotDriver->m_tFlatGrooveScanPosture.dRX;		// 更新扫描姿态
	m_dGrooveScanRy = m_pRobotDriver->m_tFlatGrooveScanPosture.dRY;
	m_dGrooveScanOffsetRz = m_pRobotDriver->m_tFlatGrooveScanPosture.dRZ;
	m_dGrooveWeldRx = m_pRobotDriver->m_tFlatGrooveWeldPosture.dRX;		// 更新焊接姿态
	m_dGrooveWeldRy = m_pRobotDriver->m_tFlatGrooveWeldPosture.dRY;
	m_dGrooveWeldRz = m_pRobotDriver->m_tFlatGrooveWeldPosture.dRZ;
	if (0.5 < dDisZ / dDis)
	{
		m_eWeldSeamType = E_STAND_GROOVE;
		tFixCoord = m_pRobotDriver->m_tStandDownGrooveScanPosture;
		m_dGrooveScanRx = m_pRobotDriver->m_tStandDownGrooveScanPosture.dRX;		// 更新扫描姿态
		m_dGrooveScanRy = m_pRobotDriver->m_tStandDownGrooveScanPosture.dRY;
		m_dGrooveScanOffsetRz = m_pRobotDriver->m_tStandDownGrooveScanPosture.dRZ;
		m_dGrooveWeldRx = m_pRobotDriver->m_tStandDownGrooveWeldPosture.dRX;		// 更新焊接姿态
		m_dGrooveWeldRy = m_pRobotDriver->m_tStandDownGrooveWeldPosture.dRY;
		m_dGrooveWeldRz = m_pRobotDriver->m_tStandDownGrooveWeldPosture.dRZ;
		if (pS.z < 1800.0 && pE.z < 1800.0)
		{
			tFixCoord = m_pRobotDriver->m_tStandUpGrooveScanPosture;
			m_dGrooveScanRx = m_pRobotDriver->m_tStandUpGrooveScanPosture.dRX;		// 更新扫描姿态
			m_dGrooveScanRy = m_pRobotDriver->m_tStandUpGrooveScanPosture.dRY;
			m_dGrooveScanOffsetRz = m_pRobotDriver->m_tStandUpGrooveScanPosture.dRZ;
			m_dGrooveWeldRx = m_pRobotDriver->m_tStandUpGrooveWeldPosture.dRX;		// 更新焊接姿态
			m_dGrooveWeldRy = m_pRobotDriver->m_tStandUpGrooveWeldPosture.dRY;
			m_dGrooveWeldRz = m_pRobotDriver->m_tStandUpGrooveWeldPosture.dRZ;
		}
	}

	T_ROBOT_COORS tThersholdCoord(10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 0.0, 0.0, 0.0); // 相同测量点判断阈值
	vtMeasureCoord.clear(); // 转跟踪相机工具后
	vtMeasurePulse.clear();
	vnMeasureType.clear();
	m_vtTeachData.clear();
	m_vnTeachTrackOrder.clear();
	m_vtTeachResult.clear();
	T_TEACH_DATA tTeachData;
	vector<T_TEACH_DATA> vtTeachData;
	vector<LineOrCircularArcWeldingLine> vtSeamGroup = m_vvtWeldSeamGroup[nGroupNo];

	CvPoint3D64f tTotalValue = { 0 }; // 统计XYZ值 确定外部轴位置
	double dPartMaxHeight = -99999.0; // 正座记录工件最高点Z值 倒挂记录工件最低点
	for (int nWeldSeamNo = 0; nWeldSeamNo < vtSeamGroup.size(); nWeldSeamNo++)
	{
		tTotalValue.x += (vtSeamGroup[nWeldSeamNo].StartPoint.x);
		tTotalValue.x += (vtSeamGroup[nWeldSeamNo].EndPoint.x);
		tTotalValue.y += (vtSeamGroup[nWeldSeamNo].StartPoint.y);
		tTotalValue.y += (vtSeamGroup[nWeldSeamNo].EndPoint.y);
		tTotalValue.z += (vtSeamGroup[nWeldSeamNo].StartPoint.z);
		tTotalValue.z += (vtSeamGroup[nWeldSeamNo].EndPoint.z);

		dPartMaxHeight = dPartMaxHeight < vtSeamGroup[nWeldSeamNo].StartPoint.z * m_nRobotInstallDir ? vtSeamGroup[nWeldSeamNo].StartPoint.z * m_nRobotInstallDir : dPartMaxHeight;
		dPartMaxHeight = dPartMaxHeight < vtSeamGroup[nWeldSeamNo].EndPoint.z * m_nRobotInstallDir ? vtSeamGroup[nWeldSeamNo].EndPoint.z * m_nRobotInstallDir : dPartMaxHeight;
	}
	dPartMaxHeight *= m_nRobotInstallDir;

	std::vector<T_ROBOT_COORS> vtMeasureCoordGunTool(0); // 合并后的焊枪工具坐标
	for (int nSeamNo = 0; nSeamNo < vtSeamGroup.size(); nSeamNo++)
	{
		CHECK_BOOL_RETURN(CalcGrooveScanTrack_G(nGroupNo, nSeamNo, 0.0, vtSeamGroup, vtTeachData));
		for (int nNo = 0; nNo < vtTeachData.size(); nNo++)
		{
			tTeachData = vtTeachData[nNo];
			tTeachData.nMeasurePtnNo = vtMeasureCoord.size();
			vtMeasureCoordGunTool.push_back(tTeachData.tMeasureCoordGunTool);
			vtMeasureCoord.push_back(tTeachData.tMeasureCoordGunTool);
			vnMeasureType.push_back(tTeachData.nMeasureType);
			m_vtTeachData.push_back(tTeachData);
		}
	}

	// 保存测量轨迹焊枪工具坐标（xyz保存世界坐标）
	CString sFileName;
	sFileName.Format("%s%d-MeasureTrackGunTool.txt", m_sDataSavePath, nGroupNo);
	FILE* pf = fopen(sFileName.GetBuffer(), "w");
	for (int i = 0; i < m_vtTeachData.size(); i++)
	{
		fprintf(pf, "%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%4d%4d " + GetTeachDataString(m_vtTeachData[i]) + "\n",
			m_vtTeachData[i].tMeasureCoordGunTool.dX, m_vtTeachData[i].tMeasureCoordGunTool.dY, m_vtTeachData[i].tMeasureCoordGunTool.dZ,
			m_vtTeachData[i].tMeasureCoordGunTool.dRX, m_vtTeachData[i].tMeasureCoordGunTool.dRY, m_vtTeachData[i].tMeasureCoordGunTool.dRZ,
			m_vtTeachData[i].tMeasureCoordGunTool.dBX, m_vtTeachData[i].tMeasureCoordGunTool.dBY, m_vtTeachData[i].tMeasureCoordGunTool.dBZ,
			m_vtTeachData[i].nWeldNo, m_vtTeachData[i].nMeasurePtnNo);
	}
	fclose(pf);

	// 外部轴位置偏移 : 0.0  ±100.0  ±200.0  ±300.0 ……
	vector<double> vdExAxleOffset;
	vdExAxleOffset.clear();
	vdExAxleOffset.push_back(0.0);
	//for (int i = 100; i < 501; i += 100)
	//{
	//	vdExAxleOffset.push_back((double)i);
	//}

	bool bCalcSuccess = false;
	for (int nOffsetIdx = 0; (false == bCalcSuccess) && (nOffsetIdx < vdExAxleOffset.size()); nOffsetIdx++)
	{
		std::vector<T_ROBOT_COORS> vtTempCoord(vtMeasureCoord);
		std::vector<T_ANGLE_PULSE> vtTempPulse(vtMeasurePulse);
		vector<int> vnTempType(vnMeasureType);

		m_tFixCoord = tFixCoord;
		m_tFixCoord.dY += vdExAxleOffset[nOffsetIdx];

		//sFileName.Format("%s%d_Offset%.0lf_SortMeasureCoordBefore.txt", m_sDataSavePath, nGroupNo, vdExAxleOffset[nOffsetIdx]);
		//if (false == SortMeasureCoordNew(dPartMaxHeight, 0.0,
		//	vdExAxleOffset[nOffsetIdx], dTempExAxlePos, vtTempCoord, vnTempType, m_vnTeachTrackOrder))
		//{
		//	WriteLog("ExAxleOffset:%3lf SortMeasureCoord Fail!", vdExAxleOffset[nOffsetIdx]);
		//	continue;
		//}

		sFileName.Format("%s%d_Offset%.0lf_SortMeasureCoord.txt", m_sDataSavePath, nGroupNo, vdExAxleOffset[nOffsetIdx]);
		// 添加收枪 下枪坐标
		dSafeHeight = dPartMaxHeight + (m_dGunDownBackSafeDis * (double)m_nRobotInstallDir);
		if (false == AddSafeCoord_G(vtTempCoord, vnTempType, dPartMaxHeight, m_dGunDownBackSafeDis, m_eWeldSeamType))
		{
			WriteLog("ExAxleOffset:%3lf AddSafeCoord Fail!", vdExAxleOffset[nOffsetIdx]);
			continue;
		}
		sFileName.Format("%s%d_Offset%.0lf_AddSafeCoord.txt", m_sDataSavePath, nGroupNo, vdExAxleOffset[nOffsetIdx]);
		//SaveCoordToFile(vtTempCoord, sFileName);

		// 分解世界坐标xyz 到 机器人xyz + 外部轴BxByBz
		if (false == DecomposeCoordinate(vtTempCoord, m_tFixCoord, m_ptUnit->m_nMeasureAxisNo))
		{
			WriteLog("DecomposeCoordinate Fail!");
			continue;
		}

		SaveCoordExAxleProcess(vtTempCoord, vnTempType);

		// 转相机工具坐标
		if (false == TransCameraToolCoord(vtTempCoord, m_ptUnit->m_nMeasureCameraNo))
		{
			WriteLog("TransCameraToolCoord Fail!");
			continue;
		}

		if (false == CalcContinuePulseForWeld(vtTempCoord, vtTempPulse, false))
		{
			WriteLog("ExAxleOffset:%3lf CalcContinuePulseForWeld Fail!", vdExAxleOffset[nOffsetIdx]);
			continue;
		}

		bCalcSuccess = true;
		vtMeasureCoord.clear();
		vtMeasurePulse.clear();
		vnMeasureType.clear();
		vtMeasureCoord.assign(vtTempCoord.begin(), vtTempCoord.end());
		vtMeasurePulse.assign(vtTempPulse.begin(), vtTempPulse.end());
		vnMeasureType.assign(vnTempType.begin(), vnTempType.end());

		// 计算成功，检查当前位置和测量轨迹第一个坐标R轴差值，超过阈值再添加一个过渡点
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
					(vtMeasurePulse[0].nTPulse - tCurAnglePulse.nTPulse) / 2,
					vtMeasurePulse[0].lBXPulse, vtMeasurePulse[0].lBYPulse, vtMeasurePulse[0].lBZPulse);
				m_pRobotDriver->RobotKinematics(tSafeDownGunPulse, m_pRobotDriver->m_tTools.tGunTool, tSafeDownCoord);
				vtMeasurePulse.insert(vtMeasurePulse.begin(), tSafeDownGunPulse);
				vtMeasureCoord.insert(vtMeasureCoord.begin(), tSafeDownCoord);
				vnMeasureType.insert(vnMeasureType.begin(), E_TRANSITION_POINT);
			}
		}

		WriteLog("ExAxleOffset:%3lf 计算连续测量运动轨迹成功", vdExAxleOffset[nOffsetIdx]);
	}
	if (false == bCalcSuccess)
	{
		XiMessageBox("多位置计算连续测量轨迹失败！");
		return false;
	}

	// 保存实际测量运行轨迹 关节坐标 直角坐标 外部轴 
	sFileName.Format("%s%d-MeasureTrackRealCoordPulseEx.txt", m_sDataSavePath, nGroupNo);
	pf = fopen(sFileName.GetBuffer(), "w");
	for (int i = 0; i < vtMeasureCoord.size(); i++)
	{
		fprintf(pf, "%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%10d%10d%10d%10d%10d%10d%10d%10d%10d\n",
			vtMeasureCoord[i].dX, vtMeasureCoord[i].dY, vtMeasureCoord[i].dZ,
			vtMeasureCoord[i].dRX, vtMeasureCoord[i].dRY, vtMeasureCoord[i].dRZ,
			vtMeasureCoord[i].dBX, vtMeasureCoord[i].dBY, vtMeasureCoord[i].dBZ,
			vtMeasurePulse[i].nSPulse, vtMeasurePulse[i].nLPulse, vtMeasurePulse[i].nUPulse,
			vtMeasurePulse[i].nRPulse, vtMeasurePulse[i].nBPulse, vtMeasurePulse[i].nTPulse,
			vtMeasurePulse[i].lBXPulse, vtMeasurePulse[i].lBYPulse, vtMeasurePulse[i].lBZPulse);
	}
	fclose(pf);

	// 离线调试使用
	CString sJobName;
	sJobName.Format("MOVJTEACHBOARD%d", nGroupNo);
	GenerateJob(vtMeasurePulse, MOVJ, sJobName);
	GenerateFilePLY(nGroupNo, 0.0); // 调试使用

	return true;
}

bool GrooveWeld::ScanEndpoint_G(int nGroupNo,int nLayerNo, int nCameraNo, vector<T_ANGLE_PULSE> vtPulse, vector<int> vnType, vector<T_TEACH_RESULT>& vtTeachResult)
{
	vtTeachResult.clear();
	m_pRobotDriver->m_vtWeldLineInWorldPoints.clear();
	E_SCANMODE eScanMode = E_SCANMODE_2DPOINT;
	T_ROBOT_COORS tCoord;
	vector<T_ROBOT_COORS> vtCoord(0);
	T_ROBOT_MOVE_INFO tRobotMoveInfo;
	vector<T_ROBOT_MOVE_INFO> vtRobotMoveInfo(0);
	for (int i = 0; i < vtPulse.size(); i++)
	{
		m_pRobotDriver->RobotKinematics(vtPulse[i], m_pRobotDriver->m_tTools.tGunTool, tCoord);
		vtCoord.push_back(tCoord);

		if (E_TRANSITION_POINT != vnType[i])
		{
			m_pRobotDriver->m_vtWeldLineInWorldPoints.push_back(tCoord);
		}
	}

	// 运动外部轴到安全位置 接头测试临时去除
	 m_ptUnit->MoveExAxleToSafe(&(vtPulse[0]));

	//// 运动外部轴到位
	//T_ANGLE_PULSE tPulse = m_pRobotDriver->GetCurrentPulse();
	//tPulse.lBXPulse = vtPulse[0].lBXPulse;
	//tPulse.lBYPulse = vtPulse[0].lBYPulse;
	//tPulse.lBZPulse = vtPulse[0].lBZPulse;
	//tRobotMoveInfo = m_pRobotDriver->PVarToRobotMoveInfo(0, tPulse, m_pRobotDriver->m_tExAxlePulseSpeed, MOVJ); 
	//vtRobotMoveInfo.push_back(tRobotMoveInfo);
	//m_pRobotDriver->SetMoveValue(vtRobotMoveInfo, false, true);
	//m_pRobotDriver->CallJob("CONTIMOVANY-BP");
	//m_ptUnit->RobotCheckDone();

	// 下枪运动到扫描起点位置 MOVJ MOVJ
	int i = 0;
	vtRobotMoveInfo.clear();
	for (i = 0; i < vnType.size(); i++)
	{
		if (E_TRANSITION_POINT != vnType[i])
		{
			tRobotMoveInfo = m_pRobotDriver->PVarToRobotMoveInfo(0, vtPulse[i], m_pRobotDriver->m_tPulseHighSpeed, MOVJ);
			vtRobotMoveInfo.push_back(tRobotMoveInfo);
			break;
		}
		tRobotMoveInfo = m_pRobotDriver->PVarToRobotMoveInfo(0, vtPulse[i], m_pRobotDriver->m_tPulseHighSpeed, MOVJ);
		vtRobotMoveInfo.push_back(tRobotMoveInfo);
	}
	vtRobotMoveInfo[0].tSpeed = m_pRobotDriver->m_tExAxlePulseSpeed;
	m_pRobotDriver->SetMoveValue(vtRobotMoveInfo, true, false);
	m_pRobotDriver->CallJob("CONTIMOVANY");
	//m_pRobotDriver->SetMoveValue(vtRobotMoveInfo, true, true);
	//m_pRobotDriver->CallJob("CONTIMOVANY-BP");
	m_ptUnit->RobotCheckDone();
	
	// 运动外部轴到位
	vtRobotMoveInfo.clear();
	T_ANGLE_PULSE tPulse = m_pRobotDriver->GetCurrentPulse();
	tPulse.lBXPulse = vtPulse[0].lBXPulse;
	tPulse.lBYPulse = vtPulse[0].lBYPulse;
	tPulse.lBZPulse = vtPulse[0].lBZPulse;
	tRobotMoveInfo = m_pRobotDriver->PVarToRobotMoveInfo(0, tPulse, m_pRobotDriver->m_tExAxlePulseSpeed, MOVJ); 
	vtRobotMoveInfo.push_back(tRobotMoveInfo);
	m_pRobotDriver->SetMoveValue(vtRobotMoveInfo, false, true);
	m_pRobotDriver->CallJob("CONTIMOVANY-BP");
	m_ptUnit->RobotCheckDone();

	E_DHGIGE_ACQUISITION_MODE eCameraMode = E_ACQUISITION_MODE_SOURCE_SOFTWARE;
	E_DHGIGE_CALL_BACK eCallBack = E_CALL_BACK_MODE_OFF;
	m_ptUnit->SwitchDHCamera(m_ptUnit->m_nMeasureCameraNo, true, true, eCameraMode, eCallBack); // 下枪时 开相机 激光
	m_ptUnit->m_vpImageCapture[m_ptUnit->m_nMeasureCameraNo]->StartAcquisition();
	m_ptUnit->RobotCheckDone();
	CHECK_PULSE_RETURN_BOOL(m_pRobotDriver, vtPulse[i]);

	E_FLIP_MODE eFlipMode = m_ptUnit->GetCameraParam(m_ptUnit->m_nMeasureCameraNo).eFlipMode;

	double dScanSpeed = 1500;
	vtRobotMoveInfo.clear();
	T_ROBOT_MOVE_SPEED tScanSpeed(dScanSpeed / 6, 100, 100);
	tRobotMoveInfo = m_pRobotDriver->PVarToRobotMoveInfo(0, vtCoord[1], tScanSpeed, MOVL);
	vtRobotMoveInfo.push_back(tRobotMoveInfo);
	tRobotMoveInfo = m_pRobotDriver->PVarToRobotMoveInfo(1, vtCoord[2], tScanSpeed, MOVL);
	vtRobotMoveInfo.push_back(tRobotMoveInfo);
	m_pRobotDriver->SetMoveValue(vtRobotMoveInfo, false, true);
	m_pScanInit->SetScanParam(TRUE == *m_pIsNaturalPop, eFlipMode, eScanMode, "CONTIMOVANY-BP");

	CString sPointFileName;
	sPointFileName.Format("Layer%d_Group%d_PointCloud.txt", nLayerNo, nGroupNo);
	if (!m_pScanInit->InitDynamicCapture_H_M(E_POINT_CLOUD_PROC_MOMENT_DYNAMIC, E_IMAGE_PROC_METHOD_DoubleVGroove, E_POINT_CLOUD_PROC_METHOD_DoubleVGroove,
		m_ptUnit->m_nMeasureCameraNo, OUTPUT_PATH + m_pRobotDriver->m_strRobotName + GROOVE_DATA_PATH, sPointFileName, 80, m_eWeldSeamType, m_nGroupNo, m_nLayerNo, true, true))
	{
		return false;
	}

	m_pScanInit->DynamicCaptureNew_H_M(m_pRobotDriver, m_ptUnit->m_nMeasureCameraNo, 5, 50);

	// 收枪运动 MOVL
	vtRobotMoveInfo.clear();
	tRobotMoveInfo = m_pRobotDriver->PVarToRobotMoveInfo(0, vtPulse.back(), m_pRobotDriver->m_tPulseLowSpeed, MOVJ);
	vtRobotMoveInfo.push_back(tRobotMoveInfo);
	m_pRobotDriver->SetMoveValue(vtRobotMoveInfo, false, true);
	m_pRobotDriver->CallJob("CONTIMOVANY-BP");
	m_ptUnit->SwitchDHCamera(m_ptUnit->m_nMeasureCameraNo, false); // 收枪时关闭相机和激光
	m_ptUnit->RobotCheckDone();
	CHECK_PULSE_RETURN_BOOL(m_pRobotDriver, vtPulse.back());
	CHECK_BOOL_RETURN(GetScanProcResult_G(nGroupNo, nLayerNo, vtCoord, vtTeachResult));
	return true;
}

bool GrooveWeld::GeneralGrooveWeldTrack_G(int nGroupNo)
{
	return true;
}

bool GrooveWeld::GeneralGrooveWeldTrack_G_First(int nGroupNo)
{
	return false;
}

bool GrooveWeld::GeneralGrooveWeldTrack_G_Other(int nGroupNo, int nLayerNo)
{
	//while (true)  // 每次测量后 调整第nGroupNo组焊缝，第nLayerNo及以后所有焊接轨迹 最后几层无法测量 使用最后一次调整后焊接轨迹焊接
	//{
	//// 加载第nGroupNo组 第nLayerNo道 焊接轨迹 vtWeldTrack
	//	E_WELD_SEAM_TYPE eWeldSeamType;
	//	double dExAxlePos = 0.0;
	//	std::vector<T_ROBOT_COORS> vtWeldTrack(0);
	//	std::vector<int> vnPtnType(0);
	//	CString sFileName;
	//	CString sBackUpFileName;
	//	sFileName.Format("%s%d_%d_RealWeldCoord.txt", m_sDataSavePath, nGroupNo, nLayerNo);
	//	sBackUpFileName.Format("%s%d_%d_RealWeldCoord_SrcBackUp.txt", m_sDataSavePath, nGroupNo, nLayerNo);
	//	bool bExistBackUp = CheckFileExists(sBackUpFileName, false);
	//	CString sReadFileName = bExistBackUp ? sBackUpFileName : sFileName; // 保证加载的是第一次测量的原始轨迹 避免重复修正
	//	if (false == LoadRealWeldTrack_G(sReadFileName, eWeldSeamType, dExAxlePos, vtWeldTrack, vnPtnType))
	//	{
	//		m_pRobotDriver->m_cLog->Write("%s 第%d道 第%d组 焊缝轨迹调整加载轨迹失败！轨迹调整结束", m_pRobotDriver->m_strRobotName, nLayerNo + 1, nGroupNo + 1);
	//		break;
	//	}
	//	for (int i = 0; i < vtWeldTrack.size(); i++) // 合并外部轴坐标道xyz
	//	{
	//		vtWeldTrack[i].dX += vtWeldTrack[i].dBX; vtWeldTrack[i].dBX = 0.0;
	//		vtWeldTrack[i].dY += vtWeldTrack[i].dBY; vtWeldTrack[i].dBY = 0.0;
	//		vtWeldTrack[i].dZ += vtWeldTrack[i].dBZ; vtWeldTrack[i].dBZ = 0.0;
	//	}

	//	// 加载第nGroupNo组 首次测量结果 FirstMeasureResult
	//	std::vector<T_TEACH_RESULT> vtFitstTeachResult(0);
	//	LoadTeachResult(nGroupNo, 0, vtFitstTeachResult);

	//	// 将坡口原始测量结果恢复成 GroovePointInfo

	//	std::vector<GroovePointInfo> vtGroovePtnInfoBase(vtFitstTeachResult[0].vtLeftPtns3D.size() / 5);
	//	for (int i = 5; i <= vtFitstTeachResult[0].vtLeftPtns3D.size(); i += 5)
	//	{
	//		Trans3D(vtFitstTeachResult[0].vtLeftPtns3D[i - 5], vtGroovePtnInfoBase[i / 5 - 1].GrooveTopStaPnt);
	//		Trans3D(vtFitstTeachResult[0].vtLeftPtns3D[i - 4], vtGroovePtnInfoBase[i / 5 - 1].GroovePntStaPnt);
	//		Trans3D(vtFitstTeachResult[0].vtLeftPtns3D[i - 3], vtGroovePtnInfoBase[i / 5 - 1].BottomCenterStaPnt);
	//		Trans3D(vtFitstTeachResult[0].vtLeftPtns3D[i - 2], vtGroovePtnInfoBase[i / 5 - 1].FlatTopPntStaPnt);
	//		Trans3D(vtFitstTeachResult[0].vtLeftPtns3D[i - 1], vtGroovePtnInfoBase[i / 5 - 1].FlatBottomStaPnt);
	//	}

	//	// 将坡口最新测量结果恢复成 GroovePointInfo
	//	std::vector<GroovePointInfo> vtGroovePtnInfoNew(m_vtTeachResult[0].vtLeftPtns3D.size() / 5);
	//	for (int i = 5; i <= m_vtTeachResult[0].vtLeftPtns3D.size(); i += 5)
	//	{
	//		Trans3D(m_vtTeachResult[0].vtLeftPtns3D[i - 5], vtGroovePtnInfoNew[i / 5 - 1].GrooveTopStaPnt);
	//		Trans3D(m_vtTeachResult[0].vtLeftPtns3D[i - 4], vtGroovePtnInfoNew[i / 5 - 1].GroovePntStaPnt);
	//		Trans3D(m_vtTeachResult[0].vtLeftPtns3D[i - 3], vtGroovePtnInfoNew[i / 5 - 1].BottomCenterStaPnt);
	//		Trans3D(m_vtTeachResult[0].vtLeftPtns3D[i - 2], vtGroovePtnInfoNew[i / 5 - 1].FlatTopPntStaPnt);
	//		Trans3D(m_vtTeachResult[0].vtLeftPtns3D[i - 1], vtGroovePtnInfoNew[i / 5 - 1].FlatBottomStaPnt);
	//	}

	//	// 摆动焊接轨迹修正
	//	//CHECK_BOOL_RETURN(AdjustWaveWeldPath(vtGroovePtnInfoBase, vtGroovePtnInfoNew, vtWeldTrack));

	//	if (false == DecomposeCoordinate(vtWeldTrack, m_tFixCoord, m_ptUnit->m_nMeasureAxisNo))
	//	{
	//		m_pRobotDriver->m_cLog->Write("GereralGrooveWeldTrack:外部轴坐标拆分计算失败!");
	//		return false;
	//	}

	//	// 备份调整前轨迹 并 保存新轨迹
	//	if (false == bExistBackUp) // 确保仅备份第一次测量的原始轨迹
	//	{
	//		CopyFile((LPCTSTR)sFileName, (LPCTSTR)sBackUpFileName, FALSE);
	//	}
	//	// 保存调整后的 第nGroupNo组 第nLayerNo道 焊接轨迹 vtWeldTrack
	//	FILE* pf = fopen(sFileName, "w");
	//	for (int nPtnIdx = 0; nPtnIdx < vtWeldTrack.size(); nPtnIdx++)
	//	{
	//		T_ROBOT_COORS& tCoord = vtWeldTrack[nPtnIdx];
	//		fprintf(pf, "%d%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%4d%11.3lf\n", nPtnIdx,
	//			tCoord.dX, tCoord.dY, tCoord.dZ, tCoord.dRX, tCoord.dRY, tCoord.dRZ, tCoord.dBX, tCoord.dBY, tCoord.dBZ,
	//			m_vdGrooveWeldSpeedRate[nPtnIdx], eWeldSeamType, m_vdWaveWaitTime[nPtnIdx]);
	//	}
	//	fclose(pf);
	//	nLayerNo++;
	//}
	return true;
}

//bool GrooveWeld::AdjustWaveWeldPath(const std::vector<GroovePointInfo>& vtInfoBase, const std::vector<GroovePointInfo>& vtInfoNew, vector<T_ROBOT_COORS>& vtCoord)
//{
//	
//	return true;
//}

//double GrooveWeld::CalcUpperWidth(XI_POINT p, int nBasePtnNo, const std::vector<GroovePointInfo>& vtInfoBase)
//{
//	return 0.0;
//}

XI_POINT GrooveWeld::MidPtn(CvPoint3D32f p1, CvPoint3D32f p2)
{
	XI_POINT tPtn;
	tPtn.x = (p1.x + p2.x) / 2.0;
	tPtn.y = (p1.y + p2.y) / 2.0;
	tPtn.z = (p1.z + p2.z) / 2.0;
	return tPtn;
}

bool GrooveWeld::DoWelding_G(int nGroupNo, int nWeldNo, E_WELD_SEAM_TYPE eWeldSeamType, std::vector<T_ROBOT_COORS>& vtWeldPathPoints,
	const vector<int>& vnPtnType, const T_WELD_PARA& tWeldPara)
{
	long long lTimeS = XI_clock();
	vector<T_ANGLE_PULSE> vtAccurateWeldPulse;
	vector<T_ROBOT_COORS> vtAccurateWeldCoord; // 焊接轨迹 + 下枪收枪坐标
	T_ANGLE_PULSE tDownGunPulse;
	T_ANGLE_PULSE tBackGunPulse;
	T_ROBOT_COORS tDownCoord; // 下枪坐标
	T_ROBOT_COORS tBackCoord; // 收枪坐标
	T_ROBOT_MOVE_INFO tRobotMoveInfo;
	vector<T_ROBOT_MOVE_INFO> vtRobotMoveInfo;

	CRobotDriverAdaptor* pRobotDriver = m_pRobotDriver;
	int nRobotNo = pRobotDriver->m_nRobotNo;
	int nTotalPointNum = vtWeldPathPoints.size();
	if (nTotalPointNum < 10)
	{
		XUI::MesBox::PopError("焊接轨迹数据点数{0} 过少，焊接失败！", nTotalPointNum);
		return false;
	}
	// 下枪
	XiAlgorithm alg;
	LineOrCircularArcWeldingLine tWeldLine = m_vvtWeldLineInfoGroup[nGroupNo][0].tWeldLine;
	double dNorAngle = alg.CalcArcAngle(tWeldLine.StartNormalVector.x, tWeldLine.StartNormalVector.y);
	double dH1 = tWeldLine.StartPoint.z * m_nRobotInstallDir;
	double dH2 = tWeldLine.EndPoint.z * m_nRobotInstallDir;
	double dMaxHeight = dH1 > dH2 ? dH1 * m_nRobotInstallDir : dH2 * m_nRobotInstallDir;

	tDownCoord = vtWeldPathPoints[0];
	//tDownCoord.dX += (m_dGunDownBackSafeDis / 2 * CosD(dNorAngle/*RzToDirAngle(tDownCoord.dRZ)*/));
	//tDownCoord.dY += (m_dGunDownBackSafeDis / 2 * SinD(dNorAngle/*RzToDirAngle(tDownCoord.dRZ)*/));
	if (E_STAND_GROOVE ==  m_eWeldSeamType)
	{
		tDownCoord.dY += m_dGunDownBackSafeDis;
	}
	tDownCoord.dZ += (/*dMaxHeight - tDownCoord.dBZ + */m_dGunDownBackSafeDis * ((double)m_nRobotInstallDir));
	vtAccurateWeldCoord.push_back(tDownCoord);

	vtAccurateWeldCoord.insert(vtAccurateWeldCoord.end(), vtWeldPathPoints.begin(), vtWeldPathPoints.end());

	tBackCoord = vtWeldPathPoints[vtWeldPathPoints.size() - 1];
	//tBackCoord.dX += (m_dGunDownBackSafeDis / 2 * CosD(dNorAngle/*RzToDirAngle(tBackCoord.dRZ)*/));
	//tBackCoord.dY += (m_dGunDownBackSafeDis / 2 * SinD(dNorAngle/*RzToDirAngle(tBackCoord.dRZ)*/));
	if (E_STAND_GROOVE == m_eWeldSeamType)
	{
		tBackCoord.dY += m_dGunDownBackSafeDis;
	}
	tBackCoord.dZ += (/*dMaxHeight - tBackCoord.dBZ + */m_dGunDownBackSafeDis * ((double)m_nRobotInstallDir));
	vtAccurateWeldCoord.push_back(tBackCoord);

	if (false == CalcContinuePulseForWeld(vtAccurateWeldCoord, vtAccurateWeldPulse))
	{
		bool bCalcRst = false;
		bool bAddorSubtract = false;//true加false减
		int nIndex = 1;
		while (true) {
			vector<T_ROBOT_COORS> vtTempWeldCoord(vtAccurateWeldCoord); // 焊接轨迹 + 下枪收枪坐标

			bCalcRst = CalcContinuePulseForWeld(vtTempWeldCoord, vtAccurateWeldPulse);
			if (true == bCalcRst)
			{
				vtWeldPathPoints.clear();
				vtWeldPathPoints.insert(vtWeldPathPoints.begin(), vtTempWeldCoord.begin() + 1, vtTempWeldCoord.end() - 1);
				tBackCoord = vtTempWeldCoord[vtTempWeldCoord.size() - 1];
				break;
			}
		}
		if (false == bCalcRst)
		{
			XiMessageBox("多外部轴位置，焊接下枪过渡点计算失败！");
			return false;
		}
	}

	// 接头测试临时去除
	m_ptUnit->MoveExAxleToSafe(&(vtAccurateWeldPulse[0]));

	int nAxisNo = m_ptUnit->m_nMeasureAxisNo;
	tDownGunPulse = vtAccurateWeldPulse[0];
	vtRobotMoveInfo.clear();

	tRobotMoveInfo = m_pRobotDriver->PVarToRobotMoveInfo(0, tDownGunPulse, m_pRobotDriver->m_tPulseHighSpeed, MOVJ);
	vtRobotMoveInfo.push_back(tRobotMoveInfo);

	tRobotMoveInfo = m_pRobotDriver->PVarToRobotMoveInfo(1, vtWeldPathPoints[0], m_pRobotDriver->m_tCoordLowSpeed, MOVL);
	vtRobotMoveInfo.push_back(tRobotMoveInfo);

	if (false == m_bIsLocalDebug) // 下枪
	{
		// 下枪运动 ！！！！
		vtRobotMoveInfo[0].tSpeed = m_pRobotDriver->m_tExAxlePulseSpeed;
		//m_pRobotDriver->SetMoveValue(vtRobotMoveInfo, true, true);
		//m_pRobotDriver->CallJob("CONTIMOVANY-BP");
		m_pRobotDriver->SetMoveValue(vtRobotMoveInfo, true, false);
		m_pRobotDriver->CallJob("CONTIMOVANY");
		m_ptUnit->RobotCheckDone();
		if (false == m_pRobotDriver->CompareCoords(vtWeldPathPoints[0], m_pRobotDriver->GetCurrentPos()))
		{
			WriteLog("Check Coors Err1");
			return false;
		}; // 与最后一个下枪坐标有关
	}

	// 运动外部轴到位
	vtRobotMoveInfo.clear();
	T_ANGLE_PULSE tPulse = m_pRobotDriver->GetCurrentPulse();
	tPulse.lBXPulse = vtAccurateWeldPulse[0].lBXPulse;
	tPulse.lBYPulse = vtAccurateWeldPulse[0].lBYPulse;
	tPulse.lBZPulse = vtAccurateWeldPulse[0].lBZPulse;
	tRobotMoveInfo = m_pRobotDriver->PVarToRobotMoveInfo(0, tPulse, m_pRobotDriver->m_tExAxlePulseSpeed, MOVJ);
	vtRobotMoveInfo.push_back(tRobotMoveInfo);
	m_pRobotDriver->SetMoveValue(vtRobotMoveInfo, false, true);
	m_pRobotDriver->CallJob("CONTIMOVANY-BP");
	m_ptUnit->RobotCheckDone();

	WriteLog("下枪完成");
	if (false == m_bIsLocalDebug) // 焊接
	{
		// 外部轴不动 只有机器人6轴运动
		//if (0 > m_ptUnit->WeldMove(vtWeldPathPoints, vnPtnType, tWeldPara, dRealWeldExPos, eWeldSeamType, TRUE == *m_pIsArcOn))
		//{
		//	return false;
		//}

		// 机器人外部轴联动
		//if (0 > m_ptUnit->WeldMove_BP(vtWeldPathPoints, vnPtnType, tWeldPara, eWeldSeamType, TRUE == *m_pIsArcOn))
		//{
		//	return false;
		//}
		if (0 > m_ptUnit->GrooveWeldMove(vtWeldPathPoints, tWeldPara, 0.0, TRUE == *m_pIsArcOn, tWeldPara.tGrooveWaveParam, m_vdGrooveWeldSpeedRate, m_vdWaveWaitTime))
		{
			return false;
		}

		//double dTotalVal = tWeldPara.WeldVelocity/* 800*/; //焊接速度 mm/min
		//m_ptUnit->m_bSwingState = true;
		//int SwingNumber = tWeldPara.nWrapConditionNo/*666*/;
		//int nLayerNo = tWeldPara.nLayerNo;
		//m_ptUnit->LoadWeldParam(SwingNumber, nLayerNo);
		////m_ptUnit->WriteSwingParam(2, 1, 1);
		//m_ptUnit->SwingTracking(pRobotDriver, vtWeldPathPoints, dTotalVal);

		m_ptUnit->RobotCheckDone();	
		//CHECK_COORS_RETURN_BOOL(m_pRobotDriver, vtWeldPathPoints[vtWeldPathPoints.size() - 1]);
		CHECK_COORS_RETURN_BOOL(m_pRobotDriver, m_pRobotDriver->m_vtWeldLineInWorldPoints.back());
	}
	WriteLog("焊接完成");
	Sleep(5000); // 

	vtRobotMoveInfo.clear();
	tRobotMoveInfo = m_pRobotDriver->PVarToRobotMoveInfo(0, tBackCoord, m_pRobotDriver->m_tCoordLowSpeed, MOVL);
	vtRobotMoveInfo.push_back(tRobotMoveInfo);
	if (false == m_bIsLocalDebug) // 收枪
	{
		m_pRobotDriver->SetMoveValue(vtRobotMoveInfo, false, true);
		m_pRobotDriver->CallJob("CONTIMOVANY-BP");
		m_ptUnit->RobotCheckDone();
		CHECK_COORS_RETURN_BOOL(m_pRobotDriver, tBackCoord);
	}
	WriteLog("收枪完成");

	//long long lWeldTime = XI_clock() - lTimeS; //一条焊道的时间
	//double dWeldLenOld = (double)nTotalPointNum / 1000.0; // 单位米
	//LineOrCircularArcWeldingLine tWeld = m_vvtWeldSeamGroupAdjust[nGroupNo][nWeldNo];
	////一条焊道的长度
	//double dWeldLen = TwoPointDis(tWeld.StartPoint.x, tWeld.StartPoint.y, tWeld.StartPoint.z, tWeld.EndPoint.x, tWeld.EndPoint.y, tWeld.EndPoint.z) / 1000.0;
	//CTime time = CTime::GetCurrentTime();
	//CString sProductDataFileName;
	//sProductDataFileName.Format("Monitor\\%s\\ProductData_%4d.%.2d.%.2d.csv", m_pRobotDriver->m_strRobotName, time.GetYear(), time.GetMonth(), time.GetDay());
	//FILE* RecordTheoryData;
	//OpenProductLog(&RecordTheoryData, sProductDataFileName);
	//SaveProductData(RecordTheoryData, dWeldLen, lWeldTime, nGroupNo);
	//CloseProductLog(RecordTheoryData);
	//WriteLog("焊缝长度统计：Old:%.3lf New:%.3lf", dWeldLenOld, dWeldLen);
	//m_dCurWeldLenBeforeCleanGun += dWeldLen;
	//if (m_dCurWeldLenBeforeCleanGun > m_dCleanGunDis) // 大于3米进行一次清枪
	//{
	//	//CHECK_BOOL_RETURN(m_pRobotDriver->CleanGunH());
	//	m_dCurWeldLenBeforeCleanGun = 0.0;
	//}

	// 生成焊接轨迹JOB 测试用
	CString sJobName;
	sJobName.Format("DOWELDING%d-%d-%d", nGroupNo, nWeldNo, tWeldPara.nLayerNo);
	GenerateJob(vtAccurateWeldPulse, MOVL, sJobName);
	//GenerateJobCoord(vtAccurateWeldCoord, MOVL, sJobName);

	return true;
}

bool GrooveWeld::LoadRealWeldTrack_G(CString sWeldTrackFileName, E_WELD_SEAM_TYPE& eWeldSeamType, double& dExAxlePos, vector<T_ROBOT_COORS>& vtRealWeldTrack, vector<int>& vnPtnType)
{
	int nDataColNum = 13;
	m_vdGrooveWeldSpeedRate.clear();
	m_vdWaveWaitTime.clear();
	vtRealWeldTrack.clear();
	vnPtnType.clear();
	CString sFileName = sWeldTrackFileName;
	//sFileName.Format("%s%d_%d_RealWeldCoord.txt", m_sDataSavePath.GetBuffer(), nGroupNo, nWeldNo);
	FILE* pf = fopen(sFileName.GetBuffer(), "r");
	if (NULL == pf)
	{
		//XiMessageBox("加载第%d组 第%d个焊缝轨迹失败", nGroupNo, nWeldNo);
		return false;
	}
	// 序号 直角坐标 外部轴坐标 焊接类型
	int nIdx = 0;
	int nWeldSeamType = 0;
	T_ROBOT_COORS tCoord;
	double dWaveWaitTime = 0;
	double dSpeedRate = 0.0;
	int nRst = fscanf(pf, "%d%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%d%lf", &nIdx,
		&tCoord.dX, &tCoord.dY, &tCoord.dZ, &tCoord.dRX, &tCoord.dRY, &tCoord.dRZ,
		&tCoord.dBX, &tCoord.dBY, &tCoord.dBZ, &dSpeedRate, &nWeldSeamType, &dWaveWaitTime);
	while (nDataColNum == nRst)
	{
		m_vdGrooveWeldSpeedRate.push_back(dSpeedRate);
		m_vdWaveWaitTime.push_back(dWaveWaitTime);
		vtRealWeldTrack.push_back(tCoord);
		vnPtnType.push_back(E_WELD_TRACK);
		nRst = fscanf(pf, "%d%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%d%lf", &nIdx,
			&tCoord.dX, &tCoord.dY, &tCoord.dZ, &tCoord.dRX, &tCoord.dRY, &tCoord.dRZ,
			&tCoord.dBX, &tCoord.dBY, &tCoord.dBZ, &dSpeedRate, &nWeldSeamType, &dWaveWaitTime);
	}
	T_ROBOT_COORS tS = vtRealWeldTrack.front();
	T_ROBOT_COORS tE = vtRealWeldTrack.back();
	double dDis = TwoPointDis(tS.dX + tS.dBX, tS.dY + tS.dBY, tS.dZ + tS.dBZ, tE.dX + tE.dBX, tE.dY + tE.dBY, tE.dZ + tE.dBZ);
	double dDisZ = fabs(tS.dZ + tS.dBZ - tE.dZ - tE.dBZ);
	eWeldSeamType = dDisZ / dDis > 0.5 ? E_STAND_GROOVE : E_PLAT_GROOVE;

	if ((EOF != nRst) && (nDataColNum != nRst))
	{
		//已修改
		XUI::MesBox::PopInfo("加载{0}焊缝轨迹 检测到不完整数据", sWeldTrackFileName.GetBuffer());
		//XiMessageBox("加载%s焊缝轨迹 检测到不完整数据", sWeldTrackFileName);
		return false;
	}
	fclose(pf);
	return true;
}

void GrooveWeld::SaveCoordExAxleProcess(std::vector<T_ROBOT_COORS>& vtCoord, std::vector<int> vnType)
{
	for (int i = 0; i < vnType.size(); i++) // 下枪过渡点
	{
		if (E_TRANSITION_POINT != vnType[i])
		{
			T_ROBOT_COORS tCoord = vtCoord[i];
			for (int j = 0; j < i; j++)
			{
				vtCoord[j].dX = vtCoord[j].dX + vtCoord[j].dBX - tCoord.dBX; vtCoord[j].dBX = tCoord.dBX;
				vtCoord[j].dY = vtCoord[j].dY + vtCoord[j].dBY - tCoord.dBY; vtCoord[j].dBY = tCoord.dBY;
				vtCoord[j].dZ = vtCoord[j].dZ + vtCoord[j].dBZ - tCoord.dBZ; vtCoord[j].dBZ = tCoord.dBZ;
			}
			break;
		}
	}
	for (int i = vnType.size() - 1; i >= 0; i--) // 收枪过渡点
	{
		if (E_TRANSITION_POINT != vnType[i])
		{
			T_ROBOT_COORS tCoord = vtCoord[i];
			for (int j = vnType.size() - 1; j > i; j--)
			{
				vtCoord[j].dX = vtCoord[j].dX + vtCoord[j].dBX - tCoord.dBX; vtCoord[j].dBX = tCoord.dBX;
				vtCoord[j].dY = vtCoord[j].dY + vtCoord[j].dBY - tCoord.dBY; vtCoord[j].dBY = tCoord.dBY;
				vtCoord[j].dZ = vtCoord[j].dZ + vtCoord[j].dBZ - tCoord.dBZ; vtCoord[j].dBZ = tCoord.dBZ;
			}
			break;
		}
	}
}

void GrooveWeld::GenerateJob(vector<T_ANGLE_PULSE> vtRobotPulse, int nMoveType, CString sJobName)
{
	double dPulseLBY = 0.00766967710464; // 调试仿真使用转换
	double dPulseLBZ = 0.00766967685128; // 调试仿真使用转换
	int nPtnNum = vtRobotPulse.size();
	if (nPtnNum <= 0) return;

	CString sMoveCommand = nMoveType == MOVL ? "MOVL" : "MOVJ";
	CString sSpeedName = nMoveType == MOVL ? "V" : "VJ";
	CString sSavePathName;
	sSavePathName.Format(OUTPUT_PATH + m_ptUnit->GetUnitName() + "\\JOB\\%s.JBI", sJobName);
	FILE* pf = fopen(sSavePathName, "w");

	fprintf(pf, "/JOB\n");
	fprintf(pf, "//NAME " + sJobName + "\n");
	fprintf(pf, "//POS\n");
	fprintf(pf, "///NPOS %d,%d,0,0,0,0\n", nPtnNum, nPtnNum);
	fprintf(pf, "///TOOL 1\n");
	fprintf(pf, "///POSTYPE PULSE\n");
	fprintf(pf, "///PULSE\n");
	for (int i = 0; i < nPtnNum; i++)
	{
		fprintf(pf, "C%05d=%ld,%ld,%ld,%ld,%ld,%ld\n", i,
			vtRobotPulse[i].nSPulse, vtRobotPulse[i].nLPulse, vtRobotPulse[i].nUPulse,
			vtRobotPulse[i].nRPulse, vtRobotPulse[i].nBPulse, vtRobotPulse[i].nTPulse);
	}
	fprintf(pf, "///TOOL 1\n");
	for (int i = 0; i < nPtnNum; i++)
	{
		fprintf(pf, "BC%05d=%ld,%ld\n", i, 
			(long)((double)vtRobotPulse[i].lBYPulse * m_pRobotDriver->m_tExternalAxle[1].dPulse / dPulseLBY),
			(long)((double)vtRobotPulse[i].lBZPulse * m_pRobotDriver->m_tExternalAxle[2].dPulse / dPulseLBZ));
	}
	fprintf(pf, "//INST\n");
	fprintf(pf, "///DATE 2021/03/10 17:07\n");
	fprintf(pf, "///ATTR SC,RW\n");
	fprintf(pf, "///GROUP1 RB1,BS1\n");
	fprintf(pf, "NOP\n");
	for (int i = 0; i < nPtnNum; i++)
	{
		if (0 == i || (i == nPtnNum - 1))
		{
			fprintf(pf, "MOVJ C%05d BC%05d VJ=I004\n", i, i);
		}
		else
		{
			fprintf(pf, sMoveCommand + " C%05d  BC%05d " + sSpeedName + "=I005\n", i, i);
		}
	}
	fprintf(pf, "END\n");
	fclose(pf);
	SaveToJobBuffer(vtRobotPulse);
}

int GrooveWeld::GetGroovePara(const T_ROBOT_COORS& tRobotStartCoord, const T_ROBOT_COORS& tRobotEndCoord, vector<T_WAVE_PARA>& vtTWavePara)
{
	bool isVerticalWeld = false;//是否立焊
	double disEndToStart = sqrt(pow((tRobotEndCoord.dX - tRobotStartCoord.dX), 2) + pow((tRobotEndCoord.dY - tRobotStartCoord.dY), 2) + pow((tRobotEndCoord.dZ - tRobotStartCoord.dZ), 2));
	double vecX = (tRobotEndCoord.dX - tRobotStartCoord.dX) / disEndToStart;
	double vecY = (tRobotEndCoord.dY - tRobotStartCoord.dY) / disEndToStart;
	double vecZ = (tRobotEndCoord.dZ - tRobotStartCoord.dZ) / disEndToStart;
	if (abs(vecZ) > 0.5)
	{
		isVerticalWeld = true;
	}
	if (isVerticalWeld)
	{
		GetVerGroovePara(vtTWavePara);
	}
	else
	{
		GetFlatGroovePara(vtTWavePara);
	}
	return 0;
}

int GrooveWeld::GetFlatGroovePara(vector<T_WAVE_PARA>& vtTWavePara)
{
	vtTWavePara.clear();
	ChangeGroovePara  GroovePara;
	vtTWavePara = GroovePara.vtFlatWavePara;
	return 0;
}

int GrooveWeld::GetVerGroovePara(vector<T_WAVE_PARA>& vtTWavePara)
{
	vtTWavePara.clear();
	ChangeGroovePara  GroovePara;
	vtTWavePara = GroovePara.vtVerWavePara;
	return 0;
}

int GrooveWeld::GetTeachPos(T_ROBOT_COORS& tRobotStartCoord, T_ROBOT_COORS& tRobotEndCoord, int nRobotNo)
{
	//获取示教的点位
	double pTeach[2][6] = { 0 };
	UINT* pToolNo = new UINT;
	UINT* pUserNo = new UINT;
	UINT* pPosture = new UINT;
	m_pRobotDriver->GetMultiPosVar(2, 111, pTeach, pToolNo, pUserNo, pPosture);

	//确定起点位置
	tRobotStartCoord.dX = pTeach[0][0];
	tRobotStartCoord.dY = pTeach[0][1];
	tRobotStartCoord.dZ = pTeach[0][2];
	tRobotStartCoord.dRX = pTeach[0][3];
	tRobotStartCoord.dRY = pTeach[0][4];
	tRobotStartCoord.dRZ = pTeach[0][5];

	//确定终点位置
	tRobotEndCoord.dX = pTeach[1][0];
	tRobotEndCoord.dY = pTeach[1][1];
	tRobotEndCoord.dZ = pTeach[1][2];
	tRobotEndCoord.dRX = pTeach[1][3];
	tRobotEndCoord.dRY = pTeach[1][4];
	tRobotEndCoord.dRZ = pTeach[1][5];
	return 0;
}

int GrooveWeld::CalWavePath(T_GROOVE_INFOR tGrooveInfor, T_INFOR_WAVE_RAND tGrooveRand, T_WAVE_PARA tWavePara, int nLayerNo, vector<T_ROBOT_COORS>& vtGrooveWavePath, vector<double>& weldSpeedRate, vector<double>& vdWaveWaitTime)
{
	T_ROBOT_COORS tRobotStartCoord = tGrooveRand.tStartPoint;
	T_ROBOT_COORS tRobotEndCoord = tGrooveRand.tEndPoint;
	bool isVerticalWeld = false;//是否立焊
	vtGrooveWavePath.clear();
	weldSpeedRate.clear();
	vdWaveWaitTime.clear();
	double disEndToStart = sqrt(pow((tRobotEndCoord.dX - tRobotStartCoord.dX), 2) + pow((tRobotEndCoord.dY - tRobotStartCoord.dY), 2) + pow((tRobotEndCoord.dZ - tRobotStartCoord.dZ), 2));
	double vecX = (tRobotEndCoord.dX - tRobotStartCoord.dX) / disEndToStart;
	double vecY = (tRobotEndCoord.dY - tRobotStartCoord.dY) / disEndToStart;
	double vecZ = (tRobotEndCoord.dZ - tRobotStartCoord.dZ) / disEndToStart;
	if (abs(vecZ) > 0.5)
	{
		isVerticalWeld = true;
	}
	T_ROBOT_COORS tmpCoor = tRobotStartCoord;
	//计算起点至终点距离以及方向向量
	
	//确定姿态以及插入起点以及起点速率
	
	//if (!isVerticalWeld)
	//{
	//	tmpCoor.dRZ = atan2(vecY, vecX) * 180 / PI;
	//	/*tmpCoor.dRY = 0;*/
	//	tmpCoor.dRX = 0 - tGrooveRand.dAngleOffset;
	//	/*if (tGrooveRand.dAngleOffset > 0)
	//	{
	//		tmpCoor.dRX -= tGrooveRand.dAngleOffset;
	//	}
	//	else if (tGrooveRand.dAngleOffset < 0)
	//	{
	//		tmpCoor.dRX = -180 - tGrooveRand.dAngleOffset;
	//	}*/
	//}
	//else
	//{
	//	tmpCoor.dRX = 0.0;
	//	tmpCoor.dRZ += tGrooveRand.dAngleOffset;
	//	if (tmpCoor.dRZ > 180)
	//	{
	//		tmpCoor.dRZ = tmpCoor.dRZ - 360;
	//	}
	//	if (tmpCoor.dRZ < -180)
	//	{
	//		tmpCoor.dRZ = tmpCoor.dRZ + 360;
	//	}
	//}
	tmpCoor.dRX = m_dGrooveWeldRx;
	tmpCoor.dRY = m_dGrooveWeldRy;
	tmpCoor.dRZ = m_dGrooveWeldRz;
	vtGrooveWavePath.push_back(tmpCoor);
	weldSpeedRate.push_back(tGrooveRand.dStartSpeedRate);
	vdWaveWaitTime.push_back(tGrooveRand.dStartWaveWaitTime);

	double vecOnlyX;
	double vecOnlyY;
	double vecXAntiClockWise;//计算摆弧偏移方向向量逆时针旋转
	double vecYAntiClockWise;
	double vecXClockWise;//计算摆弧偏移方向向量顺时针旋转
	double vecYClockWise;
	if (isVerticalWeld)
	{
		//计算摆弧点偏移向量,焊枪角度为焊道方向
		double weldAngle = PI * tGrooveInfor.weldAngle / 180;
		vecOnlyX = cos(weldAngle);
		vecOnlyY = sin(weldAngle);		//计算摆弧偏移方向向量逆时针旋转
		//////vecXAntiClockWise = 1.0;
		//////vecYAntiClockWise = 0.0;
		////////计算摆弧偏移方向向量顺时针旋转
		//////vecXClockWise = -1.0;
		//////vecYClockWise = 0.0;
	}
	else
	{
		////计算摆弧偏移方向向量旋转前（不考虑Z方向）
		double disXY = sqrt(pow((tRobotEndCoord.dX - tRobotStartCoord.dX), 2) + pow((tRobotEndCoord.dY - tRobotStartCoord.dY), 2));
		vecOnlyX = (tRobotEndCoord.dX - tRobotStartCoord.dX) / disXY;
		vecOnlyY = (tRobotEndCoord.dY - tRobotStartCoord.dY) / disXY;
	}
	//计算摆弧偏移方向向量逆时针旋转
	vecXAntiClockWise = vecOnlyX * cos(PI / 2) - vecOnlyY * sin(PI / 2);
	vecYAntiClockWise = vecOnlyX * sin(PI / 2) - vecOnlyY * cos(PI / 2);
	//计算摆弧偏移方向向量顺时针旋转
	vecXClockWise = vecOnlyX * cos(-PI / 2) - vecOnlyY * sin(-PI / 2);
	vecYClockWise = vecOnlyX * sin(-PI / 2) - vecOnlyY * cos(-PI / 2);

	// 自动计算摆动幅度临时补偿去除
	double dWaveComp = 0.0; // -0.5; -1.0;
	tWavePara.dStartWave = tGrooveRand.dStartWaveDis + dWaveComp;
	tWavePara.dEndWave = tGrooveRand.dEndWaveDis + dWaveComp;


	double disWaveEndtoStart = tWavePara.dEndWave - tWavePara.dStartWave;//起点终点摆幅差
	/*double waveAngle = tWavePara.dWaveAngle * PI / 180;*/
	double disWave = tWavePara.dStartWave; //每段摆幅度
	/*double disWaveRunVec = disWave * cos(waveAngle) / sin(waveAngle);*/
	double sunDisRunVec = tWavePara.dWaveDistance;//记录沿焊缝方向距离
	int totalWave = disEndToStart / tWavePara.dWaveDistance;//初步计算摆幅总段数
	double surplusWave = disEndToStart - totalWave * tWavePara.dWaveDistance;//确定剩余摆弧余量
	//确定摆弧总段数
	if (surplusWave > (tWavePara.dWaveDistance / 2))
	{
		totalWave++;
	}
	if (!isVerticalWeld || (isVerticalWeld && 0 == tWavePara.waveType))
	{
		//确定摆幅固定增加变量
		double waveChangeValue = disWaveEndtoStart / totalWave;
		//插入摆弧的第一个摆幅点
		tmpCoor.dX = tRobotStartCoord.dX + (sunDisRunVec - tWavePara.dWaveDistance / 2) * vecX;
		tmpCoor.dY = tRobotStartCoord.dY + (sunDisRunVec - tWavePara.dWaveDistance / 2) * vecY;
		tmpCoor.dZ = tRobotStartCoord.dZ + (sunDisRunVec - tWavePara.dWaveDistance / 2) * vecZ;
		tmpCoor.dX = tmpCoor.dX + disWave * vecXAntiClockWise;
		tmpCoor.dY = tmpCoor.dY + disWave * vecYAntiClockWise;
		if (0 == nLayerNo) tmpCoor.dX -= 2.0; // 第一道靠近坡口的起弧点向坡口面偏
		vtGrooveWavePath.push_back(tmpCoor);
		weldSpeedRate.push_back(tGrooveRand.dStartSpeedRate);
		vdWaveWaitTime.push_back(tGrooveRand.dStartWaveWaitTime);

		for (int waveNum = 1; waveNum < totalWave; waveNum++)
		{
			sunDisRunVec += tWavePara.dWaveDistance;
			disWave += waveChangeValue;

			tWavePara.dEndSpeedRate = tGrooveRand.dEndSpeedRate;
			if (fabs(tWavePara.dEndWave - tWavePara.dStartWave) < 0.01)
			{
				weldSpeedRate.push_back(tGrooveRand.dStartSpeedRate);
			}
			else
			{
				weldSpeedRate.push_back(tGrooveRand.dStartSpeedRate - (disWave - tWavePara.dStartWave) * (tGrooveRand.dStartSpeedRate - tWavePara.dEndSpeedRate) / (tWavePara.dEndWave - tWavePara.dStartWave));
			}
			vdWaveWaitTime.push_back(tGrooveRand.dStartWaveWaitTime + (tGrooveRand.dEndWaveWaitTime - tGrooveRand.dStartWaveWaitTime) * (double)waveNum / (double)totalWave);
			//weldSpeedRate.push_back(1 - (disWave - tWavePara.dStartWave) * (1 - tWavePara.dEndSpeedRate) / (tWavePara.dEndWave - tWavePara.dStartWave));
			if (1 == waveNum % 2)
			{
				tmpCoor.dX = tRobotStartCoord.dX + (sunDisRunVec - tWavePara.dWaveDistance / 2) * vecX;
				tmpCoor.dY = tRobotStartCoord.dY + (sunDisRunVec - tWavePara.dWaveDistance / 2) * vecY;
				tmpCoor.dZ = tRobotStartCoord.dZ + (sunDisRunVec - tWavePara.dWaveDistance / 2) * vecZ;
				tmpCoor.dX = tmpCoor.dX + disWave * vecXClockWise;
				tmpCoor.dY = tmpCoor.dY + disWave * vecYClockWise;
				vtGrooveWavePath.push_back(tmpCoor);
			}
			else
			{
				tmpCoor.dX = tRobotStartCoord.dX + (sunDisRunVec - tWavePara.dWaveDistance / 2) * vecX;
				tmpCoor.dY = tRobotStartCoord.dY + (sunDisRunVec - tWavePara.dWaveDistance / 2) * vecY;
				tmpCoor.dZ = tRobotStartCoord.dZ + (sunDisRunVec - tWavePara.dWaveDistance / 2) * vecZ;
				tmpCoor.dX = tmpCoor.dX + disWave * vecXAntiClockWise;
				tmpCoor.dY = tmpCoor.dY + disWave * vecYAntiClockWise;
				vtGrooveWavePath.push_back(tmpCoor);
			}
		}
		tmpCoor.dX = tRobotEndCoord.dX;
		tmpCoor.dY = tRobotEndCoord.dY;
		tmpCoor.dZ = tRobotEndCoord.dZ;
		vtGrooveWavePath.push_back(tmpCoor);
		weldSpeedRate.push_back(tWavePara.dEndSpeedRate);
		vdWaveWaitTime.push_back(tGrooveRand.dEndWaveWaitTime);
		return 0;
	}
	else if (1 == tWavePara.waveType)
	{
		//确定摆幅固定增加变量
		double waveChangeValue = disWaveEndtoStart / (totalWave - 1);
		for (int waveNum = 0; waveNum < totalWave; waveNum++)
		{
			if (0 == waveNum % 2)
			{
				tmpCoor.dX = tRobotStartCoord.dX + (sunDisRunVec - tWavePara.dWaveDistance / 2) * vecX;
				tmpCoor.dY = tRobotStartCoord.dY + (sunDisRunVec - tWavePara.dWaveDistance / 2) * vecY;
				tmpCoor.dZ = tRobotStartCoord.dZ + (sunDisRunVec - tWavePara.dWaveDistance / 2) * vecZ;
				tmpCoor.dX = tmpCoor.dX + vecOnlyX * tWavePara.dWaveHeight;
				tmpCoor.dY = tmpCoor.dY + vecOnlyY * tWavePara.dWaveHeight;
				tmpCoor.dX = tmpCoor.dX + disWave * vecXClockWise;
				tmpCoor.dY = tmpCoor.dY + disWave * vecYClockWise;
				vtGrooveWavePath.push_back(tmpCoor);
				weldSpeedRate.push_back(1 - (disWave - tWavePara.dStartWave) * (1 - tWavePara.dEndSpeedRate) / (tWavePara.dEndWave - tWavePara.dStartWave));
			}
			else
			{
				tmpCoor.dX = tRobotStartCoord.dX + (sunDisRunVec - tWavePara.dWaveDistance / 2) * vecX;
				tmpCoor.dY = tRobotStartCoord.dY + (sunDisRunVec - tWavePara.dWaveDistance / 2) * vecY;
				tmpCoor.dZ = tRobotStartCoord.dZ + (sunDisRunVec - tWavePara.dWaveDistance / 2) * vecZ;
				tmpCoor.dX = tmpCoor.dX + vecOnlyX * tWavePara.dWaveHeight;
				tmpCoor.dY = tmpCoor.dY + vecOnlyY * tWavePara.dWaveHeight;
				tmpCoor.dX = tmpCoor.dX + disWave * vecXAntiClockWise;
				tmpCoor.dY = tmpCoor.dY + disWave * vecYAntiClockWise;
				vtGrooveWavePath.push_back(tmpCoor);
				weldSpeedRate.push_back(1 - (disWave - tWavePara.dStartWave) * (1 - tWavePara.dEndSpeedRate) / (tWavePara.dEndWave - tWavePara.dStartWave));
				if (sunDisRunVec < disEndToStart)
				{
					tmpCoor.dX = tRobotStartCoord.dX + sunDisRunVec * vecX;
					tmpCoor.dY = tRobotStartCoord.dY + sunDisRunVec * vecY;
					tmpCoor.dZ = tRobotStartCoord.dZ + sunDisRunVec * vecZ;
					vtGrooveWavePath.push_back(tmpCoor);
					weldSpeedRate.push_back(1 - (disWave - tWavePara.dStartWave) * (1 - tWavePara.dEndSpeedRate) / (tWavePara.dEndWave - tWavePara.dStartWave));
				}
			}
			sunDisRunVec += tWavePara.dWaveDistance;
			disWave += waveChangeValue;
		}
		tmpCoor.dX = tRobotEndCoord.dX;
		tmpCoor.dY = tRobotEndCoord.dY;
		tmpCoor.dZ = tRobotEndCoord.dZ;
		vtGrooveWavePath.push_back(tmpCoor);
		weldSpeedRate.push_back(1 - (disWave - tWavePara.dStartWave) * (1 - tWavePara.dEndSpeedRate) / (tWavePara.dEndWave - tWavePara.dStartWave));
		return 0;
	}
	else
	{
		XiMessageBox("暂无此摆动");
		return -1;
	}
}

int GrooveWeld::DoGrooveWeld(vector<vector<T_ROBOT_COORS>> vvtGrooveWavePath, vector<T_WAVE_PARA> vtTWavePara, vector<vector<double>> vWeldSpeedRate, int nRobotNo)
{
	//CRobotDriverAdaptor* pRobotDriver = m_vpRobotDriver[nRobotNo];
	//CUnit* pUnit = m_vpUnit[nRobotNo];
	// 创建并初始化加劲板焊接实例
	//WAM::WeldAfterMeasure* pWeldAfterMeasure = NULL;// = m_pWeldAfterMeasure; // 为了不修以下变量名
	//CHECK_BOOL_RETURN(CreateObject(SMALL_PIECE/*pRobotDriver->m_tChoseWorkPieceType*/, m_vpUnit[nRobotNo], &pWeldAfterMeasure));
	//pWeldAfterMeasure->SetRecoParam();
	//pWeldAfterMeasure->SetHardware(m_vpScanInit[nRobotNo]);
	//pWeldAfterMeasure->SetWeldParam(&m_bStartArcOrNot, &m_bNaturalPop, &m_bTeachPop, &m_bNeedWrap);
	//pWeldAfterMeasure->m_pColorImg = &m_vpShowLaserImgBuff[nRobotNo];
	//pWeldAfterMeasure->m_bIsLocalDebug = GetLocalDebugMark() == TRUE;
	//pWeldAfterMeasure->LoadPauseInfo();
	//CHECK_BOOL_RETURN(pWeldAfterMeasure->FreeWaveGrooveWeld(vvtGrooveWavePath, vtTWavePara, vWeldSpeedRate));
	return true;
}

int GrooveWeld::GrooveAutoRandNew(T_GROOVE_INFOR tGrooveInfor, vector<T_WAVE_PARA> vtTWavePara, vector<T_INFOR_WAVE_RAND>& vtGrooveRand, int nRobotDir)
{
	T_INFOR_WAVE_RAND tmpPointOffset;
	int noCount = 0;//按照道数进行偏移
	double dInSideDis = 1.5; // 焊丝摆动进入坡口距离
	double dSameLayerDis = 2.0; // 一层两道之间焊丝最短距离
	double dArcThickness = 5.0;//每层焊接厚度
	double dUpperFace = tGrooveInfor.dStartUpperFace > tGrooveInfor.dEndUpperFace ? tGrooveInfor.dStartUpperFace : tGrooveInfor.dEndUpperFace;
	double dLowerFace = tGrooveInfor.dStartLowerFace > tGrooveInfor.dEndLowerFace ? tGrooveInfor.dStartLowerFace : tGrooveInfor.dEndLowerFace;
	double disFace = dUpperFace - dLowerFace;

	double dStartOffsetDis = 0.0; //多层多道每次起弧位置偏移
	dStartOffsetDis = tGrooveInfor.nStartChangeType == 1 ? 20.0 : dStartOffsetDis;
	dStartOffsetDis = tGrooveInfor.nStartChangeType == 2 ? -20.0 : dStartOffsetDis;

	double dEndOffsetDis = 0.0; //多层多道每次收弧位置偏移
	dEndOffsetDis = tGrooveInfor.nEndChangeType == 1 ? 20.0 : dEndOffsetDis;
	dEndOffsetDis = tGrooveInfor.nEndChangeType == 2 ? -20.0 : dEndOffsetDis;
	
	int numLayel = tGrooveInfor.dPlateThickness / dArcThickness;
	double dRemain = tGrooveInfor.dPlateThickness - numLayel * dArcThickness;
	if (dArcThickness / 2 < dRemain)
	{
		numLayel++;
	}
	double disRateFace = disFace / numLayel;

	double dStartUpperFace = tGrooveInfor.dStartUpperFace;
	double dStartLowerFace = tGrooveInfor.dStartLowerFace;
	double StartDisFace = dStartUpperFace - dStartLowerFace;
	double StartDisRateFace = StartDisFace / numLayel;

	double dEndUpperFace = tGrooveInfor.dEndUpperFace;
	double dEndLowerFace = tGrooveInfor.dEndLowerFace;
	double EndDisFace = dEndUpperFace - dEndLowerFace;
	double EndDisRateFace = EndDisFace / numLayel;

	bool isVerticalWeld = false;//是否立焊
	T_ROBOT_COORS tRobotStartCoord = tGrooveInfor.tStartPoint;
	T_ROBOT_COORS tRobotEndCoord = tGrooveInfor.tEndPoint;
	double disEndToStart = sqrt(pow((tRobotEndCoord.dX - tRobotStartCoord.dX), 2) + pow((tRobotEndCoord.dY - tRobotStartCoord.dY), 2) + pow((tRobotEndCoord.dZ - tRobotStartCoord.dZ), 2));
	//if (120 > disEndToStart)
	if (30 > disEndToStart)
	{
		XiMessageBox("焊缝距离过短!");
		return -1;
	}
	double vecX = (tRobotEndCoord.dX - tRobotStartCoord.dX) / disEndToStart;
	double vecY = (tRobotEndCoord.dY - tRobotStartCoord.dY) / disEndToStart;
	double vecZ = (tRobotEndCoord.dZ - tRobotStartCoord.dZ) / disEndToStart;
	double dHorOffsetX;
	double dHorOffsetY;
	double vecOnlyX;
	double vecOnlyY;
	double vecOnlyZ;
	//double offset;
	if (abs(vecZ) > 0.5)
	{
		isVerticalWeld = true;
		//double weldAngle = PI * tRobotStartCoord.dRZ / 180;
		double weldAngle = PI * tGrooveInfor.weldAngle / 180;
		dHorOffsetX = sin(weldAngle) * nRobotDir;
		dHorOffsetY = -cos(weldAngle) * nRobotDir;
		vecOnlyX = cos(weldAngle) * vecZ * nRobotDir;
		vecOnlyY = sin(weldAngle) * vecZ * nRobotDir;
		vecOnlyZ = sqrt(pow(vecX, 2) + pow(vecY, 2)) * nRobotDir;
	}
	else
	{
		dHorOffsetX = -vecY * nRobotDir;
		dHorOffsetY = vecX * nRobotDir;
		vecOnlyZ = sqrt(pow(vecX, 2) + pow(vecY, 2)) * nRobotDir;
	}

	tmpPointOffset.tStartPoint = tRobotStartCoord;
	tmpPointOffset.tEndPoint = tRobotEndCoord;
	if (!isVerticalWeld)
	{
		for (size_t i = 0; i < numLayel; i++)
		{
			tmpPointOffset.nLayerNo = i; // 层号  一层可能对应多道
			if (i < 2 /* || (dLowerFace < 16 && dLowerFace > 0)*/)
			{
				double dChangeS = (tGrooveInfor.dStartLowerFace - 5.0) * (1.0 - vtTWavePara[noCount].dEndSpeedRate) / (5.0 - 12.0);
				double dChangeE = (tGrooveInfor.dEndLowerFace - 5.0) * (1.0 - vtTWavePara[noCount].dEndSpeedRate) / (5.0 - 12.0);
				tmpPointOffset.dStartSpeedRate = 1.0 + dChangeS;
				tmpPointOffset.dEndSpeedRate = 1.0 + dChangeE;

				dChangeS = (tGrooveInfor.dStartLowerFace - 5.0) * (vtTWavePara[noCount].dStartWave - vtTWavePara[noCount].dEndWave) / (5.0 - 12.0); // 5mm n1 12mm n2    4mm
				dChangeE = (tGrooveInfor.dEndLowerFace - 12.0) * (vtTWavePara[noCount].dStartWave - vtTWavePara[noCount].dEndWave) / (5.0 - 12.0); // 5mm n1 12mm n2    4mm
				tmpPointOffset.dStartWaveDis = vtTWavePara[noCount].dStartWave + dChangeS;
				tmpPointOffset.dEndWaveDis = vtTWavePara[noCount].dEndWave + dChangeE;

				dChangeS = (tGrooveInfor.dStartLowerFace - 5.0) * (vtTWavePara[noCount].dLeftWaitTime - vtTWavePara[noCount].dMidWaitTime) / (5.0 - 12.0);
				dChangeE = (tGrooveInfor.dEndLowerFace - 12.0) * (vtTWavePara[noCount].dLeftWaitTime - vtTWavePara[noCount].dMidWaitTime) / (5.0 - 12.0);
				tmpPointOffset.dStartWaveWaitTime = vtTWavePara[noCount].dLeftWaitTime + dChangeS;
				tmpPointOffset.dEndWaveWaitTime = vtTWavePara[noCount].dMidWaitTime + dChangeE;

				if (0 == i)
				{
					dStartLowerFace += (StartDisFace * vtTWavePara[noCount].dNormalOffset / tGrooveInfor.dStartThickness);
					dEndLowerFace += (EndDisFace * vtTWavePara[noCount].dNormalOffset / tGrooveInfor.dEndThickness);
					tmpPointOffset.tStartPoint.dX += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetX;
					tmpPointOffset.tStartPoint.dY += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetY;
					tmpPointOffset.tStartPoint.dZ += vtTWavePara[noCount].dNormalOffset * vecOnlyZ;

					tmpPointOffset.tEndPoint.dX += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetX;
					tmpPointOffset.tEndPoint.dY += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetY;
					tmpPointOffset.tEndPoint.dZ += vtTWavePara[noCount].dNormalOffset * vecOnlyZ;

					tmpPointOffset.dAngleOffset = 0;
					vtGrooveRand.push_back(tmpPointOffset);
					noCount++;
				}
				else
				{
					dStartLowerFace += (StartDisFace * vtTWavePara[noCount].dNormalOffset / tGrooveInfor.dStartThickness);
					dEndLowerFace += (EndDisFace * vtTWavePara[noCount].dNormalOffset / tGrooveInfor.dEndThickness);
					tmpPointOffset.tStartPoint.dX += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetX + dStartOffsetDis * vecX;
					tmpPointOffset.tStartPoint.dY += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetY + dStartOffsetDis * vecY;
					tmpPointOffset.tStartPoint.dZ += vtTWavePara[noCount].dNormalOffset * vecOnlyZ + dStartOffsetDis * vecZ;

					tmpPointOffset.tEndPoint.dX += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetX - dEndOffsetDis * vecX;
					tmpPointOffset.tEndPoint.dY += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetY - dEndOffsetDis * vecY;
					tmpPointOffset.tEndPoint.dZ += vtTWavePara[noCount].dNormalOffset * vecOnlyZ - dEndOffsetDis * vecZ;

					tmpPointOffset.dAngleOffset = 0;
					vtGrooveRand.push_back(tmpPointOffset);
					noCount++;
				}
			}
			else if (i > 1 /* && (dLowerFace >= 16 && dLowerFace < 24)*/)
			{

				dStartLowerFace += (StartDisFace * vtTWavePara[noCount].dNormalOffset / tGrooveInfor.dStartThickness);
				dEndLowerFace += (EndDisFace * vtTWavePara[noCount].dNormalOffset / tGrooveInfor.dEndThickness);

				// 摆动幅度自动计算
				tmpPointOffset.dStartWaveDis = (dStartLowerFace + (dInSideDis * 2.0) - dSameLayerDis) / 4.0; // 摆动幅度
				tmpPointOffset.dEndWaveDis = (dEndLowerFace + (dInSideDis * 2.0) - dSameLayerDis) / 4.0;
				//dChangeS = (tGrooveInfor.dStartLowerFace - 5.0) * (vtTWavePara[noCount].dStartWave - vtTWavePara[noCount].dEndWave) / (5.0 - 12.0); // 5mm n1 12mm n2    4mm
				//dChangeE = (tGrooveInfor.dEndLowerFace - 12.0) * (vtTWavePara[noCount].dStartWave - vtTWavePara[noCount].dEndWave) / (5.0 - 12.0); // 5mm n1 12mm n2    4mm
				//tmpPointOffset.dStartWaveDis = vtTWavePara[noCount].dStartWave + dChangeS;
				//tmpPointOffset.dEndWaveDis = vtTWavePara[noCount].dEndWave + dChangeE;

				//double dStartOffset = (dStartLowerFace + (dInSideDis * 2.0)) / 4.0; // 偏移
				//double dEndOffset = (dEndLowerFace + (dInSideDis * 2.0)) / 4.0;
				double dStartOffset = tmpPointOffset.dStartWaveDis + dSameLayerDis / 2.0;
				double dEndOffset = tmpPointOffset.dEndWaveDis + dSameLayerDis / 2.0;

				double dChangeS = (tGrooveInfor.dStartLowerFace - 5.0) * (1.0 - vtTWavePara[noCount].dEndSpeedRate) / (5.0 - 12.0);
				double dChangeE = (tGrooveInfor.dEndLowerFace - 5.0) * (1.0 - vtTWavePara[noCount].dEndSpeedRate) / (5.0 - 12.0);
				tmpPointOffset.dStartSpeedRate = 1.0 + dChangeS;
				tmpPointOffset.dEndSpeedRate = 1.0 + dChangeE;

				dChangeS = (tGrooveInfor.dStartLowerFace - 5.0) * (vtTWavePara[noCount].dLeftWaitTime - vtTWavePara[noCount].dMidWaitTime) / (5.0 - 12.0);
				dChangeE = (tGrooveInfor.dEndLowerFace - 12.0) * (vtTWavePara[noCount].dLeftWaitTime - vtTWavePara[noCount].dMidWaitTime) / (5.0 - 12.0);
				tmpPointOffset.dStartWaveWaitTime = vtTWavePara[noCount].dLeftWaitTime + dChangeS;
				tmpPointOffset.dEndWaveWaitTime = vtTWavePara[noCount].dMidWaitTime + dChangeE;

					
				//tmpPointOffset.tStartPoint.dX += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetX + dArcRandOffset * vecX;
				//tmpPointOffset.tStartPoint.dY += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetY + dArcRandOffset * vecY;
				tmpPointOffset.tStartPoint.dX = tRobotStartCoord.dX + dStartOffset * dHorOffsetX + dStartOffsetDis * i * vecX;
				tmpPointOffset.tStartPoint.dY = tRobotStartCoord.dY + dStartOffset * dHorOffsetY + dStartOffsetDis * i * vecY;
				tmpPointOffset.tStartPoint.dZ += vtTWavePara[noCount].dNormalOffset * vecOnlyZ + dStartOffsetDis * i * vecZ;

				//tmpPointOffset.tEndPoint.dX += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetX - dArcRandOffset * vecX;
				//tmpPointOffset.tEndPoint.dY += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetY - dArcRandOffset * vecY;
				tmpPointOffset.tEndPoint.dX = tRobotEndCoord.dX + dEndOffset * dHorOffsetX - dEndOffsetDis * i * vecX;
				tmpPointOffset.tEndPoint.dY = tRobotEndCoord.dY + dEndOffset * dHorOffsetY - dEndOffsetDis * i * vecY;
				tmpPointOffset.tEndPoint.dZ += vtTWavePara[noCount].dNormalOffset * vecOnlyZ - dEndOffsetDis * i * vecZ;

				tmpPointOffset.dAngleOffset = 0.0;
				vtGrooveRand.push_back(tmpPointOffset);
				noCount++;

				dStartLowerFace += (StartDisFace * vtTWavePara[noCount].dNormalOffset / tGrooveInfor.dStartThickness);
				dEndLowerFace += (EndDisFace * vtTWavePara[noCount].dNormalOffset / tGrooveInfor.dEndThickness);
				dStartOffset = dStartOffset * -1.0;
				dEndOffset = dEndOffset * -1.0;			

				// 摆动幅度自动计算
				tmpPointOffset.dStartWaveDis = (dStartLowerFace + (dInSideDis * 2.0) - dSameLayerDis) / 4.0; // 摆动幅度
				tmpPointOffset.dEndWaveDis = (dEndLowerFace + (dInSideDis * 2.0) - dSameLayerDis) / 4.0;
				//dChangeS = (tGrooveInfor.dStartLowerFace - 5.0) * (vtTWavePara[noCount].dStartWave - vtTWavePara[noCount].dEndWave) / (5.0 - 12.0); // 5mm n1 12mm n2    4mm
				//dChangeE = (tGrooveInfor.dEndLowerFace - 12.0) * (vtTWavePara[noCount].dStartWave - vtTWavePara[noCount].dEndWave) / (5.0 - 12.0); // 5mm n1 12mm n2    4mm
				//tmpPointOffset.dStartWaveDis = vtTWavePara[noCount].dStartWave + dChangeS;
				//tmpPointOffset.dEndWaveDis = vtTWavePara[noCount].dEndWave + dChangeE;

				dChangeS = (tGrooveInfor.dStartLowerFace - 5.0) * (1.0 - vtTWavePara[noCount].dEndSpeedRate) / (5.0 - 12.0);
				dChangeE = (tGrooveInfor.dEndLowerFace - 5.0) * (1.0 - vtTWavePara[noCount].dEndSpeedRate) / (5.0 - 12.0);
				tmpPointOffset.dStartSpeedRate = 1.0 + dChangeS;
				tmpPointOffset.dEndSpeedRate = 1.0 + dChangeE;

				dChangeS = (tGrooveInfor.dStartLowerFace - 5.0) * (vtTWavePara[noCount].dLeftWaitTime - vtTWavePara[noCount].dMidWaitTime) / (5.0 - 12.0);
				dChangeE = (tGrooveInfor.dEndLowerFace - 12.0) * (vtTWavePara[noCount].dLeftWaitTime - vtTWavePara[noCount].dMidWaitTime) / (5.0 - 12.0);
				tmpPointOffset.dStartWaveWaitTime = vtTWavePara[noCount].dLeftWaitTime + dChangeS;
				tmpPointOffset.dEndWaveWaitTime = vtTWavePara[noCount].dMidWaitTime + dChangeE;

				//tmpPointOffset.tStartPoint.dX += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetX;
				//tmpPointOffset.tStartPoint.dY += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetY;
				tmpPointOffset.tStartPoint.dX = tRobotStartCoord.dX + dStartOffset * dHorOffsetX + dStartOffsetDis * i * vecX;;
				tmpPointOffset.tStartPoint.dY = tRobotStartCoord.dY + dStartOffset * dHorOffsetY + dStartOffsetDis * i * vecY;;
				tmpPointOffset.tStartPoint.dZ += vtTWavePara[noCount].dNormalOffset * vecOnlyZ + dStartOffsetDis * i * vecZ;

				//tmpPointOffset.tEndPoint.dX += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetX;
				//tmpPointOffset.tEndPoint.dY += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetY;
				tmpPointOffset.tEndPoint.dX = tRobotEndCoord.dX + dEndOffset * dHorOffsetX - dEndOffsetDis * i * vecX;
				tmpPointOffset.tEndPoint.dY = tRobotEndCoord.dY + dEndOffset * dHorOffsetY - dEndOffsetDis * i * vecY;
				tmpPointOffset.tEndPoint.dZ += vtTWavePara[noCount].dNormalOffset * vecOnlyZ - dEndOffsetDis * i * vecZ;

				tmpPointOffset.dAngleOffset = 0.0;
				vtGrooveRand.push_back(tmpPointOffset);
				noCount++;
			}
			//else if (i > 1 && dLowerFace >= 40/*24*/)
			//{
			//	tmpPointOffset.tStartPoint.dX += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetX + dArcRandOffset * vecX;
			//	tmpPointOffset.tStartPoint.dY += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetY + dArcRandOffset * vecY;
			//	tmpPointOffset.tStartPoint.dZ += vtTWavePara[noCount].dNormalOffset * vecOnlyZ + dArcRandOffset * vecZ;
			//
			//	tmpPointOffset.tEndPoint.dX += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetX + dArcRandOffset * vecX;
			//	tmpPointOffset.tEndPoint.dY += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetY + dArcRandOffset * vecY;
			//	tmpPointOffset.tEndPoint.dZ += vtTWavePara[noCount].dNormalOffset * vecOnlyZ + dArcRandOffset * vecZ;
			//
			//	tmpPointOffset.dAngleOffset = 0.0;
			//	vtGrooveRand.push_back(tmpPointOffset);
			//	noCount++;
			//
			//	tmpPointOffset.tStartPoint.dX += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetX;
			//	tmpPointOffset.tStartPoint.dY += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetY;
			//	tmpPointOffset.tStartPoint.dZ += vtTWavePara[noCount].dNormalOffset * vecOnlyZ;
			//
			//	tmpPointOffset.tEndPoint.dX += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetX;
			//	tmpPointOffset.tEndPoint.dY += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetY;
			//	tmpPointOffset.tEndPoint.dZ += vtTWavePara[noCount].dNormalOffset * vecOnlyZ;
			//
			//	tmpPointOffset.dAngleOffset = 0;
			//	vtGrooveRand.push_back(tmpPointOffset);
			//	noCount++;
			//
			//	tmpPointOffset.tStartPoint.dX += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetX;
			//	tmpPointOffset.tStartPoint.dY += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetY;
			//	tmpPointOffset.tStartPoint.dZ += vtTWavePara[noCount].dNormalOffset * vecOnlyZ;
			//
			//	tmpPointOffset.tEndPoint.dX += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetX;
			//	tmpPointOffset.tEndPoint.dY += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetY;
			//	tmpPointOffset.tEndPoint.dZ += vtTWavePara[noCount].dNormalOffset * vecOnlyZ;
			//
			//	tmpPointOffset.dAngleOffset = 0.0;
			//	vtGrooveRand.push_back(tmpPointOffset);
			//	noCount++;
			//}
			else
			{
				XiMessageBox("底面焊缝过宽！");
				return -1;
			}
			dLowerFace += disRateFace;
			//dStartLowerFace += StartDisRateFace;
			//dEndLowerFace += EndDisRateFace;
		}
	}
	else
	{
		dInSideDis = 2.5;
		for (size_t i = 0; i < numLayel; i++)
		{
			//if (i < numLayel - 1)
			{
				if (0 == i)
				{
					dStartLowerFace += StartDisFace * vtTWavePara[noCount].dNormalOffset / tGrooveInfor.dStartThickness;
					dEndLowerFace += EndDisFace * vtTWavePara[noCount].dNormalOffset / tGrooveInfor.dEndThickness;
					tmpPointOffset.dStartWaveDis = dStartLowerFace / 2.0 + dInSideDis;
					tmpPointOffset.dEndWaveDis = dEndLowerFace / 2.0 + dInSideDis;
					double dChangeS = (tGrooveInfor.dStartLowerFace - 5.0) * (1.0 - vtTWavePara[noCount].dEndSpeedRate) / (5.0 - 12.0);
					double dChangeE = (tGrooveInfor.dEndLowerFace - 5.0) * (1.0 - vtTWavePara[noCount].dEndSpeedRate) / (5.0 - 12.0);
					tmpPointOffset.dStartSpeedRate = 1.0 + dChangeS;
					tmpPointOffset.dEndSpeedRate = 1.0 + dChangeE;

					dChangeS = (tGrooveInfor.dStartLowerFace - 5.0) * (vtTWavePara[noCount].dLeftWaitTime - vtTWavePara[noCount].dMidWaitTime) / (5.0 - 12.0);
					dChangeE = (tGrooveInfor.dEndLowerFace - 12.0) * (vtTWavePara[noCount].dLeftWaitTime - vtTWavePara[noCount].dMidWaitTime) / (5.0 - 12.0);
					tmpPointOffset.dStartWaveWaitTime = vtTWavePara[noCount].dLeftWaitTime + dChangeS;
					tmpPointOffset.dEndWaveWaitTime = vtTWavePara[noCount].dMidWaitTime + dChangeE;

					tmpPointOffset.tStartPoint.dX += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetX + vtTWavePara[noCount].dNormalOffset * vecOnlyX;
					tmpPointOffset.tStartPoint.dY += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetY + vtTWavePara[noCount].dNormalOffset * vecOnlyY;
					tmpPointOffset.tStartPoint.dZ += vtTWavePara[noCount].dNormalOffset * vecOnlyZ;

					tmpPointOffset.tEndPoint.dX += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetX + vtTWavePara[noCount].dNormalOffset * vecOnlyX;
					tmpPointOffset.tEndPoint.dY += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetY + vtTWavePara[noCount].dNormalOffset * vecOnlyY;
					tmpPointOffset.tEndPoint.dZ += vtTWavePara[noCount].dNormalOffset * vecOnlyZ;

					tmpPointOffset.dAngleOffset = 0;
					vtGrooveRand.push_back(tmpPointOffset);
					noCount++;
				}
				else
				{
					if ((numLayel - 1) == i) dInSideDis = 1.7;
					dStartLowerFace += StartDisFace * vtTWavePara[noCount].dNormalOffset / tGrooveInfor.dStartThickness;
					dEndLowerFace += EndDisFace * vtTWavePara[noCount].dNormalOffset / tGrooveInfor.dEndThickness;
					tmpPointOffset.dStartWaveDis = dStartLowerFace / 2.0 + dInSideDis;
					tmpPointOffset.dEndWaveDis = dEndLowerFace / 2.0 + dInSideDis;
					double dChangeS = (tGrooveInfor.dStartLowerFace - 5.0) * (1.0 - vtTWavePara[noCount].dEndSpeedRate) / (5.0 - 12.0);
					double dChangeE = (tGrooveInfor.dEndLowerFace - 5.0) * (1.0 - vtTWavePara[noCount].dEndSpeedRate) / (5.0 - 12.0);
					tmpPointOffset.dStartSpeedRate = 1.0 + dChangeS;
					tmpPointOffset.dEndSpeedRate = 1.0 + dChangeE;

					dChangeS = (tGrooveInfor.dStartLowerFace - 5.0) * (vtTWavePara[noCount].dLeftWaitTime - vtTWavePara[noCount].dMidWaitTime) / (5.0 - 12.0);
					dChangeE = (tGrooveInfor.dEndLowerFace - 12.0) * (vtTWavePara[noCount].dLeftWaitTime - vtTWavePara[noCount].dMidWaitTime) / (5.0 - 12.0);
					tmpPointOffset.dStartWaveWaitTime = vtTWavePara[noCount].dLeftWaitTime + dChangeS;
					tmpPointOffset.dEndWaveWaitTime = vtTWavePara[noCount].dMidWaitTime + dChangeE;

					tmpPointOffset.tStartPoint.dX += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetX + vtTWavePara[noCount].dNormalOffset * vecOnlyX + dStartOffsetDis * vecX;
					tmpPointOffset.tStartPoint.dY += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetY + vtTWavePara[noCount].dNormalOffset * vecOnlyY + dStartOffsetDis * vecY;
					tmpPointOffset.tStartPoint.dZ += vtTWavePara[noCount].dNormalOffset * vecOnlyZ + dStartOffsetDis * vecZ;

					tmpPointOffset.tEndPoint.dX += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetX + vtTWavePara[noCount].dNormalOffset * vecOnlyX - dEndOffsetDis * vecX;
					tmpPointOffset.tEndPoint.dY += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetY + vtTWavePara[noCount].dNormalOffset * vecOnlyY - dEndOffsetDis * vecY;
					tmpPointOffset.tEndPoint.dZ += vtTWavePara[noCount].dNormalOffset * vecOnlyZ - dEndOffsetDis * vecZ;

					tmpPointOffset.dAngleOffset = 0;
					vtGrooveRand.push_back(tmpPointOffset);
					noCount++;
				}
			}
			//else if (numLayel - 1 == i)
			//{
			//	tmpPointOffset.tStartPoint.dX += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetX + vtTWavePara[noCount].dNormalOffset * vecOnlyX + dArcRandOffset * vecX;
			//	tmpPointOffset.tStartPoint.dY += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetY + vtTWavePara[noCount].dNormalOffset * vecOnlyY + dArcRandOffset * vecY;
			//	tmpPointOffset.tStartPoint.dZ += vtTWavePara[noCount].dNormalOffset * vecOnlyZ + dArcRandOffset * vecZ;
			//
			//	tmpPointOffset.tEndPoint.dX += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetX + vtTWavePara[noCount].dNormalOffset * vecOnlyX - dArcRandOffset * vecX;
			//	tmpPointOffset.tEndPoint.dY += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetY + vtTWavePara[noCount].dNormalOffset * vecOnlyY - dArcRandOffset * vecY;
			//	tmpPointOffset.tEndPoint.dZ += vtTWavePara[noCount].dNormalOffset * vecOnlyZ - dArcRandOffset * vecZ;
			//
			//	tmpPointOffset.dAngleOffset = 0.0;
			//	vtGrooveRand.push_back(tmpPointOffset);
			//	noCount++;
			//
			//	tmpPointOffset.tStartPoint.dX += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetX + vtTWavePara[noCount].dNormalOffset * vecOnlyX;
			//	tmpPointOffset.tStartPoint.dY += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetY + vtTWavePara[noCount].dNormalOffset * vecOnlyY;
			//	tmpPointOffset.tStartPoint.dZ += vtTWavePara[noCount].dNormalOffset * vecOnlyZ;
			//	
			//	tmpPointOffset.tEndPoint.dX += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetX + vtTWavePara[noCount].dNormalOffset * vecOnlyX;
			//	tmpPointOffset.tEndPoint.dY += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetY + vtTWavePara[noCount].dNormalOffset * vecOnlyY;
			//	tmpPointOffset.tEndPoint.dZ += vtTWavePara[noCount].dNormalOffset * vecOnlyZ;
			//
			//	tmpPointOffset.dAngleOffset = 0.0;
			//	vtGrooveRand.push_back(tmpPointOffset);
			//	noCount++;
			//}
			//else
			//{
			//	XiMessageBox("底面焊缝过宽！");
			//	return -1;
			//}
			dLowerFace += disRateFace;
		}
	}
	return 0;
}

bool GrooveWeld::AdjustStartEndPtn(int nRobotNo, int nWeldIdx, E_WELD_SEAM_TYPE eWeldSeamType, T_ROBOT_COORS& tStartPtn, T_ROBOT_COORS& tEndPtn)
{
	double dMinDisThreshold = 200.0;
	double dReplateDis = 10.0;

	std::vector < std::vector<T_ROBOT_COORS>> vvtWeldTrackCoord(0);
	// 加载第nGrouoNo组焊缝以前的所有焊缝 平焊加载两个机器人平焊 立焊只加载当前机器人立焊
	int nGroupNo = (nRobotNo == 0) ? nWeldIdx - 1 : nWeldIdx - 2; // 当前机器人已焊过焊缝

	E_WELD_SEAM_TYPE eReadWeldSeamType;
	double dExAxlePos = 0.0;
	std::vector<T_ROBOT_COORS> vtWeldTrack(0);
	std::vector<int> vnPtnType(0);
	CString sFileName;
	CString sBackUpFileName;
	bool bExistBackUp = false;

	for (; nGroupNo >= 0; nGroupNo--) // 加载当前机器人 当前索引的平焊或
	{
		sFileName.Format("%s%d_%d_RealWeldCoord.txt", m_sDataSavePath, nGroupNo, 0);
		sBackUpFileName.Format("%s%d_%d_RealWeldCoord_SrcBackUp.txt", m_sDataSavePath, nGroupNo, 0);
		bExistBackUp = CheckFileExists(sBackUpFileName, false);
		CString sReadFileName = bExistBackUp ? sBackUpFileName : sFileName; // 保证加载的是第一次测量的原始轨迹 避免重复修正
		if (false == LoadRealWeldTrack_G(sReadFileName, eReadWeldSeamType, dExAxlePos, vtWeldTrack, vnPtnType))
		{
			m_pRobotDriver->m_cLog->Write("%s 第%d道 第%d组 焊缝轨迹调整加载轨迹失败！端点调整失败", m_pRobotDriver->m_strRobotName, 1, nGroupNo + 1);
			break;
		}
		if (eReadWeldSeamType == eWeldSeamType)
		{
			vvtWeldTrackCoord.push_back(vtWeldTrack);
		}
	}


	if (E_PLAT_GROOVE == eWeldSeamType) // 加载非当前机器人 平焊焊缝 Y值取反
	{
		nGroupNo = (nRobotNo == 0) ? nWeldIdx - 2 : nWeldIdx - 1; // 非当前机器人已焊过焊缝
		CString sDataSavePath = OUTPUT_PATH + (0 == nRobotNo ? "RobotB" : "RobotA") + RECOGNITION_FOLDER;
		for (; nGroupNo >= 0; nGroupNo--) // 加载当前机器人 当前索引的平焊或
		{
			sFileName.Format("%s%d_%d_RealWeldCoord.txt", sDataSavePath, nGroupNo, 0);
			sBackUpFileName.Format("%s%d_%d_RealWeldCoord_SrcBackUp.txt", sDataSavePath, nGroupNo, 0);
			bExistBackUp = CheckFileExists(sBackUpFileName, false);
			CString sReadFileName = bExistBackUp ? sBackUpFileName : sFileName; // 保证加载的是第一次测量的原始轨迹 避免重复修正
			if (false == LoadRealWeldTrack_G(sReadFileName, eReadWeldSeamType, dExAxlePos, vtWeldTrack, vnPtnType))
			{
				m_pRobotDriver->m_cLog->Write("%s 第%d道 第%d组 焊缝轨迹调整加载轨迹失败！端点调整失败", m_pRobotDriver->m_strRobotName, 1, nGroupNo + 1);
				break;
			}
			if (eReadWeldSeamType == eWeldSeamType) // 调整非当前机器人焊接轨迹坐标，Y值取反
			{
				for (int i = 0; i < vtWeldTrack.size(); i++)
				{
					vtWeldTrack[i].dY *= (-1.0);
					vtWeldTrack[i].dBY *= (-1.0);
				}
				vvtWeldTrackCoord.push_back(vtWeldTrack);
			}
		}
	}

	// 获取所有已焊接焊缝vvtWeldTrackCoord端点坐标 到 vtPtns
	std::vector<T_ROBOT_COORS> vtPtns(0);
	for (int i = 0; i < vvtWeldTrackCoord.size(); i++)
	{
		vtPtns.push_back(vvtWeldTrackCoord[i].front());
		vtPtns.push_back(vvtWeldTrackCoord[i].back());
	}

	// 查找当前焊缝起终点 距离 其他焊缝的起终点最小距离d 及 最小距离坐标p （Y Z 平面距离）
	double dMinDisS = 999999.0;
	double dMinDisE = 999999.0;
	T_ROBOT_COORS tMinDisCoordS;
	T_ROBOT_COORS tMinDisCoordE;
	for (int i = 0; i < vtPtns.size(); i++)
	{
		double dDisS = TwoPointDis(tStartPtn.dY + tStartPtn.dBY, tStartPtn.dZ + tStartPtn.dBZ, vtPtns[i].dY + vtPtns[i].dBY, vtPtns[i].dZ + vtPtns[i].dBZ);
		double dDisE = TwoPointDis(tEndPtn.dY + tEndPtn.dBY, tEndPtn.dZ + tEndPtn.dBZ, vtPtns[i].dY + vtPtns[i].dBY, vtPtns[i].dZ + vtPtns[i].dBZ);
		if (dDisS < dMinDisS)
		{
			dMinDisS = dDisS;
			tMinDisCoordS = vtPtns[i];
		}
		if (dDisE < dMinDisE)
		{
			dMinDisE = dDisE;
			tMinDisCoordE = vtPtns[i];
		}
	}

	// 起点或终点最小距离d小于dMinDisThreshold 时，将起终点坐标Y值(立焊Z值)调整到最小坐标P的位置
	double dDis = TwoPointDis(
		tStartPtn.dX + tStartPtn.dBX, tStartPtn.dY + tStartPtn.dBY, tStartPtn.dZ + tStartPtn.dBZ,
		tEndPtn.dX + tEndPtn.dBX, tEndPtn.dY + tEndPtn.dBY, tEndPtn.dZ + tEndPtn.dBZ);
	double dDisX = (tEndPtn.dX + tEndPtn.dBX) - (tStartPtn.dX + tStartPtn.dBX);
	double dDisY = (tEndPtn.dY + tEndPtn.dBY) - (tStartPtn.dY + tStartPtn.dBY);
	double dDisZ = (tEndPtn.dZ + tEndPtn.dBZ) - (tStartPtn.dZ + tStartPtn.dBZ);
	if (dMinDisS < dMinDisThreshold)
	{
		double dChange; 
		double dChangeRate;
		if (E_PLAT_GROOVE == eWeldSeamType)
		{
			dChange = (tMinDisCoordS.dY + tMinDisCoordS.dBY) - tStartPtn.dY;
			dChangeRate = dChange / dDisY;
		}
		else
		{
			dChange = (tMinDisCoordS.dZ + tMinDisCoordS.dBZ) - tStartPtn.dZ;
			dChangeRate = dChange / dDisZ;
		}
		tStartPtn.dX += dDisX * dChangeRate;
		tStartPtn.dY += dDisY * dChangeRate;
		tStartPtn.dZ += dDisZ * dChangeRate;
		T_ROBOT_COORS tDirS = tStartPtn;
		tDirS.dX += tDirS.dBX; tDirS.dBX = 0.0;
		tDirS.dY += tDirS.dBY; tDirS.dBY = 0.0;
		tDirS.dZ += tDirS.dBZ; tDirS.dBZ = 0.0;
		T_ROBOT_COORS tDirE = tEndPtn;
		tDirE.dX += tDirE.dBX; tDirE.dBX = 0.0;
		tDirE.dY += tDirE.dBY; tDirE.dBY = 0.0;
		tDirE.dZ += tDirE.dBZ; tDirE.dBZ = 0.0;

		double dDecLength = 0.0;
		if (1 == nRobotNo && 1 == nWeldIdx) // 临时：机器人B 第一个平焊 接头搭接长度减小长度
		{
			dDecLength = -15.0;
			dReplateDis += dDecLength;
		}
		tStartPtn = RobotCoordPosOffset(tStartPtn, tDirE, tDirS, dReplateDis);
		dReplateDis -= dDecLength;
	}

	if (dMinDisE < dMinDisThreshold)
	{
		double dChange;
		double dChangeRate;
		if (E_PLAT_GROOVE == eWeldSeamType)
		{
			dChange = (tMinDisCoordE.dY + tMinDisCoordE.dBY) - tEndPtn.dY;
			dChangeRate = dChange / dDisY;
		}
		else
		{
			dChange = (tMinDisCoordE.dZ + tMinDisCoordE.dBZ) - tEndPtn.dZ;
			dChangeRate = dChange / dDisZ;
		}
		tEndPtn.dX += dDisX * dChangeRate;
		tEndPtn.dY += dDisY * dChangeRate;
		tEndPtn.dZ += dDisZ * dChangeRate;
		T_ROBOT_COORS tDirS = tStartPtn;
		tDirS.dX += tDirS.dBX; tDirS.dBX = 0.0;
		tDirS.dY += tDirS.dBY; tDirS.dBY = 0.0;
		tDirS.dZ += tDirS.dBZ; tDirS.dBZ = 0.0;
		T_ROBOT_COORS tDirE = tEndPtn;
		tDirE.dX += tDirE.dBX; tDirE.dBX = 0.0;
		tDirE.dY += tDirE.dBY; tDirE.dBY = 0.0;
		tDirE.dZ += tDirE.dBZ; tDirE.dBZ = 0.0;
		tEndPtn = RobotCoordPosOffset(tEndPtn, tDirS, tDirE, dReplateDis);
	}
	return true;
}
