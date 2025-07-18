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
	
	//已修改
	default: XUI::MesBox::PopInfo("GenericWeld不支持工件类型!")/*XiMessageBox("GenericWeld不支持工件类型%d!", m_eWorkPieceType)*/; break;
	//default: XiMessageBox("GenericWeld不支持工件类型%d!", m_eWorkPieceType); break;
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
	LineOrCircularArcWeldingLine* pLineOrCircularArcWeldingLine; // 李伟
	//m_sPointCloudFileName = "LocalFiles\\OutputFiles\\RobotA\\Recognition\\PointCloud_0.txt";
	int nCurUseTable = 0;
	COPini opini2;
	
	opini2.SetFileName(DATA_PATH + m_ptUnit->m_tContralUnit.strUnitName + LINE_SCAN_PARAM);
	opini2.SetSectionName("CurUseTableNo");
	opini2.ReadString("CurUseTableNo", &nCurUseTable);

	//线扫
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


		//SaveContourData();	//将所有点云数据保存到一个文件中
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
			//	// 工字钢 小件接口 
			//	pLineOrCircularArcWeldingLine = SmallSamplePointCloudToWeldingLines(
			//		(Three_DPoint*)pPointCloud, PointCloudSize, &nWeldingSeamNumber, bInstallDir, m_dZPallet, 5.0, 25, 60,80,  false, false, 8/*, true, false, 0, false*/);
			//}
			//else 
			//{
			//	//// 工字钢 牛腿接口
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
			//// 圆弧
			/*pLineOrCircularArcWeldingLine = ArcPlatePointCloudToWeldingLinesExtraction(
				(Three_DPoint*)pPointCloud, PointCloudSize, &nWeldingSeamNumber,
				bInstallDir, true, m_dZPallet, 5.0, false, true, false, 15, 4, 1);*/
				// 工字钢 小件接口 
			pLineOrCircularArcWeldingLine = SmallSamplePointCloudToWeldingLines(
				(Three_DPoint*)pPointCloud, PointCloudSize, &nWeldingSeamNumber, bInstallDir, m_dZPallet, 5.0, 12, 60, 80, false, false, 8/*, true, false, 0, false*/);
			SavePointCloudProcessResult((LineOrCircularArcWeldingLine*)pLineOrCircularArcWeldingLine, nWeldingSeamNumber);
			ReleaseWeldingLines(&pLineOrCircularArcWeldingLine);
			/*LoadCloudProcessResultNew(OUTPUT_PATH + m_ptUnit->m_tContralUnit.strUnitName + "\\" + POINT_CLOUD_IDENTIFY_RESULT);
			Segmentation();*/
#if 0
			std::string ModelFileName1 = OUTPUT_PATH + "ModelMatch1\\样件202240409_2.txt";// 原始理想模型
			std::string AllModelFileRoute = OUTPUT_PATH + "ModelMatch1";
			std::string SaveRoute_weldline_point = ".\\LocalFiles\\OutputFiles\\RobotA\\Recognition\\SaveRoute_weldline_point.txt";
			//FigureAllTemplateTransformMatrix
			int nMatrixNum = 0;
			TemplateTransformMatrix* pTemplateTransMat = FigureAllTemplateTransformMatrix((Three_DPoint*)pPointCloud, PointCloudSize,
				&nMatrixNum, bInstallDir, true, m_dZPallet, 5.0, 5, true, 20.0, AllModelFileRoute.c_str(), SaveRoute_weldline_point.c_str(), 4);

			double dCoarseMatrix[4][4];
			double dFineMatrix[4][4];
			//FILE* pf = fopen("TemplateTransformMatrix.txt", "w");
			// 原始矩阵赋值
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

			// 精确矩阵赋值
			for (int i = 0; i < 4; i++)
			{
				for (int j = 0; j < 4; j++)
				{
					pTemplateTransMat[0].Transform_Matrix[i][j] = dFineMatrix[i][j];
				}
			}

			std::string RealModelFileName1 = OUTPUT_PATH + "ModelMatch2"; // 真实要焊接模型
			std::string SaveRoute_AllWeldingLine  = ".\\LocalFiles\\OutputFiles\\RobotA\\Recognition\\PointCloudIdentifyReault.txt"; //输出结果路径
			TransformAllTemplatesToReal(RealModelFileName1.c_str(), pTemplateTransMat, nMatrixNum, SaveRoute_AllWeldingLine.c_str());
			return true;
#endif // 0
		}
		else if (STIFFEN_PLATE == m_eWorkPieceType)
		{
			//// 腾龙吊车梁
			//LineOrCircularArcWeldingSeam* pLineOrCircularArcWeldingSeam;
			//WeldingSeamGroup* pWeldingSeamGroup;
			//pWeldingSeamGroup = RecognizeWeldingSeamsAccordingToRule1(
			//	pPointCloud, PointCloudSize, &nWeldingSeamNumber, m_dSideBoardThick,
			//	m_dZPallet, true, 8, 1000);
			//SavePointCloudProcessResult(pLineOrCircularArcWeldingSeam, nWeldingSeamNumber);
			//ReleaseLineOrCircularArcWeldingSeamsPointer(&pLineOrCircularArcWeldingSeam);
			// 李伟吊车梁
			pLineOrCircularArcWeldingLine = BeamPointCloudToWeldingLines((Three_DPoint*)pPointCloud, PointCloudSize, &nWeldingSeamNumber, bInstallDir, m_dZPallet, 5.0);
			SavePointCloudProcessResult((LineOrCircularArcWeldingLine*)pLineOrCircularArcWeldingLine, nWeldingSeamNumber);
			ReleaseWeldingLines(&pLineOrCircularArcWeldingLine);
		}
		else if (E_CORBEL == m_eWorkPieceType)
		{
			// 李伟识别接口 牛腿
			//pLineOrCircularArcWeldingLine = BracketPointCloudToWeldingLines(
			//	(Three_DPoint*)pPointCloud, PointCloudSize, &nWeldingSeamNumber, bInstallDir, m_dZPallet, 5.0, false);
			//SavePointCloudProcessResult((LineOrCircularArcWeldingLine*)pLineOrCircularArcWeldingLine, nWeldingSeamNumber);
			//ReleaseWeldingLines(&pLineOrCircularArcWeldingLine);
		}
		else if (E_END_PLATE == m_eWorkPieceType)
		{
			// 函数返回值为焊缝信息结构体的指针
			// 李伟识别接口 端板  屋面梁:筋板 檩托 端板 隅撑
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
			//((CButton*)GetDlgItem(按钮ID))->EnableWindow(TRUE);
			pLineOrCircularArcWeldingLine = GroovePointCloudToWeldingLines(
				(Three_DPoint*)pPointCloud, PointCloudSize, &nWeldingSeamNumber, bInstallDir, m_dZPallet, m_dSideBoardThick);
			SavePointCloudProcessResult((LineOrCircularArcWeldingLine*)pLineOrCircularArcWeldingLine, nWeldingSeamNumber);
			ReleaseWeldingLines(&pLineOrCircularArcWeldingLine);
		}
		else
		{
			XiMessageBox("通用类：类型错误 处理失败！");
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

bool GenericWeld::WeldSeamGrouping(int& nWeldGroupNum)
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
	double dMinStandSeamLen = 10.0; // 最小立焊长度
	m_vvtWeldSeamGroup.clear();
	// 区分平焊立焊 及 平焊起点终点排序
	vector<WeldLineInfo> vtWeldSeamPlane(0);
	vector<WeldLineInfo> vtWeldLineStandard(0);
	vector<WeldLineInfo> vtWeldLineArc(0);
	vector<WeldLineInfo> vtWeldLinePlaneOverLength(0);
	vector<CvPoint3D64f> vtPtn(0); // 平焊分组使用(合并重合的点)
	vector<vector<int>> vvnWeldNo(0); // 平焊分组使用(每个合并点所关联的多个焊缝编号号)
	XiAlgorithm alg;
	CheckSeamDataEndpointType(dGroupDisThreshold); // 检查端点类型是否正确(避免干涉识别为自由)
	for (int nWeldNo = 0; nWeldNo < m_vtWeldSeamInfo.size(); nWeldNo++)
	{
		WeldLineInfo &tLineSeamInfo = m_vtWeldSeamInfo[nWeldNo];
		LineOrCircularArcWeldingLine &tLineSeam = tLineSeamInfo.tWeldLine;
		double dWeldSeamLen = TwoPointDis(
			tLineSeam.StartPoint.x, tLineSeam.StartPoint.y, tLineSeam.StartPoint.z,
			tLineSeam.EndPoint.x, tLineSeam.EndPoint.y, tLineSeam.EndPoint.z);
		tLineSeamInfo.tAtrribute.bWeldMode = false; // 默认不跟踪
		if (IsStandWeldSeam(tLineSeam.StartNormalVector)) // 立焊起点终点法向量Z值为0
		{
			if (dWeldSeamLen < dMinStandSeamLen)
			{
				//已修改
				XUI::MesBox::PopInfo("立焊焊缝{0} 长度小于{1:.1f},自动删除！", nWeldNo, dMinStandSeamLen);
				//XiMessageBoxOk("立焊焊缝%d 长度小于%.1lf,自动删除！", nWeldNo, dMinStandSeamLen);
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
				//已修改
				XUI::MesBox::PopInfo("平焊焊缝{0} 长度小于{1:.1f},自动删除！", nWeldNo, dMinFlatSeamLen);
				//XiMessageBoxOk("平焊焊缝%d 长度小于%.1lf,自动删除！", nWeldNo, dMinFlatSeamLen);
				continue;
			}
			if (dWeldSeamLen < m_dShortSeamThreshold &&
				0 < tLineSeam.StartPointType &&
				0 < tLineSeam.EndPointType)
			{
				//已修改
				XUI::MesBox::PopInfo("双侧干涉平焊焊缝{0} 长度小于{1:.1f},自动删除！", nWeldNo, m_dShortSeamThreshold);
				//XiMessageBoxOk("双侧干涉平焊焊缝%d 长度小于%.1lf,自动删除！", nWeldNo, m_dShortSeamThreshold);
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

	Sleep(50); // 确保运行 WinExec("del_pic.bat", SW_SHOW);后
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
	m_vvtCowWeldSeamGroupAdjust = m_vvtWeldSeamGroup;
	SaveWeldSeamGroupInfo();
	return true;
}

bool GenericWeld::CalcMeasureTrack(int nGroupNo, std::vector<T_ROBOT_COORS>& vtMeasureCoord, std::vector<T_ANGLE_PULSE>& vtMeasurePulse, vector<int>& vnMeasureType, double& dExAxlePos, double& dSafeHeight)
{
	double dExAxlePos_y;

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

	//double dTotalValueY = 0.0; // 统计Y值 确定外部轴位置
	CvPoint3D64f tTotalValue = {0}; // 统计XYZ值 确定外部轴位置
	CvPoint3D64f tTotalDirValue = {0};
	double dPartMaxHeight = -99999.0; // 正座记录工件最高点Z值 倒挂记录工件最低点
	double dPartMinHeight = 99999.0; // 正座记录工件最底点Z值 倒挂记录工件最高点
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
	dExAxlePos = tTotalValue.y / (vtSeamGroup.size() * 2); // 外部轴位置:注意此外部轴位置可能被后续计算更新
#else
	dExAxlePos = tTotalValue.x / (vtSeamGroup.size() * 2); // 外部轴位置:注意此外部轴位置可能被后续计算更新
	dExAxlePos_y = 0.0;
#endif // SINGLE_ROBOT

	// 获取龙门坐标

	// 查找所有组焊缝中心x值 总和
	double dAllWeldTotalX = 0.0;   ///小部件没用 小组立用
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
		// 获取龙门坐标
		CHECK_BOOL_RETURN(GetGantryPos(vtSeamGroup, dExAxlePos, dPartMaxHeight,
			m_vvtWeldLineInfoGroup[nGroupNo].at(0).tAtrribute.dGroupTrackPos, dPartMinHeight, tTotalValue, tTotalDirValue));
	}

	// 搜索示教数据先单独保存
	vector<T_TEACH_DATA> vtSearchTeachData;
	vtSearchTeachData.clear();
	std::vector<T_ROBOT_COORS> vtMeasureCoordGunTool(0); // 合并后的焊枪工具坐标
 	for (int nSeamNo = 0; nSeamNo < vtSeamGroup.size(); nSeamNo++)
	{
		vtTeachData.clear();
		//在这个函数 CalcSeamTeachData 里面增加通过平焊的起点终点的长度来多加多少个测量点
 		CHECK_BOOL_RETURN(CalcSeamTeachData(nGroupNo, nSeamNo, dExAxlePos, vtSeamGroup, vtTeachData));
		//SaveTeachDataEverySeam(nGroupNo, nSeamNo, dExAxlePos, vtTeachData); // 调试使用		
		for (int nNewTeachPtnNo = 0; nNewTeachPtnNo < vtTeachData.size(); nNewTeachPtnNo++) // 记录每个测量点信息
		{
			tTeachData = vtTeachData[nNewTeachPtnNo];
			// 搜索端点先单独保存
			if (E_SEARCH_POINT & tTeachData.nMeasureType)
			{
				vtSearchTeachData.push_back(tTeachData);
				continue;
			}
			int nIdx = CheckTeachCoord(tTeachData.tMeasureCoordGunTool, vtMeasureCoordGunTool, tThersholdCoord);
			// 不存在该测量坐标 或 测量点属于搜索类型 或 搜索到的第nIdx个点是搜索点 添加新测量坐标
			if ((-1 == nIdx))
			{
				tTeachData.nMeasurePtnNo = vtMeasureCoord.size();
				vtMeasureCoordGunTool.push_back(tTeachData.tMeasureCoordGunTool);
				//vtMeasureCoord.push_back(tTeachData.tMeasureCoordCamTool);
				vtMeasureCoord.push_back(tTeachData.tMeasureCoordGunTool); // 焊枪坐标 后续计算连续轨迹是转相机坐标
				vnMeasureType.push_back(tTeachData.nMeasureType);
			}
			else  // 存在该测量坐标
			{
				tTeachData.nMeasurePtnNo = nIdx;
				tTeachData.tMeasureCoordCamTool = vtMeasureCoord[nIdx];
			}
			m_vtTeachData.push_back(tTeachData);
		}


		
	}
 	int m_vtTeachDataSize = m_vtTeachData.size();
	int nMeasureTypeSize = vnMeasureType.size();
	// 保存搜索测量数据
	for (int nNo = 0; nNo < vtSearchTeachData.size(); nNo++)
	{
		tTeachData = vtSearchTeachData[nNo];
		tTeachData.nMeasurePtnNo = vtMeasureCoord.size();
		vtMeasureCoordGunTool.push_back(tTeachData.tMeasureCoordGunTool);
		//vtMeasureCoord.push_back(tTeachData.tMeasureCoordCamTool);
		vtMeasureCoord.push_back(tTeachData.tMeasureCoordGunTool); // 焊枪坐标 后续计算连续轨迹是转相机坐标
		vnMeasureType.push_back(tTeachData.nMeasureType);
		m_vtTeachData.push_back(tTeachData);
	}

	// 保存测量轨迹焊枪工具坐标
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


	// 外部轴位置偏移 : 0.0  ±100.0  ±200.0  ±300.0 ……
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
				WriteLog("外部轴%d 目标位置%.3lf 超出设定极限%.3lf - %.3lf!", m_ptUnit->m_nMeasureAxisNo, vdExAxleOffset[nOffsetIdx], dMinExAxlePos, dMaxExAxlePos);
				continue;
			}

			if (-1 != m_ptUnit->m_nMeasureAxisNo_up)
			{
				if (vdExAxleOffset_y[nOffsetIdx_y] + dTempExAxlePos_y > dMaxExAxlePos_y || vdExAxleOffset_y[nOffsetIdx_y] + dTempExAxlePos_y < dMinExAxlePos_y)
				{
					WriteLog("外部轴%d 目标位置%.3lf 超出设定极限%.3lf - %.3lf!", m_ptUnit->m_nMeasureAxisNo_up, vdExAxleOffset_y[nOffsetIdx_y], dMinExAxlePos_y, dMaxExAxlePos_y);
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

			// 测量激光与相机视野避障检测
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
			// 添加收枪 下枪坐标
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
			if (false == CheckFlangeToolCoord(vtTempPulse)) // 检查所欲测量轨迹点 距离法兰距离是否足够大
			{
				WriteLog("ExAxleOffset:%3lf CalcContinuePulseForWeld Fail!", vdExAxleOffset[nOffsetIdx]);
				WriteLog("ExAxleOffset_y:%3lf CalcContinuePulseForWeld Fail!", vdExAxleOffset_y[nOffsetIdx_y]);
				continue;
			}

			// 按点云识别结果生成纯理论焊缝信息 并 计算是否可以连续运行
			// 保证测量时和焊接是外部轴及小车位置相同 (焊接时仍可能少量移动) 测量轨迹和焊接轨迹都可以连续运行时才成功
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
						(vtMeasurePulse[0].nTPulse - tCurAnglePulse.nTPulse) / 2, 0, 0, 0);
					m_pRobotDriver->RobotKinematics(tSafeDownGunPulse, m_pRobotDriver->m_tTools.tGunTool, tSafeDownCoord);
					vtMeasurePulse.insert(vtMeasurePulse.begin(), tSafeDownGunPulse);
					vtMeasureCoord.insert(vtMeasureCoord.begin(), tSafeDownCoord);
					vnMeasureType.insert(vnMeasureType.begin(), E_TRANSITION_POINT);
				}
			}
			for (int i = 0; i < vtMeasureCoord.size(); i++) // 将外部轴坐标和机器人坐标 结合
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
			WriteLog("ExAxleOffset:%3lf 计算连续测量运动轨迹成功", vdExAxleOffset[nOffsetIdx]);
		}
	}
	WriteLog("迭代次数：%d\n", continuePC);
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

	// 离线调试使用
	CString sJobName;
	sJobName.Format("MOVJTEACHBOARD%d", nGroupNo);
	GenerateJobLocalVariable(vtMeasurePulse, MOVJ, sJobName);
	//GenerateFilePLY(nGroupNo, dExAxlePos); // 调试使用
	return true;
}


void GenericWeld::GetWeldLineDir(std::vector<T_ROBOT_COORS> vtMeasureCoord, std::vector<T_LINE_DIR_3D>& vtWeldLineDir)
{
	T_LINE_DIR_3D tWeldLineDir;

	// ljx 临时
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

				// 板厚
				double boardThick = 20.0;

				// 板厚方向
				double startAngleForBoard = RzToDirAngle(alg.CalcArcAngle(
					tSeam.StartNormalVector.x,
					tSeam.StartNormalVector.y)) - 180.0 * m_nRobotInstallDir;
				double endAngleForBoard = RzToDirAngle(alg.CalcArcAngle(
					tSeam.EndNormalVector.x,
					tSeam.EndNormalVector.y)) - 180.0 * m_nRobotInstallDir;

				// 添加板厚
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
					tSeam.isLeft ? tSeam.ZSide : tSeam.EndPoint.z; // ZSide 在左?
				double dAdjoinSeam2Height = true == 
					tSeam.isLeft ? tSeam.EndPoint.z : tSeam.ZSide;
				double dRobotChangeDir = 1 == m_nRobotInstallDir ? 1.0 : -1.0;
				double dAdjoinOffsetDis = 250.0;
				double dNorAngle = alg.CalcArcAngle(
					tSeam.StartNormalVector.x, 
					tSeam.StartNormalVector.y);
				CvPoint3D64f tPtn = tSeam.StartPoint;
				double dAdjoinSeamNorAngle = dNorAngle + (45.0 * dRobotChangeDir);	// 测量点rz
				double dOffsetDir = dNorAngle - (45.0 * dRobotChangeDir);			// 计算测量点偏移角度
				tCoord = GenerateRobotCoord(tPtn, m_dPlatWeldRx, m_dPlatWeldRy, DirAngleToRz(dAdjoinSeamNorAngle));
				RobotCoordPosOffset(tCoord, dOffsetDir, dAdjoinOffsetDis); // 终点与立焊起点相接的平焊上测量位置无需手眼偏移

				blockPlate.tDownEndPoint.x = tCoord.dX;
				blockPlate.tDownEndPoint.y = tCoord.dY;
				blockPlate.tDownEndPoint.z = tCoord.dZ;

				blockPlate.tUpEndPoint.x = tCoord.dX;
				blockPlate.tUpEndPoint.y = tCoord.dY;
				blockPlate.tUpEndPoint.z = dAdjoinSeam1Height;

				vtCheckBlockPlate.push_back(blockPlate);

				// 板厚
				double boardThick = 20.0;

				// 板厚方向
				double startAngleForBoard = dOffsetDir - 90.0 * dRobotChangeDir;
				double endAngleForBoard = dOffsetDir - 90.0 * dRobotChangeDir;

				// 添加板厚
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
				RobotCoordPosOffset(tCoord, dOffsetDir, dAdjoinOffsetDis + m_dHandEyeDis); // 起点与立焊起点相连的平焊上测量点需要添加手眼偏移距离


				blockPlate.tDownEndPoint.x = tCoord.dX;
				blockPlate.tDownEndPoint.y = tCoord.dY;
				blockPlate.tDownEndPoint.z = tCoord.dZ;

				blockPlate.tUpEndPoint.x = tCoord.dX;
				blockPlate.tUpEndPoint.y = tCoord.dY;
				blockPlate.tUpEndPoint.z = dAdjoinSeam2Height;

				vtCheckBlockPlate.push_back(blockPlate);

				// 板厚
				boardThick = 20.0;

				// 板厚方向
				startAngleForBoard = dOffsetDir - 90.0 * dRobotChangeDir;
				endAngleForBoard = dOffsetDir - 90.0 * dRobotChangeDir;

				// 添加板厚
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
	if (nTeachResultNum != m_nTeachPtnNum)
	{
		XiMessageBox("示教点数不相同");
		return false;
	}
	// 演示所有工件进入一下函数计算理论轨迹
	if (DetermineGroupMode(nGroupNo))
	{
		// 组内存在跟踪焊缝,计算跟踪组焊缝轨迹
		CHECK_BOOL_RETURN(CalcAndTrackWeldingTry(nGroupNo));
		return true;
	}
	// 将 m_vtTeachResult 按 m_vnTeachTrackOrder 恢复顺序
	CHECK_BOOL_RETURN(RestoreOrderTeachResult());

	// 使用测量数据修正焊缝信息(与识别数据对比，相差过大不能焊接)
	CHECK_BOOL_RETURN(WeldSeamAdjust(nGroupNo));

	// 生成焊接轨迹文件
	CHECK_BOOL_RETURN(GenerateWeldLineData(nGroupNo, m_dWeldHoleSize, 0.0));

	return true;
}

bool GenericWeld::SortMeasureCoord(double dPartHeight, double dFirstSeamNorAngle, double dExAxleOffsetDis, 
	double &dExAxlePos, vector<T_ROBOT_COORS>& vtMeasureCoord, vector<int> &vnMeasureType, vector<int>& vtSortIdx)
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
	int nRzChangeDir = 1 == m_nRobotInstallDir ? -1 : 1; // -1:焊缝间顺序测量Rz减小  1:Rz增加
	double dStartDirAngle = dFirstSeamNorAngle; // 根据输入角度决定从哪个焊缝开始测量
	double dEndDirAngle = dStartDirAngle + 360.0;
	double dHomePulseRz = 0.0;

	dExAxlePos += dExAxleOffsetDis; // 使用外部轴偏移 修改外部轴坐标
	for (int nIdx = 0; nIdx < vtMeasureCoord.size(); nIdx++)
	{
		vtMeasureCoord[nIdx].dY -= dExAxleOffsetDis; // 使用外部轴偏移 修改机器人坐标
		// Rz转换到 dStartDirAngle - dEndDirAngle 范围内 后续排序使用
		vtMeasureCoord[nIdx].dRZ = fmod(vtMeasureCoord[nIdx].dRZ, 360.0);
		vtMeasureCoord[nIdx].dRZ = vtMeasureCoord[nIdx].dRZ > (dStartDirAngle) ? vtMeasureCoord[nIdx].dRZ - 360.0 : vtMeasureCoord[nIdx].dRZ;
		vtMeasureCoord[nIdx].dRZ = vtMeasureCoord[nIdx].dRZ < (dEndDirAngle) ? vtMeasureCoord[nIdx].dRZ + 360.0 : vtMeasureCoord[nIdx].dRZ;
	}

	// 将vtMeasureCoord按照Rz递增排序 并 获取排序后 序号索引
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
		if (fabs(tPreCoord.dRZ - tCurCoord.dRZ) > 100.0) // 相邻测量点Rz相差超过100°
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
		vtMeasureCoord[nIdx].dX -= dExAxleOffsetDis; // 使用外部轴偏移 修改机器人坐标
#else
		vtMeasureCoord[nIdx].dY -= dExAxleOffsetDis; // 使用外部轴偏移 修改机器人坐标
#endif // SINGLE_ROBOT

		//// 焊枪工具 转 跟踪相机工具
		//if (false == m_pRobotDriver->MoveToolByWeldGun(vtMeasureCoord[nIdx], m_pRobotDriver->m_tTools.tGunTool,
		//	vtMeasureCoord[nIdx], m_pRobotDriver->m_tTools.tCameraTool, vtMeasureCoord[nIdx]))
		//{
		//	m_pRobotDriver->m_cLog->Write("Robot %d 第%d个测量点 转换失败！", m_pRobotDriver->m_nRobotNo, nIdx);
		//	return false;
		//}

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

	//// 外圈焊缝重新排序
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
	//		XiMessageBoxOk("坐标转换失败bOutsideWeld");
	//		return false;
	//	}
	//	double dDirAngle2 = alg.CalcArcAngle(tCoord2.dX - tCoord1.dX, tCoord2.dY - tCoord1.dY);
	//	if (!JudgeDirAngle(dDirAngle1, dDirAngle2, 95.0)) // 外圈焊缝 各角度测量组颠倒
	//	{
	//		bOutsideWeld = true;
	//		for (int i = 0; i < vvtCoord.size() / 2; i++)
	//		{
	//			swap(vvtCoord[i], vvtCoord[vvtCoord.size() - 1 - i]);
	//		}
	//	}
	//}

	//// 每组路径排序
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
	//		tPtn.x = vtCoord[0].tCoord.dX + 999999.0 * CosD(dMoveDirAngle); // 生成排序辅助点
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
	//				if (dCurLevelDis < dMinLevelDis && dCurLevelDis < 20.0) // 变姿态，导致没有正上方坐标
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

	//// 相邻测量位置姿态变化过大添加过渡点
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
	//		(false == JudgeAngle(180.0, -180.0, tPreCoord.dRZ, tCurCoord.dRZ, 45.0))) // 外圈焊缝 Rz变化超过45°
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
	//		(dDis > 200.0))// 相邻测量点Rz相差超过100°
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
		double dDis = TwoPointDis(
			tPreCoord.dX + tPreCoord.dBX, tPreCoord.dY + tPreCoord.dBY,
			tCurCoord.dX + tCurCoord.dBX, tCurCoord.dY + tCurCoord.dBY);
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
		else if (false == JudgeAngle(180.0, -180.0, tPreCoord.dRZ, tCurCoord.dRZ, 100.0) ||
			(dDis > 200.0))// 相邻测量点Rz相差超过100°
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
		XiMessageBox("测量点数量和测量类型数量不匹配，排序失败");
		return false;
	}

	dExAxlePos += dExAxleOffsetDis; // 使用外部轴偏移 修改外部轴坐标
	for (int nIdx = 0; nIdx < vtMeasureCoord.size(); nIdx++)
	{
		vtSortIdx.push_back(nIdx);
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
	}
	return true;
}

bool GenericWeld::RestoreOrderTeachResult()
{
	int nTeachResultNum = m_vtTeachResult.size();
	int nOrderIdxNum = m_vnTeachTrackOrder.size();
	if (nTeachResultNum != nOrderIdxNum)
	{
		XUI::MesBox::PopOkCancel("测量结果和测量位置数量不符！{0} {1}", nTeachResultNum, nOrderIdxNum);
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
	if (NULL != ptAdjoinSeamS) // 测量姿态Rz考虑相邻平焊角度 避免干涉
	{
		double dAdjoinSeamDir = alg.CalcArcAngle(
			ptAdjoinSeamS->StartPoint.x - ptAdjoinSeamS->EndPoint.x,
			ptAdjoinSeamS->StartPoint.y - ptAdjoinSeamS->EndPoint.y);
		if (JudgeAngle(360.0, 0.0, dAdjoinSeamDir, dStoEDir, dMinAcuteAngle))
		{
			//已修改
			XUI::MesBox::PopInfo("立板件锐角小于{0:.3f}°，无法焊接！", dMinAcuteAngle);
			//XiMessageBox("立板件锐角小于%.3lf°，无法焊接！", dMinAcuteAngle);
			return false;
		}
		if (JudgeAngle(360.0, 0.0, dAdjoinSeamDir, dStoEDir, dMaxAcuteAngle)) // 锐角处测量焊枪Rz以相邻焊缝方向为准
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
			//已修改
			XUI::MesBox::PopInfo("立板件锐角小于{0:.3f}°，无法焊接！", dMinAcuteAngle);
			//XiMessageBox("立板件锐角小于%.3lf°，无法焊接！", dMinAcuteAngle);
			return false;
		}
		if (JudgeAngle(360.0, 0.0, dAdjoinSeamDir, dEtoSDir, dMaxAcuteAngle)) // 锐角处测量焊枪Rz以相邻焊缝方向为准
		{
			dNorAngleE = dAdjoinSeamDir;
			WriteLog("立板件锐角小于%.3lf°，无法焊接！", dMinAcuteAngle);
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
	if (NULL != ptAdjoinSeamS) // 测量姿态Rz考虑相邻平焊角度 避免干涉
	{
		double dAdjoinSeamDir = alg.CalcArcAngle(
			ptAdjoinSeamS->StartPoint.x - ptAdjoinSeamS->EndPoint.x,
			ptAdjoinSeamS->StartPoint.y - ptAdjoinSeamS->EndPoint.y);
		if (JudgeAngle(360.0, 0.0, dAdjoinSeamDir, dStoEDir, dMinAcuteAngle))
		{
			//已修改
			XUI::MesBox::PopInfo("立板件锐角小于{0:.3f}°，无法焊接！", dMinAcuteAngle);
			//XiMessageBox("立板件锐角小于%.3lf°，无法焊接！", dMinAcuteAngle);
			return false;
		}
		if (JudgeAngle(360.0, 0.0, dAdjoinSeamDir, dStoEDir, dMaxAcuteAngle)) // 锐角处测量焊枪Rz以相邻焊缝方向为准
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
			//已修改
			XUI::MesBox::PopInfo("立板件锐角小于{0:.3f}°，无法焊接！", dMinAcuteAngle);
			//XiMessageBox("立板件锐角小于%.3lf°，无法焊接！", dMinAcuteAngle);
			return false;
		}
		if (JudgeAngle(360.0, 0.0, dAdjoinSeamDir, dEtoSDir, dMaxAcuteAngle)) // 锐角处测量焊枪Rz以相邻焊缝方向为准
		{
			dNorAngleE = dAdjoinSeamDir;
			WriteLog("立板件锐角小于%.3lf°，无法焊接！", dMinAcuteAngle);
		}
#endif // 0
	double dHandEyeDis = true == bOffsetHandEyeDis ? m_dHandEyeDis : 0.0;
	if (bIsStart)
	{
		// 圆弧定长搜起点，待优化
		tCoord = GenerateRobotCoord(tSeam.StartPoint, m_dPlatWeldRx, m_dPlatWeldRy, DirAngleToRz(dNorAngleS));
		RobotCoordPosOffset(tCoord, dOffsetDirS, dOfficeDis * 1.0 + dHandEyeDis);//搜索距离
		CalcCorvrAngle(true, !tSeam.StartPointType, !tSeam.EndPointType, tCoord, dSeamLen);
		vtMeasureCoord.push_back(tCoord);

		tCoord = GenerateRobotCoord(tSeam.StartPoint, m_dPlatWeldRx, m_dPlatWeldRy, DirAngleToRz(dNorAngleS));
		RobotCoordPosOffset(tCoord, dOffsetDirS, -10.0 + dHandEyeDis);//搜索距离
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
		RobotCoordPosOffset(tCoord, dOffsetDirE, /*dOfficeDis*/-10.0);//搜索距离
		CalcCorvrAngle(false, !tSeam.EndPointType, !tSeam.StartPointType, tCoord, dSeamLen);
		vtMeasureCoord.push_back(tCoord);
		
	}
	return true;
}

bool GenericWeld::CalcSeamTeachData(int nGroupNo, int nSeamNo, double dExAxlePos, vector<LineOrCircularArcWeldingLine> vtSeamGroup, vector<T_TEACH_DATA>& vtTeachData)
{
	/*
	 * 不同端点测量分为一下几种形式：
	 * 1、平焊 起点 自由端点 (搜索？测棱角？)
	 * 2、平焊 起点 干涉端点
	 * 3、平焊 终点 自由端点 (搜索？测棱角？)
	 * 4、平焊 终点 干涉端点
	 * 5、立焊 起点 自由端点 (檩托倒放长立焊 暂不考虑)
	 * 6、立焊 起点 干涉端点
	 * 7、立焊 终点 干涉端点
	 * 8、封闭圆焊缝
	 * 9、圆弧 起点 终点 中间测量
	 */

	//if (IsStandWeldSeam(vtSeamGroup[nSeamNo].StartNormalVector))
	//{
	//	double sZside = 0;
	//	double eZside = 0;
	//	double sMeasureDis = 0;
	//	double eMeasureDis = 0;


	//}

	if (10 == m_vvtWeldLineInfoGroup[nGroupNo][nSeamNo].tAtrribute.nStartWrapType) // 扫描局部点云提取焊缝，仅支持单个焊缝
	{
		return CalcScanMeasureTeachData(nGroupNo, nSeamNo, dExAxlePos, vtSeamGroup, vtTeachData);
	}
	else if (DetermineGroupMode(nGroupNo) || WeldingLineIsArc(vtSeamGroup[nSeamNo])) // 圆圈先测后焊,闭合圆弧跟踪，自由曲线跟踪
	{
		return CalcSeamArcTeachData(nGroupNo, nSeamNo, dExAxlePos, vtSeamGroup, vtTeachData);
	}
	double dSearchDis = m_dEndpointSearchDis; // 自由端搜索长度距离
	vtTeachData.clear();
	XiAlgorithm alg;
	double dAngleChangeDir = 1 == m_nRobotInstallDir ? 1.0 : -1.0;///计算测量位置收缩方向
	//T_TEACH_DATA tTeachData;
	vector<T_TEACH_DATA> vtTempTeachData;
	LineOrCircularArcWeldingLine tSeam = vtSeamGroup[nSeamNo];
	bool bIsFlatWeld = !IsStandWeldSeam(tSeam.StartNormalVector);  //true为平焊 false为立焊
	bool bFreeStartPoint = !tSeam.StartPointType;
	bool bFreeEndPoint = !tSeam.EndPointType;
	double dNorAngleS = alg.CalcArcAngle(tSeam.StartNormalVector.x, tSeam.StartNormalVector.y);
	double dNorAngleE = alg.CalcArcAngle(tSeam.EndNormalVector.x, tSeam.EndNormalVector.y);
	LineOrCircularArcWeldingLine *ptAdjoinSeamS = NULL; //&tAdjoinSeamS; // 平焊起点相邻的平焊缝 或 起点与立焊起点相邻的平焊缝
	LineOrCircularArcWeldingLine *ptAdjoinSeamE = NULL; //&tAdjoinSeamE; // 平焊终点相邻的平焊缝 或 终点与立焊起点相邻的平焊缝
	GetAdjoinSeam(nSeamNo, vtSeamGroup, &ptAdjoinSeamS, &ptAdjoinSeamE);


	double dShortSeamThreshold = m_dShortSeamThreshold; //测量起点终点的测量点的偏移距离
	if (bIsFlatWeld) {
		CalcFlatSeamMeasurementEnhancement(nSeamNo, bIsFlatWeld, true, tSeam, dNorAngleS, dExAxlePos, ptAdjoinSeamS, vtTempTeachData);
	}
	AppendTeachData(vtTeachData, vtTempTeachData);
	// 焊缝起点测量数据
	if (bIsFlatWeld && bFreeStartPoint)			// 起点 平焊 自由端点
	{
		CHECK_BOOL_RETURN(CalcFreeTeachDataFlat(nGroupNo, nSeamNo, bIsFlatWeld, true, tSeam, dNorAngleS, dSearchDis, dExAxlePos, vtTempTeachData));
	}
	else if (bIsFlatWeld && !bFreeStartPoint)	// 起点 平焊 干涉端点
	{
		// 黄埔 更改先测后焊干涉段获取端点方式
		//CHECK_BOOL_RETURN(CalcInterfereTeachDataFlatFix(nSeamNo, bIsFlatWeld, true, vtSeamGroup, dNorAngleS, dExAxlePos, vtTempTeachData));
		// 原有流程
		CHECK_BOOL_RETURN(CalcInterfereTeachDataFlat(nSeamNo, bIsFlatWeld, true, vtSeamGroup, dNorAngleS, dExAxlePos, vtTempTeachData));
	}
	else if (!bIsFlatWeld && !bFreeStartPoint)	// 起点 立焊 干涉端点
	{
		CHECK_BOOL_RETURN(CalcTeachDataStand(nSeamNo, bIsFlatWeld, true, vtSeamGroup, dExAxlePos, vtTempTeachData));
	}
	else if (!bIsFlatWeld && bFreeStartPoint)	// 起点 立焊 自由端点
	{
		CHECK_BOOL_RETURN(CalcTeachDataStandFreeStart(nSeamNo, bIsFlatWeld, true, tSeam, dExAxlePos, ptAdjoinSeamS, ptAdjoinSeamE, vtTempTeachData));
	}
	else 
	{
		//已修改
		XUI::MesBox::PopInfo("起点测量位置计算失败！焊缝类型{0} 终点类型{1}", bIsFlatWeld, bFreeStartPoint);
		//XiMessageBox("起点测量位置计算失败！焊缝类型%d 起点类型%d", bIsFlatWeld, bFreeStartPoint);
		return false;
	}
	AppendTeachData(vtTeachData, vtTempTeachData);

	// 焊缝终点测量数据
	if (bIsFlatWeld && bFreeEndPoint)			// 终点 平焊 自由端点
	{
		CHECK_BOOL_RETURN(CalcFreeTeachDataFlat(nGroupNo, nSeamNo, bIsFlatWeld, false, tSeam, dNorAngleE, dSearchDis, dExAxlePos, vtTempTeachData));
	}
	else if (bIsFlatWeld && !bFreeEndPoint)		// 终点 平焊 干涉端点
	{
		// 黄埔 更改先测后焊干涉段获取端点方式
		//CHECK_BOOL_RETURN(CalcInterfereTeachDataFlatFix(nSeamNo, bIsFlatWeld, false, vtSeamGroup, dNorAngleS, dExAxlePos, vtTempTeachData));
		// 原有流程
		CHECK_BOOL_RETURN(CalcInterfereTeachDataFlat(nSeamNo, bIsFlatWeld, false, vtSeamGroup, dNorAngleE, dExAxlePos, vtTempTeachData));
	}
	else if (!bIsFlatWeld && bFreeEndPoint)		// 终点 立焊 自由端点
	{
		CHECK_BOOL_RETURN(CalcTeachDataStand(nSeamNo, bIsFlatWeld, false, vtSeamGroup, dExAxlePos, vtTempTeachData));
	}
	else
	{
		//已修改
		XUI::MesBox::PopInfo("终点测量位置计算失败！焊缝类型{0} 终点类型{1}", bIsFlatWeld, bFreeEndPoint);
		//XiMessageBox("起点测量位置计算失败！焊缝类型%d 终点类型%d", bIsFlatWeld, bFreeEndPoint);
		return false;
	}
	AppendTeachData(vtTeachData, vtTempTeachData);

	//for (int i = 0; i < vtTeachData.size(); i++)
	//{
	//	vtTeachData[i].tMeasureCoordGunTool.dRY -= 10.0;
	//}

	//// 调试使用
	//CString sFileName;
	//sFileName.Format("%s%d-%d-MeasureTrackGunTool.txt", m_sDataSavePath, nGroupNo, nSeamNo);
	//FILE *CoutFile = fopen(sFileName.GetBuffer(0), "w");
	//for (int nNo = 0; nNo < vtTeachData.size(); nNo++)
	//{
	//	//SaveRobotCoor(CoutFile, vtTeachData[nNo].tMeasureCoordGunTool);
	//}
	//fclose(CoutFile);

	//// 焊枪工具 转 跟踪相机工具 移动到后续计算连续运动轨迹前转换
	//for (int nNo = 0; nNo < vtTeachData.size(); nNo++)
	//{
	//	if (false == m_pRobotDriver->MoveToolByWeldGun(vtTeachData[nNo].tMeasureCoordGunTool, m_pRobotDriver->m_tTools.tGunTool,
	//		vtTeachData[nNo].tMeasureCoordGunTool, m_pRobotDriver->m_tTools.tCameraTool, vtTeachData[nNo].tMeasureCoordCamTool))
	//	{
	//		XiMessageBox("焊缝%d 第%d个测量点 转换失败！", nSeamNo, nNo);
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
	T_TEACH_DATA tTeachData; // nMeasurePtnNo和tMeasureCoordCamTool外部合并测量点是赋值
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
	// 垂直法相偏移
	RobotCoordPosOffset(tCoord, dNorAngle + 90.0, dOffsetDis, 0.0);
	tCoord.dRZ -= 25.0;
	DecomposeExAxle(tCoord, dExAxlePos);
	tTeachData.tMeasureCoordGunTool = tCoord;
	if (0 != m_pRobotDriver->m_nRobotNo)
	{
		tTeachData.tMeasureCoordGunTool.dRX = 0.0;
	}
	vtTeachData.push_back(tTeachData);

	// 垂直法相偏移
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
			XiMessageBox("立缝测量理论轨迹转换失败");
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
		T_TEACH_DATA tTeachData; // nMeasurePtnNo和tMeasureCoordCamTool外部合并测量点是赋值
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

		//测量偏移量应保持一致，否则测量结果会计算错误
		//如果测量轨迹偏了，应先检查理想焊缝的位置是不是对的，而不是立即修改偏移量
		double dOffsetDis = 50.0;
		if (0 == m_pRobotDriver->m_nRobotNo)
		{
			RobotCoordPosOffset(tCoord, dNorAngle, dOffsetDis, 70.0 * m_nRobotInstallDir);
		}
		else
		{
			RobotCoordPosOffset(tCoord, dNorAngle, dOffsetDis, 50.0 * m_nRobotInstallDir);
		}
		// 垂直法相偏移
		RobotCoordPosOffset(tCoord, dNorAngle + 90.0, dOffsetDis, 0.0);
		tCoord.dRZ -= 50.0;
		DecomposeExAxle(tCoord, dExAxlePos);
		tTeachData.tMeasureCoordGunTool = tCoord;
		if (0 != m_pRobotDriver->m_nRobotNo)
		{
			tTeachData.tMeasureCoordGunTool.dRX = 0.0;
		}
		vtTeachData.push_back(tTeachData);

		// 垂直法相偏移
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
	else//添加爬坡立焊
	{
		T_ROBOT_COORS tCoord;
		T_TEACH_DATA tTeachData; // nMeasurePtnNo和tMeasureCoordCamTool外部合并测量点是赋值
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
		// 垂直法相偏移
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

		// 垂直法相偏移
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
	//		XiMessageBox("立缝测量理论轨迹转换失败");
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
//	T_TEACH_DATA tTeachData; // nMeasurePtnNo和tMeasureCoordCamTool外部合并测量点是赋值
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
//			XiMessageBox("立缝测量理论轨迹转换失败");
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
	if (dStartEndDis < 1.0 && !m_vvtWeldLineInfoGroup[nGroupNo][nSeamNo].tAtrribute.bWeldMode) // 闭合圆形焊缝
	{
		CHECK_BOOL_RETURN(CalcTeachDataCircle(nGroupNo, nSeamNo, dExAxlePos, vtSeamGroup, vtTeachData));
	}
	else // 自由焊缝
	{
		CHECK_BOOL_RETURN(CalcTeachDataFreeCurve(nGroupNo, nSeamNo, dExAxlePos, vtSeamGroup, vtTeachData));
	}
	for (int i = 0; i < vtTeachData.size(); i++)
	{
		if (!m_pRobotDriver->MoveToolByWeldGun(vtTeachData[i].tMeasureCoordGunTool, m_pRobotDriver->m_tTools.tGunTool,
			vtTeachData[i].tMeasureCoordGunTool, m_pRobotDriver->m_tTools.tCameraTool, vtTeachData[i].tMeasureCoordCamTool))
		{
			//XiMessageBox("圆弧焊缝%d 第%d个测量点 转换失败！", nSeamNo, i);
			//已修改
			XUI::MesBox::PopInfo("圆弧焊缝{0} 第{1}个测量点 转换失败！", nSeamNo, i);
			return false;
		}
	}
	return true;
}

bool GenericWeld::CalcTeachDataCircle(int nGroupNo, int nSeamNo, double dExAxlePos, vector<LineOrCircularArcWeldingLine> vtSeamGroup, vector<T_TEACH_DATA>& vtTeachData)
{
	/*
	 * int nWeldNo;							// ！测量点所属焊缝编号
	 * int nMeasurePtnNo;					// 测量点编号(同一个测量点可用于计算多个焊缝端点)
	 * int nMeasureType;					// ！测量类型： 先测后焊示教 测量修正 搜索端点 测棱投影
	 * E_WELD_STAM_TYPE eWeldSeamType;		// ！测量点所属焊缝的焊缝类型: 平焊 立焊 ……
	 * E_ATTRIBUTE eAttribute;				// ！测量点属性：用于测量起点 终点 或焊缝中间修正
	 * E_ENDPOINT_TYPE eEndpointType;		// ！测量点所属端点的端点类型: 自由端点 干涉端点
	 * T_ROBOT_COORS tMeasureCoordGunTool;	// ！测量位置直角坐标 焊枪工具
	 * T_ROBOT_COORS tMeasureCoordCamTool;	// 测量位置直角坐标 相机工具
	 */
	vtTeachData.clear();
	T_ROBOT_COORS tCoord;
	T_TEACH_DATA tTeachData; // nMeasurePtnNo和tMeasureCoordCamTool外部合并测量点是赋值
	tTeachData.nWeldNo = nSeamNo;
	tTeachData.nMeasurePtnNo = 0;
	tTeachData.nMeasureType = E_DOUBLE_LONG_LINE | E_TEACH_POINT;
	tTeachData.eWeldSeamType = E_FLAT_SEAM;
	tTeachData.eAttribute = E_BELONG_START;
	tTeachData.eEndpointType = E_FREE_POINT;

	// 正座倒挂 顺逆时针 开始测量方向角 测量角度间隔(等角度或等距离) 
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
	m_pTraceModel->m_tWorkPieceType = m_eWorkPieceType; // 初始化工件类型
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
	double dStartEndDis = TwoPointDis(
		tSeam.StartPoint.x, tSeam.StartPoint.y, tSeam.StartPoint.z,
		tSeam.EndPoint.x, tSeam.EndPoint.y, tSeam.EndPoint.z);
	bool IsCircle = dStartEndDis < 1.0 ? TRUE : FALSE; // 闭合圆弧
	// 起始结尾是否定长搜索
	bool bFixScanStart = m_vvtWeldLineInfoGroup.at(nGroupNo).at(nSeamNo).tAtrribute.bStartFixScan;
	bool bFixScanEnd = m_vvtWeldLineInfoGroup.at(nGroupNo).at(nSeamNo).tAtrribute.bEndFixScan;
	// 是否跟踪
	bool bTrack = m_vvtWeldLineInfoGroup.at(nGroupNo).at(nSeamNo).tAtrribute.bWeldMode;
	// 唯一需要搜索结尾条件，起点为中间点测量（缓焊）:2,/ 起点为间接干涉：3，结尾自由或干涉
	int nStartType = m_vvtWeldLineInfoGroup.at(nGroupNo).at(nSeamNo).tWeldLine.StartPointType;

	if (IsCircle) // 闭合圆弧
	{
		m_pScanInit->m_pTraceModel->m_tWorkPieceType = E_CLOSEDARC;
		CHECK_BOOL_RETURN(CalcInterfereTeachDataFlatFix(nSeamNo, bIsFlatWeld, true, vtSeamGroup, dNorAngleS, dExAxlePos, vtTempTeachData));
	}
	else if (bIsFlatWeld && bFreeStartPoint&& !bFixScanStart)			// 起点 平焊 自由端点
	{
		double dSearchLegth = dSearchDis;
		if (WeldingLineIsArc(m_vvtWeldLineInfoGroup.at(nGroupNo).at(nSeamNo).tWeldLine))
		{
			dSearchLegth = dSearchDis * 2;
		}
		CHECK_BOOL_RETURN(CalcFreeTeachDataFlat(nGroupNo, nSeamNo, bIsFlatWeld, true, tSeam, dNorAngleS, dSearchLegth, dExAxlePos, vtTempTeachData));
	}
	else if (bIsFlatWeld && !bFreeStartPoint&& bFixScanStart)	// 起点 平焊 干涉端点
	{
		CHECK_BOOL_RETURN(CalcInterfereTeachDataFlatFix(nSeamNo, bIsFlatWeld, true, vtSeamGroup, dNorAngleS, dExAxlePos, vtTempTeachData));
	}
	else
	{
		//已修改
		XUI::MesBox::PopInfo("终点测量位置计算失败！焊缝类型{0} 起点类型{1}", bIsFlatWeld, bFreeEndPoint);
		//XiMessageBox("起点测量位置计算失败！焊缝类型%d 起点类型%d", bIsFlatWeld, bFreeStartPoint);
		return false;
	}
	AppendTeachData(vtTeachData, vtTempTeachData);
	// 搜索端点次数
	m_vvtWeldLineInfoGroup.at(nGroupNo).at(nSeamNo).tAtrribute.nScanEndpointNum++;

	// 焊缝终点测量数据
	//if (/*2 != */nStartType < 2/*IsCircle|| bFixScanEnd|| !bTrack*/) // 闭合圆弧
	//{
	//	vtTempTeachData.clear();
	//}
	if (IsCircle) // 闭合圆弧
	{
		vtTempTeachData.clear();
	}
	else if (bIsFlatWeld && bFreeEndPoint && !bFixScanEnd)			// 终点 平焊 自由端点
	{
		CHECK_BOOL_RETURN(CalcFreeTeachDataFlat(nGroupNo, nSeamNo, bIsFlatWeld, false, tSeam, dNorAngleE, dSearchDis, dExAxlePos, vtTempTeachData));
	}
	else if (bIsFlatWeld && !bFreeEndPoint && bFixScanEnd)		// 终点 平焊 干涉端点
	{
		CHECK_BOOL_RETURN(CalcInterfereTeachDataFlatFix(nSeamNo, bIsFlatWeld, false, vtSeamGroup, dNorAngleE, dExAxlePos, vtTempTeachData));
	}
	else
	{
		//已修改
		XUI::MesBox::PopInfo("起点测量位置计算失败！焊缝类型{0} 终点类型{1}", bIsFlatWeld, bFreeStartPoint);
		//XiMessageBox("起点测量位置计算失败！焊缝类型%d 终点类型%d", bIsFlatWeld, bFreeEndPoint);
		return false;
	}
	AppendTeachData(vtTeachData, vtTempTeachData);
	if (vtTempTeachData.size() > 0)
	{
		// 搜索端点次数
		m_vvtWeldLineInfoGroup.at(nGroupNo).at(nSeamNo).tAtrribute.nScanEndpointNum++;
	}
	// 拆分外部轴和机器人坐标,待优化
	bool bFirstExPos = false;
	double dMachinePos = 0.0;
	for (int i = 0; i < vtTeachData.size(); i++)
	{
		double dMachineCarMovePos = 0.0;
		double dRobotOffsetPos = -500.0;//机器人固定位置	
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
		int nWeldNo;						// ！测量点所属焊缝编号
		int nMeasurePtnNo;					// 测量点编号(同一个测量点可用于计算多个焊缝端点)
		int nMeasureType;					// ！测量类型： 先测后焊示教 测量修正 搜索端点 测棱投影
		E_WELD_STAM_TYPE eWeldSeamType;		// ！测量点所属焊缝的焊缝类型: 平焊 立焊 ……
		E_ATTRIBUTE eAttribute;				// ！测量点属性：用于测量起点 终点 或焊缝中间修正
		E_ENDPOINT_TYPE eEndpointType;		// ！测量点所属端点的端点类型: 自由端点 干涉端点
		T_ROBOT_COORS tMeasureCoordGunTool;	// ！测量位置直角坐标 焊枪工具
		T_ROBOT_COORS tMeasureCoordCamTool;	// 测量位置直角坐标 相机工具
	*/
	// 测量点顺序：从焊缝内向焊缝外搜索
	vtTeachData.clear();
	if (!bIsFlatSeam)
	{
		XiMessageBox("非平焊无法计算平焊自由端点测量数据！");
		return false;
	}
	T_ROBOT_COORS tCoord;
	T_TEACH_DATA tTeachData; // nMeasurePtnNo和tMeasureCoordCamTool外部合并测量点是赋值
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
	//bool bMeasureNeedOffset = true; // !!!! 分段焊接 与依次包角类型设置冲突
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
	// 测量点顺序：当前边两个测量位置 相邻边测量两个位置(是否有相邻焊缝测量位置不同)
	vtTeachData.clear();
	if (!bIsFlatSeam)
	{
		XiMessageBox("非平焊无法计算平焊干涉端点测量数据！");
		return false;
	}
	LineOrCircularArcWeldingLine tSeam = vtSeam[nSeamNo];
	double dShortSeamThreshold = m_dShortSeamThreshold;
	T_ROBOT_COORS tCoord; 
	vector<T_ROBOT_COORS> vtMeasureCoord;
	T_TEACH_DATA tTeachData; // nMeasurePtnNo和tMeasureCoordCamTool外部合并测量点是赋值
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

	// 当前焊缝上的测量点	
	LineOrCircularArcWeldingLine* ptAdjoinSeamS = NULL; //&tAdjoinSeamS; // 平焊起点相邻的平焊缝 或 起点与立焊起点相邻的平焊缝
	LineOrCircularArcWeldingLine* ptAdjoinSeamE = NULL; //&tAdjoinSeamE; // 平焊终点相邻的平焊缝 或 终点与立焊起点相邻的平焊缝
	GetAdjoinSeam(nSeamNo, vtSeam, &ptAdjoinSeamS, &ptAdjoinSeamE);
	dCurSeamOffsetDis = dCurSeamLen < dShortSeamThreshold ? dCurSeamLen / 3.0 : dCurSeamOffsetDis;
	CHECK_BOOL_RETURN(CalcSeamMeasureCoord(tSeam, dCurSeamOffsetDis, dRobotChangeDir, vtMeasureCoord, ptAdjoinSeamS, ptAdjoinSeamE));
	for (int nIdx = 0; nIdx < vtMeasureCoord.size(); nIdx++)
	{
		DecomposeExAxle(vtMeasureCoord[nIdx], dExAxlePos);
		tTeachData.tMeasureCoordGunTool = vtMeasureCoord[nIdx];
		vtTeachData.push_back(tTeachData);
	}
	// 相邻焊缝上的测量点
	double dAdjoinSeamLen = 0.0;
	LineOrCircularArcWeldingLine* ptAdjoinSeam = bIsStartPtn ? ptAdjoinSeamS : ptAdjoinSeamE;
	if (NULL != ptAdjoinSeam) // 干涉端点相邻边有焊缝
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
	else // 干涉端点相邻边无焊缝
	{
		CvPoint3D64f tPtn = bIsStartPtn ? tSeam.StartPoint : tSeam.EndPoint;
		double dHandEyeOffsetDis = !bIsStartPtn ? m_dHandEyeDis : 0.0; // 平焊终点干涉且无相邻焊缝时 假设相邻了一个焊缝的起点 需要添加手眼偏移
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
		CalcCorvrAngle(!bIsStartPtn, false, false, tCoord); // 去掉防止遮挡
		tTeachData.tMeasureCoordGunTool = tCoord;
		vtTeachData.push_back(tTeachData);
	}
	return true;
}

bool GenericWeld::CalcInterfereTeachDataFlatFix(int nSeamNo, bool bIsFlatSeam, bool bIsStartPtn, vector<LineOrCircularArcWeldingLine> vtSeam, double dNorAngle, double dExAxlePos, vector<T_TEACH_DATA>& vtTeachData)
{
	// 测量点顺序：当前边两个测量位置 相邻边测量两个位置(是否有相邻焊缝测量位置不同)
	vtTeachData.clear();
	if (!bIsFlatSeam)
	{
		XiMessageBox("非平焊无法计算平焊干涉端点测量数据！");
		return false;
	}
	LineOrCircularArcWeldingLine tSeam = vtSeam[nSeamNo];
	double dShortSeamThreshold = m_dShortSeamThreshold;
	T_ROBOT_COORS tCoord;
	vector<T_ROBOT_COORS> vtMeasureCoord;
	T_TEACH_DATA tTeachData; // nMeasurePtnNo和tMeasureCoordCamTool外部合并测量点是赋值
	tTeachData.nWeldNo = nSeamNo;
	tTeachData.nMeasurePtnNo = 0;
	tTeachData.nMeasureType = E_DOUBLE_LONG_LINE | E_SEARCH_POINT | E_SEARCH_POINT_FIX;
	tTeachData.eWeldSeamType = bIsFlatSeam ? E_FLAT_SEAM : E_STAND_SEAM;
	tTeachData.eAttribute = bIsStartPtn ? E_BELONG_START : E_BELONG_END;
	tTeachData.eEndpointType = E_INTERFERE_POINT;
	double dCurSeamOffsetDis = m_dEndpointSearchDis;// m_dMeasureDisThreshold;//搜索距离
	double dAdjoinOffsetDis = m_dEndpointSearchDis;//m_dMeasureDisThreshold;//搜索距离
	double dCurSeamLen = TwoPointDis(tSeam.StartPoint.x, tSeam.StartPoint.y, tSeam.StartPoint.z, tSeam.EndPoint.x, tSeam.EndPoint.y, tSeam.EndPoint.z);
	double dRobotChangeDir = 1 == m_nRobotInstallDir ? 1.0 : -1.0;
	double dSEChangeDir = true == bIsStartPtn ? 1.0 : -1.0;

	// 当前焊缝上的测量点	
	LineOrCircularArcWeldingLine* ptAdjoinSeamS = NULL; //&tAdjoinSeamS; // 平焊起点相邻的平焊缝 或 起点与立焊起点相邻的平焊缝
	LineOrCircularArcWeldingLine* ptAdjoinSeamE = NULL; //&tAdjoinSeamE; // 平焊终点相邻的平焊缝 或 终点与立焊起点相邻的平焊缝
	GetAdjoinSeam(nSeamNo, vtSeam, &ptAdjoinSeamS, &ptAdjoinSeamE);
	//dCurSeamOffsetDis = dCurSeamLen < dShortSeamThreshold ? dCurSeamLen / 3.0 : dCurSeamOffsetDis;
	// 圆弧定长搜索	
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
	// 测量点顺序：顺时针相邻焊缝测量 逆时针相邻焊缝测量
	vtTeachData.clear();
	if (bIsFlatSeam)
	{
		XiMessageBox("非立焊无法计算立焊缝测量数据！");
		return false;
	}
	LineOrCircularArcWeldingLine tSeam = vtSeam[nSeamNo];
	double dShortSeamThreshold = m_dShortSeamThreshold;
	XiAlgorithm alg;
	T_ROBOT_COORS tCoord;
	vector<T_ROBOT_COORS> vtMeasureCoord;
	E_ENDPOINT_TYPE eEndpointTypeS = 0 <  tSeam.StartPointType ? E_INTERFERE_POINT : E_FREE_POINT;
	E_ENDPOINT_TYPE eEndpointTypeE = 0 <  tSeam.EndPointType ? E_INTERFERE_POINT : E_FREE_POINT;
	vector<T_TEACH_DATA> vtTeachDataStandUpper; // 立焊高度测量点
	vtTeachDataStandUpper.clear();
	T_TEACH_DATA tTeachData; // nMeasurePtnNo和tMeasureCoordCamTool外部合并测量点是赋值
	tTeachData.nWeldNo = nSeamNo;
	tTeachData.nMeasurePtnNo = 0;
	tTeachData.nMeasureType = E_INTERFERE_POINT == eEndpointTypeS ? E_DOUBLE_LONG_LINE | E_TEACH_POINT : E_L_LINE_POINT | E_TEACH_POINT; // 立焊终点板子上端面测量类型不一致
	tTeachData.eWeldSeamType = bIsFlatSeam ? E_FLAT_SEAM : E_STAND_SEAM;
	tTeachData.eAttribute = bIsStartPtn ? E_BELONG_START : E_BELONG_END;
	tTeachData.eEndpointType = true == bIsStartPtn ? eEndpointTypeS : eEndpointTypeE;
	double dAdjoinOffsetDis = m_dMeasureDisThreshold;
	double dCurSeamLen = TwoPointDis(tSeam.StartPoint.x, tSeam.StartPoint.y, tSeam.StartPoint.z, tSeam.EndPoint.x, tSeam.EndPoint.y, tSeam.EndPoint.z);
	double dRobotChangeDir = 1 == m_nRobotInstallDir ? 1.0 : -1.0;
	double dNorAngle = alg.CalcArcAngle(tSeam.StartNormalVector.x, tSeam.StartNormalVector.y);

	// 立焊终点测量必须区分确定：立焊 顺时针 逆时针 和 两个方向立焊高度 tSeam.EndPoint.z tSeam.ZSide 的关系
	// 区分立焊左右两侧高度
	double dAdjoinSeam1Height = true == tSeam.isLeft ? tSeam.ZSide : tSeam.EndPoint.z; // ZSide 在左?
	double dAdjoinSeam2Height = true == tSeam.isLeft ? tSeam.EndPoint.z : tSeam.ZSide;
	LineOrCircularArcWeldingLine* ptAdjoinSeamS = NULL;
	LineOrCircularArcWeldingLine* ptAdjoinSeamE = NULL;
	GetAdjoinSeam(tSeam, vtSeam, &ptAdjoinSeamS, &ptAdjoinSeamE);
	LineOrCircularArcWeldingLine* ptAdjoinSeam1 = ptAdjoinSeamE; // 焊枪角度立焊左侧平焊
	LineOrCircularArcWeldingLine* ptAdjoinSeam2 = ptAdjoinSeamS; // 焊枪角度立焊右侧平焊

	if (NULL != ptAdjoinSeam1)	// 立焊顺时针方向存在相邻焊缝
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
		if (false == bIsStartPtn) // 立焊终点高度测量点
		{
			//tTeachData.tMeasureCoordGunTool.dRX = m_dPlatWeldRx;
			//tTeachData.tMeasureCoordGunTool.dRY = m_dPlatWeldRy;
			//tTeachData.tMeasureCoordGunTool = RobotCoordPosOffset(tTeachData.tMeasureCoordGunTool, ptAdjoinSeam1->StartPoint, ptAdjoinSeam1->EndPoint, 18.0);
			tTeachData.tMeasureCoordGunTool.dZ = dAdjoinSeam1Height;
			vtTeachDataStandUpper.push_back(tTeachData);
		}
	}
	else						// 立焊顺时针方向不存在相邻焊缝
	{
		CvPoint3D64f tPtn = tSeam.StartPoint;
		double dAdjoinSeamNorAngle = dNorAngle + (45.0 * dRobotChangeDir);
		double dOffsetDir = dNorAngle - (45.0 * dRobotChangeDir);
		tCoord = GenerateRobotCoord(tPtn, m_dPlatWeldRx, m_dPlatWeldRy, DirAngleToRz(dAdjoinSeamNorAngle));
		RobotCoordPosOffset(tCoord, dOffsetDir, dAdjoinOffsetDis); // 终点与立焊起点相接的平焊上测量位置无需手眼偏移
		DecomposeExAxle(tCoord, dExAxlePos);
		CalcCorvrAngle(false, false, false, tCoord); // 与ptAdjoinSeamE相关
		tTeachData.tMeasureCoordGunTool = tCoord;
		vtTeachData.push_back(tTeachData);

		if (false == bIsStartPtn) // 立焊终点高度测量点
		{
			//tTeachData.tMeasureCoordGunTool.dRX = m_dPlatWeldRx;
			//tTeachData.tMeasureCoordGunTool.dRY = m_dPlatWeldRy;
			tTeachData.tMeasureCoordGunTool.dZ = dAdjoinSeam1Height;
			vtTeachDataStandUpper.push_back(tTeachData);
		}

		tCoord = GenerateRobotCoord(tPtn, m_dPlatWeldRx, m_dPlatWeldRy, DirAngleToRz(dAdjoinSeamNorAngle));
		RobotCoordPosOffset(tCoord, dOffsetDir, dAdjoinOffsetDis * 2.0/* - 10.0*/);  // 鸿路临时-10
		DecomposeExAxle(tCoord, dExAxlePos);
		CalcCorvrAngle(false, false, false, tCoord); // 与ptAdjoinSeamE相关 // 去掉防止遮挡
		tTeachData.tMeasureCoordGunTool = tCoord;
		vtTeachData.push_back(tTeachData);
	}

	dAdjoinOffsetDis = m_dMeasureDisThreshold;
	if (NULL != ptAdjoinSeam2)	// 立焊逆时针方向存在相邻焊缝
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
		if (false == bIsStartPtn) // 立焊终点高度测量点
		{
			tTeachData.tMeasureCoordGunTool = vtMeasureCoord[0];
			//tTeachData.tMeasureCoordGunTool.dRX = m_dPlatWeldRx;
			//tTeachData.tMeasureCoordGunTool.dRY = m_dPlatWeldRy;
			//tTeachData.tMeasureCoordGunTool = RobotCoordPosOffset(tTeachData.tMeasureCoordGunTool, ptAdjoinSeam2->EndPoint, ptAdjoinSeam2->StartPoint, 18.0);
			tTeachData.tMeasureCoordGunTool.dZ = dAdjoinSeam2Height;
			vtTeachDataStandUpper.push_back(tTeachData);
		}
	}
	else						// 立焊逆时针方向不存在相邻焊缝
	{
		CvPoint3D64f tPtn = tSeam.StartPoint;
		double dAdjoinSeamNorAngle = dNorAngle - (45.0 * dRobotChangeDir);
		double dOffsetDir = dNorAngle + (45.0 * dRobotChangeDir);
		tCoord = GenerateRobotCoord(tPtn, m_dPlatWeldRx, m_dPlatWeldRy, DirAngleToRz(dAdjoinSeamNorAngle));
		RobotCoordPosOffset(tCoord, dOffsetDir, dAdjoinOffsetDis + m_dHandEyeDis); // 起点与立焊起点相连的平焊上测量点需要添加手眼偏移距离
		DecomposeExAxle(tCoord, dExAxlePos);
		CalcCorvrAngle(true, false, false, tCoord); // 与ptAdjoinSeamS相关
		tTeachData.tMeasureCoordGunTool = tCoord;
		vtTeachData.push_back(tTeachData);

		if (false == bIsStartPtn) // 立焊终点高度测量点
		{
			//tTeachData.tMeasureCoordGunTool.dRX = m_dPlatWeldRx;
			//tTeachData.tMeasureCoordGunTool.dRY = m_dPlatWeldRy;
			tTeachData.tMeasureCoordGunTool.dZ = dAdjoinSeam2Height;
			vtTeachDataStandUpper.push_back(tTeachData);
		}

		tCoord = GenerateRobotCoord(tPtn, m_dPlatWeldRx, m_dPlatWeldRy, DirAngleToRz(dAdjoinSeamNorAngle));
		RobotCoordPosOffset(tCoord, dOffsetDir, dAdjoinOffsetDis * 2.0/* - 10.0*/ + m_dHandEyeDis); // 鸿路临时-10 // 起点与立焊起点相连的平焊上测量点需要添加手眼偏移距离
		DecomposeExAxle(tCoord, dExAxlePos);
		CalcCorvrAngle(true, false, false, tCoord); // 与ptAdjoinSeamS相关  // 去掉防止遮挡
		tTeachData.tMeasureCoordGunTool = tCoord;
		vtTeachData.push_back(tTeachData);
	}
	for (int i = 0; i < vtTeachDataStandUpper.size(); i++)
	{
		vtTeachDataStandUpper[i].nMeasureType = E_LS_RL_FLIP | E_TEACH_POINT;
	}
	AppendTeachData(vtTeachData, vtTeachDataStandUpper); // 追加立焊终点高度测量数据

	return true;
}

bool GenericWeld::CalcFlatSeamMeasurementEnhancement(int nSeamNo, bool bIsFlatSeam, bool bIsStartPtn, LineOrCircularArcWeldingLine tSeam, double dNorAngle, double dExAxlePos, LineOrCircularArcWeldingLine* ptAdjoinSeam, vector<T_TEACH_DATA>& vtTeachData)
{
	//补充测量点距离
	double dPointSpacing = m_dPointSpacing;
	//平焊缝上测量点间的距离
	double dShortSeamThreshold = m_dShortSeamThreshold;
	//临时保存使用的焊枪坐标
	T_ROBOT_COORS tCoord;
	vector<T_ROBOT_COORS> vtTempCoord;
	vector<T_ROBOT_COORS> vtMeasureCoord;
	T_TEACH_DATA tTeachData; // nMeasurePtnNo和tMeasureCoordCamTool外部合并测量点是赋值
	//设置补充测量点的参数
	tTeachData.nWeldNo = nSeamNo;
	tTeachData.nMeasurePtnNo = 0;
	tTeachData.nMeasureType = E_DOUBLE_LONG_LINE | E_ADJUST_POINT;
	tTeachData.eWeldSeamType = E_FLAT_SEAM;
	tTeachData.eAttribute =  E_BELONG_MIDDLE;
	tTeachData.eEndpointType = E_INTERFERE_POINT;
	double dCurSeamOffsetDis = m_dMeasureDisThreshold;
	double dAdjoinOffsetDis = m_dMeasureDisThreshold;
	//起点和终点之间的距离
	double dCurSeamLen = TwoPointDis(tSeam.StartPoint.x, tSeam.StartPoint.y, tSeam.StartPoint.z, tSeam.EndPoint.x, tSeam.EndPoint.y, tSeam.EndPoint.z);
	//判断机器人安装方向
	double dRobotChangeDir = 1 == m_nRobotInstallDir ? 1.0 : -1.0;
	double dSEChangeDir = true == bIsStartPtn ? 1.0 : -1.0;
	//算法类
	XiAlgorithm alg;
	double NorAngle = alg.CalcArcAngle(tSeam.StartNormalVector.x, tSeam.StartNormalVector.y);
	//首先判断在平焊缝上生成的两个测量点之间的距离是否大于补充测量点阈值两倍的距离
	//1. 生成平焊缝上两个测量点
		// 当前焊缝上的测量点	
	dCurSeamOffsetDis = dCurSeamLen < dShortSeamThreshold ? dCurSeamLen / 3.0 : dCurSeamOffsetDis;
	CHECK_BOOL_RETURN(CalcSeamMeasureCoord(tSeam, dCurSeamOffsetDis, dRobotChangeDir, vtTempCoord, NULL, NULL));
	//2. 判断两个平焊缝之间的距离是否大于测量点上的阈值 如果大于2倍阈值， 则可生成补充点
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
	// 测量点顺序：顺时针相邻焊缝测量 逆时针相邻焊缝测量
	vtTeachData.clear();
	if (bIsFlatSeam || !bIsStartPtn)
	{
		XiMessageBox("非立焊自由端点 计算测量数据失败！");
		return false;
	}
	double dShortSeamThreshold = m_dShortSeamThreshold;
	XiAlgorithm alg;
	T_ROBOT_COORS tCoord;
	vector<T_ROBOT_COORS> vtMeasureCoord;
	E_ENDPOINT_TYPE eEndpointTypeS = 0 <  tSeam.StartPointType ? E_INTERFERE_POINT : E_FREE_POINT;
	E_ENDPOINT_TYPE eEndpointTypeE = 0 <  tSeam.EndPointType ? E_INTERFERE_POINT : E_FREE_POINT;
	vector<T_TEACH_DATA> vtTeachDataStandUpper; // 立焊高度测量点
	vtTeachDataStandUpper.clear();
	T_TEACH_DATA tTeachData; 
	tTeachData.nWeldNo = nSeamNo;
	tTeachData.nMeasurePtnNo = 0;
	tTeachData.nMeasureType = E_L_LINE_POINT | E_TEACH_POINT; // 立焊终点板子上端面测量类型不一致
	tTeachData.eWeldSeamType = bIsFlatSeam ? E_FLAT_SEAM : E_STAND_SEAM;
	tTeachData.eAttribute = bIsStartPtn ? E_BELONG_START : E_BELONG_END;
	tTeachData.eEndpointType = true == bIsStartPtn ? eEndpointTypeS : eEndpointTypeE;
	double dAdjoinOffsetDis = m_dMeasureDisThreshold;
	double dRobotChangeDir = 1 == m_nRobotInstallDir ? 1.0 : -1.0;
	double dNorAngle = alg.CalcArcAngle(tSeam.StartNormalVector.x, tSeam.StartNormalVector.y);

	// 平焊终点与立焊起点相接 方向的板上 的测量点
	CvPoint3D64f tPtn = tSeam.StartPoint;
	double dAdjoinSeamNorAngle = dNorAngle + (45.0 * dRobotChangeDir);
	double dOffsetDir = dNorAngle - (45.0 * dRobotChangeDir);
	tCoord = GenerateRobotCoord(tPtn, m_dPlatWeldRx, m_dPlatWeldRy, DirAngleToRz(dAdjoinSeamNorAngle));
	RobotCoordPosOffset(tCoord, dOffsetDir, dAdjoinOffsetDis); // 终点与立焊起点相接的平焊上测量位置无需手眼偏移
	DecomposeExAxle(tCoord, dExAxlePos);
	CalcCorvrAngle(false, false, false, tCoord);
	tTeachData.tMeasureCoordGunTool = tCoord;
	vtTeachData.push_back(tTeachData);

	tCoord = GenerateRobotCoord(tPtn, m_dPlatWeldRx, m_dPlatWeldRy, DirAngleToRz(dAdjoinSeamNorAngle));
	RobotCoordPosOffset(tCoord, dOffsetDir, dAdjoinOffsetDis * 2.0);
	DecomposeExAxle(tCoord, dExAxlePos);
	//tCoord.dRX += CalcCorvrAngle(false, false, false); // 去掉防止遮挡
	tTeachData.tMeasureCoordGunTool = tCoord;
	vtTeachData.push_back(tTeachData);

	dAdjoinOffsetDis = m_dMeasureDisThreshold;
	// 平焊起点与立焊起点相接 方向的板上 的测量点			
	{
		CvPoint3D64f tPtn = tSeam.StartPoint;
		double dAdjoinSeamNorAngle = dNorAngle - (45.0 * dRobotChangeDir);
		double dOffsetDir = dNorAngle + (45.0 * dRobotChangeDir);
		tCoord = GenerateRobotCoord(tPtn, m_dPlatWeldRx, m_dPlatWeldRy, DirAngleToRz(dAdjoinSeamNorAngle));
		RobotCoordPosOffset(tCoord, dOffsetDir, dAdjoinOffsetDis + m_dHandEyeDis); // 起点与立焊起点相连的平焊上测量点需要添加手眼偏移距离
		DecomposeExAxle(tCoord, dExAxlePos);
		CalcCorvrAngle(true, false, false, tCoord);
		tTeachData.tMeasureCoordGunTool = tCoord;
		vtTeachData.push_back(tTeachData);

		tCoord = GenerateRobotCoord(tPtn, m_dPlatWeldRx, m_dPlatWeldRy, DirAngleToRz(dAdjoinSeamNorAngle));
		RobotCoordPosOffset(tCoord, dOffsetDir, dAdjoinOffsetDis * 2.0 + m_dHandEyeDis); // 起点与立焊起点相连的平焊上测量点需要添加手眼偏移距离
		DecomposeExAxle(tCoord, dExAxlePos);
		//tCoord.dRX += CalcCorvrAngle(true, false, false);  // 去掉防止遮挡
		tTeachData.tMeasureCoordGunTool = tCoord;
		vtTeachData.push_back(tTeachData);
	}

	return true;
}

void GenericWeld::CalcCorvrAngle(bool bBelongStart, bool bBelongPtnFree, bool bOtherPtnFree, T_ROBOT_COORS& tCoord, double dSeamLen/* = 0*/)
{
	// 测量点属于起点附近还是终点附近 ？ 
	// 测量点所属点类型(true:自由  false:干涉)
	// 测量点所属点以外的点类型(true:自由  false:干涉)
	// 焊缝长度 默认0表示 测量点不属于某个焊道 是根据干涉端点推算出的焊缝
	
	// 双自由端 不变姿态
	// 所属点自由 长度大 不变姿态
	// 所属点自由 且 另一侧端点干涉 且 长度小 按 !bBelongStart 计算变姿态
	// 所属点干涉 按bBelongStart 计算变姿态

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
	//double dChangeAngle = bBelongStart ? m_dGunLaserAngle : m_dGunCameraAngle; // 起点附近遮挡激光 终点附近遮挡相机
	double dChangeAngle = bBelongStart ? m_dGunCameraAngle : m_dGunLaserAngle; // 起点附近遮挡激光 终点附近遮挡相机
	double dChangeDir = bBelongStart ? m_dRotateToCamRxDir : -m_dRotateToCamRxDir; // 挡激光转向相机 挡相机转向激光
	tCoord.dRX += (dChangeAngle * dChangeDir);
	double dRZAdjust = 0.0;
	//if (m_pRobotDriver->m_nRobotNo > 1)
	{
		//dRZAdjust = 10.0;
	}
	//tCoord.dRZ += ((dChangeAngle+ dRZAdjust) * (dChangeDir));// 江南工件纵横交错增大Rz转角
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
	std::vector<std::vector<T_TEACH_DATA>> vvtTeachData(tSeamGroup.size(), vtTeachData); // 每个焊缝区分：索引1焊缝号 索引2测量点号

	// 检查示教结果数量及编号是否满足m_vtTeachData中所需 并查找当前焊缝组每条焊缝的测量数据
	for (int nDataNo = 0; nDataNo < m_vtTeachData.size(); nDataNo++)
	{
		mnnMeasureNo[m_vtTeachData[nDataNo].nMeasurePtnNo] = 0;
		vvtTeachData[m_vtTeachData[nDataNo].nWeldNo].push_back(m_vtTeachData[nDataNo]);
	}
	if (m_vtTeachResult.size() != mnnMeasureNo.size())
	{
		//已修改
		XUI::MesBox::PopInfo("测量数据错误! 焊缝调整失败! {0} {1}", mnnMeasureNo.size(), m_vtTeachResult.size());
		//XiMessageBox("测量数据错误! 焊缝调整失败! %d %d", mnnMeasureNo.size(), m_vtTeachResult.size());
		return false;
	}

	// 修正焊缝组nGroup中每个焊缝
	for (int nWeldNo = 0; nWeldNo < tSeamGroup.size(); nWeldNo++)
	{
		// 获取焊缝的所有测量数据
		std::vector<T_TEACH_DATA>& tData = vvtTeachData[nWeldNo];
		if (10 == m_vvtWeldLineInfoGroup[nGroupNo][nWeldNo].tAtrribute.nStartWrapType)
		{
			CHECK_BOOL_RETURN(AdjustScanWeldSeam(nGroupNo, nWeldNo, tData));
			continue;
		}
		else if (WeldingLineIsArc(tSeamGroup[nWeldNo])) // 圆弧 计算 起点终点
		{
			CHECK_BOOL_RETURN(AdjustWeldSeamArc(nGroupNo, nWeldNo, tData));
			continue;
		}
		// 根据 平焊/立焊  起点/终点  自由/干涉 等属性计算准确端点数据
		CHECK_BOOL_RETURN(CalcAccurateEndpoint(nGroupNo, nWeldNo, tData, tAlgPtnS, tAlgPtnE, dNorAngle, dZSide));

		// 修正一条焊缝
		m_vvtWeldSeamGroupAdjust[nGroupNo][nWeldNo] = m_vvtWeldSeamGroup[nGroupNo][nWeldNo]; // 重复运行同一组焊缝恢复调整前识别数据
		CHECK_BOOL_RETURN(AdjsutWeldSeam(m_vvtWeldSeamGroupAdjust[nGroupNo][nWeldNo], tAlgPtnS, tAlgPtnE, dNorAngle, dZSide));
	}
	return true;
}

bool GenericWeld::CalcAccurateEndpoint(int nGroupNo, int nWeldNo, std::vector<T_TEACH_DATA>& tData,
	T_ALGORITHM_POINT& tS, T_ALGORITHM_POINT& tE, double& dNorAngle, double& dZSide)
{
	// 起点 平焊 自由端点	两个测量位置：取端点
	// 终点 平焊 自由端点	两个测量位置：取端点
	// 起点 平焊 干涉端点	四个测量位置：三面相交
	// 起点 立焊 干涉端点	四个测量位置：三面相交
	// 终点 平焊 干涉端点	四个测量位置：三面相交
	// 终点 立焊 自由端点	六个测量位置：两个里面相交和测高

	// 获取每个端点的所有测量数据 并判断有效性
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
		//已修改
		XUI::MesBox::PopInfo("焊缝组{0} 焊缝{1} 测量数据{2} {3}异常！", nGroupNo, nWeldNo, tTeachDataS.size(), tTeachDataE.size());
		//XiMessageBox("焊缝组%d 焊缝%d 测量数据%d %d异常！", nGroupNo, nWeldNo, tTeachDataS.size(), tTeachDataE.size());
		return false;
	}
	bool bIsFlatWeld = E_FLAT_SEAM == tTeachDataS[0].eWeldSeamType;
	bool bFreePointS = E_FREE_POINT == tTeachDataS[0].eEndpointType;
	bool bFreePointE = E_FREE_POINT == tTeachDataE[0].eEndpointType;

	// 黄埔干涉段都搜索，计算时更改端点为搜索获取端点
	//if (bIsFlatWeld)
	//{
	//	bFreePointS = true;
	//	bFreePointE = true;
	//}
	//-------------------------

	CHECK_BOOL_RETURN(CheckTeachDataNumValid(tTeachDataS.size(), bIsFlatWeld, true, bFreePointS));
	CHECK_BOOL_RETURN(CheckTeachDataNumValid(tTeachDataE.size(), bIsFlatWeld, false, bFreePointE));

	// 计算精确起点
	if (bIsFlatWeld && bFreePointS)			// 起点 平焊 自由端点 1
	{
		CHECK_BOOL_RETURN(CalcFreePoint(tTeachDataS, tS));
	}
	else if (bIsFlatWeld && !bFreePointS)	// 起点 平焊 干涉端点 2
	{
		CHECK_BOOL_RETURN(CalcInterferePoint(tTeachDataS, tS));
	}
	else if (!bIsFlatWeld && !bFreePointS)	// 起点 立焊 干涉端点 2
	{
		CHECK_BOOL_RETURN(CalcInterferePointStand(tTeachDataS, tS));
	}
	else if (!bIsFlatWeld && bFreePointS)	// 起点 立焊 自由端点 4
	{
		CHECK_BOOL_RETURN(CalcFreeStartPointStand(tTeachDataS, tS));
	}
	else
	{
		//已修改
		XUI::MesBox::PopInfo("焊缝组{0} 焊缝{1} 计算起点失败！焊缝类型{2} 起点类型{3}", nGroupNo, nWeldNo, bIsFlatWeld, bFreePointS);
		//XiMessageBox("焊缝组%d 焊缝%d 计算起点失败！焊缝类型%d 起点类型%d", nGroupNo, nWeldNo, bIsFlatWeld, bFreePointS);
		return false;
	}

	// 计算精确终点
	if (bIsFlatWeld && bFreePointE)			// 终点 平焊 自由端点 1
	{
		CHECK_BOOL_RETURN(CalcFreePoint(tTeachDataE, tE));
	}
	else if (bIsFlatWeld && !bFreePointE)		// 终点 平焊 干涉端点 2
	{
		CHECK_BOOL_RETURN(CalcInterferePoint(tTeachDataE, tE));
	}
	else if (!bIsFlatWeld && bFreePointE)		// 终点 立焊 自由端点 3
	{
		CHECK_BOOL_RETURN(CalcFreeEndPointStand(tTeachDataE, tE));
	}
	else
	{
		//已修改
		XUI::MesBox::PopInfo("焊缝组{0} 焊缝{1} 计算终点失败！焊缝类型{2} 起点类型{3}", nGroupNo, nWeldNo, bIsFlatWeld, bFreePointE);
		//XiMessageBox("焊缝组%d 焊缝%d 计算终点失败！焊缝类型%d 起点类型%d", nGroupNo, nWeldNo, bIsFlatWeld, bFreePointE);
		return false;
	}

	// 计算焊缝法相
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
	// 起点 平焊 自由端点	两个测量位置：取端点
	// 终点 平焊 自由端点	两个测量位置：取端点
	// 起点 平焊 干涉端点	四个测量位置：三面相交
	// 起点 立焊 干涉端点	四个测量位置：三面相交
	// 终点 平焊 干涉端点	四个测量位置：三面相交
	// 终点 立焊 自由端点	六个测量位置：两个里面相交和测高
	int nFlatSeamFreeNum = 2;
	int nThreePlaneEndPointNum = 4;
	int nStandSeamFreeEndNum = 6;
	int nStandSeamFreeStartNum = 4;
	bool bIsValid = false;
	if (bIsFlatSeam && bIsFreePoint) // 平焊 自由端点
	{
		bIsValid = nFlatSeamFreeNum == nNumber ? true : false;
	}
	else if (!bIsFreePoint)			// 干涉端点
	{
		bIsValid = nThreePlaneEndPointNum == nNumber ? true : false;
	}
	else if (!bIsStartPoint && !bIsFlatSeam && bIsFreePoint)	// 终点 立焊 自由端点
	{
		bIsValid = nStandSeamFreeEndNum == nNumber ? true : false;
	}
	else if (bIsStartPoint && !bIsFlatSeam && bIsFreePoint)
	{
		bIsValid = nStandSeamFreeStartNum == nNumber ? true : false;
	}
	else
	{
		XUI::MesBox::PopOkCancel("参数错误，检查测量结果数量失败! {0} {1} {2} {3}", nNumber, bIsFlatSeam, bIsStartPoint, bIsFreePoint);
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

	// 获取tTeachData对应的所有测量结果数据
	for (int i = 0; i < tTeachData.size(); i++)
	{
		vtTeachResult.push_back(m_vtTeachResult[tTeachData[i].nMeasurePtnNo]);
	}

	// 获取三个面上的点
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

	// 拟合三个平面
	tPlane1 = alg.CalcPlaneParamRansac(vtFirstPoints, 0.8);
	tPlane2 = alg.CalcPlaneParamRansac(vtSecondPoints, 0.8);
	tPlane3 = alg.CalcPlaneParamRansac(vtThirdPoints, 0.8);

	// 两面相交得到交线
	tAlgLine = PlanePlaneInterLine(tPlane1, tPlane2);

	// 交线和第三面相交得角点
	if (false == alg.LinePlateInterSection(tAlgLine, tPlane3, tAlgPoint))
	{
		XiMessageBox("干涉端点: 线面相交计算失败!");
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

	// 获取tTeachData对应的所有测量结果数据
	for (int i = 0; i < tTeachData.size(); i++)
	{
		vtTeachResult.push_back(m_vtTeachResult[tTeachData[i].nMeasurePtnNo]);
	}

	// 获取三个面上的点
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

	// 拟合三个平面
	tPlane1 = alg.CalcPlaneParamRansac(vtFirstPoints, 0.8);
	tPlane2 = alg.CalcPlaneParamRansac(vtSecondPoints, 0.8);
	tPlane3 = alg.CalcPlaneParamRansac(vtThirdPoints, 0.8);

	// 两面相交得到交线
	tAlgLine = PlanePlaneInterLine(tPlane1, tPlane2);

	// 交线和第三面相交得角点
	if (false == alg.LinePlateInterSection(tAlgLine, tPlane3, tAlgPoint))
	{
		XiMessageBox("干涉端点: 线面相交计算失败!");
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

	// 获取tTeachData对应的所有测量结果数据
	for (int i = 0; i < tTeachData.size(); i++)
	{
		vtTeachResult.push_back(m_vtTeachResult[tTeachData[i].nMeasurePtnNo]);
	}

	// 获取两个立面上的点 (两个线面相交 取最高点 )
	XiPointAddToAlgPtn(vtTeachResult[0].vtLeftPtns3D, vtFirstPoints);
	XiPointAddToAlgPtn(vtTeachResult[1].vtLeftPtns3D, vtFirstPoints);
	XiPointAddToAlgPtn(vtTeachResult[2].vtLeftPtns3D, vtSecondPoints);
	XiPointAddToAlgPtn(vtTeachResult[3].vtLeftPtns3D, vtSecondPoints);

	// 拟合两个立板平面
	tPlane1 = alg.CalcPlaneParamRansac(vtFirstPoints, 0.8);
	tPlane2 = alg.CalcPlaneParamRansac(vtSecondPoints, 0.8);

	// 两面相交得到交线
	tAlgLine = PlanePlaneInterLine(tPlane1, tPlane2);

	// 获取立焊起点高度 取高点
	dMaxHeight = vtTeachResult[0].tKeyPtn3D.z * (double)m_nRobotInstallDir > dMaxHeight ? vtTeachResult[0].tKeyPtn3D.z * (double)m_nRobotInstallDir : dMaxHeight;
	dMaxHeight = vtTeachResult[1].tKeyPtn3D.z * (double)m_nRobotInstallDir > dMaxHeight ? vtTeachResult[1].tKeyPtn3D.z * (double)m_nRobotInstallDir : dMaxHeight;
	dMaxHeight = vtTeachResult[2].tKeyPtn3D.z * (double)m_nRobotInstallDir > dMaxHeight ? vtTeachResult[2].tKeyPtn3D.z * (double)m_nRobotInstallDir : dMaxHeight;
	dMaxHeight = vtTeachResult[3].tKeyPtn3D.z * (double)m_nRobotInstallDir > dMaxHeight ? vtTeachResult[3].tKeyPtn3D.z * (double)m_nRobotInstallDir : dMaxHeight;
	dMaxHeight *= (double)m_nRobotInstallDir;

	// 交线与平面z=height相交 得端点
	memset(&tPlane, 0, sizeof(tPlane));
	tPlane.c = 1;
	tPlane.d = -dMaxHeight;
	if (false == alg.LinePlateInterSection(tAlgLine, tPlane, tAlgPoint))
	{
		XiMessageBox("立焊自由起点: 线面相交计算失败!");
		return false;
	}

	return true;
}

bool GenericWeld::CalcFreeEndPointStand(const std::vector<T_TEACH_DATA>& tTeachData, T_ALGORITHM_POINT& tAlgPoint)
{
	bool bBasedOnHeight = false; // true:立焊高度取高点 false:立焊高度取低点 (需改为测量两个点 直线与平面交点 取较低交点)
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

	// 获取tTeachData对应的所有测量结果数据
	for (int i = 0; i < tTeachData.size(); i++)
	{
		vtTeachResult.push_back(m_vtTeachResult[tTeachData[i].nMeasurePtnNo]);
	}

	// 获取两个立面上的点 取平面上三条激光线上点，且去除原理每个激光线上远离角点的误差相对最大的两个点，再拟合平面，更准确
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
	//// 获取两个立面上的点
	//XiPointAddToAlgPtn(vtTeachResult[0].vtLeftPtns3D, vtFirstPoints);
	//XiPointAddToAlgPtn(vtTeachResult[1].vtLeftPtns3D, vtFirstPoints);
	//XiPointAddToAlgPtn(vtTeachResult[2].vtLeftPtns3D, vtSecondPoints);
	//XiPointAddToAlgPtn(vtTeachResult[3].vtLeftPtns3D, vtSecondPoints);

	// 拟合两个立板平面
	tPlane1 = alg.CalcPlaneParamRansac(vtFirstPoints, 0.8);
	tPlane2 = alg.CalcPlaneParamRansac(vtSecondPoints, 0.8);

	// 两面相交得到交线
	tAlgLine = PlanePlaneInterLine(tPlane1, tPlane2);

	// 获取立焊高度
	dMinHeight = vtTeachResult[4].tKeyPtn3D.z * (double)m_nRobotInstallDir;
	dMaxHeight = vtTeachResult[5].tKeyPtn3D.z * (double)m_nRobotInstallDir;
	if (dMinHeight > dMaxHeight)
	{
		swap(dMinHeight, dMaxHeight);
	}
	dEndHeight = true == bBasedOnHeight ? dMaxHeight : dMinHeight;
	dEndHeight *= (double)m_nRobotInstallDir;

	// 交线与平面z=height相交 得端点
	memset(&tPlane, 0, sizeof(tPlane));
	tPlane.c = 1;
	tPlane.d = -dEndHeight;
	if (false == alg.LinePlateInterSection(tAlgLine, tPlane, tAlgPoint))
	{
		XiMessageBox("立焊自由终点: 线面相交计算失败!");
		return false;
	}

	return true;
}

void GenericWeld::CheckSeamDataEndpointType(double dDisThreshold)
{
	int nSeamNum = m_vtWeldSeamInfo.size();
	vector<int> vnStartIdx(nSeamNum); // 起点编号索引
	vector<int> vnEndIdx(nSeamNum); // 终点编号索引
	vector<CvPoint3D64f> vtPtns(0); // 点集 相近点重合
	vector<int> vnPtnNum(0); // 点集中各点的数量

	//// 移动到识别后
	////长焊缝分解为短焊缝
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

	//// 保存分段后数据
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
		if (vnPtnNum[vnStartIdx[nSeamNo]] > 1 && false == m_vtWeldSeamData[nSeamNo].StartPointType) // 有两个或以上的点相同为干涉端点
		{
			//已修改
			XUI::MesBox::PopInfo("焊缝{0}起点干涉识别为自由，自动修正", nSeamNo);
			//XiMessageBox("焊缝%d起点干涉识别为自由，自动修正", nSeamNo);
			m_vtWeldSeamData[nSeamNo].StartPointType = true;
			m_vtWeldSeamInfo[nSeamNo].tWeldLine.StartPointType = true;
		}
		if (vnPtnNum[vnEndIdx[nSeamNo]] > 1 && false == m_vtWeldSeamData[nSeamNo].EndPointType) // 有两个或以上的点相同为干涉端点
		{
			//已修改
			XUI::MesBox::PopInfo("焊缝{0}终点干涉识别为自由，自动修正", nSeamNo);
			//XiMessageBox("焊缝%d终点干涉识别为自由，自动修正", nSeamNo);
			m_vtWeldSeamData[nSeamNo].EndPointType = true;
			m_vtWeldSeamInfo[nSeamNo].tWeldLine.EndPointType = true;
		}
	}
}

void GenericWeld::Segmentation()
{
	//长焊缝分解为短焊缝
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

	// 保存分段后数据
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
//		// 第一段
//		tSeamInfo = tWeldSeamInfo;
//		tSeamInfo.tWeldLine.EndPoint.x = tSeamInfo.tWeldLine.StartPoint.x + (dSegmLen - m_dEndpointSearchDis + dOffset) * dDisX / dWeldSeamLen;
//		tSeamInfo.tWeldLine.EndPoint.y = tSeamInfo.tWeldLine.StartPoint.y + (dSegmLen - m_dEndpointSearchDis + dOffset) * dDisY / dWeldSeamLen;
//		tSeamInfo.tWeldLine.EndPoint.z = tSeamInfo.tWeldLine.StartPoint.z + (dSegmLen - m_dEndpointSearchDis + dOffset) * dDisZ / dWeldSeamLen;
//		tSeamInfo.tWeldLine.EndPointType = false; // 不相接
//		tSeamInfo.tAtrribute.dEndHoleSize = 0; // 
//		vtWeldSeams.push_back(tSeamInfo);
//
//		// 中间段
//		for (int i = 1; i < nSegmNum - 1; i++)
//		{
//			tSeamInfo = tWeldSeamInfo;
//			tSeamInfo.tWeldLine.EndPoint.x = tSeamInfo.tWeldLine.StartPoint.x + ((dSegmLen * (i + 1)) - m_dEndpointSearchDis + dOffset) * dDisX / dWeldSeamLen;
//			tSeamInfo.tWeldLine.EndPoint.y = tSeamInfo.tWeldLine.StartPoint.y + ((dSegmLen * (i + 1)) - m_dEndpointSearchDis + dOffset) * dDisY / dWeldSeamLen;
//			tSeamInfo.tWeldLine.EndPoint.z = tSeamInfo.tWeldLine.StartPoint.z + ((dSegmLen * (i + 1)) - m_dEndpointSearchDis + dOffset) * dDisZ / dWeldSeamLen;
//			tSeamInfo.tWeldLine.EndPointType = false; // 不相接
//			tSeamInfo.tAtrribute.dEndHoleSize = 0; // 
//
//			tSeamInfo.tWeldLine.StartPoint.x = tSeamInfo.tWeldLine.StartPoint.x + ((dSegmLen * i) + m_dEndpointSearchDis - dOffset) * dDisX / dWeldSeamLen;
//			tSeamInfo.tWeldLine.StartPoint.y = tSeamInfo.tWeldLine.StartPoint.y + ((dSegmLen * i) + m_dEndpointSearchDis - dOffset) * dDisY / dWeldSeamLen;
//			tSeamInfo.tWeldLine.StartPoint.z = tSeamInfo.tWeldLine.StartPoint.z + ((dSegmLen * i) + m_dEndpointSearchDis - dOffset) * dDisZ / dWeldSeamLen;
//			tSeamInfo.tWeldLine.StartPointType = false; // 不相接
//			tSeamInfo.tAtrribute.dStartHoleSize = 0; // 
//
//			vtWeldSeams.push_back(tSeamInfo);
//		}
//
//		// 最后一段
//		tSeamInfo = tWeldSeamInfo;
//		tSeamInfo.tWeldLine.StartPoint.x = tSeamInfo.tWeldLine.StartPoint.x + ((dSegmLen * (nSegmNum - 1)) + m_dEndpointSearchDis - dOffset) * dDisX / dWeldSeamLen;
//		tSeamInfo.tWeldLine.StartPoint.y = tSeamInfo.tWeldLine.StartPoint.y + ((dSegmLen * (nSegmNum - 1)) + m_dEndpointSearchDis - dOffset) * dDisY / dWeldSeamLen;
//		tSeamInfo.tWeldLine.StartPoint.z = tSeamInfo.tWeldLine.StartPoint.z + ((dSegmLen * (nSegmNum - 1)) + m_dEndpointSearchDis - dOffset) * dDisZ / dWeldSeamLen;
//		tSeamInfo.tWeldLine.StartPointType = false; // 不相接
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
		// 第一段
		tSeamInfo = tWeldSeamInfo;
		tSeamInfo.tWeldLine.EndPoint.x = tSeamInfo.tWeldLine.StartPoint.x + (dSegmLen + dOffset) * dDisX / dWeldSeamLen;
		tSeamInfo.tWeldLine.EndPoint.y = tSeamInfo.tWeldLine.StartPoint.y + (dSegmLen + dOffset) * dDisY / dWeldSeamLen;
		tSeamInfo.tWeldLine.EndPoint.z = tSeamInfo.tWeldLine.StartPoint.z + (dSegmLen + dOffset) * dDisZ / dWeldSeamLen;
		tSeamInfo.tWeldLine.EndPointType = 0; // 分段打断端点类型
		tSeamInfo.tAtrribute.dEndHoleSize = 0; 
		tSeamInfo.tAtrribute.nEndWrapType = 0; // 打断端点不包角
		vtWeldSeams.push_back(tSeamInfo);

		// 中间段
		for (int i = 1; i < nSegmNum - 1; i++)
		{
			tSeamInfo = tWeldSeamInfo;
			tSeamInfo.tWeldLine.EndPoint.x = tSeamInfo.tWeldLine.StartPoint.x + ((dSegmLen * (i + 1)) + dOffset) * dDisX / dWeldSeamLen;
			tSeamInfo.tWeldLine.EndPoint.y = tSeamInfo.tWeldLine.StartPoint.y + ((dSegmLen * (i + 1)) + dOffset) * dDisY / dWeldSeamLen;
			tSeamInfo.tWeldLine.EndPoint.z = tSeamInfo.tWeldLine.StartPoint.z + ((dSegmLen * (i + 1)) + dOffset) * dDisZ / dWeldSeamLen;
			tSeamInfo.tWeldLine.EndPointType = 0; // 分段打断端点类型
			tSeamInfo.tAtrribute.dEndHoleSize = 0;
			tSeamInfo.tAtrribute.nEndWrapType = 0; // 打断端点不包角

			tSeamInfo.tWeldLine.StartPoint.x = tSeamInfo.tWeldLine.StartPoint.x + ((dSegmLen * i) - dOffset) * dDisX / dWeldSeamLen;
			tSeamInfo.tWeldLine.StartPoint.y = tSeamInfo.tWeldLine.StartPoint.y + ((dSegmLen * i) - dOffset) * dDisY / dWeldSeamLen;
			tSeamInfo.tWeldLine.StartPoint.z = tSeamInfo.tWeldLine.StartPoint.z + ((dSegmLen * i) - dOffset) * dDisZ / dWeldSeamLen;
			tSeamInfo.tWeldLine.StartPointType = 0; // 分段打断端点类型
			tSeamInfo.tAtrribute.dStartHoleSize = 0;
			tSeamInfo.tAtrribute.nStartWrapType = 0; // 打断端点不包角

			vtWeldSeams.push_back(tSeamInfo);
		}

		// 最后一段
		tSeamInfo = tWeldSeamInfo;
		tSeamInfo.tWeldLine.StartPoint.x = tSeamInfo.tWeldLine.StartPoint.x + ((dSegmLen * (nSegmNum - 1)) - dOffset) * dDisX / dWeldSeamLen;
		tSeamInfo.tWeldLine.StartPoint.y = tSeamInfo.tWeldLine.StartPoint.y + ((dSegmLen * (nSegmNum - 1)) - dOffset) * dDisY / dWeldSeamLen;
		tSeamInfo.tWeldLine.StartPoint.z = tSeamInfo.tWeldLine.StartPoint.z + ((dSegmLen * (nSegmNum - 1)) - dOffset) * dDisZ / dWeldSeamLen;
		tSeamInfo.tWeldLine.StartPointType = 0; // 分段打断端点类型
		tSeamInfo.tAtrribute.dStartHoleSize = 0;
		tSeamInfo.tAtrribute.nStartWrapType = 0; // 打断端点不包角

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
	// 宝桥龙门添加大车停靠位置（小件焊接时外不轴不运动）
/* *********************************（工件）
*					*
*					*
*					*
*					*
*					*(龙门停靠位置)
*/
// 保证机械臂和工件之间始终保持舒服姿态调整龙门Z值
	double dExAxisZPos = dPartMinHeight - 2000.0;
	// 没有外部Z轴,存在Z轴时注释 //dExAxisZPos = 0.0;
	dExAxisZPos = 0.0;
	// ------------------
	dPartMaxHeight -= dExAxisZPos;
	// 计算偏移方向
	double dAdjustDir = atan2(tTotalDirValue.y, tTotalDirValue.x) * 180 / 3.1415926;
	// 判断是否为合理方向余弦，此判断方法可能不合理需要优化
	if (fabs(tTotalDirValue.x) < 0.5 && fabs(tTotalDirValue.y) < 0.5)
	{
		dAdjustDir = 180;
	}
	// 先根据法向计算一个固定偏移的大车坐标，根据该位置计算机器人运动是否极限，再次计算大车位置
	XI_POINT tMachineCoor;
	tMachineCoor.x = tTotalValue.x / (vtSeamGroup.size() * 2);
	tMachineCoor.y = tTotalValue.y / (vtSeamGroup.size() * 2);
	tMachineCoor.z = dExAxisZPos;
	// 宝桥龙门设备暂定机械臂联动外部轴为X（横梁方向），Y（轨道方向）为附加轴,计算时去除轨道数据
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
		
		if (!m_ptUnit->m_bSingleRobotWork)// 多机
		{
			//0号机械臂： 机器人X要走的坐标 = 不动的机器人坐标系X+原始点在龙门坐标下Y值+基座中心到龙门中心的距离
			//1号机械臂： 机器人X要走的坐标 = 不动的机器人坐标系X-原始点在龙门坐标下Y值-基座中心到龙门中心的距离

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
		else //单机
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
		//单机测试时使用   xia 20231117
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
			XiMessageBoxOk("机器人%d 外部轴%d 目标位置%.3lf 超出设定极限%.3lf - %.3lf!", m_pRobotDriver->m_nRobotNo, dExAxlePos, dMinExAxlePos, dMaxExAxlePos);
			continue;
		}

		if (dExAxisPosTrack > dMaxTrackPos || dExAxisPosTrack < dMinTraclPos)
		{
			bIsContinueRun = false;
			XiMessageBoxOk("机器人%d 外部轴%d 目标位置%.3lf 超出设定极限%.3lf - %.3lf!", m_pRobotDriver->m_nRobotNo, dExAxisPosTrack, dMinTraclPos, dMaxTrackPos);
			continue;
		}*/

		std::vector<T_ROBOT_COORS> vtRobotCoors, vtTotalRobotCoors;
		// 去除外部Y和外部Z值
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
		XUI::MesBox::PopOkCancel("{0}号机器人计算大车位置失败", m_pRobotDriver->m_nRobotNo);
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
	// 暂时闭合圆弧和立缝不进入跟踪计算
	for (int nWeldNo = 0; nWeldNo < m_vvtWeldLineInfoGroup[nGroupNo].size(); nWeldNo++)
	{
		if (10 == m_vvtWeldLineInfoGroup[nGroupNo][nWeldNo].tAtrribute.nStartWrapType)// 立缝
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
//	// 判断方法: 1.已知终点(干涉终点、提前搜索起始结尾点), 2.用理论长度,在起始位置留焊时也搜索两次端点， 3.焊接过程中判断(自由终点)
//	// 根据判断方法确定单条焊缝有几次搜索几次结尾点
//	int bRst = 1;
//	switch (nMethod)
//	{
//	case 1: bRst = 2; break;
//	case 2: bRst = 1; break;
//	case 3: bRst = 1; break;
//	default:XiMessageBoxOk("不存在跟踪终点判断方法%d!"); break;
//	}
//	return bRst;
//}

bool GenericWeld::CalcAndTrackWeldingTry(int nGroupNo)
{
	// 跟踪焊缝所需数据，搜索/测量得到初始段数据（不小于跟踪手眼长度），结尾点
	double dStepDis = 3.0;
	vector<T_ROBOT_COORS> vtWeldCoors; // 正向轨迹
	vector<T_ROBOT_COORS> vtReverseCoors; //反相轨迹
	vector<int> vtWeldCoorsType;
	E_WELD_SEAM_TYPE eWeldSeamType;
	double dExPos = 0.0;
	
	
	// 修正焊缝组nGroup中每个焊缝
	for (int nWeldNo = 0; nWeldNo < m_vvtWeldLineInfoGroup[nGroupNo].size(); nWeldNo++)
	{
		// 演示只取一组内第一条焊缝，此次分组包角焊缝分为一组，最多两条焊缝，其余焊缝单独作为一组
		
		if (nWeldNo > 0&& DetermineWarpMode(nGroupNo)>0)
		{
			break;
		}
		// ----------------------------
		m_pScanInit->m_pTraceModel->m_eStartWrapAngleType = E_WRAPANGLE_EMPTY_SINGLE;
		// 搜索端点类型，单条缝提前搜索起始结尾数据（已知终点类型），或者只搜索起始数据（跟踪过程搜索结尾点、理论长度焊接）
		// 根据判断结尾类型决定加载几次结尾数据
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
				//已修改
				XUI::MesBox::PopInfo("{0}文件加载数据失败", strFileName.GetBuffer());
				//XiMessageBox("%s 文件加载数据失败", strFileName);
				return false;
			}
			if (vtpoints.size() > 10) {
				// 对初始轨迹进行样条滤波
				CHECK_BOOL_RETURN(m_pScanInit->DataPointsProcess(m_ptUnit->GetRobotCtrl(), vtpoints, vtCoutPoints, vtCoutRobotCoors));
			}
			else {
				vtCoutPoints.insert(vtCoutPoints.end(), vtpoints.begin(), vtpoints.end());
				// 直缝 更新理论长度
				double dLength = TwoPointDis(
					vtCoutPoints[0].x, vtCoutPoints[0].y, vtCoutPoints[0].z,
					vtCoutPoints[vtCoutPoints.size() - 1].x, vtCoutPoints[vtCoutPoints.size() - 1].y, vtCoutPoints[vtCoutPoints.size() - 1].z);

				m_vvtWeldLineInfoGroup[nGroupNo][nWeldNo].tAtrribute.dThoeryLength = dLength;
			}
		}
		//记录跟踪理论数据
		m_vtTrackTheoryTrack.insert(m_vtTrackTheoryTrack.end(), vtCoutPoints.begin(), vtCoutPoints.end());
		if (false) //理论长度、跟踪过程搜索结尾点
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
		// 起点类型为2，3时，起终点都要搜索
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
			double dChangeDisS = 45.0;	// 变姿态距离
			double dChangeDisE = 45.0;	// 变姿态距离
			double dChangeAngle = 30.0; // 变姿态角度
			double dPtnInterval = 2.0;  // 轨迹点间隔

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
			// 获取焊缝类型 生成焊接轨迹
			eWeldSeamType = GetWeldSeamType(tSeam);
			//double dTempVar = dChangeDisS;

			int nChangePtnNum = (int)(dChangeDisS / dPtnInterval); // 变姿态点数

			double dStepChangeAngle = (double)m_nRobotInstallDir * dChangeAngle / (double)nChangePtnNum; // 相邻点姿态变化角度
			int nChangePtnNumS = (int)(dChangeDisS / dPtnInterval); // 变姿态点数
			int nChangePtnNumE = (int)(dChangeDisE / dPtnInterval); // 变姿态点数
			double dStepChangeAngleS = (double)m_nRobotInstallDir * dChangeAngle / (double)nChangePtnNumS; // 相邻点姿态变化角度
			double dStepChangeAngleE = (double)m_nRobotInstallDir * dChangeAngle / (double)nChangePtnNumE; // 相邻点姿态变化角度

			int nWeldTrackPtnNum = vtWeldCoors.size();

			// 干涉端点变姿态  删除过焊孔轨迹
			if (E_FLAT_SEAM == eWeldSeamType && 0 < tSeam.EndPointType) // 平焊干涉终点 
			{
				int nBaseIndex = nWeldTrackPtnNum - 1 - nChangePtnNumE;
				double dSrcRz = vtWeldCoors[nBaseIndex].dRZ;
				for (int n = nWeldTrackPtnNum - nChangePtnNumE; n < nWeldTrackPtnNum; n++)
				{
					vtWeldCoors[n].dRZ = dSrcRz - ((n - nBaseIndex) * dStepChangeAngleE); // 姿态减小(和正座倒挂安装方式相关)(支持内外侧焊缝)
				}
			}
			if (E_FLAT_SEAM == eWeldSeamType && 0 < tSeam.StartPointType) // 平焊干涉起点
			{
				double dSrcRz = vtWeldCoors[nChangePtnNumS].dRZ;
				for (int n = nChangePtnNumS - 1; n >= 0; n--)
				{
					vtWeldCoors[n].dRZ = dSrcRz + ((nChangePtnNumS - n) * dStepChangeAngleS); // 姿态增加(和正座倒挂安装方式相关)(支持内外侧焊缝)
				}
			}
			// 计算龙门外部轴
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
			bool IsCircle = dStartEndDis < 1.0 ? TRUE : FALSE; // 闭合圆弧	
			// 补偿测量相机跟踪相机间精度调整量，临时操作
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
			if (DetermineWarpMode(nGroupNo) > 0) // 包角
			{	
				// 包角类型
				m_pScanInit->m_pTraceModel->m_eStartWrapAngleType = E_WRAPANGLE_JUMP;
				// 如果包角			
				
				double dChangeDisS = 35.0;	// 变姿态距离
				double dChangeDisE = 35.0;	// 变姿态距离
				double dChangeAngle = 30.0; // 变姿态角度
				double dPtnInterval = 2.0;  // 轨迹点间隔
				int nChangePtnNum = (int)(dChangeDisS / dPtnInterval); // 变姿态点数
				double dStepChangeAngle = (double)m_nRobotInstallDir * dChangeAngle / (double)nChangePtnNum; // 相邻点姿态变化角度
				int nChangePtnNumS = (int)(dChangeDisS / dPtnInterval); // 变姿态点数
				int nChangePtnNumE = (int)(dChangeDisE / dPtnInterval); // 变姿态点数
				double dStepChangeAngleS = (double)m_nRobotInstallDir * dChangeAngle / (double)nChangePtnNumS; // 相邻点姿态变化角度
				double dStepChangeAngleE = (double)m_nRobotInstallDir * dChangeAngle / (double)nChangePtnNumE; // 相邻点姿态变化角度
				int nWeldTrackPtnNum = vtWeldCoors.size();
				// 以下包角暂时不用，临时实现包角功能，在此判断同组内包角次数，此次演示工件只存在一头包或两头包
				//double dThink = 12.0;// 板厚
				//----------------------------------------------------------------------------------------------------------
				//如果需要包角:-头包：跳抢或旋转，两头包；两头都跳，两头都旋转，一头旋转一头包
				//根据初始段轨迹，理论长度(或实际结尾点)生成完整的理论数据，包含跟踪轨迹，包角轨迹，跳枪轨迹
				m_pScanInit->m_pTraceModel->dBoardThink = 10.0;
				vtWeldCoors.clear();
				vtReverseCoors.clear();
				// 结尾跳枪,如果一头包角会产生两个结尾点:

				//e----------------------------
				//								|
				//s----------------------------e
				// 如果两头包角会产生三个结尾点:
				// e--------------------------s
				// |						  |
				// s-----------------e s------e

				// 包一次
				double dThoeryLen = m_vvtWeldLineInfoGroup[nGroupNo][nWeldNo].tAtrribute.dThoeryLength;

				dThoeryLen = TwoPointDis(vtCoutPoints[0].x, vtCoutPoints[0].y, vtCoutPoints[0].z,
					vtCoutPoints[vtCoutPoints.size() - 1].x, vtCoutPoints[vtCoutPoints.size() - 1].y, vtCoutPoints[vtCoutPoints.size() - 1].z);
				size_t i = 0;
				if (2 == DetermineWarpMode(nGroupNo))
				{
					i = (24 / dPtnInterval) + 0;// 1:起始点向里偏移一步//包角长度
				}
				else if (0 != m_vvtWeldLineInfoGroup[nGroupNo][nWeldNo].tWeldLine.StartPointType)
				{
					i = 5;
				}
				
				int n1 = dThoeryLen / dPtnInterval;
				int n2 = 24 / dPtnInterval;//包角长度
				if (n1 * 2 < dThoeryLen)
				{
					n1 += 4; //结尾包角添加几步
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
					// 计算对面坐标
					RobotCoordPosOffset(tPoint, dNormal + 180.0, m_pScanInit->m_pTraceModel->dBoardThink /*+ dNormalDirOffset*/);
					tPoint.dRZ = DirAngleToRz(dNormal + 180.0);
					//tPoint.dZ += dZValDirOffset;
					vtReverseCoors.push_back(tPoint);
				}
				// 将对面数据反向排布
				reverse(vtReverseCoors.begin(), vtReverseCoors.end());
				CString strFileAuto = OUTPUT_PATH + m_ptUnit->m_tContralUnit.strUnitName + "\\" + RECOGNITION_FOLDER;
				if (1 == DetermineWarpMode(nGroupNo))
				{
					// 结尾点类型，用于起点干涉段结尾自由包角工件类型，反向结尾点变姿态
					m_pScanInit->m_pTraceModel->m_vtEndPointType.push_back(m_vvtWeldLineInfoGroup[nGroupNo][nWeldNo].tWeldLine.EndPointType);
					m_pScanInit->m_pTraceModel->m_vtEndPointType.push_back(m_vvtWeldLineInfoGroup[nGroupNo][nWeldNo].tWeldLine.StartPointType);
					m_pScanInit->m_pTraceModel->m_vtRealEndpointCoor.push_back(vtWeldCoors[vtWeldCoors.size() - 1]);
					m_pScanInit->m_pTraceModel->m_vtRealEndpointCoor.push_back(vtReverseCoors[vtReverseCoors.size() - 1]);				

					if (E_FLAT_SEAM == eWeldSeamType && 0 < tSeam.StartPointType)// 平焊干涉起点
					{
						double dSrcRz = vtWeldCoors[nChangePtnNumS].dRZ;
						for (int n = nChangePtnNumS - 1; n >= 0; n--)
						{
							vtWeldCoors[n].dRZ = dSrcRz + ((nChangePtnNumS - n) * dStepChangeAngleS); // 姿态增加(和正座倒挂安装方式相关)(支持内外侧焊缝)
						}
					}
					else
					{
						nChangePtnNum = 0;
					}

					// 添加完整轨迹
					// 添加过度点
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
				// 包两次
				if (2 == DetermineWarpMode(nGroupNo))
				{
					// 两面包角不变姿态
					nChangePtnNum = 0;
					m_pScanInit->m_pTraceModel->m_vtEndPointType.push_back(0);
					m_pScanInit->m_pTraceModel->m_vtEndPointType.push_back(0);
					m_pScanInit->m_pTraceModel->m_vtEndPointType.push_back(0);

					m_pScanInit->m_pTraceModel->m_vtRealEndpointCoor.clear();
					m_pScanInit->m_pTraceModel->m_vtRealEndpointCoor.push_back(vtReverseCoors[vtReverseCoors.size() - 1]);
					m_pScanInit->m_pTraceModel->m_vtRealEndpointCoor.push_back(vtWeldCoors[vtWeldCoors.size() - 1]);


					vector<T_ROBOT_COORS> vtWeldCoorsTemp(vtWeldCoors); // 正向轨迹
					vtWeldCoors.clear();
					vtWeldCoors.insert(vtWeldCoors.end(), vtReverseCoors.end() - 100, vtReverseCoors.end());
					// 添加完整轨迹
					// 添加过度点
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
				// 生成完整轨迹
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
					// 生成完整轨迹
					for (int i = 0; i < vtCoutPoints.size(); i++)
					{
						vtWeldCoorsType.push_back(E_WELD_TRACK);
					}

					int nStartChangeStepNo = 0, nEndChangeStepNo = 0;
					CalcRobotPostureCoors(vtCoutPoints, m_dPlatWeldRx, m_dPlatWeldRy, dStepDis, 35.0, 35.0, 35.0, 35.0, /*1*/ 0 != m_vvtWeldLineInfoGroup[nGroupNo][nWeldNo].tWeldLine.StartPointType, 
						0, dStartHoleSize, dEndHoleSize, nStartChangeStepNo, nEndChangeStepNo, vtWeldCoors);
					// 补偿初始数据和跟踪相机一致
					for (size_t i = 0; i < vtWeldCoors.size(); i++)
					{
						RobotCoordPosOffset(vtWeldCoors[i], dNormal, dNormalDirOffset, dZValDirOffset);
					}
					//---------------------------------------------------------------------------------

					// 江南平缝结尾变姿态
					//m_vvtWeldLineInfoGroup[nGroupNo][nWeldNo].tAtrribute.bEndFixScan = true;

					for (int i = 0; i < nStartChangeStepNo; i++)
					{
						vtWeldCoorsType[i] = E_WELD_TRACK | E_WELD_TRACK_CHANGE_POSTURE;
					}

					vector<XI_POINT> vtTrack;
					XI_POINT tSartPoint = vtCoutPoints.at(vtCoutPoints.size() - 2), tEndpoint = vtCoutPoints.at(vtCoutPoints.size() - 1);
					// 去除结尾数据
					vtWeldCoors.pop_back();
					vtWeldCoors.pop_back();
					// 计算连续理论轨迹
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
						//已修改
						XUI::MesBox::PopInfo("{0}文件加载数据失败", strFileName.GetBuffer());
						//XiMessageBox("%s 文件加载数据失败", strFileName);
						return false;
					}
					CHECK_BOOL_RETURN(m_pScanInit->DataPointsProcess(m_ptUnit->GetRobotCtrl(), vtpoints, vtCoutPoints, vtCoutRobotCoors));
					int nStartChangeStepNo, nEndChangeStepNo;
					CalcRobotPostureCoors(vtCoutPoints, m_dPlatWeldRx, m_dPlatWeldRy, dStepDis, 45.0, 45.0, 45.0, 45.0, 0, 0, 0, 0,
						nStartChangeStepNo, nEndChangeStepNo, vtWeldCoors);
					// 生成完整轨迹
					for (int i = 0; i < vtWeldCoors.size(); i++)
					{
						vtWeldCoorsType.push_back(E_WELD_TRACK);
					}
				}
			}
		}

		// 保存 序号 直角坐标 外部轴坐标 焊接类型
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
//	// 跟踪焊缝所需数据，搜索/测量得到初始段数据（不小于跟踪手眼长度），结尾点
//	double dStepDis = 3.0;
//	vector<T_ROBOT_COORS> vtWeldCoors; // 正向轨迹
//	vector<T_ROBOT_COORS> vtReverseCoors; //反相轨迹
//	vector<int> vtWeldCoorsType;
//	E_WELD_SEAM_TYPE eWeldSeamType;
//	double dExPos = 0.0;
//	// 修正焊缝组nGroup中每个焊缝
//	for (int nWeldNo = 0; nWeldNo < m_vvtWeldLineInfoGroup[nGroupNo].size(); nWeldNo++)
//	{
//		// 演示只取一组内第一条焊缝，此次分组包角焊缝分为一组，最多两条焊缝，其余焊缝单独作为一组
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
//		// 起点类型为2，3时，起终点都要搜索
//		int nStartType = m_vvtWeldLineInfoGroup.at(nGroupNo).at(nWeldNo).tWeldLine.StartPointType;
//
//		LineOrCircularArcWeldingLine tSeam = m_vvtWeldLineInfoGroup[nGroupNo][nWeldNo].tWeldLine;
//		if (!m_vvtWeldLineInfoGroup[nGroupNo][nWeldNo].tAtrribute.bWeldMode)
//		{
//			vector<XI_POINT> vtpoints;
//			CString strFileName;
//			strFileName.Format("%s%s%sEndpointCoors-%d-0.txt", OUTPUT_PATH, m_pRobotDriver->m_strRobotName, RECOGNITION_FOLDER, nGroupNo);
//			if (!LoadPointsData(vtpoints, strFileName)) {
//				XiMessageBox("%s 文件加载数据失败", strFileName);
//				return false;
//			}
//			T_LINE_PARA tLineParam = CalcLineParamRansac(vtpoints, 0.7);
//			double dNormal = atan2(tLineParam.dDirY, tLineParam.dDirX) * 180 / 3.1415926 - 90 * m_nRobotInstallDir;;
//			double dChangeDisS = 45.0;	// 变姿态距离
//			double dChangeDisE = 45.0;	// 变姿态距离
//			double dChangeAngle = 45.0; // 变姿态角度
//			double dPtnInterval = 2.0;  // 轨迹点间隔
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
//			// 获取焊缝类型 生成焊接轨迹
//			eWeldSeamType = GetWeldSeamType(tSeam);
//			double dTempVar = dChangeDisS;
//
//			int nChangePtnNum = (int)(dChangeDisS / dPtnInterval); // 变姿态点数
//			double dStepChangeAngle = (double)m_nRobotInstallDir * dChangeAngle / (double)nChangePtnNum; // 相邻点姿态变化角度
//			int nChangePtnNumS = (int)(dChangeDisS / dPtnInterval); // 变姿态点数
//			int nChangePtnNumE = (int)(dChangeDisE / dPtnInterval); // 变姿态点数
//			double dStepChangeAngleS = (double)m_nRobotInstallDir * dChangeAngle / (double)nChangePtnNumS; // 相邻点姿态变化角度
//			double dStepChangeAngleE = (double)m_nRobotInstallDir * dChangeAngle / (double)nChangePtnNumE; // 相邻点姿态变化角度
//
//			int nWeldTrackPtnNum = vtWeldCoors.size();
//
//			// 干涉端点变姿态  删除过焊孔轨迹
//			if (E_FLAT_SEAM == eWeldSeamType && true == tSeam.EndPointType) // 平焊干涉终点 
//			{
//				int nBaseIndex = nWeldTrackPtnNum - 1 - nChangePtnNumE;
//				double dSrcRz = vtWeldCoors[nBaseIndex].dRZ;
//				for (int n = nWeldTrackPtnNum - nChangePtnNumE; n < nWeldTrackPtnNum; n++)
//				{
//					vtWeldCoors[n].dRZ = dSrcRz - ((n - nBaseIndex) * dStepChangeAngleE ); // 姿态减小(和正座倒挂安装方式相关)(支持内外侧焊缝)
//				}
//			}
//			if (E_FLAT_SEAM == eWeldSeamType && true == tSeam.StartPointType) // 平焊干涉起点
//			{
//				double dSrcRz = vtWeldCoors[nChangePtnNumS].dRZ;
//				for (int n = nChangePtnNumS - 1; n >= 0; n--)
//				{
//					vtWeldCoors[n].dRZ = dSrcRz + ((nChangePtnNumS - n) * dStepChangeAngleS); // 姿态增加(和正座倒挂安装方式相关)(支持内外侧焊缝)
//				}
//			}
//			// 计算龙门外部轴
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
//			bool IsCircle = dStartEndDis < 1.0 ? TRUE : FALSE; // 闭合圆弧
//			// 闭合圆弧
//			vector<XI_POINT> vtpoints, vtCoutPoints;
//			vector<T_ROBOT_COORS> vtCoutRobotCoors;
//			vector<T_ANGLE_PULSE> vtWeldPulse;
//			if (DetermineWarpMode(nGroupNo) > 0) // 包角
//			{
//				// 加载初始段数据
//				CString strFileName;
//				strFileName.Format("%s%s%sEndpointCoors-%d-0.txt", OUTPUT_PATH, m_pRobotDriver->m_strRobotName, RECOGNITION_FOLDER, nGroupNo);
//				if (!LoadPointsData(vtpoints, strFileName)) {
//					XiMessageBox("%s 文件加载数据失败", strFileName);
//					return false;
//				}
//				// 如果包角
//				T_LINE_PARA tLineParam = CalcLineParamRansac(vtpoints, 0.7);
//				double dNormal = atan2(tLineParam.dDirY, tLineParam.dDirX) * 180 / 3.1415926 - 90 * m_nRobotInstallDir;;
//				double dChangeDisS = 45.0;	// 变姿态距离
//				double dChangeDisE = 45.0;	// 变姿态距离
//				double dChangeAngle = 45.0; // 变姿态角度
//				double dPtnInterval = 2.0;  // 轨迹点间隔
//				m_pScanInit->m_pTraceModel->m_vtRealEndpointCoor.clear();
//				// 以下包角暂时不用，临时实现包角功能，在此判断同组内包角次数，此次演示工件只存在一头包或两头包
//				double dThink = 10.0;// 板厚
//				//----------------------------------------------------------------------------------------------------------
//				//如果需要包角:-头包：跳抢或旋转，两头包；两头都跳，两头都旋转，一头旋转一头包
//				//根据初始段轨迹，理论长度(或实际结尾点)生成完整的理论数据，包含跟踪轨迹，包角轨迹，跳枪轨迹
//				vtWeldCoors.clear();
//				vtReverseCoors.clear();
//				// 结尾跳枪,如果一头包角会产生两个结尾点:
//
//				//e----------------------------
//				//								|
//				//s----------------------------e
//				// 如果两头包角会产生三个结尾点:
//				// e--------------------------s
//				// |						  |
//				// s-----------------e s------e
//
//				// 包一次
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
//					// 计算对面坐标
//					RobotCoordPosOffset(tPoint, dNormal + 180.0, dThink + 2.0);
//					tPoint.dRZ = DirAngleToRz(dNormal + 180.0);
//					tPoint.dZ += 1.5;
//					vtReverseCoors.push_back(tPoint);
//				}
//				// 将对面数据反向排布
//				reverse(vtReverseCoors.begin(), vtReverseCoors.end());
//				if (1 == DetermineWarpMode(nGroupNo))
//				{
//					m_pScanInit->m_pTraceModel->m_vtRealEndpointCoor.push_back(vtWeldCoors[vtWeldCoors.size() - 1]);
//					m_pScanInit->m_pTraceModel->m_vtRealEndpointCoor.push_back(vtReverseCoors[vtReverseCoors.size() - 1]);
//				}
//				SaveCoordToFile(vtWeldCoors, "AAAAA-vtWeldCoors.txt");
//				SaveCoordToFile(vtReverseCoors, "AAAAA-vtReverseCoors.txt");
//				// 包两次
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
//				// 生成完整轨迹
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
//					// 非闭合圆弧，两段轨迹，初始数据和结尾点		
//					for (int i = 0; i < 2; i++)
//					{
//						vtpoints.clear();
//						if (i == 0 || nStartType > 1)
//						{
//							// 演示件都不提前搜索终点，自由端终点数据使用理论数据
//							CString strFileName;
//							strFileName.Format("%s%s%sEndpointCoors-%d-%d.txt", OUTPUT_PATH, m_pRobotDriver->m_strRobotName, RECOGNITION_FOLDER, nGroupNo, i);
//							if (!LoadPointsData(vtpoints, strFileName)) {
//								XiMessageBox("%s 文件加载数据失败", strFileName);
//								return false;
//							}
//						}
//						if (vtpoints.size() > 10) {
//							// 对初始轨迹进行样条滤波
//							CHECK_BOOL_RETURN(m_pScanInit->DataPointsProcess(m_ptUnit->GetRobotCtrl(), vtpoints, vtCoutPoints, vtCoutRobotCoors));
//						}
//						else {
//							if (/*2 == */nStartType > 1)
//							{
//								vtCoutPoints.insert(vtCoutPoints.end(), vtpoints.begin(), vtpoints.end());
//
//								// 更新理论长度
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
//					// 生成完整轨迹
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
//						// 计算连续理论轨迹
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
//						XiMessageBox("%s 文件加载数据失败", strFileName);
//						return false;
//					}
//					CHECK_BOOL_RETURN(m_pScanInit->DataPointsProcess(m_ptUnit->GetRobotCtrl(), vtpoints, vtCoutPoints, vtCoutRobotCoors));
//					int nStartChangeStepNo, nEndChangeStepNo;
//					CalcRobotPostureCoors(vtCoutPoints, m_dPlatWeldRx, m_dPlatWeldRy, dStepDis, 45.0, 45.0, 45.0, 45.0, 0, 0, 0, 0,
//						nStartChangeStepNo, nEndChangeStepNo, vtWeldCoors);
//					// 生成完整轨迹
//					for (int i = 0; i < vtWeldCoors.size(); i++)
//					{
//						vtWeldCoorsType.push_back(E_WELD_TRACK);
//					}
//				}
//			}					
//		}
//
//		// 保存 序号 直角坐标 外部轴坐标 焊接类型
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








