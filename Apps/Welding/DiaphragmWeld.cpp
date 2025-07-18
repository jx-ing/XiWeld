// DiaphragmWeld.cpp: implementation of the CDiaphragmWeld class.
//
//////////////////////////////////////////////////////////////////////
#include "stdafx.h"
#include <iostream>
#include <string>
#include <sstream>
#include <cmath>
#include "WrapAngleParam.h"
#include "DiaphragmWeld.h"
#include "WeldParamProcess.h"
#include "LocalFiles/ExLib/ALGO/include/correct_circle.h"



#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#define new DEBUG_NEW
#endif

#define  CHECK_ROBOT_EMG_BREAK(ptUnit) if (ptUnit->RobotEmg(false) == true) \
{ \
	break ; \
}
#define  CHECK_STOP_BREAK(pRobotDriver) if (pRobotDriver->m_eThreadStatus == INCISEHEAD_THREAD_STATUS_STOPPED) \
{ \
	break ; \
}

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////


/**
 * @brief 隔板类构造函数（直接继承先测后焊）
 * @param ptUnit 
 * @param eWorkPieceType 
*/
CDiaphragmWeld::CDiaphragmWeld(CUnit* ptUnit, E_WORKPIECE_TYPE eWorkPieceType)
	:WeldAfterMeasure(ptUnit, eWorkPieceType)
{
	m_dHandEyeDis = 100.0;
}

CDiaphragmWeld::~CDiaphragmWeld()
{

}

void CDiaphragmWeld::SetRecoParam()
{
}
bool CDiaphragmWeld::WeldSeamGrouping(int& nWeldGroupNum)
{
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
	//CheckSeamDataEndpointType(dGroupDisThreshold); // 检查端点类型是否正确(避免干涉识别为自由)
	for (int nWeldNo = 0; nWeldNo < m_vtWeldSeamInfo.size(); nWeldNo++)
	{
		WeldLineInfo tLineSeamInfo = m_vtWeldSeamInfo[nWeldNo];
		LineOrCircularArcWeldingLine tLineSeam = tLineSeamInfo.tWeldLine;
		double dWeldSeamLen = TwoPointDis(
			tLineSeam.StartPoint.x, tLineSeam.StartPoint.y, tLineSeam.StartPoint.z,
			tLineSeam.EndPoint.x, tLineSeam.EndPoint.y, tLineSeam.EndPoint.z);
		if (IsStandWeldSeam(tLineSeam.StartNormalVector)) // 立焊起点终点法向量Z值为0
		{
			if (dWeldSeamLen < dMinStandSeamLen)
			{
				//已修改
				XUI::MesBox::PopInfo("立焊焊缝%d 长度小于%.1lf,自动删除！", nWeldNo, dMinStandSeamLen);
				//XiMessageBoxOk("立焊焊缝%d 长度小于%.1lf,自动删除！", nWeldNo, dMinStandSeamLen);
				continue;
			}
			vtWeldLineStandard.push_back(tLineSeamInfo);
		}
		else
		{
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
	// 分配焊接方式
	DetermineWeldingMode();
	// 保存分组结果
	SaveWeldSeamGroupInfo();
	return true;
}
bool CDiaphragmWeld::CalcMeasureTrack(int nGroupNo, std::vector<T_ROBOT_COORS>& vtMeasureCoord, std::vector<T_ANGLE_PULSE>& vtMeasurePulse, vector<int>& vnMeasureType, double& dExAxlePos, double& dSafeHeight)
{
	return 0;
}
bool CDiaphragmWeld::CalcWeldTrack(int nGroupNo)
{
	return 0;
}

bool CDiaphragmWeld::PointCloudProcess(bool bUseModel, CvPoint3D64f* pPointCloud, int PointCloudSize)
{
	if (bUseModel)
		return PointCloudProcessWithModel();

	m_vtWeldSeamData.clear();
	int nWeldingSeamNumber = 0;
	bool bInstallDir = 1 == m_nRobotInstallDir ? true : false;
	vector<CvPoint3D64f> vtPointCloud;
	LineOrCircularArcWeldingLine* pLineOrCircularArcWeldingLine = NULL;	// 李伟
	LineOrCircularArcWeldingSeam* pLineOrCircularArcWeldingSeam = NULL;	// 腾龙
	try
	{
		if (NULL == pPointCloud){
			LoadContourData(m_pRobotDriver, m_sPointCloudFileName, vtPointCloud);
			pPointCloud = (CvPoint3D64f*)vtPointCloud.data();
			PointCloudSize = vtPointCloud.size();
		}
		switch (m_eWorkPieceType)
		{
		case E_DIAPHRAGM:
			// 腾龙识别接口 隔板
			pLineOrCircularArcWeldingSeam = RecognizeWeldingSeamsAccordingToRule4(pPointCloud, 
																				  PointCloudSize, 
																				  &nWeldingSeamNumber,
																				  m_dSideBoardThick,
																				  m_dZPallet,
																				  8, 
																				  1000);
			SavePointCloudProcessResult(pLineOrCircularArcWeldingSeam, nWeldingSeamNumber);
			ReleaseLineOrCircularArcWeldingSeamsPointer(&pLineOrCircularArcWeldingSeam);
			break;
		case E_LINELLAE:
			if (0 == m_nExPartType){
				// 工字钢 小件接口 
				pLineOrCircularArcWeldingLine = SmallSamplePointCloudToWeldingLines((Three_DPoint*)pPointCloud,
																					PointCloudSize,
																					&nWeldingSeamNumber,
																					bInstallDir,
																					m_dZPallet,
																					5.0,
																					15, 0, 0, false, false, 8, 24, true, false, 0, false);
			}
			else{
				// 工字钢 牛腿接口
				//pLineOrCircularArcWeldingLine = BracketPointCloudToWeldingLines((Three_DPoint*)pPointCloud,
				//																PointCloudSize,
				//																&nWeldingSeamNumber,
				//																bInstallDir,
				//																m_dZPallet,
				//																5.0,false);
			}
			SavePointCloudProcessResult(pLineOrCircularArcWeldingLine, nWeldingSeamNumber);
			ReleaseWeldingLines(&pLineOrCircularArcWeldingLine);
			break;
		case SMALL_PIECE:
			// 李伟识别接口 小散件
			pLineOrCircularArcWeldingLine = SmallSamplePointCloudToWeldingLines((Three_DPoint*)pPointCloud,
																				PointCloudSize,
																				&nWeldingSeamNumber,
																				bInstallDir,
																				m_dZPallet,
																				5.0,
																				15, 0, 0, false, false, 8, 24, true, false, 0, false);
			SavePointCloudProcessResult((LineOrCircularArcWeldingLine*)pLineOrCircularArcWeldingLine, nWeldingSeamNumber);
			ReleaseWeldingLines(&pLineOrCircularArcWeldingLine);
			break;
		case STIFFEN_PLATE:
			// 李伟识别接口 吊车梁
			pLineOrCircularArcWeldingLine = BeamPointCloudToWeldingLines((Three_DPoint*)pPointCloud,
																		 PointCloudSize,
																		 &nWeldingSeamNumber,
																		 bInstallDir,
																		 m_dZPallet,
																		 5.0);
			SavePointCloudProcessResult((LineOrCircularArcWeldingLine*)pLineOrCircularArcWeldingLine, nWeldingSeamNumber);
			ReleaseWeldingLines(&pLineOrCircularArcWeldingLine);
			break;
		case E_END_PLATE:
			break;
		case E_CORBEL:
			//// 李伟识别接口 牛腿
			//pLineOrCircularArcWeldingLine = BracketPointCloudToWeldingLines((Three_DPoint*)pPointCloud,
			//																PointCloudSize,
			//																&nWeldingSeamNumber,
			//																bInstallDir,
			//																m_dZPallet,
			//																5.0, false);
			SavePointCloudProcessResult((LineOrCircularArcWeldingLine*)pLineOrCircularArcWeldingLine, nWeldingSeamNumber);
			ReleaseWeldingLines(&pLineOrCircularArcWeldingLine);
			break;
		default:
			XiMessageBox("通用类：类型错误 处理失败！");
			return false;
		}

	}
	catch (exception _XiException) {
		TRACE("%s\n", _XiException.what());
		XiMessageBox("点云处理异常");
		return false;
	}

	//CDiaphragmWeld cDiaphragmWeld(pRobotDriver, m_pScanInit);
	/*cDiaphragmWeld.*/InitWorkPieceReleveInfo(m_pRobotDriver/*m_pRobotDriver*/);

	//已修改
	XUI::MesBox::PopInfo("焊道数量为:{0}", nWeldingSeamNumber);
	//XiMessageBox("焊道数量为:%d", nWeldingSeamNumber);
	if (nWeldingSeamNumber <= 0)
	{
		return false;
	}

	return true;
}

void CDiaphragmWeld::WriteLimitHight(double dMaxH, double dMinH)
{
	COPini OPini;
	CString cstr;
	cstr = DATA_PATH + m_ptUnit->GetUnitName() + "\\SpeedAndSafeHeight.ini";
	OPini.SetFileName(cstr);
	OPini.SetSectionName("LimitHeight");
	OPini.WriteString("WorkpieceHighSpot", dMaxH);
	OPini.WriteString("WorkpieceLowSpot", dMinH);
}

void CDiaphragmWeld::LoadLimitHight()
{
	COPini OPini;
	CString cstr;
	cstr = DATA_PATH + m_ptUnit->GetUnitName() + "\\SpeedAndSafeHeight.ini";
	OPini.SetFileName(cstr);
	OPini.SetSectionName("LimitHeight");
	OPini.ReadString("WorkpieceHighSpot", &m_dWorkPieceHighTop);
	OPini.ReadString("WorkpieceLowSpot", &m_dWorkPieceLowTop);
	OPini.ReadString("RobotLimitHighSpot", &m_dRobotHighTop);
	OPini.ReadString("RobotLimitLowSpot", &m_dRobotHLowTop);
}

bool CDiaphragmWeld::SwingTracking(CRobotDriverAdaptor* pRobotDriver, std::vector<T_ROBOT_COORS> vtRobotCoors, double dSpeed)
{
	if (!m_bSwingState) {
		return true;
	}
	//1.准备参数 
	// 先分离机器人坐标和外部轴坐标 然后需要将外部轴和机器人分别走到起点 分解焊接速度确定外部轴和机器人的速度 合并参数到焊接结构体中
	// 外部轴由于一直匀速运动所以不用合并参数 但是需要记录世界坐标的外部轴坐标(测试和错误处理使用) 
	std::vector<double> vdAxisRealCoors; //外部轴坐标
	std::vector<T_ROBOT_COORS> vtRobotRealCoors; //机器人坐标
	m_pRobotDriver = pRobotDriver;
	if (vtRobotCoors.size() == 0) {
		XiMessageBox("SwingTracking接收到的轨迹为空！！！");
		WriteLog("SwingTracking接收到的轨迹为空！！！");
		return false;
	}

	//(1) 分离坐标
	for (int i = 0; i < vtRobotCoors.size(); i++) {
		T_ROBOT_COORS tempRobotCoors = vtRobotCoors[i];
#ifdef SINGLE_ROBOT
		vdAxisRealCoors.push_back(vtRobotCoors[i].dBY);
		tempRobotCoors.dBY = 0;
		vtRobotRealCoors.push_back(tempRobotCoors);
#else

		vdAxisRealCoors.push_back(vtRobotCoors[i].dBX);
		tempRobotCoors.dBX = 0;
		vtRobotRealCoors.push_back(tempRobotCoors);
#endif		
	}
	//(2) 获取起点终点
	SaveSEPtn(vtRobotCoors[0], m_pTraceModel->m_tRealEndpointCoor);
	//(3) 分解速度
	double dRobotSpeed = 0;
	double dAxisSpeed = 0;
	DecomposeSpeed(m_swingTrackWeldParam.startPtn, m_swingTrackWeldParam.endPtn, dSpeed, dAxisSpeed, dRobotSpeed);
	//1. 计算速度 v=2π×频率×振幅 mm/s（暂定）
	double dSwingSpeed = (m_swingTrackWeldParam.dSwingRightAmplitude + m_swingTrackWeldParam.dSwingLeftAmplitude) * 2 * m_swingTrackWeldParam.dSwingFrequency * 60;
	//2. 转换单位 mm/min
	dSwingSpeed = (dSwingSpeed / 6);
	m_swingTrackWeldParam.dRobotWeldSpeed = dSwingSpeed;

	//(4)初始化部分焊接结构体参数
	m_swingTrackWeldParam.vtWorldCoors = vtRobotCoors; //合并未处理的焊道世界坐标
	m_swingTrackWeldParam.vtWeldRealCoors = vtRobotRealCoors; //合并机器人焊道
	m_swingTrackWeldParam.vdAxisCoors = vdAxisRealCoors; //合并外部轴焊道
														 //m_swingTrackWeldParam.dRobotWeldSpeed = dRobotSpeed; //合并机器人速度
	m_swingTrackWeldParam.dAxisWeldSpeed = dAxisSpeed / 60; //合并外部轴速度 （大车单抽运动 速度单位 mm/s 此处除60）
	m_swingTrackWeldParam.vtWeldPara;  //合并工艺参数
									   //WriteSwingParam();			//写入频率 左摆幅 右摆幅
									   //2. 启动控制线程
	clock_t testTime = clock();
	m_bSwingState = false;
	AfxBeginThread(ThreadMainContData, this);
	WriteLog("\n已经启动主控线程 时间:%d\n", clock() - testTime);
	return true;
}

void CDiaphragmWeld::SeparateAndSaveWeldingPath(std::vector<T_ROBOT_COORS> vtWorldCoors)
{
	//1. 定义单一元素
	T_ROBOT_COORS tRobotCoors; // 机器人分离坐标
	double dAxisCoors;  //外部轴坐标
	std::vector<T_ROBOT_COORS> vtRobotCoors; //分离出的机器人轨迹
	std::vector<double> vdAxisCoors; //分离出的外部轴轨迹

									 //2. 看运动方向分离坐标
#ifdef SINGLE_ROBOT
	for (int i = 0; i < vtWorldCoors.size(); i++) {
		dAxisCoors = vtWorldCoors[i].dY;
		vtWorldCoors[i].dY = 0;
		tRobotCoors = vtWorldCoors[i];
		vtRobotCoors.push_back(tRobotCoors);
		vdAxisCoors.push_back(dAxisCoors);
	}
#else
	for (int i = 0; i < vtWorldCoors.size(); i++) {
		dAxisCoors = vtWorldCoors[i].dBX;
		vtWorldCoors[i].dBX = 0;
		tRobotCoors = vtWorldCoors[i];
		vtRobotCoors.push_back(tRobotCoors);
		vdAxisCoors.push_back(dAxisCoors);
	}

#endif		
	//3. 保存分离坐标
	m_swingTrackWeldParam.vdAxisCoors.insert(m_swingTrackWeldParam.vdAxisCoors.end(), vdAxisCoors.begin(), vdAxisCoors.end());
	m_swingTrackWeldParam.vtWeldRealCoors.insert(m_swingTrackWeldParam.vtWeldRealCoors.end(), vtRobotCoors.begin(), vtRobotCoors.end());
}

void CDiaphragmWeld::SeparateAndSaveWeldingPath(T_ROBOT_COORS tWorldCoors)
{
	//1. 定义单一元素
	T_ROBOT_COORS tRobotCoors; // 机器人分离坐标
	double dAxisCoors;  //外部轴坐标
						//2. 看运动方向分离坐标
#ifdef SINGLE_ROBOT
	dAxisCoors = tWorldCoors.dY;
	tWorldCoors.dY = 0;
	tRobotCoors = tWorldCoors;
#else
	dAxisCoors = tWorldCoors.dBX;
	tWorldCoors.dX = 0;
	tRobotCoors = tWorldCoors;
#endif
	//3. 保存分离坐标
	m_swingTrackWeldParam.vdAxisCoors.push_back(dAxisCoors);
	m_swingTrackWeldParam.vtWeldRealCoors.push_back(tRobotCoors);
}

void CDiaphragmWeld::SaveSEPtn(T_ROBOT_COORS tStartPtn, T_ROBOT_COORS tEndPtn)
{
	m_swingTrackWeldParam.startPtn = tStartPtn;
	m_swingTrackWeldParam.endPtn = tEndPtn;
}

bool CDiaphragmWeld::SendWeldData()
{
	/*
	I51记录已走过点数

	I2-I10记录焊接信息\
	焊接速度
	起弧电流
	起弧电压
	焊接电流
	焊接电压
	收弧电流
	收弧电压
	*/
	CRobotDriverAdaptor* pRobotDriver = m_pRobotDriver;
	//高速发送准备
	MP_USR_VAR_INFO tUsrVarInfo;
	vector<MP_USR_VAR_INFO> vtUsrVarInfo;
	//准备速度
	tUsrVarInfo = pRobotDriver->PrepareValData(99, m_swingTrackWeldParam.dRobotWeldSpeed);
	vtUsrVarInfo.push_back(tUsrVarInfo);
	//准备焊接参数（先不写）

	//发送终点(给机器人一个开始的运动方向)
	//准备摆动坐标结构体
	WriteSwing(m_swingTrackWeldParam.vtWeldRealCoors[0]);

	tUsrVarInfo = pRobotDriver->PrepareValData(101, m_swingTrackWeldParam.swingTrackPoint.tLeftPoint);
	vtUsrVarInfo.push_back(tUsrVarInfo);
	tUsrVarInfo = pRobotDriver->PrepareValData(102, m_swingTrackWeldParam.swingTrackPoint.tMiddlePoint);
	vtUsrVarInfo.push_back(tUsrVarInfo);
	tUsrVarInfo = pRobotDriver->PrepareValData(103, m_swingTrackWeldParam.swingTrackPoint.tRightPoint);
	vtUsrVarInfo.push_back(tUsrVarInfo);
	pRobotDriver->SetMultiVar_H(vtUsrVarInfo);

	vector<T_WELD_PARA> vtWeldPara;
	T_WELD_PARA tWeldPara;
	E_WELD_SEAM_TYPE eWeldSeamType = E_FLAT_SEAM;
	if (false == GetCurWeldParam(eWeldSeamType, vtWeldPara))
	{
		XiMessageBox("跟踪：加载焊接工艺参数失败！");
		return false;
	}
	m_ptUnit->SendWeldParam(*m_pIsArcOn, vtWeldPara[0]);
	return true;
}

void CDiaphragmWeld::SendCoors()
{
	//获取外部轴坐标
	CRobotDriverAdaptor* pRobotDriver = m_pRobotDriver;
	double newPos = m_ptUnit->GetPositionDis();
	//寻找对应的机器人坐标的下标
	int index = 0;
	int nDir = 0;
	double dDis = m_swingTrackWeldParam.vdAxisCoors[m_swingTrackWeldParam.vdAxisCoors.size() - 1] - m_swingTrackWeldParam.vdAxisCoors[0];
	if (dDis > 0) {
		nDir = -1;
	}
	else {
		nDir = 1;
	}
	for (int i = 0; i < m_swingTrackWeldParam.vdAxisCoors.size(); i++) {
		if ((newPos*nDir) >(m_swingTrackWeldParam.vdAxisCoors[i] * nDir)) {
			index = i;
			break;
		}
	}
	//准备摆动坐标结构体
	WriteSwing(m_swingTrackWeldParam.vtWeldRealCoors[index]);

	//高速发送准备
	MP_USR_VAR_INFO tUsrVarInfo;
	vector<MP_USR_VAR_INFO> vtUsrVarInfo;
	//准备坐标
	tUsrVarInfo = pRobotDriver->PrepareValData(101, m_swingTrackWeldParam.swingTrackPoint.tLeftPoint);
	vtUsrVarInfo.push_back(tUsrVarInfo);
	tUsrVarInfo = pRobotDriver->PrepareValData(102, m_swingTrackWeldParam.swingTrackPoint.tMiddlePoint);
	vtUsrVarInfo.push_back(tUsrVarInfo);
	tUsrVarInfo = pRobotDriver->PrepareValData(103, m_swingTrackWeldParam.swingTrackPoint.tRightPoint);
	vtUsrVarInfo.push_back(tUsrVarInfo);
	m_pTraceModel->m_nWeldStepNo = index;//用于先测后焊记录步号
										 //pRobotDriver->m_nCurWeldStepNo = index; // 断点续焊使用

	WriteLog("正在发送第%d个坐标 发送的中间点坐标为X:%lf Y:%lf Z:%lf", index,
		m_swingTrackWeldParam.swingTrackPoint.tMiddlePoint.dX,
		m_swingTrackWeldParam.swingTrackPoint.tMiddlePoint.dY,
		m_swingTrackWeldParam.swingTrackPoint.tMiddlePoint.dZ);
	//准备机器人速度
	//tUsrVarInfo = pRobotDriver->PrepareValData(2, m_swingTrackWeldParam.dRobotWeldSpeed);
	//vtUsrVarInfo.push_back(tUsrVarInfo);
	pRobotDriver->SetMultiVar_H(vtUsrVarInfo);

	m_swingTrackWeldParam.dRobotWeldSpeed = 0;
}

void CDiaphragmWeld::WriteSwing(T_ROBOT_COORS tRobotCoors)
{
	//1. 计算速度 v=2π×频率×振幅 mm/s（暂定）
	double dSwingSpeed = (m_swingTrackWeldParam.dSwingRightAmplitude + m_swingTrackWeldParam.dSwingLeftAmplitude) * m_swingTrackWeldParam.dSwingFrequency * 60;
	//2. 转换单位 mm/min
	dSwingSpeed = (dSwingSpeed * 60 / 6);
	//3. 计算各个分量需要加减的值
	//左点
	double dXLeftDis = m_swingTrackWeldParam.dSwingLeftAmplitude * m_swingTrackWeldParam.norm[0];
	double dYLeftDis = m_swingTrackWeldParam.dSwingLeftAmplitude * m_swingTrackWeldParam.norm[1];
	double dZLeftDis = m_swingTrackWeldParam.dSwingLeftAmplitude * m_swingTrackWeldParam.norm[2];
	//右点
	double dXRightDis = m_swingTrackWeldParam.dSwingRightAmplitude * m_swingTrackWeldParam.norm[0];
	double dYRightDis = m_swingTrackWeldParam.dSwingRightAmplitude * m_swingTrackWeldParam.norm[1];
	double dZRightDis = m_swingTrackWeldParam.dSwingRightAmplitude * m_swingTrackWeldParam.norm[2];
	//4. 增加偏移量
	//COPini opini;
	//opini.SetFileName("./Data/WeldPoint");
	//double FlatWeldHorComp0 = 0;
	//double FlatWeldHeightComp0 = 0;
	//double FlatWeldHorComp90 = 0;
	//double FlatWeldHeightComp90 = 0;
	//double FlatWeldHorComp180 = 0;
	//double FlatWeldHeightComp180 = 0;
	//double FlatWeldHorComp270 = 0;
	//double FlatWeldHeightComp270 = 0;


	//opini.SetSectionName("WeldCompVa0");
	//opini.ReadString("FlatWeldHorComp", &FlatWeldHorComp0);
	//opini.ReadString("FlatWeldHeightComp", &FlatWeldHeightComp0);
	//opini.SetSectionName("WeldCompVa90");
	//opini.ReadString("FlatWeldHorComp", &FlatWeldHorComp90);
	//opini.ReadString("FlatWeldHeightComp", &FlatWeldHeightComp90);
	//opini.SetSectionName("WeldCompVa180");
	//opini.ReadString("FlatWeldHorComp", &FlatWeldHorComp180);
	//opini.ReadString("FlatWeldHeightComp", &FlatWeldHeightComp180);
	//opini.SetSectionName("WeldCompVa270");
	//opini.ReadString("FlatWeldHorComp", &FlatWeldHorComp0);
	//opini.ReadString("FlatWeldHeightComp", &FlatWeldHeightComp270);


	//4. 计算摆动运动结构体
	T_ROBOT_COORS tSwingLeftPtn;
	tSwingLeftPtn.dX = tRobotCoors.dX - dXLeftDis;
	tSwingLeftPtn.dY = tRobotCoors.dY - dYLeftDis;
	tSwingLeftPtn.dZ = tRobotCoors.dZ + dZLeftDis;
	tSwingLeftPtn.dRX = tRobotCoors.dRX;
	tSwingLeftPtn.dRY = tRobotCoors.dRY;
	tSwingLeftPtn.dRZ = tRobotCoors.dRZ;

	T_ROBOT_COORS tSwingMiddlePtn;
	tSwingMiddlePtn = tRobotCoors;

	T_ROBOT_COORS tSwingRightPtn;
	tSwingRightPtn.dX = tRobotCoors.dX + dXRightDis;
	tSwingRightPtn.dY = tRobotCoors.dY + dYRightDis;
	tSwingRightPtn.dZ = tRobotCoors.dZ - dZRightDis;
	tSwingRightPtn.dRX = tRobotCoors.dRX;
	tSwingRightPtn.dRY = tRobotCoors.dRY;
	tSwingRightPtn.dRZ = tRobotCoors.dRZ;

	m_swingTrackWeldParam.swingTrackPoint.tLeftPoint = tSwingLeftPtn;
	m_swingTrackWeldParam.swingTrackPoint.tMiddlePoint = tSwingMiddlePtn;
	m_swingTrackWeldParam.swingTrackPoint.tRightPoint = tSwingRightPtn;
	//5. 计算速度 

	//m_swingTrackWeldParam.dRobotWeldSpeed = std::sqrt(m_swingTrackWeldParam.dRobotWeldSpeed * m_swingTrackWeldParam.dRobotWeldSpeed + dSwingSpeed * dSwingSpeed);
	m_swingTrackWeldParam.dRobotWeldSpeed = dSwingSpeed;
}

void CDiaphragmWeld::DecomposeSpeed(T_ROBOT_COORS tStartPytn, T_ROBOT_COORS tEnd, double nSpeed, double& dMainSpeed, double& dSecSpeed)
{
	//计算向量准备
	cv::Vec6d twoPoint(tStartPytn.dX, tStartPytn.dY, tStartPytn.dZ, tEnd.dX, tEnd.dY, tEnd.dZ);
#ifdef SINGLE_ROBOT
	cv::Vec3d norm(

		twoPoint[1] - twoPoint[4],
		twoPoint[0] - twoPoint[3],
		twoPoint[2] - twoPoint[5]
		);
#else
	cv::Vec3d norm(

		twoPoint[0] - twoPoint[3],
		twoPoint[1] - twoPoint[4],
		twoPoint[2] - twoPoint[5]
	);
#endif
	if (nSpeed == 0) {
		XiMessageBox("输入的速度为0， 请检查输入速度！！！");
		return;
	}
	//保存所计算出的向量
	double dTempDot = std::sqrt(norm[0] * norm[0] + norm[1] * norm[1] + norm[2] * norm[2]);
	m_swingTrackWeldParam.norm = norm;
	m_swingTrackWeldParam.norm[0] = fabs(m_swingTrackWeldParam.norm[0] / dTempDot);
	m_swingTrackWeldParam.norm[1] = fabs(m_swingTrackWeldParam.norm[1] / dTempDot);
	m_swingTrackWeldParam.norm[2] = fabs(m_swingTrackWeldParam.norm[2] / dTempDot);

	//计算速度的实际值
	double dot = std::sqrt(norm[0] * norm[0] + norm[1] * norm[1] + norm[2] * norm[2]) / nSpeed;
	norm[0] = fabs(norm[0] / dot);
	norm[1] = fabs(norm[1] / dot);
	norm[2] = fabs(norm[2] / dot);


	if (norm[0] < norm[1]) {
		dMainSpeed = norm[1];
		dSecSpeed = std::sqrt(norm[0] * norm[0] + norm[2] * norm[2]);
	}
	else {
		dMainSpeed = norm[0];
		dSecSpeed = std::sqrt(norm[1] * norm[1] + norm[2] * norm[2]);
	}
	return;
}

bool CDiaphragmWeld::isAxisToEnd()
{
	CRobotDriverAdaptor* pRobotDriver = m_pRobotDriver;
	double newPos = m_ptUnit->GetPositionDis();
	double dStart = m_swingTrackWeldParam.vdAxisCoors[0];
	double dEnd = m_swingTrackWeldParam.vdAxisCoors[m_swingTrackWeldParam.vdAxisCoors.size() - 1];
	double dWeldDir = dEnd > dStart ? 1.0 : -1.0;
	if ((newPos * dWeldDir) >= (dEnd * dWeldDir)) {
		return true;
	}
	return false;
}

void CDiaphragmWeld::UpdateRealRobotCoors()
{
	// 记录 m_btRealRobotCoors 的大小
	m_swingTrackWeldParam.nFlat = m_swingTrackWeldParam.vtWorldCoors.size();
	WriteLog("结构体中世界坐标大小：%d    总世界坐标大小:%d", m_swingTrackWeldParam.vtWorldCoors.size(), m_pRobotDriver->m_vtWeldLineInWorldPoints.size());

	// 判断 m_vtweldlneTnWor1dPoints 的大小是否大于 nFalt
	if (m_pRobotDriver->m_vtWeldLineInWorldPoints.size() > m_swingTrackWeldParam.nFlat) {
		// 获取迭代器指向的位置
		auto it = m_pRobotDriver->m_vtWeldLineInWorldPoints.begin() + m_swingTrackWeldParam.nFlat;

		// 将大于的部分赋值 给 m_btRealRobotCoors
		m_swingTrackWeldParam.vtWorldCoors.insert(m_swingTrackWeldParam.vtWorldCoors.end(), it, m_pRobotDriver->m_vtWeldLineInWorldPoints.end());

		//分解新添加的世界坐标
		std::vector<T_ROBOT_COORS> temp(m_swingTrackWeldParam.vtWorldCoors.begin() + m_swingTrackWeldParam.nFlat, m_swingTrackWeldParam.vtWorldCoors.end());

		for (int i = 0; i < temp.size(); i++) {
			double dAxis = temp[i].dBY;
			temp[i].dBY = 0;
			T_ROBOT_COORS tRobotCoors = temp[i];

			m_swingTrackWeldParam.vtWeldRealCoors.push_back(tRobotCoors);
			m_swingTrackWeldParam.vdAxisCoors.push_back(dAxis);
		}
		WriteLog("整合完：结构体中世界坐标大小：%d    总世界坐标大小:%d", m_swingTrackWeldParam.vtWorldCoors.size(), m_pRobotDriver->m_vtWeldLineInWorldPoints.size());
	}


}

UINT CDiaphragmWeld::ThreadCheckSwingConsistency(void* pParam)
{
	CDiaphragmWeld* pDiaphragmWeld = (CDiaphragmWeld*)pParam;
	CRobotDriverAdaptor* pRobotDriver = pDiaphragmWeld->m_pRobotDriver;
	while (!pDiaphragmWeld->m_bSwingState) {
		pDiaphragmWeld->UpdateRealRobotCoors();
		Sleep(500);
	}
	return 0;
}

void CDiaphragmWeld::WriteSwingParam(double dSwingFrequency, double dSwingLeftAmplitude, double dSwingRightAmplitude)
{
	m_swingTrackWeldParam.dSwingFrequency = dSwingFrequency;
	m_swingTrackWeldParam.dSwingLeftAmplitude = dSwingLeftAmplitude;
	m_swingTrackWeldParam.dSwingRightAmplitude = dSwingRightAmplitude;
}

UINT CDiaphragmWeld::ThreadMainContData(void* pParam)
{
	CDiaphragmWeld* pDiaphragmWeld = (CDiaphragmWeld*)pParam;
	CRobotDriverAdaptor* pRobotDriver = pDiaphragmWeld->m_pRobotDriver;
	pRobotDriver->SetIntVar(12, 0);
	//记录时间
	clock_t startTime = clock();

	//1. 发送焊接信息
	if (false == pDiaphragmWeld->SendWeldData())
	{
		XiMessageBox("焊接参数设置失败！");
		return 0;
	}

	//2. 开启外部轴
	CUnitDriver* pUnitDriver = pRobotDriver->m_pvpMotorDriver->at(0);/*pDiaphragmWeld->m_vpUnit[0]->GetRobotCtrl()->m_pvpMotorDriver->at(0)*/;
	double dDis = pDiaphragmWeld->m_swingTrackWeldParam.vdAxisCoors[pDiaphragmWeld->m_swingTrackWeldParam.vdAxisCoors.size() - 1] - pDiaphragmWeld->m_swingTrackWeldParam.vdAxisCoors[0];
	int nDir = dDis > 0.0 ? 1 : 0;
	//pDiaphragmWeld->pUnitDriver->ContiMove(nDir, pDiaphragmWeld->m_swingTrackWeldParam.dAxisWeldSpeed, 0.5);
	pUnitDriver->ContiMove(nDir, pDiaphragmWeld->m_swingTrackWeldParam.dAxisWeldSpeed, 0.5);

	//3. 开启机器人
	pRobotDriver->CallJob("SWINGTRACKING");
	int nMaxWaitTime = 3000;
	int nCurWaitTime = 0;
	while (((/*TRUE != pDiaphragmWeld->m_ptUnit->CheckInIO("Running")*/TRUE != pDiaphragmWeld->m_ptUnit->WorldIsRunning()) /*|| (pDiaphragmWeld->m_ptUnit->CheckRobotDone)*//*||*/ /*(0 != pRobotDriver->m_pMoveCtrl->CheckDone(pRobotDriver->m_nAXIS_X))*/) &&
		(nCurWaitTime < nMaxWaitTime))
	{
		nCurWaitTime += 50;
		Sleep(50);
	}

	WriteLog("\n机器人线程开始成功 时间:%d\n", clock() - pDiaphragmWeld->testTime);
	//4. 开启实施更新线程
	CWinThread* pUpdataThread = AfxBeginThread(ThreadCheckSwingConsistency, pDiaphragmWeld);
	while (1) {
		//如果没有达到终点 则发送下一个机器人坐标
		if (!pDiaphragmWeld->isAxisToEnd() &&
			(/*TRUE == pDiaphragmWeld->m_ptUnit->CheckInIO("Running")*/TRUE == pDiaphragmWeld->m_ptUnit->WorldIsRunning()) /*&&
																														   (0 == pRobotDriver->m_pMoveCtrl->CheckDone(pRobotDriver->m_nAXIS_X))*/) {
			pDiaphragmWeld->SendCoors();
		}
		else {
			pDiaphragmWeld->m_pTraceModel->m_bCameraFindWeldEnd = true; // 退出跟踪线程
			pDiaphragmWeld->m_cMoveCtrl.DecelStop(pDiaphragmWeld->m_pTraceModel->m_nAXIS_X/*m_ptUnit->m_nTrackAxisNo*/, 0.5, 0);
			pRobotDriver->SetIntVar(12, 1);
			pRobotDriver->HoldOn();
			Sleep(200);
			pRobotDriver->HoldOff();
			WriteLog("ThreadMainContData Exit!");
			break;
		}
		Sleep(200);
	}
	clock_t endTime = clock() - startTime;
	//5. 记录数据(外部轴移动的距离为焊道长度)
	dDis = pDiaphragmWeld->m_swingTrackWeldParam.vdAxisCoors[pDiaphragmWeld->m_swingTrackWeldParam.vdAxisCoors.size() - 1] - pDiaphragmWeld->m_swingTrackWeldParam.vdAxisCoors[0];
	/*CStatisticalData* pStatisticalData = CStatisticalData::getInstance();
	dDis = dDis / 1000;
	if (dDis > 0) {
	pStatisticalData->UpdateWeldData(E_FLAT_SEAM, dDis, endTime);
	}
	else {
	pStatisticalData->UpdateWeldData(E_FLAT_SEAM, -dDis, endTime);
	}

	pStatisticalData->UpDateStatisticsData();*/

	//6. 清枪
	/*if (false == pRobotDriver->CleanGunH())
	{
	XiMessageBox("清枪失败");
	}
	else
	{
	XiMessageBox("清枪完成");
	}	*/
	pDiaphragmWeld->m_bSwingState = true;
	return 0;
}




T_CORNER_LINE_INFO_NEW CDiaphragmWeld::FindCornerLineInfoFun(int nBoardNo, VT_CORNER_LINE_INFO_NEW* pvtCornerLineInfo)
{
	T_CORNER_LINE_INFO_NEW tCornerLine;
	//遍历端点表
	if (pvtCornerLineInfo == NULL)
	{
		WriteLog("焊缝总表为空，请检查数据是否有误");
		XiMessageBox("焊缝总表为空，请检查数据是否有误");
	}
	int nCor = 0;
	for (nCor = 0; nCor < pvtCornerLineInfo->size(); nCor++)
	{
		if (pvtCornerLineInfo->at(nCor).nBoardNo == nBoardNo)
		{
			WriteLog("第：%d条角缝和立缝：%d 关联", nCor, nBoardNo);
			tCornerLine = pvtCornerLineInfo->at(nCor);
			return tCornerLine;
		}
	}

	if (nCor == pvtCornerLineInfo->size())
	{
		AfxMessageBox("角缝%d找不到对应的端点索引,请检查BoardInfo.txt端点表", nBoardNo);
	}
	return T_CORNER_LINE_INFO_NEW();
}

T_ENDPOINT_INFO_NEW CDiaphragmWeld::FindEndPointInfoFun(int nEndPointNo, VT_ENDPOINT_INFO_NEW* pvtPointInfo)
{
	T_ENDPOINT_INFO_NEW tEndPoint;
	//遍历端点表
	if (pvtPointInfo == NULL)
	{
		WriteLog("角缝端点表为空，请检查数据是否有误");
		XiMessageBox("角缝端点表为空，请检查数据是否有误");
	}
	int nEp = 0;
	for (nEp = 0; nEp < pvtPointInfo->size(); nEp++)
	{
		if (pvtPointInfo->at(nEp).nEndPointNo == nEndPointNo)
		{
			WriteLog("第:%d条角缝端点：%d ", nEp, nEndPointNo);
			tEndPoint = pvtPointInfo->at(nEp);
			return tEndPoint;
		}
	}
	if (nEp == pvtPointInfo->size())
	{
		WriteLog("端点%d找不到对应的端点索引,请检查BoardInfo.txt端点表", nEndPointNo);
	}
	return T_ENDPOINT_INFO_NEW();
}

bool CDiaphragmWeld::FindRelevanceVerticalFun(T_VERTICAL_INFO_NEW tVerticalInfo, VT_VERTICAL_INFO_NEW* ptVerticalInfo, VT_ENDPOINT_INFO_NEW* pvtPointInfo, VT_CORNER_LINE_INFO_NEW* pvtCornerLineInfo, T_VERTICAL_INFO_NEW& tVertical)
{
	//T_VERTICAL_INFO_NEW tVertical = {-1};

	T_ENDPOINT_INFO_NEW tEndPoint1 = FindEndPointInfoFun(tVerticalInfo.nStartNo, pvtPointInfo);
	T_CORNER_LINE_INFO_NEW tCornerLine, tCornerLineReleve;
	tCornerLine = FindCornerLineInfoFun(tVerticalInfo.nCornerLineNo, pvtCornerLineInfo);
	tCornerLineReleve = FindCornerLineInfoFun(tCornerLine.bStartContiguousCor, pvtCornerLineInfo);
	int nEp = 0;
	for (nEp = 0; nEp < ptVerticalInfo->size(); nEp++)
	{
		if (ptVerticalInfo->at(nEp).nBoardNo == tVerticalInfo.nBoardNo)
		{
			continue;
		}
		T_ENDPOINT_INFO_NEW tEndPoint2 = FindEndPointInfoFun(ptVerticalInfo->at(nEp).nStartNo, pvtPointInfo);
		T_CORNER_LINE_INFO_NEW tCornerLine1, tCornerLineReleve1;
		tCornerLine1 = FindCornerLineInfoFun(ptVerticalInfo->at(nEp).nCornerLineNo, pvtCornerLineInfo);
		tCornerLineReleve1 = FindCornerLineInfoFun(tCornerLine1.bStartContiguousCor, pvtCornerLineInfo);

		double dAngleOffset = fmod((tCornerLineReleve.dEndAngle - tCornerLine1.dStartAngle), 360);
		double dAngleOffset2 = fmod((tCornerLine.dEndAngle - tCornerLineReleve1.dStartAngle), 360);

		dAngleOffset = dAngleOffset > 180.0 ? dAngleOffset - 360.0 : dAngleOffset;
		dAngleOffset = dAngleOffset < -180.0 ? dAngleOffset + 360.0 : dAngleOffset;

		dAngleOffset2 = dAngleOffset2 > 180.0 ? dAngleOffset2 - 360.0 : dAngleOffset2;
		dAngleOffset2 = dAngleOffset2 < -180.0 ? dAngleOffset2 + 360.0 : dAngleOffset2;
		bool bTemp = false;
		if (dAngleOffset < 3 || dAngleOffset2 < 3)
		{
			bTemp = true;
		}

		double dis = TwoPointDis(tEndPoint1.tPointData.x, tEndPoint1.tPointData.y, tEndPoint1.tPointData.z,
			tEndPoint2.tPointData.x, tEndPoint2.tPointData.y, tEndPoint2.tPointData.z);
		bool bRst = LoadWeldStatus(ptVerticalInfo->at(nEp).nBoardNo);

		if (dis < 35 && bTemp && !bRst && !ptVerticalInfo->at(nEp).bIfWeldCom)
		{
			WriteLog("第:%d条立缝,和：%d 立缝 相近 ", nEp, tVerticalInfo.nBoardNo);
			tVertical = ptVerticalInfo->at(nEp);
			return true;
		}
	}
	if (nEp == ptVerticalInfo->size())
	{
		WriteLog("端点 %d 找不到相近的端点索引", tVerticalInfo.nBoardNo);
	}
	return false;
}

T_CIRCLE_CENTER_INFO_NEW CDiaphragmWeld::FindCenterPointInfoFun(int nEndPointNo, VT_CIRCLE_CENTER_INFO_NEW* pvtPointInfo)
{
	T_CIRCLE_CENTER_INFO_NEW tCenterPoint;
	//遍历端点表
	if (pvtPointInfo == NULL)
	{
		WriteLog("圆心点表为空，请检查数据是否有误");
		XiMessageBox("圆心端点表为空，请检查数据是否有误");
	}
	int nEp = 0;
	for (nEp = 0; nEp < pvtPointInfo->size(); nEp++)
	{
		if (pvtPointInfo->at(nEp).nBoardNo == nEndPointNo)
		{
			WriteLog("第:%d条角缝端点：%d ", nEp, nEndPointNo);
			tCenterPoint = pvtPointInfo->at(nEp);
			return tCenterPoint;
		}
	}
	if (nEp == pvtPointInfo->size())
	{
		AfxMessageBox("中心点%d找不到对应的端点索引,请检查BoardInfo.txt端点表", nEndPointNo);
	}
	return T_CIRCLE_CENTER_INFO_NEW();
}

bool CDiaphragmWeld::CorrectCenterPointInfoFun(int nEndPointNo, XI_POINT tCenTerNew, VT_CIRCLE_CENTER_INFO_NEW* pvtPointInfo)
{
	//遍历端点表
	if (pvtPointInfo == NULL)
	{
		WriteLog("圆心点表为空，请检查数据是否有误");
		XiMessageBox("圆心端点表为空，请检查数据是否有误");
	}
	int nEp = 0;
	for (nEp = 0; nEp < pvtPointInfo->size(); nEp++)
	{
		if (pvtPointInfo->at(nEp).nBoardNo == nEndPointNo)
		{
			WriteLog("第:%d条角缝端点：%d ", nEp, nEndPointNo);
			pvtPointInfo->at(nEp).dCenterX = tCenTerNew.x;
			pvtPointInfo->at(nEp).dCenterY = tCenTerNew.y;
			pvtPointInfo->at(nEp).dCenterZ = tCenTerNew.z;
			return true;
		}
	}
	if (nEp == pvtPointInfo->size())
	{
		AfxMessageBox("中心点%d找不到对应的端点索引,请检查BoardInfo.txt端点表", nEndPointNo);
		return false;
	}
	return false;
}

vector<XI_POINT> CDiaphragmWeld::GetMeasurePointFun(int nBoardNo, VT_MEASURE_INFO_NEW* pvMeasureInfo)
{
	vector<XI_POINT> VMeasurePoint;
	int nMeasureNo = 0;
	for (nMeasureNo = 0; nMeasureNo < pvMeasureInfo->size(); nMeasureNo++)
	{
		if (pvMeasureInfo->at(nMeasureNo).nBoardNo == nBoardNo)
		{
			VMeasurePoint = pvMeasureInfo->at(nMeasureNo).VMeasurePOINT;
			for (int n = 0; n < VMeasurePoint.size(); n++)
			{
				WriteLog("测量点：%lf %lf %lf", VMeasurePoint[n].x, VMeasurePoint[n].y, VMeasurePoint[n].z);
			}

			break;
		}
	}
	return VMeasurePoint;
}

T_VERTICAL_INFO_NEW CDiaphragmWeld::FindVercalInfoFun(int nBoardNo, VT_VERTICAL_INFO_NEW* ptVerticalInfo)
{
	T_VERTICAL_INFO_NEW tVerticalInfo;
	//遍历立焊表
	if (ptVerticalInfo == NULL)
	{
		WriteLog("立焊缝表为空，请检查数据是否有误");
		XiMessageBox("立焊缝表为空，请检查数据是否有误");
	}
	int nCor = 0;
	for (nCor = 0; nCor < ptVerticalInfo->size(); nCor++)
	{
		if (ptVerticalInfo->at(nCor).nBoardNo == nBoardNo)
		{
			WriteLog("第：%d条角缝和立缝：%d 关联,当前立缝是否焊接：%d", nBoardNo, nCor, ptVerticalInfo->at(nCor).bIfWeldCom);
			tVerticalInfo = ptVerticalInfo->at(nCor);
			return tVerticalInfo;
		}
	}
	if (nCor == ptVerticalInfo->size())
	{
		AfxMessageBox("角缝%d找不到对应的立缝索引,请检查BoardInfo.txt端点表", nBoardNo);
	}
	return T_VERTICAL_INFO_NEW();
}

bool CDiaphragmWeld::CheckSafeFun(CRobotDriverAdaptor* pRobotDriver)
{
	bool bDir = m_nRobotInstallDir == -1 ? false : true;
	T_ROBOT_COORS tCurrentPos = GetCurrentPos(pRobotDriver);
	double dMaxPos = 1400.0, dMinPos = 1600.0;
	if (!bDir)
	{
		dMaxPos = 1600.0;
		dMinPos = 1400.0;
	}
	else
	{
		dMaxPos = 100.0;
		dMinPos = -300.0;
	}

	if (tCurrentPos.dZ < dMinPos && ! g_bLocalDebugMark)
	{
		pRobotDriver->PosMove(ROBOT_AXIS_Z, dMinPos, 500, COORD_ABS);
		m_ptUnit->RobotCheckDone();
		if (!m_ptUnit->m_bBreakPointContinue) {
			/*if (!pRobotDriver->BackSafePosError(1000)) {
				return false;
			}*/
		}
	}
	if (tCurrentPos.dZ > dMaxPos && !g_bLocalDebugMark)//1600
	{
		pRobotDriver->PosMove(ROBOT_AXIS_Z, dMaxPos, 500, COORD_ABS);//1600
		m_ptUnit->RobotCheckDone();
		if (!m_ptUnit->m_bBreakPointContinue) {
			/*if (!pRobotDriver->BackSafePosError(1000)) {
				return false;
			}*/
		}
	}
	return true;
}

//bool CDiaphragmWeld::DiaphragmWeld()
//{
//	AfxBeginThread(ThreadDiaphragmWeldProject, this);
//	return true;
//}

//UINT CDiaphragmWeld::ThreadDiaphragmWeldProject(void* pProjec)
//{
//	CDiaphragmWeld* pMyObj = (CDiaphragmWeld*)pProjec;
//	if (pMyObj->m_bWorking)
//	{
//		XiMessageBox("隔板调度函数已在运行");
//		return false;
//	}
//	pMyObj->m_bWorking = true;
//	pMyObj->DiaphragmWeldProject(pMyObj->m_pRobotDriver);
//	pMyObj->m_bWorking = false;
//	return true;
//}

//bool CDiaphragmWeld::DiaphragmWeldProject(CRobotDriverAdaptor* pRobotDriver)
//{
//	//加载焊接数据
//	m_dUltraTransitionSpeed = 10000;	//满速，焊接过程过渡点尽可能避免冷接头
//	m_dSafePosRunSpeed = 3500;			//安全位置移动速度，快速
//	m_dFastApproachToWorkSpeed = 3000;	//快速靠近工件， 中速
//	m_dSafeApproachToWorkSpeed = 2500;	//安全移动到工件速度，慢速
//
//	//安全检查函数
//	CheckSafeFun(pRobotDriver);
//	// 加载数据
//	LoadWorkPieceInfo();
//	//总表
//	VT_CORNER_LINE_INFO_NEW* pvtCnr;
//	//端点
//	VT_ENDPOINT_INFO_NEW* pvtEi;
//	//测量点
//	VT_MEASURE_INFO_NEW* pvtMi;
//	//立焊
//	VT_VERTICAL_INFO_NEW* pvtVi;
//	//圆心
//	VT_CIRCLE_CENTER_INFO_NEW* pvCenter;
//	pvtCnr = NULL;
//	pvtEi = NULL;
//	pvtMi = NULL;
//	pvtVi = NULL;
//	pvCenter = NULL;
//	int nWorkpieceNo = 0;
//
//	if (m_ptUnit->m_bBreakPointContinue)
//	{
//		LoadWorkpieceNo(pRobotDriver, nWorkpieceNo);
//	}
//
//	int nVerSize = 0;
//	int nWorkpieceNum = m_vvtCornerLineInfoNew.size();
//	WriteLog("nWorkpieceNum:%d", m_vvtCornerLineInfoNew.size());
//	int nWorkpieceIdx = 0;
//	if (m_vvtCornerLineInfoNew.size())
//	{
//		pvtCnr = &m_vvtCornerLineInfoNew[nWorkpieceIdx];//总表，角缝表	
//	}
//	if (m_vvtPointInfoNew.size())
//	{
//		pvtEi = &m_vvtPointInfoNew[nWorkpieceIdx];//端点表
//	}
//	if (m_vvMeasureInfoNew.size())
//	{
//		pvtMi = &m_vvMeasureInfoNew[nWorkpieceIdx];//测量点
//	}
//	if (m_vvtCircleCenterInfo.size() > 0)
//	{
//		pvCenter = &m_vvtCircleCenterInfo[nWorkpieceIdx];
//	}
//
//	//先焊接立焊
//	int nVerticalNum = m_vvtVerticalInfoNew.size();
//	if (nVerticalNum > 0)
//	{
//		//根据立焊所在角缝位置查找需要的信息，焊接立缝时需要信息：关联角缝，测量点，端点
//		pvtVi = &m_vvtVerticalInfoNew[nWorkpieceIdx];
//
//		if (nWorkpieceNo < pvtVi->size())
//		{
//			for (int nVi = nWorkpieceNo; nVi < pvtVi->size(); nVi++)
//			{
//
//				if (LoadWeldStatus(pvtVi->at(nVi).nBoardNo) || pvtVi->at(nVi).bIfWeldCom)
//				{
//					pRobotDriver->m_cLog->Write(" 第 %d 条立峰已完成焊接，或跳过焊接", pRobotDriver, pvtVi->at(nVi).nBoardNo);
//					continue;
//				}
//				if (IDOK != XiMessageBoxGroup(*m_pIsNaturalPop, "是否开始焊接：%d 条立缝", pvtVi->at(nVi).nBoardNo))
//				{
//					continue;
//				}
//				if (false == VerticalWeldFunNew(pRobotDriver, pvtVi->at(nVi), pvtVi, pvtCnr, pvtEi, pvtMi))
//				{
//					return false;
//				}
//			}
//		}
//	}
//	else
//	{
//		WriteLog("不存在立焊信息");
//	}
//	//角缝焊接
//	int nCornerNum = m_vvtCornerLineInfoNew.size();
//	if (nCornerNum > 0)
//	{
//		WriteLog("pvtCnr->size():%d", pvtCnr->size());
//		if (nVerticalNum > 0 && m_ptUnit->m_bBreakPointContinue)
//		{
//			nWorkpieceNo -= pvtVi->size();
//		}
//		if (nWorkpieceNo < pvtCnr->size())
//		{
//			for (int nCor = nWorkpieceNo; nCor < pvtCnr->size(); nCor++)
//			{
//				if (((LoadWeldStatus(pvtCnr->at(nCor).nBoardNo) || pvtCnr->at(nCor).bIfWeldCom)))
//				{	// 判断是否焊接完成
//					pRobotDriver->m_cLog->Write(" 第 %d 条角缝已完成焊接，或跳过焊接", pRobotDriver, pvtCnr->at(nCor).nBoardNo);
//					continue;
//				}
//				if (IDOK != XiMessageBoxGroup(*m_pIsNaturalPop, "是否开始焊接：%d 条角缝", pvtCnr->at(nCor).nBoardNo))
//				{
//					continue;
//				}
//				//判断是否为闭合单一圆弧
//				double dRadius = 0.0;
//				if (SelectLinesAndCircle(pRobotDriver, pvtCnr->at(nCor), pvtEi, pvCenter, dRadius))//直线
//				{
//					//角封焊接需要先中间后两边单条焊接
//					//焊接角缝需要知道的信息有理论的起点、终点，线起点的法向、
//					m_eWorkPieceType = E_LINELLAE;
//					if (false == CornerLineWeldFunNew(pRobotDriver, pvtCnr->at(nCor), pvtCnr, pvtEi, pvtMi, pvCenter))
//					{
//						break;
//					}
//				}
//				else//闭合圆弧,单一加强圈隔板,
//				{
//					m_eWorkPieceType = E_CLOSEDARC;
//					if (dRadius > 200)
//					{
//						if (false == StrengthRingWeldFunNew(pRobotDriver, pvtCnr->at(nCor), pvtCnr, pvtEi))
//						{
//							break;
//						}
//					}
//					else if (dRadius <= 200 && dRadius > 50)
//					{
//						//if (false == CircleWeldFunNew(pRobotDriver, pvtCnr->at(nCor), pvtEi, pvCenter, pvtMi))
//						{
//							//break;
//						}
//					}
//					else
//					{
//						XiMessageBox("工件超出设备焊接范围 工件半径：%lf，点击确定继续焊接", dRadius);
//						continue;
//					}
//				}
//			}
//		}
//	}
//	XiMessageBoxPopup("所有工件以焊接完成", *m_pIsNaturalPop);
//	return true;
//}

void CDiaphragmWeld::LoadWeldLineData(E_WELD_LINE& vcPointOnTheSameLine, CString str)
{
	vcPointOnTheSameLine.vdNormal.clear();
	vcPointOnTheSameLine.vtPoint.clear();
	double dNormal;
	XI_POINT tPoint;
	int nNo;
	ifstream CoutData(str.GetBuffer(0));
	while (!CoutData.eof())
	{
		CoutData >> nNo >> tPoint.x >> tPoint.y >> tPoint.z >> dNormal;
		vcPointOnTheSameLine.vdNormal.push_back(dNormal);
		vcPointOnTheSameLine.vtPoint.push_back(tPoint);
	}
	CoutData.close();
}

void CDiaphragmWeld::GetCircleRealWeldPoint(CRobotDriverAdaptor* pRobotDriver, vector<T_ROBOT_COORS>& realWeldPoint, E_WELD_LINE vcPointOnTheSameLine, double dMachineCarMovePos)
{
	XiAlgorithm alg;
	T_ROBOT_COORS midWledPoint;
	double Offset = m_pTraceModel->m_dGunToEyeCompenX;
	double zOffset = m_pTraceModel->m_dGunToEyeCompenZ;

	for (int i = 0; i < vcPointOnTheSameLine.vtPoint.size(); i += 3)
	{
		midWledPoint.dX = vcPointOnTheSameLine.vtPoint[i].x + Offset * CosD(vcPointOnTheSameLine.vdNormal[i]);
		midWledPoint.dY = vcPointOnTheSameLine.vtPoint[i].y + Offset * SinD(vcPointOnTheSameLine.vdNormal[i]);
		midWledPoint.dZ = vcPointOnTheSameLine.vtPoint[i].z + zOffset;
		midWledPoint.dRX = m_dPlatWeldRx/*0*/;
		midWledPoint.dRY = m_dPlatWeldRy/*-45*/;
		midWledPoint.dRZ = vcPointOnTheSameLine.vdNormal[i];
		midWledPoint.dBX = 0;
		midWledPoint.dBY = dMachineCarMovePos;
		midWledPoint.dBZ = 0;

		realWeldPoint.push_back(midWledPoint);
	}

	//多走3mm，做搭接
	//if(realWeldPoint.at())
	realWeldPoint.push_back(realWeldPoint.at(0));
	realWeldPoint.push_back(realWeldPoint.at(1));



	FILE* fp = fopen("WeldData\\WeldRobotLeft\\先测后焊焊接轨迹.txt", "w");
	for (int nRealWeldPoint = 0; nRealWeldPoint < realWeldPoint.size(); nRealWeldPoint++)
	{
		fprintf(fp, "%.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf\n",
			realWeldPoint.at(nRealWeldPoint).dX, realWeldPoint.at(nRealWeldPoint).dY, realWeldPoint.at(nRealWeldPoint).dZ,
			realWeldPoint.at(nRealWeldPoint).dRX, realWeldPoint.at(nRealWeldPoint).dRY, realWeldPoint.at(nRealWeldPoint).dRZ,
			realWeldPoint.at(nRealWeldPoint).dBX, realWeldPoint.at(nRealWeldPoint).dBY, realWeldPoint.at(nRealWeldPoint).dBZ);
	}
	fclose(fp);
}

/*
bool CDiaphragmWeld::ProduceVerticanWeld(CRobotDriverAdaptor* pRobotDriver, vector<T_ALGORITHM_POINT> vtLinePoints, double dTrackAngle, double dTrackAngleLeft)
{
	//double dTrackAngle, double dTrackAngleLeft Reep点添加，是起点沿着组成立焊的两个面往外延伸得到
	vector<T_ROBOT_COORS> vtRobotWeldPoints, vtAdjustRobotWeldPoints;
	vtRobotWeldPoints.clear();
	vtAdjustRobotWeldPoints.clear();
	T_ROBOT_COORS tRobotPoint;
	double dStartRx = 0.0, dStartRy = 0.0, dStartRz = 0.0;
	double dEndRx = 0.0, dEndRy = 0.0, dEndRz = 0.0;
	double dMedianRx = 0.0, dMedianRy = 0.0, dMedianRz = 0.0;
	double dThroghHoleHight = 0;
	double dAdjustX = 0.0, dAdjustY = 0.0;
	double dStartAdjustX = 0.0, dStartAdjustY = 0.0;

	COPini opini;
	CString cstrTmp;
	CString verticalNo;
	CString cstrAdjust;//调整量
	verticalNo = "WELD_VERTICAL";
	cstrAdjust = "WELD_VERTICAL_ADJUST";
	cstrTmp = DATA_PATH + m_ptUnit->GetUnitName() + "\\VerticalPostureParam.ini";
	opini.SetFileName(cstrTmp);
	opini.SetSectionName(verticalNo);
	opini.ReadString("KEY_D_LEFT_ROBOT_START_RX", &dStartRx);
	opini.ReadString("KEY_D_LEFT_ROBOT_START_RY", &dStartRy);
	opini.ReadString("KEY_D_LEFT_ROBOT_START_RZ", &dStartRz);

	opini.ReadString("KEY_D_LEFT_ROBOT_MEDIAN_RX", &dMedianRx);
	opini.ReadString("KEY_D_LEFT_ROBOT_MEDIAN_RY", &dMedianRy);
	opini.ReadString("KEY_D_LEFT_ROBOT_MEDIAN_RZ", &dMedianRz);

	opini.ReadString("KEY_D_LEFT_ROBOT_END_RX", &dEndRx);
	opini.ReadString("KEY_D_LEFT_ROBOT_END_RY", &dEndRy);
	opini.ReadString("KEY_D_LEFT_ROBOT_END_RZ", &dEndRz);
	opini.SetSectionName(cstrAdjust);
	opini.ReadString("KEY_D_LEFT_ADJUST_X_START", &dStartAdjustX);
	opini.ReadString("KEY_D_LEFT_ADJUST_Y_START", &dStartAdjustY);
	opini.ReadString("KEY_D_LEFT_ADJUST_X", &dAdjustX);
	opini.ReadString("KEY_D_LEFT_ADJUST_Y", &dAdjustY);
	WriteLog("立缝:补偿值：%lf %lf %lf %lf", dStartAdjustX, dStartAdjustY, dAdjustX, dAdjustY);

	double dDirx = CosD(dTrackAngle);
	double dDiry = SinD(dTrackAngle);

	double dLeftDirx = CosD(dTrackAngleLeft);
	double dLeftDiry = SinD(dTrackAngleLeft);

	double dRZPosture = atan2(dDiry + dLeftDiry, dDirx + dLeftDirx) * 180 / 3.1415926;


	int nSize = vtLinePoints.size();
	if (nSize < 5)
	{
		XiMessageBox("立峰数据错误，请检查:%d", nSize);
	}
	for (int nPointNo = 0; nPointNo < vtLinePoints.size(); nPointNo++)
	{
		//坐标加姿态

		tRobotPoint.dX = vtLinePoints[nPointNo].dCoorX + dAdjustX;
		tRobotPoint.dY = vtLinePoints[nPointNo].dCoorY + dAdjustY;
		tRobotPoint.dZ = vtLinePoints[nPointNo].dCoorZ - dThroghHoleHight;
		//第一段
		if (nPointNo < 5)
		{
			tRobotPoint.dRX = dStartRx + (dMedianRx - dStartRx) * (nPointNo / 4.0);
			tRobotPoint.dRY = dStartRy + (dMedianRy - dStartRy) * (nPointNo / 4.0);
			tRobotPoint.dRZ = dRZPosture;
		}
		//第二段
		else if (nPointNo > 4)
		{
			tRobotPoint.dRX = dMedianRx;
			tRobotPoint.dRY = dMedianRy;
			tRobotPoint.dRZ = dRZPosture;
		}
		//第三段
		// 			else if (15 < nPointNo)
		// 			{
		// 				tRobotPoint.dRX = dMedianRx + (dEndRx - dMedianRx)*((nPointNo - 15)/5.0);
		// 				tRobotPoint.dRY = dMedianRy + (dEndRy - dMedianRy)*((nPointNo - 15)/5.0);
		// 				tRobotPoint.dRZ = dMedianRz + (dEndRz - dMedianRz)*((nPointNo - 15)/5.0);
		// 			}

		vtRobotWeldPoints.push_back(tRobotPoint);
	}

	// 加载工艺参数
	vector<T_WELD_PARA> vtWeldPara;
	T_WELD_PARA tWeldPara;
	E_WELD_SEAM_TYPE eWeldSeamType = E_STAND_SEAM;
	if (false == GetCurWeldParam(eWeldSeamType, vtWeldPara))
	{
		XiMessageBox("加载焊接工艺参数失败！");
		return false;
	}
	for (int n = 0; n < vtWeldPara.size(); n++)
	{
		tWeldPara = vtWeldPara.at(0);
		TrackComp(eWeldSeamType, vtRobotWeldPoints, tWeldPara, vtAdjustRobotWeldPoints);
		CHECK_BOOL_RETURN(m_ptUnit->SendWeldParam(*m_pIsArcOn, tWeldPara));
		//计算安全位置，检查轨迹是否超限
		double dSafeRY = -65;
		T_ROBOT_COORS tFirstSafePos = vtAdjustRobotWeldPoints[vtAdjustRobotWeldPoints.size() - 1];
		T_ROBOT_COORS tFirstSafePosOne = vtAdjustRobotWeldPoints[vtAdjustRobotWeldPoints.size() - 1];
		RobotCoordPosOffset(tFirstSafePosOne, dRZPosture, 200., 100. * m_nRobotInstallDir);
		RobotCoordPosOffset(tFirstSafePos, dRZPosture, 50., 50. * m_nRobotInstallDir);

		tFirstSafePosOne.dRY = m_dTransitionsRy;
		tFirstSafePos.dRY = dStartRy;

		bool bTransPosture = false;
		int nTrck = 0;
		T_ANGLE_PULSE tReferPulse = pRobotDriver->GetCurrentPulse(), tRstPulse, tSafePulse, tSafeOnePulse;
		vector<T_ANGLE_PULSE> vtMeasurePulse;

		vtAdjustRobotWeldPoints.insert(vtAdjustRobotWeldPoints.begin(), tFirstSafePos);
		vtAdjustRobotWeldPoints.insert(vtAdjustRobotWeldPoints.begin(), tFirstSafePosOne);
		vtAdjustRobotWeldPoints.push_back(tFirstSafePosOne);
		vtAdjustRobotWeldPoints.push_back(tFirstSafePos);
		if (!CalcContinuePulseForWeld(vtAdjustRobotWeldPoints, vtMeasurePulse, false))
		{
			return false;
		}

		//移动到焊接安全位置
		T_ROBOT_MOVE_SPEED tPulseMove(1000, 20, 20);
		//pRobotDriver->SafeMoveJByJobSaxis(vtMeasurePulse.at(0), tPulseMove);
		CHECK_BOOL_RETURN(m_ptUnit->CheckRobotDone(vtMeasurePulse.at(0)));
		pRobotDriver->SetIntVar(19, m_dSafeApproachToWorkSpeed / 6);
		pRobotDriver->SetPosVar(18, vtMeasurePulse.at(1));
		pRobotDriver->SetPosVar(19, vtMeasurePulse.at(2));
		vtMeasurePulse.erase(vtMeasurePulse.begin(), vtMeasurePulse.begin() + 2);
		vtAdjustRobotWeldPoints.erase(vtAdjustRobotWeldPoints.begin(), vtAdjustRobotWeldPoints.begin() + 2);

		pRobotDriver->SetPosVar(61, vtMeasurePulse.at(vtMeasurePulse.size() - 1));
		pRobotDriver->SetPosVar(62, vtMeasurePulse.at(vtMeasurePulse.size() - 2));
		vtMeasurePulse.pop_back();
		vtMeasurePulse.pop_back();

		vtAdjustRobotWeldPoints.pop_back();
		vtAdjustRobotWeldPoints.pop_back();

		long long tStart = XI_clock();
		//添加Reep点
		tStart = XI_clock();
		XI_POINT tPoint1 = { vtAdjustRobotWeldPoints[vtAdjustRobotWeldPoints.size() - 1].dX,vtAdjustRobotWeldPoints[vtAdjustRobotWeldPoints.size() - 1].dY,vtAdjustRobotWeldPoints[vtAdjustRobotWeldPoints.size() - 1].dZ };
		XI_POINT tPoint2;
		CalcOffsetInBase(tPoint1, 10, 0.0, dTrackAngle + 90 * m_nRobotInstallDir, tPoint2);
		double ReepPointOne[6] = { tPoint2.x,tPoint2.y,tPoint2.z,vtAdjustRobotWeldPoints[vtAdjustRobotWeldPoints.size() - 1].dRX,vtAdjustRobotWeldPoints[vtAdjustRobotWeldPoints.size() - 1].dRY,
			vtAdjustRobotWeldPoints[vtAdjustRobotWeldPoints.size() - 1].dRZ };
		pRobotDriver->SetPosVar(1, ReepPointOne, 0);
		CalcOffsetInBase(tPoint1, 10, 0.0, dTrackAngleLeft - 90 * m_nRobotInstallDir, tPoint2);
		double ReepPointTwo[6] = { tPoint2.x,tPoint2.y,tPoint2.z,vtAdjustRobotWeldPoints[vtAdjustRobotWeldPoints.size() - 1].dRX,vtAdjustRobotWeldPoints[vtAdjustRobotWeldPoints.size() - 1].dRY,
			vtAdjustRobotWeldPoints[vtAdjustRobotWeldPoints.size() - 1].dRZ };
		pRobotDriver->SetPosVar(2, ReepPointTwo, 0);
		pRobotDriver->SetPosVar(19, vtAdjustRobotWeldPoints.at(0));
		vtAdjustRobotWeldPoints.erase(vtAdjustRobotWeldPoints.begin(), vtAdjustRobotWeldPoints.begin() + 1);
		pRobotDriver->SetIntVar(19, 500);
		SendDataTOCallJobRobot(pRobotDriver, vtAdjustRobotWeldPoints);
		m_ptUnit->RobotCheckDone();
	}
	return true;
}*/

bool CDiaphragmWeld::GetVerticalReleveInfo(T_VERTICAL_INFO_NEW tVerticalInfo, VT_VERTICAL_INFO_NEW* pvtVerticalInfo,
	VT_CORNER_LINE_INFO_NEW* pvtCornerLineInfo, VT_ENDPOINT_INFO_NEW* pvtPointInfo, T_VERTICAL_RELEVE_INFO& tVerticalReleInfo)
{
	vector<T_CORNER_LINE_INFO_NEW> vtCorner_Info;
	VT_ENDPOINT_INFO_NEW vtEndpoint_Info;
	T_CORNER_LINE_INFO_NEW tCornerLine, tCornerLineReleve, tReleveVerticalCor;
	T_ENDPOINT_INFO_NEW tStartPoint, tEndPoint;
	tCornerLine = FindCornerLineInfoFun(tVerticalInfo.nCornerLineNo, pvtCornerLineInfo);
	tCornerLineReleve = FindCornerLineInfoFun(tCornerLine.bStartContiguousCor, pvtCornerLineInfo);
	tStartPoint = FindEndPointInfoFun(tVerticalInfo.nStartNo, pvtPointInfo);
	tEndPoint = FindEndPointInfoFun(tVerticalInfo.nEndNo, pvtPointInfo);
	//端点信息
	vtEndpoint_Info.push_back(tStartPoint);
	vtEndpoint_Info.push_back(tEndPoint);
	//角缝信息
	vtCorner_Info.push_back(tCornerLine);
	vtCorner_Info.push_back(tCornerLineReleve);
	//放入信息
	tVerticalReleInfo.vvtEndpoint_Info.push_back(vtEndpoint_Info);
	tVerticalReleInfo.vtVertical_Info.push_back(tVerticalInfo);
	tVerticalReleInfo.vvtCorner_Info.push_back(vtCorner_Info);

	//查找关联立缝
	tVerticalReleInfo.bIsReleveVertical = false;
	T_VERTICAL_INFO_NEW tRelevanceVertical;
	if (FindRelevanceVerticalFun(tVerticalInfo,
		pvtVerticalInfo, pvtPointInfo, pvtCornerLineInfo, tRelevanceVertical))
	{
		tVerticalReleInfo.bIsReleveVertical = true;
		vtCorner_Info.clear();
		vtEndpoint_Info.clear();
		tReleveVerticalCor = FindCornerLineInfoFun(tRelevanceVertical.nCornerLineNo, pvtCornerLineInfo);
		tCornerLineReleve = FindCornerLineInfoFun(tReleveVerticalCor.bStartContiguousCor, pvtCornerLineInfo);

		vtCorner_Info.push_back(tReleveVerticalCor);
		vtCorner_Info.push_back(tCornerLineReleve);

		tStartPoint = FindEndPointInfoFun(tRelevanceVertical.nStartNo, pvtPointInfo);
		tEndPoint = FindEndPointInfoFun(tRelevanceVertical.nEndNo, pvtPointInfo);
		//端点信息
		vtEndpoint_Info.push_back(tStartPoint);
		vtEndpoint_Info.push_back(tEndPoint);

		tVerticalReleInfo.vvtEndpoint_Info.push_back(vtEndpoint_Info);
		tVerticalReleInfo.vtVertical_Info.push_back(tRelevanceVertical);
		tVerticalReleInfo.vvtCorner_Info.push_back(vtCorner_Info);

		// 检查关联立焊在原始立焊那一侧
		double dverangle = tVerticalReleInfo.vvtCorner_Info.at(0).at(0).dStartAngle;
		double dReleverangle = tVerticalReleInfo.vvtCorner_Info.at(1).at(0).dStartAngle;
		double dOffset = dverangle - dReleverangle;
		dOffset = dOffset < 0 ? dOffset + 360 : dOffset;
		if (dOffset > 180.0)//左侧
		{
			reverse(tVerticalReleInfo.vvtEndpoint_Info.begin(), tVerticalReleInfo.vvtEndpoint_Info.end());
			reverse(tVerticalReleInfo.vtVertical_Info.begin(), tVerticalReleInfo.vtVertical_Info.end());
			reverse(tVerticalReleInfo.vvtCorner_Info.begin(), tVerticalReleInfo.vvtCorner_Info.end());
		}
	}

	return true;
}

bool CDiaphragmWeld::GetVerticalMeasureData(CRobotDriverAdaptor* pRobotDriver, T_VERTICAL_RELEVE_INFO tVerticalReleInfo,
	vector<T_ROBOT_COORS>& vtMeasurePoint, vector<T_ANGLE_PULSE>& vtMeasurePulse, vector<int>& vnMeasureType, double& dExAxis)
{
	if (tVerticalReleInfo.vtVertical_Info.size() < 1)
	{
		XiMessageBox("获取测量信息函数输入有误");
		return false;
	}

	//计算测量位置
	vector<XI_POINT> vtMeasureData;
	T_ROBOT_COORS tMeasurePointR, tMeasurePointL, tTempPoint;
	vector<T_ROBOT_COORS> vtMeasureTemp;
	CString cstrTmp;
	vtMeasureData.clear();
	vtMeasurePoint.clear();
	vnMeasureType.clear();
	vtMeasurePulse.clear();
	if (tVerticalReleInfo.bIsReleveVertical)
	{
		XI_POINT tStartPointReleve = tVerticalReleInfo.vvtEndpoint_Info.at(1).at(0).tPointData;
		double dNorAngleReleve = tVerticalReleInfo.vvtCorner_Info.at(1).at(1).dStartAngle;

		//关联立焊测量信息
		if (tVerticalReleInfo.vvtCorner_Info.at(1).at(1).dBoardLength <
			(m_dHandEyeDis + m_dMeasureDisThreshold * 3))
		{
			//关联立焊测量信息
			double dDiff = (tVerticalReleInfo.vvtCorner_Info.at(1).at(1).dBoardLength - m_dHandEyeDis - m_dMeasureDisThreshold) / 2;
			if (0 == tVerticalReleInfo.vvtCorner_Info.at(1).at(1).nStartType)
			{
				dDiff = (tVerticalReleInfo.vvtCorner_Info.at(1).at(1).dBoardLength - m_dMeasureDisThreshold) / 2;
			}
			if (dDiff > m_dMeasureDisThreshold)
			{
				dDiff = m_dMeasureDisThreshold;
			}
			if (dDiff < 10)
			{
				XiMessageBox("测量焊缝太短，测量干涉");
				return false;
			}
			CalcMeasureInBase(tStartPointReleve, m_dMeasureDisThreshold + dDiff, 0, dNorAngleReleve, tTempPoint);//关联板起点相邻焊缝测量
			vtMeasurePoint.push_back(tTempPoint);
			vnMeasureType.push_back(E_DOUBLE_LONG_LINE);
			CalcMeasureInBase(tStartPointReleve, m_dMeasureDisThreshold, 0, dNorAngleReleve, tTempPoint);//关联板起点相邻焊缝测量
			vtMeasurePoint.push_back(tTempPoint);
			vnMeasureType.push_back(E_DOUBLE_LONG_LINE);
		}
		else
		{
			//关联立焊测量信息
			CalcMeasureInBase(tStartPointReleve, m_dMeasureDisThreshold, 0, dNorAngleReleve, tTempPoint);//关联板面测量
			vtMeasurePoint.push_back(tTempPoint);
			vnMeasureType.push_back(E_DOUBLE_LONG_LINE);
			CalcMeasureInBase(tStartPointReleve, m_dMeasureDisThreshold * 2, 0, dNorAngleReleve, tTempPoint);//关联板面测量
			vtMeasurePoint.push_back(tTempPoint);
			vnMeasureType.push_back(E_DOUBLE_LONG_LINE);

		}
		tTempPoint.dZ = m_dWorkPieceHighTop + 20 * m_nRobotInstallDir;
		vtMeasurePoint.push_back(tTempPoint);
		vnMeasureType.push_back(E_TRANSITION_POINT);
		tTempPoint.dRZ -= 90 * m_nRobotInstallDir;
		vtMeasurePoint.push_back(tTempPoint);
		vnMeasureType.push_back(E_TRANSITION_POINT);
	}

	XI_POINT tStartPoint = tVerticalReleInfo.vvtEndpoint_Info.at(0).at(0).tPointData;
	double dNorAngle = tVerticalReleInfo.vvtCorner_Info.at(0).at(0).dStartAngle;
	double dNorAngleEnd = tVerticalReleInfo.vvtCorner_Info.at(0).at(1).dEndAngle;
	//右侧测量点
	if (tVerticalReleInfo.vvtCorner_Info.at(0).at(0).dBoardLength <
		(m_dHandEyeDis + m_dMeasureDisThreshold * 3) || tVerticalReleInfo.vvtCorner_Info.at(0).at(0).dBoardLength < 150)
	{
		//测量立板长度小于250时需要便姿态测量，使测量面尽量靠近焊缝交点。
		//测量时激光角点在在焊道上偏移80MM时，此时枪尖相对交点法相偏移90，焊道方向偏移20,
		//角度逆向偏转55度
		double dShiftDis1 = -50.0;
		double dShiftDis2 = -(tVerticalReleInfo.vvtCorner_Info.at(0).at(0).dBoardLength - fabs(dShiftDis1)) / 2;
		CalcMeasureInBase(tStartPoint, dShiftDis1, 0, dNorAngle, tMeasurePointR);//关联板面测量
		tMeasurePointR.dRZ += 55 * m_nRobotInstallDir;
		tMeasurePointR.dRZ = tMeasurePointR.dRZ > 180 ? tMeasurePointR.dRZ - 360 : tMeasurePointR.dRZ;
		tMeasurePointR.dRZ = tMeasurePointR.dRZ < -180 ? tMeasurePointR.dRZ + 360 : tMeasurePointR.dRZ;
		tTempPoint = tMeasurePointR;
		tTempPoint.dZ = m_dWorkPieceHighTop + 20 * m_nRobotInstallDir;
		vtMeasurePoint.push_back(tTempPoint);
		vnMeasureType.push_back(E_TRANSITION_POINT);
		vtMeasurePoint.push_back(tMeasurePointR);
		vnMeasureType.push_back(E_DOUBLE_LONG_LINE);
		CalcMeasureInBase(tStartPoint, dShiftDis1 + dShiftDis2, 0, dNorAngle, tMeasurePointR);//关联板面测量
		tMeasurePointR.dRZ += 55 * m_nRobotInstallDir;
		tMeasurePointR.dRZ = tMeasurePointR.dRZ > 180 ? tMeasurePointR.dRZ - 360 : tMeasurePointR.dRZ;
		tMeasurePointR.dRZ = tMeasurePointR.dRZ < -180 ? tMeasurePointR.dRZ + 360 : tMeasurePointR.dRZ;
		vtMeasurePoint.push_back(tMeasurePointR);
		vnMeasureType.push_back(E_DOUBLE_LONG_LINE);

	}
	else
	{
		CalcMeasureInBase(tStartPoint, -(m_dHandEyeDis + m_dMeasureDisThreshold * 2), 0, dNorAngle, tMeasurePointR);//关联板面测量
		vtMeasurePoint.push_back(tMeasurePointR);
		vnMeasureType.push_back(E_DOUBLE_LONG_LINE);
		CalcMeasureInBase(tStartPoint, -(m_dHandEyeDis + m_dMeasureDisThreshold), 0, dNorAngle, tMeasurePointR);//关联板面测量
		tTempPoint = tMeasurePointR;
		tTempPoint.dZ = m_dWorkPieceHighTop + 20 * m_nRobotInstallDir;
		vtMeasurePoint.push_back(tMeasurePointR);
		vnMeasureType.push_back(E_DOUBLE_LONG_LINE);

	}

	//左侧测量点
	if (tVerticalReleInfo.vvtCorner_Info.at(0).at(1).dBoardLength <
		(m_dHandEyeDis + m_dMeasureDisThreshold * 3))
	{
		double dDiff = (tVerticalReleInfo.vvtCorner_Info.at(0).at(1).dBoardLength - m_dHandEyeDis - m_dMeasureDisThreshold) / 2;
		if (0 == tVerticalReleInfo.vvtCorner_Info.at(0).at(1).nStartType)
		{
			dDiff = (tVerticalReleInfo.vvtCorner_Info.at(0).at(1).dBoardLength - m_dMeasureDisThreshold) / 2;
		}
		if (dDiff > m_dMeasureDisThreshold)
		{
			dDiff = m_dMeasureDisThreshold;
		}
		if (dDiff < 10)
		{
			XiMessageBox("测量焊缝太短，测量干涉");
			return false;
		}
		CalcMeasureInBase(tStartPoint, m_dMeasureDisThreshold + dDiff, 0, dNorAngleEnd, tMeasurePointL);//关联板起点相邻焊缝测量
		vtMeasurePoint.push_back(tMeasurePointL);
		vnMeasureType.push_back(E_DOUBLE_LONG_LINE);
		CalcMeasureInBase(tStartPoint, m_dMeasureDisThreshold, 0, dNorAngleEnd, tMeasurePointL);//关联板起点相邻焊缝测量
		vtMeasurePoint.push_back(tMeasurePointL);
		vnMeasureType.push_back(E_DOUBLE_LONG_LINE);

	}
	else
	{
		CalcMeasureInBase(tStartPoint, m_dMeasureDisThreshold * 2, 0, dNorAngleEnd, tMeasurePointL);//关联板起点相邻焊缝测量
		vtMeasurePoint.push_back(tMeasurePointL);
		vnMeasureType.push_back(E_DOUBLE_LONG_LINE);
		CalcMeasureInBase(tStartPoint, m_dMeasureDisThreshold, 0, dNorAngleEnd, tMeasurePointL);//关联板起点相邻焊缝测量
		vtMeasurePoint.push_back(tMeasurePointL);
		vnMeasureType.push_back(E_DOUBLE_LONG_LINE);
	}

	//板高测量点
	tMeasurePointL.dZ = tVerticalReleInfo.vvtCorner_Info.at(0).at(1).dBoardHight;
	vtMeasurePoint.push_back(tMeasurePointL);
	vnMeasureType.push_back(E_LS_RL_FLIP);
	tMeasurePointR.dZ = tVerticalReleInfo.vvtCorner_Info.at(0).at(0).dBoardHight;
	vtMeasurePoint.push_back(tMeasurePointR);
	vnMeasureType.push_back(E_LS_RL_FLIP);
	reverse(vtMeasurePoint.begin(), vtMeasurePoint.end());//转换测量顺序
	reverse(vnMeasureType.begin(), vnMeasureType.end());
	// 分解世界坐标
	CHECK_BOOL_RETURN(DecomposeWorldCoordinates(pRobotDriver, vtMeasureTemp, vtMeasurePoint, true));
	// 添加收下枪安全位置
	double dAdjustdis = 50.0;
	CHECK_BOOL_RETURN(AddSafeDownGunPos(vtMeasureTemp, vnMeasureType, dAdjustdis, m_dWorkPieceHighTop));

	//计算连续可运行脉冲坐标
	if (!CalcContinuePulseForWeld(vtMeasureTemp, vtMeasurePulse, false))
	{
		return false;
	}
	vtMeasurePoint.clear();
	vtMeasurePoint.insert(vtMeasurePoint.end(), vtMeasureTemp.begin(), vtMeasureTemp.end());
	return true;
}

bool CDiaphragmWeld::CalcVerticalTrack(CRobotDriverAdaptor* pRobotDriver, T_VERTICAL_RELEVE_INFO& tVerticalReleInfo)
{

	if (m_vtTeachResult.size() < 4)
	{
		XiMessageBox("立缝测量数据超出最少数量");
		return false;
	}
	//01 测量高点 23测量公共面 4567 组成立焊面的数据 23左45左线23右线  23左线67左线67右线
	vector<T_ALGORITHM_POINT> vtFirstFacePoints, vtSecondFacePoints, vtThirdFacePoints, vtFourthFacePoints, vtFifthFacePoints;
	vector<T_ALGORITHM_POINT> vtAlgPtns;
	T_ALGORITHM_POINT tIntersectionPoint, tReleIntersectionPt;//角点坐标
	vector<vector<T_ALGORITHM_POINT>> vvtUpLinePoints;
	vector<T_ALGORITHM_POINT> vtUpLinePoints;
	//23测量时组成立面
	XiPointAddToAlgPtn(m_vtTeachResult[2].vtLeftPtns3D, vtFirstFacePoints);
	XiPointAddToAlgPtn(m_vtTeachResult[3].vtLeftPtns3D, vtFirstFacePoints);

	//23测量时组成底面
	XiPointAddToAlgPtn(m_vtTeachResult[2].vtRightPtns3D, vtThirdFacePoints);
	XiPointAddToAlgPtn(m_vtTeachResult[3].vtRightPtns3D, vtThirdFacePoints);

	//45测量时组成立面
	XiPointAddToAlgPtn(m_vtTeachResult[4].vtLeftPtns3D, vtSecondFacePoints);
	XiPointAddToAlgPtn(m_vtTeachResult[5].vtLeftPtns3D, vtSecondFacePoints);


	// 立焊Z值
	double dVreticalHightZ = m_vtTeachResult.at(0).tKeyPtn3D.z;
	if ((m_vtTeachResult.at(0).tKeyPtn3D.z < m_vtTeachResult.at(1).tKeyPtn3D.z
		&& -1 == m_nRobotInstallDir) ||
		(m_vtTeachResult.at(0).tKeyPtn3D.z > m_vtTeachResult.at(1).tKeyPtn3D.z
			&& 1 == m_nRobotInstallDir))
	{
		dVreticalHightZ = m_vtTeachResult.at(1).tKeyPtn3D.z;
	}

	//根据测量数据计算立缝数据
	XiAlgorithm xiAlgorithm;
	xiAlgorithm.GetThreePlaneInterSectionPoint(vtFirstFacePoints, vtSecondFacePoints,
		vtThirdFacePoints, tIntersectionPoint, 500, 1, vtUpLinePoints);
	if (vtUpLinePoints.size() < 1)
	{
		XiMessageBox("立焊数据有误");
		return false;
	}
	int nNo = 0, n = 0;
	for (n = 0; n < vtUpLinePoints.size(); n++)
	{
		if (vtUpLinePoints[n].dCoorZ < dVreticalHightZ)
		{
			break;
		}
	}
	vtUpLinePoints.erase(vtUpLinePoints.begin() + n, vtUpLinePoints.end());
	CString cstrTmp;
	cstrTmp.Format("%sVerticalMachine.txt", tVerticalReleInfo.strFile);
	SaveDataAlgorithm(vtUpLinePoints, cstrTmp.GetBuffer(0));
	tVerticalReleInfo.vvtVerticalRealTrackCoors.push_back(vtUpLinePoints);


	if (tVerticalReleInfo.bIsReleveVertical)
	{
		//67测量时组成立面
		XiPointAddToAlgPtn(m_vtTeachResult[6].vtLeftPtns3D, vtFourthFacePoints);
		XiPointAddToAlgPtn(m_vtTeachResult[7].vtLeftPtns3D, vtFourthFacePoints);

		//67测量时组成底面
		XiPointAddToAlgPtn(m_vtTeachResult[6].vtRightPtns3D, vtFifthFacePoints);
		XiPointAddToAlgPtn(m_vtTeachResult[7].vtRightPtns3D, vtFifthFacePoints);

		vtUpLinePoints.clear();
		xiAlgorithm.GetThreePlaneInterSectionPoint(vtFirstFacePoints, vtFourthFacePoints,
			vtFifthFacePoints, tReleIntersectionPt, 500, 1, vtUpLinePoints);
		if (vtUpLinePoints.size() < 1)
		{
			XiMessageBox("立焊数据有误");
			return false;
		}
		int nNo = 0, n = 0;
		for (n = 0; n < vtUpLinePoints.size(); n++)
		{
			if (vtUpLinePoints[n].dCoorZ < dVreticalHightZ)
			{
				break;
			}
		}
		vtUpLinePoints.erase(vtUpLinePoints.begin() + n, vtUpLinePoints.end());
		cstrTmp.Format("%s%d_VerticalMachineReleve.txt", tVerticalReleInfo.strFile, tVerticalReleInfo.vtVertical_Info.at(1).nBoardNo);
		SaveDataAlgorithm(vtUpLinePoints, cstrTmp.GetBuffer(0));
		tVerticalReleInfo.vvtVerticalRealTrackCoors.push_back(vtUpLinePoints);
	}
	return true;
}

//bool CDiaphragmWeld::VerticalWeldFunNew(CRobotDriverAdaptor* pRobotDriver, T_VERTICAL_INFO_NEW tVerticalInfo, VT_VERTICAL_INFO_NEW* pvtVerticalInfo,
//	VT_CORNER_LINE_INFO_NEW* pvtCornerLineInfo, VT_ENDPOINT_INFO_NEW* pvtPointInfo, VT_MEASURE_INFO_NEW* pvMeasureInfo)
//{
//	/*立焊焊接逻辑:每次只焊接一条立缝
//	*1、根据工件数据信息获取，关联角缝信息，关联角缝起点相邻角缝信息，端点信息，
//	*2、测量立缝关联立板高度：确定立焊高度，两立板板面及底面信息：拿到完整的立缝信息
//	*3、将获取到的立缝起点更新到关联的角缝起终点。
//	*3、开始焊接
//	*/
//	//CHECK_STOP_RETURN_STATE(pRobotDriver);
//	long long dStartTim;
//	dStartTim = XI_clock();
//	int nRobotTypeNo = 0;
//	CString cstrPath,cstrTmp;
//	cstrPath.Format(OUTPUT_PATH + m_ptUnit->GetUnitName() + WELDDATA_PATH + "VerticalWeld\\%d_", tVerticalInfo.nBoardNo);
//	SaveWorkpieceNo(pRobotDriver, tVerticalInfo.nBoardNo);	//保存工件序号
//	InitWeldStartVal(pRobotDriver);							//初始化参数，工艺
//
//	//获取关联信息
//	T_VERTICAL_RELEVE_INFO tVerticalReleInfo;
//	CHECK_BOOL_RETURN(GetVerticalReleveInfo(tVerticalInfo, pvtVerticalInfo, pvtCornerLineInfo, pvtPointInfo, tVerticalReleInfo));
//	tVerticalReleInfo.strFile = cstrPath;
//	//获取测量信息
//	vector<T_ANGLE_PULSE> vtMeasurePulse;
//	vector<int> vnMeasureType;
//	vector<T_ROBOT_COORS> vtMeasurePoint;
//	double dExAxisPos = 0;
//	CHECK_BOOL_RETURN(GetVerticalMeasureData(pRobotDriver, tVerticalReleInfo, vtMeasurePoint, vtMeasurePulse, vnMeasureType, dExAxisPos));
//	//开始测量
//	if (true)
//	{
//		// 移动大车到安全位置
//		CHECK_BOOL_RETURN(PosMoveMachine(pRobotDriver, dExAxisPos, m_dSafePosRunSpeed, COORD_ABS));
//		 T_ROBOT_MOVE_SPEED tPulseMove ( m_dFastApproachToWorkSpeed, 50,50 );
//		//pRobotDriver->SafeMoveJByJobSaxis(vtMeasurePulse.at(0), tPulseMove);
//		CHECK_BOOL_RETURN(m_ptUnit->CheckRobotDone(vtMeasurePulse.at(0)));
//		// 开始示教	
//		if (!g_bLocalDebugMark)
//		{
//			 CHECK_BOOL_RETURN(DoTeach(tVerticalInfo.nBoardNo, vtMeasurePulse, vnMeasureType, dExAxisPos));
//		}
//		else {
//			return true;
//		}
//		// 计算焊接轨迹		 		 
//		CHECK_BOOL_RETURN(CalcVerticalTrack(pRobotDriver, tVerticalReleInfo));
//	}
//	else
//	{
//		vector<T_ALGORITHM_POINT> vtUpLinePoints, vtReleUpLinePoints;
//		cstrTmp = cstrPath + "VerticalMachine.txt";
//		LoadDataAlgorithm(vtUpLinePoints, cstrTmp.GetBuffer());
//		tVerticalReleInfo.vvtVerticalRealTrackCoors.push_back(vtUpLinePoints);
//		cstrTmp.Format("%s%d_VerticalMachineReleve.txt", cstrPath, tVerticalReleInfo.vtVertical_Info.at(1).nBoardNo);
//		LoadDataAlgorithm(vtReleUpLinePoints, cstrTmp.GetBuffer());
//		tVerticalReleInfo.vvtVerticalRealTrackCoors.push_back(vtReleUpLinePoints);
//	}
//	//先焊接后测量焊道
//	reverse(tVerticalReleInfo.vvtVerticalRealTrackCoors.begin(), tVerticalReleInfo.vvtVerticalRealTrackCoors.end());
//	reverse(tVerticalReleInfo.vvtCorner_Info.begin(), tVerticalReleInfo.vvtCorner_Info.end());
//	reverse(tVerticalReleInfo.vtVertical_Info.begin(), tVerticalReleInfo.vtVertical_Info.end());
//	reverse(tVerticalReleInfo.vvtEndpoint_Info.begin(), tVerticalReleInfo.vvtEndpoint_Info.end());
//
//	for (int nVer = 0; nVer < tVerticalReleInfo.vvtVerticalRealTrackCoors.size(); nVer++)
//	{
//		int nSize = tVerticalReleInfo.vvtVerticalRealTrackCoors.at(nVer).size();
//		//更新关联端点
//		XI_POINT tPEndPoint = {
//			tVerticalReleInfo.vvtVerticalRealTrackCoors.at(nVer).at(0).dCoorX,
//			tVerticalReleInfo.vvtVerticalRealTrackCoors.at(nVer).at(0).dCoorY + dExAxisPos,
//			tVerticalReleInfo.vvtVerticalRealTrackCoors.at(nVer).at(0).dCoorZ };
//
//		RecordCorrectEndPointFun(pRobotDriver, tVerticalReleInfo.vvtCorner_Info.at(nVer).at(0).nStartNo, tPEndPoint);
//		RecordCorrectEndPointFun(pRobotDriver, tVerticalReleInfo.vvtCorner_Info.at(nVer).at(1).nEndNo, tPEndPoint);
//		//采样
//		int nsize = tVerticalReleInfo.vvtVerticalRealTrackCoors.at(nVer).size();
//		int nstep = nsize / 20;
//		vector<T_ALGORITHM_POINT> vtRealWeldDatas;
//		vtRealWeldDatas.clear();
//		for (int n = 0; n < tVerticalReleInfo.vvtVerticalRealTrackCoors.at(nVer).size(); n++)
//		{
//			tVerticalReleInfo.vvtVerticalRealTrackCoors.at(nVer).at(n).dCoorY = tVerticalReleInfo.vvtVerticalRealTrackCoors.at(nVer).at(n).dCoorY;
//			if (n % nstep == 0)
//			{
//				vtRealWeldDatas.push_back(tVerticalReleInfo.vvtVerticalRealTrackCoors.at(nVer).at(n));
//			}
//		}
//		//插入结尾点
//		double dis = TwoPointDis(
//			vtRealWeldDatas.at(vtRealWeldDatas.size() - 1).dCoorX,
//			vtRealWeldDatas.at(vtRealWeldDatas.size() - 1).dCoorY,
//			vtRealWeldDatas.at(vtRealWeldDatas.size() - 1).dCoorZ,
//			tVerticalReleInfo.vvtVerticalRealTrackCoors.at(nVer).at(nsize - 1).dCoorX,
//			tVerticalReleInfo.vvtVerticalRealTrackCoors.at(nVer).at(nsize - 1).dCoorY,
//			tVerticalReleInfo.vvtVerticalRealTrackCoors.at(nVer).at(nsize - 1).dCoorZ);
//		if (dis > 1)
//		{
//			vtRealWeldDatas.push_back(tVerticalReleInfo.vvtVerticalRealTrackCoors.at(nVer).at(nsize - 1));
//		}
//		cstrTmp = cstrPath + "VerticalRobot.txt";
//		SaveDataAlgorithm(vtRealWeldDatas, cstrTmp.GetBuffer());
//		//开始焊接
//		if (!ProduceVerticanWeld(pRobotDriver,
//								 vtRealWeldDatas,
//								 tVerticalReleInfo.vvtCorner_Info.at(nVer).at(0).dStartAngle,
//								 tVerticalReleInfo.vvtCorner_Info.at(nVer).at(1).dEndAngle))
//		{
//			return false;
//		}
//		//记录焊接状态
//		RecordWeldStatus(tVerticalReleInfo.vtVertical_Info.at(nVer).nBoardNo, true);
//	}
//	//回安全位置
//	//pRobotDriver->BackSafePosError(m_dSafeApproachToWorkSpeed);
//	return true;
//}

bool CDiaphragmWeld::GetVerticalMeasureDataNew(CRobotDriverAdaptor* pRobotDriver, T_BOARD_INFO_NEW& tBoardInfo, vector<T_ROBOT_COORS>& vtMeasurePoint
	, vector<T_ANGLE_PULSE>& vtMeasurePulse, vector<int>& vnMeasureType, double& dExAxisPos, int nIndex, bool bIsStart, T_ANGLE_PULSE* ptPrePulse)
{
	if (tBoardInfo.vtCornerRelsve_Info.size() < 1)
	{
		XiMessageBox("获取测量信息函数输入有误");
		return false;
	}

	// 测量如果为弧线且为起始点，现有结构测量干涉，需要更改姿态测量后拟合求交获取起始段数据用于跟踪

	 //计算测量位置
	T_ROBOT_COORS tMeasurePointR, tMeasurePointL, tTempPoint;
	vtMeasurePoint.clear();
	vnMeasureType.clear();
	int nIndex1 = 0, nIndex2 = 0, nIndex3 = 0;
	if (bIsStart)//起点
	{
		nIndex1 = nIndex;// 先测量焊缝序号
		nIndex2 = nIndex - 1;// 后测量焊缝序号
	}
	else//终点
	{
		nIndex1 = nIndex + 1;
		nIndex2 = nIndex;
	}
	// 工件类型
	bool bWorkPieceType = tBoardInfo.vtCornerRelsve_Info.at(nIndex1).bBoardType;
	bool bWorkPieceType2 = tBoardInfo.vtCornerRelsve_Info.at(nIndex2).bBoardType;

	int nTrackSize = tBoardInfo.vvtheoryTrackRobotCoors.at(nIndex1).size();// 先测焊缝
	int nTrackSize2 = tBoardInfo.vvtheoryTrackRobotCoors.at(nIndex2).size();// 后测焊缝
	// 先测取起点数据
	int nSIndx = m_dHandEyeDis + m_dMeasureDisThreshold;
	int nSIndx2 = m_dHandEyeDis + m_dMeasureDisThreshold * 2;
	int nSIndx3 = m_dHandEyeDis + m_dMeasureDisThreshold * 3;
	// 后测取起点数据
	int nEIndx = nTrackSize2 - m_dMeasureDisThreshold;
	int nEIndx2 = nTrackSize2 - m_dMeasureDisThreshold * 2;
	int nEIndx3 = nTrackSize2 - m_dMeasureDisThreshold * 3;

	int nSeamPtnNum = tBoardInfo.vvtheoryTrackRobotCoors.at(nIndex1).size();

	//右侧测量点
	if (tBoardInfo.vtCornerRelsve_Info.at(nIndex1).dBoardLength <
		(m_dHandEyeDis + m_dMeasureDisThreshold * 3))
	{
		//测量立板长度小于250时需要便姿态测量，使测量面尽量靠近焊缝交点。
		//测量时激光角点在在焊道上偏移80MM时，此时枪尖相对交点法相偏移90，焊道方向偏移20,
		//角度逆向偏转55度
		int nStep = 2;
		if (bWorkPieceType)//圆弧
		{
			nStep = 4;
		}
		double dShiftDis1 = 60.0;
		double dShiftDis2 = (tBoardInfo.vtCornerRelsve_Info.at(nIndex1).dBoardLength - fabs(dShiftDis1)) / nStep;

		tMeasurePointR = tBoardInfo.vvtheoryTrackRobotCoors.at(nIndex1).at(dShiftDis1);
		tMeasurePointR.dRZ += 55 * m_nRobotInstallDir;
		tMeasurePointR.dRZ = tMeasurePointR.dRZ > 180 ? tMeasurePointR.dRZ - 360 : tMeasurePointR.dRZ;
		tMeasurePointR.dRZ = tMeasurePointR.dRZ < -180 ? tMeasurePointR.dRZ + 360 : tMeasurePointR.dRZ;
		vtMeasurePoint.push_back(tMeasurePointR);
		vnMeasureType.push_back(E_DOUBLE_LONG_LINE);
		tMeasurePointR = tBoardInfo.vvtheoryTrackRobotCoors.at(nIndex1).at(dShiftDis1 + dShiftDis2);
		tMeasurePointR.dRZ += 55 * m_nRobotInstallDir;
		tMeasurePointR.dRZ = tMeasurePointR.dRZ > 180 ? tMeasurePointR.dRZ - 360 : tMeasurePointR.dRZ;
		tMeasurePointR.dRZ = tMeasurePointR.dRZ < -180 ? tMeasurePointR.dRZ + 360 : tMeasurePointR.dRZ;
		vtMeasurePoint.push_back(tMeasurePointR);
		vnMeasureType.push_back(E_DOUBLE_LONG_LINE);
		if (bWorkPieceType)//圆弧
		{
			tMeasurePointR = tBoardInfo.vvtheoryTrackRobotCoors.at(nIndex1).at(dShiftDis1 + dShiftDis2 * 2);
			tMeasurePointR.dRZ += 55 * m_nRobotInstallDir;
			tMeasurePointR.dRZ = tMeasurePointR.dRZ > 180 ? tMeasurePointR.dRZ - 360 : tMeasurePointR.dRZ;
			tMeasurePointR.dRZ = tMeasurePointR.dRZ < -180 ? tMeasurePointR.dRZ + 360 : tMeasurePointR.dRZ;
			vtMeasurePoint.push_back(tMeasurePointR);
			vnMeasureType.push_back(E_DOUBLE_LONG_LINE);
		}
	}
	else
	{
		{
			tMeasurePointR = tBoardInfo.vvtheoryTrackRobotCoors.at(nIndex1).at(nSIndx);
			vtMeasurePoint.push_back(tMeasurePointR);
			vnMeasureType.push_back(E_DOUBLE_LONG_LINE);
			tMeasurePointR = tBoardInfo.vvtheoryTrackRobotCoors.at(nIndex1).at(nSIndx2);
			vtMeasurePoint.push_back(tMeasurePointR);
			vnMeasureType.push_back(E_DOUBLE_LONG_LINE);
			if (bWorkPieceType)//圆弧
			{
				tMeasurePointR = tBoardInfo.vvtheoryTrackRobotCoors.at(nIndex1).at(nSIndx3);
				vtMeasurePoint.push_back(tMeasurePointR);
				vnMeasureType.push_back(E_DOUBLE_LONG_LINE);
			}
		}

	}
	// 后测
	tMeasurePointR = tBoardInfo.vvtheoryTrackRobotCoors.at(nIndex2).at(nEIndx);
	vtMeasurePoint.push_back(tMeasurePointR);
	vnMeasureType.push_back(E_DOUBLE_LONG_LINE);
	tMeasurePointR = tBoardInfo.vvtheoryTrackRobotCoors.at(nIndex2).at(nEIndx2);
	vtMeasurePoint.push_back(tMeasurePointR);
	vnMeasureType.push_back(E_DOUBLE_LONG_LINE);
	if (bWorkPieceType2)//圆弧
	{
		tMeasurePointR = tBoardInfo.vvtheoryTrackRobotCoors.at(nIndex2).at(nEIndx3);
		vtMeasurePoint.push_back(tMeasurePointR);
		vnMeasureType.push_back(E_DOUBLE_LONG_LINE);
	}
	string str = tBoardInfo.vtBoardPartInfo.at(nIndex).strfileName.substr(0, tBoardInfo.vtBoardPartInfo.at(nIndex).strfileName.find_last_of("\\"));
	CString cstr;

	// 存储调试用数据
	if (bIsStart)
	{
		tBoardInfo.vvtMeasure_Data_Start.push_back(vtMeasurePoint);
		cstr.Format("%s\\%d-StartPointThoryTeachCoors.txt", str.c_str(), tBoardInfo.vtCornerRelsve_Info.at(nIndex).nBoardNo);
	}
	else
	{
		tBoardInfo.vvtMeasure_Data_End.push_back(vtMeasurePoint);
		cstr.Format("%s\\%d-EndPointThoryTeachCoors.txt", str.c_str(), tBoardInfo.vtCornerRelsve_Info.at(nIndex).nBoardNo);
	}
	SaveDataRobotCoors(vtMeasurePoint, cstr.GetBuffer(0));
	// 分解世界坐标
	vector<T_ROBOT_COORS> vtMeasureTemp;
	CHECK_BOOL_RETURN(DecomposeWorldCoordinates(pRobotDriver, vtMeasureTemp, vtMeasurePoint, true));

	// 添加收下枪安全位置
	double dAdjustdis = 50.0;
	CHECK_BOOL_RETURN(AddSafeDownGunPos(vtMeasureTemp, vnMeasureType, dAdjustdis, m_dWorkPieceHighTop));
	 SaveDataRobotCoors(vtMeasureTemp, tBoardInfo.strfileName + "TeachPointCameraTool.txt");
	 //计算连续可运行脉冲坐标
	 CHECK_BOOL_RETURN(CalcContinuePulseForWeld(vtMeasureTemp, vtMeasurePulse, false));
	 vtMeasurePoint.clear();
	 vtMeasurePoint.insert(vtMeasurePoint.end(), vtMeasureTemp.begin(), vtMeasureTemp.end());
	 return true;
 }

bool CDiaphragmWeld::CalcMeasureIntersection(CRobotDriverAdaptor* pRobotDriver, T_BOARD_INFO_NEW& tBoardInfo, vector<T_TEACH_RESULT> vtTeachResult,
	double dExAxisPos, int nIndex, XI_POINT& tIntersection, bool bIsStart)
{
	if (vtTeachResult.size() < 4)
	{
		XiMessageBox("测量结果数据数量有误请检查数据");
		return false;

	}

	XI_POINT tStartp, tEndP;
	int nweldLineNum = tBoardInfo.vtCornerRelsve_Info.size();
	int nIndex1 = 0, nIndex2 = 1;
	if (!bIsStart)// 终点
	{
		nIndex1 = nIndex;
		nIndex2 = nIndex + 1;
	}
	else // 起点
	{
		nIndex1 = nIndex - 1;
		nIndex2 = nIndex;
	}
	XiAlgorithm* xiAlgorithm;
	xiAlgorithm = new XiAlgorithm;
	if (0 == tBoardInfo.vtCornerRelsve_Info.at(nIndex1).bBoardType
		&& 0 == tBoardInfo.vtCornerRelsve_Info.at(nIndex2).bBoardType)// 两直线相交
	{
		vector<T_ALGORITHM_POINT> vtFirstFacePoints;
		vector<T_ALGORITHM_POINT> vtSecondFacePoints;
		vector<T_ALGORITHM_POINT> vtThirdFacePoints;

		if (g_bLocalDebugMark)
		{
			tIntersection = bIsStart == true ? tBoardInfo.vtBoardPartInfo.at(nIndex).tStartPoint : tBoardInfo.vtBoardPartInfo.at(nIndex).tEndPoint;
			return true;
		}
		// 测量数据 01 测量立面 23立面 01底面
		//23测量时组成立面
		XiPointAddToAlgPtn(m_vtTeachResult[0].vtLeftPtns3D, vtFirstFacePoints);
		XiPointAddToAlgPtn(m_vtTeachResult[1].vtLeftPtns3D, vtFirstFacePoints);

		//23测量时组成底面
		XiPointAddToAlgPtn(m_vtTeachResult[0].vtRightPtns3D, vtThirdFacePoints);
		XiPointAddToAlgPtn(m_vtTeachResult[1].vtRightPtns3D, vtThirdFacePoints);

		//45测量时组成立面
		XiPointAddToAlgPtn(m_vtTeachResult[2].vtLeftPtns3D, vtSecondFacePoints);
		XiPointAddToAlgPtn(m_vtTeachResult[3].vtLeftPtns3D, vtSecondFacePoints);


		T_ALGORITHM_POINT tIntersectionPoint;//角点坐标
		vector<vector<T_ALGORITHM_POINT>> vvtUpLinePoints;
		vector<T_ALGORITHM_POINT> vtUpLinePoints;
		//根据测量数据计算立缝数据		 

		xiAlgorithm->GetThreePlaneInterSectionPoint(vtFirstFacePoints, vtSecondFacePoints,
			vtThirdFacePoints, tIntersectionPoint, 500, 1, vtUpLinePoints);
		if (vtUpLinePoints.size() < 1)
		{
			XiMessageBox("测量交点数据有误");
			return false;
		}

		tIntersection.x = tIntersectionPoint.dCoorX;
		tIntersection.y = tIntersectionPoint.dCoorY + dExAxisPos;
		tIntersection.z = tIntersectionPoint.dCoorZ;
	}
	else if (0 == tBoardInfo.vtCornerRelsve_Info.at(nIndex1).bBoardType
		|| 0 == tBoardInfo.vtCornerRelsve_Info.at(nIndex2).bBoardType)//直线曲线相交
	{
		T_ALGORITHM_POINT_2D tPointStart;
		T_ALGORITHM_POINT_2D tPointEnd;
		T_ALGORITHM_POINT_2D tCenter;
		double dRadius;
		T_ALGORITHM_POINT_2D tFirstIntersectionPoint;
		T_ALGORITHM_POINT_2D tSecondIntersectionPoint;
		vector<XI_POINT> vtFitOutputPoints, vtLinePoint;
		double dStepDis = 1.0;
		vector<XI_POINT> vtOutputPoints;
		if (!g_bLocalDebugMark)
		{
			/*for (int i = 0; i < vtTeachResult.size(); i++)
			{
				vtTeachResult.at(3).tKeyPtn3D.y += dExAxisPos;
			}*/
		}

		if (0 == tBoardInfo.vtCornerRelsve_Info.at(nIndex1).bBoardType
			&& 1 == tBoardInfo.vtCornerRelsve_Info.at(nIndex2).bBoardType)// 圆弧 前三个数据拟合圆心
		{
			for (int n = 0; n < 3; n++)
			{
				XI_POINT tp = { m_vtTeachResult.at(n).tKeyPtn3D.x,m_vtTeachResult.at(n).tKeyPtn3D.y ,m_vtTeachResult.at(n).tKeyPtn3D.z };
				vtFitOutputPoints.push_back(tp);
			}
			tStartp = m_vtTeachResult.at(3).tKeyPtn3D;
			tEndP = m_vtTeachResult.at(4).tKeyPtn3D;

			tPointStart.x = m_vtTeachResult.at(3).tKeyPtn3D.x;
			tPointStart.y = m_vtTeachResult.at(3).tKeyPtn3D.y;
			tPointEnd.x = m_vtTeachResult.at(4).tKeyPtn3D.x;
			tPointEnd.y = m_vtTeachResult.at(4).tKeyPtn3D.y;
		}
		else if (1 == tBoardInfo.vtCornerRelsve_Info.at(nIndex1).bBoardType
			&& 0 == tBoardInfo.vtCornerRelsve_Info.at(nIndex2).bBoardType)// 圆弧 前三个数据拟合圆心
		{
			for (int n = 2; n < 5; n++)
			{
				XI_POINT tp = { m_vtTeachResult.at(n).tKeyPtn3D.x,m_vtTeachResult.at(n).tKeyPtn3D.y ,m_vtTeachResult.at(n).tKeyPtn3D.z };
				vtFitOutputPoints.push_back(tp);
			}
			tStartp = m_vtTeachResult.at(0).tKeyPtn3D;
			tEndP = m_vtTeachResult.at(1).tKeyPtn3D;
			tPointStart.x = m_vtTeachResult.at(0).tKeyPtn3D.x;
			tPointStart.y = m_vtTeachResult.at(0).tKeyPtn3D.y;
			tPointEnd.x = m_vtTeachResult.at(1).tKeyPtn3D.x;
			tPointEnd.y = m_vtTeachResult.at(1).tKeyPtn3D.y;
		}
		// 拟合圆弧
		vtLinePoint.push_back(tStartp);
		vtLinePoint.push_back(tEndP);
		SaveDataXiPoint(vtFitOutputPoints, tBoardInfo.vtBoardPartInfo.at(nIndex2).strfileName + "ThoeryThreePoint.txt");
		SaveDataXiPoint(vtLinePoint, tBoardInfo.vtBoardPartInfo.at(nIndex2).strfileName + "ThoeryLineTwoPoint.txt");
		//bool CalCircleThreePoint(T_SPACE_CIRCLE_PARAM & tCircle, vector<XI_POINT> vtFitOutputPoints, double dStepDis, vector<XI_POINT>&vtOutputPoints);
		//bool CalThreeDotToCircle(const XI_POINT & tFirstPoint, const XI_POINT & tSecondPoint, const XI_POINT & tThirdPoint, T_SPACE_CIRCLE_PARAM & CircleParam);
		T_SPACE_CIRCLE_PARAM tCircleParam;
		if (CalCircleThreePoint(tCircleParam, vtFitOutputPoints, dStepDis, vtOutputPoints) || vtOutputPoints.size() < 3)
		{
			//已修改
			XUI::MesBox::PopInfo("{0}拟合圆弧失败", tBoardInfo.strfileName.c_str());
			//XiMessageBox("%s拟合圆弧失败", tBoardInfo.strfileName);
			return false;
		}
		tCenter.x = tCircleParam.tCenterPoint.dCoorX;
		tCenter.y = tCircleParam.tCenterPoint.dCoorY;
		dRadius = tCircleParam.dRadius;
		// 计算交点
		xiAlgorithm->CalLineArcTwoIntersectionPoint(tPointStart, tPointEnd, tCenter, dRadius,
			tFirstIntersectionPoint, tSecondIntersectionPoint);

		XI_POINT tP1 = { tFirstIntersectionPoint.x,tFirstIntersectionPoint.y,tCircleParam.tCenterPoint.dCoorZ };
		XI_POINT tP2 = { tSecondIntersectionPoint.x,tSecondIntersectionPoint.y,tCircleParam.tCenterPoint.dCoorZ };

		double dDisInter1 = TwoPointDis(tFirstIntersectionPoint.x, tFirstIntersectionPoint.y, tPointStart.x, tPointStart.y);
		double dDisInter2 = TwoPointDis(tSecondIntersectionPoint.x, tSecondIntersectionPoint.y, tPointStart.x, tPointStart.y);

		tIntersection = dDisInter1 < dDisInter2 ? tP1 : tP2;

		vector<XI_POINT> vtRealPoint;
		if (1 == tBoardInfo.vtCornerRelsve_Info.at(nIndex2).bBoardType)
		{
			// 截取两测量线段一段数据
			int nSizecircle = vtOutputPoints.size() - 1;
			int nIndex = 0, nStepNo = 0;
			double nDisPoint = 9999.0, dLength = 150.;

			while (true)
			{
				double dis = TwoPointDis(tIntersection.x, tIntersection.y, vtOutputPoints.at(nIndex).x, vtOutputPoints.at(nIndex).y);
				if (nDisPoint > dis)
				{
					nDisPoint = dis;
					nStepNo = nIndex;
				}
				nIndex++;
				//if (!bIsStart)// 终点
				//{
				   // if (nIndex == nSizecircle)
				   // {
				   //	 for (int j = 0; j < dLength; j++)
				   //	 {
				   //		 int nStp = (nStepNo + j) > nSizecircle ? (nStepNo + j - nSizecircle) : nStepNo + j;
				   //		 vtRealPoint.push_back(vtOutputPoints.at(nStp));
				   //	 }
				   //	 break;
				   // }
				//}
				//else
				{
					if (nIndex == nSizecircle)
					{
						for (int j = 0; j < dLength; j++)
						{
							int nStp = (nStepNo - j) < 0 ? nSizecircle - (nStepNo - j) : nStepNo - j;
							vtRealPoint.push_back(vtOutputPoints.at(nStp));
						}
						break;
					}
				}
			}
			vector<T_ROBOT_COORS> vtCoutCoors;
			CalcRobotCoorsAccordingTrack(vtRealPoint, vtCoutCoors);
			SaveDataRobotCoors(vtCoutCoors, tBoardInfo.vtBoardPartInfo.at(nIndex2).strfileName + "TrackStartCoors.txt");
		}
		// 输出测试结果
		vector<XI_POINT> vtTrack;
		GenerateTrack(vtTrack, tP1, tP2, tP2, 1.0, false);
		SaveDataXiPoint(vtTrack, tBoardInfo.vtBoardPartInfo.at(nIndex2).strfileName + "IntersectData.txt");
		GenerateTrack(vtTrack, tStartp, tEndP, tStartp, 1.0, false);
		SaveDataXiPoint(vtOutputPoints, tBoardInfo.vtBoardPartInfo.at(nIndex2).strfileName + "ThoryCircleData.txt");
		SaveDataXiPoint(vtTrack, tBoardInfo.vtBoardPartInfo.at(nIndex2).strfileName + "ThoryLine.txt");
		tIntersection.y += dExAxisPos;

	}
	else if (1 == tBoardInfo.vtCornerRelsve_Info.at(nIndex1).bBoardType
		&& 1 == tBoardInfo.vtCornerRelsve_Info.at(nIndex2).bBoardType) // 两曲线相交
	{
		T_ALGORITHM_POINT_2D tFirstIntersectionPoint;
		T_ALGORITHM_POINT_2D tSecondIntersecitonPoint;
		vector<XI_POINT> vtFitOutputPoints, vtFitOutputPoints2;
		double dStepDis = 1.0;
		vector<XI_POINT> vtOutputPoints, vtOutputPoints2;
		for (int n = 0; n < 6; n++)
		{
			XI_POINT tp = { m_vtTeachResult.at(n).tKeyPtn3D.x,m_vtTeachResult.at(n).tKeyPtn3D.y ,m_vtTeachResult.at(n).tKeyPtn3D.z };
			if (n < 3)
			{
				vtFitOutputPoints.push_back(tp);
			}
			else
			{
				vtFitOutputPoints2.push_back(tp);
			}
		}
		T_SPACE_CIRCLE_PARAM tCircleParam;
		T_SPACE_CIRCLE_PARAM tCircleParam2;
		if (!CalCircleThreePoint(tCircleParam, vtFitOutputPoints, dStepDis, vtOutputPoints)
			|| !CalCircleThreePoint(tCircleParam2, vtFitOutputPoints2, dStepDis, vtOutputPoints2))
		{
			XiMessageBox("拟合圆弧失败");
			return false;
		}

		T_ALGORITHM_POINT_2D tPointStart, tPointStart2;
		T_ALGORITHM_POINT_2D tPointEnd, tPointEnd2;
		T_ALGORITHM_POINT_2D tCenter, tCenter2;
		double dRadius, dRadius2;

		tCenter.x = tCircleParam.tCenterPoint.dCoorX;
		tCenter.y = tCircleParam.tCenterPoint.dCoorY;
		dRadius = tCircleParam.dRadius;

		tPointStart.x = tCenter.x + dRadius * CosD(0);
		tPointStart.y = tCenter.y + dRadius * SinD(0);
		tPointEnd = tPointStart;

		tCenter2.x = tCircleParam2.tCenterPoint.dCoorX;
		tCenter2.y = tCircleParam2.tCenterPoint.dCoorY;
		dRadius2 = tCircleParam2.dRadius;
		tPointStart2.x = tCenter2.x + dRadius2 * CosD(0);
		tPointStart2.y = tCenter2.y + dRadius2 * SinD(0);
		tPointEnd2 = tPointStart2;
		xiAlgorithm->CalArcArcIntersectionPoint(tPointStart, tPointEnd, tCenter, dRadius, tPointStart2, tPointEnd2, tCenter2, dRadius2,
			tFirstIntersectionPoint, tSecondIntersecitonPoint);
	}

	delete xiAlgorithm;
	return true;
}

/*
	 函数功能：计算测量数据
	 2023.10.15 郭嘉谊新增默认形参T_ROBOT_COORS *ptPreCoord  若有起始参考点位，可使用此参数 防止某轴转动过大
				不为nullptr时,暂时屏蔽先测后焊的CalcContinuePulseForWeld
 */
bool CDiaphragmWeld::CalcScanPos(CRobotDriverAdaptor* pRobotDriver, double dNormalAgl, XI_POINT tPoint, vector<T_ROBOT_COORS>& vtScanCoors, vector<T_ANGLE_PULSE>& vtScanPulses, vector<int>& vtScanCoorsType,
	double& dExAxisPos, bool bIsStart, bool bIsFixedPointScan, T_ANGLE_PULSE* ptPrePulse)
{
	vtScanCoors.clear();
	vtScanCoorsType.clear();
	vtScanPulses.clear();
	long long tTime = XI_clock();
	double dDownGunDisFromStrat = 0;
	COPini Opini;
	Opini.SetFileName("Data\\WeldParam.ini");
	Opini.SetSectionName("WeldParam");
	Opini.ReadString("StartPointScanDis", &dDownGunDisFromStrat);
	WriteLog("起始下枪扫描位置：%.3lf", dDownGunDisFromStrat);
	vector<XI_POINT> vtScanData;
	vector<T_ROBOT_COORS> vtScanPoint, vtScanPointTemp;
	vtScanPointTemp.clear();
	XI_POINT tScanPoint;
	double dAdjustAgl = 90; //1
	double dDiff = -100, dDiff2 = 200, dSafeDis = 10;
	if (E_CLOSEDARC == m_eWorkPieceType)//圆弧时扫描需要搜起点需整改角度，便于锁定
	{
		dDiff = -50;
		dDiff2 = 150;
		dSafeDis = 0;
	}
	if (bIsFixedPointScan)//定长搜索
	{
		dDiff = -150;
		dDiff2 = 150;
		dDownGunDisFromStrat = 0;
	}
	if (bIsStart)
	{
		CalcOffsetInBase(tPoint, dSafeDis, 25 * m_nRobotInstallDir, dNormalAgl, tPoint, false);
		CalcOffsetInBase(tPoint, dDiff - dDownGunDisFromStrat, 0.0, dNormalAgl + dAdjustAgl * m_nRobotInstallDir, tScanPoint, false);
		vtScanData.push_back(tScanPoint);
		CalcOffsetInBase(tScanPoint, dDiff2, 0.0, dNormalAgl + dAdjustAgl * m_nRobotInstallDir, tScanPoint, false);
		vtScanData.push_back(tScanPoint);
	}
	else
	{
		//dDiff = -130;
		//dDiff2 = 300;
		CalcOffsetInBase(tPoint, dSafeDis, 25 * m_nRobotInstallDir, dNormalAgl, tPoint, false);
		CalcOffsetInBase(tPoint, dDiff - dDownGunDisFromStrat, 0, dNormalAgl - dAdjustAgl * m_nRobotInstallDir, tScanPoint, false);
		vtScanData.push_back(tScanPoint);
		CalcOffsetInBase(tScanPoint, dDiff2, 0.0, dNormalAgl - dAdjustAgl * m_nRobotInstallDir, tScanPoint, false);
		vtScanData.push_back(tScanPoint);
	}
	if (E_CLOSEDARC == m_eWorkPieceType)//圆弧时扫描需要搜起点需整改角度，便于锁定
	{
		dNormalAgl += 5.0 * m_nRobotInstallDir;
	}
	double dRZ = pRobotDriver->DirAngleToRz(dNormalAgl);
	vtScanPoint.clear();

	for (int i = 0; i < vtScanData.size(); i++)
	{
//		E_ROBOT_CAR_COORS_ERROR tMoveDir;
		T_ROBOT_COORS dCoor(vtScanData.at(i).x, vtScanData.at(i).y, vtScanData.at(i).z, m_dPlatWeldRx, m_dPlatWeldRy, dRZ, 0, 0, 0), dOutCoor;
		if (!CalcRobotAndMachinePos(pRobotDriver, dCoor, dOutCoor))
		{
			return false;
		}
		//vtScanPointTemp.push_back(dOutCoor);
		T_ROBOT_COORS tMagneticCoors = dOutCoor, tGunCoorsMoveMagnetic;
		if (!pRobotDriver->MoveToolByWeldGun(dOutCoor, pRobotDriver->m_tTools.tGunTool, tMagneticCoors, m_ptUnit->GetCameraTool(m_ptUnit->m_nTrackCameraNo), tGunCoorsMoveMagnetic))
		{
			return false;
		}
		tGunCoorsMoveMagnetic.dY += tGunCoorsMoveMagnetic.dBY;
		tGunCoorsMoveMagnetic.dBY = 0;
		if (!CalcRobotAndMachinePos(pRobotDriver, tGunCoorsMoveMagnetic, dOutCoor))
		{
			return false;
		}
		vtScanPoint.push_back(dOutCoor);
	}
	// 防止扫描极限
	bool bReverse = vtScanPoint.at(0).dBY > vtScanPoint.at(1).dBY ? true : false;
	for (int i = vtScanPoint.size() - 1; i >= 0; i--)
	{
		if (!bReverse)
		{
			if (i < vtScanPoint.size() - 1)
			{
				vtScanPoint.at(i).dY += vtScanPoint.at(i).dBY - vtScanPoint.at(vtScanPoint.size() - 1).dBY;
				vtScanPoint.at(i).dBY = vtScanPoint.at(vtScanPoint.size() - 1).dBY;
			}
		}
		else
		{
			if (i > 0)
			{
				vtScanPoint.at(i).dY += vtScanPoint.at(i).dBY - vtScanPoint.at(0).dBY;
				vtScanPoint.at(i).dBY = vtScanPoint.at(0).dBY;
			}
		}

		T_ROBOT_COORS dCoor = vtScanPoint.at(i);
		vtScanCoors.push_back(dCoor);
		vtScanCoorsType.push_back(E_DOUBLE_LONG_LINE);
	}
	dExAxisPos = vtScanCoors.at(0).dBY;
	// 添加安全位置
	T_ROBOT_COORS dCoor = vtScanCoors.at(0);
	dCoor.dZ = m_dWorkPieceHighTop + 20 * m_nRobotInstallDir;
	vtScanCoors.insert(vtScanCoors.begin(), dCoor);
	vtScanCoorsType.insert(vtScanCoorsType.begin(), E_TRANSITION_POINT);
	dCoor = vtScanCoors.at(vtScanCoors.size() - 1);
	dCoor.dZ = m_dWorkPieceHighTop + 20 * m_nRobotInstallDir;
	vtScanCoors.push_back(dCoor);
	vtScanCoorsType.push_back(E_TRANSITION_POINT);
	reverse(vtScanCoors.begin(), vtScanCoors.end());
	reverse(vtScanCoorsType.begin(), vtScanCoorsType.end());
	//计算连续可运行脉冲坐标

	/*for (int n=0;n<vtScanCoors.size();n++)
	{
		vtScanCoors.at(n).dRX += -10;
		vtScanCoors.at(n).dRY += -5;
	}*/
	//添加一个机器人的当前坐标，用于转换连续运动脉冲
	vtScanPulses.resize(vtScanCoors.size());
	if (ptPrePulse != nullptr)
	{
		T_ANGLE_PULSE tBackupPulse = *ptPrePulse;
		for (int i = 0; i < vtScanCoors.size(); i++)
		{
			if (!pRobotDriver->RobotInverseKinematics(
				vtScanCoors[i],
				*ptPrePulse,
				pRobotDriver->m_tTools.tGunTool,
				vtScanPulses[i])
				)
			{
				XiMessageBoxOk("带有参考坐标的关节坐标逆解失败");
				return false;
			}
			*ptPrePulse = vtScanPulses[i];
		}
		// 指针操作需要还原初始值
		*ptPrePulse = tBackupPulse;
		if (vtScanPulses.size() < 3)
		{
			XiMessageBoxOk("带有参考坐标的关节坐标逆解失败,轨迹点数少于3");
			return false;
		}
		for (int i = 1; i < vtScanPulses.size() - 3; i++)
		{
			if (pRobotDriver->m_cXiRobotAbsCoorsTrans->GetAnglePulseDisNew(vtScanPulses[i], vtScanPulses[i + 1]) >= 15.0)
			{
				XiMessageBoxOk("带有参考坐标的关节坐标逆解失败,测量轨迹点姿态变化过大");
				return false;
			}
		}
	}
	else
	{
		CHECK_BOOL_RETURN(CalcContinuePulseForWeld(vtScanCoors, vtScanPulses, false));
	}

	return true;
}

bool CDiaphragmWeld::SetScanPos(CRobotDriverAdaptor* pRobotDriver, vector<T_ROBOT_COORS> vtCooors, vector<T_ANGLE_PULSE> vtPulses, vector<int> vTeachType)
{
	if (vtCooors.size() < 4 || vtCooors.size() != vtPulses.size())
	{
		//已修改
		XUI::MesBox::PopInfo("搜索端点数据有误{0}", vtCooors.size());
		//XiMessageBox("搜索端点数据有误:%d", vtCooors.size());
		return false;
	}
	int nIndex = 0;
	for (int i = 0; i < vtCooors.size(); i++)
	{
		if (E_TRANSITION_POINT & vTeachType[i])
		{
			continue;
		}
		else
		{
			T_ROBOT_COORS tCoor = vtCooors.at(i);
			pRobotDriver->SetPosVar(7 + nIndex, tCoor);
			nIndex++;
		}
	}
	 // 移动到安全位置
	 if (!PosMoveMachine(pRobotDriver, vtCooors.at(0).dBY, m_gExAxisSpeed/*m_dSafePosRunSpeed * 3*/, COORD_ABS))
	 {
		 return false;
	 }

	 vector<T_ROBOT_MOVE_INFO> vtRobotMoveInfo(0);
	 T_ROBOT_MOVE_INFO tRobotMoveInfo;
	 T_ROBOT_MOVE_SPEED tPulseMove(m_dSafePosRunSpeed, 50, 50);

	 tRobotMoveInfo = pRobotDriver->PVarToRobotMoveInfo(0, vtCooors.at(0), tPulseMove, MOVJ);
	 vtRobotMoveInfo.push_back(tRobotMoveInfo);
	 tPulseMove.dSpeed = m_dSafePosRunSpeed;
	 tRobotMoveInfo = pRobotDriver->PVarToRobotMoveInfo(0, vtCooors.at(1), tPulseMove, MOVL);
	 vtRobotMoveInfo.push_back(tRobotMoveInfo);

	 pRobotDriver->SetMoveValue(vtRobotMoveInfo);
	 pRobotDriver->CallJob("CONTIMOVANY");
	 CHECK_BOOL_RETURN(m_ptUnit->CheckRobotDone(vtCooors.at(1)));

	 return true;
 }

//bool CDiaphragmWeld::SetScanPos(CRobotDriverAdaptor* pRobotDriver, double dNormalAgl, XI_POINT tPoint, bool bIsStart, bool bIsFixedPointScan)
//{
//	long long tTime = XI_clock();
//	double dDownGunDisFromStrat = 0;
//	COPini Opini;
//	Opini.SetFileName("Data\\WeldParam.ini");
//	Opini.SetSectionName("WeldParam");
//	Opini.ReadString("StartPointScanDis", &dDownGunDisFromStrat);
//	WriteLog("起始下枪扫描位置：%.3lf", dDownGunDisFromStrat);
//	vector<XI_POINT> vtScanData;
//	vector<T_ROBOT_COORS> vtScanPoint, vtScanPointTemp;
//	vtScanPointTemp.clear();
//	XI_POINT tScanPoint;
//	double dAdjustAgl = 90; //1
//	double dDiff = -100, dDiff2 = 200, dSafeDis = 20;
//	if (E_CLOSEDARC == m_eWorkPieceType)//圆弧时扫描需要搜起点需整改角度，便于锁定
//	{
//		dDiff = -50;
//		dDiff2 = 150;
//		dSafeDis = 0;
//	}
//	if (bIsFixedPointScan)//定长搜索
//	{
//		dDiff = -150;
//		dDiff2 = 150;
//		dDownGunDisFromStrat = 0;
//	}
//	if (bIsStart)
//	{
//		CalcOffsetInBase(tPoint, dSafeDis, 30 * m_nRobotInstallDir, dNormalAgl, tPoint, false);
//		CalcOffsetInBase(tPoint, dDiff, 0.0, dNormalAgl + dAdjustAgl * m_nRobotInstallDir, tScanPoint, false);
//		vtScanData.push_back(tScanPoint);
//		CalcOffsetInBase(tScanPoint, dDiff2, 0.0, dNormalAgl + dAdjustAgl * m_nRobotInstallDir, tScanPoint, false);
//		vtScanData.push_back(tScanPoint);
//	}
//	else
//	{
//		dDiff = -130;
//		dDiff2 = 300;
//		CalcOffsetInBase(tPoint, dSafeDis, 30 * m_nRobotInstallDir, dNormalAgl, tPoint, false);
//		CalcOffsetInBase(tPoint, dDiff + dDownGunDisFromStrat, 0, dNormalAgl - dAdjustAgl * m_nRobotInstallDir, tScanPoint, false);
//		vtScanData.push_back(tScanPoint);
//		CalcOffsetInBase(tScanPoint, dDiff2, 0.0, dNormalAgl - dAdjustAgl * m_nRobotInstallDir, tScanPoint, false);
//		vtScanData.push_back(tScanPoint);
//	}
//	if (E_CLOSEDARC == m_eWorkPieceType)//圆弧时扫描需要搜起点需整改角度，便于锁定
//	{
//		dNormalAgl += 5.0 * m_nRobotInstallDir;
//	}
//	double dRZ = pRobotDriver->DirAngleToRz(dNormalAgl);
//	vtScanPoint.clear();
//	for (int i = 0; i < vtScanData.size(); i++)
//	{
////		E_ROBOT_CAR_COORS_ERROR tMoveDir;
//		T_ROBOT_COORS dCoor(vtScanData.at(i).x, vtScanData.at(i).y, vtScanData.at(i).z, m_dPlatWeldRx, m_dPlatWeldRy, dRZ, 0, 0, 0), dOutCoor;
//		if (!CalcRobotAndMachinePos(pRobotDriver, dCoor, dOutCoor))
//		{
//			return false;
//		}
//		//vtScanPointTemp.push_back(dOutCoor);
//		T_ROBOT_COORS tMagneticCoors = dOutCoor, tGunCoorsMoveMagnetic;
//		if (!pRobotDriver->MoveToolByWeldGun(dOutCoor, pRobotDriver->m_tTools.tGunTool, tMagneticCoors, m_ptUnit->GetCameraTool(m_ptUnit->m_nTrackCameraNo), tGunCoorsMoveMagnetic))
//		{
//			return false;
//		}
//		tGunCoorsMoveMagnetic.dY += tGunCoorsMoveMagnetic.dBY;
//		tGunCoorsMoveMagnetic.dBY = 0;
//		if (!CalcRobotAndMachinePos(pRobotDriver, tGunCoorsMoveMagnetic, dOutCoor))
//		{
//			return false;
//		}
//		vtScanPoint.push_back(dOutCoor);
//	}
//	for (int i = vtScanPoint.size() - 1; i >= 0; i--)
//	{
//		if (i < vtScanPoint.size() - 1)
//		{
//			vtScanPoint.at(i).dY += vtScanPoint.at(i).dBY - vtScanPoint.at(vtScanPoint.size() - 1).dBY;
//			vtScanPoint.at(i).dBY = vtScanPoint.at(vtScanPoint.size() - 1).dBY;
//		}
//		T_ROBOT_COORS dCoor = vtScanPoint.at(i);
//		pRobotDriver->SetPosVar(7 + i, dCoor);
//	}
//	if (IDOK == XiMessageBoxPopup("开始移动到测量点位置", *m_pIsNaturalPop))
//	{
//		T_ROBOT_MOVE_SPEED tPulseMove2(m_dSafePosRunSpeed, 20, 20);
//         T_ROBOT_MOVE_SPEED tPulseMove ( m_dFastApproachToWorkSpeed,20,20 );
//		T_ROBOT_COORS tRobotCoor = vtScanPoint.at(0);
//		tRobotCoor.dZ += 200 * m_nRobotInstallDir;
//		T_ANGLE_PULSE tReference, tResult;
//		if (E_CLOSEDARC == m_eWorkPieceType)
//		{
//			T_ANGLE_PULSE tRefer(-71800, -84070, -54050, 0, 78750, -83400, 0, 0, 0);
//			tReference = tRefer;
//		}
//		else
//		{
//			GetTheoryReferencePulse(pRobotDriver, tRobotCoor, tReference);
//		}
//
//		if (!pRobotDriver->RobotInverseKinematics(tRobotCoor, tReference, pRobotDriver->m_tTools.tGunTool, tResult))
//		{
//			XiMessageBox("测量点目标点机械臂超出极限");
//			return false;
//		}
//		if (!PosMoveMachine(pRobotDriver, tResult.lBYPulse * m_ptUnit->GetExPulseEquivalent(m_ptUnit->m_nTrackAxisNo), tPulseMove2.dSpeed * 3, COORD_ABS))
//		{
//			return false;
//		}
//		//pRobotDriver->SafeMoveJByJobSaxis(tResult, tPulseMove);
//		if (!m_ptUnit->CheckRobotDone(tResult))
//		{
//			return false;
//		}
//		//CHECK_BOOL_RETURN(m_ptUnit->CheckRobotDone( tResult));
//		tRobotCoor.dZ -= 200 * m_nRobotInstallDir;
//		if (!MoveByJob(pRobotDriver, tRobotCoor, tPulseMove, pRobotDriver->m_nExternalAxleType, "MOVJ"))
//		{
//			return false;
//		}
//		//设置扫描位置
//		//for (int i = 0; i < vtScanPoint.size(); i++)
//		//{
//		//    if (i>0)
//		//    {
//		//        vtScanPoint.at(i).dY += vtScanPoint.at(i).dBY - vtScanPoint.at(0).dBY;
//		//    }
//		//    T_ROBOT_COORS dCoor = vtScanPoint.at(i);
//		//    pRobotDriver->SetPosVar(7 + i, dCoor);
//		//}
//		RecordRunTime(pRobotDriver, tTime, "移动到搜起点位置");
//	}
//	else
//	{
//		return false;
//	}
//	return true;
//}


bool CDiaphragmWeld::CalcLineLineIntersection(XI_POINT tStart, double dStartAngle, XI_POINT tEnd, double dEndAngle, XI_POINT& intersection)
{
	double slope1 = TanD(dStartAngle), slope2 = TanD(dEndAngle);
	if (fabs(slope1) == fabs(slope2))
	{
		XiMessageBox("两直线平行，无交点");
		return false;
	}
	// 计算交点的 x 坐标
	intersection.x = (tEnd.y - tStart.y + slope1 * tStart.x - slope2 * tEnd.x) / (slope1 - slope2);
	// 使用其中一条直线的方程计算交点的 y 坐标
	intersection.y = tStart.y + slope1 * (intersection.x - tStart.x);
	intersection.z = (tStart.z + tEnd.z) / 2;
	return true;
}

bool CDiaphragmWeld::CalcLineLineIntersection(T_ROBOT_COORS tStart, double dStartAngle, T_ROBOT_COORS tEnd, double dEndAngle, XI_POINT& intersection)
{
	double slope1 = TanD(dStartAngle), slope2 = TanD(dEndAngle);
	if (fabs(slope1) == fabs(slope2))
	{
		XiMessageBox("两直线平行，无交点");
		return false;
	}
	// 计算交点的 x 坐标
	intersection.x = (tEnd.dY - tStart.dY + slope1 * tStart.dX - slope2 * tEnd.dX) / (slope1 - slope2);
	// 使用其中一条直线的方程计算交点的 y 坐标
	intersection.y = tStart.dY + slope1 * (intersection.x - tStart.dX);
	intersection.z = (tStart.dZ + tEnd.dZ) / 2;
	return true;
}

T_BOARD_INFO_NEW CDiaphragmWeld::GetBoardInfo(CRobotDriverAdaptor* pRobotDriver, T_CORNER_LINE_INFO_NEW tCornerLineInfo,
	VT_CORNER_LINE_INFO_NEW* pvtCornerLineInfo, VT_ENDPOINT_INFO_NEW* pvtPointInfo, VT_CIRCLE_CENTER_INFO_NEW* pvtCenterInfo, CString strFile)
{
	/*类型：工件起终点形状,起终点状态：起终点获取方式：
   **1.直线：起点自由，终点自由：未被更新：大于1500，提前搜索起点，中间跟踪，跟踪过程搜索终点。
								 小于1500大于500，提前搜索起点终点，中间跟踪。
								 小于500，提前搜索起点终点，中间加测量点先测后焊
   **2.直线：起点自由，终点自由：被更新：大于1500，提前搜索起点，中间跟踪，跟踪过程搜索终点。
								 小于1500大于500，不再搜索起终点，中间加跟踪。
								 小于500，不再搜索起终点，中间加测量点先测后焊。
   **3.直线：起点自由，终点干涉：未被更新：大于1500，提前搜索起点，测量终点，中间跟踪。
								 小于1500大于500，提前搜索起点，测量终点，中间跟踪。
								 小于500，提前搜索起点，测量终点，中间加测量点先测后焊。
   **4.直线：起点干涉，终点自由：未被更新：大于1500，测量起点得到起始段数据，中间跟踪，跟踪过程搜索终点。
								 小于1500大于500，测量起点，搜索终点，中间跟踪。
								 小于500，测量起点，搜索终点，中间加测量点先测后焊。
   **5.直线：两端干涉：未被更新：大于1500，测量起点得到起始段数据，测量终点，中间跟踪。
								 小于1500大于500，测量起点得到起始段数据，测量终点，中间跟踪。
								 小于500，测量起点，测量终点，中间加测量点先测后焊。
   暂未更新完成
   */

   /*1.直线小于300mm先测后焊，大于300小于1500先测起终点中间跟踪，大于1500，搜索起始点跟踪过程搜索结尾点
   **2.非闭合曲线长度大于200小于1500，先测起终点中间跟踪，大于1500，搜索起始点跟踪过程搜索结尾点
   */

   // 直线过程跟踪过程，起终点测量或搜索得到，中间不测量。弧线过程跟踪，起终点被起点搜索得到，终点测量或搜索得到可直接跟踪焊接
   // 若起点为测量得到或被更新得到不可直接跟踪焊接需要获取起始段数据方可焊接（可改变姿态测量靠近起始段数据进行焊接）
	T_BOARD_INFO_NEW tBoardInfo;
	tBoardInfo.vbBoardDoesItExist.clear();
	tBoardInfo.vtCornerRelsve_Info.clear();
	tBoardInfo.vtCenter_Info.clear();
	tBoardInfo.vtEndpoint_info.clear();

	if (1 == tCornerLineInfo.nProperty)//加强圈
	{
		tBoardInfo.vtCornerRelsve_Info.push_back(tCornerLineInfo);
		T_CORNER_LINE_INFO_NEW tCorner = tCornerLineInfo;
		while (true)//查找加强圈关联焊缝
		{
			tCorner = FindCornerLineInfoFun(tCorner.bEndContiguousCor, pvtCornerLineInfo);
			if (tCorner.nBoardNo == tCornerLineInfo.nBoardNo)
			{
				pRobotDriver->m_cLog->Write("闭合加强圈");
				tBoardInfo.bIsClosedRing = true;
				break;
			}
			if (-1 == tCorner.bEndContiguousCor)
			{
				pRobotDriver->m_cLog->Write("非闭合");
				break;
			}
			tBoardInfo.vtCornerRelsve_Info.push_back(tCorner);//焊缝
		}
		//添加关联端点
		for (int n = 0; n < tBoardInfo.vtCornerRelsve_Info.size(); n++)
		{
			T_ENDPOINT_INFO_NEW tSp = FindEndPointInfoFun(tBoardInfo.vtCornerRelsve_Info.at(n).nStartNo, pvtPointInfo);
			T_ENDPOINT_INFO_NEW tEp = FindEndPointInfoFun(tBoardInfo.vtCornerRelsve_Info.at(n).nEndNo, pvtPointInfo);
			VT_ENDPOINT_INFO_NEW vtEp;
			vtEp.push_back(tSp);
			vtEp.push_back(tEp);
			if (1 == tBoardInfo.vtCornerRelsve_Info.at(n).bBoardType)//圆弧
			{
				m_pTraceModel->m_dWeldLen += tBoardInfo.vtCornerRelsve_Info.at(n).dBoardLength;
				T_CIRCLE_CENTER_INFO_NEW tCenterPoint = FindCenterPointInfoFun(tBoardInfo.vtCornerRelsve_Info.at(n).nBoardNo, pvtCenterInfo);
				double dRadius = TwoPointDis(tCenterPoint.dCenterX, tCenterPoint.dCenterY, tCenterPoint.dCenterZ,
					tSp.tPointData.x, tSp.tPointData.y, tSp.tPointData.z);
				if (dRadius > 200)//可以跟踪
				{
					tBoardInfo.bTracking = true;
					tBoardInfo.bIsMiddleMeasure = false;
				}
				tBoardInfo.vtCenter_Info.push_back(tCenterPoint);//圆心
			}
			tBoardInfo.vtEndpoint_info.push_back(vtEp);//端点
		}
		return tBoardInfo;
	}
	XI_POINT intersection;
	m_pTraceModel->m_dWeldLen = tCornerLineInfo.dBoardLength;
	T_CORNER_LINE_INFO_NEW tCornerLineStartReleve, tLineStartReleve2, tCornerLineEndReleve, tLineEndReleve2;
	if (tCornerLineInfo.bStartContiguousCor != -1)// 起始关联
	{
		tCornerLineStartReleve = FindCornerLineInfoFun(tCornerLineInfo.bStartContiguousCor, pvtCornerLineInfo);
		if (tCornerLineStartReleve.bStartContiguousCor != -1)// 起始关联的关联
		{
			tLineStartReleve2 = FindCornerLineInfoFun(tCornerLineStartReleve.bStartContiguousCor, pvtCornerLineInfo);
			tBoardInfo.vbBoardDoesItExist.push_back(true);

		}
		else
		{
			tBoardInfo.vbBoardDoesItExist.push_back(false);
		}
		tBoardInfo.vbBoardDoesItExist.push_back(true);
	}
	else
	{
		tBoardInfo.vbBoardDoesItExist.push_back(false);
		tBoardInfo.vbBoardDoesItExist.push_back(false);
	}
	tBoardInfo.vbBoardDoesItExist.push_back(true);
	if (tCornerLineInfo.bEndContiguousCor != -1)
	{
		tCornerLineEndReleve = FindCornerLineInfoFun(tCornerLineInfo.bEndContiguousCor, pvtCornerLineInfo);
		tBoardInfo.vbBoardDoesItExist.push_back(true);
		if (tCornerLineEndReleve.bEndContiguousCor != -1)
		{
			tLineEndReleve2 = FindCornerLineInfoFun(tCornerLineEndReleve.bEndContiguousCor, pvtCornerLineInfo);
			tBoardInfo.vbBoardDoesItExist.push_back(true);
		}
		else
		{
			tBoardInfo.vbBoardDoesItExist.push_back(false);
		}
	}
	else
	{
		tBoardInfo.vbBoardDoesItExist.push_back(false);
		tBoardInfo.vbBoardDoesItExist.push_back(false);
	}
	// 五条焊缝，第一条和第五条不参与焊接只辅助第二条和倒数第二条焊缝测量
	tBoardInfo.vtCornerRelsve_Info.push_back(tLineStartReleve2); // 起始关联的焊缝起始关联		可以 顺口溜
	tBoardInfo.vtCornerRelsve_Info.push_back(tCornerLineStartReleve); // 起始关联
	tBoardInfo.vtCornerRelsve_Info.push_back(tCornerLineInfo); // 自身
	tBoardInfo.vtCornerRelsve_Info.push_back(tCornerLineEndReleve);// 结尾关联
	tBoardInfo.vtCornerRelsve_Info.push_back(tLineEndReleve2);// 结尾关联的哈能结尾关联
	for (int n = 0; n < tBoardInfo.vtCornerRelsve_Info.size(); n++)
	{
		T_BOARD_PART_INFO tboardPartInfo;
		tboardPartInfo.strfileName = strFile;
		tboardPartInfo.bTracking = false;
		tboardPartInfo.eGetStartMethod = E_NO_OPERATION;
		tboardPartInfo.bFixedPointScanStart = false;
		tboardPartInfo.eGetEndMethod = E_NO_OPERATION;
		tboardPartInfo.bFixedPointScanEnd = false;
		tboardPartInfo.bIsTrackingScanEnd = false;
		tboardPartInfo.bIsMiddleMeasure = true;
		tboardPartInfo.bIsClosedRing = false;
		tboardPartInfo.eWarpType = E_WRAPANGLE_EMPTY_SINGLE;

		string str = tboardPartInfo.strfileName.substr(0, tboardPartInfo.strfileName.find_last_of("\\"));
		CString strFile, str2;
		strFile.Format("%s\\%d-TheroyTrack.txt", str.c_str(), tBoardInfo.vtCornerRelsve_Info.at(n).nBoardNo);
		str2.Format("%s\\%d-", str.c_str(), tBoardInfo.vtCornerRelsve_Info.at(n).nBoardNo);
		tboardPartInfo.strfileName = str2.GetBuffer(0);
		vector<XI_POINT> vtTrack;
		T_ENDPOINT_INFO_NEW tSp, tEp;
		T_CIRCLE_CENTER_INFO_NEW tCenterPoint;// 圆心
		VT_ENDPOINT_INFO_NEW vEp;
		vector<T_ROBOT_COORS> vtCoutCoors;
		if (!tBoardInfo.vbBoardDoesItExist.at(n))
		{
			tBoardInfo.vtBoardPartInfo.push_back(tboardPartInfo);
			tBoardInfo.vtCenter_Info.push_back(tCenterPoint);//圆心
			tBoardInfo.vtEndpoint_info.push_back(vEp);//端点
			tBoardInfo.vvtheoryTrack.push_back(vtTrack);
			tBoardInfo.vvtheoryTrackRobotCoors.push_back(vtCoutCoors);
			continue;
		}
		// 包角类型判断
		if (!tBoardInfo.vtCornerRelsve_Info.at(n).bIsDoubleWeld && 0 == tBoardInfo.vtCornerRelsve_Info.at(n).nStartIfWrap)
		{
			tboardPartInfo.eWarpType = E_WRAPANGLE_EMPTY_SINGLE;
		}
		else if (tBoardInfo.vtCornerRelsve_Info.at(n).bIsDoubleWeld && 0 == tBoardInfo.vtCornerRelsve_Info.at(n).nStartIfWrap)
		{
			tboardPartInfo.eWarpType = E_WRAPANGLE_EMPTY_DOUBLE;
		}
		else if (!tBoardInfo.vtCornerRelsve_Info.at(n).bIsDoubleWeld && 2 == tBoardInfo.vtCornerRelsve_Info.at(n).nStartIfWrap)
		{
			tboardPartInfo.eWarpType = E_WRAPANGLE_TWO_SINGLE;
		}
		else if (tBoardInfo.vtCornerRelsve_Info.at(n).bIsDoubleWeld && 2 == tBoardInfo.vtCornerRelsve_Info.at(n).nStartIfWrap)
		{
			tboardPartInfo.eWarpType = E_WRAPANGLE_TWO_DOUBLE;
		}
		else if (1 == tBoardInfo.vtCornerRelsve_Info.at(n).nStartIfWrap)
		{
			tboardPartInfo.eWarpType = E_WRAPANGLE_ONCE;
		}
		m_pTraceModel->m_eStartWrapAngleType = tboardPartInfo.eWarpType;
		tSp = FindEndPointInfoFun(tBoardInfo.vtCornerRelsve_Info.at(n).nStartNo, pvtPointInfo);
		if (1 == tBoardInfo.vtCornerRelsve_Info.at(n).nStartType)// 干涉端检查是否被更新，更新后使用更新端点计算测量数据
		{
			ReadCorrectEndPointFun(pRobotDriver, tBoardInfo.vtCornerRelsve_Info.at(n).nStartNo, tSp.tPointData);
		}
		tEp = FindEndPointInfoFun(tBoardInfo.vtCornerRelsve_Info.at(n).nEndNo, pvtPointInfo);
		if (1 == tBoardInfo.vtCornerRelsve_Info.at(n).nStartType)
		{
			ReadCorrectEndPointFun(pRobotDriver, tBoardInfo.vtCornerRelsve_Info.at(n).nEndNo, tEp.tPointData);
		}
		if (1 == tBoardInfo.vtCornerRelsve_Info.at(n).bBoardType)
		{
			//计算交点
			CalcLineLineIntersection(tSp.tPointData, tBoardInfo.vtCornerRelsve_Info.at(n).dStartAngle,
				tEp.tPointData, tBoardInfo.vtCornerRelsve_Info.at(n).dEndAngle, intersection);
			// 更新圆心坐标
			CorrectCenterPointInfoFun(tBoardInfo.vtCornerRelsve_Info.at(n).nBoardNo, intersection, pvtCenterInfo);
			// 理论轨迹拟合
			GenerateTrack(vtTrack, tSp.tPointData, tBoardInfo.vtCornerRelsve_Info.at(n).dStartAngle,
				tEp.tPointData, tBoardInfo.vtCornerRelsve_Info.at(n).dEndAngle, intersection, 1.0, true);

			tCenterPoint = FindCenterPointInfoFun(tBoardInfo.vtCornerRelsve_Info.at(n).nBoardNo, pvtCenterInfo);

		}
		else
		{
			XI_POINT tct = { 0,0,0 };
			GenerateTrack(vtTrack, tSp.tPointData, tEp.tPointData, tct);
		}
		tBoardInfo.vtCenter_Info.push_back(tCenterPoint);//圆心
		tBoardInfo.vtCornerRelsve_Info.at(n).dBoardLength = vtTrack.size();
		m_pTraceModel->m_dWeldLen = tBoardInfo.vtCornerRelsve_Info.at(n).dBoardLength;

		CalcRobotCoorsAccordingTrack(vtTrack, vtCoutCoors);
		SaveDataRobotCoors(vtCoutCoors, strFile.GetBuffer(0));

		vEp.push_back(tSp);
		vEp.push_back(tEp);
		tBoardInfo.vtEndpoint_info.push_back(vEp);//端点
		tBoardInfo.vvtheoryTrack.push_back(vtTrack);
		tBoardInfo.vvtheoryTrackRobotCoors.push_back(vtCoutCoors);

		if (tBoardInfo.vtCornerRelsve_Info.at(n).dBoardLength > 300 && m_dHandEyeDis > 50)
		{
			//是否开启跟踪
			tboardPartInfo.bTracking = true;
		}
		if (0 == tBoardInfo.vtCornerRelsve_Info.at(n).nStartType &&
			tBoardInfo.vtCornerRelsve_Info.at(n).dBoardLength > m_dMeasureDisThreshold * 2 + m_dHandEyeDis &&
			tBoardInfo.vtCornerRelsve_Info.at(n).dBoardLength > 150)
		{
			tboardPartInfo.eGetStartMethod = E_SEARCH_ENDPOINT;
		}
		else if (0 == tBoardInfo.vtCornerRelsve_Info.at(n).nStartType &&
			1 == tBoardInfo.vtCornerRelsve_Info.at(n).nEndType)
		{
			tboardPartInfo.eGetStartMethod = E_CORRECT_ENDPOINT;
		}
		else if (1 == tBoardInfo.vtCornerRelsve_Info.at(n).nStartType)
		{
			tboardPartInfo.eGetStartMethod = E_MEASURE_ENDPOINT;
		}
		else if (4 == tBoardInfo.vtCornerRelsve_Info.at(n).nStartType)
		{
			tboardPartInfo.eGetStartMethod = E_MEASURE_PROJPOINT;
		}
		 if (0 == tBoardInfo.vtCornerRelsve_Info.at(n).nEndType && 
			 tBoardInfo.vtCornerRelsve_Info.at(n).dBoardLength > m_dMeasureDisThreshold * 2 + m_dHandEyeDis &&
			 tBoardInfo.vtCornerRelsve_Info.at(n).dBoardLength > 150)
		 {
			 tboardPartInfo.eGetEndMethod = E_SEARCH_ENDPOINT;
			//if (tBoardInfo.vtCornerRelsve_Info.at(n).dBoardLength > 3000)//开启跟踪过程搜索结尾点
			// {
			//	 m_pTraceModel->m_bIsTrackingScanEnd = true;
			//	 tboardPartInfo.bIsTrackingScanEnd = true;
			//	 tboardPartInfo.eGetEndMethod = E_NO_OPERATION;
			// }
		 }
		 else if (0 == tBoardInfo.vtCornerRelsve_Info.at(n).nEndType&&
			 1 == tBoardInfo.vtCornerRelsve_Info.at(n).nStartType)
		 {
			 tboardPartInfo.eGetEndMethod = E_CORRECT_ENDPOINT;
		 }
		 else if (1 == tBoardInfo.vtCornerRelsve_Info.at(n).nEndType)
		 {
			 tboardPartInfo.eGetEndMethod = E_MEASURE_ENDPOINT;
		 }
		 if (!ReadCorrectEndPointFun(pRobotDriver, tBoardInfo.vtCornerRelsve_Info.at(n).nStartNo, tboardPartInfo.tStartPoint))
		 {
			 T_ENDPOINT_INFO_NEW tp = FindEndPointInfoFun(tBoardInfo.vtCornerRelsve_Info.at(n).nStartNo, pvtPointInfo);
			 tboardPartInfo.tStartPoint = tp.tPointData;
		 }
		 else
		 {
			 tboardPartInfo.eGetStartMethod = E_UPDATED_ENDPOINT;
		 }
		 if (!ReadCorrectEndPointFun(pRobotDriver, tBoardInfo.vtCornerRelsve_Info.at(n).nEndNo, tboardPartInfo.tEndPoint))
		 {
			 T_ENDPOINT_INFO_NEW tp = FindEndPointInfoFun(tBoardInfo.vtCornerRelsve_Info.at(n).nEndNo, pvtPointInfo);
			 tboardPartInfo.tEndPoint = tp.tPointData;
		 }
		 else
		 {
			 tboardPartInfo.eGetEndMethod = E_UPDATED_ENDPOINT;
		 }
		 if (0 == tBoardInfo.vtCornerRelsve_Info.at(n).bBoardType
			 && E_UPDATED_ENDPOINT != tboardPartInfo.eGetStartMethod
			 && E_UPDATED_ENDPOINT != tboardPartInfo.eGetEndMethod
			 && E_CORRECT_ENDPOINT != tboardPartInfo.eGetStartMethod
			 && E_CORRECT_ENDPOINT != tboardPartInfo.eGetEndMethod && tboardPartInfo.bTracking ||
			 1 == tBoardInfo.vtCornerRelsve_Info.at(n).bBoardType && tboardPartInfo.bTracking ||
			 tboardPartInfo.bIsTrackingScanEnd)//直线起终点都是新测量得到不测中间点，圆弧且跟踪不测终点
		 {
			 tboardPartInfo.bIsMiddleMeasure = false;
		 }
		 tBoardInfo.vtBoardPartInfo.push_back(tboardPartInfo);
	 }
	 
	 
	 tBoardInfo.eContinueType = E_CONTINUOUS;
	 if (tBoardInfo.vtBoardPartInfo.at(2).bTracking)// 不连续焊接
	 {
		 tBoardInfo.eContinueType = E_NO_CONTINUOUS;
	 }
	 tBoardInfo.nContinueStartNo = 0, tBoardInfo.nContinueNum = 0;
	 if (tBoardInfo.eContinueType != E_NO_CONTINUOUS)
	 {
		 for (int n = 0; n < tBoardInfo.vtBoardPartInfo.size(); n++)
		 {
			 if ((tBoardInfo.vtBoardPartInfo.at(n).bTracking || !tBoardInfo.vbBoardDoesItExist.at(n)))
			 {
				 if (n > 2)
				 {
					 break;
				 }
				 tBoardInfo.nContinueNum = 0;
				 continue;
			 }
			 if (0 == tBoardInfo.nContinueNum)
			 {
				 tBoardInfo.nContinueStartNo = n;
			 }
			 tBoardInfo.nContinueNum++;
		 }
		 if (tBoardInfo.nContinueNum < 2)
		 {
			 tBoardInfo.eContinueType = E_NO_CONTINUOUS;
		 }
		 if (5 == tBoardInfo.nContinueNum)
		 {
			 tBoardInfo.vbBoardDoesItExist.at(0) = false;
			 tBoardInfo.vbBoardDoesItExist.at(4) = false;
			 tBoardInfo.nContinueStartNo = 1;
			 tBoardInfo.nContinueNum = 3;
		 }
	 }
	 else
	 {
		 tBoardInfo.nContinueStartNo = 2;
		 tBoardInfo.nContinueNum = 1;
	 }
     return tBoardInfo;
 }

bool CDiaphragmWeld::SelectLinesAndCircle(CRobotDriverAdaptor* pRobotDriver, T_CORNER_LINE_INFO_NEW tCornerLineInfo, VT_ENDPOINT_INFO_NEW* pvtPointInfo, VT_CIRCLE_CENTER_INFO_NEW* pvtCenterInfo, double& dRadius)
{
	dRadius = 0;
	if (1 == tCornerLineInfo.bBoardType)
	{
		T_ENDPOINT_INFO_NEW tStart = FindEndPointInfoFun(tCornerLineInfo.nStartNo, pvtPointInfo);
		T_ENDPOINT_INFO_NEW tEnd = FindEndPointInfoFun(tCornerLineInfo.nEndNo, pvtPointInfo);
		double dDis = TwoPointDis(tStart.tPointData.x, tStart.tPointData.y, tStart.tPointData.z,
			tEnd.tPointData.x, tEnd.tPointData.y, tEnd.tPointData.z);
		if (dDis < 5)//闭合圆弧
		{
			T_CIRCLE_CENTER_INFO_NEW tCenter = FindCenterPointInfoFun(tCornerLineInfo.nBoardNo, pvtCenterInfo);
			dRadius = TwoPointDis(tStart.tPointData.x, tStart.tPointData.y, tStart.tPointData.z,
				tCenter.dCenterX, tCenter.dCenterY, tCenter.dCenterZ);
			return false;
		}
	}
	return true;
}

//bool CDiaphragmWeld::CornerLineWeldFunNew(CRobotDriverAdaptor* pRobotDriver, T_CORNER_LINE_INFO_NEW tCornerLineInfo, VT_CORNER_LINE_INFO_NEW* pvtCornerLineInfo,
//	VT_ENDPOINT_INFO_NEW* pvtPointInfo, VT_MEASURE_INFO_NEW* pvMeasureInfo, VT_CIRCLE_CENTER_INFO_NEW* pvtCenterInfo)
//{
//	/*焊接角缝先焊接封闭的角缝，后焊接两端全是自由端的，在焊接一端自由一端干涉的焊缝
//	**拿到一条焊道后最简单的是，找到这条焊道前后关联的角焊缝，查看是都已经更新得起终点，
//	**如果跟新了较短的焊道只需要测量中间数据拟合直线后将起终点投影到该直线上。如果没有更新
//	**则需要搜索真实的起或终点。短焊道直接拟合直线焊接，较长焊道先拟合一段数据后中间利用开启跟踪用以修正
//	*/
//	//CHECK_STOP_RETURN_STATE(pRobotDriver);
//	bool bIfMessage = 1;
//	long long dStartTim;
//	dStartTim = XI_clock();
//	int nRobotTypeNo = 0;
//	CString strRobotName = pRobotDriver->m_strRobotName;
//	CString strPath;
//	strPath.Format(".\\WeldData\\%s\\CornerWeld\\%d_", strRobotName, tCornerLineInfo.nBoardNo);
//	string strPathAdr = strPath;
//	SaveWorkpieceNo(pRobotDriver, tCornerLineInfo.nBoardNo);//保存工件序号
//	InitWeldStartVal(pRobotDriver);//初始化参数，工艺
//	// 进来一条焊缝判断这条焊缝是否需要跟踪，是否需要搜起点终点，
//	// 长度超过三百添加跟踪，端点为自由端且没有被更新需要搜索起始和终止点
//	T_BOARD_INFO_NEW tBoardInfo;
//	tBoardInfo.strfileName = strPathAdr;
//	tBoardInfo = GetBoardInfo(pRobotDriver, tCornerLineInfo, pvtCornerLineInfo, pvtPointInfo, pvtCenterInfo, strPath);
//	// 计算测量数据 隔板为跟踪枪头，测量时需要注意起始点是否干涉
//	vector<T_ROBOT_COORS> vtRealPoint;
//	vtRealPoint.clear();
//	if (!m_ptUnit->m_bBreakPointContinue)
//	{
//		// 获取示教数据
//		CHECK_BOOL_RETURN(GetThoryMeasureTrack(pRobotDriver, tBoardInfo));
//#if 1
//		// 获取测量结果
//		CHECK_BOOL_RETURN(GetRealTeachData(pRobotDriver, tBoardInfo, pvtPointInfo));
//		// 计算焊接轨迹
//		CHECK_BOOL_RETURN(CalcRealWeldTrack(pRobotDriver, tBoardInfo));
//#else
//		 int nWarpNum = 0;
//		 LoadWarpNumber(pRobotDriver, nWarpNum);
//		 m_pTraceModel->m_vvtTrackWarpCoors.clear();
//		 m_pTraceModel->m_vvnTrackWarpPtType.clear();
//		 m_pTraceModel->m_vvtWarpCorrectCoors.clear();
//		 m_pTraceModel->m_vvtTrackWarpCoors.resize(nWarpNum);
//		 m_pTraceModel->m_vvnTrackWarpPtType.resize(nWarpNum);
//		 m_pTraceModel->m_vvtWarpCorrectCoors.resize(nWarpNum);
//		 for (int i = 0; i < nWarpNum; i++)
//		 {
//			 CString strWarpFile;
//			 string strFile;
//			 strWarpFile.Format("%s%d", tBoardInfo.vtBoardPartInfo.at(2).strfileName.c_str(), i);
//			 strFile = strWarpFile.GetBuffer(0);
//			 LoadDataRobotCoors(m_pTraceModel->m_vvtTrackWarpCoors.at(i), strFile + "TrackWarpCoors.txt");
//			 LoadDataInt(m_pTraceModel->m_vvnTrackWarpPtType.at(i), strFile + "TrackWarpPtType.txt");
//			 LoadDataRobotCoors(m_pTraceModel->m_vvtWarpCorrectCoors.at(i), strFile + "WarpCorrectCoors.txt");
//		 }
//		LoadDataRobotCoors(tBoardInfo.vtRealWeldTrackCoors, tBoardInfo.vtBoardPartInfo.at(2).strfileName + "DoWeldTrackCoors.txt");
//		LoadDataInt(tBoardInfo.vnDataPointType, tBoardInfo.vtBoardPartInfo.at(2).strfileName + "DoWeldTrackType.txt");
//		//计算连续可运行脉冲坐标
//		CHECK_BOOL_RETURN(CalcContinuePulseForWeld(tBoardInfo.vtRealWeldTrackCoors, tBoardInfo.vtRealWeldTrackPulse, false));
//#endif 	 
//	}
//	else
//	{
//		 tBoardInfo.vtRealWeldTrackCoors.clear();
//		if (tBoardInfo.vtBoardPartInfo.at(2).bTracking)//跟踪
//		{
//			//获取焊接轨迹
//			 LoadDataRobotCoors(tBoardInfo.vtRealWeldTrackCoors, tBoardInfo.vnDataPointType, ".\\WeldData\\WeldRobotLeft\\理论焊接数据.txt");
//			 int nWarpNum = 0;
//			 LoadWarpNumber(pRobotDriver, nWarpNum);
//			 
//			 m_pTraceModel->m_vvtTrackWarpCoors.clear();
//			 m_pTraceModel->m_vvnTrackWarpPtType.clear();
//			 m_pTraceModel->m_vvtWarpCorrectCoors.clear();
//			 m_pTraceModel->m_vvtTrackWarpCoors.resize(nWarpNum);
//			 m_pTraceModel->m_vvnTrackWarpPtType.resize(nWarpNum);
//			 m_pTraceModel->m_vvtWarpCorrectCoors.resize(nWarpNum);
//			 for (int i = 0; i < nWarpNum; i++)
//			 {
//				 CString strWarpFile;
//				 string strFile;
//				 strWarpFile.Format("%s%d", tBoardInfo.vtBoardPartInfo.at(2).strfileName.c_str(), i);
//				 strFile = strWarpFile.GetBuffer(0);
//				 LoadDataInt(m_pTraceModel->m_vvnTrackWarpPtType.at(i), strFile + "TrackWarpPtType.txt");
//				 LoadDataRobotCoors(m_pTraceModel->m_vvtTrackWarpCoors.at(i), strFile + "TrackWarpCoors.txt");			 
//				 LoadDataRobotCoors(m_pTraceModel->m_vvtWarpCorrectCoors.at(i), strFile + "WarpCorrectCoors.txt");
//			 }	 
//		}
//		else//先测后焊
//		{
//			 LoadDataRobotCoors(tBoardInfo.vtRealWeldTrackCoors, strPathAdr + "RealWeldTrackRobotCoors.txt");
//		 }
//		 //计算连续可运行脉冲坐标
//		 CHECK_BOOL_RETURN(CalcContinuePulseForWeld(tBoardInfo.vtRealWeldTrackCoors, tBoardInfo.vtRealWeldTrackPulse, false));
//	 } 	 	 
//	 // 开始焊接
//	 CHECK_BOOL_RETURN(DoWeld(pRobotDriver, tBoardInfo));
//
//	 m_ptUnit->m_bBreakPointContinue = false;
//	 //回安全位置
//	 m_ptUnit->RobotCheckDone();
//	 //CHECK_BOOL_RETURN(pRobotDriver->BackSafePosError(m_dFastApproachToWorkSpeed));
//	return true;
//}

bool CDiaphragmWeld::CalcRealWeldTrackAccordingToMeasureData(CRobotDriverAdaptor* pRobotDriver, T_BOARD_INFO_NEW tBoardInfo, vector<T_ROBOT_COORS>& vtRealPoint)
{
	//单个的直线和圆弧,轨迹拟合
	vector<XI_POINT> vtRealWeldCoors, vtRealWeldPosture;
	if (tBoardInfo.vtCornerRelsve_Info.at(0).bBoardType)//圆弧
	{
		int nSize = tBoardInfo.vvtMeasureResult_Info.at(0).size();
		if (nSize < 3)
		{
			XiMessageBoxOk("圆弧测量数据小于3个");
			return false;
		}
		vector<XI_POINT> vtFitInputPoints, vtOutputPoints;
		for (int n = 0; n < nSize; n++)
		{
			XI_POINT tp = {
				tBoardInfo.vvtMeasureResult_Info.at(0).at(n).dCoorX ,
				tBoardInfo.vvtMeasureResult_Info.at(0).at(n).dCoorY,
				tBoardInfo.vvtMeasureResult_Info.at(0).at(n).dCoorZ };
			vtFitInputPoints.push_back(tp);
		}
		reverse(vtFitInputPoints.begin(), vtFitInputPoints.end());
		SaveDataXiPoint(vtFitInputPoints, tBoardInfo.strfileName + "FitArctheoryData.txt");
		//拟合实际圆弧
		double center[3];
		center[0] = tBoardInfo.vtCenter_Info.at(0).dCenterX;
		center[1] = tBoardInfo.vtCenter_Info.at(0).dCenterY;
		center[2] = tBoardInfo.vtCenter_Info.at(0).dCenterZ;
		double detX = tBoardInfo.tStartPoint.x - center[0];
		double detY = tBoardInfo.tStartPoint.y - center[1];
		double radius = sqrt(detX * detX + detY * detY);
		if (!CorrectCircled(center, radius, vtFitInputPoints))
		{
			return false;
		}
		E_WELD_LINE vcPointOnTheSameLine;
		for (int i = 0; i < vtFitInputPoints.size() - 1; i++)
		{

			E_WELD_LINE vcPoint;
			CString cStr;
			cStr.Format("result\\%d.txt", i);
			LoadWeldLineData(vcPoint, cStr);
			vcPointOnTheSameLine.vtPoint.insert(vcPointOnTheSameLine.vtPoint.end(), vcPoint.vtPoint.begin(), vcPoint.vtPoint.end());
			vcPointOnTheSameLine.vdNormal.insert(vcPointOnTheSameLine.vdNormal.end(), vcPoint.vdNormal.begin(), vcPoint.vdNormal.end());
		}

		double dAdjustX = m_pTraceModel->m_dGunToEyeCompenX;
		double dAdjustZ = m_pTraceModel->m_dGunToEyeCompenZ;

		for (int nIdx = 0; nIdx < vcPointOnTheSameLine.vdNormal.size(); nIdx += 3)
		{
			vcPointOnTheSameLine.vdNormal.at(nIdx) = pRobotDriver->DirAngleToRz(vcPointOnTheSameLine.vdNormal.at(nIdx));
			vcPointOnTheSameLine.vtPoint.at(nIdx).x += dAdjustX * CosD(vcPointOnTheSameLine.vdNormal.at(nIdx));
			vcPointOnTheSameLine.vtPoint.at(nIdx).y += dAdjustX * SinD(vcPointOnTheSameLine.vdNormal.at(nIdx));
			vcPointOnTheSameLine.vtPoint.at(nIdx).z += dAdjustZ;
			T_ROBOT_COORS tCoor(
				vcPointOnTheSameLine.vtPoint.at(nIdx).x,
				vcPointOnTheSameLine.vtPoint.at(nIdx).y,
				vcPointOnTheSameLine.vtPoint.at(nIdx).z,
				m_dPlatWeldRx,
				m_dPlatWeldRy,
				vcPointOnTheSameLine.vdNormal.at(nIdx),
				0, 0, 0
			);
			vtRealPoint.push_back(tCoor);
		}
		//获取实际焊接轨迹
		SaveDataXiPoint(vcPointOnTheSameLine.vtPoint, tBoardInfo.strfileName + "CornerRealWeldCoors.txt");
		SaveDataRobotCoors(vtRealPoint, tBoardInfo.strfileName + "CornerAdjustRealWeldCoors.txt");
		return true;
	}
	else
	{
		double dNormalAngle;
		//直线
		// 起点或终点需要根据理论长度推算出来
		if (E_CORRECT_ENDPOINT == tBoardInfo.eGetStartMethod || E_CORRECT_ENDPOINT == tBoardInfo.eGetEndMethod)
		{
			vector<XI_POINT> vtMeasureResult;
			for (int n = 0; n < tBoardInfo.vvtMeasureResult_Info.at(0).size(); n++)//测量数据是由终点指向起点
			{
				XI_POINT tp = { tBoardInfo.vvtMeasureResult_Info.at(0).at(n).dCoorX,tBoardInfo.vvtMeasureResult_Info.at(0).at(n).dCoorY,tBoardInfo.vvtMeasureResult_Info.at(0).at(n).dCoorZ };
				vtMeasureResult.push_back(tp);
			}
			if (E_CORRECT_ENDPOINT == tBoardInfo.eGetEndMethod)// 起点需要修正
			{
				reverse(vtMeasureResult.begin(), vtMeasureResult.end());
			}
			T_LINE_PARA tLineParam;
			tLineParam = CalcLineParamRansac(vtMeasureResult, 0.7);
			WriteLog("修正直线参数：%lf %lf %lf %lf %lf %lf", tLineParam.dPointX, tLineParam.dPointY, tLineParam.dPointZ,
				tLineParam.dDirX, tLineParam.dDirY, tLineParam.dDirZ);
			if (E_CORRECT_ENDPOINT == tBoardInfo.eGetEndMethod)// 终点需要修正
			{
				tBoardInfo.tEndPoint.x = tBoardInfo.tStartPoint.x + tBoardInfo.vtCornerRelsve_Info.at(0).dBoardLength * tLineParam.dDirX;
				tBoardInfo.tEndPoint.y = tBoardInfo.tStartPoint.y + tBoardInfo.vtCornerRelsve_Info.at(0).dBoardLength * tLineParam.dDirY;
				tBoardInfo.tEndPoint.z = tBoardInfo.tStartPoint.z + tBoardInfo.vtCornerRelsve_Info.at(0).dBoardLength * tLineParam.dDirZ;
			}
			else if (E_CORRECT_ENDPOINT == tBoardInfo.eGetStartMethod)// 起点需要修正
			{
				tBoardInfo.tStartPoint.x = tBoardInfo.tEndPoint.x + tBoardInfo.vtCornerRelsve_Info.at(0).dBoardLength * tLineParam.dDirX;
				tBoardInfo.tStartPoint.y = tBoardInfo.tEndPoint.y + tBoardInfo.vtCornerRelsve_Info.at(0).dBoardLength * tLineParam.dDirY;
				tBoardInfo.tStartPoint.z = tBoardInfo.tEndPoint.z + tBoardInfo.vtCornerRelsve_Info.at(0).dBoardLength * tLineParam.dDirZ;
			}
		}
		if (!FitCalLinePointsByMiddlePoints(tBoardInfo.tStartPoint, tBoardInfo.tEndPoint, tBoardInfo.vvtMeasureResult_Info.at(0), vtRealWeldCoors, vtRealWeldPosture, dNormalAngle, m_pTraceModel->m_nOpenTrackingPos,
			m_pTraceModel->m_nCloseTrackingPos, 3.0, 20, true, 20, tBoardInfo.vtCornerRelsve_Info.at(0).nStartType > 0, tBoardInfo.vtCornerRelsve_Info.at(0).nEndType > 0))
		{
			XiMessageBoxOk("直线拟合失败");
			return false;
		}
	}
	//偏移量
	CalcMeasurePosInBase(vtRealWeldCoors, m_pTraceModel->m_dGunToEyeCompenX, m_pTraceModel->m_dGunToEyeCompenZ, vtRealWeldPosture, vtRealWeldCoors);
	SaveDataXiPoint(vtRealWeldCoors, tBoardInfo.strfileName + "CornerRealWeldCoors.txt");
	//添加起始结尾安全位置，姿态分配,可以是跟踪也可以是先测后焊
	vtRealPoint = GetRealWeldData(pRobotDriver, vtRealWeldCoors, vtRealWeldPosture);
	return true;
}

//单一加强圈隔板焊接
bool CDiaphragmWeld::StrengthRingWeldFunNew(CRobotDriverAdaptor* pRobotDriver, T_CORNER_LINE_INFO_NEW tCornerLineInfo,
	VT_CORNER_LINE_INFO_NEW* pvtCornerLineInfo, VT_ENDPOINT_INFO_NEW* pvtPointInfo)
{
	return true;
}


bool CDiaphragmWeld::CorrectEndPoint(CRobotDriverAdaptor* pRobotDriver, T_BOARD_INFO_NEW tBoardInfo,
	VT_ENDPOINT_INFO_NEW* pvtPointInfo, int nIndex, XI_POINT tStartPoint, XI_POINT tEndPoint)
{
	if (tBoardInfo.vtCornerRelsve_Info.size() < 1)
	{
		XiMessageBox("CorrectEndPoint：输入数据有误");
		return false;
	}
	XI_POINT tPoint;

	if ((0 == tBoardInfo.vtCornerRelsve_Info.at(nIndex).nStartType &&
		E_SEARCH_ENDPOINT == tBoardInfo.vtBoardPartInfo.at(nIndex).eGetStartMethod) ||
		(4 == tBoardInfo.vtCornerRelsve_Info.at(nIndex).nStartType &&
			E_MEASURE_PROJPOINT == tBoardInfo.vtBoardPartInfo.at(nIndex).eGetStartMethod)
		)//自由端，起点搜索得到，背面关联焊道
	{
		T_ENDPOINT_INFO_NEW tThyp = FindEndPointInfoFun(tBoardInfo.vtCornerRelsve_Info.at(nIndex).nStartNo, pvtPointInfo);
		for (int i = 0; i < pvtPointInfo->size(); i++)
		{
			T_ENDPOINT_INFO_NEW tp = FindEndPointInfoFun(i, pvtPointInfo);
			if (tThyp.nEndPointNo == tp.nEndPointNo)
			{
				continue;
			}
			double dDis = TwoPointDis(tThyp.tPointData.x, tThyp.tPointData.y, tThyp.tPointData.z,
				tp.tPointData.x, tp.tPointData.y, tp.tPointData.z);
			if (dDis < 40)//则认为其为间接关联端点
			{
				if (!ReadCorrectEndPointFun(pRobotDriver, tp.nEndPointNo, tPoint))
				{
					RecordCorrectEndPointFun(pRobotDriver, tp.nEndPointNo, tStartPoint);
				}
				break;
			}
		}
	}
	else if (0 != tBoardInfo.vtCornerRelsve_Info.at(nIndex).nStartType
		&& E_UPDATED_ENDPOINT != tBoardInfo.vtBoardPartInfo.at(nIndex).eGetStartMethod)//起点非自由端且自身不是被更新的端点,跟新起点关联
	{
		if (!ReadCorrectEndPointFun(pRobotDriver, tBoardInfo.vtCornerRelsve_Info.at(nIndex - 1).nEndNo, tPoint))
		{
			RecordCorrectEndPointFun(pRobotDriver, tBoardInfo.vtCornerRelsve_Info.at(nIndex - 1).nEndNo, tStartPoint);
		}
	}


	if (0 != tBoardInfo.vtCornerRelsve_Info.at(nIndex).nEndType
		&& E_UPDATED_ENDPOINT != tBoardInfo.vtBoardPartInfo.at(nIndex).eGetEndMethod)
	{
		if (!ReadCorrectEndPointFun(pRobotDriver, tBoardInfo.vtCornerRelsve_Info.at(nIndex + 1).nStartNo, tPoint))
		{
			RecordCorrectEndPointFun(pRobotDriver, tBoardInfo.vtCornerRelsve_Info.at(nIndex + 1).nStartNo, tEndPoint);
		}
	}
	else if (0 == tBoardInfo.vtCornerRelsve_Info.at(nIndex).nEndType
		&& E_SEARCH_ENDPOINT == tBoardInfo.vtBoardPartInfo.at(nIndex).eGetEndMethod/*搜索的端点*/)
	{
		T_ENDPOINT_INFO_NEW tThyp = FindEndPointInfoFun(tBoardInfo.vtCornerRelsve_Info.at(nIndex).nEndNo, pvtPointInfo);
		for (int i = 0; i < pvtPointInfo->size(); i++)
		{
			T_ENDPOINT_INFO_NEW tp = FindEndPointInfoFun(i, pvtPointInfo);
			if (tThyp.nEndPointNo == tp.nEndPointNo)
			{
				continue;
			}
			double dDis = TwoPointDis(tThyp.tPointData.x, tThyp.tPointData.y, tThyp.tPointData.z,
				tp.tPointData.x, tp.tPointData.y, tp.tPointData.z);
			if (dDis < 40)//则认为其为间接关联端点
			{
				if (!ReadCorrectEndPointFun(pRobotDriver, tp.nEndPointNo, tPoint))
				{
					RecordCorrectEndPointFun(pRobotDriver, tp.nEndPointNo, tEndPoint);
				}
				break;
			}
		}
	}
	return true;
}



bool CDiaphragmWeld::GetTheoryReferencePulse(CRobotDriverAdaptor* pRobotDriver, T_ROBOT_COORS tTeachCoord, T_ANGLE_PULSE& tTeachPulse)
{
	tTeachCoord.dZ = fmod(tTeachCoord.dRZ, 360);
	tTeachCoord.dRZ = tTeachCoord.dRZ < -180 ? tTeachCoord.dRZ + 360 : tTeachCoord.dRZ;
	tTeachCoord.dRZ = tTeachCoord.dRZ > 180 ? tTeachCoord.dRZ - 360 : tTeachCoord.dRZ;
	if (tTeachCoord.dX < 400 && tTeachCoord.dY < 0.0)
	{
		tTeachPulse = m_pTraceModel->m_vtTheoryMeasurePointLeft[0];
	}
	else if (tTeachCoord.dX < 400 && tTeachCoord.dY>0.0)
	{
		tTeachPulse = m_pTraceModel->m_vtTheoryMeasurePointLeft[2];
	}
	else if (tTeachCoord.dX > 400)
	{
		tTeachPulse = m_pTraceModel->m_vtTheoryMeasurePointLeft[1];
	}
	return true;
}

bool CDiaphragmWeld::GetMeasureDataWithMoveNew(CRobotDriverAdaptor* pRobotDriver, T_BOARD_INFO_NEW& tBoardInfo, int nIndex, vector<T_ALGORITHM_POINT>& vtTeachResult)
{
	int nRobotNo = pRobotDriver->m_nRobotNo;
	// E_CAM_ID eCamId = nRobotNo == 0 ? E_LEFT_ROBOT_LEFT_CAM_H : E_RIGHT_ROBOT_LEFT_CAM_H;
	int nSize = tBoardInfo.vtMeasureTrack.at(nIndex).vtMeasurePoint.size();
	if (nSize < 2)
	{
		XiMessageBox("GetMeasureDataWithMoveNew:输入数据有误");
		return false;
	}
	//暂时只测一条焊缝
	nSize = 1;
	vector<T_ROBOT_COORS> vtTeachCoord;
	vector<T_ANGLE_PULSE> vtTeachPulse;
	vector<int> vtMeasureType;
	vtTeachCoord.clear();
	vtTeachPulse.clear();
	vtMeasureType.clear();

	vtTeachCoord = tBoardInfo.vtMeasureTrack.at(nIndex).vtMeasurePoint;
	vtTeachPulse = tBoardInfo.vtMeasureTrack.at(nIndex).vtMeasurePulse;
	vtMeasureType = tBoardInfo.vtMeasureTrack.at(nIndex).vnMeasureType;
	reverse(vtTeachCoord.begin(), vtTeachCoord.end());
	reverse(vtTeachPulse.begin(), vtTeachPulse.end());
	reverse(vtMeasureType.begin(), vtMeasureType.end());
	reverse(tBoardInfo.vvtMeasure_Data_Mind.at(nIndex).begin(), tBoardInfo.vvtMeasure_Data_Mind.at(nIndex).end());

	if (!g_bLocalDebugMark)
	{
		m_ptUnit->SwitchDHCamera(m_ptUnit->m_nMeasureCameraNo, true, E_CALL_BACK_MODE_WAIT_IMAGE, E_ACQUISITION_MODE_CONTINUE);
	}
	
	 T_ROBOT_MOVE_SPEED tPulseMove ( m_dFastApproachToWorkSpeed ,100,100 );
	// 移动大车工件位置
	CHECK_BOOL_RETURN(PosMoveMachine(pRobotDriver, vtTeachCoord.at(0).dBY, tPulseMove.dSpeed * 2, COORD_ABS));
	//pRobotDriver->SafeMoveJByJobSaxis(vtTeachPulse.at(0), tPulseMove);
	CHECK_BOOL_RETURN(m_ptUnit->CheckRobotDone(vtTeachPulse.at(0)));

	T_ALGORITHM_POINT tPointOnTheSameLine;
	vector<T_ALGORITHM_POINT> vtPointOnTheSameLine;
	vector<T_ALGORITHM_POINT> vtLeftPoints;
	vector<T_ALGORITHM_POINT> vtRightPoints;

	for (int i = 1; i < vtTeachCoord.size(); i++)
	{
		T_ROBOT_COORS tGunDownCoors = vtTeachCoord[i];
		tPulseMove.dSpeed = m_dFastApproachToWorkSpeed;
		if (!MoveByJob(pRobotDriver, tGunDownCoors, tPulseMove, pRobotDriver->m_nExternalAxleType, "MOVL"))
		{
			XiMessageBox("机器人未运动到位");
			return false;
		}
		if (E_TRANSITION_POINT & vtMeasureType[i])
		{
			continue;
		}
		// 采图测量
		CString str;
		str.Format("Corner_%d", i);
		if (!g_bLocalDebugMark)
		{
			T_ROBOT_COORS tCurPos = GetCurrentPos(pRobotDriver);
			T_ANGLE_PULSE tCurPulse = GetCurrentPulse(pRobotDriver);
			MeasurePointLaserData(pRobotDriver, str, tPointOnTheSameLine, vtLeftPoints, vtRightPoints, tCurPulse, tCurPos, 0/*, eCamId*/, m_ptUnit->m_nMeasureCameraNo);
			vtTeachResult.push_back(tPointOnTheSameLine);
		}
		else
		{
			// 调试用

			tPointOnTheSameLine.dCoorX = tBoardInfo.vvtMeasure_Data_Mind.at(nIndex).at(i - 1).dX;
			tPointOnTheSameLine.dCoorY = tBoardInfo.vvtMeasure_Data_Mind.at(nIndex).at(i - 1).dY;
			tPointOnTheSameLine.dCoorZ = tBoardInfo.vvtMeasure_Data_Mind.at(nIndex).at(i - 1).dZ;
			vtTeachResult.push_back(tPointOnTheSameLine);
		}
	}
	if (!g_bLocalDebugMark)
	{
		m_ptUnit->SwitchDHCamera(m_ptUnit->m_nMeasureCameraNo,false);
	}
	return true;
	
}

bool CDiaphragmWeld::DoTeachWithMove(CRobotDriverAdaptor* pRobotDriver, vector<T_ROBOT_COORS> vtTeachCoors, vector<T_ROBOT_COORS> vtTeachCoorsTest, vector<T_ANGLE_PULSE> vtTeachPulse,
	vector<int> vtMeasureType, vector<T_ALGORITHM_POINT>& vtTeachResult, bool bIsReverse)
{
	int nRobotNo = pRobotDriver->m_nRobotNo;
	// E_CAM_ID eCamId = nRobotNo == 0 ? E_LEFT_ROBOT_LEFT_CAM_H : E_RIGHT_ROBOT_LEFT_CAM_H;
	int nSize = vtTeachCoors.size();
	if (nSize < 2)
	{
		XiMessageBox("GetMeasureDataWithMoveNew:输入数据有误");
		return false;
	}
	if (bIsReverse)
	{
		reverse(vtTeachCoors.begin(), vtTeachCoors.end());
		reverse(vtTeachPulse.begin(), vtTeachPulse.end());
		reverse(vtMeasureType.begin(), vtMeasureType.end());
		reverse(vtTeachCoorsTest.begin(), vtTeachCoorsTest.end());
	}

	if (!g_bLocalDebugMark)
	{
		m_ptUnit->SwitchDHCamera(m_ptUnit->m_nMeasureCameraNo, true, E_CALL_BACK_MODE_WAIT_IMAGE, E_ACQUISITION_MODE_CONTINUE);
	}

	// 移动大车工件位置 快速移动时可能会PoMoveMachine失败，给两次修正机会
	
	CHECK_BOOL_RETURN(PosMoveMachine(pRobotDriver, vtTeachCoors.at(0).dBY, m_gExAxisSpeed/*m_dFastApproachToWorkSpeed * 3*/, COORD_ABS));

	//pRobotDriver->SafeMoveJByJobSaxis(vtTeachPulse.at(0), tPulseMove);
	//CHECK_PULSE_RETURN_BOOL(pRobotDriver, vtTeachPulse.at(0));

	vector<T_ROBOT_MOVE_INFO> vtRobotMoveInfo(0);
	T_ROBOT_MOVE_INFO tRobotMoveInfo;
	T_ROBOT_MOVE_SPEED tPulseMove(m_dSafePosRunSpeed, 50, 50);
	vector<T_ANGLE_PULSE> vtSafeMovePulse(0);
	//CHECK_BOOL_RETURN(pRobotDriver->GetSafeMoveJPulse(vtSafeMovePulse,vtTeachPulse.at(0)));
	for (int i = vtSafeMovePulse.size() - 1; i >= 0; i--)
	{
		tRobotMoveInfo = pRobotDriver->PVarToRobotMoveInfo(0, vtSafeMovePulse[i], tPulseMove, MOVJ);
		vtRobotMoveInfo.push_back(tRobotMoveInfo);
	}
	tPulseMove.dSpeed = m_dSafeApproachToWorkSpeed;
	tRobotMoveInfo = pRobotDriver->PVarToRobotMoveInfo(0, vtTeachCoors[1], tPulseMove, MOVL);
	vtRobotMoveInfo.push_back(tRobotMoveInfo);

	pRobotDriver->SetMoveValue(vtRobotMoveInfo);
	pRobotDriver->CallJob("CONTIMOVANY");
	m_ptUnit->CheckRobotDone(vtTeachCoors[1]);

	tPulseMove.dSpeed = m_dFastApproachToWorkSpeed;
	tPulseMove.dACC = 20;
	tPulseMove.dDEC = 20;
	T_ALGORITHM_POINT tPointOnTheSameLine;
	vector<T_ALGORITHM_POINT> vtPointOnTheSameLine;
	vector<T_ALGORITHM_POINT> vtLeftPoints;
	vector<T_ALGORITHM_POINT> vtRightPoints;

	 for (int i = 1; i < vtTeachCoors.size(); i++)
	 {
		 T_ROBOT_COORS tGunDownCoors = vtTeachCoors[i];
		if (fabs(TwoPointDis(vtTeachCoors[i - 1].dX, vtTeachCoors[i - 1].dY+ vtTeachCoors[i - 1].dBY, tGunDownCoors.dX, tGunDownCoors.dY+ tGunDownCoors.dBY)) < 50.0)
		{
			tPulseMove.dSpeed = m_dFastApproachToWorkSpeed * 2;
		}
		else
		{
			tPulseMove.dSpeed = m_dFastApproachToWorkSpeed * 3;
		}
		 if (!MoveByJob(pRobotDriver, tGunDownCoors, tPulseMove, pRobotDriver->m_nExternalAxleType, "MOVL"))
		 {
			 XiMessageBox("机器人未运动到位");
			 return false;
		 }
		 if (E_TRANSITION_POINT & vtMeasureType[i])
		 {
			 continue;
		 }
		 // 采图测量
		 CString str;
		 str.Format("Corner_%d", i);
		 if (!g_bLocalDebugMark)
		 {
			 T_ROBOT_COORS tCurPos = GetCurrentPos(pRobotDriver);
			 T_ANGLE_PULSE tCurPulse = GetCurrentPulse(pRobotDriver);
			 MeasurePointLaserData(pRobotDriver, str, tPointOnTheSameLine, vtLeftPoints, vtRightPoints, tCurPulse, tCurPos, 0/*, eCamId*/, m_ptUnit->m_nMeasureCameraNo);
			 vtTeachResult.push_back(tPointOnTheSameLine);
		 }
		 else
		 {
			 // 调试用

			tPointOnTheSameLine.dCoorX = vtTeachCoorsTest.at(i - 1).dX;
			tPointOnTheSameLine.dCoorY = vtTeachCoorsTest.at(i - 1).dY;
			tPointOnTheSameLine.dCoorZ = vtTeachCoorsTest.at(i - 1).dZ;
			vtTeachResult.push_back(tPointOnTheSameLine);
		}
	}
	if (!g_bLocalDebugMark)
	{
		m_ptUnit->SwitchDHCamera(m_ptUnit->m_nMeasureCameraNo,false);
	}
	return true;
}

bool CDiaphragmWeld::CalcContinuePulse(CRobotDriverAdaptor* pRobotDriver, T_BOARD_INFO_NEW& tBoardInfo/*vector<T_ROBOT_COORS> &vtRobotCoors*/,
	T_ANGLE_PULSE tStandardRefPulse/*, vector<T_ANGLE_PULSE> &vtRobotPulse*/)
{
	/*if (!tBoardInfo.bIsMiddleMeasure)
	{
		return true;
	}*/
	int nCoordNum = tBoardInfo.vvtMeasure_Info.size();
	if (nCoordNum <= 0)
	{
		return false;
	}
	tBoardInfo.vvtMsePulse_Info.clear();
	for (int n = 0; n < nCoordNum; n++)
	{
		vector<T_ANGLE_PULSE> vtRobotPulse;
		vector<T_ROBOT_COORS> vtRobotCoors = tBoardInfo.vvtMeasure_Info.at(n);
		vtRobotPulse.clear();
		if (!tBoardInfo.vbBoardDoesItExist.at(n) || !tBoardInfo.vtBoardPartInfo.at(n).bIsMiddleMeasure)
		{
			tBoardInfo.vvtMsePulse_Info.push_back(vtRobotPulse);
			continue;
		}

		int nCorNum = vtRobotCoors.size();
		if (nCorNum < 1)
		{
			tBoardInfo.vvtMsePulse_Info.push_back(vtRobotPulse);
			//已修改
			XUI::MesBox::PopInfo("第：{0}次测量数据有误", n);
			//XiMessageBox("第：%d 次测量数据有误", n);
			continue;
		}
		T_ROBOT_COORS tCoutRobotCoor;
		for (int i = 0; i < nCorNum; i++)
		{
			vtRobotCoors.at(i).dX += 20 * CosD(vtRobotCoors.at(i).dRZ);
			vtRobotCoors.at(i).dY += 20 * SinD(vtRobotCoors.at(i).dRZ);
			if (CalcRobotAndMachinePos(pRobotDriver, vtRobotCoors.at(i), tCoutRobotCoor))
			{
				tBoardInfo.vvtMeasure_Info.at(n).at(i) = tCoutRobotCoor;
			}
			else
			{
				XiMessageBox("测量位置超出极限");
				return false;
			}

		}
		vtRobotPulse.clear();
		vtRobotPulse.resize(nCorNum);
		CString cFileName = ".\\GraphData\\TeachPoint.txt";
		FILE* pfTeachPoints = fopen(cFileName, "w");
		T_ROBOT_COORS tMagneticCoors;
		T_ROBOT_COORS tTrackCamCoors;
		for (int i = 0; i < nCorNum; i++)
		{
			tMagneticCoors = tBoardInfo.vvtMeasure_Info.at(n).at(i);
			if (!pRobotDriver->MoveToolByWeldGun(tBoardInfo.vvtMeasure_Info.at(n).at(i), pRobotDriver->m_tTools.tGunTool, tMagneticCoors, m_ptUnit->GetCameraTool(m_ptUnit->m_nTrackCameraNo), tTrackCamCoors))
			{
				return false;
			}
			tBoardInfo.vvtMeasure_Info.at(n).at(i) = tTrackCamCoors;
			fprintf(pfTeachPoints, "%d%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf\n",
				i, tTrackCamCoors.dX, tTrackCamCoors.dY, tTrackCamCoors.dZ,
				tTrackCamCoors.dRX, tTrackCamCoors.dRY, tTrackCamCoors.dRZ);
		}
		fclose(pfTeachPoints);

		// Rz范围转换到±180.0以内
		for (int i = 0; i < nCorNum; i++)
		{
			tBoardInfo.vvtMeasure_Info.at(n)[i].dRZ = fmod(tBoardInfo.vvtMeasure_Info.at(n)[i].dRZ, 360.0);
			tBoardInfo.vvtMeasure_Info.at(n)[i].dRZ = tBoardInfo.vvtMeasure_Info.at(n)[i].dRZ > 180.0 ? tBoardInfo.vvtMeasure_Info.at(n)[i].dRZ - 360.0 : tBoardInfo.vvtMeasure_Info.at(n)[i].dRZ;
			tBoardInfo.vvtMeasure_Info.at(n)[i].dRZ = tBoardInfo.vvtMeasure_Info.at(n)[i].dRZ < -180.0 ? tBoardInfo.vvtMeasure_Info.at(n)[i].dRZ + 360.0 : tBoardInfo.vvtMeasure_Info.at(n)[i].dRZ;
		}

		// 使Rz相邻变化量小于180°
		vector<double> vdRz;
		vdRz.resize(nCorNum);
		vdRz[0] = tBoardInfo.vvtMeasure_Info.at(n)[0].dRZ;
		double dMaxRz = vdRz[0];
		double dMinRz = vdRz[0];
		for (int i = 1; i < nCorNum; i++)
		{
			if (fabs(tBoardInfo.vvtMeasure_Info.at(n)[i].dRZ - vdRz[i - 1]) <= 180.0)
			{
				vdRz[i] = tBoardInfo.vvtMeasure_Info.at(n)[i].dRZ;
			}
			else if (tBoardInfo.vvtMeasure_Info.at(n)[i].dRZ > vdRz[i - 1])
			{
				vdRz[i] = tBoardInfo.vvtMeasure_Info.at(n)[i].dRZ - 360.0;
			}
			else
			{
				vdRz[i] = tBoardInfo.vvtMeasure_Info.at(n)[i].dRZ + 360.0;
			}
			dMaxRz = dMaxRz < vdRz[i] ? vdRz[i] : dMaxRz;
			dMinRz = dMinRz > vdRz[i] ? vdRz[i] : dMinRz;
		}

		// 缩小连续的Rz范围
		double dMaxMinAverageRz = (dMaxRz + dMinRz) / 2.0;
		if (fabs(dMaxMinAverageRz) > 180.0)
		{
			double dOffsetDir = dMaxMinAverageRz > 0.0 ? -1.0 : 1.0;
			int nOffsetTimes = fabs(dMaxMinAverageRz) / 360;
			double dOffsetDis = (fabs(dMaxMinAverageRz) - (double)nOffsetTimes * 360.0) > 180.0 ? 360.0 : 0.0;
			for (int i = 0; i < nCorNum; i++)
			{
				vdRz[i] += (dOffsetDir * (double)nOffsetTimes * 360.0 + dOffsetDis);
			}
		}

		// 计算转换参考关节坐标 并 计算直角坐标对应的关节坐标
		double dAxisUnitT = pRobotDriver->m_cXiRobotAbsCoorsTrans->m_dTPulse; // T轴脉冲当量

		T_ROBOT_COORS tStandardRefCoord;
		pRobotDriver->RobotKinematics(tStandardRefPulse, pRobotDriver->m_tTools.tGunTool,
			tStandardRefCoord);
		for (int i = 0; i < nCorNum; i++)
		{
			bool bRst = GetTheoryReferencePulse(pRobotDriver, tBoardInfo.vvtMeasure_Info.at(n)[i], vtRobotPulse[i]);
			if (false == bRst)
			{
				vtRobotPulse.clear();
				//已修改
				XUI::MesBox::PopInfo("转换第{0}个直角坐标失败!", i);
				//XiMessageBox("转换第%d个直角坐标失败!", i);
				//return false;
			}
		}
		tBoardInfo.vvtMsePulse_Info.push_back(vtRobotPulse);
	}
	return true;
}

bool CDiaphragmWeld::GenerateTrack(vector<XI_POINT>& vtTrack, XI_POINT tStart, double dStartAngle, XI_POINT tEndPoint, double dEndAngle, XI_POINT tCenter, double dStepDis, bool bIsArc)
{
	vtTrack.clear();
	XI_POINT tPtn;
	double dTotalLen = 0.0;
	int nTotalPtnNum = 0;
	if (bIsArc)
	{
		XiAlgorithm alg;
		BOOL bIsclockwise = false;//顺逆时针
		double dStartDirAngle = alg.CalcArcAngle(tStart.x - tCenter.x, tStart.y - tCenter.y);
		double dEndDirAngle = alg.CalcArcAngle(tEndPoint.x - tCenter.x, tEndPoint.y - tCenter.y);
		if (fabs(dStartDirAngle - dStartAngle) > 90)
		{
			bIsclockwise = true;//顺时针
			//dStartDirAngle = dStartDirAngle + 180 > 360 ? (dStartDirAngle + 180) - 360 : (dStartDirAngle + 180);
			//dEndDirAngle = dEndDirAngle + 180 > 360 ? (dEndDirAngle + 180) - 360 : (dEndDirAngle + 180);
		}
		//dEndDirAngle = dEndDirAngle < dStartDirAngle ? dEndDirAngle + 360.0 : dEndDirAngle;
		// 逆时针焊接，起点到终点方向角应该是减少的，若终点方向角比起点大则，
		dEndDirAngle = dEndDirAngle < 0 ? dEndDirAngle + 360 : dEndDirAngle;
		double dTotalAngle = dEndDirAngle - dStartDirAngle;
		if (dEndDirAngle > dStartDirAngle && !bIsclockwise)
		{
			dTotalAngle = (dEndDirAngle - dStartDirAngle) - 360;
		}

		if (dTotalAngle == 0)
		{
			dTotalAngle = 360.0;
		}
		// 标准圆弧半径一致，中和非标准圆弧
		double dRadius1 = TwoPointDis(
			tStart.x, tStart.y, tStart.z, tCenter.x, tCenter.y, tCenter.z);
		double dRadius2 = TwoPointDis(
			tEndPoint.x, tEndPoint.y, tEndPoint.z, tCenter.x, tCenter.y, tCenter.z);
		double dRadius = (dRadius1 + dRadius2) / 2;
		double dArcLen = 2.0 * PI * dRadius * fabs(dTotalAngle) / 360.0;
		nTotalPtnNum = dArcLen / dStepDis;
		double dAngleInterval = dTotalAngle / (double)nTotalPtnNum;
		for (int i = 0; i < labs(nTotalPtnNum); i++)
		{
			double dDirAngle = dStartDirAngle + dAngleInterval * i;
			tPtn.x = tCenter.x + dRadius * CosD(dDirAngle);
			tPtn.y = tCenter.y + dRadius * SinD(dDirAngle);
			tPtn.z = tCenter.z;
			vtTrack.push_back(tPtn);
		}
	}
	else
	{
		dTotalLen = TwoPointDis(tStart.x, tStart.y, tStart.z, tEndPoint.x, tEndPoint.y, tEndPoint.z);
		nTotalPtnNum = dTotalLen / dStepDis;
		dStepDis = dTotalLen / (nTotalPtnNum);
		XI_POINT tDir;
		tDir.x = (tEndPoint.x - tStart.x) / dTotalLen;
		tDir.y = (tEndPoint.y - tStart.y) / dTotalLen;
		tDir.z = (tEndPoint.z - tStart.z) / dTotalLen;
		for (int i = 0; i < nTotalPtnNum; i++)
		{
			tPtn.x = tStart.x + (tDir.x * i);
			tPtn.y = tStart.y + (tDir.y * i);
			tPtn.z = tStart.z + (tDir.z * i);
			vtTrack.push_back(tPtn);
		}
	}
	return true;
}

bool CDiaphragmWeld::GenerateTrack(vector<XI_POINT>& vtTrack, XI_POINT tStart, XI_POINT tEndPoint, XI_POINT tCenter, double dStepDis, bool bIsArc)
{
	vtTrack.clear();
	XI_POINT tPtn;
	double dTotalLen = 0.0;
	int nTotalPtnNum = 0;
	if (bIsArc)
	{
		XiAlgorithm alg;
		double dStartDirAngle = alg.CalcArcAngle(tStart.x - tCenter.x, tStart.y - tCenter.y);
		double dEndDirAngle = alg.CalcArcAngle(tEndPoint.x - tCenter.x, tEndPoint.y - tCenter.y);
		//dEndDirAngle = dEndDirAngle < dStartDirAngle ? dEndDirAngle + 360.0 : dEndDirAngle;
		// 逆时针焊接，起点到终点方向角应该是减少的，若终点方向角比起点大则，
		dEndDirAngle = dEndDirAngle < 0 ? dEndDirAngle + 360 : dEndDirAngle;
		double dTotalAngle = dEndDirAngle - dStartDirAngle;
		if (dEndDirAngle > dStartDirAngle)
		{
			dTotalAngle = (dEndDirAngle - dStartDirAngle) - 360;
		}

		if (dTotalAngle == 0)
		{
			dTotalAngle = 360.0;
		}
		double dRadius = TwoPointDis(
			tStart.x, tStart.y, tStart.z, tCenter.x, tCenter.y, tCenter.z);

		double dArcLen = 2.0 * PI * dRadius * fabs(dTotalAngle) / 360.0;
		nTotalPtnNum = dArcLen / dStepDis;
		double dAngleInterval = dTotalAngle / (double)nTotalPtnNum;
		for (int i = 0; i < labs(nTotalPtnNum); i++)
		{
			double dDirAngle = dStartDirAngle + dAngleInterval * i;
			tPtn.x = tCenter.x + dRadius * CosD(dDirAngle);
			tPtn.y = tCenter.y + dRadius * SinD(dDirAngle);
			tPtn.z = tCenter.z;
			vtTrack.push_back(tPtn);
		}
	}
	else
	{
		dTotalLen = TwoPointDis(tStart.x, tStart.y, tStart.z, tEndPoint.x, tEndPoint.y, tEndPoint.z);
		nTotalPtnNum = dTotalLen / dStepDis;
		dStepDis = dTotalLen / (nTotalPtnNum);
		XI_POINT tDir;
		tDir.x = (tEndPoint.x - tStart.x) / dTotalLen;
		tDir.y = (tEndPoint.y - tStart.y) / dTotalLen;
		tDir.z = (tEndPoint.z - tStart.z) / dTotalLen;
		for (int i = 0; i < nTotalPtnNum; i++)
		{
			tPtn.x = tStart.x + (tDir.x * i);
			tPtn.y = tStart.y + (tDir.y * i);
			tPtn.z = tStart.z + (tDir.z * i);
			vtTrack.push_back(tPtn);
		}
	}
	return true;
}

bool CDiaphragmWeld::GetThoryMeasureTrack(CRobotDriverAdaptor* pRobotDriver, T_BOARD_INFO_NEW& tBoardInfo)
{
	if (tBoardInfo.vtCornerRelsve_Info.size() < 5)
	{
		XiMessageBoxOk("关联焊缝数据有误,请检查数据");
		return false;
	}
	// 计算测量轨迹
	tBoardInfo.vtMeasureTrack.clear();
	tBoardInfo.vvtMeasure_Data_Start.clear();
	tBoardInfo.vvtMeasure_Data_Mind.clear();
	tBoardInfo.vvtMeasure_Data_End.clear();
	for (int i = 0; i < tBoardInfo.vtCornerRelsve_Info.size(); i++)
	{
		E_MEASURE_TRACK tMeasureData;
		if (E_NO_CONTINUOUS == tBoardInfo.eContinueType && 2 != i ||
			(tBoardInfo.nContinueStartNo > i || i > tBoardInfo.nContinueStartNo + tBoardInfo.nContinueNum - 1) ||
			!tBoardInfo.vbBoardDoesItExist.at(i))// 自身焊接或不存在
		{
			// 调试用数据
			tBoardInfo.vtMeasureTrack.push_back(tMeasureData);
			tBoardInfo.vvtMeasure_Data_Start.push_back(tMeasureData.vtMeasurePointStart);
			tBoardInfo.vvtMeasure_Data_Mind.push_back(tMeasureData.vtMeasurePointStart);
			tBoardInfo.vvtMeasure_Data_End.push_back(tMeasureData.vtMeasurePointStart);
			tBoardInfo.vvtMeasure_Data_Warp.push_back(tMeasureData.vtMeasureWarp);
			tBoardInfo.vvtMeasure_Data_ProEnd.push_back(tMeasureData.vtMeasureProEnd);
			tBoardInfo.vvtMeasure_Data_ProStart.push_back(tMeasureData.vtMeasureProStart);
			tBoardInfo.vvtMeasure_Pulse_Mind.push_back(tMeasureData.vtMeasurePulse);
			tBoardInfo.vvtMeasure_Pulse_Start.push_back(tMeasureData.vtMeasurePulseStart);
			tBoardInfo.vvtMeasure_Pulse_End.push_back(tMeasureData.vtMeasurePulseEnd);
			tBoardInfo.vvtMeasure_Pulse_Warp.push_back(tMeasureData.vtMeasurePulseWarp);
			tBoardInfo.vvtMeasure_Pulse_ProEnd.push_back(tMeasureData.vtMeasurePulseProEnd);
			tBoardInfo.vvtMeasure_Pulse_ProStart.push_back(tMeasureData.vtMeasurePulseProStart);
			continue;
		}
		vector<T_ROBOT_COORS> vtScanCoors; // 搜索起终点数据
		bool bIsFixedPointScan = false;
		vector<T_ROBOT_COORS> vtTeachTrack;// 中间测量数据
		vector<T_ROBOT_COORS> vtMeasurePoint; // 起终点测量数据
		vector<T_ANGLE_PULSE> vtMeasurePulse;
		vector<int> vnMeasureType; // 测量类型

		if (E_SEARCH_ENDPOINT == tBoardInfo.vtBoardPartInfo.at(i).eGetEndMethod)// 终点搜索
		{
			T_ANGLE_PULSE tRefPulse = pRobotDriver->m_tHomePulse;
			CHECK_BOOL_RETURN(
				CalcScanPos(
					pRobotDriver,
					tBoardInfo.vtCornerRelsve_Info.at(i).dEndAngle,
					tBoardInfo.vtBoardPartInfo.at(i).tEndPoint,
					tMeasureData.vtMeasurePointEnd,
					tMeasureData.vtMeasurePulseEnd,
					tMeasureData.vnMeasureTypeEnd,
					tMeasureData.dExAxisPosEnd,
					true,
					bIsFixedPointScan,
					&tRefPulse
				)
			);
			tBoardInfo.vvtMeasure_Data_End.push_back(tMeasureData.vtMeasurePointEnd);
			tBoardInfo.vvtMeasure_Pulse_End.push_back(tMeasureData.vtMeasurePulseEnd);
		}
		else if (E_MEASURE_ENDPOINT == tBoardInfo.vtBoardPartInfo.at(i).eGetEndMethod)// 终点测量
		{
			T_ANGLE_PULSE tRefPulse = pRobotDriver->GetCurrentPulse();
			CHECK_BOOL_RETURN(
				GetVerticalMeasureDataNew(
					pRobotDriver,
					tBoardInfo,
					tMeasureData.vtMeasurePointEnd,
					tMeasureData.vtMeasurePulseEnd,
					tMeasureData.vnMeasureTypeEnd,
					tMeasureData.dExAxisPosEnd,
					i,
					false,
					&tRefPulse
				)
			);
			tBoardInfo.vvtMeasure_Data_End.push_back(tMeasureData.vtMeasurePointEnd);
			tBoardInfo.vvtMeasure_Pulse_End.push_back(tMeasureData.vtMeasurePulseEnd);
		}
		else
		{
			vector<T_ROBOT_COORS> vtTempCoors;
			vector<T_ANGLE_PULSE> vtTempPulse;
			T_ROBOT_COORS tCoor(tBoardInfo.vtBoardPartInfo.at(i).tEndPoint.x, tBoardInfo.vtBoardPartInfo.at(i).tEndPoint.y, tBoardInfo.vtBoardPartInfo.at(i).tEndPoint.z,
				0, 0, 0, 0, 0, 0);
			T_ANGLE_PULSE tPulse(0, 0, 0, 0, 0, 0, 0, 0, 0);
			vtTempCoors.push_back(tCoor);
			vtTempPulse.push_back(tPulse);
			tBoardInfo.vvtMeasure_Data_End.push_back(vtTempCoors);
			tBoardInfo.vvtMeasure_Pulse_End.push_back(vtTempPulse);
			tMeasureData.dExAxisPosEnd = 0.0;
		}
		if (tBoardInfo.vtBoardPartInfo.at(i).bIsMiddleMeasure)// 中间测量
		{
			T_ANGLE_PULSE tRefPulse;
			if (tMeasureData.vtMeasurePulseEnd.size() == 0)
			{
				tRefPulse = pRobotDriver->GetCurrentPulse();
			}
			else
			{
				tRefPulse = tMeasureData.vtMeasurePulseEnd.back();
			}
			CHECK_BOOL_RETURN(
				GetTeachTrackNew(
					pRobotDriver,
					tBoardInfo,
					i,
					tMeasureData.vtMeasurePoint,
					tMeasureData.vtMeasurePulse,
					tMeasureData.vnMeasureType,
					&tRefPulse
				)
			);
			tBoardInfo.vvtMeasure_Data_Mind.push_back(tMeasureData.vtMeasurePoint);
			tBoardInfo.vvtMeasure_Pulse_Mind.push_back(tMeasureData.vtMeasurePulse);
		}
		else
		{
			vector<T_ROBOT_COORS> vtTempCoors;
			vector<T_ANGLE_PULSE> vtTempPulse;
			tBoardInfo.vvtMeasure_Data_Mind.push_back(vtTempCoors);
			tBoardInfo.vvtMeasure_Pulse_Mind.push_back(vtTempPulse);
		}
		if (E_SEARCH_ENDPOINT == tBoardInfo.vtBoardPartInfo.at(i).eGetStartMethod)// 起点点搜索
		{
			//用终点搜索或中间测量的抬枪点作为参考点搜索连续的Pulse
			T_ANGLE_PULSE tRefPulse;
			if (tMeasureData.vtMeasurePulseEnd.size() == 0 && tMeasureData.vtMeasurePulse.size() == 0)
			{
				tRefPulse = pRobotDriver->m_tHomePulse;
			}
			else
			{
				tRefPulse = tMeasureData.vtMeasurePulseEnd.size() == 0 ?
					tMeasureData.vtMeasurePulse.back() : tMeasureData.vtMeasurePulseEnd.back();
			}
			CHECK_BOOL_RETURN(
				CalcScanPos(
					pRobotDriver,
					tBoardInfo.vtCornerRelsve_Info.at(i).dStartAngle,
					tBoardInfo.vtBoardPartInfo.at(i).tStartPoint,
					tMeasureData.vtMeasurePointStart,
					tMeasureData.vtMeasurePulseStart,
					tMeasureData.vnMeasureTypeStart,
					tMeasureData.dExAxisPosStart,
					false,
					bIsFixedPointScan,
					&tRefPulse
				)
			);
			tBoardInfo.vvtMeasure_Data_Start.push_back(tMeasureData.vtMeasurePointStart);
			tBoardInfo.vvtMeasure_Pulse_Start.push_back(tMeasureData.vtMeasurePulseStart);
			tBoardInfo.vvtMeasure_Data_ProStart.push_back(vector<T_ROBOT_COORS>(0));
			tBoardInfo.vvtMeasure_Pulse_ProStart.push_back(vector<T_ANGLE_PULSE>(0));
		}
		else if (E_MEASURE_ENDPOINT == tBoardInfo.vtBoardPartInfo.at(i).eGetStartMethod)// 起点点测量
		{
			T_ANGLE_PULSE tRefPulse;
			if (tMeasureData.vtMeasurePulseEnd.size() == 0 && tMeasureData.vtMeasurePulse.size() == 0)
			{
				tRefPulse = pRobotDriver->GetCurrentPulse();
			}
			else
			{
				tRefPulse = tMeasureData.vtMeasurePulseEnd.size() == 0 ?
					tMeasureData.vtMeasurePulse.back() : tMeasureData.vtMeasurePulseEnd.back();
			}

			CHECK_BOOL_RETURN(
				GetVerticalMeasureDataNew(
					pRobotDriver,
					tBoardInfo,
					tMeasureData.vtMeasurePointStart,
					tMeasureData.vtMeasurePulseStart,
					tMeasureData.vnMeasureTypeStart,
					tMeasureData.dExAxisPosStart,
					i,
					true,
					&tRefPulse
				)
			);

			tBoardInfo.vvtMeasure_Data_Start.push_back(tMeasureData.vtMeasurePointStart);
			tBoardInfo.vvtMeasure_Pulse_Start.push_back(tMeasureData.vtMeasurePulseStart);
			tBoardInfo.vvtMeasure_Data_ProStart.push_back(vector<T_ROBOT_COORS>(0));
			tBoardInfo.vvtMeasure_Pulse_ProStart.push_back(vector<T_ANGLE_PULSE>(0));
		}
		else if (E_MEASURE_PROJPOINT == tBoardInfo.vtBoardPartInfo.at(i).eGetStartMethod)
		{
			XI_POINT tct = { 0,0,0 };
			vector<XI_POINT> vtTrack;
			T_ANGLE_PULSE tRefPulse;
			if (tMeasureData.vtMeasurePulseEnd.size() == 0 && tMeasureData.vtMeasurePulse.size() == 0)
			{
				tRefPulse = pRobotDriver->GetCurrentPulse();
			}
			else
			{
				tRefPulse = tMeasureData.vtMeasurePulseEnd.size() == 0 ?
					tMeasureData.vtMeasurePulse.back() : tMeasureData.vtMeasurePulseEnd.back();
			}
			XI_POINT tStartPoint = tBoardInfo.vtBoardPartInfo.at(i).tStartPoint;
			XI_POINT tEndPoint = tBoardInfo.vtBoardPartInfo.at(i).tEndPoint;
			vector<T_ROBOT_COORS> vtWeldTrack;
			//根据线扫结果生成 起→终（有向）点轨迹
			GenerateTrack(vtTrack, tStartPoint, tEndPoint, tct);
			CalcMeasurePosInBase(vtTrack, 15, 0, pRobotDriver->DirAngleToRz(tBoardInfo.vtCornerRelsve_Info.at(i).dStartAngle + 180), vtTrack);
			if (vtTrack.size() <= m_dHandEyeDis + m_dMeasureDisThreshold * 5)
			{
				//已修改
				XUI::MesBox::PopInfo("识别出焊道距离小于：{0:.2f}，无法完成测量", m_dHandEyeDis + m_dMeasureDisThreshold * 5);
				//XiMessageBoxOk("识别出焊道距离小于：%.2f，无法完成测量", m_dHandEyeDis + m_dMeasureDisThreshold * 5);
			}
			vtTrack.erase(vtTrack.begin() + m_dMeasureDisThreshold * 5 + m_dHandEyeDis, vtTrack.end());
			SaveDataXiPoint(vtTrack, tBoardInfo.vtBoardPartInfo.at(i).strfileName + "OnceWarpMeasureProjS.txt");
			CHECK_BOOL_RETURN(CalcRobotCoorsAccordingTrack(vtTrack, vtWeldTrack));
			CHECK_BOOL_RETURN(
				GetMeasureCoors(
					pRobotDriver,
					vtWeldTrack,
					tMeasureData.vtMeasurePointStart,
					tMeasureData.vnMeasureTypeStart,
					false,
					tBoardInfo.vtCornerRelsve_Info.at(i).bBoardType,
					true,
					tBoardInfo.vtCornerRelsve_Info.at(i).nEndType > 0,
					m_dMeasureDisThreshold
				)
			);
			CHECK_BOOL_RETURN(
				TransCameraToolAndCalcContinuePulse(
					pRobotDriver,
					tMeasureData.vtMeasurePointStart,
					tMeasureData.vtMeasurePulseStart,
					tMeasureData.vnMeasureTypeStart,
					&tRefPulse
				)
			);
			///////////////////////
			tBoardInfo.vvtMeasure_Data_Start.push_back(tMeasureData.vtMeasurePointStart);
			tBoardInfo.vvtMeasure_Pulse_Start.push_back(tMeasureData.vtMeasurePulseStart);
			// 保存测量点轨迹
			string strFileName = tBoardInfo.vtBoardPartInfo[i].strfileName + "ProjStrMeaCoords.txt";
			SaveDataRobotCoors(tMeasureData.vtMeasurePointStart, strFileName);
			//生成用于投影用的点的扫描轨迹
			tRefPulse = pRobotDriver->m_tHomePulse;
			double dBoardThick = 15.0;
			XI_POINT tmpStrXiPtn;
			CalcOffsetInBase(
				tBoardInfo.vtBoardPartInfo.at(i).tStartPoint,
				dBoardThick,
				0.0,
				tBoardInfo.vtCornerRelsve_Info.at(i).dStartAngle + 180.0,
				tmpStrXiPtn,
				m_nRobotInstallDir > 0
			);

			CHECK_BOOL_RETURN(
				CalcScanPos(
					pRobotDriver,
					tBoardInfo.vtCornerRelsve_Info.at(i).dStartAngle + 180.0,
					tmpStrXiPtn,
					tMeasureData.vtMeasureProStart,
					tMeasureData.vtMeasurePulseProStart,
					tMeasureData.vnMeasureProStart,
					tMeasureData.dExAxisPosProStart,
					true,
					bIsFixedPointScan,
					&tRefPulse
				)
			);
			tBoardInfo.vvtMeasure_Data_ProStart.push_back(tMeasureData.vtMeasureProStart);
			tBoardInfo.vvtMeasure_Pulse_ProStart.push_back(tMeasureData.vtMeasurePulseProStart);
		}
		else
		{
			vector<T_ROBOT_COORS> vtTempCoors;
			vector<T_ANGLE_PULSE> vtTempPulse;
			T_ROBOT_COORS tCoor(tBoardInfo.vtBoardPartInfo.at(i).tStartPoint.x, tBoardInfo.vtBoardPartInfo.at(i).tStartPoint.y, tBoardInfo.vtBoardPartInfo.at(i).tStartPoint.z,
				0, 0, 0, 0, 0, 0);
			T_ANGLE_PULSE tPulse(0, 0, 0, 0, 0, 0, 0, 0, 0);
			vtTempCoors.push_back(tCoor);
			vtTempPulse.push_back(tPulse);
			tBoardInfo.vvtMeasure_Data_Start.push_back(vtTempCoors);
			tBoardInfo.vvtMeasure_Pulse_Start.push_back(vtTempPulse);
			tBoardInfo.vvtMeasure_Data_ProStart.push_back(vtTempCoors);
			tBoardInfo.vvtMeasure_Pulse_ProStart.push_back(vtTempPulse);
			tMeasureData.dExAxisPosStart = 0.0;
		}
		//一次包角，且为投影测量模式
		if (E_WRAPANGLE_ONCE == tBoardInfo.vtBoardPartInfo.at(i).eWarpType && tBoardInfo.vtBoardPartInfo.at(i).eGetStartMethod
			&& !tBoardInfo.vtCornerRelsve_Info.at(i).bBoardType)
		{
			XI_POINT tct = { 0,0,0 };
			vector<XI_POINT> vtTrack;
			XI_POINT tStartPoint = tBoardInfo.vtBoardPartInfo.at(i).tEndPoint;
			XI_POINT tEndPoint = tBoardInfo.vtBoardPartInfo.at(i).tStartPoint;
			T_ANGLE_PULSE tRefPulse = tMeasureData.vtMeasurePulseProStart.back();
			vector<T_ROBOT_COORS> vtWeldTrack;
			//根据线扫结果生成 起→终（有向）点轨迹
			GenerateTrack(vtTrack, tStartPoint, tEndPoint, tct);
			if (vtTrack.size() <= m_dHandEyeDis + m_dMeasureDisThreshold * 5)
			{
				//已修改
				XUI::MesBox::PopInfo("识别出焊道距离小于：{0:.2f}，无法完成测量", m_dHandEyeDis + m_dMeasureDisThreshold * 5);
				//XiMessageBoxOk("识别出焊道距离小于：%.2f，无法完成测量", m_dHandEyeDis + m_dMeasureDisThreshold * 5);
			}
			//将轨迹朝向焊缝方向偏移20
			CalcMeasurePosInBase(vtTrack, 15, 0, pRobotDriver->DirAngleToRz(tBoardInfo.vtCornerRelsve_Info.at(i).dStartAngle + 180), vtTrack);
			vtTrack.erase(vtTrack.begin() + m_dMeasureDisThreshold * 5 + m_dHandEyeDis, vtTrack.end());
			SaveDataXiPoint(vtTrack, tBoardInfo.vtBoardPartInfo.at(i).strfileName + "OnceWarpMeasureProjE.txt");
			CHECK_BOOL_RETURN(CalcRobotCoorsAccordingTrack(vtTrack, vtWeldTrack));
			CHECK_BOOL_RETURN(
				GetMeasureCoors(
					pRobotDriver,
					vtWeldTrack,
					tMeasureData.vtMeasureProEnd,
					tMeasureData.vnMeasureProEnd,
					false,
					tBoardInfo.vtCornerRelsve_Info.at(i).bBoardType,
					false,
					tBoardInfo.vtCornerRelsve_Info.at(i).nEndType > 0,
					m_dMeasureDisThreshold
				)
			);
			
			CHECK_BOOL_RETURN(
				TransCameraToolAndCalcContinuePulse(
					pRobotDriver,
					tMeasureData.vtMeasureProEnd,
					tMeasureData.vtMeasurePulseProEnd,
					tMeasureData.vnMeasureProEnd,
					&tRefPulse
				)
			);
			///////////////////////
			tBoardInfo.vvtMeasure_Data_ProEnd.push_back(tMeasureData.vtMeasureProEnd);
			tBoardInfo.vvtMeasure_Pulse_ProEnd.push_back(tMeasureData.vtMeasurePulseProEnd);
			// 保存测量点轨迹
			string strFileName = tBoardInfo.vtBoardPartInfo[i].strfileName + "ProjEndMeaCoords.txt";
			SaveDataRobotCoors(tMeasureData.vtMeasurePointEnd, strFileName);
			// 原先的流程加上包角轨迹为空 保证vvt结构正确
			vector<T_ROBOT_COORS> vtTempCoor;
			vector<T_ANGLE_PULSE> vtTempPulse;
			tBoardInfo.vvtMeasure_Data_Warp.push_back(vtTempCoor);
			tBoardInfo.vvtMeasure_Pulse_Warp.push_back(vtTempPulse);
		}
		//默认完全不干涉 一次包角
		else if (E_WRAPANGLE_ONCE == tBoardInfo.vtBoardPartInfo.at(i).eWarpType && !tBoardInfo.vtCornerRelsve_Info.at(i).bBoardType)// 一次包角时
		{
			T_ANGLE_PULSE tRefPulse = pRobotDriver->m_tHomePulse;
			double dBoardThick = 15.0;
			XI_POINT tmpStrXiPtn;
			CalcOffsetInBase(
				tBoardInfo.vtBoardPartInfo.at(i).tStartPoint,
				dBoardThick,
				0.0,
				tBoardInfo.vtCornerRelsve_Info.at(i).dStartAngle + 180.0,
				tmpStrXiPtn,
				m_nRobotInstallDir > 0
			);

			CHECK_BOOL_RETURN(
				CalcScanPos(
					pRobotDriver,
					tBoardInfo.vtCornerRelsve_Info.at(i).dStartAngle + 180.0,
					tmpStrXiPtn,
					tMeasureData.vtMeasureWarp,
					tMeasureData.vtMeasurePulseWarp,
					tMeasureData.vnMeasureTypeWarp,
					tMeasureData.dExAxisPosWarp,
					true,
					bIsFixedPointScan,
					&tRefPulse
				)
			);

			//XI_POINT tct = { 0,0,0 };
			//vector<XI_POINT> vtTrack;
			//XI_POINT tStartPoint = tBoardInfo.vtBoardPartInfo.at(i).tEndPoint;
			//XI_POINT tEndPoint = tBoardInfo.vtBoardPartInfo.at(i).tStartPoint;
			//vector<T_ROBOT_COORS> vtWeldTrack;
			////根据线扫结果生成起终点轨迹
			//GenerateTrack(vtTrack, tStartPoint, tEndPoint, tct);
			////将轨迹朝向焊缝方向偏移20
			//CalcMeasurePosInBase(vtTrack, 15, 0, pRobotDriver->DirAngleToRz(tBoardInfo.vtCornerRelsve_Info.at(i).dStartAngle + 180), vtTrack);
			//if (vtTrack.size() < m_dHandEyeDis + m_dMeasureDisThreshold * 5)
			//{
			//	XiMessageBoxOk("错误！识别到的焊缝长度小于%dmm，无法完成测量", m_dHandEyeDis + m_dMeasureDisThreshold * 5);
			//	return false;
			//}
			//SaveDataXiPoint(vtTrack, tBoardInfo.vtBoardPartInfo.at(i).strfileName + "OnceWarpMeasureCoorsAfter.txt");
			//CHECK_BOOL_RETURN(CalcRobotCoorsAccordingTrack(vtTrack, vtWeldTrack));
			//// 20231027改双边测量
			//vector<T_ROBOT_COORS> vtMeasureSimLine;
			//vector<T_ANGLE_PULSE> vtMeasurePulseSimLine;
			//vector<T_ROBOT_COORS> vtLineStart, vtLineEnd;
			//vector<int> vnMeasureTypeSimLine;
			//vtLineStart.insert(vtLineStart.end(), vtWeldTrack.rbegin(), vtWeldTrack.rbegin() + m_dMeasureDisThreshold * 5 + m_dHandEyeDis);
			//vtLineEnd.insert(vtLineEnd.end(), vtWeldTrack.begin(), vtWeldTrack.begin() + m_dMeasureDisThreshold * 5 + m_dHandEyeDis);
			//////背面起点处测量
			//CHECK_BOOL_RETURN(GetMeasureCoors(
			//	pRobotDriver,
			//	vtLineStart,
			//	vtMeasureSimLine,
			//	vnMeasureTypeSimLine,
			//	false,
			//	tBoardInfo.vtCornerRelsve_Info.at(i).bBoardType,
			//	true,
			//	tBoardInfo.vtCornerRelsve_Info.at(i).nEndType,
			//	m_dMeasureDisThreshold)
			//);
			//if (tMeasureData.vtMeasurePulseStart.size() == 0)
			//{
			//	XiMessageBoxOk("错误，寻找测量投影面的下枪参考点失败！");
			//	return false;
			//}
			//T_ANGLE_PULSE tRefPulse = m_pRobotDriver->m_tHomePulse;
			//CHECK_BOOL_RETURN(TransCameraToolAndCalcContinuePulse(
			//	pRobotDriver,
			//	vtMeasureSimLine,
			//	vtMeasurePulseSimLine,
			//	vnMeasureTypeSimLine,
			//	&tRefPulse)
			//);
			//tMeasureData.vtMeasureWarp.insert(tMeasureData.vtMeasureWarp.end(), vtMeasureSimLine.begin(), vtMeasureSimLine.end());
			//tMeasureData.vtMeasurePulseWarp.insert(tMeasureData.vtMeasurePulseWarp.end(), vtMeasurePulseSimLine.begin(), vtMeasurePulseSimLine.end());
			//tMeasureData.vnMeasureTypeWarp.insert(tMeasureData.vnMeasureTypeWarp.end(), vnMeasureTypeSimLine.begin(), vnMeasureTypeSimLine.end());


			////背面终点处测量
			//CHECK_BOOL_RETURN(
			//	GetMeasureCoors(
			//		pRobotDriver,
			//		vtLineEnd,
			//		vtMeasureSimLine,
			//		vnMeasureTypeSimLine,
			//		false,
			//		tBoardInfo.vtCornerRelsve_Info.at(i).bBoardType,
			//		false,
			//		tBoardInfo.vtCornerRelsve_Info.at(i).nEndType,
			//		m_dMeasureDisThreshold
			//	)
			//);
			//CHECK_BOOL_RETURN(
			//	TransCameraToolAndCalcContinuePulse(
			//		pRobotDriver,
			//		vtMeasureSimLine,
			//		vtMeasurePulseSimLine,
			//		vnMeasureTypeSimLine,
			//		&tRefPulse
			//	)
			//);
			//	tMeasureData.vtMeasureWarp.insert(tMeasureData.vtMeasureWarp.end(), vtMeasureSimLine.rbegin(), vtMeasureSimLine.rend());
			//	tMeasureData.vtMeasurePulseWarp.insert(tMeasureData.vtMeasurePulseWarp.end(), vtMeasurePulseSimLine.rbegin(), vtMeasurePulseSimLine.rend());
			//	tMeasureData.vnMeasureTypeWarp.insert(tMeasureData.vnMeasureTypeWarp.end(), vnMeasureTypeSimLine.rbegin(), vnMeasureTypeSimLine.rend());
			tBoardInfo.vvtMeasure_Data_Warp.push_back(tMeasureData.vtMeasureWarp);
			tBoardInfo.vvtMeasure_Pulse_Warp.push_back(tMeasureData.vtMeasurePulse);
			SaveDataRobotCoors(tMeasureData.vtMeasureWarp, tBoardInfo.vtBoardPartInfo.at(i).strfileName + "OnceWarpTheroyMeasureCoors.txt");
			tBoardInfo.vvtMeasure_Data_ProEnd.push_back(vector<T_ROBOT_COORS>(0));
			tBoardInfo.vvtMeasure_Pulse_ProEnd.push_back(vector<T_ANGLE_PULSE>(0));
		}
		else
		{
			tBoardInfo.vvtMeasure_Data_Warp.push_back(vector<T_ROBOT_COORS>(0));
			tBoardInfo.vvtMeasure_Pulse_Warp.push_back(vector<T_ANGLE_PULSE>(0));
			tBoardInfo.vvtMeasure_Data_ProEnd.push_back(vector<T_ROBOT_COORS>(0));
			tBoardInfo.vvtMeasure_Pulse_ProEnd.push_back(vector<T_ANGLE_PULSE>(0));
		}
		tBoardInfo.vtMeasureTrack.push_back(tMeasureData);
	}
	return true;
}

bool CDiaphragmWeld::GetRealTeachData(CRobotDriverAdaptor* pRobotDriver, T_BOARD_INFO_NEW& tBoardInfo, VT_ENDPOINT_INFO_NEW* pvtPointInfo)
{

	if (tBoardInfo.vtCornerRelsve_Info.size() < 5)
	{
		XiMessageBoxOk("关联焊缝数据有误,请检查数据");
		return false;
	}
	T_ROBOT_MOVE_SPEED tMove(m_dFastApproachToWorkSpeed, 20, 20);
	vector<T_ALGORITHM_POINT> vtTeachResult, vtTeachResultWarp;
	T_LINE_PARA tLineStart, tLineProEnd;
	for (int i = 0; i < tBoardInfo.vtCornerRelsve_Info.size(); i++)
	{
		vtTeachResult.clear();
		vtTeachResultWarp.clear();
		if (E_NO_CONTINUOUS == tBoardInfo.eContinueType && 2 != i ||
			(tBoardInfo.nContinueStartNo > i || i > tBoardInfo.nContinueStartNo + tBoardInfo.nContinueNum - 1) ||
			!tBoardInfo.vbBoardDoesItExist.at(i))// 自身焊接或不存在	 
		{
			tBoardInfo.vvtMeasureResult_Info.push_back(vtTeachResult);
			tBoardInfo.vvtMeasureResult_Warp.push_back(vtTeachResultWarp);
			continue;
		}
		if (LoadWeldStatus(tBoardInfo.vtCornerRelsve_Info.at(i).nBoardNo) || tBoardInfo.vtCornerRelsve_Info.at(i).bIfWeldCom)
		{
			tBoardInfo.vvtMeasureResult_Info.push_back(vtTeachResult);
			tBoardInfo.vvtMeasureResult_Warp.push_back(vtTeachResultWarp);
			continue;
		}
		if (ReadCorrectEndPointFun(pRobotDriver, tBoardInfo.vtCornerRelsve_Info.at(i).nStartNo, tBoardInfo.vtBoardPartInfo.at(i).tStartPoint))//加载更新端点存在 
		{
			GetNaturalPop().PopInfo("{0}：号工件 起点：{1} 号端点在过程中被更新", tBoardInfo.vtCornerRelsve_Info.at(i).nBoardNo, tBoardInfo.vtCornerRelsve_Info.at(i).nStartNo);
			if (E_MEASURE_ENDPOINT == tBoardInfo.vtBoardPartInfo.at(i).eGetStartMethod)
			{
				T_ALGORITHM_POINT tPEnd = { tBoardInfo.vtBoardPartInfo.at(i).tStartPoint.x,
					tBoardInfo.vtBoardPartInfo.at(i).tStartPoint.y,tBoardInfo.vtBoardPartInfo.at(i).tStartPoint.z };
				vtTeachResult.push_back(tPEnd);
			}
			tBoardInfo.vtBoardPartInfo.at(i).eGetStartMethod = E_UPDATED_ENDPOINT;
		}		
		
		if (E_SEARCH_ENDPOINT == tBoardInfo.vtBoardPartInfo.at(i).eGetStartMethod)
		{
			if (!GetNaturalPop().PopOkCancel("开始搜索起始端点")) return false;
			m_pScanInit->m_bFixedPointScan = tBoardInfo.vtBoardPartInfo.at(i).bFixedPointScanEnd;
			//扫描运动数据计算  
			if (!g_bLocalDebugMark)
			{
				m_ptUnit->SwitchDHCamera(m_ptUnit->m_nMeasureCameraNo, true, E_CALL_BACK_MODE_WAIT_IMAGE, E_ACQUISITION_MODE_CONTINUE);
				CHECK_BOOL_RETURN(
					SetScanPos(
						pRobotDriver,
						tBoardInfo.vtMeasureTrack.at(i).vtMeasurePointStart,
						tBoardInfo.vtMeasureTrack.at(i).vtMeasurePulseStart,
						tBoardInfo.vtMeasureTrack.at(i).vnMeasureTypeStart
					)
				);
				m_pScanInit->m_eEndPointType = GROUP_STAND_DIP_NS::E_PIECE_START;
				CHECK_BOOL_RETURN(m_pScanInit->DynamicScanningEndpoint(pRobotDriver));
				m_ptUnit->SwitchDHCamera(m_ptUnit->m_nMeasureCameraNo,false);
				CString cstr;
				XI_POINT tDynamicScanEndPoint = m_pScanInit->GetDynamicScanEndPoint();
				cstr.Format("原始搜索起点结果 x:%.3f\ty:%.3f\tz:%.3f\n", tDynamicScanEndPoint.x, tDynamicScanEndPoint.y, tDynamicScanEndPoint.z);
				TRACE(cstr);		
				
				//将搜索端点投影到搜索线段上
				T_LINE_PARA tLine = CalcLineParamRansac(m_pScanInit->GetVtImgProcResult3D(), 0.8);
				//2.投影
				PointtoLineProjection(tLine, tDynamicScanEndPoint, tDynamicScanEndPoint);
				m_pScanInit->SetDynamicScanEndPoint(tDynamicScanEndPoint);
				//搜搜起点补偿3mm
				if (!tBoardInfo.vtCornerRelsve_Info.at(i).bBoardType)
				{
					/*double dOffset = -9.0;
					m_pScanInit->m_tDynamicScanEndPoint.x = m_pScanInit->m_tDynamicScanEndPoint.x + dOffset * CosD(tBoardInfo.vtCornerRelsve_Info.at(i).dStartAngle - 90 * m_nRobotInstallDir);
					m_pScanInit->m_tDynamicScanEndPoint.y = m_pScanInit->m_tDynamicScanEndPoint.y + dOffset * SinD(tBoardInfo.vtCornerRelsve_Info.at(i).dStartAngle - 90 * m_nRobotInstallDir);*/
				}
				cstr.Format("修正搜索起点结果 x:%.3f\ty:%.3f\tz:%.3f\n", tDynamicScanEndPoint.x, tDynamicScanEndPoint.y, tDynamicScanEndPoint.z);
				
				TRACE(cstr);

				tBoardInfo.vtBoardPartInfo.at(i).tStartPoint = m_pScanInit->GetDynamicScanEndPoint();
				T_ALGORITHM_POINT tPEnd = *(T_ALGORITHM_POINT*)&tDynamicScanEndPoint;
				vtTeachResult.push_back(tPEnd);
				//获取扫描数据
				vector<XI_POINT> vtxiData; vector<T_ROBOT_COORS> vtRealPoint; double dDirAngle;
				CHECK_BOOL_RETURN(m_pScanInit->GetScanData(pRobotDriver, vtxiData, vtRealPoint, dDirAngle));
				SaveDataRobotCoors(vtRealPoint, tBoardInfo.vtBoardPartInfo.at(i).strfileName + "TrackStartCoors.txt");
				//移动到安全位置
				T_ROBOT_COORS tRobotCoor = *tBoardInfo.vtMeasureTrack[i].vtMeasurePointStart.rbegin();
				pRobotDriver->MoveByJob(tRobotCoor, tMove, pRobotDriver->m_nExternalAxleType, "MOVL");
				CHECK_BOOL_RETURN(m_ptUnit->CheckRobotDone(tRobotCoor));
			}
			else
			{
				T_ALGORITHM_POINT tPEnd = { tBoardInfo.vtBoardPartInfo.at(i).tStartPoint.x,
					tBoardInfo.vtBoardPartInfo.at(i).tStartPoint.y,tBoardInfo.vtBoardPartInfo.at(i).tStartPoint.z };
				vtTeachResult.push_back(tPEnd);
			}
		}
		else if (E_MEASURE_ENDPOINT == tBoardInfo.vtBoardPartInfo.at(i).eGetStartMethod)
		{
			XI_POINT tIntersection;
			// 开始移动到安全位置
			CHECK_BOOL_RETURN(PosMoveMachine(pRobotDriver, tBoardInfo.vtMeasureTrack.at(i).dExAxisPosStart, m_dSafePosRunSpeed * 1.5, COORD_ABS));
			//pRobotDriver->SafeMoveJByJobSaxis(tBoardInfo.vtMeasureTrack.at(i).vtMeasurePulseStart[0], tMove);
			CHECK_BOOL_RETURN(m_ptUnit->CheckRobotDone(tBoardInfo.vtMeasureTrack.at(i).vtMeasurePulseStart[0]));
			// 开始示教	
			if (!g_bLocalDebugMark)
			{
				CHECK_BOOL_RETURN(DoTeach(tBoardInfo.vtCornerRelsve_Info.at(i).nBoardNo,
					tBoardInfo.vtMeasureTrack.at(i).vtMeasurePulseStart,
					tBoardInfo.vtMeasureTrack.at(i).vnMeasureTypeStart,
					tBoardInfo.vtMeasureTrack.at(i).dExAxisPosStart
				));
			}
			else
			{
				m_vtTeachResult.clear();
				for (int n = 0; n < tBoardInfo.vvtMeasure_Data_Start.at(i).size(); n++)
				{
					T_ROBOT_COORS tCoor = tBoardInfo.vvtMeasure_Data_Start.at(i).at(n);
					T_TEACH_RESULT tTeach;
					tTeach.tKeyPtn3D.x = tCoor.dX;
					tTeach.tKeyPtn3D.y = tCoor.dY;
					tTeach.tKeyPtn3D.z = tCoor.dZ;
					m_vtTeachResult.push_back(tTeach);
				}
			}
			//计算交点
			CHECK_BOOL_RETURN(CalcMeasureIntersection(pRobotDriver, tBoardInfo, m_vtTeachResult,
				tBoardInfo.vtMeasureTrack.at(i).dExAxisPosStart, i, tIntersection, true));
			tBoardInfo.vtBoardPartInfo.at(i).tStartPoint = tIntersection;
			T_ALGORITHM_POINT tPEnd = { tIntersection.x,tIntersection.y,tIntersection.z };
			vtTeachResult.push_back(tPEnd);
		}
		if (E_WRAPANGLE_ONCE == tBoardInfo.vtBoardPartInfo.at(i).eWarpType && !tBoardInfo.vtCornerRelsve_Info.at(i).bBoardType&&E_MEASURE_PROJPOINT != tBoardInfo.vtBoardPartInfo.at(i).eGetStartMethod)// 一次包角时
		{
			//23.11.22 背面测点改搜一段轨迹
			if (!GetNaturalPop().PopOkCancel("开始搜索焊缝背面特征点？")) return false;
			m_ptUnit->SwitchDHCamera(m_ptUnit->m_nMeasureCameraNo, true, E_CALL_BACK_MODE_WAIT_IMAGE, E_ACQUISITION_MODE_CONTINUE); 
			CHECK_BOOL_RETURN(
				SetScanPos(
					pRobotDriver,
					tBoardInfo.vtMeasureTrack.at(i).vtMeasureWarp,
					tBoardInfo.vtMeasureTrack.at(i).vtMeasurePulseWarp,
					tBoardInfo.vtMeasureTrack.at(i).vnMeasureTypeWarp
				)
			);
			
			CHECK_BOOL_RETURN(
				m_pScanInit->DynamicScanningEndpoint(pRobotDriver)
			);
			m_ptUnit->SwitchDHCamera(m_ptUnit->m_nMeasureCameraNo,false);
			vtTeachResultWarp.clear();
			vector<XI_POINT> vtImgProcess3DPointResult = m_pScanInit->GetVtImgProcResult3D();
			for (auto itera = vtImgProcess3DPointResult.begin(); itera != vtImgProcess3DPointResult.end(); itera++)
			{
				T_ALGORITHM_POINT tAlgPoint = { itera->x, itera->y, itera->z };
				vtTeachResultWarp.push_back(tAlgPoint);
			}
			SaveDataAlgorithm(vtTeachResultWarp, tBoardInfo.vtBoardPartInfo.at(i).strfileName + "RealScanCoorsWarp.txt");
			//移动到安全位置
			T_ROBOT_COORS tRobotCoor = *tBoardInfo.vtMeasureTrack[i].vtMeasureWarp.rbegin();
			pRobotDriver->MoveByJob(tRobotCoor, tMove, pRobotDriver->m_nExternalAxleType, "MOVL");
			CHECK_BOOL_RETURN(m_ptUnit->CheckRobotDone(tRobotCoor));
		}
		if (tBoardInfo.vtBoardPartInfo.at(i).bIsMiddleMeasure)
		{
			CHECK_BOOL_RETURN(GetMeasureDataWithMoveNew(pRobotDriver, tBoardInfo, i, vtTeachResult));
		}
		if (ReadCorrectEndPointFun(pRobotDriver, tBoardInfo.vtCornerRelsve_Info.at(i).nEndNo, tBoardInfo.vtBoardPartInfo.at(i).tEndPoint))//加载更新端点存在 
		{
			GetNaturalPop().PopOkCancel("{0}：号工件 结尾点：{1} 号端点在过程中被更新", tBoardInfo.vtCornerRelsve_Info.at(i).nBoardNo, tBoardInfo.vtCornerRelsve_Info.at(i).nEndNo);
			if (E_MEASURE_ENDPOINT == tBoardInfo.vtBoardPartInfo.at(i).eGetEndMethod)
			{
				T_ALGORITHM_POINT tPEnd = { tBoardInfo.vtBoardPartInfo.at(i).tEndPoint.x,
					tBoardInfo.vtBoardPartInfo.at(i).tEndPoint.y,tBoardInfo.vtBoardPartInfo.at(i).tEndPoint.z };
				vtTeachResult.push_back(tPEnd);
			}
			tBoardInfo.vtBoardPartInfo.at(i).eGetStartMethod = E_UPDATED_ENDPOINT;
		}
		else if (E_SEARCH_ENDPOINT == tBoardInfo.vtBoardPartInfo.at(i).eGetEndMethod)
		{

			if (!GetNaturalPop().PopOkCancel("开始搜索结尾端点？"))  return false;
			m_pScanInit->m_bFixedPointScan = tBoardInfo.vtBoardPartInfo.at(i).bFixedPointScanEnd;
			//	搜索端点
			if (!g_bLocalDebugMark)
			{
				m_ptUnit->SwitchDHCamera(m_ptUnit->m_nMeasureCameraNo, true, E_CALL_BACK_MODE_WAIT_IMAGE, E_ACQUISITION_MODE_CONTINUE);
				CHECK_BOOL_RETURN(SetScanPos(pRobotDriver, tBoardInfo.vtMeasureTrack.at(i).vtMeasurePointEnd,
					tBoardInfo.vtMeasureTrack.at(i).vtMeasurePulseEnd, tBoardInfo.vtMeasureTrack.at(i).vnMeasureTypeEnd));
				m_pScanInit->m_eEndPointType = GROUP_STAND_DIP_NS::E_PIECE_END;

				CHECK_BOOL_RETURN(m_pScanInit->DynamicScanningEndpoint(pRobotDriver));
				m_ptUnit->SwitchDHCamera(m_ptUnit->m_nMeasureCameraNo,false);
				CString cstr;
				XI_POINT tDynamicScanEndPoint = m_pScanInit->GetDynamicScanEndPoint();
				cstr.Format("原始搜索终点结果 x:%.3f\ty:%.3f\tz:%.3f\n", tDynamicScanEndPoint.x, tDynamicScanEndPoint.y, tDynamicScanEndPoint.z);
				TRACE(cstr);

				//1.拟合搜索线段
				T_LINE_PARA tLine = CalcLineParamRansac(m_pScanInit->GetVtImgProcResult3D(), 0.8);
				//2.投影
				PointtoLineProjection(tLine, tDynamicScanEndPoint, tDynamicScanEndPoint);
				m_pScanInit->SetDynamicScanEndPoint(tDynamicScanEndPoint);
				//	搜搜结尾补偿3mm
				if (!tBoardInfo.vtCornerRelsve_Info.at(i).bBoardType)
				{
					/*double dOffset = -20;
					m_pScanInit->m_tDynamicScanEndPoint.x = m_pScanInit->m_tDynamicScanEndPoint.x + dOffset * CosD(tBoardInfo.vtCornerRelsve_Info.at(i).dStartAngle + 90 * m_nRobotInstallDir);
					m_pScanInit->m_tDynamicScanEndPoint.y = m_pScanInit->m_tDynamicScanEndPoint.y + dOffset * SinD(tBoardInfo.vtCornerRelsve_Info.at(i).dStartAngle + 90 * m_nRobotInstallDir);*/
				}
				cstr.Format("修正搜索终点结果 x:%.3f\ty:%.3f\tz:%.3f\n", tDynamicScanEndPoint.x, tDynamicScanEndPoint.y, tDynamicScanEndPoint.z);
				TRACE(cstr);

				tBoardInfo.vtBoardPartInfo.at(i).tEndPoint = m_pScanInit->GetDynamicScanEndPoint();
				T_ALGORITHM_POINT tPEnd = *(T_ALGORITHM_POINT*)&tDynamicScanEndPoint;
				vtTeachResult.push_back(tPEnd);
				//	移动到安全位置
				T_ROBOT_COORS tRobotCoor = *tBoardInfo.vtMeasureTrack[i].vtMeasurePointEnd.rbegin();
				pRobotDriver->MoveByJob(tRobotCoor, tMove, pRobotDriver->m_nExternalAxleType, "MOVL");
				CHECK_BOOL_RETURN(m_ptUnit->CheckRobotDone(tRobotCoor));
			}
			else
			{	// 调试使用
				T_ALGORITHM_POINT tPEnd = { tBoardInfo.vtBoardPartInfo.at(i).tEndPoint.x,
					tBoardInfo.vtBoardPartInfo.at(i).tEndPoint.y,tBoardInfo.vtBoardPartInfo.at(i).tEndPoint.z };
				vtTeachResult.push_back(tPEnd);
			}
		}
		else if (E_MEASURE_ENDPOINT == tBoardInfo.vtBoardPartInfo.at(i).eGetEndMethod)
		{
			XI_POINT tIntersection;
			// 开始移动到安全位置
			CHECK_BOOL_RETURN(PosMoveMachine(pRobotDriver, tBoardInfo.vtMeasureTrack.at(i).dExAxisPosEnd, m_dSafePosRunSpeed * 1.5, COORD_ABS));
			//pRobotDriver->SafeMoveJByJobSaxis(tBoardInfo.vtMeasureTrack.at(i).vtMeasurePulseEnd[0], tMove);
			CHECK_BOOL_RETURN(m_ptUnit->CheckRobotDone(tBoardInfo.vtMeasureTrack.at(i).vtMeasurePulseEnd[0]));
			// 开始示教	
			if (!g_bLocalDebugMark)
			{
				CHECK_BOOL_RETURN(DoTeach(tBoardInfo.vtCornerRelsve_Info.at(i).nBoardNo, tBoardInfo.vtMeasureTrack.at(i).vtMeasurePulseEnd,
					tBoardInfo.vtMeasureTrack.at(i).vnMeasureTypeEnd, tBoardInfo.vtMeasureTrack.at(i).dExAxisPosEnd));
			}
			else
			{
				m_vtTeachResult.clear();
				for (int n = 0; n < tBoardInfo.vvtMeasure_Data_End.at(i).size(); n++)
				{
					T_ROBOT_COORS tCoor = tBoardInfo.vvtMeasure_Data_End.at(i).at(n);
					T_TEACH_RESULT tTeach;
					tTeach.tKeyPtn3D.x = tCoor.dX;
					tTeach.tKeyPtn3D.y = tCoor.dY;
					tTeach.tKeyPtn3D.z = tCoor.dZ;
					m_vtTeachResult.push_back(tTeach);
				}
			}
			//计算交点
			CHECK_BOOL_RETURN(CalcMeasureIntersection(pRobotDriver, tBoardInfo, m_vtTeachResult,
				tBoardInfo.vtMeasureTrack.at(i).dExAxisPosEnd, i, tIntersection, false));
			tBoardInfo.vtBoardPartInfo.at(i).tEndPoint = tIntersection;
			T_ALGORITHM_POINT tPEnd = { tIntersection.x,tIntersection.y,tIntersection.z };
			vtTeachResult.push_back(tPEnd);
		}
		if (E_MEASURE_PROJPOINT == tBoardInfo.vtBoardPartInfo.at(i).eGetStartMethod)
		{
			/* 新流程测量起始端线段 */
			if (!GetNaturalPop().PopOkCancel("开始拟合起始线段")) return false;
			vector<XI_POINT> vtPoint;
			vector<T_ALGORITHM_POINT> vtSimLineMeasResult;
			/***** 开始示教焊缝起点	*****/
			if (!g_bLocalDebugMark)
			{
				CHECK_BOOL_RETURN(DoTeachWithMove(
					pRobotDriver, tBoardInfo.vtMeasureTrack[i].vtMeasurePointStart,
					tBoardInfo.vtMeasureTrack[i].vtMeasurePointStart,
					tBoardInfo.vtMeasureTrack[i].vtMeasurePulseStart,
					tBoardInfo.vtMeasureTrack[i].vnMeasureTypeStart,
					vtSimLineMeasResult)
				);
				vector<T_ALGORITHM_POINT> vtResult;
				vtResult.insert(vtResult.end(), vtSimLineMeasResult.begin(), vtSimLineMeasResult.end());
				bool bFlag = false;
				if (vtSimLineMeasResult.front().dCoorY > vtSimLineMeasResult.back().dCoorY)
				{
					bFlag = true;
				}
				sort(vtResult.begin(), vtResult.end(),
					[](T_ALGORITHM_POINT t1, T_ALGORITHM_POINT t2)
				{
					return t1.dCoorZ > t2.dCoorZ;
				}
				);
				vtResult.pop_back();
				vtResult.erase(vtResult.begin());
				vtSimLineMeasResult.clear();
				vtSimLineMeasResult.insert(vtSimLineMeasResult.end(), vtResult.begin(), vtResult.end());
				if (bFlag)
				{
					sort(vtSimLineMeasResult.begin(), vtSimLineMeasResult.end(),
						[](T_ALGORITHM_POINT t1, T_ALGORITHM_POINT t2)
					{
						return t1.dCoorY < t2.dCoorY;
					}
					);
				}
				else
				{
					sort(vtSimLineMeasResult.begin(), vtSimLineMeasResult.end(),
						[](T_ALGORITHM_POINT t1, T_ALGORITHM_POINT t2)
					{
						return t1.dCoorY > t2.dCoorY;
					}
					);
				}
			}
			else
			{
				for (int n = 0; n < tBoardInfo.vvtMeasure_Data_Start.at(i).size(); n++)
				{
					T_ROBOT_COORS tCoor = tBoardInfo.vvtMeasure_Data_Start.at(i).at(n);
					XI_POINT tmpXiPtn;
					tmpXiPtn.x = tCoor.dX;
					tmpXiPtn.y = tCoor.dY;
					tmpXiPtn.z = tCoor.dZ;
					vtPoint.push_back(tmpXiPtn);
				}
			}
			// 转换示教结果 
			for (int i = 0; i < vtSimLineMeasResult.size(); i++)
			{
				XI_POINT tmpXiPtn;
				tmpXiPtn = *(XI_POINT*)&vtSimLineMeasResult[i];
				vtPoint.push_back(tmpXiPtn);
			}
			// 起点线参计算
			tLineStart = CalcLineParamRansac(vtPoint, 0.8);
			// 背面搜结尾点
			if (!GetNaturalPop().PopOkCancel("开始搜索焊缝背面结尾端点"))  return false;
			m_pScanInit->m_bFixedPointScan = tBoardInfo.vtBoardPartInfo.at(i).bFixedPointScanEnd;
			// 搜索端点 
			if (!g_bLocalDebugMark)
			{
				m_ptUnit->SwitchDHCamera(m_ptUnit->m_nMeasureCameraNo, true, E_CALL_BACK_MODE_WAIT_IMAGE, E_ACQUISITION_MODE_CONTINUE); 
				CHECK_BOOL_RETURN(
					SetScanPos(
						pRobotDriver,
						tBoardInfo.vtMeasureTrack.at(i).vtMeasureProStart,
						tBoardInfo.vtMeasureTrack.at(i).vtMeasurePulseProStart,
						tBoardInfo.vtMeasureTrack.at(i).vnMeasureProStart
					)
				);
				m_pScanInit->m_eEndPointType = GROUP_STAND_DIP_NS::E_PIECE_START;
				CHECK_BOOL_RETURN(
					m_pScanInit->DynamicScanningEndpoint(pRobotDriver)
				);
				m_ptUnit->SwitchDHCamera(m_ptUnit->m_nMeasureCameraNo,false);
				CString cstr;
				XI_POINT tDynamicScanEndPoint = m_pScanInit->GetDynamicScanEndPoint();
				cstr.Format("原始搜索终点结果 x:%.3f\ty:%.3f\tz:%.3f\n", tDynamicScanEndPoint.x, tDynamicScanEndPoint.y, tDynamicScanEndPoint.z);
				TRACE(cstr);

				//将搜索端点投影到搜索线段上
				T_LINE_PARA tLine = CalcLineParamRansac(m_pScanInit->GetVtImgProcResult3D(), 0.8);
				//2.投影
				PointtoLineProjection(tLine, tDynamicScanEndPoint, tDynamicScanEndPoint);
				m_pScanInit->SetDynamicScanEndPoint(tDynamicScanEndPoint);
				//搜搜起点补偿3mm
				if (!tBoardInfo.vtCornerRelsve_Info.at(i).bBoardType)
				{
					/*double dOffset = -9.0;
					m_pScanInit->m_tDynamicScanEndPoint.x = m_pScanInit->m_tDynamicScanEndPoint.x + dOffset * CosD(tBoardInfo.vtCornerRelsve_Info.at(i).dStartAngle - 90 * m_nRobotInstallDir);
					m_pScanInit->m_tDynamicScanEndPoint.y = m_pScanInit->m_tDynamicScanEndPoint.y + dOffset * SinD(tBoardInfo.vtCornerRelsve_Info.at(i).dStartAngle - 90 * m_nRobotInstallDir);*/
				}
				cstr.Format("修正搜索起点结果 x:%.3f\ty:%.3f\tz:%.3f\n", tDynamicScanEndPoint.x, tDynamicScanEndPoint.y, tDynamicScanEndPoint.z);
				
				TRACE(cstr);
				//搜搜结尾补偿3mm
				/*if (!tBoardInfo.vtCornerRelsve_Info.at(i).bBoardType)
				{
					double dOffset = -18;
					m_pScanInit->m_tDynamicScanEndPoint.x = m_pScanInit->m_tDynamicScanEndPoint.x + dOffset * CosD(tBoardInfo.vtCornerRelsve_Info.at(i).dStartAngle + 90 * m_nRobotInstallDir);
					m_pScanInit->m_tDynamicScanEndPoint.y = m_pScanInit->m_tDynamicScanEndPoint.y + dOffset * SinD(tBoardInfo.vtCornerRelsve_Info.at(i).dStartAngle + 90 * m_nRobotInstallDir);
				}*/
				
				T_ALGORITHM_POINT tPEnd = *(T_ALGORITHM_POINT*)&tDynamicScanEndPoint;
				//将补偿后的示教结果加入vtTeachResult
				vtTeachResult.push_back(tPEnd);
				T_ROBOT_COORS tRobotCoor = *tBoardInfo.vtMeasureTrack[i].vtMeasureProStart.rbegin();
				//抬枪 MOVL
				pRobotDriver->MoveByJob(tRobotCoor, tMove, pRobotDriver->m_nExternalAxleType, "MOVL");
				CHECK_BOOL_RETURN(m_ptUnit->CheckRobotDone(tRobotCoor));
			}
			else
			{
				T_ALGORITHM_POINT tTeach;
				tTeach.dCoorX = 0;
				tTeach.dCoorY = 0;
				tTeach.dCoorZ = 0;
				vtTeachResult.push_back(tTeach);
			}
			if (E_WRAPANGLE_ONCE == tBoardInfo.vtBoardPartInfo.at(i).eWarpType && !tBoardInfo.vtCornerRelsve_Info.at(i).bBoardType)
			{
				if (!GetNaturalPop().PopOkCancel("开始拟合终点投影线段"))  return false;
				// 开始示教用于拟合投影线的终点
				vtSimLineMeasResult.clear();
				if (!g_bLocalDebugMark)
				{
					CHECK_BOOL_RETURN(DoTeachWithMove(
						pRobotDriver, tBoardInfo.vtMeasureTrack[i].vtMeasureProEnd,
						tBoardInfo.vtMeasureTrack[i].vtMeasureProEnd,
						tBoardInfo.vtMeasureTrack[i].vtMeasurePulseProEnd,
						tBoardInfo.vtMeasureTrack[i].vnMeasureProEnd,
						vtSimLineMeasResult)
					);
					vector<T_ALGORITHM_POINT> vtResult;
					vtResult.insert(vtResult.end(), vtSimLineMeasResult.begin(), vtSimLineMeasResult.end());
					bool bFlag = false;
					if (vtSimLineMeasResult.front().dCoorY > vtSimLineMeasResult.back().dCoorY)
					{
						bFlag = true;
					}
					sort(vtResult.begin(), vtResult.end(),
						[](T_ALGORITHM_POINT t1, T_ALGORITHM_POINT t2)
					{
						return t1.dCoorZ > t2.dCoorZ;
					}
					);

					vtResult.pop_back();
					vtResult.erase(vtResult.begin());
					vtSimLineMeasResult.clear();
					vtSimLineMeasResult.insert(vtSimLineMeasResult.end(), vtResult.begin(), vtResult.end());
					if (bFlag)
					{
						sort(vtSimLineMeasResult.begin(), vtSimLineMeasResult.end(),
							[](T_ALGORITHM_POINT t1, T_ALGORITHM_POINT t2)
						{
							return t1.dCoorY < t2.dCoorY;
						}
						);
					}
					else
					{
						sort(vtSimLineMeasResult.begin(), vtSimLineMeasResult.end(),
							[](T_ALGORITHM_POINT t1, T_ALGORITHM_POINT t2)
						{
							return t1.dCoorY > t2.dCoorY;
						}
						);
					}
					// 转换示教结果
					vtPoint.clear();
					for (int i = 0; i < vtSimLineMeasResult.size(); i++)
					{
						XI_POINT tmpXiPtn;
						tmpXiPtn = *(XI_POINT*)&vtSimLineMeasResult[i];
						vtPoint.push_back(tmpXiPtn);
					}
					tLineProEnd = CalcLineParamRansac(vtPoint, 0.8);
				}
				else
				{
					for (int n = 0; n < tBoardInfo.vvtMeasure_Data_ProEnd.at(i).size(); n++)
					{
						T_ROBOT_COORS tCoor = tBoardInfo.vvtMeasure_Data_Start.at(i).at(n);
						XI_POINT tmpXiPtn;
						tmpXiPtn.x = tCoor.dX;
						tmpXiPtn.y = tCoor.dY;
						tmpXiPtn.z = tCoor.dZ;
						vtPoint.push_back(tmpXiPtn);
					}
				}
				tBoardInfo.vtCornerRelsve_Info[i].nStartType = 0;
			}
			XI_POINT tXiPtn;
			XI_POINT tXiPtnPro;
			T_ALGORITHM_POINT tAlgPtn;
			vector<T_ALGORITHM_POINT> vt3_2Line;
			//计算投影出来的焊缝起始端点
			tXiPtn = *(XI_POINT*)&vtTeachResult.back();
			PointtoLineProjection(tLineStart, tXiPtn, tXiPtnPro);
			tAlgPtn.dCoorX = tXiPtnPro.x;
			tAlgPtn.dCoorY = tXiPtnPro.y;
			tAlgPtn.dCoorZ = tXiPtnPro.z;
			vtTeachResult.emplace(vtTeachResult.begin() + 1, tAlgPtn);
			//计算投影出来的焊缝对面的起点
			if (E_WRAPANGLE_ONCE == tBoardInfo.vtBoardPartInfo.at(i).eWarpType && !tBoardInfo.vtCornerRelsve_Info.at(i).bBoardType)
			{
				tXiPtn = *(XI_POINT*)&vtTeachResult.front();
				PointtoLineProjection(tLineProEnd, tXiPtn, tXiPtnPro);
				tAlgPtn.dCoorX = tXiPtnPro.x;
				tAlgPtn.dCoorY = tXiPtnPro.y;
				tAlgPtn.dCoorZ = tXiPtnPro.z;
				vtTeachResult.push_back(tAlgPtn);
				//执行到这里的vtTeachResult结构
				//		* *
				//		* *
				//		* *  2 * * * * * * * * 3
				//		* *  *				   *
				//		* *  1 * * * * * * * * 0
				//		* *
				//		* *
				//拟合 3 → 2 线段（有方向性,在计算焊接轨迹时判断RZ要用）
				vt3_2Line.push_back(vtTeachResult[3]);
				vt3_2Line.push_back(vtTeachResult[2]);
				vector<XI_POINT> vtXiPtn;
				//替换一次包角的测量结果
				vtTeachResultWarp = vt3_2Line;
				SaveDataAlgorithm(vtTeachResultWarp, tBoardInfo.vtBoardPartInfo.at(i).strfileName + "RealTeachCoorsWarp.txt");
				vtTeachResult.erase(vtTeachResult.end() - 2, vtTeachResult.end());
			}
			else
			{
				vtTeachResult.erase(vtTeachResult.end() - 1, vtTeachResult.end());
			}
			vector<XI_POINT> vtXiPtn;
			vector<T_ALGORITHM_POINT> vtAlgPtn;
			tBoardInfo.vtBoardPartInfo[i].tStartPoint = *(XI_POINT*)&vtTeachResult[1];
			tBoardInfo.vtBoardPartInfo[i].tEndPoint = *(XI_POINT*)&vtTeachResult[0];
			//执行到这里vtTeachResult只剩1，0两个点
		}
		SaveDataAlgorithm(vtTeachResult, tBoardInfo.vtBoardPartInfo.at(i).strfileName + "RealTeachCoors.txt");
		tBoardInfo.vvtMeasureResult_Info.push_back(vtTeachResult);// 真实测量结果
		tBoardInfo.vvtMeasureResult_Warp.push_back(vtTeachResultWarp);// 包角测量结果
		// 跟新端点
		CorrectEndPoint(pRobotDriver, tBoardInfo, pvtPointInfo, i,
			tBoardInfo.vtBoardPartInfo.at(i).tStartPoint, tBoardInfo.vtBoardPartInfo.at(i).tEndPoint);
	}
	return true;
}

bool CDiaphragmWeld::CalcRealWeldTrack(CRobotDriverAdaptor* pRobotDriver, T_BOARD_INFO_NEW& tBoardInfo)
{
	if (tBoardInfo.vtCornerRelsve_Info.size() < 5)
	{
		XiMessageBoxOk("关联焊缝数据有误,请检查数据");
		return false;
	}
	vector<T_ROBOT_COORS> vtRealPoint;
	T_ROBOT_MOVE_SPEED tMove(m_dFastApproachToWorkSpeed / 2, 20, 20);
	vector<T_ALGORITHM_POINT> vtTeachResult;
	vector<XI_POINT> vtRealWeldCoors, vtRealWeldPosture;
	for (int i = 0; i < tBoardInfo.vtCornerRelsve_Info.size(); i++)
	{
		vtRealPoint.clear();
		vtTeachResult.clear();
		vtRealWeldCoors.clear();
		vtRealWeldPosture.clear();
		if (E_NO_CONTINUOUS == tBoardInfo.eContinueType && 2 != i ||
			(tBoardInfo.nContinueStartNo > i || i > tBoardInfo.nContinueStartNo + tBoardInfo.nContinueNum - 1) ||
			!tBoardInfo.vbBoardDoesItExist.at(i) || LoadWeldStatus(tBoardInfo.vtCornerRelsve_Info.at(i).nBoardNo) ||
			tBoardInfo.vtCornerRelsve_Info.at(i).bIfWeldCom)// 自身焊接或不存在	 
		{
			tBoardInfo.vvRealTrackRobotCoors.push_back(vtRealPoint);
			continue;
		}
		// 拟合实际轨迹 直线 圆弧 
		// 计算转变角度和距离：
		int nStartStepNo = 0, nEndstepNo = 0;
		double dStChangeAngle = 0, dEdChangeAngle = 0;
		double dChangePostureThresVal = fabs(pRobotDriver->m_tTools.tGunTool.dX);
		//起点干涉 且 非半干涉
		if (1 == tBoardInfo.vtCornerRelsve_Info.at(i).nStartType)
		{
			XI_POINT tLeftCenter = { tBoardInfo.vtCenter_Info.at(i - 1).dCenterX ,tBoardInfo.vtCenter_Info.at(i - 1).dCenterY,tBoardInfo.vtCenter_Info.at(i - 1).dCenterZ };
			XI_POINT tRightCenter = { tBoardInfo.vtCenter_Info.at(i).dCenterX ,tBoardInfo.vtCenter_Info.at(i).dCenterY,tBoardInfo.vtCenter_Info.at(i).dCenterZ };
			GetChangePosturePare(tBoardInfo.vvtheoryTrackRobotCoors.at(i - 1), tLeftCenter, tBoardInfo.vtCornerRelsve_Info.at(i - 1).dEndAngle,
				tBoardInfo.vtCornerRelsve_Info.at(i - 1).bBoardType, tBoardInfo.vvtheoryTrackRobotCoors.at(i), tRightCenter,
				tBoardInfo.vtCornerRelsve_Info.at(i).dStartAngle, tBoardInfo.vtCornerRelsve_Info.at(i).bBoardType, true,
				nStartStepNo, dStChangeAngle, dChangePostureThresVal);
		}
		//终点干涉
		if (1 == tBoardInfo.vtCornerRelsve_Info.at(i).nEndType && tBoardInfo.vtCornerRelsve_Info.at(i).nStartType != 4 )
		{
			XI_POINT tLeftCenter = { tBoardInfo.vtCenter_Info.at(i).dCenterX ,tBoardInfo.vtCenter_Info.at(i).dCenterY,tBoardInfo.vtCenter_Info.at(i).dCenterZ };
			XI_POINT tRightCenter = { tBoardInfo.vtCenter_Info.at(i + 1).dCenterX ,tBoardInfo.vtCenter_Info.at(i + 1).dCenterY,tBoardInfo.vtCenter_Info.at(i + 1).dCenterZ };
			GetChangePosturePare(tBoardInfo.vvtheoryTrackRobotCoors.at(i), tLeftCenter, tBoardInfo.vtCornerRelsve_Info.at(i).dEndAngle,
				tBoardInfo.vtCornerRelsve_Info.at(i).bBoardType, tBoardInfo.vvtheoryTrackRobotCoors.at(i + 1), tRightCenter,
				tBoardInfo.vtCornerRelsve_Info.at(i + 1).dStartAngle, tBoardInfo.vtCornerRelsve_Info.at(i + 1).bBoardType, false,
				nEndstepNo, dEdChangeAngle, dChangePostureThresVal);
		}
		// 直线非焊接过程搜结尾点
		if (0 == tBoardInfo.vtCornerRelsve_Info.at(i).bBoardType &&
			!tBoardInfo.vtBoardPartInfo.at(i).bIsTrackingScanEnd)
		{
			double dNormalAngle = 0;
			tBoardInfo.vtBoardPartInfo.at(i).vRealTrackCoors.clear();
			vtRealPoint.clear();

			if (E_CORRECT_ENDPOINT == tBoardInfo.vtBoardPartInfo.at(i).eGetStartMethod || E_CORRECT_ENDPOINT == tBoardInfo.vtBoardPartInfo.at(i).eGetEndMethod)
			{
				vector<XI_POINT> vtMeasureResult;
				for (int n = 0; n < tBoardInfo.vvtMeasureResult_Info.at(i).size(); n++)//测量数据是由终点指向起点
				{
					XI_POINT tp = { tBoardInfo.vvtMeasureResult_Info.at(i).at(n).dCoorX,tBoardInfo.vvtMeasureResult_Info.at(i).at(n).dCoorY,tBoardInfo.vvtMeasureResult_Info.at(i).at(n).dCoorZ };
					vtMeasureResult.push_back(tp);
				}
				if (E_CORRECT_ENDPOINT == tBoardInfo.vtBoardPartInfo.at(i).eGetEndMethod)// 起点需要修正
				{
					reverse(vtMeasureResult.begin(), vtMeasureResult.end());
				}
				T_LINE_PARA tLineParam;
				tLineParam = CalcLineParamRansac(vtMeasureResult, 0.7);
				WriteLog("修正直线参数：%lf %lf %lf %lf %lf %lf", tLineParam.dPointX, tLineParam.dPointY, tLineParam.dPointZ,
					tLineParam.dDirX, tLineParam.dDirY, tLineParam.dDirZ);
				if (E_CORRECT_ENDPOINT == tBoardInfo.vtBoardPartInfo.at(i).eGetEndMethod)// 终点需要修正
				{
					tBoardInfo.vtBoardPartInfo.at(i).tEndPoint.x = tBoardInfo.vtBoardPartInfo.at(i).tStartPoint.x + tBoardInfo.vtCornerRelsve_Info.at(i).dBoardLength * tLineParam.dDirX;
					tBoardInfo.vtBoardPartInfo.at(i).tEndPoint.y = tBoardInfo.vtBoardPartInfo.at(i).tStartPoint.y + tBoardInfo.vtCornerRelsve_Info.at(i).dBoardLength * tLineParam.dDirY;
					tBoardInfo.vtBoardPartInfo.at(i).tEndPoint.z = tBoardInfo.vtBoardPartInfo.at(i).tStartPoint.z + tBoardInfo.vtCornerRelsve_Info.at(i).dBoardLength * tLineParam.dDirZ;
				}
				else if (E_CORRECT_ENDPOINT == tBoardInfo.vtBoardPartInfo.at(i).eGetStartMethod)// 起点需要修正
				{
					tBoardInfo.vtBoardPartInfo.at(i).tStartPoint.x = tBoardInfo.vtBoardPartInfo.at(i).tEndPoint.x + tBoardInfo.vtCornerRelsve_Info.at(i).dBoardLength * tLineParam.dDirX;
					tBoardInfo.vtBoardPartInfo.at(i).tStartPoint.y = tBoardInfo.vtBoardPartInfo.at(i).tEndPoint.y + tBoardInfo.vtCornerRelsve_Info.at(i).dBoardLength * tLineParam.dDirY;
					tBoardInfo.vtBoardPartInfo.at(i).tStartPoint.z = tBoardInfo.vtBoardPartInfo.at(i).tEndPoint.z + tBoardInfo.vtCornerRelsve_Info.at(i).dBoardLength * tLineParam.dDirZ;
				}
			}

			if (!FitCalLinePointsByMiddlePoints(tBoardInfo.vtBoardPartInfo.at(i).tStartPoint, tBoardInfo.vtBoardPartInfo.at(i).tEndPoint,
				tBoardInfo.vvtMeasureResult_Info.at(i), vtRealWeldCoors, 35., 3.0))
			{
				XiMessageBoxOk("直线拟合失败");
				return false;
			}
			if (!tBoardInfo.vtBoardPartInfo.at(i).bTracking)// 不跟踪
			{
				vector<T_ALGORITHM_POINT> vtMeasurePt;
				T_ALGORITHM_POINT tSegmentStartPoint = { vtRealWeldCoors.at(0).x,vtRealWeldCoors.at(0).y,vtRealWeldCoors.at(0).z };
				T_ALGORITHM_POINT tSegmentEndPoint = { vtRealWeldCoors.at(vtRealWeldCoors.size() - 1).x,vtRealWeldCoors.at(vtRealWeldCoors.size() - 1).y,vtRealWeldCoors.at(vtRealWeldCoors.size() - 1).z };
				if (JudjeMeasureIsavailable(tSegmentStartPoint, tSegmentEndPoint, tBoardInfo.vvtMeasureResult_Info.at(i),
					vtMeasurePt, 5., true))
				{
					SaveDataAlgorithm(vtMeasurePt, tBoardInfo.vtBoardPartInfo.at(i).strfileName + "TeachCoorsTest.txt");
					vtRealWeldCoors.clear();
					CHECK_BOOL_RETURN(PointConnection(vtMeasurePt, vtRealWeldCoors, 3.0));
				}
			}
			CHECK_BOOL_RETURN(CalcRobotPostureCoors(vtRealWeldCoors, m_dPlatWeldRx, m_dPlatWeldRy, 3.0, nStartStepNo, dStChangeAngle, nEndstepNo, dEdChangeAngle,
				tBoardInfo.vtCornerRelsve_Info.at(i).nStartType > 0, tBoardInfo.vtCornerRelsve_Info.at(i).nEndType > 0, 0, 0,
				m_pTraceModel->m_nOpenTrackingPos, m_pTraceModel->m_nCloseTrackingPos, vtRealWeldPosture));

		}
		// 圆弧不跟踪
		else if (tBoardInfo.vtCornerRelsve_Info.at(i).bBoardType && !tBoardInfo.vtBoardPartInfo.at(i).bTracking)
		{
			int nSize = tBoardInfo.vvtMeasureResult_Info.at(i).size();
			if (nSize < 3)
			{
				XiMessageBoxOk("圆弧测量数据小于3个");
				return false;
			}
			vector<XI_POINT> vtFitInputPoints, vtOutputPoints;
			for (int n = 0; n < nSize; n++)
			{
				XI_POINT tp = {
					tBoardInfo.vvtMeasureResult_Info.at(i).at(n).dCoorX ,
					tBoardInfo.vvtMeasureResult_Info.at(i).at(n).dCoorY,
					tBoardInfo.vvtMeasureResult_Info.at(i).at(n).dCoorZ };
				vtFitInputPoints.push_back(tp);
			}
			reverse(vtFitInputPoints.begin(), vtFitInputPoints.end());
			SaveDataXiPoint(vtFitInputPoints, tBoardInfo.vtBoardPartInfo.at(i).strfileName + "FitArcTheoryData.txt");
			//拟合实际圆弧
			double center[3];
			center[0] = tBoardInfo.vtCenter_Info.at(i).dCenterX;
			center[1] = tBoardInfo.vtCenter_Info.at(i).dCenterY;
			center[2] = tBoardInfo.vtCenter_Info.at(i).dCenterZ;
			double detX = tBoardInfo.tStartPoint.x - center[0];
			double detY = tBoardInfo.tStartPoint.y - center[1];
			double radius = sqrt(detX * detX + detY * detY);
			if (!CorrectCircled(center, radius, vtFitInputPoints))
			{
				return false;
			}
			E_WELD_LINE vcPointOnTheSameLine;
			for (int i = 0; i < vtFitInputPoints.size() - 1; i++)
			{

				E_WELD_LINE vcPoint;
				CString cStr;
				cStr.Format("result\\%d.txt", i);
				LoadWeldLineData(vcPoint, cStr);
				vcPointOnTheSameLine.vtPoint.insert(vcPointOnTheSameLine.vtPoint.end(), vcPoint.vtPoint.begin(), vcPoint.vtPoint.end());
				vcPointOnTheSameLine.vdNormal.insert(vcPointOnTheSameLine.vdNormal.end(), vcPoint.vdNormal.begin(), vcPoint.vdNormal.end());
			}
			for (int nIdx = 0; nIdx < vcPointOnTheSameLine.vdNormal.size(); nIdx += 3)
			{
				vtRealWeldCoors.push_back(vcPointOnTheSameLine.vtPoint.at(nIdx));
				/*XI_POINT tPosture = { m_dPlatWeldRx ,m_dPlatWeldRy, pRobotDriver->DirAngleToRz(vcPointOnTheSameLine.vdNormal.at(nIdx))};
				vtRealWeldPosture.push_back(tPosture);*/
			}
			CHECK_BOOL_RETURN(CalcRobotPostureCoors(vtRealWeldCoors, m_dPlatWeldRx, m_dPlatWeldRy, 3.0, nStartStepNo, dStChangeAngle, nEndstepNo, dEdChangeAngle,
				tBoardInfo.vtCornerRelsve_Info.at(i).nStartType > 0, tBoardInfo.vtCornerRelsve_Info.at(i).nEndType > 0, 0, 0,
				m_pTraceModel->m_nOpenTrackingPos, m_pTraceModel->m_nCloseTrackingPos, vtRealWeldPosture));
		}
		else// 加载初始段数据跟踪焊接
		{
			LoadDataRobotCoors(vtRealPoint, tBoardInfo.vtBoardPartInfo.at(i).strfileName + "TrackStartCoors.txt");
			tBoardInfo.vvRealTrackRobotCoors.push_back(vtRealPoint);
			tBoardInfo.vtBoardPartInfo.at(i).vRealTrackCoors = vtRealPoint;
			return true;
		}
		SaveDataXiPoint(vtRealWeldCoors, tBoardInfo.vtBoardPartInfo.at(i).strfileName + "RealWeldTrackBefor.txt");
		if (!tBoardInfo.vtBoardPartInfo.at(i).bTracking)
		{
			CalcMeasurePosInBase(vtRealWeldCoors, m_pTraceModel->m_dGunToEyeCompenX, m_pTraceModel->m_dGunToEyeCompenZ, vtRealWeldPosture, vtRealWeldCoors);
		}
		//添加起始结尾安全位置，姿态分配,可以是跟踪也可以是先测后焊
		vtRealPoint = GetRealWeldData(pRobotDriver, vtRealWeldCoors, vtRealWeldPosture);
		SaveDataXiPoint(vtRealWeldCoors, tBoardInfo.vtBoardPartInfo.at(i).strfileName + "RealWeldTrack.txt");
		SaveDataRobotCoors(vtRealPoint, tBoardInfo.vtBoardPartInfo.at(i).strfileName + "RealWeldRobotTrack.txt");
		int nIndex = 0, nsize = vtRealPoint.size();

		double dWarpLength = m_tWrapAngelParaNew.m_dWrapdParallel5;
		if (fabs(dWarpLength) > 60 || fabs(dWarpLength) < 30)
		{
			dWarpLength = 30.;
			m_tWrapAngelParaNew.m_dWrapdParallel5 = 30.;
		}

		for (nIndex = 0; nIndex < vtRealPoint.size(); nIndex++)
		{
			if (tBoardInfo.vvtMeasureResult_Warp.at(i).size() > 1)
			{
				double dis = TwoPointDis(vtRealPoint.at(nIndex).dX, vtRealPoint.at(nIndex).dY + vtRealPoint.at(nIndex).dBY, vtRealPoint.at(nIndex).dZ,
					vtRealPoint.at(0).dX, vtRealPoint.at(0).dY + vtRealPoint.at(0).dBY, vtRealPoint.at(0).dZ);
				double disEnd = TwoPointDis(vtRealPoint.at(nIndex).dX, vtRealPoint.at(nIndex).dY + vtRealPoint.at(nIndex).dBY, vtRealPoint.at(nIndex).dZ,
					vtRealPoint.at(nsize - 1).dX, vtRealPoint.at(nsize - 1).dY + vtRealPoint.at(nsize - 1).dBY, vtRealPoint.at(nsize - 1).dZ);
				 
				T_ROBOT_COORS tMeasureTrans;
				if (dis < fabs(dWarpLength))
				{
					CalcMeasureInBase(vtRealPoint.at(nIndex), m_tWrapAngelParaNew.m_dWrapdVertical, 0.0, tMeasureTrans);
					vtRealPoint.at(nIndex) = tMeasureTrans;
					tBoardInfo.vtBoardPartInfo.at(i).vPointType.push_back(E_WELD_WRAP);
					continue;
				}
				else if (disEnd < fabs(dWarpLength))
				{
					CalcMeasureInBase(vtRealPoint.at(nIndex), m_tWrapAngelParaNew.m_dWrapdVertical2, 0.0, tMeasureTrans);
					vtRealPoint.at(nIndex) = tMeasureTrans;
					tBoardInfo.vtBoardPartInfo.at(i).vPointType.push_back(E_WELD_WRAP);
					continue;
				}
			}
			tBoardInfo.vtBoardPartInfo.at(i).vPointType.push_back(E_WELD_TRACK);

		}
		//执行到这里是
		//
		//			× × × × × × × × × × × × × × × ×
		//			×											 ×
		//			×											 ×
		//			×											 ×
		//			包 包 包 焊 焊 焊 焊 焊 焊 焊 焊 焊 焊 包 包 包
		//
		//

		// 读删点数量
		COPini iniDelNum;
		int nStartDelNum = 0, nEndDelNum = 0;
		iniDelNum.SetFileName("Data\\RobotandCar.ini");
		iniDelNum.SetSectionName("RecoParam");
		iniDelNum.ReadString("StartRemainLength", &nStartDelNum);
		iniDelNum.ReadString("EndRemainLength", &nEndDelNum);
		nStartDelNum /= 3;		//拟合的轨迹间隔为3mm所以除以3
		nEndDelNum /= 3;
		//包角理论轨迹
		m_pTraceModel->m_vvtTrackWarpCoors.clear();
		m_pTraceModel->m_vvnTrackWarpPtType.clear();
		m_pTraceModel->m_vvtWarpCorrectCoors.clear();
		// 二次包角
		if (E_WRAPANGLE_TWO_SINGLE == tBoardInfo.vtBoardPartInfo.at(i).eWarpType)//先测后焊终点包角
		{
			vector<T_ROBOT_COORS> tWarpEnd;
			vector<int> vnPtType;
			int nSize = vtRealPoint.size();
			T_ROBOT_COORS tStartCoor = vtRealPoint.at(0);
			T_ROBOT_COORS tCoor = vtRealPoint.at(nSize - 1), tEndCoor;
			double dNormalAngle = pRobotDriver->RzToDirAngle(tCoor.dRZ);
			double dDirAngle = pRobotDriver->RzToDirAngle(tCoor.dRZ) + 90 * m_nRobotInstallDir;
			//调整起始点
			 tStartCoor.dX -= m_tWrapAngelParaNew.m_dWrapdParallel * CosD(dDirAngle);
			 tStartCoor.dBY -= m_tWrapAngelParaNew.m_dWrapdParallel * SinD(dDirAngle);
			// 调整结尾点
			tCoor.dY += tCoor.dBY;
			 tCoor.dX += m_tWrapAngelParaNew.m_dWrapdParallel2 * CosD(dDirAngle);
			 tCoor.dY += m_tWrapAngelParaNew.m_dWrapdParallel2 * SinD(dDirAngle);
			 tCoor.dRZ += fabs(m_tWrapAngelParaNew.m_dWrapdRZ2) * m_nRobotInstallDir;

			tEndCoor = tCoor;
			double dDis = -fabs(m_tWrapAngelParaNew.m_dWrapdVertical2)/*m_dVerticalAdjust2*/;
			if (fabs(m_tWrapAngelParaNew.m_dWrapdVertical2) > 1)
			{
				tEndCoor.dX += dDis * CosD(dNormalAngle);
				tEndCoor.dY += dDis * SinD(dNormalAngle);

				int nStep = dDis / 2;
				if (nStep < 4)
				{
					nStep = 4;
				}
				double dStepDis = dDis / (nStep - 1);
				double dStepAngle = fabs(m_tWrapAngelParaNew.m_dWrapdRZ2)/*45.0*/ / (nStep - 1);
				double dDirX = (tEndCoor.dX - tCoor.dX) / dDis;
				double dDirY = (tEndCoor.dY - tCoor.dY) / dDis;
				double dDirZ = (tEndCoor.dZ - tCoor.dZ) / dDis;
				for (int n = 0; n < nStep; n++)
				{
					T_ROBOT_COORS Stratpt = tCoor;
					Stratpt.dX += dDirX * dStepDis * n;
					Stratpt.dY += dDirY * dStepDis * n;
					Stratpt.dZ += dDirZ * dStepDis * n;
					Stratpt.dRZ += dStepAngle * n * m_nRobotInstallDir;

					Stratpt.dBY = Stratpt.dY + 250.;
					Stratpt.dY = -250.;
					tWarpEnd.push_back(Stratpt);
					vnPtType.push_back(E_WELD_WRAP);
				}
			}

			// 添加起始点
			if (fabs(m_tWrapAngelParaNew.m_dWrapdParallel) > 0.0)
			{
				vtRealPoint.insert(vtRealPoint.begin(), tStartCoor);
				tBoardInfo.vtBoardPartInfo.at(i).vPointType.insert(tBoardInfo.vtBoardPartInfo.at(i).vPointType.begin(),
					E_WELD_TRACK);
			}
			//更改包角前几步为包角数据
			for (int nW = 0; nW < 5; nW++)
			{
				tBoardInfo.vtBoardPartInfo.at(i).vPointType.at(
					tBoardInfo.vtBoardPartInfo.at(i).vPointType.size() - nW - 1) = E_WELD_WRAP;
			}

			// 添加结尾点
			vtRealPoint.insert(vtRealPoint.end(), tWarpEnd.begin(), tWarpEnd.end());
			tBoardInfo.vtBoardPartInfo.at(i).vPointType.insert(tBoardInfo.vtBoardPartInfo.at(i).vPointType.end(),
				vnPtType.begin(), vnPtType.end());

			tBoardInfo.vvtTrackWarpCoors.push_back(tWarpEnd);
			tBoardInfo.vvnTrackWarpPtType.push_back(vnPtType);
			m_pTraceModel->m_vvtTrackWarpCoors = tBoardInfo.vvtTrackWarpCoors;
			m_pTraceModel->m_vvnTrackWarpPtType = tBoardInfo.vvnTrackWarpPtType;

			 m_pTraceModel->m_vvtWarpCorrectCoors = tBoardInfo.vvtTrackWarpCoors;
		}
		// 一次包角
		if (tBoardInfo.vvtMeasureResult_Warp.at(i).size() > 1)
		{
			vector<int> vPtType;
			vector<XI_POINT> vtTrackCoors, vtTrackPosture;
			vector<T_ROBOT_COORS> vtRobotCoors;
			/*if (!FitCalLinePointsByMiddlePoints(tBoardInfo.vtBoardPartInfo.at(i).tEndPoint, tBoardInfo.vtBoardPartInfo.at(i).tStartPoint,
				tBoardInfo.vvtMeasureResult_Warp.at(i), vtTrackCoors, 35., 3.0))
			{
				XiMessageBoxOk("直线拟合失败");
				return false;
			}
			SaveDataXiPoint(vtTrackCoors, tBoardInfo.vtBoardPartInfo.at(i).strfileName + "WarpWeldTyeoryTrack.txt");
			*/
			T_ALGORITHM_POINT tSegmentStartPoint;
			T_ALGORITHM_POINT tSegmentEndPoint;
			//获取起终点投影  非投影测量模式
			if (tBoardInfo.vvtMeasureResult_Warp.at(i).size() >= 4 && tBoardInfo.vtBoardPartInfo[i].eGetStartMethod != E_MEASURE_PROJPOINT)
			{

				//vector<T_ALGORITHM_POINT > vtMeasureResultStrat;
				//vector<T_ALGORITHM_POINT > vtMeasureResultEnd;
				////包角段起点线段拟合
				//vtMeasureResultStrat.insert(vtMeasureResultStrat.end(),
				//	tBoardInfo.vvtMeasureResult_Warp[i].rbegin(),
				//	tBoardInfo.vvtMeasureResult_Warp[i].rbegin() + tBoardInfo.vvtMeasureResult_Warp[i].size() / 2
				//);
				////包角段终点线段拟合
				//vtMeasureResultEnd.insert(vtMeasureResultEnd.end(),
				//	tBoardInfo.vvtMeasureResult_Warp[i].begin(),
				//	tBoardInfo.vvtMeasureResult_Warp[i].begin() + tBoardInfo.vvtMeasureResult_Warp[i].size() / 2
				//);
				////容器翻转顺序
				//reverse(vtMeasureResultEnd.begin(), vtMeasureResultEnd.end());
				//
				//T_LINE_PARA tLineParam = CalcLineParamRansac(vtMeasureResultStrat, 0.8);
				//T_LINE_PARA tLineParam2 = CalcLineParamRansac(vtMeasureResultEnd, 0.8);
				T_LINE_PARA tLineParam = CalcLineParamRansac(tBoardInfo.vvtMeasureResult_Warp[i], 0.8);
				vector<XI_POINT> vtPoint;
				XI_POINT tStartPoint;
				XI_POINT tEndPoint;
				PointtoLineProjection(tLineParam, tBoardInfo.vtBoardPartInfo.at(i).tStartPoint, tEndPoint);
				////求投影点XI_POINT tStartPoint;//起点
				//PointtoLineProjection(tLineParam2, tBoardInfo.vtBoardPartInfo.at(i).tEndPoint, tStartPoint);
				//PointtoLineProjection(tLineParam, tBoardInfo.vtBoardPartInfo.at(i).tStartPoint, tEndPoint);
				
				/* //终点投影偏移量 代替 起点投影
				tEndPoint.x = tBoardInfo.vtBoardPartInfo.at(i).tStartPoint.x + tStartPoint.x - tBoardInfo.vtBoardPartInfo.at(i).tEndPoint.x;
				tEndPoint.y = tBoardInfo.vtBoardPartInfo.at(i).tStartPoint.y + tStartPoint.y - tBoardInfo.vtBoardPartInfo.at(i).tEndPoint.y;
				tEndPoint.z = tBoardInfo.vtBoardPartInfo.at(i).tStartPoint.z + tStartPoint.z - tBoardInfo.vtBoardPartInfo.at(i).tEndPoint.z;*/

				//添加板厚及方向向量
				tSegmentStartPoint.dCoorX = tBoardInfo.vtBoardPartInfo.at(i).tEndPoint.x + tEndPoint.x - tBoardInfo.vtBoardPartInfo.at(i).tStartPoint.x;
				tSegmentStartPoint.dCoorY = tBoardInfo.vtBoardPartInfo.at(i).tEndPoint.y + tEndPoint.y - tBoardInfo.vtBoardPartInfo.at(i).tStartPoint.y;
				tSegmentStartPoint.dCoorZ = tBoardInfo.vtBoardPartInfo.at(i).tEndPoint.z + tEndPoint.z - tBoardInfo.vtBoardPartInfo.at(i).tStartPoint.z;
				tStartPoint = *(XI_POINT*)&tSegmentStartPoint;
				tSegmentEndPoint.dCoorX = tEndPoint.x;
				tSegmentEndPoint.dCoorY = tEndPoint.y;
				tSegmentEndPoint.dCoorZ = tEndPoint.z;
				vtPoint.push_back(tStartPoint);
				vtPoint.push_back(tEndPoint);
				tBoardInfo.vvtMeasureResult_Warp[i].clear();
				tBoardInfo.vvtMeasureResult_Warp[i].push_back(tSegmentEndPoint);
				tBoardInfo.vvtMeasureResult_Warp[i].push_back(tSegmentStartPoint);
				SaveDataXiPoint(vtPoint, tBoardInfo.vtBoardPartInfo.at(i).strfileName + "WarpWeldTyeoryEndpoint.txt");
			}
			//投影测量模式
			else if (tBoardInfo.vvtMeasureResult_Warp.at(i).size() == 2 && tBoardInfo.vtBoardPartInfo[i].eGetStartMethod == E_MEASURE_PROJPOINT)
			{
				vector<XI_POINT> vtPoint;
				XI_POINT tXiPoint;
				T_ALGORITHM_POINT tAlgPtn;
				tAlgPtn = *tBoardInfo.vvtMeasureResult_Warp[i].begin();
				tXiPoint = *(XI_POINT*)&tAlgPtn;
				tSegmentStartPoint = tAlgPtn;
				vtPoint.push_back(tXiPoint);
				tAlgPtn = *tBoardInfo.vvtMeasureResult_Warp[i].rbegin();
				tXiPoint = *(XI_POINT*)&tAlgPtn;
				vtPoint.push_back(tXiPoint);
				tSegmentEndPoint = tAlgPtn;
				SaveDataXiPoint(vtPoint, tBoardInfo.vtBoardPartInfo.at(i).strfileName + "WarpWeldTyeoryEndpoint.txt");
			}
			vector<T_ALGORITHM_POINT> vtMeasurePt;


			if (JudjeMeasureIsavailable(tSegmentStartPoint, tSegmentEndPoint, tBoardInfo.vvtMeasureResult_Warp.at(i),
				vtMeasurePt, 5.))
			{
				vtTrackCoors.clear();
				CHECK_BOOL_RETURN(PointConnection(vtMeasurePt, vtTrackCoors, 3.0));
			}
			SaveDataXiPoint(vtTrackCoors, tBoardInfo.vtBoardPartInfo.at(i).strfileName + "WarpWeldTyeoryTrack2.txt");
			//计算机器人姿态 RX,RY,RZ
			CHECK_BOOL_RETURN(CalcRobotPostureCoors(vtTrackCoors, m_dPlatWeldRx, m_dPlatWeldRy, 3.0, nStartStepNo, dStChangeAngle, nEndstepNo, dEdChangeAngle,
				tBoardInfo.vtCornerRelsve_Info.at(i).nStartType > 0, tBoardInfo.vtCornerRelsve_Info.at(i).nEndType > 0, 0, 0,
				m_pTraceModel->m_nOpenTrackingPos, m_pTraceModel->m_nCloseTrackingPos, vtTrackPosture));

			if (!tBoardInfo.vtBoardPartInfo.at(i).bTracking)
			{
				// 补偿数据
				CalcMeasurePosInBase(vtTrackCoors, m_pTraceModel->m_dGunToEyeCompenX, m_pTraceModel->m_dGunToEyeCompenZ, vtTrackPosture, vtTrackCoors);
			}
			//将投影轨迹转化为机器人坐标T_ROBOT_COORS
			vtRobotCoors = GetRealWeldData(pRobotDriver, vtTrackCoors, vtTrackPosture);
			if (vtRobotCoors.size() < 3)
			{
				return false;
			}
			//保存轨迹
			SaveDataRobotCoors(vtRobotCoors, tBoardInfo.vtBoardPartInfo.at(i).strfileName + "WarpWeldTrack.txt");
			int nIndex = 0, nsize = vtRobotCoors.size();
			//添加轨迹点类型
			for (nIndex = 0; nIndex < vtRobotCoors.size(); nIndex++)
			{
				double dis = TwoPointDis(vtRobotCoors.at(nIndex).dX, vtRobotCoors.at(nIndex).dY + vtRobotCoors.at(nIndex).dBY, vtRobotCoors.at(nIndex).dZ,
					vtRobotCoors.at(0).dX, vtRobotCoors.at(0).dY + vtRobotCoors.at(0).dBY, vtRobotCoors.at(0).dZ);
				double disEnd = TwoPointDis(vtRobotCoors.at(nIndex).dX, vtRobotCoors.at(nIndex).dY + vtRobotCoors.at(nIndex).dBY, vtRobotCoors.at(nIndex).dZ,
					vtRobotCoors.at(nsize - 1).dX, vtRobotCoors.at(nsize - 1).dY + vtRobotCoors.at(nsize - 1).dBY, vtRobotCoors.at(nsize - 1).dZ);
				T_ROBOT_COORS tMeasureTrans;
				if (dis < fabs(m_tWrapAngelParaNew.m_dWrapdParallel5))
				{
					CalcMeasureInBase(vtRobotCoors.at(nIndex), m_tWrapAngelParaNew.m_dWrapdVertical3, 0.0, tMeasureTrans);
					vtRobotCoors.at(nIndex) = tMeasureTrans;
					vPtType.push_back(E_WELD_WRAP);
					continue;
				}
				else if (disEnd < fabs(m_tWrapAngelParaNew.m_dWrapdParallel5))
				{
					CalcMeasureInBase(vtRobotCoors.at(nIndex), m_tWrapAngelParaNew.m_dWrapdVertical4, 0.0, tMeasureTrans);
					vtRobotCoors.at(nIndex) = tMeasureTrans;
					vPtType.push_back(E_WELD_WRAP);
					continue;
				}
				vPtType.push_back(E_WELD_TRACK);
			}

			//执行到这里是
			//
			//			包 包 包 焊 焊 焊 焊 焊 焊 焊 焊 焊 焊 包 包 包
			//			×											 ×
			//			×											 ×
			//			×											 ×
			//			包 包 包 焊 焊 焊 焊 焊 焊 焊 焊 焊 焊 包 包 包
			//
			//

			//添加包角坐标  vtRobotCoors方向 2 → 3 vtRealPoint方向 0 → 1
			 /*3********************2(vtRobotCoors)
			   4					5
			   0********************1(vtRealPoint)
			 */
			 // 0,1,2,3:vtRealPoint的第一个和最后一个，vtRobotCoors.第一个和最后一个,vtEndpointAfter:4起始中间点，5结尾中间点
			vector<T_ROBOT_COORS> vtEndpoint, vtWarpStrat, vtWarpEnd,vtTmpCoord;
			vector<int> vtTmpType;
			vtEndpoint.push_back(vtRealPoint.front());
			vtEndpoint.push_back(vtRealPoint.back());
			vtEndpoint.push_back(vtRobotCoors.front());
			vtEndpoint.push_back(vtRobotCoors.back());
			CHECK_BOOL_RETURN(CalcWarpTrack(vtEndpoint, vtWarpStrat, vtWarpEnd));
			int Change = m_tWrapAngelParaNew.m_dWrapdParallel5 / 3 + 1;
			vPtType.at(10) = E_WELD_OPEN_TRACKING;//开启第二次跟踪标志
			vPtType.at(vPtType.size()- Change) = E_WELD_WRAP_CHANGE;
			int ndx = 0;
			//vtWarpEnd为1 → 5 → 2
			reverse(vtWarpEnd.begin(), vtWarpEnd.end());
			//dInsertPercentIn跳枪正面插入板厚的多少百分比,dInsertPercentOut跳枪反面插入板厚的多少百分比
			double dInsertPercentIn, dInsertPercentOut;
			dInsertPercentIn = 6.0;
			dInsertPercentOut = 6.0;
			switch (*m_pUnWarpBoundWeld)
			{
			case 1:	//起点跳枪
				vtTmpCoord.clear();
				vtTmpType.clear();
				GenerateWrapBoundTrack(vtRobotCoors.back(), vtRealPoint.front(), vtTmpCoord, vtTmpType, dInsertPercentIn, dInsertPercentOut, 6.0, 25.0);
				vtRobotCoors.insert(vtRobotCoors.end(), vtTmpCoord.begin(),vtTmpCoord.end());
				vPtType.insert(vPtType.end(), vtTmpType.begin(),vtTmpType.end());
				for (ndx = 0; ndx < vtWarpEnd.size(); ndx++)
				{
					//终点包角轨迹生成
					vtRobotCoors.insert(vtRobotCoors.begin(), vtWarpEnd.at(ndx));
					vPtType.insert(vPtType.begin(), E_WELD_WRAP);
				}
				break;
			case 2:	//终点跳枪
				vtTmpCoord.clear();
				vtTmpType.clear();
				GenerateWrapBoundTrack(vtRealPoint.back(), vtRobotCoors.front(), vtTmpCoord, vtTmpType, dInsertPercentIn, dInsertPercentOut, 6.0, 25.0);
				vtRobotCoors.insert(vtRobotCoors.begin(), vtTmpCoord.begin(), vtTmpCoord.end());
				vPtType.insert(vPtType.begin(), vtTmpType.begin(),vtTmpType.end());
				for (ndx = 0; ndx < vtWarpStrat.size(); ndx++)
				{
					//起点包角轨迹生成
					vtRobotCoors.push_back(vtWarpStrat.at(ndx));
					vPtType.push_back(E_WELD_WRAP);
				}
				break;
			case 3:	//起终点都跳
				//起点跳
				vtTmpCoord.clear();
				vtTmpType.clear();
				GenerateWrapBoundTrack(vtRobotCoors.back(), vtRealPoint.front(), vtTmpCoord, vtTmpType, dInsertPercentIn, dInsertPercentOut, 6.0, 25.0);
				vtRobotCoors.insert(vtRobotCoors.end(), vtTmpCoord.begin(),vtTmpCoord.end());
				vPtType.insert(vPtType.end(), vtTmpType.begin(),vtTmpType.end());
				//终点跳
				vtTmpCoord.clear();
				vtTmpType.clear();
				GenerateWrapBoundTrack(vtRealPoint.back(), vtRobotCoors.front(), vtTmpCoord, vtTmpType, dInsertPercentIn, dInsertPercentOut, 6.0, 25.0);
				vtRobotCoors.insert(vtRobotCoors.begin(), vtTmpCoord.begin(),vtTmpCoord.end());
				vPtType.insert(vPtType.begin(), vtTmpType.begin(),vtTmpType.end());
				break;
			default:
				//不跳枪进这里
				for (ndx = 0; ndx < vtWarpEnd.size(); ndx++)
				{
					//终点包角轨迹生成
					vtRobotCoors.insert(vtRobotCoors.begin(), vtWarpEnd.at(ndx));
					vPtType.insert(vPtType.begin(), E_WELD_WRAP);
				}
				for (ndx = 0; ndx < vtWarpStrat.size(); ndx++)
				{
					//起点包角轨迹生成
					vtRobotCoors.push_back(vtWarpStrat.at(ndx));
					vPtType.push_back(E_WELD_WRAP);
				}
				break;
			}
			
			

			//合并包角数据
			tBoardInfo.vtBoardPartInfo.at(i).vPointType.at(10) = E_WELD_OPEN_TRACKING;// 第1次开启跟踪
			tBoardInfo.vtBoardPartInfo.at(i).vPointType.at(tBoardInfo.vtBoardPartInfo.at(i).vPointType.size() - Change) = E_WELD_WRAP_CHANGE;
			SaveDataRobotCoors(vtRobotCoors, tBoardInfo.vtBoardPartInfo.at(i).strfileName + "RealWeldRobotTrackWarpAfter.txt");
			vtRealPoint.insert(vtRealPoint.end(), vtRobotCoors.begin(), vtRobotCoors.end());
			tBoardInfo.vtBoardPartInfo.at(i).vPointType.insert(tBoardInfo.vtBoardPartInfo.at(i).vPointType.end(),
				vPtType.begin(), vPtType.end());

			//  非跳枪		vtRealPoint执行到这里是														跳枪(针对自由端)	vtRealPoint执行到这里是
			//
			//
			//																								跳（下→上）→→	跳（上）
			//			包 包 包	 焊	 焊		焊	焊	焊 焊 焊 焊 焊 焊 包 包 包						  ↑包 包 包 焊 焊	焊	焊	焊	焊	焊	焊	焊	焊	包	包	包	
			//			包													    包						  ↑包（进2/3BoardThick）	↓							    包
			//			包												   	    包							||					跳（上）						    包
			//			包(轨迹End)											    包							包（进1/2BoardThick  轨迹End）						    包
			//			包(轨迹Start) 包 包 焊 焊 焊 焊 焊 焊 焊 焊 焊 焊 包 包 包						  ↑包(轨迹Start) 包 包 焊  ↓焊 焊 焊 焊 焊 焊 焊 焊 包 包 包
			//																							  ↑跳（上→下）	←←跳（上）
			//


			//现在vtRealPoint为 0 → 1 → 5 → 2 → 3 → 4 
			if (tBoardInfo.vtBoardPartInfo.at(i).vPointType.size() != vtRealPoint.size())
			{
				XiMessageBox("点类型和点数数量不一致");
			}
			// 将包角数据放在最后
			vector<T_ROBOT_COORS> vtRobotWarpCoorsTemp;
			vector<int> vnPtTypeTemp;
			E_MEASURE_POINT_TYPE tAddDataType = E_WELD_WRAP;
#if 0
			tAddDataType = E_WELD_WRAP;
			for (nIndex = 0; nIndex < tBoardInfo.vtBoardPartInfo.at(i).vPointType.size(); nIndex++)
			{
				if (E_WELD_TRACK == tBoardInfo.vtBoardPartInfo.at(i).vPointType.at(nIndex))
				{
					break;
				}
				else {
					vtRobotWarpCoorsTemp.push_back(vtRealPoint.at(nIndex));
					vnPtTypeTemp.push_back(tBoardInfo.vtBoardPartInfo.at(i).vPointType.at(nIndex));
				}
			}
#else
			// 将包角数据放在开始
			tAddDataType = E_WELD_TRACK;
			bool bFindWarp = false, bFindTrack = false/*是否找到过焊接轨迹*/;
			int nInster = 0;
			int nFirstTrackStepNum = m_dHandEyeDis * 2 / 3;
			// 拷贝    初始包角直线段+焊接轨迹段 到vtRobotWarpCoorsTemp,vnPtTypeTemp
			for (nIndex = 0; nIndex < tBoardInfo.vtBoardPartInfo.at(i).vPointType.size(); nIndex++)
			{
				if (E_WELD_TRACK == tBoardInfo.vtBoardPartInfo.at(i).vPointType.at(nIndex))
				{
					bFindTrack = true;
					vtRobotWarpCoorsTemp.push_back(vtRealPoint.at(nIndex));
					vnPtTypeTemp.push_back(tBoardInfo.vtBoardPartInfo.at(i).vPointType.at(nIndex));
				}
				else if (E_WELD_TRACK != tBoardInfo.vtBoardPartInfo.at(i).vPointType.at(nIndex))
				{
					if (bFindWarp && bFindTrack)
					{
						break;
					}
					bFindWarp = true;
					vtRobotWarpCoorsTemp.push_back(vtRealPoint.at(nIndex));
					vnPtTypeTemp.push_back(tBoardInfo.vtBoardPartInfo.at(i).vPointType.at(nIndex));
				}
			}
			if (nFirstTrackStepNum < vtRobotWarpCoorsTemp.size())
			{
				vtRobotWarpCoorsTemp.erase(vtRobotWarpCoorsTemp.end() - nFirstTrackStepNum, vtRobotWarpCoorsTemp.end());
				vnPtTypeTemp.erase(vnPtTypeTemp.end() - nFirstTrackStepNum, vnPtTypeTemp.end());
			}
			else
			{
				XiMessageBox("删除数据超出完整轨迹");
				return false;
			}
#endif // 0
			// 对整合的完整焊接轨迹处理
			if (tBoardInfo.vtBoardPartInfo[i].eGetStartMethod == E_MEASURE_PROJPOINT && 0==*m_pUnWarpBoundWeld)
			{
				// 半干涉 非跳枪 直接删除首段包角轨迹 末段包角轨迹
				int nNum = 0;
				for (auto itera = tBoardInfo.vtBoardPartInfo[i].vPointType.rbegin(); itera != tBoardInfo.vtBoardPartInfo[i].vPointType.rend(); itera++)
				{
					if (*itera == E_WELD_WRAP)
					{
						nNum++;
					}
					else
					{
						break;
					}
				}
				vtRealPoint.erase(vtRealPoint.end() - nNum, vtRealPoint.end());
				tBoardInfo.vtBoardPartInfo[i].vPointType.erase(tBoardInfo.vtBoardPartInfo[i].vPointType.end() - nNum,
					tBoardInfo.vtBoardPartInfo[i].vPointType.end());
				nNum = 0;
				for (auto itera = tBoardInfo.vtBoardPartInfo[i].vPointType.begin(); itera != tBoardInfo.vtBoardPartInfo[i].vPointType.end(); itera++)
				{
					if (*itera == E_WELD_WRAP)
					{
						nNum++;
					}
					else
					{
						break;
					}
				}
				vtRealPoint.erase(vtRealPoint.begin(), vtRealPoint.begin() + nNum + 1);
				tBoardInfo.vtBoardPartInfo[i].vPointType.erase(tBoardInfo.vtBoardPartInfo[i].vPointType.begin(),
					tBoardInfo.vtBoardPartInfo[i].vPointType.begin() + nNum + 1);
			}
			else
			{ // 将截断部分首尾替换
				vtRealPoint.erase(vtRealPoint.begin(), vtRealPoint.begin() + vtRobotWarpCoorsTemp.size());
				vtRealPoint.insert(vtRealPoint.end(), vtRobotWarpCoorsTemp.begin(), vtRobotWarpCoorsTemp.end());
				tBoardInfo.vtBoardPartInfo.at(i).vPointType.erase(tBoardInfo.vtBoardPartInfo.at(i).vPointType.begin(),
					tBoardInfo.vtBoardPartInfo.at(i).vPointType.begin() + vnPtTypeTemp.size());
				tBoardInfo.vtBoardPartInfo.at(i).vPointType.insert(tBoardInfo.vtBoardPartInfo.at(i).vPointType.end(),
					vnPtTypeTemp.begin(), vnPtTypeTemp.end());
			}


			//添加起收弧覆盖距离
			//vector<T_ROBOT_COORS> vtRobotCoverCoors;
			for (int index = 0; index < 6; index++)
			{
				vtRealPoint.push_back(vtRealPoint.at(index));
				tBoardInfo.vtBoardPartInfo.at(i).vPointType.push_back(tAddDataType);
			}
			//-----------------------------------------------------------------
			//跟踪时截取包角部分

			//半干涉 删除起始段
			if (tBoardInfo.vtBoardPartInfo[i].eGetStartMethod == E_MEASURE_PROJPOINT)
			{
				vtRealPoint.erase(vtRealPoint.begin(), vtRealPoint.begin() + nStartDelNum);
				vtRealPoint.erase(vtRealPoint.end() - nEndDelNum, vtRealPoint.end());
				auto vtPtnType = &tBoardInfo.vtBoardPartInfo.at(i).vPointType;
				vtPtnType->erase(vtPtnType->begin(), vtPtnType->begin() + nStartDelNum);
				vtPtnType->erase(vtPtnType->end() - nEndDelNum, vtPtnType->end());
				// 当前结构为：
				//
				//		× × 焊 焊 焊 焊 焊 焊 焊 焊 包 包 包
				//	半干涉端								包
				//	半干涉端								包
				//		× × 焊 焊 焊 焊 焊 焊 焊 焊 包 包 包
				//
			}


			double dStepDis = 3.0;
			 int nStepNum = 90.0 / dStepDis;
			bool bGetCoors = false;
			tBoardInfo.vvtTrackWarpCoors.clear();
			tBoardInfo.vvnTrackWarpPtType.clear();
			m_pTraceModel->m_vvtTrackWarpCoors.clear();
			m_pTraceModel->m_vvnTrackWarpPtType.clear();

			vector<T_ROBOT_COORS> vtCoors;
			vector<int> vnPtType;
			bool bIsFirst = true;
			if (tBoardInfo.vtBoardPartInfo.at(i).bTracking)
			{
				for (nIndex = 0; nIndex < vtRealPoint.size(); nIndex++)
				{
#if 0
					if (E_WELD_TRACK != tBoardInfo.vtBoardPartInfo.at(i).vPointType.at(nIndex))
					{
						vtCoors.push_back(vtRealPoint.at(nIndex));
						vnPtType.push_back(tBoardInfo.vtBoardPartInfo.at(i).vPointType.at(nIndex));
						bGetCoors = true;
					}
					else if (E_WELD_TRACK == tBoardInfo.vtBoardPartInfo.at(i).vPointType.at(nIndex)
						&& bGetCoors && nStepNum > 0)
					{
						nStepNum--;
						vtCoors.push_back(vtRealPoint.at(nIndex));
						vnPtType.push_back(tBoardInfo.vtBoardPartInfo.at(i).vPointType.at(nIndex));
						continue;
					}
					if (E_WELD_TRACK == tBoardInfo.vtBoardPartInfo.at(i).vPointType.at(nIndex) && bGetCoors
						|| bGetCoors && nIndex == vtRealPoint.size() - 1) {
						tBoardInfo.vvtTrackWarpCoors.push_back(vtCoors);
						tBoardInfo.vvnTrackWarpPtType.push_back(vnPtType);
						bGetCoors = false;
						vtCoors.clear();
						vnPtType.clear();
					}
#else
					 if (E_WELD_TRACK != tBoardInfo.vtBoardPartInfo.at(i).vPointType.at(nIndex))
					{
						vtCoors.push_back(vtRealPoint.at(nIndex));
						vnPtType.push_back(tBoardInfo.vtBoardPartInfo.at(i).vPointType.at(nIndex));
						bGetCoors = true;
					}
					else if (E_WELD_TRACK == tBoardInfo.vtBoardPartInfo.at(i).vPointType.at(nIndex)
						&& bGetCoors && nStepNum > 0)
					{
						nStepNum--;
						vtCoors.push_back(vtRealPoint.at(nIndex));
						vnPtType.push_back(tBoardInfo.vtBoardPartInfo.at(i).vPointType.at(nIndex));
						continue;
					}
					if (E_WELD_TRACK == tBoardInfo.vtBoardPartInfo.at(i).vPointType.at(nIndex) && bGetCoors
						/*|| bGetCoors && nIndex == vtRealPoint.size() - 1*/) {
						tBoardInfo.vvtTrackWarpCoors.push_back(vtCoors);
						tBoardInfo.vvnTrackWarpPtType.push_back(vnPtType);
						bGetCoors = false;
						vtCoors.clear();
						vnPtType.clear();
						 nStepNum = 90.0 / dStepDis;
					}
#endif // 0
				}
				m_pTraceModel->m_vvtTrackWarpCoors = tBoardInfo.vvtTrackWarpCoors;
				m_pTraceModel->m_vvnTrackWarpPtType = tBoardInfo.vvnTrackWarpPtType;
              // 获取包角起始终止点和转角特征点
				 m_pTraceModel->m_vvtWarpCorrectCoors.clear();
				 vector<T_ROBOT_COORS> vtWarpCoors;
				 vtWarpCoors.push_back(vtWarpEnd.at(vtWarpEnd.size() - 1));
				 vtWarpCoors.push_back(vtWarpEnd.at(0));
				 m_pTraceModel->m_vvtWarpCorrectCoors.push_back(vtWarpCoors);

				 vtWarpCoors.clear();
				 vtWarpCoors.push_back(vtWarpStrat.at(0));
				 vtWarpCoors.push_back(vtWarpStrat.at(vtWarpStrat.size() - 1));
				 m_pTraceModel->m_vvtWarpCorrectCoors.push_back(vtWarpCoors);
				 
				 for (int  i = 0; i < m_pTraceModel->m_vvtTrackWarpCoors.size(); i++)
				 {
					 m_pTraceModel->m_vvtWarpCorrectCoors.at(i).insert(m_pTraceModel->m_vvtWarpCorrectCoors.at(i).begin(),
						 m_pTraceModel->m_vvtTrackWarpCoors.at(i).at(0));
					 m_pTraceModel->m_vvtWarpCorrectCoors.at(i).push_back(
						 m_pTraceModel->m_vvtTrackWarpCoors.at(i).at(m_pTraceModel->m_vvtTrackWarpCoors.at(i).size()-1));
				 }
			}
		}
		else if (tBoardInfo.vvtMeasureResult_Warp[i].size() == 0)
		{

			vtRealPoint.erase(vtRealPoint.begin(), vtRealPoint.begin() + nStartDelNum);
			vtRealPoint.erase(vtRealPoint.end() - nEndDelNum, vtRealPoint.end());
			auto vtPtnType = &tBoardInfo.vtBoardPartInfo.at(i).vPointType;
			vtPtnType->erase(vtPtnType->begin(), vtPtnType->begin() + nStartDelNum);
			vtPtnType->erase(vtPtnType->end() - nEndDelNum, vtPtnType->end());
		}
		tBoardInfo.vvRealTrackRobotCoors.push_back(vtRealPoint);
		tBoardInfo.vtBoardPartInfo.at(i).vRealTrackCoors = vtRealPoint;
	}

	//处理焊接数据 打断郭总施法是小狗
	tBoardInfo.vtRealWeldTrackCoors.clear();
	tBoardInfo.vnDataPointType.clear();
	for (int n = 0; n < tBoardInfo.vvRealTrackRobotCoors.size(); n++)
	{
		if (tBoardInfo.vvRealTrackRobotCoors.at(n).size() <= 0)
		{
			continue;
		}
		tBoardInfo.vtRealWeldTrackCoors.insert(tBoardInfo.vtRealWeldTrackCoors.end(),
			tBoardInfo.vvRealTrackRobotCoors.at(n).begin(), tBoardInfo.vvRealTrackRobotCoors.at(n).end());
		tBoardInfo.vnDataPointType.insert(tBoardInfo.vnDataPointType.end(), tBoardInfo.vtBoardPartInfo.at(n).vPointType.begin(), tBoardInfo.vtBoardPartInfo.at(n).vPointType.end());
	}
	double dAdjustDisX = 50., dAdjustDisY = 50.;
	// 添加收下枪安全位置
	T_ROBOT_COORS tCoorSafe = tBoardInfo.vtRealWeldTrackCoors.at(0);
	tCoorSafe.dX = tCoorSafe.dX + dAdjustDisX * CosD(m_pRobotDriver->RzToDirAngle(tCoorSafe.dRZ));
	tCoorSafe.dY = tCoorSafe.dY - dAdjustDisY * SinD(m_pRobotDriver->RzToDirAngle(tCoorSafe.dRZ));
	tCoorSafe.dZ = m_dWorkPieceHighTop + 20 * m_nRobotInstallDir;
	tBoardInfo.vtRealWeldTrackCoors.insert(tBoardInfo.vtRealWeldTrackCoors.begin(), tCoorSafe);
	 tBoardInfo.vnDataPointType.insert(tBoardInfo.vnDataPointType.begin(), E_TRANSITION_POINT);

	tCoorSafe = tBoardInfo.vtRealWeldTrackCoors.at(tBoardInfo.vtRealWeldTrackCoors.size() - 1);
	tCoorSafe.dX = tCoorSafe.dX + dAdjustDisX * CosD(m_pRobotDriver->RzToDirAngle(tCoorSafe.dRZ));
	tCoorSafe.dY = tCoorSafe.dY - dAdjustDisY * SinD(m_pRobotDriver->RzToDirAngle(tCoorSafe.dRZ));
	tCoorSafe.dZ = m_dWorkPieceHighTop + 20 * m_nRobotInstallDir;
	tBoardInfo.vtRealWeldTrackCoors.push_back(tCoorSafe);
	 tBoardInfo.vnDataPointType.push_back(E_TRANSITION_POINT);
	SaveDataRobotCoors(tBoardInfo.vtRealWeldTrackCoors, tBoardInfo.vtBoardPartInfo.at(2).strfileName + "DoWeldTrackCoors.txt");
	SaveDataInt(tBoardInfo.vnDataPointType, tBoardInfo.vtBoardPartInfo.at(2).strfileName + "DoWeldTrackType.txt");
	 for (int i = 0; i < m_pTraceModel->m_vvtTrackWarpCoors.size(); i++)
	 {
		 CString strWarpFile;
		 string strFile;
		 strWarpFile.Format("%s%d", tBoardInfo.vtBoardPartInfo.at(2).strfileName.c_str(), i);
		 strFile = strWarpFile.GetBuffer(0);
		 SaveDataRobotCoors(m_pTraceModel->m_vvtTrackWarpCoors.at(i), strFile + "TrackWarpCoors.txt");
		 SaveDataInt(m_pTraceModel->m_vvnTrackWarpPtType.at(i), strFile + "TrackWarpPtType.txt");
		 SaveDataRobotCoors(m_pTraceModel->m_vvtWarpCorrectCoors.at(i), strFile + "WarpCorrectCoors.txt");
	 }
	 SaveWarpNumber(pRobotDriver, m_pTraceModel->m_vvtTrackWarpCoors.size());
	 //计算连续可运行脉冲坐标
	 if (!CalcContinuePulseForWeld(tBoardInfo.vtRealWeldTrackCoors, tBoardInfo.vtRealWeldTrackPulse, false))
	 {
		 XiMessageBoxOk("计算连续焊接轨迹关节坐标失败！");
		 return false;
	 }
	 // 生成理论轨迹job,用于测试
	 GenerateJobLocalVariable(tBoardInfo.vtRealWeldTrackPulse, MOVJ, "REALWELDTRACK");
	 return true;
 }

 bool CDiaphragmWeld::DoWeld(CRobotDriverAdaptor* pRobotDriver, T_BOARD_INFO_NEW& tBoardInfo)
 {
	 if (tBoardInfo.vvRealTrackRobotCoors.size() < 5)
	 {
		 XiMessageBox("数据有误请检查");
	 }
	 vector<T_ROBOT_COORS> vtRealWeldCoors = tBoardInfo.vtRealWeldTrackCoors;
	 vector<T_ANGLE_PULSE> vtRealWeldPulse = tBoardInfo.vtRealWeldTrackPulse;

	int nSize = vtRealWeldCoors.size();
	// 记录跟踪结尾数据
	if (!tBoardInfo.vtBoardPartInfo.at(2).bIsTrackingScanEnd
		&& tBoardInfo.vtBoardPartInfo.at(2).bTracking
		&& !m_ptUnit->m_bBreakPointContinue)
	{
		RecordEndPointCoors(pRobotDriver, vtRealWeldCoors, m_pTraceModel->m_vtWeldLinePointType, tBoardInfo.vtBoardPartInfo.at(2).eWarpType);
	}
	else if (m_ptUnit->m_bBreakPointContinue)
	{
		m_pTraceModel->m_vtRealEndpointCoor.clear();
		CString str;
		str.Format(".\\WeldData\\%s\\%d_RealEndpointCoors.txt", pRobotDriver->m_strRobotName, tBoardInfo.vtCornerRelsve_Info.at(2).nBoardNo);
		LoadDataRobotCoors(m_pTraceModel->m_vtRealEndpointCoor, str.GetBuffer(0));
	}
	if (!g_bLocalDebugMark && tBoardInfo.vtBoardPartInfo.at(2).bTracking)
	{
		m_ptUnit->SwitchDHCamera(m_ptUnit->m_nMeasureCameraNo, true, E_CALL_BACK_MODE_WAIT_IMAGE, E_ACQUISITION_MODE_CONTINUE);
	}
	//初始段数据处理
	 if (!WeldProcessBefor(pRobotDriver, vtRealWeldCoors, tBoardInfo.vnDataPointType, vtRealWeldPulse,tBoardInfo.vtBoardPartInfo.at(2).bTracking)) // 下标2焊缝为初始索引焊道
	{
		XiMessageBoxOk("初始段数据处理失败");
		return false;
	}
	//开始焊接
	CHECK_BOOL_RETURN(DoweldRuning(pRobotDriver, pRobotDriver->m_vtWeldLineInWorldPoints, m_pTraceModel->m_vtWeldLinePointType, tBoardInfo.vtBoardPartInfo.at(2).bTracking));
	// 更改焊接状态
	for (int i = 0; i < tBoardInfo.vtCornerRelsve_Info.size(); i++)
	{
		if (E_NO_CONTINUOUS == tBoardInfo.eContinueType && 2 != i ||
			(tBoardInfo.nContinueStartNo > i || i > tBoardInfo.nContinueStartNo + tBoardInfo.nContinueNum - 1) ||
			!tBoardInfo.vbBoardDoesItExist.at(i) || LoadWeldStatus(tBoardInfo.vtCornerRelsve_Info.at(i).nBoardNo) ||
			tBoardInfo.vtCornerRelsve_Info.at(i).bIfWeldCom)// 自身焊接或不存在	 
		{
			continue;
		}
		RecordWeldStatus(tBoardInfo.vtCornerRelsve_Info.at(i).nBoardNo, true);
	}
	if (!g_bLocalDebugMark && tBoardInfo.vtBoardPartInfo.at(2).bTracking)
	{
		m_ptUnit->SwitchDHCamera(m_ptUnit->m_nMeasureCameraNo,false);
	}
	return true;
}

 bool CDiaphragmWeld::Weld(int nGroupNo)
 {
	 m_dSafePosRunSpeed = 1000.;//安全位置移动速度，快速
	 m_dFastApproachToWorkSpeed = 500.;//快速靠近工件， 中速
	 m_dSafeApproachToWorkSpeed = 300.;//安全移动到工件速度，慢速}	


	 // 加载 理论轨迹数据
	 vector<T_ROBOT_COORS> vtRealWeldCoors;
	 vector<T_ANGLE_PULSE> vtRealWeldPulse;
	 vector<int> vnDataPointType;

	 // 加载搜索数据，世界坐标

	 AddSafeDownGunPos(vtRealWeldCoors, vnDataPointType, 50, 1800.0);
	 // 记录结尾数据
	 RecordEndPointCoors(m_ptUnit->GetRobotCtrl(), vtRealWeldCoors, vnDataPointType);
	 // 初始段数据处理
	 E_DHGIGE_ACQUISITION_MODE eCameraMode = E_ACQUISITION_MODE_SOURCE_SOFTWARE;
	 E_DHGIGE_CALL_BACK eCallBack = E_CALL_BACK_MODE_OFF;
	 m_ptUnit->SwitchDHCamera(m_ptUnit->m_nTrackCameraNo, true, true, eCameraMode, eCallBack); // 下枪时 开相机 激光
	 m_ptUnit->m_vpImageCapture[m_ptUnit->m_nTrackCameraNo]->StartAcquisition();
	 if (!WeldProcessBefor(m_ptUnit->GetRobotCtrl(), vtRealWeldCoors, vnDataPointType, vtRealWeldPulse, TRUE))
	 {
		 XiMessageBoxOk("初始段数据处理失败");
		 return false;
	 }
	 //开始焊接
	 if (!DoweldRuning(m_ptUnit->GetRobotCtrl(), m_ptUnit->GetRobotCtrl()->m_vtWeldLineInWorldPoints, vnDataPointType, true))
	 {
		 XiMessageBoxOk("焊接失败");
		 return false;
	 }
	 m_ptUnit->SwitchDHCamera(m_ptUnit->m_nTrackCameraNo, false); // 下枪时 开相机 激光
	 return true;
 }

bool CDiaphragmWeld::RecordEndPointCoors(CRobotDriverAdaptor* pRobotDriver, vector<T_ROBOT_COORS> vtWeldCoors,vector<int> vnWeldCoorsType, E_WRAPANGLE_TYPE eWarpType)
{
	// 结尾数据存储
	if (m_ptUnit->m_bBreakPointContinue)
	{
		//LoadEndpointData(pRobotDriver, m_pTraceModel->m_tRealEndpointCoor, false);
		return true;
	}
	if (0 == m_pTraceModel->m_vtRealEndpointCoor.size())
	{
		m_pTraceModel->m_vtRealEndpointCoor.clear();
		T_ROBOT_COORS tEndpont;
		vector<int> vFirstWarpStepNo;
		bool bIsRecord = false;
		switch (eWarpType)
		{
		case E_WRAPANGLE_ONCE:
		case E_WRAPANGLE_TWO_SINGLE:
		case E_WRAPANGLE_TWO_DOUBLE:
			for (int n = 0; n < vnWeldCoorsType.size(); n++) {
				if (E_WELD_WRAP == vnWeldCoorsType.at(n) && !bIsRecord)
				{
					vFirstWarpStepNo.push_back(n);
					bIsRecord = true;
				}
				else if (E_WELD_TRACK == vnWeldCoorsType.at(n) && bIsRecord) {
					bIsRecord = false;
				}
			}
			for (int i = 0; i < vFirstWarpStepNo.size(); i++)
			{
				m_pTraceModel->m_vtRealEndpointCoor.push_back(vtWeldCoors.at(vFirstWarpStepNo.at(i)));
				if (0 == i) {
					tEndpont = vtWeldCoors.at(vFirstWarpStepNo.at(i));
				}
			}

			break;
		default:
			break;
		}

		// !!!!!!!! 一次包角终点提前30mm 点间距2mm 15个点
		/*int nEndOffsetPtnNum = E_WRAPANGLE_ONCE == eWarpType ? 15 : 0;
		vtWeldCoors[vtWeldCoors.size() - 1 - nEndOffsetPtnNum] = vtWeldCoors[vtWeldCoors.size() - 1];
		vtWeldCoors.erase(vtWeldCoors.end() - nEndOffsetPtnNum, vtWeldCoors.end());
		vnWeldCoorsType.erase(vnWeldCoorsType.end() - nEndOffsetPtnNum, vnWeldCoorsType.end());*/

		// 最后安全位置前一个为结尾点
		m_pTraceModel->m_vtRealEndpointCoor.push_back(vtWeldCoors.at(vtWeldCoors.size() - 2));
		// 闭合圆弧起点做终点
		if (E_CLOSEDARC == m_pTraceModel->m_tWorkPieceType)
		{
			m_pTraceModel->m_vtRealEndpointCoor.clear();
			m_pTraceModel->m_vtRealEndpointCoor.push_back(vtWeldCoors.at(1));
		}
	}
	
	// 流程内使用结尾点赋初始值
	m_pTraceModel->m_tRealEndpointCoor = m_pTraceModel->m_vtRealEndpointCoor.at(0);
	CString str;
	//str.Format(".\\WeldData\\%s\\%d_RealEndpointCoors.txt", pRobotDriver->m_strRobotName, tBoardInfo.vtCornerRelsve_Info.at(2).nBoardNo);
	//SaveDataRobotCoors(m_pTraceModel->m_vtRealEndpointCoor, str.GetBuffer(0));
	SaveEndpointData(pRobotDriver, m_pTraceModel->m_tRealEndpointCoor, false);//起点
	return true;
}

bool CDiaphragmWeld::CalcWarpTrack(vector<T_ROBOT_COORS> vtEndpoint, vector<T_ROBOT_COORS>& vtWarpStart, vector<T_ROBOT_COORS>& vtWarpEnd)
{
	if (vtEndpoint.size() < 1)
	{
		return false;
	}
	vector<T_ROBOT_COORS> vtEndpointAfter;
	for (int n = 0; n < vtEndpoint.size(); n++)
	{
		vtEndpoint.at(n).dY += vtEndpoint.at(n).dBY;
	}
	vtEndpointAfter.push_back(RobotCoordPosOffset(vtEndpoint.at(0), vtEndpoint.at(1), vtEndpoint.at(0), m_tWrapAngelParaNew.m_dWrapdParallel/*m_dParallelAdjust1*/));//正面起点
	vtEndpointAfter.push_back(RobotCoordPosOffset(vtEndpoint.at(1), vtEndpoint.at(0), vtEndpoint.at(1), m_tWrapAngelParaNew.m_dWrapdParallel2/*m_dParallelAdjust1*/));//正面结尾					 
	vtEndpointAfter.push_back(RobotCoordPosOffset(vtEndpoint.at(2), vtEndpoint.at(3), vtEndpoint.at(2), m_tWrapAngelParaNew.m_dWrapdParallel3/*m_dParallelAdjust1*/));//反面起点
	vtEndpointAfter.push_back(RobotCoordPosOffset(vtEndpoint.at(3), vtEndpoint.at(2), vtEndpoint.at(3), m_tWrapAngelParaNew.m_dWrapdParallel4/*m_dParallelAdjust1*/));//反面结尾					 
	vtEndpointAfter.at(0).dRZ -= 45.0 * m_nRobotInstallDir;
	vtEndpointAfter.at(2).dRZ -= 45.0 * m_nRobotInstallDir;
	vtEndpointAfter.at(1).dRZ += 45.0 * m_nRobotInstallDir;
	vtEndpointAfter.at(3).dRZ += 45.0 * m_nRobotInstallDir;
	vtWarpStart.clear();
	vtWarpEnd.clear();

	for (int i = 0; i < 2; i++)
	{
		int n1 = 0, n2 = 3;
		if (i > 0)
		{
			n1 = 2, n2 = 1;
		}
		double dDis = TwoPointDis(vtEndpointAfter.at(n1).dX, vtEndpointAfter.at(n1).dY, vtEndpointAfter.at(n1).dZ,
			vtEndpointAfter.at(n2).dX, vtEndpointAfter.at(n2).dY, vtEndpointAfter.at(n2).dZ);
		int nStep = dDis / 2;
		if (nStep < 4)
		{
			nStep = 4;
		}
		double dStepDis = dDis / (nStep - 1);
		double dStepAngle = 90.0 / (nStep - 1);
		double dDirX = (vtEndpointAfter.at(n1).dX - vtEndpointAfter.at(n2).dX) / dDis;
		double dDirY = (vtEndpointAfter.at(n1).dY - vtEndpointAfter.at(n2).dY) / dDis;
		double dDirZ = (vtEndpointAfter.at(n1).dZ - vtEndpointAfter.at(n2).dZ) / dDis;
		for (int n = 0; n < nStep; n++)
		{
			T_ROBOT_COORS Stratpt = vtEndpointAfter.at(n2);
			Stratpt.dX += dDirX * dStepDis * n;
			Stratpt.dY += dDirY * dStepDis * n;
			Stratpt.dZ += dDirZ * dStepDis * n;
			Stratpt.dRZ += dStepAngle * n * m_nRobotInstallDir;

			Stratpt.dBY = Stratpt.dY + 250.;
			Stratpt.dY = -250.;

			if (i > 0)
			{
				vtWarpEnd.push_back(Stratpt);
			}
			else
			{
				vtWarpStart.push_back(Stratpt);
			}
		}
	}
	return true;
}

void CDiaphragmWeld::GetChangePosturePare(vector<T_ROBOT_COORS> tLeftTrackCoors, XI_POINT tLeftCenter, double dLeftNormal, bool bLeftIsArc, vector<T_ROBOT_COORS> tRightTrackCoors, XI_POINT tRightCenter, double dRightNormal, bool bRightIsArc, bool bStartOrEnd,
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
	if (dAngle < 85) //夹角小于85度
	{
		bool bIsLineLeft = bLeftIsArc;
		bool bIsLineRight = bRightIsArc;
		if (!bStartOrEnd) // 结尾时左侧反转数据
		{
			reverse(tLeftTrackCoors.begin(), tLeftTrackCoors.end());
		}
		int nSize = tLeftTrackCoors.size() < tRightTrackCoors.size() ? tLeftTrackCoors.size() : tRightTrackCoors.size();
		for (int n = 0; n < nSize - 1; n++)
		{
			if (bStartOrEnd) // 起始段
			{
				// 直线 直线 // 直线 圆弧
				if (!bIsLineLeft && !bIsLineRight || !bIsLineLeft && bIsLineRight)
				{
					CalcLineLineIntersection(tRightTrackCoors.at(n), m_pRobotDriver->RzToDirAngle(tRightTrackCoors.at(n).dRZ),
						tLeftTrackCoors.at(0), m_pRobotDriver->RzToDirAngle(tLeftTrackCoors.at(0).dRZ - 90 * m_nRobotInstallDir), intersection);

				}
				else if (bIsLineLeft && !bIsLineRight)// 圆弧 直线
				{
					XiAlgorithm xiAlg;
					T_ALGORITHM_POINT_2D tPointStart = { tRightTrackCoors.at(n).dX,tRightTrackCoors.at(n).dY };
					T_ALGORITHM_POINT_2D tPointEnd = { tRightTrackCoors.at(n).dX + 50 * CosD(m_pRobotDriver->RzToDirAngle(tRightTrackCoors.at(n).dRZ)),
													   tRightTrackCoors.at(n).dY + 50 * SinD(m_pRobotDriver->RzToDirAngle(tRightTrackCoors.at(n).dRZ)) };
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
			else //结尾段
			{
				// 直线 直线 // 圆弧 直线 
				if (!bIsLineLeft && !bIsLineRight || bIsLineLeft && !bIsLineRight)
				{
					CalcLineLineIntersection(tLeftTrackCoors.at(n), m_pRobotDriver->RzToDirAngle(tLeftTrackCoors.at(n).dRZ),
						tRightTrackCoors.at(0), m_pRobotDriver->RzToDirAngle(tRightTrackCoors.at(0).dRZ - 90 * m_nRobotInstallDir), intersection);

				}
				else if (!bIsLineLeft && bIsLineRight)// 直线 圆弧 
				{
					XiAlgorithm xiAlg;
					T_ALGORITHM_POINT_2D tPointStart = { tLeftTrackCoors.at(n).dX,tLeftTrackCoors.at(n).dY };
					T_ALGORITHM_POINT_2D tPointEnd = { tLeftTrackCoors.at(n).dX + 50 * CosD(m_pRobotDriver->RzToDirAngle(tLeftTrackCoors.at(n).dRZ)),
													   tLeftTrackCoors.at(n).dY + 50 * SinD(m_pRobotDriver->RzToDirAngle(tLeftTrackCoors.at(n).dRZ)) };
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


bool CDiaphragmWeld::GetTeachTrackNew(CRobotDriverAdaptor* pRobotDriver, T_BOARD_INFO_NEW& tBoardInfo, int nIndex, vector<T_ROBOT_COORS>& vtTeachTrack, vector<T_ANGLE_PULSE>& vtMeasurePulse, vector<int>& vnMeasureType, T_ANGLE_PULSE* ptPrePulse)
{
	vector<T_ROBOT_COORS> vtWeldTrack;
	T_ROBOT_COORS tTeachCoord;
	T_ANGLE_PULSE tTeachPulse;
	vtTeachTrack.clear();
	vtMeasurePulse.clear();
	vnMeasureType.clear();
	if (tBoardInfo.vtCornerRelsve_Info.size() < 1)
	{
		XiMessageBox("GetTeachTrack:输入数据有误");
		return false;
	}
	tBoardInfo.vvtMeasure_Info.clear();
	int nSize = tBoardInfo.vtCornerRelsve_Info.size();
	bool bTracking = tBoardInfo.vtBoardPartInfo.at(nIndex).bTracking;
	T_CORNER_LINE_INFO_NEW tCornerLineInfo = tBoardInfo.vtCornerRelsve_Info.at(nIndex);
	vtWeldTrack = tBoardInfo.vvtheoryTrackRobotCoors.at(nIndex);
	int nSeamPtnNum = vtWeldTrack.size();
	if (nSeamPtnNum < (int)(m_dMeasureDisThreshold * 3.0 + m_dHandEyeDis) &&
		nSeamPtnNum >80 && 3 == tCornerLineInfo.nStartType)
	{
		//测量立板长度小于250时需要便姿态测量，使测量面尽量靠近焊缝交点。
		//测量时激光角点在在焊道上偏移80MM时，此时枪尖相对交点法相偏移90，焊道方向偏移20,
		//角度逆向偏转55度
		int nStep = 2;
		if (tCornerLineInfo.bBoardType)//圆弧
		{
			nStep = 4;
		}
		double dShiftDis1 = 60.0;
		double dShiftDis2 = (tBoardInfo.vtCornerRelsve_Info.at(nIndex).dBoardLength - fabs(dShiftDis1)) / nStep;

		tTeachCoord = vtWeldTrack[dShiftDis1];
		tTeachCoord.dRZ += 55 * m_nRobotInstallDir;
		tTeachCoord.dRZ = tTeachCoord.dRZ > 180 ? tTeachCoord.dRZ - 360 : tTeachCoord.dRZ;
		tTeachCoord.dRZ = tTeachCoord.dRZ < -180 ? tTeachCoord.dRZ + 360 : tTeachCoord.dRZ;
		vtTeachTrack.push_back(tTeachCoord);
		vnMeasureType.push_back(E_DOUBLE_LONG_LINE);
		tTeachCoord = vtWeldTrack[dShiftDis1 + dShiftDis2];
		tTeachCoord.dRZ += 55 * m_nRobotInstallDir;
		tTeachCoord.dRZ = tTeachCoord.dRZ > 180 ? tTeachCoord.dRZ - 360 : tTeachCoord.dRZ;
		tTeachCoord.dRZ = tTeachCoord.dRZ < -180 ? tTeachCoord.dRZ + 360 : tTeachCoord.dRZ;
		vtTeachTrack.push_back(tTeachCoord);
		vnMeasureType.push_back(E_DOUBLE_LONG_LINE);
		if (true == tCornerLineInfo.bBoardType)//圆弧
		{
			tTeachCoord = vtWeldTrack[dShiftDis1 + dShiftDis2 * 2];
			tTeachCoord.dRZ += 55 * m_nRobotInstallDir;
			tTeachCoord.dRZ = tTeachCoord.dRZ > 180 ? tTeachCoord.dRZ - 360 : tTeachCoord.dRZ;
			tTeachCoord.dRZ = tTeachCoord.dRZ < -180 ? tTeachCoord.dRZ + 360 : tTeachCoord.dRZ;
			vtTeachTrack.push_back(tTeachCoord);
			vnMeasureType.push_back(E_DOUBLE_LONG_LINE);
		}
	}
	else
	{
		int nSIdx = 0;
		int nEIdx = 0;
		if (0 == tCornerLineInfo.bBoardType)
		{
			nSIdx = (int)(m_dMeasureDisThreshold + m_dHandEyeDis);
			nEIdx = (int)(nSeamPtnNum - 2 * m_dMeasureDisThreshold);

			if (nSeamPtnNum < (int)(m_dMeasureDisThreshold * 3.0 + m_dHandEyeDis) &&
				nSeamPtnNum >80 && 0 == tCornerLineInfo.nStartType)
			{
				nSIdx = (int)m_dMeasureDisThreshold;
				nEIdx = (int)m_dMeasureDisThreshold + (nSeamPtnNum - m_dMeasureDisThreshold) / 2;
			}
		}
		else
		{
			nSIdx = 0 == tCornerLineInfo.nStartType ? (int)m_dHandEyeDis : m_dHandEyeDis + m_dMeasureDisThreshold;
			nEIdx = (int)(nSeamPtnNum - m_dMeasureDisThreshold);
		}


		if (nSIdx >= nEIdx)
		{
			XiMessageBox("计算示教轨迹失败！");
			return false;
		}
		vtTeachTrack.push_back(vtWeldTrack[nSIdx]);
		vnMeasureType.push_back(E_DOUBLE_LONG_LINE);
		if (true == tCornerLineInfo.bBoardType)
		{
			vtTeachTrack.push_back(vtWeldTrack[(nSIdx + nEIdx) / 2]);
			vnMeasureType.push_back(E_DOUBLE_LONG_LINE);
		}
		vtTeachTrack.push_back(vtWeldTrack[nEIdx]);
		vnMeasureType.push_back(E_DOUBLE_LONG_LINE);

		CString str1;
		str1.Format("%sMidThoeryTeachCoors.txt", tBoardInfo.vtBoardPartInfo.at(nIndex).strfileName.c_str());
		SaveDataRobotCoors(vtTeachTrack, str1.GetBuffer(0));

		// 圆弧测量点间隔过小不测量
		double dDisScanInterval = TwoPointDis(
			vtTeachTrack[0].dX, vtTeachTrack[0].dY, vtTeachTrack[0].dZ,
			vtTeachTrack[1].dX, vtTeachTrack[1].dY, vtTeachTrack[1].dZ);
		if (true == tCornerLineInfo.bBoardType && dDisScanInterval < 20.0)
		{
			vtTeachTrack.clear();
			XiMessageBox("中间测量数据相邻距离太短");
			return false;
		}
		//直线先测后焊，中间间隔较远，插入中间测量数据
		if (false == tCornerLineInfo.bBoardType && !bTracking)
		{
			double dStepDis = 100.0;
			double dDis = TwoPointDis(vtTeachTrack[0].dX, vtTeachTrack[0].dY, vtTeachTrack[0].dZ,
				vtTeachTrack[vtTeachTrack.size() - 1].dX, vtTeachTrack[vtTeachTrack.size() - 1].dY, vtTeachTrack[vtTeachTrack.size() - 1].dZ);
			int nTempNum = dDis / dStepDis;
			if (nTempNum >= 2)
			{
				T_ROBOT_COORS tTempCoord;
				vector<T_ROBOT_COORS> vtMeasureCoordGunTool;
				vector<int> vMeasureType;
				for (int nPtnNo = 0; nPtnNo <= nTempNum; nPtnNo++)
				{
					tTempCoord = vtTeachTrack[0];
					tTempCoord.dX += ((vtTeachTrack[vtTeachTrack.size() - 1].dX - vtTeachTrack[0].dX) * (double)nPtnNo / (double)nTempNum);
					tTempCoord.dY += ((vtTeachTrack[vtTeachTrack.size() - 1].dY - vtTeachTrack[0].dY) * (double)nPtnNo / (double)nTempNum);
					tTempCoord.dZ += ((vtTeachTrack[vtTeachTrack.size() - 1].dZ - vtTeachTrack[0].dZ) * (double)nPtnNo / (double)nTempNum);
					vtMeasureCoordGunTool.push_back(tTempCoord);
					vMeasureType.push_back(E_DOUBLE_LONG_LINE);
				}
				vtTeachTrack.clear();
				vnMeasureType.clear();
				vtTeachTrack = vtMeasureCoordGunTool;
				vnMeasureType = vMeasureType;
			}
		}
	}
	tBoardInfo.vvtMeasure_Data_Mind.push_back(vtTeachTrack);

	// 分解世界坐标
	vector<T_ROBOT_COORS> vtTempCoors;
	CHECK_BOOL_RETURN(DecomposeWorldCoordinates(pRobotDriver, vtTempCoors, vtTeachTrack));
	vtTeachTrack.clear();
	vtTeachTrack = vtTempCoors;

	 double dAdjustDis = 50.;
	 // 添加收下枪安全位置
	 CHECK_BOOL_RETURN(AddSafeDownGunPos(vtTeachTrack, vnMeasureType, dAdjustDis, m_dWorkPieceHighTop));

	 //计算连续可运行脉冲坐标
	 CHECK_BOOL_RETURN(CalcContinuePulseForWeld(vtTeachTrack, vtMeasurePulse, false));
	 
	 return true;
 }
bool CDiaphragmWeld::GetMeasureCoors(CRobotDriverAdaptor* pRobotDriver, vector<T_ROBOT_COORS> vtWeldTrack, vector<T_ROBOT_COORS>& vtTeachTrack,
	vector<int>& vnMeasureType, bool bTracking, bool bIsArc, bool bStartType, bool bEndType, double dStepDis, bool bExAxisMoving)
{
	T_ROBOT_COORS tTeachCoord;
	T_ANGLE_PULSE tTeachPulse;
	vtTeachTrack.clear();
	vnMeasureType.clear();
	if (vtWeldTrack.size() < 1)
	{
		XiMessageBox("GetMeasureData:输入数据有误");
		return false;
	}
	int nSize = vtWeldTrack.size();
	int nSeamPtnNum = vtWeldTrack.size();
	if (nSeamPtnNum < (int)(m_dMeasureDisThreshold * 3.0 + m_dHandEyeDis) &&
		nSeamPtnNum >80 && bStartType)
	{
		//测量立板长度小于250时需要便姿态测量，使测量面尽量靠近焊缝交点。
		//测量时激光角点在在焊道上偏移80MM时，此时枪尖相对交点法相偏移90，焊道方向偏移20,
		//角度逆向偏转55度
		int nStep = 2;
		if (bIsArc)//圆弧
		{
			nStep = 4;
		}
		double dShiftDis1 = 60.0;
		double dShiftDis2 = (nSize - fabs(dShiftDis1)) / nStep;

		tTeachCoord = vtWeldTrack[dShiftDis1];
		tTeachCoord.dRZ += 55 * m_nRobotInstallDir;
		tTeachCoord.dRZ = tTeachCoord.dRZ > 180 ? tTeachCoord.dRZ - 360 : tTeachCoord.dRZ;
		tTeachCoord.dRZ = tTeachCoord.dRZ < -180 ? tTeachCoord.dRZ + 360 : tTeachCoord.dRZ;
		vtTeachTrack.push_back(tTeachCoord);
		vnMeasureType.push_back(E_DOUBLE_LONG_LINE);
		tTeachCoord = vtWeldTrack[dShiftDis1 + dShiftDis2];
		tTeachCoord.dRZ += 55 * m_nRobotInstallDir;
		tTeachCoord.dRZ = tTeachCoord.dRZ > 180 ? tTeachCoord.dRZ - 360 : tTeachCoord.dRZ;
		tTeachCoord.dRZ = tTeachCoord.dRZ < -180 ? tTeachCoord.dRZ + 360 : tTeachCoord.dRZ;
		vtTeachTrack.push_back(tTeachCoord);
		vnMeasureType.push_back(E_DOUBLE_LONG_LINE);
		if (bIsArc)//圆弧
		{
			tTeachCoord = vtWeldTrack[dShiftDis1 + dShiftDis2 * 2];
			tTeachCoord.dRZ += 55 * m_nRobotInstallDir;
			tTeachCoord.dRZ = tTeachCoord.dRZ > 180 ? tTeachCoord.dRZ - 360 : tTeachCoord.dRZ;
			tTeachCoord.dRZ = tTeachCoord.dRZ < -180 ? tTeachCoord.dRZ + 360 : tTeachCoord.dRZ;
			vtTeachTrack.push_back(tTeachCoord);
			vnMeasureType.push_back(E_DOUBLE_LONG_LINE);
		}
	}
	else
	{
		int nSIdx = 0;
		int nEIdx = 0;
		if (!bIsArc)
		{
			nSIdx = bStartType==true? (int)m_dMeasureDisThreshold:(int)(m_dHandEyeDis+m_dMeasureDisThreshold);
			nSIdx = (int)m_dHandEyeDis-1;
			nEIdx = dStepDis == 100 ? (int)(nSeamPtnNum - 2 * m_dMeasureDisThreshold) : (int)m_dHandEyeDis+m_dMeasureDisThreshold * 4-1;

			if (nSeamPtnNum < (int)(m_dMeasureDisThreshold * 3.0 + m_dHandEyeDis) &&
				nSeamPtnNum >80 && false == bStartType)
			{
				nSIdx = (int)m_dMeasureDisThreshold;
				nEIdx = (int)m_dMeasureDisThreshold + (nSeamPtnNum - m_dMeasureDisThreshold) / 2;
			}
		}
		else
		{
			nSIdx = false == bStartType ? (int)m_dHandEyeDis : m_dHandEyeDis + m_dMeasureDisThreshold;
			nEIdx = (int)(nSeamPtnNum - m_dMeasureDisThreshold);
		}


		 if (nSIdx >= nEIdx)
		 {
			 XiMessageBox("计算示教轨迹失败！");
			 return false;
		 }
		if (nSIdx >= vtWeldTrack.size() || nEIdx >= vtWeldTrack.size())
		{
			XiMessageBoxOk("件过短，大车示教测量轨迹计算失败 ");
			return false;
		}
		 vtTeachTrack.push_back(vtWeldTrack[nSIdx]);
		 vnMeasureType.push_back(E_DOUBLE_LONG_LINE);
		 if (bIsArc)
		 {
			 vtTeachTrack.push_back(vtWeldTrack[(nSIdx + nEIdx) / 2]);
			 vnMeasureType.push_back(E_DOUBLE_LONG_LINE);
		 }
		 vtTeachTrack.push_back(vtWeldTrack[nEIdx]);
		 vnMeasureType.push_back(E_DOUBLE_LONG_LINE);

		// 圆弧测量点间隔过小不测量
		double dDisScanInterval = TwoPointDis(
			vtTeachTrack[0].dX, vtTeachTrack[0].dY, vtTeachTrack[0].dZ,
			vtTeachTrack[1].dX, vtTeachTrack[1].dY, vtTeachTrack[1].dZ);
		if (bIsArc && dDisScanInterval < 20.0)
		{
			vtTeachTrack.clear();
			XiMessageBox("中间测量数据相邻距离太短");
			return false;
		}
		//直线先测后焊，中间间隔较远，插入中间测量数据
		if (false == bIsArc && !bTracking)
		{

			double dDis = TwoPointDis(vtTeachTrack[0].dX, vtTeachTrack[0].dY, vtTeachTrack[0].dZ,
				vtTeachTrack[vtTeachTrack.size() - 1].dX, vtTeachTrack[vtTeachTrack.size() - 1].dY, vtTeachTrack[vtTeachTrack.size() - 1].dZ);
			int nTempNum = Round(dDis/ dStepDis);

			if (nTempNum >= 2)
			{
				T_ROBOT_COORS tTempCoord;
				vector<T_ROBOT_COORS> vtMeasureCoordGunTool;
				vector<int> vMeasureType;
				for (int nPtnNo = 0; nPtnNo <= nTempNum; nPtnNo++)
				{
					tTempCoord = vtTeachTrack[0];
					tTempCoord.dX += ((vtTeachTrack[vtTeachTrack.size() - 1].dX - vtTeachTrack[0].dX) * (double)nPtnNo / (double)nTempNum);
					tTempCoord.dY += ((vtTeachTrack[vtTeachTrack.size() - 1].dY - vtTeachTrack[0].dY) * (double)nPtnNo / (double)nTempNum);
					tTempCoord.dZ += ((vtTeachTrack[vtTeachTrack.size() - 1].dZ - vtTeachTrack[0].dZ) * (double)nPtnNo / (double)nTempNum);
					vtMeasureCoordGunTool.push_back(tTempCoord);
					vMeasureType.push_back(E_DOUBLE_LONG_LINE);
				}
				vtTeachTrack.clear();
				vnMeasureType.clear();
				vtTeachTrack = vtMeasureCoordGunTool;
				vnMeasureType = vMeasureType;
			}
		}
	}
	return true;
}
bool CDiaphragmWeld::TransCameraToolAndCalcContinuePulse(CRobotDriverAdaptor* pRobotDriver, vector<T_ROBOT_COORS>& vtTeachTrack, vector<T_ANGLE_PULSE>& vtMeasurePulse, vector<int>& vnMeasureType, T_ANGLE_PULSE* ptPrePulse)
{
	if (vtTeachTrack.size() < 1 || vtTeachTrack.size() != vnMeasureType.size())
	{
		return false;
	}
	vtMeasurePulse.clear();
	// 分解世界坐标
	vector<T_ROBOT_COORS> vtTempCoors;
	CHECK_BOOL_RETURN(DecomposeWorldCoordinates(pRobotDriver, vtTempCoors, vtTeachTrack));
	vtTeachTrack.clear();
	vtTeachTrack = vtTempCoors;

	 // 添加收下枪安全位置
	 double dAdjustdis = 50.0;
	 CHECK_BOOL_RETURN(AddSafeDownGunPos(vtTeachTrack, vnMeasureType, dAdjustdis, m_dWorkPieceHighTop));
	 //计算连续可运行脉冲坐标
	 CHECK_BOOL_RETURN(CalcContinuePulseForWeld(vtTeachTrack, vtMeasurePulse, false));

	return true;
}

bool CDiaphragmWeld::JudjeMeasureIsavailable(T_ALGORITHM_POINT tStartPoint, T_ALGORITHM_POINT tEndPoint, vector<T_ALGORITHM_POINT> vtMaesureCoors,
	vector<T_ALGORITHM_POINT>& vtCoutPoint, double ThreShold, bool bIsReverse)
{
	if (vtMaesureCoors.size() < 2)
	{
		return false;
	}
	vector<int> vnStepNo;
	XiAlgorithm xiAlg;
	for (int n = 0; n < vtMaesureCoors.size(); n++)
	{
		T_ALGORITHM_POINT tPoint = vtMaesureCoors.at(n);
		double dDis = xiAlg.CalDisPointToLine(tPoint, tStartPoint, tEndPoint);
		if (dDis > ThreShold)
		{
			vnStepNo.push_back(n);
		}
		else
		{
			vtCoutPoint.push_back(vtMaesureCoors.at(n));
		}
	}
	if (vnStepNo.size() > vtCoutPoint.size())
	{
		XiMessageBox("测量数据偏差超出阈值");
		return false;
	}
	if (bIsReverse)
	{
		reverse(vtCoutPoint.begin(), vtCoutPoint.end());
	}
	double dStartDis = TwoPointDis(vtCoutPoint.at(0).dCoorX, vtCoutPoint.at(0).dCoorY, vtCoutPoint.at(0).dCoorZ,
		tStartPoint.dCoorX, tStartPoint.dCoorY, tStartPoint.dCoorZ);
	double dEndDis = TwoPointDis(vtCoutPoint.at(vtCoutPoint.size() - 1).dCoorX, vtCoutPoint.at(vtCoutPoint.size() - 1).dCoorY,
		vtCoutPoint.at(vtCoutPoint.size() - 1).dCoorZ, tStartPoint.dCoorX, tStartPoint.dCoorY, tStartPoint.dCoorZ);
	if (dStartDis > 20.)
	{
		vtCoutPoint.insert(vtCoutPoint.begin(), tStartPoint);
	}
	else
	{
		vtCoutPoint.at(0) = tStartPoint;
	}
	if (dEndDis > 20.)
	{
		vtCoutPoint.push_back(tEndPoint);
	}
	else
	{
		vtCoutPoint.at(vtCoutPoint.size() - 1) = tEndPoint;
	}
	return true;
}

bool CDiaphragmWeld::PointConnection(vector<T_ALGORITHM_POINT> vtCinPoint, vector<XI_POINT>& vtCoutPoint, double dStepDis)
{
	if (vtCinPoint.size() < 2)
	{
		XiMessageBox("PointConnection 数据量太少");
		return false;
	}
	vector<XI_POINT> vtPoint;
	double dSteplen = 1;
	vtCoutPoint.clear();
	for (int index = 0; index < vtCinPoint.size() - 1; index++)
	{
		double dis = TwoPointDis(vtCinPoint.at(index).dCoorX, vtCinPoint.at(index).dCoorY, vtCinPoint.at(index).dCoorZ,
			vtCinPoint.at(index + 1).dCoorX, vtCinPoint.at(index + 1).dCoorY, vtCinPoint.at(index + 1).dCoorZ);
		double dDirX = (vtCinPoint.at(index + 1).dCoorX - vtCinPoint.at(index).dCoorX) / dis;
		double dDirY = (vtCinPoint.at(index + 1).dCoorY - vtCinPoint.at(index).dCoorY) / dis;
		double dDirZ = (vtCinPoint.at(index + 1).dCoorZ - vtCinPoint.at(index).dCoorZ) / dis;
		int nStep = dis / dSteplen;
		for (int j = 0; j < (int)nStep; j++)
		{
			XI_POINT tPoint = {
			vtCinPoint.at(index).dCoorX + dDirX * dSteplen * j,
			vtCinPoint.at(index).dCoorY + dDirY * dSteplen * j,
			vtCinPoint.at(index).dCoorZ + dDirZ * dSteplen * j,
			};
			vtPoint.push_back(tPoint);
		}
	}
	for (int n = 0; n < vtPoint.size(); n += dStepDis)
	{
		vtCoutPoint.push_back(vtPoint.at(n));
	}
	double dis = TwoPointDis(vtCoutPoint.at(vtCoutPoint.size() - 1).x, vtCoutPoint.at(vtCoutPoint.size() - 1).y, vtCoutPoint.at(vtCoutPoint.size() - 1).z,
		vtCinPoint.at(vtCinPoint.size() - 1).dCoorX, vtCinPoint.at(vtCinPoint.size() - 1).dCoorY, vtCinPoint.at(vtCinPoint.size() - 1).dCoorZ);
	if (dis > 1)
	{
		XI_POINT tp = { vtCinPoint.at(vtCinPoint.size() - 1).dCoorX, vtCinPoint.at(vtCinPoint.size() - 1).dCoorY, vtCinPoint.at(vtCinPoint.size() - 1).dCoorZ };
		vtCoutPoint.push_back(tp);
	}
	return true;
}
bool CDiaphragmWeld::CalcCoordUsePosDirAngle(XI_POINT tPtn, double dDirAngle, double dExAxlePos, T_ROBOT_COORS& tRobotCoord, double dRx, double dRy)
{
	// Rz - InitRz = Dir * (DirAngle - InitDirAngle)    =》    Rz = Dir * (DirAngle - InitDirAngle) + InitRz
	double dDir = 1.0; // 方向角和Rz变化方向相同 1.0 相反 -1.0
	double dInitRz = 180.0;
	double dInitDirAngle = 180.0;

	tRobotCoord.dX = tPtn.x;
	tRobotCoord.dY = tPtn.y - dExAxlePos;
	tRobotCoord.dZ = tPtn.z;
	tRobotCoord.dRX = dRx;
	tRobotCoord.dRY = dRy;
	tRobotCoord.dRZ = dDir * (dDirAngle - dInitDirAngle) + dInitRz;
	return true;
}
void CDiaphragmWeld::RecordWeldStatus(int nEndPointNo, bool bStatus)
{
	COPini OPini;
	CString srtFilePath;
	srtFilePath.Format("Data\\RelevanceFill\\RelevanceWeldStatus.ini");
	OPini.SetFileName(srtFilePath.GetBuffer(0));
	CString strNo;
	strNo.Format("EndPoint_%d", nEndPointNo);
	OPini.SetSectionName(strNo);
	OPini.WriteString("bWeldStatus", bStatus);
}
bool CDiaphragmWeld::LoadWeldStatus(int nEndPointNo)
{
	bool bStatus;
	COPini _opini;
	CString cstrTmp;
	cstrTmp = DATA_PATH +m_ptUnit->GetUnitName() + "\\DiaphragmRelevance\\RelevanceWeldStatus.ini";
	_opini.SetFileName(cstrTmp);
	cstrTmp.Format("EndPoint_%d", nEndPointNo);
	_opini.SetSectionName(cstrTmp);
	_opini.ReadString("bWeldStatus", &bStatus);
	return  bStatus;
}


void CDiaphragmWeld::RecordCorrectEndPointFun(CRobotDriverAdaptor* pRobotDriver, int EndPointNo, XI_POINT tpEndPoint)
{
	int nRenew = 1;
	CString strNo;
	strNo.Format("EndPoint_%d", EndPointNo);
	COPini OPini;
	CString cstrFilePath;
	cstrFilePath = DATA_PATH + m_ptUnit->GetUnitName() + "\\DiaphragmRelevance\\RelevanceEndPoint.ini";
	//cstrFilePath.Format("Data\\RelevanceFill\\RelevanceEndPoint.ini");
	OPini.SetFileName(cstrFilePath);
	OPini.SetSectionName(strNo);
	OPini.WriteString("Renew", nRenew);
	OPini.WriteString("dEndPointX", tpEndPoint.x);
	OPini.WriteString("dEndPointY", tpEndPoint.y);
	OPini.WriteString("dEndPointZ", tpEndPoint.z);
}

bool CDiaphragmWeld::ReadCorrectEndPointFun(CRobotDriverAdaptor* pRobotDriver, int EndPointNo, XI_POINT& tpEndPoint)
{
	int nRenew = 0;
	CString strNo;
	strNo.Format("EndPoint_%d", EndPointNo);
	COPini OPini;
	CString srtFilePath;
	srtFilePath.Format("Data\\RelevanceFill\\RelevanceEndPoint.ini");
	OPini.SetFileName(srtFilePath.GetBuffer(0));
	OPini.SetSectionName(strNo);
	OPini.ReadString("Renew", &nRenew);
	if (1 != nRenew)
	{
		return false;
	}
	OPini.ReadString("dEndPointX", &tpEndPoint.x);
	OPini.ReadString("dEndPointY", &tpEndPoint.y);
	OPini.ReadString("dEndPointZ", &tpEndPoint.z);
	return true;
}


bool CDiaphragmWeld::CallJobRobotWeld(CRobotDriverAdaptor* pRobotDriver, vector<XI_POINT> vcPointOnTheSameLine, vector<XI_POINT> vcWeldPosture, bool bCalcPosture, bool bIfWeld)
{
	//CHECK_STOP;
	int nPostionSize = vcPointOnTheSameLine.size();
	int nPosTureSize = vcWeldPosture.size();
	if (nPostionSize != nPosTureSize || nPostionSize || nPosTureSize)
	{
		//已修改
		XUI::MesBox::PopInfo("CallJobRobotWeld函数输入数据有误：{0} {1} ", vcPointOnTheSameLine.size(), 0 == vcWeldPosture.size());
		//XiMessageBox("CallJobRobotWeld函数输入数据有误：%d %d ", vcPointOnTheSameLine.size(), 0 == vcWeldPosture.size());
		return false;
	}
	long long dStartTim;
	dStartTim = XI_clock();
	
	CXiRobotCtrl* pRobotDriver1 = m_pRobotDriver->GetXiRobotCtrl();
	int ncount = 0;
	int nindex = 0;
	int nCountInt1 = 0;
	double dPosCoorLRobot[6] = { 0 };
	vector<XI_POINT> vcWeldPostureOutPut;
	vcWeldPostureOutPut.clear();
	T_ROBOT_POSVAR_DATA tPosVar[100];
	vcWeldPostureOutPut = vcWeldPosture;
	//添加起始终点安全位置
	T_ANGLE_PULSE tRefPulse = m_pRobotDriver->GetCurrentPulse(), tResultPulse;
	T_ROBOT_MOVE_SPEED tPulseMove(500, 20, 20);
	T_ROBOT_COORS tStartSafe(
		vcPointOnTheSameLine.at(0).x,
		vcPointOnTheSameLine.at(0).y,
		vcPointOnTheSameLine.at(0).z,
		vcWeldPosture.at(0).x,
		vcWeldPosture.at(0).y,
		vcWeldPosture.at(0).z
		, 0, 0, 0
	);
	T_ROBOT_COORS tEndSafe(
		vcPointOnTheSameLine.at(nPostionSize - 1).x,
		vcPointOnTheSameLine.at(nPostionSize - 1).y,
		vcPointOnTheSameLine.at(nPostionSize - 1).z,
		vcWeldPosture.at(nPosTureSize - 1).x,
		vcWeldPosture.at(nPosTureSize - 1).y,
		vcWeldPosture.at(nPosTureSize - 1).z,
		0, 0, 0
	);
	double dStDirAgl = m_pRobotDriver->RzToDirAngle(tStartSafe.dRZ);
	double dEdDirAgl = m_pRobotDriver->RzToDirAngle(tStartSafe.dRZ);
	RobotCoordPosOffset(tStartSafe, dStDirAgl, 30., 30.);
	RobotCoordPosOffset(tEndSafe, dStDirAgl, 30., 30.);
	if (!pRobotDriver->RobotInverseKinematics(tStartSafe, tRefPulse, pRobotDriver->m_tTools.tGunTool, tResultPulse))
	{
		XiMessageBox("初始安全位置超出极限");
		return false;
	}
	if (!pRobotDriver->RobotInverseKinematics(tEndSafe, tRefPulse, pRobotDriver->m_tTools.tGunTool, tResultPulse))
	{
		XiMessageBox("结尾安全位置超出极限");
		return false;
	}
	pRobotDriver->MoveByJob(tStartSafe, tPulseMove, pRobotDriver->m_nExternalAxleType, "MOVL");
	if (!m_ptUnit->CheckRobotDone(tStartSafe))
	{
		XiMessageBox("为移动的安全位置");
		return false;
	}

	//设置第一个位置
	double dFirstPt[6] = { vcPointOnTheSameLine[0].x,vcPointOnTheSameLine[0].y,vcPointOnTheSameLine[0].z,
		vcWeldPostureOutPut[0].x,vcWeldPostureOutPut[0].y,vcWeldPostureOutPut[0].z };
	pRobotDriver->SetPosVar(19, dFirstPt, 0);
	vcPointOnTheSameLine.erase(vcPointOnTheSameLine.begin(), vcPointOnTheSameLine.begin() + 1);
	vcWeldPostureOutPut.erase(vcWeldPostureOutPut.begin(), vcWeldPostureOutPut.begin() + 1);

	pRobotDriver1->SetIntVar(2, nPostionSize);
	pRobotDriver1->SetIntVar(1, 0);
	WriteLog("进入赋值线程 ,size:%d", nPostionSize);
	int nPointNum = 0;;
	int nCirculateNum = 40;

	long long time2 = XI_clock();
	while (nindex < nPostionSize)
	{
		if (ncount < nPostionSize)
		{
			int nn = 0;
			if (ncount < nCirculateNum)
			{
				WriteLog("ncount:%d,焊接轨迹:%11.3lf%11.3lf%11.3lf", ncount, vcPointOnTheSameLine.at(nindex).x, vcPointOnTheSameLine.at(nindex).y, vcPointOnTheSameLine.at(nindex).z);

				tPosVar[ncount].usPorBPorExVar = MP_RESTYPE_VAR_ROBOT;
				tPosVar[ncount].usIndex = (ncount % 20 + 20);
				tPosVar[ncount].usPulseOrCart = MP_ROBO_COORD;
				tPosVar[ncount].usPosture = 4;
				tPosVar[ncount].usToolNum = 1;
				tPosVar[ncount].usUserCoordNum = 0;//该参数无意义
				tPosVar[ncount].ulValue[0] = (long)(vcPointOnTheSameLine[nindex].x * 1000);
				tPosVar[ncount].ulValue[1] = (long)(vcPointOnTheSameLine[nindex].y * 1000);
				tPosVar[ncount].ulValue[2] = (long)(vcPointOnTheSameLine[nindex].z * 1000);
				tPosVar[ncount].ulValue[3] = (long)(vcWeldPostureOutPut[nindex].x * 10000);
				tPosVar[ncount].ulValue[4] = (long)(vcWeldPostureOutPut[nindex].y * 10000);
				tPosVar[ncount].ulValue[5] = (long)(vcWeldPostureOutPut[nindex].z * 10000);
				tPosVar[ncount].ulValue[6] = 0;//该参数无意义
				tPosVar[ncount].ulValue[7] = 0;//该参数无意义
				if (ncount == nCirculateNum || ncount == (nPostionSize - 1))
				{
					pRobotDriver1->SetMultiPosVar((ncount + 1), tPosVar);
					GetNaturalPop().PopOkCancel("p变量发送完成");
					pRobotDriver->CallJob("CIRCULATION007");
					Sleep(10);
				}
				ncount++;
				nindex++;
				if (ncount == (nPostionSize))
				{
					WriteLog("数据小于19个直接运行job:%d", nPostionSize);
					break;
				}
			}
			else
			{
				nCountInt1 = pRobotDriver1->GetIntVar(1);

				while (nCountInt1 > (ncount - nCirculateNum))
				{
					if (nindex < nPostionSize)
					{
						WriteLog(0, "pRobotDriver->GetIntVar(1):%d nindex:%d nvcount:%d", pRobotDriver1->GetIntVar(1), nindex, nPostionSize);
						WriteLog("nCountInt1:%d,  ncount:%d,   mv_size:%d", nCountInt1, ncount, nPostionSize);
						dPosCoorLRobot[0] = vcPointOnTheSameLine[nindex].x;
						dPosCoorLRobot[1] = vcPointOnTheSameLine[nindex].y;
						dPosCoorLRobot[2] = vcPointOnTheSameLine[nindex].z;
						dPosCoorLRobot[3] = vcWeldPostureOutPut[nindex].x;
						dPosCoorLRobot[4] = vcWeldPostureOutPut[nindex].y;
						dPosCoorLRobot[5] = vcWeldPostureOutPut[nindex].z;
						m_pRobotDriver->SetPosVar(ncount % 20 + 20, dPosCoorLRobot, 0);
						//pRobotDriver->SetIntVar(ncount % 20 + 20, 800/6);
						WriteLog("nindex:%d,nvcount:%d,轨迹X:%lf Y: %lf Z :%lf", nindex, nPostionSize, dPosCoorLRobot[0], dPosCoorLRobot[1], dPosCoorLRobot[2]);
						ncount++;
						nindex++;
						
					}
					else
					{
						break;
					}
				}
			}
		}
	}
	WriteLog("赋值线程 RED exit!");
	return true;
}

T_LINE_PARA CDiaphragmWeld::CalcLineParamRansac(vector<XI_POINT> vtPoint, double dSelectedRatio)
{
	T_LINE_PARA tBestLineParam;
	T_SPACE_LINE_DIR tLineDir;

	double dMinError = 9999999.0;

	int nIdx = 0;
	int sIdx = 0;
	int tIdx = 0;
	int rIdx = 0;

	double dDist = 0.0;
	vector<double> vdDist;

	for (nIdx = 0; nIdx < vtPoint.size() - 1; nIdx++)
	{
		for (sIdx = nIdx + 1; sIdx < vtPoint.size(); sIdx++)
		{
			double dError = 0.0;
			vdDist.clear();
			for (rIdx = 0; rIdx < vtPoint.size(); rIdx++)
			{
				dDist = CalDisPointToLine(vtPoint[rIdx], vtPoint[nIdx], vtPoint[sIdx]);
				vdDist.push_back(dDist);
			}
			sort(vdDist.begin(), vdDist.end(), dCmpIncFunc);

			for (rIdx = 0; rIdx < vtPoint.size() * dSelectedRatio; rIdx++)
			{
				dError += vdDist[rIdx];
			}

			if (dError < dMinError)
			{
				dMinError = dError;
				tBestLineParam.dPointX = vtPoint[nIdx].x;
				tBestLineParam.dPointY = vtPoint[nIdx].y;
				tBestLineParam.dPointZ = vtPoint[nIdx].z;

				tLineDir = CalLineDir(vtPoint[nIdx], vtPoint[sIdx]);
				tBestLineParam.dDirX = tLineDir.dDirX;
				tBestLineParam.dDirY = tLineDir.dDirY;
				tBestLineParam.dDirZ = tLineDir.dDirZ;
				tBestLineParam.dLineError = dMinError;
			}
		}
	}

	return tBestLineParam;
}
T_LINE_PARA	CDiaphragmWeld::CalcLineParamRansac(vector<T_ALGORITHM_POINT> vtPoint, double dSelectedRatio)
{
	vector<XI_POINT> vtAlgorPoint;
	for (int n = 0; n < vtPoint.size(); n++)
	{
		XI_POINT tp = { vtPoint.at(n).dCoorX,vtPoint.at(n).dCoorY,vtPoint.at(n).dCoorZ };
		vtAlgorPoint.push_back(tp);
	}
	return CalcLineParamRansac(vtAlgorPoint, dSelectedRatio);
}

bool CDiaphragmWeld::dCmpIncFunc(double dFirstValue, double dSecondValue)
{
	return(dFirstValue < dSecondValue);
}

T_SPACE_LINE_DIR CDiaphragmWeld::CalLineDir(XI_POINT tStartPoint, XI_POINT tEndPoint)
{
	double dLineLength = 0.0;
	T_SPACE_LINE_DIR tLineDir;
	double dX = 0.0, dY = 0.0, dZ = 0.0;

	dX = tEndPoint.x - tStartPoint.x;
	dY = tEndPoint.y - tStartPoint.y;
	dZ = tEndPoint.z - tStartPoint.z;

	dLineLength = sqrt(SQUARE(dX) + SQUARE(dY) + SQUARE(dZ));

	if (fabs(dLineLength) < 0.0001)
	{
		tLineDir.dDirX = 0.0;
		tLineDir.dDirY = 0.0;
		tLineDir.dDirZ = 0.0;
	}
	else
	{
		tLineDir.dDirX = dX / dLineLength;
		tLineDir.dDirY = dY / dLineLength;
		tLineDir.dDirZ = dZ / dLineLength;
	}

	return tLineDir;
}

double CDiaphragmWeld::CalDisPointToLine(XI_POINT tPoint, XI_POINT tSegmentStartPoint, XI_POINT tSegmentEndPoint)
{
	double p, q, r;
	double m, n;
	double dDis = 0.0;

	p = tSegmentEndPoint.x - tSegmentStartPoint.x;
	q = tSegmentEndPoint.y - tSegmentStartPoint.y;
	r = tSegmentEndPoint.z - tSegmentStartPoint.z;

	m = sqrt((q * (tPoint.z - tSegmentStartPoint.z) - r * (tPoint.y - tSegmentStartPoint.y)) * (q * (tPoint.z - tSegmentStartPoint.z) - r * (tPoint.y - tSegmentStartPoint.y))
		+ (r * (tPoint.x - tSegmentStartPoint.x) - p * (tPoint.z - tSegmentStartPoint.z)) * (r * (tPoint.x - tSegmentStartPoint.x) - p * (tPoint.z - tSegmentStartPoint.z))
		+ (p * (tPoint.y - tSegmentStartPoint.y) - q * (tPoint.x - tSegmentStartPoint.x)) * (p * (tPoint.y - tSegmentStartPoint.y) - q * (tPoint.x - tSegmentStartPoint.x)));
	n = sqrt(p * p + q * q + r * r);

	if (n > 0)
	{
		dDis = m / n;
	}
	else
	{
		dDis = sqrt(SQUARE(tPoint.x - tSegmentStartPoint.x) + SQUARE(tPoint.y - tSegmentStartPoint.y) + SQUARE(tPoint.z - tSegmentStartPoint.z));
	}

	return dDis;
}

bool CDiaphragmWeld::FitCalLinePointsByMiddlePoints(XI_POINT tStartPoint, XI_POINT tEndPoint, vector<XI_POINT> vtScanPoints, vector<XI_POINT>& vtOutPoints, vector<XI_POINT>& vtPosture,
	double& dNormalAngle, int& nStartChangePointNo, int& nEndChangePointNo, double dDisPoint, int nPointNum, bool bCheckDisORNum, int nThresVal, bool bStartChangePosture, bool bEndChangePosture)
{
	//CHECK_STOP_RET(0);
	bool bIfSmoothSucess = false;
	if (vtScanPoints.size() < 2)
	{
		//已修改
		XUI::MesBox::PopInfo("FitLineDataX输入数据有误请检查：{0}", vtScanPoints.size());
		//XiMessageBox("FitLineDataX输入数据有误请检查：%d", vtScanPoints.size());
	}
	int nSize = vtScanPoints.size();
	XI_POINT tPoint, tStartProj, tEndProj;
	vector<XI_POINT> vtPoints;
	vector<XI_POINT> vtPointsAverage;
	vtPoints.clear();
	vtPointsAverage.clear();
	double dSelectedRatio = 0.7;
	T_LINE_PARA tLineParam;

	tLineParam = CalcLineParamRansac(vtScanPoints, dSelectedRatio);
	WriteLog("直线参数：%lf %lf %lf %lf %lf %lf", tLineParam.dPointX, tLineParam.dPointY, tLineParam.dPointZ,
		tLineParam.dDirX, tLineParam.dDirY, tLineParam.dDirZ);
	//求投影点
	PointtoLineProjection(tLineParam, tStartPoint, tStartProj);
	PointtoLineProjection(tLineParam, tEndPoint, tEndProj);

	double dLen, dTotalLen;
	dTotalLen = TwoPointDis(tStartProj.x, tStartProj.y, tStartProj.z, tEndProj.x, tEndProj.y, tEndProj.z);
	if (dTotalLen < 50.0)
	{
		//已修改
		XUI::MesBox::PopInfo("输入数据有误:{0:.3f}", dTotalLen);
		//XiMessageBox("输入数据有误:%.3lf", dTotalLen);
		return false;
	}
	if (bCheckDisORNum)
	{
		dLen = dDisPoint;
	}
	else
	{
		dLen = dTotalLen / nPointNum;
	}

	double dDisStart1 = TwoPointDis(tStartPoint.x, tStartPoint.y, tStartPoint.z, tStartProj.x, tStartProj.y, tStartProj.z);;
	double dDisEnd1 = TwoPointDis(tEndPoint.x, tEndPoint.y, tEndPoint.z, tEndProj.x, tEndProj.y, tEndProj.z);

	WriteLog("投影点：%lf %lf %lf 距离：%lf %lf", tStartProj.x, tStartProj.y, tStartProj.z, dDisStart1, dDisEnd1);
	if (dDisStart1 > nThresVal || dDisEnd1 > nThresVal)
	{

		GetNaturalPop().PopOkCancel("投影点：{0} {1} {2} 距离：{3} {4}", tStartProj.x, tStartProj.y, tStartProj.z, dDisStart1, dDisEnd1);
		return bIfSmoothSucess;
	}
	else
	{
		bIfSmoothSucess = true;
	}

	double dDirx = (tEndProj.x - tStartProj.x) / dTotalLen;
	double dDiry = (tEndProj.y - tStartProj.y) / dTotalLen;
	double dDirz = (tEndProj.z - tStartProj.z) / dTotalLen;

	for (int n = 0; n<int(dTotalLen / dLen); n++)
	{
		tPoint.x = tStartProj.x + n * dLen * dDirx;
		tPoint.y = tStartProj.y + n * dLen * dDiry;
		tPoint.z = tStartProj.z + n * dLen * dDirz;
		vtPointsAverage.push_back(tPoint);
	}
	XI_POINT dEd = { tEndProj.x,tEndProj.y,tEndProj.z };
	double dSmooth = TwoPointDis(vtPointsAverage[0].x, vtPointsAverage[0].y, vtPointsAverage[0].z, vtPointsAverage[vtPointsAverage.size() - 1].x, vtPointsAverage[vtPointsAverage.size() - 1].y, vtPointsAverage[vtPointsAverage.size() - 1].z);
	if (dSmooth < dTotalLen)
	{
		vtPointsAverage.push_back(dEd);
	}
	vtOutPoints.clear();
	vtOutPoints = vtPointsAverage;
	//添加坐标姿态
	vtPosture.clear();
	int nDir = 1;//姿态变化方向
	bool bStartChangeAngle = true;//顺序输出
	double dChangeAngleTheshold = 40.0; // 多长距离变姿态
	m_pTraceModel->m_dChangeAngleTheshold = dChangeAngleTheshold;
	int nChangePtnNum = dChangeAngleTheshold / dLen; // 多少个点变完姿态,3mm一个点45mm变化完成
	double dChangeAngleUnit = 45.0 / (double)nChangePtnNum; // 相邻点间角度变化幅度
	if (true == bStartChangePosture)
	{
		nStartChangePointNo = nChangePtnNum;
	}
	if (true == bEndChangePosture)
	{
		nEndChangePointNo = nChangePtnNum;
	}

	for (int i = 0; i < vtOutPoints.size() - 1; i++)
	{
		XI_POINT tEndp = vtOutPoints.at(i + 1);
		XI_POINT tStartp = vtOutPoints.at(i);
		double dLength = TwoPointDis(tEndp.x, tEndp.y, tEndp.z, tStartp.x, tStartp.y, tStartp.z);
		XI_POINT tDir = {
			(tEndp.x - tStartp.x) / dLength,
			(tEndp.y - tStartp.y) / dLength ,
			(tEndp.z - tStartp.z) / dLength };
		double dDireAngle = atan2(tDir.y, tDir.x) * 180 / 3.1415926;
		double dNormal = dDireAngle - 90 * m_nRobotInstallDir;

		if (true == bStartChangePosture && i < nChangePtnNum) // 起点处
		{
			dNormal += (double)(nChangePtnNum - i) * dChangeAngleUnit * m_nRobotInstallDir;
		}
		if (true == bEndChangePosture && i >= (vtOutPoints.size() - nChangePtnNum)) // 终点处
		{
			dNormal -= (double)(i - (vtOutPoints.size() - nChangePtnNum)) * dChangeAngleUnit * m_nRobotInstallDir;
		}

		XI_POINT tPosture = { m_dPlatWeldRx,m_dPlatWeldRy,dNormal };
		vtPosture.push_back(tPosture);
	}
	//添加左后一个点姿态
	vtPosture.push_back(vtPosture.at(vtPosture.size() - 1));
	if (vtPosture.size() != vtOutPoints.size())
	{
		XiMessageBox("点和姿态数量不匹配");
		return false;
	}
	dNormalAngle = atan2(dDiry, dDirx) * 180 / 3.1415926;
	return bIfSmoothSucess;
}
bool CDiaphragmWeld::FitCalLinePointsByMiddlePoints(XI_POINT tStartPoint, XI_POINT tEndPoint, vector<XI_POINT> vtScanPoints, vector<XI_POINT>& vtOutPoints, double dThresVal, double dStepDis)
{
	bool bIfSmoothSucess = false;
	if (vtScanPoints.size() < 2)
	{
		//已修改
		XUI::MesBox::PopInfo("FitLineDataX输入数据有误请检查：{0}", vtScanPoints.size());
		//XiMessageBox("FitLineDataX输入数据有误请检查：%d", vtScanPoints.size());
	}
	int nSize = vtScanPoints.size();
	XI_POINT tPoint, tStartProj, tEndProj;
	vector<XI_POINT> vtPoints;
	vector<XI_POINT> vtPointsAverage;
	vtPoints.clear();
	vtPointsAverage.clear();
	double dSelectedRatio = 0.7;
	T_LINE_PARA tLineParam;

	tLineParam = CalcLineParamRansac(vtScanPoints, dSelectedRatio);
	WriteLog("直线参数：%lf %lf %lf %lf %lf %lf", tLineParam.dPointX, tLineParam.dPointY, tLineParam.dPointZ,
		tLineParam.dDirX, tLineParam.dDirY, tLineParam.dDirZ);
	//求投影点
	PointtoLineProjection(tLineParam, tStartPoint, tStartProj);
	PointtoLineProjection(tLineParam, tEndPoint, tEndProj);

	double dTotalLen;
	dTotalLen = TwoPointDis(tStartProj.x, tStartProj.y, tStartProj.z, tEndProj.x, tEndProj.y, tEndProj.z);
	if (dTotalLen < 50.0)
	{
		//已修改
		XUI::MesBox::PopInfo("输入数据有误:{0:.3f}", dTotalLen);
		//XiMessageBox("输入数据有误:%.3lf", dTotalLen);
		return false;
	}

	double dDisStart1 = TwoPointDis(tStartPoint.x, tStartPoint.y, tStartPoint.z, tStartProj.x, tStartProj.y, tStartProj.z);;
	double dDisEnd1 = TwoPointDis(tEndPoint.x, tEndPoint.y, tEndPoint.z, tEndProj.x, tEndProj.y, tEndProj.z);

	WriteLog("投影点：%lf %lf %lf 距离：%lf %lf", tStartProj.x, tStartProj.y, tStartProj.z, dDisStart1, dDisEnd1);
	if (dDisStart1 > dThresVal || dDisEnd1 > dThresVal)
	{
		GetNaturalPop().PopOkCancel("投影点：{0} {1} {2} 距离：{3} {4}", tStartProj.x, tStartProj.y, tStartProj.z, dDisStart1, dDisEnd1);
		return bIfSmoothSucess;
	}
	else
	{
		bIfSmoothSucess = true;
	}

	double dDirx = (tEndProj.x - tStartProj.x) / dTotalLen;
	double dDiry = (tEndProj.y - tStartProj.y) / dTotalLen;
	double dDirz = (tEndProj.z - tStartProj.z) / dTotalLen;
	int nStepNum = int(dTotalLen / dStepDis) + 1;
	for (int n = 0; n < nStepNum; n++)
	{
		tPoint.x = tStartProj.x + n * dStepDis * dDirx;
		tPoint.y = tStartProj.y + n * dStepDis * dDiry;
		tPoint.z = tStartProj.z + n * dStepDis * dDirz;
		vtPointsAverage.push_back(tPoint);
	}
	XI_POINT dEd = { tEndProj.x,tEndProj.y,tEndProj.z };
	double dSmooth = TwoPointDis(vtPointsAverage[0].x, vtPointsAverage[0].y, vtPointsAverage[0].z, vtPointsAverage[vtPointsAverage.size() - 1].x, vtPointsAverage[vtPointsAverage.size() - 1].y, vtPointsAverage[vtPointsAverage.size() - 1].z);
	if (dSmooth < dTotalLen)
	{
		vtPointsAverage.push_back(dEd);
	}
	vtOutPoints.clear();
	vtOutPoints = vtPointsAverage;
	return true;
}

bool CDiaphragmWeld::FitCalLinePointsByMiddlePoints(XI_POINT tStartPoint, XI_POINT tEndPoint, vector<T_ALGORITHM_POINT> vtScanPoints, vector<XI_POINT>& vtOutPoints, double dThresVal, double dStepDis)
{
	if (vtScanPoints.size() < 2)
	{
		//已修改
		XUI::MesBox::PopInfo("FitLineDataX输入数据有误请检查：{0}", vtScanPoints.size());
		//XiMessageBox("FitLineDataX输入数据有误请检查：%d", vtScanPoints.size());
	}
	vector<XI_POINT> vtCinPoints;
	for (int n = 0; n < vtScanPoints.size(); n++)
	{
		XI_POINT tp = { vtScanPoints.at(n).dCoorX,vtScanPoints.at(n).dCoorY,vtScanPoints.at(n).dCoorZ };
		vtCinPoints.push_back(tp);
	}
	return FitCalLinePointsByMiddlePoints(tStartPoint, tEndPoint, vtCinPoints, vtOutPoints, dThresVal, dStepDis);
}


bool CDiaphragmWeld::CalcRobotCoorsAccordingTrack(vector<XI_POINT> vtCinPoints, vector<T_ROBOT_COORS>& vtCoutCoors)
{
	int nSize = vtCinPoints.size();
	if (nSize < 1)
	{
		return false;
	}
	for (int i = 0; i < vtCinPoints.size() - 1; i++)
	{
		XI_POINT tEndp = vtCinPoints.at(i + 1);
		XI_POINT tStartp = vtCinPoints.at(i);
		double dLength = TwoPointDis(tEndp.x, tEndp.y, tEndp.z, tStartp.x, tStartp.y, tStartp.z);
		XI_POINT tDir = {
			(tEndp.x - tStartp.x) / dLength,
			(tEndp.y - tStartp.y) / dLength ,
			(tEndp.z - tStartp.z) / dLength };
		double dDireAngle = atan2(tDir.y, tDir.x) * 180 / 3.1415926;
		double dNormal = m_pRobotDriver->DirAngleToRz(dDireAngle - 90 * m_nRobotInstallDir);

		T_ROBOT_COORS tPosture(vtCinPoints.at(i).x, vtCinPoints.at(i).y, vtCinPoints.at(i).z, m_dPlatWeldRx, m_dPlatWeldRy, dNormal, 0, 0, 0);
		vtCoutCoors.push_back(tPosture);
	}
	//添加左后一个点姿态
	T_ROBOT_COORS tPosture = vtCoutCoors.at(vtCoutCoors.size() - 1);
	tPosture.dX = vtCinPoints.at(nSize - 1).x;
	tPosture.dY = vtCinPoints.at(nSize - 1).y;
	tPosture.dZ = vtCinPoints.at(nSize - 1).z;
	vtCoutCoors.push_back(tPosture);
	return true;
}

bool CDiaphragmWeld::TransRobotPostureCoors(vector<XI_POINT> vtOutPoints, vector<XI_POINT>& vtPosture,
	int& nStartChangePointNo, int& nEndChangePointNo, double dRX, double dRY, double dDisPoint, bool bStartChangePosture, bool bEndChangePosture)
{

	//添加坐标姿态
	vtPosture.clear();
	int nDir = 1;//姿态变化方向
	bool bStartChangeAngle = true;//顺序输出
	double dChangeAngleTheshold = 40.0; // 多长距离变姿态
	int nChangePtnNum = dChangeAngleTheshold / dDisPoint; // 多少个点变完姿态
	double dChangeAngleUnit = 45.0 / (double)nChangePtnNum; // 相邻点间角度变化幅度
	if (true == bStartChangePosture)
	{
		nStartChangePointNo = nChangePtnNum;
	}
	if (true == bEndChangePosture)
	{
		nEndChangePointNo = nChangePtnNum;
	}

	for (int i = 0; i < vtOutPoints.size() - 1; i++)
	{
		XI_POINT tEndp = vtOutPoints.at(i + 1);
		XI_POINT tStartp = vtOutPoints.at(i);
		double dLength = TwoPointDis(tEndp.x, tEndp.y, tEndp.z, tStartp.x, tStartp.y, tStartp.z);
		XI_POINT tDir = {
			(tEndp.x - tStartp.x) / dLength,
			(tEndp.y - tStartp.y) / dLength ,
			(tEndp.z - tStartp.z) / dLength };
		double dDireAngle = atan2(tDir.y, tDir.x) * 180 / 3.1415926;
		double dNormal = dDireAngle - 90 * m_nRobotInstallDir;

		if (true == bStartChangePosture && i < nChangePtnNum) // 起点处
		{
			dNormal += (double)(nChangePtnNum - i) * dChangeAngleUnit * m_nRobotInstallDir;
		}
		if (true == bEndChangePosture && i >= (vtOutPoints.size() - nChangePtnNum)) // 终点处
		{
			dNormal -= (double)(i - (vtOutPoints.size() - nChangePtnNum)) * dChangeAngleUnit * m_nRobotInstallDir;
		}

		XI_POINT tPosture = { dRX,dRY,dNormal };
		vtPosture.push_back(tPosture);
	}
	//添加左后一个点姿态
	vtPosture.push_back(vtPosture.at(vtPosture.size() - 1));
	if (vtPosture.size() != vtOutPoints.size())
	{
		XiMessageBox("点和姿态数量不匹配");
		return false;
	}
	return true;
}

bool CDiaphragmWeld::FitCalLinePointsByMiddlePoints(XI_POINT tStartPoint, XI_POINT tEndPoint, vector<T_ALGORITHM_POINT> vtScanPoints, vector<XI_POINT>& vtOutPoints, vector<XI_POINT>& vtPosture,
	double& dNormalAngle, int& nStartChangePointNo, int& nEndChangePointNo, double dDisPoint, int nPointNum, bool bCheckDisORNum, int nThresVal, bool bStartChangePosture, bool bEndChangePosture)
{
	vector<XI_POINT> vtInPoints;
	for (int n = 0; n < vtScanPoints.size(); n++)
	{
		XI_POINT tALG = { vtScanPoints[n].dCoorX,vtScanPoints[n].dCoorY,vtScanPoints[n].dCoorZ };
		vtInPoints.push_back(tALG);
	}
	if (!FitCalLinePointsByMiddlePoints(tStartPoint, tEndPoint, vtInPoints, vtOutPoints, vtPosture, dNormalAngle, nStartChangePointNo, nEndChangePointNo, dDisPoint, nPointNum, bCheckDisORNum, nThresVal, bStartChangePosture, bEndChangePosture))
	{
		return false;
	}
	return true;
}

double CDiaphragmWeld::CalcDistancePointToLine(XI_POINT linePoint1, XI_POINT linePoint2, XI_POINT point)
{
	/*d = |(x2 - x1)(y - y1) - (y2 - y1)(x - x1) + (z2 - z1)(y1 - y)| / sqrt((x2 - x1)^2 + (y2 - y1)^2 + (z2 - z1)^2)*/
	double numerator = abs((linePoint2.x - linePoint1.x) * (point.y - linePoint1.y) - (linePoint2.y - linePoint1.y) * (point.x - linePoint1.x) + (linePoint2.z - linePoint1.z) * (linePoint1.y - point.y));
	double denominator = sqrt(pow(linePoint2.x - linePoint1.x, 2) + pow(linePoint2.y - linePoint1.y, 2) + pow(linePoint2.z - linePoint1.z, 2));
	double distance = numerator / denominator;

	return distance;
}

void CDiaphragmWeld::SaveDataAlgorithm(vector<T_ALGORITHM_POINT> vcPointOnTheSameLine, string str)
{
	ofstream CoutData(str.c_str());
	for (int nred = 0; nred < vcPointOnTheSameLine.size(); nred++)
	{
		if (nred < vcPointOnTheSameLine.size() - 1)
		{
			CoutData << vcPointOnTheSameLine[nred].dCoorX << "	" << vcPointOnTheSameLine[nred].dCoorY << "	" << vcPointOnTheSameLine[nred].dCoorZ << endl;
		}
		else
		{
			CoutData << vcPointOnTheSameLine[nred].dCoorX << "	" << vcPointOnTheSameLine[nred].dCoorY << "	" << vcPointOnTheSameLine[nred].dCoorZ;
		}
	}
	CoutData.close();
}
void CDiaphragmWeld::SaveDataXiPoint(vector<XI_POINT> vcPointOnTheSameLine, string str)
{
	ofstream CoutData(str.c_str());
	//CoutData.setf(ios::fixed,ios::floatfield);
	//CoutData.precision(2);
	for (int nred = 0; nred < vcPointOnTheSameLine.size(); nred++)
	{
		CoutData << vcPointOnTheSameLine[nred].x << "	" << vcPointOnTheSameLine[nred].y << "	" << vcPointOnTheSameLine[nred].z << endl;
	}
	CoutData.close();
}

void CDiaphragmWeld::SaveDataTrackFilt(vector<TrackFilter_XYZ> vtPoints, string str)
{
	ofstream CoutData(str.c_str());
	for (int nred = 0; nred < vtPoints.size(); nred++)
	{
		if (nred != vtPoints.size() - 1)
		{
			CoutData << vtPoints.at(nred).x_ << "	" << vtPoints.at(nred).y_ << "	" << vtPoints.at(nred).z_ << endl;
		}
		else
		{
			CoutData << vtPoints.at(nred).x_ << "	" << vtPoints.at(nred).y_ << "	" << vtPoints.at(nred).z_;
		}

	}
	CoutData.close();
}

void CDiaphragmWeld::SaveData(vector<XI_POINT> vcPointOnTheSameLine, string str)
{
	if (vcPointOnTheSameLine.size() < 1)
	{
		XUI::MesBox::PopError("SaveData函数存储数据有误：{0}", vcPointOnTheSameLine.size());
	}
	ofstream CoutData(str.c_str());
	for (int nred = 0; nred < vcPointOnTheSameLine.size(); nred++)
	{
		CoutData << vcPointOnTheSameLine[nred].x << "	" << vcPointOnTheSameLine[nred].y << "	" << vcPointOnTheSameLine[nred].z << endl;
	}
	CoutData.close();
}

void CDiaphragmWeld::SaveDataInt(vector<int> vnCoors, string str)
{
	if (vnCoors.size() < 1)
	{
		XUI::MesBox::PopError("SaveDataInt函数存储数据有误：{0}", vnCoors.size());
	}
	ofstream CoutData(str.c_str());
	for (int nred = 0; nred < vnCoors.size(); nred++)
	{
		CoutData << vnCoors.at(nred) << endl;
	}
	CoutData.close();
}

void CDiaphragmWeld::LoadDataInt(vector<int>& vnCoors, string str)
{
	vnCoors.clear();
	ifstream CoutData(str.c_str());
	int nNo;
	while (!CoutData.eof())
	{
		CoutData >> nNo;
		vnCoors.push_back(nNo);
	}
	CoutData.close();
	vnCoors.pop_back();
}
void CDiaphragmWeld::LoadTrackFilterCoors(vector<TrackFilter_XYZ>& vcPointOnTheSameLine, string str)
{
	if (!CheckFileExists(str.c_str()))
	{
		//已修改
		XUI::MesBox::PopInfo("{0}文件不存在", str.c_str());
		//XiMessageBox("%s 文件不存在", str.c_str());
	}
	vcPointOnTheSameLine.clear();
	TrackFilter_XYZ tPoint;
	ifstream CintData(str.c_str());
	while (!CintData.eof())
	{
		CintData >> tPoint.x_ >> tPoint.y_ >> tPoint.z_;
		vcPointOnTheSameLine.push_back(tPoint);
}
	CintData.close();
}


void CDiaphragmWeld::SaveTrackFilterCoors(vector<TrackFilter_XYZ> vcPointOnTheSameLine, string str)
{
	int nSize = vcPointOnTheSameLine.size();
	ofstream CoutData(str.c_str());
	for (int nred = 0; nred < nSize; nred++)
	{
		CoutData <<
			vcPointOnTheSameLine[nred].x_ << "	" <<
			vcPointOnTheSameLine[nred].y_ << "	" <<
			vcPointOnTheSameLine[nred].z_ << endl;
	}
	CoutData.close();
}
void CDiaphragmWeld::SaveDataRobotCoors(vector<T_ROBOT_COORS> vcPointOnTheSameLine, string str)
{
	int nSize = vcPointOnTheSameLine.size();
	ofstream CoutData(str.c_str());
	for (int nred = 0; nred < nSize; nred++)
	{
		if (nred == nSize - 1)
		{
			CoutData <<
				vcPointOnTheSameLine[nred].dX << "	" <<
				vcPointOnTheSameLine[nred].dY << "	" <<
				vcPointOnTheSameLine[nred].dZ << "	" <<
				vcPointOnTheSameLine[nred].dRX << "	" <<
				vcPointOnTheSameLine[nred].dRY << "	" <<
				vcPointOnTheSameLine[nred].dRZ << "	" <<
				vcPointOnTheSameLine[nred].dBX << "	" <<
				vcPointOnTheSameLine[nred].dBY << "	" <<
				vcPointOnTheSameLine[nred].dBZ;
			break;
		}
		CoutData <<
			vcPointOnTheSameLine[nred].dX << "	" <<
			vcPointOnTheSameLine[nred].dY << "	" <<
			vcPointOnTheSameLine[nred].dZ << "	" <<
			vcPointOnTheSameLine[nred].dRX << "	" <<
			vcPointOnTheSameLine[nred].dRY << "	" <<
			vcPointOnTheSameLine[nred].dRZ << "	" <<
			vcPointOnTheSameLine[nred].dBX << "	" <<
			vcPointOnTheSameLine[nred].dBY << "	" <<
			vcPointOnTheSameLine[nred].dBZ << endl;
	}
	CoutData.close();
}

void CDiaphragmWeld::SaveDataRobotPulse(T_ANGLE_PULSE tPulse, string str)
{
	ofstream CoutData(str.c_str());

	CoutData <<
		tPulse.nSPulse << "	" <<
		tPulse.nLPulse << "	" <<
		tPulse.nUPulse << "	" <<
		tPulse.nRPulse << "	" <<
		tPulse.nBPulse << "	" <<
		tPulse.nTPulse << "	" <<
		tPulse.lBXPulse << "	" <<
		tPulse.lBYPulse << "	" <<
		tPulse.lBZPulse;
	CoutData.close();
}
void CDiaphragmWeld::LoadDataRobotPulse(T_ANGLE_PULSE& tPulse, string str)
{
	ifstream CoutData(str.c_str());

	CoutData >>
		tPulse.nSPulse >>
		tPulse.nLPulse >>
		tPulse.nUPulse >>
		tPulse.nRPulse >>
		tPulse.nBPulse >>
		tPulse.nTPulse >>
		tPulse.lBXPulse >>
		tPulse.lBYPulse >>
		tPulse.lBZPulse;
	CoutData.close();
}

void CDiaphragmWeld::LoadDataRobotCoors(vector<T_ROBOT_COORS>& vcPointOnTheSameLine, string str)
{
	vcPointOnTheSameLine.clear();
	ifstream CoutData(str.c_str());
	T_ROBOT_COORS tCoor;
//	double dval;
	while (!CoutData.eof())
	{
		CoutData >>
			tCoor.dX >>
			tCoor.dY >>
			tCoor.dZ >>
			tCoor.dRX >>
			tCoor.dRY >>
			tCoor.dRZ >>
			tCoor.dBX >>
			tCoor.dBY >>
			tCoor.dBZ;/*>>
			dval;*/
		vcPointOnTheSameLine.push_back(tCoor);
	}
	CoutData.close();
}
void CDiaphragmWeld::LoadDataRobotCoors(vector<T_ROBOT_COORS> &vcPointOnTheSameLine, vector<int> &vnCoorsType, string str)
{
	vcPointOnTheSameLine.clear();
	vnCoorsType.clear();
	ifstream CoutData(str.c_str());
	T_ROBOT_COORS tCoor;
	int nType;
	while (!CoutData.eof())
	{
		CoutData >>
			tCoor.dX >>
			tCoor.dY >>
			tCoor.dZ >>
			tCoor.dRX >>
			tCoor.dRY >>
			tCoor.dRZ >>
			tCoor.dBX >>
			tCoor.dBY >>
			tCoor.dBZ>>
			nType;
		vcPointOnTheSameLine.push_back(tCoor);
		vnCoorsType.push_back(nType);
	}
	CoutData.close();
}
void CDiaphragmWeld::LoadDataXiPoint(vector<XI_POINT>& vcPointOnTheSameLine, string str)
{
	if (!CheckFileExists(str.c_str()))
	{
		//已修改
		XUI::MesBox::PopInfo("{0}文件不存在", str.c_str());
		//XiMessageBox("%s 文件不存在", str.c_str());
	}
	vcPointOnTheSameLine.clear();
	XI_POINT tPoint;
	ifstream CintData(str.c_str());
	while (!CintData.eof())
	{
		CintData >> tPoint.x >> tPoint.y >> tPoint.z;
		vcPointOnTheSameLine.push_back(tPoint);
	}
	CintData.close();
}
void CDiaphragmWeld::LoadDataAlgorithm(vector<T_ALGORITHM_POINT>& vcPointOnTheSameLine, string str)
{
	if (!CheckFileExists(str.c_str()))
	{
		//已修改
		XUI::MesBox::PopInfo("{0}文件不存在", str.c_str());
		//XiMessageBox("%s 文件不存在", str);
	}
	vcPointOnTheSameLine.clear();
	T_ALGORITHM_POINT tPoint;
	ifstream CintData(str.c_str());
	while (!CintData.eof())
	{
		CintData >> tPoint.dCoorX >> tPoint.dCoorY >> tPoint.dCoorZ;
		vcPointOnTheSameLine.push_back(tPoint);
	}
	CintData.close();
}
void CDiaphragmWeld::TransiformData(vector<XI_POINT> vcPointOnTheSameLine, vector<XI_POINT>& vtPoints)
{
	XI_POINT tPoints;
	vtPoints.clear();
	for (int nn = 0; nn < vcPointOnTheSameLine.size(); nn++)
	{
		tPoints.x = vcPointOnTheSameLine[nn].x;
		tPoints.y = vcPointOnTheSameLine[nn].y;
		tPoints.z = vcPointOnTheSameLine[nn].z;
		vtPoints.push_back(tPoints);
	}
}
void CDiaphragmWeld::TransiformMeasureData(vector<XI_POINT> vtPoints, vector<XI_POINT>& vcPointOnTheSameLine)
{
	XI_POINT tPoints;
	vcPointOnTheSameLine.clear();
	for (int nn = 0; nn < vtPoints.size(); nn++)
	{
		tPoints.x = vtPoints[nn].x;
		tPoints.y = vtPoints[nn].y;
		tPoints.z = vtPoints[nn].z;
		vcPointOnTheSameLine.push_back(tPoints);
	}
}
void CDiaphragmWeld::TransALGORITHMToXiPoint(vector<T_ALGORITHM_POINT> vtALGORITHMPoints, vector<XI_POINT>& vtPoints)
{
	XI_POINT tPoints;
	vtPoints.clear();
	for (int nn = 0; nn < vtALGORITHMPoints.size(); nn++)
	{
		tPoints.x = vtALGORITHMPoints[nn].dCoorX;
		tPoints.y = vtALGORITHMPoints[nn].dCoorY;
		tPoints.z = vtALGORITHMPoints[nn].dCoorZ;
		vtPoints.push_back(tPoints);
	}
}
void CDiaphragmWeld::TransXiPointToALGORITHM(vector<XI_POINT> vtPoints, vector<T_ALGORITHM_POINT>& vtALGORITHMPoints)
{
	T_ALGORITHM_POINT tPoints;
	vtALGORITHMPoints.clear();
	for (int nn = 0; nn < vtPoints.size(); nn++)
	{
		tPoints.dCoorX = vtPoints[nn].x;
		tPoints.dCoorY = vtPoints[nn].y;
		tPoints.dCoorZ = vtPoints[nn].z;
		vtALGORITHMPoints.push_back(tPoints);
	}
}
void CDiaphragmWeld::TransMeasureToAlgorithm(vector<XI_POINT> vcPointOnTheSameLine, vector<T_ALGORITHM_POINT>& vtALGORITHMPoints)
{
	T_ALGORITHM_POINT tALGORITHMPoints;
	vtALGORITHMPoints.clear();
	for (int nn = 0; nn < vcPointOnTheSameLine.size(); nn++)
	{
		tALGORITHMPoints.dCoorX = vcPointOnTheSameLine[nn].x;
		tALGORITHMPoints.dCoorY = vcPointOnTheSameLine[nn].y;
		tALGORITHMPoints.dCoorZ = vcPointOnTheSameLine[nn].z;
		vtALGORITHMPoints.push_back(tALGORITHMPoints);
	}
}
void CDiaphragmWeld::TransAlgorithmToMeasure(vector<T_ALGORITHM_POINT> vtALGORITHMPoints, vector<XI_POINT>& vcPointOnTheSameLine)
{
	XI_POINT tMEASUREPoints;
	vcPointOnTheSameLine.clear();
	for (int nn = 0; nn < vtALGORITHMPoints.size(); nn++)
	{
		tMEASUREPoints.x = vtALGORITHMPoints[nn].dCoorX;
		tMEASUREPoints.y = vtALGORITHMPoints[nn].dCoorY;
		tMEASUREPoints.z = vtALGORITHMPoints[nn].dCoorZ;
		vcPointOnTheSameLine.push_back(tMEASUREPoints);
	}
}

void CDiaphragmWeld::CalcMeasurePosInBase(vector<XI_POINT> tMeasurePoint, double dDis, double dZDis, double dAngleVer, vector<XI_POINT>& tMeasureTrans)
{
	XI_POINT tPoint;
	vector<XI_POINT> vtPoint;
	vtPoint.clear();
	double dAngle = dAngleVer;
	for (int n = 0; n < tMeasurePoint.size(); n++)
	{
		tPoint.x = tMeasurePoint[n].x + dDis * CosD(dAngle);
		tPoint.y = tMeasurePoint[n].y + dDis * SinD(dAngle);
		tPoint.z = tMeasurePoint[n].z + dZDis;
		vtPoint.push_back(tPoint);
	}
	tMeasureTrans.clear();
	tMeasureTrans = vtPoint;
}

void CDiaphragmWeld::CalcMeasurePosInBase(vector<XI_POINT> tMeasurePoint, double dDis, double dZDis, vector<XI_POINT> tPosture, vector<XI_POINT>& tMeasureTrans)
{
	XI_POINT tPoint;
	vector<XI_POINT> vtPoint;
	vtPoint.clear();
	double dAngle;
	for (int n = 0; n < tMeasurePoint.size(); n++)
	{
		dAngle = m_pRobotDriver->RzToDirAngle(tPosture.at(n).z);
		tPoint.x = tMeasurePoint[n].x + dDis * CosD(dAngle);
		tPoint.y = tMeasurePoint[n].y + dDis * SinD(dAngle);
		tPoint.z = tMeasurePoint[n].z + dZDis;
		vtPoint.push_back(tPoint);
	}
	tMeasureTrans.clear();
	tMeasureTrans = vtPoint;
}

void CDiaphragmWeld::CalcOffsetInBase(XI_POINT tMeasurePoint, double dDis, double dMoveZ, double dAngleVer, XI_POINT& tMeasureTrans, bool _isDown)
{
	if (!_isDown)
	{
		double dAngle = dAngleVer;
		tMeasureTrans.x = tMeasurePoint.x + dDis * CosD(dAngle);
		tMeasureTrans.y = tMeasurePoint.y + dDis * SinD(dAngle);
		tMeasureTrans.z = tMeasurePoint.z + dMoveZ;
	}
	else
	{
		double dAngle = dAngleVer;
		tMeasureTrans.x = tMeasurePoint.x + dDis * CosD(dAngle);
		tMeasureTrans.y = tMeasurePoint.y + dDis * SinD(dAngle);
		tMeasureTrans.z = tMeasurePoint.z - dMoveZ;
	}
}
void CDiaphragmWeld::CalcMeasureInBase(XI_POINT tMeasurePoint, double dDis, double dMoveZ, double dAngleVer, T_ROBOT_COORS& tMeasureTrans)
{
	XI_POINT tPoint;
	CalcOffsetInBase(tMeasurePoint, dDis, dMoveZ, dAngleVer - 90 * m_nRobotInstallDir, tPoint);
	tMeasureTrans.dX = tPoint.x;
	tMeasureTrans.dY = tPoint.y;
	tMeasureTrans.dZ = tPoint.z;
	tMeasureTrans.dRX = m_dPlatWeldRx;
	tMeasureTrans.dRY = m_dPlatWeldRy;
	tMeasureTrans.dRZ = m_pRobotDriver->DirAngleToRz(dAngleVer);
}

void CDiaphragmWeld::CalcMeasureInBase(T_ROBOT_COORS tMeasurePoint, double dDis, double dMoveZ, T_ROBOT_COORS& tMeasureTrans)
{
	tMeasureTrans = tMeasurePoint;
	tMeasureTrans.dX +=dDis*CosD(m_pRobotDriver->RzToDirAngle(tMeasureTrans.dRZ));
	tMeasureTrans.dBY +=dDis*SinD(m_pRobotDriver->RzToDirAngle(tMeasureTrans.dRZ));
	tMeasureTrans.dZ += dMoveZ;
}

double CDiaphragmWeld::CalcRobotAngleRZInBase(double dTrackVerDegree)
{
	//在机械臂坐标系下法相和机器人姿态的关系
	//法向角 = （RZ）
	double dRZangle = dTrackVerDegree;
	if (dRZangle > 180)
	{
		dRZangle -= 360;
	}
	else if (dRZangle < -180)
	{
		dRZangle += 360;
	}
	return dRZangle;
}
void CDiaphragmWeld::GetVerWeldLaserLineImageCoors(CvPoint& tCornerImagePoint, vector<CvPoint>& vtLeftImagePoints, vector<CvPoint>& vtRightImagePoints, int nImageType, int nCameraNo)
{
	vtLeftImagePoints.clear();
	vtRightImagePoints.clear();


	CvPoint cpGreenMidKeyPoint;
	CvPoint cpGreenBesideKeyPoint;

	vector<CvPoint> vCpRedLeftPoints;
	vector<CvPoint> vCpRedRightPoints;
	char str[20] = "SrcImage";
	char str1[20] = "Image";
	char ci[2];
	static int nLeftLineSrcIndex = 0;
	itoa(/*m_nLeftLineSrcIndex*/nLeftLineSrcIndex, ci, 10);
	strcat(str, ci);
	strcat(str1, ci);
	//m_pDHCameraVision->CamId2ImageProcess(eCamId)->m_nShowImage = 1;
	CDHGigeImageCapture* pDHGigeDrv = (CDHGigeImageCapture*)m_ptUnit->GetCameraCtrl(nCameraNo);
	IplImage* pImgBuffSrc = NULL;// = cvCloneImage(pDHGigeDrv->CaptureImage(false));
	if (!pDHGigeDrv->CaptureImage(pImgBuffSrc, 1))
	{
		pDHGigeDrv->ShowErrorString();
	}
	CImageProcess _ImgProcess = CImageProcess(pImgBuffSrc->width, pImgBuffSrc->height, m_pRobotDriver->m_nRobotNo, m_ptUnit->GetTrackTipDataFileName(nCameraNo));
	//cvCopyImage(m_pDHCameraVision->CamId2CamDrv(eCamId)->ImageCapture(), m_pDHCameraVision->CamId2ImageBuff(eCamId));
	
	Sleep(10);
	_ImgProcess.m_pXiCvObj->xiSaveImage(pImgBuffSrc, ".\\RightGun_RightCam\\", str, nLeftLineSrcIndex, 1);
	int nHight = pDHGigeDrv->m_tCameraPara.tDHCameraDriverPara.nMaxHeight;
	if (0 == nImageType)
	{
		_ImgProcess.GetBesideKeyPoints(pImgBuffSrc,
									   cpGreenMidKeyPoint,
									   cpGreenBesideKeyPoint,
									   vCpRedLeftPoints,
									   vCpRedRightPoints,
									   false,
									   500, 500,
									   300, 300,
									   20, 20);
	}
	else if (1 == nImageType)
	{
		cvFlip(pImgBuffSrc, pImgBuffSrc, 0);
		_ImgProcess.GetBesideKeyPoints(pImgBuffSrc, 
									   cpGreenMidKeyPoint, 
									   cpGreenBesideKeyPoint,
									   vCpRedLeftPoints, 
									   vCpRedRightPoints, 
									   false,
									   500, 500,
									   300, 300,
									   20, 20);

		for (int n = 0; n < vCpRedLeftPoints.size(); n++)
		{
			vCpRedLeftPoints.at(n).y = (nHight / 2 - vCpRedLeftPoints.at(n).y) * +vCpRedLeftPoints.at(n).y;
		}
		for (int n = 0; n < vCpRedRightPoints.size(); n++)
		{
			vCpRedRightPoints.at(n).y = (nHight / 2 - vCpRedRightPoints.at(n).y) * 2 + vCpRedRightPoints.at(n).y;
		}
		cpGreenMidKeyPoint.y = (nHight / 2 - cpGreenMidKeyPoint.y) * 2 + cpGreenMidKeyPoint.y;
	}
	else if (2 == nImageType)
	{
		_ImgProcess.GetBesideKeyPoints(pImgBuffSrc, 
									   cpGreenMidKeyPoint, 
									   cpGreenBesideKeyPoint,
									   vCpRedLeftPoints, 
									   vCpRedRightPoints, 
									   false,
									   500, 500,
									   300, 300,
									   20, 20);
	}


	if (cpGreenMidKeyPoint.x > 1 && cpGreenMidKeyPoint.x < 2591 && cpGreenMidKeyPoint.y>1 && cpGreenMidKeyPoint.y < 1993)
	{
		cvCvtColor(pImgBuffSrc, m_pScanInit->m_pShowImage, CV_GRAY2RGB);
		cvCircle(m_pScanInit->m_pShowImage, cpGreenMidKeyPoint, 8, CV_RGB(255, 0, 0), 4);
		cvCircle(m_pScanInit->m_pShowImage, cpGreenBesideKeyPoint, 8, CV_RGB(255, 0, 255), 4);
	}

	WriteLog("测量点：X；%d Y:%d", cpGreenMidKeyPoint.x, cpGreenMidKeyPoint.y);
	cvCircle(pImgBuffSrc, cpGreenMidKeyPoint, 3, CV_RGB(125, 0, 0), 3);

	_ImgProcess.m_pXiCvObj->xiSaveImage(pImgBuffSrc, ".\\LeftGun_LeftCam\\", str, nLeftLineSrcIndex, 1);
	cvReleaseImage(&pImgBuffSrc);
	tCornerImagePoint = cpGreenMidKeyPoint;
	SampleImagePoints(vCpRedLeftPoints, 20, vtLeftImagePoints);//////需要确认是否是立板上的图像点
	SampleImagePoints(vCpRedRightPoints, 20, vtRightImagePoints);//////需要确认是否是底板上的图像点	

}

void CDiaphragmWeld::SampleImagePoints(vector<CvPoint> vtImagePoints, int nSampleNum, vector<CvPoint>& vtSampleImagePoints)
{
	vtSampleImagePoints.clear();

	if (vtImagePoints.size() <= nSampleNum)
	{
		vtSampleImagePoints.insert(vtSampleImagePoints.end(), vtImagePoints.begin(), vtImagePoints.end());
	}
	else
	{
		int nDivid = int(vtImagePoints.size() / nSampleNum);
		for (int nPointNo = 0; nPointNo < nSampleNum; nPointNo++)
		{
			if (nPointNo * nDivid < vtImagePoints.size() - 1)
			{
				vtSampleImagePoints.push_back(vtImagePoints[nPointNo * nDivid]);
			}
		}
	}
}
void CDiaphragmWeld::MeasureVerLaserPoint(CRobotDriverAdaptor* pRobotDriver, T_ALGORITHM_POINT& tFirstPointOnLine, vector<T_ALGORITHM_POINT>& vcpUpPlatePoints,
	vector<T_ALGORITHM_POINT>& vcpDownPlatePoints, T_ANGLE_PULSE tRobotPulses, T_ROBOT_COORS tRobotCoord, int nImageType, int nCameraNo)
{
	vcpUpPlatePoints.clear();
	vcpDownPlatePoints.clear();

	CvPoint tRedCornerImagePoint;
	vector<CvPoint> vtRedUpPlateImagePoints;
	vector<CvPoint> vtRedDownPlateImagePoints;

	vtRedUpPlateImagePoints.clear();
	vtRedDownPlateImagePoints.clear();


	T_ALGORITHM_POINT tMeasurePoint;
	T_ABS_POS_IN_BASE tPointAbsLeftCamInBase;
	T_ABS_POS_IN_BASE tPointAbsRightCamInBase;
	GetVerWeldLaserLineImageCoors(tRedCornerImagePoint, vtRedUpPlateImagePoints, vtRedDownPlateImagePoints, nImageType/*, eCamId*/, nCameraNo);
	//m_pDHCameraVision->GetMeasurePosInBaseNew(pRobotDriver->m_strRobotName,
	//										  tRedCornerImagePoint,
	//										  /*eCamId,*/ 
	//										  tPointAbsLeftCamInBase,
	//										  tRobotCoord,
	//										  tRobotPulses,
	//										  pRobotDriver->m_eManipulatorType);
	m_ptUnit->TranImageToBase(nCameraNo,
							  tRedCornerImagePoint,
							  tRobotCoord,
							  tRobotPulses,
							  &tPointAbsLeftCamInBase);

	tFirstPointOnLine.dCoorX = tPointAbsLeftCamInBase.tWeldLinePos.x;
	tFirstPointOnLine.dCoorY = tPointAbsLeftCamInBase.tWeldLinePos.y;
	tFirstPointOnLine.dCoorZ = tPointAbsLeftCamInBase.tWeldLinePos.z;
	int nPointNo = 0;
	for (nPointNo = 0; nPointNo < vtRedUpPlateImagePoints.size(); nPointNo++)
	{
		//m_pDHCameraVision->GetMeasurePosInBaseNew(pRobotDriver->m_strRobotName,
		//										  vtRedUpPlateImagePoints[nPointNo],
		//										  /*eCamId,*/ 
		//										  tPointAbsLeftCamInBase,
		//										  tRobotCoord,
		//										  tRobotPulses,
		//										  pRobotDriver->m_eManipulatorType);
		m_ptUnit->TranImageToBase(nCameraNo,
			vtRedUpPlateImagePoints[nPointNo],
			tRobotCoord,
			tRobotPulses,
			&tPointAbsLeftCamInBase);

		tMeasurePoint.dCoorX = tPointAbsLeftCamInBase.tWeldLinePos.x;
		tMeasurePoint.dCoorY = tPointAbsLeftCamInBase.tWeldLinePos.y;
		tMeasurePoint.dCoorZ = tPointAbsLeftCamInBase.tWeldLinePos.z;

		vcpUpPlatePoints.push_back(tMeasurePoint);
	}

	for (nPointNo = 0; nPointNo < vtRedDownPlateImagePoints.size(); nPointNo++)
	{
		//m_pDHCameraVision->GetMeasurePosInBaseNew(pRobotDriver->m_strRobotName,
		//										  vtRedDownPlateImagePoints[nPointNo],
		//										  /*eCamId,*/  
		//										  tPointAbsRightCamInBase,
		//										  tRobotCoord,
		//										  tRobotPulses,
		//										  pRobotDriver->m_eManipulatorType);

		m_ptUnit->TranImageToBase(nCameraNo,
								  vtRedDownPlateImagePoints[nPointNo],
								  tRobotCoord,
								  tRobotPulses,
								  &tPointAbsRightCamInBase);

		tMeasurePoint.dCoorX = tPointAbsRightCamInBase.tWeldLinePos.x;
		tMeasurePoint.dCoorY = tPointAbsRightCamInBase.tWeldLinePos.y;
		tMeasurePoint.dCoorZ = tPointAbsRightCamInBase.tWeldLinePos.z;

		vcpDownPlatePoints.push_back(tMeasurePoint);
	}
}
void CDiaphragmWeld::MeasurePointLaserData(CRobotDriverAdaptor* pRobotDriver, CString str, T_ALGORITHM_POINT& vcPointOnTheSameLine, vector<T_ALGORITHM_POINT>& vcpUpPlatePoints,
	vector<T_ALGORITHM_POINT>& vcpDownPlatePoints, T_ANGLE_PULSE tRobotPulses, T_ROBOT_COORS tRobotCoord, int nImageType, int nCameraNo)
{
	T_ALGORITHM_POINT tFirstPointOnLineRed;
	//T_ALGORITHM_POINT tSecondPointOnLineRed;
	vector<T_ALGORITHM_POINT> vcpUpPlatePointsRed;
	vector<T_ALGORITHM_POINT> vcpDownPlatePointsRed;
	//vcPointOnTheSameLine.clear();
	int nred = 0;
	CString Cout1 = "WeldData\\CoutRedUp";
	CString Cout2 = "WeldData\\CoutRedDown";
	CString Cout5 = "WeldData\\CoutPoint";
	Cout1 += str + ".txt";
	Cout2 += str + ".txt";
	Cout5 += str + ".txt";
	FILE* CoutRed = fopen(Cout1, "w");
	FILE* CoutRedDown = fopen(Cout2, "w");
	FILE* CoutPoint = fopen(Cout5, "w");


	vcpUpPlatePointsRed.clear();
	vcpDownPlatePointsRed.clear();
	MeasureVerLaserPoint(pRobotDriver, tFirstPointOnLineRed, vcpUpPlatePointsRed, vcpDownPlatePointsRed, tRobotPulses, tRobotCoord, nImageType,nCameraNo/*, eCamId*/);

	tFirstPointOnLineRed.dCoorY += tRobotCoord.dBY;
	vcPointOnTheSameLine = tFirstPointOnLineRed;
	for (nred = 0; nred < vcpUpPlatePointsRed.size(); nred++)
	{
		vcpUpPlatePointsRed[nred].dCoorY += tRobotCoord.dBY;
		fprintf(CoutRed, "%lf %lf %lf\n",
			vcpUpPlatePointsRed[nred].dCoorX,
			vcpUpPlatePointsRed[nred].dCoorY,
			vcpUpPlatePointsRed[nred].dCoorZ);
		vcpUpPlatePoints.push_back(vcpUpPlatePointsRed[nred]);
	}
	for (nred = 0; nred < vcpDownPlatePointsRed.size(); nred++)
	{
		vcpDownPlatePointsRed[nred].dCoorY += tRobotCoord.dBY;
		fprintf(CoutRedDown, "%lf %lf %lf\n",
			vcpDownPlatePointsRed[nred].dCoorX,
			vcpDownPlatePointsRed[nred].dCoorY,
			vcpDownPlatePointsRed[nred].dCoorZ);
		vcpDownPlatePoints.push_back(vcpDownPlatePointsRed[nred]);
	}
	fclose(CoutRed);
	fclose(CoutRedDown);
	fclose(CoutPoint);
}

vector<T_ROBOT_COORS> CDiaphragmWeld::GetRealWeldData(CRobotDriverAdaptor* pRobotDriver, vector<XI_POINT> tWeldPointData, vector<XI_POINT> tWeldPosture)
{
	vector<T_ROBOT_COORS> vtWeldCoors;
	int nWeldPointNum = tWeldPointData.size();
	int nWeldPostureNum = tWeldPosture.size();
	if (nWeldPointNum < 1)
	{
		XiMessageBox("数据有误");
		return vtWeldCoors;
	}
	//将点和姿态转合并
	bool bRobotWeldDir = false;
	m_bRobotWeldDir = false;
	for (size_t i = 0; i < nWeldPointNum; i++)
	{
		T_ROBOT_COORS tCoutRobotCoor;
		T_ROBOT_COORS WorldCoors(
			tWeldPointData.at(i).x,
			tWeldPointData.at(i).y,
			tWeldPointData.at(i).z,
			tWeldPosture.at(i).x,
			tWeldPosture.at(i).y,
			tWeldPosture.at(i).z,
			0.0, 0.0, 0.0
		);
		CalcRobotAndMachinePos(pRobotDriver, WorldCoors, tCoutRobotCoor);
		vtWeldCoors.push_back(tCoutRobotCoor);

	}

	return vtWeldCoors;
}


vector<T_ROBOT_COORS> CDiaphragmWeld::GetRealWeldData(CRobotDriverAdaptor* pRobotDriver, vector<T_ROBOT_COORS> tWeldPointData)
{
	vector<T_ROBOT_COORS> vtWeldCoors;
	int nWeldPointNum = tWeldPointData.size();
	if (nWeldPointNum < 1)
	{
		XiMessageBox("数据有误");
		return vtWeldCoors;
	}
	//将点和姿态转合并
	bool bRobotWeldDir = false;
	m_bRobotWeldDir = false;
	for (size_t i = 0; i < nWeldPointNum; i++)
	{
		T_ROBOT_COORS tCoutRobotCoor;
		CalcRobotAndMachinePos(pRobotDriver, tWeldPointData.at(i), tCoutRobotCoor);
		vtWeldCoors.push_back(tCoutRobotCoor);

	}

	return vtWeldCoors;
}


bool CDiaphragmWeld::WeldProcessBefor(CRobotDriverAdaptor* pRobotDriver, vector<T_ROBOT_COORS> vtRealWeldCoors, vector<int> vtRealWeldCoorType, vector<T_ANGLE_PULSE> vtRealWeldPulse, bool bIsTracking)
{
	long long tTime = XI_clock();
	if (vtRealWeldCoorType.size() != vtRealWeldCoors.size())
	{
		XiMessageBox("轨迹数量和轨迹属性数量不相同");
		return false;
	}
	int nWeldPointNum = vtRealWeldCoors.size();
	if (nWeldPointNum < 1)
	{
		XiMessageBox("WeldProcessBefor输入数据有误");
		return false;
	}
	int nWeldStepNo = 0;
	double dAbjustRz = 0.0;

	m_pTraceModel->m_dRecordBreakPointRZ = 0;
	m_pTraceModel->m_nWeldStepNo = 0;
	T_ANGLE_PULSE tRefPulse = GetCurrentPulse(pRobotDriver), tResultPulse, tResultPulse2;
	double dOverLength = m_pTraceModel->m_dWeldLen;
	if (m_ptUnit->m_bBreakPointContinue)
	{
		//获取焊接轨迹，获取焊接长度，步号，rz调整角度     
		LoadWeldLengthData(pRobotDriver, m_pTraceModel->m_dCurrentWeldLen, dAbjustRz, nWeldStepNo, dOverLength);
		dAbjustRz = 0;
		//dAbjustRz += m_dWeldDipAngle;
		//m_pTraceModel->m_dRecordBreakPointRZ = dAbjustRz;//用于Rz调整
		if (nWeldStepNo > 10)//续焊时回退两步，填弧坑
		{
			nWeldStepNo -= 3;
		}
		if (nWeldPointNum - nWeldStepNo < 30 && nWeldStepNo>30)//防止断点续焊时开始跟踪和原始数据间隔较远
		{
			nWeldStepNo = nWeldStepNo - (30 - (nWeldPointNum - nWeldStepNo));
		}
		//if (!bIsTracking)
		{
			m_pTraceModel->m_nWeldStepNo = nWeldStepNo;//用于先测后焊记录步号
		}
		//加载参考位置
		CString strName;
		//strName.Format(".\\WeldData\\%s\\BreakPointReferencePulse.txt", pRobotDriver->m_strRobotName);
		//LoadDataRobotPulse(tRefPulse, strName.GetBuffer(0));
	}
	double dCoor[6] = { 0 };
	dCoor[5] = dAbjustRz;
	pRobotDriver->SetPosVar(99, dCoor, 0);
	
	//起始结束安全位置
	T_ROBOT_COORS tStartSafe = vtRealWeldCoors.at(nWeldStepNo);
	T_ANGLE_PULSE tStartSafePulse = vtRealWeldPulse.at(nWeldStepNo);
	if (!m_ptUnit->m_bBreakPointContinue)
	{
		//去掉起始安全位置
		vtRealWeldCoors.erase(vtRealWeldCoors.begin());
		vtRealWeldPulse.erase(vtRealWeldPulse.begin());
		vtRealWeldCoorType.erase(vtRealWeldCoorType.begin());
		//去掉起始安全位置
		vtRealWeldCoors.pop_back();
		vtRealWeldPulse.pop_back();
		vtRealWeldCoorType.pop_back();
	}
	else
	{
		double dAdjustDis = 50.;
		// 添加收下枪安全位置
		T_ROBOT_COORS tCoorSafe = vtRealWeldCoors.at(nWeldStepNo);
		tCoorSafe.dX = tCoorSafe.dX + dAdjustDis * CosD(m_pRobotDriver->RzToDirAngle(tCoorSafe.dRZ + dAbjustRz));
		tCoorSafe.dY = tCoorSafe.dY + dAdjustDis * SinD(m_pRobotDriver->RzToDirAngle(tCoorSafe.dRZ + dAbjustRz));
		tCoorSafe.dZ = m_dWorkPieceHighTop + 20 * m_nRobotInstallDir;
		tStartSafe = tCoorSafe;
		if (!pRobotDriver->RobotInverseKinematics(tStartSafe, tRefPulse, pRobotDriver->m_tTools.tGunTool, tStartSafePulse))
		{
			return false;
		}
	}	
	nWeldPointNum = vtRealWeldCoors.size();

	
	// 初始化焊接全局变量
	vector<int> vnChangePostureStep; //记录初始变姿态轨迹
	vector<T_ROBOT_COORS> vnChangePostureCoor;
	m_pTraceModel->m_vtWeldLinePointType.clear();
	pRobotDriver->m_vtWeldLineInWorldPoints.clear();//焊接轨迹
	m_pTraceModel->m_vtBreakContinueBeforPoints.clear();//断续前数据
	m_pTraceModel->m_vtBreakContinueBeforPointType.clear();
	// 记录断点前数据
	m_pTraceModel->m_vtBreakContinueBeforPoints.insert(m_pTraceModel->m_vtBreakContinueBeforPoints.end(),
		vtRealWeldCoors.begin(), vtRealWeldCoors.begin() + nWeldStepNo);
	m_pTraceModel->m_vtBreakContinueBeforPointType.insert(m_pTraceModel->m_vtBreakContinueBeforPointType.end(),
		vtRealWeldCoorType.begin(), vtRealWeldCoorType.begin() + nWeldStepNo);
	// 截取初始段数据
	for (size_t i = nWeldStepNo; i < nWeldPointNum; i++)
	{
		T_ROBOT_COORS tPoint = vtRealWeldCoors.at(i);
		if (!m_ptUnit->m_bBreakPointContinue){
			tPoint.dRZ += m_dWeldDipAngle;
		}	

		double dDis = TwoPointDis(
			vtRealWeldCoors.at(nWeldStepNo).dX + vtRealWeldCoors.at(nWeldStepNo).dBX,
			vtRealWeldCoors.at(nWeldStepNo).dY + vtRealWeldCoors.at(nWeldStepNo).dBY,
			vtRealWeldCoors.at(nWeldStepNo).dZ,
			vtRealWeldCoors.at(i).dX + vtRealWeldCoors.at(i).dBX,
			vtRealWeldCoors.at(i).dY + vtRealWeldCoors.at(i).dBY,
			vtRealWeldCoors.at(i).dZ);
		
		if (E_WELD_TRACK_CHANGE_POSTURE & vtRealWeldCoorType.at(i))
		{
			vnChangePostureStep.push_back(vtRealWeldCoorType.at(i));
			vnChangePostureCoor.push_back(tPoint);
		}
		if (bIsTracking && 
			/*m_dHandEyeDis + vnChangePostureCoor.size() * m_pTraceModel->m_dMoveStepDis < dDis &&*/
			m_dTrackCamHandEyeDis + vnChangePostureCoor.size() * m_pTraceModel->m_dMoveStepDis < dDis &&
			!m_ptUnit->m_bBreakPointContinue|| E_TRANSITION_POINT & vtRealWeldCoorType.at(i))
		{
			pRobotDriver->m_cLog->Write("传入数据：%d,距离：%.3lf", i, dDis);
			break;
		}

		pRobotDriver->m_vtWeldLineInWorldPoints.push_back(tPoint);
		m_pTraceModel->m_vtWeldLinePointType.push_back(vtRealWeldCoorType.at(i));
	}
	// 转姿态步数
	m_pTraceModel->m_nOpenTrackingPos = vnChangePostureCoor.size();
	// 剔除跟踪轨迹以外的点用于起始段数据初始化
	double dbReakLength = 0.0;
	int nTrackNo = 0;
	vector<T_ROBOT_COORS> vTrackCoors;	
	for (nTrackNo = 0; nTrackNo < pRobotDriver->m_vtWeldLineInWorldPoints.size(); nTrackNo++)
	{
		if (E_WELD_TRACK & m_pTraceModel->m_vtWeldLinePointType.at(nTrackNo)){
			vTrackCoors.push_back(pRobotDriver->m_vtWeldLineInWorldPoints.at(nTrackNo));
		}
		else{
			vTrackCoors.clear();
		}
		
	}
	if (m_ptUnit->m_bBreakPointContinue)
	{
		for (nTrackNo = 0; nTrackNo < pRobotDriver->m_vtWeldLineInWorldPoints.size() - 1; nTrackNo++)
		{
			// 计算续焊时初始段长度
			double dDis = TwoPointDis(pRobotDriver->m_vtWeldLineInWorldPoints[nTrackNo].dX, pRobotDriver->m_vtWeldLineInWorldPoints[nTrackNo].dBY, pRobotDriver->m_vtWeldLineInWorldPoints[nTrackNo].dZ,
				pRobotDriver->m_vtWeldLineInWorldPoints[nTrackNo + 1].dX, pRobotDriver->m_vtWeldLineInWorldPoints[nTrackNo + 1].dBY, pRobotDriver->m_vtWeldLineInWorldPoints[nTrackNo + 1].dZ
			);
			dbReakLength += dDis;

		}
		CString strPath;
		strPath.Format("%s%s%s", OUTPUT_PATH, m_ptUnit->GetRobotCtrl()->m_strRobotName, RECOGNITION_FOLDER);
		//加载缓存位置
		XI_POINT tPoint;
		vector<XI_POINT> vtPoint;
		CString strFile;
		strFile.Format("%s\\滤波缓存数据.txt", strPath);
		std::ifstream cin(strFile);
		while (!cin.eof())
		{
			cin >> tPoint.x >> tPoint.y >> tPoint.z;
			vtPoint.push_back(tPoint);
		}
		cin.close();

		for (nTrackNo = 0; nTrackNo < vtPoint.size() - 1; nTrackNo++)
		{
			double dDis = TwoPointDis(vtPoint[nTrackNo].x, vtPoint[nTrackNo].y, vtPoint[nTrackNo].z,
				vtPoint[nTrackNo + 1].x, vtPoint[nTrackNo + 1].y, vtPoint[nTrackNo + 1].z
			);
			dbReakLength += dDis;
		}
	}
	
	

	pRobotDriver->m_vtWeldLineInWorldPoints.erase(pRobotDriver->m_vtWeldLineInWorldPoints.end() - vTrackCoors.size(), pRobotDriver->m_vtWeldLineInWorldPoints.end());
	m_pTraceModel->m_vtWeldLinePointType.erase(m_pTraceModel->m_vtWeldLinePointType.end() - vTrackCoors.size(), m_pTraceModel->m_vtWeldLinePointType.end());
	//初始段数据处理
	if (vTrackCoors.size() > 0)
	{
		if (!m_pScanInit->StartDataPointPro(pRobotDriver, vTrackCoors, bIsTracking, dOverLength+ dbReakLength)){
			return false;
		}
		for (nTrackNo = 0; nTrackNo < vTrackCoors.size(); nTrackNo++){
			if (nTrackNo< vnChangePostureStep.size())
			{
				vTrackCoors.at(nTrackNo).dRZ = vnChangePostureCoor[nTrackNo].dRZ;
				pRobotDriver->m_vtWeldLineInWorldPoints.push_back(vTrackCoors.at(nTrackNo));
				m_pTraceModel->m_vtWeldLinePointType.push_back(vnChangePostureStep[nTrackNo]);
				continue;
			}
			pRobotDriver->m_vtWeldLineInWorldPoints.push_back(vTrackCoors.at(nTrackNo));
			m_pTraceModel->m_vtWeldLinePointType.push_back(E_WELD_TRACK);
		}
	}
	// 移动外部轴到起始安全位置
	int nAxisNo = m_ptUnit->m_nTrackAxisNo;

	double dMoveVal = 0.0;
	dMoveVal = 1 == nAxisNo ? pRobotDriver->m_vtWeldLineInWorldPoints[0].dBX : dMoveVal;
	dMoveVal = 2 == nAxisNo ? pRobotDriver->m_vtWeldLineInWorldPoints[0].dBY : dMoveVal;
	dMoveVal = 3 == nAxisNo ? pRobotDriver->m_vtWeldLineInWorldPoints[0].dBZ : dMoveVal;
	if (0 != m_ptUnit->MoveExAxisFun(dMoveVal, 9000, nAxisNo))return false;
	m_ptUnit->WorldCheckRobotDone();

	//精准对枪前安全位置
	T_ROBOT_MOVE_SPEED tPulseMove(m_dSafePosRunSpeed, 50, 50);
	vector<T_ROBOT_MOVE_INFO> vtRobotMoveInfo(0);
	vtRobotMoveInfo.push_back(pRobotDriver->PVarToRobotMoveInfo(0, tStartSafePulse, tPulseMove, MOVJ));

	// 干涉端点锁定
	vector<T_ROBOT_COORS> tCoutCoors;
	WeldLockBefor(pRobotDriver, pRobotDriver->m_vtWeldLineInWorldPoints, tCoutCoors,true);
	tPulseMove.dSpeed = m_dSafeApproachToWorkSpeed;
	for (size_t i = 0; i < tCoutCoors.size(); i++)
	{
		vtRobotMoveInfo.push_back(pRobotDriver->PVarToRobotMoveInfo(0, tCoutCoors[i], tPulseMove, MOVL));
		pRobotDriver->SetMoveValue(vtRobotMoveInfo, false);	
	}
	if (tCoutCoors.size() > 0) // 姿态恢复位置：起弧前锁定
	{
		pRobotDriver->CallJob("CONTIMOVANY");
		CHECK_BOOL_RETURN(m_ptUnit->CheckRobotDone(tCoutCoors[tCoutCoors.size()-1]));
		//起弧前锁定
		if (!g_bLocalDebugMark) {
			CHECK_BOOL_RETURN(m_pScanInit->RealTimeTrackLockArcBefor(pRobotDriver, m_ptUnit->m_bBreakPointContinue));
		}
		vtRobotMoveInfo.clear();
		vtRobotMoveInfo.push_back(pRobotDriver->PVarToRobotMoveInfo(0, tCoutCoors[0], tPulseMove, MOVL));
	}
	
	//焊接开始位置
	T_ROBOT_COORS tPrecisionGunPos = pRobotDriver->m_vtWeldLineInWorldPoints.at(0);
	tPrecisionGunPos.dRZ += dAbjustRz;
	/*tPrecisionGunPos.dRX = m_dPlatWeldRx;
	tPrecisionGunPos.dRY = m_dPlatWeldRy;*/
	tPulseMove.dSpeed = m_dSafeApproachToWorkSpeed;
	vtRobotMoveInfo.push_back(pRobotDriver->PVarToRobotMoveInfo(0, tPrecisionGunPos, tPulseMove, MOVL));
	pRobotDriver->SetMoveValue(vtRobotMoveInfo, false);
	//XiMessageBox("开始运动");
	pRobotDriver->CallJob("CONTIMOVANY");
	CHECK_BOOL_RETURN(m_ptUnit->CheckRobotDone(tPrecisionGunPos));
	if (0 == tCoutCoors.size()) // 焊接起点位置：起弧前锁定
	{
		if (!g_bLocalDebugMark){
			CHECK_BOOL_RETURN(m_pScanInit->RealTimeTrackLockArcBefor(pRobotDriver,m_ptUnit->m_bBreakPointContinue));
		}
	}

	if (!m_ptUnit->m_bBreakPointContinue)
	{
		if (E_CLOSEDARC == m_eWorkPieceType)
		{
			m_pTraceModel->m_tRealEndpointCoor = tPrecisionGunPos;//闭合圆弧起点也是终点
			SaveEndpointData(pRobotDriver, m_pTraceModel->m_tRealEndpointCoor, false);//终点
		}
		else if (!bIsTracking)
		{
			m_pTraceModel->m_tRealEndpointCoor = vtRealWeldCoors.at(nWeldPointNum - 1);
			SaveEndpointData(pRobotDriver, m_pTraceModel->m_tRealEndpointCoor, false);//终点
		}
	}
	else
	{
		LoadEndpointData(pRobotDriver, m_pTraceModel->m_tRealEndpointCoor, false);
	}
	RecordRunTime(pRobotDriver, tTime, "精准对枪加初始数据处理");
	return true;
}
bool CDiaphragmWeld::GetTruncatedData(CRobotDriverAdaptor* pRobotDriver, T_ROBOT_COORS tRobotRealPos, MoveHandle_Pnt& tRobotCoor, long& lRobotVel, long& lMachinePos, long& lMachineVel)
{	
	// 当外部轴为x轴时为兼容算法内部以Y轴参数Y轴为外部轴将坐标系根据原有安装方向旋转90度（正转和负转需要注意）
	XI_POINT tTempWeldTrackCoor;
#ifdef SINGLE_ROBOT
	tTempWeldTrackCoor.x = tRobotRealPos.dX;
	tTempWeldTrackCoor.y = tRobotRealPos.dBY;
	tTempWeldTrackCoor.z = tRobotRealPos.dZ;
#else
	tTempWeldTrackCoor.x = tRobotRealPos.dBX;
	tTempWeldTrackCoor.y = tRobotRealPos.dY;
	tTempWeldTrackCoor.z = tRobotRealPos.dZ;
#endif

	RobotAxisToAlgoAxis(tTempWeldTrackCoor.x, tTempWeldTrackCoor.y, tTempWeldTrackCoor.z);
	bool bRtn = MoveHandle_SetNextPoint(m_ptUnit->m_nRobotSmooth, tTempWeldTrackCoor.x, tTempWeldTrackCoor.y, tTempWeldTrackCoor.z);
	//! 消除截断误差的 下一点，单位：( mm * 1000)
	tRobotCoor = MoveHandle_GetNextPoint(m_ptUnit->m_nRobotSmooth);
	//! 当前点到下一点的机器人速度，单位：(mm/s * 10)
	lRobotVel = MoveHandle_GetToNextPointRobotVel(m_ptUnit->m_nRobotSmooth);
	pRobotDriver->m_cLog->Write("机器人数据：速度 %ld 位移：X:%ld Z:%ld", lRobotVel, tRobotCoor.x_, tRobotCoor.z_);
	//! 获取大车需走的脉冲个数 pulse
	lMachinePos = MoveHandle_GetYAxisPulseSize(m_ptUnit->m_nRobotSmooth);
	//! 获取大车 消除截断误差 的速度，单位：脉冲每秒 (pulse/s)
	lMachineVel = MoveHandle_GetYAxisVelocity(m_ptUnit->m_nRobotSmooth);

	//--------------------------------------------------------------
	AlgoAxisToRobotAxis(tRobotCoor.x_, tRobotCoor.y_, tRobotCoor.z_);
	//---------------------------------------------------------------
	pRobotDriver->m_cLog->Write(" 大车数据：位移：%ld 速度 %ld  bRtn:%d", lMachinePos, lMachineVel, bRtn);
	return bRtn;
}

void CDiaphragmWeld::SwitchWeldSpeed(CRobotDriverAdaptor* pRobotDriver, int nCoorIndex,int nCoorType, double dWeldSpeed, double dWarpSpeed, long &lRobotvel, bool& bIsChangeSpeed)
{
	if (E_WELD_WRAP == nCoorType && !bIsChangeSpeed)
	{
		bIsChangeSpeed = true;
		if ((pRobotDriver->m_nExternalAxleType >> 3) & 0x01 || (pRobotDriver->m_nExternalAxleType >> 4) & 0x01 || (pRobotDriver->m_nExternalAxleType >> 5) & 0x01)
		{
			lRobotvel = dWarpSpeed / 6;
		}
		else
		{
			double dPulseToDisX = m_ptUnit->GetExPulseEquivalent(m_ptUnit->m_nTrackAxisNo);
			MoveHandle_ChangeParameters(m_ptUnit->m_nRobotSmooth, dWarpSpeed, &dPulseToDisX);
		}

		pRobotDriver->m_cLog->Write("切换包角速度速度：%d %.3lf", nCoorIndex, dWarpSpeed);
	}
	else if (E_WELD_TRACK == nCoorType && bIsChangeSpeed)
	{
		bIsChangeSpeed = false;
		if ((pRobotDriver->m_nExternalAxleType >> 3) & 0x01 || (pRobotDriver->m_nExternalAxleType >> 4) & 0x01 || (pRobotDriver->m_nExternalAxleType >> 5) & 0x01)
		{
			lRobotvel = dWeldSpeed / 6;
		}
		else
		{
			double dPulseToDisX = m_ptUnit->GetExPulseEquivalent(m_ptUnit->m_nTrackAxisNo);
			MoveHandle_ChangeParameters(m_ptUnit->m_nRobotSmooth, dWeldSpeed, &dPulseToDisX);
		}
		pRobotDriver->m_cLog->Write("切换焊接速度：%d %.3lf", nCoorIndex, dWeldSpeed);
	}
}

void CDiaphragmWeld::SwitchWeldCurrent(CRobotDriverAdaptor* pRobotDriver, int nCoorIndex, int nCoorType, T_WELD_PARA tWeldParam,bool& bChangeVoltage)
{
	if (E_WELD_WRAP_CHANGE == nCoorType)
	{
		// 切换焊接速度和电流电压
		pRobotDriver->m_cLog->Write("切换焊接电流电压和速度：%.3lf %.3lf %d", 
			tWeldParam.dTrackCurrent, tWeldParam.dTrackVoltage, nCoorIndex);
		vector<MP_USR_VAR_INFO> vtCurVarInfo;
		vtCurVarInfo.push_back(pRobotDriver->PrepareValData(4, tWeldParam.dWrapCurrentt2));
		vtCurVarInfo.push_back(pRobotDriver->PrepareValData(5, tWeldParam.dWrapVoltage2 * 10));
		vtCurVarInfo.push_back(pRobotDriver->PrepareValData(3, 1));
		pRobotDriver->SetMultiVar_H(vtCurVarInfo);
		Sleep(50);
	}
	else if (E_WELD_WRAP == nCoorType && !bChangeVoltage)
	{
		// 切换包角速度和电流电压
		bChangeVoltage = true;
		pRobotDriver->m_cLog->Write("切换包角电流电压和速度：%.3lf %.3lf %d",
			tWeldParam.dWrapCurrentt1, tWeldParam.dWrapVoltage1, nCoorIndex);
		vector<MP_USR_VAR_INFO> vtCurVarInfo;
		vtCurVarInfo.push_back(pRobotDriver->PrepareValData(4, tWeldParam.dWrapCurrentt1));
		vtCurVarInfo.push_back(pRobotDriver->PrepareValData(5, tWeldParam.dWrapVoltage1 * 10));
		vtCurVarInfo.push_back(pRobotDriver->PrepareValData(3, 1));
		pRobotDriver->SetMultiVar_H(vtCurVarInfo);
		Sleep(50);
	}
	else if (E_WELD_TRACK == nCoorType && bChangeVoltage)
	{
		// 切换焊接速度和电流电压
		bChangeVoltage = false;
		pRobotDriver->m_cLog->Write("切换焊接电流电压和速度：%.3lf %.3lf %d",
			tWeldParam.dTrackCurrent, tWeldParam.dTrackVoltage, nCoorIndex);
		vector<MP_USR_VAR_INFO> vtCurVarInfo;
		vtCurVarInfo.push_back(pRobotDriver->PrepareValData(4, tWeldParam.dTrackCurrent));
		vtCurVarInfo.push_back(pRobotDriver->PrepareValData(5, tWeldParam.dTrackVoltage * 10));
		vtCurVarInfo.push_back(pRobotDriver->PrepareValData(3, 1));
		pRobotDriver->SetMultiVar_H(vtCurVarInfo);
		Sleep(50);
	}
}

bool CDiaphragmWeld::DoweldRuningSwing(CRobotDriverAdaptor* pRobotDriver, vector<T_ROBOT_COORS> vtRealWeldCoors, vector<int> vnPointType, bool bIsTracking)
{
	if (g_bLocalDebugMark)
	{
		return true;
	}
	GetNaturalPop().popInfo("开始焊接运动");
	//double dPulseToDisX = m_ptUnit->GetExPulseEquivalent(m_ptUnit->m_nTrackAxisNo);
	//MoveHandle_SetMMPulse(m_ptUnit->m_nRobotSmooth, dPulseToDisX);//算法设置脉冲当量	

	if (!bIsTracking)
	{
		//m_pIOControl->CloseLeftRedALaser();
	}
	clock_t tTime = clock();
	double dRobotWeldAdjustRZ = m_dWeldDipAngle;//根据工艺需要更改Rz角度
	int nFirstNum = pRobotDriver->m_vtWeldLineInWorldPoints.size();
	CString strRobot = pRobotDriver->m_strRobotName, strCoutPath;
	// 输出文件路径
	strCoutPath.Format("%s%s%s", OUTPUT_PATH, pRobotDriver->m_strRobotName, RECOGNITION_FOLDER);
	clock_t dStartTim = clock();
	m_pTraceModel->m_bIfWeldToEndPoint = false;
	m_pTraceModel->m_bCameraFindWeldEnd = false;
	m_pTraceModel->m_bSendWeldDataFinish = false;
	m_pTraceModel->m_bSendSafeFlag = false;
	m_pTraceModel->m_bIsCloseTrackThread = false;
	m_pScanInit->m_bOpenTrackingStatus = false;
	m_pTraceModel->m_bIfOpenExternal = true;
	m_pTraceModel->m_bIfJudgeEndOpen = false;
	//初始机械臂Y坐标
	int ncount = 0;
	int nindex = 0;
	int nDelPointNum = 0;// 删除重复点
	int nCountInt1 = 0;
	int nPointNum = 40;// 缓存点数
	m_dVelocityTransitionBound = 10000.0; // 临时测试跳枪速度
	pRobotDriver->SetIntVar(8, 0);
	if (bIsTracking) {
		pRobotDriver->SetIntVar(2, 9999);
	}
	else {
		pRobotDriver->SetIntVar(2, nFirstNum - 1);
	}
	//初始化跳枪起终序号
	bool bChangeSpeed = false; // 改变速度
	pRobotDriver->SetIntVar(7, -1);
	pRobotDriver->SetIntVar(6, 0);	// 焊接过程停弧 过渡
	pRobotDriver->SetIntVar(1, 0);
	pRobotDriver->SetIntVar(1, 0);
	double dTotalVal = m_pTraceModel->m_dWeldVelocity;
	int time2 = clock();
//	long lBPValue[3];
	T_ROBOT_COORS tTempRobotCoor;
//	MoveHandle_Pnt tRobot;
	vector<long> vlWeldPulseX, vlWeldPulseY;
	vector<double> vdWeldStepVel;
	vector<MP_USR_VAR_INFO> vtVarInfo;
	long lRobotvel = dTotalVal / 6;
//	long lMachine;
//	long MachVel;
	vtVarInfo.clear();
	vlWeldPulseX.clear();
	vlWeldPulseY.clear();
	vdWeldStepVel.clear();
	FILE* RecordRealData;
	if (m_ptUnit->m_bBreakPointContinue)
	{
		RecordRealData = fopen(strCoutPath + "实际焊接数据.txt", "a+");
	}
	else
	{
		RecordRealData = fopen(strCoutPath + "实际焊接数据.txt", "w");
	}

	m_pScanInit->m_pRecordTheoryPoint = fopen(strCoutPath + "理论焊接数据.txt", "w"); Sleep(50);
	if (m_ptUnit->m_bBreakPointContinue)
	{
		SaveRobotCoor(m_pScanInit->m_pRecordTheoryPoint, m_pTraceModel->m_vtBreakContinueBeforPoints, m_pTraceModel->m_vtBreakContinueBeforPointType);//保存断续前理论数据
	}
	SaveRobotCoor(m_pScanInit->m_pRecordTheoryPoint, pRobotDriver->m_vtWeldLineInWorldPoints, m_pTraceModel->m_vtWeldLinePointType);
																														
	int nWarpNum = 0;// 包角次数
	int nNextOpenTrackStep = 9999, nRecordNextOpenTrackStep = 0;;// 再次开启跟踪步号
	bool bChangeVoltage = false;// 改变电流电压
	double dRecordAdjustRZ = m_pTraceModel->m_dRecordBreakPointRZ;//记录RZ变化量
	int nRefer = ncount;
	int nProMoveNextNo = 1;
//	double dRecordMachinePos;// = m_dMachineAxis_X; // 及时记录机器人走过一份时大车位置
//	double dRecordRobotPosY;// = m_dRobotAxis_Y; // 及时记录机器人走过一份时机器人Y值
//	double dRecordRobotPosX;
//	double dRecordRobotPosZ;
	//double dAdjustRobotRZ = m_pTraceModel->m_dRecordBreakPointRZ;
	m_pTraceModel->m_dAdjustRobotRZ = m_pTraceModel->m_dRecordBreakPointRZ;
	int    nErrorAdjustRz = 0;
	int    nWeldLinePtnNum = nFirstNum;
	pRobotDriver->m_cLog->Write("初始I1变量：%d", pRobotDriver->GetIntVar(1));
	int nSave = 0;
	m_pTraceModel->m_vvtTrackWarpCoors.clear();
	m_pTraceModel->m_vvnTrackWarpPtType.clear();
	if (m_ptUnit->m_bBreakPointContinue)
	{
		m_pScanInit->LoadIsTrackCompleteFlag(pRobotDriver, m_pTraceModel->m_bCameraFindWeldEnd);
		LoadCurrentWarpNumber(pRobotDriver, nWarpNum);
	}

	bool bIsPause = false; // 是否正在跳枪外部轴停止状态
	int nPrePauseRobotNo = 0;
	vector<long> vlWeldPulseXNew(0), vlWeldPulseYNew(0);
	vector<double> vdWeldStepVelNew(0);
	
	//开启跟踪
	bool bIsOpenTrackAgain = false;
	E_MEASURE_POINT_TYPE tPtType = E_WELD_TRACK;
	if (nCountInt1 < m_pTraceModel->m_vtWeldLinePointType.size())
	{
		tPtType = (E_MEASURE_POINT_TYPE)m_pTraceModel->m_vtWeldLinePointType.at(nCountInt1);
		if (E_WELD_OPEN_TRACKING == tPtType &&
			nNextOpenTrackStep != nCountInt1)
		{
			nNextOpenTrackStep = nCountInt1;
			bIsOpenTrackAgain = true;
			pRobotDriver->m_cLog->Write("开启跟踪线程:nCountInt22:%d", nCountInt1);
		}
	}
	

	m_pScanInit->m_bOpenTrackingStatus = true;
	m_pTraceModel->m_bCameraFindWeldEnd = FALSE;
	m_ptUnit->SwitchIO("TrackLaser", true); Sleep(50);
	m_pScanInit->DynamicRealTimeTracking();
	pRobotDriver->m_cLog->Write("开启跟踪线程:nCountInt1:%d", nCountInt1);

	m_bSwingState = true;
	WriteSwingParam(1.5, 4.5, 0);
	SwingTracking(pRobotDriver, pRobotDriver->m_vtWeldLineInWorldPoints, dTotalVal);

	RecordRunTime(pRobotDriver, tTime, "焊接");
	fclose(RecordRealData);
	//记录焊接长度和当前运行步号
	//SaveWeldLengthData(pRobotDriver, m_pTraceModel->m_dCurrentWeldLen, dRecordAdjustRZ, nCountInt1 + m_pTraceModel->m_nWeldStepNo);
	//SaveCurrentWarpNumber(pRobotDriver, nWarpNum);
	//获取滤波缓存数据
	vector<TrackFilter_XYZ> vtBufferData = TrackFilter_GetCachePntsPlus(m_ptUnit->m_nRobotSmooth);
	CString strFile, strFile2;
	strFile.Format("%s滤波缓存数据.txt", strCoutPath);
	SaveDataTrackFilt(vtBufferData, strFile.GetBuffer(0));
	strFile2.Format("%sBreakPointReferencePulse.txt", strCoutPath);
	SaveDataRobotPulse(GetCurrentPulse(pRobotDriver), strFile2.GetBuffer(0));//存储端点时脉冲当做下枪参考坐标
	pRobotDriver->m_cLog->Write("%s 赋值线程退出!", strRobot);
	m_ptUnit->RobotCheckDone();
	pRobotDriver->HoldOff();
	m_pTraceModel->m_cStepMoveControl.EmgStop();
	m_pTraceModel->m_cStepMoveControl.StopStep();
	fclose(m_pScanInit->m_pRecordTheoryPoint);
	T_ROBOT_COORS tEndSafe2 = pRobotDriver->GetCurrentPos();
	RobotCoordPosOffset(tEndSafe2, tEndSafe2.dRZ, 50., 0);
	tEndSafe2.dZ = m_dWorkPieceHighTop + 20 * m_nRobotInstallDir;
	//tEndSafe2.dZ = m_dWorkPieceHighTop + 20 * m_nRobotInstallDir;
	T_ROBOT_MOVE_SPEED tPulseMove(m_dFastApproachToWorkSpeed, 20, 20);
	vector<T_ROBOT_MOVE_INFO> vtRobotMoveInfo(0);
	vtRobotMoveInfo.push_back(pRobotDriver->PVarToRobotMoveInfo(0, tEndSafe2, tPulseMove, MOVL));
	pRobotDriver->SetMoveValue(vtRobotMoveInfo);
	pRobotDriver->CallJob("CONTIMOVANY");
	CHECK_BOOL_RETURN(m_ptUnit->CheckRobotDone(tEndSafe2));

	return true;
}

UINT CDiaphragmWeld::ThreadMainContData4Estun(void* pParam)
{
	long lTimeS = GetTickCount(); //记录焊接开始时间

	int maxPullNum = 10; //8批量传输时，每次传输最大的传输容量(必须是偶数！！！)（埃斯顿目前每次传输最大的传输容量为10，但是最大速率传输机器人性能不够
	int batchSendSleepTime = 100; //批量传输一批数据后的等待时间
	int singleSendSleepTime = 200; //单个传输点后的等待时间
	/************** 收集到的参数 **********************/
	ESTUN_TRACK_PARAM* pEstunParam = reinterpret_cast<ESTUN_TRACK_PARAM*>(pParam);
	//vector<T_ROBOT_COORS>* weldPoint = pEstunParam->tdWeldTrace; //实时跟踪得到的全部轨迹的集合
	CRobotDriverAdaptor* pRobotDriver = pEstunParam->tdRobotDriver;

	/************** 要发出的参数 *********************/
	vector<T_ROBOT_COORS> WeldPathPoint; //每次实际发给埃斯顿的数据
	vector<vector<int>> cfg; //每次发送到数据对应的config

	/************** 计算cfg *************************/
	int cnfgTmp[7] = { 0 };
	T_ANGLE_PULSE CurPls = pRobotDriver->GetCurrentPulse();
	pRobotDriver->CalModeValue(CurPls, "8", cnfgTmp); //计算config值  -_-|||
	vector<int> vec(cnfgTmp, cnfgTmp + sizeof(cnfgTmp) / sizeof(int)); //接口的格式转换 (0_o |||

	/********** 整理点位数据及cfg数据 ***************/
	int curIndex = 0;
	int a = (pRobotDriver->m_vtWeldLineInWorldPoints).size();
	int count = 0;
	T_ROBOT_COORS tmp;

	T_ROBOT_COORS startPos = pRobotDriver->GetCurrentPos();
	if (!pRobotDriver->EstunTrackInit(startPos)) {
		XiMessageBox("埃斯顿跟踪接口初始化失败");
		return -1;
	}; //初始化跟踪数据

	if (false == pRobotDriver->EstunClearWeldBuffer()) {
		XiMessageBox("清理数据缓冲区失败");
	};
	Sleep(50); //开启成功后等一会

	/******** 这个while循环先批量发送数据，每次发 maxPullNum 个数据 直到剩余点位不足 maxPullNum 个再一个一个发。从而保证运动不卡 ********/
	TRACE(" * 当前时间： %ld :进入跟踪时缓冲区数据余量为:%d\n", clock(), (pRobotDriver->m_vtWeldLineInWorldPoints).size());
	if (pEstunParam->tdWaveType == 0)//不用摆焊
	{ 
		int epoch = 0;
		//(pRobotDriver->m_vtWeldLineInWorldPoints).size() / maxPullNum   //只要满足剩下的轨迹超过maxPullNum，就批量发maxPullNum
		while ((pRobotDriver->m_vtWeldLineInWorldPoints).size() / maxPullNum > epoch) {
			WeldPathPoint.clear();
			cfg.clear();
			try
			{
				pRobotDriver->m_mutex.Lock();
				for (int i = 0; i < maxPullNum; i++) {
					tmp = (pRobotDriver->m_vtWeldLineInWorldPoints).at(curIndex);
					tmp.dBY = 0;
					tmp.dBZ = 0;
					WeldPathPoint.push_back(tmp); //压入一个坐标
					cfg.push_back(vec); //tmp对应第config
					curIndex += 1;
					TRACE(" 当前时间： %ld :第%d个跟踪数据:  dX:%lf, dY:%lf,dZ:%lf,dRX:%lf,dRY:%lf,dRZ:%lf,dBX:%lf,dBY:%lf,dBZ:%lf\n", clock(), count++, tmp.dX, tmp.dY, tmp.dZ, tmp.dRZ, tmp.dRY, tmp.dRX, tmp.dBX, tmp.dBY, tmp.dBZ);
				}
				pRobotDriver->m_mutex.Unlock();
			}
			catch (const std::exception&)
			{
				XiMessageBox("访问跟踪轨迹越界");
			}
			/************** 发送数据 ******************/
			pRobotDriver->EstunSendTrackData(WeldPathPoint, cfg);
			TRACE(" **  当前时间： %ld :第%d个epoch，一共发送%d个WeldPathPoint，和%d个cfg\n", clock(), ++epoch, WeldPathPoint.size(), cfg.size());
			Sleep(batchSendSleepTime);//发完maxPullNum个数据等一会
		}
	}
	/*这里批量发送摆焊的数据，每次发n个前进数据+n个对应的摆动轨迹*/
	else 
	{
		int epoch = 0;
		if (curIndex == 0) 
		{
			WeldPathPoint.clear();
			cfg.clear();
			pRobotDriver->m_mutex.Lock();
			tmp = (pRobotDriver->m_vtWeldLineInWorldPoints).at(curIndex);
			pRobotDriver->m_mutex.Unlock();
			curIndex++;
			tmp.dBY = 0;
			tmp.dBZ = 0;
			WeldPathPoint.push_back(tmp); //压入一个坐标
			cfg.push_back(vec);
			TRACE(" 当前时间： %ld :第%d个跟踪数据:  dX:%lf, dY:%lf,dZ:%lf,dRX:%lf,dRY:%lf,dRZ:%lf,dBX:%lf,dBY:%lf,dBZ:%lf\n", clock(), count++, tmp.dX, tmp.dY, tmp.dZ, tmp.dRZ, tmp.dRY, tmp.dRX, tmp.dBX, tmp.dBY, tmp.dBZ);
			pRobotDriver->m_mutex.Lock();
			tmp = (pRobotDriver->m_vtWeldLineInWorldPoints).at(curIndex);
			pRobotDriver->m_mutex.Unlock();
			curIndex++;
			tmp.dBY = 0;
			tmp.dBZ = 0;
			WeldPathPoint.push_back(tmp); //压入第二个点
			TRACE(" 当前时间： %ld :第%d个跟踪数据:  dX:%lf, dY:%lf,dZ:%lf,dRX:%lf,dRY:%lf,dRZ:%lf,dBX:%lf,dBY:%lf,dBZ:%lf\n", clock(), count++, tmp.dX, tmp.dY, tmp.dZ, tmp.dRZ, tmp.dRY, tmp.dRX, tmp.dBX, tmp.dBY, tmp.dBZ);
			cfg.push_back(vec);

			pRobotDriver->m_mutex.Lock();
			for (int i = 0; i < (maxPullNum / 2 - 1); i++)
			{
				tmp = (pRobotDriver->m_vtWeldLineInWorldPoints).at(curIndex);
				tmp.dBY = 0;
				tmp.dBZ = 0;
				WeldPathPoint.push_back(tmp);
				cfg.push_back(vec);
				TRACE(" 当前时间： %ld :第%d个跟踪数据:  dX:%lf, dY:%lf,dZ:%lf,dRX:%lf,dRY:%lf,dRZ:%lf,dBX:%lf,dBY:%lf,dBZ:%lf\n", clock(), count++, tmp.dX, tmp.dY, tmp.dZ, tmp.dRZ, tmp.dRY, tmp.dRX, tmp.dBX, tmp.dBY, tmp.dBZ);
				tmp = (pRobotDriver->m_vtWeldLineInWorldPoints).at(curIndex - 2); //当前点对应的摆动点
				tmp.dBY = 0;
				tmp.dBZ = 0;
				WeldPathPoint.push_back(tmp);
				TRACE(" 当前时间： %ld :第%d个跟踪数据:  dX:%lf, dY:%lf,dZ:%lf,dRX:%lf,dRY:%lf,dRZ:%lf,dBX:%lf,dBY:%lf,dBZ:%lf\n", clock(), count++, tmp.dX, tmp.dY, tmp.dZ, tmp.dRZ, tmp.dRY, tmp.dRX, tmp.dBX, tmp.dBY, tmp.dBZ);
				cfg.push_back(vec);
				curIndex++;
			}
			pRobotDriver->m_mutex.Unlock();
			TRACE(" **  当前时间： %ld :第%d个epoch，一共发送%d个WeldPathPoint，和%d个cfg\n", clock(), ++epoch, WeldPathPoint.size(), cfg.size());
			pRobotDriver->EstunSendTrackData(WeldPathPoint, cfg);
			Sleep(batchSendSleepTime);//发完maxPullNum个数据等一会
		}

		while (((pRobotDriver->m_vtWeldLineInWorldPoints).size() - (maxPullNum / 2 + 1)) / (maxPullNum / 2) >= epoch) 
		{
			WeldPathPoint.clear();
			pRobotDriver->m_mutex.Lock();
			for (int i = 0; i < 4; i++)
			{
				tmp = (pRobotDriver->m_vtWeldLineInWorldPoints).at(curIndex);
				tmp.dBY = 0;
				tmp.dBZ = 0;
				WeldPathPoint.push_back(tmp);
				cfg.push_back(vec);
				TRACE(" 当前时间： %ld :第%d个跟踪数据:  dX:%lf, dY:%lf,dZ:%lf,dRX:%lf,dRY:%lf,dRZ:%lf,dBX:%lf,dBY:%lf,dBZ:%lf\n", clock(), count++, tmp.dX, tmp.dY, tmp.dZ, tmp.dRZ, tmp.dRY, tmp.dRX, tmp.dBX, tmp.dBY, tmp.dBZ);
				tmp = (pRobotDriver->m_vtWeldLineInWorldPoints).at(curIndex - 2); //当前点对应的摆动点
				tmp.dBY = 0;
				tmp.dBZ = 0;
				TRACE(" 当前时间： %ld :第%d个跟踪数据:  dX:%lf, dY:%lf,dZ:%lf,dRX:%lf,dRY:%lf,dRZ:%lf,dBX:%lf,dBY:%lf,dBZ:%lf\n", clock(), count++, tmp.dX, tmp.dY, tmp.dZ, tmp.dRZ, tmp.dRY, tmp.dRX, tmp.dBX, tmp.dBY, tmp.dBZ);
				WeldPathPoint.push_back(tmp);
				cfg.push_back(vec);
				curIndex++;
			}
			pRobotDriver->m_mutex.Unlock();
			TRACE(" **  当前时间： %ld :第%d个epoch，一共发送%d个WeldPathPoint，和%d个cfg\n", clock(), ++epoch, WeldPathPoint.size(), cfg.size());
			pRobotDriver->EstunSendTrackData(WeldPathPoint, cfg);

			epoch++;
			Sleep(batchSendSleepTime);//发完maxPullNum个数据等一会
		}
	}

	TraceModel* pTraceModel = nullptr;
	/********* 这个循环将剩余点位一个一个发下去 **************************/
	while (TRUE != pTraceModel->m_bCameraFindWeldEnd && TRUE == pRobotDriver->WorldIsRunning())
	{
		if (curIndex != (pRobotDriver->m_vtWeldLineInWorldPoints).size() - 1) 
		{
			WeldPathPoint.clear();
			cfg.clear();
			if (pEstunParam->tdWaveType == 0) //不用摆焊
			{  

				try
				{
					pRobotDriver->m_mutex.Lock();
					tmp = (pRobotDriver->m_vtWeldLineInWorldPoints).at(curIndex);
					tmp.dBY = 0;
					tmp.dBZ = 0;
					pRobotDriver->m_mutex.Unlock();
					curIndex++;
					WeldPathPoint.push_back(tmp);
					cfg.push_back(vec);
					//WriteLog("第%d个跟踪数据:  dX:%lf, dY:%lf,dZ:%lf,dRX:%lf,dRY:%lf,dRZ:%lf,dBX:%lf,dBY:%lf,dBZ:%lf", count, tmp.dX, tmp.dY, tmp.dZ, tmp.dRX, tmp.dRY, tmp.dRZ, tmp.dBX, tmp.dBY, tmp.dBZ);
					TRACE(" 当前时间： %ld :第%d个跟踪数据:  dX:%lf, dY:%lf,dZ:%lf,dRX:%lf,dRY:%lf,dRZ:%lf,dBX:%lf,dBY:%lf,dBZ:%lf\n", clock(), count++, tmp.dX, tmp.dY, tmp.dZ, tmp.dRZ, tmp.dRY, tmp.dRX, tmp.dBX, tmp.dBY, tmp.dBZ);
				}
				catch (const std::exception&)
				{
					XiMessageBox("访问跟踪轨迹时越界");
				}

			}
			else  //摆焊
			{ 
				try
				{
					pRobotDriver->m_mutex.Lock();
					WeldPathPoint.push_back((pRobotDriver->m_vtWeldLineInWorldPoints).at(curIndex - 2)); //如果要摆动的话就插入当前点及上上一个点
					WeldPathPoint.push_back((pRobotDriver->m_vtWeldLineInWorldPoints).at(curIndex));
					pRobotDriver->m_mutex.Unlock();

					cfg.push_back(vec);
					cfg.push_back(vec); //重复塞两次因为有两个坐标
					curIndex++;
					//WriteLog("第%d个跟踪数据:  dX:%lf, dY:%lf,dZ:%lf,dRX:%lf,dRY:%lf,dRZ:%lf,dBX:%lf,dBY:%lf,dBZ:%lf", count, tmp.dX, tmp.dY, tmp.dZ, tmp.dRX, tmp.dRY, tmp.dRZ, tmp.dBX, tmp.dBY, tmp.dBZ);
					TRACE(" 当前时间： %ld :第%d个跟踪数据:  dX:%lf, dY:%lf,dZ:%lf,dRX:%lf,dRY:%lf,dRZ:%lf,dBX:%lf,dBY:%lf,dBZ:%lf\n", clock(), count++, tmp.dX, tmp.dY, tmp.dZ, tmp.dRZ, tmp.dRY, tmp.dRX, tmp.dBX, tmp.dBY, tmp.dBZ);
				}
				catch (const std::exception&)
				{
					XiMessageBox("访问跟踪轨迹时越界");
				}
			}
			/************** 发送数据 ******************/
			pRobotDriver->EstunSendTrackData(WeldPathPoint, cfg); //补充发送单个剩余点
			//Sleep(singleSendSleepTime); //等待图像处理
		}
		else 
		{
			Sleep(1);
			TRACE("* 发送单个数据中, 当前时间： %ld 正在等待第%d个跟踪数据\n", clock(), count);
		}

	}

	/********* 焊接所有点位已经发送完了 下面是做结尾检测 以及停止跟踪***********/
	TRACE(" 当前时间： %ld :结束时容器大小:%d\n", clock(), (pRobotDriver->m_vtWeldLineInWorldPoints).size());
	Sleep(1500);
	TRACE("1.5秒后容器大小:%d\n", (pRobotDriver->m_vtWeldLineInWorldPoints).size());
	pRobotDriver->EstunTrackStop();

	//焊接数据统计

	double dLen = 0;
#ifdef SINGLE_ROBOT
	dLen = fabs(pRobotDriver->m_vtWeldLineInWorldPoints.back().dBY - pRobotDriver->m_vtWeldLineInWorldPoints.front().dBY) / 1000;
#else
	dLen = fabs(pRobotDriver->m_vtWeldLineInWorldPoints.back().dBX - pRobotDriver->m_vtWeldLineInWorldPoints.front().dBX) / 1000;
#endif // !SINGLE_ROBOT

	//long lTimeE = GetTickCount() - lTimeS; //记录焊接开始时间
	//CStatisticalData* pStatisticalData = CStatisticalData::getInstance();
	//pStatisticalData->UpdateWeldData(E_FLAT_SEAM,dLen,lTimeE);

	return 0;
}

bool CDiaphragmWeld::EstunTracking(CRobotDriverAdaptor* pRobotDriver, std::vector<T_ROBOT_COORS> vtRobotCoors, double dSpeed, int waveType)
{
	if (!m_bSwingState) {
		return true;
	}

	bool ESTstrat = false;
	clock_t testTime = clock();

	estunParam.tdRobotDriver = pRobotDriver;
	estunParam.tdWeldTrace = &(pRobotDriver->m_vtWeldLineInWorldPoints); //所有实时轨迹
	estunParam.tdWaveType = waveType; // 默认摆动方式为0


	if (0 > pRobotDriver->CallJob("track")) {
		//调用跟踪程序失败
		XiMessageBox("埃斯顿跟踪程序调用失败");
	}

	pRobotDriver->TrackingState = true; //开启跟踪成功

	//设置焊接工艺
	vector<T_WELD_PARA> vtWeldPara;
	T_WELD_PARA tWeldPara;
	E_WELD_SEAM_TYPE eWeldSeamType = E_FLAT_SEAM;
	if (false == GetCurWeldParam(eWeldSeamType, vtWeldPara))
	{
		XiMessageBox("跟踪：加载焊接工艺参数失败！");
		return false;
	}
	//EstunAdaptor* pEst;
	//pEst->SetRealVar(0, 280);
	//pEst->SetRealVar(1, 28);


	CWinThread* pThread = AfxBeginThread(ThreadMainContData4Estun, &estunParam);
	if (pThread != NULL) {
		WriteLog("\n已经启动埃斯顿主控线程 时间:%d\n", clock() - testTime);
	}
	else {
		WriteLog("创建埃斯顿主控线程失败 时间:%d\n", clock() - testTime);
		XiMessageBox("创建埃斯顿跟踪主控线程失败！");
		pRobotDriver->TrackingState = false; //关闭跟踪
	}
	m_bSwingState = false;


	return true;
}


bool CDiaphragmWeld::DoweldRuning(CRobotDriverAdaptor* pRobotDriver, vector<T_ROBOT_COORS> vtRealWeldCoors, vector<int> vnPointType, bool bIsTracking, E_WELD_SEAM_TYPE eWeldSeamType)
{
	if (g_bLocalDebugMark)
	{
		return true;
	}
	GetNaturalPop().popInfo("开始焊接运动");
	double dPulseToDisX = m_ptUnit->GetExPulseEquivalent(m_ptUnit->m_nTrackAxisNo);
	MoveHandle_SetMMPulse(m_ptUnit->m_nRobotSmooth, dPulseToDisX);//算法设置脉冲当量	

	if (!bIsTracking)
	{
		//m_pIOControl->CloseLeftRedALaser();
	}
	long long tTime = XI_clock();
	double dRobotWeldAdjustRZ = m_dWeldDipAngle;//根据工艺需要更改Rz角度
	int nFirstNum = pRobotDriver->m_vtWeldLineInWorldPoints.size();
	CString strRobot = pRobotDriver->m_strRobotName, strCoutPath;
	// 输出文件路径
	strCoutPath.Format("%s%s%s", OUTPUT_PATH, pRobotDriver->m_strRobotName, RECOGNITION_FOLDER);
	long long dStartTim = XI_clock();
	m_pTraceModel->m_bIfWeldToEndPoint = false;
	m_pTraceModel->m_bCameraFindWeldEnd = false;
	m_pTraceModel->m_bSendWeldDataFinish = false;
	m_pTraceModel->m_bSendSafeFlag = false;
	m_pTraceModel->m_bIsCloseTrackThread = false;
	m_pScanInit->m_bOpenTrackingStatus = false;
	m_pTraceModel->m_bIfOpenExternal = true;
	m_pTraceModel->m_bIfJudgeEndOpen = false;
	//初始机械臂Y坐标
	int ncount = 0;
	int nindex = 0;
	int nDelPointNum = 0;// 删除重复点
	int nCountInt1 = 0;
	int nPointNum = 40;// 缓存点数
	m_dVelocityTransitionBound = 10000.0; // 临时测试跳枪速度
	pRobotDriver->SetIntVar(8, 0);
	if (bIsTracking) {
		pRobotDriver->SetIntVar(2, 9999);
	}
	else {
		pRobotDriver->SetIntVar(2, nFirstNum - 1);
	}
	//初始化跳枪起终序号
	bool bChangeSpeed = false; // 改变速度
	pRobotDriver->SetIntVar(7, -1);
	pRobotDriver->SetIntVar(6, 0);	// 焊接过程停弧 过渡
	pRobotDriver->SetIntVar(1, 0);
	pRobotDriver->SetIntVar(1, 0);
	double dTotalVal = m_pTraceModel->m_dWeldVelocity;
	long long time2 = XI_clock();
	long lBPValue[3];
	T_ROBOT_COORS tTempRobotCoor;
	MoveHandle_Pnt tRobot;
	vector<long> vlWeldPulseX, vlWeldPulseY;
	vector<double> vdWeldStepVel;
	vector<MP_USR_VAR_INFO> vtVarInfo;
	long lRobotvel = dTotalVal / 6;
	long lMachine;
	long MachVel;
	vtVarInfo.clear();
	vlWeldPulseX.clear();
	vlWeldPulseY.clear();
	vdWeldStepVel.clear();
	FILE* RecordRealData;
	if (m_ptUnit->m_bBreakPointContinue)
	{
		RecordRealData = fopen(strCoutPath + "实际焊接数据.txt", "a+");
	}
	else
	{
		RecordRealData = fopen(strCoutPath + "实际焊接数据.txt", "w");
	}
	m_pScanInit->m_pRecordTheoryPoint = fopen(strCoutPath + "理论焊接数据.txt", "w"); Sleep(50);
	if (m_ptUnit->m_bBreakPointContinue)
	{
		SaveRobotCoor(m_pScanInit->m_pRecordTheoryPoint, m_pTraceModel->m_vtBreakContinueBeforPoints,m_pTraceModel->m_vtBreakContinueBeforPointType);//保存断续前理论数据
	}
	SaveRobotCoor(m_pScanInit->m_pRecordTheoryPoint, pRobotDriver->m_vtWeldLineInWorldPoints, m_pTraceModel->m_vtWeldLinePointType);//保存理论数据
	if (m_pTraceModel->m_bIfOpenExternal)
	{
#ifdef SINGLE_ROBOT
		MoveHandle_Init(m_ptUnit->m_nRobotSmooth, pRobotDriver->m_vtWeldLineInWorldPoints[0].dX,
			pRobotDriver->m_vtWeldLineInWorldPoints[0].dBY, pRobotDriver->m_vtWeldLineInWorldPoints[0].dZ, dTotalVal);
#else
		MoveHandle_Init(m_ptUnit->m_nRobotSmooth, pRobotDriver->m_vtWeldLineInWorldPoints[0].dY * (-1),
			pRobotDriver->m_vtWeldLineInWorldPoints[0].dBX, pRobotDriver->m_vtWeldLineInWorldPoints[0].dZ, dTotalVal);
#endif
	}
	while (false == m_pTraceModel->m_bIfWeldToEndPoint)
	{
		if (ncount < nFirstNum)
		{
			if (ncount < nPointNum)
			{
				pRobotDriver->m_cLog->Write("%s 原始数据:%d %20.10lf %20.10lf %20.10lf %20.10lf %20.10lf %20.10lf", strRobot, ncount,
					pRobotDriver->m_vtWeldLineInWorldPoints[nindex].dX, pRobotDriver->m_vtWeldLineInWorldPoints[nindex].dY,
					pRobotDriver->m_vtWeldLineInWorldPoints[nindex].dZ, pRobotDriver->m_vtWeldLineInWorldPoints[nindex].dBX,
					pRobotDriver->m_vtWeldLineInWorldPoints[nindex].dBY, pRobotDriver->m_vtWeldLineInWorldPoints[nindex].dRZ
				);
				// 计算焊接和包角时速度
				SwitchWeldSpeed(pRobotDriver, nindex, m_pTraceModel->m_vtWeldLinePointType.at(nindex),
					dTotalVal, m_dVelocityWarp, lRobotvel, bChangeSpeed);
				if (m_pTraceModel->m_bIfOpenExternal && !m_ptUnit->GetExAxisType(m_ptUnit->m_nTrackAxisNo))
				{
					// 获取截断数据
					GetTruncatedData(pRobotDriver, pRobotDriver->m_vtWeldLineInWorldPoints[nindex], tRobot, lRobotvel, lMachine, MachVel);
					vlWeldPulseX.push_back(lMachine);
					vlWeldPulseY.push_back(0);
					vdWeldStepVel.push_back((double)MachVel);
					tTempRobotCoor = pRobotDriver->m_vtWeldLineInWorldPoints[nindex];
#ifdef SINGLE_ROBOT
					tTempRobotCoor.dX = tRobot.x_ / 1000.;
#else	
					tTempRobotCoor.dY = tRobot.y_ / 1000.;
#endif			
					tTempRobotCoor.dZ = tRobot.z_ / 1000.;

					pRobotDriver->m_cLog->Write("%s ncount:%d,初次焊接轨迹11:%11.20lf%20.10lf%20.10lf RZ:%lf 基坐标：%20.10lf",
						strRobot, ncount, tRobot.x_ / 1000., tTempRobotCoor.dY, tRobot.z_ / 1000.,
						pRobotDriver->m_vtWeldLineInWorldPoints[nindex].dRZ, pRobotDriver->m_vtWeldLineInWorldPoints[nindex].dBY);
				}
				else
				{
					tTempRobotCoor = pRobotDriver->m_vtWeldLineInWorldPoints[nindex];
					lBPValue[0] = (long)(pRobotDriver->m_vtWeldLineInWorldPoints[nindex].dBX * 1000);
					lBPValue[1] = (long)(pRobotDriver->m_vtWeldLineInWorldPoints[nindex].dBY * 1000);
					lBPValue[2] = (long)(pRobotDriver->m_vtWeldLineInWorldPoints[nindex].dBZ * 1000);
					vtVarInfo.push_back(pRobotDriver->PrepareValData((ncount % nPointNum + 20), lBPValue));
				}
				// 焊接姿态
				/*tTempRobotCoor.dRX = m_dNormalWeldRx;
				tTempRobotCoor.dRY = m_dNormalWeldRy;*/
				vtVarInfo.push_back(pRobotDriver->PrepareValData((ncount % nPointNum + 20), tTempRobotCoor));
				vtVarInfo.push_back(pRobotDriver->PrepareValData((ncount % nPointNum + 16), lRobotvel));

				if (ncount == nPointNum - 1 || ncount == (nFirstNum - 1))
				{
					pRobotDriver->SetMultiVar_H(vtVarInfo);
					pRobotDriver->HoldOff(); Sleep(100);
					pRobotDriver->ServoOn(); Sleep(100);
					if (m_ptUnit->GetExAxisType(m_ptUnit->m_nTrackAxisNo)) //安川外部轴
					{
						//jwq 平焊立焊调用不同JOB 
						if (E_FLAT_SEAM == eWeldSeamType)
						{
							pRobotDriver->CallJob("CIRCULATIONWELD-BP");
						}
						else if (E_STAND_SEAM == eWeldSeamType)
						{
							pRobotDriver->CallJob("CIRCULATIONWELD-L-BP");
						}
						else
						{
							XiMessageBox("不存在的焊接类型！");
							return false;
						}
					}
					else
					{
						pRobotDriver->CallJob("CIRCULATIONWELD");
						Sleep(250);
						if (0 == vdWeldStepVel.at(0))
						{
							vdWeldStepVel.at(0) = vdWeldStepVel.at(1);
						}
						m_pTraceModel->m_cStepMoveControl.m_bInsterBuffer = true;
						//轴号
						WORD awAxis[] = { WORD(m_pTraceModel->m_nAXIS_X), WORD(m_pTraceModel->m_nAXIS_Y) };
						//实际运行脉冲向量
						vector <long> avlRelPulse[] = { vlWeldPulseX,vlWeldPulseY };
						m_pTraceModel->m_cStepMoveControl.SetStartStopStepNum(0);
						m_pTraceModel->m_cStepMoveControl.SetBuffNum(2);
						m_pTraceModel->m_cStepMoveControl.SetStartStepNo(0);
						m_pTraceModel->m_cStepMoveControl.StepMove(awAxis, avlRelPulse, vdWeldStepVel, 2, 1);
					}
				}
				ncount++;
				nindex++;
				if (ncount == (nFirstNum))
				{
					WriteLog("%s 数据小于19个直接运行job:%d", strRobot, nFirstNum);
					break;
				}
			}
			else
			{
				break;
			}
		}
		Sleep(5);
		DoEvent();
	}
	int nWarpNum = 0;// 包角次数
	int nNextOpenTrackStep = 9999, nRecordNextOpenTrackStep = 0;;// 再次开启跟踪步号
	bool bChangeVoltage = false;// 改变电流电压
	double dRecordAdjustRZ = m_pTraceModel->m_dRecordBreakPointRZ;//记录RZ变化量
	int nRefer = ncount;
	int nProMoveNextNo = 1;
	double dRecordMachinePos;// = m_dMachineAxis_X; // 及时记录机器人走过一份时大车位置
	double dRecordRobotPosY;// = m_dRobotAxis_Y; // 及时记录机器人走过一份时机器人Y值
	double dRecordRobotPosX;
	double dRecordRobotPosZ;
	//double dAdjustRobotRZ = m_pTraceModel->m_dRecordBreakPointRZ;
	m_pTraceModel->m_dAdjustRobotRZ = m_pTraceModel->m_dRecordBreakPointRZ;
	int    nErrorAdjustRz = 0;
	int    nWeldLinePtnNum = nFirstNum;
	pRobotDriver->m_cLog->Write("初始I1变量：%d", pRobotDriver->GetIntVar(1));
	int nSave = 0;
	m_pTraceModel->m_vvtTrackWarpCoors.clear();
	m_pTraceModel->m_vvnTrackWarpPtType.clear();
	if (m_ptUnit->m_bBreakPointContinue)
	{
		m_pScanInit->LoadIsTrackCompleteFlag(pRobotDriver, m_pTraceModel->m_bCameraFindWeldEnd);
		LoadCurrentWarpNumber(pRobotDriver, nWarpNum);
	}
	int nPointNoAfterJump = -1;//跳枪后轨迹序号
	while (false == m_pTraceModel->m_bIfWeldToEndPoint)
	{
		CHECK_ROBOT_EMG_BREAK(m_ptUnit);
		T_ROBOT_COORS tRecordCurrentPos = pRobotDriver->GetCurrentPos();
		/*if (pRobotDriver->GetIntVar(8)==32767)
		{
			m_pTraceModel->m_cStepMoveControl.ContiMovePause();
		}
		else
		{
			m_pTraceModel->m_cStepMoveControl.ContiMoveRestore();
		}*/
		if (0 == nSave % 10) {
			SaveRobotCoor(RecordRealData, tRecordCurrentPos);//保存实际位置数据
		}
		nSave++;
		if (TRUE == m_pTraceModel->m_bIfWeldToEndPoint) {
			pRobotDriver->m_cLog->Write("%s 赋值线程 break!", strRobot);
			break;
		}
		if (nCountInt1 == nWeldLinePtnNum - 1 && !bIsTracking) {
			pRobotDriver->m_cLog->Write("%s 赋值线程退出break:%d!", strRobot, nWeldLinePtnNum);
			pRobotDriver->SetIntVar(3, 6666);
			break;
		}
		nCountInt1 = pRobotDriver->GetIntVar(1);
		//开启跟踪
		bool bIsOpenTrackAgain = false;
		E_MEASURE_POINT_TYPE tPtType = E_WELD_TRACK;
		if (nCountInt1 < m_pTraceModel->m_vtWeldLinePointType.size())
		{
			tPtType = (E_MEASURE_POINT_TYPE)m_pTraceModel->m_vtWeldLinePointType.at(nCountInt1);
			//pRobotDriver->m_cLog->Write("开启跟踪线程:tPtType22:%d nCountInt1;%d m_vtWeldLinePointType;%d", tPtType, nCountInt1, m_pTraceModel->m_vtWeldLinePointType.size());
			if (E_WELD_OPEN_TRACKING == tPtType &&
				nNextOpenTrackStep != nCountInt1)
			{
				nNextOpenTrackStep = nCountInt1;
				bIsOpenTrackAgain = true;
				pRobotDriver->m_cLog->Write("开启跟踪线程:nCountInt22:%d", nCountInt1);
			}
		}
		// 单次跟踪过程只进一次跟踪线程，开启跟踪条件：初次起弧后进入正常焊接姿态或包角结束后再次进入跟踪姿态
		if (bIsTracking && (E_WELD_TRACK & tPtType) && !(E_WELD_TRACK_CHANGE_POSTURE & tPtType) &&
			!m_pScanInit->m_bOpenTrackingStatus &&
			!m_pTraceModel->m_bCameraFindWeldEnd ||
			bIsOpenTrackAgain) {
			m_pScanInit->m_bOpenTrackingStatus = true;
			m_pTraceModel->m_bCameraFindWeldEnd = FALSE;
			//------------------------------------
			m_pTraceModel->m_bIfWeldToEndPoint = false; // !!!!!!!!
			m_pTraceModel->m_bSendWeldDataFinish = false;
			m_pTraceModel->m_bSendSafeFlag = false;
			m_pTraceModel->m_bIsCloseTrackThread = false;

			m_ptUnit->SwitchIO("TrackLaser", true); Sleep(50); //m_pIOControl->OpenLeftRedALaser(); 

			//---------------------------------------
			E_JUDGE_END_METHOD eJudgeEndMethod = nWarpNum < 2 ? E_JUDGE_END_PROCESS_SEARCH : E_JUDGE_END_KNOW;
			T_ROBOT_COORS* pEndCoord = nWarpNum < 2 ? NULL : &pRobotDriver->m_vtWeldLineInWorldPoints[0];
			m_pScanInit->InitRealTimeTracking(eJudgeEndMethod, pEndCoord); // 点云方式跟踪初始化

			m_pScanInit->DynamicRealTimeTracking();
			pRobotDriver->m_cLog->Write("开启跟踪线程:nCountInt1:%d", nCountInt1);
		}
		// 发送包角数据
		int nPointCountBeforeJump = -1;//跳枪前轨迹总数
		int nPointCountAfterJump = -1;//跳枪后轨迹总数
		if (m_pTraceModel->m_bIsTrackingDataSendOver)
		{
			nPointCountBeforeJump = pRobotDriver->m_vtWeldLineInWorldPoints.size();
			m_pTraceModel->m_bIsTrackingDataSendOver = false;
			pRobotDriver->m_cLog->Write("发送包角数据：%d", pRobotDriver->m_vtWeldLineInWorldPoints.size());
			m_ptUnit->m_bBreakPointContinue = false;
			// 计算跳枪包角轨迹
			vector<T_ROBOT_COORS> vtWarpCoors(0);
			vector<int> vtWarpPtType(0);
			// 跳枪包角时调整量，临时
			//double dAdjustVal = 0.0;
			//if (0 == pRobotDriver->m_nRobotNo)
			//{
			//	dAdjustVal = 0.0; //3.0;
			//}
			//else if (1 == pRobotDriver->m_nRobotNo)
			//{
			//	dAdjustVal = 0.0; //3.0;
			//}
			//else if (2 == pRobotDriver->m_nRobotNo)
			//{
			//	dAdjustVal = 0.0;
			//}
			//else if (2 == pRobotDriver->m_nRobotNo)
			//{
			//	dAdjustVal = 0.0;
			//}
			//----------------------------------

			//if (!CalcJumpWarpTrack(nWarpNum, m_pScanInit->m_pTraceModel->dBoardThink, m_pTraceModel->m_dGunToEyeCompenX, m_pScanInit->m_pTraceModel->m_dFlatHorComp, pRobotDriver->m_vtWeldLineInWorldPoints, m_dWorkPieceHighTop,
			//	vtWarpCoors, vtWarpPtType))
			//{
			//	pRobotDriver->HoldOn();
			//	//pRobotDriver->RobotEmg();
			//	XiMessageBoxOk("跳枪数据计算失败！");
			//	return false;
			//}
			// 获取包角参数
			E_WRAPANGLE_PARAM tWarpParam = LoadWarpParam(m_ptUnit->GetRobotCtrl()->m_strRobotName, E_WRAPANGLE_ONCE, nWarpNum);
			

			// 过程搜终点一次包角：删除 pRobotDriver->m_vtWeldLineInWorldPoints 中结束段包角轨迹
			if (nWarpNum < 2 && m_pTraceModel->m_eStartWrapAngleType == E_WRAPANGLE_ONCE) // 需要一次包角
			{
				T_ROBOT_COORS tLastCoord = pRobotDriver->m_vtWeldLineInWorldPoints.back();
				for (int i = pRobotDriver->m_vtWeldLineInWorldPoints.size() - 1; i > 0; i--)
				{
					T_ROBOT_COORS tCurFindCoord = pRobotDriver->m_vtWeldLineInWorldPoints[i];
					double dDis = TwoPointDis(tCurFindCoord.dX + tCurFindCoord.dBX, tCurFindCoord.dY + tCurFindCoord.dBY, tCurFindCoord.dZ + tCurFindCoord.dBZ,
						tLastCoord.dX + tLastCoord.dBX, tLastCoord.dY + tLastCoord.dBY, tLastCoord.dZ + tLastCoord.dBZ);
					if (dDis < tWarpParam.dWarpLength)
					{
						pRobotDriver->m_vtWeldLineInWorldPoints.pop_back();
						m_pTraceModel->m_vtWeldLinePointType.pop_back();
					}
					else
					{
						break;
					}
				}
			}


			if (!CalcOnceWarpTrack(nWarpNum, m_pScanInit->m_pTraceModel->dBoardThink, m_pTraceModel->m_dGunToEyeCompenX, m_pScanInit->m_pTraceModel->m_dFlatHorComp, 
				pRobotDriver->m_vtWeldLineInWorldPoints, m_dWorkPieceHighTop, vtWarpCoors, vtWarpPtType, tWarpParam))
			{
				pRobotDriver->HoldOn();
				XiMessageBoxOk("一次包角数据计算失败！");
				return false;
			}

			nPointCountAfterJump = nPointCountBeforeJump + vtWarpCoors.size();
			m_pTraceModel->m_vvtTrackWarpCoors.push_back(vtWarpCoors);
			m_pTraceModel->m_vvnTrackWarpPtType.push_back(vtWarpPtType);
			CString strFile;
			strFile.Format("A_Test_%d.txt", nWarpNum);
			FILE* pf = fopen(strFile, "w");
			SaveRobotCoor(pf, vtWarpCoors, vtWarpPtType);
			fclose(pf);
			strFile.Format("A_Test2_%d.txt", nWarpNum);
			FILE* pf2 = fopen(strFile, "w");
			SaveRobotCoor(pf2, pRobotDriver->m_vtWeldLineInWorldPoints, m_pTraceModel->m_vtWeldLinePointType);
			fclose(pf2);
			// ---------------------------------------------------------------
//			m_pTraceModel->m_vtRealEndpointCoor.size() - 1;
			if (nWarpNum < 2/*m_pTraceModel->m_vvtTrackWarpCoors.size()*/ && 
				m_pTraceModel->m_eStartWrapAngleType == E_WRAPANGLE_ONCE)
			{
				// 一次包角 跳枪处 不使用 修正包角数据
				//switch (*m_pUnWarpBoundWeld)
				//{
				//// 一次包角 跳枪处 不使用 修正包角数据
				//switch (*m_pUnWarpBoundWeld)
				//{
				//case 0:		//不跳枪
				//	CHECK_BOOL_RETURN(CorrectWarpDataFun(m_pTraceModel->m_vtRecordWarpBeforWeldData, m_pTraceModel->m_vvtWarpCorrectCoors.at(nWarpNum), nWarpNum,
				//		m_pTraceModel->m_vvtTrackWarpCoors.at(nWarpNum), m_pTraceModel->m_vvnTrackWarpPtType.at(nWarpNum)));
				//	break;
				//case 1:		//起点跳枪
				//	if (nWarpNum % 2 == 0)
				//	{
				//		CHECK_BOOL_RETURN(CorrectWarpDataFun(m_pTraceModel->m_vtRecordWarpBeforWeldData, m_pTraceModel->m_vvtWarpCorrectCoors.at(nWarpNum), nWarpNum,
				//			m_pTraceModel->m_vvtTrackWarpCoors.at(nWarpNum), m_pTraceModel->m_vvnTrackWarpPtType.at(nWarpNum)));
				//	}
				//	break;
				//case 2:		//终点跳枪
				//	if (nWarpNum % 2 == 1)
				//	{
				//		CHECK_BOOL_RETURN(CorrectWarpDataFun(m_pTraceModel->m_vtRecordWarpBeforWeldData, m_pTraceModel->m_vvtWarpCorrectCoors.at(nWarpNum), nWarpNum,
				//			m_pTraceModel->m_vvtTrackWarpCoors.at(nWarpNum), m_pTraceModel->m_vvnTrackWarpPtType.at(nWarpNum)));
				//	}
				//	break;
				//case 3:		//两边都条
				//	break;
				//default:
				//	CHECK_BOOL_RETURN(CorrectWarpDataFun(m_pTraceModel->m_vtRecordWarpBeforWeldData, m_pTraceModel->m_vvtWarpCorrectCoors.at(nWarpNum), nWarpNum,
				//		m_pTraceModel->m_vvtTrackWarpCoors.at(nWarpNum), m_pTraceModel->m_vvnTrackWarpPtType.at(nWarpNum)));
				//	break;
				//} 							

				bool bNeedOpenTrack = false;
				bool bIsSendTransitionPoint = true;// = m_pUnWarpBoundWeld > 0 ? true : false;	//从一号变量开始发 1-10为过渡点
				int nTrackNo, nIndx;
				vector<T_ROBOT_COORS> vtStartTrackCoors;
				for (nTrackNo = 0; nTrackNo < m_pTraceModel->m_vvtTrackWarpCoors.at(nWarpNum).size(); nTrackNo++) {
					//跟踪包角数据 补偿
					T_ROBOT_COORS tCoors = m_pTraceModel->m_vvtTrackWarpCoors.at(nWarpNum).at(nTrackNo);
					if (E_WELD_TRACK != m_pTraceModel->m_vvnTrackWarpPtType.at(nWarpNum).at(nTrackNo) &&
						E_TRANSITION_ARCOFF_FAST_POINT != m_pTraceModel->m_vvnTrackWarpPtType.at(nWarpNum).at(nTrackNo))
					{
						if (E_WELD_OPEN_TRACKING == m_pTraceModel->m_vvnTrackWarpPtType.at(nWarpNum).at(nTrackNo))
						{
							CalcMeasureInBase(m_pTraceModel->m_vvtTrackWarpCoors.at(nWarpNum).at(nTrackNo),
								m_pTraceModel->m_dGunToEyeCompenX, m_pTraceModel->m_dGunToEyeCompenZ, tCoors);
						}
						/*CalcMeasureInBase(m_pTraceModel->m_vvtTrackWarpCoors.at(nWarpNum).at(nTrackNo),
							m_pTraceModel->m_dGunToEyeCompenX, m_pTraceModel->m_dGunToEyeCompenZ, tCoors);*/
						if (bIsTracking)
						{
							tCoors.dRZ += dRobotWeldAdjustRZ;//根据工艺需要更改Rz角度
						}
						SaveRobotCoor(m_pScanInit->m_pRecordTheoryPoint, tCoors, m_pTraceModel->m_vvnTrackWarpPtType.at(nWarpNum).at(nTrackNo));
						pRobotDriver->m_vtWeldLineInWorldPoints.push_back(tCoors);
						m_pTraceModel->m_vtWeldLinePointType.push_back(m_pTraceModel->m_vvnTrackWarpPtType.at(nWarpNum).at(nTrackNo));
					}
					if (E_TRANSITION_ARCOFF_FAST_POINT == m_pTraceModel->m_vvnTrackWarpPtType.at(nWarpNum).at(nTrackNo) && bIsSendTransitionPoint)
					{
						m_dUltraTransitionSpeed = 2000;
						bIsSendTransitionPoint = false;
						int nLastTransitionPointIndex = 1;
						vector<MP_USR_VAR_INFO> vtVarInfo, vtVarInfoBp;
						//过渡点不大于十个
						for (int i = 0; i < 10; i++)
						{
							if (E_TRANSITION_ARCOFF_FAST_POINT == m_pTraceModel->m_vvnTrackWarpPtType.at(nWarpNum).at(nTrackNo + i))
							{
								nLastTransitionPointIndex = i;


								vtVarInfo.push_back(pRobotDriver->PrepareValData(i + 1, m_pTraceModel->m_vvtTrackWarpCoors.at(nWarpNum).at(nTrackNo + nLastTransitionPointIndex)));
								vtVarInfo.push_back(pRobotDriver->PrepareValData(89, m_dUltraTransitionSpeed));

								lBPValue[0] = (long)(m_pTraceModel->m_vvtTrackWarpCoors.at(nWarpNum).at(nTrackNo + nLastTransitionPointIndex).dBX * 1000);
								lBPValue[1] = (long)(m_pTraceModel->m_vvtTrackWarpCoors.at(nWarpNum).at(nTrackNo + nLastTransitionPointIndex).dBY * 1000);
								lBPValue[2] = (long)(m_pTraceModel->m_vvtTrackWarpCoors.at(nWarpNum).at(nTrackNo + nLastTransitionPointIndex).dBZ * 1000);
								vtVarInfoBp.push_back(pRobotDriver->PrepareValData(i + 1, lBPValue));
							}
							else
							{
								lBPValue[0] = (long)(m_pTraceModel->m_vvtTrackWarpCoors.at(nWarpNum).at(nTrackNo + nLastTransitionPointIndex).dBX * 1000);
								lBPValue[1] = (long)(m_pTraceModel->m_vvtTrackWarpCoors.at(nWarpNum).at(nTrackNo + nLastTransitionPointIndex).dBY * 1000);
								lBPValue[2] = (long)(m_pTraceModel->m_vvtTrackWarpCoors.at(nWarpNum).at(nTrackNo + nLastTransitionPointIndex).dBZ * 1000);
								vtVarInfo.push_back(pRobotDriver->PrepareValData(i + 1, m_pTraceModel->m_vvtTrackWarpCoors.at(nWarpNum).at(nTrackNo + nLastTransitionPointIndex)));
								vtVarInfo.push_back(pRobotDriver->PrepareValData(89, m_dUltraTransitionSpeed));
								vtVarInfoBp.push_back(pRobotDriver->PrepareValData(i + 1, lBPValue));
							}
						}
						vtVarInfo[nLastTransitionPointIndex * 2 + 1].val.i = m_dUltraTransitionSpeed / 8;
						if (pRobotDriver->m_vtWeldLineInWorldPoints.size() < 1)
						{
							pRobotDriver->HoldOn();
							//pRobotDriver->RobotEmg();
							XiMessageBoxOk("跳枪数据发送失败！焊接数据为空！");
							return false;
						}
						nPointNoAfterJump = pRobotDriver->m_vtWeldLineInWorldPoints.size() - 2;
						vtVarInfo.push_back(pRobotDriver->PrepareValData(7, pRobotDriver->m_vtWeldLineInWorldPoints.size() - 1));
						pRobotDriver->SetMultiVar_H(vtVarInfo); Sleep(50);
						pRobotDriver->SetMultiVar_H(vtVarInfoBp); Sleep(50);
						int nCountTmp = 0;
						while (pRobotDriver->GetIntVar(7) != pRobotDriver->m_vtWeldLineInWorldPoints.size() - 1/* - nWarpNum*/)
						{
							if (nCountTmp >= 100)
							{
								pRobotDriver->HoldOn();
								//pRobotDriver->RobotEmg();
								XiMessageBoxOk("跳枪数据发送失败！未知错误");
								return false;
							}
							pRobotDriver->SetIntVar(7, pRobotDriver->m_vtWeldLineInWorldPoints.size() - 1);
							Sleep(10);
							nCountTmp++;
						}
						//发送过渡轨迹的最后一个点
						//pRobotDriver->m_vtWeldLineInWorldPoints.push_back(m_pTraceModel->m_vvtTrackWarpCoors.at(nWarpNum).at(nTrackNo + nLastTransitionPointIndex));
						//m_pTraceModel->m_vtWeldLinePointType.push_back(E_TRANSITION_ARCOFF_FAST_POINT);
					}
					//第二面焊接数据
					if (E_WELD_TRACK == m_pTraceModel->m_vvnTrackWarpPtType.at(nWarpNum).at(nTrackNo))// 二次初始段段过滤波
					{
						bNeedOpenTrack = true;
						vtStartTrackCoors.insert(vtStartTrackCoors.end(),
							m_pTraceModel->m_vvtTrackWarpCoors.at(nWarpNum).begin() + nTrackNo, m_pTraceModel->m_vvtTrackWarpCoors.at(nWarpNum).end());
						m_pScanInit->StartDataPointPro(pRobotDriver, vtStartTrackCoors, true, m_pTraceModel->m_dWeldLen/*,false*/);
						for (nIndx = 1; nIndx < vtStartTrackCoors.size(); nIndx++)
						{
							if (bIsTracking)
							{
								vtStartTrackCoors.at(nIndx).dRZ += dRobotWeldAdjustRZ;
							}
							SaveRobotCoor(m_pScanInit->m_pRecordTheoryPoint, vtStartTrackCoors.at(nIndx), E_WELD_TRACK);
							pRobotDriver->m_vtWeldLineInWorldPoints.push_back(vtStartTrackCoors.at(nIndx));
							m_pTraceModel->m_vtWeldLinePointType.push_back(E_WELD_TRACK);
						}
						break;
					}
				}

				pRobotDriver->m_cLog->Write("++：%d %d nDelPointNum:%d", nNextOpenTrackStep, pRobotDriver->m_vtWeldLineInWorldPoints.size(), nDelPointNum);
				//更新结尾数据
				nWarpNum++;
				if (!bNeedOpenTrack)// 二次一次包角包角设置真实熄弧结尾点
				{
					m_pTraceModel->m_tRealEndpointCoor = pRobotDriver->m_vtWeldLineInWorldPoints.at(pRobotDriver->m_vtWeldLineInWorldPoints.size() - 1);
					pRobotDriver->SetIntVar(2, pRobotDriver->m_vtWeldLineInWorldPoints.size() - 1 - nDelPointNum);
					Sleep(50);
				}
				else
				{
					//m_pTraceModel->m_tRealEndpointCoor = m_pTraceModel->m_vtRealEndpointCoor.at(nWarpNum);
					m_pTraceModel->m_tRealEndpointCoor = pRobotDriver->m_vtWeldLineInWorldPoints.at(pRobotDriver->m_vtWeldLineInWorldPoints.size() - 1);
				}
				SaveEndpointData(pRobotDriver, m_pTraceModel->m_tRealEndpointCoor, false);//终点
				// 结尾点类型赋值 0726
				/*if (m_pScanInit->m_pTraceModel->m_vtEndPointType[nWarpNum] > 0)
				{*/
					//m_pScanInit->m_pTraceModel->m_nCloseTrackingPos = 10;
				//}
			}
			else
			{
				pRobotDriver->SetIntVar(2, pRobotDriver->m_vtWeldLineInWorldPoints.size() - 1 - nDelPointNum); Sleep(50);
				SaveEndpointData(pRobotDriver, pRobotDriver->m_vtWeldLineInWorldPoints.at(pRobotDriver->m_vtWeldLineInWorldPoints.size() - 1 - nDelPointNum), false);//终点
				pRobotDriver->m_cLog->Write("设置结尾数据总数:%d", pRobotDriver->m_vtWeldLineInWorldPoints.size() - 1 - nDelPointNum);

				// 发送结尾数据点数，用于结束跟踪流程,合
				T_ROBOT_COORS tEndpoint = pRobotDriver->m_vtWeldLineInWorldPoints.at(pRobotDriver->m_vtWeldLineInWorldPoints.size() - 1 - nDelPointNum);
				m_pTraceModel->m_tRealEndpointCoor = tEndpoint;

			}
		}
		if (nCountInt1 >= m_pTraceModel->m_vtWeldLinePointType.size())
		{
			pRobotDriver->HoldOn();
			//pRobotDriver->RobotEmg();
			XiMessageBoxOk("未发送跟踪数据，机械臂急停");
			continue;
		}
		// 切换焊接和包角时电流电压
		if (nCountInt1 < m_pTraceModel->m_vtWeldLinePointType.size())
		{
			SwitchWeldCurrent(pRobotDriver, nCountInt1, m_pTraceModel->m_vtWeldLinePointType.at(nCountInt1), m_pTraceModel->m_tWeldParam, bChangeVoltage);
		}
		if (nindex < m_pTraceModel->m_vtWeldLinePointType.size())
		{
			// 切换焊接和包角时速度
			SwitchWeldSpeed(pRobotDriver, nindex, m_pTraceModel->m_vtWeldLinePointType.at(nindex),
				dTotalVal, m_dVelocityWarp, lRobotvel, bChangeSpeed);
		}

		if (nCountInt1 > (ncount - nRefer))
		{
			// 获取当前松下轴和机器人位置,临时更改，后添加获取指定外部轴函数
			dRecordMachinePos = m_ptUnit->GetExPositionDis(m_ptUnit->m_nTrackAxisNo);

			dRecordRobotPosY = tRecordCurrentPos.dY;
			dRecordRobotPosX = tRecordCurrentPos.dX;
			dRecordRobotPosZ = tRecordCurrentPos.dZ;
			bool bAddVel = false;
			long lAddSpeed = 0;
			if ((TRUE == m_pTraceModel->m_bIfOpenExternal) && (nCountInt1 > 1) && (nProMoveNextNo != nCountInt1)
				&& !m_ptUnit->GetExAxisType(m_ptUnit->m_nTrackAxisNo))
			{
				for (int n = nProMoveNextNo; n < nCountInt1; n++) // 防止机器人人运动多份后才检测到 漏掉MoveNext调用
				{
					lAddSpeed = MoveHandle_MoveNext(m_ptUnit->m_nRobotSmooth,
						(long)((dRecordMachinePos/* + dRecordRobotPosY*/) / dPulseToDisX),
						dRecordRobotPosX, dRecordRobotPosZ, 4, 5);
					pRobotDriver->m_cLog->Write("%s MoveHandle_MoveNext lAddSpeed=%ld %11.3lf%11.3lf%11.3lf %11.3lf %d %d",
						strRobot, lAddSpeed, dRecordRobotPosX, dRecordRobotPosY, dRecordRobotPosZ, dRecordMachinePos, nProMoveNextNo, nCountInt1);
				}
				bAddVel = true;
				//pRobotDriver->SetIntVar(66, lAddSpeed);
				nProMoveNextNo = nCountInt1;
			}
			//static int nDelePointNum = 10;
			/*if (true == m_pTraceModel->m_bCameraFindWeldEnd&&(nDelePointNum==10))
			{
				pRobotDriver->m_vtWeldLineInWorldPoints.erase(pRobotDriver->m_vtWeldLineInWorldPoints.end() - nDelePointNum, pRobotDriver->m_vtWeldLineInWorldPoints.end());
				nDelePointNum = 0;
			}*/

			nWeldLinePtnNum = pRobotDriver->m_vtWeldLineInWorldPoints.size();
			if (false == m_pTraceModel->m_bCameraFindWeldEnd) // 未发现终点时保留包角长度轨迹不发送
			{
				nWeldLinePtnNum = pRobotDriver->m_vtWeldLineInWorldPoints.size();
				double dWrapLen = 20.0;
				T_ROBOT_COORS tLastCoord = pRobotDriver->m_vtWeldLineInWorldPoints.back();
				for (int i = nWeldLinePtnNum - 2; i > 0; i--)
				{
					T_ROBOT_COORS tFindCoord = pRobotDriver->m_vtWeldLineInWorldPoints[i];
					double dDis = TwoPointDis(
						tLastCoord.dX + tLastCoord.dBX, tLastCoord.dY + tLastCoord.dBY, tLastCoord.dZ + tLastCoord.dBZ,
						tFindCoord.dX + tFindCoord.dBX, tFindCoord.dY + tFindCoord.dBY, tFindCoord.dZ + tFindCoord.dBZ);
					if (dWrapLen < dDis)
					{
						nWeldLinePtnNum = i + 1;
						//pRobotDriver->m_cLog->Write(" 未发现终点时保留包角长度轨迹不发送i;%d nWeldLinePtnNum:%d", i, nWeldLinePtnNum);
						break;
					}
					else
					{
						//pRobotDriver->m_cLog->Write(" 22未发现终点时保留包角长度轨迹不发送i;%d nWeldLinePtnNum:%d", i, nWeldLinePtnNum);
						nWeldLinePtnNum = 0;
					}
				}
			}

			//pRobotDriver->m_cLog->Write("nWeldLinePtnNum:%d", nWeldLinePtnNum);
			if (nindex < nWeldLinePtnNum)
			{
				vtVarInfo.clear();
				dStartTim = XI_clock();
				time2 = XI_clock();
				pRobotDriver->m_cLog->Write("%s i变量：%d nindex:%d 轨迹size:%d",
					strRobot, nCountInt1, nindex, pRobotDriver->m_vtWeldLineInWorldPoints.size());
				pRobotDriver->m_cLog->Write("%s 原始数据:%d %20.10lf %20.10lf %20.10lf %20.10lf %20.10lf %20.10lf", strRobot, ncount,
					pRobotDriver->m_vtWeldLineInWorldPoints[nindex].dX, pRobotDriver->m_vtWeldLineInWorldPoints[nindex].dY,
					pRobotDriver->m_vtWeldLineInWorldPoints[nindex].dZ, pRobotDriver->m_vtWeldLineInWorldPoints[nindex].dBX,
					pRobotDriver->m_vtWeldLineInWorldPoints[nindex].dBY, pRobotDriver->m_vtWeldLineInWorldPoints[nindex].dRZ
				);
				if (m_pTraceModel->m_bIfOpenExternal && !m_ptUnit->GetExAxisType(m_ptUnit->m_nTrackAxisNo))
				{
					vlWeldPulseX.clear();
					vlWeldPulseY.clear();
					vdWeldStepVel.clear();
					// 获取截断数据
					if (!GetTruncatedData(pRobotDriver, pRobotDriver->m_vtWeldLineInWorldPoints[nindex], tRobot, lRobotvel, lMachine, MachVel))
					{
						nDelPointNum++;
						nindex++;
						continue;
					}
					vlWeldPulseX.push_back(lMachine);
					vlWeldPulseY.push_back(0);
					vdWeldStepVel.push_back((double)MachVel);

					tTempRobotCoor = pRobotDriver->m_vtWeldLineInWorldPoints[nindex];
#ifdef SINGLE_ROBOT
					tTempRobotCoor.dX = tRobot.x_ / 1000.;
#else	
					tTempRobotCoor.dY = tRobot.y_ / 1000.;
#endif					
					tTempRobotCoor.dZ = tRobot.z_ / 1000.;

					WORD awAxis[] = { WORD(m_pTraceModel->m_nAXIS_X), WORD(m_pTraceModel->m_nAXIS_Y) };
					vector <long> avlRelPulse[] = { vlWeldPulseX, vlWeldPulseY };
					if (TRUE == m_pTraceModel->m_cStepMoveControl.AdjustpvlRelPulseIsNULL())
					{
						//该函数仅在大车停止或程序运行结束时返回TRUE,若中间返回TRUE即为错误。若继续执行程序回崩溃
						pRobotDriver->m_cLog->Write("插入控制卡指针数组为空");
						//CheckMachineEmg(pRobotDriver);	//原先的先测后焊类里有，新版无 240124备注
						XiMessageBox("线程为空");
						break;
					}
					m_pTraceModel->m_cStepMoveControl.InsertData(avlRelPulse, vdWeldStepVel, 2);
					int nCurStepNo = m_pTraceModel->m_cStepMoveControl.GetCurrentMark();
					pRobotDriver->m_cLog->Write("%s nindex:%d CurMove:%d nCurStepNo:%d 焊接轨迹11:%11.3lf%11.3lf%11.3lf Coordin:%d",
						strRobot, nindex, nCountInt1, nCurStepNo, tRobot.x_ / 1000., tTempRobotCoor.dY, tRobot.z_ / 1000.,
						m_pTraceModel->m_cStepMoveControl.m_nCoordinate);
				}
				else
				{
					tTempRobotCoor = pRobotDriver->m_vtWeldLineInWorldPoints[nindex];
					lBPValue[0] = (long)(pRobotDriver->m_vtWeldLineInWorldPoints[nindex].dBX * 1000);
					lBPValue[1] = (long)(pRobotDriver->m_vtWeldLineInWorldPoints[nindex].dBY * 1000);
					lBPValue[2] = (long)(pRobotDriver->m_vtWeldLineInWorldPoints[nindex].dBZ * 1000);
					vtVarInfo.push_back(pRobotDriver->PrepareValData((ncount % nPointNum + 20), lBPValue));
				}
				//焊接倾角 焊接姿态
				/*tTempRobotCoor.dRX = m_dNormalWeldRx;
				tTempRobotCoor.dRY = m_dNormalWeldRy;*/
				double dDipAngle = 0;
				if (!bIsTracking)//不跟踪时所有点位都需要添加焊接倾角
				{
					tTempRobotCoor.dRZ += dRobotWeldAdjustRZ;
				}
				else//跟踪时焊接倾角，在获取跟踪数据时已经添加
				{
					dDipAngle = dRobotWeldAdjustRZ;
				}

				T_ROBOT_COORS tChangeRZRobot;
				tChangeRZRobot.dRZ = m_pTraceModel->m_dAdjustRobotRZ + dDipAngle;
				dRecordAdjustRZ = m_pTraceModel->m_dAdjustRobotRZ;
				nErrorAdjustRz = 0;
				//jwq临时改跳枪后角度偏移量清空
				if (nPointNoAfterJump >= 0 && nCountInt1 >= nPointNoAfterJump)
				{
					m_pTraceModel->m_dAdjustRobotRZ = 0.0;
					nPointNoAfterJump = -1;
				}
				//姿态				
				vtVarInfo.push_back(pRobotDriver->PrepareValData((ncount % nPointNum + 20), tTempRobotCoor));
				vtVarInfo.push_back(pRobotDriver->PrepareValData(99, tChangeRZRobot));
				vtVarInfo.push_back(pRobotDriver->PrepareValData((ncount % nPointNum + 16), lRobotvel));
				if (bAddVel)
				{
					vtVarInfo.push_back(pRobotDriver->PrepareValData(66, lAddSpeed));
				}
				pRobotDriver->SetMultiVar_H(vtVarInfo);

				pRobotDriver->m_cLog->Write("单词发送P变量时间:%.3lf", (double)(XI_clock() - dStartTim) / CLOCKS_PER_SEC);
				ncount++;
				nindex++;
			}
			else
			{
				if (bAddVel)
				{
					vtVarInfo.clear();
					vtVarInfo.push_back(pRobotDriver->PrepareValData(66, lAddSpeed));
					pRobotDriver->SetMultiVar_H(vtVarInfo);
				}
			}
		}

		if (false == m_ptUnit->RobotCheckRunning()) // 运动停止就退出
		{
			m_pTraceModel->m_bIfWeldToEndPoint = true;
			break;
		}

		Sleep(20);
		DoEvent();
	}
	RecordRunTime(pRobotDriver, tTime, "焊接");
	fclose(RecordRealData);
	//记录焊接长度和当前运行步号
	SaveWeldLengthData(pRobotDriver, m_pTraceModel->m_dCurrentWeldLen, dRecordAdjustRZ, nCountInt1 + m_pTraceModel->m_nWeldStepNo);
	SaveCurrentWarpNumber(pRobotDriver, nWarpNum);
	//获取滤波缓存数据
	vector<TrackFilter_XYZ> vtBufferData = TrackFilter_GetCachePntsPlus(m_ptUnit->m_nRobotSmooth);
	CString strFile, strFile2;
	strFile.Format("%s滤波缓存数据.txt", strCoutPath);
	SaveDataTrackFilt(vtBufferData, strFile.GetBuffer(0));
	strFile2.Format("%sBreakPointReferencePulse.txt", strCoutPath);
	SaveDataRobotPulse(GetCurrentPulse(pRobotDriver), strFile2.GetBuffer(0));//存储端点时脉冲当做下枪参考坐标
	pRobotDriver->m_cLog->Write("%s 赋值线程退出!", strRobot);
	m_ptUnit->RobotCheckDone();
	pRobotDriver->HoldOff();
	m_pTraceModel->m_cStepMoveControl.EmgStop();
	m_pTraceModel->m_cStepMoveControl.StopStep();
	fclose(m_pScanInit->m_pRecordTheoryPoint);
	T_ROBOT_COORS tEndSafe2 = pRobotDriver->GetCurrentPos();
	RobotCoordPosOffset(tEndSafe2, tEndSafe2.dRZ, 50., 200. * m_nRobotInstallDir);
	//tEndSafe2.dZ = m_dWorkPieceHighTop + 20 * m_nRobotInstallDir;
	T_ROBOT_MOVE_SPEED tPulseMove(m_dFastApproachToWorkSpeed, 20, 20);
	vector<T_ROBOT_MOVE_INFO> vtRobotMoveInfo(0);
	vtRobotMoveInfo.push_back(pRobotDriver->PVarToRobotMoveInfo(0, tEndSafe2, tPulseMove, MOVL));
	pRobotDriver->SetMoveValue(vtRobotMoveInfo);
	pRobotDriver->CallJob("CONTIMOVANY");
	CHECK_BOOL_RETURN(m_ptUnit->CheckRobotDone(tEndSafe2));

	//ZHELI
	return true;
}

bool CDiaphragmWeld::TrackingWeldMoveForPanasonic(CRobotDriverAdaptor* pRobotDriver, vector<T_ROBOT_COORS> vtRealWeldCoors, vector<int> vnPointType, bool bIsTracking, E_WELD_SEAM_TYPE eWeldSeamType)
{
	if (g_bLocalDebugMark) return true;

	GetNaturalPop().popInfo("开始焊接运动");

	long long tTime = XI_clock();
	double dRobotWeldAdjustRZ = m_dWeldDipAngle;//根据工艺需要更改Rz角度
	int nFirstNum = pRobotDriver->m_vtWeldLineInWorldPoints.size();
	CString strRobot = pRobotDriver->m_strRobotName, strCoutPath;
	// 输出文件路径
	strCoutPath.Format("%s%s%s", OUTPUT_PATH, pRobotDriver->m_strRobotName, RECOGNITION_FOLDER);
	long long dStartTim = XI_clock();
	m_pTraceModel->m_bIfWeldToEndPoint = false;
	m_pTraceModel->m_bCameraFindWeldEnd = false;
	m_pTraceModel->m_bSendWeldDataFinish = false;
	m_pTraceModel->m_bSendSafeFlag = false;
	m_pTraceModel->m_bIsCloseTrackThread = false;
	m_pScanInit->m_bOpenTrackingStatus = false;
	m_pTraceModel->m_bIfOpenExternal = true;
	m_pTraceModel->m_bIfJudgeEndOpen = false;
	//初始机械臂Y坐标
	int ncount = 0;
	int nindex = 0;
	int nDelPointNum = 0;// 删除重复点
	int nCountInt1 = 0;
	int nPointNum = 40;// 缓存点数
	pRobotDriver->SetIntVar(8, 0);
	if (bIsTracking) {
		pRobotDriver->SetIntVar(2, 9999);
	}
	else {
		pRobotDriver->SetIntVar(2, nFirstNum - 1);
	}
	//初始化跳枪起终序号
	bool bChangeSpeed = false; // 改变速度
	pRobotDriver->SetIntVar(7, -1);
	pRobotDriver->SetIntVar(6, 0);	// 焊接过程停弧 过渡
	pRobotDriver->SetIntVar(1, 0);
	pRobotDriver->SetIntVar(1, 0);
	double dTotalVal = m_pTraceModel->m_dWeldVelocity;
	long long time2 = XI_clock();
	long lBPValue[3];
	T_ROBOT_COORS tTempRobotCoor;
	MoveHandle_Pnt tRobot;
	vector<long> vlWeldPulseX, vlWeldPulseY;
	vector<double> vdWeldStepVel;
	vector<MP_USR_VAR_INFO> vtVarInfo;
	long lRobotvel = dTotalVal / 6;
	long lMachine;
	long MachVel;
	vtVarInfo.clear();
	vlWeldPulseX.clear();
	vlWeldPulseY.clear();
	vdWeldStepVel.clear();
	FILE* RecordRealData;
	if (m_ptUnit->m_bBreakPointContinue)
	{
		RecordRealData = fopen(strCoutPath + "实际焊接数据.txt", "a+");
	}
	else
	{
		RecordRealData = fopen(strCoutPath + "实际焊接数据.txt", "w");
	}

	m_pScanInit->m_pRecordTheoryPoint = fopen(strCoutPath + "理论焊接数据.txt", "w"); Sleep(50);
	if (m_ptUnit->m_bBreakPointContinue)
	{
		SaveRobotCoor(m_pScanInit->m_pRecordTheoryPoint, m_pTraceModel->m_vtBreakContinueBeforPoints, m_pTraceModel->m_vtBreakContinueBeforPointType);//保存断续前理论数据
	}
	SaveRobotCoor(m_pScanInit->m_pRecordTheoryPoint, pRobotDriver->m_vtWeldLineInWorldPoints, m_pTraceModel->m_vtWeldLinePointType);//保存理论数据

	// 安川松下插补算法初始化
	InitMoveHandleYaskawaPanasonic(pRobotDriver->m_vtWeldLineInWorldPoints[0], m_pTraceModel->m_dWeldVelocity);

	while (false == m_pTraceModel->m_bIfWeldToEndPoint)
	{
		if (ncount < nFirstNum)
		{
			if (ncount < nPointNum)
			{
				pRobotDriver->m_cLog->Write("%s 原始数据:%d %20.10lf %20.10lf %20.10lf %20.10lf %20.10lf %20.10lf", strRobot, ncount,
					pRobotDriver->m_vtWeldLineInWorldPoints[nindex].dX, pRobotDriver->m_vtWeldLineInWorldPoints[nindex].dY,
					pRobotDriver->m_vtWeldLineInWorldPoints[nindex].dZ, pRobotDriver->m_vtWeldLineInWorldPoints[nindex].dBX,
					pRobotDriver->m_vtWeldLineInWorldPoints[nindex].dBY, pRobotDriver->m_vtWeldLineInWorldPoints[nindex].dRZ
				);

				// 计算焊接和包角时速度
				SwitchWeldSpeed(pRobotDriver, nindex, m_pTraceModel->m_vtWeldLinePointType.at(nindex),
					dTotalVal, m_dVelocityWarp, lRobotvel, bChangeSpeed);

				// 获取截断数据
				GetTruncatedData(pRobotDriver, pRobotDriver->m_vtWeldLineInWorldPoints[nindex], tRobot, lRobotvel, lMachine, MachVel);
				vlWeldPulseX.push_back(lMachine);
				vlWeldPulseY.push_back(0);
				vdWeldStepVel.push_back((double)MachVel);

				tTempRobotCoor = pRobotDriver->m_vtWeldLineInWorldPoints[nindex];
#ifdef SINGLE_ROBOT
				tTempRobotCoor.dX = tRobot.x_ / 1000.;
#else	
				tTempRobotCoor.dY = tRobot.y_ / 1000.;
#endif			
				tTempRobotCoor.dZ = tRobot.z_ / 1000.;

				pRobotDriver->m_cLog->Write("%s ncount:%d,初次焊接轨迹11:%11.20lf%20.10lf%20.10lf RZ:%lf 基坐标：%20.10lf",
					strRobot, ncount, tRobot.x_ / 1000., tTempRobotCoor.dY, tRobot.z_ / 1000.,
					pRobotDriver->m_vtWeldLineInWorldPoints[nindex].dRZ, pRobotDriver->m_vtWeldLineInWorldPoints[nindex].dBY);

				vtVarInfo.push_back(pRobotDriver->PrepareValData((ncount % nPointNum + 20), tTempRobotCoor));
				vtVarInfo.push_back(pRobotDriver->PrepareValData((ncount % nPointNum + 16), lRobotvel));

				if (ncount == nPointNum - 1 || ncount == (nFirstNum - 1))
				{
					pRobotDriver->SetMultiVar_H(vtVarInfo);
					pRobotDriver->HoldOff(); Sleep(100);
					pRobotDriver->ServoOn(); Sleep(100);

					pRobotDriver->CallJob("CIRCULATIONWELD");
					Sleep(250);
					if (0 == vdWeldStepVel.at(0))
					{
						vdWeldStepVel.at(0) = vdWeldStepVel.at(1);
					}
					m_pTraceModel->m_cStepMoveControl.m_bInsterBuffer = true;
					//轴号
					WORD awAxis[] = { WORD(m_pTraceModel->m_nAXIS_X), WORD(m_pTraceModel->m_nAXIS_Y) };
					//实际运行脉冲向量
					vector <long> avlRelPulse[] = { vlWeldPulseX,vlWeldPulseY };
					m_pTraceModel->m_cStepMoveControl.SetStartStopStepNum(0);
					m_pTraceModel->m_cStepMoveControl.SetBuffNum(2);
					m_pTraceModel->m_cStepMoveControl.SetStartStepNo(0);
					m_pTraceModel->m_cStepMoveControl.StepMove(awAxis, avlRelPulse, vdWeldStepVel, 2, 1);
				}
				ncount++;
				nindex++;
				if (ncount == (nFirstNum))
				{
					WriteLog("%s 数据小于19个直接运行job:%d", strRobot, nFirstNum);
					break;
				}
			}
			else
			{
				break;
			}
		}
		Sleep(5);
		DoEvent();
	}
	int nWarpNum = 0;// 包角次数
	int nNextOpenTrackStep = 9999, nRecordNextOpenTrackStep = 0;;// 再次开启跟踪步号
	bool bChangeVoltage = false;// 改变电流电压
	double dRecordAdjustRZ = m_pTraceModel->m_dRecordBreakPointRZ;//记录RZ变化量
	int nRefer = ncount;
	int nProMoveNextNo = 1;
	double dRecordMachinePos;// = m_dMachineAxis_X; // 及时记录机器人走过一份时大车位置
	double dRecordRobotPosY;// = m_dRobotAxis_Y; // 及时记录机器人走过一份时机器人Y值
	double dRecordRobotPosX;
	double dRecordRobotPosZ;
	m_pTraceModel->m_dAdjustRobotRZ = m_pTraceModel->m_dRecordBreakPointRZ;
	int    nErrorAdjustRz = 0;
	int    nWeldLinePtnNum = nFirstNum;
	pRobotDriver->m_cLog->Write("初始I1变量：%d", pRobotDriver->GetIntVar(1));
	int nSave = 0;
	m_pTraceModel->m_vvtTrackWarpCoors.clear();
	m_pTraceModel->m_vvnTrackWarpPtType.clear();
	if (m_ptUnit->m_bBreakPointContinue)
	{
		m_pScanInit->LoadIsTrackCompleteFlag(pRobotDriver, m_pTraceModel->m_bCameraFindWeldEnd);
		LoadCurrentWarpNumber(pRobotDriver, nWarpNum);
	}
	int nPointNoAfterJump = -1;//跳枪后轨迹序号
	while (false == m_pTraceModel->m_bIfWeldToEndPoint)
	{
		CHECK_ROBOT_EMG_BREAK(m_ptUnit);
		T_ROBOT_COORS tRecordCurrentPos = pRobotDriver->GetCurrentPos();
		if (0 == nSave % 10) {
			SaveRobotCoor(RecordRealData, tRecordCurrentPos);//保存实际位置数据
		}
		nSave++;
		if (TRUE == m_pTraceModel->m_bIfWeldToEndPoint) {
			pRobotDriver->m_cLog->Write("%s 赋值线程 break!", strRobot);
			break;
		}
		if (nCountInt1 == nWeldLinePtnNum - 1 && !bIsTracking) {
			pRobotDriver->m_cLog->Write("%s 赋值线程退出break:%d!", strRobot, nWeldLinePtnNum);
			pRobotDriver->SetIntVar(3, 6666);
			break;
		}
		nCountInt1 = pRobotDriver->GetIntVar(1);
		//开启跟踪
		bool bIsOpenTrackAgain = false;
		E_MEASURE_POINT_TYPE tPtType = E_WELD_TRACK;
		if (nCountInt1 < m_pTraceModel->m_vtWeldLinePointType.size())
		{
			tPtType = (E_MEASURE_POINT_TYPE)m_pTraceModel->m_vtWeldLinePointType.at(nCountInt1);
			//pRobotDriver->m_cLog->Write("开启跟踪线程:tPtType22:%d nCountInt1;%d m_vtWeldLinePointType;%d", tPtType, nCountInt1, m_pTraceModel->m_vtWeldLinePointType.size());
			if (E_WELD_OPEN_TRACKING == tPtType &&
				nNextOpenTrackStep != nCountInt1)
			{
				nNextOpenTrackStep = nCountInt1;
				bIsOpenTrackAgain = true;
				pRobotDriver->m_cLog->Write("开启跟踪线程:nCountInt22:%d", nCountInt1);
			}
		}
		// 单次跟踪过程只进一次跟踪线程，开启跟踪条件：初次起弧后进入正常焊接姿态或包角结束后再次进入跟踪姿态
		if (bIsTracking && (E_WELD_TRACK & tPtType) && !(E_WELD_TRACK_CHANGE_POSTURE & tPtType) &&
			!m_pScanInit->m_bOpenTrackingStatus &&
			!m_pTraceModel->m_bCameraFindWeldEnd ||
			bIsOpenTrackAgain) {
			m_pScanInit->m_bOpenTrackingStatus = true;
			m_pTraceModel->m_bCameraFindWeldEnd = FALSE;
			//------------------------------------
			m_pTraceModel->m_bIfWeldToEndPoint = false; // !!!!!!!!
			m_pTraceModel->m_bSendWeldDataFinish = false;
			m_pTraceModel->m_bSendSafeFlag = false;
			m_pTraceModel->m_bIsCloseTrackThread = false;

			m_ptUnit->SwitchIO("TrackLaser", true); Sleep(50); //m_pIOControl->OpenLeftRedALaser(); 

			//---------------------------------------
			E_JUDGE_END_METHOD eJudgeEndMethod = nWarpNum < 2 ? E_JUDGE_END_PROCESS_SEARCH : E_JUDGE_END_KNOW;
			T_ROBOT_COORS* pEndCoord = nWarpNum < 2 ? NULL : &pRobotDriver->m_vtWeldLineInWorldPoints[0];
			m_pScanInit->InitRealTimeTracking(eJudgeEndMethod, pEndCoord); // 点云方式跟踪初始化

			m_pScanInit->DynamicRealTimeTracking();
			pRobotDriver->m_cLog->Write("开启跟踪线程:nCountInt1:%d", nCountInt1);
		}
		// 发送包角数据
		int nPointCountBeforeJump = -1;//跳枪前轨迹总数
		int nPointCountAfterJump = -1;//跳枪后轨迹总数
		if (m_pTraceModel->m_bIsTrackingDataSendOver)
		{
			nPointCountBeforeJump = pRobotDriver->m_vtWeldLineInWorldPoints.size();
			m_pTraceModel->m_bIsTrackingDataSendOver = false;
			pRobotDriver->m_cLog->Write("发送包角数据：%d", pRobotDriver->m_vtWeldLineInWorldPoints.size());
			m_ptUnit->m_bBreakPointContinue = false;
			// 计算跳枪包角轨迹
			vector<T_ROBOT_COORS> vtWarpCoors(0);
			vector<int> vtWarpPtType(0);
			E_WRAPANGLE_PARAM tWarpParam = LoadWarpParam(m_ptUnit->GetRobotCtrl()->m_strRobotName, E_WRAPANGLE_ONCE, nWarpNum);


			// 过程搜终点一次包角：删除 pRobotDriver->m_vtWeldLineInWorldPoints 中结束段包角轨迹
			if (nWarpNum < 2 && m_pTraceModel->m_eStartWrapAngleType == E_WRAPANGLE_ONCE) // 需要一次包角
			{
				T_ROBOT_COORS tLastCoord = pRobotDriver->m_vtWeldLineInWorldPoints.back();
				for (int i = pRobotDriver->m_vtWeldLineInWorldPoints.size() - 1; i > 0; i--)
				{
					T_ROBOT_COORS tCurFindCoord = pRobotDriver->m_vtWeldLineInWorldPoints[i];
					double dDis = TwoPointDis(tCurFindCoord.dX + tCurFindCoord.dBX, tCurFindCoord.dY + tCurFindCoord.dBY, tCurFindCoord.dZ + tCurFindCoord.dBZ,
						tLastCoord.dX + tLastCoord.dBX, tLastCoord.dY + tLastCoord.dBY, tLastCoord.dZ + tLastCoord.dBZ);
					if (dDis < tWarpParam.dWarpLength)
					{
						pRobotDriver->m_vtWeldLineInWorldPoints.pop_back();
						m_pTraceModel->m_vtWeldLinePointType.pop_back();
					}
					else
					{
						break;
					}
				}
			}


			if (!CalcOnceWarpTrack(nWarpNum, m_pScanInit->m_pTraceModel->dBoardThink, m_pTraceModel->m_dGunToEyeCompenX, m_pScanInit->m_pTraceModel->m_dFlatHorComp,
				pRobotDriver->m_vtWeldLineInWorldPoints, m_dWorkPieceHighTop, vtWarpCoors, vtWarpPtType, tWarpParam))
			{
				pRobotDriver->HoldOn();
				XiMessageBoxOk("一次包角数据计算失败！");
				return false;
			}

			nPointCountAfterJump = nPointCountBeforeJump + vtWarpCoors.size();
			m_pTraceModel->m_vvtTrackWarpCoors.push_back(vtWarpCoors);
			m_pTraceModel->m_vvnTrackWarpPtType.push_back(vtWarpPtType);
			CString strFile;
			strFile.Format("A_Test_%d.txt", nWarpNum);
			FILE* pf = fopen(strFile, "w");
			SaveRobotCoor(pf, vtWarpCoors, vtWarpPtType);
			fclose(pf);
			strFile.Format("A_Test2_%d.txt", nWarpNum);
			FILE* pf2 = fopen(strFile, "w");
			SaveRobotCoor(pf2, pRobotDriver->m_vtWeldLineInWorldPoints, m_pTraceModel->m_vtWeldLinePointType);
			fclose(pf2);
			// ---------------------------------------------------------------
//			m_pTraceModel->m_vtRealEndpointCoor.size() - 1;
			if (nWarpNum < 2 &&	m_pTraceModel->m_eStartWrapAngleType == E_WRAPANGLE_ONCE)
			{
				bool bNeedOpenTrack = false;
				bool bIsSendTransitionPoint = true;// = m_pUnWarpBoundWeld > 0 ? true : false;	//从一号变量开始发 1-10为过渡点
				int nTrackNo, nIndx;
				vector<T_ROBOT_COORS> vtStartTrackCoors;
				for (nTrackNo = 0; nTrackNo < m_pTraceModel->m_vvtTrackWarpCoors.at(nWarpNum).size(); nTrackNo++) {
					//跟踪包角数据 补偿
					T_ROBOT_COORS tCoors = m_pTraceModel->m_vvtTrackWarpCoors.at(nWarpNum).at(nTrackNo);
					if (E_WELD_TRACK != m_pTraceModel->m_vvnTrackWarpPtType.at(nWarpNum).at(nTrackNo) &&
						E_TRANSITION_ARCOFF_FAST_POINT != m_pTraceModel->m_vvnTrackWarpPtType.at(nWarpNum).at(nTrackNo))
					{
						if (E_WELD_OPEN_TRACKING == m_pTraceModel->m_vvnTrackWarpPtType.at(nWarpNum).at(nTrackNo))
						{
							CalcMeasureInBase(m_pTraceModel->m_vvtTrackWarpCoors.at(nWarpNum).at(nTrackNo),
								m_pTraceModel->m_dGunToEyeCompenX, m_pTraceModel->m_dGunToEyeCompenZ, tCoors);
						}
						if (bIsTracking)
						{
							tCoors.dRZ += dRobotWeldAdjustRZ;//根据工艺需要更改Rz角度
						}
						SaveRobotCoor(m_pScanInit->m_pRecordTheoryPoint, tCoors, m_pTraceModel->m_vvnTrackWarpPtType.at(nWarpNum).at(nTrackNo));
						pRobotDriver->m_vtWeldLineInWorldPoints.push_back(tCoors);
						m_pTraceModel->m_vtWeldLinePointType.push_back(m_pTraceModel->m_vvnTrackWarpPtType.at(nWarpNum).at(nTrackNo));
					}

					//第二面焊接数据
					if (E_WELD_TRACK == m_pTraceModel->m_vvnTrackWarpPtType.at(nWarpNum).at(nTrackNo))// 二次初始段段过滤波
					{
						bNeedOpenTrack = true;
						vtStartTrackCoors.insert(vtStartTrackCoors.end(),
							m_pTraceModel->m_vvtTrackWarpCoors.at(nWarpNum).begin() + nTrackNo, m_pTraceModel->m_vvtTrackWarpCoors.at(nWarpNum).end());
						m_pScanInit->StartDataPointPro(pRobotDriver, vtStartTrackCoors, true, m_pTraceModel->m_dWeldLen/*,false*/);
						for (nIndx = 1; nIndx < vtStartTrackCoors.size(); nIndx++)
						{
							if (bIsTracking)
							{
								vtStartTrackCoors.at(nIndx).dRZ += dRobotWeldAdjustRZ;
							}
							SaveRobotCoor(m_pScanInit->m_pRecordTheoryPoint, vtStartTrackCoors.at(nIndx), E_WELD_TRACK);
							pRobotDriver->m_vtWeldLineInWorldPoints.push_back(vtStartTrackCoors.at(nIndx));
							m_pTraceModel->m_vtWeldLinePointType.push_back(E_WELD_TRACK);
						}
						break;
					}
				}

				pRobotDriver->m_cLog->Write("++：%d %d nDelPointNum:%d", nNextOpenTrackStep, pRobotDriver->m_vtWeldLineInWorldPoints.size(), nDelPointNum);
				//更新结尾数据
				nWarpNum++;
				if (!bNeedOpenTrack)// 二次一次包角包角设置真实熄弧结尾点
				{
					m_pTraceModel->m_tRealEndpointCoor = pRobotDriver->m_vtWeldLineInWorldPoints.at(pRobotDriver->m_vtWeldLineInWorldPoints.size() - 1);
					pRobotDriver->SetIntVar(2, pRobotDriver->m_vtWeldLineInWorldPoints.size() - 1 - nDelPointNum);
					Sleep(50);
				}
				else
				{
					//m_pTraceModel->m_tRealEndpointCoor = m_pTraceModel->m_vtRealEndpointCoor.at(nWarpNum);
					m_pTraceModel->m_tRealEndpointCoor = pRobotDriver->m_vtWeldLineInWorldPoints.at(pRobotDriver->m_vtWeldLineInWorldPoints.size() - 1);
				}
				SaveEndpointData(pRobotDriver, m_pTraceModel->m_tRealEndpointCoor, false);//终点
			}
			else
			{
				pRobotDriver->SetIntVar(2, pRobotDriver->m_vtWeldLineInWorldPoints.size() - 1 - nDelPointNum); Sleep(50);
				SaveEndpointData(pRobotDriver, pRobotDriver->m_vtWeldLineInWorldPoints.at(pRobotDriver->m_vtWeldLineInWorldPoints.size() - 1 - nDelPointNum), false);//终点
				pRobotDriver->m_cLog->Write("设置结尾数据总数:%d", pRobotDriver->m_vtWeldLineInWorldPoints.size() - 1 - nDelPointNum);

				// 发送结尾数据点数，用于结束跟踪流程,合
				T_ROBOT_COORS tEndpoint = pRobotDriver->m_vtWeldLineInWorldPoints.at(pRobotDriver->m_vtWeldLineInWorldPoints.size() - 1 - nDelPointNum);
				m_pTraceModel->m_tRealEndpointCoor = tEndpoint;

			}
		}
		if (nCountInt1 >= m_pTraceModel->m_vtWeldLinePointType.size())
		{
			pRobotDriver->HoldOn();
			XiMessageBoxOk("未发送跟踪数据，机械臂急停");
			continue;
		}
		// 切换焊接和包角时电流电压
		if (nCountInt1 < m_pTraceModel->m_vtWeldLinePointType.size())
		{
			SwitchWeldCurrent(pRobotDriver, nCountInt1, m_pTraceModel->m_vtWeldLinePointType.at(nCountInt1), m_pTraceModel->m_tWeldParam, bChangeVoltage);
		}
		if (nindex < m_pTraceModel->m_vtWeldLinePointType.size())
		{
			// 切换焊接和包角时速度
			SwitchWeldSpeed(pRobotDriver, nindex, m_pTraceModel->m_vtWeldLinePointType.at(nindex),
				dTotalVal, m_dVelocityWarp, lRobotvel, bChangeSpeed);
		}

		if (nCountInt1 > (ncount - nRefer))
		{
			// 获取当前松下轴和机器人位置,临时更改，后添加获取指定外部轴函数
			dRecordMachinePos = m_ptUnit->GetExPositionDis(m_ptUnit->m_nTrackAxisNo);

			dRecordRobotPosY = tRecordCurrentPos.dY;
			dRecordRobotPosX = tRecordCurrentPos.dX;
			dRecordRobotPosZ = tRecordCurrentPos.dZ;
			bool bAddVel = false;
			long lAddSpeed = 0;
			if ((nCountInt1 > 1) && (nProMoveNextNo != nCountInt1) && !m_ptUnit->GetExAxisType(m_ptUnit->m_nTrackAxisNo))
			{
				for (int n = nProMoveNextNo; n < nCountInt1; n++) // 防止机器人人运动多份后才检测到 漏掉MoveNext调用
				{
					lAddSpeed = MoveHandle_MoveNext(m_ptUnit->m_nRobotSmooth,
						(long)((dRecordMachinePos) / m_ptUnit->GetExPulseEquivalent(m_ptUnit->m_nTrackAxisNo)),
						dRecordRobotPosX, dRecordRobotPosZ, 4, 5);
					pRobotDriver->m_cLog->Write("%s MoveHandle_MoveNext lAddSpeed=%ld %11.3lf%11.3lf%11.3lf %11.3lf %d %d",
						strRobot, lAddSpeed, dRecordRobotPosX, dRecordRobotPosY, dRecordRobotPosZ, dRecordMachinePos, nProMoveNextNo, nCountInt1);
				}
				bAddVel = true;
				nProMoveNextNo = nCountInt1;
			}

			nWeldLinePtnNum = pRobotDriver->m_vtWeldLineInWorldPoints.size();
			if (false == m_pTraceModel->m_bCameraFindWeldEnd) // 未发现终点时保留包角长度轨迹不发送
			{
				nWeldLinePtnNum = pRobotDriver->m_vtWeldLineInWorldPoints.size();
				double dWrapLen = 20.0;
				T_ROBOT_COORS tLastCoord = pRobotDriver->m_vtWeldLineInWorldPoints.back();
				for (int i = nWeldLinePtnNum - 2; i > 0; i--)
				{
					T_ROBOT_COORS tFindCoord = pRobotDriver->m_vtWeldLineInWorldPoints[i];
					double dDis = TwoPointDis(
						tLastCoord.dX + tLastCoord.dBX, tLastCoord.dY + tLastCoord.dBY, tLastCoord.dZ + tLastCoord.dBZ,
						tFindCoord.dX + tFindCoord.dBX, tFindCoord.dY + tFindCoord.dBY, tFindCoord.dZ + tFindCoord.dBZ);
					if (dWrapLen < dDis)
					{
						nWeldLinePtnNum = i + 1;
						//pRobotDriver->m_cLog->Write(" 未发现终点时保留包角长度轨迹不发送i;%d nWeldLinePtnNum:%d", i, nWeldLinePtnNum);
						break;
					}
					else
					{
						//pRobotDriver->m_cLog->Write(" 22未发现终点时保留包角长度轨迹不发送i;%d nWeldLinePtnNum:%d", i, nWeldLinePtnNum);
						nWeldLinePtnNum = 0;
					}
				}
			}

			if (nindex < nWeldLinePtnNum)
			{
				vtVarInfo.clear();
				dStartTim = XI_clock();
				time2 = XI_clock();
				pRobotDriver->m_cLog->Write("%s i变量：%d nindex:%d 轨迹size:%d",
					strRobot, nCountInt1, nindex, pRobotDriver->m_vtWeldLineInWorldPoints.size());
				pRobotDriver->m_cLog->Write("%s 原始数据:%d %20.10lf %20.10lf %20.10lf %20.10lf %20.10lf %20.10lf", strRobot, ncount,
					pRobotDriver->m_vtWeldLineInWorldPoints[nindex].dX, pRobotDriver->m_vtWeldLineInWorldPoints[nindex].dY,
					pRobotDriver->m_vtWeldLineInWorldPoints[nindex].dZ, pRobotDriver->m_vtWeldLineInWorldPoints[nindex].dBX,
					pRobotDriver->m_vtWeldLineInWorldPoints[nindex].dBY, pRobotDriver->m_vtWeldLineInWorldPoints[nindex].dRZ
				);
				if (m_pTraceModel->m_bIfOpenExternal && !m_ptUnit->GetExAxisType(m_ptUnit->m_nTrackAxisNo))
				{
					vlWeldPulseX.clear();
					vlWeldPulseY.clear();
					vdWeldStepVel.clear();
					// 获取截断数据
					if (!GetTruncatedData(pRobotDriver, pRobotDriver->m_vtWeldLineInWorldPoints[nindex], tRobot, lRobotvel, lMachine, MachVel))
					{
						nDelPointNum++;
						nindex++;
						continue;
					}
					vlWeldPulseX.push_back(lMachine);
					vlWeldPulseY.push_back(0);
					vdWeldStepVel.push_back((double)MachVel);

					tTempRobotCoor = pRobotDriver->m_vtWeldLineInWorldPoints[nindex];
#ifdef SINGLE_ROBOT
					tTempRobotCoor.dX = tRobot.x_ / 1000.;
#else	
					tTempRobotCoor.dY = tRobot.y_ / 1000.;
#endif					
					tTempRobotCoor.dZ = tRobot.z_ / 1000.;

					WORD awAxis[] = { WORD(m_pTraceModel->m_nAXIS_X), WORD(m_pTraceModel->m_nAXIS_Y) };
					vector <long> avlRelPulse[] = { vlWeldPulseX, vlWeldPulseY };

					m_pTraceModel->m_cStepMoveControl.InsertData(avlRelPulse, vdWeldStepVel, 2);
					int nCurStepNo = m_pTraceModel->m_cStepMoveControl.GetCurrentMark();
					pRobotDriver->m_cLog->Write("%s nindex:%d CurMove:%d nCurStepNo:%d 焊接轨迹11:%11.3lf%11.3lf%11.3lf Coordin:%d",
						strRobot, nindex, nCountInt1, nCurStepNo, tRobot.x_ / 1000., tTempRobotCoor.dY, tRobot.z_ / 1000.,
						m_pTraceModel->m_cStepMoveControl.m_nCoordinate);
				}
				else
				{
					tTempRobotCoor = pRobotDriver->m_vtWeldLineInWorldPoints[nindex];
					lBPValue[0] = (long)(pRobotDriver->m_vtWeldLineInWorldPoints[nindex].dBX * 1000);
					lBPValue[1] = (long)(pRobotDriver->m_vtWeldLineInWorldPoints[nindex].dBY * 1000);
					lBPValue[2] = (long)(pRobotDriver->m_vtWeldLineInWorldPoints[nindex].dBZ * 1000);
					vtVarInfo.push_back(pRobotDriver->PrepareValData((ncount % nPointNum + 20), lBPValue));
				}
				//焊接倾角 焊接姿态
				/*tTempRobotCoor.dRX = m_dNormalWeldRx;
				tTempRobotCoor.dRY = m_dNormalWeldRy;*/
				double dDipAngle = 0;
				if (!bIsTracking)//不跟踪时所有点位都需要添加焊接倾角
				{
					tTempRobotCoor.dRZ += dRobotWeldAdjustRZ;
				}
				else//跟踪时焊接倾角，在获取跟踪数据时已经添加
				{
					dDipAngle = dRobotWeldAdjustRZ;
				}

				T_ROBOT_COORS tChangeRZRobot;
				tChangeRZRobot.dRZ = m_pTraceModel->m_dAdjustRobotRZ + dDipAngle;
				dRecordAdjustRZ = m_pTraceModel->m_dAdjustRobotRZ;
				nErrorAdjustRz = 0;
				//jwq临时改跳枪后角度偏移量清空
				if (nPointNoAfterJump >= 0 && nCountInt1 >= nPointNoAfterJump)
				{
					m_pTraceModel->m_dAdjustRobotRZ = 0.0;
					nPointNoAfterJump = -1;
				}
				//姿态				
				vtVarInfo.push_back(pRobotDriver->PrepareValData((ncount % nPointNum + 20), tTempRobotCoor));
				vtVarInfo.push_back(pRobotDriver->PrepareValData(99, tChangeRZRobot));
				vtVarInfo.push_back(pRobotDriver->PrepareValData((ncount % nPointNum + 16), lRobotvel));
				if (bAddVel)
				{
					vtVarInfo.push_back(pRobotDriver->PrepareValData(66, lAddSpeed));
				}
				pRobotDriver->SetMultiVar_H(vtVarInfo);

				pRobotDriver->m_cLog->Write("单词发送P变量时间:%.3lf", (double)(XI_clock() - dStartTim) / CLOCKS_PER_SEC);
				ncount++;
				nindex++;
			}
			else
			{
				if (bAddVel)
				{
					vtVarInfo.clear();
					vtVarInfo.push_back(pRobotDriver->PrepareValData(66, lAddSpeed));
					pRobotDriver->SetMultiVar_H(vtVarInfo);
				}
			}
		}

		if (false == m_ptUnit->RobotCheckRunning()) // 运动停止就退出
		{
			m_pTraceModel->m_bIfWeldToEndPoint = true;
			break;
		}

		Sleep(20);
		DoEvent();
	}
	RecordRunTime(pRobotDriver, tTime, "焊接");
	fclose(RecordRealData);
	//记录焊接长度和当前运行步号
	SaveWeldLengthData(pRobotDriver, m_pTraceModel->m_dCurrentWeldLen, dRecordAdjustRZ, nCountInt1 + m_pTraceModel->m_nWeldStepNo);
	SaveCurrentWarpNumber(pRobotDriver, nWarpNum);
	//获取滤波缓存数据
	vector<TrackFilter_XYZ> vtBufferData = TrackFilter_GetCachePntsPlus(m_ptUnit->m_nRobotSmooth);
	CString strFile, strFile2;
	strFile.Format("%s滤波缓存数据.txt", strCoutPath);
	SaveDataTrackFilt(vtBufferData, strFile.GetBuffer(0));
	strFile2.Format("%sBreakPointReferencePulse.txt", strCoutPath);
	SaveDataRobotPulse(GetCurrentPulse(pRobotDriver), strFile2.GetBuffer(0));//存储端点时脉冲当做下枪参考坐标
	pRobotDriver->m_cLog->Write("%s 赋值线程退出!", strRobot);
	m_ptUnit->RobotCheckDone();
	pRobotDriver->HoldOff();
	m_pTraceModel->m_cStepMoveControl.EmgStop();
	m_pTraceModel->m_cStepMoveControl.StopStep();
	fclose(m_pScanInit->m_pRecordTheoryPoint);
	T_ROBOT_COORS tEndSafe2 = pRobotDriver->GetCurrentPos();
	RobotCoordPosOffset(tEndSafe2, tEndSafe2.dRZ, 50., 200. * m_nRobotInstallDir);
	//tEndSafe2.dZ = m_dWorkPieceHighTop + 20 * m_nRobotInstallDir;
	T_ROBOT_MOVE_SPEED tPulseMove(m_dFastApproachToWorkSpeed, 20, 20);
	vector<T_ROBOT_MOVE_INFO> vtRobotMoveInfo(0);
	vtRobotMoveInfo.push_back(pRobotDriver->PVarToRobotMoveInfo(0, tEndSafe2, tPulseMove, MOVL));
	pRobotDriver->SetMoveValue(vtRobotMoveInfo);
	pRobotDriver->CallJob("CONTIMOVANY");
	CHECK_BOOL_RETURN(m_ptUnit->CheckRobotDone(tEndSafe2));
	return true;
}

void CDiaphragmWeld::InitMoveHandleYaskawaPanasonic(T_ROBOT_COORS tTrackingStartCoord, double dWeldSpeed)
{
	double dPulseToDisX = m_ptUnit->GetExPulseEquivalent(m_ptUnit->m_nTrackAxisNo);
	MoveHandle_SetMMPulse(m_ptUnit->m_nRobotSmooth, dPulseToDisX);//算法设置脉冲当量	
#ifdef SINGLE_ROBOT
	MoveHandle_Init(m_ptUnit->m_nRobotSmooth, m_pRobotDriver->m_vtWeldLineInWorldPoints[0].dX,
		m_pRobotDriver->m_vtWeldLineInWorldPoints[0].dBY, m_pRobotDriver->m_vtWeldLineInWorldPoints[0].dZ, dWeldSpeed);
#else
	MoveHandle_Init(m_ptUnit->m_nRobotSmooth, m_pRobotDriver->m_vtWeldLineInWorldPoints[0].dY * (-1),
		m_pRobotDriver->m_vtWeldLineInWorldPoints[0].dBX, m_pRobotDriver->m_vtWeldLineInWorldPoints[0].dZ, dWeldSpeed);
#endif
}

// 大车运行
bool CDiaphragmWeld::PosMoveMachine(CRobotDriverAdaptor* pRobotDriver, double dMoveDis, double dSpeed, bool bCheckDone)
{
	if (g_bLocalDebugMark)
	{
		return true;
	}	
	if ((pRobotDriver->m_nExternalAxleType >> 3) & 0x01 || (pRobotDriver->m_nExternalAxleType >> 4) & 0x01 || (pRobotDriver->m_nExternalAxleType >> 5) & 0x01)
	{
		T_ROBOT_MOVE_SPEED tPulseMove{ dSpeed / 6,100,100 };
		T_ROBOT_COORS tRobotCoors = pRobotDriver->GetCurrentPos();
		tRobotCoors.dBY = dMoveDis;
		pRobotDriver->MoveByJob(tRobotCoors, tPulseMove, pRobotDriver->m_nExternalAxleType, "MOVL");
	}
	else
	{
		m_ptUnit->MoveExAxisForLineScan(dMoveDis, dSpeed);
	}
	if (bCheckDone)
	{
		m_ptUnit->WorldCheckRobotDone();
	}

	return true;
}
// 直角坐标运动函数
bool CDiaphragmWeld::MoveByJob(CRobotDriverAdaptor* pRobotDriver, T_ROBOT_COORS tRobotJointCoord, T_ROBOT_MOVE_SPEED tPulseMove, int nExternalAxleType, CString JobName, bool bSingleMove)
{
	/*if (!pRobotDriver->CheckRobotLimitRange(tRobotJointCoord))
	{
		XiMessageBox("目标位置超出机械臂极限请检查");
		return false;
	}*/
	if (g_bLocalDebugMark)
	{
		return true;
	}
	double dDis = 0;
	WORD wPositionMode = COORD_ABS;
	double dMaxVel = tPulseMove.dSpeed;

	nExternalAxleType = pRobotDriver->m_nExternalAxleType;
	if (((nExternalAxleType >> 2) & 0x01) == 1)
	{
		dDis = tRobotJointCoord.dBZ;
	}
	if (((nExternalAxleType >> 1) & 0x01) == 1)
	{
		dDis = tRobotJointCoord.dBY;
	}
	if ((nExternalAxleType & 0x01) == 1)
	{
		dDis = tRobotJointCoord.dBX;
	}
	if ((pRobotDriver->m_nExternalAxleType >> 3) & 0x01 || (pRobotDriver->m_nExternalAxleType >> 4) & 0x01 || (pRobotDriver->m_nExternalAxleType >> 5) & 0x01)
	{
		wPositionMode = COORD_REL;
		dMaxVel = 50.0;
		dDis = 0.0;
	}
	if (!((pRobotDriver->m_nExternalAxleType >> 3) & 0x01 || (pRobotDriver->m_nExternalAxleType >> 4) & 0x01 || (pRobotDriver->m_nExternalAxleType >> 5) & 0x01))
	{
		// 机械臂运行时长
		double dCurrentPos = m_cMoveCtrl.GetPositionDis(m_pTraceModel->m_nAXIS_X);
		T_ROBOT_COORS tCoor = GetCurrentPos(pRobotDriver);
		double dRobotMoveDis = TwoPointDis(tRobotJointCoord.dX, tRobotJointCoord.dY, tRobotJointCoord.dZ, tCoor.dX, tCoor.dY, tCoor.dZ);
		double dMachineDis = fabs(dCurrentPos - dDis);

		if (dRobotMoveDis > dMachineDis)
		{
			double dRobotMoveTime = dRobotMoveDis / tPulseMove.dSpeed;

			double dMachineVel = dMachineDis / dRobotMoveTime;
			pRobotDriver->m_cLog->Write("MoveByJob修正大车速度1：After %.3lf befor:.3lf", dMachineVel, dMaxVel);
			if (dMachineVel < dMaxVel && dMachineVel>0.)
			{
				dMaxVel = dMachineVel;
			}
		}
		else if (dMachineDis > dRobotMoveDis)
		{
			double dMachineMoveTime = dMachineDis / dMaxVel;

			double dRobotVel = dRobotMoveDis / dMachineMoveTime;
			pRobotDriver->m_cLog->Write("MoveByJob修正大车速度2：After %.3lf befor:.3lf", dRobotVel, dMaxVel);
			if (dRobotVel < tPulseMove.dSpeed && dRobotVel>0.0)
			{
				tPulseMove.dSpeed = dRobotVel;
			}
		}
	}

	tPulseMove.dSpeed /= 6;
	pRobotDriver->MoveByJob(tRobotJointCoord, tPulseMove, nExternalAxleType, JobName);
	if (bSingleMove || (pRobotDriver->m_nExternalAxleType >> 3) & 0x01 || (pRobotDriver->m_nExternalAxleType >> 4) & 0x01 || (pRobotDriver->m_nExternalAxleType >> 5) & 0x01)
	{
		if (!m_ptUnit->CheckRobotDone(tRobotJointCoord))
		{
			XiMessageBox("机器人未运行到目标位置");
			return false;
		}
	}

	if ((pRobotDriver->m_nExternalAxleType >> 3) & 0x01 || (pRobotDriver->m_nExternalAxleType >> 4) & 0x01 || (pRobotDriver->m_nExternalAxleType >> 5) & 0x01)
	{
		return true;
	}
	m_cMoveCtrl.PosMoveDis(m_pTraceModel->m_nAXIS_X, dDis, wPositionMode, 0, dMaxVel, 0.3, 0.3);

	if (!bSingleMove)
	{
		if (!m_ptUnit->CheckRobotDone(tRobotJointCoord))
		{
			XiMessageBox("机器人未运行到目标位置");
			return false;
		}
	}
	m_cMoveCtrl.CheckDoneDis(m_pTraceModel->m_nAXIS_X);
	double dPos = m_cMoveCtrl.GetPositionDis(m_pTraceModel->m_nAXIS_X);
	if (fabs(dPos - dDis) > 1.0 && wPositionMode == COORD_ABS)
	{
		//已修改
		XUI::MesBox::PopInfo("大车未运行到目标位置,目标点：{0:.3f},当前位置：{1:.3f}", dDis, dPos);
		//XiMessageBox("大车未运行到目标位置,目标点：%.3lf,当前位置：%.3lf", dDis, dPos);
		return false;
	}
	return true;
}
//关节坐标运动函数
bool CDiaphragmWeld::MoveByJob(CRobotDriverAdaptor* pRobotDriver, T_ANGLE_PULSE tRobotJointCoord, T_ROBOT_MOVE_SPEED tPulseMove, int nExternalAxleType, CString JobName, bool bSingleMove)
{
	/*if (!pRobotDriver->CheckRobotLimitRange(tRobotJointCoord))
	{
		XiMessageBox("目标位置超出机械臂极限请检查");
		return false;
	}*/
	if (g_bLocalDebugMark)
	{
		return true;
	}
	long lDis = 0;
	WORD wPositionMode = COORD_ABS;
	double dMaxVel = tPulseMove.dSpeed;
	nExternalAxleType = pRobotDriver->m_nExternalAxleType;
	if (((nExternalAxleType >> 2) & 0x01) == 1)
	{
		lDis = tRobotJointCoord.lBZPulse;
	}
	if (((nExternalAxleType >> 1) & 0x01) == 1)
	{
		lDis = tRobotJointCoord.lBYPulse;
	}
	if ((nExternalAxleType & 0x01) == 1)
	{
		lDis = tRobotJointCoord.lBXPulse;
	}
	if ((pRobotDriver->m_nExternalAxleType >> 3) & 0x01 || (pRobotDriver->m_nExternalAxleType >> 4) & 0x01 || (pRobotDriver->m_nExternalAxleType >> 5) & 0x01)
	{
		wPositionMode = COORD_REL;
		dMaxVel = 50.0;
		lDis = 0;
	}

	tPulseMove.dSpeed /= 6;
	pRobotDriver->MoveByJob(tRobotJointCoord, tPulseMove, nExternalAxleType, JobName);

	if (bSingleMove)
	{
		if (!m_ptUnit->CheckRobotDone(tRobotJointCoord))
		{
			XiMessageBox("机器人未运行到目标位置");
			return false;
		}
	}
	//m_cMoveCtrl.PosMoveDis(m_pTraceModel->m_nAXIS_X, lDis * m_cMoveCtrl.m_mapPulseToDis[m_pTraceModel->m_nAXIS_X], wPositionMode, 0, dMaxVel, 0.3, 0.3);
	if (!bSingleMove)
	{
		if (!m_ptUnit->CheckRobotDone(tRobotJointCoord))
		{
			XiMessageBox("机器人未运行到目标位置");
			return false;
		}
	}
	m_cMoveCtrl.CheckDoneDis(m_pTraceModel->m_nAXIS_X);
	if (false == pRobotDriver->ComparePulse(tRobotJointCoord, GetCurrentPulse(pRobotDriver)))
	{
		XiMessageBox("机器人未运动到位");
		return false;
	}

	return true;
}

//
bool CDiaphragmWeld::CalcRobotAndMachinePos(CRobotDriverAdaptor* pRobotDriver, T_ROBOT_COORS WorldCoors, T_ROBOT_COORS& tCoutRobotCoor)
{
	tCoutRobotCoor = WorldCoors;
	// 分配机器人外部轴位置，机器人固定位置，移动分量给到外部轴
	double dTrackWeldRobotPos = -500.0;
	if (1 == m_pRobotDriver->m_nRobotInstallDir) dTrackWeldRobotPos = 0.0;
	double dTeackWeldExPos = 0.0;
	// 跟踪外部轴位置
	dTeackWeldExPos = 1 == m_ptUnit->m_nTrackAxisNo ? (WorldCoors.dX + WorldCoors.dBX - dTrackWeldRobotPos) : dTeackWeldExPos;
	dTeackWeldExPos = 2 == m_ptUnit->m_nTrackAxisNo ? (WorldCoors.dY + WorldCoors.dBY - dTrackWeldRobotPos) : dTeackWeldExPos;
	dTeackWeldExPos = 3 == m_ptUnit->m_nTrackAxisNo ? (WorldCoors.dZ + WorldCoors.dBZ - dTrackWeldRobotPos) : dTeackWeldExPos;

	tCoutRobotCoor.dBX = 1 == m_ptUnit->m_nTrackAxisNo ? dTeackWeldExPos : tCoutRobotCoor.dBX;
	tCoutRobotCoor.dBY = 2 == m_ptUnit->m_nTrackAxisNo ? dTeackWeldExPos : tCoutRobotCoor.dBY;
	tCoutRobotCoor.dBZ = 3 == m_ptUnit->m_nTrackAxisNo ? dTeackWeldExPos : tCoutRobotCoor.dBZ;

	// 跟踪同外部轴平行坐标轴位置
	tCoutRobotCoor.dX = 1 == m_ptUnit->m_nTrackAxisNo ? dTrackWeldRobotPos : tCoutRobotCoor.dX;
	tCoutRobotCoor.dY = 2 == m_ptUnit->m_nTrackAxisNo ? dTrackWeldRobotPos : tCoutRobotCoor.dY;
	tCoutRobotCoor.dZ = 3 == m_ptUnit->m_nTrackAxisNo ? dTrackWeldRobotPos : tCoutRobotCoor.dZ;

#if 0
	eMoveDir = ROBOT_CAR_STOP;
	if (true != nIsExAxis)
	{
		tCoutRobotCoor = WorldCoors;
		return ROBOT_CAR_SUCCESS;
	}
	if (WorldCoors.dX > pRobotDriver->m_tRobotLimitation.drXMax || WorldCoors.dX < pRobotDriver->m_tRobotLimitation.drXMin)
	{
		return X_COOR_OVERRUN;
	}
	else if (WorldCoors.dZ > pRobotDriver->m_tRobotLimitation.drZMax || WorldCoors.dZ < pRobotDriver->m_tRobotLimitation.drZMin)
	{
		return Z_COOR_OVERRUN;
	}
	else if (WorldCoors.dY<pRobotDriver->m_tRobotLimitation.drYMinCar + pRobotDriver->m_tRobotLimitation.drYMinRobot || WorldCoors.dY>pRobotDriver->m_tRobotLimitation.drYMaxCar + pRobotDriver->m_tRobotLimitation.drYMaxRobot)
	{
		return Y_COOR_OVERRUN;
	}
	int nDir = 1;
    double y = -250 * nDir;//机器人固定位置
	double dMachineCarMovePos = 0.0;
	if ((WorldCoors.dY <= pRobotDriver->m_tRobotLimitation.drYMaxCar - 200 &&
		WorldCoors.dY >= pRobotDriver->m_tRobotLimitation.drYMinCar + 200 ||
		(WorldCoors.dY < pRobotDriver->m_tRobotLimitation.drYMinCar + 200)) && !bRobotDir)
	{
		tCoutRobotCoor = WorldCoors;
		dMachineCarMovePos = WorldCoors.dY - y;
		tCoutRobotCoor.dY = y;
		tCoutRobotCoor.dBY = dMachineCarMovePos;
		eMoveDir = WELD_ROBOT_LEFT;
	}
	else if (WorldCoors.dY > pRobotDriver->m_tRobotLimitation.drYMaxCar - 200 || bRobotDir)
	{

		tCoutRobotCoor = WorldCoors;
		dMachineCarMovePos = WorldCoors.dY + y;
		tCoutRobotCoor.dY = -y;
		tCoutRobotCoor.dBY = dMachineCarMovePos;
		eMoveDir = WELD_ROBOT_RIGHT;
	}

#endif // 0
	return true;
}


T_ANGLE_PULSE CDiaphragmWeld::GetCurrentPulse(CRobotDriverAdaptor* pRobotDriver)
{
	T_ANGLE_PULSE tRobotCurPulses;
	if (g_bLocalDebugMark)
	{
		return tRobotCurPulses;
	}
	tRobotCurPulses.nSPulse = pRobotDriver->GetCurrentPulse(SAxis);
	tRobotCurPulses.nLPulse = pRobotDriver->GetCurrentPulse(LAxis);
	tRobotCurPulses.nUPulse = pRobotDriver->GetCurrentPulse(UAxis);
	tRobotCurPulses.nRPulse = pRobotDriver->GetCurrentPulse(RAxis);
	tRobotCurPulses.nBPulse = pRobotDriver->GetCurrentPulse(BAxis);
	tRobotCurPulses.nTPulse = pRobotDriver->GetCurrentPulse(TAxis);

	if (!((pRobotDriver->m_nExternalAxleType >> 3) & 0x01 || (pRobotDriver->m_nExternalAxleType >> 4) & 0x01 || (pRobotDriver->m_nExternalAxleType >> 5) & 0x01))
	{
		tRobotCurPulses.lBXPulse = 0.;
		tRobotCurPulses.lBYPulse = m_cMoveCtrl.GetPosition(m_pTraceModel->m_nAXIS_X);
		tRobotCurPulses.lBZPulse = 0.;
	}
	else
	{
		tRobotCurPulses.lBXPulse = pRobotDriver->GetCurrentPulse(BPXAxis);
		tRobotCurPulses.lBYPulse = pRobotDriver->GetCurrentPulse(BPYAxis);
		tRobotCurPulses.lBZPulse = pRobotDriver->GetCurrentPulse(BPZAxis);
	}

	return tRobotCurPulses;
}


T_ROBOT_COORS CDiaphragmWeld::GetCurrentPos(CRobotDriverAdaptor* pRobotDriver)
{
	T_ROBOT_COORS tRobotCurCoord;
	if (g_bLocalDebugMark)
	{
		return tRobotCurCoord;
	}
	tRobotCurCoord.dX = pRobotDriver->GetCurrentPos(ROBOT_AXIS_X);
	tRobotCurCoord.dY = pRobotDriver->GetCurrentPos(ROBOT_AXIS_Y);
	tRobotCurCoord.dZ = pRobotDriver->GetCurrentPos(ROBOT_AXIS_Z);
	tRobotCurCoord.dRX = pRobotDriver->GetCurrentPos(ROBOT_AXIS_RX);
	tRobotCurCoord.dRY = pRobotDriver->GetCurrentPos(ROBOT_AXIS_RY);
	tRobotCurCoord.dRZ = pRobotDriver->GetCurrentPos(ROBOT_AXIS_RZ);
	if (!((pRobotDriver->m_nExternalAxleType >> 3) & 0x01 || (pRobotDriver->m_nExternalAxleType >> 4) & 0x01 || (pRobotDriver->m_nExternalAxleType >> 5) & 0x01))
	{
		tRobotCurCoord.dBX = 0.;
		tRobotCurCoord.dBY = m_cMoveCtrl.GetPositionDis(m_pTraceModel->m_nAXIS_X);
		tRobotCurCoord.dBZ = 0.;
	}
	else
	{
		tRobotCurCoord.dBX = pRobotDriver->GetCurrentPos(ROBOT_AXIS_BPX);
		tRobotCurCoord.dBY = pRobotDriver->GetCurrentPos(ROBOT_AXIS_BPY);
		tRobotCurCoord.dBZ = pRobotDriver->GetCurrentPos(ROBOT_AXIS_BPZ);
		tRobotCurCoord.dX -= tRobotCurCoord.dBX;
		tRobotCurCoord.dY -= tRobotCurCoord.dBY;
		tRobotCurCoord.dZ -= tRobotCurCoord.dBZ;
	}

	return tRobotCurCoord;
}

void CDiaphragmWeld::IntValWaitFun(CRobotDriverAdaptor* pRobotDriver, int nIntVal, int nVal)
{
	int nWhileNum = 0;
	int nIval = 0;
	while (1)
	{
		CHECK_STOP_BREAK(pRobotDriver);
		nIval = pRobotDriver->GetIntVar(nIntVal);
		if (nVal == nIval)
		{
			break;
		}
		nWhileNum++;
		DoEvent();
		Sleep(50);
	}
	pRobotDriver->m_cLog->Write("IntValWaitFun 结束循环：%d %d 等待次数：%d", nIval, nIntVal, nWhileNum);
}

void CDiaphragmWeld::InitWorkPieceReleveInfo(CRobotDriverAdaptor* pRobotDriver)
{
	//VVT_CORNER_LINE_INFO_NEW vvtCornerLineInfoNew;
	//VVT_ENDPOINT_INFO_NEW vvtPointInfoNew;
	//VVT_VERTICAL_INFO_NEW vvtVerticalInfoNew;
	//VVT_MEASURE_INFO_NEW vvMeasureInfoNew;
	//VVT_CIRCLE_CENTER_INFO_NEW vvtCircleCenterInfo;
	//ReadWorkpieceInfomation(vvtCornerLineInfoNew, vvtPointInfoNew, vvtVerticalInfoNew, vvMeasureInfoNew, vvtCircleCenterInfo);
	LoadWorkPieceInfo();
	int nWeldSeamNum = 0;
	if (m_vvtCornerLineInfoNew.size() > 0)
	{
		int nSize = m_vvtCornerLineInfoNew.at(0).size();
		nWeldSeamNum += nSize;
	}
	if (m_vvtVerticalInfoNew.size() > 0)
	{
		int nSize = m_vvtVerticalInfoNew.at(0).size();
		nWeldSeamNum += nSize;
	}
	RefreshWeldReleveParam(pRobotDriver, nWeldSeamNum);
	if (m_vvtPointInfoNew.size() > 0)
	{
		RefreshCorrectEndPointFun(pRobotDriver, m_vvtPointInfoNew.at(0).size());
	}
}


void CDiaphragmWeld::RefreshWeldReleveParam(CRobotDriverAdaptor* pRobotDriver, int nWeldSeamNum)
{
	COPini OPini;
	CString srtFilePath;
	srtFilePath.Format("Data\\RelevanceFill\\RelevanceWeldStatus.ini");
	OPini.SetFileName(srtFilePath.GetBuffer(0));
	int nStatus = 0;
	for (int n = 0; n < nWeldSeamNum; n++)
	{
		CString strNo;
		strNo.Format("EndPoint_%d", n);
		OPini.SetSectionName(strNo);
		OPini.WriteString("bWeldStatus", nStatus);
	}
}
void CDiaphragmWeld::RefreshCorrectEndPointFun(CRobotDriverAdaptor* pRobotDriver, int EndPointNum)
{
	int nRenew = 0;
	XI_POINT tPoint = { 0 };
	COPini OPini;
	CString srtFilePath;
	srtFilePath.Format("Data\\RelevanceFill\\RelevanceEndPoint.ini");
	OPini.SetFileName(srtFilePath.GetBuffer(0));
	for (int n = 0; n < EndPointNum; n++)
	{
		CString strNo;
		strNo.Format("EndPoint_%d", n);
		OPini.SetSectionName(strNo);
		OPini.WriteString("Renew", nRenew);
		OPini.WriteString("dEndPointX", tPoint.x);
		OPini.WriteString("dEndPointY", tPoint.y);
		OPini.WriteString("dEndPointZ", tPoint.z);
	}

}

void CDiaphragmWeld::LoadWorkPieceInfo()
{
	double dMaxVal = -99999.0;//最高点
	double dMinVal = 99999.0;//最低点
	m_vvtCornerLineInfoNew.clear();
	m_vvtPointInfoNew.clear();
	m_vvtVerticalInfoNew.clear();
	m_vvMeasureInfoNew.clear();
	m_vvtCircleCenterInfo.clear();
	T_CORNER_LINE_INFO_NEW tBi;
	VT_CORNER_LINE_INFO_NEW vtBi;
	CString cstrTmp;
	int nnn = 0;
	int nWorkpieceNoLast = -1;
	int nTotalBoardNum;
	cstrTmp = OUTPUT_PATH + "BoardInfo.txt";
	FILE* pfileBoardInfo = fopen(cstrTmp, "r");
	fscanf(pfileBoardInfo, "CornerSeamsNum: %d\n", &nTotalBoardNum);
	WriteLog("Boardnum:%d", nTotalBoardNum);
	for (int nBoardNo = 0; nBoardNo < nTotalBoardNum; nBoardNo++)
	{
		fscanf(pfileBoardInfo, "%d%d%d%lf%lf%lf%d%d%d%d%d%d%d%lf%d%d%d%d%d%d%d%lf%d%d%lf%lf%d%d%d%d%d\n", &tBi.nWorkpieceNo, &tBi.nBoardNo, (int*)(void*)&tBi.bBoardType,
			&tBi.dBoardLength, &tBi.dBoardThick, &tBi.dBoardHight, &tBi.nStartNo, &tBi.nStartType, &tBi.nStartShape, &tBi.nStartIfWrap, (int*)(void*)&tBi.bStartReleVircital, (int*)(void*)&tBi.bStartReleFlat, &tBi.bStartContiguousCor,
			&tBi.dStartAngle, &tBi.nEndNo, &tBi.nEndType, &tBi.nEndShape, &tBi.nEndIfWrap, (int*)(void*)&tBi.bEndReleVircital, (int*)(void*)&tBi.bEndReleFlat, &tBi.bEndContiguousCor, &tBi.dEndAngle, &tBi.nProperty,
			&tBi.nWeldLineWeldType, &tBi.dInterWeldLength, &tBi.dInterStopLength, &tBi.nInterWeldType, (int*)(void*)&tBi.bIsDoubleWeld, (int*)(void*)&tBi.bIfArc, &tBi.bIfWeldCom, &tBi.nWitchRobot);

		if (dMaxVal < tBi.dBoardHight * m_nRobotInstallDir)
		{
			dMaxVal = tBi.dBoardHight * m_nRobotInstallDir;
		}
		if (dMinVal > tBi.dBoardHight * m_nRobotInstallDir)
		{
			dMinVal = tBi.dBoardHight * m_nRobotInstallDir;
		}

		if (0 == nBoardNo)
		{
			nWorkpieceNoLast = tBi.nWorkpieceNo;
		}
		if (nWorkpieceNoLast == tBi.nWorkpieceNo) //同一种
		{
			vtBi.push_back(tBi);
		}
		else if (nWorkpieceNoLast != tBi.nWorkpieceNo)
		{
			m_vvtCornerLineInfoNew.push_back(vtBi);
			vtBi.clear();
			vtBi.push_back(tBi);
			nWorkpieceNoLast = tBi.nWorkpieceNo;
		}
		if (nBoardNo == nTotalBoardNum - 1)
		{
			m_vvtCornerLineInfoNew.push_back(vtBi);
		}
		WriteLog("vtBi:%d m_vvtCornerLineInfoNew:%d", vtBi.size(), m_vvtCornerLineInfoNew.size());
		WriteLog("总表信息：%d %d %d %lf %lf %lf %d %d %d %d %d %d %d %lf %d %d %d %d %d %d %d %lf %d %d %lf %lf %d %d %d %d\n", tBi.nWorkpieceNo, tBi.nBoardNo, tBi.bBoardType,
			tBi.dBoardLength, tBi.dBoardThick, tBi.dBoardHight, tBi.nStartNo, tBi.nStartType, tBi.nStartShape, tBi.nStartIfWrap, tBi.bStartReleVircital, tBi.bStartReleFlat, tBi.bStartContiguousCor,
			tBi.dStartAngle, tBi.nEndNo, tBi.nEndType, tBi.nEndShape, tBi.nEndIfWrap, tBi.bEndReleVircital, tBi.bEndReleFlat, tBi.bEndContiguousCor, tBi.dEndAngle, tBi.nProperty,
			tBi.nWeldLineWeldType, tBi.dInterWeldLength, tBi.dInterStopLength, tBi.nInterWeldType, tBi.bIfArc, tBi.bIfWeldCom, tBi.nWitchRobot);
	}

	int nTotalEndNum = 0;

	T_ENDPOINT_INFO_NEW tEi;
	VT_ENDPOINT_INFO_NEW vtEi;
	fscanf(pfileBoardInfo, "TotalEndPointNum:%d\n", &nTotalEndNum);
	WriteLog("TotalEndPointNum:%d", nTotalEndNum);
	for (int nEndIdx = 0; nEndIdx < nTotalEndNum; nEndIdx++)
	{
		fscanf(pfileBoardInfo, "%d%d%d%lf%lf%lf\n", &tEi.nWorkpieceNo, &tEi.nBoardNo, &tEi.nEndPointNo, &tEi.tPointData.x, &tEi.tPointData.y, &tEi.tPointData.z);
		//获取Z值最高点最低点
		if (dMaxVal < tEi.tPointData.z * m_nRobotInstallDir)
		{
			dMaxVal = tEi.tPointData.z * m_nRobotInstallDir;
		}
		if (dMinVal > tEi.tPointData.z * m_nRobotInstallDir)
		{
			dMinVal = tEi.tPointData.z * m_nRobotInstallDir;
		}
		if (0 == nEndIdx)
		{
			nWorkpieceNoLast = tEi.nWorkpieceNo;
		}
		if (nWorkpieceNoLast == tEi.nWorkpieceNo) //同一种
		{
			vtEi.push_back(tEi);
		}
		else if (nWorkpieceNoLast != tEi.nWorkpieceNo)
		{
			m_vvtPointInfoNew.push_back(vtEi);
			vtEi.clear();
			vtEi.push_back(tEi);
			nWorkpieceNoLast = tEi.nWorkpieceNo;
		}
		if (nEndIdx == nTotalEndNum - 1)
		{
			m_vvtPointInfoNew.push_back(vtEi);
		}
		WriteLog("vtEi:%d m_vvtPointInfoNew:%d", vtEi.size(), m_vvtPointInfoNew.size());
		WriteLog("端点信息：%d %d %d %lf %lf %lf\n", tEi.nWorkpieceNo, tEi.nBoardNo, tEi.nEndPointNo, tEi.tPointData.x, tEi.tPointData.y, tEi.tPointData.z);
	}


	T_VERTICAL_INFO_NEW tVi;
	VT_VERTICAL_INFO_NEW vtVi;
	int nTotalVerWeldEndPointNum = 0;
	fscanf(pfileBoardInfo, "VerWeldEndPointNum:%d\n", &nTotalVerWeldEndPointNum);
	WriteLog("nTotalVerWeldEndPointNum:%d", nTotalVerWeldEndPointNum);
	for (int nVerIdx = 0; nVerIdx < nTotalVerWeldEndPointNum; nVerIdx++)
	{
		fscanf(pfileBoardInfo, "%d%d%d%lf%lf %lf%lf%lf%d%d %d%d%d%lf%lf%d%d%d%d%d\n",
			&tVi.nWorkpieceNo, &tVi.nBoardNo, &tVi.nCornerLineNo, &tVi.dContiguousFlat, &tVi.dVerHoleHeight,
			&tVi.dVerWeldLength, &tVi.dMainAngle, &tVi.dStartAngle, &tVi.nStartNo, &tVi.nStartIfWrap, &tVi.nEndNo, &tVi.nEndIfWrap, &tVi.nWeldLineWeldType,
			&tVi.dInterWeldLength, &tVi.dInterStopLength, &tVi.nInterWeldType, (int*)(void*)&tVi.bIsDoubleWeld, (int*)(void*)&tVi.bIfArc, (int*)(void*)&tVi.bIfWeldCom, &tVi.nWitchRobot);
		if (0 == nVerIdx)
		{
			nWorkpieceNoLast = tVi.nWorkpieceNo;
		}
		if (nWorkpieceNoLast == tVi.nWorkpieceNo) //同一种
		{
			vtVi.push_back(tVi);
		}
		else if (nWorkpieceNoLast != tVi.nWorkpieceNo)
		{
			m_vvtVerticalInfoNew.push_back(vtVi);//有效填充
			vtVi.clear();
			vtVi.push_back(tVi);
			nWorkpieceNoLast = tVi.nWorkpieceNo;
		}
		if (nVerIdx == nTotalVerWeldEndPointNum - 1)
		{
			m_vvtVerticalInfoNew.push_back(vtVi);
		}
		WriteLog("vtVi:%d m_vvtVerticalInfoNew:%d", vtVi.size(), m_vvtVerticalInfoNew.size());
		WriteLog("立缝信息：%d %d %d %lf %lf %lf %lf %lf %d %d %d %lf %lf %d %d %d %d\n", tVi.nWorkpieceNo, tVi.nBoardNo, tVi.nCornerLineNo, tVi.dContiguousFlat,
			tVi.dVerHoleHeight, tVi.dVerWeldLength, tVi.dMainAngle, tVi.dStartAngle, tVi.nStartNo, tVi.nEndNo, tVi.nWeldLineWeldType, tVi.dInterWeldLength, tVi.dInterStopLength, tVi.nInterWeldType, tVi.bIfArc, tVi.bIfWeldCom, tVi.nWitchRobot);
	}

	T_MEASURE_INFO_NEW tMi;
	VT_MEASURE_INFO_NEW vtMi;
	int nTotalMeasurePointNum = 0;
	fscanf(pfileBoardInfo, "MeasurePointNum:%d\n", &nTotalMeasurePointNum);
	WriteLog("MeasurePointNum:%d", nTotalMeasurePointNum);
	for (int nMeaIdx = 0; nMeaIdx < nTotalMeasurePointNum; nMeaIdx++)
	{
		tMi.VMeasurePOINT.clear();
		fscanf(pfileBoardInfo, "%d%d%d", &tMi.nWorkpieceNo, &tMi.nBoardNo, &tMi.nMeasurePointNum);
		while (tMi.nMeasurePointNum > 0)
		{
			XI_POINT XPoint;
			if (tMi.nMeasurePointNum == 1)
			{
				fscanf(pfileBoardInfo, "%lf%lf%lf\n", &XPoint.x, &XPoint.y, &XPoint.z);
				tMi.VMeasurePOINT.push_back(XPoint);
			}
			else
			{
				fscanf(pfileBoardInfo, "%lf%lf%lf", &XPoint.x, &XPoint.y, &XPoint.z);
				tMi.VMeasurePOINT.push_back(XPoint);
			}
			WriteLog("测量点：%d %lf %lf %lf ", tMi.nMeasurePointNum, XPoint.x, XPoint.y, XPoint.z);
			tMi.nMeasurePointNum--;
		}
		if (0 == nMeaIdx)
		{
			nWorkpieceNoLast = tMi.nWorkpieceNo;
		}
		if (nWorkpieceNoLast == tMi.nWorkpieceNo) //同一种
		{
			vtMi.push_back(tMi);
		}
		else if (nWorkpieceNoLast != tMi.nWorkpieceNo)
		{
			m_vvMeasureInfoNew.push_back(vtMi);//有效填充
			vtMi.clear();
			vtMi.push_back(tMi);
			nWorkpieceNoLast = tMi.nWorkpieceNo;
		}
		if (nMeaIdx == nTotalMeasurePointNum - 1)
		{
			m_vvMeasureInfoNew.push_back(vtMi);
		}
		WriteLog("测量信息：%d %d\n", tMi.nWorkpieceNo, tMi.nBoardNo);
		WriteLog("vtMi:%d m_vvMeasureInfoNew:%d", vtMi.size(), m_vvMeasureInfoNew.size());
	}

	T_CIRCLE_CENTER_INFO_NEW tCirCtTi;
	VT_CIRCLE_CENTER_INFO_NEW vtCirCtTi;
	int nCirCtPointNum = 0;
	fscanf(pfileBoardInfo, "TotalRingCenterNum:%d\n", &nCirCtPointNum);
	WriteLog("TotalRingCenterNum:%d", nCirCtPointNum);
	for (int nFtIdx = 0; nFtIdx < nCirCtPointNum; nFtIdx++)
	{
		fscanf(pfileBoardInfo, "%d%d%lf%lf%lf\n", &tCirCtTi.nWorkpieceNo, &tCirCtTi.nBoardNo, &tCirCtTi.dCenterX, &tCirCtTi.dCenterY, &tCirCtTi.dCenterZ);//获取Z值最高点最低点
		if (0 == nFtIdx)
		{
			nWorkpieceNoLast = tCirCtTi.nWorkpieceNo;
		}
		if (nWorkpieceNoLast == tCirCtTi.nWorkpieceNo) //同一种
		{
			vtCirCtTi.push_back(tCirCtTi);
		}
		else if (nWorkpieceNoLast != tCirCtTi.nWorkpieceNo)
		{
			m_vvtCircleCenterInfo.push_back(vtCirCtTi);//有效填充
			vtCirCtTi.clear();
			vtCirCtTi.push_back(tCirCtTi);
			nWorkpieceNoLast = tCirCtTi.nWorkpieceNo;
		}
		if (nFtIdx == nCirCtPointNum - 1)
		{
			m_vvtCircleCenterInfo.push_back(vtCirCtTi);
		}
		WriteLog("vtFLATi:%d vvFlatSeamInfo:%d", vtCirCtTi.size(), m_vvtCircleCenterInfo.size());
		WriteLog("圆心信息：%d %d %lf %lf %lf\n", tCirCtTi.nWorkpieceNo, tCirCtTi.nBoardNo, tCirCtTi.dCenterX, tCirCtTi.dCenterY, tCirCtTi.dCenterZ);
	}

	fclose(pfileBoardInfo);
	WriteLimitHight(dMaxVal * m_nRobotInstallDir, dMinVal * m_nRobotInstallDir);
	LoadLimitHight();
	WriteLog("最高点：%lf 最低点：%lf", dMaxVal, dMinVal);
}

bool CDiaphragmWeld::SendDataTOCallJobRobot(CRobotDriverAdaptor* pRobotDriver, vector<T_ROBOT_COORS> vtRealWeldCoors)
{

	if (*m_pIsArcOn)
	{
		pRobotDriver->SetIntVar(58, 1);//是否开放焊接条件
	}
	else
	{
		pRobotDriver->SetIntVar(58, 0);
	}

	double dNormalDis = 30;
	int nWeldStepNo = 0;
	double dAbjustRz = 0.0;
	double dOverLength = 0.0;
	m_pTraceModel->m_nWeldStepNo = 0;
	if (m_ptUnit->m_bBreakPointContinue)
	{
		//获取焊接轨迹， 获取焊接长度， 步号， rz调整角度
		LoadWeldLengthData(pRobotDriver, m_pTraceModel->m_dCurrentWeldLen, dAbjustRz, nWeldStepNo, dOverLength);
		m_pTraceModel->m_nWeldStepNo = nWeldStepNo;//用于先测后焊记录步号
		vtRealWeldCoors.erase(vtRealWeldCoors.begin(), vtRealWeldCoors.begin() + m_pTraceModel->m_nWeldStepNo);
		//StepPointToSafe(pRobotDriver, vtRealWeldCoors.at(0), dNormalDis, 1600);
	}
	CString strRobot = pRobotDriver->m_strRobotName;
	long long dStartTim;
	dStartTim = XI_clock();
	int ncount = 0;
	int nindex = 0;
	int nCountInt1 = 0;
	double dPosCoorLRobot[6] = { 0 };
	MP_USR_VAR_INFO tVarInfo[100];
	MP_USR_VAR_INFO tVarInfoTrack[100];
	long lIntVar[50];
	int nvcount = 9999;
	long lRobotvel = m_pTraceModel->m_dWeldVelocity / 6;

	int nPointNum = 40;
	long long time2 = XI_clock();
	int nFirstNum = vtRealWeldCoors.size();
	pRobotDriver->SetIntVar(1, 0);
	pRobotDriver->SetIntVar(2, nFirstNum);
	vector<long> vlWeldPulseX;
	vector<long> vlWeldPulseY;
	vector<double> vdWeldStepVel;
	vlWeldPulseX.clear();
	vlWeldPulseY.clear();
	vdWeldStepVel.clear();

	while (true)
	{
		if (ncount < nPointNum)
		{
			tVarInfo[nindex].var_type = MP_VAR_P;
			tVarInfo[nindex].var_no = (nindex % nPointNum + 20);
			tVarInfo[nindex].val.p.dtype = MP_ROBO_DTYPE;
			tVarInfo[nindex].val.p.uf_no = 0;//该参数无意义
			tVarInfo[nindex].val.p.tool_no = 1;
			tVarInfo[nindex].val.p.fig_ctrl = 4;
			tVarInfo[nindex].val.p.data[0] = (long)(vtRealWeldCoors[ncount].dX * 1000);
			tVarInfo[nindex].val.p.data[1] = (long)(vtRealWeldCoors[ncount].dY * 1000);
			tVarInfo[nindex].val.p.data[2] = (long)(vtRealWeldCoors[ncount].dZ * 1000);
			tVarInfo[nindex].val.p.data[3] = (long)(vtRealWeldCoors[ncount].dRX * 10000);
			tVarInfo[nindex].val.p.data[4] = (long)(vtRealWeldCoors[ncount].dRY * 10000);
			tVarInfo[nindex].val.p.data[5] = (long)(vtRealWeldCoors[ncount].dRZ * 10000);
			tVarInfo[nindex].val.p.data[6] = 0;//该参数无意义
			tVarInfo[nindex].val.p.data[7] = 0;//该参数无意义
			lIntVar[nindex % nPointNum] = lRobotvel;
			if (nindex == nPointNum - 1 || nindex == (nFirstNum - 1))
			{
				pRobotDriver->SetMultiVar_H((nindex + 1), tVarInfo); Sleep(50);
				pRobotDriver->SetMultiVar((nindex + 1), 20, lIntVar); Sleep(50);
				m_ptUnit->RobotCheckDone();//检查前一步是否运行完成
				pRobotDriver->HoldOff(); Sleep(100);
				pRobotDriver->ServoOn(); Sleep(100);
				pRobotDriver->CallJob("WELD_PROGRAM");
			}
			ncount++;
			nindex++;
			if (nindex == (nFirstNum))
			{
				WriteLog("%s 数据小于19个直接运行job:%d", strRobot, nFirstNum);
				break;
			}
		}
		else
		{
			break;
		}
		if (pRobotDriver->m_eThreadStatus == INCISEHEAD_THREAD_STATUS_STOPPED)
		{
			XiMessageBox("机械臂急停");
			break;
		}
	}
	//I变量校验

	WriteLog("开始I变量校验");
	nCountInt1 = pRobotDriver->GetIntVar(1);
	nCountInt1 = pRobotDriver->GetIntVar(1);
	if (nCountInt1 != 0)
	{
		XUI::MesBox::PopError("Error:nInteger1 = {0}，不符合设定！", nCountInt1);
		return false;
	}
	int nRefer = ncount;
	while (TRUE/*ncount < nFirstNum*/)
	{
		nCountInt1 = pRobotDriver->GetIntVar(1);
		if (nCountInt1 > (ncount - nRefer) && ncount < nFirstNum)
		{
			tVarInfoTrack[0].var_type = MP_VAR_P;
			tVarInfoTrack[0].var_no = (nindex % nPointNum + 20);
			tVarInfoTrack[0].val.p.dtype = MP_ROBO_DTYPE;
			tVarInfoTrack[0].val.p.uf_no = 0;//该参数无意义
			tVarInfoTrack[0].val.p.tool_no = 1;
			tVarInfoTrack[0].val.p.fig_ctrl = 4;
			tVarInfoTrack[0].val.p.data[0] = (long)(vtRealWeldCoors[ncount].dX * 1000);
			tVarInfoTrack[0].val.p.data[1] = (long)(vtRealWeldCoors[ncount].dY * 1000);
			tVarInfoTrack[0].val.p.data[2] = (long)(vtRealWeldCoors[ncount].dZ * 1000);
			tVarInfoTrack[0].val.p.data[3] = (long)(vtRealWeldCoors[ncount].dRX * 10000);
			tVarInfoTrack[0].val.p.data[4] = (long)(vtRealWeldCoors[ncount].dRY * 10000);
			tVarInfoTrack[0].val.p.data[5] = (long)(vtRealWeldCoors[ncount].dRZ * 10000);
			tVarInfoTrack[0].val.p.data[6] = 0;//该参数无意义
			tVarInfoTrack[0].val.p.data[7] = 0;//该参数无意义	
			//速度
			tVarInfoTrack[1].var_type = MP_VAR_I;
			tVarInfoTrack[1].var_no = nindex % nPointNum + 20;
			tVarInfoTrack[1].val.i = lRobotvel;
			pRobotDriver->SetMultiVar_H(2, tVarInfoTrack);
			ncount++;
			nindex++;
		}
		if (nCountInt1 >= nFirstNum - 1)
		{
			pRobotDriver->m_cLog->Write("%s 赋值线程break!", strRobot);
			break;
		}

		if (pRobotDriver->m_eThreadStatus == INCISEHEAD_THREAD_STATUS_STOPPED)
		{
			XiMessageBox("机械臂急停");
			break;
		}
		Sleep(20);
		DoEvent();
	}
	m_ptUnit->RobotCheckDone();//检查前一步是否运行完成
	SaveWeldLengthData(pRobotDriver, m_pTraceModel->m_dCurrentWeldLen, dAbjustRz, nCountInt1 + m_pTraceModel->m_nWeldStepNo);
	pRobotDriver->m_cLog->Write("%s 赋值线程退出!", strRobot);
	if (nCountInt1 + m_pTraceModel->m_nWeldStepNo >= nFirstNum - 1)
	{
		return true;
	}
	else
	{
		return false;
	}
}

//bool CDiaphragmWeld::WeldLockBefor(CRobotDriverAdaptor* pRobotDriver,T_ROBOT_COORS tStartP, T_ROBOT_COORS tEndP,bool bIsTrack)
//{
//	if (!bIsTrack)
//	{
//		return true;
//	}
//	double dLen = TwoPointDis(tStartP.dX, tStartP.dY+ tStartP.dBY, tStartP.dZ, tEndP.dX,
//		tEndP.dY+tEndP.dBY, tEndP.dZ);
//	double ddirX = (tEndP.dX - tStartP.dX) / dLen;
//	double ddirY = ((tEndP.dY + tEndP.dBY) - (tStartP.dY + tStartP.dBY)) / dLen;
//	double ddirZ = (tEndP.dZ - tStartP.dZ) / dLen;
//	double dNormal = atan2(ddirY, ddirX) * 180 / 3.1415926 -90 * m_nRobotInstallDir;
//
//	tStartP.dRZ = pRobotDriver->DirAngleToRz(dNormal);
//	RobotCoordPosOffset(tStartP, dNormal + 90 * m_nRobotInstallDir, 50., 0.*m_nRobotInstallDir);
//	RobotCoordPosOffset(tStartP, dNormal, 50., 50.*m_nRobotInstallDir);
//	T_ROBOT_MOVE_SPEED tMove(2000,20,20);
//	if (!MoveByJob(pRobotDriver, tStartP, tMove, pRobotDriver->m_nExternalAxleType, "MOVL"))
//	{
//		XiMessageBox("未移动的安全位置");
//		return false;
//	}
//	RobotCoordPosOffset(tStartP, dNormal, -50., -50.*m_nRobotInstallDir);
//	if (!MoveByJob(pRobotDriver, tStartP, tMove, pRobotDriver->m_nExternalAxleType, "MOVL"))
//	{
//		XiMessageBox("未移动的安全位置");
//		return false;
//	}
//	CHECK_BOOL_RETURN(m_pScanInit->RealTimeTrackLockArcBefor(pRobotDriver));
//	return true;
//}
bool CDiaphragmWeld::WeldLockBefor(CRobotDriverAdaptor* pRobotDriver, vector<T_ROBOT_COORS> tCoors, vector<int> tCoorsType, vector<T_ROBOT_COORS>& tCoutCoors, bool bIsTrack)
{
	if (!bIsTrack || m_pTraceModel->m_nOpenTrackingPos == 0)
	{
		return true;
	}
	tCoutCoors.clear();
	if (tCoors.size() < 15)
	{
		XiMessageBox("数据有误");
		return false;
	}

	T_ROBOT_COORS tStartP = tCoors.at(0);
	for (size_t i = 0; i < tCoorsType.size(); i++)
	{
		if (E_WELD_TRACK_CHANGE_POSTURE & tCoorsType.at(i))
		{
			continue;
		}
		tStartP = tCoors[i];
		break;
	}
	RobotCoordPosOffset(tStartP, RzToDirAngle(tStartP.dRZ), m_dGunDownBackSafeDis / 2.0, m_dGunDownBackSafeDis / 2.0 * m_nRobotInstallDir);
	tCoutCoors.push_back(tStartP);
	RobotCoordPosOffset(tStartP, RzToDirAngle(tStartP.dRZ), -1 * m_dGunDownBackSafeDis / 2.0, -1 * m_dGunDownBackSafeDis / 2.0 * m_nRobotInstallDir);
	tCoutCoors.push_back(tStartP);

	for (size_t i = 0; i < tCoutCoors.size(); i++)
	{
		tCoutCoors.at(i).dY += tCoutCoors.at(i).dBY - tCoors.at(0).dBY;
	}

	return true;
}
bool CDiaphragmWeld::WeldLockBefor(CRobotDriverAdaptor* pRobotDriver, vector<T_ROBOT_COORS> tCoors, vector<T_ROBOT_COORS> &tCoutCoors, bool bIsTrack)
{
	if (!bIsTrack|| m_pTraceModel->m_nOpenTrackingPos == 0)
	{
		return true;
	}
	tCoutCoors.clear();
	if (tCoors.size() < 15)
	{
		XiMessageBox("数据有误");
		return false;
	}

	int nDirPointNum = 20;
	//if (tCoors.size() < nDirPointNum)
	{
		nDirPointNum = tCoors.size();
	}

	vector<XI_POINT> vtPoint;
	for (int i = 0; i < nDirPointNum; i++)
	{
		XI_POINT tp = { tCoors[i].dX,tCoors[i].dY + tCoors[i].dBY,tCoors[i].dZ };
		vtPoint.push_back(tp);
	}
	T_LINE_PARA	tLine = CalcLineParamRansac(vtPoint, 0.7);
	double dNormal = atan2(tLine.dDirY, tLine.dDirX) * 180 / 3.1415926 - 90 * m_nRobotInstallDir;

	T_ROBOT_COORS tStartP = tCoors.at(0);
	//if ( > 0)
	{
		tStartP = tCoors.at(15);
	}
	tStartP.dRZ = pRobotDriver->DirAngleToRz(dNormal);
	RobotCoordPosOffset(tStartP, dNormal, 30., 30. * m_nRobotInstallDir);
	tCoutCoors.push_back(tStartP);
	RobotCoordPosOffset(tStartP, dNormal, -30., -30. * m_nRobotInstallDir);
	/*tStartP.dRX = m_dNormalWeldRx;
	tStartP.dRY = m_dNormalWeldRy;
	tStartP.dRZ += m_dWeldDipAngle;*/
	tCoutCoors.push_back(tStartP);

	for (size_t i = 0; i < tCoutCoors.size(); i++)
	{
		tCoutCoors.at(i).dY += tCoutCoors.at(i).dBY - tCoors.at(0).dBY;
	}

    return true;
}



bool CDiaphragmWeld::CalThreeDotToCircle(const XI_POINT& tFirstPoint, const XI_POINT& tSecondPoint, const XI_POINT& tThirdPoint, T_SPACE_CIRCLE_PARAM& CircleParam)
{
	double dParaA = 0.0, dParaB = 0.0, dParaC = 0.0;
	double dParaD = 0.0, dParaE = 0.0, dParaF = 0.0;

	dParaA = 2 * (tSecondPoint.x - tFirstPoint.x);
	dParaB = 2 * (tSecondPoint.y - tFirstPoint.y);
	dParaC = pow(tSecondPoint.x, 2) + pow(tSecondPoint.y, 2) - pow(tFirstPoint.x, 2) - pow(tFirstPoint.y, 2);
	dParaD = 2 * (tThirdPoint.x - tSecondPoint.x);
	dParaE = 2 * (tThirdPoint.y - tSecondPoint.y);
	dParaF = pow(tThirdPoint.x, 2) + pow(tThirdPoint.y, 2) - pow(tSecondPoint.x, 2) - pow(tSecondPoint.y, 2);

	if ((dParaB * dParaD - dParaE * dParaA) != 0)
	{
		CircleParam.tCenterPoint.dCoorX = (dParaB * dParaF - dParaE * dParaC) / (dParaB * dParaD - dParaE * dParaA);
		CircleParam.tCenterPoint.dCoorY = (dParaD * dParaC - dParaA * dParaF) / (dParaB * dParaD - dParaE * dParaA);
		CircleParam.dRadius = sqrt(pow(CircleParam.tCenterPoint.dCoorX - tFirstPoint.x, 2) + pow(CircleParam.tCenterPoint.dCoorY - tFirstPoint.y, 2));
		return true;
	}
	else
	{
		CircleParam.tCenterPoint.dCoorX = tFirstPoint.x;
		CircleParam.tCenterPoint.dCoorY = tFirstPoint.y;
		CircleParam.dRadius = 99999.0;
		return false;
	}
}

bool CDiaphragmWeld::CalCircleThreePoint(T_SPACE_CIRCLE_PARAM& tCircle, vector<XI_POINT> vtFitOutputPoints, double dStepDis, vector<XI_POINT>& vtOutputPoints)
{
	vtOutputPoints.clear();
	//T_SPACE_CIRCLE_PARAM tCenter;
	if (vtFitOutputPoints.size() < 3)
	{
		XiMessageBox("输入数据有误");
		return false;
	}
	//	XI_POINT tFirstPoint;
	//	XI_POINT tSecondPoint;
		//XI_POINT tThirdPoint;
	if (!CalThreeDotToCircle(vtFitOutputPoints.at(0), vtFitOutputPoints.at(1), vtFitOutputPoints.at(2), tCircle))
	{
		return false;
	}
	tCircle.tCenterPoint.dCoorZ = (vtFitOutputPoints.at(0).z + vtFitOutputPoints.at(1).z + vtFitOutputPoints.at(2).z) / 3;
	XI_POINT tPtn;
	int nTotalPtnNum = 0;
	double dArcLen = 2.0 * PI * tCircle.dRadius;
	nTotalPtnNum = dArcLen / dStepDis;
	double dAngleInterval = 360.0 / (double)nTotalPtnNum;
	for (int i = 0; i < nTotalPtnNum; i++)
	{
		double dDirAngle = dAngleInterval * i;
		tPtn.x = tCircle.tCenterPoint.dCoorX + tCircle.dRadius * CosD(dDirAngle);
		tPtn.y = tCircle.tCenterPoint.dCoorY + tCircle.dRadius * SinD(dDirAngle);
		tPtn.z = tCircle.tCenterPoint.dCoorZ;
		vtOutputPoints.push_back(tPtn);
	}
	return true;

}
void CDiaphragmWeld::SaveEndpointData(CRobotDriverAdaptor* pRobotDriver, T_ROBOT_COORS tEndpoint, bool bIsStart)
{
	CString str;
	if (bIsStart)
	{
		str.Format(".\\ConfigFiles\\%s\\StartPoint.txt", pRobotDriver->m_strRobotName);
	}
	else
	{
		str.Format(".\\ConfigFiles\\%s\\EndPoint.txt", pRobotDriver->m_strRobotName);
	}

	ofstream Cout(str.GetBuffer(0));
	Cout << tEndpoint.dX << " " << tEndpoint.dY << " " << tEndpoint.dZ << " " << tEndpoint.dRX << " " << tEndpoint.dRY << " " << tEndpoint.dRZ
		<< " " << tEndpoint.dBX << " " << tEndpoint.dBY << " " << tEndpoint.dBZ;
	Cout.close();
}
void CDiaphragmWeld::LoadEndpointData(CRobotDriverAdaptor* pRobotDriver, T_ROBOT_COORS& tEndpoint, bool bIsStart)
{
	CString str;
	if (bIsStart)
	{
		str.Format(".\\ConfigFiles\\%s\\StartPoint.txt", pRobotDriver->m_strRobotName);
	}
	else
	{
		str.Format(".\\ConfigFiles\\%s\\EndPoint.txt", pRobotDriver->m_strRobotName);
	}

	ifstream Cout(str.GetBuffer(0));
	Cout >> tEndpoint.dX >> tEndpoint.dY >> tEndpoint.dZ >> tEndpoint.dRX >> tEndpoint.dRY >> tEndpoint.dRZ
		>> tEndpoint.dBX >> tEndpoint.dBY >> tEndpoint.dBZ;
	Cout.close();
}


void CDiaphragmWeld::SaveWarpNumber(CRobotDriverAdaptor* pRobotDriver,int nWarpNumber)
{
	CString str;
	str.Format(".\\ConfigFiles\\%s\\Pause.ini", pRobotDriver->m_strRobotName);
	COPini opini;
	opini.SetFileName(str);
	opini.SetSectionName("PauseInfo");
	opini.WriteString("WeldWarpNumber", nWarpNumber);
}

void CDiaphragmWeld::LoadWarpNumber(CRobotDriverAdaptor* pRobotDriver, int &nWarpNumber)
{
	CString str;
	str.Format(".\\ConfigFiles\\%s\\Pause.ini", pRobotDriver->m_strRobotName);
	COPini opini;
	opini.SetFileName(str);
	opini.SetSectionName("PauseInfo");
	opini.ReadString("WeldWarpNumber", &nWarpNumber);
}

void CDiaphragmWeld::SaveCurrentWarpNumber(CRobotDriverAdaptor* pRobotDriver, int nWarpNumber)
{
	CString str;
	str.Format(".\\ConfigFiles\\%s\\Pause.ini", pRobotDriver->m_strRobotName);
	COPini opini;
	opini.SetFileName(str);
	opini.SetSectionName("PauseInfo");
	opini.WriteString("CurrentWeldWarpNumber", nWarpNumber);
}



void CDiaphragmWeld::LoadCurrentWarpNumber(CRobotDriverAdaptor* pRobotDriver, int &nWarpNumber)
{
	CString str;
	str.Format(".\\ConfigFiles\\%s\\Pause.ini", pRobotDriver->m_strRobotName);
	COPini opini;
	opini.SetFileName(str);
	opini.SetSectionName("PauseInfo");
	opini.ReadString("CurrentWeldWarpNumber", &nWarpNumber);
}

void CDiaphragmWeld::SaveWeldLengthData(CRobotDriverAdaptor* pRobotDriver, double dWeldLength, double dAbjustRz, int nWeldStepNo)
{
	CString str;
	str.Format(".\\ConfigFiles\\%s\\Pause.ini", pRobotDriver->m_strRobotName);
	COPini opini;
	opini.SetFileName(str);
	opini.SetSectionName("PauseInfo");
	opini.WriteString("WeldLength", dWeldLength);
	opini.WriteString("WeldStepNo", nWeldStepNo);
	opini.WriteString("AbjustRz", dAbjustRz);
}

void CDiaphragmWeld::LoadWeldLengthData(CRobotDriverAdaptor* pRobotDriver, double& dWeldLength, double& dAbjustRz, int& nWeldStepNo,double &dVoerLength)
{
	CString str;
	str.Format(".\\ConfigFiles\\%s\\Pause.ini", pRobotDriver->m_strRobotName);
	COPini opini;
	opini.SetFileName(str);
	opini.SetSectionName("PauseInfo");
	opini.ReadString("WeldLength", &dWeldLength);
	opini.ReadString("WeldStepNo", &nWeldStepNo);
	opini.ReadString("AbjustRz", &dAbjustRz);
	opini.ReadString("VoerLength", &dVoerLength);
}

void CDiaphragmWeld::SaveWorkpieceType(CRobotDriverAdaptor* pRobotDriver)
{
	int nNo = 0;
	CString str;
	str.Format(".\\ConfigFiles\\%s\\Pause.ini", pRobotDriver->m_strRobotName);
	COPini opini;
	opini.SetFileName(str);
	opini.SetSectionName("PauseInfo");
	nNo = (int)m_eWorkPieceType;
	opini.WriteString("WorkPieceType", nNo);
}

void CDiaphragmWeld::LoadWorkpieceType(CRobotDriverAdaptor* pRobotDriver)
{
	int nNo = 0;
	CString str;
	str.Format(".\\ConfigFiles\\%s\\Pause.ini", pRobotDriver->m_strRobotName);
	COPini opini;
	opini.SetFileName(str);
	opini.SetSectionName("PauseInfo");
	opini.ReadString("WorkPieceType", &nNo);
	m_eWorkPieceType = (E_WORKPIECE_TYPE)nNo;
}
void CDiaphragmWeld::SaveWorkpieceNo(CRobotDriverAdaptor* pRobotDriver, int nWorkpieceNo)
{
	CString cstrTmp;
	COPini _opini;
	cstrTmp = DATA_PATH + m_ptUnit->GetUnitName() + "\\Pause.ini";
	_opini.SetFileName(cstrTmp);
	_opini.SetSectionName("PauseInfo");
	_opini.WriteString("WorkpieceNo", nWorkpieceNo);
}

void CDiaphragmWeld::LoadWorkpieceNo(CRobotDriverAdaptor* pRobotDriver, int& nWorkpieceNo)
{
	if (!m_ptUnit->m_bBreakPointContinue)
	{
		nWorkpieceNo = 0;
		return;
	}
	CString cstr;
	cstr = DATA_PATH + m_ptUnit->GetUnitName() + "\\Pause.ini";
	COPini opini;
	opini.SetFileName(cstr);
	opini.SetSectionName("PauseInfo");
	opini.ReadString("WorkpieceNo", &nWorkpieceNo);
}

void CDiaphragmWeld::InitWeldStartVal(CRobotDriverAdaptor* pRobotDriver)
{
	// 
	m_pTraceModel = m_pScanInit->m_pTraceModel;
	//
	m_pTraceModel->m_dWeldLen				= 0.0;
	m_pTraceModel->m_dCurrentWeldLen		= 0.0;
	m_pTraceModel->m_dChangeAngleTheshold	= 0.0;
	m_pTraceModel->m_dScanStartOffset		= 0.0;
	m_pTraceModel->m_dWeldEndPointOffset	= 0.0;
	m_pTraceModel->m_dDisSafeGunToEnd		= 4.0;
	m_pTraceModel->m_dMoveStepDis		    = 3.0;
	
	m_pTraceModel->m_bIfJudgeEndOpen		= FALSE;
	m_pTraceModel->m_bUpdatedWarpData		= FALSE;
	m_pTraceModel->m_bIsTrackingScanEnd		= FALSE;
	m_pTraceModel->m_bIsCloseTrackThread	= FALSE;
	m_pTraceModel->m_bIfOpenExternal		= TRUE;
	m_pTraceModel->m_bIfWeldToEndPoint		= FALSE;
	m_pTraceModel->m_bCameraFindWeldEnd		= FALSE;
	
	m_pTraceModel->m_nOpenTrackingPos	= 0;
	m_pTraceModel->m_nCloseTrackingPos	= 0;

	m_pTraceModel->m_eStartWrapAngleType	= E_WRAPANGLE_EMPTY_SINGLE;

	m_pTraceModel->dHandEyeDis				= m_dHandEyeDis;

	m_pScanInit->m_bOpenTrackingStatus		= FALSE;
	m_pScanInit->m_bFixedPointScan			= FALSE;							//定长搜索
	m_pScanInit->m_bIfOpenExternal			= m_pTraceModel->m_bIfOpenExternal;
	m_pScanInit->m_dScanStartWrapLength		= 0;

	T_ROBOT_COORS tCoord;
//	MP_USR_VAR_INFO tUsrVarInfo;
	vector<MP_USR_VAR_INFO> vtUsrVarInfo;
	vtUsrVarInfo.clear();
	vtUsrVarInfo.push_back(pRobotDriver->PrepareValData(1, 0));//运动步数
	vtUsrVarInfo.push_back(pRobotDriver->PrepareValData(2, 0));//跳出循环
	vtUsrVarInfo.push_back(pRobotDriver->PrepareValData(99, tCoord));//角度增量
	pRobotDriver->SetMultiVar_H(vtUsrVarInfo); // 发送
}

void CDiaphragmWeld::SetWeldTlyParam(T_WELD_PARA tWeldPara)
{
	m_ptUnit->SendWeldParam(*m_pIsArcOn, tWeldPara);
	m_pTraceModel->m_tWeldParam = tWeldPara;

	double dFlatHorComp = m_mdFlatHorComp[180.0];
	double dFlatHeightComp = m_mdFlatHeightComp[180.0];

	m_pTraceModel->m_dWeldVelocity = m_pTraceModel->m_tWeldParam.WeldVelocity;
	m_pScanInit->m_pTraceModel->m_dGunToEyeCompenX = m_pTraceModel->m_tWeldParam.CrosswiseOffset +dFlatHorComp;	// 外加内减
	m_pScanInit->m_pTraceModel->m_dFlatHorComp = dFlatHorComp;
	m_pScanInit->m_pTraceModel->m_dGunToEyeCompenZ = m_pTraceModel->m_tWeldParam.verticalOffset +dFlatHeightComp;		// 向上补加 向下补减
	m_pScanInit->m_pTraceModel->m_dGunToEyeCompenZ *= m_nRobotInstallDir;
	//倾角
	m_dWeldDipAngle = m_pTraceModel->m_tWeldParam.dWeldDipAngle * m_nRobotInstallDir;
	if (m_dWeldDipAngle > 15)
	{
		m_dWeldDipAngle = 15;
	}
	if (m_dWeldDipAngle < -15)
	{
		m_dWeldDipAngle = -15;
	}
	//包角参数
	CWrapAngleParam cWrapAngleParam(m_ptUnit->GetUnitName());
	WELD_WRAP_ANGLE tWrapAngelPara;
	tWrapAngelPara = cWrapAngleParam.InitWrapAngleParam();

	m_tWrapAngelParaNew = tWrapAngelPara;
	m_dParallelAdjust1 = fabs(tWrapAngelPara.m_dWrapdParallel);
	m_dVerticalAdjust1 = tWrapAngelPara.m_dWrapdVertical;
	m_dVelocityWarp = fabs(tWrapAngelPara.m_dWrapdVelocity);//包角速度

	m_dParallelAdjust2 = fabs(tWrapAngelPara.m_dWrapdParallel2);
	m_dVerticalAdjust2 = fabs(tWrapAngelPara.m_dWrapdVertical2);
	m_dVelocityWarp2 = fabs(tWrapAngelPara.m_dWrapdVelocity2);//包角速度
	if (m_dParallelAdjust2 < 30)//一次包角据起点长度
	{
		m_dParallelAdjust2 = 30.;
	}

	m_ptUnit->GetRobotCtrl()->m_cLog->Write("焊接参数;%s 板厚:%s，速度：%lf 补偿：%lf %lf",
		m_ptUnit->GetUnitName(),
		m_pTraceModel->m_tWeldParam.strWorkPeace,
		m_pTraceModel->m_dWeldVelocity,
		m_pTraceModel->m_dGunToEyeCompenX,
		m_pTraceModel->m_dGunToEyeCompenZ
	);

}

bool CDiaphragmWeld::CorrectCircled(double center[3], double radius, vector<XI_POINT> vtPoint)
{
	if (vtPoint.size() < 3)
	{
		return false;
	}

	int k_point_size = vtPoint.size();

	double** points = new double* [k_point_size];
	int i = 0;
	// assign points
	for (int i = 0; i < k_point_size; ++i) {
		points[i] = new double[3];
		points[i][0] = vtPoint.at(i).x;
		points[i][1] = vtPoint.at(i).y;
		points[i][2] = vtPoint.at(i).z;
	}
	// do something
	//correctCircle(center, radius, points, k_point_size, "result");
	// release
	for (int i = 0; i < k_point_size; ++i) {
		delete[] points[i];
		points[i] = 0;
	}
	delete[] points;
	points = 0;
	return true;
}
void CDiaphragmWeld::SetThreadStatus(E_INCISEHEAD_THREAD_STATUS eStatus)
{
	m_pRobotDriver->m_eThreadStatus = INCISEHEAD_THREAD_STATUS_START;
	g_eThreadStatus = eStatus;
	ofstream fileThreadStatus("Data\\ThreadStatus.txt");
	fileThreadStatus << eStatus;
}


bool CDiaphragmWeld::CorrectWarpDataFun(vector<TrackFilter_XYZ> vWeldCoors,vector<T_ROBOT_COORS> vtWarpCoors,int nWarpIndex,vector<T_ROBOT_COORS> &vtCorrectWarpCoors )
{
	if (vWeldCoors.size() < 10|| vtWarpCoors.size()<4)
	{
		//已修改
		XUI::MesBox::PopInfo("CorrectWarpDataFun 数据有误：{0} {0}", vWeldCoors.size(), vtWarpCoors.size());
		//XiMessageBox("CorrectWarpDataFun 数据有误：%d %d", vWeldCoors.size(), vtWarpCoors.size());
		return false;
	}
	CString str;
	str.Format(".\\WeldData\\WeldRobotLeft\\%d_", nWarpIndex);
	string strfile;
	strfile = str.GetBuffer(0);
	SaveTrackFilterCoors(vWeldCoors, strfile +"CorrectTrackThoeryCoors.txt");
	SaveDataRobotCoors(vtWarpCoors, strfile+"CorrectbeforWarpCoors.txt");
	int nBeforNum = vtCorrectWarpCoors.size();
	
	vector<XI_POINT> p_;
	vector<XI_POINT> filletP_;

	vector<XI_POINT> vtRobotPostureCoors;
	vector<XI_POINT> resposture;
	int n = 0;
	for (n = 0; n < vWeldCoors.size() ;n++)
	{
		XI_POINT tp;
		tp.x = vWeldCoors.at(n).x_;
		tp.y = vWeldCoors.at(n).y_;
		tp.z = vWeldCoors.at(n).z_;
		p_.push_back(tp);
	}

	for (n = 0; n < vtWarpCoors.size(); n++)
	{
		XI_POINT tp,tpos;
		tp.x = vtWarpCoors.at(n).dX;
		tp.y = vtWarpCoors.at(n).dY+vtWarpCoors.at(n).dBY;
		tp.z = vtWarpCoors.at(n).dZ;
		filletP_.push_back(tp);

		tpos.x = vtWarpCoors.at(n).dRX;
		tpos.y = vtWarpCoors.at(n).dRY;
		tpos.z = vtWarpCoors.at(n).dRZ;

		vtRobotPostureCoors.push_back(tpos);
	}
	// 包角转角数据放入修正理论数据，算法使用

	p_.push_back(filletP_.at(1));

	correctFilletError a(p_, filletP_, 3, 2, 3,0.4,0.98);
	a.getResNew();
	vector<XI_POINT> respos = a.GetGdiff(vtRobotPostureCoors, resposture, m_nRobotInstallDir);
//	E_ROBOT_CAR_COORS_ERROR tMoveDir;
	vector<T_ROBOT_COORS> tWarpCoors;
	vtCorrectWarpCoors.clear();
	for (n = 0; n < respos.size(); n++)
	{
		T_ROBOT_COORS tCoor, dOutCoor;
		tCoor.dX = respos.at(n).x;
		tCoor.dY = respos.at(n).y;
		tCoor.dZ = respos.at(n).z;
		tCoor.dRX = resposture.at(n).x;
		tCoor.dRY = resposture.at(n).y;
		tCoor.dRZ = resposture.at(n).z;
		
		
		if (!CalcRobotAndMachinePos(m_pRobotDriver, tCoor, dOutCoor))
		{
			return false;
		}
		tWarpCoors.push_back(tCoor);
		vtCorrectWarpCoors.push_back(dOutCoor);

	}
	int nAfterNum = vtCorrectWarpCoors.size();
	SaveDataRobotCoors(tWarpCoors, strfile + "CorrectAfterWarpCoors.txt");
	SaveDataRobotCoors(vtCorrectWarpCoors, strfile + "CorrectAfterWarpRobotCoors.txt");
	if (nBeforNum>2* nAfterNum || nAfterNum > 2 * nBeforNum)
	{	
		//已修改
		XUI::MesBox::PopInfo("包角修正数据数量偏差过大：nBeforNum:{0} nAfterNum:{0}", nBeforNum, nAfterNum);
		//XiMessageBox("包角修正数据数量偏差过大：nBeforNum:%d nAfterNum:%d", nBeforNum, nAfterNum);
		return false;
	}

	return true;
}


bool CDiaphragmWeld::CorrectWarpDataFun(vector<TrackFilter_XYZ> vWeldCoors, vector<T_ROBOT_COORS> vtWarpCoors, int nWarpIndex, vector<T_ROBOT_COORS>& vtCorrectWarpCoors, vector<int>& vtCorrectWarpCoorsType)
{
	if (E_WRAPANGLE_ONCE != m_pTraceModel->m_eStartWrapAngleType)
	{
		return true;
	}
	if (vWeldCoors.size() > 60)
	{
		vWeldCoors.erase(vWeldCoors.begin(), vWeldCoors.begin()+(vWeldCoors.size()-60));
	}
	if (!CorrectWarpDataFun(vWeldCoors, vtWarpCoors, nWarpIndex, vtCorrectWarpCoors))
	{
		XiMessageBox("包角修正失败");
		return false;
	}

	if (vtCorrectWarpCoors.size() != vtCorrectWarpCoorsType.size())
	{
		if (vtCorrectWarpCoors.size() > vtCorrectWarpCoorsType.size())
		{
			int ndiffnum = (vtCorrectWarpCoors.size() - vtCorrectWarpCoorsType.size());
			for (int i = 0; i < ndiffnum; i++)
			{
				vtCorrectWarpCoorsType.push_back(E_WELD_TRACK);
			}
		}
		else
		{
			int ndiffnum = -1*(vtCorrectWarpCoors.size() - vtCorrectWarpCoorsType.size());
			vtCorrectWarpCoorsType.erase(vtCorrectWarpCoorsType.end() - ndiffnum, vtCorrectWarpCoorsType.end());
		}
	}
	return true;
}

bool CDiaphragmWeld::AddSafeDownGunPos(vector<T_ROBOT_COORS>& vCoors, vector<int>& vnCoorType, double dAdjustDis, double dPieceHight)
{
	if (vCoors.size() < 1 || vnCoorType.size() < 1)
	{
		XUI::MesBox::PopOkCancel("添加收下枪函数入参有误：{0} {1}", vCoors.size(), vnCoorType.size());
		return false;
	}
	T_ROBOT_COORS tCoorSafe = vCoors.at(0);
	tCoorSafe.dX += dAdjustDis * CosD(m_pRobotDriver->RzToDirAngle(tCoorSafe.dRZ));
	tCoorSafe.dY += dAdjustDis * SinD(m_pRobotDriver->RzToDirAngle(tCoorSafe.dRZ));
	tCoorSafe.dZ = dPieceHight + m_dGunDownBackSafeDis * m_pRobotDriver->m_nRobotInstallDir;
	vCoors.insert(vCoors.begin(), tCoorSafe);
	vnCoorType.insert(vnCoorType.begin(), E_TRANSITION_POINT);
	tCoorSafe = vCoors.at(vCoors.size() - 1);
	tCoorSafe.dX += dAdjustDis * CosD(m_pRobotDriver->RzToDirAngle(tCoorSafe.dRZ));
	tCoorSafe.dY += dAdjustDis * SinD(m_pRobotDriver->RzToDirAngle(tCoorSafe.dRZ));
	tCoorSafe.dZ = dPieceHight + m_dGunDownBackSafeDis * m_pRobotDriver->m_nRobotInstallDir;
	vCoors.push_back(tCoorSafe);
	vnCoorType.push_back(E_TRANSITION_POINT);
	return true;
}


bool CDiaphragmWeld::DecomposeWorldCoordinates(CRobotDriverAdaptor* pRobotCtrl, vector<T_ROBOT_COORS>& vtCoutCoors, vector<T_ROBOT_COORS> vtCinCoors, bool bIsLockExAxis, double dExAixsPos)
{
	if (vtCinCoors.size() < 1)
	{
		//已修改
		XUI::MesBox::PopInfo("分解世界坐标入参有误：{0}", vtCinCoors.size());
		//XiMessageBox("分解世界坐标入参有误：%d", vtCinCoors.size());
		return false;
	}
	T_ROBOT_COORS tCoutRobotCoor, tCoutRobotCoor1, tTrackCamCoors;
	vtCoutCoors.clear();
	for (int n = 0; n < vtCinCoors.size(); n++)
	{
		T_ROBOT_COORS WorldCoors = vtCinCoors[n];
		if (!CalcRobotAndMachinePos(pRobotCtrl, WorldCoors, tCoutRobotCoor))
		{
			XiMessageBox("目标点转化坐标失败");
			return false;
		}

		tCoutRobotCoor1 = tCoutRobotCoor;
		if (!pRobotCtrl->MoveToolByWeldGun(tCoutRobotCoor, pRobotCtrl->m_tTools.tGunTool, tCoutRobotCoor1,
			pRobotCtrl->m_tTools.tCameraTool, tTrackCamCoors))
		{
			XiMessageBox("枪尖到相机坐标转化坐标失败");
			return false;
		}
		vtCoutCoors.push_back(tTrackCamCoors);
	}
	if (bIsLockExAxis)
	{
#ifdef SINGLE_ROBOT
		double dExAxis = vtCoutCoors.at(0).dBY;
		if (dExAixsPos != -99999.0)
		{
			dExAxis = dExAixsPos;
		}
		for (int n = 0; n < vtCoutCoors.size(); n++)
		{
			tTrackCamCoors = vtCoutCoors.at(n);
			tTrackCamCoors.dY = tTrackCamCoors.dBY + tTrackCamCoors.dY - dExAxis;
			tTrackCamCoors.dBY = dExAxis;
			vtCoutCoors.at(n) = tTrackCamCoors;
		}
#else
		double dExAxis = vtCoutCoors.at(0).dBX;
		if (dExAixsPos != -9999.0)
		{
			dExAxis = dExAixsPos;
		}
		for (int n = 0; n < vtCoutCoors.size(); n++)
		{
			tTrackCamCoors = vtCoutCoors.at(n);
			tTrackCamCoors.dX = tTrackCamCoors.dBX + tTrackCamCoors.dX - dExAxis;
			tTrackCamCoors.dBX = dExAxis;
			vtCoutCoors.at(n) = tTrackCamCoors;
		}
#endif // 

	}
	return true;
}

//void CDiaphragmWeld::GenerateWrapBoundTrack(T_ROBOT_COORS tWrapStart, T_ROBOT_COORS tWrapEnd, vector<T_ROBOT_COORS>& vtRobotCoor, vector<int>& vtPtType, double dInPercentStart, double dInPercentEnd, double dBackGunPercent, double dDirRotation)
//{
//	// 跳枪终点大车和机械臂Y的补偿
//	double dDeltaY = tWrapEnd.dBY - tWrapStart.dBY;
//	vtRobotCoor.clear();
//	vtPtType.clear();
//	T_ROBOT_COORS tInsertCoord;
//	XiAlgorithm tAlg;
//	double dDirWrap, dBoardThick;
//	dDirRotation = fabs(dDirRotation);
//	if (dDirRotation > 90 || dDirRotation < 0)
//	{
//		dDirRotation = 0.0;
//	}
//	dBoardThick = TwoPointDis(tWrapStart.dX + tWrapStart.dBX, tWrapStart.dY + tWrapStart.dBY, tWrapStart.dZ,
//		tWrapEnd.dX + tWrapEnd.dBX, tWrapEnd.dY + tWrapEnd.dBY, tWrapEnd.dZ);
//	dDirWrap = tAlg.CalcArcAngle((tWrapEnd.dX + tWrapEnd.dBX) - (tWrapStart.dX + tWrapStart.dBX), (tWrapEnd.dY + tWrapEnd.dBY) - (tWrapStart.dY + tWrapStart.dBY));
//	//包角起点向终点偏移dInPercentStart个板厚
//	tInsertCoord = tWrapStart;
//	RobotCoordPosOffset(tInsertCoord, dDirWrap, dBoardThick * dInPercentStart);
//	tInsertCoord.dRZ += dDirRotation * m_nRobotInstallDir;
//	vtRobotCoor.push_back(tInsertCoord);
//	vtRobotCoor.push_back(tInsertCoord);
//	vtPtType.push_back(E_WELD_WRAP);
//	vtPtType.push_back(E_TRANSITION_ARCOFF_FAST_POINT);
//	/* 开始插入过渡点 */
//	// 抬至工件最高位置+20 转枪
//	tInsertCoord = tWrapStart;
//	RobotCoordPosOffset(tInsertCoord, dDirWrap + 180.0, dBoardThick * dBackGunPercent);
//	//20应改为变量,暂未考虑
//	tInsertCoord.dZ = m_dWorkPieceHighTop + m_nRobotInstallDir * 20;
//	vtRobotCoor.push_back(tInsertCoord);
//	vtPtType.push_back(E_TRANSITION_ARCOFF_FAST_POINT);
//	//180°分成2个90°逆时针旋转
//	/*
//	*	(0°)
//			→
//	*			(90°)
//			←
//	*	(180°)
//	*/
//	RobotCoordPosOffset(tInsertCoord, dDirWrap, dBoardThick * (dBackGunPercent + 0.5));
//	RobotCoordPosOffset(tInsertCoord, dDirWrap + 90.0 * m_nRobotInstallDir, fabs(m_pRobotDriver->m_tTools.tGunTool.dX) + 20.0/*m_dGunHeadToGunBarrel*/);
//	tInsertCoord.dRZ += 90.0 * m_nRobotInstallDir;
//	vtRobotCoor.push_back(tInsertCoord);
//	vtPtType.push_back(E_TRANSITION_ARCOFF_FAST_POINT);
//	tInsertCoord.dRZ += 90.0 * m_nRobotInstallDir;
//	RobotCoordPosOffset(tInsertCoord, dDirWrap, dBoardThick * (dBackGunPercent + 0.5));
//	RobotCoordPosOffset(tInsertCoord, dDirWrap - 90.0 * m_nRobotInstallDir, fabs(m_pRobotDriver->m_tTools.tGunTool.dX) + 20.0/*m_dGunHeadToGunBarrel*/);
//	vtRobotCoor.push_back(tInsertCoord);
//	vtPtType.push_back(E_TRANSITION_ARCOFF_FAST_POINT);
//	//下枪点
//	tInsertCoord.dZ = tWrapEnd.dZ + m_nRobotInstallDir * 20;
//	tInsertCoord.dRZ -= dDirRotation * m_nRobotInstallDir;
//	vtRobotCoor.push_back(tInsertCoord);
//	vtPtType.push_back(E_TRANSITION_ARCOFF_FAST_POINT);
//	//对枪点 （既是对枪点 也是焊接点）dInPercentEnd个板厚
//	tInsertCoord = tWrapEnd;
//	RobotCoordPosOffset(tInsertCoord, dDirWrap + 180.0, dBoardThick * dInPercentEnd);
//	tInsertCoord.dRZ -= dDirRotation * m_nRobotInstallDir;
//	tInsertCoord.dBY -= dDeltaY;
//	tInsertCoord.dY += dDeltaY;
//	vtRobotCoor.push_back(tInsertCoord);
//	//vtRobotCoor.push_back(tInsertCoord);
//	vtPtType.push_back(E_TRANSITION_ARCOFF_FAST_POINT);
//	//vtPtType.push_back(E_WELD_WRAP);
//	return;
//}

correctFilletError::correctFilletError(vector<XI_POINT> p_, vector<XI_POINT> filletP_, double diffD1_, double diffD2_, int minNum2_, double permissibleError_, double permissibleRate_)
	: p(p_), filletP(filletP_), diffD1(diffD1_), diffD2(diffD2_), minNum2(minNum2_)
{
	permissibleError = permissibleError_;
	permissibleRate = permissibleRate_;
}


vector<XI_POINT> correctFilletError::getRes()
{
	// 拟合跟踪点集的直线
	fittedLine();
	// 通过拟合直线与原始包角点重新计算包角点
	correctFillet();

	return diff();
}

void correctFilletError::getResNew()
{
	// 拟合跟踪点集的直线
	fittedLine();
	// 通过拟合直线与原始包角点重新计算包角点
	correctFillet();

}


double correctFilletError::distance(size_t a, size_t b)
{
	return distance(p[a], p[b]);
}


double correctFilletError::distance(const XI_POINT& a, const XI_POINT& b)
{
	return sqrt(
		(a.x - b.x) * (a.x - b.x) +
		(a.y - b.y) * (a.y - b.y) +
		(a.z - b.z) * (a.z - b.z));
}


void correctFilletError::fittedLine()
{
	size_t pSize = p.size();
	double maxFittingRate = 0;
	size_t maxRateStartPointIndex = 0;
	size_t maxRateEndPointIndex = 0;

	for (size_t i = 0; i < pSize; ++i) {
		for (size_t j = i + 1; j < pSize; ++j) {
			int t = 0;
			for (size_t n = 0; n < pSize; ++n) {
				if (disToLine(i, j, n) <= permissibleError) {
					++t;
				}
			}

			double rate = t * 1.0 / pSize;
			if (rate >= permissibleRate) {
				fittedStartPointIndex = i;
				fittedEndPointIndex = j;
				return;
			}
			else if (rate >= maxFittingRate) {
				maxFittingRate = rate;
				maxRateStartPointIndex = i;
				maxRateEndPointIndex = j;
			}
		}
	}

	fittedStartPointIndex = maxRateStartPointIndex;
	fittedEndPointIndex = maxRateEndPointIndex;

	return;
}


double correctFilletError::disToLine(size_t tempFittedStartPointIndex, size_t tempFittedEndPointIndex, size_t C)
{
	double a = distance(tempFittedEndPointIndex, C);
	double b = distance(tempFittedStartPointIndex, C);
	double c = distance(tempFittedStartPointIndex, tempFittedEndPointIndex);

	double p = (a + b + c) / 2;
	double s = sqrt(p * (p - a) * (p - b) * (p - c));

	return 2 * s / c;
}


void correctFilletError::correctFillet()
{
	// 计算包角开始点
	correctFilletP.push_back(find0and1(0));

	// 第一拐点
	correctFilletP.push_back(find0and1(1));

	// 计算第二个拐点
	correctFilletP.push_back(find2and3(2));

	// 计算结束点
	correctFilletP.push_back(find2and3(3));

	return;
}


XI_POINT correctFilletError::findPoint(const XI_POINT& start, const XI_POINT& end, double dis)
{
	// start点与end点间的距离 
	double d = distance(start, end);

	double x_change = (end.x - start.x) * dis / d;
	double y_change = (end.y - start.y) * dis / d;
	double z_change = (end.z - start.z) * dis / d;

	XI_POINT p;
	p.x = start.x + x_change;
	p.y = start.y + y_change;
	p.z = start.z + z_change;

	return p;
}


XI_POINT correctFilletError::find0and1(size_t n)
{
	double a = distance(p[fittedEndPointIndex], filletP[n]);
	double b = distance(p[fittedStartPointIndex], filletP[n]);
	double c = distance(p[fittedStartPointIndex], p[fittedEndPointIndex]);

	double t = (a + b + c) / 2;
	double s = sqrt(t * (t - a) * (t - b) * (t - c));
	double h = 2 * s / c;

	double d = sqrt(b * b - h * h);
	return findPoint(p[fittedStartPointIndex], p[fittedEndPointIndex], d);
}


XI_POINT correctFilletError::find2and3(size_t n)
{
	double d = distance(filletP[0], filletP[1]);
	double zD = filletP[n].z - filletP[1].z;
	double z = correctFilletP[1].z + zD;
	double r1 = sqrt(pow(distance(filletP[0], filletP[n]), 2) - pow((z - correctFilletP[0].z), 2));
	double r2 = sqrt(pow(distance(filletP[1], filletP[n]), 2) - pow(zD, 2));

	// 如果用于定位的两圆不相交，返回输入点
	if (d >= r1 + r2 || d < fabs(r1 - r2))
		return filletP[n];

	XI_POINT p1;
	XI_POINT p2;
	p1.z = z;
	p2.z = z;

	double a = (r1 * r1 - r2 * r2 + d * d) / (2.0 * d);
	double h = sqrt(r1 * r1 - a * a);

	double x1 = correctFilletP[0].x + a * (correctFilletP[1].x - correctFilletP[0].x) / d + h * (correctFilletP[1].y - correctFilletP[0].y) / d;
	double y1 = correctFilletP[0].y + a * (correctFilletP[1].y - correctFilletP[0].y) / d - h * (correctFilletP[1].x - correctFilletP[0].x) / d;

	double x2 = correctFilletP[0].x + a * (correctFilletP[1].x - correctFilletP[0].x) / d - h * (correctFilletP[1].y - correctFilletP[0].y) / d;
	double y2 = correctFilletP[0].y + a * (correctFilletP[1].y - correctFilletP[0].y) / d + h * (correctFilletP[1].x - correctFilletP[0].x) / d;

	p1.x = x1;
	p1.y = y1;
	p2.x = x2;
	p2.y = y2;

	if (distance(p1, filletP[n]) >= distance(p2, filletP[n])) {
		return p2;
	}
	else {
		return p1;
	}
}


vector<XI_POINT> correctFilletError::diff()
{
	vector<XI_POINT> res;

	// 第一边
	res.push_back(correctFilletP[0]);
	double d1 = distance(correctFilletP[0], correctFilletP[1]);
	size_t i = 1;
	while (i * diffD1 <= (d1 - 2 * diffD1 / 3)) {
		res.push_back(findPoint(correctFilletP[0], correctFilletP[1], i * diffD1));
		++i;

	}

	// 第二边
	res.push_back(correctFilletP[1]);
	double d2 = distance(correctFilletP[1], correctFilletP[2]);
	int num = floor((d2 - diffD2 / 2) / diffD2) + 2;

	if (num >= minNum2) {
		i = 1;
		while (i * diffD2 <= (d2 - diffD2 / 2)) {
			res.push_back(findPoint(correctFilletP[1], correctFilletP[2], i * diffD2));
			++i;
		}
	}
	else {
		double diffD2_ = d2 / (minNum2 - 1);
		i = 1;
		while (i * diffD2_ <= (d2 - diffD2_ / 3)) {
			res.push_back(findPoint(correctFilletP[1], correctFilletP[2], i * diffD2_));
			++i;
		}
	}

	res.push_back(correctFilletP[2]);

	// 第三边
	double d3 = distance(correctFilletP[2], correctFilletP[3]);
	i = 1;
	while (i * diffD1 <= (d3 - 2 * diffD1 / 3)) {
		res.push_back(findPoint(correctFilletP[2], correctFilletP[3], i * diffD1));
		++i;
	}
	res.push_back(correctFilletP[3]);

	return res;
}vector<XI_POINT> correctFilletError::GetGdiff(vector<XI_POINT> vtRobotPostureCoors, vector<XI_POINT>& vtRobotPosture, int nRobotDir)
{
	vector<XI_POINT> res;
	vtRobotPosture.clear();
	// 第一边
	res.push_back(correctFilletP[0]);
	double d1 = distance(correctFilletP[0], correctFilletP[1]);
	size_t i = 1;
	while (i * diffD1 <= (d1 - 2 * diffD1 / 3)) {
		res.push_back(findPoint(correctFilletP[0], correctFilletP[1], i * diffD1));
		++i;

	}
	int n = 0;
	for (n = 0; n < res.size(); n++)
	{
		vtRobotPosture.push_back(vtRobotPostureCoors.at(0));
	}

	// 第二边
	res.push_back(correctFilletP[1]);
	vtRobotPosture.push_back(vtRobotPostureCoors.at(1));

	double d2 = distance(correctFilletP[1], correctFilletP[2]);
	int num = floor((d2 - diffD2 / 2) / diffD2) + 2;

	if (num >= minNum2) {
		i = 1;
		while (i * diffD2 <= (d2 - diffD2 / 2)) {
			res.push_back(findPoint(correctFilletP[1], correctFilletP[2], i * diffD2));
			++i;
		}
	}
	else {
		double diffD2_ = d2 / (minNum2 - 1);
		i = 1;
		while (i * diffD2_ <= (d2 - diffD2_ / 3)) {
			res.push_back(findPoint(correctFilletP[1], correctFilletP[2], i * diffD2_));
			++i;
		}
	}


	res.push_back(correctFilletP[2]);

	int nSize = vtRobotPosture.size();
	int nPointNum = res.size() - vtRobotPosture.size();
	double dRZdiff = 90.0 / nPointNum;
	for (n = 1; n <= nPointNum; n++)
	{
		XI_POINT tp = vtRobotPostureCoors.at(1);
		tp.z = tp.z + dRZdiff * n * nRobotDir;
		vtRobotPosture.push_back(tp);
	}

	// 第三边
	double d3 = distance(correctFilletP[2], correctFilletP[3]);
	i = 1;
	while (i * diffD1 <= (d3 - 2 * diffD1 / 3)) {
		res.push_back(findPoint(correctFilletP[2], correctFilletP[3], i * diffD1));
		++i;
	}
	res.push_back(correctFilletP[3]);

	nPointNum = res.size() - vtRobotPosture.size();
	for (n = 0; n < nPointNum; n++)
	{
		vtRobotPosture.push_back(vtRobotPostureCoors.at(3));
	}

	return res;
}




