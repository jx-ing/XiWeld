#include "stdafx.h"
#include "AdjustTrack.h"
#include "Apps/PLib/CtrlUnit/CUnit.h"
#include "LocalFiles/ExLib/ALGO/include/GeometricEntitiesSpf.h"
#include "LocalFiles/ExLib/ALGO/include/getGenDate.h"

RTT::AdjustTrack::AdjustTrack()
{
}

RTT::AdjustTrack::~AdjustTrack()
{
	DELETE_POINTER_ARRAY(m_ptPlaneContourInfo);
}

bool RTT::AdjustTrack::loadPara()
{
	XiBase::COPini opini;
	opini.SetFileName("ConfigFiles\\RTT.ini");
	opini.SetSectionName("Base");
	opini.ReadString("DisLimitForCalDoTracking ", &m_dDisLimitForCalDoTracking);
	opini.ReadString("AngleLimitForCalDoTracking ", &m_dAngleLimitForCalDoTracking);
	opini.ReadString("EnableChangeRZ ", &m_bEnableChangeRZ);
	return opini.CheckRWIsSuc();
}

void RTT::AdjustTrack::setUnit(CUnit& cUnit)
{
	m_pUnit = &cUnit;
	m_pRobot = m_pUnit->GetRobotCtrl();
	m_cXiRobotAbsCoorsTrans = m_pUnit->GetRobotCtrl()->m_cXiRobotAbsCoorsTrans;
}

void RTT::AdjustTrack::initTrack(PartData::WeldTrack& cWeldTrack)
{
	m_pWeldTrack = &cWeldTrack;
}

bool RTT::AdjustTrack::initBeforeTrack()
{
	m_nAdjustTimes = 0;
	E_DOTRACKING_CONTOUR_TYPE eDoTrackingContourType = E_OUTTERCONTOUR_TRACKING;	// 内外轮廓
	E_DOTRACKING_BEVEL_TYPE eDoTrackingBevelType = E_BEVEL_UP_TRACKING;			// 上下坡口


	DELETE_POINTER_ARRAY(m_ptPlaneContourInfo);
	m_ptPlaneContourInfo = new PlaneContourInfo[100];

	//拟合实体
	if (!calcTrackEntity()) return false;
	if (!makeEntitiesClose()) return false;
	CopyFile(getDataFileRootFolder() + "PlaneContourInfo.txt", getDataFileRootFolder() + "PlaneContourInfoFirst.txt", FALSE);
	if (!makesureStratPnt()) return false;
	if (!loadClosedEntities()) return false;

	DELETE_POINTER_ARRAY(m_ptPlaneContourInfo);

	auto vtOldCoord = m_pWeldTrack->getTrack();
	auto vtOld3DPnts = m_pWeldTrack->toWorld<CvPoint3D64f>();
	if (!calcClosedEntitiesZ(vtOldCoord, vtOld3DPnts)) return false;
	if (!loadClosedTrack(vtOldCoord, vtOld3DPnts)) return false;

	/*if (IDOK != XiMessageBox("确认轨迹正确再点击此弹窗"))
	{
		return false;
	}*/

	//对于焊接，认为外轮廓和坡口线是重合的
	m_vtBevelPoints = m_vtPartOutPoints;
	m_vnBevelEntityNo = m_vnPartOutEntityNo;
	m_vtTeachRobotWeldPoints = m_vtPartOutPoints;
	m_vtOrgRobotWeldPoints = m_vtTeachRobotWeldPoints;

	if (!GetTrackingInciseData(eDoTrackingContourType, eDoTrackingBevelType,
		m_vtContourEntities, m_vtPartOutPoints, m_vnPartOutEntityNo,
		m_vtBevelPoints, m_vnBevelEntityNo,
		m_vvtInciseEntityWithBevelPoints, m_vnInciseEntityWithBevelNo, m_vvtInciseEntityPointAdjustDir, m_vdInciseToBevelDis,
		m_vvtBevelEntityPoints, m_vnOutBevelEntityNo))
	{
		return false;
	}

	T_XIGEO_PLANE_PARA tContourPlanePara = calcContourPlanePara();
	CalcIfDoTracking(tContourPlanePara, m_vtContourEntities, m_vnBevelEntityNo, m_vtDoTrackPara);

	// 计算调整方向
	CalBevelAdjustDirForTracking(m_vtBevelPoints, m_vvtInciseEntityWithBevelPoints, m_vvtInciseEntityPointAdjustDir, m_vtBevelAdjustDir);

	// 初始化滤波输入容器 (每个实体号对应一个滤波输入集合，每个集合的初始时为空)
	m_mnvFilterInput.clear();
	for (int i = 0; i < m_vtDoTrackPara.size(); i++)
	{
		if (m_vtDoTrackPara[i].nEntityNo >= 0)
		{
			m_mnvFilterInput[m_vtDoTrackPara[i].nEntityNo].clear();
			writeLog("是否可跟踪：%4d %d %d %d", i,
				m_vtDoTrackPara[i].bIfDoTracking, m_vtDoTrackPara[i].nEntityNo, m_vtDoTrackPara[i].nEntityType);
		}
	}

	return true;
}

bool RTT::AdjustTrack::determineWhetherToProcessImg(T_ROBOT_COORS tImgCoord, CvPoint& tIdeal2DKeyPoint, CvPoint& tHorTrackingPointInImage, CvPoint& tVerTrackingPointInImage)
{
	auto tCurSeenEntity = FindCurSeenEntities(tImgCoord);
	int nIndex = tCurSeenEntity - m_vtDoTrackPara.data();
	tIdeal2DKeyPoint.x = m_vtTrackingPointInImage[nIndex].x;
	tIdeal2DKeyPoint.y = m_vtTrackingPointInImage[nIndex].y;
	tHorTrackingPointInImage.x = m_vtHorTrackingPointInImage[nIndex].x;
	tHorTrackingPointInImage.y = m_vtHorTrackingPointInImage[nIndex].y;
	tVerTrackingPointInImage.x = m_vtVerTrackingPointInImage[nIndex].x;
	tVerTrackingPointInImage.y = m_vtVerTrackingPointInImage[nIndex].y;
	return tCurSeenEntity->bIfDoTracking;
}

bool RTT::AdjustTrack::adjustTrack(T_ROBOT_COORS tImgCoord, T_ROBOT_COORS tSeenCoord)
{
	// 图片上看到的是什么实体
	T_DOTRACKING_PARA tSeenEntity = *FindCurSeenEntities(tImgCoord);

	//判断是否调整并输出滤波结果
	T_ALGORITHM_POINT tFilterOutPoint;//滤波后返回的坐标
	bool bFilterOutValid = false;//滤波成功/失败
	T_ALGORITHM_POINT tSeenPos{};
	tSeenPos.dCoorX = tSeenCoord.dX + tSeenCoord.dBX;
	tSeenPos.dCoorY = tSeenCoord.dY + tSeenCoord.dBY;
	tSeenPos.dCoorZ = tSeenCoord.dZ + tSeenCoord.dBZ;
	bool bNeedCalcAdjsut = JudgeToDoAdjust(tSeenPos, tSeenEntity, tFilterOutPoint, bFilterOutValid);

	//不调整则退出
	if (!bFilterOutValid || !bNeedCalcAdjsut)
	{
		return false;
	}

	// 最新的跟踪数据
	T_TEACH_PARA tTeachPara;
	tTeachPara.tTeachPoint.x = tFilterOutPoint.dCoorX;
	tTeachPara.tTeachPoint.y = tFilterOutPoint.dCoorY;
	tTeachPara.tTeachPoint.z = tFilterOutPoint.dCoorZ;
	tTeachPara.nTeachEntityNo = tSeenEntity.nEntityNo;
	tTeachPara.nTeachEntityType = tSeenEntity.nEntityType;
	tTeachPara.nTeachEntityOrder = -1;

	//调整轨迹
	auto tLastSendPnt = getLastSendPnt();//上次发送的坐标
	std::vector<T_POINT_3D>	vtNewRobotWeldPoints;//调整后的坡口轨迹
	T_POINT_3D tCapRobotPoint3D;
	tCapRobotPoint3D.x = tImgCoord.dX + tImgCoord.dBX;
	tCapRobotPoint3D.y = tImgCoord.dY + tImgCoord.dBY;
	tCapRobotPoint3D.z = tImgCoord.dZ + tImgCoord.dBZ;
	writeLog("修正轨迹计算输入：%.3lf %.3lf %.3lf %.3lf %.3lf %.3lf 实体：%d %d %d 交点：%.3lf %.3lf %.3lf",
		tLastSendPnt.x, tLastSendPnt.y, tLastSendPnt.z,
		tTeachPara.tTeachPoint.x, tTeachPara.tTeachPoint.y, tTeachPara.tTeachPoint.z,
		tTeachPara.nTeachEntityNo, tTeachPara.nTeachEntityType, tTeachPara.nTeachEntityOrder,
		tCapRobotPoint3D.x, tCapRobotPoint3D.y, tCapRobotPoint3D.z);
	auto llStartCalTime = XI_clock();
	AdjustBevelDataByTeachPointsForTrackingNew(
		m_vvtInciseEntityWithBevelPoints, m_vnInciseEntityWithBevelNo, m_vvtInciseEntityPointAdjustDir, m_vdInciseToBevelDis, m_vtDoTrackPara,
		m_vtOrgRobotWeldPoints, m_vtBevelAdjustDir, m_vtTeachRobotWeldPoints, tCapRobotPoint3D,
		tLastSendPnt, tTeachPara, vtNewRobotWeldPoints);
	writeLog("修正轨迹计算耗时：%dms", XI_clock() - llStartCalTime);

	//修改调整后的轨迹
	if (!m_pWeldTrack->adjustTrack(vtNewRobotWeldPoints, m_nLastAdjustStartPntNo, m_nLastAdjustEndPntNo))
	{
		writeLog("修正轨迹失败");
		return false;
	}
	m_pWeldTrack->saveTrack(getDataFileRootFolder() + "AdjustTrack//LastestAdjustTrack.txt");	
	m_nAdjustTimes++;
	saveAdjustTrack(vtNewRobotWeldPoints);
	{
		CRITICAL_AUTO_LOCK(m_cLock);
		m_vtTeachRobotWeldPoints = vtNewRobotWeldPoints;
	}
	writeLog("修正轨迹成功%d-%d", m_nLastAdjustStartPntNo, m_nLastAdjustEndPntNo);
	return true;
}

void RTT::AdjustTrack::setLastSendPnt(T_ROBOT_COORS tLastSendPnt, int nPntNo)
{
	m_nLastSendPntNo = nPntNo;
	CRITICAL_AUTO_LOCK(m_cPntLock);
	m_tLastSendPnt.x = tLastSendPnt.dX + tLastSendPnt.dBX;
	m_tLastSendPnt.y = tLastSendPnt.dY + tLastSendPnt.dBY;
	m_tLastSendPnt.z = tLastSendPnt.dZ + tLastSendPnt.dBZ;
}

T_POINT_3D RTT::AdjustTrack::getLastSendPnt()
{
	CRITICAL_AUTO_LOCK(m_cPntLock);
	return m_tLastSendPnt;
}

double RTT::AdjustTrack::calcNearestPointNormalDegree(T_ROBOT_COORS tSeenCoord)
{
	//互斥拷贝一份数据
	std::vector<T_POINT_3D> vtTeachRobotWeldPoints;
	{
		CRITICAL_AUTO_LOCK(m_cLock);
		vtTeachRobotWeldPoints = m_vtTeachRobotWeldPoints;
	}

	//找出最近点
	T_POINT_3D tPoint;
	tPoint.x = tSeenCoord.dX + tSeenCoord.dBX;
	tPoint.y = tSeenCoord.dY + tSeenCoord.dBY;
	tPoint.z = tSeenCoord.dZ + tSeenCoord.dBZ;
	int nIndex = FindNearestPoint(tPoint, vtTeachRobotWeldPoints);

	//前两个点视为和第三个点法向相同
	if (nIndex < 2)
	{
		nIndex = 2;
	}

	//最后两个点视为和倒数第三个点法向相同
	if (nIndex >= vtTeachRobotWeldPoints.size() - 2)
	{
		nIndex = vtTeachRobotWeldPoints.size() - 3;
	}

	//计算轨迹方向
	double dDirX = vtTeachRobotWeldPoints[nIndex + 1].x - vtTeachRobotWeldPoints[nIndex - 1].x
		+ vtTeachRobotWeldPoints[nIndex + 2].x - vtTeachRobotWeldPoints[nIndex - 2].x;
	double dDirY = vtTeachRobotWeldPoints[nIndex + 1].y - vtTeachRobotWeldPoints[nIndex - 1].y
		+ vtTeachRobotWeldPoints[nIndex + 2].y - vtTeachRobotWeldPoints[nIndex - 2].y;

	//计算法向角
	return atan2(dDirY, dDirX) * 180.0 / PI - 90.0;
}

bool RTT::AdjustTrack::supportUpdatingEndPntOrNot()
{
	return false;
}

bool RTT::AdjustTrack::updateEndPnt(T_ROBOT_COORS tEndPnt)
{
	return true;
}

CString RTT::AdjustTrack::getDataFileRootFolder() const
{
	return WELDDATA_PATH + m_pRobot->m_strRobotName + "\\Track\\";
}

bool RTT::AdjustTrack::AdjustMeasureCoors(
	CUnit& cUnit, 
	std::vector<T_ROBOT_COORS_IN_WORLD> vtMeasureRobotPoints,  
	std::vector<T_LINE_DIR_3D> vtWeldLineDir,
	E_CAM_ID eCamId, 
	std::vector<T_XIGEO_WELDLINE> vtCheckBlockPlate, 
	std::vector<T_ROBOT_COORS_IN_WORLD>& vtAdjustMeasureRobotPoints)
{
	setUnit(cUnit);
	if (!AdjustMeasureRobotCoors(
		vtMeasureRobotPoints,
		vtWeldLineDir,
		eCamId, 
		vtCheckBlockPlate, 
		vtAdjustMeasureRobotPoints))
	{
		return false;
	}
	return true;
}

void RTT::AdjustTrack::writeLog(const char* format, ...) const
{
	CString str;
	va_list args;
	__crt_va_start(args, format);
	str.FormatV(format, args);
	__crt_va_end(args);

	m_pRobot->m_cLog->Write("跟踪调整：" + str);
}

bool RTT::AdjustTrack::calcTrackEntity()
{
	//保存旧轨迹
	m_pWeldTrack->saveTrack(getDataFileRootFolder() + "InputTrack.txt");
	m_pWeldTrack->saveTrackWorld(getDataFileRootFolder() + "InputTrackWorld.txt");

	//生成世界坐标系下轨迹
	std::vector<CvPoint3D32f> vtCloudPoint = m_pWeldTrack->toWorld<CvPoint3D32f>();

	//分割为实体
	m_nEntityCount = ContourCloudClassify(vtCloudPoint.data(), vtCloudPoint.size(), m_ptPlaneContourInfo, "ContourCloudClassify");
	if (m_nEntityCount <= 0)
	{
		XiMessageBox("焊接轨迹分割异常");
		return false;
	}

	//计算起点所在实体的序号
	PlaneContourInfo** groupInfo = new PlaneContourInfo * [vtCloudPoint.size()];
	if (!ContourCloudGrouping(m_ptPlaneContourInfo, m_nEntityCount, vtCloudPoint.data(), groupInfo, vtCloudPoint.size(), "ContourCloudGrouping"))
	{
		XiMessageBox("焊接轨迹分割序号异常");
		return false;
	}
	m_nStartPntEntityNo = groupInfo[0] - m_ptPlaneContourInfo;

	//如果第一个实体起终点反了，交换一下
	//if (m_nEntityCount > 1)
	{
		auto& tFirstPnt = vtCloudPoint[0];
		double dDisBetweenStartAndFirstPnt = TwoPointDis(m_ptPlaneContourInfo[0].staPoint.x, m_ptPlaneContourInfo[0].staPoint.y, tFirstPnt.x, tFirstPnt.y);
		double dDisBetweenEndAndFirstPnt = TwoPointDis(m_ptPlaneContourInfo[0].endPoint.x, m_ptPlaneContourInfo[0].endPoint.y, tFirstPnt.x, tFirstPnt.y);
		if (dDisBetweenStartAndFirstPnt > dDisBetweenEndAndFirstPnt)
		{
			std::swap(m_ptPlaneContourInfo[0].staPoint, m_ptPlaneContourInfo[0].endPoint);
		}
	}

	//对其他实体重新排序
	for (int i = 1; i < m_nEntityCount; i++)
	{
		auto& tCurStartPnt = m_ptPlaneContourInfo[i].staPoint;
		auto& tCurEndPnt = m_ptPlaneContourInfo[i].endPoint;
		auto& tLastEndPnt = m_ptPlaneContourInfo[i-1].endPoint;
		double dDisBetweenStartAndLastEnd = TwoPointDis(tCurStartPnt.x, tCurStartPnt.y, tLastEndPnt.x, tLastEndPnt.y);
		double dDisBetweenEndAndLastEnd = TwoPointDis(tCurEndPnt.x, tCurEndPnt.y, tLastEndPnt.x, tLastEndPnt.y);
		if (dDisBetweenStartAndLastEnd > dDisBetweenEndAndLastEnd)
		{
			std::swap(m_ptPlaneContourInfo[i].staPoint, m_ptPlaneContourInfo[i].endPoint);
		}
	}

	//保存实体
	return savePlaneContourInfo();
}

bool RTT::AdjustTrack::makeEntitiesClose()
{
	gen::ProjectConfiguration projectconfiguration = gen::MaWeIShipyardInvertedWorkbenchProject;

	CString path = getDataFileRootFolder();
	XiBase::GetSystemPath(path);
	gen::JudgeCorrectFreeEdgeOfWorkpiece freeedgeofworkpiece;

	freeedgeofworkpiece.setDistanceBetweenTeachPoints(1.0);
	freeedgeofworkpiece.setShortEdgeScanningTrajectory(5.0);
	freeedgeofworkpiece.setResultpath(path);
	freeedgeofworkpiece.readworkpiece(path + "//PlaneContourInfo.txt");
	freeedgeofworkpiece.setContourClosureError(15.0);
	//freeedgeofworkpiece.outContourwithhole(path + "//InputEntities.txt");

	/*if (!m_bOnlyLineEntity && freeedgeofworkpiece.correctContour_TangentArc(45))
	{
		if(IDOK != XiMessageBox("新跟踪：圆弧修正相切失败, 是否继续"));
			return false;
	}*/


	if ((m_pWeldTrack->isClosed() && !freeedgeofworkpiece.correctContour(2))
		|| (!m_pWeldTrack->isClosed() && !freeedgeofworkpiece.correctUnclosedContour(2)))
	//if (!freeedgeofworkpiece.correctContour(2))
	//if (!freeedgeofworkpiece.correctUnclosedContour(2))
	{
		XiMessageBox("新跟踪：修正误差过大");
		return false;
	}


	freeedgeofworkpiece.perform();
	if (freeedgeofworkpiece.isDone()) 
	{
		writeLog("makeEntitiesClose计算成功");
		return true;
	}
	XiMessageBox("makeEntitiesClose计算失败");
	return false;
}

bool RTT::AdjustTrack::makesureStratPnt()
{
	//起点
	T_ROBOT_COORS tStartCoord = m_pWeldTrack->getPoint(0);
	CvPoint3D64f tStartPnt{};
	tStartPnt.x = tStartCoord.dX + tStartCoord.dBX;
	tStartPnt.y = tStartCoord.dY + tStartCoord.dBY;
	tStartPnt.z = tStartCoord.dZ + tStartCoord.dBZ;

	//读取起点所在实体的离散点数据，并找出距离起点最近的点
	int nNearestPntNo = -1;
	double dMinDis = 99999.0;
	std::vector<CvPoint3D32f> vtPnts;
	int nIndex = 0;
	CvPoint3D32f tPnt{};
	CString sFileName;
	sFileName.Format("结果//旧段%zd.txt", m_nStartPntEntityNo);
	auto file = fopen(getDataFileRootFolder() + sFileName, "r");
	while (EOF != fscanf(file, "%d%f%f%f",
		&nIndex, &tPnt.x, &tPnt.y, &tPnt.z))
	{
		vtPnts.push_back(tPnt);

		double dTemp = square(tStartPnt.x - tPnt.x)
			+ square(tStartPnt.y - tPnt.y);

		//小于阈值，说明当前点更近
		if (dTemp < dMinDis)
		{
			dMinDis = dTemp;
			nNearestPntNo = nIndex;
		}
	}
	fclose(file);
	if (nNearestPntNo < 0)
	{
		XiMessageBox("新跟踪：找不到距离最近的焊缝起点");
		return false;
	}

	//将实体数据缓存到容器
	std::vector<PlaneContourInfo> vtPlaneContourInfo(0);
	for (int i = 0; i < m_nEntityCount; i++)
	{
		vtPlaneContourInfo.push_back(m_ptPlaneContourInfo[i]);
	}

	//距离起点太近，起点所在实体的序号应为0
	if (nNearestPntNo < 4)
	{
		for (int i = 0; i < m_nEntityCount; i++)
		{
			int nRealNo = m_nStartPntEntityNo + i;
			if (nRealNo >= m_nEntityCount)
			{
				nRealNo -= m_nEntityCount;
			}
			m_ptPlaneContourInfo[i] = vtPlaneContourInfo[nRealNo];
		}
	}
	//距离终点太近，起点所在实体的序号应为最后一个
	else if (vtPnts.size() - nNearestPntNo < 4)
	{
		for (int i = 0; i < m_nEntityCount; i++)
		{
			int nRealNo = m_nStartPntEntityNo + i + 1;
			if (nRealNo >= m_nEntityCount)
			{
				nRealNo -= m_nEntityCount;
			}
			m_ptPlaneContourInfo[i] = vtPlaneContourInfo[nRealNo];
		}
	}
	//其他情况，需要打断当前实体，再对实体重新排序
	else
	{
		//先将未被分割的实体重新赋值
		DELETE_POINTER_ARRAY(m_ptPlaneContourInfo);
		m_ptPlaneContourInfo = new PlaneContourInfo[m_nEntityCount + 1];
		for (int i = 1; i < m_nEntityCount; i++)
		{
			int nRealNo = m_nStartPntEntityNo + i;
			if (nRealNo >= m_nEntityCount)
			{
				nRealNo -= m_nEntityCount;
			}
			m_ptPlaneContourInfo[i] = vtPlaneContourInfo[nRealNo];
		}

		//修改第一个实体
		m_ptPlaneContourInfo[0] = vtPlaneContourInfo[m_nStartPntEntityNo];
		m_ptPlaneContourInfo[0].staPoint = vtPnts[nNearestPntNo];
		m_ptPlaneContourInfo[0].midPoint = vtPnts[(vtPnts.size() - nNearestPntNo) / 2];
		m_ptPlaneContourInfo[0].endPoint = vtPnts.back();

		//修改最后一个实体
		m_ptPlaneContourInfo[m_nEntityCount] = vtPlaneContourInfo[m_nStartPntEntityNo];
		m_ptPlaneContourInfo[m_nEntityCount].staPoint = vtPnts[0];
		m_ptPlaneContourInfo[m_nEntityCount].midPoint = vtPnts[nNearestPntNo / 2];
		m_ptPlaneContourInfo[m_nEntityCount].endPoint = vtPnts[nNearestPntNo];

		//实体数目增加
		m_nEntityCount++;
	}

	//如果最后一个实体与前一个实体不连续，就删掉
	if (m_nEntityCount > 1
		&& 20.0 < TwoPointDis(m_ptPlaneContourInfo[m_nEntityCount - 1].staPoint.x, m_ptPlaneContourInfo[m_nEntityCount - 1].staPoint.y,
			m_ptPlaneContourInfo[m_nEntityCount - 2].endPoint.x, m_ptPlaneContourInfo[m_nEntityCount - 2].endPoint.y))
	{
		//m_nEntityCount--;
		writeLog("最后一个实体与前一个实体不连续，已删除");
	}

	//保存新的实体数据，重新使实体闭合
	return savePlaneContourInfo()
		&& makeEntitiesClose();
}

bool RTT::AdjustTrack::loadClosedEntities()
{
	m_vtContourEntities.clear();
	PlaneContourInfo tEntity;// { 0 };
	memset(&tEntity, 0, sizeof(tEntity));
	T_DXF_ENTITY_PARA tNewEntity{ 0 };
	int nUselessVar = 0;
	int nIndex = 0;
	auto file = fopen(getDataFileRootFolder() + "结果//GrayscaleResult_Entity2D.txt", "r");
	while (EOF != fscanf(file, "%f%f%f%f%f%f%d",
		&tEntity.staPoint.x, &tEntity.staPoint.y,
		&tEntity.endPoint.x, &tEntity.endPoint.y,
		&tEntity.midPoint.x, &tEntity.midPoint.y, &nUselessVar))
	{
		tEntity.midPoint.x = tEntity.midPoint.x + (tEntity.staPoint.x + tEntity.endPoint.x) / 2.0;
		tEntity.midPoint.y = tEntity.midPoint.y + (tEntity.staPoint.y + tEntity.endPoint.y) / 2.0;
		PlaneContourInfo2T_DXF_ENTITY_PARA(tEntity, nIndex, tNewEntity);
		m_vtContourEntities.push_back(tNewEntity);
		nIndex++;
	}
	::fclose(file);
	return true;
}

bool RTT::AdjustTrack::calcClosedEntitiesZ(const std::vector<T_ROBOT_COORS>& vtOldCoord, const std::vector<CvPoint3D64f>& vtOld3DPnts)
{
	for (size_t i = 0; i < m_vtContourEntities.size(); i++)
	{
		//找出距离起点最近的点
		int nNearestPntNo = -1;
		double dMinDis = 99999.0;
		for (size_t j = 0; j < vtOld3DPnts.size(); j++)
		{
			double dTemp = square(m_vtContourEntities[i].dStartX - vtOld3DPnts[j].x)
				+ square(m_vtContourEntities[i].dStartY - vtOld3DPnts[j].y);

			//小于阈值，说明当前点更近
			if (dTemp < dMinDis)
			{
				dMinDis = dTemp;
				nNearestPntNo = j;
			}
		}
		if (nNearestPntNo < 0)
		{
			XiMessageBox("新跟踪：找不到距离最近的焊缝实体");
			return false;
		}
		m_vtContourEntities[i].dStartZ = vtOld3DPnts[nNearestPntNo].z;

		//找出距离终点最近的点
		nNearestPntNo = -1;
		dMinDis = 99999.0;
		for (size_t j = 0; j < vtOld3DPnts.size(); j++)
		{
			double dTemp = square(m_vtContourEntities[i].dEndX - vtOld3DPnts[j].x)
				+ square(m_vtContourEntities[i].dEndY - vtOld3DPnts[j].y);

			//小于阈值，说明当前点更近
			if (dTemp < dMinDis)
			{
				dMinDis = dTemp;
				nNearestPntNo = j;
			}
		}
		if (nNearestPntNo < 0)
		{
			XiMessageBox("新跟踪：找不到距离最近的焊缝实体");
			return false;
		}
		m_vtContourEntities[i].dEndZ = vtOld3DPnts[nNearestPntNo].z;

		//中间点直接按均值
		m_vtContourEntities[i].dCenterZ = (m_vtContourEntities[i].dStartZ + m_vtContourEntities[i].dEndZ) / 2.0;
	}

	return true;
}

bool RTT::AdjustTrack::loadClosedTrack(const std::vector<T_ROBOT_COORS>& vtOldCoord, const std::vector<CvPoint3D64f>& vtOld3DPnts)
{
	//读取新轨迹
	m_vtPartOutPoints.clear();
	m_vnPartOutEntityNo.clear();
	std::vector<int> vnPntCount{};//每个实体及其之前实体的点数之和
	for (size_t i = 0; i < m_vtContourEntities.size(); i++)
	{
		int nPartOutPointsCountBefore = m_vnPartOutEntityNo.size();

		//读取单个实体的离散点
		T_POINT_3D tPnt{ 0 };//三维点
		int nIndex = 0;//序号
		CString sFileName;//文件名
		sFileName.Format("结果//%zd.txt", i);
		auto file = fopen(getDataFileRootFolder() + sFileName, "r");
		while (EOF != fscanf(file, "%d%lf%lf%lf",
			&nIndex, &tPnt.x, &tPnt.y, &tPnt.z))
		{
			m_vtPartOutPoints.push_back(tPnt);//保存三维点
			m_vnPartOutEntityNo.push_back(i);//保存实体序号
		}
		::fclose(file);

		//去掉除了最后一个实体之外的其余实体的最后一个离散点
		if (i < m_vtContourEntities.size() - 1)
		{
			m_vtPartOutPoints.pop_back();
			m_vnPartOutEntityNo.pop_back();
		}

		vnPntCount.push_back(m_vtPartOutPoints.size() - nPartOutPointsCountBefore);
	}

	//将世界坐标转换到机器人坐标系下，并将Z赋值
	auto llTime = XiBase::XI_clock();
	std::vector<T_ROBOT_COORS> vtInitWeldCoord(m_vtPartOutPoints.size());
	for (size_t i = 0; i < m_vtPartOutPoints.size(); i++)
	{
		//找出距离原始轨迹最近的点
		int nNearestPntNo = -1;
		double dMinDis = 99999.0;
		for (size_t j = 0; j < vtOld3DPnts.size(); j++)
		{
			auto& tAimCoord = m_vtPartOutPoints[i];
			double dTemp = square(tAimCoord.x - vtOld3DPnts[j].x)
				+ square(tAimCoord.y - vtOld3DPnts[j].y);

			//小于等于阈值，说明当前点更近
			if (dTemp <= dMinDis)
			{
				dMinDis = dTemp;
				nNearestPntNo = j;
			}
		}
		if (nNearestPntNo < 0)
		{
			XiMessageBox("新跟踪：找不到距离最近的初始机器人坐标");
			return false;
		}

		//z赋值
		m_vtPartOutPoints[i].z = vtOld3DPnts[nNearestPntNo].z + vtOldCoord[nNearestPntNo].dBZ;

		//转化到机器人坐标系下
		vtInitWeldCoord[i].dX = m_vtPartOutPoints[i].x - vtOldCoord[nNearestPntNo].dBX;
		vtInitWeldCoord[i].dY = m_vtPartOutPoints[i].y - vtOldCoord[nNearestPntNo].dBY;
		vtInitWeldCoord[i].dZ = m_vtPartOutPoints[i].z - vtOldCoord[nNearestPntNo].dBZ;
		vtInitWeldCoord[i].dRX = vtOldCoord[nNearestPntNo].dRX;
		vtInitWeldCoord[i].dRY = vtOldCoord[nNearestPntNo].dRY;
		vtInitWeldCoord[i].dRZ = vtOldCoord[nNearestPntNo].dRZ;
		vtInitWeldCoord[i].dBX = vtOldCoord[nNearestPntNo].dBX;
		vtInitWeldCoord[i].dBY = vtOldCoord[nNearestPntNo].dBY;
		vtInitWeldCoord[i].dBZ = vtOldCoord[nNearestPntNo].dBZ;
	}
	writeLog("计算初始轨迹的Z值耗时：%dms", XiBase::XI_clock() - llTime);


	if (m_bEnableChangeRZ && !changeRZForArc(vnPntCount, vtInitWeldCoord))
	{
		XiMessageBox("改变圆弧焊接姿态失败");
		return false;
	}

	//如果相邻RZ变化太大
	for (size_t i = 1; i < vtInitWeldCoord.size(); i++)
	{
		double dDeltaRZ = fabs(vtInitWeldCoord[i].dRZ - vtInitWeldCoord[i - 1].dRZ);
		if (dDeltaRZ > 15.0 && dDeltaRZ < 355.0)
		{
			XiMessageBox("新跟踪：理论轨迹的RZ变化太大");
			return false;
		}
	}

	//重置轨迹
	m_pWeldTrack->initTrack(vtInitWeldCoord);

#ifdef LOCAL_DEBUG
	return true;
#endif // LOCAL_DEBUG

	//从当前位置出发是否可以走完整个轨迹
	return isSafeTrack(m_pRobot->GetCurrentPulse(), vtInitWeldCoord);
}

bool RTT::AdjustTrack::changeRZForArc(const std::vector<int>& vnPntCount, std::vector<T_ROBOT_COORS>& vtInitWeldCoord)
{
	int nMaxCountOfChangedLine = 40;//直线段改变RZ的最大轨迹份数
	double dChangeRZOfEachLinePnt = 0.5;//每个直线轨迹点改变RZ的值

	//遍历实体，找到圆弧和直线相连的位置
	bool bFindNewArc = false;
	for (size_t i = 1; i < m_vtContourEntities.size(); i++)
	{
		//上一段和下一段是直线，并且这一段是圆弧
		if (i < m_vtContourEntities.size() - 1
			&& m_vtContourEntities[i - 1].eEntityType == E_DXF_ENTITY_TYPE_LINE
			&& m_vtContourEntities[i + 1].eEntityType == E_DXF_ENTITY_TYPE_LINE
			&& (m_vtContourEntities[i].eEntityType == E_DXF_ENTITY_TYPE_ARC_ANTICLOCKWISE
				|| m_vtContourEntities[i].eEntityType == E_DXF_ENTITY_TYPE_ARC_CLOCKWISE))
		{
			//直线段1提前变姿态份数
			int nLineCount1 = vnPntCount[i - 1] > nMaxCountOfChangedLine ? nMaxCountOfChangedLine : vnPntCount[i - 1];
			//直线段2提前变姿态份数
			int nLineCount2 = vnPntCount[i + 1] > nMaxCountOfChangedLine ? nMaxCountOfChangedLine : vnPntCount[i + 1];
			//保证直线段变姿态份数相同
			nLineCount1 = nLineCount1 > nLineCount2 ? nLineCount2 : nLineCount1;
			nLineCount2 = nLineCount2 > nLineCount1 ? nLineCount1 : nLineCount2;
			//直线段1提前变姿态每一份角度值
			double dChangeRZOnLine1 =
				m_vtContourEntities[i].eEntityType == E_DXF_ENTITY_TYPE_ARC_ANTICLOCKWISE ?
				dChangeRZOfEachLinePnt : -dChangeRZOfEachLinePnt;
			//直线段2提前变姿态每一份角度值
			double dChangeRZOnLine2 =
				m_vtContourEntities[i].eEntityType == E_DXF_ENTITY_TYPE_ARC_ANTICLOCKWISE ?
				dChangeRZOfEachLinePnt : -dChangeRZOfEachLinePnt;
			//角度变化总数
			double dAngle = nLineCount1 * dChangeRZOnLine1;

			//圆弧段第一个实体的点数序号
			int nFirstArcPntNo = 0;
			for (size_t j = 0; j < i; j++)
			{
				nFirstArcPntNo += vnPntCount[j];
			}

			//修改直线段1末尾的角度
			for (size_t j = 0; j < nLineCount1; j++)
			{
				int nRealPntNo = j + nFirstArcPntNo - nLineCount1;
				vtInitWeldCoord[nRealPntNo].dRZ = vtInitWeldCoord[nRealPntNo].dRZ + dChangeRZOnLine1 * j;
			}

			//修改圆弧段的角度
			for (size_t j = 0; j < vnPntCount[i]; j++)
			{
				int nRealPntNo = nFirstArcPntNo + j;
				vtInitWeldCoord[nRealPntNo].dRZ = vtInitWeldCoord[nRealPntNo].dRZ + dAngle;
			}

			//修改直线段2开始的角度
			for (size_t j = 0; j < nLineCount2; j++)
			{
				int nRealPntNo = nLineCount2 + nFirstArcPntNo + vnPntCount[i] - j - 1;
				vtInitWeldCoord[nRealPntNo].dRZ = vtInitWeldCoord[nRealPntNo].dRZ + dChangeRZOnLine1 * j;
			}
		}
		//上一段是直线，并且这一段是圆弧
		else if (m_vtContourEntities[i - 1].eEntityType == E_DXF_ENTITY_TYPE_LINE
			&& (m_vtContourEntities[i].eEntityType == E_DXF_ENTITY_TYPE_ARC_ANTICLOCKWISE
				|| m_vtContourEntities[i].eEntityType == E_DXF_ENTITY_TYPE_ARC_CLOCKWISE))
		{
			//直线段提前变姿态份数
			int nLineCount = vnPntCount[i - 1] > nMaxCountOfChangedLine ? nMaxCountOfChangedLine : vnPntCount[i - 1];
			//直线段提前变姿态每一份角度值
			double dChangeRZOnLine =
				m_vtContourEntities[i].eEntityType == E_DXF_ENTITY_TYPE_ARC_ANTICLOCKWISE ?
				dChangeRZOfEachLinePnt : -dChangeRZOfEachLinePnt;
			//圆弧段减少变姿态份数
			int nArcCount = vnPntCount[i] / 2 > nLineCount ? vnPntCount[i] / 2 : nLineCount;
			nArcCount = vnPntCount[i] < nArcCount ? vnPntCount[i] : nArcCount;
			//圆弧段减少变姿态每一份角度值
			double dChangeRZOnArc = nLineCount * dChangeRZOnLine / nArcCount;

			//圆弧段第一个实体的点数序号
			int nFirstArcPntNo = 0;
			for (size_t j = 0; j < i; j++)
			{
				nFirstArcPntNo += vnPntCount[j];
			}

			//修改直线段末尾的角度
			for (size_t j = 0; j < nLineCount; j++)
			{
				int nRealPntNo = j + nFirstArcPntNo - nLineCount;
				vtInitWeldCoord[nRealPntNo].dRZ = vtInitWeldCoord[nRealPntNo].dRZ + dChangeRZOnLine * j;
			}

			//修改圆弧段初始的角度
			for (size_t j = 0; j < nArcCount; j++)
			{
				int nRealPntNo = nFirstArcPntNo + nArcCount - j - 1;
				vtInitWeldCoord[nRealPntNo].dRZ = vtInitWeldCoord[nRealPntNo].dRZ + dChangeRZOnArc * j;
			}
		}
	}
	return true;
}

void RTT::AdjustTrack::saveAdjustTrack(const std::vector<T_POINT_3D>& vtNewPoints)
{
	CString sFileName;
	sFileName.Format(getDataFileRootFolder() + "AdjustTrack//AdjustTrack-%d.txt", m_nAdjustTimes);

	FILE* file;
	fopen_s(&file, sFileName, "w");
	for (size_t i = 0; i < vtNewPoints.size(); i++)
	{
		fprintf_s(file, "%4zd %12.3lf  %12.3lf  %12.3lf \n",
			i, vtNewPoints[i].x, vtNewPoints[i].y, vtNewPoints[i].z);
	}
	fclose(file);
}

bool RTT::AdjustTrack::isSafeTrack(T_ANGLE_PULSE tStartPulse, const std::vector<T_ROBOT_COORS>& vtTrack)
{
	//设置限制
	T_AXISUNIT tAxleUnit = m_pRobot->m_tAxisUnit;//脉冲参数
	long nMinLPulse = -1 == m_pRobot->m_nRobotInstallDir ? (long)(-60.0 / tAxleUnit.dLPulse) : (long)(-180.0 / tAxleUnit.dLPulse);//L轴最小值
	long nMaxLPulse = -1 == m_pRobot->m_nRobotInstallDir ? (long)(30.0 / tAxleUnit.dLPulse) : (long)(180.0 / tAxleUnit.dLPulse);//L轴最大值
	T_ANGLE_THETA tThetaThreshold = T_ANGLE_THETA(90.0, 45.0, 45.0, 50.0, 90.0, 100.0);//角度限制
	T_ANGLE_PULSE tAngleThreshold(	//脉冲限制
		(long)(tThetaThreshold.dThetaS / tAxleUnit.dSPulse), 
		(long)(tThetaThreshold.dThetaL / tAxleUnit.dLPulse),
		(long)(tThetaThreshold.dThetaU / tAxleUnit.dUPulse), 
		(long)(tThetaThreshold.dThetaR / tAxleUnit.dRPulse),
		(long)(tThetaThreshold.dThetaB / tAxleUnit.dBPulse), 
		(long)(tThetaThreshold.dThetaT / tAxleUnit.dTPulse),
		0, 0, 0);

	//求出每一个直角坐标的脉冲解
	vector<vector<T_ANGLE_PULSE>> vvtRobotPulse(vtTrack.size());	// 每个直角坐标 对应的关节坐标集合
	for (int i = 0; i < vtTrack.size(); i++)
	{
		if (!m_pRobot->RobotInverseKinematics(vtTrack[i], m_pRobot->m_tTools.tGunTool, vvtRobotPulse[i]))
		{
			XiMessageBox("连续运动轨迹求逆解失败!");
			return false;
		}
	}

	//把第一个坐标改为参考的起点位置
	vvtRobotPulse[0].clear();
	vvtRobotPulse[0].push_back(tStartPulse);

	//初始化可能脉冲轨迹的第一个点及其误差
	vector<vector<T_ANGLE_PULSE>> vvtTrackPulse(0);					// 多个可连续运动的关节坐标轨迹
	vector<T_ANGLE_PULSE> vtTotalPulse(0);							// 每个连续运动轨迹各轴累计脉冲				
	vector<vector<double>> vvtMaxAngleErr(vvtRobotPulse[0].size());	// 多个可连续运动的关节坐标轨迹
	for (int nTrackNo = 0; nTrackNo < vvtRobotPulse[0].size(); nTrackNo++) // 每组轨迹第一个点
	{
		vector<T_ANGLE_PULSE> vtTrackPulse(vvtRobotPulse.size());
		vtTrackPulse[0] = vvtRobotPulse[0][nTrackNo];
		vvtTrackPulse.push_back(vtTrackPulse);
		vtTotalPulse.push_back(vvtRobotPulse[0][nTrackNo]);
		vvtMaxAngleErr[nTrackNo].resize(vvtRobotPulse.size());
	}

	int nFoundIdx = 0;
	for (int nFoundIdx = 1; nFoundIdx < vvtRobotPulse.size(); nFoundIdx++) // 查找每组轨迹 第二个及以后的点 N
	{
		//当前坐标点待选的脉冲集合
		vector<T_ANGLE_PULSE>& vtTempPulse = vvtRobotPulse[nFoundIdx];

		//遍历所有可选连续脉冲轨迹
		for (int nTrackNo = 0; nTrackNo < vvtTrackPulse.size(); )
		{
			const T_ANGLE_PULSE& tPulse = vvtTrackPulse[nTrackNo][nFoundIdx - 1];//上一个直角坐标的对应脉冲坐标
			T_ANGLE_PULSE& tCurPulse = vvtTrackPulse[nTrackNo][nFoundIdx];//当前直角坐标的对应脉冲坐标

			//找出所有符合要求的当前脉冲坐标
			int nMatchNum = 0;//当前符合要求的脉冲坐标数目
			for (int nTempPulseIdx = 0; nTempPulseIdx < vtTempPulse.size(); nTempPulseIdx++)
			{
				//当前直角坐标的对应脉冲坐标
				const T_ANGLE_PULSE& tNewPulse = vtTempPulse[nTempPulseIdx];

				//当前脉冲坐标与上一个脉冲坐标的差值
				T_ANGLE_PULSE tDelta;
				tDelta.nSPulse = labs(tPulse.nSPulse - tNewPulse.nSPulse);
				tDelta.nLPulse = labs(tPulse.nLPulse - tNewPulse.nLPulse);
				tDelta.nUPulse = labs(tPulse.nUPulse - tNewPulse.nUPulse);
				tDelta.nRPulse = labs(tPulse.nRPulse - tNewPulse.nRPulse);
				tDelta.nBPulse = labs(tPulse.nBPulse - tNewPulse.nBPulse);
				tDelta.nTPulse = labs(tPulse.nTPulse - tNewPulse.nTPulse);

				//如果符合阈值限制
				if ((tDelta.nSPulse < tAngleThreshold.nSPulse) &&
					(tDelta.nLPulse < tAngleThreshold.nLPulse) &&
					(tDelta.nUPulse < tAngleThreshold.nUPulse) &&
					(tDelta.nRPulse < tAngleThreshold.nRPulse) &&
					(tDelta.nBPulse < tAngleThreshold.nBPulse) &&
					(tDelta.nTPulse < tAngleThreshold.nTPulse) &&
					(tNewPulse.nLPulse > nMinLPulse) &&
					(tNewPulse.nLPulse < nMaxLPulse))
				{
					//处理多个结果时选取脉冲差值最小的一个
					if (nMatchNum >= 1)
					{
						long lDiffe = tDelta.nSPulse + tDelta.nLPulse +
							tDelta.nUPulse + tDelta.nRPulse +
							tDelta.nBPulse + tDelta.nTPulse;
						long lDiffe2 = labs(tPulse.nSPulse - tCurPulse.nSPulse) +
							labs(tPulse.nLPulse - tCurPulse.nLPulse) +
							labs(tPulse.nUPulse - tCurPulse.nUPulse) +
							labs(tPulse.nRPulse - tCurPulse.nRPulse) +
							labs(tPulse.nBPulse - tCurPulse.nBPulse) +
							labs(tPulse.nTPulse - tCurPulse.nTPulse);
						if (lDiffe2 < lDiffe)
						{
							continue;
						}
					}
					tCurPulse = tNewPulse;
					nMatchNum++;
				}
			}

			//没有符合要求的脉冲坐标
			if (nMatchNum < 1)
			{
				vvtTrackPulse.erase(vvtTrackPulse.begin() + nTrackNo);
				vtTotalPulse.erase(vtTotalPulse.begin() + nTrackNo);
				continue; // 删除轨迹后 索引nTrackNo不变
			}

			//累计脉冲
			vtTotalPulse[nTrackNo].nSPulse += tCurPulse.nSPulse;
			vtTotalPulse[nTrackNo].nLPulse += tCurPulse.nLPulse;
			vtTotalPulse[nTrackNo].nUPulse += tCurPulse.nUPulse;
			vtTotalPulse[nTrackNo].nRPulse += tCurPulse.nRPulse;
			vtTotalPulse[nTrackNo].nBPulse += tCurPulse.nBPulse;
			vtTotalPulse[nTrackNo].nTPulse += tCurPulse.nTPulse;
			nTrackNo++;
		}
	}
	if (vvtTrackPulse.size() < 1)
	{
		XiMessageBox("连续运动轨迹计算失败!");
		return false;
	}

	// 每个连续运动轨迹各轴平均脉冲
	vector<T_ANGLE_PULSE> vtAveragePulse(vtTotalPulse.size());
	for (int nTrackNo = 0; nTrackNo < vvtTrackPulse.size(); nTrackNo++)
	{
		long lPtnNum = vvtTrackPulse[nTrackNo].size();
		vtAveragePulse[nTrackNo].nSPulse = vtTotalPulse[nTrackNo].nSPulse / lPtnNum;
		vtAveragePulse[nTrackNo].nLPulse = vtTotalPulse[nTrackNo].nLPulse / lPtnNum;
		vtAveragePulse[nTrackNo].nUPulse = vtTotalPulse[nTrackNo].nUPulse / lPtnNum;
		vtAveragePulse[nTrackNo].nRPulse = vtTotalPulse[nTrackNo].nRPulse / lPtnNum;
		vtAveragePulse[nTrackNo].nBPulse = vtTotalPulse[nTrackNo].nBPulse / lPtnNum;
		vtAveragePulse[nTrackNo].nTPulse = vtTotalPulse[nTrackNo].nTPulse / lPtnNum;
	}

	int nChooseIdx = 0;
	//// 取 S R T 脉冲和最小轨迹
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
	//// 取 按R轴 0 +180 -180 顺序选择
	nChooseIdx = -1;
	double dCurAngleR = 999.0;
	double dCurAngleT = 999.0;
	for (int i = 0; i < vtAveragePulse.size(); i++) // 差找R最接近0的轨迹索引
	{
		double dAngleR = vtAveragePulse[i].nRPulse * tAxleUnit.dRPulse;
		double dAngleT = vtAveragePulse[i].nTPulse * tAxleUnit.dTPulse;
		// R轴不翻转(小于90) 且 T轴最小
		if ((fabs(dAngleR) < 90.0)/* && (fabs(dAngleR) < fabs(dCurAngleR) */ && fabs(dAngleT) < fabs(dCurAngleT))
		{
			nChooseIdx = i;
			dCurAngleR = dAngleR;
			dCurAngleT = dAngleT;
		}
	}
	if (nChooseIdx < 0 && -1 == m_pRobot->m_nRobotInstallDir) // 查找R轴最大轨迹索引 (倒装专用 正装R轴不翻转)
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
		XiMessageBox("连续运动轨迹选择失败!");
		return false;
	}

	//生成最终连续脉冲轨迹
	//vector<T_ANGLE_PULSE> vtRobotPulse(0);
	//vtRobotPulse.reserve(vvtTrackPulse[nChooseIdx].size());
	//for (int i = 0; i < vvtTrackPulse[nChooseIdx].size(); i++)
	//{
	//	vtRobotPulse.push_back(vvtTrackPulse[nChooseIdx][i]);
	//}
	return true;
}

bool RTT::AdjustTrack::PlaneContourInfo2T_DXF_ENTITY_PARA(const PlaneContourInfo& info, int nContourNo, T_DXF_ENTITY_PARA& tEntities)
{
	gen::ThreeDotContour threedot;
	threedot.Completion(
		spf::SimplePoint(info.staPoint.x, info.staPoint.y, info.staPoint.z),
		spf::SimplePoint(info.midPoint.x, info.midPoint.y, info.midPoint.z),
		spf::SimplePoint(info.endPoint.x, info.endPoint.y, info.endPoint.z)
	);

	//厚度，对于外轮廓表示钢板厚度，对于坡口线表示坡口深度，对于焊接，设为0.1
	tEntities.dThick = 0.1;

	//线型比例，用于表示坡口宽度，对于焊接，设为0.1
	tEntities.dLineRatio = 0.1;

	//颜色，切割工艺顺逆时针
	tEntities.nColorNum = 0;

	//实体对应的组值
	tEntities.vtGroupNum.clear();
	tEntities.vtGroupNum.push_back(nContourNo);

	//图层，对于焊接，设为外轮廓
	tEntities.eLayer = E_DXF_LAYER_OUTER;

	tEntities.dStartX = info.staPoint.x;
	tEntities.dStartY = info.staPoint.y;
	tEntities.dStartZ = info.staPoint.z;

	tEntities.dEndX = info.endPoint.x;
	tEntities.dEndY = info.endPoint.y;
	tEntities.dEndZ = info.endPoint.z;

	tEntities.dCenterX = threedot.Origin.X;
	tEntities.dCenterY = threedot.Origin.Y;
	tEntities.dCenterZ = 0.0;

	tEntities.dStartAngle = threedot.dStartAngle;
	tEntities.dEndAngle = threedot.dEndAngle;
	tEntities.nContourEntityNo = nContourNo;

	if (threedot.IsLine1())
	{
		tEntities.eEntityType = E_DXF_ENTITY_TYPE_LINE;
	}
	else
	{
		if (threedot.ForwardReverse)
		{
			tEntities.eEntityType = E_DXF_ENTITY_TYPE_ARC_CLOCKWISE;
		}
		else
		{
			tEntities.eEntityType = E_DXF_ENTITY_TYPE_ARC_ANTICLOCKWISE;
		}
	}

	return true;
}

T_XIGEO_PLANE_PARA RTT::AdjustTrack::calcContourPlanePara()
{
	T_XIGEO_PLANE_PARA tContourPlanePara{};

	//小部件特别指定方向
	tContourPlanePara.tPlateVerDir.dDirX = 0.0;
	tContourPlanePara.tPlateVerDir.dDirY = 0.0;
	tContourPlanePara.tPlateVerDir.dDirZ = 1.0;

	//取轮廓中间点
	tContourPlanePara.tPoint.x = m_vtPartOutPoints[m_vtPartOutPoints.size() / 2].x;
	tContourPlanePara.tPoint.y = m_vtPartOutPoints[m_vtPartOutPoints.size() / 2].y;
	tContourPlanePara.tPoint.z = m_vtPartOutPoints[m_vtPartOutPoints.size() / 2].z;
	return tContourPlanePara;

	//统计每个实体都有多少个点
	std::map<int, int> mnPntCount;
	for (size_t i = 0; i < m_vnPartOutEntityNo.size(); i++)
	{
		auto iter = mnPntCount.find(m_vnPartOutEntityNo[i]);
		if (iter == mnPntCount.end())
		{
			mnPntCount[m_vnPartOutEntityNo[i]] = 1;
		}
		else
		{
			iter->second++;
		}
	}

	//从轮廓上提取若干点
	std::vector <T_POINT_3D> vtTeachPoints;
	for (size_t i = 0; i < m_vtContourEntities.size(); i++)
	{
		//找出该实体上的点数
		auto iter = mnPntCount.find(i);
		if (iter == mnPntCount.end())
		{
			continue;
		}
		int nPntCount = iter->second;

		//如果是直线
		if (m_vtContourEntities[i].eEntityType == E_DXF_ENTITY_TYPE_LINE)
		{
			//取中间点
			if (nPntCount < 3)
			{
				vtTeachPoints.push_back(m_vtPartOutPoints[m_vtPartOutPoints.size() / 2]);
			}
			//取等间隔两点
			else if (nPntCount < 300)
			{
				vtTeachPoints.push_back(m_vtPartOutPoints[m_vtPartOutPoints.size() / 3]);
				vtTeachPoints.push_back(m_vtPartOutPoints[m_vtPartOutPoints.size() * 2 / 3]);
			}
			//每200取一个点，不足部分首尾均分
			else
			{
				//计算首尾剩余点数
				int nRemainingQuantity = nPntCount % 200;
				nRemainingQuantity = nRemainingQuantity < 60 ? nRemainingQuantity + 200 : nRemainingQuantity;

				//等间隔取点
				for (size_t j = nRemainingQuantity / 2; j < nPntCount; j += 200)
				{
					vtTeachPoints.push_back(m_vtPartOutPoints[j]);
				}
			}
		}
		//其他情况视为圆弧
		else
		{
			//取中间点
			if (nPntCount < 5)
			{
				vtTeachPoints.push_back(m_vtPartOutPoints[m_vtPartOutPoints.size() / 2]);
			}
			//取等间隔三点
			else if (nPntCount < 400)
			{
				vtTeachPoints.push_back(m_vtPartOutPoints[m_vtPartOutPoints.size() / 4]);
				vtTeachPoints.push_back(m_vtPartOutPoints[m_vtPartOutPoints.size() * 2 / 4]);
				vtTeachPoints.push_back(m_vtPartOutPoints[m_vtPartOutPoints.size() * 3 / 4]);
			}
			//每150取一个点，不足部分首尾均分
			else
			{
				//计算首尾剩余点数
				int nRemainingQuantity = nPntCount % 150;
				nRemainingQuantity = nRemainingQuantity < 40 ? nRemainingQuantity + 150 : nRemainingQuantity;

				//等间隔取点
				for (size_t j = nRemainingQuantity / 2; j < nPntCount; j += 150)
				{
					vtTeachPoints.push_back(m_vtPartOutPoints[j]);
				}
			}
		}
	}

	//生成平面参数
	T_PLANE_PARAM tPlaneParam = m_cAnalyzeObj.CalcPlaneParamRansac(vtTeachPoints, 1.0);
	tContourPlanePara.tPlateVerDir.dDirX = tPlaneParam.a;
	tContourPlanePara.tPlateVerDir.dDirY = tPlaneParam.b;
	tContourPlanePara.tPlateVerDir.dDirZ = tPlaneParam.c;

	return tContourPlanePara;
}


bool RTT::AdjustTrack::GetTrackingInciseData(E_DOTRACKING_CONTOUR_TYPE eDoTrackingContourType, E_DOTRACKING_BEVEL_TYPE eDoTrackingBevelType,
	VT_GRAPH vtContourEntities, std::vector<T_POINT_3D> vtIncisePoints, std::vector<int> vnIncisePointEntityNo, 
	std::vector<T_POINT_3D> vtBevelPoints, std::vector<int> vnBevelPointEntityNo, 
	std::vector<vector<T_POINT_3D>>& vvtInciseEntityWithBevelPoints, std::vector<int>& vnInciseEntityWithBevelNo, std::vector<vector<T_LINE_DIR>>& vvtInciseEntityPointAdjustDir, std::vector<double>& vdInciseToBevelDis, std::vector<vector<T_POINT_3D>>& vvtBevelEntityPoints, std::vector<int>& vnBevelEntityNo)
{
	vvtInciseEntityWithBevelPoints.clear();
	vnInciseEntityWithBevelNo.clear();
	vvtInciseEntityPointAdjustDir.clear();
	vvtBevelEntityPoints.clear();
	vdInciseToBevelDis.clear();
	vnBevelEntityNo.clear();

	std::vector<T_POINT_3D> vtInciseEntityPoints;
	vtInciseEntityPoints.clear();
	std::vector<vector<T_POINT_3D>> vvtAllInciseEntityPoints;
	vvtAllInciseEntityPoints.clear();
	std::vector<int> vnInciseEntityNo;
	vnInciseEntityNo.clear();

	for (size_t nPointNo = 0; nPointNo < vtIncisePoints.size(); nPointNo++)
	{
		vtInciseEntityPoints.push_back(vtIncisePoints[nPointNo]);

		if (nPointNo < vtIncisePoints.size() - 1)
		{
			if (vnIncisePointEntityNo[nPointNo] != vnIncisePointEntityNo[nPointNo + 1])
			{
				vnInciseEntityNo.push_back(vnIncisePointEntityNo[nPointNo]);
				vvtAllInciseEntityPoints.push_back(vtInciseEntityPoints);
				vtInciseEntityPoints.clear();
			}
		}
		else
		{
			vnInciseEntityNo.push_back(vnIncisePointEntityNo[nPointNo]);
			vvtAllInciseEntityPoints.push_back(vtInciseEntityPoints);
		}
	}

	std::vector<T_POINT_3D> vtBevelEntityPoints;
	vtBevelEntityPoints.clear();

	for (size_t nPointNo = 0; nPointNo < vtBevelPoints.size(); nPointNo++)
	{
		vtBevelEntityPoints.push_back(vtBevelPoints[nPointNo]);

		if (nPointNo < vtBevelPoints.size() - 1)
		{
			if (vnBevelPointEntityNo[nPointNo] != vnBevelPointEntityNo[nPointNo + 1])
			{
				vnBevelEntityNo.push_back(vnBevelPointEntityNo[nPointNo]);
				vvtBevelEntityPoints.push_back(vtBevelEntityPoints);
				vtBevelEntityPoints.clear();
			}
		}
		else
		{
			vnBevelEntityNo.push_back(vnBevelPointEntityNo[nPointNo]);
			vvtBevelEntityPoints.push_back(vtBevelEntityPoints);
		}
	}

	std::vector<T_LINE_DIR> vtInciseEntityPointAdjustDir;
	vtInciseEntityPointAdjustDir.clear();
	vtInciseEntityPoints.clear();

	double dInciseToBevelDis = 0.0;

	for (int nBevelEntityNo = 0; nBevelEntityNo < vnBevelEntityNo.size(); nBevelEntityNo++)
	{
		vtInciseEntityPoints.clear();
		vtInciseEntityPointAdjustDir.clear();
		for (int nInciseEntityNo = 0; nInciseEntityNo < vnInciseEntityNo.size(); nInciseEntityNo++)
		{
			if (vnBevelEntityNo[nBevelEntityNo] == vnInciseEntityNo[nInciseEntityNo])
			{
				vnInciseEntityWithBevelNo.push_back(vnInciseEntityNo[nInciseEntityNo]);
				vtInciseEntityPoints.insert(vtInciseEntityPoints.end(), vvtAllInciseEntityPoints[nInciseEntityNo].begin(), vvtAllInciseEntityPoints[nInciseEntityNo].end());
				CalAdjustDirAndDisForTracking(eDoTrackingContourType, eDoTrackingBevelType, vtContourEntities[vnInciseEntityNo[nInciseEntityNo]], vvtAllInciseEntityPoints[nInciseEntityNo], vvtBevelEntityPoints[nBevelEntityNo], vtInciseEntityPointAdjustDir, dInciseToBevelDis);
				vdInciseToBevelDis.push_back(dInciseToBevelDis);
				vvtInciseEntityWithBevelPoints.push_back(vtInciseEntityPoints);
				vvtInciseEntityPointAdjustDir.push_back(vtInciseEntityPointAdjustDir);
				if (vtInciseEntityPoints.size() != vtInciseEntityPointAdjustDir.size())
				{
					XiMessageBox("轨迹调整方向计算错误");
					return false;
				}
			}
		}
	}
	return true;
}

void RTT::AdjustTrack::CalAdjustDirAndDisForTracking(E_DOTRACKING_CONTOUR_TYPE eDoTrackingContourType, E_DOTRACKING_BEVEL_TYPE eDoTrackingBevelType, T_DXF_ENTITY_PARA tEntity, std::vector<T_POINT_3D> vtIncisePoints, std::vector<T_POINT_3D> vtBevelPoints, std::vector<T_LINE_DIR>& vtAdjustDir, double& dAdjustDis)
{
	vtAdjustDir.clear();

	double dTransMatrix[4][4];
	double dMinDis = 999999.0;
	int nMinNo = 0;
	double dDis = 0.0;
	//T_LINE_DIR tLineDir;
	T_LINE_DIR tEntityLineDir;
	T_LINE_DIR tAdjustLineDir{0};
	T_POINT_3D tAdjustDir;

	if (eDoTrackingContourType == E_OUTTERCONTOUR_TRACKING)
	{
		for (int nPointNo = 0; nPointNo < vtIncisePoints.size(); nPointNo++)
		{
			if (tEntity.eEntityType == E_DXF_ENTITY_TYPE_LINE)
			{
				tEntityLineDir = CalLineDir(tEntity.dStartX, tEntity.dStartY, tEntity.dEndX, tEntity.dEndY);
				m_cXiRobotAbsCoorsTrans->GetRotateAnyDirMatrix(-90.0, tEntityLineDir.dDirX, tEntityLineDir.dDirY, 0.0, dTransMatrix);

				tAdjustDir.x = 0.0;
				tAdjustDir.y = 0.0;
				tAdjustDir.z = -1.0;
				CoorTransform(tAdjustDir, dTransMatrix);

				tAdjustLineDir.dDirX = tAdjustDir.x;
				tAdjustLineDir.dDirY = tAdjustDir.y;

				vtAdjustDir.push_back(tAdjustLineDir);
			}
			else if (tEntity.eEntityType == E_DXF_ENTITY_TYPE_ARC_CLOCKWISE)
			{
				tAdjustLineDir = CalLineDir(vtIncisePoints[nPointNo].x, vtIncisePoints[nPointNo].y, tEntity.dCenterX, tEntity.dCenterY);
				vtAdjustDir.push_back(tAdjustLineDir);
			}
			else if (tEntity.eEntityType == E_DXF_ENTITY_TYPE_ARC_ANTICLOCKWISE)
			{
				tAdjustLineDir = CalLineDir(tEntity.dCenterX, tEntity.dCenterY, vtIncisePoints[nPointNo].x, vtIncisePoints[nPointNo].y);
				vtAdjustDir.push_back(tAdjustLineDir);
			}
		}

		if (eDoTrackingBevelType == E_BEVEL_UP_TRACKING)
		{
			T_POINT_2D_DOUBLE tInputPoint{};
			T_POINT_2D_DOUBLE tStartPoint{};
			T_POINT_2D_DOUBLE tEndPoint{};
			tInputPoint.x = vtBevelPoints[int(vtBevelPoints.size() / 2.0)].x;
			tInputPoint.y = vtBevelPoints[int(vtBevelPoints.size() / 2.0)].y;

			if (tEntity.eEntityType == E_DXF_ENTITY_TYPE_LINE)
			{
				tStartPoint.x = tEntity.dStartX;
				tStartPoint.y = tEntity.dStartY;
				tEndPoint.x = tEntity.dEndX;
				tEndPoint.y = tEntity.dEndY;
				dAdjustDis = m_cAnalyzeObj.CalDisPointToLine(tInputPoint, tStartPoint, tEndPoint);
			}
			else
			{
				dAdjustDis = fabs(m_cAnalyzeObj.CalDisPointToPoint(tEntity.dStartX, tEntity.dStartY, tEntity.dCenterX, tEntity.dCenterY) -
					m_cAnalyzeObj.CalDisPointToPoint(tInputPoint.x, tInputPoint.y, tEntity.dCenterX, tEntity.dCenterY));
			}
		}
		else if (eDoTrackingBevelType == E_BEVEL_DOWN_TRACKING)
		{
			dAdjustDis = 0.0;
		}
		else
		{
			XiMessageBox("跟踪坡口类型有误!");
		}
	}
	else if (eDoTrackingContourType == E_INNERCONTOUR_TRACKING)
	{
		for (int nPointNo = 0; nPointNo < vtIncisePoints.size(); nPointNo++)
		{
			if (tEntity.eEntityType == E_DXF_ENTITY_TYPE_LINE)
			{
				tEntityLineDir = CalLineDir(tEntity.dStartX, tEntity.dStartY, tEntity.dEndX, tEntity.dEndY);
				m_cXiRobotAbsCoorsTrans->GetRotateAnyDirMatrix(90.0, tEntityLineDir.dDirX, tEntityLineDir.dDirY, 0.0, dTransMatrix);

				tAdjustDir.x = 0.0;
				tAdjustDir.y = 0.0;
				tAdjustDir.z = -1.0;
				CoorTransform(tAdjustDir, dTransMatrix);

				tAdjustLineDir.dDirX = tAdjustDir.x;
				tAdjustLineDir.dDirY = tAdjustDir.y;

				vtAdjustDir.push_back(tAdjustLineDir);
			}
			else if (tEntity.eEntityType == E_DXF_ENTITY_TYPE_ARC_ANTICLOCKWISE)
			{
				tAdjustLineDir = CalLineDir(vtIncisePoints[nPointNo].x, vtIncisePoints[nPointNo].y, tEntity.dCenterX, tEntity.dCenterY);
				vtAdjustDir.push_back(tAdjustLineDir);
			}
			else if (tEntity.eEntityType == E_DXF_ENTITY_TYPE_ARC_CLOCKWISE)
			{
				tAdjustLineDir = CalLineDir(tEntity.dCenterX, tEntity.dCenterY, vtIncisePoints[nPointNo].x, vtIncisePoints[nPointNo].y);
				vtAdjustDir.push_back(tAdjustLineDir);
			}
		}

		if (eDoTrackingBevelType == E_BEVEL_UP_TRACKING)
		{
			T_POINT_2D_DOUBLE tInputPoint{};
			T_POINT_2D_DOUBLE tStartPoint{};
			T_POINT_2D_DOUBLE tEndPoint{};
			tInputPoint.x = vtBevelPoints[int(vtBevelPoints.size() / 2.0)].x;
			tInputPoint.y = vtBevelPoints[int(vtBevelPoints.size() / 2.0)].y;

			if (tEntity.eEntityType == E_DXF_ENTITY_TYPE_LINE)
			{
				tStartPoint.x = tEntity.dStartX;
				tStartPoint.y = tEntity.dStartY;
				tEndPoint.x = tEntity.dEndX;
				tEndPoint.y = tEntity.dEndY;
				dAdjustDis = m_cAnalyzeObj.CalDisPointToLine(tInputPoint, tStartPoint, tEndPoint);
			}
			else
			{
				dAdjustDis = fabs(m_cAnalyzeObj.CalDisPointToPoint(tEntity.dStartX, tEntity.dStartY, tEntity.dCenterX, tEntity.dCenterY) -
					m_cAnalyzeObj.CalDisPointToPoint(tInputPoint.x, tInputPoint.y, tEntity.dCenterX, tEntity.dCenterY));
			}
		}
		else if (eDoTrackingBevelType == E_BEVEL_DOWN_TRACKING)
		{
			dAdjustDis = 0.0;
		}
		else
		{
			XiMessageBox("跟踪坡口类型有误!");
		}
	}
	else
	{
		XiMessageBox("跟踪轮廓类型有误!");
	}
}

void RTT::AdjustTrack::CalcIfDoTracking(T_XIGEO_PLANE_PARA tContourPlanePara, VT_GRAPH vtContourEntities, std::vector<int> vnBevelEntityNo, std::vector<T_DOTRACKING_PARA>& vtDoTrackPara)
{
	auto tCameraPara = m_pUnit->GetCameraParam(m_pUnit->m_nTrackCameraNo);
	int nImageWidth = tCameraPara.tDHCameraDriverPara.nRoiWidth;
	int nImageHeight = tCameraPara.tDHCameraDriverPara.nRoiHeight;
	int nMinROI_X = 500;
	int nMinROI_Y = 100;
	int nMaxROI_X = nImageWidth - 100;
	int nMaxROI_Y = nImageHeight - 100;

	T_ROBOT_COORS_IN_WORLD tRobotCoorsInWorld;
	//	double dBX, dBY, dBZ;
	//	double dThickness;
	//	int nPointNo;
	vtDoTrackPara.clear();

	E_CAM_ID eCamId = E_RIGHT_ROBOT_RIGHT_CAM;
	std::vector<BOOL> vbIfDoTracking;
	std::vector<T_POINT_2D> vtTrackingPointInImage;
	std::vector<T_POINT_3D> vtBevelPoints;

	//将轨迹转到世界坐标系下
	std::vector<T_ROBOT_COORS_IN_WORLD> vtCutRobotCoors = m_pWeldTrack->toWorldD<T_ROBOT_COORS_IN_WORLD>();
	vtBevelPoints = m_vtBevelPoints;

	//GetCameraIfDoTrackingNew(tContourPlanePara, vtContourEntities, vtBevelPoints, vnBevelEntityNo, vtCutRobotCoors, eCamId, vtDoTrackPara, vtTrackingPointInImage);
	GetCameraIfDoTrackingNew(tContourPlanePara, vtContourEntities, vtBevelPoints, vnBevelEntityNo, vtCutRobotCoors, eCamId, vtDoTrackPara, vtTrackingPointInImage, m_vtHorTrackingPointInImage, m_vtVerTrackingPointInImage);

	//末尾少看一点
	//如果是一个完整的圆，会有问题
	bool isSeeZeroNo = true;
	int nDeleteNo = 0;
	if (m_pWeldTrack->isClosed())
	{
		for (int n = vtDoTrackPara.size() - 1; n > 0; n--)
		{
			//如果末尾看到0号实体，则认为看不见
			if (isSeeZeroNo
				&& vtDoTrackPara[n].nEntityNo == 0
				&& vtContourEntities.size() > 1)
			{
				vtDoTrackPara[n].bIfDoTracking = false;
			}
			else
			{
				isSeeZeroNo = false;
			}
			//如果末尾看不到0号实体，少看Nmm
			if (!isSeeZeroNo
				&& nDeleteNo < 18)
			{
				vtDoTrackPara[n].bIfDoTracking = false;
				nDeleteNo++;
			}
		}
	}

	CString sFileName;
	sFileName.Format(getDataFileRootFolder() + "TrackIfDoTracking.txt");
	FILE* pfDoTracking = fopen(sFileName, "w");
	int nCutRobotPtnSum = vtCutRobotCoors.size();
	int nIfDoTrackSum = vbIfDoTracking.size();
	int nTrackPtnInImgSum = vtTrackingPointInImage.size();
	bool bIfDoTracking = true;
	int nImgNo = 0;
	bool isNotZeroNo = false;
	for (int i = 0; i < vtDoTrackPara.size(); i++)
	{
		//闭合曲线尾部看到0号实体不可以跟踪
		//if (vtDoTrackPara[i].nEntityNo > 0)
		//	isNotZeroNo = true;
		//if (vtDoTrackPara[i].nEntityNo == 0 && isNotZeroNo)
		//{
		//	vtDoTrackPara[i].bIfDoTracking = false;
		//}

		//从无法跟踪到可以跟踪时，前N个轨迹仍然认为不可跟踪，以避免斜率错误
		if (!vtDoTrackPara[i].bIfDoTracking)
		{
			bIfDoTracking = false;
		}
		else if (!bIfDoTracking)
		{
			if (nImgNo < 0)
			{
				vtDoTrackPara[i].bIfDoTracking = false;
				nImgNo++;
			}
			else
			{
				bIfDoTracking = true;
				nImgNo = 0;
			}
		}

		//非闭合曲线的前50mm不跟踪，以避免斜率错误
		if (!m_pWeldTrack->isClosed() && i < 50)
		{
			vtDoTrackPara[i].bIfDoTracking = false;
		}

		//如果计算的结果超出了视野，认为不可跟踪
		if (vtTrackingPointInImage[i].x < nMinROI_X
			|| m_vtHorTrackingPointInImage[i].x < nMinROI_X
			|| m_vtVerTrackingPointInImage[i].x < nMinROI_X
			|| vtTrackingPointInImage[i].y < nMinROI_Y
			|| m_vtHorTrackingPointInImage[i].y < nMinROI_Y
			|| m_vtVerTrackingPointInImage[i].y < nMinROI_Y
			|| vtTrackingPointInImage[i].x > nMaxROI_X
			|| m_vtHorTrackingPointInImage[i].x > nMaxROI_X
			|| m_vtVerTrackingPointInImage[i].x > nMaxROI_X
			|| vtTrackingPointInImage[i].y > nMaxROI_Y
			|| m_vtHorTrackingPointInImage[i].y > nMaxROI_Y
			|| m_vtVerTrackingPointInImage[i].y > nMaxROI_Y
			)
		{
			vtDoTrackPara[i].bIfDoTracking = false;
		}

		fprintf(pfDoTracking,
			"%4d %11.3lf %11.3lf %11.3lf %11.3lf "
			"%11.3lf %11.3lf %4d %4d %4d "
			"%6.0lf %6.0lf %6.0lf %6.0lf %6.0lf "
			"%6.0lf \n",
			i, vtCutRobotCoors[i].dX, vtCutRobotCoors[i].dY, vtCutRobotCoors[i].dZ,
			vtDoTrackPara[i].tIntersectionPoint.x, vtDoTrackPara[i].tIntersectionPoint.y, vtDoTrackPara[i].tIntersectionPoint.z,
			vtDoTrackPara[i].bIfDoTracking, vtDoTrackPara[i].nEntityNo, vtDoTrackPara[i].nEntityType,
			vtTrackingPointInImage[i].x, vtTrackingPointInImage[i].y,
			m_vtHorTrackingPointInImage[i].x, m_vtHorTrackingPointInImage[i].y,
			m_vtVerTrackingPointInImage[i].x, m_vtVerTrackingPointInImage[i].y
		);
		fflush(pfDoTracking);
	}
	fclose(pfDoTracking);

	// 查找最后一个能跟踪的数据的索引
	int n = 0;
	for (n = vtDoTrackPara.size() - 1; n > 0; n--)
	{
		if (true == vtDoTrackPara[n].bIfDoTracking)
		{
			break;
		}
	}
	m_nLastPointOfTracking = n;

	// 动态修改终点不调增阈值，确保在此范围内有跟踪示教点
	int nUnTrackingPointNum = m_vtDoTrackPara.size() - m_nLastPointOfTracking;
	if (nUnTrackingPointNum > 100) // 100为手眼关系距离中的份数 1毫米一份
	{
		m_nEndAdjustLimit = m_nEndAdjustLimit + (nUnTrackingPointNum - 100);
	}

	m_vtTrackingPointInImage.swap(vtTrackingPointInImage);
	writeLog("实时跟踪：最后一个可跟踪点索引：%d, 轨迹点数:%d，终点不调整点数：%d",
		m_nLastPointOfTracking, vtDoTrackPara.size(), m_nEndAdjustLimit);
}


//void RTT::AdjustTrack::GetCameraIfDoTrackingNew(T_XIGEO_PLANE_PARA tContourPlanePara, std::vector<T_DXF_ENTITY_PARA> vtContourEntities, std::vector<T_POINT_3D> vtBevelPoints, std::vector<int> vnEntityNo, std::vector<T_ROBOT_COORS_IN_WORLD> vtCutRobotCoors, E_CAM_ID eCamId, std::vector<T_DOTRACKING_PARA>& vtIfDoTracking, std::vector<T_POINT_2D>& vtTrackingPointInImage)
//{
//	T_XIGEO_PLANE_PARA tPlanePara;
//	T_POINT_3D tIntersectionPoint;
//	T_POINT_3D tBasePoint;
//	double dDis = 0.0;
//	double dAngle = 0.0;
//	T_XIGEO_LINE_PARA tIntersectionLine;
//	T_POINT_3D tLineStartPoint;
//	T_POINT_3D tLineEndPoint;
//	double dDisLimit = m_dDisLimitForCalDoTracking;			// 当前激光点 和 图像中心激光点 距离差 mm
//	double dAngleLimit = m_dAngleLimitForCalDoTracking;// 45.0;
//	T_DOTRACKING_PARA tDoTrackingPara;
//	BOOL nIfDo = FALSE;
//	int nStartNo = 0;
//
//	for (int nPointNo = 0; nPointNo < vtCutRobotCoors.size(); nPointNo++)
//	{
//		tPlanePara = CalcLaserPlate(vtCutRobotCoors[nPointNo], eCamId);
//		tBasePoint = P2P<T_POINT_3D>(tPlanePara.tPoint);
//
//		nIfDo = FALSE;
//		tDoTrackingPara.bIfDoTracking = FALSE;
//		tDoTrackingPara.nEntityNo = -1;
//		tDoTrackingPara.nEntityType = -1;
//		tDoTrackingPara.tIntersectionPoint.x = 0.0;
//		tDoTrackingPara.tIntersectionPoint.y = 0.0;
//		tDoTrackingPara.tIntersectionPoint.z = 0.0;
//
//
//		for (int nEntityNo = 0; nEntityNo < vtContourEntities.size(); nEntityNo++)
//		{
//			if (GetEntityAndPlaneInterseciton(vtContourEntities[nEntityNo], tContourPlanePara, tBasePoint, tPlanePara, tIntersectionPoint))
//			{
//				dDis = m_cAnalyzeObj.CalDisPointToPoint(tIntersectionPoint.x, tIntersectionPoint.y, tIntersectionPoint.z, tBasePoint.x, tBasePoint.y, tBasePoint.z);
//
//				GetTwoPlaneIntersectionLine(tContourPlanePara, tPlanePara, tIntersectionLine);
//				tLineStartPoint.x = tIntersectionLine.tPoint.x;
//				tLineStartPoint.y = tIntersectionLine.tPoint.y;
//				tLineStartPoint.z = tIntersectionLine.tPoint.z;
//
//				tLineEndPoint.x = tIntersectionLine.tPoint.x + 20.0 * tIntersectionLine.tLineDir.dDirX;
//				tLineEndPoint.y = tIntersectionLine.tPoint.y + 20.0 * tIntersectionLine.tLineDir.dDirY;
//				tLineEndPoint.z = tIntersectionLine.tPoint.z + 20.0 * tIntersectionLine.tLineDir.dDirZ;
//
//				dAngle = CalAngleEntityAndLine(tIntersectionPoint, vtContourEntities[nEntityNo], tLineStartPoint, tLineEndPoint);
//
//				if (dDis <= dDisLimit && fabs(90 - fabs(dAngle)) < dAngleLimit)
//				{
//					int nNearestNo = FindNearestPoint(tIntersectionPoint, vtBevelPoints);
//
//					if (vnEntityNo[nNearestNo] == -1)
//					{
//						tDoTrackingPara.bIfDoTracking = FALSE;
//						tDoTrackingPara.nEntityNo = vtContourEntities[nEntityNo].nContourEntityNo;
//						tDoTrackingPara.tIntersectionPoint = tIntersectionPoint;
//						if (vtContourEntities[nEntityNo].eEntityType == E_DXF_ENTITY_TYPE_LINE)
//						{
//							tDoTrackingPara.nEntityType = 1;
//						}
//						else if (vtContourEntities[nEntityNo].eEntityType == E_DXF_ENTITY_TYPE_ARC_ANTICLOCKWISE || vtContourEntities[nEntityNo].eEntityType == E_DXF_ENTITY_TYPE_ARC_CLOCKWISE)
//						{
//							tDoTrackingPara.nEntityType = 2;
//						}
//						else
//						{
//							tDoTrackingPara.nEntityType = -1;
//						}
//						break;
//					}
//					else
//					{
//						double dEntityLength = m_cAnalyzeObj.GetEntitiesLength(vtContourEntities[nEntityNo]);
//
//						if (dEntityLength < 60)
//						{
//							tDoTrackingPara.bIfDoTracking = FALSE;
//							tDoTrackingPara.nEntityNo = vtContourEntities[nEntityNo].nContourEntityNo;
//							tDoTrackingPara.tIntersectionPoint = tIntersectionPoint;
//							if (vtContourEntities[nEntityNo].eEntityType == E_DXF_ENTITY_TYPE_LINE)
//							{
//								tDoTrackingPara.nEntityType = 1;
//							}
//							else if (vtContourEntities[nEntityNo].eEntityType == E_DXF_ENTITY_TYPE_ARC_ANTICLOCKWISE || vtContourEntities[nEntityNo].eEntityType == E_DXF_ENTITY_TYPE_ARC_CLOCKWISE)
//							{
//								tDoTrackingPara.nEntityType = 2;
//							}
//							else
//							{
//								tDoTrackingPara.nEntityType = -1;
//							}
//							break;
//						}
//						else
//						{
//							nIfDo = TRUE;
//							tDoTrackingPara.bIfDoTracking = TRUE;
//							tDoTrackingPara.nEntityNo = vtContourEntities[nEntityNo].nContourEntityNo;
//							tDoTrackingPara.tIntersectionPoint = tIntersectionPoint;
//							if (vtContourEntities[nEntityNo].eEntityType == E_DXF_ENTITY_TYPE_LINE)
//							{
//								tDoTrackingPara.nEntityType = 1;
//							}
//							else if (vtContourEntities[nEntityNo].eEntityType == E_DXF_ENTITY_TYPE_ARC_ANTICLOCKWISE || vtContourEntities[nEntityNo].eEntityType == E_DXF_ENTITY_TYPE_ARC_CLOCKWISE)
//							{
//								tDoTrackingPara.nEntityType = 2;
//							}
//							else
//							{
//								tDoTrackingPara.nEntityType = -1;
//							}
//							break;
//						}
//					}
//				}
//			}
//		}
//		vtIfDoTracking.push_back(tDoTrackingPara);
//	}
//}

bool RTT::AdjustTrack::GetEntityAndPlaneInterseciton(T_DXF_ENTITY_PARA tEntity, T_XIGEO_PLANE_PARA tEntityPlanePara, T_POINT_3D tBasePoint, T_XIGEO_PLANE_PARA tPlanePara, T_POINT_3D& tIntersectionPoint)
{
	T_XIGEO_LINE_PARA tLinePara;
	T_XIGEO_ARC_PARA tArcPara;
	double dMin = 999999.0;
	double dDis = 0.0;
	double dMinDis = 999999.0;

	std::vector<T_POINT_3D> vtLineArcIntersectionPoint;
	vtLineArcIntersectionPoint.clear();

	if (tEntity.eEntityType == E_DXF_ENTITY_TYPE_LINE)
	{
		tLinePara = GetXiLinePara(tEntity.dStartX, tEntity.dStartY, tEntity.dStartZ, tEntity.dEndX, tEntity.dEndY, tEntity.dEndZ);
		if (LinePlateInterSection(tLinePara, tPlanePara, tIntersectionPoint))
		{
			if (m_cAnalyzeObj.JudgePointOnEntity(tIntersectionPoint.x, tIntersectionPoint.y, tEntity))
			{
				return true;
			}
			else
			{
				return false;
			}
		}
		else
		{
			return false;
		}
	}
	else if (tEntity.eEntityType == E_DXF_ENTITY_TYPE_ARC_CLOCKWISE || tEntity.eEntityType == E_DXF_ENTITY_TYPE_ARC_ANTICLOCKWISE)
	{
		tArcPara = GetXiArcPara(tEntity.dStartX, tEntity.dStartY, tEntity.dStartZ, tEntity.dEndX, tEntity.dEndY, tEntity.dEndZ,
			tEntity.dCenterX, tEntity.dCenterY, tEntity.dCenterZ, tEntityPlanePara.tPlateVerDir.dDirX, tEntityPlanePara.tPlateVerDir.dDirY,
			tEntityPlanePara.tPlateVerDir.dDirZ);

		if (ArcPlateInterSection(tArcPara, tPlanePara, vtLineArcIntersectionPoint))
		{
			for (int nPointNo = 0; nPointNo < vtLineArcIntersectionPoint.size(); nPointNo++)
			{
				dDis = m_cAnalyzeObj.CalDisPointToPoint(tBasePoint.x, tBasePoint.y, tBasePoint.z, vtLineArcIntersectionPoint[nPointNo].x, vtLineArcIntersectionPoint[nPointNo].y, vtLineArcIntersectionPoint[nPointNo].z);

				if (dDis < dMinDis)
				{
					dMinDis = dDis;
					tIntersectionPoint = vtLineArcIntersectionPoint[nPointNo];
				}
			}
			if (m_cAnalyzeObj.JudgePointOnEntity(tIntersectionPoint.x, tIntersectionPoint.y, tEntity))
			{
				return true;
			}
			else
			{
				return false;
			}
		}
		else
		{
			return false;
		}
	}
	return false;
}

T_XIGEO_LINE_PARA RTT::AdjustTrack::GetXiLinePara(double dStartX, double dStartY, double dStartZ, double dEndX, double dEndY, double dEndZ)
{
	T_LINE_DIR_3D tLineDir;
	T_XIGEO_LINE_DIR tXiGeoLineDir{};
	XI_POINT tXiPoint{};
	T_XIGEO_LINE_PARA tXiGeoLinePara{};

	tXiPoint.x = dStartX;
	tXiPoint.y = dStartY;
	tXiPoint.z = dStartZ;
	tLineDir = CalLineDir(dStartX, dStartY, dStartZ, dEndX, dEndY, dEndZ);

	tXiGeoLineDir.dDirX = tLineDir.dDirX;
	tXiGeoLineDir.dDirY = tLineDir.dDirY;
	tXiGeoLineDir.dDirZ = tLineDir.dDirZ;

	tXiGeoLinePara.tLineDir = tXiGeoLineDir;
	tXiGeoLinePara.tPoint = tXiPoint;

	return tXiGeoLinePara;
}

RTT::AdjustTrack::T_XIGEO_ARC_PARA RTT::AdjustTrack::GetXiArcPara(double dStartPointX, double dStartPointY, double dStartPointZ,
	double dEndPointX, double dEndPointY, double dEndPointZ,
	double dCenterX, double dCenterY, double dCenterZ,
	double dDirX, double dDirY, double dDirZ)
{
	//T_LINE_DIR_3D tLineDir;
	//T_XIGEO_LINE_DIR tXiGeoLineDir;
	//XI_POINT tXiPoint;
	T_XIGEO_ARC_PARA tXiArcPara{};

	tXiArcPara.tStartPoint.x = dStartPointX;
	tXiArcPara.tStartPoint.y = dStartPointY;
	tXiArcPara.tStartPoint.z = dStartPointZ;

	tXiArcPara.tEndPoint.x = dEndPointX;
	tXiArcPara.tEndPoint.y = dEndPointY;
	tXiArcPara.tEndPoint.z = dEndPointZ;

	tXiArcPara.tCenterPoint.x = dCenterX;
	tXiArcPara.tCenterPoint.y = dCenterY;
	tXiArcPara.tCenterPoint.z = dCenterZ;

	tXiArcPara.dRadius = m_cAnalyzeObj.CalDisPointToPoint(dStartPointX, dStartPointY, dStartPointZ, dCenterX, dCenterY, dCenterZ);

	tXiArcPara.tVerDir.dDirX = dDirX;
	tXiArcPara.tVerDir.dDirY = dDirY;
	tXiArcPara.tVerDir.dDirZ = dDirZ;

	return tXiArcPara;
}

BOOL RTT::AdjustTrack::LinePlateInterSection(T_XIGEO_LINE_PARA tLinePara, T_XIGEO_PLANE_PARA tPlanePara, T_POINT_3D& tIntersectionPoint)
{
	double dRoot = 0.0;
	double dJudgeCondition = 0.0;
	double dNumbernator = 0.0, dDenominator = 0.0;

	dJudgeCondition = tLinePara.tLineDir.dDirX * tPlanePara.tPlateVerDir.dDirX + tLinePara.tLineDir.dDirY * tPlanePara.tPlateVerDir.dDirY
		+ tLinePara.tLineDir.dDirZ * tPlanePara.tPlateVerDir.dDirZ;

	dNumbernator = -(tLinePara.tPoint.x * tPlanePara.tPlateVerDir.dDirX + tLinePara.tPoint.y * tPlanePara.tPlateVerDir.dDirY
		+ tLinePara.tPoint.z * tPlanePara.tPlateVerDir.dDirZ - tPlanePara.tPlateVerDir.dDirX * tPlanePara.tPoint.x
		- tPlanePara.tPlateVerDir.dDirY * tPlanePara.tPoint.y - tPlanePara.tPlateVerDir.dDirZ * tPlanePara.tPoint.z);
	dDenominator = (tLinePara.tLineDir.dDirX * tPlanePara.tPlateVerDir.dDirX + tLinePara.tLineDir.dDirY * tPlanePara.tPlateVerDir.dDirY
		+ tLinePara.tLineDir.dDirZ * tPlanePara.tPlateVerDir.dDirZ);

	dRoot = dNumbernator / dDenominator;

	if (fabs(dJudgeCondition) < 0.0001)
	{
		tIntersectionPoint.x = tLinePara.tPoint.x;
		tIntersectionPoint.y = tLinePara.tPoint.y;
		tIntersectionPoint.z = tLinePara.tPoint.z;
		return FALSE;
	}
	else
	{
		tIntersectionPoint.x = tLinePara.tPoint.x + tLinePara.tLineDir.dDirX * dRoot;
		tIntersectionPoint.y = tLinePara.tPoint.y + tLinePara.tLineDir.dDirY * dRoot;
		tIntersectionPoint.z = tLinePara.tPoint.z + tLinePara.tLineDir.dDirZ * dRoot;

		return TRUE;
	}
}

BOOL RTT::AdjustTrack::ArcPlateInterSection(T_XIGEO_ARC_PARA tArcPara, T_XIGEO_PLANE_PARA tPlanePara, std::vector<T_POINT_3D>& vtIntersectionPoint)
{
	vtIntersectionPoint.clear();
	double dInnerAngle = VectorsInnerAngle(tArcPara.tVerDir.dDirX, tArcPara.tVerDir.dDirY, tArcPara.tVerDir.dDirZ, tPlanePara.tPlateVerDir.dDirX, tPlanePara.tPlateVerDir.dDirY, tPlanePara.tPlateVerDir.dDirZ);
	double dAngleError = 0.5;

	if (fabs(dInnerAngle - 0.0) < dAngleError || fabs(dInnerAngle - 180.0) < dAngleError)
	{
		return FALSE;
	}

	T_XIGEO_PLANE_PARA tFirstPlanePara{};
	T_XIGEO_PLANE_PARA tSecondPlanePara;
	T_XIGEO_LINE_PARA tIntersectionLine;

	tFirstPlanePara.tPlateVerDir = tArcPara.tVerDir;
	tFirstPlanePara.tPoint = tArcPara.tCenterPoint;
	tSecondPlanePara = tPlanePara;

	GetTwoPlaneIntersectionLine(tFirstPlanePara, tPlanePara, tIntersectionLine);

	XI_POINT tLineStartPoint, tLineEndPoint{};
	tLineStartPoint = tIntersectionLine.tPoint;
	tLineEndPoint.x = tLineStartPoint.x + 20.0 * tIntersectionLine.tLineDir.dDirX;
	tLineEndPoint.y = tLineStartPoint.y + 20.0 * tIntersectionLine.tLineDir.dDirY;
	tLineEndPoint.z = tLineStartPoint.z + 20.0 * tIntersectionLine.tLineDir.dDirZ;

	T_LINE_DIR_3D tIdealVerLineDir{};
	tIdealVerLineDir.dDirX = 0.0;
	tIdealVerLineDir.dDirY = 0.0;
	tIdealVerLineDir.dDirZ = 1.0;

	T_POINT_3D tLineStartPoint3D, tLineEndPoint3D;
	T_POINT_3D tCenterPoint3D;

	double dAngleBetweenIdealToReal = VectorsInnerAngle(tIdealVerLineDir.dDirX, tIdealVerLineDir.dDirY, tIdealVerLineDir.dDirZ,
		tArcPara.tVerDir.dDirX, tArcPara.tVerDir.dDirY, tArcPara.tVerDir.dDirZ);

	P2P(tLineStartPoint, tLineStartPoint3D);
	P2P(tLineEndPoint, tLineEndPoint3D);
	P2P(tArcPara.tCenterPoint, tCenterPoint3D);

	if (fabs(dAngleBetweenIdealToReal - 0.0) < dAngleError || fabs(dAngleBetweenIdealToReal - 180.0) < dAngleError)
	{
		m_cAnalyzeObj.CalcLineArcIntersectionForTracking(tLineStartPoint3D, tLineEndPoint3D, tCenterPoint3D, tArcPara.dRadius, vtIntersectionPoint);
	}
	else
	{
		T_LINE_DIR_3D tArcPlaneVerLineDir;
		TransXiLineDirToLineDir(tArcPara.tVerDir, tArcPlaneVerLineDir);

		T_LINE_DIR_3D  tRotateLineDir = m_cAnalyzeObj.CrossProduct(tArcPlaneVerLineDir, tIdealVerLineDir);

		double dTheta = 0.0;
		double dTransMatrix[4][4];
		double dInverseMatrix[16]{};
		double dInverseTransMatrix[4][4]{};
		int nRows = 4, nCols = 4;
		dTheta = VectorsInnerAngle(tArcPlaneVerLineDir.dDirX, tArcPlaneVerLineDir.dDirY, tArcPlaneVerLineDir.dDirZ, tIdealVerLineDir.dDirX, tIdealVerLineDir.dDirY, tIdealVerLineDir.dDirZ);
		m_cXiRobotAbsCoorsTrans->GetRotateAnyDirMatrix(dTheta, tRotateLineDir.dDirX, tRotateLineDir.dDirY, tRotateLineDir.dDirZ, dTransMatrix);

		for (nRows = 0; nRows < 4; nRows++)
		{
			for (nCols = 0; nCols < 4; nCols++)
			{
				dInverseMatrix[nRows * 4 + nCols] = dTransMatrix[nRows][nCols];
			}
		}

		m_cXiRobotAbsCoorsTrans->InverseMatrix(dInverseMatrix, 4);

		for (nRows = 0; nRows < 4; nRows++)
		{
			for (nCols = 0; nCols < 4; nCols++)
			{
				dInverseTransMatrix[nRows][nCols] = dInverseMatrix[nRows * 4 + nCols];
			}
		}

		CoorTransform(tLineStartPoint3D, dTransMatrix);
		CoorTransform(tLineEndPoint3D, dTransMatrix);
		CoorTransform(tCenterPoint3D, dTransMatrix);

		m_cAnalyzeObj.CalcLineArcIntersectionForTracking(tLineStartPoint3D, tLineEndPoint3D, tCenterPoint3D, tArcPara.dRadius, vtIntersectionPoint);

		for (int nPointNo = 0; nPointNo < vtIntersectionPoint.size(); nPointNo++)
		{
			CoorTransform(vtIntersectionPoint[nPointNo], dInverseTransMatrix);
		}
	}


	if (vtIntersectionPoint.size() > 0)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

bool RTT::AdjustTrack::GetTwoPlaneIntersectionLine(T_XIGEO_PLANE_PARA tFirstPlanePara, T_XIGEO_PLANE_PARA tSecondPlanePara, T_XIGEO_LINE_PARA& tIntersectionLine)
{
	T_LINE_DIR_3D tFirstPlaneVerDir, tSecondPlaneVerDir;
	TransXiLineDirToLineDir(tFirstPlanePara.tPlateVerDir, tFirstPlaneVerDir);
	TransXiLineDirToLineDir(tSecondPlanePara.tPlateVerDir, tSecondPlaneVerDir);

	T_LINE_DIR_3D tIntersectionLineDir;
	T_POINT_3D tPointOnPlane;
	T_XIGEO_PLANE_PARAMTYPE tPlaneParamType;
	double  dDisPointToPlane = 0.0;

	double dInnerAngle = VectorsInnerAngle(tFirstPlaneVerDir.dDirX, tFirstPlaneVerDir.dDirY, tFirstPlaneVerDir.dDirZ, tSecondPlaneVerDir.dDirX, tSecondPlaneVerDir.dDirY, tSecondPlaneVerDir.dDirZ);

	if (fabs(dInnerAngle) == 0.0 || fabs(dInnerAngle) == 180.0)
	{
		return false;
	}
	else
	{
		tIntersectionLineDir = m_cAnalyzeObj.CrossProduct(tFirstPlaneVerDir, tSecondPlaneVerDir);

		tPointOnPlane.x = tFirstPlanePara.tPoint.x;
		tPointOnPlane.y = tFirstPlanePara.tPoint.y;
		tPointOnPlane.z = tFirstPlanePara.tPoint.z;

		ConvertPlaneType(tSecondPlanePara, tPlaneParamType);
		dDisPointToPlane = CalDisPointToPlane(tPointOnPlane, tPlaneParamType);

		tIntersectionLine.tLineDir.dDirX = tIntersectionLineDir.dDirX;
		tIntersectionLine.tLineDir.dDirY = tIntersectionLineDir.dDirY;
		tIntersectionLine.tLineDir.dDirZ = tIntersectionLineDir.dDirZ;

		if (dDisPointToPlane == 0.0)
		{
			tIntersectionLine.tPoint.x = tPointOnPlane.x;
			tIntersectionLine.tPoint.y = tPointOnPlane.y;
			tIntersectionLine.tPoint.z = tPointOnPlane.z;

			return true;
		}
		else
		{
			T_LINE_DIR_3D tCommonVerLineDir;
			tCommonVerLineDir = m_cAnalyzeObj.CrossProduct(tFirstPlaneVerDir, tIntersectionLineDir);
			T_XIGEO_LINE_DIR tXiGeoCommonVerLineDir;
			TransLineDirToXiLineDir(tCommonVerLineDir, tXiGeoCommonVerLineDir);
			T_XIGEO_LINE_PARA tCommonVerLinePara{};
			tCommonVerLinePara.tLineDir = tXiGeoCommonVerLineDir;
			tCommonVerLinePara.tPoint = tFirstPlanePara.tPoint;
			T_POINT_3D tPointOnCommonVerLine;
			LinePlateInterSection(tCommonVerLinePara, tSecondPlanePara, tPointOnCommonVerLine);

			tIntersectionLine.tPoint.x = tPointOnCommonVerLine.x;
			tIntersectionLine.tPoint.y = tPointOnCommonVerLine.y;
			tIntersectionLine.tPoint.z = tPointOnCommonVerLine.z;

			return true;
		}
	}
}

double RTT::AdjustTrack::CalAngleEntityAndLine(T_POINT_3D tIntersectionPoint, T_DXF_ENTITY_PARA tEntity, T_POINT_3D tLineStartPoint, T_POINT_3D tLineEndPoint)
{
	T_LINE_DIR_3D tEntityDir;
	T_LINE_DIR_3D tLineDir;
	double dAngle = 0.0;

	if (tEntity.eEntityType == E_DXF_ENTITY_TYPE_LINE)
	{
		tEntityDir = CalLineDir(tEntity.dStartX, tEntity.dStartY, tEntity.dStartZ, tEntity.dEndX, tEntity.dEndY, tEntity.dEndZ);
		tLineDir = CalLineDir(tLineStartPoint.x, tLineStartPoint.y, tLineStartPoint.z, tLineEndPoint.x, tLineEndPoint.y, tLineEndPoint.z);
		dAngle = VectorsInnerAngle(tEntityDir.dDirX, tEntityDir.dDirY, tEntityDir.dDirZ, tLineDir.dDirX, tLineDir.dDirY, tLineDir.dDirZ);
	}
	else if (tEntity.eEntityType == E_DXF_ENTITY_TYPE_ARC_CLOCKWISE || tEntity.eEntityType == E_DXF_ENTITY_TYPE_ARC_ANTICLOCKWISE)
	{
		tEntityDir = CalLineDir(tIntersectionPoint.x, tIntersectionPoint.y, 0.0, tEntity.dCenterX, tEntity.dCenterY, 0.0);
		tLineDir = CalLineDir(tLineStartPoint.x, tLineStartPoint.y, 0.0, tLineEndPoint.x, tLineEndPoint.y, 0.0);
		dAngle = VectorsInnerAngle(-tEntityDir.dDirY, tEntityDir.dDirX, 0.0, tLineDir.dDirX, tLineDir.dDirY, 0.0);
	}

	return dAngle;
}

double RTT::AdjustTrack::VectorsInnerAngle(double dFirstVecX, double dFirstVecY, double dFirstVecZ, double dSecondVecX, double dSecondVecY, double dSecondVecZ)
{
	double dInnerProduct = 0.0;
	double dFirstLength = 0.0, dSecondLength = 0.0;
	double dVectorAngle = 0.0;
	double dResultAngle = 0.0;

	dFirstLength = sqrt(square(dFirstVecX) + square(dFirstVecY) + square(dFirstVecZ));
	dSecondLength = sqrt(square(dSecondVecX) + square(dSecondVecY) + square(dSecondVecZ));

	if (fabs(dFirstLength) < 0.0001 || fabs(dSecondLength) < 0.0001)
	{
		dResultAngle = 0.0;
		return dResultAngle;
	}

	dInnerProduct = dFirstVecX * dSecondVecX + dFirstVecY * dSecondVecY + dFirstVecZ * dSecondVecZ;

	if (fabs(dInnerProduct / (dFirstLength * dSecondLength)) > 1.0)
	{
		dResultAngle = 0.0;
		return dResultAngle;
	}

	return RadToDegree(acos(dInnerProduct / (dFirstLength * dSecondLength)));
}

void RTT::AdjustTrack::ConvertPlaneType(T_XIGEO_PLANE_PARA tPlanePara, T_XIGEO_PLANE_PARAMTYPE& tPlaneParamType)
{
	tPlaneParamType.a = tPlanePara.tPlateVerDir.dDirX;
	tPlaneParamType.b = tPlanePara.tPlateVerDir.dDirY;
	tPlaneParamType.c = tPlanePara.tPlateVerDir.dDirZ;
	tPlaneParamType.d = -(tPlanePara.tPlateVerDir.dDirX * tPlanePara.tPoint.x + tPlanePara.tPlateVerDir.dDirY * tPlanePara.tPoint.y + tPlanePara.tPlateVerDir.dDirZ * tPlanePara.tPoint.z);
}

double RTT::AdjustTrack::CalDisPointToPlane(T_POINT_3D tPoint, T_XIGEO_PLANE_PARAMTYPE tPlaneParam)
{
	double dTmp = sqrt(pow(tPlaneParam.a, 2.0) + pow(tPlaneParam.b, 2.0) + pow(tPlaneParam.c, 2.0));

	if (fabs(dTmp) < 1e-3)
	{
		dTmp = 1.0;
	}

	return fabs(tPlaneParam.a * tPoint.x + tPlaneParam.b * tPoint.y + tPlaneParam.c * tPoint.z + tPlaneParam.d) / dTmp;
}

T_XIGEO_PLANE_PARA RTT::AdjustTrack::CalcLaserPlate(T_ROBOT_COORS_IN_WORLD tCutPoint, E_CAM_ID eCamId)
{
	int nRobotId = eCamId / 4;
	int nCamId = eCamId % 4;
	double dMatrixCameraToTool[4][4];

	auto tCameraPara = m_pUnit->GetCameraParam(m_pUnit->m_nTrackCameraNo);

	T_ROBOT_COORS tCameraReadyRobotCoors = tCameraPara.tHandEyeCaliPara.tCameraReadyRobotCoors;
	double dXMoveToGun = tCameraPara.tHandEyeCaliPara.tGunMoveToCameraCenterRobotCoors.dX - tCameraPara.tHandEyeCaliPara.tCameraReadyRobotCoors.dX;
	double dYMoveToGun = tCameraPara.tHandEyeCaliPara.tGunMoveToCameraCenterRobotCoors.dY - tCameraPara.tHandEyeCaliPara.tCameraReadyRobotCoors.dY;
	double dZMoveToGun = tCameraPara.tHandEyeCaliPara.tGunMoveToCameraCenterRobotCoors.dZ - tCameraPara.tHandEyeCaliPara.tCameraReadyRobotCoors.dZ;

	double dCameraDx = tCameraPara.tHandEyeCaliPara.tCameraInnerPara.dPixelX;
	double dCameraDy = tCameraPara.tHandEyeCaliPara.tCameraInnerPara.dPixelY;
	double dCameraFocal = tCameraPara.tHandEyeCaliPara.tCameraInnerPara.dFocal;
	double dCameraBaseLineLength = tCameraPara.tHandEyeCaliPara.tCameraInnerPara.dBaseLineLength;
	double dCameraBaseLineCtan = tCameraPara.tHandEyeCaliPara.tCameraInnerPara.dCtanBaseLineAngle;

	m_cXiRobotAbsCoorsTrans->m_pCamDirInRobotBase[nRobotId][nCamId][0] = tCameraPara.tHandEyeCaliPara.nCameraXAxisInRobotCoordinate;
	m_cXiRobotAbsCoorsTrans->m_pCamDirInRobotBase[nRobotId][nCamId][1] = tCameraPara.tHandEyeCaliPara.nCameraYAxisInRobotCoordinate;
	m_cXiRobotAbsCoorsTrans->m_pCamDirInRobotBase[nRobotId][nCamId][2] = tCameraPara.tHandEyeCaliPara.nCameraZAxisInRobotCoordinate;

	T_POINT_3D tLaserPlateDirPoint;
	tLaserPlateDirPoint.x = 0.0;
	tLaserPlateDirPoint.y = sin(atan(1 / dCameraBaseLineCtan));
	tLaserPlateDirPoint.z = -cos(atan(1 / dCameraBaseLineCtan));

	T_POINT_3D tPointOnLaserPlate;
	tPointOnLaserPlate.x = 0.0;
	tPointOnLaserPlate.y = 0.0;
	tPointOnLaserPlate.z = 0.0;

	double dPointInCameraMatrix[4][4]{};
	dPointInCameraMatrix[0][0] = 1.0; dPointInCameraMatrix[0][1] = 0.0; dPointInCameraMatrix[0][2] = 0.0; dPointInCameraMatrix[0][3] = tPointOnLaserPlate.x;
	dPointInCameraMatrix[1][0] = 0.0; dPointInCameraMatrix[1][1] = 1.0; dPointInCameraMatrix[1][2] = 0.0; dPointInCameraMatrix[1][3] = tPointOnLaserPlate.y;
	dPointInCameraMatrix[2][0] = 0.0; dPointInCameraMatrix[2][1] = 0.0; dPointInCameraMatrix[2][2] = 1.0; dPointInCameraMatrix[2][3] = tPointOnLaserPlate.z;
	dPointInCameraMatrix[3][0] = 0.0; dPointInCameraMatrix[3][1] = 0.0; dPointInCameraMatrix[3][2] = 0.0; dPointInCameraMatrix[3][3] = 1.0;

	double dDirPointInCameraMatrix[4][4]{};
	dDirPointInCameraMatrix[0][0] = 1.0; dDirPointInCameraMatrix[0][1] = 0.0; dDirPointInCameraMatrix[0][2] = 0.0; dDirPointInCameraMatrix[0][3] = tLaserPlateDirPoint.x;
	dDirPointInCameraMatrix[1][0] = 0.0; dDirPointInCameraMatrix[1][1] = 1.0; dDirPointInCameraMatrix[1][2] = 0.0; dDirPointInCameraMatrix[1][3] = tLaserPlateDirPoint.y;
	dDirPointInCameraMatrix[2][0] = 0.0; dDirPointInCameraMatrix[2][1] = 0.0; dDirPointInCameraMatrix[2][2] = 1.0; dDirPointInCameraMatrix[2][3] = tLaserPlateDirPoint.z;
	dDirPointInCameraMatrix[3][0] = 0.0; dDirPointInCameraMatrix[3][1] = 0.0; dDirPointInCameraMatrix[3][2] = 0.0; dDirPointInCameraMatrix[3][3] = 1.0;

	m_cXiRobotAbsCoorsTrans->GenerateCameraToToolMatrix(tCameraReadyRobotCoors, dXMoveToGun, dYMoveToGun, dZMoveToGun, dMatrixCameraToTool, eCamId);

	double dGunMatrixInBase[4][4];
	m_cXiRobotAbsCoorsTrans->GenerateToolToBaseMatrix(tCutPoint.dX, tCutPoint.dY, tCutPoint.dZ, tCutPoint.dRX, tCutPoint.dRY, tCutPoint.dRZ, dGunMatrixInBase);

	double dCameraMatrixInBase[4][4];
	int nRows = 4;
	int nCols = 4;
	m_cXiRobotAbsCoorsTrans->Trmul(dGunMatrixInBase, nRows, nCols, dMatrixCameraToTool, nRows, nCols, dCameraMatrixInBase, nRows, nCols);

	double dPointInBaseMatrix[4][4];
	double dDirPointInBaseMatrix[4][4];

	m_cXiRobotAbsCoorsTrans->Trmul(dCameraMatrixInBase, nRows, nCols, dPointInCameraMatrix, nRows, nCols, dPointInBaseMatrix, nRows, nCols);
	m_cXiRobotAbsCoorsTrans->Trmul(dCameraMatrixInBase, nRows, nCols, dDirPointInCameraMatrix, nRows, nCols, dDirPointInBaseMatrix, nRows, nCols);

	T_POINT_3D tPointInBase, tDirPointInBase;

	tPointInBase.x = dPointInBaseMatrix[0][3];
	tPointInBase.y = dPointInBaseMatrix[1][3];
	tPointInBase.z = dPointInBaseMatrix[2][3];

	tDirPointInBase.x = dDirPointInBaseMatrix[0][3];
	tDirPointInBase.y = dDirPointInBaseMatrix[1][3];
	tDirPointInBase.z = dDirPointInBaseMatrix[2][3];

	T_LINE_DIR_3D tPlaneVerLineDir;
	tPlaneVerLineDir = CalLineDir(tPointInBase.x, tPointInBase.y, tPointInBase.z, tDirPointInBase.x, tDirPointInBase.y, tDirPointInBase.z);

	XI_POINT tPlanePoint{};
	T_XIGEO_LINE_DIR tPlateVerDir{};

	tPlanePoint.x = tPointInBase.x;
	tPlanePoint.y = tPointInBase.y;
	tPlanePoint.z = tPointInBase.z;

	tPlateVerDir.dDirX = tPlaneVerLineDir.dDirX;
	tPlateVerDir.dDirY = tPlaneVerLineDir.dDirY;
	tPlateVerDir.dDirZ = tPlaneVerLineDir.dDirZ;

	T_XIGEO_PLANE_PARA tPlanePara{};
	tPlanePara.tPlateVerDir = tPlateVerDir;
	tPlanePara.tPoint = tPlanePoint;

	return tPlanePara;
}

void RTT::AdjustTrack::InitPointInCamerMatrix(T_POINT_3D tPoint, double dMatrix[][4])
{
	dMatrix[0][0] = 1.0; dMatrix[0][1] = 0.0; dMatrix[0][2] = 0.0; dMatrix[0][3] = tPoint.x;
	dMatrix[1][0] = 0.0; dMatrix[1][1] = 1.0; dMatrix[1][2] = 0.0; dMatrix[1][3] = tPoint.y;
	dMatrix[2][0] = 0.0; dMatrix[2][1] = 0.0; dMatrix[2][2] = 1.0; dMatrix[2][3] = tPoint.z;
	dMatrix[3][0] = 0.0; dMatrix[3][1] = 0.0; dMatrix[3][2] = 0.0; dMatrix[3][3] = 1.0;
}

T_XIGEO_PLANE_PARA RTT::AdjustTrack::CalcTrackCameraAndLaserParam(T_ROBOT_COORS_IN_WORLD tWeldPoint, T_LINE_DIR_3D tWeldLineDir, E_CAM_ID eCamId, T_POINT_3D &tLaserCenterPoint, T_POINT_3D &tCamCenterPoint, std::vector<T_POINT_3D> &vtLaserPlanePoints, std::vector<T_POINT_3D> &vtCameraPoints)
{
	vtCameraPoints.clear();
	vtLaserPlanePoints.clear();

	int nRobotId = eCamId / 4;
	int nCamId = eCamId % 4;
	double dMatrixCameraToTool[4][4];

	auto tCameraPara = m_pUnit->GetCameraParam(m_pUnit->m_nMeasureCameraNo);

	T_ROBOT_COORS tCameraReadyRobotCoors = tCameraPara.tHandEyeCaliPara.tCameraReadyRobotCoors;
	double dXMoveToGun = tCameraPara.tHandEyeCaliPara.tGunMoveToCameraCenterRobotCoors.dX - tCameraPara.tHandEyeCaliPara.tCameraReadyRobotCoors.dX;
	double dYMoveToGun = tCameraPara.tHandEyeCaliPara.tGunMoveToCameraCenterRobotCoors.dY - tCameraPara.tHandEyeCaliPara.tCameraReadyRobotCoors.dY;
	double dZMoveToGun = tCameraPara.tHandEyeCaliPara.tGunMoveToCameraCenterRobotCoors.dZ - tCameraPara.tHandEyeCaliPara.tCameraReadyRobotCoors.dZ;

	double dCameraDx = tCameraPara.tHandEyeCaliPara.tCameraInnerPara.dPixelX;
	double dCameraDy = tCameraPara.tHandEyeCaliPara.tCameraInnerPara.dPixelY;
	double dCameraFocal = tCameraPara.tHandEyeCaliPara.tCameraInnerPara.dFocal;
	double dCameraBaseLineLength = tCameraPara.tHandEyeCaliPara.tCameraInnerPara.dBaseLineLength;
	double dCameraBaseLineCtan = tCameraPara.tHandEyeCaliPara.tCameraInnerPara.dCtanBaseLineAngle;

	m_cXiRobotAbsCoorsTrans->m_pCamDirInRobotBase[nRobotId][nCamId][0] = tCameraPara.tHandEyeCaliPara.nCameraXAxisInRobotCoordinate;
	m_cXiRobotAbsCoorsTrans->m_pCamDirInRobotBase[nRobotId][nCamId][1] = tCameraPara.tHandEyeCaliPara.nCameraYAxisInRobotCoordinate;
	m_cXiRobotAbsCoorsTrans->m_pCamDirInRobotBase[nRobotId][nCamId][2] = tCameraPara.tHandEyeCaliPara.nCameraZAxisInRobotCoordinate;

	T_POINT_3D tLaserPlateDirPoint;
	tLaserPlateDirPoint.x = 0.0;
	tLaserPlateDirPoint.y = sin(atan(1 / dCameraBaseLineCtan));
	tLaserPlateDirPoint.z = -cos(atan(1 / dCameraBaseLineCtan));

	T_POINT_3D tPointOnLaserPlate;
	tPointOnLaserPlate.x = 0.0;
	tPointOnLaserPlate.y = 0.0;
	tPointOnLaserPlate.z = 0.0;

	T_POINT_3D tLeftPointOnLaserPlate, tRightPointOnLaserPlate;
	tLeftPointOnLaserPlate.x = -70.0;
	tLeftPointOnLaserPlate.y = 0.0;
	tLeftPointOnLaserPlate.z = 0.0;

	tRightPointOnLaserPlate.x = 70.0;
	tRightPointOnLaserPlate.y = 0.0;
	tRightPointOnLaserPlate.z = 0.0;

	T_POINT_3D tCameraCenter, tLaserCenter;
	tCameraCenter.x = 0.0;
	tCameraCenter.y = 0.0;
	tCameraCenter.z = -fabs(dCameraBaseLineLength/dCameraBaseLineCtan);

	tLaserCenter.x = 0.0;
	tLaserCenter.y = -dCameraBaseLineLength;
	tLaserCenter.z = -fabs(dCameraBaseLineLength/dCameraBaseLineCtan);

	double dPointInCameraMatrix[4][4];
	InitPointInCamerMatrix(tPointOnLaserPlate, dPointInCameraMatrix);

	double dCamCenterInCameraMatrix[4][4];
	double dLaserCenterInCameraMatrix[4][4];
	InitPointInCamerMatrix(tCameraCenter, dCamCenterInCameraMatrix);
	InitPointInCamerMatrix(tLaserCenter, dLaserCenterInCameraMatrix);
	
	double dLeftPointInCameraMatrix[4][4];
	double dRightPointInCameraMatrix[4][4];
	InitPointInCamerMatrix(tLeftPointOnLaserPlate, dLeftPointInCameraMatrix);
	InitPointInCamerMatrix(tRightPointOnLaserPlate, dRightPointInCameraMatrix);
	
	double dDirPointInCameraMatrix[4][4];
	dDirPointInCameraMatrix[0][0] = 1.0; dDirPointInCameraMatrix[0][1] = 0.0; dDirPointInCameraMatrix[0][2] = 0.0; dDirPointInCameraMatrix[0][3] = tLaserPlateDirPoint.x;
	dDirPointInCameraMatrix[1][0] = 0.0; dDirPointInCameraMatrix[1][1] = 1.0; dDirPointInCameraMatrix[1][2] = 0.0; dDirPointInCameraMatrix[1][3] = tLaserPlateDirPoint.y;
	dDirPointInCameraMatrix[2][0] = 0.0; dDirPointInCameraMatrix[2][1] = 0.0; dDirPointInCameraMatrix[2][2] = 1.0; dDirPointInCameraMatrix[2][3] = tLaserPlateDirPoint.z;
	dDirPointInCameraMatrix[3][0] = 0.0; dDirPointInCameraMatrix[3][1] = 0.0; dDirPointInCameraMatrix[3][2] = 0.0; dDirPointInCameraMatrix[3][3] = 1.0;

	m_cXiRobotAbsCoorsTrans->GenerateCameraToToolMatrix(tCameraReadyRobotCoors, dXMoveToGun, dYMoveToGun, dZMoveToGun, dMatrixCameraToTool, eCamId);

	double dCameraMatrixInBase[4][4];
	double dTestMatrix[4][4];
	int nRows = 4;
	int nCols = 4;
//	double dGunMatrixInBase[4][4];
	m_cXiRobotAbsCoorsTrans->GenerateToolToBaseMatrix(tWeldPoint.dX, tWeldPoint.dY, tWeldPoint.dZ, tWeldPoint.dRX, tWeldPoint.dRY, tWeldPoint.dRZ, dTestMatrix);
	m_cXiRobotAbsCoorsTrans->Trmul(dTestMatrix, nRows, nCols, dMatrixCameraToTool, nRows, nCols, dCameraMatrixInBase, nRows, nCols);
	dCameraMatrixInBase[0][3] = tWeldPoint.dX;
	dCameraMatrixInBase[1][3] = tWeldPoint.dY;
	dCameraMatrixInBase[2][3] = tWeldPoint.dZ;

	double dPointInBaseMatrix[4][4];
	double dDirPointInBaseMatrix[4][4];
	double dCamCenterInBaseMatrix[4][4];
	double dLaserCenterInBaseMatrix[4][4];
	double dLeftPointInBaseMatrix[4][4];
	double dRightPointInBaseMatrix[4][4];

	m_cXiRobotAbsCoorsTrans->Trmul(dCameraMatrixInBase, nRows, nCols, dPointInCameraMatrix, nRows, nCols, dPointInBaseMatrix, nRows, nCols);
	m_cXiRobotAbsCoorsTrans->Trmul(dCameraMatrixInBase, nRows, nCols, dDirPointInCameraMatrix, nRows, nCols, dDirPointInBaseMatrix, nRows, nCols);

	m_cXiRobotAbsCoorsTrans->Trmul(dCameraMatrixInBase, nRows, nCols, dCamCenterInCameraMatrix, nRows, nCols, dCamCenterInBaseMatrix, nRows, nCols);
	m_cXiRobotAbsCoorsTrans->Trmul(dCameraMatrixInBase, nRows, nCols, dLaserCenterInCameraMatrix, nRows, nCols, dLaserCenterInBaseMatrix, nRows, nCols);

	m_cXiRobotAbsCoorsTrans->Trmul(dCameraMatrixInBase, nRows, nCols, dLeftPointInCameraMatrix, nRows, nCols, dLeftPointInBaseMatrix, nRows, nCols);
	m_cXiRobotAbsCoorsTrans->Trmul(dCameraMatrixInBase, nRows, nCols, dRightPointInCameraMatrix, nRows, nCols, dRightPointInBaseMatrix, nRows, nCols);

	tCamCenterPoint.x = dCamCenterInBaseMatrix[0][3];
	tCamCenterPoint.y = dCamCenterInBaseMatrix[1][3];
	tCamCenterPoint.z = dCamCenterInBaseMatrix[2][3];

	tLaserCenterPoint.x = dLaserCenterInBaseMatrix[0][3];
	tLaserCenterPoint.y = dLaserCenterInBaseMatrix[1][3];
	tLaserCenterPoint.z = dLaserCenterInBaseMatrix[2][3];

	T_POINT_3D tLeftPointInBase, tRightPointInBase;
	tLeftPointInBase.x = dLeftPointInBaseMatrix[0][3];
	tLeftPointInBase.y = dLeftPointInBaseMatrix[1][3];
	tLeftPointInBase.z = dLeftPointInBaseMatrix[2][3];

	tRightPointInBase.x = dRightPointInBaseMatrix[0][3];
	tRightPointInBase.y = dRightPointInBaseMatrix[1][3];
	tRightPointInBase.z = dRightPointInBaseMatrix[2][3];

	vtLaserPlanePoints.push_back(tLeftPointInBase);
	vtLaserPlanePoints.push_back(tRightPointInBase);

	double dTransWeldPostureMatrix[4][4];
	double dRotateDirMatrix[4][4];
	double dLength = 50.0;
	double dEyeTrans = 20.0;
	T_POINT_3D tUpCameraPointInBase, tDownCameraPointInBase;

	T_POINT tStartPoint, tEndPoint;
	tStartPoint.dX = 0.0;
	tStartPoint.dY = 0.0;
	tStartPoint.dZ = 0.0;

	tEndPoint.dX = 0.0;
	tEndPoint.dY = 0.0;
	tEndPoint.dZ = 0.0;
	//T_DIR tWeldLineDir = m_cXiRobotAbsCoorsTrans->CalLineDir(tStartPoint, tEndPoint);
	m_cXiRobotAbsCoorsTrans->GetRotateAnyDirMatrix(45, tWeldLineDir.dDirX, tWeldLineDir.dDirY, tWeldLineDir.dDirZ, dRotateDirMatrix);
	m_cXiRobotAbsCoorsTrans->Trmul(dRotateDirMatrix, nRows, nCols, dTestMatrix, nRows, nCols, dTransWeldPostureMatrix, nRows, nCols);
	tUpCameraPointInBase.x = tWeldPoint.dX - dLength * dTransWeldPostureMatrix[0][2] - dEyeTrans * dCameraMatrixInBase[0][2];
	tUpCameraPointInBase.y = tWeldPoint.dY - dLength * dTransWeldPostureMatrix[1][2] - dEyeTrans * dCameraMatrixInBase[1][2];
	tUpCameraPointInBase.z = tWeldPoint.dZ - dLength * dTransWeldPostureMatrix[2][2] - dEyeTrans * dCameraMatrixInBase[2][2];

	m_cXiRobotAbsCoorsTrans->GetRotateAnyDirMatrix(-45, tWeldLineDir.dDirX, tWeldLineDir.dDirY, tWeldLineDir.dDirZ, dRotateDirMatrix);
	m_cXiRobotAbsCoorsTrans->Trmul(dRotateDirMatrix, nRows, nCols, dTestMatrix, nRows, nCols, dTransWeldPostureMatrix, nRows, nCols);
	tDownCameraPointInBase.x = tWeldPoint.dX - dLength * dTransWeldPostureMatrix[0][2] - dEyeTrans * dCameraMatrixInBase[0][2];
	tDownCameraPointInBase.y = tWeldPoint.dY - dLength * dTransWeldPostureMatrix[1][2] - dEyeTrans * dCameraMatrixInBase[1][2];
	tDownCameraPointInBase.z = tWeldPoint.dZ - dLength * dTransWeldPostureMatrix[2][2] - dEyeTrans * dCameraMatrixInBase[2][2];

	vtCameraPoints.push_back(tUpCameraPointInBase);
	vtCameraPoints.push_back(tDownCameraPointInBase);

	T_POINT_3D tPointInBase, tDirPointInBase;
	tPointInBase.x = dPointInBaseMatrix[0][3];
	tPointInBase.y = dPointInBaseMatrix[1][3];
	tPointInBase.z = dPointInBaseMatrix[2][3];

	tDirPointInBase.x = dDirPointInBaseMatrix[0][3];
	tDirPointInBase.y = dDirPointInBaseMatrix[1][3];
	tDirPointInBase.z = dDirPointInBaseMatrix[2][3];

	T_LINE_DIR_3D tPlaneVerLineDir;
	tPlaneVerLineDir = CalLineDir(tPointInBase.x, tPointInBase.y, tPointInBase.z, tDirPointInBase.x, tDirPointInBase.y, tDirPointInBase.z);

	XI_POINT tPlanePoint{};
	T_XIGEO_LINE_DIR tPlateVerDir{};

	tPlanePoint.x = tPointInBase.x;
	tPlanePoint.y = tPointInBase.y;
	tPlanePoint.z = tPointInBase.z;

	tPlateVerDir.dDirX = tPlaneVerLineDir.dDirX;
	tPlateVerDir.dDirY = tPlaneVerLineDir.dDirY;
	tPlateVerDir.dDirZ = tPlaneVerLineDir.dDirZ;

	T_XIGEO_PLANE_PARA tPlanePara{};
	tPlanePara.tPlateVerDir = tPlateVerDir;
	tPlanePara.tPoint = tPlanePoint;

	return tPlanePara;
}

void RTT::AdjustTrack::TransData(std::vector<T_POINT_3D> vtPoints, std::vector<coll::Point3D> &vtTransPoints)
{
	vtTransPoints.clear();

	coll::Point3D tPoint3D;

	for (int nPointNo = 0; nPointNo < vtPoints.size(); nPointNo++)
	{
		tPoint3D.x = vtPoints[nPointNo].x;
		tPoint3D.y = vtPoints[nPointNo].y;
		tPoint3D.z = vtPoints[nPointNo].z;

		vtTransPoints.push_back(tPoint3D);
	}
}

bool RTT::AdjustTrack::CheckCollision(std::vector<T_POINT_3D> vtLaserPolygon, std::vector<T_POINT_3D> vtCameraPolygon, std::vector<vector<T_POINT_3D>> vvtCheckBlockPoints)
{
	std::vector<coll::Point3D> vtTransLaserPoints;
	std::vector<coll::Point3D> vtTransCameraPoints;
	std::vector<coll::Point3D> vtTransEntityPoints;
	bool bIfCollision = false;

	for (int nEntityNo = 0;  nEntityNo < vvtCheckBlockPoints.size(); nEntityNo++)
	{
		TransData(vtLaserPolygon, vtTransLaserPoints);
		TransData(vtCameraPolygon, vtTransCameraPoints);
		TransData(vvtCheckBlockPoints[nEntityNo], vtTransEntityPoints);

		coll::CollisionObject cObjLaser(vtTransLaserPoints);
		coll::CollisionObject cObjCamera(vtTransCameraPoints);
		coll::CollisionObject cObjBlock(vtTransEntityPoints);

		bool bLasercollision = coll::arePolygonsColliding(cObjLaser, cObjBlock);
		bool bCameracollision = coll::arePolygonsColliding(cObjCamera, cObjBlock);

		if (bLasercollision || bCameracollision)
		{
			bIfCollision = true;
			return bIfCollision;
		}
	}
	return bIfCollision;
}

bool RTT::AdjustTrack::CalIfCollison(T_ROBOT_COORS_IN_WORLD tRobotCoors, T_LINE_DIR_3D tWeldLineDir, E_CAM_ID eCamId, std::vector<T_XIGEO_WELDLINE> vtCheckBlockPlate)
{
	T_ROBOT_COORS_IN_WORLD tRobotCoor;
	T_POINT_3D tLaserCenterPoint;
	T_POINT_3D tCamCenterPoint;
	std::vector<T_POINT_3D> vtLaserPlanePoints;
	std::vector<T_POINT_3D> vtCameraPoints;

	vtLaserPlanePoints.clear();
	vtCameraPoints.clear();

	CalcTrackCameraAndLaserParam(tRobotCoors, tWeldLineDir, eCamId, tLaserCenterPoint, tCamCenterPoint, vtLaserPlanePoints, vtCameraPoints);

	std::vector<T_POINT_3D> vtLaserPolygonPoints;
	vtLaserPolygonPoints.clear();
	vtLaserPolygonPoints.push_back(tLaserCenterPoint);
	vtLaserPolygonPoints.insert(vtLaserPolygonPoints.end(), vtCameraPoints.begin(), vtCameraPoints.end());

	std::vector<T_POINT_3D> vtCameraPolygonPoints;
	vtCameraPolygonPoints.clear();
	vtCameraPolygonPoints.push_back(tCamCenterPoint);
	vtCameraPolygonPoints.insert(vtCameraPolygonPoints.end(), vtCameraPoints.begin(), vtCameraPoints.end());

	CString sFileName;
	sFileName.Format("Avoid\\CameraPolygon.txt");
	FILE* pf = fopen(sFileName, "w");
	for (auto& cl : vtCameraPolygonPoints)
	{
		fprintf(pf,
			"%11.3lf %11.3lf %11.3lf\n",
			cl.x, cl.y, cl.z);
	}
	fclose(pf);

	CString sFileName1;
	sFileName1.Format("Avoid\\LaserPolygon.txt");
	pf = fopen(sFileName1, "w");
	for (auto& c2 : vtLaserPolygonPoints)
	{
		fprintf(pf,
			"%11.3lf %11.3lf %11.3lf\n",
			c2.x, c2.y, c2.z);
	}
	fclose(pf);

	std::vector<T_POINT_3D> vtBlockPoints;
	std::vector<vector<T_POINT_3D>> vvtBlockPoints;
	vtBlockPoints.clear();
	vvtBlockPoints.clear();

	for (int nEntityNo = 0; nEntityNo < vtCheckBlockPlate.size(); nEntityNo++)
	{
		vtBlockPoints.clear();
		vtBlockPoints.push_back(vtCheckBlockPlate[nEntityNo].tUpStartPoint);
		vtBlockPoints.push_back(vtCheckBlockPlate[nEntityNo].tUpEndPoint);
		vtBlockPoints.push_back(vtCheckBlockPlate[nEntityNo].tDownEndPoint);
		vtBlockPoints.push_back(vtCheckBlockPlate[nEntityNo].tDownStartPoint);
		vvtBlockPoints.push_back(vtBlockPoints);
	}

	bool bIfAdjust = CheckCollision(vtLaserPolygonPoints, vtCameraPolygonPoints, vvtBlockPoints);

	return bIfAdjust;
}

void RTT::AdjustTrack::CheckInputRobotRxRy(T_ROBOT_COORS_IN_WORLD& tRobotCoors)
{
	if (tRobotCoors.dRX > 180.0)
	{
		tRobotCoors.dRX = tRobotCoors.dRX - 360.0;
	}

	if (tRobotCoors.dRY > 180.0)
	{
		tRobotCoors.dRY = tRobotCoors.dRY - 360.0;
	}

	if (tRobotCoors.dRX < -180.0)
	{
		tRobotCoors.dRX = tRobotCoors.dRX + 360.0;
	}

	if (tRobotCoors.dRY < -180.0)
	{
		tRobotCoors.dRY = tRobotCoors.dRY + 360.0;
	}
}

bool RTT::AdjustTrack::CalIfAdjust(T_ROBOT_COORS_IN_WORLD tRobotCoors, double dRxSearchRange, double dRySearchRange, double dSearchUnit, T_LINE_DIR_3D tWeldLineDir, E_CAM_ID eCamId, std::vector<T_XIGEO_WELDLINE> vtCheckBlockPlate, T_ROBOT_COORS_IN_WORLD& tAdjustRobotCoors)
{
	int nRxSearchNum = int(dRxSearchRange / dSearchUnit);
	int nRySearchNum = int(dRySearchRange / dSearchUnit);

	T_ROBOT_COORS_IN_WORLD tTempRobotCoors;

	tTempRobotCoors = tRobotCoors;

	for (int nRyNo = 0; nRyNo <= int(nRySearchNum / 2.0); nRyNo++)
	{
		for (int nRxNo = 0; nRxNo <= int(nRxSearchNum / 2.0); nRxNo++)
		{
			tTempRobotCoors.dRX = tRobotCoors.dRX + nRxNo * dSearchUnit;
			tTempRobotCoors.dRY = tRobotCoors.dRY + nRyNo * dSearchUnit;
			CheckInputRobotRxRy(tTempRobotCoors);
			bool bIfCollison = CalIfCollison(tTempRobotCoors, tWeldLineDir, eCamId, vtCheckBlockPlate);
			if (!bIfCollison)
			{
				tAdjustRobotCoors = tTempRobotCoors;
				return true;
			}

			tTempRobotCoors.dRX = tRobotCoors.dRX + nRxNo * dSearchUnit;
			tTempRobotCoors.dRY = tRobotCoors.dRY - nRyNo * dSearchUnit;
			CheckInputRobotRxRy(tTempRobotCoors);
			bIfCollison = CalIfCollison(tTempRobotCoors, tWeldLineDir, eCamId, vtCheckBlockPlate);
			if (!bIfCollison)
			{
				tAdjustRobotCoors = tTempRobotCoors;
				return true;
			}

			tTempRobotCoors.dRX = tRobotCoors.dRX - nRxNo * dSearchUnit;
			tTempRobotCoors.dRY = tRobotCoors.dRY + nRyNo * dSearchUnit;
			CheckInputRobotRxRy(tTempRobotCoors);
			bIfCollison = CalIfCollison(tTempRobotCoors, tWeldLineDir, eCamId, vtCheckBlockPlate);
			if (!bIfCollison)
			{
				tAdjustRobotCoors = tTempRobotCoors;
				return true;
			}

			tTempRobotCoors.dRX = tRobotCoors.dRX - nRxNo * dSearchUnit;
			tTempRobotCoors.dRY = tRobotCoors.dRY - nRyNo * dSearchUnit;
			CheckInputRobotRxRy(tTempRobotCoors);
			bIfCollison = CalIfCollison(tTempRobotCoors, tWeldLineDir, eCamId, vtCheckBlockPlate);
			if (!bIfCollison)
			{
				tAdjustRobotCoors = tTempRobotCoors;
				return true;
			}
		}
	}
	return false;
}

//bool RTT::AdjustTrack::CalIfAdjust(T_ROBOT_COORS_IN_WORLD tRobotCoors, double dRxSearchRange, double dRySearchRange, double dSearchUnit, T_LINE_DIR_3D tWeldLineDir, E_CAM_ID eCamId, std::vector<T_XIGEO_WELDLINE> vtCheckBlockPlate, T_ROBOT_COORS_IN_WORLD &tAdjustRobotCoors)
//{
//	int nRxSearchNum = int(dRxSearchRange / dSearchUnit);
//	int nRySearchNum = int(dRySearchRange / dSearchUnit);
//
//	T_ROBOT_COORS_IN_WORLD tTempRobotCoors;
//
//	tTempRobotCoors = tRobotCoors;
//
//	for (int nRyNo = -int(nRySearchNum/2.0); nRyNo <= int(nRySearchNum / 2.0); nRyNo++)
//	{
//		for (int nRxNo = -int(nRxSearchNum / 2.0); nRxNo <= int(nRxSearchNum / 2.0); nRxNo++)
//		{
//			tTempRobotCoors.dRX = tRobotCoors.dRX + nRxNo * dSearchUnit;
//			tTempRobotCoors.dRY = tRobotCoors.dRY + nRyNo * dSearchUnit;
//
//			if (tTempRobotCoors.dRX > 180.0)
//			{
//				tTempRobotCoors.dRX = tTempRobotCoors.dRX - 360.0;
//			}
//
//			if (tTempRobotCoors.dRY > 180.0)
//			{
//				tTempRobotCoors.dRY = tTempRobotCoors.dRY - 360.0;
//			}
//
//			if (tTempRobotCoors.dRX < -180.0)
//			{
//				tTempRobotCoors.dRX = tTempRobotCoors.dRX + 360.0;
//			}
//
//			if (tTempRobotCoors.dRY < -180.0)
//			{
//				tTempRobotCoors.dRY = tTempRobotCoors.dRY + 360.0;
//			}
//
//			bool bIfCollison = CalIfCollison(tTempRobotCoors,tWeldLineDir, eCamId, vtCheckBlockPlate);
//
//			if (!bIfCollison)
//			{
//				tAdjustRobotCoors = tTempRobotCoors;
//				return true;
//			}
//		}
//	}
//	return false;
//}

bool RTT::AdjustTrack::AdjustMeasureRobotCoors(std::vector<T_ROBOT_COORS_IN_WORLD> vtMeasureRobotPoints, std::vector<T_LINE_DIR_3D> vtWeldLineDir, E_CAM_ID eCamId, std::vector<T_XIGEO_WELDLINE> vtCheckBlockPlate, std::vector<T_ROBOT_COORS_IN_WORLD> &vtAdjustMeasureRobotPoints)
{
	T_ROBOT_COORS_IN_WORLD tRobotCoor;
	T_POINT_3D tLaserCenterPoint;
	T_POINT_3D tCamCenterPoint;
	std::vector<T_POINT_3D> vtLaserPlanePoints;
	std::vector<T_POINT_3D> vtCameraPoints;

	double dRxSearchRange = 30.0;//计算测量轨迹时，RX的搜索范围
	double dRySearchRange = 20.0;//计算测量轨迹时，RY的搜索范围
	double dSearchUnit = 5.0;
	T_ROBOT_COORS_IN_WORLD tAdjustRobotCoors;
	bool bIfAdjustSucceed = true;

	for (int nPointNo = 0; nPointNo < vtMeasureRobotPoints.size(); nPointNo++)
	{
		bool bIfAdjust = CalIfCollison(vtMeasureRobotPoints[nPointNo], vtWeldLineDir[nPointNo], eCamId, vtCheckBlockPlate);

		if (bIfAdjust)
		{
			bIfAdjustSucceed = CalIfAdjust(vtMeasureRobotPoints[nPointNo], dRxSearchRange, dRySearchRange, dSearchUnit, vtWeldLineDir[nPointNo], eCamId, vtCheckBlockPlate, tAdjustRobotCoors);

			if (bIfAdjustSucceed)
			{
				vtAdjustMeasureRobotPoints.push_back(tAdjustRobotCoors);
			}
			else
			{
				bIfAdjustSucceed = false;
				vtAdjustMeasureRobotPoints.push_back(vtMeasureRobotPoints[nPointNo]);
			}
		}
		else
		{
			vtAdjustMeasureRobotPoints.push_back(vtMeasureRobotPoints[nPointNo]);
		}
	}
	return bIfAdjustSucceed;
}

T_LINE_DIR_3D RTT::AdjustTrack::CalLineDir(double dStartX, double dStartY, double dStartZ, double dEndX, double dEndY, double dEndZ)
{
	double dLineLength = 0.0;
	T_LINE_DIR_3D tLineDir{};
	double dX = 0.0, dY = 0.0, dZ = 0.0;

	dX = dEndX - dStartX;
	dY = dEndY - dStartY;
	dZ = dEndZ - dStartZ;

	dLineLength = sqrt(square(dX) + square(dY) + square(dZ));

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

T_LINE_DIR RTT::AdjustTrack::CalLineDir(double dStartX, double dStartY, double dEndX, double dEndY)
{
	double dLineLength = 0.0;
	T_LINE_DIR tLineDir{};
	double dX = 0.0, dY = 0.0;

	dX = dEndX - dStartX;
	dY = dEndY - dStartY;

	dLineLength = sqrt(square(dX) + square(dY));

	if (fabs(dLineLength) < 0.0001)
	{
		//AfxMessageBox("直线方向计算有误");
		tLineDir.dDirX = 0.0;
		tLineDir.dDirY = 0.0;
	}
	else
	{
		tLineDir.dDirX = dX / dLineLength;
		tLineDir.dDirY = dY / dLineLength;
	}

	return tLineDir;
}

void RTT::AdjustTrack::CoorTransform(T_POINT_3D& tPoint, double dTransMatrix[][4])
{
	double dPointMatrix[4][4]{};
	double dResultMatrix[4][4];
	int nRows = 4;
	int nCols = 4;

	dPointMatrix[0][0] = 1.0;
	dPointMatrix[1][0] = 0.0;
	dPointMatrix[2][0] = 0.0;
	dPointMatrix[3][0] = 0.0;

	dPointMatrix[0][1] = 0.0;
	dPointMatrix[1][1] = 1.0;
	dPointMatrix[2][1] = 0.0;
	dPointMatrix[3][1] = 0.0;

	dPointMatrix[0][2] = 0.0;
	dPointMatrix[1][2] = 0.0;
	dPointMatrix[2][2] = 1.0;
	dPointMatrix[3][2] = 0.0;

	dPointMatrix[0][3] = tPoint.x;
	dPointMatrix[1][3] = tPoint.y;
	dPointMatrix[2][3] = tPoint.z;
	dPointMatrix[3][3] = 1.0;

	m_cXiRobotAbsCoorsTrans->Trmul(dTransMatrix, nRows, nCols, dPointMatrix, nRows, nCols, dResultMatrix, nRows, nCols);

	tPoint.x = dResultMatrix[0][3];
	tPoint.y = dResultMatrix[1][3];
	tPoint.z = dResultMatrix[2][3];
}

int RTT::AdjustTrack::FindNearestPoint(T_POINT_3D tPoint, const std::vector<T_POINT_3D>& vtPoints)
{
	double dDis = 0.0;
	double dMinDis = 999999.0;
	int nMinPointNo = 0;

	for (int nPointNo = 0; nPointNo < vtPoints.size(); nPointNo++)
	{
		dDis = square(tPoint.x - vtPoints[nPointNo].x)
			+ square(tPoint.y - vtPoints[nPointNo].y);

		if (dDis < dMinDis)
		{
			dMinDis = dDis;
			nMinPointNo = nPointNo;
		}
	}

	return nMinPointNo;
}

int RTT::AdjustTrack::FindNearestPointNew(T_POINT_3D tPoint, std::vector<T_POINT_3D> vtPoints, std::vector<int> vnEntityNo, int nEntityNo)
{
	double dDis = 0.0;
	double dMinDis = 999999.0;
	int nMinPointNo = 0;

	for (int nPointNo = 0; nPointNo < vtPoints.size(); nPointNo++)
	{
		if (vnEntityNo[nPointNo] == nEntityNo)
		{
			dDis = square(tPoint.x - vtPoints[nPointNo].x)
				+ square(tPoint.y - vtPoints[nPointNo].y);

			if (dDis < dMinDis)
			{
				dMinDis = dDis;
				nMinPointNo = nPointNo;
			}
		}
	}

	return nMinPointNo;
}


void RTT::AdjustTrack::TransXiLineDirToLineDir(T_XIGEO_LINE_DIR tXiLineDir, T_LINE_DIR_3D& tLineDir)
{
	tLineDir.dDirX = tXiLineDir.dDirX;
	tLineDir.dDirY = tXiLineDir.dDirY;
	tLineDir.dDirZ = tXiLineDir.dDirZ;
}

void RTT::AdjustTrack::TransLineDirToXiLineDir(T_LINE_DIR_3D tLineDir, T_XIGEO_LINE_DIR& tXiLineDir)
{
	tXiLineDir.dDirX = tLineDir.dDirX;
	tXiLineDir.dDirY = tLineDir.dDirY;
	tXiLineDir.dDirZ = tLineDir.dDirZ;
}

void RTT::AdjustTrack::CalBevelAdjustDirForTracking(std::vector<T_POINT_3D> vtBevelPoints, std::vector<vector<T_POINT_3D>> vvtInciseEntityWithBevelPoints, std::vector<vector<T_LINE_DIR>> vvtInciseEntityPointAdjustDir, std::vector<T_LINE_DIR>& vtBevelAdjustDir)
{
	vtBevelAdjustDir.clear();
	std::vector<T_POINT_3D> vtIncisePoints;
	std::vector<T_LINE_DIR> vtInciseAdjustLineDir;
	vtIncisePoints.clear();
	vtInciseAdjustLineDir.clear();

	for (int nEntityNo = 0; nEntityNo < vvtInciseEntityWithBevelPoints.size(); nEntityNo++)
	{
		for (int nPointNo = 0; nPointNo < vvtInciseEntityWithBevelPoints[nEntityNo].size(); nPointNo++)
		{
			vtIncisePoints.push_back(vvtInciseEntityWithBevelPoints[nEntityNo][nPointNo]);
			vtInciseAdjustLineDir.push_back(vvtInciseEntityPointAdjustDir[nEntityNo][nPointNo]);
		}
	}

	for (int nPointNo = 0; nPointNo < vtBevelPoints.size(); nPointNo++)
	{
		int nNearNo = FindNearestPoint(vtBevelPoints[nPointNo], vtIncisePoints);
		vtBevelAdjustDir.push_back(vtInciseAdjustLineDir[nNearNo]);
	}
}

RTT::AdjustTrack::T_DOTRACKING_PARA* RTT::AdjustTrack::FindCurSeenEntities(T_ROBOT_COORS tImgCoord)
{
	//互斥拷贝一份数据
	std::vector<T_POINT_3D> vtTeachRobotWeldPoints;
	{
		CRITICAL_AUTO_LOCK(m_cLock);
		vtTeachRobotWeldPoints = m_vtTeachRobotWeldPoints;
	}

	// 在m_vtBevelPoints中查找采图时最近的点的索引
	T_POINT_3D tPoint;
	tPoint.x = tImgCoord.dX + tImgCoord.dBX;
	tPoint.y = tImgCoord.dY + tImgCoord.dBY;
	tPoint.z = tImgCoord.dZ + tImgCoord.dBZ;

	auto time = XI_clock();
	int nIndex = FindNearestPoint(tPoint, vtTeachRobotWeldPoints);

	writeLog("距离采图位置最近的切割轨迹点索引：%d, 看到的实体号：%d，检索用时：%dms",
		nIndex, m_vtDoTrackPara[nIndex].nEntityNo, XI_clock() - time);

	return &(m_vtDoTrackPara[nIndex]);
}

bool RTT::AdjustTrack::JudgeToDoAdjust(T_ALGORITHM_POINT tSeenPos,
	T_DOTRACKING_PARA tSeenEntity, T_ALGORITHM_POINT& tFilterOutPoint, bool& bOutValid)
{
	// 根据采图时的坐标 判断是否看得见 看到的是哪个实体 看到的是直线还是圆弧 
	// 1、看得见才使用看见的数据
	// 2、每个实体使用单独的一个滤波输入集合(两个连接的直线也是两个实体，滤波也需要分开)
	// 3、看到的是圆弧还是直线 滤波参数不同
	if (false == tSeenEntity.bIfDoTracking
		|| tSeenEntity.nEntityNo < 0) // 不能跟踪
	{
		writeLog("当前看到的位置无法跟踪");
		bOutValid = false;
		return false;
	}

	// 添加到滤波输入
	//double dHandEyeDis = 70.0; // 滤波输入最多输入多长的数据进行滤波
	//double dFilterInputPointSpaceing = 1.0;
	std::vector<T_ALGORITHM_POINT>& vtFilterInput = m_mnvFilterInput[tSeenEntity.nEntityNo];
	int nSrcFilterInputNum = vtFilterInput.size();
	if (0 == nSrcFilterInputNum) // 第一个点直接添加
	{
		vtFilterInput.push_back(tSeenPos);
	}
	else
	{
		T_ALGORITHM_POINT tLastPoint = vtFilterInput[nSrcFilterInputNum - 1];
		double dNewPointToLastPointDis = TwoPointDis(
			tLastPoint.dCoorX, tLastPoint.dCoorY, tLastPoint.dCoorZ,
			tSeenPos.dCoorX, tSeenPos.dCoorY, tSeenPos.dCoorZ);
		if (dNewPointToLastPointDis > m_dMinFilterInputPointSpacing)
		{
			vtFilterInput.push_back(tSeenPos);
		}
		else
		{
			writeLog("滤波输入新点与上一个输入点间距小于%.3lf mm", m_dMinFilterInputPointSpacing);
			bOutValid = false;
			return false;
		}
	}

	// 删除多余的点 从最新看到的位置往回 只保留一个手眼的距离作为滤波输入
	int nInputNum = vtFilterInput.size();
	int nDelIndex;
	for (nDelIndex = 0; nDelIndex < nInputNum; nDelIndex++)
	{
		double dDis = TwoPointDis(vtFilterInput[nDelIndex].dCoorX, vtFilterInput[nDelIndex].dCoorY, vtFilterInput[nDelIndex].dCoorZ,
			vtFilterInput[nInputNum - 1].dCoorX, vtFilterInput[nInputNum - 1].dCoorY, vtFilterInput[nInputNum - 1].dCoorZ);
		if (dDis < m_dMaxFilterInputPointDis/*dHandEyeDis*/)
		{
			break;
		}
	}
	nDelIndex -= 1;
	writeLog("开始删除 size %d", vtFilterInput.size());
	if (nDelIndex >= 0)
	{
		vtFilterInput.erase(vtFilterInput.begin(), vtFilterInput.begin() + nDelIndex);
	}
	writeLog("删除结束 size %d", vtFilterInput.size());

	// 判断当前滤波输入集合点数是否可以进行滤波
	if (m_nMinFilterInputPointNum > vtFilterInput.size()) // 输入点太少 不滤波
	{
		writeLog("实时跟踪：数据太少不能滤波");
		bOutValid = false;
		return false;
	}

	// 执行滤波
	std::vector<T_ALGORITHM_POINT> vtOutput;
	E_FIT_TYPE eFitType = (1 == tSeenEntity.nEntityType ? E_LINE : E_CIRCLE);
	m_cXiAlgorithm.PointSmooth(vtFilterInput, eFitType/*E_LINE*/, 0.8, vtOutput);
	tFilterOutPoint = vtOutput[vtOutput.size() - 1];
	bOutValid = true;
	m_nFilterOutValidNum++;

	// 滤波输出点中每几个点调整一次
	return 0 == (m_nFilterOutValidNum % m_nPntCountBetweenEachModification);
}

void RTT::AdjustTrack::AdjustBevelDataByTeachPointsForTrackingNew(std::vector<vector<T_POINT_3D>> vvtInciseEntityWithBevelPoints, std::vector<int> vnInciseEntityWithBevelNo, std::vector<vector<T_LINE_DIR>> vvtInciseEntityPointAdjustDir, std::vector<double> vdInciseToBevelDis,
	const std::vector<T_DOTRACKING_PARA>& vtIfDoTracking, std::vector<T_POINT_3D> vtBevelPointsWithoutAdjust, std::vector<T_LINE_DIR> vtBevelAdjustDir, std::vector<T_POINT_3D> vtBevelPoints, T_POINT_3D tCurrentGunPoint,
	T_POINT_3D tCurrentBevelCutPoint, T_TEACH_PARA tTeachParaPoints, std::vector<T_POINT_3D>& vtAdjustPoints)
{
	vtAdjustPoints.clear();
	double dDis = 0.0;
	double dMinDis = 999999.0;
	int nMinNo = 0;
	int nEntityNo = 0;
	int nAdjustNumOnce = 60; // 30;
	T_POINT_3D tBevelPoint;
	bool bIfAdjust = false;

	for (int nInciseEntityNo = 0; nInciseEntityNo < vnInciseEntityWithBevelNo.size(); nInciseEntityNo++)
	{
		if (vnInciseEntityWithBevelNo[nInciseEntityNo] == tTeachParaPoints.nTeachEntityNo)
		{
			for (int nPointNo = 0; nPointNo < vvtInciseEntityWithBevelPoints[nInciseEntityNo].size(); nPointNo++)
			{
				dDis = m_cAnalyzeObj.CalDisPointToPoint(tTeachParaPoints.tTeachPoint.x, tTeachParaPoints.tTeachPoint.y,
					vvtInciseEntityWithBevelPoints[nInciseEntityNo][nPointNo].x, vvtInciseEntityWithBevelPoints[nInciseEntityNo][nPointNo].y);

				if (dDis < dMinDis)
				{
					bIfAdjust = true;
					dMinDis = dDis;
					nMinNo = nPointNo;
					nEntityNo = nInciseEntityNo;
				}
			}
		}
	}

	if (bIfAdjust == false)
	{
		vtAdjustPoints = vtBevelPoints;
		return;
	}

	tBevelPoint.x = tTeachParaPoints.tTeachPoint.x + vdInciseToBevelDis[nEntityNo] * vvtInciseEntityPointAdjustDir[nEntityNo][nMinNo].dDirX;
	tBevelPoint.y = tTeachParaPoints.tTeachPoint.y + vdInciseToBevelDis[nEntityNo] * vvtInciseEntityPointAdjustDir[nEntityNo][nMinNo].dDirY;
	tBevelPoint.z = tTeachParaPoints.tTeachPoint.z;

	int nAdjustNo, nCurrentNo, nAdjustEndNo;

	GetAdjustInfo(tCurrentGunPoint, tCurrentBevelCutPoint, tBevelPoint, vtIfDoTracking, vtBevelPoints, nAdjustNumOnce, nCurrentNo, nAdjustNo, nAdjustEndNo);

	if (nCurrentNo >= nAdjustNo || nAdjustNo >= nAdjustEndNo)
	{
		vtAdjustPoints = vtBevelPoints;
		return;
	}
	m_nLastAdjustStartPntNo = nCurrentNo;
	m_nLastAdjustEndPntNo = nAdjustEndNo;
	int nIdealNearestPointNo = FindNearestPoint(tBevelPoint, vtBevelPointsWithoutAdjust);
	//int nIdealNearestPointNo = FindNearestPointNew(tBevelPoint, vtBevelPointsWithoutAdjust, m_vnBevelEntityNo, vnInciseEntityWithBevelNo[nEntityNo]);

	double dXAdjust = 0.0;
	double dYAdjust = 0.0;
	double dZAdjust = 0.0;
	double dAdjustDis = 0.0;

	if (nAdjustNo < vtBevelPoints.size())
	{
		dXAdjust = tBevelPoint.x - vtBevelPoints[nAdjustNo].x;
		dYAdjust = tBevelPoint.y - vtBevelPoints[nAdjustNo].y;
		dZAdjust = tBevelPoint.z - vtBevelPoints[nAdjustNo].z;

		T_POINT_2D_DOUBLE tPoint{};
		T_POINT_2D_DOUBLE tLineStartPoint{}, tLineEndPoint{};
		tPoint.x = tBevelPoint.x;
		tPoint.y = tBevelPoint.y;

		tLineStartPoint.x = vtBevelPoints[nAdjustNo - 1].x;
		tLineStartPoint.y = vtBevelPoints[nAdjustNo - 1].y;

		tLineEndPoint.x = vtBevelPoints[nAdjustNo].x;
		tLineEndPoint.y = vtBevelPoints[nAdjustNo].y;

		double dAdjustAngle = VectorsInnerAngle(dXAdjust, dYAdjust, dZAdjust, vtBevelAdjustDir[nAdjustNo].dDirX, vtBevelAdjustDir[nAdjustNo].dDirY, 0.0);

		if (dAdjustAngle > 90)
		{
			//dAdjustDis = -CalDisPointToPoint(tBevelPoint.x, tBevelPoint.y, tBevelPoint.z, vtBevelPoints[nAdjustNo].x, vtBevelPoints[nAdjustNo].y, vtBevelPoints[nAdjustNo].z);
			dAdjustDis = -m_cAnalyzeObj.CalDisPointToLine(tPoint, tLineStartPoint, tLineEndPoint);
		}
		else
		{
			//dAdjustDis = CalDisPointToPoint(tBevelPoint.x, tBevelPoint.y, tBevelPoint.z, vtBevelPoints[nAdjustNo].x, vtBevelPoints[nAdjustNo].y, vtBevelPoints[nAdjustNo].z);
			dAdjustDis = m_cAnalyzeObj.CalDisPointToLine(tPoint, tLineStartPoint, tLineEndPoint);
		}
		double dAdjustDisZ = fabs(tBevelPoint.z - vtBevelPoints[nAdjustNo].z);
		if (fabs(dAdjustDis) > m_dMaxAdjustDisXY || dAdjustDisZ > m_dMaxAdjustDisZ)
		{
			writeLog("超出调整阈值，%.3lf，%.3lf", dAdjustDis, dAdjustDisZ);
			dXAdjust = 0.0;
			dYAdjust = 0.0;
			dZAdjust = 0.0;
			dAdjustDis = 0.0;
		}
		else
		{
			dXAdjust = dAdjustDis * vtBevelAdjustDir[nAdjustNo].dDirX;
			dYAdjust = dAdjustDis * vtBevelAdjustDir[nAdjustNo].dDirY;
			dZAdjust = dZAdjust;
		}
	}
	else
	{
		dXAdjust = 0.0;
		dYAdjust = 0.0;
		dZAdjust = 0.0;
		dAdjustDis = 0.0;
	}

	T_POINT_3D tBevelAdjustPoint;
	T_POINT_3D tTempPoint;
	double dTempAdjustX = 0.0, dTempAdjustY = 0.0, dTempAdjustZ = 0.0;

	if (tTeachParaPoints.nTeachEntityType == 1)
	{
		for (int nBevelNo = 0; nBevelNo < vtBevelPoints.size(); nBevelNo++)
		{
			if (nBevelNo < nCurrentNo)
			{
				vtAdjustPoints.push_back(vtBevelPoints[nBevelNo]);
			}
			else if (nBevelNo >= nCurrentNo && nBevelNo <= nAdjustNo)
			{
				// 				tBevelAdjustPoint.x = vtBevelPoints[nCurrentNo].x + (nBevelNo - nCurrentNo)*(vtBevelPoints[nAdjustNo].x - vtBevelPoints[nCurrentNo].x + dXAdjust) / (nAdjustNo - nCurrentNo);
				// 				tBevelAdjustPoint.y = vtBevelPoints[nCurrentNo].y + (nBevelNo - nCurrentNo)*(vtBevelPoints[nAdjustNo].y - vtBevelPoints[nCurrentNo].y + dYAdjust) / (nAdjustNo - nCurrentNo);
				// 				tBevelAdjustPoint.z = vtBevelPoints[nCurrentNo].z + (nBevelNo - nCurrentNo)*(vtBevelPoints[nAdjustNo].z - vtBevelPoints[nCurrentNo].z + dZAdjust) / (nAdjustNo - nCurrentNo);

				double dStartXError = vtBevelPoints[nCurrentNo].x - vtBevelPointsWithoutAdjust[nCurrentNo].x;
				double dStartYError = vtBevelPoints[nCurrentNo].y - vtBevelPointsWithoutAdjust[nCurrentNo].y;
				double dStartZError = vtBevelPoints[nCurrentNo].z - vtBevelPointsWithoutAdjust[nCurrentNo].z;
				double dEndXError = vtBevelPoints[nAdjustNo].x - vtBevelPointsWithoutAdjust[nAdjustNo].x;
				double dEndYError = vtBevelPoints[nAdjustNo].y - vtBevelPointsWithoutAdjust[nAdjustNo].y;
				double dEndZError = vtBevelPoints[nAdjustNo].z - vtBevelPointsWithoutAdjust[nAdjustNo].z;

				tBevelAdjustPoint.x = vtBevelPointsWithoutAdjust[nBevelNo].x + dStartXError + (nBevelNo - nCurrentNo) * (dEndXError - dStartXError) / (nAdjustNo - nCurrentNo) + (nBevelNo - nCurrentNo) * dXAdjust / (nAdjustNo - nCurrentNo);
				tBevelAdjustPoint.y = vtBevelPointsWithoutAdjust[nBevelNo].y + dStartYError + (nBevelNo - nCurrentNo) * (dEndYError - dStartYError) / (nAdjustNo - nCurrentNo) + (nBevelNo - nCurrentNo) * dYAdjust / (nAdjustNo - nCurrentNo);
				tBevelAdjustPoint.z = vtBevelPointsWithoutAdjust[nBevelNo].z + dStartZError + (nBevelNo - nCurrentNo) * (dEndZError - dStartZError) / (nAdjustNo - nCurrentNo) + (nBevelNo - nCurrentNo) * dZAdjust / (nAdjustNo - nCurrentNo);

				vtAdjustPoints.push_back(tBevelAdjustPoint);
			}
			else
			{
				if (nAdjustNo < vtBevelPoints.size() - 1 && nIdealNearestPointNo < vtBevelPoints.size() - 1)
				{
					tTempPoint.x = vtBevelPoints[nAdjustNo].x + dXAdjust + (vtBevelPointsWithoutAdjust[nIdealNearestPointNo + 1].x - vtBevelPointsWithoutAdjust[nIdealNearestPointNo].x);
					tTempPoint.y = vtBevelPoints[nAdjustNo].y + dYAdjust + (vtBevelPointsWithoutAdjust[nIdealNearestPointNo + 1].y - vtBevelPointsWithoutAdjust[nIdealNearestPointNo].y);
					tTempPoint.z = vtBevelPoints[nAdjustNo].z + dZAdjust + (vtBevelPointsWithoutAdjust[nIdealNearestPointNo + 1].z - vtBevelPointsWithoutAdjust[nIdealNearestPointNo].z);

					dTempAdjustX = tTempPoint.x - vtBevelPointsWithoutAdjust[nIdealNearestPointNo + 1].x;
					dTempAdjustY = tTempPoint.y - vtBevelPointsWithoutAdjust[nIdealNearestPointNo + 1].y;
					dTempAdjustZ = tTempPoint.z - vtBevelPointsWithoutAdjust[nIdealNearestPointNo + 1].z;
				}
				else
				{
					tTempPoint.x = vtBevelPoints[vtBevelPoints.size() - 2].x + dXAdjust + (vtBevelPointsWithoutAdjust[vtBevelPoints.size() - 1].x - vtBevelPointsWithoutAdjust[vtBevelPoints.size() - 2].x);
					tTempPoint.y = vtBevelPoints[vtBevelPoints.size() - 2].y + dYAdjust + (vtBevelPointsWithoutAdjust[vtBevelPoints.size() - 1].y - vtBevelPointsWithoutAdjust[vtBevelPoints.size() - 2].y);
					tTempPoint.z = vtBevelPoints[vtBevelPoints.size() - 2].z + dZAdjust + (vtBevelPointsWithoutAdjust[vtBevelPoints.size() - 1].z - vtBevelPointsWithoutAdjust[vtBevelPoints.size() - 2].z);

					dTempAdjustX = tTempPoint.x - vtBevelPointsWithoutAdjust[vtBevelPoints.size() - 1].x;
					dTempAdjustY = tTempPoint.y - vtBevelPointsWithoutAdjust[vtBevelPoints.size() - 1].y;
					dTempAdjustZ = tTempPoint.z - vtBevelPointsWithoutAdjust[vtBevelPoints.size() - 1].z;
				}

				if (nBevelNo <= nAdjustEndNo)
				{
					if (nIdealNearestPointNo + nBevelNo - nAdjustNo < vtBevelPoints.size())
					{
						tBevelAdjustPoint.x = vtBevelPointsWithoutAdjust[nIdealNearestPointNo + nBevelNo - nAdjustNo].x + dTempAdjustX;
						tBevelAdjustPoint.y = vtBevelPointsWithoutAdjust[nIdealNearestPointNo + nBevelNo - nAdjustNo].y + dTempAdjustY;
						tBevelAdjustPoint.z = vtBevelPointsWithoutAdjust[nIdealNearestPointNo + nBevelNo - nAdjustNo].z + dTempAdjustZ;
						vtAdjustPoints.push_back(tBevelAdjustPoint);
					}
					else
					{
						tBevelAdjustPoint.x = vtBevelPointsWithoutAdjust[vtBevelPointsWithoutAdjust.size() - 1].x + dTempAdjustX;
						tBevelAdjustPoint.y = vtBevelPointsWithoutAdjust[vtBevelPointsWithoutAdjust.size() - 1].y + dTempAdjustY;
						tBevelAdjustPoint.z = vtBevelPointsWithoutAdjust[vtBevelPointsWithoutAdjust.size() - 1].z + dTempAdjustZ;
						vtAdjustPoints.push_back(tBevelAdjustPoint);
					}
				}
				else
				{
					vtAdjustPoints.push_back(vtBevelPoints[nBevelNo]);
				}
			}
		}
	}
	else if (tTeachParaPoints.nTeachEntityType == 2)
	{
		for (int nBevelNo = 0; nBevelNo < vtBevelPoints.size(); nBevelNo++)
		{
			if (nBevelNo < nCurrentNo)
			{
				vtAdjustPoints.push_back(vtBevelPoints[nBevelNo]);
			}
			else if (nBevelNo >= nCurrentNo && nBevelNo <= nAdjustNo)
			{
				double dStartXError = vtBevelPoints[nCurrentNo].x - vtBevelPointsWithoutAdjust[nCurrentNo].x;
				double dStartYError = vtBevelPoints[nCurrentNo].y - vtBevelPointsWithoutAdjust[nCurrentNo].y;
				double dStartZError = vtBevelPoints[nCurrentNo].z - vtBevelPointsWithoutAdjust[nCurrentNo].z;
				double dEndXError = vtBevelPoints[nAdjustNo].x - vtBevelPointsWithoutAdjust[nAdjustNo].x;
				double dEndYError = vtBevelPoints[nAdjustNo].y - vtBevelPointsWithoutAdjust[nAdjustNo].y;
				double dEndZError = vtBevelPoints[nAdjustNo].z - vtBevelPointsWithoutAdjust[nAdjustNo].z;

				tBevelAdjustPoint.x = vtBevelPointsWithoutAdjust[nBevelNo].x + dStartXError + (nBevelNo - nCurrentNo) * (dEndXError - dStartXError) / (nAdjustNo - nCurrentNo) + (nBevelNo - nCurrentNo) * dXAdjust / (nAdjustNo - nCurrentNo);
				tBevelAdjustPoint.y = vtBevelPointsWithoutAdjust[nBevelNo].y + dStartYError + (nBevelNo - nCurrentNo) * (dEndYError - dStartYError) / (nAdjustNo - nCurrentNo) + (nBevelNo - nCurrentNo) * dYAdjust / (nAdjustNo - nCurrentNo);
				tBevelAdjustPoint.z = vtBevelPointsWithoutAdjust[nBevelNo].z + dStartZError + (nBevelNo - nCurrentNo) * (dEndZError - dStartZError) / (nAdjustNo - nCurrentNo) + (nBevelNo - nCurrentNo) * dZAdjust / (nAdjustNo - nCurrentNo);

				vtAdjustPoints.push_back(tBevelAdjustPoint);
			}
			else
			{
				if (nAdjustNo < vtBevelPoints.size() - 1 && nIdealNearestPointNo < vtBevelPoints.size() - 1)
				{
					tTempPoint.x = vtBevelPoints[nAdjustNo].x + dXAdjust + (vtBevelPointsWithoutAdjust[nIdealNearestPointNo + 1].x - vtBevelPointsWithoutAdjust[nIdealNearestPointNo].x);
					tTempPoint.y = vtBevelPoints[nAdjustNo].y + dYAdjust + (vtBevelPointsWithoutAdjust[nIdealNearestPointNo + 1].y - vtBevelPointsWithoutAdjust[nIdealNearestPointNo].y);
					tTempPoint.z = vtBevelPoints[nAdjustNo].z + dZAdjust + (vtBevelPointsWithoutAdjust[nIdealNearestPointNo + 1].z - vtBevelPointsWithoutAdjust[nIdealNearestPointNo].z);

					dTempAdjustX = tTempPoint.x - vtBevelPointsWithoutAdjust[nIdealNearestPointNo + 1].x;
					dTempAdjustY = tTempPoint.y - vtBevelPointsWithoutAdjust[nIdealNearestPointNo + 1].y;
					dTempAdjustZ = tTempPoint.z - vtBevelPointsWithoutAdjust[nIdealNearestPointNo + 1].z;
				}
				else
				{
					tTempPoint.x = vtBevelPoints[vtBevelPoints.size() - 2].x + dXAdjust + (vtBevelPointsWithoutAdjust[vtBevelPoints.size() - 1].x - vtBevelPointsWithoutAdjust[vtBevelPoints.size() - 2].x);
					tTempPoint.y = vtBevelPoints[vtBevelPoints.size() - 2].y + dYAdjust + (vtBevelPointsWithoutAdjust[vtBevelPoints.size() - 1].y - vtBevelPointsWithoutAdjust[vtBevelPoints.size() - 2].y);
					tTempPoint.z = vtBevelPoints[vtBevelPoints.size() - 2].z + dZAdjust + (vtBevelPointsWithoutAdjust[vtBevelPoints.size() - 1].z - vtBevelPointsWithoutAdjust[vtBevelPoints.size() - 2].z);

					dTempAdjustX = tTempPoint.x - vtBevelPointsWithoutAdjust[vtBevelPoints.size() - 1].x;
					dTempAdjustY = tTempPoint.y - vtBevelPointsWithoutAdjust[vtBevelPoints.size() - 1].y;
					dTempAdjustZ = tTempPoint.z - vtBevelPointsWithoutAdjust[vtBevelPoints.size() - 1].z;
				}

				if (nBevelNo <= nAdjustEndNo)
				{
					if (nIdealNearestPointNo + nBevelNo - nAdjustNo < vtBevelPoints.size())
					{
						tBevelAdjustPoint.x = vtBevelPointsWithoutAdjust[nIdealNearestPointNo + nBevelNo - nAdjustNo].x + dTempAdjustX;
						tBevelAdjustPoint.y = vtBevelPointsWithoutAdjust[nIdealNearestPointNo + nBevelNo - nAdjustNo].y + dTempAdjustY;
						tBevelAdjustPoint.z = vtBevelPointsWithoutAdjust[nIdealNearestPointNo + nBevelNo - nAdjustNo].z + dTempAdjustZ;
						vtAdjustPoints.push_back(tBevelAdjustPoint);
					}
					else
					{
						tBevelAdjustPoint.x = vtBevelPointsWithoutAdjust[vtBevelPointsWithoutAdjust.size() - 1].x + dTempAdjustX;
						tBevelAdjustPoint.y = vtBevelPointsWithoutAdjust[vtBevelPointsWithoutAdjust.size() - 1].y + dTempAdjustY;
						tBevelAdjustPoint.z = vtBevelPointsWithoutAdjust[vtBevelPointsWithoutAdjust.size() - 1].z + dTempAdjustZ;
						vtAdjustPoints.push_back(tBevelAdjustPoint);
					}
				}
				else
				{
					vtAdjustPoints.push_back(vtBevelPoints[nBevelNo]);
				}
			}
		}
	}
	else
	{
		for (int nBevelNo = 0; nBevelNo < vtBevelPoints.size(); nBevelNo++)
		{
			vtAdjustPoints.push_back(vtBevelPoints[nBevelNo]);
		}
	}
}

void RTT::AdjustTrack::GetAdjustInfo(T_POINT_3D tCurrentGunPoint, T_POINT_3D tCurrentBevelCutPoint, T_POINT_3D tCurrentBevelMeasurePoint, const std::vector<T_DOTRACKING_PARA>& vtIfDoTracking, std::vector<T_POINT_3D> vtBevelPoints, int nAdjustNumOnce, int& nCurrentCutNo, int& nCurrentMeasureNo, int& nAdjustEndNo)
{
	int nCurrentGunNo = FindNearestPoint(tCurrentGunPoint, vtBevelPoints);
	nCurrentMeasureNo = FindNearestPoint(tCurrentBevelMeasurePoint, vtBevelPoints);
	nCurrentCutNo = FindNearestPoint(tCurrentBevelCutPoint, vtBevelPoints);
	nAdjustEndNo = GetAdjustEndNoNew(nCurrentGunNo, nCurrentMeasureNo, nAdjustNumOnce, vtIfDoTracking);

	if (vtIfDoTracking.size() > 120 && nAdjustEndNo > vtIfDoTracking.size() - 120)
	{
		nAdjustEndNo = vtIfDoTracking.size() - 1;
	}
}

int RTT::AdjustTrack::GetAdjustEndNoNew(int nCurrentCutNo, int nCurrentMesureNo, int nAdjustNumOnce, const std::vector<T_DOTRACKING_PARA>& vtIfDoTracking)
{
	int nAdjustSum = 0;

	if (nCurrentMesureNo + nAdjustNumOnce < vtIfDoTracking.size())
	{
		for (int nPointNo = nCurrentCutNo; nPointNo < vtIfDoTracking.size(); nPointNo++)
		{
			nAdjustSum = 0;
			if (nPointNo + (nCurrentMesureNo - nCurrentCutNo) + nAdjustNumOnce < vtIfDoTracking.size())
			{
				for (int nAdjustNo = 0; nAdjustNo < nAdjustNumOnce; nAdjustNo++)
				{
					if (nPointNo + nAdjustNo >= vtIfDoTracking.size())
					{
						return vtIfDoTracking.size() - 1;
					}
					nAdjustSum += (int)vtIfDoTracking[nPointNo + nAdjustNo].bIfDoTracking;
				}

				if (nAdjustSum == nAdjustNumOnce)
				{
					return  nPointNo + (nCurrentMesureNo - nCurrentCutNo) + nAdjustNumOnce;
				}
			}
			else
			{
				return vtIfDoTracking.size() - 1;
			}
		}
	}
	else
	{
		return vtIfDoTracking.size() - 1;
	}
	return nCurrentCutNo;
}

bool RTT::AdjustTrack::savePlaneContourInfo()
{
	m_bOnlyLineEntity = true;
	auto file = fopen(getDataFileRootFolder() + "PlaneContourInfo.txt", "w");
	for (size_t i = 0; i < m_nEntityCount; i++)
	{
		//计算偏振
		CvPoint3D32f temp{};
		temp.x = (m_ptPlaneContourInfo[i].staPoint.x + m_ptPlaneContourInfo[i].endPoint.x) / 2.0;
		temp.y = (m_ptPlaneContourInfo[i].staPoint.y + m_ptPlaneContourInfo[i].endPoint.y) / 2.0;
		temp.z = (m_ptPlaneContourInfo[i].staPoint.z + m_ptPlaneContourInfo[i].endPoint.z) / 2.0;

		if (m_ptPlaneContourInfo[i].CounterType == 0)
		{
			temp.x = m_ptPlaneContourInfo[i].midPoint.x - temp.x;
			temp.y = m_ptPlaneContourInfo[i].midPoint.y - temp.y;
			temp.z = m_ptPlaneContourInfo[i].midPoint.z - temp.z;
			m_bOnlyLineEntity = false;
		}
		else
		{
			temp.x = 0.0;
			temp.y = 0.0;
			temp.z = 0.0;
		}

		fprintf(file, "%.6f %.6f %.6f %.6f %.6f %.6f\n",
			m_ptPlaneContourInfo[i].staPoint.x, m_ptPlaneContourInfo[i].staPoint.y,
			m_ptPlaneContourInfo[i].endPoint.x, m_ptPlaneContourInfo[i].endPoint.y,
			temp.x, temp.y);
	}
	::fclose(file);
	return true;
}

void RTT::AdjustTrack::GetCameraIfDoTrackingNew(T_XIGEO_PLANE_PARA tContourPlanePara, std::vector<T_DXF_ENTITY_PARA> vtContourEntities, std::vector<T_POINT_3D> vtBevelPoints, std::vector<int> vnEntityNo, std::vector<T_ROBOT_COORS_IN_WORLD> vtCutRobotCoors, E_CAM_ID eCamId, std::vector<T_DOTRACKING_PARA>& vtIfDoTracking, std::vector<T_POINT_2D>& vtTrackingPointInImage)
{
	T_XIGEO_PLANE_PARA tPlanePara;
	T_POINT_3D tIntersectionPoint;
	T_POINT_3D tBasePoint;
	double dDis = 0.0;
	double dAngle = 0.0;
	T_XIGEO_LINE_PARA tIntersectionLine;
	T_POINT_3D tLineStartPoint;
	T_POINT_3D tLineEndPoint;
	double dDisLimit = m_dDisLimitForCalDoTracking;			// 当前激光点 和 图像中心激光点 距离差 mm
	double dAngleLimit = m_dAngleLimitForCalDoTracking;// 45.0;
	T_DOTRACKING_PARA tDoTrackingPara;
	BOOL nIfDo = FALSE;
	int nStartNo = 0;

	auto tCameraPara = m_pUnit->GetCameraParam(m_pUnit->m_nTrackCameraNo);
	int nImageWidth = tCameraPara.tDHCameraDriverPara.nRoiWidth;
	int nImageHeight = tCameraPara.tDHCameraDriverPara.nRoiHeight;
	T_POINT_2D tImagePoint;
	tImagePoint.x = -1;
	tImagePoint.y = -1;

	for (int nPointNo = 0; nPointNo < vtCutRobotCoors.size(); nPointNo++)
	{
		tPlanePara = CalcLaserPlate(vtCutRobotCoors[nPointNo], eCamId);
		tBasePoint = P2P<T_POINT_3D>(tPlanePara.tPoint);

		nIfDo = FALSE;
		tDoTrackingPara.bIfDoTracking = FALSE;
		tDoTrackingPara.nEntityNo = -1;
		tDoTrackingPara.nEntityType = -1;
		tDoTrackingPara.tIntersectionPoint.x = 0.0;
		tDoTrackingPara.tIntersectionPoint.y = 0.0;
		tDoTrackingPara.tIntersectionPoint.z = 0.0;


		for (int nEntityNo = 0; nEntityNo < vtContourEntities.size(); nEntityNo++)
		{
			if (GetEntityAndPlaneInterseciton(vtContourEntities[nEntityNo], tContourPlanePara, tBasePoint, tPlanePara, tIntersectionPoint))
			{
				dDis = m_cAnalyzeObj.CalDisPointToPoint(tIntersectionPoint.x, tIntersectionPoint.y, tIntersectionPoint.z, tBasePoint.x, tBasePoint.y, tBasePoint.z);

				GetTwoPlaneIntersectionLine(tContourPlanePara, tPlanePara, tIntersectionLine);
				tLineStartPoint.x = tIntersectionLine.tPoint.x;
				tLineStartPoint.y = tIntersectionLine.tPoint.y;
				tLineStartPoint.z = tIntersectionLine.tPoint.z;

				tLineEndPoint.x = tIntersectionLine.tPoint.x + 20.0 * tIntersectionLine.tLineDir.dDirX;
				tLineEndPoint.y = tIntersectionLine.tPoint.y + 20.0 * tIntersectionLine.tLineDir.dDirY;
				tLineEndPoint.z = tIntersectionLine.tPoint.z + 20.0 * tIntersectionLine.tLineDir.dDirZ;

				dAngle = CalAngleEntityAndLine(tIntersectionPoint, vtContourEntities[nEntityNo], tLineStartPoint, tLineEndPoint);

				if (dDis <= dDisLimit && fabs(90 - fabs(dAngle)) < dAngleLimit)
				{
					int nNearestNo = FindNearestPoint(tIntersectionPoint, vtBevelPoints);

					if (vnEntityNo[nNearestNo] == -1)
					{
						tDoTrackingPara.bIfDoTracking = FALSE;
						tDoTrackingPara.nEntityNo = vtContourEntities[nEntityNo].nContourEntityNo;
						tDoTrackingPara.tIntersectionPoint = tIntersectionPoint;
						if (vtContourEntities[nEntityNo].eEntityType == E_DXF_ENTITY_TYPE_LINE)
						{
							tDoTrackingPara.nEntityType = 1;
						}
						else if (vtContourEntities[nEntityNo].eEntityType == E_DXF_ENTITY_TYPE_ARC_ANTICLOCKWISE || vtContourEntities[nEntityNo].eEntityType == E_DXF_ENTITY_TYPE_ARC_CLOCKWISE)
						{
							tDoTrackingPara.nEntityType = 2;
						}
						else
						{
							tDoTrackingPara.nEntityType = -1;
						}
						break;
					}
					else
					{
						double dEntityLength = m_cAnalyzeObj.GetEntitiesLength(vtContourEntities[nEntityNo]);

						if (dEntityLength < 60)
						{
							tDoTrackingPara.bIfDoTracking = FALSE;
							tDoTrackingPara.nEntityNo = vtContourEntities[nEntityNo].nContourEntityNo;
							tDoTrackingPara.tIntersectionPoint = tIntersectionPoint;
							if (vtContourEntities[nEntityNo].eEntityType == E_DXF_ENTITY_TYPE_LINE)
							{
								tDoTrackingPara.nEntityType = 1;
							}
							else if (vtContourEntities[nEntityNo].eEntityType == E_DXF_ENTITY_TYPE_ARC_ANTICLOCKWISE || vtContourEntities[nEntityNo].eEntityType == E_DXF_ENTITY_TYPE_ARC_CLOCKWISE)
							{
								tDoTrackingPara.nEntityType = 2;
							}
							else
							{
								tDoTrackingPara.nEntityType = -1;
							}
							break;
						}
						else
						{
							nIfDo = TRUE;
							tDoTrackingPara.bIfDoTracking = TRUE;
							tDoTrackingPara.nEntityNo = vtContourEntities[nEntityNo].nContourEntityNo;
							tDoTrackingPara.tIntersectionPoint = tIntersectionPoint;
							if (vtContourEntities[nEntityNo].eEntityType == E_DXF_ENTITY_TYPE_LINE)
							{
								tDoTrackingPara.nEntityType = 1;
							}
							else if (vtContourEntities[nEntityNo].eEntityType == E_DXF_ENTITY_TYPE_ARC_ANTICLOCKWISE || vtContourEntities[nEntityNo].eEntityType == E_DXF_ENTITY_TYPE_ARC_CLOCKWISE)
							{
								tDoTrackingPara.nEntityType = 2;
							}
							else
							{
								tDoTrackingPara.nEntityType = -1;
							}

							tImagePoint = GetIntersectionPointInImage(vtCutRobotCoors[nPointNo], tIntersectionPoint, nImageWidth, nImageHeight, eCamId);
							break;
						}
					}
				}
			}
		}
		vtIfDoTracking.push_back(tDoTrackingPara);
		vtTrackingPointInImage.push_back(tImagePoint);
	}
}

T_POINT_2D RTT::AdjustTrack::GetIntersectionPointInImage(T_ROBOT_COORS_IN_WORLD tCutPoint, T_POINT_3D tIntersectionPoint, int nImageWidth, int nImageHeight, E_CAM_ID eCamId)
{
	int nRobotId = eCamId / 4;
	int nCamId = eCamId % 4;
	double dMatrixCameraToTool[4][4];
	T_POINT_2D tImagePoint;

	auto tCameraPara = m_pUnit->GetCameraParam(m_pUnit->m_nTrackCameraNo);

	T_ROBOT_COORS tCameraReadyRobotCoors = tCameraPara.tHandEyeCaliPara.tCameraReadyRobotCoors;
	double dXMoveToGun = tCameraPara.tHandEyeCaliPara.tGunMoveToCameraCenterRobotCoors.dX - tCameraPara.tHandEyeCaliPara.tCameraReadyRobotCoors.dX;
	double dYMoveToGun = tCameraPara.tHandEyeCaliPara.tGunMoveToCameraCenterRobotCoors.dY - tCameraPara.tHandEyeCaliPara.tCameraReadyRobotCoors.dY;
	double dZMoveToGun = tCameraPara.tHandEyeCaliPara.tGunMoveToCameraCenterRobotCoors.dZ - tCameraPara.tHandEyeCaliPara.tCameraReadyRobotCoors.dZ;

	double dCameraDx = tCameraPara.tHandEyeCaliPara.tCameraInnerPara.dPixelX;
	double dCameraDy = tCameraPara.tHandEyeCaliPara.tCameraInnerPara.dPixelY;
	double dCameraFocal = tCameraPara.tHandEyeCaliPara.tCameraInnerPara.dFocal;
	double dCameraBaseLineLength = tCameraPara.tHandEyeCaliPara.tCameraInnerPara.dBaseLineLength;
	double dCameraBaseLineCtan = tCameraPara.tHandEyeCaliPara.tCameraInnerPara.dCtanBaseLineAngle;

	m_cXiRobotAbsCoorsTrans->m_pCamDirInRobotBase[nRobotId][nCamId][0] = tCameraPara.tHandEyeCaliPara.nCameraXAxisInRobotCoordinate;
	m_cXiRobotAbsCoorsTrans->m_pCamDirInRobotBase[nRobotId][nCamId][1] = tCameraPara.tHandEyeCaliPara.nCameraYAxisInRobotCoordinate;
	m_cXiRobotAbsCoorsTrans->m_pCamDirInRobotBase[nRobotId][nCamId][2] = tCameraPara.tHandEyeCaliPara.nCameraZAxisInRobotCoordinate;

	m_cXiRobotAbsCoorsTrans->GenerateCameraToToolMatrix(tCameraReadyRobotCoors, dXMoveToGun, dYMoveToGun, dZMoveToGun, dMatrixCameraToTool, eCamId);

	double dGunMatrixInBase[4][4];
	m_cXiRobotAbsCoorsTrans->GenerateToolToBaseMatrix(tCutPoint.dX, tCutPoint.dY, tCutPoint.dZ, tCutPoint.dRX, tCutPoint.dRY, tCutPoint.dRZ, dGunMatrixInBase);

	double dCameraMatrixInBase[4][4];
	int nRows = 4;
	int nCols = 4;
	m_cXiRobotAbsCoorsTrans->Trmul(dGunMatrixInBase, nRows, nCols, dMatrixCameraToTool, nRows, nCols, dCameraMatrixInBase, nRows, nCols);

	double dMatrixBaseToCamera[4][4];
	m_cXiRobotAbsCoorsTrans->InverseTransMatrix(dCameraMatrixInBase, dMatrixBaseToCamera);

	double dPointInBaseMatrix[4][4];
	dPointInBaseMatrix[0][0] = 1.0; dPointInBaseMatrix[0][1] = 0.0; dPointInBaseMatrix[0][2] = 0.0; dPointInBaseMatrix[0][3] = tIntersectionPoint.x;
	dPointInBaseMatrix[1][0] = 0.0; dPointInBaseMatrix[1][1] = 1.0; dPointInBaseMatrix[1][2] = 0.0; dPointInBaseMatrix[1][3] = tIntersectionPoint.y;
	dPointInBaseMatrix[2][0] = 0.0; dPointInBaseMatrix[2][1] = 0.0; dPointInBaseMatrix[2][2] = 1.0; dPointInBaseMatrix[2][3] = tIntersectionPoint.z;
	dPointInBaseMatrix[3][0] = 0.0; dPointInBaseMatrix[3][1] = 0.0; dPointInBaseMatrix[3][2] = 0.0; dPointInBaseMatrix[3][3] = 1.0;

	double dPointInCameraMatrix[4][4];
	m_cXiRobotAbsCoorsTrans->Trmul(dMatrixBaseToCamera, nRows, nCols, dPointInBaseMatrix, nRows, nCols, dPointInCameraMatrix, nRows, nCols);

	T_POINT_3D tPointInCamera;
	tPointInCamera.x = dPointInCameraMatrix[0][3];
	tPointInCamera.y = dPointInCameraMatrix[1][3];
	tPointInCamera.z = dPointInCameraMatrix[2][3] + fabs(dCameraBaseLineLength / dCameraBaseLineCtan);

	double dXScale = tPointInCamera.z / fabs(dCameraFocal);
	double dYScale = tPointInCamera.z / fabs(dCameraFocal);

	tImagePoint.x = (tPointInCamera.x / dXScale) / dCameraDx + nImageWidth / 2.0;
	tImagePoint.y = (tPointInCamera.y / dYScale) / dCameraDy + nImageHeight / 2.0;

	return tImagePoint;
}

void RTT::AdjustTrack::GetCameraIfDoTrackingNew(T_XIGEO_PLANE_PARA tContourPlanePara, std::vector<T_DXF_ENTITY_PARA> vtContourEntities, std::vector<T_POINT_3D> vtBevelPoints, std::vector<int> vnEntityNo, std::vector<T_ROBOT_COORS_IN_WORLD> vtCutRobotCoors, E_CAM_ID eCamId, std::vector<T_DOTRACKING_PARA>& vtIfDoTracking, std::vector<T_POINT_2D>& vtTrackingPointInImage, std::vector<T_POINT_2D>& vtHorTrackingPointInImage, std::vector<T_POINT_2D>& vtVerTrackingPointInImage)
{

	vtIfDoTracking.clear();
	vtTrackingPointInImage.clear();
	vtHorTrackingPointInImage.clear();
	vtVerTrackingPointInImage.clear();

	T_XIGEO_PLANE_PARA tPlanePara;
	T_POINT_3D tIntersectionPoint;
	T_POINT_3D tBasePoint;
	double dDis = 0.0;
	double dAngle = 0.0;

	T_XIGEO_LINE_PARA tIntersectionLine;
	T_POINT_3D tLineStartPoint;
	T_POINT_3D tLineEndPoint;
	T_POINT_3D tHorLineEndPoint;
	T_POINT_3D tVerLineEndPoint;

	double dDisLimit = m_dDisLimitForCalDoTracking;			// 当前激光点 和 图像中心激光点 距离差 mm
	double dAngleLimit = m_dAngleLimitForCalDoTracking;// 45.0;
	T_DOTRACKING_PARA tDoTrackingPara;
	BOOL nIfDo = FALSE;
	int nStartNo = 0;

	auto tCameraPara = m_pUnit->GetCameraParam(m_pUnit->m_nTrackCameraNo);
	int nImageWidth = tCameraPara.tDHCameraDriverPara.nRoiWidth;
	int nImageHeight = tCameraPara.tDHCameraDriverPara.nRoiHeight;
	T_POINT_2D tImagePoint;
	tImagePoint.x = -1;
	tImagePoint.y = -1;

	T_POINT_2D tHorPoint;
	tHorPoint.x = -1;
	tHorPoint.y = -1;

	T_POINT_2D tVerPoint;
	tVerPoint.x = -1;
	tVerPoint.y = -1;

	for (int nPointNo = 0; nPointNo < vtCutRobotCoors.size(); nPointNo++)
	{
		tPlanePara = CalcLaserPlate(vtCutRobotCoors[nPointNo], eCamId);
		tBasePoint = P2P<T_POINT_3D>(tPlanePara.tPoint);

		nIfDo = FALSE;
		tDoTrackingPara.bIfDoTracking = FALSE;
		tDoTrackingPara.nEntityNo = -1;
		tDoTrackingPara.nEntityType = -1;
		tDoTrackingPara.tIntersectionPoint.x = 0.0;
		tDoTrackingPara.tIntersectionPoint.y = 0.0;
		tDoTrackingPara.tIntersectionPoint.z = 0.0;

		for (int nEntityNo = 0; nEntityNo < vtContourEntities.size(); nEntityNo++)
		{
			if (GetEntityAndPlaneInterseciton(vtContourEntities[nEntityNo], tContourPlanePara, tBasePoint, tPlanePara, tIntersectionPoint))
			{
				dDis = m_cAnalyzeObj.CalDisPointToPoint(tIntersectionPoint.x, tIntersectionPoint.y, tIntersectionPoint.z, tBasePoint.x, tBasePoint.y, tBasePoint.z);

				GetTwoPlaneIntersectionLine(tContourPlanePara, tPlanePara, tIntersectionLine);
				tLineStartPoint.x = tIntersectionLine.tPoint.x;
				tLineStartPoint.y = tIntersectionLine.tPoint.y;
				tLineStartPoint.z = tIntersectionLine.tPoint.z;

				double dLineDirLength = TwoPointDis(tIntersectionLine.tLineDir.dDirX, tIntersectionLine.tLineDir.dDirY,
					tIntersectionLine.tLineDir.dDirZ, 0.0, 0.0, 0.0);
				//tIntersectionLine.tLineDir.dDirX /= dDis;
				//tIntersectionLine.tLineDir.dDirY /= dDis;
				//tIntersectionLine.tLineDir.dDirZ /= dDis;

				tLineEndPoint.x = tIntersectionLine.tPoint.x + 5.0 * tIntersectionLine.tLineDir.dDirX;
				tLineEndPoint.y = tIntersectionLine.tPoint.y + 5.0 * tIntersectionLine.tLineDir.dDirY;
				tLineEndPoint.z = tIntersectionLine.tPoint.z + 5.0 * tIntersectionLine.tLineDir.dDirZ;

				tHorLineEndPoint.x = tIntersectionPoint.x - 10.0 * tIntersectionLine.tLineDir.dDirX / dLineDirLength;
				tHorLineEndPoint.y = tIntersectionPoint.y - 10.0 * tIntersectionLine.tLineDir.dDirY / dLineDirLength;
				tHorLineEndPoint.z = tIntersectionPoint.z - 10.0 * tIntersectionLine.tLineDir.dDirZ / dLineDirLength;

				dAngle = CalAngleEntityAndLine(tIntersectionPoint, vtContourEntities[nEntityNo], tLineStartPoint, tLineEndPoint);

				if (dDis <= dDisLimit && fabs(90 - fabs(dAngle)) < dAngleLimit)
				{
					int nNearestNo = FindNearestPoint(tIntersectionPoint, vtBevelPoints);

					if (vnEntityNo[nNearestNo] == -1)
					{
						tDoTrackingPara.bIfDoTracking = FALSE;
						tDoTrackingPara.nEntityNo = vtContourEntities[nEntityNo].nContourEntityNo;
						tDoTrackingPara.tIntersectionPoint = tIntersectionPoint;
						if (vtContourEntities[nEntityNo].eEntityType == E_DXF_ENTITY_TYPE_LINE)
						{
							tDoTrackingPara.nEntityType = 1;
						}
						else if (vtContourEntities[nEntityNo].eEntityType == E_DXF_ENTITY_TYPE_ARC_ANTICLOCKWISE || vtContourEntities[nEntityNo].eEntityType == E_DXF_ENTITY_TYPE_ARC_CLOCKWISE)
						{
							tDoTrackingPara.nEntityType = 2;
						}
						else
						{
							tDoTrackingPara.nEntityType = -1;
						}
						break;
					}
					else
					{
						double dEntityLength = m_cAnalyzeObj.GetEntitiesLength(vtContourEntities[nEntityNo]);

						if (dEntityLength < 60)
						{
							tDoTrackingPara.bIfDoTracking = FALSE;
							tDoTrackingPara.nEntityNo = vtContourEntities[nEntityNo].nContourEntityNo;
							tDoTrackingPara.tIntersectionPoint = tIntersectionPoint;
							if (vtContourEntities[nEntityNo].eEntityType == E_DXF_ENTITY_TYPE_LINE)
							{
								tDoTrackingPara.nEntityType = 1;
							}
							else if (vtContourEntities[nEntityNo].eEntityType == E_DXF_ENTITY_TYPE_ARC_ANTICLOCKWISE || vtContourEntities[nEntityNo].eEntityType == E_DXF_ENTITY_TYPE_ARC_CLOCKWISE)
							{
								tDoTrackingPara.nEntityType = 2;
							}
							else
							{
								tDoTrackingPara.nEntityType = -1;
							}
							break;
						}
						else
						{
							nIfDo = TRUE;
							tDoTrackingPara.bIfDoTracking = TRUE;
							tDoTrackingPara.nEntityNo = vtContourEntities[nEntityNo].nContourEntityNo;
							tDoTrackingPara.tIntersectionPoint = tIntersectionPoint;
							if (vtContourEntities[nEntityNo].eEntityType == E_DXF_ENTITY_TYPE_LINE)
							{
								tDoTrackingPara.nEntityType = 1;
							}
							else if (vtContourEntities[nEntityNo].eEntityType == E_DXF_ENTITY_TYPE_ARC_ANTICLOCKWISE || vtContourEntities[nEntityNo].eEntityType == E_DXF_ENTITY_TYPE_ARC_CLOCKWISE)
							{
								tDoTrackingPara.nEntityType = 2;
							}
							else
							{
								tDoTrackingPara.nEntityType = -1;
							}

							////计算激光图像参数
							T_LINE_DIR_3D tVerPlaneVerDir;
							T_LINE_DIR_3D tFirstLineDir, tSecondLineDir;

							tSecondLineDir.dDirX = tContourPlanePara.tPlateVerDir.dDirX;
							tSecondLineDir.dDirY = tContourPlanePara.tPlateVerDir.dDirY;
							tSecondLineDir.dDirZ = tContourPlanePara.tPlateVerDir.dDirZ;

							if (nPointNo != vtCutRobotCoors.size() - 1 && vtCutRobotCoors.size() >= 2)
							{
								tFirstLineDir = m_cAnalyzeObj.CalLineDir(vtCutRobotCoors[nPointNo].dX, vtCutRobotCoors[nPointNo].dY, vtCutRobotCoors[nPointNo].dZ, vtCutRobotCoors[nPointNo + 1].dX, vtCutRobotCoors[nPointNo + 1].dY, vtCutRobotCoors[nPointNo + 1].dZ);
								tVerPlaneVerDir = m_cAnalyzeObj.CrossProduct(tFirstLineDir, tSecondLineDir);
							}
							else if (nPointNo == vtCutRobotCoors.size() - 1 && vtCutRobotCoors.size() >= 2)
							{
								tFirstLineDir = m_cAnalyzeObj.CalLineDir(vtCutRobotCoors[nPointNo].dX, vtCutRobotCoors[nPointNo].dY, vtCutRobotCoors[nPointNo].dZ, vtCutRobotCoors[nPointNo - 1].dX, vtCutRobotCoors[nPointNo - 1].dY, vtCutRobotCoors[nPointNo - 1].dZ);
								tVerPlaneVerDir = m_cAnalyzeObj.CrossProduct(tFirstLineDir, tSecondLineDir);
							}
							else
							{
								tVerPlaneVerDir.dDirX = -tFirstLineDir.dDirY;
								tVerPlaneVerDir.dDirY = tFirstLineDir.dDirX;
								tVerPlaneVerDir.dDirZ = 0.0;
							}

							T_XIGEO_PLANE_PARA tVerPlanePara;
							tVerPlanePara.tPlateVerDir.dDirX = tVerPlaneVerDir.dDirX;
							tVerPlanePara.tPlateVerDir.dDirY = tVerPlaneVerDir.dDirY;
							tVerPlanePara.tPlateVerDir.dDirZ = tVerPlaneVerDir.dDirZ;
							tVerPlanePara.tPoint.x = tIntersectionPoint.x;
							tVerPlanePara.tPoint.y = tIntersectionPoint.y;
							tVerPlanePara.tPoint.z = tIntersectionPoint.z;

							GetTwoPlaneIntersectionLine(tVerPlanePara, tPlanePara, tIntersectionLine);

							double dVerLineDirLength = TwoPointDis(tIntersectionLine.tLineDir.dDirX, tIntersectionLine.tLineDir.dDirY,
								tIntersectionLine.tLineDir.dDirZ, 0.0, 0.0, 0.0);
							tVerLineEndPoint.x = tIntersectionPoint.x + 10.0 * tIntersectionLine.tLineDir.dDirX / dVerLineDirLength;
							tVerLineEndPoint.y = tIntersectionPoint.y + 10.0 * tIntersectionLine.tLineDir.dDirY / dVerLineDirLength;
							tVerLineEndPoint.z = tIntersectionPoint.z + 10.0 * tIntersectionLine.tLineDir.dDirZ / dVerLineDirLength;

							tHorPoint = GetIntersectionPointInImage(vtCutRobotCoors[nPointNo], tHorLineEndPoint, nImageWidth, nImageHeight, eCamId);
							tVerPoint = GetIntersectionPointInImage(vtCutRobotCoors[nPointNo], tVerLineEndPoint, nImageWidth, nImageHeight, eCamId);
							tImagePoint = GetIntersectionPointInImage(vtCutRobotCoors[nPointNo], tIntersectionPoint, nImageWidth, nImageHeight, eCamId);
							
							//激光倒V型固定方向，交点的像素Y最小
							if (tImagePoint.y > tHorPoint.y)
							{
								tHorPoint.x = tImagePoint.x + (tImagePoint.x - tHorPoint.x);
								tHorPoint.y = tImagePoint.y + (tImagePoint.y - tHorPoint.y);
							}
							if (tImagePoint.y > tVerPoint.y)
							{
								tVerPoint.x = tImagePoint.x + (tImagePoint.x - tVerPoint.x);
								tVerPoint.y = tImagePoint.y + (tImagePoint.y - tVerPoint.y);
							}
							
							break;
						}
					}
				}
			}
		}
		vtIfDoTracking.push_back(tDoTrackingPara);
		vtTrackingPointInImage.push_back(tImagePoint);
		vtHorTrackingPointInImage.push_back(tHorPoint);
		vtVerTrackingPointInImage.push_back(tVerPoint);
	}
}

