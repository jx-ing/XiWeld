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
	E_DOTRACKING_CONTOUR_TYPE eDoTrackingContourType = E_OUTTERCONTOUR_TRACKING;	// ��������
	E_DOTRACKING_BEVEL_TYPE eDoTrackingBevelType = E_BEVEL_UP_TRACKING;			// �����¿�


	DELETE_POINTER_ARRAY(m_ptPlaneContourInfo);
	m_ptPlaneContourInfo = new PlaneContourInfo[100];

	//���ʵ��
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

	/*if (IDOK != XiMessageBox("ȷ�Ϲ켣��ȷ�ٵ���˵���"))
	{
		return false;
	}*/

	//���ں��ӣ���Ϊ���������¿������غϵ�
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

	// �����������
	CalBevelAdjustDirForTracking(m_vtBevelPoints, m_vvtInciseEntityWithBevelPoints, m_vvtInciseEntityPointAdjustDir, m_vtBevelAdjustDir);

	// ��ʼ���˲��������� (ÿ��ʵ��Ŷ�Ӧһ���˲����뼯�ϣ�ÿ�����ϵĳ�ʼʱΪ��)
	m_mnvFilterInput.clear();
	for (int i = 0; i < m_vtDoTrackPara.size(); i++)
	{
		if (m_vtDoTrackPara[i].nEntityNo >= 0)
		{
			m_mnvFilterInput[m_vtDoTrackPara[i].nEntityNo].clear();
			writeLog("�Ƿ�ɸ��٣�%4d %d %d %d", i,
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
	// ͼƬ�Ͽ�������ʲôʵ��
	T_DOTRACKING_PARA tSeenEntity = *FindCurSeenEntities(tImgCoord);

	//�ж��Ƿ����������˲����
	T_ALGORITHM_POINT tFilterOutPoint;//�˲��󷵻ص�����
	bool bFilterOutValid = false;//�˲��ɹ�/ʧ��
	T_ALGORITHM_POINT tSeenPos{};
	tSeenPos.dCoorX = tSeenCoord.dX + tSeenCoord.dBX;
	tSeenPos.dCoorY = tSeenCoord.dY + tSeenCoord.dBY;
	tSeenPos.dCoorZ = tSeenCoord.dZ + tSeenCoord.dBZ;
	bool bNeedCalcAdjsut = JudgeToDoAdjust(tSeenPos, tSeenEntity, tFilterOutPoint, bFilterOutValid);

	//���������˳�
	if (!bFilterOutValid || !bNeedCalcAdjsut)
	{
		return false;
	}

	// ���µĸ�������
	T_TEACH_PARA tTeachPara;
	tTeachPara.tTeachPoint.x = tFilterOutPoint.dCoorX;
	tTeachPara.tTeachPoint.y = tFilterOutPoint.dCoorY;
	tTeachPara.tTeachPoint.z = tFilterOutPoint.dCoorZ;
	tTeachPara.nTeachEntityNo = tSeenEntity.nEntityNo;
	tTeachPara.nTeachEntityType = tSeenEntity.nEntityType;
	tTeachPara.nTeachEntityOrder = -1;

	//�����켣
	auto tLastSendPnt = getLastSendPnt();//�ϴη��͵�����
	std::vector<T_POINT_3D>	vtNewRobotWeldPoints;//��������¿ڹ켣
	T_POINT_3D tCapRobotPoint3D;
	tCapRobotPoint3D.x = tImgCoord.dX + tImgCoord.dBX;
	tCapRobotPoint3D.y = tImgCoord.dY + tImgCoord.dBY;
	tCapRobotPoint3D.z = tImgCoord.dZ + tImgCoord.dBZ;
	writeLog("�����켣�������룺%.3lf %.3lf %.3lf %.3lf %.3lf %.3lf ʵ�壺%d %d %d ���㣺%.3lf %.3lf %.3lf",
		tLastSendPnt.x, tLastSendPnt.y, tLastSendPnt.z,
		tTeachPara.tTeachPoint.x, tTeachPara.tTeachPoint.y, tTeachPara.tTeachPoint.z,
		tTeachPara.nTeachEntityNo, tTeachPara.nTeachEntityType, tTeachPara.nTeachEntityOrder,
		tCapRobotPoint3D.x, tCapRobotPoint3D.y, tCapRobotPoint3D.z);
	auto llStartCalTime = XI_clock();
	AdjustBevelDataByTeachPointsForTrackingNew(
		m_vvtInciseEntityWithBevelPoints, m_vnInciseEntityWithBevelNo, m_vvtInciseEntityPointAdjustDir, m_vdInciseToBevelDis, m_vtDoTrackPara,
		m_vtOrgRobotWeldPoints, m_vtBevelAdjustDir, m_vtTeachRobotWeldPoints, tCapRobotPoint3D,
		tLastSendPnt, tTeachPara, vtNewRobotWeldPoints);
	writeLog("�����켣�����ʱ��%dms", XI_clock() - llStartCalTime);

	//�޸ĵ�����Ĺ켣
	if (!m_pWeldTrack->adjustTrack(vtNewRobotWeldPoints, m_nLastAdjustStartPntNo, m_nLastAdjustEndPntNo))
	{
		writeLog("�����켣ʧ��");
		return false;
	}
	m_pWeldTrack->saveTrack(getDataFileRootFolder() + "AdjustTrack//LastestAdjustTrack.txt");	
	m_nAdjustTimes++;
	saveAdjustTrack(vtNewRobotWeldPoints);
	{
		CRITICAL_AUTO_LOCK(m_cLock);
		m_vtTeachRobotWeldPoints = vtNewRobotWeldPoints;
	}
	writeLog("�����켣�ɹ�%d-%d", m_nLastAdjustStartPntNo, m_nLastAdjustEndPntNo);
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
	//���⿽��һ������
	std::vector<T_POINT_3D> vtTeachRobotWeldPoints;
	{
		CRITICAL_AUTO_LOCK(m_cLock);
		vtTeachRobotWeldPoints = m_vtTeachRobotWeldPoints;
	}

	//�ҳ������
	T_POINT_3D tPoint;
	tPoint.x = tSeenCoord.dX + tSeenCoord.dBX;
	tPoint.y = tSeenCoord.dY + tSeenCoord.dBY;
	tPoint.z = tSeenCoord.dZ + tSeenCoord.dBZ;
	int nIndex = FindNearestPoint(tPoint, vtTeachRobotWeldPoints);

	//ǰ��������Ϊ�͵������㷨����ͬ
	if (nIndex < 2)
	{
		nIndex = 2;
	}

	//�����������Ϊ�͵����������㷨����ͬ
	if (nIndex >= vtTeachRobotWeldPoints.size() - 2)
	{
		nIndex = vtTeachRobotWeldPoints.size() - 3;
	}

	//����켣����
	double dDirX = vtTeachRobotWeldPoints[nIndex + 1].x - vtTeachRobotWeldPoints[nIndex - 1].x
		+ vtTeachRobotWeldPoints[nIndex + 2].x - vtTeachRobotWeldPoints[nIndex - 2].x;
	double dDirY = vtTeachRobotWeldPoints[nIndex + 1].y - vtTeachRobotWeldPoints[nIndex - 1].y
		+ vtTeachRobotWeldPoints[nIndex + 2].y - vtTeachRobotWeldPoints[nIndex - 2].y;

	//���㷨���
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

	m_pRobot->m_cLog->Write("���ٵ�����" + str);
}

bool RTT::AdjustTrack::calcTrackEntity()
{
	//����ɹ켣
	m_pWeldTrack->saveTrack(getDataFileRootFolder() + "InputTrack.txt");
	m_pWeldTrack->saveTrackWorld(getDataFileRootFolder() + "InputTrackWorld.txt");

	//������������ϵ�¹켣
	std::vector<CvPoint3D32f> vtCloudPoint = m_pWeldTrack->toWorld<CvPoint3D32f>();

	//�ָ�Ϊʵ��
	m_nEntityCount = ContourCloudClassify(vtCloudPoint.data(), vtCloudPoint.size(), m_ptPlaneContourInfo, "ContourCloudClassify");
	if (m_nEntityCount <= 0)
	{
		XiMessageBox("���ӹ켣�ָ��쳣");
		return false;
	}

	//�����������ʵ������
	PlaneContourInfo** groupInfo = new PlaneContourInfo * [vtCloudPoint.size()];
	if (!ContourCloudGrouping(m_ptPlaneContourInfo, m_nEntityCount, vtCloudPoint.data(), groupInfo, vtCloudPoint.size(), "ContourCloudGrouping"))
	{
		XiMessageBox("���ӹ켣�ָ�����쳣");
		return false;
	}
	m_nStartPntEntityNo = groupInfo[0] - m_ptPlaneContourInfo;

	//�����һ��ʵ�����յ㷴�ˣ�����һ��
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

	//������ʵ����������
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

	//����ʵ��
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
		if(IDOK != XiMessageBox("�¸��٣�Բ����������ʧ��, �Ƿ����"));
			return false;
	}*/


	if ((m_pWeldTrack->isClosed() && !freeedgeofworkpiece.correctContour(2))
		|| (!m_pWeldTrack->isClosed() && !freeedgeofworkpiece.correctUnclosedContour(2)))
	//if (!freeedgeofworkpiece.correctContour(2))
	//if (!freeedgeofworkpiece.correctUnclosedContour(2))
	{
		XiMessageBox("�¸��٣�����������");
		return false;
	}


	freeedgeofworkpiece.perform();
	if (freeedgeofworkpiece.isDone()) 
	{
		writeLog("makeEntitiesClose����ɹ�");
		return true;
	}
	XiMessageBox("makeEntitiesClose����ʧ��");
	return false;
}

bool RTT::AdjustTrack::makesureStratPnt()
{
	//���
	T_ROBOT_COORS tStartCoord = m_pWeldTrack->getPoint(0);
	CvPoint3D64f tStartPnt{};
	tStartPnt.x = tStartCoord.dX + tStartCoord.dBX;
	tStartPnt.y = tStartCoord.dY + tStartCoord.dBY;
	tStartPnt.z = tStartCoord.dZ + tStartCoord.dBZ;

	//��ȡ�������ʵ�����ɢ�����ݣ����ҳ������������ĵ�
	int nNearestPntNo = -1;
	double dMinDis = 99999.0;
	std::vector<CvPoint3D32f> vtPnts;
	int nIndex = 0;
	CvPoint3D32f tPnt{};
	CString sFileName;
	sFileName.Format("���//�ɶ�%zd.txt", m_nStartPntEntityNo);
	auto file = fopen(getDataFileRootFolder() + sFileName, "r");
	while (EOF != fscanf(file, "%d%f%f%f",
		&nIndex, &tPnt.x, &tPnt.y, &tPnt.z))
	{
		vtPnts.push_back(tPnt);

		double dTemp = square(tStartPnt.x - tPnt.x)
			+ square(tStartPnt.y - tPnt.y);

		//С����ֵ��˵����ǰ�����
		if (dTemp < dMinDis)
		{
			dMinDis = dTemp;
			nNearestPntNo = nIndex;
		}
	}
	fclose(file);
	if (nNearestPntNo < 0)
	{
		XiMessageBox("�¸��٣��Ҳ�����������ĺ������");
		return false;
	}

	//��ʵ�����ݻ��浽����
	std::vector<PlaneContourInfo> vtPlaneContourInfo(0);
	for (int i = 0; i < m_nEntityCount; i++)
	{
		vtPlaneContourInfo.push_back(m_ptPlaneContourInfo[i]);
	}

	//�������̫�����������ʵ������ӦΪ0
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
	//�����յ�̫�����������ʵ������ӦΪ���һ��
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
	//�����������Ҫ��ϵ�ǰʵ�壬�ٶ�ʵ����������
	else
	{
		//�Ƚ�δ���ָ��ʵ�����¸�ֵ
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

		//�޸ĵ�һ��ʵ��
		m_ptPlaneContourInfo[0] = vtPlaneContourInfo[m_nStartPntEntityNo];
		m_ptPlaneContourInfo[0].staPoint = vtPnts[nNearestPntNo];
		m_ptPlaneContourInfo[0].midPoint = vtPnts[(vtPnts.size() - nNearestPntNo) / 2];
		m_ptPlaneContourInfo[0].endPoint = vtPnts.back();

		//�޸����һ��ʵ��
		m_ptPlaneContourInfo[m_nEntityCount] = vtPlaneContourInfo[m_nStartPntEntityNo];
		m_ptPlaneContourInfo[m_nEntityCount].staPoint = vtPnts[0];
		m_ptPlaneContourInfo[m_nEntityCount].midPoint = vtPnts[nNearestPntNo / 2];
		m_ptPlaneContourInfo[m_nEntityCount].endPoint = vtPnts[nNearestPntNo];

		//ʵ����Ŀ����
		m_nEntityCount++;
	}

	//������һ��ʵ����ǰһ��ʵ�岻��������ɾ��
	if (m_nEntityCount > 1
		&& 20.0 < TwoPointDis(m_ptPlaneContourInfo[m_nEntityCount - 1].staPoint.x, m_ptPlaneContourInfo[m_nEntityCount - 1].staPoint.y,
			m_ptPlaneContourInfo[m_nEntityCount - 2].endPoint.x, m_ptPlaneContourInfo[m_nEntityCount - 2].endPoint.y))
	{
		//m_nEntityCount--;
		writeLog("���һ��ʵ����ǰһ��ʵ�岻��������ɾ��");
	}

	//�����µ�ʵ�����ݣ�����ʹʵ��պ�
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
	auto file = fopen(getDataFileRootFolder() + "���//GrayscaleResult_Entity2D.txt", "r");
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
		//�ҳ������������ĵ�
		int nNearestPntNo = -1;
		double dMinDis = 99999.0;
		for (size_t j = 0; j < vtOld3DPnts.size(); j++)
		{
			double dTemp = square(m_vtContourEntities[i].dStartX - vtOld3DPnts[j].x)
				+ square(m_vtContourEntities[i].dStartY - vtOld3DPnts[j].y);

			//С����ֵ��˵����ǰ�����
			if (dTemp < dMinDis)
			{
				dMinDis = dTemp;
				nNearestPntNo = j;
			}
		}
		if (nNearestPntNo < 0)
		{
			XiMessageBox("�¸��٣��Ҳ�����������ĺ���ʵ��");
			return false;
		}
		m_vtContourEntities[i].dStartZ = vtOld3DPnts[nNearestPntNo].z;

		//�ҳ������յ�����ĵ�
		nNearestPntNo = -1;
		dMinDis = 99999.0;
		for (size_t j = 0; j < vtOld3DPnts.size(); j++)
		{
			double dTemp = square(m_vtContourEntities[i].dEndX - vtOld3DPnts[j].x)
				+ square(m_vtContourEntities[i].dEndY - vtOld3DPnts[j].y);

			//С����ֵ��˵����ǰ�����
			if (dTemp < dMinDis)
			{
				dMinDis = dTemp;
				nNearestPntNo = j;
			}
		}
		if (nNearestPntNo < 0)
		{
			XiMessageBox("�¸��٣��Ҳ�����������ĺ���ʵ��");
			return false;
		}
		m_vtContourEntities[i].dEndZ = vtOld3DPnts[nNearestPntNo].z;

		//�м��ֱ�Ӱ���ֵ
		m_vtContourEntities[i].dCenterZ = (m_vtContourEntities[i].dStartZ + m_vtContourEntities[i].dEndZ) / 2.0;
	}

	return true;
}

bool RTT::AdjustTrack::loadClosedTrack(const std::vector<T_ROBOT_COORS>& vtOldCoord, const std::vector<CvPoint3D64f>& vtOld3DPnts)
{
	//��ȡ�¹켣
	m_vtPartOutPoints.clear();
	m_vnPartOutEntityNo.clear();
	std::vector<int> vnPntCount{};//ÿ��ʵ�弰��֮ǰʵ��ĵ���֮��
	for (size_t i = 0; i < m_vtContourEntities.size(); i++)
	{
		int nPartOutPointsCountBefore = m_vnPartOutEntityNo.size();

		//��ȡ����ʵ�����ɢ��
		T_POINT_3D tPnt{ 0 };//��ά��
		int nIndex = 0;//���
		CString sFileName;//�ļ���
		sFileName.Format("���//%zd.txt", i);
		auto file = fopen(getDataFileRootFolder() + sFileName, "r");
		while (EOF != fscanf(file, "%d%lf%lf%lf",
			&nIndex, &tPnt.x, &tPnt.y, &tPnt.z))
		{
			m_vtPartOutPoints.push_back(tPnt);//������ά��
			m_vnPartOutEntityNo.push_back(i);//����ʵ�����
		}
		::fclose(file);

		//ȥ���������һ��ʵ��֮�������ʵ������һ����ɢ��
		if (i < m_vtContourEntities.size() - 1)
		{
			m_vtPartOutPoints.pop_back();
			m_vnPartOutEntityNo.pop_back();
		}

		vnPntCount.push_back(m_vtPartOutPoints.size() - nPartOutPointsCountBefore);
	}

	//����������ת��������������ϵ�£�����Z��ֵ
	auto llTime = XiBase::XI_clock();
	std::vector<T_ROBOT_COORS> vtInitWeldCoord(m_vtPartOutPoints.size());
	for (size_t i = 0; i < m_vtPartOutPoints.size(); i++)
	{
		//�ҳ�����ԭʼ�켣����ĵ�
		int nNearestPntNo = -1;
		double dMinDis = 99999.0;
		for (size_t j = 0; j < vtOld3DPnts.size(); j++)
		{
			auto& tAimCoord = m_vtPartOutPoints[i];
			double dTemp = square(tAimCoord.x - vtOld3DPnts[j].x)
				+ square(tAimCoord.y - vtOld3DPnts[j].y);

			//С�ڵ�����ֵ��˵����ǰ�����
			if (dTemp <= dMinDis)
			{
				dMinDis = dTemp;
				nNearestPntNo = j;
			}
		}
		if (nNearestPntNo < 0)
		{
			XiMessageBox("�¸��٣��Ҳ�����������ĳ�ʼ����������");
			return false;
		}

		//z��ֵ
		m_vtPartOutPoints[i].z = vtOld3DPnts[nNearestPntNo].z + vtOldCoord[nNearestPntNo].dBZ;

		//ת��������������ϵ��
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
	writeLog("�����ʼ�켣��Zֵ��ʱ��%dms", XiBase::XI_clock() - llTime);


	if (m_bEnableChangeRZ && !changeRZForArc(vnPntCount, vtInitWeldCoord))
	{
		XiMessageBox("�ı�Բ��������̬ʧ��");
		return false;
	}

	//�������RZ�仯̫��
	for (size_t i = 1; i < vtInitWeldCoord.size(); i++)
	{
		double dDeltaRZ = fabs(vtInitWeldCoord[i].dRZ - vtInitWeldCoord[i - 1].dRZ);
		if (dDeltaRZ > 15.0 && dDeltaRZ < 355.0)
		{
			XiMessageBox("�¸��٣����۹켣��RZ�仯̫��");
			return false;
		}
	}

	//���ù켣
	m_pWeldTrack->initTrack(vtInitWeldCoord);

#ifdef LOCAL_DEBUG
	return true;
#endif // LOCAL_DEBUG

	//�ӵ�ǰλ�ó����Ƿ�������������켣
	return isSafeTrack(m_pRobot->GetCurrentPulse(), vtInitWeldCoord);
}

bool RTT::AdjustTrack::changeRZForArc(const std::vector<int>& vnPntCount, std::vector<T_ROBOT_COORS>& vtInitWeldCoord)
{
	int nMaxCountOfChangedLine = 40;//ֱ�߶θı�RZ�����켣����
	double dChangeRZOfEachLinePnt = 0.5;//ÿ��ֱ�߹켣��ı�RZ��ֵ

	//����ʵ�壬�ҵ�Բ����ֱ��������λ��
	bool bFindNewArc = false;
	for (size_t i = 1; i < m_vtContourEntities.size(); i++)
	{
		//��һ�κ���һ����ֱ�ߣ�������һ����Բ��
		if (i < m_vtContourEntities.size() - 1
			&& m_vtContourEntities[i - 1].eEntityType == E_DXF_ENTITY_TYPE_LINE
			&& m_vtContourEntities[i + 1].eEntityType == E_DXF_ENTITY_TYPE_LINE
			&& (m_vtContourEntities[i].eEntityType == E_DXF_ENTITY_TYPE_ARC_ANTICLOCKWISE
				|| m_vtContourEntities[i].eEntityType == E_DXF_ENTITY_TYPE_ARC_CLOCKWISE))
		{
			//ֱ�߶�1��ǰ����̬����
			int nLineCount1 = vnPntCount[i - 1] > nMaxCountOfChangedLine ? nMaxCountOfChangedLine : vnPntCount[i - 1];
			//ֱ�߶�2��ǰ����̬����
			int nLineCount2 = vnPntCount[i + 1] > nMaxCountOfChangedLine ? nMaxCountOfChangedLine : vnPntCount[i + 1];
			//��ֱ֤�߶α���̬������ͬ
			nLineCount1 = nLineCount1 > nLineCount2 ? nLineCount2 : nLineCount1;
			nLineCount2 = nLineCount2 > nLineCount1 ? nLineCount1 : nLineCount2;
			//ֱ�߶�1��ǰ����̬ÿһ�ݽǶ�ֵ
			double dChangeRZOnLine1 =
				m_vtContourEntities[i].eEntityType == E_DXF_ENTITY_TYPE_ARC_ANTICLOCKWISE ?
				dChangeRZOfEachLinePnt : -dChangeRZOfEachLinePnt;
			//ֱ�߶�2��ǰ����̬ÿһ�ݽǶ�ֵ
			double dChangeRZOnLine2 =
				m_vtContourEntities[i].eEntityType == E_DXF_ENTITY_TYPE_ARC_ANTICLOCKWISE ?
				dChangeRZOfEachLinePnt : -dChangeRZOfEachLinePnt;
			//�Ƕȱ仯����
			double dAngle = nLineCount1 * dChangeRZOnLine1;

			//Բ���ε�һ��ʵ��ĵ������
			int nFirstArcPntNo = 0;
			for (size_t j = 0; j < i; j++)
			{
				nFirstArcPntNo += vnPntCount[j];
			}

			//�޸�ֱ�߶�1ĩβ�ĽǶ�
			for (size_t j = 0; j < nLineCount1; j++)
			{
				int nRealPntNo = j + nFirstArcPntNo - nLineCount1;
				vtInitWeldCoord[nRealPntNo].dRZ = vtInitWeldCoord[nRealPntNo].dRZ + dChangeRZOnLine1 * j;
			}

			//�޸�Բ���εĽǶ�
			for (size_t j = 0; j < vnPntCount[i]; j++)
			{
				int nRealPntNo = nFirstArcPntNo + j;
				vtInitWeldCoord[nRealPntNo].dRZ = vtInitWeldCoord[nRealPntNo].dRZ + dAngle;
			}

			//�޸�ֱ�߶�2��ʼ�ĽǶ�
			for (size_t j = 0; j < nLineCount2; j++)
			{
				int nRealPntNo = nLineCount2 + nFirstArcPntNo + vnPntCount[i] - j - 1;
				vtInitWeldCoord[nRealPntNo].dRZ = vtInitWeldCoord[nRealPntNo].dRZ + dChangeRZOnLine1 * j;
			}
		}
		//��һ����ֱ�ߣ�������һ����Բ��
		else if (m_vtContourEntities[i - 1].eEntityType == E_DXF_ENTITY_TYPE_LINE
			&& (m_vtContourEntities[i].eEntityType == E_DXF_ENTITY_TYPE_ARC_ANTICLOCKWISE
				|| m_vtContourEntities[i].eEntityType == E_DXF_ENTITY_TYPE_ARC_CLOCKWISE))
		{
			//ֱ�߶���ǰ����̬����
			int nLineCount = vnPntCount[i - 1] > nMaxCountOfChangedLine ? nMaxCountOfChangedLine : vnPntCount[i - 1];
			//ֱ�߶���ǰ����̬ÿһ�ݽǶ�ֵ
			double dChangeRZOnLine =
				m_vtContourEntities[i].eEntityType == E_DXF_ENTITY_TYPE_ARC_ANTICLOCKWISE ?
				dChangeRZOfEachLinePnt : -dChangeRZOfEachLinePnt;
			//Բ���μ��ٱ���̬����
			int nArcCount = vnPntCount[i] / 2 > nLineCount ? vnPntCount[i] / 2 : nLineCount;
			nArcCount = vnPntCount[i] < nArcCount ? vnPntCount[i] : nArcCount;
			//Բ���μ��ٱ���̬ÿһ�ݽǶ�ֵ
			double dChangeRZOnArc = nLineCount * dChangeRZOnLine / nArcCount;

			//Բ���ε�һ��ʵ��ĵ������
			int nFirstArcPntNo = 0;
			for (size_t j = 0; j < i; j++)
			{
				nFirstArcPntNo += vnPntCount[j];
			}

			//�޸�ֱ�߶�ĩβ�ĽǶ�
			for (size_t j = 0; j < nLineCount; j++)
			{
				int nRealPntNo = j + nFirstArcPntNo - nLineCount;
				vtInitWeldCoord[nRealPntNo].dRZ = vtInitWeldCoord[nRealPntNo].dRZ + dChangeRZOnLine * j;
			}

			//�޸�Բ���γ�ʼ�ĽǶ�
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
	//��������
	T_AXISUNIT tAxleUnit = m_pRobot->m_tAxisUnit;//�������
	long nMinLPulse = -1 == m_pRobot->m_nRobotInstallDir ? (long)(-60.0 / tAxleUnit.dLPulse) : (long)(-180.0 / tAxleUnit.dLPulse);//L����Сֵ
	long nMaxLPulse = -1 == m_pRobot->m_nRobotInstallDir ? (long)(30.0 / tAxleUnit.dLPulse) : (long)(180.0 / tAxleUnit.dLPulse);//L�����ֵ
	T_ANGLE_THETA tThetaThreshold = T_ANGLE_THETA(90.0, 45.0, 45.0, 50.0, 90.0, 100.0);//�Ƕ�����
	T_ANGLE_PULSE tAngleThreshold(	//��������
		(long)(tThetaThreshold.dThetaS / tAxleUnit.dSPulse), 
		(long)(tThetaThreshold.dThetaL / tAxleUnit.dLPulse),
		(long)(tThetaThreshold.dThetaU / tAxleUnit.dUPulse), 
		(long)(tThetaThreshold.dThetaR / tAxleUnit.dRPulse),
		(long)(tThetaThreshold.dThetaB / tAxleUnit.dBPulse), 
		(long)(tThetaThreshold.dThetaT / tAxleUnit.dTPulse),
		0, 0, 0);

	//���ÿһ��ֱ������������
	vector<vector<T_ANGLE_PULSE>> vvtRobotPulse(vtTrack.size());	// ÿ��ֱ������ ��Ӧ�Ĺؽ����꼯��
	for (int i = 0; i < vtTrack.size(); i++)
	{
		if (!m_pRobot->RobotInverseKinematics(vtTrack[i], m_pRobot->m_tTools.tGunTool, vvtRobotPulse[i]))
		{
			XiMessageBox("�����˶��켣�����ʧ��!");
			return false;
		}
	}

	//�ѵ�һ�������Ϊ�ο������λ��
	vvtRobotPulse[0].clear();
	vvtRobotPulse[0].push_back(tStartPulse);

	//��ʼ����������켣�ĵ�һ���㼰�����
	vector<vector<T_ANGLE_PULSE>> vvtTrackPulse(0);					// ����������˶��Ĺؽ�����켣
	vector<T_ANGLE_PULSE> vtTotalPulse(0);							// ÿ�������˶��켣�����ۼ�����				
	vector<vector<double>> vvtMaxAngleErr(vvtRobotPulse[0].size());	// ����������˶��Ĺؽ�����켣
	for (int nTrackNo = 0; nTrackNo < vvtRobotPulse[0].size(); nTrackNo++) // ÿ��켣��һ����
	{
		vector<T_ANGLE_PULSE> vtTrackPulse(vvtRobotPulse.size());
		vtTrackPulse[0] = vvtRobotPulse[0][nTrackNo];
		vvtTrackPulse.push_back(vtTrackPulse);
		vtTotalPulse.push_back(vvtRobotPulse[0][nTrackNo]);
		vvtMaxAngleErr[nTrackNo].resize(vvtRobotPulse.size());
	}

	int nFoundIdx = 0;
	for (int nFoundIdx = 1; nFoundIdx < vvtRobotPulse.size(); nFoundIdx++) // ����ÿ��켣 �ڶ������Ժ�ĵ� N
	{
		//��ǰ������ѡ�����弯��
		vector<T_ANGLE_PULSE>& vtTempPulse = vvtRobotPulse[nFoundIdx];

		//�������п�ѡ��������켣
		for (int nTrackNo = 0; nTrackNo < vvtTrackPulse.size(); )
		{
			const T_ANGLE_PULSE& tPulse = vvtTrackPulse[nTrackNo][nFoundIdx - 1];//��һ��ֱ������Ķ�Ӧ��������
			T_ANGLE_PULSE& tCurPulse = vvtTrackPulse[nTrackNo][nFoundIdx];//��ǰֱ������Ķ�Ӧ��������

			//�ҳ����з���Ҫ��ĵ�ǰ��������
			int nMatchNum = 0;//��ǰ����Ҫ�������������Ŀ
			for (int nTempPulseIdx = 0; nTempPulseIdx < vtTempPulse.size(); nTempPulseIdx++)
			{
				//��ǰֱ������Ķ�Ӧ��������
				const T_ANGLE_PULSE& tNewPulse = vtTempPulse[nTempPulseIdx];

				//��ǰ������������һ����������Ĳ�ֵ
				T_ANGLE_PULSE tDelta;
				tDelta.nSPulse = labs(tPulse.nSPulse - tNewPulse.nSPulse);
				tDelta.nLPulse = labs(tPulse.nLPulse - tNewPulse.nLPulse);
				tDelta.nUPulse = labs(tPulse.nUPulse - tNewPulse.nUPulse);
				tDelta.nRPulse = labs(tPulse.nRPulse - tNewPulse.nRPulse);
				tDelta.nBPulse = labs(tPulse.nBPulse - tNewPulse.nBPulse);
				tDelta.nTPulse = labs(tPulse.nTPulse - tNewPulse.nTPulse);

				//���������ֵ����
				if ((tDelta.nSPulse < tAngleThreshold.nSPulse) &&
					(tDelta.nLPulse < tAngleThreshold.nLPulse) &&
					(tDelta.nUPulse < tAngleThreshold.nUPulse) &&
					(tDelta.nRPulse < tAngleThreshold.nRPulse) &&
					(tDelta.nBPulse < tAngleThreshold.nBPulse) &&
					(tDelta.nTPulse < tAngleThreshold.nTPulse) &&
					(tNewPulse.nLPulse > nMinLPulse) &&
					(tNewPulse.nLPulse < nMaxLPulse))
				{
					//���������ʱѡȡ�����ֵ��С��һ��
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

			//û�з���Ҫ�����������
			if (nMatchNum < 1)
			{
				vvtTrackPulse.erase(vvtTrackPulse.begin() + nTrackNo);
				vtTotalPulse.erase(vtTotalPulse.begin() + nTrackNo);
				continue; // ɾ���켣�� ����nTrackNo����
			}

			//�ۼ�����
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
		XiMessageBox("�����˶��켣����ʧ��!");
		return false;
	}

	// ÿ�������˶��켣����ƽ������
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
	if (nChooseIdx < 0 && -1 == m_pRobot->m_nRobotInstallDir) // ����R�����켣���� (��װר�� ��װR�᲻��ת)
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
		XiMessageBox("�����˶��켣ѡ��ʧ��!");
		return false;
	}

	//����������������켣
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

	//��ȣ�������������ʾ�ְ��ȣ������¿��߱�ʾ�¿���ȣ����ں��ӣ���Ϊ0.1
	tEntities.dThick = 0.1;

	//���ͱ��������ڱ�ʾ�¿ڿ�ȣ����ں��ӣ���Ϊ0.1
	tEntities.dLineRatio = 0.1;

	//��ɫ���и��˳��ʱ��
	tEntities.nColorNum = 0;

	//ʵ���Ӧ����ֵ
	tEntities.vtGroupNum.clear();
	tEntities.vtGroupNum.push_back(nContourNo);

	//ͼ�㣬���ں��ӣ���Ϊ������
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

	//С�����ر�ָ������
	tContourPlanePara.tPlateVerDir.dDirX = 0.0;
	tContourPlanePara.tPlateVerDir.dDirY = 0.0;
	tContourPlanePara.tPlateVerDir.dDirZ = 1.0;

	//ȡ�����м��
	tContourPlanePara.tPoint.x = m_vtPartOutPoints[m_vtPartOutPoints.size() / 2].x;
	tContourPlanePara.tPoint.y = m_vtPartOutPoints[m_vtPartOutPoints.size() / 2].y;
	tContourPlanePara.tPoint.z = m_vtPartOutPoints[m_vtPartOutPoints.size() / 2].z;
	return tContourPlanePara;

	//ͳ��ÿ��ʵ�嶼�ж��ٸ���
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

	//����������ȡ���ɵ�
	std::vector <T_POINT_3D> vtTeachPoints;
	for (size_t i = 0; i < m_vtContourEntities.size(); i++)
	{
		//�ҳ���ʵ���ϵĵ���
		auto iter = mnPntCount.find(i);
		if (iter == mnPntCount.end())
		{
			continue;
		}
		int nPntCount = iter->second;

		//�����ֱ��
		if (m_vtContourEntities[i].eEntityType == E_DXF_ENTITY_TYPE_LINE)
		{
			//ȡ�м��
			if (nPntCount < 3)
			{
				vtTeachPoints.push_back(m_vtPartOutPoints[m_vtPartOutPoints.size() / 2]);
			}
			//ȡ�ȼ������
			else if (nPntCount < 300)
			{
				vtTeachPoints.push_back(m_vtPartOutPoints[m_vtPartOutPoints.size() / 3]);
				vtTeachPoints.push_back(m_vtPartOutPoints[m_vtPartOutPoints.size() * 2 / 3]);
			}
			//ÿ200ȡһ���㣬���㲿����β����
			else
			{
				//������βʣ�����
				int nRemainingQuantity = nPntCount % 200;
				nRemainingQuantity = nRemainingQuantity < 60 ? nRemainingQuantity + 200 : nRemainingQuantity;

				//�ȼ��ȡ��
				for (size_t j = nRemainingQuantity / 2; j < nPntCount; j += 200)
				{
					vtTeachPoints.push_back(m_vtPartOutPoints[j]);
				}
			}
		}
		//���������ΪԲ��
		else
		{
			//ȡ�м��
			if (nPntCount < 5)
			{
				vtTeachPoints.push_back(m_vtPartOutPoints[m_vtPartOutPoints.size() / 2]);
			}
			//ȡ�ȼ������
			else if (nPntCount < 400)
			{
				vtTeachPoints.push_back(m_vtPartOutPoints[m_vtPartOutPoints.size() / 4]);
				vtTeachPoints.push_back(m_vtPartOutPoints[m_vtPartOutPoints.size() * 2 / 4]);
				vtTeachPoints.push_back(m_vtPartOutPoints[m_vtPartOutPoints.size() * 3 / 4]);
			}
			//ÿ150ȡһ���㣬���㲿����β����
			else
			{
				//������βʣ�����
				int nRemainingQuantity = nPntCount % 150;
				nRemainingQuantity = nRemainingQuantity < 40 ? nRemainingQuantity + 150 : nRemainingQuantity;

				//�ȼ��ȡ��
				for (size_t j = nRemainingQuantity / 2; j < nPntCount; j += 150)
				{
					vtTeachPoints.push_back(m_vtPartOutPoints[j]);
				}
			}
		}
	}

	//����ƽ�����
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
					XiMessageBox("�켣��������������");
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
			XiMessageBox("�����¿���������!");
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
			XiMessageBox("�����¿���������!");
		}
	}
	else
	{
		XiMessageBox("����������������!");
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

	//���켣ת����������ϵ��
	std::vector<T_ROBOT_COORS_IN_WORLD> vtCutRobotCoors = m_pWeldTrack->toWorldD<T_ROBOT_COORS_IN_WORLD>();
	vtBevelPoints = m_vtBevelPoints;

	//GetCameraIfDoTrackingNew(tContourPlanePara, vtContourEntities, vtBevelPoints, vnBevelEntityNo, vtCutRobotCoors, eCamId, vtDoTrackPara, vtTrackingPointInImage);
	GetCameraIfDoTrackingNew(tContourPlanePara, vtContourEntities, vtBevelPoints, vnBevelEntityNo, vtCutRobotCoors, eCamId, vtDoTrackPara, vtTrackingPointInImage, m_vtHorTrackingPointInImage, m_vtVerTrackingPointInImage);

	//ĩβ�ٿ�һ��
	//�����һ��������Բ����������
	bool isSeeZeroNo = true;
	int nDeleteNo = 0;
	if (m_pWeldTrack->isClosed())
	{
		for (int n = vtDoTrackPara.size() - 1; n > 0; n--)
		{
			//���ĩβ����0��ʵ�壬����Ϊ������
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
			//���ĩβ������0��ʵ�壬�ٿ�Nmm
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
		//�պ�����β������0��ʵ�岻���Ը���
		//if (vtDoTrackPara[i].nEntityNo > 0)
		//	isNotZeroNo = true;
		//if (vtDoTrackPara[i].nEntityNo == 0 && isNotZeroNo)
		//{
		//	vtDoTrackPara[i].bIfDoTracking = false;
		//}

		//���޷����ٵ����Ը���ʱ��ǰN���켣��Ȼ��Ϊ���ɸ��٣��Ա���б�ʴ���
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

		//�Ǳպ����ߵ�ǰ50mm�����٣��Ա���б�ʴ���
		if (!m_pWeldTrack->isClosed() && i < 50)
		{
			vtDoTrackPara[i].bIfDoTracking = false;
		}

		//�������Ľ����������Ұ����Ϊ���ɸ���
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

	// �������һ���ܸ��ٵ����ݵ�����
	int n = 0;
	for (n = vtDoTrackPara.size() - 1; n > 0; n--)
	{
		if (true == vtDoTrackPara[n].bIfDoTracking)
		{
			break;
		}
	}
	m_nLastPointOfTracking = n;

	// ��̬�޸��յ㲻������ֵ��ȷ���ڴ˷�Χ���и���ʾ�̵�
	int nUnTrackingPointNum = m_vtDoTrackPara.size() - m_nLastPointOfTracking;
	if (nUnTrackingPointNum > 100) // 100Ϊ���۹�ϵ�����еķ��� 1����һ��
	{
		m_nEndAdjustLimit = m_nEndAdjustLimit + (nUnTrackingPointNum - 100);
	}

	m_vtTrackingPointInImage.swap(vtTrackingPointInImage);
	writeLog("ʵʱ���٣����һ���ɸ��ٵ�������%d, �켣����:%d���յ㲻����������%d",
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
//	double dDisLimit = m_dDisLimitForCalDoTracking;			// ��ǰ����� �� ͼ�����ļ���� ����� mm
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

	double dRxSearchRange = 30.0;//��������켣ʱ��RX��������Χ
	double dRySearchRange = 20.0;//��������켣ʱ��RY��������Χ
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
		//AfxMessageBox("ֱ�߷����������");
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
	//���⿽��һ������
	std::vector<T_POINT_3D> vtTeachRobotWeldPoints;
	{
		CRITICAL_AUTO_LOCK(m_cLock);
		vtTeachRobotWeldPoints = m_vtTeachRobotWeldPoints;
	}

	// ��m_vtBevelPoints�в��Ҳ�ͼʱ����ĵ������
	T_POINT_3D tPoint;
	tPoint.x = tImgCoord.dX + tImgCoord.dBX;
	tPoint.y = tImgCoord.dY + tImgCoord.dBY;
	tPoint.z = tImgCoord.dZ + tImgCoord.dBZ;

	auto time = XI_clock();
	int nIndex = FindNearestPoint(tPoint, vtTeachRobotWeldPoints);

	writeLog("�����ͼλ��������и�켣��������%d, ������ʵ��ţ�%d��������ʱ��%dms",
		nIndex, m_vtDoTrackPara[nIndex].nEntityNo, XI_clock() - time);

	return &(m_vtDoTrackPara[nIndex]);
}

bool RTT::AdjustTrack::JudgeToDoAdjust(T_ALGORITHM_POINT tSeenPos,
	T_DOTRACKING_PARA tSeenEntity, T_ALGORITHM_POINT& tFilterOutPoint, bool& bOutValid)
{
	// ���ݲ�ͼʱ������ �ж��Ƿ񿴵ü� ���������ĸ�ʵ�� ��������ֱ�߻���Բ�� 
	// 1�����ü���ʹ�ÿ���������
	// 2��ÿ��ʵ��ʹ�õ�����һ���˲����뼯��(�������ӵ�ֱ��Ҳ������ʵ�壬�˲�Ҳ��Ҫ�ֿ�)
	// 3����������Բ������ֱ�� �˲�������ͬ
	if (false == tSeenEntity.bIfDoTracking
		|| tSeenEntity.nEntityNo < 0) // ���ܸ���
	{
		writeLog("��ǰ������λ���޷�����");
		bOutValid = false;
		return false;
	}

	// ��ӵ��˲�����
	//double dHandEyeDis = 70.0; // �˲������������೤�����ݽ����˲�
	//double dFilterInputPointSpaceing = 1.0;
	std::vector<T_ALGORITHM_POINT>& vtFilterInput = m_mnvFilterInput[tSeenEntity.nEntityNo];
	int nSrcFilterInputNum = vtFilterInput.size();
	if (0 == nSrcFilterInputNum) // ��һ����ֱ�����
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
			writeLog("�˲������µ�����һ���������С��%.3lf mm", m_dMinFilterInputPointSpacing);
			bOutValid = false;
			return false;
		}
	}

	// ɾ������ĵ� �����¿�����λ������ ֻ����һ�����۵ľ�����Ϊ�˲�����
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
	writeLog("��ʼɾ�� size %d", vtFilterInput.size());
	if (nDelIndex >= 0)
	{
		vtFilterInput.erase(vtFilterInput.begin(), vtFilterInput.begin() + nDelIndex);
	}
	writeLog("ɾ������ size %d", vtFilterInput.size());

	// �жϵ�ǰ�˲����뼯�ϵ����Ƿ���Խ����˲�
	if (m_nMinFilterInputPointNum > vtFilterInput.size()) // �����̫�� ���˲�
	{
		writeLog("ʵʱ���٣�����̫�ٲ����˲�");
		bOutValid = false;
		return false;
	}

	// ִ���˲�
	std::vector<T_ALGORITHM_POINT> vtOutput;
	E_FIT_TYPE eFitType = (1 == tSeenEntity.nEntityType ? E_LINE : E_CIRCLE);
	m_cXiAlgorithm.PointSmooth(vtFilterInput, eFitType/*E_LINE*/, 0.8, vtOutput);
	tFilterOutPoint = vtOutput[vtOutput.size() - 1];
	bOutValid = true;
	m_nFilterOutValidNum++;

	// �˲��������ÿ���������һ��
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
			writeLog("����������ֵ��%.3lf��%.3lf", dAdjustDis, dAdjustDisZ);
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
		//����ƫ��
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
	double dDisLimit = m_dDisLimitForCalDoTracking;			// ��ǰ����� �� ͼ�����ļ���� ����� mm
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

	double dDisLimit = m_dDisLimitForCalDoTracking;			// ��ǰ����� �� ͼ�����ļ���� ����� mm
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

							////���㼤��ͼ�����
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
							
							//���⵹V�͹̶����򣬽��������Y��С
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

