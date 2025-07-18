#include "stdafx.h"
#include "RealTimeTrack.h"
#include ".\Apps\PLib\CtrlUnit\CUnit.h"
#include "Project/AssemblyWeld.h"

struct RTT::RealTimeTrack::T_IMG_INFO
{
	/*******************************采集数据*******************************/
	IplImage* img = nullptr;						//图片地址
	int nRealImgNo = -1;							//真实图片序号
	bool bIsInvalid = false;						//是否是无效的图片数据
	T_ROBOT_COORS tRobotCoord;						//机器人直角坐标
	T_ANGLE_PULSE tRobotPulse;						//机器人脉冲坐标

	/*******************************交点数据*******************************/
	CvPoint tIdeal2DKeyPoint = {0};					//理论2D激光交点
	CvPoint tHorTrackingPointInImage = { 0 };		//理论底板方向激光二维点
	CvPoint tVerTrackingPointInImage = { 0 };		//理论立板方向激光二维点

	CvPoint t2DKeyPoint;							//2D激光交点
	T_ROBOT_COORS t3DKeyPoint;						//3D激光交点
	bool bGetKeyPntResult = false;					//从图片获得交点成功/失败
	std::atomic<bool> bGetKeyPntComplete = false;	//已从图片获得交点（不代表获取的交点是否有效）

	/*******************************轨迹修正*******************************/
	std::atomic<bool> bIsUsedByAdjustTrack = false;	//已经被轨迹修正使用(包括被丢弃的)
};

struct RTT::RealTimeTrack::T_PRO_IMG_THREAD_INFO
{
	RealTimeTrack* pThis = nullptr;
	int nThreadNo = 0;								//子线程序号
	int nPreviousImgNo = -1;						//已经处理的图片序号
	int nCurImgNo = -1;								//正在处理的图片序号
	std::atomic<bool> bIsThreadEnd;					//线程是否已经退出

	void resetLockInfo()
	{
		vtWidgetLaserInfo.clear();
		vtWidgetLaserInfo.reserve(m_nLockInfoBufferSize + 5);
	}

	void addNewLockInfo(const WidgetLaserInfo& tWidgetLaserInfo)
	{
		vtWidgetLaserInfo.push_back(tWidgetLaserInfo);
		while (vtWidgetLaserInfo.size() >= m_nLockInfoBufferSize)
		{
			vtWidgetLaserInfo.erase(vtWidgetLaserInfo.begin());
		}
	}

	void addIdealLockInfo(const WidgetLaserInfo& tWidgetLaserInfo)
	{
		//插入理想斜率
		if (vtWidgetLaserInfo.size() > 5)
		{
			vtWidgetLaserInfo.insert(vtWidgetLaserInfo.end() - 5, tWidgetLaserInfo);
		}
		else
		{
			vtWidgetLaserInfo.insert(vtWidgetLaserInfo.begin(), tWidgetLaserInfo);
		}
		
		while (vtWidgetLaserInfo.size() >= m_nLockInfoBufferSize)
		{
			vtWidgetLaserInfo.erase(vtWidgetLaserInfo.begin());
		}
	}

	WidgetLaserInfo* getLastLockInfo()
	{
		return &vtWidgetLaserInfo.back();
	}

	bool getKeyPnt(RTT::RealTimeTrack::T_IMG_INFO& tCurImgInfo, CString sPara = "WidgetLaserTrack")
	{
		for (int i = vtWidgetLaserInfo.size() - 1; i >= 0; i--)
		{
			WidgetLaserInfo tNewWidgetLaserInfo = vtWidgetLaserInfo[i];
			if (WidgetLaserTrack(tCurImgInfo.img, &tNewWidgetLaserInfo, sPara.GetBuffer()))
			{
				addNewLockInfo(tNewWidgetLaserInfo);
				return true;
			}
		}
		return false;
	}

private:
	//WidgetLaserInfo* pWidgetLaserInfo = nullptr;	//图像处理函数参数
	std::vector<WidgetLaserInfo> vtWidgetLaserInfo;	//图像处理函数参数
	//WidgetLaserTrackTool* pWidgetLaserTrack = nullptr;	//图像处理类指针
	int m_nLockInfoBufferSize = 32;								//锁定数据缓存大小
};

RTT::RealTimeTrack::RealTimeTrack()
{
}

RTT::RealTimeTrack::~RealTimeTrack()
{
	for (size_t i = 0; i < m_vpProImgThreadInfo.size(); i++)
	{
		//deletePointer(m_vpProImgThreadInfo[i]->pWidgetLaserInfo);
		//deletePointer(m_vpProImgThreadInfo[i]->pWidgetLaserTrack);
		deletePointer(m_vpProImgThreadInfo[i]);
	}
	releaseImgBuffer();
	deletePointer(m_pWidgetLaserInfo);
}

void RTT::RealTimeTrack::setEnable(bool bEnable)
{
	m_bEnable = bEnable;
}

bool RTT::RealTimeTrack::isEnable() const
{
	return m_bEnable;
}

void RTT::RealTimeTrack::setUnit(CUnit& cUnit)
{
	m_pUnit = &cUnit;
	setCamera(*((CDHGigeImageCapture*)m_pUnit->GetCameraCtrl(m_pUnit->m_nTrackCameraNo)));
	setRobot(*(m_pUnit->GetRobotCtrl()));
}

void RTT::RealTimeTrack::setCamera(CDHGigeImageCapture& cCamera)
{
	m_pCamera = &cCamera;
}

void RTT::RealTimeTrack::setRobot(CRobotDriverAdaptor& cRobot)
{
	m_pRobot = &cRobot;
}

void RTT::RealTimeTrack::initTrack(PartData::WeldTrack& cWeldTrack)
{
	m_pWeldTrack = &cWeldTrack;
	auto tTheoryEndPnt = cWeldTrack.back();
	m_tTheoryEndPnt.x = tTheoryEndPnt.dX + tTheoryEndPnt.dBY;
	m_tTheoryEndPnt.y = tTheoryEndPnt.dY + tTheoryEndPnt.dBY;
	m_tTheoryEndPnt.z = tTheoryEndPnt.dZ + tTheoryEndPnt.dBZ;
}

void RTT::RealTimeTrack::setLaserLockResult(WidgetLaserInfo& tWidgetLaserInfo)
{
	auto pWidgetLaserInfo = new WidgetLaserInfo;
	memcpy(pWidgetLaserInfo, &tWidgetLaserInfo, sizeof(WidgetLaserInfo));
	std::swap(m_pWidgetLaserInfo, pWidgetLaserInfo);
	delete pWidgetLaserInfo;
}

void RTT::RealTimeTrack::setCompensation(double dNormalComp, double dHeightComp)
{
	m_dNormalComp = dNormalComp;
	m_dHeightComp = dHeightComp;
	writeLog("dNormalComp=%.3lf dHeightComp=%.3lf", m_dNormalComp, m_dHeightComp);
}

bool RTT::RealTimeTrack::initBeforeRun()
{
	//初始化跟踪调整
	m_cAdjustTrack.setUnit(*m_pUnit);
	m_cAdjustTrack.initTrack(*m_pWeldTrack);
	return m_cAdjustTrack.initBeforeTrack();
}

void RTT::RealTimeTrack::startRun()
{
	if (!m_bEnable)
	{
		//重置采图子线程状态
		m_bEndGetImage = true;
		m_bPauseGetImage = true;
		m_bIsGetImageEnd = true;

		//重置图片处理子线程状态
		for (size_t i = 0; i < m_vpProImgThreadInfo.size(); i++)
		{
			//deletePointer(m_vpProImgThreadInfo[i]->pWidgetLaserInfo);
			//deletePointer(m_vpProImgThreadInfo[i]->pWidgetLaserTrack);
			deletePointer(m_vpProImgThreadInfo[i]);
		}
		m_vpProImgThreadInfo.clear();

		//重置轨迹修正子线程状态
		m_bIsAdjustTrackEnd = true;
		return;
	}

	//开始采图
	createImgBuffer();
	m_bEndGetImage = false;
	m_bPauseGetImage = false;
	m_bIsGetImageEnd = false;
	m_nNextGetImgNo = 0;
	m_nLockFailTimes = 0;
	m_bFirstLockComplete = false;
	AfxBeginThread(ThreadGetImg, this);

	//开始处理
	for (size_t i = 0; i < m_vpProImgThreadInfo.size(); i++)
	{
		//deletePointer(m_vpProImgThreadInfo[i]->pWidgetLaserInfo);
		//deletePointer(m_vpProImgThreadInfo[i]->pWidgetLaserTrack);
		deletePointer(m_vpProImgThreadInfo[i]);
	}
	m_vpProImgThreadInfo.resize(m_nImgProcessThreadCount);
	for (size_t i = 0; i < m_nImgProcessThreadCount; i++)
	{
		m_vpProImgThreadInfo[i] = new T_PRO_IMG_THREAD_INFO;
		m_vpProImgThreadInfo[i]->pThis = this;
		m_vpProImgThreadInfo[i]->nThreadNo = i;
		m_vpProImgThreadInfo[i]->bIsThreadEnd = false;
		m_vpProImgThreadInfo[i]->resetLockInfo();
		m_vpProImgThreadInfo[i]->addNewLockInfo(*m_pWidgetLaserInfo);
		//m_vpProImgThreadInfo[i]->pWidgetLaserInfo = new WidgetLaserInfo;
		//m_vpProImgThreadInfo[i]->pWidgetLaserTrack = new WidgetLaserTrackTool;
		//memcpy(m_vpProImgThreadInfo[i]->pWidgetLaserInfo, m_pWidgetLaserInfo, sizeof(WidgetLaserInfo));
		//m_vpProImgThreadInfo[i]->pWidgetLaserTrack->AddTrackParamSet("WidgetLaserTrack");
		//m_vpProImgThreadInfo[i]->pWidgetLaserTrack->AddLockInfo(m_pWidgetLaserInfo);
		AfxBeginThread(ThreadImageProcess, m_vpProImgThreadInfo[i]);
	}

	//开始修正焊接轨迹
	m_bPauseAdjustTrack = false;
	m_bIsAdjustTrackEnd = false;
	AfxBeginThread(ThreadAdjustTrack, this);

	//终点判断线程
	m_bIsStartGetPntCloud = false;
}

void RTT::RealTimeTrack::endRun()
{
	//停止采图
	m_bEndGetImage = true;
}

void RTT::RealTimeTrack::waitUntilEnd()
{	
	bool bIsEnd = false;
	while (!bIsEnd)
	{
		Sleep(20);

		//采图子线程状态
		bIsEnd = m_bIsGetImageEnd;

		//图像处理子线程状态
		for (size_t i = 0; i < m_vpProImgThreadInfo.size(); i++)
		{
			bIsEnd = bIsEnd && m_vpProImgThreadInfo[i]->bIsThreadEnd;
		}

		//轨迹修正子线程状态
		bIsEnd = bIsEnd && m_bIsAdjustTrackEnd;

		//终点判断子线程状态

	}
}

void RTT::RealTimeTrack::emg()
{
	//结束运行
	endRun();

	//停止焊接运动
	m_pRobot->m_eThreadStatus = INCISEHEAD_THREAD_STATUS_STOPPED;
	m_pRobot->HoldOn();
	m_pRobot->ServoOff();
	//已修改
	XUI::MesBox::PopInfo("跟踪：{0}机械臂急停", m_pRobot->m_strRobotName.GetBuffer());
	//XiMessageBox("跟踪：%s 机械臂急停", m_pRobot->m_strRobotName);
}

void RTT::RealTimeTrack::pauseGetImg()
{
	m_bPauseGetImage = true;
}

void RTT::RealTimeTrack::continueGetImg()
{
	m_bPauseGetImage = false;
}

void RTT::RealTimeTrack::pauseAdjustTrack()
{
	m_bPauseAdjustTrack = true;
}

void RTT::RealTimeTrack::continueAdjustTrack()
{
	m_bPauseAdjustTrack = false;
}

UINT RTT::RealTimeTrack::ThreadGetImg(void* pParam)
{
	RTT::RealTimeTrack* pcMyObj = (RTT::RealTimeTrack*)pParam;
	pcMyObj->getImg();
	return 0;
}

UINT RTT::RealTimeTrack::ThreadImageProcess(void* pParam)
{
	T_PRO_IMG_THREAD_INFO* pcMyObj = (T_PRO_IMG_THREAD_INFO*)pParam;
	pcMyObj->pThis->processImage(*pcMyObj);
	return 0;
}

UINT RTT::RealTimeTrack::ThreadAdjustTrack(void* pParam)
{
	RTT::RealTimeTrack* pcMyObj = (RTT::RealTimeTrack*)pParam;
	pcMyObj->adjustTrack();
	return 0;
}

void RTT::RealTimeTrack::getImg()
{
	long long llStartTime = XI_clock();//开始采图时间
	long long llLastTime = llStartTime;//上一次采图时间
	int nPauseTimes = 0;//暂停循环次数

	while (!m_bEndGetImage)
	{
		//如果暂停采图
		if (m_bPauseGetImage)
		{
			Sleep(1000);
			nPauseTimes++;
		}

		//限制最大帧率
		long long llWaitTime = llLastTime + (1000 / m_dMaxFrame + 1) - XI_clock();
		if (llWaitTime > 0)
		{
			Sleep(llWaitTime);
		}
		llLastTime = XI_clock();

		//将要采集的图片数据缓存区
		T_IMG_INFO& tNewEndPointInfo = *(m_vpImgInfo[m_nNextGetImgNo % m_nImgBufferCount]);

		//如果上一次的图还没处理
		if (m_nNextGetImgNo >= m_nImgBufferCount
			&& !tNewEndPointInfo.bIsUsedByAdjustTrack
			&& !tNewEndPointInfo.bIsInvalid)
		{
			emg();
			XiMessageBox("跟踪：图像不能及时处理");
			break;
		}

		//获取图片
		if (!m_pCamera->CaptureImage(tNewEndPointInfo.img, 1))
		{
			emg();
			XiMessageBox("跟踪：采图失败！");
			break;
		}
		tNewEndPointInfo.nRealImgNo = m_nNextGetImgNo;

		//获取坐标
		tNewEndPointInfo.bGetKeyPntResult = false;
		tNewEndPointInfo.bGetKeyPntComplete = false;
		tNewEndPointInfo.bIsUsedByAdjustTrack = false;
		tNewEndPointInfo.tRobotCoord = m_pRobot->GetCurrentPos();
		tNewEndPointInfo.tRobotPulse = m_pRobot->GetCurrentPulse();

		//如果是有效图片，则存图
		if (m_cAdjustTrack.determineWhetherToProcessImg(tNewEndPointInfo.tRobotCoord, 
			tNewEndPointInfo.tIdeal2DKeyPoint, tNewEndPointInfo.tHorTrackingPointInImage, tNewEndPointInfo.tVerTrackingPointInImage))
		{
			long long llTime2 = XI_clock();
			saveImg(tNewEndPointInfo);
			auto llSaveTime = XI_clock() - llTime2;
			if (llSaveTime > 20)
			{
				writeLog("保存图片大量用时：%dms", llSaveTime);
			}
			else
			{
				writeLog("保存图片用时：%dms", llSaveTime);
			}
			tNewEndPointInfo.bIsInvalid = false;
		}
		//无效图片
		else
		{
			tNewEndPointInfo.bGetKeyPntResult = false;
			tNewEndPointInfo.bGetKeyPntComplete = false;
			tNewEndPointInfo.bIsInvalid = true;
		}
		//更新数据
		writeLog("更新图片%d，该图片%s", (int)m_nNextGetImgNo,
			tNewEndPointInfo.bIsInvalid ? "无效" : "有效");
		m_nNextGetImgNo++;
	}

	//退出处理
	m_bIsGetImageEnd = true;
	writeLog("结尾采图线程退出。帧率：%.3lf",
		m_nNextGetImgNo * 1000.0 / double(XI_clock() - llStartTime - 1000.0 * nPauseTimes));
}

void RTT::RealTimeTrack::processImage(T_PRO_IMG_THREAD_INFO& tProImgInfo)
{
	//初始化参数
	int nProcessImgCount = 0;//处理图片总数
	long long llProcessTime = 0;//处理总时间
	tProImgInfo.nPreviousImgNo = tProImgInfo.nThreadNo - m_nImgProcessThreadCount;//保证下一张图是线程序号

	//持续处理图片
	while (true)
	{
		//等待获取有效图片
		auto pCurImgInfo = waitForNextImgForProcess(tProImgInfo);
		if (!pCurImgInfo)
		{
			break;
		}

		//提取激光交点
		long long llTime1 = XI_clock();
		auto pWidgetLaserInfo = tProImgInfo.getLastLockInfo();
		writeLog("m_pLaserInfo 入参：%d %d %d %d %d %d ", pWidgetLaserInfo->crossPoint.x, pWidgetLaserInfo->crossPoint.y,
			pWidgetLaserInfo->verticalPlatePnt.x, pWidgetLaserInfo->verticalPlatePnt.y,
			pWidgetLaserInfo->bottomPlatePnt.x, pWidgetLaserInfo->bottomPlatePnt.y);

		WidgetLaserInfo tIdealWidgetLaserInfo = *pWidgetLaserInfo;
		tIdealWidgetLaserInfo.crossPoint = pCurImgInfo->tIdeal2DKeyPoint;
		tIdealWidgetLaserInfo.bottomPlatePnt = pCurImgInfo->tHorTrackingPointInImage;
		tIdealWidgetLaserInfo.verticalPlatePnt = pCurImgInfo->tVerTrackingPointInImage;

		tProImgInfo.addIdealLockInfo(tIdealWidgetLaserInfo);
		pCurImgInfo->bGetKeyPntResult = tProImgInfo.getKeyPnt(*pCurImgInfo);
			//WidgetLaserTrack(pCurImgInfo->img, pWidgetLaserInfo, "WidgetLaserTrack");
			//tProImgInfo.pWidgetLaserTrack->DoTrack(pCurImgInfo->img, pWidgetLaserInfo, "WidgetLaserTrack");

		pWidgetLaserInfo = tProImgInfo.getLastLockInfo();

		nProcessImgCount++;
		if (!pCurImgInfo->bGetKeyPntResult)
		{
			writeLog("图片%d处理失败", pCurImgInfo->nRealImgNo);
			pCurImgInfo->bGetKeyPntComplete = true;
			continue;
		}
		pCurImgInfo->t2DKeyPoint = pWidgetLaserInfo->crossPoint;

		writeLog("图片%d，理论二维点：%d-%d，实际二维点：%d-%d", pCurImgInfo->nRealImgNo,
			pCurImgInfo->tIdeal2DKeyPoint.x, pCurImgInfo->tIdeal2DKeyPoint.y,
			pCurImgInfo->t2DKeyPoint.x, pCurImgInfo->t2DKeyPoint.y);

		//采集点云
		if (m_bIsStartGetPntCloud)
		{

		}

		//二维转三维
		laserPnt2DTo3D(*pCurImgInfo);
		long long llTime5 = XI_clock();
		llProcessTime = llProcessTime + llTime5 - llTime1;

		//显示
		if (pCurImgInfo->nRealImgNo % (m_nMinShowImgSampleGap + 1) == 0)
		{
			auto llShowTime = XiBase::XI_clock();
			IplImage* pColorImg = cvCreateImage(cvSize(pCurImgInfo->img->width, pCurImgInfo->img->height), IPL_DEPTH_8U, 3);
			cvCvtColor(pCurImgInfo->img, pColorImg, CV_GRAY2RGB);
			cvCircle(pColorImg, pCurImgInfo->t2DKeyPoint, 10, CV_RGB(255, 0, 0), 3);
			((CAssemblyWeld*)getMainDlg())->ShowTeachImage(pColorImg);
			cvReleaseImage(&pColorImg);
			writeLog("显示耗时：%dms", XiBase::XI_clock() - llShowTime);
		}

		//刷新状态
		pCurImgInfo->bIsInvalid = false;
		pCurImgInfo->bGetKeyPntComplete = true;
	}
	writeLog("线程%d处理图片平均用时：%.2lfms",
		tProImgInfo.nThreadNo,
		llProcessTime / (nProcessImgCount * 1.0));
	tProImgInfo.bIsThreadEnd = true;
}

RTT::RealTimeTrack::T_IMG_INFO* RTT::RealTimeTrack::waitForNextImgForProcess(T_PRO_IMG_THREAD_INFO& tProImgInfo)
{
	bool bIsInvalid = false;
	bool bIsUsable = false;
	bool bLockFail = false;

	//仍在采图
	while (!m_bIsGetImageEnd)
	{
		Sleep(10);

		//更新正在处理的图片序号
		tProImgInfo.nCurImgNo = tProImgInfo.nPreviousImgNo + m_nImgProcessThreadCount;

		//如果未采集到需要的图
		if (m_nNextGetImgNo <= tProImgInfo.nCurImgNo)
		{
			//等待一帧
			Sleep(1000 / m_dMaxFrame);
			continue;
		}

		//当前正在处理的图片数据指针
		auto pCurImgInfo = m_vpImgInfo[tProImgInfo.nCurImgNo % m_nImgBufferCount];

		//更新已经处理的图片序号
		tProImgInfo.nPreviousImgNo = tProImgInfo.nCurImgNo;

		//无效图片不处理
		if (pCurImgInfo->bIsInvalid)
		{
			bIsInvalid = true;
			pCurImgInfo->bGetKeyPntResult = false;
			pCurImgInfo->bGetKeyPntComplete = false;
			continue;
		}
		bIsUsable = true;

		//0号线程还没有完后初次锁定，或者出现无效到有效的转变
		if (tProImgInfo.nThreadNo == 0 && bIsUsable && (!m_bFirstLockComplete || bIsInvalid || bLockFail))
		{
			m_bPauseGetImage = true;
			Sleep(300);

			writeLog("运行中开始锁定斜率");
			auto llLockTime = XiBase::XI_clock();
			//使用理论斜率求取
			{
				m_pWidgetLaserInfo->crossPoint = pCurImgInfo->tIdeal2DKeyPoint;
				m_pWidgetLaserInfo->bottomPlatePnt = pCurImgInfo->tHorTrackingPointInImage;
				m_pWidgetLaserInfo->verticalPlatePnt = pCurImgInfo->tVerTrackingPointInImage;

				//提取激光交点
				long long llTime1 = XI_clock();
				writeLog("图片%d锁定，m_pLaserInfo 入参：%d %d %d %d %d %d ", pCurImgInfo->nRealImgNo,
					m_pWidgetLaserInfo->crossPoint.x, m_pWidgetLaserInfo->crossPoint.y,
					m_pWidgetLaserInfo->verticalPlatePnt.x, m_pWidgetLaserInfo->verticalPlatePnt.y,
					m_pWidgetLaserInfo->bottomPlatePnt.x, m_pWidgetLaserInfo->bottomPlatePnt.y);

				//保存理论图
				{
					IplImage* pColorImg = cvCreateImage(cvSize(pCurImgInfo->img->width, pCurImgInfo->img->height), IPL_DEPTH_8U, 3);
					cvCvtColor(pCurImgInfo->img, pColorImg, CV_GRAY2RGB);
					cvCircle(pColorImg, m_pWidgetLaserInfo->crossPoint, 10, CV_RGB(0, 255, 0), 6);
					cvCircle(pColorImg, m_pWidgetLaserInfo->verticalPlatePnt, 10, CV_RGB(255, 255, 0), 6);
					cvCircle(pColorImg, m_pWidgetLaserInfo->bottomPlatePnt, 10, CV_RGB(255, 0, 0), 6);
					CString sFileName;
					sFileName.Format(getDataFileRootFolder() + "Lock//LockImg%d.jpg", pCurImgInfo->nRealImgNo);
					int p[3];
					p[0] = CV_IMWRITE_JPEG_QUALITY;
					p[1] = 10;
					p[2] = 0;
					cvSaveImage(sFileName, pColorImg, p);
					cvReleaseImage(&pColorImg);
				}

				tProImgInfo.addNewLockInfo(*m_pWidgetLaserInfo);
				auto bGetKeyPntResult = 
					tProImgInfo.getKeyPnt(*pCurImgInfo, "WidgetLaserReLock");
					//WidgetLaserTrack(pCurImgInfo->img, m_pWidgetLaserInfo, "WidgetLaserReLock");
					//tProImgInfo.pWidgetLaserTrack->DoTrack(pCurImgInfo->img, m_pWidgetLaserInfo, "WidgetLaserTrack");
				memcpy(m_pWidgetLaserInfo, tProImgInfo.getLastLockInfo(), sizeof(WidgetLaserInfo));

				if (!bGetKeyPntResult)
				{
					emg();
					XiMessageBoxOk("初始对点！(错误代码：找不到关键点)");
					return false;
				}

				writeLog("图片%d锁定，m_pLaserInfo 出参：%d %d %d %d %d %d ", pCurImgInfo->nRealImgNo,
					m_pWidgetLaserInfo->crossPoint.x, m_pWidgetLaserInfo->crossPoint.y,
					m_pWidgetLaserInfo->verticalPlatePnt.x, m_pWidgetLaserInfo->verticalPlatePnt.y,
					m_pWidgetLaserInfo->bottomPlatePnt.x, m_pWidgetLaserInfo->bottomPlatePnt.y);

				for (size_t k = 0; k < m_vpProImgThreadInfo.size(); k++)
				{
					//memcpy(m_vpProImgThreadInfo[k]->pWidgetLaserInfo, m_pWidgetLaserInfo, sizeof(WidgetLaserInfo));
					m_vpProImgThreadInfo[k]->addNewLockInfo(*m_pWidgetLaserInfo);// ->AddLockInfo(m_pWidgetLaserInfo);
				}
				m_bFirstLockComplete = true;
			}
			//重新锁定斜率
			{
				////锁定
				//CvPoint cpMidKeyPoint;//交点
				//WidgetLaserInfo* pWidgetLaserInfo = new WidgetLaserInfo[10];
				//int nRst = WidgetLaserLock(pCurImgInfo->img, pWidgetLaserInfo, false, false, "WidgetLaserLock");

				////寻找符合要求的交点
				//int nFindNo = -1;//符合要求的交点
				//double dMinDis = 20.0;//最近点的距离
				//for (int i = 0; i < nRst; i++)
				//{
				//	//计算交点三维坐标
				//	T_ABS_POS_IN_BASE tPointAbsCoordInBase;
				//	T_ROBOT_COORS tAimCoord = m_pUnit->TranImageToBase(
				//		m_pUnit->m_nTrackCameraNo, pWidgetLaserInfo[i].crossPoint,
				//		pCurImgInfo->tRobotCoord, pCurImgInfo->tRobotPulse,
				//		&tPointAbsCoordInBase);

				//	//找到最近的坐标
				//	T_ROBOT_COORS tNearestCoord;
				//	if (m_pWeldTrack->getNearestCoordFast(tNearestCoord, tAimCoord, dMinDis, 0))
				//	{
				//		double dTemp = SQUARE(tNearestCoord.dX + tNearestCoord.dBX - tAimCoord.dX - tAimCoord.dBX)
				//			+ SQUARE(tNearestCoord.dY + tNearestCoord.dBY - tAimCoord.dY - tAimCoord.dBY)
				//			+ SQUARE(tNearestCoord.dZ + tNearestCoord.dBZ - tAimCoord.dZ - tAimCoord.dBZ);
				//		dMinDis = sqrt(dTemp);//更新最近距离
				//		cpMidKeyPoint = pWidgetLaserInfo[i].crossPoint;//更新交点
				//		setLaserLockResult(pWidgetLaserInfo[i]);//更新锁定参数
				//		for (size_t k = 0; k < m_vpProImgThreadInfo.size(); k++)
				//		{
				//			memcpy(m_vpProImgThreadInfo[k]->pWidgetLaserInfo, m_pWidgetLaserInfo, sizeof(WidgetLaserInfo));
				//		}
				//		nFindNo = i;//更新序号
				//	}
				//}
				//DELETE_POINTER_ARRAY(pWidgetLaserInfo);

				////没找到合适的交点
				//bLockFail = nRst <= 0 || nFindNo < 0;
				//if (bLockFail)
				//{
				//	emg();
				//	XiMessageBoxOk("初始对点！(错误代码：找不到关键点)");
				//	CTime cTime;
				//	cTime = CTime::GetCurrentTime();
				//	int nCurrentHour = cTime.GetHour();
				//	int nCurrentMinute = cTime.GetMinute();
				//	int nCurrentSecond = cTime.GetSecond();
				//	CString strFile;
				//	strFile.Format("%s%s\\LockError\\%d-%d-%d-%d-%d-%d.jpg", OUTPUT_PATH, m_pRobot->m_strRobotName, cTime.GetYear(), cTime.GetMonth(), cTime.GetDay(), nCurrentHour, nCurrentMinute, nCurrentSecond);
				//	CString strFileFolder;
				//	XiBase::GetFileDirFromPath(strFile, strFileFolder);
				//	CheckFolder(strFileFolder);
				//	SaveImage(pCurImgInfo->img, strFile.GetBuffer());
				//	m_nLockFailTimes++;
				//	if(m_nLockFailTimes >= m_nLockFailTimesThre)
				//		return false;
				//}
			}
			writeLog("运行中锁定耗时：%dms", XiBase::XI_clock() - llLockTime);

			m_bPauseGetImage = false;

			pCurImgInfo->bGetKeyPntResult = false;
			pCurImgInfo->bGetKeyPntComplete = true;
			pCurImgInfo->bIsInvalid = true;
			bIsInvalid = false;
			bIsUsable = false;
			continue;
		}

		return pCurImgInfo;
	}

	return nullptr;
}

//RTT::RealTimeTrack::T_IMG_INFO* RTT::RealTimeTrack::waitForNextImgForProcess(T_PRO_IMG_THREAD_INFO& tProImgInfo)
//{
//	//仍在采图
//	while (!m_bIsGetImageEnd)
//	{
//		//更新正在处理的图片序号
//		tProImgInfo.nCurImgNo = tProImgInfo.nPreviousImgNo + m_nImgProcessThreadCount;
//
//		//如果未采集到需要的图
//		if (m_nNextGetImgNo <= tProImgInfo.nCurImgNo)
//		{
//			//等待一帧
//			Sleep(1000 / m_dMaxFrame);
//			continue;
//		}
//
//		//当前正在处理的图片数据指针
//		auto pCurImgInfo = m_vpImgInfo[tProImgInfo.nCurImgNo % m_nImgBufferCount];
//
//		//更新已经处理的图片序号
//		tProImgInfo.nPreviousImgNo = tProImgInfo.nCurImgNo;
//
//		//无效图片不处理
//		if (pCurImgInfo->bIsInvalid)
//		{
//			pCurImgInfo->bGetKeyPntResult = false;
//			pCurImgInfo->bGetKeyPntComplete = true;
//			continue;
//		}
//
//		return pCurImgInfo;
//	}
//
//	return nullptr;
//}

void RTT::RealTimeTrack::laserPnt2DTo3D(T_IMG_INFO& tImgInfo)
{
	T_ABS_POS_IN_BASE tPointAbsCoordInBase;
	tImgInfo.t3DKeyPoint = m_pUnit->TranImageToBase(
		m_pUnit->m_nTrackCameraNo, tImgInfo.t2DKeyPoint,
		tImgInfo.tRobotCoord, tImgInfo.tRobotPulse,
		&tPointAbsCoordInBase);

	//tImgInfo.t3DKeyPoint.dX += tImgInfo.tRobotCoord.dBX;
	//tImgInfo.t3DKeyPoint.dY += tImgInfo.tRobotCoord.dBY;
	//tImgInfo.t3DKeyPoint.dZ += tImgInfo.tRobotCoord.dBZ;
}

void RTT::RealTimeTrack::adjustTrack()
{
	//初始化变量
	int nNextImgNo = 0;//下一次被使用图片的序号

	//输入点保存文件
	CString sAdjustInputPntFile = getDataFileRootFolder() + "AdjustInputPntFile.txt";
	FILE* file = fopen(sAdjustInputPntFile, "w");
	fclose(file);
	file = fopen(sAdjustInputPntFile, "a+");

	//持续处理
	while (true)
	{
		//等待获取有效图片
		auto pCurImgInfo = waitForNextImgForAdjustTrack(nNextImgNo);
		if (!pCurImgInfo)
		{
			break;
		}

		//计算轨迹法向角
		double dTrackVerDegree = m_cAdjustTrack.calcNearestPointNormalDegree(pCurImgInfo->t3DKeyPoint);

		//补偿交点
		T_ROBOT_COORS tNewCoord = pCurImgInfo->t3DKeyPoint;
		tNewCoord.dX += m_dNormalComp * CosD(dTrackVerDegree);
		tNewCoord.dY += m_dNormalComp * SinD(dTrackVerDegree);
		tNewCoord.dZ += m_dHeightComp;

		writeLog("跟踪交点：%.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf",
			tNewCoord.dX, tNewCoord.dY, tNewCoord.dZ,
			tNewCoord.dRX, tNewCoord.dRY, tNewCoord.dRZ,
			tNewCoord.dBX, tNewCoord.dBY, tNewCoord.dBZ);
		fprintf(file, "%.3lf %.3lf %.3lf \n",
			tNewCoord.dX + tNewCoord.dBX,
			tNewCoord.dY + tNewCoord.dBY,
			tNewCoord.dZ + tNewCoord.dBZ);

		//如果没有暂停修正
		if (!m_bPauseAdjustTrack)
		{
			//修正轨迹
			m_cAdjustTrack.adjustTrack(pCurImgInfo->tRobotCoord, tNewCoord);
		}

		//距离理论终点一定距离，开始采集终点点云
		if (!m_bIsStartGetPntCloud //未开启点云采集
			&& m_cAdjustTrack.supportUpdatingEndPntOrNot() //支持更新终点
			&& m_dDisOfGetPntCloud > TwoPointDis(m_tTheoryEndPnt.x, m_tTheoryEndPnt.y, tNewCoord.dX + tNewCoord.dBX, tNewCoord.dY + tNewCoord.dBY) //距离理论终点一定距离
			)
		{
			writeLog("开启终点点云采集");
			m_bIsStartGetPntCloud = true;
		}

		//刷新状态
		pCurImgInfo->bIsUsedByAdjustTrack = true;
	}

	fclose(file);

	//退出
	m_bIsAdjustTrackEnd = true;
}

RTT::RealTimeTrack::T_IMG_INFO* RTT::RealTimeTrack::waitForNextImgForAdjustTrack(int& nNextImgNo)
{
	int nFirstSearchImgNo = nNextImgNo;//第一张搜索的图片序号
	T_IMG_INFO* pRtnImgInfo = nullptr;//此函数返回的新的未使用图片

	//仍在采图
	while (!m_bIsGetImageEnd)
	{
		Sleep(10);

		//下次采集图片的序号
		int nNextGetImgNo = m_nNextGetImgNo;

		//只查找一半缓存内容
		int nMinImgNo = nNextGetImgNo - m_nImgBufferCount / 2;
		nFirstSearchImgNo = nFirstSearchImgNo > nMinImgNo ? nFirstSearchImgNo : nMinImgNo;
		writeLog("waitForNextImgForAdjustTrack:%d", nFirstSearchImgNo);

		//查找是否连续处理失败
		int nGetKeyPntFailCount = 0;//连续提取交点失败次数
		int nFirstCheckNo = nNextGetImgNo - 10 - m_nGetKeyPntFailThre;
		if (nFirstCheckNo < 0) 
			nFirstCheckNo = 0;
		for (size_t i = nFirstCheckNo; i < nNextGetImgNo; i++)
		{
			//当前正在查找的图片数据
			auto& pCurImgInfo = m_vpImgInfo[i % m_nImgBufferCount];
			if (pCurImgInfo->bGetKeyPntComplete
				&& !pCurImgInfo->bGetKeyPntResult)
			{
				nGetKeyPntFailCount++;
				if (nGetKeyPntFailCount > m_nGetKeyPntFailThre)
				{
					emg();
					XUI::MesBox::PopOkCancel("跟踪：提取交点连续失败次数超过{0}次！", m_nGetKeyPntFailThre);
					return nullptr;
				}
			}
			else
			{
				nGetKeyPntFailCount = 0;
			}
		}

		//倒序找出最新的可用图片
		for (int i = nNextGetImgNo - 1; i >= nFirstSearchImgNo; i--)
		{
			//当前正在查找的图片数据
			auto& pCurImgInfo = m_vpImgInfo[i % m_nImgBufferCount];

			//已经被用于修正轨迹，结束查找
			if (pCurImgInfo->bIsUsedByAdjustTrack)
			{
				//m_nGetKeyPntFailCount = 0;
				nFirstSearchImgNo = i + 1;
				break;
			}

			//无效数据，继续查找
			if (pCurImgInfo->bIsInvalid)
			{
				//无效且未曾获取交点，说明不是处理失败
				//if (!pCurImgInfo->bGetKeyPntComplete)
				//{
					//m_nGetKeyPntFailCount = 0;
					//if (i < nNextGetImgNo - 11)
					//	writeLog("无效且未曾获取交点%d", i);
				//}
				continue;
			}

			//未获取交点，继续查找
			if (!pCurImgInfo->bGetKeyPntComplete)
			{
				//m_nGetKeyPntFailCount = 0;
				//if (i < nNextGetImgNo - 11)
				//	writeLog("未获取交点%d", i);
				continue;
			}

			//获取交点成功，结束查找
			if (pCurImgInfo->bGetKeyPntResult)
			{
				pRtnImgInfo = pCurImgInfo;
				nNextImgNo = i + 1;
				//m_nGetKeyPntFailCount = 0;
				writeLog("获取交点成功%d", i);
				//把未使用的数据全部设为已使用
				for (; nFirstSearchImgNo < i; nFirstSearchImgNo++)
				{
					m_vpImgInfo[nFirstSearchImgNo % m_nImgBufferCount]->bIsUsedByAdjustTrack = true;
				}
				nFirstSearchImgNo++;
				break;
			}
			//获取交点失败，继续查找
			else
			{
				pCurImgInfo->bIsInvalid = true;
				//m_nGetKeyPntFailCount++;
				//writeLog("m_nGetKeyPntFailCount=%d %d", m_nGetKeyPntFailCount, i);
			}
		}

		//如果找到新的未使用图片
		if (pRtnImgInfo)
		{
			return pRtnImgInfo;
		}

		//等待一帧
		Sleep(1000 / m_dMaxFrame);
	}

	return nullptr;
}

void RTT::RealTimeTrack::createImgBuffer()
{
	releaseImgBuffer();
	m_vpImgInfo.resize(m_nImgBufferCount);
	for (size_t i = 0; i < m_vpImgInfo.size(); i++)
	{
		m_vpImgInfo[i] = new T_IMG_INFO;
		m_vpImgInfo[i]->img = cvCreateImage(cvSize(m_pCamera->m_nImageWidth, m_pCamera->m_nImageHeight), IPL_DEPTH_8U, 1);
	}
}

void RTT::RealTimeTrack::releaseImgBuffer()
{
	for (size_t i = 0; i < m_vpImgInfo.size(); i++)
	{
		if (m_vpImgInfo[i])
		{
			if (m_vpImgInfo[i]->img)
			{
				cvReleaseImage(&(m_vpImgInfo[i]->img));
			}
			delete m_vpImgInfo[i];
		}
	}
	m_vpImgInfo.clear();
}

CString RTT::RealTimeTrack::getDataFileRootFolder() const
{
	return WELDDATA_PATH + m_pRobot->m_strRobotName + "\\Track\\";
}

void RTT::RealTimeTrack::saveImg(T_IMG_INFO& tImgInfo)
{
	CString strImg;
	strImg.Format(getDataFileRootFolder() + "Pic\\%d.bin", tImgInfo.nRealImgNo % 1000);
	char sImgFile[255];
	strcpy(sImgFile, strImg);
	BinaryImgSaveAsynchronous(sImgFile, tImgInfo.img, 0);
}

void RTT::RealTimeTrack::writeLog(const char* format, ...) const
{
	CString str;
	va_list args;
	__crt_va_start(args, format);
	str.FormatV(format, args);
	__crt_va_end(args);

	m_pRobot->m_cLog->Write("跟踪：" + str);
}
