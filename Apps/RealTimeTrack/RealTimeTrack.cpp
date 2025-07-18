#include "stdafx.h"
#include "RealTimeTrack.h"
#include ".\Apps\PLib\CtrlUnit\CUnit.h"
#include "Project/AssemblyWeld.h"

struct RTT::RealTimeTrack::T_IMG_INFO
{
	/*******************************�ɼ�����*******************************/
	IplImage* img = nullptr;						//ͼƬ��ַ
	int nRealImgNo = -1;							//��ʵͼƬ���
	bool bIsInvalid = false;						//�Ƿ�����Ч��ͼƬ����
	T_ROBOT_COORS tRobotCoord;						//������ֱ������
	T_ANGLE_PULSE tRobotPulse;						//��������������

	/*******************************��������*******************************/
	CvPoint tIdeal2DKeyPoint = {0};					//����2D���⽻��
	CvPoint tHorTrackingPointInImage = { 0 };		//���۵װ巽�򼤹��ά��
	CvPoint tVerTrackingPointInImage = { 0 };		//�������巽�򼤹��ά��

	CvPoint t2DKeyPoint;							//2D���⽻��
	T_ROBOT_COORS t3DKeyPoint;						//3D���⽻��
	bool bGetKeyPntResult = false;					//��ͼƬ��ý���ɹ�/ʧ��
	std::atomic<bool> bGetKeyPntComplete = false;	//�Ѵ�ͼƬ��ý��㣨�������ȡ�Ľ����Ƿ���Ч��

	/*******************************�켣����*******************************/
	std::atomic<bool> bIsUsedByAdjustTrack = false;	//�Ѿ����켣����ʹ��(������������)
};

struct RTT::RealTimeTrack::T_PRO_IMG_THREAD_INFO
{
	RealTimeTrack* pThis = nullptr;
	int nThreadNo = 0;								//���߳����
	int nPreviousImgNo = -1;						//�Ѿ������ͼƬ���
	int nCurImgNo = -1;								//���ڴ����ͼƬ���
	std::atomic<bool> bIsThreadEnd;					//�߳��Ƿ��Ѿ��˳�

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
		//��������б��
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
	//WidgetLaserInfo* pWidgetLaserInfo = nullptr;	//ͼ����������
	std::vector<WidgetLaserInfo> vtWidgetLaserInfo;	//ͼ����������
	//WidgetLaserTrackTool* pWidgetLaserTrack = nullptr;	//ͼ������ָ��
	int m_nLockInfoBufferSize = 32;								//�������ݻ����С
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
	//��ʼ�����ٵ���
	m_cAdjustTrack.setUnit(*m_pUnit);
	m_cAdjustTrack.initTrack(*m_pWeldTrack);
	return m_cAdjustTrack.initBeforeTrack();
}

void RTT::RealTimeTrack::startRun()
{
	if (!m_bEnable)
	{
		//���ò�ͼ���߳�״̬
		m_bEndGetImage = true;
		m_bPauseGetImage = true;
		m_bIsGetImageEnd = true;

		//����ͼƬ�������߳�״̬
		for (size_t i = 0; i < m_vpProImgThreadInfo.size(); i++)
		{
			//deletePointer(m_vpProImgThreadInfo[i]->pWidgetLaserInfo);
			//deletePointer(m_vpProImgThreadInfo[i]->pWidgetLaserTrack);
			deletePointer(m_vpProImgThreadInfo[i]);
		}
		m_vpProImgThreadInfo.clear();

		//���ù켣�������߳�״̬
		m_bIsAdjustTrackEnd = true;
		return;
	}

	//��ʼ��ͼ
	createImgBuffer();
	m_bEndGetImage = false;
	m_bPauseGetImage = false;
	m_bIsGetImageEnd = false;
	m_nNextGetImgNo = 0;
	m_nLockFailTimes = 0;
	m_bFirstLockComplete = false;
	AfxBeginThread(ThreadGetImg, this);

	//��ʼ����
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

	//��ʼ�������ӹ켣
	m_bPauseAdjustTrack = false;
	m_bIsAdjustTrackEnd = false;
	AfxBeginThread(ThreadAdjustTrack, this);

	//�յ��ж��߳�
	m_bIsStartGetPntCloud = false;
}

void RTT::RealTimeTrack::endRun()
{
	//ֹͣ��ͼ
	m_bEndGetImage = true;
}

void RTT::RealTimeTrack::waitUntilEnd()
{	
	bool bIsEnd = false;
	while (!bIsEnd)
	{
		Sleep(20);

		//��ͼ���߳�״̬
		bIsEnd = m_bIsGetImageEnd;

		//ͼ�������߳�״̬
		for (size_t i = 0; i < m_vpProImgThreadInfo.size(); i++)
		{
			bIsEnd = bIsEnd && m_vpProImgThreadInfo[i]->bIsThreadEnd;
		}

		//�켣�������߳�״̬
		bIsEnd = bIsEnd && m_bIsAdjustTrackEnd;

		//�յ��ж����߳�״̬

	}
}

void RTT::RealTimeTrack::emg()
{
	//��������
	endRun();

	//ֹͣ�����˶�
	m_pRobot->m_eThreadStatus = INCISEHEAD_THREAD_STATUS_STOPPED;
	m_pRobot->HoldOn();
	m_pRobot->ServoOff();
	//���޸�
	XUI::MesBox::PopInfo("���٣�{0}��е�ۼ�ͣ", m_pRobot->m_strRobotName.GetBuffer());
	//XiMessageBox("���٣�%s ��е�ۼ�ͣ", m_pRobot->m_strRobotName);
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
	long long llStartTime = XI_clock();//��ʼ��ͼʱ��
	long long llLastTime = llStartTime;//��һ�β�ͼʱ��
	int nPauseTimes = 0;//��ͣѭ������

	while (!m_bEndGetImage)
	{
		//�����ͣ��ͼ
		if (m_bPauseGetImage)
		{
			Sleep(1000);
			nPauseTimes++;
		}

		//�������֡��
		long long llWaitTime = llLastTime + (1000 / m_dMaxFrame + 1) - XI_clock();
		if (llWaitTime > 0)
		{
			Sleep(llWaitTime);
		}
		llLastTime = XI_clock();

		//��Ҫ�ɼ���ͼƬ���ݻ�����
		T_IMG_INFO& tNewEndPointInfo = *(m_vpImgInfo[m_nNextGetImgNo % m_nImgBufferCount]);

		//�����һ�ε�ͼ��û����
		if (m_nNextGetImgNo >= m_nImgBufferCount
			&& !tNewEndPointInfo.bIsUsedByAdjustTrack
			&& !tNewEndPointInfo.bIsInvalid)
		{
			emg();
			XiMessageBox("���٣�ͼ���ܼ�ʱ����");
			break;
		}

		//��ȡͼƬ
		if (!m_pCamera->CaptureImage(tNewEndPointInfo.img, 1))
		{
			emg();
			XiMessageBox("���٣���ͼʧ�ܣ�");
			break;
		}
		tNewEndPointInfo.nRealImgNo = m_nNextGetImgNo;

		//��ȡ����
		tNewEndPointInfo.bGetKeyPntResult = false;
		tNewEndPointInfo.bGetKeyPntComplete = false;
		tNewEndPointInfo.bIsUsedByAdjustTrack = false;
		tNewEndPointInfo.tRobotCoord = m_pRobot->GetCurrentPos();
		tNewEndPointInfo.tRobotPulse = m_pRobot->GetCurrentPulse();

		//�������ЧͼƬ�����ͼ
		if (m_cAdjustTrack.determineWhetherToProcessImg(tNewEndPointInfo.tRobotCoord, 
			tNewEndPointInfo.tIdeal2DKeyPoint, tNewEndPointInfo.tHorTrackingPointInImage, tNewEndPointInfo.tVerTrackingPointInImage))
		{
			long long llTime2 = XI_clock();
			saveImg(tNewEndPointInfo);
			auto llSaveTime = XI_clock() - llTime2;
			if (llSaveTime > 20)
			{
				writeLog("����ͼƬ������ʱ��%dms", llSaveTime);
			}
			else
			{
				writeLog("����ͼƬ��ʱ��%dms", llSaveTime);
			}
			tNewEndPointInfo.bIsInvalid = false;
		}
		//��ЧͼƬ
		else
		{
			tNewEndPointInfo.bGetKeyPntResult = false;
			tNewEndPointInfo.bGetKeyPntComplete = false;
			tNewEndPointInfo.bIsInvalid = true;
		}
		//��������
		writeLog("����ͼƬ%d����ͼƬ%s", (int)m_nNextGetImgNo,
			tNewEndPointInfo.bIsInvalid ? "��Ч" : "��Ч");
		m_nNextGetImgNo++;
	}

	//�˳�����
	m_bIsGetImageEnd = true;
	writeLog("��β��ͼ�߳��˳���֡�ʣ�%.3lf",
		m_nNextGetImgNo * 1000.0 / double(XI_clock() - llStartTime - 1000.0 * nPauseTimes));
}

void RTT::RealTimeTrack::processImage(T_PRO_IMG_THREAD_INFO& tProImgInfo)
{
	//��ʼ������
	int nProcessImgCount = 0;//����ͼƬ����
	long long llProcessTime = 0;//������ʱ��
	tProImgInfo.nPreviousImgNo = tProImgInfo.nThreadNo - m_nImgProcessThreadCount;//��֤��һ��ͼ���߳����

	//��������ͼƬ
	while (true)
	{
		//�ȴ���ȡ��ЧͼƬ
		auto pCurImgInfo = waitForNextImgForProcess(tProImgInfo);
		if (!pCurImgInfo)
		{
			break;
		}

		//��ȡ���⽻��
		long long llTime1 = XI_clock();
		auto pWidgetLaserInfo = tProImgInfo.getLastLockInfo();
		writeLog("m_pLaserInfo ��Σ�%d %d %d %d %d %d ", pWidgetLaserInfo->crossPoint.x, pWidgetLaserInfo->crossPoint.y,
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
			writeLog("ͼƬ%d����ʧ��", pCurImgInfo->nRealImgNo);
			pCurImgInfo->bGetKeyPntComplete = true;
			continue;
		}
		pCurImgInfo->t2DKeyPoint = pWidgetLaserInfo->crossPoint;

		writeLog("ͼƬ%d�����۶�ά�㣺%d-%d��ʵ�ʶ�ά�㣺%d-%d", pCurImgInfo->nRealImgNo,
			pCurImgInfo->tIdeal2DKeyPoint.x, pCurImgInfo->tIdeal2DKeyPoint.y,
			pCurImgInfo->t2DKeyPoint.x, pCurImgInfo->t2DKeyPoint.y);

		//�ɼ�����
		if (m_bIsStartGetPntCloud)
		{

		}

		//��άת��ά
		laserPnt2DTo3D(*pCurImgInfo);
		long long llTime5 = XI_clock();
		llProcessTime = llProcessTime + llTime5 - llTime1;

		//��ʾ
		if (pCurImgInfo->nRealImgNo % (m_nMinShowImgSampleGap + 1) == 0)
		{
			auto llShowTime = XiBase::XI_clock();
			IplImage* pColorImg = cvCreateImage(cvSize(pCurImgInfo->img->width, pCurImgInfo->img->height), IPL_DEPTH_8U, 3);
			cvCvtColor(pCurImgInfo->img, pColorImg, CV_GRAY2RGB);
			cvCircle(pColorImg, pCurImgInfo->t2DKeyPoint, 10, CV_RGB(255, 0, 0), 3);
			((CAssemblyWeld*)getMainDlg())->ShowTeachImage(pColorImg);
			cvReleaseImage(&pColorImg);
			writeLog("��ʾ��ʱ��%dms", XiBase::XI_clock() - llShowTime);
		}

		//ˢ��״̬
		pCurImgInfo->bIsInvalid = false;
		pCurImgInfo->bGetKeyPntComplete = true;
	}
	writeLog("�߳�%d����ͼƬƽ����ʱ��%.2lfms",
		tProImgInfo.nThreadNo,
		llProcessTime / (nProcessImgCount * 1.0));
	tProImgInfo.bIsThreadEnd = true;
}

RTT::RealTimeTrack::T_IMG_INFO* RTT::RealTimeTrack::waitForNextImgForProcess(T_PRO_IMG_THREAD_INFO& tProImgInfo)
{
	bool bIsInvalid = false;
	bool bIsUsable = false;
	bool bLockFail = false;

	//���ڲ�ͼ
	while (!m_bIsGetImageEnd)
	{
		Sleep(10);

		//�������ڴ����ͼƬ���
		tProImgInfo.nCurImgNo = tProImgInfo.nPreviousImgNo + m_nImgProcessThreadCount;

		//���δ�ɼ�����Ҫ��ͼ
		if (m_nNextGetImgNo <= tProImgInfo.nCurImgNo)
		{
			//�ȴ�һ֡
			Sleep(1000 / m_dMaxFrame);
			continue;
		}

		//��ǰ���ڴ����ͼƬ����ָ��
		auto pCurImgInfo = m_vpImgInfo[tProImgInfo.nCurImgNo % m_nImgBufferCount];

		//�����Ѿ������ͼƬ���
		tProImgInfo.nPreviousImgNo = tProImgInfo.nCurImgNo;

		//��ЧͼƬ������
		if (pCurImgInfo->bIsInvalid)
		{
			bIsInvalid = true;
			pCurImgInfo->bGetKeyPntResult = false;
			pCurImgInfo->bGetKeyPntComplete = false;
			continue;
		}
		bIsUsable = true;

		//0���̻߳�û�����������������߳�����Ч����Ч��ת��
		if (tProImgInfo.nThreadNo == 0 && bIsUsable && (!m_bFirstLockComplete || bIsInvalid || bLockFail))
		{
			m_bPauseGetImage = true;
			Sleep(300);

			writeLog("�����п�ʼ����б��");
			auto llLockTime = XiBase::XI_clock();
			//ʹ������б����ȡ
			{
				m_pWidgetLaserInfo->crossPoint = pCurImgInfo->tIdeal2DKeyPoint;
				m_pWidgetLaserInfo->bottomPlatePnt = pCurImgInfo->tHorTrackingPointInImage;
				m_pWidgetLaserInfo->verticalPlatePnt = pCurImgInfo->tVerTrackingPointInImage;

				//��ȡ���⽻��
				long long llTime1 = XI_clock();
				writeLog("ͼƬ%d������m_pLaserInfo ��Σ�%d %d %d %d %d %d ", pCurImgInfo->nRealImgNo,
					m_pWidgetLaserInfo->crossPoint.x, m_pWidgetLaserInfo->crossPoint.y,
					m_pWidgetLaserInfo->verticalPlatePnt.x, m_pWidgetLaserInfo->verticalPlatePnt.y,
					m_pWidgetLaserInfo->bottomPlatePnt.x, m_pWidgetLaserInfo->bottomPlatePnt.y);

				//��������ͼ
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
					XiMessageBoxOk("��ʼ�Ե㣡(������룺�Ҳ����ؼ���)");
					return false;
				}

				writeLog("ͼƬ%d������m_pLaserInfo ���Σ�%d %d %d %d %d %d ", pCurImgInfo->nRealImgNo,
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
			//��������б��
			{
				////����
				//CvPoint cpMidKeyPoint;//����
				//WidgetLaserInfo* pWidgetLaserInfo = new WidgetLaserInfo[10];
				//int nRst = WidgetLaserLock(pCurImgInfo->img, pWidgetLaserInfo, false, false, "WidgetLaserLock");

				////Ѱ�ҷ���Ҫ��Ľ���
				//int nFindNo = -1;//����Ҫ��Ľ���
				//double dMinDis = 20.0;//�����ľ���
				//for (int i = 0; i < nRst; i++)
				//{
				//	//���㽻����ά����
				//	T_ABS_POS_IN_BASE tPointAbsCoordInBase;
				//	T_ROBOT_COORS tAimCoord = m_pUnit->TranImageToBase(
				//		m_pUnit->m_nTrackCameraNo, pWidgetLaserInfo[i].crossPoint,
				//		pCurImgInfo->tRobotCoord, pCurImgInfo->tRobotPulse,
				//		&tPointAbsCoordInBase);

				//	//�ҵ����������
				//	T_ROBOT_COORS tNearestCoord;
				//	if (m_pWeldTrack->getNearestCoordFast(tNearestCoord, tAimCoord, dMinDis, 0))
				//	{
				//		double dTemp = SQUARE(tNearestCoord.dX + tNearestCoord.dBX - tAimCoord.dX - tAimCoord.dBX)
				//			+ SQUARE(tNearestCoord.dY + tNearestCoord.dBY - tAimCoord.dY - tAimCoord.dBY)
				//			+ SQUARE(tNearestCoord.dZ + tNearestCoord.dBZ - tAimCoord.dZ - tAimCoord.dBZ);
				//		dMinDis = sqrt(dTemp);//�����������
				//		cpMidKeyPoint = pWidgetLaserInfo[i].crossPoint;//���½���
				//		setLaserLockResult(pWidgetLaserInfo[i]);//������������
				//		for (size_t k = 0; k < m_vpProImgThreadInfo.size(); k++)
				//		{
				//			memcpy(m_vpProImgThreadInfo[k]->pWidgetLaserInfo, m_pWidgetLaserInfo, sizeof(WidgetLaserInfo));
				//		}
				//		nFindNo = i;//�������
				//	}
				//}
				//DELETE_POINTER_ARRAY(pWidgetLaserInfo);

				////û�ҵ����ʵĽ���
				//bLockFail = nRst <= 0 || nFindNo < 0;
				//if (bLockFail)
				//{
				//	emg();
				//	XiMessageBoxOk("��ʼ�Ե㣡(������룺�Ҳ����ؼ���)");
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
			writeLog("������������ʱ��%dms", XiBase::XI_clock() - llLockTime);

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
//	//���ڲ�ͼ
//	while (!m_bIsGetImageEnd)
//	{
//		//�������ڴ����ͼƬ���
//		tProImgInfo.nCurImgNo = tProImgInfo.nPreviousImgNo + m_nImgProcessThreadCount;
//
//		//���δ�ɼ�����Ҫ��ͼ
//		if (m_nNextGetImgNo <= tProImgInfo.nCurImgNo)
//		{
//			//�ȴ�һ֡
//			Sleep(1000 / m_dMaxFrame);
//			continue;
//		}
//
//		//��ǰ���ڴ����ͼƬ����ָ��
//		auto pCurImgInfo = m_vpImgInfo[tProImgInfo.nCurImgNo % m_nImgBufferCount];
//
//		//�����Ѿ������ͼƬ���
//		tProImgInfo.nPreviousImgNo = tProImgInfo.nCurImgNo;
//
//		//��ЧͼƬ������
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
	//��ʼ������
	int nNextImgNo = 0;//��һ�α�ʹ��ͼƬ�����

	//����㱣���ļ�
	CString sAdjustInputPntFile = getDataFileRootFolder() + "AdjustInputPntFile.txt";
	FILE* file = fopen(sAdjustInputPntFile, "w");
	fclose(file);
	file = fopen(sAdjustInputPntFile, "a+");

	//��������
	while (true)
	{
		//�ȴ���ȡ��ЧͼƬ
		auto pCurImgInfo = waitForNextImgForAdjustTrack(nNextImgNo);
		if (!pCurImgInfo)
		{
			break;
		}

		//����켣�����
		double dTrackVerDegree = m_cAdjustTrack.calcNearestPointNormalDegree(pCurImgInfo->t3DKeyPoint);

		//��������
		T_ROBOT_COORS tNewCoord = pCurImgInfo->t3DKeyPoint;
		tNewCoord.dX += m_dNormalComp * CosD(dTrackVerDegree);
		tNewCoord.dY += m_dNormalComp * SinD(dTrackVerDegree);
		tNewCoord.dZ += m_dHeightComp;

		writeLog("���ٽ��㣺%.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf",
			tNewCoord.dX, tNewCoord.dY, tNewCoord.dZ,
			tNewCoord.dRX, tNewCoord.dRY, tNewCoord.dRZ,
			tNewCoord.dBX, tNewCoord.dBY, tNewCoord.dBZ);
		fprintf(file, "%.3lf %.3lf %.3lf \n",
			tNewCoord.dX + tNewCoord.dBX,
			tNewCoord.dY + tNewCoord.dBY,
			tNewCoord.dZ + tNewCoord.dBZ);

		//���û����ͣ����
		if (!m_bPauseAdjustTrack)
		{
			//�����켣
			m_cAdjustTrack.adjustTrack(pCurImgInfo->tRobotCoord, tNewCoord);
		}

		//���������յ�һ�����룬��ʼ�ɼ��յ����
		if (!m_bIsStartGetPntCloud //δ�������Ʋɼ�
			&& m_cAdjustTrack.supportUpdatingEndPntOrNot() //֧�ָ����յ�
			&& m_dDisOfGetPntCloud > TwoPointDis(m_tTheoryEndPnt.x, m_tTheoryEndPnt.y, tNewCoord.dX + tNewCoord.dBX, tNewCoord.dY + tNewCoord.dBY) //���������յ�һ������
			)
		{
			writeLog("�����յ���Ʋɼ�");
			m_bIsStartGetPntCloud = true;
		}

		//ˢ��״̬
		pCurImgInfo->bIsUsedByAdjustTrack = true;
	}

	fclose(file);

	//�˳�
	m_bIsAdjustTrackEnd = true;
}

RTT::RealTimeTrack::T_IMG_INFO* RTT::RealTimeTrack::waitForNextImgForAdjustTrack(int& nNextImgNo)
{
	int nFirstSearchImgNo = nNextImgNo;//��һ��������ͼƬ���
	T_IMG_INFO* pRtnImgInfo = nullptr;//�˺������ص��µ�δʹ��ͼƬ

	//���ڲ�ͼ
	while (!m_bIsGetImageEnd)
	{
		Sleep(10);

		//�´βɼ�ͼƬ�����
		int nNextGetImgNo = m_nNextGetImgNo;

		//ֻ����һ�뻺������
		int nMinImgNo = nNextGetImgNo - m_nImgBufferCount / 2;
		nFirstSearchImgNo = nFirstSearchImgNo > nMinImgNo ? nFirstSearchImgNo : nMinImgNo;
		writeLog("waitForNextImgForAdjustTrack:%d", nFirstSearchImgNo);

		//�����Ƿ���������ʧ��
		int nGetKeyPntFailCount = 0;//������ȡ����ʧ�ܴ���
		int nFirstCheckNo = nNextGetImgNo - 10 - m_nGetKeyPntFailThre;
		if (nFirstCheckNo < 0) 
			nFirstCheckNo = 0;
		for (size_t i = nFirstCheckNo; i < nNextGetImgNo; i++)
		{
			//��ǰ���ڲ��ҵ�ͼƬ����
			auto& pCurImgInfo = m_vpImgInfo[i % m_nImgBufferCount];
			if (pCurImgInfo->bGetKeyPntComplete
				&& !pCurImgInfo->bGetKeyPntResult)
			{
				nGetKeyPntFailCount++;
				if (nGetKeyPntFailCount > m_nGetKeyPntFailThre)
				{
					emg();
					XUI::MesBox::PopOkCancel("���٣���ȡ��������ʧ�ܴ�������{0}�Σ�", m_nGetKeyPntFailThre);
					return nullptr;
				}
			}
			else
			{
				nGetKeyPntFailCount = 0;
			}
		}

		//�����ҳ����µĿ���ͼƬ
		for (int i = nNextGetImgNo - 1; i >= nFirstSearchImgNo; i--)
		{
			//��ǰ���ڲ��ҵ�ͼƬ����
			auto& pCurImgInfo = m_vpImgInfo[i % m_nImgBufferCount];

			//�Ѿ������������켣����������
			if (pCurImgInfo->bIsUsedByAdjustTrack)
			{
				//m_nGetKeyPntFailCount = 0;
				nFirstSearchImgNo = i + 1;
				break;
			}

			//��Ч���ݣ���������
			if (pCurImgInfo->bIsInvalid)
			{
				//��Ч��δ����ȡ���㣬˵�����Ǵ���ʧ��
				//if (!pCurImgInfo->bGetKeyPntComplete)
				//{
					//m_nGetKeyPntFailCount = 0;
					//if (i < nNextGetImgNo - 11)
					//	writeLog("��Ч��δ����ȡ����%d", i);
				//}
				continue;
			}

			//δ��ȡ���㣬��������
			if (!pCurImgInfo->bGetKeyPntComplete)
			{
				//m_nGetKeyPntFailCount = 0;
				//if (i < nNextGetImgNo - 11)
				//	writeLog("δ��ȡ����%d", i);
				continue;
			}

			//��ȡ����ɹ�����������
			if (pCurImgInfo->bGetKeyPntResult)
			{
				pRtnImgInfo = pCurImgInfo;
				nNextImgNo = i + 1;
				//m_nGetKeyPntFailCount = 0;
				writeLog("��ȡ����ɹ�%d", i);
				//��δʹ�õ�����ȫ����Ϊ��ʹ��
				for (; nFirstSearchImgNo < i; nFirstSearchImgNo++)
				{
					m_vpImgInfo[nFirstSearchImgNo % m_nImgBufferCount]->bIsUsedByAdjustTrack = true;
				}
				nFirstSearchImgNo++;
				break;
			}
			//��ȡ����ʧ�ܣ���������
			else
			{
				pCurImgInfo->bIsInvalid = true;
				//m_nGetKeyPntFailCount++;
				//writeLog("m_nGetKeyPntFailCount=%d %d", m_nGetKeyPntFailCount, i);
			}
		}

		//����ҵ��µ�δʹ��ͼƬ
		if (pRtnImgInfo)
		{
			return pRtnImgInfo;
		}

		//�ȴ�һ֡
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

	m_pRobot->m_cLog->Write("���٣�" + str);
}
