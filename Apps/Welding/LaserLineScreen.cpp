#include "stdafx.h"
#include "LaserLineScreen.h"
#include "RiserManagement.h"


CLaserLineScreen::CLaserLineScreen(CUnit* pUnit, BOOL *pbNaturalPop):
	m_ptUnit(pUnit),
	m_pRobotDriver(pUnit->GetRobotCtrl()),
	m_pLaserCamera(pUnit->GetDHCameraCtrl(0)),
	m_pbNaturalPop(pbNaturalPop)
{
	m_bComplete = true;
	m_vtPointCloud.clear();
	//m_vtpLaserCamera = m_ptUnit->m_vpImageCapture;
	for (int nLineCamNo  = m_ptUnit->m_nLineScanCameraNo; nLineCamNo  < m_ptUnit->m_vpImageCapture.size(); nLineCamNo ++)
	{
		m_vtpLaserCamera.push_back(m_ptUnit->m_vpImageCapture[nLineCamNo]);
	}
	m_vvtPointCloud.resize(LINESCAN_CAMERA_NUM);
	m_vbComplete.resize(LINESCAN_CAMERA_NUM);			// ��ɨ����״̬
	m_vbProcessStartMark.resize(LINESCAN_CAMERA_NUM);	//����ִ���̱߳�־
	m_vbStartHandleMark.resize(LINESCAN_CAMERA_NUM);	//��ʼ�����־
	m_vbScanMark.resize(LINESCAN_CAMERA_NUM);			//�ɼ���־
	m_nThreadNoNew.resize(LINESCAN_CAMERA_NUM);		//����ִ���̱߳��(�����߳�����ʱʹ��)
	m_vnDrawingNum.resize(LINESCAN_CAMERA_NUM);			//�ɼ�ͼƬ����
	m_vnProcessNum.resize(LINESCAN_CAMERA_NUM);			//����ͼƬ����
	m_vnThreadNoNew.resize(LINESCAN_CAMERA_NUM);		// �������̱߳�ű�
	m_vhParallelProcessingSem.resize(LINESCAN_CAMERA_NUM);
	LoadLineScanParam(0);
}

CLaserLineScreen::~CLaserLineScreen()
{
	m_ptUnit = NULL;
	m_pRobotDriver = NULL;
	m_pLaserCamera = NULL;
	m_pbNaturalPop = NULL;
}

bool CLaserLineScreen::Start()
{
	if (false == m_bComplete)
	{
		//���޸�
		XUI::MesBox::PopInfo("{0}��ɨ������,�����ظ�������", m_pRobotDriver->m_strRobotName.GetBuffer());
		//XiMessageBox("%s ��ɨ������,�����ظ�������", m_pRobotDriver->m_strRobotName);
		return false;
	}
	m_bComplete = false;
	bool bRst = LaserScanRuning() == TRUE;
	m_bComplete = true;
	return bRst;
}

CvPoint3D64f* CLaserLineScreen::GetPointCloudData(int &nPtnNum)
{
	if (false == m_bComplete || m_vtPointCloud.size() <= 0)
	{
		nPtnNum = 0;
		return NULL;
	}
	nPtnNum = m_vtPointCloud.size();
	return m_vtPointCloud.data();
}

void CLaserLineScreen::ReleasePointCloud()
{
	if (false == m_bComplete)
	{
		XUI::MesBox::PopOkCancel("{0} ��ɨ������,�����ͷŵ������ݣ�", m_pRobotDriver->m_strRobotName);
		return;
	}
	m_vtPointCloud.clear();
}

UINT CLaserLineScreen::ThreadSchedulingFunction(void* pParam)
{
	CLaserLineScreen* pObj = (CLaserLineScreen*)pParam;
	pObj->SchedulingFunction();
	return 0;
}

UINT CLaserLineScreen::ThreadExecutiveFunction(void* pParam)
{
	CLaserLineScreen* pObj = (CLaserLineScreen*)pParam;
	pObj->ExecutiveFunction(pObj->m_nThreadNo);
	return 0;
}

UINT CLaserLineScreen::ThreadSchedulingFunctionNew(void* pParam)
{
	T_GET_LINESCAN_PHOTO* pObj = (T_GET_LINESCAN_PHOTO*)pParam;
	pObj->pcLineScan->SchedulingFunction(pObj->nCameraNo, pObj->nScanNum);
	return 0;
}

UINT CLaserLineScreen::ThreadExecutiveFunctionNew(void* pParam)
{
	T_GET_LINESCAN_PHOTO* pObj = (T_GET_LINESCAN_PHOTO*)pParam;
	pObj->pcLineScan->ExecutiveFunction(pObj->pcLineScan->m_nThreadNoNew[pObj->nCameraNo], pObj->nCameraNo);
	return 0;
}


void CLaserLineScreen::SchedulingFunction()
{
	int nRobotNo = m_pRobotDriver->m_nRobotNo;
	BOOL bRecordTime = TRUE;

	m_vnThreadNo.clear();
	m_bProcessStartMark = TRUE;
	m_bStartHandleMark = TRUE;
	m_nDrawingNum = 0;
	m_nProcessNum = 0;

	m_hParallelProcessingSem = CreateSemaphore(NULL, MAX_PROCESS_THREAD_NUM, MAX_PROCESS_THREAD_NUM, m_pRobotDriver->m_strRobotName);
	if (m_hParallelProcessingSem == NULL)
	{
		XiMessageBox("������������ʧ�ܣ�");
		return;
	}
	for (int nNum = 0; nNum < MAX_PROCESS_THREAD_NUM; nNum++)
	{
		CvSize ImageSize;
		ImageSize.width = m_ptUnit->m_vtCameraPara[m_ptUnit->m_nLineScanCameraNo].tDHCameraDriverPara.nRoiWidth;
		ImageSize.height = m_ptUnit->m_vtCameraPara[m_ptUnit->m_nLineScanCameraNo].tDHCameraDriverPara.nRoiHeight;
		IplImage* cvImage = cvCreateImage(ImageSize, IPL_DEPTH_8U, 1);
		m_vtImgCollectDataBuff[nNum].tImageData.ptImg = cvImage;
		m_eThreadStateMark[nNum] = PROCESS_THREAD_FREE;
	}
	m_pRobotDriver->m_cLog->Write("���д�����ʼ�����");

	FILE* fRecordTimeInfo;
	CString sPointCloudFile = OUTPUT_PATH + m_ptUnit->m_tContralUnit.strUnitName + "\\" + POINT_CLOUD_FILE;
	CheckFolder(OUTPUT_PATH + m_ptUnit->m_tContralUnit.strUnitName + "\\" + POINT_CLOUD_IMAGE);
	bRecordTime = !XI_fopen_s(&fRecordTimeInfo, sPointCloudFile.GetBuffer(), "w");

	if (bRecordTime == FALSE)
	{
		m_pRobotDriver->m_cLog->Write("�� HandleScanDataTimeInfo.txt �ļ�ʧ��, ���رռ�¼����ʱ�书��");
	}

	for (int nNum = 0; nNum < MAX_PROCESS_THREAD_NUM; nNum++)
	{
		m_nThreadNo = nNum;

		AfxBeginThread(ThreadExecutiveFunction, this);
		int n = 3;
		while (m_nThreadNo != -1)
		{
			//CHECK_STOP_BREAK(m_pRobotDriver);
			Sleep(10);
			DoEvent();
			n--;
			if (n < 0)
			{
				m_pRobotDriver->m_cLog->Write("�����߳� %d ʧ��", nNum);
				break;
			}
		}
	}
	int nProcessedNum = 0;
	int oldnProcessedNum = 0;
	int Comsize = 0;
	bool bExitHandleCondition(FALSE);	//�Ƿ��˳��ɷ��߳�����2
	while (TRUE == m_bStartHandleMark)
	{
		//----------------------- ������ִ���߳� ------------------//
		m_pRobotDriver->m_cLog->Write("������");
		WaitForSingleObject(m_hParallelProcessingSem, INFINITE);
		int nCurrentFreeThreadNo = GetFreeThreadNo(nRobotNo);
		if (nCurrentFreeThreadNo == -1)
		{
			ReleaseSemaphore(m_hParallelProcessingSem, 1, NULL);
			Sleep(1);
			m_pRobotDriver->m_cLog->Write("������");
			continue;
		}
		m_pRobotDriver->m_cLog->Write("����ִ���߳�  %d", nCurrentFreeThreadNo);
		m_pRobotDriver->m_cLog->Write("���洦����");

		//----------------------- ���洦���� -----------------------//
		SaveLineScanResult(fRecordTimeInfo);
		m_nDrawingNum = GetCallBackImageNum();
		m_pRobotDriver->m_cLog->Write("�жϴ������ ͼ����%d", m_nDrawingNum);

		//----------------------- �жϴ������ -----------------------//
		if (m_nDrawingNum <= nProcessedNum)
		{
			if (oldnProcessedNum == m_nDrawingNum)
			{
				Comsize++;
			}
			else
			{
				oldnProcessedNum = m_nDrawingNum;
				Comsize = 0;
			}

			m_pRobotDriver->m_cLog->Write("�ɼ�ͼƬ ͼ����%d������ͼƬ ͼ����%d", m_nDrawingNum, nProcessedNum);
			if (TRUE == CheckThreadFree(nRobotNo) && FALSE == m_bScanMark)
			{
				ReleaseSemaphore(m_hParallelProcessingSem, 1, NULL);
				m_pRobotDriver->m_cLog->Write("�˳�");
				break;
			}
			Sleep(5);
			ReleaseSemaphore(m_hParallelProcessingSem, 1, NULL);

			if (Comsize >= 20)
			{
				m_pRobotDriver->m_cLog->Write("�˳�");
				//XiMessageBox("%s ��ɨ�����쳣������������������ɨ", pRobotDriver->m_strRobotName);
				//OnBtnStop(pRobotDriver);
				break;
			}

			m_pRobotDriver->m_cLog->Write("����");
			continue;
		}
		else
		{
			bExitHandleCondition = FALSE;
			if (CheckCallBackImage(nProcessedNum) == FALSE)
			{
				m_pRobotDriver->m_cLog->Write("�� %d �δ�������쳣�������˴δ���", nProcessedNum);
				nProcessedNum++;
				ReleaseSemaphore(m_hParallelProcessingSem, 1, NULL);
				continue;
			}
			m_pRobotDriver->m_cLog->Write("�ܼ������� %d���Ѿ��ɷ������� %d", m_nDrawingNum, nProcessedNum);
		}
		m_pRobotDriver->m_cLog->Write("�ɷ�������");
		//----------------------- �ɷ������� -----------------------//

		m_vtImgCollectDataBuff[nCurrentFreeThreadNo].nDataNo = nProcessedNum;

		IplImage* pImage = GetCallBackImage(nProcessedNum);
		if (pImage == NULL)
		{
			m_pRobotDriver->m_cLog->Write("�� %d �δ���ͼƬ�쳣�������˴δ���", nProcessedNum);
			nProcessedNum++;
			ReleaseSemaphore(m_hParallelProcessingSem, 1, NULL);
			continue;
		}

		m_pRobotDriver->m_cLog->Write("Before Copy");
		cvCopyImage(pImage, m_vtImgCollectDataBuff[nCurrentFreeThreadNo].tImageData.ptImg);
		m_pRobotDriver->m_cLog->Write("After Copy");

		// ɨ�跽������Ӽ�����ֵ����ɨ���ٶ�(mm/s)/��ͼƵ��(Hz)
		double dOffsetDir = m_nScanDir > 0 ? 1.0 : -1.0;
		double dCaptureOffsetDis = ((double)(m_tRobotLimitation.drScanSpeed / 60.0 / CAPTURE_IMAGE_FRAME_RATE));
		m_vtImgCollectDataBuff[nCurrentFreeThreadNo].tImageData.tRobotCoord = m_tRobotScanPos;
		m_vtImgCollectDataBuff[nCurrentFreeThreadNo].tImageData.tRobotPulse = m_tRobotScanPulse;
		m_vtImgCollectDataBuff[nCurrentFreeThreadNo].tImageData.dExAxisOffsetX = nProcessedNum * ((1 == labs(m_nScanDir) ? dCaptureOffsetDis * dOffsetDir : 0.0));
		m_vtImgCollectDataBuff[nCurrentFreeThreadNo].tImageData.dExAxisOffsetY = nProcessedNum * ((2 == labs(m_nScanDir) ? dCaptureOffsetDis * dOffsetDir : 0.0));
		m_vtImgCollectDataBuff[nCurrentFreeThreadNo].tImageData.dExAxisOffsetZ = nProcessedNum * ((3 == labs(m_nScanDir) ? dCaptureOffsetDis * dOffsetDir : 0.0));

		pImage = NULL;
		ReleaseCallBackImage(nProcessedNum);

		m_eThreadStateMark[nCurrentFreeThreadNo] = PROCESS_THREAD_WORKING;
		nProcessedNum++;
		m_pRobotDriver->m_cLog->Write("�ɷ����� %d", nCurrentFreeThreadNo);
		//----------------------- ���� -----------------------//
		ReleaseSemaphore(m_hParallelProcessingSem, 1, NULL);
		//Sleep(50);
	}
	m_pRobotDriver->m_cLog->Write("ѭ������");
	Sleep(500);
	m_bProcessStartMark = FALSE;
	m_pRobotDriver->m_cLog->Write("m_bProcessStartMark %d", m_bProcessStartMark);
	for (int n = 0; n < MAX_PROCESS_THREAD_NUM; n++)
	{
		ReleaseSemaphore(m_vtImgCollectDataBuff[n].m_hWorkHandle, 1, NULL);
	}
	m_pRobotDriver->m_cLog->Write("�������");

	while (true)
	{
		//CHECK_STOP_BREAK(m_pRobotDriver);
		DoEvent();
		if (TRUE == CheckThreadQuit(nRobotNo))
		{
			break;
		}
		Sleep(150);		//����鴦����ʾ�����������ڸ�Ϊ50ms
	}
	CloseHandle(m_hParallelProcessingSem);
	fclose(fRecordTimeInfo);
	for (int nNum = 0; nNum < MAX_PROCESS_THREAD_NUM; nNum++)
	{
		cvReleaseImage(&m_vtImgCollectDataBuff[nNum].tImageData.ptImg);
	}
	//XiMessageBox("%d�Ż����� �������", pRobotDriver->m_nRobotNo);
}
void CLaserLineScreen::SchedulingFunction(int nCamareNo, int scanNum)
{
	int nRobotNo = m_pRobotDriver->m_nRobotNo;
	BOOL bRecordTime = TRUE;

	m_vnThreadNoNew[nCamareNo].clear();
	m_vbProcessStartMark[nCamareNo] = TRUE;
	m_vbStartHandleMark[nCamareNo] = TRUE;
	m_vnDrawingNum[nCamareNo] = 0;
	m_vnProcessNum[nCamareNo] = 0;

	m_vhParallelProcessingSem[nCamareNo] = CreateSemaphore(NULL, MAX_PROCESS_THREAD_NUM, MAX_PROCESS_THREAD_NUM, m_pRobotDriver->m_strRobotName);
	if (m_vhParallelProcessingSem[nCamareNo] == NULL)
	{
		XiMessageBox("������������ʧ�ܣ�");
		return;
	}
	for (int nNum = 0; nNum < MAX_PROCESS_THREAD_NUM; nNum++)
	{
		//CvSize ImageSize = cvSize(m_pDHCameraVision->m_tCameraPare.ImageSize.x, m_pDHCameraVision->m_tCameraPare.ImageSize.y);
		CvSize ImageSize;
		ImageSize.width = m_ptUnit->m_vtCameraPara[m_ptUnit->m_nLineScanCameraNo+ nCamareNo].tDHCameraDriverPara.nRoiWidth;
		ImageSize.height = m_ptUnit->m_vtCameraPara[m_ptUnit->m_nLineScanCameraNo+ nCamareNo].tDHCameraDriverPara.nRoiHeight;
		IplImage* cvImage = cvCreateImage(ImageSize, IPL_DEPTH_8U, 1);
		m_vvtImgCollectDataBuff[nCamareNo][nNum].tImageData.ptImg = cvImage;
		m_veThreadStateMark[nCamareNo][nNum] = PROCESS_THREAD_FREE;
	}
	m_pRobotDriver->m_cLog->Write("���д�����ʼ�����");

	FILE* fRecordTimeInfo;
	if (0 == nRobotNo)
	{
		CString strName;
		strName.Format(OUTPUT_PATH + m_ptUnit->m_tContralUnit.strUnitName + RECOGNITION_FOLDER+"PointCloud_%d.txt", scanNum);
		bRecordTime = !XI_fopen_s(&fRecordTimeInfo, strName, "w");
	}
	
	if (bRecordTime == FALSE)
	{
		m_pRobotDriver->m_cLog->Write("�� HandleScanDataTimeInfo.txt �ļ�ʧ��, ���رռ�¼����ʱ�书��");
	}

	for (int nNum = 0; nNum < MAX_PROCESS_THREAD_NUM; nNum++)
	{
		m_nThreadNoNew[nCamareNo] = nNum;

		T_GET_LINESCAN_PHOTO* tLineScanParam = new T_GET_LINESCAN_PHOTO();
		tLineScanParam->nCameraNo = nCamareNo;
		tLineScanParam->pcLineScan = this;

		AfxBeginThread(ThreadExecutiveFunctionNew, tLineScanParam);
		int n = 3;
		while (m_nThreadNoNew[nCamareNo] != -1)
		{
			//CHECK_STOP_BREAK(m_pRobotDriver);
			Sleep(10);
			DoEvent();
			n--;
			if (n < 0)
			{
				m_pRobotDriver->m_cLog->Write("�����߳� %d ʧ��", nNum);
				break;
			}
		}
	}
	int nProcessedNum = 0;
	int oldnProcessedNum = 0;
	int Comsize = 0;
	while (/*TRUE*/m_vbStartHandleMark[nCamareNo])
	{
		//----------------------- ������ִ���߳� ------------------//
		m_pRobotDriver->m_cLog->Write("������");
		WaitForSingleObject(m_vhParallelProcessingSem[nCamareNo], INFINITE);
		int nCurrentFreeThreadNo = GetFreeThreadNo(nRobotNo, nCamareNo);
		if (nCurrentFreeThreadNo == -1)
		{
			ReleaseSemaphore(m_vhParallelProcessingSem[nCamareNo], 1, NULL);
			Sleep(1);
			m_pRobotDriver->m_cLog->Write("������");
			continue;
		}
		m_pRobotDriver->m_cLog->Write("����ִ���߳�  %d", nCurrentFreeThreadNo);
		m_pRobotDriver->m_cLog->Write("���洦����");

		//----------------------- ���洦���� -----------------------//
		SaveLineScanResult(fRecordTimeInfo, nCamareNo);
		m_vnDrawingNum[nCamareNo] = GetCallBackImageNum(nCamareNo);
		m_pRobotDriver->m_cLog->Write("nCamareNo:%d �жϴ������ ͼ���� %d", nCamareNo, m_vnDrawingNum[nCamareNo]);

		//----------------------- �жϴ������ -----------------------//
		if (m_vnDrawingNum[nCamareNo] <= nProcessedNum)
		{
			/*WaitForSingleObject(m_hScanMoveOver, INFINITY);
			if (m_bScanMoveOver) 
			{
				ReleaseSemaphore(m_hScanMoveOver, 1, NULL);
				ReleaseSemaphore(m_vhParallelProcessingSem[nCamareNo], 1, NULL);
				m_pRobotDriver->m_cLog->Write("�˳�");
				break;
			}*/
			if (oldnProcessedNum == m_vnDrawingNum[nCamareNo])
			{
				Comsize++;
			}
			else
			{
				oldnProcessedNum = m_vnDrawingNum[nCamareNo];
				Comsize = 0;
			}

			m_pRobotDriver->m_cLog->Write("nCamareNo:%d �ɼ�ͼƬ ͼ����%d������ͼƬ ͼ����%d", 
				nCamareNo,m_vnDrawingNum[nCamareNo], nProcessedNum);
			if (TRUE == CheckThreadFree(nRobotNo, nCamareNo) && FALSE == m_vbScanMark[nCamareNo])
			{
				ReleaseSemaphore(m_vhParallelProcessingSem[nCamareNo], 1, NULL);
				m_pRobotDriver->m_cLog->Write("�˳�");
				break;
			}
			Sleep(5);
			ReleaseSemaphore(m_vhParallelProcessingSem[nCamareNo], 1, NULL);

			if (Comsize >= 20)
			{
				m_pRobotDriver->m_cLog->Write("nCamareNo:%d �˳�", nCamareNo);
				//XiMessageBox("%s ��ɨ�����쳣������������������ɨ", pRobotDriver->m_strRobotName);
				//OnBtnStop(pRobotDriver);
				break;
			}

			m_pRobotDriver->m_cLog->Write("����");
			continue;
		}
		else
		{
			if (CheckCallBackImage(nProcessedNum, nCamareNo) == FALSE)
			{
				m_pRobotDriver->m_cLog->Write("nCamareNo:%d �� %d �δ�������쳣�������˴δ��� ", nCamareNo, nProcessedNum);
				nProcessedNum++;
				ReleaseSemaphore(m_vhParallelProcessingSem[nCamareNo], 1, NULL);
				continue;
			}
			m_pRobotDriver->m_cLog->Write("nCamareNo:%d �ܼ������� %d���Ѿ��ɷ������� %d ", nCamareNo, m_vnDrawingNum[nCamareNo], nProcessedNum);
		}
		m_pRobotDriver->m_cLog->Write("�ɷ�������");
		//----------------------- �ɷ������� -----------------------//

		m_vvtImgCollectDataBuff[nCamareNo][nCurrentFreeThreadNo].nDataNo = nProcessedNum;

		IplImage* pImage = GetCallBackImage(nProcessedNum, nCamareNo);
		if (pImage == NULL)
		{
			m_pRobotDriver->m_cLog->Write("nCamareNo:%d �� %d �δ���ͼƬ�쳣�������˴δ��� ", nCamareNo, nProcessedNum);
			nProcessedNum++;
			ReleaseSemaphore(m_vhParallelProcessingSem[nCamareNo], 1, NULL);
			continue;
		}

		//m_pRobotDriver->m_cLog->Write("cvCopyImage(pImage, m_pShowLaserImgBuff);");
		cvCopyImage(pImage, m_vvtImgCollectDataBuff[nCamareNo][nCurrentFreeThreadNo].tImageData.ptImg);
		
		
		// ɨ�跽������Ӽ�����ֵ����ɨ���ٶ�(mm/s)/��ͼƵ��(Hz)		
		double dCaptureOffsetDis = ((double)(m_tRobotLimitation.drScanSpeed / 60.0 / CAPTURE_IMAGE_FRAME_RATE));
		double dOffsetDir = m_nScanDir > 0 ? 1.0 : -1.0;
		m_vvtImgCollectDataBuff[nCamareNo][nCurrentFreeThreadNo].tImageData.tRobotCoord = m_tRobotScanPos;
		m_vvtImgCollectDataBuff[nCamareNo][nCurrentFreeThreadNo].tImageData.tRobotPulse = m_tRobotScanPulse;
		m_vvtImgCollectDataBuff[nCamareNo][nCurrentFreeThreadNo].tImageData.dExAxisOffsetX = nProcessedNum * ((1 == labs(m_nScanDir) ? dCaptureOffsetDis * dOffsetDir : 0.0));
		m_vvtImgCollectDataBuff[nCamareNo][nCurrentFreeThreadNo].tImageData.dExAxisOffsetY = nProcessedNum * ((2 == labs(m_nScanDir) ? dCaptureOffsetDis * dOffsetDir : 0.0));
		m_vvtImgCollectDataBuff[nCamareNo][nCurrentFreeThreadNo].tImageData.dExAxisOffsetZ = nProcessedNum * ((3 == labs(m_nScanDir) ? dCaptureOffsetDis * dOffsetDir : 0.0));
		m_pRobotDriver->m_cLog->Write("cvCopyImage(pImage,nCamareNo:%d m_pRobotDriver->m_tRobotLimitation.drScanSpeed:%lf �ⲿ��λ�ã� % .3lf % .3lf ƫ����; % .3lf % .3lf",
			nCamareNo, m_tRobotLimitation.drScanSpeed, m_tRobotScanPos.dBX, m_tRobotScanPos.dBY,
			m_vvtImgCollectDataBuff[nCamareNo][nCurrentFreeThreadNo].tImageData.dExAxisOffsetX,
			m_vvtImgCollectDataBuff[nCamareNo][nCurrentFreeThreadNo].tImageData.dExAxisOffsetY);
		pImage = NULL;
		ReleaseCallBackImage(nProcessedNum, nCamareNo);

		m_veThreadStateMark[nCamareNo][nCurrentFreeThreadNo] = PROCESS_THREAD_WORKING;
		nProcessedNum++;
		m_pRobotDriver->m_cLog->Write("nCamareNo:%d �ɷ����� %d ", nCamareNo, nCurrentFreeThreadNo);
		//----------------------- ���� -----------------------//
		ReleaseSemaphore(m_vhParallelProcessingSem[nCamareNo], 1, NULL);
		//Sleep(50);
	}
	m_pRobotDriver->m_cLog->Write("ѭ������");
	Sleep(500);
	m_vbProcessStartMark[nCamareNo] = FALSE;
	m_pRobotDriver->m_cLog->Write("m_bProcessStartMark %d nCamareNo:%d",m_vbProcessStartMark[nCamareNo], nCamareNo);
	for (int n = 0; n < MAX_PROCESS_THREAD_NUM; n++)
	{
		//ReleaseSemaphore(m_vtImgCollectDataBuff[nRobotNo][n].m_hWorkHandle, 1, NULL);
	}
	m_pRobotDriver->m_cLog->Write("�������");

	while (true)
	{
		//CHECK_STOP_BREAK(m_pRobotDriver);
		DoEvent();
		if (TRUE == CheckThreadQuit(nRobotNo, nCamareNo))
		{
			break;
		}
		Sleep(150);		//����鴦����ʾ�����������ڸ�Ϊ50ms
	}
	CloseHandle(m_vhParallelProcessingSem[nCamareNo]);
	fclose(fRecordTimeInfo);
	for (int nNum = 0; nNum < MAX_PROCESS_THREAD_NUM; nNum++)
	{
		cvReleaseImage(&m_vvtImgCollectDataBuff[nCamareNo][nNum].tImageData.ptImg);
	}
	//XiMessageBox("%d�Ż����� �������", pRobotDriver->m_nRobotNo);
}

void CLaserLineScreen::ExecutiveFunction(int nThreadNo)
{
	m_nThreadNo = -1;
	if (nThreadNo >= 0 && nThreadNo <= MAX_PROCESS_THREAD_NUM)
	{
		int nNum;
		for (nNum = 0; nNum < m_vnThreadNo.size(); nNum++)
		{
			if (m_vnThreadNo[nNum] == nThreadNo)
			{
				break;
			}
		}
		if (nNum >= m_vnThreadNo.size())
		{
			m_vnThreadNo.push_back(nThreadNo);
		}
		else
		{
			m_pRobotDriver->m_cLog->Write("�ظ������̣߳�%d", nThreadNo);
			return;
		}
	}
	else
	{
		m_pRobotDriver->m_cLog->Write("�̱߳�Ŵ���%d", nThreadNo);
		return;
	}
	m_pRobotDriver->m_cLog->Write("�����߳� %d ", nThreadNo);
	while (TRUE == m_bProcessStartMark)
	{
		DoEvent();
		if (m_eThreadStateMark[nThreadNo] != PROCESS_THREAD_WORKING)
		{
			Sleep(5);
			continue;
		}
		if (FALSE == m_bProcessStartMark)
		{
			break;
		}

		m_pRobotDriver->m_cLog->Write("�߳� %d ��������", nThreadNo);
		WaitForSingleObject(m_hParallelProcessingSem, INFINITE);
		if (TRUE == ImageProcess(m_vtImgCollectDataBuff[nThreadNo]))
		{
			m_pRobotDriver->m_cLog->Write("�߳� %d �������", nThreadNo);
			m_eThreadStateMark[nThreadNo] = PROCESS_THREAD_COMPLETE;
		}
		else
		{
			m_pRobotDriver->m_cLog->Write("�߳� %d ����ʧ��", nThreadNo);
			m_eThreadStateMark[nThreadNo] = PROCESS_THREAD_FAIL;
		}

		ReleaseSemaphore(m_hParallelProcessingSem, 1, NULL);

	}
	m_eThreadStateMark[nThreadNo] = PROCESS_THREAD_QUIT;
	m_pRobotDriver->m_cLog->Write("�߳� %d �˳�", nThreadNo);
}

void CLaserLineScreen::ExecutiveFunction(int nThreadNo, int nCamareNo)
{
	m_nThreadNoNew.at(nCamareNo) = -1;
	if (nThreadNo >= 0 && nThreadNo <= MAX_PROCESS_THREAD_NUM)
	{
		int nNum;
		for (nNum = 0; nNum < m_vnThreadNoNew[nCamareNo].size(); nNum++)
		{
			if (m_vnThreadNoNew[nCamareNo][nNum] == nThreadNo)
			{
				break;
			}
		}
		if (nNum >= m_vnThreadNoNew[nCamareNo].size())
		{
			m_vnThreadNoNew[nCamareNo].push_back(nThreadNo);
		}
		else
		{
			m_pRobotDriver->m_cLog->Write("�ظ������̣߳�%d", nThreadNo);
			return;
		}
	}
	else
	{
		m_pRobotDriver->m_cLog->Write("�̱߳�Ŵ���%d", nThreadNo);
		return;
	}
	m_pRobotDriver->m_cLog->Write("�����߳� nCamareNo:%d %d ", nCamareNo, nThreadNo);
	while (TRUE == m_vbProcessStartMark[nCamareNo])
	{
		DoEvent();
		if (m_veThreadStateMark[nCamareNo][nThreadNo] != PROCESS_THREAD_WORKING)
		{
			Sleep(5);
			continue;
		}
		if (FALSE == m_vbProcessStartMark[nCamareNo])
		{
			break;
		}

		m_pRobotDriver->m_cLog->Write("nCamareNo:%d �߳� %d ��������", nCamareNo,nThreadNo);
		WaitForSingleObject(m_vhParallelProcessingSem[nCamareNo], INFINITE);
		if (TRUE == ImageProcess(m_vvtImgCollectDataBuff[nCamareNo][nThreadNo], nCamareNo))
		{
			m_pRobotDriver->m_cLog->Write("nCamareNo:%d �߳� %d �������", nCamareNo, nThreadNo);
			m_veThreadStateMark[nCamareNo][nThreadNo] = PROCESS_THREAD_COMPLETE;
		}
		else
		{
			m_pRobotDriver->m_cLog->Write("�߳� %d ����ʧ��", nThreadNo);
			m_veThreadStateMark[nCamareNo][nThreadNo] = PROCESS_THREAD_FAIL;
		}

		ReleaseSemaphore(m_vhParallelProcessingSem[nCamareNo], 1, NULL);

	}
	m_veThreadStateMark[nCamareNo][nThreadNo] = PROCESS_THREAD_QUIT;
	m_pRobotDriver->m_cLog->Write("nCamareNo:%d �߳� %d �˳�", nCamareNo, nThreadNo);
}

BOOL CLaserLineScreen::LaserScanRuning()
{
	std::vector<CvPoint3D64f> vtTempPtn;
	std::vector< CvPoint3D64f> vtArea;
	for (int scanNum = 1; scanNum <= 2; scanNum++)
	{
	CRobotDriverAdaptor* pRobotDriver = m_pRobotDriver;
	int nRobotNo = pRobotDriver->m_nRobotNo;
	m_hScanMoveOver = CreateSemaphore(NULL,1,1,"��ɨ�Ƿ��˶�����");
	m_bScanMoveOver = FALSE;
	long long dStar1;
	dStar1 = XI_clock();
//	E_CAM_ID eCamId;
	m_pRobotDriver->m_cLog->Write("��ʼ��ɨʶ��");
	GetMemoryInfo("��ʼ��ɨʶ��");
	LoadLineScanParam(scanNum);
	CheckFolder(OUTPUT_PATH + m_pRobotDriver->m_strRobotName + "\\" + POINT_CLOUD_IMAGE);
	CString sPointCloudFile = OUTPUT_PATH + m_pRobotDriver->m_strRobotName + "\\" + POINT_CLOUD_FILE;

	double dCurRobotZ = pRobotDriver->GetCurrentPos(ROBOT_AXIS_Z);
	if (dCurRobotZ > 1700.0 || dCurRobotZ < -500)
	{
		WriteLog("��ǰλ��RobotZ%d����ȫ��ֹ�ƶ���", dCurRobotZ);
		//XiMessageBox("��ǰλ��RobotZ%d����ȫ��ֹ�ƶ���", dCurRobotZ);
		//CHECK_BOOL_RETURN(m_ptUnit->MoveToSafeHeight());
		//return 0;
	}


	//���޸�
	if ((TRUE == *m_pbNaturalPop) && IDOK != XUI::MesBox::PopOkCancel("��ע�⣬�Ƿ�����ɨ�裬�ⲿ�����{0:.3f �յ�{1:.3f}", m_dScanStartCarLoction, m_dScanStartCarLoction + m_dScanLength)/*XiMessageBox("��ע�⣬�Ƿ�����ɨ�裬�ⲿ�����%.3lf �յ�%.3lf",
		m_dScanStartCarLoction, m_dScanStartCarLoction + m_dScanLength)*/)
//	if ((TRUE == *m_pbNaturalPop) && IDOK != XiMessageBox("��ע�⣬�Ƿ�����ɨ�裬�ⲿ�����%.3lf �յ�%.3lf",m_dScanStartCarLoction, m_dScanStartCarLoction + m_dScanLength))
	{
		return 0;
	}

	// ���뱣��������ݵ�vectorԤ���ռ�   ɨ�����d(mm) ֡��f(Hz) ɨ���ٶ�v(mm/min) �������sample(pixel) ͼ����ImageWidth(pixel)
	// ����ɨ��ʱ��T(s) = d / v * 60.0 s
	// ���۶�ά����PointCloudNum = T * f * ImageWidth / sample
	double d = fabs(m_dScanLength/*m_tEndWorldCoors.dY - m_tStartWorldCoors.dY*/);
	double f = CAPTURE_IMAGE_FRAME_RATE;
	double v = m_tRobotLimitation.drScanSpeed;
	int sample = LINE_SCAN_STEP_2D_POINT;
	int ImageWidth = m_ptUnit->m_vtCameraPara[m_ptUnit->m_nLineScanCameraNo].tDHCameraDriverPara.nMaxWidth;
	double T = d / v * 60.0;
	int nPointCloudNum = (int)(T * f * (double)ImageWidth / (double)sample);
	nPointCloudNum = (int)((double)nPointCloudNum * 1.1); // ��Ԥ��10%�ռ�
	for (int n = 0; n < LINESCAN_CAMERA_NUM; n++)
	{
		m_vvtPointCloud.at(n).reserve(nPointCloudNum);
	}
	m_vtPointCloud.reserve(nPointCloudNum * LINESCAN_CAMERA_NUM);

	/*********************************�˶�����ɨ��ʼ��*******************************************/
	if (false == g_bGantryEnableMark)
	{ 
		//	������+�ⲿ���˶�����ɨ���
		if (0 != m_ptUnit->WorldMoveByJob(m_tStartPlus, m_pRobotDriver->m_tPulseHighSpeed, "MOVJ"))
		{
			return false;
		}
	}
	else
	{
		double dScanMovePos = 0.0;
		dScanMovePos = 1 == labs(m_nScanDir) ? m_tStartWorldCoors.dBX : dScanMovePos;
		dScanMovePos = 2 == labs(m_nScanDir) ? m_tStartWorldCoors.dBY : dScanMovePos;
		dScanMovePos = 3 == labs(m_nScanDir) ? m_tStartWorldCoors.dBZ : dScanMovePos;
		if (0 != m_ptUnit->MoveExAxisFun(dScanMovePos, m_tRobotLimitation.drRunSpeed, m_ptUnit->m_nLinedScanAxisNo))return false;
	}
	m_ptUnit->WorldCheckRobotDone();

	//���޸�
	if ((TRUE == *m_pbNaturalPop) && IDOK != XUI::MesBox::PopOkCancel("{0}�Ż����� ��ʼ��ɨ", nRobotNo)/*XiMessageBox("%d�Ż����� ��ʼ��ɨ", nRobotNo)*/)
	//if ((TRUE == *m_pbNaturalPop) && IDOK != XiMessageBox("%d�Ż����� ��ʼ��ɨ", nRobotNo))
	{
		return 0;
	}

	//m_tRobotScanPos = m_ptUnit->GetCurrentPosWorld();	//����ɨ����е��ֱ������
	//m_tRobotScanPulse = m_ptUnit->GetCurrentPulseWorld();	//����ɨ����е�۹ؽ�����
	m_tRobotScanPos = m_ptUnit->GetRobotCtrl()->GetCurrentPos();	//����ɨ����е��ֱ������
	m_tRobotScanPulse = m_ptUnit->GetRobotCtrl()->GetCurrentPulse();	//����ɨ����е�۹ؽ�����
	// ��ȡ�������� ���ת����ϵ,����ʱת������Ϊ��λ��
	/*CvPoint3D64f CarPoint;
	CarPoint.x = m_tRobotScanPos.dBX;
	CarPoint.y = m_tRobotScanPos.dBY;
	CarPoint.z = m_tRobotScanPos.dBZ;
	CvPoint3D64f RealCarPoint = m_ptUnit->TransCoor_Robot2Gantry(CarPoint);
	m_tRobotScanPos.dBX = (1 == labs(m_nScanDir) ? RealCarPoint.x : 0.0);
	m_tRobotScanPos.dBY = (2 == labs(m_nScanDir) ? RealCarPoint.y : 0.0);
	m_tRobotScanPos.dBZ = (3 == labs(m_nScanDir) ? RealCarPoint.z : 0.0);*/
	//------------------------------------------------------------------------
	long long dStart = XI_clock();

	double dMachineCarMovePos = m_dScanStartCarLoction + m_dScanLength;
	//CString s;
	//s.Format("��������ɨ�ӵ�ǰλ�ô��˶�Ŀ�����λ�� %.3lf mm �ٶȣ�%.3lf mm/min", dMachineCarMovePos, m_tRobotLimitation.drScanSpeed);
	//XiMessageBoxPopup(m_pRobotDriver->m_cLog, s, *m_pbNaturalPop);

	if (0 != m_ptUnit->MoveExAxisFun(dMachineCarMovePos, m_tRobotLimitation.drScanSpeed, m_ptUnit->m_nLinedScanAxisNo))return false;

	/*********************************��ͼ*******************************************/
	m_bScanMark = TRUE;	//��ɨ��־λ����Ϊ��
	//��ʼ����ͼ�����������̵� ������д�ļ���ʼ��������̱߳�־λ�ж��߳���ͣ���жϲ�ͼ�Ƿ���������
	for (int nCameraNo = 0; nCameraNo < LINESCAN_CAMERA_NUM; nCameraNo++)
	{
		//��ʼ����ͼ�����������̵� ������д�ļ���ʼ��������̱߳�־λ�ж��߳���ͣ���жϲ�ͼ�Ƿ���������
		m_vbScanMark[nCameraNo] = true;
		T_GET_LINESCAN_PHOTO* tLineScanParam = new T_GET_LINESCAN_PHOTO();
		tLineScanParam->nCameraNo = nCameraNo;
			tLineScanParam->nScanNum = scanNum;
		tLineScanParam->pcLineScan = this;
		m_pcThreadSchedulingFunction[nCameraNo] = AfxBeginThread(ThreadSchedulingFunctionNew, tLineScanParam);
		// �˶� �� ��ͼ�߳�
		StartCallBackImage(nCameraNo);
	}

	Sleep(1000); // ȷ������
	while (m_ptUnit->WorldIsRunning()) // �ȴ��˶�ֹͣ
	{
		DoEvent();
		Sleep(100);
	}
	/*WaitForSingleObject(m_hScanMoveOver, INFINITY);
	m_bScanMoveOver = TRUE;
	ReleaseSemaphore(m_hScanMoveOver,1,NULL);
	Sleep(50);*/
	vector<double> vdCurFrameRate(LINESCAN_CAMERA_NUM,0.0);
	for (int nCameraNo = 0; nCameraNo < LINESCAN_CAMERA_NUM; nCameraNo++)
	{		
		StopCallBackImage(nCameraNo);	
	}
	for (int nCameraNo = 0; nCameraNo < LINESCAN_CAMERA_NUM; nCameraNo++)
	{
		WaitForSingleObject(m_pcThreadSchedulingFunction[nCameraNo]->m_hThread, INFINITE);
		m_vtpLaserCamera[nCameraNo]->ReadCapInfo(false, &vdCurFrameRate[nCameraNo]);
		ClearCallBackImage(nCameraNo);
	}
	// �����߳̽����� ���Ƽ���
	for (int nCameraNo = 0; nCameraNo < LINESCAN_CAMERA_NUM; nCameraNo++)
	{
		pRobotDriver->m_cLog->Write("StopCallBackImage nCameraNo:%d", nCameraNo);
		m_vtpLaserCamera[nCameraNo]->ReadCapInfo(false, &vdCurFrameRate[nCameraNo]);
		WriteLog("����֡�ʣ�%d  ʵ��֡�ʣ�%.3lf", CAPTURE_IMAGE_FRAME_RATE, vdCurFrameRate[nCameraNo]);
		if (fabs(vdCurFrameRate[nCameraNo] - (double)CAPTURE_IMAGE_FRAME_RATE) > 0.1)
		{
			FILE* pf = fopen(sPointCloudFile, "w");
			fclose(pf);
			m_vtPointCloud.clear();
			m_vtPointCloud.shrink_to_fit();
			XUI::MesBox::PopInfo("���ۣ�{0}  ʵ�ʣ�{1} ɨ��ʧ��", CAPTURE_IMAGE_FRAME_RATE, vdCurFrameRate[nCameraNo]);
			return false;
		}

		m_vbScanMark[nCameraNo] = FALSE;
		m_vbStartHandleMark[nCameraNo] = false;
		pRobotDriver->m_cLog->Write("��ɨ�˶�ֹͣ���ȴ������߳̽��� nCameraNo:%d m_bStartHandleMark %d", nCameraNo,m_vbScanMark[nCameraNo]);
		pRobotDriver->m_cLog->Write("�����߳��Ѿ�����nCameraNo:%d", nCameraNo);
		// ���ܵ�������
		m_vtPointCloud.insert(m_vtPointCloud.end(), m_vvtPointCloud[nCameraNo].begin(), m_vvtPointCloud[nCameraNo].end());
		if (scanNum == 2)
		{
			for (int i = 0; i < m_vvtPointCloud.size(); i++)
			{
				m_vvtPointCloud[nCameraNo][i].x += 0/*8.871389*/;
				m_vvtPointCloud[nCameraNo][i].y += 0;
			}
			vtArea = vtTempPtn;
		}
		vtTempPtn.insert(vtTempPtn.end(), m_vvtPointCloud[nCameraNo].begin(), m_vvtPointCloud[nCameraNo].end());
		
		std::vector<CvPoint3D64f> vtpoint;
		m_vvtPointCloud[nCameraNo] = vtpoint;
		m_vvtPointCloud[nCameraNo].clear();
	}

//#ifndef SINGLE_ROBOT
	// �����Ƿ��˶���λ
	double dCurCarPos = m_ptUnit->GetExPositionDis(m_ptUnit->m_nLinedScanAxisNo);
	if (fabs(dCurCarPos - dMachineCarMovePos) > 3.0) // �ⲿ��δ�˶���λ
	{
		XiMessageBox("��ɨδ�˶���Ŀ��λ�ã���ɨʧ�ܣ�");
		m_vtPointCloud.clear();
		m_vtPointCloud.shrink_to_fit();
		return false;
	}
//#endif // SINGLE_ROBOT

	
	//CloseHandle(m_hScanMoveOver);
	GetMemoryInfo("������ɨʶ��");
	}
//		CvPoint3D64f tTempPtn;

	CString strName0, strName1, strName2;
	double nNo=0, nNo2=0, nNo3=0;
	strName0.Format(OUTPUT_PATH + m_ptUnit->m_tContralUnit.strUnitName + RECOGNITION_FOLDER + "PointCloud_0.txt");
	FILE* pf = fopen(strName0, "w");
	for (int i = 0; i < vtTempPtn.size(); i++)
	{ 
		fprintf(pf, "%d %lf %lf %lf %lf %lf\n", i, vtTempPtn[i].x, vtTempPtn[i].y, vtTempPtn[i].z, nNo2, nNo3);

	}
	
	fclose(pf);
	//fclose(pf1);
	//fclose(pf2);
	
	return true;
}

double CLaserLineScreen::GetLaserCamCurrentFrameRate()
{
	double dCurrentFrameRate;
	m_pLaserCamera->GetCurrentFrameRate(dCurrentFrameRate);
	return dCurrentFrameRate;
}

double CLaserLineScreen::GetLaserCamCurrentFrameRate(int nCamareNo)
{
	double dCurrentFrameRate;
	m_vtpLaserCamera[nCamareNo]->GetCurrentFrameRate(dCurrentFrameRate);
	return dCurrentFrameRate;
}

int CLaserLineScreen::GetCallBackImageNum()
{
	return m_pLaserCamera->m_vtCallBackImage.size();
}
int CLaserLineScreen::GetCallBackImageNum(int nCamareNo)
{
	return m_vtpLaserCamera[nCamareNo]->m_vtCallBackImage.size();
}
BOOL CLaserLineScreen::CheckCallBackImage(int nImageID)
{
	//WriteLog("CheckCallBackImage->GetCallBackImageNum: %d", GetCallBackImageNum());
	for (int nNum = 0; nNum < GetCallBackImageNum(); nNum++)
	{
		if (m_pLaserCamera->m_vtCallBackImage[nNum]->nImageID == nImageID)
		{
			return TRUE;
		}
	}
	if (GetCallBackImageNum() >= 1)
	{
		WriteLog("����쳣����ţ�%d��ͼ��ID��%d", nImageID, m_pLaserCamera->m_vtCallBackImage[GetCallBackImageNum() - 1]->nImageID);
	}
	return FALSE;
}
BOOL CLaserLineScreen::CheckCallBackImage(int nImageID, int nCamareNo)
{
	//WriteLog("nCamareNo:%d CheckCallBackImage->GetCallBackImageNum: %d", nCamareNo, GetCallBackImageNum(nCamareNo));
	for (int nNum = 0; nNum < GetCallBackImageNum(nCamareNo); nNum++)
	{
		if (m_vtpLaserCamera[nCamareNo]->m_vtCallBackImage[nNum]->nImageID == nImageID)
		{
			return TRUE;
		}
	}
	if (GetCallBackImageNum(nCamareNo) >= 1)
	{
		WriteLog("����쳣����ţ�%d %d��ͼ��ID��%d", nCamareNo,nImageID, m_vtpLaserCamera[nCamareNo]->m_vtCallBackImage[GetCallBackImageNum(nCamareNo) - 1]->nImageID);
	}
	return FALSE;
}
BOOL CLaserLineScreen::StartCallBackImage()
{
	return m_pLaserCamera->StartAcquisition();
}

BOOL CLaserLineScreen::StartCallBackImage(int nCamareNo)
{
	return m_vtpLaserCamera[nCamareNo]->StartAcquisition();
}

BOOL CLaserLineScreen::StopCallBackImage()
{
	return m_pLaserCamera->StopAcquisition();
}

BOOL CLaserLineScreen::StopCallBackImage(int nCamareNo)
{
	return m_vtpLaserCamera[nCamareNo]->StopAcquisition(false);
}

void CLaserLineScreen::ClearCallBackImage()
{
	m_pLaserCamera->ClearCallBackImage();
}
void CLaserLineScreen::ClearCallBackImage(int nCamareNo)
{
	m_vtpLaserCamera[nCamareNo]->ClearCallBackImage();
}

void CLaserLineScreen::ReleaseCallBackImage(int nImageID)
{
	m_pLaserCamera->m_vtCallBackImage[nImageID]->Clear();
}

void CLaserLineScreen::ReleaseCallBackImage(int nImageID, int nCamareNo)
{
	m_vtpLaserCamera[nCamareNo]->m_vtCallBackImage[nImageID]->Clear();
}

void CLaserLineScreen::ReadCapInfo(bool bIsShowInfo)
{
	m_pLaserCamera->ReadCapInfo(bIsShowInfo);
}

void CLaserLineScreen::ReadCapInfo(bool bIsShowInfo, int nCamareNo)
{
	m_vtpLaserCamera[nCamareNo]->ReadCapInfo(bIsShowInfo);
}

IplImage * CLaserLineScreen::GetCallBackImage(int nImageID)
{
	if (FALSE == CheckCallBackImage(nImageID))
	{
		return NULL;
	}
	for (int nNum = 0; nNum < GetCallBackImageNum(); nNum++)
	{
		if (m_pLaserCamera->m_vtCallBackImage[nNum]->nImageID == nImageID)
		{
			return m_pLaserCamera->m_vtCallBackImage[nNum]->pImage;
		}
	}
	return NULL;
}

IplImage* CLaserLineScreen::GetCallBackImage(int nImageID, int nCamareNo)
{
	if (FALSE == CheckCallBackImage(nImageID, nCamareNo))
	{
		return NULL;
	}
	for (int nNum = 0; nNum < GetCallBackImageNum(nCamareNo); nNum++)
	{
		if (m_vtpLaserCamera[nCamareNo]->m_vtCallBackImage[nNum]->nImageID == nImageID)
		{
			return m_vtpLaserCamera[nCamareNo]->m_vtCallBackImage[nNum]->pImage;
		}
	}
	return NULL;
}

int CLaserLineScreen::GetFreeThreadNo(int nRobotNo)
{
	for (int nNum = 0; nNum < MAX_PROCESS_THREAD_NUM; nNum++)
	{
		if (m_eThreadStateMark[nNum] != PROCESS_THREAD_WORKING)
		{
			return nNum;
		}
	}
	return -1;
}

int CLaserLineScreen::GetFreeThreadNo(int nRobotNo, int nCamareNo)
{
	for (int nNum = 0; nNum < MAX_PROCESS_THREAD_NUM; nNum++)
	{
		if (m_veThreadStateMark[nCamareNo][nNum] != PROCESS_THREAD_WORKING)
		{
			return nNum;
		}
	}
	return -1;
}

BOOL CLaserLineScreen::CheckThreadFree(int nRobotNo)
{
	for (int nNum = 0; nNum < MAX_PROCESS_THREAD_NUM; nNum++)
	{
		if (m_eThreadStateMark[nNum] != PROCESS_THREAD_FREE)
		{
			return FALSE;
		}
	}
	return TRUE;
}

BOOL CLaserLineScreen::CheckThreadFree(int nRobotNo, int nCamareNo)
{
	for (int nNum = 0; nNum < MAX_PROCESS_THREAD_NUM; nNum++)
	{
		if (m_veThreadStateMark[nCamareNo][nNum] != PROCESS_THREAD_FREE)
		{
			return FALSE;
		}
	}
	return TRUE;
}

BOOL CLaserLineScreen::CheckThreadQuit(int nRobotNo)
{
	for (int nNum = 0; nNum < MAX_PROCESS_THREAD_NUM; nNum++)
	{
		if (m_eThreadStateMark[nNum] != PROCESS_THREAD_QUIT)
		{
			return FALSE;
		}
	}
	return TRUE;
}

BOOL CLaserLineScreen::CheckThreadQuit(int nRobotNo, int nCamareNo)
{
	for (int nNum = 0; nNum < MAX_PROCESS_THREAD_NUM; nNum++)
	{
		if (m_veThreadStateMark[nCamareNo][nNum] != PROCESS_THREAD_QUIT)
		{
			return FALSE;
		}
	}
	return TRUE;
}

BOOL CLaserLineScreen::ImageProcess(T_THREAD_PROCESS_DATA& tImgCollectData)
{
	long long lTime1 = XI_clock();
	int nRobotNo = m_pRobotDriver->m_nRobotNo;
//	T_ABS_POS_IN_BASE tPointAbsCoordInBase;
	T_ROBOT_COORS tRobotCoors;
	memset(&tRobotCoors, 0, sizeof(tRobotCoors));

	long long lTime2 = XI_clock();

	//ֱ�ӻ�ȡ�������ϵ�ȫ����
	m_pRobotDriver->m_cLog->Write("ͼ���С��%d %d",
		tImgCollectData.tImageData.ptImg->width, tImgCollectData.tImageData.ptImg->height);

	//GetLaserPointFromImageX(tImgCollectData.tImageData.ptImg, LINE_SCAN_BINARY_THRESHOLD,
	//	LINE_SCAN_STEP_2D_POINT, 20, tImgCollectData.vcpLaserPoint);


	long long lTime3 = XI_clock();

	int nLaserPointNum = tImgCollectData.vcpLaserPoint.size();
	tImgCollectData.vtLaserPoint.resize(nLaserPointNum, tRobotCoors);
	long long lTime5, lTime6;


	std::vector<T_ROBOT_COORS> vtRobotCoors(0);
//	if (false == m_pDHCameraVision->m_tCameraPare.bEnable)
//	{
//		lTime5 = XI_clock();
//		for (int nNum = 0; nNum < nLaserPointNum; nNum++)
//		{
//			if (tImgCollectData.vcpLaserPoint[nNum].y <= 10 || // 10 Ϊ��ʱ������ֻ�ͼ���� ��ͼyֵ����10
//				tImgCollectData.vcpLaserPoint[nNum].y >= tImgCollectData.tImageData.ptImg->height)
//			{
//				tRobotCoors.dZ = -1000.0; // �ȹ���̨ƽ���
//				continue;
//			}
//			m_pDHCameraVision->GetMeasurePosInBaseRoiNew(
//				m_pRobotDriver->m_strRobotName, tImgCollectData.vcpLaserPoint[nNum], eCamId, tPointAbsCoordInBase,
//				tImgCollectData.tImageData.tRobotCoord, tImgCollectData.tImageData.tRobotPulse,
//				m_pRobotDriver->m_eManipulatorType);
//			tImgCollectData.vtLaserPoint[nNum].dX = tPointAbsCoordInBase.tWeldLinePos.x + tExAxlePos.x;
//			tImgCollectData.vtLaserPoint[nNum].dY = tPointAbsCoordInBase.tWeldLinePos.y + tExAxlePos.y;
//			tImgCollectData.vtLaserPoint[nNum].dZ = tPointAbsCoordInBase.tWeldLinePos.z + tExAxlePos.z;
//
//			//����ͼ������Χ
//			if (true == LimitPointCloudRange(tImgCollectData.vtLaserPoint[nNum])) {
//				vtRobotCoors.push_back(tImgCollectData.vtLaserPoint[nNum]);
//			}
//#ifdef LINE_SCAN_SAVE_IMAGE
//			cvCircle(tImgCollectData.tImageData.ptImg, tImgCollectData.vcpLaserPoint[nNum], 6, CV_RGB(127, 127, 127), 3);
//#endif // LINE_SCAN_SAVE_IMAGE
//		}
//		lTime6 = XI_clock();
//	}
//	else
	{
		//�±궨�����ӿ�
		lTime5 = XI_clock();

		std::vector<T_ROBOT_COORS> vtResult = m_ptUnit->GetMeasureInWorldLeaser(
			m_ptUnit->m_nLineScanCameraNo, m_ptUnit->P2P(tImgCollectData.vcpLaserPoint),
			tImgCollectData.tImageData.tRobotCoord, tImgCollectData.tImageData.tRobotPulse);

		lTime6 = XI_clock();
		for (int i = 0; i < vtResult.size(); i++)
		{
#ifdef LINE_SCAN_SAVE_IMAGE
			cvCircle(tImgCollectData.tImageData.ptImg, tImgCollectData.vcpLaserPoint[i], 6, CV_RGB(127, 127, 127), 3);
#endif 
			if (tImgCollectData.vcpLaserPoint[i].y <= 10 || // 10 Ϊ��ʱ������ֻ�ͼ���� ��ͼyֵ����10
				tImgCollectData.vcpLaserPoint[i].y >= tImgCollectData.tImageData.ptImg->height)
			{
				continue;
			}
			// ������������ + ����ⲿ������ + ��ͼλ���������ƫ��
			tImgCollectData.vtLaserPoint[i].dX = vtResult.at(i).dX + tImgCollectData.tImageData.tRobotCoord.dBX + tImgCollectData.tImageData.dExAxisOffsetX;
			tImgCollectData.vtLaserPoint[i].dY = vtResult.at(i).dY + tImgCollectData.tImageData.tRobotCoord.dBY + tImgCollectData.tImageData.dExAxisOffsetY;
			tImgCollectData.vtLaserPoint[i].dZ = vtResult.at(i).dZ + tImgCollectData.tImageData.tRobotCoord.dBZ + tImgCollectData.tImageData.dExAxisOffsetZ;

			//// ��ɨ�±궨��������λ�ò�����ֻ֧�ֵ�һ������ɨ
			//if (tImgCollectData.tImageData.dExAxisOffsetX > 0 
			//	|| tImgCollectData.tImageData.dExAxisOffsetY > 0
			//	|| tImgCollectData.tImageData.dExAxisOffsetZ > 0)
			//{
			//	tImgCollectData.vtLaserPoint[i].dX += m_tPositivePointCloudCompen.x;
			//	tImgCollectData.vtLaserPoint[i].dY += m_tPositivePointCloudCompen.y;
			//	tImgCollectData.vtLaserPoint[i].dZ += m_tPositivePointCloudCompen.z;
			//}
			//else
			//{
			//	tImgCollectData.vtLaserPoint[i].dX += m_tNegativePointCloudCompen.x;
			//	tImgCollectData.vtLaserPoint[i].dY += m_tNegativePointCloudCompen.y;
			//	tImgCollectData.vtLaserPoint[i].dZ += m_tNegativePointCloudCompen.z;
			//}

			//����ͼ������Χ
			if (true == LimitPointCloudRange(tImgCollectData.vtLaserPoint[i])) {
				vtRobotCoors.push_back(tImgCollectData.vtLaserPoint[i]);
			}
		}
	}
	tImgCollectData.vtLaserPoint = vtRobotCoors;

#ifdef LINE_SCAN_SAVE_IMAGE
	CString str = "";
	str.Format("%s%d.jpg", OUTPUT_PATH + m_pRobotDriver->m_strRobotName + POINT_CLOUD_IMAGE, tImgCollectData.nDataNo);
	SaveImage(tImgCollectData.tImageData.ptImg, str);
#endif // LINE_SCAN_SAVE_IMAGE
	long long lTime4 = XI_clock();

	m_pRobotDriver->m_cLog->Write("��ͼ��ʱ��%dms �����ʱ��%dms ��άת3ά��ʱ��%dms ����%dת�����ʱ��%dms �ܺ�ʱ��%dms ͼ���С��%d %d",
		lTime2 - lTime1, lTime3 - lTime2, lTime6 - lTime5, nLaserPointNum, lTime4 - lTime3, lTime4 - lTime1,
		tImgCollectData.tImageData.ptImg->width, tImgCollectData.tImageData.ptImg->height);
	return TRUE;
}

void CLaserLineScreen::SaveLineScanResult(FILE* fRecordTimeInfo)//�����������
{
	long long lTime = XI_clock();
	for (int nCurrentFreeThreadNo = 0; nCurrentFreeThreadNo < MAX_PROCESS_THREAD_NUM; nCurrentFreeThreadNo++)
	{
		if (m_eThreadStateMark[nCurrentFreeThreadNo] != PROCESS_THREAD_FREE && m_eThreadStateMark[nCurrentFreeThreadNo] != PROCESS_THREAD_WORKING)
		{
			if (m_eThreadStateMark[nCurrentFreeThreadNo] == PROCESS_THREAD_COMPLETE)
			{
				// �˴���ÿ���̴߳���ĵ������ݱ��浽�ļ���ָ�����������ڵ��ƴ�����������
				for (int i = 0; i < m_vtImgCollectDataBuff[nCurrentFreeThreadNo].vtLaserPoint.size(); i++)
				{
					fprintf(fRecordTimeInfo, "%d%11.3lf%11.3lf%11.3lf %d %d\n",
						m_vtImgCollectDataBuff[nCurrentFreeThreadNo].nDataNo,
						m_vtImgCollectDataBuff[nCurrentFreeThreadNo].vtLaserPoint[i].dX,
						m_vtImgCollectDataBuff[nCurrentFreeThreadNo].vtLaserPoint[i].dY,
						m_vtImgCollectDataBuff[nCurrentFreeThreadNo].vtLaserPoint[i].dZ,
						m_vtImgCollectDataBuff[nCurrentFreeThreadNo].vcpLaserPoint[i].x,
						m_vtImgCollectDataBuff[nCurrentFreeThreadNo].vcpLaserPoint[i].y);
					m_vtPointCloud.push_back(cvPoint3D64f(
						m_vtImgCollectDataBuff[nCurrentFreeThreadNo].vtLaserPoint[i].dX,
						m_vtImgCollectDataBuff[nCurrentFreeThreadNo].vtLaserPoint[i].dY,
						m_vtImgCollectDataBuff[nCurrentFreeThreadNo].vtLaserPoint[i].dZ));
					if (m_vtPointCloud.size() >= m_vtPointCloud.capacity())
					{
						m_pRobotDriver->m_cLog->Write("��ɨԤ���ռ��Ѿ������� Capacity:%d Size:%d", m_vtPointCloud.capacity(), m_vtPointCloud.size());
					}
				}
			}
			m_vtImgCollectDataBuff[nCurrentFreeThreadNo].vcpLaserPoint.clear();
			m_vtImgCollectDataBuff[nCurrentFreeThreadNo].vtLaserPoint.clear();

			m_eThreadStateMark[nCurrentFreeThreadNo] = PROCESS_THREAD_FREE;
			m_nProcessNum++;
		}
	}
	m_pRobotDriver->m_cLog->Write("������ɨ��������ʱ: %d ms", XI_clock() - lTime);
}

void CLaserLineScreen::SaveLineScanResult(FILE* fRecordTimeInfo,int nCamareNo)//�����������
{
	long long lTime = XI_clock();
	for (int nCurrentFreeThreadNo = 0; nCurrentFreeThreadNo < MAX_PROCESS_THREAD_NUM; nCurrentFreeThreadNo++)
	{
		if (m_veThreadStateMark[nCamareNo][nCurrentFreeThreadNo] != PROCESS_THREAD_FREE && m_veThreadStateMark[nCamareNo][nCurrentFreeThreadNo] != PROCESS_THREAD_WORKING)
		{
			if (m_veThreadStateMark[nCamareNo][nCurrentFreeThreadNo] == PROCESS_THREAD_COMPLETE)
			{
				// �˴���ÿ���̴߳���ĵ������ݱ��浽�ļ���ָ�����������ڵ��ƴ�����������
				for (int i = 0; i < m_vvtImgCollectDataBuff[nCamareNo][nCurrentFreeThreadNo].vtLaserPoint.size(); i++)
				{
					fprintf(fRecordTimeInfo, "%d%11.3lf%11.3lf%11.3lf %d %d\n",
						m_vvtImgCollectDataBuff[nCamareNo][nCurrentFreeThreadNo].nDataNo,
						m_vvtImgCollectDataBuff[nCamareNo][nCurrentFreeThreadNo].vtLaserPoint[i].dX,
						m_vvtImgCollectDataBuff[nCamareNo][nCurrentFreeThreadNo].vtLaserPoint[i].dY,
						m_vvtImgCollectDataBuff[nCamareNo][nCurrentFreeThreadNo].vtLaserPoint[i].dZ,
						m_vvtImgCollectDataBuff[nCamareNo][nCurrentFreeThreadNo].vcpLaserPoint[i].x,
						m_vvtImgCollectDataBuff[nCamareNo][nCurrentFreeThreadNo].vcpLaserPoint[i].y);
					
					m_vvtPointCloud.at(nCamareNo).push_back(cvPoint3D64f(
						m_vvtImgCollectDataBuff[nCamareNo][nCurrentFreeThreadNo].vtLaserPoint[i].dX,
						m_vvtImgCollectDataBuff[nCamareNo][nCurrentFreeThreadNo].vtLaserPoint[i].dY,
						m_vvtImgCollectDataBuff[nCamareNo][nCurrentFreeThreadNo].vtLaserPoint[i].dZ));
					if (m_vvtPointCloud.at(nCamareNo).size() >= m_vvtPointCloud.at(nCamareNo).capacity())
					{
						m_pRobotDriver->m_cLog->Write("nCamareNo:%d ��ɨԤ���ռ��Ѿ������� Capacity:%d Size:%d", nCamareNo,m_vvtPointCloud.at(nCamareNo).capacity(), m_vvtPointCloud.at(nCamareNo).size());
					}
				}
fflush(fRecordTimeInfo);
			}
			m_vvtImgCollectDataBuff[nCamareNo][nCurrentFreeThreadNo].vcpLaserPoint.clear();
			m_vvtImgCollectDataBuff[nCamareNo][nCurrentFreeThreadNo].vtLaserPoint.clear();

			m_veThreadStateMark[nCamareNo][nCurrentFreeThreadNo] = PROCESS_THREAD_FREE;
			m_nProcessNum++;
		}
	}
	m_pRobotDriver->m_cLog->Write("������ɨ��������ʱ: %d ms", XI_clock() - lTime);
}
BOOL CLaserLineScreen::ImageProcess(T_THREAD_PROCESS_DATA& tImgCollectData , int nCamareNo)
{
	long long lTime1 = XI_clock();
	int nRobotNo = m_pRobotDriver->m_nRobotNo;
//	T_ABS_POS_IN_BASE tPointAbsCoordInBase;
	T_ROBOT_COORS tRobotCoors;
	memset(&tRobotCoors, 0, sizeof(tRobotCoors));

	long long lTime2 = XI_clock();

	GetLaserPointFromImageX(tImgCollectData.tImageData.ptImg, LINE_SCAN_BINARY_THRESHOLD,
		LINE_SCAN_STEP_2D_POINT, LINE_SCAN_LASER_WIDTH, tImgCollectData.vcpLaserPoint);


	long long lTime3 = XI_clock();

	int nLaserPointNum = tImgCollectData.vcpLaserPoint.size();
	tImgCollectData.vtLaserPoint.resize(nLaserPointNum, tRobotCoors);
	long long lTime5, lTime6;
	std::vector<T_ROBOT_COORS> vtRobotCoors(0);
//	if (false)
//	{
//		lTime5 = XI_clock();
//		for (int nNum = 0; nNum < nLaserPointNum; nNum++)
//		{
//			if (tImgCollectData.vcpLaserPoint[nNum].y <= 10 || // 10 Ϊ��ʱ������ֻ�ͼ���� ��ͼyֵ����10
//				tImgCollectData.vcpLaserPoint[nNum].y >= tImgCollectData.tImageData.ptImg->height)
//			{
//				tRobotCoors.dZ = -1000.0; // �ȹ���̨ƽ���
//				continue;
//			}
//			m_pDHCameraVision->GetMeasurePosInBaseRoiNew(
//				m_pRobotDriver->m_strRobotName, tImgCollectData.vcpLaserPoint[nNum], eCamId, tPointAbsCoordInBase,
//				tImgCollectData.tImageData.tRobotCoord, tImgCollectData.tImageData.tRobotPulse,
//				m_pRobotDriver->m_eManipulatorType);
//			tImgCollectData.vtLaserPoint[nNum].dX = tPointAbsCoordInBase.tWeldLinePos.x;
//			tImgCollectData.vtLaserPoint[nNum].dY = tPointAbsCoordInBase.tWeldLinePos.y + tImgCollectData.tImageData.dExAxisY;
//			tImgCollectData.vtLaserPoint[nNum].dZ = tPointAbsCoordInBase.tWeldLinePos.z;
//#ifdef LINE_SCAN_SAVE_IMAGE
//			cvCircle(tImgCollectData.tImageData.ptImg, tImgCollectData.vcpLaserPoint[nNum], 6, CV_RGB(127, 127, 127), 3);
//#endif // LINE_SCAN_SAVE_IMAGE
//		}
//		lTime6 = XI_clock();
//	}
//	else
	{
		//�±궨�����ӿ�		
		lTime5 = XI_clock();
		//std::vector<T_ROBOT_COORS> vtResult = m_ptUnit->GetMeasureInWorldLeaser(m_ptUnit->m_nLineScanCameraNo+ nCamareNo,
		//	m_ptUnit->P2P(tImgCollectData.vcpLaserPoint));
		std::vector<T_ROBOT_COORS> vtResult = m_ptUnit->GetMeasureInWorldLeaser(
			m_ptUnit->m_nLineScanCameraNo + nCamareNo, m_ptUnit->P2P(tImgCollectData.vcpLaserPoint),
			tImgCollectData.tImageData.tRobotCoord, tImgCollectData.tImageData.tRobotPulse);

		m_pRobotDriver->m_cLog->Write(" �����%d ͼ���С��%d %d λ�����꣺%.3lf %.3lf", nCamareNo,
			tImgCollectData.tImageData.ptImg->width, tImgCollectData.tImageData.ptImg->height, 
			tImgCollectData.tImageData.tRobotCoord.dBY, tImgCollectData.tImageData.dExAxisOffsetY);
		lTime6 = XI_clock();
		for (int i = 0; i < vtResult.size(); i++)
		{
#ifdef LINE_SCAN_SAVE_IMAGE
			//cvCircle(tImgCollectData.tImageData.ptImg, tImgCollectData.vcpLaserPoint[i], 6, CV_RGB(127, 127, 127), 3);
#endif 
			if (tImgCollectData.vcpLaserPoint[i].y <= 10 || // 10 Ϊ��ʱ������ֻ�ͼ���� ��ͼyֵ����10
				tImgCollectData.vcpLaserPoint[i].y >= tImgCollectData.tImageData.ptImg->height)
			{
				continue;
			}
			tImgCollectData.vtLaserPoint[i].dX = vtResult.at(i).dX + tImgCollectData.tImageData.tRobotCoord.dBX + tImgCollectData.tImageData.dExAxisOffsetX;
			tImgCollectData.vtLaserPoint[i].dY = vtResult.at(i).dY + tImgCollectData.tImageData.tRobotCoord.dBY + tImgCollectData.tImageData.dExAxisOffsetY;
			tImgCollectData.vtLaserPoint[i].dZ = vtResult.at(i).dZ + tImgCollectData.tImageData.tRobotCoord.dBZ + tImgCollectData.tImageData.dExAxisOffsetZ;
			// ��ɨ�±궨��������λ�ò���

			double adCmpDir[3] = { 1.0 , 1.0, 1.0};
			if (m_nScanDir < 0) // ɨ�跽��ͬ ɨ�跽��Ĳ������� ��ͬ
			{
				adCmpDir[m_ptUnit->m_nLinedScanAxisNo - 1] *= -1.0;
			}

			// ��ɨ�±궨��������λ�ò�����ֻ֧�ֵ�һ������ɨ
			if (tImgCollectData.tImageData.dExAxisOffsetX > 0
				|| tImgCollectData.tImageData.dExAxisOffsetY > 0
				|| tImgCollectData.tImageData.dExAxisOffsetZ > 0)
			{
				tImgCollectData.vtLaserPoint[i].dX += m_tPositivePointCloudCompen.x;
				tImgCollectData.vtLaserPoint[i].dY += m_tPositivePointCloudCompen.y;
				tImgCollectData.vtLaserPoint[i].dZ += m_tPositivePointCloudCompen.z;
			}
			else
			{
				tImgCollectData.vtLaserPoint[i].dX += m_tNegativePointCloudCompen.x;
				tImgCollectData.vtLaserPoint[i].dY += m_tNegativePointCloudCompen.y;
				tImgCollectData.vtLaserPoint[i].dZ += m_tNegativePointCloudCompen.z;
			}

			tImgCollectData.vtLaserPoint[i].dX += (m_ptUnit->m_vtCameraPara[m_ptUnit->m_nLineScanCameraNo + nCamareNo].tPointCloudCompen.x * adCmpDir[0]);
			tImgCollectData.vtLaserPoint[i].dY += (m_ptUnit->m_vtCameraPara[m_ptUnit->m_nLineScanCameraNo + nCamareNo].tPointCloudCompen.y * adCmpDir[1]);
			tImgCollectData.vtLaserPoint[i].dZ += (m_ptUnit->m_vtCameraPara[m_ptUnit->m_nLineScanCameraNo + nCamareNo].tPointCloudCompen.z * adCmpDir[2]);

			//����ͼ������Χ
			if (true == LimitPointCloudRange(tImgCollectData.vtLaserPoint[i])) {
				vtRobotCoors.push_back(tImgCollectData.vtLaserPoint[i]);
			}
		}
	}
	tImgCollectData.vtLaserPoint = vtRobotCoors;
#ifdef LINE_SCAN_SAVE_IMAGE
	CString str = "";
	str.Format("%s%d.jpg", OUTPUT_PATH + m_pRobotDriver->m_strRobotName + "\\" + POINT_CLOUD_IMAGE, tImgCollectData.nDataNo);
	SaveImage(tImgCollectData.tImageData.ptImg, str);
#endif // LINE_SCAN_SAVE_IMAGE
	long long lTime4 = XI_clock();

	m_pRobotDriver->m_cLog->Write("��ͼ��ʱ��%dms �����ʱ��%dms ��άת3ά��ʱ��%dms ����%dת�����ʱ��%dms �ܺ�ʱ��%dms ͼ���С��%d %d",
		lTime2 - lTime1, lTime3 - lTime2, lTime6 - lTime5, nLaserPointNum, lTime4 - lTime3, lTime4 - lTime1,
		tImgCollectData.tImageData.ptImg->width, tImgCollectData.tImageData.ptImg->height);
	return TRUE;
}

bool CLaserLineScreen::LimitPointCloudRange(T_ROBOT_COORS tDealPoint)
{
	if (tDealPoint.dX > m_dRange_XMax || tDealPoint.dX < m_dRange_XMin) {
		return false;
	}
	if (tDealPoint.dY > m_dRange_YMax || tDealPoint.dY < m_dRange_YMin) {
		return false;
	}
	if (tDealPoint.dZ > m_dRange_ZMax || tDealPoint.dZ < m_dRange_ZMin) {
		return false;
	}
	return true;
}

bool CLaserLineScreen::LoadLineScanParam(int scanNum)
{
	int  nCurTableNo = 0;
	CString table = "Table0";
	COPini opini;
	opini.SetFileName(DATA_PATH + m_ptUnit->m_tContralUnit.strUnitName + LINE_SCAN_PARAM);
	opini.SetSectionName("CurUseTableNo");
	opini.ReadString("CurUseTableNo", &nCurTableNo);
	table.Format("Table%d", nCurTableNo);

	opini.SetSectionName(table);
	opini.ReadString("YMaxCar", &m_tRobotLimitation.drYMaxCar);
	opini.ReadString("YMinCar", &m_tRobotLimitation.drYMinCar);
	opini.ReadString("YMaxRobot", &m_tRobotLimitation.drYMaxRobot);
	opini.ReadString("YMinRobot", &m_tRobotLimitation.drYMinRobot);
	opini.ReadString("XMax", &m_tRobotLimitation.drXMax);
	opini.ReadString("XMin", &m_tRobotLimitation.drXMin);
	opini.ReadString("ZMax", &m_tRobotLimitation.drZMax);
	opini.ReadString("ZMin", &m_tRobotLimitation.drZMin);
	opini.ReadString("dMinVel", &m_tRobotLimitation.drdMinVel);
	opini.ReadString("dAcc", &m_tRobotLimitation.drdAcc);
	opini.ReadString("dDec", &m_tRobotLimitation.drdDec);
	opini.ReadString("ScanSpeed", &m_tRobotLimitation.drScanSpeed);
	opini.ReadString("RunSpeed", &m_tRobotLimitation.drRunSpeed);
	opini.ReadString("TableZ", &m_tRobotLimitation.drTableZ);
	opini.ReadString("TableY", &m_tRobotLimitation.drTableY);
	switch (scanNum)
	{
	case 0:
		opini.ReadString("startpulse.nS", &m_tStartPlus.nSPulse);
		opini.ReadString("startpulse.nL", &m_tStartPlus.nLPulse);
		opini.ReadString("startpulse.nU", &m_tStartPlus.nUPulse);
		opini.ReadString("startpulse.nR", &m_tStartPlus.nRPulse);
		opini.ReadString("startpulse.nB", &m_tStartPlus.nBPulse);
		opini.ReadString("startpulse.nT", &m_tStartPlus.nTPulse);
		opini.ReadString("startpulse.lBX", &m_tStartPlus.lBXPulse);
		opini.ReadString("startpulse.lBY", &m_tStartPlus.lBYPulse);
		opini.ReadString("startpulse.lBZ", &m_tStartPlus.lBZPulse);
		break;
	case 1:
		opini.ReadString("startpulse1.nS", &m_tStartPlus.nSPulse);
		opini.ReadString("startpulse1.nL", &m_tStartPlus.nLPulse);
		opini.ReadString("startpulse1.nU", &m_tStartPlus.nUPulse);
		opini.ReadString("startpulse1.nR", &m_tStartPlus.nRPulse);
		opini.ReadString("startpulse1.nB", &m_tStartPlus.nBPulse);
		opini.ReadString("startpulse1.nT", &m_tStartPlus.nTPulse);
		opini.ReadString("startpulse1.lBX", &m_tStartPlus.lBXPulse);
		opini.ReadString("startpulse1.lBY", &m_tStartPlus.lBYPulse);
		opini.ReadString("startpulse1.lBZ", &m_tStartPlus.lBZPulse);
		break;
	case 2:
		opini.ReadString("startpulse2.nS", &m_tStartPlus.nSPulse);
		opini.ReadString("startpulse2.nL", &m_tStartPlus.nLPulse);
		opini.ReadString("startpulse2.nU", &m_tStartPlus.nUPulse);
		opini.ReadString("startpulse2.nR", &m_tStartPlus.nRPulse);
		opini.ReadString("startpulse2.nB", &m_tStartPlus.nBPulse);
		opini.ReadString("startpulse2.nT", &m_tStartPlus.nTPulse);
		opini.ReadString("startpulse2.lBX", &m_tStartPlus.lBXPulse);
		opini.ReadString("startpulse2.lBY", &m_tStartPlus.lBYPulse);
		opini.ReadString("startpulse2.lBZ", &m_tStartPlus.lBZPulse);
		break;

	}
	opini.ReadString("ScanStartCarLoction", &m_dScanStartCarLoction);
	opini.ReadString("Scanlength", &m_dScanLength);
	opini.ReadString("ScanDir", &m_nScanDir);
	opini.ReadString("ExAxisEnable", &m_bExAxisEnable);
	if (scanNum == 2)
	{
		double length = m_dScanStartCarLoction;
		m_dScanStartCarLoction = length + m_dScanLength;
		m_dScanLength = length - m_dScanStartCarLoction;
	}
	if (m_dScanLength > 0)
	{
		m_nScanDir = labs(m_nScanDir);
	}
	else
	{
		m_nScanDir = -1*labs(m_nScanDir);
	}
	//opini.ReadString("XScanSize", &dXScanSize);
	//opini.ReadString("ZScanSize", &dZScanSize);
	//opini.ReadString("RXScanSize", &RXScanSize);
	//opini.ReadString("RYScanSize", &RYScanSize);
	//opini.ReadString("RZScanSize", &RZScanSize);

	//AppendLineScanCoord(m_bExAxisEnable, m_nScanDir, m_tStartPlus, m_dScanStartCarLoction);
	m_ptUnit->GetRobotCtrl()->RobotKinematics(m_tStartPlus, m_ptUnit->GetRobotCtrl()->m_tTools.tGunTool, m_tStartWorldCoors);

//	// ��ɨλ�ú���̬ ���� ��ʱ
	
	m_tStartWorldCoors.dX += (!m_bExAxisEnable && 1 == labs(m_nScanDir)) ? m_dScanStartCarLoction : 0.0;
	m_tStartWorldCoors.dY += (!m_bExAxisEnable && 2 == labs(m_nScanDir)) ? m_dScanStartCarLoction : 0.0;
	m_tStartWorldCoors.dZ += (!m_bExAxisEnable && 3 == labs(m_nScanDir)) ? m_dScanStartCarLoction : 0.0;
	m_tStartWorldCoors.dBX += (m_bExAxisEnable && 1 == labs(m_nScanDir)) ? m_dScanStartCarLoction : 0.0;
	m_tStartWorldCoors.dBY += (m_bExAxisEnable && 2 == labs(m_nScanDir)) ? m_dScanStartCarLoction : 0.0;
	m_tStartWorldCoors.dBZ += (m_bExAxisEnable && 3 == labs(m_nScanDir)) ? m_dScanStartCarLoction : 0.0;
	bool bRtn = m_ptUnit->GetRobotCtrl()->RobotInverseKinematics(m_tStartWorldCoors, m_tStartPlus, m_ptUnit->GetRobotCtrl()->m_tTools.tGunTool, m_tStartPlus);
	CHECK_BOOL_BOX_RTN_UNIT(bRtn, "��ɨ�������ת��ʧ��");

	double dScanDir = m_nScanDir > 0 ? 1.0 : -1.0;
	m_tEndWorldCoors = m_tStartWorldCoors;
	m_tEndWorldCoors.dX += (!m_bExAxisEnable && 1 == labs(m_nScanDir)) ? m_dScanLength * dScanDir : 0.0;
	m_tEndWorldCoors.dY += (!m_bExAxisEnable && 2 == labs(m_nScanDir)) ? m_dScanLength * dScanDir : 0.0;
	m_tEndWorldCoors.dZ += (!m_bExAxisEnable && 3 == labs(m_nScanDir)) ? m_dScanLength * dScanDir : 0.0;
	m_tEndWorldCoors.dBX += (m_bExAxisEnable && 1 == labs(m_nScanDir)) ? m_dScanLength * dScanDir : 0.0;
	m_tEndWorldCoors.dBY += (m_bExAxisEnable && 2 == labs(m_nScanDir)) ? m_dScanLength * dScanDir : 0.0;
	m_tEndWorldCoors.dBZ += (m_bExAxisEnable && 3 == labs(m_nScanDir)) ? m_dScanLength * dScanDir : 0.0;

	bRtn = m_ptUnit->GetRobotCtrl()->RobotInverseKinematics(m_tEndWorldCoors, m_tStartPlus, m_ptUnit->GetRobotCtrl()->m_tTools.tGunTool, m_tEndPlus);
	CHECK_BOOL_BOX_RTN_UNIT(bRtn, "��ɨ�յ�����ת��ʧ��");


	//������ɨ���ƴ���Χ
	opini.ReadString("Range_XMax", &m_dRange_XMax);
	opini.ReadString("Range_XMin", &m_dRange_XMin);
	opini.ReadString("Range_YMax", &m_dRange_YMax);
	opini.ReadString("Range_YMin", &m_dRange_YMin);
	opini.ReadString("Range_ZMax", &m_dRange_ZMax);
	opini.ReadString("Range_ZMin", &m_dRange_ZMin);

	opini.SetSectionName("PositivePointCloudCompen");
	opini.ReadString("X", &m_tPositivePointCloudCompen.x);
	opini.ReadString("Y", &m_tPositivePointCloudCompen.y);
	opini.ReadString("Z", &m_tPositivePointCloudCompen.z);

	opini.SetSectionName("NegativePointCloudCompen");
	opini.ReadString("X", &m_tNegativePointCloudCompen.x);
	opini.ReadString("Y", &m_tNegativePointCloudCompen.y);
	opini.ReadString("Z", &m_tNegativePointCloudCompen.z);

	return true;
}

CLog* CLaserLineScreen::GetLog()
{
	return m_ptUnit->GetLog();
}