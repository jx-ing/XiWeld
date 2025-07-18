// ScanWork.cpp: implementation of the CScanWork class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "ScanInitModule.h"
#include "res\resource.h"
//#include "AxisName.h"
#include <algorithm>
#include <string>
#include "Apps/PLib/BasicFunc/Log.h"
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <opencv2\opencv.hpp>
#include <cstring>
#include "GFPGJointLib.h"



//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CScanInitModule::CScanInitModule(CRobotDriverAdaptor *pRobotDriver, CUnit* ptUnit, UINT* pUnTracePointCloudProcess, BOOL* pbSaveScanImg, E_FLIP_MODE* peScanFlipMode, E_SCANMODE* peScanMode, UINT* pUnSingleCameraThrdNum)
	: m_pRobotDriver(pRobotDriver),m_ptUnit(ptUnit)
{
	__flag_self_create = pUnTracePointCloudProcess == NULL ? TRUE : FALSE;
	Init(pUnTracePointCloudProcess,pbSaveScanImg,peScanFlipMode,peScanMode);
	LoadEquipmentParam();
	m_IsOpenJudgeEnd_ProcessSearch = false;
}

CScanInitModule::~CScanInitModule()
{
	UnInit();
}

void CScanInitModule::Init(UINT* pUnTracePointCloudProcess, BOOL* pbSaveScanImg, E_FLIP_MODE* peScanFlipMode, E_SCANMODE* peScanMode,UINT* pUnSingleCameraThrdNum)
{
	m_eThreadStatus = SCANWORK_THREAD_STATUS_START;
	m_eScanMode     = COARSE_SCAN_MODE;

	m_dStepDis           = 2.0;
	m_nTheoryNum         = 300;

	// mm/min
	m_dCoarseScanVel     = 0.0;
	m_dAccurateScanVel   = 0.0;

	m_nStartPointFilterNum = 5;
	m_dStartPointDelRatio  = 0.3;
	
	m_bIfRightDown = TRUE;
	m_bIfLeftDown = TRUE;

	m_bStartPointCapCur = FALSE;
	m_bStartPointProcese = FALSE;
	m_bFixedPointScan = FALSE;

	m_vtSearchStartPoint3d.clear();

	// ͼ��ʹ�� Ĭ�ϲ���
	m_ePieceClass = GROUP_STAND_DIP_NS::E_TI_PIECE;
	m_iBaseLineLenInZone = 50;
	m_iMoveLineLenInZone = 30;
	m_iRoiWidSize = 100;
	m_iIfEndConditionIdx = 1;

	m_bScanStartTwoLineOnStandBoard = FALSE;

	m_bCameraFindWeldStart = FALSE;
	m_nEndPointCapCursor = 0;  //�˵��ͼ���
	m_nEndPointProcCursor = 0; //����ͼ��

    m_bEndPointCapCur = FALSE;     //�жϲ�ͼ�߳��Ƿ����
    m_bEndPointProcess = FALSE;   //�жϴ����߳��Ƿ����
    m_bRealTimeTrackstate = FALSE; //�����߳�״̬
    m_bDynamicCapturestate = FALSE;//��ͼ�߳�״̬
    m_bDynamicProcstate = FALSE;   //�����߳�״̬
	m_bOpenTrackingStatus = FALSE;	

	//���߳��������
	m_pMapImgProcOutput = NULL;
	m_ptStartPointInfo = new T_START_POINT_INFO[MAX_ARRAY_NUM_START];
	InitStartPointInfoArray();
	m_pcWrapAngleParam = new CWrapAngleParam(m_pRobotDriver->m_strRobotName);
	m_hGetCameraCtrl = CreateSemaphore(NULL, 1, 1, "GetCameraCtrl");
	
	m_eEndPointType = GROUP_STAND_DIP_NS::E_PIECE_END;
	m_sMoveJobName = "CONTIMOVANY";
	if (__flag_self_create) {
		m_pUnSingleCameraThrdNum = new UINT(4);
		m_pUnTracePointCloudProcess = new UINT(0);
		m_pSaveTraceScanPic = new BOOL(FALSE);
		m_peScanFlipMode = new E_FLIP_MODE(E_FLIP_NONE);
		m_peScanMode = new E_SCANMODE(E_SCANMODE_2DPOINT);
		COPini opini;
		opini.SetFileName(OPTIONAL_FUNCTION);
		opini.SetSectionName("SCANMODULE_PARAM");
		opini.ReadString("SingleCameraThreadNum", (int*)m_pUnSingleCameraThrdNum);
		opini.ReadString("SavePointCloud", (int*)m_pUnTracePointCloudProcess);
		opini.ReadString("SavePicture", (int*)m_pSaveTraceScanPic);
		opini.ReadString("ScanMode", (int*)m_peScanMode);
	}
	else {
		m_pUnSingleCameraThrdNum = pUnSingleCameraThrdNum;
		m_pUnTracePointCloudProcess = pUnTracePointCloudProcess;
		m_pSaveTraceScanPic = pbSaveScanImg;
		m_peScanFlipMode = peScanFlipMode;
		m_peScanMode = peScanMode;
	}

}

void CScanInitModule::SetScanParam(/*bool bSavePointCloud, bool bSaveScanImg, */bool bNaturalPop, E_FLIP_MODE eFlipMode, E_SCANMODE eScanMode, CString sMoveJobName)
{
	/**m_pUnTracePointCloudProcess = bSavePointCloud;
	*m_pSaveTraceScanPic = bSaveScanImg;*/
	*m_peScanFlipMode = eFlipMode;
	*m_peScanMode = eScanMode;
	m_sMoveJobName = sMoveJobName;
}

void CScanInitModule::UnInit()
{
	m_pShowImage = NULL;
	if (__flag_self_create) {
		delete m_pUnSingleCameraThrdNum;
		delete m_pUnTracePointCloudProcess;
		delete m_pSaveTraceScanPic;
		delete m_peScanFlipMode;
		delete m_peScanMode;
	}
	m_pUnSingleCameraThrdNum = NULL;
	m_pUnTracePointCloudProcess = NULL;
	m_pSaveTraceScanPic = NULL;
	m_peScanFlipMode = NULL;
	m_peScanMode = NULL;
	delete m_pcWrapAngleParam;
	m_pcWrapAngleParam = NULL;
	//���ͷ�ͼƬ���ڴ�
	ClearStartPointInfoArray();
	//���ͷ�ͼƬ��Ϣ������ڴ�
	delete[] m_ptStartPointInfo;
	m_ptStartPointInfo = NULL;
	CloseHandle(m_hGetCameraCtrl);
}

void CScanInitModule::SetParam
(
	double dXStart, double dYStart, double dXEnd, double dYEnd,
	E_SCAN_MODE eScanMode
)
{
	m_eScanMode = eScanMode;

	m_dXStart = dXStart;
	m_dYStart = dYStart;
	m_dXEnd   = dXEnd;
	m_dYEnd   = dYEnd;

	double dDist = TwoPointDis(m_dXStart, m_dYStart, m_dXEnd, m_dYEnd);
	m_nTheoryNum = dDist / m_dStepDis;
	WriteLog("ɨ�貽��:%lf ���� %lf m_nTheoryNum %d",m_dStepDis,dDist,m_nTheoryNum);
}

void CScanInitModule::SetScanStartParam(double dStartUAngle, XI_POINT startPoint, int nStartPointType, int nStartShape, BOOL bFixedPointScan,BOOL bStartIfWrap)
{
	m_dStartUAngle = dStartUAngle;
	m_tStartPoint = startPoint;
	m_nStartPointType = nStartPointType;
	m_nStartPointShape = nStartShape;
	m_nStartIfWrap = bStartIfWrap;
	m_bFixedPointScan = bFixedPointScan;
	if ( FALSE == m_bFixedPointScan)
	{
		TranslateDrawingToImageParam(); // ����ͼֽ���ݳ�ʼ�� ͼ��Ҫ�õ������� �� ��Ҫ���⴦���õ�������
	}
	else
	{
		ResumeDrawingToImageParam();
	}
	WriteLog("SetWhichPlaceClass���� eWhichPlaceClass:%d m_bFixedPointScan:%d", m_ePieceClass, m_bFixedPointScan);
}
	
void CScanInitModule::TranslateDrawingToImageParam()
	{
	/*
	 *  IfPieceStartOrEndPoint(IplImage *Src, const double &dVelocity, double &dCompensateValue, CvPoint &cpKeyPoint, 
	 *  						  E_START_OR_END eStartOrEnd = E_PIECE_END, 
	 *  						  E_WHICH_PIECE_CLASS ePieceClass = E_R_PIECE, 
	 *  						  int iBaseLineLenInZone = 50, int iMoveLineLenInZone = 30, 
	 *  						  int iRoiWidSize = 100, int iIfEndConditionIdx = 1);
	 * 
	 * ����յ���״	ͼֽ��ʾ  ePieceClass����
	 * 
	 *    T��		   0	  E_TI_PIECE	
	 *    ˮ��		   4	  E_TI_PIECE
	 *   �����ཻ	   5	  E_TI_PIECE
	 *  �����������   6	  E_TI_PIECE
	 *   �װ�R��	   3	  E_R_PIECE
	 *   ����R��	   2	  E_R_PIECE
	 *  F��(ֱ�Ǳ�)	   1	  E_R_PIECE
	 * 
	 *	 ����		iBaseLineLenInZone����		iMoveLineLenInZone����		iRoiWidSize����		iIfEndConditionIdx����
	 * 
	 *	���¿�				50							30						  100					1
	 *	���¿�				30							125						  250					5
	 *	��ˮ��				50							50						  250					10
	 */
	WriteLog("Before:m_nStartPointShape:%d, m_ePieceClass:%d, m_iBaseLineLenInZone:%d, m_iMoveLineLenInZone:%d, m_iRoiWidSize:%d, m_iIfEndConditionIdx:%d"
		,m_nStartPointShape, m_ePieceClass, m_iBaseLineLenInZone, m_iMoveLineLenInZone, m_iRoiWidSize, m_iIfEndConditionIdx);
	//XiMessageBox("Before:m_nStartPointShape:%d, m_ePieceClass:%d, m_iBaseLineLenInZone:%d, m_iMoveLineLenInZone:%d, m_iRoiWidSize:%d, m_iIfEndConditionIdx:%d"
	//	,m_nStartPointShape, m_ePieceClass, m_iBaseLineLenInZone, m_iMoveLineLenInZone, m_iRoiWidSize, m_iIfEndConditionIdx);
	if ((0 == m_nStartPointShape) || (4 == m_nStartPointShape) || (5 == m_nStartPointShape)||(6 == m_nStartPointShape))
	{
		m_ePieceClass = GROUP_STAND_DIP_NS::E_TI_PIECE;
	}
	else if ((3 == m_nStartPointShape) || (2 == m_nStartPointShape) || (1 == m_nStartPointShape))
	{
		m_ePieceClass = GROUP_STAND_DIP_NS::E_R_PIECE;
	}
	else
	{
		XUI::MesBox::PopOkCancel("�����״����m_nStartPointShape = {0}", m_nStartPointShape);
	}
	
	if ((4 == m_nStartPointShape)) // ��ˮ�׻��������ཻ ��ʹ�ù�ˮ�ײ���
	{
		//m_iBaseLineLenInZone = 40;
		//m_iMoveLineLenInZone = 40;
		//m_iRoiWidSize = 150;
		//m_iIfEndConditionIdx = 9;
		//if (0 == m_ptUnit->GetRobotCtrl()->m_nRobotNo)
		//{
		//	/*m_iBaseLineLenInZone = 40;
		//	m_iMoveLineLenInZone = 55;
		//	m_iRoiWidSize = 150;
		//	m_iIfEndConditionIdx = 1;*/

		//	m_iBaseLineLenInZone = 90;
		//	m_iMoveLineLenInZone = 70;
		//	m_iRoiWidSize = 150;
		//	m_iIfEndConditionIdx = 9;
		//}
		//else
		{
			m_iBaseLineLenInZone = 50;
			m_iMoveLineLenInZone = 40;
			m_iRoiWidSize = 150;
			m_iIfEndConditionIdx = 9;
		}
		

		m_bScanStartTwoLineOnStandBoard = FALSE;
		m_dScanStartExtendLength = 0/*30*/;
	}
	else if (5 == m_nStartPointShape)
	{
		m_iBaseLineLenInZone = 90;
		m_iMoveLineLenInZone = 80;
		m_iRoiWidSize = 150;
		m_iIfEndConditionIdx = 6;
		m_bScanStartTwoLineOnStandBoard = FALSE;
		m_dScanStartExtendLength = 0/*30*/;
	}
	else if((3 == m_nStartPointShape) || (2 == m_nStartPointShape) || (1 == m_nStartPointShape)|| (0 == m_nStartPointShape))// Ŀǰû�������Ƿ����¿ڣ���ʹ�����¿ڲ���
	{
		m_iBaseLineLenInZone = 50;
		m_iMoveLineLenInZone = 30;
		m_iRoiWidSize = 100;
		m_iIfEndConditionIdx = 1;
		m_bScanStartTwoLineOnStandBoard = FALSE;

		m_dScanStartExtendLength = 0;
	}
	else if (6 == m_nStartPointShape)//��������
	{
		m_iBaseLineLenInZone = 30;
		m_iMoveLineLenInZone = 0;
		m_iRoiWidSize = 250;
		m_iIfEndConditionIdx = 7;
		m_bScanStartTwoLineOnStandBoard = FALSE;
		m_dScanStartExtendLength = 0;
	}
	WriteLog("After:m_nStartPointShape:%d, m_ePieceClass:%d, m_iBaseLineLenInZone:%d, m_iMoveLineLenInZone:%d, m_iRoiWidSize:%d, m_iIfEndConditionIdx:%d"
		,m_nStartPointShape, m_ePieceClass, m_iBaseLineLenInZone, m_iMoveLineLenInZone, m_iRoiWidSize, m_iIfEndConditionIdx);
	//XiMessageBox("After:m_nStartPointShape:%d, m_ePieceClass:%d, m_iBaseLineLenInZone:%d, m_iMoveLineLenInZone:%d, m_iRoiWidSize:%d, m_iIfEndConditionIdx:%d"
	//	,m_nStartPointShape, m_ePieceClass, m_iBaseLineLenInZone, m_iMoveLineLenInZone, m_iRoiWidSize, m_iIfEndConditionIdx);
}

void CScanInitModule::ResumeDrawingToImageParam()
{
	m_ePieceClass = GROUP_STAND_DIP_NS::E_R_PIECE;
	m_iBaseLineLenInZone = 50;
	m_iMoveLineLenInZone = 30;
	m_iRoiWidSize = 100;
	m_iIfEndConditionIdx = 1;
	m_bScanStartTwoLineOnStandBoard = FALSE;
	m_dScanStartExtendLength = 0;
}
	
BOOL CScanInitModule::IsComplete()
{
	if(SCANWORK_THREAD_STATUS_COMPLETE == m_eThreadStatus)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

void CScanInitModule::ClearStartPointInfoArray()
{
	for (int _index = 0; _index < MAX_ARRAY_NUM_START; _index++) {
		if (m_ptStartPointInfo[_index].img != NULL) {
			cvReleaseImage(&m_ptStartPointInfo[_index].img);
			m_ptStartPointInfo[_index].img = NULL;
		}
	}
	return;
}

void CScanInitModule::InitStartPointInfoArray()
{
	WaitForSingleObject(m_hGetCameraCtrl,INFINITE);
	CDHGigeImageCapture* tDHTraceCamer = (CDHGigeImageCapture*)m_ptUnit->GetCameraCtrl(m_ptUnit->m_nMeasureCameraNo);
	ReleaseSemaphore(m_hGetCameraCtrl,1,NULL);
	for (int _index = 0; _index < MAX_ARRAY_NUM_START; _index++) {
		m_ptStartPointInfo[_index].img= cvCreateImage(cvSize(tDHTraceCamer->m_tCameraPara.tDHCameraDriverPara.nMaxWidth,
															 tDHTraceCamer->m_tCameraPara.tDHCameraDriverPara.nMaxHeight),
			IPL_DEPTH_8U,
			1);
	}
	
}

BOOL CScanInitModule::UpdateTheoryInfoAccurate(CRobotDriverAdaptor *pRobotDriver)
{
	CString strPath;
	strPath.Format(".\\WeldData\\%s\\", pRobotDriver->m_strRobotName);
	//string strPathAdr = strPath;
	std::vector<T_ROBOT_COORS> vtStartCoors;
	std::vector<XI_POINT> vtReferCoors;
	vtReferCoors.clear();
	vtStartCoors.clear();
	std::ifstream fileCurntLugNo(strPath+"CoutStartPointInfoFileInWorldCopy.txt");
	std::ofstream fileCurnt(strPath+"CoutStartPointInfoFileInWorldSmooth2.txt");//����˲�������*/
	std::ofstream fileCurnt3(".\\CoutStartPointInfoFileInWorldSmooth3.txt");//����˲�������*/
	int nSeq;
	int nDelPointTolNum = 0;
    double dMachineY = 0.0;
    if (m_bIfOpenExternal)
    {
        ////////dMachineY = m_MoveCtrlModule.GetPositionDis(pRobotDriver->m_nAXIS_X);
    }
	pRobotDriver->m_cLog->Write("��ǹ������꣺%lf", dMachineY);
	T_ROBOT_COORS tStartCoors;
	m_nStartIfWrap = 1;
	// ��������ɶ˵�����㣬ɾ��������(һ���һ����ƫ��)
	if (0 == m_nStartIfWrap)//��ǹ��㲻����
	{
		nDelPointTolNum = 5;
	}
	for (int nDelPointNum = 0; nDelPointNum < nDelPointTolNum; nDelPointNum++)
	{
		fileCurntLugNo >> nSeq>>tStartCoors.dX>>tStartCoors.dY >> tStartCoors.dZ;
		pRobotDriver->m_cLog->Write("ɾ����%d���㣺%d, %.3lf, %.3lf, %.3lf",nDelPointNum,nSeq,tStartCoors.dX,tStartCoors.dY,tStartCoors.dZ);
	}
	while (!fileCurntLugNo.eof())
	{
		fileCurntLugNo >> nSeq>>tStartCoors.dX>>tStartCoors.dY >> tStartCoors.dZ;
		XI_POINT tPointCoor = { tStartCoors.dX , tStartCoors.dY+ dMachineY , tStartCoors.dZ };
		pRobotDriver->m_cLog->Write("tStartCoors %d %lf %lf %lf", nSeq, tStartCoors.dX, tStartCoors.dY+ dMachineY, tStartCoors.dZ);
		vtReferCoors.push_back(tPointCoor);		
		tStartCoors.dY += dMachineY;
		vtStartCoors.push_back(tStartCoors);
	}
	fileCurntLugNo.close();
	//���㷽��
	//FitLineParamRansac(vtReferCoors,40);
	int width= vtReferCoors.size();//�����˲�����
	pRobotDriver->m_cLog->Write("�˲�ǰ��ֵ %d",width);
	if (width<7)
	{
		XUI::MesBox::PopOkCancel("{0}�Ż�������ʼ��ɨ������̫�٣��޷�����˲������ݣ�{1}", pRobotDriver->m_nRobotNo,width);
		return FALSE;
	}

	//double dis = CalcPolarAngle(vtReferCoors.at(width - 1).x - vtReferCoors.at(0).x,
	//	vtReferCoors.at(width - 1).y - vtReferCoors.at(0).y);
	//
	double dis = TwoPointDis(vtReferCoors.at(width - 1).x, vtReferCoors.at(width - 1).y, vtReferCoors.at(width - 1).z, vtReferCoors.at(0).x,
		 vtReferCoors.at(0).y, vtReferCoors.at(0).z);
	if (dis < 48)
	{
		XUI::MesBox::PopOkCancel("{0}�Ż������˲�ǰ��ֵ�쳣 {1}", pRobotDriver->m_nRobotNo, dis);
		return FALSE;
	}

//	CvPoint3D64f cp3DOutPoint; 
//	CvPoint3D64f cp3DInPoint; 
//	BOOL bIfOutPointValid;
//	CvPoint3D64f cpTrackDir; 
//	CvPoint2D64f cpTrackNormalDir;
//	BOOL bIfDirValid;
	double dDirWinMaxLen = 60;
//	XI_POINT tPoint;
	m_vtScanInitWeldLineInWorldPoints.clear();
////////	TrackFilter_Init(m_ptUnit->m_nRobotSmooth,3.0);
////////	//m_pLaserVision->CamId2ImageProcess(E_LEFT_ROBOT_LEFT_CAM_H)->SetInit_For_FilterCurve();//��ʼ���˲�������
////////    double dTrackVerDegree;
////////	double dTrackVerDegree6th;
////////	//������
////////	double dInitComp = pRobotDriver->m_dGunToEyeCompenX;
////////    double dAbjustHeight = pRobotDriver->m_dGunToEyeCompenZ; // ���ϲ��� ���²���
////////	pRobotDriver->m_cLog->Write("��ʼ�β�����%.3lf %.3lf", dInitComp, dAbjustHeight);
////////#if 1
////////	std::vector<TrackFilter_Node> CoutTrackPoint;
////////	BOOL bNormalDir = FALSE;
////////	if (-1 == pRobotDriver->m_nRobotInstallDir)
////////	{
////////		bNormalDir = TRUE;
////////	}
////////	for (int index = 0; index < width; index++)
////////	{	
////////		CoutTrackPoint.clear();
////////		cp3DInPoint.x =vtStartCoors[index].dX;
////////		cp3DInPoint.y =vtStartCoors[index].dY;
////////		cp3DInPoint.z =vtStartCoors[index].dZ;
////////		
////////		CoutTrackPoint = TrackFilter_FilterCurvePointInPointOut(pRobotDriver->m_nRobotSmooth,cp3DInPoint.x, cp3DInPoint.y, cp3DInPoint.z, bNormalDir);
////////		pRobotDriver->m_cLog->Write("��ʼ���˲�:%d %d", index,CoutTrackPoint.size());
////////		for (int nSize = 0;nSize<CoutTrackPoint.size();nSize++)
////////		{
////////			// ��ʼ�β���
////////			dTrackVerDegree = atan2(CoutTrackPoint[nSize].normalDir_.y_, CoutTrackPoint[nSize].normalDir_.x_) * 180.0 / PI;
////////			cp3DOutPoint.x = CoutTrackPoint[nSize].pnt_.x_ + (dInitComp * CosD(dTrackVerDegree));
////////			cp3DOutPoint.y = CoutTrackPoint[nSize].pnt_.y_ + (dInitComp * SinD(dTrackVerDegree));
////////			cp3DOutPoint.z = CoutTrackPoint[nSize].pnt_.z_ + (dAbjustHeight);
////////			pRobotDriver->m_cLog->Write("index:%d, �˲��󲹳���  %.3lf  %.3lf  %.3lf dTrackVerDegree:%lf %lf %lf  ", index, cp3DOutPoint.x, cp3DOutPoint.y, cp3DOutPoint.z, dTrackVerDegree, CoutTrackPoint[nSize].normalDir_.y_, CoutTrackPoint[nSize].normalDir_.x_);
////////			tPoint.x = cp3DOutPoint.x;
////////			tPoint.y = cp3DOutPoint.y;
////////			tPoint.z = cp3DOutPoint.z;
////////
////////			pRobotDriver->m_cLog->Write("��ʼ�� ���˲���%lf %lf %lf ", cp3DOutPoint.x, cp3DOutPoint.y, cp3DOutPoint.z);
////////			m_vtScanInitWeldLineInWorldPoints.push_back(tPoint);
////////		}
////////	}
////////	int nSize = m_vtScanInitWeldLineInWorldPoints.size();
////////		
////////	pRobotDriver->m_cLog->Write("�˲�����ֵ��%d ",nSize);
////////	if (nSize <=2)
////////	{
////////
////////		XiMessageBox("�˲�����ֵ�쳣 %d", nSize);
////////		return FALSE;
////////	}
////////
////////    int n;
////////	for (n = 0;n<nSize;n++)
////////	{
////////		fileCurnt << n<<" "<<m_vtScanInitWeldLineInWorldPoints[n].x<<" "<<m_vtScanInitWeldLineInWorldPoints[n].y <<" "<< m_vtScanInitWeldLineInWorldPoints[n].z<<std::endl;
////////	}
////////	fileCurnt.close();
////////	
////////
////////	pRobotDriver->m_cLog->Write("ɨ��������� size:%d. %lf %lf %lf ", nSize, m_vtScanInitWeldLineInWorldPoints.at(0).x, m_vtScanInitWeldLineInWorldPoints.at(0).y, m_vtScanInitWeldLineInWorldPoints.at(0).z);
////////#endif
	m_dRealWeldStartPosX = m_vtScanInitWeldLineInWorldPoints[0].x;
	m_dRealWeldStartPosY = m_vtScanInitWeldLineInWorldPoints[0].y;
	m_dRealWeldStartPosZ = m_vtScanInitWeldLineInWorldPoints[0].z;
	pRobotDriver->m_cLog->Write("ʵ�ʵ����:%lf %lf %lf", m_dRealWeldStartPosX, m_dRealWeldStartPosY, m_dRealWeldStartPosZ);
	return TRUE;
}

void CScanInitModule::FixedStartProc(CRobotDriverAdaptor *pRobotDriver)
{
////////    WriteLog("����ɨ��");
////////	CString strPath;
////////	strPath.Format(".\\WeldData\\%s\\%d_", pRobotDriver->m_strRobotName, m_eEndPointType);
////////	std::string str = strPath;
////////	E_CAM_ID eCamId;
////////	GROUP_STAND_DIP_NS::CImageProcess *pImageProcess;
////////	CCameraDriverAdaptor *pCameraDriver;
////////	IplImage *pImageBuff;
////////
////////	CString strSavePath = "";
////////	CString strImgPathName = "";
////////	if (0 == pRobotDriver->m_nRobotNo)
////////	{
////////		eCamId = E_LEFT_ROBOT_LEFT_CAM_H;
////////		strSavePath.Format("LeftGun_LeftCam\\Src\\EndPoint_%d\\", m_eEndPointType);
////////	}
////////	else if (1 == pRobotDriver->m_nRobotNo)
////////	{
////////		eCamId = E_RIGHT_ROBOT_LEFT_CAM_H;
////////		strSavePath.Format("RightGun_LeftCam\\Src\\EndPoint_%d\\", m_eEndPointType);
////////	}
////////	else
////////	{
////////		XiMessageBox("JudgeStartProcû��ָ����ȷ��CamId!");
////////
////////		return;
////////	}
////////
////////	T_SAVE_IMAGE tSaveImage /*= new T_SAVE_IMAGE*/;
////////	for (int nS = 0; nS < SAVE_IMAGE_THREAD_MUN; nS++)
////////	{
////////		tSaveImage.bSaveImageStaus[nS] = FALSE;
////////	}
////////	tSaveImage.nImageNo = 0;
////////
////////	CString strCamId = m_pLaserVision->CamId2String(eCamId);
////////	pImageProcess = m_pLaserVision->CamId2ImageProcess(eCamId);
////////	pCameraDriver = m_pLaserVision->CamId2CamDrv(eCamId);
////////	pImageBuff = m_pLaserVision->CamId2ImageBuff(eCamId);
////////
////////	GROUP_STAND_DIP_NS::E_START_OR_END eStartOrEnd = m_eEndPointType;
////////	double dCurPosX = 0;
////////	double dCurPosY = 0;
////////	double dCamerCoorX = 0;
////////	double dCamerCoorY = 0;
////////	double dCamerCoorZ = 0;
////////	double dOffsetStartPoint = 0;
////////	double dScanVel = 500;//
////////	m_nTimeStart = 0;//
////////	std::ofstream CoutEndPointInfoFileInWorld(strPath + "CoutEndPointInfoFileInWorld.txt");
////////	T_ABS_POS_IN_BASE tPointAbsCoordInBase;
////////	T_CART_COOR dMachineCoorsProcess;//��¼������
////////	m_bCameraFindWeldStart = FALSE;
////////	m_bEndPointProcess = FALSE;
////////	CvPoint cpKeyPoint;
////////
////////	//��ʱ���
////////	m_nStartPointShape = 4;
////////	m_bFixedPointScan = FALSE;
////////	//------------------------
////////	if (FALSE == m_bFixedPointScan)
////////	{
////////		TranslateDrawingToImageParam(); // ����ͼֽ���ݳ�ʼ�� ͼ��Ҫ�õ������� �� ��Ҫ���⴦���õ�������
////////	}
////////	else
////////	{
////////		ResumeDrawingToImageParam();
////////	}
////////	int nSaveImgNo = 0;
////////	while (FALSE == m_bCameraFindWeldStart)
////////	{
////////		if (pRobotDriver->m_eThreadStatus == INCISEHEAD_THREAD_STATUS_STOPPED)
////////		{
////////			break;
////////		}
////////		if (0 == pRobotDriver->GetIntVar(1))
////////		{
////////			m_bCameraFindWeldStart = TRUE;
////////			break;
////////		}
////////		// �����ȡ������������Ч���Ż�(��ע�͵�)
////////		if (m_nEndPointCapCursor < 2) // �Ӳɼ���������ͼ��ʼ
////////		{
////////			continue;
////////		}
////////
////////		m_nEndPointProcCursor = m_nEndPointCapCursor - 1;
////////
////////		if (NULL == m_ptStartPointInfo[m_nEndPointProcCursor % MAX_ARRAY_NUM_START].img)
////////		{
////////			pRobotDriver->m_cLog->Write("kong!!!!!!!");
////////		}
////////		clock_t tTime = XI_clock();
////////		pImageProcess->IfPieceStartOrEndPoint
////////		(
////////			m_ptStartPointInfo[m_nEndPointProcCursor % MAX_ARRAY_NUM_START].img,
////////			dScanVel, dOffsetStartPoint, cpKeyPoint, eStartOrEnd,
////////			m_ePieceClass, m_iBaseLineLenInZone, m_iMoveLineLenInZone, m_iRoiWidSize, m_iIfEndConditionIdx
////////		);
////////		pRobotDriver->m_cLog->Write("����㴦��ʱ�䣺%d %d %d", XI_clock() - tTime, cpKeyPoint.x, cpKeyPoint.y);
////////		//continue;
////////		tSaveImage.nImageNo++;
////////		strImgPathName.Format("%sForwardScan\\%d.jpg", strSavePath, tSaveImage.nImageNo);
////////		tSaveImage.strId = strImgPathName;
////////
////////		if (pRobotDriver->m_nStartSaveImage)
////////		{
////////			tSaveImage.pImage = m_ptStartPointInfo[m_nEndPointProcCursor % MAX_ARRAY_NUM_START].img;
////////			AfxBeginThread(ThreadSaveImage, &tSaveImage);
////////		}
////////		if (cpKeyPoint.x > 1 && cpKeyPoint.x < 2591 && cpKeyPoint.y>1 && cpKeyPoint.y < 1993)
////////		{
////////			cvCvtColor(m_ptStartPointInfo[m_nEndPointProcCursor % MAX_ARRAY_NUM_START].img, m_pShowImage, CV_GRAY2RGB);
////////			cvCircle(m_pShowImage, cpKeyPoint, 8, CV_RGB(255, 0, 0), 4);
////////		}	
////////		//������������ϵ�µĵ�
////////		m_pLaserVision->GetMeasurePosInBaseNew(pRobotDriver->m_strRobotName, cpKeyPoint, eCamId, tPointAbsCoordInBase, m_ptStartPointInfo[m_nEndPointProcCursor % MAX_ARRAY_NUM_START].tRobotCoors,
////////			m_ptStartPointInfo[m_nEndPointProcCursor % MAX_ARRAY_NUM_START].tRobotPulse, pRobotDriver->m_eManipulatorType);
////////		tPointAbsCoordInBase.tWeldLinePos.y += m_ptStartPointInfo[m_nEndPointProcCursor % MAX_ARRAY_NUM_START].dMachineCoors.dY;
////////		//��¼��������ϵ�µĵ�
////////		CoutEndPointInfoFileInWorld << m_nEndPointProcCursor << " " << tPointAbsCoordInBase.tWeldLinePos.x << " " << tPointAbsCoordInBase.tWeldLinePos.y << " " << tPointAbsCoordInBase.tWeldLinePos.z << std::endl;
////////		Sleep(30);
////////		DoEvent();
////////	}
////////	//���ݷ����ҳ���ȷ���
////////	pImageProcess->IfPieceStartOrEndPoint(
////////		m_ptStartPointInfo[m_nEndPointProcCursor % MAX_ARRAY_NUM_START].img,
////////		dScanVel, dOffsetStartPoint, cpKeyPoint, eStartOrEnd,
////////		m_ePieceClass, m_iBaseLineLenInZone, m_iMoveLineLenInZone, m_iRoiWidSize, m_iIfEndConditionIdx);
////////
////////	m_pLaserVision->GetMeasurePosInBaseNew(pRobotDriver->m_strRobotName, cpKeyPoint, eCamId, tPointAbsCoordInBase, m_ptStartPointInfo[m_nEndPointProcCursor % MAX_ARRAY_NUM_START].tRobotCoors,
////////		m_ptStartPointInfo[m_nEndPointProcCursor % MAX_ARRAY_NUM_START].tRobotPulse, pRobotDriver->m_eManipulatorType);
////////	tPointAbsCoordInBase.tWeldLinePos.y += m_ptStartPointInfo[m_nEndPointProcCursor % MAX_ARRAY_NUM_START].dMachineCoors.dY;
////////
////////	XI_POINT  tWeldLinepointInWorld = tPointAbsCoordInBase.tWeldLinePos;
////////	m_tWeldLinepointInWorld = tWeldLinepointInWorld;
////////	m_tDynamicScanEndPoint = m_tWeldLinepointInWorld;
////////	pRobotDriver->m_cLog->Write("���Ѷ˵㣺X:%lf Y:%lf Z:%lf", tWeldLinepointInWorld.x, tWeldLinepointInWorld.y, tWeldLinepointInWorld.z);
////////	FILE *pfStartPoint = fopen("AA_StartUpBoardPoints.txt", "a+");
////////	fprintf(pfStartPoint, "%d%11.3lf%11.3lf%11.3lf\n", -2,
////////		tWeldLinepointInWorld.x, tWeldLinepointInWorld.y, tWeldLinepointInWorld.z);
////////	fclose(pfStartPoint);
////////	// ������̬ɨ�����ʱ �ж��Ƿ�ʹ���㷨����������Զ����Ϊ���
////////	CoutEndPointInfoFileInWorld.close();
////////	//������ʼɨ������ȥ����ʵ�����������ݻ������ϵ��
////////	SwapVector(tWeldLinepointInWorld, str + "CoutEndPointInfoFileInWorld.txt", str + "CoutStartPointInfoFileInWorldCopy.txt");
////////	//CoordinatePointProcessFuncation(tWeldLinepointInWorld, str + "CoutEndPointInfoFileInWorld.txt", str + "CoutStartPointInfoFileInWorldCopy.txt");
////////	pRobotDriver->HoldOn();
////////	pRobotDriver->m_cLog->Write("%s ��ʼ����ͼ�߳�", pRobotDriver->m_strRobotName);
////////	while (TRUE)
////////	{
////////		if (pRobotDriver->m_eThreadStatus == INCISEHEAD_THREAD_STATUS_STOPPED)
////////		{
////////			break;
////////		}
////////		int nThreathNum = 0;
////////		for (int ns = 0; ns < SAVE_IMAGE_THREAD_MUN; ns++)
////////		{
////////			if (FALSE == tSaveImage.bSaveImageStaus[ns])
////////			{
////////				nThreathNum++;
////////			}
////////		}
////////		if (SAVE_IMAGE_THREAD_MUN == nThreathNum)
////////		{
////////			pRobotDriver->m_cLog->Write("%s ��ͼ�߳̿���", pRobotDriver->m_strRobotName);
////////			break;
////////		}
////////		DoEvent();
////////		Sleep(10);
////////	}
////////	m_bEndPointProcess = TRUE;
////////	pRobotDriver->m_cLog->Write("%s�˵㴦���߳��˳�:%d", pRobotDriver->m_strRobotName, m_bEndPointProcess);
////////	//delete tSaveImage;
	return;
}

void CScanInitModule::CoordinatePointProcessFuncation(XI_POINT tWeldLinepointCoor,std::string FileNameCin,std::string FileNameCout)
{
	std::ifstream fileCurntLugNoCin(FileNameCin.c_str());
	std::ofstream fileCurntLugNoCout(FileNameCout.c_str());
	XI_POINT	tStartScanPoint;//��¼��ʼ��ɨ���
	XI_POINT    tFirstPoint; // ��ʼ�����һ����
	std::vector<XI_POINT> tvRecordStartScanPoint;
	tvRecordStartScanPoint.clear();
	int nSeq;
	fileCurntLugNoCin >> nSeq>>tStartScanPoint.x>>tStartScanPoint.y >> tStartScanPoint.z; //�ȶ�һ��
	tvRecordStartScanPoint.push_back(tStartScanPoint);
	tFirstPoint = tStartScanPoint;
	double dMaxDis = TwoPointDis(tFirstPoint.x,tFirstPoint.y,tWeldLinepointCoor.x,tWeldLinepointCoor.y);
	WriteLog("��ʼ�ξ����յ���Զ���룺%.3lf",dMaxDis);
	while (!fileCurntLugNoCin.eof())//ɸѡ�����������ĵ�����
	{
		fileCurntLugNoCin >> nSeq>>tStartScanPoint.x>>tStartScanPoint.y >> tStartScanPoint.z;
		
		double dDisPoint = TwoPointDis(tStartScanPoint.x,tStartScanPoint.y,tFirstPoint.x,tFirstPoint.y);
		//WriteLog("������ʼ�㣺%.3lf �����룺%.3lf",dDisPoint, dMaxDis);
		if (dDisPoint < dMaxDis)
		{
			tvRecordStartScanPoint.push_back(tStartScanPoint);
			//WriteLog("��ʼ�β�������");
		}
		else
		{
			break;
		}
	}

	tStartScanPoint.x = tWeldLinepointCoor.x;
	tStartScanPoint.y = tWeldLinepointCoor.y;
	tvRecordStartScanPoint.push_back(tStartScanPoint);
	int nSize = tvRecordStartScanPoint.size();
    int n = 0;
    int m = 0;
	for (n = 0,m=nSize-1; n < m;n++,m--)
	{
		std::swap(tvRecordStartScanPoint[n],tvRecordStartScanPoint[m]);//�����ɨ����������㵽��β��ڷ�
	}
	
	XI_POINT prePoint = tvRecordStartScanPoint[0];//��һ����
	
	for ( n = 0;n< nSize ;n++)//����ת�Ժ�λ���ݷŵ��ı���
	{

		if (0 == n)
		{
			fileCurntLugNoCout<<n<<" "<<tvRecordStartScanPoint[n].x<<" "<<tvRecordStartScanPoint[n].y<<" "<<tvRecordStartScanPoint[n].z;
		}
		else
		{
// 			// ����������С��1.5���˵�
			if(1.5 > TwoPointDis(prePoint.x,prePoint.y,tvRecordStartScanPoint[n].x,tvRecordStartScanPoint[n].y))
			{
				continue;
			}
			// ��������Ҫ30mm����̬����Ҫһ�ΰ��ǣ�����128�����ۼӰ��ǳ��ȵ����ݣ�����ֻ�����۹�ϵ���ȵ���ʼ��110
			if ((m_dHandEyeDis+20 + m_dScanStartExtendLength+fabs(m_dScanStartWrapLength)) < TwoPointDis(tvRecordStartScanPoint[n].x,tvRecordStartScanPoint[n].y,tWeldLinepointCoor.x,tWeldLinepointCoor.y))
			{
				continue;
			}
			fileCurntLugNoCout<<"\n"<<n<<" "<<tvRecordStartScanPoint[n].x<<" "<<tvRecordStartScanPoint[n].y<<" "<<tvRecordStartScanPoint[n].z;
			prePoint = tvRecordStartScanPoint[n];
		}
		
	}
	
	fileCurntLugNoCin.close();
    fileCurntLugNoCout.close();
}
void CScanInitModule::SwapVector(XI_POINT tStartPoint, std::string FileNameCin,std::string FileNameCout)
{
	std::ifstream fileCurntLugNoCin(FileNameCin.c_str());
	std::ofstream fileCurntLugNoCout(FileNameCout.c_str());
	XI_POINT	tStartScanPoint;//��¼��ʼ��ɨ���
	std::vector<XI_POINT> tvRecordStartScanPoint;
	tvRecordStartScanPoint.clear();
	
	int nSeq;
	XI_POINT tLastPoint;
	fileCurntLugNoCin >> nSeq>>tStartScanPoint.x>>tStartScanPoint.y >> tStartScanPoint.z;
	tvRecordStartScanPoint.push_back(tStartScanPoint);
	tLastPoint = tStartScanPoint;
	double dMaxDis = TwoPointDis(tLastPoint.x,tLastPoint.y,tStartPoint.x,tStartPoint.y);
	
	while (!fileCurntLugNoCin.eof()) // ���˵�����Ժ�ĵ�
	{
		fileCurntLugNoCin >> nSeq >> tStartScanPoint.x >> tStartScanPoint.y >> tStartScanPoint.z;
		if (dMaxDis > TwoPointDis(tStartScanPoint.x, tStartScanPoint.y, tLastPoint.x, tLastPoint.y))
		{
			tvRecordStartScanPoint.push_back(tStartScanPoint);
		}
		else
		{
			continue;
		}
	}
	int nSize = tvRecordStartScanPoint.size();
	int n = 0;
	int m = 0;
	for (n = 0, m = nSize - 1; n < m; n++, m--)
	{
		std::swap(tvRecordStartScanPoint[n], tvRecordStartScanPoint[m]);//�����ɨ����������㵽��β��ڷ�
	}
	
	XI_POINT tTempPoint = tvRecordStartScanPoint[0];
	for ( n = 0;n< nSize ;n++)//����ת�Ժ�λ���ݷŵ��ı���
	{
		if (0 == n)
		{
			fileCurntLugNoCout<<n<<" "<<tvRecordStartScanPoint[n].x<<" "<<tvRecordStartScanPoint[n].y<<" "<<tvRecordStartScanPoint[n].z;
		}
		else
		{
			//if (1.5 > TwoPointDis(tvRecordStartScanPoint[n].x,tvRecordStartScanPoint[n].y,tTempPoint.x,tTempPoint.y))
			//{
			//	continue;
			//}
			//// ��ǰ�����ɶ���ɨ�账����ã�ֻ��һ�����۹�ϵ�ľ���
			//if (100 < TwoPointDis(tvRecordStartScanPoint[n].x,tvRecordStartScanPoint[n].y,tStartPoint.x,tStartPoint.y))
			//{
			//	continue;
			//}
			fileCurntLugNoCout<<"\n"<<n<<" "<<tvRecordStartScanPoint[n].x<<" "<<tvRecordStartScanPoint[n].y<<" "<<tvRecordStartScanPoint[n].z;
			tTempPoint = tvRecordStartScanPoint[n];
			
		}
		
	}
	
	fileCurntLugNoCin.close();
	fileCurntLugNoCout.close();
}
BOOL CScanInitModule::DynamicScanningEndpoint(CRobotDriverAdaptor *pRobotDriver)
{
	if (g_bLocalDebugMark) return TRUE;

	pRobotDriver->SetIntVar(1, 1);
	m_nEndPointCapCursor = 0;  //�˵��ͼ���
	m_nEndPointProcCursor = 0; //����ͼ��
	m_bEndPointCapCur = FALSE;
	m_bEndPointProcess = FALSE;
	m_bSuccessScanFlag = FALSE;
	//��ɨ��ε�ǰ��Ҫ��ʼ����������б��
	//��ʼ������  ��24/01/02���ݾ�������
	//if (!ScanEndpointLockInit(pRobotDriver)) return FALSE;
	long long tTime = XI_clock();

	//��ͼ�̣߳���ͼ�߳�ӵ�дθ����ȼ���
	AfxBeginThread(ThreadDynamicScanningCapture, this, THREAD_PRIORITY_HIGHEST - 1);
	if (m_bFixedPointScan){
		XiMessageBoxOk("��δ��ֲ�������ڴ���");
		return FALSE;
		//AfxBeginThread(ThreadFixedStartProc, this); // ����ɨ�����
	}
	else{
		//�����߳�(�����߳�ӵ��������ȼ�)
		AfxBeginThread(ThreadDynamicScanningProc, this);
	}

	//��������Ƿ��˳�����ͼ�ʹ����̶߳��˳������˳���
	while (TRUE)
	{
		BOOL bFlag(FALSE);

		//R0LevelLock
		while (m_MutexCapThrdExitFlag.trylock() == false) UsSleep(2); {
			//R0LevelLock
			while (m_MutexProcThrdExitFlag.trylock() == false) UsSleep(2); {
				if (m_bEndPointCapCur && m_bEndPointProcess) bFlag = TRUE;
			}m_MutexProcThrdExitFlag.unlock();
		}m_MutexCapThrdExitFlag.unlock();
		if (!bFlag) {
			DoEvent();
			Sleep(40);
			continue;
		}
		break;
	}
	//��û���ѵ��˵�

	BOOL tresult = TRUE;
	//�˵㼰���������ݴ���
	if (0 == (int)m_eEndPointType)//��ʼ��
	{
		//tresult = DynamicEndPointResultPro(pRobotDriver);
	}
	else if (1 == (int)m_eEndPointType)//��β��
	{

	}
	pRobotDriver->HoldOff();
	pRobotDriver->ServoOn();
	m_bCameraFindWeldStart = FALSE;
	RecordRunTime(pRobotDriver, tTime, "��������");
	if (!tresult || !m_bSuccessScanFlag) XiMessageBox("�˵�����ʧ��");
	return (tresult && m_bSuccessScanFlag);
}
BOOL CScanInitModule::DynamicRealTimeTracking()
{
	AfxBeginThread(ThreadRealTimeTracking, this);
	return TRUE;
}
UINT CScanInitModule::ThreadRealTimeTracking(void *pParam)
{
    CScanInitModule *pcMyObj = (CScanInitModule *)pParam;
    if (TRUE == pcMyObj->m_bRealTimeTrackstate)
    {
        XiMessageBox("�ظ���������߳�");
        return FALSE;
    }
    pcMyObj->m_bRealTimeTrackstate = TRUE;
	//pcMyObj->RealTimeTracking(pcMyObj->m_pRobotDriver);
	pcMyObj->RealTimeTrackingNew(pcMyObj->m_pRobotDriver);// ���嵥��
	//pcMyObj->RealTimeTrackingNew_H(pcMyObj->m_pRobotDriver);
    pcMyObj->m_bRealTimeTrackstate = FALSE;
    return TRUE;
}
UINT CScanInitModule::ThreadFlipImage(void *pParam)
{
	CScanInitModule *pcMyObj = (CScanInitModule *)pParam;
	if (TRUE == pcMyObj->m_bFlipedImage)
	{
		XiMessageBox("hh");
		return FALSE;
	}
	pcMyObj->m_bFlipedImage = TRUE;
	pcMyObj->FlipImageNew(pcMyObj->m_pRobotDriver);
	pcMyObj->m_bFlipedImage = FALSE;
	return 0;
}

UINT CScanInitModule::ThreadDynamicScanningCapture(void *pParam)
{
    CScanInitModule *pcMyObj = (CScanInitModule *)pParam;
    if (TRUE == pcMyObj->m_bDynamicCapturestate)
    {
        XiMessageBox("�ظ�����˵��ͼ�߳�");
        return FALSE;
    }
    pcMyObj->m_bDynamicCapturestate = TRUE;
    pcMyObj->DynamicCapture(pcMyObj->m_pRobotDriver);
    pcMyObj->m_bDynamicCapturestate = FALSE;
    return 0;
}

//2023.2.26 ��Ӹ��ٶ�����������
UINT CScanInitModule::ThreadDynamicScanningProc(void *pParam)
{
	CScanInitModule *pcMyObj = (CScanInitModule *)pParam;
	if (TRUE == pcMyObj->m_bDynamicProcstate)
	{
		pcMyObj->m_pRobotDriver->HoldOn();
		XiMessageBoxOk("�ظ�����˵�ͼƬ�߳�");
		return FALSE;
	}
	pcMyObj->m_bDynamicProcstate = TRUE;
	//		�Ϸ���			
	//pcMyObj->DynamicEndPointProc(pcMyObj->m_pRobotDriver);  
	//      2023.09.26 ���߳��Ż�ͼ���� 2023.10.25 

	int nScansensitivity = 20, nThreshold = 0;
	if (*pcMyObj->m_pUnTracePointCloudProcess > 0)
	{
		nScansensitivity = 20;
		if (*pcMyObj->m_pUnTracePointCloudProcess == 2)
		{
			nThreshold = 3;
		}
	}
	//��ʱ��������ʹ���˶�ģʽ
	pcMyObj->ImgProcDistribute(pcMyObj->m_pRobotDriver,
							   *pcMyObj->m_pUnSingleCameraThrdNum,
							   nScansensitivity,
							   nThreshold,
							   0,
							   10000,
							   *pcMyObj->m_pSaveTraceScanPic == TRUE,
							   *pcMyObj->m_pUnTracePointCloudProcess > 0,
							   pcMyObj->m_eEndPointType,
							   *pcMyObj->m_peScanFlipMode,
							   *pcMyObj->m_peScanMode,
							   pcMyObj->m_sMoveJobName);
	pcMyObj->m_bDynamicProcstate = FALSE;
	return 0;
}

void CScanInitModule::FlipImageNew(CRobotDriverAdaptor *pRobotDriver)
{
	Mat image_org = cvarrToMat(m_ptStartPointInfo[m_nEndPointCapCursor % MAX_ARRAY_NUM_START].img);
	Mat image_fliped;
	flip(image_org, image_fliped, 0);
	m_ptStartPointInfo[m_nEndPointCapCursor % MAX_ARRAY_NUM_START].img = cvCreateImage(cvSize(image_fliped.cols, image_fliped.rows), IPL_DEPTH_8U, 1);
}
/**
 * @brief �������ܣ�΢�뼶˯��
 * @param nUs��΢�� 
*/
void CScanInitModule::UsSleep(const unsigned int& nUs)
{
	LARGE_INTEGER fre;
	if (QueryPerformanceFrequency(&fre))
	{
		LARGE_INTEGER run, pre, curr;
		//ȡ΢�뼶ִ�м�����Ԫ
		run.QuadPart = fre.QuadPart * nUs / 1000000;
		//ȡִ��ǰʱ�Ӽ�����
		QueryPerformanceCounter(&pre);
		do 
		{
			QueryPerformanceCounter(&curr);
		} while (curr.QuadPart - pre.QuadPart < run.QuadPart);
	}
	return;
}
XI_POINT CScanInitModule::Camera2DTo3D(CvPoint tCamera2DPoint, T_CAMREA_PARAM tCamParam, XiCV_Image_Processing* pXiCvProcess)
{
	XiCV_Image_Processing* pXcvImgProc;
	if (pXiCvProcess == NULL) {
		pXcvImgProc = &XiCV_Image_Processing(tCamParam.tDHCameraDriverPara.nMaxWidth, tCamParam.tDHCameraDriverPara.nMaxHeight);
	}
	else {
		pXcvImgProc = pXiCvProcess;
	}
	
	XiLaserLightParamNode tLaserLightParam = XiLaserLightParamNode();

	XI_POINT tKeyPoint3D = XI_POINT();
	XI_POINT tCamCenter3D = XI_POINT();

	tLaserLightParam.l_dx =tCamParam.tHandEyeCaliPara.tCameraInnerPara.dPixelX;
	tLaserLightParam.l_dy =tCamParam.tHandEyeCaliPara.tCameraInnerPara.dPixelY;
	tLaserLightParam.l_f =tCamParam.tHandEyeCaliPara.tCameraInnerPara.dFocal;
	tLaserLightParam.l_len =tCamParam.tHandEyeCaliPara.tCameraInnerPara.dBaseLineLength;
	tLaserLightParam.l_ctan =tCamParam.tHandEyeCaliPara.tCameraInnerPara.dCtanBaseLineAngle;

	CvPoint3D64f cptKeyPnt3D = pXcvImgProc->KeyPoint2DTo3D(tCamera2DPoint, 1, tLaserLightParam);
	tKeyPoint3D.x = cptKeyPnt3D.x;
	tKeyPoint3D.y = cptKeyPnt3D.y;
	tKeyPoint3D.z = cptKeyPnt3D.z;


	CvPoint      tCamCentrePnt = cvPoint(tCamParam.tDHCameraDriverPara.nMaxWidth / 2, tCamParam.tDHCameraDriverPara.nMaxHeight / 2);
	CvPoint3D64f cpCamCenterPnt3D = pXcvImgProc->KeyPoint2DTo3D(tCamCentrePnt, 1, tLaserLightParam);
	tCamCenter3D.x = cpCamCenterPnt3D.x;
	tCamCenter3D.y = cpCamCenterPnt3D.y;
	tCamCenter3D.z = cpCamCenterPnt3D.z;
	XI_POINT tCornerPoint = XI_POINT();

	tCornerPoint.x = (tKeyPoint3D.x - tCamCenter3D.x) * (1.0);
	tCornerPoint.y = (tKeyPoint3D.y - tCamCenter3D.y) * (1.0);
	tCornerPoint.z = (tKeyPoint3D.z - tCamCenter3D.z) * (1.0);
	return tCornerPoint;
}

T_ROBOT_COORS CScanInitModule::TransCameraToRobot(XI_POINT tCameraCoor,T_CAMREA_PARAM tCamParam, GROUP_ROBOT_ABS_COORS_TRANS::XiRobotAbsCoorsTrans cAbsCoorsTrans, E_MANIPULATOR_TYPE eManipType, T_ROBOT_COORS tCaptureRobotCoor, T_ANGLE_PULSE tCaptureRobotPulse)
{
	T_ABS_POS_IN_BASE tAbsCoor = cAbsCoorsTrans.MeasureWeldLinePointInBase(
		tCameraCoor,tCaptureRobotCoor, tCaptureRobotPulse, eManipType, tCamParam.tHandEyeCaliPara
	);
	tCaptureRobotCoor.dX = tAbsCoor.tWeldLinePos.x;
	tCaptureRobotCoor.dY = tAbsCoor.tWeldLinePos.y;
	tCaptureRobotCoor.dZ = tAbsCoor.tWeldLinePos.z;
	return tCaptureRobotCoor;
}

T_ROBOT_COORS CScanInitModule::TransCamera2DToBase(CvPoint tCamera2DPoint, T_CAMREA_PARAM tCamParm, GROUP_ROBOT_ABS_COORS_TRANS::XiRobotAbsCoorsTrans cAbsCoorsTrans,
	E_MANIPULATOR_TYPE eManipType,T_ROBOT_COORS tCaptureRobotCoor, T_ANGLE_PULSE tCaptureRobotPulse, T_ABS_POS_IN_BASE & tPointAbsCoordInBase, XiCV_Image_Processing* pXiCvProcess)
{
	tCamera2DPoint.x += tCamParm.tDHCameraDriverPara.nRoiOffsetX;
	tCamera2DPoint.y += tCamParm.tDHCameraDriverPara.nRoiOffsetY;

	XI_POINT _InPtn = Camera2DTo3D(tCamera2DPoint, tCamParm, pXiCvProcess);
	T_ROBOT_COORS tRetCoors = TransCameraToRobot(_InPtn,tCamParm,cAbsCoorsTrans,eManipType,tCaptureRobotCoor,tCaptureRobotPulse);
	tPointAbsCoordInBase.tWeldGunPos.x = tCaptureRobotCoor.dX;
	tPointAbsCoordInBase.tWeldGunPos.y = tCaptureRobotCoor.dY;
	tPointAbsCoordInBase.tWeldGunPos.z = tCaptureRobotCoor.dZ;
	tPointAbsCoordInBase.tWeldLinePos.x = tRetCoors.dX;
	tPointAbsCoordInBase.tWeldLinePos.y = tRetCoors.dY;
	tPointAbsCoordInBase.tWeldLinePos.z = tRetCoors.dZ;
	return tRetCoors;
}
void CScanInitModule::DynamicCapture(CRobotDriverAdaptor* pRobotDriver)
{
	T_ROBOT_COORS tRobotCurCoord;
	T_ANGLE_PULSE tRobotCurPulses;
	T_CART_COOR dMachineCoors;//��¼������
	long long nTime = XI_clock();
	vector<int> vnParam(3);
	vnParam[0] = IMWRITE_JPEG_QUALITY;
	vnParam[1] = 40;
	vnParam[2] = 1;
	//��ȡ�������ָ��
	WaitForSingleObject(m_hGetCameraCtrl, INFINITE);
	CDHGigeImageCapture* pDHCamDrv = (CDHGigeImageCapture*)m_ptUnit->GetCameraCtrl(m_ptUnit->m_nMeasureCameraNo);
	ReleaseSemaphore(m_hGetCameraCtrl, 1, NULL);
	//����ѭ����Ψһ����Ϊ m_bCameraFindWeldStart == TRUE
	while (TRUE)
	{
		//���ﲻ�ӻ��������ܳ��ַ���Ȩ���쳣��2024/01/03 ���ν��飩
		if (pRobotDriver->m_eThreadStatus == INCISEHEAD_THREAD_STATUS_STOPPED) break;

		//R1LevelLock
		m_MutexFindEndPointFlag.lock(); {
			if (m_bCameraFindWeldStart == TRUE) {
				m_MutexFindEndPointFlag.unlock();
				break;
			}
		}m_MutexFindEndPointFlag.unlock();

		long long lStartTime = XI_clock();
		nTime = XI_clock();
		//��ȡ��ͼλ��
		tRobotCurCoord = pRobotDriver->GetCurrentPos();
		tRobotCurPulses = pRobotDriver->GetCurrentPulse();
		//���¿����ʱ
		dMachineCoors.dX = tRobotCurCoord.dBX;
		dMachineCoors.dY = tRobotCurCoord.dBY;
		dMachineCoors.dZ = tRobotCurCoord.dBZ;

		//R2LevelLock
		m_MutexPicCapture.lock(); {
			m_ptStartPointInfo[m_nEndPointCapCursor % MAX_ARRAY_NUM_START].tRobotCoors = tRobotCurCoord;
			m_ptStartPointInfo[m_nEndPointCapCursor % MAX_ARRAY_NUM_START].tRobotPulse = tRobotCurPulses;
			m_ptStartPointInfo[m_nEndPointCapCursor % MAX_ARRAY_NUM_START].dMachineCoors = dMachineCoors;
			//��ȡͼƬ
			pDHCamDrv->CaptureImage(m_ptStartPointInfo[m_nEndPointCapCursor % MAX_ARRAY_NUM_START].img, 1);
			//cvCopyImage(pDHCamDrv->CaptureImage(FALSE), m_ptStartPointInfo[m_nEndPointCapCursor % MAX_ARRAY_NUM_START].img);
		}m_MutexPicCapture.unlock();

		Sleep(20);
		//R2LevelLock
		m_MutexPicCapture.lock(); {
			m_nEndPointCapCursor++;
#ifdef __ScanInitModule_ConsoleDbgPrint
			TRACE("ɨ���ͼ%d��ʱ%dms\n", m_nEndPointCapCursor, XI_clock() - nTime);
#endif
			pRobotDriver->m_cLog->Write("ɨ���ͼ%d��ʱ%dms", m_nEndPointCapCursor, XI_clock() - nTime);
		}m_MutexPicCapture.unlock();

	}

	//R0LevelLock
	m_MutexCapThrdExitFlag.lock(); {
		m_bEndPointCapCur = TRUE;
		m_nEndPointCapCursor = 0;
#ifdef __ScanInitModule_ConsoleDbgPrint
		TRACE("%s ��̬��ͼ�߳��˳���%d\n", m_ptUnit->GetUnitName(), m_bEndPointCapCur);
#endif
		pRobotDriver->m_cLog->Write("%s ��̬��ͼ�߳��˳���%d", m_ptUnit->GetUnitName(), m_bEndPointCapCur);
	}m_MutexCapThrdExitFlag.unlock();
}

void CScanInitModule::DynamicEndPointProc(CRobotDriverAdaptor *pRobotDriver, BOOL bSaveImg)//������ID�����ID���˲����ͣ��Ƿ��ƿ�
{
	CString cstrDataSavePath;
	IplImage* pImage;
	CString cstrImgSavePath;
	T_SAVE_IMAGE tSaveImage;

	cstrDataSavePath.Format(OUTPUT_PATH+ m_ptUnit->GetUnitName()+WELDDATA_PATH + "\\%d\\", m_eEndPointType);
	//MakeSureDirectoryPathExists(cstrDataSavePath);
	CheckFolder(cstrDataSavePath);
	for (int nS = 0; nS < SAVE_IMAGE_THREAD_MUN; nS++)
	{
		tSaveImage.bSaveImageStaus[nS] = FALSE;
	}
	tSaveImage.nImageNo = 0;

	if (m_pTraceImgProcess == NULL) {
		XUI::MesBox::PopInfo("δ��ʼ�����Ƶ�Ԫ{0}�ĸ������ͼ�������!", m_ptUnit->GetUnitName());

		//R1LevelLock
		while (m_MutexFindEndPointFlag.trylock() == false) UsSleep(2); {
			m_bCameraFindWeldStart = TRUE;
		}m_MutexFindEndPointFlag.unlock();

		//R0LevelLock
		while (m_MutexProcThrdExitFlag.trylock() == false) UsSleep(2); {
			m_bEndPointProcess = TRUE;
		}m_MutexProcThrdExitFlag.unlock();
		return;
	}
	pImage = m_pTraceLockImg;

	E_START_OR_END eStartOrEnd =m_eEndPointType;
	double dCurPosX = 0;
	double dCurPosY = 0;
	double dCamerCoorX = 0;
	double dCamerCoorY = 0;
	double dCamerCoorZ = 0;
	double dOffsetStartPoint = 0;
	double dScanVel = 500;
	m_nTimeStart = 0;
	std::ofstream CoutEndPointInfoFileInWorld(cstrDataSavePath + "CoutEndPointInfoFileInWorld.txt");
	T_ABS_POS_IN_BASE tPointAbsCoordInBase;
//	T_CART_COOR dMachineCoorsProcess;//��¼������

	//R1LevelLock
	while (m_MutexFindEndPointFlag.trylock() == false) UsSleep(2); {
		m_bCameraFindWeldStart = FALSE;
	}m_MutexFindEndPointFlag.unlock();
	//R0LevelLock
	while (m_MutexProcThrdExitFlag.trylock() == false) UsSleep(2); {
		m_bEndPointProcess = FALSE;
	}m_MutexProcThrdExitFlag.unlock();
	
	CvPoint cpKeyPoint;

	//��ʱ���
	m_nStartPointShape = 4;
	m_bFixedPointScan = FALSE;
	//------------------------
	if (FALSE == m_bFixedPointScan){
		TranslateDrawingToImageParam(); // ����ͼֽ���ݳ�ʼ�� ͼ��Ҫ�õ������� �� ��Ҫ���⴦���õ�������
	}
	else{
		ResumeDrawingToImageParam();
	}
	int nSaveImgNo = 0;

	// �˳�ѭ����Ψһ��ʽ m_bCameraFindWeldStart == FALSE
	while (TRUE) {
		//R1LevelLock
		while (m_MutexFindEndPointFlag.trylock() == false) UsSleep(2); {
			if (FALSE == m_bCameraFindWeldStart) {
				if (m_ptUnit->RobotEmg(FALSE)) {
					m_MutexFindEndPointFlag.unlock();
					break;
				}
				//���ﲻ�ӻ��������ܳ��ַ���Ȩ���쳣��2024/01/03 ���ν��飩
				if (pRobotDriver->m_eThreadStatus == INCISEHEAD_THREAD_STATUS_STOPPED) {
					m_MutexFindEndPointFlag.unlock();
					break;
				}
				// �����ȡ������������Ч���Ż�(��ע�͵�)

				//R2LevelLock 
				while (m_MutexPicCapture.trylock() == false) UsSleep(2); {
					if (m_nEndPointCapCursor < 2) {
						m_MutexPicCapture.unlock();
						m_MutexFindEndPointFlag.unlock();
						continue;// �Ӳɼ���������ͼ��ʼ 
					}
					m_nEndPointProcCursor = m_nEndPointCapCursor - 1;
					if (NULL == m_ptStartPointInfo[m_nEndPointProcCursor % MAX_ARRAY_NUM_START].img) pRobotDriver->m_cLog->Write("kong!!!!!!!");
					long long tTime = XI_clock();
					m_bCameraFindWeldStart = m_pTraceImgProcess->IfPieceStartOrEndPoint(
						m_ptStartPointInfo[m_nEndPointProcCursor % MAX_ARRAY_NUM_START].img,
						dScanVel, dOffsetStartPoint, cpKeyPoint, eStartOrEnd,
						m_ePieceClass, m_iBaseLineLenInZone, m_iMoveLineLenInZone, m_iRoiWidSize, m_iIfEndConditionIdx
					);
					pRobotDriver->m_cLog->Write("����㴦��ʱ�䣺%d %d %d", XI_clock() - tTime, cpKeyPoint.x, cpKeyPoint.y);
					if (bSaveImg) {
						tSaveImage.nImageNo++;
						cstrImgSavePath.Format(OUTPUT_PATH+ m_ptUnit->GetUnitName() +SEARCH_SRC_PATH+"\\PointType_%d\\", m_eEndPointType);
						tSaveImage.cstrId .Format(cstrImgSavePath+"%d.jgp",tSaveImage.nImageNo);
						tSaveImage.pImage = m_ptStartPointInfo[m_nEndPointProcCursor % MAX_ARRAY_NUM_START].img;
						AfxBeginThread(ThreadSaveImage, &tSaveImage);
					}
					if (cpKeyPoint.x > 1 && cpKeyPoint.x < 2591 && cpKeyPoint.y>1 && cpKeyPoint.y < 1993) {
						cvCvtColor(m_ptStartPointInfo[m_nEndPointProcCursor % MAX_ARRAY_NUM_START].img, m_pShowImage, CV_GRAY2RGB);
						cvCircle(m_pShowImage, cpKeyPoint, 8, CV_RGB(255, 0, 0), 4);
					}
					//δɨ�賬���˵� ��¼�õ���ά����
					if (FALSE == m_bCameraFindWeldStart) {
						//������������ϵ�µĵ�
						m_ptUnit->TranImageToBase(
							m_ptUnit->m_nTrackCameraNo,
							cpKeyPoint,
							m_ptStartPointInfo[m_nEndPointProcCursor % MAX_ARRAY_NUM_START].tRobotCoors,
							m_ptStartPointInfo[m_nEndPointProcCursor % MAX_ARRAY_NUM_START].tRobotPulse,
							&tPointAbsCoordInBase
						);
						tPointAbsCoordInBase.tWeldLinePos.y += m_ptStartPointInfo[m_nEndPointProcCursor % MAX_ARRAY_NUM_START].dMachineCoors.dY;
						//��¼��������ϵ�µĵ�
						CoutEndPointInfoFileInWorld << m_nEndPointProcCursor << " " << tPointAbsCoordInBase.tWeldLinePos.x << " " << tPointAbsCoordInBase.tWeldLinePos.y << " " << tPointAbsCoordInBase.tWeldLinePos.z << std::endl;
					}
					//ɨ�賬���˵� ���ҵ�
					else {
						// ���ֶ˵�   
						pRobotDriver->m_cLog->Write("Come In11: %4d %4d", m_nEndPointProcCursor, m_bCameraFindWeldStart);
						pRobotDriver->m_cLog->Write("�����Ҳ���: m_iRoiWidSize = %d m_iBaseLineLenInZone = %d",
							m_iRoiWidSize, m_iBaseLineLenInZone);
						long long lBackFindTime = XI_clock();
						int nStartBackFineNo = 0;
						CString strImgPath;
						while (TRUE == m_bCameraFindWeldStart){
							//���ﲻ�ӻ��������ܳ��ַ���Ȩ���쳣��2024/01/03 ���ν��飩
							if (pRobotDriver->m_eThreadStatus == INCISEHEAD_THREAD_STATUS_STOPPED) break;
							m_nEndPointProcCursor = (m_nEndPointProcCursor - 1 + MAX_ARRAY_NUM_START) % MAX_ARRAY_NUM_START;
							m_bCameraFindWeldStart = m_pTraceImgProcess->IfPieceStartOrEndPoint(m_ptStartPointInfo[m_nEndPointProcCursor % MAX_ARRAY_NUM_START].img,
								dScanVel, dOffsetStartPoint, cpKeyPoint, eStartOrEnd,
								m_ePieceClass, m_iBaseLineLenInZone, m_iMoveLineLenInZone, m_iRoiWidSize, m_iIfEndConditionIdx);
							tSaveImage.nImageNo = nStartBackFineNo;
							strImgPath.Format(SEARCH_SRC_PATH + "\\%s\\EndPoint_ReverseScan_%d\\%d.jpg", m_ptUnit->GetUnitName(), m_eEndPointType, tSaveImage.nImageNo);
							tSaveImage.cstrId = strImgPath;
							if (bSaveImg) {
								tSaveImage.pImage = m_ptStartPointInfo[m_nEndPointProcCursor % MAX_ARRAY_NUM_START].img;
								AfxBeginThread(ThreadSaveImage, &tSaveImage);
							}
							nStartBackFineNo++;
							if (nStartBackFineNo > MAX_ARRAY_NUM_START && TRUE == m_bCameraFindWeldStart){
								pRobotDriver->m_eThreadStatus = INCISEHEAD_THREAD_STATUS_STOPPED;
								//pRobotDriver->m_cStepMoveControl.EmgStop(); //���⣺��ת��ΪCUnit���RobotDriverAdaptor�����
								//pRobotDriver->m_cStepMoveControl.StopStep(); //���⣺��ת��ΪCUnit���RobotDriverAdaptor�����
								pRobotDriver->HoldOn();
								pRobotDriver->ServoOff();
								m_bCameraFindWeldStart = TRUE;
								XUI::MesBox::PopError("����ͼƬ�ﵽ�����������ͣ��е��");
								break;
							}
						}
						m_nEndPointProcCursor = (m_nEndPointProcCursor + 1) % MAX_ARRAY_NUM_START;
						pRobotDriver->m_cLog->Write("Come Out11: %4d %4d ���� ������%d ��ʱ��%d",
							m_nEndPointProcCursor, m_bCameraFindWeldStart,
							nStartBackFineNo, XI_clock() - lBackFindTime);
						m_bCameraFindWeldStart = TRUE;
					}
				}m_MutexPicCapture.unlock();

				if (TRUE == m_bCameraFindWeldStart){
					m_MutexFindEndPointFlag.unlock();
					break;
				}
				Sleep(10);
				DoEvent();
			}
			else {
				m_MutexFindEndPointFlag.unlock();
				break;
			}
		}m_MutexFindEndPointFlag.unlock();
	}

	cstrImgSavePath.Format(OUTPUT_PATH+ m_ptUnit->GetUnitName()+SEARCH_SRC_PATH + "\\��ʵ�˵�ͼƬ.jpg");

	//R1LevelLock
	while (m_MutexPicCapture.trylock() == false) UsSleep(2); {
		SaveImage(m_ptStartPointInfo[m_nEndPointProcCursor % MAX_ARRAY_NUM_START].img, cstrImgSavePath);
		//���ݷ����ҳ���ȷ���
		m_pTraceImgProcess->IfPieceStartOrEndPoint(
			m_ptStartPointInfo[m_nEndPointProcCursor % MAX_ARRAY_NUM_START].img,
			dScanVel, dOffsetStartPoint, cpKeyPoint, eStartOrEnd,
			m_ePieceClass, m_iBaseLineLenInZone, m_iMoveLineLenInZone, m_iRoiWidSize, m_iIfEndConditionIdx);
		//��άת��ά
		m_ptUnit->TranImageToBase(
			m_ptUnit->m_nTrackCameraNo,
			cpKeyPoint,
			m_ptStartPointInfo[m_nEndPointProcCursor % MAX_ARRAY_NUM_START].tRobotCoors,
			m_ptStartPointInfo[m_nEndPointProcCursor % MAX_ARRAY_NUM_START].tRobotPulse,
			&tPointAbsCoordInBase
		);
		tPointAbsCoordInBase.tWeldLinePos.y += m_ptStartPointInfo[m_nEndPointProcCursor % MAX_ARRAY_NUM_START].dMachineCoors.dY;
	}m_MutexPicCapture.unlock();
	

	XI_POINT  tWeldLinepointInWorld = tPointAbsCoordInBase.tWeldLinePos;
	m_tWeldLinepointInWorld = tWeldLinepointInWorld;
	m_tDynamicScanEndPoint = m_tWeldLinepointInWorld;
	pRobotDriver->m_cLog->Write("���Ѷ˵㣺X:%lf Y:%lf Z:%lf", tWeldLinepointInWorld.x, tWeldLinepointInWorld.y, tWeldLinepointInWorld.z);
	FILE* pfStartPoint = fopen(cstrDataSavePath + "AA_StartUpBoardPoints.txt", "a+");
	fprintf(pfStartPoint, "%d%11.3lf%11.3lf%11.3lf\n", -2,
		tWeldLinepointInWorld.x, tWeldLinepointInWorld.y, tWeldLinepointInWorld.z);
	fclose(pfStartPoint);
	// ������̬ɨ�����ʱ �ж��Ƿ�ʹ���㷨����������Զ����Ϊ���
	CoutEndPointInfoFileInWorld.close();
	//������ʼɨ������ȥ����ʵ�����������ݻ������ϵ��
	CoordinatePointProcessFuncation(tWeldLinepointInWorld, (cstrDataSavePath + "CoutEndPointInfoFileInWorld.txt").GetBuffer(), (cstrDataSavePath + "CoutStartPointInfoFileInWorldCopy.txt").GetBuffer());
	pRobotDriver->HoldOn();
	pRobotDriver->m_cLog->Write("%s ��ʼ����ͼ�߳�", m_ptUnit->GetUnitName());
	while (TRUE){
		//���ﲻ�ӻ��������ܳ��ַ���Ȩ���쳣��2024/01/03 ���ν��飩
		if (pRobotDriver->m_eThreadStatus == INCISEHEAD_THREAD_STATUS_STOPPED) break;
		int nThreathNum = 0;
		for (int ns = 0; ns < SAVE_IMAGE_THREAD_MUN; ns++){
			if (FALSE == tSaveImage.bSaveImageStaus[ns]) nThreathNum++;
		}
		if (SAVE_IMAGE_THREAD_MUN == nThreathNum){
			pRobotDriver->m_cLog->Write("%s ��ͼ�߳̿���", m_ptUnit->GetUnitName());
			break;
		}
		DoEvent();
		Sleep(10);
	}

	while (m_MutexProcThrdExitFlag.trylock() == false) UsSleep(2); {
		m_bEndPointProcess = TRUE;
		pRobotDriver->m_cLog->Write("%s�˵㴦���߳��˳�:%d", m_ptUnit->GetUnitName(), m_bEndPointProcess);
	}m_MutexProcThrdExitFlag.unlock();

	while (m_MutexSuccessScanFlag.trylock() == false) UsSleep(2); {
		m_bSuccessScanFlag = TRUE;
	}m_MutexSuccessScanFlag.unlock();
	if (NULL != m_pTraceImgProcess)
	{
		delete m_pTraceImgProcess;
		m_pTraceImgProcess = NULL;
	}
	return;
}

/*
�������ܣ�ͼ����ַ��߳�	��������Ϊ����������ϵ�����꣩
*	pRobotDriver							�����˿�������
*	MaxThreadNum							�����������߳���		��Ĭ��3����
*	ScanSensitivity							���������ȣ�����ͼƬ������  ��Ĭ��5�ţ�
*	Threshold								���������Ĵ��������ֵΪThreshold	��Ĭ��0��
*	PicStrIndex								��ʼ����ĵ�һ��ͼƬ���
*	WaitOutTime								�ȴ�ͼƬ��ʱʱ��	��Ĭ��5000ms��
*	SaveImg									�Ƿ��ͼ	��Ĭ�ϲ��棩
*	SavePointCloud							�Ƿ�����	��Ĭ�ϲ��棩
*	PtType									�˵����ͣ��������ڱ����ļ������޸ģ��������ô���
*	FlipMode								��תģʽ��Ĭ�ϲ���ת��
*	ScanMode								ɨ��ģʽ��Ĭ��������ά�˵㣩
*	JobName									�Ѷ˵�ʹ�õ�Job���ƣ�Ĭ�ϡ�DYNAMICSCANNING����
*	JudgeRobotStopTime						��Ϊ������Ϊֹͣ״̬��ʱ����(��λ��ms��Ĭ��500ms)
*	NumOfSavePCThreadsInSingleProcThrd		���������̴߳����Ĵ�����߳�����(Ĭ��4���߳�)
(2023/11/23���� �������������� �޸�Ϊһ���Դ�������߳����� ������ռ��� �޸����ֱ���BUG �޸ĵ�����λ��)
(2024/01/11����	�Ż��ڴ����,��ͣ��������������Job������[Ĭ��Ϊ"DYNAMICSCANNING"])
(2024/02/26���� E_SCANMODE������������E_SCANMODE_FIXEDSCAN)
(2024/03/13���� �޸ĵ��Ʊ��淽ʽΪ���̱߳��棬����������ڱ������߳��ڣ��޸��������߳�ThreadAsynDynamic...��)
*/
void CScanInitModule::ImgProcDistribute(CRobotDriverAdaptor* pRobotDriver, unsigned int MaxThreadNum, const int& ScanSensitivity, const unsigned int& Threshold,
	const int& PicStrIndex, const int& WaitOutTime, const bool& SaveImg, const bool& SavePointCloud, E_START_OR_END PtType, const E_FLIP_MODE& FlipMode, const E_SCANMODE& ScanMode,
	const CString JobName, const unsigned int JudgeRobotStopTime, const unsigned int NumOfSavePCThreadsInSingleProcThrd)
{
	/*
	*		2024/01/04 �����걸ע��
	*		������ʹ��������д����������ʽ��ռ
	*		������ while(_mutex.trylock()==false) UsSleep(50);{...}_mutex.unlock();
	*		��ʾ���̼߳以��������{...}������if(true){...}�еġ�{...}��
	*		��ȷ��������λ�� �Լ��Ƿ���{...}�������ͷ���
	*		2024/01/11 �޸ģ������˳����������̶��ڱ�����ĩβ�������������⴦return������ͳһ�ڴ����
	*/

	/******                           ******/
	m_nStartPointShape = 4;             //�Ϸ�������ʱʹ�ã��ڴ�Ҳ��ʱʹ��
	TranslateDrawingToImageParam();     //���ڸ�ֵ��Ա����
	/****   ����Ϊ �Ϸ��� �Ĺ̶�����   ****/
	m_nScanSensitivity = ScanSensitivity > 0 ? ScanSensitivity : 1;
	MaxThreadNum = MaxThreadNum <= 0 ? 2 : MaxThreadNum;
	//��ʼ��ͼ��ʶ�����
	T_IMAGE_RECOGNIZE_PARAM tImgRecParam(m_iBaseLineLenInZone, m_iMoveLineLenInZone, m_iRoiWidSize,
		m_iIfEndConditionIdx, m_ePieceClass, m_bScanStartTwoLineOnStandBoard, m_dScanStartExtendLength);
	//��ʼ�����߳�
	AsynchrousProc asynProc = AsynchrousProc(2);
	//��ʼ����Ա����	�ɱ������ͷ��ڴ�
	m_pMapImgProcOutput = new map<int, T_THREAD_DYNAMICSCAN_OUT>;
	m_vtImgProcThreadStatus.resize(MaxThreadNum);
	m_vtImgProcThread.resize(MaxThreadNum);
	m_vtImgProcInput.resize(MaxThreadNum);
	for (unsigned int nCount = 0; nCount < MaxThreadNum; nCount++) {
		m_vtImgProcInput[nCount] = NULL;
		m_vtImgProcThread[nCount] = NULL;
		m_vtImgProcThreadStatus[nCount] = NULL;
	}

	//R0LevelLock
	m_MutexSuccessScanFlag.lock(); {
		m_bSuccessScanFlag = FALSE;
	}m_MutexSuccessScanFlag.unlock();
	pair<UINT, clock_t> prWaitProcessAccumTime_LastTime = { 0,0 };							//������_��¼�ȴ���һ���������������ۼ�ʱ�����ϴ��������������ɵ�ʱ���
	pair<UINT, clock_t> prWaitPictureAccumTime_LastTime = { 0,0 };							//������_��¼�ȴ�ͼƬ���ۼ�ʱ�����ϴλ�ȡ��ͼƬ��ʱ���
	pair<UINT, pair<UINT, clock_t>> prPosEqualNum_prAccumTime_LastTime = { 0,{0,0} };		//������_��¼�����˴�����ͬ����λ�õĴ������ۼ�ʱ�����ϴλ�ȡ������λ�õ�ʱ���
	long long tStartTime = 0;																//��¼�Ѷ˵���ʼʱ��
	long long tTmpTime = 0;																	//��ʱ����ʹ�õ���ʱ������ʱ��
	int nCurThreadIndex = 0;																//�߳����к�
	int nCurPicIndex = 0;																	//ͼƬ���к�
	int nMapHeadIndex = PicStrIndex;														//��ʼ�����ͼ��
	int nSuccessScanIndex = 0;																//�������ͼƬ��
	BOOL bExitWhileCirc = FALSE;															//�˳���whileѭ����־λ
	BOOL bIsCheck = TRUE;																	//�Ƿ���н������
	BOOL bHasCalling = FALSE;																//�Ƿ�Call��ɨ��Job�������Զ���Job��
	T_ROBOT_COORS preCoor;																	//��¼����������
	CImageProcess* pImgProcess;																//ͼ����ָ��
	vector<pair<int, T_START_POINT_INFO>>* pVtnImgProcInput;								//�洢ͼƬ�����̵߳�ͼƬ��Ϣ����
	T_THREAD_DYNAMICSCAN_PARAM* ptImgProcInput;												//�洢�����߳��������
//	XiLineParamNode tFrontLine, tBackLine;													//�Ӿ�б�������õ�������
	CDHGigeImageCapture* pDHCamDrv;															//����������
	CString cstrTrackDipParamFilePath;														//ͼ����ʹ�õĲ���·��
	CString cstrTemp;																		//���⴦��ʱʹ�õ��ַ���
	CWinThread* pProcessThread;																//�����߳�ָ��
	FILE* pFilePointCloud;																//���c���ļ�
	int nSavePointCloudCount = 0;															//������õ������
	SYSTEMTIME tSystemtime;
	GetSystemTime(&tSystemtime);
	if (SavePointCloud)
	{
		//���c�Ƶ�ݔ���ļ�Ŀ�
		cstrTemp.Format(OUTPUT_PATH + m_ptUnit->GetUnitName() + TRACEPROCRESULT_PATH + "%s_TRACE_PointCloud_%04d%02d%02d%02d%02d%02d.txt",
			m_ptUnit->GetUnitName(), tSystemtime.wYear, tSystemtime.wMonth, tSystemtime.wDay, tSystemtime.wHour, tSystemtime.wMinute, tSystemtime.wSecond);
		pFilePointCloud = fopen(cstrTemp, "w");
		m_sPointCloudFileName = cstrTemp;
	}
	tStartTime = XI_clock();
	//ȷ������·������
	CheckFolder(OUTPUT_PATH + m_ptUnit->GetUnitName() + WELDDATA_PATH);
	CheckFolder(OUTPUT_PATH + m_ptUnit->GetUnitName() + TRACEPROCRESULT_PATH);

	cstrTemp.Format(OUTPUT_PATH + m_ptUnit->GetUnitName() + WELDDATA_PATH + "CoutStartPointInfoFileInWorldCopy.txt");
	ofstream fileCurntLugNoCout(cstrTemp);													 //�����Ŀ¼
	cstrTrackDipParamFilePath = m_ptUnit->GetTrackTipDataFileName(m_ptUnit->m_nMeasureCameraNo);
	//��ȡ��Ӧ���Ƶ�Ԫ������
	WaitForSingleObject(m_hGetCameraCtrl, INFINITE);
	pDHCamDrv = (CDHGigeImageCapture*)m_ptUnit->GetCameraCtrl(m_ptUnit->m_nMeasureCameraNo);
	ReleaseSemaphore(m_hGetCameraCtrl, 1, NULL);
	//���ϲ�ͼ��,Ȼ����߳�б������		�������ĵ�while(TRUE)ѭ���ڷ�����
	//R2LevelLock
	m_MutexPicCapture.lock(); {
		//��ȡ����б���õ�ͼƬ
		IplImage* pScanLockImgSrc = NULL; pDHCamDrv->CaptureImage(pScanLockImgSrc, 1);
		IplImage* pScanLockImgDst = cvCreateImage(cvSize(pScanLockImgSrc->width, pScanLockImgSrc->height), pScanLockImgSrc->depth, pScanLockImgSrc->nChannels);
		CString cstrImgSavePath;
		cstrImgSavePath.Format(OUTPUT_PATH + m_ptUnit->GetUnitName() + SEARCH_SRC_PATH + "\\PointType_%d\\", PtType);
		//ȷ����ͼĿ¼����,���������Զ�����		WinApi::MakeSureDirectoryPathExists()
		CheckFolder(cstrImgSavePath);
		/*
		*		�ȴ����̣߳���Ϊÿ���߳�����б�ʡ�
		*		�������̵߳�new�ڴ����������ͷţ����⣺pBisExit�ɱ��߳��ͷ�
		*/
		for (unsigned int i = 0; i < MaxThreadNum; i++) {
			CString cstrTmp;
			CvPoint cpMidKeyPoint, cpBesideKeyPoint, LeftPtn, RightPtn, tKeyPoint;
			XiLineParamNode tFrontLine, tBackLine;
			vector<CvPoint> vtLeftPtns, vtRightPtns;
			//���߳�ͼ�����ָ�� ��ʼ��
			pImgProcess = new CImageProcess(
				m_ptUnit->GetCameraParam(m_ptUnit->m_nMeasureCameraNo).tDHCameraDriverPara.nRoiWidth,
				m_ptUnit->GetCameraParam(m_ptUnit->m_nMeasureCameraNo).tDHCameraDriverPara.nRoiHeight,
				0,
				cstrTrackDipParamFilePath
			);
			/*
			*		��ʼ��ͼ��ɨ����� �����Ѷ˵�һ��Ҫ��ʼ����������ͼ����ָ����ɲ�����ʼ��
			*		��������ͼ����ָ�� ���̻߳���ʳ�ͻ
			*/
			if (ScanMode == E_SCANMODE_2DPOINT ||
				ScanMode == E_SCANMODE_FIXEDSCAN) {
				pImgProcess->InitIfStartOrEndPntParam(E_PIECE_START);
				switch (FlipMode)
				{
				case E_FLIP_HORIZEN:
					cvFlip(pScanLockImgSrc, pScanLockImgDst, 0);
					break;
				case E_FLIP_VERTICAL:
					cvFlip(pScanLockImgSrc, pScanLockImgDst, 1);
					break;
				case E_FLIP_BOTH:
					cvFlip(pScanLockImgSrc, pScanLockImgDst, -1);
					break;
				default:
					cvCopyImage(pScanLockImgSrc, pScanLockImgDst);
					break;
				}
				//��ؼ������� �������� ��������б�� �������ImgProcess�ĳ�Ա����
				pImgProcess->GetBesideKeyPoints(pScanLockImgDst, cpMidKeyPoint, cpBesideKeyPoint, vtLeftPtns, vtRightPtns, false,
					500, 500,
					300, 300,
					20, 20);
				if (vtLeftPtns.size() <= 0 || vtRightPtns.size() <= 0) {
					//���ͼ�ͣ�ź�
					pRobotDriver->HoldOn();

					//R1LevelLock
					m_MutexFindEndPointFlag.lock(); {
						m_bCameraFindWeldStart = TRUE;
					}m_MutexFindEndPointFlag.unlock();

					//R0LevelLock
					m_MutexSuccessScanFlag.lock(); {
						m_bSuccessScanFlag = FALSE;
					}m_MutexSuccessScanFlag.unlock();
					//����ʧ�ܴ�ͼ
					cstrTemp = OUTPUT_PATH + m_ptUnit->GetUnitName() + TRACEPROCRESULT_PATH + "scan_lock_bad.jpg";
					int nParamsArray[3];            //��ͼ��ʽ
					nParamsArray[0] = (int)CV_IMWRITE_JPEG_QUALITY;
					nParamsArray[1] = (int)(0.3 * 100);
					nParamsArray[2] = 0;
					cvSaveImage((LPCSTR)cstrTemp, pScanLockImgDst, nParamsArray);
#ifdef __ScanInitModule_ConsoleDbgPrint
					TRACE("���̼߳���б������ʧ�ܣ��̺߳�:%d\n", i);
#endif
					pRobotDriver->m_cLog->Write("���̼߳���б������ʧ�ܣ��̺߳�:%d", i);
					XUI::MesBox::PopInfo("���̼߳���б������ʧ�ܣ��̺߳�:{0}", i);
					bExitWhileCirc = TRUE;
					delete pImgProcess;
					pImgProcess = NULL;
					break;
				}
				else {
					LeftPtn = vtLeftPtns[vtLeftPtns.size() / 2];
					RightPtn = vtRightPtns[vtRightPtns.size() / 2];
					tKeyPoint = pImgProcess->HandlockKeyPoint(cpMidKeyPoint, LeftPtn, RightPtn, tFrontLine, tBackLine);
					if (tKeyPoint.x >= pDHCamDrv->m_nImageWidth ||
						tKeyPoint.y >= pDHCamDrv->m_nImageHeight ||
						tKeyPoint.x == 0 || tKeyPoint.y == 0) 
					{
						//���ͼ�ͣ�ź�
						pRobotDriver->HoldOn();

						//R1LevelLock
						m_MutexFindEndPointFlag.lock(); {
							m_bCameraFindWeldStart = TRUE;
						}m_MutexFindEndPointFlag.unlock();

						//R0LevelLock
						m_MutexSuccessScanFlag.lock(); {
							m_bSuccessScanFlag = FALSE;
						}m_MutexSuccessScanFlag.unlock();
						//����ʧ�ܴ�ͼ
						cstrTemp = OUTPUT_PATH + m_ptUnit->GetUnitName() + TRACEPROCRESULT_PATH + "scan_lock_bad.jpg";
						int nParamsArray[3];            //��ͼ��ʽ
						nParamsArray[0] = (int)CV_IMWRITE_JPEG_QUALITY;
						nParamsArray[1] = (int)(0.3 * 100);
						nParamsArray[2] = 0;
						cvSaveImage((LPCSTR)cstrTemp, pScanLockImgDst, nParamsArray);
#ifdef __ScanInitModule_ConsoleDbgPrint
						TRACE("���̼߳���б������ʧ�ܣ��̺߳�:%d\n", i);
#endif
						pRobotDriver->m_cLog->Write("���̼߳���б������ʧ�ܣ��̺߳�:%d", i);
						XUI::MesBox::PopInfo("���̼߳���б������ʧ�ܣ��̺߳�:{0}", i);
						bExitWhileCirc = TRUE;
						delete pImgProcess;
						pImgProcess = NULL;
						break;
					}
				}
			}
			//���쵥�̵߳Ĵ������������
			Mutex* pMutexVtInput = new Mutex;
			//���߳�ͼ�������ָ�� ��ʼ��
			pVtnImgProcInput = new vector<pair<int, T_START_POINT_INFO>>;
			//���̴߳���������ļ�Ŀ¼
			cstrTmp.Format("%d���߳��Ѷ˵�ֱ�_%4d%2d%2d%2d%2d%2d.txt", i, tSystemtime.wYear, tSystemtime.wMonth, tSystemtime.wDay, tSystemtime.wHour, tSystemtime.wMinute, tSystemtime.wSecond);
			ofstream* pOutFile = new ofstream(OUTPUT_PATH + m_ptUnit->GetUnitName() + TRACEPROCRESULT_PATH + cstrTmp);
			ptImgProcInput = new T_THREAD_DYNAMICSCAN_PARAM(
				PtType, pRobotDriver, pVtnImgProcInput, m_pMapImgProcOutput, tImgRecParam, pOutFile, pFilePointCloud, &asynProc,
				&m_MutexMapProcOut, &m_MutexExitFlag, pMutexVtInput, &m_MutexSavePointCloud,/*Lock*/
				m_pShowImage, SaveImg, i, new BOOL(FALSE), SavePointCloud,
				pImgProcess, FlipMode, ScanMode, m_ptUnit->GetUnitName(),
				m_ptUnit->GetCameraParam(m_ptUnit->m_nMeasureCameraNo), *pRobotDriver->m_cXiRobotAbsCoorsTrans,
				cstrImgSavePath, &nSavePointCloudCount, NumOfSavePCThreadsInSingleProcThrd);
			//�߳�����״̬��false
			m_vtImgProcThreadStatus[i] = FALSE;
			//ͼ�����߳����������HOOK
			m_vtImgProcInput[i] = ptImgProcInput;
			//�����������߳� ������
			pProcessThread = AfxBeginThread(ThreadAsynDynamicScanEndPointProc, (LPVOID)ptImgProcInput, THREAD_PRIORITY_ABOVE_NORMAL, 0, CREATE_SUSPENDED);
			m_vtImgProcThread[i] = pProcessThread;
		}
		//�ͷ�����ͼƬ���ڴ�
		cvReleaseImage(&pScanLockImgSrc);
		cvReleaseImage(&pScanLockImgDst);
		//����ͼ�����Ϊ����ͼƬ��ʼ���
		m_nEndPointCapCursor = PicStrIndex;
	}

	/*
	*		���̵߳���ѭ�� ��Ȼ����������bExitWhileCirc == TRUE
	*		�뿪��ѭ��ʱ����֤��ͼ�̹߳ر�,����Ҫ��m_bCameraFindWeldStart = TRUE
	*		2024/01/11�޸ģ�ɾ������ѭ����return�����з�֧�����Է�return��ʽ�˳���ѭ��������ͳһ�ڴ����
	*/
	prWaitProcessAccumTime_LastTime.second = XI_clock();
	prWaitPictureAccumTime_LastTime.second = XI_clock();
	prPosEqualNum_prAccumTime_LastTime.second.second = XI_clock();
	//��¼��ǰ��̬
	preCoor = pRobotDriver->GetCurrentPos();
	while (TRUE)
	{
		//�ָ���ͼ ������		��while(TRUE)ѭ����Ψһ�ָ���λ�ã�
		m_MutexPicCapture.unlock();
		if (bExitWhileCirc) break;
		Sleep(10);
		prWaitPictureAccumTime_LastTime.first = XI_clock() - prWaitPictureAccumTime_LastTime.second;
		//��ʼ�������̺߳�
		nCurThreadIndex = 0;
		//�ȴ���ͼ��ʱ,�˳��߳�
		if (prWaitPictureAccumTime_LastTime.first > (UINT)WaitOutTime) {
			pRobotDriver->HoldOn();

			//R2LevelLock
			m_MutexFindEndPointFlag.lock(); {
				m_bCameraFindWeldStart = TRUE;
			}m_MutexFindEndPointFlag.unlock();

			//R0LevelLock
			m_MutexSuccessScanFlag.lock(); {
				m_bSuccessScanFlag = FALSE;
			}m_MutexSuccessScanFlag.unlock();
#ifdef __ScanInitModule_ConsoleDbgPrint
			TRACE("���߳��Ѷ˵㴦��ʧ�ܣ��ȴ�ͼƬ��ʱ��\n");
#endif
			pRobotDriver->m_cLog->Write("���߳��Ѷ˵㴦��ʧ�ܣ��ȴ�ͼƬ��ʱ��");
			XiMessageBoxOk("���߳��Ѷ˵㴦��ʧ�ܣ��ȴ�ͼƬ��ʱ��");

			//R2LevelLock
			m_MutexPicCapture.lock();
			bExitWhileCirc = TRUE;
			continue;
		}
		//�ȴ��������������ʱ,�˳��߳�
		if (prWaitProcessAccumTime_LastTime.first > (UINT)WaitOutTime) {
			pRobotDriver->HoldOn();

			//R2LevelLock
			m_MutexFindEndPointFlag.lock(); {
				m_bCameraFindWeldStart = TRUE;
			}m_MutexFindEndPointFlag.unlock();

			//R0LevelLock
			m_MutexSuccessScanFlag.lock(); {
				m_bSuccessScanFlag = FALSE;
			}m_MutexSuccessScanFlag.unlock();
#ifdef __ScanInitModule_ConsoleDbgPrint
			TRACE("���߳��Ѷ˵㴦��ʧ�ܣ��ȴ�������������ʱ��\n");
#endif
			pRobotDriver->m_cLog->Write("���߳��Ѷ˵㴦��ʧ�ܣ��ȴ�������������ʱ��");
			XiMessageBoxOk("���߳��Ѷ˵㴦��ʧ�ܣ��ȴ�������������ʱ��");

			//R2LevelLock
			m_MutexPicCapture.lock();
			bExitWhileCirc = TRUE;
			continue;
		}

		//��ͼ������			��while(TRUE)ѭ����֮��Ĵ�������������		 Ψһ����λ�ã�
		//R2LevelLock
		m_MutexPicCapture.lock();

		if (nCurPicIndex >= m_nEndPointCapCursor) continue;

		//���̴߳���ʱ���޸ĵ������ȼ� ��2024/03/25���ε��ȣ�
		{
			// 		if (m_vtImgProcInput.size() > 1) {
			// 		unsigned int nHeadMinNum = 0xFFFF, nMaxHeadNum = 0x0000;
			// 		unsigned int nHeadMinIndex = m_vtImgProcThread.size() - 1;
			// 		unsigned int nHeadMaxIndex = m_vtImgProcThread.size() - 1;
			// 		for (int i = 0; i < m_vtImgProcInput.size(); i++) {
			// 			//R2LevelLock
			// 			while (m_vtImgProcInput[i]->pMutexVtInput->lock(); {
			// 				if (m_vtImgProcInput[i]->pVtnImgProcInput->size() != 0) {
			// 					//������в�Ϊ��ʱ�ж�
			// 					if (m_vtImgProcInput[i]->pVtnImgProcInput->begin()->first < nHeadMinNum) {
			// 						//�ָ��߳�����(������л���ͼƬ��)
			// 						m_vtImgProcThreadStatus[i] = TRUE;
			// 						nHeadMinNum = m_vtImgProcInput[i]->pVtnImgProcInput->begin()->first;
			// 						nHeadMinIndex = i;
			// 						while (m_vtImgProcThread[i]->ResumeThread() > 0) continue;
			// 					}
			// 					if (m_vtImgProcInput[i]->pVtnImgProcInput->begin()->first > nMaxHeadNum) {
			// 						nMaxHeadNum = m_vtImgProcInput[i]->pVtnImgProcInput->begin()->first;
			// 						nHeadMaxIndex = i;
			// 					}
			// 				}
			// 				else {
			// 					//�������Ϊ��ʱ�������ȼ�
			// 					m_vtImgProcThread[i]->SetThreadPriority(THREAD_PRIORITY_NORMAL);
			// 				}
			// 			}m_vtImgProcInput[i]->pMutexVtInput->unlock();
			// 		}
			// 		//���ȼ�����������ˮƽ
			// 		if (m_vtImgProcThread[nHeadMaxIndex]->GetThreadPriority() > THREAD_PRIORITY_NORMAL) {
			// #ifdef __ScanInitModule_ConsoleDbgPrint
			// 			TRACE("�߳�%d����Ϊ���ȼ�%d\n", nHeadMaxIndex, m_vtImgProcThread[nHeadMaxIndex]->GetThreadPriority() - 1);
			// #endif
			// 			m_vtImgProcThread[nHeadMaxIndex]->SetThreadPriority(m_vtImgProcThread[nHeadMaxIndex]->GetThreadPriority() - 1);
			// 		}
			// 		else {
			// #ifdef __ScanInitModule_ConsoleDbgPrint
			// 			TRACE("�߳�%d����Ϊ���ȼ�%d\n", nHeadMaxIndex, THREAD_PRIORITY_NORMAL);
			// #endif
			// 			m_vtImgProcThread[nHeadMaxIndex]->SetThreadPriority(THREAD_PRIORITY_NORMAL);
			// 		}
			// 		//���ܳ��������߳�||��ͼ�̵߳����ȼ�
			// 		if (m_vtImgProcThread[nHeadMinIndex]->GetThreadPriority() < THREAD_PRIORITY_ABOVE_NORMAL) {
			// #ifdef __ScanInitModule_ConsoleDbgPrint
			// 			TRACE("�߳�%d����Ϊ���ȼ�%d\n", nHeadMinIndex, m_vtImgProcThread[nHeadMinIndex]->GetThreadPriority() + 1);
			// #endif
			// 			m_vtImgProcThread[nHeadMinIndex]->SetThreadPriority(m_vtImgProcThread[nHeadMinIndex]->GetThreadPriority() + 1);
			// 		}
			// 		else {
			// #ifdef __ScanInitModule_ConsoleDbgPrint
			// 			TRACE("�߳�%d����Ϊ���ȼ�%d\n", nHeadMinIndex, THREAD_PRIORITY_NORMAL);
			// #endif
			// 			m_vtImgProcThread[nHeadMinIndex]->SetThreadPriority(THREAD_PRIORITY_NORMAL);
			// 		}
			// 		}
		}

		//�ȴ���ͼƬ����
		if (nCurPicIndex < m_nEndPointCapCursor && m_nEndPointCapCursor > PicStrIndex) {
			//��ʱ��������
			prWaitPictureAccumTime_LastTime.first = 0;
			prWaitPictureAccumTime_LastTime.second = XI_clock();

			//2024/03/25 ���ε���
			{
				//				//�ж��Ƿ��д����̵߳Ĵ������Ϊ�գ������������б�־λΪfalse,�������߳�
				//				//��ͼƬ������߳�������
				//				for (int i = 0; i < m_vtImgProcThreadStatus.size(); i++) {
				//					//R2LevelLock
				//					while (m_vtImgProcInput[i]->pMutexVtInput->lock(); {
				//						if (m_vtImgProcInput[i]->pVtnImgProcInput->size() == 0) {
				//#ifdef __ScanInitModule_ConsoleDbgPrint
				//							TRACE("�����߳�%d����ͼƬ������ʱ����\n", i);
				//#endif
				//							pRobotDriver->m_cLog->Write("�����߳�%d����ͼƬ������ʱ����", i);
				//							m_vtImgProcThreadStatus[i] = FALSE;
				//							m_vtImgProcThread[i]->SuspendThread();
				//						}
				//					}m_vtImgProcInput[i]->pMutexVtInput->unlock();
				//				}
				//
				//				//Ѱ�ҿ��д����߳�
				//				for (int i = 0; i < m_vtImgProcThreadStatus.size(); i++) {
				//					nCurThreadIndex = i;
				//					if (m_vtImgProcThreadStatus[i] == FALSE)  break;
				//				}
				//
				//				//�޿��д����߳�
				//				if (m_vtImgProcThreadStatus[nCurThreadIndex] == TRUE) {
				//					//2023.12���Ӿ��� Intel i7-6700���˴���һ��2448*2048�Ҷ�ͼƬ��ʱԼΪ40ms
				//					unsigned int nMinProcNum = 65535, nTmpCount = 0, nRecordIndex = 0;
				//					//�ҵ���������߳�����Ҫ����ͼƬ�������ٵ��߳�
				//					for (auto itera = m_vtImgProcInput.begin(); itera != m_vtImgProcInput.end(); itera++) {
				//						//R2LevelLock
				//						while ((*itera)->pMutexVtInput->lock(); {
				//							if ((*itera)->pVtnImgProcInput->size() < nMinProcNum) {
				//								nMinProcNum = (*itera)->pVtnImgProcInput->size();
				//								nRecordIndex = nTmpCount;
				//							}
				//						}(*itera)->pMutexVtInput->unlock();
				//						nTmpCount++;
				//					}
				//					T_START_POINT_INFO tInfo;
				//					while (tInfo.tRobotPulse.nSPulse == 0 && tInfo.tRobotPulse.nLPulse == 0 && tInfo.tRobotPulse.nUPulse == 0 &&
				//						tInfo.tRobotPulse.nRPulse == 0 && tInfo.tRobotPulse.nBPulse == 0 && tInfo.tRobotPulse.nTPulse == 0 || tInfo.img == NULL) {
				//						tInfo = m_ptStartPointInfo[nCurPicIndex % MAX_ARRAY_NUM_START];
				//					}
				//					if (tInfo.img->nChannels < 0 || tInfo.img->nChannels > 3) {
				//#ifdef __ScanInitModule_ConsoleDbgPrint
				//						TRACE("ͼ����ַ��̼߳�⵽������ͼƬ��Ϣ nCurId:%d\n", nCurPicIndex);
				//#endif
				//						pRobotDriver->m_cLog->Write("ͼ����ַ��̼߳�⵽������ͼƬ��Ϣ nCurId:%d\n", nCurPicIndex);
				//
				//					}
				//					tInfo.img = cvCloneImage(tInfo.img);
				//
				//					//R2LevelLock
				//					while (m_vtImgProcInput[nRecordIndex]->pMutexVtInput->lock(); {
				//						m_vtImgProcInput[nRecordIndex]->pVtnImgProcInput->push_back(make_pair(nCurPicIndex, tInfo));
				//					}m_vtImgProcInput[nRecordIndex]->pMutexVtInput->unlock();
				//				}
				//				//�п��д����߳�
				//				else {
				//					//���̱߳�־λΪæ
				//#ifdef __ScanInitModule_ConsoleDbgPrint
				//					TRACE("���д����߳�%d������\n", nCurThreadIndex);
				//#endif
				//					pRobotDriver->m_cLog->Write("���д����߳�%d������", nCurThreadIndex);
				//					m_vtImgProcThreadStatus[nCurThreadIndex] = true;
				//					T_START_POINT_INFO tInfo;
				//					while (tInfo.tRobotPulse.nSPulse == 0 && tInfo.tRobotPulse.nLPulse == 0 && tInfo.tRobotPulse.nUPulse == 0 &&
				//						tInfo.tRobotPulse.nRPulse == 0 && tInfo.tRobotPulse.nBPulse == 0 && tInfo.tRobotPulse.nTPulse == 0 || tInfo.img == NULL) {
				//						tInfo = m_ptStartPointInfo[nCurPicIndex % MAX_ARRAY_NUM_START];
				//					}
				//					if (tInfo.img->nChannels < 0 || tInfo.img->nChannels > 3) {
				//#ifdef __ScanInitModule_ConsoleDbgPrint
				//						TRACE("ͼ����ַ��̼߳�⵽������ͼƬ��Ϣ nCurId:%d\n", nCurPicIndex);
				//#endif
				//						pRobotDriver->m_cLog->Write("ͼ����ַ��̼߳�⵽������ͼƬ��Ϣ nCurId:%d\n", nCurPicIndex);
				//					}
				//					tInfo.img = cvCloneImage(tInfo.img);
				//
				//					//���봦����Ϣ
				//					//R2LevelLock
				//					while (m_vtImgProcInput[nCurThreadIndex]->pMutexVtInput->lock(); {
				//						m_vtImgProcInput[nCurThreadIndex]->pVtnImgProcInput->push_back(make_pair(nCurPicIndex, tInfo));
				//					}m_vtImgProcInput[nCurThreadIndex]->pMutexVtInput->unlock();
				//					//�ָ������߳�
				//					while (m_vtImgProcThread[nCurThreadIndex]->ResumeThread() > 0);
				//
				//				}
			}

			//2024/03/25 �޸ĵ�������Ϊ�����ȡ����벻ͬ�߳���
			{
				int nProcThreadNo = nCurPicIndex % m_vtImgProcThread.size();	//�����ͼƬ���߳����
				T_START_POINT_INFO tInfo;
				while (tInfo.tRobotPulse.nSPulse == 0 && tInfo.tRobotPulse.nLPulse == 0 &&
					tInfo.tRobotPulse.nUPulse == 0 && tInfo.tRobotPulse.nRPulse == 0 &&
					tInfo.tRobotPulse.nBPulse == 0 && tInfo.tRobotPulse.nTPulse == 0 || tInfo.img == NULL)
				{
					tInfo = m_ptStartPointInfo[nCurPicIndex % MAX_ARRAY_NUM_START];
				}
				//�쳣�ж�
				if (tInfo.img->nChannels < 0 || tInfo.img->nChannels > 3) {
#ifdef __ScanInitModule_ConsoleDbgPrint
					TRACE("ͼ����ַ��̼߳�⵽������ͼƬ��Ϣ nCurId:%d\n", nCurPicIndex);
#endif
					pRobotDriver->m_cLog->Write("ͼ����ַ��̼߳�⵽������ͼƬ��Ϣ nCurId:%d\n", nCurPicIndex);

				}
				tInfo.img = cvCloneImage(tInfo.img);

				//R2LevelLock
				m_vtImgProcInput[nProcThreadNo]->pMutexVtInput->lock(); {
					//���������̴߳�������
					m_vtImgProcInput[nProcThreadNo]->pVtnImgProcInput->push_back(make_pair(nCurPicIndex, tInfo));
					//�ָ��߳�ִ��
					while (m_vtImgProcThread[nProcThreadNo]->ResumeThread() > 0);
				}m_vtImgProcInput[nProcThreadNo]->pMutexVtInput->unlock();
			}


			//�ж��˳�switchʱ�Ƿ���Ҫִ��continue������while(TRUE)ѭ���Ķ���
			BOOL bContinue(FALSE);
			//�жϴ���ģʽ
			switch (ScanMode) {
			case E_SCANMODE_2DPOINT:	//�Ѷ˵㣬ִ���Ƿ�����������ж�

				//���˴�Ϊ�㷨���֣����ݽṹ�����Ա����ܣ����ң�	���迴��ʵ�֣���ʵ�ֵĹ���Ϊ���ҵ���N��ͼƬ����N��ͼƬ������������:
				/*
					1.����ͼƬ֮ǰ��ScanSensitivity��ͼƬ�������ģ���֤�����[n+1,n+2,n+3,n+4...,n+ScanSensitivity]����ScanSensitivity������ImgProcDis�����Ĳ�������Ϊ���������ȣ�
						������[n,n+1]�������Ľ����[n+1,n+2]�������Ľ����[n+1,n+3]���������Ľ��...
						1.1���Thresholdֵ�����㣬�����ͼƬǰ��ScanSensitivity��ͼƬ���Բ��ϸ�������ֻҪ�����ŵĲ�ֵ��Threshold+1��Χ�ڶ����������	��Threshold������ImgProcDis�����Ĳ�������Ϊ�ݲ

					2.����ͼƬ�������ScanSensitivity��ͼƬ�� �Ҳ�����άͼ��ļ��⽻��
						�Ҳ�����άͼ��ļ��⽻�㣺�������̵߳��Ӿ��� GSImageProcess �еĺ����ӿ� IfPieceStartOrEndPoint() ���� ScanSensitivity �� true
						���������Ķ���Ҳ����1.1�е�����
				*/
				//R2LevelLock
				m_MutexMapProcOut.lock(); {
					//����ӳ��size����������ʱ��ʼ�ж��Ƿ��ѵ��˵�
					if (m_pMapImgProcOutput->size() > m_nScanSensitivity) {
						bIsCheck = FALSE;
						//�ж��Ƿ���������������û�����ж�,�����ȴ�������ͼƬ�������
						int nTmpMapHeadIndex = nMapHeadIndex;
						for (int i = 0; i < m_nScanSensitivity; i++) {
							if (m_pMapImgProcOutput->count(nTmpMapHeadIndex + 1) == 0) {
								if (Threshold > 0) {
									// �����ݲ��
									for (unsigned int j = 1; j <= Threshold; j++) {
										if (m_pMapImgProcOutput->count(nTmpMapHeadIndex + 1 + j) > 0) {
											// ͨ���ݲ��
											nTmpMapHeadIndex = nTmpMapHeadIndex + 1 + j;
											bIsCheck = FALSE;
											break;
										}
										//δ��ͨ���ݲ��
										if (j == Threshold)	bIsCheck = TRUE;
									}
								}
								else {
									bIsCheck = FALSE;
									break;
								}
							}
							else {
								nTmpMapHeadIndex++;
							}
							// δͨ�����
							if (bIsCheck == TRUE) {
								bIsCheck = FALSE;
								break;
							}
							//ͨ�����
							if (i + 1 == m_nScanSensitivity) bIsCheck = TRUE;
						}

						//������������
						if (bIsCheck) {
							prWaitProcessAccumTime_LastTime.first = 0;
#ifdef __ScanInitModule_ConsoleDbgPrint
							TRACE("��⵽����������");
#endif
							prWaitProcessAccumTime_LastTime.second = XI_clock();
							/*
							*		�����һ�����������:aPositionItera
							*		��m_nScanSensitivity+Threshold��Χ�ڶ�λ�ĵ�����:aTmpItera
							*/
							map<int, T_THREAD_DYNAMICSCAN_OUT>::iterator aPositionItera, aTmpItera;
							BOOL bWaitFirstResult = TRUE;		//�ȴ���һ��������
							//�ҵ���һ��������
							for (int i = nMapHeadIndex; i < nMapHeadIndex + m_nScanSensitivity + (int)Threshold; i++) {
								if (m_pMapImgProcOutput->count(i) != 0) {
									aPositionItera = m_pMapImgProcOutput->find(i);
									aTmpItera = m_pMapImgProcOutput->find(i);
									break;
								}
								if (i + 1 == nMapHeadIndex + m_nScanSensitivity + Threshold) bWaitFirstResult = FALSE;
							}
							if (bWaitFirstResult == FALSE) {
								//�ָ���ͼƬ������� ������
								m_MutexMapProcOut.unlock();
								//����switchѭ����ִ��continue��� ������while(TRUE)ѭ���Ķ���
								bContinue = TRUE;
								break;
							}
							//������aTmpItera��λ����һ�� �ҽ���Ϊfalse ��λ��,
							for (int i = 0; i < m_nScanSensitivity + (int)Threshold; i++) {
								if (m_pMapImgProcOutput->count(nMapHeadIndex + i) == 0) continue;
								aTmpItera = m_pMapImgProcOutput->find(nMapHeadIndex + i);
								if ((*aTmpItera).second.bFindIntersectPoint == FALSE) break;
							}
							if (aTmpItera->first == aPositionItera->first && aTmpItera->second.bFindIntersectPoint == FALSE) {
								//�ж��Ƿ�������ScanSensitivity��ͼƬ�Ҳ�������,�еĻ���ʾͼƬ������ɣ������õ���������һ���ҵõ������λ��
								for (int i = 0; i < m_nScanSensitivity; i++) {
									if (FALSE == aTmpItera->second.bFindIntersectPoint) {
										aTmpItera++;
									}
									else {
										break;
									}
									if (i == m_nScanSensitivity - 1) aTmpItera--; //����һ��λ��
								}
								//����aTmpIteraҪôָ���ҵ������λ��Ҫôָ��÷�Χ[now,now+m_nScanSensitivity-1]�����һ��λ��
								//�ƶ�m_nScanSensitivity - 1 �� �鿴�±��Ƿ��aTmpIteraһ��
								//aPositionItera[a]-[a+m_nScanSensitivity - 1] ��m_nScanSensitivity��Ԫ��
								for (int i = 0; i < m_nScanSensitivity - 1; i++, aPositionItera++);
								if (aPositionItera->first == aTmpItera->first) {
									//����ͼƬ����,�ҵ��˵�
									pRobotDriver->HoldOn();
									while (aTmpItera->second.bFindIntersectPoint == FALSE) aTmpItera--;
									//�����Ϊ�ҵ��Ķ˵��
									nSuccessScanIndex = aTmpItera->first;

									//R1LevelLock
									m_MutexFindEndPointFlag.lock(); {
										m_bCameraFindWeldStart = TRUE;
									}m_MutexFindEndPointFlag.unlock();

									//R0LevelLock
									m_MutexSuccessScanFlag.lock(); {
										m_bSuccessScanFlag = TRUE;
									}m_MutexSuccessScanFlag.unlock();
									//����switchѭ����ִ��continue��� ������while(TRUE)ѭ���Ķ���
									m_MutexMapProcOut.unlock();
									bContinue = TRUE;
									bExitWhileCirc = TRUE;
									break;
								}
								else {
									if (aTmpItera != m_pMapImgProcOutput->end()) {
										nMapHeadIndex = (++aTmpItera)->first;
									}
									else {
										nMapHeadIndex = aTmpItera->first;
									}
								}
							}
							else {
								if (aTmpItera->first != m_pMapImgProcOutput->rbegin()->first) {
									nMapHeadIndex = (++aTmpItera)->first;
								}
								else {
									nMapHeadIndex = aTmpItera->first;
								}
							}
						}
						else {
							prWaitProcessAccumTime_LastTime.first = XI_clock() - prWaitProcessAccumTime_LastTime.second;
						}
					}
				}m_MutexMapProcOut.unlock();
				break;
			case E_SCANMODE_KINEMATIC:	//�˶�ģʽ����ִ���Ƿ�����������ж�
				break;
			case E_SCANMODE_FIXEDSCAN:	//����ģʽ����ִ���Ƿ�����������ж�
				break;
			default:
				//R1LevelLock
				m_MutexFindEndPointFlag.lock(); {
					m_bCameraFindWeldStart = TRUE;
				}m_MutexFindEndPointFlag.unlock();

				//R0LevelLock
				m_MutexSuccessScanFlag.lock(); {
					m_bSuccessScanFlag = FALSE;
				}m_MutexSuccessScanFlag.unlock();
#ifdef __ScanInitModule_ConsoleDbgPrint
				TRACE("ImgProcDis��������ָ���˴��������ģʽ��\n");
#endif
				pRobotDriver->m_cLog->Write("ImgProcDis��������ָ���˴��������ģʽ��");
				XiMessageBoxOk("ImgProcDis��������ָ���˴��������ģʽ��");
				bExitWhileCirc = TRUE;
				bContinue = TRUE;
				break;
			}
			if (bContinue) continue;

			if (bHasCalling == FALSE) {
				nMapHeadIndex = nCurPicIndex;
				m_pRobotDriver->CallJob(JobName);
				bHasCalling = TRUE;
			}
			//ͼ���1
			nCurPicIndex++;
		}

		//�ж��Ƿ񵽴������յ�	ͬһ����̬���ֳ�ʱʱ��(��)�����򵽴��յ�
		if (preCoor == pRobotDriver->GetCurrentPos()) {
			prPosEqualNum_prAccumTime_LastTime.first++;
			prPosEqualNum_prAccumTime_LastTime.second.first = XI_clock() - prPosEqualNum_prAccumTime_LastTime.second.second;
#ifdef __ScanInitModule_ConsoleDbgPrint
			TRACE("��%d count:%d x:%.3f y:%.3f z:%.3f\n",
				prPosEqualNum_prAccumTime_LastTime.second.first,
				prPosEqualNum_prAccumTime_LastTime.first,
				preCoor.dX,
				preCoor.dY,
				preCoor.dZ);
#endif
		}
		else {
			preCoor = pRobotDriver->GetCurrentPos();
			prPosEqualNum_prAccumTime_LastTime.first = 0;
			prPosEqualNum_prAccumTime_LastTime.second.first = 0;
			prPosEqualNum_prAccumTime_LastTime.second.second = XI_clock();
#ifdef __ScanInitModule_ConsoleDbgPrint
			TRACE("����\n");
#endif
		}
		//������λ��ͬһλ��ʱ��������JudgeRobotStopTime����ʼ�ж��Ƿ��������
		if (prPosEqualNum_prAccumTime_LastTime.second.first >= JudgeRobotStopTime && prPosEqualNum_prAccumTime_LastTime.first > JudgeRobotStopTime / 100) {
			pRobotDriver->HoldOn();
			pRobotDriver->m_cLog->Write("��ֹ��ʱ����ʼ�ж��Ƿ��������\n");

			//R1LevelLock
			//��ֹͣ��ͼ
			m_MutexFindEndPointFlag.lock(); {
				m_bCameraFindWeldStart = TRUE;
			}m_MutexFindEndPointFlag.unlock();

			//�Ѷ˵�ģʽ,��ѯɨ����
			if (ScanMode == E_SCANMODE_2DPOINT) {
				//R2LevelLock
				m_MutexMapProcOut.lock(); {
					if (m_pMapImgProcOutput->size() > 0) {
						nSuccessScanIndex = (*m_pMapImgProcOutput->rbegin()).first;		//���������д�������Ч�ģ�����ע��
					}
					else {
						//R0LevelLock
						m_MutexSuccessScanFlag.lock(); {
							m_bSuccessScanFlag = FALSE;
						}m_MutexSuccessScanFlag.unlock();
#ifdef __ScanInitModule_ConsoleDbgPrint
						TRACE("�߳�ɨ���ͷʧ�ܣ���������Ϊ��\n");
#endif
						pRobotDriver->m_cLog->Write("���߳�ɨ���ͷʧ�ܣ���������Ϊ��");
						XiMessageBoxOk("���߳�ɨ���ͷʧ�ܣ���������Ϊ��");
						bExitWhileCirc = TRUE;
					}
				}m_MutexMapProcOut.unlock();
			}
			if (bExitWhileCirc) continue;

			//�ȴ������̴߳�����ɣ��������һ���˵�����
			for (int i = 0; i < m_vtImgProcThreadStatus.size(); i++) {
				tTmpTime = XI_clock();
				m_vtImgProcInput[i]->pMutexVtInput->lock(); {
					while (m_vtImgProcInput[i]->pVtnImgProcInput->size() > 0) {
						m_vtImgProcInput[i]->pMutexVtInput->unlock();
						while (m_vtImgProcThread[i]->ResumeThread() > 0) m_vtImgProcThread[i]->ResumeThread();	//��֤�̴߳�������״̬
						if (XI_clock() - tTmpTime >= WaitOutTime) {
							//R0LevelLock
							m_MutexSuccessScanFlag.lock(); {
								m_bSuccessScanFlag = FALSE;
							}m_MutexSuccessScanFlag.unlock();
#ifdef __ScanInitModule_ConsoleDbgPrint
							TRACE("����������β�㣬�߳�%d�ȴ�ͼƬ����ʱ�䳬��%d��,����ʧ��\n", i, WaitOutTime / 1000);
#endif
							pRobotDriver->m_cLog->Write("����������β�㣬�߳�%d�ȴ�ͼƬ����ʱ�䳬��%d��,����ʧ��", i, WaitOutTime / 1000);
							XUI::MesBox::PopInfo("����������β�㣬�߳�{0}�ȴ�ͼƬ����ʱ�䳬��{1}��,����ʧ��", i, WaitOutTime / 1000);
							bExitWhileCirc = TRUE;
						}
						Sleep(WaitOutTime / 100);
						m_vtImgProcInput[i]->pMutexVtInput->lock();
						if (bExitWhileCirc) break;
					}
				}m_vtImgProcInput[i]->pMutexVtInput->unlock();
				tTmpTime = 0;
				if (bExitWhileCirc) break;
			}
			if (bExitWhileCirc) continue;
			switch (ScanMode)
			{
			case E_SCANMODE_2DPOINT:
				cstrTemp.Format("�Ѷ˵�");
				break;
			case E_SCANMODE_KINEMATIC:
				cstrTemp.Format("�˶�");
				break;
			case E_SCANMODE_FIXEDSCAN:
				cstrTemp.Format("��������");
				break;
			default:
				break;
			}
#ifdef __ScanInitModule_ConsoleDbgPrint
			TRACE("%sģʽ������������β��\n", cstrTemp);
#endif
			pRobotDriver->m_cLog->Write("%sģʽ������������β��", cstrTemp);
			//ɨ���ά��ģʽ||��������ģʽ ������Ӧ��ά����������,
			if (ScanMode == E_SCANMODE_2DPOINT ||
				ScanMode == E_SCANMODE_FIXEDSCAN
				) {
				//��ʱ���д����߳��ѽ���
				//R2LevelLock
				m_MutexMapProcOut.lock(); {
					//�Ӻ���ǰ��
					for (auto itera = m_pMapImgProcOutput->rbegin(); itera != m_pMapImgProcOutput->rend(); itera++) {
						if (itera->second.bFindIntersectPoint == TRUE) {
							//ͼ����Ϊ�ҵ���ͼƬ
							nSuccessScanIndex = itera->first;
							break;
						}
						if (itera == m_pMapImgProcOutput->rend()) {
							//R0LevelLock
							m_MutexSuccessScanFlag.lock(); {
								m_bSuccessScanFlag = FALSE;
							}m_MutexSuccessScanFlag.unlock();
#ifdef __ScanInitModule_ConsoleDbgPrint
							TRACE("δ�ҵ��κι���������Ϣ����������\n");
#endif
							pRobotDriver->m_cLog->Write("δ�ҵ��κι���������Ϣ����������");
							XiMessageBoxOk("δ�ҵ��κι���������Ϣ����������");
							bExitWhileCirc = TRUE;
						}
					}
				}m_MutexMapProcOut.unlock();
			}
			if (bExitWhileCirc) continue;
			m_MutexSuccessScanFlag.lock(); {
				m_bSuccessScanFlag = TRUE;
			}m_MutexSuccessScanFlag.unlock();
			bExitWhileCirc = TRUE;
		}
	}

	if (nSuccessScanIndex < PicStrIndex) nSuccessScanIndex = PicStrIndex;
	//�������й�����߳�
	for (auto itera = m_vtImgProcThread.begin(); itera != m_vtImgProcThread.end(); itera++) {
		if (*itera != NULL) {
			while ((*itera)->ResumeThread() > 0) (*itera)->ResumeThread();
		}
	}
	//�������д����߳�
	for (auto itera = m_vtImgProcInput.begin(); itera != m_vtImgProcInput.end(); itera++) {
		//�����˳���־λ ������
	//R2LevelLock
		if ((*itera) == NULL) break;
		m_MutexExitFlag.lock(); {
			*(*itera)->pBisExit = TRUE;
		}m_MutexExitFlag.unlock();
		Sleep(10);

		//R2LevelLock
		m_MutexExitFlag.lock(); {
			while (*(*itera)->pBisExit == TRUE) {
				//ѭ���ȴ��߳��ͷ�
				m_MutexExitFlag.unlock();
				Sleep(10);
				m_MutexExitFlag.lock();
			}
		}m_MutexExitFlag.unlock();
	}
	//�Ѷ˵�ģʽ-(��������㣬�����켣  ���ļ�)  ��������ļ�
	m_vtImgProc3DPoint.clear();
	// ��ȡ�Ѷ˵�ʱ�ⲿ������
	double dExPos = m_ptUnit->GetExPositionDis(m_ptUnit->m_nMeasureAxisNo);
	//R0LevelLock
	m_MutexSuccessScanFlag.lock(); {
		if (m_bSuccessScanFlag) {
			//����˵㣬�켣���ļ�
			if (ScanMode == E_SCANMODE_2DPOINT ||
				ScanMode == E_SCANMODE_FIXEDSCAN) {
				/* ��������� */
				m_tWeldLinepointInWorld = m_pMapImgProcOutput->find(nSuccessScanIndex)->second.tPointAbsCoordInBase.tWeldLinePos;
				m_tDynamicScanEndPoint = m_tWeldLinepointInWorld;           //ԭ�����̵ĸ�ֵ��ʽ

#ifdef __ScanInitModule_ConsoleDbgPrint
				TRACE("���Ѷ˵㣺X:%lf Y:%lf Z:%lf �ܺ�ʱ��\n", m_tWeldLinepointInWorld.x, m_tWeldLinepointInWorld.y, m_tWeldLinepointInWorld.z, XI_clock() - tStartTime);
#endif
				pRobotDriver->m_cLog->Write("���Ѷ˵㣺X:%lf Y:%lf Z:%lf �ܺ�ʱ��", m_tWeldLinepointInWorld.x, m_tWeldLinepointInWorld.y, m_tWeldLinepointInWorld.z, XI_clock() - tStartTime);
				FILE* pfStartPoint = fopen(OUTPUT_PATH + m_ptUnit->GetUnitName() + WELDDATA_PATH + "AA_StartUpBoardPoints.txt", "a+");
				fprintf(pfStartPoint, "%d%11.3lf%11.3lf%11.3lf\n", -2, m_tWeldLinepointInWorld.x, m_tWeldLinepointInWorld.y, m_tWeldLinepointInWorld.z);
				fclose(pfStartPoint);
				auto preItera = m_pMapImgProcOutput->find(nSuccessScanIndex);
				XI_POINT tRecordStartScanPoint = preItera->second.tPointAbsCoordInBase.tWeldLinePos;
				for (auto itera = m_pMapImgProcOutput->find(nSuccessScanIndex); itera != m_pMapImgProcOutput->begin(); itera--) {
					if (preItera == itera) {
						fileCurntLugNoCout << nSuccessScanIndex - (*itera).first << " " << (*itera).second.tPointAbsCoordInBase.tWeldLinePos.x << " "
							<< (*itera).second.tPointAbsCoordInBase.tWeldLinePos.y + dExPos << " " << (*itera).second.tPointAbsCoordInBase.tWeldLinePos.z << endl;
					}
					else {
						if (TwoPointDis((*preItera).second.tPointAbsCoordInBase.tWeldLinePos.x, (*preItera).second.tPointAbsCoordInBase.tWeldLinePos.y,
							(*itera).second.tPointAbsCoordInBase.tWeldLinePos.x, (*itera).second.tPointAbsCoordInBase.tWeldLinePos.y) < 1.5) continue;
						// ��������Ҫ30mm����̬����Ҫһ�ΰ��ǣ�����128�����ۼӰ��ǳ��ȵ����ݣ�����ֻ�����۹�ϵ���ȵ���ʼ��110
						if ((m_dHandEyeDis + 20 + m_dScanStartExtendLength + fabs(m_dScanStartWrapLength)) < TwoPointDis(tRecordStartScanPoint.x, tRecordStartScanPoint.y,
							(*itera).second.tPointAbsCoordInBase.tWeldLinePos.x, (*itera).second.tPointAbsCoordInBase.tWeldLinePos.y)) continue;
						fileCurntLugNoCout << nSuccessScanIndex - (*itera).first << " " << (*itera).second.tPointAbsCoordInBase.tWeldLinePos.x << " "
							<< (*itera).second.tPointAbsCoordInBase.tWeldLinePos.y + dExPos << " " << (*itera).second.tPointAbsCoordInBase.tWeldLinePos.z << endl;
					}
					preItera = itera;
				}
				preItera = m_pMapImgProcOutput->begin();
				fileCurntLugNoCout << nSuccessScanIndex - (*preItera).first << " " << (*preItera).second.tPointAbsCoordInBase.tWeldLinePos.x << " "
					<< (*preItera).second.tPointAbsCoordInBase.tWeldLinePos.y + dExPos << " " << (*preItera).second.tPointAbsCoordInBase.tWeldLinePos.z;
				fileCurntLugNoCout.close();
				/* ��������� */

				/* ��������켣 */
				CString cstr;
				cstr.Format(OUTPUT_PATH + m_ptUnit->GetUnitName() + TRACEPROCRESULT_PATH + "���߳��Ѷ˵��ܱ�_%4d%2d%2d%2d%2d%2d.txt",
					tSystemtime.wYear, tSystemtime.wMonth, tSystemtime.wDay, tSystemtime.wHour, tSystemtime.wMinute, tSystemtime.wSecond
				);
				ofstream ofLaserTrack(cstr);
				for (auto itera = m_pMapImgProcOutput->begin(); itera->first != nSuccessScanIndex; itera++) {
					CString cstrTmp;
					XI_POINT xiPoint = itera->second.tPointAbsCoordInBase.tWeldLinePos;
					cstrTmp.Format("Num:%d\t\t%.3f\t\t%.3f\t\t%.3f\t\t%.3f\t\t%.3f\t\t%.3f\n",
						itera->first,
						itera->second.tPointAbsCoordInBase.tWeldLinePos.x,
						itera->second.tPointAbsCoordInBase.tWeldLinePos.y,
						itera->second.tPointAbsCoordInBase.tWeldLinePos.z,

						itera->second.tPointAbsCoordInBase.tWeldLinePos.y + pRobotDriver->GetCurrentPos().dBX,
						itera->second.tPointAbsCoordInBase.tWeldLinePos.y + pRobotDriver->GetCurrentPos().dBY,
						itera->second.tPointAbsCoordInBase.tWeldLinePos.y + pRobotDriver->GetCurrentPos().dBZ

					);
					m_vtImgProc3DPoint.push_back(xiPoint);
					TRACE(cstrTmp);
					ofLaserTrack << cstrTmp;
				}
				ofLaserTrack.close();
				/* ��������켣 */
			}
			if (SavePointCloud) {
				for (auto& itera = m_pMapImgProcOutput->begin(); itera != m_pMapImgProcOutput->end(); itera++)
				{
					m_vtScanPointCloud.insert(m_vtScanPointCloud.end(),
						itera->second.vtLaserPoint.begin(),
						itera->second.vtLaserPoint.end());
				}
				fclose(pFilePointCloud);
			}
		}
		else {
#ifdef __ScanInitModule_ConsoleDbgPrint
			TRACE("ImgProcDis����ʧ��\n");
			pRobotDriver->m_cLog->Write("ImgProcDis����ʧ��");
			XiMessageBoxOk("ImgProcDis����ʧ��");
#endif
		}
	}m_MutexSuccessScanFlag.unlock();

	pRobotDriver->HoldOff();
	/*****	new�ڴ��ͷ���	 *****/
	//1.	m_pMapImgProcOutput
	delete m_pMapImgProcOutput;
	m_pMapImgProcOutput = NULL;
	//2.	pVtnImgProcInput
	while (m_vtImgProcInput.size() > 0 && m_vtImgProcInput.back() != NULL) {
		//2.1		pBisExit
		delete m_vtImgProcInput.back()->pBisExit;
		m_vtImgProcInput.back()->pBisExit = NULL;

		delete m_vtImgProcInput.back();
		m_vtImgProcInput.back() = NULL;
		m_vtImgProcInput.pop_back();
	}
	/*****	new�ڴ��ͷ���	 *****/
	Sleep(1);
	DoEvent();

	//R0LevelLock
	m_MutexProcThrdExitFlag.lock(); {
		m_bEndPointProcess = TRUE;
	}m_MutexProcThrdExitFlag.unlock();
	return;
}

UINT CScanInitModule::ThreadSavePointCloud(LPVOID pParam)
{
	T_THREAD_SAVEPOINTCLOUD tParam = *(T_THREAD_SAVEPOINTCLOUD*)pParam;
	CString csBuffer;
	while (TRUE)
	{
		//�����̷߳����˳��ź�
		if (*tParam.pExit == true && tParam.pVvtLaserLine->size() <= 0)	break;

		//д������
		tParam.pMutexWrite->lock(); {
			//��ʱ��ֹ�ⲿ��Vectorд��
			tParam.pMutex_MyVec->lock(); {
				for (auto& itera = tParam.pVvtLaserLine->begin();
					itera != tParam.pVvtLaserLine->end();
					itera = tParam.pVvtLaserLine->erase(tParam.pVvtLaserLine->begin()))
				{
					for (auto& _MyLaserData : *itera)
					{
						csBuffer.Format("%d %.3lf %.3lf %.3lf\n", *tParam.pCount, _MyLaserData.x, _MyLaserData.y, _MyLaserData.z);
						fprintf_s(tParam.pOutputFile, csBuffer);
						(*tParam.pCount)++;
					}
#ifdef __ScanInitModule_ConsoleDbgPrint
					TRACE("������߳����һ�����������ݴ洢 nCount:%d\n", (*tParam.pCount) - 1);
#endif
				}
			}tParam.pMutex_MyVec->unlock();
		}tParam.pMutexWrite->unlock();
		Sleep(10);
	}

#ifdef __ScanInitModule_ConsoleDbgPrint
	TRACE("������߳��˳�\n");
#endif
	delete tParam.pVvtLaserLine;
	tParam.pVvtLaserLine = NULL;
	delete tParam.pMutex_MyVec;
	tParam.pMutex_MyVec = NULL;
	return 0;
}

//����̬ɨ�˵㣨�£� ����֧���¿�� 2024/01/07	���̲߳�����new		�˳�ʱ�ͷ�pParam�е�new������pBisExit�ɵ����߳��ͷţ�
UINT CScanInitModule::ThreadAsynDynamicScanEndPointProc(LPVOID pInputParam)
{
	T_THREAD_DYNAMICSCAN_PARAM* pParam = (T_THREAD_DYNAMICSCAN_PARAM*)pInputParam;	//�������
	T_SAVE_IMAGE tSaveImage;														//��ͼ�ṹ��		
	CString cstrImgSaveName;														//�����ͼƬ����
	IplImage* pImageBuffSrc;														//ͼƬָ�루ԭͼ��
	IplImage* pImageBuffDst;														//ͼƬָ�루����ͼ��
	T_START_POINT_INFO tInfo;														//ͼƬ��������Ϣ
	T_ABS_POS_IN_BASE tPointAbsCoordInBase;											//��¼��������
//	T_CART_COOR dMachineCoorsProcess;												//��¼������
	E_START_OR_END eStartOrEnd = E_PIECE_START;										//��ʼ�������Ϊ ���
	vector<CWinThread*> vtpThreadSavePointCloud;									//������̵߳�ָ��
	vector<T_THREAD_SAVEPOINTCLOUD*> vtpThreadSavePCParam;							//������̵߳Ĳ�����
	bool bExitFlagOfSavePCThread(false);											//ָʾ������߳��˳��ı�־

	XiCV_Image_Processing tXiCvImgProc = XiCV_Image_Processing(
		pParam->tCamParam.tDHCameraDriverPara.nMaxWidth,
		pParam->tCamParam.tDHCameraDriverPara.nMaxHeight
	);
	/* ��ʼ�����������ͼ���ά����& �����ά���꣩ ���ƫ���� ɨ���ٶ� ɨ��ʱ�� */
	double dCurPosX = 0;
	double dCurPosY = 0;
	double dCamerCoorX = 0;
	double dCamerCoorY = 0;
	double dCamerCoorZ = 0;
	double dOffsetStartPoint = 0;
	double dScanVel = 600;
	int nIndex = 0;
	CvPoint cpKeyPointSrc;		//ԭͼ�еļ��⽻��λ��
	CvPoint cpKeyPointDst;		//����ͼ�еļ��⽻��λ��
	CvPoint cpOutputPoint;		//�����map�ĵ�
	CString cstrTmp;
	tSaveImage.nImageNo = 0;
	//while(TURE)ѭ���˳���Ψһ������*pParam->pBisExit == true
	while (TRUE)
	{
		/*
		*	�߳��˳���־λ��Ψһ����ѭ���ķ�ʽ��
		*	������˳�while (TRUE)ѭ�� ��pMutexExitFlag�ں�������ǰ��Ϊunlock
		*/
		//R2LevelLock
		pParam->pMutexExitFlag->lock(); {
			if (*pParam->pBisExit == TRUE) {
				break;
			}
		}pParam->pMutexExitFlag->unlock();

		//ѭ������ͼƬ
		//R2LevelLock
		pParam->pMutexVtInput->lock(); {
			//�뿪if (pParam->pVtnImgProcInput->size() != 0)���ʱ����֤pParam->pMutexVtInput״̬Ϊlock
			if (pParam->pVtnImgProcInput->size() != 0) {
				pImageBuffSrc = (*pParam->pVtnImgProcInput->begin()).second.img;
				pImageBuffDst = cvCreateImage(cvSize(pImageBuffSrc->width, pImageBuffSrc->height), pImageBuffSrc->depth, pImageBuffSrc->nChannels);
				tInfo = (*pParam->pVtnImgProcInput->begin()).second;
				nIndex = (*pParam->pVtnImgProcInput->begin()).first;
				pParam->pMutexVtInput->unlock();

				//�ж�ͼƬ��תģʽ
				switch (pParam->eFlipMode)
				{
				case E_FLIP_VERTICAL:
					cvFlip(pImageBuffSrc, pImageBuffDst, 1);
					break;
				case E_FLIP_HORIZEN:
					cvFlip(pImageBuffSrc, pImageBuffDst, 0);
					break;
				case E_FLIP_BOTH:
					cvFlip(pImageBuffSrc, pImageBuffDst, -1);
					break;
				default:
					cvCopyImage(pImageBuffSrc, pImageBuffDst);
					break;
				}

				long long tTime = XI_clock();
				if (pParam->bSaveImg) {   //��Ҫ��ͼ����ԭͼ
					cstrImgSaveName.Format(pParam->cstrImgSavePath + "%d.jpg", nIndex);
					int nParamsArray[3];            //��ͼ��ʽ
					nParamsArray[0] = (int)CV_IMWRITE_JPEG_QUALITY;
					nParamsArray[1] = (int)(0.3 * 100);
					nParamsArray[2] = 0;
					cvSaveImage((LPCSTR)cstrImgSaveName, pImageBuffDst, nParamsArray);
#ifdef __ScanInitModule_ConsoleDbgPrint
					TRACE("����%d��ͼƬ��ʱ��%d\n", nIndex, XI_clock() - tTime);
#endif
					pParam->pRobotDriver->m_cLog->Write("����%d��ͼƬ��ʱ��%d", nIndex, XI_clock() - tTime);
				}
				//���ԭͼ�쳣����ӡ������̨�������־
				if (pImageBuffSrc->nChannels < 0 || pImageBuffSrc->nChannels>3) {
					SYSTEMTIME tSystime = SYSTEMTIME();
					GetSystemTime(&tSystime);
#ifdef __ScanInitModule_ConsoleDbgPrint
					TRACE("IfPieceStartOrEndPoint����ǰ����ImgBuffer�쳣,%d:%d:&d\n", tSystime.wHour, tSystime.wMinute, tSystime.wSecond);
#endif
					pParam->pRobotDriver->m_cLog->Write("�߳�%d IfPieceStartOrEndPoint����ǰ����ImgBuffer�쳣,%d:%d:&d", pParam->nThreadIndex, tSystime.wHour, tSystime.wMinute, tSystime.wSecond);
				}
				BOOL bFlag(FALSE);
				/*
				*		�Ѷ˵�ģʽ�ж��Ƿ��ҵ��˵�
				*		IfPieceStartOrEndPoint����true��ʾ�Ҳ����˵㣬������ȡ����Ϊ�жϱ�־��
				*		bFlagΪtrue��ʾ���ҵ��˵�
				 */
				tTime = XI_clock();
				//XiLineParamNode tFrontLine, tBackLine;
				if (pParam->eScanMode == E_SCANMODE_2DPOINT ||
					pParam->eScanMode == E_SCANMODE_FIXEDSCAN) {
					bFlag = !pParam->pImgProcess->IfPieceStartOrEndPoint(
						pImageBuffDst,
						dScanVel,
						dOffsetStartPoint,
						cpKeyPointDst,
						eStartOrEnd,
						pParam->tImgRecParam.ePieceClass,
						pParam->tImgRecParam.iBaseLineLenInZone,
						pParam->tImgRecParam.iMoveLineLenInZone,
						pParam->tImgRecParam.iRoiWidSize,
						pParam->tImgRecParam.iIfEndConditionIdx
					);
					/*cpKeyPointDst = pParam->pImgProcess->GetGroupStandKeyPoint(pImageBuffDst, tFrontLine, tBackLine, true, false, false);
					if (cpKeyPointDst.x > 0 && cpKeyPointDst.y > 0)
					{
						bFlag = true;
					}*/
				}
				else {
					bFlag = FALSE;
				}
				TRACE("�߳�%d IfPeice��ʱ:%d\n", pParam->nThreadIndex, XI_clock() - tTime);
				//��ȡԭͼ�еļ��⽻��λ��	cpKeyPointSrc
				switch (pParam->eFlipMode) {
				case E_FLIP_VERTICAL:
					cpKeyPointSrc.x = pImageBuffDst->width - cpKeyPointDst.x;
					cpKeyPointSrc.y = cpKeyPointDst.y;
					break;
				case E_FLIP_HORIZEN:
					cpKeyPointSrc.y = pImageBuffDst->height - cpKeyPointDst.y;
					cpKeyPointSrc.x = cpKeyPointDst.x;
					break;
				case E_FLIP_BOTH:
					cpKeyPointSrc.y = pImageBuffDst->height - cpKeyPointDst.y;
					cpKeyPointSrc.x = pImageBuffDst->width - cpKeyPointDst.x;
					break;
				default:
					cpKeyPointSrc = cpKeyPointDst;
					break;
				}
				//R2LevelLock
#ifdef __ScanInitModule_ConsoleDbgPrint
				long long tGetLockFirst = XI_clock();
				
#endif
				pParam->pMutexMapLock->lock(); {
#ifdef __ScanInitModule_ConsoleDbgPrint
					TRACE("�߳�%d��ȡmap��������ʱ:%d\n", pParam->nThreadIndex,XI_clock() - tGetLockFirst);
#endif

					if (pParam->pMapDynamicScanOut->size() > 0 && pParam->pMapDynamicScanOut->rbegin()->first < nIndex) {
						cvCvtColor(pImageBuffSrc, pParam->pShowImg, CV_GRAY2RGB);
						if (bFlag &&
							(cpKeyPointSrc.x > pImageBuffSrc->width * 0.15 && cpKeyPointSrc.x < pImageBuffSrc->width * 0.85) &&
							(cpKeyPointSrc.y > pImageBuffSrc->height * 0.15 && cpKeyPointSrc.x < pImageBuffSrc->height * 0.85)
							) {
							cvCircle(pParam->pShowImg, cpKeyPointSrc, 12, CV_RGB(0, 255, 0), 4);
						}
					}
#ifdef __ScanInitModule_ConsoleDbgPrint
					TRACE("�߳�%d������ά���ʱ:%d\n", pParam->nThreadIndex, XI_clock() - tGetLockFirst);
#endif
				}pParam->pMutexMapLock->unlock();
#ifdef __ScanInitModule_ConsoleDbgPrint
				TRACE("�߳�%d����%d��ͼƬ��ʱ��%d\n\tԭͼ��ά�㣺X %d Y %d\n\t����ͼ��ά�㣺X %d Y %d\n",
					pParam->nThreadIndex, nIndex, XI_clock() - tTime, cpKeyPointSrc.x, cpKeyPointSrc.y, cpKeyPointDst.x, cpKeyPointDst.y
				);
#endif
				pParam->pRobotDriver->m_cLog->Write("�߳�%d����%d��ͼƬ��ʱ��%d\n\tԭͼ��ά�㣺X %d Y %d\n\t����ͼ��ά�㣺X %d Y %d",
					pParam->nThreadIndex, nIndex, XI_clock() - tTime, cpKeyPointSrc.x, cpKeyPointSrc.y, cpKeyPointDst.x, cpKeyPointDst.y
				);

				cpOutputPoint.x = bFlag ? cpKeyPointSrc.x : 0;
				cpOutputPoint.y = bFlag ? cpKeyPointSrc.y : 0;
				if (
					(pParam->eScanMode == E_SCANMODE_2DPOINT || pParam->eScanMode == E_SCANMODE_FIXEDSCAN) &&
					bFlag
					) {
					//����������ά���굽�ļ�����ԭͼ�Ķ�ά�㣩

					TransCamera2DToBase(
						cpKeyPointSrc,
						pParam->tCamParam,
						pParam->cAbsCoorsTrans,
						pParam->pRobotDriver->m_eManipulatorType,
						tInfo.tRobotCoors,
						tInfo.tRobotPulse,
						tPointAbsCoordInBase,
						&tXiCvImgProc
					);
					*pParam->pOutFile << nIndex << " " << tPointAbsCoordInBase.tWeldLinePos.x << " " << tPointAbsCoordInBase.tWeldLinePos.y << " " << tPointAbsCoordInBase.tWeldLinePos.z << std::endl;
				}
				vector<CvPoint> vtLaserPoint2DSrc;				//ͼ�񼤹��ά�㼯
				vector<XI_POINT> vtLaserPoint3D;				//��ά�㼯


				//ʹ��ԭͼ���ɵ���
				if (pParam->bSavePointCloud) {
					int nGetStep = pParam->eScanMode == E_SCANMODE_2DPOINT ? 4 : 4;

					CvPoint cPoint[20000] = { 0,0 };

					int nLength = FindLaserMidPiontEE(pImageBuffSrc, cPoint, 3, 30, 30, 5, 11, 0, 3, 20000, 3, 1, 2, false);

					for (int i = 0; i < nLength; vtLaserPoint2DSrc.push_back(cPoint[i]), i++);
					//���ɼ�����
					for (auto itera = vtLaserPoint2DSrc.begin(); itera != vtLaserPoint2DSrc.end(); itera++) {
						T_ABS_POS_IN_BASE tmpAPIB;
						TransCamera2DToBase(
							*itera,
							pParam->tCamParam,
							pParam->cAbsCoorsTrans,
							pParam->pRobotDriver->m_eManipulatorType,
							tInfo.tRobotCoors,
							tInfo.tRobotPulse,
							tmpAPIB,
							&tXiCvImgProc
						);
						vtLaserPoint3D.push_back(tmpAPIB.tWeldLinePos);
					}

					//����������̣߳�û���������̵߳�����£�
					if (vtpThreadSavePointCloud.size() < pParam->nNumOfSavePointThreads)
					{
						vector<vector<XI_POINT>>* pVvtLaserPoint = new vector<vector<XI_POINT>>;		//��Ŷ������������		�ڴ����������ͷ�
						Mutex* pMutexVtAccess = new Mutex;												//vvtLaserPoint�ķ�����		�ڴ����������ͷ�
						pVvtLaserPoint->push_back(vtLaserPoint3D);
						//����������̣߳��ڴ��ɱ��߳����˳�ǰ�ͷţ�
						T_THREAD_SAVEPOINTCLOUD* pSavePCParam = new
							T_THREAD_SAVEPOINTCLOUD(
								pParam->pOutFilePointCloud,
								pVvtLaserPoint,				/*�ڴ����������ͷ�*/
								pParam->pMutexOutFilePC,
								pMutexVtAccess,				/*�ڴ����������ͷ�*/
								pParam->pSavePointCloudCount,
								&bExitFlagOfSavePCThread
							);
						CWinThread* pThreadSavePC = AfxBeginThread(ThreadSavePointCloud, (LPVOID)pSavePCParam);
						if (pThreadSavePC != NULL)
						{
							//�޸��̵߳������ɱ��߳�ִ�У������������
							pThreadSavePC->m_bAutoDelete = false;
							vtpThreadSavePointCloud.push_back(pThreadSavePC);
							vtpThreadSavePCParam.push_back(pSavePCParam);
						}
						else
						{
							//û�д����ɹ��̣߳���ҳ�ػ�������ܳ������쳣
							if (pSavePCParam != nullptr)
							{
								delete pSavePCParam;
								pSavePCParam = NULL;
							}
#ifdef __ScanInitModule_ConsoleDbgPrint
							TRACE("������%d��������߳�ʧ��,\n", vtpThreadSavePointCloud.size() + 1);
#endif
							pParam->pRobotDriver->m_cLog->Write("������%d��������߳�ʧ��,\n", vtpThreadSavePointCloud.size() + 1);
						}

					}
					else//��������̴߳�����(�Ѿ��������д�����̵߳������)
					{
						int nSelectNo = 0;	//������̶߳�������������С���̵߳�����
						int nMinSize = vtpThreadSavePCParam.front()->pVvtLaserLine->size();	//��С������
						for (auto& itera = vtpThreadSavePCParam.begin(); itera != vtpThreadSavePCParam.end(); itera++)
						{
							//������������С��
							if (nMinSize > (*itera)->pVvtLaserLine->size())
							{
								nMinSize = (*itera)->pVvtLaserLine->size() > nMinSize;
								nSelectNo = distance(vtpThreadSavePCParam.begin(), itera);
							}
							if (nMinSize == 0) break;	//��С�������Ѵﵽ��Сֵ��ֱ���˳�ѭ��
						}
						//��������
						vtpThreadSavePCParam[nSelectNo]->pMutex_MyVec->lock(); {
							vtpThreadSavePCParam[nSelectNo]->pVvtLaserLine->push_back(vtLaserPoint3D);
						}vtpThreadSavePCParam[nSelectNo]->pMutex_MyVec->unlock();
					}

				}
				pair<int, T_THREAD_DYNAMICSCAN_OUT> pirTmp = make_pair(
					nIndex,
					T_THREAD_DYNAMICSCAN_OUT(cpOutputPoint, tPointAbsCoordInBase, bFlag, vtLaserPoint3D)
				);
				T_IMAGE_PROCINPUT_PARAM tTmpParam = T_IMAGE_PROCINPUT_PARAM(pParam->pMapDynamicScanOut, pirTmp);

				//ִ�л��⴦��
				//R2LevelLock
				pParam->pMutexMapLock->lock(); {
					pParam->pAsynProc->MutexFunction(ImgProcInsertPairInMap, (LPVOID)&tTmpParam, 0, 1);
				}pParam->pMutexMapLock->unlock();
				//�ͷ�ͼ���ڴ� 
				if (pImageBuffSrc != NULL && &pImageBuffSrc != NULL) cvReleaseImage(&pImageBuffSrc);
				if (pImageBuffDst != NULL && &pImageBuffDst != NULL) cvReleaseImage(&pImageBuffDst);

				//R2LevelLock
				pParam->pMutexVtInput->lock(); {
					pParam->pVtnImgProcInput->begin() = pParam->pVtnImgProcInput->erase(pParam->pVtnImgProcInput->begin());
				}
			}
		}pParam->pMutexVtInput->unlock();
		//�����ڹ�����߳�ǰռ�ù���CPU��Դ
		Sleep(5);
	}

	//�ȴ�������̴߳������,�ͷ��ڴ�
	if (pParam->bSavePointCloud)
	{
		bExitFlagOfSavePCThread = true;
		for (CWinThread* pThread : vtpThreadSavePointCloud)
		{
			WaitForSingleObject(pThread->m_hThread, INFINITE);
			pThread->Delete();
		}
		for (auto& _MyParam : vtpThreadSavePCParam)
		{
			delete _MyParam;
			_MyParam = NULL;
		}
	}
	/* �ͷ�pParam��new�ڴ� */
	//1.pVtnImgProcInput��δ�������ͼƬ�ڴ�ȫ���ͷ�
	for (auto itera = pParam->pVtnImgProcInput->begin(); itera != pParam->pVtnImgProcInput->end(); cvReleaseImage(&itera->second.img), itera++);
	//2.pVtnImgProcInput
	if (NULL != pParam->pVtnImgProcInput)
	{
		delete pParam->pVtnImgProcInput;
		pParam->pVtnImgProcInput = NULL;
	}
	//3.pImgProcess
	if (NULL != pParam->pImgProcess)
	{
		delete pParam->pImgProcess;
		pParam->pImgProcess = NULL;
	}
	//4.pMutexVtInput
	if (NULL != pParam->pMutexVtInput)
	{
		delete pParam->pMutexVtInput;
		pParam->pMutexVtInput = NULL;
	}
	//5.pOutFile
	pParam->pOutFile->close();
	if (NULL != pParam->pOutFile)
	{
		delete pParam->pOutFile;
		pParam->pOutFile = NULL;
	}
	//�ͷ��߳��˳���־��
	*pParam->pBisExit = FALSE;
	pParam->pMutexExitFlag->unlock();
	return 0;
}

BOOL CScanInitModule::DynamicEndPointResultPro(CRobotDriverAdaptor *pRobotDriver)
{
////////    CString strPath;
////////    strPath.Format(".\\WeldData\\%s\\%d_", pRobotDriver->m_strRobotName, m_eEndPointType);
////////    //string strPathAdr = strPath;
////////    std::vector<T_ROBOT_COORS> vtStartCoors,vtStartScanData;
////////    std::vector<XI_POINT> vtReferCoors;
////////    vtReferCoors.clear();
////////    vtStartCoors.clear();
////////	vtStartScanData.clear();
////////    std::ifstream fileCurntLugNo(strPath + "CoutStartPointInfoFileInWorldCopy.txt");
////////    std::ofstream fileCurnt(strPath + "CoutStartPointInfoFileInWorldSmooth2.txt");//����˲�������*/
////////	std::ofstream fileRobotCoors(strPath + "StartPointInfoFileInRobotCoors.txt");//����˲�������*/
////////	int nSeq;
////////
////////	T_ROBOT_COORS tStartCoors;
////////	while (!fileCurntLugNo.eof())
////////	{
////////		fileCurntLugNo >> nSeq >> tStartCoors.dX >> tStartCoors.dY >> tStartCoors.dZ;
////////		XI_POINT tPointCoor = { tStartCoors.dX , tStartCoors.dY, tStartCoors.dZ };
////////		vtReferCoors.push_back(tPointCoor);
////////		vtStartCoors.push_back(tStartCoors);
////////	}
////////	fileCurntLugNo.close();
////////	//���㷨��Ƕ�
////////	T_LINE_PARA	tLine = CalcLineParamRansac(vtReferCoors, 0.7);
////////	double dNormal = atan2(tLine.dDirY, tLine.dDirX) * 180 / 3.1415926 - 90 * pRobotDriver->m_nRobotInstallDir;
////////
////////	int width = vtReferCoors.size();//�����˲�����
////////	pRobotDriver->m_cLog->Write("�˲�ǰ��ֵ %d", width);
////////	double dis = TwoPointDis(vtReferCoors.at(width - 1).x, vtReferCoors.at(width - 1).y, vtReferCoors.at(width - 1).z, vtReferCoors.at(0).x,
////////		vtReferCoors.at(0).y, vtReferCoors.at(0).z);
////////
////////	std::ofstream fileAdjustAfter(strPath + "CoutStartPointInfoFileInWorldCopyAfter.txt");
////////	// ͶӰ
////////	XI_POINT tProjectpoint;
////////	PointtoLineProjection(tLine, vtReferCoors[0], tProjectpoint);
////////	vtStartCoors.clear();
////////	int nPontNum = dis / 2.0;
////////	for (int iPointNo = 0; iPointNo < nPontNum; iPointNo++)
////////	{
////////		T_ROBOT_COORS tCoor;
////////		tCoor.dX = tProjectpoint.x + iPointNo * 2 * tLine.dDirX;
////////		tCoor.dY = tProjectpoint.y + iPointNo * 2 * tLine.dDirY;
////////		tCoor.dZ = tProjectpoint.z + iPointNo * 2 * tLine.dDirZ;
////////		vtStartCoors.push_back(tCoor);
////////		fileAdjustAfter << tCoor.dX << " " << tCoor.dY << " " << tCoor.dZ << std::endl;
////////	}
////////	fileAdjustAfter.close();
////////	width = vtStartCoors.size();
////////
////////	if (dis < 30 || width < 7)
////////	{
////////		XiMessageBox("%d�Ż������˲�ǰ��ֵ�쳣 %lf", pRobotDriver->m_nRobotNo, dis);
////////		return FALSE;
////////	}
////////
////////    CvPoint3D64f cp3DOutPoint;
////////    CvPoint3D64f cp3DInPoint;
////////    BOOL bIfOutPointValid;
////////    CvPoint3D64f cpTrackDir;
////////    CvPoint2D64f cpTrackNormalDir;
////////    BOOL bIfDirValid;
////////    double dDirWinMaxLen = 60;
////////    XI_POINT tPoint;
////////	T_ROBOT_COORS tRobotCoor;
////////    m_vtScanInitWeldLineInWorldPoints.clear();
////////    TrackFilter_Init(pRobotDriver->m_nRobotSmooth, 3.0);
////////    double dTrackVerDegree;
////////    double dTrackVerDegree6th;
////////    //������
////////    double dInitComp = 0.0/*pRobotDriver->m_dGunToEyeCompenX*/;
////////    double dAbjustHeight = 0.0/*pRobotDriver->m_dGunToEyeCompenZ*/; // ���ϲ��� ���²���
////////	BOOL bNormalDir = FALSE;
////////	if (-1 == pRobotDriver->m_nRobotInstallDir)
////////	{
////////		bNormalDir = TRUE;
////////	}
////////    std::vector<TrackFilter_Node> CoutTrackPoint;
////////    for (int index = 0; index < width; index++)
////////    {
////////        CoutTrackPoint.clear();
////////        cp3DInPoint.x = vtStartCoors[index].dX;
////////        cp3DInPoint.y = vtStartCoors[index].dY;
////////        cp3DInPoint.z = vtStartCoors[index].dZ;
////////
////////        CoutTrackPoint = TrackFilter_FilterCurvePointInPointOut(pRobotDriver->m_nRobotSmooth, cp3DInPoint.x, cp3DInPoint.y, cp3DInPoint.z, bNormalDir);
////////        pRobotDriver->m_cLog->Write("��ʼ���˲�:%d %d", index, CoutTrackPoint.size());
////////        for (int nSize = 0; nSize<CoutTrackPoint.size(); nSize++)
////////        {
////////            // ��ʼ�β���
////////            dTrackVerDegree = atan2(CoutTrackPoint[nSize].normalDir_.y_, CoutTrackPoint[nSize].normalDir_.x_) * 180.0 / PI;
////////            cp3DOutPoint.x = CoutTrackPoint[nSize].pnt_.x_ + (dInitComp * CosD(dTrackVerDegree));
////////            cp3DOutPoint.y = CoutTrackPoint[nSize].pnt_.y_ + (dInitComp * SinD(dTrackVerDegree));
////////            cp3DOutPoint.z = CoutTrackPoint[nSize].pnt_.z_ + (dAbjustHeight);
////////            pRobotDriver->m_cLog->Write("index:%d, �˲��󲹳���  %.3lf  %.3lf  %.3lf dTrackVerDegree:%lf %lf %lf  ", index, cp3DOutPoint.x, cp3DOutPoint.y, cp3DOutPoint.z, dTrackVerDegree, CoutTrackPoint[nSize].normalDir_.y_, CoutTrackPoint[nSize].normalDir_.x_);
////////			tRobotCoor.dX = tPoint.x = cp3DOutPoint.x;
////////			tRobotCoor.dY = tPoint.y = cp3DOutPoint.y;
////////			tRobotCoor.dZ = tPoint.z = cp3DOutPoint.z;
////////			tRobotCoor.dRX = m_dPlatWeldRx;
////////			tRobotCoor.dRY = m_dPlatWeldRy;
////////			tRobotCoor.dRZ = DirAngleToRz(dNormal)/*dTrackVerDegree*/;
////////
////////            pRobotDriver->m_cLog->Write("��ʼ�� ���˲���%lf %lf %lf ", cp3DOutPoint.x, cp3DOutPoint.y, cp3DOutPoint.z);
////////            m_vtScanInitWeldLineInWorldPoints.push_back(tPoint);
////////			vtStartScanData.push_back(tRobotCoor);
////////        }
////////    }
////////    int nSize = m_vtScanInitWeldLineInWorldPoints.size();
////////
////////    pRobotDriver->m_cLog->Write("�˲�����ֵ��%d ", nSize);
////////    if (nSize <= 2)
////////    {
////////
////////        XiMessageBox("�˲�����ֵ�쳣 %d", nSize);
////////        return FALSE;
////////    }
////////    for (int n = 0; n<nSize; n++)
////////    {
////////        fileCurnt << n << " " << m_vtScanInitWeldLineInWorldPoints[n].x << " " << m_vtScanInitWeldLineInWorldPoints[n].y << " " << m_vtScanInitWeldLineInWorldPoints[n].z << std::endl;
////////		fileRobotCoors << vtStartScanData.at(n).dX << " " << vtStartScanData.at(n).dY << " " << vtStartScanData.at(n).dZ << " " <<
////////			vtStartScanData.at(n).dRX << " " << vtStartScanData.at(n).dRY << " " << vtStartScanData.at(n).dRZ << std::endl;
////////	}
////////    fileCurnt.close();
////////	fileRobotCoors.close();
////////    pRobotDriver->m_cLog->Write("ɨ��������� size:%d. %lf %lf %lf ", nSize, m_vtScanInitWeldLineInWorldPoints.at(0).x, m_vtScanInitWeldLineInWorldPoints.at(0).y, m_vtScanInitWeldLineInWorldPoints.at(0).z);
////////    m_tRealEndPoint = m_vtScanInitWeldLineInWorldPoints[0];
////////    pRobotDriver->m_cLog->Write("ʵ�ʵ����:%lf %lf %lf", m_tRealEndPoint.x, m_tRealEndPoint.y, m_tRealEndPoint.z);
    return TRUE;
}

BOOL CScanInitModule::StartDataPointPro(CRobotDriverAdaptor* pRobotDriver, std::vector<T_ROBOT_COORS>& vtStartCoors, BOOL bIsTracking, double dTheoryLength,bool bFirstFit)
{ 
	if (!bIsTracking){return TRUE;}
	CString strPath;
	strPath.Format("%s%s%s", OUTPUT_PATH, m_ptUnit->GetRobotCtrl()->m_strRobotName, RECOGNITION_FOLDER);
	std::vector<XI_POINT> vtReferCoors;
	std::vector<T_ROBOT_COORS> vtCoors;
	vtReferCoors.clear();
	vtCoors.clear();
	vtCoors = vtStartCoors;
	vtStartCoors.clear();
	std::ofstream fileCurnt(strPath + "CoutStartPointInfoFileInWorldSmooth.txt");//����˲�������*/
	TrackFilter_Init(m_ptUnit->m_nRobotSmooth, m_pTraceModel->m_dMoveStepDis, /*dTheoryLength*/0.0,15);
	double dTrackVerDegree;
//	double dTrackVerDegree6th;
	int nSizeCoors = vtCoors.size();
	//������
	double dInitComp = m_pTraceModel->m_dGunToEyeCompenX;
	double dAbjustHeight = m_pTraceModel->m_dGunToEyeCompenZ/**pRobotDriver->m_nRobotInstallDir*/; // ���ϲ��� ���²���
	if (!bFirstFit)
	{
		dInitComp = 0.0;
		dAbjustHeight = 0.0;
	}
	T_ROBOT_COORS tPoint;
	if (m_ptUnit->m_bBreakPointContinue && !m_ptUnit->GetExAxisType(m_ptUnit->m_nTrackAxisNo))
	{
		//�ϵ��������������Ѿ��ӹ�������
		double dAngle = 0.0;

		dAngle = CalcPolarAngle((vtCoors.at(nSizeCoors - 1).dX + vtCoors.at(nSizeCoors - 1).dBX) - (vtCoors.at(0).dX, vtCoors.at(0).dBX),
								(vtCoors.at(nSizeCoors - 1).dY + vtCoors.at(nSizeCoors - 1).dBY) - (vtCoors.at(0).dY + vtCoors.at(0).dBY)) - 90 * pRobotDriver->m_nRobotInstallDir;

		for (int index = 0; index < vtCoors.size(); index++)
		{
			vtCoors.at(index).dX -= dInitComp * CosD(dAngle);
			vtCoors.at(index).dY -= dInitComp * SinD(dAngle);
			vtCoors.at(index).dZ -= dAbjustHeight;
		}		

		//���ػ���λ��
		T_ROBOT_COORS tPt = vtCoors.at(vtCoors.size() - 1);
		tPoint = tPt;
		CString strFile;
		strFile.Format("%s\\�˲���������.txt", strPath);
		std::ifstream cin(strFile);
		while (!cin.eof())
		{
			cin >> tPoint.dX >> tPoint.dY >> tPoint.dZ;
			tPoint.dBY = tPoint.dY - tPt.dY;
			tPoint.dY = tPt.dY;
			//vtCoors.push_back(tPoint);
		}
		cin.close();
	}

	std::vector<TrackFilter_Node> CoutTrackPoint;
	bool bNormalDir = false;
	if (-1 == pRobotDriver->m_nRobotInstallDir)
	{
		bNormalDir = TRUE;
	}
	for (int index = 0; index < vtCoors.size(); index++)
	{
		CoutTrackPoint.clear();
		CoutTrackPoint = TrackFilter_FilterCurvePointInPointOut(m_ptUnit->m_nRobotSmooth, vtCoors[index].dX + vtCoors[index].dBX,
			vtCoors[index].dY + vtCoors[index].dBY, vtCoors[index].dZ + vtCoors[index].dBZ, bNormalDir);

		pRobotDriver->m_cLog->Write("��ʼ���˲�:%d %d", index, CoutTrackPoint.size());
		for (int nSize = 0; nSize < CoutTrackPoint.size(); nSize++)
		{
			T_ROBOT_COORS tPoint = vtCoors[index];
			// ��ʼ�β���
			dTrackVerDegree = atan2(CoutTrackPoint[nSize].normalDir_.y_, CoutTrackPoint[nSize].normalDir_.x_) * 180.0 / PI;
			double dDy = tPoint.dY;
			if (1 == labs(m_ptUnit->m_nTrackAxisNo))
			{
				dDy = tPoint.dX;
				tPoint.dX = dDy;
				tPoint.dY = CoutTrackPoint[nSize].pnt_.y_ + (dInitComp * SinD(dTrackVerDegree));
				tPoint.dBX = CoutTrackPoint[nSize].pnt_.x_ + (dInitComp * CosD(dTrackVerDegree)) - dDy;
			}
			else if (2 == labs(m_ptUnit->m_nTrackAxisNo))
			{
				dDy = tPoint.dY;
				tPoint.dX = CoutTrackPoint[nSize].pnt_.x_ + (dInitComp * CosD(dTrackVerDegree));
				tPoint.dY = dDy;
				tPoint.dBY = CoutTrackPoint[nSize].pnt_.y_ + (dInitComp * SinD(dTrackVerDegree)) - dDy;
			}

			tPoint.dZ = CoutTrackPoint[nSize].pnt_.z_ + (dAbjustHeight);
			tPoint.dRZ = m_pRobotDriver->DirAngleToRz(dTrackVerDegree);		
			
			fileCurnt << tPoint.dX << " " << tPoint.dY << " " << tPoint.dZ << " " << tPoint.dRX << " " << tPoint.dRY << " " << tPoint.dRZ << " " << tPoint.dBY << std::endl;
			vtStartCoors.push_back(tPoint);

			pRobotDriver->m_cLog->Write("index:%d, �˲��󲹳���  %.3lf  %.3lf  %.3lf %.3lf dTrackVerDegree:%lf %lf %lf  ", index,
				tPoint.dX, tPoint.dY, tPoint.dZ, tPoint.dBY, dTrackVerDegree, CoutTrackPoint[nSize].normalDir_.y_, CoutTrackPoint[nSize].normalDir_.x_);
		}
	}
	fileCurnt.close();
	if (vtStartCoors.size() <= 2)
	{

		XUI::MesBox::PopOkCancel("�˲�����ֵ�쳣 {0}", vtStartCoors.size());
		return false;
	}
	return true;
}

BOOL CScanInitModule::DataPointsProcess(CRobotDriverAdaptor* pRobotDriver, std::vector<XI_POINT> vtCinPoints,
	std::vector<XI_POINT> &vtCoutPoints, vector<T_ROBOT_COORS> &vtCoutRobotCoors)
{
	vtCoutPoints.clear();
	vtCoutRobotCoors.clear();
	
	TrackFilter_Init(m_ptUnit->m_nRobotSmooth, 3.0);
	std::vector<TrackFilter_Node> CoutTrackPoint;
	bool bNormalDir = false;
	if (-1 == pRobotDriver->m_nRobotInstallDir)
	{
		bNormalDir = TRUE;
	}
	for (int index = 0; index < vtCinPoints.size(); index++)
	{
		CoutTrackPoint.clear();
		CoutTrackPoint = TrackFilter_FilterCurvePointInPointOut(m_ptUnit->m_nRobotSmooth, vtCinPoints[index].x,
			vtCinPoints[index].y, vtCinPoints[index].z, bNormalDir);

		pRobotDriver->m_cLog->Write("��ʼ���˲�:%d %d", index, CoutTrackPoint.size());
		for (int nSize = 0; nSize < CoutTrackPoint.size(); nSize++)
		{
			// ��ʼ�β���
			double dTrackVerDegree = atan2(CoutTrackPoint[nSize].normalDir_.y_, CoutTrackPoint[nSize].normalDir_.x_) * 180.0 / PI;

			XI_POINT tXiPoint = { CoutTrackPoint[nSize].pnt_.x_,CoutTrackPoint[nSize].pnt_.y_,CoutTrackPoint[nSize].pnt_.z_ };
			T_ROBOT_COORS tRobotPoint(CoutTrackPoint[nSize].pnt_.x_, CoutTrackPoint[nSize].pnt_.y_, CoutTrackPoint[nSize].pnt_.z_,
				m_dPlatWeldRx, m_dPlatWeldRy, m_pRobotDriver->DirAngleToRz(dTrackVerDegree),0.0,0.0,0.0);
			
			vtCoutPoints.push_back(tXiPoint);
			vtCoutRobotCoors.push_back(tRobotPoint);

			pRobotDriver->m_cLog->Write("index:%d, �˲��󲹳���  %.3lf  %.3lf  %.3lf dTrackVerDegree:%.3lf  ", index,
				CoutTrackPoint[nSize].normalDir_.y_, CoutTrackPoint[nSize].normalDir_.x_, CoutTrackPoint[nSize].normalDir_.y_, dTrackVerDegree);
		}
	}
	if (vtCoutRobotCoors.size() <= 2)
	{
		XUI::MesBox::PopOkCancel("�˲�����ֵ�쳣 {0}", vtCoutRobotCoors.size());
		return false;
	}
	return true;
}

BOOL CScanInitModule::SinglePointMeasure(CRobotDriverAdaptor* pRobotCtrl)
{ 
	if (GetLocalDebugMark())
	{
		return true;
	}
	//��ɨ��ε�ǰ��Ҫ��ʼ����������б��
	bool bIsFlip = false;
	CDHGigeImageCapture* pDHCamDrv = (CDHGigeImageCapture*)m_ptUnit->GetCameraCtrl(m_ptUnit->m_nMeasureCameraNo);	
	CvSize ImageSize;
	ImageSize.width = m_ptUnit->m_vtCameraPara[m_ptUnit->m_nMeasureCameraNo].tDHCameraDriverPara.nRoiWidth;
	ImageSize.height = m_ptUnit->m_vtCameraPara[m_ptUnit->m_nMeasureCameraNo].tDHCameraDriverPara.nRoiHeight;
	IplImage* cvImage = cvCreateImage(ImageSize, IPL_DEPTH_8U, 1);
	CString sIniFile;
	sIniFile = m_ptUnit->GetTrackTipDataFileName(m_ptUnit->m_nMeasureCameraNo);
	//sIniFile.Format("%s%s\\TrackDipParam_Cam%d.ini", DATA_PATH, m_ptUnit->m_tContralUnit.strUnitName, m_ptUnit->m_nMeasureCameraNo);
	GROUP_STAND_DIP_NS::CImageProcess* pImgProcess = new GROUP_STAND_DIP_NS::CImageProcess(ImageSize.width, ImageSize.height, pRobotCtrl->m_nRobotNo, sIniFile);
	// ��ȡ����
	T_ROBOT_COORS tCoord = pRobotCtrl->GetCurrentPos();
	T_ANGLE_PULSE tPulse = pRobotCtrl->GetCurrentPulse();
	// ��ʱ�޸�
	double dExAxisPos_y = tCoord.dBY;
#ifdef SINGLE_ROBOT
	double dExAxisPos = tCoord.dBY;;
#else
	double dExAxisPos = tCoord.dBX;
#endif // SINGLE_ROBOT

	// �ɼ�ͼ��
	static int nNumber = 0;
	CString strOrgPicPath;
	strOrgPicPath.Format(".\\SteelStructure\\Picture\\Org\\SinglePointMeasure%d.jpg", nNumber);

	pDHCamDrv->CaptureImage(cvImage, 1);// cvCopyImage(, cvImage);
	SaveImage(cvImage, strOrgPicPath);

	FlipImage(cvImage,*m_peScanFlipMode);
	// ͼ����
	CvPoint cpMidKeyPoint, cpBesideKeyPoint;
	vector<CvPoint> vtLeftPtns, vtRightPtns;
	pImgProcess->GetBesideKeyPoints(cvImage, cpMidKeyPoint, cpBesideKeyPoint, vtLeftPtns, vtRightPtns, false,
		500, 500,
		300, 300,
		20, 20);

	// ����ǵ����������	
	ResumeImagePoint2D(ImageSize.width, ImageSize.height, cpMidKeyPoint, *m_peScanFlipMode);

	T_ROBOT_COORS tRetCoord = m_ptUnit->TranImageToBase(m_ptUnit->m_nMeasureCameraNo, cpMidKeyPoint, tCoord, tPulse);


	// ����ͼ���㡢��ʾ������
	ResumeImagePoint2D(cvImage->width, cvImage->height, cpMidKeyPoint, *m_peScanFlipMode);
	cvCvtColor(cvImage, m_pShowImage, CV_GRAY2RGB);
	cvCircle(m_pShowImage, cpMidKeyPoint, 8, CV_RGB(255, 0, 0), 4);
	CString strProPicPath;
	strProPicPath.Format(".\\SteelStructure\\Picture\\Pro\\SinglePointMeasure%d.jpg", nNumber);
	nNumber++;
	SaveImage(m_pShowImage, strProPicPath);
	delete pImgProcess;
	// ������������ �� �������ֵ
	m_tDynamicScanEndPoint.x = tRetCoord.dX;
	m_tDynamicScanEndPoint.y = tRetCoord.dY;
	m_tDynamicScanEndPoint.z = tRetCoord.dZ;
#ifdef SINGLE_ROBOT
	m_tDynamicScanEndPoint.y += dExAxisPos;
#else
	m_tDynamicScanEndPoint.x += dExAxisPos;
	m_tDynamicScanEndPoint.y += dExAxisPos_y;
#endif // SINGLE_ROBOT
	return true;
}

BOOL CScanInitModule::ScanEndpointLockInit(CRobotDriverAdaptor *pRobotDriver) { 
	XiLineParamNode tFrontLine, tBackLine;
	CvPoint cpKeyPoint;
	CString cstrTrackDipParamPath;
	BOOL bFlag(TRUE);
	CDHGigeImageCapture *pDHCam=NULL;

	//m_pLaserVision->CamId2ImageProcess(eCamId)->m_nShowImage = 1;

	cstrTrackDipParamPath = m_ptUnit->GetTrackTipDataFileName(m_ptUnit->m_nTrackCameraNo);
	if (m_pTraceImgProcess == NULL) {
		m_pTraceImgProcess = new CImageProcess(
			m_ptUnit->GetCameraParam(m_ptUnit->m_nTrackCameraNo).tDHCameraDriverPara.nMaxWidth,
			m_ptUnit->GetCameraParam(m_ptUnit->m_nTrackCameraNo).tDHCameraDriverPara.nMaxHeight,
			1,
			cstrTrackDipParamPath
		);
	}
	m_pTraceImgProcess->InitIfStartOrEndPntParam(GROUP_STAND_DIP_NS::E_PIECE_START);
	Sleep(50);
	WaitForSingleObject(m_hGetCameraCtrl, INFINITE);
	pDHCam = (CDHGigeImageCapture*)m_ptUnit->GetCameraCtrl(m_ptUnit->m_nTrackCameraNo);
	ReleaseSemaphore(m_hGetCameraCtrl, 1, NULL);
	if (pDHCam == NULL) {
		XiMessageBoxOk("������������ʼ������������");
	}
	//m_pTraceLockImg = cvCloneImage(pDHCam->CaptureImage(FALSE));
	if (!pDHCam->CaptureImage(m_pTraceLockImg, 1))
	{
		//pDHCam->ShowErrorString();
	}
	SaveImage(m_pTraceLockImg, OUTPUT_PATH+m_ptUnit->GetUnitName()+ SEARCH_SCANLOCK_IMG);
	cpKeyPoint = m_pTraceImgProcess->GetGroupStandKeyPoint(m_pTraceLockImg, tFrontLine, tBackLine, TRUE, FALSE, FALSE, TRUE);
	bFlag = bFlag && (cpKeyPoint.x > m_pTraceLockImg->width * 0.15 && cpKeyPoint.x < m_pTraceLockImg->width * 0.85);
	bFlag = bFlag && (cpKeyPoint.y > m_pTraceLockImg->height * 0.15 && cpKeyPoint.y < m_pTraceLockImg->height * 0.85);
	if (m_pTraceLockImg != NULL) {
		//�õ�Ԫ������Ƭ���ȴ���Ƭ�����ͷ��ڴ�
		cvReleaseImage(&m_pTraceLockImg);
	}
	if (!bFlag) XiMessageBoxOk("��������б������ʧ�ܣ�(������룺�Ҳ����ؼ���)");
	return bFlag;
}


BOOL CScanInitModule::RealTimeTrackLockArcBefor(CRobotDriverAdaptor *pRobotDriver, BOOL bIsContinueWeld) 
{ 
	XiLineParamNode tFrontLine, tBackLine;
	CvPoint cpKeyPoint;
	CString cstrTrackDipParamPath;
	BOOL bFlag(TRUE);
	CDHGigeImageCapture* pDHCam = NULL;

	//m_pLaserVision->CamId2ImageProcess(eCamId)->m_nShowImage = 1;

	cstrTrackDipParamPath = m_ptUnit->GetTrackTipDataFileName(m_ptUnit->m_nTrackCameraNo);
	if (m_pTraceImgProcess == NULL) {
		m_pTraceImgProcess = new CImageProcess(
			m_ptUnit->GetCameraParam(m_ptUnit->m_nTrackCameraNo).tDHCameraDriverPara.nMaxWidth,
			m_ptUnit->GetCameraParam(m_ptUnit->m_nTrackCameraNo).tDHCameraDriverPara.nMaxHeight,
			1,
			cstrTrackDipParamPath
		);
	}
	m_pTraceImgProcess->InitIfStartOrEndPntParam(GROUP_STAND_DIP_NS::E_PIECE_START);
	pDHCam = (CDHGigeImageCapture*)m_ptUnit->GetCameraCtrl(m_ptUnit->m_nTrackCameraNo);

	if (pDHCam == NULL) {
		XiMessageBoxOk("������������ʼ������������");
	}
	if (!bIsContinueWeld){
		//m_pTraceLockImg = cvCloneImage(pDHCam->CaptureImage(FALSE));
		if (!pDHCam->CaptureImage(m_pTraceLockImg, 1))
		{
			//pDHCam->ShowErrorString();
		}
		SaveImage(m_pTraceLockImg, OUTPUT_PATH + m_ptUnit->GetUnitName() + SEARCH_SCANLOCK_IMG);
	}
	else {
		m_pTraceLockImg = cvLoadImage(OUTPUT_PATH + m_ptUnit->GetUnitName() + SEARCH_SCANLOCK_IMG,0);
	}
	
	CvPoint cpBesideKeyPoint;
	vector<CvPoint> vtLeftPtns, vtRightPtns;
	m_pTraceImgProcess->GetBesideKeyPoints(m_pTraceLockImg, cpKeyPoint, cpBesideKeyPoint, vtLeftPtns, vtRightPtns, false,
		500, 500,
		300, 300,
		20, 20);
	if (vtLeftPtns.size()<1||vtLeftPtns.size()<1)
	{
		XiMessageBoxOk("��������б������ʧ�ܣ�(������룺�Ҳ����������ϵĵ�)"); return false;
	}
	CvPoint LeftPtn = vtLeftPtns[vtLeftPtns.size() / 2];
	CvPoint RightPtn = vtRightPtns[vtRightPtns.size() / 2];

	cpKeyPoint = m_pTraceImgProcess->HandlockKeyPoint(cpKeyPoint, LeftPtn, RightPtn, tFrontLine, tBackLine);

	m_pTraceModel->tFrontLine = tFrontLine;
	m_pTraceModel->tBackLine = tBackLine;

	//cpKeyPoint = m_pTraceImgProcess->GetGroupStandKeyPoint(m_pTraceLockImg, tFrontLine, tBackLine, TRUE, FALSE, FALSE, TRUE);
	bFlag = bFlag && (cpKeyPoint.x > m_pTraceLockImg->width * 0.15 && cpKeyPoint.x < m_pTraceLockImg->width * 0.85);
	bFlag = bFlag && (cpKeyPoint.y > m_pTraceLockImg->height * 0.15 && cpKeyPoint.y < m_pTraceLockImg->height * 0.85);

	CvPoint p1 = cvPoint(0, tFrontLine.b);
	CvPoint p2 = cvPoint(m_pTraceLockImg->width, m_pTraceLockImg->width * tFrontLine.k + tFrontLine.b);
	CvPoint p3 = cvPoint(0, tBackLine.b);
	CvPoint p4 = cvPoint(m_pTraceLockImg->width, m_pTraceLockImg->width * tBackLine.k + tBackLine.b);

	cvCvtColor(m_pTraceLockImg, m_pShowImage, CV_GRAY2RGB);
	cvCircle(m_pShowImage, cpKeyPoint, 10, CV_RGB(255, 0, 0), 3);
	cvLine(m_pShowImage, p1, p2, CV_RGB(0, 255, 0), 2);
	cvLine(m_pShowImage, p3, p4, CV_RGB(0, 0, 255), 2);

	if (m_pTraceLockImg != NULL) {
		//�õ�Ԫ������Ƭ���ȴ���Ƭ�����ͷ��ڴ�
		cvReleaseImage(&m_pTraceLockImg);
	}
	if (!bFlag) XiMessageBoxOk("��������б������ʧ�ܣ�(������룺�Ҳ����ؼ���)");
	return bFlag;
}

void CScanInitModule::SetTrackProcessParam(int nLayerNo)
{
	CString cstrTrackDipParamPath;
	cstrTrackDipParamPath = m_ptUnit->GetTrackTipDataFileName(m_ptUnit->m_nTrackCameraNo);
	if (m_pTraceImgProcess == NULL)
	{
		m_pTraceImgProcess = new CImageProcess(
			m_ptUnit->GetCameraParam(m_ptUnit->m_nTrackCameraNo).tDHCameraDriverPara.nMaxWidth,
			m_ptUnit->GetCameraParam(m_ptUnit->m_nTrackCameraNo).tDHCameraDriverPara.nMaxHeight,
			1,
			cstrTrackDipParamPath
		);
	}
	if (nLayerNo>0)
	{
		m_pTraceImgProcess->m_nBig_RoiFrontXOffsetSt = -200;
		m_pTraceImgProcess->m_nBig_RoiFrontXOffsetEnd = -60;
		m_pTraceImgProcess->m_nBig_RoiBackXOffsetSt = 60;
		m_pTraceImgProcess->m_nBig_RoiBackXOffsetEnd = 200;
	}
}
BOOL CScanInitModule::RealTimeTrackLockArcAfter(CRobotDriverAdaptor *pRobotDriver) { return FALSE; }

void CScanInitModule::RealTimeTracking(CRobotDriverAdaptor * pRobotCtrl)
{
#if 1
	//CHECK_ROBOT_EMG(pRobotCtrl);
	CString strRobot = pRobotCtrl->m_strRobotName;
	CString strPath;
	strPath.Format("%s%s%s", OUTPUT_PATH, pRobotCtrl->m_strRobotName, WELDDATA_PATH);
	std::string strPathAdr = strPath;

	CString cstrTrackDipParamPath;
	// ��ʼ�������Դ
	CDHGigeImageCapture* pDHCam = NULL;
	pDHCam = (CDHGigeImageCapture*)m_ptUnit->GetCameraCtrl(m_ptUnit->m_nTrackCameraNo);
	// ��������������
	cstrTrackDipParamPath = m_ptUnit->GetTrackTipDataFileName(m_ptUnit->m_nTrackCameraNo);
	GROUP_STAND_DIP_NS::CImageProcess* pImageProcess = m_pTraceImgProcess;
	// ����ͼƬ��Դ
	IplImage *pImageBuff = cvCreateImage(cvSize(
		m_ptUnit->GetCameraParam(m_ptUnit->m_nTrackCameraNo).tDHCameraDriverPara.nMaxWidth,
		m_ptUnit->GetCameraParam(m_ptUnit->m_nTrackCameraNo).tDHCameraDriverPara.nMaxHeight),
		IPL_DEPTH_8U, 1);
	
	int nCount = 0;
	// �˲�������
	FILE * pfRecordPointFilterOut = fopen(strPath + "RecordPointFilterOut.txt", "w+");
	// �˲�ǰԭʼ����
	FILE * pfRecordPointRawData = fopen(strPath + "RecordPointRawData.txt", "w+");
	// ��άת��ά
	CvPoint Corner2D;
	T_ABS_POS_IN_BASE tPointAbsCoordInBase;	
	XiLineParamNode tFrontLine,tBackLine;
	// �˲������������
	CvPoint3D64f cp3DOutPoint;
	CvPoint3D64f cp3DInPoint = cvPoint3D64f(0.0, 0.0, 0.0);
	// �˲���Ӳ���ֵ
	double dInitComp = m_pTraceModel->m_dGunToEyeCompenX;// ����ڼ�
	double dAbjustHeight = m_pTraceModel->m_dGunToEyeCompenZ;// ���ϲ��� ���²���
	pRobotCtrl->m_cLog->Write("dInitComp:%lf dAbjustHeight:%lf", dInitComp, dAbjustHeight);
	// ��е�۳�ʼλ��
	T_ROBOT_COORS tWeldLinePointPos;
	T_ROBOT_COORS tRobotStartCoord = pRobotCtrl->GetCurrentPos();
	double dAdvenceGunRzAfter = tRobotStartCoord.dRZ/*- pRobotCtrl->m_dRecordBreakPointRZ*/;
	//��е�۳�ʼ��Yֵ
#ifdef SINGLE_ROBOT
	double dStartRobotY = tRobotStartCoord.dY;
#else
	double dStartRobotY = tRobotStartCoord.dX;
#endif	
	// ��ʼ�ⲿ������
	double dStartMachinePos = m_ptUnit->GetExPositionDis(m_ptUnit->m_nTrackAxisNo);
	// ��ʼ��������
	double dStartBaseAxisCoorY = dStartMachinePos + dStartRobotY;

	pRobotCtrl->m_cLog->Write("dAdvenceGunRzBefor:%lf,dStartMachinePos:%.3lf,dStartRobotY:%.3lf",
		dAdvenceGunRzAfter, dStartMachinePos, dStartRobotY);

	// ���ڼ�¼����һ������
	double dLastMachinePos = dStartMachinePos;
	double dLastPosRobotX = tRobotStartCoord.dX;
	double dLastPosRobotY = tRobotStartCoord.dY;
	double dLastPosRobotZ = tRobotStartCoord.dZ;
	//��ʼ������
	int nvSize = pRobotCtrl->m_vtWeldLineInWorldPoints.size();
	if (nvSize<2)
	{
		XUI::MesBox::PopOkCancel("���������ʼ��������������{0} ", nvSize);
		return;
	}
	double dScanDis = TwoPointDis(pRobotCtrl->m_vtWeldLineInWorldPoints.at(0).dX, pRobotCtrl->m_vtWeldLineInWorldPoints.at(0).dY,
		pRobotCtrl->m_vtWeldLineInWorldPoints.at(nvSize - 1).dX, pRobotCtrl->m_vtWeldLineInWorldPoints.at(nvSize - 1).dY);

	// �ؼ��㣬Zֵ������Rz�������˲�ʧ�ܣ���¼����
	int iFindImageKeyPointFailNum = 0;
	int nAbjustZValErrorNum = 0, nAdjustRzErrorNum = 0;
	int nTrackDataEmptyNum = 0;
	// �Ƕȵ�����ֵ
	double dAdjustMaxInterval = 2.0;
	double dAdjustMinInterval = 0.1;

	double dPreRzChange = m_pTraceModel->m_dRecordBreakPointRZ;
	m_pTraceModel->m_dAdjustRobotRZ = m_pTraceModel->m_dRecordBreakPointRZ;
    double dInitRz = dAdvenceGunRzAfter; // ������㴦Rz
	if (m_ptUnit->m_bBreakPointContinue
		&&E_WRAPANGLE_ONCE != m_pTraceModel->m_eStartWrapAngleType)
	{
		dInitRz =pRobotCtrl->m_vtWeldLineInWorldPoints[0].dRZ; // ������㴦Rz
	}
	double dDefaultRz = dInitRz;
	dDefaultRz = fmod(dDefaultRz, 360.0);
	if (0.0 > dDefaultRz)
	{
		dDefaultRz += 360.0; // 0 - 360
	}
	double dDiff = 300;//������β�ж�
	if (E_CLOSEDARC == m_pTraceModel->m_tWorkPieceType)
	{
		dDiff = m_pTraceModel->m_dWeldLen - 2000;//�պ�Բ����ɨ���Ȳ��ɿ�
		if (dDiff < 500)
		{
			dDiff = 500;
		}
	}
	if (E_WRAPANGLE_JUMP == m_pTraceModel->m_eStartWrapAngleType)
	{
		dDiff = m_pTraceModel->m_dWeldLen - 100;//��ǹ���溸�ӽ϶�
	}
	// һ�ΰ���ʱ����ǰ��֪�յ㣿����
	if (E_WRAPANGLE_ONCE == m_pTraceModel->m_eStartWrapAngleType && !m_pTraceModel->m_bIsTrackingScanEnd)
	{
		dDiff = m_pTraceModel->m_dWeldLen - m_dHandEyeDis / 2;
	}
	pRobotCtrl->m_cLog->Write("��ʼRZ��%.3lf %.3lf dDiff��%.3lf", dInitRz, dDefaultRz, dDiff);
	std::vector<double> vdTrackOrgRz;
	std::vector<TrackFilter_Node> CoutTrackPoint;
	vdTrackOrgRz.clear();
	int Errorstop = 0;

	// ��¼����ǰ��������������������
	std::vector<T_ROBOT_COORS> m_vtRecordWarpBeforWeldData;
	m_vtRecordWarpBeforWeldData.clear();

	// ��ʼ����ͼ״̬
	CString strSavePath = "";
	strSavePath.Format("%s%s\\Track\\", OUTPUT_PATH, pRobotCtrl->m_strRobotName);
	//DeleteFolder(strSavePath);
	T_SAVE_IMAGE tSaveImage;
	for (int nS = 0; nS < SAVE_IMAGE_THREAD_MUN; nS++)
	{
		tSaveImage.bSaveImageStaus[nS] = false;
	}
	tSaveImage.nImageNo = 0;

	if (m_pTraceModel->m_dCurrentWeldLen < 0)
	{
		m_pTraceModel->m_dCurrentWeldLen = 0;
	}
	// ����β��ʣ�����۳���
	double overlength = 9999.0;
	bool bCompelOver = false;// ǿ�ƽ�������
	// �˲����Ӵ��ſ��ж�����
	TrackFilter_NextPushPntLoose(m_ptUnit->m_nRobotSmooth);
	while (FALSE == m_pTraceModel->m_bCameraFindWeldEnd)//�ж��ҵ���β���ٲɵ�0
	{
		if (pRobotCtrl->m_eThreadStatus == INCISEHEAD_THREAD_STATUS_STOPPED)
		{
			pRobotCtrl->m_cLog->Write("��ͣ���������̣߳� %d", pRobotCtrl->m_eThreadStatus);
			break;
		}
		if (m_pTraceModel->m_bIsCloseTrackThread)//��ǰ֪����β���ж�
		{
			//�����β����
			GetEndSectionCoors(pRobotCtrl, m_pTraceModel->m_tRealEndpointCoor);
			if (E_WRAPANGLE_EMPTY_SINGLE == m_pTraceModel->m_eStartWrapAngleType
				|| E_WRAPANGLE_EMPTY_DOUBLE == m_pTraceModel->m_eStartWrapAngleType)
			{
				int nTotal = 2;
				pRobotCtrl->SetIntVar(nTotal, pRobotCtrl->m_vtWeldLineInWorldPoints.size() - 1);
			}
			else
			{
				// ���ٽ������Ͱ�������
				m_pTraceModel->m_bIsTrackingDataSendOver = true;
			}
			//m_pTraceModel->m_bIsTrackingDataSendOver = true;// ��ʱ�����ó�
			m_pTraceModel->m_bCameraFindWeldEnd = TRUE;
			pRobotCtrl->m_cLog->Write("��β�������������̣߳�%d ", m_pTraceModel->m_bIsCloseTrackThread);
			break;
		}

		if (FALSE == m_pTraceModel->m_bCameraFindWeldEnd)
		{
			nvSize = pRobotCtrl->m_vtWeldLineInWorldPoints.size();
			CoutTrackPoint.clear();
			long long lStartTime = XI_clock();
			long long lCurTime = XI_clock();
			//��ȡ��ǰλ��
//			T_CART_COOR dMachineCoors;
			long long lsTime = XI_clock();
			T_ROBOT_COORS tRobotCurCoord = pRobotCtrl->GetCurrentPos();
			T_ANGLE_PULSE tRobotCurPulses = pRobotCtrl->GetCurrentPulse();
			double dMachineY = m_ptUnit->GetExPositionDis(m_ptUnit->m_nTrackAxisNo);
#ifdef SINGLE_ROBOT
			double dBaseAxisCoorY = dMachineY + tRobotCurCoord.dY;
#else
			double dBaseAxisCoorY = dMachineY + tRobotCurCoord.dX;
#endif	
			//cvCopyImage(pDHCam->CaptureImage(FALSE), pImageBuff);
			if (!pDHCam->CaptureImage(pImageBuff, 1))
			{
				//pDHCam->ShowErrorString();
			}

			long long leTime = XI_clock();
			//��ͼ
			lCurTime = XI_clock();
			//Corner2D = pImageProcess->GetGroupStandKeyPoint
			//(
			//	pImageBuff, tFrontLine, tBackLine, FALSE, TRUE, TRUE
			//);
			WeightedPointCluster(pImageBuff, &m_pTraceModel->tFrontLine, &m_pTraceModel->tBackLine, &Corner2D, false, "WeightedPointCluster");
			
			pRobotCtrl->m_cLog->Write("%d �����ҵ�ʱ��:%.3lf %d %d", tSaveImage.nImageNo,(double)(XI_clock() - lCurTime) / CLOCKS_PER_SEC, Corner2D.x, Corner2D.y);
			lCurTime = XI_clock();
			tWeldLinePointPos = m_ptUnit->TranImageToBase(m_ptUnit->m_nTrackCameraNo, Corner2D, tRobotCurCoord, tRobotCurPulses, &tPointAbsCoordInBase);

			if (Corner2D.x > 1 && Corner2D.x < 2591 && Corner2D.y>1 && Corner2D.y < 1993)
			{
				cvCvtColor(pImageBuff, m_pShowImage, CV_GRAY2RGB);
				cvCircle(m_pShowImage, Corner2D, 8, CV_RGB(255, 0, 0), 4);
			}
			CString strImgPathName = "";
			strImgPathName.Format("%s%d.jpg", strSavePath, tSaveImage.nImageNo);
			tSaveImage.nImageNo++;
			tSaveImage.cstrId = strImgPathName;
			tSaveImage.pImage = pImageBuff;
			m_pTraceModel->m_bTrackSaveImage = true;
			if (m_pTraceModel->m_bTrackSaveImage)
			{
				AfxBeginThread(ThreadSaveImage, &tSaveImage);
			}
			//Ŀ���Y����������λ��			
#ifdef SINGLE_ROBOT
			double dBaseAxisAbsCoorY = dMachineY + tWeldLinePointPos.dY;
#else
			double dBaseAxisAbsCoorY = dMachineY + tWeldLinePointPos.dX;
#endif	
			pRobotCtrl->m_cLog->Write("%s ������Ŀ��㣺%.3lf %.3lf %.3lf ��ͼʱ��λ�ã�%.3lf �������Y����������λ�ã�%.3lf ,nvSize:%d %d",
				strRobot, tWeldLinePointPos.dX, tWeldLinePointPos.dY, tWeldLinePointPos.dZ,
				dMachineY, dBaseAxisAbsCoorY, nvSize, tSaveImage.nImageNo);
			lCurTime = XI_clock();
			if (1 > Corner2D.x || 1 > Corner2D.y || 2448 < Corner2D.x || 2048 < Corner2D.y)
			{
				pRobotCtrl->m_cLog->Write("%s kong Corner2D.x:%d Corner2D.y:%d", strRobot, Corner2D.x, Corner2D.y);
				iFindImageKeyPointFailNum++;
				if (1 < iFindImageKeyPointFailNum)
				{
					// ���ٵ���PosMove֮ǰ�жϣ�����Ѿ������յ�˴β�����(�������յ�ɨ���������̬��ͻ)
					if (TRUE == m_pTraceModel->m_bCameraFindWeldEnd)
					{
						continue;
					}
				}
				nCount++;
				
				if (iFindImageKeyPointFailNum > 30)
				{
					pRobotCtrl->m_cLog->Write("%s ����8���ҵ���ֵ������ͣ����", strRobot);
					CheckMachineEmg(pRobotCtrl);
					XUI::MesBox::PopOkCancel("{0}�Ż���������8���ҵ���ֵ������ͣ����", pRobotCtrl->m_nRobotNo);
					break;
				}
				if (iFindImageKeyPointFailNum > 20 && overlength < 30)// ǿ�ƽ���
				{
					bCompelOver = true;
					pRobotCtrl->m_cLog->Write("%s ǿ�ƽ�������", strRobot);
				}
				else
				{
					continue;
				}
			}
			else
			{
				iFindImageKeyPointFailNum = 0; // ��ά����Чʱ ���ñ�־

			}
			long long time1 = XI_clock();

			if (nvSize > 0)
			{
				cp3DInPoint.x = tWeldLinePointPos.dX;
				cp3DInPoint.y = tWeldLinePointPos.dY;
				cp3DInPoint.z = tWeldLinePointPos.dZ;
				double dLatestDistance = 0.0;
				if (m_pTraceModel->m_bIfOpenExternal)
				{
#ifdef SINGLE_ROBOT
					cp3DInPoint.y = dBaseAxisAbsCoorY;
					dLatestDistance = sqrt(pow(tRobotCurCoord.dX - dLastPosRobotX, 2) + pow(dBaseAxisCoorY - dStartBaseAxisCoorY, 2) + pow(tRobotCurCoord.dZ - dLastPosRobotZ, 2));
#else
					cp3DInPoint.x = dBaseAxisAbsCoorY;
					dLatestDistance = sqrt(pow(dBaseAxisCoorY - (dLastMachinePos + dLastPosRobotX)/*dStartBaseAxisCoorY*/, 2) +
						pow(tRobotCurCoord.dY - dLastPosRobotY, 2) +  pow(tRobotCurCoord.dZ - dLastPosRobotZ, 2));
#endif
				}
				else
				{
					cp3DInPoint.y = tWeldLinePointPos.dY;
					dLatestDistance = sqrt(pow(tRobotCurCoord.dX - dLastPosRobotX, 2) + pow(tRobotCurCoord.dY - dLastPosRobotY, 2) + pow(tRobotCurCoord.dZ - dLastPosRobotZ, 2));
				}
				if (dLatestDistance < 2.0)
				{
					pRobotCtrl->m_cLog->Write("%s �����������ϴξ���С��2mm,�����д���:%lf", strRobot, dLatestDistance);
					Errorstop++;
					if (Errorstop >= 30)
					{
						CheckMachineEmg(pRobotCtrl);
						//���޸�
						XUI::MesBox::PopInfo("{0}�Ż����˴�ͣ��!��Ϣ��!�ɶϵ�����", pRobotCtrl->m_nRobotNo);
						//XiMessageBox("%d�Ż����˴�ͣ��!��Ϣ��!�ɶϵ�����", pRobotCtrl->m_nRobotNo);
					}
					continue;
				}
				else
				{
					//���ϴεĴ�λ�ø��м�ֵ
					dLastMachinePos = dMachineY;
					dStartBaseAxisCoorY = dBaseAxisCoorY;
					dLastPosRobotX = tRobotCurCoord.dX;
					dLastPosRobotY = tRobotCurCoord.dY;
					dLastPosRobotZ = tRobotCurCoord.dZ;
				}
				Errorstop = 0;
				m_pTraceModel->m_dCurrentWeldLen += dLatestDistance;
				pRobotCtrl->m_cLog->Write("%s %d �����ܳ��ȣ�%11.3lf,��ǰ���ӳ���:%11.3lf��dLatestDistance:%lf �ⲿ���ƶ���%lf",
					strRobot, nCount, m_pTraceModel->m_dWeldLen, m_pTraceModel->m_dCurrentWeldLen, dLatestDistance, dStartBaseAxisCoorY);
				
				if (TRUE == m_pTraceModel->m_bCameraFindWeldEnd)
				{
					pRobotCtrl->m_cLog->Write("%s ��ǹ�����Ѿ������յ㣬���ٸ���ͼ����˲��������", strRobot);
					continue;
				}
				lCurTime = XI_clock();
				//�˲���
				pRobotCtrl->m_cLog->Write("%s �˲�ǰ�������:, %11.3lf %11.3lf %11.3lf", 
					strRobot, cp3DInPoint.x, cp3DInPoint.y, cp3DInPoint.z);
				bool bNormalDir = false;
				if (-1 == pRobotCtrl->m_nRobotInstallDir)
				{
					bNormalDir = TRUE;
				}
				fprintf(pfRecordPointRawData, "�˲�ǰ�������: seq: %d, %11.3lf %11.3lf %11.3lf \n", nCount, cp3DInPoint.x, cp3DInPoint.y, cp3DInPoint.z);
				fflush(pfRecordPointRawData);

				CoutTrackPoint = TrackFilter_FilterCurvePointInPointOut(m_ptUnit->m_nRobotSmooth, cp3DInPoint.x, cp3DInPoint.y, cp3DInPoint.z, bNormalDir);

				//! �ж��Ƿ���������ƶ�����, return true : ����ƶ�, ����δ���.- overlength : �ṩһ��doubleָ��, ��ȡʣ�೤��.
				bool bIsOver = false;
				//bIsOver = TrackFilter_DoneMove(m_ptUnit->m_nRobotSmooth, &overlength);
				if (bIsOver|| bCompelOver)
				{
					// ������ֽ�β��
					m_pTraceModel->m_bCameraFindWeldEnd = TRUE;
					CoutTrackPoint.clear();
					//! �� TrackFilter_DoneMove() == trueʱ�����øú�������ȡֱ�����۳��Ƚ��� ʱ�Ĺ켣�㼯.
					//!  ���� overlength �㹻Сʱ, ���øú���, ǿ�ƻ�ȡ����β��. ��ʱ��ȡ�Ľ�β��Ϊֱ���ӳ�.
					TrackFilter_Answer ans = TrackFilter_GetToEndPnts(m_ptUnit->m_nRobotSmooth, bNormalDir);

					size_t index = 0;
					for (index = 0; index < ans.araaySize_; ++index) {
						CoutTrackPoint.push_back(ans.segmentArray_[index]);
					}
					TrackFilter_Free(ans);
					pRobotCtrl->m_cLog->Write("�������ݴﵽ��β�㣺%d  ", CoutTrackPoint.size());
				}
				pRobotCtrl->m_cLog->Write("�Ƿ񵽴��β���۳��ȣ�%d ���β���۳��ȣ�%lf ", bIsOver, overlength);
				double dTrackVerDegree;
				if (nTrackDataEmptyNum > 30)
				{
					CheckMachineEmg(pRobotCtrl);
					pRobotCtrl->m_cLog->Write("%s CoutTrackPoint����Ϊ�մ������࣬һ���ⲻ̫������ͣ����������̧ǹ���������", strRobot);
					XiMessageBox("CoutTrackPoint����Ϊ�մ������࣬һ���ⲻ̫������ͣ����������̧ǹ���������");
				}
				if (CoutTrackPoint.size() < 1)
				{
					pRobotCtrl->m_cLog->Write("%s CoutTrackPoint����Ϊ��", strRobot);
					nTrackDataEmptyNum++;
					continue;
				}
				nTrackDataEmptyNum = 0;
				for (int nSize = 0; nSize < CoutTrackPoint.size(); nSize++)
				{
					// ��ʼ�β���
					dTrackVerDegree = atan2(CoutTrackPoint[nSize].normalDir_.y_, CoutTrackPoint[nSize].normalDir_.x_) * 180.0 / PI;
					// ��¼�˲��󲹳�ǰ��������������������
					m_pTraceModel->m_vtRecordWarpBeforWeldData.push_back(CoutTrackPoint[nSize].pnt_);

					cp3DOutPoint.x = CoutTrackPoint[nSize].pnt_.x_ + (dInitComp * CosD(dTrackVerDegree));
					cp3DOutPoint.y = CoutTrackPoint[nSize].pnt_.y_ + (dInitComp * SinD(dTrackVerDegree));
					cp3DOutPoint.z = CoutTrackPoint[nSize].pnt_.z_ + (dAbjustHeight);
					pRobotCtrl->m_cLog->Write(" %s �˲��󲹳���  %.3lf  %.3lf  %.3lf dTrackVerDegree:%lf %lf %lf  ",
						strRobot, cp3DOutPoint.x, cp3DOutPoint.y, cp3DOutPoint.z, dTrackVerDegree,
						CoutTrackPoint[nSize].normalDir_.y_, CoutTrackPoint[nSize].normalDir_.x_);
					fprintf(pfRecordPointFilterOut, "�˲�������: seq: %d, %11.3lf %11.3lf %11.3lf\n", nCount,
						cp3DOutPoint.x, cp3DOutPoint.y, cp3DOutPoint.z);
					fflush(pfRecordPointFilterOut);
					// �ж����µ��˲��������, �Ƿ����
					int nS = pRobotCtrl->m_vtWeldLineInWorldPoints.size();
					if (nS < 2)
					{
						XiMessageBox("m_vtWeldLineInWorldPoints�����ڴ������⣬����");
						continue;
					}

					dTrackVerDegree = SmoothRz(dTrackVerDegree, vdTrackOrgRz);//���˷���
					
					// ����ǰ�����������������Rz
					double dTestRz = DirAngleToRz(dTrackVerDegree);//���������Rz  ����ʵʱ�仯��ǰ�������()��U��Ƕȼ���Rz 																						  // �ٽ��������ԭʼdTestRzȡƽ��ֵ��ʹ					 

					if (dTestRz > 180.0) {
						dTestRz -= 360.0;
					}
					else if (dTestRz < -180.0) {
						dTestRz += 360.0;
					}
					pRobotCtrl->m_cLog->Write("����Ǽ���RZ:%lf", dTestRz);
					double dTargetPostion[6] = { 0 };
					dTargetPostion[0] = cp3DOutPoint.x;
					dTargetPostion[1] = cp3DOutPoint.y;
					double dHeight = pRobotCtrl->m_vtWeldLineInWorldPoints[nS - 1].dZ - cp3DOutPoint.z;
					if ((dHeight > 4) || (dHeight < -4)) // �����˸߶ȱ仯��ʱ���޸�
					{
						pRobotCtrl->m_cLog->Write("Zֵ������ԭʼ���ݣ�%lf", dHeight);
						dHeight = dHeight > 0 ? 2 : -2;

						dTargetPostion[2] = pRobotCtrl->GetCurrentPos(ROBOT_AXIS_Z) - dHeight;
						nAbjustZValErrorNum++;
						pRobotCtrl->m_cLog->Write("%s �����߶�dHeight:%lf nAbjustZValErrorNum:%d", strRobot, dHeight, nAbjustZValErrorNum);
						if (nAbjustZValErrorNum > 13)
						{
							CheckMachineEmg(pRobotCtrl);
							pRobotCtrl->m_cLog->Write("%s zֵ��������������࣬ͣ����������̧ǹ���������", strRobot);
							XiMessageBox("zֵ��������������࣬ͣ����������̧ǹ���������");
							break;
						}
					}
					else
					{
						dTargetPostion[2] = cp3DOutPoint.z;
						nAbjustZValErrorNum = 0;
					}

					lCurTime = XI_clock();
					dTargetPostion[3] = tRobotStartCoord.dRX;//pRobotCtrl->GetCurrentPos(ROBOT_AXIS_RX);
					dTargetPostion[4] = tRobotStartCoord.dRY;// pRobotCtrl->GetCurrentPos(ROBOT_AXIS_RY);
					double dCurRz = pRobotCtrl->GetCurrentPos(ROBOT_AXIS_RZ);
					dTestRz = fmod(dTestRz, 360.0);
					if (0.0 > dTestRz) {
						dTestRz += 360.0; // 0 - 360
					}
					double dRzChange = dTestRz - dDefaultRz;
					dRzChange = fmod(dRzChange, 360.0);
					if (-180 > dRzChange)//��ֹ����dTestRz��360��0
					{
						dRzChange += 360.0; // 0 - 360
					}
					else if (180 < dRzChange)
					{
						dRzChange -= 360.0;
					}
					//RZǰ���ֵ
					double dRzBefAndAftDiff = dRzChange - dPreRzChange;
					if (dRzBefAndAftDiff > 180)
					{
						dRzBefAndAftDiff -= 360;
					}
					else if (dRzBefAndAftDiff < -180)
					{
						dRzBefAndAftDiff += 360;
					}

					pRobotCtrl->m_cLog->Write("%s �������dTestRz: %lf RZ��ֵ: %lf dRzChange %lf dPreRzChange:%lf",
						strRobot, dTestRz, dRzBefAndAftDiff, dRzChange, dPreRzChange);
					if ((fabs(dRzBefAndAftDiff) > dAdjustMaxInterval)) // ��̬�ǶȲ������仯ֵ����
					{
						double dDir = dRzBefAndAftDiff > 0.0 ? 1.0 : -1.0;
						dRzChange = dPreRzChange + (dAdjustMaxInterval * dDir);
						nAdjustRzErrorNum++;
						if (nAdjustRzErrorNum > 10)
						{
							CheckMachineEmg(pRobotCtrl);
							pRobotCtrl->m_cLog->Write("%s Rzֵ��������������࣬ͣ����������̧ǹ���������", strRobot);
							XiMessageBox("Rzֵ��������������࣬ͣ����������̧ǹ���������");
							break;
						}
					}
					else
					{
						nAdjustRzErrorNum = 0;
					}
					if ((fabs(dRzBefAndAftDiff) < dAdjustMinInterval)) // ��̬�ǶȲ�����С�仯ֵ����
					{
						dRzChange = dPreRzChange;
					}
					dPreRzChange = dRzChange; // ��¼��һ��Rz���� P99
					dTargetPostion[5] = /*dAdvenceGunRzAfter*/dInitRz; // ���ٹ켣Rz���䣬ͨ��ʵʱRz�����Rz��ֵ��Ϊ�������е���
					double dCoor[6] = { 0 };
					dCoor[5] = dRzChange;
					m_pTraceModel->m_dAdjustRobotRZ = dRzChange;
					pRobotCtrl->m_cLog->Write("%s �����������ݣ�dRzChange:%lf %lf %lf %lf %lf %lf %lf ", strRobot, dRzChange, dTargetPostion[0], dTargetPostion[1], dTargetPostion[2], dTargetPostion[3], dTargetPostion[4], dTargetPostion[5]);
					BOOL bIfWindowOn = TRUE;
					//SetDlgItemData(IDC_STATIC_OPERATE_HINT, this, bIfWindowOn, "%s ����λ��RZ��%lf��Z:%lf", pRobotCtrl->m_strRobotName, dTargetPostion[5], dTargetPostion[2]);
					T_ROBOT_COORS tRealWeldPoint;
					if (m_pTraceModel->m_bIfOpenExternal)
					{

#ifdef SINGLE_ROBOT
						T_ROBOT_COORS tMeasurePoint(
							dTargetPostion[0], dStartRobotY, dTargetPostion[2],
							dTargetPostion[3], dTargetPostion[4], dTargetPostion[5],
							0, dTargetPostion[1] - dStartRobotY, 0);
						tRealWeldPoint = tMeasurePoint;
#else
						T_ROBOT_COORS tMeasurePoint(
							dStartRobotY, dTargetPostion[1], dTargetPostion[2],
							dTargetPostion[3], dTargetPostion[4], dTargetPostion[5],
							dTargetPostion[0] - dStartRobotY, 0, 0);
						tRealWeldPoint = tMeasurePoint;
#endif						
					}
					else
					{
						T_ROBOT_COORS tMeasurePoint(
							dTargetPostion[0], dTargetPostion[1], dTargetPostion[2],
							dTargetPostion[3], dTargetPostion[4], dTargetPostion[5], 0, dMachineY, 0);
						tRealWeldPoint = tMeasurePoint;
					}
					pRobotCtrl->m_vtWeldLineInWorldPoints.push_back(tRealWeldPoint);
					m_pTraceModel->m_vtWeldLinePointType.push_back(E_WELD_TRACK);
                    SaveRobotCoor(m_pRecordTheoryPoint, tRealWeldPoint, E_WELD_TRACK);//������������
				}

			    // ��ȡ�µĽ�β����
				if (bIsOver)
				{					
					// ���ͽ�β���ݵ��������ڽ�����������
					int nOverSize = pRobotCtrl->m_vtWeldLineInWorldPoints.size();
					//pRobotCtrl->SetIntVar(2, pRobotCtrl->m_vtWeldLineInWorldPoints.size()-1);
					T_ROBOT_COORS tEndpoint = pRobotCtrl->m_vtWeldLineInWorldPoints.at(pRobotCtrl->m_vtWeldLineInWorldPoints.size() - 1);
					pRobotCtrl->m_cLog->Write("ԭʼ��β���� %.3lf %.3lf %.3lf %.3lf�����º��β���ݣ�%.3lf %.3lf %.3lf %.3lf", 
						m_pTraceModel->m_tRealEndpointCoor.dX, m_pTraceModel->m_tRealEndpointCoor.dY, m_pTraceModel->m_tRealEndpointCoor.dZ,
						m_pTraceModel->m_tRealEndpointCoor.dBY,
						tEndpoint.dX, tEndpoint.dY, tEndpoint.dZ,tEndpoint.dBY);
					m_pTraceModel->m_tRealEndpointCoor = tEndpoint;

					if (m_pTraceModel->m_nCloseTrackingPos > 0)
					{
						double DAngle = 45.0;
						double dStepDis = DAngle / m_pTraceModel->m_nCloseTrackingPos;						
						for (int n = m_pTraceModel->m_nCloseTrackingPos; n > 0; n--)
						{
							pRobotCtrl->m_vtWeldLineInWorldPoints.at(nOverSize - (m_pTraceModel->m_nCloseTrackingPos - n) - 1).dRZ -= dStepDis * n * pRobotCtrl->m_nRobotInstallDir;
							pRobotCtrl->m_cLog->Write("��β�仯��̬��%lf %lf",
								pRobotCtrl->m_vtWeldLineInWorldPoints.at(nOverSize - (m_pTraceModel->m_nCloseTrackingPos - n) - 1).dRZ, dStepDis);
						}
					}
					m_pTraceModel->m_bIsCloseTrackThread = true;
					m_pTraceModel->m_bIsTrackingDataSendOver = true;

				}

				// �յ��ж�  m_dDisSafeGunToEnd + 4 ʱ�յ��Ԥ������ 4�ǵ��յ��ж���ֵ
#ifdef SINGLE_ROBOT
				pRobotCtrl->m_cLog->Write("%s �������ȣ�%.3lf  %.3lf %.3lf  �켣size:%d", strRobot, m_pTraceModel->m_dWeldLen, m_pTraceModel->m_dCurrentWeldLen, dDiff,
					pRobotCtrl->m_vtWeldLineInWorldPoints.size());
#else
				pRobotCtrl->m_cLog->Write("%s �������ȣ�%.3lf  %.3lf %.3lf  �켣size:%d", strRobot, m_pTraceModel->m_dWeldLen, m_pTraceModel->m_dCurrentWeldLen, dDiff,
					pRobotCtrl->m_vtWeldLineInWorldPoints.size());

#endif // SINGLE_ROBOT

				if (m_pTraceModel->m_dCurrentWeldLen > (m_pTraceModel->m_dWeldLen - dDiff) &&
					m_pTraceModel->m_dCurrentWeldLen > 150.0 &&
					FALSE == m_pTraceModel->m_bIfJudgeEndOpen)//������β�ж�
				{
					m_pTraceModel->m_bIfJudgeEndOpen = TRUE;
					if (m_pTraceModel->m_bIsTrackingScanEnd)//������β
					{
						pRobotCtrl->m_cLog->Write("%s �����յ��ж�", strRobot);						
						pImageProcess->InitIfStartOrEndPntParam(GROUP_STAND_DIP_NS::E_PIECE_END); Sleep(100);
						Corner2D = pImageProcess->GetGroupStandKeyPoint(pImageBuff, tFrontLine, tBackLine, FALSE, TRUE, TRUE);
						pRobotCtrl->m_cLog->Write("%s ��ʼ���յ� :%d %d", strRobot, Corner2D.x, Corner2D.y);
						m_nEndPointCapCursor = 0;
						m_nEndPointProcCursor = 0;
						m_pTraceModel->m_bCameraFindWeldEnd = FALSE;

						AfxBeginThread(ThreadJudgeEndCaptureRobot, this);
						AfxBeginThread(ThreadJudgeEndProcLineGroupStand, this);
					}
					else//��֪��β�ж�						
					{
						pRobotCtrl->m_cLog->Write("��ʼ��β�ж� ��֪��β�ж�");
						CallJudgeEndpointFun();
					}

					pRobotCtrl->m_cLog->Write("��ʼ��β�ж�");
				}
				nCount++;
			}
			else
			{
				pRobotCtrl->m_cLog->Write("%s 00�ɵ�����size:%d,count:%d %11.3lf%11.3lf%11.3lf", strRobot, nvSize, nCount, tWeldLinePointPos.dX, tWeldLinePointPos.dY, tWeldLinePointPos.dZ);
			}
		}
		Sleep(20);
		DoEvent();
	}
	fclose(pfRecordPointFilterOut);
	fclose(pfRecordPointRawData);
	//�������̽����رռ���
	m_ptUnit->SwitchIO("TrackLaser", false);

	// $$$$ �ȴ���ͼ�߳��˳�
	// �洢�����Ƿ������־
	SaveIsTrackCompleteFlag(pRobotCtrl, m_pTraceModel->m_bCameraFindWeldEnd, overlength);
	pRobotCtrl->m_cLog->Write("%s ��ʼ����ͼ�̸߳���", pRobotCtrl->m_strRobotName);
	while (true)
	{
		//CHECK_ROBOT_EMG_BREAK(pRobotCtrl);
		int nThreathNum = 0;
		for (int ns = 0; ns < SAVE_IMAGE_THREAD_MUN; ns++)
		{
			if (FALSE == tSaveImage.bSaveImageStaus[ns])
			{
				nThreathNum++;
			}
		}
		if (SAVE_IMAGE_THREAD_MUN == nThreathNum)
		{
			pRobotCtrl->m_cLog->Write("%s ��ͼ�߳̿��и���", pRobotCtrl->m_strRobotName);
			break;
		}
		DoEvent();
		Sleep(10);
	}
	pRobotCtrl->m_cLog->Write("%s �����߳̽���", strRobot);
#endif // 0
}

UINT CScanInitModule::ThreadRealTimeTrackCapture_H(void* pParam)
{
	CScanInitModule* pObj = (CScanInitModule*)pParam;
	pObj->FuncRealTimeTrackCapture_H(pObj->m_pRobotDriver, pObj->m_ptUnit->m_nTrackCameraNo, 6, 50);
	return 0;
}

void CScanInitModule::FuncRealTimeTrackCapture_H(CRobotDriverAdaptor* pRobotDriver, int nCameraNo, int nProcThreadNum, int nMaxImgBufSize/* = 50*/, int nTimeOut/* = 2000*/)
{
	long lStartTime = 0;
	int nCapImageNo = 0;
	int nSaveBufNo = 0;
	bool bCapSuc = false;
	int nImgStatus = 0;
	T_ROBOT_COORS tRobotCoord;
	T_ANGLE_PULSE tRobotPulse;
	std::vector<CWinThread*> vpWinThread(0);
	CDHGigeImageCapture* pDHCamDrv = (CDHGigeImageCapture*)m_ptUnit->GetCameraCtrl(nCameraNo);
	IplImage* pImageBuf = pDHCamDrv->m_pImageBuff;
	T_CAMREA_PARAM tCameraParam = m_ptUnit->GetCameraParam(nCameraNo);

	m_pTraceModel->lnImageNo.clear();	// ��ǰ���Ƽ�����ÿ��ͼ���
	m_pTraceModel->lnImagePtnNum.clear();	// ��ǰ���Ƽ�����ÿ��ͼ����
	m_pTraceModel->ltPointCloud.clear(); // ��ǰ���Ƽ���
	m_pTraceModel->ltCapWorldCoord.clear(); // ��ǰ���Ƽ�����ÿ��ͼ�ɼ�ʱ��������
	m_pTraceModel->vtPointCloudEndPoints.clear(); // ÿ�ε��ƴ�����ȡ�Ķ˵���������
	m_pTraceModel->vtConnerWorldCoords.clear();
	m_pTraceModel->m_bCameraFindWeldEnd = false;
	m_pTraceModel->pfPointCloud = fopen(OUTPUT_PATH + m_pRobotDriver->m_strRobotName + WELDDATA_PATH + "RecordPointCloud.txt", "w");
	m_pTraceModel->pfImgCalcData = fopen(OUTPUT_PATH + m_pRobotDriver->m_strRobotName + WELDDATA_PATH + "RecordCapImgCoord.txt", "w");

	m_pCapImageData = new T_CAP_IMAGE_DATA(nMaxImgBufSize); // �������ݻ�����

	m_IsOpenImageProcess = true;
	for (int i = 0; i < nProcThreadNum; i++) // ����ͼƬ�����߳�
	{
		CWinThread* tProcImgThread = AfxBeginThread(ThreadImageProcess, this);
		tProcImgThread->m_bAutoDelete = false;
		vpWinThread.push_back(tProcImgThread);

	}
	CWinThread* tProcPtnCldThread = AfxBeginThread(ThreadJudgeEnd_ProcessSearch, this); // �������ƴ����߳�
	tProcPtnCldThread->m_bAutoDelete = false;
	vpWinThread.push_back(tProcPtnCldThread);

	int nCurWaitTime = 0;
	int nWaitTimeStep = 100;
	bool bIsRunning = m_ptUnit->WorldIsRunning();
	while (!m_ptUnit->WorldIsRunning()) // �ȴ��˶�
	{
		Sleep(nWaitTimeStep);
		nCurWaitTime += nWaitTimeStep;
		if (nCurWaitTime > nTimeOut)
		{
			DELETE_POINTER(m_pCapImageData);
			pRobotDriver->m_cLog->Write("[�����Ѷ˵�]: �ȴ��˶���ʱ��");
			return;
		}
	}

	// �����˶��� �� δ�����յ�
	while (m_ptUnit->WorldIsRunning() && true == m_IsOpenJudgeEnd_ProcessSearch)
	{
		lStartTime = GetTickCount();

		// ������
		tRobotCoord = pRobotDriver->GetCurrentPos();
		tRobotPulse = pRobotDriver->GetCurrentPulse();

		// ��ͼ
		pDHCamDrv->CaptureImage(pImageBuf, 1);

		// ���浽������
		//m_pCapImageData->m_MutexReadWrite.lock();
		nSaveBufNo = m_pCapImageData->nTotalCapImgNum % m_pCapImageData->nMaxBufferSize;
		m_pCapImageData->vtCapCoord[nSaveBufNo] = tRobotCoord;
		m_pCapImageData->vtCapPulse[nSaveBufNo] = tRobotPulse;
		if (NULL == m_pCapImageData->vpImg[nSaveBufNo])
		{
			m_pCapImageData->vpImg[nSaveBufNo] = cvCreateImage(cvSize(tCameraParam.tDHCameraDriverPara.nRoiWidth, tCameraParam.tDHCameraDriverPara.nRoiHeight), IPL_DEPTH_8U, 1);
		}
		cvCopyImage(pImageBuf, m_pCapImageData->vpImg[nSaveBufNo]);
		m_pCapImageData->nTotalCapImgNum = nCapImageNo + 1;
		//m_pCapImageData->m_MutexReadWrite.unlock();

		pRobotDriver->m_cLog->Write("[�����Ѷ˵�]: ɨ���ͼ�ܱ��%d �����%d ��ʱ%dms ��ͼ�ɹ�%d ͼƬ״̬%d",
			nCapImageNo, nSaveBufNo, GetTickCount() - lStartTime, bCapSuc, nImgStatus);
		nCapImageNo++;

	}
	m_IsOpenImageProcess = false;

	long lTime = GetTickCount();
	WaitAndCheckAllThreadExit(vpWinThread);
	pRobotDriver->m_cLog->Write("[�����Ѷ˵�]: WaitAndCheckAllThreadExit ��ʱ%dms", GetTickCount() - lTime);

	// �ͷ� m_pCapImageData
	for (int i = 0; i < m_pCapImageData->nMaxBufferSize; i++)
	{
		if (NULL != m_pCapImageData->vpImg[i])
		{
			cvReleaseImage(&(m_pCapImageData->vpImg[i]));
		}
	}
	DELETE_POINTER(m_pCapImageData);
	fclose(m_pTraceModel->pfPointCloud);
	fclose(m_pTraceModel->pfImgCalcData);
	pRobotDriver->m_cLog->Write("[�����Ѷ˵�]: FuncRealTimeTrackCapture_H ����");
}

void CScanInitModule::RealTimeTrackingNew_H(CRobotDriverAdaptor* pRobotCtrl)
{
	pRobotCtrl->m_cLog->Write("[���ٵ���]: �����߳̿�ʼ");
	// ������ʼ��
	bool bSaveImage = false;// �Ƿ񱣴����ͼƬ
	int nJudgeEndMethod = 3;	// 1.��֪�յ� 2.�����۳��� 3.���ӹ������ж�
	int nImageNo = -1;	// ͼƬ���
	int nMaxImgProFailNum = 20;	// �������ͼ����ʧ�ܴ���
	int nMaxAdjustZErrNum = 5;	// �������Zֵ�仯�������
	int nMaxFilterFailNum = 40;	// ��������˲�ʧ�ܴ���
	int nMaxErrStopNum = 50;	// ����������ֹͣ����
	int nImgProFailNum = 0;	// ����ͼ����ʧ�ܴ���
	int nAdjustZErrNum = 0;	// ����Zֵ�仯�������
	int nFilterFailNum = 0;	// �����˲�ʧ�ܴ���
	int nErrStopNum = 0;	// �������ֹͣ����
	double dMinJoinPtnDis = 0.5;	// ��ͼ��������С���ھ���
	int nConnerRstNum = 0; // ��¼ÿ�λ�ȡ��������ʱ ����Ľǵ�����--
	int nPointCloudRstNum = 0; // ��¼ÿ�λ�ȡ���������� ���ƴ���������--
	T_ROBOT_COORS tWeldLineWorldCoord;	// ʵʱ��������������������� X Y Z

	m_IsOpenJudgeEnd_ProcessSearch = false;
	m_pTraceModel->lnImageNo.clear();
	m_pTraceModel->lnImagePtnNum.clear();
	m_pTraceModel->ltPointCloud.clear();
	m_pTraceModel->ltCapWorldCoord.clear();
	m_pTraceModel->vtPointCloudEndPoints.clear();
	m_pTraceModel->vtConnerWorldCoords.clear();
	m_pTraceModel->m_dCurrentWeldLen = 0.0 > m_pTraceModel->m_dCurrentWeldLen ? 0.0 : m_pTraceModel->m_dCurrentWeldLen;	// ��ǰ�ۼƺ��ӳ��ȳ�ʼ��
	m_pTraceModel->m_dPreRzChange = m_pTraceModel->m_dRecordBreakPointRZ;
	m_pTraceModel->m_dAdjustRobotRZ = m_pTraceModel->m_dRecordBreakPointRZ;
	int nAllTrackNum = pRobotCtrl->m_vtWeldLineInWorldPoints.size();
	if (nAllTrackNum < 2)
	{
		XUI::MesBox::PopOkCancel("[���ٵ���]:���������ʼ��������������{0} ", nAllTrackNum);
		return;
	}

	// ·�� �� �ַ��� ��ʼ��
	CString strRobot = pRobotCtrl->m_strRobotName;
	CString strPath = OUTPUT_PATH + pRobotCtrl->m_strRobotName + WELDDATA_PATH;
	CString strSavePath = OUTPUT_PATH + pRobotCtrl->m_strRobotName + "\\Track\\";
	CString strImgPathName = "";

	// �����ļ���ʼ��
	FILE* pfRawData = fopen(strPath + "RecordPointRawData.txt", "a+");// �˲�ǰԭʼ����
	FILE* pfFilterOut = fopen(strPath + "RecordPointFilterOut.txt", "a+");// �˲�������
	FILE* pfPointCloud = fopen(strPath + "RecordPointCloud.txt", "a+");// ������Ʊ���

																	   // ��Դ����
	CDHGigeImageCapture* pDHCam = (CDHGigeImageCapture*)m_ptUnit->GetCameraCtrl(m_ptUnit->m_nTrackCameraNo); // ���ָ��
	IplImage* pImageBuff = pDHCam->m_pImageBuff; // ͼƬ������ //IplImage* pImageBuff = cvCreateImage(cvSize(tCameraParam.tDHCameraDriverPara.nMaxWidth, tCameraParam.tDHCameraDriverPara.nMaxHeight), IPL_DEPTH_8U, 1); // ͼƬ������
	std::vector<TrackFilter_Node> vtCoutTrackPoint(0);
	IplImage *pTempImgBuf = cvCreateImage(cvSize(pImageBuff->width, pImageBuff->height), IPL_DEPTH_8U, 1);

	pRobotCtrl->m_cLog->Write("[���ٵ���]:���ٹ켣������dInitComp:%lf dAbjustHeight:%lf", m_pTraceModel->m_dGunToEyeCompenX, m_pTraceModel->m_dGunToEyeCompenZ);

	AfxBeginThread(ThreadRealTimeTrackCapture_H, this); // �������ٲɼ������߳�
	// ��ʼ���ٹ���
	while (FALSE == m_pTraceModel->m_bCameraFindWeldEnd)//�ж��ҵ���β���ٲɵ�0
	{
		if (pRobotCtrl->m_eThreadStatus == INCISEHEAD_THREAD_STATUS_STOPPED)
		{
			pRobotCtrl->m_cLog->Write("[���ٵ���]:��ͣ���������̣߳� %d", pRobotCtrl->m_eThreadStatus);
			break;
		}

		// �յ��жϣ�1.��֪�յ�(�����յ�) 2.�����۳���(����) 3.���ӹ������ж�(�����յ�) ���ַ�ʽ
		// 1���յ��жϣ������յ����� �� ����m_pTraceModel->m_bCameraFindWeldEndΪtrue
		//JudgeFindWeldLineEnd(pRobotCtrl, nJudgeEndMethod); // ThreadRealTimeTrackCapture_H�а����յ��ж�
		//if (TRUE == m_pTraceModel->m_bCameraFindWeldEnd) break;

		// ��ThreadRealTimeTrackCapture_H�Ĵ������л�ȡ���º�������
		CHECK_BOOL_CONTINUE(GetTrackWeldLineWorldCoord(tWeldLineWorldCoord, nConnerRstNum, nPointCloudRstNum, nImgProFailNum, nMaxImgProFailNum, pfRawData));

		// 6������˲��Ƿ����� ��������˲�ʧ��ֹͣ���� ������λ�ۼƼ��� �������º�������tWeldLineWorldCoord ����˲����vtCoutTrackPoint �˲��������ݴ����ļ�pfRawData
		CHECK_BOOL_CONTINUE(CheckTrackFilterValid(pRobotCtrl, nImageNo, tWeldLineWorldCoord, vtCoutTrackPoint, nFilterFailNum, nMaxFilterFailNum));

		// 7���˲�������ݴ��� ������ε���Zֵʧ�� ֹͣ���� ������λ�ۼƼ��� ��vtCoutTrackPoint�е�ÿ������Ӳ��� д���ļ� �� ׷�ӵ����ٺ��ӹ켣������
		CHECK_BOOL_BREAK(FilterResultTrans(pRobotCtrl, nImageNo, vtCoutTrackPoint, nAdjustZErrNum, nMaxAdjustZErrNum, pfFilterOut));

		Sleep(100);
		DoEvent();
	}
	// �����յ�� ��ȡ�������յ������
	if (TRUE == m_pTraceModel->m_bCameraFindWeldEnd)
	{
		// ��ȡ���ٽ�β���յ�����켣
		GetEndSectionCoorsNew(pRobotCtrl, m_pTraceModel->m_tRealEndpointCoor);
		// ���ݰ��ǲ��� ��Ӱ��ǹ켣 

		// ���ٽ��������ر���
		m_pTraceModel->m_bIsTrackingDataSendOver = true; // !!!! ��ʱΪ���Ͱ��ǹ켣�ṩ�ж�����

	}
	fclose(pfFilterOut);
	fclose(pfRawData);
	fclose(pfPointCloud);
	m_ptUnit->SwitchIO("TrackLaser", false); //�������̽����رռ���

	pRobotCtrl->m_cLog->Write("[���ٵ���]: �����߳̽���");
}

bool CScanInitModule::GetTrackWeldLineWorldCoord(T_ROBOT_COORS &tWeldLineWorldCoord, int &nConnerRstNum, int &nPointCloudRstNum, int &nFailNum, int nMaxFailNum, FILE* pfFile)
{
	bool bSuccess = true;
	int nOldConnerRstNum = nConnerRstNum;
	int nOldPointCloudRstNum = nPointCloudRstNum;
	T_ROBOT_COORS tConnerWorldCoord;
	Three_DPoint tPointCloudWorldPtn;
	m_MutexTrackPointCloud.lock();
	nConnerRstNum = m_pTraceModel->vtConnerWorldCoords.size();
	nPointCloudRstNum = m_pTraceModel->vtPointCloudEndPoints.size();
	if (nConnerRstNum > 0)
	{
		tConnerWorldCoord = m_pTraceModel->vtConnerWorldCoords.back();
	}
	if (nPointCloudRstNum > 0)
	{
		tPointCloudWorldPtn = m_pTraceModel->vtPointCloudEndPoints.back();
	}
	m_MutexTrackPointCloud.unlock();

	if ((nPointCloudRstNum <= 0) && (nConnerRstNum > nOldConnerRstNum)) // �޵��ƽ�� �� �ǵ�������
	{
		tWeldLineWorldCoord = tConnerWorldCoord;
		fprintf(pfFile, "%11.3lf%11.3lf%11.3lf %10d%10d\n", tWeldLineWorldCoord.dX, tWeldLineWorldCoord.dY, tWeldLineWorldCoord.dZ, nConnerRstNum, nPointCloudRstNum); 
		nFailNum = 0;
	}
	else if (nPointCloudRstNum > nOldPointCloudRstNum) // �е��ƽ������
	{
		tWeldLineWorldCoord = T_ROBOT_COORS(tPointCloudWorldPtn.x, tPointCloudWorldPtn.y, tPointCloudWorldPtn.z, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
		fprintf(pfFile, "%11.3lf%11.3lf%11.3lf %10d%10d\n", tWeldLineWorldCoord.dX, tWeldLineWorldCoord.dY, tWeldLineWorldCoord.dZ, nConnerRstNum, nPointCloudRstNum);
		nFailNum = 0;
	}
	else
	{
		nFailNum++;
		if (nFailNum > nMaxFailNum)
		{
			CheckMachineEmg(m_pRobotDriver);
			m_pRobotDriver->m_cLog->Write("[��ȡ��������]:%s ����%d�λ�ȡ��������ʧ�ܣ�", m_pRobotDriver->m_strRobotName, nMaxFailNum);
			XUI::MesBox::PopInfo("[��ȡ��������]:{0} ����{1}�λ�ȡ��������ʧ�ܣ�", m_pRobotDriver->m_strRobotName, nMaxFailNum);
		}
		bSuccess = false;
		Sleep(100);
	}
	return bSuccess;
}


void CScanInitModule::InitRealTimeTracking(E_JUDGE_END_METHOD eJudgeEndMethod, T_ROBOT_COORS* pEndCoord/* = NULL*/)
{
	m_eJudgeEndMethod = eJudgeEndMethod;
	if ((NULL != pEndCoord) && (E_JUDGE_END_KNOW == m_eJudgeEndMethod)) // ��֪�յ�����յ��ж�
	{
		m_pTraceModel->m_tRealEndpointCoor = *pEndCoord;
		m_pTraceModel->m_tRealEndpointCoor.dX += m_pTraceModel->m_tRealEndpointCoor.dBX; m_pTraceModel->m_tRealEndpointCoor.dBX = 0.0;
		m_pTraceModel->m_tRealEndpointCoor.dY += m_pTraceModel->m_tRealEndpointCoor.dBY; m_pTraceModel->m_tRealEndpointCoor.dBY = 0.0;
		m_pTraceModel->m_tRealEndpointCoor.dZ += m_pTraceModel->m_tRealEndpointCoor.dBZ; m_pTraceModel->m_tRealEndpointCoor.dBZ = 0.0;
	}
}

/*
	ʹ��˵������
	1��Ҫ�� pRobotCtrl->GetCurrentPos(); ��ȡ���Ǹ��ٹ����� �����в�������˶��� �Ļ���������ϵ�����꣬���÷���ͬ����Ҫ���������������
	2���������ݣ�
		1����ʼ�δ���
			��ʼ��ԭʼ�ɼ����ݣ���� ֱ������ ����ֱ������
			��ʼ�δ�����˶�����: ��� ֱ������ ����ֱ������
		2�������˶���
			���з����˶��켣
			ʵ���˶�ʵʱ��ȡ�켣
		3�����ٹ��̣��� 
			1.���ٲɼ�ԭʼ����: ͼƬ��š���ά�㡢�������ꡢ��ͼֱ�����ꡢ��ͼ�ؽ�����
			2.�˲���:ͼƬ��� �˲�ԭʼ��� �˲������Ӳ��� �˲��������� �˲������ ˮƽ����ֵ �߶Ȳ���ֵ
		4���յ��жϣ�
			��֪�յ㣺�յ�ι켣 ͶӰǰ�յ����� ͶӰ���յ�����
		5�����ǹ켣��
			�������� �������
*/
void CScanInitModule::RealTimeTrackingNew(CRobotDriverAdaptor* pRobotCtrl)
{
	pRobotCtrl->m_cLog->Write("[���ٵ���]: �����߳̿�ʼ");
	// ������ʼ��
	bool bSaveImage			= false;// �Ƿ񱣴����ͼƬ
	int nJudgeEndMethod		= 3;	// 1.��֪�յ� 2.�����۳��� 3.���ӹ������ж�
	int nImageNo			= -1;	// ͼƬ���
	int nMaxImgProFailNum	= 20;	// �������ͼ����ʧ�ܴ���
	int nMaxAdjustZErrNum	= 5;	// �������Zֵ�仯�������
	int nMaxFilterFailNum	= 30;	// ��������˲�ʧ�ܴ���
	int nMaxErrStopNum		= 50;	// ����������ֹͣ����
	int nImgProFailNum		= 0;	// ����ͼ����ʧ�ܴ���
	int nAdjustZErrNum		= 0;	// ����Zֵ�仯�������
	int nFilterFailNum		= 0;	// �����˲�ʧ�ܴ���
	int nErrStopNum			= 0;	// �������ֹͣ����
	double dMinJoinPtnDis	= 0.5;	// ��ͼ��������С���ھ���
	T_ROBOT_COORS tWeldLineRobotCoord;	// ʵʱ����������������� X Y Z RX RY RZ
	T_ROBOT_COORS tWeldLineWorldCoord;	// ʵʱ��������������������� X Y Z
	T_ROBOT_COORS tCapRobotCurCoord;	// ��ͼʱ������ֱ������ X Y Z RX RY RZ BX BY BZ
	T_ANGLE_PULSE tCapRobotCurPulse;	// ��ͼʱ�����˹ؽ����� S L U R B T BX BY BZ
	T_ROBOT_COORS tCapWorldCurCoord;	// ��ͼʱ�������������� X Y Z RX RY RZ
	ReadTrackCapCoord(pRobotCtrl, tCapRobotCurCoord, tCapRobotCurPulse, tCapWorldCurCoord);
	T_ROBOT_COORS tRobotStartCoord(tCapRobotCurCoord);	// ��ʼ����ǰ�Ļ�е�۳�ʼλ��
	T_ROBOT_COORS tLastCapWorldCoord(tCapWorldCurCoord);// ʵʱ���һ�β�ͼʱ������������

	m_IsOpenJudgeEnd_ProcessSearch = false;
	m_pTraceModel->lnImageNo.clear();
	m_pTraceModel->lnImagePtnNum.clear();
	m_pTraceModel->ltPointCloud.clear();
	m_pTraceModel->ltCapWorldCoord.clear();
	m_pTraceModel->vtPointCloudEndPoints.clear();
	m_pTraceModel->m_dCurrentWeldLen = 0.0 > m_pTraceModel->m_dCurrentWeldLen ? 0.0 : m_pTraceModel->m_dCurrentWeldLen;	// ��ǰ�ۼƺ��ӳ��ȳ�ʼ��
	m_pTraceModel->m_dPreRzChange = m_pTraceModel->m_dRecordBreakPointRZ;
	m_pTraceModel->m_dAdjustRobotRZ = m_pTraceModel->m_dRecordBreakPointRZ;
	int nAllTrackNum = pRobotCtrl->m_vtWeldLineInWorldPoints.size();
	if (nAllTrackNum < 2)
	{
		XUI::MesBox::PopOkCancel("[���ٵ���]:���������ʼ��������������{0} ", nAllTrackNum);
		return;
	}

	// ·�� �� �ַ��� ��ʼ��
	CString strRobot = pRobotCtrl->m_strRobotName;
	CString strPath = OUTPUT_PATH + pRobotCtrl->m_strRobotName + WELDDATA_PATH;
	CString strSavePath = OUTPUT_PATH + pRobotCtrl->m_strRobotName + "\\Track\\";
	CString strImgPathName = "";

	// �����ļ���ʼ��
	FILE* pfRawData = fopen(strPath + "RecordPointRawData.txt", "w");// �˲�ǰԭʼ����
	FILE* pfFilterOut = fopen(strPath + "RecordPointFilterOut.txt", "w");// �˲�������
	FILE* pfPointCloud = fopen(strPath + "RecordPointCloud.txt", "w");// ������Ʊ���

	// ��Դ����
	CDHGigeImageCapture* pDHCam = (CDHGigeImageCapture*)m_ptUnit->GetCameraCtrl(m_ptUnit->m_nTrackCameraNo); // ���ָ��
	IplImage* pImageBuff = pDHCam->m_pImageBuff; // ͼƬ������ //IplImage* pImageBuff = cvCreateImage(cvSize(tCameraParam.tDHCameraDriverPara.nMaxWidth, tCameraParam.tDHCameraDriverPara.nMaxHeight), IPL_DEPTH_8U, 1); // ͼƬ������
	std::vector<TrackFilter_Node> vtCoutTrackPoint(0);

	pRobotCtrl->m_cLog->Write("[���ٵ���]:���ٹ켣������dInitComp:%lf dAbjustHeight:%lf", m_pTraceModel->m_dGunToEyeCompenX, m_pTraceModel->m_dGunToEyeCompenZ);

	// ��ʼ���ٹ���
	while (FALSE == m_pTraceModel->m_bCameraFindWeldEnd)//�ж��ҵ���β���ٲɵ�0
	{
		if (pRobotCtrl->m_eThreadStatus == INCISEHEAD_THREAD_STATUS_STOPPED)
		{
			pRobotCtrl->m_cLog->Write("[���ٵ���]:��ͣ���������̣߳� %d", pRobotCtrl->m_eThreadStatus);
			break;
		}

		// �յ��жϣ�1.��֪�յ�(�����յ�) 2.�����۳���(����) 3.���ӹ������ж�(�����յ�) ���ַ�ʽ
		// 1���յ��жϣ������յ����� �� ����m_pTraceModel->m_bCameraFindWeldEndΪtrue
		m_bDynamicJudgeEndPoint = true;
		JudgeFindWeldLineEnd(pRobotCtrl, nJudgeEndMethod);
		if (TRUE == m_pTraceModel->m_bCameraFindWeldEnd) break;

		// 2����ȡ����
		ReadTrackCapCoord(pRobotCtrl, tCapRobotCurCoord, tCapRobotCurPulse, tCapWorldCurCoord, nImageNo + 1);
		pRobotCtrl->m_cLog->Write("[��ȡ����]");
		// 3��ͼƬ�ɼ� ͼƬ���nImageNo��������Ϊ��ͼƬ������, ����ģʽ ��ȡ����ͼƬ
		CHECK_BOOL_BREAK(CaptureTrackImage(m_ptUnit->m_nTrackCameraNo, nImageNo, &pImageBuff, bSaveImage, strSavePath));
		if (TRUE == m_pTraceModel->m_bCameraFindWeldEnd) break;
		pRobotCtrl->m_cLog->Write("[ͼƬ�ɼ�]");
		// 4��ͼ���� ����ʧ�ܽ�������ѭ�� �������ֹͣ���� ������λ�ۼƼ���
		CHECK_BOOL_CONTINUE(TrackProcess(pRobotCtrl, nImageNo, &pImageBuff, tCapRobotCurCoord, tCapRobotCurPulse, tWeldLineRobotCoord, tWeldLineWorldCoord, nImgProFailNum, nMaxImgProFailNum, pfRawData, pfPointCloud));
		if (TRUE == m_pTraceModel->m_bCameraFindWeldEnd) break;

		// 5������������δ����Ƿ���� ������ι���ֹͣ���� ������λ�ۼƼ��� �� ���������ٲ�ͼλ��tLastCapWorldCoord �����ۼƺ��ӳ���
		CHECK_BOOL_CONTINUE(CheckAdjoinCapCoord(pRobotCtrl, nImageNo, tLastCapWorldCoord, tCapWorldCurCoord, dMinJoinPtnDis, nErrStopNum, nMaxErrStopNum));

		// 6������˲��Ƿ����� ��������˲�ʧ��ֹͣ���� ������λ�ۼƼ��� �������º�������tWeldLineWorldCoord ����˲����vtCoutTrackPoint �˲��������ݴ����ļ�pfRawData
		CHECK_BOOL_CONTINUE(CheckTrackFilterValid(pRobotCtrl, nImageNo, tWeldLineWorldCoord, vtCoutTrackPoint, nFilterFailNum, nMaxFilterFailNum));

		// 7���˲�������ݴ��� ������ε���Zֵʧ�� ֹͣ���� ������λ�ۼƼ��� ��vtCoutTrackPoint�е�ÿ������Ӳ��� д���ļ� �� ׷�ӵ����ٺ��ӹ켣������
		CHECK_BOOL_BREAK(FilterResultTrans(pRobotCtrl, nImageNo, vtCoutTrackPoint, nAdjustZErrNum, nMaxAdjustZErrNum, pfFilterOut));

		Sleep(20);
		DoEvent();
	}
	// �����յ�� ��ȡ�������յ������
	if (TRUE == m_pTraceModel->m_bCameraFindWeldEnd)
	{
		// ��ȡ���ٽ�β���յ�����켣
		GetEndSectionCoorsNew(pRobotCtrl, m_pTraceModel->m_tRealEndpointCoor);
		// ���ݰ��ǲ��� ��Ӱ��ǹ켣 

		// ���ٽ��������ر���
		m_pTraceModel->m_bIsTrackingDataSendOver = true; // !!!! ��ʱΪ���Ͱ��ǹ켣�ṩ�ж�����

	}
	fclose(pfFilterOut);
	fclose(pfRawData);
	fclose(pfPointCloud);
	m_ptUnit->SwitchIO("TrackLaser", false); //�������̽����رռ���

	pRobotCtrl->m_cLog->Write("[���ٵ���]: �����߳̽���");
}

void CScanInitModule::ReadTrackCapCoord(CRobotDriverAdaptor* pRobotCtrl, T_ROBOT_COORS& tRobotCoord, T_ANGLE_PULSE& tRobotPulse, T_ROBOT_COORS& tWorldCoord, int nImageNo)
{
	if (GetLocalDebugMark())
	{
		if (nImageNo < 0) nImageNo = 0;
		if (nImageNo >= m_pTraceModel->vtTrackRawData.size()) nImageNo = m_pTraceModel->vtTrackRawData.size() - 1;
		if (nImageNo > m_pTraceModel->vtTrackRawData.size())
		{
			XUI::MesBox::PopInfo("���Ի�ȡͼ{0}�Ĳɼ�����ʧ�� ������{1}", nImageNo, m_pTraceModel->vtTrackRawData.size());
			return;
		}
		tRobotCoord = m_pTraceModel->vtTrackRawData[nImageNo].tCapRobotCoord;
		tRobotPulse = m_pTraceModel->vtTrackRawData[nImageNo].tCapRobotPulse;
		tWorldCoord = tRobotCoord;
		tWorldCoord.dX += tWorldCoord.dBX; tWorldCoord.dBX = 0.0;
		tWorldCoord.dY += tWorldCoord.dBY; tWorldCoord.dBY = 0.0;
		tWorldCoord.dZ += tWorldCoord.dBZ; tWorldCoord.dBZ = 0.0;
	}
	else
	{
		//��ȡ��ǰλ��
		tRobotCoord = pRobotCtrl->GetCurrentPos();
		tRobotPulse = pRobotCtrl->GetCurrentPulse();
		//tRobotCoord.dBX = 0.0;
		//tRobotPulse.lBXPulse = 0;
		// ��ͼʱ��������������
		tWorldCoord = T_ROBOT_COORS(tRobotCoord.dX + tRobotCoord.dBX, tRobotCoord.dY + tRobotCoord.dBY,
			tRobotCoord.dZ + tRobotCoord.dBZ, tRobotCoord.dRX, tRobotCoord.dRY, tRobotCoord.dRZ, 0.0, 0.0, 0.0);
	}
}

bool CScanInitModule::CaptureTrackImage(int nCameraNo, int &nImageNo, IplImage** ppImageBuff, bool bSaveImage, CString sSavePath)
{
	bool bSuccess = true;
	CString sImageFileName;
	nImageNo++; // ͼƬ��� ������0��ʼ
	sImageFileName.Format("%s%d.jpg", sSavePath, nImageNo);
	if (GetLocalDebugMark())
	{
		if (!CheckFileExists(sImageFileName))
		{
			return true;
		}
		*ppImageBuff = cvLoadImage(sImageFileName.GetBuffer(), 0);
		if (m_pTraceImgProcess == NULL)
		{
			XiLineParamNode tFrontLine;
			XiLineParamNode tBackLine;
			m_pTraceImgProcess = new CImageProcess(
				m_ptUnit->GetCameraParam(m_ptUnit->m_nTrackCameraNo).tDHCameraDriverPara.nMaxWidth,
				m_ptUnit->GetCameraParam(m_ptUnit->m_nTrackCameraNo).tDHCameraDriverPara.nMaxHeight,
				1, m_ptUnit->GetTrackTipDataFileName(m_ptUnit->m_nTrackCameraNo));
			ImageLockProcess(m_pTraceImgProcess, ppImageBuff, tFrontLine, tBackLine);
		}
	}
	else
	{
		CDHGigeImageCapture* pDHCam = (CDHGigeImageCapture*)m_ptUnit->GetCameraCtrl(m_ptUnit->m_nTrackCameraNo); // ���ָ��
		if (!pDHCam->CaptureImage(*ppImageBuff,1))
		{
			return false;
		}
		//cvCopyImage(pDHCam->CaptureImage(FALSE), *ppImageBuff);
		if (bSaveImage)SaveImage(*ppImageBuff, "%s", sImageFileName); // ��ͼ
	}
	return bSuccess;
}

bool CScanInitModule::ImageLockProcess(CImageProcess* pTraceImgProcess, IplImage** ppImage, XiLineParamNode& tFrontLine, XiLineParamNode& tBackLine, E_FLIP_MODE eFlipMode/* = E_FLIP_NONE*/)
{
	T_ANGLE_PULSE tCurPulse = m_pRobotDriver->GetCurrentPulse();
	T_ROBOT_COORS tRobotCoors, tCameraCoors;
	m_pRobotDriver->RobotKinematics(tCurPulse, m_pRobotDriver->m_tTools.tGunTool, tRobotCoors);
	m_pRobotDriver->RobotKinematics(tCurPulse, m_pRobotDriver->m_tTools.tCameraTool, tCameraCoors);

	CvPoint cpMidKeyPoint, LeftPtn, RightPtn, tKeyPoint;
	vector<CvPoint> vtLeftPtns, vtRightPtns;
	pTraceImgProcess->InitIfStartOrEndPntParam(E_PIECE_START);

	int nParamsArrayAAA[3];            //��ͼ��ʽ
	nParamsArrayAAA[0] = (int)CV_IMWRITE_JPEG_QUALITY;
	nParamsArrayAAA[1] = (int)(0.1 * 100);
	nParamsArrayAAA[2] = 0;
	cvSaveImage("AAA.jpg", /**ppImage*/ *ppImage, nParamsArrayAAA);
	FlipImage(*ppImage, eFlipMode); // ��ת

	IplImage* pImg = *ppImage;

	WidgetLaserInfo* LaserInfo = new WidgetLaserInfo[10];
	char* LogName = "WidgetLaserLock";

	int PtnNum = WidgetLaserLock(pImg, LaserInfo, true, false, LogName);
	if (PtnNum <= 0)
	{
		XiMessageBoxOk("ʾ��ʧ�ܣ����鼤��ͼƬAAA.jpg�Ƿ�����");

	}
	else if (PtnNum == 1)
	{
		cpMidKeyPoint = LaserInfo[0].crossPoint;
		for (int i = 0; i < LaserInfo[0].samPntNum; i++)
		{
			// ����ͼ��༤���� ����
			vtLeftPtns.push_back(LaserInfo[0].verticalPlateLineSamPnt[i]);
			// ����ͼ�Ҳ༤���� �װ�
			vtRightPtns.push_back(LaserInfo[0].bottomPlateLineSamPnt[i]);
		}
	}
	else if (PtnNum > 1)
	{
		CvPoint tmpPoint;
		T_ROBOT_COORS tmpKeyPoint;
		double wDis;
		double zDis;
		vector<double> vtWorldDis;
		vector<double> vtHeightDis;
		for (int keyNum = 0; keyNum < PtnNum; keyNum++)
		{
			tmpPoint = LaserInfo[keyNum].crossPoint;
			ResumeImagePoint2D(pImg->width, pImg->height, tmpPoint, eFlipMode);
			tmpKeyPoint = m_ptUnit->TranImageToBase(m_ptUnit->m_nMeasureCameraNo, tmpPoint, tRobotCoors, tCurPulse);
			wDis = TwoPointDis(tmpKeyPoint.dX, tmpKeyPoint.dY, tmpKeyPoint.dZ,
				tCameraCoors.dX, tCameraCoors.dY, tCameraCoors.dZ);
			zDis = fabs(tmpKeyPoint.dZ - tCameraCoors.dZ);
			vtHeightDis.push_back(zDis);
			vtWorldDis.push_back(wDis);
		}

		int realKey = 0;
		auto minHeightPosition = min_element(vtHeightDis.begin(), vtHeightDis.end());
		auto maxHeightPosition = max_element(vtHeightDis.begin(), vtHeightDis.end());
		auto minWorldPosition = min_element(vtWorldDis.begin(), vtWorldDis.end());

		if (fabs(*maxHeightPosition - *minHeightPosition) > 5)
		{
			realKey = minHeightPosition - vtHeightDis.begin();
		}
		else
		{
			realKey = minWorldPosition - vtWorldDis.begin();
		}
		cpMidKeyPoint = LaserInfo[realKey].crossPoint;
		for (int i = 0; i < LaserInfo[realKey].samPntNum; i++)
		{
			// ����ͼ��༤���� ����
			vtLeftPtns.push_back(LaserInfo[realKey].verticalPlateLineSamPnt[i]);
			// ����ͼ�Ҳ༤���� �װ�
			vtRightPtns.push_back(LaserInfo[realKey].bottomPlateLineSamPnt[i]);
		}
	}

	/*pTraceImgProcess->GetBesideKeyPoints(*ppImage, cpMidKeyPoint, cpBesideKeyPoint, vtLeftPtns, vtRightPtns, false,
		500, 500,
		300, 300,
		20, 20);*/
	if (vtLeftPtns.size() <= 0 || vtRightPtns.size() <= 0)
	{
		XiMessageBox("����б������ʧ��");
		return false;
	}

	LeftPtn = vtLeftPtns[vtLeftPtns.size() / 2];
	RightPtn = vtRightPtns[vtRightPtns.size() / 2];

	ResumeImagePoint2D((*ppImage)->width, (*ppImage)->height, cpMidKeyPoint, eFlipMode);
	ResumeImagePoint2D((*ppImage)->width, (*ppImage)->height, LeftPtn, eFlipMode);
	ResumeImagePoint2D((*ppImage)->width, (*ppImage)->height, RightPtn, eFlipMode); // ��ά��ָ�
	tKeyPoint = pTraceImgProcess->HandlockKeyPoint(cpMidKeyPoint, LeftPtn, RightPtn, tFrontLine, tBackLine);

	CvPoint p1 = cvPoint(0, tFrontLine.b);
	CvPoint p2 = cvPoint((*ppImage)->width, (*ppImage)->width * tFrontLine.k + tFrontLine.b);
	CvPoint p3 = cvPoint(0, tBackLine.b);
	CvPoint p4 = cvPoint((*ppImage)->width, (*ppImage)->width * tBackLine.k + tBackLine.b);

	FlipImage(*ppImage, eFlipMode); // ��ת�ָ�

	cvCvtColor(*ppImage, m_pShowImage, CV_GRAY2RGB);
	cvCircle(m_pShowImage, tKeyPoint, 10, CV_RGB(255, 0, 0), 3);
	cvLine(m_pShowImage, p1, p2, CV_RGB(0, 255, 0), 2);
	cvLine(m_pShowImage, p3, p4, CV_RGB(0, 0, 255), 2);
	CString cstrTemp = OUTPUT_PATH + m_ptUnit->GetUnitName() + SEARCH_SCANLOCK_IMG;
	int nParamsArray[3];            //��ͼ��ʽ
	nParamsArray[0] = (int)CV_IMWRITE_JPEG_QUALITY;
	nParamsArray[1] = (int)(0.1 * 100);
	nParamsArray[2] = 0;
	cvSaveImage((LPCSTR)cstrTemp, /**ppImage*/ m_pShowImage, nParamsArray);

	return true;
}

bool CScanInitModule::TrackProcess(CRobotDriverAdaptor* pRobotCtrl, int nImageNo, IplImage** pImageBuff, T_ROBOT_COORS tRobotCoord, T_ANGLE_PULSE tRobotPulse,
	T_ROBOT_COORS& tWeldLineRobotCoord, T_ROBOT_COORS& tWeldLineWorldCoord, int &nFailNum, int nMaxFailNum, FILE *pfFile, FILE* pfPointCloud)
{
	auto dPointCloudLengthForFindEndpnt = PARA_FLAT_MEASURE(dPointCloudLengthForFindEndpnt);
	long long lTime = XI_clock();
	bool bSuccess = true;
	CString strRobot = pRobotCtrl->m_strRobotName;
	CvPoint Corner2D;
	T_ABS_POS_IN_BASE tPointAbsCoordInBase;
//	XiLineParamNode tFrontLine, tBackLine;
	T_CAMREA_PARAM tCameraParam = m_ptUnit->GetCameraParam(m_ptUnit->m_nTrackCameraNo);
	int nMinX = tCameraParam.tDHCameraDriverPara.nRoiWidth / 4;
	int nMaxX = tCameraParam.tDHCameraDriverPara.nRoiWidth * 3 / 4;

	//Corner2D = m_pTraceImgProcess->GetGroupStandKeyPoint(*pImageBuff, tFrontLine, tBackLine, FALSE, TRUE, TRUE);
	if (nImageNo < 5) {
		bool bProcRst = WeightedPointCluster(*pImageBuff, &m_pTraceModel->tFrontLine, &m_pTraceModel->tBackLine, &Corner2D, false, "WeightedPointCluster_Big");
		CString cstr;
		cstr.Format("��%d��ͼ", nImageNo);
		TRACE(cstr.GetBuffer());
	}
	else
	{
		bool bProcRst = WeightedPointCluster(*pImageBuff, &m_pTraceModel->tFrontLine, &m_pTraceModel->tBackLine, &Corner2D, false, "WeightedPointCluster");
		CString cstr;
		cstr.Format("��%d��ͼ", nImageNo);
		TRACE(cstr.GetBuffer());
	}

	CvPoint* points = new CvPoint[10000];
	//int nLength = FindLaserMidPiontEE(*pImageBuff, points, 3, 30, 90, 5, 11, 0, 3, 20000, 3, 1, 2, false);
	//int nLength = FindLaserMidPiontEEEEEInTrack_(*pImageBuff, m_pTraceModel->tFrontLine, Corner2D, points, 100, 0, 3, 30, 8, 30, 10, 0.5, 10000, 3, 11, 0);
	int nLength = FindLaserMidPiontEEEEEInTrack_Fix(*pImageBuff, m_pTraceModel->tFrontLine, m_pTraceModel->tBackLine,points, "FindLaserMidPiontEEEEEInTrack_Fix");
	//nLength += FindLaserMidPiontEEEEEInTrack_Fix(*pImageBuff, m_pTraceModel->tBackLine, Corner2D, points + nLength, 50, 0, 3, 90, 11, 90, 10, 0.5, 10000 - nLength, 3, 11, 0);


	tWeldLineRobotCoord = m_ptUnit->TranImageToBase(m_ptUnit->m_nTrackCameraNo, Corner2D, tRobotCoord, tRobotPulse, &tPointAbsCoordInBase);
	tWeldLineWorldCoord = tWeldLineRobotCoord;
	tWeldLineWorldCoord.dX += tRobotCoord.dBX;
	tWeldLineWorldCoord.dY += tRobotCoord.dBY;
	tWeldLineWorldCoord.dZ += tRobotCoord.dBZ;

	// �е��ƴ�����ʱʹ�õ��ƴ�������
	if (m_pTraceModel->vtPointCloudEndPoints.size() > 0)
	{
		Three_DPoint tPtn = m_pTraceModel->vtPointCloudEndPoints.back();
		tWeldLineWorldCoord.dX = tPtn.x;
		tWeldLineWorldCoord.dY = tPtn.y;
		tWeldLineWorldCoord.dZ = tPtn.z;
		T_ROBOT_COORS tRefCoord(tWeldLineRobotCoord);
		tWeldLineRobotCoord = tWeldLineWorldCoord;
		SplitCoord(tRefCoord, tWeldLineRobotCoord, m_ptUnit->m_nTrackAxisNo);
		pRobotCtrl->m_cLog->Write("[ͼ����]: ʹ�õ��ƴ����� ͼ��%d", nImageNo);
	}

	// �������ݱ���
	m_MutexTrackPointCloud.lock();
	T_ROBOT_COORS tPointCloudRobotCoord;
	T_ROBOT_COORS tPointCloudWorldCoord; 
	Three_DPoint tThreePoint;
	int nSaveNum = 0;
	for (int i = 0; i < nLength; i++)
	{
		if (points[i].x < nMinX || points[i].x > nMaxX) continue; // ��Ч����������ݲ�����
		tPointCloudRobotCoord = m_ptUnit->TranImageToBase(m_ptUnit->m_nTrackCameraNo, points[i], tRobotCoord, tRobotPulse, &tPointAbsCoordInBase);
		tPointCloudWorldCoord = tPointCloudRobotCoord;
		tPointCloudWorldCoord.dX += tRobotCoord.dBX;
		tPointCloudWorldCoord.dY += tRobotCoord.dBY;
		tPointCloudWorldCoord.dZ += tRobotCoord.dBZ;
		tThreePoint.x = tPointCloudWorldCoord.dX;
		tThreePoint.y = tPointCloudWorldCoord.dY;
		tThreePoint.z = tPointCloudWorldCoord.dZ;
		m_pTraceModel->ltPointCloud.push_back(tThreePoint); 
		nSaveNum++;
		fprintf(pfPointCloud, "ͼ��:%d ���%d %11.3lf%11.3lf%11.3lf\n", nImageNo, i, tPointCloudWorldCoord.dX, tPointCloudWorldCoord.dY, tPointCloudWorldCoord.dZ);
	}

	m_pTraceModel->lnImageNo.push_back(nImageNo); 
	m_pTraceModel->lnImagePtnNum.push_back(nSaveNum); 
	m_pTraceModel->ltCapWorldCoord.push_back(tRobotCoord);
	// ɾ���ɵ�������
	for (int i = 0; (i < m_pTraceModel->ltCapWorldCoord.size()) && (m_pTraceModel->ltCapWorldCoord.size() > 60); i++)
	{
		T_ROBOT_COORS tCoordS = m_pTraceModel->ltCapWorldCoord.front();
		T_ROBOT_COORS tCoordE = m_pTraceModel->ltCapWorldCoord.back();
		double dDis = TwoPointDis(
			tCoordS.dX + tCoordS.dBX, tCoordS.dY + tCoordS.dBY, tCoordS.dZ + tCoordS.dBZ,
			tCoordE.dX + tCoordE.dBX, tCoordE.dY + tCoordE.dBY, tCoordE.dZ + tCoordE.dBZ);
		if (dDis > dPointCloudLengthForFindEndpnt)
		{
			int nDelPtnNum = m_pTraceModel->lnImagePtnNum.front();
			std::list<Three_DPoint>::iterator iter = m_pTraceModel->ltPointCloud.begin();
			std::advance(iter, nDelPtnNum);
			m_pTraceModel->ltPointCloud.erase(m_pTraceModel->ltPointCloud.begin(), iter);
			m_pTraceModel->ltCapWorldCoord.pop_front();
			m_pTraceModel->lnImagePtnNum.pop_front();
			m_pTraceModel->lnImageNo.pop_front();
		}
	}
	m_MutexTrackPointCloud.unlock();

	if (Corner2D.x > 1 && Corner2D.x < 2591 && Corner2D.y>1 && Corner2D.y < 1993)
	{
		cvCvtColor(*pImageBuff, m_pShowImage, CV_GRAY2RGB);
		cvCircle(m_pShowImage, Corner2D, 8, CV_RGB(255, 0, 0), 4);
	}

	pRobotCtrl->m_cLog->Write("[ͼ����]:%s ͼƬ��:%d �����������꣺%.3lf %.3lf %.3lf ��ͼ�������꣺%.3lf %.3lf %.3lf", strRobot, nImageNo,
		tWeldLineRobotCoord.dX, tWeldLineRobotCoord.dY, tWeldLineRobotCoord.dZ, 
		tRobotCoord.dX + tRobotCoord.dBX, tRobotCoord.dY + tRobotCoord.dBY, tRobotCoord.dZ + tRobotCoord.dBZ);
	if (!GetLocalDebugMark())
	{
		// ���ٲɼ�ԭʼ����: ͼƬ���n����ά��x y��������������x y z����ͼֱ������x y z rx ry rz bx by bz����ͼ�ؽ�����S L U R B T BX BY BZ
		fprintf(pfFile, "%d %4d%5d %11.3lf%11.3lf%11.3lf %11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf %10d%10d%10d%10d%10d%10d%10d%10d%10d\n",
			nImageNo, Corner2D.x, Corner2D.y,
			tWeldLineWorldCoord.dX, tWeldLineWorldCoord.dY, tWeldLineWorldCoord.dZ,
			tRobotCoord.dX, tRobotCoord.dY, tRobotCoord.dZ,
			tRobotCoord.dRX, tRobotCoord.dRY, tRobotCoord.dRZ,
			tRobotCoord.dBX, tRobotCoord.dBY, tRobotCoord.dBZ,
			tRobotPulse.nSPulse, tRobotPulse.nLPulse, tRobotPulse.nUPulse,
			tRobotPulse.nRPulse, tRobotPulse.nBPulse, tRobotPulse.nTPulse,
			tRobotPulse.lBXPulse, tRobotPulse.lBYPulse, tRobotPulse.lBZPulse);
		fflush(pfFile);
	}

	// ����ɹ���
	if (1 > Corner2D.x || 1 > Corner2D.y || 2448 < Corner2D.x || 2048 < Corner2D.y)
	{
		pRobotCtrl->m_cLog->Write("[ͼ����]:%s kong Corner2D.x:%d Corner2D.y:%d", strRobot, Corner2D.x, Corner2D.y);
		nFailNum++;
		if (nFailNum > nMaxFailNum)
		{
			pRobotCtrl->m_cLog->Write("[ͼ����]:%s ����%d���ҵ���ֵ������ͣ����", strRobot, nMaxFailNum);
			CheckMachineEmg(pRobotCtrl);
			XUI::MesBox::PopOkCancel("[ͼ����]:{0}�Ż����� ����{1}����ҵ���ֵ������ͣ����", pRobotCtrl->m_nRobotNo, nMaxFailNum);
		}
		bSuccess = false;
	}
	pRobotCtrl->m_cLog->Write("[ͼ����]:%d ���δ���ʱ��:%dms %d %d ���ĵ���ȡ������%d",
		nImageNo, XI_clock() - lTime, Corner2D.x, Corner2D.y, nLength);
	return bSuccess;
}

bool CScanInitModule::CheckAdjoinCapCoord(CRobotDriverAdaptor* pRobotCtrl, int nImageNo, T_ROBOT_COORS& tLastCapWorldCoord, T_ROBOT_COORS tCurCapWorldCoord, double dDisThreshold, int& nErrNum, int nMaxErrNum)
{
	bool bValid = false;
	double dLatestDistance = sqrt(
		pow(tCurCapWorldCoord.dX - tLastCapWorldCoord.dX, 2) +
		pow(tCurCapWorldCoord.dY - tLastCapWorldCoord.dY, 2) +
		pow(tCurCapWorldCoord.dZ - tLastCapWorldCoord.dZ, 2));
	if (dLatestDistance < dDisThreshold)
	{
		pRobotCtrl->m_cLog->Write("[������]: %s �����������ϴξ���С��%.3lfmm,�����д���:%.3lf", pRobotCtrl->m_strRobotName, dDisThreshold, dLatestDistance);
		nErrNum++;
		if (nErrNum >= nMaxErrNum)
		{
			CheckMachineEmg(pRobotCtrl);
			XUI::MesBox::PopOkCancel("[������]: {0}�Ż����˴�ͣ��!��Ϣ��!�ɶϵ�����", pRobotCtrl->m_nRobotNo);
		}
	}
	else
	{
		// �������һ�β�ͼ��������
		tLastCapWorldCoord = tCurCapWorldCoord;
		nErrNum = 0;
		bValid = true;
		m_pTraceModel->m_dCurrentWeldLen += dLatestDistance;
	}
	pRobotCtrl->m_cLog->Write("[������]: %s ͼ��:%d ���������ܳ��ȣ�%11.3lf,��ǰ���ӳ���:%11.3lf���ƶ�����:%lf ��ǰ�������꣺%.3lf %.3lf %.3lf",
		pRobotCtrl->m_strRobotName, nImageNo, m_pTraceModel->m_dWeldLen, m_pTraceModel->m_dCurrentWeldLen, dLatestDistance,
		tCurCapWorldCoord.dBX, tCurCapWorldCoord.dBY, tCurCapWorldCoord.dBZ);
	return bValid;
}

bool CScanInitModule::CheckTrackFilterValid(CRobotDriverAdaptor* pRobotCtrl, int nImageNo, T_ROBOT_COORS tInputCoord, std::vector<TrackFilter_Node>& vtCoutTrackPoint, int &nInvalidNum, int nMaxInvalidNum)
{
	bool bValid = true;
	vtCoutTrackPoint.clear();
	CString strRobot = pRobotCtrl->m_strRobotName;
	pRobotCtrl->m_cLog->Write("[�˲�����]:%s �˲�ǰ����:ͼ��:%d, %11.3lf %11.3lf %11.3lf �ۼ���Ч����:%d", strRobot, nImageNo, tInputCoord.dX, tInputCoord.dY, tInputCoord.dZ, nInvalidNum);

	vtCoutTrackPoint = TrackFilter_FilterCurvePointInPointOut(m_ptUnit->m_nRobotSmooth, tInputCoord.dX, tInputCoord.dY, tInputCoord.dZ, 1 == pRobotCtrl->m_nRobotInstallDir ? false : true);
	// �˲��ɹ���
	if (vtCoutTrackPoint.size() < 1)
	{
		pRobotCtrl->m_cLog->Write("[�˲�����]:%s CoutTrackPoint����Ϊ�� %d", strRobot, vtCoutTrackPoint.size());
		nInvalidNum++;
		if (nInvalidNum > nMaxInvalidNum)
		{
			CheckMachineEmg(pRobotCtrl);
			pRobotCtrl->m_cLog->Write("[�˲�����]:%s CoutTrackPoint����Ϊ�մ������࣬һ���ⲻ̫������ͣ����������̧ǹ���������", strRobot);
			XUI::MesBox::PopInfo("[�˲�����]:{0} CoutTrackPoint����Ϊ�մ������࣬һ���ⲻ̫������ͣ����������̧ǹ���������", strRobot);
		}
		bValid = false;;
	}
	else
	{
		bValid = true;
		nInvalidNum = 0; // �˲�ʧ�ܴ�����ո�λ
	}
	return bValid;
}

bool CScanInitModule::FilterResultTrans(CRobotDriverAdaptor* pRobotCtrl, int nImageNo, const std::vector<TrackFilter_Node>& vtCoutTrackPoint, int& nAdjustErrNum, int nMaxAdjustErrNum, FILE* pfFile)
{
	bool bSuccess = true;
	CString strRobot = pRobotCtrl->m_strRobotName;
	double dTrackVerDegree = 0.0;
	double dInitComp = m_pTraceModel->m_dGunToEyeCompenX;// �˲���Ӳ���ֵ������ڼ�
	double dAbjustHeight = m_pTraceModel->m_dGunToEyeCompenZ;// �˲���Ӳ���ֵ�����ϲ��� ���²���

	for (int nSize = 0; nSize < vtCoutTrackPoint.size(); nSize++)
	{
		dTrackVerDegree = atan2(vtCoutTrackPoint[nSize].normalDir_.y_, vtCoutTrackPoint[nSize].normalDir_.x_) * 180.0 / PI;
		T_ROBOT_COORS tLastTrackCoord = pRobotCtrl->m_vtWeldLineInWorldPoints.back();
		T_ROBOT_COORS tFilterOutCoord(
			vtCoutTrackPoint[nSize].pnt_.x_ + (dInitComp * CosD(dTrackVerDegree)),
			vtCoutTrackPoint[nSize].pnt_.y_ + (dInitComp * SinD(dTrackVerDegree)),
			vtCoutTrackPoint[nSize].pnt_.z_ + (dAbjustHeight),
			tLastTrackCoord.dRX, tLastTrackCoord.dRY, tLastTrackCoord.dRZ, 0.0, 0.0, 0.0);

		pRobotCtrl->m_cLog->Write("[�켣���]: %s ͼ��:%d �˲��󲹳�: %.3lf  %.3lf  %.3lf �˲������:%.3lf �˲�����:%.3lf %.3lf %.3lf  ",
			strRobot, nImageNo, tFilterOutCoord.dX, tFilterOutCoord.dY, tFilterOutCoord.dZ, dTrackVerDegree,
			vtCoutTrackPoint[nSize].normalDir_.x_, vtCoutTrackPoint[nSize].normalDir_.y_, vtCoutTrackPoint[nSize].normalDir_.z_);
		if (!GetLocalDebugMark())
		{
			// ͼƬ���n �˲�ԭʼ���xyz �˲������Ӳ���xyz �˲���������xyz �˲������A ˮƽ����ֵa �߶Ȳ���ֵb
			fprintf(pfFile, "%d %11.3lf%11.3lf%11.3lf %11.3lf%11.3lf%11.3lf %8.3lf%8.3lf%8.3lf %8.3lf%8.3lf%8.3lf\n",
				nImageNo, vtCoutTrackPoint[nSize].pnt_.x_, vtCoutTrackPoint[nSize].pnt_.y_, vtCoutTrackPoint[nSize].pnt_.z_,
				tFilterOutCoord.dX, tFilterOutCoord.dY, tFilterOutCoord.dZ,
				vtCoutTrackPoint[nSize].normalDir_.x_, vtCoutTrackPoint[nSize].normalDir_.y_, vtCoutTrackPoint[nSize].normalDir_.z_,
				dTrackVerDegree, dInitComp, dAbjustHeight);
			fflush(pfFile);
		}

		double dAdjustZ = tFilterOutCoord.dZ - tLastTrackCoord.dZ;
		// Zֵ����������
		if (fabs(dAdjustZ) > 4.0) // �����˸߶ȱ仯��ʱ���޸�
		{
			double dRealAdjustZ = dAdjustZ > 0.0 ? 2.0 : -2.0;
			tFilterOutCoord.dZ = tLastTrackCoord.dZ + dRealAdjustZ;
			nAdjustErrNum++;
			pRobotCtrl->m_cLog->Write("[�켣���]:%s �˲������߶�dAdjustZ:%.3lf ʵ�ʵ����߶�dRealAdjustZ:%.3lf nAbjustZErrNum:%d nMaxAdjustErrNum:%d",
				strRobot, dAdjustZ, dRealAdjustZ, nAdjustErrNum, nMaxAdjustErrNum);
			if (nAdjustErrNum > nMaxAdjustErrNum)
			{
				CheckMachineEmg(pRobotCtrl);
				pRobotCtrl->m_cLog->Write("[�켣���]:%s zֵ��������������࣬ͣ����������̧ǹ���������", strRobot);
				XiMessageBox("[�켣���]: Zֵ��������������࣬ͣ����������̧ǹ���������"); 
				bSuccess = false;
				break;
			}
		}
		else
		{
			nAdjustErrNum = 0;
		}

		//----------------------- Բ��������̬�������� ����ں��������̬��Rz��ֵ
		long lMinAjustRzTimeInterval = 1000;
		long lCurTimeTick = GetTickCount();
		long lTimeInterval = lCurTimeTick - m_pTraceModel->m_lPreAdjustRzTimeTick;
		if (labs(lTimeInterval) > lMinAjustRzTimeInterval && true == m_bWorkpieceShape) // Բ��������̬
		{
			m_pTraceModel->m_lPreAdjustRzTimeTick = lCurTimeTick; // ����ʱ���
			double dDefaultRz = pRobotCtrl->m_vtWeldLineInWorldPoints.front().dRZ; // tFilterOutCoord.dRZ;
			double dTestRz = DirAngleToRz(dTrackVerDegree);//���������Rz  ����ʵʱ�仯��ǰ�������()��U��Ƕȼ���Rz 																						  // �ٽ��������ԭʼdTestRzȡƽ��ֵ��ʹ					 

			pRobotCtrl->m_cLog->Write("����Ǽ���RZ:%lf dDefaultRz:%.3lf", dTestRz, dDefaultRz);

			// ���Ƽ���Rz��0-360����
			dTestRz = fmod(dTestRz, 360.0);
			if (dTestRz < 0.0) {
				dTestRz += 360.0; // 0 - 360
			}
			double dRzChange = dTestRz - dDefaultRz;
			dRzChange = fmod(dRzChange, 360.0);
			if (dRzChange < -180)//��ֹ����dTestRz��360��0
			{
				dRzChange += 360.0; // 0 - 360
			}
			else if (dRzChange > 180)
			{
				dRzChange -= 360.0;
			}
			//����RZ���ϴμ�¼RZ��ֵ
			double dRzBefAndAftDiff = dRzChange - m_pTraceModel->m_dPreRzChange;
			if (dRzBefAndAftDiff > 180)
			{
				dRzBefAndAftDiff -= 360;
			}
			else if (dRzBefAndAftDiff < -180)
			{
				dRzBefAndAftDiff += 360;
			}
			int nAdjustRzErrorNum = 0;
			double dAdjustMaxInterval = 2.0, dAdjustMinInterval = 0.5;

			if ((fabs(dRzBefAndAftDiff) > dAdjustMaxInterval)) // ��̬�ǶȲ������仯ֵ����
			{
				double dDir = dRzBefAndAftDiff > 0.0 ? 1.0 : -1.0;
				dRzChange = m_pTraceModel->m_dPreRzChange + (dAdjustMaxInterval * dDir);
				nAdjustRzErrorNum++;
				if (nAdjustRzErrorNum > 10)
				{
					CheckMachineEmg(pRobotCtrl);
					pRobotCtrl->m_cLog->Write("%s Rzֵ��������������࣬ͣ����������̧ǹ���������", strRobot);
					XiMessageBox("Rzֵ��������������࣬ͣ����������̧ǹ���������");
					break;
				}
			}
			else
			{
				nAdjustRzErrorNum = 0;
			}
			pRobotCtrl->m_cLog->Write("%s �������dTestRz: %lf RZ��ֵ: %lf dRzChange %lf dPreRzChange:%lf",
				strRobot, dTestRz, dRzBefAndAftDiff, dRzChange, m_pTraceModel->m_dPreRzChange);
			if ((fabs(dRzBefAndAftDiff) < dAdjustMinInterval)) // ��̬�ǶȲ�����С�仯ֵ����
			{
				dRzChange = m_pTraceModel->m_dPreRzChange;
			}
			m_pTraceModel->m_dPreRzChange = dRzChange; // ��¼��һ��Rz���� P99
			m_pTraceModel->m_dAdjustRobotRZ = dRzChange;
			//--------------------------
		}

		pRobotCtrl->m_cLog->Write("[�켣���]: %s ͼ��:%d �����������ݣ� %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf ",
			strRobot, nImageNo, tFilterOutCoord.dX, tFilterOutCoord.dY, tFilterOutCoord.dZ, tFilterOutCoord.dRX, tFilterOutCoord.dRY, tFilterOutCoord.dRZ);

		// ��tFilterOutCoord��������XYZ��ֵ��ⲿ��
		T_ROBOT_COORS tRobotStartCoord = pRobotCtrl->m_vtWeldLineInWorldPoints[0];
		SplitCoord(tRobotStartCoord, tFilterOutCoord, m_ptUnit->m_nTrackAxisNo);
		pRobotCtrl->m_vtWeldLineInWorldPoints.push_back(tFilterOutCoord);
		m_pTraceModel->m_vtWeldLinePointType.push_back(E_WELD_TRACK);
		SaveRobotCoor(m_pRecordTheoryPoint, tFilterOutCoord, E_WELD_TRACK);//������������
	}
	return bSuccess;
}

void CScanInitModule::SplitCoord(T_ROBOT_COORS tRefCoord, T_ROBOT_COORS& tSplitCoord, int nSplitAxisNo)
{
	switch (nSplitAxisNo)
	{
	case 1:
		tSplitCoord.dBX = tSplitCoord.dX - tRefCoord.dX;
		tSplitCoord.dX = tRefCoord.dX;
		break;
	case 2:
		tSplitCoord.dBY = tSplitCoord.dY - tRefCoord.dY;
		tSplitCoord.dY = tRefCoord.dY;
		break;
	case 3:
		tSplitCoord.dBZ = tSplitCoord.dZ - tRefCoord.dZ;
		tSplitCoord.dZ = tRefCoord.dZ;
		break;
	default:
		tSplitCoord.dX -= tRefCoord.dBX;
		tSplitCoord.dY -= tRefCoord.dBY;
		tSplitCoord.dZ -= tRefCoord.dBZ;
		tSplitCoord.dBX = tRefCoord.dBX;
		tSplitCoord.dBY = tRefCoord.dBY;
		tSplitCoord.dBZ = tRefCoord.dBZ;
		break;
	}
}

bool CScanInitModule::JudgeFindWeldLineEnd(CRobotDriverAdaptor* pRobotCtrl, int nMethod)
{
	// �жϷ���: 1.��֪�յ�(�����յ�) 2.�����۳���(����) 3.���ӹ������ж�(�����յ�)
	bool bRst = false;
	switch (nMethod)
	{
	case 1: /*bRst = JudgeEnd_Know(pRobotCtrl);*/ break;
	case 2: bRst = JudgeEnd_TheoryLength(pRobotCtrl); break;
	case 3: bRst = JudgeEnd_ProcessSearch(pRobotCtrl); break;
	default:XUI::MesBox::PopInfo("�����ڸ����յ��жϷ���!");break;
	}
	return bRst;
}

bool CScanInitModule::JudgeEnd_Know(const std::vector<Three_DPoint>& vtPtns, T_ROBOT_COORS tTheoryEndCoord)
{
	bool bFond = false;
	double dDisThreshold = 10.0;
	Three_DPoint tNewPtn = vtPtns.back();
	double dDis = TwoPointDis(tNewPtn.x, tNewPtn.y, tNewPtn.z,
		tTheoryEndCoord.dX + tTheoryEndCoord.dBX,
		tTheoryEndCoord.dY + tTheoryEndCoord.dBY,
		tTheoryEndCoord.dZ + tTheoryEndCoord.dBZ);
	if (dDis < dDisThreshold) // ��Ҫ�Ż� ���ӹ����б��� Ӧ�����۶˵�����ٹ켣ֱ����ͶӰ�� ������룡������������������������
	{
		bFond = true;
		// �����յ�
		m_pTraceModel->m_bCameraFindWeldEnd = true;
	}
	return bFond;
}

bool CScanInitModule::JudgeEnd_TheoryLength(CRobotDriverAdaptor* pRobotCtrl)
{	
	//! ע��˺��� ���ڷ����е�� ��Ҫʹ��vtCoutTrackPoint�޸ĺ��ӹ켣������ ����˵㲻׼(���ַ�ʽ������޷�����׼ȷ)
	bool bIsFindEnd = false;
	double dOverLength = 999999.999;
	std::vector<TrackFilter_Node> vtCoutTrackPoint(0);
	bool bNormalDir = (1 == pRobotCtrl->m_nRobotInstallDir) ? false : true;
	//! �ж��Ƿ���������ƶ�����, return true : ����ƶ�, ����δ���.- overlength : �ṩһ��doubleָ��, ��ȡʣ�೤��.
	bool bIsOver = TrackFilter_DoneMove(m_ptUnit->m_nRobotSmooth, &dOverLength);
	if (bIsOver)
	{
		bIsFindEnd = true;
		//! �� TrackFilter_DoneMove() == trueʱ�����øú�������ȡֱ�����۳��Ƚ��� ʱ�Ĺ켣�㼯.
		//!  ���� overlength �㹻Сʱ, ���øú���, ǿ�ƻ�ȡ����β��. ��ʱ��ȡ�Ľ�β��Ϊֱ���ӳ�.
		TrackFilter_Answer ans = TrackFilter_GetToEndPnts(m_ptUnit->m_nRobotSmooth, bNormalDir);

		size_t index = 0;
		for (index = 0; index < ans.araaySize_; ++index) {
			vtCoutTrackPoint.push_back(ans.segmentArray_[index]);
			pRobotCtrl->m_cLog->Write("[�յ��ж�]:���۳����жϽ�βPtn%d: %11.3lf%11.3lf%11.3lf", index,
				ans.segmentArray_[index].pnt_.x_, ans.segmentArray_[index].pnt_.y_, ans.segmentArray_[index].pnt_.z_);
		}
		// �����յ�
		m_pTraceModel->m_tRealEndpointCoor = T_ROBOT_COORS(
			ans.segmentArray_[ans.araaySize_ - 1].pnt_.x_,
			ans.segmentArray_[ans.araaySize_ - 1].pnt_.y_,
			ans.segmentArray_[ans.araaySize_ - 1].pnt_.z_, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
		TrackFilter_Free(ans);
		// �����յ�
		m_pTraceModel->m_bCameraFindWeldEnd = TRUE; 
		pRobotCtrl->m_cLog->Write("[�յ��ж�]:���۳����жϽ�β���ﵽ��β�㣺%d", vtCoutTrackPoint.size());
	}
	pRobotCtrl->m_cLog->Write("[�յ��ж�]:�Ƿ񵽴��β���۳��ȣ�%d ���β���۳��ȣ�%lf ", bIsOver, dOverLength);
	return bIsFindEnd;
}

bool CScanInitModule::JudgeEnd_ProcessSearch(CRobotDriverAdaptor* pRobotCtrl)
{
	// ��ʱ������ȫ�̿���������һ��? ���������յ��㹻�̾��룿
	// ȫ�̿����ĺô������ٴ���ʧ�ܺ� ����ʹ�����µ��ƴ���Ľ��
	if (true == m_IsOpenJudgeEnd_ProcessSearch)
	{
		return true;
	}
	AfxBeginThread(ThreadJudgeEnd_ProcessSearch, this);
	return true;
}

UINT CScanInitModule::ThreadJudgeEnd_ProcessSearch(void* pParam)
{
	CScanInitModule* pObj = (CScanInitModule*)pParam;
	if (pObj->m_IsOpenJudgeEnd_ProcessSearch)
	{
		return 0;
	}
	pObj->m_IsOpenJudgeEnd_ProcessSearch = true;
	pObj->m_pRobotDriver->m_cLog->Write("FuncJudgeEnd_ProcessSearch Start");
	pObj->FuncJudgeEnd_ProcessSearch(pObj->m_pRobotDriver, 5000);
	pObj->m_pRobotDriver->m_cLog->Write("FuncJudgeEnd_ProcessSearch End");
	pObj->m_IsOpenJudgeEnd_ProcessSearch = false;
	return 0;
}

bool CScanInitModule::FuncJudgeEnd_ProcessSearch(CRobotDriverAdaptor* pRobotCtrl, int nTimeOut)
{
	double dSameDisThreshold = 1.0;		// �յ���ͬ������ֵ�ж� ���Ӿ����� ���ȸ���Ҫ��С����ֵ ��֮��Ҫ����
	int nSameEndPtnNumThreshold = PARA_FLAT_TRACK(nSameEndPtnNumThreshold);	// ���¶˵�ʹ�ǰnSamePtnNumThreshold�������С��dSameDisThreshold �����յ� 
	int nMinProcessImageNum = 60;		// ���ٶ�����ͼ�ľֲ����ƲŴ���һ��
	double dProcessSampDis = 2.0;		// ������ȡ����е���
	int nWaitTimeStep = 100;			// �ȴ��˶����ʱ�䲽��
	bool bIsRunning = false;
	int nPreImageNoE = 0;
	int nErrorNum = 0;					// ��������������
	int nErrorThreshold = 3;			// ��������������ν�������
	bool bResult = true;				// ���ؽ��
	std::vector<Three_DPoint> vtThreePoint(0);
	CString sTempFileName;

	// �ȴ������˶�
	int nCurWaitTime = 0;
	bIsRunning = m_ptUnit->WorldIsRunning();
	while (!bIsRunning)
	{
		Sleep(nWaitTimeStep);
		nCurWaitTime += nWaitTimeStep;
		if (nCurWaitTime > nTimeOut)
		{
			pRobotCtrl->m_cLog->Write("[�����Ѷ˵�]: �յ��жϵȴ��˶���ʱ��");
			return false;
		}
		bIsRunning = m_ptUnit->WorldIsRunning();
	}

	// �˶��� �� δ�����յ�
	FILE* pfPointCloudInput = fopen(OUTPUT_PATH + pRobotCtrl->m_strRobotName + WELDDATA_PATH + "RecordPointCloud_Input.txt", "w");
	CheckFolder(OUTPUT_PATH + pRobotCtrl->m_strRobotName + WELDDATA_PATH);
	while (bIsRunning)
	{
		m_MutexTrackPointCloud.lock();
		int nCurImageNum = m_pTraceModel->lnImageNo.size();
		if (nCurImageNum < nMinProcessImageNum)
		{
			m_MutexTrackPointCloud.unlock();
			Sleep(nWaitTimeStep);
			bIsRunning = m_ptUnit->WorldIsRunning();
			continue;
		}
		long long lTime = XI_clock();

		// ���⿽������������
		int nPointCloudImageNum = m_pTraceModel->lnImageNo.size();
		int nImageNoS = m_pTraceModel->lnImageNo.front();
		int nImageNoE = m_pTraceModel->lnImageNo.back();
		int nTotalPtnNum = m_pTraceModel->ltPointCloud.size();
		Three_DPoint* pThreeDPoint = new Three_DPoint[nTotalPtnNum];
		//std::unique_ptr<Three_DPoint[]> pThreeDPoint(new Three_DPoint[nTotalPtnNum]);
		int nPtnNum = 0;
		if (m_pTraceModel->ltPointCloud.size() <= 0)
		{
			continue;
		}
		for (const auto& point : m_pTraceModel->ltPointCloud) 
		{
			pThreeDPoint[nPtnNum++] = point;
		}
		/*std::list<Three_DPoint>::iterator iter = m_pTraceModel->ltPointCloud.begin();
		int nPtnNum = 0;
		for (; iter != m_pTraceModel->ltPointCloud.end(); iter++)
		{
			pThreeDPoint[nPtnNum++] = *(iter);
		}*/
		T_ROBOT_COORS tScanS = m_pTraceModel->ltCapWorldCoord.front();
		T_ROBOT_COORS tScanE = m_pTraceModel->ltCapWorldCoord.back();
		m_MutexTrackPointCloud.unlock();

		if (nImageNoE <= nPreImageNoE) // ÿ����һ��ͼ���ֻ����һ�ε���
		{
			DELETE_POINTER_ARRAY(pThreeDPoint);
			Sleep(nWaitTimeStep);
			continue;
		}
		nPreImageNoE = nImageNoE;

		// ׼������ֲ�������������
		CvPoint3D32f tRefPtn[2] = { 0.0 };
		//�ο������ɨ�跽���Ϊ���ۺ�������
		//tRefPtn[0] = cvPoint3D32f(tScanS.dX + tScanS.dBX, tScanS.dY + tScanS.dBY, tScanS.dZ);
		//tRefPtn[1] = cvPoint3D32f(tScanE.dX + tScanE.dBX, tScanE.dY + tScanE.dBY, tScanE.dZ);
		T_ROBOT_COORS tRealScanE, tRealScanS;
		if (!pRobotCtrl->MoveToolByWeldGun(tScanE, pRobotCtrl->m_tTools.tCameraTool, tScanE, pRobotCtrl->m_tTools.tGunTool, tRealScanE)
			|| !pRobotCtrl->MoveToolByWeldGun(tScanS, pRobotCtrl->m_tTools.tCameraTool, tScanS, pRobotCtrl->m_tTools.tGunTool, tRealScanS))
		{
			pRobotCtrl->m_cLog->Write("ʹ�þɷ�������ο����򣬿���������");
			tRefPtn[0] = cvPoint3D32f(tScanS.dX + tScanS.dBX, tScanS.dY + tScanS.dBY, tScanS.dZ);
			tRefPtn[1] = cvPoint3D32f(tScanE.dX + tScanE.dBX, tScanE.dY + tScanE.dBY, tScanE.dZ);
		}
		else
		{
			tRefPtn[0] = cvPoint3D32f(tRealScanS.dX + tRealScanS.dBX, tRealScanS.dY + tRealScanS.dBY, tRealScanS.dZ);
			tRefPtn[1] = cvPoint3D32f(tRealScanE.dX + tRealScanE.dBX, tRealScanE.dY + tRealScanE.dBY, tRealScanE.dZ);
		}
		double dDirAngle = m_ptUnit->GetRobotCtrl()->RzToDirAngle(tScanE.dRZ);
		CvPoint3D32f tCameraNorm = cvPoint3D32f(-CosD(dDirAngle), -SinD(dDirAngle), m_ptUnit->GetRobotCtrl()->m_nRobotInstallDir > 0.0 ? -1.0 : 1.0);
		CvPoint3D32f tPlaneHNorm = cvPoint3D32f(0.0, 0.0, m_ptUnit->GetRobotCtrl()->m_nRobotInstallDir > 0.0 ? 1.0 : -1.0);
		RiserEndPointInfo tWeldInfo[3] = { 0 };

		// ����ÿ�δ�����������
		fprintf(pfPointCloudInput, "nImageNoS:%d nImageNoE:%d tCameraNorm:%11.3lf%11.3lf%11.3lf tPlaneHNorm:%11.3lf%11.3lf%11.3lf tRefPtn[0]:%11.3lf%11.3lf%11.3lf tRefPtn[1]:%11.3lf%11.3lf%11.3lf\n",
			nImageNoS, nImageNoE, tCameraNorm.x, tCameraNorm.y, tCameraNorm.z, tPlaneHNorm.x, tPlaneHNorm.y, tPlaneHNorm.z,
			tRefPtn[0].x, tRefPtn[0].y, tRefPtn[0].z, tRefPtn[1].x, tRefPtn[1].y, tRefPtn[1].z);
		fflush(pfPointCloudInput);
		
		try
		{
			// ���ƴ���
			int nWeldInfoNum = GetRiserEndPoint((CvPoint3D64f*)pThreeDPoint, nPtnNum, tWeldInfo, tCameraNorm, tPlaneHNorm, tRefPtn, 2, "GetRiserEndPoint");

			double dDis = TwoPointDis(tWeldInfo[0].staPnt.x, tWeldInfo[0].staPnt.y, tWeldInfo[0].staPnt.z, tWeldInfo[0].endPnt.x, tWeldInfo[0].endPnt.y, tWeldInfo[0].endPnt.z);
			double dDisX = tWeldInfo[0].endPnt.x - tWeldInfo[0].staPnt.x;
			double dDisY = tWeldInfo[0].endPnt.y - tWeldInfo[0].staPnt.y;
			double dDisZ = tWeldInfo[0].endPnt.z - tWeldInfo[0].staPnt.z;

			// �����к����� || ��������յ㲻��� || ��������������������ֵ ����ʧ�ܴ�������1
			if ((0 >= nWeldInfoNum) || (dDis < dProcessSampDis) || (nErrorNum >= nErrorThreshold))
			{
				pRobotCtrl->m_cLog->Write("[�����Ѷ˵�]: ���ƶ˵���ȡʧ�ܣ�ImgNo_S%d-E%d", nImageNoS, nImageNoE);
				sTempFileName.Format("%sImgNo_S%d-E%d-����ʧ�ܵ���.txt", OUTPUT_PATH + pRobotCtrl->m_strRobotName + ERROR_PATH, nImageNoS, nImageNoE);
				SavePointCloudProcErrData(sTempFileName, nImageNoS, nImageNoE, tCameraNorm, tPlaneHNorm, tRefPtn, 2, pThreeDPoint, nPtnNum);
				DELETE_POINTER_ARRAY(pThreeDPoint);
				nErrorNum++;
				continue;
			}
			if (nErrorNum > nErrorThreshold) // ����ʧ�ܶ��
			{
				DELETE_POINTER_ARRAY(pThreeDPoint);
				CheckMachineEmg(pRobotCtrl);
				XUI::MesBox::PopOkCancel("[�����Ѷ˵�]:GetRiserEndPoint�޽��! ͼ��{0}-{1}", nImageNoS, nImageNoE);
				fclose(pfPointCloudInput);
				bResult = false;
				break;
			}
			else
			{
				nErrorNum = 0;
				int nNo = (int)(dDis / dProcessSampDis);
				double dStepDis = dDis / (double)nNo;
				vtThreePoint.resize(nNo + 1);
				for (int i = 0; i <= nNo; i++)
				{
					vtThreePoint[i].x = tWeldInfo[0].staPnt.x + dDisX * i / nNo;
					vtThreePoint[i].y = tWeldInfo[0].staPnt.y + dDisY * i / nNo;
					vtThreePoint[i].z = tWeldInfo[0].staPnt.z + dDisZ * i / nNo;
				}
			}
		}
		catch (...)
		{
			// �쳣���� �˴����Բ�����������������ʧ�ܴ�������
			sTempFileName.Format("%sImgNo_S%d-E%d_�����쳣����.txt",
				OUTPUT_PATH + pRobotCtrl->m_strRobotName + ERROR_PATH, nImageNoS, nImageNoE);
			SavePointCloudProcErrData(sTempFileName, nImageNoS, nImageNoE, tCameraNorm, tPlaneHNorm, tRefPtn, 2, pThreeDPoint, nPtnNum);
			fclose(pfPointCloudInput);
			CheckMachineEmg(pRobotCtrl);
			//���޸�
			XUI::MesBox::PopInfo("{0} [�����Ѷ˵�]:���ƴ����쳣! ͼ��{1}-{2}", pRobotCtrl->m_strRobotName.GetBuffer(), nImageNoS, nImageNoE);
			//XiMessageBox("%s [�����Ѷ˵�]:���ƴ����쳣! ͼ��%d-%d", pRobotCtrl->m_strRobotName, nImageNoS, nImageNoE);
			bResult = false;
			DELETE_POINTER_ARRAY(pThreeDPoint);
			break;
		}

		// ���浥�δ�����
		sTempFileName.Format("%sImgNo_S%d-E%d_Result.txt",
			OUTPUT_PATH + pRobotCtrl->m_strRobotName + WELDDATA_PATH, nImageNoS, nImageNoE, vtThreePoint.size());
		FILE* pf = fopen(sTempFileName.GetBuffer(), "w");
		for (int i = 0; i < vtThreePoint.size(); i++)
		{
			fprintf(pf, "��ʼͼ��%d ����ͼ��%d %11.3lf%11.3lf%11.3lf\n",
				nImageNoS, nImageNoE, vtThreePoint[i].x, vtThreePoint[i].y, vtThreePoint[i].z);
		}
		fclose(pf);

		Three_DPoint tNewEndPtn = vtThreePoint.back();
		if (m_pTraceModel->vtPointCloudEndPoints.size() <= 0) // ��һ�α������д�����
		{
			for (int i = 0; i < vtThreePoint.size(); i++)
			{
				m_pTraceModel->vtPointCloudEndPoints.push_back(vtThreePoint[i]);
			}
		}
		else // ��������ֻ�������һ����
		{
			m_pTraceModel->vtPointCloudEndPoints.push_back(tNewEndPtn);
		}

		pRobotCtrl->m_cLog->Write("[�����Ѷ˵�]: ���ƶ˵���ȡ�յ㣺ImgNo_S%d-E%d_Result%d %11.3lf%11.3lf%11.3lf ��ʱ:%dms",
			nImageNoS, nImageNoE, vtThreePoint.size(), tNewEndPtn.x, tNewEndPtn.y, tNewEndPtn.z, XI_clock() - lTime);

		DELETE_POINTER_ARRAY(pThreeDPoint);

		// �ж��յ�
		if (m_bDynamicJudgeEndPoint)
		{
			if ((E_JUDGE_END_PROCESS_SEARCH == m_eJudgeEndMethod) && (JudgeFindEndPoint(m_pTraceModel->vtPointCloudEndPoints, dSameDisThreshold, nSameEndPtnNumThreshold)))
			{
				pRobotCtrl->m_cLog->Write("[�����Ѷ˵�]: JudgeFindEndPoint �����յ� E_JUDGE_END_PROCESS_SEARCH");
				break;
			}
			
			if (E_JUDGE_END_KNOW == m_eJudgeEndMethod && JudgeEnd_Know(m_pTraceModel->vtPointCloudEndPoints, m_pTraceModel->m_tRealEndpointCoor))
			{
				pRobotCtrl->m_cLog->Write("[�����Ѷ˵�]: JudgeFindEndPoint �����յ� E_JUDGE_END_KNOW");
				break;
			}
		}

		bIsRunning = m_ptUnit->WorldIsRunning();
	}
	fclose(pfPointCloudInput);
	
	if (!bIsRunning)
	{
		pRobotCtrl->m_cLog->Write("[�����Ѷ˵�]: �˶��쳣ֹͣ���յ��жϽ���!");
	}
	else
	{
		pRobotCtrl->m_cLog->Write("[�����Ѷ˵�]: �յ��ж���������!");
	}
	return bResult;
}

bool CScanInitModule::GetEndSectionCoorsNew(CRobotDriverAdaptor* pRobotDriver, T_ROBOT_COORS tEndpointCoor)
{
	// ��������ע�� tEndpointCoorΪ�յ㺸���������� ��XYZ��ֵ ����ֵ��0
	double dEndToLastDisThreshold = 130.0; // �յ�������¸��ٹ켣�������ֵ

	int nSize = pRobotDriver->m_vtWeldLineInWorldPoints.size();
	T_ROBOT_COORS tTrackLastCoord = pRobotDriver->m_vtWeldLineInWorldPoints[nSize - 1];
	T_ROBOT_COORS tEndpointTemp = tEndpointCoor;

	pRobotDriver->m_cLog->Write("[�յ㴦��]: �յ㣺%.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf ���켣�㣺%.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf",
		tEndpointCoor.dX, tEndpointCoor.dY, tEndpointCoor.dZ, 
		tEndpointCoor.dRX, tEndpointCoor.dRY, tEndpointCoor.dRZ, 
		tEndpointCoor.dBX, tEndpointCoor.dBY, tEndpointCoor.dBZ,
		tTrackLastCoord.dX, tTrackLastCoord.dY, tTrackLastCoord.dZ, 
		tTrackLastCoord.dRX, tTrackLastCoord.dRY, tTrackLastCoord.dRZ, 
		tTrackLastCoord.dBX, tTrackLastCoord.dBY, tTrackLastCoord.dBZ);

	// ȡ���漸������Ϻ����յ㸽��ֱ��
	vector<XI_POINT> vtPoint;
	for (int n = nSize - 10; n < nSize; n++)
	{
		XI_POINT tp = { 
			pRobotDriver->m_vtWeldLineInWorldPoints[n].dX + pRobotDriver->m_vtWeldLineInWorldPoints[n].dBX,
			pRobotDriver->m_vtWeldLineInWorldPoints[n].dY + pRobotDriver->m_vtWeldLineInWorldPoints[n].dBY,
			pRobotDriver->m_vtWeldLineInWorldPoints[n].dZ + pRobotDriver->m_vtWeldLineInWorldPoints[n].dBZ };
		vtPoint.push_back(tp);
	}
	T_LINE_PARA	tLine;
	if (!CalcLineParamRansac(tLine,vtPoint, 0.7))
	{
		return false;
	}
	// �ж��յ�����һ�����ٹ켣������Ƿ���
	double dDisError = TwoPointDis(
		tEndpointCoor.dX + tEndpointCoor.dBX, tEndpointCoor.dY + tEndpointCoor.dBY, tEndpointCoor.dZ + tEndpointCoor.dBZ,
		tTrackLastCoord.dX + tTrackLastCoord.dBX, tTrackLastCoord.dY + tTrackLastCoord.dBY, tTrackLastCoord.dZ + tTrackLastCoord.dBZ);
	if (dDisError > dEndToLastDisThreshold)
	{
		tEndpointCoor = pRobotDriver->m_vtWeldLineInWorldPoints.at(nSize - 1);
		//����ǰֹͣ�򲻱��Ǻ���������ֹͣ���ӣ�����ʾ
		//m_pTraceModel->m_eWrapAngleType = E_WRAPANGLE_EMPTY_SINGLE;
		//SetIntValFun(pRobotDriver, 7, 0);
		pRobotDriver->m_cLog->Write(" ��β���ж�ʧ������һ��Ϊ��β��ֹͣ�豸����������������Ϊ�����ǣ�%lf ", dDisError);
		m_pTraceModel->m_tRealEndpointCoor = tEndpointCoor;
		return false;
	}
	// ��ʼ����β���˲�����
	TrackSmooth_Init(m_ptUnit->m_nRobotSmooth, m_pTraceModel->m_dMoveStepDis, tTrackLastCoord.dX + tTrackLastCoord.dBX, 
		tTrackLastCoord.dY + tTrackLastCoord.dBY, tTrackLastCoord.dZ + tTrackLastCoord.dBZ);

	XI_POINT tEndpoint = { tEndpointCoor.dX + tEndpointCoor.dBX, tEndpointCoor.dY + tEndpointCoor.dBY, tEndpointCoor.dZ + tTrackLastCoord.dBZ };
	// ͶӰ
	XI_POINT tProjectpoint;
	PointtoLineProjection(tLine, tEndpoint, tProjectpoint);
	tEndpointCoor.dX = tProjectpoint.x;
	tEndpointCoor.dY = tProjectpoint.y;
	tEndpointCoor.dZ = tProjectpoint.z;
	pRobotDriver->m_cLog->Write("[�յ㴦��]: ��β��ͶӰ:%11.3lf%11.3lf%11.3lf", tEndpointCoor.dX, tEndpointCoor.dY, tEndpointCoor.dZ);

	// ��ȡ��β������ �����˲��� �����յ� ȡ�յ� Start  ���㷨�յ��ȡ������Ҫ�޸ģ�
	T_ROBOT_COORS tTempCoors;
	TrackSmooth_PntArray tTrackPtnArray;
	tTrackPtnArray.arraySize_ = 0;
	tTrackPtnArray.pnts_ = NULL;
	TrackSmooth_PushNextPoint(m_ptUnit->m_nRobotSmooth, tEndpointCoor.dX + tEndpointCoor.dBX,
		tEndpointCoor.dY + tEndpointCoor.dBY, tEndpointCoor.dZ + tEndpointCoor.dBZ, &tTrackPtnArray);
	for (int n = 0; n < tTrackPtnArray.arraySize_; n++)
	{
		tTempCoors = T_ROBOT_COORS(tTrackPtnArray.pnts_[n].x_, tTrackPtnArray.pnts_[n].y_, tTrackPtnArray.pnts_[n].z_,
			tTrackLastCoord.dRX, tTrackLastCoord.dRY, tTrackLastCoord.dRZ, 0.0, 0.0, 0.0);
		SplitCoord(tTrackLastCoord, tTempCoors, m_ptUnit->m_nTrackAxisNo); // ������굽�ⲿ��
		pRobotDriver->m_vtWeldLineInWorldPoints.push_back(tTempCoors);
		m_pTraceModel->m_vtWeldLinePointType.push_back(E_WELD_TRACK);
		//SaveRobotCoor(m_pRecordTheoryPoint, tTempCoors, E_WELD_TRACK);//������������
		pRobotDriver->m_cLog->Write("[�յ㴦��]: TrackSmooth_PushNextPoint: %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf",
			tTempCoors.dX, tTempCoors.dY, tTempCoors.dZ, tTempCoors.dRX, tTempCoors.dRY, tTempCoors.dRZ, tTempCoors.dBX, tTempCoors.dBY, tTempCoors.dBZ);
	}
	TrackSmooth_FreePntArray(&tTrackPtnArray);

	//��ȡ��β�㣬����βʱ���ã�����һ��ľ���С�� moveMinDis 
	TrackSmooth_GetEndPoint(m_ptUnit->m_nRobotSmooth, &tTrackPtnArray);
	for (int n = 0; n < tTrackPtnArray.arraySize_; n++)
	{
		tTempCoors = T_ROBOT_COORS(tTrackPtnArray.pnts_[n].x_, tTrackPtnArray.pnts_[n].y_, tTrackPtnArray.pnts_[n].z_,
			tTrackLastCoord.dRX, tTrackLastCoord.dRY, tTrackLastCoord.dRZ, 0.0, 0.0, 0.0);
		SplitCoord(tTrackLastCoord, tTempCoors, m_ptUnit->m_nTrackAxisNo); // ������굽�ⲿ��
		pRobotDriver->m_vtWeldLineInWorldPoints.push_back(tTempCoors);
		m_pTraceModel->m_vtWeldLinePointType.push_back(E_WELD_TRACK);
		//SaveRobotCoor(m_pRecordTheoryPoint, tTempCoors, E_WELD_TRACK);//������������
		pRobotDriver->m_cLog->Write("[�յ㴦��]: TrackSmooth_GetEndPoint: %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf",
			tTempCoors.dX, tTempCoors.dY, tTempCoors.dZ, tTempCoors.dRX, tTempCoors.dRY, tTempCoors.dRZ, tTempCoors.dBX, tTempCoors.dBY, tTempCoors.dBZ);
	}
	TrackSmooth_FreePntArray(&tTrackPtnArray);

	//���½�β����
	m_pTraceModel->m_tRealEndpointCoor = pRobotDriver->m_vtWeldLineInWorldPoints.back();
	return true;
}


void CScanInitModule::SavePointCloudProcErrData(CString sFileName, int nImgNoS, int nImgNoE, CvPoint3D32f tCameraNorm, CvPoint3D32f
	tPlaneHNorm, CvPoint3D32f* tRefPtn, int tRefPtnNum, Three_DPoint* ptPointCloud, int nPointCloudNum)
{
	FILE* pf = fopen(sFileName.GetBuffer(), "w");
	fprintf(pf, "nImageNoS:%d nImageNoE:%d tCameraNorm:%11.3lf%11.3lf%11.3lf tPlaneHNorm:%11.3lf%11.3lf%11.3lf tRefPtn[0]:%11.3lf%11.3lf%11.3lf tRefPtn[1]:%11.3lf%11.3lf%11.3lf\n",
		nImgNoS, nImgNoE, tCameraNorm.x, tCameraNorm.y, tCameraNorm.z, tPlaneHNorm.x, tPlaneHNorm.y, tPlaneHNorm.z,
		tRefPtn[0].x, tRefPtn[0].y, tRefPtn[0].z, tRefPtn[1].x, tRefPtn[1].y, tRefPtn[1].z);
	for (int i = 0; i < nPointCloudNum; i++)
	{
		fprintf(pf, "%d%11.3lf%11.3lf%11.3lf\n", i, ptPointCloud[i].x, ptPointCloud[i].y, ptPointCloud[i].z);
	}
	fclose(pf);
}

bool CScanInitModule::JudgeFindEndPoint(std::vector<Three_DPoint>& vtPtns, double dDisThreshold, double nPtnNumThreshold)
{
	bool bFrond = false;
	int nSameEndPtnNum = 0;
	int nEndPtnNum = vtPtns.size();
	Three_DPoint tNewEndPtn = vtPtns.back();
	Three_DPoint tOldEndPtn;
	for (int i = nEndPtnNum - 2; i >= 0; i--)
	{
		tOldEndPtn = vtPtns[i];
		double dDis = TwoPointDis(tNewEndPtn.x, tNewEndPtn.y, tNewEndPtn.z, tOldEndPtn.x, tOldEndPtn.y, tOldEndPtn.z);
		if (dDis < dDisThreshold)
		{
			nSameEndPtnNum++;
			if (nSameEndPtnNum >= nPtnNumThreshold)
			{
				m_pRobotDriver->m_cLog->Write("[�����Ѷ˵�]: �����յ� %11.3lf%11.3lf%11.3lf", tNewEndPtn.x, tNewEndPtn.y, tNewEndPtn.z);

				// �����յ� ����յ㴦��ȡ�˵㲨������Ҫȡ��������յ�ƽ����
				m_pTraceModel->m_tRealEndpointCoor = T_ROBOT_COORS(tNewEndPtn.x, tNewEndPtn.y, tNewEndPtn.z, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
				// �����յ�
				m_pTraceModel->m_bCameraFindWeldEnd = true;
				bFrond = true;
				break;
			}
		}
		else
		{
			break;
		}
	}
	return bFrond;
}

void CScanInitModule::DynamicCaptureNew_ScanWeldLine(CRobotDriverAdaptor* pRobotDriver, int nCameraNo, CString sPointCloudFileName, int nTimeOut/* = 2000*/)
{
	auto dPointCloudLengthForFindEndpnt = PARA_FLAT_MEASURE(dPointCloudLengthForFindEndpnt);
	bool bDynamicProc = E_POINT_CLOUD_PROC_MOMENT_DYNAMIC == m_ePointCloudProcMoment ? true : false; // ʵʱ���� �� ɨ������괦��
	long lStartTime = 0;
	int nCapImageNo = 0;
	T_ROBOT_COORS tRobotCoord; //��ͼʱ��ֱ������
	T_ANGLE_PULSE tRobotPulse; //��ͼʱ�Ĺؽ�����
	vector<T_ROBOT_COORS> t;
//	T_ABS_POS_IN_BASE tPointAbsCoordInBase;

	CDHGigeImageCapture* pDHCamDrv = (CDHGigeImageCapture*)m_ptUnit->GetCameraCtrl(nCameraNo);
	IplImage* pImageBuf = NULL;
	T_CAMREA_PARAM tCameraParam = m_ptUnit->GetCameraParam(nCameraNo);
	int nMinX = 0;
	int nMaxX = tCameraParam.tDHCameraDriverPara.nRoiWidth;// *3 / 4;
	std::vector<CvPoint> vtPoint2D(0);
	FILE* pfPointCloud = fopen(sPointCloudFileName.GetBuffer(), "w");
	m_sPointCloudFileName = sPointCloudFileName;

	m_pTraceModel->lnImageNo.clear();	// ��ǰ���Ƽ�����ÿ��ͼ���
	m_pTraceModel->lnImagePtnNum.clear();	// ��ǰ���Ƽ�����ÿ��ͼ����
	m_pTraceModel->ltPointCloud.clear(); // ��ǰ���Ƽ���
	m_pTraceModel->ltCapWorldCoord.clear(); // ��ǰ���Ƽ�����ÿ��ͼ�ɼ�ʱ��������
	m_pTraceModel->vtPointCloudEndPoints.clear(); // ÿ�ε��ƴ�����ȡ�Ķ˵���������
	m_pTraceModel->vtConnerWorldCoords.clear(); // ÿ��ͼ����ǵ���������
	m_pTraceModel->m_bCameraFindWeldEnd = false;
	m_IsOpenJudgeEnd_ProcessSearch = false;

	if (bDynamicProc)
	{
		AfxBeginThread(ThreadJudgeEnd_ProcessSearch, this);
	}

	if (m_bInsideCallJob)
	{
		pRobotDriver->CallJob(m_sMoveJobName);
	}

	// �ȴ�ɨ���˶�
	int nCurWaitTime = 0;
	int nWaitTimeStep = 100;
	while (!m_ptUnit->WorldIsRunning())
	{
		Sleep(nWaitTimeStep);
		nCurWaitTime += nWaitTimeStep;
		if (nCurWaitTime > nTimeOut)
		{
			pRobotDriver->m_cLog->Write("[�����Ѷ˵�]: �ȴ��˶���ʱ��");
			return;
		}
	}

	int i = 0;

	// �����˶��� �� δ�����յ�
	while (m_ptUnit->WorldIsRunning())
	{
		if (bDynamicProc && true != m_IsOpenJudgeEnd_ProcessSearch) // ʵʱ���� �� ����ر�  ����
		{
			break;
		}
		lStartTime = XI_clock();

		// ������
		tRobotCoord = pRobotDriver->GetCurrentPos();
		tRobotPulse = pRobotDriver->GetCurrentPulse();
		t.push_back(tRobotCoord);
		long long lTime1 = XI_clock();

		// ��ͼ
		//if (!pDHCamDrv->CaptureImage(pImageBuf, 1))
		if (!pDHCamDrv->CaptureImage(1))
		{
			pRobotDriver->m_cLog->Write("DynamicCaptureNew ��ͼʧ�ܣ�");
			return;
		}

		long long lTime1_1 = XI_clock();
		pImageBuf = pDHCamDrv->m_tLatestImage.pImage;
		cvCvtColor(pImageBuf, m_pShowImage, CV_GRAY2RGB);

		char szName[56] = { 0 };
		sprintf(szName, ".\\LocalFiles\\OutputFiles\\RobotA\\EndSrc\\%d.bin", i++);//���ñ���·��

		long long lTime1_2 = XI_clock();
		//BinaryImgSave(szName, pImageBuf);
		/*cvSaveImage(szName, pImageBuf);*/


		long long lTime2 = XI_clock();

		// ����
		switch (m_eImgProcMethod)
		{
		case E_IMAGE_PROC_METHOD_EEEEE:
			CHECK_BOOL(FuncImageProcessEEEEE(pImageBuf, vtPoint2D));
			break;
		case E_IMAGE_PROC_METHOD_EEEEEInTrack:
			CHECK_BOOL(FuncImageProcessInTrack(pImageBuf, m_pTraceModel->tFrontLine, m_pTraceModel->tBackLine, vtPoint2D, tCameraParam.eFlipMode));
			break;
		case E_IMAGE_PROC_METHOD_DoubleVGroove:
			break;
		default:
			break;
		}

		// ת����
		T_ROBOT_COORS tPointCloudRobotCoord; //����ת���Ļ���������
		T_ROBOT_COORS tPointCloudWorldCoord; //����ת������������
		Three_DPoint tThreePoint;  //3D����?
		int nSaveNum = 0;
		m_MutexTrackPointCloud.lock();
		long long lTime3 = XI_clock();

		std::vector<T_ROBOT_COORS> vtPoint3D = m_ptUnit->TranImageToBase(nCameraNo, vtPoint2D, tRobotCoord, tRobotPulse);

		for (int i = 0; i < vtPoint2D.size(); i++)
		{
			if (vtPoint2D[i].x < nMinX || vtPoint2D[i].x > nMaxX) continue; // ��Ч����������ݲ�����
			//tPointCloudRobotCoord = m_ptUnit->TranImageToBase(nCameraNo, vtPoint2D[i], tRobotCoord, tRobotPulse, &tPointAbsCoordInBase);
			tPointCloudRobotCoord = vtPoint3D[i];
			tPointCloudWorldCoord = tPointCloudRobotCoord;

			tPointCloudWorldCoord.dX += tRobotCoord.dBX;
			tPointCloudWorldCoord.dY += tRobotCoord.dBY;
			tPointCloudWorldCoord.dZ += tRobotCoord.dBZ;
			tThreePoint.x = tPointCloudWorldCoord.dX;
			tThreePoint.y = tPointCloudWorldCoord.dY;
			tThreePoint.z = tPointCloudWorldCoord.dZ;
			m_pTraceModel->ltPointCloud.push_back(tThreePoint);
			nSaveNum++;
			fprintf(pfPointCloud, "%d%11.3lf%11.3lf%11.3lf\n", nCapImageNo, tPointCloudWorldCoord.dX, tPointCloudWorldCoord.dY, tPointCloudWorldCoord.dZ);
			fflush(pfPointCloud);
			if (0 == i) // ��һ����ÿ��ͼ����Ľǵ�
			{
				m_pTraceModel->vtConnerWorldCoords.push_back(tPointCloudWorldCoord);
			}
		}
		m_pTraceModel->lnImageNo.push_back(nCapImageNo);
		m_pTraceModel->lnImagePtnNum.push_back(nSaveNum);
		m_pTraceModel->ltCapWorldCoord.push_back(tRobotCoord);
		long long lTime4 = XI_clock();

		if (bDynamicProc)
		{
			// ɾ���ɵ�������
			for (int i = 0; (i < m_pTraceModel->ltCapWorldCoord.size()) && (m_pTraceModel->ltCapWorldCoord.size() > 60); i++)
			{
				T_ROBOT_COORS tCoordS = m_pTraceModel->ltCapWorldCoord.front();
				T_ROBOT_COORS tCoordE = m_pTraceModel->ltCapWorldCoord.back();
				double dDis = TwoPointDis(
					tCoordS.dX + tCoordS.dBX, tCoordS.dY + tCoordS.dBY, tCoordS.dZ + tCoordS.dBZ,
					tCoordE.dX + tCoordE.dBX, tCoordE.dY + tCoordE.dBY, tCoordE.dZ + tCoordE.dBZ);
				if (dDis > dPointCloudLengthForFindEndpnt)
				{
					int nDelPtnNum = m_pTraceModel->lnImagePtnNum.front();
					std::list<Three_DPoint>::iterator iter = m_pTraceModel->ltPointCloud.begin();
					std::advance(iter, nDelPtnNum);
					m_pTraceModel->ltPointCloud.erase(m_pTraceModel->ltPointCloud.begin(), iter);
					m_pTraceModel->ltCapWorldCoord.pop_front();
					m_pTraceModel->lnImagePtnNum.pop_front();
					m_pTraceModel->lnImageNo.pop_front();
				}
			}
		}

		long long lTime5 = XI_clock();
		m_MutexTrackPointCloud.unlock();

		pRobotDriver->m_cLog->Write("[�����Ѷ˵�]: ɨ���ͼ����%d�ܺ�ʱ:%dms �����ȡ:%d ͼ��ɼ�%d ͼ��ת��%d ͼ�񱣴�%d ͼ����:%d ����ת��:%d ����ɾ��:%d CurXYZ:%.3lf %.3lf %.3lf",
			nCapImageNo, XI_clock() - lStartTime, lTime1 - lStartTime, lTime1_1 - lTime1, lTime1_2 - lTime1_1, lTime2 - lTime1_2, lTime3 - lTime2, lTime4 - lTime3, lTime5 - lTime4,
			tRobotCoord.dX, tRobotCoord.dY, tRobotCoord.dZ);
		nCapImageNo++;
	}
	fclose(pfPointCloud);
	m_pRobotDriver->HoldOn();
	Sleep(200);
	m_pRobotDriver->HoldOff();
	/*FILE* f = fopen(".\\LocalFiles\\OutputFiles\\RobotA\\EndSrc\\test.txt", "w");
	for (int l = 0; l < t.size(); l++)
	{
		fprintf(f, "%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf\n",
			t[l].dX, t[l].dY, t[l].dZ, t[l].dRX, t[l].dRY, t[l].dRZ, t[l].dBX, t[l].dBY, t[l].dBZ
			, t[l].dX+t[l].dBX, t[l].dY + t[l].dBY, t[l].dZ);
	}
	fclose(f);*/
	return;
}

void CScanInitModule::DynamicCaptureNew(CRobotDriverAdaptor* pRobotDriver, int nCameraNo, CString sPointCloudFileName, int nTimeOut/* = 2000*/)
{
	auto dPointCloudLengthForFindEndpnt = PARA_FLAT_MEASURE(dPointCloudLengthForFindEndpnt);
	int nErrorTimes = 0;//����ʧ�ܵĴ���
	int nMaxErrorTimes = 10;//����ʧ�ܵĴ�������
	bool bDynamicProc = E_POINT_CLOUD_PROC_MOMENT_DYNAMIC == m_ePointCloudProcMoment ? true : false; // ʵʱ���� �� ɨ������괦��
	long lStartTime = 0;
	int nCapImageNo = 0;
	T_ROBOT_COORS tRobotCoord; //��ͼʱ��ֱ������
	T_ANGLE_PULSE tRobotPulse; //��ͼʱ�Ĺؽ�����
//	T_ABS_POS_IN_BASE tPointAbsCoordInBase;

	CDHGigeImageCapture* pDHCamDrv = (CDHGigeImageCapture*)m_ptUnit->GetCameraCtrl(nCameraNo);
	IplImage* pImageBuf = NULL;
	T_CAMREA_PARAM tCameraParam = m_ptUnit->GetCameraParam(nCameraNo);
	int nMinX = 0;
	int nMaxX = tCameraParam.tDHCameraDriverPara.nRoiWidth; //*2 / 3;
	std::vector<CvPoint> vtPoint2D(0);
	FILE* pfPointCloud = fopen(sPointCloudFileName.GetBuffer(), "w");
	m_sPointCloudFileName = sPointCloudFileName;

	m_pTraceModel->lnImageNo.clear();	// ��ǰ���Ƽ�����ÿ��ͼ���
	m_pTraceModel->lnImagePtnNum.clear();	// ��ǰ���Ƽ�����ÿ��ͼ����
	m_pTraceModel->ltPointCloud.clear(); // ��ǰ���Ƽ���
	m_pTraceModel->ltCapWorldCoord.clear(); // ��ǰ���Ƽ�����ÿ��ͼ�ɼ�ʱ��������
	m_pTraceModel->vtPointCloudEndPoints.clear(); // ÿ�ε��ƴ�����ȡ�Ķ˵���������
	m_pTraceModel->vtConnerWorldCoords.clear(); // ÿ��ͼ����ǵ���������
	m_pTraceModel->m_bCameraFindWeldEnd = false;
	m_IsOpenJudgeEnd_ProcessSearch = false;

	if (bDynamicProc)
	{
		AfxBeginThread(ThreadJudgeEnd_ProcessSearch, this);
	}

	if (m_bInsideCallJob)
	{
		pRobotDriver->CallJob(m_sMoveJobName);
	}

	// �ȴ�ɨ���˶�
	int nCurWaitTime = 0;
	int nWaitTimeStep = 100;
	while (!m_ptUnit->WorldIsRunning())
	{
		Sleep(nWaitTimeStep);
		nCurWaitTime += nWaitTimeStep;
		if (nCurWaitTime > nTimeOut)
		{
			pRobotDriver->m_cLog->Write("[�����Ѷ˵�]: �ȴ��˶���ʱ��");
			return;
		}
	}

	int i = 0;
	vector<T_ANGLE_PULSE> vt;
	// �����˶��� �� δ�����յ�
	while (m_ptUnit->WorldIsRunning())
	{
		if (bDynamicProc && false == m_IsOpenJudgeEnd_ProcessSearch) // ʵʱ���� �� ����ر�  ����
		{
			break;
		}
		lStartTime = XI_clock();

		// ������
		tRobotCoord = pRobotDriver->GetCurrentPos_ESTUN();
		if (tRobotCoord.dX == 0 && tRobotCoord.dY == 0 && tRobotCoord.dZ == 0 && tRobotCoord.dRY == 0)
		{
			tRobotCoord = pRobotDriver->GetCurrentPos();
		}
		pRobotDriver->RobotInverseKinematics(tRobotCoord, pRobotDriver->m_tTools.tGunTool, vt);
		tRobotPulse = vt[0]/*pRobotDriver->GetCurrentPulse_ESTUN()*/;
		long long lTime1 = XI_clock();

		// ��ͼ
		if (!pDHCamDrv->CaptureImage(pImageBuf,1)) 
		{ 
			pRobotDriver->m_cLog->Write("DynamicCaptureNew ��ͼʧ�ܣ�");
			return;
		}
		cvCvtColor(pImageBuf, m_pShowImage, CV_GRAY2RGB);

		char szName[256] = { 0 };
		//sprintf(szName, ".\\LocalFiles\\OutputFiles\\RobotA\\EndSrc\\%d.bin", i++);//���ñ���·��
		//BinaryImgSaveAsynchronous(szName, pImageBuf, 0);
		
		//sprintf(szName, ".\\LocalFiles\\OutputFiles\\RobotA\\EndSrc\\%d.jpg", i++);//���ñ���·��
		//cvSaveImage(szName, pImageBuf);

		long long lTime2 = XI_clock();

		// ����
		CString sWidgetLaserTrackParaFile = "WidgetLaserTrack";
		CString sWidgetLaserPntExtParaFile = "WidgetLaserPntExt";
		
		switch (m_eImgProcMethod)
		{
		case E_IMAGE_PROC_METHOD_EEEEE:
			CHECK_BOOL(FuncImageProcessEEEEE(pImageBuf, vtPoint2D));
			break;
		case E_IMAGE_PROC_METHOD_EEEEEInTrack:
			//CHECK_BOOL(FuncImageProcessInTrack_Lock(pImageBuf, m_pTraceModel->tFrontLine, m_pTraceModel->tBackLine, vtPoint2D, tCameraParam.eFlipMode));
			if (!get2DPnts(vtPoint2D, pImageBuf, sWidgetLaserTrackParaFile, sWidgetLaserPntExtParaFile))
			{
				nErrorTimes++;
				m_pRobotDriver->m_cLog->Write("ʧ�ܴ���%d", nErrorTimes);
				if (nErrorTimes > nMaxErrorTimes)
				{
					XiMessageBox("�����Ѷ˵�ʧ��");
					return;
				}
				
			}
			else
			{
				nErrorTimes = 0;
			}
			break;
		case E_IMAGE_PROC_METHOD_DoubleVGroove:
			break;
		default:
			break;
		}

		// ת����
		T_ROBOT_COORS tPointCloudRobotCoord; //����ת���Ļ���������
		T_ROBOT_COORS tPointCloudWorldCoord; //����ת������������
		Three_DPoint tThreePoint;  //3D����?
		int nSaveNum = 0;
		m_MutexTrackPointCloud.lock();
		long long lTime3 = XI_clock();

		std::vector<T_ROBOT_COORS> vtPoint3D = m_ptUnit->TranImageToBase(nCameraNo, vtPoint2D, tRobotCoord, tRobotPulse);

		for (int i = 0; i < vtPoint2D.size(); i++)
		{
			if (vtPoint2D[i].x < nMinX || vtPoint2D[i].x > nMaxX) continue; // ��Ч����������ݲ�����
			//tPointCloudRobotCoord = m_ptUnit->TranImageToBase(nCameraNo, vtPoint2D[i], tRobotCoord, tRobotPulse, &tPointAbsCoordInBase);
			tPointCloudRobotCoord = vtPoint3D[i];
			tPointCloudWorldCoord = tPointCloudRobotCoord;
			
			tPointCloudWorldCoord.dX += tRobotCoord.dBX;
			tPointCloudWorldCoord.dY += tRobotCoord.dBY;
			tPointCloudWorldCoord.dZ += tRobotCoord.dBZ;
			tThreePoint.x = tPointCloudWorldCoord.dX;
			tThreePoint.y = tPointCloudWorldCoord.dY;
			tThreePoint.z = tPointCloudWorldCoord.dZ;
			m_pTraceModel->ltPointCloud.push_back(tThreePoint);
			nSaveNum++;
			fprintf(pfPointCloud, "%d%11.3lf%11.3lf%11.3lf\n", nCapImageNo, tPointCloudWorldCoord.dX, tPointCloudWorldCoord.dY, tPointCloudWorldCoord.dZ);
			fflush(pfPointCloud);
			if (0 == i) // ��һ����ÿ��ͼ����Ľǵ�
			{
				m_pTraceModel->vtConnerWorldCoords.push_back(tPointCloudWorldCoord);
			}
		}
		m_pTraceModel->lnImageNo.push_back(nCapImageNo);
		m_pTraceModel->lnImagePtnNum.push_back(nSaveNum);
		m_pTraceModel->ltCapWorldCoord.push_back(tRobotCoord);
		long long lTime4 = XI_clock();

		if (bDynamicProc)
		{
			// ɾ���ɵ�������
			for (int i = 0; (i < m_pTraceModel->ltCapWorldCoord.size()) && (m_pTraceModel->ltCapWorldCoord.size() > 60); i++)
			{
				T_ROBOT_COORS tCoordS = m_pTraceModel->ltCapWorldCoord.front();
				T_ROBOT_COORS tCoordE = m_pTraceModel->ltCapWorldCoord.back();
				double dDis = TwoPointDis(
					tCoordS.dX + tCoordS.dBX, tCoordS.dY + tCoordS.dBY, tCoordS.dZ + tCoordS.dBZ,
					tCoordE.dX + tCoordE.dBX, tCoordE.dY + tCoordE.dBY, tCoordE.dZ + tCoordE.dBZ);
				if (dDis > dPointCloudLengthForFindEndpnt)
				{
					int nDelPtnNum = m_pTraceModel->lnImagePtnNum.front();
					std::list<Three_DPoint>::iterator iter = m_pTraceModel->ltPointCloud.begin();
					std::advance(iter, nDelPtnNum);
					m_pTraceModel->ltPointCloud.erase(m_pTraceModel->ltPointCloud.begin(), iter);
					m_pTraceModel->ltCapWorldCoord.pop_front();
					m_pTraceModel->lnImagePtnNum.pop_front();
					m_pTraceModel->lnImageNo.pop_front();
				}
			}
		}

		long long lTime5 = XI_clock();
		m_MutexTrackPointCloud.unlock();

		pRobotDriver->m_cLog->Write("[�����Ѷ˵�]: ɨ���ͼ����%d�ܺ�ʱ:%dms �����ȡ:%d ͼ��ɼ�%d ͼ����:%d ����ת��:%d ����ɾ��:%d CurXYZ:%.3lf %.3lf %.3lf",
			nCapImageNo,XI_clock() - lStartTime, lTime1 - lStartTime, lTime2 - lTime1, lTime3 - lTime2, lTime4 - lTime3, lTime5 - lTime4,
			tRobotCoord.dX, tRobotCoord.dY, tRobotCoord.dZ);
		nCapImageNo++;
	}
	fclose(pfPointCloud);
	m_pRobotDriver->HoldOn();
	Sleep(200);
	m_pRobotDriver->HoldOff();
	return;
}


bool CScanInitModule::FuncImageProcessInTrack_Lock(IplImage* pImg, XiLineParamNode& tFrontLine, XiLineParamNode& tBackLine, std::vector<CvPoint>& vtPoint2D, E_FLIP_MODE eFlipMode)
{
	vtPoint2D.clear();
	CvPoint Corner2D;
	CvPoint* tPoints = new CvPoint[10000];
	//FlipImage(pImg, eFlipMode); // ��ת
	bool bProcRst = WeightedPointCluster(pImg, &tBackLine, &tFrontLine, &Corner2D, true, "WeightedPointCluster");//����
	//WidgetLaserTrack();
	if(bProcRst)
		vtPoint2D.push_back(Corner2D);
	int nLength = FindLaserMidPiontEEEEEInTrack_Fix(pImg, tFrontLine, tBackLine, tPoints, "FindLaserMidPiontEEEEEInTrack_Fix");//ֻ�ܸ����ã�������+�����ĵ�
	//int nLength = FindLaserMidPiontEEEEE_(pImg, tPoints, 3, 60, 15, 5, 10, 0.5, 20000, 3, 25, false);
	//WidgetLaserPntExt();
	//FlipImage(pImg, eFlipMode); // ��ת�ָ�
	for (int i = 0; i < nLength; i++)
	{
		//ResumeImagePoint2D(pImg->width, pImg->height, tPoints[i], eFlipMode); // ��ά��ָ�
		vtPoint2D.push_back(tPoints[i]);
	}
	delete[] tPoints;
	return true;
}

bool CScanInitModule::FuncImageProcessInTrack(IplImage* pImg, XiLineParamNode& tFrontLine, XiLineParamNode& tBackLine, std::vector<CvPoint>& vtPoint2D, E_FLIP_MODE eFlipMode)
{
	vtPoint2D.clear();
	CvPoint Corner2D;
	CvPoint* tPoints = new CvPoint[10000];
	//FlipImage(pImg, eFlipMode); // ��ת
	//bool bProcRst = WeightedPointCluster(pImg, &tFrontLine, &tBackLine, &Corner2D, false, "WeightedPointCluster");//����
	vtPoint2D.push_back(Corner2D);
	//int nLength = FindLaserMidPiontEEEEEInTrack/*_Fix*/(pImg, tFrontLine, tBackLine, tPoints, 50, 0, 3, 90, 15, 60, 10,0.5,10000,3,11/*, "FindLaserMidPiontEEEEEInTrack_Fix"*/);//ֻ�ܸ����ã�������+�����ĵ�
	//int nLength = FindLaserMidPiontEEEEE_(pImg, tPoints, 3, 60, 15, 5, 10, 0.5, 20000, 3, 25, false);
	// 
	int nLength = LaserPntExtByWinCenterIter(pImg, tPoints, "LaserPntExtByWinCenterIter");
	// 
	//FlipImage(pImg, eFlipMode); // ��ת�ָ�
	for (int i = 0; i <nLength; i++)
	{
		//ResumeImagePoint2D(pImg->width, pImg->height, tPoints[i], eFlipMode); // ��ά��ָ�
		vtPoint2D.push_back(tPoints[i]);
	}
	delete[] tPoints;
	return true;
}

bool CScanInitModule::FuncImageProcessEEEEE(IplImage* pImg, std::vector<CvPoint>& vtPoint2D)
{
	vtPoint2D.clear();
//	CvPoint Corner2D;
	CvPoint* tPoints = new CvPoint[10000];
	int nLength = LaserPntExtByWinCenterIter(pImg, tPoints, "LaserPntExtByWinCenterIter");
	//int nLength = FindLaserMidPiontEEEEE(pImg, tPoints, 3, 30, 15, 30, 10, 0.5, 10000, 3, 15);
	for (int i = 0; i < nLength; i++)
	{
		vtPoint2D.push_back(tPoints[i]);
	}
	delete[] tPoints;
	return true;
}

bool CScanInitModule::lockLaser(CRobotDriverAdaptor* pRobotDriver, int nCameraNo)
{
	CDHGigeImageCapture* pDHCamDrv = (CDHGigeImageCapture*)m_ptUnit->GetCameraCtrl(nCameraNo);
	T_CAMREA_PARAM tCameraParam = m_ptUnit->GetCameraParam(nCameraNo);

	// ��ͼ
	if (!pDHCamDrv->CaptureImage(m_pTraceLockImg, 3))
	{
		pRobotDriver->m_cLog->Write("lockLaser ��ͼʧ�ܣ�");
		return false;
	}

	//���ǵ��ͺ���ɨ�豣��һ�£�����Ҳ���ᷭתͼ��
	//���Ҫ��תͼ�񣬺���ɨ��ҲҪ��ת
	//FlipImage(m_pTraceLockImg, m_ptUnit->GetCameraParam(nCameraNo).eFlipMode);

	//��������ͼ
	SaveImage(m_pTraceLockImg, OUTPUT_PATH + m_ptUnit->GetUnitName() + SEARCH_SCANLOCK_IMG);

	//����
	CvPoint cpKeyPoint, LeftPtn, RightPtn;
	bool assembleFilp = true;
	bool crossFilp = false;
	int nRst = WidgetLaserLock(m_pTraceLockImg, &m_tSearchLockLaserInfo, assembleFilp, crossFilp, "WidgetLaserLock");
	if (nRst > 0)
	{
		cpKeyPoint	= m_tSearchLockLaserInfo.crossPoint;
		LeftPtn		= m_tSearchLockLaserInfo.verticalPlatePnt;
		RightPtn	= m_tSearchLockLaserInfo.bottomPlatePnt;
	}
	else
	{
		XiMessageBoxOk("��������б������ʧ�ܣ�(������룺�Ҳ����ؼ���)");
		CTime cTime;
		cTime = CTime::GetCurrentTime();
		int nCurrentHour = cTime.GetHour();
		int nCurrentMinute = cTime.GetMinute();
		int nCurrentSecond = cTime.GetSecond();
		CString strFile;
		strFile.Format("%s%s\\LockError\\%d-%d-%d-%d-%d-%d.jpg", OUTPUT_PATH, m_ptUnit->GetRobotCtrl()->m_strRobotName, cTime.GetYear(), cTime.GetMonth(), cTime.GetDay(), nCurrentHour, nCurrentMinute, nCurrentSecond);
		SaveImage(m_pTraceLockImg, strFile.GetBuffer());
		if (m_pTraceLockImg != NULL)
		{
			cvReleaseImage(&m_pTraceLockImg);
		}
		return false;
	}

	//��ʾ���
	cvCvtColor(m_pTraceLockImg, m_pShowImage, CV_GRAY2RGB);
	cvLine(m_pShowImage, cpKeyPoint, LeftPtn, CV_RGB(0, 255, 0), 2);
	cvLine(m_pShowImage, cpKeyPoint, RightPtn, CV_RGB(0, 0, 255), 2);
	cvCircle(m_pShowImage, cpKeyPoint, 10, CV_RGB(255, 0, 0), 3);

	if (m_pTraceLockImg != NULL) {
		//�õ�Ԫ������Ƭ���ȴ���Ƭ�����ͷ��ڴ�
		cvReleaseImage(&m_pTraceLockImg);
	}
	return true;
}

int CScanInitModule::get2DPnts(std::vector<CvPoint>& vtPoint2D, IplImage* pImg, CString sWidgetLaserTrackParaFile, CString sWidgetLaserPntExtParaFile)
{
	vtPoint2D.clear();

	if (!WidgetLaserTrack(pImg, &m_tSearchLockLaserInfo, sWidgetLaserTrackParaFile.GetBuffer()))
	{
		m_pRobotDriver->m_cLog->Write("WidgetLaserTrack����ʧ��");
		return false;
	}

	CvPoint* pPnts = new CvPoint[10000];
	int nCounts = WidgetLaserPntExt(pImg, &m_tSearchLockLaserInfo, pPnts, sWidgetLaserPntExtParaFile.GetBuffer());
	for (int i = 0; i < nCounts; i++)
	{
		vtPoint2D.push_back(pPnts[i]);
	}
	delete[] pPnts;

	if (nCounts <= 0)
	{
		m_pRobotDriver->m_cLog->Write("WidgetLaserPntExt����ʧ��");
		return false;
	}

	return true;
}

// ����ͼƬ�����߳� �������ƴ����߳� �˶� ����������ɼ�ͼ������
void CScanInitModule::DynamicCaptureNew_H(CRobotDriverAdaptor* pRobotDriver, int nCameraNo, int nProcThreadNum, int nMaxImgBufSize/* = 50*/, int nTimeOut/* = 2000*/)
{
	long lPreStartTime = GetTickCount();
	long lStartTime = 0;
	int nCapImageNo = 0;
	int nSaveBufNo = 0;
	bool bCapSuc = false;
	int nImgStatus = 0;
	int nMinCapTimeInterval = 60; // ms
	T_ROBOT_COORS tRobotCoord;
	T_ANGLE_PULSE tRobotPulse;
	std::vector<CWinThread*> vpWinThread(0);
	CDHGigeImageCapture* pDHCamDrv = (CDHGigeImageCapture*)m_ptUnit->GetCameraCtrl(nCameraNo);
	IplImage* pImageBuf = pDHCamDrv->m_pImageBuff;
	T_CAMREA_PARAM tCameraParam = m_ptUnit->GetCameraParam(nCameraNo);

	m_pTraceModel->lnImageNo.clear();	// ��ǰ���Ƽ�����ÿ��ͼ���
	m_pTraceModel->lnImagePtnNum.clear();	// ��ǰ���Ƽ�����ÿ��ͼ����
	m_pTraceModel->ltPointCloud.clear(); // ��ǰ���Ƽ���
	m_pTraceModel->ltCapWorldCoord.clear(); // ��ǰ���Ƽ�����ÿ��ͼ�ɼ�ʱ��������
	m_pTraceModel->vtPointCloudEndPoints.clear(); // ÿ�ε��ƴ�����ȡ�Ķ˵���������
	m_pTraceModel->m_bCameraFindWeldEnd = false;
	m_pTraceModel->pfPointCloud = fopen(OUTPUT_PATH + m_pRobotDriver->m_strRobotName + WELDDATA_PATH + "AA_PointCloud.txt", "w");
	m_pTraceModel->pfImgCalcData = fopen(OUTPUT_PATH + m_pRobotDriver->m_strRobotName + WELDDATA_PATH + "AA_CapImgCoord.txt", "w");

	m_pCapImageData = new T_CAP_IMAGE_DATA(nMaxImgBufSize); // �������ݻ�����

	m_IsOpenImageProcess = true;
	for (int i = 0; i < nProcThreadNum; i++) // ����ͼƬ�����߳�
	{
		CWinThread* tProcImgThread = AfxBeginThread(ThreadImageProcess, this);
		tProcImgThread->m_bAutoDelete = false;
		vpWinThread.push_back(tProcImgThread);

	}
	CWinThread* tProcPtnCldThread = AfxBeginThread(ThreadJudgeEnd_ProcessSearch, this); // �������ƴ����߳�
	tProcPtnCldThread->m_bAutoDelete = false;
	vpWinThread.push_back(tProcPtnCldThread);

	pRobotDriver->CallJob(m_sMoveJobName); // ��ʼ�˶�

	int nCurWaitTime = 0;
	int nWaitTimeStep = 100;
	bool bIsRunning = m_ptUnit->WorldIsRunning();
	while (!m_ptUnit->WorldIsRunning()) // �ȴ��˶�
	{
		Sleep(nWaitTimeStep);
		nCurWaitTime += nWaitTimeStep;
		if (nCurWaitTime > nTimeOut)
		{
			DELETE_POINTER(m_pCapImageData);
			pRobotDriver->m_cLog->Write("[�����Ѷ˵�]: �ȴ��˶���ʱ��");
			return;
		}
	}

	// �����˶��� �� δ�����յ�

	while (m_ptUnit->WorldIsRunning() && true == m_IsOpenJudgeEnd_ProcessSearch)
	{
		lStartTime = GetTickCount();
		if (labs(lStartTime - lPreStartTime) < nMinCapTimeInterval)
		{
			Sleep(nMinCapTimeInterval - labs(lStartTime - lPreStartTime));
		}
		lStartTime = GetTickCount();
		lPreStartTime = lStartTime;

		// ������
		tRobotCoord = pRobotDriver->GetCurrentPos();
		tRobotPulse = pRobotDriver->GetCurrentPulse();
		// ��ͼ
		pDHCamDrv->CaptureImage(pImageBuf, 1);

		// ���浽������
		nSaveBufNo = m_pCapImageData->nTotalCapImgNum % m_pCapImageData->nMaxBufferSize;
		m_pCapImageData->vtCapCoord[nSaveBufNo] = tRobotCoord;
		m_pCapImageData->vtCapPulse[nSaveBufNo] = tRobotPulse;
		if (NULL == m_pCapImageData->vpImg[nSaveBufNo])
		{
			m_pCapImageData->vpImg[nSaveBufNo] = cvCreateImage(cvSize(tCameraParam.tDHCameraDriverPara.nRoiWidth, tCameraParam.tDHCameraDriverPara.nRoiHeight), IPL_DEPTH_8U, 1);
		}
		cvCopyImage(pImageBuf, m_pCapImageData->vpImg[nSaveBufNo]);
		m_pCapImageData->nTotalCapImgNum = nCapImageNo + 1;

		pRobotDriver->m_cLog->Write("[�����Ѷ˵�]: ɨ���ͼ�ܱ��%d �����%d ��ʱ%dms ��ͼ�ɹ�%d ͼƬ״̬%d",
			nCapImageNo, nSaveBufNo, GetTickCount() - lStartTime, bCapSuc, nImgStatus);
		nCapImageNo++;

	}
	m_IsOpenImageProcess = false;

	m_pRobotDriver->HoldOn();
	Sleep(500);
	m_pRobotDriver->HoldOff();

	WaitAndCheckAllThreadExit(vpWinThread);

	// �ͷ� m_pCapImageData
	for (int i = 0; i < m_pCapImageData->nMaxBufferSize; i++)
	{
		if (NULL != m_pCapImageData->vpImg[i])
		{
			cvReleaseImage(&(m_pCapImageData->vpImg[i]));
		}
	}
	DELETE_POINTER(m_pCapImageData);
	fclose(m_pTraceModel->pfPointCloud);
	fclose(m_pTraceModel->pfImgCalcData);
}

UINT CScanInitModule::ThreadImageProcess(void* pParam)
{
	CScanInitModule* pObj = (CScanInitModule*)pParam;
	pObj->FuncImageProcess(pObj->m_pRobotDriver);
	return 0;
}

bool CScanInitModule::FuncImageProcess(CRobotDriverAdaptor* pRobotCtrl)
{
	auto dPointCloudLengthForFindEndpnt = PARA_FLAT_MEASURE(dPointCloudLengthForFindEndpnt);
	m_pRobotDriver->m_cLog->Write("[�����Ѷ˵�]: FuncImageProcess��ʼ!");
	bool bSaveImg = true;
	int nSaveImgStepNo = 2;
	int nCameraNo = m_ptUnit->m_nTrackCameraNo;
	int nPtn2DStepDis = 2;
	int nTotalImageNo = 0;
	int nProcImageNo = 0;
	int nMaxBufSize = 0;
	int nBufNo = 0;
	IplImage* pImg = NULL;
	T_ROBOT_COORS tCapRobot;
	T_ANGLE_PULSE tCapPulse;
	T_ROBOT_COORS tPointCloudRobotCoord;
	T_ROBOT_COORS tConnerWorldCoord;
	T_ABS_POS_IN_BASE tPointAbsCoordInBase;
	Three_DPoint tThreePoint;
	std::list<Three_DPoint> ltThreePoint;
	CvPoint* tPoints = new CvPoint[10000];
	long lTime1 = 0, lTime2 = 0, lTime3 = 0, lTime4 = 0, lTime5 = 0;
	CvPoint Corner2D;
	XiLineParamNode tFrontLine = m_pTraceModel->tFrontLine;
	XiLineParamNode tBackLine = m_pTraceModel->tBackLine;
	CString sVisionIniName;

	while (m_IsOpenImageProcess)
	{
		// ��������ͼƬ�Ͳ�ͼ��������
		lTime1 = GetTickCount();
		while (m_IsOpenImageProcess && false == m_pCapImageData->m_MutexReadWrite.trylock()) Sleep(10);
		if (!m_IsOpenImageProcess) break;
		nTotalImageNo = m_pCapImageData->nTotalCapImgNum;
		nProcImageNo = m_pCapImageData->nTotalProImgNum;
		nMaxBufSize = m_pCapImageData->nMaxBufferSize;
		nBufNo = nProcImageNo % m_pCapImageData->nMaxBufferSize;
		if ((nTotalImageNo > 0) && (nProcImageNo < nTotalImageNo)) // ��δ�����ͼ ȡ���ݴ���
		{
			if (NULL == pImg)
			{
				sVisionIniName.Format("WeightedPointCluster%d", nProcImageNo);
				pImg = cvCreateImage(cvSize(m_pCapImageData->vpImg[nBufNo]->width, m_pCapImageData->vpImg[nBufNo]->height), IPL_DEPTH_8U, 1);
			}
			cvCopyImage(m_pCapImageData->vpImg[nBufNo], pImg);
			cvReleaseImage(&m_pCapImageData->vpImg[nBufNo]);
			m_pCapImageData->vpImg[nBufNo] = NULL;
			tCapRobot = m_pCapImageData->vtCapCoord[nBufNo];
			tCapPulse = m_pCapImageData->vtCapPulse[nBufNo];
			m_pCapImageData->nTotalProImgNum++;
			m_pCapImageData->m_MutexReadWrite.unlock();
		}
		else
		{
			m_pCapImageData->m_MutexReadWrite.unlock();
			Sleep(10);
			continue;
		}
		// �����ٶȹ�����
		if ((nTotalImageNo - nProcImageNo) >= nMaxBufSize)
		{
			m_IsOpenImageProcess = false;
			CheckMachineEmg(pRobotCtrl);
			XUI::MesBox::PopOkCancel("[�����Ѷ˵�]: {0} ͼ�������! Cap{1} Proc{2} MaxBuf{3}", m_pRobotDriver->m_strRobotName, nTotalImageNo, nProcImageNo, nMaxBufSize);
			continue;
		}
		lTime2 = GetTickCount();

		// ��ͼ ���� ����
		if (bSaveImg && (0 == nProcImageNo % nSaveImgStepNo))
		{
			SaveImage(&pImg, "%s%s\\Track\\%d.jpg", OUTPUT_PATH, m_pRobotDriver->m_strRobotName, nProcImageNo);
		}
		// ����ʱ�Ķ˵�ɨ����Բ�����ǰ�����ӿ� �𻡸��ٺ����Ǳ������ȫ�������ӿ�
		long lTime21 = GetTickCount();

		bool bProcRst = WeightedPointCluster(pImg, &tFrontLine, &tBackLine, &Corner2D, false, /*sVisionIniName.GetBuffer()*/"WeightedPointCluster");//����
		long lTime22 = GetTickCount();

		long lTime23 = GetTickCount();
		int nLength = FindLaserMidPiontEEEEEInTrack_Fix(pImg, tFrontLine, tBackLine, tPoints, "FindLaserMidPiontEEEEEInTrack_Fix");//ֻ�ܸ����ã�������+�����ĵ�
		long lTime24 = GetTickCount();

		int nSaveNum = 0;
		int nMinX = pImg->width / 4;
		int nMaxX = pImg->width * 3 / 4;
		ltThreePoint.clear();

		// �ǵ�����
		tConnerWorldCoord = m_ptUnit->TranImageToBase(nCameraNo, Corner2D, tCapRobot, tCapPulse, &tPointAbsCoordInBase);
		tConnerWorldCoord.dX += tCapRobot.dBX;
		tConnerWorldCoord.dY += tCapRobot.dBY;
		tConnerWorldCoord.dZ += tCapRobot.dBZ;
		fprintf(m_pTraceModel->pfPointCloud, "ͼ��:%d ���%d %11.3lf%11.3lf%11.3lf\n", nProcImageNo, -1, tConnerWorldCoord.dX, tConnerWorldCoord.dY, tConnerWorldCoord.dZ); // -1�������ֽǵ�͵�������
																																						// ��������
		for (int i = 0; i < nLength; i += nPtn2DStepDis)
		{
			if (tPoints[i].x < nMinX || tPoints[i].x > nMaxX) continue; // ��Ч����������ݲ�����
			tPointCloudRobotCoord = m_ptUnit->TranImageToBase(nCameraNo, tPoints[i], tCapRobot, tCapPulse, &tPointAbsCoordInBase);
			tThreePoint.x = tPointCloudRobotCoord.dX + tCapRobot.dBX;
			tThreePoint.y = tPointCloudRobotCoord.dY + tCapRobot.dBY;
			tThreePoint.z = tPointCloudRobotCoord.dZ + tCapRobot.dBZ;
			ltThreePoint.push_back(tThreePoint);
			nSaveNum++;
			fprintf(m_pTraceModel->pfPointCloud, "ͼ��:%d ���%d %11.3lf%11.3lf%11.3lf\n", nProcImageNo, i, tThreePoint.x, tThreePoint.y, tThreePoint.z);
			fflush(m_pTraceModel->pfPointCloud);
		}
		lTime3 = GetTickCount();

		// ��������������ݲ�ɾ����������
		m_MutexTrackPointCloud.lock();
		m_pTraceModel->ltPointCloud.insert(m_pTraceModel->ltPointCloud.end(), ltThreePoint.begin(), ltThreePoint.end());
		m_pTraceModel->lnImageNo.push_back(nProcImageNo);
		m_pTraceModel->lnImagePtnNum.push_back(nSaveNum);
		m_pTraceModel->ltCapWorldCoord.push_back(tCapRobot);
		m_pTraceModel->vtConnerWorldCoords.push_back(tConnerWorldCoord);
		for (int i = 0; (i < m_pTraceModel->ltCapWorldCoord.size()) && (m_pTraceModel->ltCapWorldCoord.size() > 40); i++) // ɾ���ɵ�������
		{
			T_ROBOT_COORS tCoordS = m_pTraceModel->ltCapWorldCoord.front();
			T_ROBOT_COORS tCoordE = m_pTraceModel->ltCapWorldCoord.back();
			double dDis = TwoPointDis(
				tCoordS.dX + tCoordS.dBX, tCoordS.dY + tCoordS.dBY, tCoordS.dZ + tCoordS.dBZ,
				tCoordE.dX + tCoordE.dBX, tCoordE.dY + tCoordE.dBY, tCoordE.dZ + tCoordE.dBZ);
			if (dDis > dPointCloudLengthForFindEndpnt)
			{
				int nDelPtnNum = m_pTraceModel->lnImagePtnNum.front();
				std::list<Three_DPoint>::iterator iter = m_pTraceModel->ltPointCloud.begin();
				std::advance(iter, nDelPtnNum);
				m_pTraceModel->ltPointCloud.erase(m_pTraceModel->ltPointCloud.begin(), iter);
				m_pTraceModel->ltCapWorldCoord.pop_front();
				m_pTraceModel->lnImagePtnNum.pop_front();
				m_pTraceModel->lnImageNo.pop_front();
			}
		}

		lTime4 = GetTickCount();
		if (0 == nProcImageNo % nSaveImgStepNo) // ��ʾ������ ������һ���߳���ʾ�����ƶ���������
		{
			DrawAndShowImg(&pImg, &m_pShowImage, Corner2D, tFrontLine, tBackLine);
		}
		fprintf(m_pTraceModel->pfImgCalcData, "ͼ��:%d ֱ�ǣ�%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf �ؽ�:%10d%10d%10d%10d%10d%10d%10d%10d%10d ��ά��:%10d%10d ��ά��:%11.3lf%11.3lf%11.3lf\n",
			nProcImageNo, tCapRobot.dX, tCapRobot.dY, tCapRobot.dZ, tCapRobot.dRX, tCapRobot.dRY, tCapRobot.dRZ, tCapRobot.dBX, tCapRobot.dBY, tCapRobot.dBZ,
			tCapPulse.nSPulse, tCapPulse.nLPulse, tCapPulse.nUPulse, tCapPulse.nRPulse, tCapPulse.nBPulse, tCapPulse.nTPulse, tCapPulse.lBXPulse, tCapPulse.lBYPulse, tCapPulse.lBZPulse,
			Corner2D.x, Corner2D.y, tConnerWorldCoord.dX, tConnerWorldCoord.dY, tConnerWorldCoord.dZ);
		m_MutexTrackPointCloud.unlock();
		lTime5 = GetTickCount();

		m_pRobotDriver->m_cLog->Write("[�����Ѷ˵�]: ɨ�账���ܱ��%d�ܺ�ʱ:%d ����׼��:%d ����ͼƬ%d ���ٴ���:%d �ɽ�����%d ���ƴ���%d ����ת��%d ���µ���%d ��ʾ���%d",
			nProcImageNo, lTime5 - lTime1, lTime2 - lTime1, lTime21 - lTime2, lTime22 - lTime21, lTime23 - lTime22, lTime24 - lTime23, lTime3 - lTime24, lTime4 - lTime3, lTime5 - lTime4);
	}
	if (NULL != pImg) cvReleaseImage(&pImg);
	delete[] tPoints;
	m_pRobotDriver->m_cLog->Write("[�����Ѷ˵�]: FuncImageProcess����!");
	return true;
}

void CScanInitModule::DrawAndShowImg(IplImage** ppSrc, IplImage** ppShowImg, const CvPoint& tPtn2D, const XiLineParamNode& tFrontLine, const XiLineParamNode& tBackLine)
{
	cvCvtColor(*ppSrc, *ppShowImg, CV_GRAY2RGB);
	cvCircle(*ppShowImg, tPtn2D, 10, CV_RGB(255, 0, 0), 3);
	CvPoint p1 = cvPoint(0, tFrontLine.b);
	CvPoint p2 = cvPoint((*ppShowImg)->width, (*ppShowImg)->width * tFrontLine.k + tFrontLine.b);
	CvPoint p3 = cvPoint(0, tBackLine.b);
	CvPoint p4 = cvPoint((*ppShowImg)->width, (*ppShowImg)->width * tBackLine.k + tBackLine.b);
	cvLine(*ppShowImg, p1, p2, CV_RGB(0, 255, 0), 2);
	cvLine(*ppShowImg, p3, p4, CV_RGB(0, 0, 255), 2);
}

bool CScanInitModule::InitDynamicCapture_H_M(
	E_POINT_CLOUD_PROC_MOMENT ePointCloudProcMoment, E_IMAGE_PROC_METHOD eImgProcMethod,
	E_POINT_CLOUD_PROC_METHOD ePointCloudProcMethod, int nCamNo, CString sSaveFilePath, CString sPointCloudFileName,
	double dSavePointCloudLen, E_WELD_SEAM_TYPE eWeldSeamType, int nGroupNo, int nLayerNo, bool bInsideCallJob/* = true*/,
	bool bJudgeEndPoint/* = false*/, bool bSaveImage/* = false*/,int nSaveImgStepNo/* = 2*/, int nPtn2DStepNo/* = 2*/)
{
	m_ePointCloudProcMoment = ePointCloudProcMoment;
	m_eImgProcMethod = eImgProcMethod;
	m_ePointCloudProcMethod = ePointCloudProcMethod;
	m_nCameraNo = nCamNo;
	m_sSaveFilePath = sSaveFilePath;
	m_sGroovePointCloudFileName = sPointCloudFileName;
	m_dSavePointCloudLen = dSavePointCloudLen;
	m_bDynamicJudgeEndPoint = bJudgeEndPoint;
	m_bSaveImg = bSaveImage;
	m_nSaveImgStepNo = nSaveImgStepNo;
	m_nPtn2DStepDis = nPtn2DStepNo;
	m_eWeldSeamType = eWeldSeamType;
	m_nGroupNo = nGroupNo;
	m_nLayerNo = nLayerNo;
	m_bInsideCallJob = bInsideCallJob;

	CheckFolder(m_sSaveFilePath);

	//����ͼ����ʽ ��ʼ��ʱ���� m_pTraceModel->tFrontLine  m_pTraceModel->tBackLine
	if (E_IMAGE_PROC_METHOD_EEEEEInTrack == m_eImgProcMethod)
	{
		// ����
		CDHGigeImageCapture* pDHCamDrv = (CDHGigeImageCapture*)m_ptUnit->GetCameraCtrl(m_nCameraNo);
		IplImage* pImageBuf = NULL;
		T_CAMREA_PARAM tCameraParam = m_ptUnit->GetCameraParam(m_nCameraNo);
//		CvPoint Corner2D;
		std::vector<CvPoint> vtLeftCorner2D;
		std::vector<CvPoint> vtRightCorner2D;
		if (!pDHCamDrv->CaptureImage(pImageBuf, 1))
		{
			XiMessageBox("InitDynamicCapture_H_M: �ɼ�ͼƬʧ��");
			return false;
		}
		CImageProcess* pImgProcess = new CImageProcess(
			tCameraParam.tDHCameraDriverPara.nRoiWidth,
			tCameraParam.tDHCameraDriverPara.nRoiHeight,
			0,
			m_ptUnit->GetTrackTipDataFileName(m_nCameraNo));

		if (false == ImageLockProcess(pImgProcess, &pImageBuf, m_pTraceModel->tFrontLine, m_pTraceModel->tBackLine, tCameraParam.eFlipMode))
		{
			delete pImgProcess;
			XiMessageBox("InitDynamicCapture_H_M: ����ʧ��");
			return false;
		}
		delete pImgProcess;
		return true;
	}
	return true;
}

void CScanInitModule::DynamicCaptureNew_H_M(CRobotDriverAdaptor* pRobotDriver, int nCameraNo, int nProcThreadNum, int nMaxImgBufSize/* = 50*/, int nTimeOut/* = 2000*/)
{
	long lPreStartTime = GetTickCount();
	long lStartTime = 0;
	int nCapImageNo = 0;
	int nSaveBufNo = 0;
	bool bCapSuc = false;
	int nImgStatus = 0;
	int nMinCapTimeInterval = 50; // ms
	T_ROBOT_COORS tRobotCoord;
	T_ANGLE_PULSE tRobotPulse;
	std::vector<CWinThread*> vpWinThread(0);
	CDHGigeImageCapture* pDHCamDrv = (CDHGigeImageCapture*)m_ptUnit->GetCameraCtrl(nCameraNo);
	IplImage* pImageBuf = pDHCamDrv->m_pImageBuff;
	T_CAMREA_PARAM tCameraParam = m_ptUnit->GetCameraParam(nCameraNo);

	m_IsOpenPointCloudProcess_H_M = false;
	m_pTraceModel->lnImageNo.clear();	// ��ǰ���Ƽ�����ÿ��ͼ���
	m_pTraceModel->lnImagePtnNum.clear();	// ��ǰ���Ƽ�����ÿ��ͼ����
	m_pTraceModel->ltPointCloud.clear(); // ��ǰ���Ƽ���
	m_pTraceModel->ltCapWorldCoord.clear(); // ��ǰ���Ƽ�����ÿ��ͼ�ɼ�ʱ��������
	m_pTraceModel->vtPointCloudEndPoints.clear(); // ÿ�ε��ƴ�����ȡ�Ķ˵���������
	m_pTraceModel->m_bCameraFindWeldEnd = false;
	m_pTraceModel->pfPointCloud = fopen(m_sSaveFilePath + m_sGroovePointCloudFileName/*"AA_PointCloud.txt"*/, "w");
	m_pTraceModel->pfImgCalcData = fopen(m_sSaveFilePath + "AA_CapImgCoord.txt", "w");
	m_pCapImageData_H_M = new T_CAP_IMAGE_DATA(nMaxImgBufSize); // �������ݻ�����

	m_IsOpenImageProcess_H_M = true;
	for (int i = 0; i < nProcThreadNum; i++) // ����ͼƬ�����߳�
	{
		CWinThread* tProcImgThread = AfxBeginThread(ThreadImageProcess_H_M, this);
		Sleep(50);
		tProcImgThread->m_bAutoDelete = false;
		vpWinThread.push_back(tProcImgThread);
	}
	if (m_ePointCloudProcMoment == E_POINT_CLOUD_PROC_MOMENT_DYNAMIC)
	{
		CWinThread* tProcPtnCldThread = AfxBeginThread(ThreadPointCloudProcess_H_M, this); // �������ƴ����߳�
		tProcPtnCldThread->m_bAutoDelete = false;
		vpWinThread.push_back(tProcPtnCldThread);
	}

	// �˴��˶���Ϊ���� m_pRobotDriver->m_vtWeldLineInWorldPoints;
	if (54 == m_pRobotDriver->m_nExternalAxleType)
	{
		pRobotDriver->CallJob(m_sMoveJobName); // ��ʼ�˶�
	}
	else
	{
		double dTotalVal = 800; // mm/min
		m_ptUnit->m_bSwingState = true;
		m_ptUnit->WriteSwingParam(1.5, 0, 0);
		m_ptUnit->SwingTracking(pRobotDriver, pRobotDriver->m_vtWeldLineInWorldPoints, dTotalVal);
	}


	int nCurWaitTime = 0;
	int nWaitTimeStep = 100;
	bool bIsRunning = m_ptUnit->WorldIsRunning();
	while (!m_ptUnit->WorldIsRunning()) // �ȴ��˶�
	{
		Sleep(nWaitTimeStep);
		nCurWaitTime += nWaitTimeStep;
		if (nCurWaitTime > nTimeOut)
		{
			DELETE_POINTER(m_pCapImageData_H_M);
			pRobotDriver->m_cLog->Write("[�����Ѷ˵�]: �ȴ��˶���ʱ��");
			return;
		}
	}

	// �����˶��� �� δ�����յ�
	while (m_ptUnit->WorldIsRunning() && (true == m_IsOpenPointCloudProcess_H_M || E_POINT_CLOUD_PROC_MOMENT_FINAL != m_ePointCloudProcMoment))
	{
		lStartTime = GetTickCount();
		if (labs(lStartTime - lPreStartTime) < nMinCapTimeInterval)
		{
			Sleep(nMinCapTimeInterval - labs(lStartTime - lPreStartTime));
		}
		lStartTime = GetTickCount();
		lPreStartTime = lStartTime;

		// ������
		tRobotCoord = pRobotDriver->GetCurrentPos();
		tRobotPulse = pRobotDriver->GetCurrentPulse();
		long lTime1 = GetTickCount();
		// ��ͼ
		pDHCamDrv->CaptureImage(pImageBuf, 1);

		//GrooveEndPntsInfo Info;
		//int nRst = FindGrooveEndPnts(pImageBuf, 1, &Info, 190, 1, 1, 0, "Local_Files\\ExtLib\\Vision\\ConfigFiles\\GroovePnts.ini");

		// ���浽������
		nSaveBufNo = m_pCapImageData_H_M->nTotalCapImgNum % m_pCapImageData_H_M->nMaxBufferSize;
		m_pCapImageData_H_M->vtCapCoord[nSaveBufNo] = tRobotCoord;
		m_pCapImageData_H_M->vtCapPulse[nSaveBufNo] = tRobotPulse;
		if (NULL == m_pCapImageData_H_M->vpImg[nSaveBufNo])
		{
			m_pCapImageData_H_M->vpImg[nSaveBufNo] = cvCreateImage(cvSize(tCameraParam.tDHCameraDriverPara.nRoiWidth, tCameraParam.tDHCameraDriverPara.nRoiHeight), IPL_DEPTH_8U, 1);
		}
		cvCopyImage(pImageBuf, m_pCapImageData_H_M->vpImg[nSaveBufNo]);
		long lTime2 = GetTickCount();
		m_pCapImageData_H_M->nTotalCapImgNum = nCapImageNo + 1;

		pRobotDriver->m_cLog->Write("[�����Ѷ˵�]: ɨ���ͼ�ܱ��%d �����%d ��ʱ%dms %dms %dms ��ͼ�ɹ�%d ͼƬ״̬%d %d",
			nCapImageNo, nSaveBufNo, GetTickCount() - lTime2, lTime2 - lTime1, lTime1 - lStartTime, bCapSuc, nImgStatus, 0/*Info.BPntsNum*/);
		nCapImageNo++;

	}
	m_IsOpenImageProcess_H_M = false;

	m_pRobotDriver->HoldOn();
	Sleep(500);
	m_pRobotDriver->HoldOff();

	WaitAndCheckAllThreadExit(vpWinThread);

	// �ͷ� m_pCapImageData
	for (int i = 0; i < m_pCapImageData_H_M->nMaxBufferSize; i++)
	{
		if (NULL != m_pCapImageData_H_M->vpImg[i])
		{
			cvReleaseImage(&(m_pCapImageData_H_M->vpImg[i]));
		}
	}
	DELETE_POINTER(m_pCapImageData_H_M);
	fclose(m_pTraceModel->pfPointCloud);
	fclose(m_pTraceModel->pfImgCalcData);

	if (m_ePointCloudProcMoment == E_POINT_CLOUD_PROC_MOMENT_FINAL)
	{
		FinalProc_Groove();
	}
}

UINT CScanInitModule::ThreadImageProcess_H_M(void* pParam)
{
	CScanInitModule* pObj = (CScanInitModule*)pParam;
	pObj->FuncImageProcess_H_M(pObj->m_pRobotDriver);
	return 0;
}

bool CScanInitModule::FuncImageProcess_H_M(CRobotDriverAdaptor* pRobotCtrl)
{
	bool bRst = false;
	switch (m_eImgProcMethod)
	{
	case E_IMAGE_PROC_METHOD_EEEEE: bRst = FuncImageProcess_H_M_EEEEE(pRobotCtrl); break;
	case E_IMAGE_PROC_METHOD_EEEEEInTrack: bRst = FuncImageProcess_H_M_EEEEEInTrack(pRobotCtrl); break;
	case E_IMAGE_PROC_METHOD_DoubleVGroove: bRst = FuncImageProcess_H_M_DoubleVGroove(pRobotCtrl); break;
	default: XUI::MesBox::PopOkCancel("H_M:ͼ��������������{0}", (int)m_eImgProcMethod); break;
	}
	return bRst;
}

bool CScanInitModule::FuncImageProcess_H_M_EEEEE(CRobotDriverAdaptor* pRobotCtrl)
{
	auto dPointCloudLengthForFindEndpnt = PARA_FLAT_MEASURE(dPointCloudLengthForFindEndpnt);
	m_pRobotDriver->m_cLog->Write("[�����Ѷ˵�]: FuncImageProcess��ʼ!");
	int nTotalImageNo = 0;
	int nProcImageNo = 0;
	int nMaxBufSize = 0;
	int nBufNo = 0;
	IplImage* pImg = NULL;
	T_ROBOT_COORS tCapRobot;
	T_ANGLE_PULSE tCapPulse;
	T_ROBOT_COORS tPointCloudRobotCoord;
	T_ROBOT_COORS tConnerWorldCoord;
	T_ABS_POS_IN_BASE tPointAbsCoordInBase;
	Three_DPoint tThreePoint;
	std::list<Three_DPoint> ltThreePoint;
	CvPoint* tPoints = new CvPoint[10000];
	long lTime1 = 0, lTime2 = 0, lTime3 = 0, lTime4 = 0, lTime5 = 0;
	IplImage* pTempImageBuff = NULL;
	CvPoint Corner2D;
	XiLineParamNode tFrontLine = m_pTraceModel->tFrontLine;
	XiLineParamNode tBackLine = m_pTraceModel->tBackLine;
	CString sVisionIniName;

	while (m_IsOpenImageProcess_H_M)
	{
		// ��������ͼƬ�Ͳ�ͼ��������
		lTime1 = GetTickCount();
		while (m_IsOpenImageProcess_H_M && false == m_pCapImageData_H_M->m_MutexReadWrite.trylock()) Sleep(10);
		if (!m_IsOpenImageProcess_H_M) break;
		nTotalImageNo = m_pCapImageData_H_M->nTotalCapImgNum;
		nProcImageNo = m_pCapImageData_H_M->nTotalProImgNum;
		nMaxBufSize = m_pCapImageData_H_M->nMaxBufferSize;
		nBufNo = nProcImageNo % m_pCapImageData_H_M->nMaxBufferSize;
		if ((nTotalImageNo > 0) && (nProcImageNo < nTotalImageNo)) // ��δ�����ͼ ȡ���ݴ���
		{
			if (NULL == pImg)
			{
				sVisionIniName.Format("WeightedPointCluster%d", nProcImageNo);
				pImg = cvCreateImage(cvSize(m_pCapImageData_H_M->vpImg[nBufNo]->width, m_pCapImageData_H_M->vpImg[nBufNo]->height), IPL_DEPTH_8U, 1);
			}
			cvCopyImage(m_pCapImageData_H_M->vpImg[nBufNo], pImg);
			cvReleaseImage(&m_pCapImageData_H_M->vpImg[nBufNo]);
			m_pCapImageData_H_M->vpImg[nBufNo] = NULL;
			tCapRobot = m_pCapImageData_H_M->vtCapCoord[nBufNo];
			tCapPulse = m_pCapImageData_H_M->vtCapPulse[nBufNo];
			m_pCapImageData_H_M->nTotalProImgNum++;
			m_pCapImageData_H_M->m_MutexReadWrite.unlock();
		}
		else
		{
			m_pCapImageData_H_M->m_MutexReadWrite.unlock();
			Sleep(10);
			continue;
		}
		// �����ٶȹ�����
		if ((nTotalImageNo - nProcImageNo) >= nMaxBufSize)
		{
			m_IsOpenImageProcess_H_M = false;
			CheckMachineEmg(pRobotCtrl);
			XUI::MesBox::PopOkCancel("[�����Ѷ˵�]: {0} ͼ�������! Cap{1} Proc{2} MaxBuf{3}", m_pRobotDriver->m_strRobotName, nTotalImageNo, nProcImageNo, nMaxBufSize);
			continue;
		}
		lTime2 = GetTickCount();

		// ��ͼ ���� ����
		if (m_bSaveImg && (0 == nProcImageNo % m_nSaveImgStepNo))
		{
			SaveImage(&pImg, "%s%s\\Track\\%d.jpg", OUTPUT_PATH, m_pRobotDriver->m_strRobotName, nProcImageNo);
		}

		if (NULL == pTempImageBuff)
		{
			pTempImageBuff = cvCreateImage(cvSize(pImg->width, pImg->height), IPL_DEPTH_8U, 1);
			cvSet(pTempImageBuff, CV_RGB(0, 0, 0));
		}
		// ����ʱ�Ķ˵�ɨ����Բ�����ǰ�����ӿ� �𻡸��ٺ����Ǳ������ȫ�������ӿ�
		long lTime21 = GetTickCount();
		//bool bProcRst = WeightedPointCluster(pImg, &tFrontLine, &tBackLine, &Corner2D, false, "WeightedPointCluster");//����
		long lTime22 = GetTickCount();
		int nLength = FindLaserMidPiontEEEEE(pImg, tPoints, 3, 30, 15, 30, 10, 0.5, 10000, 3, 15);
		//int nLength = FindLaserMidPiontEEEEEInTrack_(pImg, m_pTraceModel->tFrontLine, Corner2D, tPoints, 50, 0, 3, 90, 11, 90, 10, 0.5, 10000, 3, 11, 0);
		//nLength += FindLaserMidPiontEEEEEInTrack_(pImg, m_pTraceModel->tBackLine, Corner2D, tPoints + nLength, 50, 0, 3, 90, 11, 90, 10, 0.5, 10000 - nLength, 3, 11, 0);
		long lTime23 = GetTickCount();

		int nSaveNum = 0;
		int nMinX = pImg->width / 4;
		int nMaxX = pImg->width * 3 / 4;
		ltThreePoint.clear();

		//// �ǵ�����
		//tConnerWorldCoord = m_ptUnit->TranImageToBase(m_nCameraNo, Corner2D, tCapRobot, tCapPulse, &tPointAbsCoordInBase);
		//tConnerWorldCoord.dX += tCapRobot.dBX;
		//tConnerWorldCoord.dY += tCapRobot.dBY;
		//tConnerWorldCoord.dZ += tCapRobot.dBZ;
		//fprintf(m_pTraceModel->pfPointCloud, "ͼ��:%d ���%d %11.3lf%11.3lf%11.3lf\n", nProcImageNo, -1, tConnerWorldCoord.dX, tConnerWorldCoord.dY, tConnerWorldCoord.dZ); // -1�������ֽǵ�͵�������
		// ��������
		for (int i = 0; i < nLength; i += m_nPtn2DStepDis)
		{
			if (tPoints[i].x < nMinX || tPoints[i].x > nMaxX) continue; // ��Ч����������ݲ�����
			tPointCloudRobotCoord = m_ptUnit->TranImageToBase(m_nCameraNo, tPoints[i], tCapRobot, tCapPulse, &tPointAbsCoordInBase);
			tThreePoint.x = tPointCloudRobotCoord.dX + tCapRobot.dBX;
			tThreePoint.y = tPointCloudRobotCoord.dY + tCapRobot.dBY;
			tThreePoint.z = tPointCloudRobotCoord.dZ + tCapRobot.dBZ;
			ltThreePoint.push_back(tThreePoint);
			nSaveNum++;
			//fprintf(m_pTraceModel->pfPointCloud, "ͼ��:%d ���%d %11.3lf%11.3lf%11.3lf\n", nProcImageNo, i, tThreePoint.x, tThreePoint.y, tThreePoint.z);
			fprintf(m_pTraceModel->pfPointCloud, "%d%11.3lf%11.3lf%11.3lf\n", nProcImageNo, tThreePoint.x, tThreePoint.y, tThreePoint.z);
			fflush(m_pTraceModel->pfPointCloud);
		}
		lTime3 = GetTickCount();

		// ��������������ݲ�ɾ����������
		m_Mutex_H_M.lock();
		m_pTraceModel->ltPointCloud.insert(m_pTraceModel->ltPointCloud.end(), ltThreePoint.begin(), ltThreePoint.end());
		m_pTraceModel->lnImageNo.push_back(nProcImageNo);
		m_pTraceModel->lnImagePtnNum.push_back(nSaveNum);
		m_pTraceModel->ltCapWorldCoord.push_back(tCapRobot);
		m_pTraceModel->vtConnerWorldCoords.push_back(tConnerWorldCoord);
		for (int i = 0; i < m_pTraceModel->ltCapWorldCoord.size(); i++) // ɾ���ɵ�������
		{
			T_ROBOT_COORS tCoordS = m_pTraceModel->ltCapWorldCoord.front();
			T_ROBOT_COORS tCoordE = m_pTraceModel->ltCapWorldCoord.back();
			double dDis = TwoPointDis(
				tCoordS.dX + tCoordS.dBX, tCoordS.dY + tCoordS.dBY, tCoordS.dZ + tCoordS.dBZ,
				tCoordE.dX + tCoordE.dBX, tCoordE.dY + tCoordE.dBY, tCoordE.dZ + tCoordE.dBZ);
			if (dDis > dPointCloudLengthForFindEndpnt)
			{
				int nDelPtnNum = m_pTraceModel->lnImagePtnNum.front();
				std::list<Three_DPoint>::iterator iter = m_pTraceModel->ltPointCloud.begin();
				std::advance(iter, nDelPtnNum);
				m_pTraceModel->ltPointCloud.erase(m_pTraceModel->ltPointCloud.begin(), iter);
				m_pTraceModel->ltCapWorldCoord.pop_front();
				m_pTraceModel->lnImagePtnNum.pop_front();
				m_pTraceModel->lnImageNo.pop_front();
			}
		}

		lTime4 = GetTickCount();
		if (0 == nProcImageNo % m_nSaveImgStepNo) // ��ʾ������ ������һ���߳���ʾ�����ƶ���������
		{
			DrawAndShowImg(&pImg, &m_pShowImage, Corner2D, tFrontLine, tBackLine);
		}
		fprintf(m_pTraceModel->pfImgCalcData, "ͼ��:%d ֱ�ǣ�%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf �ؽ�:%10d%10d%10d%10d%10d%10d%10d%10d%10d ��ά��:%10d%10d ��ά��:%11.3lf%11.3lf%11.3lf\n",
			nProcImageNo, tCapRobot.dX, tCapRobot.dY, tCapRobot.dZ, tCapRobot.dRX, tCapRobot.dRY, tCapRobot.dRZ, tCapRobot.dBX, tCapRobot.dBY, tCapRobot.dBZ,
			tCapPulse.nSPulse, tCapPulse.nLPulse, tCapPulse.nUPulse, tCapPulse.nRPulse, tCapPulse.nBPulse, tCapPulse.nTPulse, tCapPulse.lBXPulse, tCapPulse.lBYPulse, tCapPulse.lBZPulse,
			Corner2D.x, Corner2D.y, tConnerWorldCoord.dX, tConnerWorldCoord.dY, tConnerWorldCoord.dZ);
		m_Mutex_H_M.unlock();
		lTime5 = GetTickCount();

		m_pRobotDriver->m_cLog->Write("[�����Ѷ˵�]: ɨ�账���ܱ��%d�ܺ�ʱ:%d ����׼��:%d ����ͼƬ%d ���ٴ���:%d �ɽ�����%d ����ת��%d ���µ���%d ��ʾ���%d",
			nProcImageNo, lTime5 - lTime1, lTime2 - lTime1, lTime21 - lTime2, lTime22 - lTime21, lTime23 - lTime22, lTime3 - lTime23, lTime4 - lTime3, lTime5 - lTime4);
	}
	if (NULL != pImg) cvReleaseImage(&pImg);
	if (NULL != pTempImageBuff) cvReleaseImage(&pTempImageBuff);
	delete[] tPoints;
	m_pRobotDriver->m_cLog->Write("[�����Ѷ˵�]: FuncImageProcess����!");
	return true;
}

bool CScanInitModule::FuncImageProcess_H_M_EEEEEInTrack(CRobotDriverAdaptor* pRobotCtrl)
{
	auto dPointCloudLengthForFindEndpnt = PARA_FLAT_MEASURE(dPointCloudLengthForFindEndpnt);
	m_pRobotDriver->m_cLog->Write("[�����Ѷ˵�]: FuncImageProcess��ʼ!");
	int nTotalImageNo = 0;
	int nProcImageNo = 0;
	int nMaxBufSize = 0;
	int nBufNo = 0;
	IplImage* pImg = NULL;
	T_ROBOT_COORS tCapRobot;
	T_ANGLE_PULSE tCapPulse;
	T_ROBOT_COORS tPointCloudRobotCoord;
	T_ROBOT_COORS tConnerWorldCoord;
	T_ABS_POS_IN_BASE tPointAbsCoordInBase;
	Three_DPoint tThreePoint;
	std::list<Three_DPoint> ltThreePoint;
	CvPoint* tPoints = new CvPoint[10000];
	long lTime1 = 0, lTime2 = 0, lTime3 = 0, lTime4 = 0, lTime5 = 0;
	IplImage* pTempImageBuff = NULL;
	CvPoint Corner2D;
	XiLineParamNode tFrontLine = m_pTraceModel->tFrontLine;
	XiLineParamNode tBackLine = m_pTraceModel->tBackLine;
	CString sVisionIniName;

	while (m_IsOpenImageProcess_H_M)
	{
		// ��������ͼƬ�Ͳ�ͼ��������
		lTime1 = GetTickCount();
		while (m_IsOpenImageProcess_H_M && false == m_pCapImageData_H_M->m_MutexReadWrite.trylock()) Sleep(10);
		if (!m_IsOpenImageProcess_H_M) break;
		nTotalImageNo = m_pCapImageData_H_M->nTotalCapImgNum;
		nProcImageNo = m_pCapImageData_H_M->nTotalProImgNum;
		nMaxBufSize = m_pCapImageData_H_M->nMaxBufferSize;
		nBufNo = nProcImageNo % m_pCapImageData_H_M->nMaxBufferSize;
		if ((nTotalImageNo > 0) && (nProcImageNo < nTotalImageNo)) // ��δ�����ͼ ȡ���ݴ���
		{
			if (NULL == pImg)
			{
				sVisionIniName.Format("WeightedPointCluster%d", nProcImageNo);
				pImg = cvCreateImage(cvSize(m_pCapImageData_H_M->vpImg[nBufNo]->width, m_pCapImageData_H_M->vpImg[nBufNo]->height), IPL_DEPTH_8U, 1);
			}
			cvCopyImage(m_pCapImageData_H_M->vpImg[nBufNo], pImg);
			cvReleaseImage(&m_pCapImageData_H_M->vpImg[nBufNo]);
			m_pCapImageData_H_M->vpImg[nBufNo] = NULL;
			tCapRobot = m_pCapImageData_H_M->vtCapCoord[nBufNo];
			tCapPulse = m_pCapImageData_H_M->vtCapPulse[nBufNo];
			m_pCapImageData_H_M->nTotalProImgNum++;
			m_pCapImageData_H_M->m_MutexReadWrite.unlock();
		}
		else
		{
			m_pCapImageData_H_M->m_MutexReadWrite.unlock();
			Sleep(10);
			continue;
		}
		// �����ٶȹ�����
		if ((nTotalImageNo - nProcImageNo) >= nMaxBufSize)
		{
			m_IsOpenImageProcess_H_M = false;
			CheckMachineEmg(pRobotCtrl);
			XUI::MesBox::PopOkCancel("[�����Ѷ˵�]: {0} ͼ�������! Cap{1} Proc{2} MaxBuf{3}", m_pRobotDriver->m_strRobotName, nTotalImageNo, nProcImageNo, nMaxBufSize);
			continue;
		}
		lTime2 = GetTickCount();

		// ��ͼ ���� ����
		if (m_bSaveImg && (0 == nProcImageNo % m_nSaveImgStepNo))
		{
			SaveImage(&pImg, "%s%s\\Track\\%d.jpg", OUTPUT_PATH, m_pRobotDriver->m_strRobotName, nProcImageNo);
		}

		if (NULL == pTempImageBuff)
		{
			pTempImageBuff = cvCreateImage(cvSize(pImg->width, pImg->height), IPL_DEPTH_8U, 1);
			cvSet(pTempImageBuff, CV_RGB(0, 0, 0));
		}
		// ����ʱ�Ķ˵�ɨ����Բ�����ǰ�����ӿ� �𻡸��ٺ����Ǳ������ȫ�������ӿ�
		long lTime21 = GetTickCount();
		bool bProcRst = WeightedPointCluster(pImg, &tFrontLine, &tBackLine, &Corner2D, false, "WeightedPointCluster");//����
		long lTime22 = GetTickCount();
		long lTime23 = GetTickCount();
		int nLength = FindLaserMidPiontEEEEEInTrack_Fix(pImg, tFrontLine, tBackLine, tPoints, "FindLaserMidPiontEEEEEInTrack_Fix");//ֻ�ܸ����ã�������+�����ĵ�
		long lTime24 = GetTickCount();

		int nSaveNum = 0;
		int nMinX = pImg->width / 4;
		int nMaxX = pImg->width * 3 / 4;
		ltThreePoint.clear();

		// �ǵ�����
		tConnerWorldCoord = m_ptUnit->TranImageToBase(m_nCameraNo, Corner2D, tCapRobot, tCapPulse, &tPointAbsCoordInBase);
		tConnerWorldCoord.dX += tCapRobot.dBX;
		tConnerWorldCoord.dY += tCapRobot.dBY;
		tConnerWorldCoord.dZ += tCapRobot.dBZ;
		fprintf(m_pTraceModel->pfPointCloud, "ͼ��:%d ���%d %11.3lf%11.3lf%11.3lf\n", nProcImageNo, -1, tConnerWorldCoord.dX, tConnerWorldCoord.dY, tConnerWorldCoord.dZ); // -1�������ֽǵ�͵�������
		// ��������
		for (int i = 0; i < nLength; i += m_nPtn2DStepDis)
		{
			if (tPoints[i].x < nMinX || tPoints[i].x > nMaxX) continue; // ��Ч����������ݲ�����
			tPointCloudRobotCoord = m_ptUnit->TranImageToBase(m_nCameraNo, tPoints[i], tCapRobot, tCapPulse, &tPointAbsCoordInBase);
			tThreePoint.x = tPointCloudRobotCoord.dX + tCapRobot.dBX;
			tThreePoint.y = tPointCloudRobotCoord.dY + tCapRobot.dBY;
			tThreePoint.z = tPointCloudRobotCoord.dZ + tCapRobot.dBZ;
			ltThreePoint.push_back(tThreePoint);
			nSaveNum++;
			fprintf(m_pTraceModel->pfPointCloud, "%d %11.3lf%11.3lf%11.3lf\n", nProcImageNo, tThreePoint.x, tThreePoint.y, tThreePoint.z);
			fflush(m_pTraceModel->pfPointCloud);
		}
		lTime3 = GetTickCount();

		// ��������������ݲ�ɾ����������
		m_Mutex_H_M.lock();
		m_pTraceModel->ltPointCloud.insert(m_pTraceModel->ltPointCloud.end(), ltThreePoint.begin(), ltThreePoint.end());
		m_pTraceModel->lnImageNo.push_back(nProcImageNo);
		m_pTraceModel->lnImagePtnNum.push_back(nSaveNum);
		m_pTraceModel->ltCapWorldCoord.push_back(tCapRobot);
		m_pTraceModel->vtConnerWorldCoords.push_back(tConnerWorldCoord);
		for (int i = 0; (i < m_pTraceModel->ltCapWorldCoord.size()) && (m_pTraceModel->ltCapWorldCoord.size() > 40); i++) // ɾ���ɵ�������
		{
			T_ROBOT_COORS tCoordS = m_pTraceModel->ltCapWorldCoord.front();
			T_ROBOT_COORS tCoordE = m_pTraceModel->ltCapWorldCoord.back();
			double dDis = TwoPointDis(
				tCoordS.dX + tCoordS.dBX, tCoordS.dY + tCoordS.dBY, tCoordS.dZ + tCoordS.dBZ,
				tCoordE.dX + tCoordE.dBX, tCoordE.dY + tCoordE.dBY, tCoordE.dZ + tCoordE.dBZ);
			if (dDis > dPointCloudLengthForFindEndpnt)
			{
				int nDelPtnNum = m_pTraceModel->lnImagePtnNum.front();
				std::list<Three_DPoint>::iterator iter = m_pTraceModel->ltPointCloud.begin();
				std::advance(iter, nDelPtnNum);
				m_pTraceModel->ltPointCloud.erase(m_pTraceModel->ltPointCloud.begin(), iter);
				m_pTraceModel->ltCapWorldCoord.pop_front();
				m_pTraceModel->lnImagePtnNum.pop_front();
				m_pTraceModel->lnImageNo.pop_front();
			}
		}

		lTime4 = GetTickCount();
		if (0 == nProcImageNo % m_nSaveImgStepNo) // ��ʾ������ ������һ���߳���ʾ�����ƶ���������
		{
			DrawAndShowImg(&pImg, &m_pShowImage, Corner2D, tFrontLine, tBackLine);
		}
		fprintf(m_pTraceModel->pfImgCalcData, "ͼ��:%d ֱ�ǣ�%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf �ؽ�:%10d%10d%10d%10d%10d%10d%10d%10d%10d ��ά��:%10d%10d ��ά��:%11.3lf%11.3lf%11.3lf\n",
			nProcImageNo, tCapRobot.dX, tCapRobot.dY, tCapRobot.dZ, tCapRobot.dRX, tCapRobot.dRY, tCapRobot.dRZ, tCapRobot.dBX, tCapRobot.dBY, tCapRobot.dBZ,
			tCapPulse.nSPulse, tCapPulse.nLPulse, tCapPulse.nUPulse, tCapPulse.nRPulse, tCapPulse.nBPulse, tCapPulse.nTPulse, tCapPulse.lBXPulse, tCapPulse.lBYPulse, tCapPulse.lBZPulse,
			Corner2D.x, Corner2D.y, tConnerWorldCoord.dX, tConnerWorldCoord.dY, tConnerWorldCoord.dZ);
		m_Mutex_H_M.unlock();
		lTime5 = GetTickCount();

		m_pRobotDriver->m_cLog->Write("[�����Ѷ˵�]: ɨ�账���ܱ��%d�ܺ�ʱ:%d ����׼��:%d ����ͼƬ%d ���ٴ���:%d �ɽ�����%d ���ƴ���%d ����ת��%d ���µ���%d ��ʾ���%d",
			nProcImageNo, lTime5 - lTime1, lTime2 - lTime1, lTime21 - lTime2, lTime22 - lTime21, lTime23 - lTime22, lTime24 - lTime23, lTime3 - lTime24, lTime4 - lTime3, lTime5 - lTime4);
	}
	if (NULL != pImg) cvReleaseImage(&pImg);
	if (NULL != pTempImageBuff) cvReleaseImage(&pTempImageBuff);
	delete[] tPoints;
	m_pRobotDriver->m_cLog->Write("[�����Ѷ˵�]: FuncImageProcess����!");
	return true;
}

bool CScanInitModule::FuncImageProcess_H_M_DoubleVGroove(CRobotDriverAdaptor* pRobotCtrl)
{
	auto dPointCloudLengthForFindEndpnt = PARA_FLAT_MEASURE(dPointCloudLengthForFindEndpnt);
	m_pRobotDriver->m_cLog->Write("[�����Ѷ˵�]: FuncImageProcess_H_M_DoubleVGroove��ʼ!");
	int nTotalImageNo = 0;
	int nProcImageNo = 0;
	int nMaxBufSize = 0;
	int nBufNo = 0;
	IplImage* pImg = NULL;
	T_ROBOT_COORS tCapRobot;
	T_ANGLE_PULSE tCapPulse;
	T_ROBOT_COORS tPointCloudRobotCoord;
	T_ROBOT_COORS tConnerWorldCoord;
	T_ABS_POS_IN_BASE tPointAbsCoordInBase;
	Three_DPoint tThreePoint;
	std::list<Three_DPoint> ltThreePoint;
	CvPoint* tPoints = new CvPoint[10000];
	long lTime1 = 0, lTime2 = 0, lTime3 = 0, lTime4 = 0, lTime5 = 0;
	IplImage* pTempImageBuff = NULL;
	CvPoint Corner2D{0,0};
	XiLineParamNode tFrontLine = m_pTraceModel->tFrontLine;
	XiLineParamNode tBackLine = m_pTraceModel->tBackLine;
	CString sVisionIniName;

	//T_CAMREA_PARAM tCameraParam = m_ptUnit->GetCameraParam(m_ptUnit->m_nTrackCameraNo);
	//CDHGigeImageCapture* pDHCamera = (CDHGigeImageCapture*)m_ptUnit->GetCameraCtrl(m_ptUnit->m_nMeasureCameraNo);
	//IplImage* pBufferImg = pDHCamera->m_pImageBuff;
	//CString sIniFile;
	//sIniFile.Format("%s%s\\FindCornnerParam.ini", DATA_PATH, m_ptUnit->m_tContralUnit.strUnitName);
	//CFindCornner cFindCornner(pBufferImg->width, pBufferImg->height, sIniFile);

	while (m_IsOpenImageProcess_H_M)
	{
		// ��������ͼƬ�Ͳ�ͼ��������
		lTime1 = GetTickCount();
		while (m_IsOpenImageProcess_H_M && false == m_pCapImageData_H_M->m_MutexReadWrite.trylock()) Sleep(10);
		if (!m_IsOpenImageProcess_H_M) break;
		nTotalImageNo = m_pCapImageData_H_M->nTotalCapImgNum;
		nProcImageNo = m_pCapImageData_H_M->nTotalProImgNum;
		nMaxBufSize = m_pCapImageData_H_M->nMaxBufferSize;
		nBufNo = nProcImageNo % m_pCapImageData_H_M->nMaxBufferSize;
		if ((nTotalImageNo > 0) && (nProcImageNo < nTotalImageNo)) // ��δ�����ͼ ȡ���ݴ���
		{
			if (NULL == pImg)
			{
				sVisionIniName.Format("WeightedPointCluster%d", nProcImageNo);
				pImg = cvCreateImage(cvSize(m_pCapImageData_H_M->vpImg[nBufNo]->width, m_pCapImageData_H_M->vpImg[nBufNo]->height), IPL_DEPTH_8U, 1);
			}
			cvCopyImage(m_pCapImageData_H_M->vpImg[nBufNo], pImg);
			cvReleaseImage(&m_pCapImageData_H_M->vpImg[nBufNo]);
			m_pCapImageData_H_M->vpImg[nBufNo] = NULL;
			tCapRobot = m_pCapImageData_H_M->vtCapCoord[nBufNo];
			tCapPulse = m_pCapImageData_H_M->vtCapPulse[nBufNo];
			m_pCapImageData_H_M->nTotalProImgNum++;
			m_pCapImageData_H_M->m_MutexReadWrite.unlock();
		}
		else
		{
			m_pCapImageData_H_M->m_MutexReadWrite.unlock();
			Sleep(10);
			continue;
		}
		// �����ٶȹ�����
		if ((nTotalImageNo - nProcImageNo) >= nMaxBufSize)
		{
			m_IsOpenImageProcess_H_M = false;
			CheckMachineEmg(pRobotCtrl);
			XUI::MesBox::PopOkCancel("[�����Ѷ˵�]: {0} ͼ�������! Cap{1} Proc{2} MaxBuf{3}", m_pRobotDriver->m_strRobotName, nTotalImageNo, nProcImageNo, nMaxBufSize);
			continue;
		}
		lTime2 = GetTickCount();

		// ��ͼ ���� ����
		if (m_bSaveImg && (0 == nProcImageNo % m_nSaveImgStepNo))
		{
			SaveImage(&pImg, "%s%s\\Track\\%d.jpg", OUTPUT_PATH, m_pRobotDriver->m_strRobotName, nProcImageNo);
		}

		if (NULL == pTempImageBuff)
		{
			pTempImageBuff = cvCreateImage(cvSize(pImg->width, pImg->height), IPL_DEPTH_8U, 1);
			cvSet(pTempImageBuff, CV_RGB(0, 0, 0));
		}
		long lTime21 = GetTickCount();

		GrooveEndPntsInfo tInfo;
		char sNo[100];
		itoa(nProcImageNo, sNo, sizeof(100));
		//int nRst = FindGrooveEndPnts(pImg, 1, &tInfo, 180, 1, 1, 0, sNo);
		int nRst = FindGrooveEndPnts(pImg, 1, &tInfo, 180, 0, 1, 0, sNo);
		cvCvtColor(pImg, m_pShowImage, CV_GRAY2BGR);
		int nLength = 0;
		for (int i = 0; i < 4; i++)
		{
			tPoints[nLength++] = tInfo.outputPnts[i];
			cvCircle(m_pShowImage, tInfo.outputPnts[i], 10, CV_RGB(255, 0, 255), 3);
		}
		for (int i = 0; i < tInfo.HPntsNum; i++)
		{
			tPoints[nLength++] = tInfo.HorizontalLinePnts[i];
			cvCircle(m_pShowImage, tInfo.HorizontalLinePnts[i], 3, CV_RGB(255, 0, 0), 1);
		}
		for (int i = 0; i < tInfo.GPntsNum; i++)
		{
			tPoints[nLength++] = tInfo.GrooveLinePnts[i];
			cvCircle(m_pShowImage, tInfo.GrooveLinePnts[i], 3, CV_RGB(0, 255, 0), 1);
		}
		for (int i = 0; i < tInfo.BPntsNum; i++)
		{
			tPoints[nLength++] = tInfo.BottomLinePnts[i];
			cvCircle(m_pShowImage, tInfo.BottomLinePnts[i], 3, CV_RGB(0, 0, 255), 1);
		}
		m_pRobotDriver->m_cLog->Write("[�����Ѷ˵�]:����ͼ��%d ResultNum: %d %d %d", nProcImageNo, tInfo.HPntsNum, tInfo.GPntsNum, tInfo.BPntsNum);
		long lTime22 = GetTickCount();

		int nSaveNum = 0;
		int nMinX = pImg->width / 4;
		int nMaxX = pImg->width * 3 / 4;
		ltThreePoint.clear();

		//// �ǵ�����
		//tConnerWorldCoord = m_ptUnit->TranImageToBase(m_nCameraNo, Corner2D, tCapRobot, tCapPulse, &tPointAbsCoordInBase);
		//tConnerWorldCoord.dX += tCapRobot.dBX;
		//tConnerWorldCoord.dY += tCapRobot.dBY;
		//tConnerWorldCoord.dZ += tCapRobot.dBZ;
		//fprintf(m_pTraceModel->pfPointCloud, "ͼ��:%d ���%d %11.3lf%11.3lf%11.3lf\n", nProcImageNo, -1, tConnerWorldCoord.dX, tConnerWorldCoord.dY, tConnerWorldCoord.dZ); // -1�������ֽǵ�͵�������
		// ��������
		for (int i = 0; i < nLength; i += m_nPtn2DStepDis)
		{
			if (tPoints[i].x < nMinX || tPoints[i].x > nMaxX) continue; // ��Ч����������ݲ�����
			tPointCloudRobotCoord = m_ptUnit->TranImageToBase(m_nCameraNo, tPoints[i], tCapRobot, tCapPulse, &tPointAbsCoordInBase);
			tThreePoint.x = tPointCloudRobotCoord.dX + tCapRobot.dBX;
			tThreePoint.y = tPointCloudRobotCoord.dY + tCapRobot.dBY;
			tThreePoint.z = tPointCloudRobotCoord.dZ + tCapRobot.dBZ;
			ltThreePoint.push_back(tThreePoint);
			nSaveNum++;
			fprintf(m_pTraceModel->pfPointCloud, "%d %11.3lf%11.3lf%11.3lf\n",
				nProcImageNo, tThreePoint.x, tThreePoint.y, tThreePoint.z);
			fflush(m_pTraceModel->pfPointCloud);
		}
		lTime3 = GetTickCount();

		// ��������������ݲ�ɾ����������
		m_Mutex_H_M.lock();
		m_pTraceModel->ltPointCloud.insert(m_pTraceModel->ltPointCloud.end(), ltThreePoint.begin(), ltThreePoint.end());
		m_pTraceModel->lnImageNo.push_back(nProcImageNo);
		m_pTraceModel->lnImagePtnNum.push_back(nSaveNum);
		m_pTraceModel->ltCapWorldCoord.push_back(tCapRobot);
		m_pTraceModel->vtConnerWorldCoords.push_back(tConnerWorldCoord);
		for (int i = 0; (i < m_pTraceModel->ltCapWorldCoord.size()) && (m_pTraceModel->ltCapWorldCoord.size() > 40); i++) // ɾ���ɵ�������
		{
			T_ROBOT_COORS tCoordS = m_pTraceModel->ltCapWorldCoord.front();
			T_ROBOT_COORS tCoordE = m_pTraceModel->ltCapWorldCoord.back();
			double dDis = TwoPointDis(
				tCoordS.dX + tCoordS.dBX, tCoordS.dY + tCoordS.dBY, tCoordS.dZ + tCoordS.dBZ,
				tCoordE.dX + tCoordE.dBX, tCoordE.dY + tCoordE.dBY, tCoordE.dZ + tCoordE.dBZ);
			if (dDis > dPointCloudLengthForFindEndpnt)
			{
				int nDelPtnNum = m_pTraceModel->lnImagePtnNum.front();
				std::list<Three_DPoint>::iterator iter = m_pTraceModel->ltPointCloud.begin();
				std::advance(iter, nDelPtnNum);
				m_pTraceModel->ltPointCloud.erase(m_pTraceModel->ltPointCloud.begin(), iter);
				m_pTraceModel->ltCapWorldCoord.pop_front();
				m_pTraceModel->lnImagePtnNum.pop_front();
				m_pTraceModel->lnImageNo.pop_front();
			}
		}

		lTime4 = GetTickCount();

		fprintf(m_pTraceModel->pfImgCalcData, "ͼ��:%d ֱ�ǣ�%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf �ؽ�:%10d%10d%10d%10d%10d%10d%10d%10d%10d ��ά��:%10d%10d ��ά��:%11.3lf%11.3lf%11.3lf\n",
			nProcImageNo, tCapRobot.dX, tCapRobot.dY, tCapRobot.dZ, tCapRobot.dRX, tCapRobot.dRY, tCapRobot.dRZ, tCapRobot.dBX, tCapRobot.dBY, tCapRobot.dBZ,
			tCapPulse.nSPulse, tCapPulse.nLPulse, tCapPulse.nUPulse, tCapPulse.nRPulse, tCapPulse.nBPulse, tCapPulse.nTPulse, tCapPulse.lBXPulse, tCapPulse.lBYPulse, tCapPulse.lBZPulse,
			Corner2D.x, Corner2D.y, tConnerWorldCoord.dX, tConnerWorldCoord.dY, tConnerWorldCoord.dZ);
		m_Mutex_H_M.unlock();
		lTime5 = GetTickCount();

		m_pRobotDriver->m_cLog->Write("[�����Ѷ˵�]: ɨ�账���ܱ��%d�ܺ�ʱ:%d ����׼��:%d ����ͼƬ%d ���ٴ���:%d ����ת��%d ���µ���%d ��ʾ���%d",
			nProcImageNo, lTime5 - lTime1, lTime2 - lTime1, lTime21 - lTime2, lTime22 - lTime21, lTime3 - lTime22, lTime4 - lTime3, lTime5 - lTime4);
	}
	if (NULL != pImg) cvReleaseImage(&pImg);
	if (NULL != pTempImageBuff) cvReleaseImage(&pTempImageBuff);
	delete[] tPoints;
	m_pRobotDriver->m_cLog->Write("[�����Ѷ˵�]: FuncImageProcess_H_M_DoubleVGroove����!");
	return true;
}

UINT CScanInitModule::ThreadPointCloudProcess_H_M(void* pParam)
{
	CScanInitModule* pObj = (CScanInitModule*)pParam;
	if (pObj->m_IsOpenPointCloudProcess_H_M)
	{
		return 0;
	}
	pObj->m_IsOpenPointCloudProcess_H_M = true;
	pObj->FuncPointCloudProcess_H_M(pObj->m_pRobotDriver);
	pObj->m_IsOpenPointCloudProcess_H_M = false;
	return 0;
}

bool CScanInitModule::FuncPointCloudProcess_H_M(CRobotDriverAdaptor* pRobotCtrl, int nTimeOut/* = 2000*/)
{
	bool bRst = false;
	switch (m_ePointCloudProcMethod)
	{
	case E_POINT_CLOUD_PROC_METHOD_GetRiserEndPoint:bRst = FuncPointCloudProcess_H_M_GetRiser(pRobotCtrl, nTimeOut); break;
	case E_POINT_CLOUD_PROC_METHOD_Groove:bRst = FuncPointCloudProcess_H_M_Groove(pRobotCtrl, nTimeOut); break;
	case E_POINT_CLOUD_PROC_METHOD_DoubleVGroove:bRst = FuncPointCloudProcess_H_M_DoubleVGroove(pRobotCtrl, nTimeOut); break;
	default: XUI::MesBox::PopOkCancel("H_M:���ƴ�������������{0}", (int)m_ePointCloudProcMethod); break;
	}
	return bRst;
}

bool CScanInitModule::FuncPointCloudProcess_H_M_GetRiser(CRobotDriverAdaptor* pRobotCtrl, int nTimeOut/* = 2000*/)
{
	XiMessageBoxOk("δʵ��");
	return true;
}

bool CScanInitModule::FuncPointCloudProcess_H_M_Groove(CRobotDriverAdaptor* pRobotCtrl, int nTimeOut/* = 2000*/)
{
	//m_pRobotDriver->m_cLog->Write("[�¿�ɨ��]: FuncPointCloudProcess_H_M_Groove��ʼ!");
	//double dMinProcessLen = m_dSavePointCloudLen - 5.0; // ��С���ƴ����� ��С����󱣴泤��
	//int nWaitTimeStep = 100;			// �ȴ��˶����ʱ�䲽��
	//bool bIsRunning = false;
	//int nPreImageNoE = 0;
	//int nErrorNum = 0;					// ��������������
	//int nErrorThreshold = 3;			// ��������������ν�������
	//double dProcStepDis = 50.0;			// ���¶೤����ִ��һ�δ���
	//T_ROBOT_COORS tPreScanE = m_pRobotDriver->GetCurrentPos();			// ��¼ÿ�δ����ǵ����յ�����


	//// �ȴ������˶�
	//int nCurWaitTime = 0;
	//bIsRunning = m_ptUnit->WorldIsRunning();
	//while (!bIsRunning)
	//{
	//	Sleep(nWaitTimeStep);
	//	nCurWaitTime += nWaitTimeStep;
	//	if (nCurWaitTime > nTimeOut)
	//	{
	//		pRobotCtrl->m_cLog->Write("[�����Ѷ˵�]: �յ��жϵȴ��˶���ʱ��");
	//		return false;
	//	}
	//	bIsRunning = m_ptUnit->WorldIsRunning();
	//}

	//// �˶��� �� δ�����յ�
	//while (m_ptUnit->WorldIsRunning())
	//{
	//	m_Mutex_H_M.lock();
	//	int nCurCapWorldCoordNum = m_pTraceModel->ltCapWorldCoord.size();
	//	T_ROBOT_COORS tScanS = nCurCapWorldCoordNum > 0 ? m_pTraceModel->ltCapWorldCoord.front() : T_ROBOT_COORS();
	//	T_ROBOT_COORS tScanE = nCurCapWorldCoordNum > 0 ? m_pTraceModel->ltCapWorldCoord.back() : T_ROBOT_COORS();
	//	double dPointCloudLen = TwoPointDis(
	//		tScanE.dX + tScanE.dBX, tScanE.dY + tScanE.dBY, tScanE.dZ + tScanE.dBZ, 
	//		tScanS.dX + tScanS.dBX, tScanS.dY + tScanS.dBY, tScanS.dZ + tScanS.dBZ);
	//	if (dPointCloudLen < dMinProcessLen) // ���Ƴ���С����С������
	//	{
	//		m_Mutex_H_M.unlock();
	//		Sleep(nWaitTimeStep);
	//		continue;
	//	}
	//	long long lTime = XI_clock();

	//	// ���⿽������������
	//	int nPointCloudImageNum = m_pTraceModel->lnImageNo.size();
	//	int nImageNoS = m_pTraceModel->lnImageNo.front();
	//	int nImageNoE = m_pTraceModel->lnImageNo.back();
	//	int nTotalPtnNum = m_pTraceModel->ltPointCloud.size();
	//	Three_DPoint* pThreeDPoint = new Three_DPoint[nTotalPtnNum];
	//	std::list<Three_DPoint>::iterator iter = m_pTraceModel->ltPointCloud.begin();
	//	int nPtnNum = 0;
	//	for (; iter != m_pTraceModel->ltPointCloud.end(); iter++)
	//	{
	//		pThreeDPoint[nPtnNum++] = *(iter);
	//	}
	//	m_Mutex_H_M.unlock();

	//	double dMoveDis = TwoPointDis(
	//		tScanE.dX + tScanE.dBX, tScanE.dY + tScanE.dBY, tScanE.dZ + tScanE.dBZ,
	//		tPreScanE.dX + tPreScanE.dBX, tPreScanE.dY + tPreScanE.dBY, tPreScanE.dZ + tPreScanE.dBZ);
	//	if (nImageNoE <= nPreImageNoE || fabs(dMoveDis) < dProcStepDis) // ͼƬҪ���� �� ���dProcStepDis����һ�� �ص����� m_dSavePointCloudLen - dProcStepDis
	//	{
	//		DELETE_POINTER_ARRAY(pThreeDPoint);
	//		Sleep(nWaitTimeStep);
	//		continue;
	//	}
	//	nPreImageNoE = nImageNoE;
	//	tPreScanE = tScanE;

	//	// ����ÿ�δ���ĵ��Ƶ�����
	//	CString ss;
	//	ss.Format("%sImgNo_S%d-E%d_�ֲ�����.txt", m_sSaveFilePath, nImageNoS, nImageNoE);
	//	FILE* pf2 = fopen(ss.GetBuffer(), "w");
	//	for (int i = 0; i < nPtnNum; i++)
	//	{
	//		fprintf(pf2, "%d%11.3lf%11.3lf%11.3lf\n", i, pThreeDPoint[i].x, pThreeDPoint[i].y, pThreeDPoint[i].z);
	//	}
	//	fclose(pf2);

	//	bool bProcSuccess = false;
	//	GroovePointInfo* pGrooveInfo_Triangle = new GroovePointInfo;
	//	GroovePointInfo* pGrooveInfo_Trapezoid = new GroovePointInfo;
	//	cv::Vec6f ROI((float)INT_MIN, (float)INT_MAX, (float)INT_MIN, (float)INT_MAX, (float)INT_MIN, (float)INT_MAX);
	//	int refNorm = -1 * pRobotCtrl->m_nRobotInstallDir;

	//	try
	//	{
	//		//bProcSuccess = GrooveSolutionInterface((CvPoint3D64f*)pThreeDPoint, nPtnNum, ROI, 30.0, pGrooveInfo_Triangle, pGrooveInfo_Trapezoid/*, refNorm*/);
	//		pRobotCtrl->m_cLog->Write("[�����Ѷ˵�]: �¿���ȡ ����%d  %d  %d  %d ", nPtnNum,bProcSuccess, pGrooveInfo_Triangle->flag, pGrooveInfo_Trapezoid->flag);
	//	}
	//	catch (...)
	//	{
	//		CString s;
	//		s.Format("%sImgNo_S%d-E%d_�����쳣����.txt", m_sSaveFilePath, nImageNoS, nImageNoE);
	//		FILE* pf = fopen(s.GetBuffer(), "w");
	//		for (int i = 0; i < nPtnNum; i++)
	//		{
	//			fprintf(pf, "%d%11.3lf%11.3lf%11.3lf\n", i, pThreeDPoint[i].x, pThreeDPoint[i].y, pThreeDPoint[i].z);
	//		}
	//		fclose(pf);
	//		CheckMachineEmg(pRobotCtrl);
	//		XiMessageBox("%s [�����Ѷ˵�]:���ƴ����쳣! ͼ��%d-%d", pRobotCtrl->m_strRobotName, nImageNoS, nImageNoE);
	//		DELETE_POINTER_ARRAY(pThreeDPoint);
	//		DELETE_POINTER(pGrooveInfo_Triangle);
	//		DELETE_POINTER(pGrooveInfo_Trapezoid);
	//		break;
	//	}
	//	if (false == bProcSuccess)
	//	{
	//		DELETE_POINTER_ARRAY(pThreeDPoint);
	//		DELETE_POINTER(pGrooveInfo_Triangle);
	//		DELETE_POINTER(pGrooveInfo_Trapezoid);
	//		nErrorNum++;
	//		continue;
	//	}
	//	nErrorNum = 0;

	//	// ���浥�δ�����
	//	CString s;
	//	s.Format("%sImgNo_S%d-E%d_ProcRst%d_%d_%d.txt", m_sSaveFilePath, nImageNoS, nImageNoE, bProcSuccess, pGrooveInfo_Triangle->flag, pGrooveInfo_Trapezoid->flag);
	//	FILE* pf = fopen(s.GetBuffer(), "w");
	//	bool bSaveBothStartEnd = (0 >= m_pTraceModel->vtPointCloudEndPoints.size());
	//	SaveGrooveInfoToFile(pf, *pGrooveInfo_Triangle, bSaveBothStartEnd); // ��¼�������� m_pTraceModel->vtPointCloudEndPoints
	//	SaveGrooveInfoToFile(pf, *pGrooveInfo_Trapezoid, bSaveBothStartEnd);
	//	fclose(pf);

	//	pRobotCtrl->m_cLog->Write("[�����Ѷ˵�]: ���ƶ˵���ȡ�յ㣺ImgNo_S%d-E%d pGrooveInfo_Triangle->flag:%d ��ʱ:%dms",
	//		nImageNoS, nImageNoE, pGrooveInfo_Triangle->flag, XI_clock() - lTime);
	//	DELETE_POINTER_ARRAY(pThreeDPoint);
	//	DELETE_POINTER(pGrooveInfo_Triangle);
	//	DELETE_POINTER(pGrooveInfo_Trapezoid);
	//}
	//m_pRobotDriver->m_cLog->Write("[�¿�ɨ��]: FuncPointCloudProcess_H_M_Groove����!");
	return true;
}

bool CScanInitModule::FuncPointCloudProcess_H_M_DoubleVGroove(CRobotDriverAdaptor* pRobotCtrl, int nTimeOut/* = 2000*/)
{
	//m_pRobotDriver->m_cLog->Write("[�¿�ɨ��]: FuncPointCloudProcess_H_M_DoubleVGroove��ʼ!");
	//double dMinProcessLen = m_dSavePointCloudLen - 5.0; // ��С���ƴ����� ��С����󱣴泤��
	//int nWaitTimeStep = 100;			// �ȴ��˶����ʱ�䲽��
	//bool bIsRunning = false;
	//int nPreImageNoE = 0;
	//int nErrorNum = 0;					// ��������������
	//int nErrorThreshold = 3;			// ��������������ν�������
	//double dProcStepDis = 50.0;			// ���¶೤����ִ��һ�δ���
	//T_ROBOT_COORS tPreScanE = m_pRobotDriver->GetCurrentPos();			// ��¼ÿ�δ����ǵ����յ�����


	//std::vector<T_TEACH_RESULT> vtTeachResult;
	//	GroovePntsVectorInfo InfoVector = { {0},0 };
	//if (m_nLayerNo > 0)
	//{
	//	LoadTeachResult(m_nGroupNo, m_nLayerNo - 1, vtTeachResult);
	//	std::vector<GroovePointInfo> vtGroovePtnInfoBase(vtTeachResult[0].vtLeftPtns3D.size() / 5);
	//	for (int i = 5; i <= vtTeachResult[0].vtLeftPtns3D.size(); i += 5)
	//	{
	//		Trans3D(vtTeachResult[0].vtLeftPtns3D[i - 5], vtGroovePtnInfoBase[i / 5 - 1].GrooveTopStaPnt);
	//		Trans3D(vtTeachResult[0].vtLeftPtns3D[i - 4], vtGroovePtnInfoBase[i / 5 - 1].GroovePntStaPnt);
	//		Trans3D(vtTeachResult[0].vtLeftPtns3D[i - 3], vtGroovePtnInfoBase[i / 5 - 1].BottomCenterStaPnt);
	//		Trans3D(vtTeachResult[0].vtLeftPtns3D[i - 2], vtGroovePtnInfoBase[i / 5 - 1].FlatTopPntStaPnt);
	//		Trans3D(vtTeachResult[0].vtLeftPtns3D[i - 1], vtGroovePtnInfoBase[i / 5 - 1].FlatBottomStaPnt);
	//	}
	//	InfoVector.num = vtGroovePtnInfoBase.size() - 1;
	//	InfoVector.num = InfoVector.num > 30 ? 30 : InfoVector.num;
	//	//std::vector<GrooveCloudExtPointInfoM> vtInputInfo(vtGroovePtnInfoBase.size() - 1);
	//	for (int i = 0; i < InfoVector.num; i++)
	//	{
	//		InfoVector.outputGrooveInfo[i].TopLeftStaPnt = vtGroovePtnInfoBase[i].GrooveTopStaPnt;
	//		InfoVector.outputGrooveInfo[i].BottomLeftStaPnt = vtGroovePtnInfoBase[i].GroovePntStaPnt;
	//		InfoVector.outputGrooveInfo[i].BottomCenterStaPnt = vtGroovePtnInfoBase[i].BottomCenterStaPnt;
	//		InfoVector.outputGrooveInfo[i].TopRightStaPnt = vtGroovePtnInfoBase[i].FlatTopPntStaPnt;
	//		InfoVector.outputGrooveInfo[i].BottomRightStaPnt = vtGroovePtnInfoBase[i].FlatBottomStaPnt;

	//		InfoVector.outputGrooveInfo[i].TopLeftEndPnt = vtGroovePtnInfoBase[i + 1].GrooveTopStaPnt;
	//		InfoVector.outputGrooveInfo[i].BottomLeftEndPnt = vtGroovePtnInfoBase[i + 1].GroovePntStaPnt;
	//		InfoVector.outputGrooveInfo[i].BottomCenterEndPnt = vtGroovePtnInfoBase[i + 1].BottomCenterStaPnt;
	//		InfoVector.outputGrooveInfo[i].TopRightEndPnt = vtGroovePtnInfoBase[i + 1].FlatTopPntStaPnt;
	//		InfoVector.outputGrooveInfo[i].BottomRightEndPnt = vtGroovePtnInfoBase[i + 1].FlatBottomStaPnt;
	//	}
	//}

	//// �ȴ������˶�
	//int nCurWaitTime = 0;
	//bIsRunning = m_ptUnit->WorldIsRunning();
	//while (!bIsRunning)
	//{
	//	Sleep(nWaitTimeStep);
	//	nCurWaitTime += nWaitTimeStep;
	//	if (nCurWaitTime > nTimeOut)
	//	{
	//		pRobotCtrl->m_cLog->Write("[�����Ѷ˵�]: �յ��жϵȴ��˶���ʱ��");
	//		return false;
	//	}
	//	bIsRunning = m_ptUnit->WorldIsRunning();
	//}

	//// �˶��� �� δ�����յ�	
	//bool bLastProc = false;
	//while (true == (bIsRunning = m_ptUnit->WorldIsRunning()) || (false == bLastProc)) // ֹͣ�Ժ�Ҫ�ٴ���һ�ε��� ��֤ɨ�赽�Ĵ�������
	//{
	//	if (false == bIsRunning)
	//	{
	//		bLastProc = true;
	//		Sleep(2000);
	//	}
	//	m_Mutex_H_M.lock();
	////while (m_ptUnit->WorldIsRunning())
	////{
	////	m_Mutex_H_M.lock();
	//	int nCurCapWorldCoordNum = m_pTraceModel->ltCapWorldCoord.size();
	//	T_ROBOT_COORS tScanS = nCurCapWorldCoordNum > 0 ? m_pTraceModel->ltCapWorldCoord.front() : T_ROBOT_COORS();
	//	T_ROBOT_COORS tScanE = nCurCapWorldCoordNum > 0 ? m_pTraceModel->ltCapWorldCoord.back() : T_ROBOT_COORS();
	//	double dPointCloudLen = TwoPointDis(
	//		tScanE.dX + tScanE.dBX, tScanE.dY + tScanE.dBY, tScanE.dZ + tScanE.dBZ,
	//		tScanS.dX + tScanS.dBX, tScanS.dY + tScanS.dBY, tScanS.dZ + tScanS.dBZ);
	//	if (dPointCloudLen < dMinProcessLen) // ���Ƴ���С����С������
	//	{
	//		m_Mutex_H_M.unlock();
	//		Sleep(nWaitTimeStep);
	//		continue;
	//	}
	//	long long lTime = XI_clock();

	//	// ���⿽������������
	//	int nPointCloudImageNum = m_pTraceModel->lnImageNo.size();
	//	int nImageNoS = m_pTraceModel->lnImageNo.front();
	//	int nImageNoE = m_pTraceModel->lnImageNo.back();
	//	int nTotalPtnNum = m_pTraceModel->ltPointCloud.size();
	//	Three_DPoint* pThreeDPoint = new Three_DPoint[nTotalPtnNum];
	//	std::list<Three_DPoint>::iterator iter = m_pTraceModel->ltPointCloud.begin();
	//	int nPtnNum = 0;
	//	for (; iter != m_pTraceModel->ltPointCloud.end(); iter++)
	//	{
	//		pThreeDPoint[nPtnNum++] = *(iter);
	//	}
	//	m_Mutex_H_M.unlock();

	//	double dMoveDis = TwoPointDis(
	//		tScanE.dX + tScanE.dBX, tScanE.dY + tScanE.dBY, tScanE.dZ + tScanE.dBZ,
	//		tPreScanE.dX + tPreScanE.dBX, tPreScanE.dY + tPreScanE.dBY, tPreScanE.dZ + tPreScanE.dBZ);
	//	if (nImageNoE <= nPreImageNoE || fabs(dMoveDis) < dProcStepDis) // ͼƬҪ���� �� ���dProcStepDis����һ�� �ص����� m_dSavePointCloudLen - dProcStepDis
	//	{
	//		DELETE_POINTER_ARRAY(pThreeDPoint);
	//		Sleep(nWaitTimeStep);
	//		continue;
	//	}
	//	nPreImageNoE = nImageNoE;
	//	tPreScanE = tScanE;

	//	// ����ÿ�δ���ĵ��Ƶ�����f
	//	CString ss;
	//	ss.Format("%sImgNo_S%d-E%d_�ֲ�����.txt", m_sSaveFilePath, nImageNoS, nImageNoE);
	//	FILE* pf2 = fopen(ss.GetBuffer(), "w");
	//	for (int i = 0; i < nPtnNum; i++)
	//	{
	//		fprintf(pf2, "%d%11.3lf%11.3lf%11.3lf\n", i, pThreeDPoint[i].x, pThreeDPoint[i].y, pThreeDPoint[i].z);
	//	}
	//	fclose(pf2);

	//	bool bProcSuccess = false;

	//	GrooveCloudExtPointInfoM* pGrooveInfo = new GrooveCloudExtPointInfoM();
	//	//GrooveCloudExtPointInfo* pGrooveInfo = new GrooveCloudExtPointInfo();

	//	CvPoint3D32f FaceNorm;
	//	CvPoint3D32f MarchNorm;
	//	if (E_PLAT_GROOVE == m_eWeldSeamType)
	//	{
	//		FaceNorm = cvPoint3D32f(0.0, 0.0, 1.0);
	//		MarchNorm = cvPoint3D32f(0.0, 1.0, 0.0);
	//	}
	//	else
	//	{
	//		FaceNorm = cvPoint3D32f(0.0, -0.93, 0.34);
	//		MarchNorm = cvPoint3D32f(0.0, -0.34, -0.93);
	//	}

	//	cv::Vec6f ROI((float)INT_MIN, (float)INT_MAX, (float)INT_MIN, (float)INT_MAX, (float)INT_MIN, (float)INT_MAX);
	//	try
	//	{
	//		CvPoint3D32f* pPtns = new CvPoint3D32f[nPtnNum];
	//		for (int i = 0; i < nPtnNum; i++)
	//		{
	//			pPtns[i].x = pThreeDPoint[i].x;
	//			pPtns[i].y = pThreeDPoint[i].y;
	//			pPtns[i].z = pThreeDPoint[i].z;
	//		}
	//		int nUseHeight = 0 == m_nLayerNo ? 1 : 0;
	//		float fRefHeitht = 0 == m_nLayerNo ? 24 : 15;
	//		
	//		std::vector<T_TEACH_RESULT> vtTeachResult;
	//		if (2 == m_nLayerNo)
	//		{
	//			fRefHeitht = 11;
	//		}
	//		if (m_eWeldSeamType == E_STAND_GROOVE)
	//		{
	//			fRefHeitht -= 4;
	//		}
	//		if (m_nLayerNo < 2)
	//		{
	//			bProcSuccess = DoubleSlopesGrooveCloudExtPointInterfaceM(pPtns, nPtnNum, ROI, FaceNorm, MarchNorm, fRefHeitht, pGrooveInfo, nUseHeight);
	//			//bProcSuccess = DoubleSlopesGrooveCloudExtPointInterface(pPtns, nPtnNum, ROI, FaceNorm, MarchNorm, 24, pGrooveInfo);
	//		}
	//		else
	//		{
	//			bProcSuccess = DoubleSlopesGrooveCloudExtPointInterfaceM2(pPtns, nPtnNum, ROI, FaceNorm, MarchNorm, fRefHeitht, &InfoVector, pGrooveInfo);
	//		}
	//		DELETE_POINTER_ARRAY(pPtns);
	//		pRobotCtrl->m_cLog->Write("[�����Ѷ˵�]: �¿���ȡ ����%d  %d RefHeight:%.3lf UseHeight:%d", nPtnNum, bProcSuccess, fRefHeitht, nUseHeight);
	//	}	
	//	catch (...)
	//	{
	//		CString s;
	//		s.Format("%sImgNo_S%d-E%d_�����쳣����.txt", m_sSaveFilePath, nImageNoS, nImageNoE);
	//		FILE* pf = fopen(s.GetBuffer(), "w");
	//		for (int i = 0; i < nPtnNum; i++)
	//		{
	//			fprintf(pf, "%d%11.3lf%11.3lf%11.3lf\n", i, pThreeDPoint[i].x, pThreeDPoint[i].y, pThreeDPoint[i].z);
	//		}
	//		fclose(pf);
	//		CheckMachineEmg(pRobotCtrl);
	//		XiMessageBox("%s [�����Ѷ˵�]:���ƴ����쳣! ͼ��%d-%d", pRobotCtrl->m_strRobotName, nImageNoS, nImageNoE);
	//		DELETE_POINTER_ARRAY(pThreeDPoint);
	//		DELETE_POINTER(pGrooveInfo);
	//		break;
	//	}
	//	if (false == bProcSuccess)
	//	{
	//		DELETE_POINTER_ARRAY(pThreeDPoint);
	//		DELETE_POINTER(pGrooveInfo);
	//		nErrorNum++;
	//		continue;
	//	}
	//	nErrorNum = 0;

	//	// ���浥�δ�����
	//	CString s;
	//	s.Format("%sImgNo_S%d-E%d_ProcRst%d.txt", m_sSaveFilePath, nImageNoS, nImageNoE, bProcSuccess);
	//	FILE* pf = fopen(s.GetBuffer(), "w");
	//	bool bSaveBothStartEnd = (0 >= m_pTraceModel->vtPointCloudEndPoints.size());
	//	SaveGrooveInfoToFile(pf, *pGrooveInfo, bSaveBothStartEnd); // ��¼�������� m_pTraceModel->vtPointCloudEndPoints
	//	fclose(pf);

	//	pRobotCtrl->m_cLog->Write("[�����Ѷ˵�]: ���ƶ˵���ȡ�յ㣺ImgNo_S%d-E%d ��ʱ:%dms",
	//		nImageNoS, nImageNoE, XI_clock() - lTime);
	//	DELETE_POINTER_ARRAY(pThreeDPoint);
	//	DELETE_POINTER(pGrooveInfo);
	//}
	//m_pRobotDriver->m_cLog->Write("[�¿�ɨ��]: FuncPointCloudProcess_H_M_DoubleVGroove����!");
	return true;
}

bool CScanInitModule::FinalProc_Groove()
{
	//cv::Vec6f ROI((float)INT_MIN, (float)INT_MAX, (float)INT_MIN, (float)INT_MAX, (float)INT_MIN, (float)INT_MAX);
	//GroovePointInfo* pGrooveInfo_Triangle = new GroovePointInfo();
	//GroovePointInfo* pGrooveInfo_Trapezoid = new GroovePointInfo();
	//CString sPointFileName = m_sSaveFilePath + "AA_PointCloud.txt";
	//CString sProcRstFileName = m_sSaveFilePath + "AA_PointCloud_Rst_Point.txt";

	//bool bRst = false;/*GrooveSolutionInterface(sPointFileName.GetBuffer(), ROI, 30.0, pGrooveInfo_Triangle, pGrooveInfo_Trapezoid);*/

	//m_pRobotDriver->m_cLog->Write("FinalProc_Groove Rst: ������%d ��V:%d ˫V:%d", bRst, pGrooveInfo_Triangle->flag, pGrooveInfo_Trapezoid->flag);
	//
	//FILE *pf = fopen(sProcRstFileName, "w");
	//if (true == bRst)
	//{
	//	SaveGrooveInfoToFile(pf, *pGrooveInfo_Triangle, true);
	//	SaveGrooveInfoToFile(pf, *pGrooveInfo_Trapezoid, true);
	//	//AppendGrooveInfoPtn(m_pTraceModel->vtPointCloudEndPoints, pGrooveInfo_Triangle->GrooveTopStaPnt); // ��һ�α���������
	//	//AppendGrooveInfoPtn(m_pTraceModel->vtPointCloudEndPoints, pGrooveInfo_Triangle->GroovePntStaPnt);
	//	//AppendGrooveInfoPtn(m_pTraceModel->vtPointCloudEndPoints, pGrooveInfo_Triangle->BottomCenterStaPnt);
	//	//AppendGrooveInfoPtn(m_pTraceModel->vtPointCloudEndPoints, pGrooveInfo_Triangle->FlatTopPntStaPnt);
	//	//AppendGrooveInfoPtn(m_pTraceModel->vtPointCloudEndPoints, pGrooveInfo_Triangle->FlatBottomStaPnt);
	//	//AppendGrooveInfoPtn(m_pTraceModel->vtPointCloudEndPoints, pGrooveInfo_Triangle->GrooveTopEndPnt);// �����յ����
	//	//AppendGrooveInfoPtn(m_pTraceModel->vtPointCloudEndPoints, pGrooveInfo_Triangle->GroovePntEndPnt);
	//	//AppendGrooveInfoPtn(m_pTraceModel->vtPointCloudEndPoints, pGrooveInfo_Triangle->BottomCenterEndPnt);
	//	//AppendGrooveInfoPtn(m_pTraceModel->vtPointCloudEndPoints, pGrooveInfo_Triangle->FlatTopPntEndPnt);
	//	//AppendGrooveInfoPtn(m_pTraceModel->vtPointCloudEndPoints, pGrooveInfo_Triangle->FlatBottomEndPnt);
	//}
	//fclose(pf);

	//delete pGrooveInfo_Triangle;
	//delete pGrooveInfo_Trapezoid;
	return true;
}

bool CScanInitModule::LoadTeachResult(int nGroupNo, int nLayerNo, std::vector<T_TEACH_RESULT>& vtTeachResult)
{
	vtTeachResult.clear();
	int nDataColNum = 20;
	CString sFileName;
	CString sDataSavePath = OUTPUT_PATH + m_pRobotDriver->m_strRobotName + RECOGNITION_FOLDER;
	sFileName.Format("%s%d-%d-TeachResult.txt", sDataSavePath, nLayerNo, nGroupNo);

	FILE* pf = fopen(sFileName.GetBuffer(), "r");
	if (NULL == pf)
	{
		XUI::MesBox::PopOkCancel("���ص�{0}��ʾ�̽���ļ� {1} ��ʧ�ܣ�", nGroupNo, sFileName);
		return false;
	}

	T_TEACH_RESULT tTeachResult;
	T_TEACH_RESULT tTempTeachResult;
	int nPreReadIdx = -1;
	int nCurReadIdx = 0;
	int nPtnType = 0; // 0���� 1���� 2����
	CvPoint tKeyPtn2D;
	XI_POINT tKeyPtn3D;
	int nRst = fscanf(pf, "%d%d%lf%lf%lf%d%d%lf%lf%lf%lf%lf%lf%d%d%d%d%d%d%lf",
		&nCurReadIdx, &nPtnType, &tKeyPtn3D.x, &tKeyPtn3D.y, &tKeyPtn3D.z, &tKeyPtn2D.x, &tKeyPtn2D.y,
		&tTeachResult.tRobotCoors.dX, &tTeachResult.tRobotCoors.dY, &tTeachResult.tRobotCoors.dZ,
		&tTeachResult.tRobotCoors.dRX, &tTeachResult.tRobotCoors.dRY, &tTeachResult.tRobotCoors.dRZ,
		&tTeachResult.tRobotPulse.nSPulse, &tTeachResult.tRobotPulse.nLPulse, &tTeachResult.tRobotPulse.nUPulse,
		&tTeachResult.tRobotPulse.nRPulse, &tTeachResult.tRobotPulse.nBPulse, &tTeachResult.tRobotPulse.nTPulse,
		&tTeachResult.dExAxlePos);
	while (nDataColNum == nRst)
	{
		if (nPreReadIdx != nCurReadIdx)
		{
			if (nPreReadIdx >= 0)
			{
				vtTeachResult.push_back(tTeachResult);
			}
			tTeachResult.vtLeftPtns2D.clear();
			tTeachResult.vtLeftPtns3D.clear();
			tTeachResult.vtRightPtns2D.clear();
			tTeachResult.vtRightPtns3D.clear();
			nPreReadIdx = nCurReadIdx;
		}
		switch (nPtnType)
		{
		case 0:
			tTeachResult.tKeyPtn2D = tKeyPtn2D;
			tTeachResult.tKeyPtn3D = tKeyPtn3D;
			break;
		case 1:
			tTeachResult.vtLeftPtns2D.push_back(tKeyPtn2D);
			tTeachResult.vtLeftPtns3D.push_back(tKeyPtn3D);
			break;
		case 2:
			tTeachResult.vtRightPtns2D.push_back(tKeyPtn2D);
			tTeachResult.vtRightPtns3D.push_back(tKeyPtn3D);
			break;
		default:
			XUI::MesBox::PopOkCancel("���ص�{0}��ʾ�̽���ļ� {1} ��⴦�������������", nGroupNo, sFileName);
			fclose(pf);
			return false;
		}

		nRst = fscanf(pf, "%d%d%lf%lf%lf%d%d%lf%lf%lf%lf%lf%lf%d%d%d%d%d%d%lf",
			&nCurReadIdx, &nPtnType, &tKeyPtn3D.x, &tKeyPtn3D.y, &tKeyPtn3D.z, &tKeyPtn2D.x, &tKeyPtn2D.y,
			&tTempTeachResult.tRobotCoors.dX, &tTempTeachResult.tRobotCoors.dY, &tTempTeachResult.tRobotCoors.dZ,
			&tTempTeachResult.tRobotCoors.dRX, &tTempTeachResult.tRobotCoors.dRY, &tTempTeachResult.tRobotCoors.dRZ,
			&tTempTeachResult.tRobotPulse.nSPulse, &tTempTeachResult.tRobotPulse.nLPulse, &tTempTeachResult.tRobotPulse.nUPulse,
			&tTempTeachResult.tRobotPulse.nRPulse, &tTempTeachResult.tRobotPulse.nBPulse, &tTempTeachResult.tRobotPulse.nTPulse,
			&tTempTeachResult.dExAxlePos);
		if (-1 == nCurReadIdx) // ����ɨ�赽�Ķ˵����� �� ��1������λ��tEndPtn3D��
		{
			vtTeachResult[0].tEndPtn3D = tKeyPtn3D;
			break;
		}
		tTeachResult.tRobotCoors.dX = tTempTeachResult.tRobotCoors.dX;
		tTeachResult.tRobotCoors.dY = tTempTeachResult.tRobotCoors.dY;
		tTeachResult.tRobotCoors.dZ = tTempTeachResult.tRobotCoors.dZ;
		tTeachResult.tRobotCoors.dRX = tTempTeachResult.tRobotCoors.dRX;
		tTeachResult.tRobotCoors.dRY = tTempTeachResult.tRobotCoors.dRY;
		tTeachResult.tRobotCoors.dRZ = tTempTeachResult.tRobotCoors.dRZ;
		tTeachResult.tRobotPulse.nSPulse = tTempTeachResult.tRobotPulse.nSPulse;
		tTeachResult.tRobotPulse.nLPulse = tTempTeachResult.tRobotPulse.nLPulse;
		tTeachResult.tRobotPulse.nUPulse = tTempTeachResult.tRobotPulse.nUPulse;
		tTeachResult.tRobotPulse.nRPulse = tTempTeachResult.tRobotPulse.nRPulse;
		tTeachResult.tRobotPulse.nBPulse = tTempTeachResult.tRobotPulse.nBPulse;
		tTeachResult.tRobotPulse.nTPulse = tTempTeachResult.tRobotPulse.nTPulse;
		tTeachResult.dExAxlePos = tTempTeachResult.dExAxlePos;
	}
	fclose(pf);
	vtTeachResult.push_back(tTeachResult); // �������һ��ʾ��λ�ý��

	if ((EOF != nRst) && (nDataColNum != nRst))
	{
		XUI::MesBox::PopOkCancel("���ص�{0}��ʾ�̽�������ļ� {1} ��⵽���������ݣ�", nGroupNo, sFileName);
		vtTeachResult.clear();
		return false;
	}
	return true;
}

//void CScanInitModule::SaveGrooveInfoToFile(FILE* pf, const GroovePointInfo& tGrooveInfo, bool bSaveBoth)
//{
//	if (false == tGrooveInfo.flag) return;
//
//	
//	SavePtn3D(pf, tGrooveInfo.GrooveTopEndPnt);
//	SavePtn3D(pf, tGrooveInfo.GroovePntEndPnt);
//	SavePtn3D(pf, tGrooveInfo.BottomCenterEndPnt);
//	SavePtn3D(pf, tGrooveInfo.FlatTopPntEndPnt);
//	SavePtn3D(pf, tGrooveInfo.FlatBottomEndPnt);
//	SavePtn3D(pf, tGrooveInfo.GrooveTopStaPnt);
//	SavePtn3D(pf, tGrooveInfo.GroovePntStaPnt);
//	SavePtn3D(pf, tGrooveInfo.BottomCenterStaPnt);
//	SavePtn3D(pf, tGrooveInfo.FlatTopPntStaPnt);
//	SavePtn3D(pf, tGrooveInfo.FlatBottomStaPnt);
//	if (bSaveBoth)
//	{
//		AppendGrooveInfoPtn(m_pTraceModel->vtPointCloudEndPoints, tGrooveInfo.GrooveTopEndPnt);// �����յ����
//		AppendGrooveInfoPtn(m_pTraceModel->vtPointCloudEndPoints, tGrooveInfo.GroovePntEndPnt);
//		AppendGrooveInfoPtn(m_pTraceModel->vtPointCloudEndPoints, tGrooveInfo.BottomCenterEndPnt);
//		AppendGrooveInfoPtn(m_pTraceModel->vtPointCloudEndPoints, tGrooveInfo.FlatTopPntEndPnt);
//		AppendGrooveInfoPtn(m_pTraceModel->vtPointCloudEndPoints, tGrooveInfo.FlatBottomEndPnt);
//	}
//	
//	AppendGrooveInfoPtn(m_pTraceModel->vtPointCloudEndPoints, tGrooveInfo.GrooveTopStaPnt); // ��һ�α���������
//	AppendGrooveInfoPtn(m_pTraceModel->vtPointCloudEndPoints, tGrooveInfo.GroovePntStaPnt);
//	AppendGrooveInfoPtn(m_pTraceModel->vtPointCloudEndPoints, tGrooveInfo.BottomCenterStaPnt);
//	AppendGrooveInfoPtn(m_pTraceModel->vtPointCloudEndPoints, tGrooveInfo.FlatTopPntStaPnt);
//	AppendGrooveInfoPtn(m_pTraceModel->vtPointCloudEndPoints, tGrooveInfo.FlatBottomStaPnt);
//}
//
//void CScanInitModule::SaveGrooveInfoToFile(FILE* pf, const GrooveCloudExtPointInfo& tGrooveInfo, bool bSaveBoth)
//{
//	SavePtn3D(pf, tGrooveInfo.TopLeftStaPnt);
//	SavePtn3D(pf, tGrooveInfo.BottomLeftStaPnt);
//	SavePtn3D(pf, tGrooveInfo.BottomCenterStaPnt);
//	SavePtn3D(pf, tGrooveInfo.TopRightStaPnt);
//	SavePtn3D(pf, tGrooveInfo.BottomRightStaPnt);
//	SavePtn3D(pf, tGrooveInfo.TopLeftEndPnt);
//	SavePtn3D(pf, tGrooveInfo.BottomLeftEndPnt);
//	SavePtn3D(pf, tGrooveInfo.BottomCenterEndPnt);
//	SavePtn3D(pf, tGrooveInfo.TopRightEndPnt);
//	SavePtn3D(pf, tGrooveInfo.BottomRightEndPnt);
//	if (bSaveBoth)
//	{
//		AppendGrooveInfoPtn(m_pTraceModel->vtPointCloudEndPoints, tGrooveInfo.TopLeftStaPnt); // ��һ�α���������
//		AppendGrooveInfoPtn(m_pTraceModel->vtPointCloudEndPoints, tGrooveInfo.BottomLeftStaPnt);
//		AppendGrooveInfoPtn(m_pTraceModel->vtPointCloudEndPoints, tGrooveInfo.BottomCenterStaPnt);
//		AppendGrooveInfoPtn(m_pTraceModel->vtPointCloudEndPoints, tGrooveInfo.TopRightStaPnt);
//		AppendGrooveInfoPtn(m_pTraceModel->vtPointCloudEndPoints, tGrooveInfo.BottomRightStaPnt);
//	}
//	AppendGrooveInfoPtn(m_pTraceModel->vtPointCloudEndPoints, tGrooveInfo.TopLeftEndPnt);// �����յ����
//	AppendGrooveInfoPtn(m_pTraceModel->vtPointCloudEndPoints, tGrooveInfo.BottomLeftEndPnt);
//	AppendGrooveInfoPtn(m_pTraceModel->vtPointCloudEndPoints, tGrooveInfo.BottomCenterEndPnt);
//	AppendGrooveInfoPtn(m_pTraceModel->vtPointCloudEndPoints, tGrooveInfo.TopRightEndPnt);
//	AppendGrooveInfoPtn(m_pTraceModel->vtPointCloudEndPoints, tGrooveInfo.BottomRightEndPnt);
//}
//
//void CScanInitModule::SaveGrooveInfoToFile(FILE* pf, const GrooveCloudExtPointInfoM& tGrooveInfo, bool bSaveBoth)
//{
//	SavePtn3D(pf, tGrooveInfo.TopLeftStaPnt);
//	SavePtn3D(pf, tGrooveInfo.BottomLeftStaPnt);
//	SavePtn3D(pf, tGrooveInfo.BottomCenterStaPnt);
//	SavePtn3D(pf, tGrooveInfo.TopRightStaPnt);
//	SavePtn3D(pf, tGrooveInfo.BottomRightStaPnt);
//	SavePtn3D(pf, tGrooveInfo.TopLeftEndPnt);
//	SavePtn3D(pf, tGrooveInfo.BottomLeftEndPnt);
//	SavePtn3D(pf, tGrooveInfo.BottomCenterEndPnt);
//	SavePtn3D(pf, tGrooveInfo.TopRightEndPnt);
//	SavePtn3D(pf, tGrooveInfo.BottomRightEndPnt);
//	if (bSaveBoth)
//	{
//		AppendGrooveInfoPtn(m_pTraceModel->vtPointCloudEndPoints, tGrooveInfo.TopLeftStaPnt); // ��һ�α���������
//		AppendGrooveInfoPtn(m_pTraceModel->vtPointCloudEndPoints, tGrooveInfo.BottomLeftStaPnt);
//		AppendGrooveInfoPtn(m_pTraceModel->vtPointCloudEndPoints, tGrooveInfo.BottomCenterStaPnt);
//		AppendGrooveInfoPtn(m_pTraceModel->vtPointCloudEndPoints, tGrooveInfo.TopRightStaPnt);
//		AppendGrooveInfoPtn(m_pTraceModel->vtPointCloudEndPoints, tGrooveInfo.BottomRightStaPnt);
//	}
//	AppendGrooveInfoPtn(m_pTraceModel->vtPointCloudEndPoints, tGrooveInfo.TopLeftEndPnt);// �����յ����
//	AppendGrooveInfoPtn(m_pTraceModel->vtPointCloudEndPoints, tGrooveInfo.BottomLeftEndPnt);
//	AppendGrooveInfoPtn(m_pTraceModel->vtPointCloudEndPoints, tGrooveInfo.BottomCenterEndPnt);
//	AppendGrooveInfoPtn(m_pTraceModel->vtPointCloudEndPoints, tGrooveInfo.TopRightEndPnt);
//	AppendGrooveInfoPtn(m_pTraceModel->vtPointCloudEndPoints, tGrooveInfo.BottomRightEndPnt);
//}

void CScanInitModule::SavePtn3D(FILE *pf, CvPoint3D32f tPtn)
{
	fprintf(pf, "%11.3lf%11.3lf%11.3lf\n", tPtn.x, tPtn.y, tPtn.z);
}

void CScanInitModule::AppendGrooveInfoPtn(std::vector<Three_DPoint>& vtThree_Points, const CvPoint3D32f& tPtn)
{
	Three_DPoint tThreePtn;
	tThreePtn.x = tPtn.x;
	tThreePtn.y = tPtn.y;
	tThreePtn.z = tPtn.z;
	vtThree_Points.push_back(tThreePtn);
}

BOOL CScanInitModule::JudgeFilterDataValid(CRobotDriverAdaptor *pRobotDriver, CvPoint3D64f tNewPoint, T_ROBOT_COORS tLastValidPoint, T_ROBOT_COORS tPenultPoint) { return FALSE; }

double CScanInitModule::SmoothRz(double dRz, std::vector<double> &vdRecordRz, int nRecordNum)
{
    double dRst = 0;
    double dNewRz = dRz;
    int nSize = vdRecordRz.size();
    dNewRz = fmod(dNewRz, 360.0); // ��360
    if (0.0 > dNewRz)
    {
        dNewRz += 360.0; // 0-360
    }
    if (nSize>1)
    {
        double dRZ = vdRecordRz.at(nSize - 1) - dNewRz;

        if (dRZ>180)
        {
            dNewRz += 360;
        }
        else if (dRZ < -180)
        {
            dNewRz -= 360;
        }
        WriteLog("ƽ������dRZ��%lf dNewRz:%lf", dRZ, dNewRz);
    }
    vdRecordRz.push_back(dNewRz);
    int nItemNum = vdRecordRz.size();
    if (nItemNum > nRecordNum)
    {
        vdRecordRz.erase(vdRecordRz.begin(), vdRecordRz.begin() + (nItemNum - nRecordNum));
        nItemNum = vdRecordRz.size();
    }
    double dSum = 0;
    CString str = "";
    CString sTemp = "";
    for (int i = 0; i < nItemNum; i++)
    {
        dSum += vdRecordRz[i];
        sTemp.Format("%s %.3lf ", str, vdRecordRz[i]);
        str = sTemp;
    }
    dRst = dSum / (double)nItemNum;
    WriteLog("��ǰ����Ԫ������%d  ֵ��%s ��������%.3lf", nItemNum, str, dRst);
    return dRst;
}

double CScanInitModule::DirAngleToRz(double dDirAngle)
{
    // Rz - Rz0 = dir * (dirAngle - dirAngle0)
    double dRz = (dDirAngle - m_dWeldNorAngleInHome) + m_tRobotHomeCoors.dRZ;
    if (dRz>180.0)
    {
        dRz -= 360.0;
    }
    else if (dRz<-180.0)
    {
        dRz += 360.0;
    }
    return dRz;
}

void CScanInitModule::SetWeldParam(double dHandEyeDis, double dMeasureDisThreshold, double dPlatWeldRx, double dPlatWeldRy,
    double dStandWeldRx, double dStandWeldRy, double dTransitionsRx, double dTransitionsRy, double dWeldNorAngleInHome,
	BOOL* pIsNaturalPop, int nRobotInsterDir)
{
    m_dHandEyeDis = dHandEyeDis;
    m_dMeasureDisThreshold = dMeasureDisThreshold;
    m_dPlatWeldRx = dPlatWeldRx;
    m_dPlatWeldRy = dPlatWeldRy;
    m_dStandWeldRx = dStandWeldRx;
    m_dStandWeldRy = dStandWeldRy;
    m_dTransitionsRx = dTransitionsRx;
    m_dTransitionsRy = dTransitionsRy;
    m_dWeldNorAngleInHome = dWeldNorAngleInHome;
}

BOOL CScanInitModule::GetScanData(CRobotDriverAdaptor *pRobotDriver, std::vector<XI_POINT> &tScanData, std::vector<T_ROBOT_COORS> &tScanRobotData,double &dDirAngle)
{
    tScanData.clear();
	tScanRobotData.clear();
	CString strPath, strPath2;
	strPath.Format(".\\WeldData\\%s\\%d_CoutStartPointInfoFileInWorldSmooth2.txt", pRobotDriver->m_strRobotName, m_eEndPointType);
	strPath2.Format(".\\WeldData\\%s\\%d_StartPointInfoFileInRobotCoors.txt", pRobotDriver->m_strRobotName, m_eEndPointType);
	std::ifstream cinData(strPath);
	std::ifstream cinRobotcoors(strPath2);
	if (!CheckFileExists(strPath))
	{
		//���޸�
		XUI::MesBox::PopInfo("{0}���ݼ���ʧ��", strPath.GetBuffer());
		//XiMessageBox("%s ���ݼ���ʧ��", strPath);
		return FALSE;
	}
	if (!CheckFileExists(strPath2))
	{

		//���޸�
		XUI::MesBox::PopInfo("{0}���ݼ���ʧ��", strPath2.GetBuffer());
		//XiMessageBox("%s ���ݼ���ʧ��", strPath2);
		return FALSE;
	}
	int n = 0, nPointNo = 0;
    XI_POINT tPoint;
	T_ROBOT_COORS tRobot;
	double dFirstRZ = 0.0;
    while (!cinData.eof())
    {
        cinData >> n >> tPoint.x >> tPoint.y >> tPoint.z;
		cinRobotcoors >> tRobot.dX >> tRobot.dY >> tRobot.dZ >> tRobot.dRX >> tRobot.dRY >> tRobot.dRZ;
		if (0 == nPointNo )
		{
			dFirstRZ = tRobot.dRZ;
		}
		else
		{
			tRobot.dRZ = dFirstRZ;
		}
        tScanData.push_back(tPoint);
		tScanRobotData.push_back(tRobot);
		nPointNo++;
    }
	cinRobotcoors.close();
	cinData.close();
    //������ʼ�η���
    std::vector<XI_POINT> vtScanPoints;//��¼��ʼ��ɨ���
    std::vector<double> vdWorkPieceDir;
    vdWorkPieceDir.clear();
    vtScanPoints.clear();
    vtScanPoints = tScanData;
	T_LINE_PARA	tLine;
	if (!CalcLineParamRansac(tLine, vtScanPoints, 0.7))
	{
		return FALSE;
	}
	dDirAngle = atan2(tLine.dDirY, tLine.dDirX) * 180 / 3.1415926/* - 90 * pRobotDriver->m_nRobotInstallDir*/;

    dDirAngle = DirAngleToRz(dDirAngle);
    pRobotDriver->m_cLog->Write(" Start nDir:%d ",dDirAngle);
    return TRUE;
}
BOOL CScanInitModule::CompDoubleFunc(double a, double b)
{
    return a < b;
}

void CScanInitModule::CheckMachineEmg(CRobotDriverAdaptor *pRobotDriver)
{
	pRobotDriver->m_eThreadStatus = INCISEHEAD_THREAD_STATUS_STOPPED;
	int n = 0;
	pRobotDriver->HoldOn();
	pRobotDriver->ServoOff();
	//pRobotDriver->m_cStepMoveControl.EmgStop();
	//pRobotDriver->m_cStepMoveControl.StopStep();
	XUI::MesBox::PopError("��е�ۼ�ͣ");
}

void CScanInitModule::CallJudgeEndpointFun()
{
	AfxBeginThread(ThreadJudgeIsWeldCompen, this);
}

UINT CScanInitModule::ThreadJudgeIsWeldCompen(void *pParam) 
{ 
	CScanInitModule* pcMyObj = (CScanInitModule*)pParam;
	while (true)
	{
		if (pcMyObj->m_ptUnit->GetRobotCtrl()->m_eThreadStatus == INCISEHEAD_THREAD_STATUS_STOPPED)
		{
			pcMyObj->m_ptUnit->GetRobotCtrl()->m_cLog->Write("ThreadJudgeIsWeldCompen�߳̽���");
			return false;
		}
		if (pcMyObj->m_pTraceModel->m_bIfWeldToEndPoint)
		{
			pcMyObj->m_ptUnit->GetRobotCtrl()->m_cLog->Write("ThreadJudgeIsWeldCompen����");
			return true;
		}
		pcMyObj->JudgeIsWeldCompen(pcMyObj->m_ptUnit->GetRobotCtrl(), pcMyObj->m_pTraceModel->m_tRealEndpointCoor);
		Sleep(50);
	}
	return true;
}

BOOL CScanInitModule::JudgeIsWeldCompen(CRobotDriverAdaptor *pRobotDriver, T_ROBOT_COORS tEndpointCoor) 
{
	double dEndTrackDis = 0.0;
	double dExAxisPos = m_ptUnit->GetExPositionDis(m_ptUnit->m_nTrackAxisNo);
	T_ROBOT_COORS tRobotCoor = pRobotDriver->GetCurrentPos();
#ifdef SINGLE_ROBOT
	double dWorldDis = TwoPointDis(tEndpointCoor.dX, tEndpointCoor.dY + tEndpointCoor.dBY
		, tRobotCoor.dX, tRobotCoor.dY + dExAxisPos);
	double dAxisY = fabs(tEndpointCoor.dBY - dExAxisPos);
#else
	// tEndpointCoor ��dX dY dZ dBX �����յ���������
	double dWorldDis = TwoPointDis(
		tEndpointCoor.dX + tEndpointCoor.dBX, tEndpointCoor.dY,
		tRobotCoor.dX + dExAxisPos, tRobotCoor.dY);
	double dAxisY = fabs(tEndpointCoor.dBX - dExAxisPos);
#endif
	pRobotDriver->m_cLog->Write("���ҵ���β��1��%.3lf %.3lf %.3lf %.3lf %.3lf %.3lf",
		dWorldDis, dAxisY, m_pTraceModel->dHandEyeDis, m_pTraceModel->m_dChangeAngleTheshold,
		tEndpointCoor.dX + tEndpointCoor.dBX, tEndpointCoor.dY + tEndpointCoor.dBY);
	if (dWorldDis < 2.5 && dAxisY < 1.5)
	{
		m_pTraceModel->m_bIfWeldToEndPoint = TRUE;
		pRobotDriver->m_cLog->Write("���ҵ���β�㣺%.3lf %.3lf", dWorldDis, dAxisY);
	}

	if (m_pTraceModel->m_dChangeAngleTheshold > m_pTraceModel->dHandEyeDis)
	{
		dEndTrackDis = m_pTraceModel->m_dChangeAngleTheshold;
	}
	else
	{
		dEndTrackDis = m_pTraceModel->dHandEyeDis;
	}

	if (dWorldDis < dEndTrackDis && !m_pTraceModel->m_bIsCloseTrackThread /*||
		m_pTraceModel->m_dCurrentWeldLen>(m_pTraceModel->m_dWeldLen - m_pTraceModel->dHandEyeDis)*/)//��������,��ǰ���ӳ��ȴ����ܳ��ȼ�ȥ���۳��Ƚ�������
	{
		m_pTraceModel->m_bIsCloseTrackThread = true;
		pRobotDriver->m_cLog->Write("��������:dEndTrackDis:%lf dWorldDis:%lf", dEndTrackDis, dWorldDis);
	}
	return true;
}

BOOL CScanInitModule::GetEndSectionCoors(CRobotDriverAdaptor *pRobotDriver, T_ROBOT_COORS &tEndpointCoor) 
{ 
	int nSize = pRobotDriver->m_vtWeldLineInWorldPoints.size();
	T_ROBOT_COORS tEndpoint = pRobotDriver->m_vtWeldLineInWorldPoints.at(nSize - 1);
	T_ROBOT_COORS tEndpointTemp = tEndpointCoor;

	pRobotDriver->m_cLog->Write("��¼��β���ݣ� %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf",
		tEndpointCoor.dX, tEndpointCoor.dY, tEndpointCoor.dZ, tEndpointCoor.dRZ,tEndpointCoor.dBY,
		tEndpoint.dX, tEndpoint.dY,tEndpoint.dZ, tEndpoint.dRZ, tEndpoint.dBY);

	//ȡ���漸������Ϸ���
	vector<XI_POINT> vtPoint;
	for (int n = nSize - 10; n < nSize; n++)
	{
		XI_POINT tp = { pRobotDriver->m_vtWeldLineInWorldPoints.at(n).dX + pRobotDriver->m_vtWeldLineInWorldPoints.at(n).dBX,
			pRobotDriver->m_vtWeldLineInWorldPoints.at(n).dY + pRobotDriver->m_vtWeldLineInWorldPoints.at(n).dBY,
			pRobotDriver->m_vtWeldLineInWorldPoints.at(n).dZ };
		vtPoint.push_back(tp);
	}
	T_LINE_PARA	tLine;
	if (!CalcLineParamRansac(tLine, vtPoint, 0.7))
	{
		return false;
	}
	double dNormal = atan2(tLine.dDirY, tLine.dDirX) * 180 / 3.1415926 - 90 * pRobotDriver->m_nRobotInstallDir;

	//���½�β��������
	//m_pTraceModel->m_dWorkPieceDir = dNormal;

	double dDisError = TwoPointDis(tEndpointCoor.dX + tEndpointCoor.dBX, tEndpointCoor.dY + tEndpointCoor.dBY, tEndpointCoor.dZ,
		pRobotDriver->m_vtWeldLineInWorldPoints.at(nSize - 1).dX + pRobotDriver->m_vtWeldLineInWorldPoints.at(nSize - 1).dBX,
		pRobotDriver->m_vtWeldLineInWorldPoints.at(nSize - 1).dY + pRobotDriver->m_vtWeldLineInWorldPoints.at(nSize - 1).dBY,
		pRobotDriver->m_vtWeldLineInWorldPoints.at(nSize - 1).dZ);
	//���ƫ����
	double dInitComp = m_pTraceModel->m_dGunToEyeCompenX;// ����ڼ�
	double dAbjustHeight = m_pTraceModel->m_dGunToEyeCompenZ;// ���ϲ��� ���²���
	if (E_CLOSEDARC == m_pTraceModel->m_tWorkPieceType)
	{
		dInitComp = 0;
		dAbjustHeight = 0;
	}
	if (dDisError > 130)
	{
		tEndpointCoor = pRobotDriver->m_vtWeldLineInWorldPoints.at(nSize - 1);
		//����ǰֹͣ�򲻱��Ǻ���������ֹͣ���ӣ�����ʾ
		//m_pTraceModel->m_eWrapAngleType = E_WRAPANGLE_EMPTY_SINGLE;
		//SetIntValFun(pRobotDriver, 7, 0);
		pRobotDriver->m_cLog->Write(" ��β���ж�ʧ������һ��Ϊ��β��ֹͣ�豸����������������Ϊ�����ǣ�%lf ", dDisError);

	}
	else
	{
		// �����˲��� �����յ� ȡ�յ� Start  ���㷨�յ��ȡ������Ҫ�޸ģ�
		T_ROBOT_COORS tTempCoors;
		TrackSmooth_PntArray tTrackPtnArray;
		tTrackPtnArray.arraySize_ = 0;
		tTrackPtnArray.pnts_ = NULL;
		// ��ʼ����β���˲�����
		TrackSmooth_Init(m_ptUnit->m_nRobotSmooth, m_pTraceModel->m_dMoveStepDis,
			pRobotDriver->m_vtWeldLineInWorldPoints[nSize - 1].dX + pRobotDriver->m_vtWeldLineInWorldPoints[nSize - 1].dBX,
			pRobotDriver->m_vtWeldLineInWorldPoints[nSize - 1].dY + pRobotDriver->m_vtWeldLineInWorldPoints[nSize - 1].dBY,
			pRobotDriver->m_vtWeldLineInWorldPoints[nSize - 1].dZ);
		if (E_CLOSEDARC == m_pTraceModel->m_tWorkPieceType)
		{
			tEndpointCoor.dX += dInitComp * CosD(dNormal);
			tEndpointCoor.dY += dInitComp * SinD(dNormal);
			tEndpointCoor.dZ += dAbjustHeight;
		}
		else
		{
			XI_POINT tEndpoint = { tEndpointCoor.dX + tEndpointCoor.dBX, tEndpointCoor.dY + tEndpointCoor.dBY, tEndpointCoor.dZ };
			// ͶӰ
			XI_POINT tProjectpoint;
			PointtoLineProjection(tLine, tEndpoint, tProjectpoint);
			tEndpointCoor.dX = tProjectpoint.x - tEndpointCoor.dBX;
			tEndpointCoor.dY = tProjectpoint.y - tEndpointCoor.dBY;
			tEndpointCoor.dZ = tProjectpoint.z;

			pRobotDriver->m_cLog->Write("��β��ͶӰ:%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf \n",
				tEndpointCoor.dX, tEndpointCoor.dY, tEndpointCoor.dZ, tEndpointCoor.dBY, tEndpointCoor.dY + tEndpointCoor.dBY);
		}
		// ��ȡ��β������
		TrackSmooth_PushNextPoint(m_ptUnit->m_nRobotSmooth, tEndpointCoor.dX + tEndpointCoor.dBX,
			tEndpointCoor.dY + tEndpointCoor.dBY, tEndpointCoor.dZ, &tTrackPtnArray);
		for (int n = 0; n < tTrackPtnArray.arraySize_; n++)
		{
			tTempCoors = pRobotDriver->m_vtWeldLineInWorldPoints[nSize - 1];
			if (m_pTraceModel->m_bIfOpenExternal)
			{
#ifdef SINGLE_ROBOT
				tTempCoors.dX = tTrackPtnArray.pnts_[n].x_;
				tTempCoors.dBY = (tTrackPtnArray.pnts_[n].y_ - tTempCoors.dY);
				tTempCoors.dZ = tTrackPtnArray.pnts_[n].z_;
#else

				tTempCoors.dBX = (tTrackPtnArray.pnts_[n].x_ - tTempCoors.dX);
				tTempCoors.dY = tTrackPtnArray.pnts_[n].y_;
				tTempCoors.dZ = tTrackPtnArray.pnts_[n].z_;
#endif			
			}
			else
			{
				tTempCoors.dX = tTrackPtnArray.pnts_[n].x_;
				tTempCoors.dY = tTrackPtnArray.pnts_[n].y_ - tEndpointTemp.dBY;
				tTempCoors.dZ = tTrackPtnArray.pnts_[n].z_;
				tTempCoors.dBY = tEndpointTemp.dBY;
			}
			pRobotDriver->m_vtWeldLineInWorldPoints.push_back(tTempCoors);
			m_pTraceModel->m_vtWeldLinePointType.push_back(E_WELD_TRACK);
			SaveRobotCoor(m_pRecordTheoryPoint, tTempCoors, E_WELD_TRACK);//������������
			pRobotDriver->m_cLog->Write("TrackSmooth_GetEndPoint1:%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf \n",
				tTempCoors.dX, tTempCoors.dY, tTempCoors.dZ, tTempCoors.dBY, dNormal);
		}
		TrackSmooth_FreePntArray(&tTrackPtnArray);

		//��ȡ��β�㣬����βʱ���ã�����һ��ľ���С�� moveMinDis 
		TrackSmooth_GetEndPoint(m_ptUnit->m_nRobotSmooth, &tTrackPtnArray);
		for (int n = 0; n < tTrackPtnArray.arraySize_; n++)
		{
			tTempCoors = pRobotDriver->m_vtWeldLineInWorldPoints[nSize - 1];
			if (m_pTraceModel->m_bIfOpenExternal)
			{
#ifdef SINGLE_ROBOT
				tTempCoors.dX = tTrackPtnArray.pnts_[n].x_;
				tTempCoors.dBY = (tTrackPtnArray.pnts_[n].y_ - tTempCoors.dY);
				tTempCoors.dZ = tTrackPtnArray.pnts_[n].z_;
#else
				tTempCoors.dBX = (tTrackPtnArray.pnts_[n].x_ - tTempCoors.dX);
				tTempCoors.dY = tTrackPtnArray.pnts_[n].y_;
				tTempCoors.dZ = tTrackPtnArray.pnts_[n].z_;
#endif	
			}
			else
			{
				tTempCoors.dX = tTrackPtnArray.pnts_[n].x_;
				tTempCoors.dY = tTrackPtnArray.pnts_[n].y_ - tEndpointTemp.dBY;
				tTempCoors.dZ = tTrackPtnArray.pnts_[n].z_;
				tTempCoors.dBY = tEndpointTemp.dBY;
			}

			pRobotDriver->m_vtWeldLineInWorldPoints.push_back(tTempCoors);
			m_pTraceModel->m_vtWeldLinePointType.push_back(E_WELD_TRACK);
			SaveRobotCoor(m_pRecordTheoryPoint, tTempCoors, E_WELD_TRACK);//������������

			pRobotDriver->m_cLog->Write("TrackSmooth_GetEndPoint:%11.3lf%11.3lf%11.3lf%11.3lf\n",
				tTempCoors.dX, tTempCoors.dY, tTempCoors.dZ, tTempCoors.dBY);
		}
		if (tTrackPtnArray.arraySize_ > 0)
		{
			tEndpointCoor = tTempCoors;
		}
		else
		{
			tEndpointCoor = pRobotDriver->m_vtWeldLineInWorldPoints[pRobotDriver->m_vtWeldLineInWorldPoints.size() - 1];
		}
		TrackSmooth_FreePntArray(&tTrackPtnArray);
		//���½�β����
		m_pTraceModel->m_tRealEndpointCoor = tEndpointCoor;
		m_pTraceModel->m_tRealEndpointCoor.dRZ = DirAngleToRz(dNormal);

		nSize = pRobotDriver->m_vtWeldLineInWorldPoints.size();
		//����̬
		if (m_pTraceModel->m_nCloseTrackingPos > 0)
		{
			double DAngle = 45;/*tEndpointTemp.dRZ - tEndpoint.dRZ;
			DAngle = DAngle > 180 ? DAngle - 360 : DAngle;
			DAngle = DAngle < -180 ? DAngle + 360 : DAngle;*/
			double dStepDis = DAngle / m_pTraceModel->m_nCloseTrackingPos;
			if (fabs(dStepDis) > 10 || fabs(DAngle) > 90)
			{
				XUI::MesBox::PopOkCancel("�����������⣺{0} {1}", dStepDis, DAngle);
				return false;
			}
			for (int n = m_pTraceModel->m_nCloseTrackingPos; n > 0; n--)
			{
				pRobotDriver->m_vtWeldLineInWorldPoints.at(nSize - (m_pTraceModel->m_nCloseTrackingPos - n) - 1).dRZ -= dStepDis * n * pRobotDriver->m_nRobotInstallDir;
				pRobotDriver->m_cLog->Write("��β�仯��̬��%lf %lf",
					pRobotDriver->m_vtWeldLineInWorldPoints.at(nSize - (m_pTraceModel->m_nCloseTrackingPos - n) - 1).dRZ, dStepDis);
			}
		}
	}
	return true;
}

double CScanInitModule::RecordRunTime(CRobotDriverAdaptor *pRobotDriver, long ltimeBegin, CString str)
{
	long long dEnd;
	dEnd = XI_clock();
	double time = (double)(dEnd - ltimeBegin) / CLOCKS_PER_SEC;
	pRobotDriver->m_cLog->Write("%s ʱ��ͳ�ƣ�%s ʱ��:%11.3lf", pRobotDriver->m_strRobotName, str, time);
	return time;
}

UINT CScanInitModule::ThreadJudgeEndCaptureRobot(void *pParam)
{
    CScanInitModule *pcMyObj = (CScanInitModule *)pParam;
    pcMyObj->JudgeEndCaptureRobot(pcMyObj->m_pRobotDriver);
    return 0;
}

UINT CScanInitModule::ThreadJudgeEndProcLineGroupStand(void *pParam)
{
    CScanInitModule *pcMyObj = (CScanInitModule *)pParam;
    pcMyObj->JudgeEndProcLineGroupStand(pcMyObj->m_pRobotDriver);
    return 0;
}


void CScanInitModule::JudgeEndCaptureRobot(CRobotDriverAdaptor *pRobotDriver) 
{ 

	T_ROBOT_COORS tRobotCurCoord;
	T_ANGLE_PULSE tRobotCurPulses;
	T_CART_COOR dMachineCoors;//��¼������
	long long nTime = XI_clock();
	m_bEndPointCapCur = false;
	CDHGigeImageCapture* pDHCamDrv = (CDHGigeImageCapture*)m_ptUnit->GetCameraCtrl(m_ptUnit->m_nTrackCameraNo);
	//����ѭ����Ψһ����Ϊ m_bCameraFindWeldStart == TRUE
	while (FALSE == m_pTraceModel->m_bCameraFindWeldEnd)
	{
		if (pRobotDriver->m_eThreadStatus == INCISEHEAD_THREAD_STATUS_STOPPED) break;
		long long lStartTime = XI_clock();
		nTime = XI_clock();
		//��ȡ��ͼλ��
		tRobotCurCoord = pRobotDriver->GetCurrentPos();
		tRobotCurPulses = pRobotDriver->GetCurrentPulse();
		//���¿����ʱ
		dMachineCoors.dX = tRobotCurCoord.dBX;
		dMachineCoors.dY = tRobotCurCoord.dBY;
		dMachineCoors.dZ = tRobotCurCoord.dBZ;

		m_ptStartPointInfo[m_nEndPointCapCursor % MAX_ARRAY_NUM_START].tRobotCoors = tRobotCurCoord;
		m_ptStartPointInfo[m_nEndPointCapCursor % MAX_ARRAY_NUM_START].tRobotPulse = tRobotCurPulses;
		m_ptStartPointInfo[m_nEndPointCapCursor % MAX_ARRAY_NUM_START].dMachineCoors = dMachineCoors;
		//��ȡͼƬ
		pDHCamDrv->CaptureImage(m_ptStartPointInfo[m_nEndPointCapCursor % MAX_ARRAY_NUM_START].img, 1);
		//cvCopyImage(, m_ptStartPointInfo[m_nEndPointCapCursor % MAX_ARRAY_NUM_START].img);
		Sleep(100);
		m_nEndPointCapCursor++;
		pRobotDriver->m_cLog->Write("��βɨ���ͼ%d��ʱ%dms", m_nEndPointCapCursor, XI_clock() - nTime);
	}
	m_bEndPointCapCur = true;
	pRobotDriver->m_cLog->Write("%s ��β��ͼ�߳��˳���%d", m_ptUnit->GetUnitName(), m_bEndPointCapCur);

}

void CScanInitModule::TranslateDrawingToImageParam(CRobotDriverAdaptor *pRobotDriver) { return; }

//����С�����յ㺯��						
void CScanInitModule::JudgeEndProcLineGroupStand(CRobotDriverAdaptor * pRobotCtrl)
{ 
	CString strRobot = pRobotCtrl->m_strRobotName;
	CString strPath;
	strPath.Format(".\\WeldData\\WeldRobot%d\\", pRobotCtrl->m_nRobotNo);
	string strPathAdr = strPath;
	//pRobotCtrl->m_eStartAndEnd = GROUP_STAND_DIP_NS::E_PIECE_END;
	double dWeldVel = 400.0;
	bool bIfClacAngle = FALSE;
	T_ABS_POS_IN_BASE tPointAbsCoordInBase;
	std::vector<XI_POINT> vtWeldLineInWorldPoints;
	std::vector<XI_POINT> vtWeldLineInWorldCoor;
	vtWeldLineInWorldPoints.clear();
	vtWeldLineInWorldCoor.clear();
//	E_CAM_ID eCamId;
	CString strSavePath = "";
	CString strImgPathName = "";
	CString strSavePath1 = "";

	T_SAVE_IMAGE tSaveImage /*= new T_SAVE_IMAGE*/;
	/*for (int nS = 0; nS<SAVE_IMAGE_THREAD_MUN; nS++)
	{
		tSaveImage.bSaveImageStaus[nS] = false;
	}
	tSaveImage.nImageNo = 0;*/

	strSavePath1.Format("%s%s\\EndSrc\\ForwardScan", OUTPUT_PATH, pRobotCtrl->m_strRobotName);
	strSavePath.Format("%s%s\\EndSrc\\", OUTPUT_PATH, pRobotCtrl->m_strRobotName);
	//DeleteFolder(strSavePath1);

	double dCurPosX = 0, dCurPosY = 0, dCamerCoorX = 0, dCamerCoorY = 0, dCamerCoorZ = 0,
		dOffsetEndPoint = 0, dTwoPointDis = 100.0;
	CvPoint cpKeyPoint;
	long long m_nTimeStart = XI_clock();
	m_nStartPointShape = 5;
	TranslateDrawingToImageParam();
	long long tClock;
	while (FALSE == m_pTraceModel->m_bCameraFindWeldEnd)
	{
		//CHECK_STOP_BREAK(pRobotCtrl);
		m_nTimeStart = XI_clock();
		// �����ȡ������������Ч���Ż�(��ע�͵�)
		if (m_nEndPointCapCursor < 2) // �Ӳɼ���������ͼ��ʼ
		{
			continue;
		}
		m_nEndPointProcCursor = m_nEndPointCapCursor - 1;
		tSaveImage.nImageNo++;

		/*if (pRobotCtrl->m_nEndSaveImage)
		{
			strImgPathName.Format("%sForwardScan\\%d.jpg", strSavePath, tSaveImage.nImageNo);
			tSaveImage.cstrId = strImgPathName;
			tSaveImage.pImage = m_ptStartPointInfo[pRobotCtrl->m_nEndPointProcCursor % MAX_ARRAY_NUM_START].img;
			AfxBeginThread(ThreadSaveImage, &tSaveImage);
		}*/
		tClock = XI_clock();
		m_pTraceModel->m_bCameraFindWeldEnd = m_pTraceImgProcess->IfPieceStartOrEndPoint
		(
			m_ptStartPointInfo[m_nEndPointProcCursor % MAX_ARRAY_NUM_START].img, dWeldVel,
			dOffsetEndPoint, cpKeyPoint, GROUP_STAND_DIP_NS::E_PIECE_END, m_ePieceClass,
			m_iBaseLineLenInZone, m_iMoveLineLenInZone, m_iRoiWidSize, m_iIfEndConditionIdx
		);
		pRobotCtrl->m_cLog->Write("%s ��βͼ���� %d %d %d", strRobot, m_pTraceModel->m_bCameraFindWeldEnd, cpKeyPoint.x, cpKeyPoint.y);

		if (TRUE == m_pTraceModel->m_bCameraFindWeldEnd)
		{
			pRobotCtrl->m_cLog->Write("%s Come In end: %4d %4d", strRobot, m_nEndPointProcCursor, m_pTraceModel->m_bCameraFindWeldEnd);
			int nImageBackFindSum = 0;
			CString strImgPath;
			bool bCameraFindWeldEnd = true;
			while (bCameraFindWeldEnd)
			{
				//CHECK_STOP_BREAK(pRobotCtrl);
				m_nEndPointProcCursor = (m_nEndPointProcCursor - 1 + MAX_ARRAY_NUM_START) % MAX_ARRAY_NUM_START;
				bCameraFindWeldEnd = m_pTraceImgProcess->IfPieceStartOrEndPoint
				(
					m_ptStartPointInfo[m_nEndPointProcCursor % MAX_ARRAY_NUM_START].img, dWeldVel,
					dOffsetEndPoint, cpKeyPoint, GROUP_STAND_DIP_NS::E_PIECE_END, m_ePieceClass,
					m_iBaseLineLenInZone, m_iMoveLineLenInZone, m_iRoiWidSize, m_iIfEndConditionIdx
				);
				if (false/*pRobotCtrl->m_nEndSaveImage*/)
				{
					tSaveImage.nImageNo = nImageBackFindSum;
					strImgPath.Format("%sReverseScan\\%d.jpg", strSavePath, tSaveImage.nImageNo);
					tSaveImage.cstrId = strImgPath;
					tSaveImage.pImage = m_ptStartPointInfo[m_nEndPointProcCursor % MAX_ARRAY_NUM_START].img;
					AfxBeginThread(ThreadSaveImage, &tSaveImage);
				}
				nImageBackFindSum++;
				m_ptStartPointInfo[m_nEndPointProcCursor % MAX_ARRAY_NUM_START].sata = bCameraFindWeldEnd;
				m_ptStartPointInfo[m_nEndPointProcCursor % MAX_ARRAY_NUM_START].nScanImageNo = MAX_ARRAY_NUM_START + nImageBackFindSum;

				if (nImageBackFindSum > MAX_ARRAY_NUM_START)
				{
					CheckMachineEmg(pRobotCtrl);
					pRobotCtrl->m_cLog->Write("%s ͼ�����δ�ҵ��յ㣬����ֹͣ", strRobot);
					XUI::MesBox::PopInfo("{0}�Ż�����ͼ�����δ�ҵ��յ㣬����ֹͣ", pRobotCtrl->m_nRobotNo);
					break;
				}
			}
			// ��һ�λ����Ѿ��ҵ�һ��ԭ���̵��յ� ���㱣����Ϊ�Ա�
			m_ptUnit->TranImageToBase(m_ptUnit->m_nTrackCameraNo, cpKeyPoint, m_ptStartPointInfo[m_nEndPointProcCursor % MAX_ARRAY_NUM_START].tRobotCoors,
				m_ptStartPointInfo[m_nEndPointProcCursor % MAX_ARRAY_NUM_START].tRobotPulse, &tPointAbsCoordInBase);

			pRobotCtrl->m_cLog->Write("%s ԭ����ɨ�赽���յ㣺%11.3lf%11.3lf%11.3lf nImageBackFindSum:%d", strRobot,
				tPointAbsCoordInBase.tWeldLinePos.x,
				tPointAbsCoordInBase.tWeldLinePos.y + m_ptStartPointInfo[m_nEndPointProcCursor % MAX_ARRAY_NUM_START].dMachineY,
				tPointAbsCoordInBase.tWeldLinePos.z, nImageBackFindSum);

			m_nEndPointProcCursor = (m_nEndPointProcCursor - 1 + MAX_ARRAY_NUM_START) % MAX_ARRAY_NUM_START;
			pRobotCtrl->m_cLog->Write("%s Come Out: %4d %4d", strRobot, m_nEndPointProcCursor, m_pTraceModel->m_bCameraFindWeldEnd);
			m_pTraceModel->m_bCameraFindWeldEnd = TRUE;
		}
		if (TRUE == m_pTraceModel->m_bCameraFindWeldEnd)
		{
			break;
		}
		Sleep(10);
		DoEvent();
	}
	pRobotCtrl->m_cLog->Write("%s ��ʼ����ͼ�߳̽�β", pRobotCtrl->m_strRobotName);
	if (TRUE == m_pTraceModel->m_bCameraFindWeldEnd)
	{
		pRobotCtrl->m_cLog->Write("%s ����ҵ��յ�: m_bCameraFindWeldEnd=%d cpKeyPoint:%d %d", strRobot, m_pTraceModel->m_bCameraFindWeldEnd,cpKeyPoint.x, cpKeyPoint.y);
		pRobotCtrl->m_cLog->Write("%s �յ�ɼ�����:%d �յ㴦������:%d ", strRobot, m_nEndPointCapCursor, m_nEndPointProcCursor);

		m_ptUnit->TranImageToBase(m_ptUnit->m_nTrackCameraNo, cpKeyPoint, m_ptStartPointInfo[m_nEndPointProcCursor % MAX_ARRAY_NUM_START].tRobotCoors,
			m_ptStartPointInfo[m_nEndPointProcCursor % MAX_ARRAY_NUM_START].tRobotPulse, &tPointAbsCoordInBase);
		// �ֽ�β��ֵ
		T_ROBOT_COORS tRealEndpoint;
		tRealEndpoint.dX = 1 == m_ptUnit->m_nTrackCameraNo ? tPointAbsCoordInBase.tWeldLinePos.x + m_ptStartPointInfo[m_nEndPointProcCursor % MAX_ARRAY_NUM_START].tRobotCoors.dBX : tPointAbsCoordInBase.tWeldLinePos.x;
		tRealEndpoint.dY = 2 == m_ptUnit->m_nTrackCameraNo ? tPointAbsCoordInBase.tWeldLinePos.y + m_ptStartPointInfo[m_nEndPointProcCursor % MAX_ARRAY_NUM_START].tRobotCoors.dBY : tPointAbsCoordInBase.tWeldLinePos.y;
		tRealEndpoint.dZ = tPointAbsCoordInBase.tWeldLinePos.z;
		m_pTraceModel->m_tRealEndpointCoor = tRealEndpoint;
		// ��β���˲�
		GetEndSectionCoors(pRobotCtrl, m_pTraceModel->m_tRealEndpointCoor);
		// �жϽ�β�߳�
		CallJudgeEndpointFun();

	}
	return;
}

BOOL CScanInitModule::WeldRealEndPointSingle(CRobotDriverAdaptor *pRobotDriver) { return FALSE; }

T_ROBOT_COORS CScanInitModule::StartWrapAngleFun(CRobotDriverAdaptor *pRobotDriver, double dWorkPieceDir, T_ROBOT_COORS tRobotEndPoint) { return T_ROBOT_COORS(); }

BOOL CScanInitModule::EndWrapAngleFun(CRobotDriverAdaptor *pRobotDriver, double dWorkPieceDir, T_ROBOT_COORS tRobotEndPoint) { return FALSE; }

BOOL CScanInitModule::EndSafeBackFun(CRobotDriverAdaptor *pRobotDriver, double dWorkPieceDir, T_ROBOT_COORS tRobotEndPoint) { return FALSE; }

BOOL CScanInitModule::CompareCoordValue(CRobotDriverAdaptor *pRobotDriver, int nPIndex, T_ROBOT_COORS tRobotCoor, double nErrorVal)
{
    //��ȡ
    UINT unCount = 1;
    MP_VAR_INFO tVarInfo[1];
    tVarInfo[0].usIndex = nPIndex;
    tVarInfo[0].usType = MP_RESTYPE_VAR_ROBOT;
    LONG lPosVarData[10];
	long long tTime = XI_clock();
    pRobotDriver->GetMultiPosVar(unCount, tVarInfo, lPosVarData);
    pRobotDriver->m_cLog->Write("22 %d", XI_clock() - tTime);
    //д��
    if (MP_ROBO_COORD == (lPosVarData[0] & 0x3F))//ֱ������
    {

        pRobotDriver->m_cLog->Write("�Ա����꣺%d  %.3lf   %.3lf   %.3lf   %.3lf   %.3lf   %.3lf",
            nPIndex, lPosVarData[2] / 1000.0, lPosVarData[3] / 1000.0, lPosVarData[4] / 1000.0,
            lPosVarData[5] / 10000.0, lPosVarData[6] / 10000.0, lPosVarData[7] / 10000.0);
        T_ROBOT_COORS tCoor(lPosVarData[2] / 1000.0, lPosVarData[3] / 1000.0, lPosVarData[4] / 1000.0,
            lPosVarData[5] / 10000.0, lPosVarData[6] / 10000.0, lPosVarData[7] / 10000.0 ,0,0,0);
        ;
        if (!pRobotDriver->CompareCoords(tRobotCoor, tCoor, 1, 3, 1))
        {
            return FALSE;
        }
    }
    else if (MP_PULSE_COORD == (lPosVarData[0] & 0x3F))//�ؽ�����
    {
        pRobotDriver->m_cLog->Write("�Ա���������:%d %10ld %10ld %10ld %10ld %10ld %10ld", nPIndex, lPosVarData[2], lPosVarData[3],
            lPosVarData[4], lPosVarData[5], lPosVarData[6], lPosVarData[7]);
        return FALSE;
    }
    return TRUE;
}

void CScanInitModule::CalcNormalOffsetInBase(T_ROBOT_COORS &tRobotWrapData, double dDis, double dZDis, double dAngleVer)
{
    double dAngle = dAngleVer;
    tRobotWrapData.dX = tRobotWrapData.dX + dDis*CosD(dAngle);
    tRobotWrapData.dY = tRobotWrapData.dY + dDis*SinD(dAngle);
    tRobotWrapData.dZ = tRobotWrapData.dZ + dZDis;

}
double CScanInitModule::ControlRobotAngle(double dAngle)
{
    if (dAngle>180)
    {
        dAngle -= 360;
    }
    else if (dAngle<(-180))
    {
        dAngle += 360;
    }
    return  dAngle;
}

T_ROBOT_COORS CScanInitModule::CalcWrapOffsetFun(double dWrapdParalle, double dVertical, double dWorkPieceDir, T_ROBOT_COORS tRobotEndPoint, int nRobotInsterDir)
{
    T_ROBOT_COORS tRobotPoint = tRobotEndPoint;
    tRobotPoint.dX = tRobotPoint.dX + dWrapdParalle*CosD(dWorkPieceDir);
    tRobotPoint.dY = tRobotPoint.dY + dWrapdParalle*SinD(dWorkPieceDir);

    tRobotPoint.dX = tRobotPoint.dX + dVertical*CosD(dWorkPieceDir + 90 * nRobotInsterDir);
    tRobotPoint.dY = tRobotPoint.dY + dVertical*SinD(dWorkPieceDir + 90 * nRobotInsterDir);

    return tRobotPoint;

}
void CScanInitModule::Initvariable() { return; }

void CScanInitModule::LoadEquipmentParam()
{
	COPini opini;
	opini.SetFileName(DATA_PATH + m_pRobotDriver->m_strRobotName + SYETEM_PARAM_FILE);
	opini.SetSectionName("EquipmentParam");
	opini.ReadString("RobotInstallDir", &m_pRobotDriver->m_nRobotInstallDir);
	//opini.ReadString("TrackCamBothFlip", (int*)m_peScanFlipMode);
	//opini.ReadString("GunAngle", &m_dGunAngle);
	//opini.ReadString("GunLaserAngle", &m_dGunLaserAngle);
	//opini.ReadString("GunCameraAngle", &m_dGunCameraAngle);
	//opini.ReadString("RotateToCamRxDir", &m_dRotateToCamRxDir);
	opini.ReadString("HandEyeDis", &m_dHandEyeDis);
	opini.ReadString("MeasureDisThreshold", &m_dMeasureDisThreshold);
	opini.ReadString("FlatWeldRx", &m_dPlatWeldRx);
	opini.ReadString("FlatWeldRy", &m_dPlatWeldRy);
	opini.ReadString("NormalWeldRx", &m_dNormalWeldRx);
	opini.ReadString("NormalWeldRy", &m_dNormalWeldRy);
	opini.ReadString("StandWeldRx", &m_dStandWeldRx);
	opini.ReadString("StandWeldRy", &m_dStandWeldRy);
	opini.ReadString("TransitionsRx", &m_dTransitionsRx);
	opini.ReadString("TransitionsRy", &m_dTransitionsRy);
	opini.ReadString("WeldNorAngleInHome", &m_dWeldNorAngleInHome);
	/*opini.ReadString("EndpointSearchDis", &m_dEndpointSearchDis);
	opini.ReadString("GunDownBackSafeDis", &m_dGunDownBackSafeDis);
	opini.ReadString("ShortSeamThreshold", &m_dShortSeamThreshold);*/

	//m_pRobotDriver->m_nRobotInstallDir = m_nRobotInstallDir;
	m_pRobotDriver->RobotKinematics(m_pRobotDriver->m_tHomePulse, m_pRobotDriver->m_tTools.tGunTool, m_tRobotHomeCoors);
}

//����ͼ�����⺯��,�����ṹ��Ϊ T_IMAGE_PROCINPUT_PARAM
void CScanInitModule::ImgProcInsertPairInMap(LPVOID pParam)
{
	T_IMAGE_PROCINPUT_PARAM *tParam = (T_IMAGE_PROCINPUT_PARAM*)pParam;
	tParam->pMap->emplace(tParam->pairInput);
	return;
}


void CScanInitModule::SaveIsTrackCompleteFlag(CRobotDriverAdaptor* pRobotDriver, int TrackFlag,double dVoerLength)
{
	CString str;
	str.Format(".\\ConfigFiles\\%s\\Pause.ini", pRobotDriver->m_strRobotName);
	COPini opini;
	opini.SetFileName(str);
	opini.SetSectionName("PauseInfo");
	opini.WriteString("TrackCompleteFlag", TrackFlag);
	opini.WriteString("VoerLength", dVoerLength);
}
void CScanInitModule::LoadIsTrackCompleteFlag(CRobotDriverAdaptor* pRobotDriver, int& TrackFlag)
{
	CString str;
	str.Format(".\\ConfigFiles\\%s\\Pause.ini", pRobotDriver->m_strRobotName);
	COPini opini;
	opini.SetFileName(str);
	opini.SetSectionName("PauseInfo");
	opini.ReadString("TrackCompleteFlag", &TrackFlag);
}

void CScanInitModule::SaveEndpointData(CString RobotName, int nGroupNo, int nEndpointNo, XI_POINT tRealEndpoint,bool bShape)
{
	// ��ʼ�δ洢�������ݣ���β�δ洢��β��
	XI_POINT tPoint;
	double dExPos = m_ptUnit->GetExPositionDis(m_ptUnit->m_nMeasureAxisNo);
	vector<XI_POINT> vtEndpoint, vtRealEndpoint(0);
	if (0 == nEndpointNo)
	{
		//vtRealEndpoint = GetVtImgProcResult3D();
		XI_POINT tPtn;
		for (int i = 0; i < m_pTraceModel->vtPointCloudEndPoints.size(); i++)
		{
			tPtn.x = m_pTraceModel->vtPointCloudEndPoints[i].x;
			tPtn.y = m_pTraceModel->vtPointCloudEndPoints[i].y;
			tPtn.z = m_pTraceModel->vtPointCloudEndPoints[i].z;
			//tPtn.x = 1 == m_ptUnit->m_nMeasureAxisNo ? tPtn.x - dExPos : tPtn.x; // ԭ�˴���Ҫ����������
			//tPtn.y = 2 == m_ptUnit->m_nMeasureAxisNo ? tPtn.y - dExPos : tPtn.y;
			//tPtn.z = 3 == m_ptUnit->m_nMeasureAxisNo ? tPtn.z - dExPos : tPtn.z;
			vtRealEndpoint.push_back(tPtn);
		}
	}
	vtRealEndpoint.push_back(/*GetDynamicScanEndPoint()*/tRealEndpoint);
	double dLength = 0;
	if (vtRealEndpoint.size()>1)
	{		
		if (!bShape)
		{
		reverse(vtRealEndpoint.begin(), vtRealEndpoint.end());
		T_LINE_PARA tLineParam;
		if (!CalcLineParamRansac(tLineParam,vtRealEndpoint, 0.7)) { return ; }
		vtRealEndpoint.clear();
		for (size_t i = 0; i < 75; i++)
		{
			XI_POINT tPoint;
			tPoint.x = tRealEndpoint.x + i * tLineParam.dDirX * 2.0;
			tPoint.y = tRealEndpoint.y + i * tLineParam.dDirY * 2.0;
			tPoint.z = tRealEndpoint.z + i * tLineParam.dDirZ * 2.0;
			vtRealEndpoint.push_back(tPoint);
		}		
		reverse(vtRealEndpoint.begin(), vtRealEndpoint.end());
		}
		
		dLength = TwoPointDis(vtRealEndpoint.at(0).x, vtRealEndpoint.at(0).y, vtRealEndpoint.at(0).z,
			vtRealEndpoint.at(vtRealEndpoint.size() - 1).x, vtRealEndpoint.at(vtRealEndpoint.size() - 1).y, vtRealEndpoint.at(vtRealEndpoint.size() - 1).z);
	}
	// ���˵����꣬ȥ���ظ�������С����
	for (int nPointNo = 0; nPointNo < vtRealEndpoint.size(); nPointNo++)
	{
		if (nPointNo > 0){
			// ������㳤��
			double dStartDis = TwoPointDis(vtRealEndpoint[nPointNo].x, vtRealEndpoint[nPointNo].y, vtRealEndpoint[nPointNo].z,
				vtRealEndpoint[0].x, vtRealEndpoint[0].y, vtRealEndpoint[0].z);
			// ����һ���㳤��
			double dDis = TwoPointDis(vtRealEndpoint[nPointNo].x, vtRealEndpoint[nPointNo].y, vtRealEndpoint[nPointNo].z,tPoint.x, tPoint.y, tPoint.z);
			if (dDis < 1.0 || dDis >5.0 || dStartDis > dLength)
			{ continue; }
			tPoint = vtRealEndpoint[nPointNo];
		}
		else{
			tPoint = vtRealEndpoint[nPointNo];
		}
		vtEndpoint.push_back(tPoint);
	}
	// ����β���Ƿ����
	dLength = TwoPointDis(tRealEndpoint.x, tRealEndpoint.y, tRealEndpoint.z,
		vtEndpoint.at(vtEndpoint.size() - 1).x, vtEndpoint.at(vtEndpoint.size() - 1).y, vtEndpoint.at(vtEndpoint.size() - 1).z);
	if (dLength > 0.1)
	{
		vtEndpoint.push_back(tRealEndpoint);
	}

	// ����ⲿ�ᣬ�ϲ���������
	for (int i = 0; i < vtEndpoint.size(); i++)
	{
		vtEndpoint[i].x = 1 == m_ptUnit->m_nMeasureAxisNo ? vtEndpoint[i].x + dExPos : vtEndpoint[i].x;
		vtEndpoint[i].y = 2 == m_ptUnit->m_nMeasureAxisNo ? vtEndpoint[i].y + dExPos : vtEndpoint[i].y;
		vtEndpoint[i].z = 3 == m_ptUnit->m_nMeasureAxisNo ? vtEndpoint[i].z + dExPos : vtEndpoint[i].z;
	}

	// ��ת��ʼ������
	if (0 == nEndpointNo)
	{
		reverse(vtEndpoint.begin(), vtEndpoint.end());
	}
	
	// �洢����
	CString strFileName;
	strFileName.Format("%s%s%sEndpointCoors-%d-%d.txt", OUTPUT_PATH,RobotName,RECOGNITION_FOLDER, nGroupNo, nEndpointNo);
	SavePointsData(vtEndpoint, strFileName);
}





