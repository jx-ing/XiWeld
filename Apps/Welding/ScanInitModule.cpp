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

	// 图像使用 默认参数
	m_ePieceClass = GROUP_STAND_DIP_NS::E_TI_PIECE;
	m_iBaseLineLenInZone = 50;
	m_iMoveLineLenInZone = 30;
	m_iRoiWidSize = 100;
	m_iIfEndConditionIdx = 1;

	m_bScanStartTwoLineOnStandBoard = FALSE;

	m_bCameraFindWeldStart = FALSE;
	m_nEndPointCapCursor = 0;  //端点采图序号
	m_nEndPointProcCursor = 0; //处理图号

    m_bEndPointCapCur = FALSE;     //判断采图线程是否结束
    m_bEndPointProcess = FALSE;   //判断处理线程是否结束
    m_bRealTimeTrackstate = FALSE; //跟踪线程状态
    m_bDynamicCapturestate = FALSE;//采图线程状态
    m_bDynamicProcstate = FALSE;   //处理线程状态
	m_bOpenTrackingStatus = FALSE;	

	//多线程输出数组
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
	//先释放图片堆内存
	ClearStartPointInfoArray();
	//再释放图片信息数组堆内存
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
	WriteLog("扫描步距:%lf 距离 %lf m_nTheoryNum %d",m_dStepDis,dDist,m_nTheoryNum);
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
		TranslateDrawingToImageParam(); // 根据图纸数据初始化 图像要用到的数据 和 需要特殊处理用到的数据
	}
	else
	{
		ResumeDrawingToImageParam();
	}
	WriteLog("SetWhichPlaceClass函数 eWhichPlaceClass:%d m_bFixedPointScan:%d", m_ePieceClass, m_bFixedPointScan);
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
	 * 起点终点形状	图纸表示  ePieceClass参数
	 * 
	 *    T型		   0	  E_TI_PIECE	
	 *    水孔		   4	  E_TI_PIECE
	 *   三面相交	   5	  E_TI_PIECE
	 *  起点立板悬空   6	  E_TI_PIECE
	 *   底板R型	   3	  E_R_PIECE
	 *   立板R型	   2	  E_R_PIECE
	 *  F型(直角边)	   1	  E_R_PIECE
	 * 
	 *	 类型		iBaseLineLenInZone参数		iMoveLineLenInZone参数		iRoiWidSize参数		iIfEndConditionIdx参数
	 * 
	 *	无坡口				50							30						  100					1
	 *	有坡口				30							125						  250					5
	 *	过水孔				50							50						  250					10
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
		XUI::MesBox::PopOkCancel("起点形状错误：m_nStartPointShape = {0}", m_nStartPointShape);
	}
	
	if ((4 == m_nStartPointShape)) // 过水孔或者三面相交 都使用过水孔参数
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
	else if((3 == m_nStartPointShape) || (2 == m_nStartPointShape) || (1 == m_nStartPointShape)|| (0 == m_nStartPointShape))// 目前没有区分是否有坡口，都使用无坡口参数
	{
		m_iBaseLineLenInZone = 50;
		m_iMoveLineLenInZone = 30;
		m_iRoiWidSize = 100;
		m_iIfEndConditionIdx = 1;
		m_bScanStartTwoLineOnStandBoard = FALSE;

		m_dScanStartExtendLength = 0;
	}
	else if (6 == m_nStartPointShape)//立板悬空
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
	std::ofstream fileCurnt(strPath+"CoutStartPointInfoFileInWorldSmooth2.txt");//输出滤波后数据*/
	std::ofstream fileCurnt3(".\\CoutStartPointInfoFileInWorldSmooth3.txt");//输出滤波后数据*/
	int nSeq;
	int nDelPointTolNum = 0;
    double dMachineY = 0.0;
    if (m_bIfOpenExternal)
    {
        ////////dMachineY = m_MoveCtrlModule.GetPositionDis(pRobotDriver->m_nAXIS_X);
    }
	pRobotDriver->m_cLog->Write("对枪后大车坐标：%lf", dMachineY);
	T_ROBOT_COORS tStartCoors;
	m_nStartIfWrap = 1;
	// 如果是自由端点搜起点，删掉几个点(一般第一个点偏外)
	if (0 == m_nStartIfWrap)//单枪起点不包脚
	{
		nDelPointTolNum = 5;
	}
	for (int nDelPointNum = 0; nDelPointNum < nDelPointTolNum; nDelPointNum++)
	{
		fileCurntLugNo >> nSeq>>tStartCoors.dX>>tStartCoors.dY >> tStartCoors.dZ;
		pRobotDriver->m_cLog->Write("删除第%d个点：%d, %.3lf, %.3lf, %.3lf",nDelPointNum,nSeq,tStartCoors.dX,tStartCoors.dY,tStartCoors.dZ);
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
	//计算方向
	//FitLineParamRansac(vtReferCoors,40);
	int width= vtReferCoors.size();//参与滤波点数
	pRobotDriver->m_cLog->Write("滤波前数值 %d",width);
	if (width<7)
	{
		XUI::MesBox::PopOkCancel("{0}号机器人起始段扫描数据太少，无法完成滤波。数据：{1}", pRobotDriver->m_nRobotNo,width);
		return FALSE;
	}

	//double dis = CalcPolarAngle(vtReferCoors.at(width - 1).x - vtReferCoors.at(0).x,
	//	vtReferCoors.at(width - 1).y - vtReferCoors.at(0).y);
	//
	double dis = TwoPointDis(vtReferCoors.at(width - 1).x, vtReferCoors.at(width - 1).y, vtReferCoors.at(width - 1).z, vtReferCoors.at(0).x,
		 vtReferCoors.at(0).y, vtReferCoors.at(0).z);
	if (dis < 48)
	{
		XUI::MesBox::PopOkCancel("{0}号机器人滤波前数值异常 {1}", pRobotDriver->m_nRobotNo, dis);
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
////////	//m_pLaserVision->CamId2ImageProcess(E_LEFT_ROBOT_LEFT_CAM_H)->SetInit_For_FilterCurve();//初始化滤波用容器
////////    double dTrackVerDegree;
////////	double dTrackVerDegree6th;
////////	//调整量
////////	double dInitComp = pRobotDriver->m_dGunToEyeCompenX;
////////    double dAbjustHeight = pRobotDriver->m_dGunToEyeCompenZ; // 向上补加 向下补减
////////	pRobotDriver->m_cLog->Write("初始段补偿：%.3lf %.3lf", dInitComp, dAbjustHeight);
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
////////		pRobotDriver->m_cLog->Write("初始段滤波:%d %d", index,CoutTrackPoint.size());
////////		for (int nSize = 0;nSize<CoutTrackPoint.size();nSize++)
////////		{
////////			// 初始段补偿
////////			dTrackVerDegree = atan2(CoutTrackPoint[nSize].normalDir_.y_, CoutTrackPoint[nSize].normalDir_.x_) * 180.0 / PI;
////////			cp3DOutPoint.x = CoutTrackPoint[nSize].pnt_.x_ + (dInitComp * CosD(dTrackVerDegree));
////////			cp3DOutPoint.y = CoutTrackPoint[nSize].pnt_.y_ + (dInitComp * SinD(dTrackVerDegree));
////////			cp3DOutPoint.z = CoutTrackPoint[nSize].pnt_.z_ + (dAbjustHeight);
////////			pRobotDriver->m_cLog->Write("index:%d, 滤波后补偿：  %.3lf  %.3lf  %.3lf dTrackVerDegree:%lf %lf %lf  ", index, cp3DOutPoint.x, cp3DOutPoint.y, cp3DOutPoint.z, dTrackVerDegree, CoutTrackPoint[nSize].normalDir_.y_, CoutTrackPoint[nSize].normalDir_.x_);
////////			tPoint.x = cp3DOutPoint.x;
////////			tPoint.y = cp3DOutPoint.y;
////////			tPoint.z = cp3DOutPoint.z;
////////
////////			pRobotDriver->m_cLog->Write("初始化 金滤波：%lf %lf %lf ", cp3DOutPoint.x, cp3DOutPoint.y, cp3DOutPoint.z);
////////			m_vtScanInitWeldLineInWorldPoints.push_back(tPoint);
////////		}
////////	}
////////	int nSize = m_vtScanInitWeldLineInWorldPoints.size();
////////		
////////	pRobotDriver->m_cLog->Write("滤波后数值：%d ",nSize);
////////	if (nSize <=2)
////////	{
////////
////////		XiMessageBox("滤波后数值异常 %d", nSize);
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
////////	pRobotDriver->m_cLog->Write("扫描起点数据 size:%d. %lf %lf %lf ", nSize, m_vtScanInitWeldLineInWorldPoints.at(0).x, m_vtScanInitWeldLineInWorldPoints.at(0).y, m_vtScanInitWeldLineInWorldPoints.at(0).z);
////////#endif
	m_dRealWeldStartPosX = m_vtScanInitWeldLineInWorldPoints[0].x;
	m_dRealWeldStartPosY = m_vtScanInitWeldLineInWorldPoints[0].y;
	m_dRealWeldStartPosZ = m_vtScanInitWeldLineInWorldPoints[0].z;
	pRobotDriver->m_cLog->Write("实际的起点:%lf %lf %lf", m_dRealWeldStartPosX, m_dRealWeldStartPosY, m_dRealWeldStartPosZ);
	return TRUE;
}

void CScanInitModule::FixedStartProc(CRobotDriverAdaptor *pRobotDriver)
{
////////    WriteLog("定长扫描");
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
////////		XiMessageBox("JudgeStartProc没有指定正确的CamId!");
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
////////	T_CART_COOR dMachineCoorsProcess;//记录大车坐标
////////	m_bCameraFindWeldStart = FALSE;
////////	m_bEndPointProcess = FALSE;
////////	CvPoint cpKeyPoint;
////////
////////	//临时添加
////////	m_nStartPointShape = 4;
////////	m_bFixedPointScan = FALSE;
////////	//------------------------
////////	if (FALSE == m_bFixedPointScan)
////////	{
////////		TranslateDrawingToImageParam(); // 根据图纸数据初始化 图像要用到的数据 和 需要特殊处理用到的数据
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
////////		// 这里采取蛙跳法来进行效率优化(先注释掉)
////////		if (m_nEndPointCapCursor < 2) // 从采集到第三张图开始
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
////////		pRobotDriver->m_cLog->Write("搜起点处理时间：%d %d %d", XI_clock() - tTime, cpKeyPoint.x, cpKeyPoint.y);
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
////////		//计算世界坐标系下的点
////////		m_pLaserVision->GetMeasurePosInBaseNew(pRobotDriver->m_strRobotName, cpKeyPoint, eCamId, tPointAbsCoordInBase, m_ptStartPointInfo[m_nEndPointProcCursor % MAX_ARRAY_NUM_START].tRobotCoors,
////////			m_ptStartPointInfo[m_nEndPointProcCursor % MAX_ARRAY_NUM_START].tRobotPulse, pRobotDriver->m_eManipulatorType);
////////		tPointAbsCoordInBase.tWeldLinePos.y += m_ptStartPointInfo[m_nEndPointProcCursor % MAX_ARRAY_NUM_START].dMachineCoors.dY;
////////		//记录世界坐标系下的点
////////		CoutEndPointInfoFileInWorld << m_nEndPointProcCursor << " " << tPointAbsCoordInBase.tWeldLinePos.x << " " << tPointAbsCoordInBase.tWeldLinePos.y << " " << tPointAbsCoordInBase.tWeldLinePos.z << std::endl;
////////		Sleep(30);
////////		DoEvent();
////////	}
////////	//根据反搜找出精确起点
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
////////	pRobotDriver->m_cLog->Write("精搜端点：X:%lf Y:%lf Z:%lf", tWeldLinepointInWorld.x, tWeldLinepointInWorld.y, tWeldLinepointInWorld.z);
////////	FILE *pfStartPoint = fopen("AA_StartUpBoardPoints.txt", "a+");
////////	fprintf(pfStartPoint, "%d%11.3lf%11.3lf%11.3lf\n", -2,
////////		tWeldLinepointInWorld.x, tWeldLinepointInWorld.y, tWeldLinepointInWorld.z);
////////	fclose(pfStartPoint);
////////	// 不变姿态扫描起点时 判断是否使用算法计算立板最远点作为起点
////////	CoutEndPointInfoFileInWorld.close();
////////	//处理起始扫描数据去除真实起点以外的数据机世界标系下
////////	SwapVector(tWeldLinepointInWorld, str + "CoutEndPointInfoFileInWorld.txt", str + "CoutStartPointInfoFileInWorldCopy.txt");
////////	//CoordinatePointProcessFuncation(tWeldLinepointInWorld, str + "CoutEndPointInfoFileInWorld.txt", str + "CoutStartPointInfoFileInWorldCopy.txt");
////////	pRobotDriver->HoldOn();
////////	pRobotDriver->m_cLog->Write("%s 开始检测存图线程", pRobotDriver->m_strRobotName);
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
////////			pRobotDriver->m_cLog->Write("%s 存图线程空闲", pRobotDriver->m_strRobotName);
////////			break;
////////		}
////////		DoEvent();
////////		Sleep(10);
////////	}
////////	m_bEndPointProcess = TRUE;
////////	pRobotDriver->m_cLog->Write("%s端点处理线程退出:%d", pRobotDriver->m_strRobotName, m_bEndPointProcess);
////////	//delete tSaveImage;
	return;
}

void CScanInitModule::CoordinatePointProcessFuncation(XI_POINT tWeldLinepointCoor,std::string FileNameCin,std::string FileNameCout)
{
	std::ifstream fileCurntLugNoCin(FileNameCin.c_str());
	std::ofstream fileCurntLugNoCout(FileNameCout.c_str());
	XI_POINT	tStartScanPoint;//记录起始段扫描点
	XI_POINT    tFirstPoint; // 起始段最后一个点
	std::vector<XI_POINT> tvRecordStartScanPoint;
	tvRecordStartScanPoint.clear();
	int nSeq;
	fileCurntLugNoCin >> nSeq>>tStartScanPoint.x>>tStartScanPoint.y >> tStartScanPoint.z; //先读一个
	tvRecordStartScanPoint.push_back(tStartScanPoint);
	tFirstPoint = tStartScanPoint;
	double dMaxDis = TwoPointDis(tFirstPoint.x,tFirstPoint.y,tWeldLinepointCoor.x,tWeldLinepointCoor.y);
	WriteLog("起始段距离终点最远距离：%.3lf",dMaxDis);
	while (!fileCurntLugNoCin.eof())//筛选真正起点以外的点数据
	{
		fileCurntLugNoCin >> nSeq>>tStartScanPoint.x>>tStartScanPoint.y >> tStartScanPoint.z;
		
		double dDisPoint = TwoPointDis(tStartScanPoint.x,tStartScanPoint.y,tFirstPoint.x,tFirstPoint.y);
		//WriteLog("距离起始点：%.3lf 最大距离：%.3lf",dDisPoint, dMaxDis);
		if (dDisPoint < dMaxDis)
		{
			tvRecordStartScanPoint.push_back(tStartScanPoint);
			//WriteLog("初始段补偿操作");
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
		std::swap(tvRecordStartScanPoint[n],tvRecordStartScanPoint[m]);//将起点扫描数据由起点到结尾点摆放
	}
	
	XI_POINT prePoint = tvRecordStartScanPoint[0];//上一个点
	
	for ( n = 0;n< nSize ;n++)//将翻转以后单位数据放到文本里
	{

		if (0 == n)
		{
			fileCurntLugNoCout<<n<<" "<<tvRecordStartScanPoint[n].x<<" "<<tvRecordStartScanPoint[n].y<<" "<<tvRecordStartScanPoint[n].z;
		}
		else
		{
// 			// 相邻两个点小于1.5过滤掉
			if(1.5 > TwoPointDis(prePoint.x,prePoint.y,tvRecordStartScanPoint[n].x,tvRecordStartScanPoint[n].y))
			{
				continue;
			}
			// 如果起点需要30mm变姿态或需要一次包角，留下128或手眼加包角长度的数据，否则只留手眼关系长度的起始段110
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
	XI_POINT	tStartScanPoint;//记录起始段扫描点
	std::vector<XI_POINT> tvRecordStartScanPoint;
	tvRecordStartScanPoint.clear();
	
	int nSeq;
	XI_POINT tLastPoint;
	fileCurntLugNoCin >> nSeq>>tStartScanPoint.x>>tStartScanPoint.y >> tStartScanPoint.z;
	tvRecordStartScanPoint.push_back(tStartScanPoint);
	tLastPoint = tStartScanPoint;
	double dMaxDis = TwoPointDis(tLastPoint.x,tLastPoint.y,tStartPoint.x,tStartPoint.y);
	
	while (!fileCurntLugNoCin.eof()) // 过滤掉起点以后的点
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
		std::swap(tvRecordStartScanPoint[n], tvRecordStartScanPoint[m]);//将起点扫描数据由起点到结尾点摆放
	}
	
	XI_POINT tTempPoint = tvRecordStartScanPoint[0];
	for ( n = 0;n< nSize ;n++)//将翻转以后单位数据放到文本里
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
			//// 当前函数由定长扫描处理调用，只留一个手眼关系的距离
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
	m_nEndPointCapCursor = 0;  //端点采图序号
	m_nEndPointProcCursor = 0; //处理图号
	m_bEndPointCapCur = FALSE;
	m_bEndPointProcess = FALSE;
	m_bSuccessScanFlag = FALSE;
	//在扫描段点前需要初始化锁定激光斜率
	//初始化锁定  （24/01/02兼容旧搜索）
	//if (!ScanEndpointLockInit(pRobotDriver)) return FALSE;
	long long tTime = XI_clock();

	//采图线程（采图线程拥有次高优先级）
	AfxBeginThread(ThreadDynamicScanningCapture, this, THREAD_PRIORITY_HIGHEST - 1);
	if (m_bFixedPointScan){
		XiMessageBoxOk("暂未移植，尽情期待！");
		return FALSE;
		//AfxBeginThread(ThreadFixedStartProc, this); // 定长扫描起点
	}
	else{
		//调度线程(调度线程拥有最高优先级)
		AfxBeginThread(ThreadDynamicScanningProc, this);
	}

	//互斥访问是否退出（采图和处理线程都退出才能退出）
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
	//并没有搜到端点

	BOOL tresult = TRUE;
	//端点及搜索段数据处理
	if (0 == (int)m_eEndPointType)//起始段
	{
		//tresult = DynamicEndPointResultPro(pRobotDriver);
	}
	else if (1 == (int)m_eEndPointType)//结尾段
	{

	}
	pRobotDriver->HoldOff();
	pRobotDriver->ServoOn();
	m_bCameraFindWeldStart = FALSE;
	RecordRunTime(pRobotDriver, tTime, "搜起点结束");
	if (!tresult || !m_bSuccessScanFlag) XiMessageBox("端点搜索失败");
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
        XiMessageBox("重复进入跟踪线程");
        return FALSE;
    }
    pcMyObj->m_bRealTimeTrackstate = TRUE;
	//pcMyObj->RealTimeTracking(pcMyObj->m_pRobotDriver);
	pcMyObj->RealTimeTrackingNew(pcMyObj->m_pRobotDriver);// 单板单筋
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
        XiMessageBox("重复进入端点采图线程");
        return FALSE;
    }
    pcMyObj->m_bDynamicCapturestate = TRUE;
    pcMyObj->DynamicCapture(pcMyObj->m_pRobotDriver);
    pcMyObj->m_bDynamicCapturestate = FALSE;
    return 0;
}

//2023.2.26 添加跟踪定长搜索功能
UINT CScanInitModule::ThreadDynamicScanningProc(void *pParam)
{
	CScanInitModule *pcMyObj = (CScanInitModule *)pParam;
	if (TRUE == pcMyObj->m_bDynamicProcstate)
	{
		pcMyObj->m_pRobotDriver->HoldOn();
		XiMessageBoxOk("重复进入端点图片线程");
		return FALSE;
	}
	pcMyObj->m_bDynamicProcstate = TRUE;
	//		老方法			
	//pcMyObj->DynamicEndPointProc(pcMyObj->m_pRobotDriver);  
	//      2023.09.26 多线程优化图像处理 2023.10.25 

	int nScansensitivity = 20, nThreshold = 0;
	if (*pcMyObj->m_pUnTracePointCloudProcess > 0)
	{
		nScansensitivity = 20;
		if (*pcMyObj->m_pUnTracePointCloudProcess == 2)
		{
			nThreshold = 3;
		}
	}
	//临时测试允许使用运动模式
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
 * @brief 函数功能：微秒级睡眠
 * @param nUs：微秒 
*/
void CScanInitModule::UsSleep(const unsigned int& nUs)
{
	LARGE_INTEGER fre;
	if (QueryPerformanceFrequency(&fre))
	{
		LARGE_INTEGER run, pre, curr;
		//取微秒级执行计数单元
		run.QuadPart = fre.QuadPart * nUs / 1000000;
		//取执行前时钟计数器
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
	T_CART_COOR dMachineCoors;//记录大车坐标
	long long nTime = XI_clock();
	vector<int> vnParam(3);
	vnParam[0] = IMWRITE_JPEG_QUALITY;
	vnParam[1] = 40;
	vnParam[2] = 1;
	//获取相机驱动指针
	WaitForSingleObject(m_hGetCameraCtrl, INFINITE);
	CDHGigeImageCapture* pDHCamDrv = (CDHGigeImageCapture*)m_ptUnit->GetCameraCtrl(m_ptUnit->m_nMeasureCameraNo);
	ReleaseSemaphore(m_hGetCameraCtrl, 1, NULL);
	//结束循环的唯一条件为 m_bCameraFindWeldStart == TRUE
	while (TRUE)
	{
		//这里不加互斥锁可能出现访问权限异常（2024/01/03 郭嘉建议）
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
		//获取采图位置
		tRobotCurCoord = pRobotDriver->GetCurrentPos();
		tRobotCurPulses = pRobotDriver->GetCurrentPulse();
		//合新框架临时
		dMachineCoors.dX = tRobotCurCoord.dBX;
		dMachineCoors.dY = tRobotCurCoord.dBY;
		dMachineCoors.dZ = tRobotCurCoord.dBZ;

		//R2LevelLock
		m_MutexPicCapture.lock(); {
			m_ptStartPointInfo[m_nEndPointCapCursor % MAX_ARRAY_NUM_START].tRobotCoors = tRobotCurCoord;
			m_ptStartPointInfo[m_nEndPointCapCursor % MAX_ARRAY_NUM_START].tRobotPulse = tRobotCurPulses;
			m_ptStartPointInfo[m_nEndPointCapCursor % MAX_ARRAY_NUM_START].dMachineCoors = dMachineCoors;
			//获取图片
			pDHCamDrv->CaptureImage(m_ptStartPointInfo[m_nEndPointCapCursor % MAX_ARRAY_NUM_START].img, 1);
			//cvCopyImage(pDHCamDrv->CaptureImage(FALSE), m_ptStartPointInfo[m_nEndPointCapCursor % MAX_ARRAY_NUM_START].img);
		}m_MutexPicCapture.unlock();

		Sleep(20);
		//R2LevelLock
		m_MutexPicCapture.lock(); {
			m_nEndPointCapCursor++;
#ifdef __ScanInitModule_ConsoleDbgPrint
			TRACE("扫描采图%d耗时%dms\n", m_nEndPointCapCursor, XI_clock() - nTime);
#endif
			pRobotDriver->m_cLog->Write("扫描采图%d耗时%dms", m_nEndPointCapCursor, XI_clock() - nTime);
		}m_MutexPicCapture.unlock();

	}

	//R0LevelLock
	m_MutexCapThrdExitFlag.lock(); {
		m_bEndPointCapCur = TRUE;
		m_nEndPointCapCursor = 0;
#ifdef __ScanInitModule_ConsoleDbgPrint
		TRACE("%s 动态采图线程退出：%d\n", m_ptUnit->GetUnitName(), m_bEndPointCapCur);
#endif
		pRobotDriver->m_cLog->Write("%s 动态采图线程退出：%d", m_ptUnit->GetUnitName(), m_bEndPointCapCur);
	}m_MutexCapThrdExitFlag.unlock();
}

void CScanInitModule::DynamicEndPointProc(CRobotDriverAdaptor *pRobotDriver, BOOL bSaveImg)//机器人ID，相机ID，端部类型，是否破口
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
		XUI::MesBox::PopInfo("未初始化控制单元{0}的跟踪相机图像处理程序!", m_ptUnit->GetUnitName());

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
//	T_CART_COOR dMachineCoorsProcess;//记录大车坐标

	//R1LevelLock
	while (m_MutexFindEndPointFlag.trylock() == false) UsSleep(2); {
		m_bCameraFindWeldStart = FALSE;
	}m_MutexFindEndPointFlag.unlock();
	//R0LevelLock
	while (m_MutexProcThrdExitFlag.trylock() == false) UsSleep(2); {
		m_bEndPointProcess = FALSE;
	}m_MutexProcThrdExitFlag.unlock();
	
	CvPoint cpKeyPoint;

	//临时添加
	m_nStartPointShape = 4;
	m_bFixedPointScan = FALSE;
	//------------------------
	if (FALSE == m_bFixedPointScan){
		TranslateDrawingToImageParam(); // 根据图纸数据初始化 图像要用到的数据 和 需要特殊处理用到的数据
	}
	else{
		ResumeDrawingToImageParam();
	}
	int nSaveImgNo = 0;

	// 退出循环的唯一方式 m_bCameraFindWeldStart == FALSE
	while (TRUE) {
		//R1LevelLock
		while (m_MutexFindEndPointFlag.trylock() == false) UsSleep(2); {
			if (FALSE == m_bCameraFindWeldStart) {
				if (m_ptUnit->RobotEmg(FALSE)) {
					m_MutexFindEndPointFlag.unlock();
					break;
				}
				//这里不加互斥锁可能出现访问权限异常（2024/01/03 郭嘉建议）
				if (pRobotDriver->m_eThreadStatus == INCISEHEAD_THREAD_STATUS_STOPPED) {
					m_MutexFindEndPointFlag.unlock();
					break;
				}
				// 这里采取蛙跳法来进行效率优化(先注释掉)

				//R2LevelLock 
				while (m_MutexPicCapture.trylock() == false) UsSleep(2); {
					if (m_nEndPointCapCursor < 2) {
						m_MutexPicCapture.unlock();
						m_MutexFindEndPointFlag.unlock();
						continue;// 从采集到第三张图开始 
					}
					m_nEndPointProcCursor = m_nEndPointCapCursor - 1;
					if (NULL == m_ptStartPointInfo[m_nEndPointProcCursor % MAX_ARRAY_NUM_START].img) pRobotDriver->m_cLog->Write("kong!!!!!!!");
					long long tTime = XI_clock();
					m_bCameraFindWeldStart = m_pTraceImgProcess->IfPieceStartOrEndPoint(
						m_ptStartPointInfo[m_nEndPointProcCursor % MAX_ARRAY_NUM_START].img,
						dScanVel, dOffsetStartPoint, cpKeyPoint, eStartOrEnd,
						m_ePieceClass, m_iBaseLineLenInZone, m_iMoveLineLenInZone, m_iRoiWidSize, m_iIfEndConditionIdx
					);
					pRobotDriver->m_cLog->Write("搜起点处理时间：%d %d %d", XI_clock() - tTime, cpKeyPoint.x, cpKeyPoint.y);
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
					//未扫描超出端点 记录该点三维坐标
					if (FALSE == m_bCameraFindWeldStart) {
						//计算世界坐标系下的点
						m_ptUnit->TranImageToBase(
							m_ptUnit->m_nTrackCameraNo,
							cpKeyPoint,
							m_ptStartPointInfo[m_nEndPointProcCursor % MAX_ARRAY_NUM_START].tRobotCoors,
							m_ptStartPointInfo[m_nEndPointProcCursor % MAX_ARRAY_NUM_START].tRobotPulse,
							&tPointAbsCoordInBase
						);
						tPointAbsCoordInBase.tWeldLinePos.y += m_ptStartPointInfo[m_nEndPointProcCursor % MAX_ARRAY_NUM_START].dMachineCoors.dY;
						//记录世界坐标系下的点
						CoutEndPointInfoFileInWorld << m_nEndPointProcCursor << " " << tPointAbsCoordInBase.tWeldLinePos.x << " " << tPointAbsCoordInBase.tWeldLinePos.y << " " << tPointAbsCoordInBase.tWeldLinePos.z << std::endl;
					}
					//扫描超出端点 回找点
					else {
						// 发现端点   
						pRobotDriver->m_cLog->Write("Come In11: %4d %4d", m_nEndPointProcCursor, m_bCameraFindWeldStart);
						pRobotDriver->m_cLog->Write("起点回找参数: m_iRoiWidSize = %d m_iBaseLineLenInZone = %d",
							m_iRoiWidSize, m_iBaseLineLenInZone);
						long long lBackFindTime = XI_clock();
						int nStartBackFineNo = 0;
						CString strImgPath;
						while (TRUE == m_bCameraFindWeldStart){
							//这里不加互斥锁可能出现访问权限异常（2024/01/03 郭嘉建议）
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
								//pRobotDriver->m_cStepMoveControl.EmgStop(); //问题：需转换为CUnit类或RobotDriverAdaptor类操作
								//pRobotDriver->m_cStepMoveControl.StopStep(); //问题：需转换为CUnit类或RobotDriverAdaptor类操作
								pRobotDriver->HoldOn();
								pRobotDriver->ServoOff();
								m_bCameraFindWeldStart = TRUE;
								XUI::MesBox::PopError("反搜图片达到最大张数，暂停机械臂");
								break;
							}
						}
						m_nEndPointProcCursor = (m_nEndPointProcCursor + 1) % MAX_ARRAY_NUM_START;
						pRobotDriver->m_cLog->Write("Come Out11: %4d %4d 回找 次数：%d 耗时：%d",
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

	cstrImgSavePath.Format(OUTPUT_PATH+ m_ptUnit->GetUnitName()+SEARCH_SRC_PATH + "\\真实端点图片.jpg");

	//R1LevelLock
	while (m_MutexPicCapture.trylock() == false) UsSleep(2); {
		SaveImage(m_ptStartPointInfo[m_nEndPointProcCursor % MAX_ARRAY_NUM_START].img, cstrImgSavePath);
		//根据反搜找出精确起点
		m_pTraceImgProcess->IfPieceStartOrEndPoint(
			m_ptStartPointInfo[m_nEndPointProcCursor % MAX_ARRAY_NUM_START].img,
			dScanVel, dOffsetStartPoint, cpKeyPoint, eStartOrEnd,
			m_ePieceClass, m_iBaseLineLenInZone, m_iMoveLineLenInZone, m_iRoiWidSize, m_iIfEndConditionIdx);
		//二维转三维
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
	pRobotDriver->m_cLog->Write("精搜端点：X:%lf Y:%lf Z:%lf", tWeldLinepointInWorld.x, tWeldLinepointInWorld.y, tWeldLinepointInWorld.z);
	FILE* pfStartPoint = fopen(cstrDataSavePath + "AA_StartUpBoardPoints.txt", "a+");
	fprintf(pfStartPoint, "%d%11.3lf%11.3lf%11.3lf\n", -2,
		tWeldLinepointInWorld.x, tWeldLinepointInWorld.y, tWeldLinepointInWorld.z);
	fclose(pfStartPoint);
	// 不变姿态扫描起点时 判断是否使用算法计算立板最远点作为起点
	CoutEndPointInfoFileInWorld.close();
	//处理起始扫描数据去除真实起点以外的数据机世界标系下
	CoordinatePointProcessFuncation(tWeldLinepointInWorld, (cstrDataSavePath + "CoutEndPointInfoFileInWorld.txt").GetBuffer(), (cstrDataSavePath + "CoutStartPointInfoFileInWorldCopy.txt").GetBuffer());
	pRobotDriver->HoldOn();
	pRobotDriver->m_cLog->Write("%s 开始检测存图线程", m_ptUnit->GetUnitName());
	while (TRUE){
		//这里不加互斥锁可能出现访问权限异常（2024/01/03 郭嘉建议）
		if (pRobotDriver->m_eThreadStatus == INCISEHEAD_THREAD_STATUS_STOPPED) break;
		int nThreathNum = 0;
		for (int ns = 0; ns < SAVE_IMAGE_THREAD_MUN; ns++){
			if (FALSE == tSaveImage.bSaveImageStaus[ns]) nThreathNum++;
		}
		if (SAVE_IMAGE_THREAD_MUN == nThreathNum){
			pRobotDriver->m_cLog->Write("%s 存图线程空闲", m_ptUnit->GetUnitName());
			break;
		}
		DoEvent();
		Sleep(10);
	}

	while (m_MutexProcThrdExitFlag.trylock() == false) UsSleep(2); {
		m_bEndPointProcess = TRUE;
		pRobotDriver->m_cLog->Write("%s端点处理线程退出:%d", m_ptUnit->GetUnitName(), m_bEndPointProcess);
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
函数功能：图像处理分发线程	（处理结果为机器人坐标系下坐标）
*	pRobotDriver							机器人控制驱动
*	MaxThreadNum							允许的最大处理线程数		（默认3个）
*	ScanSensitivity							搜索灵敏度（回搜图片张数）  （默认5张）
*	Threshold								允许不连续的处理结果最大值为Threshold	（默认0）
*	PicStrIndex								开始处理的第一张图片序号
*	WaitOutTime								等待图片超时时间	（默认5000ms）
*	SaveImg									是否存图	（默认不存）
*	SavePointCloud							是否存点云	（默认不存）
*	PtType									端点类型（单纯用于保存文件名的修改，无其他用处）
*	FlipMode								翻转模式（默认不翻转）
*	ScanMode								扫描模式（默认搜索二维端点）
*	JobName									搜端点使用的Job名称（默认“DYNAMICSCANNING”）
*	JudgeRobotStopTime						认为机器人为停止状态的时间间隔(单位：ms，默认500ms)
*	NumOfSavePCThreadsInSingleProcThrd		单个处理线程创建的存点云线程数量(默认4个线程)
(2023/11/23更新 新增多机处理参数 修改为一次性创建最大线程数量 新增抢占输出 修复部分崩溃BUG 修改调度锁位置)
(2024/01/11更新	优化内存管理,判停条件，新增调用Job的名称[默认为"DYNAMICSCANNING"])
(2024/02/26更新 E_SCANMODE新增定长搜索E_SCANMODE_FIXEDSCAN)
(2024/03/13更新 修改点云保存方式为多线程保存，保存操作不在本调度线程内，修改至处理线程ThreadAsynDynamic...中)
*/
void CScanInitModule::ImgProcDistribute(CRobotDriverAdaptor* pRobotDriver, unsigned int MaxThreadNum, const int& ScanSensitivity, const unsigned int& Threshold,
	const int& PicStrIndex, const int& WaitOutTime, const bool& SaveImg, const bool& SavePointCloud, E_START_OR_END PtType, const E_FLIP_MODE& FlipMode, const E_SCANMODE& ScanMode,
	const CString JobName, const unsigned int JudgeRobotStopTime, const unsigned int NumOfSavePCThreadsInSingleProcThrd)
{
	/*
	*		2024/01/04 郭嘉谊备注：
	*		本函数使用自旋锁写法，非阻塞式抢占
	*		如遇见 while(_mutex.trylock()==false) UsSleep(50);{...}_mutex.unlock();
	*		表示多线程间互斥锁。“{...}”当做if(true){...}中的“{...}”
	*		明确看出锁的位置 以及是否在{...}后立即释放锁
	*		2024/01/11 修改：所有退出本函数流程都在本函数末尾，而不能在任意处return（方便统一内存管理）
	*/

	/******                           ******/
	m_nStartPointShape = 4;             //老方法里临时使用，在此也临时使用
	TranslateDrawingToImageParam();     //用于赋值成员变量
	/****   上述为 老方法 的固定调用   ****/
	m_nScanSensitivity = ScanSensitivity > 0 ? ScanSensitivity : 1;
	MaxThreadNum = MaxThreadNum <= 0 ? 2 : MaxThreadNum;
	//初始化图像识别参数
	T_IMAGE_RECOGNIZE_PARAM tImgRecParam(m_iBaseLineLenInZone, m_iMoveLineLenInZone, m_iRoiWidSize,
		m_iIfEndConditionIdx, m_ePieceClass, m_bScanStartTwoLineOnStandBoard, m_dScanStartExtendLength);
	//初始化多线程
	AsynchrousProc asynProc = AsynchrousProc(2);
	//初始化成员变量	由本函数释放内存
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
	pair<UINT, clock_t> prWaitProcessAccumTime_LastTime = { 0,0 };							//联合体_记录等待下一串连续处理结果的累计时长，上次连续结果处理完成的时间点
	pair<UINT, clock_t> prWaitPictureAccumTime_LastTime = { 0,0 };							//联合体_记录等待图片的累计时长，上次获取到图片的时间点
	pair<UINT, pair<UINT, clock_t>> prPosEqualNum_prAccumTime_LastTime = { 0,{0,0} };		//联合体_记录机器人处于相同坐标位置的次数、累计时长、上次获取机器人位置的时间点
	long long tStartTime = 0;																//记录搜端点起始时间
	long long tTmpTime = 0;																	//随时可以使用的临时变量计时器
	int nCurThreadIndex = 0;																//线程序列号
	int nCurPicIndex = 0;																	//图片序列号
	int nMapHeadIndex = PicStrIndex;														//起始处理的图序
	int nSuccessScanIndex = 0;																//搜索结果图片号
	BOOL bExitWhileCirc = FALSE;															//退出主while循环标志位
	BOOL bIsCheck = TRUE;																	//是否进行结果检索
	BOOL bHasCalling = FALSE;																//是否Call过扫描Job（可以自定义Job）
	T_ROBOT_COORS preCoor;																	//记录机器人坐标
	CImageProcess* pImgProcess;																//图像处理指针
	vector<pair<int, T_START_POINT_INFO>>* pVtnImgProcInput;								//存储图片处理线程的图片信息队列
	T_THREAD_DYNAMICSCAN_PARAM* ptImgProcInput;												//存储处理线程输入参数
//	XiLineParamNode tFrontLine, tBackLine;													//视觉斜率锁定用的左右线
	CDHGigeImageCapture* pDHCamDrv;															//大恒相机驱动
	CString cstrTrackDipParamFilePath;														//图像处理使用的参数路径
	CString cstrTemp;																		//任意处临时使用的字符串
	CWinThread* pProcessThread;																//处理线程指针
	FILE* pFilePointCloud;																//存點云文件
	int nSavePointCloudCount = 0;															//存点云用到点序号
	SYSTEMTIME tSystemtime;
	GetSystemTime(&tSystemtime);
	if (SavePointCloud)
	{
		//存點云的輸出文件目錄
		cstrTemp.Format(OUTPUT_PATH + m_ptUnit->GetUnitName() + TRACEPROCRESULT_PATH + "%s_TRACE_PointCloud_%04d%02d%02d%02d%02d%02d.txt",
			m_ptUnit->GetUnitName(), tSystemtime.wYear, tSystemtime.wMonth, tSystemtime.wDay, tSystemtime.wHour, tSystemtime.wMinute, tSystemtime.wSecond);
		pFilePointCloud = fopen(cstrTemp, "w");
		m_sPointCloudFileName = cstrTemp;
	}
	tStartTime = XI_clock();
	//确保以下路径存在
	CheckFolder(OUTPUT_PATH + m_ptUnit->GetUnitName() + WELDDATA_PATH);
	CheckFolder(OUTPUT_PATH + m_ptUnit->GetUnitName() + TRACEPROCRESULT_PATH);

	cstrTemp.Format(OUTPUT_PATH + m_ptUnit->GetUnitName() + WELDDATA_PATH + "CoutStartPointInfoFileInWorldCopy.txt");
	ofstream fileCurntLugNoCout(cstrTemp);													 //总输出目录
	cstrTrackDipParamFilePath = m_ptUnit->GetTrackTipDataFileName(m_ptUnit->m_nMeasureCameraNo);
	//获取对应控制单元大恒相机
	WaitForSingleObject(m_hGetCameraCtrl, INFINITE);
	pDHCamDrv = (CDHGigeImageCapture*)m_ptUnit->GetCameraCtrl(m_ptUnit->m_nMeasureCameraNo);
	ReleaseSemaphore(m_hGetCameraCtrl, 1, NULL);
	//先上采图锁,然后多线程斜率锁定		（在下文的while(TRUE)循环内放锁）
	//R2LevelLock
	m_MutexPicCapture.lock(); {
		//获取锁定斜率用的图片
		IplImage* pScanLockImgSrc = NULL; pDHCamDrv->CaptureImage(pScanLockImgSrc, 1);
		IplImage* pScanLockImgDst = cvCreateImage(cvSize(pScanLockImgSrc->width, pScanLockImgSrc->height), pScanLockImgSrc->depth, pScanLockImgSrc->nChannels);
		CString cstrImgSavePath;
		cstrImgSavePath.Format(OUTPUT_PATH + m_ptUnit->GetUnitName() + SEARCH_SRC_PATH + "\\PointType_%d\\", PtType);
		//确保存图目录存在,不存在则自动创建		WinApi::MakeSureDirectoryPathExists()
		CheckFolder(cstrImgSavePath);
		/*
		*		先创建线程，再为每个线程锁定斜率。
		*		所有子线程的new内存由其自身释放，例外：pBisExit由本线程释放
		*/
		for (unsigned int i = 0; i < MaxThreadNum; i++) {
			CString cstrTmp;
			CvPoint cpMidKeyPoint, cpBesideKeyPoint, LeftPtn, RightPtn, tKeyPoint;
			XiLineParamNode tFrontLine, tBackLine;
			vector<CvPoint> vtLeftPtns, vtRightPtns;
			//单线程图像处理库指针 初始化
			pImgProcess = new CImageProcess(
				m_ptUnit->GetCameraParam(m_ptUnit->m_nMeasureCameraNo).tDHCameraDriverPara.nRoiWidth,
				m_ptUnit->GetCameraParam(m_ptUnit->m_nMeasureCameraNo).tDHCameraDriverPara.nRoiHeight,
				0,
				cstrTrackDipParamFilePath
			);
			/*
			*		初始化图像扫描参数 ！！搜端点一定要初始化，让所有图像处理指针完成参数初始化
			*		不允许共用图像处理指针 多线程会访问冲突
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
				//求关键点坐标 以左右线 锁定激光斜率 会更新至ImgProcess的成员变量
				pImgProcess->GetBesideKeyPoints(pScanLockImgDst, cpMidKeyPoint, cpBesideKeyPoint, vtLeftPtns, vtRightPtns, false,
					500, 500,
					300, 300,
					20, 20);
				if (vtLeftPtns.size() <= 0 || vtRightPtns.size() <= 0) {
					//发送急停信号
					pRobotDriver->HoldOn();

					//R1LevelLock
					m_MutexFindEndPointFlag.lock(); {
						m_bCameraFindWeldStart = TRUE;
					}m_MutexFindEndPointFlag.unlock();

					//R0LevelLock
					m_MutexSuccessScanFlag.lock(); {
						m_bSuccessScanFlag = FALSE;
					}m_MutexSuccessScanFlag.unlock();
					//锁定失败存图
					cstrTemp = OUTPUT_PATH + m_ptUnit->GetUnitName() + TRACEPROCRESULT_PATH + "scan_lock_bad.jpg";
					int nParamsArray[3];            //存图格式
					nParamsArray[0] = (int)CV_IMWRITE_JPEG_QUALITY;
					nParamsArray[1] = (int)(0.3 * 100);
					nParamsArray[2] = 0;
					cvSaveImage((LPCSTR)cstrTemp, pScanLockImgDst, nParamsArray);
#ifdef __ScanInitModule_ConsoleDbgPrint
					TRACE("多线程激光斜率锁定失败，线程号:%d\n", i);
#endif
					pRobotDriver->m_cLog->Write("多线程激光斜率锁定失败，线程号:%d", i);
					XUI::MesBox::PopInfo("多线程激光斜率锁定失败，线程号:{0}", i);
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
						//发送急停信号
						pRobotDriver->HoldOn();

						//R1LevelLock
						m_MutexFindEndPointFlag.lock(); {
							m_bCameraFindWeldStart = TRUE;
						}m_MutexFindEndPointFlag.unlock();

						//R0LevelLock
						m_MutexSuccessScanFlag.lock(); {
							m_bSuccessScanFlag = FALSE;
						}m_MutexSuccessScanFlag.unlock();
						//锁定失败存图
						cstrTemp = OUTPUT_PATH + m_ptUnit->GetUnitName() + TRACEPROCRESULT_PATH + "scan_lock_bad.jpg";
						int nParamsArray[3];            //存图格式
						nParamsArray[0] = (int)CV_IMWRITE_JPEG_QUALITY;
						nParamsArray[1] = (int)(0.3 * 100);
						nParamsArray[2] = 0;
						cvSaveImage((LPCSTR)cstrTemp, pScanLockImgDst, nParamsArray);
#ifdef __ScanInitModule_ConsoleDbgPrint
						TRACE("多线程激光斜率锁定失败，线程号:%d\n", i);
#endif
						pRobotDriver->m_cLog->Write("多线程激光斜率锁定失败，线程号:%d", i);
						XUI::MesBox::PopInfo("多线程激光斜率锁定失败，线程号:{0}", i);
						bExitWhileCirc = TRUE;
						delete pImgProcess;
						pImgProcess = NULL;
						break;
					}
				}
			}
			//构造单线程的处理函数输入参数
			Mutex* pMutexVtInput = new Mutex;
			//单线程图像处理队列指针 初始化
			pVtnImgProcInput = new vector<pair<int, T_START_POINT_INFO>>;
			//单线程处理结果输出文件目录
			cstrTmp.Format("%d号线程搜端点分表_%4d%2d%2d%2d%2d%2d.txt", i, tSystemtime.wYear, tSystemtime.wMonth, tSystemtime.wDay, tSystemtime.wHour, tSystemtime.wMinute, tSystemtime.wSecond);
			ofstream* pOutFile = new ofstream(OUTPUT_PATH + m_ptUnit->GetUnitName() + TRACEPROCRESULT_PATH + cstrTmp);
			ptImgProcInput = new T_THREAD_DYNAMICSCAN_PARAM(
				PtType, pRobotDriver, pVtnImgProcInput, m_pMapImgProcOutput, tImgRecParam, pOutFile, pFilePointCloud, &asynProc,
				&m_MutexMapProcOut, &m_MutexExitFlag, pMutexVtInput, &m_MutexSavePointCloud,/*Lock*/
				m_pShowImage, SaveImg, i, new BOOL(FALSE), SavePointCloud,
				pImgProcess, FlipMode, ScanMode, m_ptUnit->GetUnitName(),
				m_ptUnit->GetCameraParam(m_ptUnit->m_nMeasureCameraNo), *pRobotDriver->m_cXiRobotAbsCoorsTrans,
				cstrImgSavePath, &nSavePointCloudCount, NumOfSavePCThreadsInSingleProcThrd);
			//线程运行状态置false
			m_vtImgProcThreadStatus[i] = FALSE;
			//图像处理线程输入参数表HOOK
			m_vtImgProcInput[i] = ptImgProcInput;
			//创建单处理线程 并挂起
			pProcessThread = AfxBeginThread(ThreadAsynDynamicScanEndPointProc, (LPVOID)ptImgProcInput, THREAD_PRIORITY_ABOVE_NORMAL, 0, CREATE_SUSPENDED);
			m_vtImgProcThread[i] = pProcessThread;
		}
		//释放锁定图片堆内存
		cvReleaseImage(&pScanLockImgSrc);
		cvReleaseImage(&pScanLockImgDst);
		//将采图序号置为保存图片起始标号
		m_nEndPointCapCursor = PicStrIndex;
	}

	/*
	*		多线程调度循环 自然跳出条件：bExitWhileCirc == TRUE
	*		离开此循环时，保证采图线程关闭,即需要让m_bCameraFindWeldStart = TRUE
	*		2024/01/11修改：删除所有循环内return，所有分支必须以非return方式退出该循环，方便统一内存管理
	*/
	prWaitProcessAccumTime_LastTime.second = XI_clock();
	prWaitPictureAccumTime_LastTime.second = XI_clock();
	prPosEqualNum_prAccumTime_LastTime.second.second = XI_clock();
	//记录当前姿态
	preCoor = pRobotDriver->GetCurrentPos();
	while (TRUE)
	{
		//恢复采图 互斥锁		（while(TRUE)循环中唯一恢复锁位置）
		m_MutexPicCapture.unlock();
		if (bExitWhileCirc) break;
		Sleep(10);
		prWaitPictureAccumTime_LastTime.first = XI_clock() - prWaitPictureAccumTime_LastTime.second;
		//初始化操作线程号
		nCurThreadIndex = 0;
		//等待采图超时,退出线程
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
			TRACE("多线程搜端点处理失败，等待图片超时！\n");
#endif
			pRobotDriver->m_cLog->Write("多线程搜端点处理失败，等待图片超时！");
			XiMessageBoxOk("多线程搜端点处理失败，等待图片超时！");

			//R2LevelLock
			m_MutexPicCapture.lock();
			bExitWhileCirc = TRUE;
			continue;
		}
		//等待处理连续结果超时,退出线程
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
			TRACE("多线程搜端点处理失败，等待连续处理结果超时！\n");
#endif
			pRobotDriver->m_cLog->Write("多线程搜端点处理失败，等待连续处理结果超时！");
			XiMessageBoxOk("多线程搜端点处理失败，等待连续处理结果超时！");

			//R2LevelLock
			m_MutexPicCapture.lock();
			bExitWhileCirc = TRUE;
			continue;
		}

		//采图互斥锁			（while(TRUE)循环中之后的代码无上锁操作		 唯一上锁位置）
		//R2LevelLock
		m_MutexPicCapture.lock();

		if (nCurPicIndex >= m_nEndPointCapCursor) continue;

		//多线程处理时，修改调度优先级 （2024/03/25屏蔽调度）
		{
			// 		if (m_vtImgProcInput.size() > 1) {
			// 		unsigned int nHeadMinNum = 0xFFFF, nMaxHeadNum = 0x0000;
			// 		unsigned int nHeadMinIndex = m_vtImgProcThread.size() - 1;
			// 		unsigned int nHeadMaxIndex = m_vtImgProcThread.size() - 1;
			// 		for (int i = 0; i < m_vtImgProcInput.size(); i++) {
			// 			//R2LevelLock
			// 			while (m_vtImgProcInput[i]->pMutexVtInput->lock(); {
			// 				if (m_vtImgProcInput[i]->pVtnImgProcInput->size() != 0) {
			// 					//输入队列不为空时判断
			// 					if (m_vtImgProcInput[i]->pVtnImgProcInput->begin()->first < nHeadMinNum) {
			// 						//恢复线程运行(处理队列还有图片的)
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
			// 					//输入队列为空时降低优先级
			// 					m_vtImgProcThread[i]->SetThreadPriority(THREAD_PRIORITY_NORMAL);
			// 				}
			// 			}m_vtImgProcInput[i]->pMutexVtInput->unlock();
			// 		}
			// 		//优先级至少是正常水平
			// 		if (m_vtImgProcThread[nHeadMaxIndex]->GetThreadPriority() > THREAD_PRIORITY_NORMAL) {
			// #ifdef __ScanInitModule_ConsoleDbgPrint
			// 			TRACE("线程%d号置为优先级%d\n", nHeadMaxIndex, m_vtImgProcThread[nHeadMaxIndex]->GetThreadPriority() - 1);
			// #endif
			// 			m_vtImgProcThread[nHeadMaxIndex]->SetThreadPriority(m_vtImgProcThread[nHeadMaxIndex]->GetThreadPriority() - 1);
			// 		}
			// 		else {
			// #ifdef __ScanInitModule_ConsoleDbgPrint
			// 			TRACE("线程%d号置为优先级%d\n", nHeadMaxIndex, THREAD_PRIORITY_NORMAL);
			// #endif
			// 			m_vtImgProcThread[nHeadMaxIndex]->SetThreadPriority(THREAD_PRIORITY_NORMAL);
			// 		}
			// 		//不能超过调度线程||采图线程的优先级
			// 		if (m_vtImgProcThread[nHeadMinIndex]->GetThreadPriority() < THREAD_PRIORITY_ABOVE_NORMAL) {
			// #ifdef __ScanInitModule_ConsoleDbgPrint
			// 			TRACE("线程%d号置为优先级%d\n", nHeadMinIndex, m_vtImgProcThread[nHeadMinIndex]->GetThreadPriority() + 1);
			// #endif
			// 			m_vtImgProcThread[nHeadMinIndex]->SetThreadPriority(m_vtImgProcThread[nHeadMinIndex]->GetThreadPriority() + 1);
			// 		}
			// 		else {
			// #ifdef __ScanInitModule_ConsoleDbgPrint
			// 			TRACE("线程%d号置为优先级%d\n", nHeadMinIndex, THREAD_PRIORITY_NORMAL);
			// #endif
			// 			m_vtImgProcThread[nHeadMinIndex]->SetThreadPriority(THREAD_PRIORITY_NORMAL);
			// 		}
			// 		}
		}

		//等待新图片出现
		if (nCurPicIndex < m_nEndPointCapCursor && m_nEndPointCapCursor > PicStrIndex) {
			//超时计数清零
			prWaitPictureAccumTime_LastTime.first = 0;
			prWaitPictureAccumTime_LastTime.second = XI_clock();

			//2024/03/25 屏蔽调度
			{
				//				//判断是否有处理线程的处理队列为空，空则置其运行标志位为false,并挂起线程
				//				//无图片处理的线程做挂起
				//				for (int i = 0; i < m_vtImgProcThreadStatus.size(); i++) {
				//					//R2LevelLock
				//					while (m_vtImgProcInput[i]->pMutexVtInput->lock(); {
				//						if (m_vtImgProcInput[i]->pVtnImgProcInput->size() == 0) {
				//#ifdef __ScanInitModule_ConsoleDbgPrint
				//							TRACE("处理线程%d因无图片处理，暂时挂起\n", i);
				//#endif
				//							pRobotDriver->m_cLog->Write("处理线程%d因无图片处理，暂时挂起", i);
				//							m_vtImgProcThreadStatus[i] = FALSE;
				//							m_vtImgProcThread[i]->SuspendThread();
				//						}
				//					}m_vtImgProcInput[i]->pMutexVtInput->unlock();
				//				}
				//
				//				//寻找空闲处理线程
				//				for (int i = 0; i < m_vtImgProcThreadStatus.size(); i++) {
				//					nCurThreadIndex = i;
				//					if (m_vtImgProcThreadStatus[i] == FALSE)  break;
				//				}
				//
				//				//无空闲处理线程
				//				if (m_vtImgProcThreadStatus[nCurThreadIndex] == TRUE) {
				//					//2023.12月视觉库 Intel i7-6700单核处理一张2448*2048灰度图片耗时约为40ms
				//					unsigned int nMinProcNum = 65535, nTmpCount = 0, nRecordIndex = 0;
				//					//找到多个处理线程中需要处理图片数量最少的线程
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
				//						TRACE("图像处理分发线程检测到不完整图片信息 nCurId:%d\n", nCurPicIndex);
				//#endif
				//						pRobotDriver->m_cLog->Write("图像处理分发线程检测到不完整图片信息 nCurId:%d\n", nCurPicIndex);
				//
				//					}
				//					tInfo.img = cvCloneImage(tInfo.img);
				//
				//					//R2LevelLock
				//					while (m_vtImgProcInput[nRecordIndex]->pMutexVtInput->lock(); {
				//						m_vtImgProcInput[nRecordIndex]->pVtnImgProcInput->push_back(make_pair(nCurPicIndex, tInfo));
				//					}m_vtImgProcInput[nRecordIndex]->pMutexVtInput->unlock();
				//				}
				//				//有空闲处理线程
				//				else {
				//					//置线程标志位为忙
				//#ifdef __ScanInitModule_ConsoleDbgPrint
				//					TRACE("空闲处理线程%d被开启\n", nCurThreadIndex);
				//#endif
				//					pRobotDriver->m_cLog->Write("空闲处理线程%d被开启", nCurThreadIndex);
				//					m_vtImgProcThreadStatus[nCurThreadIndex] = true;
				//					T_START_POINT_INFO tInfo;
				//					while (tInfo.tRobotPulse.nSPulse == 0 && tInfo.tRobotPulse.nLPulse == 0 && tInfo.tRobotPulse.nUPulse == 0 &&
				//						tInfo.tRobotPulse.nRPulse == 0 && tInfo.tRobotPulse.nBPulse == 0 && tInfo.tRobotPulse.nTPulse == 0 || tInfo.img == NULL) {
				//						tInfo = m_ptStartPointInfo[nCurPicIndex % MAX_ARRAY_NUM_START];
				//					}
				//					if (tInfo.img->nChannels < 0 || tInfo.img->nChannels > 3) {
				//#ifdef __ScanInitModule_ConsoleDbgPrint
				//						TRACE("图像处理分发线程检测到不完整图片信息 nCurId:%d\n", nCurPicIndex);
				//#endif
				//						pRobotDriver->m_cLog->Write("图像处理分发线程检测到不完整图片信息 nCurId:%d\n", nCurPicIndex);
				//					}
				//					tInfo.img = cvCloneImage(tInfo.img);
				//
				//					//传入处理信息
				//					//R2LevelLock
				//					while (m_vtImgProcInput[nCurThreadIndex]->pMutexVtInput->lock(); {
				//						m_vtImgProcInput[nCurThreadIndex]->pVtnImgProcInput->push_back(make_pair(nCurPicIndex, tInfo));
				//					}m_vtImgProcInput[nCurThreadIndex]->pMutexVtInput->unlock();
				//					//恢复空闲线程
				//					while (m_vtImgProcThread[nCurThreadIndex]->ResumeThread() > 0);
				//
				//				}
			}

			//2024/03/25 修改调度流程为按序号取余放入不同线程中
			{
				int nProcThreadNo = nCurPicIndex % m_vtImgProcThread.size();	//处理该图片的线程序号
				T_START_POINT_INFO tInfo;
				while (tInfo.tRobotPulse.nSPulse == 0 && tInfo.tRobotPulse.nLPulse == 0 &&
					tInfo.tRobotPulse.nUPulse == 0 && tInfo.tRobotPulse.nRPulse == 0 &&
					tInfo.tRobotPulse.nBPulse == 0 && tInfo.tRobotPulse.nTPulse == 0 || tInfo.img == NULL)
				{
					tInfo = m_ptStartPointInfo[nCurPicIndex % MAX_ARRAY_NUM_START];
				}
				//异常判断
				if (tInfo.img->nChannels < 0 || tInfo.img->nChannels > 3) {
#ifdef __ScanInitModule_ConsoleDbgPrint
					TRACE("图像处理分发线程检测到不完整图片信息 nCurId:%d\n", nCurPicIndex);
#endif
					pRobotDriver->m_cLog->Write("图像处理分发线程检测到不完整图片信息 nCurId:%d\n", nCurPicIndex);

				}
				tInfo.img = cvCloneImage(tInfo.img);

				//R2LevelLock
				m_vtImgProcInput[nProcThreadNo]->pMutexVtInput->lock(); {
					//给处理子线程传递数据
					m_vtImgProcInput[nProcThreadNo]->pVtnImgProcInput->push_back(make_pair(nCurPicIndex, tInfo));
					//恢复线程执行
					while (m_vtImgProcThread[nProcThreadNo]->ResumeThread() > 0);
				}m_vtImgProcInput[nProcThreadNo]->pMutexVtInput->unlock();
			}


			//判断退出switch时是否需要执行continue返回至while(TRUE)循环的顶部
			BOOL bContinue(FALSE);
			//判断处理模式
			switch (ScanMode) {
			case E_SCANMODE_2DPOINT:	//搜端点，执行是否搜索到点的判断

				//（此处为算法部分，数据结构：线性表，功能：查找）	无需看懂实现，其实现的功能为查找到第N张图片。第N张图片具有如下属性:
				/*
					1.该张图片之前的ScanSensitivity张图片是连续的，保证序号是[n+1,n+2,n+3,n+4...,n+ScanSensitivity]。（ScanSensitivity是输入ImgProcDis函数的参数，称为搜索灵敏度）
						连续：[n,n+1]是连续的结果，[n+1,n+2]是连续的结果，[n+1,n+3]不是连续的结果...
						1.1如果Threshold值大于零，则该张图片前的ScanSensitivity张图片可以不严格连续。只要结果序号的差值在Threshold+1范围内都算连续结果	（Threshold是输入ImgProcDis函数的参数，称为容差）

					2.该张图片后的连续ScanSensitivity张图片都 找不到二维图像的激光交点
						找不到二维图像的激光交点：即处理线程的视觉库 GSImageProcess 中的函数接口 IfPieceStartOrEndPoint() 返回 ScanSensitivity 次 true
						这里连续的定义也满足1.1中的修饰
				*/
				//R2LevelLock
				m_MutexMapProcOut.lock(); {
					//当总映射size大于灵敏度时开始判断是否搜到端点
					if (m_pMapImgProcOutput->size() > m_nScanSensitivity) {
						bIsCheck = FALSE;
						//判断是否有连续处理结果，没有则不判断,继续等待至连续图片处理完成
						int nTmpMapHeadIndex = nMapHeadIndex;
						for (int i = 0; i < m_nScanSensitivity; i++) {
							if (m_pMapImgProcOutput->count(nTmpMapHeadIndex + 1) == 0) {
								if (Threshold > 0) {
									// 尝试容差处理
									for (unsigned int j = 1; j <= Threshold; j++) {
										if (m_pMapImgProcOutput->count(nTmpMapHeadIndex + 1 + j) > 0) {
											// 通过容差处理
											nTmpMapHeadIndex = nTmpMapHeadIndex + 1 + j;
											bIsCheck = FALSE;
											break;
										}
										//未能通过容差处理
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
							// 未通过检查
							if (bIsCheck == TRUE) {
								bIsCheck = FALSE;
								break;
							}
							//通过检查
							if (i + 1 == m_nScanSensitivity) bIsCheck = TRUE;
						}

						//有连续处理结果
						if (bIsCheck) {
							prWaitProcessAccumTime_LastTime.first = 0;
#ifdef __ScanInitModule_ConsoleDbgPrint
							TRACE("检测到连续处理结果");
#endif
							prWaitProcessAccumTime_LastTime.second = XI_clock();
							/*
							*		保存第一个结果迭代器:aPositionItera
							*		在m_nScanSensitivity+Threshold范围内定位的迭代器:aTmpItera
							*/
							map<int, T_THREAD_DYNAMICSCAN_OUT>::iterator aPositionItera, aTmpItera;
							BOOL bWaitFirstResult = TRUE;		//等待第一个处理结果
							//找到第一个处理结果
							for (int i = nMapHeadIndex; i < nMapHeadIndex + m_nScanSensitivity + (int)Threshold; i++) {
								if (m_pMapImgProcOutput->count(i) != 0) {
									aPositionItera = m_pMapImgProcOutput->find(i);
									aTmpItera = m_pMapImgProcOutput->find(i);
									break;
								}
								if (i + 1 == nMapHeadIndex + m_nScanSensitivity + Threshold) bWaitFirstResult = FALSE;
							}
							if (bWaitFirstResult == FALSE) {
								//恢复总图片处理队列 互斥锁
								m_MutexMapProcOut.unlock();
								//跳出switch循环后执行continue语句 返回至while(TRUE)循环的顶部
								bContinue = TRUE;
								break;
							}
							//迭代器aTmpItera定位至第一个 找交点为false 的位置,
							for (int i = 0; i < m_nScanSensitivity + (int)Threshold; i++) {
								if (m_pMapImgProcOutput->count(nMapHeadIndex + i) == 0) continue;
								aTmpItera = m_pMapImgProcOutput->find(nMapHeadIndex + i);
								if ((*aTmpItera).second.bFindIntersectPoint == FALSE) break;
							}
							if (aTmpItera->first == aPositionItera->first && aTmpItera->second.bFindIntersectPoint == FALSE) {
								//判断是否有连续ScanSensitivity个图片找不到交点,有的话表示图片处理完成，否则置迭代器至下一个找得到交点的位置
								for (int i = 0; i < m_nScanSensitivity; i++) {
									if (FALSE == aTmpItera->second.bFindIntersectPoint) {
										aTmpItera++;
									}
									else {
										break;
									}
									if (i == m_nScanSensitivity - 1) aTmpItera--; //回移一个位置
								}
								//至此aTmpItera要么指向找到交点的位置要么指向该范围[now,now+m_nScanSensitivity-1]的最后一个位置
								//移动m_nScanSensitivity - 1 次 查看下标是否和aTmpItera一致
								//aPositionItera[a]-[a+m_nScanSensitivity - 1] 共m_nScanSensitivity个元素
								for (int i = 0; i < m_nScanSensitivity - 1; i++, aPositionItera++);
								if (aPositionItera->first == aTmpItera->first) {
									//结束图片处理,找到端点
									pRobotDriver->HoldOn();
									while (aTmpItera->second.bFindIntersectPoint == FALSE) aTmpItera--;
									//置序号为找到的端点号
									nSuccessScanIndex = aTmpItera->first;

									//R1LevelLock
									m_MutexFindEndPointFlag.lock(); {
										m_bCameraFindWeldStart = TRUE;
									}m_MutexFindEndPointFlag.unlock();

									//R0LevelLock
									m_MutexSuccessScanFlag.lock(); {
										m_bSuccessScanFlag = TRUE;
									}m_MutexSuccessScanFlag.unlock();
									//跳出switch循环后执行continue语句 返回至while(TRUE)循环的顶部
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
			case E_SCANMODE_KINEMATIC:	//运动模式，不执行是否搜索到点的判断
				break;
			case E_SCANMODE_FIXEDSCAN:	//定长模式，不执行是否搜索到点的判断
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
				TRACE("ImgProcDis参数错误！指定了错误的搜索模式！\n");
#endif
				pRobotDriver->m_cLog->Write("ImgProcDis参数错误！指定了错误的搜索模式！");
				XiMessageBoxOk("ImgProcDis参数错误！指定了错误的搜索模式！");
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
			//图序加1
			nCurPicIndex++;
		}

		//判断是否到达搜索终点	同一个姿态保持超时时长(秒)以上则到达终点
		if (preCoor == pRobotDriver->GetCurrentPos()) {
			prPosEqualNum_prAccumTime_LastTime.first++;
			prPosEqualNum_prAccumTime_LastTime.second.first = XI_clock() - prPosEqualNum_prAccumTime_LastTime.second.second;
#ifdef __ScanInitModule_ConsoleDbgPrint
			TRACE("等%d count:%d x:%.3f y:%.3f z:%.3f\n",
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
			TRACE("清零\n");
#endif
		}
		//机器人位于同一位置时长超过了JudgeRobotStopTime，开始判断是否搜索完毕
		if (prPosEqualNum_prAccumTime_LastTime.second.first >= JudgeRobotStopTime && prPosEqualNum_prAccumTime_LastTime.first > JudgeRobotStopTime / 100) {
			pRobotDriver->HoldOn();
			pRobotDriver->m_cLog->Write("静止超时：开始判断是否搜索完毕\n");

			//R1LevelLock
			//先停止采图
			m_MutexFindEndPointFlag.lock(); {
				m_bCameraFindWeldStart = TRUE;
			}m_MutexFindEndPointFlag.unlock();

			//搜端点模式,查询扫描结果
			if (ScanMode == E_SCANMODE_2DPOINT) {
				//R2LevelLock
				m_MutexMapProcOut.lock(); {
					if (m_pMapImgProcOutput->size() > 0) {
						nSuccessScanIndex = (*m_pMapImgProcOutput->rbegin()).first;		//理论上这行代码是无效的，可以注释
					}
					else {
						//R0LevelLock
						m_MutexSuccessScanFlag.lock(); {
							m_bSuccessScanFlag = FALSE;
						}m_MutexSuccessScanFlag.unlock();
#ifdef __ScanInitModule_ConsoleDbgPrint
						TRACE("线程扫描端头失败，处理数据为空\n");
#endif
						pRobotDriver->m_cLog->Write("多线程扫描端头失败，处理数据为空");
						XiMessageBoxOk("多线程扫描端头失败，处理数据为空");
						bExitWhileCirc = TRUE;
					}
				}m_MutexMapProcOut.unlock();
			}
			if (bExitWhileCirc) continue;

			//等待所有线程处理完成，回搜最后一个端点数据
			for (int i = 0; i < m_vtImgProcThreadStatus.size(); i++) {
				tTmpTime = XI_clock();
				m_vtImgProcInput[i]->pMutexVtInput->lock(); {
					while (m_vtImgProcInput[i]->pVtnImgProcInput->size() > 0) {
						m_vtImgProcInput[i]->pMutexVtInput->unlock();
						while (m_vtImgProcThread[i]->ResumeThread() > 0) m_vtImgProcThread[i]->ResumeThread();	//保证线程处于运行状态
						if (XI_clock() - tTmpTime >= WaitOutTime) {
							//R0LevelLock
							m_MutexSuccessScanFlag.lock(); {
								m_bSuccessScanFlag = FALSE;
							}m_MutexSuccessScanFlag.unlock();
#ifdef __ScanInitModule_ConsoleDbgPrint
							TRACE("到达搜索结尾点，线程%d等待图片处理时间超过%d秒,搜索失败\n", i, WaitOutTime / 1000);
#endif
							pRobotDriver->m_cLog->Write("到达搜索结尾点，线程%d等待图片处理时间超过%d秒,搜索失败", i, WaitOutTime / 1000);
							XUI::MesBox::PopInfo("到达搜索结尾点，线程{0}等待图片处理时间超过{1}秒,搜索失败", i, WaitOutTime / 1000);
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
				cstrTemp.Format("搜端点");
				break;
			case E_SCANMODE_KINEMATIC:
				cstrTemp.Format("运动");
				break;
			case E_SCANMODE_FIXEDSCAN:
				cstrTemp.Format("定长搜索");
				break;
			default:
				break;
			}
#ifdef __ScanInitModule_ConsoleDbgPrint
			TRACE("%s模式，到达搜索结尾点\n", cstrTemp);
#endif
			pRobotDriver->m_cLog->Write("%s模式，到达搜索结尾点", cstrTemp);
			//扫描二维点模式||定长搜索模式 给出对应二维点的序列序号,
			if (ScanMode == E_SCANMODE_2DPOINT ||
				ScanMode == E_SCANMODE_FIXEDSCAN
				) {
				//此时所有处理线程已结束
				//R2LevelLock
				m_MutexMapProcOut.lock(); {
					//从后往前搜
					for (auto itera = m_pMapImgProcOutput->rbegin(); itera != m_pMapImgProcOutput->rend(); itera++) {
						if (itera->second.bFindIntersectPoint == TRUE) {
							//图号置为找到的图片
							nSuccessScanIndex = itera->first;
							break;
						}
						if (itera == m_pMapImgProcOutput->rend()) {
							//R0LevelLock
							m_MutexSuccessScanFlag.lock(); {
								m_bSuccessScanFlag = FALSE;
							}m_MutexSuccessScanFlag.unlock();
#ifdef __ScanInitModule_ConsoleDbgPrint
							TRACE("未找到任何工件交点信息，搜索错误\n");
#endif
							pRobotDriver->m_cLog->Write("未找到任何工件交点信息，搜索错误");
							XiMessageBoxOk("未找到任何工件交点信息，搜索错误");
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
	//激活所有挂起的线程
	for (auto itera = m_vtImgProcThread.begin(); itera != m_vtImgProcThread.end(); itera++) {
		if (*itera != NULL) {
			while ((*itera)->ResumeThread() > 0) (*itera)->ResumeThread();
		}
	}
	//销毁所有处理线程
	for (auto itera = m_vtImgProcInput.begin(); itera != m_vtImgProcInput.end(); itera++) {
		//访问退出标志位 互斥锁
	//R2LevelLock
		if ((*itera) == NULL) break;
		m_MutexExitFlag.lock(); {
			*(*itera)->pBisExit = TRUE;
		}m_MutexExitFlag.unlock();
		Sleep(10);

		//R2LevelLock
		m_MutexExitFlag.lock(); {
			while (*(*itera)->pBisExit == TRUE) {
				//循环等待线程释放
				m_MutexExitFlag.unlock();
				Sleep(10);
				m_MutexExitFlag.lock();
			}
		}m_MutexExitFlag.unlock();
	}
	//搜端点模式-(输出搜索点，搜索轨迹  到文件)  输出点云文件
	m_vtImgProc3DPoint.clear();
	// 获取搜端点时外部轴坐标
	double dExPos = m_ptUnit->GetExPositionDis(m_ptUnit->m_nMeasureAxisNo);
	//R0LevelLock
	m_MutexSuccessScanFlag.lock(); {
		if (m_bSuccessScanFlag) {
			//输出端点，轨迹到文件
			if (ScanMode == E_SCANMODE_2DPOINT ||
				ScanMode == E_SCANMODE_FIXEDSCAN) {
				/* 输出搜索点 */
				m_tWeldLinepointInWorld = m_pMapImgProcOutput->find(nSuccessScanIndex)->second.tPointAbsCoordInBase.tWeldLinePos;
				m_tDynamicScanEndPoint = m_tWeldLinepointInWorld;           //原先流程的赋值方式

#ifdef __ScanInitModule_ConsoleDbgPrint
				TRACE("精搜端点：X:%lf Y:%lf Z:%lf 总耗时：\n", m_tWeldLinepointInWorld.x, m_tWeldLinepointInWorld.y, m_tWeldLinepointInWorld.z, XI_clock() - tStartTime);
#endif
				pRobotDriver->m_cLog->Write("精搜端点：X:%lf Y:%lf Z:%lf 总耗时：", m_tWeldLinepointInWorld.x, m_tWeldLinepointInWorld.y, m_tWeldLinepointInWorld.z, XI_clock() - tStartTime);
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
						// 如果起点需要30mm变姿态或需要一次包角，留下128或手眼加包角长度的数据，否则只留手眼关系长度的起始段110
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
				/* 输出搜索点 */

				/* 输出搜索轨迹 */
				CString cstr;
				cstr.Format(OUTPUT_PATH + m_ptUnit->GetUnitName() + TRACEPROCRESULT_PATH + "多线程搜端点总表_%4d%2d%2d%2d%2d%2d.txt",
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
				/* 输出搜索轨迹 */
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
			TRACE("ImgProcDis处理失败\n");
			pRobotDriver->m_cLog->Write("ImgProcDis处理失败");
			XiMessageBoxOk("ImgProcDis处理失败");
#endif
		}
	}m_MutexSuccessScanFlag.unlock();

	pRobotDriver->HoldOff();
	/*****	new内存释放区	 *****/
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
	/*****	new内存释放区	 *****/
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
		//处理线程发来退出信号
		if (*tParam.pExit == true && tParam.pVvtLaserLine->size() <= 0)	break;

		//写入数据
		tParam.pMutexWrite->lock(); {
			//此时禁止外部向Vector写入
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
					TRACE("存点云线程完成一个激光线数据存储 nCount:%d\n", (*tParam.pCount) - 1);
#endif
				}
			}tParam.pMutex_MyVec->unlock();
		}tParam.pMutexWrite->unlock();
		Sleep(10);
	}

#ifdef __ScanInitModule_ConsoleDbgPrint
	TRACE("存点云线程退出\n");
#endif
	delete tParam.pVvtLaserLine;
	tParam.pVvtLaserLine = NULL;
	delete tParam.pMutex_MyVec;
	tParam.pMutex_MyVec = NULL;
	return 0;
}

//处理动态扫端点（新） 更新支持新框架 2024/01/07	该线程不存在new		退出时释放pParam中的new变量（pBisExit由调度线程释放）
UINT CScanInitModule::ThreadAsynDynamicScanEndPointProc(LPVOID pInputParam)
{
	T_THREAD_DYNAMICSCAN_PARAM* pParam = (T_THREAD_DYNAMICSCAN_PARAM*)pInputParam;	//输入参数
	T_SAVE_IMAGE tSaveImage;														//存图结构体		
	CString cstrImgSaveName;														//保存的图片名称
	IplImage* pImageBuffSrc;														//图片指针（原图）
	IplImage* pImageBuffDst;														//图片指针（处理图）
	T_START_POINT_INFO tInfo;														//图片处理组信息
	T_ABS_POS_IN_BASE tPointAbsCoordInBase;											//记录世界坐标
//	T_CART_COOR dMachineCoorsProcess;												//记录大车坐标
	E_START_OR_END eStartOrEnd = E_PIECE_START;										//初始化处理点为 起点
	vector<CWinThread*> vtpThreadSavePointCloud;									//存点云线程的指针
	vector<T_THREAD_SAVEPOINTCLOUD*> vtpThreadSavePCParam;							//存点云线程的参数表
	bool bExitFlagOfSavePCThread(false);											//指示存点云线程退出的标志

	XiCV_Image_Processing tXiCvImgProc = XiCV_Image_Processing(
		pParam->tCamParam.tDHCameraDriverPara.nMaxWidth,
		pParam->tCamParam.tDHCameraDriverPara.nMaxHeight
	);
	/* 初始化坐标变量（图像二维坐标& 相机三维坐标） 起点偏移量 扫描速度 扫描时间 */
	double dCurPosX = 0;
	double dCurPosY = 0;
	double dCamerCoorX = 0;
	double dCamerCoorY = 0;
	double dCamerCoorZ = 0;
	double dOffsetStartPoint = 0;
	double dScanVel = 600;
	int nIndex = 0;
	CvPoint cpKeyPointSrc;		//原图中的激光交点位置
	CvPoint cpKeyPointDst;		//处理图中的激光交点位置
	CvPoint cpOutputPoint;		//输出至map的点
	CString cstrTmp;
	tSaveImage.nImageNo = 0;
	//while(TURE)循环退出的唯一条件：*pParam->pBisExit == true
	while (TRUE)
	{
		/*
		*	线程退出标志位（唯一结束循环的方式）
		*	如果能退出while (TRUE)循环 则pMutexExitFlag在函数返回前置为unlock
		*/
		//R2LevelLock
		pParam->pMutexExitFlag->lock(); {
			if (*pParam->pBisExit == TRUE) {
				break;
			}
		}pParam->pMutexExitFlag->unlock();

		//循环处理图片
		//R2LevelLock
		pParam->pMutexVtInput->lock(); {
			//离开if (pParam->pVtnImgProcInput->size() != 0)语句时，保证pParam->pMutexVtInput状态为lock
			if (pParam->pVtnImgProcInput->size() != 0) {
				pImageBuffSrc = (*pParam->pVtnImgProcInput->begin()).second.img;
				pImageBuffDst = cvCreateImage(cvSize(pImageBuffSrc->width, pImageBuffSrc->height), pImageBuffSrc->depth, pImageBuffSrc->nChannels);
				tInfo = (*pParam->pVtnImgProcInput->begin()).second;
				nIndex = (*pParam->pVtnImgProcInput->begin()).first;
				pParam->pMutexVtInput->unlock();

				//判断图片翻转模式
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
				if (pParam->bSaveImg) {   //需要存图，存原图
					cstrImgSaveName.Format(pParam->cstrImgSavePath + "%d.jpg", nIndex);
					int nParamsArray[3];            //存图格式
					nParamsArray[0] = (int)CV_IMWRITE_JPEG_QUALITY;
					nParamsArray[1] = (int)(0.3 * 100);
					nParamsArray[2] = 0;
					cvSaveImage((LPCSTR)cstrImgSaveName, pImageBuffDst, nParamsArray);
#ifdef __ScanInitModule_ConsoleDbgPrint
					TRACE("保存%d号图片耗时：%d\n", nIndex, XI_clock() - tTime);
#endif
					pParam->pRobotDriver->m_cLog->Write("保存%d号图片耗时：%d", nIndex, XI_clock() - tTime);
				}
				//如果原图异常，打印至控制台，输出日志
				if (pImageBuffSrc->nChannels < 0 || pImageBuffSrc->nChannels>3) {
					SYSTEMTIME tSystime = SYSTEMTIME();
					GetSystemTime(&tSystime);
#ifdef __ScanInitModule_ConsoleDbgPrint
					TRACE("IfPieceStartOrEndPoint调用前遇见ImgBuffer异常,%d:%d:&d\n", tSystime.wHour, tSystime.wMinute, tSystime.wSecond);
#endif
					pParam->pRobotDriver->m_cLog->Write("线程%d IfPieceStartOrEndPoint调用前遇见ImgBuffer异常,%d:%d:&d", pParam->nThreadIndex, tSystime.wHour, tSystime.wMinute, tSystime.wSecond);
				}
				BOOL bFlag(FALSE);
				/*
				*		搜端点模式判断是否找到端点
				*		IfPieceStartOrEndPoint返回true表示找不到端点，在这里取反作为判断标志。
				*		bFlag为true表示仍找到端点
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
				TRACE("线程%d IfPeice耗时:%d\n", pParam->nThreadIndex, XI_clock() - tTime);
				//获取原图中的激光交点位置	cpKeyPointSrc
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
					TRACE("线程%d获取map插入锁耗时:%d\n", pParam->nThreadIndex,XI_clock() - tGetLockFirst);
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
					TRACE("线程%d画出二维点耗时:%d\n", pParam->nThreadIndex, XI_clock() - tGetLockFirst);
#endif
				}pParam->pMutexMapLock->unlock();
#ifdef __ScanInitModule_ConsoleDbgPrint
				TRACE("线程%d处理%d号图片耗时：%d\n\t原图二维点：X %d Y %d\n\t处理图二维点：X %d Y %d\n",
					pParam->nThreadIndex, nIndex, XI_clock() - tTime, cpKeyPointSrc.x, cpKeyPointSrc.y, cpKeyPointDst.x, cpKeyPointDst.y
				);
#endif
				pParam->pRobotDriver->m_cLog->Write("线程%d处理%d号图片耗时：%d\n\t原图二维点：X %d Y %d\n\t处理图二维点：X %d Y %d",
					pParam->nThreadIndex, nIndex, XI_clock() - tTime, cpKeyPointSrc.x, cpKeyPointSrc.y, cpKeyPointDst.x, cpKeyPointDst.y
				);

				cpOutputPoint.x = bFlag ? cpKeyPointSrc.x : 0;
				cpOutputPoint.y = bFlag ? cpKeyPointSrc.y : 0;
				if (
					(pParam->eScanMode == E_SCANMODE_2DPOINT || pParam->eScanMode == E_SCANMODE_FIXEDSCAN) &&
					bFlag
					) {
					//输出激光点三维坐标到文件（用原图的二维点）

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
				vector<CvPoint> vtLaserPoint2DSrc;				//图像激光二维点集
				vector<XI_POINT> vtLaserPoint3D;				//三维点集


				//使用原图生成点云
				if (pParam->bSavePointCloud) {
					int nGetStep = pParam->eScanMode == E_SCANMODE_2DPOINT ? 4 : 4;

					CvPoint cPoint[20000] = { 0,0 };

					int nLength = FindLaserMidPiontEE(pImageBuffSrc, cPoint, 3, 30, 30, 5, 11, 0, 3, 20000, 3, 1, 2, false);

					for (int i = 0; i < nLength; vtLaserPoint2DSrc.push_back(cPoint[i]), i++);
					//生成激光线
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

					//开启存点云线程（没开启足量线程的情况下）
					if (vtpThreadSavePointCloud.size() < pParam->nNumOfSavePointThreads)
					{
						vector<vector<XI_POINT>>* pVvtLaserPoint = new vector<vector<XI_POINT>>;		//存放多个激光线数据		内存由其自身释放
						Mutex* pMutexVtAccess = new Mutex;												//vvtLaserPoint的访问锁		内存由其自身释放
						pVvtLaserPoint->push_back(vtLaserPoint3D);
						//开启存点云线程（内存由本线程在退出前释放）
						T_THREAD_SAVEPOINTCLOUD* pSavePCParam = new
							T_THREAD_SAVEPOINTCLOUD(
								pParam->pOutFilePointCloud,
								pVvtLaserPoint,				/*内存由其自身释放*/
								pParam->pMutexOutFilePC,
								pMutexVtAccess,				/*内存由其自身释放*/
								pParam->pSavePointCloudCount,
								&bExitFlagOfSavePCThread
							);
						CWinThread* pThreadSavePC = AfxBeginThread(ThreadSavePointCloud, (LPVOID)pSavePCParam);
						if (pThreadSavePC != NULL)
						{
							//修改线程的销毁由本线程执行，并非类对象本身
							pThreadSavePC->m_bAutoDelete = false;
							vtpThreadSavePointCloud.push_back(pThreadSavePC);
							vtpThreadSavePCParam.push_back(pSavePCParam);
						}
						else
						{
							//没有创建成功线程，分页池或主存可能出现了异常
							if (pSavePCParam != nullptr)
							{
								delete pSavePCParam;
								pSavePCParam = NULL;
							}
#ifdef __ScanInitModule_ConsoleDbgPrint
							TRACE("开启第%d个存点云线程失败,\n", vtpThreadSavePointCloud.size() + 1);
#endif
							pParam->pRobotDriver->m_cLog->Write("开启第%d个存点云线程失败,\n", vtpThreadSavePointCloud.size() + 1);
						}

					}
					else//给存点云线程传数据(已经开启所有存点云线程的情况下)
					{
						int nSelectNo = 0;	//存点云线程队列中任务量最小的线程的索引
						int nMinSize = vtpThreadSavePCParam.front()->pVvtLaserLine->size();	//最小任务量
						for (auto& itera = vtpThreadSavePCParam.begin(); itera != vtpThreadSavePCParam.end(); itera++)
						{
							//查找任务量最小的
							if (nMinSize > (*itera)->pVvtLaserLine->size())
							{
								nMinSize = (*itera)->pVvtLaserLine->size() > nMinSize;
								nSelectNo = distance(vtpThreadSavePCParam.begin(), itera);
							}
							if (nMinSize == 0) break;	//最小任务量已达到最小值，直接退出循环
						}
						//传输数据
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

				//执行互斥处理
				//R2LevelLock
				pParam->pMutexMapLock->lock(); {
					pParam->pAsynProc->MutexFunction(ImgProcInsertPairInMap, (LPVOID)&tTmpParam, 0, 1);
				}pParam->pMutexMapLock->unlock();
				//释放图像内存 
				if (pImageBuffSrc != NULL && &pImageBuffSrc != NULL) cvReleaseImage(&pImageBuffSrc);
				if (pImageBuffDst != NULL && &pImageBuffDst != NULL) cvReleaseImage(&pImageBuffDst);

				//R2LevelLock
				pParam->pMutexVtInput->lock(); {
					pParam->pVtnImgProcInput->begin() = pParam->pVtnImgProcInput->erase(pParam->pVtnImgProcInput->begin());
				}
			}
		}pParam->pMutexVtInput->unlock();
		//避免在挂起该线程前占用过多CPU资源
		Sleep(5);
	}

	//等待存点云线程存完点云,释放内存
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
	/* 释放pParam的new内存 */
	//1.pVtnImgProcInput中未处理完的图片内存全部释放
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
	//释放线程退出标志锁
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
////////    std::ofstream fileCurnt(strPath + "CoutStartPointInfoFileInWorldSmooth2.txt");//输出滤波后数据*/
////////	std::ofstream fileRobotCoors(strPath + "StartPointInfoFileInRobotCoors.txt");//输出滤波后数据*/
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
////////	//计算法相角度
////////	T_LINE_PARA	tLine = CalcLineParamRansac(vtReferCoors, 0.7);
////////	double dNormal = atan2(tLine.dDirY, tLine.dDirX) * 180 / 3.1415926 - 90 * pRobotDriver->m_nRobotInstallDir;
////////
////////	int width = vtReferCoors.size();//参与滤波点数
////////	pRobotDriver->m_cLog->Write("滤波前数值 %d", width);
////////	double dis = TwoPointDis(vtReferCoors.at(width - 1).x, vtReferCoors.at(width - 1).y, vtReferCoors.at(width - 1).z, vtReferCoors.at(0).x,
////////		vtReferCoors.at(0).y, vtReferCoors.at(0).z);
////////
////////	std::ofstream fileAdjustAfter(strPath + "CoutStartPointInfoFileInWorldCopyAfter.txt");
////////	// 投影
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
////////		XiMessageBox("%d号机器人滤波前数值异常 %lf", pRobotDriver->m_nRobotNo, dis);
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
////////    //调整量
////////    double dInitComp = 0.0/*pRobotDriver->m_dGunToEyeCompenX*/;
////////    double dAbjustHeight = 0.0/*pRobotDriver->m_dGunToEyeCompenZ*/; // 向上补加 向下补减
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
////////        pRobotDriver->m_cLog->Write("初始段滤波:%d %d", index, CoutTrackPoint.size());
////////        for (int nSize = 0; nSize<CoutTrackPoint.size(); nSize++)
////////        {
////////            // 初始段补偿
////////            dTrackVerDegree = atan2(CoutTrackPoint[nSize].normalDir_.y_, CoutTrackPoint[nSize].normalDir_.x_) * 180.0 / PI;
////////            cp3DOutPoint.x = CoutTrackPoint[nSize].pnt_.x_ + (dInitComp * CosD(dTrackVerDegree));
////////            cp3DOutPoint.y = CoutTrackPoint[nSize].pnt_.y_ + (dInitComp * SinD(dTrackVerDegree));
////////            cp3DOutPoint.z = CoutTrackPoint[nSize].pnt_.z_ + (dAbjustHeight);
////////            pRobotDriver->m_cLog->Write("index:%d, 滤波后补偿：  %.3lf  %.3lf  %.3lf dTrackVerDegree:%lf %lf %lf  ", index, cp3DOutPoint.x, cp3DOutPoint.y, cp3DOutPoint.z, dTrackVerDegree, CoutTrackPoint[nSize].normalDir_.y_, CoutTrackPoint[nSize].normalDir_.x_);
////////			tRobotCoor.dX = tPoint.x = cp3DOutPoint.x;
////////			tRobotCoor.dY = tPoint.y = cp3DOutPoint.y;
////////			tRobotCoor.dZ = tPoint.z = cp3DOutPoint.z;
////////			tRobotCoor.dRX = m_dPlatWeldRx;
////////			tRobotCoor.dRY = m_dPlatWeldRy;
////////			tRobotCoor.dRZ = DirAngleToRz(dNormal)/*dTrackVerDegree*/;
////////
////////            pRobotDriver->m_cLog->Write("初始化 金滤波：%lf %lf %lf ", cp3DOutPoint.x, cp3DOutPoint.y, cp3DOutPoint.z);
////////            m_vtScanInitWeldLineInWorldPoints.push_back(tPoint);
////////			vtStartScanData.push_back(tRobotCoor);
////////        }
////////    }
////////    int nSize = m_vtScanInitWeldLineInWorldPoints.size();
////////
////////    pRobotDriver->m_cLog->Write("滤波后数值：%d ", nSize);
////////    if (nSize <= 2)
////////    {
////////
////////        XiMessageBox("滤波后数值异常 %d", nSize);
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
////////    pRobotDriver->m_cLog->Write("扫描起点数据 size:%d. %lf %lf %lf ", nSize, m_vtScanInitWeldLineInWorldPoints.at(0).x, m_vtScanInitWeldLineInWorldPoints.at(0).y, m_vtScanInitWeldLineInWorldPoints.at(0).z);
////////    m_tRealEndPoint = m_vtScanInitWeldLineInWorldPoints[0];
////////    pRobotDriver->m_cLog->Write("实际的起点:%lf %lf %lf", m_tRealEndPoint.x, m_tRealEndPoint.y, m_tRealEndPoint.z);
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
	std::ofstream fileCurnt(strPath + "CoutStartPointInfoFileInWorldSmooth.txt");//输出滤波后数据*/
	TrackFilter_Init(m_ptUnit->m_nRobotSmooth, m_pTraceModel->m_dMoveStepDis, /*dTheoryLength*/0.0,15);
	double dTrackVerDegree;
//	double dTrackVerDegree6th;
	int nSizeCoors = vtCoors.size();
	//调整量
	double dInitComp = m_pTraceModel->m_dGunToEyeCompenX;
	double dAbjustHeight = m_pTraceModel->m_dGunToEyeCompenZ/**pRobotDriver->m_nRobotInstallDir*/; // 向上补加 向下补减
	if (!bFirstFit)
	{
		dInitComp = 0.0;
		dAbjustHeight = 0.0;
	}
	T_ROBOT_COORS tPoint;
	if (m_ptUnit->m_bBreakPointContinue && !m_ptUnit->GetExAxisType(m_ptUnit->m_nTrackAxisNo))
	{
		//断点续焊加载数据已经加过补偿量
		double dAngle = 0.0;

		dAngle = CalcPolarAngle((vtCoors.at(nSizeCoors - 1).dX + vtCoors.at(nSizeCoors - 1).dBX) - (vtCoors.at(0).dX, vtCoors.at(0).dBX),
								(vtCoors.at(nSizeCoors - 1).dY + vtCoors.at(nSizeCoors - 1).dBY) - (vtCoors.at(0).dY + vtCoors.at(0).dBY)) - 90 * pRobotDriver->m_nRobotInstallDir;

		for (int index = 0; index < vtCoors.size(); index++)
		{
			vtCoors.at(index).dX -= dInitComp * CosD(dAngle);
			vtCoors.at(index).dY -= dInitComp * SinD(dAngle);
			vtCoors.at(index).dZ -= dAbjustHeight;
		}		

		//加载缓存位置
		T_ROBOT_COORS tPt = vtCoors.at(vtCoors.size() - 1);
		tPoint = tPt;
		CString strFile;
		strFile.Format("%s\\滤波缓存数据.txt", strPath);
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

		pRobotDriver->m_cLog->Write("初始段滤波:%d %d", index, CoutTrackPoint.size());
		for (int nSize = 0; nSize < CoutTrackPoint.size(); nSize++)
		{
			T_ROBOT_COORS tPoint = vtCoors[index];
			// 初始段补偿
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

			pRobotDriver->m_cLog->Write("index:%d, 滤波后补偿：  %.3lf  %.3lf  %.3lf %.3lf dTrackVerDegree:%lf %lf %lf  ", index,
				tPoint.dX, tPoint.dY, tPoint.dZ, tPoint.dBY, dTrackVerDegree, CoutTrackPoint[nSize].normalDir_.y_, CoutTrackPoint[nSize].normalDir_.x_);
		}
	}
	fileCurnt.close();
	if (vtStartCoors.size() <= 2)
	{

		XUI::MesBox::PopOkCancel("滤波后数值异常 {0}", vtStartCoors.size());
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

		pRobotDriver->m_cLog->Write("初始段滤波:%d %d", index, CoutTrackPoint.size());
		for (int nSize = 0; nSize < CoutTrackPoint.size(); nSize++)
		{
			// 初始段补偿
			double dTrackVerDegree = atan2(CoutTrackPoint[nSize].normalDir_.y_, CoutTrackPoint[nSize].normalDir_.x_) * 180.0 / PI;

			XI_POINT tXiPoint = { CoutTrackPoint[nSize].pnt_.x_,CoutTrackPoint[nSize].pnt_.y_,CoutTrackPoint[nSize].pnt_.z_ };
			T_ROBOT_COORS tRobotPoint(CoutTrackPoint[nSize].pnt_.x_, CoutTrackPoint[nSize].pnt_.y_, CoutTrackPoint[nSize].pnt_.z_,
				m_dPlatWeldRx, m_dPlatWeldRy, m_pRobotDriver->DirAngleToRz(dTrackVerDegree),0.0,0.0,0.0);
			
			vtCoutPoints.push_back(tXiPoint);
			vtCoutRobotCoors.push_back(tRobotPoint);

			pRobotDriver->m_cLog->Write("index:%d, 滤波后补偿：  %.3lf  %.3lf  %.3lf dTrackVerDegree:%.3lf  ", index,
				CoutTrackPoint[nSize].normalDir_.y_, CoutTrackPoint[nSize].normalDir_.x_, CoutTrackPoint[nSize].normalDir_.y_, dTrackVerDegree);
		}
	}
	if (vtCoutRobotCoors.size() <= 2)
	{
		XUI::MesBox::PopOkCancel("滤波后数值异常 {0}", vtCoutRobotCoors.size());
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
	//在扫描段点前需要初始化锁定激光斜率
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
	// 读取坐标
	T_ROBOT_COORS tCoord = pRobotCtrl->GetCurrentPos();
	T_ANGLE_PULSE tPulse = pRobotCtrl->GetCurrentPulse();
	// 临时修改
	double dExAxisPos_y = tCoord.dBY;
#ifdef SINGLE_ROBOT
	double dExAxisPos = tCoord.dBY;;
#else
	double dExAxisPos = tCoord.dBX;
#endif // SINGLE_ROBOT

	// 采集图像
	static int nNumber = 0;
	CString strOrgPicPath;
	strOrgPicPath.Format(".\\SteelStructure\\Picture\\Org\\SinglePointMeasure%d.jpg", nNumber);

	pDHCamDrv->CaptureImage(cvImage, 1);// cvCopyImage(, cvImage);
	SaveImage(cvImage, strOrgPicPath);

	FlipImage(cvImage,*m_peScanFlipMode);
	// 图像处理
	CvPoint cpMidKeyPoint, cpBesideKeyPoint;
	vector<CvPoint> vtLeftPtns, vtRightPtns;
	pImgProcess->GetBesideKeyPoints(cvImage, cpMidKeyPoint, cpBesideKeyPoint, vtLeftPtns, vtRightPtns, false,
		500, 500,
		300, 300,
		20, 20);

	// 计算角点机器人坐标	
	ResumeImagePoint2D(ImageSize.width, ImageSize.height, cpMidKeyPoint, *m_peScanFlipMode);

	T_ROBOT_COORS tRetCoord = m_ptUnit->TranImageToBase(m_ptUnit->m_nMeasureCameraNo, cpMidKeyPoint, tCoord, tPulse);


	// 处理图画点、显示、保存
	ResumeImagePoint2D(cvImage->width, cvImage->height, cpMidKeyPoint, *m_peScanFlipMode);
	cvCvtColor(cvImage, m_pShowImage, CV_GRAY2RGB);
	cvCircle(m_pShowImage, cpMidKeyPoint, 8, CV_RGB(255, 0, 0), 4);
	CString strProPicPath;
	strProPicPath.Format(".\\SteelStructure\\Picture\\Pro\\SinglePointMeasure%d.jpg", nNumber);
	nNumber++;
	SaveImage(m_pShowImage, strProPicPath);
	delete pImgProcess;
	// 计算世界坐标 并 将结果赋值
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
		XiMessageBoxOk("跟踪锁定，初始化大恒相机错误");
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
		//该单元存在照片，先存照片，再释放内存
		cvReleaseImage(&m_pTraceLockImg);
	}
	if (!bFlag) XiMessageBoxOk("搜索激光斜率锁定失败！(错误代码：找不到关键点)");
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
		XiMessageBoxOk("跟踪锁定，初始化大恒相机错误");
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
		XiMessageBoxOk("搜索激光斜率锁定失败！(错误代码：找不到激光线上的点)"); return false;
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
		//该单元存在照片，先存照片，再释放内存
		cvReleaseImage(&m_pTraceLockImg);
	}
	if (!bFlag) XiMessageBoxOk("搜索激光斜率锁定失败！(错误代码：找不到关键点)");
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
	// 初始化相机资源
	CDHGigeImageCapture* pDHCam = NULL;
	pDHCam = (CDHGigeImageCapture*)m_ptUnit->GetCameraCtrl(m_ptUnit->m_nTrackCameraNo);
	// 创建处理函数对象
	cstrTrackDipParamPath = m_ptUnit->GetTrackTipDataFileName(m_ptUnit->m_nTrackCameraNo);
	GROUP_STAND_DIP_NS::CImageProcess* pImageProcess = m_pTraceImgProcess;
	// 创建图片资源
	IplImage *pImageBuff = cvCreateImage(cvSize(
		m_ptUnit->GetCameraParam(m_ptUnit->m_nTrackCameraNo).tDHCameraDriverPara.nMaxWidth,
		m_ptUnit->GetCameraParam(m_ptUnit->m_nTrackCameraNo).tDHCameraDriverPara.nMaxHeight),
		IPL_DEPTH_8U, 1);
	
	int nCount = 0;
	// 滤波后数据
	FILE * pfRecordPointFilterOut = fopen(strPath + "RecordPointFilterOut.txt", "w+");
	// 滤波前原始数据
	FILE * pfRecordPointRawData = fopen(strPath + "RecordPointRawData.txt", "w+");
	// 二维转三维
	CvPoint Corner2D;
	T_ABS_POS_IN_BASE tPointAbsCoordInBase;	
	XiLineParamNode tFrontLine,tBackLine;
	// 滤波输入输出变量
	CvPoint3D64f cp3DOutPoint;
	CvPoint3D64f cp3DInPoint = cvPoint3D64f(0.0, 0.0, 0.0);
	// 滤波后加补偿值
	double dInitComp = m_pTraceModel->m_dGunToEyeCompenX;// 外加内减
	double dAbjustHeight = m_pTraceModel->m_dGunToEyeCompenZ;// 向上补加 向下补减
	pRobotCtrl->m_cLog->Write("dInitComp:%lf dAbjustHeight:%lf", dInitComp, dAbjustHeight);
	// 机械臂初始位置
	T_ROBOT_COORS tWeldLinePointPos;
	T_ROBOT_COORS tRobotStartCoord = pRobotCtrl->GetCurrentPos();
	double dAdvenceGunRzAfter = tRobotStartCoord.dRZ/*- pRobotCtrl->m_dRecordBreakPointRZ*/;
	//机械臂初始的Y值
#ifdef SINGLE_ROBOT
	double dStartRobotY = tRobotStartCoord.dY;
#else
	double dStartRobotY = tRobotStartCoord.dX;
#endif	
	// 初始外部轴坐标
	double dStartMachinePos = m_ptUnit->GetExPositionDis(m_ptUnit->m_nTrackAxisNo);
	// 初始世界坐标
	double dStartBaseAxisCoorY = dStartMachinePos + dStartRobotY;

	pRobotCtrl->m_cLog->Write("dAdvenceGunRzBefor:%lf,dStartMachinePos:%.3lf,dStartRobotY:%.3lf",
		dAdvenceGunRzAfter, dStartMachinePos, dStartRobotY);

	// 用于记录最新一次坐标
	double dLastMachinePos = dStartMachinePos;
	double dLastPosRobotX = tRobotStartCoord.dX;
	double dLastPosRobotY = tRobotStartCoord.dY;
	double dLastPosRobotZ = tRobotStartCoord.dZ;
	//初始段数据
	int nvSize = pRobotCtrl->m_vtWeldLineInWorldPoints.size();
	if (nvSize<2)
	{
		XUI::MesBox::PopOkCancel("跟踪输入初始段数据数量有误：{0} ", nvSize);
		return;
	}
	double dScanDis = TwoPointDis(pRobotCtrl->m_vtWeldLineInWorldPoints.at(0).dX, pRobotCtrl->m_vtWeldLineInWorldPoints.at(0).dY,
		pRobotCtrl->m_vtWeldLineInWorldPoints.at(nvSize - 1).dX, pRobotCtrl->m_vtWeldLineInWorldPoints.at(nvSize - 1).dY);

	// 关键点，Z值调整，Rz调整，滤波失败，记录次数
	int iFindImageKeyPointFailNum = 0;
	int nAbjustZValErrorNum = 0, nAdjustRzErrorNum = 0;
	int nTrackDataEmptyNum = 0;
	// 角度调整阈值
	double dAdjustMaxInterval = 2.0;
	double dAdjustMinInterval = 0.1;

	double dPreRzChange = m_pTraceModel->m_dRecordBreakPointRZ;
	m_pTraceModel->m_dAdjustRobotRZ = m_pTraceModel->m_dRecordBreakPointRZ;
    double dInitRz = dAdvenceGunRzAfter; // 跟踪起点处Rz
	if (m_ptUnit->m_bBreakPointContinue
		&&E_WRAPANGLE_ONCE != m_pTraceModel->m_eStartWrapAngleType)
	{
		dInitRz =pRobotCtrl->m_vtWeldLineInWorldPoints[0].dRZ; // 跟踪起点处Rz
	}
	double dDefaultRz = dInitRz;
	dDefaultRz = fmod(dDefaultRz, 360.0);
	if (0.0 > dDefaultRz)
	{
		dDefaultRz += 360.0; // 0 - 360
	}
	double dDiff = 300;//开启结尾判断
	if (E_CLOSEDARC == m_pTraceModel->m_tWorkPieceType)
	{
		dDiff = m_pTraceModel->m_dWeldLen - 2000;//闭合圆弧线扫长度不可靠
		if (dDiff < 500)
		{
			dDiff = 500;
		}
	}
	if (E_WRAPANGLE_JUMP == m_pTraceModel->m_eStartWrapAngleType)
	{
		dDiff = m_pTraceModel->m_dWeldLen - 100;//跳枪反面焊接较短
	}
	// 一次包角时，提前已知终点？？？
	if (E_WRAPANGLE_ONCE == m_pTraceModel->m_eStartWrapAngleType && !m_pTraceModel->m_bIsTrackingScanEnd)
	{
		dDiff = m_pTraceModel->m_dWeldLen - m_dHandEyeDis / 2;
	}
	pRobotCtrl->m_cLog->Write("起始RZ：%.3lf %.3lf dDiff：%.3lf", dInitRz, dDefaultRz, dDiff);
	std::vector<double> vdTrackOrgRz;
	std::vector<TrackFilter_Node> CoutTrackPoint;
	vdTrackOrgRz.clear();
	int Errorstop = 0;

	// 记录包角前焊接数据用于修正包角
	std::vector<T_ROBOT_COORS> m_vtRecordWarpBeforWeldData;
	m_vtRecordWarpBeforWeldData.clear();

	// 初始化存图状态
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
	// 到结尾点剩余理论长度
	double overlength = 9999.0;
	bool bCompelOver = false;// 强制结束条件
	// 滤波连接处放宽判断条件
	TrackFilter_NextPushPntLoose(m_ptUnit->m_nRobotSmooth);
	while (FALSE == m_pTraceModel->m_bCameraFindWeldEnd)//判断找到结尾后不再采点0
	{
		if (pRobotCtrl->m_eThreadStatus == INCISEHEAD_THREAD_STATUS_STOPPED)
		{
			pRobotCtrl->m_cLog->Write("急停结束跟踪线程： %d", pRobotCtrl->m_eThreadStatus);
			break;
		}
		if (m_pTraceModel->m_bIsCloseTrackThread)//提前知道结尾点判断
		{
			//插入结尾数据
			GetEndSectionCoors(pRobotCtrl, m_pTraceModel->m_tRealEndpointCoor);
			if (E_WRAPANGLE_EMPTY_SINGLE == m_pTraceModel->m_eStartWrapAngleType
				|| E_WRAPANGLE_EMPTY_DOUBLE == m_pTraceModel->m_eStartWrapAngleType)
			{
				int nTotal = 2;
				pRobotCtrl->SetIntVar(nTotal, pRobotCtrl->m_vtWeldLineInWorldPoints.size() - 1);
			}
			else
			{
				// 跟踪结束发送包角数据
				m_pTraceModel->m_bIsTrackingDataSendOver = true;
			}
			//m_pTraceModel->m_bIsTrackingDataSendOver = true;// 临时测试拿出
			m_pTraceModel->m_bCameraFindWeldEnd = TRUE;
			pRobotCtrl->m_cLog->Write("结尾函数结束跟踪线程：%d ", m_pTraceModel->m_bIsCloseTrackThread);
			break;
		}

		if (FALSE == m_pTraceModel->m_bCameraFindWeldEnd)
		{
			nvSize = pRobotCtrl->m_vtWeldLineInWorldPoints.size();
			CoutTrackPoint.clear();
			long long lStartTime = XI_clock();
			long long lCurTime = XI_clock();
			//获取当前位置
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
			//存图
			lCurTime = XI_clock();
			//Corner2D = pImageProcess->GetGroupStandKeyPoint
			//(
			//	pImageBuff, tFrontLine, tBackLine, FALSE, TRUE, TRUE
			//);
			WeightedPointCluster(pImageBuff, &m_pTraceModel->tFrontLine, &m_pTraceModel->tBackLine, &Corner2D, false, "WeightedPointCluster");
			
			pRobotCtrl->m_cLog->Write("%d 单次找点时间:%.3lf %d %d", tSaveImage.nImageNo,(double)(XI_clock() - lCurTime) / CLOCKS_PER_SEC, Corner2D.x, Corner2D.y);
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
			//目标点Y轴世界坐标位置			
#ifdef SINGLE_ROBOT
			double dBaseAxisAbsCoorY = dMachineY + tWeldLinePointPos.dY;
#else
			double dBaseAxisAbsCoorY = dMachineY + tWeldLinePointPos.dX;
#endif	
			pRobotCtrl->m_cLog->Write("%s 机器人目标点：%.3lf %.3lf %.3lf 采图时大车位置：%.3lf 测量结果Y轴世界坐标位置：%.3lf ,nvSize:%d %d",
				strRobot, tWeldLinePointPos.dX, tWeldLinePointPos.dY, tWeldLinePointPos.dZ,
				dMachineY, dBaseAxisAbsCoorY, nvSize, tSaveImage.nImageNo);
			lCurTime = XI_clock();
			if (1 > Corner2D.x || 1 > Corner2D.y || 2448 < Corner2D.x || 2048 < Corner2D.y)
			{
				pRobotCtrl->m_cLog->Write("%s kong Corner2D.x:%d Corner2D.y:%d", strRobot, Corner2D.x, Corner2D.y);
				iFindImageKeyPointFailNum++;
				if (1 < iFindImageKeyPointFailNum)
				{
					// 跟踪调整PosMove之前判断，如果已经发现终点此次不调整(避免与终点扫描立板变姿态冲突)
					if (TRUE == m_pTraceModel->m_bCameraFindWeldEnd)
					{
						continue;
					}
				}
				nCount++;
				
				if (iFindImageKeyPointFailNum > 30)
				{
					pRobotCtrl->m_cLog->Write("%s 连续8次找点数值出错，暂停焊接", strRobot);
					CheckMachineEmg(pRobotCtrl);
					XUI::MesBox::PopOkCancel("{0}号机器人连续8次找点数值出错，暂停焊接", pRobotCtrl->m_nRobotNo);
					break;
				}
				if (iFindImageKeyPointFailNum > 20 && overlength < 30)// 强制结束
				{
					bCompelOver = true;
					pRobotCtrl->m_cLog->Write("%s 强制结束焊接", strRobot);
				}
				else
				{
					continue;
				}
			}
			else
			{
				iFindImageKeyPointFailNum = 0; // 二维点有效时 重置标志

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
					pRobotCtrl->m_cLog->Write("%s 测量数据与上次距离小于2mm,不进行处理:%lf", strRobot, dLatestDistance);
					Errorstop++;
					if (Errorstop >= 30)
					{
						CheckMachineEmg(pRobotCtrl);
						//已修改
						XUI::MesBox::PopInfo("{0}号机器人大车停动!已息弧!可断点续焊", pRobotCtrl->m_nRobotNo);
						//XiMessageBox("%d号机器人大车停动!已息弧!可断点续焊", pRobotCtrl->m_nRobotNo);
					}
					continue;
				}
				else
				{
					//将上次的大车位置给中间值
					dLastMachinePos = dMachineY;
					dStartBaseAxisCoorY = dBaseAxisCoorY;
					dLastPosRobotX = tRobotCurCoord.dX;
					dLastPosRobotY = tRobotCurCoord.dY;
					dLastPosRobotZ = tRobotCurCoord.dZ;
				}
				Errorstop = 0;
				m_pTraceModel->m_dCurrentWeldLen += dLatestDistance;
				pRobotCtrl->m_cLog->Write("%s %d 工件总长度：%11.3lf,当前焊接长度:%11.3lf，dLatestDistance:%lf 外部轴移动：%lf",
					strRobot, nCount, m_pTraceModel->m_dWeldLen, m_pTraceModel->m_dCurrentWeldLen, dLatestDistance, dStartBaseAxisCoorY);
				
				if (TRUE == m_pTraceModel->m_bCameraFindWeldEnd)
				{
					pRobotCtrl->m_cLog->Write("%s 单枪跟踪已经发现终点，不再根据图像和滤波添加数据", strRobot);
					continue;
				}
				lCurTime = XI_clock();
				//滤波用
				pRobotCtrl->m_cLog->Write("%s 滤波前输出数据:, %11.3lf %11.3lf %11.3lf", 
					strRobot, cp3DInPoint.x, cp3DInPoint.y, cp3DInPoint.z);
				bool bNormalDir = false;
				if (-1 == pRobotCtrl->m_nRobotInstallDir)
				{
					bNormalDir = TRUE;
				}
				fprintf(pfRecordPointRawData, "滤波前输出数据: seq: %d, %11.3lf %11.3lf %11.3lf \n", nCount, cp3DInPoint.x, cp3DInPoint.y, cp3DInPoint.z);
				fflush(pfRecordPointRawData);

				CoutTrackPoint = TrackFilter_FilterCurvePointInPointOut(m_ptUnit->m_nRobotSmooth, cp3DInPoint.x, cp3DInPoint.y, cp3DInPoint.z, bNormalDir);

				//! 判断是否完成理论移动长度, return true : 完成移动, 否则未完成.- overlength : 提供一个double指针, 获取剩余长度.
				bool bIsOver = false;
				//bIsOver = TrackFilter_DoneMove(m_ptUnit->m_nRobotSmooth, &overlength);
				if (bIsOver|| bCompelOver)
				{
					// 相机发现结尾点
					m_pTraceModel->m_bCameraFindWeldEnd = TRUE;
					CoutTrackPoint.clear();
					//! 在 TrackFilter_DoneMove() == true时，调用该函数，获取直到理论长度结束 时的轨迹点集.
					//!  或在 overlength 足够小时, 调用该函数, 强制获取到结尾点. 此时获取的结尾点为直线延长.
					TrackFilter_Answer ans = TrackFilter_GetToEndPnts(m_ptUnit->m_nRobotSmooth, bNormalDir);

					size_t index = 0;
					for (index = 0; index < ans.araaySize_; ++index) {
						CoutTrackPoint.push_back(ans.segmentArray_[index]);
					}
					TrackFilter_Free(ans);
					pRobotCtrl->m_cLog->Write("理论数据达到结尾点：%d  ", CoutTrackPoint.size());
				}
				pRobotCtrl->m_cLog->Write("是否到达结尾理论长度：%d 距结尾理论长度：%lf ", bIsOver, overlength);
				double dTrackVerDegree;
				if (nTrackDataEmptyNum > 30)
				{
					CheckMachineEmg(pRobotCtrl);
					pRobotCtrl->m_cLog->Write("%s CoutTrackPoint数据为空次数过多，一般这不太正常，停弧保护，请抬枪后继续焊接", strRobot);
					XiMessageBox("CoutTrackPoint数据为空次数过多，一般这不太正常，停弧保护，请抬枪后继续焊接");
				}
				if (CoutTrackPoint.size() < 1)
				{
					pRobotCtrl->m_cLog->Write("%s CoutTrackPoint数据为空", strRobot);
					nTrackDataEmptyNum++;
					continue;
				}
				nTrackDataEmptyNum = 0;
				for (int nSize = 0; nSize < CoutTrackPoint.size(); nSize++)
				{
					// 初始段补偿
					dTrackVerDegree = atan2(CoutTrackPoint[nSize].normalDir_.y_, CoutTrackPoint[nSize].normalDir_.x_) * 180.0 / PI;
					// 记录滤波后补偿前坐标用于修正包角数据
					m_pTraceModel->m_vtRecordWarpBeforWeldData.push_back(CoutTrackPoint[nSize].pnt_);

					cp3DOutPoint.x = CoutTrackPoint[nSize].pnt_.x_ + (dInitComp * CosD(dTrackVerDegree));
					cp3DOutPoint.y = CoutTrackPoint[nSize].pnt_.y_ + (dInitComp * SinD(dTrackVerDegree));
					cp3DOutPoint.z = CoutTrackPoint[nSize].pnt_.z_ + (dAbjustHeight);
					pRobotCtrl->m_cLog->Write(" %s 滤波后补偿：  %.3lf  %.3lf  %.3lf dTrackVerDegree:%lf %lf %lf  ",
						strRobot, cp3DOutPoint.x, cp3DOutPoint.y, cp3DOutPoint.z, dTrackVerDegree,
						CoutTrackPoint[nSize].normalDir_.y_, CoutTrackPoint[nSize].normalDir_.x_);
					fprintf(pfRecordPointFilterOut, "滤波后数据: seq: %d, %11.3lf %11.3lf %11.3lf\n", nCount,
						cp3DOutPoint.x, cp3DOutPoint.y, cp3DOutPoint.z);
					fflush(pfRecordPointFilterOut);
					// 判断最新的滤波输出数据, 是否可用
					int nS = pRobotCtrl->m_vtWeldLineInWorldPoints.size();
					if (nS < 2)
					{
						XiMessageBox("m_vtWeldLineInWorldPoints向量内存有问题，请检查");
						continue;
					}

					dTrackVerDegree = SmoothRz(dTrackVerDegree, vdTrackOrgRz);//过滤方向
					
					// 根据前进法向量计算机器人Rz
					double dTestRz = DirAngleToRz(dTrackVerDegree);//计算机器人Rz  根据实时变化的前进法相角()和U轴角度计算Rz 																						  // 临近几个点的原始dTestRz取平均值再使					 

					if (dTestRz > 180.0) {
						dTestRz -= 360.0;
					}
					else if (dTestRz < -180.0) {
						dTestRz += 360.0;
					}
					pRobotCtrl->m_cLog->Write("法相角计算RZ:%lf", dTestRz);
					double dTargetPostion[6] = { 0 };
					dTargetPostion[0] = cp3DOutPoint.x;
					dTargetPostion[1] = cp3DOutPoint.y;
					double dHeight = pRobotCtrl->m_vtWeldLineInWorldPoints[nS - 1].dZ - cp3DOutPoint.z;
					if ((dHeight > 4) || (dHeight < -4)) // 机器人高度变化大时不修改
					{
						pRobotCtrl->m_cLog->Write("Z值调整量原始数据：%lf", dHeight);
						dHeight = dHeight > 0 ? 2 : -2;

						dTargetPostion[2] = pRobotCtrl->GetCurrentPos(ROBOT_AXIS_Z) - dHeight;
						nAbjustZValErrorNum++;
						pRobotCtrl->m_cLog->Write("%s 调整高度dHeight:%lf nAbjustZValErrorNum:%d", strRobot, dHeight, nAbjustZValErrorNum);
						if (nAbjustZValErrorNum > 13)
						{
							CheckMachineEmg(pRobotCtrl);
							pRobotCtrl->m_cLog->Write("%s z值调整错误次数过多，停弧保护，请抬枪后继续焊接", strRobot);
							XiMessageBox("z值调整错误次数过多，停弧保护，请抬枪后继续焊接");
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
					if (-180 > dRzChange)//防止出现dTestRz从360到0
					{
						dRzChange += 360.0; // 0 - 360
					}
					else if (180 < dRzChange)
					{
						dRzChange -= 360.0;
					}
					//RZ前后差值
					double dRzBefAndAftDiff = dRzChange - dPreRzChange;
					if (dRzBefAndAftDiff > 180)
					{
						dRzBefAndAftDiff -= 360;
					}
					else if (dRzBefAndAftDiff < -180)
					{
						dRzBefAndAftDiff += 360;
					}

					pRobotCtrl->m_cLog->Write("%s 法相计算dTestRz: %lf RZ差值: %lf dRzChange %lf dPreRzChange:%lf",
						strRobot, dTestRz, dRzBefAndAftDiff, dRzChange, dPreRzChange);
					if ((fabs(dRzBefAndAftDiff) > dAdjustMaxInterval)) // 姿态角度补偿最大变化值限制
					{
						double dDir = dRzBefAndAftDiff > 0.0 ? 1.0 : -1.0;
						dRzChange = dPreRzChange + (dAdjustMaxInterval * dDir);
						nAdjustRzErrorNum++;
						if (nAdjustRzErrorNum > 10)
						{
							CheckMachineEmg(pRobotCtrl);
							pRobotCtrl->m_cLog->Write("%s Rz值调整错误次数过多，停弧保护，请抬枪后继续焊接", strRobot);
							XiMessageBox("Rz值调整错误次数过多，停弧保护，请抬枪后继续焊接");
							break;
						}
					}
					else
					{
						nAdjustRzErrorNum = 0;
					}
					if ((fabs(dRzBefAndAftDiff) < dAdjustMinInterval)) // 姿态角度补偿最小变化值限制
					{
						dRzChange = dPreRzChange;
					}
					dPreRzChange = dRzChange; // 记录上一次Rz补偿 P99
					dTargetPostion[5] = /*dAdvenceGunRzAfter*/dInitRz; // 跟踪轨迹Rz不变，通过实时Rz与跟踪Rz差值最为增量进行调整
					double dCoor[6] = { 0 };
					dCoor[5] = dRzChange;
					m_pTraceModel->m_dAdjustRobotRZ = dRzChange;
					pRobotCtrl->m_cLog->Write("%s 跟踪世界数据：dRzChange:%lf %lf %lf %lf %lf %lf %lf ", strRobot, dRzChange, dTargetPostion[0], dTargetPostion[1], dTargetPostion[2], dTargetPostion[3], dTargetPostion[4], dTargetPostion[5]);
					BOOL bIfWindowOn = TRUE;
					//SetDlgItemData(IDC_STATIC_OPERATE_HINT, this, bIfWindowOn, "%s 调整位置RZ：%lf，Z:%lf", pRobotCtrl->m_strRobotName, dTargetPostion[5], dTargetPostion[2]);
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
                    SaveRobotCoor(m_pRecordTheoryPoint, tRealWeldPoint, E_WELD_TRACK);//保存理论数据
				}

			    // 获取新的结尾数据
				if (bIsOver)
				{					
					// 发送结尾数据点数，用于结束跟踪流程
					int nOverSize = pRobotCtrl->m_vtWeldLineInWorldPoints.size();
					//pRobotCtrl->SetIntVar(2, pRobotCtrl->m_vtWeldLineInWorldPoints.size()-1);
					T_ROBOT_COORS tEndpoint = pRobotCtrl->m_vtWeldLineInWorldPoints.at(pRobotCtrl->m_vtWeldLineInWorldPoints.size() - 1);
					pRobotCtrl->m_cLog->Write("原始结尾数据 %.3lf %.3lf %.3lf %.3lf，更新后结尾数据：%.3lf %.3lf %.3lf %.3lf", 
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
							pRobotCtrl->m_cLog->Write("结尾变化姿态：%lf %lf",
								pRobotCtrl->m_vtWeldLineInWorldPoints.at(nOverSize - (m_pTraceModel->m_nCloseTrackingPos - n) - 1).dRZ, dStepDis);
						}
					}
					m_pTraceModel->m_bIsCloseTrackThread = true;
					m_pTraceModel->m_bIsTrackingDataSendOver = true;

				}

				// 终点判断  m_dDisSafeGunToEnd + 4 时终点初预留长度 4是到终点判断阈值
#ifdef SINGLE_ROBOT
				pRobotCtrl->m_cLog->Write("%s 工件长度：%.3lf  %.3lf %.3lf  轨迹size:%d", strRobot, m_pTraceModel->m_dWeldLen, m_pTraceModel->m_dCurrentWeldLen, dDiff,
					pRobotCtrl->m_vtWeldLineInWorldPoints.size());
#else
				pRobotCtrl->m_cLog->Write("%s 工件长度：%.3lf  %.3lf %.3lf  轨迹size:%d", strRobot, m_pTraceModel->m_dWeldLen, m_pTraceModel->m_dCurrentWeldLen, dDiff,
					pRobotCtrl->m_vtWeldLineInWorldPoints.size());

#endif // SINGLE_ROBOT

				if (m_pTraceModel->m_dCurrentWeldLen > (m_pTraceModel->m_dWeldLen - dDiff) &&
					m_pTraceModel->m_dCurrentWeldLen > 150.0 &&
					FALSE == m_pTraceModel->m_bIfJudgeEndOpen)//开启结尾判断
				{
					m_pTraceModel->m_bIfJudgeEndOpen = TRUE;
					if (m_pTraceModel->m_bIsTrackingScanEnd)//搜索结尾
					{
						pRobotCtrl->m_cLog->Write("%s 开启终点判断", strRobot);						
						pImageProcess->InitIfStartOrEndPntParam(GROUP_STAND_DIP_NS::E_PIECE_END); Sleep(100);
						Corner2D = pImageProcess->GetGroupStandKeyPoint(pImageBuff, tFrontLine, tBackLine, FALSE, TRUE, TRUE);
						pRobotCtrl->m_cLog->Write("%s 初始化终点 :%d %d", strRobot, Corner2D.x, Corner2D.y);
						m_nEndPointCapCursor = 0;
						m_nEndPointProcCursor = 0;
						m_pTraceModel->m_bCameraFindWeldEnd = FALSE;

						AfxBeginThread(ThreadJudgeEndCaptureRobot, this);
						AfxBeginThread(ThreadJudgeEndProcLineGroupStand, this);
					}
					else//已知结尾判断						
					{
						pRobotCtrl->m_cLog->Write("开始结尾判断 已知结尾判断");
						CallJudgeEndpointFun();
					}

					pRobotCtrl->m_cLog->Write("开始结尾判断");
				}
				nCount++;
			}
			else
			{
				pRobotCtrl->m_cLog->Write("%s 00采点数据size:%d,count:%d %11.3lf%11.3lf%11.3lf", strRobot, nvSize, nCount, tWeldLinePointPos.dX, tWeldLinePointPos.dY, tWeldLinePointPos.dZ);
			}
		}
		Sleep(20);
		DoEvent();
	}
	fclose(pfRecordPointFilterOut);
	fclose(pfRecordPointRawData);
	//跟踪流程结束关闭激光
	m_ptUnit->SwitchIO("TrackLaser", false);

	// $$$$ 等待存图线程退出
	// 存储跟踪是否结束标志
	SaveIsTrackCompleteFlag(pRobotCtrl, m_pTraceModel->m_bCameraFindWeldEnd, overlength);
	pRobotCtrl->m_cLog->Write("%s 开始检测存图线程跟踪", pRobotCtrl->m_strRobotName);
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
			pRobotCtrl->m_cLog->Write("%s 存图线程空闲跟踪", pRobotCtrl->m_strRobotName);
			break;
		}
		DoEvent();
		Sleep(10);
	}
	pRobotCtrl->m_cLog->Write("%s 跟踪线程结束", strRobot);
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

	m_pTraceModel->lnImageNo.clear();	// 当前点云集合中每张图编号
	m_pTraceModel->lnImagePtnNum.clear();	// 当前点云集合中每张图点数
	m_pTraceModel->ltPointCloud.clear(); // 当前点云集合
	m_pTraceModel->ltCapWorldCoord.clear(); // 当前点云集合中每张图采集时世界坐标
	m_pTraceModel->vtPointCloudEndPoints.clear(); // 每次点云处理提取的端点世界坐标
	m_pTraceModel->vtConnerWorldCoords.clear();
	m_pTraceModel->m_bCameraFindWeldEnd = false;
	m_pTraceModel->pfPointCloud = fopen(OUTPUT_PATH + m_pRobotDriver->m_strRobotName + WELDDATA_PATH + "RecordPointCloud.txt", "w");
	m_pTraceModel->pfImgCalcData = fopen(OUTPUT_PATH + m_pRobotDriver->m_strRobotName + WELDDATA_PATH + "RecordCapImgCoord.txt", "w");

	m_pCapImageData = new T_CAP_IMAGE_DATA(nMaxImgBufSize); // 创建数据缓冲区

	m_IsOpenImageProcess = true;
	for (int i = 0; i < nProcThreadNum; i++) // 开启图片处理线程
	{
		CWinThread* tProcImgThread = AfxBeginThread(ThreadImageProcess, this);
		tProcImgThread->m_bAutoDelete = false;
		vpWinThread.push_back(tProcImgThread);

	}
	CWinThread* tProcPtnCldThread = AfxBeginThread(ThreadJudgeEnd_ProcessSearch, this); // 开启点云处理线程
	tProcPtnCldThread->m_bAutoDelete = false;
	vpWinThread.push_back(tProcPtnCldThread);

	int nCurWaitTime = 0;
	int nWaitTimeStep = 100;
	bool bIsRunning = m_ptUnit->WorldIsRunning();
	while (!m_ptUnit->WorldIsRunning()) // 等待运动
	{
		Sleep(nWaitTimeStep);
		nCurWaitTime += nWaitTimeStep;
		if (nCurWaitTime > nTimeOut)
		{
			DELETE_POINTER(m_pCapImageData);
			pRobotDriver->m_cLog->Write("[点云搜端点]: 等待运动超时！");
			return;
		}
	}

	// 正在运动中 且 未发现终点
	while (m_ptUnit->WorldIsRunning() && true == m_IsOpenJudgeEnd_ProcessSearch)
	{
		lStartTime = GetTickCount();

		// 读坐标
		tRobotCoord = pRobotDriver->GetCurrentPos();
		tRobotPulse = pRobotDriver->GetCurrentPulse();

		// 采图
		pDHCamDrv->CaptureImage(pImageBuf, 1);

		// 保存到缓冲区
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

		pRobotDriver->m_cLog->Write("[点云搜端点]: 扫描采图总编号%d 缓存号%d 耗时%dms 采图成功%d 图片状态%d",
			nCapImageNo, nSaveBufNo, GetTickCount() - lStartTime, bCapSuc, nImgStatus);
		nCapImageNo++;

	}
	m_IsOpenImageProcess = false;

	long lTime = GetTickCount();
	WaitAndCheckAllThreadExit(vpWinThread);
	pRobotDriver->m_cLog->Write("[点云搜端点]: WaitAndCheckAllThreadExit 耗时%dms", GetTickCount() - lTime);

	// 释放 m_pCapImageData
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
	pRobotDriver->m_cLog->Write("[点云搜端点]: FuncRealTimeTrackCapture_H 结束");
}

void CScanInitModule::RealTimeTrackingNew_H(CRobotDriverAdaptor* pRobotCtrl)
{
	pRobotCtrl->m_cLog->Write("[跟踪调度]: 跟踪线程开始");
	// 变量初始化
	bool bSaveImage = false;// 是否保存跟踪图片
	int nJudgeEndMethod = 3;	// 1.已知终点 2.用理论长度 3.焊接过程中判断
	int nImageNo = -1;	// 图片序号
	int nMaxImgProFailNum = 20;	// 最大连续图像处理失败次数
	int nMaxAdjustZErrNum = 5;	// 最大连续Z值变化过大次数
	int nMaxFilterFailNum = 40;	// 最大连续滤波失败次数
	int nMaxErrStopNum = 50;	// 最大连续检测停止次数
	int nImgProFailNum = 0;	// 连续图像处理失败次数
	int nAdjustZErrNum = 0;	// 连续Z值变化过大次数
	int nFilterFailNum = 0;	// 连续滤波失败次数
	int nErrStopNum = 0;	// 连续检测停止次数
	double dMinJoinPtnDis = 0.5;	// 采图测量点最小相邻距离
	int nConnerRstNum = 0; // 记录每次获取焊缝坐标时 处理的角点数量--
	int nPointCloudRstNum = 0; // 记录每次获取焊缝坐标是 点云处理结果数量--
	T_ROBOT_COORS tWeldLineWorldCoord;	// 实时测量焊缝机器人世界坐标 X Y Z

	m_IsOpenJudgeEnd_ProcessSearch = false;
	m_pTraceModel->lnImageNo.clear();
	m_pTraceModel->lnImagePtnNum.clear();
	m_pTraceModel->ltPointCloud.clear();
	m_pTraceModel->ltCapWorldCoord.clear();
	m_pTraceModel->vtPointCloudEndPoints.clear();
	m_pTraceModel->vtConnerWorldCoords.clear();
	m_pTraceModel->m_dCurrentWeldLen = 0.0 > m_pTraceModel->m_dCurrentWeldLen ? 0.0 : m_pTraceModel->m_dCurrentWeldLen;	// 当前累计焊接长度初始化
	m_pTraceModel->m_dPreRzChange = m_pTraceModel->m_dRecordBreakPointRZ;
	m_pTraceModel->m_dAdjustRobotRZ = m_pTraceModel->m_dRecordBreakPointRZ;
	int nAllTrackNum = pRobotCtrl->m_vtWeldLineInWorldPoints.size();
	if (nAllTrackNum < 2)
	{
		XUI::MesBox::PopOkCancel("[跟踪调度]:跟踪输入初始段数据数量有误：{0} ", nAllTrackNum);
		return;
	}

	// 路径 和 字符串 初始化
	CString strRobot = pRobotCtrl->m_strRobotName;
	CString strPath = OUTPUT_PATH + pRobotCtrl->m_strRobotName + WELDDATA_PATH;
	CString strSavePath = OUTPUT_PATH + pRobotCtrl->m_strRobotName + "\\Track\\";
	CString strImgPathName = "";

	// 数据文件初始化
	FILE* pfRawData = fopen(strPath + "RecordPointRawData.txt", "a+");// 滤波前原始数据
	FILE* pfFilterOut = fopen(strPath + "RecordPointFilterOut.txt", "a+");// 滤波后数据
	FILE* pfPointCloud = fopen(strPath + "RecordPointCloud.txt", "a+");// 焊缝点云保存

																	   // 资源对象
	CDHGigeImageCapture* pDHCam = (CDHGigeImageCapture*)m_ptUnit->GetCameraCtrl(m_ptUnit->m_nTrackCameraNo); // 相机指针
	IplImage* pImageBuff = pDHCam->m_pImageBuff; // 图片缓冲区 //IplImage* pImageBuff = cvCreateImage(cvSize(tCameraParam.tDHCameraDriverPara.nMaxWidth, tCameraParam.tDHCameraDriverPara.nMaxHeight), IPL_DEPTH_8U, 1); // 图片缓冲区
	std::vector<TrackFilter_Node> vtCoutTrackPoint(0);
	IplImage *pTempImgBuf = cvCreateImage(cvSize(pImageBuff->width, pImageBuff->height), IPL_DEPTH_8U, 1);

	pRobotCtrl->m_cLog->Write("[跟踪调度]:跟踪轨迹补偿：dInitComp:%lf dAbjustHeight:%lf", m_pTraceModel->m_dGunToEyeCompenX, m_pTraceModel->m_dGunToEyeCompenZ);

	AfxBeginThread(ThreadRealTimeTrackCapture_H, this); // 开启跟踪采集处理线程
	// 开始跟踪过程
	while (FALSE == m_pTraceModel->m_bCameraFindWeldEnd)//判断找到结尾后不再采点0
	{
		if (pRobotCtrl->m_eThreadStatus == INCISEHEAD_THREAD_STATUS_STOPPED)
		{
			pRobotCtrl->m_cLog->Write("[跟踪调度]:急停结束跟踪线程： %d", pRobotCtrl->m_eThreadStatus);
			break;
		}

		// 终点判断：1.已知终点(干涉终点) 2.用理论长度(留焊) 3.焊接过程中判断(自由终点) 三种方式
		// 1、终点判断：更新终点坐标 并 设置m_pTraceModel->m_bCameraFindWeldEnd为true
		//JudgeFindWeldLineEnd(pRobotCtrl, nJudgeEndMethod); // ThreadRealTimeTrackCapture_H中包含终点判断
		//if (TRUE == m_pTraceModel->m_bCameraFindWeldEnd) break;

		// 从ThreadRealTimeTrackCapture_H的处理结果中获取最新焊缝坐标
		CHECK_BOOL_CONTINUE(GetTrackWeldLineWorldCoord(tWeldLineWorldCoord, nConnerRstNum, nPointCloudRstNum, nImgProFailNum, nMaxImgProFailNum, pfRawData));

		// 6、检测滤波是否正常 连续多次滤波失败停止跟踪 正常复位累计计数 输入最新焊缝坐标tWeldLineWorldCoord 输出滤波结果vtCoutTrackPoint 滤波输入数据存入文件pfRawData
		CHECK_BOOL_CONTINUE(CheckTrackFilterValid(pRobotCtrl, nImageNo, tWeldLineWorldCoord, vtCoutTrackPoint, nFilterFailNum, nMaxFilterFailNum));

		// 7、滤波输出数据处理 连续多次调整Z值失败 停止跟踪 正常复位累计计数 将vtCoutTrackPoint中的每个点添加补偿 写入文件 并 追加到跟踪焊接轨迹缓冲区
		CHECK_BOOL_BREAK(FilterResultTrans(pRobotCtrl, nImageNo, vtCoutTrackPoint, nAdjustZErrNum, nMaxAdjustZErrNum, pfFilterOut));

		Sleep(100);
		DoEvent();
	}
	// 发现终点后 获取并更新终点段数据
	if (TRUE == m_pTraceModel->m_bCameraFindWeldEnd)
	{
		// 获取跟踪结尾及终点坐标轨迹
		GetEndSectionCoorsNew(pRobotCtrl, m_pTraceModel->m_tRealEndpointCoor);
		// 根据包角参数 添加包角轨迹 

		// 跟踪结束后的相关变量
		m_pTraceModel->m_bIsTrackingDataSendOver = true; // !!!! 临时为发送包角轨迹提供判断条件

	}
	fclose(pfFilterOut);
	fclose(pfRawData);
	fclose(pfPointCloud);
	m_ptUnit->SwitchIO("TrackLaser", false); //跟踪流程结束关闭激光

	pRobotCtrl->m_cLog->Write("[跟踪调度]: 跟踪线程结束");
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

	if ((nPointCloudRstNum <= 0) && (nConnerRstNum > nOldConnerRstNum)) // 无点云结果 且 角点结果更新
	{
		tWeldLineWorldCoord = tConnerWorldCoord;
		fprintf(pfFile, "%11.3lf%11.3lf%11.3lf %10d%10d\n", tWeldLineWorldCoord.dX, tWeldLineWorldCoord.dY, tWeldLineWorldCoord.dZ, nConnerRstNum, nPointCloudRstNum); 
		nFailNum = 0;
	}
	else if (nPointCloudRstNum > nOldPointCloudRstNum) // 有点云结果更新
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
			m_pRobotDriver->m_cLog->Write("[获取焊缝坐标]:%s 连续%d次获取焊缝坐标失败！", m_pRobotDriver->m_strRobotName, nMaxFailNum);
			XUI::MesBox::PopInfo("[获取焊缝坐标]:{0} 连续{1}次获取焊缝坐标失败！", m_pRobotDriver->m_strRobotName, nMaxFailNum);
		}
		bSuccess = false;
		Sleep(100);
	}
	return bSuccess;
}


void CScanInitModule::InitRealTimeTracking(E_JUDGE_END_METHOD eJudgeEndMethod, T_ROBOT_COORS* pEndCoord/* = NULL*/)
{
	m_eJudgeEndMethod = eJudgeEndMethod;
	if ((NULL != pEndCoord) && (E_JUDGE_END_KNOW == m_eJudgeEndMethod)) // 已知终点进行终点判断
	{
		m_pTraceModel->m_tRealEndpointCoor = *pEndCoord;
		m_pTraceModel->m_tRealEndpointCoor.dX += m_pTraceModel->m_tRealEndpointCoor.dBX; m_pTraceModel->m_tRealEndpointCoor.dBX = 0.0;
		m_pTraceModel->m_tRealEndpointCoor.dY += m_pTraceModel->m_tRealEndpointCoor.dBY; m_pTraceModel->m_tRealEndpointCoor.dBY = 0.0;
		m_pTraceModel->m_tRealEndpointCoor.dZ += m_pTraceModel->m_tRealEndpointCoor.dBZ; m_pTraceModel->m_tRealEndpointCoor.dBZ = 0.0;
	}
}

/*
	使用说明：：
	1、要求 pRobotCtrl->GetCurrentPos(); 读取的是跟踪过程中 “所有参与跟踪运动” 的机器人坐标系下坐标，如用法不同，需要添加其他函数代替
	2、保存数据：
		1、初始段处理：
			初始段原始采集数据：点号 直角坐标 世界直角坐标
			初始段处理后运动数据: 点号 直角坐标 世界直角坐标
		2、跟踪运动：
			所有发送运动轨迹
			实际运动实时读取轨迹
		3、跟踪过程：√ 
			1.跟踪采集原始数据: 图片序号、二维点、焊缝坐标、采图直角坐标、采图关节坐标
			2.滤波后:图片序号 滤波原始输出 滤波输出添加补偿 滤波法向向量 滤波法向角 水平补偿值 高度补偿值
		4、终点判断：
			已知终点：终点段轨迹 投影前终点坐标 投影后终点坐标
		5、包角轨迹：
			计算输入 计算输出
*/
void CScanInitModule::RealTimeTrackingNew(CRobotDriverAdaptor* pRobotCtrl)
{
	pRobotCtrl->m_cLog->Write("[跟踪调度]: 跟踪线程开始");
	// 变量初始化
	bool bSaveImage			= false;// 是否保存跟踪图片
	int nJudgeEndMethod		= 3;	// 1.已知终点 2.用理论长度 3.焊接过程中判断
	int nImageNo			= -1;	// 图片序号
	int nMaxImgProFailNum	= 20;	// 最大连续图像处理失败次数
	int nMaxAdjustZErrNum	= 5;	// 最大连续Z值变化过大次数
	int nMaxFilterFailNum	= 30;	// 最大连续滤波失败次数
	int nMaxErrStopNum		= 50;	// 最大连续检测停止次数
	int nImgProFailNum		= 0;	// 连续图像处理失败次数
	int nAdjustZErrNum		= 0;	// 连续Z值变化过大次数
	int nFilterFailNum		= 0;	// 连续滤波失败次数
	int nErrStopNum			= 0;	// 连续检测停止次数
	double dMinJoinPtnDis	= 0.5;	// 采图测量点最小相邻距离
	T_ROBOT_COORS tWeldLineRobotCoord;	// 实时测量焊缝机器人坐标 X Y Z RX RY RZ
	T_ROBOT_COORS tWeldLineWorldCoord;	// 实时测量焊缝机器人世界坐标 X Y Z
	T_ROBOT_COORS tCapRobotCurCoord;	// 采图时机器人直角坐标 X Y Z RX RY RZ BX BY BZ
	T_ANGLE_PULSE tCapRobotCurPulse;	// 采图时机器人关节坐标 S L U R B T BX BY BZ
	T_ROBOT_COORS tCapWorldCurCoord;	// 采图时机器人世界坐标 X Y Z RX RY RZ
	ReadTrackCapCoord(pRobotCtrl, tCapRobotCurCoord, tCapRobotCurPulse, tCapWorldCurCoord);
	T_ROBOT_COORS tRobotStartCoord(tCapRobotCurCoord);	// 开始跟踪前的机械臂初始位置
	T_ROBOT_COORS tLastCapWorldCoord(tCapWorldCurCoord);// 实时最后一次采图时坐标世界坐标

	m_IsOpenJudgeEnd_ProcessSearch = false;
	m_pTraceModel->lnImageNo.clear();
	m_pTraceModel->lnImagePtnNum.clear();
	m_pTraceModel->ltPointCloud.clear();
	m_pTraceModel->ltCapWorldCoord.clear();
	m_pTraceModel->vtPointCloudEndPoints.clear();
	m_pTraceModel->m_dCurrentWeldLen = 0.0 > m_pTraceModel->m_dCurrentWeldLen ? 0.0 : m_pTraceModel->m_dCurrentWeldLen;	// 当前累计焊接长度初始化
	m_pTraceModel->m_dPreRzChange = m_pTraceModel->m_dRecordBreakPointRZ;
	m_pTraceModel->m_dAdjustRobotRZ = m_pTraceModel->m_dRecordBreakPointRZ;
	int nAllTrackNum = pRobotCtrl->m_vtWeldLineInWorldPoints.size();
	if (nAllTrackNum < 2)
	{
		XUI::MesBox::PopOkCancel("[跟踪调度]:跟踪输入初始段数据数量有误：{0} ", nAllTrackNum);
		return;
	}

	// 路径 和 字符串 初始化
	CString strRobot = pRobotCtrl->m_strRobotName;
	CString strPath = OUTPUT_PATH + pRobotCtrl->m_strRobotName + WELDDATA_PATH;
	CString strSavePath = OUTPUT_PATH + pRobotCtrl->m_strRobotName + "\\Track\\";
	CString strImgPathName = "";

	// 数据文件初始化
	FILE* pfRawData = fopen(strPath + "RecordPointRawData.txt", "w");// 滤波前原始数据
	FILE* pfFilterOut = fopen(strPath + "RecordPointFilterOut.txt", "w");// 滤波后数据
	FILE* pfPointCloud = fopen(strPath + "RecordPointCloud.txt", "w");// 焊缝点云保存

	// 资源对象
	CDHGigeImageCapture* pDHCam = (CDHGigeImageCapture*)m_ptUnit->GetCameraCtrl(m_ptUnit->m_nTrackCameraNo); // 相机指针
	IplImage* pImageBuff = pDHCam->m_pImageBuff; // 图片缓冲区 //IplImage* pImageBuff = cvCreateImage(cvSize(tCameraParam.tDHCameraDriverPara.nMaxWidth, tCameraParam.tDHCameraDriverPara.nMaxHeight), IPL_DEPTH_8U, 1); // 图片缓冲区
	std::vector<TrackFilter_Node> vtCoutTrackPoint(0);

	pRobotCtrl->m_cLog->Write("[跟踪调度]:跟踪轨迹补偿：dInitComp:%lf dAbjustHeight:%lf", m_pTraceModel->m_dGunToEyeCompenX, m_pTraceModel->m_dGunToEyeCompenZ);

	// 开始跟踪过程
	while (FALSE == m_pTraceModel->m_bCameraFindWeldEnd)//判断找到结尾后不再采点0
	{
		if (pRobotCtrl->m_eThreadStatus == INCISEHEAD_THREAD_STATUS_STOPPED)
		{
			pRobotCtrl->m_cLog->Write("[跟踪调度]:急停结束跟踪线程： %d", pRobotCtrl->m_eThreadStatus);
			break;
		}

		// 终点判断：1.已知终点(干涉终点) 2.用理论长度(留焊) 3.焊接过程中判断(自由终点) 三种方式
		// 1、终点判断：更新终点坐标 并 设置m_pTraceModel->m_bCameraFindWeldEnd为true
		m_bDynamicJudgeEndPoint = true;
		JudgeFindWeldLineEnd(pRobotCtrl, nJudgeEndMethod);
		if (TRUE == m_pTraceModel->m_bCameraFindWeldEnd) break;

		// 2、读取坐标
		ReadTrackCapCoord(pRobotCtrl, tCapRobotCurCoord, tCapRobotCurPulse, tCapWorldCurCoord, nImageNo + 1);
		pRobotCtrl->m_cLog->Write("[读取坐标]");
		// 3、图片采集 图片序号nImageNo递增后作为新图片的索引, 调试模式 读取本地图片
		CHECK_BOOL_BREAK(CaptureTrackImage(m_ptUnit->m_nTrackCameraNo, nImageNo, &pImageBuff, bSaveImage, strSavePath));
		if (TRUE == m_pTraceModel->m_bCameraFindWeldEnd) break;
		pRobotCtrl->m_cLog->Write("[图片采集]");
		// 4、图像处理 处理失败结束本次循环 连续多次停止跟踪 正常复位累计计数
		CHECK_BOOL_CONTINUE(TrackProcess(pRobotCtrl, nImageNo, &pImageBuff, tCapRobotCurCoord, tCapRobotCurPulse, tWeldLineRobotCoord, tWeldLineWorldCoord, nImgProFailNum, nMaxImgProFailNum, pfRawData, pfPointCloud));
		if (TRUE == m_pTraceModel->m_bCameraFindWeldEnd) break;

		// 5、检测相邻两次处理是否过近 连续多次过近停止跟踪 正常复位累计计数 并 更新最后跟踪采图位置tLastCapWorldCoord 更新累计焊接长度
		CHECK_BOOL_CONTINUE(CheckAdjoinCapCoord(pRobotCtrl, nImageNo, tLastCapWorldCoord, tCapWorldCurCoord, dMinJoinPtnDis, nErrStopNum, nMaxErrStopNum));

		// 6、检测滤波是否正常 连续多次滤波失败停止跟踪 正常复位累计计数 输入最新焊缝坐标tWeldLineWorldCoord 输出滤波结果vtCoutTrackPoint 滤波输入数据存入文件pfRawData
		CHECK_BOOL_CONTINUE(CheckTrackFilterValid(pRobotCtrl, nImageNo, tWeldLineWorldCoord, vtCoutTrackPoint, nFilterFailNum, nMaxFilterFailNum));

		// 7、滤波输出数据处理 连续多次调整Z值失败 停止跟踪 正常复位累计计数 将vtCoutTrackPoint中的每个点添加补偿 写入文件 并 追加到跟踪焊接轨迹缓冲区
		CHECK_BOOL_BREAK(FilterResultTrans(pRobotCtrl, nImageNo, vtCoutTrackPoint, nAdjustZErrNum, nMaxAdjustZErrNum, pfFilterOut));

		Sleep(20);
		DoEvent();
	}
	// 发现终点后 获取并更新终点段数据
	if (TRUE == m_pTraceModel->m_bCameraFindWeldEnd)
	{
		// 获取跟踪结尾及终点坐标轨迹
		GetEndSectionCoorsNew(pRobotCtrl, m_pTraceModel->m_tRealEndpointCoor);
		// 根据包角参数 添加包角轨迹 

		// 跟踪结束后的相关变量
		m_pTraceModel->m_bIsTrackingDataSendOver = true; // !!!! 临时为发送包角轨迹提供判断条件

	}
	fclose(pfFilterOut);
	fclose(pfRawData);
	fclose(pfPointCloud);
	m_ptUnit->SwitchIO("TrackLaser", false); //跟踪流程结束关闭激光

	pRobotCtrl->m_cLog->Write("[跟踪调度]: 跟踪线程结束");
}

void CScanInitModule::ReadTrackCapCoord(CRobotDriverAdaptor* pRobotCtrl, T_ROBOT_COORS& tRobotCoord, T_ANGLE_PULSE& tRobotPulse, T_ROBOT_COORS& tWorldCoord, int nImageNo)
{
	if (GetLocalDebugMark())
	{
		if (nImageNo < 0) nImageNo = 0;
		if (nImageNo >= m_pTraceModel->vtTrackRawData.size()) nImageNo = m_pTraceModel->vtTrackRawData.size() - 1;
		if (nImageNo > m_pTraceModel->vtTrackRawData.size())
		{
			XUI::MesBox::PopInfo("调试获取图{0}的采集坐标失败 总数量{1}", nImageNo, m_pTraceModel->vtTrackRawData.size());
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
		//获取当前位置
		tRobotCoord = pRobotCtrl->GetCurrentPos();
		tRobotPulse = pRobotCtrl->GetCurrentPulse();
		//tRobotCoord.dBX = 0.0;
		//tRobotPulse.lBXPulse = 0;
		// 采图时机器人世界坐标
		tWorldCoord = T_ROBOT_COORS(tRobotCoord.dX + tRobotCoord.dBX, tRobotCoord.dY + tRobotCoord.dBY,
			tRobotCoord.dZ + tRobotCoord.dBZ, tRobotCoord.dRX, tRobotCoord.dRY, tRobotCoord.dRZ, 0.0, 0.0, 0.0);
	}
}

bool CScanInitModule::CaptureTrackImage(int nCameraNo, int &nImageNo, IplImage** ppImageBuff, bool bSaveImage, CString sSavePath)
{
	bool bSuccess = true;
	CString sImageFileName;
	nImageNo++; // 图片序号 索引从0开始
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
		CDHGigeImageCapture* pDHCam = (CDHGigeImageCapture*)m_ptUnit->GetCameraCtrl(m_ptUnit->m_nTrackCameraNo); // 相机指针
		if (!pDHCam->CaptureImage(*ppImageBuff,1))
		{
			return false;
		}
		//cvCopyImage(pDHCam->CaptureImage(FALSE), *ppImageBuff);
		if (bSaveImage)SaveImage(*ppImageBuff, "%s", sImageFileName); // 存图
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

	int nParamsArrayAAA[3];            //存图格式
	nParamsArrayAAA[0] = (int)CV_IMWRITE_JPEG_QUALITY;
	nParamsArrayAAA[1] = (int)(0.1 * 100);
	nParamsArrayAAA[2] = 0;
	cvSaveImage("AAA.jpg", /**ppImage*/ *ppImage, nParamsArrayAAA);
	FlipImage(*ppImage, eFlipMode); // 翻转

	IplImage* pImg = *ppImage;

	WidgetLaserInfo* LaserInfo = new WidgetLaserInfo[10];
	char* LogName = "WidgetLaserLock";

	int PtnNum = WidgetLaserLock(pImg, LaserInfo, true, false, LogName);
	if (PtnNum <= 0)
	{
		XiMessageBoxOk("示教失败，请检查激光图片AAA.jpg是否正常");

	}
	else if (PtnNum == 1)
	{
		cpMidKeyPoint = LaserInfo[0].crossPoint;
		for (int i = 0; i < LaserInfo[0].samPntNum; i++)
		{
			// 激光图左侧激光线 立板
			vtLeftPtns.push_back(LaserInfo[0].verticalPlateLineSamPnt[i]);
			// 激光图右侧激光线 底板
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
			// 激光图左侧激光线 立板
			vtLeftPtns.push_back(LaserInfo[realKey].verticalPlateLineSamPnt[i]);
			// 激光图右侧激光线 底板
			vtRightPtns.push_back(LaserInfo[realKey].bottomPlateLineSamPnt[i]);
		}
	}

	/*pTraceImgProcess->GetBesideKeyPoints(*ppImage, cpMidKeyPoint, cpBesideKeyPoint, vtLeftPtns, vtRightPtns, false,
		500, 500,
		300, 300,
		20, 20);*/
	if (vtLeftPtns.size() <= 0 || vtRightPtns.size() <= 0)
	{
		XiMessageBox("跟踪斜率锁定失败");
		return false;
	}

	LeftPtn = vtLeftPtns[vtLeftPtns.size() / 2];
	RightPtn = vtRightPtns[vtRightPtns.size() / 2];

	ResumeImagePoint2D((*ppImage)->width, (*ppImage)->height, cpMidKeyPoint, eFlipMode);
	ResumeImagePoint2D((*ppImage)->width, (*ppImage)->height, LeftPtn, eFlipMode);
	ResumeImagePoint2D((*ppImage)->width, (*ppImage)->height, RightPtn, eFlipMode); // 二维点恢复
	tKeyPoint = pTraceImgProcess->HandlockKeyPoint(cpMidKeyPoint, LeftPtn, RightPtn, tFrontLine, tBackLine);

	CvPoint p1 = cvPoint(0, tFrontLine.b);
	CvPoint p2 = cvPoint((*ppImage)->width, (*ppImage)->width * tFrontLine.k + tFrontLine.b);
	CvPoint p3 = cvPoint(0, tBackLine.b);
	CvPoint p4 = cvPoint((*ppImage)->width, (*ppImage)->width * tBackLine.k + tBackLine.b);

	FlipImage(*ppImage, eFlipMode); // 翻转恢复

	cvCvtColor(*ppImage, m_pShowImage, CV_GRAY2RGB);
	cvCircle(m_pShowImage, tKeyPoint, 10, CV_RGB(255, 0, 0), 3);
	cvLine(m_pShowImage, p1, p2, CV_RGB(0, 255, 0), 2);
	cvLine(m_pShowImage, p3, p4, CV_RGB(0, 0, 255), 2);
	CString cstrTemp = OUTPUT_PATH + m_ptUnit->GetUnitName() + SEARCH_SCANLOCK_IMG;
	int nParamsArray[3];            //存图格式
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
		cstr.Format("第%d张图", nImageNo);
		TRACE(cstr.GetBuffer());
	}
	else
	{
		bool bProcRst = WeightedPointCluster(*pImageBuff, &m_pTraceModel->tFrontLine, &m_pTraceModel->tBackLine, &Corner2D, false, "WeightedPointCluster");
		CString cstr;
		cstr.Format("第%d张图", nImageNo);
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

	// 有点云处理结果时使用点云处理数据
	if (m_pTraceModel->vtPointCloudEndPoints.size() > 0)
	{
		Three_DPoint tPtn = m_pTraceModel->vtPointCloudEndPoints.back();
		tWeldLineWorldCoord.dX = tPtn.x;
		tWeldLineWorldCoord.dY = tPtn.y;
		tWeldLineWorldCoord.dZ = tPtn.z;
		T_ROBOT_COORS tRefCoord(tWeldLineRobotCoord);
		tWeldLineRobotCoord = tWeldLineWorldCoord;
		SplitCoord(tRefCoord, tWeldLineRobotCoord, m_ptUnit->m_nTrackAxisNo);
		pRobotCtrl->m_cLog->Write("[图像处理]: 使用点云处理结果 图号%d", nImageNo);
	}

	// 点云数据保存
	m_MutexTrackPointCloud.lock();
	T_ROBOT_COORS tPointCloudRobotCoord;
	T_ROBOT_COORS tPointCloudWorldCoord; 
	Three_DPoint tThreePoint;
	int nSaveNum = 0;
	for (int i = 0; i < nLength; i++)
	{
		if (points[i].x < nMinX || points[i].x > nMaxX) continue; // 无效区域点云数据不保存
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
		fprintf(pfPointCloud, "图号:%d 点号%d %11.3lf%11.3lf%11.3lf\n", nImageNo, i, tPointCloudWorldCoord.dX, tPointCloudWorldCoord.dY, tPointCloudWorldCoord.dZ);
	}

	m_pTraceModel->lnImageNo.push_back(nImageNo); 
	m_pTraceModel->lnImagePtnNum.push_back(nSaveNum); 
	m_pTraceModel->ltCapWorldCoord.push_back(tRobotCoord);
	// 删除旧点云数据
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

	pRobotCtrl->m_cLog->Write("[图像处理]:%s 图片号:%d 焊缝世界坐标：%.3lf %.3lf %.3lf 采图世界坐标：%.3lf %.3lf %.3lf", strRobot, nImageNo,
		tWeldLineRobotCoord.dX, tWeldLineRobotCoord.dY, tWeldLineRobotCoord.dZ, 
		tRobotCoord.dX + tRobotCoord.dBX, tRobotCoord.dY + tRobotCoord.dBY, tRobotCoord.dZ + tRobotCoord.dBZ);
	if (!GetLocalDebugMark())
	{
		// 跟踪采集原始数据: 图片序号n、二维点x y、焊缝世界坐标x y z、采图直角坐标x y z rx ry rz bx by bz、采图关节坐标S L U R B T BX BY BZ
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

	// 处理成功？
	if (1 > Corner2D.x || 1 > Corner2D.y || 2448 < Corner2D.x || 2048 < Corner2D.y)
	{
		pRobotCtrl->m_cLog->Write("[图像处理]:%s kong Corner2D.x:%d Corner2D.y:%d", strRobot, Corner2D.x, Corner2D.y);
		nFailNum++;
		if (nFailNum > nMaxFailNum)
		{
			pRobotCtrl->m_cLog->Write("[图像处理]:%s 连续%d次找点数值出错，暂停焊接", strRobot, nMaxFailNum);
			CheckMachineEmg(pRobotCtrl);
			XUI::MesBox::PopOkCancel("[图像处理]:{0}号机器人 连续{1}多次找点数值出错，暂停焊接", pRobotCtrl->m_nRobotNo, nMaxFailNum);
		}
		bSuccess = false;
	}
	pRobotCtrl->m_cLog->Write("[图像处理]:%d 单次处理时间:%dms %d %d 中心点提取数量：%d",
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
		pRobotCtrl->m_cLog->Write("[距离检查]: %s 测量数据与上次距离小于%.3lfmm,不进行处理:%.3lf", pRobotCtrl->m_strRobotName, dDisThreshold, dLatestDistance);
		nErrNum++;
		if (nErrNum >= nMaxErrNum)
		{
			CheckMachineEmg(pRobotCtrl);
			XUI::MesBox::PopOkCancel("[距离检查]: {0}号机器人大车停动!已息弧!可断点续焊", pRobotCtrl->m_nRobotNo);
		}
	}
	else
	{
		// 更新最后一次采图世界坐标
		tLastCapWorldCoord = tCurCapWorldCoord;
		nErrNum = 0;
		bValid = true;
		m_pTraceModel->m_dCurrentWeldLen += dLatestDistance;
	}
	pRobotCtrl->m_cLog->Write("[距离检查]: %s 图号:%d 工件理论总长度：%11.3lf,当前焊接长度:%11.3lf，移动距离:%lf 当前世界坐标：%.3lf %.3lf %.3lf",
		pRobotCtrl->m_strRobotName, nImageNo, m_pTraceModel->m_dWeldLen, m_pTraceModel->m_dCurrentWeldLen, dLatestDistance,
		tCurCapWorldCoord.dBX, tCurCapWorldCoord.dBY, tCurCapWorldCoord.dBZ);
	return bValid;
}

bool CScanInitModule::CheckTrackFilterValid(CRobotDriverAdaptor* pRobotCtrl, int nImageNo, T_ROBOT_COORS tInputCoord, std::vector<TrackFilter_Node>& vtCoutTrackPoint, int &nInvalidNum, int nMaxInvalidNum)
{
	bool bValid = true;
	vtCoutTrackPoint.clear();
	CString strRobot = pRobotCtrl->m_strRobotName;
	pRobotCtrl->m_cLog->Write("[滤波处理]:%s 滤波前数据:图号:%d, %11.3lf %11.3lf %11.3lf 累计无效次数:%d", strRobot, nImageNo, tInputCoord.dX, tInputCoord.dY, tInputCoord.dZ, nInvalidNum);

	vtCoutTrackPoint = TrackFilter_FilterCurvePointInPointOut(m_ptUnit->m_nRobotSmooth, tInputCoord.dX, tInputCoord.dY, tInputCoord.dZ, 1 == pRobotCtrl->m_nRobotInstallDir ? false : true);
	// 滤波成功？
	if (vtCoutTrackPoint.size() < 1)
	{
		pRobotCtrl->m_cLog->Write("[滤波处理]:%s CoutTrackPoint数据为空 %d", strRobot, vtCoutTrackPoint.size());
		nInvalidNum++;
		if (nInvalidNum > nMaxInvalidNum)
		{
			CheckMachineEmg(pRobotCtrl);
			pRobotCtrl->m_cLog->Write("[滤波处理]:%s CoutTrackPoint数据为空次数过多，一般这不太正常，停弧保护，请抬枪后继续焊接", strRobot);
			XUI::MesBox::PopInfo("[滤波处理]:{0} CoutTrackPoint数据为空次数过多，一般这不太正常，停弧保护，请抬枪后继续焊接", strRobot);
		}
		bValid = false;;
	}
	else
	{
		bValid = true;
		nInvalidNum = 0; // 滤波失败次数清空复位
	}
	return bValid;
}

bool CScanInitModule::FilterResultTrans(CRobotDriverAdaptor* pRobotCtrl, int nImageNo, const std::vector<TrackFilter_Node>& vtCoutTrackPoint, int& nAdjustErrNum, int nMaxAdjustErrNum, FILE* pfFile)
{
	bool bSuccess = true;
	CString strRobot = pRobotCtrl->m_strRobotName;
	double dTrackVerDegree = 0.0;
	double dInitComp = m_pTraceModel->m_dGunToEyeCompenX;// 滤波后加补偿值：外加内减
	double dAbjustHeight = m_pTraceModel->m_dGunToEyeCompenZ;// 滤波后加补偿值：向上补加 向下补减

	for (int nSize = 0; nSize < vtCoutTrackPoint.size(); nSize++)
	{
		dTrackVerDegree = atan2(vtCoutTrackPoint[nSize].normalDir_.y_, vtCoutTrackPoint[nSize].normalDir_.x_) * 180.0 / PI;
		T_ROBOT_COORS tLastTrackCoord = pRobotCtrl->m_vtWeldLineInWorldPoints.back();
		T_ROBOT_COORS tFilterOutCoord(
			vtCoutTrackPoint[nSize].pnt_.x_ + (dInitComp * CosD(dTrackVerDegree)),
			vtCoutTrackPoint[nSize].pnt_.y_ + (dInitComp * SinD(dTrackVerDegree)),
			vtCoutTrackPoint[nSize].pnt_.z_ + (dAbjustHeight),
			tLastTrackCoord.dRX, tLastTrackCoord.dRY, tLastTrackCoord.dRZ, 0.0, 0.0, 0.0);

		pRobotCtrl->m_cLog->Write("[轨迹添加]: %s 图号:%d 滤波后补偿: %.3lf  %.3lf  %.3lf 滤波法向角:%.3lf 滤波法向:%.3lf %.3lf %.3lf  ",
			strRobot, nImageNo, tFilterOutCoord.dX, tFilterOutCoord.dY, tFilterOutCoord.dZ, dTrackVerDegree,
			vtCoutTrackPoint[nSize].normalDir_.x_, vtCoutTrackPoint[nSize].normalDir_.y_, vtCoutTrackPoint[nSize].normalDir_.z_);
		if (!GetLocalDebugMark())
		{
			// 图片序号n 滤波原始输出xyz 滤波输出添加补偿xyz 滤波法向向量xyz 滤波法向角A 水平补偿值a 高度补偿值b
			fprintf(pfFile, "%d %11.3lf%11.3lf%11.3lf %11.3lf%11.3lf%11.3lf %8.3lf%8.3lf%8.3lf %8.3lf%8.3lf%8.3lf\n",
				nImageNo, vtCoutTrackPoint[nSize].pnt_.x_, vtCoutTrackPoint[nSize].pnt_.y_, vtCoutTrackPoint[nSize].pnt_.z_,
				tFilterOutCoord.dX, tFilterOutCoord.dY, tFilterOutCoord.dZ,
				vtCoutTrackPoint[nSize].normalDir_.x_, vtCoutTrackPoint[nSize].normalDir_.y_, vtCoutTrackPoint[nSize].normalDir_.z_,
				dTrackVerDegree, dInitComp, dAbjustHeight);
			fflush(pfFile);
		}

		double dAdjustZ = tFilterOutCoord.dZ - tLastTrackCoord.dZ;
		// Z值调整正常？
		if (fabs(dAdjustZ) > 4.0) // 机器人高度变化大时不修改
		{
			double dRealAdjustZ = dAdjustZ > 0.0 ? 2.0 : -2.0;
			tFilterOutCoord.dZ = tLastTrackCoord.dZ + dRealAdjustZ;
			nAdjustErrNum++;
			pRobotCtrl->m_cLog->Write("[轨迹添加]:%s 滤波调整高度dAdjustZ:%.3lf 实际调整高度dRealAdjustZ:%.3lf nAbjustZErrNum:%d nMaxAdjustErrNum:%d",
				strRobot, dAdjustZ, dRealAdjustZ, nAdjustErrNum, nMaxAdjustErrNum);
			if (nAdjustErrNum > nMaxAdjustErrNum)
			{
				CheckMachineEmg(pRobotCtrl);
				pRobotCtrl->m_cLog->Write("[轨迹添加]:%s z值调整错误次数过多，停弧保护，请抬枪后继续焊接", strRobot);
				XiMessageBox("[轨迹添加]: Z值调整错误次数过多，停弧保护，请抬枪后继续焊接"); 
				bSuccess = false;
				break;
			}
		}
		else
		{
			nAdjustErrNum = 0;
		}

		//----------------------- 圆弧跟踪姿态调整计算 相对于焊接起点姿态的Rz差值
		long lMinAjustRzTimeInterval = 1000;
		long lCurTimeTick = GetTickCount();
		long lTimeInterval = lCurTimeTick - m_pTraceModel->m_lPreAdjustRzTimeTick;
		if (labs(lTimeInterval) > lMinAjustRzTimeInterval && true == m_bWorkpieceShape) // 圆弧调整姿态
		{
			m_pTraceModel->m_lPreAdjustRzTimeTick = lCurTimeTick; // 更新时间戳
			double dDefaultRz = pRobotCtrl->m_vtWeldLineInWorldPoints.front().dRZ; // tFilterOutCoord.dRZ;
			double dTestRz = DirAngleToRz(dTrackVerDegree);//计算机器人Rz  根据实时变化的前进法相角()和U轴角度计算Rz 																						  // 临近几个点的原始dTestRz取平均值再使					 

			pRobotCtrl->m_cLog->Write("法相角计算RZ:%lf dDefaultRz:%.3lf", dTestRz, dDefaultRz);

			// 控制计算Rz在0-360以内
			dTestRz = fmod(dTestRz, 360.0);
			if (dTestRz < 0.0) {
				dTestRz += 360.0; // 0 - 360
			}
			double dRzChange = dTestRz - dDefaultRz;
			dRzChange = fmod(dRzChange, 360.0);
			if (dRzChange < -180)//防止出现dTestRz从360到0
			{
				dRzChange += 360.0; // 0 - 360
			}
			else if (dRzChange > 180)
			{
				dRzChange -= 360.0;
			}
			//本次RZ和上次记录RZ差值
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

			if ((fabs(dRzBefAndAftDiff) > dAdjustMaxInterval)) // 姿态角度补偿最大变化值限制
			{
				double dDir = dRzBefAndAftDiff > 0.0 ? 1.0 : -1.0;
				dRzChange = m_pTraceModel->m_dPreRzChange + (dAdjustMaxInterval * dDir);
				nAdjustRzErrorNum++;
				if (nAdjustRzErrorNum > 10)
				{
					CheckMachineEmg(pRobotCtrl);
					pRobotCtrl->m_cLog->Write("%s Rz值调整错误次数过多，停弧保护，请抬枪后继续焊接", strRobot);
					XiMessageBox("Rz值调整错误次数过多，停弧保护，请抬枪后继续焊接");
					break;
				}
			}
			else
			{
				nAdjustRzErrorNum = 0;
			}
			pRobotCtrl->m_cLog->Write("%s 法相计算dTestRz: %lf RZ差值: %lf dRzChange %lf dPreRzChange:%lf",
				strRobot, dTestRz, dRzBefAndAftDiff, dRzChange, m_pTraceModel->m_dPreRzChange);
			if ((fabs(dRzBefAndAftDiff) < dAdjustMinInterval)) // 姿态角度补偿最小变化值限制
			{
				dRzChange = m_pTraceModel->m_dPreRzChange;
			}
			m_pTraceModel->m_dPreRzChange = dRzChange; // 记录上一次Rz补偿 P99
			m_pTraceModel->m_dAdjustRobotRZ = dRzChange;
			//--------------------------
		}

		pRobotCtrl->m_cLog->Write("[轨迹添加]: %s 图号:%d 跟踪世界数据： %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf ",
			strRobot, nImageNo, tFilterOutCoord.dX, tFilterOutCoord.dY, tFilterOutCoord.dZ, tFilterOutCoord.dRX, tFilterOutCoord.dRY, tFilterOutCoord.dRZ);

		// 将tFilterOutCoord世界坐标XYZ拆分到外部轴
		T_ROBOT_COORS tRobotStartCoord = pRobotCtrl->m_vtWeldLineInWorldPoints[0];
		SplitCoord(tRobotStartCoord, tFilterOutCoord, m_ptUnit->m_nTrackAxisNo);
		pRobotCtrl->m_vtWeldLineInWorldPoints.push_back(tFilterOutCoord);
		m_pTraceModel->m_vtWeldLinePointType.push_back(E_WELD_TRACK);
		SaveRobotCoor(m_pRecordTheoryPoint, tFilterOutCoord, E_WELD_TRACK);//保存理论数据
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
	// 判断方法: 1.已知终点(干涉终点) 2.用理论长度(留焊) 3.焊接过程中判断(自由终点)
	bool bRst = false;
	switch (nMethod)
	{
	case 1: /*bRst = JudgeEnd_Know(pRobotCtrl);*/ break;
	case 2: bRst = JudgeEnd_TheoryLength(pRobotCtrl); break;
	case 3: bRst = JudgeEnd_ProcessSearch(pRobotCtrl); break;
	default:XUI::MesBox::PopInfo("不存在跟踪终点判断方法!");break;
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
	if (dDis < dDisThreshold) // 需要优化 焊接过程中变形 应将理论端点想跟踪轨迹直线上投影后 计算距离！！！！！！！！！！！！！
	{
		bFond = true;
		// 发现终点
		m_pTraceModel->m_bCameraFindWeldEnd = true;
	}
	return bFond;
}

bool CScanInitModule::JudgeEnd_TheoryLength(CRobotDriverAdaptor* pRobotCtrl)
{	
	//! 注意此函数 对于发现中点后 需要使用vtCoutTrackPoint修改焊接轨迹缓冲区 否则端点不准(这种方式本身就无法做到准确)
	bool bIsFindEnd = false;
	double dOverLength = 999999.999;
	std::vector<TrackFilter_Node> vtCoutTrackPoint(0);
	bool bNormalDir = (1 == pRobotCtrl->m_nRobotInstallDir) ? false : true;
	//! 判断是否完成理论移动长度, return true : 完成移动, 否则未完成.- overlength : 提供一个double指针, 获取剩余长度.
	bool bIsOver = TrackFilter_DoneMove(m_ptUnit->m_nRobotSmooth, &dOverLength);
	if (bIsOver)
	{
		bIsFindEnd = true;
		//! 在 TrackFilter_DoneMove() == true时，调用该函数，获取直到理论长度结束 时的轨迹点集.
		//!  或在 overlength 足够小时, 调用该函数, 强制获取到结尾点. 此时获取的结尾点为直线延长.
		TrackFilter_Answer ans = TrackFilter_GetToEndPnts(m_ptUnit->m_nRobotSmooth, bNormalDir);

		size_t index = 0;
		for (index = 0; index < ans.araaySize_; ++index) {
			vtCoutTrackPoint.push_back(ans.segmentArray_[index]);
			pRobotCtrl->m_cLog->Write("[终点判断]:理论长度判断结尾Ptn%d: %11.3lf%11.3lf%11.3lf", index,
				ans.segmentArray_[index].pnt_.x_, ans.segmentArray_[index].pnt_.y_, ans.segmentArray_[index].pnt_.z_);
		}
		// 更新终点
		m_pTraceModel->m_tRealEndpointCoor = T_ROBOT_COORS(
			ans.segmentArray_[ans.araaySize_ - 1].pnt_.x_,
			ans.segmentArray_[ans.araaySize_ - 1].pnt_.y_,
			ans.segmentArray_[ans.araaySize_ - 1].pnt_.z_, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
		TrackFilter_Free(ans);
		// 发现终点
		m_pTraceModel->m_bCameraFindWeldEnd = TRUE; 
		pRobotCtrl->m_cLog->Write("[终点判断]:理论长度判断结尾：达到结尾点：%d", vtCoutTrackPoint.size());
	}
	pRobotCtrl->m_cLog->Write("[终点判断]:是否到达结尾理论长度：%d 距结尾理论长度：%lf ", bIsOver, dOverLength);
	return bIsFindEnd;
}

bool CScanInitModule::JudgeEnd_ProcessSearch(CRobotDriverAdaptor* pRobotCtrl)
{
	// 何时开启：全程开启？焊接一半? 距离理论终点足够短距离？
	// 全程开启的好处：跟踪处理失败后 可以使用最新点云处理的结果
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
	double dSameDisThreshold = 1.0;		// 终点相同距离阈值判断 （视觉处理 精度高需要减小此阈值 反之需要增大）
	int nSameEndPtnNumThreshold = PARA_FLAT_TRACK(nSameEndPtnNumThreshold);	// 最新端点和此前nSamePtnNumThreshold个点距离小于dSameDisThreshold 发现终点 
	int nMinProcessImageNum = 60;		// 最少多少张图的局部点云才处理一次
	double dProcessSampDis = 2.0;		// 点云提取结果中点间距
	int nWaitTimeStep = 100;			// 等待运动检查时间步距
	bool bIsRunning = false;
	int nPreImageNoE = 0;
	int nErrorNum = 0;					// 连续处理出错计数
	int nErrorThreshold = 3;			// 连续处理出错三次结束流程
	bool bResult = true;				// 返回结果
	std::vector<Three_DPoint> vtThreePoint(0);
	CString sTempFileName;

	// 等待跟踪运动
	int nCurWaitTime = 0;
	bIsRunning = m_ptUnit->WorldIsRunning();
	while (!bIsRunning)
	{
		Sleep(nWaitTimeStep);
		nCurWaitTime += nWaitTimeStep;
		if (nCurWaitTime > nTimeOut)
		{
			pRobotCtrl->m_cLog->Write("[过程搜端点]: 终点判断等待运动超时！");
			return false;
		}
		bIsRunning = m_ptUnit->WorldIsRunning();
	}

	// 运动中 且 未发现终点
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

		// 互斥拷贝出最新数据
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

		if (nImageNoE <= nPreImageNoE) // 每更新一张图最多只处理一次点云
		{
			DELETE_POINTER_ARRAY(pThreeDPoint);
			Sleep(nWaitTimeStep);
			continue;
		}
		nPreImageNoE = nImageNoE;

		// 准备处理局部点云输入数据
		CvPoint3D32f tRefPtn[2] = { 0.0 };
		//参考方向从扫描方向改为理论焊道方向
		//tRefPtn[0] = cvPoint3D32f(tScanS.dX + tScanS.dBX, tScanS.dY + tScanS.dBY, tScanS.dZ);
		//tRefPtn[1] = cvPoint3D32f(tScanE.dX + tScanE.dBX, tScanE.dY + tScanE.dBY, tScanE.dZ);
		T_ROBOT_COORS tRealScanE, tRealScanS;
		if (!pRobotCtrl->MoveToolByWeldGun(tScanE, pRobotCtrl->m_tTools.tCameraTool, tScanE, pRobotCtrl->m_tTools.tGunTool, tRealScanE)
			|| !pRobotCtrl->MoveToolByWeldGun(tScanS, pRobotCtrl->m_tTools.tCameraTool, tScanS, pRobotCtrl->m_tTools.tGunTool, tRealScanS))
		{
			pRobotCtrl->m_cLog->Write("使用旧方法赋予参考方向，可能有问题");
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

		// 保留每次处理输入数据
		fprintf(pfPointCloudInput, "nImageNoS:%d nImageNoE:%d tCameraNorm:%11.3lf%11.3lf%11.3lf tPlaneHNorm:%11.3lf%11.3lf%11.3lf tRefPtn[0]:%11.3lf%11.3lf%11.3lf tRefPtn[1]:%11.3lf%11.3lf%11.3lf\n",
			nImageNoS, nImageNoE, tCameraNorm.x, tCameraNorm.y, tCameraNorm.z, tPlaneHNorm.x, tPlaneHNorm.y, tPlaneHNorm.z,
			tRefPtn[0].x, tRefPtn[0].y, tRefPtn[0].z, tRefPtn[1].x, tRefPtn[1].y, tRefPtn[1].z);
		fflush(pfPointCloudInput);
		
		try
		{
			// 点云处理
			int nWeldInfoNum = GetRiserEndPoint((CvPoint3D64f*)pThreeDPoint, nPtnNum, tWeldInfo, tCameraNorm, tPlaneHNorm, tRefPtn, 2, "GetRiserEndPoint");

			double dDis = TwoPointDis(tWeldInfo[0].staPnt.x, tWeldInfo[0].staPnt.y, tWeldInfo[0].staPnt.z, tWeldInfo[0].endPnt.x, tWeldInfo[0].endPnt.y, tWeldInfo[0].endPnt.z);
			double dDisX = tWeldInfo[0].endPnt.x - tWeldInfo[0].staPnt.x;
			double dDisY = tWeldInfo[0].endPnt.y - tWeldInfo[0].staPnt.y;
			double dDisZ = tWeldInfo[0].endPnt.z - tWeldInfo[0].staPnt.z;

			// 处理有焊缝结果 || 焊缝起点终点不相近 || 连续错误数量不超过阈值 否则失败次数递增1
			if ((0 >= nWeldInfoNum) || (dDis < dProcessSampDis) || (nErrorNum >= nErrorThreshold))
			{
				pRobotCtrl->m_cLog->Write("[过程搜端点]: 点云端点提取失败：ImgNo_S%d-E%d", nImageNoS, nImageNoE);
				sTempFileName.Format("%sImgNo_S%d-E%d-处理失败点云.txt", OUTPUT_PATH + pRobotCtrl->m_strRobotName + ERROR_PATH, nImageNoS, nImageNoE);
				SavePointCloudProcErrData(sTempFileName, nImageNoS, nImageNoE, tCameraNorm, tPlaneHNorm, tRefPtn, 2, pThreeDPoint, nPtnNum);
				DELETE_POINTER_ARRAY(pThreeDPoint);
				nErrorNum++;
				continue;
			}
			if (nErrorNum > nErrorThreshold) // 连续失败多次
			{
				DELETE_POINTER_ARRAY(pThreeDPoint);
				CheckMachineEmg(pRobotCtrl);
				XUI::MesBox::PopOkCancel("[过程搜端点]:GetRiserEndPoint无结果! 图号{0}-{1}", nImageNoS, nImageNoE);
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
			// 异常捕获 此处可以不结束，先增加连续失败次数变量
			sTempFileName.Format("%sImgNo_S%d-E%d_处理异常点云.txt",
				OUTPUT_PATH + pRobotCtrl->m_strRobotName + ERROR_PATH, nImageNoS, nImageNoE);
			SavePointCloudProcErrData(sTempFileName, nImageNoS, nImageNoE, tCameraNorm, tPlaneHNorm, tRefPtn, 2, pThreeDPoint, nPtnNum);
			fclose(pfPointCloudInput);
			CheckMachineEmg(pRobotCtrl);
			//已修改
			XUI::MesBox::PopInfo("{0} [过程搜端点]:点云处理异常! 图号{1}-{2}", pRobotCtrl->m_strRobotName.GetBuffer(), nImageNoS, nImageNoE);
			//XiMessageBox("%s [过程搜端点]:点云处理异常! 图号%d-%d", pRobotCtrl->m_strRobotName, nImageNoS, nImageNoE);
			bResult = false;
			DELETE_POINTER_ARRAY(pThreeDPoint);
			break;
		}

		// 保存单次处理结果
		sTempFileName.Format("%sImgNo_S%d-E%d_Result.txt",
			OUTPUT_PATH + pRobotCtrl->m_strRobotName + WELDDATA_PATH, nImageNoS, nImageNoE, vtThreePoint.size());
		FILE* pf = fopen(sTempFileName.GetBuffer(), "w");
		for (int i = 0; i < vtThreePoint.size(); i++)
		{
			fprintf(pf, "开始图号%d 结束图号%d %11.3lf%11.3lf%11.3lf\n",
				nImageNoS, nImageNoE, vtThreePoint[i].x, vtThreePoint[i].y, vtThreePoint[i].z);
		}
		fclose(pf);

		Three_DPoint tNewEndPtn = vtThreePoint.back();
		if (m_pTraceModel->vtPointCloudEndPoints.size() <= 0) // 第一次保存所有处理结果
		{
			for (int i = 0; i < vtThreePoint.size(); i++)
			{
				m_pTraceModel->vtPointCloudEndPoints.push_back(vtThreePoint[i]);
			}
		}
		else // 后续处理只保存最后一个点
		{
			m_pTraceModel->vtPointCloudEndPoints.push_back(tNewEndPtn);
		}

		pRobotCtrl->m_cLog->Write("[过程搜端点]: 点云端点提取终点：ImgNo_S%d-E%d_Result%d %11.3lf%11.3lf%11.3lf 耗时:%dms",
			nImageNoS, nImageNoE, vtThreePoint.size(), tNewEndPtn.x, tNewEndPtn.y, tNewEndPtn.z, XI_clock() - lTime);

		DELETE_POINTER_ARRAY(pThreeDPoint);

		// 判断终点
		if (m_bDynamicJudgeEndPoint)
		{
			if ((E_JUDGE_END_PROCESS_SEARCH == m_eJudgeEndMethod) && (JudgeFindEndPoint(m_pTraceModel->vtPointCloudEndPoints, dSameDisThreshold, nSameEndPtnNumThreshold)))
			{
				pRobotCtrl->m_cLog->Write("[过程搜端点]: JudgeFindEndPoint 发现终点 E_JUDGE_END_PROCESS_SEARCH");
				break;
			}
			
			if (E_JUDGE_END_KNOW == m_eJudgeEndMethod && JudgeEnd_Know(m_pTraceModel->vtPointCloudEndPoints, m_pTraceModel->m_tRealEndpointCoor))
			{
				pRobotCtrl->m_cLog->Write("[过程搜端点]: JudgeFindEndPoint 发现终点 E_JUDGE_END_KNOW");
				break;
			}
		}

		bIsRunning = m_ptUnit->WorldIsRunning();
	}
	fclose(pfPointCloudInput);
	
	if (!bIsRunning)
	{
		pRobotCtrl->m_cLog->Write("[过程搜端点]: 运动异常停止，终点判断结束!");
	}
	else
	{
		pRobotCtrl->m_cLog->Write("[过程搜端点]: 终点判断正常结束!");
	}
	return bResult;
}

bool CScanInitModule::GetEndSectionCoorsNew(CRobotDriverAdaptor* pRobotDriver, T_ROBOT_COORS tEndpointCoor)
{
	// ！！！！注意 tEndpointCoor为终点焊缝世界坐标 仅XYZ有值 其他值是0
	double dEndToLastDisThreshold = 130.0; // 终点距离最新跟踪轨迹点距离阈值

	int nSize = pRobotDriver->m_vtWeldLineInWorldPoints.size();
	T_ROBOT_COORS tTrackLastCoord = pRobotDriver->m_vtWeldLineInWorldPoints[nSize - 1];
	T_ROBOT_COORS tEndpointTemp = tEndpointCoor;

	pRobotDriver->m_cLog->Write("[终点处理]: 终点：%.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf 最后轨迹点：%.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf",
		tEndpointCoor.dX, tEndpointCoor.dY, tEndpointCoor.dZ, 
		tEndpointCoor.dRX, tEndpointCoor.dRY, tEndpointCoor.dRZ, 
		tEndpointCoor.dBX, tEndpointCoor.dBY, tEndpointCoor.dBZ,
		tTrackLastCoord.dX, tTrackLastCoord.dY, tTrackLastCoord.dZ, 
		tTrackLastCoord.dRX, tTrackLastCoord.dRY, tTrackLastCoord.dRZ, 
		tTrackLastCoord.dBX, tTrackLastCoord.dBY, tTrackLastCoord.dBZ);

	// 取后面几个点拟合焊缝终点附近直线
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
	// 判断终点和最后一个跟踪轨迹点距离是否超限
	double dDisError = TwoPointDis(
		tEndpointCoor.dX + tEndpointCoor.dBX, tEndpointCoor.dY + tEndpointCoor.dBY, tEndpointCoor.dZ + tEndpointCoor.dBZ,
		tTrackLastCoord.dX + tTrackLastCoord.dBX, tTrackLastCoord.dY + tTrackLastCoord.dBY, tTrackLastCoord.dZ + tTrackLastCoord.dBZ);
	if (dDisError > dEndToLastDisThreshold)
	{
		tEndpointCoor = pRobotDriver->m_vtWeldLineInWorldPoints.at(nSize - 1);
		//若提前停止则不保角焊接且立即停止焊接，并提示
		//m_pTraceModel->m_eWrapAngleType = E_WRAPANGLE_EMPTY_SINGLE;
		//SetIntValFun(pRobotDriver, 7, 0);
		pRobotDriver->m_cLog->Write(" 结尾点判断失误将以上一点为结尾点停止设备，并将包角类型置为不保角：%lf ", dDisError);
		m_pTraceModel->m_tRealEndpointCoor = tEndpointCoor;
		return false;
	}
	// 初始化结尾段滤波函数
	TrackSmooth_Init(m_ptUnit->m_nRobotSmooth, m_pTraceModel->m_dMoveStepDis, tTrackLastCoord.dX + tTrackLastCoord.dBX, 
		tTrackLastCoord.dY + tTrackLastCoord.dBY, tTrackLastCoord.dZ + tTrackLastCoord.dBZ);

	XI_POINT tEndpoint = { tEndpointCoor.dX + tEndpointCoor.dBX, tEndpointCoor.dY + tEndpointCoor.dBY, tEndpointCoor.dZ + tTrackLastCoord.dBZ };
	// 投影
	XI_POINT tProjectpoint;
	PointtoLineProjection(tLine, tEndpoint, tProjectpoint);
	tEndpointCoor.dX = tProjectpoint.x;
	tEndpointCoor.dY = tProjectpoint.y;
	tEndpointCoor.dZ = tProjectpoint.z;
	pRobotDriver->m_cLog->Write("[终点处理]: 结尾点投影:%11.3lf%11.3lf%11.3lf", tEndpointCoor.dX, tEndpointCoor.dY, tEndpointCoor.dZ);

	// 获取结尾段数据 二次滤波获 放入终点 取终点 Start  （算法终点获取函数需要修改）
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
		SplitCoord(tTrackLastCoord, tTempCoors, m_ptUnit->m_nTrackAxisNo); // 拆分坐标到外部轴
		pRobotDriver->m_vtWeldLineInWorldPoints.push_back(tTempCoors);
		m_pTraceModel->m_vtWeldLinePointType.push_back(E_WELD_TRACK);
		//SaveRobotCoor(m_pRecordTheoryPoint, tTempCoors, E_WELD_TRACK);//保存理论数据
		pRobotDriver->m_cLog->Write("[终点处理]: TrackSmooth_PushNextPoint: %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf",
			tTempCoors.dX, tTempCoors.dY, tTempCoors.dZ, tTempCoors.dRX, tTempCoors.dRY, tTempCoors.dRZ, tTempCoors.dBX, tTempCoors.dBY, tTempCoors.dBZ);
	}
	TrackSmooth_FreePntArray(&tTrackPtnArray);

	//获取结尾点，仅结尾时调用，与上一点的距离小于 moveMinDis 
	TrackSmooth_GetEndPoint(m_ptUnit->m_nRobotSmooth, &tTrackPtnArray);
	for (int n = 0; n < tTrackPtnArray.arraySize_; n++)
	{
		tTempCoors = T_ROBOT_COORS(tTrackPtnArray.pnts_[n].x_, tTrackPtnArray.pnts_[n].y_, tTrackPtnArray.pnts_[n].z_,
			tTrackLastCoord.dRX, tTrackLastCoord.dRY, tTrackLastCoord.dRZ, 0.0, 0.0, 0.0);
		SplitCoord(tTrackLastCoord, tTempCoors, m_ptUnit->m_nTrackAxisNo); // 拆分坐标到外部轴
		pRobotDriver->m_vtWeldLineInWorldPoints.push_back(tTempCoors);
		m_pTraceModel->m_vtWeldLinePointType.push_back(E_WELD_TRACK);
		//SaveRobotCoor(m_pRecordTheoryPoint, tTempCoors, E_WELD_TRACK);//保存理论数据
		pRobotDriver->m_cLog->Write("[终点处理]: TrackSmooth_GetEndPoint: %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf",
			tTempCoors.dX, tTempCoors.dY, tTempCoors.dZ, tTempCoors.dRX, tTempCoors.dRY, tTempCoors.dRZ, tTempCoors.dBX, tTempCoors.dBY, tTempCoors.dBZ);
	}
	TrackSmooth_FreePntArray(&tTrackPtnArray);

	//更新结尾数据
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
				m_pRobotDriver->m_cLog->Write("[过程搜端点]: 发现终点 %11.3lf%11.3lf%11.3lf", tNewEndPtn.x, tNewEndPtn.y, tNewEndPtn.z);

				// 更新终点 如果终点处提取端点波动大需要取最近几次终点平均？
				m_pTraceModel->m_tRealEndpointCoor = T_ROBOT_COORS(tNewEndPtn.x, tNewEndPtn.y, tNewEndPtn.z, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
				// 发现终点
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
	bool bDynamicProc = E_POINT_CLOUD_PROC_MOMENT_DYNAMIC == m_ePointCloudProcMoment ? true : false; // 实时处理 ？ 扫描点云完处理？
	long lStartTime = 0;
	int nCapImageNo = 0;
	T_ROBOT_COORS tRobotCoord; //采图时的直角坐标
	T_ANGLE_PULSE tRobotPulse; //采图时的关节坐标
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

	m_pTraceModel->lnImageNo.clear();	// 当前点云集合中每张图编号
	m_pTraceModel->lnImagePtnNum.clear();	// 当前点云集合中每张图点数
	m_pTraceModel->ltPointCloud.clear(); // 当前点云集合
	m_pTraceModel->ltCapWorldCoord.clear(); // 当前点云集合中每张图采集时世界坐标
	m_pTraceModel->vtPointCloudEndPoints.clear(); // 每次点云处理提取的端点世界坐标
	m_pTraceModel->vtConnerWorldCoords.clear(); // 每次图像处理角点世界坐标
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

	// 等待扫描运动
	int nCurWaitTime = 0;
	int nWaitTimeStep = 100;
	while (!m_ptUnit->WorldIsRunning())
	{
		Sleep(nWaitTimeStep);
		nCurWaitTime += nWaitTimeStep;
		if (nCurWaitTime > nTimeOut)
		{
			pRobotDriver->m_cLog->Write("[点云搜端点]: 等待运动超时！");
			return;
		}
	}

	int i = 0;

	// 正在运动中 且 未发现终点
	while (m_ptUnit->WorldIsRunning())
	{
		if (bDynamicProc && true != m_IsOpenJudgeEnd_ProcessSearch) // 实时处理 且 处理关闭  结束
		{
			break;
		}
		lStartTime = XI_clock();

		// 读坐标
		tRobotCoord = pRobotDriver->GetCurrentPos();
		tRobotPulse = pRobotDriver->GetCurrentPulse();
		t.push_back(tRobotCoord);
		long long lTime1 = XI_clock();

		// 采图
		//if (!pDHCamDrv->CaptureImage(pImageBuf, 1))
		if (!pDHCamDrv->CaptureImage(1))
		{
			pRobotDriver->m_cLog->Write("DynamicCaptureNew 采图失败！");
			return;
		}

		long long lTime1_1 = XI_clock();
		pImageBuf = pDHCamDrv->m_tLatestImage.pImage;
		cvCvtColor(pImageBuf, m_pShowImage, CV_GRAY2RGB);

		char szName[56] = { 0 };
		sprintf(szName, ".\\LocalFiles\\OutputFiles\\RobotA\\EndSrc\\%d.bin", i++);//设置保存路径

		long long lTime1_2 = XI_clock();
		//BinaryImgSave(szName, pImageBuf);
		/*cvSaveImage(szName, pImageBuf);*/


		long long lTime2 = XI_clock();

		// 处理
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

		// 转坐标
		T_ROBOT_COORS tPointCloudRobotCoord; //单点转出的机器人坐标
		T_ROBOT_COORS tPointCloudWorldCoord; //单点转出的世界坐标
		Three_DPoint tThreePoint;  //3D坐标?
		int nSaveNum = 0;
		m_MutexTrackPointCloud.lock();
		long long lTime3 = XI_clock();

		std::vector<T_ROBOT_COORS> vtPoint3D = m_ptUnit->TranImageToBase(nCameraNo, vtPoint2D, tRobotCoord, tRobotPulse);

		for (int i = 0; i < vtPoint2D.size(); i++)
		{
			if (vtPoint2D[i].x < nMinX || vtPoint2D[i].x > nMaxX) continue; // 无效区域点云数据不保存
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
			if (0 == i) // 第一个是每张图处理的角点
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
			// 删除旧点云数据
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

		pRobotDriver->m_cLog->Write("[点云搜端点]: 扫描采图处理%d总耗时:%dms 坐标读取:%d 图像采集%d 图像转换%d 图像保存%d 图像处理:%d 坐标转换:%d 点云删除:%d CurXYZ:%.3lf %.3lf %.3lf",
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
	int nErrorTimes = 0;//处理失败的次数
	int nMaxErrorTimes = 10;//处理失败的次数上限
	bool bDynamicProc = E_POINT_CLOUD_PROC_MOMENT_DYNAMIC == m_ePointCloudProcMoment ? true : false; // 实时处理 ？ 扫描点云完处理？
	long lStartTime = 0;
	int nCapImageNo = 0;
	T_ROBOT_COORS tRobotCoord; //采图时的直角坐标
	T_ANGLE_PULSE tRobotPulse; //采图时的关节坐标
//	T_ABS_POS_IN_BASE tPointAbsCoordInBase;

	CDHGigeImageCapture* pDHCamDrv = (CDHGigeImageCapture*)m_ptUnit->GetCameraCtrl(nCameraNo);
	IplImage* pImageBuf = NULL;
	T_CAMREA_PARAM tCameraParam = m_ptUnit->GetCameraParam(nCameraNo);
	int nMinX = 0;
	int nMaxX = tCameraParam.tDHCameraDriverPara.nRoiWidth; //*2 / 3;
	std::vector<CvPoint> vtPoint2D(0);
	FILE* pfPointCloud = fopen(sPointCloudFileName.GetBuffer(), "w");
	m_sPointCloudFileName = sPointCloudFileName;

	m_pTraceModel->lnImageNo.clear();	// 当前点云集合中每张图编号
	m_pTraceModel->lnImagePtnNum.clear();	// 当前点云集合中每张图点数
	m_pTraceModel->ltPointCloud.clear(); // 当前点云集合
	m_pTraceModel->ltCapWorldCoord.clear(); // 当前点云集合中每张图采集时世界坐标
	m_pTraceModel->vtPointCloudEndPoints.clear(); // 每次点云处理提取的端点世界坐标
	m_pTraceModel->vtConnerWorldCoords.clear(); // 每次图像处理角点世界坐标
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

	// 等待扫描运动
	int nCurWaitTime = 0;
	int nWaitTimeStep = 100;
	while (!m_ptUnit->WorldIsRunning())
	{
		Sleep(nWaitTimeStep);
		nCurWaitTime += nWaitTimeStep;
		if (nCurWaitTime > nTimeOut)
		{
			pRobotDriver->m_cLog->Write("[点云搜端点]: 等待运动超时！");
			return;
		}
	}

	int i = 0;
	vector<T_ANGLE_PULSE> vt;
	// 正在运动中 且 未发现终点
	while (m_ptUnit->WorldIsRunning())
	{
		if (bDynamicProc && false == m_IsOpenJudgeEnd_ProcessSearch) // 实时处理 且 处理关闭  结束
		{
			break;
		}
		lStartTime = XI_clock();

		// 读坐标
		tRobotCoord = pRobotDriver->GetCurrentPos_ESTUN();
		if (tRobotCoord.dX == 0 && tRobotCoord.dY == 0 && tRobotCoord.dZ == 0 && tRobotCoord.dRY == 0)
		{
			tRobotCoord = pRobotDriver->GetCurrentPos();
		}
		pRobotDriver->RobotInverseKinematics(tRobotCoord, pRobotDriver->m_tTools.tGunTool, vt);
		tRobotPulse = vt[0]/*pRobotDriver->GetCurrentPulse_ESTUN()*/;
		long long lTime1 = XI_clock();

		// 采图
		if (!pDHCamDrv->CaptureImage(pImageBuf,1)) 
		{ 
			pRobotDriver->m_cLog->Write("DynamicCaptureNew 采图失败！");
			return;
		}
		cvCvtColor(pImageBuf, m_pShowImage, CV_GRAY2RGB);

		char szName[256] = { 0 };
		//sprintf(szName, ".\\LocalFiles\\OutputFiles\\RobotA\\EndSrc\\%d.bin", i++);//设置保存路径
		//BinaryImgSaveAsynchronous(szName, pImageBuf, 0);
		
		//sprintf(szName, ".\\LocalFiles\\OutputFiles\\RobotA\\EndSrc\\%d.jpg", i++);//设置保存路径
		//cvSaveImage(szName, pImageBuf);

		long long lTime2 = XI_clock();

		// 处理
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
				m_pRobotDriver->m_cLog->Write("失败次数%d", nErrorTimes);
				if (nErrorTimes > nMaxErrorTimes)
				{
					XiMessageBox("测量搜端点失败");
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

		// 转坐标
		T_ROBOT_COORS tPointCloudRobotCoord; //单点转出的机器人坐标
		T_ROBOT_COORS tPointCloudWorldCoord; //单点转出的世界坐标
		Three_DPoint tThreePoint;  //3D坐标?
		int nSaveNum = 0;
		m_MutexTrackPointCloud.lock();
		long long lTime3 = XI_clock();

		std::vector<T_ROBOT_COORS> vtPoint3D = m_ptUnit->TranImageToBase(nCameraNo, vtPoint2D, tRobotCoord, tRobotPulse);

		for (int i = 0; i < vtPoint2D.size(); i++)
		{
			if (vtPoint2D[i].x < nMinX || vtPoint2D[i].x > nMaxX) continue; // 无效区域点云数据不保存
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
			if (0 == i) // 第一个是每张图处理的角点
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
			// 删除旧点云数据
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

		pRobotDriver->m_cLog->Write("[点云搜端点]: 扫描采图处理%d总耗时:%dms 坐标读取:%d 图像采集%d 图像处理:%d 坐标转换:%d 点云删除:%d CurXYZ:%.3lf %.3lf %.3lf",
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
	//FlipImage(pImg, eFlipMode); // 翻转
	bool bProcRst = WeightedPointCluster(pImg, &tBackLine, &tFrontLine, &Corner2D, true, "WeightedPointCluster");//跟踪
	//WidgetLaserTrack();
	if(bProcRst)
		vtPoint2D.push_back(Corner2D);
	int nLength = FindLaserMidPiontEEEEEInTrack_Fix(pImg, tFrontLine, tBackLine, tPoints, "FindLaserMidPiontEEEEEInTrack_Fix");//只能跟踪用，提区域+提中心点
	//int nLength = FindLaserMidPiontEEEEE_(pImg, tPoints, 3, 60, 15, 5, 10, 0.5, 20000, 3, 25, false);
	//WidgetLaserPntExt();
	//FlipImage(pImg, eFlipMode); // 翻转恢复
	for (int i = 0; i < nLength; i++)
	{
		//ResumeImagePoint2D(pImg->width, pImg->height, tPoints[i], eFlipMode); // 二维点恢复
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
	//FlipImage(pImg, eFlipMode); // 翻转
	//bool bProcRst = WeightedPointCluster(pImg, &tFrontLine, &tBackLine, &Corner2D, false, "WeightedPointCluster");//跟踪
	vtPoint2D.push_back(Corner2D);
	//int nLength = FindLaserMidPiontEEEEEInTrack/*_Fix*/(pImg, tFrontLine, tBackLine, tPoints, 50, 0, 3, 90, 15, 60, 10,0.5,10000,3,11/*, "FindLaserMidPiontEEEEEInTrack_Fix"*/);//只能跟踪用，提区域+提中心点
	//int nLength = FindLaserMidPiontEEEEE_(pImg, tPoints, 3, 60, 15, 5, 10, 0.5, 20000, 3, 25, false);
	// 
	int nLength = LaserPntExtByWinCenterIter(pImg, tPoints, "LaserPntExtByWinCenterIter");
	// 
	//FlipImage(pImg, eFlipMode); // 翻转恢复
	for (int i = 0; i <nLength; i++)
	{
		//ResumeImagePoint2D(pImg->width, pImg->height, tPoints[i], eFlipMode); // 二维点恢复
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

	// 采图
	if (!pDHCamDrv->CaptureImage(m_pTraceLockImg, 3))
	{
		pRobotDriver->m_cLog->Write("lockLaser 采图失败！");
		return false;
	}

	//考虑到和后面扫描保持一致，这里也不会翻转图像
	//如果要翻转图像，后面扫描也要翻转
	//FlipImage(m_pTraceLockImg, m_ptUnit->GetCameraParam(nCameraNo).eFlipMode);

	//保存锁定图
	SaveImage(m_pTraceLockImg, OUTPUT_PATH + m_ptUnit->GetUnitName() + SEARCH_SCANLOCK_IMG);

	//锁定
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
		XiMessageBoxOk("搜索激光斜率锁定失败！(错误代码：找不到关键点)");
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

	//显示结果
	cvCvtColor(m_pTraceLockImg, m_pShowImage, CV_GRAY2RGB);
	cvLine(m_pShowImage, cpKeyPoint, LeftPtn, CV_RGB(0, 255, 0), 2);
	cvLine(m_pShowImage, cpKeyPoint, RightPtn, CV_RGB(0, 0, 255), 2);
	cvCircle(m_pShowImage, cpKeyPoint, 10, CV_RGB(255, 0, 0), 3);

	if (m_pTraceLockImg != NULL) {
		//该单元存在照片，先存照片，再释放内存
		cvReleaseImage(&m_pTraceLockImg);
	}
	return true;
}

int CScanInitModule::get2DPnts(std::vector<CvPoint>& vtPoint2D, IplImage* pImg, CString sWidgetLaserTrackParaFile, CString sWidgetLaserPntExtParaFile)
{
	vtPoint2D.clear();

	if (!WidgetLaserTrack(pImg, &m_tSearchLockLaserInfo, sWidgetLaserTrackParaFile.GetBuffer()))
	{
		m_pRobotDriver->m_cLog->Write("WidgetLaserTrack处理失败");
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
		m_pRobotDriver->m_cLog->Write("WidgetLaserPntExt处理失败");
		return false;
	}

	return true;
}

// 开启图片处理线程 开启点云处理线程 运动 创建并保存采集图像数据
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

	m_pTraceModel->lnImageNo.clear();	// 当前点云集合中每张图编号
	m_pTraceModel->lnImagePtnNum.clear();	// 当前点云集合中每张图点数
	m_pTraceModel->ltPointCloud.clear(); // 当前点云集合
	m_pTraceModel->ltCapWorldCoord.clear(); // 当前点云集合中每张图采集时世界坐标
	m_pTraceModel->vtPointCloudEndPoints.clear(); // 每次点云处理提取的端点世界坐标
	m_pTraceModel->m_bCameraFindWeldEnd = false;
	m_pTraceModel->pfPointCloud = fopen(OUTPUT_PATH + m_pRobotDriver->m_strRobotName + WELDDATA_PATH + "AA_PointCloud.txt", "w");
	m_pTraceModel->pfImgCalcData = fopen(OUTPUT_PATH + m_pRobotDriver->m_strRobotName + WELDDATA_PATH + "AA_CapImgCoord.txt", "w");

	m_pCapImageData = new T_CAP_IMAGE_DATA(nMaxImgBufSize); // 创建数据缓冲区

	m_IsOpenImageProcess = true;
	for (int i = 0; i < nProcThreadNum; i++) // 开启图片处理线程
	{
		CWinThread* tProcImgThread = AfxBeginThread(ThreadImageProcess, this);
		tProcImgThread->m_bAutoDelete = false;
		vpWinThread.push_back(tProcImgThread);

	}
	CWinThread* tProcPtnCldThread = AfxBeginThread(ThreadJudgeEnd_ProcessSearch, this); // 开启点云处理线程
	tProcPtnCldThread->m_bAutoDelete = false;
	vpWinThread.push_back(tProcPtnCldThread);

	pRobotDriver->CallJob(m_sMoveJobName); // 开始运动

	int nCurWaitTime = 0;
	int nWaitTimeStep = 100;
	bool bIsRunning = m_ptUnit->WorldIsRunning();
	while (!m_ptUnit->WorldIsRunning()) // 等待运动
	{
		Sleep(nWaitTimeStep);
		nCurWaitTime += nWaitTimeStep;
		if (nCurWaitTime > nTimeOut)
		{
			DELETE_POINTER(m_pCapImageData);
			pRobotDriver->m_cLog->Write("[点云搜端点]: 等待运动超时！");
			return;
		}
	}

	// 正在运动中 且 未发现终点

	while (m_ptUnit->WorldIsRunning() && true == m_IsOpenJudgeEnd_ProcessSearch)
	{
		lStartTime = GetTickCount();
		if (labs(lStartTime - lPreStartTime) < nMinCapTimeInterval)
		{
			Sleep(nMinCapTimeInterval - labs(lStartTime - lPreStartTime));
		}
		lStartTime = GetTickCount();
		lPreStartTime = lStartTime;

		// 读坐标
		tRobotCoord = pRobotDriver->GetCurrentPos();
		tRobotPulse = pRobotDriver->GetCurrentPulse();
		// 采图
		pDHCamDrv->CaptureImage(pImageBuf, 1);

		// 保存到缓冲区
		nSaveBufNo = m_pCapImageData->nTotalCapImgNum % m_pCapImageData->nMaxBufferSize;
		m_pCapImageData->vtCapCoord[nSaveBufNo] = tRobotCoord;
		m_pCapImageData->vtCapPulse[nSaveBufNo] = tRobotPulse;
		if (NULL == m_pCapImageData->vpImg[nSaveBufNo])
		{
			m_pCapImageData->vpImg[nSaveBufNo] = cvCreateImage(cvSize(tCameraParam.tDHCameraDriverPara.nRoiWidth, tCameraParam.tDHCameraDriverPara.nRoiHeight), IPL_DEPTH_8U, 1);
		}
		cvCopyImage(pImageBuf, m_pCapImageData->vpImg[nSaveBufNo]);
		m_pCapImageData->nTotalCapImgNum = nCapImageNo + 1;

		pRobotDriver->m_cLog->Write("[点云搜端点]: 扫描采图总编号%d 缓存号%d 耗时%dms 采图成功%d 图片状态%d",
			nCapImageNo, nSaveBufNo, GetTickCount() - lStartTime, bCapSuc, nImgStatus);
		nCapImageNo++;

	}
	m_IsOpenImageProcess = false;

	m_pRobotDriver->HoldOn();
	Sleep(500);
	m_pRobotDriver->HoldOff();

	WaitAndCheckAllThreadExit(vpWinThread);

	// 释放 m_pCapImageData
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
	m_pRobotDriver->m_cLog->Write("[点云搜端点]: FuncImageProcess开始!");
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
		// 拷贝处理图片和采图坐标数据
		lTime1 = GetTickCount();
		while (m_IsOpenImageProcess && false == m_pCapImageData->m_MutexReadWrite.trylock()) Sleep(10);
		if (!m_IsOpenImageProcess) break;
		nTotalImageNo = m_pCapImageData->nTotalCapImgNum;
		nProcImageNo = m_pCapImageData->nTotalProImgNum;
		nMaxBufSize = m_pCapImageData->nMaxBufferSize;
		nBufNo = nProcImageNo % m_pCapImageData->nMaxBufferSize;
		if ((nTotalImageNo > 0) && (nProcImageNo < nTotalImageNo)) // 有未处理的图 取数据处理
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
		// 处理速度过慢？
		if ((nTotalImageNo - nProcImageNo) >= nMaxBufSize)
		{
			m_IsOpenImageProcess = false;
			CheckMachineEmg(pRobotCtrl);
			XUI::MesBox::PopOkCancel("[点云搜端点]: {0} 图像处理过慢! Cap{1} Proc{2} MaxBuf{3}", m_pRobotDriver->m_strRobotName, nTotalImageNo, nProcImageNo, nMaxBufSize);
			continue;
		}
		lTime2 = GetTickCount();

		// 存图 处理 计算
		if (bSaveImg && (0 == nProcImageNo % nSaveImgStepNo))
		{
			SaveImage(&pImg, "%s%s\\Track\\%d.jpg", OUTPUT_PATH, m_pRobotDriver->m_strRobotName, nProcImageNo);
		}
		// 不起弧时的端点扫描可以不调用前两个接口 起弧跟踪焊接是必须调用全部三个接口
		long lTime21 = GetTickCount();

		bool bProcRst = WeightedPointCluster(pImg, &tFrontLine, &tBackLine, &Corner2D, false, /*sVisionIniName.GetBuffer()*/"WeightedPointCluster");//跟踪
		long lTime22 = GetTickCount();

		long lTime23 = GetTickCount();
		int nLength = FindLaserMidPiontEEEEEInTrack_Fix(pImg, tFrontLine, tBackLine, tPoints, "FindLaserMidPiontEEEEEInTrack_Fix");//只能跟踪用，提区域+提中心点
		long lTime24 = GetTickCount();

		int nSaveNum = 0;
		int nMinX = pImg->width / 4;
		int nMaxX = pImg->width * 3 / 4;
		ltThreePoint.clear();

		// 角点坐标
		tConnerWorldCoord = m_ptUnit->TranImageToBase(nCameraNo, Corner2D, tCapRobot, tCapPulse, &tPointAbsCoordInBase);
		tConnerWorldCoord.dX += tCapRobot.dBX;
		tConnerWorldCoord.dY += tCapRobot.dBY;
		tConnerWorldCoord.dZ += tCapRobot.dBZ;
		fprintf(m_pTraceModel->pfPointCloud, "图号:%d 点号%d %11.3lf%11.3lf%11.3lf\n", nProcImageNo, -1, tConnerWorldCoord.dX, tConnerWorldCoord.dY, tConnerWorldCoord.dZ); // -1用于区分角点和点云坐标
																																						// 点云坐标
		for (int i = 0; i < nLength; i += nPtn2DStepDis)
		{
			if (tPoints[i].x < nMinX || tPoints[i].x > nMaxX) continue; // 无效区域点云数据不保存
			tPointCloudRobotCoord = m_ptUnit->TranImageToBase(nCameraNo, tPoints[i], tCapRobot, tCapPulse, &tPointAbsCoordInBase);
			tThreePoint.x = tPointCloudRobotCoord.dX + tCapRobot.dBX;
			tThreePoint.y = tPointCloudRobotCoord.dY + tCapRobot.dBY;
			tThreePoint.z = tPointCloudRobotCoord.dZ + tCapRobot.dBZ;
			ltThreePoint.push_back(tThreePoint);
			nSaveNum++;
			fprintf(m_pTraceModel->pfPointCloud, "图号:%d 点号%d %11.3lf%11.3lf%11.3lf\n", nProcImageNo, i, tThreePoint.x, tThreePoint.y, tThreePoint.z);
			fflush(m_pTraceModel->pfPointCloud);
		}
		lTime3 = GetTickCount();

		// 加锁保存点云数据并删除多余数据
		m_MutexTrackPointCloud.lock();
		m_pTraceModel->ltPointCloud.insert(m_pTraceModel->ltPointCloud.end(), ltThreePoint.begin(), ltThreePoint.end());
		m_pTraceModel->lnImageNo.push_back(nProcImageNo);
		m_pTraceModel->lnImagePtnNum.push_back(nSaveNum);
		m_pTraceModel->ltCapWorldCoord.push_back(tCapRobot);
		m_pTraceModel->vtConnerWorldCoords.push_back(tConnerWorldCoord);
		for (int i = 0; (i < m_pTraceModel->ltCapWorldCoord.size()) && (m_pTraceModel->ltCapWorldCoord.size() > 40); i++) // 删除旧点云数据
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
		if (0 == nProcImageNo % nSaveImgStepNo) // 显示处理结果 如果最多一个线程显示可以移动到锁外面
		{
			DrawAndShowImg(&pImg, &m_pShowImage, Corner2D, tFrontLine, tBackLine);
		}
		fprintf(m_pTraceModel->pfImgCalcData, "图号:%d 直角：%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf 关节:%10d%10d%10d%10d%10d%10d%10d%10d%10d 二维点:%10d%10d 三维点:%11.3lf%11.3lf%11.3lf\n",
			nProcImageNo, tCapRobot.dX, tCapRobot.dY, tCapRobot.dZ, tCapRobot.dRX, tCapRobot.dRY, tCapRobot.dRZ, tCapRobot.dBX, tCapRobot.dBY, tCapRobot.dBZ,
			tCapPulse.nSPulse, tCapPulse.nLPulse, tCapPulse.nUPulse, tCapPulse.nRPulse, tCapPulse.nBPulse, tCapPulse.nTPulse, tCapPulse.lBXPulse, tCapPulse.lBYPulse, tCapPulse.lBZPulse,
			Corner2D.x, Corner2D.y, tConnerWorldCoord.dX, tConnerWorldCoord.dY, tConnerWorldCoord.dZ);
		m_MutexTrackPointCloud.unlock();
		lTime5 = GetTickCount();

		m_pRobotDriver->m_cLog->Write("[点云搜端点]: 扫描处理总编号%d总耗时:%d 数据准备:%d 保存图片%d 跟踪处理:%d 飞溅处理%d 点云处理%d 坐标转换%d 更新点云%d 显示结果%d",
			nProcImageNo, lTime5 - lTime1, lTime2 - lTime1, lTime21 - lTime2, lTime22 - lTime21, lTime23 - lTime22, lTime24 - lTime23, lTime3 - lTime24, lTime4 - lTime3, lTime5 - lTime4);
	}
	if (NULL != pImg) cvReleaseImage(&pImg);
	delete[] tPoints;
	m_pRobotDriver->m_cLog->Write("[点云搜端点]: FuncImageProcess结束!");
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

	//跟踪图像处理方式 初始化时锁定 m_pTraceModel->tFrontLine  m_pTraceModel->tBackLine
	if (E_IMAGE_PROC_METHOD_EEEEEInTrack == m_eImgProcMethod)
	{
		// 锁定
		CDHGigeImageCapture* pDHCamDrv = (CDHGigeImageCapture*)m_ptUnit->GetCameraCtrl(m_nCameraNo);
		IplImage* pImageBuf = NULL;
		T_CAMREA_PARAM tCameraParam = m_ptUnit->GetCameraParam(m_nCameraNo);
//		CvPoint Corner2D;
		std::vector<CvPoint> vtLeftCorner2D;
		std::vector<CvPoint> vtRightCorner2D;
		if (!pDHCamDrv->CaptureImage(pImageBuf, 1))
		{
			XiMessageBox("InitDynamicCapture_H_M: 采集图片失败");
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
			XiMessageBox("InitDynamicCapture_H_M: 锁定失败");
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
	m_pTraceModel->lnImageNo.clear();	// 当前点云集合中每张图编号
	m_pTraceModel->lnImagePtnNum.clear();	// 当前点云集合中每张图点数
	m_pTraceModel->ltPointCloud.clear(); // 当前点云集合
	m_pTraceModel->ltCapWorldCoord.clear(); // 当前点云集合中每张图采集时世界坐标
	m_pTraceModel->vtPointCloudEndPoints.clear(); // 每次点云处理提取的端点世界坐标
	m_pTraceModel->m_bCameraFindWeldEnd = false;
	m_pTraceModel->pfPointCloud = fopen(m_sSaveFilePath + m_sGroovePointCloudFileName/*"AA_PointCloud.txt"*/, "w");
	m_pTraceModel->pfImgCalcData = fopen(m_sSaveFilePath + "AA_CapImgCoord.txt", "w");
	m_pCapImageData_H_M = new T_CAP_IMAGE_DATA(nMaxImgBufSize); // 创建数据缓冲区

	m_IsOpenImageProcess_H_M = true;
	for (int i = 0; i < nProcThreadNum; i++) // 开启图片处理线程
	{
		CWinThread* tProcImgThread = AfxBeginThread(ThreadImageProcess_H_M, this);
		Sleep(50);
		tProcImgThread->m_bAutoDelete = false;
		vpWinThread.push_back(tProcImgThread);
	}
	if (m_ePointCloudProcMoment == E_POINT_CLOUD_PROC_MOMENT_DYNAMIC)
	{
		CWinThread* tProcPtnCldThread = AfxBeginThread(ThreadPointCloudProcess_H_M, this); // 开启点云处理线程
		tProcPtnCldThread->m_bAutoDelete = false;
		vpWinThread.push_back(tProcPtnCldThread);
	}

	// 此处运动改为联动 m_pRobotDriver->m_vtWeldLineInWorldPoints;
	if (54 == m_pRobotDriver->m_nExternalAxleType)
	{
		pRobotDriver->CallJob(m_sMoveJobName); // 开始运动
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
	while (!m_ptUnit->WorldIsRunning()) // 等待运动
	{
		Sleep(nWaitTimeStep);
		nCurWaitTime += nWaitTimeStep;
		if (nCurWaitTime > nTimeOut)
		{
			DELETE_POINTER(m_pCapImageData_H_M);
			pRobotDriver->m_cLog->Write("[点云搜端点]: 等待运动超时！");
			return;
		}
	}

	// 正在运动中 且 未发现终点
	while (m_ptUnit->WorldIsRunning() && (true == m_IsOpenPointCloudProcess_H_M || E_POINT_CLOUD_PROC_MOMENT_FINAL != m_ePointCloudProcMoment))
	{
		lStartTime = GetTickCount();
		if (labs(lStartTime - lPreStartTime) < nMinCapTimeInterval)
		{
			Sleep(nMinCapTimeInterval - labs(lStartTime - lPreStartTime));
		}
		lStartTime = GetTickCount();
		lPreStartTime = lStartTime;

		// 读坐标
		tRobotCoord = pRobotDriver->GetCurrentPos();
		tRobotPulse = pRobotDriver->GetCurrentPulse();
		long lTime1 = GetTickCount();
		// 采图
		pDHCamDrv->CaptureImage(pImageBuf, 1);

		//GrooveEndPntsInfo Info;
		//int nRst = FindGrooveEndPnts(pImageBuf, 1, &Info, 190, 1, 1, 0, "Local_Files\\ExtLib\\Vision\\ConfigFiles\\GroovePnts.ini");

		// 保存到缓冲区
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

		pRobotDriver->m_cLog->Write("[点云搜端点]: 扫描采图总编号%d 缓存号%d 耗时%dms %dms %dms 采图成功%d 图片状态%d %d",
			nCapImageNo, nSaveBufNo, GetTickCount() - lTime2, lTime2 - lTime1, lTime1 - lStartTime, bCapSuc, nImgStatus, 0/*Info.BPntsNum*/);
		nCapImageNo++;

	}
	m_IsOpenImageProcess_H_M = false;

	m_pRobotDriver->HoldOn();
	Sleep(500);
	m_pRobotDriver->HoldOff();

	WaitAndCheckAllThreadExit(vpWinThread);

	// 释放 m_pCapImageData
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
	default: XUI::MesBox::PopOkCancel("H_M:图像处理方法参数错误：{0}", (int)m_eImgProcMethod); break;
	}
	return bRst;
}

bool CScanInitModule::FuncImageProcess_H_M_EEEEE(CRobotDriverAdaptor* pRobotCtrl)
{
	auto dPointCloudLengthForFindEndpnt = PARA_FLAT_MEASURE(dPointCloudLengthForFindEndpnt);
	m_pRobotDriver->m_cLog->Write("[点云搜端点]: FuncImageProcess开始!");
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
		// 拷贝处理图片和采图坐标数据
		lTime1 = GetTickCount();
		while (m_IsOpenImageProcess_H_M && false == m_pCapImageData_H_M->m_MutexReadWrite.trylock()) Sleep(10);
		if (!m_IsOpenImageProcess_H_M) break;
		nTotalImageNo = m_pCapImageData_H_M->nTotalCapImgNum;
		nProcImageNo = m_pCapImageData_H_M->nTotalProImgNum;
		nMaxBufSize = m_pCapImageData_H_M->nMaxBufferSize;
		nBufNo = nProcImageNo % m_pCapImageData_H_M->nMaxBufferSize;
		if ((nTotalImageNo > 0) && (nProcImageNo < nTotalImageNo)) // 有未处理的图 取数据处理
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
		// 处理速度过慢？
		if ((nTotalImageNo - nProcImageNo) >= nMaxBufSize)
		{
			m_IsOpenImageProcess_H_M = false;
			CheckMachineEmg(pRobotCtrl);
			XUI::MesBox::PopOkCancel("[点云搜端点]: {0} 图像处理过慢! Cap{1} Proc{2} MaxBuf{3}", m_pRobotDriver->m_strRobotName, nTotalImageNo, nProcImageNo, nMaxBufSize);
			continue;
		}
		lTime2 = GetTickCount();

		// 存图 处理 计算
		if (m_bSaveImg && (0 == nProcImageNo % m_nSaveImgStepNo))
		{
			SaveImage(&pImg, "%s%s\\Track\\%d.jpg", OUTPUT_PATH, m_pRobotDriver->m_strRobotName, nProcImageNo);
		}

		if (NULL == pTempImageBuff)
		{
			pTempImageBuff = cvCreateImage(cvSize(pImg->width, pImg->height), IPL_DEPTH_8U, 1);
			cvSet(pTempImageBuff, CV_RGB(0, 0, 0));
		}
		// 不起弧时的端点扫描可以不调用前两个接口 起弧跟踪焊接是必须调用全部三个接口
		long lTime21 = GetTickCount();
		//bool bProcRst = WeightedPointCluster(pImg, &tFrontLine, &tBackLine, &Corner2D, false, "WeightedPointCluster");//跟踪
		long lTime22 = GetTickCount();
		int nLength = FindLaserMidPiontEEEEE(pImg, tPoints, 3, 30, 15, 30, 10, 0.5, 10000, 3, 15);
		//int nLength = FindLaserMidPiontEEEEEInTrack_(pImg, m_pTraceModel->tFrontLine, Corner2D, tPoints, 50, 0, 3, 90, 11, 90, 10, 0.5, 10000, 3, 11, 0);
		//nLength += FindLaserMidPiontEEEEEInTrack_(pImg, m_pTraceModel->tBackLine, Corner2D, tPoints + nLength, 50, 0, 3, 90, 11, 90, 10, 0.5, 10000 - nLength, 3, 11, 0);
		long lTime23 = GetTickCount();

		int nSaveNum = 0;
		int nMinX = pImg->width / 4;
		int nMaxX = pImg->width * 3 / 4;
		ltThreePoint.clear();

		//// 角点坐标
		//tConnerWorldCoord = m_ptUnit->TranImageToBase(m_nCameraNo, Corner2D, tCapRobot, tCapPulse, &tPointAbsCoordInBase);
		//tConnerWorldCoord.dX += tCapRobot.dBX;
		//tConnerWorldCoord.dY += tCapRobot.dBY;
		//tConnerWorldCoord.dZ += tCapRobot.dBZ;
		//fprintf(m_pTraceModel->pfPointCloud, "图号:%d 点号%d %11.3lf%11.3lf%11.3lf\n", nProcImageNo, -1, tConnerWorldCoord.dX, tConnerWorldCoord.dY, tConnerWorldCoord.dZ); // -1用于区分角点和点云坐标
		// 点云坐标
		for (int i = 0; i < nLength; i += m_nPtn2DStepDis)
		{
			if (tPoints[i].x < nMinX || tPoints[i].x > nMaxX) continue; // 无效区域点云数据不保存
			tPointCloudRobotCoord = m_ptUnit->TranImageToBase(m_nCameraNo, tPoints[i], tCapRobot, tCapPulse, &tPointAbsCoordInBase);
			tThreePoint.x = tPointCloudRobotCoord.dX + tCapRobot.dBX;
			tThreePoint.y = tPointCloudRobotCoord.dY + tCapRobot.dBY;
			tThreePoint.z = tPointCloudRobotCoord.dZ + tCapRobot.dBZ;
			ltThreePoint.push_back(tThreePoint);
			nSaveNum++;
			//fprintf(m_pTraceModel->pfPointCloud, "图号:%d 点号%d %11.3lf%11.3lf%11.3lf\n", nProcImageNo, i, tThreePoint.x, tThreePoint.y, tThreePoint.z);
			fprintf(m_pTraceModel->pfPointCloud, "%d%11.3lf%11.3lf%11.3lf\n", nProcImageNo, tThreePoint.x, tThreePoint.y, tThreePoint.z);
			fflush(m_pTraceModel->pfPointCloud);
		}
		lTime3 = GetTickCount();

		// 加锁保存点云数据并删除多余数据
		m_Mutex_H_M.lock();
		m_pTraceModel->ltPointCloud.insert(m_pTraceModel->ltPointCloud.end(), ltThreePoint.begin(), ltThreePoint.end());
		m_pTraceModel->lnImageNo.push_back(nProcImageNo);
		m_pTraceModel->lnImagePtnNum.push_back(nSaveNum);
		m_pTraceModel->ltCapWorldCoord.push_back(tCapRobot);
		m_pTraceModel->vtConnerWorldCoords.push_back(tConnerWorldCoord);
		for (int i = 0; i < m_pTraceModel->ltCapWorldCoord.size(); i++) // 删除旧点云数据
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
		if (0 == nProcImageNo % m_nSaveImgStepNo) // 显示处理结果 如果最多一个线程显示可以移动到锁外面
		{
			DrawAndShowImg(&pImg, &m_pShowImage, Corner2D, tFrontLine, tBackLine);
		}
		fprintf(m_pTraceModel->pfImgCalcData, "图号:%d 直角：%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf 关节:%10d%10d%10d%10d%10d%10d%10d%10d%10d 二维点:%10d%10d 三维点:%11.3lf%11.3lf%11.3lf\n",
			nProcImageNo, tCapRobot.dX, tCapRobot.dY, tCapRobot.dZ, tCapRobot.dRX, tCapRobot.dRY, tCapRobot.dRZ, tCapRobot.dBX, tCapRobot.dBY, tCapRobot.dBZ,
			tCapPulse.nSPulse, tCapPulse.nLPulse, tCapPulse.nUPulse, tCapPulse.nRPulse, tCapPulse.nBPulse, tCapPulse.nTPulse, tCapPulse.lBXPulse, tCapPulse.lBYPulse, tCapPulse.lBZPulse,
			Corner2D.x, Corner2D.y, tConnerWorldCoord.dX, tConnerWorldCoord.dY, tConnerWorldCoord.dZ);
		m_Mutex_H_M.unlock();
		lTime5 = GetTickCount();

		m_pRobotDriver->m_cLog->Write("[点云搜端点]: 扫描处理总编号%d总耗时:%d 数据准备:%d 保存图片%d 跟踪处理:%d 飞溅处理%d 坐标转换%d 更新点云%d 显示结果%d",
			nProcImageNo, lTime5 - lTime1, lTime2 - lTime1, lTime21 - lTime2, lTime22 - lTime21, lTime23 - lTime22, lTime3 - lTime23, lTime4 - lTime3, lTime5 - lTime4);
	}
	if (NULL != pImg) cvReleaseImage(&pImg);
	if (NULL != pTempImageBuff) cvReleaseImage(&pTempImageBuff);
	delete[] tPoints;
	m_pRobotDriver->m_cLog->Write("[点云搜端点]: FuncImageProcess结束!");
	return true;
}

bool CScanInitModule::FuncImageProcess_H_M_EEEEEInTrack(CRobotDriverAdaptor* pRobotCtrl)
{
	auto dPointCloudLengthForFindEndpnt = PARA_FLAT_MEASURE(dPointCloudLengthForFindEndpnt);
	m_pRobotDriver->m_cLog->Write("[点云搜端点]: FuncImageProcess开始!");
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
		// 拷贝处理图片和采图坐标数据
		lTime1 = GetTickCount();
		while (m_IsOpenImageProcess_H_M && false == m_pCapImageData_H_M->m_MutexReadWrite.trylock()) Sleep(10);
		if (!m_IsOpenImageProcess_H_M) break;
		nTotalImageNo = m_pCapImageData_H_M->nTotalCapImgNum;
		nProcImageNo = m_pCapImageData_H_M->nTotalProImgNum;
		nMaxBufSize = m_pCapImageData_H_M->nMaxBufferSize;
		nBufNo = nProcImageNo % m_pCapImageData_H_M->nMaxBufferSize;
		if ((nTotalImageNo > 0) && (nProcImageNo < nTotalImageNo)) // 有未处理的图 取数据处理
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
		// 处理速度过慢？
		if ((nTotalImageNo - nProcImageNo) >= nMaxBufSize)
		{
			m_IsOpenImageProcess_H_M = false;
			CheckMachineEmg(pRobotCtrl);
			XUI::MesBox::PopOkCancel("[点云搜端点]: {0} 图像处理过慢! Cap{1} Proc{2} MaxBuf{3}", m_pRobotDriver->m_strRobotName, nTotalImageNo, nProcImageNo, nMaxBufSize);
			continue;
		}
		lTime2 = GetTickCount();

		// 存图 处理 计算
		if (m_bSaveImg && (0 == nProcImageNo % m_nSaveImgStepNo))
		{
			SaveImage(&pImg, "%s%s\\Track\\%d.jpg", OUTPUT_PATH, m_pRobotDriver->m_strRobotName, nProcImageNo);
		}

		if (NULL == pTempImageBuff)
		{
			pTempImageBuff = cvCreateImage(cvSize(pImg->width, pImg->height), IPL_DEPTH_8U, 1);
			cvSet(pTempImageBuff, CV_RGB(0, 0, 0));
		}
		// 不起弧时的端点扫描可以不调用前两个接口 起弧跟踪焊接是必须调用全部三个接口
		long lTime21 = GetTickCount();
		bool bProcRst = WeightedPointCluster(pImg, &tFrontLine, &tBackLine, &Corner2D, false, "WeightedPointCluster");//跟踪
		long lTime22 = GetTickCount();
		long lTime23 = GetTickCount();
		int nLength = FindLaserMidPiontEEEEEInTrack_Fix(pImg, tFrontLine, tBackLine, tPoints, "FindLaserMidPiontEEEEEInTrack_Fix");//只能跟踪用，提区域+提中心点
		long lTime24 = GetTickCount();

		int nSaveNum = 0;
		int nMinX = pImg->width / 4;
		int nMaxX = pImg->width * 3 / 4;
		ltThreePoint.clear();

		// 角点坐标
		tConnerWorldCoord = m_ptUnit->TranImageToBase(m_nCameraNo, Corner2D, tCapRobot, tCapPulse, &tPointAbsCoordInBase);
		tConnerWorldCoord.dX += tCapRobot.dBX;
		tConnerWorldCoord.dY += tCapRobot.dBY;
		tConnerWorldCoord.dZ += tCapRobot.dBZ;
		fprintf(m_pTraceModel->pfPointCloud, "图号:%d 点号%d %11.3lf%11.3lf%11.3lf\n", nProcImageNo, -1, tConnerWorldCoord.dX, tConnerWorldCoord.dY, tConnerWorldCoord.dZ); // -1用于区分角点和点云坐标
		// 点云坐标
		for (int i = 0; i < nLength; i += m_nPtn2DStepDis)
		{
			if (tPoints[i].x < nMinX || tPoints[i].x > nMaxX) continue; // 无效区域点云数据不保存
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

		// 加锁保存点云数据并删除多余数据
		m_Mutex_H_M.lock();
		m_pTraceModel->ltPointCloud.insert(m_pTraceModel->ltPointCloud.end(), ltThreePoint.begin(), ltThreePoint.end());
		m_pTraceModel->lnImageNo.push_back(nProcImageNo);
		m_pTraceModel->lnImagePtnNum.push_back(nSaveNum);
		m_pTraceModel->ltCapWorldCoord.push_back(tCapRobot);
		m_pTraceModel->vtConnerWorldCoords.push_back(tConnerWorldCoord);
		for (int i = 0; (i < m_pTraceModel->ltCapWorldCoord.size()) && (m_pTraceModel->ltCapWorldCoord.size() > 40); i++) // 删除旧点云数据
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
		if (0 == nProcImageNo % m_nSaveImgStepNo) // 显示处理结果 如果最多一个线程显示可以移动到锁外面
		{
			DrawAndShowImg(&pImg, &m_pShowImage, Corner2D, tFrontLine, tBackLine);
		}
		fprintf(m_pTraceModel->pfImgCalcData, "图号:%d 直角：%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf 关节:%10d%10d%10d%10d%10d%10d%10d%10d%10d 二维点:%10d%10d 三维点:%11.3lf%11.3lf%11.3lf\n",
			nProcImageNo, tCapRobot.dX, tCapRobot.dY, tCapRobot.dZ, tCapRobot.dRX, tCapRobot.dRY, tCapRobot.dRZ, tCapRobot.dBX, tCapRobot.dBY, tCapRobot.dBZ,
			tCapPulse.nSPulse, tCapPulse.nLPulse, tCapPulse.nUPulse, tCapPulse.nRPulse, tCapPulse.nBPulse, tCapPulse.nTPulse, tCapPulse.lBXPulse, tCapPulse.lBYPulse, tCapPulse.lBZPulse,
			Corner2D.x, Corner2D.y, tConnerWorldCoord.dX, tConnerWorldCoord.dY, tConnerWorldCoord.dZ);
		m_Mutex_H_M.unlock();
		lTime5 = GetTickCount();

		m_pRobotDriver->m_cLog->Write("[点云搜端点]: 扫描处理总编号%d总耗时:%d 数据准备:%d 保存图片%d 跟踪处理:%d 飞溅处理%d 点云处理%d 坐标转换%d 更新点云%d 显示结果%d",
			nProcImageNo, lTime5 - lTime1, lTime2 - lTime1, lTime21 - lTime2, lTime22 - lTime21, lTime23 - lTime22, lTime24 - lTime23, lTime3 - lTime24, lTime4 - lTime3, lTime5 - lTime4);
	}
	if (NULL != pImg) cvReleaseImage(&pImg);
	if (NULL != pTempImageBuff) cvReleaseImage(&pTempImageBuff);
	delete[] tPoints;
	m_pRobotDriver->m_cLog->Write("[点云搜端点]: FuncImageProcess结束!");
	return true;
}

bool CScanInitModule::FuncImageProcess_H_M_DoubleVGroove(CRobotDriverAdaptor* pRobotCtrl)
{
	auto dPointCloudLengthForFindEndpnt = PARA_FLAT_MEASURE(dPointCloudLengthForFindEndpnt);
	m_pRobotDriver->m_cLog->Write("[点云搜端点]: FuncImageProcess_H_M_DoubleVGroove开始!");
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
		// 拷贝处理图片和采图坐标数据
		lTime1 = GetTickCount();
		while (m_IsOpenImageProcess_H_M && false == m_pCapImageData_H_M->m_MutexReadWrite.trylock()) Sleep(10);
		if (!m_IsOpenImageProcess_H_M) break;
		nTotalImageNo = m_pCapImageData_H_M->nTotalCapImgNum;
		nProcImageNo = m_pCapImageData_H_M->nTotalProImgNum;
		nMaxBufSize = m_pCapImageData_H_M->nMaxBufferSize;
		nBufNo = nProcImageNo % m_pCapImageData_H_M->nMaxBufferSize;
		if ((nTotalImageNo > 0) && (nProcImageNo < nTotalImageNo)) // 有未处理的图 取数据处理
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
		// 处理速度过慢？
		if ((nTotalImageNo - nProcImageNo) >= nMaxBufSize)
		{
			m_IsOpenImageProcess_H_M = false;
			CheckMachineEmg(pRobotCtrl);
			XUI::MesBox::PopOkCancel("[点云搜端点]: {0} 图像处理过慢! Cap{1} Proc{2} MaxBuf{3}", m_pRobotDriver->m_strRobotName, nTotalImageNo, nProcImageNo, nMaxBufSize);
			continue;
		}
		lTime2 = GetTickCount();

		// 存图 处理 计算
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
		m_pRobotDriver->m_cLog->Write("[点云搜端点]:处理图号%d ResultNum: %d %d %d", nProcImageNo, tInfo.HPntsNum, tInfo.GPntsNum, tInfo.BPntsNum);
		long lTime22 = GetTickCount();

		int nSaveNum = 0;
		int nMinX = pImg->width / 4;
		int nMaxX = pImg->width * 3 / 4;
		ltThreePoint.clear();

		//// 角点坐标
		//tConnerWorldCoord = m_ptUnit->TranImageToBase(m_nCameraNo, Corner2D, tCapRobot, tCapPulse, &tPointAbsCoordInBase);
		//tConnerWorldCoord.dX += tCapRobot.dBX;
		//tConnerWorldCoord.dY += tCapRobot.dBY;
		//tConnerWorldCoord.dZ += tCapRobot.dBZ;
		//fprintf(m_pTraceModel->pfPointCloud, "图号:%d 点号%d %11.3lf%11.3lf%11.3lf\n", nProcImageNo, -1, tConnerWorldCoord.dX, tConnerWorldCoord.dY, tConnerWorldCoord.dZ); // -1用于区分角点和点云坐标
		// 点云坐标
		for (int i = 0; i < nLength; i += m_nPtn2DStepDis)
		{
			if (tPoints[i].x < nMinX || tPoints[i].x > nMaxX) continue; // 无效区域点云数据不保存
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

		// 加锁保存点云数据并删除多余数据
		m_Mutex_H_M.lock();
		m_pTraceModel->ltPointCloud.insert(m_pTraceModel->ltPointCloud.end(), ltThreePoint.begin(), ltThreePoint.end());
		m_pTraceModel->lnImageNo.push_back(nProcImageNo);
		m_pTraceModel->lnImagePtnNum.push_back(nSaveNum);
		m_pTraceModel->ltCapWorldCoord.push_back(tCapRobot);
		m_pTraceModel->vtConnerWorldCoords.push_back(tConnerWorldCoord);
		for (int i = 0; (i < m_pTraceModel->ltCapWorldCoord.size()) && (m_pTraceModel->ltCapWorldCoord.size() > 40); i++) // 删除旧点云数据
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

		fprintf(m_pTraceModel->pfImgCalcData, "图号:%d 直角：%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf 关节:%10d%10d%10d%10d%10d%10d%10d%10d%10d 二维点:%10d%10d 三维点:%11.3lf%11.3lf%11.3lf\n",
			nProcImageNo, tCapRobot.dX, tCapRobot.dY, tCapRobot.dZ, tCapRobot.dRX, tCapRobot.dRY, tCapRobot.dRZ, tCapRobot.dBX, tCapRobot.dBY, tCapRobot.dBZ,
			tCapPulse.nSPulse, tCapPulse.nLPulse, tCapPulse.nUPulse, tCapPulse.nRPulse, tCapPulse.nBPulse, tCapPulse.nTPulse, tCapPulse.lBXPulse, tCapPulse.lBYPulse, tCapPulse.lBZPulse,
			Corner2D.x, Corner2D.y, tConnerWorldCoord.dX, tConnerWorldCoord.dY, tConnerWorldCoord.dZ);
		m_Mutex_H_M.unlock();
		lTime5 = GetTickCount();

		m_pRobotDriver->m_cLog->Write("[点云搜端点]: 扫描处理总编号%d总耗时:%d 数据准备:%d 保存图片%d 跟踪处理:%d 坐标转换%d 更新点云%d 显示结果%d",
			nProcImageNo, lTime5 - lTime1, lTime2 - lTime1, lTime21 - lTime2, lTime22 - lTime21, lTime3 - lTime22, lTime4 - lTime3, lTime5 - lTime4);
	}
	if (NULL != pImg) cvReleaseImage(&pImg);
	if (NULL != pTempImageBuff) cvReleaseImage(&pTempImageBuff);
	delete[] tPoints;
	m_pRobotDriver->m_cLog->Write("[点云搜端点]: FuncImageProcess_H_M_DoubleVGroove结束!");
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
	default: XUI::MesBox::PopOkCancel("H_M:点云处理方法参数错误：{0}", (int)m_ePointCloudProcMethod); break;
	}
	return bRst;
}

bool CScanInitModule::FuncPointCloudProcess_H_M_GetRiser(CRobotDriverAdaptor* pRobotCtrl, int nTimeOut/* = 2000*/)
{
	XiMessageBoxOk("未实现");
	return true;
}

bool CScanInitModule::FuncPointCloudProcess_H_M_Groove(CRobotDriverAdaptor* pRobotCtrl, int nTimeOut/* = 2000*/)
{
	//m_pRobotDriver->m_cLog->Write("[坡口扫描]: FuncPointCloudProcess_H_M_Groove开始!");
	//double dMinProcessLen = m_dSavePointCloudLen - 5.0; // 最小点云处理长度 略小于最大保存长度
	//int nWaitTimeStep = 100;			// 等待运动检查时间步距
	//bool bIsRunning = false;
	//int nPreImageNoE = 0;
	//int nErrorNum = 0;					// 连续处理出错计数
	//int nErrorThreshold = 3;			// 连续处理出错三次结束流程
	//double dProcStepDis = 50.0;			// 更新多长点云执行一次处理
	//T_ROBOT_COORS tPreScanE = m_pRobotDriver->GetCurrentPos();			// 记录每次处理是点云终点坐标


	//// 等待跟踪运动
	//int nCurWaitTime = 0;
	//bIsRunning = m_ptUnit->WorldIsRunning();
	//while (!bIsRunning)
	//{
	//	Sleep(nWaitTimeStep);
	//	nCurWaitTime += nWaitTimeStep;
	//	if (nCurWaitTime > nTimeOut)
	//	{
	//		pRobotCtrl->m_cLog->Write("[过程搜端点]: 终点判断等待运动超时！");
	//		return false;
	//	}
	//	bIsRunning = m_ptUnit->WorldIsRunning();
	//}

	//// 运动中 且 未发现终点
	//while (m_ptUnit->WorldIsRunning())
	//{
	//	m_Mutex_H_M.lock();
	//	int nCurCapWorldCoordNum = m_pTraceModel->ltCapWorldCoord.size();
	//	T_ROBOT_COORS tScanS = nCurCapWorldCoordNum > 0 ? m_pTraceModel->ltCapWorldCoord.front() : T_ROBOT_COORS();
	//	T_ROBOT_COORS tScanE = nCurCapWorldCoordNum > 0 ? m_pTraceModel->ltCapWorldCoord.back() : T_ROBOT_COORS();
	//	double dPointCloudLen = TwoPointDis(
	//		tScanE.dX + tScanE.dBX, tScanE.dY + tScanE.dBY, tScanE.dZ + tScanE.dBZ, 
	//		tScanS.dX + tScanS.dBX, tScanS.dY + tScanS.dBY, tScanS.dZ + tScanS.dBZ);
	//	if (dPointCloudLen < dMinProcessLen) // 点云长度小于最小处理长度
	//	{
	//		m_Mutex_H_M.unlock();
	//		Sleep(nWaitTimeStep);
	//		continue;
	//	}
	//	long long lTime = XI_clock();

	//	// 互斥拷贝出最新数据
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
	//	if (nImageNoE <= nPreImageNoE || fabs(dMoveDis) < dProcStepDis) // 图片要更新 且 间隔dProcStepDis处理一次 重叠长度 m_dSavePointCloudLen - dProcStepDis
	//	{
	//		DELETE_POINTER_ARRAY(pThreeDPoint);
	//		Sleep(nWaitTimeStep);
	//		continue;
	//	}
	//	nPreImageNoE = nImageNoE;
	//	tPreScanE = tScanE;

	//	// 保存每次处理的点云到问题
	//	CString ss;
	//	ss.Format("%sImgNo_S%d-E%d_局部点云.txt", m_sSaveFilePath, nImageNoS, nImageNoE);
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
	//		pRobotCtrl->m_cLog->Write("[过程搜端点]: 坡口提取 点数%d  %d  %d  %d ", nPtnNum,bProcSuccess, pGrooveInfo_Triangle->flag, pGrooveInfo_Trapezoid->flag);
	//	}
	//	catch (...)
	//	{
	//		CString s;
	//		s.Format("%sImgNo_S%d-E%d_处理异常点云.txt", m_sSaveFilePath, nImageNoS, nImageNoE);
	//		FILE* pf = fopen(s.GetBuffer(), "w");
	//		for (int i = 0; i < nPtnNum; i++)
	//		{
	//			fprintf(pf, "%d%11.3lf%11.3lf%11.3lf\n", i, pThreeDPoint[i].x, pThreeDPoint[i].y, pThreeDPoint[i].z);
	//		}
	//		fclose(pf);
	//		CheckMachineEmg(pRobotCtrl);
	//		XiMessageBox("%s [过程搜端点]:点云处理异常! 图号%d-%d", pRobotCtrl->m_strRobotName, nImageNoS, nImageNoE);
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

	//	// 保存单次处理结果
	//	CString s;
	//	s.Format("%sImgNo_S%d-E%d_ProcRst%d_%d_%d.txt", m_sSaveFilePath, nImageNoS, nImageNoE, bProcSuccess, pGrooveInfo_Triangle->flag, pGrooveInfo_Trapezoid->flag);
	//	FILE* pf = fopen(s.GetBuffer(), "w");
	//	bool bSaveBothStartEnd = (0 >= m_pTraceModel->vtPointCloudEndPoints.size());
	//	SaveGrooveInfoToFile(pf, *pGrooveInfo_Triangle, bSaveBothStartEnd); // 记录处理结果到 m_pTraceModel->vtPointCloudEndPoints
	//	SaveGrooveInfoToFile(pf, *pGrooveInfo_Trapezoid, bSaveBothStartEnd);
	//	fclose(pf);

	//	pRobotCtrl->m_cLog->Write("[过程搜端点]: 点云端点提取终点：ImgNo_S%d-E%d pGrooveInfo_Triangle->flag:%d 耗时:%dms",
	//		nImageNoS, nImageNoE, pGrooveInfo_Triangle->flag, XI_clock() - lTime);
	//	DELETE_POINTER_ARRAY(pThreeDPoint);
	//	DELETE_POINTER(pGrooveInfo_Triangle);
	//	DELETE_POINTER(pGrooveInfo_Trapezoid);
	//}
	//m_pRobotDriver->m_cLog->Write("[坡口扫描]: FuncPointCloudProcess_H_M_Groove结束!");
	return true;
}

bool CScanInitModule::FuncPointCloudProcess_H_M_DoubleVGroove(CRobotDriverAdaptor* pRobotCtrl, int nTimeOut/* = 2000*/)
{
	//m_pRobotDriver->m_cLog->Write("[坡口扫描]: FuncPointCloudProcess_H_M_DoubleVGroove开始!");
	//double dMinProcessLen = m_dSavePointCloudLen - 5.0; // 最小点云处理长度 略小于最大保存长度
	//int nWaitTimeStep = 100;			// 等待运动检查时间步距
	//bool bIsRunning = false;
	//int nPreImageNoE = 0;
	//int nErrorNum = 0;					// 连续处理出错计数
	//int nErrorThreshold = 3;			// 连续处理出错三次结束流程
	//double dProcStepDis = 50.0;			// 更新多长点云执行一次处理
	//T_ROBOT_COORS tPreScanE = m_pRobotDriver->GetCurrentPos();			// 记录每次处理是点云终点坐标


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

	//// 等待跟踪运动
	//int nCurWaitTime = 0;
	//bIsRunning = m_ptUnit->WorldIsRunning();
	//while (!bIsRunning)
	//{
	//	Sleep(nWaitTimeStep);
	//	nCurWaitTime += nWaitTimeStep;
	//	if (nCurWaitTime > nTimeOut)
	//	{
	//		pRobotCtrl->m_cLog->Write("[过程搜端点]: 终点判断等待运动超时！");
	//		return false;
	//	}
	//	bIsRunning = m_ptUnit->WorldIsRunning();
	//}

	//// 运动中 且 未发现终点	
	//bool bLastProc = false;
	//while (true == (bIsRunning = m_ptUnit->WorldIsRunning()) || (false == bLastProc)) // 停止以后要再处理一次点云 保证扫描到哪处理到哪里
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
	//	if (dPointCloudLen < dMinProcessLen) // 点云长度小于最小处理长度
	//	{
	//		m_Mutex_H_M.unlock();
	//		Sleep(nWaitTimeStep);
	//		continue;
	//	}
	//	long long lTime = XI_clock();

	//	// 互斥拷贝出最新数据
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
	//	if (nImageNoE <= nPreImageNoE || fabs(dMoveDis) < dProcStepDis) // 图片要更新 且 间隔dProcStepDis处理一次 重叠长度 m_dSavePointCloudLen - dProcStepDis
	//	{
	//		DELETE_POINTER_ARRAY(pThreeDPoint);
	//		Sleep(nWaitTimeStep);
	//		continue;
	//	}
	//	nPreImageNoE = nImageNoE;
	//	tPreScanE = tScanE;

	//	// 保存每次处理的点云到问题f
	//	CString ss;
	//	ss.Format("%sImgNo_S%d-E%d_局部点云.txt", m_sSaveFilePath, nImageNoS, nImageNoE);
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
	//		pRobotCtrl->m_cLog->Write("[过程搜端点]: 坡口提取 点数%d  %d RefHeight:%.3lf UseHeight:%d", nPtnNum, bProcSuccess, fRefHeitht, nUseHeight);
	//	}	
	//	catch (...)
	//	{
	//		CString s;
	//		s.Format("%sImgNo_S%d-E%d_处理异常点云.txt", m_sSaveFilePath, nImageNoS, nImageNoE);
	//		FILE* pf = fopen(s.GetBuffer(), "w");
	//		for (int i = 0; i < nPtnNum; i++)
	//		{
	//			fprintf(pf, "%d%11.3lf%11.3lf%11.3lf\n", i, pThreeDPoint[i].x, pThreeDPoint[i].y, pThreeDPoint[i].z);
	//		}
	//		fclose(pf);
	//		CheckMachineEmg(pRobotCtrl);
	//		XiMessageBox("%s [过程搜端点]:点云处理异常! 图号%d-%d", pRobotCtrl->m_strRobotName, nImageNoS, nImageNoE);
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

	//	// 保存单次处理结果
	//	CString s;
	//	s.Format("%sImgNo_S%d-E%d_ProcRst%d.txt", m_sSaveFilePath, nImageNoS, nImageNoE, bProcSuccess);
	//	FILE* pf = fopen(s.GetBuffer(), "w");
	//	bool bSaveBothStartEnd = (0 >= m_pTraceModel->vtPointCloudEndPoints.size());
	//	SaveGrooveInfoToFile(pf, *pGrooveInfo, bSaveBothStartEnd); // 记录处理结果到 m_pTraceModel->vtPointCloudEndPoints
	//	fclose(pf);

	//	pRobotCtrl->m_cLog->Write("[过程搜端点]: 点云端点提取终点：ImgNo_S%d-E%d 耗时:%dms",
	//		nImageNoS, nImageNoE, XI_clock() - lTime);
	//	DELETE_POINTER_ARRAY(pThreeDPoint);
	//	DELETE_POINTER(pGrooveInfo);
	//}
	//m_pRobotDriver->m_cLog->Write("[坡口扫描]: FuncPointCloudProcess_H_M_DoubleVGroove结束!");
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

	//m_pRobotDriver->m_cLog->Write("FinalProc_Groove Rst: 处理结果%d 单V:%d 双V:%d", bRst, pGrooveInfo_Triangle->flag, pGrooveInfo_Trapezoid->flag);
	//
	//FILE *pf = fopen(sProcRstFileName, "w");
	//if (true == bRst)
	//{
	//	SaveGrooveInfoToFile(pf, *pGrooveInfo_Triangle, true);
	//	SaveGrooveInfoToFile(pf, *pGrooveInfo_Trapezoid, true);
	//	//AppendGrooveInfoPtn(m_pTraceModel->vtPointCloudEndPoints, pGrooveInfo_Triangle->GrooveTopStaPnt); // 第一次保存起点截面
	//	//AppendGrooveInfoPtn(m_pTraceModel->vtPointCloudEndPoints, pGrooveInfo_Triangle->GroovePntStaPnt);
	//	//AppendGrooveInfoPtn(m_pTraceModel->vtPointCloudEndPoints, pGrooveInfo_Triangle->BottomCenterStaPnt);
	//	//AppendGrooveInfoPtn(m_pTraceModel->vtPointCloudEndPoints, pGrooveInfo_Triangle->FlatTopPntStaPnt);
	//	//AppendGrooveInfoPtn(m_pTraceModel->vtPointCloudEndPoints, pGrooveInfo_Triangle->FlatBottomStaPnt);
	//	//AppendGrooveInfoPtn(m_pTraceModel->vtPointCloudEndPoints, pGrooveInfo_Triangle->GrooveTopEndPnt);// 保存终点截面
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
		XUI::MesBox::PopOkCancel("加载第{0}组示教结果文件 {1} 打开失败！", nGroupNo, sFileName);
		return false;
	}

	T_TEACH_RESULT tTeachResult;
	T_TEACH_RESULT tTempTeachResult;
	int nPreReadIdx = -1;
	int nCurReadIdx = 0;
	int nPtnType = 0; // 0交点 1左线 2右线
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
			XUI::MesBox::PopOkCancel("加载第{0}组示教结果文件 {1} 检测处错误的数据类型", nGroupNo, sFileName);
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
		if (-1 == nCurReadIdx) // 保存扫描到的端点坐标 到 第1个测量位置tEndPtn3D中
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
	vtTeachResult.push_back(tTeachResult); // 保存最后一个示教位置结果

	if ((EOF != nRst) && (nDataColNum != nRst))
	{
		XUI::MesBox::PopOkCancel("加载第{0}组示教结果数据文件 {1} 检测到不完整数据！", nGroupNo, sFileName);
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
//		AppendGrooveInfoPtn(m_pTraceModel->vtPointCloudEndPoints, tGrooveInfo.GrooveTopEndPnt);// 保存终点截面
//		AppendGrooveInfoPtn(m_pTraceModel->vtPointCloudEndPoints, tGrooveInfo.GroovePntEndPnt);
//		AppendGrooveInfoPtn(m_pTraceModel->vtPointCloudEndPoints, tGrooveInfo.BottomCenterEndPnt);
//		AppendGrooveInfoPtn(m_pTraceModel->vtPointCloudEndPoints, tGrooveInfo.FlatTopPntEndPnt);
//		AppendGrooveInfoPtn(m_pTraceModel->vtPointCloudEndPoints, tGrooveInfo.FlatBottomEndPnt);
//	}
//	
//	AppendGrooveInfoPtn(m_pTraceModel->vtPointCloudEndPoints, tGrooveInfo.GrooveTopStaPnt); // 第一次保存起点截面
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
//		AppendGrooveInfoPtn(m_pTraceModel->vtPointCloudEndPoints, tGrooveInfo.TopLeftStaPnt); // 第一次保存起点截面
//		AppendGrooveInfoPtn(m_pTraceModel->vtPointCloudEndPoints, tGrooveInfo.BottomLeftStaPnt);
//		AppendGrooveInfoPtn(m_pTraceModel->vtPointCloudEndPoints, tGrooveInfo.BottomCenterStaPnt);
//		AppendGrooveInfoPtn(m_pTraceModel->vtPointCloudEndPoints, tGrooveInfo.TopRightStaPnt);
//		AppendGrooveInfoPtn(m_pTraceModel->vtPointCloudEndPoints, tGrooveInfo.BottomRightStaPnt);
//	}
//	AppendGrooveInfoPtn(m_pTraceModel->vtPointCloudEndPoints, tGrooveInfo.TopLeftEndPnt);// 保存终点截面
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
//		AppendGrooveInfoPtn(m_pTraceModel->vtPointCloudEndPoints, tGrooveInfo.TopLeftStaPnt); // 第一次保存起点截面
//		AppendGrooveInfoPtn(m_pTraceModel->vtPointCloudEndPoints, tGrooveInfo.BottomLeftStaPnt);
//		AppendGrooveInfoPtn(m_pTraceModel->vtPointCloudEndPoints, tGrooveInfo.BottomCenterStaPnt);
//		AppendGrooveInfoPtn(m_pTraceModel->vtPointCloudEndPoints, tGrooveInfo.TopRightStaPnt);
//		AppendGrooveInfoPtn(m_pTraceModel->vtPointCloudEndPoints, tGrooveInfo.BottomRightStaPnt);
//	}
//	AppendGrooveInfoPtn(m_pTraceModel->vtPointCloudEndPoints, tGrooveInfo.TopLeftEndPnt);// 保存终点截面
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
    dNewRz = fmod(dNewRz, 360.0); // ±360
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
        WriteLog("平滑方向：dRZ：%lf dNewRz:%lf", dRZ, dNewRz);
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
    WriteLog("当前队列元素数量%d  值：%s 计算结果：%.3lf", nItemNum, str, dRst);
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
		//已修改
		XUI::MesBox::PopInfo("{0}数据加载失败", strPath.GetBuffer());
		//XiMessageBox("%s 数据加载失败", strPath);
		return FALSE;
	}
	if (!CheckFileExists(strPath2))
	{

		//已修改
		XUI::MesBox::PopInfo("{0}数据加载失败", strPath2.GetBuffer());
		//XiMessageBox("%s 数据加载失败", strPath2);
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
    //计算起始段方向
    std::vector<XI_POINT> vtScanPoints;//记录起始段扫描点
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
	XUI::MesBox::PopError("机械臂急停");
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
			pcMyObj->m_ptUnit->GetRobotCtrl()->m_cLog->Write("ThreadJudgeIsWeldCompen线程结束");
			return false;
		}
		if (pcMyObj->m_pTraceModel->m_bIfWeldToEndPoint)
		{
			pcMyObj->m_ptUnit->GetRobotCtrl()->m_cLog->Write("ThreadJudgeIsWeldCompen结束");
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
	// tEndpointCoor ：dX dY dZ dBX 焊接终点世界坐标
	double dWorldDis = TwoPointDis(
		tEndpointCoor.dX + tEndpointCoor.dBX, tEndpointCoor.dY,
		tRobotCoor.dX + dExAxisPos, tRobotCoor.dY);
	double dAxisY = fabs(tEndpointCoor.dBX - dExAxisPos);
#endif
	pRobotDriver->m_cLog->Write("查找到结尾点1：%.3lf %.3lf %.3lf %.3lf %.3lf %.3lf",
		dWorldDis, dAxisY, m_pTraceModel->dHandEyeDis, m_pTraceModel->m_dChangeAngleTheshold,
		tEndpointCoor.dX + tEndpointCoor.dBX, tEndpointCoor.dY + tEndpointCoor.dBY);
	if (dWorldDis < 2.5 && dAxisY < 1.5)
	{
		m_pTraceModel->m_bIfWeldToEndPoint = TRUE;
		pRobotDriver->m_cLog->Write("查找到结尾点：%.3lf %.3lf", dWorldDis, dAxisY);
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
		m_pTraceModel->m_dCurrentWeldLen>(m_pTraceModel->m_dWeldLen - m_pTraceModel->dHandEyeDis)*/)//结束跟踪,当前焊接长度大于总长度减去手眼长度结束跟踪
	{
		m_pTraceModel->m_bIsCloseTrackThread = true;
		pRobotDriver->m_cLog->Write("结束跟踪:dEndTrackDis:%lf dWorldDis:%lf", dEndTrackDis, dWorldDis);
	}
	return true;
}

BOOL CScanInitModule::GetEndSectionCoors(CRobotDriverAdaptor *pRobotDriver, T_ROBOT_COORS &tEndpointCoor) 
{ 
	int nSize = pRobotDriver->m_vtWeldLineInWorldPoints.size();
	T_ROBOT_COORS tEndpoint = pRobotDriver->m_vtWeldLineInWorldPoints.at(nSize - 1);
	T_ROBOT_COORS tEndpointTemp = tEndpointCoor;

	pRobotDriver->m_cLog->Write("记录结尾数据： %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf",
		tEndpointCoor.dX, tEndpointCoor.dY, tEndpointCoor.dZ, tEndpointCoor.dRZ,tEndpointCoor.dBY,
		tEndpoint.dX, tEndpoint.dY,tEndpoint.dZ, tEndpoint.dRZ, tEndpoint.dBY);

	//取后面几个点拟合方向
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

	//更新结尾工件方向
	//m_pTraceModel->m_dWorkPieceDir = dNormal;

	double dDisError = TwoPointDis(tEndpointCoor.dX + tEndpointCoor.dBX, tEndpointCoor.dY + tEndpointCoor.dBY, tEndpointCoor.dZ,
		pRobotDriver->m_vtWeldLineInWorldPoints.at(nSize - 1).dX + pRobotDriver->m_vtWeldLineInWorldPoints.at(nSize - 1).dBX,
		pRobotDriver->m_vtWeldLineInWorldPoints.at(nSize - 1).dY + pRobotDriver->m_vtWeldLineInWorldPoints.at(nSize - 1).dBY,
		pRobotDriver->m_vtWeldLineInWorldPoints.at(nSize - 1).dZ);
	//添加偏移量
	double dInitComp = m_pTraceModel->m_dGunToEyeCompenX;// 外加内减
	double dAbjustHeight = m_pTraceModel->m_dGunToEyeCompenZ;// 向上补加 向下补减
	if (E_CLOSEDARC == m_pTraceModel->m_tWorkPieceType)
	{
		dInitComp = 0;
		dAbjustHeight = 0;
	}
	if (dDisError > 130)
	{
		tEndpointCoor = pRobotDriver->m_vtWeldLineInWorldPoints.at(nSize - 1);
		//若提前停止则不保角焊接且立即停止焊接，并提示
		//m_pTraceModel->m_eWrapAngleType = E_WRAPANGLE_EMPTY_SINGLE;
		//SetIntValFun(pRobotDriver, 7, 0);
		pRobotDriver->m_cLog->Write(" 结尾点判断失误将以上一点为结尾点停止设备，并将包角类型置为不保角：%lf ", dDisError);

	}
	else
	{
		// 二次滤波获 放入终点 取终点 Start  （算法终点获取函数需要修改）
		T_ROBOT_COORS tTempCoors;
		TrackSmooth_PntArray tTrackPtnArray;
		tTrackPtnArray.arraySize_ = 0;
		tTrackPtnArray.pnts_ = NULL;
		// 初始化结尾段滤波函数
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
			// 投影
			XI_POINT tProjectpoint;
			PointtoLineProjection(tLine, tEndpoint, tProjectpoint);
			tEndpointCoor.dX = tProjectpoint.x - tEndpointCoor.dBX;
			tEndpointCoor.dY = tProjectpoint.y - tEndpointCoor.dBY;
			tEndpointCoor.dZ = tProjectpoint.z;

			pRobotDriver->m_cLog->Write("结尾点投影:%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf \n",
				tEndpointCoor.dX, tEndpointCoor.dY, tEndpointCoor.dZ, tEndpointCoor.dBY, tEndpointCoor.dY + tEndpointCoor.dBY);
		}
		// 获取结尾段数据
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
			SaveRobotCoor(m_pRecordTheoryPoint, tTempCoors, E_WELD_TRACK);//保存理论数据
			pRobotDriver->m_cLog->Write("TrackSmooth_GetEndPoint1:%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf \n",
				tTempCoors.dX, tTempCoors.dY, tTempCoors.dZ, tTempCoors.dBY, dNormal);
		}
		TrackSmooth_FreePntArray(&tTrackPtnArray);

		//获取结尾点，仅结尾时调用，与上一点的距离小于 moveMinDis 
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
			SaveRobotCoor(m_pRecordTheoryPoint, tTempCoors, E_WELD_TRACK);//保存理论数据

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
		//更新结尾数据
		m_pTraceModel->m_tRealEndpointCoor = tEndpointCoor;
		m_pTraceModel->m_tRealEndpointCoor.dRZ = DirAngleToRz(dNormal);

		nSize = pRobotDriver->m_vtWeldLineInWorldPoints.size();
		//变姿态
		if (m_pTraceModel->m_nCloseTrackingPos > 0)
		{
			double DAngle = 45;/*tEndpointTemp.dRZ - tEndpoint.dRZ;
			DAngle = DAngle > 180 ? DAngle - 360 : DAngle;
			DAngle = DAngle < -180 ? DAngle + 360 : DAngle;*/
			double dStepDis = DAngle / m_pTraceModel->m_nCloseTrackingPos;
			if (fabs(dStepDis) > 10 || fabs(DAngle) > 90)
			{
				XUI::MesBox::PopOkCancel("参数存在问题：{0} {1}", dStepDis, DAngle);
				return false;
			}
			for (int n = m_pTraceModel->m_nCloseTrackingPos; n > 0; n--)
			{
				pRobotDriver->m_vtWeldLineInWorldPoints.at(nSize - (m_pTraceModel->m_nCloseTrackingPos - n) - 1).dRZ -= dStepDis * n * pRobotDriver->m_nRobotInstallDir;
				pRobotDriver->m_cLog->Write("结尾变化姿态：%lf %lf",
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
	pRobotDriver->m_cLog->Write("%s 时间统计：%s 时间:%11.3lf", pRobotDriver->m_strRobotName, str, time);
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
	T_CART_COOR dMachineCoors;//记录大车坐标
	long long nTime = XI_clock();
	m_bEndPointCapCur = false;
	CDHGigeImageCapture* pDHCamDrv = (CDHGigeImageCapture*)m_ptUnit->GetCameraCtrl(m_ptUnit->m_nTrackCameraNo);
	//结束循环的唯一条件为 m_bCameraFindWeldStart == TRUE
	while (FALSE == m_pTraceModel->m_bCameraFindWeldEnd)
	{
		if (pRobotDriver->m_eThreadStatus == INCISEHEAD_THREAD_STATUS_STOPPED) break;
		long long lStartTime = XI_clock();
		nTime = XI_clock();
		//获取采图位置
		tRobotCurCoord = pRobotDriver->GetCurrentPos();
		tRobotCurPulses = pRobotDriver->GetCurrentPulse();
		//合新框架临时
		dMachineCoors.dX = tRobotCurCoord.dBX;
		dMachineCoors.dY = tRobotCurCoord.dBY;
		dMachineCoors.dZ = tRobotCurCoord.dBZ;

		m_ptStartPointInfo[m_nEndPointCapCursor % MAX_ARRAY_NUM_START].tRobotCoors = tRobotCurCoord;
		m_ptStartPointInfo[m_nEndPointCapCursor % MAX_ARRAY_NUM_START].tRobotPulse = tRobotCurPulses;
		m_ptStartPointInfo[m_nEndPointCapCursor % MAX_ARRAY_NUM_START].dMachineCoors = dMachineCoors;
		//获取图片
		pDHCamDrv->CaptureImage(m_ptStartPointInfo[m_nEndPointCapCursor % MAX_ARRAY_NUM_START].img, 1);
		//cvCopyImage(, m_ptStartPointInfo[m_nEndPointCapCursor % MAX_ARRAY_NUM_START].img);
		Sleep(100);
		m_nEndPointCapCursor++;
		pRobotDriver->m_cLog->Write("结尾扫描采图%d耗时%dms", m_nEndPointCapCursor, XI_clock() - nTime);
	}
	m_bEndPointCapCur = true;
	pRobotDriver->m_cLog->Write("%s 结尾采图线程退出：%d", m_ptUnit->GetUnitName(), m_bEndPointCapCur);

}

void CScanInitModule::TranslateDrawingToImageParam(CRobotDriverAdaptor *pRobotDriver) { return; }

//先行小组找终点函数						
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
		// 这里采取蛙跳法来进行效率优化(先注释掉)
		if (m_nEndPointCapCursor < 2) // 从采集到第三张图开始
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
		pRobotCtrl->m_cLog->Write("%s 结尾图像处理 %d %d %d", strRobot, m_pTraceModel->m_bCameraFindWeldEnd, cpKeyPoint.x, cpKeyPoint.y);

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
					pRobotCtrl->m_cLog->Write("%s 图像回找未找到终点，焊接停止", strRobot);
					XUI::MesBox::PopInfo("{0}号机器人图像回找未找到终点，焊接停止", pRobotCtrl->m_nRobotNo);
					break;
				}
			}
			// 第一次回找已经找到一个原流程的终点 计算保存作为对比
			m_ptUnit->TranImageToBase(m_ptUnit->m_nTrackCameraNo, cpKeyPoint, m_ptStartPointInfo[m_nEndPointProcCursor % MAX_ARRAY_NUM_START].tRobotCoors,
				m_ptStartPointInfo[m_nEndPointProcCursor % MAX_ARRAY_NUM_START].tRobotPulse, &tPointAbsCoordInBase);

			pRobotCtrl->m_cLog->Write("%s 原流程扫描到的终点：%11.3lf%11.3lf%11.3lf nImageBackFindSum:%d", strRobot,
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
	pRobotCtrl->m_cLog->Write("%s 开始检测存图线程结尾", pRobotCtrl->m_strRobotName);
	if (TRUE == m_pTraceModel->m_bCameraFindWeldEnd)
	{
		pRobotCtrl->m_cLog->Write("%s 相机找到终点: m_bCameraFindWeldEnd=%d cpKeyPoint:%d %d", strRobot, m_pTraceModel->m_bCameraFindWeldEnd,cpKeyPoint.x, cpKeyPoint.y);
		pRobotCtrl->m_cLog->Write("%s 终点采集索引:%d 终点处理索引:%d ", strRobot, m_nEndPointCapCursor, m_nEndPointProcCursor);

		m_ptUnit->TranImageToBase(m_ptUnit->m_nTrackCameraNo, cpKeyPoint, m_ptStartPointInfo[m_nEndPointProcCursor % MAX_ARRAY_NUM_START].tRobotCoors,
			m_ptStartPointInfo[m_nEndPointProcCursor % MAX_ARRAY_NUM_START].tRobotPulse, &tPointAbsCoordInBase);
		// 粗结尾赋值
		T_ROBOT_COORS tRealEndpoint;
		tRealEndpoint.dX = 1 == m_ptUnit->m_nTrackCameraNo ? tPointAbsCoordInBase.tWeldLinePos.x + m_ptStartPointInfo[m_nEndPointProcCursor % MAX_ARRAY_NUM_START].tRobotCoors.dBX : tPointAbsCoordInBase.tWeldLinePos.x;
		tRealEndpoint.dY = 2 == m_ptUnit->m_nTrackCameraNo ? tPointAbsCoordInBase.tWeldLinePos.y + m_ptStartPointInfo[m_nEndPointProcCursor % MAX_ARRAY_NUM_START].tRobotCoors.dBY : tPointAbsCoordInBase.tWeldLinePos.y;
		tRealEndpoint.dZ = tPointAbsCoordInBase.tWeldLinePos.z;
		m_pTraceModel->m_tRealEndpointCoor = tRealEndpoint;
		// 结尾点滤波
		GetEndSectionCoors(pRobotCtrl, m_pTraceModel->m_tRealEndpointCoor);
		// 判断结尾线程
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
    //读取
    UINT unCount = 1;
    MP_VAR_INFO tVarInfo[1];
    tVarInfo[0].usIndex = nPIndex;
    tVarInfo[0].usType = MP_RESTYPE_VAR_ROBOT;
    LONG lPosVarData[10];
	long long tTime = XI_clock();
    pRobotDriver->GetMultiPosVar(unCount, tVarInfo, lPosVarData);
    pRobotDriver->m_cLog->Write("22 %d", XI_clock() - tTime);
    //写入
    if (MP_ROBO_COORD == (lPosVarData[0] & 0x3F))//直角坐标
    {

        pRobotDriver->m_cLog->Write("对比坐标：%d  %.3lf   %.3lf   %.3lf   %.3lf   %.3lf   %.3lf",
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
    else if (MP_PULSE_COORD == (lPosVarData[0] & 0x3F))//关节坐标
    {
        pRobotDriver->m_cLog->Write("对比坐标脉冲:%d %10ld %10ld %10ld %10ld %10ld %10ld", nPIndex, lPosVarData[2], lPosVarData[3],
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

//多线图像处理互斥函数,参数结构体为 T_IMAGE_PROCINPUT_PARAM
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
	// 初始段存储完整数据，结尾段存储结尾点
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
			//tPtn.x = 1 == m_ptUnit->m_nMeasureAxisNo ? tPtn.x - dExPos : tPtn.x; // 原此处需要机器人坐标
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
	// 检查端点坐标，去除重复或间隔过小坐标
	for (int nPointNo = 0; nPointNo < vtRealEndpoint.size(); nPointNo++)
	{
		if (nPointNo > 0){
			// 距离起点长度
			double dStartDis = TwoPointDis(vtRealEndpoint[nPointNo].x, vtRealEndpoint[nPointNo].y, vtRealEndpoint[nPointNo].z,
				vtRealEndpoint[0].x, vtRealEndpoint[0].y, vtRealEndpoint[0].z);
			// 距上一个点长度
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
	// 检查结尾点是否存在
	dLength = TwoPointDis(tRealEndpoint.x, tRealEndpoint.y, tRealEndpoint.z,
		vtEndpoint.at(vtEndpoint.size() - 1).x, vtEndpoint.at(vtEndpoint.size() - 1).y, vtEndpoint.at(vtEndpoint.size() - 1).z);
	if (dLength > 0.1)
	{
		vtEndpoint.push_back(tRealEndpoint);
	}

	// 添加外部轴，合并世界坐标
	for (int i = 0; i < vtEndpoint.size(); i++)
	{
		vtEndpoint[i].x = 1 == m_ptUnit->m_nMeasureAxisNo ? vtEndpoint[i].x + dExPos : vtEndpoint[i].x;
		vtEndpoint[i].y = 2 == m_ptUnit->m_nMeasureAxisNo ? vtEndpoint[i].y + dExPos : vtEndpoint[i].y;
		vtEndpoint[i].z = 3 == m_ptUnit->m_nMeasureAxisNo ? vtEndpoint[i].z + dExPos : vtEndpoint[i].z;
	}

	// 翻转起始段数据
	if (0 == nEndpointNo)
	{
		reverse(vtEndpoint.begin(), vtEndpoint.end());
	}
	
	// 存储数据
	CString strFileName;
	strFileName.Format("%s%s%sEndpointCoors-%d-%d.txt", OUTPUT_PATH,RobotName,RECOGNITION_FOLDER, nGroupNo, nEndpointNo);
	SavePointsData(vtEndpoint, strFileName);
}





