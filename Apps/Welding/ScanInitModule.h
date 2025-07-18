/*********************************************************************
* 版权所有 (C)2018, 哈尔滨行健智能机器人股份有限公司
* 
* 文件名称： ScanInitModule.h
* 内容摘要： 
* 当前版本： 1.0
* 作    者： 刘富强
* 完成日期： 2018年3月14日
* 其它说明： 
* 
* 修改记录1：
*    修改日期：
*    版 本 号：
*    修 改 人：刘继兴 张深
*    修改内容： 。。。。
* 修改记录2：…

使用说明：
1.定义对象
2.

注意：

**********************************************************************/

#if !defined(AFX_SCANWORK_H__075B4081_4CAC_47E6_A30F_411__INCLUDED_)
#define AFX_SCANWORK_H__075B4081_4CAC_47E6_A30F_411__INCLUDED_


#include <atlstr.h>
#include <string>
#include <cstring>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <opencv2\opencv.hpp>
#include <map>
#include <vector>
#include <dbghelp.h>

#include ".\Apps\PLib\YaskawaRobot\RobotDriverAdaptor.h"
#include ".\Apps\PLib\LeisaiCtrl\MoveCtrlModule.h"
#include ".\OpenClass\FileOP\ini\opini.h"
#include ".\Apps\PLib\CtrlUnit\CUnit.h"
#include ".\Project\WrapAngleParam.h"
#include "XiAlgorithm.h"
#include "GsImageProcess.h"
#include "AsynchrousProc.h"
//#include "Get Laser Point.h"

#include ".\Apps\PLib\LeisaiCtrl\StepMoveControlJY.h"
#include ".\LocalFiles\ExLib\ALGO\include\MoveHandle_C_DLL.h"
#include ".\LocalFiles\ExLib\ALGO\include\TrackFilterHandle_C_DLL.h"
#include ".\LocalFiles\ExLib\ALGO\include\TrackSmoothHandle_C_DLL.h"

#include "PointCloudWeldingLinesExtraction.h"
#include "RiserManagement.h"
#include "FindCornner.h"
#include "CloudManagement.h"

#define __ScanInitModule_ConsoleDbgPrint			//控制台调试输出模式

#define MAX_ARRAY_NUM_START 5
#define MAX_COPY_START 5

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

using namespace std;
using namespace cv;
using namespace GROUP_STAND_DIP_NS;
using namespace GROUP_ROBOT_ABS_COORS_TRANS;
//using namespace DHGIGE_IMAGECAPTURE_NS;

typedef enum E_POINT_CLOUD_PROC_MOMENT				// 点云扫描:点云处理时刻
{
	E_POINT_CLOUD_PROC_MOMENT_DYNAMIC = 0,	// 实时处理
	E_POINT_CLOUD_PROC_MOMENT_FINAL = 1		//扫描完后处理
}E_POINT_CLOUD_PROC_MOMENT;

typedef enum E_IMAGE_PROC_METHOD			// 点云扫描:图区激光中心点方法
{
	E_IMAGE_PROC_METHOD_EEEEE = 0,			// FindLaserMidPiontEEEEE
	E_IMAGE_PROC_METHOD_EEEEEInTrack = 1,	// FindLaserMidPiontEEEEEInTrack
	E_IMAGE_PROC_METHOD_DoubleVGroove = 2,	// 马亚飞坡口激光中心点提取
}E_IMAGE_PROC_METHOD;

typedef enum E_POINT_CLOUD_PROC_METHOD				// 点云扫描:点云处理方法
{
	E_POINT_CLOUD_PROC_METHOD_GetRiserEndPoint = 0,	// GetRiserEndPoint
	E_POINT_CLOUD_PROC_METHOD_Groove = 1,			// GrooveSolutionInterface_
	E_POINT_CLOUD_PROC_METHOD_DoubleVGroove = 2			// GrooveSolutionInterface_
}E_POINT_CLOUD_PROC_METHOD;

typedef enum E_JUDGE_END_METHOD				// 终点判断方法
{
	E_JUDGE_END_KNOW = 0,					// 已知端点(需要提前设置端点)
	E_JUDGE_END_THEORY_LENGTH = 1,			// 理论长都判断
	E_JUDGE_END_PROCESS_SEARCH = 2			// 过程中搜索端点
}E_JUDGE_END_METHOD;

typedef enum E_SCANWORK_THREAD_STATUS
{
	SCANWORK_THREAD_STATUS_START = 0,
	SCANWORK_THREAD_STATUS_SCAN,
	SCANWORK_THREAD_STATUS_STOPPED,
	SCANWORK_THREAD_STATUS_COMPLETE
}E_SCANWORK_THREAD_STATUS;

typedef enum E_SCAN_MODE
{
	COARSE_SCAN_MODE = 0,
	ACCURATE_SCAN_MODE
}E_SCAN_MODE;

// nMaxBufferSize：最大数据保存数量
// nTotalCapImgNum：采集图片递增
// nTotalProImgNum：处理图片递增
// vpImg：与序号对应的图片集合，循环保存，处理时 加锁->先拷贝->释放->解锁->处理
// vtCapCoord：采集直角坐标
// vtCapPulse：采集关节坐标
typedef struct T_CAP_IMAGE_DATA
{
	int nMaxBufferSize = 0;
	int nTotalCapImgNum; // 当前采集总数量
	int nTotalProImgNum; // 当前处理总数量
	std::vector<IplImage*> vpImg; // 图片指针
	std::vector<T_ROBOT_COORS> vtCapCoord; // 采图直角
	std::vector<T_ANGLE_PULSE> vtCapPulse; // 采图关节
	Mutex m_MutexReadWrite; // 保存和处理互斥锁
	T_CAP_IMAGE_DATA(int nMaxBufSize = 50)
	{
		this->nMaxBufferSize = nMaxBufSize;
		this->nTotalCapImgNum = 0;
		this->nTotalProImgNum = 0;
		this->vpImg.resize(this->nMaxBufferSize, NULL);
		this->vtCapCoord.resize(this->nMaxBufferSize);
		this->vtCapPulse.resize(this->nMaxBufferSize);
	}
}T_CAP_IMAGE_DATA;

typedef struct T_START_POINT_INFO
{
	double dMachineX;
	double dMachineY;
	T_CART_COOR dMachineCoors;
	T_ROBOT_COORS tRobotCoors;
	T_ANGLE_PULSE tRobotPulse;
	XI_POINT tWeldLineCoor;
	IplImage *img;
	int sata;
	int num;
    int nScanImageNo;
	T_START_POINT_INFO()
	{
		dMachineX = 0.0;
		dMachineY = 0.0;
		tRobotCoors;
		tRobotPulse;
		img = NULL;
		sata = 0;
		num = 0;
        nScanImageNo = 0;
	}
	T_START_POINT_INFO& operator=(const T_START_POINT_INFO &obj)
	{
		this->dMachineX = obj.dMachineX;
		this->dMachineY = obj.dMachineY;
		this->dMachineCoors.dX = obj.dMachineCoors.dX;
		this->dMachineCoors.dY = obj.dMachineCoors.dY;
		this->dMachineCoors.dZ = obj.dMachineCoors.dZ;
		this->dMachineCoors.dU = obj.dMachineCoors.dU;
		this->tRobotCoors.dX = obj.tRobotCoors.dX;
		this->tRobotCoors.dY = obj.tRobotCoors.dY;
		this->tRobotCoors.dZ = obj.tRobotCoors.dZ;
		this->tRobotCoors.dRX = obj.tRobotCoors.dRX;
		this->tRobotCoors.dRY = obj.tRobotCoors.dRY;
		this->tRobotCoors.dRZ = obj.tRobotCoors.dRZ;
		this->tRobotCoors.dBX = obj.tRobotCoors.dBX;
		this->tRobotCoors.dBY = obj.tRobotCoors.dBY;
		this->tRobotCoors.dBZ = obj.tRobotCoors.dBZ;
		this->tRobotPulse.nSPulse = obj.tRobotPulse.nSPulse;
		this->tRobotPulse.nLPulse = obj.tRobotPulse.nLPulse;
		this->tRobotPulse.nUPulse = obj.tRobotPulse.nUPulse;
		this->tRobotPulse.nRPulse = obj.tRobotPulse.nRPulse;
		this->tRobotPulse.nBPulse = obj.tRobotPulse.nBPulse;
		this->tRobotPulse.nTPulse = obj.tRobotPulse.nTPulse;
		this->tRobotPulse.lBXPulse = obj.tRobotPulse.lBXPulse;
		this->tRobotPulse.lBYPulse = obj.tRobotPulse.lBYPulse;
		this->tRobotPulse.lBZPulse = obj.tRobotPulse.lBZPulse;
		this->tWeldLineCoor.x = obj.tWeldLineCoor.x;
		this->tWeldLineCoor.y = obj.tWeldLineCoor.y;
		this->tWeldLineCoor.z = obj.tWeldLineCoor.z;
		this->img = obj.img;
		this->sata = obj.sata;
		this->num = obj.num;
		this->nScanImageNo = obj.nScanImageNo;
		return *this;
	}
}T_START_POINT_INFO;

typedef struct T_SCAN_RESULT
{
	double dStartPosX;
	double dStartPosY;
	double dWorkAngle;
}T_SCAN_RESULT;

//ImgProcDis中的子线程ThreadAsyn...的图像识别参数
struct T_IMAGE_RECOGNIZE_PARAM
{
	int iBaseLineLenInZone;
	int iMoveLineLenInZone;
	int iRoiWidSize;
	int iIfEndConditionIdx;
	GROUP_STAND_DIP_NS::E_WHICH_PIECE_CLASS ePieceClass;
	BOOL bScanStartTwoLineOnStandBoard;
	double dScanStartExtendLength;

	T_IMAGE_RECOGNIZE_PARAM(int param1,int param2,int param3,int param4, 
		GROUP_STAND_DIP_NS::E_WHICH_PIECE_CLASS param5,BOOL param6,double param7):
		iBaseLineLenInZone(param1), iMoveLineLenInZone(param2), iRoiWidSize(param3),iIfEndConditionIdx(param4),
		ePieceClass(param5), bScanStartTwoLineOnStandBoard(param6), dScanStartExtendLength(param7) {}
};

//多线程存点云输入参数
struct T_THREAD_SAVEPOINTCLOUD
{
	FILE* pOutputFile;						//输出文件指针
	vector<vector<XI_POINT>>* pVvtLaserLine;	//处理的激光线队列
	Mutex* pMutexWrite;							//文件写入锁
	Mutex* pMutex_MyVec;						//vvtLaserLine读写锁
	int* pCount;								//计数器
	bool* pExit;								//是否退出存点云线程

	T_THREAD_SAVEPOINTCLOUD(FILE* pOutputFile, vector<vector<XI_POINT>>* pVvtLaserLine, Mutex* pMutexWrite, Mutex* pMutex_MyVec, int* pCount, bool* pExit)
		: pOutputFile(pOutputFile), pVvtLaserLine(pVvtLaserLine), pMutexWrite(pMutexWrite), pMutex_MyVec(pMutex_MyVec), pCount(pCount), pExit(pExit)
	{
	}
};

//m_pMapImgProcOutput {键-值}队中的值类型 
struct T_THREAD_DYNAMICSCAN_OUT
{
	CvPoint cpKeyPoint;									//二维交点位置,没交点默认是(0,0)
	T_ABS_POS_IN_BASE tPointAbsCoordInBase;
	BOOL bFindIntersectPoint;
	vector<XI_POINT> vtLaserPoint;						//激光线 点云集
	T_THREAD_DYNAMICSCAN_OUT(CvPoint param1,T_ABS_POS_IN_BASE	param2, BOOL param3,vector<XI_POINT> param4) :
		cpKeyPoint(param1),tPointAbsCoordInBase(param2), bFindIntersectPoint(param3), vtLaserPoint(param4){}
};

//搜端点处理线程（多线程）输入参数
struct T_THREAD_DYNAMICSCAN_PARAM
{
	E_START_OR_END eEndPointType;										//搜索类型
	CRobotDriverAdaptor* pRobotDriver;									//机器人控制器指针
	vector<pair<int, T_START_POINT_INFO>>* pVtnImgProcInput;				//图片处理队列指针
	map<int, T_THREAD_DYNAMICSCAN_OUT>* pMapDynamicScanOut;				//输出结果表指针
	T_IMAGE_RECOGNIZE_PARAM tImgRecParam;								//图像处理参数
	ofstream* pOutFile;													//线程内部输出文件指针
	FILE* pOutFilePointCloud;											//存点云点文件指针（使用FLIE* 比ofstream* 速度快指数级）
	AsynchrousProc* pAsynProc;											//异步处理指针
	Mutex* pMutexMapLock;												//Map互斥锁				R2LevelLock
	Mutex* pMutexExitFlag;												//线程退出标志位锁		R2LevelLock
	Mutex* pMutexVtInput;												//图片队列访问锁		R2LevelLock
	Mutex* pMutexOutFilePC;												//点云文件锁
	IplImage* pShowImg;													//显示图像
	BOOL bSaveImg;														//存图标志位
	int nThreadIndex;													//该线程在总处理线程队列中的编号
	BOOL* pBisExit;														//线程退出标志位
	BOOL bSavePointCloud;												//点云标志位
	CImageProcess* pImgProcess;											//图像处理指针
	E_FLIP_MODE eFlipMode;												//二维图像翻转类型
	E_SCANMODE eScanMode;												//扫描模式
	CString cstrUnitName;												//控制单元名称
	T_CAMREA_PARAM tCamParam;											//相机参数
	XiRobotAbsCoorsTrans cAbsCoorsTrans;								//坐标转换类
	CString cstrImgSavePath;											//图片保存路径
	int* pSavePointCloudCount;											//点云序号计数器
	int nNumOfSavePointThreads;											//存点云的最大线程数量


	T_THREAD_DYNAMICSCAN_PARAM(const E_START_OR_END& eEndPointType, CRobotDriverAdaptor* pRobotDriver, vector<pair<int, T_START_POINT_INFO>>* pVtnImgProcInput,
		map<int, T_THREAD_DYNAMICSCAN_OUT>* pMapDynamicScanOut, const T_IMAGE_RECOGNIZE_PARAM& tImgRecParam, ofstream* pOutFile, FILE* pOutFilePointCloud, AsynchrousProc* pAsynProc,
		Mutex* pMutexMapLock, Mutex* pMutexExitFlag, Mutex* pMutexVtInput, Mutex* pMutexOutFilePC, IplImage* pShowImg, const BOOL& bSaveImg, int nThreadIndex, BOOL* pBisExit, const BOOL& bSavePointCloud,
		CImageProcess* pImgProcess, E_FLIP_MODE eFlipMode, E_SCANMODE eScanMode, CString cstrUnitName, T_CAMREA_PARAM tCamParam, XiRobotAbsCoorsTrans cAbsCoorsTrans,
		CString cstrImgSavePath, int* pSavePointCloudCount, int nNumOfSavePointThreads
	)
		: eEndPointType(eEndPointType), pRobotDriver(pRobotDriver), pVtnImgProcInput(pVtnImgProcInput), pMapDynamicScanOut(pMapDynamicScanOut), tImgRecParam(tImgRecParam), pOutFile(pOutFile),
		pOutFilePointCloud(pOutFilePointCloud), pAsynProc(pAsynProc), pMutexMapLock(pMutexMapLock), pMutexExitFlag(pMutexExitFlag), pMutexVtInput(pMutexVtInput), pMutexOutFilePC(pMutexOutFilePC),
		pShowImg(pShowImg), bSaveImg(bSaveImg), nThreadIndex(nThreadIndex), pBisExit(pBisExit), bSavePointCloud(bSavePointCloud), pImgProcess(pImgProcess), eFlipMode(eFlipMode), eScanMode(eScanMode),
		cstrUnitName(cstrUnitName), tCamParam(tCamParam), cAbsCoorsTrans(cAbsCoorsTrans), cstrImgSavePath(cstrImgSavePath), pSavePointCloudCount(pSavePointCloudCount), nNumOfSavePointThreads(nNumOfSavePointThreads) {}
};

//ImgProcDis中的子线程ThreadAsyn...将数据插入m_pMapImgProcOutput中的函数的参数
struct T_IMAGE_PROCINPUT_PARAM
{
	map<int, T_THREAD_DYNAMICSCAN_OUT>* pMap;
	pair<int, T_THREAD_DYNAMICSCAN_OUT> pairInput;

	T_IMAGE_PROCINPUT_PARAM(map<int, T_THREAD_DYNAMICSCAN_OUT>* param1, pair<int, T_THREAD_DYNAMICSCAN_OUT> param2)
		:pMap(param1), pairInput(param2) {}
};

struct TrackRawData
{
	int nImageNo;
	CvPoint tKeyPtn2D;
	T_ROBOT_COORS tWeldLineWorldCoord;
	T_ROBOT_COORS tCapRobotCoord;
	T_ANGLE_PULSE tCapRobotPulse;
};

//移植隔板类使用的数据模型

//class TraceModel {
//public:

struct TraceModel
{
	/*=============原先放置在RobotDriver的类型==============*/

	/*=========InitWeldStartVal=========*/
	DOUBLE m_dWeldLen;								//工件总长度
	DOUBLE m_dCurrentWeldLen;						//当前焊接长度
	DOUBLE m_dChangeAngleTheshold;					//变姿态距离
	DOUBLE m_dScanStartOffset;						//起点处留出多长不焊接
	DOUBLE m_dWeldEndPointOffset;					//终点处留出多长不焊接
	DOUBLE m_dDisSafeGunToEnd;						//过水孔15或三面相交5焊接终点处预留长度
	DOUBLE m_BoardThick;							//选择工艺参数使用的板厚
	DOUBLE m_dWeldVelocity;							//焊接速度
	DOUBLE m_dGunToEyeCompenX;						//补偿量
	DOUBLE m_dGunToEyeCompenZ;						//补偿量
	DOUBLE m_dFlatHorComp = 0.0;

	BOOL m_bUpdatedWarpData;						//更新包角数据
	BOOL m_bIsTrackingScanEnd;						//跟踪过程搜索结尾位置
	BOOL m_bIsCloseTrackThread;						//是否关闭跟踪线程
	BOOL m_bIfOpenExternal;							//是否启用外部轴
	BOOL m_bIfWeldToEndPoint;
	BOOL m_bCameraFindWeldEnd;

	INT m_nOpenTrackingPos;							//开启跟踪位置，用于变姿态后跟踪
	INT m_nCloseTrackingPos;						//关闭跟踪位置，用于变姿态后跟踪

	E_WRAPANGLE_TYPE m_eStartWrapAngleType;			//包角类型

	/*===合流程新增===*/
	INT m_nWeldSeamType;							//焊缝类型
	/*===合流程新增===*/

	T_WELD_PARA m_tWeldParam;
	/*=========InitWeldStartVal=========*/


	/*=========CornerLineWeldFunNew=========*/
	vector<vector<T_ROBOT_COORS>> m_vvtWarpCorrectCoors;	// 包角修正特征坐标
	vector<vector<T_ROBOT_COORS>> m_vvtTrackWarpCoors;
	vector<vector<INT>> m_vvnTrackWarpPtType;
	/*=========CornerLineWeldFunNew=========*/


	/*=========GetTheoryReferencePulse=========*/
	vector<T_ANGLE_PULSE> m_vtTheoryMeasurePointLeft;
	/*=========GetTheoryReferencePulse=========*/


	/*=========DoWeld=========*/
	vector<T_ROBOT_COORS>	m_vtRealEndpointCoor;			//真实的结尾点
	vector<int>				m_vtEndPointType;				//结尾点类型
	/*=========DoWeld=========*/


	/*=========WeldProcessBefor=========*/
	DOUBLE m_dRecordBreakPointRZ;							//停弧时Rz变化量
	INT m_nWeldStepNo;										//记录焊接步号，用于先测后焊断续焊接
	/*=========WeldProcessBefor=========*/


	/*=========DoweldRuning=========*/
	vector<TrackFilter_XYZ> m_vtRecordWarpBeforWeldData;	//记录包角前焊接数据
	vector<T_ROBOT_COORS> m_vtBreakContinueBeforPoints;		//断续焊接时存储已焊接数据
	vector<INT> m_vtBreakContinueBeforPointType;			//断续焊接时存储已焊接数据
	vector<INT> m_vtWeldLinePointType;						//焊接数据类型
	BOOL m_bSendWeldDataFinish;								//数据放入全局向量结束
	BOOL m_bSendSafeFlag;									//发送结尾标志位
	BOOL m_bIsTrackingDataSendOver;
	DOUBLE m_dMoveStepDis;									//焊接移动步距,滤波用
	DOUBLE m_dAdjustRobotRZ;
	DOUBLE m_dPreRzChange;
	long m_lPreAdjustRzTimeTick = 0;						// 记录每次调整时间戳
	T_ROBOT_COORS m_tRealEndpointCoor;						//真实的结尾点
	T_ROBOT_COORS m_tRobotCurentCoor;						//机器人及大车当前坐标
	/*=========DoweldRuning=========*/


	/*=========CorrectWarpDataFun=========*/
	//E_WRAPANGLE_TYPE m_eStartWrapAngleType;					//包角类型
	/*=========CorrectWarpDataFun=========*/


	/*=============原先放置在RobotDriver的类型==============*/
	bool m_bIfJudgeEndOpen;// 开启结尾判断
	E_WORKPIECE_TYPE m_tWorkPieceType;// 工件类型,并非实际选择类型，一般用于区分是否为闭合工件
	CStepMoveControlJY      m_cStepMoveControl;//连续插补对象
	int m_nAXIS_X = 0;
	int m_nAXIS_Y = 1;

	// 多线程存图标志
	bool m_bTrackSaveImage;
	// 手眼长度
	double dHandEyeDis;
// 板厚，跳枪用
	double dBoardThink = 10.0;

	// 实时跟踪参考线：锁定输出传入第一次跟踪  上一次跟踪输出传入下一次跟踪
	XiLineParamNode tFrontLine, tBackLine;

	// 离线调试使用
	std::vector<TrackRawData> vtTrackRawData; // 跟踪采集原始数据

	// 局部点云数据信息
	std::list<int> lnImageNo;	// 当前点云集合中每张图编号
	std::list<int> lnImagePtnNum;	// 当前点云集合中每张图点数
	std::list<Three_DPoint> ltPointCloud; // 当前点云集合
	std::list<T_ROBOT_COORS> ltCapWorldCoord; // 当前点云集合中每张图采集时世界坐标
	std::vector<Three_DPoint> vtPointCloudEndPoints; // 每次点云处理提取的端点世界坐标
	std::vector<T_ROBOT_COORS> vtConnerWorldCoords; // 每次处理角点世界坐标
	FILE *pfPointCloud; // 点云文件
	FILE *pfImgCalcData; // 每张图片采图坐标等计算数据保存
};

class CScanInitModule  
{

public:
	CScanInitModule(CRobotDriverAdaptor* pRobotDriver, CUnit* ptUnit, UINT* pUnTracePointCloudProcess = NULL,
		BOOL* pbSaveScanImg = NULL, E_FLIP_MODE* peScanFlipMode = NULL, E_SCANMODE* peScanMode = NULL, UINT* pUnSingleCameraThrdNum = NULL);
	virtual ~CScanInitModule();

public:
	void          SetParam(double dXStart, double dYStart, double dXEnd, double dYEnd, E_SCAN_MODE eScanMode);
	void		  SetScanStartParam(double dStartUAngle, XI_POINT startPoint, int nStartPointTyoe, 
									int nStartShape, BOOL bIsFixedScanLen,BOOL bStartIfWrap);
	void		  TranslateDrawingToImageParam(); // 图纸数据转换为图像处理所需参数
	void		  ResumeDrawingToImageParam(); // 恢复图纸数据转换为图像处理所需参数
	BOOL          IsComplete();

	void		  SetScanParam(/*bool bSavePointCloud, bool bSaveScanImg,*/ bool bNaturalPop, E_FLIP_MODE eFlipMode, E_SCANMODE eScanMode, CString sMoveJobName);
private:
	void          Init(UINT* pUnTracePointCloudProcess = NULL, BOOL* pbSaveScanImg = NULL,
		E_FLIP_MODE* peScanFlipMode = NULL, E_SCANMODE* peScanMode = NULL, UINT* pUnSingleCameraThrdNum = NULL);
	void          UnInit();



	// 原始代码是在粗搜索之后就进行信息的更新
	void		  InitStartPointInfoArray();
	void		  ClearStartPointInfoArray();

    
public:
	BOOL          UpdateTheoryInfoAccurate(CRobotDriverAdaptor *pRobotDriver);
	T_LINE_PARA m_ScantLineParam;

	double			 m_ScandTotalVal;
	double			 m_ScandMachineVal;
	double			 m_ScandRobotValX;
	double			 m_ScandRobotValZ;
    BOOL             m_bIfOpenExternal;
	//------------------------------------------------------------------------------------------


public:
	BOOL		m_bIfLeftDown;
	BOOL		m_bIfRightDown;
	std::vector<XI_POINT> m_vtScanInitWeldLineInWorldPoints;		//记录起始段扫描点
	std::vector<XI_POINT> m_vtScanStartInWorldPoints;				//记录起始段扫描点
	std::vector<XI_POINT> m_vtScanInitWeldLineInBase;				//记录起始段扫描点
	double		m_dWorkpieceThickness;								//工件厚度
    double m_dRealWeldStartPosX;
    double m_dRealWeldStartPosY;
    double m_dRealWeldStartPosZ;


	CMoveCtrlModule          m_MoveCtrlModule;

	// 外部读入初始化变量
	double m_dStepDis;
	double m_dCoarseScanVel;
	double m_dAccurateScanVel;
	int    m_nTheoryNum;

	double m_dStartUAngle;											// 起点位置对应的U轴角度
	XI_POINT m_tStartPoint;
	int m_nStartPointType;
	int m_nStartPointShape;
	BOOL m_nStartIfWrap;
	BOOL m_bFixedPointScan;											//是否定长
private:
	double m_dXStart;
	double m_dYStart;
	double m_dXEnd;
	double m_dYEnd;

	// 计算理论轨迹变量
	std::vector <long>   m_vlXRelPulse;
	std::vector <long>   m_vlYRelPulse;
	std::vector <double> m_vdStepVel;

	std::vector<T_POINT_3D_DOUBLE>    m_vtSearchStartPoint3d;
	double                            m_ptSearchStartPointHead3dX[101];
	double                            m_ptSearchStartPointHead3dY[101];
	double                            m_ptSearchStartPointHead3dZ[101];
	double                            m_ptSearchStartPointTail3dX[101];
	double                            m_ptSearchStartPointTail3dY[101];
	double                            m_ptSearchStartPointTail3dZ[101];
	int                               m_nStartPointFilterNum;
	double                            m_dStartPointDelRatio;

	E_SCANWORK_THREAD_STATUS m_eThreadStatus;
	E_SCAN_MODE              m_eScanMode;

	COPini                   m_cOpini;

	// 图像使用参数

    GROUP_STAND_DIP_NS::E_WHICH_PIECE_CLASS m_ePieceClass;

	int m_iBaseLineLenInZone;
	int m_iMoveLineLenInZone;
	int m_iRoiWidSize;
	int m_iIfEndConditionIdx;
	double m_dScanStartExtendLength; // 过滤用:正常扫描长度98mm基础上的扩展长度(例如过水孔需要30mm变姿态后再跟踪则扩展长度为30mm)
	
	// 是否需要扫描立板两条线 并 用算法计算准确起点
	BOOL m_bScanStartTwoLineOnStandBoard;

public:
	double m_dScanStartWrapLength;	// 正常扫描长度98mm基础上的扩展长度(例如包角长度)
    typedef struct
    {
        CRobotDriverAdaptor *pRobotDriver;
        CScanInitModule *pcMyObj;
    }T_ROBOT;
	//-------------------------起始点采图处理-----------------------
	void                       FixedStartProc(CRobotDriverAdaptor *pRobotDriver);
	T_START_POINT_INFO        *m_ptStartPointInfo;
	//--------------------------机械臂彩图测试------------------------------

	void                      CoordinatePointProcessFuncation(XI_POINT tWeldLinepointCoor,std::string FileNameCin,std::string FileNameCout);//坐标点处理函数
	void                      SwapVector(XI_POINT startPoint, std::string FileNameCin,std::string FileNameCout);


    //-------------------动态扫描端点及跟踪-开始---------------------

    /*跟踪过程：
    **1、DynamicScanningEndpoint（）：扫描初始段数据结果写入文件，在开始跟踪前将数据加载到缓存向量：pRobotCtrl->m_vtWeldLineInWorldPoints 内使用GetScanData()函数。
    **2、开始跟踪前使用DoweldRuning(）向下位机job缓存内发送一段数据，在发送完初始段数据后会立即开始运行job和跟踪。
    **3、ThreadRealTimeTracking():跟踪函数在跟踪过程中弧获取位置和姿态，位置实时写入缓存，姿态根据初始姿态差值进行调整
    **4、跟踪过程中会调用结尾判断函数，用以结束判断。
    **初步考虑大车和机械臂联动插补使用同一个Job。
    **焊接数据（先测后焊或实时跟踪）存储在pRobotCtrl->m_vtWeldLineInWorldPoints向量内，该
    **向量存储实时位置和初始焊接姿态，跟踪变化的姿态写入p99位置变量累加在每个位姿上，用以实现实时调整姿态。
    **先测后焊该向量存储实际焊接位姿。
    **注：端点扫描使用同一个端点扫描函数DynamicScanningEndpoint（），会得到扫描段数据和端点数据。
    */
	

	/*一次流程内视觉调用顺序：
	1 .InitIfStartOrEndPntParam(GROUP_STAND_DIP_NS::E_PIECE_START)
	1 .GetGroupStandKeyPoint():锁定
	1 .IfPieceStartOrEndPoint（）：搜端点
	1 .InitIfStartOrEndPntParam(GROUP_STAND_DIP_NS::E_PIECE_START)
	1 .GetGroupStandKeyPoint():锁定
	1 .IfPieceStartOrEndPoint（）：搜端点
	1 .GetGroupStandKeyPoint():锁定
	1 .GetGroupStandKeyPoint():跟踪
	*/	
public:
/*	****************************函	数****************************	*/
	static UINT             ThreadDynamicScanningCapture(void* pParam);
	static UINT             ThreadDynamicScanningProc(void* pParam);
	//图像翻转
	void					FlipImageNew(CRobotDriverAdaptor* pRobotCtrl);
	BOOL					DynamicRealTimeTracking();
	//跟踪线程
	static UINT             ThreadRealTimeTracking(void* pParam);
	//图片翻转线程
	static UINT				ThreadFlipImage(void* pParam);
	//跟踪起弧前锁定，需要调整到正常跟踪姿态下进行锁定
	BOOL                    RealTimeTrackLockArcBefor(CRobotDriverAdaptor* pRobotDriver, BOOL bIsContinueWeld = FALSE);
	//跟踪前锁定，需要调整到正常跟踪姿态下进行锁定,该函数可在起弧后进行锁定
	BOOL                    RealTimeTrackLockArcAfter(CRobotDriverAdaptor* pRobotDriver);
	void                    RealTimeTracking(CRobotDriverAdaptor* pRobotCtrl);

	// 运动 跟踪处理 终点判断 包角
	/**************************** 跟踪优化添加 Start ****************************/

	static UINT ThreadRealTimeTrackCapture_H(void* pParam);
	void FuncRealTimeTrackCapture_H(CRobotDriverAdaptor* pRobotDriver, int nCameraNo, int nProcThreadNum, int nMaxImgBufSize = 50, int nTimeOut = 2000); // 点云方式：端点搜索图片采集和处理
	// 实时跟踪线程函数(高帧率)
	void RealTimeTrackingNew_H(CRobotDriverAdaptor* pRobotCtrl);
	// 跟踪高帧率多线程处理 获取最新的焊缝世界坐标：无点云结果 使用角点结果 没有最新结果记录失败次数 超过最大阈值返回false 否则计数清零
	bool GetTrackWeldLineWorldCoord(T_ROBOT_COORS &tWeldLineWorldCoord, int &nConnerRstNum, int &nPointCloudRstNum, int &nFailNum, int nMaxFailNum, FILE* pfFile);

	// 跟踪初始化 配合RealTimeTrackingNew使用
	void InitRealTimeTracking(E_JUDGE_END_METHOD eJudgeEndMethod, T_ROBOT_COORS* pEndCoord = NULL);
	// 实时跟踪线程函数
	void RealTimeTrackingNew(CRobotDriverAdaptor* pRobotCtrl);

	// 读取跟踪采图坐标 参数nImageNo用于离线调试时获取采图坐标
	void ReadTrackCapCoord(CRobotDriverAdaptor* pRobotCtrl, T_ROBOT_COORS& tRobotCoord, T_ANGLE_PULSE& tRobotPulse, T_ROBOT_COORS& tWorldCoord, int nImageNo = 0);

	// 图片采集 图片序号nImageNo递增后作为新图片的索引, 调试模式 读取本地图片
	bool CaptureTrackImage(int nCameraNo, int &nImageNo, IplImage** pImageBuff, bool bSaveImage, CString sSavePath);
	bool ImageLockProcess(CImageProcess* pTraceImgProcess, IplImage** ppImage, XiLineParamNode &tFrontLine, XiLineParamNode &tBackLine, E_FLIP_MODE eFlipMode = E_FLIP_NONE);

	// 跟踪图像处理 返回是否处理成功 连续多次停止跟踪 正常复位累计计数 计算焊缝机器人坐标和焊缝机器人世界坐标
	bool TrackProcess(CRobotDriverAdaptor* pRobotCtrl, int nImageNo, IplImage** pImageBuff, T_ROBOT_COORS tRobotCoord, T_ANGLE_PULSE tRobotPulse, 
		T_ROBOT_COORS& tWeldLineRobotCoord, T_ROBOT_COORS& tWeldLineWorldCoord, int& nFailNum, int nMaxFailNum, FILE* pfFile, FILE* pfPointCloud);

	// 检测相邻两次处理是否过近 连续多次过近停止跟踪 正常复位累计计数 并 更新最后跟踪采图位置tLastCapWorldCoord 更新累计焊接长度
	bool CheckAdjoinCapCoord(CRobotDriverAdaptor* pRobotCtrl, int nImageNo, T_ROBOT_COORS& tLastCapWorldCoord, T_ROBOT_COORS tCurCapWorldCoord, double dDisThreshold, int& nErrNum, int nMaxErrNum);
	
	// 检测滤波是否正常 连续多次滤波失败停止跟踪 正常复位累计计数 输入最新焊缝坐标tInputCoord 输出滤波结果vtCoutTrackPoint 滤波输入数据存入文件pfFile
	bool CheckTrackFilterValid(CRobotDriverAdaptor* pRobotCtrl, int nImageNo, T_ROBOT_COORS tInputCoord, std::vector<TrackFilter_Node>& vtCoutTrackPoint, int& nInvalidNum, int nMaxInvalidNum);

	// 滤波输出数据处理 连续多次调整Z值失败 停止跟踪 正常复位累计计数 将vtCoutTrackPoint中的每个点添加补偿 写入文件 并 追加到跟踪焊接轨迹缓冲区
	bool FilterResultTrans(CRobotDriverAdaptor* pRobotCtrl, int nImageNo, const std::vector<TrackFilter_Node>& vtCoutTrackPoint, int& nAdjustErrNum, int nMaxAdjustErrNum, FILE* pfFile);

	// 函数功能：将tSplitCoord世界坐标XYZ拆分到外部轴
	// tRefCoord：参考坐标 x y z rx ry rz bx by bz 取bFixExAxis对应轴的值作为固定值使用
	// tSplitCoord：要拆分的坐标 输入x y z rx ry rz 输出x y z rx ry rz bx by bz
	// nSplitAxisNo：拆分的轴号 与 m_ptUnit->m_nTrackAxisNo 定义相同：123表示xyz 其他表示外部轴固定
	void SplitCoord(T_ROBOT_COORS tRefCoord, T_ROBOT_COORS& tSplitCoord, int nSplitAxisNo);

	// 终点判断：判断方法nMethod: 1.已知终点(干涉终点) 2.用理论长度(留焊) 3.焊接过程中判断(自由终点)
	// 发现终点后：更新终点轨迹 并 添加包角轨迹 设置m_pTraceModel->m_bCameraFindWeldEnd为true
	bool JudgeFindWeldLineEnd(CRobotDriverAdaptor *pRobotCtrl, int nMethod);
	bool JudgeEnd_Know(const std::vector<Three_DPoint>& vtPtns, T_ROBOT_COORS tTheoryEndCoord); // 已知终点
	bool JudgeEnd_TheoryLength(CRobotDriverAdaptor* pRobotCtrl); // 用理论长度
	bool JudgeEnd_ProcessSearch(CRobotDriverAdaptor* pRobotCtrl); // 焊接过程中搜索判断

	static UINT ThreadJudgeEnd_ProcessSearch(void* pParam);
	bool FuncJudgeEnd_ProcessSearch(CRobotDriverAdaptor* pRobotCtrl, int nTimeOut = 5000); // 等待 nTimeOut ms 未运动则超时
	bool GetEndSectionCoorsNew(CRobotDriverAdaptor* pRobotDriver, T_ROBOT_COORS tEndpointCoor);
	bool m_IsOpenJudgeEnd_ProcessSearch;
	Mutex m_MutexTrackPointCloud; // 跟踪处理 和 终点判断时 互斥访问

	// 保存局部点云处理失败或异常数据
	void SavePointCloudProcErrData(CString sFileName, int nImgNoS, int nImgNoE, CvPoint3D32f tCameraNorm, CvPoint3D32f 
		tPlaneHNorm, CvPoint3D32f* tRefPtn, int tRefPtnNum, Three_DPoint* ptPointCloud, int nPointCloudNum);

	// 终点判断
	bool JudgeFindEndPoint(std::vector<Three_DPoint>& vtPtns, double dDisThreshold, double nPtnNumThreshold);
	/**************************** 跟踪优化添加 End ****************************/

	/**************************** 端点搜索优化添加 start ****************************/
	//动态采图 单线程采图处理 需要使用InitDynamicCapture_H_M先初始化再调用
	void DynamicCaptureNew(CRobotDriverAdaptor* pRobotDriver, int nCameraNo, CString sPointCloudFileName,int nTimeOut = 5000); // 点云方式：端点搜索图片采集和处理
	//全扫描动态采图
	void DynamicCaptureNew_ScanWeldLine(CRobotDriverAdaptor* pRobotDriver, int nCameraNo, CString sPointCloudFileName, int nTimeOut = 5000);
	// 提取图片激光中心点
	bool FuncImageProcessInTrack_Lock(IplImage* pImg, XiLineParamNode& tFrontLine, XiLineParamNode& tBackLine, std::vector<CvPoint>& vtPoint2D, E_FLIP_MODE eFlipMode);
	bool FuncImageProcessInTrack(IplImage* pImg, XiLineParamNode& tFrontLine, XiLineParamNode& tBackLine, std::vector<CvPoint>& vtPoint2D, E_FLIP_MODE eFlipMode);
	bool FuncImageProcessEEEEE(IplImage* pImg, std::vector<CvPoint>& vtPoint2D);

	void DynamicCaptureNew_H(CRobotDriverAdaptor* pRobotDriver, int nCameraNo, int nProcThreadNum, int nMaxImgBufSize = 50, int nTimeOut = 5000); // 点云方式：端点搜索图片采集和处理
	static UINT ThreadImageProcess(void* pParam);
	bool FuncImageProcess(CRobotDriverAdaptor* pRobotCtrl);
	bool m_IsOpenImageProcess;
	T_CAP_IMAGE_DATA* m_pCapImageData; // DynamicCaptureNew_H中创建

	void DrawAndShowImg(IplImage** ppSrc, IplImage** ppShowImg, const CvPoint& tPtn2D, const XiLineParamNode& tFrontLine, const XiLineParamNode& tBackLine);

	/**************************** 端点搜索优化添加 End ****************************/

	/**************************** 扫描点云添加 Start ****************************/

	// 直立板扫描	EEEEEInTrack	GetRiserEndPoint			非/动态处理		输出：直缝焊缝轨迹及端点坐标
	// 立焊扫描		EEEEE			GetRiserEndPoint			非动态处理		输出：立焊起终点坐标
	// 箱型梁坡口	EEEEE			GrooveSolutionInterface_	非/动态处理		输出：坡口多截面坐标
	
	// 运动采图 生成点云 并提取关键点
	// 可实时处理点云 可扫描完整后处理点云
	// 可指定不同提取激光中心点方式
	// 可指定不同的点云提取方式
	// 可指定结果保存形式
	// 使用前必须使用 InitDynamicCapture_H_M 初始化	
	bool InitDynamicCapture_H_M(
		E_POINT_CLOUD_PROC_MOMENT ePointCloudProcMoment, E_IMAGE_PROC_METHOD eImgProcMethod,
		E_POINT_CLOUD_PROC_METHOD ePointCloudProcMethod, int nCamNo, CString sSaveFilePath, CString sPointCloudFileName,
		double dSavePointCloudLen, E_WELD_SEAM_TYPE eWeldSeamType, int nGroupNo, int nLayerNo, bool bInsideCallJob = true, 
		bool bJudgeEndPoint = false, bool bSaveImage = false, int nSaveImgStepNo = 2, int nPtn2DStepNo = 2);
	void DynamicCaptureNew_H_M(CRobotDriverAdaptor* pRobotDriver, int nCameraNo, int nProcThreadNum, int nMaxImgBufSize = 50, int nTimeOut = 2000); // 点云方式：端点搜索图片采集和处理
	CString m_sSaveFilePath;
	CString m_sGroovePointCloudFileName;
	double m_dSavePointCloudLen; // 保留最多少张图片的点云
	T_CAP_IMAGE_DATA* m_pCapImageData_H_M; // DynamicCaptureNew_H_M中创建
	E_POINT_CLOUD_PROC_MOMENT m_ePointCloudProcMoment = E_POINT_CLOUD_PROC_MOMENT_DYNAMIC;
	E_IMAGE_PROC_METHOD m_eImgProcMethod = E_IMAGE_PROC_METHOD_EEEEE;
	E_POINT_CLOUD_PROC_METHOD m_ePointCloudProcMethod = E_POINT_CLOUD_PROC_METHOD_GetRiserEndPoint;
	E_JUDGE_END_METHOD m_eJudgeEndMethod = E_JUDGE_END_PROCESS_SEARCH; // 终点判断方法
	bool m_bDynamicJudgeEndPoint = false; // 是否扫描过程动态终点判断
	bool m_bInsideCallJob = true;

	// 图片处理
	static UINT ThreadImageProcess_H_M(void* pParam);
	bool FuncImageProcess_H_M(CRobotDriverAdaptor* pRobotCtrl);
	bool FuncImageProcess_H_M_EEEEE(CRobotDriverAdaptor* pRobotCtrl);
	bool FuncImageProcess_H_M_EEEEEInTrack(CRobotDriverAdaptor* pRobotCtrl);
	bool FuncImageProcess_H_M_DoubleVGroove(CRobotDriverAdaptor* pRobotCtrl);
	bool m_IsOpenImageProcess_H_M = false;
	int m_nCameraNo = -1;
	bool m_bSaveImg = false;
	int m_nSaveImgStepNo = 2;
	int m_nPtn2DStepDis = 2;
	E_WELD_SEAM_TYPE m_eWeldSeamType = E_PLAT_GROOVE;
	int m_nGroupNo;
	int m_nLayerNo;

	// 点云处理
	static UINT ThreadPointCloudProcess_H_M(void* pParam);
	bool FuncPointCloudProcess_H_M(CRobotDriverAdaptor* pRobotCtrl, int nTimeOut = 2000); // 等待 nTimeOut ms 未运动则超时
	bool FuncPointCloudProcess_H_M_GetRiser(CRobotDriverAdaptor* pRobotCtrl, int nTimeOut = 2000);
	bool FuncPointCloudProcess_H_M_Groove(CRobotDriverAdaptor* pRobotCtrl, int nTimeOut = 2000);
	bool FuncPointCloudProcess_H_M_DoubleVGroove(CRobotDriverAdaptor* pRobotCtrl, int nTimeOut = 2000);
	bool m_IsOpenPointCloudProcess_H_M;
	Mutex m_Mutex_H_M; // 图片处理 和 点云处理 互斥访问

	// 点云提取结果保存
	bool FinalProc_Groove();
	bool LoadTeachResult(int nGroupNo, int nLayerNo, std::vector<T_TEACH_RESULT>& vtTeachResult);

	//void SaveGrooveInfoToFile(FILE *pf, const GroovePointInfo &tGrooveInfo, bool bSaveBoth); // bSaveBoth :true起点截面+终点截面 false:只保存终点截面
	//void SaveGrooveInfoToFile(FILE *pf, const GrooveCloudExtPointInfo&tGrooveInfo, bool bSaveBoth); // bSaveBoth :true起点截面+终点截面 false:只保存终点截面
	//void SaveGrooveInfoToFile(FILE *pf, const GrooveCloudExtPointInfoM&tGrooveInfo, bool bSaveBoth); // bSaveBoth :true起点截面+终点截面 false:只保存终点截面
	void SavePtn3D(FILE *pf, CvPoint3D32f tPtn);
	void AppendGrooveInfoPtn(std::vector<Three_DPoint>& vtThree_Points, const CvPoint3D32f& tPtn);
	/**************************** 扫描点云添加 End ****************************/

	// 单点测量作为端点（结果为世界坐标）
	BOOL					SinglePointMeasure(CRobotDriverAdaptor* pRobotCtrl);
	//搜索端点前初始化,两次搜索前只初始化一次
	BOOL					ScanEndpointLockInit(CRobotDriverAdaptor* pRobotDriver);
	//搜索端点
	BOOL                    DynamicScanningEndpoint(CRobotDriverAdaptor* pRobotDriver);
	//动态采图
	void                    DynamicCapture(CRobotDriverAdaptor* pRobotDriver);
	//处理
	void                    DynamicEndPointProc(CRobotDriverAdaptor* pRobotDriver, BOOL bSaveImg = FALSE);
	//搜端点调度函数
	void					ImgProcDistribute(CRobotDriverAdaptor* pRobotDriver, unsigned int nMaxThreadNum = 3, const int& nScanSensitivity = 5,
											  const unsigned int& nThreshold = 0, const int& nPicStrIndex = 3, const int& nWaitOutTime = 5000,
											  const bool& bSaveImg = false, const bool& bSavePointCloud = false, E_START_OR_END ePtType = E_PIECE_END,
											  const E_FLIP_MODE& eFlipMode = E_FLIP_NONE, const E_SCANMODE& eScanMode = E_SCANMODE_2DPOINT,
											  const CString cstrJobName = "DYNAMICSCANNING", const unsigned int nJudgeRobotStopTime = 500,
											  const unsigned int nNumOfSavePCThreadsInSingleProcThrd = 1);
	//多线程存点云
	static UINT				ThreadSavePointCloud(LPVOID pParam);
	//搜端点处理函数（多线程）
	static UINT				ThreadAsynDynamicScanEndPointProc(LPVOID pParam);
	//扫描结果处理
	BOOL                    DynamicEndPointResultPro(CRobotDriverAdaptor* pRobotDriver);
	BOOL					StartDataPointPro(CRobotDriverAdaptor* pRobotDriver, std::vector<T_ROBOT_COORS>& vtStartCoors, BOOL bIsTracking = FALSE,double dTheoryLength = 0.0,bool bFirstFit = true);
	BOOL					DataPointsProcess(CRobotDriverAdaptor* pRobotDriver, std::vector<XI_POINT> vtCinPoints, std::vector<XI_POINT>& vtCoutPoints, vector<T_ROBOT_COORS>& vtCoutRobotCoors);
	BOOL                    GetScanData(CRobotDriverAdaptor* pRobotDriver, std::vector<XI_POINT>& tScanData, std::vector<T_ROBOT_COORS>& tScanRobotData, double& dDirAngle);
	static BOOL             CompDoubleFunc(double a, double b);
	//检测机器人急停
	void					CheckMachineEmg(CRobotDriverAdaptor* pRobotDriver);
	//结尾点判断
	void					CallJudgeEndpointFun();
	static UINT				ThreadJudgeIsWeldCompen(void* pParam);
	BOOL					JudgeIsWeldCompen(CRobotDriverAdaptor* pRobotDriver, T_ROBOT_COORS tEndpointCoor);
	//插入结尾数据
	BOOL				    GetEndSectionCoors(CRobotDriverAdaptor* pRobotDriver, T_ROBOT_COORS& tEndpointCoor);
	//记录时间
	double					RecordRunTime(CRobotDriverAdaptor* pRobotDriver, long ltimeBegin, CString str);
	static UINT             ThreadJudgeEndProcLineGroupStand(void* pParam);
	static UINT             ThreadJudgeEndCaptureRobot(void* pParam);
	void                    JudgeEndCaptureRobot(CRobotDriverAdaptor* pRobotDriver);
	void                    JudgeEndProcLineGroupStand(CRobotDriverAdaptor* pRobotCtrl);
	BOOL                    WeldRealEndPointSingle(CRobotDriverAdaptor* pRobotDriver);
	T_ROBOT_COORS           StartWrapAngleFun(CRobotDriverAdaptor* pRobotDriver, double dWorkPieceDir, T_ROBOT_COORS tRobotEndPoint);
	BOOL                    EndSafeBackFun(CRobotDriverAdaptor* pRobotDriver, double dWorkPieceDir, T_ROBOT_COORS tRobotEndPoint);
	BOOL                    EndWrapAngleFun(CRobotDriverAdaptor* pRobotDriver, double dWorkPieceDir, T_ROBOT_COORS tRobotEndPoint);
	void                    CalcNormalOffsetInBase(T_ROBOT_COORS& tRobotWrapData, double dDis, double dZDis, double dAngleVer);
	BOOL                    CompareCoordValue(CRobotDriverAdaptor* pRobotDriver, int nPIndex, T_ROBOT_COORS tRobotCoor, double nErrorVal);
	T_ROBOT_COORS           CalcWrapOffsetFun(double dWrapdParalle, double dVertical, double dWorkPieceDir, T_ROBOT_COORS tRobotEndPoint, int nRobotInsterDir);
	double                  ControlRobotAngle(double dAngle);
	void					TranslateDrawingToImageParam(CRobotDriverAdaptor* pRobotDriver);
	void                    Initvariable();
	void					LoadEquipmentParam();
	double					DirAngleToRz(double dDirAngle);
	static double			SmoothRz(double dRz, std::vector<double>& vdRecordRz, int nRecordNum = 3);
	BOOL					JudgeFilterDataValid(CRobotDriverAdaptor* pRobotDriver, CvPoint3D64f tNewPoint, T_ROBOT_COORS tLastValidPoint, T_ROBOT_COORS tPenultPoint);
	void					SetWeldParam(double dHandEyeDis, double dMeasureDisThreshold, double dPlatWeldRx, double dPlatWeldRy,
										 double dStandWeldRx, double dStandWeldRy, double dTransitionsRx, double dTransitionsRy,
										 double dWeldNorAngleInHome, BOOL* pIsNaturalPop, int nRobotInsterDir = 1);
	//多线图像处理互斥函数,参数结构体为 T_IMAGE_PROCINPUT_PARAM
	static void				ImgProcInsertPairInMap(LPVOID pParam);
	static void				UsSleep(const unsigned int& nUs);
	static XI_POINT			Camera2DTo3D(CvPoint tCamera2DPoint, T_CAMREA_PARAM tCamParam, XiCV_Image_Processing* pXiCvProcess=NULL);
	static T_ROBOT_COORS	TransCameraToRobot(XI_POINT tCameraCoor, T_CAMREA_PARAM tCamParam, GROUP_ROBOT_ABS_COORS_TRANS::XiRobotAbsCoorsTrans cAbsCoorsTrans, E_MANIPULATOR_TYPE eManipType, T_ROBOT_COORS tCaptureRobotCoor, T_ANGLE_PULSE tCaptureRobotPulse);
	static T_ROBOT_COORS	TransCamera2DToBase(CvPoint tCamera2DPoint, T_CAMREA_PARAM tCamParm, GROUP_ROBOT_ABS_COORS_TRANS::XiRobotAbsCoorsTrans cAbsCoorsTrans, E_MANIPULATOR_TYPE eManipType, T_ROBOT_COORS tCaptureRobotCoor, T_ANGLE_PULSE tCaptureRobotPulse, T_ABS_POS_IN_BASE& tPointAbsCoordInBase,XiCV_Image_Processing* pXiCvProcess=NULL);

	void					LoadIsTrackCompleteFlag(CRobotDriverAdaptor* pRobotCtrl, int& TrackFlag);
	void					SaveIsTrackCompleteFlag(CRobotDriverAdaptor* pRobotCtrl, int TrackFlag,double dVoerLength);

	void					SaveEndpointData(CString RobotName,int nGroupNo, int nEndpointNo,XI_POINT tRealEndpoint, bool bShape = false);
	// 跟踪多层多道时设置参数,仅支持底板非窄边焊道
	void					SetTrackProcessParam(int nLayerNo);

	/********************	   Get函数		********************/

	//取搜端点处理三维点点集
	vector<XI_POINT> GetVtImgProcResult3D() {
		return m_vtImgProc3DPoint;
	}

	//获取扫描点云文件名称（临时使用，需改为直接获取点云）
	CString GetPointCloudFileName()
	{
		return m_sPointCloudFileName;
	}

	//取搜端点的端点结果
	XI_POINT GetDynamicScanEndPoint() {
		return m_tDynamicScanEndPoint;
	}

	vector<XI_POINT> GetScanPointCloud() {
		return m_vtScanPointCloud;
	}
	/********************	   Get函数		********************/
	/********************	   Set函数		********************/
	VOID SetDynamicScanEndPoint(DOUBLE x,DOUBLE y,DOUBLE z) {
		m_tDynamicScanEndPoint.x = x;
		m_tDynamicScanEndPoint.y = y;
		m_tDynamicScanEndPoint.z = z;
	}
	VOID SetDynamicScanEndPoint(const XI_POINT& obj) {
		m_tDynamicScanEndPoint = obj;
	}

	VOID SetScanFlipMode(E_SCANMODE eScanMode) {
		*m_peScanMode = eScanMode;
	}
	/********************	   Set函数		********************/

/*	****************************函	数****************************	*/

/*	****************************参	数****************************	*/

	CWrapAngleParam* m_pcWrapAngleParam;
	FILE* m_pRecordTheoryPoint;							//存储理论数据
	double m_dWeldInHomeRz;
	int m_nStartPointCapCursor;
	int m_nStartPointProcCursor;
	//找到端点标志位
	BOOL m_bCameraFindWeldStart;
	BOOL m_bIfOpenJudgeStartCap;
	int	m_nTimeStart;
	XI_POINT m_tAlgCalcStartPoint;						// 起点三维扫描算法计算得到的起点
	BOOL m_bStartPointCapCur;							//判断线程是否结束
	BOOL m_bStartPointProcese;
	BOOL m_bScanStartFlag;								//横纵标志位
	XI_POINT m_tWeldLinepointInWorld;
	CRobotDriverAdaptor* m_pRobotDriver;
	IplImage* m_pShowImage;								//主界面显示图像

	/*****	搜端点使用的public参数	*****/
	E_START_OR_END m_eEndPointType;						//扫描端点类型：起点或终点
    int  m_nEndPointCapCursor;							//端点采图序号
    int  m_nEndPointProcCursor;							//处理图号
	int	 m_nScanSensitivity;							//搜索灵敏度
    BOOL m_bEndPointCapCur;								//判断采图线程是否结束 
    BOOL m_bEndPointProcess;							//判断处理线程是否结束
	BOOL m_bSuccessScanFlag;							//成功搜索标志位
    BOOL m_bRealTimeTrackstate;							//跟踪线程状态
	BOOL m_bOpenTrackingStatus;							//跟踪线程是否已开启
	BOOL m_bLaserLightStatus;							//激光状态
    BOOL m_bDynamicCapturestate;						//采图线程状态
    BOOL m_bDynamicProcstate;							//处理线程状态
	BOOL m_bFlipedImage;								//翻转线程状态
	UINT* m_pUnSingleCameraThrdNum;						//单相机处理线程数量
	UINT* m_pUnTracePointCloudProcess;					//点云处理类型 0不存 1存点云 2 存+处理点云
	BOOL* m_pSaveTraceScanPic;							//是否存搜端点原图
	E_FLIP_MODE* m_peScanFlipMode;						//扫描时图像的翻转情况
	E_SCANMODE* m_peScanMode;							//多线程搜索模式
	CString m_sMoveJobName;								//搜索运动Job名称
	/*****	搜端点使用的public参数	*****/

	/*****		焊接参数		*****/
	double m_dHandEyeDis;								// 手眼距离：焊接前进方向距离
	double m_dMeasureDisThreshold;						// 测量点距离焊缝起点终点距离阈值
	double m_dPlatWeldRx;								// 标准测量和焊接姿态Rx
	double m_dPlatWeldRy;								// 标准测量和焊接姿态Ry
	double m_dNormalWeldRx;								// 正常平角焊接姿态Rx,用于测量和焊接姿态不同时
	double m_dNormalWeldRy;								// 正常平角焊接姿态Ry,用于测量和焊接姿态不同时
	double m_dStandWeldRx;								// 标准立焊姿态Rx
	double m_dStandWeldRy;								// 标准立焊姿态Ry
	double m_dTransitionsRx;							// 过度点姿态Rx
	double m_dTransitionsRy;							// 过度点姿态Ry
	double m_dWeldNorAngleInHome;						// 机器人安全位置关节坐标状态 对应的焊缝法相角
	T_ROBOT_COORS m_tRobotHomeCoors;					// 机器人安全位置关节坐标对应的直角坐标(焊枪工具)，构造函数初始化
	bool m_bWorkpieceShape;								// 跟踪焊缝形状 false直线 true圆弧
	/*****		焊接参数		*****/

	// -------------跟踪原流程变量----------------
	TraceModel* m_pTraceModel  = new TraceModel();;//跟踪使用结构体
	//--------------------------------------------
	
/*	****************************参	数****************************	*/
private:
/*	****************************参	数****************************	*/

	BOOL __flag_self_create;		//是否自我构造，判断是否传入存点云指针，存图指针，弹窗指针,若传入表示非自我构造，则该标志位为FALSE

	/********************	多线程锁	********************/
	/*		0 - > 99+ 权限依次下降	*/

	//R0LevelLock		主线程与调度线程 的互斥
	Mutex m_MutexCapThrdExitFlag;									//采图线程结束标志位锁 m_bEndPointCapCur						只能由采图线程将m_bEndPointCapCur置为TRUE
	Mutex m_MutexProcThrdExitFlag;									//处理线程结束标志位锁 m_bEndPointProcess						只能由处理线程将m_bEndPointProcess置为TRUE
	Mutex m_MutexSuccessScanFlag;									//处理线程（新流程）处理是否成功标志位锁 m_bSuccessScanFlag		只能由处理线程修改该标志位

	//R1LevelLock		调度线程间（采图、处理分发线程）的互斥
	Mutex m_MutexFindEndPointFlag;									//找到端点标志位锁 m_bCameraFindWeldStart

	//R2LevelLock		处理分发线程间（调度、处理线程）的互斥
	Mutex m_MutexMapProcOut;										//处理结果Map锁	m_pMapImgProcOutput
	Mutex m_MutexExitFlag;											//处理线程退出标志位锁
	Mutex m_MutexPicCapture;										//采图互斥锁 m_ptStartPointInfo、m_nEndPointCapCursor
	Mutex m_MutexSavePointCloud;									//存點云鎖（文件寫入）
	//信号量
	HANDLE m_hGetCameraCtrl;
	

	/********************	多线程锁	********************/

	vector<BOOL> m_vtImgProcThreadStatus;							//扫描图像处理线程使用情况
	vector<CWinThread*> m_vtImgProcThread;							//扫描图像处理线程队列
	vector<T_THREAD_DYNAMICSCAN_PARAM*> m_vtImgProcInput;			//扫描图像处理线程输入参数表
	map<int, T_THREAD_DYNAMICSCAN_OUT>* m_pMapImgProcOutput;		//扫描图像处理线程输出结果表
	vector<XI_POINT> m_vtImgProc3DPoint;							//扫描图像处理线程输出的线（点集）
	CString m_sPointCloudFileName;									//扫描点云输出文件名
	vector<XI_POINT> m_vtScanPointCloud;							//输出点云的数据

    //XI_POINT m_tRealEndPoint;										//真实的端点，（加了补偿）
    XI_POINT m_tDynamicScanEndPoint;								//动态测量的端点,如果单纯为拿到端点用该数据即可

/*	****************************参	数****************************	*/
protected:
	CUnit* m_ptUnit;
	CImageProcess* m_pTraceImgProcess = NULL;					//跟踪相机的图像处理对象（为兼容老搜索流程）
	IplImage* m_pTraceLockImg = NULL;							//跟踪相机的锁定图片（为兼容老搜索流程）

public:
	//锁定激光线
	bool lockLaser(CRobotDriverAdaptor* pRobotDriver, int nCameraNo);
	//跟踪+提取二维点
	int get2DPnts(std::vector<CvPoint>& vtPoint2D, IplImage* pImg, CString sWidgetLaserTrackParaFile, CString sWidgetLaserPntExtParaFile);
	WidgetLaserInfo m_tSearchLockLaserInfo;
};
#endif // !defined(AFX_SCANWORK_H__9D10EBBC_C52B_4630_8467_105C08A5360D__INCLUDED_)
