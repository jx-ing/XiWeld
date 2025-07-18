/*********************************************************************
* ��Ȩ���� (C)2018, �������н����ܻ����˹ɷ����޹�˾
* 
* �ļ����ƣ� ScanInitModule.h
* ����ժҪ�� 
* ��ǰ�汾�� 1.0
* ��    �ߣ� ����ǿ
* ������ڣ� 2018��3��14��
* ����˵���� 
* 
* �޸ļ�¼1��
*    �޸����ڣ�
*    �� �� �ţ�
*    �� �� �ˣ������� ����
*    �޸����ݣ� ��������
* �޸ļ�¼2����

ʹ��˵����
1.�������
2.

ע�⣺

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

#define __ScanInitModule_ConsoleDbgPrint			//����̨�������ģʽ

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

typedef enum E_POINT_CLOUD_PROC_MOMENT				// ����ɨ��:���ƴ���ʱ��
{
	E_POINT_CLOUD_PROC_MOMENT_DYNAMIC = 0,	// ʵʱ����
	E_POINT_CLOUD_PROC_MOMENT_FINAL = 1		//ɨ�������
}E_POINT_CLOUD_PROC_MOMENT;

typedef enum E_IMAGE_PROC_METHOD			// ����ɨ��:ͼ���������ĵ㷽��
{
	E_IMAGE_PROC_METHOD_EEEEE = 0,			// FindLaserMidPiontEEEEE
	E_IMAGE_PROC_METHOD_EEEEEInTrack = 1,	// FindLaserMidPiontEEEEEInTrack
	E_IMAGE_PROC_METHOD_DoubleVGroove = 2,	// ���Ƿ��¿ڼ������ĵ���ȡ
}E_IMAGE_PROC_METHOD;

typedef enum E_POINT_CLOUD_PROC_METHOD				// ����ɨ��:���ƴ�����
{
	E_POINT_CLOUD_PROC_METHOD_GetRiserEndPoint = 0,	// GetRiserEndPoint
	E_POINT_CLOUD_PROC_METHOD_Groove = 1,			// GrooveSolutionInterface_
	E_POINT_CLOUD_PROC_METHOD_DoubleVGroove = 2			// GrooveSolutionInterface_
}E_POINT_CLOUD_PROC_METHOD;

typedef enum E_JUDGE_END_METHOD				// �յ��жϷ���
{
	E_JUDGE_END_KNOW = 0,					// ��֪�˵�(��Ҫ��ǰ���ö˵�)
	E_JUDGE_END_THEORY_LENGTH = 1,			// ���۳����ж�
	E_JUDGE_END_PROCESS_SEARCH = 2			// �����������˵�
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

// nMaxBufferSize��������ݱ�������
// nTotalCapImgNum���ɼ�ͼƬ����
// nTotalProImgNum������ͼƬ����
// vpImg������Ŷ�Ӧ��ͼƬ���ϣ�ѭ�����棬����ʱ ����->�ȿ���->�ͷ�->����->����
// vtCapCoord���ɼ�ֱ������
// vtCapPulse���ɼ��ؽ�����
typedef struct T_CAP_IMAGE_DATA
{
	int nMaxBufferSize = 0;
	int nTotalCapImgNum; // ��ǰ�ɼ�������
	int nTotalProImgNum; // ��ǰ����������
	std::vector<IplImage*> vpImg; // ͼƬָ��
	std::vector<T_ROBOT_COORS> vtCapCoord; // ��ͼֱ��
	std::vector<T_ANGLE_PULSE> vtCapPulse; // ��ͼ�ؽ�
	Mutex m_MutexReadWrite; // ����ʹ�������
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

//ImgProcDis�е����߳�ThreadAsyn...��ͼ��ʶ�����
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

//���̴߳�����������
struct T_THREAD_SAVEPOINTCLOUD
{
	FILE* pOutputFile;						//����ļ�ָ��
	vector<vector<XI_POINT>>* pVvtLaserLine;	//����ļ����߶���
	Mutex* pMutexWrite;							//�ļ�д����
	Mutex* pMutex_MyVec;						//vvtLaserLine��д��
	int* pCount;								//������
	bool* pExit;								//�Ƿ��˳�������߳�

	T_THREAD_SAVEPOINTCLOUD(FILE* pOutputFile, vector<vector<XI_POINT>>* pVvtLaserLine, Mutex* pMutexWrite, Mutex* pMutex_MyVec, int* pCount, bool* pExit)
		: pOutputFile(pOutputFile), pVvtLaserLine(pVvtLaserLine), pMutexWrite(pMutexWrite), pMutex_MyVec(pMutex_MyVec), pCount(pCount), pExit(pExit)
	{
	}
};

//m_pMapImgProcOutput {��-ֵ}���е�ֵ���� 
struct T_THREAD_DYNAMICSCAN_OUT
{
	CvPoint cpKeyPoint;									//��ά����λ��,û����Ĭ����(0,0)
	T_ABS_POS_IN_BASE tPointAbsCoordInBase;
	BOOL bFindIntersectPoint;
	vector<XI_POINT> vtLaserPoint;						//������ ���Ƽ�
	T_THREAD_DYNAMICSCAN_OUT(CvPoint param1,T_ABS_POS_IN_BASE	param2, BOOL param3,vector<XI_POINT> param4) :
		cpKeyPoint(param1),tPointAbsCoordInBase(param2), bFindIntersectPoint(param3), vtLaserPoint(param4){}
};

//�Ѷ˵㴦���̣߳����̣߳��������
struct T_THREAD_DYNAMICSCAN_PARAM
{
	E_START_OR_END eEndPointType;										//��������
	CRobotDriverAdaptor* pRobotDriver;									//�����˿�����ָ��
	vector<pair<int, T_START_POINT_INFO>>* pVtnImgProcInput;				//ͼƬ�������ָ��
	map<int, T_THREAD_DYNAMICSCAN_OUT>* pMapDynamicScanOut;				//��������ָ��
	T_IMAGE_RECOGNIZE_PARAM tImgRecParam;								//ͼ�������
	ofstream* pOutFile;													//�߳��ڲ�����ļ�ָ��
	FILE* pOutFilePointCloud;											//����Ƶ��ļ�ָ�루ʹ��FLIE* ��ofstream* �ٶȿ�ָ������
	AsynchrousProc* pAsynProc;											//�첽����ָ��
	Mutex* pMutexMapLock;												//Map������				R2LevelLock
	Mutex* pMutexExitFlag;												//�߳��˳���־λ��		R2LevelLock
	Mutex* pMutexVtInput;												//ͼƬ���з�����		R2LevelLock
	Mutex* pMutexOutFilePC;												//�����ļ���
	IplImage* pShowImg;													//��ʾͼ��
	BOOL bSaveImg;														//��ͼ��־λ
	int nThreadIndex;													//���߳����ܴ����̶߳����еı��
	BOOL* pBisExit;														//�߳��˳���־λ
	BOOL bSavePointCloud;												//���Ʊ�־λ
	CImageProcess* pImgProcess;											//ͼ����ָ��
	E_FLIP_MODE eFlipMode;												//��άͼ��ת����
	E_SCANMODE eScanMode;												//ɨ��ģʽ
	CString cstrUnitName;												//���Ƶ�Ԫ����
	T_CAMREA_PARAM tCamParam;											//�������
	XiRobotAbsCoorsTrans cAbsCoorsTrans;								//����ת����
	CString cstrImgSavePath;											//ͼƬ����·��
	int* pSavePointCloudCount;											//������ż�����
	int nNumOfSavePointThreads;											//����Ƶ�����߳�����


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

//ImgProcDis�е����߳�ThreadAsyn...�����ݲ���m_pMapImgProcOutput�еĺ����Ĳ���
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

//��ֲ������ʹ�õ�����ģ��

//class TraceModel {
//public:

struct TraceModel
{
	/*=============ԭ�ȷ�����RobotDriver������==============*/

	/*=========InitWeldStartVal=========*/
	DOUBLE m_dWeldLen;								//�����ܳ���
	DOUBLE m_dCurrentWeldLen;						//��ǰ���ӳ���
	DOUBLE m_dChangeAngleTheshold;					//����̬����
	DOUBLE m_dScanStartOffset;						//��㴦�����೤������
	DOUBLE m_dWeldEndPointOffset;					//�յ㴦�����೤������
	DOUBLE m_dDisSafeGunToEnd;						//��ˮ��15�������ཻ5�����յ㴦Ԥ������
	DOUBLE m_BoardThick;							//ѡ���ղ���ʹ�õİ��
	DOUBLE m_dWeldVelocity;							//�����ٶ�
	DOUBLE m_dGunToEyeCompenX;						//������
	DOUBLE m_dGunToEyeCompenZ;						//������
	DOUBLE m_dFlatHorComp = 0.0;

	BOOL m_bUpdatedWarpData;						//���°�������
	BOOL m_bIsTrackingScanEnd;						//���ٹ���������βλ��
	BOOL m_bIsCloseTrackThread;						//�Ƿ�رո����߳�
	BOOL m_bIfOpenExternal;							//�Ƿ������ⲿ��
	BOOL m_bIfWeldToEndPoint;
	BOOL m_bCameraFindWeldEnd;

	INT m_nOpenTrackingPos;							//��������λ�ã����ڱ���̬�����
	INT m_nCloseTrackingPos;						//�رո���λ�ã����ڱ���̬�����

	E_WRAPANGLE_TYPE m_eStartWrapAngleType;			//��������

	/*===����������===*/
	INT m_nWeldSeamType;							//��������
	/*===����������===*/

	T_WELD_PARA m_tWeldParam;
	/*=========InitWeldStartVal=========*/


	/*=========CornerLineWeldFunNew=========*/
	vector<vector<T_ROBOT_COORS>> m_vvtWarpCorrectCoors;	// ����������������
	vector<vector<T_ROBOT_COORS>> m_vvtTrackWarpCoors;
	vector<vector<INT>> m_vvnTrackWarpPtType;
	/*=========CornerLineWeldFunNew=========*/


	/*=========GetTheoryReferencePulse=========*/
	vector<T_ANGLE_PULSE> m_vtTheoryMeasurePointLeft;
	/*=========GetTheoryReferencePulse=========*/


	/*=========DoWeld=========*/
	vector<T_ROBOT_COORS>	m_vtRealEndpointCoor;			//��ʵ�Ľ�β��
	vector<int>				m_vtEndPointType;				//��β������
	/*=========DoWeld=========*/


	/*=========WeldProcessBefor=========*/
	DOUBLE m_dRecordBreakPointRZ;							//ͣ��ʱRz�仯��
	INT m_nWeldStepNo;										//��¼���Ӳ��ţ������Ȳ�󺸶�������
	/*=========WeldProcessBefor=========*/


	/*=========DoweldRuning=========*/
	vector<TrackFilter_XYZ> m_vtRecordWarpBeforWeldData;	//��¼����ǰ��������
	vector<T_ROBOT_COORS> m_vtBreakContinueBeforPoints;		//��������ʱ�洢�Ѻ�������
	vector<INT> m_vtBreakContinueBeforPointType;			//��������ʱ�洢�Ѻ�������
	vector<INT> m_vtWeldLinePointType;						//������������
	BOOL m_bSendWeldDataFinish;								//���ݷ���ȫ����������
	BOOL m_bSendSafeFlag;									//���ͽ�β��־λ
	BOOL m_bIsTrackingDataSendOver;
	DOUBLE m_dMoveStepDis;									//�����ƶ�����,�˲���
	DOUBLE m_dAdjustRobotRZ;
	DOUBLE m_dPreRzChange;
	long m_lPreAdjustRzTimeTick = 0;						// ��¼ÿ�ε���ʱ���
	T_ROBOT_COORS m_tRealEndpointCoor;						//��ʵ�Ľ�β��
	T_ROBOT_COORS m_tRobotCurentCoor;						//�����˼��󳵵�ǰ����
	/*=========DoweldRuning=========*/


	/*=========CorrectWarpDataFun=========*/
	//E_WRAPANGLE_TYPE m_eStartWrapAngleType;					//��������
	/*=========CorrectWarpDataFun=========*/


	/*=============ԭ�ȷ�����RobotDriver������==============*/
	bool m_bIfJudgeEndOpen;// ������β�ж�
	E_WORKPIECE_TYPE m_tWorkPieceType;// ��������,����ʵ��ѡ�����ͣ�һ�����������Ƿ�Ϊ�պϹ���
	CStepMoveControlJY      m_cStepMoveControl;//�����岹����
	int m_nAXIS_X = 0;
	int m_nAXIS_Y = 1;

	// ���̴߳�ͼ��־
	bool m_bTrackSaveImage;
	// ���۳���
	double dHandEyeDis;
// �����ǹ��
	double dBoardThink = 10.0;

	// ʵʱ���ٲο��ߣ�������������һ�θ���  ��һ�θ������������һ�θ���
	XiLineParamNode tFrontLine, tBackLine;

	// ���ߵ���ʹ��
	std::vector<TrackRawData> vtTrackRawData; // ���ٲɼ�ԭʼ����

	// �ֲ�����������Ϣ
	std::list<int> lnImageNo;	// ��ǰ���Ƽ�����ÿ��ͼ���
	std::list<int> lnImagePtnNum;	// ��ǰ���Ƽ�����ÿ��ͼ����
	std::list<Three_DPoint> ltPointCloud; // ��ǰ���Ƽ���
	std::list<T_ROBOT_COORS> ltCapWorldCoord; // ��ǰ���Ƽ�����ÿ��ͼ�ɼ�ʱ��������
	std::vector<Three_DPoint> vtPointCloudEndPoints; // ÿ�ε��ƴ�����ȡ�Ķ˵���������
	std::vector<T_ROBOT_COORS> vtConnerWorldCoords; // ÿ�δ���ǵ���������
	FILE *pfPointCloud; // �����ļ�
	FILE *pfImgCalcData; // ÿ��ͼƬ��ͼ����ȼ������ݱ���
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
	void		  TranslateDrawingToImageParam(); // ͼֽ����ת��Ϊͼ�����������
	void		  ResumeDrawingToImageParam(); // �ָ�ͼֽ����ת��Ϊͼ�����������
	BOOL          IsComplete();

	void		  SetScanParam(/*bool bSavePointCloud, bool bSaveScanImg,*/ bool bNaturalPop, E_FLIP_MODE eFlipMode, E_SCANMODE eScanMode, CString sMoveJobName);
private:
	void          Init(UINT* pUnTracePointCloudProcess = NULL, BOOL* pbSaveScanImg = NULL,
		E_FLIP_MODE* peScanFlipMode = NULL, E_SCANMODE* peScanMode = NULL, UINT* pUnSingleCameraThrdNum = NULL);
	void          UnInit();



	// ԭʼ�������ڴ�����֮��ͽ�����Ϣ�ĸ���
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
	std::vector<XI_POINT> m_vtScanInitWeldLineInWorldPoints;		//��¼��ʼ��ɨ���
	std::vector<XI_POINT> m_vtScanStartInWorldPoints;				//��¼��ʼ��ɨ���
	std::vector<XI_POINT> m_vtScanInitWeldLineInBase;				//��¼��ʼ��ɨ���
	double		m_dWorkpieceThickness;								//�������
    double m_dRealWeldStartPosX;
    double m_dRealWeldStartPosY;
    double m_dRealWeldStartPosZ;


	CMoveCtrlModule          m_MoveCtrlModule;

	// �ⲿ�����ʼ������
	double m_dStepDis;
	double m_dCoarseScanVel;
	double m_dAccurateScanVel;
	int    m_nTheoryNum;

	double m_dStartUAngle;											// ���λ�ö�Ӧ��U��Ƕ�
	XI_POINT m_tStartPoint;
	int m_nStartPointType;
	int m_nStartPointShape;
	BOOL m_nStartIfWrap;
	BOOL m_bFixedPointScan;											//�Ƿ񶨳�
private:
	double m_dXStart;
	double m_dYStart;
	double m_dXEnd;
	double m_dYEnd;

	// �������۹켣����
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

	// ͼ��ʹ�ò���

    GROUP_STAND_DIP_NS::E_WHICH_PIECE_CLASS m_ePieceClass;

	int m_iBaseLineLenInZone;
	int m_iMoveLineLenInZone;
	int m_iRoiWidSize;
	int m_iIfEndConditionIdx;
	double m_dScanStartExtendLength; // ������:����ɨ�賤��98mm�����ϵ���չ����(�����ˮ����Ҫ30mm����̬���ٸ�������չ����Ϊ30mm)
	
	// �Ƿ���Ҫɨ������������ �� ���㷨����׼ȷ���
	BOOL m_bScanStartTwoLineOnStandBoard;

public:
	double m_dScanStartWrapLength;	// ����ɨ�賤��98mm�����ϵ���չ����(������ǳ���)
    typedef struct
    {
        CRobotDriverAdaptor *pRobotDriver;
        CScanInitModule *pcMyObj;
    }T_ROBOT;
	//-------------------------��ʼ���ͼ����-----------------------
	void                       FixedStartProc(CRobotDriverAdaptor *pRobotDriver);
	T_START_POINT_INFO        *m_ptStartPointInfo;
	//--------------------------��е�۲�ͼ����------------------------------

	void                      CoordinatePointProcessFuncation(XI_POINT tWeldLinepointCoor,std::string FileNameCin,std::string FileNameCout);//����㴦����
	void                      SwapVector(XI_POINT startPoint, std::string FileNameCin,std::string FileNameCout);


    //-------------------��̬ɨ��˵㼰����-��ʼ---------------------

    /*���ٹ��̣�
    **1��DynamicScanningEndpoint������ɨ���ʼ�����ݽ��д���ļ����ڿ�ʼ����ǰ�����ݼ��ص�����������pRobotCtrl->m_vtWeldLineInWorldPoints ��ʹ��GetScanData()������
    **2����ʼ����ǰʹ��DoweldRuning(������λ��job�����ڷ���һ�����ݣ��ڷ������ʼ�����ݺ��������ʼ����job�͸��١�
    **3��ThreadRealTimeTracking():���ٺ����ڸ��ٹ����л���ȡλ�ú���̬��λ��ʵʱд�뻺�棬��̬���ݳ�ʼ��̬��ֵ���е���
    **4�����ٹ����л���ý�β�жϺ��������Խ����жϡ�
    **�������Ǵ󳵺ͻ�е�������岹ʹ��ͬһ��Job��
    **�������ݣ��Ȳ�󺸻�ʵʱ���٣��洢��pRobotCtrl->m_vtWeldLineInWorldPoints�����ڣ���
    **�����洢ʵʱλ�úͳ�ʼ������̬�����ٱ仯����̬д��p99λ�ñ����ۼ���ÿ��λ���ϣ�����ʵ��ʵʱ������̬��
    **�Ȳ�󺸸������洢ʵ�ʺ���λ�ˡ�
    **ע���˵�ɨ��ʹ��ͬһ���˵�ɨ�躯��DynamicScanningEndpoint��������õ�ɨ������ݺͶ˵����ݡ�
    */
	

	/*һ���������Ӿ�����˳��
	1 .InitIfStartOrEndPntParam(GROUP_STAND_DIP_NS::E_PIECE_START)
	1 .GetGroupStandKeyPoint():����
	1 .IfPieceStartOrEndPoint�������Ѷ˵�
	1 .InitIfStartOrEndPntParam(GROUP_STAND_DIP_NS::E_PIECE_START)
	1 .GetGroupStandKeyPoint():����
	1 .IfPieceStartOrEndPoint�������Ѷ˵�
	1 .GetGroupStandKeyPoint():����
	1 .GetGroupStandKeyPoint():����
	*/	
public:
/*	****************************��	��****************************	*/
	static UINT             ThreadDynamicScanningCapture(void* pParam);
	static UINT             ThreadDynamicScanningProc(void* pParam);
	//ͼ��ת
	void					FlipImageNew(CRobotDriverAdaptor* pRobotCtrl);
	BOOL					DynamicRealTimeTracking();
	//�����߳�
	static UINT             ThreadRealTimeTracking(void* pParam);
	//ͼƬ��ת�߳�
	static UINT				ThreadFlipImage(void* pParam);
	//������ǰ��������Ҫ����������������̬�½�������
	BOOL                    RealTimeTrackLockArcBefor(CRobotDriverAdaptor* pRobotDriver, BOOL bIsContinueWeld = FALSE);
	//����ǰ��������Ҫ����������������̬�½�������,�ú��������𻡺��������
	BOOL                    RealTimeTrackLockArcAfter(CRobotDriverAdaptor* pRobotDriver);
	void                    RealTimeTracking(CRobotDriverAdaptor* pRobotCtrl);

	// �˶� ���ٴ��� �յ��ж� ����
	/**************************** �����Ż���� Start ****************************/

	static UINT ThreadRealTimeTrackCapture_H(void* pParam);
	void FuncRealTimeTrackCapture_H(CRobotDriverAdaptor* pRobotDriver, int nCameraNo, int nProcThreadNum, int nMaxImgBufSize = 50, int nTimeOut = 2000); // ���Ʒ�ʽ���˵�����ͼƬ�ɼ��ʹ���
	// ʵʱ�����̺߳���(��֡��)
	void RealTimeTrackingNew_H(CRobotDriverAdaptor* pRobotCtrl);
	// ���ٸ�֡�ʶ��̴߳��� ��ȡ���µĺ����������꣺�޵��ƽ�� ʹ�ýǵ��� û�����½����¼ʧ�ܴ��� ���������ֵ����false �����������
	bool GetTrackWeldLineWorldCoord(T_ROBOT_COORS &tWeldLineWorldCoord, int &nConnerRstNum, int &nPointCloudRstNum, int &nFailNum, int nMaxFailNum, FILE* pfFile);

	// ���ٳ�ʼ�� ���RealTimeTrackingNewʹ��
	void InitRealTimeTracking(E_JUDGE_END_METHOD eJudgeEndMethod, T_ROBOT_COORS* pEndCoord = NULL);
	// ʵʱ�����̺߳���
	void RealTimeTrackingNew(CRobotDriverAdaptor* pRobotCtrl);

	// ��ȡ���ٲ�ͼ���� ����nImageNo�������ߵ���ʱ��ȡ��ͼ����
	void ReadTrackCapCoord(CRobotDriverAdaptor* pRobotCtrl, T_ROBOT_COORS& tRobotCoord, T_ANGLE_PULSE& tRobotPulse, T_ROBOT_COORS& tWorldCoord, int nImageNo = 0);

	// ͼƬ�ɼ� ͼƬ���nImageNo��������Ϊ��ͼƬ������, ����ģʽ ��ȡ����ͼƬ
	bool CaptureTrackImage(int nCameraNo, int &nImageNo, IplImage** pImageBuff, bool bSaveImage, CString sSavePath);
	bool ImageLockProcess(CImageProcess* pTraceImgProcess, IplImage** ppImage, XiLineParamNode &tFrontLine, XiLineParamNode &tBackLine, E_FLIP_MODE eFlipMode = E_FLIP_NONE);

	// ����ͼ���� �����Ƿ���ɹ� �������ֹͣ���� ������λ�ۼƼ��� ���㺸�����������ͺ����������������
	bool TrackProcess(CRobotDriverAdaptor* pRobotCtrl, int nImageNo, IplImage** pImageBuff, T_ROBOT_COORS tRobotCoord, T_ANGLE_PULSE tRobotPulse, 
		T_ROBOT_COORS& tWeldLineRobotCoord, T_ROBOT_COORS& tWeldLineWorldCoord, int& nFailNum, int nMaxFailNum, FILE* pfFile, FILE* pfPointCloud);

	// ����������δ����Ƿ���� ������ι���ֹͣ���� ������λ�ۼƼ��� �� ���������ٲ�ͼλ��tLastCapWorldCoord �����ۼƺ��ӳ���
	bool CheckAdjoinCapCoord(CRobotDriverAdaptor* pRobotCtrl, int nImageNo, T_ROBOT_COORS& tLastCapWorldCoord, T_ROBOT_COORS tCurCapWorldCoord, double dDisThreshold, int& nErrNum, int nMaxErrNum);
	
	// ����˲��Ƿ����� ��������˲�ʧ��ֹͣ���� ������λ�ۼƼ��� �������º�������tInputCoord ����˲����vtCoutTrackPoint �˲��������ݴ����ļ�pfFile
	bool CheckTrackFilterValid(CRobotDriverAdaptor* pRobotCtrl, int nImageNo, T_ROBOT_COORS tInputCoord, std::vector<TrackFilter_Node>& vtCoutTrackPoint, int& nInvalidNum, int nMaxInvalidNum);

	// �˲�������ݴ��� ������ε���Zֵʧ�� ֹͣ���� ������λ�ۼƼ��� ��vtCoutTrackPoint�е�ÿ������Ӳ��� д���ļ� �� ׷�ӵ����ٺ��ӹ켣������
	bool FilterResultTrans(CRobotDriverAdaptor* pRobotCtrl, int nImageNo, const std::vector<TrackFilter_Node>& vtCoutTrackPoint, int& nAdjustErrNum, int nMaxAdjustErrNum, FILE* pfFile);

	// �������ܣ���tSplitCoord��������XYZ��ֵ��ⲿ��
	// tRefCoord���ο����� x y z rx ry rz bx by bz ȡbFixExAxis��Ӧ���ֵ��Ϊ�̶�ֵʹ��
	// tSplitCoord��Ҫ��ֵ����� ����x y z rx ry rz ���x y z rx ry rz bx by bz
	// nSplitAxisNo����ֵ���� �� m_ptUnit->m_nTrackAxisNo ������ͬ��123��ʾxyz ������ʾ�ⲿ��̶�
	void SplitCoord(T_ROBOT_COORS tRefCoord, T_ROBOT_COORS& tSplitCoord, int nSplitAxisNo);

	// �յ��жϣ��жϷ���nMethod: 1.��֪�յ�(�����յ�) 2.�����۳���(����) 3.���ӹ������ж�(�����յ�)
	// �����յ�󣺸����յ�켣 �� ��Ӱ��ǹ켣 ����m_pTraceModel->m_bCameraFindWeldEndΪtrue
	bool JudgeFindWeldLineEnd(CRobotDriverAdaptor *pRobotCtrl, int nMethod);
	bool JudgeEnd_Know(const std::vector<Three_DPoint>& vtPtns, T_ROBOT_COORS tTheoryEndCoord); // ��֪�յ�
	bool JudgeEnd_TheoryLength(CRobotDriverAdaptor* pRobotCtrl); // �����۳���
	bool JudgeEnd_ProcessSearch(CRobotDriverAdaptor* pRobotCtrl); // ���ӹ����������ж�

	static UINT ThreadJudgeEnd_ProcessSearch(void* pParam);
	bool FuncJudgeEnd_ProcessSearch(CRobotDriverAdaptor* pRobotCtrl, int nTimeOut = 5000); // �ȴ� nTimeOut ms δ�˶���ʱ
	bool GetEndSectionCoorsNew(CRobotDriverAdaptor* pRobotDriver, T_ROBOT_COORS tEndpointCoor);
	bool m_IsOpenJudgeEnd_ProcessSearch;
	Mutex m_MutexTrackPointCloud; // ���ٴ��� �� �յ��ж�ʱ �������

	// ����ֲ����ƴ���ʧ�ܻ��쳣����
	void SavePointCloudProcErrData(CString sFileName, int nImgNoS, int nImgNoE, CvPoint3D32f tCameraNorm, CvPoint3D32f 
		tPlaneHNorm, CvPoint3D32f* tRefPtn, int tRefPtnNum, Three_DPoint* ptPointCloud, int nPointCloudNum);

	// �յ��ж�
	bool JudgeFindEndPoint(std::vector<Three_DPoint>& vtPtns, double dDisThreshold, double nPtnNumThreshold);
	/**************************** �����Ż���� End ****************************/

	/**************************** �˵������Ż���� start ****************************/
	//��̬��ͼ ���̲߳�ͼ���� ��Ҫʹ��InitDynamicCapture_H_M�ȳ�ʼ���ٵ���
	void DynamicCaptureNew(CRobotDriverAdaptor* pRobotDriver, int nCameraNo, CString sPointCloudFileName,int nTimeOut = 5000); // ���Ʒ�ʽ���˵�����ͼƬ�ɼ��ʹ���
	//ȫɨ�趯̬��ͼ
	void DynamicCaptureNew_ScanWeldLine(CRobotDriverAdaptor* pRobotDriver, int nCameraNo, CString sPointCloudFileName, int nTimeOut = 5000);
	// ��ȡͼƬ�������ĵ�
	bool FuncImageProcessInTrack_Lock(IplImage* pImg, XiLineParamNode& tFrontLine, XiLineParamNode& tBackLine, std::vector<CvPoint>& vtPoint2D, E_FLIP_MODE eFlipMode);
	bool FuncImageProcessInTrack(IplImage* pImg, XiLineParamNode& tFrontLine, XiLineParamNode& tBackLine, std::vector<CvPoint>& vtPoint2D, E_FLIP_MODE eFlipMode);
	bool FuncImageProcessEEEEE(IplImage* pImg, std::vector<CvPoint>& vtPoint2D);

	void DynamicCaptureNew_H(CRobotDriverAdaptor* pRobotDriver, int nCameraNo, int nProcThreadNum, int nMaxImgBufSize = 50, int nTimeOut = 5000); // ���Ʒ�ʽ���˵�����ͼƬ�ɼ��ʹ���
	static UINT ThreadImageProcess(void* pParam);
	bool FuncImageProcess(CRobotDriverAdaptor* pRobotCtrl);
	bool m_IsOpenImageProcess;
	T_CAP_IMAGE_DATA* m_pCapImageData; // DynamicCaptureNew_H�д���

	void DrawAndShowImg(IplImage** ppSrc, IplImage** ppShowImg, const CvPoint& tPtn2D, const XiLineParamNode& tFrontLine, const XiLineParamNode& tBackLine);

	/**************************** �˵������Ż���� End ****************************/

	/**************************** ɨ�������� Start ****************************/

	// ֱ����ɨ��	EEEEEInTrack	GetRiserEndPoint			��/��̬����		�����ֱ�캸��켣���˵�����
	// ����ɨ��		EEEEE			GetRiserEndPoint			�Ƕ�̬����		������������յ�����
	// �������¿�	EEEEE			GrooveSolutionInterface_	��/��̬����		������¿ڶ��������
	
	// �˶���ͼ ���ɵ��� ����ȡ�ؼ���
	// ��ʵʱ������� ��ɨ�������������
	// ��ָ����ͬ��ȡ�������ĵ㷽ʽ
	// ��ָ����ͬ�ĵ�����ȡ��ʽ
	// ��ָ�����������ʽ
	// ʹ��ǰ����ʹ�� InitDynamicCapture_H_M ��ʼ��	
	bool InitDynamicCapture_H_M(
		E_POINT_CLOUD_PROC_MOMENT ePointCloudProcMoment, E_IMAGE_PROC_METHOD eImgProcMethod,
		E_POINT_CLOUD_PROC_METHOD ePointCloudProcMethod, int nCamNo, CString sSaveFilePath, CString sPointCloudFileName,
		double dSavePointCloudLen, E_WELD_SEAM_TYPE eWeldSeamType, int nGroupNo, int nLayerNo, bool bInsideCallJob = true, 
		bool bJudgeEndPoint = false, bool bSaveImage = false, int nSaveImgStepNo = 2, int nPtn2DStepNo = 2);
	void DynamicCaptureNew_H_M(CRobotDriverAdaptor* pRobotDriver, int nCameraNo, int nProcThreadNum, int nMaxImgBufSize = 50, int nTimeOut = 2000); // ���Ʒ�ʽ���˵�����ͼƬ�ɼ��ʹ���
	CString m_sSaveFilePath;
	CString m_sGroovePointCloudFileName;
	double m_dSavePointCloudLen; // �����������ͼƬ�ĵ���
	T_CAP_IMAGE_DATA* m_pCapImageData_H_M; // DynamicCaptureNew_H_M�д���
	E_POINT_CLOUD_PROC_MOMENT m_ePointCloudProcMoment = E_POINT_CLOUD_PROC_MOMENT_DYNAMIC;
	E_IMAGE_PROC_METHOD m_eImgProcMethod = E_IMAGE_PROC_METHOD_EEEEE;
	E_POINT_CLOUD_PROC_METHOD m_ePointCloudProcMethod = E_POINT_CLOUD_PROC_METHOD_GetRiserEndPoint;
	E_JUDGE_END_METHOD m_eJudgeEndMethod = E_JUDGE_END_PROCESS_SEARCH; // �յ��жϷ���
	bool m_bDynamicJudgeEndPoint = false; // �Ƿ�ɨ����̶�̬�յ��ж�
	bool m_bInsideCallJob = true;

	// ͼƬ����
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

	// ���ƴ���
	static UINT ThreadPointCloudProcess_H_M(void* pParam);
	bool FuncPointCloudProcess_H_M(CRobotDriverAdaptor* pRobotCtrl, int nTimeOut = 2000); // �ȴ� nTimeOut ms δ�˶���ʱ
	bool FuncPointCloudProcess_H_M_GetRiser(CRobotDriverAdaptor* pRobotCtrl, int nTimeOut = 2000);
	bool FuncPointCloudProcess_H_M_Groove(CRobotDriverAdaptor* pRobotCtrl, int nTimeOut = 2000);
	bool FuncPointCloudProcess_H_M_DoubleVGroove(CRobotDriverAdaptor* pRobotCtrl, int nTimeOut = 2000);
	bool m_IsOpenPointCloudProcess_H_M;
	Mutex m_Mutex_H_M; // ͼƬ���� �� ���ƴ��� �������

	// ������ȡ�������
	bool FinalProc_Groove();
	bool LoadTeachResult(int nGroupNo, int nLayerNo, std::vector<T_TEACH_RESULT>& vtTeachResult);

	//void SaveGrooveInfoToFile(FILE *pf, const GroovePointInfo &tGrooveInfo, bool bSaveBoth); // bSaveBoth :true������+�յ���� false:ֻ�����յ����
	//void SaveGrooveInfoToFile(FILE *pf, const GrooveCloudExtPointInfo&tGrooveInfo, bool bSaveBoth); // bSaveBoth :true������+�յ���� false:ֻ�����յ����
	//void SaveGrooveInfoToFile(FILE *pf, const GrooveCloudExtPointInfoM&tGrooveInfo, bool bSaveBoth); // bSaveBoth :true������+�յ���� false:ֻ�����յ����
	void SavePtn3D(FILE *pf, CvPoint3D32f tPtn);
	void AppendGrooveInfoPtn(std::vector<Three_DPoint>& vtThree_Points, const CvPoint3D32f& tPtn);
	/**************************** ɨ�������� End ****************************/

	// ���������Ϊ�˵㣨���Ϊ�������꣩
	BOOL					SinglePointMeasure(CRobotDriverAdaptor* pRobotCtrl);
	//�����˵�ǰ��ʼ��,��������ǰֻ��ʼ��һ��
	BOOL					ScanEndpointLockInit(CRobotDriverAdaptor* pRobotDriver);
	//�����˵�
	BOOL                    DynamicScanningEndpoint(CRobotDriverAdaptor* pRobotDriver);
	//��̬��ͼ
	void                    DynamicCapture(CRobotDriverAdaptor* pRobotDriver);
	//����
	void                    DynamicEndPointProc(CRobotDriverAdaptor* pRobotDriver, BOOL bSaveImg = FALSE);
	//�Ѷ˵���Ⱥ���
	void					ImgProcDistribute(CRobotDriverAdaptor* pRobotDriver, unsigned int nMaxThreadNum = 3, const int& nScanSensitivity = 5,
											  const unsigned int& nThreshold = 0, const int& nPicStrIndex = 3, const int& nWaitOutTime = 5000,
											  const bool& bSaveImg = false, const bool& bSavePointCloud = false, E_START_OR_END ePtType = E_PIECE_END,
											  const E_FLIP_MODE& eFlipMode = E_FLIP_NONE, const E_SCANMODE& eScanMode = E_SCANMODE_2DPOINT,
											  const CString cstrJobName = "DYNAMICSCANNING", const unsigned int nJudgeRobotStopTime = 500,
											  const unsigned int nNumOfSavePCThreadsInSingleProcThrd = 1);
	//���̴߳����
	static UINT				ThreadSavePointCloud(LPVOID pParam);
	//�Ѷ˵㴦���������̣߳�
	static UINT				ThreadAsynDynamicScanEndPointProc(LPVOID pParam);
	//ɨ��������
	BOOL                    DynamicEndPointResultPro(CRobotDriverAdaptor* pRobotDriver);
	BOOL					StartDataPointPro(CRobotDriverAdaptor* pRobotDriver, std::vector<T_ROBOT_COORS>& vtStartCoors, BOOL bIsTracking = FALSE,double dTheoryLength = 0.0,bool bFirstFit = true);
	BOOL					DataPointsProcess(CRobotDriverAdaptor* pRobotDriver, std::vector<XI_POINT> vtCinPoints, std::vector<XI_POINT>& vtCoutPoints, vector<T_ROBOT_COORS>& vtCoutRobotCoors);
	BOOL                    GetScanData(CRobotDriverAdaptor* pRobotDriver, std::vector<XI_POINT>& tScanData, std::vector<T_ROBOT_COORS>& tScanRobotData, double& dDirAngle);
	static BOOL             CompDoubleFunc(double a, double b);
	//�������˼�ͣ
	void					CheckMachineEmg(CRobotDriverAdaptor* pRobotDriver);
	//��β���ж�
	void					CallJudgeEndpointFun();
	static UINT				ThreadJudgeIsWeldCompen(void* pParam);
	BOOL					JudgeIsWeldCompen(CRobotDriverAdaptor* pRobotDriver, T_ROBOT_COORS tEndpointCoor);
	//�����β����
	BOOL				    GetEndSectionCoors(CRobotDriverAdaptor* pRobotDriver, T_ROBOT_COORS& tEndpointCoor);
	//��¼ʱ��
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
	//����ͼ�����⺯��,�����ṹ��Ϊ T_IMAGE_PROCINPUT_PARAM
	static void				ImgProcInsertPairInMap(LPVOID pParam);
	static void				UsSleep(const unsigned int& nUs);
	static XI_POINT			Camera2DTo3D(CvPoint tCamera2DPoint, T_CAMREA_PARAM tCamParam, XiCV_Image_Processing* pXiCvProcess=NULL);
	static T_ROBOT_COORS	TransCameraToRobot(XI_POINT tCameraCoor, T_CAMREA_PARAM tCamParam, GROUP_ROBOT_ABS_COORS_TRANS::XiRobotAbsCoorsTrans cAbsCoorsTrans, E_MANIPULATOR_TYPE eManipType, T_ROBOT_COORS tCaptureRobotCoor, T_ANGLE_PULSE tCaptureRobotPulse);
	static T_ROBOT_COORS	TransCamera2DToBase(CvPoint tCamera2DPoint, T_CAMREA_PARAM tCamParm, GROUP_ROBOT_ABS_COORS_TRANS::XiRobotAbsCoorsTrans cAbsCoorsTrans, E_MANIPULATOR_TYPE eManipType, T_ROBOT_COORS tCaptureRobotCoor, T_ANGLE_PULSE tCaptureRobotPulse, T_ABS_POS_IN_BASE& tPointAbsCoordInBase,XiCV_Image_Processing* pXiCvProcess=NULL);

	void					LoadIsTrackCompleteFlag(CRobotDriverAdaptor* pRobotCtrl, int& TrackFlag);
	void					SaveIsTrackCompleteFlag(CRobotDriverAdaptor* pRobotCtrl, int TrackFlag,double dVoerLength);

	void					SaveEndpointData(CString RobotName,int nGroupNo, int nEndpointNo,XI_POINT tRealEndpoint, bool bShape = false);
	// ���ٶ����ʱ���ò���,��֧�ֵװ��խ�ߺ���
	void					SetTrackProcessParam(int nLayerNo);

	/********************	   Get����		********************/

	//ȡ�Ѷ˵㴦����ά��㼯
	vector<XI_POINT> GetVtImgProcResult3D() {
		return m_vtImgProc3DPoint;
	}

	//��ȡɨ������ļ����ƣ���ʱʹ�ã����Ϊֱ�ӻ�ȡ���ƣ�
	CString GetPointCloudFileName()
	{
		return m_sPointCloudFileName;
	}

	//ȡ�Ѷ˵�Ķ˵���
	XI_POINT GetDynamicScanEndPoint() {
		return m_tDynamicScanEndPoint;
	}

	vector<XI_POINT> GetScanPointCloud() {
		return m_vtScanPointCloud;
	}
	/********************	   Get����		********************/
	/********************	   Set����		********************/
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
	/********************	   Set����		********************/

/*	****************************��	��****************************	*/

/*	****************************��	��****************************	*/

	CWrapAngleParam* m_pcWrapAngleParam;
	FILE* m_pRecordTheoryPoint;							//�洢��������
	double m_dWeldInHomeRz;
	int m_nStartPointCapCursor;
	int m_nStartPointProcCursor;
	//�ҵ��˵��־λ
	BOOL m_bCameraFindWeldStart;
	BOOL m_bIfOpenJudgeStartCap;
	int	m_nTimeStart;
	XI_POINT m_tAlgCalcStartPoint;						// �����άɨ���㷨����õ������
	BOOL m_bStartPointCapCur;							//�ж��߳��Ƿ����
	BOOL m_bStartPointProcese;
	BOOL m_bScanStartFlag;								//���ݱ�־λ
	XI_POINT m_tWeldLinepointInWorld;
	CRobotDriverAdaptor* m_pRobotDriver;
	IplImage* m_pShowImage;								//��������ʾͼ��

	/*****	�Ѷ˵�ʹ�õ�public����	*****/
	E_START_OR_END m_eEndPointType;						//ɨ��˵����ͣ������յ�
    int  m_nEndPointCapCursor;							//�˵��ͼ���
    int  m_nEndPointProcCursor;							//����ͼ��
	int	 m_nScanSensitivity;							//����������
    BOOL m_bEndPointCapCur;								//�жϲ�ͼ�߳��Ƿ���� 
    BOOL m_bEndPointProcess;							//�жϴ����߳��Ƿ����
	BOOL m_bSuccessScanFlag;							//�ɹ�������־λ
    BOOL m_bRealTimeTrackstate;							//�����߳�״̬
	BOOL m_bOpenTrackingStatus;							//�����߳��Ƿ��ѿ���
	BOOL m_bLaserLightStatus;							//����״̬
    BOOL m_bDynamicCapturestate;						//��ͼ�߳�״̬
    BOOL m_bDynamicProcstate;							//�����߳�״̬
	BOOL m_bFlipedImage;								//��ת�߳�״̬
	UINT* m_pUnSingleCameraThrdNum;						//����������߳�����
	UINT* m_pUnTracePointCloudProcess;					//���ƴ������� 0���� 1����� 2 ��+�������
	BOOL* m_pSaveTraceScanPic;							//�Ƿ���Ѷ˵�ԭͼ
	E_FLIP_MODE* m_peScanFlipMode;						//ɨ��ʱͼ��ķ�ת���
	E_SCANMODE* m_peScanMode;							//���߳�����ģʽ
	CString m_sMoveJobName;								//�����˶�Job����
	/*****	�Ѷ˵�ʹ�õ�public����	*****/

	/*****		���Ӳ���		*****/
	double m_dHandEyeDis;								// ���۾��룺����ǰ���������
	double m_dMeasureDisThreshold;						// ��������뺸������յ������ֵ
	double m_dPlatWeldRx;								// ��׼�����ͺ�����̬Rx
	double m_dPlatWeldRy;								// ��׼�����ͺ�����̬Ry
	double m_dNormalWeldRx;								// ����ƽ�Ǻ�����̬Rx,���ڲ����ͺ�����̬��ͬʱ
	double m_dNormalWeldRy;								// ����ƽ�Ǻ�����̬Ry,���ڲ����ͺ�����̬��ͬʱ
	double m_dStandWeldRx;								// ��׼������̬Rx
	double m_dStandWeldRy;								// ��׼������̬Ry
	double m_dTransitionsRx;							// ���ȵ���̬Rx
	double m_dTransitionsRy;							// ���ȵ���̬Ry
	double m_dWeldNorAngleInHome;						// �����˰�ȫλ�ùؽ�����״̬ ��Ӧ�ĺ��취���
	T_ROBOT_COORS m_tRobotHomeCoors;					// �����˰�ȫλ�ùؽ������Ӧ��ֱ������(��ǹ����)�����캯����ʼ��
	bool m_bWorkpieceShape;								// ���ٺ�����״ falseֱ�� trueԲ��
	/*****		���Ӳ���		*****/

	// -------------����ԭ���̱���----------------
	TraceModel* m_pTraceModel  = new TraceModel();;//����ʹ�ýṹ��
	//--------------------------------------------
	
/*	****************************��	��****************************	*/
private:
/*	****************************��	��****************************	*/

	BOOL __flag_self_create;		//�Ƿ����ҹ��죬�ж��Ƿ�������ָ�룬��ͼָ�룬����ָ��,�������ʾ�����ҹ��죬��ñ�־λΪFALSE

	/********************	���߳���	********************/
	/*		0 - > 99+ Ȩ�������½�	*/

	//R0LevelLock		���߳�������߳� �Ļ���
	Mutex m_MutexCapThrdExitFlag;									//��ͼ�߳̽�����־λ�� m_bEndPointCapCur						ֻ���ɲ�ͼ�߳̽�m_bEndPointCapCur��ΪTRUE
	Mutex m_MutexProcThrdExitFlag;									//�����߳̽�����־λ�� m_bEndPointProcess						ֻ���ɴ����߳̽�m_bEndPointProcess��ΪTRUE
	Mutex m_MutexSuccessScanFlag;									//�����̣߳������̣������Ƿ�ɹ���־λ�� m_bSuccessScanFlag		ֻ���ɴ����߳��޸ĸñ�־λ

	//R1LevelLock		�����̼߳䣨��ͼ������ַ��̣߳��Ļ���
	Mutex m_MutexFindEndPointFlag;									//�ҵ��˵��־λ�� m_bCameraFindWeldStart

	//R2LevelLock		����ַ��̼߳䣨���ȡ������̣߳��Ļ���
	Mutex m_MutexMapProcOut;										//������Map��	m_pMapImgProcOutput
	Mutex m_MutexExitFlag;											//�����߳��˳���־λ��
	Mutex m_MutexPicCapture;										//��ͼ������ m_ptStartPointInfo��m_nEndPointCapCursor
	Mutex m_MutexSavePointCloud;									//���c���i���ļ����룩
	//�ź���
	HANDLE m_hGetCameraCtrl;
	

	/********************	���߳���	********************/

	vector<BOOL> m_vtImgProcThreadStatus;							//ɨ��ͼ�����߳�ʹ�����
	vector<CWinThread*> m_vtImgProcThread;							//ɨ��ͼ�����̶߳���
	vector<T_THREAD_DYNAMICSCAN_PARAM*> m_vtImgProcInput;			//ɨ��ͼ�����߳����������
	map<int, T_THREAD_DYNAMICSCAN_OUT>* m_pMapImgProcOutput;		//ɨ��ͼ�����߳���������
	vector<XI_POINT> m_vtImgProc3DPoint;							//ɨ��ͼ�����߳�������ߣ��㼯��
	CString m_sPointCloudFileName;									//ɨ���������ļ���
	vector<XI_POINT> m_vtScanPointCloud;							//������Ƶ�����

    //XI_POINT m_tRealEndPoint;										//��ʵ�Ķ˵㣬�����˲�����
    XI_POINT m_tDynamicScanEndPoint;								//��̬�����Ķ˵�,�������Ϊ�õ��˵��ø����ݼ���

/*	****************************��	��****************************	*/
protected:
	CUnit* m_ptUnit;
	CImageProcess* m_pTraceImgProcess = NULL;					//���������ͼ�������Ϊ�������������̣�
	IplImage* m_pTraceLockImg = NULL;							//�������������ͼƬ��Ϊ�������������̣�

public:
	//����������
	bool lockLaser(CRobotDriverAdaptor* pRobotDriver, int nCameraNo);
	//����+��ȡ��ά��
	int get2DPnts(std::vector<CvPoint>& vtPoint2D, IplImage* pImg, CString sWidgetLaserTrackParaFile, CString sWidgetLaserPntExtParaFile);
	WidgetLaserInfo m_tSearchLockLaserInfo;
};
#endif // !defined(AFX_SCANWORK_H__9D10EBBC_C52B_4630_8467_105C08A5360D__INCLUDED_)
