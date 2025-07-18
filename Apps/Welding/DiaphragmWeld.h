// DiaphragmWeld.h: interface for the CDiaphragmWeld class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_DIAPHRAGMWELD_H__27CA3DC8_2FB9_4286_8B23_8FBE2B7E4E60__INCLUDED_)
#define AFX_DIAPHRAGMWELD_H__27CA3DC8_2FB9_4286_8B23_8FBE2B7E4E60__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
#ifndef CONST_H
#define CONST_H
#include ".\Apps\PLib\BasicFunc\Const.h"
#endif
#include <iostream>
#include <cmath>

#include "WeldAfterMeasure.h"
#include ".\Apps\PLib\YaskawaRobot\RobotDriverAdaptor.h"
#include ".\Apps\Welding\ScanInitModule.h"


//#include "correctFilletError.h"
//#include "GetRiserEndPoint.h"
//#include "AbsCoorTransLib.h"
//#include "MoveCtrlModule.h"
//#include "GroupLaserVision.h"

#define INVERST_ROBOT TRUE //����������������

const double m_gExAxisSpeed = 8000.0;

using namespace std;
using namespace WAM;

typedef struct
{
	int nWorkpieceNo;//�������
	int nBoardNo;//�������
	int nEndPointNo;//�˵���
	XI_POINT tPointData;
}T_ENDPOINT_INFO_NEW;
typedef vector <T_ENDPOINT_INFO_NEW> VT_ENDPOINT_INFO_NEW;
typedef vector <VT_ENDPOINT_INFO_NEW> VVT_ENDPOINT_INFO_NEW;

typedef struct
{
	XI_POINT tPoint;
	XI_POINT tDir;
} T_POINT_DIR;

typedef struct
{
	int nWorkpieceNo;//�������									
	int nBoardNo;   //���ӱ��
	bool bBoardType; //ֱ�ߡ�Բ��
	double dBoardLength;//���ӳ���
	double dBoardThick;//���Ӻ��
	double dBoardHight;//���Ӹ߶�
	int nStartNo;//�����
	int nStartType;//�������
	int nStartShape;//�����״
	int nStartIfWrap;//����������
	bool bStartReleVircital;//�Ƿ��������
	bool bStartReleFlat;//�Ƿ����ƽ��
	int bStartContiguousCor;//���ڽǷ�
	double dStartAngle;//��㷨��Ƕ�
	int nEndNo;//�յ���
	int nEndType;//�յ�����
	int nEndShape;//�����״
	int nEndIfWrap;//�յ��������
	bool bEndReleVircital;//�Ƿ��������
	bool bEndReleFlat;//�Ƿ����ƽ��
	int bEndContiguousCor;//�յ����ڽǷ�
	double dEndAngle;//����㷨��Ƕ�
	int nProperty;//����������ͨ�Ƿ���ǿ�壩
	int nWeldLineWeldType; //��������(����������)
	double dInterWeldLength;//�������ӳ���
	double dInterStopLength;//����ͣ������
	int nInterWeldType;  //��������
	bool bIsDoubleWeld;//�Ƿ�˫�溸��
	bool bIfArc;//�Ƿ� ��
	BOOL bIfWeldCom; //�Ƿ��������ӣ�����ʱ��
	int nWitchRobot;//����������

}T_CORNER_LINE_INFO_NEW;

typedef vector <T_CORNER_LINE_INFO_NEW> VT_CORNER_LINE_INFO_NEW;
typedef vector <VT_CORNER_LINE_INFO_NEW> VVT_CORNER_LINE_INFO_NEW;

typedef struct
{
	int nWorkpieceNo;//������ţ�ǰ�壺0����� 1
	int nBoardNo;   //���ӱ��
	int nCornerLineNo; //�����Ƿ����
	double dContiguousFlat;//����ƽ��
	double dVerHoleHeight;//��ˮ�׸߶�
	double dVerWeldLength;//��������
	double dMainAngle;//�����н�
	double dStartAngle;//��㷨��Ƕ�
	int nStartNo;//�����
	int nStartIfWrap;//����������
	int nEndNo;//�յ���
	int nEndIfWrap;//�յ��������
	int nWeldLineWeldType; //��������(����������)
	double dInterWeldLength;//�������ӳ���
	double dInterStopLength;//����ͣ������
	int nInterWeldType;  //��������
	bool bIsDoubleWeld;//�Ƿ�˫�溸��
	bool bIfArc;//�Ƿ� ��
	bool bIfWeldCom;//�Ƿ񺸽����
	int nWitchRobot;//����������
}T_VERTICAL_INFO_NEW;
typedef vector <T_VERTICAL_INFO_NEW> VT_VERTICAL_INFO_NEW;
typedef vector <VT_VERTICAL_INFO_NEW> VVT_VERTICAL_INFO_NEW;

typedef struct
{
	vector<double> vdNormal;
	vector<XI_POINT>  vtPoint;
} E_WELD_LINE;


typedef struct
{
	int nWorkpieceNo;//������ţ�ǰ�壺0����� 1
	int nBoardNo;   //���ӱ��
	int nMeasurePointNum;//��������
	vector <XI_POINT> VMeasurePOINT;

}T_MEASURE_INFO_NEW;
typedef vector <T_MEASURE_INFO_NEW> VT_MEASURE_INFO_NEW;
typedef vector <VT_MEASURE_INFO_NEW> VVT_MEASURE_INFO_NEW;

typedef struct
{
	int nWorkpieceNo;//�������
	int nBoardNo;//�������
	double dCenterX;
	double dCenterY;
	double dCenterZ;
}T_CIRCLE_CENTER_INFO_NEW;
typedef vector <T_CIRCLE_CENTER_INFO_NEW> VT_CIRCLE_CENTER_INFO_NEW;
typedef vector <VT_CIRCLE_CENTER_INFO_NEW> VVT_CIRCLE_CENTER_INFO_NEW;

typedef struct tPoint3D
{
	double x;
	double y;
	double z;
}tPoint3D;

struct ESTUN_TRACK_PARAM {
	CRobotDriverAdaptor* tdRobotDriver; //�̲߳���
	vector<T_ROBOT_COORS>* tdWeldTrace; //�����ĺ��ӹ켣
	int tdWaveType; //�ڶ���ʽ(0:���ڶ� 1:ǰ��ڶ�)
};

typedef enum
{
	E_SEARCH_ENDPOINT = 0,//�����˵�
	E_MEASURE_ENDPOINT,//�����˵�,ƽ��
	E_MEASURE_ENDPOINT_VTCL,//�����˵�,����
	E_CORRECT_ENDPOINT,//�϶̺��������˵�
	E_UPDATED_ENDPOINT,//�����¶˵�
	E_MEASURE_PROJPOINT, //����ͶӰ�ߵĲ�����
	E_NO_OPERATION//������
}E_GET_ENDPOINT_METHOD;//��ȡ�˵�ģʽ

//���������Ϣ��
typedef struct
{
	CString strFile;
	bool bIsReleveVertical;//�Ƿ��������	 
	VVT_ENDPOINT_INFO_NEW vvtEndpoint_Info;//�˵���Ϣ
	VT_VERTICAL_INFO_NEW vtVertical_Info;//������Ϣ
	VVT_CORNER_LINE_INFO_NEW vvtCorner_Info;//�����Ƿ���Ϣ
	vector<vector<T_ALGORITHM_POINT>> vvtVerticalRealTrackCoors;
}T_VERTICAL_RELEVE_INFO;

//typedef struct
//{
//    string strfileName;//���ݴ洢·��
//    bool bTracking;//�Ƿ���Ҫ����
//    E_GET_ENDPOINT_METHOD eGetStartMethod;//����ȡ��ʽ
//	bool bFixedPointScanStart;//��㶨����
//    E_GET_ENDPOINT_METHOD eGetEndMethod;//�յ��ȡ��ʽ
//	bool bFixedPointScanEnd;//�յ㶨����
//    bool bIsTrackingScanEnd;//�Ƿ񺸽ӹ�����������βλ��
//	bool bIsMiddleMeasure;//�м�λ���Ƿ���Ҫ����
//	bool bIsClosedRing;//�Ƿ��Ǳպϼ�ǿȦ
//	int  nOpenTrackingNo;//�������ٵ�λ
//	int  nCloseTrackNo;//�رո��ٵ�λ
//    XI_POINT tStartPoint;//���
//    XI_POINT tEndPoint;//�յ�
//	VT_CORNER_LINE_INFO_NEW vtCornerRelsve_Info;//0������1����������2���յ����.��ǿȦʱ��˳�����
//	VT_CIRCLE_CENTER_INFO_NEW vtCenter_Info;//Բ��
//	VVT_ENDPOINT_INFO_NEW vtEndpoint_info;//�˵���Ϣ
//	vector<vector<T_ROBOT_COORS>> vvtMeasure_Info;
//	vector<vector<T_ANGLE_PULSE>> vvtMsePulse_Info;
//	vector<vector<T_ALGORITHM_POINT>> vvtMeasureResult_Info;
//	vector<vector<XI_POINT>> vvtheoryTrack;
//	vector<vector<T_ROBOT_COORS>> vvtheoryTrackRobotCoors;
//	vector<T_ROBOT_COORS > vtMeasure_Data;//�������ݵ�����
//	vector<T_ROBOT_COORS > vtMiddleMeasure_Data;//�������ݵ�����
//}T_BOARD_INFO_NEW;

typedef struct
{
	vector<T_ROBOT_COORS> vtMeasurePointStart; // ����������
	vector<T_ANGLE_PULSE> vtMeasurePulseStart;
	vector<int> vnMeasureTypeStart; // ��������
	double dExAxisPosStart;

	vector<T_ROBOT_COORS> vtMeasurePoint; // �м��������
	vector<T_ANGLE_PULSE> vtMeasurePulse;
	vector<int> vnMeasureType; // ��������
	double dExAxisPos;

	vector<T_ROBOT_COORS> vtMeasurePointEnd; // �յ��������
	vector<T_ANGLE_PULSE> vtMeasurePulseEnd;
	vector<int> vnMeasureTypeEnd; // ��������
	double dExAxisPosEnd;

	vector<T_ROBOT_COORS> vtMeasureWarp; // ���ǲ�������
	vector<T_ANGLE_PULSE> vtMeasurePulseWarp;
	vector<int> vnMeasureTypeWarp; // ��������
	double dExAxisPosWarp;

	vector<T_ROBOT_COORS> vtMeasureProStart; // ͶӰ������������
	vector<T_ANGLE_PULSE> vtMeasurePulseProStart;
	vector<int> vnMeasureProStart; // ��������
	double dExAxisPosProStart;

	vector<T_ROBOT_COORS> vtMeasureProEnd;	// ͶӰ���յ��������
	vector<T_ANGLE_PULSE> vtMeasurePulseProEnd;
	vector<int> vnMeasureProEnd; // ��������
	double dExAxisPosProEnd;
}E_MEASURE_TRACK;

typedef enum
{
	E_CONTINUOUS = 0,
	E_NO_CONTINUOUS,
	E_FRONT_CONTINUOUS,//ǰ������
	E_CONTINUOUS_AFTER,//��������
	E_CONTINUOUS_ALL//ȫ������
}E_COUNTINUOUS_WELDING_TYPE;

typedef struct
{
	string strfileName;//���ݴ洢·��
	bool bTracking;//�Ƿ���Ҫ����
	E_GET_ENDPOINT_METHOD eGetStartMethod;//����ȡ��ʽ
	bool bFixedPointScanStart;//��㶨����
	E_GET_ENDPOINT_METHOD eGetEndMethod;//�յ��ȡ��ʽ
	bool bFixedPointScanEnd;//�յ㶨����
	bool bIsTrackingScanEnd;//�Ƿ񺸽ӹ�����������βλ��
	bool bIsMiddleMeasure;//�м�λ���Ƿ���Ҫ����
	bool bIsClosedRing;//�Ƿ��Ǳպϼ�ǿȦ
	int  nOpenTrackingNo;//�������ٵ�λ
	int  nCloseTrackNo;//�رո��ٵ�λ
	XI_POINT tStartPoint;//���
	XI_POINT tEndPoint;//�յ�
	E_WRAPANGLE_TYPE eWarpType;// ��������
	vector<T_ROBOT_COORS> vRealTrackCoors;//ʵ�ʺ��ӹ켣 ����
	vector<int> vPointType;//������
}T_BOARD_PART_INFO;// ����������Ϣ

typedef struct
{
	string strfileName;//���ݴ洢·��
	bool bTracking;//�Ƿ���Ҫ����
	E_GET_ENDPOINT_METHOD eGetStartMethod;//����ȡ��ʽ
	bool bFixedPointScanStart;//��㶨����
	E_GET_ENDPOINT_METHOD eGetEndMethod;//�յ��ȡ��ʽ
	bool bFixedPointScanEnd;//�յ㶨����
	bool bIsTrackingScanEnd;//�Ƿ񺸽ӹ�����������βλ��
	bool bIsMiddleMeasure;//�м�λ���Ƿ���Ҫ����
	bool bIsClosedRing;//�Ƿ��Ǳպϼ�ǿȦ
	int  nOpenTrackingNo;//�������ٵ�λ
	int  nCloseTrackNo;//�رո��ٵ�λ
	XI_POINT tStartPoint;//���
	XI_POINT tEndPoint;//�յ�
	int nContinueStartNo; // ����������ʼ����
	int nContinueNum; // �������Ӹ���
	E_COUNTINUOUS_WELDING_TYPE eContinueType;//������������
	vector<bool> vbBoardDoesItExist;// �����Ƿ����
	vector<T_BOARD_PART_INFO> vtBoardPartInfo;//����������Ϣ�����������ȷ�����յ��ж�����
	VT_CORNER_LINE_INFO_NEW vtCornerRelsve_Info;//0������1����������2���յ����.��ǿȦʱ��˳�����
	VT_CIRCLE_CENTER_INFO_NEW vtCenter_Info;//Բ��
	VVT_ENDPOINT_INFO_NEW vtEndpoint_info;//�˵���Ϣ����ʼֵ
	vector<E_MEASURE_TRACK> vtMeasureTrack; // �����켣
	vector<vector<T_ROBOT_COORS>> vvtMeasure_Info;
	vector<vector<T_ANGLE_PULSE>> vvtMsePulse_Info;
	vector<vector<T_ALGORITHM_POINT>> vvtMeasureResult_Info;// ��ʵ�������
	vector<vector<T_ALGORITHM_POINT>> vvtMeasureResult_Warp;// һ�ΰ��ǲ������
	vector<vector<XI_POINT>> vvtheoryTrack;
	vector<vector<T_ROBOT_COORS>> vvtheoryTrackRobotCoors;//���۹켣
	vector<vector<T_ROBOT_COORS>> vvRealTrackRobotCoors;//ʵ�ʺ��ӹ켣
	vector<vector<T_ROBOT_COORS >> vvtMeasure_Data_Start;//�������ݵ����� ���
	vector<vector<T_ROBOT_COORS >> vvtMeasure_Data_Mind;//�������ݵ����� �м�
	vector<vector<T_ROBOT_COORS >> vvtMeasure_Data_End;//�������ݵ����� �յ�
	vector<vector<T_ROBOT_COORS >> vvtMeasure_Data_Warp;//�������ݵ����� ����
	vector<vector<T_ROBOT_COORS >> vvtMeasure_Data_ProStart;//�������ݵ����� ͶӰɨ���
	vector<vector<T_ROBOT_COORS >> vvtMeasure_Data_ProEnd;//�������ݵ����� ͶӰ������
	vector<vector<T_ROBOT_COORS >> vvtMeasure_Data;
	vector<vector<T_ANGLE_PULSE>> vvtMeasure_Pulse_Start;	//������̬ ���
	vector<vector<T_ANGLE_PULSE>> vvtMeasure_Pulse_Mind;	//������̬ �м��
	vector<vector<T_ANGLE_PULSE>> vvtMeasure_Pulse_End;		//������̬ β��
	vector<vector<T_ANGLE_PULSE>> vvtMeasure_Pulse_Warp;	//������̬ ���ǵ�
	vector<vector<T_ANGLE_PULSE>> vvtMeasure_Pulse_ProStart;//������̬ ͶӰ���
	vector<vector<T_ANGLE_PULSE>> vvtMeasure_Pulse_ProEnd;	//������̬ ͶӰ�յ�
	vector<T_ROBOT_COORS > vtMiddleMeasure_Data;//�������ݵ�����
	vector<T_ROBOT_COORS > vtRealWeldTrackCoors;//�������ӹ켣���������ӻ��������ӣ����չ켣,������ȫ��ņ̀ǹλ��
	vector<T_ANGLE_PULSE > vtRealWeldTrackPulse;//�������ӹ켣���������ӻ��������ӣ����չ켣
	vector<int> vnDataPointType;// ���ݵ�����
	vector<vector<T_ROBOT_COORS>> vvtTrackWarpCoors; //����ʱ�洢��������
	vector<vector<int>> vvnTrackWarpPtType;//����ʱ�洢������������

}T_BOARD_INFO_NEW;



class CDiaphragmWeld :public WeldAfterMeasure
{
public:
	CDiaphragmWeld(CUnit* ptUnit, E_WORKPIECE_TYPE eWorkPieceType);
	virtual ~CDiaphragmWeld();

public:
	virtual void SetRecoParam() override;
	virtual bool PointCloudProcess(bool bUseModel, CvPoint3D64f* pPointCloud = NULL, int PointCloudSize = 0) override;
	virtual bool WeldSeamGrouping(int& nWeldGroupNum) override;
	virtual bool CalcMeasureTrack(int nGroupNo, std::vector<T_ROBOT_COORS>& vtMeasureCoord, std::vector<T_ANGLE_PULSE>& vtMeasurePulse, vector<int>& vnMeasureType, double& dExAxlePos, double& dSafeHeight) override;
	virtual bool CalcWeldTrack(int nGroupNo) override;

public:
	// ���ٰڶ�
	bool SwingTracking(CRobotDriverAdaptor* pRobotDriver, std::vector<T_ROBOT_COORS> vtRobotCoors, double speed);
	//����ͱ��溸��(һ���ֹ켣)
	void SeparateAndSaveWeldingPath(std::vector<T_ROBOT_COORS> vtWorldCoors);
	//����ͱ��溸��(����)
	void SeparateAndSaveWeldingPath(T_ROBOT_COORS tWorldCoors);
	//��������յ���Ϣ(��¼��ʼ����յ�)
	void SaveSEPtn(T_ROBOT_COORS tStartPtn, T_ROBOT_COORS tEndPtn);
	//���ͺ�������Ҫ����Ϣ
	bool SendWeldData();
	//���ͺ�������
	void SendCoors();
	//д��ڶ� (����1 ������ʵ���˶�����)(����ֱ�ߺ���)
	void WriteSwing(T_ROBOT_COORS tRobotCoors);
	//�ֽ��ٶ� ( �������������1 �켣��� ����2 �켣�յ� ����3�����ٶ� ������� ����4�ⲿ���ٶ� ����5�������ٶ�)
	void DecomposeSpeed(T_ROBOT_COORS tStartPytn, T_ROBOT_COORS tEnd, double nSpeed, double& dMainSpeed, double& dSecSpeed);
	//����ⲿ���Ƿ񵽴��յ�(�жϺ����Ƿ�ֹͣ���� ����true���Ѿ������յ�)
	bool isAxisToEnd();
	//�ϲ�����
	void UpdateRealRobotCoors();
	//ʵʱ���ڶ��ṹ���еĺ�����Ϣ����������еĺ�����Ϣһ��(�߳�)
	static UINT ThreadCheckSwingConsistency(void* pParam);
	//д��ڶ�����������1Ƶ�� ����2�����
	void WriteSwingParam(double dSwingFrequency, double dSwingLeftAmplitude, double dSwingRightAmplitude);
	//�ⲿ���˶���Ϣ�߳�
	//static UINT ThreadAxisData(void* pParam);
	////�������˶���Ϣ�߳�
	//static UINT ThreadRobotData(void* pParam);
	//�����߳�(��Ҫ�޸�)
	static UINT ThreadMainContData(void* pParam);
	//������Ϣ����
	SwingTrackWeldParam m_swingTrackWeldParam;
	//�ڶ�����״̬λ(Ϊ�˷�ֹ�ظ����ø�������) true��û���� false�����ڵ���
	bool m_bSwingState;
	std::vector<CUnit*> m_vpUnit;							//���Ƶ�Ԫ����
	CUnitDriver* pUnitDriver;
	//�߳�ʹ��
	CEvent g_RobotState;		 //���ƻ������˶���Ϣ�̹߳���ͻָ�
	CEvent g_AxisState;			 //�����ⲿ���˶���Ϣ�̹߳���ͻָ�
	CEvent g_RobotShutdownEvent;	//���ƻ������˶���Ϣ�߳̽���
	CEvent g_AxisShutdownEvent;		//�����ⲿ���˶���Ϣ�߳̽���
	clock_t testTime; //����ʹ��
	CWinThread* pMainThread;	//�����߳̿���ָ��
	CWinThread* pRobotThread;	//�������߳̿���ָ��
	CWinThread* pAxisThread;	//�ⲿ���߳̿���ָ��
	CWinThread* pUpdataThread;	//ʵʱ�����߳̿���ָ��
	ESTUN_TRACK_PARAM estunParam;//��˹�ٻ�����ʵʱ���ٵĲ���


public:
	//���¹�������ֵ
	void WriteLimitHight(double dMaxH, double dMinH);
	void LoadLimitHight();
	//���ҹ�����Ϣ
	T_CORNER_LINE_INFO_NEW  FindCornerLineInfoFun(int nBoardNo, VT_CORNER_LINE_INFO_NEW* pvtCornerLineInfo);
	T_ENDPOINT_INFO_NEW		FindEndPointInfoFun(int nEndPointNo, VT_ENDPOINT_INFO_NEW* pvtPointInfo);
	vector<XI_POINT>   GetMeasurePointFun(int nBoardNo, VT_MEASURE_INFO_NEW* pvMeasureInfo);
	T_VERTICAL_INFO_NEW		FindVercalInfoFun(int nBoardNo, VT_VERTICAL_INFO_NEW* ptVerticalInfo);
	T_CIRCLE_CENTER_INFO_NEW FindCenterPointInfoFun(int nEndPointNo, VT_CIRCLE_CENTER_INFO_NEW* pvtPointInfo);
	// ����Բ��
	bool CorrectCenterPointInfoFun(int nEndPointNo, XI_POINT tCenTerNew, VT_CIRCLE_CENTER_INFO_NEW* pvtPointInfo);

	bool FindRelevanceVerticalFun(T_VERTICAL_INFO_NEW tVerticalInfo, VT_VERTICAL_INFO_NEW* ptVerticalInfo, VT_ENDPOINT_INFO_NEW* pvtPointInfo, VT_CORNER_LINE_INFO_NEW* pvtCornerLineInfo, T_VERTICAL_INFO_NEW& tVertical);
	//���Ⱥ���
	//bool DiaphragmWeld();//�ⲿ����
	//static UINT ThreadDiaphragmWeldProject(void* pProjec);
	//bool DiaphragmWeldProject(CRobotDriverAdaptor* pRobotDriver);

	//���캸�Ӻ���
	//bool VerticalWeldFunNew(CRobotDriverAdaptor* pRobotDriver, T_VERTICAL_INFO_NEW tVerticalInfo, VT_VERTICAL_INFO_NEW* pvtVerticalInfo,
	//	VT_CORNER_LINE_INFO_NEW* pvtCornerLineInfo, VT_ENDPOINT_INFO_NEW* pvtPointInfo, VT_MEASURE_INFO_NEW* pvMeasureInfo);
	//��ȡ���������Ϣ
	bool GetVerticalReleveInfo(T_VERTICAL_INFO_NEW tVerticalInfo, VT_VERTICAL_INFO_NEW* pvtVerticalInfo,
		VT_CORNER_LINE_INFO_NEW* pvtCornerLineInfo, VT_ENDPOINT_INFO_NEW* pvtPointInfo, T_VERTICAL_RELEVE_INFO& tVerticalReleInfo);

	//bool ProduceVerticanWeld(CRobotDriverAdaptor* pRobotDriver, vector<T_ALGORITHM_POINT> vtLinePoints, double dTrackAngle, double dTrackAngleLeft);

	//�Ƿ캸�Ӻ���
	//bool CornerLineWeldFunNew(CRobotDriverAdaptor* pRobotDriver, T_CORNER_LINE_INFO_NEW tCornerLineInfo, VT_CORNER_LINE_INFO_NEW* pvtCornerLineInfo,
	//	VT_ENDPOINT_INFO_NEW* pvtPointInfo, VT_MEASURE_INFO_NEW* pvMeasureInfo, VT_CIRCLE_CENTER_INFO_NEW* pvtCenterInfo);

	//��һ��ǿȦ���庸��
	bool StrengthRingWeldFunNew(CRobotDriverAdaptor* pRobotDriver, T_CORNER_LINE_INFO_NEW tCornerLineInfo,
		VT_CORNER_LINE_INFO_NEW* pvtCornerLineInfo, VT_ENDPOINT_INFO_NEW* pvtPointInfo);

	//ѡ��������պ�Բ��
	bool SelectLinesAndCircle(CRobotDriverAdaptor* pRobotDriver, T_CORNER_LINE_INFO_NEW tCornerLineInfo, VT_ENDPOINT_INFO_NEW* pvtPointInfo, VT_CIRCLE_CENTER_INFO_NEW* pvtCenterInfo, double& dRadius);
	//�����˵���м������
	bool CalcCoordUsePosDirAngle(XI_POINT tPtn, double dDirAngle, double dExAxlePos, T_ROBOT_COORS& tRobotCoord, double dRx = 0.0, double dRy = -45.0);
	bool GenerateTrack(vector<XI_POINT>& vtTrack, XI_POINT tStart, double dStartAngle, XI_POINT tEndPoint, double dEndAngle, XI_POINT tCenter, double dStepDis = 1.0, bool bIsArc = false);
	bool GenerateTrack(vector<XI_POINT>& vtTrack, XI_POINT tStart, XI_POINT tEndPoint, XI_POINT tCenter, double dStepDis = 1.0, bool bIsArc = false);

	bool GetTeachTrackNew(CRobotDriverAdaptor* pRobotDriver, T_BOARD_INFO_NEW& tBoardInfo, int nIndex, vector<T_ROBOT_COORS>& vtTeachTrack, vector<T_ANGLE_PULSE>& vtMeasurePulse, vector<int>& vnMeasureType, T_ANGLE_PULSE* ptPrePulse = nullptr);
	// ��ȡ���۲����켣
	bool GetThoryMeasureTrack(CRobotDriverAdaptor* pRobotDriver, T_BOARD_INFO_NEW& tBoardInfo);
	// ��ȡʵ�ʵ�ʾ�̽��
	bool GetRealTeachData(CRobotDriverAdaptor* pRobotDriver, T_BOARD_INFO_NEW& tBoardInfo, VT_ENDPOINT_INFO_NEW* pvtPointInfo);
	// ����ʵ�ʵĺ��ӹ켣
	bool CalcRealWeldTrack(CRobotDriverAdaptor* pRobotDriver, T_BOARD_INFO_NEW& tBoardInfo);
	// ��ʼ����
	bool DoWeld(CRobotDriverAdaptor* pRobotDriver, T_BOARD_INFO_NEW& tBoardInfo);

	bool Weld(int nGroupNo);

	// ��¼��β���ݣ�һ�ΰ��Ǵ��ڶ�ν������ٵĽ�β����
	bool RecordEndPointCoors(CRobotDriverAdaptor* pRobotDriver, vector<T_ROBOT_COORS> vtWeldCoors, vector<int> vnWeldCoorsType, E_WRAPANGLE_TYPE eWarpType = E_WRAPANGLE_EMPTY_SINGLE);

	// �ֽ���������
	bool DecomposeWorldCoordinates(CRobotDriverAdaptor* pRobotCtrl, vector<T_ROBOT_COORS>& vtCoutCoors, vector<T_ROBOT_COORS> vtCinCoors, bool bIsLockExAxis = false,double dExAixsPos = -99999.0);

	// �������ǹ��ȫλ��
	bool AddSafeDownGunPos(vector<T_ROBOT_COORS>& vCoors, vector<int>& vnCoorType, double dAdjustDis, double dPieceHight);

	// ����һ�ΰ�������
	bool CalcWarpTrack(vector<T_ROBOT_COORS> vtEndpoint, vector<T_ROBOT_COORS>& vtWarpStart, vector<T_ROBOT_COORS>& vtWarpEnd);
	// �жϲ������Ƿ����
	bool JudjeMeasureIsavailable(T_ALGORITHM_POINT tStartPoint, T_ALGORITHM_POINT tEndPoint, vector<T_ALGORITHM_POINT> vtMaesureCoors,
		vector<T_ALGORITHM_POINT>& vtCoutPoint, double ThreShold = 20., bool bIsReverse = false);
	// ������
	static bool PointConnection(vector<T_ALGORITHM_POINT> vtCinPoint, vector<XI_POINT>& vtCoutPoint, double dStepDis = 1.0);
	// �������۹켣��ȡ��������
	bool GetMeasureCoors(CRobotDriverAdaptor* pRobotDriver, vector<T_ROBOT_COORS> vtWeldTrack, vector<T_ROBOT_COORS>& vtTeachTrack,
		vector<int>& vnMeasureType, bool bTracking = false, bool bIsArc = false, bool bStartType = false, bool bEndType = false, double dStepDis = 100.0, bool bExAxisMoving = true);
	// ת��������ߣ��������ǹλ�� ��������������������
	bool TransCameraToolAndCalcContinuePulse(CRobotDriverAdaptor* pRobotDriver, vector<T_ROBOT_COORS>& vtTeachTrack, vector<T_ANGLE_PULSE>& vtMeasurePulse, vector<int>& vnMeasureType, T_ANGLE_PULSE* tRefPulse = nullptr);
	// ʾ��ͬʱ�ƶ���
	bool DoTeachWithMove(CRobotDriverAdaptor* pRobotDriver, vector<T_ROBOT_COORS> vtTeachCoors, vector<T_ROBOT_COORS> vtTeachCoorsTest, vector<T_ANGLE_PULSE> vtTeachPulse,
		vector<int> vtMeasureType, vector<T_ALGORITHM_POINT>& vtTeachResult, bool bIsReverse = false);
	bool CalcContinuePulse(CRobotDriverAdaptor* pRobotDriver, T_BOARD_INFO_NEW& tBoardInfo/*vector<T_ROBOT_COORS> &vtRobotCoors*/, T_ANGLE_PULSE tStandardRefPulse/*, vector<T_ANGLE_PULSE> &vtRobotPulse*/);
	//��ȡ������Ϣ
	T_BOARD_INFO_NEW GetBoardInfo(CRobotDriverAdaptor* pRobotDriver, T_CORNER_LINE_INFO_NEW tCornerLineInfo,
		VT_CORNER_LINE_INFO_NEW* pvtCornerLineInfo, VT_ENDPOINT_INFO_NEW* pvtPointInfo, VT_CIRCLE_CENTER_INFO_NEW* pvtCenterInfo, CString strFile = "");
	//����ɨ������
	//bool SetScanPos(CRobotDriverAdaptor* pRobotDriver, double dNormalAgl, XI_POINT tPoint, bool bIsStart, bool bIsFixedPointScan = false);
	bool SetScanPos(CRobotDriverAdaptor* pRobotDriver, vector<T_ROBOT_COORS> vtCooors, vector<T_ANGLE_PULSE> vtPulses, vector<int> vTeachType);
	// ����������� 
	bool CalcScanPos(CRobotDriverAdaptor* pRobotDriver, double dNormalAgl, XI_POINT tPoint, vector<T_ROBOT_COORS>& vtScanCoors, vector<T_ANGLE_PULSE>& vtScanPulses,
		vector<int>& vtScanCoorsType, double& dExAxisPos, bool bIsStart, bool bIsFixedPointScan, T_ANGLE_PULSE* ptPrePulse = nullptr);
	//���ݲ��������ȡ����켣����һ��ֱ�߻��ߣ�
	bool CalcRealWeldTrackAccordingToMeasureData(CRobotDriverAdaptor* pRobotDriver, T_BOARD_INFO_NEW tBoardInfo, vector<T_ROBOT_COORS>& vtRealPoint);
	//�õ��������
	bool GetMeasureDataWithMoveNew(CRobotDriverAdaptor* pRobotDriver, T_BOARD_INFO_NEW& tBoardInfo, int nIndex, vector<T_ALGORITHM_POINT>& vtTeachResult);
	//��ȡ������Ϣ
	bool GetVerticalMeasureData(CRobotDriverAdaptor* pRobotDriver, T_VERTICAL_RELEVE_INFO tVerticalReleInfo,
		vector<T_ROBOT_COORS>& vtMeasurePoint, vector<T_ANGLE_PULSE>& vtMeasurePulse, vector<int>& vnMeasureType, double& dExAxis);

	bool GetVerticalMeasureDataNew(CRobotDriverAdaptor* pRobotDriver, T_BOARD_INFO_NEW& tBoardInfo, vector<T_ROBOT_COORS>& vtMeasurePoint
		, vector<T_ANGLE_PULSE>& vtMeasurePulse, vector<int>& vnMeasureType, double& dExAxisPos, int nIndex, bool bIsStart = true, T_ANGLE_PULSE* ptPrePulse = nullptr);
	// �������������켣
	bool CalcVerticalTrack(CRobotDriverAdaptor* pRobotDriver, T_VERTICAL_RELEVE_INFO& tVerticalReleInfo);
	// ���ݲ������ݼ��㽻�� ֱ�ߣ�ֱ�� ֱ�ߣ�Բ�� Բ����Բ��
	bool CalcMeasureIntersection(CRobotDriverAdaptor* pRobotDriver, T_BOARD_INFO_NEW& tBoardInfo, vector<T_TEACH_RESULT> vtTeachResult,
		double dExAxisPos, int nIndex, XI_POINT& tIntersection, bool bIsStart);

	//��ȡ���۲ο�����
	bool GetTheoryReferencePulse(CRobotDriverAdaptor* pRobotDriver, T_ROBOT_COORS tTeachCoord, T_ANGLE_PULSE& tTeachPulse);
	//��ʼ������״̬
	void RefreshWeldReleveParam(CRobotDriverAdaptor* pRobotDriver, int nWeldSeamNum);
	//ˢ�¹����˵�
	void RefreshCorrectEndPointFun(CRobotDriverAdaptor* pRobotDriver, int EndPointNum);
	//��¼�����˵�
	void RecordCorrectEndPointFun(CRobotDriverAdaptor* pRobotDriver, int EndPointNo, XI_POINT tpEndPoint);
	//��ȡ�����˵�
	bool ReadCorrectEndPointFun(CRobotDriverAdaptor* pRobotDriver, int EndPointNo, XI_POINT& tpEndPoint);
	//���¶˵�
	bool CorrectEndPoint(CRobotDriverAdaptor* pRobotDriver, T_BOARD_INFO_NEW tBoardInfo,
		VT_ENDPOINT_INFO_NEW* pvtPointInfo, int nIndex, XI_POINT tStartPoint, XI_POINT tEndPoint);

	//��¼����״̬
	void RecordWeldStatus(int nEndPointNo, bool bStatus);
	bool LoadWeldStatus(int nEndPointNo);
	//���ֱ��
	static T_LINE_PARA		 CalcLineParamRansac(vector<T_ALGORITHM_POINT> vtPoint, double dSelectedRatio);
	static T_LINE_PARA		 CalcLineParamRansac(vector<XI_POINT> vtPoint, double dSelectedRatio);
	static bool	 	 dCmpIncFunc(double dFirstValue, double dSecondValue);
	static	T_SPACE_LINE_DIR CalLineDir(XI_POINT tStartPoint, XI_POINT tEndPoint);
	static double			 CalDisPointToLine(XI_POINT tPoint, XI_POINT tSegmentStartPoint, XI_POINT tSegmentEndPoint);
	//���Բ
	bool CalCircleThreePoint(T_SPACE_CIRCLE_PARAM& tCircle, vector<XI_POINT> vtFitOutputPoints, double dStepDis, vector<XI_POINT>& vtOutputPoints);
	bool CalThreeDotToCircle(const XI_POINT& tFirstPoint, const XI_POINT& tSecondPoint, const XI_POINT& tThirdPoint, T_SPACE_CIRCLE_PARAM& CircleParam);
	// �㵽ֱ�ߵľ���
	double			 CalcDistancePointToLine(XI_POINT linePoint1, XI_POINT linePoint2, XI_POINT point);

	// ������ֱ���ϸ�һ����ߵķ�����󽻵�
	bool			 CalcLineLineIntersection(XI_POINT tStart, double dStartAngle, XI_POINT tEnd, double dEndAngle, XI_POINT& intersection);
	bool			 CalcLineLineIntersection(T_ROBOT_COORS tStart, double dStartAngle, T_ROBOT_COORS tEnd, double dEndAngle, XI_POINT& intersection);
	//��϶��Բ��
	bool			CorrectCircled(double center[3], double radius, vector<XI_POINT> vtPoint);
	//���ݲ�������������յ���ֱ��
	bool			 FitCalLinePointsByMiddlePoints(XI_POINT tStartPoint, XI_POINT tEndPoint, vector<XI_POINT> vtScanPoints, vector<XI_POINT>& vtOutPoints, double dThresVal = 20., double dStepDis = 1.0);
	bool			 FitCalLinePointsByMiddlePoints(XI_POINT tStartPoint, XI_POINT tEndPoint, vector<T_ALGORITHM_POINT> vtScanPoints, vector<XI_POINT>& vtOutPoints, double dThresVal = 20., double dStepDis = 1.0);

	bool			 FitCalLinePointsByMiddlePoints(XI_POINT tStartPoint, XI_POINT tEndPoint, vector<XI_POINT> vtScanPoints, vector<XI_POINT>& vtOutPoints, vector<XI_POINT>& vtPosture, double& dNormalAngle, int& nStartChangePointNo, int& nEndChangePointNo,
		double dDisPoint, int nPointNum, bool bCheckDisORNum, int nThresVal, bool bStartChangePosture = false, bool bEndChangePosture = false);

	bool			 FitCalLinePointsByMiddlePoints(XI_POINT tStartPoint, XI_POINT tEndPoint, vector<T_ALGORITHM_POINT> vtScanPoints, vector<XI_POINT>& vtOutPoints, vector<XI_POINT>& vtPosture, double& dNormalAngle, int& nStartChangePointNo, int& nEndChangePointNo,
		double dDisPoint, int nPointNum, bool bCheckDisORNum, int nThresVal, bool bStartChangePosture = false, bool bEndChangePosture = false);
	// �������۹켣������ʼ��ֹ��ת��̬�������Ҫת��Ƕ�
	/*
	* ����˵����
	* ��ɼн���ຸ��
	*/
	void			 GetChangePosturePare(vector<T_ROBOT_COORS> tLeftTrackCoors, XI_POINT tLeftCenter, double dLeftNormal, bool bLeftIsArc, vector<T_ROBOT_COORS> tRightTrackCoors, XI_POINT tRightCenter, double dRightNormal, bool bRightIsArc, bool bStartOrEnd,
		int& nStartStepNo, double& dStChangeAngle, double dChangePostureThresVal = 100.0);

	//���ݺ������ݼ����������̬

	bool TransRobotPostureCoors(vector<XI_POINT> vtOutPoints, vector<XI_POINT>& vtPosture, int& nStartChangePointNo,
		int& nEndChangePointNo, double dRX, double dRY, double dDisPoint, bool bStartChangePosture, bool bEndChangePosture);
	bool CalcRobotCoorsAccordingTrack(vector<XI_POINT> vtCinPoints, vector<T_ROBOT_COORS>& vtCoutCoors);
	//����T_ALGORITHM_POINT�������ݵ��ļ�
	void			SaveDataAlgorithm(vector<T_ALGORITHM_POINT> vcPointOnTheSameLine, string str);
	//����XI_POINT�������ݵ��ļ�
	void			SaveDataXiPoint(vector<XI_POINT> vcPointOnTheSameLine, string str);
	//����XI_POINT�������ݵ��ļ�
	void			SaveData(vector<XI_POINT> vcPointOnTheSameLine, string str);
	void            SaveDataTrackFilt(vector<TrackFilter_XYZ> vtPoints, string str);
	void			SaveTrackFilterCoors(vector<TrackFilter_XYZ> vcPointOnTheSameLine, string str);
	void            SaveDataRobotCoors(vector<T_ROBOT_COORS> vcPointOnTheSameLine, string str);
	void            SaveDataRobotPulse(T_ANGLE_PULSE tPulse, string str);
	void			SaveDataInt(vector<int> vnCoors, string str);
	//��������
	void			LoadDataXiPoint(vector<XI_POINT>& vcPointOnTheSameLine, string str);
	void			LoadDataAlgorithm(vector<T_ALGORITHM_POINT>& vcPointOnTheSameLine, string str);
	void            LoadDataRobotCoors(vector<T_ROBOT_COORS>& vcPointOnTheSameLine, string str);
	void		    LoadDataRobotCoors(vector<T_ROBOT_COORS> &vcPointOnTheSameLine, vector<int> &vnCoorsType, string str);
	void            LoadDataRobotPulse(T_ANGLE_PULSE& tPulse, string str);
	void			LoadDataInt(vector<int>& vnCoors, string str);
	void			LoadTrackFilterCoors(vector<TrackFilter_XYZ>& vcPointOnTheSameLine, string str);
	//���غ�������
	void LoadWeldLineData(E_WELD_LINE& vcPointOnTheSameLine, CString str);
	void GetCircleRealWeldPoint(CRobotDriverAdaptor* pRobotDriver, vector<T_ROBOT_COORS>& realWeldPoint, E_WELD_LINE vcPointOnTheSameLine, double dMachineCarMovePos);
	//ת����������
	void TransiformData(vector<XI_POINT> vcPointOnTheSameLine, vector<XI_POINT>& vtPoints);
	void TransiformMeasureData(vector<XI_POINT> vtPoints, vector<XI_POINT>& vcPointOnTheSameLine);
	void TransALGORITHMToXiPoint(vector<T_ALGORITHM_POINT> vtALGORITHMPoints, vector<XI_POINT>& vtPoints);
	void TransXiPointToALGORITHM(vector<XI_POINT> vtPoints, vector<T_ALGORITHM_POINT>& vtALGORITHMPoints);
	void TransMeasureToAlgorithm(vector<XI_POINT> vcPointOnTheSameLine, vector<T_ALGORITHM_POINT>& vtALGORITHMPoints);
	void TransAlgorithmToMeasure(vector<T_ALGORITHM_POINT> vtALGORITHMPoints, vector<XI_POINT>& vcPointOnTheSameLine);

	//����ƫ��ֵ ,����������
	void   CalcMeasurePosInBase(vector<XI_POINT> tMeasurePoint, double dDis, double dZDis, double dAngleVer, vector<XI_POINT>& tMeasureTrans);
	void   CalcMeasurePosInBase(vector<XI_POINT> tMeasurePoint, double dDis, double dZDis, vector<XI_POINT> tPosture, vector<XI_POINT>& tMeasureTrans);
	void   CalcOffsetInBase(XI_POINT tMeasurePoint, double dDis, double dMoveZ, double dAngleVer, XI_POINT& tMeasureTrans, bool _isDown = false);
	void   CalcMeasureInBase(XI_POINT tMeasurePoint, double dDis, double dMoveZ, double dAngleVer, T_ROBOT_COORS& tMeasureTrans);
	void   CalcMeasureInBase(T_ROBOT_COORS tMeasurePoint, double dDis, double dMoveZ, T_ROBOT_COORS& tMeasureTrans);
	//��е������ϵ��
	double CalcRobotAngleRZInBase(double dTrackVerDegree);
	//ͼ����
	void MeasurePointLaserData(CRobotDriverAdaptor* pRobotDriver, CString str, T_ALGORITHM_POINT& vcPointOnTheSameLine, vector<T_ALGORITHM_POINT>& vcpUpPlatePoints, vector<T_ALGORITHM_POINT>& vcpDownPlatePoints, T_ANGLE_PULSE tRobotPulses, T_ROBOT_COORS tRobotCoord, int nImageType, int nCameraNo);
	void MeasureVerLaserPoint(CRobotDriverAdaptor* pRobotDriver, T_ALGORITHM_POINT& tFirstPointOnLine, vector<T_ALGORITHM_POINT>& vcpUpPlatePoints,
		vector<T_ALGORITHM_POINT>& vcpDownPlatePoints, T_ANGLE_PULSE tRobotPulses, T_ROBOT_COORS tRobotCoord, int nImageType, int nCameraNo);
	void GetVerWeldLaserLineImageCoors(CvPoint& tCornerImagePoint, vector<CvPoint>& vtLeftImagePoints, vector<CvPoint>& vtRightImagePoints, int nImageType, int nCameraNo);
	void SampleImagePoints(vector<CvPoint> vtImagePoints, int nSampleNum, vector<CvPoint>& vtSampleImagePoints);

	vector<T_ROBOT_COORS> GetRealWeldData(CRobotDriverAdaptor* pRobotDriver, vector<XI_POINT> tWeldPointData, vector<XI_POINT> tWeldPosture);
	vector<T_ROBOT_COORS> GetRealWeldData(CRobotDriverAdaptor* pRobotDriver, vector<T_ROBOT_COORS> tWeldPointData);

    
    //����ǰ���ݴ���������ʼ�β�������׼��ǹ
    //bool WeldProcessBefor(CRobotDriverAdaptor *pRobotDriver, vector<T_ROBOT_COORS> vtRealWeldCoors, bool bIsTracking);

	bool WeldProcessBefor(CRobotDriverAdaptor* pRobotDriver, vector<T_ROBOT_COORS> vtRealWeldCoors, vector<int> vtRealWeldCoorType,
		vector<T_ANGLE_PULSE> vtRealWeldPulse,bool bIsTracking);
	// ��ȡ�ض�����
	bool GetTruncatedData(CRobotDriverAdaptor* pRobotDriver, T_ROBOT_COORS tRobotRealPos, MoveHandle_Pnt& tRobotCoor, long& lRobotVel, long& lMachinePos, long& lMachineVel);
	//���ͺ�������
	bool CallJobRobotWeld(CRobotDriverAdaptor* pRobotDriver, vector<XI_POINT> vcPointOnTheSameLine, vector<XI_POINT> vcWeldPosture, bool bCalcPosture, bool bIfWeld);
	//���˶���������е�ۻ�ʹ�ͬʱ�ƶ�,bIsTracking:trueʵʱ���٣�false�Ȳ��
	bool DoweldRuning(CRobotDriverAdaptor* pRobotDriver, vector<T_ROBOT_COORS>vtRealWeldCoors, vector<int> vnPointType, bool bIsTracking, E_WELD_SEAM_TYPE eWeldSeamType = E_FLAT_SEAM);
	
	// ���������˺������ⲿ������˶�
	bool TrackingWeldMoveForPanasonic(CRobotDriverAdaptor* pRobotDriver, vector<T_ROBOT_COORS> vtRealWeldCoors, vector<int> vnPointType, bool bIsTracking, E_WELD_SEAM_TYPE eWeldSeamType);
	// ���������㷨�岹�㷨��ʼ��
	void InitMoveHandleYaskawaPanasonic(T_ROBOT_COORS tTrackingStartCoord, double dWeldSpeed);

	bool DoweldRuningSwing(CRobotDriverAdaptor* pRobotDriver, vector<T_ROBOT_COORS>vtRealWeldCoors, vector<int> vnPointType, bool bIsTracking);
	//��˹�ٸ��������߳�
	static UINT ThreadMainContData4Estun(void* pParam);
	//��˹�ٰڶ��������� waveType == 0 ���ڶ��� wave Type == 1ǰ��ڶ�
	bool EstunTracking(CRobotDriverAdaptor* pRobotDriver, std::vector<T_ROBOT_COORS> vtRobotCoors, double dSpeed, int waveType);

	bool SendDataTOCallJobRobot(CRobotDriverAdaptor* pRobotDriver, vector<T_ROBOT_COORS> vtRealWeldCoors);
	//ֱ�������˶�����(������)
	bool MoveByJob(CRobotDriverAdaptor* pRobotDriver, T_ROBOT_COORS tRobotJointCoord, T_ROBOT_MOVE_SPEED tPulseMove, int nExternalAxleType, CString JobName = "MOVJ", bool bSingleMove = false);
	//�ؽ������˶�����(������)
	bool MoveByJob(CRobotDriverAdaptor* pRobotDriver, T_ANGLE_PULSE tRobotJointCoord, T_ROBOT_MOVE_SPEED tPulseMove, int nExternalAxleType, CString JobName = "MOVJ", bool bSingleMove = false);
	// ���ƶ�
	bool PosMoveMachine(CRobotDriverAdaptor* pRobotDriver, double dMoveDis, double dSpeed, bool bCheckDone = true);
	//����󳵺ͻ�е��λ��
	//����bRobotDir��Ϊtrueǿ�ƻ�е�����ұߣ�false,�����豸���޼��㣬nIsExAxis����ʾ�Ƿ����ⲿ��
	//Ĭ�ϻ�е����X��������У����ǹ������Ҳ༫��λ��
	bool CalcRobotAndMachinePos(CRobotDriverAdaptor* pRobotDriver, T_ROBOT_COORS WorldCoors, T_ROBOT_COORS& tCoutRobotCoor);
	//��ȡλ��(������)
	T_ANGLE_PULSE GetCurrentPulse(CRobotDriverAdaptor* pRobotDriver);
	T_ROBOT_COORS GetCurrentPos(CRobotDriverAdaptor* pRobotDriver);

	void    IntValWaitFun(CRobotDriverAdaptor* pRobotDriver, int nIntVal, int nVal);
	bool    m_bRobotWeldDir;//�����˺��ӷ���Ĭ����X�����
	bool WeldLockBefor(CRobotDriverAdaptor * pRobotDriver, vector<T_ROBOT_COORS> tCoors, vector<int> tCoorsType, vector<T_ROBOT_COORS>& tCoutCoors, bool bIsTrack);
	//����ǰ���������ڱ���̬
	//bool	WeldLockBefor(CRobotDriverAdaptor* pRobotDriver, T_ROBOT_COORS tStartP, T_ROBOT_COORS tEndP, bool bIsTrack = false);
    bool    WeldLockBefor(CRobotDriverAdaptor* pRobotDriver, vector<T_ROBOT_COORS> tCoors, vector<T_ROBOT_COORS>& tCoutCoors, bool bIsTrack);

	typedef struct
	{
		CString strRobotName;
		vector<T_ANGLE_PULSE> vtSafePulse;
	}T_STREET_LAMP;
	//���ⲿ��
	vector<T_STREET_LAMP> m_vtStreetLamp;
	//�ⲿ��
	int m_nHandlingExternalAxisSafeMove;						//������̨����
	vector<T_ANGLE_PULSE> m_vtHandling_ExternalAxisPulse;		//��ȫλ��

	/*
	**��ɨ���ݸ�ʽ���ģ�ɸѡ������Ϣ
	**���飺����ͬʱ���ӵ�ƽ�죨ֱ�߻�Բ�������з���������ӣ��㹻���ĺ�������Ϊһ�飬��ȫ�ڻ�е���г��ڵ������ĺ����ɶ��Ϊһ�飬ÿ�����쵥��Ϊһ����
	**���㵥�������Ϣ������ʱ�������ҳ���С�����۾���1.5�����ҵĶ̱߲���ʱ��Ҫ������̬��λ�ý��в�����
	**��ʼ�����õ��������ݣ���Ҫ���µĶ˵㣨����ʵ�������ĺ�������ʱ���߱�������ɶ�С�̱߸��ݰ��ͶӰ�õ����׼ȷ�Ķ˵㣩
	**��ʼ���ӣ�����ʱ�ڻ�е���г��ڵ�ֻ����е�ۺ��ӣ�������е���г̵Ĵ���������
	**��ʼ��β����������ν������������ཻ�������ཻ��ö˵㣬���ɶ����������ķ�ʽ�õ�
	**���ȳ���500mm�ĺ����м�������Ӹ���
	*/
	//��ʼ������������
	void InitWeldStartVal(CRobotDriverAdaptor* pRobotDriver);
	void SetWeldTlyParam(T_WELD_PARA tWeldPara);
	void LoadWorkPieceInfo();										//���ع�����Ϣ
	void InitWorkPieceReleveInfo(CRobotDriverAdaptor* pRobotDriver);

	VVT_CORNER_LINE_INFO_NEW m_vvtCornerLineInfoNew;
	VVT_ENDPOINT_INFO_NEW m_vvtPointInfoNew;
	VVT_VERTICAL_INFO_NEW m_vvtVerticalInfoNew;
	VVT_MEASURE_INFO_NEW m_vvMeasureInfoNew;
	VVT_CIRCLE_CENTER_INFO_NEW m_vvtCircleCenterInfo;

	double m_dWorkPieceHighTop;										// ������ߵ�
	double m_dWorkPieceLowTop;										// ������͵�
	double m_dRobotHighTop;											// ��������ߵ�
	double m_dRobotHLowTop;											// �����˵͵�
	double m_dWeldDipAngle;											// �������
	double m_dParallelAdjust1;										// ���ǲ���
	double m_dVerticalAdjust1;
	double m_dVelocityWarp;											// �����ٶ�
	double m_dParallelAdjust2;
	double m_dVerticalAdjust2;
	double m_dVelocityWarp2;										// �����ٶ�
	double m_dVelocityTransitionBound;
	

	WELD_WRAP_ANGLE m_tWrapAngelParaNew;

	/*----------------------------------------�ϵ�����------------------------------------------*/
	void        LoadEndpointData(CRobotDriverAdaptor* pRobotDriver, T_ROBOT_COORS& tEndpoint, bool bIsStart);
	void        SaveWeldLengthData(CRobotDriverAdaptor* pRobotDriver, double dWeldLength, double dAbjustRz, int nWeldStepNo);
	void        LoadWeldLengthData(CRobotDriverAdaptor* pRobotDriver, double& dWeldLength, double& dAbjustRz, int& nWeldStepNo,double &dVoerLength);
	void        SaveEndpointData(CRobotDriverAdaptor* pRobotDriver, T_ROBOT_COORS tEndpoint, bool bIsStart);
	void        LoadWorkpieceNo(CRobotDriverAdaptor* pRobotDriver, int& nWorkpieceNo);
	void        SaveWorkpieceNo(CRobotDriverAdaptor* pRobotDriver, int nWorkpieceNo);
	void        LoadWorkpieceType(CRobotDriverAdaptor* pRobotDriver);
	void        SaveWorkpieceType(CRobotDriverAdaptor* pRobotDriver);

	void		SaveWarpNumber(CRobotDriverAdaptor* pRobotDriver, int nWarpNumber);
	void		LoadWarpNumber(CRobotDriverAdaptor* pRobotDriver, int& nWarpNumber);
	void		LoadCurrentWarpNumber(CRobotDriverAdaptor* pRobotDriver, int &nWarpNumber);
	void		SaveCurrentWarpNumber(CRobotDriverAdaptor* pRobotDriver, int nWarpNumber);
	/*------------------------------------------------------------------------------------------*/
	void SetThreadStatus(E_INCISEHEAD_THREAD_STATUS eStatus);
	bool CheckSafeFun(CRobotDriverAdaptor* pRobotDriver);

	// �л����ӺͰ����ٶ�
	void SwitchWeldSpeed(CRobotDriverAdaptor* pRobotDriver, int nCoorIndex, int nCoorType, double dWeldSpeed, double dWarpSpeed, long& lRobotvel, bool& bIsChangeSpeed);
	// �л����ӺͰ��ǵ�����ѹ
	void SwitchWeldCurrent(CRobotDriverAdaptor* pRobotDriver, int nCoorIndex, int nCoorType, T_WELD_PARA tWeldParam, bool& bChangeVoltage);
	// ������������
	bool CorrectWarpDataFun(vector<TrackFilter_XYZ> vWeldCoors, vector<T_ROBOT_COORS> vtWarpCoors, int nWarpIndex, vector<T_ROBOT_COORS>& vtCorrectWarpCoors);
	bool CorrectWarpDataFun(vector<TrackFilter_XYZ> vWeldCoors, vector<T_ROBOT_COORS> vtWarpCoors, int nWarpIndex, vector<T_ROBOT_COORS>& vtCorrectWarpCoors, vector<int>& vtCorrectWarpCoorsType);

	
public:
	CMoveCtrlModule         m_cMoveCtrl;
	//��ֲ�¿��ʱ����
	BOOL m_bWorking;
};

class correctFilletError {
public:

	// - P_ Ϊ���ٵ㼯��ѡȡ���ǵڶ���ǰ��һ�θ��ٵ㼯
	// - filletP_ Ϊ���۰��ǵ��ĵ㣬��һ��Ϊ��ʼ�㣬�ڶ���Ϊ��һ��ת��㣬������Ϊ�ڶ���ת��㣬���ĵ�Ϊ������
	// - diffD1_ Ϊ�ָ� �ɵ�һ�����ڶ��� �� �ɵ����������ĵ� �������߶Σ���ƽ���ߣ��ķָ��ȣ���λΪ mm
	// - diffD2_ Ϊ�ָ� �ɵڶ����������� ���߶εķָ��ȣ���λΪ mm
	// - minNum2_ Ϊ�ָ� �ɵڶ����������� �߶εķָ��������ޣ���λΪ ��
	// - permissibleError_ Ϊ���ֱ��ʱʹ�õľ��Ȳ�������λΪ mm ����������£���ֵԽС����Խ�ߣ�����С����ֵ���ܻᵼ���޷���ϣ���ʱ��Ĭ��ʹ�ø��ٵ㼯��ǰ���������ֱ�ߣ�Ӱ�쾫�ȡ��ο���Χ��0.5--1��
	// - permissibleRate_ Ϊ���ֱ��ʱ��Ŀ�꾫�Ȳ�������λΪ 100% ����������£���ֵԽ�󾫶�Խ�ߣ�������ֵ���ܻ�Ӱ����������ٶȡ��ο���Χ��0.8-0.95��
	correctFilletError(std::vector<XI_POINT> p_, std::vector<XI_POINT> filletP_, double diffD1_, double diffD2_, int minNum2_, double permissibleError_ = 0.55, double permissibleRate_ = 0.9);

	std::vector<XI_POINT> getRes();

	void getResNew();
	// nRobotDir:�����˰�װ����
	std::vector<XI_POINT> GetGdiff(std::vector<XI_POINT> vtRobotPostureCoors, std::vector<XI_POINT>& vtRobotPosture, int nRobotDir = 1);

private:

	double distance(size_t a, size_t b);
	double distance(const XI_POINT& a, const XI_POINT& b);

	// ��ϸ��ٵ㼯ֱ��
	void fittedLine();

	// ����㼯�������������ֱ�ߵľ���
	double disToLine(size_t tempFittedStartPointIndex, size_t tempFittedEndPointIndex, size_t C);

	// ����У����İ��ǵ�
	void correctFillet();

	// ��start����end��������ϸ��ݾ���start��ľ����ҵ�
	XI_POINT findPoint(const XI_POINT& start, const XI_POINT& end, double dis);

	// �������ֱ�߼���ǰ����
	XI_POINT find0and1(size_t n);

	// �����ĵ㼯��ǰ������������
	XI_POINT find2and3(size_t n);

	// ��У׼����ĵ������ɢ�㼯
	std::vector<XI_POINT> diff();

private:
	//------------------------------------����-----------------------------------

	// ���ٵ㼯
	const std::vector<XI_POINT> p;

	// ��ɨ���۰����ĵ㣨��ʼ�㣬�յ㣬�յ㣬�����㣩
	const std::vector<XI_POINT> filletP;

	// ��ϸ��ٵ㼯ֱ��ʱ��������������/
	double permissibleError;

	// ��ϸ��ٵ㼯ֱ��ʱ���������/
	double permissibleRate;

	// ��ɢ��һ��������ߵľ��볣��
	const double diffD1;

	// ��ɢ�ڶ��ߵľ��볣��
	const double diffD2;

	// ��ɢ��ڶ������ٵ������������˵㣬��С��3��
	const int minNum2;

	// -------------------�м�ֵ--------------------------------------------------

	// У��������ĵ�
	std::vector<XI_POINT> correctFilletP;

	// ���ٵ㼯���ֱ�ߵ�һ�������
	size_t fittedStartPointIndex;

	// ���ٵ㼯���ֱ�ߵڶ��������
	size_t fittedEndPointIndex;
};
#endif // !defined(AFX_DIAPHRAGMWELD_H__27CA3DC8_2FB9_4286_8B23_8FBE2B7E4E60__INCLUDED_)
