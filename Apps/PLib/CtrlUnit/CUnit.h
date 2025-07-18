#pragma once
#include "CUnitDataManagement.h"
#include "UnitStruct.h"
#include ".\LocalFiles\ExLib\ALGO\include\MoveHandle_C_DLL.h"
#include ".\LocalFiles\ExLib\ALGO\include\TrackSmoothHandle_C_DLL.h"
#include "Apps/RealTimeTrack/RealTimeTrack.h"

#ifndef WRITE_LOG_COMMON
#define WRITE_LOG_COMMON(unit, data) {CString __str;\
__str.Format("Function:[%s] Line:[%d]	\ndata: %s",__FUNCTION__, __LINE__, data);\
unit->GetLog()->Write(__str);}
#endif

#ifndef MESSAGE_BOX_COMMON
#define MESSAGE_BOX_COMMON(unit, data) {CString __str;\
__str.Format("Function:[%s] Line:[%d]	\ndata: %s",__FUNCTION__, __LINE__, data);\
XiMessageBoxOk(__str);\
}
#endif

class CUnit 
	: public CUnitDataManagement
{
public:
	CUnit(T_CONTRAL_UNIT tCtrlUnitInfo, CServoMotorDriver *pServoMotorDriver = NULL);

	bool m_bNaturalPop = true;



	//�����˶��켣
	bool SetMoveValue(std::vector<T_ROBOT_MOVE_INFO> vtMoveInfo);
	bool SetMoveValue(CRobotMove cMoveInfo);
	int GetMoveStep();
	int CleanMoveStep();

	void CallJob(CString sJobName, int nExternalType);

	//����Ƚ�
	bool ComparePulse(T_ANGLE_PULSE tPulse1, T_ANGLE_PULSE tPulse2, long lLimit = 30);
	bool ComparePulse_Base(T_ANGLE_PULSE tPulse1, double dLimit = 1.0);

	//��Ԫ״̬���

	//��ͣ
	void UnitEmgStop();
	bool RobotEmg(bool bPopup = FALSE);
	bool RobotBackHome(T_ROBOT_MOVE_SPEED tSpeed = T_ROBOT_MOVE_SPEED(2000, 70, 70));
	bool MoveExAxleToSafe(T_ANGLE_PULSE* ptDstPulse = NULL);

	//���þ�������Ϊ��ǰ����
	T_RUNNING_RTN_RESULT SetAbsEncoderCoor();


	/**************** ������� Start ******************/
	// ��ȡ�����������Ӿ������ļ���
public:
	// ��ȡ�ⲿ�����嵱��
	double GetExPulseEquivalent(int nMoveXYZAxisNo);
	// �����ˣ������ȴ��˶�ֹͣ	
	bool CheckRobotDone(T_ROBOT_COORS tAimCoor, CString sInfo = "");
	bool CheckRobotDone(T_ANGLE_PULSE tAimPulse, CString sInfo = "");
	CString GetTrackTipDataFileName(int nCameraNo);

	// �������꣺��ȡ���ⲿ��λ��
	double GetPositionDis(); //��ѯ�ⲿ������ (��ŵ�ջ򰲴������ⲿ��)
	//// �������꣺��ȡֱ������
	//T_ROBOT_COORS GetCurrentPosWorld();
	//// �������꣺��ȡ�ؽ�����
	//T_ANGLE_PULSE GetCurrentPulseWorld();
	// �������꣺�˶� ���� �����ⲿ��ͨ��
	int WorldMoveByJob(T_ANGLE_PULSE tPulse, T_ROBOT_MOVE_SPEED tPulseMove, CString JobName);
	// �������꣺�Ƿ������˶�
	bool WorldIsRunning(); 
	// �������꣺�ȴ��˶�ֹͣ
	void WorldCheckRobotDone(int nDelayTime = 1200);
	// �������꣺�ⲿ���˶�����,��ʱ����ƶ������ⲿ��
	int MoveExAxisForLineScan(double dDist, int nSpeed,bool bIsMoveYasExAxis = false);
	// �������꣺�ⲿ���˶�����,�����������ж��ƶ���
	int MoveExAxisFun(double dDist, int nSpeed, int nMoveXYZAxisNo, double dMaxExSpeed = -1); // ����ⲿ���ٶȷ���չ�ⲿ���ٶ�mm/min С��0ʹ��nSpeed
	int XYZAxisNoToSoftAxisNo(int nMoveXYZAxisNo);
	// ��ȡ�ⲿ�����꣬����/����������
	double GetExPositionDis(int nMoveXYZAxisNo);
	// ��ȡ������,true:�����ⲿ�ᣬfalse;�ǰ����ⲿ��
	bool  GetExAxisType(int nMoveXYZAxisNo);
	// �����ˣ������ϵ�״̬ δ�ϵ�ʱ��ִ���ϵ�
	bool CheckIsReadyRun();	
	// �����ˣ��ƶ���һ���߶�(�밲ȫλ����������������й�)
	//bool MoveToSafeHeight();
	// Ӳ����ʾ���˶� �˶��ⲿ�� �� ������ (��һ������ⲿ��ͬʱ�˶� �� �ȶ��ⲿ���ٶ�������)
	bool TeachMove(const std::vector<T_ANGLE_PULSE>& vtMeasurePulse, double dExAxlePos, const  std::vector<int>& vnMeasureType, int nTrigSigTime);
	// �����˶� ()
	int WeldMove(const std::vector<T_ROBOT_COORS>& vtWeldPathPoints, const  std::vector<int>& vnPtnType, const T_WELD_PARA& tWeldPara, double dExAxlePos, E_WELD_SEAM_TYPE eWeldSeamType, bool bIsArcOn, int ToolNum = 1U);
	int WeldMove_BP(const std::vector<T_ROBOT_COORS>& vtWeldPathPoints, const  std::vector<int>& vnPtnType, const T_WELD_PARA& tWeldPara, E_WELD_SEAM_TYPE eWeldSeamType, bool bIsArcOn);
	int WeldMove_Scan(const std::vector<T_ROBOT_COORS>& vtWeldPathPoints, const std::vector<int>& vnPtnType, const T_WELD_PARA& tWeldPara, double dExAxlePos, E_WELD_SEAM_TYPE eWeldSeamType, bool bIsArcOn, int ToolNum);
	// ����:�������ù��ղ��� �Ƿ��𻡣�I058 �𻡰��Ǻ���ͣ��������ѹ��I080-I087 ͣ��ͣ��ʱ��:I70
	bool SendWeldParam(BOOL bIsArcOn, const T_WELD_PARA& tWeldParam);

	//�¿ں���
	bool SendGrooveParam(BOOL bIsArcOn, const T_WELD_PARA& tWeldParam);
	int GrooveWeldMove(const std::vector<T_ROBOT_COORS>& vtWeldPathPoints, const T_WELD_PARA& tWeldPara, double dExAxlePos, bool bIsArcOn, T_WAVE_PARA tWavePara, vector<double> weldSpeedRate, vector<double> vdWaveWaitTime = vector<double>(0));
	bool SendGrooveWeldData(BOOL bIsArcOn, const std::vector<T_ROBOT_COORS>& vtWeldCoord, const T_WELD_PARA& tWeldParam, bool& bIsCallJob, T_WAVE_PARA tWavePara, vector<double> weldSpeedRate, vector<double> vdWaveWaitTime = vector<double>(0));
private:
	// ����Ӳ����ʾ���˶����� nDownSpeed�ؽڲ岹�ٶ� nTeachSpeed �� nUpSpeed ֱ�߲岹�ٶ�
	bool SendTeachMoveData(const std::vector<T_ANGLE_PULSE>& vtMeasurePulse, const  std::vector<int>& vnMeasureType, int nDownSpeed, int nTeachSpeed, int nUpSpeed, int nTrigSigTime);
	// ��ȡ��һ�κ��ӻ���ǹ켣
	bool GetNextSectionTrack(const std::vector<T_ROBOT_COORS>& vtSrcCoord, const  std::vector<int>& vnPtnType, int& nCursor, std::vector<T_ROBOT_COORS>& vtWeldCoord, int& nPtnType);
	// �����𻡹켣����
	bool SendArcOnData(const  std::vector<T_ROBOT_COORS>& vtWeldCoord, const T_WELD_PARA& tWeldParam, int ToolNum = 1U);
	// ���Ͱ��ǹ켣����
	bool SendWrapData(const  std::vector<T_ROBOT_COORS>& vtWeldCoord, const T_WELD_PARA& tWeldParam/*, const WELD_WRAP_ANGLE& tWrapParam*/);
	// ���ͺ��ӹ켣����(����40����ѭ������)
	bool SendWeldData(std::vector<T_ROBOT_COORS>& vtWeldCoord, const T_WELD_PARA& tWeldParam, E_WELD_SEAM_TYPE eWeldSeamType, bool& bIsCallJob, int ToolNum = 1U); // ѭ������																																		 // ����ڶ��ο��㣨2023/12/21������ռ��
	bool SendWeldData_BP(std::vector<T_ROBOT_COORS>& vtWeldCoord, const T_WELD_PARA& tWeldParam, E_WELD_SEAM_TYPE eWeldSeamType, bool& bIsCallJob); // ѭ������																																		 // ����ڶ��ο��㣨2023/12/21������ռ��
	void CalcSwayRefpCoord(T_ROBOT_COORS tStartCoord, E_WELD_SEAM_TYPE eWeldSeamType, T_ROBOT_COORS& tRefp1, T_ROBOT_COORS& tRefp2);

public:
	//****************����С��������ת�� ****************//
	int CaliTranMat(std::vector<cv::Mat>& vGantryTransRobotMat, std::vector<cv::Mat>& vRobotTransGantryMat, bool save = false);	//����ת�����󣨲ɼ��ĸ�������ĵ����������ļ��м��㣩
	int LoadTranMat(std::vector<cv::Mat>& vGantryTransRobotMat, std::vector<cv::Mat>& vRobotTransGantryMat, CString IniFileName = MAT_GANTRY_ROBOT);	//����ת������
	int LoadOriginDis(std::vector<double>& vGantryRobotOriginDis, CString IniFileName = MAT_GANTRY_ROBOT);
	// ����ת����ϵ
	bool CalculateTransRelation(cv::Mat &GantryTransRobotMat, cv::Mat &RobotTransGantryMat, double& dBaseDis,double& OriginDis);
	//std::vector<cv::Mat> m_vGantryTransRobotMat;	//��������ϵת����������ϵת������
	//std::vector<cv::Mat> m_vRobotTransGantryMat;	//����������ϵת��������ϵת������
	//std::vector<double> m_vGantryRobotOriginDis;	//����������ԭ�����������ԭ��֮��ľ���
	//����������ϵ�µĺ�����Ϣת��������������ϵ��
	CvPoint3D64f TransCoor_Gantry2RobotNew(CvPoint3D64f& BasePoint);
	CvPoint3D64f TransCoor_Gantry2Robot(CvPoint3D64f& BasePoint);
	CvPoint3D64f TransCoor_Robot2Gantry(CvPoint3D64f& BasePoint);
	CvPoint3D64f TransNorVector_Gantry2Robot(CvPoint3D64f& BasePoint);
	CvPoint3D64f TransNorVector_Robot2Gantry(CvPoint3D64f& BasePoint);
	std::vector<CvPoint3D64f> TransCoor_Gantry2RobotNew(std::vector<CvPoint3D64f>& BasePoints);	//��������ת����������
	std::vector<CvPoint3D64f> TransCoor_Gantry2Robot(std::vector<CvPoint3D64f>& BasePoints);	//��������ת����������
	std::vector<CvPoint3D64f> TransCoor_Robot2Gantry(std::vector<CvPoint3D64f>& BasePoints);	//����������ת��������		
	std::vector<CvPoint3D64f> TransNorVector_Gantry2Robot(std::vector<CvPoint3D64f>& BasePoints);	//��������������ϵת����������ϵ����λ��������
	std::vector<CvPoint3D64f> TransNorVector_Robot2Gantry(std::vector<CvPoint3D64f>& BasePoints);	//����������������ϵת��������ϵ����λ��������

	double TransNorAngle_Gantry2Robot(double dNorAngle);

	// ���������˲�����
	void CreatTrackFilteObject();
	int m_nRobotSmooth;

	//  ��ȡ�ⲿ������
	void LoadExAixsFun();
	int m_nLinedScanAxisNo;
	int m_nMeasureAxisNo;
	int m_nMeasureAxisNo_up;
	int m_nTrackAxisNo;
	// �Ȳ���Ƿ���Ҫ�ƶ��ⲿ��
	bool m_bWeldAfterMeasureMoveExAxis = false;

	// ��Ԫȫ�ֱ���
	double m_dExAxisYPos;// ����ʱ����ͣ������
	double m_dExAxisXPos;
	double m_dExAxisZPos;
	double m_dRobotBaseToGantryCtrDis;//�����˻������ĵ��������ĵľ���,
	double m_dGantryRobotOriginDis;//�����˻������ĵ������������ĵľ���
	bool   m_bSingleRobotWork;// ��˫�����ӱ�־
	bool m_ScanTrackingWeldEnable;		// �Ƿ������Ȳ�󺸸���

	cv::Mat m_GantryTransRobotMat;	//��������ϵת����������ϵת������
	cv::Mat m_RobotTransGantryMat;	//����������ϵת��������ϵת������

	// �ϵ�����
	bool m_bBreakPointContinue = false;							// ȫ�ֶϵ�����
	// ��ʾ��Ϣ
	CString m_sHintInfo = "null";



	/**************** ������� End ******************/

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
		//���ذڶ��ļ�
		void LoadWeldParam(int SwingNumber, int nLayerNo);

		//������Ϣ����
		SwingTrackWeldParam m_swingTrackWeldParam;
		std::vector<SwingTrackWeldParam> m_vtSwingTrackWeldParam;
		//�ڶ�����״̬λ(Ϊ�˷�ֹ�ظ����ø�������) true��û���� false�����ڵ���
		bool m_bSwingState;
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

		/**************** �¸��� jwq��� ******************/
// ���ͺ��ӹ켣����(����40����ѭ������)
		bool SendWeldDataNew(FILE* file, const T_WELD_PARA& tWeldParam, E_WELD_SEAM_TYPE eWeldSeamType, bool& bIsCallJob, int ToolNum = 1U);
		// �¸�������������Ҫ���������ۺ��ӹ켣
		bool LockForNewRTTA(IplImage* pShowImage);

		RTT::RealTimeTrack m_cRealTimeTrack;//��ʵʱ����
		PartData::WeldTrack m_cWeldTrack;//�º��ӹ켣

};

