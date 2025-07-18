#pragma once

#include "cv.h"
#include ".\Apps\PLib\CtrlUnit\CUnit.h"
#include "RiserManagement.h"
#include "Get Laser Point.h"

typedef enum
{
	PROCESS_THREAD_FREE,
	PROCESS_THREAD_WORKING,
	PROCESS_THREAD_COMPLETE,
	PROCESS_THREAD_FAIL,
	PROCESS_THREAD_QUIT,
}E_PROCESS_THREAD_STATE;

typedef struct
{
	T_ROBOT_COORS tRobotCoord;
	T_ANGLE_PULSE tRobotPulse;
	double dExAxisOffsetX; // ���Ƕ��ⲿ��ʹ�� Ŀǰֻʹ�� Y���ⲿ��
	double dExAxisOffsetY;
	double dExAxisOffsetZ;
	IplImage* ptImg;
}T_IMG_CACHE;

typedef struct
{
	int nDataNo;
	HANDLE m_hWorkHandle;
	T_IMG_CACHE tImageData;
	long alUsedTimeData[6];
	CvPoint cpLeftEndPoint;
	CvPoint cpRightEndPoint;
	T_ROBOT_COORS tHandleResultLeft;
	T_ROBOT_COORS tHandleResultRight;
	std::vector<T_ROBOT_COORS> vtLaserPoint;
	std::vector<CvPoint> vcpLaserPoint;
}T_THREAD_PROCESS_DATA;

class CLaserLineScreen
{
public:
	//CLaserLineScreen(CUnit *pUnit,  vector<DHGIGE_IMAGECAPTURE_NS::CDHGigeImageCapture*>vpLaserCamera,BOOL *pbNaturalPop = false);
	CLaserLineScreen(CUnit *pUnit, BOOL *pbNaturalPop);
	~CLaserLineScreen();

	/************************ ����ӿ� *************************/
public:
	// ��ʼ��ɨ ��ɨ���������� ����ֵΪ��ɨ�Ƿ�ɹ�
	bool Start();
	// ��ȡ�������� ʧ��ԭ��1��δ������ɨ 2����ɨʧ�� 3������ResetStatus���ȡ
	CvPoint3D64f* GetPointCloudData(int &nPtnNum);
	// �ͷŵ�������
	void ReleasePointCloud(); 

	// ������ɨ����
	bool LoadLineScanParam(int scanNum);
	T_ROBOT_LIMITATION m_tRobotLimitation;

public:
	//�������Ƶ��Ʒ�Χ�Ĳ���
	double m_dRange_XMax;
	double m_dRange_YMax;
	double m_dRange_ZMax;
	double m_dRange_XMin;
	double m_dRange_YMin;
	double m_dRange_ZMin;
	
	T_POINT_3D m_tPositivePointCloudCompen; // ��������ƾ��Ȳ���
	T_POINT_3D m_tNegativePointCloudCompen; // ��������ƾ��Ȳ���

	/************************ �ڲ��ӿںͳ�Ա *************************/
private:
	// ��ɨ�����߳�
	static UINT ThreadSchedulingFunction(void* pParam);
	// �����ɨ����
	static UINT ThreadSchedulingFunctionNew(void* pParam);
	// ��ɨ�����߳�
	static UINT ThreadExecutiveFunction(void* pParam);
	// �����ɨ�����߳�
	static UINT ThreadExecutiveFunctionNew(void* pParam);
	// ��ɨ���Ⱥ���
	void SchedulingFunction();
	void SchedulingFunction(int nCamareNo, int scanNum);
	// ��ɨ������
	void ExecutiveFunction(int nThreadNo);
	void ExecutiveFunction(int nThreadNo, int nCamareNo);
	//��ɨ����
	BOOL LaserScanRuning();
	//��ȡͼ��֡��
	double GetLaserCamCurrentFrameRate();
	double GetLaserCamCurrentFrameRate(int nCamareNo);
	//��ʼ�����ص���ͼ
	BOOL StartCallBackImage();
	BOOL StartCallBackImage(int nCamareNo);
	//ֹͣ�����ص���ͼ
	BOOL StopCallBackImage();
	BOOL StopCallBackImage(int nCamareNo);
	//����ص���ͼ����
	void ClearCallBackImage();
	void ClearCallBackImage(int nCamareNo);
	// �ͷŻص���ͼ�������е�һ��ͼ����Դ
	void ReleaseCallBackImage(int nImageID);
	void ReleaseCallBackImage(int nImageID, int nCamareNo);

	// ��ȡ����ڲ�ͳ��֡�� ��֡�� ��֡��
	void ReadCapInfo(bool bIsShowInfo = false);
	void ReadCapInfo(bool bIsShowInfo, int nCamareNo);
	//�õ��ص���ͼ����
	int GetCallBackImageNum();
	int GetCallBackImageNum(int nCamareNo);
	//���ͼ��ID
	BOOL CheckCallBackImage(int nImageID);
	BOOL CheckCallBackImage(int nImageID, int nCamareNo);
	//�õ��ص���ͼ
	IplImage *GetCallBackImage(int nImageID);
	IplImage* GetCallBackImage(int nImageID, int nCamareNo);
	// ��ȡ��ǰ�����̱߳��
	int GetFreeThreadNo(int nRobotNo);  
	int GetFreeThreadNo(int nRobotNo, int nCamareNo);
	// ����߳��Ƿ�ȫ������
	BOOL CheckThreadFree(int nRobotNo); 
	BOOL CheckThreadFree(int nRobotNo, int nCamareNo);

	// ����߳��Ƿ�ȫ���˳�
	BOOL CheckThreadQuit(int nRobotNo); 
	BOOL CheckThreadQuit(int nRobotNo, int nCamareNo);

	// ��ɨͼ����
	BOOL ImageProcess(T_THREAD_PROCESS_DATA& tImgCollectData);
	// ������ɨ���(��������)
	void SaveLineScanResult(FILE* fRecordTimeInfo); 
	void SaveLineScanResult(FILE* fRecordTimeInfo, int nCamareNo);

	BOOL ImageProcess(T_THREAD_PROCESS_DATA& tImgCollectData, int nCamareNo);


	// �жϵ������������Ƿ������÷�Χ��
	bool LimitPointCloudRange(T_ROBOT_COORS tDealPoint);
	

	// ��ȡ��������־ָ��
	CLog* GetLog();

	// ��ɨ��������
	BOOL *m_pbNaturalPop;			// ��ʾ�Ե�������
	bool m_bComplete;			// ��ɨ����״̬
	BOOL m_bProcessStartMark;	//����ִ���̱߳�־
	BOOL m_bStartHandleMark;	//��ʼ�����־
	BOOL m_bScanMark;			//�ɼ���־
	int m_nThreadNo;			//����ִ���̱߳��(�����߳�����ʱʹ��)
	int m_nDrawingNum;			//�ɼ�ͼƬ����
	int m_nProcessNum;			//����ͼƬ����
	BOOL m_bScanMoveOver;		//�Ƿ���ɨ�˶�����
	HANDLE m_hScanMoveOver;		//�ź���m_bScanMoveOver

	std::vector<int> m_vnThreadNo;		// �������̱߳�ű�
	double m_dScanExAxisPos;			// ��ɨǰ�ⲿ��λ��
	double m_dScanExAxisZPos;			// ��ɨǰ�ⲿ��Zλ��
	T_ROBOT_COORS m_tRobotScanPos;		// ��ɨǰ������ֱ������
	T_ANGLE_PULSE m_tRobotScanPulse;		// ��ɨǰ�����˹ؽ�����
	HANDLE m_hParallelProcessingSem;	// ���д����ź������
	std::vector<CvPoint3D64f> m_vtPointCloud;	// �������� ����ɨ����� �ٶ� ��ͼ֡�� ������� �Ȳ�����ǰԤ���㹻�ռ�
	CWinThread* m_cThreadSchedulingFunction;	// �����߳�ָ��
	T_THREAD_PROCESS_DATA m_vtImgCollectDataBuff[MAX_PROCESS_THREAD_NUM];	// ��������������
	E_PROCESS_THREAD_STATE m_eThreadStateMark[MAX_PROCESS_THREAD_NUM];		// �����߳�״̬

	// ���������

	double m_vdScanExAxisPos[LINESCAN_CAMERA_NUM];
	vector<bool> m_vbComplete;			// ��ɨ����״̬
	vector<bool> m_vbProcessStartMark;	//����ִ���̱߳�־
	vector<bool> m_vbStartHandleMark;	//��ʼ�����־
	vector<bool> m_vbScanMark;			//�ɼ���־
	vector<int> m_nThreadNoNew;		//����ִ���̱߳��(�����߳�����ʱʹ��)
	vector<int> m_vnDrawingNum;			//�ɼ�ͼƬ����
	vector<int> m_vnProcessNum;			//����ͼƬ����
	vector < std::vector<int>> m_vnThreadNoNew;		// �������̱߳�ű�
	//vector<double> m_vdScanExAxisPos;			// ��ɨǰ�ⲿ��λ��
	//T_ROBOT_COORS m_tRobotScanPos;		// ��ɨǰ������ֱ������
	//T_ANGLE_PULSE m_tRobotScanPuls;		// ��ɨǰ�����˹ؽ�����
	vector<HANDLE> m_vhParallelProcessingSem;	// ���д����ź������
	vector <std::vector<CvPoint3D64f>> m_vvtPointCloud;	// �������� ����ɨ����� �ٶ� ��ͼ֡�� ������� �Ȳ�����ǰԤ���㹻�ռ�
	CWinThread* m_pcThreadSchedulingFunction[LINESCAN_CAMERA_NUM];	// �����߳�ָ��
	T_THREAD_PROCESS_DATA m_vvtImgCollectDataBuff[LINESCAN_CAMERA_NUM][MAX_PROCESS_THREAD_NUM];	// ��������������
	E_PROCESS_THREAD_STATE m_veThreadStateMark[LINESCAN_CAMERA_NUM][MAX_PROCESS_THREAD_NUM];		// �����߳�״̬


	// Ӳ����������
	CUnit* m_ptUnit;									//���Ƶ�Ԫ
	CDHGigeImageCapture *m_pLaserCamera;
	vector<CDHGigeImageCapture*> m_vtpLaserCamera;
	CRobotDriverAdaptor* m_pRobotDriver;

	T_ANGLE_PULSE m_tStartPlus;
	T_ANGLE_PULSE m_tEndPlus;
	T_ROBOT_COORS m_tStartWorldCoors;
	T_ROBOT_COORS m_tEndWorldCoors;//��ɨ���յ�
	double m_dScanStartCarLoction;
	double m_dScanLength;
	int m_nScanDir; // ɨ�跽�� 123��ʾxyz ������ʾ���� ���磺2��ʾY+����   -1��ʾX-����
	bool m_bExAxisEnable; // �ⲿ��ʹ�� 1�˶��ⲿ�� 0�˶�������	
	typedef struct
	{
		int nCameraNo;
		int nScanNum;
		CLaserLineScreen* pcLineScan;
	}T_GET_LINESCAN_PHOTO;
};

