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
	double dExAxisOffsetX; // 考虑多外部轴使用 目前只使用 Y轴外部轴
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

	/************************ 对外接口 *************************/
public:
	// 开始线扫 线扫过程中阻塞 返回值为线扫是否成功
	bool Start();
	// 获取点云数据 失败原因：1、未调用线扫 2、线扫失败 3、调用ResetStatus后获取
	CvPoint3D64f* GetPointCloudData(int &nPtnNum);
	// 释放点云数据
	void ReleasePointCloud(); 

	// 加载线扫参数
	bool LoadLineScanParam(int scanNum);
	T_ROBOT_LIMITATION m_tRobotLimitation;

public:
	//六个限制点云范围的参数
	double m_dRange_XMax;
	double m_dRange_YMax;
	double m_dRange_ZMax;
	double m_dRange_XMin;
	double m_dRange_YMin;
	double m_dRange_ZMin;
	
	T_POINT_3D m_tPositivePointCloudCompen; // 正方向点云精度补偿
	T_POINT_3D m_tNegativePointCloudCompen; // 负方向点云精度补偿

	/************************ 内部接口和成员 *************************/
private:
	// 线扫调度线程
	static UINT ThreadSchedulingFunction(void* pParam);
	// 多机线扫调度
	static UINT ThreadSchedulingFunctionNew(void* pParam);
	// 线扫处理线程
	static UINT ThreadExecutiveFunction(void* pParam);
	// 多机线扫处理线程
	static UINT ThreadExecutiveFunctionNew(void* pParam);
	// 线扫调度函数
	void SchedulingFunction();
	void SchedulingFunction(int nCamareNo, int scanNum);
	// 线扫处理函数
	void ExecutiveFunction(int nThreadNo);
	void ExecutiveFunction(int nThreadNo, int nCamareNo);
	//线扫流程
	BOOL LaserScanRuning();
	//获取图像帧数
	double GetLaserCamCurrentFrameRate();
	double GetLaserCamCurrentFrameRate(int nCamareNo);
	//开始连续回调采图
	BOOL StartCallBackImage();
	BOOL StartCallBackImage(int nCamareNo);
	//停止连续回调采图
	BOOL StopCallBackImage();
	BOOL StopCallBackImage(int nCamareNo);
	//清除回调采图数据
	void ClearCallBackImage();
	void ClearCallBackImage(int nCamareNo);
	// 释放回调存图缓冲区中的一个图像资源
	void ReleaseCallBackImage(int nImageID);
	void ReleaseCallBackImage(int nImageID, int nCamareNo);

	// 读取相机内部统计帧数 残帧数 丢帧数
	void ReadCapInfo(bool bIsShowInfo = false);
	void ReadCapInfo(bool bIsShowInfo, int nCamareNo);
	//得到回调存图总数
	int GetCallBackImageNum();
	int GetCallBackImageNum(int nCamareNo);
	//检查图像ID
	BOOL CheckCallBackImage(int nImageID);
	BOOL CheckCallBackImage(int nImageID, int nCamareNo);
	//得到回调存图
	IplImage *GetCallBackImage(int nImageID);
	IplImage* GetCallBackImage(int nImageID, int nCamareNo);
	// 获取当前空闲线程编号
	int GetFreeThreadNo(int nRobotNo);  
	int GetFreeThreadNo(int nRobotNo, int nCamareNo);
	// 检查线程是否全部空闲
	BOOL CheckThreadFree(int nRobotNo); 
	BOOL CheckThreadFree(int nRobotNo, int nCamareNo);

	// 检查线程是否全部退出
	BOOL CheckThreadQuit(int nRobotNo); 
	BOOL CheckThreadQuit(int nRobotNo, int nCamareNo);

	// 线扫图像处理
	BOOL ImageProcess(T_THREAD_PROCESS_DATA& tImgCollectData);
	// 保存线扫结果(点云数据)
	void SaveLineScanResult(FILE* fRecordTimeInfo); 
	void SaveLineScanResult(FILE* fRecordTimeInfo, int nCamareNo);

	BOOL ImageProcess(T_THREAD_PROCESS_DATA& tImgCollectData, int nCamareNo);


	// 判断点云所有坐标是否在设置范围内
	bool LimitPointCloudRange(T_ROBOT_COORS tDealPoint);
	

	// 获取机器人日志指针
	CLog* GetLog();

	// 线扫流程数据
	BOOL *m_pbNaturalPop;			// 提示性弹窗开关
	bool m_bComplete;			// 线扫运行状态
	BOOL m_bProcessStartMark;	//开启执行线程标志
	BOOL m_bStartHandleMark;	//开始处理标志
	BOOL m_bScanMark;			//采集标志
	int m_nThreadNo;			//开启执行线程编号(处理线程启动时使用)
	int m_nDrawingNum;			//采集图片数量
	int m_nProcessNum;			//处理图片数量
	BOOL m_bScanMoveOver;		//是否线扫运动结束
	HANDLE m_hScanMoveOver;		//信号量m_bScanMoveOver

	std::vector<int> m_vnThreadNo;		// 处理子线程编号表
	double m_dScanExAxisPos;			// 线扫前外部轴位置
	double m_dScanExAxisZPos;			// 线扫前外部轴Z位置
	T_ROBOT_COORS m_tRobotScanPos;		// 线扫前机器人直角坐标
	T_ANGLE_PULSE m_tRobotScanPulse;		// 线扫前机器人关节坐标
	HANDLE m_hParallelProcessingSem;	// 并行处理信号量句柄
	std::vector<CvPoint3D64f> m_vtPointCloud;	// 点云数据 根据扫描距离 速度 采图帧率 采样间隔 等参数提前预留足够空间
	CWinThread* m_cThreadSchedulingFunction;	// 调度线程指针
	T_THREAD_PROCESS_DATA m_vtImgCollectDataBuff[MAX_PROCESS_THREAD_NUM];	// 处理函数处理任务
	E_PROCESS_THREAD_STATE m_eThreadStateMark[MAX_PROCESS_THREAD_NUM];		// 处理线程状态

	// 多相机处理

	double m_vdScanExAxisPos[LINESCAN_CAMERA_NUM];
	vector<bool> m_vbComplete;			// 线扫运行状态
	vector<bool> m_vbProcessStartMark;	//开启执行线程标志
	vector<bool> m_vbStartHandleMark;	//开始处理标志
	vector<bool> m_vbScanMark;			//采集标志
	vector<int> m_nThreadNoNew;		//开启执行线程编号(处理线程启动时使用)
	vector<int> m_vnDrawingNum;			//采集图片数量
	vector<int> m_vnProcessNum;			//处理图片数量
	vector < std::vector<int>> m_vnThreadNoNew;		// 处理子线程编号表
	//vector<double> m_vdScanExAxisPos;			// 线扫前外部轴位置
	//T_ROBOT_COORS m_tRobotScanPos;		// 线扫前机器人直角坐标
	//T_ANGLE_PULSE m_tRobotScanPuls;		// 线扫前机器人关节坐标
	vector<HANDLE> m_vhParallelProcessingSem;	// 并行处理信号量句柄
	vector <std::vector<CvPoint3D64f>> m_vvtPointCloud;	// 点云数据 根据扫描距离 速度 采图帧率 采样间隔 等参数提前预留足够空间
	CWinThread* m_pcThreadSchedulingFunction[LINESCAN_CAMERA_NUM];	// 调度线程指针
	T_THREAD_PROCESS_DATA m_vvtImgCollectDataBuff[LINESCAN_CAMERA_NUM][MAX_PROCESS_THREAD_NUM];	// 处理函数处理任务
	E_PROCESS_THREAD_STATE m_veThreadStateMark[LINESCAN_CAMERA_NUM][MAX_PROCESS_THREAD_NUM];		// 处理线程状态


	// 硬件或计算对象
	CUnit* m_ptUnit;									//控制单元
	CDHGigeImageCapture *m_pLaserCamera;
	vector<CDHGigeImageCapture*> m_vtpLaserCamera;
	CRobotDriverAdaptor* m_pRobotDriver;

	T_ANGLE_PULSE m_tStartPlus;
	T_ANGLE_PULSE m_tEndPlus;
	T_ROBOT_COORS m_tStartWorldCoors;
	T_ROBOT_COORS m_tEndWorldCoors;//线扫起终点
	double m_dScanStartCarLoction;
	double m_dScanLength;
	int m_nScanDir; // 扫描方向 123表示xyz 正负表示方向 例如：2表示Y+方向   -1表示X-方向
	bool m_bExAxisEnable; // 外部轴使能 1运动外部轴 0运动机器人	
	typedef struct
	{
		int nCameraNo;
		int nScanNum;
		CLaserLineScreen* pcLineScan;
	}T_GET_LINESCAN_PHOTO;
};

