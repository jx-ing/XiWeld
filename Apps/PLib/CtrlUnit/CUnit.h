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



	//设置运动轨迹
	bool SetMoveValue(std::vector<T_ROBOT_MOVE_INFO> vtMoveInfo);
	bool SetMoveValue(CRobotMove cMoveInfo);
	int GetMoveStep();
	int CleanMoveStep();

	void CallJob(CString sJobName, int nExternalType);

	//坐标比较
	bool ComparePulse(T_ANGLE_PULSE tPulse1, T_ANGLE_PULSE tPulse2, long lLimit = 30);
	bool ComparePulse_Base(T_ANGLE_PULSE tPulse1, double dLimit = 1.0);

	//单元状态相关

	//急停
	void UnitEmgStop();
	bool RobotEmg(bool bPopup = FALSE);
	bool RobotBackHome(T_ROBOT_MOVE_SPEED tSpeed = T_ROBOT_MOVE_SPEED(2000, 70, 70));
	bool MoveExAxleToSafe(T_ANGLE_PULSE* ptDstPulse = NULL);

	//设置绝对坐标为当前坐标
	T_RUNNING_RTN_RESULT SetAbsEncoderCoor();


	/**************** 焊接添加 Start ******************/
	// 获取大恒相机跟踪视觉配置文件名
public:
	// 获取外部轴脉冲当量
	double GetExPulseEquivalent(int nMoveXYZAxisNo);
	// 机器人：阻塞等待运动停止	
	bool CheckRobotDone(T_ROBOT_COORS tAimCoor, CString sInfo = "");
	bool CheckRobotDone(T_ANGLE_PULSE tAimPulse, CString sInfo = "");
	CString GetTrackTipDataFileName(int nCameraNo);

	// 世界坐标：获取单外部轴位置
	double GetPositionDis(); //查询外部轴坐标 (卡诺普或安川松下外部轴)
	//// 世界坐标：获取直角坐标
	//T_ROBOT_COORS GetCurrentPosWorld();
	//// 世界坐标：获取关节坐标
	//T_ANGLE_PULSE GetCurrentPulseWorld();
	// 世界坐标：运动 安川 松下外部轴通用
	int WorldMoveByJob(T_ANGLE_PULSE tPulse, T_ROBOT_MOVE_SPEED tPulseMove, CString JobName);
	// 世界坐标：是否增在运动
	bool WorldIsRunning(); 
	// 世界坐标：等待运动停止
	void WorldCheckRobotDone(int nDelayTime = 1200);
	// 世界坐标：外部轴运动函数,临时添加移动安川外部轴
	int MoveExAxisForLineScan(double dDist, int nSpeed,bool bIsMoveYasExAxis = false);
	// 世界坐标：外部轴运动函数,增加轴号入参判定移动轴
	int MoveExAxisFun(double dDist, int nSpeed, int nMoveXYZAxisNo, double dMaxExSpeed = -1); // 最大外部周速度非扩展外部轴速度mm/min 小于0使用nSpeed
	int XYZAxisNoToSoftAxisNo(int nMoveXYZAxisNo);
	// 获取外部轴坐标，单轴/安川或松下
	double GetExPositionDis(int nMoveXYZAxisNo);
	// 获取轴类型,true:安川外部轴，false;非安川外部轴
	bool  GetExAxisType(int nMoveXYZAxisNo);
	// 机器人：返回上电状态 未上电时先执行上电
	bool CheckIsReadyRun();	
	// 机器人：移动到一定高度(与安全位置坐标和整座倒挂有关)
	//bool MoveToSafeHeight();
	// 硬触发示教运动 运动外部轴 和 机器人 (第一个点和外部轴同时运动 或 先动外部轴再动机器人)
	bool TeachMove(const std::vector<T_ANGLE_PULSE>& vtMeasurePulse, double dExAxlePos, const  std::vector<int>& vnMeasureType, int nTrigSigTime);
	// 焊接运动 ()
	int WeldMove(const std::vector<T_ROBOT_COORS>& vtWeldPathPoints, const  std::vector<int>& vnPtnType, const T_WELD_PARA& tWeldPara, double dExAxlePos, E_WELD_SEAM_TYPE eWeldSeamType, bool bIsArcOn, int ToolNum = 1U);
	int WeldMove_BP(const std::vector<T_ROBOT_COORS>& vtWeldPathPoints, const  std::vector<int>& vnPtnType, const T_WELD_PARA& tWeldPara, E_WELD_SEAM_TYPE eWeldSeamType, bool bIsArcOn);
	int WeldMove_Scan(const std::vector<T_ROBOT_COORS>& vtWeldPathPoints, const std::vector<int>& vnPtnType, const T_WELD_PARA& tWeldPara, double dExAxlePos, E_WELD_SEAM_TYPE eWeldSeamType, bool bIsArcOn, int ToolNum);
	// 安川:发送设置工艺参数 是否起弧：I058 起弧包角焊接停弧电流电压：I080-I087 停弧停留时间:I70
	bool SendWeldParam(BOOL bIsArcOn, const T_WELD_PARA& tWeldParam);

	//坡口焊接
	bool SendGrooveParam(BOOL bIsArcOn, const T_WELD_PARA& tWeldParam);
	int GrooveWeldMove(const std::vector<T_ROBOT_COORS>& vtWeldPathPoints, const T_WELD_PARA& tWeldPara, double dExAxlePos, bool bIsArcOn, T_WAVE_PARA tWavePara, vector<double> weldSpeedRate, vector<double> vdWaveWaitTime = vector<double>(0));
	bool SendGrooveWeldData(BOOL bIsArcOn, const std::vector<T_ROBOT_COORS>& vtWeldCoord, const T_WELD_PARA& tWeldParam, bool& bIsCallJob, T_WAVE_PARA tWavePara, vector<double> weldSpeedRate, vector<double> vdWaveWaitTime = vector<double>(0));
private:
	// 发送硬触发示教运动数据 nDownSpeed关节插补速度 nTeachSpeed 和 nUpSpeed 直线插补速度
	bool SendTeachMoveData(const std::vector<T_ANGLE_PULSE>& vtMeasurePulse, const  std::vector<int>& vnMeasureType, int nDownSpeed, int nTeachSpeed, int nUpSpeed, int nTrigSigTime);
	// 获取下一段焊接或包角轨迹
	bool GetNextSectionTrack(const std::vector<T_ROBOT_COORS>& vtSrcCoord, const  std::vector<int>& vnPtnType, int& nCursor, std::vector<T_ROBOT_COORS>& vtWeldCoord, int& nPtnType);
	// 发送起弧轨迹数据
	bool SendArcOnData(const  std::vector<T_ROBOT_COORS>& vtWeldCoord, const T_WELD_PARA& tWeldParam, int ToolNum = 1U);
	// 发送包角轨迹数据
	bool SendWrapData(const  std::vector<T_ROBOT_COORS>& vtWeldCoord, const T_WELD_PARA& tWeldParam/*, const WELD_WRAP_ANGLE& tWrapParam*/);
	// 发送焊接轨迹数据(超过40个点循环发送)
	bool SendWeldData(std::vector<T_ROBOT_COORS>& vtWeldCoord, const T_WELD_PARA& tWeldParam, E_WELD_SEAM_TYPE eWeldSeamType, bool& bIsCallJob, int ToolNum = 1U); // 循环发送																																		 // 计算摆动参考点（2023/12/21安川独占）
	bool SendWeldData_BP(std::vector<T_ROBOT_COORS>& vtWeldCoord, const T_WELD_PARA& tWeldParam, E_WELD_SEAM_TYPE eWeldSeamType, bool& bIsCallJob); // 循环发送																																		 // 计算摆动参考点（2023/12/21安川独占）
	void CalcSwayRefpCoord(T_ROBOT_COORS tStartCoord, E_WELD_SEAM_TYPE eWeldSeamType, T_ROBOT_COORS& tRefp1, T_ROBOT_COORS& tRefp2);

public:
	//****************龙门小部件坐标转换 ****************//
	int CaliTranMat(std::vector<cv::Mat>& vGantryTransRobotMat, std::vector<cv::Mat>& vRobotTransGantryMat, bool save = false);	//计算转换矩阵（采集四个不共面的点填入配置文件中计算）
	int LoadTranMat(std::vector<cv::Mat>& vGantryTransRobotMat, std::vector<cv::Mat>& vRobotTransGantryMat, CString IniFileName = MAT_GANTRY_ROBOT);	//加载转换矩阵
	int LoadOriginDis(std::vector<double>& vGantryRobotOriginDis, CString IniFileName = MAT_GANTRY_ROBOT);
	// 计算转换关系
	bool CalculateTransRelation(cv::Mat &GantryTransRobotMat, cv::Mat &RobotTransGantryMat, double& dBaseDis,double& OriginDis);
	//std::vector<cv::Mat> m_vGantryTransRobotMat;	//龙门坐标系转机器人坐标系转换矩阵；
	//std::vector<cv::Mat> m_vRobotTransGantryMat;	//机器人坐标系转龙门坐标系转换矩阵
	//std::vector<double> m_vGantryRobotOriginDis;	//机器人坐标原点和龙门坐标原点之间的距离
	//将龙门坐标系下的焊缝信息转换到机器人坐标系下
	CvPoint3D64f TransCoor_Gantry2RobotNew(CvPoint3D64f& BasePoint);
	CvPoint3D64f TransCoor_Gantry2Robot(CvPoint3D64f& BasePoint);
	CvPoint3D64f TransCoor_Robot2Gantry(CvPoint3D64f& BasePoint);
	CvPoint3D64f TransNorVector_Gantry2Robot(CvPoint3D64f& BasePoint);
	CvPoint3D64f TransNorVector_Robot2Gantry(CvPoint3D64f& BasePoint);
	std::vector<CvPoint3D64f> TransCoor_Gantry2RobotNew(std::vector<CvPoint3D64f>& BasePoints);	//龙门坐标转机器人坐标
	std::vector<CvPoint3D64f> TransCoor_Gantry2Robot(std::vector<CvPoint3D64f>& BasePoints);	//龙门坐标转机器人坐标
	std::vector<CvPoint3D64f> TransCoor_Robot2Gantry(std::vector<CvPoint3D64f>& BasePoints);	//机器人坐标转龙门坐标		
	std::vector<CvPoint3D64f> TransNorVector_Gantry2Robot(std::vector<CvPoint3D64f>& BasePoints);	//法向量龙门坐标系转机器人坐标系（单位法向量）
	std::vector<CvPoint3D64f> TransNorVector_Robot2Gantry(std::vector<CvPoint3D64f>& BasePoints);	//法向量机器人坐标系转龙门坐标系（单位法向量）

	double TransNorAngle_Gantry2Robot(double dNorAngle);

	// 创建跟踪滤波对向
	void CreatTrackFilteObject();
	int m_nRobotSmooth;

	//  获取外部轴作用
	void LoadExAixsFun();
	int m_nLinedScanAxisNo;
	int m_nMeasureAxisNo;
	int m_nMeasureAxisNo_up;
	int m_nTrackAxisNo;
	// 先测后焊是否需要移动外部轴
	bool m_bWeldAfterMeasureMoveExAxis = false;

	// 单元全局变量
	double m_dExAxisYPos;// 焊接时龙门停靠坐标
	double m_dExAxisXPos;
	double m_dExAxisZPos;
	double m_dRobotBaseToGantryCtrDis;//机器人基座中心到龙门中心的距离,
	double m_dGantryRobotOriginDis;//机器人基座中心到世界坐标中心的距离
	bool   m_bSingleRobotWork;// 单双机焊接标志
	bool m_ScanTrackingWeldEnable;		// 是否启用先测后焊跟踪

	cv::Mat m_GantryTransRobotMat;	//龙门坐标系转机器人坐标系转换矩阵；
	cv::Mat m_RobotTransGantryMat;	//机器人坐标系转龙门坐标系转换矩阵

	// 断点续焊
	bool m_bBreakPointContinue = false;							// 全局断点续焊
	// 提示信息
	CString m_sHintInfo = "null";



	/**************** 焊接添加 End ******************/

	public:
		// 跟踪摆动
		bool SwingTracking(CRobotDriverAdaptor* pRobotDriver, std::vector<T_ROBOT_COORS> vtRobotCoors, double speed);
		//分离和保存焊道(一部分轨迹)
		void SeparateAndSaveWeldingPath(std::vector<T_ROBOT_COORS> vtWorldCoors);
		//分离和保存焊道(单点)
		void SeparateAndSaveWeldingPath(T_ROBOT_COORS tWorldCoors);
		//输入起点终点信息(记录起始点和终点)
		void SaveSEPtn(T_ROBOT_COORS tStartPtn, T_ROBOT_COORS tEndPtn);
		//发送焊接所需要的信息
		bool SendWeldData();
		//发送焊接坐标
		void SendCoors();
		//写入摆动 (参数1 机器人实际运动坐标)(仅限直线焊接)
		void WriteSwing(T_ROBOT_COORS tRobotCoors);
		//分解速度 ( 输入参数：参数1 轨迹起点 参数2 轨迹终点 参数3焊接速度 输出参数 参数4外部轴速度 参数5机器人速度)
		void DecomposeSpeed(T_ROBOT_COORS tStartPytn, T_ROBOT_COORS tEnd, double nSpeed, double& dMainSpeed, double& dSecSpeed);
		//检查外部轴是否到达终点(判断焊道是否停止焊接 返回true是已经到达终点)
		bool isAxisToEnd();
		//合并焊道
		void UpdateRealRobotCoors();
		//实时检查摆动结构体中的焊道信息与跟踪流程中的焊道信息一致(线程)
		static UINT ThreadCheckSwingConsistency(void* pParam);
		//写入摆动参数（参数1频率 参数2振幅）
		void WriteSwingParam(double dSwingFrequency, double dSwingLeftAmplitude, double dSwingRightAmplitude);
		//外部轴运动信息线程
		//static UINT ThreadAxisData(void* pParam);
		////机器人运动信息线程
		//static UINT ThreadRobotData(void* pParam);
		//主控线程(需要修改)
		static UINT ThreadMainContData(void* pParam);
		//加载摆动文件
		void LoadWeldParam(int SwingNumber, int nLayerNo);

		//焊接信息参数
		SwingTrackWeldParam m_swingTrackWeldParam;
		std::vector<SwingTrackWeldParam> m_vtSwingTrackWeldParam;
		//摆动流程状态位(为了防止重复调用跟踪流程) true是没调用 false是正在调用
		bool m_bSwingState;
		CUnitDriver* pUnitDriver;
		//线程使用
		CEvent g_RobotState;		 //控制机器人运动信息线程挂起和恢复
		CEvent g_AxisState;			 //控制外部轴运动信息线程挂起和恢复
		CEvent g_RobotShutdownEvent;	//控制机器人运动信息线程结束
		CEvent g_AxisShutdownEvent;		//控制外部轴运动信息线程结束
		clock_t testTime; //测试使用
		CWinThread* pMainThread;	//主控线程控制指针
		CWinThread* pRobotThread;	//机器人线程控制指针
		CWinThread* pAxisThread;	//外部轴线程控制指针
		CWinThread* pUpdataThread;	//实时更新线程控制指针

		/**************** 新跟踪 jwq添加 ******************/
// 发送焊接轨迹数据(超过40个点循环发送)
		bool SendWeldDataNew(FILE* file, const T_WELD_PARA& tWeldParam, E_WELD_SEAM_TYPE eWeldSeamType, bool& bIsCallJob, int ToolNum = 1U);
		// 新跟踪用锁定，需要先设置理论焊接轨迹
		bool LockForNewRTTA(IplImage* pShowImage);

		RTT::RealTimeTrack m_cRealTimeTrack;//新实时跟踪
		PartData::WeldTrack m_cWeldTrack;//新焊接轨迹

};

