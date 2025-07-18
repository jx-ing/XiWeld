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

#define INVERST_ROBOT TRUE //机器人正坐，倒挂

const double m_gExAxisSpeed = 8000.0;

using namespace std;
using namespace WAM;

typedef struct
{
	int nWorkpieceNo;//工件编号
	int nBoardNo;//所属板号
	int nEndPointNo;//端点编号
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
	int nWorkpieceNo;//工件编号									
	int nBoardNo;   //板子编号
	bool bBoardType; //直线、圆弧
	double dBoardLength;//板子长度
	double dBoardThick;//板子厚度
	double dBoardHight;//板子高度
	int nStartNo;//起点编号
	int nStartType;//起点类型
	int nStartShape;//起点形状
	int nStartIfWrap;//起点包角类型
	bool bStartReleVircital;//是否关联立焊
	bool bStartReleFlat;//是否关联平缝
	int bStartContiguousCor;//相邻角缝
	double dStartAngle;//起点法向角度
	int nEndNo;//终点编号
	int nEndType;//终点类型
	int nEndShape;//起点形状
	int nEndIfWrap;//终点包角类型
	bool bEndReleVircital;//是否关联立焊
	bool bEndReleFlat;//是否关联平缝
	int bEndContiguousCor;//终点相邻角缝
	double dEndAngle;//终起点法向角度
	int nProperty;//所属区域（普通角缝或加强板）
	int nWeldLineWeldType; //焊接类型(满焊断续焊)
	double dInterWeldLength;//断续焊接长度
	double dInterStopLength;//断续停弧长度
	int nInterWeldType;  //断续类型
	bool bIsDoubleWeld;//是否双面焊接
	bool bIfArc;//是否 起弧
	BOOL bIfWeldCom; //是否跳过焊接，调试时用
	int nWitchRobot;//所属机器人

}T_CORNER_LINE_INFO_NEW;

typedef vector <T_CORNER_LINE_INFO_NEW> VT_CORNER_LINE_INFO_NEW;
typedef vector <VT_CORNER_LINE_INFO_NEW> VVT_CORNER_LINE_INFO_NEW;

typedef struct
{
	int nWorkpieceNo;//工件编号，前板：0，后板 1
	int nBoardNo;   //板子编号
	int nCornerLineNo; //关联角缝序号
	double dContiguousFlat;//相连平缝
	double dVerHoleHeight;//过水孔高度
	double dVerWeldLength;//立焊长度
	double dMainAngle;//立焊夹角
	double dStartAngle;//起点法向角度
	int nStartNo;//起点编号
	int nStartIfWrap;//起点包角类型
	int nEndNo;//终点编号
	int nEndIfWrap;//终点包角类型
	int nWeldLineWeldType; //焊接类型(满焊断续焊)
	double dInterWeldLength;//断续焊接长度
	double dInterStopLength;//断续停弧长度
	int nInterWeldType;  //断续类型
	bool bIsDoubleWeld;//是否双面焊接
	bool bIfArc;//是否 起弧
	bool bIfWeldCom;//是否焊接完成
	int nWitchRobot;//所属机器人
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
	int nWorkpieceNo;//工件编号，前板：0，后板 1
	int nBoardNo;   //板子编号
	int nMeasurePointNum;//测量点数
	vector <XI_POINT> VMeasurePOINT;

}T_MEASURE_INFO_NEW;
typedef vector <T_MEASURE_INFO_NEW> VT_MEASURE_INFO_NEW;
typedef vector <VT_MEASURE_INFO_NEW> VVT_MEASURE_INFO_NEW;

typedef struct
{
	int nWorkpieceNo;//工件编号
	int nBoardNo;//所属板号
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
	CRobotDriverAdaptor* tdRobotDriver; //线程参数
	vector<T_ROBOT_COORS>* tdWeldTrace; //完整的焊接轨迹
	int tdWaveType; //摆动方式(0:不摆动 1:前后摆动)
};

typedef enum
{
	E_SEARCH_ENDPOINT = 0,//搜索端点
	E_MEASURE_ENDPOINT,//测量端点,平测
	E_MEASURE_ENDPOINT_VTCL,//测量端点,立测
	E_CORRECT_ENDPOINT,//较短焊缝修正端点
	E_UPDATED_ENDPOINT,//被更新端点
	E_MEASURE_PROJPOINT, //用作投影线的测量点
	E_NO_OPERATION//不操作
}E_GET_ENDPOINT_METHOD;//获取端点模式

//立缝关联信息表
typedef struct
{
	CString strFile;
	bool bIsReleveVertical;//是否关联立缝	 
	VVT_ENDPOINT_INFO_NEW vvtEndpoint_Info;//端点信息
	VT_VERTICAL_INFO_NEW vtVertical_Info;//立缝信息
	VVT_CORNER_LINE_INFO_NEW vvtCorner_Info;//关联角缝信息
	vector<vector<T_ALGORITHM_POINT>> vvtVerticalRealTrackCoors;
}T_VERTICAL_RELEVE_INFO;

//typedef struct
//{
//    string strfileName;//数据存储路径
//    bool bTracking;//是否需要跟踪
//    E_GET_ENDPOINT_METHOD eGetStartMethod;//起点获取方式
//	bool bFixedPointScanStart;//起点定长搜
//    E_GET_ENDPOINT_METHOD eGetEndMethod;//终点获取方式
//	bool bFixedPointScanEnd;//终点定长搜
//    bool bIsTrackingScanEnd;//是否焊接过程中搜索结尾位置
//	bool bIsMiddleMeasure;//中间位置是否需要测量
//	bool bIsClosedRing;//是否是闭合加强圈
//	int  nOpenTrackingNo;//开启跟踪点位
//	int  nCloseTrackNo;//关闭跟踪点位
//    XI_POINT tStartPoint;//起点
//    XI_POINT tEndPoint;//终点
//	VT_CORNER_LINE_INFO_NEW vtCornerRelsve_Info;//0：自身，1，起点关联，2，终点关联.加强圈时及顺序添加
//	VT_CIRCLE_CENTER_INFO_NEW vtCenter_Info;//圆心
//	VVT_ENDPOINT_INFO_NEW vtEndpoint_info;//端点信息
//	vector<vector<T_ROBOT_COORS>> vvtMeasure_Info;
//	vector<vector<T_ANGLE_PULSE>> vvtMsePulse_Info;
//	vector<vector<T_ALGORITHM_POINT>> vvtMeasureResult_Info;
//	vector<vector<XI_POINT>> vvtheoryTrack;
//	vector<vector<T_ROBOT_COORS>> vvtheoryTrackRobotCoors;
//	vector<T_ROBOT_COORS > vtMeasure_Data;//测量数据调试用
//	vector<T_ROBOT_COORS > vtMiddleMeasure_Data;//测量数据调试用
//}T_BOARD_INFO_NEW;

typedef struct
{
	vector<T_ROBOT_COORS> vtMeasurePointStart; // 起点测量数据
	vector<T_ANGLE_PULSE> vtMeasurePulseStart;
	vector<int> vnMeasureTypeStart; // 测量类型
	double dExAxisPosStart;

	vector<T_ROBOT_COORS> vtMeasurePoint; // 中间测量数据
	vector<T_ANGLE_PULSE> vtMeasurePulse;
	vector<int> vnMeasureType; // 测量类型
	double dExAxisPos;

	vector<T_ROBOT_COORS> vtMeasurePointEnd; // 终点测量数据
	vector<T_ANGLE_PULSE> vtMeasurePulseEnd;
	vector<int> vnMeasureTypeEnd; // 测量类型
	double dExAxisPosEnd;

	vector<T_ROBOT_COORS> vtMeasureWarp; // 包角测量数据
	vector<T_ANGLE_PULSE> vtMeasurePulseWarp;
	vector<int> vnMeasureTypeWarp; // 测量类型
	double dExAxisPosWarp;

	vector<T_ROBOT_COORS> vtMeasureProStart; // 投影点起点测量数据
	vector<T_ANGLE_PULSE> vtMeasurePulseProStart;
	vector<int> vnMeasureProStart; // 测量类型
	double dExAxisPosProStart;

	vector<T_ROBOT_COORS> vtMeasureProEnd;	// 投影点终点测量数据
	vector<T_ANGLE_PULSE> vtMeasurePulseProEnd;
	vector<int> vnMeasureProEnd; // 测量类型
	double dExAxisPosProEnd;
}E_MEASURE_TRACK;

typedef enum
{
	E_CONTINUOUS = 0,
	E_NO_CONTINUOUS,
	E_FRONT_CONTINUOUS,//前面连续
	E_CONTINUOUS_AFTER,//后面连续
	E_CONTINUOUS_ALL//全部连续
}E_COUNTINUOUS_WELDING_TYPE;

typedef struct
{
	string strfileName;//数据存储路径
	bool bTracking;//是否需要跟踪
	E_GET_ENDPOINT_METHOD eGetStartMethod;//起点获取方式
	bool bFixedPointScanStart;//起点定长搜
	E_GET_ENDPOINT_METHOD eGetEndMethod;//终点获取方式
	bool bFixedPointScanEnd;//终点定长搜
	bool bIsTrackingScanEnd;//是否焊接过程中搜索结尾位置
	bool bIsMiddleMeasure;//中间位置是否需要测量
	bool bIsClosedRing;//是否是闭合加强圈
	int  nOpenTrackingNo;//开启跟踪点位
	int  nCloseTrackNo;//关闭跟踪点位
	XI_POINT tStartPoint;//起点
	XI_POINT tEndPoint;//终点
	E_WRAPANGLE_TYPE eWarpType;// 包角类型
	vector<T_ROBOT_COORS> vRealTrackCoors;//实际焊接轨迹 单条
	vector<int> vPointType;//点类型
}T_BOARD_PART_INFO;// 工件部分信息

typedef struct
{
	string strfileName;//数据存储路径
	bool bTracking;//是否需要跟踪
	E_GET_ENDPOINT_METHOD eGetStartMethod;//起点获取方式
	bool bFixedPointScanStart;//起点定长搜
	E_GET_ENDPOINT_METHOD eGetEndMethod;//终点获取方式
	bool bFixedPointScanEnd;//终点定长搜
	bool bIsTrackingScanEnd;//是否焊接过程中搜索结尾位置
	bool bIsMiddleMeasure;//中间位置是否需要测量
	bool bIsClosedRing;//是否是闭合加强圈
	int  nOpenTrackingNo;//开启跟踪点位
	int  nCloseTrackNo;//关闭跟踪点位
	XI_POINT tStartPoint;//起点
	XI_POINT tEndPoint;//终点
	int nContinueStartNo; // 连续焊接起始焊道
	int nContinueNum; // 连续焊接个数
	E_COUNTINUOUS_WELDING_TYPE eContinueType;//连续焊接类型
	vector<bool> vbBoardDoesItExist;// 工件是否存在
	vector<T_BOARD_PART_INFO> vtBoardPartInfo;//工件关联信息，过程中如何确定起终点判断条件
	VT_CORNER_LINE_INFO_NEW vtCornerRelsve_Info;//0：自身，1，起点关联，2，终点关联.加强圈时及顺序添加
	VT_CIRCLE_CENTER_INFO_NEW vtCenter_Info;//圆心
	VVT_ENDPOINT_INFO_NEW vtEndpoint_info;//端点信息，初始值
	vector<E_MEASURE_TRACK> vtMeasureTrack; // 测量轨迹
	vector<vector<T_ROBOT_COORS>> vvtMeasure_Info;
	vector<vector<T_ANGLE_PULSE>> vvtMsePulse_Info;
	vector<vector<T_ALGORITHM_POINT>> vvtMeasureResult_Info;// 真实测量结果
	vector<vector<T_ALGORITHM_POINT>> vvtMeasureResult_Warp;// 一次包角测量结果
	vector<vector<XI_POINT>> vvtheoryTrack;
	vector<vector<T_ROBOT_COORS>> vvtheoryTrackRobotCoors;//理论轨迹
	vector<vector<T_ROBOT_COORS>> vvRealTrackRobotCoors;//实际焊接轨迹
	vector<vector<T_ROBOT_COORS >> vvtMeasure_Data_Start;//测量数据调试用 起点
	vector<vector<T_ROBOT_COORS >> vvtMeasure_Data_Mind;//测量数据调试用 中间
	vector<vector<T_ROBOT_COORS >> vvtMeasure_Data_End;//测量数据调试用 终点
	vector<vector<T_ROBOT_COORS >> vvtMeasure_Data_Warp;//测量数据调试用 包角
	vector<vector<T_ROBOT_COORS >> vvtMeasure_Data_ProStart;//测量数据调试用 投影扫描端
	vector<vector<T_ROBOT_COORS >> vvtMeasure_Data_ProEnd;//测量数据调试用 投影测量端
	vector<vector<T_ROBOT_COORS >> vvtMeasure_Data;
	vector<vector<T_ANGLE_PULSE>> vvtMeasure_Pulse_Start;	//测量姿态 起点
	vector<vector<T_ANGLE_PULSE>> vvtMeasure_Pulse_Mind;	//测量姿态 中间点
	vector<vector<T_ANGLE_PULSE>> vvtMeasure_Pulse_End;		//测量姿态 尾点
	vector<vector<T_ANGLE_PULSE>> vvtMeasure_Pulse_Warp;	//测量姿态 包角点
	vector<vector<T_ANGLE_PULSE>> vvtMeasure_Pulse_ProStart;//测量姿态 投影起点
	vector<vector<T_ANGLE_PULSE>> vvtMeasure_Pulse_ProEnd;	//测量姿态 投影终点
	vector<T_ROBOT_COORS > vtMiddleMeasure_Data;//测量数据调试用
	vector<T_ROBOT_COORS > vtRealWeldTrackCoors;//完整焊接轨迹，单独焊接或连续焊接，最终轨迹,包含安全下枪抬枪位置
	vector<T_ANGLE_PULSE > vtRealWeldTrackPulse;//完整焊接轨迹，单独焊接或连续焊接，最终轨迹
	vector<int> vnDataPointType;// 数据点类型
	vector<vector<T_ROBOT_COORS>> vvtTrackWarpCoors; //跟踪时存储包角坐标
	vector<vector<int>> vvnTrackWarpPtType;//跟踪时存储包角坐标类型

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
	//焊接信息参数
	SwingTrackWeldParam m_swingTrackWeldParam;
	//摆动流程状态位(为了防止重复调用跟踪流程) true是没调用 false是正在调用
	bool m_bSwingState;
	std::vector<CUnit*> m_vpUnit;							//控制单元对象
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
	ESTUN_TRACK_PARAM estunParam;//埃斯顿机器人实时跟踪的参数


public:
	//更新工件极限值
	void WriteLimitHight(double dMaxH, double dMinH);
	void LoadLimitHight();
	//查找关联信息
	T_CORNER_LINE_INFO_NEW  FindCornerLineInfoFun(int nBoardNo, VT_CORNER_LINE_INFO_NEW* pvtCornerLineInfo);
	T_ENDPOINT_INFO_NEW		FindEndPointInfoFun(int nEndPointNo, VT_ENDPOINT_INFO_NEW* pvtPointInfo);
	vector<XI_POINT>   GetMeasurePointFun(int nBoardNo, VT_MEASURE_INFO_NEW* pvMeasureInfo);
	T_VERTICAL_INFO_NEW		FindVercalInfoFun(int nBoardNo, VT_VERTICAL_INFO_NEW* ptVerticalInfo);
	T_CIRCLE_CENTER_INFO_NEW FindCenterPointInfoFun(int nEndPointNo, VT_CIRCLE_CENTER_INFO_NEW* pvtPointInfo);
	// 更新圆心
	bool CorrectCenterPointInfoFun(int nEndPointNo, XI_POINT tCenTerNew, VT_CIRCLE_CENTER_INFO_NEW* pvtPointInfo);

	bool FindRelevanceVerticalFun(T_VERTICAL_INFO_NEW tVerticalInfo, VT_VERTICAL_INFO_NEW* ptVerticalInfo, VT_ENDPOINT_INFO_NEW* pvtPointInfo, VT_CORNER_LINE_INFO_NEW* pvtCornerLineInfo, T_VERTICAL_INFO_NEW& tVertical);
	//调度函数
	//bool DiaphragmWeld();//外部调用
	//static UINT ThreadDiaphragmWeldProject(void* pProjec);
	//bool DiaphragmWeldProject(CRobotDriverAdaptor* pRobotDriver);

	//立缝焊接函数
	//bool VerticalWeldFunNew(CRobotDriverAdaptor* pRobotDriver, T_VERTICAL_INFO_NEW tVerticalInfo, VT_VERTICAL_INFO_NEW* pvtVerticalInfo,
	//	VT_CORNER_LINE_INFO_NEW* pvtCornerLineInfo, VT_ENDPOINT_INFO_NEW* pvtPointInfo, VT_MEASURE_INFO_NEW* pvMeasureInfo);
	//获取立缝关联信息
	bool GetVerticalReleveInfo(T_VERTICAL_INFO_NEW tVerticalInfo, VT_VERTICAL_INFO_NEW* pvtVerticalInfo,
		VT_CORNER_LINE_INFO_NEW* pvtCornerLineInfo, VT_ENDPOINT_INFO_NEW* pvtPointInfo, T_VERTICAL_RELEVE_INFO& tVerticalReleInfo);

	//bool ProduceVerticanWeld(CRobotDriverAdaptor* pRobotDriver, vector<T_ALGORITHM_POINT> vtLinePoints, double dTrackAngle, double dTrackAngleLeft);

	//角缝焊接函数
	//bool CornerLineWeldFunNew(CRobotDriverAdaptor* pRobotDriver, T_CORNER_LINE_INFO_NEW tCornerLineInfo, VT_CORNER_LINE_INFO_NEW* pvtCornerLineInfo,
	//	VT_ENDPOINT_INFO_NEW* pvtPointInfo, VT_MEASURE_INFO_NEW* pvMeasureInfo, VT_CIRCLE_CENTER_INFO_NEW* pvtCenterInfo);

	//单一加强圈隔板焊接
	bool StrengthRingWeldFunNew(CRobotDriverAdaptor* pRobotDriver, T_CORNER_LINE_INFO_NEW tCornerLineInfo,
		VT_CORNER_LINE_INFO_NEW* pvtCornerLineInfo, VT_ENDPOINT_INFO_NEW* pvtPointInfo);

	//选择线条或闭合圆弧
	bool SelectLinesAndCircle(CRobotDriverAdaptor* pRobotDriver, T_CORNER_LINE_INFO_NEW tCornerLineInfo, VT_ENDPOINT_INFO_NEW* pvtPointInfo, VT_CIRCLE_CENTER_INFO_NEW* pvtCenterInfo, double& dRadius);
	//测量端点或中间点数据
	bool CalcCoordUsePosDirAngle(XI_POINT tPtn, double dDirAngle, double dExAxlePos, T_ROBOT_COORS& tRobotCoord, double dRx = 0.0, double dRy = -45.0);
	bool GenerateTrack(vector<XI_POINT>& vtTrack, XI_POINT tStart, double dStartAngle, XI_POINT tEndPoint, double dEndAngle, XI_POINT tCenter, double dStepDis = 1.0, bool bIsArc = false);
	bool GenerateTrack(vector<XI_POINT>& vtTrack, XI_POINT tStart, XI_POINT tEndPoint, XI_POINT tCenter, double dStepDis = 1.0, bool bIsArc = false);

	bool GetTeachTrackNew(CRobotDriverAdaptor* pRobotDriver, T_BOARD_INFO_NEW& tBoardInfo, int nIndex, vector<T_ROBOT_COORS>& vtTeachTrack, vector<T_ANGLE_PULSE>& vtMeasurePulse, vector<int>& vnMeasureType, T_ANGLE_PULSE* ptPrePulse = nullptr);
	// 获取理论测量轨迹
	bool GetThoryMeasureTrack(CRobotDriverAdaptor* pRobotDriver, T_BOARD_INFO_NEW& tBoardInfo);
	// 获取实际的示教结果
	bool GetRealTeachData(CRobotDriverAdaptor* pRobotDriver, T_BOARD_INFO_NEW& tBoardInfo, VT_ENDPOINT_INFO_NEW* pvtPointInfo);
	// 计算实际的焊接轨迹
	bool CalcRealWeldTrack(CRobotDriverAdaptor* pRobotDriver, T_BOARD_INFO_NEW& tBoardInfo);
	// 开始焊接
	bool DoWeld(CRobotDriverAdaptor* pRobotDriver, T_BOARD_INFO_NEW& tBoardInfo);

	bool Weld(int nGroupNo);

	// 记录结尾数据：一次包角存在多次结束跟踪的结尾数据
	bool RecordEndPointCoors(CRobotDriverAdaptor* pRobotDriver, vector<T_ROBOT_COORS> vtWeldCoors, vector<int> vnWeldCoorsType, E_WRAPANGLE_TYPE eWarpType = E_WRAPANGLE_EMPTY_SINGLE);

	// 分解世界坐标
	bool DecomposeWorldCoordinates(CRobotDriverAdaptor* pRobotCtrl, vector<T_ROBOT_COORS>& vtCoutCoors, vector<T_ROBOT_COORS> vtCinCoors, bool bIsLockExAxis = false,double dExAixsPos = -99999.0);

	// 添加收下枪安全位置
	bool AddSafeDownGunPos(vector<T_ROBOT_COORS>& vCoors, vector<int>& vnCoorType, double dAdjustDis, double dPieceHight);

	// 计算一次包角数据
	bool CalcWarpTrack(vector<T_ROBOT_COORS> vtEndpoint, vector<T_ROBOT_COORS>& vtWarpStart, vector<T_ROBOT_COORS>& vtWarpEnd);
	// 判断测量点是否可用
	bool JudjeMeasureIsavailable(T_ALGORITHM_POINT tStartPoint, T_ALGORITHM_POINT tEndPoint, vector<T_ALGORITHM_POINT> vtMaesureCoors,
		vector<T_ALGORITHM_POINT>& vtCoutPoint, double ThreShold = 20., bool bIsReverse = false);
	// 点连线
	static bool PointConnection(vector<T_ALGORITHM_POINT> vtCinPoint, vector<XI_POINT>& vtCoutPoint, double dStepDis = 1.0);
	// 根据理论轨迹获取测量坐标
	bool GetMeasureCoors(CRobotDriverAdaptor* pRobotDriver, vector<T_ROBOT_COORS> vtWeldTrack, vector<T_ROBOT_COORS>& vtTeachTrack,
		vector<int>& vnMeasureType, bool bTracking = false, bool bIsArc = false, bool bStartType = false, bool bEndType = false, double dStepDis = 100.0, bool bExAxisMoving = true);
	// 转换相机工具，添加手下枪位置 并计算连续可运行脉冲
	bool TransCameraToolAndCalcContinuePulse(CRobotDriverAdaptor* pRobotDriver, vector<T_ROBOT_COORS>& vtTeachTrack, vector<T_ANGLE_PULSE>& vtMeasurePulse, vector<int>& vnMeasureType, T_ANGLE_PULSE* tRefPulse = nullptr);
	// 示教同时移动大车
	bool DoTeachWithMove(CRobotDriverAdaptor* pRobotDriver, vector<T_ROBOT_COORS> vtTeachCoors, vector<T_ROBOT_COORS> vtTeachCoorsTest, vector<T_ANGLE_PULSE> vtTeachPulse,
		vector<int> vtMeasureType, vector<T_ALGORITHM_POINT>& vtTeachResult, bool bIsReverse = false);
	bool CalcContinuePulse(CRobotDriverAdaptor* pRobotDriver, T_BOARD_INFO_NEW& tBoardInfo/*vector<T_ROBOT_COORS> &vtRobotCoors*/, T_ANGLE_PULSE tStandardRefPulse/*, vector<T_ANGLE_PULSE> &vtRobotPulse*/);
	//获取焊缝信息
	T_BOARD_INFO_NEW GetBoardInfo(CRobotDriverAdaptor* pRobotDriver, T_CORNER_LINE_INFO_NEW tCornerLineInfo,
		VT_CORNER_LINE_INFO_NEW* pvtCornerLineInfo, VT_ENDPOINT_INFO_NEW* pvtPointInfo, VT_CIRCLE_CENTER_INFO_NEW* pvtCenterInfo, CString strFile = "");
	//设置扫描数据
	//bool SetScanPos(CRobotDriverAdaptor* pRobotDriver, double dNormalAgl, XI_POINT tPoint, bool bIsStart, bool bIsFixedPointScan = false);
	bool SetScanPos(CRobotDriverAdaptor* pRobotDriver, vector<T_ROBOT_COORS> vtCooors, vector<T_ANGLE_PULSE> vtPulses, vector<int> vTeachType);
	// 计算测量数据 
	bool CalcScanPos(CRobotDriverAdaptor* pRobotDriver, double dNormalAgl, XI_POINT tPoint, vector<T_ROBOT_COORS>& vtScanCoors, vector<T_ANGLE_PULSE>& vtScanPulses,
		vector<int>& vtScanCoorsType, double& dExAxisPos, bool bIsStart, bool bIsFixedPointScan, T_ANGLE_PULSE* ptPrePulse = nullptr);
	//根据测量结果获取焊缝轨迹（单一的直线或弧线）
	bool CalcRealWeldTrackAccordingToMeasureData(CRobotDriverAdaptor* pRobotDriver, T_BOARD_INFO_NEW tBoardInfo, vector<T_ROBOT_COORS>& vtRealPoint);
	//得到测量结果
	bool GetMeasureDataWithMoveNew(CRobotDriverAdaptor* pRobotDriver, T_BOARD_INFO_NEW& tBoardInfo, int nIndex, vector<T_ALGORITHM_POINT>& vtTeachResult);
	//获取测量信息
	bool GetVerticalMeasureData(CRobotDriverAdaptor* pRobotDriver, T_VERTICAL_RELEVE_INFO tVerticalReleInfo,
		vector<T_ROBOT_COORS>& vtMeasurePoint, vector<T_ANGLE_PULSE>& vtMeasurePulse, vector<int>& vnMeasureType, double& dExAxis);

	bool GetVerticalMeasureDataNew(CRobotDriverAdaptor* pRobotDriver, T_BOARD_INFO_NEW& tBoardInfo, vector<T_ROBOT_COORS>& vtMeasurePoint
		, vector<T_ANGLE_PULSE>& vtMeasurePulse, vector<int>& vnMeasureType, double& dExAxisPos, int nIndex, bool bIsStart = true, T_ANGLE_PULSE* ptPrePulse = nullptr);
	// 计算立焊焊道轨迹
	bool CalcVerticalTrack(CRobotDriverAdaptor* pRobotDriver, T_VERTICAL_RELEVE_INFO& tVerticalReleInfo);
	// 根据测量数据计算交点 直线：直线 直线：圆弧 圆弧：圆弧
	bool CalcMeasureIntersection(CRobotDriverAdaptor* pRobotDriver, T_BOARD_INFO_NEW& tBoardInfo, vector<T_TEACH_RESULT> vtTeachResult,
		double dExAxisPos, int nIndex, XI_POINT& tIntersection, bool bIsStart);

	//获取理论参考坐标
	bool GetTheoryReferencePulse(CRobotDriverAdaptor* pRobotDriver, T_ROBOT_COORS tTeachCoord, T_ANGLE_PULSE& tTeachPulse);
	//初始化焊接状态
	void RefreshWeldReleveParam(CRobotDriverAdaptor* pRobotDriver, int nWeldSeamNum);
	//刷新关联端点
	void RefreshCorrectEndPointFun(CRobotDriverAdaptor* pRobotDriver, int EndPointNum);
	//记录修正端点
	void RecordCorrectEndPointFun(CRobotDriverAdaptor* pRobotDriver, int EndPointNo, XI_POINT tpEndPoint);
	//读取修正端点
	bool ReadCorrectEndPointFun(CRobotDriverAdaptor* pRobotDriver, int EndPointNo, XI_POINT& tpEndPoint);
	//更新端点
	bool CorrectEndPoint(CRobotDriverAdaptor* pRobotDriver, T_BOARD_INFO_NEW tBoardInfo,
		VT_ENDPOINT_INFO_NEW* pvtPointInfo, int nIndex, XI_POINT tStartPoint, XI_POINT tEndPoint);

	//记录焊接状态
	void RecordWeldStatus(int nEndPointNo, bool bStatus);
	bool LoadWeldStatus(int nEndPointNo);
	//拟合直线
	static T_LINE_PARA		 CalcLineParamRansac(vector<T_ALGORITHM_POINT> vtPoint, double dSelectedRatio);
	static T_LINE_PARA		 CalcLineParamRansac(vector<XI_POINT> vtPoint, double dSelectedRatio);
	static bool	 	 dCmpIncFunc(double dFirstValue, double dSecondValue);
	static	T_SPACE_LINE_DIR CalLineDir(XI_POINT tStartPoint, XI_POINT tEndPoint);
	static double			 CalDisPointToLine(XI_POINT tPoint, XI_POINT tSegmentStartPoint, XI_POINT tSegmentEndPoint);
	//拟合圆
	bool CalCircleThreePoint(T_SPACE_CIRCLE_PARAM& tCircle, vector<XI_POINT> vtFitOutputPoints, double dStepDis, vector<XI_POINT>& vtOutputPoints);
	bool CalThreeDotToCircle(const XI_POINT& tFirstPoint, const XI_POINT& tSecondPoint, const XI_POINT& tThirdPoint, T_SPACE_CIRCLE_PARAM& CircleParam);
	// 点到直线的距离
	double			 CalcDistancePointToLine(XI_POINT linePoint1, XI_POINT linePoint2, XI_POINT point);

	// 根据两直线上各一点和线的方向角求交点
	bool			 CalcLineLineIntersection(XI_POINT tStart, double dStartAngle, XI_POINT tEnd, double dEndAngle, XI_POINT& intersection);
	bool			 CalcLineLineIntersection(T_ROBOT_COORS tStart, double dStartAngle, T_ROBOT_COORS tEnd, double dEndAngle, XI_POINT& intersection);
	//拟合多段圆弧
	bool			CorrectCircled(double center[3], double radius, vector<XI_POINT> vtPoint);
	//根据测量点和理论起终点求直线
	bool			 FitCalLinePointsByMiddlePoints(XI_POINT tStartPoint, XI_POINT tEndPoint, vector<XI_POINT> vtScanPoints, vector<XI_POINT>& vtOutPoints, double dThresVal = 20., double dStepDis = 1.0);
	bool			 FitCalLinePointsByMiddlePoints(XI_POINT tStartPoint, XI_POINT tEndPoint, vector<T_ALGORITHM_POINT> vtScanPoints, vector<XI_POINT>& vtOutPoints, double dThresVal = 20., double dStepDis = 1.0);

	bool			 FitCalLinePointsByMiddlePoints(XI_POINT tStartPoint, XI_POINT tEndPoint, vector<XI_POINT> vtScanPoints, vector<XI_POINT>& vtOutPoints, vector<XI_POINT>& vtPosture, double& dNormalAngle, int& nStartChangePointNo, int& nEndChangePointNo,
		double dDisPoint, int nPointNum, bool bCheckDisORNum, int nThresVal, bool bStartChangePosture = false, bool bEndChangePosture = false);

	bool			 FitCalLinePointsByMiddlePoints(XI_POINT tStartPoint, XI_POINT tEndPoint, vector<T_ALGORITHM_POINT> vtScanPoints, vector<XI_POINT>& vtOutPoints, vector<XI_POINT>& vtPosture, double& dNormalAngle, int& nStartChangePointNo, int& nEndChangePointNo,
		double dDisPoint, int nPointNum, bool bCheckDisORNum, int nThresVal, bool bStartChangePosture = false, bool bEndChangePosture = false);
	// 根据理论轨迹计算起始终止段转姿态距离和需要转变角度
	/*
	* 参数说明：
	* 组成夹角左侧焊道
	*/
	void			 GetChangePosturePare(vector<T_ROBOT_COORS> tLeftTrackCoors, XI_POINT tLeftCenter, double dLeftNormal, bool bLeftIsArc, vector<T_ROBOT_COORS> tRightTrackCoors, XI_POINT tRightCenter, double dRightNormal, bool bRightIsArc, bool bStartOrEnd,
		int& nStartStepNo, double& dStChangeAngle, double dChangePostureThresVal = 100.0);

	//根据焊道数据计算机器人姿态

	bool TransRobotPostureCoors(vector<XI_POINT> vtOutPoints, vector<XI_POINT>& vtPosture, int& nStartChangePointNo,
		int& nEndChangePointNo, double dRX, double dRY, double dDisPoint, bool bStartChangePosture, bool bEndChangePosture);
	bool CalcRobotCoorsAccordingTrack(vector<XI_POINT> vtCinPoints, vector<T_ROBOT_COORS>& vtCoutCoors);
	//保存T_ALGORITHM_POINT类型数据到文件
	void			SaveDataAlgorithm(vector<T_ALGORITHM_POINT> vcPointOnTheSameLine, string str);
	//保存XI_POINT类型数据到文件
	void			SaveDataXiPoint(vector<XI_POINT> vcPointOnTheSameLine, string str);
	//保存XI_POINT类型数据到文件
	void			SaveData(vector<XI_POINT> vcPointOnTheSameLine, string str);
	void            SaveDataTrackFilt(vector<TrackFilter_XYZ> vtPoints, string str);
	void			SaveTrackFilterCoors(vector<TrackFilter_XYZ> vcPointOnTheSameLine, string str);
	void            SaveDataRobotCoors(vector<T_ROBOT_COORS> vcPointOnTheSameLine, string str);
	void            SaveDataRobotPulse(T_ANGLE_PULSE tPulse, string str);
	void			SaveDataInt(vector<int> vnCoors, string str);
	//加载数据
	void			LoadDataXiPoint(vector<XI_POINT>& vcPointOnTheSameLine, string str);
	void			LoadDataAlgorithm(vector<T_ALGORITHM_POINT>& vcPointOnTheSameLine, string str);
	void            LoadDataRobotCoors(vector<T_ROBOT_COORS>& vcPointOnTheSameLine, string str);
	void		    LoadDataRobotCoors(vector<T_ROBOT_COORS> &vcPointOnTheSameLine, vector<int> &vnCoorsType, string str);
	void            LoadDataRobotPulse(T_ANGLE_PULSE& tPulse, string str);
	void			LoadDataInt(vector<int>& vnCoors, string str);
	void			LoadTrackFilterCoors(vector<TrackFilter_XYZ>& vcPointOnTheSameLine, string str);
	//加载焊缝数据
	void LoadWeldLineData(E_WELD_LINE& vcPointOnTheSameLine, CString str);
	void GetCircleRealWeldPoint(CRobotDriverAdaptor* pRobotDriver, vector<T_ROBOT_COORS>& realWeldPoint, E_WELD_LINE vcPointOnTheSameLine, double dMachineCarMovePos);
	//转换数据类想
	void TransiformData(vector<XI_POINT> vcPointOnTheSameLine, vector<XI_POINT>& vtPoints);
	void TransiformMeasureData(vector<XI_POINT> vtPoints, vector<XI_POINT>& vcPointOnTheSameLine);
	void TransALGORITHMToXiPoint(vector<T_ALGORITHM_POINT> vtALGORITHMPoints, vector<XI_POINT>& vtPoints);
	void TransXiPointToALGORITHM(vector<XI_POINT> vtPoints, vector<T_ALGORITHM_POINT>& vtALGORITHMPoints);
	void TransMeasureToAlgorithm(vector<XI_POINT> vcPointOnTheSameLine, vector<T_ALGORITHM_POINT>& vtALGORITHMPoints);
	void TransAlgorithmToMeasure(vector<T_ALGORITHM_POINT> vtALGORITHMPoints, vector<XI_POINT>& vcPointOnTheSameLine);

	//计算偏移值 ,机器人坐标
	void   CalcMeasurePosInBase(vector<XI_POINT> tMeasurePoint, double dDis, double dZDis, double dAngleVer, vector<XI_POINT>& tMeasureTrans);
	void   CalcMeasurePosInBase(vector<XI_POINT> tMeasurePoint, double dDis, double dZDis, vector<XI_POINT> tPosture, vector<XI_POINT>& tMeasureTrans);
	void   CalcOffsetInBase(XI_POINT tMeasurePoint, double dDis, double dMoveZ, double dAngleVer, XI_POINT& tMeasureTrans, bool _isDown = false);
	void   CalcMeasureInBase(XI_POINT tMeasurePoint, double dDis, double dMoveZ, double dAngleVer, T_ROBOT_COORS& tMeasureTrans);
	void   CalcMeasureInBase(T_ROBOT_COORS tMeasurePoint, double dDis, double dMoveZ, T_ROBOT_COORS& tMeasureTrans);
	//机械臂坐标系下
	double CalcRobotAngleRZInBase(double dTrackVerDegree);
	//图像处理
	void MeasurePointLaserData(CRobotDriverAdaptor* pRobotDriver, CString str, T_ALGORITHM_POINT& vcPointOnTheSameLine, vector<T_ALGORITHM_POINT>& vcpUpPlatePoints, vector<T_ALGORITHM_POINT>& vcpDownPlatePoints, T_ANGLE_PULSE tRobotPulses, T_ROBOT_COORS tRobotCoord, int nImageType, int nCameraNo);
	void MeasureVerLaserPoint(CRobotDriverAdaptor* pRobotDriver, T_ALGORITHM_POINT& tFirstPointOnLine, vector<T_ALGORITHM_POINT>& vcpUpPlatePoints,
		vector<T_ALGORITHM_POINT>& vcpDownPlatePoints, T_ANGLE_PULSE tRobotPulses, T_ROBOT_COORS tRobotCoord, int nImageType, int nCameraNo);
	void GetVerWeldLaserLineImageCoors(CvPoint& tCornerImagePoint, vector<CvPoint>& vtLeftImagePoints, vector<CvPoint>& vtRightImagePoints, int nImageType, int nCameraNo);
	void SampleImagePoints(vector<CvPoint> vtImagePoints, int nSampleNum, vector<CvPoint>& vtSampleImagePoints);

	vector<T_ROBOT_COORS> GetRealWeldData(CRobotDriverAdaptor* pRobotDriver, vector<XI_POINT> tWeldPointData, vector<XI_POINT> tWeldPosture);
	vector<T_ROBOT_COORS> GetRealWeldData(CRobotDriverAdaptor* pRobotDriver, vector<T_ROBOT_COORS> tWeldPointData);

    
    //焊接前数据处理，包含初始段补偿，精准对枪
    //bool WeldProcessBefor(CRobotDriverAdaptor *pRobotDriver, vector<T_ROBOT_COORS> vtRealWeldCoors, bool bIsTracking);

	bool WeldProcessBefor(CRobotDriverAdaptor* pRobotDriver, vector<T_ROBOT_COORS> vtRealWeldCoors, vector<int> vtRealWeldCoorType,
		vector<T_ANGLE_PULSE> vtRealWeldPulse,bool bIsTracking);
	// 获取截断数据
	bool GetTruncatedData(CRobotDriverAdaptor* pRobotDriver, T_ROBOT_COORS tRobotRealPos, MoveHandle_Pnt& tRobotCoor, long& lRobotVel, long& lMachinePos, long& lMachineVel);
	//发送焊接数据
	bool CallJobRobotWeld(CRobotDriverAdaptor* pRobotDriver, vector<XI_POINT> vcPointOnTheSameLine, vector<XI_POINT> vcWeldPosture, bool bCalcPosture, bool bIfWeld);
	//，运动函数，机械臂或和大车同时移动,bIsTracking:true实时跟踪，false先测后焊
	bool DoweldRuning(CRobotDriverAdaptor* pRobotDriver, vector<T_ROBOT_COORS>vtRealWeldCoors, vector<int> vnPointType, bool bIsTracking, E_WELD_SEAM_TYPE eWeldSeamType = E_FLAT_SEAM);
	
	// 安川机器人和松下外部轴跟踪运动
	bool TrackingWeldMoveForPanasonic(CRobotDriverAdaptor* pRobotDriver, vector<T_ROBOT_COORS> vtRealWeldCoors, vector<int> vnPointType, bool bIsTracking, E_WELD_SEAM_TYPE eWeldSeamType);
	// 安川松下算法插补算法初始化
	void InitMoveHandleYaskawaPanasonic(T_ROBOT_COORS tTrackingStartCoord, double dWeldSpeed);

	bool DoweldRuningSwing(CRobotDriverAdaptor* pRobotDriver, vector<T_ROBOT_COORS>vtRealWeldCoors, vector<int> vnPointType, bool bIsTracking);
	//埃斯顿跟踪主控线程
	static UINT ThreadMainContData4Estun(void* pParam);
	//埃斯顿摆动跟踪流程 waveType == 0 不摆动； wave Type == 1前后摆动
	bool EstunTracking(CRobotDriverAdaptor* pRobotDriver, std::vector<T_ROBOT_COORS> vtRobotCoors, double dSpeed, int waveType);

	bool SendDataTOCallJobRobot(CRobotDriverAdaptor* pRobotDriver, vector<T_ROBOT_COORS> vtRealWeldCoors);
	//直角坐标运动函数(包含大车)
	bool MoveByJob(CRobotDriverAdaptor* pRobotDriver, T_ROBOT_COORS tRobotJointCoord, T_ROBOT_MOVE_SPEED tPulseMove, int nExternalAxleType, CString JobName = "MOVJ", bool bSingleMove = false);
	//关节坐标运动函数(包含大车)
	bool MoveByJob(CRobotDriverAdaptor* pRobotDriver, T_ANGLE_PULSE tRobotJointCoord, T_ROBOT_MOVE_SPEED tPulseMove, int nExternalAxleType, CString JobName = "MOVJ", bool bSingleMove = false);
	// 大车移动
	bool PosMoveMachine(CRobotDriverAdaptor* pRobotDriver, double dMoveDis, double dSpeed, bool bCheckDone = true);
	//计算大车和机械臂位置
	//参数bRobotDir：为true强制机械臂在右边，false,根据设备极限计算，nIsExAxis：表示是否有外部轴
	//默认机械臂在X轴左侧运行，除非工件在右侧极限位置
	bool CalcRobotAndMachinePos(CRobotDriverAdaptor* pRobotDriver, T_ROBOT_COORS WorldCoors, T_ROBOT_COORS& tCoutRobotCoor);
	//获取位置(包含大车)
	T_ANGLE_PULSE GetCurrentPulse(CRobotDriverAdaptor* pRobotDriver);
	T_ROBOT_COORS GetCurrentPos(CRobotDriverAdaptor* pRobotDriver);

	void    IntValWaitFun(CRobotDriverAdaptor* pRobotDriver, int nIntVal, int nVal);
	bool    m_bRobotWeldDir;//机器人焊接方向，默认在X轴左侧
	bool WeldLockBefor(CRobotDriverAdaptor * pRobotDriver, vector<T_ROBOT_COORS> tCoors, vector<int> tCoorsType, vector<T_ROBOT_COORS>& tCoutCoors, bool bIsTrack);
	//焊接前锁定，用于变姿态
	//bool	WeldLockBefor(CRobotDriverAdaptor* pRobotDriver, T_ROBOT_COORS tStartP, T_ROBOT_COORS tEndP, bool bIsTrack = false);
    bool    WeldLockBefor(CRobotDriverAdaptor* pRobotDriver, vector<T_ROBOT_COORS> tCoors, vector<T_ROBOT_COORS>& tCoutCoors, bool bIsTrack);

	typedef struct
	{
		CString strRobotName;
		vector<T_ANGLE_PULSE> vtSafePulse;
	}T_STREET_LAMP;
	//非外部轴
	vector<T_STREET_LAMP> m_vtStreetLamp;
	//外部轴
	int m_nHandlingExternalAxisSafeMove;						//按工作台划分
	vector<T_ANGLE_PULSE> m_vtHandling_ExternalAxisPulse;		//安全位置

	/*
	**线扫数据格式更改，筛选关联信息
	**分组：将可同时焊接的平缝（直线或圆弧）进行分组测量焊接：足够长的焊道自身为一组，完全在机械臂行程内的连续的焊道可多个为一组，每条立缝单独为一个组
	**计算单组测量信息（测量时起点干涉且长度小于手眼距离1.5倍左右的短边测量时需要特殊姿态及位置进行测量）
	**开始测量得到测量数据，及要更新的端点（更新实际相连的焊道，有时长边背面的自由端小短边根据板厚投影得到相对准确的端点）
	**开始焊接：焊接时在机械臂行程内的只动机械臂焊接，超出机械臂行程的大车联动焊接
	**起始结尾测量：干涉段交点利用线线相交或面面相交求得端点，自由端利用搜索的方式得到
	**长度超出500mm的焊道中间过程增加跟踪
	*/
	//初始化参数，工艺
	void InitWeldStartVal(CRobotDriverAdaptor* pRobotDriver);
	void SetWeldTlyParam(T_WELD_PARA tWeldPara);
	void LoadWorkPieceInfo();										//加载工件信息
	void InitWorkPieceReleveInfo(CRobotDriverAdaptor* pRobotDriver);

	VVT_CORNER_LINE_INFO_NEW m_vvtCornerLineInfoNew;
	VVT_ENDPOINT_INFO_NEW m_vvtPointInfoNew;
	VVT_VERTICAL_INFO_NEW m_vvtVerticalInfoNew;
	VVT_MEASURE_INFO_NEW m_vvMeasureInfoNew;
	VVT_CIRCLE_CENTER_INFO_NEW m_vvtCircleCenterInfo;

	double m_dWorkPieceHighTop;										// 工件最高点
	double m_dWorkPieceLowTop;										// 工件最低点
	double m_dRobotHighTop;											// 机器人最高点
	double m_dRobotHLowTop;											// 机器人低点
	double m_dWeldDipAngle;											// 焊接倾角
	double m_dParallelAdjust1;										// 包角参数
	double m_dVerticalAdjust1;
	double m_dVelocityWarp;											// 包角速度
	double m_dParallelAdjust2;
	double m_dVerticalAdjust2;
	double m_dVelocityWarp2;										// 包角速度
	double m_dVelocityTransitionBound;
	

	WELD_WRAP_ANGLE m_tWrapAngelParaNew;

	/*----------------------------------------断点续焊------------------------------------------*/
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

	// 切换焊接和包角速度
	void SwitchWeldSpeed(CRobotDriverAdaptor* pRobotDriver, int nCoorIndex, int nCoorType, double dWeldSpeed, double dWarpSpeed, long& lRobotvel, bool& bIsChangeSpeed);
	// 切换焊接和包角电流电压
	void SwitchWeldCurrent(CRobotDriverAdaptor* pRobotDriver, int nCoorIndex, int nCoorType, T_WELD_PARA tWeldParam, bool& bChangeVoltage);
	// 修正包角数据
	bool CorrectWarpDataFun(vector<TrackFilter_XYZ> vWeldCoors, vector<T_ROBOT_COORS> vtWarpCoors, int nWarpIndex, vector<T_ROBOT_COORS>& vtCorrectWarpCoors);
	bool CorrectWarpDataFun(vector<TrackFilter_XYZ> vWeldCoors, vector<T_ROBOT_COORS> vtWarpCoors, int nWarpIndex, vector<T_ROBOT_COORS>& vtCorrectWarpCoors, vector<int>& vtCorrectWarpCoorsType);

	
public:
	CMoveCtrlModule         m_cMoveCtrl;
	//移植新框架时创建
	BOOL m_bWorking;
};

class correctFilletError {
public:

	// - P_ 为跟踪点集，选取包角第二点前的一段跟踪点集
	// - filletP_ 为理论包角的四点，第一点为起始点，第二点为第一次转弯点，第三点为第二次转弯点，第四点为结束点
	// - diffD1_ 为分割 由第一点至第二点 和 由第三点至第四点 的两条线段（两平行线）的分割宽度，单位为 mm
	// - diffD2_ 为分割 由第二点至第三点 的线段的分割宽度，单位为 mm
	// - minNum2_ 为分割 由第二点至第三点 线段的分割点个数下限，单位为 个
	// - permissibleError_ 为拟合直线时使用的精度参数，单位为 mm 。正常情况下，数值越小精度越高，但过小的数值可能会导致无法拟合，这时候默认使用跟踪点集的前后两点拟合直线，影响精度。参考范围（0.5--1）
	// - permissibleRate_ 为拟合直线时的目标精度参数，单位为 100% 。正常情况下，数值越大精度越高，过大数值可能会影响程序运行速度。参考范围（0.8-0.95）
	correctFilletError(std::vector<XI_POINT> p_, std::vector<XI_POINT> filletP_, double diffD1_, double diffD2_, int minNum2_, double permissibleError_ = 0.55, double permissibleRate_ = 0.9);

	std::vector<XI_POINT> getRes();

	void getResNew();
	// nRobotDir:机器人安装方向
	std::vector<XI_POINT> GetGdiff(std::vector<XI_POINT> vtRobotPostureCoors, std::vector<XI_POINT>& vtRobotPosture, int nRobotDir = 1);

private:

	double distance(size_t a, size_t b);
	double distance(const XI_POINT& a, const XI_POINT& b);

	// 拟合跟踪点集直线
	void fittedLine();

	// 计算点集中其他点与拟合直线的距离
	double disToLine(size_t tempFittedStartPointIndex, size_t tempFittedEndPointIndex, size_t C);

	// 计算校正后的包角点
	void correctFillet();

	// 由start点向end点的射线上根据距离start点的距离找点
	XI_POINT findPoint(const XI_POINT& start, const XI_POINT& end, double dis);

	// 根据拟合直线计算前两点
	XI_POINT find0and1(size_t n);

	// 根据四点集中前两点计算后两点
	XI_POINT find2and3(size_t n);

	// 由校准后的四点计算离散点集
	std::vector<XI_POINT> diff();

private:
	//------------------------------------输入-----------------------------------

	// 跟踪点集
	const std::vector<XI_POINT> p;

	// 线扫理论包角四点（开始点，拐点，拐点，结束点）
	const std::vector<XI_POINT> filletP;

	// 拟合跟踪点集直线时允许的最大误差距离/
	double permissibleError;

	// 拟合跟踪点集直线时拟合率下限/
	double permissibleRate;

	// 离散第一边与第三边的距离常数
	const double diffD1;

	// 离散第二边的距离常数
	const double diffD2;

	// 离散后第二边最少点数（包括两端点，不小于3）
	const int minNum2;

	// -------------------中间值--------------------------------------------------

	// 校正后包角四点
	std::vector<XI_POINT> correctFilletP;

	// 跟踪点集拟合直线第一点的索引
	size_t fittedStartPointIndex;

	// 跟踪点集拟合直线第二点的索引
	size_t fittedEndPointIndex;
};
#endif // !defined(AFX_DIAPHRAGMWELD_H__27CA3DC8_2FB9_4286_8B23_8FBE2B7E4E60__INCLUDED_)
