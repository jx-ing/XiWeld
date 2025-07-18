/*********************************************************************
* 版权所有 (C)2018, 哈尔滨行健智能机器人股份有限公司
*
* 文件名称： WeldAfterMeasure.h
* 内容摘要： 先测后焊统一的识别测量焊接流程接口 基础计算函数 
* 当前版本： 1.0
* 作    者： 刘彦斌
* 完成日期： 2023年6月15日
* 其它说明：
*
* 修改记录1：
* 
* 修改记录2：
* 
注意：

**********************************************************************/

#pragma once
#include <vector>
#include <map>

#include ".\Apps\PLib\CtrlUnit\CUnit.h"
#include "XiAlgorithm.h"
#include "AbsCoorTransLib.h"

#include "GsImageProcess.h"
#include "WeldingLinesRecognition.h"
#include "PointCloudWeldingLinesExtraction.h"
#include "RiserManagement.h"

#include "ScanInitModule.h"
//#include "WeldParamProcess.h"
#include "Project/ParamInput.h"
//#include"StatisticalData.h"
#include "Project/WeldParamProcess.h"
#include "Project/IOCtrlDlg.h"
#include ".\Apps\PLib\LeisaiCtrl\StepMoveControlJY.h"
#include ".\LocalFiles\ExLib\ALGO\include\MoveHandle_C_DLL.h"
#include ".\LocalFiles\ExLib\ALGO\include\TrackFilterHandle_C_DLL.h"

#include "xi_export_ROBOTCOLLIDECLOSEDPLANE.h"
#include "robot_collide_closed_plane_interface.h"

using namespace std;

struct WeldSeamAtrribute
{
	int		nWeldSeamIdx; // 焊缝索引
	int		nIsDoubleWelding;
	int		nStartWrapType;// 自由端：暂定0不做操作，1：起点定点测量，终点不做操作，2：终点定点测量，起点不做操作，3：起终点都定点测量
	int		nEndWrapType;// 用作手动分组
	double 		nWeldAngleSize;
	double dStartHoleSize;
	double dEndHoleSize;
	int	   nRobotSelete;
	double dThoeryLength;// 理论长度
	double dGroupTrackPos;
	int	   nGroupNo;   // 同一组内不同小组
	int    nBigGroupNo;// 大组，不同机械臂同时焊接位置
	bool   bStartFixScan = false; // 先测后焊默认false
	bool   bEndFixScan = false; // 先测后焊默认false
	bool   bWeldMode = false; // 焊接方式：true:跟踪，false:先测后焊 
	int	   nScanEndpointNum = 0; // 搜端点次数

};

struct WeldLineInfo
{
	LineOrCircularArcWeldingLine tWeldLine;
	WeldSeamAtrribute tAtrribute;
};

struct TempStruct
{
	int nIdx;
	T_ROBOT_COORS tCoord;
};

template<typename T>
inline bool WeldingLineIsArc(T tValue)
{
	return tValue.IsArc > 0 && tValue.IsArc < 10;
}

namespace WAM
{
	#define CHECK_EXAXLE_RETURN(pMoveCtrl, dExAxlePos) \
	if (false == CompareExAxlePos(pMoveCtrl, dExAxlePos)) \
	{ \
		return false; \
	}

	class WeldAfterMeasure
	{
	public:
		WeldAfterMeasure(CUnit* ptUnit, E_WORKPIECE_TYPE eWorkPieceType);
		~WeldAfterMeasure();

		/********************* 子类通用功能函数移植 *********************/
	public:
		// 将测量数据 vtNewTeachData 追加到 vtTeachData
		void AppendTeachData(vector<T_TEACH_DATA>& vtTeachData, vector<T_TEACH_DATA> vtNewTeachData);	
		// 将世界坐标分解为外部轴坐标和机器人坐标(需要支持多轴，目前仅支持Y轴外部轴)
		void DecomposeExAxle(T_ROBOT_COORS& tRobotCoord, double dExAxlePos);
		void DecomposeExAxle(T_ROBOT_COORS& tRobotCoord, double dExAxlePos, int nExAxleNo);
		// 检查测量点tTeachCoord坐标是否已存在与vtTotalTeachCoord中 tThreshold为阈值 返回在vtTotalTeachCoord中的索引 不存在返回-1
		int CheckTeachCoord(const T_ROBOT_COORS& tTeachCoord, const vector<T_ROBOT_COORS>& vtTotalTeachCoord, T_ROBOT_COORS tThreshold);
		// 获取描述tTeachData的字符串
		CString GetTeachDataString(const T_TEACH_DATA& tTeachData);

		long GetMoveExAxisPos(T_ANGLE_PULSE tPulse, int nAxisNo);
		double GetMoveExAxisPos(T_ROBOT_COORS tCoord, int nAxisNo);

		/********************* 静态通用函数 *********************/
	public:
		static void SaveWorkpieceType(CRobotDriverAdaptor* pRobotCtrl);
		static void LoadWorkpieceType(CRobotDriverAdaptor* pRobotCtrl, std::map<int, CString> &m_nsPartType);

		/********************* 基础功能函数 *********************/
	public:
		// 加载系统设备参数
		void LoadEquipmentParam();
		// 加载焊接补偿参数
		void LoadCompParam();
		// 设置处理中间数据保存路径
		void SetFilePath();
		// 设置焊接参数：手眼距离 测量点据端点距离 焊接和测量机器人姿态Rx Ry
		void SetWeldParam(BOOL *pIsArcOn, BOOL *pIsNaturalPop, BOOL* pIsTeachPop, BOOL* bNeedWrap);
		// 设置硬件资源
		void SetHardware(/*CMoveCtrlModule* pMoveCtrl, CGroupLaserVision* pDHCameraVision, CIOControl* pIOControl, */CScanInitModule* pScanInit);
		// 加载点云数据
		void LoadContourData(CRobotDriverAdaptor* pRobotDriver, CString cFileName, CvPoint3D64f* pPointCloud, int& nPtnNum);
		void LoadContourData(CRobotDriverAdaptor* pRobotDriver, CString cFileName, vector<CvPoint3D64f> &vtPointCloud);
		void LoadContourData3D(CRobotDriverAdaptor* pRobotDriver, CString cFileName, vector<CvPoint3D64f> &vtPointCloud);
		void SaveContourData();		//将点云数据合并到一个文件中
		// 保存点云处理结果
		bool SavePointCloudProcessResult(LineOrCircularArcWeldingSeam* pWeldSeams, int nWeldLineNumber);
		bool SavePointCloudProcessResult(LineOrCircularArcWeldingLine* pWeldSeams, int nWeldLineNumber);
		bool SavePointCloudProcessResult(const Welding_Info &tWeldInfo);
		bool SaveSplitAfterPointCloudProcessResult(std::vector<WeldLineInfo> vtWeldSeamInfo, CString sFileName = "",bool IsArc = false);
		// 加载点云处理结果
        bool LoadCloudProcessResultNew(CString sFileName = "");
		bool LoadCloudProcessResult(CString sFileName, vector<WeldLineInfo>& vtWeldLineInfo); // 加载指定文件中的焊缝信息
		// 拆分点云结果,区分直线和圆弧
		bool SplitCloudProcessResult(CString sFileName = "");
		// 保存焊缝分组结果
		void SaveWeldSeamGroupInfo();
		// 保存示教结果数据
		virtual void SaveTeachResult(int nGroupNo);
		virtual void SaveTeachResultWorld(int nGroupNo);
		// 加载示教结果
		bool LoadTeachResult(int nGroupNo, int nLayerNo = -1);
		bool LoadTeachResult(int nGroupNo, int nLayerNo, std::vector<T_TEACH_RESULT> &vtTeachResult);
		// 加载焊接轨迹数据 第nGroupNo组 第nWeldNo个焊缝
		virtual bool LoadRealWeldTrack(int nGroupNo, int nWeldNo, E_WELD_SEAM_TYPE&eWeldSeamType, double &dExAxlePos, vector<T_ROBOT_COORS>& vtRealWeldTrack, vector<int> &vnPtnType);
		// 检查轨迹vtRobotPulse中坐标(XY) 法兰是否都远离坐标原点
		bool CheckFlangeToolCoord(int nGroupNo, int nWeldNo, vector<T_ANGLE_PULSE> vtRobotPulse);

		bool AddMeasurePointsToLongSeam(int nWeldNo, std::vector<std::vector<T_TEACH_DATA>> vvtTeachData, LineOrCircularArcWeldingLine tWeldSeam, 
										double dPtnInterval, double dPostureRx, double dPostureRy, double dWeldDirAngle, E_WELD_SEAM_TYPE eWeldSeamType , vector<T_ROBOT_COORS> &vtWeldCoord );
		// 保存焊缝轨迹(补偿后焊接轨迹)																																									 
		void SaveWeldTrack(int nGroupNo, int nWeldNo, int nLayerNo, E_WELD_SEAM_TYPE eWeldSeamType, double dExAxlePos, vector<T_ROBOT_COORS> vtWeldCoord, vector<int> vnPtnType);
		// 判断两个方向角差值是否小于dThreshold
		bool JudgeDirAngle(double dDir1, double dDir2, double dThreshold =45.0) const;
		bool JudgeAngle(double dPositiveLimit, double dNegativeLimit, double dAngle1, double dAngle2, double dThreshold) const;
		// 计算平面两个方向角的角平分线 方向角(取两个方向角中值)
		double TwoDirAngleMedian(double dDirAngle1, double dDirAngle);
		double TwoAngleMedian(double dPositiveLimit, double dNegativeLimit, double dAngle1, double dAngle2);
		// 方向角 -> Rz
		double DirAngleToRz(double dDirAngle);
		double DirAngleToRz(double dBaseRz, double dBaseDirAngle, double dChangeDir, double dDirAngle);
		// Rz -> 方向角
		double RzToDirAngle(double dRz);
		double RzToDirAngle(double dBaseRz, double dBaseDirAngle, double dChangeDir, double dRz);
		// 生成直角坐标
		T_ROBOT_COORS GenerateRobotCoord(CvPoint3D64f tSrcPtn3D, double dRx, double dRy, double dRz);
		T_ROBOT_COORS GenerateRobotCoord(T_ALGORITHM_POINT tAlgPtn3D, double dRx, double dRy, double dRz);
		// 生成多个坐标
		void GenerateMultipleCoord(T_ROBOT_COORS tStartCoord, T_ROBOT_COORS tEndCoord, vector<T_ROBOT_COORS>& vtCoords, double dInterval = 1.0);
		void GenerateMultipleCoord(XI_POINT tStartCoord, XI_POINT tEndCoord, vector<XI_POINT>& vtCoords, double dInterval = 1.0);
		// 将点tSrcPtn 向点tDirSPtn指向点tDirEPtn的方向偏移 dOffsetDis 单位:mm
		T_ROBOT_COORS RobotCoordPosOffset(T_ROBOT_COORS tSrcPtn, T_ROBOT_COORS tDirSPtn, T_ROBOT_COORS tDirEPtn, double dOffsetDis); 
		T_ROBOT_COORS RobotCoordPosOffset(T_ROBOT_COORS tSrcPtn, CvPoint3D64f tDirSPtn, CvPoint3D64f tDirEPtn, double dOffsetDis);
		XI_POINT RobotCoordPosOffset(XI_POINT tSrcPtn, XI_POINT tDirSPtn, XI_POINT tDirEPtn, double dOffsetDis);
		T_ROBOT_COORS RobotCoordPosOffset(T_ROBOT_COORS tSrcPtn, T_LINE_PARA tLineParam, double dOffsetDis);
		T_ALGORITHM_POINT RobotCoordPosOffset(T_ALGORITHM_POINT tSrcPtn, T_ALGORITHM_POINT tDirSPtn, T_ALGORITHM_POINT tDirEPtn, double dOffsetDis);
		// 机器人直角坐标X Y Z偏移
		void RobotCoordPosOffset(T_ROBOT_COORS& tSrcCoord, double dOffsetDir, double dOffsetDis, double dHeightOffsetDis = 0.0);
		// 三维点类型转换
		T_ALGORITHM_POINT TransPtn3D(XI_POINT tPtn);
		XI_POINT TtansPtn3D(T_ALGORITHM_POINT tPtn);
		// 计算可连续运动的 相邻坐标姿态变化小于180度的 多个关节坐标
		bool CalcContinuePulse(vector<T_ROBOT_COORS>& vtRobotCoors, vector<T_ANGLE_PULSE>& vtRobotPulse);
		bool CalcContinuePulseForWeld(vector<T_ROBOT_COORS> &vtWeldCoord, vector<T_ANGLE_PULSE>& vtWeldPulse, bool bWeldTrack = true);
		bool CalcContinuePulseForMeasure(vector<T_ROBOT_COORS>& vtRobotCoors, vector<T_ANGLE_PULSE>& vtRobotPulse, bool bWeldTrack = true);
		// 判断第nGroupNo组焊接轨迹是否可以连续运动-包容线扫误差
		bool CheckContinueMove_Offset(int nGroupNo, double dExAxisPos, double yPos, double offSet, double ry, double rx);
		// 使用用点云识别结果计算焊缝 并 判断是否可以连续运行
		bool JudgeTheoryTrackContinueMove(int nGroupNo, /*double dCarPos, */double dExAxlePos, double yPos);
		// 判断第nGroupNo组焊接轨迹是否可以连续运动
		bool CheckContinueMove(int nGroupNo, /*double dCarPos, */double dExAxisPos, double yPos);
		bool CheckContinueMove_rAngle(int nGroupNo, double dExAxisPos, double yPos, double ry, double rx);
		// 外部轴坐标对比
		bool CompareExAxlePos(CMoveCtrlModule* pMoveCtrl, double dDstPos, double dLimit = 3.0);
		bool TransCoordByCamera(std::vector<T_ROBOT_COORS>& vtCoord, T_ROBOT_COORS tSrcTool, T_ROBOT_COORS tDstTool);
		// 根据处理类型调用不同图像处理接口 返回 二维点数据
		bool ProcessTeachImage(int nTeachPtnNo, GROUP_STAND_DIP_NS::CImageProcess* pImgProcess, IplImage* pImg, T_ANGLE_PULSE tCapPulse, int nMeasureType, T_TEACH_RESULT& tTeachResult, E_FLIP_MODE eFlipMode);
		// 检查图像处理二维点有效性
		bool CheckImgProPtn2D(int nMeasureType, CvPoint tKeyPoint, vector<CvPoint> vLeftPtn, vector<CvPoint> vRightPtn);
		// 点类型转换，vtXiPoint追加保存到vtAlgPtns
		void XiPointAddToAlgPtn(const vector<XI_POINT>& vtXiPoint, vector<T_ALGORITHM_POINT>& vtAlgPtns);
		// 计算两平面交线
		T_ALGORITHM_LINE_PARA PlanePlaneInterLine(T_ALGORITHM_PLANE_PARAM tPlane1, T_ALGORITHM_PLANE_PARAM tPlane2);
		// 修正tWeldSeam的 起点 终点 起点法相 终点法相
		bool AdjsutWeldSeam(LineOrCircularArcWeldingLine& tWeldSeam, T_ALGORITHM_POINT tAlgStartPtn, T_ALGORITHM_POINT tAlgEndPtn, double dNorAngle, double dZSide);
		// 修正扫描测量的焊缝信息
		bool AdjustScanWeldSeam(int nGroupNo, int nWeldNo, std::vector<T_TEACH_DATA>& tTeachData);
		// 修正第nGroupNo组 第nWeldNo个焊缝，弧形焊缝
		bool AdjustWeldSeamArc(int nGroupNo, int nWeldNo, std::vector<T_TEACH_DATA>& tTeachData);
		// 判断LineOrCircularArcWeldingLine 焊接类型
		E_WELD_SEAM_TYPE GetWeldSeamType(const LineOrCircularArcWeldingLine& WeldSeamType);
		// 获取当前选择的平焊或立焊工艺参数
		bool GetCurWeldParam(E_WELD_SEAM_TYPE eWeldSeamType, vector<T_WELD_PARA> &vtWeldPara);
		// 获取指定平焊或立焊 指定焊脚名称的 工艺参数
		bool GetWeldParam(E_WELD_SEAM_TYPE eWeldSeamType, int nWeldAngleSize, vector<T_WELD_PARA> &vtWeldPara);
		// 计算包角轨迹 （注意不同工件类型具体实现不同）
		bool CalcWrapTrack(WeldLineInfo tWeldLineInfo, E_WELD_SEAM_TYPE eWeldSeamType, double dStandWeldWrapLen, int nStandWeldDir, vector<T_ROBOT_COORS>& vtWeldCoord, vector<int>& vnPtnType);
		// 根据方向角 + 高板是否在左 自动计算立焊包角方向（缺少信息，不完全通用，例如筋板高于翼板时）注意：dDirAngle表示法相的向反方向
		void AutoCalcStandWrapOffsetCoord(double dDis, double dDirAngle, bool bHeightBoardIsLeft, T_ROBOT_COORS& tRobotCoord);
		// 轨迹坐标添加补偿值
		void TrackComp(int nGroupNo, E_WELD_SEAM_TYPE eWeldSeamType, const vector<T_ROBOT_COORS>& vtSrcWeldCoord, const T_WELD_PARA& tWeldPara, vector<T_ROBOT_COORS>& vtAdjustWeldCoord);
		// 计算第nGrouopNo组焊缝最高点Z值
		double CalcBoardMaxHeight(int nGroupNo);
		// 判断是否是立焊 tVector焊缝法向量
		bool IsStandWeldSeam(CvPoint3D64f tVector);
		// 对缝合板焊缝(小于4道焊缝)的补全完整信息 并排序 未计算测量轨迹提供完整数据
		bool ResetWeldSeamGroup(int nGroupNo, vector<LineOrCircularArcWeldingLine>& vtWeldLineSeam);
		// 判断机头和工件是否会干涉
		bool JudgeCollision(double dRy, double dAngle, double dBackH, double dLH, double dRH, double dFrontBackDis, double dLRDis, double dGunSafeDis, double dSafeDis);
		// 使用示教结果vtTeachResult(仅用于修正直线的示教结果)
		bool AdjustLineSeamTrack(E_WELD_SEAM_TYPE eWeldSeamType, const vector<XI_POINT>& vtScanPtns, vector<T_ROBOT_COORS>& vtWeldTrack);
		// 添加两个关节坐标的过渡点(脉冲中止算姿态，位置向运动方向-90°的方向偏移) 两个点必须有足够距离
		bool CalcTwoPulseTempPtn(T_ANGLE_PULSE& tTempPulse, T_ANGLE_PULSE tStartPulse, T_ANGLE_PULSE tEndPulse, double dLevelOffsetDis, double dHeightOffset = 0.0);
		// 判断该组焊缝是否为外侧焊缝(单条焊缝按内测焊缝处理)
		bool JudgeOutsideWeld(const vector<LineOrCircularArcWeldingLine>& vtWeldSeam);
		// 将焊缝组中所有平焊焊缝编号，按可连续焊接顺序排序并输出(保证输出焊接轨迹顺序，连接后即可连续焊接)
		bool GetContiueWeldOrderFlat(const vector<LineOrCircularArcWeldingLine>& vtWeldSeam, vector<int>& vnFlatWeldNoIdx);

		/********************* 线程及线程函数 *********************/
		bool WaitAndCheckThreadExit(CWinThread* pWinThread, CString sThreadName);
		static UINT ThreadTeachProcess(void* pParam);
		bool TeachProcess(int nTimeOut = 20000); // 超过nTimeOut毫秒没有示教图 超时退出

        //记录时间
        double RecordRunTime(CRobotDriverAdaptor *pRobotCtrl, long ltimeBegin, CString str);
		//检测机器人急停
		void CheckMachineEmg(CRobotDriverAdaptor *pRobotCtrl);
        bool WeldTrackLock(CRobotDriverAdaptor* pRobotCtrl, T_ROBOT_COORS tStartP, T_ROBOT_COORS tEndP, bool bIsTrack);
		bool IsWorking();

		/********************* 坡口作业函数 *********************/
		bool FreeWaveGrooveWeld(vector<vector<T_ROBOT_COORS>> vvtGrooveWavePath, vector<T_WAVE_PARA> vtTWavePara, vector<vector<double>> vWeldSpeedRate);
		bool DoFreeGrooveWelding(std::vector<T_ROBOT_COORS>& vtWeldPathPoints, T_WAVE_PARA tWavePara, const T_WELD_PARA& tWeldPara, vector<double> weldSpeedRate);
		int m_nGroupNo; // 坡口使用 当前焊接组号
		int m_nLayerNo; // 坡口使用 当前焊接层号(此处指道号)

		/********************* 运动相关作业函数 *********************/
	public:
		//// 设置运动轨迹类型 轨迹点类型 点数 外部轴位置等示教数据
		void SetTeachData(const std::vector<T_ANGLE_PULSE>& vtMeasurePulse, const vector<int>& vnMeasureType, double dExAxlePos, double& dExAxlePos_y);
		//// 执行测量运动
		virtual bool DoTeach(int nGroupNo, const std::vector<T_ANGLE_PULSE>& vtMeasurePulse, const vector<int>& vnMeasureType, double dExAxlePos, int nLayerNo = 0);
		// 执行软触发采图
		virtual bool SpuriousTriggerTeach(int nGroupNo, const std::vector<T_ANGLE_PULSE>& vtMeasurePulse, const vector<int>& vnMeasureType, double dExAxlePos);
		//// 执行焊接运动
		bool Weld(int nGroupNo);
		//// 先测后焊通用焊接
		virtual bool DoWelding(int nGroupNo, int nWeldNo, E_WELD_SEAM_TYPE eWeldSeamType, std::vector<T_ROBOT_COORS>& vtWeldPathPoints, const vector<int>& vnPtnType, const T_WELD_PARA &tWeldPara);
		// vtPulse:下枪 搜索起点 搜索终点 收枪 nDownBackSpeed收下枪速度 nScanSpeed扫描速度 保存端点到tTeachResult.tKeyPtn3D
		virtual bool ScanEndpoint(int nGroupNo, int nCameraNo, vector<T_ANGLE_PULSE> vtPulse, vector<int> vnType, vector<T_TEACH_RESULT>& tTeachResult);
		// 获取ScanEndpoint的测量结果
		bool GetScanProcResult(int nGroupNo, bool bIsScanMode, bool bSinglePointMeasure, vector<T_ROBOT_COORS> vtCoord, vector<T_TEACH_RESULT>& vtTeachResult);
		// 在焊缝组vtSeamGroup中获取第nSeamNo条焊缝的相邻焊缝 第nSeamNo条平焊：查找平焊起点终点相邻的平焊焊缝  第nSeamNo条立焊：查找起点与立焊起点相邻和终点与立焊起点相邻的平焊焊缝
		void GetAdjoinSeam(int nSeamNo, vector<LineOrCircularArcWeldingLine>& vtSeamGroup, LineOrCircularArcWeldingLine** ptAdjoinSeamS, LineOrCircularArcWeldingLine** ptAdjoinSeamE);
		void GetAdjoinSeam(LineOrCircularArcWeldingLine tSeam, vector<LineOrCircularArcWeldingLine>& vtSeamGroup, LineOrCircularArcWeldingLine** ptAdjoinSeamS, LineOrCircularArcWeldingLine** ptAdjoinSeamE);
		// 计算锐角干涉变姿态轨迹删除信息
		void CalcInterfereInfo(int nSeamNo, vector<LineOrCircularArcWeldingLine> vtWeldSeam, vector<T_ROBOT_COORS>& vtWeldCoord,
			double& dChangeDisS, double& dChangeDisE, int& nChangePtnNum, int& nDelPtnNum, double& dWeldHoleSizeS, double& dWeldHoleSizeE);
		// 判断时候为锐角焊缝
		bool IsAcuteSeamStand(LineOrCircularArcWeldingLine tSeam, double dAcuteAngleThreshold);
		// 获取当前工件 所有焊缝信息最高点 (世界坐标 已考虑正座倒挂)
		XI_POINT GetPieceHeight(int nGroupNo, double zOffeset);

		int m_nSmallGroupNoForTeach = -1;//测量时，当前是第几小组的焊缝

		/********************* 必须重新实现的部分 *********************/
	public:
		// 设置识别参数
		virtual void SetRecoParam() = 0;
		// 点云处理
		virtual bool PointCloudProcess(bool bUseModel, CvPoint3D64f* pPointCloud = NULL, int PointCloudSize = 0) = 0;
		// 点云处理 模型
		bool PointCloudProcessWithModel();
		
		// 点云处理结果进行排序分组
		virtual bool WeldSeamGrouping(int& nWeldGroupNum) = 0;
		// 计算测量运动轨迹
		virtual bool CalcMeasureTrack(int nGroupNo, std::vector<T_ROBOT_COORS>& vtMeasureCoord, std::vector<T_ANGLE_PULSE>& vtMeasurePulse, vector<int>& vnMeasureType, double& dExAxlePos, double& dSafeHeight) = 0;
		//// 计算精确焊缝轨迹 dWeldHoleSize:过焊孔尺寸
		virtual bool CalcWeldTrack(int nGroupNo) = 0;

		/********************* 静态函数 *********************/
		public:
		// 根据工件类型获取工件名称 工件描述中文名称 和 输出类型名称
		static bool GetWorkPieceName(E_WORKPIECE_TYPE eWorkPieceType, CString& sWorkPieceName, CString& sWorkPieceTypeName);
		// 输入参数输入窗口
		static void InputParaWindow(CString sUnitName, E_WORKPIECE_TYPE eWorkPieceType);
		
		/********************* 可重新实现的部分 *********************/
	public:
		// 由起点终点坐标及姿态生成直角坐标焊缝轨迹点云 bTheoryCheck:是否是使用识别焊缝生成理论焊接轨迹(计算测量轨迹检查焊接轨迹是否可连续运动使用true)
		virtual bool GenerateWeldLineData(int nGroupNo, double dWeldHoleSize, double dBoardThick, bool bTheoryCheck = false);
		// 生成一个焊缝的焊缝轨迹
		//bool GenerateWeldLineData(const LineOrCircularArcWeldingLine& tWeldSeam, int nGroupNo, int nWeldNo, double dWeldHoleSize, double dBoardThick);
		bool GenerateWeldLineData(const LineOrCircularArcWeldingLine& tWeldSeam, int nGroupNo, int nWeldNo, double dWeldHoleSize, double dBoardThick, std::vector<XI_POINT> vtScanPtns);

		bool GenerateWeldLineData(CvPoint3D64f tStartPtn, CvPoint3D64f tEndPtn, double dInterval, double dRx, double dRy, double dWeldNorAngle, E_WELD_SEAM_TYPE eWeldSeamType, std::vector<T_ROBOT_COORS>& vtRobotCoors);
		bool GenerateWeldLineData(XI_POINT tStartPtn, XI_POINT tEndPtn, double dInterval, double dRx, double dRy, double dWeldNorAngle, E_WELD_SEAM_TYPE eWeldSeamType, std::vector<T_ROBOT_COORS>& vtRobotCoors);
		// 生成圆弧焊接轨迹
		bool GenerateWeldTrackArc(LineOrCircularArcWeldingLine tWeldLine);
		// 根据轨迹点生成机器人坐标点
		bool CalcRobotPostureCoors(vector<XI_POINT> vtOutPoints, double dRX, double dRY, double dStepDis, double dStChangePostureDis, double dStChangeAngle,
			double dEdChangePostureDis, double dEdChangeAngle, bool bStartType, bool bEndType, double dStartHoleSize, double dEndHoleSize, int& nStartChangeStepNo, int& nEndChangeStepNo, vector<T_ROBOT_COORS>& vtCoutCoors);
		//bool ModelMatching(double dCorasrMatrix[][4], std::string ModelFileName, int nWeldPartNo, std::string SaveRoute_weldline_point, double dFineMatrix[][4]);
		bool CalcRobotPostureCoors(vector<XI_POINT> vtOutPoints, double dRX, double dRY, double dStepDis, double dStChangePostureDis, double dStChangeAngle,
			double dEdChangePostureDis, double dEdChangeAngle, bool bStartType, bool bEndType, double dStartHoleSize, double dEndHoleSize, int& nStartChangeStepNo, int& nEndChangeStepNo, vector<XI_POINT>& vtPosture);

		/********************* 测试函数 *********************/
	public:
		// 测试函数
		void SaveCoordToFile(std::vector<T_ROBOT_COORS> vtCoord, CString sFileName);
		// 生成理论示教结果数据(调试使用)
		bool GeneralTeachResult(int nGroupNo, std::vector<T_ROBOT_COORS>& vtMeasureCoord, std::vector<T_ANGLE_PULSE>& vtMeasurePulse, vector<int>& vnMeasureType, double& dExAxlePos);
		void Test();
		// 生成使用程序变量的Job
		void GenerateJobSpotWeld(vector<T_ANGLE_PULSE> vtRobotPulse, int nMoveType, CString sJobName);
		void GenerateJobLocalVariable(vector<T_ANGLE_PULSE> vtRobotPulse, int nMoveType, CString sJobName, int nStep = 10);
		void GenerateJobCoord(vector<T_ROBOT_COORS> vtRobotCoord, int nMoveType, CString sJobName);
		void GenerateJobForContinueWeld(int nGroupNo, bool bUsedExAxle = false);
		//void GenerateFilePLYPlane(double dExAxisPos, CString sName = "AllWeldSeamPlane");
		void GenerateFilePLYPlane(vector<WeldLineInfo> vtWeldSeamPlane, CString sName = "AllWeldSeamPlane");
		void GenerateFilePLY(int nGroupNo, double dExAxlePos = 0.0, CString sName = "AllWeldSeam");
		void GenerateFilePLY(CvPoint3D64f* pPointCloud, int PointCloudSize, CString sFileName, int nSample = 2, double dExAxlePos = 0.0);

		void SaveToJobBuffer(const vector<T_ANGLE_PULSE> &vtPulse);
		void CleaeJobBuffer();
		vector<vector<T_ANGLE_PULSE>> m_vvtJobPulse;
		// 将点云绕指定点及Z轴旋转180°
		void PointCloudRotate180(vector<CvPoint3D64f> &vtPtns, double dCenX, double dCenY);
		// 将示教结果绕指定点及Z轴旋转180°
		void TeachResultRotate180(double dCenX, double dCenY);

		// 模板匹配得到相同工件的所有焊缝及附加属性信息
		bool GenerateTemplate();
		void TemplateMatch();

		//返回值:在同一坐标系下从原始位置到目标位置的坐标变换矩阵
		//BasePoints:原始位置四个对应点在坐标系下的坐标
		//TargetPoints:目标位置四个对应点在坐标系下的坐标
		/*若四个点共面,则矩阵BaseCoordinateMatrix不满秩,其逆矩阵不存在,在计算BaseCoordinateMatrix_1时将得到错误的结果*/
		cv::Mat CalculateTransformationMatrix(std::vector<CvPoint3D64f>& BasePoints, std::vector<CvPoint3D64f>& TargetPoints);
		//返回值:原始位置点变换到目标位置后的结果点在同一坐标系下的坐标
		//BasePoints:原始位置点坐标
		//TransformationMatrix:坐标变换矩阵
		std::vector<CvPoint3D64f> CoordinateTransformate(std::vector<CvPoint3D64f>& BasePoints, cv::Mat TransformationMatrix);
		//返回值:三维点绕指定旋转轴按右手螺旋法则逆时针旋转指定角度后的三维坐标
		//Point:三维点坐标
		//RotationCenter:旋转轴向量起点
		//RotationAxis:旋转轴向量方向
		//RotationAngle:旋转角(单位弧度)
		CvPoint3D64f RotatePoint3D(CvPoint3D64f Point, CvPoint3D64f& RotationCenter, CvPoint3D64f& RotationAxis, double RotationAngle);
		XI_POINT RotatePoint3D(XI_POINT Point, XI_POINT& RotationCenter, XI_POINT& RotationAxis, double RotationAngle);

		double calculateAngle(XI_POINT vector1, XI_POINT vector2);
		bool CalcLineLineIntersection(T_ROBOT_COORS tStart, double dStartAngle, T_ROBOT_COORS tEnd, double dEndAngle, XI_POINT& intersection);
		void GetChangePosturePare(vector<T_ROBOT_COORS> tLeftTrackCoors, XI_POINT tLeftCenter, double dLeftNormal, bool bLeftIsArc, vector<T_ROBOT_COORS> tRightTrackCoors, XI_POINT tRightCenter, double dRightNormal, bool bRightIsArc, bool bStartOrEnd,
			int& nStartStepNo, double& dStChangeAngle, double dChangePostureThresVal);
	public:
		// 是否进行本地调试
		bool m_bIsLocalDebug;
		// 传递处理图到界面
		IplImage** m_pColorImg;

		vector<int> m_vtImageNum;


		int m_ChangeDisPoint;

	/*protected*/public:
		CUnit* m_ptUnit;
		/*=====合隔板类添加======*/
		TraceModel* m_pTraceModel = new TraceModel();
		/*=====合隔板类添加======*/
		CRobotDriverAdaptor *m_pRobotDriver;
		CScanInitModule* m_pScanInit;
		int m_nCameraOpenNumber; // 记录当前相机打开的数量 最后一个相机关掉后需要CloseLib

		CString m_sPointCloudFileName; // 点云所在文件路径及文件名
		CString m_sDataSavePath; // 中间数据保存路径
		std::vector<LineOrCircularArcWeldingLine> m_vtWeldSeamData; // 点云处理得到的焊缝信息
		std::vector<WeldLineInfo> m_vtWeldSeamInfo; // 包括识别结果及焊脚等无法识别的属性信息
		std::vector<std::vector<LineOrCircularArcWeldingLine>> m_vvtWeldSeamGroup; // 分组后的焊缝信息
		std::vector<std::vector<WeldLineInfo>> m_vvtWeldLineInfoGroup; // 与m_vvtWeldSeamGroup一一对应的焊缝附加信息
		std::vector<std::vector<LineOrCircularArcWeldingLine>> m_vvtWeldSeamGroupAdjust; // 精确测量 调整后 焊缝分组信息
		std::vector<std::vector<LineOrCircularArcWeldingLine>> m_vvtCowWeldSeamGroupAdjust; // 精确测量 调整后 焊缝分组信息

		std::vector<std::vector<LineOrCircularArcWeldingLine>> m_vvtWeldSeamGroupOrg; // 原始焊缝数据 焊缝分组信息，黄埔超短立峰测试实验

		std::vector < std::vector<LineOrCircularArcWeldingLine>> m_vvtWeldSeamData; // 多机点云处理得到的焊缝信息
		std::vector < std::vector<WeldLineInfo>> m_vvtWeldSeamInfo; // 多机包括识别结果及焊脚等无法识别的属性信息

		double m_WeldSeamGroupAngle;// 加筋板绕y轴倾斜角度 正为前高后低

		// 点云处理参数
		bool m_bDeleteSeam;				// 是否删除翼板腹板焊缝
		double m_dZPallet;				// 平台上无工件位置的Z值, 用于过滤点云
		int m_nScanTrackingWeldEnable;
		double m_dSideBoardThick;		// H型钢侧板厚度		// 点云处理参数
		double m_dBoardLen;				// 行车梁筋板 或 檩托筋板长度(点云无筋板时使用)
		double m_dWeldHoleSize;			// 过焊孔大小
		double m_dEndBoardSupportLen;	// 端板支撑长度
		double m_dPurlinSupportLen;		// 檩托支撑长度
		double m_dPurlinLen;			// 檩托长度
		double m_dEndBoardLen;			// 端板长度
		int m_nExPartType;				// 工字钢外工件类型 1：檩托   2：柱脚  (工字钢焊接 0:小件接口 1:牛腿接口)
		double m_dScanEndpointOffset;	// 端点搜索结果补偿 正向外 负向里
		double m_dMaxWeldLen;			// 工字钢先测后焊焊缝最大长度
		double m_dJointLen;				// 工字钢先测后焊接头处长度
		int m_nWeldMode;				// 焊接模式：0直流1脉冲 (工字钢专用)

		// 跟踪参数
		double m_dStartDelLen; // 起点删除长度
		double m_dEndDelLen; // 终点删除长度

		// 焊接参数
		bool m_bWorking;					// 是否正在工作中 (私有成员 使用IsWorking函数访问)
		BOOL* m_pIsArcOn;					// 是否起弧
		BOOL* m_pIsNaturalPop;				// 是否开启提示弹窗 
		BOOL* m_pIsTeachPop;				// 是否开启示教弹窗
		BOOL* m_bNeedWrap;					// 是否包角
		int m_nRobotInstallDir;				// 机器人安装方向
		double m_dGunAngle;					// 焊枪角度
		double m_dGunLaserAngle;			// 焊枪和激光线夹角
		double m_dGunCameraAngle;			// 焊枪和相机光轴夹角
		double m_dRotateToCamRxDir;			// 机头转向跟踪相机Rx变化方向 增加1 减小-1
		double m_dHandEyeDis;				// 手眼距离：焊接前进方向距离
		double m_dTrackCamHandEyeDis;		// 跟踪相机手眼距离：焊接前进方向距离
		double m_dMeasureDisThreshold;		// 测量点距离焊缝起点终点距离阈值
		double m_dPlatWeldRx;				// 标准测量和焊接姿态Rx
		double m_dPlatWeldRy;				// 标准测量和焊接姿态Ry
		double m_dNormalWeldRx;				// 正常平角焊接姿态Rx
		double m_dNormalWeldRy;				// 正常平角焊接姿态Ry
		double m_dStandWeldRx;				// 标准立焊姿态Rx
		double m_dStandWeldRy;				// 标准立焊姿态Ry
        double m_dTransitionsRx;			// 过度点姿态Rx
        double m_dTransitionsRy;			// 过度点姿态Ry
		double m_dStandWeldScanRx;			// 立焊扫描姿态Rx
		double m_dStandWeldScanRy;			// 立焊扫描姿态Ry
		double m_dStandWeldScanOffsetRz;	// 标准立焊扫描姿态Rz偏移
		double m_dGrooveScanRx;				// 坡口扫描姿态Rx
		double m_dGrooveScanRy;				// 坡口扫描姿态Ry
		double m_dGrooveScanOffsetRz;		// 坡口扫描姿态Rz偏移
		double m_dGrooveWeldRx;				// 坡口焊接姿态Rx
		double m_dGrooveWeldRy;				// 坡口焊接姿态Ry
		double m_dGrooveWeldRz;				// 坡口焊接姿态Rz
		double m_dWeldNorAngleInHome;		// 机器人安全位置关节坐标状态 对应的焊缝法相角
		double m_dEndpointSearchDis;		// 自由端搜索焊缝长度(端点向焊缝内和外偏移距离)
		double m_dGunDownBackSafeDis;		// 收下枪安全距离
		double m_dLengthSeamThreshold;		// 超长平峰长度阈值(超长独立一组)
		double m_dShortSeamThreshold;		// 先测后焊重新计算测量位置的短边长度阈值
		double m_dPointSpacing;             // 修正平焊缝之间测量点间距离
		double m_dCleanGunDis;				// 焊接多长进行一次清枪剪丝 单位:米
		bool m_bFlatWeldContinue;			// 首尾相连平焊是否连续焊接（需要考虑过焊孔删除 干涉删除 不同工艺）
		bool m_bCheckCollide;				// 机头干涉检测使能
		bool m_bTrackingEnable;				// 是否启用跟踪焊接功能
		double m_dTrackingLengthThreshold;	// 超过多长焊缝使用跟踪焊接
		bool m_bAutoCalcWrapDirStand;		// 自动计算立焊包角方向
		T_ROBOT_COORS m_tRobotHomeCoors;	// 机器人安全位置关节坐标对应的直角坐标(焊枪工具)，构造函数初始化
		bool m_bIsSlope;					//爬坡立焊
		//UINT* m_pUnTracePointCloudProcess;	// 是否使用跟踪点云处理搜端点
		UINT* m_pUnWarpBoundWeld;			// 是否包角跳枪
		//BOOL* m_pIsSaveTraceScanPic;		// 是否存跟踪搜索原图
		//速度参数
		double m_dUltraTransitionSpeed;		//走焊接过渡点速度，最大速度
		double m_dSafePosRunSpeed;//安全位置移动速度，快速
		double m_dFastApproachToWorkSpeed;//快速靠近工件， 中速
		double m_dSafeApproachToWorkSpeed;//安全移动到工件速度，慢速
		double m_RobotBiasDis;			  // 机器人偏置距离，跟踪时使机器人X或Y轴固定在m_RobotBiasDis位置

		// 示教参数
		int m_nTeachPtnNum;					// 示教点数
		double m_dTeachExAxlePos;			// 示教时外部轴位置
		double m_dTeachExAxlePos_y;			// 示教时y方向外部轴位置
		double m_dPauseExAxlePos;
		double m_dPauseExAxlePos_y;
		vector<int> m_vnTeachIdxWithTemp;	// 所有测量点在包含过渡点轨迹中的索引
		vector<int> m_vnMeasurePtnType;		// 数量和示教点数相同 (不包括过渡点)
		vector<T_ANGLE_PULSE> m_vtTeachPulse;	// 示教位置关节坐标 数量和示教点数相同(不包括过渡点)
		vector<T_TEACH_RESULT> m_vtTeachResult;	// 示教测量结果：二维点 三维坐标 采图直角 采图关节 采图外部轴 

		// 焊接轨迹补偿
		map<int, double> m_mdFlatHorComp;	// 平焊水平方向补偿
		map<int, double> m_mdFlatHeightComp;	// 平焊高度补偿
		map<int, double> m_mdStandLenComp;  // 立焊干伸长度补偿
		map<int, double> m_mdStandVerComp;	// 立焊焊丝垂直方向补偿

		/********************* 通用先测候焊添加 *********************/
		
		vector<T_TEACH_DATA> m_vtTeachData; // 测量数据 测量运动数据 和 计算轨迹使用的相关参数
		vector<int> m_vnTeachTrackOrder; // 示教轨迹连续运动重新排序后 恢复顺序的索引（示教结果按这个顺序使用）
		E_WORKPIECE_TYPE m_eWorkPieceType; // 用于区分不同类型工件调用不同视觉接口

		// 测试参数
		bool m_bWorkpieceShape; // 工件形状
		double m_dOverflowHoleStart;// 过水孔
		double m_dOverflowHoleEnd;// 过水孔

		void   WeldSeamTransCoor_Gantry2Robot(std::vector<LineOrCircularArcWeldingLine>& vtWeldSeamData, std::vector<WeldLineInfo>& vtWeldSeamInfo);	//焊缝信息龙门坐标系转机器人坐标系
		// 自动分组，临时测试使用，不更改原有虚函数
		bool   WeldSeamGroupingAuto(int& nWeldGroupNum);
		// 分配焊缝焊接方式：跟踪/先测后焊
		void DetermineWeldingMode();
		// 模型匹配
		//bool ModelMatching(TransformMatrix *TransMat, std::string ModelFileName1, std::string ModelFileName2, std::string SaveRoute_weldline_point);
		
		// 根据结尾数据添加跳枪轨迹
		bool CalcJumpWarpTrack(int nWarpNum, double dBoardChick, double dGunToEyeCompen, double dFlatHorComp, std::vector<T_ROBOT_COORS> vtWeldLineTrack, double dWorkpieceHight,
			std::vector<T_ROBOT_COORS>& vtWarpCoors, std::vector<int>& vtWarpPtType);
		// 跳枪跳枪轨迹
		void GenerateWrapBoundTrack(T_ROBOT_COORS tWrapStart, T_ROBOT_COORS tWrapEnd, vector<T_ROBOT_COORS>& vtRobotCoor,
			vector<int>& vtPtType,double dWorkpieceHight ,double dInsertInDis = 8.0, double dInsertOutDis = 8.0, double dBackGunPercent = (double)(6.0), double dDirRotation = 0.0);
public:
	/********************* 碰撞检测添加 *********************/
	//// 初始化碰撞检测对象 并 设置机头模型
	//void InitCheckCollide(CString sModelFilePath);
	//// 设置碰撞检测平面
	//void SetCheckCollidePlane();
	//// 检测指定坐标是否与设置的平面干涉 tCoord:世界坐标 bOutputStep:是否输出检测结果Step文件 dCheckDis:检测距离
	//bool CheckIsCollide(T_ROBOT_COORS tCoord, bool bOutputStep = false, double dCheckDis = 1000.0);
	//// 检测指定坐标是否与设置的平面干涉 tPulse:世界坐标 bOutputStep:是否输出检测结果Step文件 dCheckDis:检测距离
	//bool CheckIsCollide(T_ANGLE_PULSE tPulse, bool bOutputStep = false, double dCheckDis = 1000.0);
	//// 检测指定坐标是否与设置的平面干涉 vtCoord:机器人坐标 dExAxis:外部轴坐标 bOutputStep:是否输出检测结果Step文件 dCheckDis:检测距离
	//bool CheckIsCollide(vector<T_ROBOT_COORS> vtCoord, double dExAxis, bool bOutputStep = false, double dCheckDis = 1000.0);
	//xi::RobotCollideClosedPlaneInterface* m_pCollide;
	//vector<T_RECT_POINT> m_vtPartPlaneRects;


public:
	/********************* 断点续焊添加 *********************/
	bool GetPauseWeldTrack(vector<T_ROBOT_COORS>& vtCoord, vector<int> &vnPtnType, E_WELD_SEAM_TYPE eSeamType);		 // 根据暂停位置截取继续焊接轨迹
	void SavePauseInfo(int nGroupNo, int nWeldNo, int nLayerNo); // 保存暂停状态信息
	void LoadPauseInfo();			// 加载暂停状态信息
	int m_nPauseGroupNo;			// 暂停时组号
	int m_nPauseWeldNo;				// 暂停时焊缝号
	int m_nPauseLayerNo;			// 暂停时焊接层号
	T_ANGLE_PULSE m_tPausePulse;	// 暂停时机器人六轴坐标

	//直线轨迹预测
	bool TrajectoryLineTrack(std::vector<T_ROBOT_COORS>& vtOutputTrack, T_LINE_PARA tRansacLine, T_ROBOT_COORS tRealStart, XI_POINT tDealEnd, int nPointCount, double dAdjustDis);
	std::vector<XI_POINT> m_vtTrackTheoryTrack;//跟踪理论轨迹

	//计算焊缝的扫描轨迹
	bool CalcScanTrack(LineOrCircularArcWeldingLine SeamData, std::vector<T_ROBOT_COORS>& vtMeasureCoord, vector<T_ANGLE_PULSE>& vtMeasurePulse, double& dExAxlePos);


	std::vector<T_ROBOT_COORS> CalLineDividePoints(T_ROBOT_COORS tStart, T_ROBOT_COORS tEnd, double dStep);
	//jwq临时，仅限不足半圆的圆弧
	std::vector<T_ROBOT_COORS> CalArcDividePoints(T_ROBOT_COORS tStart, T_ROBOT_COORS tEnd, T_ROBOT_COORS tCenter, double dStep);
	bool CalcArcMoveTrackBySeamData(int nRobotNo, LineOrCircularArcWeldingLine SeamData, std::vector<T_ROBOT_COORS>& vtMeasureCoord, vector<T_ANGLE_PULSE>& vtMeasurePulse, double dStep, int& nStartPointCount, int& nEndPointCount);

	bool CalcLineMoveTrackBySeamData(int nRobotNo, LineOrCircularArcWeldingLine SeamData, std::vector<T_ROBOT_COORS>& vtMeasureCoord, vector<T_ANGLE_PULSE>& vtMeasurePulse, double dStep);
	// 样条滤波
	bool SplineFiltering(double dExAxisPos, vector<XI_POINT> vtWeldTrack, vector<T_ROBOT_COORS>& vtWeldTrackRobot);
	// 生成真实轨迹
	bool CalcRealWeldTrack(WeldLineInfo tWeldSeam, vector<T_ROBOT_COORS>& vtWeldTrackRobot);
	// 判断组内包角形式
	int DetermineWarpMode(int nGroupNo);

	// 加载包角参数
	E_WRAPANGLE_PARAM LoadWarpParam(CString strRobotName, E_WRAPANGLE_TYPE eWarpType, int nWarpNo);
	// 根据结尾数据添加一次包角轨迹
	bool CalcOnceWarpTrack(int nWarpNum, double dBoardChick, double dGunToEyeCompen, double dFlatHorComp, std::vector<T_ROBOT_COORS> vtWeldLineTrack, double dWorkpieceHight,
		std::vector<T_ROBOT_COORS>& vtWarpCoors, std::vector<int>& vtWarpPtType, E_WRAPANGLE_PARAM tWarpParam);

	// 检查坐标法兰位置距离坐标原点距离是否大于阈值
	bool CheckFlangeToolCoord(const T_ROBOT_COORS& tRobotCoord, double dDisThreshold = 300.0);
	bool CheckFlangeToolCoord(const T_ANGLE_PULSE& tRobotPulse, double dDisThreshold = 300.0);
	bool CheckFlangeToolCoord(const vector<T_ROBOT_COORS>& vtRobotCoord, double dDisThreshold = 200.0);
	bool CheckFlangeToolCoord(const vector<T_ANGLE_PULSE>& vtRobotPulse, double dDisThreshold = 200.0);

	void WeldInfoToJson(Welding_Info tWeldInfo);

	void DelPngFile(const CString& directory);

	CvPoint3D64f* SaveRemoveCloud(CRobotDriverAdaptor* pRobotDriver, CString cFileName, vector<CvPoint3D64f>& vtPointCloud, int nPtnNum);

	void BackHome();

	//void LoadWeldInfoToJson(Welding_Info& tWeldInfo, int nWeldIndex, std::vector<T_ROBOT_COORS>& vtContour);

};
}
