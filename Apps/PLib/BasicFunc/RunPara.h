/*************************************************************************************************
 * 文件： RunPara.h
 * 说明： 运行参数，
 *		临时使用以解决旧代码魔法数太多的问题。
 *		不提倡用于常规参数。
 *		不提倡用于写入文件，因为没有考虑自动写入用宏定义的参数。
 * 作者： 江文奇
 * 日期： 2025-08-19
 * ToDo： 
 ************************************************************************************************/
#pragma once

//读取系统参数
#define PARA_SYSTEM(para) (RunPara::GetInstance().m_tSystem.para)

//读取埃斯顿参数
#define PARA_ESTUN(para) (RunPara::GetInstance().m_tEstun.para)

//读取焊缝参数
#define PARA_WELD_SEAM(para) (RunPara::GetInstance().m_tWeldSeam.para)

//读取识别参数
#define PARA_RECOGNITION(para) (RunPara::GetInstance().m_tRecognition.para)

//读取平缝测量参数
#define PARA_FLAT_MEASURE(para) (RunPara::GetInstance().m_tFlatMeasure.para)

//读取平缝跟踪参数
#define PARA_FLAT_TRACK(para) (RunPara::GetInstance().m_tFlatTrack.para)

//读取平缝焊接参数
#define PARA_FLAT_WELD(para) (RunPara::GetInstance().m_tFlatWeld.para)

//读取立缝测量参数
#define PARA_VER_MEASURE(para) (RunPara::GetInstance().m_tVerMeasure.para)

//读取立缝焊接参数
#define PARA_VER_WELD(para) (RunPara::GetInstance().m_tVerWeld.para)

//具体参数的宏定义
#pragma region
//定义系统参数
#define MACRO_SYSTEM_RUN_PARA\
    X_MACRO(int, nRobotInstallDir)\
    X_MACRO(int, nRobotCheckDoneCount)\
    X_MACRO(int, nRobotCheckDoneTime)\
    X_MACRO(double, dSafeHeightBetweenCurPosToHomePos)\
    X_MACRO(double, dMinRisingHeightForBackHome)\
    X_MACRO(double, dUpOrDownGunSafeHeight)\
    X_MACRO(double, dGunLaserAngle)\
    X_MACRO(double, dGunCameraAngle)\
    X_MACRO(double, dRotateToCamRxDir)\
    X_MACRO(double, dMaxWeldLengthForCleanGun)\
    X_MACRO(bool, bScanTrackingWeldEnable)\
    X_MACRO(int, nWeldDirection)\
    X_MACRO(bool, bCheckWeldIO)\

//定义埃斯顿参数
#define MACRO_ESTUN_RUN_PARA\
    X_MACRO(int, nRobotEachSendPosVarMaxCount)\
    X_MACRO(int, nFlatWeldMode)\
    X_MACRO(CString, sFlatNormalWelderMode_W)\
    X_MACRO(CString, sFlatNormalWelderMode_MM)\
    X_MACRO(CString, sFlatPulseWelderMode_W)\
    X_MACRO(CString, sFlatPulseWelderMode_MM)\
    X_MACRO(CString, sFlatDualPulseWelderMode_W)\
    X_MACRO(CString, sFlatDualPulseWelderMode_MM)\
    X_MACRO(int, nVerWeldMode)\
    X_MACRO(CString, sVerNormalWelderMode_W)\
    X_MACRO(CString, sVerNormalWelderMode_MM)\
    X_MACRO(CString, sVerPulseWelderMode_W)\
    X_MACRO(CString, sVerPulseWelderMode_MM)\
    X_MACRO(CString, sVerDualPulseWelderMode_W)\
    X_MACRO(CString, sVerDualPulseWelderMode_MM)\

//定义焊缝参数
#define MACRO_WELD_SEAM_RUN_PARA\

//定义识别参数
#define MACRO_RECOGNITION_RUN_PARA\
    X_MACRO(CString, sLineScanFolder)\
    X_MACRO(int, nMaxLineScanTimes)\
    X_MACRO(int, nLineScanTimes)\
    X_MACRO(double, dBackgroundCloudRemoveRadius)\
    X_MACRO(double, dBackgroundCloudRemoveOffset)\
    X_MACRO(double, dLengthSeamThreshold)\

//定义平缝测量参数
#define MACRO_FLAT_MEASURE_RUN_PARA\
    X_MACRO(double, dSafeHeight)\
    X_MACRO(double, dShortSeamThreshold)\
    X_MACRO(double, dMeasurePosRX)\
    X_MACRO(double, dMeasurePosRY)\
    X_MACRO(double, dMinAngleBetweenTwoVerticalPlates)\
    X_MACRO(double, dMaxAngleBetweenTwoVerticalPlatesForChangeRZ)\
    X_MACRO(double, dPointSpacingForLongWeldSeam)\
    X_MACRO(double, dMeasureDisThreshold)\
    X_MACRO(int, nTriggerMode)\
    X_MACRO(bool, bImageDirection)\
    X_MACRO(bool, bCrossFilp)\
    X_MACRO(double, dExAxisPosMoveSpeed)\
    X_MACRO(double, dEndpointSearchDis)\
    X_MACRO(double, dEndpointSearchDis2)\
    X_MACRO(double, dUnfreeEndpointSearchDisNotOnWeldSeam)\
    X_MACRO(double, dStartPointInterfereSearchOffsetRZ)\
    X_MACRO(double, dEndPointInterfereSearchOffsetRZ)\
    X_MACRO(double, dPointCloudLengthForFindEndpnt)\
    X_MACRO(int, nGetLaserPointMaxErrorTimes)\
    X_MACRO(double, dLineScanSpeed)\
    X_MACRO(double, dRecoTeachDisErrThreshold)\

//定义平缝跟踪参数
#define MACRO_FLAT_TRACK_RUN_PARA\
    X_MACRO(bool, bTrackingEnable)\
    X_MACRO(double, dTrackingLenThreshold)\
    X_MACRO(double, dTrackCamHandEyeDis)\
	X_MACRO(int, nMaxImgProFailNum)\
    X_MACRO(int, nMaxAdjustZErrNum)\
    X_MACRO(int, nMaxFilterFailNum)\
    X_MACRO(int, nMaxErrStopNum)\
    X_MACRO(double, dMinJoinPtnDis)\
    X_MACRO(double, dSameDisThreshold)\
    X_MACRO(int, nSearchEndPntNum)\
    X_MACRO(int, nSameEndPtnNumThreshold)\
    X_MACRO(int, nMinProcessImageNum)\
    X_MACRO(int, nMaxProcessPointCloudErrorTimes)\
    X_MACRO(double, dChangeDisOfStartPointRZ)\
    X_MACRO(double, dChangeAngleOfStartPointRZ)\
    X_MACRO(double, dChangeDisOfEndPointRZ)\
    X_MACRO(double, dChangeAngleOfEndPointRZ)\
    X_MACRO(double, dDisBetweenFirstChangeRZPosToIdealEndPnt)\
    X_MACRO(double, dDisBetweenLastChangeRZPosToIdealEndPnt)\
    X_MACRO(double, dChangeDisOfEndPointRZBeforeFindEndPoint)\
    X_MACRO(double, dChangeAngleOfEndPointRZBeforeFindEndPoint)\
    X_MACRO(double, dChangeAngleOfEndPointRZAfterFindEndPoint)\
    X_MACRO(int, nChangePntCountOfEndPointRZAfterFindEndPoint)\

//定义平缝焊接参数
#define MACRO_FLAT_WELD_RUN_PARA\
    X_MACRO(bool, bFlatWeldContinue)\
    X_MACRO(double, dWeldPosRX)\
    X_MACRO(double, dWeldPosRY)\
    X_MACRO(double, dChangeDisOfStartPointRZ)\
    X_MACRO(double, dChangeAngleOfStartPointRZ)\
    X_MACRO(double, dChangeDisOfEndPointRZ)\
    X_MACRO(double, dChangeAngleOfEndPointRZ)\
    X_MACRO(double, dWrapOffsetRZ)\
    X_MACRO(double, dLiftGunSpeed)\
    X_MACRO(double, dLiftGunHorizontalDistance)\
    X_MACRO(double, dLiftGunVerticalDistance)\

//定义立缝测量参数
#define MACRO_VER_MEASURE_RUN_PARA\
    X_MACRO(double, dLineScanPosRX)\
    X_MACRO(double, dLineScanPosRY)\
    X_MACRO(double, dLineScanHorizontalOffset)\
    X_MACRO(double, dLineScanVerticalOffset)\
    X_MACRO(double, dLineScanOffsetRZForFirstTrack)\
    X_MACRO(double, dLineScanOffsetRZForSecondTrack)\
    X_MACRO(double, dLineScanSpeed)\
    X_MACRO(double, dLineScanSpeedHeight)\

//定义立缝焊接参数
#define MACRO_VER_WELD_RUN_PARA\
    X_MACRO(double, dWeldPosRX)\
    X_MACRO(double, dWeldPosRY)\
    X_MACRO(bool, bAutoCalcWrapDirStand)\
    X_MACRO(double, dMaxWrapLength)\
    X_MACRO(double, dStandWeldWrapLenRatio)\


#pragma endregion

class RunPara
{
//具体参数的结构体定义
#pragma region
	struct System
	{
#define X_MACRO(type, var) type var;
		MACRO_SYSTEM_RUN_PARA
#undef X_MACRO
	};

	struct Estun
	{
#define X_MACRO(type, var) type var;
		MACRO_ESTUN_RUN_PARA
#undef X_MACRO
	};

	struct WeldSeam
	{
#define X_MACRO(type, var) type var;
		MACRO_WELD_SEAM_RUN_PARA
#undef X_MACRO
	};

	struct Recognition
	{
#define X_MACRO(type, var) type var;
		MACRO_RECOGNITION_RUN_PARA
#undef X_MACRO
	};

	struct FlatMeasure
	{
#define X_MACRO(type, var) type var;
		MACRO_FLAT_MEASURE_RUN_PARA
#undef X_MACRO
	};

	struct FlatTrack
	{
#define X_MACRO(type, var) type var;
		MACRO_FLAT_TRACK_RUN_PARA
#undef X_MACRO
	};

	struct FlatWeld
	{
#define X_MACRO(type, var) type var;
		MACRO_FLAT_WELD_RUN_PARA
#undef X_MACRO
	};

	struct VerMeasure
	{
#define X_MACRO(type, var) type var;
		MACRO_VER_MEASURE_RUN_PARA
#undef X_MACRO
	};

	struct VerWeld
	{
#define X_MACRO(type, var) type var;
		MACRO_VER_WELD_RUN_PARA
#undef X_MACRO
	};
#pragma endregion

private:
	RunPara();
	~RunPara();
	RunPara(const RunPara&) = delete;
	RunPara& operator=(const RunPara&) = delete;

	/// @brief 读取参数文件
	/// @return 成功返回true，失败返回false
	bool loadPara(bool bCheck, CString sFileName);

	static RunPara m_instance;
    CString m_sCustomParaFileName;//定制参数文件名

public:
	static RunPara& GetInstance() { return m_instance; }

    /// @brief 设置定制参数文件名
    /// @param sFileName 定制参数文件名
    void setCustomParaFileName(CString sFileName) { m_sCustomParaFileName = sFileName; }

	/// @brief 读取所有参数文件
	/// @return 成功返回true，失败返回false
	bool loadAllPara();

public:
	System m_tSystem;
	Estun m_tEstun;
	WeldSeam m_tWeldSeam;
	Recognition m_tRecognition;
	FlatMeasure m_tFlatMeasure;
	FlatTrack m_tFlatTrack;
	FlatWeld m_tFlatWeld;
	VerMeasure m_tVerMeasure;
	VerWeld m_tVerWeld;
};

//一些全局函数用于读写参数
#pragma region

/// @brief 设置当前使用的线扫文件夹
/// @param nTableNo 工作台号
/// @return 成功返回true，失败返回false
bool SetCurLineScanFolder(int nTableNo);

/// @brief 获取当前使用的线扫点云文件路径
/// @return 返回当前使用的线扫点云文件路径
CString GetCurLineScanPointCloudFile();

/// @brief 设置线扫次数
/// @param nLineScanTimes 线扫次数
/// @return 成功返回true，失败返回false
bool SetLineScanTimes(int nLineScanTimes);

#pragma endregion

