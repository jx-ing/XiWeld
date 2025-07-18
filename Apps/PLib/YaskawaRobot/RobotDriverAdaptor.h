// RobotDriverAdaptor.h: interface for the CRobotDriverAdaptor class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_ROBOTDRIVERADAPTOR_H__59FD46BF_CA09_4E0E_A5E0_C6B55F565624__INCLUDED_)
#define AFX_ROBOTDRIVERADAPTOR_H__59FD46BF_CA09_4E0E_A5E0_C6B55F565624__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "XiRobotCtrl.h"
#include ".\Apps\PLib\CtrlUnit\CUnitDriver.h"
#include ".\OpenClass\FileOP\ini\opini.h"
#include ".\Apps\PLib\BasicFunc\Const.h"
#include "AbsCoorTransLib.h"

#define MAXMOVENUM 10
#define MAXPOSVARNO 23
#define MAXPOSVARNO_H 25


#include "Apps/PLib/CrpRobot/ROBOTPCtr.h"


//静态变量
static bool m_bInsterPVal = false;



typedef enum
{
	ROBOT_STATE_IDLE = 0,		//空闲
	ROBOT_STATE_RUNNING = 1,	//运行中
	ROBOT_STATE_ERROR = 2,		//异常
}E_ROBOT_STATE;

typedef enum 
{
	ROBOT_BRAND_YASKAWA = 0x01,			//安川
	ROBOT_BRAND_ESTUN		= 0x02,		//埃斯顿
	ROBOT_BRAND_CRP			= 0x03			//卡诺普
}E_ROBOT_BRAND;

struct T_ROBOT_MOVE_INFO
{
	MP_USR_VAR_INFO tCoord;	//坐标
	MP_USR_VAR_INFO tPulse;	//坐标
	int nMoveType = MOVJ;	//移动方式
	T_ROBOT_MOVE_SPEED tSpeed;	//移动速度
	int nMoveDevice = -1;
	int nTrackNo = -1;
	double adBasePosVar[3]; //外部轴坐标(MP_USR_VAR_INFO中最多支持8轴)
};

typedef struct
{
	T_ROBOT_COORS tGunTool;
	T_ROBOT_COORS tMagnetTool;
	T_ROBOT_COORS tPolisherTool;
	T_ROBOT_COORS tCameraTool;
}T_ROBOT_TOOLS;

class CRobotMove
{
public:
	void Clear();
	bool InsertCoor(T_ANGLE_PULSE tPulse, T_ROBOT_MOVE_SPEED tSpeed, int nExternalAxleType, int nMoveType = MOVJ, UINT unToolNum = 1, UINT unUserNo = 0, UINT unPosture = 4);
	bool InsertCoor(T_ROBOT_COORS tCoord, T_ROBOT_MOVE_SPEED tSpeed, int nExternalAxleType, int nMoveType = MOVJ, UINT unToolNum = 1, UINT unUserNo = 0, UINT unPosture = 4);
	bool DeleteTrack(int nNum);//删除轨迹前 nNum 个点
	std::vector< T_ROBOT_MOVE_INFO> GetTrack();
	int GetTrackNum();
private:
	std::vector< T_ROBOT_MOVE_INFO> m_vRobotMoveInfo;
	int m_nTrackNum = 0;
};

class CRobotDriverAdaptor  
{
public:
	CRobotDriverAdaptor(CString strUnitName, CLog *cLog, std::vector<CUnitDriver*>* ppUnitDriver = NULL);
	virtual ~CRobotDriverAdaptor();
	BOOL InitRobotDriver(CString strUnitName, CLog *cLog);

	void SetKinematicsParam(T_KINEMATICS tKinematics, T_AXISUNIT tAxisUnit, T_AXISLIMITANGLE tAxisLimitAngle);
	//（问题：该函数的正确执行依赖路径 .\Data\SpeedAndSafeHeight.ini  新框架无此路径  该函数已完成定义）
	void LoadSpeedAndSafeHeight(); // 加载速度参数

	//（问题：未完成CheckInIO()定义：该函数依赖CheckInIO()  该函数已完成定义：注释依赖项）
	bool ContiMoveByJob(std::vector<T_ANGLE_PULSE> vtPulseMove, T_ROBOT_MOVE_SPEED tPulseMove, bool bPulseOrder, CString strJobName = "CONTIMOVE");
	void SetIntValFun(int nIntVal, int nVal);
	

	/****************************************************基本控制****************************************************/
	//设置姿态
	void ConfigRobotPosture(unsigned int nRobotPosture);
	//伺服上电
	void ServoOn();	
	//清除报警信息+
	void CleanAlarm();
	//伺服掉电
	void ServoOff();
	//机器人急停
	//void HoldOn(); 	// 改为虚函数接口
	//恢复暂停中的机器人	
	void HoldOff();	
	//调用手操盒程序
	//void CallJob(char JobName[24]);	// 改为虚函数接口
	//void CallJob(CString sJobName);	// 改为虚函数接口
	//（问题：老框架无此重载）
	void CallJob(CString sJobName, int nExternalType);
	//设置当前工具
	void SetRobotToolNo(int nToolNo);
	//2023/12/21 焊接加：注释
	CXiRobotCtrl *GetXiRobotCtrl();

	/****************************************************信息获取****************************************************/
	//查询机器人当前直角坐标
	//T_ROBOT_COORS GetCurrentPos();	// 改为虚函数接口
	//double GetCurrentPos(int nAxisNo);  	// 改为虚函数接口
	//查询机器人当前关节坐标
	//T_ANGLE_PULSE GetCurrentPulse();	// 改为虚函数接口
	//long GetCurrentPulse(int nAxisNo);	// 改为虚函数接口
	//查询机器人当前各轴角度(暂无外部轴)
	double GetCurrentDegree(int nAxisNo);
	//关节坐标转直角坐标
	int ConvAxesToCartPos(long alPulse[6], UINT * unFig_ctrl, double adCoord[6], int nGrpNo = 0, UINT unTool_no = 1);
	//直角坐标转关节坐标														
	int ConvCartToAxesPos(double adCoord[6], long alPulse[6], UINT unFig_ctrl = 4, int nGrpNo = 0, UINT unTool_no = 1, UINT unKinema_type = 0);
	//获取工具信息															
	BOOL GetToolData(UINT unToolNo, double adRobotToolData[6], UINT32 unTimeout = 2000);
	//获取报警码							
	BOOL GetAlarmCode(int &nErrorNo, int &ErrorData, int &AlarmNum, MP_ALARM_DATA *pData, UINT32 unTimeout = 2000);	
	//获取报警信息
	CString GetWarningMessage();

	/***********************************************目标位置核对***********************************************/
	//（问题：使用宏ROBOT_PAUSE_INI（目录：./Data/_RobotName_/RobotPause.ini）该函数已完成定义 新框架无此目录结构 老框架无此定义）
	bool SetAimPulse(T_ANGLE_PULSE tAimPulse);
	//（问题：使用宏ROBOT_PAUSE_INI（目录：./Data/_RobotName_/RobotPause.ini）该函数已完成定义 新框架无此目录结构 老框架无此定义）
	bool SetAimCoord(T_ROBOT_COORS tAimCoord);
	//（问题：使用宏ROBOT_PAUSE_INI（目录：./Data/_RobotName_/RobotPause.ini）该函数已完成定义 新框架无此目录结构 老框架无此定义）
	bool CheckIsAimPulse(T_ANGLE_PULSE tCompareLimit);
	//（问题：使用宏ROBOT_PAUSE_INI（目录：./Data/_RobotName_/RobotPause.ini）该函数已完成定义 新框架无此目录结构 老框架无此定义）
	bool CheckIsAimCoord(T_ROBOT_COORS tCompareLimit);

	bool CompareCurCoord(T_ROBOT_COORS tCoord, T_ROBOT_COORS tCompareLimit);
	bool CompareCurPulse(T_ANGLE_PULSE tPulse, T_ANGLE_PULSE tCompareLimit);
	bool CompareCoord(T_ROBOT_COORS tCoord1, T_ROBOT_COORS tCoord2, T_ROBOT_COORS tCompareLimit);
	bool ComparePulse(T_ANGLE_PULSE tPulse1, T_ANGLE_PULSE tPulse2, T_ANGLE_PULSE tCompareLimit);
	bool CompareCoords(T_ROBOT_COORS tCoords1, T_ROBOT_COORS tCoords2, double dCoordsLimit = 5.0, int nCheckType = 3, double dAngleLimit = 4.0);									//坐标比对函数nCheckType:1检查XYZ，2检查RXRYRZ，3都检查
	bool CompareCoords(T_ROBOT_COORS tCoords1, double dCoordsLimit = 5.0, int nCheckType = 3, double dAngleLimit = 4.0);									//坐标比对函数nCheckType:1检查XYZ，2检查RXRYRZ，3都检查
	bool CompareXY(T_ROBOT_COORS tCoords1, T_ROBOT_COORS tCoords2, double dCoordsLimit = 20.0);													//坐标比对函数
	bool ComparePulse(T_ANGLE_PULSE tPulse1, T_ANGLE_PULSE tPulse2, long lLimit = 500);
	bool ComparePulse(T_ANGLE_PULSE tPulse1, long lLimit = 500);

	/****************************************************变量读写****************************************************/
	//fast write
	BOOL SetMultiVar_H(UINT unCount, MP_USR_VAR_INFO* pVarInfo);
	//（2023/12/21安川独占）
	void SetMultiVar_H(const std::vector<MP_USR_VAR_INFO> vtVarInfo);
	//（2023/12/21安川独占）
	void SetMultiVar_H_WriteSyncIo(int nSyncId, int nSyncIoNo, UINT unCount, MP_USR_VAR_INFO* pVarInfo);

	//一次性设置大量BP变量（2023/12/21安川独占）
	void SetMultiBasePosVar(UINT unCount, UINT unIndex, long lRobotPulse[][3], int nExternalAxleType);
	//（2023/12/21安川独占）
	void SetMultiBasePosVar(UINT unCount, UINT unIndex, double dPosCoord[][3], int nExternalAxleType, int nPVarType = MP_BASE_COORD);
	//（问题：该函数已完成定义 老框架无此重载	）（2023/12/21安川独占）
	void SetMultiBasePosVar(UINT unIndex, std::vector<T_ANGLE_PULSE> vtRobotPulse, int nExternalAxleType);
	//一次性设置大量P变量 （2023/12/21安川独占）
	void SetMultiPosVar(UINT unIndex, std::vector<T_ANGLE_PULSE> vtRobotPulse, UINT unToolNum = 1, UINT unUserNo = 1, UINT unPosture = 4);
	//一次性设置大量P变量 （2023/12/21安川独占）
	void SetMultiPosVar(UINT unCount, UINT unIndex, long lRobotPulse[][6], UINT unToolNum = 1, UINT unUserNo = 1, UINT unPosture = 4);
	//一次性设置大量P变量 （2023/12/21安川独占）
	void SetMultiPosVar(UINT unCount, UINT unIndex, double adPosCoord[][6], UINT unToolNum = 1, UINT unUserNo = 1, UINT unPosture = 4);
	//一次性设置大量P变量 （2023/12/21安川独占）
	void SetMultiPosVar(UINT unCount, T_ROBOT_POSVAR_DATA* pPosVarData);
	//埃斯顿机器人一次设置大量变量 直角 //接口不稳定，用下面那个 （问题：新框架无此重载 老框架未正确定义）
	bool SetMultiPosVar(UINT unIndex, std::vector<T_ROBOT_COORS> vtRobotJointCoord, T_ROBOT_MOVE_SPEED tPosMove, int config[7] = int(0), UINT unToolNum = 1);
	//（问题：该函数已完成定义 新框架无此重载 老框架未正确定义）
	bool SetMultiPosVar(UINT unIndex, std::vector<T_ROBOT_COORS> vtRobotJointCoord, T_ROBOT_MOVE_SPEED tPosMove, CString Program_name, int config[7] = int(0), UINT unToolNum = 1);

	//一次性获取大量P变量
	void GetMultiPosVar(UINT unCount, UINT unIndex, double adRobotPos[][6], UINT *pToolNo = NULL, UINT *pUserNo = NULL, UINT *pPosture = NULL, UINT32 unTimeout = 2000);
	void GetMultiPosVar(UINT unCount, MP_VAR_INFO *mpVarInfo, LONG* pPosVarData, UINT32 unTimeout = 2000);
	//设置单个P变量 ++-
	void SetPosVar(int nIndex, double adPosVar[6], int nPVarType);
	void SetPosVar(int nIndex, long alPosVar[6], int nPVarType);
	void SetPosVar(int nIndex, T_ROBOT_COORS tRobotCoors);
	void SetPosVar(int nIndex, T_ANGLE_PULSE tRobotPulse);
	//（问题：该函数已完成定义 老框架无此重载	）（2023/12/21安川独占）
	void SetPosVar(int nIndex, long lPosVar[6]);
	
	//获取一个指定I变量
	int GetIntVar(int nIndex);
	//设置一个指定I变量 
	void SetIntVar(int nIndex, int nValue);
	//设置I变量（高速接口）
	void SetIntVar_H(int nIndex, int nValue);

	//设置大量变量（I,B,D,R）
	void SetMultiVar(UINT unCount, unsigned unIndex, LONG lRobotValue[], UINT unType = MP_RESTYPE_VAR_I);
	void SetMultiVar(UINT unCount, MP_VAR_DATA *pVarData);
	
	//（问题：该函数已完成定义 老框架无此定义	）设置BP变量（2023/12/21安川独占）
	void SetBasePosVar(int nIndex, double adBasePosVar[3], int nCoordType);
	//（问题：该函数已完成定义 老框架无此定义	）获取BP变量（2023/12/21安川独占）
	int GetBasePosVar(long lPvarIndex, double *array, UINT32 unTimeout = 2000);
	//（问题：该函数已完成定义 老框架无此定义	）获取P变量（2023/12/21安川独占）
	int GetPosVar(long lPvarIndex, double array[6]);

	/****************************************************运动函数****************************************************/
	//直角坐标系单轴运动函数(暂无外部轴)
	virtual void PosMove(int nAxisNo, double dDist, long lRobotSpd, WORD wCoorType = COORD_REL, int nToolNo = 1, long lCoordFrm = 0);
	virtual void PosMove(double adRobotCoord[10], long lRobotSpd, WORD wCoorType = COORD_REL, int nToolNo = 1, long lCoordFrm = 0);
	//关节坐标系单轴运动函数(暂无外部轴)
	int AxisPulseMove(int nAxisNo, long lDist, long lRobotSpd, WORD wCoorType = COORD_REL, int nToolNo = 1);
	void MoveToAbsPluse(long alAbsPulse[6], long lSpeedRatio = 500);
	//通用移动函数
	void MoveByJob(double *dRobotJointCoord, T_ROBOT_MOVE_SPEED tPulseMove, int nExternalAxleType, int nPVarType = PULSE_COORD, CString JobName = "MOVJ");
	//直角坐标运动函数
	void MoveByJob(T_ROBOT_COORS tRobotJointCoord, T_ROBOT_MOVE_SPEED tPulseMove, int nExternalAxleType, CString JobName = "MOVJ");
	//关节坐标运动函数
	virtual void MoveByJob(T_ANGLE_PULSE tRobotJointCoord, T_ROBOT_MOVE_SPEED tPulseMove, int nExternalAxleType, CString JobName = "MOVJ");
	void MoveByJob(long *alRobotJointCoord, T_ROBOT_MOVE_SPEED tPulseMove, int nExternalAxleType, CString JobName = "MOVJ");
	// 任意点运动 (收下枪连续多点运动) （2023/12/21卡诺普独占）
	bool DropGun(std::vector<double*> pos, std::vector<T_ROBOT_MOVE_SPEED> myspeed, std::vector<int> movType);
	virtual void MoveByJob(double* dRobotJointCoord, T_ROBOT_MOVE_SPEED tPulseMove, int nExternalAxleType, int nPVarType, CString JobName, int config[7]);
	virtual void MoveByJob(T_ROBOT_COORS tRobotJointCoord, T_ROBOT_MOVE_SPEED tPulseMove, int nExternalAxleType, CString JobName, int config[7]);////默认 mode值Cf值为零	

	void ThroughTranPointInfo(T_ANGLE_PULSE& tStartPulse, T_ANGLE_PULSE& tEndPulse, vector<T_ROBOT_MOVE_INFO>& tAllPulse);
	bool TransPulse(T_ROBOT_MOVE_INFO& tPoint, T_ANGLE_PULSE &tPulse);
	bool TheAnswer(vector<T_ROBOT_MOVE_INFO>& vtRobotMoveInfo);

	/**************************************************特殊移动函数**************************************************/
	virtual T_ROBOT_MOVE_INFO PVarToRobotMoveInfo(int nVarNo, T_ANGLE_PULSE tPulse, T_ROBOT_MOVE_SPEED tSpeed, int nMoveType = MOVJ, UINT unToolNum = 1, UINT unUserNo = 0, UINT unPosture = 4);
	virtual T_ROBOT_MOVE_INFO PVarToRobotMoveInfo(int nVarNo, T_ROBOT_COORS tCoord, T_ROBOT_MOVE_SPEED tSpeed, int nMoveType = MOVJ, UINT unToolNum = 1, UINT unUserNo = 0, UINT unPosture = 4);
	//（2023/12/21安川独占）
	// 批量混发数据，bSafeMove：默认使用安全过度点
	virtual bool SetMoveValue(std::vector<T_ROBOT_MOVE_INFO> vtMoveInfo,bool bSafeMove = true, bool bUsePB = false);
	virtual bool SetMoveValue(CRobotMove cMoveInfo);
	//（2023/12/21安川独占）
	int GetMoveStep();
	//（2023/12/21安川独占）
	int CleanMoveStep();
	// 安川外部轴坐标读取写入格式转换
	void RobotTransCoordToBase(double *adCoord, double *adBasePosVar);
	void RobotTransBaseToCoord(double *adBasePosVar, double *adCoord);

	/**************************************************硬触发专用函数**************************************************/
	//（安川独占）
	bool ReadSyncCoord(T_ANGLE_PULSE& tPulse, int nSyncId);

	/***********************************************状态信息(不可更改)***********************************************/
	int m_nExternalAxleType;							//外部轴类型
	T_EXTERNAL_AXLE_INFO m_tExternalAxle[3];			//外部轴信息

	int m_nRobotType;									//关节臂类型（按工作种类划分）
	E_ROBOT_BRAND m_eRobotBrand;						//机器人品牌

	E_MANIPULATOR_TYPE m_eManipulatorType;				//关节臂类型（按出厂型号划分）
	int m_nRobotNo;										//关节臂编号
	CString m_strRobotName;								//关节臂名称（参数调取，程序内部用）
	CString m_strCustomName;							//关节臂名称（显示用）
	T_ROBOT_TOOLS m_tTools;								//关节臂所用工具集合
	T_ROBOT_COORS m_tFirstTool;							//关节臂一号工具
	T_ANGLE_PULSE m_tHomePulse;							//关节臂非运行状态时的安全位置

	T_KINEMATICS m_tKinematics;
	T_AXISUNIT m_tAxisUnit;
	T_AXISLIMITANGLE m_tAxisLimitAngle;
	T_ROBOT_LIMITATION m_tRobotLimitation;				//机器人限制范围（24/01/23合隔板类添加）

	std::vector<T_ANGLE_PULSE> m_vtSafePulse;			//焊接避障过渡点

	double m_dWeldNorAngleInHome;
	vector<CUnitDriver*> *m_pvpMotorDriver;
	CXiRobotCtrl* m_pYaskawaRobotCtrl;					//实例化一个安川机器人
	CROBOTPCtr* m_pCrpRobotCtrl;						//实例化一个卡诺普机器人

	T_ROBOT_COORS m_tRobotHomeCoors;
	GROUP_ROBOT_ABS_COORS_TRANS::XiRobotAbsCoorsTrans* m_cXiRobotAbsCoorsTrans;

	// 机器人速度参数
	T_ROBOT_MOVE_SPEED		m_tBackHomeSpeed;
	T_ROBOT_MOVE_SPEED		m_tTeachSpeed;
	T_ROBOT_MOVE_SPEED		m_tExAxlePulseSpeed;
	T_ROBOT_MOVE_SPEED		m_tPulseHighSpeed;
	T_ROBOT_MOVE_SPEED		m_tPulseLowSpeed;
	T_ROBOT_MOVE_SPEED		m_tCoordHighSpeed;
	T_ROBOT_MOVE_SPEED		m_tCoordLowSpeed;

	//（问题：从老框架移植的变量 以下成员变量均	已使用）
	int	  m_nRobotInstallDir;
	bool		m_bIsOpenTrack;
	std::vector<T_ROBOT_COORS> m_vtWeldLineInWorldPoints;	//焊接数据

	//（2023/12/21安川独占）
	bool GetManipulatorType(CString strManipulatorType, E_MANIPULATOR_TYPE &eManipulatorType);
	//（2023/12/21安川独占）
	E_ROBOT_MODEL GetManipulatorType(E_MANIPULATOR_TYPE eManipulatorType);
	//（问题：该函数已完成定义 老框架无此定义	）
	bool IsCuttingRobot();
	//（问题：该函数已完成定义 老框架无此定义	）
	bool IsHandlingRobot();
	//（问题：该函数已完成定义 老框架无此定义	）
	bool IsPolishingRobot();

	/***********************************************状态信息(可更改)***********************************************/
	E_INCISEHEAD_THREAD_STATUS m_eThreadStatus;				//主流程下关节臂所属子线程的当前状态
	E_INCISEHEAD_THREAD_STATUS m_eLastThreadStatus;			//主流程下关节臂所属子线程的之前状态（用于继续）
	E_ROBOT_STATE m_eRobotState;							//关节臂当前运动状态（用于主流程子线程）

	CLog *m_cLog;

	/***********************************************暂停继续读写***********************************************/
	//（问题：使用宏ROBOT_PAUSE_INI（目录：./Data/_RobotName_/RobotPause.ini）该函数已完成定义 新框架无此目录结构 老框架无此定义）
	bool SavePausePara(CString sKey, int nValue);
	//（问题：使用宏ROBOT_PAUSE_INI（目录：./Data/_RobotName_/RobotPause.ini）该函数已完成定义 新框架无此目录结构 老框架无此定义）
	bool SavePausePara(CString sKey, bool bValue);
	//（问题：使用宏ROBOT_PAUSE_INI（目录：./Data/_RobotName_/RobotPause.ini）该函数已完成定义 新框架无此目录结构 老框架无此定义）
	bool SavePausePara(CString sKey, double dValue);
	//（问题：使用宏ROBOT_PAUSE_INI（目录：./Data/_RobotName_/RobotPause.ini）该函数已完成定义 新框架无此目录结构 老框架无此定义）
	bool SavePausePara(CString sKey, CString sValue);
	//（问题：使用宏ROBOT_PAUSE_INI（目录：./Data/_RobotName_/RobotPause.ini）该函数已完成定义 新框架无此目录结构 老框架无此定义）
	bool LoadPausePara(CString sKey, int &nValue);
	//（问题：使用宏ROBOT_PAUSE_INI（目录：./Data/_RobotName_/RobotPause.ini）该函数已完成定义 新框架无此目录结构 老框架无此定义）
	bool LoadPausePara(CString sKey, bool &bValue);
	//（问题：使用宏ROBOT_PAUSE_INI（目录：./Data/_RobotName_/RobotPause.ini）该函数已完成定义 新框架无此目录结构 老框架无此定义）
	bool LoadPausePara(CString sKey, double &dValue);
	//（问题：使用宏ROBOT_PAUSE_INI（目录：./Data/_RobotName_/RobotPause.ini）该函数已完成定义 新框架无此目录结构 老框架无此定义）
	bool LoadPausePara(CString sKey, CString &sValue);

	/***********************************************算法部分***********************************************/
	//（2023/12/21安川独占）
	bool RobotInverseKinematics(T_ROBOT_COORS tRobotCoors, T_ROBOT_COORS tToolCoors, std::vector<T_ANGLE_PULSE>& vtResultPulse);
	bool RobotInverseKinematics(T_ROBOT_COORS tRobotCoors, T_ANGLE_PULSE tReferencePulse, T_ROBOT_COORS tToolCoors, T_ANGLE_PULSE &tResultPulse, int nExternalAxleType = -1);
	void RobotKinematics(T_ANGLE_PULSE tCurrentPulse, T_ROBOT_COORS tToolCoors, T_ROBOT_COORS &tCurrentRobotCoors, int nExternalAxleType = -1);
	//（2023/12/21安川独占）
	bool MoveToolByWeldGun(T_ROBOT_COORS tWeldGunCoors, T_ROBOT_COORS tWeldGunTools, T_ROBOT_COORS tMagneticCoors, T_ROBOT_COORS tMagneticTools, T_ROBOT_COORS& tGunCoorsMoveMagnetic);
	T_ANGLE_PULSE ThetaToPulse(T_ANGLE_THETA tTheta);
	T_ANGLE_THETA PulseToTheta(T_ANGLE_PULSE tPulse);

	// 方向角 -> Rz
	double DirAngleToRz(double dDirAngle);
	double DirAngleToRz(double dBaseRz, double dBaseDirAngle, double dChangeDir, double dDirAngle);
	// Rz -> 方向角
	double RzToDirAngle(double dRz);
	double RzToDirAngle(double dBaseRz, double dBaseDirAngle, double dChangeDir, double dRz);

	/******************************************卡诺普机器人独占*****************************************/
	//Var
	std::map<std::string, int> XiJobCase;

	//Func
	/**
	* @description: 焊接函数，包含三部分：生成焊接文件、发送焊接文件至机器人、执行焊接文件
	* @param {vector<T_ROBOT_COORS>} vtMeasureCOORS ：焊接点位
	* @param {vector<int>} vnPtnType： 焊接点类型（MOVJ,MOVL）
	* @param {T_WELD_PARA} Para： 焊接参数
	* @param {T_ROBOT_MOVE_SPEED} tPulseMove： 焊接速度、加速度、减速度
	* @param {int} weldType： 代表不同的焊接类型(立焊、平焊)。同时也代表使用第几个焊接工艺(如：立焊使用工艺0，这个参数就是0；平焊使用工艺1)
	* @return {int}： 返回状态码
	*/
	//（2023/12/21卡诺普独占）
	int Welding(std::vector<T_ROBOT_COORS> vtWeldCOORS, double dRealWeldExPos, std::vector<int> vnPtnType, T_WELD_PARA Para, int weldType, bool arcOn = true);

	//根据轨迹制作有过度点的MOVL （线扫）（2023/12/21卡诺普独占）
	int SendTeachMoveData_L(std::vector<T_ROBOT_COORS> vtMeasureCOORS, std::vector<bool> veMeasureType, int nDowdSpeed, int nTeachSpeed, int nUpSpeed, int nTrigSigTime);
	//（2023/12/21卡诺普独占）
	int SendTeachMoveData_J(std::vector<T_ANGLE_PULSE> vtMeasurePulse, double ExternalAxle, const std::vector<int> veMeasureType, int nDowdSpeed, int nTeachSpeed, int nUpSpeed, int nTrigSigTime);
	//读取M口的状态（2023/12/21卡诺普独占）
	int Read_M(int M_Num);
	//设置M口的状态（2023/12/21卡诺普独占）
	int Write_M(int M_num, int Status);
	//读取M口的状态（2023/12/21卡诺普独占）
	int Read_Y(int Y_num);
	//设置Y口的状态（2023/12/21卡诺普独占）
	int Write_Y(int Y_num, int Status);
	//制作读写继电器的指令 （2023/12/21卡诺普独占）
	//(注意： 使用子程序需要在子程序的第一行将M500设置成1,在倒数第二行需要将M500设置为0)
	//（2023/12/21卡诺普独占）
	void MakeDout(int M_num, bool state, int lineNum, std::string* Dout_str);
	//（2023/12/21卡诺普独占）
	void MakeSleep(int SleepTime, int LineNumber, string& Sleep_Str);

	// 检查机器人是否在运动中（2023/12/21卡诺普独占）
	E_ERROR_STATE CheckRobotDone_CRP(int nDelayTime);

	/********************************************混发数据准备**********************************************/
	//（2023/12/21安川独占）
	MP_USR_VAR_INFO PrepareValData(int nIntValIdx, int nValue);
	//（2024/01/24安川独占）
	MP_USR_VAR_INFO PrepareValData(int nPosVarIdx, long lBPValue[3], UINT unToolNum = 1, UINT unUserNo = 0, UINT unPosture = 4);
	//脉冲版
	MP_USR_VAR_INFO PrepareValDataEx(int nPosVarIdx, T_ANGLE_PULSE tPulse, UINT unToolNum = 1, UINT unUserNo = 0, UINT unPosture = 4);
	//
	MP_USR_VAR_INFO PrepareValDataEx(int nPosVarIdx, T_ROBOT_COORS tPulse, UINT unToolNum = 1, UINT unUserNo = 0, UINT unPosture = 4);

	//（2023/12/21安川独占）
	MP_USR_VAR_INFO PrepareValData(int nPosVarIdx, T_ANGLE_PULSE tPulse, UINT unToolNum = 1, UINT unUserNo = 0, UINT unPosture = 4);
	//（2023/12/21安川独占）
	MP_USR_VAR_INFO PrepareValData(int nPosVarIdx, T_ROBOT_COORS tCoord, UINT unToolNum = 1, UINT unUserNo = 0, UINT unPosture = 4);

	/****************************************************其他****************************************************/
	T_ROBOT_COORS m_tRobotRunningRangeMin;//机器人极限坐标
	T_ROBOT_COORS m_tRobotRunningRangeMax;//机器人极限坐标
	
	// 10点以内任意 坐标类型、插补方式、速度 的连续运动 (运动第二个点前必须保证外部轴运动到dExAxlePos位置)
	//int ContiMoveAny(const std::vector<T_ROBOT_MOVE_INFO>& vtRobotMoveInfo);	// 改为虚函数接口
	//（问题：①使用目录 ./Data/RobotAndCar.ini 新框架无此目录结构 ②未完成SwitchIO()定义：该函数依赖SwitchIO()  该函数已完成定义：注释依赖项）
	//bool CleanGunH(int RobotNum); //1右 0左
	/***********************************************合合合合***********************************************/
	// 获取外部轴方向
	int GetPanasonicExDir(E_EXTERNAL_AXLE_TYPE eExAxisType);
	private:
		T_ANGLE_PULSE m_tAimPulse;
		T_ROBOT_COORS m_tAimCoord;

		void Open(int nSocketPort = 20001);
		void Close();
		//void AppendMotorAxisPos(T_ROBOT_COORS& tCoord);
		//void AppendMotorAxisPos(T_ANGLE_PULSE& tPulse);
		int GetPanasonicExPos(E_EXTERNAL_AXLE_TYPE eExAxisType, double &dExPos);
		int GetPanasonicExPos(E_EXTERNAL_AXLE_TYPE eExAxisType, long &lExPos);
	
public:
	// 机器人种类扩展需要重新实现的所有虚函数
	// 调用手操盒程序 ++
	virtual int CallJob(char JobName[24]);
	virtual int CallJob(CString sJobName);
	virtual void WorldCheckRobotDone(int nDelayTime = 1200);
	virtual bool CheckIsReadyRun();	// 返回上电状态 未上电时先执行上电
	//virtual E_ERROR_STATE CheckRobotDone(int nDelayTime = 1200);		//阻塞等待运动停止
	//virtual DWORD SwitchIO(CString sName, bool bOpen);
	//virtual bool CheckInIO(CString sName); // 与默认值相同 true 与默认值相反 false
	//virtual bool CheckOutIO(CString sName); // 与默认值相同 true 与默认值相反 false
	virtual double GetCurrentPos(int nAxisNo); //查询机器人当前直角坐标
	virtual bool GetInputIOVar(int name, int value);
	virtual bool SetOutIOVar(int name, int value);
	virtual T_ROBOT_COORS GetCurrentPos_ESTUN();
	virtual T_ROBOT_COORS GetCurrentPos(); //查询机器人当前直角坐标
	virtual long GetCurrentPulse(int nAxisNo); //查询机器人当前关节坐标
	virtual T_ANGLE_PULSE GetCurrentPulse(); //查询机器人当前关节坐标
	virtual double GetPositionDis(); //查询外部轴坐标 (卡诺普或安川松下外部轴)
	//virtual int scanLaser(int state); //线扫激光 state： 1 开， 0 关。
	//virtual int trackLaser(int state); //跟踪激光 state： 1 开， 0 关。
	//virtual bool MoveToSafeHeight(); // 将机器人提升到一定高度(与安全位置坐标和整座倒挂有关)
	//埃斯顿机器人一次设置大量变量 直角 //接口不稳定，用下面那个
	//virtual bool SetMultiPosVar(UINT unIndex, std::vector<T_ROBOT_COORS> vtRobotJointCoord, T_PULSE_MOVE tPosMove, int config[7] = int(0), UINT unToolNum = 1);
	//通过FTP发送大量变量 不超过300个点
	//virtual bool SetMultiPosVar(UINT unIndex, std::vector<T_ROBOT_COORS> vtRobotJointCoord, T_PULSE_MOVE tPosMove, CString Program_name, int config[7] = int(0), UINT unToolNum = 1);
	// 发送硬触发示教运动数据 nDownSpeed关节插补速度 nTeachSpeed 和 nUpSpeed 直线插补速度
	//virtual bool SendTeachMoveData(const std::vector<T_ANGLE_PULSE>& vtMeasurePulse, const  std::vector<int>& vnMeasureType, int nDownSpeed, int nTeachSpeed, int nUpSpeed, int nTrigSigTime);
	//virtual void MoveByJob(T_ANGLE_PULSE tRobotJointCoord, T_PULSE_MOVE tPulseMove, int nExternalAxleType, CString JobName = "MOVJ");
	//直角坐标运动函数 + 
	//virtual void MoveByJob(T_ROBOT_COORS tRobotJointCoord, T_PULSE_MOVE tPulseMove, int nExternalAxleType, CString JobName = "MOVJ", int config[7] = int(0));
	//外部轴运动函数
	virtual int MoveExAxisForLineScan(double dDist, int nSpeed);
	// 运动到线扫起点位置
	virtual int MoveToLineScanPos(T_ANGLE_PULSE tPulse, T_ROBOT_MOVE_SPEED tPulseMove);

	virtual bool WorldIsRunning(); // 埃斯顿专用:判断外部轴和机器人是否有运动

	// 硬触发示教运动 运动外部轴 和 机器人 (第一个点和外部轴同时运动 或 先动外部轴再动机器人)
	virtual bool TeachMove(const std::vector<T_ANGLE_PULSE>& vtMeasurePulse, double dExAxlePos, const  std::vector<int>& vnMeasureType, int nTrigSigTime);

	// 10点以内任意 坐标类型、插补方式、速度 的连续运动 (运动第二个点前必须保证外部轴运动到dExAxlePos位置)
	virtual int ContiMoveAny(const std::vector<T_ROBOT_MOVE_INFO>& vtRobotMoveInfo);
	// 焊接运动 ()
	virtual int WeldMove(const std::vector<T_ROBOT_COORS>& vtWeldPathPoints, const  std::vector<int>& vnPtnType, const T_WELD_PARA& tWeldPara, double dExAxlePos, E_WELD_SEAM_TYPE eWeldSeamType, bool bIsArcOn);
	//virtual bool CleanGunH();
	// 暂停运动
	virtual	void HoldOn();
	// 运动到指定的关节坐标
	//virtual bool MoveByJobJ(double Distence[8], int config[7], double speed, int ifAbsolutM, int ifJoint);

	virtual int MoveExAxisFun(double dDist, int nSpeed, int nMoveXYZAxisNo, double dMaxExSpeed = -1); // 最大外部周速度非扩展外部轴速度mm/min 小于0使用nSpeed
	virtual int XYZAxisNoToSoftAxisNo(int nMoveXYZAxisNo);

	// 获取外部轴坐标
	virtual double			GetExPositionDis(int nMoveXYZAxisNo);


	//埃斯顿跟踪接口
	//埃斯顿机器人 计算mode值,机器人型号，50B，20 机器人
	virtual void CalModeValue(T_ANGLE_PULSE tReferPluse, const char* RobotMode, int config[7]);
	//埃斯顿机器人 建立跟踪链接，并完成初始化(发送初始点位)
	virtual bool EstunTrackInit(const T_ROBOT_COORS& startPos, const CString& IP = "192.168.60.63", const vector<int>& Cfg = vector<int>(7, 0));
	//埃斯顿机器人 清理buffer
	virtual bool EstunClearWeldBuffer(); 
	//埃斯顿机器人 实时跟踪传递数据函数，用于将从相机得到的数据传递给机器人
	virtual bool EstunSendTrackData(const vector<T_ROBOT_COORS>& WeldPathPoint, const vector<vector<int>>& Cfg = vector<vector<int>>(2, vector<int>(7, 0)));
	//埃斯顿机器人 停止跟踪时调用
	virtual bool EstunTrackStop();
	CMutex        m_mutex;  //实时跟踪的锁
	bool TrackingState; //跟踪相机开启状态

	// 国焊坡口焊接添加
	T_ROBOT_COORS m_tFlatGrooveScanPosture; // = T_ROBOT_COORS(0.0, 500, 1600.0, 0.0, 0.0, 66.0, 0.0, 0.0, 0.0);
	T_ROBOT_COORS m_tFlatGrooveWeldPosture; // = T_ROBOT_COORS(0.0, 500, 1600.0, 4.0, -10.0, 66.0, 0.0, 0.0, 0.0);
	//T_ROBOT_COORS m_tFlatGrooveScanPosture = T_ROBOT_COORS(0.0, 1100.0, 1800.0, 0.0, 0.0, 66.0, 0.0, 0.0, 0.0);
	//T_ROBOT_COORS m_tFlatGrooveWeldPosture = T_ROBOT_COORS(0.0, 1100.0, 1800.0, 4.0, -10.0, 66.0, 0.0, 0.0, 0.0);
	T_ROBOT_COORS m_tStandDownGrooveScanPosture; // = T_ROBOT_COORS(0.0, -1200.0, 1400.0, 52.0, -60.0, 35.0, 0.0, 0.0, 0.0);
	T_ROBOT_COORS m_tStandDownGrooveWeldPosture; // = T_ROBOT_COORS(0.0, -1200.0, 1400.0, 52.0, -60.0, 35.0, 0.0, 0.0, 0.0);
	T_ROBOT_COORS m_tStandUpGrooveScanPosture; // = T_ROBOT_COORS(0.0, -1500.0, 800.0, -60.0, 62.0, -154.0, 0.0, 0.0, 0.0);
	T_ROBOT_COORS m_tStandUpGrooveWeldPosture; // = T_ROBOT_COORS(0.0, -1500.0, 800.0, -60.0, 62.0, -154.0, 0.0, 0.0, 0.0);
	int m_nScanNum;//扫描次数
};


#endif // !defined(AFX_ROBOTDRIVERADAPTOR_H__59FD46BF_CA09_4E0E_A5E0_C6B55F565624__INCLUDED_)
