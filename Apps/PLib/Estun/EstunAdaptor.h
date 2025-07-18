#ifndef __ESTUN_ADAPTOR
#define __ESTUN_ADAPTOR
#include ".\Apps\PLib\CtrlUnit\CUnitDriver.h"
#include ".\Apps\PLib\YaskawaRobot\RobotDriverAdaptor.h"
#include "EstunRemoteApiLibWeld.h"
#include "ESTUNRobotCtrl.h"
#include <Apps/PLib/LeisaiCtrl/IOControl.h>
#include "XiRobotCtrl.h"
#include ".\OpenClass\FileOP\ini\opini.h"
#include ".\Apps\PLib\BasicFunc\Const.h"
#include "AbsCoorTransLib.h"
#include ".\Apps\PLib\CtrlUnit\CContralUnit.h"


#define WELD_TRACK_MAX_NUM			498			// 焊接最大轨迹点数
#define WRAP_TRACK_MAX_NUM			2			// 包角最大轨迹点数

class EstunAdaptor : public CRobotDriverAdaptor
{
public:
	EstunAdaptor(CString strUnitName, CLog* cLog, std::vector<CUnitDriver*>* ppUnitDriver);
	virtual ~EstunAdaptor();

	CString m_sFlatNormalWelderMode_W;		//平焊直流焊接模式参数W
	CString m_sFlatNormalWelderMode_MM;		//平焊直流焊接模式参数MM
	CString m_sFlatPulseWelderMode_W;		//平焊脉冲焊接模式参数W
	CString m_sFlatPulseWelderMode_MM;		//平焊脉冲焊接模式参数MM
	CString m_sVerNormalWelderMode_W;		//立焊直流焊接模式参数W
	CString m_sVerNormalWelderMode_MM;		//立焊直流焊接模式参数MM
	CString m_sVerPulseWelderMode_W;		//立焊脉冲焊接模式参数W
	CString m_sVerPulseWelderMode_MM;		//立焊脉冲焊接模式参数MM

	bool getWelderMode(E_WELD_SEAM_TYPE eWeldSeamType, CString& sWelderMode_W, CString& sWelderMode_MM);

	int m_nFlatWelderMode;						//焊接模式0直流1脉冲仅埃斯顿使用
	//int g_bWelderType;						//焊机类型0麦格米特1奥泰
	int m_nVerWelderMode;               //焊机立焊模式,0直流，1脉冲

	/****************************************************基本控制****************************************************/
	//清除报警信息+
	void cleanAlarm();
	//设置当前模式 0-手动模式，1-自动模式，2-远程模式，-1-报错
	virtual void SetSysMode(int mode);
	//设置示教器速度
	void SetTpSpeed(int speed);//设置TP示教器上的速度%

	//加载程序 设置变量前要先加载程序
	void LoadUserProgramer(const char* projName, const char* progName);
	//设置当前工具 !+
	virtual bool SetRobotToolNo(int nToolNo);

	/****************************************************信息获取****************************************************/
	//查询机器人当前各轴角度(暂无外部轴) +
	double GetCurrentDegree(int nAxisNo);
	//获取工具信息	+		
	BOOL GetToolData(UINT unToolNo, double adRobotToolData[6]);

	/****************************************************变量读写****************************************************/
	//,int nToolNo = 1 scoper,0-系统，1-全局，2-工程，3-程序
	virtual void SetPosVar(int nIndex, T_ROBOT_COORS tRobotCoors, int config[7] = int(0), int scoper = 2);
	virtual void SetPosVar(int nIndex, T_ANGLE_PULSE tRobotPulse, int scoper = 2);
	// 埃斯顿机器人专用: 速度变量读写
	void SetMultiSpeed(UINT index, UINT count, double** speedVar);//设置大量速度
	void SetSpeed(int nIndex, double adSpeed[5]); // 设置单个速度
	//摆焊参数
	void SetWeaveDate(const char* name, ESTUN_WeaveDate WeaveDate, int scope = 2);
	//获取一个指定I变量
	int GetIntVar(int nIndex, char* cStrPreFix = "INT");
	//设置Real变量
	bool SetRealVar(int nIndex, double value, char* cStrPreFix = "REAL", int score = 1);//主要用于发送电流电压 scoper,0-系统，1-全局，2-工程，3-程序

	/****************************************************运动函数****************************************************/

	//关节坐标系单轴运动函数(暂无外部轴)++  (用当前用户和工具)	/*1 - 10000 representing 0.01 to 100.0 %),推荐500*/
	virtual int AxisPulseMove(int nAxisNo, long lDist, long lRobotSpd, WORD wCoorType = COORD_REL, int nToolNo = 1, long lCoordFrm = 1);
	//直角坐标系单轴运动函数(暂无外部轴)
	virtual void PosMove(int nAxisNo, double dDist, long lRobotSpd,  WORD wCoorType = COORD_REL, int nToolNo = 1, long lCoordFrm = 0);
	//通用移动函数
	virtual void MoveByJob(double* dRobotJointCoord, T_ROBOT_MOVE_SPEED tPulseMove, int nExternalAxleType, int nPVarType, CString JobName, int config[7]) override;
	// 埃斯顿角度坐标专用函数（只能关节插补）
	void MoveByJobForAngle(double* adRobotAngleCoord, T_ROBOT_MOVE_SPEED tPulseMove, CString JobName = "MOVJ");

	//设置IO变量
	virtual bool SetOutIOVar(int name, int value);
	//bool GetOutIOVar(int name, int& value);
	virtual bool GetInputIOVar(int name, int& value);

	//埃斯顿机器人 计算mode值,机器人型号，50B，20 机器人
	virtual void CalModeValue(T_ANGLE_PULSE tReferPluse, const char* RobotMode, int config[7]);

public:
	/**
     * @description: 实时跟踪传递数据函数，用于将从相机得到的数据传递给机器人
     * @param {T_ROBOT_COORS&} WeldPathPoints： 焊接点位数据
     * @param {vector<int>&} Cfg：点位数据对应的config(默认全0)
     * @return {*} 修改是否成功
     */
	virtual bool EstunSendTrackData(const vector<T_ROBOT_COORS>& WeldPathPoint, const vector<vector<int>>& Cfg = vector<vector<int>>(2, vector<int>(7, 0)));
	/**
	 * @description: 建立跟踪链接，并完成初始化(发送初始点位)
	 * @param {CString&} IP
	 * @param {T_ROBOT_COORS&} startPos： 跟踪起始点位数据
	 * @param {vector<int>&} Cfg：点位数据对应的config(默认全0)
	 * @return {*} 修改是否成功
	 */
	virtual bool EstunTrackInit(const T_ROBOT_COORS& startPos,const CString& IP = "192.168.60.63",  const vector<int>& Cfg = vector<int>(7, 0));
	/**
	 * @description: 停止跟踪功能(在走完缓冲区最后一个点位后一段时间内没有数据也会自动关闭跟踪服务)
	 * @return {*} {*} 修改是否成功
	 */
	virtual bool EstunTrackStop(); //停止跟踪时调用
	virtual bool EstunClearWeldBuffer(); //清理buffer
	EstunRemoteApiLibWeld api;//构造api对象
    
	//设置一个指定I变量
	virtual void SetIntVar(int nIndex, int nValue, int score = 2, char* cStrPreFix = "INT");
	/************************************************运动状态检测************************************************/
	virtual E_ERROR_STATE CheckRobotDone(int nDelayTime = 1200); //阻塞等待运动停止
	virtual bool WorldIsRunning() override; // 判断外部轴和机器人是否有运动
	virtual void WorldCheckRobotDone(int nDelayTime = 1200);
	virtual void HoldOn() override;
	//恢复暂停中的机器人 !+
	virtual void HoldOff();
	// 机器人：返回上电状态 未上电时先执行上电
	bool CheckIsReadyRun();
	/**************************************************变量写入**************************************************/
	//埃斯顿机器人一次设置大量变量 直角 //接口不稳定，用下面那个
	virtual bool SetMultiPosVar(UINT unIndex, std::vector<T_ROBOT_COORS> vtRobotJointCoord, T_ROBOT_MOVE_SPEED tPosMove, int config[7] = int(0), UINT unToolNum = 1);
	//通过FTP发送大量变量 不超过WELD_TRACK_MAX_NUM个点
	virtual bool SetMultiPosVar(UINT unIndex, std::vector<T_ROBOT_COORS> vtRobotJointCoord, T_ROBOT_MOVE_SPEED tPosMove, CString Program_name, int config[7] = int(0), UINT unToolNum = 1);
	// 发送硬触发示教运动数据 nDownSpeed关节插补速度 nTeachSpeed 和 nUpSpeed 直线插补速度
	bool SendTeachMoveData(const std::vector<T_ANGLE_PULSE>& vtMeasurePulse, const  std::vector<int>& vnMeasureType, int nDownSpeed, int nTeachSpeed, int nUpSpeed, int nTrigSigTime);

	/**************************************************坐标读取**************************************************/
	virtual double			GetCurrentPos(int nAxisNo) override;		//查询机器人当前直角坐标
	virtual T_ROBOT_COORS GetCurrentPos_ESTUN();
	virtual T_ROBOT_COORS	GetCurrentPos() override;				//查询机器人当前直角坐标
	virtual long			GetCurrentPulse(int nAxisNo) override;	//查询机器人当前关节坐标
	virtual T_ANGLE_PULSE	GetCurrentPulse() override;				//查询机器人当前关节坐标	
	virtual double			GetPositionDis();				//查询外部轴坐标	
	// 获取外部轴坐标
	virtual double			GetExPositionDis(int nMoveXYZAxisNo);

	/**************************************************IO控制函数**************************************************/
	//virtual bool	CheckOutIO(CString sName);				// ！目前只有线扫激光、跟踪激光、线扫触发、跟踪触发的iO确定，其他的确认好后填进去
	//virtual int		scanLaser(int state);					//线扫激光 state： 1 开， 0 关。
	//virtual int		trackLaser(int state);					//跟踪激光 state： 1 开， 0 关。
	/**************************************************普通移动函数**************************************************/
	virtual int CallJob(char JobName[24]); //调用手操盒程序 ++
	virtual int CallJob(CString sJobName);
	virtual void MoveByJob(T_ROBOT_COORS tRobotJointCoord, T_ROBOT_MOVE_SPEED tPulseMove, int nExternalAxleType, CString JobName, int config[7]) override;////默认 mode值Cf值为零	
	virtual void MoveByJob(T_ANGLE_PULSE tRobotJointCoord, T_ROBOT_MOVE_SPEED tPulseMove, int nExternalAxleType, CString JobName = "MOVJ");
	/**************************************************特殊移动函数**************************************************/
	//最多支持十点运动
	virtual T_ROBOT_MOVE_INFO PVarToRobotMoveInfo(int nVarNo, T_ANGLE_PULSE tPulse, T_ROBOT_MOVE_SPEED tSpeed, int nMoveType = MOVJ, UINT unToolNum = 1, UINT unUserNo = 0, UINT unPosture = 4);
	virtual T_ROBOT_MOVE_INFO PVarToRobotMoveInfo(int nVarNo, T_ROBOT_COORS tCoord, T_ROBOT_MOVE_SPEED tSpeed, int nMoveType = MOVL, UINT unToolNum = 1, UINT unUserNo = 0, UINT unPosture = 4);//RefPulse为tCoord的关节位置，埃斯顿机器人必须要给值
	virtual bool SetMoveValue(std::vector<T_ROBOT_MOVE_INFO> vtMoveInfo, bool bSafeMove = true, bool bUsePB = false);
	/**************************************************流程移动函数**************************************************/
	//外部轴移动函数
	virtual int MoveExAxisForLineScan(double dDist, int nSpeed);
	// 将机器人提升到一定高度(与安全位置坐标和整座倒挂有关)
	virtual bool MoveToSafeHeight();
	// 硬触发示教运动 运动外部轴 和 机器人 (第一个点和外部轴同时运动 或 先动外部轴再动机器人)
	virtual bool TeachMove(const std::vector<T_ANGLE_PULSE>& vtMeasurePulse, double dExAxlePos, const  std::vector<int>& vnMeasureType, int nTrigSigTime);
	// 清枪剪丝
	//virtual bool CleanGunH();
	// 运动到线扫起点位置
	virtual int MoveToLineScanPos(T_ANGLE_PULSE tPulse, T_ROBOT_MOVE_SPEED tPulseMove);

	// 10点以内任意 坐标类型、插补方式、速度 的连续运动
	virtual int ContiMoveAny(const std::vector<T_ROBOT_MOVE_INFO>& vtRobotMoveInfo) override;

	// 无摆焊 或 机器人单一摆 焊接运动
	int WeldMoveRobotWeave(const std::vector<T_ROBOT_COORS>& vtWeldPathPoints, const  std::vector<int>& vnPtnType, const T_WELD_PARA& tWeldPara, double dExAxlePos, E_WELD_SEAM_TYPE eWeldSeamType, bool bIsArcOn);
	bool SendWeldProgram(int nWeldTrackNum, E_WELD_SEAM_TYPE eWeldSeamType);
	// 自定义生成三角摆 焊接运动
	int WeldMoveTriangleWeave(const std::vector<T_ROBOT_COORS>& vtWeldPathPoints, const  std::vector<int>& vnPtnType, const T_WELD_PARA& tWeldPara, double dExAxlePos, E_WELD_SEAM_TYPE eWeldSeamType, bool bIsArcOn);
	// 计算平焊立焊 三角摆L摆 完整轨迹
	bool CalcWeaveWeldTrack(std::vector<T_ROBOT_COORS>& vtWeldTrack, const T_WELD_PARA& tWeldPara, E_WELD_SEAM_TYPE eWeldSeamType, double& dRealWeldSpeed);

	// 自定义生成L摆焊接运动
	int WeldMoveLWeave(const std::vector<T_ROBOT_COORS>& vtWeldPathPoints, const  std::vector<int>& vnPtnType, const T_WELD_PARA& tWeldPara, double dExAxlePos, E_WELD_SEAM_TYPE eWeldSeamType, bool bIsArcOn);

	// 焊接运动 ()
	virtual int WeldMove(const std::vector<T_ROBOT_COORS>& vtWeldPathPoints, const  std::vector<int>& vnPtnType, const T_WELD_PARA& tWeldPara, double dExAxlePos, E_WELD_SEAM_TYPE eWeldSeamType, bool bIsArcOn);

	bool SendWeldTriangleWeaveProgram(int nWeldTrackNum, E_WELD_SEAM_TYPE eWeldSeamType);

	bool SendWeldLWeaveProgram(int nWeldTrackNum, E_WELD_SEAM_TYPE eWeldSeamType);

	// 世界坐标：外部轴运动函数,增加轴号入参判定移动轴
	virtual int MoveExAxisFun(double dDist, int nSpeed, int nMoveXYZAxisNo, double dMaxExSpeed = -1); // 最大外部周速度非扩展外部轴速度mm/min 小于0使用nSpeed
	virtual int XYZAxisNoToSoftAxisNo(int nMoveXYZAxisNo);

private:
	// 计算姿态从dPosture1->dPosture2 变化比例为dRatio的姿态值
	double CalcPosture(double dPosture1, double dPosture2, double dRatio);

	ESTUNRobotCtrl* m_pEstunRobot;
	CString m_sRobotMode;
};

#endif // !__ESTUN_ADAPTOR
