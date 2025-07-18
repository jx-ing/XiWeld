#pragma once

#include "Apps\PLib\BasicFunc\BaseStruct.h"
#include ".\Apps\PLib\BasicFunc\Const.h"
#include ".\Apps\PLib\YaskawaRobot\RobotDriverAdaptor.h"
#include ".\Apps\PLib\Estun\EstunAdaptor.h"
#include "CUnitDriver.h"
#include ".\Apps\PLib\DahengCam\DHGigeImageCapture.h"
#include ".\Apps\PLib\KinectCam\KinectControl.h"
#include ".\Apps\PLib\LeisaiCtrl\CBasicIOControl.h"
#include ".\Apps\PLib\BasicFunc\ChoiceResources.h"
#include ".\Apps\PLib\MechMidCam\CMechEyeCtrl.h"

#ifndef ON
#define ON	0
#define OFF 1
#endif

#ifndef MAX_IO_NUM
#define MAX_IO_NUM 9999
#endif

#ifndef DELETE_UNIT
#define DELETE_UNIT(obj) if(obj != NULL)\
{\
delete obj;\
}
#endif

#ifndef MESSAGE_BOX_UNIT
#define MESSAGE_BOX_UNIT(data) {CString __str;\
__str.Format("Function:[%s] Line:[%d]	\ndata: %s",__FUNCTION__, __LINE__, data);\
XUI::MesBox::PopError((const char*)__str);\
}
#endif

#ifndef WRITE_LOG_UNIT
#define WRITE_LOG_UNIT(data) {CString __str;\
__str.Format("Function:[%s] Line:[%d]	\ndata: %s",__FUNCTION__, __LINE__, data);\
GetLog()->Write(__str);}
#endif

#ifndef CHECK_BOOL_RTN_UNIT
#define CHECK_BOOL_RTN_UNIT(data, str) if(data != TRUE)\
{\
WRITE_LOG_UNIT(str);\
return FALSE;\
}
#endif

#ifndef CHECK_BOOL_BOX_RTN_UNIT
#define CHECK_BOOL_BOX_RTN_UNIT(data, str) if(data != TRUE)\
{\
MESSAGE_BOX_UNIT(str);\
return FALSE;\
}
#endif

#ifndef CHECK_INT_RTN_UNIT
#define CHECK_INT_RTN_UNIT(data, str) {int temp = data;\
if(temp != 0)\
{\
WRITE_LOG_UNIT(str);\
return temp;\
}\
}
#endif

#ifndef CHECK_INT_BOX_RTN_UNIT
#define CHECK_INT_BOX_RTN_UNIT(data, str) {int temp = data;\
if(temp != 0)\
{\
MESSAGE_BOX_UNIT(str);\
return temp;\
}\
}
#endif

#ifndef CHECK_IO_RTN_UNIT
#define CHECK_IO_RTN_UNIT(data) if(data < 0 || data > MAX_IO_NUM)\
{\
MESSAGE_BOX_UNIT(GetStr("IO无效,编号：%d", data));\
return -1;\
}
#endif

#ifndef CHECK_CTRL_CARD_RTN_UNIT
#define CHECK_CTRL_CARD_RTN_UNIT if(m_pBasicIOControl == NULL)\
{\
MESSAGE_BOX_UNIT("控制卡驱动无效");\
return -1;\
}
#endif

typedef enum
{
	E_UNIT_TYPE_HANDLING = 0,	//搬运
	E_UNIT_TYPE_SORTING,		//分拣
	E_UNIT_TYPE_CUTTING,		//切割
	E_UNIT_TYPE_WELD,			//焊接
	E_UNIT_TYPE_POLISH,			//打磨
	E_UNIT_TYPE_FLIP,			//翻面
	E_UNIT_TYPE_MAX_NUM,
}E_UNIT_TYPE;

typedef struct
{
	int nUnitNo = -1;
	CString strUnitName;
	CString strChineseName;
	CString strUnitType;
	int nContralUnitType = 0;
}T_CONTRAL_UNIT;

class CContralUnit
{
public:
	CContralUnit(T_CONTRAL_UNIT tCtrlUnitInfo, CServoMotorDriver *pServoMotorDriver = NULL);
	~CContralUnit();

	//**********************************************//
	//*											   *//
	//******************* 初始化 *******************//
	//*											   *//
	//**********************************************//
	/*
	* InitContralResource
	* 初始化控制资源（机器人+控制卡伺服电机+相机）
	* param		
	* return	0:成功 其他:错误代码
	*/
	int InitContralResource();


	// 根据机器人号创建机器人指针(多品牌)
	CRobotDriverAdaptor* InitRobot(CString strUnitName, CLog* cLog, std::vector<CUnitDriver*>* ppUnitDriver);

	/*
	* InitCamera
	* 初始化所有相机
	* param
	* return	0:成功 其他:错误代码
	*/
	int InitCamera();

	/*
	* InitDHCamera
	* 初始化大恒相机
	* param		tCameraPara:大恒相机参数
	* return	0:成功 其他:错误代码
	*/
	int InitDHCamera(T_CAMREA_PARAM tCameraPara);

	/*
	* SwitchDHCamera
	* 开关大恒相机
	* param		
		nCameraNo	:相机总编号
		bOpen		:true打开false关闭 
		bSwitchLaser:是否开关关联的激光 
		eCaptureMode:采集模式 
		eCallBackMode:存图模式
	* return	0:成功 其他:错误代码
	* 注意：所有大恒相机必须写在配置文件的前面，非大恒相机后写！！！！！！！
	*/
	int SwitchDHCamera(int nCameraNo, bool bOpen, bool bSwitchLaser = true,
		E_DHGIGE_ACQUISITION_MODE eCaptureMode = E_ACQUISITION_MODE_SOURCE_SOFTWARE, 
		E_DHGIGE_CALL_BACK eCallBackMode = E_CALL_BACK_MODE_OFF);

	/*
	* InitDepthCamera
	* 初始化深度（全景）相机
	* param		tDepthCameraPara:深度（全景）相机参数
	* return	0:成功 其他:错误代码
	*/
	int InitDepthCamera(T_DEPTH_CAMREA_DRIVER_PARAM tDepthCameraPara);

	/*
	* InitMecheyeCamera
	* 初始化梅卡曼德相机
	* param		tMecheyeCameraDriverPara:梅卡曼德相机参数
	* return	0:成功 其他:错误代码
	*/
	int InitMecheyeCamera(T_MECHEYE_CAMREA_DRIVER_PARAM &tMecheyeCameraDriverPara);
	//**********************************************//
	
	//**********************************************//
	//*											   *//
	//*************** 机器人基础功能 ***************//
	//*											   *//
	//**********************************************//

	/*
	* GetRobotCtrl
	* 获取机器人控制指针
	* param		
	* return	机器人控制指针
	*/
	CRobotDriverAdaptor *GetRobotCtrl();

	/*
	* GetRobotEnableState
	* 得到是否使用机器人标志
	* param		
	* return	true:使用机器人 false:没有使用机器人
	*/
	bool GetRobotEnableState();

	//-------获取信息-------//
	/*
	* GetRobotConnectState
	* 得到机器人连接状态（未测试）
	* param
	* return	TRUE:连接 FALSE:未连接
	*/
	BOOL GetRobotConnectState();

	/*
	* GetRobotTool
	* 得到机器人工具
	* param		nToolNo:工具编号 tTool:机器人工具
	* return	TRUE:成功 FALSE:失败
	*/
	BOOL GetRobotTool(int nToolNo, T_ROBOT_COORS &tTool);

	/*
	* GetRobotTool
	* 得到机器人工具
	* param		nToolNo:工具编号
	* return	机器人工具
	*/
	T_ROBOT_COORS GetRobotTool(int nToolNo);

	/*
	* GetRobotPos
	* 得到机器人当前直角坐标
	* param		nToolNo:工具编号，可获取指定工具下直角坐标
	* return	当前直角坐标
	*/
	T_ROBOT_COORS GetRobotPos(int nToolNo = -1);

	/*
	* GetRobotPulse
	* 得到机器人当前关节坐标
	* param		
	* return	当前关节坐标
	*/
	T_ANGLE_PULSE GetRobotPulse();
	//-------基础操作-------//
	//-------运动相关-------//
	//-------检测停止-------//
	/*
	* RobotCheckRunning
	* 检查机器人运行状态
	* param
	* return	TRUE:运行中 FALSE:停止
	*/
	BOOL RobotCheckRunning();

	/*
	* RobotCheckDone
	* 等待机器人运动结束
	* param		nDelayTime:等待开始运动时间（ms）
	* return	TRUE:正常停止 FALSE:异常停止
	*/
	BOOL RobotCheckDone(int nDelayTime = 1000);

	/*
	* RobotCheckDone
	* 等待机器人运动结束
	* param		tAimCoor:检查目标坐标 tCompareLimit:检查坐标范围（整数表示检查允许误差，负数表示不检查） nTool:检查坐标工具
	* return	TRUE:正常停止 FALSE:异常停止
	*/
	BOOL RobotCheckDone(T_ROBOT_COORS tAimCoor, T_ROBOT_COORS tCompareLimit = T_ROBOT_COORS(1, 1, 1, -1, -1, -1, 1, 1, 1), int nTool = -1);

	/*
	* RobotCheckDone
	* 等待机器人运动结束
	* param		tAimCoor:检查目标关节坐标 tCompareLimit:检查坐标范围（整数表示检查允许误差，负数表示不检查）
	* return	TRUE:正常停止 FALSE:异常停止
	*/
	BOOL RobotCheckDone(T_ANGLE_PULSE tAimPulse, T_ANGLE_PULSE tCompareLimit = T_ANGLE_PULSE(10, 10, 10, 10, 10, 10, 10, 10, 10));
	//**********************************************//

	//**********************************************//
	//*											   *//
	//************** 伺服电机基础功能 **************//
	//*											   *//
	//**********************************************//

	/*
	* GetMotorCtrl
	* 获取伺服电机控制指针
	* param		nUnitMotorNo:电机编号
	* return	伺服电机控制指针
	*/
	CUnitDriver *GetMotorCtrl(int nUnitMotorNo = 0);

	/*
	* GetMotorEnableState
	* 得到是否使用伺服电机标志
	* param		
	* return	true:使用伺服电机 false:没有使用伺服电机
	*/
	bool GetMotorEnableState();

	/*
	* GetMotorNum
	* 获取伺服电机数量
	* param		
	* return	伺服电机数量
	*/
	int GetMotorNum();
	//-------获取信息-------//

	/*
	* GetCurrentPosition
	* 获取当前伺服电机坐标（mm）
	* param		nUnitMotorNo:单元内伺服电机编号 dPosition:伺服电机坐标（mm）
	* return	TRUE:正常停止 FALSE:异常停止
	*/
	int GetCurrentPosition(int nUnitMotorNo, double& dPosition);
	//-------基础操作-------//
	//-------运动相关-------//

	/*
	* AbsPosMove
	* 伺服电机绝对坐标运动
	* param		nUnitMotorNo:单元内伺服电机编号 dAbsPosture:伺服电机绝对坐标（mm） dSpeed:运动速度（mm/s）
	* param		dAcc: dDec: dSParam:
	* return	TRUE:正常停止 FALSE:异常停止
	*/
	int AbsPosMove(int nUnitMotorNo, double dAbsPosture, double dSpeed, double dAcc, double dDec, double dSParam = 0.1);

	/*
	* RobotCheckDone
	* 等待机器人运动结束
	* param		tAimCoor:检查目标关节坐标 tCompareLimit:检查坐标范围（整数表示检查允许误差，负数表示不检查）
	* return	TRUE:正常停止 FALSE:异常停止
	*/
	int RelaPosMove(int nUnitMotorNo, double dRelaPosture, double dSpeed, double dAcc, double dDec, double dSParam = 0.1);
	//-------检测停止-------//

	/*
	* RobotCheckDone
	* 等待机器人运动结束
	* param		tAimCoor:检查目标关节坐标 tCompareLimit:检查坐标范围（整数表示检查允许误差，负数表示不检查）
	* return	TRUE:正常停止 FALSE:异常停止
	*/
	BOOL MotorCheckRunning(int nUnitMotorNo = 0);

	/*
	* RobotCheckDone
	* 等待机器人运动结束
	* param		tAimCoor:检查目标关节坐标 tCompareLimit:检查坐标范围（整数表示检查允许误差，负数表示不检查）
	* return	TRUE:正常停止 FALSE:异常停止
	*/
	BOOL MotorCheckDone_CheckRobot(int nUnitMotorNo, double dAbsPosture);

	/*
	* RobotCheckDone
	* 等待机器人运动结束
	* param		tAimCoor:检查目标关节坐标 tCompareLimit:检查坐标范围（整数表示检查允许误差，负数表示不检查）
	* return	TRUE:正常停止 FALSE:异常停止
	*/
	BOOL MotorCheckDone(int nUnitMotorNo, double dAbsPosture);

	/*
	* RobotCheckDone
	* 等待机器人运动结束
	* param		tAimCoor:检查目标关节坐标 tCompareLimit:检查坐标范围（整数表示检查允许误差，负数表示不检查）
	* return	TRUE:正常停止 FALSE:异常停止
	*/
	BOOL MotorCheckDone_CheckRobot(int nUnitMotorNo, double dAbsPosture, double dEarlyEndDistance);

	/*
	* RobotCheckDone
	* 等待机器人运动结束
	* param		tAimCoor:检查目标关节坐标 tCompareLimit:检查坐标范围（整数表示检查允许误差，负数表示不检查）
	* return	TRUE:正常停止 FALSE:异常停止
	*/
	BOOL MotorCheckDone(int nUnitMotorNo, double dAbsPosture, double dEarlyEndDistance);
	//**********************************************//

	//**********************************************//
	//*											   *//
	//**************** 相机基础功能 ****************//
	//*											   *//
	//**********************************************//
	//需按照实际相机类型转换指针类型
	void* GetCameraCtrl(int nCameraNo);
	T_CAMREA_PARAM GetCameraParam(int nCameraNo);

	CDHGigeImageCapture *GetDHCameraCtrl(int nSameTypeNo = 0);
	CKinectControl *GetKinectCameraCtrl(int nSameTypeNo = 0);
#if ENABLE_MECH_EYE
	CMechEyeCtrl* GetMechEyeCameraCtrl(int nSameTypeNo = 0);
#endif

	//-------采图-------//
	BOOL CaptureImage(IplImage * Image, int nSameTypeNo = 0);


	//**********************************************//

	//**********************************************//
	//*											   *//
	//***************** 基础I/O功能 ****************//
	//*											   *//
	//**********************************************//

	int ReadInbit(int nIONo, WORD& wState);
	int ReadOutbit(int nIONo, WORD& wState);
	int WriteOutbit(int nIONo, WORD wState);

	//-------整体控制-------//
	int GetAllEmgStopState(WORD& wState);									//获取总急停信号
	int OpenAirSolenoidValue();												//气路电磁阀开启
	int CloseAirSolenoidValue();											//气路电磁阀关闭
	int GetAirPressureSignal(WORD &wState);									//气体压强信号
	int OpenRobotControlCabinetPower();										//开启机器控制柜电源
	//  警示灯开关（绿1，黄2，红3）
	int SetCautionLightGreen(WORD on_off);//警示灯绿
	int SetCautionLightYellow(WORD on_off);//警示灯黄
	int SetCautionLightRed(WORD on_off);//警示灯红

	//-------相机相关（全景）-------//
	//电源
	int OpenCameraPower(int nCameraNo = 0);										//打开相机电源
	int CloseCameraPower(int nCameraNo = 0);									//关闭相机电源

	//-------相机相关（大恒）-------//
	//闪光灯/补光灯
	int OpenSupLED(int nCameraNo = 0);											//打开闪光灯/补光灯电源
	int CloseSupLED(int nCameraNo = 0);											//关闭闪光灯/补光灯电源
	int OpenSupLEDCtrlSignalPower(int nCameraNo = 0);							//打开闪光灯控制信号电源
	int CloseSupLEDCtrlSignalPower(int nCameraNo = 0);							//关闭闪光灯控制信号电源

	//镜头盖
	int OpenLensCap(int nCameraNo = 0);										//打开镜头盖
	int CloseLensCap(int nCameraNo = 0);									//关闭镜头盖
	int GetLensCapState(WORD &wState, int nCameraNo = 0);					//获取镜头盖状态

	//线激光
	int GetLaserNum(int nCameraNo);
	int OpenLaser(int nCameraNo = 0, int nLeaserLineNo = 0);					//打开激光线
	int CloseLaser(int nCameraNo = 0, int nLeaserLineNo = 0);					//关闭激光线

	//-------机器人相关-------//
	//int OpenRobot();														//打开外部启动
	//int CloseRobot();														//关闭外部启动
	//int OpenCallJob();														//打开程序调出IO
	//int CloseCallJob();														//关闭程序调出IO
	//int OpenRobotPause();													//打开外部暂停IO
	//int CloseRobotPause();													//关闭外部暂停IO
	//int OpenExRobotEmg();													//打开机械臂外部急停
	//int CloseExRobotEmg();													//关闭机械臂外部急停
	//int OpenResetErrors();													//打开复位机械臂错误/报警
	//int CloseResetErrors();													//关闭复位机械臂错误/报警

	//int GetRobotRunningSignal(WORD &wState);								//获取机器人运行信号
	int GetRobotServoSignal(WORD &wState);									//获取机械臂伺服接通中信号
	//int GetRobotErrorsSignal(WORD &wState);									//获取机械臂发出错误/报警信号
	//int GetRobotBatteryWarningSignal(WORD &wState);							//获取电池警报信号
	//int GetRobotOperationalOriginSignal(WORD &wState);						//获取作业原点信号
	//int GetRobotMidwayStartSignal(WORD &wState);							//获取可中途启动信号
	//int GetRobotEmgStopSignal(WORD &wState);								//获取急停输入信号
	//int GetRobotLongRangeModeSignal(WORD &wState);							//获取远程模式信号
	//int GetRobotManualModeSignal(WORD &wState);								//获取手动模式信号
	//int GetRobotTeachModeSignal(WORD &wState);								//获取示教模式信号


	//**********************************************//


	






	//**********************************************//
	//*											   *//
	//****************** 硬件资源 ******************//
	//*											   *//
	//**********************************************//
	//机器人驱动
	CRobotDriverAdaptor *m_pYasakawaRobotDriver = NULL;//安川
	//伺服驱动
	std::vector < CUnitDriver *> m_pContralCard;//雷赛控制卡
	//IO控制
	CBasicIOControl *m_pBasicIOControl;//雷赛控制卡
	//相机驱动
	std::vector<CDHGigeImageCapture*> m_vpImageCapture;//大恒相机
	std::vector < CKinectControl *> m_vpKinectControl;//全景相机

#if ENABLE_MECH_EYE
	std::vector < CMechEyeCtrl*> m_vpMechMindDevice;//梅卡曼德相机
#endif

	//**********************************************//


	//**********************************************//
	//*											   *//
	//****************** 加载参数 ******************//
	//*											   *//
	//**********************************************//
	T_CONTRAL_UNIT m_tContralUnit;
	T_COMMON_IO m_tCommonIO;
	int m_nMeasureCameraNo = -1;
	int m_nTrackCameraNo = -1;
	int m_nLineScanCameraNo = -1;
	std::vector< T_MOTOR_PARAM> m_vtMotorPara;
	std::vector<T_CAMREA_PARAM> m_vtCameraPara;
	BOOL LoadBaseIOParam(CString strUnitName, T_COMMON_IO &m_tCommonIO);//加载基础IO参数
	BOOL LoadRobotIOParam(CString strUnitName, T_YASKAWA_ROBOT_IO &m_tRobotIO);//加载机器人IO参数
	BOOL LoadMotorParam(CString strUnitName, std::vector< T_MOTOR_PARAM> &m_vtMotorPara);//加载伺服电机参数

	//****************** 焊接添加 ******************//
	BOOL LoadDeviceIO();
	BOOL LoadRobotIOParam();
	DWORD SwitchIO(CString sName, bool bOpen);
	bool CheckCO2IO(CString sName); // 与默认值相同 true 与默认值相反 false 宣城临时
	bool CheckInIO(CString sName); // 与默认值相同 true 与默认值相反 false
	bool CheckOutIO(CString sName); // 与默认值相同 true 与默认值相反 false
	std::map<CString, T_IO_PARAM> m_mtRobotIO; // 多机器人区分的多个IO
	std::map<CString, T_IO_PARAM> m_mtDeviceIO; // 整体设备控制IO 每个控制单元都保存一份设备IO,可通过控制单元控制

	//加载相机参数
	bool LoadDHCompen(COPini& opini, XI_POINT& tPoint);
	bool LoadDHCompenNew(COPini& opini, double ppdHandEyeCompen[][4]);
	BOOL LoadCameraParam(CString strUnitName, std::vector<T_CAMREA_PARAM>& vtCameraPara);
	bool LoadDHCameraParam(COPini& opini, bool& bAutoOpen, int &nInstallPos, E_FLIP_MODE &eFilpMode, T_ROBOT_COORS& tCameraTool);
	BOOL LoadDHCameraParam(COPini& opini, T_DH_CAMREA_DRIVER_PARAM& tDHCameraDriverPara);//大恒相机
	BOOL LoadDepthCameraParam(COPini& opini, T_DEPTH_CAMREA_DRIVER_PARAM& tPANOCameraDriverPara);//全景相机
	BOOL LoadMecheyeCameraParam(COPini& opini, T_MECHEYE_CAMREA_DRIVER_PARAM& tMecheyeCameraDriverPara);//梅卡曼德相机
	BOOL LoadCameraBaseParam(COPini& opini, T_HAND_EYE_CALI_PARAM& tHandEyeCaliPara);//相机内参
	BOOL LoadCameraHandEyeCaliParam(COPini& opini, T_HAND_EYE_CALI_PARAM &tHandEyeCaliPara);//手眼关系
	BOOL LoadCameraTransfomParam(COPini& opini, T_CAMERA_TRANSFORM_PARAM& tCameraTransfomPara);//转化关系
	BOOL LoadCameraDistortionParam(COPini& opini, T_CAMERA_DISTORTION& tCameraDistortion);//畸变参数
	BOOL LoadLaserPlanEquation(COPini& opini, T_LASER_PLAN_EQUATION& tLaserPlanEquation);
	BOOL LoadCameraIOParam(COPini& opini, T_CAMERA_IO& tCameraIO);//IO参数
	//**********************************************//

	CLog* GetLog();
//protected:
	CString GetFolderPath();
	CString GetUnitName();

	
private:
	CServoMotorDriver *m_pServoMotorDriver;
	bool m_bInitRobotMark = false;
	CLog* m_cUnitLog = NULL;
};

