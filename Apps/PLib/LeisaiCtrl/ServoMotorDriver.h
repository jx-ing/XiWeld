///////////////////// 雷赛运动控制-基于脉冲 /////////////////////
/*
说明：
1、所有非bool型返回值，0表示成功，!0表示错误码;bool型返回值，true表示成功，false表示失败，无错误码；
2、错误码正数为雷赛控制卡返回，负数由下定义；
3、为便于控制，将各轴脉冲当量设置为【1.0】，请勿更改；
*---*注意：部分功能调用官方API时，将【基于脉冲控制】和【基于脉冲当量控制】函数混用；
4、【LTSMC】库中无【基于脉冲控制】接口；
5、脉冲当量计算公式为：实际脉冲/实际距离(pulse/mm)
*/


#pragma once
#include "LTDMC.h"
#include "LTSMC.h"
#include "IOC0640.H"
//#include "DriverAdapter.h"
#include ".\Apps\PLib\PanasonicServo\AbsoluteEncoder.h"
#include "MoveCtrlModule.h"

/*返回值错误码定义*/
#define INPUT_INFO_ERROR		-1	//传入信息错误
#define NO_FOLLOW_FUN			-2	//没有跟随功能
#define	CTRL_CARD_NUM_ERROR		-3	//控制卡数量错误
#define	CTRL_CARD_NO_FOUND		-4	//没有找到控制卡
#define	IO_CARD_NO_CONNECT		-5	//没有找到IO卡
#define	ABS_ENCODER_DISENABLE	-6	//绝对编码器没有使能
#define	SERIAL_PORT_DISCONNECT	-7	//串口打开失败
#define	NO_FUNITION				-8	//无此功能
#define	CAN_DISCONNECT			-9	//Can总线没有连接



#ifndef WRITE_LOG
#define WRITE_LOG(data) {CString __str;\
__str.Format("Function:[%s], Line:[%d]	\n%s",__FUNCTION__, __LINE__, data);\
WriteLog(__str);}
#endif // !WRITE_LOG

#define CHECK_RTN_INT(nRtn) if(0 != nRtn)\
{\
WRITE_LOG(GetStr("异常返回:%d", nRtn));\
return nRtn;\
}
#define CHECK_RTN_BOOL(bRtn) if(true != bRtn)\
{\
WRITE_LOG("异常返回");\
return bRtn;\
}

#ifndef ON
#define ON	0
#endif // !ON
#ifndef OFF
#define OFF 1
#endif // !OFF

typedef enum
{
	E_CONNECT_TYPE_DIRECT,//直连
	E_CONNECT_TYPE_NETWORK,//网络
	E_CONNECT_TYPE_ETHERCAT,//EtherCAT总线
//	E_CONNECT_TYPE_SERIAL_PORT,//串口
}E_CONNECT_TYPE;

typedef enum
{
	E_CTRL_CARD_TYPE_DMC5410,
	E_CTRL_CARD_TYPE_DMC5800,
	E_CTRL_CARD_TYPE_DMC5810,
	E_CTRL_CARD_TYPE_SMC604,
	E_CTRL_CARD_TYPE_IOC0640,
	E_CTRL_CARD_TYPE_IOC1280,
	E_CTRL_CARD_TYPE_IMC30G_E,
	E_CTRL_CARD_TYPE_MAX_NUM,
}E_CTRL_CARD_TYPE;

typedef enum
{
	E_SERCO_TYPE_PANASONIC,//松下伺服
	E_SERCO_TYPE_INOVANCE,//汇川伺服
//	E_SERCO_TYPE_YASKAWA,//安川伺服
E_SERVO_TYPE_MAX_NUM,
}E_SERVO_TYPE;

typedef enum
{
	E_IO_CARD_TYPE_NONE,
	E_IO_CARD_TYPE_IOC0640,
	E_IO_CARD_TYPE_IOC1280,
	E_IO_CARD_TYPE_CAN,
	E_IO_CARD_TYPE_CAT,
E_IO_CARD_TYPE_MAX_NUM,
}E_IO_CARD_TYPE;

typedef struct
{
	double dSpeed = 0;
	double dAcc = 1;
	double dDec = 1;
	double dSParam = 0.5;
}T_AXIS_SPEED;

typedef struct
{
	int nHardCtrlCardNo = -1;//控制卡物理编号
	int nSoftCtrlCardNo = -1;//软件控制卡编号
	int nPortNo = -1;//SMC端口号，0,1 表示 CANOpen，2，3 表示 EtherCAT 端口
	E_IO_CARD_TYPE eIOType = E_IO_CARD_TYPE_NONE;//IO类型
	int nLocalIONum = 0;//本地IO数量
	int nSingleIONum = 0;//单节点IO数量
	int nNoteNum = 0;//节点数
	int nCANBaudNo = 0;//CAN总线波特率
	int nCATCycleTime = 0;//CAT总线循环周期
	int nIOCFilter = 1;//IOC卡滤波参数
}T_IO_CTRL_INFO;

typedef struct
{
	int nHardCtrlCardNo = -1;//控制卡物理编号
	int nSoftCtrlCardNo = -1;//软件控制卡编号
	int nSoftAxisNo = -1;//软件轴编号
	int nHardAxisNo = -1;//硬件轴编号
	CString strAxisName = "";//轴名称
	E_SERVO_TYPE eServoType = E_SERCO_TYPE_PANASONIC;//伺服驱动类型
	double dPulseEquivalent = 0;//脉冲当量，脉冲数/实际距离（pulse/mm ）
	int nPulseMode = 0;//脉冲模式

	bool bEnableAbsEncoder = false;//使能绝对编码器
	int nAbsEncoderPartNo = -1;//绝对编码器串口号
	int nLapPulse = 2000;//单圈脉冲数
	int nAbsEncoderDir = 1;//绝对编码器方向

	bool bEnableSoftLimit = false;//使能软限位
	double dPositiveLimit = 0;//正限位
	double dNegativeLimit = 0;//负限位

	T_AXIS_SPEED tMaxSpeed;//高速,dSpeed单位mm/s,dAccdDec单位s
	T_AXIS_SPEED tMidSpeed;//中速,dSpeed单位mm/s,dAccdDec单位s
	T_AXIS_SPEED tMinSpeed;//低速,dSpeed单位mm/s,dAccdDec单位s

	bool bAlmLogic = true;//报警电平

	int nGearFollowProfile = 0;//龙门模式,0:禁止，1:主动轴，2:从动轴
	int nMasterAxisNo = -1;//主轴编号，龙门模式为从动轴时使用
}T_AXIS_CTRL_INFO;

typedef struct
{
	int nHardCtrlCardNo = -1;//控制卡物理编号
	int nSoftCtrlCardNo = -1;//软件控制卡编号
	int nAxisNum = -1;//轴数量
	E_CONNECT_TYPE eConnectType = E_CONNECT_TYPE_DIRECT;//轴类型
	CString strNetWorkIP = "";//网络连接IP地址
	E_CTRL_CARD_TYPE eCtrlCardType = E_CTRL_CARD_TYPE_DMC5410;//运动控制卡类型
	T_IO_CTRL_INFO tIOCtrlInfo;
	std::vector<T_AXIS_CTRL_INFO> vtAxisInfo;
}T_CTRL_CARD_INFO;

class CServoMotorDriver : public CMoveCtrlModule
{
public:
	CServoMotorDriver();
	~CServoMotorDriver();
	std::vector<T_CTRL_CARD_INFO> m_vtCtrlCardInfo;
	std::map<int, CAbsoluteEncoder*> m_mAbsEncoderCtrl;//绝对编码器控制

	/*******************************  初始化  *******************************/
	virtual int InitCtrlCard(std::vector<T_CTRL_CARD_INFO> vtCtrlCardInfo);//初始化控制卡
	bool LoadCtrlCardParam(std::vector<T_CTRL_CARD_INFO> &vtCtrlCardInfo);//控制卡参数
	bool SaveCtrlCardParam(std::vector<T_CTRL_CARD_INFO> vtCtrlCardInfo);

	/*******************************  伺服相关  *******************************/
	//伺服使能
	virtual int SetSevon(int nSoftAxisNo, bool bEnable);
	virtual bool GetSevon(int nSoftAxisNo, bool &bEnable);
	//伺服准备
	virtual bool GetSevonRdy(int nSoftAxisNo, bool &bSevonRdy);

	/*******************************  运动相关  *******************************/
	//单轴运动
	virtual int PosMove(int nSoftAxisNo, long lDistance, int nMode, double dSpeed, double dAcc, double dDec, double dSParam = 0.1);//单轴脉冲运动,dSpeed单位pulse/s,dAccdDec单位s, nMode运动模式，0：相对坐标模式，1：绝对坐标模式
	virtual int PosMove(int nSoftAxisNo, double dDistance, int nMode, double dSpeed, double dAcc, double dDec, double dSParam = 0.1);//单轴位置运动,dSpeed单位mm/s,dAccdDec单位s
	virtual int ContiMove(int nSoftAxisNo, int nDir, double dSpeed, double dAcc, double dSParam = 0.1);//单轴连续运动,dSpeed单位mm/s,dAcc单位s
	virtual bool CheckAxisRun(int nSoftAxisNo, int *pnError = NULL);//检查运行状态

	//停止相关
	virtual bool EmgStop(int *nSize = NULL, int *pnError = NULL);//急停
	virtual int EmgStopCtrlCard(int nSoftCtrlCardNo);
	virtual int EmgStopAxis(int nSoftAxisNo);
	virtual int SetDecelStopTime(int nSoftAxisNo, double dDec);//设置减速停止时间,dDec单位s
	virtual int DecelStop(int nSoftAxisNo);//减速停止

	//当前坐标
	virtual int GetCurrentPosition(int nSoftAxisNo, long &lPosition);//当前位置
	virtual int GetCurrentPosition(int nSoftAxisNo, double &dPosition);
	virtual int SetCurrentPosition(int nSoftAxisNo, long lPosition);
	virtual int SetCurrentPosition(int nSoftAxisNo, double dPosition);

	//目标坐标
	virtual int GetTargetPosition(int nSoftAxisNo, long &lPosition);//目标位置
	virtual int GetTargetPosition(int nSoftAxisNo, double &dPosition);
	virtual int SetTargetPosition(int nSoftAxisNo, long lPosition, bool bupdate = false);//bupdate=true 强制改变位置
	virtual int SetTargetPosition(int nSoftAxisNo, double dPosition, bool bupdate = false);

	//速度
	virtual int GetSpeed(int nSoftAxisNo, long &lSpeed);
	virtual int GetSpeed(int nSoftAxisNo, double &dSpeed);
	virtual int ChangeSpeed(int nSoftAxisNo, long lSpeed, double dChangeTime = 0.1);//中途变速
	virtual int ChangeSpeed(int nSoftAxisNo, double dSpeed, double dChangeTime = 0.1);
	virtual double GetMaxSpeed(int nSoftAxisNo); // 获取最大速度 mm/min

	//获取设定的数据（非当前值）
	virtual int GetDataPulseEquivalent(int nSoftAxisNo, double &dPulseEquivalent);//脉冲当量
	virtual int GetDataSpeedParam(int nSoftAxisNo, T_AXIS_SPEED &tMinSpeedParam, T_AXIS_SPEED &tMidSpeedParam, T_AXIS_SPEED &tMaxSpeedParam);//脉冲当量

	/*******************************  龙门模式  *******************************/
	virtual int SetGearFollowProfile(int nMasterSoftAxisNo, int nDrivenSoftAxisNo, bool bEnable, double dRatio = 1.0);
	virtual int GetGearFollowProfile(int nDrivenSoftAxisNo, int &nMasterSoftAxisNo, bool &bEnable, double &dRatio);

	/*******************************  IO功能  *******************************/
	virtual int ReadInbit(int nSoftCtrlCardNo, int bBitNo, WORD &nStatus, int nNoteID = 0);//nNoteID等于0为本地IO,扩展结点从1开始
	virtual int ReadOutbit(int nSoftCtrlCardNo, int bBitNo, WORD &nStatus, int nNoteID = 0);//nNoteID等于0为本地IO,扩展结点从1开始
	virtual int WriteOutbit(int nSoftCtrlCardNo, int bBitNo, WORD nStatus, int nNoteID = 0);//nNoteID等于0为本地IO,扩展结点从1开始

	/*******************************  ADDA功能(CAN总线)  *******************************/
	virtual int SetADMode(int nSoftCtrlCardNo, int nNoteID, int nChannel, int nMode);//nChannel:模拟量输入通道号，范围 0~3, nMode:输入模式，0：电压模式，1：电流模式
	virtual int SetDAMode(int nSoftCtrlCardNo, int nNoteID, int nChannel, int nMode);//nChannel:模拟量输出通道号，范围 0~1, nMode:输入模式，0：电压模式，1：电流模式
	virtual int SetDAChannel(int nSoftCtrlCardNo, int nNoteID, int nChannel, double dValue);//dValuev:输出值，单位：mV/mA
	virtual int GetDAChannel(int nSoftCtrlCardNo, int nNoteID, int nChannel, double &dValue);//dValuev:输出值，单位：mV/mA
	virtual int GetADChannel(int nSoftCtrlCardNo, int nNoteID, int nChannel, double &dValue);//dValuev:输入值，单位：mV/mA

	/*******************************  绝对编码器  *******************************/
	virtual int OpenAbsEncoder(int nSoftAxisNo);//打开编码器串口
	virtual int CloseAbsEncoder(int nSoftAxisNo);//关闭串口
	virtual int GetAbsData(int nSoftAxisNo, double &dAbsData);//从绝对值编码器获取当前坐标，需要 1s 左右，获取时伺服电机必须静止
	virtual int ClearManyLapData(int nSoftAxisNo);//清空多圈数据
	virtual int SetInitLapData(int nSoftAxisNo);//保存零点单圈数据

	/*******************************  插补运动  *******************************/



	/*******************************  获取轴卡信息  *******************************/
	bool GetAxisCardInfo(int nSoftAxisNo, T_AXIS_CTRL_INFO &tAxisInfo, T_CTRL_CARD_INFO &tCtrlCardInfo);
	bool GetAxisCardInfo(int nHardCtrlCardNo, int nHardAxisNo, E_CTRL_CARD_TYPE eCtrlCardType, T_AXIS_CTRL_INFO &tAxisInfo, T_CTRL_CARD_INFO &tCtrlCardInfo);
	bool GetCtrlCardInfo(int nSoftCtrlCardNo, T_CTRL_CARD_INFO &tCtrlCardInfo);
	bool GetCtrlCardInfo(int nHardCtrlCardNo, E_CTRL_CARD_TYPE eCtrlCardType, T_CTRL_CARD_INFO &tCtrlCardInfo);

	/*******************************  其他  *******************************/
	int GetSoftAxisNo(int nSoftCtrlCardNo);
	void SetSoftAxisNo();
	// 获取脉冲当量
	double GetPulseEquivalent(int nSoftAxisNo);
private:

	int InitDirectCtrlCard(T_CTRL_CARD_INFO tCtrlCardInfo, int &nCardNum);
	int InitNetWorkCtrlCard(T_CTRL_CARD_INFO tCtrlCardInfo, int &nCardNum);
	int InitIOCCtrlCard(T_CTRL_CARD_INFO tCtrlCardInfo, int &nCardNum);

	int InitCanIOCtrl(T_IO_CTRL_INFO tIOInfo);
	int InitCatIOCtrl(T_IO_CTRL_INFO tIOInfo);

	int InitDirectAxisParam(T_AXIS_CTRL_INFO tAxisInfo);
	int InitNetWorkAxisParam(T_AXIS_CTRL_INFO tAxisInfo);



	int m_nCardNum = 0;
	int m_nAxisNum = 0;
};
