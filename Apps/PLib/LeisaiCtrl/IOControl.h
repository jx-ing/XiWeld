// IOControl.h: interface for the CIOControl class.
//说明：
//DMC5400:支持单张DMC5000系列卡CAN扩展
//IOC0640:支持单张或多张IOC0640卡
//作者：
//江文奇
//修改日期：
//20210608
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_IOCONTROL_H__492E8CFB_AB04_42AC_8954_92B41A9D6719__INCLUDED_)
#define AFX_IOCONTROL_H__492E8CFB_AB04_42AC_8954_92B41A9D6719__INCLUDED_


#include ".\Apps\PLib\BasicFunc\Const.h"

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#define ON	0
#define OFF 1
#define INVALID_IO_NUM 9999

//新增类型请在初始化时将新增类型加入m_mapCardModel成员变量
const int IOC0640 = 1;// _T("IOC0640");
const int IOC1280 = 2;// _T("IOC1280");
const int DMC_CAN = 3;// _T("DMC_CAN");
const int DMC_CAT = 4;// _T("DMC_CAT");

typedef struct
{
	//通用
	int nCardModel;		
	int nHardwareCardNo;
	int nSoftwareCardNoStrat;
	int nSoftwareCardNoEnd;
	int nMaxIONum;
	//IOC
	int nIOCFILTER;
	//CAN
	int nCANCount;
	int nCANBaud;
	//CAT
	int nCATCycleTime;
	int nCATStartIONo;
}T_CARD_PARA;



typedef struct
{
	//补光灯
	int nSupLEDIO;											//补光灯总控IO
	int nSupLED1IO;											//补光灯1IO
	int nSupLED2IO;											//补光灯2IO

	//防飞溅
	int nAirSolenoidValueIO;								//气路电磁阀IO
	int nAntisplashIO;										//防飞溅挡板IO
	int nAntisplashOpenedIO;								//防飞溅挡板打开状态IO
	int nAntisplashClosedIO;								//防飞溅挡板关闭状态IO

	//激光
	int nLaserIO;											//激光IO

	//等离子
	int nPlamsaPowerIO;										//等离子电源
	int nPlamsaArcIO;										//等离子起弧IO
	int nArcFeedbackIO;										//等离子弧反馈IO
	int nSmallArcFeedbackIO;								//小弧反馈IO
}T_CUTTING_ROBOT_IO;

typedef struct
{
}T_HANDLING_ROBOT_IO;

typedef struct
{
	T_YASKAWA_ROBOT_IO tYaskawaRobotIO;
	T_CUTTING_ROBOT_IO tCuttingRobotIO;
	T_HANDLING_ROBOT_IO tHandlingRobotIO;
}T_ROBOT_IO;

class CIOControl  
{
public:
    CIOControl();
    virtual ~CIOControl();

private:
	//初始化操作
	void OpenIoCard();											//初始化IO卡
	void CloseIoCard();											//关闭IO卡

	void LoadIO();												//加载IO口
	bool LoadRobotIO();
	bool CheckBitNo(WORD bitno);								//检查IO号是否有效
	void CheckIOPara();	

    //基本读写IO函数
    DWORD ReadInbit(WORD bitno);
    DWORD ReadOutbit(WORD bitno);

public:
    DWORD WriteOutbit(WORD bitno, WORD on_off);
   
	std::vector<CString> m_vstrCardModel;		//IO卡类型
	std::map<CString, int> m_mapCardModel;		//IO卡类型文件用名指向程序用名
	std::map<int, int> m_mapCardNum;			//SoftwareCardNo指向m_vtCardPara的下标
	std::vector<T_CARD_PARA> m_vtCardPara;		//IO卡信息
	bool m_bIOCCardExists;
	bool m_bDMCCardExists;
	int m_nMagnetizingDelayTime;

public:
	//*************************************************整体*************************************************//
	bool TotalEmg(bool bStopArc, bool bPopup = true);									//急停
	void OpenAirconditioner();															//空调开启
	void CloseAirconditioner();															//空调关闭
	void OpenDustRemoval();																//除尘开启
	void CloseDustRemoval();															//除尘关闭
	void OpenWarningLights();															//警示灯闪烁开启
	void CloseWarningLights();															//警示灯闪烁关闭	

	int m_nAirconditionerIO;															//空调IO
	int m_nDustRemovalIO;																//除尘IO
	int m_nWarningLightsIO;																//警示灯IO
	int m_nTotalEmgIO;																	//设备急停IO

	//************************************************机械臂************************************************//
	void OpenRobot(int nRobotNo);														//打开外部启动
	void CloseRobot(int nRobotNo);														//关闭外部启动
	void OpenCallJob(int nRobotNo);														//打开程序调出IO
	void CloseCallJob(int nRobotNo);													//关闭程序调出IO
	void ResetErrors(int nRobotNo);														//机械臂错误/报警复位
	void ResetExRobotEmg(int nRobotNo);													//重置机械臂外部急停
	void SetExRobotEmg(int nRobotNo);													//机械臂外部急停

	bool CheckRobotMoveSafe();
	int CheckRobotDone(int nRobotNo, int nDelayTime, int nProximity = 0);				//运动结束,0禁用，1有信号停止，2无信号停止,小于0异常
	bool RobotServoOn(int nRobotNo, bool bStopArc, bool bPopup = true);					//机械臂伺服接通中
	bool Errors(int nRobotNo);															//机械臂发出错误/报警	
	bool BatteryWarning(int nRobotNo);													//电池警报
	bool OperationalOrigin(int nRobotNo);												//作业原点
	bool MidwayStart(int nRobotNo);														//可中途启动

	int m_nRobotMoveSafeIO;
	std::vector<T_ROBOT_IO> m_vtRobotIO;												//机械臂IO
	int m_nRobotNum;

	//**********************************************切割机械臂**********************************************//
	//补光灯
	void OpenSupLED(int nRobotNo);											//打开闪光灯总电源
	void CloseSupLED(int nRobotNo);											//关闭闪光灯总电源
	void OpenSupLEDCtrlSignalPower(int nRobotNo);							//打开闪光灯控制信号电源
	void CloseSupLEDCtrlSignalPower(int nRobotNo);							//关闭闪光灯控制信号电源

	//防飞溅
	void OpenAntisplash(int nRobotNo);										//打开防飞溅挡板
	void CloseAntisplash(int nRobotNo);										//关闭防飞溅挡板
	bool GetAntisplashState(int nRobotNo, bool &bState);					//获取防飞溅挡板状态
	void OpenAirSolenoidValue(int nRobotNo);								//气路电磁阀开启
	void CloseAirSolenoidValue(int nRobotNo);								//气路电磁阀关闭

	//激光
	void OpenLaser(int nRobotNo);											//打开激光
	void CloseLaser(int nRobotNo);											//关闭激光

	//等离子
	DWORD OpenPlamsa(int nRobotNo);											//打开等离子
	DWORD ClosePlamsa(int nRobotNo);										//关闭等离子
	DWORD StartPlamsaArc(int nRobotNo);										//起弧
	DWORD StopPlamsaArc(int nRobotNo);										//停弧
	bool ReadPlamsaArc(int nRobotNo);
	bool PlamsaArcFeedback(int nRobotNo);									//弧反馈
	bool SmallArcFeedback(int nRobotNo);									//小弧反馈

	//**********************************************搬运机械臂**********************************************//
	//全景相机
	void OpenPanoramaCamera();									//全景相机开启
	void ClosePanoramaCamera();									//全景相机关闭

	int m_nPanoramaCameraIO;									//全景相机IO

	//电磁铁	
	void Demagnetizing();										//电磁铁退磁
	void Magnetizing(int nLevel);								//电磁铁充磁，可选磁力等级
	void LockMagnet();											//电磁铁上锁
	void UnlockMagnet();										//电磁铁解锁
	bool GrabSuccess(int nNum);									//抓取成功
	bool MagnetizingSuccess();									//充磁成功信号
	bool DemagnetizingSuccess();								//退磁成功信号
	void MagnetizingLv1();										//磁力等级1
	void MagnetizingLv2();										//磁力等级2
	void MagnetizingLv3();										//磁力等级3
	void OpenMagnet2();											//启用电磁铁2	
	void CloseMagnet2();										//关闭电磁铁2

	int m_nMagnat1GrabSuccessIO;								//电磁铁1吸附反馈IO
	int m_nMagnat2GrabSuccessIO;								//电磁铁2吸附反馈IO
	int m_nMagnetizingSuccessIO;								//充磁成功IO
	int m_nDemagnetizingSuccessIO;								//退磁成功IO
	int m_nMagnet2ControlIO;									//电磁铁2控制IO
	int m_nDemagnetizingIO;										//电磁铁退磁IO
	int m_nMagnetUnlockIO;										//解锁退磁锁定IO		
	int m_nMagnetizingLv1IO;									//电磁铁Lv1IO
	int m_nMagnetizingLv2IO;									//电磁铁Lv2IO
	int m_nMagnetizingLv3IO;									//电磁铁Lv3IO

	//**********************************************搬运机械臂**********************************************//
	void OpenCoolingForPolisher();
	void CloseCoolingForPolisher();
	void OpenPolisher();
	void ClosePolisher();
	void OpenMagnetForPolish();
	void CloseMagnetForPolish();

	int m_nCoolingForPolisher;
	int m_nOpenPolisher;
	int m_nMagnetForPolish;

	//**********************************************其它电磁铁**********************************************//
	void PlatformMagnetizing();									//平台电磁铁充磁
	void PlatformDemagnetizing();								//平台电磁铁退磁	
	bool Door1Close();											//门禁1状态反馈
	bool Door2Close();											//门禁2状态反馈
	void OpenDoor1();											//门禁1开启
	void OpenDoor2();											//门禁2开启
	void CloseDoor1();											//门禁1关闭
	void CloseDoor2();											//门禁2关闭
	
	int m_nDoor1CloseIO;										//门禁1IO
	int m_nDoor2CloseIO;										//门禁2IO
	int m_nPlatformMagnatIO;									//平台电磁铁供电IO				
	int m_nDoor1ControlIO;										//门禁1控制IO
	int m_nDoor2ControlIO;										//门禁2控制IO

	//*************************************************平台*************************************************//
	bool PlatformUp();											//平台上升反馈信号
	bool PlatformDown();                                        //平台下降反馈信号
	void PlatformControl(bool bUpOrDown);						//平台磁力控制
	bool OpenMagnetInChamferingTable(int iNo);
	bool CloseMagnetInChamferingTable(int iNo);

	int m_nUpOrDownIO;											//平台磁力控制IO
	int m_nPlatformUpLimitIO;									//平台上升上限IO
	int m_nPlatformDownLimitIO;									//平台下降下限IO
	
	//***********************************************外置相机***********************************************//
	void OpenExCamera();										//外置相机开启
	void CloseExCamera();										//外置相机关闭

	int m_nExCameraIO;											//外置相机IO

	//*********************************************大件坡口专属*********************************************//
	void LoadIOForBiggerPart();
	bool StartPlatform(int nPlatformNo);	
	void StopPlatform(int nPlatformNo);
	void OpenTransducer();
	void CloseTransducer();
	bool GetTransducerState();
	bool GetLightCurtainState();								//光幕反馈，有信号时不能进行上下料操作
	bool PlatformStart(int nPlatformNo);
	bool PlatformDec(int nPlatformNo);
	bool PlatformStop(int nPlatformNo);

	int m_nPlatform1CorotationStartIO;
	int m_nPlatform1CorotationDecIO;
	int m_nPlatform1CorotationStopIO;
	int m_nPlatform2CorotationStartIO;
	int m_nPlatform2CorotationDecIO;
	int m_nPlatform2CorotationStopIO;
	int m_nLightCurtain1IO;
	int m_nLightCurtain2IO;
	int m_nTransducer1ClosedIO;
	int m_nTransducer2ClosedIO;
	int m_nPlatform1MoveIO;
	int m_nPlatform2MoveIO;
	
	int m_nPlatform1CorotationIO;
	int m_nPlatform1ReversalIO;
	int m_nPlatform2CorotationIO;
	int m_nPlatform2ReversalIO;
	int m_nTransducer1OpenIO;
	int m_nTransducer2OpenIO;

	//料台
	bool TableUpIn(int nTableNo);
	bool TableUpOut(int nTableNo);
	bool CheckForHandling(int nTableNo);
	bool CheckForBlanking(int nTableNo);

	HANDLE m_handleTableMove[2];

	int m_nTableUpIn[2];
	int m_nTableUpOut[2];
	int m_nTableDownIn[2];
	int m_nTableDownOut[2];

	int m_nTableUpInLimit[2];
	int m_nTableUpOutLimit[2];
	int m_nTableUpInMiddle[2];
	int m_nTableDownInLimit[2];

	int m_nTableDelayTime;
	int m_nTableTimeLimit;
};

#endif // !defined(AFX_IOCONTROL_H__492E8CFB_AB04_42AC_8954_92B41A9D6719__INCLUDED_)
