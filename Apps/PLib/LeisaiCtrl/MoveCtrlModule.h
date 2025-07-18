// MoveCtrlModule.h: interface for the CMoveCtrlModule class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_MOVECTRLMODULE_H__6F89151F_1564_4AEC_B08C_2ECA5656300E__INCLUDED_)
#define AFX_MOVECTRLMODULE_H__6F89151F_1564_4AEC_B08C_2ECA5656300E__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000




// const WORD D5800_MAX_AXIS_NO = 7;
// const WORD D5800_LOGIC = 0;
// const WORD D5800_CRD = 0;
// 
// const WORD D2C80_OUTBIT_SERVO_ENABLE = 1;
// 
// const int DMC5800_CONTIMOV_INSTRUCT_NUM = 5000;

#include ".\Apps\PLib\BasicFunc\Const.h"

/*************************** 基本运动控制 ********************************/
class CMoveCtrlModule  
{
public:
	CMoveCtrlModule();
	virtual ~CMoveCtrlModule();

private:
	
public:
    //点位运动:单位：dDis:mm,dMinVel、dMaxVel：mm/min，dAcc、dDec：s
	void                        PosMoveDis
	(
		WORD wAxis, double dDis, WORD wPositionMode, 
		double dMinVel, double dMaxVel, double dAcc, 
		double dDec, BOOL bIfCheckDone = FALSE
	);
    /*
    函数名称:
    void ContiMove(WORD wAxis, WORD wDir,double dMinVel, double dMaxVel, double dAcc, double dDec);
    功能说明:
    单轴执行连续运动运动
    参数说明:
    wDir 0:负向，1:正向;
    dAcc和dDec对于2610为加减速时间，可以取一个很小的数，如0.2，对于2c80为加速度和减速度，尽量取大点，如100000.
    */
    void ContiMoveDis(WORD wAxis, WORD wDir,
        double dMinVel, double dMaxVel, double dAcc, double dDec);

    
	DWORD                       SetPositionDis(WORD wAxis, double dPos);
    double                      GetPositionDis(WORD wAxis);

	void                        GetMachinePositionDis(double &dCurPosX, double &dCurPosY);

	WORD                        CheckDoneDis(WORD wAxis);
	WORD						CheckDoneAllDis(WORD wAxisNum);
	void                        EmgStopDis(); // 所有轴卡急停

	DWORD						ReadInbitDis(WORD wBitNo);
	//void						EmgStop(WORD wCardNo);

	// 多轴软件联动，距离单位：mm(°)，速度单位mm(°)/min，加减速度单位mm(°)/s*min(1秒钟加速到多少mm(°)/min)
    void                        UnitedMoveDis
	(
		WORD wAxisNum, WORD *pwAxis, double *dDist, WORD wPositionMode, 
        double dMinVel, double dMaxVel, double dAcc, double dDec, 
		WORD wCardNo = CONTROL_CARD_NO
	);

public:
    // 单位：dDis : pulse, dMinVel、dMaxVel：pulse / s，dAcc、dDec：s
    /*
    函数名称:
    void PosMove(WORD wAxis, long lDist, WORD wPositionMode, double dMinVel, double dMaxVel, double dAcc, double dDec)
    功能说明:
    单轴执行点位运动
    参数说明:
    lDist 单位脉冲
    wPositionMode 0:相对坐标模式，1:绝对坐标模式;
    dAcc和dDec对于2610为加减速时间，可以取一个很小的数，如0.2，对于2c80为加速度和减速度，尽量取大点，如100000.
    */
    void PosMove(WORD wAxis, long lDist, WORD wPositionMode,
        double dMinVel, double dMaxVel, double dAcc, double dDec);

    /*
    函数名称:
    void ContiMove(WORD wAxis, WORD wDir,double dMinVel, double dMaxVel, double dAcc, double dDec);
    功能说明:
    单轴执行连续运动运动
    参数说明:
    wDir 0:负向，1:正向;
    dAcc和dDec对于2610为加减速时间，可以取一个很小的数，如0.2，对于2c80为加速度和减速度，尽量取大点，如100000.
    */
    void ContiMove(WORD wAxis, WORD wDir,
        double dMinVel, double dMaxVel, double dAcc, double dDec);

    /*
    函数名称:
    long GetPosition(WORD wAxis)
    功能说明:
    读取指定轴的指令脉冲位置
    参数说明:
    wAxis 指定轴号
    返回值:
    指令脉冲位置
    */
    long GetPosition(WORD wAxis);

    /*
    函数名称:
    DWORD SetPosition(WORD wAxis, long lPosition)
    功能说明:
    设置指定轴的指令脉冲位置
    参数说明:
    wAxis 指定轴号
    lPosition 指令脉冲位置，单位pulse
    返回值:
    错误代码
    */
    DWORD SetPosition(WORD wAxis, long lPosition);

    //*************************************************************************************************
    // wLogic：逻辑电平
    // wDir：1  正向 , 2: 负向
    //*************************************************************************************************
    void HomeMove(WORD wAxis, WORD wLogic, WORD wDir, double dVel, double dAcc, double dDec);
    void ZeroMove(WORD wAxis, double dVel, double dAcc, double dDec);

    /*
    函数名称:
    WORD CheckDone(WORD wAxis)
    功能说明:
    检测指定单轴的运动状态
    参数说明:
    wAxis 指定轴号
    返回值:
    0:正在运动; 1:指定轴已停止
    */
    WORD CheckDone(WORD wAxis);

    /*
    函数名称:
    WORD CheckDoneAll(WORD wAxisNum)
    功能说明:
    检测指定数目的单轴的运动状态
    参数说明:
    wAxisNum 指定轴数
    返回值:
    0:至少存在一轴在运动; 1:指定数目的轴已停止
    */
    WORD CheckDoneAll(WORD wAxisNum);

    /*
    函数名称:
    CheckDoneMultiCoor(WORD wCardNo = CONTROL_CARD_NO, WORD wCrd = D5800_CRD)
    功能说明:
    检测坐标系的运动状态(适用于插补运动)
    参数说明:
    wCardNo 指定卡号
    wCrd 指定控制卡上的坐标系号 0~1
    返回值:
    0:正在运行中; 1:正常停止
    备注说明:
    目前支持5800卡的插补运动
    */
    WORD CheckDoneMultiCoor(WORD wCardNo = CONTROL_CARD_NO, WORD wCrd = D5800_CRD);

    /*
    函数名称:
    void MultiCoorStop(WORD wCardNo = CONTROL_CARD_NO, WORD wCrd = D5800_CRD, WORD wStopMode)
    功能说明:
    停止坐标系内所有轴的运动,支持5800控制卡(适用于插补运动)
    参数说明:
    wCardNo 指定卡号
    wCrd 指定控制卡上的坐标系号 0~1
    wStopMode 制动方式 0:减速停止；1:立即停止
    */
    void MultiCoorStop(WORD wStopMode, WORD wCrd = D5800_CRD, WORD wCardNo = CONTROL_CARD_NO);

    /*
    函数名称:
    WORD EmgStop()
    功能说明:
    紧急停止所有轴
    */
    void EmgStop();

    /*
    函数名称:
    WORD EmgStop(WORD wCardNo)
    功能说明:
    紧急停止所有轴，支持5800控制卡
    参数说明:
    wMoveControlCardType 控制卡卡号
    */
    void EmgStop(WORD wMoveControlCardType);

    /*
    函数名称:
    WORD OpenControlCard()
    功能说明:
    初始化控制卡
    返回值说明:
    0:没有找到控制卡，或者控制卡异常;
    1~8:控制卡数;
    */
    WORD OpenControlCard();

    /*
    备注说明:
    5800运动控制卡已经不提供此功能
    */
    WORD OpenControlCard(WORD wMoveControlCardType);

    /*
    函数名称:
    void CloseControlCard()
    功能说明:
    关闭控制卡
    */
    void CloseControlCard();

    /*
    函数名称:
    WORD CloseControlCard(WORD wMoveControlCardType)
    功能说明:
    关闭指定控制卡
    参数说明:
    wMoveControlCardType 控制卡卡号
    */
    void CloseControlCard(WORD wMoveControlCardType);

    /*
    函数名称:
    ConfigELMode(WORD wAxis, WORD wLogic)
    功能说明:
    设置EL限位信号
    参数说明:
    wAxis 轴号
    wLogic 0:正负限位低电平有效；1:正负限位高电平有效；2:正低有效，负高有效；3:正高有效，负低有效
    */
    void ConfigELMode(WORD wAxis, WORD wLogic);

    /*
    函数名称:
    ConfigEmgMode(WORD wEnable, WORD wLogic, WORD wCardNo = CONTROL_CARD_NO,WORD wAxisNum = D5800_MAX_AXIS_NO)
    功能说明:
    设置EMG急停信号
    参数说明:
    wCardNo 控制卡卡号
    wAxisNum 指定待设定轴数量
    wEnable 0:禁止；1:允许
    wLogic 0:低电平有效；1:高电平有效
    */
    void ConfigEmgMode(WORD wEnable, WORD wLogic, WORD wCardNo = CONTROL_CARD_NO, WORD wAxisNum = D5800_MAX_AXIS_NO);

    /*
    函数名称:
    void ConfigAlmMode(WORD wCardNo = CONTROL_CARD_NO, WORD wAxisNum = D5800_MAX_AXIS_NO, WORD wLogic = D5800_LOGIC)
    功能说明:
    设置ALM信号
    备注:
    兼容上一版本接口
    */
    void ConfigAlmMode();

    /*
    函数名称:
    void ConfigAlmMode(WORD wCardNo = CONTROL_CARD_NO, WORD wAxis, WORD wLogic)
    功能说明:
    设置ALM信号
    参数说明:
    wCardNo 控制卡卡号
    wAxis 轴号
    wLogic 0:低电平有效；1:高电平有效
    备注:
    推荐使用此函数
    */
    void ConfigAlmMode(WORD wAxis, WORD wLogic, WORD wCardNo = CONTROL_CARD_NO);

    void LoadPosition();
    void SavePosition();

    /*
    函数名称:
    void SetPulseOutmode(WORD wAxis,WORD wOutmode);
    函数功能:
    设置指定轴的脉冲输出模式
    参数说明:
    wAxis 轴号
    wOutmode 脉冲输出方式选择
    */
    void SetPulseOutmode(WORD wAxis, WORD wOutmode);

    /*
    函数名称:
    DWORD WriteOutbit(WORD wBitNo, WORD wOnOff, WORD wCardType, WORD wCardNo = CONTROL_CARD_NO)
    功能说明:
    设置指定开卡的某个输出端口的电平
    参数说明:
    wCardNo 控制卡卡号
    wBitNo 输出端口号 DM5800为0~15
    wCardType 控制卡类型
    wOnOff 0:输出低电平；1:输出高电平
    返回值说明:
    错误代码
    */
    DWORD WriteOutbit(WORD wBitNo, WORD wOnOff, WORD wCardType, WORD wCardNo = CONTROL_CARD_NO);

    /*
    函数名称:
    DWORD ReadOutbit(WORD wBitNo, WORD wCardType, WORD wCardNo = CONTROL_CARD_NO)
    功能说明:
    设置指定开卡的某个输出端口的电平
    参数说明:
    wCardNo 控制卡卡号
    wBitNo 输出端口号 DM5800为0~15
    返回值说明:
    返回输出端口电平 0:低电平；1:高电平
    */
    DWORD ReadOutbit(WORD wBitNo, WORD wCardNo = CONTROL_CARD_NO);

    /*
    函数名称:
    DWORD ReadOutbit(WORD wBitNo, WORD wCardType, WORD wCardNo = CONTROL_CARD_NO)
    功能说明:
    设置指定开卡的某个输出端口的电平
    参数说明:
    wCardNo 控制卡卡号
    wBitNo 输出端口号 DM5800为0~15
    wCardType 控制卡类型
    返回值说明:
    返回输出端口电平 0:低电平；1:高电平
    */
    DWORD ReadOutbit(WORD wBitNo, WORD wCardType, WORD wCardNo = CONTROL_CARD_NO);

    /*
    函数名称:
    DWORD ReadInbit(WORD wBitNo, WORD wCardNo = CONTROL_CARD_NO)
    功能说明:
    设置指定开卡的某个输出端口的电平
    参数说明:
    wCardNo 控制卡卡号
    wCardType 控制卡类型
    wBitNo 输入端口号 DM5800为0~15
    返回值说明:
    指定输入端口电平 1:高电平 0:低电平
    */
    DWORD ReadInbit(WORD wBitNo, WORD wCardType, WORD wCardNo = CONTROL_CARD_NO);

    DWORD IoStatus(WORD wAxis);

    BOOL CheckEmg(WORD wAxis);

    BOOL CheckAlarm(WORD wAxis);

    BOOL CheckMoveLimitPositive(WORD wAxis);

    BOOL CheckMoveLimitNegative(WORD wAxis);

    BOOL CheckHome(WORD wAxis);

    void EnableServo(WORD wLogic, WORD wCardNo = CONTROL_CARD_NO);
    void EnableServo(WORD wAxis, WORD wLogic, int nServoEnableIo = 1, WORD wCardNo = CONTROL_CARD_NO); // 2C80伺服使能是通过IO口控制的，需要指定IO

    void SetDefaultAcc(double dAcc);
    void SetDefaultDec(double dDec);
    double GetDefaultAcc();
    double GetDefaultDec();

    double GetCurrentSpeed(WORD wAxis);
    DWORD ChangeCurrentSpeed(WORD wAxis, double dVel);

    /*
    函数说明:
    void InterpolationMove2(WORD *pwAxis, long *plDist, WORD wPositionMode,
    double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo = CONTROL_CARD_NO)
    功能说明:
    以脉冲为单位的两轴直线插补运动
    备注说明:
    支持任意2轴插补，有些驱动库版本不支持，并且有些控制卡使用该函数会导致6轴插补给负脉冲时失效
    */
    void InterpolationMove2(WORD *pwAxis, long *plDist, WORD wPositionMode,
        double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo = CONTROL_CARD_NO, WORD wCoordinate = 0);

    // 支持任意3轴插补，有些驱动库版本不支持，并且有些控制卡使用该函数会导致6轴插补给负脉冲时失效
    void InterpolationMove3(WORD *pwAxis, long *plDist, WORD wPositionMode,
        double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo = CONTROL_CARD_NO, WORD wCoordinate = 0);

    // 支持任意4轴插补，d2610驱动不支持，不能用
    void InterpolationMove4(WORD awAxis[], long alDist[], WORD wPositionMode,
        double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo = CONTROL_CARD_NO, WORD wCoordinate = 0);

    // 2c80支持任意6个轴进行插补
    void InterpolationMove6(WORD awAxis[], long *lDist, WORD wPositionMode,
        double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo = CONTROL_CARD_NO, WORD wCoordinate = 0);

    // 2c80支持任意1~12个轴进行插补
    void InterpolationMoveN(WORD wAxisNum, WORD awAxis[], long *lDist, WORD wPositionMode,
        double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo = CONTROL_CARD_NO, WORD wCoordinate = 0);


    //DMC-5800 多轴软件联动，距离单位：mm(°)，速度单位mm(°)/min，加减速度单位mm(°)/s*min(1秒钟加速到多少mm(°)/min)
    void UnitedMove(WORD wAxisNum, WORD *pwAxis, double *dDist, WORD wPositionMode,
        double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo = CONTROL_CARD_NO, WORD wCoordinate = 0);
    //

    /*
    函数说明:
    void InterpolationMoveUnit(WORD wAxisNum, WORD awAxis[], double *dDist, WORD wPositionMode,
    double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo = CONTROL_CARD_NO)
    功能说明:
    以脉冲当量为单位的直线插补运动
    备注说明:
    5800支持任意2~6个轴直线插补;
    需要先使用void SetEquiv(WORD wCardNo = CONTROL_CARD_NO, WORD wAxis, double dEquiv)为各轴设置脉冲当量
    */
    void InterpolationMoveUnit(WORD wAxisNum, WORD awAxis[], double *dDist, WORD wPositionMode,
        double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo = CONTROL_CARD_NO, WORD wCoordinate = 0);

    /* + ************************** 连续插补功能 ******************************** + */
    /*
    函数说明:
    void StartContiInterMove(WORD wAxisNum, WORD *pwAxis, WORD wCardNo = CONTROL_CARD_NO);
    功能说明:
    打开连续插补缓冲区,开始连续插补
    */
    void StartContiInterMove(WORD wAxisNum, WORD *pwAxis, WORD wCardNo = CONTROL_CARD_NO, WORD wCoordinate = 0);

    /*
    函数说明:
    void ContiInterMove(WORD wAxisNum, WORD awAxis[], long *lDist, WORD wPositionMode,
    double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo = CONTROL_CARD_NO);
    功能说明:
    执行连续直线插补运动
    参数说明:
    awAxis 轴号列表
    lDist  目标位置数组，以脉冲为单位
    wPositionMode 运动模式, 0:相对坐标模式 1:绝对坐标模式
    */
    void ContiInterMove(WORD wAxisNum, WORD awAxis[], long *lDist, WORD wPositionMode,
        double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo = CONTROL_CARD_NO, WORD wCoordinate = 0);

    /*
    函数说明:
    BOOL  ContiCheckRemainSpace(WORD wCardNo = CONTROL_CARD_NO);
    功能说明:
    查询连续插补缓冲区剩余插补空间
    返回值说明:
    TRUE 连续插补缓冲区还有剩余
    */
    BOOL  ContiCheckRemainSpace(WORD wCardNo = CONTROL_CARD_NO, WORD wCoordinate = 0);

    /*
    函数说明:
    BOOL  CheckContiInterMoveState(WORD wAxisNum)
    功能说明:
    读取指定坐标系的插补运动状态
    */
    short  CheckContiInterMoveState(WORD CardNo = CONTROL_CARD_NO, WORD wCoordinate = 0);

    /*
    函数说明:
    DWORD ContiInterMovePause(WORD wCardNo = CONTROL_CARD_NO);
    功能说明:
    暂停连续插补,连续插补运动将减速停止
    返回值说明:
    返回错误代码
    */
    DWORD ContiInterMovePause(WORD wCardNo = CONTROL_CARD_NO, WORD wCoordinate = 0);

    /*
    函数说明:
    DWORD ContiInterMoveContinue(WORD wCardNo = CONTROL_CARD_NO);
    功能说明:
    继续运行暂停未完成的连续插补运动
    */
    DWORD ContiInterMoveContinue(WORD wCardNo = CONTROL_CARD_NO, WORD wCoordinate = 0);

    /*
    函数说明:
    DWORD ContiInterMoveDecelStop(WORD wCardNo = CONTROL_CARD_NO);
    功能说明:
    停止插补运动,该函数适用于所有插补运动,并使参与插补的运动轴退出插补模式
    */
    DWORD ContiInterMoveDecelStop(WORD wCardNo = CONTROL_CARD_NO, WORD wCoordinate = 0);

    /*
    函数说明:
    void  ContiInterMoveChangeSpeed(WORD wRatio, WORD wCardNo = CONTROL_CARD_NO)
    功能说明:
    动态调整连续插补速度比例
    备注说明:
    当计算机执行到此指令时，该指令将存入缓冲区，从该指令的下一条运动函数开始运动时起作用
    */
    void  ContiInterMoveChangeSpeed(WORD wRatio, WORD wCardNo = CONTROL_CARD_NO, WORD wCoordinate = 0); // wRatio：预定速度的倍数

	/*
	函数说明:
	DWORD StopContiInterMove(WORD wCardNo = CONTROL_CARD_NO)
	功能说明:
	关闭连续插补缓冲区
	*/
    DWORD StopContiInterMove(WORD wCardNo = CONTROL_CARD_NO, WORD wCoordinate = 0);

    BOOL  CheckContiInterMoveDone(WORD CardNo = CONTROL_CARD_NO);

    void SetBuffNum(int nBuffNum);
    int GetBuffNum();
    // 返回值：1：buffer完全被使用, 0：buffer未使用完
    WORD CheckPreBuff(WORD wPara, WORD wMoveControlCardType); // wPara:对于2C80为卡号，对于2610为轴号

                                                              /* - ************************** 连续插补功能 ******************************** - */

                                                              //dDec 减速停止时间,单位s
                                                              //wStopMode 制动方式 0:减速停止， 1:紧急停止
    void DecelStop(WORD wAxis, double dDec, WORD wStopMode = 0);
    void DecelStopAll(WORD wAxisNum, double dDec, WORD wStopMode = 0);

    WORD GetCardAxisNum(WORD wCardNo = CONTROL_CARD_NO);

    int m_nBuffNum;

//private:
public:

    void InitAxisPara();
    void SetPulseMode(WORD wCardNo = CONTROL_CARD_NO);

    /*************************************************************************************************/
    /*                                         D2C80驱动                                             */
    /*************************************************************************************************/
    // dAcc和dDec，对于2c80为加速度和减速度，尽量取大点，如100000
    void D2C80PosMove(WORD wAxis, long lDist, WORD wPositionMode,
        double dMinVel, double dMaxVel, double dAcc, double dDec);

    void D2C80ContiMove(WORD wAxis, WORD wDir,
        double dMinVel, double dMaxVel, double dAcc, double dDec);

    // 支持同组内任意2轴插补(0~3为第一组，4~5为第二组)，对于D2C80不受组的限制，但是接口保留
    void D2C80InterpolationMove2Group(WORD *pwAxis, long *plDist, WORD wPositionMode,
        double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo = CONTROL_CARD_NO);

    // 支持任意2轴插补，有些驱动库版本不支持，并且有些控制卡使用该函数会导致6轴插补给负脉冲时失效
    void D2C80InterpolationMove2(WORD *pwAxis, long *plDist, WORD wPositionMode,
        double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo = CONTROL_CARD_NO);

    // 支持同组内任意3轴插补(0~3为第一组，4~5为第二组)，对于D2C80不受组的限制，但是接口保留
    void D2C80InterpolationMove3Group(WORD *pwAxis, long *plDist, WORD wPositionMode,
        double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo = CONTROL_CARD_NO);

    // 支持任意3轴插补，有些驱动库版本不支持，并且有些控制卡使用该函数会导致6轴插补给负脉冲时失效
    void D2C80InterpolationMove3(WORD *pwAxis, long *plDist, WORD wPositionMode,
        double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo = CONTROL_CARD_NO);

    // 仅支持前4轴插补
    void D2C80InterpolationMove4(long lDist1, long lDist2, long lDist3, long lDist4, WORD wPositionMode,
        double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo = CONTROL_CARD_NO);
    // 支持任意4轴插补，d2610驱动不支持，不能用
    void D2C80InterpolationMove4(WORD awAxis[], long alDist[], WORD wPositionMode,
        double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo = CONTROL_CARD_NO);

    // 仅支持前6轴插补
    void D2C80InterpolationMove6(long *lDist, WORD wPositionMode,
        double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo = CONTROL_CARD_NO);

    // 2c80支持任意6个轴进行插补
    void D2C80InterpolationMove6(WORD awAxis[], long *lDist, WORD wPositionMode,
        double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo = CONTROL_CARD_NO);

    // D2C80支持任意1~12个轴进行插补
    void D2C80InterpolationMoveN(WORD wAxisNum, WORD awAxis[], long *lDist, WORD wPositionMode,
        double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo = CONTROL_CARD_NO);

    void D2C80StartContiInterMove(WORD wAxisNum, WORD *pwAxis, WORD wCardNo = CONTROL_CARD_NO);
    void D2C80ContiInterMove(WORD wAxisNum, WORD awAxis[], long *lDist, WORD wPositionMode,
        double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo = CONTROL_CARD_NO);
    BOOL  D2C80ContiCheckRemainSpace(WORD wCardNo = CONTROL_CARD_NO);
    DWORD D2C80ContiInterMovePause(WORD wCardNo = CONTROL_CARD_NO);
    DWORD D2C80ContiInterMoveContinue(WORD wCardNo = CONTROL_CARD_NO);
    DWORD D2C80ContiInterMoveDecelStop(WORD wCardNo = CONTROL_CARD_NO);
    void  D2C80ContiInterMoveChangeSpeed(WORD wRatio, WORD wCardNo = CONTROL_CARD_NO); // wRatio：预定速度的倍数
    DWORD D2C80StopContiInterMove(WORD wCardNo = CONTROL_CARD_NO);
    BOOL  D2C80CheckContiInterMoveDone();

    void D2C80DecelStop(WORD wAxis, double dDec);
    void D2C80DecelStopAll(WORD wAxisNum, double dDec);

    long D2C80GetPosition(WORD wAxis);
    DWORD D2C80SetPosition(WORD wAxis, long lPosition);

    double D2C80GetCurrentSpeed(WORD wAxis);
    DWORD D2C80ChangeCurrentSpeed(WORD wAxis, double dVel);

    //*************************************************************************************************
    // wLogic：逻辑电平
    // wDir：1  正向 , 2: 负向
    //*************************************************************************************************
    void D2C80HomeMove(WORD wAxis, WORD wLogic, WORD wDir, double dVel, double dAcc, double dDec);
    void D2C80ZeroMove(WORD wAxis, double dVel, double dAcc, double dDec);

    WORD D2C80CheckDone(WORD wAxis);
    WORD D2C80CheckDoneAll(WORD wAxisNum);
    // 返回值：1：满，0：空
    WORD D2C80CheckPreBuff(WORD wCardNo = CONTROL_CARD_NO);

    void D2C80EmgStop();

    void D2C80ConfigELMode(WORD wAxis, WORD wLogic);
    void D2C80ConfigEmgMode(WORD wEnable, WORD wLogic, WORD wCardNo = CONTROL_CARD_NO);

    void D2C80CloseControlCard();

    WORD D2C80OpenControlCard();

    void D2C80SetPulseOutmode(WORD wAxis, WORD wOutmode);

    DWORD D2C80WriteOutbit(WORD wBitNo, WORD wOnOff, WORD wCardNo = CONTROL_CARD_NO);

    DWORD D2C80ReadInbit(WORD wBitNo, WORD wCardNo = CONTROL_CARD_NO);

    DWORD D2C80IoStatus(WORD wAxis);

    BOOL D2C80CheckEmg(WORD wAxis);

    BOOL D2C80CheckAlarm(WORD wAxis);

    BOOL D2C80CheckMoveLimitPositive(WORD wAxis);

    BOOL D2C80CheckMoveLimitNegative(WORD wAxis);

    BOOL D2C80CheckHome(WORD wAxis);

    void D2C80EnableServo(WORD wAxis, WORD wLogic, WORD wCardNo = CONTROL_CARD_NO);


    /*************************************************************************************************/
    /*                                         D2610驱动                                             */
    /*************************************************************************************************/
    // dAcc和dDec，对于2610为加减速时间，可以取一个很小的数
    void D2610PositionMove(WORD wAxis, long lDist, WORD wPositionMode,
        double dMinVel, double dMaxVel, double dAcc, double dDec);

    void D2610ContiMove(WORD wAxis, WORD wDir,
        double dMinVel, double dMaxVel, double dAcc, double dDec);

    // 支持同组内任意2轴插补(0~3为第一组，4~5为第二组)，对于2C80不受组的限制，但是接口保留
    void D2610InterpolationMove2Group(WORD *pwAxis, long *plDist, WORD wPositionMode,
        double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo = CONTROL_CARD_NO);

    // 支持任意2轴插补，有些驱动库版本不支持，并且有些控制卡使用该函数会导致6轴插补给负脉冲时失效
    void D2610InterpolationMove2(WORD *pwAxis, long *plDist, WORD wPositionMode,
        double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo = CONTROL_CARD_NO);

    // 支持同组内任意3轴插补(0~3为第一组，4~5为第二组)，对于2C80不受组的限制，但是接口保留
    void D2610InterpolationMove3Group(WORD *pwAxis, long *plDist, WORD wPositionMode,
        double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo = CONTROL_CARD_NO);

    // 支持任意3轴插补，有些驱动库版本不支持，并且有些控制卡使用该函数会导致6轴插补给负脉冲时失效
    void D2610InterpolationMove3(WORD *pwAxis, long *plDist, WORD wPositionMode,
        double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo = CONTROL_CARD_NO);

    // 仅支持前4轴插补
    void D2610InterpolationMove4(long lDist1, long lDist2, long lDist3, long lDist4, WORD wPositionMode,
        double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo = CONTROL_CARD_NO);
    // 支持任意4轴插补，d2610驱动不支持，不能用
    void D2610InterpolationMove4(WORD awAxis[], long alDist[], WORD wPositionMode,
        double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo = CONTROL_CARD_NO);

    // 仅支持前6轴插补
    void D2610InterpolationMove6(long *lDist, WORD wPositionMode,
        double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo = CONTROL_CARD_NO);

    BOOL  D2610CheckContiInterMoveDone(WORD wAxisNum);

    void D2610DecelStop(WORD wAxis, double dDec);
    void D2610DecelStopAll(WORD wAxisNum, double dDec);

    long D2610GetPosition(WORD wAxis);
    DWORD D2610SetPosition(WORD wAxis, long lPosition);

    double D2610GetCurrentSpeed(WORD wAxis);
    DWORD D2610ChangeCurrentSpeed(WORD wAxis, double dVel);

    //*************************************************************************************************
    // home_mode：回原点的方法，
    // 1—正方向回原点
    // 2—负方向回原点
    // vel_mode：选择回原点的速度方式，
    // 0—低速回原点
    // 1—高速回原点，遇原点信号，减速后停止
    //*************************************************************************************************
    void D2610HomeMove(WORD wAxis, WORD wLogic, WORD wDir, double dMaxVel, double dAcc, double dDec);
    void D2610ZeroMove(WORD wAxis, double dVel, double dAcc, double dDec);

    WORD D2610CheckDone(WORD wAxis);
    WORD D2610CheckDoneAll(WORD wAxisNum);
    // 返回值：1：满，0：空
    WORD D2610CheckPreBuff(WORD wAxis);

    void D2610EmgStop();

    void D2610ConfigELMode(WORD wAxis, WORD wLogic);
    void D2610ConfigEmgMode(WORD wEnable, WORD wLogic, WORD wCardNo = CONTROL_CARD_NO);

    void D2610CloseControlCard();

    WORD D2610OpenControlCard();

    void D2610SetPulseOutmode(WORD wAxis, WORD wOutmode);

    DWORD D2610WriteOutbit(WORD wBitNo, WORD wOnOff, WORD wCardNo = CONTROL_CARD_NO);

    DWORD D2610ReadInbit(WORD wBitNo, WORD wCardNo = CONTROL_CARD_NO);

    DWORD D2610IoStatus(WORD wAxis);

    BOOL D2610CheckEmg(WORD wAxis);

    BOOL D2610CheckAlarm(WORD wAxis);

    BOOL D2610CheckMoveLimitPositive(WORD wAxis);

    BOOL D2610CheckMoveLimitNegative(WORD wAxis);

    BOOL D2610CheckHome(WORD wAxis);

    void D2610EnableServo(WORD wAxis, WORD wLogic, WORD wCardNo = CONTROL_CARD_NO);



    /*************************************************************************************************/
    /*                                         D4400驱动                                             */
    /*************************************************************************************************/
    // dAcc和dDec，对于4400为加减速时间，可以取一个很小的数
    void D4400PositionMove(WORD wAxis, long lDist, WORD wPositionMode,
        double dMinVel, double dMaxVel, double dAcc, double dDec);

    void D4400ContiMove(WORD wAxis, WORD wDir,
        double dMinVel, double dMaxVel, double dAcc, double dDec);

    // 支持任意2轴插补，有些驱动库版本不支持，并且有些控制卡使用该函数会导致6轴插补给负脉冲时失效
    void D4400InterpolationMove2(WORD *pwAxis, long *plDist, WORD wPositionMode,
        double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo = CONTROL_CARD_NO);

    // 支持任意3轴插补，有些驱动库版本不支持，并且有些控制卡使用该函数会导致6轴插补给负脉冲时失效
    void D4400InterpolationMove3(WORD *pwAxis, long *plDist, WORD wPositionMode,
        double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo = CONTROL_CARD_NO);

    // 仅支持前4轴插补
    void D4400InterpolationMove4(long lDist1, long lDist2, long lDist3, long lDist4, WORD wPositionMode,
        double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo = CONTROL_CARD_NO);

    BOOL  D4400CheckContiInterMoveDone(WORD wAxisNum);

    void D4400DecelStop(WORD wAxis, double dDec);
    void D4400DecelStopAll(WORD wAxisNum, double dDec);

    long D4400GetPosition(WORD wAxis);
    DWORD D4400SetPosition(WORD wAxis, long lPosition);

    double D4400GetCurrentSpeed(WORD wAxis);
    DWORD D4400ChangeCurrentSpeed(WORD wAxis, double dVel);

    //*************************************************************************************************
    // home_mode：回原点的方法，
    // 1—正方向回原点
    // 2—负方向回原点
    // vel_mode：选择回原点的速度方式，
    // 0—低速回原点
    // 1—高速回原点，遇原点信号，减速后停止
    //*************************************************************************************************
    void D4400HomeMove(WORD wAxis, WORD wLogic, WORD wDir, double dMaxVel, double dAcc, double dDec);
    void D4400ZeroMove(WORD wAxis, double dVel, double dAcc, double dDec);

    WORD D4400CheckDone(WORD wAxis);
    WORD D4400CheckDoneAll(WORD wAxisNum);
    // 返回值：1：满，0：空
    WORD D4400CheckPreBuff(WORD wAxis);

    void D4400EmgStop();

    void D4400ConfigELMode(WORD wAxis, WORD wLogic);
    void D4400ConfigEmgMode(WORD wEnable, WORD wLogic, WORD wCardNo = CONTROL_CARD_NO);

    void D4400CloseControlCard();

    WORD D4400OpenControlCard();

    void D4400SetPulseOutmode(WORD wAxis, WORD wOutmode);

    DWORD D4400WriteOutbit(WORD wBitNo, WORD wOnOff, WORD wCardNo = CONTROL_CARD_NO);

    DWORD D4400ReadInbit(WORD wBitNo, WORD wCardNo = CONTROL_CARD_NO);

    DWORD D4400IoStatus(WORD wAxis);

    BOOL D4400CheckEmg(WORD wAxis);

    BOOL D4400CheckAlarm(WORD wAxis);

    BOOL D4400CheckMoveLimitPositive(WORD wAxis);

    BOOL D4400CheckMoveLimitNegative(WORD wAxis);

    BOOL D4400CheckHome(WORD wAxis);

    void D4400EnableServo(WORD wAxis, WORD wLogic, WORD wCardNo = CONTROL_CARD_NO);

    /*************************************************************************************************/
    /*                                         D5800驱动                                             */
    /*************************************************************************************************/
    // dAcc和dDec，对于2c80为加速度和减速度，尽量取大点，如100000
    void D5800PosMove(WORD wCardNo, WORD wAxis, long lDist, WORD wPositionMode,
        double dMinVel, double dMaxVel, double dAcc, double dDec);

    void D5800ContiMove(WORD wCardNo, WORD wAxis, WORD wDir,
        double dMinVel, double dMaxVel, double dAcc, double dDec);

    // 支持任意2轴插补
    //速度 mm/min
    void D5800InterpolationMove2(WORD *pwAxis, double *plDist, WORD wPositionMode,
        double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo = CONTROL_CARD_NO, WORD wCoordinate = 0);

    // 支持任意3轴插补
    void D5800InterpolationMove3(WORD *pwAxis, long *plDist, WORD wPositionMode,
        double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo = CONTROL_CARD_NO, WORD wCoordinate = 0);

    // 支持任意4轴插补
    void D5800InterpolationMove4(WORD *pwAxis, long *plDist, WORD wPositionMode,
        double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo = CONTROL_CARD_NO, WORD wCoordinate = 0);

    // 2c80支持任意6个轴进行插补
    void D5800InterpolationMove6(WORD *pwAxis, long *plDist, WORD wPositionMode,
        double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo = CONTROL_CARD_NO, WORD wCoordinate = 0);

    // D5800支持任意1~12个轴进行插补
    void D5800InterpolationMove(WORD wAxisNum, WORD awAxis[], long *lDist, WORD wPositionMode,
        double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo = CONTROL_CARD_NO, WORD wCoordinate = 0);
    // D5800支持任意2~6个轴进行基于脉冲当量的直线插补
    void D5800InterpolationMoveUnit(WORD wAxisNum, WORD awAxis[], double *dDist, WORD wPositionMode,
        double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo = CONTROL_CARD_NO, WORD wCoordinate = 0);

    void D5800StartContiInterMove(WORD wAxisNum, WORD *pwAxis, WORD wCardNo = CONTROL_CARD_NO, WORD wCoordinate = 0);
    void D5800ContiInterMove(WORD wAxisNum, WORD awAxis[], long *lDist, WORD wPositionMode,
        double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo = CONTROL_CARD_NO, WORD wCoordinate = 0);
    BOOL  D5800ContiCheckRemainSpace(WORD wCardNo = CONTROL_CARD_NO, WORD wCoordinate = 0);
    DWORD D5800ContiInterMovePause(WORD wCardNo = CONTROL_CARD_NO, WORD wCoordinate = 0);
    DWORD D5800ContiInterMoveContinue(WORD wCardNo = CONTROL_CARD_NO, WORD wCoordinate = 0);
    DWORD D5800ContiInterMoveDecelStop(WORD wCardNo = CONTROL_CARD_NO, WORD wCoordinate = 0);
    void  D5800ContiInterMoveChangeSpeed(WORD wRatio, WORD wCardNo = CONTROL_CARD_NO, WORD wCoordinate = 0); // wRatio：预定速度的倍数
    DWORD D5800StopContiInterMove(WORD wCardNo = CONTROL_CARD_NO, WORD wCoordinate = 0);
    BOOL  D5800CheckContiInterMoveDone();

    void D5800DecelStop(WORD wCardNo, WORD wAxis, double dDec, WORD wStopMode);
    void D5800DecelStopAll(WORD wAxisNum, double dDec, WORD wStopMode);

    long D5800GetPosition(WORD wCardNo, WORD wAxis);
    DWORD D5800SetPosition(WORD wCardNo, WORD wAxis, long lPosition);

    double D5800GetCurrentSpeed(WORD wCardNo, WORD wAxis);
    DWORD D5800ChangeCurrentSpeed(WORD wCardNo, WORD wAxis, double dVel);

    short  D5800CheckContRunState(WORD CardNo, WORD wCoordinate = 0);

    //*************************************************************************************************
    // wLogic：逻辑电平
    // wDir：1  正向 , 2: 负向
    //*************************************************************************************************
    void D5800HomeMove(WORD wCardNo, WORD wAxis, WORD wLogic, WORD wDir, double dVel, double dAcc, double dDec);
    void D5800ZeroMove(WORD wCardNo, WORD wAxis, double dVel, double dAcc, double dDec);

    WORD D5800CheckDone(WORD wCardNo, WORD wAxis);
    WORD D5800CheckDoneAll(WORD wAxisNum);
    // 返回值：1：buffer完全被使用, 0：buffer未使用完
    WORD D5800CheckPreBuff(WORD wCardNo = CONTROL_CARD_NO);

    void D5800EmgStop(WORD wCardNo);

    void D5800ConfigELMode(WORD wCardNo, WORD wAxis, WORD wLogic);
    void D5800ConfigEmgMode(WORD wEnable, WORD wAxis, WORD wLogic, WORD wCardNo = CONTROL_CARD_NO);
    void D5800ConfigAlmMode(WORD wCardNo, WORD wAxis, WORD wLogic);

    void D5800CloseControlCard();

    WORD D5800OpenControlCard();

    void D5800SetPulseOutmode(WORD wCardNo, WORD wAxis, WORD wOutmode);

    DWORD D5800WriteOutbit(WORD wBitNo, WORD wOnOff, WORD wCardNo = CONTROL_CARD_NO);
    int   D5800ReadOutbit(WORD wBitNo, WORD wCardNo = CONTROL_CARD_NO);

    DWORD D5800ReadInbit(WORD wBitNo, WORD wCardNo = CONTROL_CARD_NO);

    DWORD D5800IoStatus(WORD wCardNo, WORD wAxis);

    BOOL D5800CheckEmg(WORD wCardNo, WORD wAxis);

    BOOL D5800CheckAlarm(WORD wCardNo, WORD wAxis);

    BOOL D5800CheckMoveLimitPositive(WORD wCardNo, WORD wAxis);

    BOOL D5800CheckMoveLimitNegative(WORD wCardNo, WORD wAxis);

    BOOL D5800CheckHome(WORD wCardNo, WORD wAxis);

    void D5800EnableServo(WORD wAxis, WORD wLogic, WORD wCardNo);

    void D5800SetPulseMode(WORD wCardNo = CONTROL_CARD_NO);
private:
    WORD m_wMoveControlCardType;
    int m_nAxisNoOffset;
    WORD m_wPosFileFlag;
    WORD m_wDefaultAcc;
    WORD m_wDefaultDec;

    typedef struct
    {
        int nPhysicalAxisNo;
        int HomeDir;
        int PulseDir;
        double PulseDisRatio;
        std::string AxisName;
    }T_AXIS_INFO;

    std::map <int, T_AXIS_INFO> m_mapCardAxis;

	double                      m_dUnitedMoveAcc;
	double                      m_dUnitedMoveDec;


};

#endif // !defined(AFX_MOVECTRLMODULE_H__6F89151F_1564_4AEC_B08C_2ECA5656300E__INCLUDED_)
