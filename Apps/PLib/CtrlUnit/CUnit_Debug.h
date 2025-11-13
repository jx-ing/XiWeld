#pragma once
#include "CUnit.h"
#include ".\Apps\PLib\BasicFunc\ChoiceResources.h"

#if ENABLE_UNIT_DEBUG
#include ".\Apps\PLib\Database\CDatabaseCtrl.h"

class CUnit_Debug :
    public CUnit
{
public:
    CUnit_Debug(T_CONTRAL_UNIT tCtrlUnitInfo, CServoMotorDriver* pServoMotorDriver = NULL);
    ~CUnit_Debug();

    bool GetEmgStopState();


    void EmgStop();
    void Rest();

    //**************************机器人相关**************************//
    //获取工具
    T_ROBOT_COORS GetRobotTool(int nToolNo);
    BOOL GetRobotTool(int nToolNo, T_ROBOT_COORS& tTool);

    //检测停止
    BOOL RobotCheckRunning();
    BOOL RobotCheckDone(int nDelayTime = 1000);
    BOOL RobotCheckDone(T_ROBOT_COORS tAimCoor, T_ROBOT_COORS tCompareLimit = T_ROBOT_COORS(1, 1, 1, -1, -1, -1, 1, 1, 1), int nTool = -1);
    BOOL RobotCheckDone(T_ANGLE_PULSE tAimPulse, T_ANGLE_PULSE tCompareLimit = T_ANGLE_PULSE(10, 10, 10, 10, 10, 10, 200, 200, 200));

    //获取机器人坐标
    T_ANGLE_PULSE GetRobotPulse();
    T_ROBOT_COORS GetRobotPos(int nToolNo = -1);
    T_ANGLE_PULSE GetCurBaseAxisPulse(T_ANGLE_PULSE tPulse = T_ANGLE_PULSE());
    T_ROBOT_COORS GetCurBaseAxisPos(T_ROBOT_COORS tCoor = T_ROBOT_COORS());

    //设置运动轨迹
    bool SetMoveValue(std::vector<T_ROBOT_MOVE_INFO> vtMoveInfo);
    bool SetMoveValue(CRobotMove cMoveInfo);
    int GetMoveStep();
    int CleanMoveStep();

    //CallJob
    void CallJob(CString sJobName, int nExternalType = 0);

    

    //**************************伺服电机相关**************************//
    //单轴运动
    int AbsPosMove(int nUnitMotorNo, double dAbsPosture, double dSpeed, double dAcc, double dDec, double dSParam = 0.1);

    //检测停止
    BOOL MotorCheckRunning(int nUnitMotorNo = 0);
    BOOL MotorCheckDone(int nUnitMotorNo, double dAbsPosture);
    BOOL MotorCheckNoWait(int nUnitMotorNo, double dAbsPosture);
    BOOL MotorCheckDone(int nUnitMotorNo, double dAbsPosture, double dEarlyEndDistance);

    //获取坐标
    int GetCurrentPosition(int nUnitMotorNo, double& dPosition);

    //IO操作


    BOOL RobotBackHome(T_ROBOT_MOVE_SPEED tSpeed = T_ROBOT_MOVE_SPEED(2000, 70, 70));


protected:
    bool m_bEmgStop = false;
    double m_dDebugSpeedRatio = 5.0;
    bool m_bStartDebug = false;
    CDatabaseCtrl m_cDatabaseCtrl;
    void CreateDatabaseTable();


public:
    //**************************机器人相关**************************//
    //获取坐标
    T_ROBOT_COORS GetRobotCoor();
    //获取工具
    T_ROBOT_COORS GetRobotTool_Database(int nToolNo);
    BOOL GetRobotTool_Database(int nToolNo, T_ROBOT_COORS& tTool);

    //检测停止
    BOOL RobotCheckRunning_Database();
    BOOL RobotCheckDone_Database(int nDelayTime = 1000);

    //获取机器人坐标

    //设置运动轨迹
    bool SetMoveValue_Database(std::vector<T_ROBOT_MOVE_INFO> vtMoveInfo);

    //CallJob
    static UINT ThreadCallJob_Database(void* pParam);//模拟运行
    void CallJob_Database();
    int CallJob_Database_MOVL(T_MOVE_VALUE tMoveValue);
    int CallJob_Database_MOVJ(T_MOVE_VALUE tMoveValue);


    //**************************伺服电机相关**************************//
    //单轴运动
    int AbsPosMove_Database(int nUnitMotorNo, double dAbsPosture, double dSpeed, double dAcc, double dDec, double dSParam = 0.1);
    static UINT ThreadPosMove_Database(void* pParam);//模拟运行
    void PosMove_Database(CString strDeviceNo);

    //检测停止
    BOOL MotorCheckRunning_Database(int nUnitMotorNo = 0);
    BOOL MotorCheckDone_Database(int nUnitMotorNo, double dAbsPosture);
    BOOL MotorCheckDone_Database(int nUnitMotorNo, double dAbsPosture, double dEarlyEndDistance);

    //获取坐标
    int GetCurrentPosition_Database(int nUnitMotorNo, double& dPosition);
};


struct T_MOTOR_MOVE
{
    CUnit_Debug* pFather;				//父指针
    CString strDeviceNo = "0";

};

#endif