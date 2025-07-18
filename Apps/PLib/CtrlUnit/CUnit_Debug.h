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

    //**************************���������**************************//
    //��ȡ����
    T_ROBOT_COORS GetRobotTool(int nToolNo);
    BOOL GetRobotTool(int nToolNo, T_ROBOT_COORS& tTool);

    //���ֹͣ
    BOOL RobotCheckRunning();
    BOOL RobotCheckDone(int nDelayTime = 1000);
    BOOL RobotCheckDone(T_ROBOT_COORS tAimCoor, T_ROBOT_COORS tCompareLimit = T_ROBOT_COORS(1, 1, 1, -1, -1, -1, 1, 1, 1), int nTool = -1);
    BOOL RobotCheckDone(T_ANGLE_PULSE tAimPulse, T_ANGLE_PULSE tCompareLimit = T_ANGLE_PULSE(10, 10, 10, 10, 10, 10, 200, 200, 200));

    //��ȡ����������
    T_ANGLE_PULSE GetRobotPulse();
    T_ROBOT_COORS GetRobotPos(int nToolNo = -1);
    T_ANGLE_PULSE GetCurBaseAxisPulse(T_ANGLE_PULSE tPulse = T_ANGLE_PULSE());
    T_ROBOT_COORS GetCurBaseAxisPos(T_ROBOT_COORS tCoor = T_ROBOT_COORS());

    //�����˶��켣
    bool SetMoveValue(std::vector<T_ROBOT_MOVE_INFO> vtMoveInfo);
    bool SetMoveValue(CRobotMove cMoveInfo);
    int GetMoveStep();
    int CleanMoveStep();

    //CallJob
    void CallJob(CString sJobName, int nExternalType = 0);

    

    //**************************�ŷ�������**************************//
    //�����˶�
    int AbsPosMove(int nUnitMotorNo, double dAbsPosture, double dSpeed, double dAcc, double dDec, double dSParam = 0.1);

    //���ֹͣ
    BOOL MotorCheckRunning(int nUnitMotorNo = 0);
    BOOL MotorCheckDone(int nUnitMotorNo, double dAbsPosture);
    BOOL MotorCheckNoWait(int nUnitMotorNo, double dAbsPosture);
    BOOL MotorCheckDone(int nUnitMotorNo, double dAbsPosture, double dEarlyEndDistance);

    //��ȡ����
    int GetCurrentPosition(int nUnitMotorNo, double& dPosition);

    //IO����


    BOOL RobotBackHome(T_ROBOT_MOVE_SPEED tSpeed = T_ROBOT_MOVE_SPEED(2000, 70, 70));


protected:
    bool m_bEmgStop = false;
    double m_dDebugSpeedRatio = 5.0;
    bool m_bStartDebug = false;
    CDatabaseCtrl m_cDatabaseCtrl;
    void CreateDatabaseTable();


public:
    //**************************���������**************************//
    //��ȡ����
    T_ROBOT_COORS GetRobotCoor();
    //��ȡ����
    T_ROBOT_COORS GetRobotTool_Database(int nToolNo);
    BOOL GetRobotTool_Database(int nToolNo, T_ROBOT_COORS& tTool);

    //���ֹͣ
    BOOL RobotCheckRunning_Database();
    BOOL RobotCheckDone_Database(int nDelayTime = 1000);

    //��ȡ����������

    //�����˶��켣
    bool SetMoveValue_Database(std::vector<T_ROBOT_MOVE_INFO> vtMoveInfo);

    //CallJob
    static UINT ThreadCallJob_Database(void* pParam);//ģ������
    void CallJob_Database();
    int CallJob_Database_MOVL(T_MOVE_VALUE tMoveValue);
    int CallJob_Database_MOVJ(T_MOVE_VALUE tMoveValue);


    //**************************�ŷ�������**************************//
    //�����˶�
    int AbsPosMove_Database(int nUnitMotorNo, double dAbsPosture, double dSpeed, double dAcc, double dDec, double dSParam = 0.1);
    static UINT ThreadPosMove_Database(void* pParam);//ģ������
    void PosMove_Database(CString strDeviceNo);

    //���ֹͣ
    BOOL MotorCheckRunning_Database(int nUnitMotorNo = 0);
    BOOL MotorCheckDone_Database(int nUnitMotorNo, double dAbsPosture);
    BOOL MotorCheckDone_Database(int nUnitMotorNo, double dAbsPosture, double dEarlyEndDistance);

    //��ȡ����
    int GetCurrentPosition_Database(int nUnitMotorNo, double& dPosition);
};


struct T_MOTOR_MOVE
{
    CUnit_Debug* pFather;				//��ָ��
    CString strDeviceNo = "0";

};

#endif