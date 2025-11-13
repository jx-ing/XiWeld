#pragma once
#include ".\Apps\PLib\LeisaiCtrl\ServoMotorDriver.h"

class CUnitDriver
{
public:
	CUnitDriver(T_MOTOR_PARAM tMotorParam, CServoMotorDriver *pServoMotorDriver);
	~CUnitDriver();

	T_MOTOR_PARAM GetMotorParam();


	/*******************************  伺服相关  *******************************/
	//伺服使能
	int SetSevon(bool bEnable);
	bool GetSevon(bool &bEnable);
	//伺服准备
	bool GetSevonRdy(bool &bSevonRdy);

	/*******************************  运动相关  *******************************/
	//单轴运动
	int PosMove(long lDistance, int nMode, double dSpeed, double dAcc, double dDec, double dSParam = 0.1);//单轴脉冲运动,dSpeed单位pulse/s,dAccdDec单位s
	int PosMove(double dDistance, int nMode, double dSpeed, double dAcc, double dDec, double dSParam = 0.1);//单轴位置运动,dSpeed单位mm/s,dAccdDec单位s
	int ContiMove(int nDir, double dSpeed, double dAcc);//单轴连续运动,dSpeed单位mm/s,dAcc单位s
	bool CheckAxisRun(int *pnError = NULL);//检查运行状态
	void CheckAxisDone();//等待运动结束
	bool CheckAxisDone(double dDistance, double dError = 0.5);//等待运动结束
	bool CheckAxisDone(long lDistance, long lError = 50);//等待运动结束

	//停止相关
	int EmgStopAxis();
	int SetDecelStopTime(double dDec);//设置减速停止时间,dDec单位s
	int DecelStop();//减速停止

	//当前坐标
	int GetCurrentPosition(long &lPosition);//当前位置
	int GetCurrentPosition(double &dPosition);
	int SetCurrentPosition(long lPosition);
	int SetCurrentPosition(double dPosition);
	long CoorChange(double dCoor);
	double CoorChange(long lCoor);

	//目标坐标
	int GetTargetPosition(long &lPosition);//目标位置
	int GetTargetPosition(double &dPosition);
	int SetTargetPosition(long lPosition, bool bupdate = false);//bupdate=true 强制改变位置
	int SetTargetPosition(double dPosition, bool bupdate = false);

	//速度
	int GetSpeed(long &lSpeed);
	int GetSpeed(double &dSpeed);
	int ChangeSpeed(long lSpeed, double dChangeTime = 0.1);//中途变速
	int ChangeSpeed(double dSpeed, double dChangeTime = 0.1);
	double GetMaxSpeed(int nSoftAxisNo);

	// 获取脉冲当量
	double GetPulseEquivalent();


	/*******************************  绝对编码器  *******************************/
	int OpenAbsEncoder();//打开编码器串口
	int CloseAbsEncoder();//关闭串口
	int GetAbsData(double &dAbsData);//从绝对值编码器获取当前坐标，获取时伺服电机必须静止
	int ResetAbsEncoder();//复位编码器


private:

	int m_nSoftAxisNo;
	T_MOTOR_PARAM m_tMotorParam;
	CServoMotorDriver* m_pServoMotorDriver;
};

