#pragma once
#include ".\Apps\PLib\LeisaiCtrl\ServoMotorDriver.h"

class CUnitDriver
{
public:
	CUnitDriver(T_MOTOR_PARAM tMotorParam, CServoMotorDriver *pServoMotorDriver);
	~CUnitDriver();

	T_MOTOR_PARAM GetMotorParam();


	/*******************************  �ŷ����  *******************************/
	//�ŷ�ʹ��
	int SetSevon(bool bEnable);
	bool GetSevon(bool &bEnable);
	//�ŷ�׼��
	bool GetSevonRdy(bool &bSevonRdy);

	/*******************************  �˶����  *******************************/
	//�����˶�
	int PosMove(long lDistance, int nMode, double dSpeed, double dAcc, double dDec, double dSParam = 0.1);//���������˶�,dSpeed��λpulse/s,dAccdDec��λs
	int PosMove(double dDistance, int nMode, double dSpeed, double dAcc, double dDec, double dSParam = 0.1);//����λ���˶�,dSpeed��λmm/s,dAccdDec��λs
	int ContiMove(int nDir, double dSpeed, double dAcc);//���������˶�,dSpeed��λmm/s,dAcc��λs
	bool CheckAxisRun(int *pnError = NULL);//�������״̬
	void CheckAxisDone();//�ȴ��˶�����
	bool CheckAxisDone(double dDistance, double dError = 0.5);//�ȴ��˶�����
	bool CheckAxisDone(long lDistance, long lError = 50);//�ȴ��˶�����

	//ֹͣ���
	int EmgStopAxis();
	int SetDecelStopTime(double dDec);//���ü���ֹͣʱ��,dDec��λs
	int DecelStop();//����ֹͣ

	//��ǰ����
	int GetCurrentPosition(long &lPosition);//��ǰλ��
	int GetCurrentPosition(double &dPosition);
	int SetCurrentPosition(long lPosition);
	int SetCurrentPosition(double dPosition);
	long CoorChange(double dCoor);
	double CoorChange(long lCoor);

	//Ŀ������
	int GetTargetPosition(long &lPosition);//Ŀ��λ��
	int GetTargetPosition(double &dPosition);
	int SetTargetPosition(long lPosition, bool bupdate = false);//bupdate=true ǿ�Ƹı�λ��
	int SetTargetPosition(double dPosition, bool bupdate = false);

	//�ٶ�
	int GetSpeed(long &lSpeed);
	int GetSpeed(double &dSpeed);
	int ChangeSpeed(long lSpeed, double dChangeTime = 0.1);//��;����
	int ChangeSpeed(double dSpeed, double dChangeTime = 0.1);
	double GetMaxSpeed(int nSoftAxisNo);

	// ��ȡ���嵱��
	double GetPulseEquivalent();


	/*******************************  ���Ա�����  *******************************/
	int OpenAbsEncoder();//�򿪱���������
	int CloseAbsEncoder();//�رմ���
	int GetAbsData(double &dAbsData);//�Ӿ���ֵ��������ȡ��ǰ���꣬��ȡʱ�ŷ�������뾲ֹ
	int ResetAbsEncoder();//��λ������


private:

	int m_nSoftAxisNo;
	T_MOTOR_PARAM m_tMotorParam;
	CServoMotorDriver* m_pServoMotorDriver;
};

