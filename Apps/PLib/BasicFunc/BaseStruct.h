#pragma once
#include "stdafx.h"
#include <vector>
#include "XiBase.h"

//struct T_ANGLE_PULSE
//{
//	long nSPulse;
//	long nLPulse;
//	long nUPulse;
//	long nRPulse;
//	long nBPulse;
//	long nTPulse;
//	long lBXPulse;
//	long lBYPulse;
//	long lBZPulse;
//
//	T_ANGLE_PULSE()
//	{
//		memset(this, 0, sizeof(T_ANGLE_PULSE));
//	}
//
//	T_ANGLE_PULSE(long nSPulse, long nLPulse, long nUPulse, long nRPulse, long nBPulse, long nTPulse, long lBXPulse, long lBYPulse, long lBZPulse)
//	{
//		this->nSPulse = nSPulse;
//		this->nLPulse = nLPulse;
//		this->nUPulse = nUPulse;
//		this->nRPulse = nRPulse;
//		this->nBPulse = nBPulse;
//		this->nTPulse = nTPulse;
//		this->lBXPulse = lBXPulse;
//		this->lBYPulse = lBYPulse;
//		this->lBZPulse = lBZPulse;
//	}
//};
//
//struct T_ROBOT_COORS
//{
//	double dX;
//	double dY;
//	double dZ;
//	double dRX;
//	double dRY;
//	double dRZ;
//	double dBX;
//	double dBY;
//	double dBZ;
//
//	T_ROBOT_COORS()
//	{
//		memset(this, 0, sizeof(T_ROBOT_COORS));
//	}
//
//	T_ROBOT_COORS(double dX, double dY, double dZ, double dRX, double dRY, double dRZ, double dBX, double dBY, double dBZ)
//	{
//		this->dX = dX;
//		this->dY = dY;
//		this->dZ = dZ;
//		this->dRX = dRX;
//		this->dRY = dRY;
//		this->dRZ = dRZ;
//		this->dBX = dBX;
//		this->dBY = dBY;
//		this->dBZ = dBZ;
//	}
//	T_ROBOT_COORS(double adRobotCoor[6])
//	{
//		this->dX = adRobotCoor[0];
//		this->dY = adRobotCoor[1];
//		this->dZ = adRobotCoor[2];
//		this->dRX = adRobotCoor[3];
//		this->dRY = adRobotCoor[4];
//		this->dRZ = adRobotCoor[5];
//		this->dBX = 0;
//		this->dBY = 0;
//		this->dBZ = 0;
//	}
//
//	T_ROBOT_COORS operator+(T_ROBOT_COORS& tPoint)
//	{
//		T_ROBOT_COORS tResult;
//		tResult.dX = this->dX + tPoint.dX;
//		tResult.dY = this->dY + tPoint.dY;
//		tResult.dZ = this->dZ + tPoint.dZ;
//		tResult.dRX = this->dRX + tPoint.dRX;
//		tResult.dRY = this->dRY + tPoint.dRY;
//		tResult.dRZ = this->dRZ + tPoint.dRZ;
//		tResult.dBX = this->dBX + tPoint.dBX;
//		tResult.dBY = this->dBY + tPoint.dBY;
//		tResult.dBZ = this->dBZ + tPoint.dBZ;
//		return tResult;
//	}
//
//	T_ROBOT_COORS operator*(double dNum)
//	{
//		T_ROBOT_COORS tResult;
//		tResult.dX = this->dX * dNum;
//		tResult.dY = this->dY * dNum;
//		tResult.dZ = this->dZ * dNum;
//		tResult.dRX = this->dRX * dNum;
//		tResult.dRY = this->dRY * dNum;
//		tResult.dRZ = this->dRZ * dNum;
//		tResult.dBX = this->dBX * dNum;
//		tResult.dBY = this->dBY * dNum;
//		tResult.dBZ = this->dBZ * dNum;
//		return tResult;
//	}
//};

struct T_ROBOT_MOVE_SPEED
{
	double dSpeed = 0;
	double dACC = 0;
	double dDEC = 0;

	T_ROBOT_MOVE_SPEED(double dSpeed, double dACC, double dDEC)
	{
		this->dSpeed = dSpeed;
		this->dACC = dACC;
		this->dDEC = dDEC;
	}
	T_ROBOT_MOVE_SPEED()
	{

	}
};

struct E_CHAMFER_TABLE_INFO
{

	int iChamferTableState = 0;	//����̨״̬  0����ʾ���У�1�������У�2����ʾ������ɣ������⣬3�������У�4��������ɣ��ȴ����ϣ�5�������У�6���������
	CString sSteelPlateNo;		//�ְ���ˮ��
	CString sPartNo;		//������
	CString sPartName;		//������
	CString sShipNo;		//����
	CString sSectionNo;		//�ֶκ�
	CString sPartFlow;		//����
	double dLenght = 0;					//��������
	double dWidth = 0;					//�������
	double dThickness = 0;              //�������
	bool bChamferingMark = false;//�Ƿ���
	double dGrabPlateChainPos = 0.0;		//�пػ�ȡ����λ��X
	CString sPartBlankTray;		//���̱��
	T_ROBOT_COORS tDiffCoor;//����̨ƫ����
	int nPartBlankTrayColNo = -1;			//���������б��
	CString sDrawName;		//ͼֽ��
	int nMagnetNum = 0;
	int nPartAngle = 0;
	std::vector<int> viMagnetNo;//ʹ�õ�������
	bool bSendGrabComplate = false;
	bool bSendPlaceComplate = false;
};

//2023/12/21 ���Ӽ�
struct T_ANGLE_THETA
{
	double dThetaS;
	double dThetaL;
	double dThetaU;
	double dThetaR;
	double dThetaB;
	double dThetaT;

	T_ANGLE_THETA()
	{
		memset(this, 0, sizeof(T_ANGLE_THETA));
	}

	T_ANGLE_THETA(double dThetaS, double dThetaL, double dThetaU, double dThetaR, double dThetaB, double dThetaT)
	{
		this->dThetaS = dThetaS;
		this->dThetaL = dThetaL;
		this->dThetaU = dThetaU;
		this->dThetaR = dThetaR;
		this->dThetaB = dThetaB;
		this->dThetaT = dThetaT;
	}
};
