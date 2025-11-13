#pragma once
#include "Database_mysql.h"
//#include ".\Apps\Handing\PrivateStruct.h"
#if ENABLE_MY_SQL

typedef struct
{
	int nID = 0;
	CString strUnitName;
	CString strDeviceType;
	CString strDeviceNo;
	BOOL bMoveState = FALSE;
	double adCoor[9] = { 0 };
	long alPulse[9] = { 0 };
	int nMoveStep = 0;
}T_DEVICE_STATE;

typedef struct
{
	int nID = 0;
	CString strMoveType;
	CString strMoveDevice;
	CString strUnitName;
	CString strDeviceNo;
	double dSpeed = 0;
	int nStepNo = 0;
	int nValueType = 0;
	long alPulse[9] = { 0 };
	double adCoor[9] = { 0 };
	int nRobotTool = 1;
}T_MOVE_VALUE;

class CDatabaseCtrl :
    public Database_mysql
{
public:
	CDatabaseCtrl(CString strIP, CString strUser, CString strKey, CString strDBName, int nPort);
	CDatabaseCtrl();

	//创建表
	BOOL CreatTable_MoveValue();//运动变量
	BOOL CreatTable_RobotTool();//机器人工具
	BOOL CreatTable_DeviceState();//状态

	//插入数据
	BOOL InsertData_RobotTool(CString strUnitName, int nToolNo, double adTool[6]);
	BOOL InsertData_DeviceState(T_DEVICE_STATE tRobotState);
	BOOL InsertData_MoveValue(T_MOVE_VALUE tMoveValue);


	//读数据
	BOOL ReadData_RobotTool(CString strUnitName, std::map< int, std::vector<double>>& mvdRobotTool);
	BOOL ReadData_DeviceState(T_DEVICE_STATE& tRobotState);
	BOOL ReadData_MoveValue(CString strUnitName, CString strMoveDevice, CString strDeviceNo, std::vector<T_MOVE_VALUE> &vtMoveValue);

	//更新数据
	BOOL UpdateData_DeviceState(T_DEVICE_STATE tRobotState);
	BOOL UpdateData_DeviceState_MoveState(CString strUnitName, CString strDeviceType, CString strDeviceNo, BOOL bMoveState);
	BOOL UpdateData_DeviceState_MoveStep(CString strUnitName, CString strDeviceType, CString strDeviceNo, int nMoveStep);
	BOOL UpdateData_DeviceState_Coor(CString strUnitName, CString strDeviceType, CString strDeviceNo, double adCoor[9], long alPulse[9]);

	//删除数据
	BOOL DeleteData_MoveValue(CString strUnitName, CString strMoveDevice, CString strDeviceNo, std::vector<T_MOVE_VALUE> vtMoveValue);




	CMyCriticalSection m_cDatabaseLock;
};


#endif // ENABLE_MY_SQL
