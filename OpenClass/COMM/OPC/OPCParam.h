#pragma once
#include <iostream>
#include <signal.h>
#include <stdlib.h>
#include <winsock2.h>
#include <string>
#include <locale>
#include <codecvt>
#include <ctime>
#include <windows.h>
#include <process.h>
#include <list>
#include <vector>
#include "open62541.h"
#include ".\Apps\PLib\ExLock\MyCriticalSection.h"

#pragma comment(lib,"ws2_32.lib")
#pragma comment(lib,"kernel32.lib")

using namespace std;

const string OPC_SERVER_FILE_PATH = "ConfigFiles\\";
//const string OPC_SERVER_FILE_PATH = "Data\\";


/*********************************通信数据与参数*********************************/
struct VarInfo
{
	char	varOrgName[100];
	char	varName[100];
	int     nNodeID = 0;
	UA_NodeId	tNodeID;
	int		nVarType;
	void	*pValue;
	int		nArrayLength;
	int     nValueLength;
	char	acServerIP[100];
	bool	bChangeVal;
	union
	{
		bool	bPreValue;
		int		nPreValue;
		int		anPreValue[1024] = {0};
		double	dPreValue;
		char	acPreValue[256];
	};
};
struct ObjectNodeInfo
{
	char				objectName[100];
	int					nOrderNumber;
	int					nNodeID;
	UA_NodeId			tNodeID;
	int					nServerVarSum;
	char				acNodePath[1024];
	char				acIP[50];
	char				acPort[10];
	bool				bTryToConnect = true;
	bool				bConnectState = false;
	bool				bTreadState = false;
	vector<VarInfo>		vServerVarInfo;
};
extern std::vector<ObjectNodeInfo> g_vtObjectNodeInfo;

typedef struct
{
	int nHB_SERVER_ACC;
	int nHB_CLIENT_ACC;
	bool bOPCConnectState = false;
	bool bStartHeartbeatDetectionvoid = false;
	long lLostConnectCount = 0;
	string strNodeName;
	int nNodeNo;
}T_HEARTBEAT; 
extern T_HEARTBEAT* g_atHeartbest;		//心跳数据

typedef struct
{
	bool bAGV_released;
	bool bRobot_released;
	char acPlateCode[1024] = { 0 };
	bool bOccupied;
	bool bWCS_released;
}T_STATION_STATE;


