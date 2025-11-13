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
#include ".\OpenClass\COMM\OPC\OPCParam.h"

#include "open62541.h"

using namespace std;

#pragma comment(lib,"ws2_32.lib")
#pragma comment(lib,"kernel32.lib")

class COPCServer
{
public:
	COPCServer(const char* cOPCServerParamIni = NULL);
	~COPCServer();
	void StartServer();
	void StartRunOPCDaemon();

	bool ReadWorkStationInfo();
	bool WriteWorkStationInfo();
	bool IntToBool(int nVal);
	void SetVal(vector<VarInfo>& vServerVarInfo, char acValName[100], void* pVal);

	int ModifyBooleanVal(bool bValue, const char *pcValName);
	int ModifyIntVal(int iValue, const char *pcValName);
	int ModifyDoubleVal(double dValue, const char *pcValName);
	int ModifyStrVal(std::string sValue, const char *pcValName);
	int ModifyTimeVal(char *sVarName, SYSTEMTIME st);
	//SetSystemTime();

	bool GetVarVal(char *pcVarName, int nVarType, void *pValue); // 通过变量名称获取服务器变量的值
	bool GetVarVal(char *pcNarName, void *pValue);

	static UINT WINAPI ThreadChangeVal(void* pParam);
	int CheckDataChange(VarInfo* pVarInfo);

private:

	std::string WstringToUtf8(const std::wstring& str);
	std::wstring Utf8ToWstring(const std::string& str);
	std::wstring String2WString(const std::string& s);
	
	void AddVariable(UA_Server *server, char *sVarName, SYSTEMTIME st, UA_NodeId *pParentNodeId = NULL, bool bIfcallBack = false, UA_ValueCallback *pCallback = NULL);
	void AddVariable(UA_Server *server, double *pdValue, int nSize, UA_NodeId *pParentNodeId = NULL, bool bIfcallBack = false, UA_ValueCallback *pCallback = NULL);
	void AddVariable(UA_Server *server, int nObjectOrderNumber, char *varName, int nVarType, void *vVal, UA_NodeId *pParentNodeId, bool bIfcallBack = false, UA_ValueCallback *pCallback = NULL);
	void AddObjectNode(UA_Server *server, char *nodeName, UA_NodeId *MotorStatorId);
	void addObjectFromFile(UA_Server *server);

	void addVariable(UA_Server *server, char *sVarName, bool bValue, UA_NodeId *pParentNodeId, bool bIfcallBack, UA_ValueCallback *pCallback);
	void addVariable(UA_Server *server, char *sVarName, int iValue, UA_NodeId *pParentNodeId, bool bIfcallBack, UA_ValueCallback *pCallback);
	void addVariable(UA_Server *server, char *sVarName, double dValue, UA_NodeId *pParentNodeId, bool bIfcallBack, UA_ValueCallback *pCallback);
	void addVariable(UA_Server *server, char *sVarName, char *cValue, UA_NodeId *pParentNodeId, bool bIfcallBack, UA_ValueCallback *pCallback);
	void addVariable(UA_Server* server, char* sVarName, SYSTEMTIME st, UA_NodeId* pParentNodeId, bool bIfcallBack, UA_ValueCallback* pCallback);



	UA_UInt32 addMonitoredItemToBoolVariable(UA_Server *server, UA_NodeId *pTargetNodeId);
	UA_UInt32 addMonitoredItemToInt32Variable(UA_Server *server, UA_NodeId *pTargetNodeId);
	UA_UInt32 addMonitoredItemToDoubleVariable(UA_Server *server, UA_NodeId *pTargetNodeId);
	UA_UInt32 addMonitoredItemToStringVariable(UA_Server *server, UA_NodeId *pTargetNodeId);
	
	//OPC服务器运行线程
	static UINT WINAPI ThreadRunOPCServer(void *pParam);

	bool SetInitVal(VarInfo &varInfo);
	static UINT WINAPI ThreadRunOPCDaemon(void *pParam);
	bool ReadWorkStationParam();

public:

	UA_Server *m_DeviceServer;
	UA_Boolean		m_bRunning;
	int				m_nPort;
	char			m_cServerPort[1024];
	char		m_cOPCServerParamIni[1024];
	char		m_cOPCServerInfoIni[1024];
	bool m_bServerQuit;

}; 


struct T_CONNECT_OPC_THREAD_DATA_SERVER
{
	COPCServer* pFather;				//父指针
	int nThreadID;						//线程ID
	ObjectNodeInfo* ptObjectNodeInfo;	//节点
	VarInfo* ptObjectValInfo;			//变量
};
