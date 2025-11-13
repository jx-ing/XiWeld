#pragma once
#include ".\OpenClass\COMM\OPC\OPCParam.h"

using namespace std;


#define RECEIVE_MAX_CHAR_LENGHT 1024

class COPCClient
{
public:
	COPCClient();
	~COPCClient();
	//开启客户端，连接服务器
	void StartAllClient();
	void StartClient(ObjectNodeInfo &tObjectNodeInfo);
	bool ReconnectionClient(ObjectNodeInfo &tObjectNodeInfo);

	//开启设置变量
	void StartRunOPCDaemon();
	int CheckDataChange(VarInfo * pVarInfo);

	//设置变量
	int ModifyBooleanVal(bool bValue, VarInfo* pVarInfo);
	int ModifyIntVal(int iValue, VarInfo* pVarInfo);
	int ModifyDoubleVal(double dValue, VarInfo* pVarInfo);
	int ModifyStrVal(std::string sValue, VarInfo* pVarInfo);
	int ModifyBooleanVal(bool bValue, const char* pcValName, const char* pcServerIP);
	int ModifyIntVal(int iValue, const char *pcValName, const char *pcServerIP);
	int ModifyDoubleVal(double dValue, const char* pcValName, const char* pcServerIP);
	int ModifyStrVal(std::string sValue, const char* pcValName, const char* pcServerIP);
	int ModifyTime(char *sVarName, SYSTEMTIME st, const char *pcServerIP);

	//获取变量
	bool GetVarVal(char *pcNarName, void *pValue);
	bool GetAllVarVal();
	bool GetVarVal(UA_Client *client, VarInfo * pVarInfo);


	//清空节点数据
	void CleanNodeInfo(vector<ObjectNodeInfo> &vtObjectNodeInfo);
private:
	//添加监视变量
	void AddMonitoredItem(UA_Client* client, int nObjectOrderNumber, char* varName, int nVarType, std::map<CString, UA_NodeId> mTotalNodeID);
	void AddMonitoredItems(UA_Client* client, std::map<int, std::vector<UA_NodeId>> mNodeId);
	UA_UInt32 addMonitoredItemToBoolVariable(UA_Client *client, UA_NodeId *pTargetNodeId);
	UA_UInt32 addMonitoredItemToBoolVariables(UA_Client* client, UA_NodeId** pTargetNodeId, size_t size);
	void addMonitoredItem_BOOL(UA_Client* client, char* sVarName, std::map<CString, UA_NodeId> mTotalNodeID);
	void addMonitoredItems_BOOL(UA_Client* client, std::vector<UA_NodeId> vTotalNodeID);
	UA_UInt32 addMonitoredItemToInt32Variable(UA_Client *client, UA_NodeId *pTargetNodeId);
	void addMonitoredItem_INT(UA_Client* client, char* sVarName, std::map<CString, UA_NodeId> mTotalNodeID);
	UA_UInt32 addMonitoredItemToDoubleVariable(UA_Client *client, UA_NodeId *pTargetNodeId);
	void addMonitoredItem_DOUBLE(UA_Client* client, char* sVarName, std::map<CString, UA_NodeId> mTotalNodeID);
	UA_UInt32 addMonitoredItemToStringVariable(UA_Client *client, UA_NodeId *pTargetNodeId);
	void addMonitoredItem_CHAR(UA_Client* client, char* sVarName, std::map<CString, UA_NodeId> mTotalNodeID);
	

	void addMonitoredItems(UA_Client* client, std::vector<UA_NodeId> vTotalNodeID, UA_Client_DataChangeNotificationCallback callback);
	UA_UInt32 addMonitoredItemVariables(UA_Client* client, UA_NodeId** pTargetNodeId, size_t size, UA_Client_DataChangeNotificationCallback callback);

	//OPC客户端运行线程
	int m_nTreadID = 0;
	static UINT ThreadRunOPCClient(void *pParam);
	void addMonitoredItemFromFile(UA_Client* client, ObjectNodeInfo* vtObjectNodeInfo, std::map<CString, UA_NodeId> mTotalNodeID);
	void addMonitoredItemFromFiles(UA_Client* client, ObjectNodeInfo* vtObjectNodeInfo, std::map<CString, UA_NodeId> mTotalNodeID);
	static UINT ThreadRunOPCDaemon(void *pParam);
	static UINT ThreadChangeVal(void *pParam);

	//加载节点变量
	bool SetInitVal(VarInfo &varInfo);
	std::vector<CString> GetAllOPCFilePath();
	bool ReadWorkStationParam(std::vector<CString> vstrFilePath);
	bool ReadWorkStationParam(CString strFilePath, ObjectNodeInfo &tObjectNodeInfo);
	
public:

	int flagFalse; //创建线程是否出错的标志位
	UA_Client  *m_DeviceClient;
	UA_Boolean		m_bRunning;
	bool m_bServerQuit;
	bool m_bThreadRunOPCDaemon;

	std::map<int, std::vector<UA_NodeId>> m_mNodeId;

};

struct T_CONNECT_OPC_THREAD_DATA_CLIENT
{
	COPCClient *pFather;				//父指针
	int nThreadID;						//线程ID
	ObjectNodeInfo *ptObjectNodeInfo;	//节点
	VarInfo *ptObjectValInfo;			//变量
};


void dataChangeNotificationBoolCallback_Client(UA_Client *client, UA_UInt32 subId, void *subContext, UA_UInt32 monId, void *monitoredItemContext, UA_DataValue *value);
void dataChangeNotificationInt32Callback_Client(UA_Client *client, UA_UInt32 subId, void *subContext, UA_UInt32 monId, void *monitoredItemContext, UA_DataValue *value);
void dataChangeNotificationDoubleCallback_Client(UA_Client *client, UA_UInt32 subId, void *subContext, UA_UInt32 monId, void *monitoredItemContext, UA_DataValue *value);
void dataChangeNotificationStringCallback_Client(UA_Client *client, UA_UInt32 subId, void *subContext, UA_UInt32 monId, void *monitoredItemContext, UA_DataValue *value);


std::string WstringToUtf8(const std::wstring& str);
std::wstring Utf8ToWstring(const std::string& str);
std::wstring String2WString(const std::string& s);
std::string WString2String(const std::wstring& s);

