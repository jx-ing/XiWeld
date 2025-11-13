#include "StdAfx.h"
#include "OPCServer.h"


COPCServer::COPCServer(const char* cOPCServerParamIni)
{
	m_bRunning = true;
	m_nPort = 4840;
	m_cServerPort[1024] = { 0 };
	if (cOPCServerParamIni != NULL)
	{
		string str = "\\OpcServerVar.ini";
		strcpy(m_cOPCServerParamIni, (cOPCServerParamIni + str).c_str());
		string str1 = "\\OpcServerVarInfo.ini";
		strcpy(m_cOPCServerInfoIni, (cOPCServerParamIni + str1).c_str());
	}
	else
	{
		strcpy(m_cOPCServerParamIni, "C:\\OpcServerVar.ini");
		strcpy(m_cOPCServerInfoIni, "C:\\OpcServerVarInfo.ini");
	}
	m_bServerQuit = FALSE;
	ReadWorkStationParam();
}

COPCServer::~COPCServer()
{
	m_bRunning = false;
	Sleep(10);
	m_bServerQuit = TRUE;
}

void COPCServer::StartServer()
{
	// OPC 服务器线程
	HANDLE hThreadOPCServer;
	UINT threadOPCServerId;
	hThreadOPCServer = (HANDLE)_beginthreadex(NULL, 0, ThreadRunOPCServer, (void*)this, 0, &threadOPCServerId);
}

void COPCServer::StartRunOPCDaemon()
{
	HANDLE hThreadOPCSendDaemon;

	UINT threadOPCServerId;
	hThreadOPCSendDaemon = (HANDLE)_beginthreadex(NULL, 0, ThreadRunOPCDaemon, (void*)this, 0, &threadOPCServerId);
}

std::string COPCServer::WstringToUtf8(const std::wstring& str)
{
	std::wstring_convert<std::codecvt_utf8<wchar_t> > strCnv;
	return strCnv.to_bytes(str);
}

std::wstring COPCServer::Utf8ToWstring(const std::string& str)
{
	std::wstring_convert< std::codecvt_utf8<wchar_t> > strCnv;
	return strCnv.from_bytes(str);
}

std::wstring COPCServer::String2WString(const std::string& s)
{
	std::string strLocale = setlocale(LC_ALL, "");
	const char* chSrc = s.c_str();
	size_t nDestSize = mbstowcs(NULL, chSrc, 0) + 1;
	wchar_t* wchDest = new wchar_t[nDestSize];
	wmemset(wchDest, 0, nDestSize);
	mbstowcs(wchDest, chSrc, nDestSize);
	std::wstring wstrResult = wchDest;
	delete[]wchDest;
	setlocale(LC_ALL, strLocale.c_str());
	return wstrResult;
}
/*********************************************************************/

void dataChangeNotificationBoolCallback(UA_Server *server, UA_UInt32 monitoredItemId,
	void *monitoredItemContext, const UA_NodeId *nodeId,
	void *nodeContext, UA_UInt32 attributeId,
	const UA_DataValue *value)
{
	UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_SERVER, "BOOL Received Notification");

	UA_NodeId * targetNodeId = (UA_NodeId*)monitoredItemContext;

	if (/*monitoredItemId == monid &&*/ UA_NodeId_equal(nodeId, targetNodeId))
	{
		UA_Boolean currentValue = *(UA_Boolean*)(value->value.data);
		//printf("Int value change :%d\n", currentValue);
		// buffer格式： 名字长	名字	类型	数据长	数据
		//				%4d		%s		%4d		%4d		?

		int nOrderNumber = 0;
		char varName[100] = { 0 };
		strncpy(varName, (char *)nodeId->identifier.string.data, nodeId->identifier.string.length);
#if 0
		int nIndex = string(varName).find_last_of('_'); // 找到最后一个分隔符的位置
		char * pIndex = varName + nIndex + 1;
		varName[nIndex] = '\0'; // 删除分割符后面的字符，只保留名字
#endif
		UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_SERVER, "BOOL Current Value:%s %d\n", varName, currentValue);
		ObjectNodeInfo *pObjectNodeInfo = &g_vtObjectNodeInfo[0];
		for (int nOPCSendVarNo = 0; nOPCSendVarNo < pObjectNodeInfo->nServerVarSum; nOPCSendVarNo++)
		{
			if (strcmp(varName, pObjectNodeInfo->vServerVarInfo[nOPCSendVarNo].varName) == 0)
			{
				bool btemp = *(bool *)(pObjectNodeInfo->vServerVarInfo[nOPCSendVarNo].pValue);
				*(bool *)(pObjectNodeInfo->vServerVarInfo[nOPCSendVarNo].pValue) = currentValue;
//				btemp = *((bool *)(pObjectNodeInfo->vServerVarInfo[nOPCSendVarNo].pValue));
				pObjectNodeInfo->vServerVarInfo[nOPCSendVarNo].bPreValue = currentValue;

				break;
			}
		}
	}
}

UA_UInt32 COPCServer::addMonitoredItemToBoolVariable(UA_Server *server, UA_NodeId *pTargetNodeId)
{
	UA_MonitoredItemCreateResult result;

	UA_MonitoredItemCreateRequest monRequest = UA_MonitoredItemCreateRequest_default(*pTargetNodeId);

	monRequest.requestedParameters.samplingInterval = 100.0; // 100 ms interval
	result = UA_Server_createDataChangeMonitoredItem(server, UA_TIMESTAMPSTORETURN_BOTH,
		monRequest, (void*)pTargetNodeId, dataChangeNotificationBoolCallback);

	if (result.statusCode == UA_STATUSCODE_GOOD)
	{

		UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_SERVER, "Add monitored item for bool variable, OK.");
		return result.monitoredItemId;
	}
	else
	{
		UA_LOG_ERROR(UA_Log_Stdout, UA_LOGCATEGORY_SERVER, "Add monitored item for bool variable, Fail.");
		return -1;
	}
}

void COPCServer::addVariable(UA_Server *server, char *sVarName, bool bValue, UA_NodeId *pParentNodeId, bool bIfcallBack, UA_ValueCallback *pCallback)
{
	/* Define the attribute of the myInteger variable node */
	UA_VariableAttributes attr = UA_VariableAttributes_default;
	UA_Boolean myInteger = bValue;
	UA_Variant_setScalar(&attr.value, &myInteger, &UA_TYPES[UA_TYPES_BOOLEAN]);
	attr.description = UA_LOCALIZEDTEXT("", sVarName);
	attr.displayName = UA_LOCALIZEDTEXT("", sVarName);
	attr.dataType = UA_TYPES[UA_TYPES_BOOLEAN].typeId;
	attr.accessLevel = UA_ACCESSLEVELMASK_READ | UA_ACCESSLEVELMASK_WRITE;

	/* Add the variable node to the information model */
	//static UA_NodeId myIntegerNodeId = UA_NODEID_STRING(1, sVarName);
	UA_NodeId * myIntegerNodeId = new UA_NodeId;
	*myIntegerNodeId = UA_NODEID_STRING(1, sVarName);
	UA_QualifiedName myIntegerName = UA_QUALIFIEDNAME(1, sVarName);
	UA_NodeId parentNodeId;
	if (NULL == pParentNodeId)
	{
		parentNodeId = UA_NODEID_NUMERIC(0, UA_NS0ID_OBJECTSFOLDER);
	}
	else
	{
		parentNodeId = *pParentNodeId;
	}
	UA_NodeId parentReferenceNodeId = UA_NODEID_NUMERIC(0, UA_NS0ID_ORGANIZES); //UA_NODEID_NUMERIC(0, UA_NS0ID_ORGANIZES);//UA_NODEID_NUMERIC(0, UA_NS0ID_BOOLEAN); 
	UA_Server_addVariableNode(server, *myIntegerNodeId, parentNodeId,
		parentReferenceNodeId, myIntegerName,
		UA_NODEID_NUMERIC(0, UA_NS0ID_BASEDATAVARIABLETYPE), attr, NULL, NULL);
//	if (bIfcallBack)
	{
		//UA_Server_setVariableNode_valueCallback(server, myIntegerNodeId, *pCallback);
		addMonitoredItemToBoolVariable(server, myIntegerNodeId);
	}

}

void dataChangeNotificationInt32Callback(UA_Server *server, UA_UInt32 monitoredItemId,
	void *monitoredItemContext, const UA_NodeId *nodeId,
	void *nodeContext, UA_UInt32 attributeId,
	const UA_DataValue *value)
{
	UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_SERVER, "INT32 Received Notification");
	UA_NodeId * targetNodeId = (UA_NodeId*)monitoredItemContext;
	if (/*monitoredItemId == monid &&*/ UA_NodeId_equal(nodeId, targetNodeId))
	{
		int nOrderNumber = 0;
		char varName[100] = { 0 };
		strncpy(varName, (char *)nodeId->identifier.string.data, nodeId->identifier.string.length);

		ObjectNodeInfo *pObjectNodeInfo = &g_vtObjectNodeInfo[0];
		UA_Int32 currentValue = *(UA_Int32*)(value->value.data);
		UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_SERVER, "INT32 Current Value:%s: %d\n", varName, currentValue);
		for (int nOPCSendVarNo = 0; nOPCSendVarNo < pObjectNodeInfo->nServerVarSum; nOPCSendVarNo++)
		{
			if (strcmp(varName, pObjectNodeInfo->vServerVarInfo[nOPCSendVarNo].varName) == 0)
			{
				*(UA_Int32*)(pObjectNodeInfo->vServerVarInfo[nOPCSendVarNo].pValue) = currentValue;
				pObjectNodeInfo->vServerVarInfo[nOPCSendVarNo].nPreValue = currentValue;
				break;
			}
		}
	}
}

UA_UInt32 COPCServer::addMonitoredItemToInt32Variable(UA_Server *server, UA_NodeId *pTargetNodeId)
{
	UA_MonitoredItemCreateResult result;

	UA_MonitoredItemCreateRequest monRequest = UA_MonitoredItemCreateRequest_default(*pTargetNodeId);

	monRequest.requestedParameters.samplingInterval = 100.0; // 100 ms interval
	result = UA_Server_createDataChangeMonitoredItem(server, UA_TIMESTAMPSTORETURN_BOTH,
		monRequest, (void*)pTargetNodeId, dataChangeNotificationInt32Callback);

	if (result.statusCode == UA_STATUSCODE_GOOD)
	{

		UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_SERVER, "Add monitored item for int32 variable, OK.");
		return result.monitoredItemId;
	}
	else
	{
		UA_LOG_ERROR(UA_Log_Stdout, UA_LOGCATEGORY_SERVER, "Add monitored item for int32 variable, Fail.");
		return -1;
	}
}

void COPCServer::addVariable(UA_Server *server, char *sVarName, int iValue, UA_NodeId *pParentNodeId, bool bIfcallBack, UA_ValueCallback *pCallback)
{
	/* Define the attribute of the myInteger variable node */
	UA_VariableAttributes attr = UA_VariableAttributes_default;
	UA_Int32 myInteger = iValue;
	UA_Variant_setScalar(&attr.value, &myInteger, &UA_TYPES[UA_TYPES_INT32]);
	attr.description = UA_LOCALIZEDTEXT("", sVarName);
	attr.displayName = UA_LOCALIZEDTEXT("", sVarName);
	attr.dataType = UA_TYPES[UA_TYPES_INT32].typeId;
	attr.accessLevel = UA_ACCESSLEVELMASK_READ | UA_ACCESSLEVELMASK_WRITE;

	/* Add the variable node to the information model */
	//UA_NodeId myIntegerNodeId = UA_NODEID_STRING(1, sVarName);
	UA_NodeId * myIntegerNodeId = new UA_NodeId;
	*myIntegerNodeId = UA_NODEID_STRING(1, sVarName);
	UA_QualifiedName myIntegerName = UA_QUALIFIEDNAME(1, sVarName);
	UA_NodeId parentNodeId;
	if (NULL == pParentNodeId)
	{
		parentNodeId = UA_NODEID_NUMERIC(0, UA_NS0ID_OBJECTSFOLDER);
	}
	else
	{
		parentNodeId = *pParentNodeId;
	}
	UA_NodeId parentReferenceNodeId = UA_NODEID_NUMERIC(0, UA_NS0ID_ORGANIZES);
	UA_Server_addVariableNode(server, *myIntegerNodeId, parentNodeId,
		parentReferenceNodeId, myIntegerName,
		UA_NODEID_NUMERIC(0, UA_NS0ID_BASEDATAVARIABLETYPE), attr, NULL, NULL);
	if (bIfcallBack)
	{
		//UA_Server_setVariableNode_valueCallback(server, myIntegerNodeId, *pCallback);
		addMonitoredItemToInt32Variable(server, myIntegerNodeId);
	}

}

void dataChangeNotificationDoubleCallback(UA_Server *server, UA_UInt32 monitoredItemId,
	void *monitoredItemContext, const UA_NodeId *nodeId,
	void *nodeContext, UA_UInt32 attributeId,
	const UA_DataValue *value)
{
	UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_SERVER, "Double Received Notification");

	UA_NodeId * targetNodeId = (UA_NodeId*)monitoredItemContext;

	if (/*monitoredItemId == monid &&*/ UA_NodeId_equal(nodeId, targetNodeId))
	{

		int nOrderNumber = 0;
		char varName[100] = { 0 };
		strncpy(varName, (char *)nodeId->identifier.string.data, nodeId->identifier.string.length);

		UA_Double currentValue = *(UA_Double*)(value->value.data);
		ObjectNodeInfo *pObjectNodeInfo = &g_vtObjectNodeInfo[0];
		for (int nOPCSendVarNo = 0; nOPCSendVarNo < pObjectNodeInfo->nServerVarSum; nOPCSendVarNo++)
		{
			if (strcmp(varName, pObjectNodeInfo->vServerVarInfo[nOPCSendVarNo].varName) == 0)
			{
				*(UA_Double*)(pObjectNodeInfo->vServerVarInfo[nOPCSendVarNo].pValue) = currentValue;
				pObjectNodeInfo->vServerVarInfo[nOPCSendVarNo].dPreValue = currentValue;
				break;
			}
		}
	}
}

UA_UInt32 COPCServer::addMonitoredItemToDoubleVariable(UA_Server *server, UA_NodeId *pTargetNodeId)
{
	UA_MonitoredItemCreateResult result;

	UA_MonitoredItemCreateRequest monRequest = UA_MonitoredItemCreateRequest_default(*pTargetNodeId);

	monRequest.requestedParameters.samplingInterval = 100.0; // 100 ms interval
	result = UA_Server_createDataChangeMonitoredItem(server, UA_TIMESTAMPSTORETURN_BOTH,
		monRequest, (void*)pTargetNodeId, dataChangeNotificationDoubleCallback);

	if (result.statusCode == UA_STATUSCODE_GOOD)
	{

		UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_SERVER, "Add monitored item for Double variable, OK.");
		return result.monitoredItemId;
	}
	else
	{
		UA_LOG_ERROR(UA_Log_Stdout, UA_LOGCATEGORY_SERVER, "Add monitored item for Double variable, Fail.");
		return -1;
	}
}

void COPCServer::addVariable(UA_Server *server, char *sVarName, double dValue, UA_NodeId *pParentNodeId, bool bIfcallBack, UA_ValueCallback *pCallback)
{
	/* Define the attribute of the myInteger variable node */
	UA_VariableAttributes attr = UA_VariableAttributes_default;
	UA_Double myInteger = dValue;
	UA_Variant_setScalar(&attr.value, &myInteger, &UA_TYPES[UA_TYPES_DOUBLE]);
	attr.description = UA_LOCALIZEDTEXT("", sVarName);
	attr.displayName = UA_LOCALIZEDTEXT("", sVarName);
	attr.dataType = UA_TYPES[UA_TYPES_DOUBLE].typeId;
	attr.accessLevel = UA_ACCESSLEVELMASK_READ | UA_ACCESSLEVELMASK_WRITE;

	/* Add the variable node to the information model */
	/*	static UA_NodeId myIntegerNodeId = UA_NODEID_STRING(1, sVarName);*/
	UA_NodeId * myIntegerNodeId = new UA_NodeId;
	*myIntegerNodeId = UA_NODEID_STRING(1, sVarName);
	UA_QualifiedName myIntegerName = UA_QUALIFIEDNAME(1, sVarName);
	UA_NodeId parentNodeId;
	if (NULL == pParentNodeId)
	{
		parentNodeId = UA_NODEID_NUMERIC(0, UA_NS0ID_OBJECTSFOLDER);
	}
	else
	{
		parentNodeId = *pParentNodeId;
	}
	UA_NodeId parentReferenceNodeId = UA_NODEID_NUMERIC(0, UA_NS0ID_ORGANIZES);
	UA_Server_addVariableNode(server, *myIntegerNodeId, parentNodeId,
		parentReferenceNodeId, myIntegerName,
		UA_NODEID_NUMERIC(0, UA_NS0ID_BASEDATAVARIABLETYPE), attr, NULL, NULL);
	if (bIfcallBack)
	{
		//UA_Server_setVariableNode_valueCallback(server, myIntegerNodeId, *pCallback);
		addMonitoredItemToDoubleVariable(server, myIntegerNodeId);
	}
}

void dataChangeNotificationStringCallback(UA_Server* server, UA_UInt32 monitoredItemId,
	void* monitoredItemContext, const UA_NodeId* nodeId,
	void* nodeContext, UA_UInt32 attributeId,
	const UA_DataValue* value)
{
	UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_SERVER, "String Received Notification");

	UA_NodeId* targetNodeId = (UA_NodeId*)monitoredItemContext;

	if (/*monitoredItemId == monid &&*/ UA_NodeId_equal(nodeId, targetNodeId))
	{
		UA_String currentValue = *(UA_String*)(value->value.data);

		int nOrderNumber = 0;
		char varName[100] = { 0 };
		strncpy(varName, (char*)nodeId->identifier.string.data, nodeId->identifier.string.length);
		//WriteLog("dataChangeNotificationStringCallback string name:%s data:%s, length:%d", varName, currentValue.data, currentValue.length);

		ObjectNodeInfo* pObjectNodeInfo = &g_vtObjectNodeInfo[0];
//		UA_DateTime;
		for (int nOPCSendVarNo = 0; nOPCSendVarNo < pObjectNodeInfo->nServerVarSum; nOPCSendVarNo++)
		{
			if (strcmp(varName, pObjectNodeInfo->vServerVarInfo[nOPCSendVarNo].varName) == 0)
			{
				//if (currentValue.length < sizeof(pObjectNodeInfo->vServerVarInfo[nOPCSendVarNo].pValue))
				if (currentValue.length < 256)
				{
					pObjectNodeInfo->vServerVarInfo[nOPCSendVarNo].nValueLength = currentValue.length;
					strncpy((char*)(pObjectNodeInfo->vServerVarInfo[nOPCSendVarNo].pValue), (char*)currentValue.data, currentValue.length);
					((char*)(pObjectNodeInfo->vServerVarInfo[nOPCSendVarNo].pValue))[currentValue.length] = '\0';
					strncpy((char*)(pObjectNodeInfo->vServerVarInfo[nOPCSendVarNo].acPreValue), (char*)currentValue.data, currentValue.length);
					pObjectNodeInfo->vServerVarInfo[nOPCSendVarNo].acPreValue[currentValue.length] = '\0';
					//WriteLog("strncpy string name:%s data:%s, length:%d", varName, (char *)(pObjectNodeInfo->vServerVarInfo[nOPCSendVarNo].pValue, sizeof((char *)(pObjectNodeInfo->vServerVarInfo[nOPCSendVarNo].pValue))));
					//*(char *)(pObjectNodeInfo->vServerVarInfo[nOPCSendVarNo].pValue) = '\0';
					break;
				}
				//				else
				{
					//UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_SERVER, "string超出字符串长度 实际:%d 允许:%d\n ", nLength, sizeof(pObjectNodeInfo->vServerSendVarInfo[nOPCSendVarNo].acValue));
				}
			}
		}
	}
}

UA_UInt32 COPCServer::addMonitoredItemToStringVariable(UA_Server *server, UA_NodeId *pTargetNodeId)
{
	UA_MonitoredItemCreateResult result;

	UA_MonitoredItemCreateRequest monRequest = UA_MonitoredItemCreateRequest_default(*pTargetNodeId);

	monRequest.requestedParameters.samplingInterval = 100.0; // 100 ms interval
	result = UA_Server_createDataChangeMonitoredItem(server, UA_TIMESTAMPSTORETURN_BOTH,
		monRequest, (void*)pTargetNodeId, dataChangeNotificationStringCallback);

	if (result.statusCode == UA_STATUSCODE_GOOD)
	{

		UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_SERVER, "Add monitored item for String variable, OK.");
		return result.monitoredItemId;
	}
	else
	{
		UA_LOG_ERROR(UA_Log_Stdout, UA_LOGCATEGORY_SERVER, "Add monitored item for String variable, Fail.");
		return -1;
	}
}

void COPCServer::addVariable(UA_Server *server, char *sVarName, char *cValue, UA_NodeId *pParentNodeId, bool bIfcallBack, UA_ValueCallback *pCallback)
{
	/* Define the attribute of the myInteger variable node */
	UA_VariableAttributes attr = UA_VariableAttributes_default;
	UA_String myString = UA_STRING(cValue);
	UA_Variant_setScalar(&attr.value, &myString, &UA_TYPES[UA_TYPES_STRING]);
	attr.description = UA_LOCALIZEDTEXT("", sVarName);
	attr.displayName = UA_LOCALIZEDTEXT("", sVarName);
	attr.dataType = UA_TYPES[UA_TYPES_STRING].typeId;
	attr.accessLevel = UA_ACCESSLEVELMASK_READ | UA_ACCESSLEVELMASK_WRITE;

	/* Add the variable node to the information model */
	UA_NodeId * myIntegerNodeId = new UA_NodeId;
	*myIntegerNodeId = UA_NODEID_STRING(1, sVarName);
	UA_QualifiedName myIntegerName = UA_QUALIFIEDNAME(1, sVarName);
	UA_NodeId parentNodeId;
	if (NULL == pParentNodeId)
	{
		parentNodeId = UA_NODEID_NUMERIC(0, UA_NS0ID_OBJECTSFOLDER);
	}
	else
	{
		parentNodeId = *pParentNodeId;
	}
	UA_NodeId parentReferenceNodeId = UA_NODEID_NUMERIC(0, UA_NS0ID_ORGANIZES);
	UA_Server_addVariableNode(server, *myIntegerNodeId, parentNodeId,
		parentReferenceNodeId, myIntegerName,
		UA_NODEID_NUMERIC(0, UA_NS0ID_BASEDATAVARIABLETYPE), attr, NULL, NULL);
//	if (bIfcallBack)
	{
		//UA_Server_setVariableNode_valueCallback(server, myIntegerNodeId, *pCallback);
		addMonitoredItemToStringVariable(server, myIntegerNodeId);
	}
}

void COPCServer::addVariable(UA_Server* server, char* sVarName, SYSTEMTIME st, UA_NodeId* pParentNodeId, bool bIfcallBack, UA_ValueCallback* pCallback)
{
	UA_VariableAttributes attr = UA_VariableAttributes_default;

	UA_DateTimeStruct dtStruct;
	memset(&dtStruct, 0, sizeof(dtStruct));
	dtStruct.year = st.wYear;
	dtStruct.month = st.wMonth;
	dtStruct.day = st.wDay;
	dtStruct.hour = st.wHour;
	dtStruct.min = st.wMinute;
	dtStruct.sec = st.wSecond;
	dtStruct.milliSec = st.wMilliseconds;
	UA_DateTime dt = UA_DateTime_fromStruct(dtStruct);

	UA_Variant_setScalar(&attr.value, &dt, &UA_TYPES[UA_TYPES_DATETIME]);
	attr.description = UA_LOCALIZEDTEXT("", sVarName);
	attr.displayName = UA_LOCALIZEDTEXT("", sVarName);
	attr.dataType = UA_TYPES[UA_TYPES_DATETIME].typeId;
	attr.accessLevel = UA_ACCESSLEVELMASK_READ | UA_ACCESSLEVELMASK_WRITE;

	/* Add the variable node to the information model */
	UA_NodeId myIntegerNodeId = UA_NODEID_STRING(1, sVarName);
	UA_QualifiedName myIntegerName = UA_QUALIFIEDNAME(1, sVarName);
	UA_NodeId parentNodeId;
	if (NULL == pParentNodeId)
	{
		parentNodeId = UA_NODEID_NUMERIC(0, UA_NS0ID_OBJECTSFOLDER);
	}
	else
	{
		parentNodeId = *pParentNodeId;
	}
	UA_NodeId parentReferenceNodeId = UA_NODEID_NUMERIC(0, UA_NS0ID_ORGANIZES);
	UA_Server_addVariableNode(server, myIntegerNodeId, parentNodeId,
		parentReferenceNodeId, myIntegerName,
		UA_NODEID_NUMERIC(0, UA_NS0ID_BASEDATAVARIABLETYPE), attr, NULL, NULL);
}

void COPCServer::AddVariable(UA_Server *server, char *sVarName, SYSTEMTIME st, UA_NodeId *pParentNodeId, bool bIfcallBack, UA_ValueCallback *pCallback)
{
	/* Define the attribute of the myInteger variable node */
	UA_VariableAttributes attr = UA_VariableAttributes_default;

	UA_DateTimeStruct dtStruct;
	memset(&dtStruct, 0, sizeof(dtStruct));
	dtStruct.year = st.wYear;
	dtStruct.month = st.wMonth;
	dtStruct.day = st.wDay;
	dtStruct.hour = st.wHour;
	dtStruct.min = st.wMinute;
	dtStruct.sec = st.wSecond;
	dtStruct.milliSec = st.wMilliseconds;
	UA_DateTime dt = UA_DateTime_fromStruct(dtStruct);

	UA_Variant_setScalar(&attr.value, &dt, &UA_TYPES[UA_TYPES_DATETIME]);
	attr.description = UA_LOCALIZEDTEXT("", sVarName);
	attr.displayName = UA_LOCALIZEDTEXT("", sVarName);
	attr.dataType = UA_TYPES[UA_TYPES_DATETIME].typeId;
	attr.accessLevel = UA_ACCESSLEVELMASK_READ | UA_ACCESSLEVELMASK_WRITE;

	/* Add the variable node to the information model */
	UA_NodeId myIntegerNodeId = UA_NODEID_STRING(1, sVarName);
	UA_QualifiedName myIntegerName = UA_QUALIFIEDNAME(1, sVarName);
	UA_NodeId parentNodeId;
	if (NULL == pParentNodeId)
	{
		parentNodeId = UA_NODEID_NUMERIC(0, UA_NS0ID_OBJECTSFOLDER);
	}
	else
	{
		parentNodeId = *pParentNodeId;
	}
	UA_NodeId parentReferenceNodeId = UA_NODEID_NUMERIC(0, UA_NS0ID_ORGANIZES);
	UA_Server_addVariableNode(server, myIntegerNodeId, parentNodeId,
		parentReferenceNodeId, myIntegerName,
		UA_NODEID_NUMERIC(0, UA_NS0ID_BASEDATAVARIABLETYPE), attr, NULL, NULL);
}

void COPCServer::AddVariable(UA_Server *server, double *pdValue, int nSize, UA_NodeId *pParentNodeId, bool bIfcallBack, UA_ValueCallback *pCallback)
{
	UA_NodeId pointTypeId;
	UA_VariableTypeAttributes vtAttr = UA_VariableTypeAttributes_default;
	vtAttr.dataType = UA_TYPES[UA_TYPES_DOUBLE].typeId;
	vtAttr.valueRank = 1;
	UA_UInt32 arrayDims[1] = { (UA_UInt32)nSize };
	vtAttr.arrayDimensions = arrayDims;
	vtAttr.arrayDimensionsSize = 1;
	vtAttr.displayName = UA_LOCALIZEDTEXT("en-US", "Double Array");
	UA_Server_addVariableTypeNode(server, UA_NODEID_NULL,
		UA_NODEID_NUMERIC(0, UA_NS0ID_BASEDATAVARIABLETYPE),
		UA_NODEID_NUMERIC(0, UA_NS0ID_HASSUBTYPE),
		UA_QUALIFIEDNAME(1, "Double Array"),
		UA_NODEID_NULL, vtAttr, NULL, &pointTypeId);

	UA_VariableAttributes vAttr = UA_VariableAttributes_default;
	vAttr.dataType = UA_TYPES[UA_TYPES_DOUBLE].typeId;
	vAttr.valueRank = 1;
	UA_UInt32 arrayDims1[1] = { (UA_UInt32)nSize };
	vAttr.arrayDimensions = arrayDims1;
	vAttr.arrayDimensionsSize = 1;
	UA_Double *pUADValue = new UA_Double(nSize); // 创建长度为nSize的数组
	for (int i = 0; i < nSize; i++)
	{
		pUADValue[i] = pdValue[i];
	}
	UA_Variant_setArray(&vAttr.value, pUADValue, nSize, &UA_TYPES[UA_TYPES_DOUBLE]);
	vAttr.displayName = UA_LOCALIZEDTEXT("en-US", "Double Array");
	vAttr.accessLevel = UA_ACCESSLEVELMASK_READ | UA_ACCESSLEVELMASK_WRITE;
	UA_NodeId currentNodeId = UA_NODEID_STRING(1, "Double Array");
	UA_NodeId parentNodeId;
	if (pParentNodeId == NULL)
	{
		parentNodeId = UA_NODEID_NUMERIC(0, UA_NS0ID_OBJECTSFOLDER);
	}
	else
	{
		parentNodeId = *pParentNodeId;
	}
	UA_Server_addVariableNode(server, currentNodeId,
		parentNodeId/*UA_NODEID_NUMERIC(0, UA_NS0ID_OBJECTSFOLDER)*/,
		UA_NODEID_NUMERIC(0, UA_NS0ID_HASCOMPONENT),
		UA_QUALIFIEDNAME(1, "Double Array"), pointTypeId,
		vAttr, NULL, NULL);

	//UA_ValueCallback callback;
	//callback.onRead = beforeReadarrtype;
	//callback.onWrite = afterWritearrtype;
	//UA_Server_setVariableNode_valueCallback(server, currentNodeId, callback);
}

void COPCServer::AddVariable(UA_Server *server, int nObjectOrderNumber, char *varName, int nVarType, void *vVal, UA_NodeId *pParentNodeId, bool bIfcallBack, UA_ValueCallback *pCallback)
{
	/*
	#Boolean 			#define UA_TYPES_BOOLEAN 			0
	#Int32 				#define UA_TYPES_INT32 				5
	#Double 			#define UA_TYPES_DOUBLE 			10
	#String 			#define UA_TYPES_STRING 			11
	#DateTime 			#define UA_TYPES_DATETIME 			12*/
	switch (nVarType)
	{
		case 0:addVariable(server, varName, *(bool *)vVal, pParentNodeId, bIfcallBack, pCallback); break;
		case 5:addVariable(server, varName, *(int *)vVal, pParentNodeId, bIfcallBack, pCallback); break;
		case 10:addVariable(server,varName, *(double *)vVal, pParentNodeId, bIfcallBack, pCallback); break;
		case 11:addVariable(server,varName, (char *)vVal, pParentNodeId, bIfcallBack, pCallback); break;
		case 12:SYSTEMTIME st;
				GetLocalTime(&st);
				AddVariable(server, varName, st, pParentNodeId); break;
		default:printf("不支持该类型变量创建：nVarType = %d\n", nVarType); break;
	}
}

int COPCServer::ModifyBooleanVal(bool bValue, const char *pcValName)
{
	char nodeIdName[1024] = { 0 };
	strcpy(nodeIdName, pcValName);
	UA_Client *client = UA_Client_new();
	UA_ClientConfig_setDefault(UA_Client_getConfig(client));
	UA_StatusCode retval = UA_Client_connect(client, m_cServerPort/*"opc.tcp://localhost:4840"*/);
	if (retval != UA_STATUSCODE_GOOD) {
		UA_Client_delete(client);
		return (int)retval;
	}

	UA_Variant value;
	UA_Variant_init(&value);

	// 变量的Node Id是string类型
	const UA_NodeId nodeId = UA_NODEID_STRING(1, nodeIdName);

	//使用UA_Client_readValueAttribute()去读取变量值
	retval = UA_Client_readValueAttribute(client, nodeId, &value);

	// 变量的数据类型是UA_Int32，第二个是判断UA_Variant里是否有UA_Int32类型的数据
	if (retval == UA_STATUSCODE_GOOD && UA_Variant_hasScalarType(&value, &UA_TYPES[UA_TYPES_BOOLEAN]))
	{
		UA_Boolean variableValue = *(UA_Boolean *)value.data;
		UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_USERLAND, "Variable Value is: %d\n", variableValue);
	}

	// 对server里的变量写入一个新值
	UA_Boolean change = bValue;
	UA_Variant newValue;
	UA_Variant_init(&newValue);
	UA_Variant_setScalar(&newValue, &change, &UA_TYPES[UA_TYPES_BOOLEAN]);
	retval = UA_Client_writeValueAttribute(client, nodeId, &newValue);

	// 重新读取变量值来验证是否写成功
	if (retval == UA_STATUSCODE_GOOD)
	{
		retval = UA_Client_readValueAttribute(client, nodeId, &value);
		if (retval == UA_STATUSCODE_GOOD && UA_Variant_hasScalarType(&value, &UA_TYPES[UA_TYPES_BOOLEAN]))
		{
			UA_Boolean variableValue = *(UA_Boolean *)value.data;
			UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_USERLAND, "New Variable Value is: %d\n", variableValue);
		}
	}

	// Clean up
	UA_Variant_clear(&value);
	UA_Client_delete(client); // Disconnects the client internally
	return EXIT_SUCCESS;
}

int COPCServer::ModifyIntVal(int iValue, const char *pcValName)
{
	char nodeIdName[1024] = { 0 };
	strcpy(nodeIdName, pcValName);
	UA_Client *client = UA_Client_new();
	UA_ClientConfig_setDefault(UA_Client_getConfig(client));
	UA_StatusCode retval = UA_Client_connect(client, m_cServerPort/*"opc.tcp://localhost:4840"*/);
	if (retval != UA_STATUSCODE_GOOD) {
		UA_Client_delete(client);
		return (int)retval;
	}

	UA_Variant value;
	UA_Variant_init(&value);

	// 变量的Node Id是string类型
	const UA_NodeId nodeId = UA_NODEID_STRING(1, nodeIdName);

	//使用UA_Client_readValueAttribute()去读取变量值
	retval = UA_Client_readValueAttribute(client, nodeId, &value);

	// 变量的数据类型是UA_Int32，第二个是判断UA_Variant里是否有UA_Int32类型的数据
	if (retval == UA_STATUSCODE_GOOD && UA_Variant_hasScalarType(&value, &UA_TYPES[UA_TYPES_INT32]))
	{
		UA_Int32 variableValue = *(UA_Int32 *)value.data;
		UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_USERLAND, "Variable Value is: %d\n", variableValue);
	}

	// 对server里的变量写入一个新值
	UA_Int32 change = iValue;
	UA_Variant newValue;
	UA_Variant_init(&newValue);
	UA_Variant_setScalar(&newValue, &change, &UA_TYPES[UA_TYPES_INT32]);
	retval = UA_Client_writeValueAttribute(client, nodeId, &newValue);

	// 重新读取变量值来验证是否写成功
	if (retval == UA_STATUSCODE_GOOD)
	{
		retval = UA_Client_readValueAttribute(client, nodeId, &value);
		if (retval == UA_STATUSCODE_GOOD && UA_Variant_hasScalarType(&value, &UA_TYPES[UA_TYPES_INT32]))
		{
			UA_Int32 variableValue = *(UA_Int32 *)value.data;
			UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_USERLAND, "New Variable Value is: %d\n", variableValue);
		}
	}

	// Clean up
	UA_Variant_clear(&value);
	UA_Client_delete(client); // Disconnects the client internally
	return EXIT_SUCCESS;
}

int COPCServer::ModifyDoubleVal(double dValue, const char *pcValName)
{
	char nodeIdName[1024] = { 0 };
	strcpy(nodeIdName, pcValName);
	UA_Client *client = UA_Client_new();
	UA_ClientConfig_setDefault(UA_Client_getConfig(client));
	UA_StatusCode retval = UA_Client_connect(client, m_cServerPort/*"opc.tcp://localhost:4840"*/);
	if (retval != UA_STATUSCODE_GOOD) {
		UA_Client_delete(client);
		return (int)retval;
	}

	UA_Variant value;
	UA_Variant_init(&value);

	// 变量的Node Id是string类型
	const UA_NodeId nodeId = UA_NODEID_STRING(1, nodeIdName);

	//使用UA_Client_readValueAttribute()去读取变量值
	retval = UA_Client_readValueAttribute(client, nodeId, &value);

	// 变量的数据类型是UA_Int32，第二个是判断UA_Variant里是否有UA_Int32类型的数据
	if (retval == UA_STATUSCODE_GOOD && UA_Variant_hasScalarType(&value, &UA_TYPES[UA_TYPES_DOUBLE]))
	{
		UA_Double variableValue = *(UA_Double *)value.data;
		UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_USERLAND, "Variable Value is: %.3lf\n", variableValue);
	}

	// 对server里的变量写入一个新值
	UA_Double change = dValue;
	UA_Variant newValue;
	UA_Variant_init(&newValue);
	UA_Variant_setScalar(&newValue, &change, &UA_TYPES[UA_TYPES_DOUBLE]);
	retval = UA_Client_writeValueAttribute(client, nodeId, &newValue);

	// 重新读取变量值来验证是否写成功
	if (retval == UA_STATUSCODE_GOOD)
	{
		retval = UA_Client_readValueAttribute(client, nodeId, &value);
		if (retval == UA_STATUSCODE_GOOD && UA_Variant_hasScalarType(&value, &UA_TYPES[UA_TYPES_DOUBLE]))
		{
			UA_Double variableValue = *(UA_Double *)value.data;
			UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_USERLAND, "New Variable Value is: %.3lf\n", variableValue);
		}
	}

	// Clean up
	UA_Variant_clear(&value);
	UA_Client_delete(client); // Disconnects the client internally
	return EXIT_SUCCESS;
}

int COPCServer::ModifyStrVal(std::string sValue, const char *pcValName)
{
	char nodeIdName[1024] = { 0 };
	strcpy(nodeIdName, pcValName);
	UA_Client *client = UA_Client_new();
	UA_ClientConfig_setDefault(UA_Client_getConfig(client));
	UA_StatusCode retval = UA_Client_connect(client, m_cServerPort/*"opc.tcp://localhost:4840"*/);
	if (retval != UA_STATUSCODE_GOOD) {
		UA_Client_delete(client);
		return (int)retval;
	}

	UA_Variant value;
	UA_Variant_init(&value);

	// 变量的Node Id是string类型
	const UA_NodeId nodeId = UA_NODEID_STRING(1, nodeIdName);

	//使用UA_Client_readValueAttribute()去读取变量值
	retval = UA_Client_readValueAttribute(client, nodeId, &value);

	// 变量的数据类型是UA_Int32，第二个是判断UA_Variant里是否有UA_Int32类型的数据
	if (retval == UA_STATUSCODE_GOOD && UA_Variant_hasScalarType(&value, &UA_TYPES[UA_TYPES_STRING]))
	{
		UA_String variableValue = *(UA_String *)value.data;
		//UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_USERLAND, "Variable Value is: %s\n", &variableValue);
	}

	// 对server里的变量写入一个新值
	std::wstring wsName = String2WString(sValue);
	std::string temp = WstringToUtf8(wsName);

	UA_String change = UA_STRING((char*)(temp.c_str()));
	UA_Variant newValue;
	UA_Variant_init(&newValue);
	UA_Variant_setScalar(&newValue, &change, &UA_TYPES[UA_TYPES_STRING]);
	retval = UA_Client_writeValueAttribute(client, nodeId, &newValue);

	// 重新读取变量值来验证是否写成功
	if (retval == UA_STATUSCODE_GOOD)
	{
		retval = UA_Client_readValueAttribute(client, nodeId, &value);
		if (retval == UA_STATUSCODE_GOOD && UA_Variant_hasScalarType(&value, &UA_TYPES[UA_TYPES_STRING]))
		{
			UA_String variableValue = *(UA_String *)value.data;
			//UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_USERLAND, "New Variable Value is: %s\n", variableValue);
		}
	}

	// Clean up
	UA_Variant_clear(&value);
	UA_Client_delete(client); // Disconnects the client internally
	return EXIT_SUCCESS;
}

int COPCServer::ModifyTimeVal(char *sVarName, SYSTEMTIME st)
{
	UA_Client *client = UA_Client_new();
	UA_ClientConfig_setDefault(UA_Client_getConfig(client));
	UA_StatusCode retval = UA_Client_connect(client, m_cServerPort/*"opc.tcp://localhost:4840"*/);
	if (retval != UA_STATUSCODE_GOOD) {
		UA_Client_delete(client);
		return (int)retval;
	}

	UA_Variant value;
	UA_Variant_init(&value);

	// 变量的Node Id是string类型
	const UA_NodeId nodeId = UA_NODEID_STRING(1, sVarName);

	//使用UA_Client_readValueAttribute()去读取变量值
	retval = UA_Client_readValueAttribute(client, nodeId, &value);

	// 变量的数据类型是UA_Int32，第二个是判断UA_Variant里是否有UA_Int32类型的数据
	if (retval == UA_STATUSCODE_GOOD && UA_Variant_hasScalarType(&value, &UA_TYPES[UA_TYPES_DATETIME]))
	{
		UA_String variableValue = *(UA_String *)value.data;
		//UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_USERLAND, "Variable Value is: %s\n", &variableValue);
	}

	// 对server里的变量写入一个新值
	UA_DateTimeStruct dtStruct;
	memset(&dtStruct, 0, sizeof(dtStruct));
	dtStruct.year = st.wYear;
	dtStruct.month = st.wMonth;
	dtStruct.day = st.wDay;
	dtStruct.hour = st.wHour;
	dtStruct.min = st.wMinute;
	dtStruct.sec = st.wSecond;
	dtStruct.milliSec = st.wMilliseconds;
	UA_DateTime dt = UA_DateTime_fromStruct(dtStruct);

	UA_Variant newValue;
	UA_Variant_init(&newValue);
	UA_Variant_setScalar(&newValue, &dt, &UA_TYPES[UA_TYPES_DATETIME]);
	retval = UA_Client_writeValueAttribute(client, nodeId, &newValue);

	// 重新读取变量值来验证是否写成功
	if (retval == UA_STATUSCODE_GOOD)
	{
		retval = UA_Client_readValueAttribute(client, nodeId, &value);
		if (retval == UA_STATUSCODE_GOOD && UA_Variant_hasScalarType(&value, &UA_TYPES[UA_TYPES_DATETIME]))
		{
			UA_String variableValue = *(UA_String *)value.data;
			//UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_USERLAND, "New Variable Value is: %s\n", variableValue);
		}
		else
		{
			return EXIT_FAILURE;
		}
	}
	else
	{
		return EXIT_FAILURE;
	}

	// Clean up
	UA_Variant_clear(&value);
	UA_Client_delete(client); // Disconnects the client internally
	return EXIT_SUCCESS;
}

bool COPCServer::GetVarVal(char *pcVarName, int nVarType, void *pValue)
{
	// 变量类型：bool:0  int:5  double:10  string:11
	bool bRst = false;
	char nodeIdName[1024] = { 0 };
	strcpy(nodeIdName, pcVarName);
	UA_Client *client = UA_Client_new();
	UA_ClientConfig_setDefault(UA_Client_getConfig(client));
	UA_StatusCode retval = UA_Client_connect(client, m_cServerPort/*"opc.tcp://localhost:4840"*/);
	if (retval != UA_STATUSCODE_GOOD) {
		UA_Client_delete(client);
		return false;
	}

	UA_Variant value;
	UA_Variant_init(&value);

	// 变量的Node Id是string类型
	const UA_NodeId nodeId = UA_NODEID_STRING(1, nodeIdName);

	//使用UA_Client_readValueAttribute()去读取变量值
	retval = UA_Client_readValueAttribute(client, nodeId, &value);


	// 判断UA_Variant里是否有 nVarType 类型的数据
	//if (retval == UA_STATUSCODE_GOOD && UA_Variant_hasScalarType(&value, &UA_TYPES[UA_TYPES_INT32]))
	if (retval == UA_STATUSCODE_GOOD && UA_Variant_hasScalarType(&value, &UA_TYPES[nVarType]))
	{
		if (0 == nVarType)
		{
			UA_Boolean bVariableValue = *(UA_Boolean *)value.data;
			*(bool*)pValue = bVariableValue;
		}
		else if (5 == nVarType)
		{
			UA_Int32 nVariableValue = *(UA_Int32 *)value.data;
			*(int*)pValue = nVariableValue;
		}
		else if (10 == nVarType)
		{
			UA_Double dVariableValue = *(UA_Double *)value.data;
			*(double*)pValue = dVariableValue;
		}
		else if(11 == nVarType)
		{
			UA_String sVariableValue = *(UA_String *)value.data;
			strncpy((char*)pValue, (const char*)sVariableValue.data, sVariableValue.length);
			((char*)pValue)[sVariableValue.length] = '\0';
		}
		else
		{
			bRst = false;
		}
		bRst = true;
	}
	else
	{
		bRst = false;
	}

	// Clean up
	UA_Variant_clear(&value);
	UA_Client_delete(client); // Disconnects the client internally
	return bRst;
}

bool COPCServer::GetVarVal(char *pcNarName, void *pValue)
{
	ObjectNodeInfo *pObjectNodeInfo = &g_vtObjectNodeInfo[0];
	int nOPCSendVarNo = 0;
	for (nOPCSendVarNo = 0; nOPCSendVarNo < pObjectNodeInfo->nServerVarSum; nOPCSendVarNo++)
	{
		if (strcmp(pcNarName, pObjectNodeInfo->vServerVarInfo[nOPCSendVarNo].varName) == 0)
		{
			UA_Boolean bVariableValue;
			UA_Int32 nVariableValue;
			UA_Double dVariableValue;
			switch (pObjectNodeInfo->vServerVarInfo[nOPCSendVarNo].nVarType)
			{
			case 0:
				bVariableValue = *(UA_Boolean *)pObjectNodeInfo->vServerVarInfo[nOPCSendVarNo].pValue;
				*(UA_Boolean*)pValue = bVariableValue;
				break;
			case 5:
				nVariableValue = *(UA_Int32 *)pObjectNodeInfo->vServerVarInfo[nOPCSendVarNo].pValue;
				*(UA_Int32*)pValue = nVariableValue;
				break;
			case 10:
				dVariableValue = *(UA_Double *)pObjectNodeInfo->vServerVarInfo[nOPCSendVarNo].pValue;
				*(UA_Double*)pValue = dVariableValue;
				break;
			case 11:
 				strncpy((char*)pValue, (char*)(pObjectNodeInfo->vServerVarInfo[nOPCSendVarNo].pValue), pObjectNodeInfo->vServerVarInfo[nOPCSendVarNo].nValueLength);
 				((char*)pValue)[pObjectNodeInfo->vServerVarInfo[nOPCSendVarNo].nValueLength] = '\0';
				break;
			case 12:
// 				UA_Boolean bVariableValue = *(UA_Boolean *)pObjectNodeInfo->vServerVarInfo[nOPCSendVarNo].pValue;
// 				*(bool*)pValue = bVariableValue;
				break;
			default:
				break;
			}
			
			break;
		}
	}
	if (nOPCSendVarNo == pObjectNodeInfo->nServerVarSum)
	{
		return false;
	}
	return true;
}

void COPCServer::AddObjectNode(UA_Server *server, char *nodeName, UA_NodeId *MotorStatorId)
{

	UA_ObjectAttributes stuAttr = UA_ObjectAttributes_default;
	stuAttr.displayName = UA_LOCALIZEDTEXT("en-US", nodeName);
	UA_Server_addObjectNode(server, UA_NODEID_NULL,
		UA_NODEID_NUMERIC(0, UA_NS0ID_OBJECTSFOLDER),
		UA_NODEID_NUMERIC(0, UA_NS0ID_ORGANIZES),
		UA_QUALIFIEDNAME(1, nodeName), UA_NODEID_NUMERIC(0, UA_NS0ID_BASEOBJECTTYPE),
		stuAttr, NULL, MotorStatorId);
}

void COPCServer::addObjectFromFile(UA_Server *server)
{	
	for (int i = 0; i < g_vtObjectNodeInfo.size(); i++)
	{
		// 创建对象节点
		UA_NodeId nodeId;
		//char *pObjectName = objectName;
		AddObjectNode(server,g_vtObjectNodeInfo[i].objectName, &nodeId);
		// 添加服务器接收变量
		for (int nServerRecvVarNo = 0; nServerRecvVarNo < g_vtObjectNodeInfo[i].nServerVarSum; nServerRecvVarNo++)
		{
			AddVariable(server, g_vtObjectNodeInfo[i].nOrderNumber, g_vtObjectNodeInfo[i].vServerVarInfo[nServerRecvVarNo].varName, 
						g_vtObjectNodeInfo[i].vServerVarInfo[nServerRecvVarNo].nVarType, g_vtObjectNodeInfo[i].vServerVarInfo[nServerRecvVarNo].pValue , &nodeId, true, NULL);
		}
	}
}

UINT COPCServer::ThreadRunOPCDaemon(void *pParam)
{
	COPCServer *pObj = (COPCServer*)pParam;
	int i;
	ObjectNodeInfo *objectNodeInfo = &g_vtObjectNodeInfo[0];
	while (!pObj->m_bServerQuit)
	{
		for (i = 0; i < objectNodeInfo->nServerVarSum; i++)
		{
			int nOffsetLength = pObj->CheckDataChange(&(objectNodeInfo->vServerVarInfo[i]));
		}
		Sleep(50);
	}
	return 0;
}

//OPC服务器运行线程
UINT COPCServer::ThreadRunOPCServer(void *pParam)
{
	COPCServer *pObj = (COPCServer*)pParam;
	//InitCallBackFunc();

	printf("ThreadRunOPCServer Start……\n");
	pObj->m_DeviceServer = UA_Server_new();
	UA_StatusCode retval = UA_ServerConfig_setMinimal(UA_Server_getConfig(pObj->m_DeviceServer), pObj->m_nPort, NULL); // 指定端口

	// 添加配置文件中的节点
	pObj->addObjectFromFile(pObj->m_DeviceServer);

	retval = UA_Server_run(pObj->m_DeviceServer, &pObj->m_bRunning);

	UA_Server_delete(pObj->m_DeviceServer);
	printf("ThreadRunOPCServer End……\n");

	return retval == UA_STATUSCODE_GOOD ? EXIT_SUCCESS : EXIT_FAILURE;
}

bool COPCServer::SetInitVal(VarInfo &varInfo)
{
	switch (varInfo.nVarType)
	{
	case 0:
		varInfo.pValue = new bool;
		*(bool *)varInfo.pValue = false;
		varInfo.bPreValue = *(bool *)varInfo.pValue;
		break;
	case 5:
		varInfo.pValue = new int;
		*(int *)varInfo.pValue = 0;
		varInfo.nPreValue = *(int *)varInfo.pValue;
		break;
	case 10:
		varInfo.pValue = new double;
		*(double *)varInfo.pValue = 0.0;
		varInfo.dPreValue = *(double *)varInfo.pValue;
		break;
	case 11:
		varInfo.pValue = new char[255];
		strcpy((char *)varInfo.pValue, "\0");
		strncpy(varInfo.acPreValue, (char *)varInfo.pValue, 1);
		break;
	case 12:
		varInfo.pValue = new char[40];
		strcpy((char *)varInfo.pValue, "null\0");
		strncpy(varInfo.acPreValue, (char *)varInfo.pValue, 1);
		break;
	default:
		return false;
		break;
	}
	return true;
}

bool COPCServer::WriteWorkStationInfo()
{
	FILE* pfServerParam = fopen(m_cOPCServerInfoIni, "w");
	if (NULL == pfServerParam)
	{
		printf("******%s文件打开失败！******\n", m_cOPCServerInfoIni);
		return false;
	}

	for (int i = 0; i < g_vtObjectNodeInfo.size(); i++)
	{
		fprintf(pfServerParam, "%s	%d\n", g_vtObjectNodeInfo[i].objectName, g_vtObjectNodeInfo[i].nServerVarSum);
		for (size_t j = 0; j < g_vtObjectNodeInfo[i].vServerVarInfo.size(); j++)
		{
			fprintf(pfServerParam, "%d\n", g_vtObjectNodeInfo[i].vServerVarInfo[j].nVarType);
			switch (g_vtObjectNodeInfo[i].vServerVarInfo[j].nVarType)
			{
			case 0:
				fprintf(pfServerParam, "%s	%d\n",
					g_vtObjectNodeInfo[i].vServerVarInfo[j].varOrgName, 
					IntToBool(*(bool*)g_vtObjectNodeInfo[i].vServerVarInfo[j].pValue));
				break;
			case 5:
				fprintf(pfServerParam, "%s	%d\n",
					g_vtObjectNodeInfo[i].vServerVarInfo[j].varOrgName,
					*(int*)g_vtObjectNodeInfo[i].vServerVarInfo[j].pValue);
				break;
			case 10:
				fprintf(pfServerParam, "%s	%lf\n",
					g_vtObjectNodeInfo[i].vServerVarInfo[j].varOrgName,
					*(double*)g_vtObjectNodeInfo[i].vServerVarInfo[j].pValue);
				break;
			case 11:
				if (strlen((char*)g_vtObjectNodeInfo[i].vServerVarInfo[j].pValue) == 0)
				{
					fprintf(pfServerParam, "%s	%s\n",
						g_vtObjectNodeInfo[i].vServerVarInfo[j].varOrgName,
						"null");
				}
				else
				{
					fprintf(pfServerParam, "%s	%s\n",
						g_vtObjectNodeInfo[i].vServerVarInfo[j].varOrgName,
						(char*)g_vtObjectNodeInfo[i].vServerVarInfo[j].pValue);
				}
				break;
			default:
				break;
			}
		}
	}


	fclose(pfServerParam);
	return true;
}

bool COPCServer::IntToBool(int nVal)
{
	if (nVal)
	{
		return true;
	}
	return false;
}

void COPCServer::SetVal(vector<VarInfo> &vServerVarInfo, char acValName[100], void *pVal)
{
	for (size_t i = 0; i < vServerVarInfo.size(); i++)
	{
		if (!strcmp(vServerVarInfo[i].varOrgName, acValName))
		{
			switch (vServerVarInfo[i].nVarType)
			{
			case 0:
				*(bool*)vServerVarInfo[i].pValue = *(bool*)pVal;
				break;
			case 5:
				*(int*)vServerVarInfo[i].pValue = *(int*)pVal;
				break;
			case 10:
				*(double*)vServerVarInfo[i].pValue = *(double*)pVal;
				break;
			case 11:
				strcpy((char*)vServerVarInfo[i].pValue, (char*)pVal);
				break;
			default:
				break;
			}
			break;
		}
	}
}

bool COPCServer::ReadWorkStationInfo()
{
	FILE* pfServerParam = fopen(m_cOPCServerInfoIni, "r");
	if (NULL == pfServerParam)
	{
		printf("******%s文件打开失败！******\n", m_cOPCServerInfoIni);
		return false;
	}

	char acName[100];
	void* pVal;
	for (int i = 0; i < g_vtObjectNodeInfo.size(); i++)
	{
		int nServerVarSum;
		fscanf(pfServerParam, "%s	%d\n", g_vtObjectNodeInfo[i].objectName, &nServerVarSum);
		for (size_t j = 0; j < nServerVarSum; j++)
		{
			int nType;
			fscanf(pfServerParam, "%d\n", &nType);
			switch (nType)
			{
			case 0:
				pVal = new bool;
				int nVal;
				fscanf(pfServerParam, "%s	%d\n",
					acName, &nVal);
				*(bool*)pVal = IntToBool(nVal);
				SetVal(g_vtObjectNodeInfo[i].vServerVarInfo, acName, pVal);
				delete pVal;
				break;
			case 5:
				pVal = new int;
				fscanf(pfServerParam, "%s	%d\n",
					acName, (int*)pVal);
				SetVal(g_vtObjectNodeInfo[i].vServerVarInfo, acName, pVal);
				delete pVal;
				break;
			case 10:
				pVal = new double;
				fscanf(pfServerParam, "%s	%lf\n",
					acName, (double*)pVal);
				SetVal(g_vtObjectNodeInfo[i].vServerVarInfo, acName, pVal);
				delete pVal;
				break;
			case 11:
				pVal = new char[255];
				fscanf(pfServerParam, "%s	%s\n",
					acName, (char*)pVal);
				if (!strcmp((char*)pVal, "null"))
				{
					SetVal(g_vtObjectNodeInfo[i].vServerVarInfo, acName, "\0");
				}
				else
				{
					SetVal(g_vtObjectNodeInfo[i].vServerVarInfo, acName, pVal);
				}
				delete[]pVal;
				break;
			default:
				break;
			}
		}
	}
	fclose(pfServerParam);
	return true;
}

bool COPCServer::ReadWorkStationParam()
{
	FILE *pfServerParam = fopen(m_cOPCServerParamIni, "r");
 	if (NULL == pfServerParam)
	{
		printf("******%s文件打开失败！******\n", m_cOPCServerParamIni);
		return false;
	}
	int nObjectSum = 0;
	string sTemp;
	fscanf(pfServerParam, "ObjectSum:%d\n", &nObjectSum);										printf("ObjectSum:%d\n", nObjectSum); 
	fscanf(pfServerParam, "Port:%d\n", &m_nPort);
	sprintf(m_cServerPort, "opc.tcp://127.0.0.1:%d", m_nPort);
	printf("ServerIP : %s\n", m_cServerPort);
	for (int i = 0; i < nObjectSum; i++)
	{
		cout << "*************loopTime : " << i << endl;
		ObjectNodeInfo objectNodeInfo;
		memset(&objectNodeInfo, 0, sizeof(objectNodeInfo));
		fscanf(pfServerParam, "ObjectName:%s\n", objectNodeInfo.objectName);					printf("ObjectName:%s\n", objectNodeInfo.objectName);
		fscanf(pfServerParam, "OrderNumber:%d\n", &(objectNodeInfo.nOrderNumber));				printf("OrderNumber:%d\n", objectNodeInfo.nOrderNumber);
		//sprintf(objectNodeInfo.objectName, "%s", objectNodeInfo.objectName);
		fscanf(pfServerParam, "nServerVarSum:%d\n", &(objectNodeInfo.nServerVarSum));		printf("nServerVarSum:%d\n", objectNodeInfo.nServerVarSum);

		// 工作站上传变量
		for (int nVarNo = 0; nVarNo < objectNodeInfo.nServerVarSum; nVarNo++)
		{
			VarInfo varInfo;
			int n = fscanf(pfServerParam, "%s%d\n", varInfo.varOrgName, &varInfo.nVarType);
			if (n == -1)
			{
				objectNodeInfo.nServerVarSum = nVarNo;
				break;
			}
			sprintf(varInfo.varName, "%s", varInfo.varOrgName);
			SetInitVal(varInfo);
			varInfo.bChangeVal = false;
			objectNodeInfo.vServerVarInfo.push_back(varInfo);								printf("VarName:%s, VarType:%d\n", objectNodeInfo.vServerVarInfo[nVarNo].varName, objectNodeInfo.vServerVarInfo[nVarNo].nVarType);
			
		}
		g_vtObjectNodeInfo.push_back(objectNodeInfo);
	}
	fclose(pfServerParam);
	return ReadWorkStationInfo();
}

UINT COPCServer::ThreadChangeVal(void* pParam)
{
	//传入自定义参数
	T_CONNECT_OPC_THREAD_DATA_SERVER pObj = *((T_CONNECT_OPC_THREAD_DATA_SERVER*)pParam);
	delete pParam;

	int nVarType = pObj.ptObjectValInfo->nVarType;
	UA_StatusCode retval = -1;

	switch (nVarType)
	{
	case 0:
		retval = pObj.pFather->ModifyBooleanVal(*((bool*)pObj.ptObjectValInfo->pValue), pObj.ptObjectValInfo->varName);
		if (retval == UA_STATUSCODE_GOOD)
		{
			pObj.ptObjectValInfo->bPreValue = *((bool*)pObj.ptObjectValInfo->pValue);
		}
		break;
	case 5:
		retval = pObj.pFather->ModifyIntVal(*((int*)pObj.ptObjectValInfo->pValue), pObj.ptObjectValInfo->varName);
		if (retval == UA_STATUSCODE_GOOD)
		{
			pObj.ptObjectValInfo->nPreValue = *((int*)pObj.ptObjectValInfo->pValue);
		}
		break;
	case 10:
		retval = pObj.pFather->ModifyDoubleVal(*((double*)pObj.ptObjectValInfo->pValue), pObj.ptObjectValInfo->varName);
		if (retval == UA_STATUSCODE_GOOD)
		{
			pObj.ptObjectValInfo->dPreValue = *((double*)pObj.ptObjectValInfo->pValue);
		}
		break;
	case 11:
		retval = pObj.pFather->ModifyStrVal((char*)pObj.ptObjectValInfo->pValue, pObj.ptObjectValInfo->varName);
		if (retval == UA_STATUSCODE_GOOD)
		{
			memset(pObj.ptObjectValInfo->acPreValue, 0, sizeof(pObj.ptObjectValInfo->acPreValue));
			strncpy(pObj.ptObjectValInfo->acPreValue, (char*)pObj.ptObjectValInfo->pValue, sizeof(pObj.ptObjectValInfo->acPreValue) - 1);
		}
		break;
	case 12:
		retval = -1;
		break;
	default:
		retval = -1;
		break;
	}
	pObj.ptObjectValInfo->bChangeVal = false;
	return 0;
}

int COPCServer::CheckDataChange(VarInfo* pVarInfo) // 根据varType将 数据长度 和 数据组合成buffer 并返回两部分长度和
{
	// 【变量名长	变量名		类型	数据长度	数据】 ……
	//		4		  n			 4			4		  n	   ……
	// 		%4d		  %s		 %4d		%4d		  ?	   ……
	int nVarType = pVarInfo->nVarType;
	void* pValue = pVarInfo->pValue;
	UA_StatusCode retval = -1;
	bool bChange = false;
	UINT threadOPCServerId;

	int n;
	switch (nVarType)
	{
	case 0:
		if (*((bool*)pValue) != pVarInfo->bPreValue && !pVarInfo->bChangeVal)
		{
			pVarInfo->bChangeVal = true;
			T_CONNECT_OPC_THREAD_DATA_SERVER* ptThreadData = new T_CONNECT_OPC_THREAD_DATA_SERVER;
			ptThreadData->nThreadID = 1000;
			ptThreadData->ptObjectValInfo = pVarInfo;
			ptThreadData->pFather = this;
			_beginthreadex(NULL, 0, ThreadChangeVal, (void*)ptThreadData, 0, &threadOPCServerId);
		}
		break;
	case 5:
		n = *((int*)pValue);
		if (*((int*)pValue) != pVarInfo->nPreValue && !pVarInfo->bChangeVal)
		{
			pVarInfo->bChangeVal = true;
			T_CONNECT_OPC_THREAD_DATA_SERVER* ptThreadData = new T_CONNECT_OPC_THREAD_DATA_SERVER;
			ptThreadData->nThreadID = 1001;
			ptThreadData->ptObjectValInfo = pVarInfo;
			ptThreadData->pFather = this;
			_beginthreadex(NULL, 0, ThreadChangeVal, (void*)ptThreadData, 0, &threadOPCServerId);
		}
		break;
	case 10:
		if (fabs(*((double*)pValue) - pVarInfo->dPreValue) > 0.01 && !pVarInfo->bChangeVal)
		{
			pVarInfo->bChangeVal = true;
			T_CONNECT_OPC_THREAD_DATA_SERVER* ptThreadData = new T_CONNECT_OPC_THREAD_DATA_SERVER;
			ptThreadData->nThreadID = 1002;
			ptThreadData->ptObjectValInfo = pVarInfo;
			ptThreadData->pFather = this;
			_beginthreadex(NULL, 0, ThreadChangeVal, (void*)ptThreadData, 0, &threadOPCServerId);
		}
		break;
	case 11:
		if (strcmp((char*)pValue, pVarInfo->acPreValue) != 0 && !pVarInfo->bChangeVal)
		{
			pVarInfo->bChangeVal = true;
			T_CONNECT_OPC_THREAD_DATA_SERVER* ptThreadData = new T_CONNECT_OPC_THREAD_DATA_SERVER;
			ptThreadData->nThreadID = 1003;
			ptThreadData->ptObjectValInfo = pVarInfo;
			ptThreadData->pFather = this;
			_beginthreadex(NULL, 0, ThreadChangeVal, (void*)ptThreadData, 0, &threadOPCServerId);
		}
		break;
	case 12:
		retval = -1;
		break;
	default:
		retval = -1;
		break;
	}
	return retval;
}