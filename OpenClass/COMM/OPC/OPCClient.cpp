#include "StdAfx.h"
#include "OPCClient.h"

void fingtotalnodeid(UA_Client* client, std::map<CString, UA_NodeId> &mTotalNodeID, UA_NodeId nNodeID)
{
	printf("Browsing nodes in objects folder:\n");
	UA_BrowseRequest bReq;
	UA_BrowseRequest_init(&bReq);
	bReq.requestedMaxReferencesPerNode = 0;//限制查到的最大节点数，0 不限制
	bReq.nodesToBrowse = UA_BrowseDescription_new();
	bReq.nodesToBrowseSize = 1;//需要浏览的节点个数，这里只寻找server节点下的节点所以为1
	/*UA_BROWSEDIRECTION_FORWARD表示向下查找（即查找添加在节点下的节点），
	UA_BROWSEDIRECTION_INVERSE表示向上查找（即查找节点的父节点），
	UA_BROWSEDIRECTION_BOTH表示上下都进行查找*/
	bReq.nodesToBrowse[0].browseDirection = UA_BROWSEDIRECTION_FORWARD;
	bReq.nodesToBrowse[0].includeSubtypes = UA_TRUE;//是否包含subtypes
	bReq.nodesToBrowse[0].nodeId = nNodeID;//设置起始浏览节点为 20KW板材分拣
//	bReq.nodesToBrowse[0].nodeId = UA_NODEID_NUMERIC(0, UA_NS0ID_TYPESFOLDER);
	bReq.nodesToBrowse[0].resultMask = UA_BROWSERESULTMASK_ALL; //返回浏览到的节点包含的信息，名称、显示名称......，UA_BROWSERESULTMASK_ALL表示返回所有信息
	//bReq.nodesToBrowse[0].referenceTypeId = UA_NODEID_NUMERIC(0, UA_NS0ID_HASCOMPONENT);//筛选引用类型
	UA_BrowseResponse bResp = UA_Client_Service_browse(client, bReq);
	//输出浏览到的每个节点信息
	TRACE("%-9s %-16s %-16s %-16s\n", "NAMESPACE", "NODEID", "BROWSE NAME", "DISPLAY NAME");
	for (size_t i = 0; i < bResp.resultsSize; ++i) {
		for (size_t j = 0; j < bResp.results[i].referencesSize; ++j) {
			UA_ReferenceDescription* ref = &(bResp.results[i].references[j]);

			CString str;
			str.Format("%-.*s", (int)ref->displayName.text.length, ref->displayName.text.data);
			str = Utf8ToGBK(str.GetBuffer());
			mTotalNodeID[str] = ref->nodeId.nodeId;

			if (ref->nodeId.nodeId.identifierType == UA_NODEIDTYPE_NUMERIC) {
				TRACE("%-9u %-16u %-16.*s %-16.*s\n", ref->nodeId.nodeId.namespaceIndex,
					ref->nodeId.nodeId.identifier.numeric, (int)ref->browseName.name.length,
					ref->browseName.name.data, (int)ref->displayName.text.length,
					ref->displayName.text.data);
			}
			else if (ref->nodeId.nodeId.identifierType == UA_NODEIDTYPE_STRING) {
				TRACE("%-9u %-16.*s %-16.*s %-16.*s\n", ref->nodeId.nodeId.namespaceIndex,
					(int)ref->nodeId.nodeId.identifier.string.length,
					ref->nodeId.nodeId.identifier.string.data,
					(int)ref->browseName.name.length, ref->browseName.name.data,
					(int)ref->displayName.text.length, ref->displayName.text.data);

				mTotalNodeID[str].identifier.string.data = new UA_Byte[mTotalNodeID[str].identifier.string.length+1];
				memcpy(mTotalNodeID[str].identifier.string.data, ref->nodeId.nodeId.identifier.string.data, mTotalNodeID[str].identifier.string.length);
				((char*)mTotalNodeID[str].identifier.string.data)[mTotalNodeID[str].identifier.string.length] = '\0';
			}
			/* TODO: distinguish further types */
		}
	}
	UA_BrowseRequest_clear(&bReq);
	UA_BrowseResponse_clear(&bResp);
}


static UA_StatusCode translateBrowsePathsToNodeIdsRequest(UA_Client* client, UA_NodeId* returnId, char *acName)
{
	UA_StatusCode ret = UA_STATUSCODE_GOOD;

#define BROWSE_PATHS_SIZE 5
	char* paths[BROWSE_PATHS_SIZE] = { "Objects", "ServerInterfaces", "服务器接口_1", "complc", "name" };
//	char* paths[BROWSE_PATHS_SIZE] = { "Objects", "111\\COMPLC", "name" };
	paths[BROWSE_PATHS_SIZE-1] = acName;
	UA_UInt32 ids[BROWSE_PATHS_SIZE] = { UA_NS0ID_ORGANIZES, UA_NS0ID_ORGANIZES, UA_NS0ID_HASCOMPONENT };
	int nsNumOfQualifiedName[BROWSE_PATHS_SIZE] = { 0, 1, 1 }; // namespace number of qualified name

	UA_BrowsePath browsePath;
	UA_BrowsePath_init(&browsePath);
	browsePath.startingNode = UA_NODEID_NUMERIC(0, UA_NS0ID_ROOTFOLDER); // start节点是Root
	browsePath.relativePath.elements = (UA_RelativePathElement*)UA_Array_new(BROWSE_PATHS_SIZE, &UA_TYPES[UA_TYPES_RELATIVEPATHELEMENT]);
	browsePath.relativePath.elementsSize = BROWSE_PATHS_SIZE;

	for (size_t i = 0; i < BROWSE_PATHS_SIZE; ++i) {
		UA_RelativePathElement* elem = &browsePath.relativePath.elements[i];
		elem->referenceTypeId = UA_NODEID_NUMERIC(0, ids[i]);
		elem->targetName = UA_QUALIFIEDNAME_ALLOC(nsNumOfQualifiedName[i], paths[i]);
	}

	UA_TranslateBrowsePathsToNodeIdsRequest request;
	UA_TranslateBrowsePathsToNodeIdsRequest_init(&request);
	request.browsePaths = &browsePath;
	request.browsePathsSize = 1;

	UA_TranslateBrowsePathsToNodeIdsResponse response = UA_Client_Service_translateBrowsePathsToNodeIds(client, request);
	if (response.responseHeader.serviceResult == UA_STATUSCODE_GOOD)
	{
		if (response.resultsSize == 1 && response.results[0].targetsSize == 1)
		{
			UA_NodeId_copy(&response.results[0].targets[0].targetId.nodeId, returnId);
		}
	}
	else
	{
		printf("Error: %s\n", UA_StatusCode_name(response.responseHeader.serviceResult));
		ret = response.responseHeader.serviceResult;
	}

	UA_BrowsePath_deleteMembers(&browsePath);
	UA_TranslateBrowsePathsToNodeIdsResponse_deleteMembers(&response);

	return ret;
}

COPCClient::COPCClient():flagFalse(0)
{
	m_bRunning = true;
	m_bServerQuit = false;
	m_bThreadRunOPCDaemon = false;
	std::vector<CString> vstrFilePath = GetAllOPCFilePath();
	ReadWorkStationParam(vstrFilePath);
}

COPCClient::~COPCClient()
{
	m_bRunning = false;
	Sleep(10);
	m_bServerQuit = true;
	int nSleepCount = 0;
	while (m_bThreadRunOPCDaemon)
	{
		Sleep(20);
		nSleepCount++;
		if (nSleepCount > 100)
		{
			break;
		}
	}
}

void COPCClient::StartAllClient()
{
	for (unsigned int n = 0; n < g_vtObjectNodeInfo.size(); n++)
	{
		StartClient(g_vtObjectNodeInfo[n]);
	}
}

void COPCClient::StartClient(ObjectNodeInfo &tObjectNodeInfo)
{
	if (tObjectNodeInfo.bTryToConnect == false)
	{
		return;
	}
	T_CONNECT_OPC_THREAD_DATA_CLIENT *ptThreadData = new T_CONNECT_OPC_THREAD_DATA_CLIENT;
	ptThreadData->nThreadID = m_nTreadID++;
	ptThreadData->ptObjectNodeInfo = &tObjectNodeInfo;
	ptThreadData->pFather = this;
	// OPC 客户端线程
	AfxBeginThread(ThreadRunOPCClient, (void*)ptThreadData);
}

bool COPCClient::ReconnectionClient(ObjectNodeInfo &tObjectNodeInfo)
{
	if (tObjectNodeInfo.bTreadState == true)
	{
		tObjectNodeInfo.bTryToConnect = false;
		long long lStartTime = XI_clock();
		while (tObjectNodeInfo.bTreadState)
		{
			if (XI_clock() - lStartTime > 1000)
			{
				return false;
			}
			Sleep(10);
		}
	}
	tObjectNodeInfo.bTryToConnect = true;
	StartClient(tObjectNodeInfo);
	return true;
}

void COPCClient::StartRunOPCDaemon()
{
	AfxBeginThread(ThreadRunOPCDaemon, (void*)this);

}

std::string WstringToUtf8(const std::wstring& str)
{
	std::wstring_convert<std::codecvt_utf8<wchar_t> > strCnv;
	return strCnv.to_bytes(str);
}

std::wstring Utf8ToWstring(const std::string& str)
{
	std::wstring_convert< std::codecvt_utf8<wchar_t> > strCnv;
	return strCnv.from_bytes(str);
}

std::wstring String2WString(const std::string& s)
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

std::string WString2String(const std::wstring& s)
{
	std::string strLocale = setlocale(LC_ALL, "");
	const wchar_t* chSrc = s.c_str();
	size_t nDestSize = (int)wcstombs(NULL, chSrc, 0) + 1;
	char* chDest = new char[nDestSize];
	memset(chDest, 0, nDestSize);
	wcstombs(chDest, chSrc, nDestSize);
	std::string strResult = chDest;
	delete[]chDest;
	setlocale(LC_ALL, strLocale.c_str());
	return strResult;
}
/*********************************************************************/

void dataChangeNotificationBoolCallback_Client(UA_Client *client, UA_UInt32 subId,
void *subContext, UA_UInt32 monId,
void *monitoredItemContext, UA_DataValue *value)
{
	UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_SERVER, "BOOL Received Notification");

	UA_NodeId * targetNodeId = (UA_NodeId*)monitoredItemContext;

	UA_Boolean currentValue = *(UA_Boolean*)(value->value.data);

	char varName[100] = { 0 };
	if (targetNodeId->identifierType == UA_NODEIDTYPE_STRING)
	{
		strncpy(varName, (char*)targetNodeId->identifier.string.data, targetNodeId->identifier.string.length);
	}
	TRACE("%d	%d	%d\n", targetNodeId->namespaceIndex, targetNodeId->identifierType, targetNodeId->identifier.numeric);
	for (unsigned int n = 0; n < g_vtObjectNodeInfo.size(); n++)
	{
		ObjectNodeInfo *pObjectNodeInfo = &g_vtObjectNodeInfo[n];
		int nOPCSendVarNo = 0;
		for (nOPCSendVarNo = 0; nOPCSendVarNo < pObjectNodeInfo->nServerVarSum; nOPCSendVarNo++)
		{
			if (targetNodeId->identifierType == UA_NODEIDTYPE_NUMERIC)
			{
				if (targetNodeId->identifier.numeric == pObjectNodeInfo->vServerVarInfo[nOPCSendVarNo].tNodeID.identifier.numeric)
				{
					*(bool*)(pObjectNodeInfo->vServerVarInfo[nOPCSendVarNo].pValue) = currentValue;
					pObjectNodeInfo->vServerVarInfo[nOPCSendVarNo].bPreValue = currentValue;
					//WriteLog("接收数据：%d", currentValue);
					break;
				}
			}
			else
			{
				if (strcmp(varName, pObjectNodeInfo->vServerVarInfo[nOPCSendVarNo].varName) == 0)
				{
					*(bool*)(pObjectNodeInfo->vServerVarInfo[nOPCSendVarNo].pValue) = currentValue;
					pObjectNodeInfo->vServerVarInfo[nOPCSendVarNo].bPreValue = currentValue;
					//WriteLog("接收数据：%d", currentValue);
					break;
				}
			}
		}
		if (nOPCSendVarNo != pObjectNodeInfo->nServerVarSum)
		{
			break;
		}
	}
}

UA_UInt32 COPCClient::addMonitoredItemToBoolVariable(UA_Client* client, UA_NodeId* pTargetNodeId)
{
	UA_CreateSubscriptionRequest request = UA_CreateSubscriptionRequest_default();
	UA_CreateSubscriptionResponse response = UA_Client_Subscriptions_create(client, request,
		NULL, NULL, NULL);

	UA_UInt32 subId = response.subscriptionId;
	if (response.responseHeader.serviceResult == UA_STATUSCODE_GOOD)
	{
		UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_CLIENT, "Create subscription succeeded, id %u\n", subId);
	}

	UA_MonitoredItemCreateResult result;

	UA_MonitoredItemCreateRequest monRequest = UA_MonitoredItemCreateRequest_default(*pTargetNodeId);

	monRequest.requestedParameters.samplingInterval = 100.0; // 100 ms interval
	result = UA_Client_MonitoredItems_createDataChange(client, response.subscriptionId, UA_TIMESTAMPSTORETURN_BOTH,
		monRequest, (void*)pTargetNodeId, dataChangeNotificationBoolCallback_Client, NULL);
	UA_NodeId temp = *pTargetNodeId;
	delete pTargetNodeId;
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

UA_UInt32 COPCClient::addMonitoredItemToBoolVariables(UA_Client* client, UA_NodeId** pTargetNodeId, size_t size)
{
	UA_CreateSubscriptionRequest request = UA_CreateSubscriptionRequest_default();
	UA_CreateSubscriptionResponse response = UA_Client_Subscriptions_create(client, request,
		NULL, NULL, NULL);

	UA_UInt32 subId = response.subscriptionId;
	if (response.responseHeader.serviceResult == UA_STATUSCODE_GOOD)
	{
		UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_CLIENT, "Create subscription succeeded, id %u\n", subId);
	}

	UA_MonitoredItemCreateRequest* items = new UA_MonitoredItemCreateRequest[size];
	UA_Client_DataChangeNotificationCallback* callbacks = new UA_Client_DataChangeNotificationCallback[size];

	UA_Client_DeleteMonitoredItemCallback* deleteCallbacks = new UA_Client_DeleteMonitoredItemCallback[size];

	//	char** contexts = new char[size][20];

	for (int i = 0; i < size; i++)
	{
		items[i] = UA_MonitoredItemCreateRequest_default(*(pTargetNodeId[i]));
		callbacks[i] = dataChangeNotificationBoolCallback_Client;
		deleteCallbacks[i] = NULL;
	}

	UA_CreateMonitoredItemsRequest createRequest;
	UA_CreateMonitoredItemsRequest_init(&createRequest);
	createRequest.subscriptionId = subId;
	createRequest.timestampsToReturn = UA_TIMESTAMPSTORETURN_BOTH;
	createRequest.itemsToCreate = items;
	createRequest.itemsToCreateSize = size;

	UA_CreateMonitoredItemsResponse createResponse =
		UA_Client_MonitoredItems_createDataChanges(client, createRequest, (void**)pTargetNodeId, callbacks, deleteCallbacks);
	delete[]items;
	delete[]callbacks;
	delete[]deleteCallbacks;

	for (size_t i = 0; i < createResponse.resultsSize; i++)
	{
		if (createResponse.results[i].statusCode == UA_STATUSCODE_GOOD)
		{

			UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_SERVER, "Add monitored item for bool variable, OK.");
			return createResponse.results[i].monitoredItemId;
		}
		else
		{
			UA_LOG_ERROR(UA_Log_Stdout, UA_LOGCATEGORY_SERVER, "Add monitored item for bool variable, Fail.");
			return -1;
		}
	}
	return -1;
}


UA_UInt32 COPCClient::addMonitoredItemVariables(UA_Client* client, UA_NodeId** pTargetNodeId, size_t size, UA_Client_DataChangeNotificationCallback callback)
{
	UA_CreateSubscriptionRequest request = UA_CreateSubscriptionRequest_default();
	UA_CreateSubscriptionResponse response = UA_Client_Subscriptions_create(client, request,
		NULL, NULL, NULL);

	UA_UInt32 subId = response.subscriptionId;
	if (response.responseHeader.serviceResult == UA_STATUSCODE_GOOD)
	{
		UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_CLIENT, "Create subscription succeeded, id %u\n", subId);
	}

	UA_MonitoredItemCreateRequest *items = new UA_MonitoredItemCreateRequest[size];
	UA_Client_DataChangeNotificationCallback* callbacks = new UA_Client_DataChangeNotificationCallback[size];

	UA_Client_DeleteMonitoredItemCallback *deleteCallbacks = new UA_Client_DeleteMonitoredItemCallback[size];

//	char** contexts = new char[size][20];

	for (int i = 0; i < size; i++)
	{
		items[i] = UA_MonitoredItemCreateRequest_default(*(pTargetNodeId[i]));
		callbacks[i] = callback;
		deleteCallbacks[i] = NULL;
	}

	UA_CreateMonitoredItemsRequest createRequest;
	UA_CreateMonitoredItemsRequest_init(&createRequest);
	createRequest.subscriptionId = subId;
	createRequest.timestampsToReturn = UA_TIMESTAMPSTORETURN_BOTH;
	createRequest.itemsToCreate = items;
	createRequest.itemsToCreateSize = size;

	UA_CreateMonitoredItemsResponse createResponse =
		UA_Client_MonitoredItems_createDataChanges(client, createRequest, (void**)pTargetNodeId, callbacks, deleteCallbacks);
	delete[]items;
	delete[]callbacks;
	delete[]deleteCallbacks;

	for (size_t i = 0; i < createResponse.resultsSize; i++)
	{
		if (createResponse.results[i].statusCode == UA_STATUSCODE_GOOD)
		{

			UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_SERVER, "Add monitored item for bool variable, OK.");
			return createResponse.results[i].monitoredItemId;
		}
		else
		{
			UA_LOG_ERROR(UA_Log_Stdout, UA_LOGCATEGORY_SERVER, "Add monitored item for bool variable, Fail.");
			return -1;
		}
	}
	return -1;
}

void COPCClient::addMonitoredItem_BOOL(UA_Client* client, char* sVarName, std::map<CString, UA_NodeId> mTotalNodeID)
{
	UA_NodeId* myIntegerNodeId = new UA_NodeId;
	if (mTotalNodeID.find(sVarName) == mTotalNodeID.end())
	{
		*myIntegerNodeId = UA_NODEID_STRING(1, sVarName);
	}
	else
	{
		*myIntegerNodeId = mTotalNodeID[sVarName];
	}

	addMonitoredItemToBoolVariable(client, myIntegerNodeId);
}

void COPCClient::addMonitoredItems_BOOL(UA_Client* client, std::vector<UA_NodeId> vTotalNodeID)
{
	UA_NodeId** myIntegerNodeId = new UA_NodeId * [vTotalNodeID.size()];
	for (size_t i = 0; i < vTotalNodeID.size(); i++)
	{
		myIntegerNodeId[i] = new UA_NodeId;
		*myIntegerNodeId[i] = vTotalNodeID[i];
	}

	addMonitoredItemToBoolVariables(client, myIntegerNodeId, vTotalNodeID.size());
	delete[]myIntegerNodeId;
}

void COPCClient::addMonitoredItems(UA_Client* client, std::vector<UA_NodeId> vTotalNodeID, UA_Client_DataChangeNotificationCallback callback)
{
	UA_NodeId** myIntegerNodeId = new UA_NodeId * [vTotalNodeID.size()];
	for (size_t i = 0; i < vTotalNodeID.size(); i++)
	{
		myIntegerNodeId[i] = new UA_NodeId;
		*myIntegerNodeId[i] = vTotalNodeID[i];
	}

	addMonitoredItemVariables(client, myIntegerNodeId, vTotalNodeID.size(), callback);
	delete[]myIntegerNodeId;
}

void dataChangeNotificationInt32Callback_Client(UA_Client *client, UA_UInt32 subId,
	void *subContext, UA_UInt32 monId,
	void *monitoredItemContext, UA_DataValue *value)
{
	UA_NodeId * targetNodeId = (UA_NodeId*)monitoredItemContext;
	UA_Int32 currentValue = *(UA_Int32*)(value->value.data);
	UA_Int16 *adata = (UA_Int16*)(value->value.data);

	int a = adata[0];
	int a1 = adata[1];
	int a2 = adata[2];
	int a3 = adata[3];
	char varName[100] = { 0 };
	if (targetNodeId->identifierType == UA_NODEIDTYPE_STRING)
	{
		strncpy(varName, (char*)targetNodeId->identifier.string.data, targetNodeId->identifier.string.length);
	}

	for (unsigned int n = 0; n < g_vtObjectNodeInfo.size(); n++)
	{
		ObjectNodeInfo *pObjectNodeInfo = &g_vtObjectNodeInfo[n];
		int nOPCSendVarNo = 0;
		for (nOPCSendVarNo = 0; nOPCSendVarNo < pObjectNodeInfo->nServerVarSum; nOPCSendVarNo++)
		{
			if (targetNodeId->identifierType == UA_NODEIDTYPE_NUMERIC)
			{
				if (targetNodeId->identifier.numeric == pObjectNodeInfo->vServerVarInfo[nOPCSendVarNo].tNodeID.identifier.numeric)
				{
					if (value->value.arrayLength == pObjectNodeInfo->vServerVarInfo[nOPCSendVarNo].nArrayLength)
					{
						for (size_t i = 0; i < value->value.arrayLength; i++)
						{
							((UA_Int16*)pObjectNodeInfo->vServerVarInfo[nOPCSendVarNo].pValue)[i] = adata[i];
							pObjectNodeInfo->vServerVarInfo[nOPCSendVarNo].anPreValue[i] = adata[i];

							UA_Int16 a = ((UA_Int16*)pObjectNodeInfo->vServerVarInfo[nOPCSendVarNo].pValue)[i];
							int n = 0;
						}
					}
					else
					{
						*(UA_Int32*)(pObjectNodeInfo->vServerVarInfo[nOPCSendVarNo].pValue) = currentValue;
						pObjectNodeInfo->vServerVarInfo[nOPCSendVarNo].nPreValue = currentValue;
					}
					break;
				}
			}
			else
			{
				if (strcmp(varName, pObjectNodeInfo->vServerVarInfo[nOPCSendVarNo].varName) == 0)
				{
					*(UA_Int32*)(pObjectNodeInfo->vServerVarInfo[nOPCSendVarNo].pValue) = currentValue;
					pObjectNodeInfo->vServerVarInfo[nOPCSendVarNo].nPreValue = currentValue;
					break;
				}
			}
		}
		if (nOPCSendVarNo != pObjectNodeInfo->nServerVarSum)
		{
			break;
		}
	}
}

UA_UInt32 COPCClient::addMonitoredItemToInt32Variable(UA_Client *client, UA_NodeId *pTargetNodeId)
{
	UA_CreateSubscriptionRequest request = UA_CreateSubscriptionRequest_default();
	UA_CreateSubscriptionResponse response = UA_Client_Subscriptions_create(client, request,
		NULL, NULL, NULL);

	UA_UInt32 subId = response.subscriptionId;
	if (response.responseHeader.serviceResult == UA_STATUSCODE_GOOD)
	{
		UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_CLIENT, "Create subscription succeeded, id %u\n", subId);
	}

	UA_MonitoredItemCreateResult result;

	UA_MonitoredItemCreateRequest monRequest = UA_MonitoredItemCreateRequest_default(*pTargetNodeId);

	monRequest.requestedParameters.samplingInterval = 100.0; // 100 ms interval
	result = UA_Client_MonitoredItems_createDataChange(client, response.subscriptionId, UA_TIMESTAMPSTORETURN_BOTH,
		monRequest, (void*)pTargetNodeId, dataChangeNotificationInt32Callback_Client, NULL);

	delete pTargetNodeId;
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
	return -1;
}

void COPCClient::addMonitoredItem_INT(UA_Client *client, char *sVarName, std::map<CString, UA_NodeId> mTotalNodeID)
{
	UA_NodeId * myIntegerNodeId = new UA_NodeId;
	if (mTotalNodeID.find(sVarName) == mTotalNodeID.end())
	{
		*myIntegerNodeId = UA_NODEID_STRING(1, sVarName);
	}
	else
	{
		*myIntegerNodeId = mTotalNodeID[sVarName];
	}
	addMonitoredItemToInt32Variable(client, myIntegerNodeId);
}

void dataChangeNotificationDoubleCallback_Client(UA_Client *client, UA_UInt32 subId,
	void *subContext, UA_UInt32 monId,
	void *monitoredItemContext, UA_DataValue *value)
{
	UA_NodeId * targetNodeId = (UA_NodeId*)monitoredItemContext;
	UA_Double currentValue = *(UA_Double*)(value->value.data);

	char varName[100] = { 0 };
	if (targetNodeId->identifierType == UA_NODEIDTYPE_STRING)
	{
		strncpy(varName, (char*)targetNodeId->identifier.string.data, targetNodeId->identifier.string.length);
	}


	for (unsigned int n = 0; n < g_vtObjectNodeInfo.size(); n++)
	{
		ObjectNodeInfo *pObjectNodeInfo = &g_vtObjectNodeInfo[n];
		int nOPCSendVarNo = 0;
		for (nOPCSendVarNo = 0; nOPCSendVarNo < pObjectNodeInfo->nServerVarSum; nOPCSendVarNo++)
		{
			if (targetNodeId->identifierType == UA_NODEIDTYPE_NUMERIC)
			{
				if (targetNodeId->identifier.numeric == pObjectNodeInfo->vServerVarInfo[nOPCSendVarNo].tNodeID.identifier.numeric)
				{
					*(UA_Double*)(pObjectNodeInfo->vServerVarInfo[nOPCSendVarNo].pValue) = currentValue;
					pObjectNodeInfo->vServerVarInfo[nOPCSendVarNo].dPreValue = currentValue;
					break;
				}
			}
			else
			{
				if (strcmp(varName, pObjectNodeInfo->vServerVarInfo[nOPCSendVarNo].varName) == 0)
				{
					*(UA_Double*)(pObjectNodeInfo->vServerVarInfo[nOPCSendVarNo].pValue) = currentValue;
					pObjectNodeInfo->vServerVarInfo[nOPCSendVarNo].dPreValue = currentValue;
					break;
				}
			}
		}
		if (nOPCSendVarNo != pObjectNodeInfo->nServerVarSum)
		{
			break;
		}
	}
}

UA_UInt32 COPCClient::addMonitoredItemToDoubleVariable(UA_Client *client, UA_NodeId *pTargetNodeId)
{
	UA_CreateSubscriptionRequest request = UA_CreateSubscriptionRequest_default();
	UA_CreateSubscriptionResponse response = UA_Client_Subscriptions_create(client, request,
		NULL, NULL, NULL);

	UA_UInt32 subId = response.subscriptionId;
	if (response.responseHeader.serviceResult == UA_STATUSCODE_GOOD)
	{
		UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_CLIENT, "Create subscription succeeded, id %u\n", subId);
	}

	UA_MonitoredItemCreateResult result;

	UA_MonitoredItemCreateRequest monRequest = UA_MonitoredItemCreateRequest_default(*pTargetNodeId);

	monRequest.requestedParameters.samplingInterval = 100.0; // 100 ms interval
	result = UA_Client_MonitoredItems_createDataChange(client, response.subscriptionId, UA_TIMESTAMPSTORETURN_BOTH,
		monRequest, (void*)pTargetNodeId, dataChangeNotificationDoubleCallback_Client, NULL);

	delete pTargetNodeId;
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
	return -1;
}

void COPCClient::addMonitoredItem_DOUBLE(UA_Client *client, char *sVarName, std::map<CString, UA_NodeId> mTotalNodeID)
{
	UA_NodeId * myIntegerNodeId = new UA_NodeId;
	if (mTotalNodeID.find(sVarName) == mTotalNodeID.end())
	{
		*myIntegerNodeId = UA_NODEID_STRING(1, sVarName);
	}
	else
	{
		*myIntegerNodeId = mTotalNodeID[sVarName];
	}
	addMonitoredItemToDoubleVariable(client, myIntegerNodeId);
}

void dataChangeNotificationStringCallback_Client(UA_Client *client, UA_UInt32 subId,
	void *subContext, UA_UInt32 monId,
	void *monitoredItemContext, UA_DataValue *value)
{
	UA_NodeId * targetNodeId = (UA_NodeId*)monitoredItemContext;


	UA_String currentValue = *(UA_String*)(value->value.data);
	char *acTemp;
	acTemp = new char[currentValue.length + 1];
	strncpy(acTemp, (char *)currentValue.data, currentValue.length);
	acTemp[currentValue.length] = '\0';
	std::string strTemp = acTemp;
	std::wstring strVal = Utf8ToWstring(strTemp);
	strTemp = WString2String(strVal);


	char varName[100] = { 0 };
	if (targetNodeId->identifierType == UA_NODEIDTYPE_STRING)
	{
		strncpy(varName, (char*)targetNodeId->identifier.string.data, targetNodeId->identifier.string.length);
	}

	for (unsigned int n = 0; n < g_vtObjectNodeInfo.size(); n++)
	{
		ObjectNodeInfo *pObjectNodeInfo = &g_vtObjectNodeInfo[n];
		int nOPCSendVarNo = 0;

		for (nOPCSendVarNo = 0; nOPCSendVarNo < pObjectNodeInfo->nServerVarSum; nOPCSendVarNo++)
		{
			if (targetNodeId->identifierType == UA_NODEIDTYPE_NUMERIC)
			{
				if (targetNodeId->identifier.numeric == pObjectNodeInfo->vServerVarInfo[nOPCSendVarNo].tNodeID.identifier.numeric)
				{
					if (strTemp.length() < RECEIVE_MAX_CHAR_LENGHT)
					{
						pObjectNodeInfo->vServerVarInfo[nOPCSendVarNo].nValueLength = strTemp.length();
						strncpy((char*)(pObjectNodeInfo->vServerVarInfo[nOPCSendVarNo].pValue), strTemp.c_str(), strTemp.length());
						((char*)(pObjectNodeInfo->vServerVarInfo[nOPCSendVarNo].pValue))[strTemp.length()] = '\0';
						strncpy((char*)(pObjectNodeInfo->vServerVarInfo[nOPCSendVarNo].acPreValue), strTemp.c_str(), strTemp.length());
						pObjectNodeInfo->vServerVarInfo[nOPCSendVarNo].acPreValue[strTemp.length()] = '\0';
						break;
					}
				}
			}
			else
			{
				if (strcmp(varName, pObjectNodeInfo->vServerVarInfo[nOPCSendVarNo].varName) == 0)
				{
					if (strTemp.length() < RECEIVE_MAX_CHAR_LENGHT)
					{
						pObjectNodeInfo->vServerVarInfo[nOPCSendVarNo].nValueLength = strTemp.length();
						strncpy((char*)(pObjectNodeInfo->vServerVarInfo[nOPCSendVarNo].pValue), strTemp.c_str(), strTemp.length());
						((char*)(pObjectNodeInfo->vServerVarInfo[nOPCSendVarNo].pValue))[strTemp.length()] = '\0';
						strncpy((char*)(pObjectNodeInfo->vServerVarInfo[nOPCSendVarNo].acPreValue), strTemp.c_str(), strTemp.length());
						pObjectNodeInfo->vServerVarInfo[nOPCSendVarNo].acPreValue[strTemp.length()] = '\0';
						break;
					}
				}
			}
		}

		if (nOPCSendVarNo != pObjectNodeInfo->nServerVarSum)
		{
			break;
		}
	}
	delete[]acTemp;
}

UA_UInt32 COPCClient::addMonitoredItemToStringVariable(UA_Client *client, UA_NodeId *pTargetNodeId)
{
	UA_CreateSubscriptionRequest request = UA_CreateSubscriptionRequest_default();
	UA_CreateSubscriptionResponse response = UA_Client_Subscriptions_create(client, request,
		NULL, NULL, NULL);

	UA_UInt32 subId = response.subscriptionId;
	if (response.responseHeader.serviceResult == UA_STATUSCODE_GOOD)
	{
		UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_CLIENT, "Create subscription succeeded, id %u\n", subId);
	}

	UA_MonitoredItemCreateResult result;

	UA_MonitoredItemCreateRequest monRequest = UA_MonitoredItemCreateRequest_default(*pTargetNodeId);

	monRequest.requestedParameters.samplingInterval = 100.0; // 100 ms interval
	result = UA_Client_MonitoredItems_createDataChange(client, response.subscriptionId, UA_TIMESTAMPSTORETURN_BOTH,
		monRequest, (void*)pTargetNodeId, dataChangeNotificationStringCallback_Client, NULL);

	delete pTargetNodeId;
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

void COPCClient::addMonitoredItem_CHAR(UA_Client *client, char *sVarName, std::map<CString, UA_NodeId> mTotalNodeID)
{
	UA_NodeId * myIntegerNodeId = new UA_NodeId;
	if (mTotalNodeID.find(sVarName) == mTotalNodeID.end())
	{
		*myIntegerNodeId = UA_NODEID_STRING(1, sVarName);
	}
	else
	{
		*myIntegerNodeId = mTotalNodeID[sVarName];
	}
	addMonitoredItemToStringVariable(client, myIntegerNodeId);
}

void COPCClient::AddMonitoredItem(UA_Client* client, int nObjectOrderNumber, char* varName, int nVarType, std::map<CString, UA_NodeId> mTotalNodeID)
{
	/*
	#Boolean 			#define UA_TYPES_BOOLEAN 			0
	#Int32 				#define UA_TYPES_INT32 				5
	#Double 			#define UA_TYPES_DOUBLE 			10
	#String 			#define UA_TYPES_STRING 			11
	#DateTime 			#define UA_TYPES_DATETIME 			12*/
	switch (nVarType)
	{
	case 0:addMonitoredItem_BOOL(client, varName, mTotalNodeID); break;
	case 5:addMonitoredItem_INT(client, varName, mTotalNodeID); break;
	case 10:addMonitoredItem_DOUBLE(client, varName, mTotalNodeID); break;
	case 11:addMonitoredItem_CHAR(client, varName, mTotalNodeID); break;
	default:printf("不支持该类型变量创建：nVarType = %d\n", nVarType); break;
	}
}

void COPCClient::AddMonitoredItems(UA_Client* client, std::map<int, std::vector<UA_NodeId>> mNodeId)
{
	/*
	#Boolean 			#define UA_TYPES_BOOLEAN 			0
	#Int32 				#define UA_TYPES_INT32 				5
	#Double 			#define UA_TYPES_DOUBLE 			10
	#String 			#define UA_TYPES_STRING 			11
	#DateTime 			#define UA_TYPES_DATETIME 			12*/
	for (auto iter = mNodeId.begin(); iter != mNodeId.end(); iter++)
	{
		switch (iter->first)
		{
		case 0:addMonitoredItems(client, iter->second, dataChangeNotificationBoolCallback_Client); break;
		case 5:addMonitoredItems(client, iter->second, dataChangeNotificationInt32Callback_Client); break;
		case 10:addMonitoredItems(client, iter->second, dataChangeNotificationDoubleCallback_Client); break;
		case 11:addMonitoredItems(client, iter->second, dataChangeNotificationStringCallback_Client); break;
		default:printf("不支持该类型变量创建：nVarType = %d\n", iter->first); break;
		}
	}
}

int COPCClient::ModifyBooleanVal(bool bValue, VarInfo* pVarInfo)
{
	//char nodeIdName[1024] = { 0 };
	//strcpy(nodeIdName, pcValName);
	UA_Client* client = UA_Client_new();
	UA_ClientConfig_setDefault(UA_Client_getConfig(client));
	UA_StatusCode retval = UA_Client_connect(client, pVarInfo->acServerIP/*"opc.tcp://localhost:4840"*/);
	if (retval != UA_STATUSCODE_GOOD) {
		UA_Client_delete(client);
		return (int)retval;
	}

	UA_Variant value;
	UA_Variant_init(&value);

	// 变量的Node Id是string类型
	const UA_NodeId nodeId = pVarInfo->tNodeID;

	//使用UA_Client_readValueAttribute()去读取变量值
	retval = UA_Client_readValueAttribute(client, nodeId, &value);

	// 变量的数据类型是UA_Int32，第二个是判断UA_Variant里是否有UA_Int32类型的数据
	if (retval == UA_STATUSCODE_GOOD && UA_Variant_hasScalarType(&value, &UA_TYPES[UA_TYPES_BOOLEAN]))
	{
		UA_Boolean variableValue = *(UA_Boolean*)value.data;
		UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_USERLAND, "Variable Value is: %d\n", variableValue);
	}

	// 对client里的变量写入一个新值
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
			UA_Boolean variableValue = *(UA_Boolean*)value.data;
			UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_USERLAND, "New Variable Value is: %d\n", variableValue);
		}
	}

	// Clean up
	UA_Variant_clear(&value);
	UA_Client_delete(client); // Disconnects the client internally
	return EXIT_SUCCESS;
}

int COPCClient::ModifyIntVal(int iValue, VarInfo* pVarInfo)
{
	//char nodeIdName[1024] = { 0 };
	//strcpy(nodeIdName, pcValName);
	UA_Client* client = UA_Client_new();
	UA_ClientConfig_setDefault(UA_Client_getConfig(client));
	UA_StatusCode retval = UA_Client_connect(client, pVarInfo->acServerIP/*"opc.tcp://localhost:4840"*/);
	if (retval != UA_STATUSCODE_GOOD) {
		UA_Client_delete(client);
		return (int)retval;
	}

	UA_Variant value;
	UA_Variant_init(&value);

	// 变量的Node Id是string类型
	const UA_NodeId nodeId = pVarInfo->tNodeID;

	//使用UA_Client_readValueAttribute()去读取变量值
	retval = UA_Client_readValueAttribute(client, nodeId, &value);

	// 变量的数据类型是UA_Int32，第二个是判断UA_Variant里是否有UA_Int32类型的数据
	if (retval == UA_STATUSCODE_GOOD && UA_Variant_hasScalarType(&value, &UA_TYPES[UA_TYPES_INT32]))
	{
		UA_Int32 variableValue = *(UA_Int32*)value.data;
		UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_USERLAND, "Variable Value is: %d\n", variableValue);
	}

	// 对client里的变量写入一个新值
// 对client里的变量写入一个新值
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
			UA_Int32 variableValue = *(UA_Int32*)value.data;
			UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_USERLAND, "New Variable Value is: %d\n", variableValue);
		}
	}

	// Clean up
	UA_Variant_clear(&value);
	UA_Client_delete(client); // Disconnects the client internally
	return EXIT_SUCCESS;
}

int COPCClient::ModifyDoubleVal(double dValue, VarInfo* pVarInfo)
{
	UA_Client* client = UA_Client_new();
	UA_ClientConfig_setDefault(UA_Client_getConfig(client));
	UA_StatusCode retval = UA_Client_connect(client, pVarInfo->acServerIP/*"opc.tcp://localhost:4840"*/);
	if (retval != UA_STATUSCODE_GOOD) {
		UA_Client_delete(client);
		return (int)retval;
	}

	UA_Variant value;
	UA_Variant_init(&value);

	// 变量的Node Id是string类型
	const UA_NodeId nodeId = pVarInfo->tNodeID;

	//使用UA_Client_readValueAttribute()去读取变量值
	retval = UA_Client_readValueAttribute(client, nodeId, &value);

	// 变量的数据类型是UA_Int32，第二个是判断UA_Variant里是否有UA_Int32类型的数据
	if (retval == UA_STATUSCODE_GOOD && UA_Variant_hasScalarType(&value, &UA_TYPES[UA_TYPES_DOUBLE]))
	{
		UA_Double variableValue = *(UA_Double*)value.data;
		UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_USERLAND, "Variable Value is: %.3lf\n", variableValue);
	}

	// 对client里的变量写入一个新值
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
			UA_Double variableValue = *(UA_Double*)value.data;
			UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_USERLAND, "New Variable Value is: %.3lf\n", variableValue);
		}
	}

	// Clean up
	UA_Variant_clear(&value);
	UA_Client_delete(client); // Disconnects the client internally
	return EXIT_SUCCESS;
}

int COPCClient::ModifyBooleanVal(bool bValue, const char *pcValName, const char *pcServerIP)
{
	char nodeIdName[1024] = { 0 };
	strcpy(nodeIdName, pcValName);
	UA_Client *client = UA_Client_new();
	UA_ClientConfig_setDefault(UA_Client_getConfig(client));
	UA_StatusCode retval = UA_Client_connect(client, pcServerIP/*"opc.tcp://localhost:4840"*/);
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

	// 对client里的变量写入一个新值
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

int COPCClient::ModifyIntVal(int iValue, const char *pcValName, const char *pcServerIP)
{
	char nodeIdName[1024] = { 0 };
	strcpy(nodeIdName, pcValName);
	UA_Client *client = UA_Client_new();
	UA_ClientConfig_setDefault(UA_Client_getConfig(client));
	UA_StatusCode retval = UA_Client_connect(client, pcServerIP/*"opc.tcp://localhost:4840"*/);
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

	// 对client里的变量写入一个新值
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

int COPCClient::ModifyDoubleVal(double dValue, const char *pcValName, const char *pcServerIP)
{
	char nodeIdName[1024] = { 0 };
	strcpy(nodeIdName, pcValName);
	UA_Client *client = UA_Client_new();
	UA_ClientConfig_setDefault(UA_Client_getConfig(client));
	UA_StatusCode retval = UA_Client_connect(client, pcServerIP/*"opc.tcp://localhost:4840"*/);
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

	// 对client里的变量写入一个新值
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

int COPCClient::ModifyStrVal(std::string sValue, VarInfo* pVarInfo)
{
	UA_Client* client = UA_Client_new();
	UA_ClientConfig_setDefault(UA_Client_getConfig(client));
	UA_StatusCode retval = UA_Client_connect(client, pVarInfo->acServerIP/*"opc.tcp://localhost:4840"*/);
	if (retval != UA_STATUSCODE_GOOD) {
		UA_Client_delete(client);
		return (int)retval;
	}

	UA_Variant value;
	UA_Variant_init(&value);

	// 变量的Node Id是string类型
	const UA_NodeId nodeId = pVarInfo->tNodeID;

	//使用UA_Client_readValueAttribute()去读取变量值
	retval = UA_Client_readValueAttribute(client, nodeId, &value);

	// 变量的数据类型是UA_Int32，第二个是判断UA_Variant里是否有UA_Int32类型的数据
	if (retval == UA_STATUSCODE_GOOD && UA_Variant_hasScalarType(&value, &UA_TYPES[UA_TYPES_STRING]))
	{
		UA_String variableValue = *(UA_String*)value.data;
		//UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_USERLAND, "Variable Value is: %s\n", &variableValue);
	}

	// 对client里的变量写入一个新值
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
			UA_String variableValue = *(UA_String*)value.data;
			//UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_USERLAND, "New Variable Value is: %s\n", variableValue);
		}
	}

	// Clean up
	UA_Variant_clear(&value);
	UA_Client_delete(client); // Disconnects the client internally
	return EXIT_SUCCESS;
}

int COPCClient::ModifyStrVal(std::string sValue, const char *pcValName, const char *pcServerIP)
{
	char nodeIdName[1024] = { 0 };
	strcpy(nodeIdName, pcValName);
	UA_Client *client = UA_Client_new();
	UA_ClientConfig_setDefault(UA_Client_getConfig(client));
	UA_StatusCode retval = UA_Client_connect(client, pcServerIP/*"opc.tcp://localhost:4840"*/);
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

	// 对client里的变量写入一个新值
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

int COPCClient::ModifyTime(char *sVarName, SYSTEMTIME st, const char *pcServerIP)
{
	UA_Client *client = UA_Client_new();
	UA_ClientConfig_setDefault(UA_Client_getConfig(client));
	UA_StatusCode retval = UA_Client_connect(client, pcServerIP/*"opc.tcp://localhost:4840"*/);
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

	// 对client里的变量写入一个新值
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

bool COPCClient::GetVarVal(char *pcNarName, void *pValue)
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

bool COPCClient::GetVarVal(UA_Client *client, VarInfo * pVarInfo)
{
	// 变量类型：bool:0  int:5  double:10  string:11
	bool bRtn = false;
	UA_Variant value;
	UA_Variant_init(&value);

	// 变量的Node Id是string类型
	UA_NodeId nodeId = pVarInfo->tNodeID;

	//使用UA_Client_readValueAttribute()去读取变量值
	UA_StatusCode retval = UA_Client_readValueAttribute(client, nodeId, &value);

	// 判断UA_Variant里是否有 nVarType 类型的数据
	if (retval == UA_STATUSCODE_GOOD/* && UA_Variant_hasScalarType(&value, &UA_TYPES[pVarInfo->nVarType])*/)
	{
		if (0 == pVarInfo->nVarType)
		{
			UA_Boolean bVariableValue = *(UA_Boolean *)value.data;
			*(bool*)pVarInfo->pValue = bVariableValue;
			pVarInfo->bPreValue = bVariableValue;
		}
		else if (5 == pVarInfo->nVarType && pVarInfo->nArrayLength == 1)
		{
			UA_Int32 nVariableValue = *(UA_Int32 *)value.data;
			*(int*)pVarInfo->pValue = nVariableValue;
			pVarInfo->nPreValue = nVariableValue;
		}
		else if (5 == pVarInfo->nVarType && pVarInfo->nArrayLength != 1)
		{
			UA_Int16* adata = (UA_Int16*)(value.data);
			for (size_t i = 0; i < value.arrayLength; i++)
			{
				int temp = adata[i];
				((int*)pVarInfo->pValue)[i] = adata[i];
				pVarInfo->anPreValue[i] = adata[i];
			}
		}
		else if (10 == pVarInfo->nVarType)
		{
			UA_Double dVariableValue = *(UA_Double *)value.data;
			*(double*)pVarInfo->pValue = dVariableValue;
			pVarInfo->dPreValue = dVariableValue;
		}
		else if (11 == pVarInfo->nVarType)
		{
			UA_String sVariableValue = *(UA_String *)value.data;
			strncpy((char*)pVarInfo->pValue, (const char*)sVariableValue.data, sVariableValue.length);
			((char*)pVarInfo->pValue)[sVariableValue.length] = '\0';
			strncpy(pVarInfo->acPreValue, (const char*)sVariableValue.data, sVariableValue.length);
			pVarInfo->acPreValue[sVariableValue.length] = '\0';
		}
		else
		{
			bRtn = false;
		}
		bRtn = true;
	}
	else
	{
		bRtn = false;
	}
	UA_Variant_clear(&value);
	return bRtn;
}

bool COPCClient::GetAllVarVal()
{
	for (unsigned int n = 0; n < g_vtObjectNodeInfo.size(); n++)
	{
		UA_Client *client = UA_Client_new();
		UA_ClientConfig_setDefault(UA_Client_getConfig(client));	
		char acServerIP[100];
		sprintf(acServerIP, "opc.tcp://%s:%s\0", g_vtObjectNodeInfo[n].acIP, g_vtObjectNodeInfo[n].acPort);
		UA_StatusCode retval = UA_Client_connect(client, acServerIP);
		if (retval != UA_STATUSCODE_GOOD) 
		{
			UA_Client_delete(client);
			break;
		}
		for (int m = 0; m < g_vtObjectNodeInfo[n].nServerVarSum; m++)
		{
			GetVarVal(client,&g_vtObjectNodeInfo[n].vServerVarInfo[m]);
		}
		UA_Client_delete(client);
	}
	return true;
}

void COPCClient::CleanNodeInfo(vector<ObjectNodeInfo> &vtObjectNodeInfo)
{
	for (unsigned int n = 0; n < g_vtObjectNodeInfo.size(); n++)
	{
		for (int m = 0; m < g_vtObjectNodeInfo[n].nServerVarSum; m++)
		{
			delete g_vtObjectNodeInfo[n].vServerVarInfo[m].pValue;
		}
		g_vtObjectNodeInfo[n].vServerVarInfo.clear();
	}
	vtObjectNodeInfo.clear();
}

void COPCClient::addMonitoredItemFromFile(UA_Client* client, ObjectNodeInfo* vtObjectNodeInfo, std::map<CString, UA_NodeId> mTotalNodeID)
{
	// 添加客户端接收变量
	for (int nServerRecvVarNo = 0; nServerRecvVarNo < vtObjectNodeInfo->nServerVarSum; nServerRecvVarNo++)
	{
		AddMonitoredItem(client, vtObjectNodeInfo->nOrderNumber, vtObjectNodeInfo->vServerVarInfo[nServerRecvVarNo].varName,
			vtObjectNodeInfo->vServerVarInfo[nServerRecvVarNo].nVarType, mTotalNodeID);
		if (false == GetVarVal(client, &vtObjectNodeInfo->vServerVarInfo[nServerRecvVarNo]))
		{
			WriteLog("%s 节点 %s 变量读取失败！", vtObjectNodeInfo->objectName, vtObjectNodeInfo->vServerVarInfo[nServerRecvVarNo].varName);
		}
	}
}


void COPCClient::addMonitoredItemFromFiles(UA_Client* client, ObjectNodeInfo* vtObjectNodeInfo, std::map<CString, UA_NodeId> mTotalNodeID)
{
	// 添加客户端接收变量
	for (int nServerRecvVarNo = 0; nServerRecvVarNo < vtObjectNodeInfo->nServerVarSum; nServerRecvVarNo++)
	{
		if (mTotalNodeID.find(vtObjectNodeInfo->vServerVarInfo[nServerRecvVarNo].varName) != mTotalNodeID.end())
		{
			m_mNodeId[vtObjectNodeInfo->vServerVarInfo[nServerRecvVarNo].nVarType].push_back(mTotalNodeID[vtObjectNodeInfo->vServerVarInfo[nServerRecvVarNo].varName]);
		}
		else
		{
			m_mNodeId[vtObjectNodeInfo->vServerVarInfo[nServerRecvVarNo].nVarType].push_back(UA_NODEID_STRING(1, vtObjectNodeInfo->vServerVarInfo[nServerRecvVarNo].varName));
		}
		if (false == GetVarVal(client, &vtObjectNodeInfo->vServerVarInfo[nServerRecvVarNo]))
		{
			WriteLog("%s 节点 %s 变量读取失败！", vtObjectNodeInfo->objectName, vtObjectNodeInfo->vServerVarInfo[nServerRecvVarNo].varName);
		}
	}
	AddMonitoredItems(client, m_mNodeId);
}

UINT COPCClient::ThreadRunOPCDaemon(void *pParam)
{
	COPCClient *pObj = (COPCClient*)pParam;
	int i;
	pObj->m_bThreadRunOPCDaemon = true;
	while (!pObj->m_bServerQuit)
	{
		for (unsigned int n = 0; n < g_vtObjectNodeInfo.size(); n++)
		{
			ObjectNodeInfo *objectNodeInfo = &g_vtObjectNodeInfo[n];
			for (i = 0; i < objectNodeInfo->nServerVarSum; i++)
			{
				int nOffsetLength = pObj->CheckDataChange(&(objectNodeInfo->vServerVarInfo[i]));
			}
		}
		Sleep(50);
	}
	pObj->m_bThreadRunOPCDaemon = false;
	return 0;
}

//OPC客户端运行线程
UINT COPCClient::ThreadRunOPCClient(void *pParam)
{
	//传入自定义参数
	T_CONNECT_OPC_THREAD_DATA_CLIENT pObj = *((T_CONNECT_OPC_THREAD_DATA_CLIENT*)pParam);
	delete pParam;

	//线程状态置true
	pObj.ptObjectNodeInfo->bTreadState = true;

	//连接状态置false
	pObj.ptObjectNodeInfo->bConnectState = false;

	//尝试连接服务器
	UA_Client  *pDeviceClient;
	pDeviceClient = UA_Client_new();
	UA_ClientConfig_setDefault(UA_Client_getConfig(pDeviceClient));
	char acServerIP[100];
	sprintf(acServerIP, "opc.tcp://%s:%s\0", pObj.ptObjectNodeInfo->acIP, pObj.ptObjectNodeInfo->acPort);
	UA_StatusCode retval = UA_Client_connect(pDeviceClient, acServerIP);

	if (retval != UA_STATUSCODE_GOOD)
	{
		//连接失败，释放客户端连接
		UA_Client_delete(pDeviceClient);

		if (pObj.pFather->m_bRunning && pObj.ptObjectNodeInfo->bTryToConnect)
		{
			;
			//连接失败，再次尝试
			/*if(pObj.pFather->flagFalse++<5)
			pObj.pFather->StartClient(*pObj.ptObjectNodeInfo);*/
		}
		else
		{
			//连接失败，停止尝试，线程状态置false
			pObj.ptObjectNodeInfo->bTreadState = false;
			WriteLog("%s 节点监视线程退出！", pObj.ptObjectNodeInfo->objectName);
		}
		return (int)retval;
	}

	//连接成功，连接状态置true
	pObj.ptObjectNodeInfo->bConnectState = true;

	std::map<CString, UA_NodeId> mTotalNodeID;
	std::vector<CString> vstrNodePath = TokenizeCString(pObj.ptObjectNodeInfo->acNodePath, "\\");

	fingtotalnodeid(pDeviceClient, mTotalNodeID, UA_NODEID_NUMERIC(0, UA_NS0ID_OBJECTSFOLDER));
	for (size_t i = 0; i < vstrNodePath.size(); i++)
	{
		if (mTotalNodeID.find(vstrNodePath[i]) != mTotalNodeID.end())
		{
			fingtotalnodeid(pDeviceClient, mTotalNodeID, mTotalNodeID[vstrNodePath[i]]);
		}
	}
	for (size_t i = 0; i < pObj.ptObjectNodeInfo->vServerVarInfo.size(); i++)
	{
		if (mTotalNodeID.find(pObj.ptObjectNodeInfo->vServerVarInfo[i].varName) != mTotalNodeID.end())
		{
			pObj.ptObjectNodeInfo->vServerVarInfo[i].tNodeID = mTotalNodeID[pObj.ptObjectNodeInfo->vServerVarInfo[i].varName];
		}
	}
	//添加监视变量（订阅回调函数）
	pObj.pFather->addMonitoredItemFromFiles(pDeviceClient, pObj.ptObjectNodeInfo, mTotalNodeID);

	//获取变量值并检查连接状态
	while (pObj.pFather->m_bRunning && pObj.ptObjectNodeInfo->bTryToConnect)
	{
		UA_SecureChannelState channelState;
		UA_StatusCode connectStatus = -1;
		UA_Client_run_iterate(pDeviceClient, 0);
		UA_Client_getState(pDeviceClient, &channelState, NULL, &connectStatus);
		if (connectStatus != 0 || channelState != UA_SECURECHANNELSTATE_OPEN)
		{
			break;
		}
		Sleep(10);
	}

	//释放客户端连接
	UA_Client_delete(pDeviceClient);

	//连接状态置false
	pObj.ptObjectNodeInfo->bConnectState = false;


	if (pObj.pFather->m_bRunning && pObj.ptObjectNodeInfo->bTryToConnect)
	{
		//连接断开，再次尝试
		pObj.pFather->StartClient(*pObj.ptObjectNodeInfo);
	}
	else
	{
		//连接断开，停止尝试，线程状态置false
		pObj.ptObjectNodeInfo->bTreadState = false;
		WriteLog("%s 节点监视线程退出！", pObj.ptObjectNodeInfo->objectName);
	}

	return retval == UA_STATUSCODE_GOOD ? EXIT_SUCCESS : EXIT_FAILURE;
}

bool COPCClient::SetInitVal(VarInfo &varInfo)
{
	switch (varInfo.nVarType)
	{
	case 0:
		varInfo.pValue = new bool;
		*(bool *)varInfo.pValue = false;
		varInfo.bPreValue = *(bool *)varInfo.pValue;
		break;
	case 5:
		if (varInfo.nArrayLength > 1)
		{
			varInfo.pValue = new int[varInfo.nArrayLength];
			*(int*)varInfo.pValue = 0;
			break;
		}
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
		varInfo.pValue = new char[RECEIVE_MAX_CHAR_LENGHT];
		strcpy((char *)varInfo.pValue, "null\0");
		strncpy(varInfo.acPreValue, (char *)varInfo.pValue, strlen((char *)varInfo.pValue) + 1);
		break;
	case 12:
		varInfo.pValue = new char[40];
		strcpy((char *)varInfo.pValue, "null\0");
		strncpy(varInfo.acPreValue, (char *)varInfo.pValue, strlen((char *)varInfo.pValue) + 1);
		break;
	default:
		return false;
		break;
	}
	return true;
}

std::vector<CString> COPCClient::GetAllOPCFilePath()
{
	return GetTotalFilePath(OPC_SERVER_FILE_PATH.c_str(), "*.ini");
}

bool COPCClient::ReadWorkStationParam(std::vector<CString> vstrFilePath)
{
	CleanNodeInfo(g_vtObjectNodeInfo);
	ObjectNodeInfo tObjectNodeInfo;
	bool bRtn = true;
	for (unsigned int n = 0; n < vstrFilePath.size(); n++)
	{
		bRtn = bRtn && ReadWorkStationParam(vstrFilePath[n], tObjectNodeInfo);
		g_vtObjectNodeInfo.push_back(tObjectNodeInfo);
	}
	return bRtn;
}

bool COPCClient::ReadWorkStationParam(CString strFilePath, ObjectNodeInfo &tObjectNodeInfo)
{
	FILE *pfServerParam = fopen(strFilePath, "r");
	if (NULL == pfServerParam)
	{
		return false;
	}
	int nObjectSum = 0;
	string sTemp;
	fscanf(pfServerParam, "ObjectSum:%d\n", &nObjectSum);
	memset(&tObjectNodeInfo, 0, sizeof(tObjectNodeInfo));
	fscanf(pfServerParam, "IP:%s\n", &tObjectNodeInfo.acIP);
	fscanf(pfServerParam, "Port:%s\n", &tObjectNodeInfo.acPort);
	fscanf(pfServerParam, "NodePath:%s\n", &tObjectNodeInfo.acNodePath);
	fscanf(pfServerParam, "nServerVarSum:%d\n", &(tObjectNodeInfo.nServerVarSum));

	// 工作站上传变量
	for (int nVarNo = 0; nVarNo < tObjectNodeInfo.nServerVarSum; nVarNo++)
	{
		VarInfo varInfo;
		varInfo.nArrayLength = 100;
		fscanf(pfServerParam, "%s%d\n", varInfo.varOrgName, &varInfo.nVarType);
		sprintf(varInfo.varName, "%s", varInfo.varOrgName);
		SetInitVal(varInfo);
		sprintf(varInfo.acServerIP, "opc.tcp://%s:%s\0", tObjectNodeInfo.acIP, tObjectNodeInfo.acPort);
		varInfo.bChangeVal = false;
		tObjectNodeInfo.vServerVarInfo.push_back(varInfo);

	}
	fclose(pfServerParam);

	tObjectNodeInfo.bTryToConnect = true;
	tObjectNodeInfo.bConnectState = false;
	tObjectNodeInfo.bTreadState = false;
	return true;
}

UINT COPCClient::ThreadChangeVal(void *pParam)
{	
	//传入自定义参数
	T_CONNECT_OPC_THREAD_DATA_CLIENT pObj = *((T_CONNECT_OPC_THREAD_DATA_CLIENT*)pParam);
	delete pParam;

	int nVarType = pObj.ptObjectValInfo->nVarType;
	UA_StatusCode retval = -1;

	switch (nVarType)
	{
	case 0:
		retval = pObj.pFather->ModifyBooleanVal(*((bool*)pObj.ptObjectValInfo->pValue), pObj.ptObjectValInfo);
		if (retval == UA_STATUSCODE_GOOD)
		{
			pObj.ptObjectValInfo->bPreValue = *((bool*)pObj.ptObjectValInfo->pValue);
		}
		break;
	case 5:
		retval = pObj.pFather->ModifyIntVal(*((int*)pObj.ptObjectValInfo->pValue), pObj.ptObjectValInfo);
		if (retval == UA_STATUSCODE_GOOD)
		{
			pObj.ptObjectValInfo->nPreValue = *((int*)pObj.ptObjectValInfo->pValue);
		}
		break;
	case 10:
		retval = pObj.pFather->ModifyDoubleVal(*((double*)pObj.ptObjectValInfo->pValue), pObj.ptObjectValInfo);
		if (retval == UA_STATUSCODE_GOOD)
		{
			pObj.ptObjectValInfo->dPreValue = *((double*)pObj.ptObjectValInfo->pValue);
		}
		break;
	case 11:
		retval = pObj.pFather->ModifyStrVal((char*)pObj.ptObjectValInfo->pValue, pObj.ptObjectValInfo);
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

int COPCClient::CheckDataChange(VarInfo * pVarInfo) // 根据varType将 数据长度 和 数据组合成buffer 并返回两部分长度和
{
	// 【变量名长	变量名		类型	数据长度	数据】 ……
	//		4		  n			 4			4		  n	   ……
	// 		%4d		  %s		 %4d		%4d		  ?	   ……
	int nVarType = pVarInfo->nVarType;
	void *pValue = pVarInfo->pValue;
	UA_StatusCode retval = -1;
	bool bChange = false;
	switch (nVarType)
	{
	case 0:
		if (*((bool*)pValue) != pVarInfo->bPreValue && !pVarInfo->bChangeVal)
		{
			pVarInfo->bChangeVal = true;
			T_CONNECT_OPC_THREAD_DATA_CLIENT *ptThreadData = new T_CONNECT_OPC_THREAD_DATA_CLIENT;
			ptThreadData->nThreadID = 1000;
			ptThreadData->ptObjectValInfo = pVarInfo;
			ptThreadData->pFather = this;
			AfxBeginThread(ThreadChangeVal, (void*)ptThreadData);
		}
		break;
	case 5:
		if (*((int*)pValue) != pVarInfo->nPreValue && !pVarInfo->bChangeVal)
		{
			pVarInfo->bChangeVal = true;
			T_CONNECT_OPC_THREAD_DATA_CLIENT *ptThreadData = new T_CONNECT_OPC_THREAD_DATA_CLIENT;
			ptThreadData->nThreadID = 1001;
			ptThreadData->ptObjectValInfo = pVarInfo;
			ptThreadData->pFather = this;
			AfxBeginThread(ThreadChangeVal, (void*)ptThreadData);
		}
		break;
	case 10:
		if (fabs(*((double*)pValue) - pVarInfo->dPreValue) > 0.01 && !pVarInfo->bChangeVal)
		{
			pVarInfo->bChangeVal = true;
			T_CONNECT_OPC_THREAD_DATA_CLIENT *ptThreadData = new T_CONNECT_OPC_THREAD_DATA_CLIENT;
			ptThreadData->nThreadID = 1002;
			ptThreadData->ptObjectValInfo = pVarInfo;
			ptThreadData->pFather = this;
			AfxBeginThread(ThreadChangeVal, (void*)ptThreadData);
		}
		break;
	case 11:
		if (strcmp((char*)pValue, pVarInfo->acPreValue) != 0 && !pVarInfo->bChangeVal)
		{
			pVarInfo->bChangeVal = true;
			T_CONNECT_OPC_THREAD_DATA_CLIENT *ptThreadData = new T_CONNECT_OPC_THREAD_DATA_CLIENT;
			ptThreadData->nThreadID = 1003;
			ptThreadData->ptObjectValInfo = pVarInfo;
			ptThreadData->pFather = this;
			AfxBeginThread(ThreadChangeVal, (void*)ptThreadData);
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