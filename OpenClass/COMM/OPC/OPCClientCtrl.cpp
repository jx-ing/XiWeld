#include "stdafx.h"
#include "OPCClientCtrl.h"

COPCClientCtrl::COPCClientCtrl()
{
	m_bConnect = true;
	InitOPCServer();
}


COPCClientCtrl::~COPCClientCtrl()
{
	if (m_bStartHeartbeatDetectionvoid == true)
	{
		m_bStartHeartbeatDetectionvoid = false;
		long long nTime = XI_clock();
		while (!m_bStartHeartbeatDetectionvoid)
		{
			if (XI_clock() - nTime > 2500)
			{
				break;
			}
			Sleep(50);
		}
	}
	if (NULL == g_atHeartbest)
	{
		delete[]g_atHeartbest;
		g_atHeartbest = NULL;
	}
	
	DELETE_POINTER(m_pOPCClient);
}


void COPCClientCtrl::InitOPCServer()
{

	m_pOPCClient = new COPCClient;
	m_pOPCClient->StartAllClient();

	BindData(m_atStationState,7,1);
	BindData(m_atStationState1,6,1);
	BindDataHeartbest();
//	BindDataBusiness();


	m_pOPCClient->StartRunOPCDaemon();


	//开启所有节点心跳
	//for (unsigned int n = 0; n < g_vtObjectNodeInfo.size(); n++)
	//{
	//	T_HEARTBEAT_THREAD_DATA *ptThreadData = new T_HEARTBEAT_THREAD_DATA;
	//	ptThreadData->nThreadID = n;
	//	ptThreadData->pHeartbeatData = &g_atHeartbest[n];
	//	ptThreadData->pFather = this;
	//	AfxBeginThread(ThreadHeartbeatDetectionvoid, ptThreadData);
	//}

}

void COPCClientCtrl::BindData(VarInfo &ServerVarInfo, const char *cName, bool &Var)
{
	if (!strcmp(ServerVarInfo.varOrgName, cName))
	{
		Var = *(bool *)ServerVarInfo.pValue;
		delete ServerVarInfo.pValue;
		ServerVarInfo.pValue = &Var;
	}
}

void COPCClientCtrl::BindData(VarInfo &ServerVarInfo, const char *cName, short*Var)
{
	if (!strcmp(ServerVarInfo.varOrgName, cName))
	{
		delete ServerVarInfo.pValue;
		ServerVarInfo.pValue = Var;
	}
}

void COPCClientCtrl::BindData(VarInfo& ServerVarInfo, const char* cName, int& Var)
{
	if (!strcmp(ServerVarInfo.varOrgName, cName))
	{
		Var = *(int*)ServerVarInfo.pValue;
		delete ServerVarInfo.pValue;
		ServerVarInfo.pValue = &Var;
	}
}

void COPCClientCtrl::BindData(VarInfo &ServerVarInfo, const char *cName, double &Var)
{
	if (!strcmp(ServerVarInfo.varOrgName, cName))
	{
		Var = *(double *)ServerVarInfo.pValue;
		delete ServerVarInfo.pValue;
		ServerVarInfo.pValue = &Var;
	}
}

void COPCClientCtrl::BindData(VarInfo &ServerVarInfo, const char *cName, char *Var)
{
	if (!strcmp(ServerVarInfo.varOrgName, cName))
	{
		strncpy(Var, (char *)ServerVarInfo.pValue, strlen((char *)ServerVarInfo.pValue) + 1);
		delete ServerVarInfo.pValue;
		ServerVarInfo.pValue = Var;
	}
}

void COPCClientCtrl::BindData(ObjectNodeInfo &vtNodeValInfo, T_HEARTBEAT &tHeartbest)
{
	int nVaeNum = vtNodeValInfo.nServerVarSum;
	for (int nVarNo = 0; nVarNo < nVaeNum; nVarNo++)
	{
		BindData(vtNodeValInfo.vServerVarInfo[nVarNo], "HB_SERVER_ACC", tHeartbest.nHB_SERVER_ACC);
		BindData(vtNodeValInfo.vServerVarInfo[nVarNo], "HB_CLIENT_ACC", tHeartbest.nHB_CLIENT_ACC);
	}
	tHeartbest.nNodeNo = vtNodeValInfo.nOrderNumber;
	tHeartbest.strNodeName = vtNodeValInfo.objectName;
}

void COPCClientCtrl::BindDataHeartbest()
{
	g_atHeartbest = new T_HEARTBEAT[g_vtObjectNodeInfo.size()];
	for (unsigned int n = 0; n < g_vtObjectNodeInfo.size(); n++)
	{
		BindData(g_vtObjectNodeInfo[n], g_atHeartbest[n]);
	}
}

void COPCClientCtrl::BindData(T_STATION_STATE* tStationState,int n, int id)
{
	int nVaeNum = g_vtObjectNodeInfo[0].nServerVarSum;
	for (int nVarNo = 0; nVarNo < nVaeNum; nVarNo++)
	{
		if (n==7)
		{
			for (size_t i = 0; i < 7; i++)
			{
				BindData(g_vtObjectNodeInfo[0].vServerVarInfo[nVarNo], GetStr("PM%d0BS1%d_WCS_released", id,i + 1).GetBuffer(), tStationState[i].bWCS_released);
				BindData(g_vtObjectNodeInfo[0].vServerVarInfo[nVarNo], GetStr("PM%d0BS1%d_Robot_released", id, i + 1).GetBuffer(), tStationState[i].bRobot_released);
				BindData(g_vtObjectNodeInfo[0].vServerVarInfo[nVarNo], GetStr("PM%d0BS1%d_PlateCode", id, i + 1).GetBuffer(), tStationState[i].acPlateCode);
				BindData(g_vtObjectNodeInfo[0].vServerVarInfo[nVarNo], GetStr("PM%d0BS1%d_Occupied", id, i + 1).GetBuffer(), tStationState[i].bOccupied);
			}
		}

		if (n==6)
		{
			for (size_t i = 0; i < 6; i++)
			{
				BindData(g_vtObjectNodeInfo[0].vServerVarInfo[nVarNo], GetStr("PM%d0BS2%d_AGV_released", id, i + 1).GetBuffer(), tStationState[i].bAGV_released);
				BindData(g_vtObjectNodeInfo[0].vServerVarInfo[nVarNo], GetStr("PM%d0BS2%d_Robot_released", id, i + 1).GetBuffer(), tStationState[i].bRobot_released);
				BindData(g_vtObjectNodeInfo[0].vServerVarInfo[nVarNo], GetStr("PM%d0BS2%d_PlateCode", id, i + 1).GetBuffer(), tStationState[i].acPlateCode);
				BindData(g_vtObjectNodeInfo[0].vServerVarInfo[nVarNo], GetStr("PM%d0BS2%d_Occupied", id, i + 1).GetBuffer(), tStationState[i].bOccupied);
			}
		}
	}
}

UINT COPCClientCtrl::ThreadHeartbeatDetectionvoid(void *pParam)
{
	T_HEARTBEAT_THREAD_DATA pObj = *((T_HEARTBEAT_THREAD_DATA*)pParam);
	delete pParam;
	pObj.pFather->HeartbeatDetectionvoid(pObj.pHeartbeatData);
	return 0;
}

void COPCClientCtrl::HeartbeatDetectionvoid(T_HEARTBEAT *pHeartbeatData)
{
	m_bStartHeartbeatDetectionvoid = true;
	pHeartbeatData->bStartHeartbeatDetectionvoid = true;
	int nPlcTimes = pHeartbeatData->nHB_SERVER_ACC;
	pHeartbeatData->lLostConnectCount = 0;
	bool bShowMark = true;
	while (pHeartbeatData->bStartHeartbeatDetectionvoid)
	{
		if (!m_bStartHeartbeatDetectionvoid)
		{
			break;
		}
		pHeartbeatData->nHB_CLIENT_ACC++;
		if (pHeartbeatData->nHB_CLIENT_ACC >= 1000)
		{
			pHeartbeatData->nHB_CLIENT_ACC = 0;
		}

		if (nPlcTimes != pHeartbeatData->nHB_SERVER_ACC)
		{
			nPlcTimes = pHeartbeatData->nHB_SERVER_ACC;
			pHeartbeatData->bOPCConnectState = true;
			if (bShowMark == true && pHeartbeatData->lLostConnectCount != 0)
			{
				WriteOPCLog("已重新连接至 %s OPC服务器，当前失联时间 %d s！", pHeartbeatData->strNodeName, pHeartbeatData->lLostConnectCount);
				bShowMark = false;
			}
			pHeartbeatData->lLostConnectCount = 0;
		}
		else
		{
			pHeartbeatData->lLostConnectCount++;
			if (pHeartbeatData->lLostConnectCount > 3)
			{
				pHeartbeatData->bOPCConnectState = false;
				if (bShowMark == false)
				{
					bShowMark = true;
					WriteOPCLog("与 %s OPC服务器失去连接！", pHeartbeatData->strNodeName);
				}
			}
		}

		Sleep(1000);

	}
	pHeartbeatData->bStartHeartbeatDetectionvoid = true;
	m_bStartHeartbeatDetectionvoid = true;
}
