#include "stdafx.h"
#include "OPCServerCtrl.h"

COPCServerCtrl::COPCServerCtrl()
{
	m_bConnect = true;
}

COPCServerCtrl::~COPCServerCtrl()
{
	if (m_bStartHeartbeatDetectionvoid == true)
	{
		m_bStartHeartbeatDetectionvoid = false;
		long long nTime = XI_clock();
		while (!m_bStartHeartbeatDetectionvoid)
		{
			if (XI_clock() - nTime > 500)
			{
				break;
			}
			Sleep(50);
		}
	}
}

void COPCServerCtrl::InitOPCServer()
{
	m_pOPCServer = new COPCServer;
	m_pOPCServer->StartServer();

	BindData(m_tHeartbest);
	m_pOPCServer->StartRunOPCDaemon();

	AfxBeginThread(ThreadHeartbeatDetectionvoid, this);

}

void COPCServerCtrl::BindData(VarInfo &ServerVarInfo, const char *cName, bool &Var)
{
	if (!strcmp(ServerVarInfo.varOrgName, cName))
	{
		Var = *(bool *)ServerVarInfo.pValue;
		delete ServerVarInfo.pValue;
		ServerVarInfo.pValue = &Var;
	}
}

void COPCServerCtrl::BindData(VarInfo &ServerVarInfo, const char *cName, int &Var)
{
	if (!strcmp(ServerVarInfo.varOrgName, cName))
	{
		Var = *(int *)ServerVarInfo.pValue;
		delete ServerVarInfo.pValue;
		ServerVarInfo.pValue = &Var;
	}
}

void COPCServerCtrl::BindData(VarInfo &ServerVarInfo, const char *cName, double &Var)
{
	if (!strcmp(ServerVarInfo.varOrgName, cName))
	{
		Var = *(double *)ServerVarInfo.pValue;
		delete ServerVarInfo.pValue;
		ServerVarInfo.pValue = &Var;
	}
}

void COPCServerCtrl::BindData(VarInfo &ServerVarInfo, const char *cName, char *Var)
{
	if (!strcmp(ServerVarInfo.varOrgName, cName))
	{
		strncpy(Var, (char *)ServerVarInfo.pValue, strlen((char *)ServerVarInfo.pValue) + 1);
		delete ServerVarInfo.pValue;
		ServerVarInfo.pValue = Var;
	}
}

void COPCServerCtrl::BindData(T_HEARTBEST &tHeartbest)
{
	int nVaeNum = g_vtObjectNodeInfo[0].nServerVarSum;
	for (int nVarNo = 0; nVarNo < nVaeNum; nVarNo++)
	{
		BindData(g_vtObjectNodeInfo[0].vServerVarInfo[nVarNo], "HB_SERVER_ACC", tHeartbest.nHB_SERVER_ACC);
		BindData(g_vtObjectNodeInfo[0].vServerVarInfo[nVarNo], "HB_CLIENT_ACC", tHeartbest.nHB_CLIENT_ACC);
	}
}

void COPCServerCtrl::BindData(T_STATION_STATE* tStationState)
{
	int nVaeNum = g_vtObjectNodeInfo[0].nServerVarSum;
	for (int nVarNo = 0; nVarNo < nVaeNum; nVarNo++)
	{
		for (size_t i = 0; i < 9; i++)
		{
			BindData(g_vtObjectNodeInfo[0].vServerVarInfo[nVarNo], GetStr("P22BS1%d_AGV_released", i + 1).GetBuffer(), tStationState[i].bAGV_released);
			BindData(g_vtObjectNodeInfo[0].vServerVarInfo[nVarNo], GetStr("P22BS1%d_Robot_released", i + 1).GetBuffer(), tStationState[i].bRobot_released);
			BindData(g_vtObjectNodeInfo[0].vServerVarInfo[nVarNo], GetStr("P22BS1%d_PlateCode", i + 1).GetBuffer(), tStationState[i].acPlateCode);
			BindData(g_vtObjectNodeInfo[0].vServerVarInfo[nVarNo], GetStr("P22BS1%d_Occupied", i + 1).GetBuffer(), tStationState[i].bOccupied);
		}
	}
}

UINT COPCServerCtrl::ThreadHeartbeatDetectionvoid(void *pParam)
{
	COPCServerCtrl *pMyParam = ((COPCServerCtrl*)pParam);
	pMyParam->HeartbeatDetectionvoid();
	return 0;
}

void COPCServerCtrl::HeartbeatDetectionvoid()
{
	m_bStartHeartbeatDetectionvoid = true;
	int nSysTimes = m_tHeartbest.nHB_CLIENT_ACC;
	m_lLostConnectCount = 0;
	bool bShowMark = true;
	while (m_bStartHeartbeatDetectionvoid)
	{
		m_tHeartbest.nHB_SERVER_ACC++;
		if (m_tHeartbest.nHB_SERVER_ACC >= 1000)
		{
			m_tHeartbest.nHB_SERVER_ACC = 0;
		}

		if (nSysTimes != m_tHeartbest.nHB_CLIENT_ACC)
		{
			nSysTimes = m_tHeartbest.nHB_CLIENT_ACC;
			SetOPCConnectState(true);
			if (bShowMark == true && m_lLostConnectCount != 0)
			{
				WriteOPCLog("已重新连接至OPC服务器，当前失联时间 %d s！", m_lLostConnectCount);
				bShowMark = false;
			}
			m_lLostConnectCount = 0;
		}
		else
		{
			m_lLostConnectCount++;
			if (m_lLostConnectCount > 5)
			{
				SetOPCConnectState(false);
				if (bShowMark == false)
				{
					bShowMark = true;
					WriteOPCLog("与OPC服务器失去连接！");
				}
			}
		}

		Sleep(1000);

	}
	m_bStartHeartbeatDetectionvoid = true;

}
