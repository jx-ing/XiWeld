#pragma once
#include "OPCClient.h"

class COPCClientCtrl
{
public:
	COPCClientCtrl();
	~COPCClientCtrl();

	void InitOPCServer();
	COPCClient *m_pOPCClient = NULL;

	//-----------------------变量绑定-----------------------//
	void BindData(ObjectNodeInfo &vtNodeValInfo,T_HEARTBEAT &tHeartbest);//绑定心跳数据
	void BindDataHeartbest();//绑定心跳数据

	T_STATION_STATE m_atStationState[7];//立库数据
	T_STATION_STATE m_atStationState1[6];//二次盘数据
	/* id表示是第几个设备（配盘共有俩，分别为1和2）  */
	void BindData(T_STATION_STATE* tStationState,int n,int id);


	//-----------------------心跳检测-----------------------//
	static UINT ThreadHeartbeatDetectionvoid(void *pParam);
	void HeartbeatDetectionvoid(T_HEARTBEAT	*pHeartbeatData);

	bool m_bConnect;
	bool m_bStartHeartbeatDetectionvoid = false;
	long m_lLostConnectCount = 0;


private:
	void BindData(VarInfo &ServerVarInfo, const char *cName, bool &Var);
	void BindData(VarInfo& ServerVarInfo, const char* cName, short* Var);
	void BindData(VarInfo &ServerVarInfo, const char *cName, char *Var);
	void BindData(VarInfo &ServerVarInfo, const char *cName, int &Var);
	void BindData(VarInfo &ServerVarInfo, const char *cName, double &Var);

	int m_nGlobalCmdId;

	
};

struct T_HEARTBEAT_THREAD_DATA
{
	COPCClientCtrl *pFather;			//父指针
	int nThreadID;						//线程ID
	T_HEARTBEAT	*pHeartbeatData;		//心跳数据
};
