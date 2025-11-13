#pragma once
#include "OPCServer.h"

#ifdef STRUCT_T_HEARTBEST
#else
#define STRUCT_T_HEARTBEST
typedef struct
{
	int nHB_SERVER_ACC;//四位
	int nHB_CLIENT_ACC;//四位
}T_HEARTBEST;
#endif

class COPCServerCtrl
{
public:
	COPCServerCtrl();
	~COPCServerCtrl();

	void InitOPCServer();


	//-----------------------变量绑定-----------------------//

	T_HEARTBEST m_tHeartbest;//心跳数据
	void BindData(T_HEARTBEST &tHeartbest);//绑定心跳数据

	T_STATION_STATE m_atStationState[9];
	void BindData(T_STATION_STATE *tStationState);


	//-----------------------心跳检测-----------------------//
	static UINT ThreadHeartbeatDetectionvoid(void *pParam);
	void HeartbeatDetectionvoid();
	bool m_bConnect;
	bool m_bStartHeartbeatDetectionvoid;
	long m_lLostConnectCount = 0;




private:
	void BindData(VarInfo &ServerVarInfo, const char *cName, bool &Var);
	void BindData(VarInfo &ServerVarInfo, const char *cName, char *Var);
	void BindData(VarInfo &ServerVarInfo, const char *cName, int &Var);
	void BindData(VarInfo &ServerVarInfo, const char *cName, double &Var);

	COPCServer *m_pOPCServer;
};

