#include "stdafx.h"
#include "ESTUNRobotCtrl.h"


ESTUNRobotCtrl::ESTUNRobotCtrl()
{
	//TCP = new TCP_Socket(); //3d接口
	ScriptTcp = new TCP_Socket_ERC();//command脚本
	FastDateTrScriptTcp = new TCP_Socket_ERC();//FDT脚本
	MultiScriptTcp = new TCP_Socket_ERC();  //Multi脚本

	sendBuf=new vector<string>();
	sendPosVar=new vector<T_ROBOT_POSVAR>();

	sendCposVar_M = new vector<ESTUN_CPOS>();
	sendAposVar_M = new vector<ESTUN_APOS>();
	sendSpeedVar_M = new vector<ESTUN_SPEED>();
	sendIntVar_M = new vector<int*>();

	sendCposName_M=new vector<int*>();
	sendAposName_M = new vector<int*>();
	sendSpeedName_M = new vector<int*>();
	sendIntName_M = new vector<int*>();


	rcvBuf = new map<int, vector<string>>();
	script_rcvBuf= new map<int, vector<string>>();
	sendSpeedVar = new vector<double *>;
	sendPosVar_noSpeed=new vector<double *>();
	memset(m_CurPos,0,10*sizeof(double));
	memset(m_CurAxis, 0, 10 * sizeof(double));
	m_FDTSctiptConnection = false;
	m_mutex = CreateMutex(NULL, FALSE, "Mutex");     //创建一把锁
}


ESTUNRobotCtrl::~ESTUNRobotCtrl()
{
	//delete TCP;
	delete ScriptTcp;
	delete FastDateTrScriptTcp;
	delete MultiScriptTcp;

	delete rcvBuf;
	delete sendBuf;
	delete sendPosVar;
	delete script_rcvBuf;
	delete sendSpeedVar;
	delete sendPosVar_noSpeed;
	delete m_mutex;

	delete sendCposVar_M;
	delete sendAposVar_M;
	delete sendSpeedVar_M;
	delete sendIntVar_M;

	delete sendCposName_M;
	delete	sendAposName_M;
	delete	sendSpeedName_M;
	delete	sendIntName_M;
}

int ESTUNRobotCtrl::initSocket(char * ip,UINT Port,bool ifRecord)
{
	int result_socket=initScriptSocket(ip,Port + 100, ifRecord);//command脚本
	int result_socket_2 = initFDTScriptSocket(ip, Port + 200, ifRecord); //FDT脚本
	int result_socket_3 = initMultiScriptSocket(ip, Port + 300, ifRecord); //Multi脚本
	if (result_socket_2 != 0)
		return result_socket_2;
	if (result_socket != 0)
		return result_socket;
	if (result_socket_3 != 0)
		return result_socket_3;

	/*if (TCP->m_connected)return -1;
	TCP->TCP_init(ip,Port);
	TCP->SetCallBackRcvData(Rcv,this);
	TCP->ifRecord = ifRecord;
	return TCP->startSocket();*/
	return result_socket;
}

int ESTUNRobotCtrl::initScriptSocket(char * ip,UINT Port, bool ifRecord)
{
	if (ScriptTcp->m_connected)return -1;
	ScriptTcp->TCP_init(ip,Port);
	ScriptTcp->SetCallBackRcvData(ScriptRcv,this);
	//ScriptTcp->ifRecord = ifRecord;//记录信息
	return ScriptTcp->startSocket();
}
//用于线程时刻数据交互
int ESTUNRobotCtrl::initFDTScriptSocket(char * ip, UINT Port, bool ifRecord)
{
	if (FastDateTrScriptTcp->m_connected)return -1;
	FastDateTrScriptTcp->TCP_init(ip, Port);
	FastDateTrScriptTcp->SetCallBackRcvData(ScriptRcv_FDT, this);
	//FastDateTrScriptTcp->ifRecord = ifRecord;//记录信息
	return FastDateTrScriptTcp->startSocket();
}

int ESTUNRobotCtrl::initMultiScriptSocket(char * ip, UINT Port, bool ifRecord)
{
	if (MultiScriptTcp->m_connected)return -1;
	MultiScriptTcp->TCP_init(ip, Port);
	MultiScriptTcp->SetCallBackRcvData(ScriptRcv, this);
	//MultiScriptTcp->ifRecord = ifRecord;//记录信息
	return MultiScriptTcp->startSocket();
}

int ESTUNRobotCtrl::SetMultiPosVar_M(UINT index, UINT count, const T_ROBOT_POSVAR var[], string Program_name, int coord, int ToolNumber, int UserCoord)
{
	//设置多个位置参数
	sendBuf->clear();
	if (ToolNumber > 20 || ToolNumber < 0) return -1;
	if (UserCoord > 20 || UserCoord < 0) return -1;
	int cmd = 0;
	if (0 == coord)
	{
		cmd = E_SetMultiCpos;
	}
	else if (1 == coord)
	{
		cmd = E_SetMultiApos;
	}
	//int cnt = count > 8 ? 8 : count;
	sendBuf->push_back(std::to_string((long long)index));
	sendBuf->push_back(std::to_string((long long)count));
	sendBuf->push_back(std::to_string((long long)ToolNumber));
	sendBuf->push_back(std::to_string((long long)UserCoord));
	sendBuf->push_back(Program_name);
	sendPosVar->clear();
	for (UINT i = 0; i < count; i++)
	{
		sendPosVar->push_back(var[i]);
	}
	ScriptsendData_M(cmd, sendBuf);
	if (script_wait(cmd, 25000) != 0)return -1;
	map<int, vector<string>>::iterator it = script_rcvBuf->find(cmd);

	WaitForSingleObject(m_mutex, 5000);

	it->second.at(0) = "F";

	if (it->second.at(1) != "1")
	{
		ReleaseMutex(m_mutex);
		return -1;
	}
	ReleaseMutex(m_mutex);
	return 0;
}

int ESTUNRobotCtrl::SetMultiPosVar(UINT index, UINT count, const T_ROBOT_POSVAR var[], int coord,int ToolNumber , int UserCoord)
{
	if (count>300 || (index+ count)>301)
	{
		return -1;
	}
	//设置多个位置参数
		sendBuf->clear();
		if (ToolNumber > 20 || ToolNumber < 0) return -1;
		if (UserCoord > 20 || UserCoord < 0) return -1;
		int cmd = 0;
		if (0 == coord)
		{
			cmd = E_MultiPos;
		}
		else if (1 == coord)
		{
			cmd = E_MultiPos_J;
		}
		//int cnt = count > 8 ? 8 : count;
		sendBuf->push_back(std::to_string((long long)index));
		sendBuf->push_back(std::to_string((long long)count));
		sendBuf->push_back(std::to_string((long long)ToolNumber));
		sendBuf->push_back(std::to_string((long long)UserCoord));
		sendPosVar->clear();
		for (UINT i = 0; i < count; i++)
		{
			sendPosVar->push_back(var[i]);
		}
		 ScriptsendData(cmd, sendBuf);
		if (script_wait(cmd,15000) != 0)return -1;
		map<int, vector<string>>::iterator it = script_rcvBuf->find(cmd);

		WaitForSingleObject(m_mutex, 5000);

		it->second.at(0) = "F";

		if (it->second.at(1) != "1")
		{
			ReleaseMutex(m_mutex);
			return -1;
		}
		ReleaseMutex(m_mutex);
		return 0;
}
int ESTUNRobotCtrl::SetMultiPosVar_8_NoSpeed(UINT index, UINT count, double** Axis_10_Date ,int coord, int ToolNumber, int UserCoord)
{
	//设置多个位置参数
	
	sendBuf->clear();
	if (ToolNumber > 20 || ToolNumber < 0) return -1;
	if (UserCoord > 20 || UserCoord < 0) return -1;
	int cmd = 0;
	if (0 == coord)
	{
		cmd = E_SerMultiPos_L;
	}
	else if (1 == coord)
	{
		cmd = E_SerMultiPos_J;
	}
	//int cnt = count > 8 ? 8 : count;
	sendBuf->push_back(std::to_string((long long)index));
	sendBuf->push_back(std::to_string((long long)count));
	sendBuf->push_back(std::to_string((long long)ToolNumber));
	sendBuf->push_back(std::to_string((long long)UserCoord));
	sendPosVar->clear();
	sendPosVar_noSpeed->clear();
	for (UINT i = 0; i < count; i++)
	{
		sendPosVar_noSpeed->push_back(Axis_10_Date[i]);
	}
	if8axis = true;
	ScriptsendData(cmd, sendBuf);
	if (script_wait(cmd, 15000) != 0)return -1;
	map<int, vector<string>>::iterator it = script_rcvBuf->find(cmd);

	WaitForSingleObject(m_mutex, 5000);

	it->second.at(0) = "F";

	if (it->second.at(1) != "1")
	{
		ReleaseMutex(m_mutex);
		return -1;
	}
	ReleaseMutex(m_mutex);
	return 0;
}
int ESTUNRobotCtrl::SetMultiPosVar_6_NoSpeed(UINT index, UINT count, double** Axis_10_Date, int coord, int ToolNumber, int UserCoord)
{
	//设置多个位置参数
	
	sendBuf->clear();
	if (ToolNumber > 20 || ToolNumber < 0) return -1;
	if (UserCoord > 20 || UserCoord < 0) return -1;
	int cmd = 0;
	if (0 == coord)
	{
		cmd = E_SerMultiPos_L;
	}
	else if (1 == coord)
	{
		cmd = E_SerMultiPos_J;
	}
	//int cnt = count > 8 ? 8 : count;
	sendBuf->push_back(std::to_string((long long)index));
	sendBuf->push_back(std::to_string((long long)count));
	sendBuf->push_back(std::to_string((long long)ToolNumber));
	sendBuf->push_back(std::to_string((long long)UserCoord));
	sendPosVar->clear();
	sendPosVar_noSpeed->clear();
	for (UINT i = 0; i < count; i++)
	{
		sendPosVar_noSpeed->push_back(Axis_10_Date[i]);
	}
	if8axis = false;
	ScriptsendData(cmd, sendBuf);
	if (script_wait(cmd, 15000) != 0)return -1;
	map<int, vector<string>>::iterator it = script_rcvBuf->find(cmd);

	WaitForSingleObject(m_mutex, 5000);

	it->second.at(0) = "F";

	if (it->second.at(1) != "1")
	{
		ReleaseMutex(m_mutex);
		return -1;
	}
	ReleaseMutex(m_mutex);
	return 0;
}
int ESTUNRobotCtrl::SetMultiSpeed(UINT index, UINT count,  double** speedVar)
{
	int cmd = E_SetMultiSpeed;
	sendBuf->clear();
	sendBuf->push_back(std::to_string((long long)index));
	sendBuf->push_back(std::to_string((long long)count));
	sendSpeedVar->clear();
	for (size_t i = 0; i < count; i++)
	{
		sendSpeedVar->push_back(speedVar[i]);
	}
	
	ScriptsendData(cmd, sendBuf);
	if (script_wait(cmd, 15000) != 0)return -1;
	map<int, vector<string>>::iterator it = script_rcvBuf->find(cmd);

	WaitForSingleObject(m_mutex, 5000);

	it->second.at(0) = "F";

	if (it->second.at(1) != "1")
	{
		ReleaseMutex(m_mutex);
		return -1;
	}
	ReleaseMutex(m_mutex);
	return 0;
}

int ESTUNRobotCtrl::SetPosVar_Py(int mP_Number, double pos[8], int config[7], int scoper, int Coord)
{
	char name[30];
	memset(name, '\0', 30);
	if (Coord == 0)//直角
	{
		sprintf_s(name, "mP%d", mP_Number);
	}
	else
	{
		sprintf_s(name, "mJP%d", mP_Number);
	}
	return SetPosVar_Py(name, pos, config,  scoper, Coord);
}

int ESTUNRobotCtrl::SetPosVar_Py(const char * PosName, double pos[8],int config[7],int scoper , int coord)
{
	if (coord==0)//直角
	{
		int cmd = E_setPosVar;
		char information[180];
		memset(information, '\0', 10);
		int mode = config[0];
		sprintf_s(information, "39:%s,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.2f,%.2f,%d,%d,%d,%d,%d,%d,%d,%d;", PosName, pos[0], pos[1], pos[2], pos[3], pos[4], pos[5], pos[6], pos[7], mode, scoper, config[1], config[2], config[3], config[4], config[5], config[6]);
		int rst = ScriptTcp->TCP_Send(ScriptTcp->ReturnSocket, information);
		if (script_wait(cmd, 5000) != 0)return -1;
		map<int, vector<string>>::iterator it = script_rcvBuf->find(cmd);

		WaitForSingleObject(m_mutex, 5000);

		it->second.at(0) = "F";

		if (it->second.at(1) != "1")
		{
			ReleaseMutex(m_mutex);
			return -1;
		}

		ReleaseMutex(m_mutex);
		return 0;
	}
	else //关节
	{
		int cmd = E_setjPosVar;
		char information[88];
		memset(information, '\0', 10);
		sprintf_s(information, "40:%s,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d;", PosName, pos[0], pos[1], pos[2], pos[3], pos[4], pos[5], pos[6], pos[7], scoper);
		int rst = ScriptTcp->TCP_Send(ScriptTcp->ReturnSocket, information);
		if (script_wait(cmd, 1000) != 0)return -1;
		map<int, vector<string>>::iterator it = script_rcvBuf->find(cmd);

		WaitForSingleObject(m_mutex, 5000);

		it->second.at(0) = "F";

		if (it->second.at(1) != "1")
		{
			ReleaseMutex(m_mutex);
			return -1;
		}

		ReleaseMutex(m_mutex);
		return 0;
	}

}
int ESTUNRobotCtrl::SetMultiVar_H(T_MIX_VAR& Mix_struct)
{
	//设置多个位置参数
	sendBuf->clear();
	int cmd = E_MixMultiValue;
	
	//int cnt = count > 8 ? 8 : count;
	//INT数量，cpos数量，apos数量，speed数量放到xml开头
	sendBuf->push_back(std::to_string((long long)Mix_struct.INT_Count));
	sendBuf->push_back(std::to_string((long long)Mix_struct.CPOS_Count));
	sendBuf->push_back(std::to_string((long long)Mix_struct.APOS_Count));
	sendBuf->push_back(std::to_string((long long)Mix_struct.SPEED_Count));
	//int名和值
	sendIntVar_M->clear();
	sendIntName_M->clear();
	sendIntName_M->push_back(Mix_struct.INT_Name);
	sendIntVar_M->push_back(Mix_struct.INT_Value);
	//cpos名和值
	sendCposName_M->clear();
	sendCposVar_M->clear();
	sendCposName_M->push_back(Mix_struct.CPOS_Name);
	for (int i = 0; i < Mix_struct.CPOS_Count; i++)
	{
		sendCposVar_M->push_back(Mix_struct.CPOS_Value[i]);
	}
	//apos名和值
	sendAposName_M->clear();
	sendAposVar_M->clear();
	sendAposName_M->push_back(Mix_struct.APOS_Name);
	for (int i = 0; i < Mix_struct.APOS_Count; i++)
	{
		sendAposVar_M->push_back(Mix_struct.APOS_Value[i]);
	}
	//speed名和值
	sendSpeedName_M->clear();
	sendSpeedVar_M->clear();
	sendSpeedName_M->push_back(Mix_struct.SPEED_Name);
	for (int i = 0; i < Mix_struct.SPEED_Count; i++)
	{
		sendSpeedVar_M->push_back(Mix_struct.SPEED_Value[i]);
	}

	ScriptsendData(cmd, sendBuf);
	if (script_wait(cmd, 15000) != 0)return -1;
	map<int, vector<string>>::iterator it = script_rcvBuf->find(cmd);

	WaitForSingleObject(m_mutex, 5000);

	it->second.at(0) = "F";

	if (it->second.at(1) != "1")
	{
		ReleaseMutex(m_mutex);
		return -1;
	}
	ReleaseMutex(m_mutex);
	return 0;
}
int ESTUNRobotCtrl::GetCPosVar(const char * name, ESTUN_CPOS * Cpos, int scoper)
{
	int cmd = E_GetCposValue;
	char information[20];
	memset(information, '\0', 10);
	sprintf_s(information, "53:%s,%d;", name, scoper);
	int rst = ScriptTcp->TCP_Send(ScriptTcp->ReturnSocket, information);
	if (script_wait(cmd, 5000) != 0)return -1;
	map<int, vector<string>>::iterator it = script_rcvBuf->find(cmd);

	WaitForSingleObject(m_mutex, 5000);

	it->second.at(0) = "F";

	if (it->second.at(1) != "1")
	{
		ReleaseMutex(m_mutex);
		return -1;
	}
	Cpos->mode = atoi(it->second.at(2).c_str());
	Cpos->cf1 = atoi(it->second.at(3).c_str());
	Cpos->cf2 = atoi(it->second.at(4).c_str());
	Cpos->cf3 = atoi(it->second.at(5).c_str());
	Cpos->cf4 = atoi(it->second.at(6).c_str());
	Cpos->cf5 = atoi(it->second.at(7).c_str());
	Cpos->cf6 = atoi(it->second.at(8).c_str());
	Cpos->X = atof(it->second.at(9).c_str());
	Cpos->Y = atof(it->second.at(10).c_str());
	Cpos->Z = atof(it->second.at(11).c_str());
	Cpos->A = atof(it->second.at(12).c_str());
	Cpos->B = atof(it->second.at(13).c_str());
	Cpos->C = atof(it->second.at(14).c_str());
	Cpos->Axis_7 = atof(it->second.at(15).c_str());
	Cpos->Axis_8 = atof(it->second.at(16).c_str());
	ReleaseMutex(m_mutex);
	return 0;
}
int ESTUNRobotCtrl::GetAPosVar(const char * name, ESTUN_APOS * Apos, int scoper)
{
	int cmd = E_GetAposValue;
	char information[90];
	memset(information, '\0', 10);
	sprintf_s(information, "54:%s,%d;", name, scoper);
	int rst = ScriptTcp->TCP_Send(ScriptTcp->ReturnSocket, information);
	if (script_wait(cmd, 5000) != 0)return -1;
	map<int, vector<string>>::iterator it = script_rcvBuf->find(cmd);

	WaitForSingleObject(m_mutex, 5000);

	it->second.at(0) = "F";

	if (it->second.at(1) != "1")
	{
		ReleaseMutex(m_mutex);
		return -1;
	}

	Apos->A1 = atof(it->second.at(2).c_str());
	Apos->A2 = atof(it->second.at(3).c_str());
	Apos->A3 = atof(it->second.at(4).c_str());
	Apos->A4 = atof(it->second.at(5).c_str());
	Apos->A5 = atof(it->second.at(6).c_str());
	Apos->A6 = atof(it->second.at(7).c_str());
	Apos->A7 = atof(it->second.at(8).c_str());
	Apos->A8 = atof(it->second.at(9).c_str());

	ReleaseMutex(m_mutex);
	return 0;
}

//index 速度号，speed：关节，直角，原点，外部轴1，外部轴2
int ESTUNRobotCtrl::SetSpeed(int index, double* speed)
{//
	
	int cmd = E_Speed;
	char information[90];
	memset(information, '\0', 10);
	sprintf_s(information, "22:%d,%.4f,%.4f,%.4f,%.4f,%.4f;", index, speed[0], speed[1], speed[2], speed[3], speed[4]);
	int rst = ScriptTcp->TCP_Send(ScriptTcp->ReturnSocket, information);
	if (script_wait(cmd, 5000) != 0)return -1;
	map<int, vector<string>>::iterator it = script_rcvBuf->find(cmd);

	WaitForSingleObject(m_mutex, 5000);

	it->second.at(0) = "F";

	if (it->second.at(1) != "1")
	{
		ReleaseMutex(m_mutex);
		return -1;
	}

	ReleaseMutex(m_mutex);
	return 0;
}

int ESTUNRobotCtrl::SetSpeed(const char * name, double * speed, int scord)
{
	int cmd = E_SetSpeed_N;
	char information[90];
	memset(information, '\0', 10);
	sprintf_s(information, "51:%s,%.4f,%.4f,%.4f,%.4f,%.4f,%d;", name, speed[0], speed[1], speed[2], speed[3], speed[4], scord);
	int rst = ScriptTcp->TCP_Send(ScriptTcp->ReturnSocket, information);
	if (script_wait(cmd, 5000) != 0)return -1;
	map<int, vector<string>>::iterator it = script_rcvBuf->find(cmd);

	WaitForSingleObject(m_mutex, 5000);

	it->second.at(0) = "F";

	if (it->second.at(1) != "1")
	{
		ReleaseMutex(m_mutex);
		return -1;
	}

	ReleaseMutex(m_mutex);
	return 0;
}

int ESTUNRobotCtrl::GetSpeed(int index, double* speed)
{
	int cmd = E_GetSpeed;
	char information[10];
	memset(information,'\0',10);
	sprintf_s(information, "24:%d;", index);
	int rst = ScriptTcp->TCP_Send(ScriptTcp->ReturnSocket, information);
	if (script_wait(cmd, 5000) != 0)return -1;
	map<int, vector<string>>::iterator it = script_rcvBuf->find(cmd);

	WaitForSingleObject(m_mutex, 5000);

	it->second.at(0) = "F";

	if (it->second.at(1) != "1")
	{
		ReleaseMutex(m_mutex);
		return -1;
	}
	for (int i = 0; i < 5; i++)
	{
		*(speed + i) = stof(it->second.at(2 + i));
	}
	ReleaseMutex(m_mutex);
	return 0;

}

int ESTUNRobotCtrl::GetSpeed(const char* speedName, double* speed)
{
	int cmd = E_GetNameSpeed;
	char information[40];
	memset(information, '\0', 10);
	sprintf_s(information, "25:%s;", speedName);
	int rst = ScriptTcp->TCP_Send(ScriptTcp->ReturnSocket, information);
	if (script_wait(cmd, 5000) != 0)return -1;
	map<int, vector<string>>::iterator it = script_rcvBuf->find(cmd);

	WaitForSingleObject(m_mutex, 5000);

	it->second.at(0) = "F";

	if (it->second.at(1) != "1")
	{
		ReleaseMutex(m_mutex);
		return -1;
	}
	for (int i = 0; i < 5; i++)
	{
		*(speed + i) = stof(it->second.at(2 + i));
	}
	ReleaseMutex(m_mutex);
	return 0;

}

int ESTUNRobotCtrl::GetRobotSerialNum(string information_s)
{
	
	int cmd = E_GetRobotInformation;
	int rst = ScriptTcp->TCP_Send(ScriptTcp->ReturnSocket, "23");
	if (script_wait(cmd, 1000) != 0)return -1;
	map<int, vector<string>>::iterator it = script_rcvBuf->find(cmd);

	WaitForSingleObject(m_mutex, 5000);

	it->second.at(0) = "F";

	if (it->second.at(1) != "1")
	{
		
		information_s = it->second.at(2).capacity();
	
		ReleaseMutex(m_mutex);
		return -1;
	}
	ReleaseMutex(m_mutex);
	return 0;
}

int ESTUNRobotCtrl::CloseScriptAutoAddPos()
{
	int result_err = SetIntVar_Py("Tp_autoAddPos_close", 1, 2);
	return result_err;
}

int ESTUNRobotCtrl::GetCurPos(double* pos,int* Mode)
{
	int cmd = E_GetCurrentPos;
	int rst = ScriptTcp->TCP_Send(ScriptTcp->ReturnSocket, "28:;");
	if (script_wait(cmd, 5000) != 0)return -1;
	map<int, vector<string>>::iterator it = script_rcvBuf->find(cmd);

	WaitForSingleObject(m_mutex, 5000);

	it->second.at(0) = "F";

	if (it->second.at(1) != "1")
	{
		ReleaseMutex(m_mutex);
		return -1;
	}
	for (size_t i = 0; i < 10; i++)
	{
		*(pos + i) = stof(it->second.at(i+2));
	}
	for (size_t i = 0; i < 7; i++)
	{
		*(Mode + i) = stoi(it->second.at(i + 12));
	}
	ReleaseMutex(m_mutex);
	return 0;

}

int ESTUNRobotCtrl::GetCurAxis(double* axis)
{
	int cmd = E_GetCurrentAxis;
	int rst = ScriptTcp->TCP_Send(ScriptTcp->ReturnSocket, "29:;");
	if (script_wait(cmd, 5000) != 0)return -1;
	map<int, vector<string>>::iterator it = script_rcvBuf->find(cmd);

	WaitForSingleObject(m_mutex, 5000);

	it->second.at(0) = "F";

	if (it->second.at(1) != "1")
	{
		ReleaseMutex(m_mutex);
		return -1;
	}
	for (size_t i = 0; i < 10; i++)
	{
		*(axis + i) = stof(it->second.at(i + 2));
	}
	ReleaseMutex(m_mutex);
	return 0;
}
//获得当前世界坐标数据
int ESTUNRobotCtrl::GetCurPos_Fast(double* pos)
{
	if (!Robot_FDTScriptConnect())
	{
		return -1;
	}
	for (size_t i = 0; i < 10; i++)
	{
		*(pos + i) = *(m_CurPos+i);
	}
	return 0;
}
//获得当前轴数据
int ESTUNRobotCtrl::GetCurAxis_Fast(double* axis)
{
	if (!Robot_FDTScriptConnect())
	{
		return -1;
	}
	for (size_t i = 0; i < 10; i++)
	{
		*(axis + i) = *(m_CurAxis + i);
	}
	return 0;
}
int ESTUNRobotCtrl::CheckDone_Fast()
{
	if (!Robot_FDTScriptConnect())
	{
		return -1;
	}
	return m_checkDone;
}
int ESTUNRobotCtrl::CheckDone()
{
	int cmd = E_GetCheckDone;
	char information[10];
	memset(information, '\0', 10);
	sprintf_s(information, "63:;");
	int rst = ScriptTcp->TCP_Send(ScriptTcp->ReturnSocket, information);
	if (script_wait(cmd, 5000) != 0)return -1;
	map<int, vector<string>>::iterator it = script_rcvBuf->find(cmd);

	WaitForSingleObject(m_mutex, 5000);

	it->second.at(0) = "F";

	int result = stoi(it->second.at(1));
	ReleaseMutex(m_mutex);
	return result;
}
int ESTUNRobotCtrl::ErrClear_Py()
{
	int cmd = E_ClearErr;
	char information[10];
	memset(information, '\0', 10);
	sprintf_s(information, "36:;");
	int rst = ScriptTcp->TCP_Send(ScriptTcp->ReturnSocket, information);
	if (script_wait(cmd, 5000) != 0)return -1;
	map<int, vector<string>>::iterator it = script_rcvBuf->find(cmd);

	WaitForSingleObject(m_mutex, 5000);

	it->second.at(0) = "F";

	if (it->second.at(1) != "1")
	{
		ReleaseMutex(m_mutex);
		return -1;
	}
	ReleaseMutex(m_mutex);
	return 0;
}
//转为xml格式发送出去
int ESTUNRobotCtrl::ScriptsendData(int cmd, vector<string> *msg)
{

	 TiXmlDocument xml;
	TiXmlPrinter printer;              //定义两个xml

	if (!ScriptTcp->isConnected())return -1; //判断socket是否断开

	xml.Clear();                                              //清空xml
	TiXmlElement *rootElement = new TiXmlElement("API");     //创建一个API元素，<API> </API>这是根元素

	char* msgValue ="Msg";                 //大于指定枚举，就是mov ，小于这个枚举就是 Msg
	TiXmlElement *msgElement = new TiXmlElement(msgValue);                  //信息元素 ，<Mov></Mov>  或者 <Msg> </Msg>
	msgElement->LinkEndChild(new TiXmlText(to_string((long long)cmd).c_str()));     //<Mov> cmd </Mov> 或者 <Msg>cmd </Msg>

	char tag[6];
	switch (cmd)
	{
	case E_SerMultiPos_L:
	case E_SerMultiPos_J:
	{
		msgElement->SetAttribute("index", msg->at(0).c_str());                      //<Msg index="msg->at(0)" count="msg->at(1)" config="msg->at(2)" >20 < / Msg>
		msgElement->SetAttribute("count", msg->at(1).c_str());
		msgElement->SetAttribute("TOOL", msg->at(2).c_str());
		msgElement->SetAttribute("USERCOOR", msg->at(3).c_str());
		//msgElement->SetAttribute("config", msg->at(2).c_str());
		TiXmlElement *varElement = new TiXmlElement("Var");                 //<Var> </Var>
		for (UINT i = 0; i < sendPosVar_noSpeed->size(); i++)
		{
			sprintf_s(tag, "v%d", i + 1);
			TiXmlElement *posElement = new TiXmlElement(tag);              //<v1></v1>														  
			if (if8axis)
			{
				for (int j = 0; j < 8; j++)
				{
					sprintf_s(tag, "p%d", j + 1);
					posElement->SetDoubleAttribute(tag, sendPosVar_noSpeed->at(i)[j]);//<v1 type="coorType" tool="toolNo" p1="1.323" p2="123.21"...p5="12.53">0</v1>
				}
				if (cmd == E_SerMultiPos_L)
				{
					if (sendPosVar_noSpeed->at(i)[8] < 0 || sendPosVar_noSpeed->at(i)[8] > 7)
					{
						posElement->SetDoubleAttribute("mode", 0); //cpos里面
					}
					else posElement->SetDoubleAttribute("mode", sendPosVar_noSpeed->at(i)[8]); //cpos里面的mode

					char name[10];
					memset(name, '\0', 10);
					for (size_t j = 0; j < 6; j++)
					{
						sprintf_s(name, "cf%zd", j + 1);
						if (sendPosVar_noSpeed->at(i)[j + 9] < -2 || sendPosVar_noSpeed->at(i)[j + 9] > 2)
						{
							posElement->SetDoubleAttribute(name, 0); //cpos里面
						}
						else posElement->SetDoubleAttribute(name, sendPosVar_noSpeed->at(i)[9]); //cpos里面的cfx
						memset(name, '\0', 10);
					}
				}

			}
			else
			{
				for (int j = 0; j < 6; j++)
				{
					sprintf_s(tag, "p%d", j + 1);
					posElement->SetDoubleAttribute(tag, sendPosVar_noSpeed->at(i)[j]);//<v1 type="coorType" tool="toolNo" p1="1.323" p2="123.21"...p5="12.53">0</v1>
				}
				posElement->SetDoubleAttribute("p7",0 );
				posElement->SetDoubleAttribute("p8",0 );
				if (cmd == E_SerMultiPos_L)
				{
					if (sendPosVar_noSpeed->at(i)[6] < 0 || sendPosVar_noSpeed->at(i)[6] > 7)
					{
						posElement->SetDoubleAttribute("mode", 0); //cpos里面
					}
					else posElement->SetDoubleAttribute("mode", sendPosVar_noSpeed->at(i)[6]); //cpos里面的mode
					char name[10];
					memset(name, '\0', 10);
					for (size_t j = 0; j < 6; j++)
					{
						sprintf_s(name, "cf%zd", j + 1);
						if (sendPosVar_noSpeed->at(i)[j + 7] < -2 || sendPosVar_noSpeed->at(i)[j + 7] > 2)
						{
							posElement->SetDoubleAttribute(name, 0); //cpos里面
						}
						else posElement->SetDoubleAttribute(name, sendPosVar_noSpeed->at(i)[9]); //cpos里面的cfx
						memset(name, '\0', 10);
					}
				}
			}

			varElement->LinkEndChild(posElement);        //<Var>  <v1 type="coorType" tool="toolNo" p1="1.323" p2="123.21"...p5="12.53">  0  </v1>   </Var>
		}
		rootElement->LinkEndChild(msgElement);
		rootElement->LinkEndChild(varElement); // <API> <Var>  <v1 type="coorType" tool="toolNo" p1="1.323" p2="123.21"...p5="12.53">  0  </v1>   </Var> < / API>
	}
	break;
	//混合变量 int cpos apos speed
	case E_MixMultiValue:
	{
		msgElement->SetAttribute("INT_C", msg->at(0).c_str());                      //<Msg INT_C="msg->at(0)" CPOS_C="msg->at(1)" APOS_C="msg->at(2)" SPEED_C="" >52 < / Msg>
		msgElement->SetAttribute("CPOS_C", msg->at(1).c_str());
		msgElement->SetAttribute("APOS_C", msg->at(2).c_str());
		msgElement->SetAttribute("SPEED_C", msg->at(3).c_str());
		TiXmlElement *varElement = new TiXmlElement("Var");                 //<Var> </Var>
		char vI_count[20];
		memset(vI_count,'\0',20);
		for (size_t i = 0; i < atoi(msg->at(0).c_str()); i++)//int数量
		{
			sprintf_s(vI_count, "vInt%zd", i + 1);
			TiXmlElement *I_Element = new TiXmlElement(vI_count);              //<vInt1 Iname="" Ivalue="" />
			I_Element->SetDoubleAttribute("Iname",sendIntName_M->at(0)[i]);
			I_Element->SetDoubleAttribute("Ivalue", sendIntVar_M->at(0)[i]);
			varElement->LinkEndChild(I_Element);
		}
		char vCpos_count[20];
		memset(vCpos_count, '\0', 20);
		for (size_t i = 0; i < atoi(msg->at(1).c_str()); i++)//Cpos数量
		{
			sprintf_s(vCpos_count, "vCpos%zd", i + 1);
			TiXmlElement *C_Element = new TiXmlElement(vCpos_count);              //<vCpos1 Cname="" Cv_mode="" Cv_cf1="" .. Cv_X=""/>

			C_Element->SetDoubleAttribute("Cname", sendCposName_M->at(0)[i]);
			C_Element->SetDoubleAttribute("Cv_mode", sendCposVar_M->at(i).mode);
			C_Element->SetDoubleAttribute("Cv_cf1", sendCposVar_M->at(i).cf1);
			C_Element->SetDoubleAttribute("Cv_cf2", sendCposVar_M->at(i).cf2);
			C_Element->SetDoubleAttribute("Cv_cf3", sendCposVar_M->at(i).cf3);
			C_Element->SetDoubleAttribute("Cv_cf4", sendCposVar_M->at(i).cf4);
			C_Element->SetDoubleAttribute("Cv_cf5", sendCposVar_M->at(i).cf5);
			C_Element->SetDoubleAttribute("Cv_cf6", sendCposVar_M->at(i).cf6);

			C_Element->SetDoubleAttribute("Cv_X", sendCposVar_M->at(i).X);
			C_Element->SetDoubleAttribute("Cv_Y", sendCposVar_M->at(i).Y);
			C_Element->SetDoubleAttribute("Cv_Z", sendCposVar_M->at(i).Z);
			C_Element->SetDoubleAttribute("Cv_A", sendCposVar_M->at(i).A);
			C_Element->SetDoubleAttribute("Cv_B", sendCposVar_M->at(i).B);
			C_Element->SetDoubleAttribute("Cv_C", sendCposVar_M->at(i).C);
			C_Element->SetDoubleAttribute("Cv_7", sendCposVar_M->at(i).Axis_7);
			C_Element->SetDoubleAttribute("Cv_8", sendCposVar_M->at(i).Axis_8);

			varElement->LinkEndChild(C_Element);
		}
		char vApos_count[20];
		memset(vApos_count, '\0', 20);
		for (size_t i = 0; i < atoi(msg->at(2).c_str()); i++)//Apos数量
		{
			sprintf_s(vApos_count, "vApos%zd", i + 1);
			TiXmlElement *A_Element = new TiXmlElement(vApos_count);              //<vApos1 Cname="" Cv_mode="" Cv_cf1="" .. Cv_X=""/>

			A_Element->SetDoubleAttribute("Aname", sendAposName_M->at(0)[i]);
			A_Element->SetDoubleAttribute("Av_A1", sendAposVar_M->at(i).A1);
			A_Element->SetDoubleAttribute("Av_A2", sendAposVar_M->at(i).A2);
			A_Element->SetDoubleAttribute("Av_A3", sendAposVar_M->at(i).A3);
			A_Element->SetDoubleAttribute("Av_A4", sendAposVar_M->at(i).A4);
			A_Element->SetDoubleAttribute("Av_A5", sendAposVar_M->at(i).A5);
			A_Element->SetDoubleAttribute("Av_A6", sendAposVar_M->at(i).A6);
			A_Element->SetDoubleAttribute("Av_A7", sendAposVar_M->at(i).A7);
			A_Element->SetDoubleAttribute("Av_A8", sendAposVar_M->at(i).A8);
			varElement->LinkEndChild(A_Element);
		}
		char vSpeed_count[20];
		memset(vSpeed_count, '\0', 20);
		for (size_t i = 0; i < atoi(msg->at(3).c_str()); i++)//SPEED数量
		{
			sprintf_s(vSpeed_count, "vSpeed%zd", i + 1);
			TiXmlElement *A_Element = new TiXmlElement(vSpeed_count);              //<vSpeed1 Cname="" Cv_mode="" Cv_cf1="" .. Cv_X=""/>

			A_Element->SetDoubleAttribute("Sname", sendSpeedName_M->at(0)[i]);
			A_Element->SetDoubleAttribute("Sv_per", sendSpeedVar_M->at(i).per);
			A_Element->SetDoubleAttribute("Sv_tcp", sendSpeedVar_M->at(i).tcp);
			A_Element->SetDoubleAttribute("Sv_ori", sendSpeedVar_M->at(i).ori);
			A_Element->SetDoubleAttribute("Sv_exp7", sendSpeedVar_M->at(i).exp_7);
			A_Element->SetDoubleAttribute("Sv_exp8", sendSpeedVar_M->at(i).exp_8);
			varElement->LinkEndChild(A_Element);
		}
		rootElement->LinkEndChild(msgElement);
		rootElement->LinkEndChild(varElement);
	
	}
	break;

	case 21:
	case 20:
	{
		msgElement->SetAttribute("index", msg->at(0).c_str());                      //<Msg index="msg->at(0)" count="msg->at(1)" config="msg->at(2)" >20 < / Msg>
		msgElement->SetAttribute("count", msg->at(1).c_str());
		msgElement->SetAttribute("TOOL", msg->at(2).c_str());
		msgElement->SetAttribute("USERCOOR", msg->at(3).c_str());
		//msgElement->SetAttribute("config", msg->at(2).c_str());
		TiXmlElement *varElement = new TiXmlElement("Var");                 //<Var> </Var>
		for (UINT i = 0; i < sendPosVar->size(); i++)
		{
			sprintf_s(tag, "v%d", i + 1);
			TiXmlElement *posElement = new TiXmlElement(tag);              //<v1></v1>

																		   //posElement->LinkEndChild(new TiXmlText("0"));                  //<v1>0</v1>
																		   //posElement->SetAttribute("type", sendPosVar->at(i).coorType);  //<v1 type="coorType">0</v1>
																		   //posElement->SetAttribute("tool", sendPosVar->at(i).toolNo);    //<v1 type="coorType" tool="toolNo">0</v1>
			for (int j = 0; j < 6; j++)
			{
				sprintf_s(tag, "p%d", j + 1);
				posElement->SetDoubleAttribute(tag, sendPosVar->at(i).p[j]);//<v1 type="coorType" tool="toolNo" p1="1.323" p2="123.21"...p5="12.53">0</v1>
			}
			posElement->SetDoubleAttribute("p7", sendPosVar->at(i).ExP[0]);
			posElement->SetDoubleAttribute("p8", sendPosVar->at(i).ExP[1]);
			posElement->SetDoubleAttribute("exj_7", sendPosVar->at(i).Ex_7_speed);
			posElement->SetDoubleAttribute("exj_8", sendPosVar->at(i).Ex_8_speed);

			
			
			if (cmd == 21)//关节坐标
			{
				if (sendPosVar->at(i).speed < 0 || sendPosVar->at(i).speed>100) return -1;
				posElement->SetDoubleAttribute("S", sendPosVar->at(i).speed);//速度值
			}
			else if (cmd == 20)//直角坐标
			{
				if (sendPosVar->at(i).speed < 0 || sendPosVar->at(i).speed>10000) return -1;
				posElement->SetDoubleAttribute("S", sendPosVar->at(i).speed);//速度值

				if (sendPosVar->at(i).mode < 0 || sendPosVar->at(i).mode > 7)
				{
					posElement->SetDoubleAttribute("mode", 0);
				}
				else posElement->SetDoubleAttribute("mode", sendPosVar->at(i).mode);

				if (sendPosVar->at(i).cf1 < -2 || sendPosVar->at(i).cf1 > 2)
				{
					posElement->SetDoubleAttribute("cf1", 0); //cpos里面
				}
				else posElement->SetDoubleAttribute("cf1", sendPosVar->at(i).cf1); //cpos里面的cfx

				if (sendPosVar->at(i).cf2 < -2 || sendPosVar->at(i).cf2 > 2)
				{
					posElement->SetDoubleAttribute("cf2", 0); //cpos里面
				}
				else posElement->SetDoubleAttribute("cf2", sendPosVar->at(i).cf2); //cpos里面的cfx

				if (sendPosVar->at(i).cf3 < -2 || sendPosVar->at(i).cf3 > 2)
				{
					posElement->SetDoubleAttribute("cf3", 0); //cpos里面
				}
				else posElement->SetDoubleAttribute("cf3", sendPosVar->at(i).cf3); //cpos里面的cfx

				if (sendPosVar->at(i).cf4 < -2 || sendPosVar->at(i).cf4 > 2)
				{
					posElement->SetDoubleAttribute("cf4", 0); //cpos里面
				}
				else posElement->SetDoubleAttribute("cf4", sendPosVar->at(i).cf4); //cpos里面的cfx

				if (sendPosVar->at(i).cf5 < -2 || sendPosVar->at(i).cf5 > 2)
				{
					posElement->SetDoubleAttribute("cf5", 0); //cpos里面
				}
				else posElement->SetDoubleAttribute("cf5", sendPosVar->at(i).cf5); //cpos里面的cfx

				if (sendPosVar->at(i).cf6 < -2 || sendPosVar->at(i).cf6 > 2)
				{
					posElement->SetDoubleAttribute("cf6", 0); //cpos里面
				}
				else posElement->SetDoubleAttribute("cf6", sendPosVar->at(i).cf6); //cpos里面的cfx

			}
			varElement->LinkEndChild(posElement);        //<Var>  <v1 type="coorType" tool="toolNo" p1="1.323" p2="123.21"...p5="12.53">  0  </v1>   </Var>
		}
		rootElement->LinkEndChild(msgElement);
		rootElement->LinkEndChild(varElement); // <API> <Var>  <v1 type="coorType" tool="toolNo" p1="1.323" p2="123.21"...p5="12.53">  0  </v1>   </Var> < / API>
	}
	break;
	case E_Speed:
	{
		msgElement->SetAttribute("index", msg->at(0).c_str());                      //<Msg index="msg->at(0)" count="msg->at(1)" config="msg->at(2)" >20 < / Msg>
		msgElement->SetAttribute("per", msg->at(1).c_str());
		msgElement->SetAttribute("tcp", msg->at(2).c_str());
		msgElement->SetAttribute("ori", msg->at(3).c_str());
		msgElement->SetAttribute("exj_l", msg->at(4).c_str());
		msgElement->SetAttribute("exj_r", msg->at(5).c_str());
		//rootElement->LinkEndChild(msgElement);
	}
		break;
	case E_GetRobotInformation:
	{
		msgElement->SetAttribute("index", msg->at(0).c_str());
		rootElement->LinkEndChild(msgElement);
	}
		break;
	case E_SetMultiSpeed:
	{
		msgElement->SetAttribute("index", msg->at(0).c_str());                      //<Msg index="msg->at(0)" count="msg->at(1)" config="msg->at(2)" >20 < / Msg>
		msgElement->SetAttribute("count", msg->at(1).c_str());
		//msgElement->SetAttribute("config", msg->at(2).c_str());
		TiXmlElement *varElement = new TiXmlElement("Var");                 //<Var> </Var>
		for (UINT i = 0; i < sendSpeedVar->size(); i++)
		{
			sprintf_s(tag, "v%d", i + 1);
			TiXmlElement *posElement = new TiXmlElement(tag);              //<v1></v1>

			
				if (sendSpeedVar->at(i)[0] < 0 || sendSpeedVar->at(i)[0]>100) return -1;
				posElement->SetDoubleAttribute("per", sendSpeedVar->at(i)[0]);//速度per值
				posElement->SetDoubleAttribute("tcp", sendSpeedVar->at(i)[1]);//速度tcp值
				posElement->SetDoubleAttribute("ori", sendSpeedVar->at(i)[2]);//速度ori值
				posElement->SetDoubleAttribute("exj_l", sendSpeedVar->at(i)[3]);//速度exj_l值
				posElement->SetDoubleAttribute("exj_r", sendSpeedVar->at(i)[4]);//速度exj_r值

			
			varElement->LinkEndChild(posElement);        //<Var>  <v1 type="coorType" tool="toolNo" p1="1.323" p2="123.21"...p5="12.53">  0  </v1>   </Var>
		}
		rootElement->LinkEndChild(msgElement);
		rootElement->LinkEndChild(varElement); 
	}
		break;
	default:
		break;
	}
	if ((cmd != 20) && (cmd != 21) &&(cmd  != E_SetMultiSpeed) && (cmd != E_SerMultiPos_L)&&(cmd != E_SerMultiPos_J)&&(cmd != E_MixMultiValue))
		rootElement->LinkEndChild(msgElement);      //<API> 前面的数据   </API>
	xml.LinkEndChild(rootElement);             //放到xml中
	xml.Accept(&printer);
	int rst = ScriptTcp->TCP_Send(ScriptTcp->ReturnSocket, printer.CStr());
	sendBuf->clear();
	return rst;
}
int ESTUNRobotCtrl::ScriptsendData_M(int cmd, vector<string>* msg)
{
	 TiXmlDocument xml;
	TiXmlPrinter printer;              //定义两个xml

	if (!MultiScriptTcp->isConnected())return -1; //判断socket是否断开

	xml.Clear();                                              //清空xml
	TiXmlElement *rootElement = new TiXmlElement("API");     //创建一个API元素，<API> </API>这是根元素

	char* msgValue = "Msg";                 //大于指定枚举，就是mov ，小于这个枚举就是 Msg
	TiXmlElement *msgElement = new TiXmlElement(msgValue);                  //信息元素 ，<Mov></Mov>  或者 <Msg> </Msg>
	msgElement->LinkEndChild(new TiXmlText(to_string((long long)cmd).c_str()));     //<Mov> cmd </Mov> 或者 <Msg>cmd </Msg>

	char tag[6];
	switch (cmd)
	{
	case E_SetMultiCpos:
	case E_SetMultiApos:
	{
		msgElement->SetAttribute("index", msg->at(0).c_str());                      //<Msg index="msg->at(0)" count="msg->at(1)" config="msg->at(2)" >20 < / Msg>
		msgElement->SetAttribute("count", msg->at(1).c_str());
		msgElement->SetAttribute("TOOL", msg->at(2).c_str());
		msgElement->SetAttribute("USERCOOR", msg->at(3).c_str());
		msgElement->SetAttribute("Program", msg->at(4).c_str());
		//msgElement->SetAttribute("config", msg->at(2).c_str());
		TiXmlElement *varElement = new TiXmlElement("Var");                 //<Var> </Var>
		for (UINT i = 0; i < sendPosVar->size(); i++)
		{
			sprintf_s(tag, "v%d", i + 1);
			TiXmlElement *posElement = new TiXmlElement(tag);              //<v1></v1>

																		   //posElement->LinkEndChild(new TiXmlText("0"));                  //<v1>0</v1>
																		   //posElement->SetAttribute("type", sendPosVar->at(i).coorType);  //<v1 type="coorType">0</v1>
																		   //posElement->SetAttribute("tool", sendPosVar->at(i).toolNo);    //<v1 type="coorType" tool="toolNo">0</v1>
			for (int j = 0; j < 6; j++)
			{
				sprintf_s(tag, "p%d", j + 1);
				posElement->SetDoubleAttribute(tag, sendPosVar->at(i).p[j]);//<v1 type="coorType" tool="toolNo" p1="1.323" p2="123.21"...p5="12.53">0</v1>
			}
			posElement->SetDoubleAttribute("p7", sendPosVar->at(i).ExP[0]);
			posElement->SetDoubleAttribute("p8", sendPosVar->at(i).ExP[1]);
			posElement->SetDoubleAttribute("exj_7", sendPosVar->at(i).Ex_7_speed);
			posElement->SetDoubleAttribute("exj_8", sendPosVar->at(i).Ex_8_speed);



			if (cmd == E_SetMultiApos)//关节坐标
			{
				if (sendPosVar->at(i).speed < 0 || sendPosVar->at(i).speed>100) return -1;
				posElement->SetDoubleAttribute("S", sendPosVar->at(i).speed);//速度值
			}
			else if (cmd == E_SetMultiCpos)//直角坐标
			{
				if (sendPosVar->at(i).speed < 0 || sendPosVar->at(i).speed>10000) return -1;
				posElement->SetDoubleAttribute("S", sendPosVar->at(i).speed);//速度值

				if (sendPosVar->at(i).mode < 0 || sendPosVar->at(i).mode > 7)
				{
					posElement->SetDoubleAttribute("mode", 0);
				}
				else posElement->SetDoubleAttribute("mode", sendPosVar->at(i).mode);

				if (sendPosVar->at(i).cf1 < -2 || sendPosVar->at(i).cf1 > 2)
				{
					posElement->SetDoubleAttribute("cf1", 0); //cpos里面
				}
				else posElement->SetDoubleAttribute("cf1", sendPosVar->at(i).cf1); //cpos里面的cfx

				if (sendPosVar->at(i).cf2 < -2 || sendPosVar->at(i).cf2 > 2)
				{
					posElement->SetDoubleAttribute("cf2", 0); //cpos里面
				}
				else posElement->SetDoubleAttribute("cf2", sendPosVar->at(i).cf2); //cpos里面的cfx

				if (sendPosVar->at(i).cf3 < -2 || sendPosVar->at(i).cf3 > 2)
				{
					posElement->SetDoubleAttribute("cf3", 0); //cpos里面
				}
				else posElement->SetDoubleAttribute("cf3", sendPosVar->at(i).cf3); //cpos里面的cfx

				if (sendPosVar->at(i).cf4 < -2 || sendPosVar->at(i).cf4 > 2)
				{
					posElement->SetDoubleAttribute("cf4", 0); //cpos里面
				}
				else posElement->SetDoubleAttribute("cf4", sendPosVar->at(i).cf4); //cpos里面的cfx

				if (sendPosVar->at(i).cf5 < -2 || sendPosVar->at(i).cf5 > 2)
				{
					posElement->SetDoubleAttribute("cf5", 0); //cpos里面
				}
				else posElement->SetDoubleAttribute("cf5", sendPosVar->at(i).cf5); //cpos里面的cfx

				if (sendPosVar->at(i).cf6 < -2 || sendPosVar->at(i).cf6 > 2)
				{
					posElement->SetDoubleAttribute("cf6", 0); //cpos里面
				}
				else posElement->SetDoubleAttribute("cf6", sendPosVar->at(i).cf6); //cpos里面的cfx

			}
			varElement->LinkEndChild(posElement);        //<Var>  <v1 type="coorType" tool="toolNo" p1="1.323" p2="123.21"...p5="12.53">  0  </v1>   </Var>
		}
		rootElement->LinkEndChild(msgElement);
		rootElement->LinkEndChild(varElement); // <API> <Var>  <v1 type="coorType" tool="toolNo" p1="1.323" p2="123.21"...p5="12.53">  0  </v1>   </Var> < / API>
	}
	break;
	default:
		break;
	}
	if ((cmd != E_SetMultiCpos) && (cmd != E_SetMultiApos) )
		rootElement->LinkEndChild(msgElement);      //<API> 前面的数据   </API>
	xml.LinkEndChild(rootElement);             //放到xml中
	xml.Accept(&printer);
	int rst = MultiScriptTcp->TCP_Send(MultiScriptTcp->ReturnSocket, printer.CStr());
	sendBuf->clear();
	return rst;
}
//sockaddr_in ESTUNRobotCtrl::Robot_ClinerAddr()
//{
//	return TCP->ClinerAddr;
//}

//bool ESTUNRobotCtrl::Robot_Send(CString str)
//{
//	return  TCP->TCP_Send(TCP->ReturnSocket, str);
//}

//bool ESTUNRobotCtrl::Robot_connect()
//{
//	return TCP->m_connected;
//}

bool ESTUNRobotCtrl::Robot_ScriptConnect()
{
	string RobotName;
	int reuslt=GetRobotSerialNum(RobotName);
	if (reuslt < 0) return false;
	else return true;
}

bool ESTUNRobotCtrl::Robot_FDTScriptConnect()
{
	return m_FDTSctiptConnection && FastDateTrScriptTcp->m_connected;
}

bool ESTUNRobotCtrl::Robot_MultiScriptConnect()
{
	return MultiScriptTcp->m_connected;
}

void ESTUNRobotCtrl::IfRecordInformation(bool MultiScriptT, bool ScriptTcp_b, bool FastDateTrScriptTcp_b)
{
	//MultiScriptTcp->ifRecord = MultiScriptT;
	//ScriptTcp->ifRecord = ScriptTcp_b;
	//FastDateTrScriptTcp->ifRecord = FastDateTrScriptTcp_b;
}



int ESTUNRobotCtrl::cfX(double axis)
{
	int cf = 0;

	if (axis > -180 && axis <= 180)
	{
		cf = 0;
	}
	else if (axis > 180 && axis <= 3*180)
	{
		cf = 1;
	}
	else if (axis > -3 * 180 && axis <= -180)
	{
		cf = -1;
	}
	return cf;
}



//int ESTUNRobotCtrl::GetVar(int type, char * VarName, int scope,char* reInfomation)
//{
//	char data_ch[100];
//	int result = 0;
//	int cmd = E_GetVarV3;
//	//[GetVarV3(1, "INT0", 1); id = 3]
//	sprintf_s(data_ch, "[GetVarV3(%d,\"%s\",%d);id=%d]", type, VarName, scope, cmd);
//	Robot_Send(data_ch);
//	if (wait(cmd) == -1)return -1;
//
//	map<int, vector<string>>::iterator it = rcvBuf->find(cmd);
//
//	WaitForSingleObject(m_mutex, 5000);
//
//	it->second.at(0) = "F";
//
//	if (it->second.at(1) != "Ok")
//	{
//		ReleaseMutex(m_mutex);
//		return -1;
//	}
//	*reInfomation = *(it->second.at(0)).c_str();
//	ReleaseMutex(m_mutex);
//	
//	return 0;
//}

//int ESTUNRobotCtrl::SetVar(int type, char * VarName, char* Var,int scoper)
//{
//	char data_ch[100];
//	int result = 0;
//	int cmd = E_GetVarV3;
//	//[GetVarV3(1, "INT0", 1); id = 3]
//	//[SetVarV3(1,"INT0","65",1);id=3
//	sprintf_s(data_ch, "[SetVarV3(%d,\"%s\",\"%s\",%d);id=%d]", type, VarName, Var, scoper, cmd);
//	Robot_Send(data_ch);
//	if (wait(cmd) == -1)return -1;
//
//	map<int, vector<string>>::iterator it = rcvBuf->find(cmd);
//
//	WaitForSingleObject(m_mutex, 5000);
//
//	it->second.at(0) = "F";
//
//	if (it->second.at(1) != "Ok")
//	{
//		ReleaseMutex(m_mutex);
//		return -1;
//	}
//	ReleaseMutex(m_mutex);
//
//	return 0;
//}











void ESTUNRobotCtrl::Rcv(void * pOwner, const char * rcvData, int nDataLen)
{
	ESTUNRobotCtrl* pThis = (ESTUNRobotCtrl*)pOwner;
	vector<string> buf;                     //string 可变长数组
	buf.clear();
	buf.push_back("N");
	string re_cmd_str,re_ifok;
	char data[1000];
	memset(data,'\0',1000);
	strcpy_s(data, rcvData);
	
	if (data[0] == '[' &&  data[nDataLen-1]==']')
	{
		for (int i = 0; i < nDataLen; i++)
		{
			if (data[i] == '=')
			{
				i++;
				for (; data[i]!=';';i++)//获取id后面的数据
				{
					re_cmd_str += data[i];
				}
				i++;
				for (;data[i]!=';' ;i++)//获取是OK还是fail
				{
					if (data[i] == ']')break;
					if(data[i]!=' ')
					re_ifok += data[i];
				}
				buf.push_back(re_ifok);           //把ok/fail放到buf.at(1)里
				if (data[i] == ']')//判断是否结束
				{
					break;
				}
				else //不是结束就获取数据
				{
					i++;
					string str_buf;
					for (; data[i] != ']'; ++i)
					{
						if (data[i] == ' ')i++;
						str_buf = "";
						for (;data[i]!=' ';i++)
						{
							if (data[i] == ']')break;
							str_buf += data[i];
						}
						buf.push_back(str_buf);           //数据放到buf.at(X)里
						if (data[i] == ']')break;
					}
				}
			}
		}
	}
	else
	{
		return;
	}
	int cmd = stoi(re_cmd_str);                                         //把 char * 转string ，把string转int
	                                             
	map<int, vector<string>>::iterator it = pThis->rcvBuf->find(cmd);  //把int的cmd放到键值对的int里面
	WaitForSingleObject(pThis->m_mutex, 5000);                       //加锁，只允许一个线程访问
	if (it == pThis->rcvBuf->end())                                //
		pThis->rcvBuf->insert(make_pair(cmd, buf));              //把buf插入到map中
	else
		it->second = buf;
	ReleaseMutex(pThis->m_mutex);
}
void ESTUNRobotCtrl::ScriptRcv(void * pOwner, const char * rcvData, int nDataLen)
{
	//分析xml信息 ，解析
	ESTUNRobotCtrl *pThis = (ESTUNRobotCtrl *)pOwner;
	 TiXmlDocument xml;
		xml.Clear();
		xml.Parse(rcvData);  //用来解析XML流的
							 //TRACE("\n rcvData: %s \n", rcvData);
		TiXmlElement *root = xml.RootElement();         //获得根元素
		if (root == NULL || string(root->Value()) != "Robot")return;  //

		vector<string> buf;                     //string 可变长数组
		for (const TiXmlNode* child = root->FirstChild(); child; child = child->NextSibling())   //根目录下的第一个子目录是什么
		{
			if (child->Type() != TiXmlNode::TINYXML_ELEMENT)continue;                           //子目录不是元素就返回找下一个子目录
			const TiXmlElement *node = child->ToElement();                                     //这个子目录是元素,进入元素内，找属性
			buf.clear();                           //清空buf
			buf.push_back("N");                   //在尾部添加一个"N"
			for (const TiXmlAttribute* att = node->FirstAttribute(); att != NULL; att = att->Next())  //便利这个node下的属性对应的值，把属性的值 字符串放到可变长数组里
			{                                                                                          //node下的属性有多少个，方便后面使用
				if (att->Value() != "")buf.push_back(att->Value());
			}
			string str;
			str = string(node->Value());
			//  <Robot> <Pos x = "1"  y = "2" z = "3" / > < / Robot>
		 if (string(node->Value()) == "Err")
			{
				if (pThis->Err_bool == false)
					pThis->Err_str = "";
				pThis->Err_bool = true;
				for (UINT i = 1; i < buf.size(); i++)
				{
					pThis->Err_str += buf.at(i);
					pThis->Err_str += " ";
				}
				std::ofstream RecordFile("EstunErrorRecord.txt", std::ios::app); //用记事本的方式记录下来
																				 //time_t t = time(NULL);
				SYSTEMTIME st = { 0 };
				GetLocalTime(&st);
				RecordFile << st.wYear << "/" << st.wMonth << "/" << st.wDay << "  " << st.wHour << ":" << st.wMinute << ":" << st.wSecond << "-" << st.wMilliseconds << ":" << "Fanuc_Err" << ":" << rcvData << "\n";

			}
		 else if (string(node->Value()) == "Pos")
		 {
			 for (size_t i = 0; i < 10; i++)
			 {
				 pThis->m_CurPos[i] = stof(buf.at(i+2));
			 }
			 pThis->m_FDTSctiptConnection = true;
		 }
		 else if (string(node->Value()) == "Axis")
		 {
			 for (size_t i = 0; i < 10; i++)
			 {
				 pThis->m_CurAxis[i] = stof(buf.at(i + 2));
			 }
			 pThis->m_FDTSctiptConnection = true;
		 }
			//<Robot> <Rst commaction="moveL">333</Rst> </Robot>
			else if (string(node->Value()) == "Rst")                           //把接受到的命令记录下来
			{

				int cmd = stoi(string(node->GetText()));                 //把 char * 转string ，把string转int

				map<int, vector<string>>::iterator it = pThis->script_rcvBuf->find(cmd);  //把int的cmd放到键值对的int里面
				WaitForSingleObject(pThis->m_mutex, 5000);                       //加锁，只允许一个线程访问
				if (it == pThis->script_rcvBuf->end())                                //
					pThis->script_rcvBuf->insert(make_pair(cmd, buf));              //把buf插入到map中
				else
					it->second = buf;
				ReleaseMutex(pThis->m_mutex);
			}
			//<Robot> <Var> <Var1 type="4"  tool="5" p1="6" p2="7" p3="8" p4="9" p5="10" p6="11">  </Var> </Robot>
			else if (string(node->Value()) == "Var")  //    放  T_ROBOT_POSVAR数据      可变长 机器人位置 UINT UINT double[6]  结构体 清空
			{

				WaitForSingleObject(pThis->m_mutex, 5000);  //加锁，等待5000ms

				char tag[3];                              //
				pThis->rcvPosVar->clear();                //可变长 机器人位置 UINT UINT double[6]  结构体 清空

				for (const TiXmlNode* child = node->FirstChild(); child; child = child->NextSibling())  //从第一个子开始遍历说有元素，获得里面的属性
				{
					if (child->Type() != TiXmlNode::TINYXML_ELEMENT)continue;

					const TiXmlElement *rst = child->ToElement(); //进入元素内

					string type = rst->Attribute("type");        //获得 属性为type的数据
					if (type == "")continue;	                  //判断type是否有数据

					T_ROBOT_POSVAR var = {};                      //定义一个 可变长 机器人位置 UINT UINT double[6]  结构体
				//	var.coorType = stoi(type);                    // 把type转int 放入到var下的coortype 坐标类型
					//var.toolNo = stoi(rst->Attribute("tool"));    // 获得属性为tool的数据并转为int放入到  --》   把tool放入到 var下的toolNo 工具坐标

					for (int j = 0; j < 6; j++)
					{
						sprintf_s(tag, "p%d", j + 1);            //打印给数组tag赋值p1，p2，p3，p4，p5，p6
						var.p[j] = stoi(rst->Attribute(tag));   //获得点p1-6的坐标值赋值到var下的数组里
					}
					pThis->rcvPosVar->push_back(var);       //把var放到可变长数组里面rcvPosVar
				}
				ReleaseMutex(pThis->m_mutex); //释放锁
			}
		}
}
void ESTUNRobotCtrl::ScriptRcv_FDT(void * pOwner, const char * rcvData, int nDataLen)
{
	//分析xml信息 ，解析
	ESTUNRobotCtrl *pThis = (ESTUNRobotCtrl *)pOwner;
	G_xml* xml = new G_xml();
	string name[] = {"Pos","Axis","IfMove"};
	vector<string> * Attribute=new vector<string>();
	int result=xml->Get_XML_date(rcvData, nDataLen);
	if (result < 0)
	{
		delete Attribute;
		delete xml;
		return;
	}
	for (int i=0;i<3;i++)
	{
		result = xml->Decompose_element(name[i], Attribute);
		if (result < 0)continue;

		//  <Robot> <Pos x = "1"  y = "2" z = "3" / > < / Robot>
		if (name[i] == "Err")
		{
			if (pThis->Err_bool == false)
				pThis->Err_str = "";
			pThis->Err_bool = true;
			for (UINT i = 1; i < Attribute->size(); i++)
			{
				pThis->Err_str += Attribute->at(i);
				pThis->Err_str += " ";
			}
			std::ofstream RecordFile("EstunErrorRecord.txt", std::ios::app); //用记事本的方式记录下来
																			 //time_t t = time(NULL);
			SYSTEMTIME st = { 0 };
			GetLocalTime(&st);
			RecordFile << st.wYear << "/" << st.wMonth << "/" << st.wDay << "  " << st.wHour << ":" << st.wMinute << ":" << st.wSecond << "-" << st.wMilliseconds << ":" << "Fanuc_Err" << ":" << rcvData << "\n";

		}
		else if (name[i] == "Pos")
		{
			for (size_t i = 0; i < 10; i++)
			{
				pThis->m_CurPos[i] = stof(Attribute->at(i + 2));
			}
			pThis->m_FDTSctiptConnection = true;
		}
		else if (name[i] == "Axis")
		{
			for (size_t i = 0; i < 10; i++)
			{
				pThis->m_CurAxis[i] = stof(Attribute->at(i + 2));
			}
			pThis->m_FDTSctiptConnection = true;
		}
		else if (name[i] == "IfMove")
		{
			pThis->m_checkDone= stof(Attribute->at(1));
			pThis->m_FDTSctiptConnection = true;
		}
		//<Robot> <Rst commaction="moveL">333</Rst> </Robot>
		else if (name[i] == "Rst")                           //把接受到的命令记录下来
		{

			int cmd = stoi(Attribute->at(0));                 //把 char * 转string ，把string转int

			map<int, vector<string>>::iterator it = pThis->script_rcvBuf->find(cmd);  //把int的cmd放到键值对的int里面
			WaitForSingleObject(pThis->m_mutex, 5000);                       //加锁，只允许一个线程访问
			if (it == pThis->script_rcvBuf->end())                                //
				pThis->script_rcvBuf->insert(make_pair(cmd, *Attribute));              //把buf插入到map中
			else
				it->second = *Attribute;
			ReleaseMutex(pThis->m_mutex);
		}
	}
		delete Attribute;
		delete xml;
}

//等待数据接受
int ESTUNRobotCtrl::wait(int id, UINT timeout)
{
	time_t start = clock();

	map<int, vector<string>>::iterator it;

	while ((clock() - start) < (timeout ? timeout : 200))
	{
		it = rcvBuf->find(id);
		if (it != rcvBuf->end() && it->second.at(0) == "N")return 0;

		Sleep(4);
	}

	return -1;
}
int ESTUNRobotCtrl::script_wait(int id, UINT timeout)
{
	time_t start = clock();

	map<int, vector<string>>::iterator it;

	while ((clock() - start) < (timeout ? timeout : 200))
	{
		it = script_rcvBuf->find(id);
		if (it != script_rcvBuf->end() && it->second.at(0) == "N")return 0;

		Sleep(4);
	}

	return -1;
}









/*
int ESTUNRobotCtrl::SetUserData(const char * name, double UserDate[6], int scope)
{
	char data_ch[100];
	int result = 0;
	int cmd = E_SetVarV3;
	sprintf_s(data_ch, "[SetVarV3(12,\"%s\",\"%f_%f_%f_%f_%f_%f_\",%d);id=%d]", name, UserDate[0], UserDate[1], UserDate[2], UserDate[3], UserDate[4], UserDate[5], scope, cmd);
	Robot_Send(data_ch);
	if (wait(cmd) == -1)return -1;
	map<int, vector<string>>::iterator it = rcvBuf->find(cmd);
	WaitForSingleObject(m_mutex, 5000);
	it->second.at(0) = 'F';
	if (it->second.at(1) != "Ok") { ReleaseMutex(m_mutex); return -1; }
	ReleaseMutex(m_mutex);
	return 0;
}
int ESTUNRobotCtrl::GetUserData(const char * name, double UserDate[6], int scope)
{
	char data_ch[100];
	int result = 0;
	int cmd = E_GetVarV3;
	sprintf_s(data_ch, "[GetVarV3(9,\"%s\",%d);id=%d]", name, scope, cmd);
	Robot_Send(data_ch);
	if (wait(cmd) == -1)return -1;
	map<int, vector<string>>::iterator it = rcvBuf->find(cmd);
	WaitForSingleObject(m_mutex, 5000);
	it->second.at(0) = 'F';
	if (it->second.at(1) != "Ok") { ReleaseMutex(m_mutex); return -1; }
	for (size_t i = 0; i < 6; i++)
	{
		*(UserDate + i) = stof(it->second.at(3 + i));
	}
	ReleaseMutex(m_mutex);
	return 0;
}
int ESTUNRobotCtrl::SetToolData(const char * name, double ToolDate[6], int scope )
{
	char data_ch[100];
	int result = 0;
	int cmd = E_SetVarV3;
	sprintf_s(data_ch, "[SetVarV3(11,\"%s\",\"%f_%f_%f_%f_%f_%f_\",%d);id=%d]", name, ToolDate[0], ToolDate[1], ToolDate[2], ToolDate[3], ToolDate[4], ToolDate[5], scope, cmd);
	Robot_Send(data_ch);
	if (wait(cmd) == -1)return -1;
	map<int, vector<string>>::iterator it = rcvBuf->find(cmd);
	WaitForSingleObject(m_mutex, 5000);
	it->second.at(0) = 'F';
	if (it->second.at(1) != "Ok") { ReleaseMutex(m_mutex); return -1; }
	ReleaseMutex(m_mutex);
	return 0;
}
int ESTUNRobotCtrl::GetToolData(const char * name, double ToolDate[6],int scope)
{
	char data_ch[100];
	int result = 0;
	int cmd = E_GetVarV3;
	sprintf_s(data_ch, "[GetVarV3(8,\"%s\",%d);id=%d]", name, scope, cmd);
	Robot_Send(data_ch);
	if (wait(cmd) == -1)return -1;
	map<int, vector<string>>::iterator it = rcvBuf->find(cmd);
	WaitForSingleObject(m_mutex, 5000);
	it->second.at(0) = 'F';
	if (it->second.at(1) != "Ok") { ReleaseMutex(m_mutex); return -1; }
	for (size_t i = 0; i < 6; i++)
	{
		*(ToolDate + i) = stof( it->second.at(3 + i));
	}
	ReleaseMutex(m_mutex);
	return 0;
}
*/
void ESTUNRobotCtrl::ModeValue(double Axis[6],const char* RobotMode, int config[7])
{
	double R = 0, L2 = 0, S = 0, L3 = 0;
	if (RobotMode=="20")
	{
		R = 199.1571;
		L2 = 791.0809;
		S = 139.6034;
		L3 = 780.1291;
	}
	else if (RobotMode == "50B")
	{
		R = 228.5205;
		L2 = 851.252;
		S = 169.7065;
		L3 = 1016.5011;
	}
	else if (RobotMode == "8")
	{
		R = 198.6796;
		L2 = 791.2341;
		S = 160.5080;
		L3 = 1009.3125;
	}
	int flag1 = 0, flag3 = 0, flag5 = 0;
	//double flag1_double = R + L3 * cos(Axis[1] + Axis[2]) + L2 * sin(Axis[1]) + S * sin(Axis[1] + Axis[2]);
	double a1 = (Axis[1] + Axis[2])* PI / 180;
	double a2 = (Axis[1])* PI / 180;
	double a3 = (Axis[1] + Axis[2])* PI / 180;
	double flag1_double = R + L3 * cos(a1) + L2 * sin(a2) + S * sin(a3);
	if (flag1_double < 0)
	{
		flag1 = 1;
	}
	else
	{
		flag1 = 0;
	}

	//double flag3_double = Axis[2] + 3.1415926/2 - atan2(S, L3);
	double flag3_double = Axis[2] *PI / 180 + PI / 2 - atan2(S, L3);
	if (flag3_double >= 0 && flag3_double <= PI)
	{
		flag3 = 0;
	}
	else if(flag3_double<0 && flag3_double > -PI)
	{
		flag3 = 1;
	}

	if (Axis[4]>=0 && Axis[4]<=180)
	{
		flag5 = 0;
	}
	else if (Axis[4]<0 && Axis[4]>-180)
	{
		flag5 = 1;
	}

	config[0]= flag5 + 2*flag3 + 4*flag1 ;
	config[1] = cfX(Axis[0]);
	config[2] = cfX(Axis[1]);
	config[3] = cfX(Axis[2]);
	config[4] = cfX(Axis[3]);
	config[5] = cfX(Axis[4]);
	config[6] = cfX(Axis[5]);
}
void ESTUNRobotCtrl::ModeValue(double Axis[6], ESTUN_MODE Rod_length, int config[7])
{
	double R = 0, L2 = 0, S = 0, L3 = 0;

		R = Rod_length.R;
		L2 = Rod_length.L2;
		S = Rod_length.S;
		L3 = Rod_length.L3;
	
	int flag1 = 0, flag3 = 0, flag5 = 0;
	//double flag1_double = R + L3 * cos(Axis[1] + Axis[2]) + L2 * sin(Axis[1]) + S * sin(Axis[1] + Axis[2]);
	double a1 = (Axis[1] + Axis[2])* PI / 180;
	double a2 = (Axis[1])* PI / 180;
	double a3 = (Axis[1] + Axis[2])* PI / 180;
	double flag1_double = R + L3 * cos(a1) + L2 * sin(a2) + S * sin(a3);
	if (flag1_double < 0)
	{
		flag1 = 1;
	}
	else
	{
		flag1 = 0;
	}

	//double flag3_double = Axis[2] + 3.1415926/2 - atan2(S, L3);
	double flag3_double = Axis[2] * PI / 180 + PI / 2 - atan2(S, L3);
	if (flag3_double >= 0 && flag3_double <= PI)
	{
		flag3 = 0;
	}
	else if (flag3_double<0 && flag3_double > -PI)
	{
		flag3 = 1;
	}

	if (Axis[4] >= 0 && Axis[4] <= 180)
	{
		flag5 = 0;
	}
	else if (Axis[4]<0 && Axis[4]>-180)
	{
		flag5 = 1;
	}

	config[0] = flag5 + 2 * flag3 + 4 * flag1;
	config[1] = cfX(Axis[0]);
	config[2] = cfX(Axis[1]);
	config[3] = cfX(Axis[2]);
	config[4] = cfX(Axis[3]);
	config[5] = cfX(Axis[4]);
	config[6] = cfX(Axis[5]);
}
int ESTUNRobotCtrl::SetTpSpeed_Py(UINT speed)
{
	int cmd = E_SetTPSpeed;
	char information[30];
	memset(information, '\0', 30);
	sprintf_s(information, "47:%d;", speed);
	int rst = ScriptTcp->TCP_Send(ScriptTcp->ReturnSocket, information);
	if (script_wait(cmd, 5000) != 0)return -1;
	map<int, vector<string>>::iterator it = script_rcvBuf->find(cmd);
	WaitForSingleObject(m_mutex, 5000);
	it->second.at(0) = "F";
	if (it->second.at(1) != "1")
	{
		ReleaseMutex(m_mutex);
		return -1;
	}
	ReleaseMutex(m_mutex);
	return 0;
}

int ESTUNRobotCtrl::GetToolName(string & ToolName)
{
	int cmd = E_GetCurToolName;
	char information[10];
	memset(information, '\0', 10);
	sprintf_s(information, "57:;");
	int rst = ScriptTcp->TCP_Send(ScriptTcp->ReturnSocket, information);
	if (script_wait(cmd, 5000) != 0)return -1;
	map<int, vector<string>>::iterator it = script_rcvBuf->find(cmd);
	WaitForSingleObject(m_mutex, 5000);
	it->second.at(0) = "F";
	if (it->second.at(1) != "1")
	{
		ReleaseMutex(m_mutex);
		return -1;
	}
	ToolName = it->second.at(2);
	ReleaseMutex(m_mutex);
	return 0;
}

int ESTUNRobotCtrl::GetUserCoolName(string & UserName)
{
	int cmd = E_GetCurUserName;
	char information[10];
	memset(information, '\0', 10);
	sprintf_s(information, "58:;");
	int rst = ScriptTcp->TCP_Send(ScriptTcp->ReturnSocket, information);
	if (script_wait(cmd, 5000) != 0)return -1;
	map<int, vector<string>>::iterator it = script_rcvBuf->find(cmd);
	WaitForSingleObject(m_mutex, 5000);
	it->second.at(0) = "F";
	if (it->second.at(1) != "1")
	{
		ReleaseMutex(m_mutex);
		return -1;
	}
	UserName = it->second.at(2);
	ReleaseMutex(m_mutex);
	return 0;
}

int ESTUNRobotCtrl::GetToolDate(const char * name, double ToolDate[6], int scope)
{
	int cmd = E_GetToolDate;
	char information[30];
	memset(information, '\0', 30);
	sprintf_s(information, "59:%s,%d;", name, scope);
	int rst = ScriptTcp->TCP_Send(ScriptTcp->ReturnSocket, information);
	if (script_wait(cmd, 5000) != 0)return -1;
	map<int, vector<string>>::iterator it = script_rcvBuf->find(cmd);
	WaitForSingleObject(m_mutex, 5000);
	it->second.at(0) = "F";
	if (it->second.at(1) != "1")
	{
		ReleaseMutex(m_mutex);
		return -1;
	}
	for (size_t i = 0; i < 6; i++)
	{
		*(ToolDate + i) =atof(it->second.at(2+i).c_str());
	}

	ReleaseMutex(m_mutex);
	return 0;
}

int ESTUNRobotCtrl::GetUserDate(const char * name, double UserDate[6], int scope)
{
	int cmd = E_GetUserDate;
	char information[30];
	memset(information, '\0', 30);
	sprintf_s(information, "60:%s,%d;", name, scope);
	int rst = ScriptTcp->TCP_Send(ScriptTcp->ReturnSocket, information);
	if (script_wait(cmd, 5000) != 0)return -1;
	map<int, vector<string>>::iterator it = script_rcvBuf->find(cmd);
	WaitForSingleObject(m_mutex, 5000);
	it->second.at(0) = "F";
	if (it->second.at(1) != "1")
	{
		ReleaseMutex(m_mutex);
		return -1;
	}
	for (size_t i = 0; i < 6; i++)
	{
		*(UserDate + i) = atof(it->second.at(2 + i).c_str());
	}
	ReleaseMutex(m_mutex);
	return 0;
}

int ESTUNRobotCtrl::SetUserDate(const char * name, double UserDate[6], int scope)
{
	int cmd = E_SetUserDate;
	char information[100];
	memset(information, '\0', 100);
	sprintf_s(information, "62:%s,%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f;", name, scope, UserDate[0], UserDate[1], UserDate[2], UserDate[3], UserDate[4], UserDate[5]);
	int rst = ScriptTcp->TCP_Send(ScriptTcp->ReturnSocket, information);
	if (script_wait(cmd, 5000) != 0)return -1;
	map<int, vector<string>>::iterator it = script_rcvBuf->find(cmd);
	WaitForSingleObject(m_mutex, 5000);
	it->second.at(0) = "F";
	if (it->second.at(1) != "1")
	{
		ReleaseMutex(m_mutex);
		return -1;
	}
	ReleaseMutex(m_mutex);
	return 0;
}

int ESTUNRobotCtrl::SetToolDate(const char * name, double ToolDate[6], int scope)
{
	int cmd = E_SetToolDate;
	char information[100];
	memset(information, '\0', 100);
	sprintf_s(information, "61:%s,%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f;", name, scope, ToolDate[0], ToolDate[1], ToolDate[2], ToolDate[3], ToolDate[4], ToolDate[5]);
	int rst = ScriptTcp->TCP_Send(ScriptTcp->ReturnSocket, information);
	if (script_wait(cmd, 5000) != 0)return -1;
	map<int, vector<string>>::iterator it = script_rcvBuf->find(cmd);
	WaitForSingleObject(m_mutex, 5000);
	it->second.at(0) = "F";
	if (it->second.at(1) != "1")
	{
		ReleaseMutex(m_mutex);
		return -1;
	}
	ReleaseMutex(m_mutex);
	return 0;
}

int ESTUNRobotCtrl::SetWeaveDate(const char * name, ESTUN_WeaveDate WeaveDate, int scope)
{
	int cmd = E_SetWeaveDate;
	char information[100];
	memset(information, '\0', 100);
	sprintf_s(information, "64:%s,%d,%d,%.3f,%.3f,%.3f,%d,%d,%d,%.3f,%.3f,%d,%d,%d;", 
		name, scope, WeaveDate.Type, WeaveDate.Freq, WeaveDate.Amp_L, WeaveDate.Amp_R, 
		WeaveDate.StopTime_L, WeaveDate.StopTime_C, WeaveDate.StopTime_R, WeaveDate.RotAngle_X, WeaveDate.RotAngle_Z, 
		WeaveDate.DelayType_L, WeaveDate.DelayType_C, WeaveDate.DelayType_R);
	int rst = ScriptTcp->TCP_Send(ScriptTcp->ReturnSocket, information);
	if (script_wait(cmd, 5000) != 0)return -1;
	map<int, vector<string>>::iterator it = script_rcvBuf->find(cmd);
	WaitForSingleObject(m_mutex, 5000);
	it->second.at(0) = "F";
	if (it->second.at(1) != "1")
	{
		ReleaseMutex(m_mutex);
		return -1;
	}
	ReleaseMutex(m_mutex);
	return 0;
}

int ESTUNRobotCtrl::GetServoSts()
{
	int status = 0;
	int status2 = 0;
	int cmd = E_GetServoSts;
	char information[10];
	memset(information, '\0', 10);
	sprintf_s(information, "65:;");
	int rst = ScriptTcp->TCP_Send(ScriptTcp->ReturnSocket, information);
	if (script_wait(cmd, 5000) != 0)return -1;
	map<int, vector<string>>::iterator it = script_rcvBuf->find(cmd);

	WaitForSingleObject(m_mutex, 5000);

	it->second.at(0) = "F";
	status = _ttoi(it->second.at(1).c_str());
	status2 = _ttoi(it->second.at(2).c_str());
	ReleaseMutex(m_mutex);
	return status2;
	
	//int result = stoi(it->second.at(2));
}


//多轴运动停止
int ESTUNRobotCtrl::Stop()
{
	int cmd = E_Stop;
	char information[10];
	memset(information, '\0', 10);
	sprintf_s(information, "30:;");
	int rst = ScriptTcp->TCP_Send(ScriptTcp->ReturnSocket, information);
	if (script_wait(cmd, 5000) != 0)return -1;
	map<int, vector<string>>::iterator it = script_rcvBuf->find(cmd);

	WaitForSingleObject(m_mutex, 5000);

	it->second.at(0) = "F";

	if (it->second.at(1) != "True")
	{
		ReleaseMutex(m_mutex);
		return -1;
	}
	
	ReleaseMutex(m_mutex);
	return 0;
	
}

int ESTUNRobotCtrl::SetSysMode(int mode)
{
	if (mode >= 3)return -1;
	int cmd = E_SetSysMod;
	char information[10];
	memset(information, '\0', 10);
	sprintf_s(information, "31:%d;", mode);
	int rst = ScriptTcp->TCP_Send(ScriptTcp->ReturnSocket, information);
	if (script_wait(cmd, 5000) != 0)return -1;
	map<int, vector<string>>::iterator it = script_rcvBuf->find(cmd);

	WaitForSingleObject(m_mutex, 5000);

	it->second.at(0) = "F";

	if (it->second.at(1) != "True")
	{
		ReleaseMutex(m_mutex);
		return -1;
	}
	ReleaseMutex(m_mutex);
	return 0;
}

int ESTUNRobotCtrl::GetSysMode()
{
	int cmd = E_GetSysMod;
	char information[10];
	memset(information, '\0', 10);
	sprintf_s(information, "32:;");
	int rst = ScriptTcp->TCP_Send(ScriptTcp->ReturnSocket, information);
	if (script_wait(cmd, 5000) != 0)return -1;
	map<int, vector<string>>::iterator it = script_rcvBuf->find(cmd);

	WaitForSingleObject(m_mutex, 5000);

	it->second.at(0) = "F";

	if (it->second.at(1) != "1")
	{
		ReleaseMutex(m_mutex);
		return -1;
	}
	int result = stoi(it->second.at(2));
	ReleaseMutex(m_mutex);
	return result;
}

int ESTUNRobotCtrl::SetPCPointNumber(int number)
{
	int cmd = E_SetPCPoint;
	char information[10];
	memset(information, '\0', 10);
	sprintf_s(information, "33:%d;",number);
	int rst = ScriptTcp->TCP_Send(ScriptTcp->ReturnSocket, information);
	if (script_wait(cmd, 5000) != 0)return -1;
	map<int, vector<string>>::iterator it = script_rcvBuf->find(cmd);
	WaitForSingleObject(m_mutex, 5000);
	it->second.at(0) = "F";
	if (it->second.at(1) != "True")
	{
		ReleaseMutex(m_mutex);
		return -1;
	}
	ReleaseMutex(m_mutex);
	return 0;
}

int ESTUNRobotCtrl::LoadUserProgramer_Py(const char* projName, const char* progName)
{
	int cmd = E_LoadProgramer;
	char information[50];
	memset(information, '\0', 10);
	sprintf_s(information, "37:%s,%s;", projName, progName);
	int rst = ScriptTcp->TCP_Send(ScriptTcp->ReturnSocket, information);
	if (script_wait(cmd, 5000) != 0)return -1;
	map<int, vector<string>>::iterator it = script_rcvBuf->find(cmd);
	WaitForSingleObject(m_mutex, 5000);
	it->second.at(0) = "F";
	if (it->second.at(1) != "1")
	{
		ReleaseMutex(m_mutex);
		return -1;
	}
	ReleaseMutex(m_mutex);
	return 0;
}

int ESTUNRobotCtrl::Prog_startRun_Py()
{
	int cmd = E_startRun;
	char information[10];
	memset(information, '\0', 10);
	sprintf_s(information, "38:;");
	int rst = ScriptTcp->TCP_Send(ScriptTcp->ReturnSocket, information);
	if (script_wait(cmd, 5000) != 0)return -1;
	map<int, vector<string>>::iterator it = script_rcvBuf->find(cmd);
	WaitForSingleObject(m_mutex, 5000);
	it->second.at(0) = "F";
	if (it->second.at(1) != "1")
	{
		ReleaseMutex(m_mutex);
		return -1;
	}
	ReleaseMutex(m_mutex);
	return 0;
}

int ESTUNRobotCtrl::SetIntVar_Py(const char * name, int value, int score)
{
	int cmd = E_SetIntVar;
	char information[30];
	memset(information, '\0', 10);
	sprintf_s(information, "41:%s,%d,%d;", name, value, score);
	int rst = ScriptTcp->TCP_Send(ScriptTcp->ReturnSocket, information);
	if (script_wait(cmd, 5000) != 0)return -1;
	map<int, vector<string>>::iterator it = script_rcvBuf->find(cmd);
	WaitForSingleObject(m_mutex, 5000);
	it->second.at(0) = "F";
	if (it->second.at(1) != "1")
	{
		ReleaseMutex(m_mutex);
		return -1;
	}
	ReleaseMutex(m_mutex);
	return 0;
}

int ESTUNRobotCtrl::GetIntVar_Py(const char * name, int& value, int score)
{
	int cmd = E_GetIntVar;
	char information[30];
	memset(information, '\0', 10);
	sprintf_s(information, "42:%s,%d;", name, score);
	int rst = ScriptTcp->TCP_Send(ScriptTcp->ReturnSocket, information);
	if (script_wait(cmd, 5000) != 0)return -1;
	map<int, vector<string>>::iterator it = script_rcvBuf->find(cmd);
	WaitForSingleObject(m_mutex, 5000);
	it->second.at(0) = "F";
	if (it->second.at(1) != "1")
	{
		ReleaseMutex(m_mutex);
		return -1;
	}
	value =atoi(it->second.at(2).c_str());
	ReleaseMutex(m_mutex);
	return 0;
}

int ESTUNRobotCtrl::SetRealVar_Py(const char * name, double value, int score)
{
	int cmd = E_SetRealVar;
	char information[30];
	memset(information, '\0', 10);
	sprintf_s(information, "43:%s,%f,%d;", name, value, score);
	int rst = ScriptTcp->TCP_Send(ScriptTcp->ReturnSocket, information);
	if (script_wait(cmd, 5000) != 0)return -1;
	map<int, vector<string>>::iterator it = script_rcvBuf->find(cmd);
	WaitForSingleObject(m_mutex, 5000);
	it->second.at(0) = "F";
	if (it->second.at(1) != "1")
	{
		ReleaseMutex(m_mutex);
		return -1;
	}
	ReleaseMutex(m_mutex);
	return 0;
}

int ESTUNRobotCtrl::GetRealVar_Py(const char * name, double & value, int score)
{
	int cmd = E_GetRealVar;
	char information[30];
	memset(information, '\0', 10);
	sprintf_s(information, "44:%s,%d;", name, score);
	int rst = ScriptTcp->TCP_Send(ScriptTcp->ReturnSocket, information);
	if (script_wait(cmd, 5000) != 0)return -1;
	map<int, vector<string>>::iterator it = script_rcvBuf->find(cmd);
	WaitForSingleObject(m_mutex, 5000);
	it->second.at(0) = "F";
	if (it->second.at(1) != "1")
	{
		ReleaseMutex(m_mutex);
		return -1;
	}
	value = atof(it->second.at(2).c_str());
	ReleaseMutex(m_mutex);
	return 0;
}

int ESTUNRobotCtrl::Set_O_Var_Py(int name, int value)
{
	int cmd = E_SetOutVar;
	char information[30];
	memset(information, '\0', 10);
	sprintf_s(information, "48:%d,%d;", name, value);
	int rst = ScriptTcp->TCP_Send(ScriptTcp->ReturnSocket, information);
	if (script_wait(cmd, 5000) != 0)return -1;
	map<int, vector<string>>::iterator it = script_rcvBuf->find(cmd);
	WaitForSingleObject(m_mutex, 5000);
	it->second.at(0) = "F";
	if (it->second.at(1) != "1")
	{
		ReleaseMutex(m_mutex);
		return -1;
	}
	ReleaseMutex(m_mutex);
	return 0;
}

int ESTUNRobotCtrl::Get_O_Var_Py(int name, int & value)
{
	int cmd = E_GetOutVar;
	char information[30];
	memset(information, '\0', 10);
	sprintf_s(information, "49:%d;", name);
	int rst = ScriptTcp->TCP_Send(ScriptTcp->ReturnSocket, information);
	if (script_wait(cmd, 5000) != 0)return -1;
	map<int, vector<string>>::iterator it = script_rcvBuf->find(cmd);
	WaitForSingleObject(m_mutex, 5000);
	it->second.at(0) = "F";
	if (it->second.at(1) != "1")
	{
		ReleaseMutex(m_mutex);
		return -1;
	}
	value = atoi(it->second.at(2).c_str());
	ReleaseMutex(m_mutex);
	return 0;
}

int ESTUNRobotCtrl::Get_I_Var_Py(int name, int & value)
{
	int cmd = E_GetInVar;
	char information[30];
	memset(information, '\0', 10);
	sprintf_s(information, "50:%d;", name);
	int rst = ScriptTcp->TCP_Send(ScriptTcp->ReturnSocket, information);
	if (script_wait(cmd, 5000) != 0)return -1;
	map<int, vector<string>>::iterator it = script_rcvBuf->find(cmd);
	WaitForSingleObject(m_mutex, 5000);
	it->second.at(0) = "F";
	if (it->second.at(1) != "1")
	{
		ReleaseMutex(m_mutex);
		return -1;
	}
	value = atoi(it->second.at(2).c_str());
	ReleaseMutex(m_mutex);
	return 0;
}

int ESTUNRobotCtrl::SetToolNum(const char * ToolName, int score)
{
	int cmd = E_SetToolNum;
	char information[30];
	memset(information, '\0', 10);
	sprintf_s(information, "45:%s,%d;", ToolName, score);
	int rst = ScriptTcp->TCP_Send(ScriptTcp->ReturnSocket, information);
	if (script_wait(cmd, 5000) != 0)return -1;
	map<int, vector<string>>::iterator it = script_rcvBuf->find(cmd);
	WaitForSingleObject(m_mutex, 5000);
	it->second.at(0) = "F";
	if (it->second.at(1) != "1")
	{
		ReleaseMutex(m_mutex);
		return -1;
	}
	ReleaseMutex(m_mutex);
	return 0;
}

int ESTUNRobotCtrl::SetUserNum(const char * UserName, int score)
{
	int cmd = E_SerUserNum;
	char information[30];
	memset(information, '\0', 10);
	sprintf_s(information, "46:%s,%d;", UserName, score);
	int rst = ScriptTcp->TCP_Send(ScriptTcp->ReturnSocket, information);
	if (script_wait(cmd, 5000) != 0)return -1;
	map<int, vector<string>>::iterator it = script_rcvBuf->find(cmd);
	WaitForSingleObject(m_mutex, 5000);
	it->second.at(0) = "F";
	if (it->second.at(1) != "1")
	{
		ReleaseMutex(m_mutex);
		return -1;
	}
	ReleaseMutex(m_mutex);
	return 0;
}



int ESTUNRobotCtrl::MoveByJob(int Axis, double Distence, int config[7], double speed, int ifAbsoluteM, int ifJoint)
{
	double date[9] = { 0 };
	//T_ROBOT_POS  pos_a;
	double pos_a1[10] = {0};
	//T_ROBOT_AXIS  axis;
	double axis_a1[10] = { 0 };
	double axis_a2[10] = { 0 };
	double speed_a[5] = { 5,5,5,255.95,5 };
	//int config[7] = { 0 };
	if (Axis == 0)return -1;
	if (ifJoint == 0)//直角
	{
		if (speed < 0)return -1;
		speed_a[1] = speed;
		speed_a[3] = speed;
	}
	else  //关节
	{
		if (speed > 100 || speed < 0)return -1;
		speed_a[0] = speed;
		speed_a[3] = speed;

	}
	if (ifAbsoluteM == 0)//相对
	{
		if (ifJoint == 0)//直角坐标
		{
			int result = GetCurPos_Fast(pos_a1);
			if (result < 0) return -1;
			for (int i = 0; i < 8; i++)
			{
				*(date + i) = *(pos_a1 + i);
			}
		}
		else           //关节坐标
		{
			int result = GetCurAxis_Fast(axis_a1);
			if (result < 0) return -1;
			for (int i = 0; i < 6; i++)
			{
				*(date + i) = *(axis_a1 + i);
			}
		}
		date[Axis - 1] = date[Axis - 1]+ Distence;
	}
	else//绝对运动
	{
		if (ifJoint == 0)//直角坐标
		{
			int result = GetCurPos_Fast(pos_a1);
			if (result < 0) return -1;
			for (int i = 0; i < 8; i++)
			{
				*(date + i) = *(pos_a1 + i);
			}
		}
		else           //关节坐标
		{
			int result = GetCurAxis_Fast(axis_a1);
			if (result < 0) return -1;
			for (int i = 0; i < 6; i++)
			{
				*(date + i) = *(axis_a1 + i);
			}
		}
		date[Axis - 1] = Distence;
	}
	int result_p;
	if (ifJoint == 0)
	{
		result_p = LoadUserProgramer_Py("Xi_Robot", "MOVL");//加载程序EstunRemoteApiLibWeld
	}
	else
	{
		result_p = LoadUserProgramer_Py("Xi_Robot", "MOVJ");
	}
	if (result_p < 0)
	{
		return -1;
	}

	int result = SetPosVar_Py(1, date,config, 2,ifJoint);  //设置P1
	if (result < 0)
	{
		//发送失败再发一次
		Sleep(600);
		int result1 = SetPosVar_Py(1, date, config ,2, ifJoint);  //设置P1
		if (result1 < 0)return -1;
	}

	int result_s = SetSpeed(1, speed_a);   //设置speed
	if (result_s < 0)
	{
		//发送失败再发一次
		Sleep(600);
		int result_s1 = SetSpeed(1, speed_a);   //设置speed
		if (result_s1 < 0) return -1;
	}
	
	int result_start = Prog_startRun_Py();   //启动
	if (result_start < 0)return -1;
	return 0;
}

int ESTUNRobotCtrl::MoveByJob(double Distence[8],int config[7], double speed, int ifAbsoluteM, int ifJoint)
{
	double date[9] = { 0 };
//	T_ROBOT_POS  pos_a;
	double pos_a1[10] = { 0 };
//	T_ROBOT_AXIS  axis;
	double axis_a1[10] = {0};
	double speed_a[5] = { 1.0000,1.0000,1.0000,255.95,1.0000 };//使用外部轴时，倒数第二个变量给255.95 即最大外部轴速度
	if (ifJoint == 0)//直角
	{
		if (speed < 0)return -1;
		speed_a[1] = speed;
		speed_a[3] = speed;
	}
	else  //关节
	{
		if (speed > 100 || speed < 0)return -1;
		speed_a[0] = speed;
	}
	if (ifAbsoluteM == 0)//相对
	{
		if (ifJoint == 0)//直角坐标
		{
			int result = GetCurPos_Fast(pos_a1);
			if (result < 0) return -1;
			for (int i = 0; i < 8; i++)
			{
				*(date + i) = *(Distence+i) + *(pos_a1 + i);
			}
		}
		else           //关节坐标
		{
			int result = GetCurAxis_Fast(axis_a1);
			if (result < 0) return -1;
			for (int i = 0; i < 8; i++)
			{
				*(date + i) = *(Distence + i) + *(axis_a1 + i);
			}
		}
	}
	else//绝对运动
	{
		for (int i = 0; i < 8; i++)
		{
			*(date + i) = *(Distence + i);
		}
	}
	//date[8] = Distence[8];
	int result_p=0;
	if (ifJoint == 0)
	{
		result_p = LoadUserProgramer_Py("Xi_Robot", "MOVL");//加载程序
	}
	else
	{
		result_p = LoadUserProgramer_Py("Xi_Robot", "MOVJ");//加载程序
	}
	if (result_p < 0)return -3;
	//Sleep(100);
	int result = SetPosVar_Py(1, date, config,2,ifJoint);  //设置P1
	if (result < 0) 
	{
		int result1 = SetPosVar_Py(1, date, config ,2, ifJoint);  //设置P1
		if (result1 < 0)return -4;
	}
	int result_s = SetSpeed(1, speed_a);   //设置speed
	if (result_s < 0) 
	{
		Sleep(500);
		result_s = SetSpeed(1, speed_a);   //设置
	}
	if (result_s < 0)return -5;

	int result_start = Prog_startRun_Py();   //启动
	if (result_start < 0)return -7;
	return 0;
}

int ESTUNRobotCtrl::ConfigFTP(CString ftp_ip, CString user_name, CString pass_word)
{
	this->ftp_ip = ftp_ip;
	this->user_name = user_name;
	this->pass_word = pass_word;
	return 0;
}


int ESTUNRobotCtrl::ConnectFtp()
{
	pInternetSession = new CInternetSession(AfxGetAppName(), 1, PRE_CONFIG_INTERNET_ACCESS);
	//利用Internet会话对象pInternetSession打开一个FTP连接
	try
	{
		pFtpConnection = pInternetSession->GetFtpConnection(ftp_ip, user_name, pass_word, 21, TRUE);
	}
	catch (CInternetException *pEx)
	{
		TCHAR szError[1024];
		if (pEx->GetErrorMessage(szError, 1024))
		{
			FTPConnetionErr=szError;
		}
		else
		{
			FTPConnetionErr="There was an exception in FTP";
		}
		pEx->Delete();
		pFtpConnection = NULL;
		return -1;
	}

	return 0;
}

int ESTUNRobotCtrl::DisConnectFtp()
{
	if (NULL != pFtpConnection)
	{
		pFtpConnection->Close();
		pFtpConnection = NULL;
		pInternetSession->Close();
		pInternetSession = NULL;
	}
	return 0;
}

int ESTUNRobotCtrl::UploadFile(CString RemoteFilePath, CString LocalFilePath)
{
	if (NULL != pFtpConnection)
	{
			CString strdir;
			pFtpConnection->GetCurrentDirectory(strdir);
			//上传文件
			BOOL bput = pFtpConnection->PutFile(LocalFilePath,RemoteFilePath);
			if (bput)
			{
				pInternetSession->Close();//关闭会话
				this->ConnectFtp();//重新连接保持持续会话
				pFtpConnection->SetCurrentDirectory(strdir);
				return 0;
			}
	}
	return -1;
}

int ESTUNRobotCtrl::DownloadFile(CString RemoteFilePath, CString LocalFilePath)
{
	Sleep(500);
	int result = 0;
	// TODO:  在此添加控件通知处理程序代码
	if (NULL != pFtpConnection)
	{
			CString strdir;
			pFtpConnection->GetCurrentDirectory(strdir);
			result = pFtpConnection->GetFile(RemoteFilePath, LocalFilePath,false, FILE_ATTRIBUTE_NORMAL, FTP_TRANSFER_TYPE_BINARY, 1);//下载文件到选定的本地位置
			pInternetSession->Close();//关闭废弃的对话
			this->ConnectFtp();//保持持续会话
			pFtpConnection->SetCurrentDirectory(strdir);
	}
	return result;
}

int ESTUNRobotCtrl::ErdFile2IniFile(CString ErdFilePath, CString IniFilePath)
{
	CStdioFile ErdFile,IniFile;
	CString strline;
	CString allString;
	BOOL flag = ErdFile.Open(ErdFilePath, CFile::modeCreate | CFile::modeNoTruncate | CFile::modeReadWrite);
	if (flag == FALSE)
	{
		auto ttt = GetLastError();
		return -1;
	}
	flag = IniFile.Open(IniFilePath, CFile::modeCreate | CFile::modeNoTruncate | CFile::modeReadWrite);
	if (flag == FALSE)
	{
		return -1;
	}
	ErdFile.SeekToBegin();
	strline = "[Paramter]";
	strline += "\n";
	IniFile.WriteString(strline);
	while (ErdFile.ReadString(allString))
	{
		if ((allString.GetLength() > 0) && ('0' == allString[0]))
		{
			break;
		}
		allString += "\n";
		IniFile.WriteString(allString);
	}
	ErdFile.Close();
	IniFile.Close();

	DeleteFile(ErdFilePath);//MFC框架中可直接调用此函数

	
	return 0;
}

int ESTUNRobotCtrl::IniFile2ErdFile(CString IniFilePath, CString ErdFilePath)
{
	CStdioFile IniFile, ErdFile;
	CString allString2;
	BOOL flag = IniFile.Open(IniFilePath, CFile::modeCreate | CFile::modeNoTruncate | CFile::modeReadWrite);
	if (flag == FALSE)
	{
		return -1;
	}
	flag = ErdFile.Open(ErdFilePath, CFile::modeCreate | CFile::modeNoTruncate | CFile::modeReadWrite);
	if (flag == FALSE)
	{
		return -1;
	}
	IniFile.SeekToBegin();
	while (IniFile.ReadString(allString2))
	{
		if ((allString2.GetLength() > 0) && ('0' == allString2[0]))
		{
			break;
		}
		if (allString2 != "[Paramter]")
		{
			allString2 += "\n";
			ErdFile.WriteString(allString2);
		}
	}
	IniFile.Close();
	ErdFile.Close();

	DeleteFile(IniFilePath);//MFC框架中可直接调用此函数

	return 0;
}

int ESTUNRobotCtrl::ReadIniFileParameter(CString IniFilePath, CString ParameterName, LPSTR ParameterValue, int ParameterLong)
{
	GetPrivateProfileString("Paramter", ParameterName, "无此对象", ParameterValue, ParameterLong, IniFilePath);
	if (ParameterValue == "无此对象") return -1;
	return 0;
}

int ESTUNRobotCtrl::WriteIniFileParameter(CString IniFilePath, CString ParameterName, CString ParameterValue)
{
	WritePrivateProfileString("Paramter", ParameterName, ParameterValue, IniFilePath);
	return 0;
}

int ESTUNRobotCtrl::CPos2ErdCPos(ESTUN_CPOS cpos, CString& ErdCPos)
{
	char information[1024];
	memset(information, '\0', 1024);
	sprintf_s(information, "{_type=\"CPOS\",confdata={_type=\"POSCFG\",mode=%d,cf1=%d,cf2=%d,cf3=%d,cf4=%d,cf5=%d,cf6=%d},x=%f,y=%f,z=%f,a=%f,b=%f,c=%f,a7=%f,a8=%f,a9=0.0000000,a10=0.0000000,a11=0.0000000,a12=0.0000000,a13=0.0000000,a14=0.0000000,a15=0.0000000,a16=0.0000000}",
		cpos.mode, cpos.cf1,cpos.cf2,cpos.cf3,cpos.cf4,cpos.cf5,cpos.cf6,cpos.X,cpos.Y,cpos.Z,cpos.A,cpos.B,cpos.C,cpos.Axis_7,cpos.Axis_8);
	ErdCPos = information;
	return 0;
}

int ESTUNRobotCtrl::APos2ErdAPos(ESTUN_APOS APos, CString& ErdAPos)
{
	char information[1024];
	memset(information, '\0', 1024);
	sprintf_s(information, "{_type=\"APOS\",a1=%f,a2=%f,a3=%f,a4=%f,a5=%f,a6=%f,a7=%f,a8=%f,a9=0.0000000,a10=0.0000000,a11=0.0000000,a12=0.0000000,a13=0.0000000,a14=0.0000000,a15=0.0000000,a16=0.0000000}", 
		APos.A1, APos.A2, APos.A3, APos.A4, APos.A5, APos.A6, APos.A7, APos.A8);
	ErdAPos = information;
	return 0;
}

int ESTUNRobotCtrl::Speed2ErdSpeed(ESTUN_SPEED speed, CString & ErdSpeed)
{
	// {_type="SPEED",per=100.000000,tcp=4000.000000,ori=360.000000,exj_l=360.000000,exj_r=180.000000}

	char information[1024];
	memset(information, '\0', 1024);
	sprintf_s(information, "{_type=\"SPEED\",per=%f,tcp=%f,ori=%f,exj_l=%f,exj_r=%f}",
		speed.per,speed.tcp,speed.ori,speed.exp_7,speed.exp_8);
	ErdSpeed = information;

	return 0;
}

int ESTUNRobotCtrl::WeaveDate2ErdWeave(ESTUN_WeaveDate weave, CString & ErdWeave)
{
	char information[1024];
	memset(information, '\0', 1024);
	sprintf_s(information, "{_type=\"WEAVE\",Type=%d,Freq=%f,Amp_L=%f,Amp_R=%f,StopTime_L=%d,StopTime_C=%d,StopTime_R=%d,RotAngle_X=%f,RotAngle_Z=%f}",
		weave.Type, weave.Freq, weave.Amp_L , weave.Amp_R, weave.StopTime_L,weave.StopTime_C,weave.StopTime_R,weave.RotAngle_X,weave.RotAngle_Z);
	ErdWeave = information;

	return 0;
}

int ESTUNRobotCtrl::ErdCPos2CPos(CString& ErdCPos, ESTUN_CPOS & cpos)
{
	map<int, vector<string>> ErdSplit;
	std::string str = ErdCPos.GetBuffer();
	if (ErdCPos == "") return-1;

	string str2;
	vector<string> strVector;

	int last_index = 0;
	int index = 0;
	while (str.find(",") + 1)//当找不到空格时返回-1，所以这里我用它返回值加1来循环
	{
		index = str.find(",");
		str2 = str.substr(0, index);
		str = str.substr(index + 1);
		strVector.push_back(str2);
	}
	//string a2 = strVector.at(0).c_str();
	int a = strVector.at(0).find("CPOS");
	if (a == -1) return -1;
	char CposName[10];
	cpos.mode = stoi(strVector.at(2).substr(strVector.at(2).find("mode=") + 5));
	for (size_t i = 0; i < 6; i++)
	{
		memset(CposName, '\0', 10);
		sprintf_s(CposName, "cf%zd=", i+1);
		if (strVector.at(i+3).find(CposName) == -1) return-1;
		*(&cpos.cf1 + i) = stoi(strVector.at(i+3).substr(strVector.at(i+3).find(CposName) + 4,1));
	}
	
		cpos.X = stod(strVector.at(9).substr(strVector.at(9).find("x=") + 2));
		cpos.Y = stod(strVector.at(10).substr(strVector.at(10).find("y=") + 2));
		cpos.Z = stod(strVector.at(11).substr(strVector.at(11).find("z=") + 2));
		cpos.A = stod(strVector.at(12).substr(strVector.at(12).find("a=") + 2));
		cpos.B = stod(strVector.at(13).substr(strVector.at(13).find("b=") + 2));
		cpos.C = stod(strVector.at(14).substr(strVector.at(14).find("c=") + 2));
		cpos.Axis_7 = stod(strVector.at(15).substr(strVector.at(15).find("a7=") + 3));
		cpos.Axis_8 = stod(strVector.at(16).substr(strVector.at(16).find("a8=") + 3));


	return 0;
}

int ESTUNRobotCtrl::ErdAPos2APos(CString& ErdAPos, ESTUN_APOS & APos)
{
	map<int, vector<string>> ErdSplit;
	std::string str = ErdAPos.GetBuffer();
	if (ErdAPos == "") return-1;

	string str2;
	vector<string> strVector;

	int last_index = 0;
	int index = 0;
	while (str.find(",") + 1)//当找不到空格时返回-1，所以这里我用它返回值加1来循环
	{
		index = str.find(",");
		str2 = str.substr(0, index);
		str = str.substr(index + 1);
		strVector.push_back(str2);
	}
	//string a2 = strVector.at(0).c_str();
	int a = strVector.at(0).find("APOS");
	if (a == -1) return -1;
	char AposName[10];
	for (size_t i = 1; i < 9; i++) 
	{
		memset(AposName,'\0',10);
		sprintf_s(AposName,"a%zd=",i);
		if (strVector.at(i).find(AposName)==-1) return-1;
		*(&APos.A1 +i-1) = stod( strVector.at(i).substr(strVector.at(i).find(AposName)+3));
	}
	return 0;
}

int ESTUNRobotCtrl::ErdSpeed2Speed(CString & ErdSpeed, ESTUN_SPEED & speed)
{
	map<int, vector<string>> ErdSplit;
	std::string str = ErdSpeed.GetBuffer();
	if (ErdSpeed == "") return-1;

	string str2;
	vector<string> strVector;

	int last_index = 0;
	int index = 0;
	while (str.find(",") + 1)//当找不到空格时返回-1，所以这里我用它返回值加1来循环
	{
		index = str.find(",");
		str2 = str.substr(0, index);
		str = str.substr(index + 1);
		strVector.push_back(str2);
	}
	strVector.push_back(str);
	int a = strVector.at(0).find("SPEED");
	if (a == -1) return -1;
	
	speed.per = stod(strVector.at(1).substr(strVector.at(1).find("per=") + 4));
	speed.tcp = stod(strVector.at(2).substr(strVector.at(2).find("tcp=") + 4));
	speed.ori = stod(strVector.at(3).substr(strVector.at(3).find("ori=") + 4));
	speed.exp_7 = stod(strVector.at(4).substr(strVector.at(4).find("exj_l=") + 6));
	speed.exp_8 = stod(strVector.at(5).substr(strVector.at(5).find("exj_r=") + 6,8));

	return 0;
}

int ESTUNRobotCtrl::ErdWeave2WeaveDate(CString & ErdWeave, ESTUN_WeaveDate & weave)
{
	map<int, vector<string>> ErdSplit;
	std::string str = ErdWeave.GetBuffer();
	if (ErdWeave == "") return-1;

	string str2;
	vector<string> strVector;

	int last_index = 0;
	int index = 0;
	while (str.find(",") + 1)//当找不到空格时返回-1，所以这里我用它返回值加1来循环
	{
		index = str.find(",");
		str2 = str.substr(0, index);
		str = str.substr(index + 1);
		strVector.push_back(str2);
	}
	strVector.push_back(str);
	int a = strVector.at(0).find("WEAVE");
	if (a == -1) return -1;


	weave.Type		 = stod(strVector.at(1).substr(strVector.at(1).find("Type=") + 5));
	weave.Freq		 = stof(strVector.at(2).substr(strVector.at(2).find("Freq=") + 5));
	weave.Amp_L		 = stof(strVector.at(3).substr(strVector.at(3).find("Amp_L=") + 6));
	weave.Amp_R		 = stof(strVector.at(4).substr(strVector.at(4).find("Amp_R=") + 6));
	weave.StopTime_L = stod(strVector.at(5).substr(strVector.at(5).find("StopTime_L=") + 11));
	weave.StopTime_C = stod(strVector.at(6).substr(strVector.at(6).find("StopTime_C=") + 11));
	weave.StopTime_R = stod(strVector.at(7).substr(strVector.at(7).find("StopTime_R=") + 11));
	weave.RotAngle_X = stof(strVector.at(8).substr(strVector.at(8).find("RotAngle_X=") + 11));
	weave.RotAngle_Z = stof(strVector.at(9).substr(strVector.at(9).find("RotAngle_Z=") + 11));


	return 0;
}
