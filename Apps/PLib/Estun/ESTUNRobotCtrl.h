#pragma once
#ifndef __ESTUN_ROBOT_CTRL
#define __ESTUN_ROBOT_CTRL
#include <vector>
#include <string>
#include <map>
#include "G_xml.h"
#include "tinyxml.h"
#include <afxinet.h>
#include "TCP_Socket_ERC.h"

//#include "./Apps/PLib/Estun/EstunRemoteApiLibWeld.h"
//#include "./Apps/PLib/Estun/ESTUN_LIB/ESTUNRobotCtrl.h"
#ifndef PI
#define PI 3.14159265
#endif // !PI

using namespace std;
typedef struct
{
	double X;
	double Y;
	double Z;
	double A;
	double B;
	double C;
}T_ROBOT_POS;

typedef struct
{
	double A1;
	double A2;
	double A3;
	double A4;
	double A5;
	double A6;
}T_ROBOT_AXIS;

typedef struct
{
	//UINT coorType;
	//UINT toolNo;
	double p[6];  //1-6轴
	double ExP[2];//7-8外部轴
	double speed; //直角坐标/关节坐标速度
	double Ex_7_speed;//7轴速度
	double Ex_8_speed;//8轴速度
	int mode;
	int cf1;
	int cf2;
	int cf3;
	int cf4;
	int cf5;
	int cf6;
}T_ROBOT_POSVAR;
typedef struct
{
	double R;
	double L2;
	double S;
	double L3;
}ESTUN_MODE;
typedef struct
{
	int Type;
	double Freq;
	double Amp_L;
	double Amp_R;
	int StopTime_L;
	int StopTime_C;
	int StopTime_R;
	double RotAngle_X;
	double RotAngle_Z; 
	int DelayType_L;
	int DelayType_C;
	int DelayType_R;
}ESTUN_WeaveDate;
typedef struct
{
	int mode;
	int cf1;
	int cf2;
	int cf3;
	int cf4;
	int cf5;
	int cf6;
	double X;
	double Y;
	double Z;
	double A;
	double B;
	double C;
	double Axis_7;
	double Axis_8;
}ESTUN_CPOS;
typedef struct
{
	double A1;
	double A2;
	double A3;
	double A4;
	double A5;
	double A6;
	double A7;
	double A8;
}ESTUN_APOS;
typedef struct
{
	double per;
	double tcp;
	double ori;
	double exp_7;
	double exp_8;
}ESTUN_SPEED;
typedef struct
{
	/****************************/
	int  INT_Count;//int变量数量，多少个int变量，比如：7
	int* INT_Name; //int变量名，比如：数组[2,3,4,7,8,9,23]
	int* INT_Value; //int变量值  比如：数组[12,23,24,24,53,345,3456]
	/****************************/
	int  CPOS_Count;//直角坐标变量数量，多少个int变量，比如7个
	int* CPOS_Name; //直角坐标变量名，比如：数组[2,3,4,7,8,9,23]
	ESTUN_CPOS* CPOS_Value; //直角变量值数组
	/****************************/
	int  APOS_Count;//关节坐标变量数量，多少个int变量，比如7个
	int* APOS_Name; //关节坐标变量名，比如：数组[2,3,4,7,8,9,23]
	ESTUN_APOS* APOS_Value; //关节变量值数组
	/***************************/
	int  SPEED_Count;//速度变量变量数量，多少个int变量，比如7个
	int* SPEED_Name; //速度变量变量名，比如：数组[2,3,4,7,8,9,23]
	ESTUN_SPEED* SPEED_Value; //速度变量值数组
	/***************************/
}T_MIX_VAR;
enum E_CMD_SCRIPT
{
	E_HAND = 0x01,
	E_MultiPos = 0x14,//直角坐标
	E_MultiPos_J = 0x15, //关节坐标
	E_Speed = 0x16,     //设置速度
	E_GetRobotInformation = 0x17,//获取机器人信息
	E_GetSpeed = 0x18,//获取速度变量
	E_GetNameSpeed = 0x19,//获取指定名string 速度变量
	E_CloseScriptAutoAddPos = 0x1a,//	取消脚本自动替换点
	E_SetMultiSpeed = 0x1b,        //设置多个速度
	E_GetCurrentPos = 0x1c,        //获取当前世界坐标
	E_GetCurrentAxis = 0x1d,        //获取当前关节坐标
	E_Stop = 0x1e,                 //停止
	E_SetSysMod = 0x1f,            //设置示教器模式，手动 自动 远程
	E_GetSysMod = 0x20,            //获得示教器模式，手动 自动 远程
	E_SetPCPoint = 0x21,         //设置PC指针
	E_SerMultiPos_L = 0x22,       //设置大量P变量，不带speed，且小于300点数，8个轴数据
	E_SerMultiPos_J = 0x23,        //设置大量P变量，不带speed，且小于300点数，8个轴数据
	E_ClearErr = 0x24,             //清除报警
	E_LoadProgramer = 0x25,        //加载程序
	E_startRun = 0x26,             //启动程序
	E_setPosVar = 0x27,           //设置pos变量
	E_setjPosVar = 0x28,           //设置Jpos变量
	E_SetIntVar = 0x29,           //设置Int变量
	E_GetIntVar = 0x2a,          //获得Int变量
	E_SetRealVar = 0x2b,         //设置Real变量
	E_GetRealVar = 0x2c,          //获得Real变量
	E_SetToolNum = 0x2d,         //设置工具号
	E_SerUserNum = 0x2e,       //设置用户号
	E_SetTPSpeed = 0x2f,       //设置TP示教器速度
	E_SetOutVar = 0x30,        //设置out端口变量
	E_GetOutVar = 0x31,        //得到out端口变量
	E_GetInVar = 0x32,        //得到Int端口变量
	E_SetSpeed_N = 0X33,     //设置速度_name
	E_MixMultiValue = 0x34, //设置大量混合变量，INT,CPOS,APOS,SPEED
	E_GetCposValue = 0x35,  //读cpos变量
	E_GetAposValue = 0x36,  //读cpos变量
	E_SetMultiCpos = 0x37, //设置大量点cpos，multi脚本
	E_SetMultiApos = 0x38,  //设置大量点Apos，multi脚本
	E_GetCurToolName = 0x39,//获得当前工具名
	E_GetCurUserName = 0x3a,//获得当前用户名
	E_GetToolDate = 0x3b,//获得工具数据
	E_GetUserDate = 0x3c,//获得当前用户数据
	E_SetToolDate = 0x3d,//设置工具数据
	E_SetUserDate = 0x3e,//设置用户数据
	E_GetCheckDone = 0x3f,//主动获取当前状态
	E_SetWeaveDate = 0x40,//设置摆弧变量数据
	E_GetServoSts = 0x41,//获取伺服状态
};
class ESTUNRobotCtrl
{
public:
	ESTUNRobotCtrl();
	~ESTUNRobotCtrl();
public:
	//机器人要在全局变量里面设置string数据类型，Py_IP:192.168.60.2 Py_Port:10000;
	int initSocket(char* ip, UINT Port, bool ifRecode = false);
	//sockaddr_in Robot_ClinerAddr();
	//bool Robot_Send(CString str);
	//bool Robot_connect();//判断3D接口是否右机器人链接过来了
	bool Robot_ScriptConnect();//判断command脚本是否连接
	bool Robot_FDTScriptConnect();//判断FDT脚本是否连接
	bool Robot_MultiScriptConnect();//判断Multi脚本是否连接
	//3d接口通讯，脚本功能通讯，快速脚本通讯,开关记录信息
	void IfRecordInformation(bool MultiScriptTcp, bool ScriptTcp, bool FastDateTrScriptTcp);
	/*****************************************************Python脚本对接功能****************************************************************/
	//超过300点用这个接口
	int SetMultiPosVar_M(UINT index, UINT count, const T_ROBOT_POSVAR var[], string Program_name, int coord, int ToolNumber, int UserCoord);//起始位置，数量，0-直角，1-关节
	//设置多个位置参数,包含7轴8轴数据，300点以下
	int SetMultiPosVar(UINT index, UINT count, const T_ROBOT_POSVAR var[], int coord, int ToolNumber, int UserCoord);//起始位置，数量，0-直角，1-关节
	//设置小于300个点的参数，包含7，8轴数据Axis_10_Date[][15]
	//数据 X,Y,Z,A,B,C,7,8,mode,cf1,cf2,cf3,cf4,cf5,cf6
	int SetMultiPosVar_8_NoSpeed(UINT index, UINT count, double** Axis_8_Date, int coord, int ToolNumber, int UserCoord);//起始位置，数量，0-直角，1-关节
	//设置小于300个点的参数,默认7，8，轴数据为0,Axis_10_Date[][13]	
	//数据 X,Y,Z,A,B,C,mode,cf1,cf2,cf3,cf4,cf5,cf6//设置小于300个点的参数
	int SetMultiPosVar_6_NoSpeed(UINT index, UINT count, double** Axis_6_Date, int coord, int ToolNumber, int UserCoord);
	//设置大量的速度speedVar[][5]
	int SetMultiSpeed(UINT index, UINT count, double** speedVar);
	//pos[9]:x y z a b c 7 8   , scoper,0-系统，1-全局，2-工程，3-程序 , Coord,0-直角，1-关节
	//pos[8]:a1 a2 a3 a4 a5 a6 a7 a8,设置关节
	//config[0]:mode,config[1];cf1,config[2];cf2,config[3];cf3,config[4];cf4,config[5];cf5,config[6];cf6
	//对mP,mJP赋值
	int SetPosVar_Py(int mP_Number, double pos[8], int config[7], int scoper = 2, int Coord = 0);
	//pos[9]:x y z a b c 7 8   , scoper,0-系统，1-全局，2-工程，3-程序 , Coord,0-直角，1-关节
	//pos[8]:a1 a2 a3 a4 a5 a6 a7 a8,设置关节
	//config[0]:mode,config[1];cf1,config[2];cf2,config[3];cf3,config[4];cf4,config[5];cf5,config[6];cf6
	int SetPosVar_Py(const char* PosName, double pos[8], int config[7], int scoper = 2, int Coord = 0);
	//混合传输类型为INT CPOS APOS SPEED类型数据
	int SetMultiVar_H(T_MIX_VAR& Mix_struct);
	// 读 cpos变量
	int GetCPosVar(const char* name, ESTUN_CPOS* Cpos, int scoper = 2);
	//读 cpos变量
	int GetAPosVar(const char* name, ESTUN_APOS* Apos, int scoper = 2);
	//设置单个速度speed[5]
	int SetSpeed(int index, double* speed);
	//设置单个速度speed[5]
	int SetSpeed(const char* name, double* speed, int scord = 2);
	//获得单个速度speed[5]
	int GetSpeed(int index, double* speed);
	//获得单个速度speed[5]
	int GetSpeed(const char* speedName, double* speed);
	//得到机器人识别码
	int GetRobotSerialNum(string information_s);
	//传输点数大于300时，使用此停止且关闭本次自动替换坐标功能
	int CloseScriptAutoAddPos();
	//获得当前的直角坐标，包含10个轴pos[10],mode[7];mode,cf1,cf2,cf3,cf4,cf5,cf6
	int GetCurPos(double* pos, int* mode);
	//获得当前的关节坐标，包含10个轴axis[10]
	int GetCurAxis(double* axis);
	//获得当前的直角坐标，包含10个轴pos[10]
	int GetCurPos_Fast(double* pos);
	//获得当前的关节坐标，包含10个轴axis[10]
	int GetCurAxis_Fast(double* axis);
	//获取当前机器人运行状态,1-运行，2-暂停，3-停止
	int CheckDone_Fast();
	//获取当前机器人运行状态，1-运行，2-暂停，3-停止
	int CheckDone();
	//清除报警
	int ErrClear_Py();
	//传输点数小于300点时，可以使用stop停止
	int Stop();   //程序停止
	//0-手动模式，1-自动模式，2-远程模式
	//设置当前模式
	int SetSysMode(int mode);
	//获得当前模式,0-手动模式，1-自动模式，2-远程模式，-1-报错
	int GetSysMode();
	//设置PC指针，未成功
	int SetPCPointNumber(int number);
	//加载程序
	int LoadUserProgramer_Py(const char* projName, const char* progName);
	//启动程序
	int Prog_startRun_Py();
	//设置INT变量
	int SetIntVar_Py(const char* name, int value, int score = 2);
	int GetIntVar_Py(const char* name, int& value, int score = 2);
	//设置Real变量
	int SetRealVar_Py(const char* name, double value, int score = 2);
	int GetRealVar_Py(const char* name, double& value, int score = 2);
	//设置IO变量
	int Set_O_Var_Py(int name, int value);
	int Get_O_Var_Py(int name, int& value);
	int Get_I_Var_Py(int name, int& value);
	//设置工具号
	int SetToolNum(const char* ToolName, int score = 2);
	//设置用户号
	int SetUserNum(const char* UserName, int score = 2);
	//计算mode值,机器人型号，50B，20 机器人
	void ModeValue(double Axis[6], const char* RobotMode, int config[7]);
	//计算mode值
	void ModeValue(double Axis[6], ESTUN_MODE Rod_length, int config[7]);
	//设置示教器速度
	int SetTpSpeed_Py(UINT speed);//设置TP示教器上的速度%
	//得到当前工具名
	int GetToolName(string& ToolName);
	//得到当前用户名
	int GetUserCoolName(string& UserName);
	////获得工具坐标
	int GetToolDate(const char* name, double ToolDate[6], int scope = 1);
	////得到用户坐标
	int GetUserDate(const char* name, double UserDate[6], int scope = 1);
	//设置用户坐标
	int SetUserDate(const char* name, double UserDate[6], int scope = 1);
	//设置工具坐标
	int SetToolDate(const char* name, double ToolDate[6], int scope = 1);
	//设置摆弧变量数据
	int SetWeaveDate(const char* name, ESTUN_WeaveDate WeaveDate, int scope = 2);
	//
	//获得伺服状态,返回值1表示使能，0表示非使能状态，-1表示通讯超时或python接口执行失败
	int GetServoSts();
	/***************************************************运动函数*********************************************************************/

	int MoveByJob(int Axis, double Distence, int config[7], double speed, int ifAbsoluteM = 0, int ifJoint = 0);//轴号，距离，速度，0-相对 1-绝对，0-直角，1-关节；
	//8个轴的数据，,第9个是mode值,速度，0-相对 1-绝对，0-直角，1-关节
	int MoveByJob(double Distence[8], int config[7], double speed, int ifAbsolutM, int ifJoint);
	/*****************************************************FTP功能****************************************************************************/
	int ConfigFTP(CString ftp_ip, CString user_name, CString pass_word);
	//FTP建立连接
	int ConnectFtp();
	//断开连接
	int DisConnectFtp();
	//上传文件给埃斯顿机器人，埃斯顿为RemoteFilePath，本地为LocalFilePath    //  .//MultiPos_Mv1.erd 
	int UploadFile(CString RemoteFilePath, CString LocalFilePath);
	//下载文件,埃斯顿为RemoteFilePath，本地为LocalFilePath
	int DownloadFile(CString RemoteFilePath, CString LocalFilePath);


	//埃斯顿低层参数数据文件转为ini格式的配置文件，主要目的是可以用ini格式接口函数修改里面的数据
	int ErdFile2IniFile(CString ErdFilePath, CString IniFilePath);
	//将ini格式文件转换为埃斯顿标准的参数文件，目的是传输回埃斯顿底层里面
	int IniFile2ErdFile(CString IniFilePath, CString ErdFilePath);
	//读取Ini文件里的参数    ParameterValue.GetBuffer(ParameterLong), ParameterLong
	int ReadIniFileParameter(CString IniFilePath, CString ParameterName, LPSTR ParameterValue, int ParameterLong);
	//修改Ini文件里的参数
	int WriteIniFileParameter(CString IniFilePath, CString ParameterName, CString ParameterValue);

	//直角坐标转埃斯顿对应的语法字符串
	int CPos2ErdCPos(ESTUN_CPOS cpos, CString& ErdCPos);
	//关节坐标转埃斯顿对应的语法字符串
	int APos2ErdAPos(ESTUN_APOS APos, CString& ErdAPos);
	//速度转埃斯顿对应的语法字符串
	int Speed2ErdSpeed(ESTUN_SPEED speed, CString& ErdSpeed);
	//焊接参数转埃斯顿对应的语法字符串
	int WeaveDate2ErdWeave(ESTUN_WeaveDate weave, CString& ErdWeave);

	//埃斯顿直角坐标字符串转直角坐标
	int ErdCPos2CPos(CString& ErdCPos, ESTUN_CPOS& cpos);
	//埃斯顿关节坐标字符串转关节坐标
	int ErdAPos2APos(CString& ErdAPos, ESTUN_APOS& APos);
	//埃斯顿速度字符串转速度值
	int ErdSpeed2Speed(CString& ErdSpeed, ESTUN_SPEED& speed);
	//埃斯顿摆焊转摆焊参数
	int ErdWeave2WeaveDate(CString& ErdWeave, ESTUN_WeaveDate& weave);



private:
	int initScriptSocket(char* ip, UINT Port, bool ifRecord);//cmd脚本
	int initFDTScriptSocket(char* ip, UINT Port, bool ifRecord);//FDT脚本
	int initMultiScriptSocket(char* ip, UINT Port, bool ifRecord);//multi脚本
	int ScriptsendData(int cmd, vector<string>* msg);//xml数据格式，command脚本接口
	int ScriptsendData_M(int cmd, vector<string>* msg);//xml数据格式,multi脚本接口
	double m_CurPos[10];
	double m_CurAxis[10];
	int m_checkDone;
	bool m_FDTSctiptConnection;
	vector<string>* sendBuf;
	vector<T_ROBOT_POSVAR>* sendPosVar;

	vector<ESTUN_CPOS>* sendCposVar_M;
	vector<ESTUN_APOS>* sendAposVar_M;
	vector<ESTUN_SPEED>* sendSpeedVar_M;
	vector<int*>* sendIntVar_M;

	vector<int*>* sendCposName_M;
	vector<int*>* sendAposName_M;
	vector<int*>* sendSpeedName_M;
	vector<int*>* sendIntName_M;

	vector<double*>* sendPosVar_noSpeed;
	vector<double*>* sendSpeedVar;
	int wait(int id, UINT timeout = 0);
	int script_wait(int id, UINT timeout = 0);
	void static Rcv(void* pOwner, const char* rcvData, int nDataLen); //3D接口
	void static ScriptRcv(void* pOwner, const char* rcvData, int nDataLen); //command接口 //Multi接口
	void static ScriptRcv_FDT(void* pOwner, const char* rcvData, int nDataLen);//FDT接口
	//void static ScriptRcv_Multi(void * pOwner, const char * rcvData, int nDataLen);//Multi接口
private:
	HANDLE m_mutex;
	map<int, vector<string>>* rcvBuf;
	map<int, vector<string>>* script_rcvBuf;
	vector<T_ROBOT_POSVAR>* rcvPosVar;
	bool if8axis;
	int cfX(double axis);
	/************FTP*****************/
	BOOL FTPConnect;
	CInternetSession* pInternetSession;
	CFtpConnection* pFtpConnection;
	CString ftp_ip, user_name, pass_word;
	CString FTPConnetionErr;

public:
	bool  Err_bool;
	string Err_str;

	TCP_Socket_ERC* ScriptTcp;
	TCP_Socket_ERC* FastDateTrScriptTcp;
	TCP_Socket_ERC* MultiScriptTcp;
};
#endif // !__ESTUN_ROBOT_CTRL
