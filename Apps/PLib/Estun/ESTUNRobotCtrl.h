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
	double p[6];  //1-6��
	double ExP[2];//7-8�ⲿ��
	double speed; //ֱ������/�ؽ������ٶ�
	double Ex_7_speed;//7���ٶ�
	double Ex_8_speed;//8���ٶ�
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
	int  INT_Count;//int�������������ٸ�int���������磺7
	int* INT_Name; //int�����������磺����[2,3,4,7,8,9,23]
	int* INT_Value; //int����ֵ  ���磺����[12,23,24,24,53,345,3456]
	/****************************/
	int  CPOS_Count;//ֱ������������������ٸ�int����������7��
	int* CPOS_Name; //ֱ����������������磺����[2,3,4,7,8,9,23]
	ESTUN_CPOS* CPOS_Value; //ֱ�Ǳ���ֵ����
	/****************************/
	int  APOS_Count;//�ؽ�����������������ٸ�int����������7��
	int* APOS_Name; //�ؽ���������������磺����[2,3,4,7,8,9,23]
	ESTUN_APOS* APOS_Value; //�ؽڱ���ֵ����
	/***************************/
	int  SPEED_Count;//�ٶȱ����������������ٸ�int����������7��
	int* SPEED_Name; //�ٶȱ��������������磺����[2,3,4,7,8,9,23]
	ESTUN_SPEED* SPEED_Value; //�ٶȱ���ֵ����
	/***************************/
}T_MIX_VAR;
enum E_CMD_SCRIPT
{
	E_HAND = 0x01,
	E_MultiPos = 0x14,//ֱ������
	E_MultiPos_J = 0x15, //�ؽ�����
	E_Speed = 0x16,     //�����ٶ�
	E_GetRobotInformation = 0x17,//��ȡ��������Ϣ
	E_GetSpeed = 0x18,//��ȡ�ٶȱ���
	E_GetNameSpeed = 0x19,//��ȡָ����string �ٶȱ���
	E_CloseScriptAutoAddPos = 0x1a,//	ȡ���ű��Զ��滻��
	E_SetMultiSpeed = 0x1b,        //���ö���ٶ�
	E_GetCurrentPos = 0x1c,        //��ȡ��ǰ��������
	E_GetCurrentAxis = 0x1d,        //��ȡ��ǰ�ؽ�����
	E_Stop = 0x1e,                 //ֹͣ
	E_SetSysMod = 0x1f,            //����ʾ����ģʽ���ֶ� �Զ� Զ��
	E_GetSysMod = 0x20,            //���ʾ����ģʽ���ֶ� �Զ� Զ��
	E_SetPCPoint = 0x21,         //����PCָ��
	E_SerMultiPos_L = 0x22,       //���ô���P����������speed����С��300������8��������
	E_SerMultiPos_J = 0x23,        //���ô���P����������speed����С��300������8��������
	E_ClearErr = 0x24,             //�������
	E_LoadProgramer = 0x25,        //���س���
	E_startRun = 0x26,             //��������
	E_setPosVar = 0x27,           //����pos����
	E_setjPosVar = 0x28,           //����Jpos����
	E_SetIntVar = 0x29,           //����Int����
	E_GetIntVar = 0x2a,          //���Int����
	E_SetRealVar = 0x2b,         //����Real����
	E_GetRealVar = 0x2c,          //���Real����
	E_SetToolNum = 0x2d,         //���ù��ߺ�
	E_SerUserNum = 0x2e,       //�����û���
	E_SetTPSpeed = 0x2f,       //����TPʾ�����ٶ�
	E_SetOutVar = 0x30,        //����out�˿ڱ���
	E_GetOutVar = 0x31,        //�õ�out�˿ڱ���
	E_GetInVar = 0x32,        //�õ�Int�˿ڱ���
	E_SetSpeed_N = 0X33,     //�����ٶ�_name
	E_MixMultiValue = 0x34, //���ô�����ϱ�����INT,CPOS,APOS,SPEED
	E_GetCposValue = 0x35,  //��cpos����
	E_GetAposValue = 0x36,  //��cpos����
	E_SetMultiCpos = 0x37, //���ô�����cpos��multi�ű�
	E_SetMultiApos = 0x38,  //���ô�����Apos��multi�ű�
	E_GetCurToolName = 0x39,//��õ�ǰ������
	E_GetCurUserName = 0x3a,//��õ�ǰ�û���
	E_GetToolDate = 0x3b,//��ù�������
	E_GetUserDate = 0x3c,//��õ�ǰ�û�����
	E_SetToolDate = 0x3d,//���ù�������
	E_SetUserDate = 0x3e,//�����û�����
	E_GetCheckDone = 0x3f,//������ȡ��ǰ״̬
	E_SetWeaveDate = 0x40,//���ðڻ���������
	E_GetServoSts = 0x41,//��ȡ�ŷ�״̬
};
class ESTUNRobotCtrl
{
public:
	ESTUNRobotCtrl();
	~ESTUNRobotCtrl();
public:
	//������Ҫ��ȫ�ֱ�����������string�������ͣ�Py_IP:192.168.60.2 Py_Port:10000;
	int initSocket(char* ip, UINT Port, bool ifRecode = false);
	//sockaddr_in Robot_ClinerAddr();
	//bool Robot_Send(CString str);
	//bool Robot_connect();//�ж�3D�ӿ��Ƿ��һ��������ӹ�����
	bool Robot_ScriptConnect();//�ж�command�ű��Ƿ�����
	bool Robot_FDTScriptConnect();//�ж�FDT�ű��Ƿ�����
	bool Robot_MultiScriptConnect();//�ж�Multi�ű��Ƿ�����
	//3d�ӿ�ͨѶ���ű�����ͨѶ�����ٽű�ͨѶ,���ؼ�¼��Ϣ
	void IfRecordInformation(bool MultiScriptTcp, bool ScriptTcp, bool FastDateTrScriptTcp);
	/*****************************************************Python�ű��Խӹ���****************************************************************/
	//����300��������ӿ�
	int SetMultiPosVar_M(UINT index, UINT count, const T_ROBOT_POSVAR var[], string Program_name, int coord, int ToolNumber, int UserCoord);//��ʼλ�ã�������0-ֱ�ǣ�1-�ؽ�
	//���ö��λ�ò���,����7��8�����ݣ�300������
	int SetMultiPosVar(UINT index, UINT count, const T_ROBOT_POSVAR var[], int coord, int ToolNumber, int UserCoord);//��ʼλ�ã�������0-ֱ�ǣ�1-�ؽ�
	//����С��300����Ĳ���������7��8������Axis_10_Date[][15]
	//���� X,Y,Z,A,B,C,7,8,mode,cf1,cf2,cf3,cf4,cf5,cf6
	int SetMultiPosVar_8_NoSpeed(UINT index, UINT count, double** Axis_8_Date, int coord, int ToolNumber, int UserCoord);//��ʼλ�ã�������0-ֱ�ǣ�1-�ؽ�
	//����С��300����Ĳ���,Ĭ��7��8��������Ϊ0,Axis_10_Date[][13]	
	//���� X,Y,Z,A,B,C,mode,cf1,cf2,cf3,cf4,cf5,cf6//����С��300����Ĳ���
	int SetMultiPosVar_6_NoSpeed(UINT index, UINT count, double** Axis_6_Date, int coord, int ToolNumber, int UserCoord);
	//���ô������ٶ�speedVar[][5]
	int SetMultiSpeed(UINT index, UINT count, double** speedVar);
	//pos[9]:x y z a b c 7 8   , scoper,0-ϵͳ��1-ȫ�֣�2-���̣�3-���� , Coord,0-ֱ�ǣ�1-�ؽ�
	//pos[8]:a1 a2 a3 a4 a5 a6 a7 a8,���ùؽ�
	//config[0]:mode,config[1];cf1,config[2];cf2,config[3];cf3,config[4];cf4,config[5];cf5,config[6];cf6
	//��mP,mJP��ֵ
	int SetPosVar_Py(int mP_Number, double pos[8], int config[7], int scoper = 2, int Coord = 0);
	//pos[9]:x y z a b c 7 8   , scoper,0-ϵͳ��1-ȫ�֣�2-���̣�3-���� , Coord,0-ֱ�ǣ�1-�ؽ�
	//pos[8]:a1 a2 a3 a4 a5 a6 a7 a8,���ùؽ�
	//config[0]:mode,config[1];cf1,config[2];cf2,config[3];cf3,config[4];cf4,config[5];cf5,config[6];cf6
	int SetPosVar_Py(const char* PosName, double pos[8], int config[7], int scoper = 2, int Coord = 0);
	//��ϴ�������ΪINT CPOS APOS SPEED��������
	int SetMultiVar_H(T_MIX_VAR& Mix_struct);
	// �� cpos����
	int GetCPosVar(const char* name, ESTUN_CPOS* Cpos, int scoper = 2);
	//�� cpos����
	int GetAPosVar(const char* name, ESTUN_APOS* Apos, int scoper = 2);
	//���õ����ٶ�speed[5]
	int SetSpeed(int index, double* speed);
	//���õ����ٶ�speed[5]
	int SetSpeed(const char* name, double* speed, int scord = 2);
	//��õ����ٶ�speed[5]
	int GetSpeed(int index, double* speed);
	//��õ����ٶ�speed[5]
	int GetSpeed(const char* speedName, double* speed);
	//�õ�������ʶ����
	int GetRobotSerialNum(string information_s);
	//�����������300ʱ��ʹ�ô�ֹͣ�ҹرձ����Զ��滻���깦��
	int CloseScriptAutoAddPos();
	//��õ�ǰ��ֱ�����꣬����10����pos[10],mode[7];mode,cf1,cf2,cf3,cf4,cf5,cf6
	int GetCurPos(double* pos, int* mode);
	//��õ�ǰ�Ĺؽ����꣬����10����axis[10]
	int GetCurAxis(double* axis);
	//��õ�ǰ��ֱ�����꣬����10����pos[10]
	int GetCurPos_Fast(double* pos);
	//��õ�ǰ�Ĺؽ����꣬����10����axis[10]
	int GetCurAxis_Fast(double* axis);
	//��ȡ��ǰ����������״̬,1-���У�2-��ͣ��3-ֹͣ
	int CheckDone_Fast();
	//��ȡ��ǰ����������״̬��1-���У�2-��ͣ��3-ֹͣ
	int CheckDone();
	//�������
	int ErrClear_Py();
	//�������С��300��ʱ������ʹ��stopֹͣ
	int Stop();   //����ֹͣ
	//0-�ֶ�ģʽ��1-�Զ�ģʽ��2-Զ��ģʽ
	//���õ�ǰģʽ
	int SetSysMode(int mode);
	//��õ�ǰģʽ,0-�ֶ�ģʽ��1-�Զ�ģʽ��2-Զ��ģʽ��-1-����
	int GetSysMode();
	//����PCָ�룬δ�ɹ�
	int SetPCPointNumber(int number);
	//���س���
	int LoadUserProgramer_Py(const char* projName, const char* progName);
	//��������
	int Prog_startRun_Py();
	//����INT����
	int SetIntVar_Py(const char* name, int value, int score = 2);
	int GetIntVar_Py(const char* name, int& value, int score = 2);
	//����Real����
	int SetRealVar_Py(const char* name, double value, int score = 2);
	int GetRealVar_Py(const char* name, double& value, int score = 2);
	//����IO����
	int Set_O_Var_Py(int name, int value);
	int Get_O_Var_Py(int name, int& value);
	int Get_I_Var_Py(int name, int& value);
	//���ù��ߺ�
	int SetToolNum(const char* ToolName, int score = 2);
	//�����û���
	int SetUserNum(const char* UserName, int score = 2);
	//����modeֵ,�������ͺţ�50B��20 ������
	void ModeValue(double Axis[6], const char* RobotMode, int config[7]);
	//����modeֵ
	void ModeValue(double Axis[6], ESTUN_MODE Rod_length, int config[7]);
	//����ʾ�����ٶ�
	int SetTpSpeed_Py(UINT speed);//����TPʾ�����ϵ��ٶ�%
	//�õ���ǰ������
	int GetToolName(string& ToolName);
	//�õ���ǰ�û���
	int GetUserCoolName(string& UserName);
	////��ù�������
	int GetToolDate(const char* name, double ToolDate[6], int scope = 1);
	////�õ��û�����
	int GetUserDate(const char* name, double UserDate[6], int scope = 1);
	//�����û�����
	int SetUserDate(const char* name, double UserDate[6], int scope = 1);
	//���ù�������
	int SetToolDate(const char* name, double ToolDate[6], int scope = 1);
	//���ðڻ���������
	int SetWeaveDate(const char* name, ESTUN_WeaveDate WeaveDate, int scope = 2);
	//
	//����ŷ�״̬,����ֵ1��ʾʹ�ܣ�0��ʾ��ʹ��״̬��-1��ʾͨѶ��ʱ��python�ӿ�ִ��ʧ��
	int GetServoSts();
	/***************************************************�˶�����*********************************************************************/

	int MoveByJob(int Axis, double Distence, int config[7], double speed, int ifAbsoluteM = 0, int ifJoint = 0);//��ţ����룬�ٶȣ�0-��� 1-���ԣ�0-ֱ�ǣ�1-�ؽڣ�
	//8��������ݣ�,��9����modeֵ,�ٶȣ�0-��� 1-���ԣ�0-ֱ�ǣ�1-�ؽ�
	int MoveByJob(double Distence[8], int config[7], double speed, int ifAbsolutM, int ifJoint);
	/*****************************************************FTP����****************************************************************************/
	int ConfigFTP(CString ftp_ip, CString user_name, CString pass_word);
	//FTP��������
	int ConnectFtp();
	//�Ͽ�����
	int DisConnectFtp();
	//�ϴ��ļ�����˹�ٻ����ˣ���˹��ΪRemoteFilePath������ΪLocalFilePath    //  .//MultiPos_Mv1.erd 
	int UploadFile(CString RemoteFilePath, CString LocalFilePath);
	//�����ļ�,��˹��ΪRemoteFilePath������ΪLocalFilePath
	int DownloadFile(CString RemoteFilePath, CString LocalFilePath);


	//��˹�ٵͲ���������ļ�תΪini��ʽ�������ļ�����ҪĿ���ǿ�����ini��ʽ�ӿں����޸����������
	int ErdFile2IniFile(CString ErdFilePath, CString IniFilePath);
	//��ini��ʽ�ļ�ת��Ϊ��˹�ٱ�׼�Ĳ����ļ���Ŀ���Ǵ���ذ�˹�ٵײ�����
	int IniFile2ErdFile(CString IniFilePath, CString ErdFilePath);
	//��ȡIni�ļ���Ĳ���    ParameterValue.GetBuffer(ParameterLong), ParameterLong
	int ReadIniFileParameter(CString IniFilePath, CString ParameterName, LPSTR ParameterValue, int ParameterLong);
	//�޸�Ini�ļ���Ĳ���
	int WriteIniFileParameter(CString IniFilePath, CString ParameterName, CString ParameterValue);

	//ֱ������ת��˹�ٶ�Ӧ���﷨�ַ���
	int CPos2ErdCPos(ESTUN_CPOS cpos, CString& ErdCPos);
	//�ؽ�����ת��˹�ٶ�Ӧ���﷨�ַ���
	int APos2ErdAPos(ESTUN_APOS APos, CString& ErdAPos);
	//�ٶ�ת��˹�ٶ�Ӧ���﷨�ַ���
	int Speed2ErdSpeed(ESTUN_SPEED speed, CString& ErdSpeed);
	//���Ӳ���ת��˹�ٶ�Ӧ���﷨�ַ���
	int WeaveDate2ErdWeave(ESTUN_WeaveDate weave, CString& ErdWeave);

	//��˹��ֱ�������ַ���תֱ������
	int ErdCPos2CPos(CString& ErdCPos, ESTUN_CPOS& cpos);
	//��˹�ٹؽ������ַ���ת�ؽ�����
	int ErdAPos2APos(CString& ErdAPos, ESTUN_APOS& APos);
	//��˹���ٶ��ַ���ת�ٶ�ֵ
	int ErdSpeed2Speed(CString& ErdSpeed, ESTUN_SPEED& speed);
	//��˹�ٰں�ת�ں�����
	int ErdWeave2WeaveDate(CString& ErdWeave, ESTUN_WeaveDate& weave);



private:
	int initScriptSocket(char* ip, UINT Port, bool ifRecord);//cmd�ű�
	int initFDTScriptSocket(char* ip, UINT Port, bool ifRecord);//FDT�ű�
	int initMultiScriptSocket(char* ip, UINT Port, bool ifRecord);//multi�ű�
	int ScriptsendData(int cmd, vector<string>* msg);//xml���ݸ�ʽ��command�ű��ӿ�
	int ScriptsendData_M(int cmd, vector<string>* msg);//xml���ݸ�ʽ,multi�ű��ӿ�
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
	void static Rcv(void* pOwner, const char* rcvData, int nDataLen); //3D�ӿ�
	void static ScriptRcv(void* pOwner, const char* rcvData, int nDataLen); //command�ӿ� //Multi�ӿ�
	void static ScriptRcv_FDT(void* pOwner, const char* rcvData, int nDataLen);//FDT�ӿ�
	//void static ScriptRcv_Multi(void * pOwner, const char * rcvData, int nDataLen);//Multi�ӿ�
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
