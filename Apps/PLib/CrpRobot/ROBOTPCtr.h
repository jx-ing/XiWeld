#pragma once
#include"TCP_Socket_CRP.h"
#include <map>
#include <vector>
#include "cmath"
#include <Eigen\Eigen>

#include <limits>
#include <cmath>
#include <limits>

using namespace std;
//#define PI 3.1415926
#define uint32 UINT 
enum Function

{
	Read_coil_bite_register = 0x01,//����Ȧ�Ĵ���
	Read_Input_bite_register = 0x02,//��ȡ����Ĵ���
	Read_retain_word_register = 0x03,//�����ּĴ���
	Read_Input_word_register = 0x04,//��ȡ����Ĵ���
	Write_coil_bite_register = 0x05,//д������Ȧ�Ĵ���
	Write_retain_word_register = 0x06,//д�������ּĴ���
	Write_Multi_coil_register = 0x0F,//д�����Ȧ�Ĵ���
	Write_Multi_retain_register = 0x10//д������ּĴ���
};
typedef struct
{
	double X;
	double Y;
	double Z;
	double A;
	double B;
	double C;
	double A_7;
	double A_8;
}CRP_CPOS;//ֱ������
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
}CRP_APOS;//�ؽ�����
typedef struct
{
	double X;
	double Y;
	double Z;
	double A;
	double B;
	double C;
	double A_7;
	double A_8;
	UINT Tool;
	UINT User;
}CRP_TU_CPOS; //�����ߺź��û���ʶ��ֱ������
typedef struct
{
	double X;
	double Y;
	double Z;
	double A;
	double B;
	double C;
}CRP_T00L; //���߽ṹ��
typedef struct
{
	double X;
	double Y;
	double Z;
	double A;
	double B;
	double C;
}CRP_USERCOOR; //�û���ʶ�Ľṹ��
enum Function_able
{
	Read_YNumber_status = 0x01
};
class CROBOTPCtr
{
public:
	CROBOTPCtr();
	~CROBOTPCtr();
	void initApp(); //ͳһ��ʼ��
	void keepAlive(); //����
	int initModbusTCP(char * m_ip, u_short Prot, int flag = CLIENT);//0-��������1-�ͻ���;
	//prot:21Ϊ����˿ڣ���ʼ�������ӷ������������Ϻ���Զ�����FTP_Land ��¼
	int initFTP_cmd(char * m_ip, u_short Prot, char * name, char * password, int flag = CLIENT);
	int initToolInfo();//�޸��û���ϸ��Ϣ
	int initUserInfo();//�޸��û���ϸ��Ϣ
	int initTrackTCP(u_short Prot);
	int initToolUserDate(CRP_T00L Tool[],int ToolCount, CRP_USERCOOR UserCoor[],int UserCount);
	/**************************************ModbusTCP���ܽӿ�*************************************************************/
	/*��һ��Y��״̬:
	ͨ����� Y �ӿڣ��ýӿڶ�ӦӲ������ӿڣ�Ϊ״̬������ON=1��OFF=0������Χ 0-79��*/
	int Read_Y_Number_status(int YNumber);
	//дһ��Y��״̬
	int Write_Y_Number_status(int YNumber, bool status);
	//��һ��M��״̬���ڲ����� M �̵������ü̵���Ϊ״̬������ON=1��OFF = 0������Χ��0 - 800
	int Read_M_Number_status(int MNumber);
	//дһ��M��״̬
	int Write_M_Number_status(int MNumber, bool status);
	//��һ��X��״̬.ͨ������ X �ӿڣ��ýӿڶ�ӦӲ������ӿڣ�Ϊ״̬������ON = 1��OFF = 0������Χ��0 - 111��
	int Read_X_Number_status(int XNumber);
	//��һ��GI��״̬ ȫ��I����.��Χ��0-99������ֵΪ���������ݱ�������������
	int Read_GI_Number_status(int GINumber);
	//дһ��GI��״̬
	int Write_GI_Number_status(int GINumber,int date);
	//дһ��GP��״̬��10�����ݣ�
	//ȫ�� P �������ñ���ֵ��¼�������ؽ���̬����������λ�����ݣ��Ƕ��������ϡ������ŷ�Χ��0 - 999�����ݺŷ�ΧΪ��0 - 11
	int Write_GP_Number_single(int GPNumber,double date[]);
	//дһ��GP��״̬��10������
	int Write_GP_Number_single(int GPNumber, CRP_TU_CPOS &date);
	//д���GP��
	int Write_GP_Number_Multi(int GPNumber, CRP_TU_CPOS date[],int count);
	//��һ��GP����״̬��10������
	int Read_GP_Number_status(int GPNumber, double date[]);
	//��ȡ������ϸ��Ϣ
	int getToolInfo(int toolNumber, double data[]);
	//��ȡ�û���ϸ��Ϣ
	int getUserInfo(int userNumber, double data[]);
	//�޸��û���ϸ��Ϣ
	int setUserInfo(int userNumber, double data[]);
	/***********************************************��λ����дʾ�����������*************************************************************************/
	template<typename T> bool pose2ora(T const (&pose)[9], T &a, T &b, T &c);
	//����ת��
	void MatrixTranspose(double RotationMatrix[3][3]);
	//ŷ���� ת ��ת����
	/*���صĽ������˳��ʹ��ʱҪת��
	          Nx  Ny  Nz                               Nx  Ox  Ax
			  Ox  Oy  Oz							   Ny  Oy  Ay
			  Ax  Ay  Az       ��Ľӿ�ʹ��Ҫת��Ϊ��  Nz  Oz  Az
	*/
	void EulerAngleToVector(double Angle[3], double RotationMatrix[3][3]);
	//��ת���� ת ŷ����
	/*����������ʽ
		Nx  Ox  Ax
		Ny  Oy  Ay
	    Nz  Oz  Az
	*/
	void RotationToEulerAngle(double RotationMatrix[3][3], double Angle[3]);
	//��������ת����Ӧ�����µ�����
	void FlangeToTool_Pos(double F_pos[6],int ToolNumber,int UserNumber,double T_pos[6]);
	//�����µ�����תΪ�����µ�����
	void Tool_PosToFlange(double T_pos[6], int ToolNumber, int UserNumber, double F_pos[6]);
	/*********************************************����ʾ����ָ��*************************************************************/
	//����ǹؽ�����Pos[8]
	void MakeMoveJ_Axis(double Axis[], double speed, int Move_Tool, int Move_UserNu, int Acc, int Dec, int Smooth, int LineNumber, string* MoveJ_str);
	//����MoveJ ,�ؽ�����
	void MakeMoveJ_Axis(CRP_APOS &Axis, double speed, int Move_Tool, int Move_UserNu, int Acc, int Dec, int Smooth, int LineNumber, string* MoveJ_str);
	//����ʹ��ȫ�ֱ�����MoveJ
	void MakeMoveJByGP(int GPIndex, double speed, int Move_Tool, int Move_UserNu, int Acc, int Dec, int Smooth, int LineNumber, string* MoveJ_str);
	//����ʹ��ȫ�ֱ�����MoveL
	void MakeMoveLByGP(int GPIndex, double speed, int Move_Tool, int Move_UserNu, int Acc, int Dec, int Smooth, int LineNumber, string* MoveL_str);
	//����MoveLָ��
	void MakeMoveL(double LPos[],double speed,int Move_Tool,int Move_UserNu,int Acc,int Dec,int Smooth,int LineNumber,string* MoveL_str);
	void MakeMoveL(CRP_CPOS &CPos, double speed, int Move_Tool, int Move_UserNu, int Acc, int Dec, int Smooth, int LineNumber, string* MoveL_str);
	//����MoveJָ��
	void MakeMoveJ(double LPos[],double speed, int Move_Tool, int Move_UserNu, int Acc, int Dec, int Smooth, int LineNumber, string* MoveL_str);
	void MakeMoveJ(CRP_CPOS &CPos, double speed, int Move_Tool, int Move_UserNu, int Acc, int Dec, int Smooth, int LineNumber, string* MoveL_str);
	//�������IO�ź�
	void MakeDout(int indexCur, bool state, int lineNum, string* Dout_str);
	//�������IO�ź�
	void MakeMout(int indexCur, bool state, int lineNum, string* Dout_str);
	//�����ı乤�������
	void MakeChangeTool(int ToolNumber, int LineNumber , string& ChangeTool_Str);
	//�����ı��û������
	void MakeChangeUse(int UserNumber, int LineNumber,string& ChangeUser_Str);
	//�ӳ���ĩβ��Ҫ�����,Զ����������ʱ������RET��β
	int programerEnd(int number_Line, string* EndLine);
	// ����setGIָ��
	void MakeSetGi(int index, int indexVal, int lineNum, string * SetGI_str);
	//����GI����ָ��
	void IncGi(int index, int lineNum, string * IncGI_str);
	//����Jumpָ�� 
	void MakeJump(int indexCur, string target, int indexTarget, int lineNum, string * Jump_str);
	//����EndLoopָ�� 
	void loopEnd(int lineNum, string target, string * endLoop_str);
	//����Timeָ�� TIME GINo=2 N9 
	void MakeTime(int GINum,int lineNum, string* time_str);
	//����PAUSEָ�� PAUSE IF GI#(2) == GP#(9) N10
	void MakePause(int GINum, int GPNum,int lineNum, string* pause_str);
	//����PAUSEָ�� PAUSE IF Y#(3) == ON N11
	void MakePause(int YNum, int lineNum,bool state, string* pause_str);
	/*������ָ��  
	  ����:  ���պ�(�����й��պŵ�GI���±�)����һ�������ǳ�������Gi�±ꡢ�ٶ����͡��ٶ�ֵ(û����0)����ѹ���������û����к�
	  speedType   0:����     1���ٷֱ��ٶ�     2��mm/s
	  arcType     0:GI       1: const*/
	void MakeArc(int arcNum,int arcType, int SpeedType,double speed,double voltage, double current,int user,int lineNum, string* arc_str);
	//��ָ�� ����Ҫ���������ѹ����
	void MakeArc(int arcNum, int arcType, int SpeedType, double speed, int user, int lineNum, string* arc_str);
	//����ͣ��ָ�� ARCEND#(2) N14
	void MakeArcEnd(int arcNum, int lineNum, string* arcEnd_str);
	//����ͣ��ָ�� ARCEND#(GI = 2) N15
	void MakeArcEndGI(int GINum, int lineNum, string* arcEnd_str);
	void MakeWaveStart(int id, int linenum, string* waveStart_str);
	void MakeWaveEnd(int linenum, string* waveEnd_str);
	//�����㺸ָ�� SPOT#(1) S#(2) P#(3.0) N16
	void MakeSpot(int spotNum, int fileS, double pressure, int lineNum, string* spot_str);
	//����ע��ָ�� ; last line is dianHan N17
	void MakeNote(string textMsg,int lineNum, string* note_str);
	//�����ֲ��ٶ�ָ�� SPEED VL = 50.0 N18
	void MakeSpeed(double speed, int lineNum, string* speed_str);
	//����ifָ�� IF GI#(0) == GI#(2) 1 N19
	void MakeIF(int GINum1,int GINum2, int lineNum, string* if_str);
	//����elseָ�� ELSE 2 N20
	void MakeELSE(int GINum, int lineNum, string* else_str);
	//����elseifָ�� ELSEIF GI#(1) == GI#(3) 2 N21
	void MakeELSEIF(int GINum1, int GINum2, int lineNum, string* elseif_str);
	//����whileָ�� WHILE GI#(2) == GI#(4) 0 N22
	void MakeWhile(int GINum1, int GINum2,int mark, int lineNum, string* while_str);
	/***************************************************�˶�����******************************************************************/
	//pos[8],�����˶�ֱ������ ֱ������
	int MoveByJob_L(double Pos[], double speed, int Tool, int UserNu, int Acc, int Dec , bool ifSubPro = true);
	int MoveByJob_J(double Pos[], double speed, int Tool, int UserNu, int Acc, int Dec , bool ifSubPro = true);
	//axis[8],�����˶�  �ؽ�����
	int MoveByJob_J_Axis(double Axis[], double speed, int Tool, int UserNu, int Acc, int Dec , bool ifSubPro = true);
	//����˶� ֱ������
	int MoveL_Relative(double PosRelative[], double speed, int Tool, int UserNu, int Acc, int Dec);
	int MoveJ_Relative(double PosRelative[], double speed, int Tool, int UserNu, int Acc, int Dec);
	//����˶� �ؽ�����
	int MoveJ_Relative_Axis(double AxisRelative[], double speed, int Tool, int UserNu, int Acc, int Dec);
	//���������,ֱ������x y z a b c,ifRelative:1-��ԣ�0-�����˶�
	int Single_MoveL(int whichAxis,  double distence ,int ifRelative,double speed, int Tool, int UserNu, int Acc, int Dec);
	int Single_MoveJ(int whichAxis, double distence, int ifRelative, double speed, int Tool, int UserNu, int Acc, int Dec);
	//������˶�,ֱ������a1 a2 a3 a4 a5 a6,ifRelative:1-��ԣ�0-�����˶�
	int Single_MoveJ_Axis(int whichAxis, double distence,int ifRelative ,double speed, int Tool, int UserNu, int Acc, int Dec);

	//����˶�,����Movָ��pos[][8]
	int Move_MultiPos(double** Pos , int Pos_Number, double speed[],int Tool , int UserNu,int LineNumber=1 , bool ifSubPro = false, string FileName= "MultiPos");
	//����˶�,����Movָ��CRP_CPOS[]
	int Move_MultiPos(CRP_CPOS Pos[], int Pos_Number, double speed[], int Tool, int UserNu, int LineNumber=1 ,bool ifSubPro=false, string FileName= "MultiPos");

	int ChangeUserNum(int newUserNum); // �����û���
	int ChangeToolNum(int newToolNum); // ���Ĺ��ߺ�
	/***************************************************��������**********************************************************************/
	//������λ�ŵĳ���
	int CallJob(UINT ProgamerNumber);	
	/*
	param: int nDelayTime -> checkdone��ʼʱ��
	param: showLog -> �Ƿ��ӡ��־������ӡ��־Ч�ʸߺܶ�׼ȷ��Ҳ���ţ���ӡ��־�����ʽ
	*/
	void checkDone(int nDelayTime = 1000,bool showLog = false);
	void checkDoneByPos(int nDelayTime = 1000, bool isSingleMov = false);

	bool isEqual_double(double a, double b);
	bool isEqual_Pos(CRP_APOS a, CRP_APOS b);
	bool isEqual_Pos(CRP_CPOS a, CRP_CPOS b);
	bool isEqual_Pos(double a[10], double b[10]);
	//0:�˶����� 1:�˶���  -1:��ȡʧ��
	int IsRunning();
	//�������
	int CleanAlarm();
	//ʹ��
	int Enable();
	//�µ�
	int Disable();
	//ֹͣ
	int StopProgamer();
	//��λ
	int ResetProgamer();
	//��ɨ���� state�� 1 ���� 0 �ء�
	int scanLaser(int state);
	//���ټ��� state�� 1 ���� 0 �ء�
	int trackLaser(int state);
	//���õ�������
	int SetPos(int GPNumber, double date[]);
	//���õ�������
	int SetPos(int GPNumber,CRP_TU_CPOS &date);
	//���ö������
	int SetPos_Multi(int GPNumber,CRP_TU_CPOS date[],int count);
	//�õ���������
	int GetPos(int GPNumber, double date[]);
	//��õ�ǰ����
	int GetCurrPos(double pos[10]);
	//��ò�ͬ�����µ�����
	int GetAppointToolPos(CRP_TU_CPOS& Cpos,int ToolN,int UserN);
	//��ò�ͬ�����µ�����
	int GetAppointToolPos(double pos[10], int ToolN, int UserN);
	//��õ�ǰ�Ĺؽ�����
	int GetCurrAxisByAngle(double Axis[10]); 
	//��õ�ǰ����������
	int GetCurrAxis(double pulse[10]);
	//��õ�ǰ���ߺ�
	int GetCurrToolNo();
	//��õ�ǰ�û���
	int GetCurrUserNo();
	//ʹ��GI��CallJob
	int XiRobot_CallJob(int numGI);
	//���Ͳ����켣���ݣ��ӽ̵������켣���ꡢ�ٶȡ���ɨ�������׷�����(��ɨΪ��0��׷�ٴ�1)��
	bool SendTeachMoveData(int pointNum, double pos[][10], int pointType[], double speed, int LineScanOrTrack);
	//���õ�ǰ�û���,������ʹ�ã�ֱ����Mov�µĹ��ߺ��û�
	//int SetTool_UserNo(int TooLNumber,int UserNumber);
	
	/***************************************************��ʵ�ֹ���*******************************************************************/
	//���Ӿ���20�㣬�ŵ����������������ɹ�������
	//�޷��Զ������û�����
	/**********************************************************************************************************************/
public:
	TCP_Socket_CRP* Obj_Modbus;
	TCP_Socket_CRP* Obj_FTP_cmd;
	TCP_Socket_CRP* Obj_FTP_date;
	TCP_Socket_CRP* Obj_Track;
	int Number_a;
	char *m_ip;
	int m_prot;
	int m_flag;
	int Modbus_Flag;//modbusTCP��������ʶ��
	char *FTPm_ip;
	int FTPm_prot;
	int FTPm_flag;
	char* FTPname;
	char* FTPpassword;
	vector<uint16_t> *sendBuf;
	uint8_t Mod_Buff[200];
	double data_RP[10];
	double m_CurrPos[10];
	double m_CurrAxis[10];
	double data_WP[10];
	double** ToolArray; //������������
	double** UserArray;//�û���������
	int m_ToolNumber;
	int m_UserNumber;
	
private:
	int Read_retain_word(int addr,int len ,double date[]);
	void static Modbus_Rcv(void *pOwner, const char *rcvData, int nDataLen);
	void static FTP_Rcv(void * pOwner, const char * rcvData, int nDataLen);
	int FTP_land(char* name, char* password);                     //FTP��¼���û�������
	int ModbusTCP(int FunctionNumber, vector<uint16_t> *sendBuf);
	int CROBOTPCtr::wait(int id, UINT timeout);
	map<int, vector<uint8_t>>* Modbus_rcvBuf;
	HANDLE m_mutex;
	
private:
		// x == y
	template <typename T> bool almostEqual(T x, T y);

		// x == y
	template <typename T> bool almostEqual(T x, T y, uint32 precise);

		// x == 0
		template <typename T> bool isZero(T x, uint32 precise);

		// v == 0
		template <typename T> inline bool isZero(T const &v);

		// float
		// v == 0.0f
		template <> inline bool isZero<float>(float const &v);

		// double
		// v == 0.0
		template <> inline bool isZero<double>(double const &v);

};


