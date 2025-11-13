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
	Read_coil_bite_register = 0x01,//读线圈寄存器
	Read_Input_bite_register = 0x02,//读取输入寄存器
	Read_retain_word_register = 0x03,//读保持寄存器
	Read_Input_word_register = 0x04,//读取输入寄存器
	Write_coil_bite_register = 0x05,//写单个线圈寄存器
	Write_retain_word_register = 0x06,//写单个保持寄存器
	Write_Multi_coil_register = 0x0F,//写多个线圈寄存器
	Write_Multi_retain_register = 0x10//写多个保持寄存器
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
}CRP_CPOS;//直角坐标
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
}CRP_APOS;//关节坐标
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
}CRP_TU_CPOS; //带工具号和用户标识的直角坐标
typedef struct
{
	double X;
	double Y;
	double Z;
	double A;
	double B;
	double C;
}CRP_T00L; //工具结构体
typedef struct
{
	double X;
	double Y;
	double Z;
	double A;
	double B;
	double C;
}CRP_USERCOOR; //用户标识的结构体
enum Function_able
{
	Read_YNumber_status = 0x01
};
class CROBOTPCtr
{
public:
	CROBOTPCtr();
	~CROBOTPCtr();
	void initApp(); //统一初始化
	void keepAlive(); //保活
	int initModbusTCP(char * m_ip, u_short Prot, int flag = CLIENT);//0-服务器，1-客户端;
	//prot:21为命令端口，初始化是连接服务器，连接上后会自动调用FTP_Land 登录
	int initFTP_cmd(char * m_ip, u_short Prot, char * name, char * password, int flag = CLIENT);
	int initToolInfo();//修改用户详细信息
	int initUserInfo();//修改用户详细信息
	int initTrackTCP(u_short Prot);
	int initToolUserDate(CRP_T00L Tool[],int ToolCount, CRP_USERCOOR UserCoor[],int UserCount);
	/**************************************ModbusTCP功能接口*************************************************************/
	/*读一个Y口状态:
	通用输出 Y 接口，该接口对应硬件物理接口，为状态变量（ON=1，OFF=0），范围 0-79。*/
	int Read_Y_Number_status(int YNumber);
	//写一个Y口状态
	int Write_Y_Number_status(int YNumber, bool status);
	//读一个M口状态．内部辅助 M 继电器。该继电器为状态变量（ON=1，OFF = 0），范围：0 - 800
	int Read_M_Number_status(int MNumber);
	//写一个M口状态
	int Write_M_Number_status(int MNumber, bool status);
	//读一个X口状态.通用输入 X 接口，该接口对应硬件物理接口，为状态变量（ON = 1，OFF = 0），范围：0 - 111。
	int Read_X_Number_status(int XNumber);
	//读一个GI数状态 全局I变量.范围：0-99，变量值为整数型数据变量，带正负号
	int Read_GI_Number_status(int GINumber);
	//写一个GI数状态
	int Write_GI_Number_status(int GINumber,int date);
	//写一个GP数状态，10个数据．
	//全局 P 变量，该变量值记录机床各关节姿态，坐标等相关位置数据，是多个数据组合。变量号范围：0 - 999，数据号范围为：0 - 11
	int Write_GP_Number_single(int GPNumber,double date[]);
	//写一个GP数状态，10个数据
	int Write_GP_Number_single(int GPNumber, CRP_TU_CPOS &date);
	//写多个GP数
	int Write_GP_Number_Multi(int GPNumber, CRP_TU_CPOS date[],int count);
	//读一个GP数据状态，10个数据
	int Read_GP_Number_status(int GPNumber, double date[]);
	//获取工具详细信息
	int getToolInfo(int toolNumber, double data[]);
	//获取用户详细信息
	int getUserInfo(int userNumber, double data[]);
	//修改用户详细信息
	int setUserInfo(int userNumber, double data[]);
	/***********************************************上位机编写示教器程序组件*************************************************************************/
	template<typename T> bool pose2ora(T const (&pose)[9], T &a, T &b, T &c);
	//矩阵转置
	void MatrixTranspose(double RotationMatrix[3][3]);
	//欧拉角 转 旋转矩阵
	/*返回的结果排列顺序，使用时要转置
	          Nx  Ny  Nz                               Nx  Ox  Ax
			  Ox  Oy  Oz							   Ny  Oy  Ay
			  Ax  Ay  Az       别的接口使用要转置为：  Nz  Oz  Az
	*/
	void EulerAngleToVector(double Angle[3], double RotationMatrix[3][3]);
	//旋转矩阵 转 欧拉角
	/*输入如下形式
		Nx  Ox  Ax
		Ny  Oy  Ay
	    Nz  Oz  Az
	*/
	void RotationToEulerAngle(double RotationMatrix[3][3], double Angle[3]);
	//法兰坐标转到对应工具下的坐标
	void FlangeToTool_Pos(double F_pos[6],int ToolNumber,int UserNumber,double T_pos[6]);
	//工具下的坐标转为法兰下的坐标
	void Tool_PosToFlange(double T_pos[6], int ToolNumber, int UserNumber, double F_pos[6]);
	/*********************************************制作示教器指令*************************************************************/
	//填的是关节坐标Pos[8]
	void MakeMoveJ_Axis(double Axis[], double speed, int Move_Tool, int Move_UserNu, int Acc, int Dec, int Smooth, int LineNumber, string* MoveJ_str);
	//制作MoveJ ,关节坐标
	void MakeMoveJ_Axis(CRP_APOS &Axis, double speed, int Move_Tool, int Move_UserNu, int Acc, int Dec, int Smooth, int LineNumber, string* MoveJ_str);
	//制作使用全局变量的MoveJ
	void MakeMoveJByGP(int GPIndex, double speed, int Move_Tool, int Move_UserNu, int Acc, int Dec, int Smooth, int LineNumber, string* MoveJ_str);
	//制作使用全局变量的MoveL
	void MakeMoveLByGP(int GPIndex, double speed, int Move_Tool, int Move_UserNu, int Acc, int Dec, int Smooth, int LineNumber, string* MoveL_str);
	//制作MoveL指令
	void MakeMoveL(double LPos[],double speed,int Move_Tool,int Move_UserNu,int Acc,int Dec,int Smooth,int LineNumber,string* MoveL_str);
	void MakeMoveL(CRP_CPOS &CPos, double speed, int Move_Tool, int Move_UserNu, int Acc, int Dec, int Smooth, int LineNumber, string* MoveL_str);
	//制作MoveJ指令
	void MakeMoveJ(double LPos[],double speed, int Move_Tool, int Move_UserNu, int Acc, int Dec, int Smooth, int LineNumber, string* MoveL_str);
	void MakeMoveJ(CRP_CPOS &CPos, double speed, int Move_Tool, int Move_UserNu, int Acc, int Dec, int Smooth, int LineNumber, string* MoveL_str);
	//制作输出IO信号
	void MakeDout(int indexCur, bool state, int lineNum, string* Dout_str);
	//制作输出IO信号
	void MakeMout(int indexCur, bool state, int lineNum, string* Dout_str);
	//制作改变工具坐标号
	void MakeChangeTool(int ToolNumber, int LineNumber , string& ChangeTool_Str);
	//制作改变用户坐标号
	void MakeChangeUse(int UserNumber, int LineNumber,string& ChangeUser_Str);
	//子程序末尾需要加这个,远程启动程序时必须有RET结尾
	int programerEnd(int number_Line, string* EndLine);
	// 制作setGI指令
	void MakeSetGi(int index, int indexVal, int lineNum, string * SetGI_str);
	//制作GI自增指令
	void IncGi(int index, int lineNum, string * IncGI_str);
	//制作Jump指令 
	void MakeJump(int indexCur, string target, int indexTarget, int lineNum, string * Jump_str);
	//制作EndLoop指令 
	void loopEnd(int lineNum, string target, string * endLoop_str);
	//制作Time指令 TIME GINo=2 N9 
	void MakeTime(int GINum,int lineNum, string* time_str);
	//制作PAUSE指令 PAUSE IF GI#(2) == GP#(9) N10
	void MakePause(int GINum, int GPNum,int lineNum, string* pause_str);
	//制作PAUSE指令 PAUSE IF Y#(3) == ON N11
	void MakePause(int YNum, int lineNum,bool state, string* pause_str);
	/*制作起弧指令  
	  参数:  工艺号(或者有工艺号的GI的下标)、上一个参数是常数还是Gi下标、速度类型、速度值(没有填0)、电压、电流、用户、行号
	  speedType   0:不填     1：百分比速度     2：mm/s
	  arcType     0:GI       1: const*/
	void MakeArc(int arcNum,int arcType, int SpeedType,double speed,double voltage, double current,int user,int lineNum, string* arc_str);
	//起弧指令 不需要输入输入电压电流
	void MakeArc(int arcNum, int arcType, int SpeedType, double speed, int user, int lineNum, string* arc_str);
	//制作停弧指令 ARCEND#(2) N14
	void MakeArcEnd(int arcNum, int lineNum, string* arcEnd_str);
	//制作停弧指令 ARCEND#(GI = 2) N15
	void MakeArcEndGI(int GINum, int lineNum, string* arcEnd_str);
	void MakeWaveStart(int id, int linenum, string* waveStart_str);
	void MakeWaveEnd(int linenum, string* waveEnd_str);
	//制作点焊指令 SPOT#(1) S#(2) P#(3.0) N16
	void MakeSpot(int spotNum, int fileS, double pressure, int lineNum, string* spot_str);
	//制作注释指令 ; last line is dianHan N17
	void MakeNote(string textMsg,int lineNum, string* note_str);
	//制作局部速度指令 SPEED VL = 50.0 N18
	void MakeSpeed(double speed, int lineNum, string* speed_str);
	//制作if指令 IF GI#(0) == GI#(2) 1 N19
	void MakeIF(int GINum1,int GINum2, int lineNum, string* if_str);
	//制作else指令 ELSE 2 N20
	void MakeELSE(int GINum, int lineNum, string* else_str);
	//制作elseif指令 ELSEIF GI#(1) == GI#(3) 2 N21
	void MakeELSEIF(int GINum1, int GINum2, int lineNum, string* elseif_str);
	//制作while指令 WHILE GI#(2) == GI#(4) 0 N22
	void MakeWhile(int GINum1, int GINum2,int mark, int lineNum, string* while_str);
	/***************************************************运动功能******************************************************************/
	//pos[8],绝对运动直角坐标 直角坐标
	int MoveByJob_L(double Pos[], double speed, int Tool, int UserNu, int Acc, int Dec , bool ifSubPro = true);
	int MoveByJob_J(double Pos[], double speed, int Tool, int UserNu, int Acc, int Dec , bool ifSubPro = true);
	//axis[8],绝对运动  关节坐标
	int MoveByJob_J_Axis(double Axis[], double speed, int Tool, int UserNu, int Acc, int Dec , bool ifSubPro = true);
	//相对运动 直角坐标
	int MoveL_Relative(double PosRelative[], double speed, int Tool, int UserNu, int Acc, int Dec);
	int MoveJ_Relative(double PosRelative[], double speed, int Tool, int UserNu, int Acc, int Dec);
	//相对运动 关节坐标
	int MoveJ_Relative_Axis(double AxisRelative[], double speed, int Tool, int UserNu, int Acc, int Dec);
	//单轴的运行,直角坐标x y z a b c,ifRelative:1-相对，0-绝对运动
	int Single_MoveL(int whichAxis,  double distence ,int ifRelative,double speed, int Tool, int UserNu, int Acc, int Dec);
	int Single_MoveJ(int whichAxis, double distence, int ifRelative, double speed, int Tool, int UserNu, int Acc, int Dec);
	//单轴的运动,直角坐标a1 a2 a3 a4 a5 a6,ifRelative:1-相对，0-绝对运动
	int Single_MoveJ_Axis(int whichAxis, double distence,int ifRelative ,double speed, int Tool, int UserNu, int Acc, int Dec);

	//多点运动,大量Mov指令pos[][8]
	int Move_MultiPos(double** Pos , int Pos_Number, double speed[],int Tool , int UserNu,int LineNumber=1 , bool ifSubPro = false, string FileName= "MultiPos");
	//多点运动,大量Mov指令CRP_CPOS[]
	int Move_MultiPos(CRP_CPOS Pos[], int Pos_Number, double speed[], int Tool, int UserNu, int LineNumber=1 ,bool ifSubPro=false, string FileName= "MultiPos");

	int ChangeUserNum(int newUserNum); // 更改用户号
	int ChangeToolNum(int newToolNum); // 更改工具号
	/***************************************************其他功能**********************************************************************/
	//启动工位号的程序
	int CallJob(UINT ProgamerNumber);	
	/*
	param: int nDelayTime -> checkdone开始时间
	param: showLog -> 是否打印日志，不打印日志效率高很多准确性也更号；打印日志方便调式
	*/
	void checkDone(int nDelayTime = 1000,bool showLog = false);
	void checkDoneByPos(int nDelayTime = 1000, bool isSingleMov = false);

	bool isEqual_double(double a, double b);
	bool isEqual_Pos(CRP_APOS a, CRP_APOS b);
	bool isEqual_Pos(CRP_CPOS a, CRP_CPOS b);
	bool isEqual_Pos(double a[10], double b[10]);
	//0:运动结束 1:运动中  -1:读取失败
	int IsRunning();
	//清除报警
	int CleanAlarm();
	//使能
	int Enable();
	//下电
	int Disable();
	//停止
	int StopProgamer();
	//复位
	int ResetProgamer();
	//线扫激光 state： 1 开， 0 关。
	int scanLaser(int state);
	//跟踪激光 state： 1 开， 0 关。
	int trackLaser(int state);
	//设置单个坐标
	int SetPos(int GPNumber, double date[]);
	//设置单个坐标
	int SetPos(int GPNumber,CRP_TU_CPOS &date);
	//设置多个坐标
	int SetPos_Multi(int GPNumber,CRP_TU_CPOS date[],int count);
	//得到单个坐标
	int GetPos(int GPNumber, double date[]);
	//获得当前坐标
	int GetCurrPos(double pos[10]);
	//获得不同工具下的坐标
	int GetAppointToolPos(CRP_TU_CPOS& Cpos,int ToolN,int UserN);
	//获得不同工具下的坐标
	int GetAppointToolPos(double pos[10], int ToolN, int UserN);
	//获得当前的关节坐标
	int GetCurrAxisByAngle(double Axis[10]); 
	//获得当前的脉冲坐标
	int GetCurrAxis(double pulse[10]);
	//获得当前工具号
	int GetCurrToolNo();
	//获得当前用户号
	int GetCurrUserNo();
	//使用GI来CallJob
	int XiRobot_CallJob(int numGI);
	//发送测量轨迹数据（视教点数、轨迹坐标、速度、线扫相机还是追踪相机(线扫为传0，追踪传1)）
	bool SendTeachMoveData(int pointNum, double pos[][10], int pointType[], double speed, int LineScanOrTrack);
	//设置当前用户号,不建议使用，直接用Mov下的工具和用户
	//int SetTool_UserNo(int TooLNumber,int UserNumber);
	
	/***************************************************可实现功能*******************************************************************/
	//用视觉标20点，放到程序里，点击程序生成工具坐标
	//无法自动设置用户坐标
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
	int Modbus_Flag;//modbusTCP里的事务标识符
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
	double** ToolArray; //工具坐标数据
	double** UserArray;//用户坐标数据
	int m_ToolNumber;
	int m_UserNumber;
	
private:
	int Read_retain_word(int addr,int len ,double date[]);
	void static Modbus_Rcv(void *pOwner, const char *rcvData, int nDataLen);
	void static FTP_Rcv(void * pOwner, const char * rcvData, int nDataLen);
	int FTP_land(char* name, char* password);                     //FTP登录，用户和密码
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


