#pragma once
#include <time.h>
#include <map>
#include <vector>
#include <string>
#include <fstream>
#define SERVER 0
#define CLIENT 1
#define CAPACITY 30
// #include <vector>

typedef void(*CALL_BACK_SOCKET_RCV_DATA_A)(void *, const char *ucRcvData, int nDataLen);
//

class TCP_Socket_CRP
{
public:
	TCP_Socket_CRP();
	~TCP_Socket_CRP();
	//std::vector<std::string> recore_information();
	HANDLE m_mutex;
	//
	/*************************************************通用socket******************************************************************/
	void TCP_init(char * m_ip, u_short Prot,int flag = SERVER);
	void TCP_Socket_CRP::SetCallBackRcvData(CALL_BACK_SOCKET_RCV_DATA_A pFun, void *pFather); //传入父类，和指针函数
	int TCP_Socket_CRP::startSocket();    //启动
	int static TCP_Socket_CRP::TCP_Cyc_Accep(LPVOID pParam);
	CALL_BACK_SOCKET_RCV_DATA_A m_pfunCallBack;                //这两个就是SetCallBackRcvData用来处理父类的函数
	void *m_pFather;                                        //父类指针
	BOOL TCP_Socket_CRP::isConnected();  //是否连接
	//返回的是连接的客户段数量，输入LParam，输出是0-接受客户端，1-接受到数据，接受的数据存储在recvbuffer。
	bool TCP_Socket_CRP::TCP_Send(SOCKET,CString);
	// 实时追踪功能
	int TCP_Socket_CRP::track(unsigned int myPort);
	/*********************************************ftp函数*****************************************************************/
	
	int user(char name[10]);
	int pass(char* password);
	int biteMode();
	bool FTP_SendFile(const char* file); //FTP上传文件至根目录
	bool FTP_SendFileToAddress(const char * file, const char * address);//FTP上传文件至目标目录
	int FTP_DownloadFile(const char* file); //FTP下载指定文件（在本地新建一个文件）
	int FTP_DownloadFileByCoverLocal(const char * file, const char * localAddress); //FTP下载指定文件（覆盖本地指定文件）

	/***********************************************ModbusTCP函数***************************************************************/
	bool Modbus_send(SOCKET,BYTE *array,int len);//发送16进制

	/***************************************************************************************************************************/
	void TCP_Socket_CRP::TCP_Close(SOCKET Socket);
	

public:
	int m_FTPClient__;
	int m_ModbusTCPClient__;
	int m_flag;
	bool m_connected ;
	bool m_iniPort;
	bool ThreadKill;
	CWinThread *Thread_listen,*Threadd_accep;
	int m_clientSocket ;
	char  recvbuffer[1000]; //接受数据

	SOCKET DefultSocket;  //客户端socket
	SOCKET ClientSocket;     //ftp客户端
	bool FTP_if_load;      //是否登录
	bool FTP_if_connection; //是否连接
	bool FTP_if_passive;    //是否为被动模式
	bool FTP_if_upComplete;  //是否上传完成

	SOCKET ReturnSocket; //连接后返回的socket
	
	sockaddr_in addr; //服务器IP地址变量
	sockaddr_in ClinerAddr; //客户端的IP地址变量

	bool ftpRec;
	char FTPbuff[50];//ftp返回数据
	char*  FTPsendbuff;//ftp发送缓存区
	BYTE cmd_send[256];//存放发送的指令
	//vector<SOCKET> Clint;
};

