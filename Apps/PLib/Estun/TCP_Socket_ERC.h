#pragma once
#ifndef __TCP_SOCKET
#define __TCP_SOCKET

#include <time.h>
#include <map>
#include <vector>
#include <string>
#include <fstream>

#define CAPACITY 30
// #include <vector>
typedef void(*CALL_BACK_ESTUN_SOCKET_RCV_DATA)(void*, const char* ucRcvData, int nDataLen);


class TCP_Socket_ERC
{
public:
	TCP_Socket_ERC();
	~TCP_Socket_ERC();
	//std::vector<std::string> recore_information();
	HANDLE m_mutex;
	//
	void TCP_init(char* m_ip, u_short Prot);
	void SetCallBackRcvData(CALL_BACK_ESTUN_SOCKET_RCV_DATA pFun, void* pFather); //传入父类，和指针函数

	int startSocket();    //启动


	int static TCP_Cyc_Accep(LPVOID pParam);


	CALL_BACK_ESTUN_SOCKET_RCV_DATA m_pfunCallBack;                //这两个就是SetCallBackRcvData用来处理父类的函数
	void* m_pFather;                                        //父类指针

	BOOL isConnected();  //是否连接
	//返回的是连接的客户段数量，输入LParam，输出是0-接受客户端，1-接受到数据，接受的数据存储在recvbuffer。

	bool TCP_Send(SOCKET, CString);
	void TCP_Close(SOCKET Socket);


public:
	//bool ifRecord;
	bool m_connected;
	bool m_iniPort;
	bool ThreadKill;
	CWinThread* Thread_listen, * Threadd_accep;
	int m_clientSocket;
	char  recvbuffer[1000]; //接受数据
	SOCKET DefultSocket;  //客户端socket
	SOCKET ReturnSocket; //连接后返回的socket

	sockaddr_in addr; //服务器IP地址变量
	sockaddr_in ClinerAddr; //客户端的IP地址变量

	CString Record_IP;//IP
	CString Record_Port;//端口


	//vector<SOCKET> Clint;
};

#endif // !__TCP_SOCKET

