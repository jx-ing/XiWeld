#include "stdafx.h"
#include "TCP_Socket_ERC.h"
#include<Ws2tcpip.h>

TCP_Socket_ERC::TCP_Socket_ERC()
{
	m_connected = false;
	m_clientSocket = 0;
	ThreadKill = false;
	m_iniPort = false;
}
TCP_Socket_ERC::~TCP_Socket_ERC()
{
	ThreadKill = true;
	m_iniPort = false;
}
//第一，第二个参数是用来 使用异步套接的
void TCP_Socket_ERC::TCP_init(char* m_ip,u_short Prot)
{
	if (m_iniPort)return;
	//1.初始化网络套接字库。勾选了，已经初始化了。

	//2.创建套接字
	DefultSocket = socket(AF_INET, SOCK_STREAM, 0);
	//3.处理地址---网络字节顺序
	addr.sin_family = AF_INET;
	addr.sin_port = htons(Prot);
	//addr.sin_addr.S_un.S_addr = (m_ip[0] == '0') ? htonl(INADDR_ANY) : inet_addr(m_ip);
	inet_pton(AF_INET, m_ip, & addr.sin_addr.S_un.S_addr);
	//addr.sin_addr.S_un.S_addr = inet_addr(m_ip);
	//addr.sin_addr.S_un.S_addr = INADDR_ANY;
	//4.绑定端口和ip
	::bind(DefultSocket, (sockaddr *)&addr, sizeof(addr));
	listen(DefultSocket, 5);
	m_iniPort = true;
	//服务器IP
	Record_IP = m_ip;
	//设置套接字为异步套接
	//WSAAsyncSelect(DefultSocket, m_hWnd, WM_SOCKET, FD_ACCEPT | FD_READ);
	//把聊天的TEXT设置为不可修改
//	m_text.EnableWindow(false);
	//修改服务器状态
//	m_text.SetWindowTextA("服务器监听已经启动");

}
void TCP_Socket_ERC::SetCallBackRcvData(CALL_BACK_ESTUN_SOCKET_RCV_DATA pFun, void *pFather)
{
	m_pFather = pFather;     //别的类使用定义的Socket对象，父类
	m_pfunCallBack = pFun;   //函数指针，指向函数 void(*CALL_BACK_SOCKET_RCV_DATA)(void *, const char *ucRcvData, int nDataLen)
}


int TCP_Socket_ERC::startSocket()
{
	Threadd_accep= AfxBeginThread((AFX_THREADPROC)TCP_Cyc_Accep, this);
	return 0;
}

BOOL TCP_Socket_ERC::isConnected()
{
	return m_connected;
}
int TCP_Socket_ERC::TCP_Cyc_Accep(LPVOID pParam)
{
	TCP_Socket_ERC*app = (TCP_Socket_ERC*)pParam; //获得主线程指针，方便调用线程里面的数据
										   // CSingleLock SingleLock(&(app->mutex));     //锁
	CString str = "", str1;

	int listen_infor;

	int *information = new int;
	SOCKET socket_this;
	//客户端IP,端口号
	CString Cline_IP, Cline_Port;
	UCHAR Ip_1,Ip_2,Ip_3,Ip_4;
	//char CIp_1, CIp_2, CIp_3, CIp_4;
	int I_Ip_1, I_Ip_2, I_Ip_3, I_Ip_4;
	while (1)
	{
		if (app->ThreadKill)
		{
			TRACE("杀死Auto_Edit线程_进入");
			delete information;

			DWORD dwExitCode;                              //指定线程退出码
			GetExitCodeThread(app->Threadd_accep, &dwExitCode);  //获取线程1的退出码
			AfxEndThread(dwExitCode, TRUE);                //退出线程			
		}
		else
		{
			int length = sizeof(app->ClinerAddr);
			//接收客户端的请求
			if (!app->m_connected)
			{
				socket_this = accept(app->DefultSocket, (sockaddr*)&app->ClinerAddr, &length);//接受客户端
				if (socket_this > 0 )
				{
					app->ReturnSocket = socket_this;
					app->m_connected = true;
					 Ip_1 = app->ClinerAddr.sin_addr.S_un.S_un_b.s_b1;
					 Ip_2 = app->ClinerAddr.sin_addr.S_un.S_un_b.s_b2;
					 Ip_3 = app->ClinerAddr.sin_addr.S_un.S_un_b.s_b3;
					 Ip_4 = app->ClinerAddr.sin_addr.S_un.S_un_b.s_b4;
					 
					 I_Ip_1 = (int)Ip_1;
					 I_Ip_2 = (int)Ip_2;
					 I_Ip_3 = (int)Ip_3;
					 I_Ip_4 = (int)Ip_4;

					
				}
			}
				memset(app->recvbuffer, '\0', sizeof(char) * 1000);
				CString num = "";
				listen_infor = recv(app->ReturnSocket, app->recvbuffer, 1000, 0); //接受信息

				if (listen_infor > 0)
				{
					//if (app->ifRecord)
					{
						std::ofstream RecordFile("./EstunComRecord.txt", std::ios::app); //用记事本的方式记录下来
																					   //time_t t = time(NULL);
						SYSTEMTIME st = { 0 };
						GetLocalTime(&st);
						//RecordFile<<ctime(&t) << "	客户端" << app->ReturnSocket << ":" << app->listen_information.recvbuffer << "\n";
						RecordFile << st.wMinute << ":" << st.wSecond << "-" << st.wMilliseconds << ":" << "客户端" << I_Ip_1 <<"."<< I_Ip_2 <<"."<< I_Ip_3 <<"."<< I_Ip_4 << ":" << app->recvbuffer << "\n";
						RecordFile.close();
					}
					app->m_pfunCallBack((void*)app->m_pFather, app->recvbuffer, listen_infor); // m_pfunCallBack是个指针，现在是 FanucRobotCtrl::Rcv,处理数据

				}
				else if ((listen_infor == -1))
				{
					app->m_connected = false;
				}
			
		}
	}

}




bool TCP_Socket_ERC::TCP_Send(SOCKET Socket, CString str)
{
	bool ifsucce;
	//m_sendtext.GetWindowTextA(str);
	if (str == "")//不能发送空消息
	{
		return false;
	}
	else
	{
		//发消息给客户端
		//成功发送
		std::ofstream RecordFile("./EstunComRecord.txt", std::ios::app); //用记事本的方式记录下来
		           										   //time_t t = time(NULL);
		SYSTEMTIME st = { 0 };
		GetLocalTime(&st);
		if (send(Socket, str.GetBuffer(1), str.GetLength(), 0) != SOCKET_ERROR)
		{
			UCHAR AD = addr.sin_addr.S_un.S_un_b.s_b1;
			RecordFile << st.wYear<<"/"<<st.wMonth<<"/"<<st.wDay<< "  "<<st.wHour<<":"<<st.wMinute<<":" << st.wSecond << "-" << st.wMilliseconds << ":" << "服务器 " << Record_IP << ":" << str << "\n";
			ifsucce = true;
		}
		else//发送失败
		{
			RecordFile << st.wSecond << "-" << st.wMilliseconds << ":" << "服务器 " << Record_IP << ":" << str << " :客户端未连接，失败" << "\n";
			ifsucce = false;
		}
		RecordFile.close();
	}
	return ifsucce;
}
void TCP_Socket_ERC::TCP_Close(SOCKET Socket)
{
	if (Socket > 0) {
		closesocket(Socket);
		Socket = 0;
	}
	m_connected = false;
}