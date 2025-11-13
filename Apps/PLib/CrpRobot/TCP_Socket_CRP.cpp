#include "stdafx.h"
#include "TCP_Socket_CRP.h"
#include<Ws2tcpip.h>
#include<io.h>
#include<iostream>
#include <sstream>
using namespace std;

TCP_Socket_CRP::TCP_Socket_CRP()
{
	m_connected = false;
	m_clientSocket = 0;
	ThreadKill = false;
	m_iniPort = false;
	FTP_if_load = false;
	FTP_if_connection = false;
	FTP_if_passive = false;
	FTP_if_upComplete = false;
	m_FTPClient__ = 0;
	m_ModbusTCPClient__ = 0;
	ClientSocket = 0;
	
}


TCP_Socket_CRP::~TCP_Socket_CRP()
{
	ThreadKill = true;
	m_iniPort = false;
}
//第一，第二个参数是用来 使用异步套接的
void TCP_Socket_CRP::TCP_init(char* m_ip,u_short Prot,int flag)
{
	if (m_iniPort)return;
	//1.初始化网络套接字库。勾选了，已经初始化了。
	m_flag = flag;
	//2.创建套接字
	DefultSocket = socket(AF_INET, SOCK_STREAM, 0);
	//3.处理地址---网络字节顺序
	addr.sin_family = AF_INET;
	addr.sin_port = htons(Prot);
	//inet_pton(AF_INET, m_ip, & addr.sin_addr.S_un.S_addr);
	addr.sin_addr.S_un.S_addr = inet_addr(m_ip);
	//addr.sin_addr.S_un.S_addr = INADDR_ANY;
	if (m_flag == SERVER)
	{
		//4.绑定端口和ip
		::bind(DefultSocket, (sockaddr *)&addr, sizeof(addr));
		listen(DefultSocket, 5);
		m_iniPort = true;
	}
	//设置套接字为异步套接
	//WSAAsyncSelect(DefultSocket, m_hWnd, WM_SOCKET, FD_ACCEPT | FD_READ);
	//把聊天的TEXT设置为不可修改
//	m_text.EnableWindow(false);
	//修改服务器状态
//	m_text.SetWindowTextA("服务器监听已经启动");

}

void TCP_Socket_CRP::SetCallBackRcvData(CALL_BACK_SOCKET_RCV_DATA_A pFun, void *pFather)
{
	m_pFather = pFather;     //别的类使用定义的Socket对象，父类
	m_pfunCallBack = pFun;   //函数指针，指向函数 void(*CALL_BACK_SOCKET_RCV_DATA_A)(void *, const char *ucRcvData, int nDataLen)
}

int TCP_Socket_CRP::startSocket()
{
	Threadd_accep= AfxBeginThread((AFX_THREADPROC)TCP_Cyc_Accep, this);
	return 0;
}

BOOL TCP_Socket_CRP::isConnected()
{
	return m_connected;
}

int TCP_Socket_CRP::TCP_Cyc_Accep(LPVOID pParam)
{
	TCP_Socket_CRP*app = (TCP_Socket_CRP*)pParam; //获得主线程指针，方便调用线程里面的数据
										   // CSingleLock SingleLock(&(app->mutex));     //锁
	CString str = "", str1;

	int listen_infor;

	int clineErr=0;

	int *information = new int;
	SOCKET socket_this=0;
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
				if (app->m_flag == SERVER)
				{
					socket_this = accept(app->DefultSocket, (sockaddr*)&app->ClinerAddr, &length);//接受客户端
				}
				if (app->m_flag == CLIENT)
				{
					app->ClientSocket = app->DefultSocket;
					if (app->m_FTPClient__ == 1)
					{
						if (clineErr == -1)//客户端断开服务器，客户端自动重新连接
						{
							app->ClientSocket = socket(AF_INET, SOCK_STREAM, 0);
							app->FTP_if_load = false;
							app->FTP_if_connection = false;
						}
					}
					else if (app->m_ModbusTCPClient__ == 1)
					{
						if (clineErr == -1)//客户端断开服务器，客户端自动重新连接
						{
							app->ClientSocket = socket(AF_INET, SOCK_STREAM, 0);
							//app->FTP_if_load = false;
							//app->FTP_if_connection = false;
						}
					}
					else
					{
						if (clineErr == -1)//客户端断开服务器，客户端自动重新连接
						{
							app->ClientSocket = socket(AF_INET, SOCK_STREAM, 0);
						}
					}
					clineErr = connect(app->ClientSocket, (sockaddr*)(&app->addr),sizeof(sockaddr_in));//连接服务器
				}
				if ((socket_this > 0)&&(app->m_flag == SERVER) )
				{
					app->ReturnSocket = socket_this;
					app->m_connected = true;
				}
				if ((clineErr == 0)&& (app->m_flag == CLIENT))
				{
					app->ReturnSocket = app->ClientSocket;
					app->m_connected = true;
				}
			}
				memset(app->recvbuffer, '\0', sizeof(char) * 1000);
				listen_infor = 0;
				CString num = "";
				//if (!app->ftpRec)
				
				listen_infor = recv(app->ReturnSocket, app->recvbuffer, 1000, 0); //接受信息

				if (listen_infor > 0)
				{
					if (app->ftpRec)//接收ftp
					{
						for (int i = 0; i < 50; i++)
						{
							app->FTPbuff[i] = app->recvbuffer[i];
						}
					}
					std::ofstream RecordFile("EstunComRecord.txt", std::ios::app); //用记事本的方式记录下来
																				   //time_t t = time(NULL);
					SYSTEMTIME st = { 0 };
					GetLocalTime(&st);
					//RecordFile<<ctime(&t) << "	客户端" << app->ReturnSocket << ":" << app->listen_information.recvbuffer << "\n";
					RecordFile << st.wSecond << "-" << st.wMilliseconds << ":" << "客户端" << app->ReturnSocket << ":" << app->recvbuffer << "\n";
					RecordFile.close();
					app->m_pfunCallBack((void*)app->m_pFather, app->recvbuffer, listen_infor); // m_pfunCallBack是个指针，现在是 FanucRobotCtrl::Rcv,处理数据

				}
				else if ((listen_infor == -1)||(listen_infor==0))
				{
					app->m_connected = false;
				}
			
		}
	}

}

/*
TCP_Send函数是对send函数的封装，主要完成了：
       1. 发送send请求
	   2. 如果发送send请求失败，就将失败信息写在日志里（CROBOTPComRecord.txt）
*/
int TCP_Socket_CRP::track(unsigned int serverPort)
{
	memset(recvbuffer, '\0', sizeof(recvbuffer));
	if (ClientSocket <= 0) {
		WSADATA wsadata;
		if (0 != WSAStartup(MAKEWORD(2, 2), &wsadata)) {
			return -1;
		}
		DefultSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
		if (DefultSocket == -1)
		{
			return -1;
		}

		// 2. 给socket绑定ip地址和端口号
		addr.sin_family = AF_INET;
		addr.sin_addr.s_addr = htonl(INADDR_ANY);	//INADDR_ANY转换过来就是0.0.0.0，泛指本机的意思，也就是表示本机的所有IP
		addr.sin_port = htons(serverPort);
		if (-1 == ::bind(DefultSocket, (struct sockaddr*)&addr, sizeof(addr))) {
			WSACleanup();
			return -1;
		}

		//3. 设置套接字为监听状态
		if (listen(DefultSocket, SOMAXCONN) < 0) // 设置监听状态失败
		{
			WSACleanup();
		}

		//4. 请求到来后，接受连接请求
		int len = sizeof(SOCKADDR);
		ClientSocket = accept(DefultSocket, (SOCKADDR *)&ClinerAddr, &len);
		if (ClientSocket < 0)
		{
			WSACleanup();
			return -1;
		}

		ofstream RecordFile; //用记事本的方式记录下来
		RecordFile.open("./file/trackLog.txt");
		RecordFile << "Start" << endl;

		//5. 用返回的套接字和客户端进行通信
		//send(cli_fd, "B\n", 9, 0);
		BYTE fffb[8] = { 0xFF,0xFB,0x06,0x00,'>','0','0','\r' };
		string strRecv;
		while (1)
		{
			memset(cmd_send, '\0', sizeof(BYTE) * 256);
			int  recv_len = recv(ClientSocket, recvbuffer, sizeof(recvbuffer), 0);
			if (recv_len < 0)
			{
				break;// 接受失败
			}
			strRecv = recvbuffer;
			if (strRecv == "JH+LON\r" || strRecv == "JH+SM00\r") {
				RecordFile << "Client:" << strRecv;
				send(ClientSocket, (char*)fffb, 8, 0);
				RecordFile << "Server:" << "FFFB0600>00\r";
			}
			else if (strRecv == "JH+LOFF\r") {
				RecordFile << "Client:" << strRecv;
				send(ClientSocket, (char*)fffb, 8, 0);
				RecordFile << "Server:" << "FFFB0600>00\r";
				closesocket(DefultSocket);
				closesocket(ClientSocket);
				WSACleanup();// 释放DLL资源
				break;
			}
			else if (strRecv == "JH+TON\r") {
				RecordFile << "Client:" << strRecv;
				send(ClientSocket, (char*)fffb, 8, 0);
				RecordFile << "Server:" << "FFFB0600>00\r";
				while (1) { //两个条件: 1. 在trake时间片内    2. 待发点位还没发完。

					memset(recvbuffer, '\0', sizeof(char) * 1000);
					if (recv(ClientSocket, recvbuffer, sizeof(recvbuffer), 0) < 0)
					{
						break;// 接受失败
					}
					string trackRecv = recvbuffer;
					if (trackRecv.substr(0, 7) == "JH+RPOS") {
						RecordFile << "Client:" << trackRecv;
						send(ClientSocket, (char*)fffb, 8, 0);
						RecordFile << "Server:" << "FFFB0600>00\r";
						memset(recvbuffer, '\0', sizeof(char) * 1000);
						continue;
					}
					else if (trackRecv == "JH+SPOS\r") {
						cmd_send[0] = 0xFF;
						cmd_send[1] = 0xFA;
						cmd_send[2] = 0x30;//39  30
						cmd_send[3] = 0x00;
						string str1 = "X>+29.0591\rY>-1376.821\rZ>+159.831\rS>C00000M00\r";
						cmd_send[2] = str1.size() + 2; //报文长度自适应
						for (int i = 0; i < str1.length(); i++) {
							cmd_send[i + 4] = str1[i];
						}
						CString str(cmd_send);
						str.Format("%02X%02X%02X%02X", cmd_send[0], cmd_send[1], cmd_send[2], cmd_send[3]);
						for (int i = 0; i < str1.size(); i++) {
							str.Format("%s%c", str, cmd_send[i + 4]);
						}
						RecordFile << "Client:" << trackRecv;
						send(ClientSocket, (char*)cmd_send, str1.size() + 10, 0);
						RecordFile << "Server:" << str;
					}

					else  if (trackRecv == "JH+TOF\r") {
						RecordFile << "Client:" << strRecv;
						send(ClientSocket, (char*)fffb, 8, 0); //trackRecv == "JH+TOF\r"	
						RecordFile << "Server:" << "FFFB0600>00\r";
						break;
					}
				}
			}
			else {
				RecordFile << "Client:" << strRecv;
			}
			memset(recvbuffer, '\0', sizeof(char) * 1000);
		}
		RecordFile.close();
	}

	return 0;
}

bool TCP_Socket_CRP::TCP_Send(SOCKET Socket, CString str)
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
		std::ofstream RecordFile("CROBOTPComRecord.txt", std::ios::app); //用记事本的方式记录下来
		           										                 //time_t t = time(NULL);
		SYSTEMTIME st = { 0 };
		GetLocalTime(&st);
		if (send(Socket, str.GetBuffer(1), str.GetLength(), 0) != SOCKET_ERROR) 
		{
			RecordFile << st.wYear<<"/"<<st.wMonth<<"/"<<st.wDay<< "  "<<st.wHour<<":"<<st.wMinute<<":" << st.wSecond << "-" << st.wMilliseconds << ":" << "服务器 "  << ":" << str << "\n";
			ifsucce = true;
		}
		else//发送失败
		{
			RecordFile << st.wSecond << "-" << st.wMilliseconds << ":" << "服务器 " << ":" << "发送失败" << "\n";
			ifsucce = false;
		}
		RecordFile.close();
	}
	return ifsucce;
}

int TCP_Socket_CRP::user(char  name[10])
{
	ftpRec = true;
	char operation[10];
	char order[30] = "\0";
	strcpy_s(operation,"user");
	strcat_s(order, operation), strcat_s(order, " "), strcat_s(order,name), strcat_s(order, "\r\n");
	TCP_Send(ReturnSocket, order);
	//recv(ReturnSocket, FTPbuff, sizeof(FTPbuff), 0); //接受信息
	ftpRec = false;
	return 0;
}

int TCP_Socket_CRP::pass(char * password)
{
	ftpRec = true;
	char operation[10];
	char order[30] = "\0";
	strcpy_s(operation, "pass");
	strcat_s(order, operation), strcat_s(order, " "), strcat_s(order, password), strcat_s(order, "\r\n");
	TCP_Send(ReturnSocket, order);
	//recv(ReturnSocket, FTPbuff, sizeof(FTPbuff), 0); //接受信息
	ftpRec = false;
	if (strcmp(FTPbuff, "wrong") == 0)
	{
		return -1;
	}
	return 0;
}

int TCP_Socket_CRP::biteMode()
{
	ftpRec = true;
	memset(FTPbuff, '\0', sizeof(FTPbuff));
	bool return_sult=TCP_Send(ReturnSocket, "TYPE I\r\n");
	if (!return_sult) return -1;
	while (1)  //得到返回数据
	{
		Sleep(1);
		if (*FTPbuff != '\0')
		{
			break;
		}
	}
	ftpRec = false;
	return 0;
}
//ftp上传文件至根目录(使用前先登录)
bool TCP_Socket_CRP::FTP_SendFile(const char * file)
{
	/************************************进入被动模式，获取数据端口********************************************/
	int a, b, c, d;//192.168.2.1，ip
	int pa, pb,DateProt;//端口
	char ipaddr[20];
	memset(ipaddr,'\0',sizeof(ipaddr));
	memset(FTPbuff,'\0',sizeof(FTPbuff));
	ftpRec = true;
	TCP_Send(ReturnSocket, "PASV\r\n");//进入被动模式，获取数据端口
	long long start;
	start = XI_clock();
	while (1)  //得到返回数据
	{
		Sleep(1);
		if (strncmp(FTPbuff, "227", 3) == 0)//PASV模式
		{
			break;
		}
		else if (*FTPbuff != '\0')
		{
			return false;
		}
		if ((start - XI_clock()) >= 200)
		{
			return false;
		}
	}

	ftpRec = false;
	char* getPort = strchr(FTPbuff,'(');//检索出（）里的数据
	sscanf(getPort,"(%d,%d,%d,%d,%d,%d)",&a,&b,&c,&d,&pa,&pb);//提取ip
	sprintf(ipaddr,"%d.%d.%d.%d",a,b,c,d);//得到ip
	DateProt = pa * 256 + pb;//端口
	/************************************连接被动模式给的数据端口*****************************************/
	SOCKET DateSocket = socket(AF_INET, SOCK_STREAM, 0);
	//3.处理地址---网络字节顺序
	sockaddr_in DateAddr;
	DateAddr.sin_family = AF_INET;
	DateAddr.sin_port = htons(DateProt);
	//inet_pton(AF_INET, m_ip, & addr.sin_addr.S_un.S_addr);
	DateAddr.sin_addr.S_un.S_addr = inet_addr(ipaddr);
	int connetErr = connect(DateSocket, (sockaddr*)&DateAddr, sizeof(DateAddr));
	if (connetErr != 0) //连接不成功
	{
		closesocket(DateSocket);
		return false;
	}
	/******************************************准备*********************************************************/
	char  cmd[30];
	char buff[100];
	memset(buff,'\0',sizeof(buff));
	memset(cmd,'\0',sizeof(cmd));
	sprintf(cmd,"STOR %s\r\n", file);
	memset(FTPbuff,'\0',sizeof(FTPbuff));
	ftpRec = true;
	TCP_Send(ReturnSocket, cmd);//
	//int len = recv(DateSocket, buff, 100, 0); //接受信息
	while (1)  //得到返回数据
	{
		Sleep(3);
		if (strncmp(FTPbuff, "150", 3) == 0)//PASV模式
		{
			break;
		}
		else if (*FTPbuff != '\0')
		{
			return false;
		}
	}
	ftpRec = false;
	/*****************************准备完成，把文件写入输入缓冲区，等待传输*********************************/
	int handle = open(file,0x0100);
	long length = filelength(handle)+1;
	close(handle);
	FILE * fd1;
	errno_t err	= fopen_s(&fd1,file, "rb");
	FTPsendbuff = new char[length];
	if (err==0)
	{
		memset(FTPsendbuff, '\0', length);
		while (1)
		{
			int len = fread(FTPsendbuff, 1, length, fd1);//一次往FTP的发送给缓冲区放1个字符，直到把fd1文件全放进去
			ftpRec = true;
			if (TCP_Send(DateSocket, FTPsendbuff) == false)
			{
				delete FTPsendbuff;
				return false;
			}
			if (len <= length -1)
			{
				break;
			}
		}
		delete FTPsendbuff;
	}
	else
	{
		return false;
	}
	fclose(fd1);
	ftpRec = true;
	memset(FTPbuff, '\0', sizeof(FTPbuff));
	closesocket(DateSocket);
	while (1)  //得到返回数据
	{
		Sleep(1);
		if (strncmp(FTPbuff, "226", 3) == 0)//上传完成
		{
			break;
		}
		else if (*FTPbuff != '\0')
		{
			return false;
		}
	}
	ftpRec = false;
	return true;
}
//ftp上传文件至目标目录
bool TCP_Socket_CRP::FTP_SendFileToAddress(const char * file,const char * address)
{
	/************************************进入被动模式，获取数据端口********************************************/
	int a, b, c, d;//192.168.2.1，ip
	int pa, pb, DateProt;//端口
	char ipaddr[20];
	memset(ipaddr, '\0', sizeof(ipaddr));
	memset(FTPbuff, '\0', sizeof(FTPbuff));
	ftpRec = true;
	TCP_Send(ReturnSocket, "PASV\r\n");//进入被动模式，获取数据端口
	long long start;
	start = XI_clock();
	while (1)  //得到返回数据
	{
		Sleep(1);
		if (strncmp(FTPbuff, "227", 3) == 0)//PASV模式
		{
			break;
		}
		else if (*FTPbuff != '\0')
		{
			return false;
		}
		if ((start - XI_clock()) >= 200)
		{
			return false;
		}
	}

	ftpRec = false;
	char* getPort = strchr(FTPbuff, '(');//检索出（）里的数据
	sscanf(getPort, "(%d,%d,%d,%d,%d,%d)", &a, &b, &c, &d, &pa, &pb);//提取ip
	sprintf(ipaddr, "%d.%d.%d.%d", a, b, c, d);//得到ip
	DateProt = pa * 256 + pb;//端口
	/************************************连接被动模式给的数据端口*****************************************/
	SOCKET DateSocket = socket(AF_INET, SOCK_STREAM, 0);
	//3.处理地址---网络字节顺序
	sockaddr_in DateAddr;
	DateAddr.sin_family = AF_INET;
	DateAddr.sin_port = htons(DateProt);
	//inet_pton(AF_INET, m_ip, & addr.sin_addr.S_un.S_addr);
	DateAddr.sin_addr.S_un.S_addr = inet_addr(ipaddr);
	int connetErr = connect(DateSocket, (sockaddr*)&DateAddr, sizeof(DateAddr));
	if (connetErr != 0) //连接不成功
	{
		closesocket(DateSocket);
		return false;
	}
	/******************************************准备*********************************************************/
	string t;
	istringstream in(file);
	vector<string> info;
	while (getline(in, t, '/')) {	//这里单引号要注意
		info.push_back(t);
	}
	int index = info.size() - 1;
	string fullFile = "STOR ";
	fullFile += address + info[index] + "\r\n";

	char buff[100];
	memset(buff, '\0', sizeof(buff));
	memset(FTPbuff, '\0', sizeof(FTPbuff));
	ftpRec = true;
	TCP_Send(ReturnSocket, fullFile.c_str());//
	//int len = recv(DateSocket, buff, 100, 0); //接受信息
	while (1)  //得到返回数据
	{
		Sleep(3);
		if (strncmp(FTPbuff, "150", 3) == 0)//PASV模式
		{
			break;
		}
		else if (*FTPbuff != '\0')
		{
			return false;
		}
	}
	ftpRec = false;
	/*****************************准备完成，把文件写入输入缓冲区，等待传输*********************************/
	int handle = open(file, 0x0100);
	long length = filelength(handle) + 1;
	close(handle);
	FILE * fd1;
	errno_t err = fopen_s(&fd1, file, "rb");
	FTPsendbuff = new char[length];
	if (err == 0)
	{
		memset(FTPsendbuff, '\0', length);
		while (1)
		{
			int len = fread(FTPsendbuff, 1, length, fd1);//一次往FTP的发送给缓冲区放1个字符，直到把fd1文件全放进去
			ftpRec = true;
			if (TCP_Send(DateSocket, FTPsendbuff) == false)
			{
				delete FTPsendbuff;
				return false;
			}
			if (len <= length - 1)
			{
				break;
			}
		}
		delete FTPsendbuff;
	}
	else
	{
		return false;
	}
	fclose(fd1);
	ftpRec = true;
	memset(FTPbuff, '\0', sizeof(FTPbuff));
	closesocket(DateSocket);
	while (1)  //得到返回数据
	{
		Sleep(1);
		if (strncmp(FTPbuff, "226", 3) == 0)//上传完成
		{
			break;
		}
		else if (*FTPbuff != '\0')
		{
			return false;
		}
	}
	ftpRec = false;
	return true;
}
//通过ftp从服务器下载文件到本地。
int TCP_Socket_CRP::FTP_DownloadFile(const char* file)
{
	/************************************进入被动模式，获取数据端口********************************************/
	int a, b, c, d;//192.168.2.1，ip
	int pa, pb, DateProt;//端口
	char ipaddr[20];
	memset(ipaddr, '\0', sizeof(ipaddr));
	memset(FTPbuff, '\0', sizeof(FTPbuff));
	ftpRec = true;
	TCP_Send(ReturnSocket, "PASV\r\n");//进入被动模式，获取数据端口
	long long start;
	start = XI_clock();
	while (1)  //得到返回数据
	{
		Sleep(1);
		if (strncmp(FTPbuff, "227", 3) == 0)//PASV模式
		{
			break;
		}
		else if (*FTPbuff != '\0')
		{
			return -1; //FTP进入被动模式失败
		}
		if ((start - XI_clock()) >= 200)
		{
			return -1; //FTP进入被动模式失败
		}
	}

	ftpRec = false;
	char* getPort = strchr(FTPbuff, '(');//检索出（）里的数据
	sscanf(getPort, "(%d,%d,%d,%d,%d,%d)", &a, &b, &c, &d, &pa, &pb);//提取ip
	sprintf(ipaddr, "%d.%d.%d.%d", a, b, c, d);//得到ip
	DateProt = pa * 256 + pb;//端口
    /************************************连接被动模式给的数据端口*****************************************/
	SOCKET DateSocket = socket(AF_INET, SOCK_STREAM, 0);
	//3.处理地址---网络字节顺序
	sockaddr_in DateAddr;
	DateAddr.sin_family = AF_INET;
	DateAddr.sin_port = htons(DateProt);
	//inet_pton(AF_INET, m_ip, & addr.sin_addr.S_un.S_addr);
	DateAddr.sin_addr.S_un.S_addr = inet_addr(ipaddr);
	int connetErr = connect(DateSocket, (sockaddr*)&DateAddr, sizeof(DateAddr));
	if (connetErr != 0) //连接不成功
	{
		closesocket(DateSocket);
		return -2; // 被动模式数据端口连接失败
	}
	/******************************************准备*********************************************************/
	char  cmd[30];
	char buff[128];
	memset(buff, '\0', sizeof(buff));
	memset(cmd, '\0', sizeof(cmd));
	sprintf(cmd, "RETR %s\r\n", file);
	memset(FTPbuff, '\0', sizeof(FTPbuff));
	ftpRec = true;
	TCP_Send(ReturnSocket, cmd);
	while (1)  //得到返回数据
	{
		Sleep(100);
		if (strncmp(FTPbuff, "226", 3) == 0)//226 Transfer OK
		{
			break;
		}
		else if (*FTPbuff != '\0')
		{
			return -3; //下载请求发送失败
		}
	}

	time_t now = time(NULL);
	tm* tm_t = localtime(&now);
	std::ostringstream ss;
	ss << tm_t->tm_year + 1900;
	string year = ss.str();
	ss.str("");
	ss <<tm_t->tm_mon + 1;
	string month = ss.str(); 
	ss.str("");
	ss <<tm_t->tm_mday;
	string day = ss.str();
	ss.str("");
	ss <<tm_t->tm_hour;
	string hour = ss.str();
	ss.str("");
	ss <<tm_t->tm_min;
	string minute = ss.str();
	ss.str("");
	ss <<tm_t->tm_sec;
	string second = ss.str();
	string fN(file);
	string fileName = "./file/Download"+ year + '_' + month + '_' + day + '_' + hour+ minute+ second;
	Sleep(100);
	fstream f;
	f.open(fileName, ios::app | ios::out);
	while ((recv(DateSocket, buff, 128, 0)) > 0)
	{	
		string data(buff);
			if (data.length() > 128) {
				data = data.substr(0, 128);
		}
		f << data;
		memset(buff, 0, sizeof(buff));
	}
	f.close();//关闭文件
	closesocket(DateSocket);//关闭套接字fclose(fp)
	return 0; //下载成功
}

int TCP_Socket_CRP::FTP_DownloadFileByCoverLocal(const char* file, const char* localAddress) {
	/************************************进入被动模式，获取数据端口********************************************/
	int a, b, c, d;//192.168.2.1，ip
	int pa, pb, DateProt;//端口
	char ipaddr[20];
	memset(ipaddr, '\0', sizeof(ipaddr));
	memset(FTPbuff, '\0', sizeof(FTPbuff));
	ftpRec = true;
	TCP_Send(ReturnSocket, "PASV\r\n");//进入被动模式，获取数据端口
	
	time_t now = time(NULL);
	int t;
	while (1)  //得到返回数据
	{
		Sleep(1);
		if (strncmp(FTPbuff, "227", 3) == 0)//PASV模式
		{
			break;
		}
		else if (*FTPbuff != '\0')
		{
			return -1; //FTP进入被动模式失败
		}
		t = time(NULL) - now;
		if (t >= 3) {
			return -1; //FTP进入被动模式失败
		}
	}
	ftpRec = false;
	char* getPort = strchr(FTPbuff, '(');//检索出（）里的数据
	sscanf(getPort, "(%d,%d,%d,%d,%d,%d)", &a, &b, &c, &d, &pa, &pb);//提取ip
	sprintf(ipaddr, "%d.%d.%d.%d", a, b, c, d);//得到ip
	DateProt = pa * 256 + pb;//端口
    /************************************连接被动模式给的数据端口*****************************************/
	SOCKET DateSocket = socket(AF_INET, SOCK_STREAM, 0);
	//3.处理地址---网络字节顺序
	sockaddr_in DateAddr;
	DateAddr.sin_family = AF_INET;
	DateAddr.sin_port = htons(DateProt);
	//inet_pton(AF_INET, m_ip, & addr.sin_addr.S_un.S_addr);
	DateAddr.sin_addr.S_un.S_addr = inet_addr(ipaddr);
	int connetErr = connect(DateSocket, (sockaddr*)&DateAddr, sizeof(DateAddr));
	if (connetErr != 0) //连接不成功
	{
		closesocket(DateSocket);
		return -2; // 被动模式数据端口连接失败
	}
	/******************************************准备*********************************************************/
	char  cmd[30];
	char buff[128];
	memset(buff, '\0', sizeof(buff));
	memset(cmd, '\0', sizeof(cmd));
	sprintf(cmd, "RETR %s\r\n", file);
	memset(FTPbuff, '\0', sizeof(FTPbuff));
	ftpRec = true;
	TCP_Send(ReturnSocket, cmd);
	while (1)  //得到返回数据
	{
		Sleep(100);
		if (strncmp(FTPbuff, "226", 3) == 0)//226 Transfer OK
		{
			break;
		}
		else if (*FTPbuff != '\0')
		{
			return -3; //下载请求发送失败
		}
	}
	string  localFileName(localAddress);
	string fileName = "./file/"+ localFileName;
	Sleep(100);
	fstream f;
	f.open(fileName, ios::out);
	while ((recv(DateSocket, buff, 128, 0)) > 0)
	{
		string data(buff);
		if (data.length() > 128) {
			data = data.substr(0, 128);
		}
		f << data;
		memset(buff, 0, sizeof(buff));
	}
	f.close();//关闭文件
	closesocket(DateSocket);//关闭套接字fclose(fp)
	return 0; //下载成功
}

bool TCP_Socket_CRP::Modbus_send(SOCKET Socket, BYTE * array,int len)
{
	bool ifsucce;
	//int len = sizeof(array);
		//发消息给客户端
		//成功发送
		std::ofstream RecordFile("CROBOTPComRecord.txt", std::ios::app); //用记事本的方式记录下来
																	   //time_t t = time(NULL);
		SYSTEMTIME st = { 0 };
		GetLocalTime(&st);
		if (send(Socket,(char *)array, len, 0) != SOCKET_ERROR)
		{
			RecordFile << st.wYear << "/" << st.wMonth << "/" << st.wDay << "  " << st.wHour << ":" << st.wMinute << ":" << st.wSecond << "-" << st.wMilliseconds << ":" << "服务器 " << ":" << (char *)array << "\n";
			ifsucce = true;
		}
		else//发送失败
		{
			RecordFile << st.wSecond << "-" << st.wMilliseconds << ":" << "服务器 " << ":" << "发送失败" << "\n";
			ifsucce = false;
		}
		RecordFile.close();
	
	return ifsucce;
}
void TCP_Socket_CRP::TCP_Close(SOCKET Socket)
{
	if (Socket > 0) {
		closesocket(Socket);
		Socket = 0;
	}
	m_connected = false;
}