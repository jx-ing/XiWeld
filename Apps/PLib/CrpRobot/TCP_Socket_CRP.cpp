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
//��һ���ڶ������������� ʹ���첽�׽ӵ�
void TCP_Socket_CRP::TCP_init(char* m_ip,u_short Prot,int flag)
{
	if (m_iniPort)return;
	//1.��ʼ�������׽��ֿ⡣��ѡ�ˣ��Ѿ���ʼ���ˡ�
	m_flag = flag;
	//2.�����׽���
	DefultSocket = socket(AF_INET, SOCK_STREAM, 0);
	//3.�����ַ---�����ֽ�˳��
	addr.sin_family = AF_INET;
	addr.sin_port = htons(Prot);
	//inet_pton(AF_INET, m_ip, & addr.sin_addr.S_un.S_addr);
	addr.sin_addr.S_un.S_addr = inet_addr(m_ip);
	//addr.sin_addr.S_un.S_addr = INADDR_ANY;
	if (m_flag == SERVER)
	{
		//4.�󶨶˿ں�ip
		::bind(DefultSocket, (sockaddr *)&addr, sizeof(addr));
		listen(DefultSocket, 5);
		m_iniPort = true;
	}
	//�����׽���Ϊ�첽�׽�
	//WSAAsyncSelect(DefultSocket, m_hWnd, WM_SOCKET, FD_ACCEPT | FD_READ);
	//�������TEXT����Ϊ�����޸�
//	m_text.EnableWindow(false);
	//�޸ķ�����״̬
//	m_text.SetWindowTextA("�����������Ѿ�����");

}

void TCP_Socket_CRP::SetCallBackRcvData(CALL_BACK_SOCKET_RCV_DATA_A pFun, void *pFather)
{
	m_pFather = pFather;     //�����ʹ�ö����Socket���󣬸���
	m_pfunCallBack = pFun;   //����ָ�룬ָ���� void(*CALL_BACK_SOCKET_RCV_DATA_A)(void *, const char *ucRcvData, int nDataLen)
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
	TCP_Socket_CRP*app = (TCP_Socket_CRP*)pParam; //������߳�ָ�룬��������߳����������
										   // CSingleLock SingleLock(&(app->mutex));     //��
	CString str = "", str1;

	int listen_infor;

	int clineErr=0;

	int *information = new int;
	SOCKET socket_this=0;
	while (1)
	{
		if (app->ThreadKill)
		{
			TRACE("ɱ��Auto_Edit�߳�_����");
			delete information;

			DWORD dwExitCode;                              //ָ���߳��˳���
			GetExitCodeThread(app->Threadd_accep, &dwExitCode);  //��ȡ�߳�1���˳���
			AfxEndThread(dwExitCode, TRUE);                //�˳��߳�			
		}
		else
		{
			int length = sizeof(app->ClinerAddr);
			//���տͻ��˵�����
			if (!app->m_connected)
			{
				if (app->m_flag == SERVER)
				{
					socket_this = accept(app->DefultSocket, (sockaddr*)&app->ClinerAddr, &length);//���ܿͻ���
				}
				if (app->m_flag == CLIENT)
				{
					app->ClientSocket = app->DefultSocket;
					if (app->m_FTPClient__ == 1)
					{
						if (clineErr == -1)//�ͻ��˶Ͽ����������ͻ����Զ���������
						{
							app->ClientSocket = socket(AF_INET, SOCK_STREAM, 0);
							app->FTP_if_load = false;
							app->FTP_if_connection = false;
						}
					}
					else if (app->m_ModbusTCPClient__ == 1)
					{
						if (clineErr == -1)//�ͻ��˶Ͽ����������ͻ����Զ���������
						{
							app->ClientSocket = socket(AF_INET, SOCK_STREAM, 0);
							//app->FTP_if_load = false;
							//app->FTP_if_connection = false;
						}
					}
					else
					{
						if (clineErr == -1)//�ͻ��˶Ͽ����������ͻ����Զ���������
						{
							app->ClientSocket = socket(AF_INET, SOCK_STREAM, 0);
						}
					}
					clineErr = connect(app->ClientSocket, (sockaddr*)(&app->addr),sizeof(sockaddr_in));//���ӷ�����
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
				
				listen_infor = recv(app->ReturnSocket, app->recvbuffer, 1000, 0); //������Ϣ

				if (listen_infor > 0)
				{
					if (app->ftpRec)//����ftp
					{
						for (int i = 0; i < 50; i++)
						{
							app->FTPbuff[i] = app->recvbuffer[i];
						}
					}
					std::ofstream RecordFile("EstunComRecord.txt", std::ios::app); //�ü��±��ķ�ʽ��¼����
																				   //time_t t = time(NULL);
					SYSTEMTIME st = { 0 };
					GetLocalTime(&st);
					//RecordFile<<ctime(&t) << "	�ͻ���" << app->ReturnSocket << ":" << app->listen_information.recvbuffer << "\n";
					RecordFile << st.wSecond << "-" << st.wMilliseconds << ":" << "�ͻ���" << app->ReturnSocket << ":" << app->recvbuffer << "\n";
					RecordFile.close();
					app->m_pfunCallBack((void*)app->m_pFather, app->recvbuffer, listen_infor); // m_pfunCallBack�Ǹ�ָ�룬������ FanucRobotCtrl::Rcv,��������

				}
				else if ((listen_infor == -1)||(listen_infor==0))
				{
					app->m_connected = false;
				}
			
		}
	}

}

/*
TCP_Send�����Ƕ�send�����ķ�װ����Ҫ����ˣ�
       1. ����send����
	   2. �������send����ʧ�ܣ��ͽ�ʧ����Ϣд����־�CROBOTPComRecord.txt��
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

		// 2. ��socket��ip��ַ�Ͷ˿ں�
		addr.sin_family = AF_INET;
		addr.sin_addr.s_addr = htonl(INADDR_ANY);	//INADDR_ANYת����������0.0.0.0����ָ��������˼��Ҳ���Ǳ�ʾ����������IP
		addr.sin_port = htons(serverPort);
		if (-1 == ::bind(DefultSocket, (struct sockaddr*)&addr, sizeof(addr))) {
			WSACleanup();
			return -1;
		}

		//3. �����׽���Ϊ����״̬
		if (listen(DefultSocket, SOMAXCONN) < 0) // ���ü���״̬ʧ��
		{
			WSACleanup();
		}

		//4. �������󣬽�����������
		int len = sizeof(SOCKADDR);
		ClientSocket = accept(DefultSocket, (SOCKADDR *)&ClinerAddr, &len);
		if (ClientSocket < 0)
		{
			WSACleanup();
			return -1;
		}

		ofstream RecordFile; //�ü��±��ķ�ʽ��¼����
		RecordFile.open("./file/trackLog.txt");
		RecordFile << "Start" << endl;

		//5. �÷��ص��׽��ֺͿͻ��˽���ͨ��
		//send(cli_fd, "B\n", 9, 0);
		BYTE fffb[8] = { 0xFF,0xFB,0x06,0x00,'>','0','0','\r' };
		string strRecv;
		while (1)
		{
			memset(cmd_send, '\0', sizeof(BYTE) * 256);
			int  recv_len = recv(ClientSocket, recvbuffer, sizeof(recvbuffer), 0);
			if (recv_len < 0)
			{
				break;// ����ʧ��
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
				WSACleanup();// �ͷ�DLL��Դ
				break;
			}
			else if (strRecv == "JH+TON\r") {
				RecordFile << "Client:" << strRecv;
				send(ClientSocket, (char*)fffb, 8, 0);
				RecordFile << "Server:" << "FFFB0600>00\r";
				while (1) { //��������: 1. ��trakeʱ��Ƭ��    2. ������λ��û���ꡣ

					memset(recvbuffer, '\0', sizeof(char) * 1000);
					if (recv(ClientSocket, recvbuffer, sizeof(recvbuffer), 0) < 0)
					{
						break;// ����ʧ��
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
						cmd_send[2] = str1.size() + 2; //���ĳ�������Ӧ
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
	if (str == "")//���ܷ��Ϳ���Ϣ
	{
		return false;
	}
	else
	{
		//����Ϣ���ͻ���
		//�ɹ�����
		std::ofstream RecordFile("CROBOTPComRecord.txt", std::ios::app); //�ü��±��ķ�ʽ��¼����
		           										                 //time_t t = time(NULL);
		SYSTEMTIME st = { 0 };
		GetLocalTime(&st);
		if (send(Socket, str.GetBuffer(1), str.GetLength(), 0) != SOCKET_ERROR) 
		{
			RecordFile << st.wYear<<"/"<<st.wMonth<<"/"<<st.wDay<< "  "<<st.wHour<<":"<<st.wMinute<<":" << st.wSecond << "-" << st.wMilliseconds << ":" << "������ "  << ":" << str << "\n";
			ifsucce = true;
		}
		else//����ʧ��
		{
			RecordFile << st.wSecond << "-" << st.wMilliseconds << ":" << "������ " << ":" << "����ʧ��" << "\n";
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
	//recv(ReturnSocket, FTPbuff, sizeof(FTPbuff), 0); //������Ϣ
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
	//recv(ReturnSocket, FTPbuff, sizeof(FTPbuff), 0); //������Ϣ
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
	while (1)  //�õ���������
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
//ftp�ϴ��ļ�����Ŀ¼(ʹ��ǰ�ȵ�¼)
bool TCP_Socket_CRP::FTP_SendFile(const char * file)
{
	/************************************���뱻��ģʽ����ȡ���ݶ˿�********************************************/
	int a, b, c, d;//192.168.2.1��ip
	int pa, pb,DateProt;//�˿�
	char ipaddr[20];
	memset(ipaddr,'\0',sizeof(ipaddr));
	memset(FTPbuff,'\0',sizeof(FTPbuff));
	ftpRec = true;
	TCP_Send(ReturnSocket, "PASV\r\n");//���뱻��ģʽ����ȡ���ݶ˿�
	long long start;
	start = XI_clock();
	while (1)  //�õ���������
	{
		Sleep(1);
		if (strncmp(FTPbuff, "227", 3) == 0)//PASVģʽ
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
	char* getPort = strchr(FTPbuff,'(');//�����������������
	sscanf(getPort,"(%d,%d,%d,%d,%d,%d)",&a,&b,&c,&d,&pa,&pb);//��ȡip
	sprintf(ipaddr,"%d.%d.%d.%d",a,b,c,d);//�õ�ip
	DateProt = pa * 256 + pb;//�˿�
	/************************************���ӱ���ģʽ�������ݶ˿�*****************************************/
	SOCKET DateSocket = socket(AF_INET, SOCK_STREAM, 0);
	//3.�����ַ---�����ֽ�˳��
	sockaddr_in DateAddr;
	DateAddr.sin_family = AF_INET;
	DateAddr.sin_port = htons(DateProt);
	//inet_pton(AF_INET, m_ip, & addr.sin_addr.S_un.S_addr);
	DateAddr.sin_addr.S_un.S_addr = inet_addr(ipaddr);
	int connetErr = connect(DateSocket, (sockaddr*)&DateAddr, sizeof(DateAddr));
	if (connetErr != 0) //���Ӳ��ɹ�
	{
		closesocket(DateSocket);
		return false;
	}
	/******************************************׼��*********************************************************/
	char  cmd[30];
	char buff[100];
	memset(buff,'\0',sizeof(buff));
	memset(cmd,'\0',sizeof(cmd));
	sprintf(cmd,"STOR %s\r\n", file);
	memset(FTPbuff,'\0',sizeof(FTPbuff));
	ftpRec = true;
	TCP_Send(ReturnSocket, cmd);//
	//int len = recv(DateSocket, buff, 100, 0); //������Ϣ
	while (1)  //�õ���������
	{
		Sleep(3);
		if (strncmp(FTPbuff, "150", 3) == 0)//PASVģʽ
		{
			break;
		}
		else if (*FTPbuff != '\0')
		{
			return false;
		}
	}
	ftpRec = false;
	/*****************************׼����ɣ����ļ�д�����뻺�������ȴ�����*********************************/
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
			int len = fread(FTPsendbuff, 1, length, fd1);//һ����FTP�ķ��͸���������1���ַ���ֱ����fd1�ļ�ȫ�Ž�ȥ
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
	while (1)  //�õ���������
	{
		Sleep(1);
		if (strncmp(FTPbuff, "226", 3) == 0)//�ϴ����
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
//ftp�ϴ��ļ���Ŀ��Ŀ¼
bool TCP_Socket_CRP::FTP_SendFileToAddress(const char * file,const char * address)
{
	/************************************���뱻��ģʽ����ȡ���ݶ˿�********************************************/
	int a, b, c, d;//192.168.2.1��ip
	int pa, pb, DateProt;//�˿�
	char ipaddr[20];
	memset(ipaddr, '\0', sizeof(ipaddr));
	memset(FTPbuff, '\0', sizeof(FTPbuff));
	ftpRec = true;
	TCP_Send(ReturnSocket, "PASV\r\n");//���뱻��ģʽ����ȡ���ݶ˿�
	long long start;
	start = XI_clock();
	while (1)  //�õ���������
	{
		Sleep(1);
		if (strncmp(FTPbuff, "227", 3) == 0)//PASVģʽ
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
	char* getPort = strchr(FTPbuff, '(');//�����������������
	sscanf(getPort, "(%d,%d,%d,%d,%d,%d)", &a, &b, &c, &d, &pa, &pb);//��ȡip
	sprintf(ipaddr, "%d.%d.%d.%d", a, b, c, d);//�õ�ip
	DateProt = pa * 256 + pb;//�˿�
	/************************************���ӱ���ģʽ�������ݶ˿�*****************************************/
	SOCKET DateSocket = socket(AF_INET, SOCK_STREAM, 0);
	//3.�����ַ---�����ֽ�˳��
	sockaddr_in DateAddr;
	DateAddr.sin_family = AF_INET;
	DateAddr.sin_port = htons(DateProt);
	//inet_pton(AF_INET, m_ip, & addr.sin_addr.S_un.S_addr);
	DateAddr.sin_addr.S_un.S_addr = inet_addr(ipaddr);
	int connetErr = connect(DateSocket, (sockaddr*)&DateAddr, sizeof(DateAddr));
	if (connetErr != 0) //���Ӳ��ɹ�
	{
		closesocket(DateSocket);
		return false;
	}
	/******************************************׼��*********************************************************/
	string t;
	istringstream in(file);
	vector<string> info;
	while (getline(in, t, '/')) {	//���ﵥ����Ҫע��
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
	//int len = recv(DateSocket, buff, 100, 0); //������Ϣ
	while (1)  //�õ���������
	{
		Sleep(3);
		if (strncmp(FTPbuff, "150", 3) == 0)//PASVģʽ
		{
			break;
		}
		else if (*FTPbuff != '\0')
		{
			return false;
		}
	}
	ftpRec = false;
	/*****************************׼����ɣ����ļ�д�����뻺�������ȴ�����*********************************/
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
			int len = fread(FTPsendbuff, 1, length, fd1);//һ����FTP�ķ��͸���������1���ַ���ֱ����fd1�ļ�ȫ�Ž�ȥ
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
	while (1)  //�õ���������
	{
		Sleep(1);
		if (strncmp(FTPbuff, "226", 3) == 0)//�ϴ����
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
//ͨ��ftp�ӷ����������ļ������ء�
int TCP_Socket_CRP::FTP_DownloadFile(const char* file)
{
	/************************************���뱻��ģʽ����ȡ���ݶ˿�********************************************/
	int a, b, c, d;//192.168.2.1��ip
	int pa, pb, DateProt;//�˿�
	char ipaddr[20];
	memset(ipaddr, '\0', sizeof(ipaddr));
	memset(FTPbuff, '\0', sizeof(FTPbuff));
	ftpRec = true;
	TCP_Send(ReturnSocket, "PASV\r\n");//���뱻��ģʽ����ȡ���ݶ˿�
	long long start;
	start = XI_clock();
	while (1)  //�õ���������
	{
		Sleep(1);
		if (strncmp(FTPbuff, "227", 3) == 0)//PASVģʽ
		{
			break;
		}
		else if (*FTPbuff != '\0')
		{
			return -1; //FTP���뱻��ģʽʧ��
		}
		if ((start - XI_clock()) >= 200)
		{
			return -1; //FTP���뱻��ģʽʧ��
		}
	}

	ftpRec = false;
	char* getPort = strchr(FTPbuff, '(');//�����������������
	sscanf(getPort, "(%d,%d,%d,%d,%d,%d)", &a, &b, &c, &d, &pa, &pb);//��ȡip
	sprintf(ipaddr, "%d.%d.%d.%d", a, b, c, d);//�õ�ip
	DateProt = pa * 256 + pb;//�˿�
    /************************************���ӱ���ģʽ�������ݶ˿�*****************************************/
	SOCKET DateSocket = socket(AF_INET, SOCK_STREAM, 0);
	//3.�����ַ---�����ֽ�˳��
	sockaddr_in DateAddr;
	DateAddr.sin_family = AF_INET;
	DateAddr.sin_port = htons(DateProt);
	//inet_pton(AF_INET, m_ip, & addr.sin_addr.S_un.S_addr);
	DateAddr.sin_addr.S_un.S_addr = inet_addr(ipaddr);
	int connetErr = connect(DateSocket, (sockaddr*)&DateAddr, sizeof(DateAddr));
	if (connetErr != 0) //���Ӳ��ɹ�
	{
		closesocket(DateSocket);
		return -2; // ����ģʽ���ݶ˿�����ʧ��
	}
	/******************************************׼��*********************************************************/
	char  cmd[30];
	char buff[128];
	memset(buff, '\0', sizeof(buff));
	memset(cmd, '\0', sizeof(cmd));
	sprintf(cmd, "RETR %s\r\n", file);
	memset(FTPbuff, '\0', sizeof(FTPbuff));
	ftpRec = true;
	TCP_Send(ReturnSocket, cmd);
	while (1)  //�õ���������
	{
		Sleep(100);
		if (strncmp(FTPbuff, "226", 3) == 0)//226 Transfer OK
		{
			break;
		}
		else if (*FTPbuff != '\0')
		{
			return -3; //����������ʧ��
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
	f.close();//�ر��ļ�
	closesocket(DateSocket);//�ر��׽���fclose(fp)
	return 0; //���سɹ�
}

int TCP_Socket_CRP::FTP_DownloadFileByCoverLocal(const char* file, const char* localAddress) {
	/************************************���뱻��ģʽ����ȡ���ݶ˿�********************************************/
	int a, b, c, d;//192.168.2.1��ip
	int pa, pb, DateProt;//�˿�
	char ipaddr[20];
	memset(ipaddr, '\0', sizeof(ipaddr));
	memset(FTPbuff, '\0', sizeof(FTPbuff));
	ftpRec = true;
	TCP_Send(ReturnSocket, "PASV\r\n");//���뱻��ģʽ����ȡ���ݶ˿�
	
	time_t now = time(NULL);
	int t;
	while (1)  //�õ���������
	{
		Sleep(1);
		if (strncmp(FTPbuff, "227", 3) == 0)//PASVģʽ
		{
			break;
		}
		else if (*FTPbuff != '\0')
		{
			return -1; //FTP���뱻��ģʽʧ��
		}
		t = time(NULL) - now;
		if (t >= 3) {
			return -1; //FTP���뱻��ģʽʧ��
		}
	}
	ftpRec = false;
	char* getPort = strchr(FTPbuff, '(');//�����������������
	sscanf(getPort, "(%d,%d,%d,%d,%d,%d)", &a, &b, &c, &d, &pa, &pb);//��ȡip
	sprintf(ipaddr, "%d.%d.%d.%d", a, b, c, d);//�õ�ip
	DateProt = pa * 256 + pb;//�˿�
    /************************************���ӱ���ģʽ�������ݶ˿�*****************************************/
	SOCKET DateSocket = socket(AF_INET, SOCK_STREAM, 0);
	//3.�����ַ---�����ֽ�˳��
	sockaddr_in DateAddr;
	DateAddr.sin_family = AF_INET;
	DateAddr.sin_port = htons(DateProt);
	//inet_pton(AF_INET, m_ip, & addr.sin_addr.S_un.S_addr);
	DateAddr.sin_addr.S_un.S_addr = inet_addr(ipaddr);
	int connetErr = connect(DateSocket, (sockaddr*)&DateAddr, sizeof(DateAddr));
	if (connetErr != 0) //���Ӳ��ɹ�
	{
		closesocket(DateSocket);
		return -2; // ����ģʽ���ݶ˿�����ʧ��
	}
	/******************************************׼��*********************************************************/
	char  cmd[30];
	char buff[128];
	memset(buff, '\0', sizeof(buff));
	memset(cmd, '\0', sizeof(cmd));
	sprintf(cmd, "RETR %s\r\n", file);
	memset(FTPbuff, '\0', sizeof(FTPbuff));
	ftpRec = true;
	TCP_Send(ReturnSocket, cmd);
	while (1)  //�õ���������
	{
		Sleep(100);
		if (strncmp(FTPbuff, "226", 3) == 0)//226 Transfer OK
		{
			break;
		}
		else if (*FTPbuff != '\0')
		{
			return -3; //����������ʧ��
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
	f.close();//�ر��ļ�
	closesocket(DateSocket);//�ر��׽���fclose(fp)
	return 0; //���سɹ�
}

bool TCP_Socket_CRP::Modbus_send(SOCKET Socket, BYTE * array,int len)
{
	bool ifsucce;
	//int len = sizeof(array);
		//����Ϣ���ͻ���
		//�ɹ�����
		std::ofstream RecordFile("CROBOTPComRecord.txt", std::ios::app); //�ü��±��ķ�ʽ��¼����
																	   //time_t t = time(NULL);
		SYSTEMTIME st = { 0 };
		GetLocalTime(&st);
		if (send(Socket,(char *)array, len, 0) != SOCKET_ERROR)
		{
			RecordFile << st.wYear << "/" << st.wMonth << "/" << st.wDay << "  " << st.wHour << ":" << st.wMinute << ":" << st.wSecond << "-" << st.wMilliseconds << ":" << "������ " << ":" << (char *)array << "\n";
			ifsucce = true;
		}
		else//����ʧ��
		{
			RecordFile << st.wSecond << "-" << st.wMilliseconds << ":" << "������ " << ":" << "����ʧ��" << "\n";
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