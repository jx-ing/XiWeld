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
//��һ���ڶ������������� ʹ���첽�׽ӵ�
void TCP_Socket_ERC::TCP_init(char* m_ip,u_short Prot)
{
	if (m_iniPort)return;
	//1.��ʼ�������׽��ֿ⡣��ѡ�ˣ��Ѿ���ʼ���ˡ�

	//2.�����׽���
	DefultSocket = socket(AF_INET, SOCK_STREAM, 0);
	//3.�����ַ---�����ֽ�˳��
	addr.sin_family = AF_INET;
	addr.sin_port = htons(Prot);
	//addr.sin_addr.S_un.S_addr = (m_ip[0] == '0') ? htonl(INADDR_ANY) : inet_addr(m_ip);
	inet_pton(AF_INET, m_ip, & addr.sin_addr.S_un.S_addr);
	//addr.sin_addr.S_un.S_addr = inet_addr(m_ip);
	//addr.sin_addr.S_un.S_addr = INADDR_ANY;
	//4.�󶨶˿ں�ip
	::bind(DefultSocket, (sockaddr *)&addr, sizeof(addr));
	listen(DefultSocket, 5);
	m_iniPort = true;
	//������IP
	Record_IP = m_ip;
	//�����׽���Ϊ�첽�׽�
	//WSAAsyncSelect(DefultSocket, m_hWnd, WM_SOCKET, FD_ACCEPT | FD_READ);
	//�������TEXT����Ϊ�����޸�
//	m_text.EnableWindow(false);
	//�޸ķ�����״̬
//	m_text.SetWindowTextA("�����������Ѿ�����");

}
void TCP_Socket_ERC::SetCallBackRcvData(CALL_BACK_ESTUN_SOCKET_RCV_DATA pFun, void *pFather)
{
	m_pFather = pFather;     //�����ʹ�ö����Socket���󣬸���
	m_pfunCallBack = pFun;   //����ָ�룬ָ���� void(*CALL_BACK_SOCKET_RCV_DATA)(void *, const char *ucRcvData, int nDataLen)
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
	TCP_Socket_ERC*app = (TCP_Socket_ERC*)pParam; //������߳�ָ�룬��������߳����������
										   // CSingleLock SingleLock(&(app->mutex));     //��
	CString str = "", str1;

	int listen_infor;

	int *information = new int;
	SOCKET socket_this;
	//�ͻ���IP,�˿ں�
	CString Cline_IP, Cline_Port;
	UCHAR Ip_1,Ip_2,Ip_3,Ip_4;
	//char CIp_1, CIp_2, CIp_3, CIp_4;
	int I_Ip_1, I_Ip_2, I_Ip_3, I_Ip_4;
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
				socket_this = accept(app->DefultSocket, (sockaddr*)&app->ClinerAddr, &length);//���ܿͻ���
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
				listen_infor = recv(app->ReturnSocket, app->recvbuffer, 1000, 0); //������Ϣ

				if (listen_infor > 0)
				{
					//if (app->ifRecord)
					{
						std::ofstream RecordFile("./EstunComRecord.txt", std::ios::app); //�ü��±��ķ�ʽ��¼����
																					   //time_t t = time(NULL);
						SYSTEMTIME st = { 0 };
						GetLocalTime(&st);
						//RecordFile<<ctime(&t) << "	�ͻ���" << app->ReturnSocket << ":" << app->listen_information.recvbuffer << "\n";
						RecordFile << st.wMinute << ":" << st.wSecond << "-" << st.wMilliseconds << ":" << "�ͻ���" << I_Ip_1 <<"."<< I_Ip_2 <<"."<< I_Ip_3 <<"."<< I_Ip_4 << ":" << app->recvbuffer << "\n";
						RecordFile.close();
					}
					app->m_pfunCallBack((void*)app->m_pFather, app->recvbuffer, listen_infor); // m_pfunCallBack�Ǹ�ָ�룬������ FanucRobotCtrl::Rcv,��������

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
	if (str == "")//���ܷ��Ϳ���Ϣ
	{
		return false;
	}
	else
	{
		//����Ϣ���ͻ���
		//�ɹ�����
		std::ofstream RecordFile("./EstunComRecord.txt", std::ios::app); //�ü��±��ķ�ʽ��¼����
		           										   //time_t t = time(NULL);
		SYSTEMTIME st = { 0 };
		GetLocalTime(&st);
		if (send(Socket, str.GetBuffer(1), str.GetLength(), 0) != SOCKET_ERROR)
		{
			UCHAR AD = addr.sin_addr.S_un.S_un_b.s_b1;
			RecordFile << st.wYear<<"/"<<st.wMonth<<"/"<<st.wDay<< "  "<<st.wHour<<":"<<st.wMinute<<":" << st.wSecond << "-" << st.wMilliseconds << ":" << "������ " << Record_IP << ":" << str << "\n";
			ifsucce = true;
		}
		else//����ʧ��
		{
			RecordFile << st.wSecond << "-" << st.wMilliseconds << ":" << "������ " << Record_IP << ":" << str << " :�ͻ���δ���ӣ�ʧ��" << "\n";
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