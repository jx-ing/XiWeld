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
	void SetCallBackRcvData(CALL_BACK_ESTUN_SOCKET_RCV_DATA pFun, void* pFather); //���븸�࣬��ָ�뺯��

	int startSocket();    //����


	int static TCP_Cyc_Accep(LPVOID pParam);


	CALL_BACK_ESTUN_SOCKET_RCV_DATA m_pfunCallBack;                //����������SetCallBackRcvData����������ĺ���
	void* m_pFather;                                        //����ָ��

	BOOL isConnected();  //�Ƿ�����
	//���ص������ӵĿͻ�������������LParam�������0-���ܿͻ��ˣ�1-���ܵ����ݣ����ܵ����ݴ洢��recvbuffer��

	bool TCP_Send(SOCKET, CString);
	void TCP_Close(SOCKET Socket);


public:
	//bool ifRecord;
	bool m_connected;
	bool m_iniPort;
	bool ThreadKill;
	CWinThread* Thread_listen, * Threadd_accep;
	int m_clientSocket;
	char  recvbuffer[1000]; //��������
	SOCKET DefultSocket;  //�ͻ���socket
	SOCKET ReturnSocket; //���Ӻ󷵻ص�socket

	sockaddr_in addr; //������IP��ַ����
	sockaddr_in ClinerAddr; //�ͻ��˵�IP��ַ����

	CString Record_IP;//IP
	CString Record_Port;//�˿�


	//vector<SOCKET> Clint;
};

#endif // !__TCP_SOCKET

