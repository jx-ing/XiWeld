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
	/*************************************************ͨ��socket******************************************************************/
	void TCP_init(char * m_ip, u_short Prot,int flag = SERVER);
	void TCP_Socket_CRP::SetCallBackRcvData(CALL_BACK_SOCKET_RCV_DATA_A pFun, void *pFather); //���븸�࣬��ָ�뺯��
	int TCP_Socket_CRP::startSocket();    //����
	int static TCP_Socket_CRP::TCP_Cyc_Accep(LPVOID pParam);
	CALL_BACK_SOCKET_RCV_DATA_A m_pfunCallBack;                //����������SetCallBackRcvData����������ĺ���
	void *m_pFather;                                        //����ָ��
	BOOL TCP_Socket_CRP::isConnected();  //�Ƿ�����
	//���ص������ӵĿͻ�������������LParam�������0-���ܿͻ��ˣ�1-���ܵ����ݣ����ܵ����ݴ洢��recvbuffer��
	bool TCP_Socket_CRP::TCP_Send(SOCKET,CString);
	// ʵʱ׷�ٹ���
	int TCP_Socket_CRP::track(unsigned int myPort);
	/*********************************************ftp����*****************************************************************/
	
	int user(char name[10]);
	int pass(char* password);
	int biteMode();
	bool FTP_SendFile(const char* file); //FTP�ϴ��ļ�����Ŀ¼
	bool FTP_SendFileToAddress(const char * file, const char * address);//FTP�ϴ��ļ���Ŀ��Ŀ¼
	int FTP_DownloadFile(const char* file); //FTP����ָ���ļ����ڱ����½�һ���ļ���
	int FTP_DownloadFileByCoverLocal(const char * file, const char * localAddress); //FTP����ָ���ļ������Ǳ���ָ���ļ���

	/***********************************************ModbusTCP����***************************************************************/
	bool Modbus_send(SOCKET,BYTE *array,int len);//����16����

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
	char  recvbuffer[1000]; //��������

	SOCKET DefultSocket;  //�ͻ���socket
	SOCKET ClientSocket;     //ftp�ͻ���
	bool FTP_if_load;      //�Ƿ��¼
	bool FTP_if_connection; //�Ƿ�����
	bool FTP_if_passive;    //�Ƿ�Ϊ����ģʽ
	bool FTP_if_upComplete;  //�Ƿ��ϴ����

	SOCKET ReturnSocket; //���Ӻ󷵻ص�socket
	
	sockaddr_in addr; //������IP��ַ����
	sockaddr_in ClinerAddr; //�ͻ��˵�IP��ַ����

	bool ftpRec;
	char FTPbuff[50];//ftp��������
	char*  FTPsendbuff;//ftp���ͻ�����
	BYTE cmd_send[256];//��ŷ��͵�ָ��
	//vector<SOCKET> Clint;
};

