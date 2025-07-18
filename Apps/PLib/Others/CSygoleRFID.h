#pragma once
#include "Apps/PLib/BasicFunc/ChoiceResources.h"

#if ENABLE_RFID

#include <iostream>
#include <VECTOR>
//using namespace std; 
//#using "Sygole.HFReader.dll"
//using namespace Sygole::HFReader;
//using namespace Sygole;

//#define byte unsigned char


enum CStatus_enum {
	//SUCCESS				= 0x00,//ִ�гɹ�
	//FAILURE				= 0x80,//ִ��ʧ��
	//NO_TAG_ERROR		= 0x90, //�ޱ�ǩ����
	//BCC_ERROR			= 0xA0,//BCCУ�����
	//BLOCK_SIZE_ERROR	= 0XB0, //����д�����ʱ�����С��������
	//TIMEOUT_ERROR		= 0XC0,//��ʱ����
	//GPO_PORT_ERROR		= 0XD0,//����˿ڴ���
	//NO_RESPONSE			= 0XF1, //����Ӧ
	//PARAM_ERR			= 0XF2,//��������
	//SERIAL_CLOSED		= 0XF3 //ͨ�Ŷ˿ڹر�
};

enum CBaudEnum {

};

struct CTagInfo
{
	byte AFI;
	byte BlockCnt;
	byte BlockSize;
	byte DSFID;
	byte ICrefcerence;
	byte InformationFlag;
	std::vector<byte> UID;
};

struct CUserCfg
{

}; 

struct CCommCfg
{

}; 

struct CAutoReadPara
{

};


class CSygoleRFID
{
public:
	CSygoleRFID();
	
	void Dispose();//�ͷ���Դ

	/*��;�� �����Ӷ˿ڡ�
		������
		addr������ָ�����д�����ӵĵ�ַ�������� IP ��ַ�������Ǵ��ڶ˿ڡ�
		Baud���ô���ͨѶʱ����ָ�������ʡ�������һ��Ϊ 115200 ���� 9600�����Ծ�����
		�Ŷ�д����˵����Ϊ׼��
		Port : ����ָ�� TCP / IP ͨѶ�ǵĶ˿ںţ���д��Ĭ�ϵĶ˿ں�Ϊ 3001��������
		�����Э��ת����������ʵ�����õĶ˿ں�Ϊ׼��
		����ֵ��bool���򿪳ɹ����� true��ʧ�ܷ��� false��*/

	bool Connect(std::string addr,int baud);//��������
	bool Connect(std::string addr , short port);//��������

	void DisConnect();//�ж�����

	/*��;�� ��ȡ��ǰ��ǩ��Ψһʶ��š�
		������
		ReaderID����д�� ID��
		UID������ȡ�ı�ǩ UID��*/
	int Inventory(byte ReaderID,  byte* UID);//��ȡ��ǰ��ǩ��Ψһʶ���

	/*��;�����ֽ���ʽ��ȡ��ǩ�ڴ�
		������
		ReaderID����д�� ID
		addr����ȡ����ʼ��ַ
		len : ��ȡ���ֽڳ���
		datas : �������ݻ�����*/
	int ReadBytes(byte ReaderID, short addr, byte len, std::vector<byte>& datas);//���ֽ���ʽ��ȡ��ǩ�ڴ�
	int ReadBytes(std::string addrs, byte ReaderID, short addr, byte len, std::vector<byte> &datas);//���ֽ���ʽ��ȡ��ǩ�ڴ�

	//����ǩ����顣���ֱ�ǩ���ܲ�֧�ֻ�ֻ֧�������Ŀ顣
	/*������
		��Ƶ��д�� ʹ���ֲ� v2.1 �㶫˼�����ܼ������޹�˾
		4
		ReaderID����д�� ID
		StartBlock������Ҫ�����ĵ�һ�����
		BlockCnt������Ҫ�����Ŀ�����
		Datas����ȡ������
		Len : ��ȡ�������ݳ���
		����ֵ��Status_enum�������� 5 �½� ����ֵ��Ϣ��
		ע��һ�������Բ��� 8 ���顣*/
	int ReadMBlock(byte ReaderID, byte StartBlock,byte BlockCnt, std::vector<byte> &datas, byte &len);

	/*��;�����ֽ���ʽ�Ա�ǩ����д����
		������
		ReaderID : ��д�� ID
		addr : д�����ʼ��ַ
		len : д����ֽڳ���
		datas : д�������
		off : ��д����ֽ�*/
	int WriteBytes(byte ReaderID, short addr, byte len, std::vector<byte>& datas, int off);
	int WriteBytes(std::string addrs, byte ReaderID, short addr, byte len, std::vector<byte> &datas, int off);

	/*��;�� д��ǩ����顣��������Ϊ8������Խ�࣬������Ҫ��ʱ��Խ�����ȶ���
		��Ӧ������
		������
		ReaderID����д�� ID
		StartBlock������Ҫ��������ʼ���
		BlockCnt��������
		BlockSize�����С��һ����˵���С�� 4 �ֽں� 8 �ֽ�����
		BlockDatas����Ҫд���ǩ������
		����ֵ��Status_enum��
		ע���ڲ������ṹ��С������£�����ʹ���ֽڲ�����ʽ��д���ݡ�һ�������Բ��� 8 ��
		�顣*/
	int WriteMBlock(byte ReaderID, byte StartBlock,byte BlockCnt, int BlockSize, std::vector<byte> &BlockDatas);

/*
	��;����ȡ��ǰ��ǩ�Ľṹ��Ϣ
		������
		ReaderID����д�� ID
		Info����ǩ��ϵͳ��Ϣ����ϸ�������£�����������庬������������һ�£�
	����ֵ��Status_enum��*/
	int GetTagInfo(byte ReaderID, CTagInfo* Info);
	int GetTagInfo(std::string addrs, byte ReaderID, CTagInfo &Info);

/*
	��;�� ��ȡ��ǩ�Ŀ鰲ȫ״̬�����ֱ�ǩ��֧�֡�
		������
		ReaderID����д�� ID
		StartBlock�����л�ȡ�ĵ�һ�����
		BlockCnt��������
		TagSecurity����ȡ�Ŀ鰲ȫ״̬��0x00 ��ʾδ������0x01 ��ʾ����
		����ֵ��Status_enum��*/
	int GetTagSecurity(byte ReaderID, byte StartBlock, byte BlockCnt, std::vector<byte> &TagSecurity);

	/*��;�� ��ȡ��д����������Ϣ��
		������
		ReaderID����д�� ID��
		Cfg����ȡ��д����������Ϣ
		����ֵ��Status_enum��*/
	int GetUserCfg(byte ReaderID, CUserCfg &cfg);

	/*��;�� ���ö�д����������Ϣ��
		������
		ReaderID����д�� ID��
		Cfg����д����������Ϣ
		����ֵ��Status_enum��*/
	int SetUserCfg(byte ReaderID, CUserCfg cfg);

	/*��;�� ��ȡ��д����ͨѶ��Ϣ��
		������
		ReaderID����д�� ID��
		Cfg���洢��д����ͨѶ������Ϣ��
		����ֵ��Status_enum��*/
	int GetCommCfg(byte ReaderID, CCommCfg &cfg);

	/*��;�� ��ȡ��д����ͨѶ��Ϣ��
		������
		ReaderID����д�� ID��
		Cfg����д����ͨѶ������Ϣ�������� MAC ��������Ч��
		����ֵ��Status_enum��*/
	int SetCommCfg(byte ReaderID, CCommCfg cfg);

	/*��;����ȡ��д��������汾�š�
		������
		ReaderID����д�� ID��
		SoftVer����д��������汾�ţ���ʾʱת��Ϊ�ַ�����
		Len���汾�ŵĳ���
		����ֵ��Status_enum��*/
	int GetSWVer(byte ReaderID, std::vector<byte> &SoftVer , byte& len);

	/*��;��GPO������ơ�
		������
		ReaderID����д�� ID
		GPO��GPO �˿ں�
		connect��GPO �˿�״̬
		����ֵ��Status_enum��*/
	int GPOCtrl(byte ReaderID, byte GPO, bool bconnect);

	/*��;�� ��ȡGPI״̬��
		������
		ReaderID����д�� ID
		GpiStatus��GPI ״̬
		cnt : �˿�����
		����ֵ��Status_enum��*/
	int GetGpiStatus(byte ReaderID, byte& GpiStatus, byte& cnt);

	/*��;�����ö�д���Զ������Ĺ����Լ�������
		������
		ReaderID����д�� ID
		Para���Զ������Ĳ���
		����ֵ��Status_enum��*/
	int GetAutoReadFunc(byte ReaderID, CAutoReadPara para);
	
	/*��;����ȡ��д���Զ������Ĺ����Լ�������
		������
		ReaderID����д�� ID
		Para���Զ������Ĳ���
		����ֵ��Status_enum��*/
	int GetAutoReadFunc(byte ReaderID, CAutoReadPara &para);

/*
	��;�����ö�д�����ڲ�����
		������
		ReaderID����д�� ID
		baud�����ڲ�����
		����ֵ��Status_enum*/
	int SetBaud(byte ReaderID, CBaudEnum baud);

	/*��;����������
		������ReaderID����д�� ID
		����ֵ��Status_enum��*/
	int SaveCfg(byte ReaderID);

/*
	��;�� �ָ�Ĭ������
		������ReaderID����д�� ID
		����ֵ��Status_enum*/
	int DefaultCfg(byte ReaderID);

	/*��;����۽������ȡ����ǩʱ������
		������
		AutoReadEventArgs Args���Զ���ȡ�����¼�ʱЯ���Ĳ������������£�
		public class AutoReadEventArgs : EventArgs
	{
		public byte[] UID;
		public AutoReadEventArgs();
		public Antenna_enum ant{ get; set; }
		public CommArgs comm{ get; set; }
	}
}
����CommArgs�Ķ������£�
public class CommArgs
{
	public CommArgs();
	public string addr{ get; set; }
	public byte ReaderID{ get; set; }
}*/
//	AutoReadEventArgs AutoReadHandler();


/*
	��;������GPI����ʱ������
		������
		GPITriggerEventArgs Args��GPI ��������¼�ʱЯ���Ĳ������������£�
		public class GPITriggerEventArgs : EventArgs
	{
		public GPITriggerEventArgs();
		public GpiEnum Gpi{ get; set; }
		public CommArgs comm{ get; set; }
	}
����GpiEnum�Ķ�������:
	public enum GpiEnum
	{
		GPI_1 = 1,
		GPI_2 = 2,
		GPI_3 = 3,
		GPI_4 = 4,
	}*/
	//GPITriggerEventArgs GPITriggerHandler();
	void test();


};

#endif // ENABLE_RFID

