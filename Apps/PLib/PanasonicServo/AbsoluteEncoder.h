// AbsoluteEncoder.h: interface for the CAbsoluteEncoder class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_ABSOLUTEENCODER_H__C94F8B2C_163F_4C1E_BC4A_DDB9C4D5DDC9__INCLUDED_)
#define AFX_ABSOLUTEENCODER_H__C94F8B2C_163F_4C1E_BC4A_DDB9C4D5DDC9__INCLUDED_

#include "v8stdint.h"
/*#include "WelderCtrl\serial\include\serial\serial.h"*/
#include "serial.h"

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#define ALLOW_REPETITION_NUM	2		//�����ظ�����

//ͣ��
//#define LAP_POLSE_NUM			2000.0	//���תһȦ��������

#define LAP_DATA_BASE			8388607	//���תһȦ����������

#define COMMUNICATION_ERROR_REPETITION if ( (bRet == FALSE) && (nErrorNum < ALLOW_REPETITION_NUM))\
{\
	nErrorNum++;\
	goto Begin;\
}\

#define COMMUNICATION_ERROR_RETURN if (bRet == FALSE)\
{\
	return FALSE;\
}\

#define COMMUNICATION_ERROR_CONTINUE if (bRet == FALSE)\
{\
	continue;\
}\

const uint8_t1 HandshakeOrderTab[31] = //����������ָ���
{
	0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8A, 0x8B, 0x8C, 0x8D, 0x8E, 0x8F, 0x90, 0x91, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9A, 0x9B, 0x9C, 0x9D, 0x9E, 0x9F,
};

const uint8_t1 AxisNoTab[31] = //��������ű�
{
	0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F,
};

const uint8_t1 OrderCheckSumTab[31] = //������LrcУ�����
{
	0x2D, 0x2C, 0x2B, 0x2A, 0x29, 0x28, 0x27, 0x26, 0x25, 0x24, 0x23, 0x22, 0x21, 0x20, 0x1F, 0x1E, 0x1D, 0x1C, 0x1B, 0x1A, 0x19, 0x18, 0x17, 0x16, 0x15, 0x14, 0x13, 0x12, 0x11, 0x10, 0x0F,
};

typedef struct 
{
// 	BOOL bHBit7;
// 	BOOL bHBit6;
	BOOL bHBit5;
	BOOL bHBit4;
// 	BOOL bHBit3;
// 	BOOL bHBit2;
// 	BOOL bHBit1;
// 	BOOL bHBit0;

	BOOL bLBit7;
	BOOL bLBit6;
	BOOL bLBit5;
/*	BOOL bLBit4;*/
	BOOL bLBit3;
	BOOL bLBit2;
	BOOL bLBit1;
	BOOL bLBit0;
}T_ENCODER_STATE_TAB;

class CAbsoluteEncoder  
{
public:
	CAbsoluteEncoder();
	virtual ~CAbsoluteEncoder();

	//��ʼ������������485ͨ��
	BOOL InitPort(	
		int nPortNo, //���ں�
		uint32_t1 nBaudrate = 9600, //������
		uint32_t1 inter_byte_timeout = 0, //��ʱ����
		uint32_t1 read_timeout_constant = 1000,				
		uint32_t1 read_timeout_multiplier = 0, 
		uint32_t1 write_timeout_constant = 1000,
        uint32_t1 write_timeout_multiplier = 0);

	//�Ӿ���ֵ��������ȡ��ǰ���꣬��Ҫ 1s ���ң���ȡʱ�ŷ�������뾲ֹ��nAxisNo Ϊ���ŷ������� Pr5.31 �����õ���ż� 1 ��ֵ��
	double GetCurrentCoorFromAbsEncoder(double dPulseToDis, int nAxisNo = 0, double dLapPulse = 2000.0, long lInitLapData = 0);

	//��ն�Ȧ����
	BOOL ClearManyLapData(int nAxisNo = 0);
	//������㵥Ȧ����
	BOOL SetInitLapData(int nAxisNo = 0);

	//�رմ���
	void ClosePort();
protected:

	void WriteToPort(std::vector<uint8_t1> vtFrameBuffer);//д����
	void ReadToPort(uint8_t1 *asBuffer, size_t nBufferSize);//������
	void ReadToPort(vector<uint8_t1> &vsBuffer, size_t nBufferSize);
	void ReadBuffer(uint8_t1 *asBuffer, size_t nBufferSize);

private:
	BOOL DataCheckSum(std::vector<uint8_t1> vtFrameBuffer);//LrcУ������
	void GetDataCheckSum( std::vector<uint8_t1> vtFrameBuffer, uint8_t1 &nDataCheckSum );//�õ�LrcУ����

	BOOL GetAbsoluteEncoderData( int nAxisNo, long &lAbsCoorLap,  long &lAbsCoorManyLap);//�õ�����������
	double CalCurrentCoor(long lAbsCoorLap,  long lAbsCoorManyLap, double dPulseToDis, double dLapPulse);//���㵱ǰ����

	BOOL Handshake( int nAxisNo );//����
	BOOL ApplicationData(int nAxisNo);//�������ֵ����
	BOOL ApplicationCmd(int nAxisNo, uint8_t1 unCmd);//�������ֵ����
	BOOL AllowSendData( int nAxisNo );//����������
	BOOL ReceiveData( int nAxisNo, std::vector<uint8_t1> &vsReceptionData );//��������
	BOOL ReceiveData(int nAxisNo, std::vector<uint8_t1> &vsReceptionData, int nSize);//��������

	void WriteEncodderLog(char *format, ... );//��������־
	
	BOOL CheckEncoderState(std::vector<uint8_t1> vtFrameBuffer, T_ENCODER_STATE_TAB &tEncoderState);//��������״̬
	void PrintEncoderError(T_ENCODER_STATE_TAB tEncoderState);//��ӡ������������Ϣ

	long m_lInitLapData;//��ʼ��Ȧ����

protected:
	int m_nPortNo = -1;//���ں�
	BOOL m_bPortSoftEnable = TRUE;//��ʹ��
	serial::Serial m_AbsEncoderSerial;
	
	/// ȫ���쳣
	class Exception : public std::runtime_error
	{
	public:
		explicit Exception(const std::string& what_arg) :
		std::runtime_error(what_arg) {}
		explicit Exception(const char * what_arg) :
		std::runtime_error(what_arg) {}
		virtual ~Exception() throw () {}
	};

	/// ���ڶ�ȡ��ʱ
	class Timeout : public Exception
	{
	public:
		explicit Timeout(const std::string& what_arg) :
		Exception(what_arg)
		{}
		explicit Timeout(const char * what_arg) :
		Exception(what_arg)
		{}
		
		virtual ~Timeout() throw()
		{}
	};

};

#endif // !defined(AFX_ABSOLUTEENCODER_H__C94F8B2C_163F_4C1E_BC4A_DDB9C4D5DDC9__INCLUDED_)
