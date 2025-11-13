// AbsoluteEncoder.cpp: implementation of the CAbsoluteEncoder class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include ".\Project\XiGrooveRobot.h"
#include "AbsoluteEncoder.h"
#include <iomanip>
#include <assert.h>
#include <vector>
#include <map>
#include "CommonAlgorithm.h"
#include "OpenClass/FileOP/ini/OPini.h"

#include "shlwapi.h"
#pragma comment(lib, "shlwapi.lib")

#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
#define new DEBUG_NEW
#endif

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CAbsoluteEncoder::CAbsoluteEncoder()
{
}

CAbsoluteEncoder::~CAbsoluteEncoder()
{

}

BOOL CAbsoluteEncoder::InitPort(
			int nPortNo, 
			uint32_t1 nBaudrate,
			uint32_t1 inter_byte_timeout, 
			uint32_t1 read_timeout_constant,				
			uint32_t1 read_timeout_multiplier, 
			uint32_t1 write_timeout_constant,
            uint32_t1 write_timeout_multiplier)
{
	m_nPortNo = nPortNo;
	m_AbsEncoderSerial.setBaudrate(nBaudrate);
	
	std::string sComPortNo;
	std::string sComPortName;
	char cComPortNo[100];
	itoa(nPortNo, cComPortNo, 10);
	std::string sComPortANo(cComPortNo);
	sComPortName = "COM" + sComPortANo;
	sComPortNo = sComPortName.c_str();
	
	m_AbsEncoderSerial.setTimeout(inter_byte_timeout, read_timeout_constant, read_timeout_multiplier, write_timeout_constant, write_timeout_multiplier);
	m_AbsEncoderSerial.setPort(sComPortNo);
	
	try
	{
		m_AbsEncoderSerial.open();
	}
	catch (std::invalid_argument)
	{
		m_bPortSoftEnable = FALSE;
		WriteEncodderLog("无效参数，打开串口 %d 失败！", nPortNo);
	}
	catch (serial::SerialException)
	{
		m_bPortSoftEnable = FALSE;
		WriteEncodderLog("通信异常，打开串口 %d 失败！", nPortNo);
	}
	catch (serial::IOException)
	{
		m_bPortSoftEnable = FALSE;
		WriteEncodderLog("串口异常，打开串口 %d 失败，请检查线路连接。", nPortNo);
	}
	return m_bPortSoftEnable;
}

void CAbsoluteEncoder::WriteToPort(std::vector<uint8_t1> vtFrameBuffer)
{
	if (m_bPortSoftEnable == FALSE)
	{
		return;
	}
	try
	{
		m_AbsEncoderSerial.write(vtFrameBuffer);
	}
	catch (serial::IOException)
	{
		m_bPortSoftEnable = FALSE;
		WriteEncodderLog("发送数据 %x - %x 失败！", vtFrameBuffer[0], vtFrameBuffer[vtFrameBuffer.size() - 1]);
	}
}

void CAbsoluteEncoder::ReadBuffer(uint8_t1 *asBuffer, size_t nBufferSize)
{
	do 
	{ 
		if (m_AbsEncoderSerial.read((asBuffer), (nBufferSize)) != (nBufferSize)) 
		{ 
			m_bPortSoftEnable = FALSE;
			WriteEncodderLog("串口通信超时，请检查串口是否正常连接"); 
			throw Timeout("串口通信超时，请检查串口是否正常连接");
		} 
	} while (false);
}

void CAbsoluteEncoder::ReadToPort(uint8_t1 *asBuffer, size_t nBufferSize)
{
	if (m_bPortSoftEnable == FALSE)
	{
		return;
	}
	try
	{
		ReadBuffer(asBuffer, nBufferSize);
	}
	catch (CAbsoluteEncoder::Timeout)
	{
		//已修改
		XUI::MesBox::PopInfo("串口通信超时，请检查串口是否正常连接");
		//AfxMessageBox("串口通信超时，请检查串口是否正常连接");
		exit(-1);
	}
}

void CAbsoluteEncoder::ReadToPort(vector<uint8_t1> &vsBuffer, size_t nBufferSize)
{
	if (m_bPortSoftEnable == FALSE)
	{
		return;
	}

	uint8_t1 *asBuffer;
	asBuffer = new uint8_t1[nBufferSize];
	try
	{
		ReadBuffer(asBuffer, nBufferSize);
	}
	catch (CAbsoluteEncoder::Timeout)
	{
		//已修改
		XUI::MesBox::PopInfo("串口通信超时，请检查串口是否正常连接");
		//AfxMessageBox("串口通信超时，请检查串口是否正常连接");
		exit(-1);
	}
	vsBuffer = vector<uint8_t1>(asBuffer, asBuffer+nBufferSize);
	delete[]asBuffer;
}

BOOL CAbsoluteEncoder::Handshake( int nAxisNo )
{
	std::vector<uint8_t1> vsFrameBuffer;
	vsFrameBuffer.clear();
	vsFrameBuffer.push_back(HandshakeOrderTab[nAxisNo]);
	vsFrameBuffer.push_back(0x05);
	WriteToPort(vsFrameBuffer);

	long long nStTime = XI_clock();
	long long nTimeDurationThresh = (0.1 * CLOCKS_PER_SEC);
	uint8_t1 HEADER[2];
	HEADER[0] = HandshakeOrderTab[nAxisNo];
	HEADER[1] = 0x04;
	uint8_t1 FrameHead[2] = {0};
	ReadToPort(FrameHead, 1);
	for (;;)
	{	
		ReadToPort(FrameHead + 1, 1);
		if (FrameHead[0] == HEADER[0] && FrameHead[1] == HEADER[1])
			break;
		else
		{
			FrameHead[0] = FrameHead[1];
			WriteEncodderLog("警告，接收数据异常"); 
		}
		
		if((XI_clock() - nStTime) > nTimeDurationThresh)
		{
			return FALSE;
		}
	}
	return TRUE;
}

BOOL CAbsoluteEncoder::ApplicationData( int nAxisNo )
{
	std::vector<uint8_t1> vsFrameBuffer;
	vsFrameBuffer.clear();
	vsFrameBuffer.push_back(0x00);
	vsFrameBuffer.push_back(AxisNoTab[nAxisNo]);
	vsFrameBuffer.push_back(0xD2);
	vsFrameBuffer.push_back(OrderCheckSumTab[nAxisNo]);
	WriteToPort(vsFrameBuffer);
	uint8_t1 HEADER[1];
	HEADER[0] = 0x06;
	uint8_t1 FrameHead[1] = {0};
	ReadToPort(FrameHead, 1);
	if (FrameHead[0] == HEADER[0])
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

BOOL CAbsoluteEncoder::ApplicationCmd(int nAxisNo, uint8_t1 unCmd)
{
	std::vector<uint8_t1> vsFrameBuffer;
	vsFrameBuffer.clear();
	vsFrameBuffer.push_back(0x00);
	vsFrameBuffer.push_back(AxisNoTab[nAxisNo]);
	vsFrameBuffer.push_back(unCmd);
	uint8_t1 unCheckSum;
	GetDataCheckSum(vsFrameBuffer, unCheckSum);
	vsFrameBuffer.push_back(unCheckSum);
	WriteToPort(vsFrameBuffer);
	uint8_t1 HEADER[1];
	HEADER[0] = 0x06;
	uint8_t1 FrameHead[1] = { 0 };
	ReadToPort(FrameHead, 1);
	if (FrameHead[0] == HEADER[0])
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

BOOL CAbsoluteEncoder::AllowSendData( int nAxisNo )
{
	uint8_t1 HEADER[2];
	HEADER[0] = 0x80;
	HEADER[1] = 0x05;
	uint8_t1 FrameHead[2] = {0};
	ReadToPort(FrameHead, 2);
	if ((FrameHead[0] != HEADER[0]) || (FrameHead[1] != HEADER[1]))
	{
		return FALSE;
	}
	std::vector<uint8_t1> vsFrameBuffer;
	vsFrameBuffer.clear();
	vsFrameBuffer.push_back(0x80);
	vsFrameBuffer.push_back(0x04);
	WriteToPort(vsFrameBuffer);

	return TRUE;
}

BOOL CAbsoluteEncoder::ReceiveData(int nAxisNo, std::vector<uint8_t1> &vsReceptionData)
{
	std::vector<uint8_t1> vsFrameBuffer;

	uint8_t1 FrameHead[15] = { 0 };
	//读取数据
	ReadToPort(FrameHead, 15);

	//保存数据
	// 	WriteEncodderLog("%x, %x, %x, %x, %x, %x, %x, %x, %x, %x, %x, %x, %x, %x, %x", 
	// 		FrameHead[0],
	// 		FrameHead[1],
	// 		FrameHead[2],
	// 		FrameHead[3],
	// 		FrameHead[4],
	// 		FrameHead[5],
	// 		FrameHead[6],
	// 		FrameHead[7],
	// 		FrameHead[8],
	// 		FrameHead[9],
	// 		FrameHead[10],
	// 		FrameHead[11],
	// 		FrameHead[12],
	// 		FrameHead[13],
	// 		FrameHead[14]);

	vsFrameBuffer.clear();
	for (int nNum = 0; nNum < 15; nNum++)
	{
		vsFrameBuffer.push_back(FrameHead[nNum]);
	}

	//校验数据
	if (DataCheckSum(vsFrameBuffer))
	{
		vsReceptionData.clear();
		vsReceptionData.insert(vsReceptionData.begin(), vsFrameBuffer.begin(), vsFrameBuffer.end());
		vsFrameBuffer.clear();
		vsFrameBuffer.push_back(0x06);
		WriteToPort(vsFrameBuffer);
		return TRUE;
	}
	else
	{
		vsFrameBuffer.clear();
		vsFrameBuffer.push_back(0x15);
		WriteToPort(vsFrameBuffer);
		return FALSE;
	}
}
BOOL CAbsoluteEncoder::ReceiveData(int nAxisNo, std::vector<uint8_t1> &vsReceptionData, int nSize)
{
	std::vector<uint8_t1> vsFrameBuffer;

	uint8_t1 FrameHead[255] = { 0 };
	//读取数据
	ReadToPort(FrameHead, nSize);

	vsFrameBuffer.clear();
	for (int nNum = 0; nNum < nSize; nNum++)
	{
		vsFrameBuffer.push_back(FrameHead[nNum]);
	}

	//校验数据
	if (DataCheckSum(vsFrameBuffer))
	{
		vsReceptionData.clear();
		vsReceptionData.insert(vsReceptionData.begin(), vsFrameBuffer.begin(), vsFrameBuffer.end());
		vsFrameBuffer.clear();
		vsFrameBuffer.push_back(0x06);
		WriteToPort(vsFrameBuffer);
		return TRUE;
	}
	else
	{
		vsFrameBuffer.clear();
		vsFrameBuffer.push_back(0x15);
		WriteToPort(vsFrameBuffer);
		return FALSE;
	}
}

BOOL CAbsoluteEncoder::DataCheckSum( std::vector<uint8_t1> vtFrameBuffer )
{
	size_t nDataSize = vtFrameBuffer.size();
	uint8_t1 DataSum = 0x00;
	for (int nDataNo = 0; nDataNo < nDataSize - 1; nDataNo++)
	{
		DataSum += vtFrameBuffer[nDataNo];
	}
	DataSum = 0xFF -  DataSum + 0x01;
	if (DataSum == vtFrameBuffer[nDataSize - 1])
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

void CAbsoluteEncoder::GetDataCheckSum( std::vector<uint8_t1> vtFrameBuffer, uint8_t1 &nDataCheckSum )
{
	size_t nDataSize = vtFrameBuffer.size();
	uint8_t1 DataSum = 0x00;
	for (int nDataNo = 0; nDataNo < nDataSize; nDataNo++)
	{
		DataSum = DataSum + vtFrameBuffer[nDataNo];
	}
	DataSum = 0xFF -  DataSum + 0x01;
	nDataCheckSum = DataSum;
}

void CAbsoluteEncoder::WriteEncodderLog(char *format, ... )
{
    va_list args;
    va_start( args, format );
    char s[1000];
    vsprintf(s, format, args);
    va_end(args);   
    CString str;
    str.Format("%s", s);
    CStdioFile AbsoluteEncoderLogFile;
	CString sTime;
	CTime cSystemTime = CTime::GetCurrentTime();
	sTime.Format("%d.%d.%d	%d:%d:%d	", cSystemTime.GetYear(), cSystemTime.GetMonth(), cSystemTime.GetDay(), cSystemTime.GetHour(), cSystemTime.GetMinute(), cSystemTime.GetSecond());
	CString sFilePath = "Monitor\\AbsoluteEncoderLog";
	if (!PathIsDirectory(sFilePath))
	{
		CreateDirectory(sFilePath, NULL);
	}
	CString sFileName;
	sFileName.Format("Monitor\\AbsoluteEncoderLog\\EncoderLog.txt");
	if (!AbsoluteEncoderLogFile.Open(sFileName, CFile::modeWrite))
	{
		if (!AbsoluteEncoderLogFile.Open(sFileName, CFile::modeCreate))
		{
			return;
		}
		else
		{
			AbsoluteEncoderLogFile.Close();
			if (!AbsoluteEncoderLogFile.Open(sFileName, CFile::modeWrite))
			{
				return;
			}
		}
	}
	AbsoluteEncoderLogFile.SeekToEnd();
	AbsoluteEncoderLogFile.WriteString("\r\n" + sTime + str);
	AbsoluteEncoderLogFile.Close();	
}

BOOL CAbsoluteEncoder::GetAbsoluteEncoderData( int nAxisNo, long &lAbsCoorLap,  long &lAbsCoorManyLap)
{
	if (m_bPortSoftEnable == FALSE)
	{
		return FALSE;
	}
	std::vector<uint8_t1> vsReceptionData;
	T_ENCODER_STATE_TAB tEncoderState;
	BOOL bRet = TRUE;
	int nSetNum = 0;
	int nErrorNum = 0;
Begin:
	bRet = TRUE;
	bRet = bRet && Handshake(nAxisNo);
	COMMUNICATION_ERROR_REPETITION;
	COMMUNICATION_ERROR_RETURN;
	bRet = bRet && ApplicationData(nAxisNo);
	COMMUNICATION_ERROR_REPETITION;
	COMMUNICATION_ERROR_RETURN;
	for (nSetNum = 0; nSetNum < 2; nSetNum++)
	{
		bRet = TRUE;
		bRet = bRet && AllowSendData(nAxisNo);
		COMMUNICATION_ERROR_REPETITION;
		COMMUNICATION_ERROR_RETURN;
		bRet = bRet && ReceiveData(nAxisNo, vsReceptionData);
		if (bRet == TRUE)
		{
			break;
		}
	}
	COMMUNICATION_ERROR_RETURN;
	bRet = bRet && CheckEncoderState(vsReceptionData, tEncoderState);
	PrintEncoderError(tEncoderState);

	uint32_t1 AbsCoorLap;
	uint32_t1 AbsCoorManyLap;
	int32_t1 AbsCoorManyLapOver;
	AbsCoorLap = vsReceptionData[9] * 0x10000 + vsReceptionData[8] * 0x100 + vsReceptionData[7];
	AbsCoorManyLap = vsReceptionData[11] * 0x100 + vsReceptionData[10];
	if ((AbsCoorManyLap >= 32768) && (AbsCoorManyLap <= 65535))
	{
		AbsCoorManyLap -= 65536;
		AbsCoorManyLapOver = (int32_t1)AbsCoorManyLap;
		lAbsCoorManyLap = AbsCoorManyLapOver;
	}
	else
	{
		lAbsCoorManyLap = AbsCoorManyLap;
	}
	lAbsCoorLap = AbsCoorLap;
	return bRet;
}

BOOL CAbsoluteEncoder::CheckEncoderState( std::vector<uint8_t1> vtFrameBuffer, T_ENCODER_STATE_TAB &tEncoderState )
{
	uint8_t1 EncoderStateH = vtFrameBuffer[6];
	uint8_t1 EncoderStateL = vtFrameBuffer[5];
	
	BOOL bHBit7 = (EncoderStateH >> 7) & 0x01;
	BOOL bHBit6 = (EncoderStateH >> 6) & 0x01;
	BOOL bHBit5 = (EncoderStateH >> 5) & 0x01;
	BOOL bHBit4 = (EncoderStateH >> 4) & 0x01;
	BOOL bHBit3 = (EncoderStateH >> 3) & 0x01;
	BOOL bHBit2 = (EncoderStateH >> 2) & 0x01;
	BOOL bHBit1 = (EncoderStateH >> 1) & 0x01;
	BOOL bHBit0 = (EncoderStateH >> 0) & 0x01;

	BOOL bLBit7 = (EncoderStateL >> 7) & 0x01;
	BOOL bLBit6 = (EncoderStateL >> 6) & 0x01;
	BOOL bLBit5 = (EncoderStateL >> 5) & 0x01;
	BOOL bLBit4 = (EncoderStateL >> 4) & 0x01;
	BOOL bLBit3 = (EncoderStateL >> 3) & 0x01;
	BOOL bLBit2 = (EncoderStateL >> 2) & 0x01;
	BOOL bLBit1 = (EncoderStateL >> 1) & 0x01;
	BOOL bLBit0 = (EncoderStateL >> 0) & 0x01;

	tEncoderState.bLBit0 = bLBit0;
	tEncoderState.bLBit1 = bLBit1;
	tEncoderState.bLBit2 = bLBit2;
	tEncoderState.bLBit3 = bLBit3;
	tEncoderState.bLBit5 = bLBit5;
	tEncoderState.bLBit6 = bLBit6;
	tEncoderState.bLBit7 = bLBit7;
	tEncoderState.bHBit4 = bHBit4;
	tEncoderState.bHBit5 = bHBit5;

	if ((bHBit4 == 1) || (bHBit5 == 1))
	{
		return FALSE;
	}
	return TRUE;
}

void CAbsoluteEncoder::PrintEncoderError(T_ENCODER_STATE_TAB tEncoderState)
{
	if (tEncoderState.bHBit4)
	{
		WriteEncodderLog("串口号 %d ，错误码(编号)：无	电池报警", m_nPortNo);
	}
	if (tEncoderState.bHBit5)
	{
		WriteEncodderLog("串口号 %d ，错误码(编号)：无	绝对式系统报警", m_nPortNo);
	}
	if (tEncoderState.bLBit0)
	{
		WriteEncodderLog("电池报警 错误码(编号)：A2	电池警告");
	}
	if (tEncoderState.bLBit1)
	{
		WriteEncodderLog("电池报警 错误码(编号)：Err40.0	绝对式系统故障异常保护");
	}
	if (tEncoderState.bLBit2)
	{
		WriteEncodderLog("多圈报警 错误码(编号)：Err45.0	绝对式多圈计数异常保护");
	}
	if (tEncoderState.bLBit3)
	{
		WriteEncodderLog("计数器溢出 错误码(编号)：Err41.0	绝对式计数溢出异常保护");
	}
	if (tEncoderState.bLBit5)
	{
		WriteEncodderLog("计数报警 错误码(编号)：Err44.0	绝对式单圈计数异常保护");
	}
	if (tEncoderState.bLBit6)
	{
		WriteEncodderLog("全绝对式状态 错误码(编号)：Err47.0	绝对式状态异常保护");
	}
	if (tEncoderState.bLBit7)
	{
		WriteEncodderLog("过速度 错误码(编号)：Err42.0	绝对式过速度异常保护");
	}
	if (tEncoderState.bHBit4)
	{
		AfxMessageBox("重要警报：绝对值编码器电池报警！\r\n详细信息见编码器日志");
	}
	if (tEncoderState.bHBit5)
	{
		AfxMessageBox("重要警报：绝对值编码器报警！\r\n详细信息见编码器日志");
	}
}

double CAbsoluteEncoder::CalCurrentCoor( long lAbsCoorLap, long lAbsCoorManyLap, double dPulseToDis, double dLapPulse)
{
	double dCoor = 0.0;
	if (m_bPortSoftEnable == FALSE)
	{
		return dCoor;
	}
	double dLapToCor = dPulseToDis * dLapPulse;
	if (((lAbsCoorLap > m_lInitLapData) && (lAbsCoorManyLap >= 0)) || 
		((lAbsCoorLap < m_lInitLapData) && (lAbsCoorManyLap >= 1)))
	{
		lAbsCoorLap -= m_lInitLapData;
		dCoor = (double)lAbsCoorManyLap * dLapToCor + dLapToCor * ((double)lAbsCoorLap / (double)LAP_DATA_BASE);
	}
	else if (((lAbsCoorLap < m_lInitLapData) && (lAbsCoorManyLap <= 0)) ||
		((lAbsCoorLap > m_lInitLapData) && (lAbsCoorManyLap <= -1)))
	{
		lAbsCoorLap = m_lInitLapData - lAbsCoorLap;
		dCoor = (double)lAbsCoorManyLap * dLapToCor - dLapToCor * ((double)lAbsCoorLap / (double)LAP_DATA_BASE);
	}
	
	return dCoor;
}

double CAbsoluteEncoder::GetCurrentCoorFromAbsEncoder( double dPulseToDis, int nAxisNo, double dLapPulse, long lInitLapData )
{
	COPini cInitLapData;
	CString sSectionName;
	
	sSectionName.Format("InitLapData_ComPort_%d_Axis_%d", m_nPortNo, nAxisNo);
	cInitLapData.SetFileName(DATA_PATH + "AbsoluteEncoder.ini");
	cInitLapData.SetSectionName("InitLapData");
	
	BOOL bRet = TRUE;
	
	if (lInitLapData == 0)
	{
		if (!cInitLapData.ReadString(sSectionName, &m_lInitLapData))
		{
			CString sErrMessage;
			sErrMessage.Format("第 %d 串口中第 %d 轴绝对值编码器初始单圈数据缺失！", m_nPortNo, nAxisNo);
			AfxMessageBox(sErrMessage);
		}
	}
	else 
	{
		m_lInitLapData = lInitLapData;
	}
	cInitLapData.WriteString(sSectionName, m_lInitLapData);

	long lLapData = 0;
	long lManyLapData = 0;
	double dCurrentAbsCoor1 = 0.0;
	double dCurrentAbsCoor2 = 0.0;
	for (int nNum = 0; nNum < ALLOW_REPETITION_NUM; nNum++)
	{
		//bRet = GetAbsoluteEncoderData(nAxisNo, lLapData, lManyLapData);
		//COMMUNICATION_ERROR_CONTINUE;
		//dCurrentAbsCoor1 = CalCurrentCoor(lLapData, lManyLapData, dPulseToDis, dLapPulse);
		lLapData = 0;
		lManyLapData = 0;
		bRet = GetAbsoluteEncoderData(nAxisNo, lLapData, lManyLapData);
		COMMUNICATION_ERROR_CONTINUE;
		dCurrentAbsCoor2 = CalCurrentCoor(lLapData, lManyLapData, dPulseToDis, dLapPulse);
		//if (IsEqual(dCurrentAbsCoor1, dCurrentAbsCoor2, 0.01))
		//{
		//	break;
		//}
	}
	if (FALSE == bRet)
	{
		CString sErrMessage;
		sErrMessage.Format("绝对值编码器读取数据失败，串口号 %d ，详细信息见日志！", m_nPortNo);
		AfxMessageBox(sErrMessage);
	}
	return dCurrentAbsCoor2;
}

BOOL CAbsoluteEncoder::ClearManyLapData(int nAxisNo /*= 0*/)
{
	if (m_bPortSoftEnable == FALSE)
	{
		return FALSE;
	}
	std::vector<uint8_t1> vsReceptionData;
	BOOL bRet = TRUE;
	int nSetNum = 0;
	int nErrorNum = 0;
Begin:
	bRet = TRUE;
	bRet = bRet && Handshake(nAxisNo);
	COMMUNICATION_ERROR_REPETITION;
	COMMUNICATION_ERROR_RETURN;
	bRet = bRet && ApplicationCmd(nAxisNo, 0xb9);
	COMMUNICATION_ERROR_REPETITION;
	COMMUNICATION_ERROR_RETURN;
	for (nSetNum = 0; nSetNum < 2; nSetNum++)
	{
		bRet = TRUE;
		bRet = bRet && AllowSendData(nAxisNo);
		COMMUNICATION_ERROR_REPETITION;
		COMMUNICATION_ERROR_RETURN;
		bRet = bRet && ReceiveData(nAxisNo, vsReceptionData, 5);
		if (bRet == TRUE)
		{
			break;
		}
	}
	return bRet;
}

void CAbsoluteEncoder::ClosePort()
{
	m_AbsEncoderSerial.close();
}

BOOL CAbsoluteEncoder::SetInitLapData( int nAxisNo /*= 0*/ )
{
	long lAbsCoorLap;
	long lAbsCoorManyLap;
	BOOL bRet = GetAbsoluteEncoderData(nAxisNo, lAbsCoorLap, lAbsCoorManyLap);

	COPini cInitLapData;
	CString sSectionName;
	
	sSectionName.Format("InitLapData_ComPort_%d_Axis_%d", m_nPortNo, nAxisNo);
	cInitLapData.SetFileName(DATA_PATH + "AbsoluteEncoder.ini");
	cInitLapData.SetSectionName("InitLapData");
	
	cInitLapData.WriteString(sSectionName, lAbsCoorLap);
	return bRet;
}