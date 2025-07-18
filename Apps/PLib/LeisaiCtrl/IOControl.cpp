// IOControl.cpp: implementation of the CIOControl class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "IOControl.h"
#include ".\OpenClass\FileOP\ini\opini.h"
#include "IOC0640.H"
#include ".\Apps\PLib\BasicFunc\Const.h"
#include "LTDMC.h"

#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
#define new DEBUG_NEW
#endif

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CIOControl::CIOControl()
{
	//郑煤机临时
	COPini opini;
	opini.SetFileName(IO_PARA);
	opini.SetSectionName("Table");
	CString str;
	for (int i = 0; i < 2; i++)
	{
		m_handleTableMove[i] = CreateMutex(NULL, TRUE, NULL);
		ReleaseMutex(m_handleTableMove[i]);
		str.Format("Table%dUpIn", i);
		opini.ReadString(str, &m_nTableUpIn[i]);
		str.Format("Table%dUpOut", i);
		opini.ReadString(str, &m_nTableUpOut[i]);
		str.Format("Table%dDownIn", i);
		opini.ReadString(str, &m_nTableDownIn[i]);
		str.Format("Table%dDownOut", i);
		opini.ReadString(str, &m_nTableDownOut[i]);
		str.Format("Table%dUpInLimit", i);
		opini.ReadString(str, &m_nTableUpInLimit[i]);
		str.Format("Table%dUpInMiddle", i);
		opini.ReadString(str, &m_nTableUpInMiddle[i]);
		str.Format("Table%dUpOutLimit", i);
		opini.ReadString(str, &m_nTableUpOutLimit[i]);
		str.Format("Table%dDownInLimit", i);
		opini.ReadString(str, &m_nTableDownInLimit[i]);
	}
	m_nTableDelayTime = 500;
	m_nTableTimeLimit = 30000;

	m_bIOCCardExists = false;
	m_bDMCCardExists = false;
	m_vstrCardModel.clear();
	m_mapCardModel.clear();
	m_mapCardNum.clear(); 
	m_vstrCardModel.push_back("IOC0640");
	m_vstrCardModel.push_back("IOC1280");
	m_vstrCardModel.push_back("DMC_CAN");
	m_vstrCardModel.push_back("DMC_CAT");
	m_mapCardModel["IOC0640"] = IOC0640;
	m_mapCardModel["IOC1280"] = IOC1280;
	m_mapCardModel["DMC_CAN"] = DMC_CAN;
	m_mapCardModel["DMC_CAT"] = DMC_CAT;

	LoadIO();
	LoadRobotIO();
	LoadIOForBiggerPart();
	//OpenIoCard();

	for (int i = 0; i < m_vtRobotIO.size(); i++)
	{
		OpenRobot(i);
		OpenCallJob(i);
		ResetExRobotEmg(i);
		OpenSupLED(i);
		OpenAirSolenoidValue(i);
		OpenPlamsa(i);	
	}	
	OpenPanoramaCamera();	

	OpenCoolingForPolisher();
	//郑煤机临时
	WriteOutbit(34, ON);
	WriteOutbit(61, ON);
}

CIOControl::~CIOControl()
{
	CloseCoolingForPolisher();

	ClosePanoramaCamera();
	
	for (int i = 0; i < m_vtRobotIO.size(); i++)
	{
		CloseSupLED(i);
		ClosePlamsa(i);
		CloseAirSolenoidValue(i);
		SetExRobotEmg(i);
		CloseCallJob(i);
		CloseRobot(i);
	}

	CloseIoCard();	
}

void CIOControl::OpenIoCard()
{
	std::vector<int> vnCardModel;
	vnCardModel.clear();
	int nCardNum = 0;

	//初始化IO卡
	if (m_bIOCCardExists)
	{
		nCardNum += ioc_board_init();
	}
	if (m_bDMCCardExists)
	{
		nCardNum += dmc_board_init();
	}
	if (nCardNum <= 0)
	{
		XiMessageBox("Error: 初始化IO卡失败！");
	}
	else
	{
		if (nCardNum != m_vtCardPara.size())
		{
			XiMessageBox("Error: 初始化IO卡数量与配置文件定义数量不匹配！");
		}
	}

	//初始化参数
	for (int i = 0; i < m_vtCardPara.size(); i++)
	{
		switch (m_vtCardPara[i].nCardModel)
		{
		case IOC0640:
		{
			ioc_set_filter(m_vtCardPara[i].nHardwareCardNo, m_vtCardPara[i].nIOCFILTER);
			break;
		}
		case IOC1280:
		{
			ioc_set_filter(m_vtCardPara[i].nHardwareCardNo, m_vtCardPara[i].nIOCFILTER);
			break;
		}
		case DMC_CAN:
		{
			nmc_set_connect_state(m_vtCardPara[i].nHardwareCardNo, m_vtCardPara[i].nCANCount, 1, m_vtCardPara[i].nCANBaud);
			Sleep(20);
			WORD wNodeNum;
			WORD wState;
			nmc_get_connect_state(m_vtCardPara[i].nHardwareCardNo, &wNodeNum, &wState);
			if (wState != 1)
			{
				XiMessageBox("Error:初始化CAN失败！");
			}
			if (wNodeNum != m_vtCardPara[i].nCANCount)
			{
				XiMessageBox("Error:初始化的CAN数量与设置不符！");
			}
			WORD TotalIn;
			WORD TotalOut;
			nmc_get_total_ionum(m_vtCardPara[i].nHardwareCardNo, &TotalIn, &TotalOut);
			if (m_vtCardPara[i].nMaxIONum != TotalIn)
			{
				//XiMessageBox("ERROR: 设定的最大IO编号不符合实际！");
			}
			break;
		}
		case DMC_CAT:
		{
			nmc_set_cycletime(m_vtCardPara[i].nHardwareCardNo, m_vtCardPara[i].nHardwareCardNo, m_vtCardPara[i].nCATCycleTime);
			Sleep(20);
			WORD wNodeNum = 0;
			WORD wState = 0;
			nmc_get_connect_state(m_vtCardPara[i].nHardwareCardNo, &wNodeNum, &wState);//这个函数有问题
			if (wState != 1)
			{
				//XiMessageBox("Error:初始化CAN失败！");
			}
			WORD TotalIn;
			WORD TotalOut;
			dmc_get_total_ionum(m_vtCardPara[i].nHardwareCardNo, &TotalIn, &TotalOut);
			m_vtCardPara[i].nCATStartIONo = TotalIn;
			nmc_get_total_ionum(m_vtCardPara[i].nHardwareCardNo, &TotalIn, &TotalOut);
			if ((m_vtCardPara[i].nSoftwareCardNoEnd - m_vtCardPara[i].nSoftwareCardNoStrat + 1) *(m_vtCardPara[i].nMaxIONum + 1) != TotalIn)
			{
				XiMessageBox("ERROR: 设定的最大IO编号或软件卡号不符合实际！");
			}
			m_vtCardPara[i].nMaxIONum = (m_vtCardPara[i].nSoftwareCardNoEnd - m_vtCardPara[i].nSoftwareCardNoStrat + 1)*(m_vtCardPara[i].nMaxIONum + 1) + 7;
			break;
		}
		default:XiMessageBox("ERROR: 无效的IO卡类型！"); break;
		}
	}
}

void CIOControl::CloseIoCard()
{
	//先将需要特殊处理的处理了
	for (int i = 0; i < m_vtCardPara.size(); i++)
	{
		switch (m_vtCardPara[i].nCardModel)
		{
		case IOC0640:
		{
			break;
		}
		case IOC1280:
		{
			break;
		}
		case DMC_CAN:
		{
			Sleep(100);
			nmc_set_connect_state(m_vtCardPara[i].nHardwareCardNo, m_vtCardPara[i].nCANCount, 0, m_vtCardPara[i].nCANBaud);			
			break;
		}
		case DMC_CAT:
		{
			break;
		}
		default:XiMessageBox("ERROR: 无效的IO卡类型！"); break;
		}
	}

	//关闭控制卡
	if (m_bIOCCardExists)
	{
		ioc_board_close();
	}
	if (m_bDMCCardExists)
	{
		dmc_board_close();
	}
}

DWORD CIOControl::ReadInbit(WORD bitno)
{
	if (bitno >= INVALID_IO_NUM || bitno < 0)
	{
		return -1;
	}
	//计算软件卡号
	WORD wSoftwareCardNo = bitno / 100;
	//计算软件IO口号
	WORD wSoftwareBitNo = bitno % 100;
	//判定软件卡号是否存在
	std::map<int, int>::iterator it;
	it = m_mapCardNum.find(wSoftwareCardNo);
	if (it == m_mapCardNum.end())
	{
		return -1;
	}
	//获取硬件卡号
	WORD wCardNo = m_vtCardPara[m_mapCardNum[wSoftwareCardNo]].nHardwareCardNo;
	//计算节点编号
	WORD wNoteID = wSoftwareCardNo - m_vtCardPara[m_mapCardNum[wSoftwareCardNo]].nSoftwareCardNoStrat;

	switch (m_vtCardPara[m_mapCardNum[wSoftwareCardNo]].nCardModel)
	{
	case IOC0640:
	{
		if (wSoftwareBitNo > m_vtCardPara[m_mapCardNum[wSoftwareCardNo]].nMaxIONum)
		{
			return -1;
		}
		return ioc_read_inbit(wCardNo, wSoftwareBitNo);
	}
	case IOC1280:
	{
		if (wSoftwareBitNo > m_vtCardPara[m_mapCardNum[wSoftwareCardNo]].nMaxIONum)
		{
			return -1;
		}
		return ioc_read_inbit(wCardNo, wSoftwareBitNo);
	}
	case DMC_CAN:
	{
		if (wSoftwareBitNo > m_vtCardPara[m_mapCardNum[wSoftwareCardNo]].nMaxIONum)
		{
			return -1;
		}
		WORD wReturn = OFF;
		nmc_read_inbit(wCardNo, wNoteID + 1, wSoftwareBitNo, &wReturn);
		return wReturn;
	}
	case DMC_CAT:
	{
		WORD wBitNo = m_vtCardPara[m_mapCardNum[wSoftwareCardNo]].nCATStartIONo + wNoteID * (m_vtCardPara[m_mapCardNum[wSoftwareCardNo]].nMaxIONum + 1) + wSoftwareBitNo;
		if (wBitNo > m_vtCardPara[m_mapCardNum[wSoftwareCardNo]].nMaxIONum)
		{
			return -1;
		}
		return dmc_read_inbit(wCardNo, wBitNo);
	}
	default:
	{
		XiMessageBox("ERROR: 无效的IO卡类型！");
		return OFF;
	}
	}
}

DWORD CIOControl::ReadOutbit(WORD bitno)
{
	if (bitno >= INVALID_IO_NUM || bitno < 0)
	{
		return -1;
	}
	//计算软件卡号
	WORD wSoftwareCardNo = bitno / 100;
	//计算软件IO口号
	WORD wSoftwareBitNo = bitno % 100;
	//判定软件卡号是否存在
	std::map<int, int>::iterator it;
	it = m_mapCardNum.find(wSoftwareCardNo);
	if (it == m_mapCardNum.end())
	{
		return -1;
	}
	//获取硬件卡号
	WORD wCardNo = m_vtCardPara[m_mapCardNum[wSoftwareCardNo]].nHardwareCardNo;
	//计算节点编号
	WORD wNoteID = wSoftwareCardNo - m_vtCardPara[m_mapCardNum[wSoftwareCardNo]].nSoftwareCardNoStrat;

	switch (m_vtCardPara[m_mapCardNum[wSoftwareCardNo]].nCardModel)
	{
	case IOC0640:
	{
		if (wSoftwareBitNo > m_vtCardPara[m_mapCardNum[wSoftwareCardNo]].nMaxIONum)
		{
			return -1;
		}
		return ioc_read_outbit(wCardNo, wSoftwareBitNo);
	}
	case IOC1280:
	{
		if (wSoftwareBitNo > m_vtCardPara[m_mapCardNum[wSoftwareCardNo]].nMaxIONum)
		{
			return -1;
		}
		return ioc_read_outbit(wCardNo, wSoftwareBitNo);
	}
	case DMC_CAN:
	{
		if (wSoftwareBitNo > m_vtCardPara[m_mapCardNum[wSoftwareCardNo]].nMaxIONum)
		{
			return -1;
		}
		WORD wReturn = OFF;
		nmc_read_outbit(wCardNo, wNoteID + 1, wSoftwareBitNo, &wReturn);
		return wReturn;
	}
	case DMC_CAT:
	{
		WORD wBitNo = m_vtCardPara[m_mapCardNum[wSoftwareCardNo]].nCATStartIONo + wNoteID * (m_vtCardPara[m_mapCardNum[wSoftwareCardNo]].nMaxIONum + 1) + wSoftwareBitNo;
		if (wBitNo > m_vtCardPara[m_mapCardNum[wSoftwareCardNo]].nMaxIONum)
		{
			return -1;
		}
		return dmc_read_outbit(wCardNo, wBitNo);
	}
	default:
	{
		XiMessageBox("ERROR: 无效的IO卡类型！");
		return OFF;
	}
	}
}

DWORD CIOControl::WriteOutbit(WORD bitno, WORD on_off)
{
	if (bitno >= INVALID_IO_NUM || bitno < 0)
	{
		return -1;
	}
	//计算软件卡号
	WORD wSoftwareCardNo = bitno / 100;
	//计算软件IO口号
	WORD wSoftwareBitNo = bitno % 100;
	//判定软件卡号是否存在
	std::map<int, int>::iterator it;
	it = m_mapCardNum.find(wSoftwareCardNo);
	if (it == m_mapCardNum.end())
	{
		return -1;
	}
	//获取硬件卡号
	WORD wCardNo = m_vtCardPara[m_mapCardNum[wSoftwareCardNo]].nHardwareCardNo;
	//计算节点编号
	WORD wNoteID = wSoftwareCardNo - m_vtCardPara[m_mapCardNum[wSoftwareCardNo]].nSoftwareCardNoStrat;

	switch (m_vtCardPara[m_mapCardNum[wSoftwareCardNo]].nCardModel)
	{
	case IOC0640:
	{
		if (wSoftwareBitNo > m_vtCardPara[m_mapCardNum[wSoftwareCardNo]].nMaxIONum)
		{
			return -1;
		}
		return ioc_write_outbit(wCardNo, wSoftwareBitNo, on_off);
	}
	case IOC1280:
	{
		if (wSoftwareBitNo > m_vtCardPara[m_mapCardNum[wSoftwareCardNo]].nMaxIONum)
		{
			return -1;
		}
		return ioc_write_outbit(wCardNo, wSoftwareBitNo, on_off);
	}
	case DMC_CAN:
	{
		if (wSoftwareBitNo > m_vtCardPara[m_mapCardNum[wSoftwareCardNo]].nMaxIONum)
		{
			return -1;
		}
		return nmc_write_outbit(wCardNo, wNoteID + 1, wSoftwareBitNo, on_off);
	}
	case DMC_CAT:
	{
		WORD wBitNo = m_vtCardPara[m_mapCardNum[wSoftwareCardNo]].nCATStartIONo + wNoteID * (m_vtCardPara[m_mapCardNum[wSoftwareCardNo]].nMaxIONum + 1) + wSoftwareBitNo;
		if (wBitNo > m_vtCardPara[m_mapCardNum[wSoftwareCardNo]].nMaxIONum)
		{
			return -1;
		}
		return dmc_write_outbit(wCardNo, wBitNo, on_off);
	}
	default:
	{
		XiMessageBox("ERROR: 无效的IO卡类型！");
		return OFF;
	}
	}
}

DWORD CIOControl::OpenPlamsa(int nRobotNo)
{
	return WriteOutbit(m_vtRobotIO[nRobotNo].tCuttingRobotIO.nPlamsaPowerIO, ON);
}

DWORD CIOControl::ClosePlamsa(int nRobotNo)
{
	return WriteOutbit(m_vtRobotIO[nRobotNo].tCuttingRobotIO.nPlamsaPowerIO, OFF);
}

DWORD CIOControl::StartPlamsaArc(int nRobotNo)
{
	return WriteOutbit(m_vtRobotIO[nRobotNo].tCuttingRobotIO.nPlamsaArcIO, ON);
}

DWORD CIOControl::StopPlamsaArc(int nRobotNo)
{
	return WriteOutbit(m_vtRobotIO[nRobotNo].tCuttingRobotIO.nPlamsaArcIO, OFF);
}

bool CIOControl::ReadPlamsaArc(int nRobotNo)
{
	if (ON == ReadOutbit(m_vtRobotIO[nRobotNo].tCuttingRobotIO.nPlamsaArcIO))
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool CIOControl::PlamsaArcFeedback(int nRobotNo)
{
	if (ON == ReadInbit(m_vtRobotIO[nRobotNo].tCuttingRobotIO.nArcFeedbackIO))
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool CIOControl::SmallArcFeedback(int nRobotNo)
{
	if (ON == ReadInbit(m_vtRobotIO[nRobotNo].tCuttingRobotIO.nSmallArcFeedbackIO))
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool CIOControl::TotalEmg(bool bStopArc, bool bPopup)
{
	//逻辑被改，不同设备不同！！！
	if (ON == ReadInbit(m_nTotalEmgIO))
	{
		if (bStopArc)
		{
			for (int i = 0; i < m_vtRobotIO.size(); i++)
			{
				StopPlamsaArc(i);
			}	
			ClosePolisher();
		}
		if (bPopup)
		{
			XiMessageBox("Warning:RobotEmg():机械臂已急停！");
		}		
		return true;
	}
	else
	{
		return false;
	}
}

void CIOControl::ResetExRobotEmg(int nRobotNo)
{
	WriteOutbit(m_vtRobotIO[nRobotNo].tYaskawaRobotIO.tRobotOutputIO.nRobotExEmgIO, ON);
}

void CIOControl::SetExRobotEmg(int nRobotNo)
{
	WriteOutbit(m_vtRobotIO[nRobotNo].tYaskawaRobotIO.tRobotOutputIO.nRobotExEmgIO, OFF);
}

bool CIOControl::CheckRobotMoveSafe()
{
	if (ON == ReadInbit(m_nRobotMoveSafeIO))
	{
		return false;
	}
	return true;
}

void CIOControl::Demagnetizing()
{
	WriteOutbit(m_nDemagnetizingIO, ON);
	UnlockMagnet();
	Sleep(700);
	WriteOutbit(m_nDemagnetizingIO, OFF);
	LockMagnet();
	Sleep(500);
}

bool CIOControl::GrabSuccess(int nNum)
{
	if (nNum == 1)
	{
		if (ON == ReadInbit(m_nMagnat1GrabSuccessIO))
		{
			return true;
		}
		else
		{
			return false;
		}
	} 
	else if(nNum == 2)
	{
		if (ON == ReadInbit(m_nMagnat2GrabSuccessIO))
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	else
	{
		return false;
	}
}

bool CIOControl::MagnetizingSuccess()
{
	if (ON == ReadInbit(m_nMagnetizingSuccessIO))
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool CIOControl::DemagnetizingSuccess()
{
	if (ON == ReadInbit(m_nDemagnetizingSuccessIO))
	{
		return true;
	}
	else
	{
		return false;
	}
}

void CIOControl::Magnetizing(int nLevel)
{
	if (nLevel < 1)
	{
		nLevel = 1;
	} 
	else if(nLevel > 6)
	{
		nLevel = 6;
	}
	switch (nLevel)
	{
	case 1:LockMagnet(); MagnetizingLv1();
		break;
	case 2:UnlockMagnet(); MagnetizingLv1(); LockMagnet();
		break;
	case 3:LockMagnet(); MagnetizingLv2();
		break;
	case 4:UnlockMagnet(); MagnetizingLv2(); LockMagnet();
		break;
	case 5:LockMagnet(); MagnetizingLv3();
		break;
	case 6:UnlockMagnet(); MagnetizingLv3(); LockMagnet();
		break;
	default:
		break;
	}
}

void CIOControl::LockMagnet()
{
	WriteOutbit(m_nMagnetUnlockIO, OFF);
}

void CIOControl::UnlockMagnet()
{
	WriteOutbit(m_nMagnetUnlockIO, ON);
}

void CIOControl::MagnetizingLv1()
{
	WriteOutbit(m_nMagnetizingLv1IO, ON);
	Sleep(m_nMagnetizingDelayTime);
	WriteOutbit(m_nMagnetizingLv1IO, OFF);
}

void CIOControl::MagnetizingLv2()
{
	WriteOutbit(m_nMagnetizingLv2IO, ON);
	Sleep(m_nMagnetizingDelayTime);
	WriteOutbit(m_nMagnetizingLv2IO, OFF);
}

void CIOControl::MagnetizingLv3()
{
	WriteOutbit(m_nMagnetizingLv3IO, ON);
	Sleep(m_nMagnetizingDelayTime);
	WriteOutbit(m_nMagnetizingLv3IO, OFF);
}

void CIOControl::OpenCoolingForPolisher()
{
	WriteOutbit(m_nCoolingForPolisher, ON);
}

void CIOControl::CloseCoolingForPolisher()
{
	WriteOutbit(m_nCoolingForPolisher, OFF);
}

void CIOControl::OpenPolisher()
{
	WriteOutbit(m_nOpenPolisher, ON);
}

void CIOControl::ClosePolisher()
{
	WriteOutbit(m_nOpenPolisher, OFF);
}

void CIOControl::OpenMagnetForPolish()
{
	WriteOutbit(m_nMagnetForPolish, ON);
}

void CIOControl::CloseMagnetForPolish()
{
	WriteOutbit(m_nMagnetForPolish, OFF);
}

void CIOControl::PlatformMagnetizing()
{
	WriteOutbit(m_nPlatformMagnatIO, ON);
}

void CIOControl::PlatformDemagnetizing()
{
	WriteOutbit(m_nPlatformMagnatIO, OFF);
}

void CIOControl::OpenMagnet2()
{
	WriteOutbit(m_nMagnet2ControlIO, ON);
}

void CIOControl::CloseMagnet2()
{
	WriteOutbit(m_nMagnet2ControlIO, OFF);
}

bool CIOControl::Door1Close()
{
	if (ON == ReadInbit(m_nDoor1CloseIO))
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool CIOControl::Door2Close()
{
	if (ON == ReadInbit(m_nDoor2CloseIO))
	{
		return true;
	}
	else
	{
		return false;
	}
}

void CIOControl::OpenDoor1()
{
	WriteOutbit(m_nDoor1ControlIO, ON);
}

void CIOControl::OpenDoor2()
{
	WriteOutbit(m_nDoor2ControlIO, ON);
}

void CIOControl::CloseDoor1()
{
	WriteOutbit(m_nDoor1ControlIO, OFF);
}

void CIOControl::CloseDoor2()
{
	WriteOutbit(m_nDoor2ControlIO, OFF);
}

bool CIOControl::PlatformUp()
{
	if (ON == ReadInbit(m_nPlatformUpLimitIO))
	{
		return true;
	}
	else
	{
		return false;
	}	
}

bool CIOControl::PlatformDown()
{
	if (ON == ReadInbit(m_nPlatformDownLimitIO))
	{
		return true;
	}
	else
	{
		return false;
	}
}

void CIOControl::PlatformControl(bool bUpOrDown)
{
	if (bUpOrDown)
	{
		WriteOutbit(m_nUpOrDownIO, ON);
	} 
	else
	{
		WriteOutbit(m_nUpOrDownIO, OFF);
	}
}

void CIOControl::OpenLaser(int nRobotNo)
{
	if (OFF == ReadOutbit(m_vtRobotIO[nRobotNo].tCuttingRobotIO.nLaserIO))
	{
		WriteOutbit(m_vtRobotIO[nRobotNo].tCuttingRobotIO.nLaserIO, ON);
	}
}

void CIOControl::CloseLaser(int nRobotNo)
{
	WriteOutbit(m_vtRobotIO[nRobotNo].tCuttingRobotIO.nLaserIO, OFF);
}

void CIOControl::OpenAirconditioner()
{
	WriteOutbit(m_nAirconditionerIO, ON);
}

void CIOControl::CloseAirconditioner()
{
	WriteOutbit(m_nAirconditionerIO, OFF);
}

void CIOControl::OpenDustRemoval()
{
	WriteOutbit(m_nDustRemovalIO, ON);
}

void CIOControl::CloseDustRemoval()
{
	WriteOutbit(m_nDustRemovalIO, OFF);
}

void CIOControl::OpenWarningLights()
{
	WriteOutbit(m_nWarningLightsIO, ON);
}

void CIOControl::CloseWarningLights()
{
	WriteOutbit(m_nWarningLightsIO, OFF);
}

void CIOControl::OpenPanoramaCamera()
{
	WriteOutbit(m_nPanoramaCameraIO, ON);
}

void CIOControl::ClosePanoramaCamera()
{
	WriteOutbit(m_nPanoramaCameraIO, OFF);
}

void CIOControl::OpenExCamera()
{
	WriteOutbit(m_nExCameraIO, ON);
}

void CIOControl::CloseExCamera()
{
	WriteOutbit(m_nExCameraIO, OFF);
}

void CIOControl::LoadIOForBiggerPart()
{
	COPini opini;
	opini.SetFileName(IO_PARA);
	opini.SetSectionName("INPUT");
	opini.ReadString("Platform1CorotationDecIO", &m_nPlatform1CorotationDecIO);
	opini.ReadString("Platform1CorotationStopIO", &m_nPlatform1CorotationStopIO);
	opini.ReadString("Platform2CorotationDecIO", &m_nPlatform2CorotationDecIO);
	opini.ReadString("Platform2CorotationStopIO", &m_nPlatform2CorotationStopIO);
	opini.ReadString("Transducer1ClosedIO", &m_nTransducer1ClosedIO);
	opini.ReadString("Transducer2ClosedIO", &m_nTransducer2ClosedIO);
	opini.ReadString("Platform1CorotationStartIO", &m_nPlatform1CorotationStartIO); 
	opini.ReadString("Platform2CorotationStartIO", &m_nPlatform2CorotationStartIO);
	opini.ReadString("LightCurtain1IO", &m_nLightCurtain1IO);
	opini.ReadString("LightCurtain2IO", &m_nLightCurtain2IO);
	opini.ReadString("Platform1MoveIO", &m_nPlatform1MoveIO);
	opini.ReadString("Platform2MoveIO", &m_nPlatform2MoveIO);

	opini.SetSectionName("OUTPUT");
	opini.ReadString("Platform1CorotationIO", &m_nPlatform1CorotationIO);
	opini.ReadString("Platform1ReversalIO", &m_nPlatform1ReversalIO);
	opini.ReadString("Platform2CorotationIO", &m_nPlatform2CorotationIO);
	opini.ReadString("Platform2ReversalIO", &m_nPlatform2ReversalIO);
	opini.ReadString("Transducer1OpenIO", &m_nTransducer1OpenIO);
	opini.ReadString("Transducer2OpenIO", &m_nTransducer2OpenIO);
}

bool CIOControl::StartPlatform(int nPlatformNo)
{
	int nPlatformCorotationIO;
	if (nPlatformNo == 1)
	{
		nPlatformCorotationIO = m_nPlatform1CorotationIO;
	} 
	else
	{
		nPlatformCorotationIO = m_nPlatform2CorotationIO;
	}
	WriteOutbit(nPlatformCorotationIO, ON);
	Sleep(3000);
	long long time = XI_clock();
	while (!PlatformStart(nPlatformNo))
	{
		Sleep(5);
		if (XI_clock() - time > 6000)
		{
			StopPlatform(nPlatformCorotationIO);
			XUI::MesBox::PopOkCancel("Error:平台{0}启动失败！", nPlatformNo);
			return false;
		}
	}
	return true;
}

void CIOControl::StopPlatform(int nPlatformNo)
{
	int nPlatformCorotationIO;
	if (nPlatformNo == 1)
	{
		nPlatformCorotationIO = m_nPlatform1CorotationIO;
	}
	else
	{
		nPlatformCorotationIO = m_nPlatform2CorotationIO;
	}
	WriteOutbit(nPlatformCorotationIO, OFF);
}

void CIOControl::OpenTransducer()
{
	WriteOutbit(m_nTransducer1OpenIO, ON);
	WriteOutbit(m_nTransducer2OpenIO, ON);
}

void CIOControl::CloseTransducer()
{
	WriteOutbit(m_nTransducer1OpenIO, OFF);
	WriteOutbit(m_nTransducer2OpenIO, OFF);
}

bool CIOControl::GetTransducerState()
{
	if (ON == ReadInbit(m_nTransducer1ClosedIO)
		&& ON == ReadInbit(m_nTransducer2ClosedIO))
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool CIOControl::GetLightCurtainState()
{
	if (ON == ReadInbit(m_nLightCurtain1IO) 
		|| ON == ReadInbit(m_nLightCurtain2IO))
	{
		return true;
	} 
	else
	{
		return false;
	}
}

bool CIOControl::PlatformStart(int nPlatformNo)
{
	int nPlatformMoveIO;
	if (nPlatformNo == 1)
	{
		nPlatformMoveIO = m_nPlatform1MoveIO;
	} 
	else
	{
		nPlatformMoveIO = m_nPlatform2MoveIO;
	}
	if (ON == ReadInbit(nPlatformMoveIO))
	{
		return true;
	} 
	else
	{
		return false;
	}
	return false;
}

bool CIOControl::PlatformDec(int nPlatformNo)
{
	int nPlatformCorotationDecIO;
	if (nPlatformNo == 1)
	{
		nPlatformCorotationDecIO = m_nPlatform1CorotationDecIO;
	}
	else
	{
		nPlatformCorotationDecIO = m_nPlatform2CorotationDecIO;
	}
	if (ON == ReadInbit(nPlatformCorotationDecIO))
	{
		return true;
	}
	else
	{
		return false;
	}
	return false;
}

bool CIOControl::PlatformStop(int nPlatformNo)
{
	int nPlatformCorotationStopIO;
	if (nPlatformNo == 1)
	{
		nPlatformCorotationStopIO = m_nPlatform1CorotationStopIO;
	}
	else
	{
		nPlatformCorotationStopIO = m_nPlatform2CorotationStopIO;
	}
	if (ON == ReadInbit(nPlatformCorotationStopIO))
	{
		return true;
	}
	else
	{
		return false;
	}
	return false;
}

bool CIOControl::TableUpIn(int nTableNo)
{
	WaitForSingleObject(m_handleTableMove[nTableNo], INFINITE);
	
	if (CheckForBlanking(nTableNo))
	{
		ReleaseMutex(m_handleTableMove[nTableNo]);
		return true;
	}
	int nLimitCounts = m_nTableTimeLimit / 200;
	int nCounts = 0;
	WriteOutbit(m_nTableUpIn[nTableNo], ON);
	Sleep(m_nTableDelayTime);
	WriteOutbit(m_nTableUpIn[nTableNo], OFF);
	while (ReadInbit(m_nTableUpInMiddle[nTableNo]) == OFF)
	{
		Sleep(200);
		nCounts++;
		if (CheckForBlanking(nTableNo))
		{
			ReleaseMutex(m_handleTableMove[nTableNo]);
			return true;
		}
		if (nCounts > nLimitCounts)
		{
			ReleaseMutex(m_handleTableMove[nTableNo]);
			return false;
		}
	}
	Sleep(1000);
	WriteOutbit(m_nTableUpIn[nTableNo], ON);
	Sleep(m_nTableDelayTime);
	WriteOutbit(m_nTableUpIn[nTableNo], OFF);
	nCounts = 0;
	while (ReadInbit(m_nTableUpInLimit[nTableNo]) == OFF)
	{
		Sleep(200);
		nCounts++;
		if (nCounts > nLimitCounts)
		{
			ReleaseMutex(m_handleTableMove[nTableNo]);
			return false;
		}
	}
	if (!CheckForBlanking(nTableNo))
	{
		ReleaseMutex(m_handleTableMove[nTableNo]);
		return false;
	}

	ReleaseMutex(m_handleTableMove[nTableNo]);
	return true;
}

bool CIOControl::TableUpOut(int nTableNo)
{
	WaitForSingleObject(m_handleTableMove[nTableNo], INFINITE);

	if (CheckForHandling(nTableNo))
	{
		ReleaseMutex(m_handleTableMove[nTableNo]);
		return true;
	}
	int nLimitCounts = m_nTableTimeLimit / 200;
	int nCounts = 0;
	WriteOutbit(m_nTableUpOut[nTableNo], ON);
	Sleep(m_nTableDelayTime);
	WriteOutbit(m_nTableUpOut[nTableNo], OFF);
	while (ReadInbit(m_nTableUpInMiddle[nTableNo]) == OFF)
	{
		Sleep(200);
		nCounts++;
// 		if (CheckForHandling(nTableNo))
// 		{
// 			ReleaseMutex(m_handleTableMove[nTableNo]);
// 			return true;
// 		}
		if (nCounts > nLimitCounts)
		{
			ReleaseMutex(m_handleTableMove[nTableNo]);
			return false;
		}
	}
	Sleep(1000);
	WriteOutbit(m_nTableUpOut[nTableNo], ON);
	Sleep(m_nTableDelayTime);
	WriteOutbit(m_nTableUpOut[nTableNo], OFF);
	nCounts = 0;
	while (ReadInbit(m_nTableUpOutLimit[nTableNo]) == OFF)
	{
		Sleep(200);
		nCounts++;
		if (nCounts > nLimitCounts)
		{
			ReleaseMutex(m_handleTableMove[nTableNo]);
			return false;
		}
	}
	if (!CheckForHandling(nTableNo))
	{
		ReleaseMutex(m_handleTableMove[nTableNo]);
		return false;
	}

	ReleaseMutex(m_handleTableMove[nTableNo]);
	return true;
}

bool CIOControl::CheckForHandling(int nTableNo)
{
	if (ReadInbit(m_nTableUpOutLimit[nTableNo]) == ON
		&& ReadInbit(m_nTableDownInLimit[nTableNo]) == ON)
	{
		return true;
	}
	return false;
}

bool CIOControl::CheckForBlanking(int nTableNo)
{
	if (ReadInbit(m_nTableUpInLimit[nTableNo]) == ON)
	{
		return true;
	}
	return false;
}

void CIOControl::OpenAirSolenoidValue(int nRobotNo)
{
	WriteOutbit(m_vtRobotIO[nRobotNo].tCuttingRobotIO.nAirSolenoidValueIO, ON);
}

void CIOControl::CloseAirSolenoidValue(int nRobotNo)
{
	WriteOutbit(m_vtRobotIO[nRobotNo].tCuttingRobotIO.nAirSolenoidValueIO, OFF);
}

void CIOControl::OpenSupLED(int nRobotNo)
{
	WriteOutbit(m_vtRobotIO[nRobotNo].tCuttingRobotIO.nSupLEDIO, ON);
	//OpenSupLEDCtrlSignalPower();
}

void CIOControl::CloseSupLED(int nRobotNo)
{
	//CloseSupLEDCtrlSignalPower();
	WriteOutbit(m_vtRobotIO[nRobotNo].tCuttingRobotIO.nSupLEDIO, OFF);
}

void CIOControl::OpenSupLEDCtrlSignalPower(int nRobotNo)
{
	WriteOutbit(m_vtRobotIO[nRobotNo].tCuttingRobotIO.nSupLED1IO, ON);
	WriteOutbit(m_vtRobotIO[nRobotNo].tCuttingRobotIO.nSupLED2IO, ON);
}

void CIOControl::CloseSupLEDCtrlSignalPower(int nRobotNo)
{
	WriteOutbit(m_vtRobotIO[nRobotNo].tCuttingRobotIO.nSupLED1IO, OFF);
	WriteOutbit(m_vtRobotIO[nRobotNo].tCuttingRobotIO.nSupLED2IO, OFF);
}

void CIOControl::OpenAntisplash(int nRobotNo)
{
	WriteOutbit(m_vtRobotIO[nRobotNo].tCuttingRobotIO.nAntisplashIO, ON);
}

void CIOControl::CloseAntisplash(int nRobotNo)
{
	WriteOutbit(m_vtRobotIO[nRobotNo].tCuttingRobotIO.nAntisplashIO, OFF);
}

bool CIOControl::GetAntisplashState(int nRobotNo, bool &bState)
{
	DWORD nOpened = ReadInbit(m_vtRobotIO[nRobotNo].tCuttingRobotIO.nAntisplashOpenedIO);
	DWORD nClosed = ReadInbit(m_vtRobotIO[nRobotNo].tCuttingRobotIO.nAntisplashClosedIO);
	if (nOpened + nClosed != 1)
	{
		bState = false;
		return false;
	}
	if (nOpened == ON)
	{
		bState = true;
	} 
	else
	{
		bState = false;
	}
	return true;
}

void CIOControl::LoadIO()
{
	COPini opini;
    BOOL bRet = TRUE;
	opini.SetFileName(IO_PARA);
	
	CString strSectionName = 0;
	CString strCardModel = 0;
	int nCardNo = 0;
	m_vtCardPara.clear();
	strSectionName.Format("Card%d", nCardNo);
	while (opini.CheckExists(IO_PARA, strSectionName, "CardModel"))
	{
		T_CARD_PARA tCardPara;
		tCardPara.nCardModel = 0;
		tCardPara.nHardwareCardNo = 0;
		tCardPara.nMaxIONum = 0;
		tCardPara.nIOCFILTER = 0;
		tCardPara.nCANCount = 0;
		tCardPara.nCANBaud = 0;
		tCardPara.nHardwareCardNo = 0;
		tCardPara.nCATCycleTime = 0;
		
		opini.SetSectionName(strSectionName);
		bRet = bRet && opini.ReadString("CardModel", strCardModel);
		tCardPara.nCardModel = m_mapCardModel.at(strCardModel);
		bRet = bRet && opini.ReadString("HardwareCardNo", &tCardPara.nHardwareCardNo);
		bRet = bRet && opini.ReadString("SoftwareCardNoStrat", &tCardPara.nSoftwareCardNoStrat);
		bRet = bRet && opini.ReadString("SoftwareCardNoEnd", &tCardPara.nSoftwareCardNoEnd);
		bRet = bRet && opini.ReadString("MaxIONum", &tCardPara.nMaxIONum);

		for (int i = tCardPara.nSoftwareCardNoStrat; i <= tCardPara.nSoftwareCardNoEnd; i++)
		{
			m_mapCardNum[i] = m_vtCardPara.size();
		}

		switch (tCardPara.nCardModel)
		{
		case IOC0640:
		{
			bRet = bRet && opini.ReadString("IOCFILTER", &tCardPara.nIOCFILTER);
			m_bIOCCardExists = true;
			break;
		}
		case IOC1280:
		{
			bRet = bRet && opini.ReadString("IOCFILTER", &tCardPara.nIOCFILTER);
			m_bIOCCardExists = true;
			break;
		}
		case DMC_CAN:
		{
			bRet = bRet && opini.ReadString("CANCount", &tCardPara.nCANCount);
			bRet = bRet && opini.ReadString("CANBaud", &tCardPara.nCANBaud);
			m_bDMCCardExists = true;
			break;
		}
		case DMC_CAT:
		{
			bRet = bRet && opini.ReadString("CATCycleTime", &tCardPara.nCATCycleTime);
			m_bDMCCardExists = true;
			break;
		}
		default:XiMessageBox("ERROR: 无效的IO卡类型！"); break;
		}

		m_vtCardPara.push_back(tCardPara);
		nCardNo++;
		strSectionName.Format("Card%d", nCardNo);
	}	

	opini.SetSectionName("INPUT");	 
	bRet = bRet && opini.ReadString("TotalEmgIO", &m_nTotalEmgIO);	
	bRet = bRet && opini.ReadString("RobotMoveSafeIO", &m_nRobotMoveSafeIO);
	bRet = bRet && opini.ReadString("Magnat1GrabSuccessIO", &m_nMagnat1GrabSuccessIO);
	bRet = bRet && opini.ReadString("Magnat2GrabSuccessIO", &m_nMagnat2GrabSuccessIO);
	bRet = bRet && opini.ReadString("MagnetizingSuccessIO", &m_nMagnetizingSuccessIO);
	bRet = bRet && opini.ReadString("DemagnetizingSuccessIO", &m_nDemagnetizingSuccessIO);
	bRet = bRet && opini.ReadString("Door1CloseIO", &m_nDoor1CloseIO);
	bRet = bRet && opini.ReadString("Door2CloseIO", &m_nDoor2CloseIO);
	bRet = bRet && opini.ReadString("PlatformUpLimitIO", &m_nPlatformUpLimitIO);
	bRet = bRet && opini.ReadString("PlatformDownLimitIO", &m_nPlatformDownLimitIO);

	opini.SetSectionName("OUTPUT");
	bRet = bRet && opini.ReadString("PlatformMagnatIO", &m_nPlatformMagnatIO);
	bRet = bRet && opini.ReadString("Magnet2ControlIO", &m_nMagnet2ControlIO); 
	bRet = bRet && opini.ReadString("DemagnetizingIO", &m_nDemagnetizingIO);
	bRet = bRet && opini.ReadString("MagnetUnlockIO", &m_nMagnetUnlockIO);
	bRet = bRet && opini.ReadString("MagnetizingLv1IO", &m_nMagnetizingLv1IO);
	bRet = bRet && opini.ReadString("MagnetizingLv2IO", &m_nMagnetizingLv2IO);
	bRet = bRet && opini.ReadString("MagnetizingLv3IO", &m_nMagnetizingLv3IO);
	bRet = bRet && opini.ReadString("Door1ControlIO", &m_nDoor1ControlIO);
	bRet = bRet && opini.ReadString("Door2ControlIO", &m_nDoor2ControlIO);
	bRet = bRet && opini.ReadString("UpOrDownIO", &m_nUpOrDownIO);	
	bRet = bRet && opini.ReadString("AirconditionerIO", &m_nAirconditionerIO);
	bRet = bRet && opini.ReadString("DustRemovalIO", &m_nDustRemovalIO);
	bRet = bRet && opini.ReadString("WarningLightsIO", &m_nWarningLightsIO);	
	bRet = bRet && opini.ReadString("PanoramaCameraIO", &m_nPanoramaCameraIO); 
	bRet = bRet && opini.ReadString("ExCameraIO", &m_nExCameraIO);	
	bRet = bRet && opini.ReadString("CoolingForPolisher", &m_nCoolingForPolisher);
	bRet = bRet && opini.ReadString("OpenPolisher", &m_nOpenPolisher);
	bRet = bRet && opini.ReadString("MagnetForPolish", &m_nMagnetForPolish);

	opini.SetFileName(SYSTEM_PARA_INI);
	opini.SetSectionName("Magnet");
	opini.ReadString("DelayTime", &m_nMagnetizingDelayTime);
	if (m_nMagnetizingDelayTime < 750)
	{
		m_nMagnetizingDelayTime = 750;
	}

	if (FALSE == bRet)
	{
		XiMessageBox("Error:IOPara.ini文件参数加载失败！");
	}

	CheckIOPara();	
}

void CIOControl::CheckIOPara()
{
	//对IO进行一次预先鉴别，影响功能选择的IO自动选择可用的功能
	//接近传感器
	if (!(CheckBitNo(m_nMagnat1GrabSuccessIO) && CheckBitNo(m_nMagnat2GrabSuccessIO)))
	{
		COPini opini2;
		opini2.SetFileName(SYSTEM_PARA_INI);
		opini2.SetSectionName("ProximityTransducer");
	}
	for (int i = 0; i < m_vtRobotIO.size(); i++)
	{
		//弧反馈
		if (!CheckBitNo(m_vtRobotIO[i].tCuttingRobotIO.nArcFeedbackIO))
		{
			COPini opini2;
			opini2.SetFileName(DEBUG_INI);
			opini2.SetSectionName("Debug");
			opini2.WriteString("EnableArcFeedbackSignal", 0);
			opini2.WriteString("EnableCheckArcFeedback", 0);
		}
		//小弧反馈
		if (!CheckBitNo(m_vtRobotIO[i].tCuttingRobotIO.nSmallArcFeedbackIO))
		{
			COPini opini2;
			opini2.SetFileName(DEBUG_INI);
			opini2.SetSectionName("Debug");
			opini2.WriteString("EnableSmallArcSignal", 0);
		}
	}
}

bool CIOControl::LoadRobotIO()
{
	m_vtRobotIO.clear();

	//加载机械臂数目
	COPini cIni;
	cIni.SetFileName(XI_ROBOT_CTRL_INI);
	cIni.SetSectionName("RobotNum");
	cIni.ReadString("RobotNum", &m_nRobotNum);

	for (int i = 0; i < m_nRobotNum; i++)
	{
		CString strRobotNo;
		CString strRobotName;
		T_ROBOT_IO tRobotIO;
		int nRobotType;

		//加载机械臂名称
		strRobotNo.Format("Robot%d", i);
		COPini cIni2;
		cIni2.SetFileName(XI_ROBOT_CTRL_INI);
		cIni2.SetSectionName("RobotName");
		cIni2.ReadString(strRobotNo, strRobotName);

		//加载机械臂类型
		cIni2.SetSectionName("RobotType");
		cIni2.ReadString(strRobotName, &nRobotType);

		//加载IO
		COPini opini;
		BOOL bRet = TRUE;
		opini.SetFileName(DATA_PATH + strRobotName + ROBOT_IO_PARA);
		opini.SetSectionName("INPUT");
		bRet = bRet && opini.ReadString("RobotRunningIO", &tRobotIO.tYaskawaRobotIO.tRobotInputIO.nRobotRunningIO);
		bRet = bRet && opini.ReadString("RobotServoOnIO", &tRobotIO.tYaskawaRobotIO.tRobotInputIO.nRobotServoOnIO);
		bRet = bRet && opini.ReadString("ErrorsIO", &tRobotIO.tYaskawaRobotIO.tRobotInputIO.nErrorsIO);
		bRet = bRet && opini.ReadString("BatteryWarningIO", &tRobotIO.tYaskawaRobotIO.tRobotInputIO.nBatteryWarningIO);
		bRet = bRet && opini.ReadString("OperationalOriginIO", &tRobotIO.tYaskawaRobotIO.tRobotInputIO.nOperationalOriginIO);
		bRet = bRet && opini.ReadString("MidwayStartIO", &tRobotIO.tYaskawaRobotIO.tRobotInputIO.nMidwayStartIO);
		if ((nRobotType & CUTTING_ROBOT) == CUTTING_ROBOT)
		{
			bRet = bRet && opini.ReadString("AntisplashOpenedIO", &tRobotIO.tCuttingRobotIO.nAntisplashOpenedIO);
			bRet = bRet && opini.ReadString("AntisplashClosedIO", &tRobotIO.tCuttingRobotIO.nAntisplashClosedIO);
			bRet = bRet && opini.ReadString("ArcFeedbackIO", &tRobotIO.tCuttingRobotIO.nArcFeedbackIO);
			bRet = bRet && opini.ReadString("SmallArcFeedbackIO", &tRobotIO.tCuttingRobotIO.nSmallArcFeedbackIO);			
		}
		else
		{
			tRobotIO.tCuttingRobotIO.nAntisplashOpenedIO = INVALID_IO_NUM;
			tRobotIO.tCuttingRobotIO.nAntisplashClosedIO = INVALID_IO_NUM;
			tRobotIO.tCuttingRobotIO.nArcFeedbackIO = INVALID_IO_NUM;
			tRobotIO.tCuttingRobotIO.nSmallArcFeedbackIO = INVALID_IO_NUM;
		}

		opini.SetSectionName("OUTPUT");
		bRet = bRet && opini.ReadString("RobotInitIO", &tRobotIO.tYaskawaRobotIO.tRobotOutputIO.nRobotInitIO);
		bRet = bRet && opini.ReadString("CallJobIO", &tRobotIO.tYaskawaRobotIO.tRobotOutputIO.nCallJobIO);
		bRet = bRet && opini.ReadString("ResetErrorsIO", &tRobotIO.tYaskawaRobotIO.tRobotOutputIO.nResetErrorsIO);
		bRet = bRet && opini.ReadString("ExRobotEmgIO", &tRobotIO.tYaskawaRobotIO.tRobotOutputIO.nRobotExEmgIO);
		if ((nRobotType & CUTTING_ROBOT) == CUTTING_ROBOT)
		{		
			bRet = bRet && opini.ReadString("SupLEDIO", &tRobotIO.tCuttingRobotIO.nSupLEDIO);
			bRet = bRet && opini.ReadString("SupLED1IO", &tRobotIO.tCuttingRobotIO.nSupLED1IO);
			bRet = bRet && opini.ReadString("SupLED2IO", &tRobotIO.tCuttingRobotIO.nSupLED2IO);
			bRet = bRet && opini.ReadString("AirSolenoidValueIO", &tRobotIO.tCuttingRobotIO.nAirSolenoidValueIO);
			bRet = bRet && opini.ReadString("AntisplashIO", &tRobotIO.tCuttingRobotIO.nAntisplashIO);
			bRet = bRet && opini.ReadString("LaserIO", &tRobotIO.tCuttingRobotIO.nLaserIO);
			bRet = bRet && opini.ReadString("PlamsaPowerIO", &tRobotIO.tCuttingRobotIO.nPlamsaPowerIO);
			bRet = bRet && opini.ReadString("PlamsaArcIO", &tRobotIO.tCuttingRobotIO.nPlamsaArcIO);			
		}
		else
		{
			tRobotIO.tCuttingRobotIO.nSupLEDIO = INVALID_IO_NUM;
			tRobotIO.tCuttingRobotIO.nSupLED1IO = INVALID_IO_NUM;
			tRobotIO.tCuttingRobotIO.nSupLED2IO = INVALID_IO_NUM;
			tRobotIO.tCuttingRobotIO.nAirSolenoidValueIO = INVALID_IO_NUM;
			tRobotIO.tCuttingRobotIO.nAntisplashIO = INVALID_IO_NUM;
			bRet = bRet && opini.ReadString("LaserIO", &tRobotIO.tCuttingRobotIO.nLaserIO);
			tRobotIO.tCuttingRobotIO.nPlamsaPowerIO = INVALID_IO_NUM;
			tRobotIO.tCuttingRobotIO.nPlamsaArcIO = INVALID_IO_NUM;
		}

		if (FALSE == bRet)
		{
			XiMessageBox("Error:IOPara.ini文件参数加载失败！");
			return false;
		}
		m_vtRobotIO.push_back(tRobotIO);
	}
	return true;
}

bool CIOControl::CheckBitNo(WORD bitno)
{
	//计算软件卡号
	WORD wSoftwareCardNo = bitno / 100;
	//计算软件IO口号
	WORD wSoftwareBitNo = bitno % 100;
	//判定软件卡号是否存在
	std::map<int, int>::iterator it;
	it = m_mapCardNum.find(wSoftwareCardNo);
	if (it == m_mapCardNum.end())
	{
		return false;
	}
	//获取硬件卡号
	WORD wCardNo = m_vtCardPara[m_mapCardNum[wSoftwareCardNo]].nHardwareCardNo;
	//计算节点编号
	WORD wNoteID = wSoftwareCardNo - m_vtCardPara[m_mapCardNum[wSoftwareCardNo]].nSoftwareCardNoStrat;

	switch (m_vtCardPara[m_mapCardNum[wSoftwareCardNo]].nCardModel)
	{
	case IOC0640:
	{
		if (wSoftwareBitNo > m_vtCardPara[m_mapCardNum[wSoftwareCardNo]].nMaxIONum)
		{
			return false;
		}
		return true;
	}
	case IOC1280:
	{
		if (wSoftwareBitNo > m_vtCardPara[m_mapCardNum[wSoftwareCardNo]].nMaxIONum)
		{
			return false;
		}
		return true;
	}
	case DMC_CAN:
	{
		if (wSoftwareBitNo > m_vtCardPara[m_mapCardNum[wSoftwareCardNo]].nMaxIONum)
		{
			return false;
		}
		return true;
	}
	case DMC_CAT:
	{
		WORD wBitNo = m_vtCardPara[m_mapCardNum[wSoftwareCardNo]].nCATStartIONo + wNoteID * (m_vtCardPara[m_mapCardNum[wSoftwareCardNo]].nMaxIONum + 1) + wSoftwareBitNo;
		if (wBitNo > m_vtCardPara[m_mapCardNum[wSoftwareCardNo]].nMaxIONum)
		{
			return false;
		}
		return true;
	}
	default:
	{
		XiMessageBox("ERROR: 无效的IO卡类型！");
		return false;
	}
	}
}

void CIOControl::OpenRobot(int nRobotNo)
{
	WriteOutbit(m_vtRobotIO[nRobotNo].tYaskawaRobotIO.tRobotOutputIO.nRobotInitIO, ON);
}

void CIOControl::CloseRobot(int nRobotNo)
{
	WriteOutbit(m_vtRobotIO[nRobotNo].tYaskawaRobotIO.tRobotOutputIO.nRobotInitIO, OFF);
}

void CIOControl::OpenCallJob(int nRobotNo)
{
	WriteOutbit(m_vtRobotIO[nRobotNo].tYaskawaRobotIO.tRobotOutputIO.nCallJobIO, ON);
}

void CIOControl::CloseCallJob(int nRobotNo)
{
	WriteOutbit(m_vtRobotIO[nRobotNo].tYaskawaRobotIO.tRobotOutputIO.nCallJobIO, OFF);
}

int CIOControl::CheckRobotDone(int nRobotNo, int nDelayTime, int nProximity /*= false*/)
{
	int nEachDelayTime = 10;
	int nDeleyTimes = nDelayTime / nEachDelayTime;
	int nRobotStatus = ReadInbit(m_vtRobotIO[nRobotNo].tYaskawaRobotIO.tRobotInputIO.nRobotRunningIO);
	while (ON != nRobotStatus)
	{
		if (nDeleyTimes < 0)
		{
			break;
		}
		Sleep(nEachDelayTime);
		nRobotStatus = ReadInbit(m_vtRobotIO[nRobotNo].tYaskawaRobotIO.tRobotInputIO.nRobotRunningIO);
		if (nProximity > 0 && !CheckRobotMoveSafe())
		{
			return -1;
		}
		if (nProximity == 1)
		{
			if (GrabSuccess(1) || GrabSuccess(2))
			{
				return 2;
			}
		}
		else if (nProximity == 2)
		{
			if ((!GrabSuccess(1)) && (!GrabSuccess(2)))
			{
				return 3;
			}
		}
		nDeleyTimes--;
	}
	nRobotStatus = ReadInbit(m_vtRobotIO[nRobotNo].tYaskawaRobotIO.tRobotInputIO.nRobotRunningIO);
	int i = 0;
	while (1 != nRobotStatus)
	{
		if (nProximity > 0 && !CheckRobotMoveSafe())
		{
			return -1;
		}
		if (nProximity == 1)
		{
			if (GrabSuccess(1) || GrabSuccess(2))
			{
				return 2;
			}
		}
		else if (nProximity == 2)
		{
			if ((!GrabSuccess(1)) && (!GrabSuccess(2)))
			{
				return 3;
			}
		}
		Sleep(10);
		nRobotStatus = ReadInbit(m_vtRobotIO[nRobotNo].tYaskawaRobotIO.tRobotInputIO.nRobotRunningIO);
		if (nRobotStatus == OFF)
		{
			i++;
			nRobotStatus = ON;
			if (i > 25)//18*10ms时刚好达到完全停住的状态（目测），为了稳定性，多延迟70ms
			{
				break;
			}
		}
		else
		{
			i = 0;
		}
	}
	return 1;
}

bool CIOControl::RobotServoOn(int nRobotNo, bool bStopArc, bool bPopup)
{
	if (OFF == ReadInbit(m_vtRobotIO[nRobotNo].tYaskawaRobotIO.tRobotInputIO.nRobotServoOnIO))
	{
		if (bStopArc)
		{
			StopPlamsaArc(nRobotNo);
			ClosePolisher();
		}		
		if (bPopup)
		{
			XiMessageBox("Warning:RobotServoOn():机械臂已急停！");
		}		
		return false;
	}
	else
	{
		return true;
	}
}

bool CIOControl::Errors(int nRobotNo)
{
	if (ON == ReadInbit(m_vtRobotIO[nRobotNo].tYaskawaRobotIO.tRobotInputIO.nErrorsIO))
	{
		return true;
	} 
	else
	{
		return false;
	}
}

void CIOControl::ResetErrors(int nRobotNo)
{
	if (Errors(nRobotNo))
	{
		WriteOutbit(m_vtRobotIO[nRobotNo].tYaskawaRobotIO.tRobotOutputIO.nResetErrorsIO, ON);
		Sleep(200);
		WriteOutbit(m_vtRobotIO[nRobotNo].tYaskawaRobotIO.tRobotOutputIO.nResetErrorsIO, OFF);
	} 
	else
	{
		WriteOutbit(m_vtRobotIO[nRobotNo].tYaskawaRobotIO.tRobotOutputIO.nResetErrorsIO, OFF);
	}	
}

bool CIOControl::BatteryWarning(int nRobotNo)
{
	if (ON == ReadInbit(m_vtRobotIO[nRobotNo].tYaskawaRobotIO.tRobotInputIO.nBatteryWarningIO))
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool CIOControl::OperationalOrigin(int nRobotNo)
{
	if (ON == ReadInbit(m_vtRobotIO[nRobotNo].tYaskawaRobotIO.tRobotInputIO.nOperationalOriginIO))
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool CIOControl::MidwayStart(int nRobotNo)
{
	if (ON == ReadInbit(m_vtRobotIO[nRobotNo].tYaskawaRobotIO.tRobotInputIO.nMidwayStartIO))
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool CIOControl::OpenMagnetInChamferingTable(int iNo)
{
	COPini opini;
	BOOL bRet = TRUE;
	int iTemp;
	CString sTemp;
	opini.SetFileName(GENERAL_CONTROL_PATH+"IOParam.ini");
	if (iNo <= 16)
	{
		opini.SetSectionName("EM5_OUT");
	}
	else
	{
		opini.SetSectionName("EM6_OUT");
	}
	sTemp.Format(_T("ChamferTableMagnet%dControl"), iNo);
	bRet = bRet && opini.ReadString(sTemp, &iTemp);
	return WriteOutbit(iTemp, ON) == ON;
}

bool CIOControl::CloseMagnetInChamferingTable(int iNo)
{
	COPini opini;
	BOOL bRet = TRUE;
	int iTemp;
	CString sTemp;
	opini.SetFileName(GENERAL_CONTROL_PATH + "IOParam.ini");
	if (iNo <= 16)
	{
		opini.SetSectionName("EM5_OUT");
	}
	else
	{
		opini.SetSectionName("EM6_OUT");
	}
	sTemp.Format(_T("ChamferTableMagnet%dControl"), iNo);
	bRet = bRet && opini.ReadString(sTemp, &iTemp);
	return WriteOutbit(iTemp, OFF) == ON;
}