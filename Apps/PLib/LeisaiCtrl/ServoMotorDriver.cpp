#include "stdafx.h"
#include "ServoMotorDriver.h"


CServoMotorDriver::CServoMotorDriver()
{
}


CServoMotorDriver::~CServoMotorDriver()
{
	for (auto it = m_mAbsEncoderCtrl.begin(); it != m_mAbsEncoderCtrl.end(); it++) 
	{
		if (it->second != NULL)
		{
			it->second->ClosePort();
			delete it->second;
			it->second = NULL;
		}
	}
}

bool CServoMotorDriver::LoadCtrlCardParam(std::vector<T_CTRL_CARD_INFO> &vtCtrlCardInfo)
{
	bool bRtn = true;
	COPini opini;
	int nCtrlCardNum = 0;
	opini.SetFileName(SERVO_CTRL_DATA_INI);
	opini.SetSectionName("CtrlCardBaseInfo");
	opini.ReadAddString("Num", &nCtrlCardNum, 0);
	if (0 == nCtrlCardNum)
	{
		WRITE_LOG(GetStr("%s->轴卡数量为0", SERVO_CTRL_DATA_INI));
		return false;
	}
	vtCtrlCardInfo.clear();
	for (int n = 0; n < nCtrlCardNum; n++)
	{
		bRtn = true;
		CString strSectionName;
		strSectionName.Format("CtrlCard_%d", n);
		opini.SetSectionName(strSectionName);
		T_CTRL_CARD_INFO tTempCtrlCardInfo;
		int nTemp;
		bRtn = bRtn && opini.ReadAddString("CtrlCardType", &nTemp, (int)E_CTRL_CARD_TYPE_DMC5410);
		bRtn = bRtn && opini.ReadAddString("HardCtrlCardNo", &tTempCtrlCardInfo.nHardCtrlCardNo, 0);
		bRtn = bRtn && opini.ReadAddString("SoftCtrlCardNo", &tTempCtrlCardInfo.nSoftCtrlCardNo, 0);
		bRtn = bRtn && opini.ReadAddString("AxisNum", &tTempCtrlCardInfo.nAxisNum, 0);
		CHECK_RTN_BOOL(bRtn);
		tTempCtrlCardInfo.eCtrlCardType = (E_CTRL_CARD_TYPE)nTemp;
		opini.ReadAddString("ConnectType", &nTemp, (int)E_CONNECT_TYPE_DIRECT);
		tTempCtrlCardInfo.eConnectType = (E_CONNECT_TYPE)nTemp;
		opini.ReadAddString("NetWorkIP", tTempCtrlCardInfo.strNetWorkIP, "192.168.5.11");

		strSectionName.Format("CtrlCard_%d_IO", n);
		opini.SetSectionName(strSectionName);

		bRtn = bRtn && opini.ReadAddString("HardCtrlCardNo", &tTempCtrlCardInfo.tIOCtrlInfo.nHardCtrlCardNo, 0);
		bRtn = bRtn && opini.ReadAddString("SoftCtrlCardNo", &tTempCtrlCardInfo.tIOCtrlInfo.nSoftCtrlCardNo, 0);
		bRtn = bRtn && opini.ReadAddString("PortNo", &tTempCtrlCardInfo.tIOCtrlInfo.nPortNo, -1);
		bRtn = bRtn && opini.ReadAddString("IOType", &nTemp, (int)E_IO_CARD_TYPE_NONE);
		tTempCtrlCardInfo.tIOCtrlInfo.eIOType = (E_IO_CARD_TYPE)nTemp;
		bRtn = bRtn && opini.ReadAddString("LocalIONum", &tTempCtrlCardInfo.tIOCtrlInfo.nLocalIONum, 0);
		bRtn = bRtn && opini.ReadAddString("SingleIONum", &tTempCtrlCardInfo.tIOCtrlInfo.nSingleIONum, 0);
		bRtn = bRtn && opini.ReadAddString("NoteNum", &tTempCtrlCardInfo.tIOCtrlInfo.nNoteNum, 0);
		bRtn = bRtn && opini.ReadAddString("CANBaudNo", &tTempCtrlCardInfo.tIOCtrlInfo.nCANBaudNo, 0);
		bRtn = bRtn && opini.ReadAddString("CATCycleTime",&tTempCtrlCardInfo.tIOCtrlInfo.nCATCycleTime, 0);
		bRtn = bRtn && opini.ReadAddString("IOCFilter", &tTempCtrlCardInfo.tIOCtrlInfo.nIOCFilter, 1);
		for (int m = 0; m < tTempCtrlCardInfo.nAxisNum; m++)
		{
//			m_vpcAbsEncoderCtrl.push_back(CAbsoluteEncoder());
			T_AXIS_CTRL_INFO tTempAxisInfo;
			strSectionName.Format("CtrlCard_%d_Axis_%d", n, m);
			opini.SetSectionName(strSectionName);
			bRtn = bRtn && opini.ReadAddString("HardCtrlCardNo", &tTempAxisInfo.nHardCtrlCardNo, 0);
			bRtn = bRtn && opini.ReadAddString("SoftCtrlCardNo", &tTempAxisInfo.nSoftCtrlCardNo, 0);
			bRtn = bRtn && opini.ReadAddString("SoftAxisNo", &tTempAxisInfo.nSoftAxisNo, 0);
			bRtn = bRtn && opini.ReadAddString("HardAxisNo", &tTempAxisInfo.nHardAxisNo, 0);
			bRtn = bRtn && opini.ReadAddString("AxisName", tTempAxisInfo.strAxisName, "X");
			bRtn = bRtn && opini.ReadAddString("ServoType", &nTemp, 0);
			tTempAxisInfo.eServoType = (E_SERVO_TYPE)nTemp;
			bRtn = bRtn && opini.ReadAddString("EnableAbsEncoder", &tTempAxisInfo.bEnableAbsEncoder, false);
			bRtn = bRtn && opini.ReadAddString("AbsEncoderPartNo", &tTempAxisInfo.nAbsEncoderPartNo, -1);
			bRtn = bRtn && opini.ReadAddString("LapPulse", &tTempAxisInfo.nLapPulse, 2000);
			bRtn = bRtn && opini.ReadAddString("AbsEncoderDir", &tTempAxisInfo.nAbsEncoderDir, 1);
			bRtn = bRtn && opini.ReadAddString("PulseEquivalent", &tTempAxisInfo.dPulseEquivalent, 0);
			tTempAxisInfo.dPulseEquivalent = fabs(tTempAxisInfo.dPulseEquivalent);
			bRtn = bRtn && opini.ReadAddString("PulseMode", &tTempAxisInfo.nPulseMode, 0);

			bRtn = bRtn && opini.ReadAddString("EnableSoftLimit", &tTempAxisInfo.bEnableSoftLimit, false);
			opini.ReadAddString("PositiveLimit", &tTempAxisInfo.dPositiveLimit, 0);
			opini.ReadAddString("NegativeLimit", &tTempAxisInfo.dNegativeLimit, 0);

			bRtn = bRtn && opini.ReadAddString("MaxSpeed", &tTempAxisInfo.tMaxSpeed.dSpeed, 0);
			bRtn = bRtn && opini.ReadAddString("MaxSpeed_ACC", &tTempAxisInfo.tMaxSpeed.dAcc, 1);
			bRtn = bRtn && opini.ReadAddString("MaxSpeed_DEC", &tTempAxisInfo.tMaxSpeed.dDec, 1);
			bRtn = bRtn && opini.ReadAddString("MidSpeed", &tTempAxisInfo.tMidSpeed.dSpeed, 0);
			bRtn = bRtn && opini.ReadAddString("MidSpeed_ACC", &tTempAxisInfo.tMidSpeed.dAcc, 1);
			bRtn = bRtn && opini.ReadAddString("MidSpeed_DEC", &tTempAxisInfo.tMidSpeed.dDec, 1);
			bRtn = bRtn && opini.ReadAddString("MinSpeed", &tTempAxisInfo.tMinSpeed.dSpeed, 0);
			bRtn = bRtn && opini.ReadAddString("MinSpeed_ACC", &tTempAxisInfo.tMinSpeed.dAcc, 1);
			bRtn = bRtn && opini.ReadAddString("MinSpeed_DEC", &tTempAxisInfo.tMinSpeed.dDec, 1);

			bRtn = bRtn && opini.ReadAddString("AlmLogic", &tTempAxisInfo.bAlmLogic, true);
			bRtn = bRtn && opini.ReadAddString("GearFollowProfile", &tTempAxisInfo.nGearFollowProfile, 0);
			opini.ReadAddString("MasterAxisNo", &tTempAxisInfo.nMasterAxisNo, -1);

			tTempCtrlCardInfo.vtAxisInfo.push_back(tTempAxisInfo);
		}
		vtCtrlCardInfo.push_back(tTempCtrlCardInfo);
	}
	return bRtn;
}

bool CServoMotorDriver::SaveCtrlCardParam(std::vector<T_CTRL_CARD_INFO> vtCtrlCardInfo)
{
	bool bRtn = true;
	COPini opini;
	int nCtrlCardNum = vtCtrlCardInfo.size();
	if (0 == nCtrlCardNum)
	{
		WRITE_LOG("轴卡数量为0");
		return false;
	}
	opini.SetFileName(SERVO_CTRL_DATA_INI);
	opini.SetSectionName("CtrlCardBaseInfo");
	opini.WriteString("Num", nCtrlCardNum);

	for (int n = 0; n < nCtrlCardNum; n++)
	{
		CString strSectionName;
		strSectionName.Format("CtrlCard_%d", n);
		opini.SetSectionName(strSectionName);
		opini.WriteString("HardCtrlCardNo", vtCtrlCardInfo[n].nHardCtrlCardNo);
		opini.WriteString("SoftCtrlCardNo", vtCtrlCardInfo[n].nSoftCtrlCardNo);
		opini.WriteString("CtrlCardType", (int)vtCtrlCardInfo[n].eCtrlCardType);
		opini.WriteString("ConnectType", (int)vtCtrlCardInfo[n].eConnectType);
		opini.WriteString("NetWorkIP", vtCtrlCardInfo[n].strNetWorkIP);
		opini.WriteString("AxisNum", vtCtrlCardInfo[n].nAxisNum);

		strSectionName.Format("CtrlCard_%d_IO", n);
		opini.SetSectionName(strSectionName);
		vtCtrlCardInfo[n].tIOCtrlInfo.nHardCtrlCardNo = vtCtrlCardInfo[n].nHardCtrlCardNo;
		vtCtrlCardInfo[n].tIOCtrlInfo.nSoftCtrlCardNo = vtCtrlCardInfo[n].nSoftCtrlCardNo;
		opini.WriteString("HardCtrlCardNo", vtCtrlCardInfo[n].tIOCtrlInfo.nHardCtrlCardNo);
		opini.WriteString("SoftCtrlCardNo", vtCtrlCardInfo[n].tIOCtrlInfo.nSoftCtrlCardNo);
		opini.WriteString("PortNo", vtCtrlCardInfo[n].tIOCtrlInfo.nPortNo);
		opini.WriteString("IOType", (int)vtCtrlCardInfo[n].tIOCtrlInfo.eIOType);
		opini.WriteString("LocalIONum", vtCtrlCardInfo[n].tIOCtrlInfo.nLocalIONum);
		opini.WriteString("SingleIONum", vtCtrlCardInfo[n].tIOCtrlInfo.nSingleIONum);
		opini.WriteString("NoteNum", vtCtrlCardInfo[n].tIOCtrlInfo.nNoteNum);
		opini.WriteString("CANBaudNo", vtCtrlCardInfo[n].tIOCtrlInfo.nCANBaudNo);
		opini.WriteString("CATCycleTime", vtCtrlCardInfo[n].tIOCtrlInfo.nCATCycleTime);
		opini.WriteString("IOCFilter", vtCtrlCardInfo[n].tIOCtrlInfo.nIOCFilter);
		for (int m = 0; m < vtCtrlCardInfo[n].nAxisNum; m++)
		{
			strSectionName.Format("CtrlCard_%d_Axis_%d", n, m);
			opini.SetSectionName(strSectionName);

			opini.WriteString("HardCtrlCardNo", vtCtrlCardInfo[n].vtAxisInfo[m].nHardCtrlCardNo);
			opini.WriteString("SoftCtrlCardNo", vtCtrlCardInfo[n].vtAxisInfo[m].nSoftCtrlCardNo);
			opini.WriteString("SoftAxisNo", vtCtrlCardInfo[n].vtAxisInfo[m].nSoftAxisNo);
			opini.WriteString("HardAxisNo", vtCtrlCardInfo[n].vtAxisInfo[m].nHardAxisNo);
			opini.WriteString("AxisName", vtCtrlCardInfo[n].vtAxisInfo[m].strAxisName);
			opini.WriteString("ServoType", (int)vtCtrlCardInfo[n].vtAxisInfo[m].eServoType);
			opini.WriteString("EnableAbsEncoder", vtCtrlCardInfo[n].vtAxisInfo[m].bEnableAbsEncoder);
			opini.WriteString("AbsEncoderPartNo", vtCtrlCardInfo[n].vtAxisInfo[m].nAbsEncoderPartNo);
			opini.WriteString("LapPulse", vtCtrlCardInfo[n].vtAxisInfo[m].nLapPulse);
			opini.WriteString("AbsEncoderDir", vtCtrlCardInfo[n].vtAxisInfo[m].nAbsEncoderDir);
			opini.WriteString("PulseEquivalent", vtCtrlCardInfo[n].vtAxisInfo[m].dPulseEquivalent,15);
			opini.WriteString("PulseMode", vtCtrlCardInfo[n].vtAxisInfo[m].nPulseMode);

			opini.WriteString("EnableSoftLimit", vtCtrlCardInfo[n].vtAxisInfo[m].bEnableSoftLimit);
			opini.WriteString("PositiveLimit", vtCtrlCardInfo[n].vtAxisInfo[m].dPositiveLimit);
			opini.WriteString("NegativeLimit", vtCtrlCardInfo[n].vtAxisInfo[m].dNegativeLimit);

			opini.WriteString("MaxSpeed", vtCtrlCardInfo[n].vtAxisInfo[m].tMaxSpeed.dSpeed);
			opini.WriteString("MaxSpeed_ACC", vtCtrlCardInfo[n].vtAxisInfo[m].tMaxSpeed.dAcc);
			opini.WriteString("MaxSpeed_DEC", vtCtrlCardInfo[n].vtAxisInfo[m].tMaxSpeed.dDec);
			opini.WriteString("MidSpeed", vtCtrlCardInfo[n].vtAxisInfo[m].tMidSpeed.dSpeed);
			opini.WriteString("MidSpeed_ACC", vtCtrlCardInfo[n].vtAxisInfo[m].tMidSpeed.dAcc);
			opini.WriteString("MidSpeed_DEC", vtCtrlCardInfo[n].vtAxisInfo[m].tMidSpeed.dDec);
			opini.WriteString("MinSpeed", vtCtrlCardInfo[n].vtAxisInfo[m].tMinSpeed.dSpeed);
			opini.WriteString("MinSpeed_ACC", vtCtrlCardInfo[n].vtAxisInfo[m].tMinSpeed.dAcc);
			opini.WriteString("MinSpeed_DEC", vtCtrlCardInfo[n].vtAxisInfo[m].tMinSpeed.dDec);

			opini.WriteString("AlmLogic", vtCtrlCardInfo[n].vtAxisInfo[m].bAlmLogic);
			opini.WriteString("GearFollowProfile", vtCtrlCardInfo[n].vtAxisInfo[m].nGearFollowProfile);
			opini.WriteString("MasterAxisNo", vtCtrlCardInfo[n].vtAxisInfo[m].nMasterAxisNo);
		}
	}
	return true;
}

int CServoMotorDriver::InitCtrlCard(std::vector<T_CTRL_CARD_INFO> vtCtrlCardInfo)
{
	int nCardNum = vtCtrlCardInfo.size();
	int nInitCardNum = 0;
	int nRtn = 0;
	bool bInitDirectCtrlCard = true;
	bool bInitIOCtrlCard = true;
	for (int n = 0; n < nCardNum; n++)
	{
		switch (vtCtrlCardInfo[n].eCtrlCardType)
		{
		case E_CTRL_CARD_TYPE_DMC5410:
		case E_CTRL_CARD_TYPE_DMC5800:
		case E_CTRL_CARD_TYPE_DMC5810:
		{
			if (bInitDirectCtrlCard)
			{
				bInitDirectCtrlCard = false;
				nRtn = InitDirectCtrlCard(vtCtrlCardInfo[n], nInitCardNum);
				CHECK_RTN_INT(nRtn);
			}
			nRtn = InitCanIOCtrl(vtCtrlCardInfo[n].tIOCtrlInfo);
			CHECK_RTN_INT(nRtn);
			nRtn = InitCatIOCtrl(vtCtrlCardInfo[n].tIOCtrlInfo);
			CHECK_RTN_INT(nRtn);
			for (int m = 0; m < vtCtrlCardInfo[n].vtAxisInfo.size(); m++)
			{
				nRtn = InitDirectAxisParam(vtCtrlCardInfo[n].vtAxisInfo[m]);
				CHECK_RTN_INT(nRtn);
				m_nAxisNum++;
			}
			break;
		}
		case E_CTRL_CARD_TYPE_SMC604:
		{
			if (vtCtrlCardInfo[n].eConnectType == E_CONNECT_TYPE_NETWORK)
			{
				nRtn = InitNetWorkCtrlCard(vtCtrlCardInfo[n], nInitCardNum);
				CHECK_RTN_INT(nRtn);
				for (int m = 0; m < vtCtrlCardInfo[n].vtAxisInfo.size(); m++)
				{
					nRtn = InitNetWorkAxisParam(vtCtrlCardInfo[n].vtAxisInfo[m]);
					CHECK_RTN_INT(nRtn);
					m_nAxisNum++;
				}
			}
			break;
		}
		case E_CTRL_CARD_TYPE_IOC0640:
		case E_CTRL_CARD_TYPE_IOC1280:
		{
			if (bInitIOCtrlCard)
			{
				bInitIOCtrlCard = false;
				nRtn = InitIOCCtrlCard(vtCtrlCardInfo[n], nInitCardNum);
				CHECK_RTN_INT(nRtn);
			}
			break;
		}
		default:
			break;
		}
	}
	if (nCardNum != nInitCardNum)
	{
		return CTRL_CARD_NUM_ERROR;
	}
	m_nCardNum = nCardNum;
	return nRtn;
}

int CServoMotorDriver::InitDirectCtrlCard(T_CTRL_CARD_INFO tCtrlCardInfo, int &nCardNum)
{
	int nInitCardNum = dmc_board_init();
	//if (0 == nInitCardNum)
	//{
	//	return CTRL_CARD_NO_FOUND;
	//}
	DWORD nTotalAxis = 0;
	WORD iret = dmc_get_total_axes(tCtrlCardInfo.nHardCtrlCardNo, &nTotalAxis);
	CHECK_RTN_INT(iret);
	WRITE_LOG(GetStr("读取轴数量：%d", nTotalAxis));
	if (tCtrlCardInfo .vtAxisInfo.size()-1 > nTotalAxis)
	{
		WRITE_LOG("轴数量错误！");
		return INPUT_INFO_ERROR;
	}
	nCardNum += nInitCardNum;
	return 0;
}

int CServoMotorDriver::InitNetWorkCtrlCard(T_CTRL_CARD_INFO tCtrlCardInfo, int &nCardNum)
{
	WORD iret = smc_board_init(tCtrlCardInfo.nHardCtrlCardNo, 2, tCtrlCardInfo.strNetWorkIP.GetBuffer(), 0);
	CHECK_RTN_INT(iret);
	nCardNum++;
	return 0;
}

int CServoMotorDriver::InitIOCCtrlCard(T_CTRL_CARD_INFO tCtrlCardInfo, int &nCardNum)
{
	int nNum = ioc_board_init();
	if (nNum == 0)
	{
		return CTRL_CARD_NO_FOUND;
	}
	WORD iret = ioc_set_filter(tCtrlCardInfo.nHardCtrlCardNo, tCtrlCardInfo.tIOCtrlInfo.nIOCFilter);
	CHECK_RTN_INT(iret);
	nCardNum += nNum;
	return 0;
}

int CServoMotorDriver::InitCanIOCtrl(T_IO_CTRL_INFO tIOInfo)
{
	if (tIOInfo.eIOType != E_IO_CARD_TYPE_CAN)
	{
		return 0;
	}
	WORD wNodeNum;
	WORD wState;
	short iRtn = 0;
	iRtn = dmc_get_can_state(tIOInfo.nHardCtrlCardNo, &wNodeNum, &wState);
	CHECK_RTN_INT(iRtn);
	if (wState != 1)
	{
		iRtn = dmc_set_can_state(tIOInfo.nHardCtrlCardNo, tIOInfo.nNoteNum, 1, tIOInfo.nCANBaudNo);
	}
	CHECK_RTN_INT(iRtn);
	Sleep(20);
	iRtn = dmc_get_can_state(tIOInfo.nHardCtrlCardNo, &wNodeNum, &wState);
	CHECK_RTN_INT(iRtn);
	if (wState != 1)
	{
		WRITE_LOG(GetStr("HardCtrlCardNo:%d,CANCountNum:%d; 连接失败", tIOInfo.nHardCtrlCardNo, tIOInfo.nNoteNum));
		return IO_CARD_NO_CONNECT;
	}
	if (wNodeNum != tIOInfo.nNoteNum)
	{
		WRITE_LOG(GetStr("HardCtrlCardNo:%d,CANCountNum:%d,ConnectNum:%d; 连接节点错误", tIOInfo.nHardCtrlCardNo, tIOInfo.nNoteNum, wNodeNum));
		return IO_CARD_NO_CONNECT;
	}
	if (tIOInfo.nLocalIONum > 0)
	{
		WORD LocalTotalIn;
		WORD LocalTotalOut;
		iRtn = dmc_get_total_ionum(tIOInfo.nHardCtrlCardNo, &LocalTotalIn, &LocalTotalOut);
		CHECK_RTN_INT(iRtn);
		if (tIOInfo.nLocalIONum != LocalTotalIn)
		{
			WRITE_LOG(GetStr("HardCtrlCardNo:%d,LocalTotal:%d,SetLocalTotal:%d; 本地IO数量与设定不相等", tIOInfo.nHardCtrlCardNo, LocalTotalIn, tIOInfo.nLocalIONum));
			//return IO_CARD_NO_CONNECT;
		}
	}
	return 0;
}

int CServoMotorDriver::InitCatIOCtrl(T_IO_CTRL_INFO tIOInfo)
{
	if (tIOInfo.eIOType != E_IO_CARD_TYPE_CAT)
	{
		return 0;
	}
	short iRtn = 0;
	iRtn = nmc_set_cycletime(tIOInfo.nHardCtrlCardNo, tIOInfo.nNoteNum, tIOInfo.nCATCycleTime);
	CHECK_RTN_INT(iRtn);
	Sleep(20);
	WORD wState;
	iRtn = nmc_get_errcode(tIOInfo.nHardCtrlCardNo, 2, &wState);
	CHECK_RTN_INT(iRtn);
	if (wState != 0)
	{
		WRITE_LOG(GetStr("HardCtrlCardNo:%d,CATCountNum:%d; 连接失败", tIOInfo.nHardCtrlCardNo, tIOInfo.nNoteNum));
//		return IO_CARD_NO_CONNECT;
	}
	WORD wNodeNum;
	iRtn = nmc_get_total_slaves(tIOInfo.nHardCtrlCardNo, 2, &wNodeNum);
	CHECK_RTN_INT(iRtn);
	if (wNodeNum != tIOInfo.nNoteNum)
	{
		WRITE_LOG(GetStr("HardCtrlCardNo:%d,CATCountNum:%d,ConnectNum:%d; 连接节点错误", tIOInfo.nHardCtrlCardNo, tIOInfo.nNoteNum, wNodeNum));
		return IO_CARD_NO_CONNECT;
	}
	WORD CatTotalIn;
	WORD CatTotalOut;
	iRtn = nmc_get_total_ionum(tIOInfo.nHardCtrlCardNo, &CatTotalIn, &CatTotalOut);
	CHECK_RTN_INT(iRtn);
	if (CatTotalIn != CatTotalOut)
	{
		WRITE_LOG(GetStr("HardCtrlCardNo:%d,CatTotalIn:%d,CatTotalOut:%d; 输入输出IO数量不相等", tIOInfo.nHardCtrlCardNo, CatTotalIn, CatTotalOut));
		return IO_CARD_NO_CONNECT;
	}
	if (CatTotalIn != tIOInfo.nSingleIONum * tIOInfo.nNoteNum)
	{
		WRITE_LOG(GetStr("HardCtrlCardNo:%d,CatTotal:%d,SetCatTotal:%d; IO数量与设定不相等", tIOInfo.nHardCtrlCardNo, CatTotalIn, tIOInfo.nSingleIONum * tIOInfo.nNoteNum));
		return IO_CARD_NO_CONNECT;
	}
	if (tIOInfo.nLocalIONum > 0)
	{
		WORD LocalTotalIn;
		WORD LocalTotalOut;
		iRtn = nmc_get_total_ionum(tIOInfo.nHardCtrlCardNo, &LocalTotalIn, &LocalTotalOut);
		CHECK_RTN_INT(iRtn);
		if (tIOInfo.nLocalIONum != LocalTotalIn)
		{
			WRITE_LOG(GetStr("HardCtrlCardNo:%d,LocalTotal:%d,SetLocalTotal:%d; 本地IO数量与设定不相等", tIOInfo.nHardCtrlCardNo, LocalTotalIn, tIOInfo.nLocalIONum));
			return IO_CARD_NO_CONNECT;
		}
	}
	return 0;
}

int CServoMotorDriver::InitDirectAxisParam(T_AXIS_CTRL_INFO tAxisInfo)
{
	int nRtn = 0;
	dmc_set_alm_mode(tAxisInfo.nHardCtrlCardNo, tAxisInfo.nHardAxisNo, 1, tAxisInfo.bAlmLogic, 0);
	if (tAxisInfo.bEnableAbsEncoder)
	{
		CAbsoluteEncoder* pcAbsEncoderCtrl = NULL;
		m_mAbsEncoderCtrl[tAxisInfo.nAbsEncoderPartNo] = pcAbsEncoderCtrl;
	}
	if (tAxisInfo.bEnableSoftLimit)
	{
		tAxisInfo.dPulseEquivalent = fabs(tAxisInfo.dPulseEquivalent);
		if (tAxisInfo.dPulseEquivalent > 1)
		{
			nRtn = dmc_set_softlimit(tAxisInfo.nHardCtrlCardNo, tAxisInfo.nHardAxisNo, tAxisInfo.bEnableSoftLimit, 0, 0,
				tAxisInfo.dNegativeLimit * tAxisInfo.dPulseEquivalent, tAxisInfo.dPositiveLimit * tAxisInfo.dPulseEquivalent);
		}
		else
		{
			nRtn = dmc_set_softlimit(tAxisInfo.nHardCtrlCardNo, tAxisInfo.nHardAxisNo, tAxisInfo.bEnableSoftLimit, 0, 0,
				tAxisInfo.dNegativeLimit / tAxisInfo.dPulseEquivalent, tAxisInfo.dPositiveLimit / tAxisInfo.dPulseEquivalent);
		}
		//CHECK_RTN_INT(nRtn);
	}
	nRtn = dmc_set_pulse_outmode(tAxisInfo.nHardCtrlCardNo, tAxisInfo.nHardAxisNo, tAxisInfo.nPulseMode);
	CHECK_RTN_INT(nRtn);
	nRtn = dmc_set_equiv(tAxisInfo.nHardCtrlCardNo, tAxisInfo.nHardAxisNo, 1.0);
	CHECK_RTN_INT(nRtn);
	double dSpeed;
	if (tAxisInfo.dPulseEquivalent > 1)
	{
		dSpeed = tAxisInfo.tMidSpeed.dSpeed * tAxisInfo.dPulseEquivalent + 0.5 / 60.0;
	}
	else
	{
		dSpeed = tAxisInfo.tMidSpeed.dSpeed / tAxisInfo.dPulseEquivalent + 0.5 / 60.0;
	}
	nRtn = dmc_set_profile(tAxisInfo.nHardCtrlCardNo, tAxisInfo.nHardAxisNo, 0, 
		dSpeed, tAxisInfo.tMidSpeed.dAcc, tAxisInfo.tMidSpeed.dDec, 0);
	CHECK_RTN_INT(nRtn);
	nRtn = dmc_set_dec_stop_time(tAxisInfo.nHardCtrlCardNo, tAxisInfo.nHardAxisNo, tAxisInfo.tMidSpeed.dDec);
	CHECK_RTN_INT(nRtn);
	if (tAxisInfo.nGearFollowProfile == 2)
	{
		nRtn = dmc_set_gear_follow_profile(tAxisInfo.nHardCtrlCardNo, tAxisInfo.nHardAxisNo, 1/*使能*/, 
			tAxisInfo.nMasterAxisNo, 1.0/*跟随倍率*/);
		CHECK_RTN_INT(nRtn);
	}
	return 0;
}

int CServoMotorDriver::InitNetWorkAxisParam(T_AXIS_CTRL_INFO tAxisInfo)
{
	int nRtn = 0;
	nRtn = smc_set_equiv(tAxisInfo.nHardCtrlCardNo, tAxisInfo.nHardAxisNo, 1.0);
	CHECK_RTN_INT(nRtn);
	nRtn = smc_set_alm_mode(tAxisInfo.nHardCtrlCardNo, tAxisInfo.nHardAxisNo, 1, tAxisInfo.bAlmLogic, 0);
	CHECK_RTN_INT(nRtn);
	nRtn = smc_set_pulse_outmode(tAxisInfo.nHardCtrlCardNo, tAxisInfo.nHardAxisNo, tAxisInfo.nPulseMode);
	CHECK_RTN_INT(nRtn);
	double dSpeed;
	if (tAxisInfo.dPulseEquivalent > 1)
	{
		dSpeed = tAxisInfo.tMidSpeed.dSpeed * tAxisInfo.dPulseEquivalent + 0.5 / 60.0;
	}
	else
	{
		dSpeed = tAxisInfo.tMidSpeed.dSpeed / tAxisInfo.dPulseEquivalent + 0.5 / 60.0;
	}
	nRtn = smc_set_profile_unit(tAxisInfo.nHardCtrlCardNo, tAxisInfo.nHardAxisNo, 0, 
		dSpeed, tAxisInfo.tMidSpeed.dAcc, tAxisInfo.tMidSpeed.dDec, 0);
	CHECK_RTN_INT(nRtn);
	nRtn = smc_set_dec_stop_time(tAxisInfo.nHardCtrlCardNo, tAxisInfo.nHardAxisNo, tAxisInfo.tMidSpeed.dDec);
	CHECK_RTN_INT(nRtn);
	if (tAxisInfo.bEnableSoftLimit)
	{
		tAxisInfo.dPulseEquivalent = fabs(tAxisInfo.dPulseEquivalent);
		if (tAxisInfo.dPulseEquivalent > 1)
		{
			nRtn = smc_set_softlimit_unit(tAxisInfo.nHardCtrlCardNo, tAxisInfo.nHardAxisNo, tAxisInfo.bEnableSoftLimit, 0, 0,
				tAxisInfo.dNegativeLimit * tAxisInfo.dPulseEquivalent, tAxisInfo.dPositiveLimit * tAxisInfo.dPulseEquivalent);
		}
		else
		{
			nRtn = smc_set_softlimit_unit(tAxisInfo.nHardCtrlCardNo, tAxisInfo.nHardAxisNo, tAxisInfo.bEnableSoftLimit, 0, 0,
				tAxisInfo.dNegativeLimit / tAxisInfo.dPulseEquivalent, tAxisInfo.dPositiveLimit / tAxisInfo.dPulseEquivalent);
		}
		CHECK_RTN_INT(nRtn);
	}
	return 0;
}

int CServoMotorDriver::GetSoftAxisNo(int nSoftCtrlCardNo)
{
	int nSoftAxisNo = 0;
	for (int n = 0; n < m_vtCtrlCardInfo.size(); n++)
	{
		for (int m = 0; m < m_vtCtrlCardInfo[n].vtAxisInfo.size(); m++)
		{
			m_vtCtrlCardInfo[n].vtAxisInfo[m].nSoftAxisNo = nSoftAxisNo++;
		}
	}
	return m_vtCtrlCardInfo[nSoftCtrlCardNo].vtAxisInfo.back().nSoftAxisNo;
}

void CServoMotorDriver::SetSoftAxisNo()
{
	int nSoftAxisNo = 0;
	for (int n = 0; n < m_vtCtrlCardInfo.size(); n++)
	{
		for (int m = 0; m < m_vtCtrlCardInfo[n].vtAxisInfo.size(); m++)
		{
			m_vtCtrlCardInfo[n].vtAxisInfo[m].nSoftAxisNo = nSoftAxisNo++;
		}
	}
}

bool CServoMotorDriver::GetAxisCardInfo(int nSoftAxisNo, T_AXIS_CTRL_INFO &tAxisInfo, T_CTRL_CARD_INFO &tCtrlCardInfo)
{
	for (int n = 0; n < m_vtCtrlCardInfo.size(); n++)
	{
		for (int m = 0; m < m_vtCtrlCardInfo[n].vtAxisInfo.size(); m++)
		{
			if (m_vtCtrlCardInfo[n].vtAxisInfo[m].nSoftAxisNo == nSoftAxisNo)
			{
				tCtrlCardInfo = m_vtCtrlCardInfo[n];
				tAxisInfo = m_vtCtrlCardInfo[n].vtAxisInfo[m];
				return true;
			}
		}
	}
	return false;
}

bool CServoMotorDriver::GetAxisCardInfo(int nHardCtrlCardNo, int nHardAxisNo, E_CTRL_CARD_TYPE eCtrlCardType, T_AXIS_CTRL_INFO &tAxisInfo, T_CTRL_CARD_INFO &tCtrlCardInfo)
{
	for (int n = 0; n < m_vtCtrlCardInfo.size(); n++)
	{
		if (m_vtCtrlCardInfo[n].eCtrlCardType != eCtrlCardType || m_vtCtrlCardInfo[n].nHardCtrlCardNo != nHardCtrlCardNo)
		{
			continue;
		}
		for (int m = 0; m < m_vtCtrlCardInfo[n].vtAxisInfo.size(); m++)
		{
			if (m_vtCtrlCardInfo[n].vtAxisInfo[m].nHardAxisNo == nHardAxisNo)
			{
				tCtrlCardInfo = m_vtCtrlCardInfo[n];
				tAxisInfo = m_vtCtrlCardInfo[n].vtAxisInfo[m];
				return true;
			}
		}
	}
	return false;
}

bool CServoMotorDriver::GetCtrlCardInfo(int nSoftCtrlCardNo, T_CTRL_CARD_INFO &tCtrlCardInfo)
{
	for (int n = 0; n < m_vtCtrlCardInfo.size(); n++)
	{
		if (m_vtCtrlCardInfo[n].nSoftCtrlCardNo == nSoftCtrlCardNo)
		{
			tCtrlCardInfo = m_vtCtrlCardInfo[n];
			return true;
		}
	}
	return false;
}

bool CServoMotorDriver::GetCtrlCardInfo(int nHardCtrlCardNo, E_CTRL_CARD_TYPE eCtrlCardType, T_CTRL_CARD_INFO &tCtrlCardInfo)
{
	for (int n = 0; n < m_vtCtrlCardInfo.size(); n++)
	{
		if (m_vtCtrlCardInfo[n].nHardCtrlCardNo == nHardCtrlCardNo && m_vtCtrlCardInfo[n].eCtrlCardType == eCtrlCardType)
		{
			tCtrlCardInfo = m_vtCtrlCardInfo[n];
			return true;
		}
	}
	return false;
}

int CServoMotorDriver::SetSevon(int nSoftAxisNo, bool bEnable)
{
	T_AXIS_CTRL_INFO tAxisInfo;
	T_CTRL_CARD_INFO tCtrlCardInfo;
	bool bRtn = GetAxisCardInfo(nSoftAxisNo, tAxisInfo, tCtrlCardInfo);
	if (!bRtn)
	{
		WRITE_LOG("获取轴卡信息失败");
		return INPUT_INFO_ERROR;
	}
	int nRtn = 0;
	if (tCtrlCardInfo.eConnectType == E_CONNECT_TYPE_DIRECT)
	{
		nRtn = dmc_write_sevon_pin(tAxisInfo.nHardCtrlCardNo, tAxisInfo.nHardAxisNo, !bEnable);
	}
	else
	{
		nRtn = smc_write_sevon_pin(tAxisInfo.nHardCtrlCardNo, tAxisInfo.nHardAxisNo, !bEnable);
	}
	return nRtn;
}

bool CServoMotorDriver::GetSevon(int nSoftAxisNo, bool &bEnable)
{
	T_AXIS_CTRL_INFO tAxisInfo;
	T_CTRL_CARD_INFO tCtrlCardInfo;
	bool bRtn = GetAxisCardInfo(nSoftAxisNo, tAxisInfo, tCtrlCardInfo);
	if (tCtrlCardInfo.eConnectType == E_CONNECT_TYPE_DIRECT)
	{
		bEnable = !dmc_read_sevon_pin(tAxisInfo.nHardCtrlCardNo, tAxisInfo.nHardAxisNo);
	}
	else
	{
		bEnable = !smc_read_sevon_pin(tAxisInfo.nHardCtrlCardNo, tAxisInfo.nHardAxisNo);
	}
	return bRtn;
}

bool CServoMotorDriver::GetSevonRdy(int nSoftAxisNo, bool &bSevonRdy)
{
	T_AXIS_CTRL_INFO tAxisInfo;
	T_CTRL_CARD_INFO tCtrlCardInfo;
	bool bRtn = GetAxisCardInfo(nSoftAxisNo, tAxisInfo, tCtrlCardInfo);
	if (tCtrlCardInfo.eConnectType == E_CONNECT_TYPE_DIRECT)
	{
		bSevonRdy = !dmc_read_rdy_pin(tAxisInfo.nHardCtrlCardNo, tAxisInfo.nHardAxisNo);
	}
	else
	{
		bSevonRdy = !smc_read_sevon_pin(tAxisInfo.nHardCtrlCardNo, tAxisInfo.nHardAxisNo);
	}
	return bRtn;
}

int CServoMotorDriver::PosMove(int nSoftAxisNo, long lDistance, int nMode, double dSpeed, double dAcc, double dDec, double dSParam /*= 0.1*/)
{
	T_AXIS_CTRL_INFO tAxisInfo;
	T_CTRL_CARD_INFO tCtrlCardInfo;
	bool bRtn = GetAxisCardInfo(nSoftAxisNo, tAxisInfo, tCtrlCardInfo);
	if (!bRtn)
	{
		WRITE_LOG("获取轴卡信息失败");
		return INPUT_INFO_ERROR;
	}
	int nRtn = 0;
	if (tCtrlCardInfo.eConnectType == E_CONNECT_TYPE_DIRECT)
	{
		nRtn = dmc_set_profile(tAxisInfo.nHardCtrlCardNo, tAxisInfo.nHardAxisNo, 0,
			dSpeed, dAcc, dDec, 0);
		CHECK_RTN_INT(nRtn);
		nRtn = dmc_set_s_profile(tAxisInfo.nHardCtrlCardNo, tAxisInfo.nHardAxisNo, 0, dSParam);
		CHECK_RTN_INT(nRtn);
		nRtn = dmc_pmove(tAxisInfo.nHardCtrlCardNo, tAxisInfo.nHardAxisNo, lDistance, nMode);
		CHECK_RTN_INT(nRtn);
	}
	else
	{
		nRtn = smc_set_profile_unit(tAxisInfo.nHardCtrlCardNo, tAxisInfo.nHardAxisNo, 0,
			dSpeed, dAcc, dDec, 0);
		CHECK_RTN_INT(nRtn);
		nRtn = smc_set_s_profile(tAxisInfo.nHardCtrlCardNo, tAxisInfo.nHardAxisNo, 0, dSParam);
		CHECK_RTN_INT(nRtn);
		nRtn = smc_pmove_unit(tAxisInfo.nHardCtrlCardNo, tAxisInfo.nHardAxisNo, lDistance, nMode);
		CHECK_RTN_INT(nRtn);
	}
	return nRtn;
}

int CServoMotorDriver::PosMove(int nSoftAxisNo, double dDistance, int nMode, double dSpeed, double dAcc, double dDec, double dSParam /*= 0.1*/)
{
	T_AXIS_CTRL_INFO tAxisInfo;
	T_CTRL_CARD_INFO tCtrlCardInfo;
	bool bRtn = GetAxisCardInfo(nSoftAxisNo, tAxisInfo, tCtrlCardInfo);
	if (!bRtn)
	{
		WRITE_LOG("获取轴卡信息失败");
		return INPUT_INFO_ERROR;
	}
	long lDistance = 0;
	tAxisInfo.dPulseEquivalent = fabs(tAxisInfo.dPulseEquivalent);
	if (tAxisInfo.dPulseEquivalent > 1)
	{
		dSpeed = dSpeed * tAxisInfo.dPulseEquivalent;
		lDistance = dDistance * tAxisInfo.dPulseEquivalent + 0.5;
	}
	else
	{
		dSpeed = dSpeed / tAxisInfo.dPulseEquivalent;
		lDistance = dDistance / tAxisInfo.dPulseEquivalent + 0.5;
	}
	int nRtn = 0;
	if (tCtrlCardInfo.eConnectType == E_CONNECT_TYPE_DIRECT)
	{
		nRtn = dmc_set_profile(tAxisInfo.nHardCtrlCardNo, tAxisInfo.nHardAxisNo, 0,
			dSpeed, dAcc, dDec, 0);
		CHECK_RTN_INT(nRtn);
		nRtn = dmc_set_s_profile(tAxisInfo.nHardCtrlCardNo, tAxisInfo.nHardAxisNo, 0, dSParam);
		CHECK_RTN_INT(nRtn);
		nRtn = dmc_pmove(tAxisInfo.nHardCtrlCardNo, tAxisInfo.nHardAxisNo, lDistance, nMode);
		CHECK_RTN_INT(nRtn);
	}
	else
	{
		nRtn = smc_set_profile_unit(tAxisInfo.nHardCtrlCardNo, tAxisInfo.nHardAxisNo, 0,
			dSpeed, dAcc, dDec, 0);
		CHECK_RTN_INT(nRtn);
		nRtn = smc_set_s_profile(tAxisInfo.nHardCtrlCardNo, tAxisInfo.nHardAxisNo, 0, dSParam);
		CHECK_RTN_INT(nRtn);
		nRtn = smc_pmove_unit(tAxisInfo.nHardCtrlCardNo, tAxisInfo.nHardAxisNo, lDistance, nMode);
		CHECK_RTN_INT(nRtn);
	}
	return nRtn;
}

int CServoMotorDriver::ContiMove(int nSoftAxisNo, int nDir, double dSpeed, double dAcc, double dSParam /*= 0.1*/)
{
	T_AXIS_CTRL_INFO tAxisInfo;
	T_CTRL_CARD_INFO tCtrlCardInfo;
	bool bRtn = GetAxisCardInfo(nSoftAxisNo, tAxisInfo, tCtrlCardInfo);
	if (!bRtn)
	{
		WRITE_LOG("获取轴卡信息失败");
		return INPUT_INFO_ERROR;
	}
	if (tAxisInfo.dPulseEquivalent > 1)
	{
		dSpeed = dSpeed * tAxisInfo.dPulseEquivalent;
	}
	else
	{
		dSpeed = dSpeed / tAxisInfo.dPulseEquivalent + 0.5;
	}
	int nRtn = 0;
	if (tCtrlCardInfo.eConnectType == E_CONNECT_TYPE_DIRECT)
	{
		nRtn = dmc_set_profile(tAxisInfo.nHardCtrlCardNo, tAxisInfo.nHardAxisNo, dSpeed / 4.0,
			dSpeed, dAcc, dAcc, 0);
		CHECK_RTN_INT(nRtn);
		nRtn = dmc_set_s_profile(tAxisInfo.nHardCtrlCardNo, tAxisInfo.nHardAxisNo, 0, dSParam);
		CHECK_RTN_INT(nRtn);
		nRtn = dmc_vmove(tAxisInfo.nHardCtrlCardNo, tAxisInfo.nHardAxisNo, nDir);
		CHECK_RTN_INT(nRtn);
	}
	else
	{
		nRtn = smc_set_profile_unit(tAxisInfo.nHardCtrlCardNo, tAxisInfo.nHardAxisNo, dSpeed / 4.0,
			dSpeed, dAcc, dAcc, 0);
		CHECK_RTN_INT(nRtn);
		nRtn = smc_set_s_profile(tAxisInfo.nHardCtrlCardNo, tAxisInfo.nHardAxisNo, 0, dSParam);
		CHECK_RTN_INT(nRtn);
		nRtn = smc_vmove(tAxisInfo.nHardCtrlCardNo, tAxisInfo.nHardAxisNo, nDir);
		CHECK_RTN_INT(nRtn);
	}
	return nRtn;
}

bool CServoMotorDriver::CheckAxisRun(int nSoftAxisNo, int *pnError /*= NULL*/)
{
	T_AXIS_CTRL_INFO tAxisInfo;
	T_CTRL_CARD_INFO tCtrlCardInfo;
	bool bRtn = GetAxisCardInfo(nSoftAxisNo, tAxisInfo, tCtrlCardInfo);
	if (!bRtn)
	{
		if (pnError != NULL)
		{
			*pnError = -1;
		}
		return false;
	}
	int nRtn = 0; 
	if (tCtrlCardInfo.eConnectType == E_CONNECT_TYPE_DIRECT)
	{
		nRtn = dmc_check_done(tAxisInfo.nHardCtrlCardNo, tAxisInfo.nHardAxisNo);
	}
	else
	{
		nRtn = smc_check_done(tAxisInfo.nHardCtrlCardNo, tAxisInfo.nHardAxisNo);
	}
	if (pnError != NULL)
	{
		*pnError = nRtn;
	}
	if (0 == nRtn)
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool CServoMotorDriver::EmgStop(int *nSize, int *pnError)
{
	int nCardNum = m_vtCtrlCardInfo.size();
	bool bRtn = true;
	int *nRtn;
	nRtn = new int[nCardNum];
	*nRtn = {0};
	for (int n = 0; n < nCardNum; n++)
	{
		switch (m_vtCtrlCardInfo[n].eCtrlCardType)
		{
		case E_CTRL_CARD_TYPE_DMC5410:
		case E_CTRL_CARD_TYPE_DMC5800:
		case E_CTRL_CARD_TYPE_DMC5810:
		{
			nRtn[n] = dmc_emg_stop(m_vtCtrlCardInfo[n].nHardCtrlCardNo);
			if (nRtn[n] != 0)
			{
				bRtn = false;
			}
			break;
		}
		case E_CTRL_CARD_TYPE_SMC604:
		{
			nRtn[n] = smc_emg_stop(m_vtCtrlCardInfo[n].nHardCtrlCardNo);
			if (nRtn[n] != 0)
			{
				bRtn = false;
			}
			break;
		}
		default:
			nRtn[n] = -1;
			break;
		}
	}
	if (nSize != NULL && pnError != NULL)
	{
		for (int n = 0; n < nCardNum; n++)
		{
			pnError[n] = nRtn[n];
		}
		*nSize = nCardNum;
	}
	delete[]nRtn;
	return bRtn;
}

int CServoMotorDriver::EmgStopCtrlCard(int nSoftCtrlCardNo)
{
	T_CTRL_CARD_INFO tCtrlCardInfo;
	bool bRtn = GetCtrlCardInfo(nSoftCtrlCardNo, tCtrlCardInfo);
	if (!bRtn)
	{
		WRITE_LOG("获取轴卡信息失败");
		return INPUT_INFO_ERROR;
	}
	int nRtn = 0;
	if (tCtrlCardInfo.eConnectType == E_CONNECT_TYPE_DIRECT)
	{
		nRtn = dmc_emg_stop(tCtrlCardInfo.nHardCtrlCardNo);
	}
	else
	{
		nRtn = smc_emg_stop(tCtrlCardInfo.nHardCtrlCardNo);
	}
	return nRtn;
}

int CServoMotorDriver::EmgStopAxis(int nSoftAxisNo)
{
	T_AXIS_CTRL_INFO tAxisInfo;
	T_CTRL_CARD_INFO tCtrlCardInfo;
	bool bRtn = GetAxisCardInfo(nSoftAxisNo, tAxisInfo, tCtrlCardInfo);
	if (!bRtn)
	{
		WRITE_LOG("获取轴卡信息失败");
		return INPUT_INFO_ERROR;
	}
	int nRtn = 0;
	if (tCtrlCardInfo.eConnectType == E_CONNECT_TYPE_DIRECT)
	{
		nRtn = dmc_stop(tAxisInfo.nHardCtrlCardNo, tAxisInfo.nHardAxisNo, 1);
	}
	else
	{
		nRtn = smc_stop(tAxisInfo.nHardCtrlCardNo, tAxisInfo.nHardAxisNo, 1);
	}
	return nRtn;
}

int CServoMotorDriver::SetDecelStopTime(int nSoftAxisNo, double dDec)
{
	T_AXIS_CTRL_INFO tAxisInfo;
	T_CTRL_CARD_INFO tCtrlCardInfo;
	bool bRtn = GetAxisCardInfo(nSoftAxisNo, tAxisInfo, tCtrlCardInfo);
	if (!bRtn)
	{
		WRITE_LOG("获取轴卡信息失败");
		return INPUT_INFO_ERROR;
	}
	int nRtn = 0;
	dDec = fabs(dDec);
	if (tCtrlCardInfo.eConnectType == E_CONNECT_TYPE_DIRECT)
	{
		nRtn = dmc_set_dec_stop_time(tAxisInfo.nHardCtrlCardNo, tAxisInfo.nHardAxisNo, dDec);
	}
	else
	{
		nRtn = smc_set_dec_stop_time(tAxisInfo.nHardCtrlCardNo, tAxisInfo.nHardAxisNo, dDec);
	}
	return nRtn;
}

int CServoMotorDriver::DecelStop(int nSoftAxisNo)
{
	T_AXIS_CTRL_INFO tAxisInfo;
	T_CTRL_CARD_INFO tCtrlCardInfo;
	bool bRtn = GetAxisCardInfo(nSoftAxisNo, tAxisInfo, tCtrlCardInfo);
	if (!bRtn)
	{
		WRITE_LOG("获取轴卡信息失败");
		return INPUT_INFO_ERROR;
	}
	int nRtn = 0;
	if (tCtrlCardInfo.eConnectType == E_CONNECT_TYPE_DIRECT)
	{
		nRtn = dmc_stop(tAxisInfo.nHardCtrlCardNo, tAxisInfo.nHardAxisNo, 0);
	}
	else
	{
		nRtn = smc_stop(tAxisInfo.nHardCtrlCardNo, tAxisInfo.nHardAxisNo, 0);
	}
	return nRtn;
}

int CServoMotorDriver::GetCurrentPosition(int nSoftAxisNo, long &lPosition)
{
	T_AXIS_CTRL_INFO tAxisInfo;
	T_CTRL_CARD_INFO tCtrlCardInfo;
	bool bRtn = GetAxisCardInfo(nSoftAxisNo, tAxisInfo, tCtrlCardInfo);
	if (!bRtn)
	{
		WRITE_LOG("获取轴卡信息失败");
		return INPUT_INFO_ERROR;
	}
	int nRtn = 0;
	double dCurrentPoeition = 0;
	if (tCtrlCardInfo.eConnectType == E_CONNECT_TYPE_DIRECT)
	{
		nRtn = dmc_get_position_unit(tAxisInfo.nHardCtrlCardNo, tAxisInfo.nHardAxisNo, &dCurrentPoeition);
	}
	else
	{
		nRtn = smc_get_position_unit(tAxisInfo.nHardCtrlCardNo, tAxisInfo.nHardAxisNo, &dCurrentPoeition);
	}
	lPosition = long(dCurrentPoeition + 0.5);
	return nRtn;
}

int CServoMotorDriver::SetCurrentPosition(int nSoftAxisNo, long lPosition)
{
	T_AXIS_CTRL_INFO tAxisInfo;
	T_CTRL_CARD_INFO tCtrlCardInfo;
	bool bRtn = GetAxisCardInfo(nSoftAxisNo, tAxisInfo, tCtrlCardInfo);
	if (!bRtn)
	{
		WRITE_LOG("获取轴卡信息失败");
		return INPUT_INFO_ERROR;
	}
	int nRtn = 0;
	if (tCtrlCardInfo.eConnectType == E_CONNECT_TYPE_DIRECT)
	{
		nRtn = dmc_set_position(tAxisInfo.nHardCtrlCardNo, tAxisInfo.nHardAxisNo, lPosition);
	}
	else
	{
		nRtn = smc_set_position_unit(tAxisInfo.nHardCtrlCardNo, tAxisInfo.nHardAxisNo, (double)lPosition);
	}
	return nRtn;
}

int CServoMotorDriver::GetCurrentPosition(int nSoftAxisNo, double &dPosition)
{
	T_AXIS_CTRL_INFO tAxisInfo;
	T_CTRL_CARD_INFO tCtrlCardInfo;
	bool bRtn = GetAxisCardInfo(nSoftAxisNo, tAxisInfo, tCtrlCardInfo);
	if (!bRtn)
	{
		WRITE_LOG("获取轴卡信息失败");
		return INPUT_INFO_ERROR;
	}
	int nRtn = 0;
	double dCurrentPoeition = 0;
	if (tCtrlCardInfo.eConnectType == E_CONNECT_TYPE_DIRECT)
	{
		nRtn = dmc_get_position_unit(tAxisInfo.nHardCtrlCardNo, tAxisInfo.nHardAxisNo, &dCurrentPoeition);
	}
	else
	{
		nRtn = smc_get_position_unit(tAxisInfo.nHardCtrlCardNo, tAxisInfo.nHardAxisNo, &dCurrentPoeition);
	}
	if (tAxisInfo.dPulseEquivalent > 1)
	{
		dPosition = dCurrentPoeition / tAxisInfo.dPulseEquivalent;
	}
	else
	{
		dPosition = dCurrentPoeition * tAxisInfo.dPulseEquivalent;
	}
	return nRtn;
}

int CServoMotorDriver::SetCurrentPosition(int nSoftAxisNo, double dPosition)
{
	T_AXIS_CTRL_INFO tAxisInfo;
	T_CTRL_CARD_INFO tCtrlCardInfo;
	bool bRtn = GetAxisCardInfo(nSoftAxisNo, tAxisInfo, tCtrlCardInfo);
	if (!bRtn)
	{
		WRITE_LOG("获取轴卡信息失败");
		return INPUT_INFO_ERROR;
	}
	int nRtn = 0;
	long lCurrentPoeition = 0;
	if (tAxisInfo.dPulseEquivalent > 1)
	{
		lCurrentPoeition = dPosition * tAxisInfo.dPulseEquivalent + 0.5;
	}
	else
	{
		lCurrentPoeition = dPosition / tAxisInfo.dPulseEquivalent + 0.5;
	}
	if (tCtrlCardInfo.eConnectType == E_CONNECT_TYPE_DIRECT)
	{
		nRtn = dmc_set_position(tAxisInfo.nHardCtrlCardNo, tAxisInfo.nHardAxisNo, lCurrentPoeition);
	}
	else
	{
		nRtn = smc_set_position_unit(tAxisInfo.nHardCtrlCardNo, tAxisInfo.nHardAxisNo, (double)lCurrentPoeition);
	}
	return nRtn;
}

int CServoMotorDriver::GetTargetPosition(int nSoftAxisNo, long &lPosition)
{
	T_AXIS_CTRL_INFO tAxisInfo;
	T_CTRL_CARD_INFO tCtrlCardInfo;
	bool bRtn = GetAxisCardInfo(nSoftAxisNo, tAxisInfo, tCtrlCardInfo);
	if (!bRtn)
	{
		WRITE_LOG("获取轴卡信息失败");
		return INPUT_INFO_ERROR;
	}
	int nRtn = 0;
	double dCurrentPoeition = 0;
	if (tCtrlCardInfo.eConnectType == E_CONNECT_TYPE_DIRECT)
	{
		nRtn = dmc_get_target_position_unit(tAxisInfo.nHardCtrlCardNo, tAxisInfo.nHardAxisNo, &dCurrentPoeition);
	}
	else
	{
		nRtn = smc_get_target_position_unit(tAxisInfo.nHardCtrlCardNo, tAxisInfo.nHardAxisNo, &dCurrentPoeition);
	}
	lPosition = long(dCurrentPoeition + 0.5);
	return nRtn;
}

int CServoMotorDriver::GetTargetPosition(int nSoftAxisNo, double &dPosition)
{
	T_AXIS_CTRL_INFO tAxisInfo;
	T_CTRL_CARD_INFO tCtrlCardInfo;
	bool bRtn = GetAxisCardInfo(nSoftAxisNo, tAxisInfo, tCtrlCardInfo);
	if (!bRtn)
	{
		WRITE_LOG("获取轴卡信息失败");
		return INPUT_INFO_ERROR;
	}
	int nRtn = 0;
	double dCurrentPoeition = 0;
	if (tCtrlCardInfo.eConnectType == E_CONNECT_TYPE_DIRECT)
	{
		nRtn = dmc_get_target_position_unit(tAxisInfo.nHardCtrlCardNo, tAxisInfo.nHardAxisNo, &dCurrentPoeition);
	}
	else
	{
		nRtn = smc_get_target_position_unit(tAxisInfo.nHardCtrlCardNo, tAxisInfo.nHardAxisNo, &dCurrentPoeition);
	}
	if (tAxisInfo.dPulseEquivalent > 1)
	{
		dPosition = dCurrentPoeition / tAxisInfo.dPulseEquivalent;
	}
	else
	{
		dPosition = dCurrentPoeition * tAxisInfo.dPulseEquivalent;
	}
	return nRtn;
}

int CServoMotorDriver::SetTargetPosition(int nSoftAxisNo, long lPosition, bool bupdate /*= false*/)
{
	T_AXIS_CTRL_INFO tAxisInfo;
	T_CTRL_CARD_INFO tCtrlCardInfo;
	bool bRtn = GetAxisCardInfo(nSoftAxisNo, tAxisInfo, tCtrlCardInfo);
	if (!bRtn)
	{
		WRITE_LOG("获取轴卡信息失败");
		return INPUT_INFO_ERROR;
	}
	int nRtn = 0;
	if (tCtrlCardInfo.eConnectType == E_CONNECT_TYPE_DIRECT)
	{
		if (bupdate)
		{
			nRtn = dmc_update_target_position(tAxisInfo.nHardCtrlCardNo, tAxisInfo.nHardAxisNo, lPosition, 0);
		}
		else
		{
			nRtn = dmc_reset_target_position(tAxisInfo.nHardCtrlCardNo, tAxisInfo.nHardAxisNo, lPosition, 0);
		}
	}
	else
	{
		if (bupdate)
		{
			nRtn = smc_update_target_position_unit(tAxisInfo.nHardCtrlCardNo, tAxisInfo.nHardAxisNo, (double)lPosition);
		}
		else
		{
			nRtn = smc_reset_target_position_unit(tAxisInfo.nHardCtrlCardNo, tAxisInfo.nHardAxisNo, (double)lPosition);
		}
	}
	return nRtn;
}

int CServoMotorDriver::SetTargetPosition(int nSoftAxisNo, double dPosition, bool bupdate /*= false*/)
{
	T_AXIS_CTRL_INFO tAxisInfo;
	T_CTRL_CARD_INFO tCtrlCardInfo;
	bool bRtn = GetAxisCardInfo(nSoftAxisNo, tAxisInfo, tCtrlCardInfo);
	if (!bRtn)
	{
		WRITE_LOG("获取轴卡信息失败");
		return INPUT_INFO_ERROR;
	}
	int nRtn = 0; 
	long lCurrentPoeition = 0;
	if (tAxisInfo.dPulseEquivalent > 1)
	{
		lCurrentPoeition = dPosition * tAxisInfo.dPulseEquivalent + 0.5;
	}
	else
	{
		lCurrentPoeition = dPosition / tAxisInfo.dPulseEquivalent + 0.5;
	}
	if (tCtrlCardInfo.eConnectType == E_CONNECT_TYPE_DIRECT)
	{
		if (bupdate)
		{
			nRtn = dmc_update_target_position(tAxisInfo.nHardCtrlCardNo, tAxisInfo.nHardAxisNo, lCurrentPoeition, 0);
		}
		else
		{
			nRtn = dmc_reset_target_position(tAxisInfo.nHardCtrlCardNo, tAxisInfo.nHardAxisNo, lCurrentPoeition, 0);
		}
	}
	else
	{
		if (bupdate)
		{
			nRtn = smc_update_target_position_unit(tAxisInfo.nHardCtrlCardNo, tAxisInfo.nHardAxisNo, (double)lCurrentPoeition);
		}
		else
		{
			nRtn = smc_reset_target_position_unit(tAxisInfo.nHardCtrlCardNo, tAxisInfo.nHardAxisNo, (double)lCurrentPoeition);
		}
	}
	return nRtn;
}

int CServoMotorDriver::GetSpeed(int nSoftAxisNo, long &lSpeed)
{
	T_AXIS_CTRL_INFO tAxisInfo;
	T_CTRL_CARD_INFO tCtrlCardInfo;
	bool bRtn = GetAxisCardInfo(nSoftAxisNo, tAxisInfo, tCtrlCardInfo);
	if (!bRtn)
	{
		WRITE_LOG("获取轴卡信息失败");
		return INPUT_INFO_ERROR;
	}
	int nRtn = 0;
	double dCurrentSpeed = 0;
	if (tCtrlCardInfo.eConnectType == E_CONNECT_TYPE_DIRECT)
	{
		nRtn = dmc_read_current_speed_unit(tAxisInfo.nHardCtrlCardNo, tAxisInfo.nHardAxisNo, &dCurrentSpeed);
	}
	else
	{
		nRtn = smc_read_current_speed_unit(tAxisInfo.nHardCtrlCardNo, tAxisInfo.nHardAxisNo, &dCurrentSpeed);
	}
	lSpeed = long(dCurrentSpeed + 0.5);
	return nRtn;
}

int CServoMotorDriver::GetSpeed(int nSoftAxisNo, double &dSpeed)
{
	T_AXIS_CTRL_INFO tAxisInfo;
	T_CTRL_CARD_INFO tCtrlCardInfo;
	bool bRtn = GetAxisCardInfo(nSoftAxisNo, tAxisInfo, tCtrlCardInfo);
	if (!bRtn)
	{
		WRITE_LOG("获取轴卡信息失败");
		return INPUT_INFO_ERROR;
	}
	int nRtn = 0;
	double dCurrentSpeed = 0;
	if (tCtrlCardInfo.eConnectType == E_CONNECT_TYPE_DIRECT)
	{
		nRtn = dmc_read_current_speed_unit(tAxisInfo.nHardCtrlCardNo, tAxisInfo.nHardAxisNo, &dCurrentSpeed);
	}
	else
	{
		nRtn = smc_read_current_speed_unit(tAxisInfo.nHardCtrlCardNo, tAxisInfo.nHardAxisNo, &dCurrentSpeed);
	}
	if (tAxisInfo.dPulseEquivalent > 1)
	{
		dSpeed = dCurrentSpeed / tAxisInfo.dPulseEquivalent;
	}
	else
	{
		dSpeed = dCurrentSpeed * tAxisInfo.dPulseEquivalent;
	}
	return nRtn;
}

int CServoMotorDriver::ChangeSpeed(int nSoftAxisNo, long lSpeed, double dChangeTime)
{
	T_AXIS_CTRL_INFO tAxisInfo;
	T_CTRL_CARD_INFO tCtrlCardInfo;
	bool bRtn = GetAxisCardInfo(nSoftAxisNo, tAxisInfo, tCtrlCardInfo);
	if (!bRtn)
	{
		WRITE_LOG("获取轴卡信息失败");
		return INPUT_INFO_ERROR;
	}
	int nRtn = 0;
	if (tCtrlCardInfo.eConnectType == E_CONNECT_TYPE_DIRECT)
	{
		nRtn = dmc_change_speed(tAxisInfo.nHardCtrlCardNo, tAxisInfo.nHardAxisNo, lSpeed, dChangeTime);
	}
	else
	{
		nRtn = smc_change_speed_unit(tAxisInfo.nHardCtrlCardNo, tAxisInfo.nHardAxisNo, (double)lSpeed, dChangeTime);
	}
	return nRtn;
}

int CServoMotorDriver::ChangeSpeed(int nSoftAxisNo, double dSpeed, double dChangeTime)
{
	T_AXIS_CTRL_INFO tAxisInfo;
	T_CTRL_CARD_INFO tCtrlCardInfo;
	bool bRtn = GetAxisCardInfo(nSoftAxisNo, tAxisInfo, tCtrlCardInfo);
	if (!bRtn)
	{
		WRITE_LOG("获取轴卡信息失败");
		return INPUT_INFO_ERROR;
	}
	int nRtn = 0;
	long lSpeed;
	if (tAxisInfo.dPulseEquivalent > 1)
	{
		lSpeed = dSpeed * tAxisInfo.dPulseEquivalent + 0.5;
	}
	else
	{
		lSpeed = dSpeed / tAxisInfo.dPulseEquivalent + 0.5;
	}
	if (tCtrlCardInfo.eConnectType == E_CONNECT_TYPE_DIRECT)
	{
		nRtn = dmc_change_speed(tAxisInfo.nHardCtrlCardNo, tAxisInfo.nHardAxisNo, lSpeed, dChangeTime);
	}
	else
	{
		nRtn = smc_change_speed_unit(tAxisInfo.nHardCtrlCardNo, tAxisInfo.nHardAxisNo, (double)lSpeed, dChangeTime);
	}
	return nRtn;
}

double CServoMotorDriver::GetMaxSpeed(int nSoftAxisNo)
{
	T_AXIS_CTRL_INFO tAxisInfo;
	T_CTRL_CARD_INFO tCtrlCardInfo;
	bool bRtn = GetAxisCardInfo(nSoftAxisNo, tAxisInfo, tCtrlCardInfo);
	return tAxisInfo.tMaxSpeed.dSpeed;
}

int CServoMotorDriver::GetDataPulseEquivalent(int nSoftAxisNo, double &dPulseEquivalent)
{
	T_AXIS_CTRL_INFO tAxisInfo;
	T_CTRL_CARD_INFO tCtrlCardInfo;
	bool bRtn = GetAxisCardInfo(nSoftAxisNo, tAxisInfo, tCtrlCardInfo);
	if (!bRtn)
	{
		WRITE_LOG("获取轴卡信息失败");
		return INPUT_INFO_ERROR;
	}
	dPulseEquivalent = tAxisInfo.dPulseEquivalent;
	return 0;
}

int CServoMotorDriver::GetDataSpeedParam(int nSoftAxisNo, T_AXIS_SPEED &tMinSpeedParam, T_AXIS_SPEED &tMidSpeedParam, T_AXIS_SPEED &tMaxSpeedParam)
{
	T_AXIS_CTRL_INFO tAxisInfo;
	T_CTRL_CARD_INFO tCtrlCardInfo;
	bool bRtn = GetAxisCardInfo(nSoftAxisNo, tAxisInfo, tCtrlCardInfo);
	if (!bRtn)
	{
		WRITE_LOG("获取轴卡信息失败");
		return INPUT_INFO_ERROR;
	}
	tMinSpeedParam = tAxisInfo.tMinSpeed;
	tMidSpeedParam = tAxisInfo.tMidSpeed;
	tMaxSpeedParam = tAxisInfo.tMaxSpeed;
	return 0;
}

int CServoMotorDriver::SetGearFollowProfile(int nMasterSoftAxisNo, int nDrivenSoftAxisNo, bool bEnable, double dRatio /*= 1.0*/)
{
	T_AXIS_CTRL_INFO tMasterAxisInfo, tDrivenAxisInfo;
	T_CTRL_CARD_INFO tCtrlCardInfo;
	bool bRtn = GetAxisCardInfo(nMasterSoftAxisNo, tMasterAxisInfo, tCtrlCardInfo);
	bRtn = bRtn && GetAxisCardInfo(nDrivenSoftAxisNo, tDrivenAxisInfo, tCtrlCardInfo);
	if (tMasterAxisInfo.nSoftCtrlCardNo != tDrivenAxisInfo.nSoftCtrlCardNo || !bRtn)
	{
		WRITE_LOG("获取轴卡信息失败");
		return INPUT_INFO_ERROR;
	}
	int nRtn = 0;
	if (tCtrlCardInfo.eConnectType == E_CONNECT_TYPE_DIRECT)
	{
		nRtn = dmc_set_gear_follow_profile(tMasterAxisInfo.nHardCtrlCardNo, tDrivenAxisInfo.nHardAxisNo, bEnable,
			tMasterAxisInfo.nHardAxisNo, dRatio);
	}
	else
	{
		WRITE_LOG("当前轴类型没有跟随功能");
		return NO_FOLLOW_FUN;
	}
	return nRtn;
}

int CServoMotorDriver::GetGearFollowProfile(int nDrivenSoftAxisNo, int &nMasterSoftAxisNo, bool &bEnable, double &dRatio)
{
	T_AXIS_CTRL_INFO tAxisInfo;
	T_CTRL_CARD_INFO tCtrlCardInfo;
	bool bRtn = GetAxisCardInfo(nDrivenSoftAxisNo, tAxisInfo, tCtrlCardInfo);
	if (!bRtn)
	{
		WRITE_LOG("获取轴卡信息失败");
		return INPUT_INFO_ERROR;
	}
	int nRtn = 0;
	WORD nEnable, nMasterAxisNo;
	if (tCtrlCardInfo.eConnectType == E_CONNECT_TYPE_DIRECT)
	{
		nRtn = dmc_get_gear_follow_profile(tAxisInfo.nHardCtrlCardNo, tAxisInfo.nHardAxisNo, &nEnable,
			&nMasterAxisNo, &dRatio);
	}
	else
	{
		WRITE_LOG("当前轴类型没有跟随功能");
		return NO_FOLLOW_FUN;
	}
	T_AXIS_CTRL_INFO tTempAxisInfo;
	T_CTRL_CARD_INFO tTempCtrlCardInfo;
	bRtn = GetAxisCardInfo(tAxisInfo.nHardCtrlCardNo, nMasterAxisNo, tCtrlCardInfo.eCtrlCardType, tTempAxisInfo, tTempCtrlCardInfo);
	if (!bRtn)
	{
		WRITE_LOG("获取轴卡信息失败");
		return INPUT_INFO_ERROR;
	}
	nMasterSoftAxisNo = tTempAxisInfo.nSoftAxisNo;
	if (nEnable == 0)
	{
		bEnable = false;
	}
	else
	{
		bEnable = true;
	}
	return nRtn;
}

int CServoMotorDriver::ReadInbit(int nSoftCtrlCardNo, int bBitNo, WORD &nStatus, int nNoteID /*= 0*/)
{
	T_CTRL_CARD_INFO tCtrlCardInfo;
	bool bRtn = GetCtrlCardInfo(nSoftCtrlCardNo, tCtrlCardInfo);	
	if (!bRtn)
	{
		WRITE_LOG("获取轴卡信息失败");
		return INPUT_INFO_ERROR;
	}
	int nRtn = 0;
	switch (tCtrlCardInfo.eCtrlCardType)
	{
	case E_CTRL_CARD_TYPE_DMC5410:
	case E_CTRL_CARD_TYPE_DMC5800:
	case E_CTRL_CARD_TYPE_DMC5810:
	{
		if (nNoteID > 0)
		{
			if (tCtrlCardInfo.tIOCtrlInfo.eIOType == E_IO_CARD_TYPE_CAN)
			{
				nStatus = dmc_read_can_inbit(tCtrlCardInfo.nHardCtrlCardNo, nNoteID, bBitNo);
			}
			else if (tCtrlCardInfo.tIOCtrlInfo.eIOType == E_IO_CARD_TYPE_CAT)
			{
				bBitNo = tCtrlCardInfo.tIOCtrlInfo.nLocalIONum + (nNoteID - 1)*tCtrlCardInfo.tIOCtrlInfo.nSingleIONum + bBitNo;
				nStatus = dmc_read_inbit(tCtrlCardInfo.nHardCtrlCardNo, bBitNo);
			}
			else
			{
				WRITE_LOG("IO信息错误");
				return INPUT_INFO_ERROR;
			}
			CHECK_RTN_INT(nRtn);
		}
		else
			nStatus = dmc_read_inbit(tCtrlCardInfo.nHardCtrlCardNo, bBitNo);
		break;
	}
	case E_CTRL_CARD_TYPE_SMC604:
	{
		if (nNoteID > 0)
			nRtn = nmcs_read_inbit(tCtrlCardInfo.nHardCtrlCardNo, tCtrlCardInfo.tIOCtrlInfo.nPortNo, nNoteID, bBitNo, &nStatus);
		else
			nStatus = smc_read_inbit(tCtrlCardInfo.nHardCtrlCardNo, bBitNo);
		CHECK_RTN_INT(nRtn);
		break;
	}
	case E_CTRL_CARD_TYPE_IOC0640:
	case E_CTRL_CARD_TYPE_IOC1280:
	{
		nStatus = ioc_read_inbit(tCtrlCardInfo.nHardCtrlCardNo, bBitNo);
		break;
	}
	default:
		return NO_FUNITION;
		break;
	}
	return 0;
}

int CServoMotorDriver::ReadOutbit(int nSoftCtrlCardNo, int bBitNo, WORD &nStatus, int nNoteID /*= 0*/)
{
	T_CTRL_CARD_INFO tCtrlCardInfo;
	bool bRtn = GetCtrlCardInfo(nSoftCtrlCardNo, tCtrlCardInfo);
	if (!bRtn)
	{
		WRITE_LOG("获取轴卡信息失败");
		return INPUT_INFO_ERROR;
	}
	int nRtn = 0;
	switch (tCtrlCardInfo.eCtrlCardType)
	{
	case E_CTRL_CARD_TYPE_DMC5410:
	case E_CTRL_CARD_TYPE_DMC5800:
	case E_CTRL_CARD_TYPE_DMC5810:
	{
		if (nNoteID > 0)
		{
			if (tCtrlCardInfo.tIOCtrlInfo.eIOType == E_IO_CARD_TYPE_CAN)
			{
				nStatus = dmc_read_can_outbit(tCtrlCardInfo.nHardCtrlCardNo, nNoteID, bBitNo);
			}
			else if (tCtrlCardInfo.tIOCtrlInfo.eIOType == E_IO_CARD_TYPE_CAT)
			{
				bBitNo = tCtrlCardInfo.tIOCtrlInfo.nLocalIONum + (nNoteID - 1)*tCtrlCardInfo.tIOCtrlInfo.nSingleIONum + bBitNo;
				nStatus = dmc_read_outbit(tCtrlCardInfo.nHardCtrlCardNo, bBitNo);
			}
			else
			{
				WRITE_LOG("IO信息错误");
				return INPUT_INFO_ERROR;
			}
			CHECK_RTN_INT(nRtn);
		}
		else
			nStatus = dmc_read_outbit(tCtrlCardInfo.nHardCtrlCardNo, bBitNo);
		break;
	}
	case E_CTRL_CARD_TYPE_SMC604:
	{
		if (nNoteID > 0)
			nRtn = nmcs_read_outbit(tCtrlCardInfo.nHardCtrlCardNo, tCtrlCardInfo.tIOCtrlInfo.nPortNo, nNoteID, bBitNo, &nStatus);
		else
			nStatus = smc_read_outbit(tCtrlCardInfo.nHardCtrlCardNo, bBitNo);
		CHECK_RTN_INT(nRtn);
		break;
	}
	case E_CTRL_CARD_TYPE_IOC0640:
	case E_CTRL_CARD_TYPE_IOC1280:
	{
		nStatus = ioc_read_outbit(tCtrlCardInfo.nHardCtrlCardNo, bBitNo);
		break;
	}
	default:
		return NO_FUNITION;
		break;
	}
	return 0;
}

int CServoMotorDriver::WriteOutbit(int nSoftCtrlCardNo, int bBitNo, WORD nStatus, int nNoteID /*= 0*/)
{
	T_CTRL_CARD_INFO tCtrlCardInfo;
	bool bRtn = GetCtrlCardInfo(nSoftCtrlCardNo, tCtrlCardInfo);
	if (!bRtn)
	{
		WRITE_LOG("获取轴卡信息失败");
		return INPUT_INFO_ERROR;
	}
	int nRtn = 0;
	switch (tCtrlCardInfo.eCtrlCardType)
	{
	case E_CTRL_CARD_TYPE_DMC5410:
	case E_CTRL_CARD_TYPE_DMC5800:
	case E_CTRL_CARD_TYPE_DMC5810:
	{
		if (nNoteID > 0)
		{
			if (tCtrlCardInfo.tIOCtrlInfo.eIOType == E_IO_CARD_TYPE_CAN)
			{
				nRtn = dmc_write_can_outbit(tCtrlCardInfo.nHardCtrlCardNo, nNoteID, bBitNo, nStatus);
			}
			else if (tCtrlCardInfo.tIOCtrlInfo.eIOType == E_IO_CARD_TYPE_CAT)
			{
				bBitNo = tCtrlCardInfo.tIOCtrlInfo.nLocalIONum + (nNoteID - 1)*tCtrlCardInfo.tIOCtrlInfo.nSingleIONum + bBitNo;
				nRtn = dmc_write_outbit(tCtrlCardInfo.nHardCtrlCardNo, bBitNo, nStatus);
			}
			else
			{
				WRITE_LOG("IO信息错误");
				return INPUT_INFO_ERROR;
			}
		}
		else
			nRtn = dmc_write_outbit(tCtrlCardInfo.nHardCtrlCardNo, bBitNo, nStatus);
		CHECK_RTN_INT(nRtn);
		break;
	}
	case E_CTRL_CARD_TYPE_SMC604:
	{
		if (nNoteID > 0)
			nRtn = nmcs_write_outbit(tCtrlCardInfo.nHardCtrlCardNo, tCtrlCardInfo.tIOCtrlInfo.nPortNo, nNoteID, bBitNo, nStatus);
		else
			nRtn = smc_write_outbit(tCtrlCardInfo.nHardCtrlCardNo, bBitNo, nStatus);
		CHECK_RTN_INT(nRtn);
		break;
	}
	case E_CTRL_CARD_TYPE_IOC0640:
	case E_CTRL_CARD_TYPE_IOC1280:
	{
		nRtn = ioc_write_outbit(tCtrlCardInfo.nHardCtrlCardNo, bBitNo, nStatus);
		CHECK_RTN_INT(nRtn);
		break;
	}
	default:
		return NO_FUNITION;
		break;
	}
	return 0;
}

int CServoMotorDriver::SetADMode(int nSoftCtrlCardNo, int nNoteID, int nChannel, int nMode)
{
	T_CTRL_CARD_INFO tCtrlCardInfo;
	bool bRtn = GetCtrlCardInfo(nSoftCtrlCardNo, tCtrlCardInfo);
	if (!bRtn || nChannel < 0 || nChannel > 3)
	{
		return INPUT_INFO_ERROR;
	}
	int nRtn = 0;
	switch (tCtrlCardInfo.eCtrlCardType)
	{
	case E_CTRL_CARD_TYPE_DMC5410:
	case E_CTRL_CARD_TYPE_DMC5800:
	case E_CTRL_CARD_TYPE_DMC5810:
	{
		if (nNoteID > 0 && tCtrlCardInfo.tIOCtrlInfo.eIOType == E_IO_CARD_TYPE_CAN)
		{
			WORD nState;
			nRtn = nmc_set_ad_mode_ex(tCtrlCardInfo.nHardCtrlCardNo, nNoteID, nChannel, nMode, 1, &nState);
			if (nState != 1)
			{
				return CAN_DISCONNECT;
			}
		}
		else
			return NO_FUNITION;
		CHECK_RTN_INT(nRtn);
		break;
	}
	case E_CTRL_CARD_TYPE_SMC604:
	case E_CTRL_CARD_TYPE_IOC0640:
	case E_CTRL_CARD_TYPE_IOC1280:
	default:
		return NO_FUNITION;
		break;
	}
	return 0;
}

int CServoMotorDriver::SetDAMode(int nSoftCtrlCardNo, int nNoteID, int nChannel, int nMode)
{
	T_CTRL_CARD_INFO tCtrlCardInfo;
	bool bRtn = GetCtrlCardInfo(nSoftCtrlCardNo, tCtrlCardInfo);
	if (!bRtn || nChannel < 0 || nChannel > 1)
	{
		return INPUT_INFO_ERROR;
	}
	int nRtn = 0;
	switch (tCtrlCardInfo.eCtrlCardType)
	{
	case E_CTRL_CARD_TYPE_DMC5410:
	case E_CTRL_CARD_TYPE_DMC5800:
	case E_CTRL_CARD_TYPE_DMC5810:
	{
		if (nNoteID > 0 && tCtrlCardInfo.tIOCtrlInfo.eIOType == E_IO_CARD_TYPE_CAN)
		{
			WORD nState;
			nRtn = nmc_set_da_mode_ex(tCtrlCardInfo.nHardCtrlCardNo, nNoteID, nChannel, nMode, 1, &nState);
			if (nState != 1)
			{
				return CAN_DISCONNECT;
			}
		}
		else
			return NO_FUNITION;
		CHECK_RTN_INT(nRtn);
		break;
	}
	case E_CTRL_CARD_TYPE_SMC604:
	case E_CTRL_CARD_TYPE_IOC0640:
	case E_CTRL_CARD_TYPE_IOC1280:
	default:
		return NO_FUNITION;
		break;
	}
	return 0;
}

int CServoMotorDriver::SetDAChannel(int nSoftCtrlCardNo, int nNoteID, int nChannel, double dValue)
{
	T_CTRL_CARD_INFO tCtrlCardInfo;
	bool bRtn = GetCtrlCardInfo(nSoftCtrlCardNo, tCtrlCardInfo);
	if (!bRtn || nChannel < 0 || nChannel > 1)
	{
		return INPUT_INFO_ERROR;
	}
	int nRtn = 0;
	switch (tCtrlCardInfo.eCtrlCardType)
	{
	case E_CTRL_CARD_TYPE_DMC5410:
	case E_CTRL_CARD_TYPE_DMC5800:
	case E_CTRL_CARD_TYPE_DMC5810:
	{
		if (nNoteID > 0 && tCtrlCardInfo.tIOCtrlInfo.eIOType == E_IO_CARD_TYPE_CAN)
		{
			WORD nState;
			nRtn = nmc_set_da_output_ex(tCtrlCardInfo.nHardCtrlCardNo, nNoteID, nChannel, dValue, &nState);
			if (nState != 1)
			{
				return CAN_DISCONNECT;
			}
		}
		else
			return NO_FUNITION;
		CHECK_RTN_INT(nRtn);
		break;
	}
	case E_CTRL_CARD_TYPE_SMC604:
	case E_CTRL_CARD_TYPE_IOC0640:
	case E_CTRL_CARD_TYPE_IOC1280:
	default:
		return NO_FUNITION;
		break;
	}
	return 0;
}

int CServoMotorDriver::GetDAChannel(int nSoftCtrlCardNo, int nNoteID, int nChannel, double &dValue)
{
	T_CTRL_CARD_INFO tCtrlCardInfo;
	bool bRtn = GetCtrlCardInfo(nSoftCtrlCardNo, tCtrlCardInfo);
	if (!bRtn || nChannel < 0 || nChannel > 1)
	{
		return INPUT_INFO_ERROR;
	}
	int nRtn = 0;
	switch (tCtrlCardInfo.eCtrlCardType)
	{
	case E_CTRL_CARD_TYPE_DMC5410:
	case E_CTRL_CARD_TYPE_DMC5800:
	case E_CTRL_CARD_TYPE_DMC5810:
	{
		if (nNoteID > 0 && tCtrlCardInfo.tIOCtrlInfo.eIOType == E_IO_CARD_TYPE_CAN)
		{
			WORD nState;
			nRtn = nmc_get_da_output_ex(tCtrlCardInfo.nHardCtrlCardNo, nNoteID, nChannel, &dValue, &nState);
			if (nState != 1)
			{
				return CAN_DISCONNECT;
			}
		}
		else
			return NO_FUNITION;
		CHECK_RTN_INT(nRtn);
		break;
	}
	case E_CTRL_CARD_TYPE_SMC604:
	case E_CTRL_CARD_TYPE_IOC0640:
	case E_CTRL_CARD_TYPE_IOC1280:
	default:
		return NO_FUNITION;
		break;
	}
	return 0;
}

int CServoMotorDriver::GetADChannel(int nSoftCtrlCardNo, int nNoteID, int nChannel, double &dValue)
{
	T_CTRL_CARD_INFO tCtrlCardInfo;
	bool bRtn = GetCtrlCardInfo(nSoftCtrlCardNo, tCtrlCardInfo);
	if (!bRtn || nChannel < 0 || nChannel > 3)
	{
		return INPUT_INFO_ERROR;
	}
	int nRtn = 0;
	switch (tCtrlCardInfo.eCtrlCardType)
	{
	case E_CTRL_CARD_TYPE_DMC5410:
	case E_CTRL_CARD_TYPE_DMC5800:
	case E_CTRL_CARD_TYPE_DMC5810:
	{
		if (nNoteID > 0 && tCtrlCardInfo.tIOCtrlInfo.eIOType == E_IO_CARD_TYPE_CAN)
		{
			WORD nState;
			nRtn = nmc_get_ad_input_ex(tCtrlCardInfo.nHardCtrlCardNo, nNoteID, nChannel, &dValue, &nState);
			if (nState != 1)
			{
				return CAN_DISCONNECT;
			}
		}
		else
			return NO_FUNITION;
		CHECK_RTN_INT(nRtn);
		break;
	}
	case E_CTRL_CARD_TYPE_SMC604:
	case E_CTRL_CARD_TYPE_IOC0640:
	case E_CTRL_CARD_TYPE_IOC1280:
	default:
		return NO_FUNITION;
		break;
	}
	return 0;
}

int CServoMotorDriver::OpenAbsEncoder(int nSoftAxisNo)
{
	T_AXIS_CTRL_INFO tAxisInfo;
	T_CTRL_CARD_INFO tCtrlCardInfo;
	bool bRtn = GetAxisCardInfo(nSoftAxisNo, tAxisInfo, tCtrlCardInfo);
	if (!bRtn)
	{
		WRITE_LOG("获取轴卡信息失败");
		return INPUT_INFO_ERROR;
	}
	if (!tAxisInfo.bEnableAbsEncoder)
	{
		return ABS_ENCODER_DISENABLE;
	}
	auto it = m_mAbsEncoderCtrl.find(tAxisInfo.nAbsEncoderPartNo);
	if (nSoftAxisNo >= m_nAxisNum || it == m_mAbsEncoderCtrl.end())
	{
		return INPUT_INFO_ERROR;
	}
	if (m_mAbsEncoderCtrl[tAxisInfo.nAbsEncoderPartNo] == NULL)
	{
		m_mAbsEncoderCtrl[tAxisInfo.nAbsEncoderPartNo] = new CAbsoluteEncoder;
		if (FALSE == m_mAbsEncoderCtrl[tAxisInfo.nAbsEncoderPartNo]->InitPort(tAxisInfo.nAbsEncoderPartNo))
		{
			return SERIAL_PORT_DISCONNECT;
		}
	}
	return 0;
}

int CServoMotorDriver::CloseAbsEncoder(int nSoftAxisNo)
{
	T_AXIS_CTRL_INFO tAxisInfo;
	T_CTRL_CARD_INFO tCtrlCardInfo;
	bool bRtn = GetAxisCardInfo(nSoftAxisNo, tAxisInfo, tCtrlCardInfo);
	if (!bRtn)
	{
		WRITE_LOG("获取轴卡信息失败");
		return INPUT_INFO_ERROR;
	}
	if (!tAxisInfo.bEnableAbsEncoder)
	{
		return ABS_ENCODER_DISENABLE;
	}
	auto it = m_mAbsEncoderCtrl.find(tAxisInfo.nAbsEncoderPartNo);
	if (nSoftAxisNo >= m_nAxisNum || it == m_mAbsEncoderCtrl.end())
	{
		return INPUT_INFO_ERROR;
	}
	if (m_mAbsEncoderCtrl[tAxisInfo.nAbsEncoderPartNo] == NULL)
	{
		WRITE_LOG("没有初始化绝对编码器驱动");
		return INPUT_INFO_ERROR;
	}
	m_mAbsEncoderCtrl[tAxisInfo.nAbsEncoderPartNo]->ClosePort();
	delete m_mAbsEncoderCtrl[tAxisInfo.nAbsEncoderPartNo];
	m_mAbsEncoderCtrl[tAxisInfo.nAbsEncoderPartNo] = NULL;
	return 0;
}

int CServoMotorDriver::GetAbsData(int nSoftAxisNo, double &dAbsData)
{
	T_AXIS_CTRL_INFO tAxisInfo;
	T_CTRL_CARD_INFO tCtrlCardInfo;
	bool bRtn = GetAxisCardInfo(nSoftAxisNo, tAxisInfo, tCtrlCardInfo);
	if (!bRtn)
	{
		WRITE_LOG("获取轴卡信息失败");
		return INPUT_INFO_ERROR;
	}
	if (!tAxisInfo.bEnableAbsEncoder)
	{
		return ABS_ENCODER_DISENABLE;
	}
	auto it = m_mAbsEncoderCtrl.find(tAxisInfo.nAbsEncoderPartNo);
	if (nSoftAxisNo >= m_nAxisNum || it == m_mAbsEncoderCtrl.end())
	{
		return INPUT_INFO_ERROR;
	}
	if (m_mAbsEncoderCtrl[tAxisInfo.nAbsEncoderPartNo] == NULL)
	{
		WRITE_LOG("没有初始化绝对编码器驱动");
		return INPUT_INFO_ERROR;
	}double dPulseEquivalent = tAxisInfo.dPulseEquivalent;
	if (tAxisInfo.dPulseEquivalent>1)
	{
		dPulseEquivalent = 1.0 / tAxisInfo.dPulseEquivalent;
	}
	dAbsData = m_mAbsEncoderCtrl[tAxisInfo.nAbsEncoderPartNo]->GetCurrentCoorFromAbsEncoder(dPulseEquivalent, tAxisInfo.nHardAxisNo, tAxisInfo.nLapPulse);
	dAbsData = dAbsData * (double)tAxisInfo.nAbsEncoderDir;
	return 0;
}

int CServoMotorDriver::ClearManyLapData(int nSoftAxisNo)
{
	T_AXIS_CTRL_INFO tAxisInfo;
	T_CTRL_CARD_INFO tCtrlCardInfo;
	bool bRtn = GetAxisCardInfo(nSoftAxisNo, tAxisInfo, tCtrlCardInfo);
	if (!bRtn)
	{
		WRITE_LOG("获取轴卡信息失败");
		return INPUT_INFO_ERROR;
	}
	if (!tAxisInfo.bEnableAbsEncoder)
	{
		return ABS_ENCODER_DISENABLE;
	}
	auto it = m_mAbsEncoderCtrl.find(tAxisInfo.nAbsEncoderPartNo);
	if (nSoftAxisNo >= m_nAxisNum || it == m_mAbsEncoderCtrl.end())
	{
		return INPUT_INFO_ERROR;
	}
	if (m_mAbsEncoderCtrl[tAxisInfo.nAbsEncoderPartNo] == NULL)
	{
		WRITE_LOG("没有初始化绝对编码器驱动");
		return INPUT_INFO_ERROR;
	}
	if (FALSE == m_mAbsEncoderCtrl[tAxisInfo.nAbsEncoderPartNo]->ClearManyLapData(tAxisInfo.nHardAxisNo))
	{
		return SERIAL_PORT_DISCONNECT;
	}
	return 0;
}

int CServoMotorDriver::SetInitLapData(int nSoftAxisNo)
{
	T_AXIS_CTRL_INFO tAxisInfo;
	T_CTRL_CARD_INFO tCtrlCardInfo;
	bool bRtn = GetAxisCardInfo(nSoftAxisNo, tAxisInfo, tCtrlCardInfo);
	if (!bRtn)
	{
		WRITE_LOG("获取轴卡信息失败");
		return INPUT_INFO_ERROR;
	}
	if (!tAxisInfo.bEnableAbsEncoder)
	{
		return ABS_ENCODER_DISENABLE;
	}
	auto it = m_mAbsEncoderCtrl.find(tAxisInfo.nAbsEncoderPartNo);
	if (nSoftAxisNo >= m_nAxisNum || it == m_mAbsEncoderCtrl.end())
	{
		return INPUT_INFO_ERROR;
	}
	if (m_mAbsEncoderCtrl[tAxisInfo.nAbsEncoderPartNo] == NULL)
	{
		WRITE_LOG("没有初始化绝对编码器驱动");
		return INPUT_INFO_ERROR;
	}
	if (FALSE == m_mAbsEncoderCtrl[tAxisInfo.nAbsEncoderPartNo]->SetInitLapData(tAxisInfo.nHardAxisNo))
	{
		return SERIAL_PORT_DISCONNECT;
	}
	return 0;
}

double CServoMotorDriver::GetPulseEquivalent(int nSoftAxisNo)
{
	T_AXIS_CTRL_INFO tAxisInfo;
	T_CTRL_CARD_INFO tCtrlCardInfo;
	bool bRtn = GetAxisCardInfo(nSoftAxisNo, tAxisInfo, tCtrlCardInfo);
	if (!bRtn)
	{
		WRITE_LOG("获取轴卡信息失败");
		return INPUT_INFO_ERROR;
	}
	long lDistance = 0;
	tAxisInfo.dPulseEquivalent = fabs(tAxisInfo.dPulseEquivalent);
	if (tAxisInfo.dPulseEquivalent > 1)
	{
		tAxisInfo.dPulseEquivalent = 1 / tAxisInfo.dPulseEquivalent;
	}
	return tAxisInfo.dPulseEquivalent;
}

