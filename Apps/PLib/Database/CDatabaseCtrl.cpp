#include "stdafx.h"
#include ".\Apps\PLib\Database\CDatabaseCtrl.h"

#if ENABLE_MY_SQL

CDatabaseCtrl::CDatabaseCtrl(CString strIP, CString strUser, CString strKey, CString strDBName, int nPort)
{
	int nRtn = Connect(strIP, strUser, strKey, strDBName, nPort);
	if (nRtn != 0)
	{
		XiMessageBox(GetErrorInfo());
	}

}

CDatabaseCtrl::CDatabaseCtrl()
{
	int nRtn = Connect("127.0.0.1", "root", "123456", "sys", 3306);
	if (nRtn != 0)
	{
		XiMessageBox(GetErrorInfo());
		return;
	}
	nRtn = CreateDatabases("offline_debug");
	if (nRtn != 0)
	{
		XiMessageBox(GetErrorInfo());
		return;
	}
	nRtn = UseDatabases("offline_debug");
	if (nRtn != 0)
	{
		XiMessageBox(GetErrorInfo());
	}

}

//运动类型-运动坐标（9）-运动速度
BOOL CDatabaseCtrl::CreatTable_MoveValue()
{
	std::vector<T_FIELD_INFO> vtFieldInfo;
	T_FIELD_INFO tTemp;
	tTemp.nType = 3;
	tTemp.nLenght = 20;
	tTemp.strDefault = "null";
	tTemp.strName = "move_type";
	tTemp.strExplanatoryNote = "运动类型";
	vtFieldInfo.push_back(tTemp);

	tTemp.strName = "move_device";
	tTemp.strExplanatoryNote = "运动设备";
	vtFieldInfo.push_back(tTemp);

	tTemp.nLenght = 50;
	tTemp.strName = "unit_name";
	tTemp.strExplanatoryNote = "控制单元编号";
	vtFieldInfo.push_back(tTemp);

	tTemp.nType = 2;
	tTemp.nLenght = -1;
	tTemp.strDefault = "";
	tTemp.strName = "move_speed";
	tTemp.strExplanatoryNote = "运动速度";
	vtFieldInfo.push_back(tTemp);

	tTemp.nType = 0;
	tTemp.strName = "move_step";
	tTemp.strExplanatoryNote = "运动步数";
	vtFieldInfo.push_back(tTemp);

	tTemp.strName = "value_type";
	tTemp.strExplanatoryNote = "变量类型";
	vtFieldInfo.push_back(tTemp);

	for (size_t i = 0; i < 9; i++)
	{
		tTemp.strName.Format("pulse_%d", i);
		tTemp.strExplanatoryNote = tTemp.strName;
		vtFieldInfo.push_back(tTemp);
	}
	tTemp.nType = 2;
	for (size_t i = 0; i < 9; i++)
	{
		tTemp.strName.Format("coor_%d", i);
		tTemp.strExplanatoryNote = tTemp.strName;
		vtFieldInfo.push_back(tTemp);
	}
	tTemp.nType = 0;
	tTemp.strName = "robot_tool";
	tTemp.strExplanatoryNote = "机器人工具";
	vtFieldInfo.push_back(tTemp);

	tTemp.nType = 3;
	tTemp.nLenght = 20;
	tTemp.strDefault = "null";
	tTemp.strName = "device_no";
	tTemp.strExplanatoryNote = "设备编号";
	vtFieldInfo.push_back(tTemp);
	
	
	if (0 != CreatTable("unit_move_value", vtFieldInfo))
	{
		AfxMessageBox(GetErrorInfo());
		return FALSE;
	}

	return TRUE;
}

BOOL CDatabaseCtrl::CreatTable_RobotTool()
{
	std::vector<T_FIELD_INFO> vtFieldInfo;
	T_FIELD_INFO tTemp;
	tTemp.nType = 3;
	tTemp.nLenght = 50;
	tTemp.strDefault = "null";
	tTemp.strName = "unit_name";
	tTemp.strExplanatoryNote = "控制单元编号";
	vtFieldInfo.push_back(tTemp);

	tTemp.nType = 0;
	tTemp.nLenght = -1;
	tTemp.strName = "tool_no";
	tTemp.strExplanatoryNote = "工具编号";
	vtFieldInfo.push_back(tTemp);

	tTemp.nType = 2;
	for (size_t i = 0; i < 6; i++)
	{
		tTemp.strName.Format("coor_%d", i);
		tTemp.strExplanatoryNote = tTemp.strName;
		vtFieldInfo.push_back(tTemp);
	}

	if (0 != CreatTable("robot_tool_value", vtFieldInfo))
	{
		AfxMessageBox(GetErrorInfo());
		return FALSE;
	}

	return TRUE;
}

BOOL CDatabaseCtrl::CreatTable_DeviceState()
{
	std::vector<T_FIELD_INFO> vtFieldInfo;
	T_FIELD_INFO tTemp;
	tTemp.nType = 3;
	tTemp.nLenght = 50;
	tTemp.strDefault = "null";
	tTemp.strName = "unit_name";
	tTemp.strExplanatoryNote = "控制单元编号";
	vtFieldInfo.push_back(tTemp);


	tTemp.strName = "device_type";
	tTemp.strExplanatoryNote = "设备类型";
	vtFieldInfo.push_back(tTemp);

	tTemp.strName = "device_no";
	tTemp.strExplanatoryNote = "设备编号";
	vtFieldInfo.push_back(tTemp);

	tTemp.nLenght = -1;
	tTemp.nType = 1;
	tTemp.strName = "move_state";
	tTemp.strExplanatoryNote = "运动状态";
	vtFieldInfo.push_back(tTemp);

	tTemp.nType = 0;
	for (size_t i = 0; i < 9; i++)
	{
		tTemp.strName.Format("pulse_%d", i);
		tTemp.strExplanatoryNote = tTemp.strName;
		vtFieldInfo.push_back(tTemp);
	}
	tTemp.nType = 2;
	for (size_t i = 0; i < 9; i++)
	{
		tTemp.strName.Format("coor_%d", i);
		tTemp.strExplanatoryNote = tTemp.strName;
		vtFieldInfo.push_back(tTemp);
	}
	tTemp.nType = 0;
	tTemp.strName.Format("move_step");
	tTemp.strExplanatoryNote = "运动步数";
	vtFieldInfo.push_back(tTemp);

	if (0 != CreatTable("device_state", vtFieldInfo))
	{
		AfxMessageBox(GetErrorInfo());
		return FALSE;
	}

	return TRUE;
}
BOOL CDatabaseCtrl::ReadData_RobotTool(CString strUnitName, std::map< int, std::vector<double>> &mvdRobotTool)
{
	m_cDatabaseLock.Lock();
	std::vector< std::vector<CString>> vvstrData;
	CString strMark;
	strMark.Format("unit_name='%s'", strUnitName);
	if (0 != QueryData("robot_tool_value", strMark, vvstrData))
	{
		AfxMessageBox(GetErrorInfo());
		m_cDatabaseLock.UnLock();
		return FALSE;
	}
	for (size_t i = 0; i < vvstrData.size(); i++)
	{
		std::vector<double> vdRobotTool;
		for (size_t j = 0; j < 6; j++)
		{
			vdRobotTool.push_back(double());
			vdRobotTool[j] = atof(vvstrData[i][3 + j]);
		}
		mvdRobotTool[atoi(vvstrData[i][2])] = vdRobotTool;
	}
	m_cDatabaseLock.UnLock();
	return TRUE;
}

BOOL CDatabaseCtrl::InsertData_RobotTool(CString strUnitName, int nToolNo, double adTool[6])
{
	m_cDatabaseLock.Lock();
	std::map<CString, CString> mstrData;
	mstrData["unit_name"] = "'" + strUnitName + "'";
	mstrData["tool_no"].Format("%d", nToolNo);
	for (size_t j = 0; j < 6; j++)
	{
		mstrData[GetStr("coor_%d", j)].Format("%lf", adTool[j]);
	}

	if (0 != InsertData("robot_tool_value", mstrData))
	{
		AfxMessageBox(GetErrorInfo());
		m_cDatabaseLock.UnLock();
		return FALSE;
	}
	m_cDatabaseLock.UnLock();
	return TRUE;
}

BOOL CDatabaseCtrl::InsertData_DeviceState(T_DEVICE_STATE tRobotState)
{
	m_cDatabaseLock.Lock();
	std::map<CString, CString> mstrData;
	mstrData["unit_name"] = "'" + tRobotState.strUnitName + "'";
	mstrData["device_type"] = "'" + tRobotState.strDeviceType + "'";
	mstrData["device_no"] = "'" + tRobotState.strDeviceNo + "'";
	mstrData["move_state"].Format("%d", tRobotState.bMoveState);
	mstrData["move_step"].Format("%d", 0);

	for (size_t j = 0; j < 9; j++)
	{
		mstrData[GetStr("coor_%d", j)].Format("%lf", tRobotState.adCoor[j]);
		mstrData[GetStr("pulse_%d", j)].Format("%d", tRobotState.alPulse[j]);
	}

	if (0 != InsertData("device_state", mstrData))
	{
		AfxMessageBox(GetErrorInfo());
		m_cDatabaseLock.UnLock();
		return FALSE;
	}
	m_cDatabaseLock.UnLock();
	return TRUE;
}

BOOL CDatabaseCtrl::InsertData_MoveValue(T_MOVE_VALUE tMoveValue)
{
	m_cDatabaseLock.Lock();
	std::map<CString, CString> mstrData;
	mstrData["unit_name"] = "'" + tMoveValue.strUnitName + "'";
	mstrData["move_device"] = "'" + tMoveValue.strMoveDevice + "'";
	mstrData["move_type"] = "'" + tMoveValue.strMoveType + "'";
	mstrData["move_speed"].Format("%lf", tMoveValue.dSpeed);
	mstrData["move_step"].Format("%d", tMoveValue.nStepNo);
	mstrData["value_type"].Format("%d", tMoveValue.nValueType);
	mstrData["robot_tool"].Format("%d", tMoveValue.nRobotTool);
	mstrData["device_no"] = "'" + tMoveValue.strDeviceNo + "'";
	for (size_t j = 0; j < 9; j++)
	{
		mstrData[GetStr("coor_%d", j)].Format("%lf", tMoveValue.adCoor[j]);
		mstrData[GetStr("pulse_%d", j)].Format("%d", tMoveValue.alPulse[j]);
	}

	if (0 != InsertData("unit_move_value", mstrData))
	{
		AfxMessageBox(GetErrorInfo());
		m_cDatabaseLock.UnLock();
		return FALSE;
	}
	m_cDatabaseLock.UnLock();
	return TRUE;
}

BOOL CDatabaseCtrl::ReadData_DeviceState(T_DEVICE_STATE& tRobotState)
{
	m_cDatabaseLock.Lock();
	std::vector< std::vector<CString>> vvstrData;
	CString strMark;
	strMark.Format("unit_name='%s' and device_type='%s' and device_no='%s'", tRobotState.strUnitName, tRobotState.strDeviceType, tRobotState.strDeviceNo);
	if (0 != QueryData("device_state", strMark, vvstrData))
	{
		AfxMessageBox(GetErrorInfo());
		m_cDatabaseLock.UnLock();
		return FALSE;
	}
	if (vvstrData.empty())
	{
		return InsertData_DeviceState(tRobotState);
	}
	tRobotState.nID = atoi(vvstrData.back()[0]);
	tRobotState.bMoveState = atoi(vvstrData.back()[4]);

	for (size_t j = 0; j < 9; j++)
	{
		if (vvstrData.back()[5 + j] == "null" || vvstrData.back()[5 + j].IsEmpty())
		{
			continue;
		}
		tRobotState.alPulse[j] = atoi(vvstrData.back()[5 + j]);

		if (vvstrData.back()[14 + j] == "null" || vvstrData.back()[14 + j].IsEmpty())
		{
			continue;
		}
		tRobotState.adCoor[j] = atof(vvstrData.back()[14 + j]);
	}
	tRobotState.nMoveStep = atoi(vvstrData.back()[23]);
	m_cDatabaseLock.UnLock();
	return TRUE;
}

BOOL CDatabaseCtrl::ReadData_MoveValue(CString strUnitName, CString strMoveDevice, CString strDeviceNo, std::vector<T_MOVE_VALUE>& vtMoveValue)
{
	m_cDatabaseLock.Lock();
	vtMoveValue.clear();
	std::vector< std::vector<CString>> vvstrData;
	CString strMark;
	strMark.Format("unit_name='%s' and move_device='%s' and device_no='%s' order by move_step asc", strUnitName, strMoveDevice, strDeviceNo);
	if (0 != QueryData("unit_move_value", strMark, vvstrData))
	{
		AfxMessageBox(GetErrorInfo());
		m_cDatabaseLock.UnLock();
		return FALSE;
	}
	if (vvstrData.empty())
	{
		m_cDatabaseLock.UnLock();
		return FALSE;
	}
	for (size_t i = 0; i < vvstrData.size(); i++)
	{
		T_MOVE_VALUE tTemp;
		tTemp.nID = atoi(vvstrData[i][0]);
		tTemp.strMoveType = vvstrData[i][1];
		tTemp.strMoveDevice = vvstrData[i][2];
		tTemp.strUnitName = vvstrData[i][3];
		tTemp.dSpeed = atof(vvstrData[i][4]);
		tTemp.nStepNo = atoi(vvstrData[i][5]);
		tTemp.nValueType = atoi(vvstrData[i][6]);
		for (size_t j = 0; j < 9; j++)
		{
			tTemp.alPulse[j] = atoi(vvstrData[i][7 + j]);
			tTemp.adCoor[j] = atof(vvstrData[i][16 + j]);
		}
		tTemp.nRobotTool = atoi(vvstrData[i][25]);
		tTemp.strDeviceNo = vvstrData[i][26];
		vtMoveValue.push_back(tTemp);
	}
	m_cDatabaseLock.UnLock();
	return TRUE;
}

BOOL CDatabaseCtrl::UpdateData_DeviceState(T_DEVICE_STATE tRobotState)
{

	return TRUE;
}

BOOL CDatabaseCtrl::UpdateData_DeviceState_MoveState(CString strUnitName, CString strDeviceType, CString strDeviceNo, BOOL bMoveState)
{
	m_cDatabaseLock.Lock();
	std::map<CString, CString> mstrData;
	mstrData["move_state"].Format("%d", bMoveState);

	CString strMark;
	strMark.Format("unit_name='%s' and device_type='%s' and device_no='%s'", strUnitName, strDeviceType, strDeviceNo);

	if (0 != UpdateData("device_state", mstrData, strMark))
	{
		AfxMessageBox(GetErrorInfo());
		m_cDatabaseLock.UnLock();
		return FALSE;
	}
	m_cDatabaseLock.UnLock();
	return TRUE;
}

BOOL CDatabaseCtrl::UpdateData_DeviceState_MoveStep(CString strUnitName, CString strDeviceType, CString strDeviceNo, int nMoveStep)
{
	m_cDatabaseLock.Lock();
	std::map<CString, CString> mstrData;
	mstrData["move_step"].Format("%d", nMoveStep);

	CString strMark;
	strMark.Format("unit_name='%s' and device_type='%s' and device_no='%s'", strUnitName, strDeviceType, strDeviceNo);

	if (0 != UpdateData("device_state", mstrData, strMark))
	{
		AfxMessageBox(GetErrorInfo());
		m_cDatabaseLock.UnLock();
		return FALSE;
	}
	m_cDatabaseLock.UnLock();
	return TRUE;
}

BOOL CDatabaseCtrl::UpdateData_DeviceState_Coor(CString strUnitName, CString strDeviceType, CString strDeviceNo, double adCoor[9], long alPulse[9])
{
	m_cDatabaseLock.Lock();
	std::map<CString, CString> mstrData;

	CString str;
	for (size_t i = 0; i < 9; i++)
	{
		mstrData[GetStr("pulse_%d", i)].Format("%d", alPulse[i]);
		mstrData[GetStr("coor_%d", i)].Format("%lf", adCoor[i]);
	}


	CString strMark;
	strMark.Format("unit_name='%s' and device_type='%s' and device_no='%s'", strUnitName, strDeviceType, strDeviceNo);

	if (0 != UpdateData("device_state", mstrData, strMark))
	{
		AfxMessageBox(GetErrorInfo());
		m_cDatabaseLock.UnLock();
		return FALSE;
	}
	m_cDatabaseLock.UnLock();
	return TRUE;
}

BOOL CDatabaseCtrl::DeleteData_MoveValue(CString strUnitName, CString strMoveDevice, CString strDeviceNo, std::vector<T_MOVE_VALUE> vtMoveValue)
{
	m_cDatabaseLock.Lock();
	CString strMark;

	for (size_t i = 0; i < vtMoveValue.size(); i++)
	{
		strMark.Format("unit_name='%s' and move_device='%s' and device_no='%s' and move_step=%d", strUnitName, strMoveDevice, strDeviceNo, vtMoveValue[i].nStepNo);
		if (0 != DeleteData("unit_move_value", strMark))
		{
			AfxMessageBox(GetErrorInfo());
			m_cDatabaseLock.UnLock();
			return FALSE;
		}
	}
	m_cDatabaseLock.UnLock();
	return TRUE;
}

#endif // ENABLE_MY_SQL





