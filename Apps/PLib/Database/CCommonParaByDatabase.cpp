#include "stdafx.h"
#include "CCommonParaByDatabase.h"

#if ENABLE_MY_SQL

CCommonParaByDatabase g_CommonPara;

CCommonParaByDatabase::CCommonParaByDatabase(CString strTableName)
	:m_strTableName(strTableName)
{
	int nRtn = Connect("127.0.0.1", "root", "123456", "sys", 3306);
	if (nRtn != 0)
	{
		XiMessageBox(GetErrorInfo());
		return;
	}
	nRtn = CreateDatabases("xirobot");
	if (nRtn != 0)
	{
		XiMessageBox(GetErrorInfo());
		return;
	}
	nRtn = UseDatabases("xirobot");
	if (nRtn != 0)
	{
		XiMessageBox(GetErrorInfo());
	}
	nRtn = CreatCommonParaTable();
	if (nRtn != 0)
	{
		XiMessageBox(GetErrorInfo());
	}
}

CCommonParaByDatabase::~CCommonParaByDatabase()
{
}

BOOL CCommonParaByDatabase::CreatCommonParaTable()
{
	std::vector<T_FIELD_INFO> vtFieldInfo;
	T_FIELD_INFO tTemp;
	tTemp.nType = 3;
	tTemp.nLenght = 255;
	tTemp.strDefault = "null";
	tTemp.strName = "para_station";
	tTemp.strExplanatoryNote = "参数节点";
	vtFieldInfo.push_back(tTemp);

	tTemp.strName = "para_name";
	tTemp.strExplanatoryNote = "参数名";
	vtFieldInfo.push_back(tTemp);

	tTemp.strName = "para_type";
	tTemp.strExplanatoryNote = "参数类型";
	vtFieldInfo.push_back(tTemp);

	tTemp.strName = "para_describe";
	tTemp.strExplanatoryNote = "参数描述";
	vtFieldInfo.push_back(tTemp);

	for (size_t i = 0; i < COMMON_TABLE_DATA_SIZE; i++)
	{
		tTemp.nType = 3;
		tTemp.nLenght = 255;
		if (i == 0)
		{
			tTemp.nType = 6;
			tTemp.nLenght = 512;
		}
		tTemp.strName = GetStr("para_%d", i);
		tTemp.strExplanatoryNote = "参数";
		vtFieldInfo.push_back(tTemp);
	}
	for (size_t i = 0; i < COMMON_TABLE_DATA_SIZE; i++)
	{
		tTemp.nType = 3;
		tTemp.nLenght = 100;
		if (i == 0)
		{
			tTemp.nType = 6;
			tTemp.nLenght = 500;
		}
		tTemp.strName = GetStr("backups_para_%d", i);
		tTemp.strExplanatoryNote = "备份参数";
		vtFieldInfo.push_back(tTemp);
	}
	int nRtn;
	if (0 != (nRtn = CreatTable(m_strTableName, vtFieldInfo)))
	{
		AfxMessageBox(GetErrorInfo());
	}
	return nRtn;
}

BOOL CCommonParaByDatabase::ReadCommonPara(T_COMMON_TABLE &tCommonPara)
{
	CRITICAL_AUTO_LOCK(m_cDatabaseLock);
	std::vector< std::vector<CString>> vvstrData;
	CString strMark;
	strMark.Format("para_name='%s' and para_station='%s' and para_type='%s'", tCommonPara.strParaName, tCommonPara.strParaStation, tCommonPara.strParaType);
	if (0 != QueryData(m_strTableName, strMark, vvstrData))
	{
		AfxMessageBox(GetErrorInfo());
		return FALSE;
	}
	if (vvstrData.empty())
	{
		return FALSE;
	}
	tCommonPara.nID = atoi(vvstrData[0][0]);
	tCommonPara.strParaDescribe = vvstrData[0][4];
	int nDataSize = COMMON_TABLE_DATA_SIZE;
	if (7 + (2 * nDataSize) != vvstrData[0].size())
	{
		nDataSize = (vvstrData[0].size() - 7) / 2 > COMMON_TABLE_DATA_SIZE ? COMMON_TABLE_DATA_SIZE : (vvstrData[0].size() - 7) / 2;
	}
	for (size_t i = 0; i < nDataSize; i++)
	{
		tCommonPara.astrPara[i] = vvstrData[0][5 + i];
		tCommonPara.astrParaBackups[i] = vvstrData[0][5 + i + nDataSize];
	}
	return TRUE;
}

BOOL CCommonParaByDatabase::ReadCommonPara(CString strStation, std::vector<T_COMMON_TABLE>& vtCommonPara)
{
	CRITICAL_AUTO_LOCK(m_cDatabaseLock);
	std::vector< std::vector<CString>> vvstrData;
	CString strMark;
	strMark.Format("para_station='%s'", strStation);
	if (0 != QueryData(m_strTableName, strMark, vvstrData))
	{
		AfxMessageBox(GetErrorInfo());
		return FALSE;
	}
	if (vvstrData.empty())
	{
		return FALSE;
	}
	vtCommonPara.clear();
	for (size_t i = 0; i < vvstrData.size(); i++)
	{
		if (vvstrData[i].size() < 5)
		{
			return FALSE;
		}
		T_COMMON_TABLE tCommonPara;
		tCommonPara.nID = atoi(vvstrData[i][0]);
		tCommonPara.strParaStation = vvstrData[i][1];
		tCommonPara.strParaName = vvstrData[i][2];
		tCommonPara.strParaType = vvstrData[i][3];
		tCommonPara.strParaDescribe = vvstrData[i][4];
		int nDataSize = COMMON_TABLE_DATA_SIZE;
		if (7 + (2 * nDataSize) != vvstrData[i].size())
		{
			nDataSize = (vvstrData[i].size() - 7) / 2 > COMMON_TABLE_DATA_SIZE ? COMMON_TABLE_DATA_SIZE : (vvstrData[i].size() - 7) / 2;
		}
		for (size_t i = 0; i < nDataSize; i++)
		{
			tCommonPara.astrPara[i] = vvstrData[i][5 + i];
			tCommonPara.astrParaBackups[i] = vvstrData[i][5 + i + nDataSize];
		}
		vtCommonPara.push_back(tCommonPara);
	}
	return TRUE;
}

BOOL CCommonParaByDatabase::UpdateCommonPara(T_COMMON_TABLE tCommonPara)
{
	CRITICAL_AUTO_LOCK(m_cDatabaseLock);
	std::map<CString, CString> mstrData;
	for (size_t i = 0; i < COMMON_TABLE_DATA_SIZE; i++)
	{
		mstrData[GetStr("para_%d", i)] = "'" + tCommonPara.astrPara[i] + "'";
		mstrData[GetStr("backups_para_%d", i)] = "'" + tCommonPara.astrParaBackups[i] + "'";
	}

	CString strMark;
	strMark.Format("para_name='%s' and para_station='%s' and para_type='%s'", tCommonPara.strParaName, tCommonPara.strParaStation, tCommonPara.strParaType);

	if (0 != UpdateData(m_strTableName, mstrData, strMark))
	{
		AfxMessageBox(GetErrorInfo());
		return FALSE;
	}
	return TRUE;
}

BOOL CCommonParaByDatabase::InsertCommonPara(T_COMMON_TABLE tCommonPara)
{
	CRITICAL_AUTO_LOCK(m_cDatabaseLock);
	std::map<CString, CString> mstrData;
	mstrData["para_station"] = "'" + tCommonPara.strParaStation + "'";
	mstrData["para_name"] = "'" + tCommonPara.strParaName + "'";
	mstrData["para_type"] = "'" + tCommonPara.strParaType + "'";
	mstrData["para_describe"] = "'" + tCommonPara.strParaDescribe + "'";
	for (size_t i = 0; i < COMMON_TABLE_DATA_SIZE; i++)
	{
		mstrData[GetStr("para_%d", i)] = "'" + tCommonPara.astrPara[i] + "'";
		mstrData[GetStr("backups_para_%d", i)] = "'" + tCommonPara.astrParaBackups[i] + "'";
	}
	if (0 != InsertData(m_strTableName, mstrData))
	{
		AfxMessageBox(GetErrorInfo());
		return FALSE;
	}
	return TRUE;
}

BOOL CCommonParaByDatabase::UpdateStationDescribe(CString strStation, CString strDescribe)
{
	CRITICAL_AUTO_LOCK(m_cDatabaseLock);
	std::map<CString, CString> mstrData;
	mstrData["para_describe"] = "'" + strDescribe + "'";

	CString strMark;
	strMark.Format("para_station='%s'", strStation);

	if (0 != UpdateData(m_strTableName, mstrData, strMark))
	{
		AfxMessageBox(GetErrorInfo());
		return FALSE;
	}
	return TRUE;
}

BOOL CCommonParaByDatabase::UpdateStationDescribe(CString strStation, CString strName, CString strDescribe)
{
	CRITICAL_AUTO_LOCK(m_cDatabaseLock);
	std::map<CString, CString> mstrData;
	mstrData["para_describe"] = "'" + strDescribe + "'";

	CString strMark;
	strMark.Format("para_station='%s' and para_name='%s'", strStation, strName);

	if (0 != UpdateData(m_strTableName, mstrData, strMark))
	{
		AfxMessageBox(GetErrorInfo());
		return FALSE;
	}
	return TRUE;
}

CString CCommonParaByDatabase::GetTableName()
{
	return m_strTableName;
}

BOOL CCommonParaByDatabase::ReadCommonPara(std::vector<T_COMMON_TABLE>& vtCommonPara)
{
	CRITICAL_AUTO_LOCK(m_cDatabaseLock);
	std::vector< std::vector<CString>> vvstrData;
	if (0 != QueryData(m_strTableName, vvstrData))
	{
		AfxMessageBox(GetErrorInfo());
		return FALSE;
	}
	if (vvstrData.empty())
	{
		return FALSE;
	}
	vtCommonPara.clear();
	for (size_t i = 0; i < vvstrData.size(); i++)
	{
		if (vvstrData[i].size() < 5)
		{
			return FALSE;
		}
		T_COMMON_TABLE tCommonPara;
		tCommonPara.nID = atoi(vvstrData[i][0]);
		tCommonPara.strParaStation = vvstrData[i][1];
		tCommonPara.strParaName = vvstrData[i][2];
		tCommonPara.strParaType = vvstrData[i][3];
		tCommonPara.strParaDescribe = vvstrData[i][4];
		int nDataSize = COMMON_TABLE_DATA_SIZE;
		if (7 + (2 * nDataSize) != vvstrData[i].size())
		{
			nDataSize = (vvstrData[i].size() - 7) / 2 > COMMON_TABLE_DATA_SIZE ? COMMON_TABLE_DATA_SIZE : (vvstrData[i].size() - 7) / 2;
		}
		for (size_t j = 0; j < nDataSize; j++)
		{
			tCommonPara.astrPara[j] = vvstrData[i][5 + j];
			tCommonPara.astrParaBackups[j] = vvstrData[i][5 + j + nDataSize];
		}
		vtCommonPara.push_back(tCommonPara);
	}
	return TRUE;
}

BOOL CCommonParaByDatabase::SetCommonPara(T_COMMON_TABLE tCommonPara)
{
	CRITICAL_AUTO_LOCK(m_cDatabaseLock);
	std::vector< std::vector<CString>> vvstrData;
	CString strMark;
	strMark.Format("para_name='%s' and para_station='%s' and para_type='%s'", tCommonPara.strParaName, tCommonPara.strParaStation, tCommonPara.strParaType);
	if (0 != QueryData(m_strTableName, strMark, vvstrData))
	{
		AfxMessageBox(GetErrorInfo());
		return FALSE;
	}
	if (vvstrData.empty())
	{
		return InsertCommonPara(tCommonPara);
	}
	int nDataSize = COMMON_TABLE_DATA_SIZE;
	if (7 + (2 * nDataSize) != vvstrData[0].size())
	{
		nDataSize = (vvstrData[0].size() - 7) / 2 > COMMON_TABLE_DATA_SIZE ? COMMON_TABLE_DATA_SIZE : (vvstrData[0].size() - 7) / 2;
	}
	for (size_t i = 0; i < nDataSize; i++)
	{
		tCommonPara.astrParaBackups[i] = vvstrData[0][5 + i];
	}
	return UpdateCommonPara(tCommonPara);
}

T_COMMON_TABLE GetPara(CString strStation, CString strName, CString strType, std::vector<CString> strPara, CString strDescribe)
{
	T_COMMON_TABLE temp;
	temp.strParaStation = strStation;
	temp.strParaName = strName;
	temp.strParaType = strType;
	temp.strParaDescribe = strDescribe;
	for (size_t i = 0; i < COMMON_TABLE_DATA_SIZE; i++)
	{
		if (i >= strPara.size())
		{
			break;
		}
		temp.astrPara[i] = strPara[i];
	}
	return temp;
}

std::vector<CString> ToCString(int val)
{
	std::vector<CString> temp;
	temp.push_back(GetStr("%d", val));
	return temp;
}

std::vector<CString> ToCString(long val)
{
	std::vector<CString> temp;
	temp.push_back(GetStr("%d", val));
	return temp;
}

std::vector<CString> ToCString(long long val)
{
	std::vector<CString> temp;
	temp.push_back(GetStr("%d", val));
	return temp;
}

std::vector<CString> ToCString(bool val)
{
	std::vector<CString> temp;
	temp.push_back(GetStr("%d", val));
	return temp;
}

std::vector<CString> ToCString(float val)
{
	std::vector<CString> temp;
	temp.push_back(GetStr("%lf", val));
	return temp;
}

std::vector<CString> ToCString(double val)
{
	std::vector<CString> temp;
	temp.push_back(GetStr("%lf", val));
	return temp;
}

std::vector<CString> ToCString(char* val)
{
	std::vector<CString> temp;
	temp.push_back(GetStr("%s", val));
	return temp;
}

std::vector<CString> ToCString(std::string val)
{
	std::vector<CString> temp;
	temp.push_back(GetStr("%s", val.c_str()));
	return temp;
}

std::vector<CString> ToCString(CString val)
{
	std::vector<CString> temp;
	temp.push_back(val);
	return temp;
}

std::vector<CString> ToCString(T_ROBOT_COORS val)
{
	std::vector<CString> temp;
	temp.push_back(GetStr("%lf", val.dX));
	temp.push_back(GetStr("%lf", val.dY));
	temp.push_back(GetStr("%lf", val.dZ));
	temp.push_back(GetStr("%lf", val.dRX));
	temp.push_back(GetStr("%lf", val.dRY));
	temp.push_back(GetStr("%lf", val.dRZ));
	temp.push_back(GetStr("%lf", val.dBX));
	temp.push_back(GetStr("%lf", val.dBY));
	temp.push_back(GetStr("%lf", val.dBZ));
	return temp;
}

std::vector<CString> ToCString(T_ANGLE_PULSE val)
{
	std::vector<CString> temp;
	temp.push_back(GetStr("%d", val.nSPulse));
	temp.push_back(GetStr("%d", val.nLPulse));
	temp.push_back(GetStr("%d", val.nUPulse));
	temp.push_back(GetStr("%d", val.nRPulse));
	temp.push_back(GetStr("%d", val.nBPulse));
	temp.push_back(GetStr("%d", val.nTPulse));
	temp.push_back(GetStr("%d", val.lBXPulse));
	temp.push_back(GetStr("%d", val.lBYPulse));
	temp.push_back(GetStr("%d", val.lBZPulse));
	return temp;
}

std::vector<CString> SetPara(T_COMMON_TABLE tCommonPara)
{
	std::vector<CString> temp(tCommonPara.astrPara, tCommonPara.astrPara + COMMON_TABLE_DATA_SIZE);
	return temp;
}

bool CStringTo(std::vector<CString> str, int& val)
{
	if (str.empty())
	{
		return false;
	}
	val = atoi(str[0]);
	return true;
}

bool CStringTo(std::vector<CString> str, long& val)
{
	if (str.empty())
	{
		return false;
	}
	val = atol(str[0]);
	return true;
}

bool CStringTo(std::vector<CString> str, long long& val)
{
	if (str.empty())
	{
		return false;
	}
	val = atoll(str[0]);
	return true;
}

bool CStringTo(std::vector<CString> str, bool& val)
{
	if (str.empty())
	{
		return false;
	}
	int n = atoi(str[0]);
	if (0 == n)
		val = false;
	else
		val = true;
	return true;
}

bool CStringTo(std::vector<CString> str, float& val)
{
	if (str.empty())
	{
		return false;
	}
	val = atof(str[0]);
	return true;
}

bool CStringTo(std::vector<CString> str, double& val)
{
	if (str.empty())
	{
		return false;
	}
	val = atof(str[0]);
	return true;
}

bool CStringTo(std::vector<CString> str, char* val)
{
	if (str.empty())
	{
		return false;
	}
	strcpy(val, str[0].GetBuffer());
	return true;
}

bool CStringTo(std::vector<CString> str, std::string& val)
{
	if (str.empty())
	{
		return false;
	}
	val = str[0].GetString();
	return true;
}

bool CStringTo(std::vector<CString> str, CString& val)
{
	if (str.empty())
	{
		return false;
	}
	val = str[0];
	return true;
}

bool CStringTo(std::vector<CString> str, T_ROBOT_COORS& val)
{
	if (str.size() < 9)
	{
		return false;
	}
	val.dX =  atof(str[0]);
	val.dY =  atof(str[1]);
	val.dZ =  atof(str[2]);
	val.dRX = atof(str[3]);
	val.dRY = atof(str[4]);
	val.dRZ = atof(str[5]);
	val.dBX = atof(str[6]);
	val.dBY = atof(str[7]);
	val.dBZ = atof(str[8]);
	return true;
}

bool CStringTo(std::vector<CString> str, T_ANGLE_PULSE& val)
{
	if (str.size() < 9)
	{
		return false;
	}
	val.nSPulse =  atol(str[0]);
	val.nLPulse =  atol(str[1]);
	val.nUPulse =  atol(str[2]);
	val.nRPulse =  atol(str[3]);
	val.nBPulse =  atol(str[4]);
	val.nTPulse =  atol(str[5]);
	val.lBXPulse = atol(str[6]);
	val.lBYPulse = atol(str[7]);
	val.lBZPulse = atol(str[8]);
	return true;
}

std::vector<CString> ToCString(T_ROBOT_MOVE_SPEED val)
{
	std::vector<CString> temp;
	temp.push_back(GetStr("%lf", val.dSpeed));
	temp.push_back(GetStr("%lf", val.dACC));
	temp.push_back(GetStr("%lf", val.dDEC));
	return temp;
}

std::vector<CString> ToCString(CvPoint3D64f val)
{
	std::vector<CString> temp;
	temp.push_back(GetStr("%lf", val.x));
	temp.push_back(GetStr("%lf", val.y));
	temp.push_back(GetStr("%lf", val.z));
	return temp;
}

std::vector<CString> ToCString(T_ENTITY_RECT val)
{
	std::vector<CString> temp;
	temp.push_back(GetStr("%lf", val.tMin.x));
	temp.push_back(GetStr("%lf", val.tMin.y));
	temp.push_back(GetStr("%lf", val.tMin.z));
	temp.push_back(GetStr("%lf", val.tMax.x));
	temp.push_back(GetStr("%lf", val.tMax.y));
	temp.push_back(GetStr("%lf", val.tMax.z));
	return temp;
}

bool CStringTo(std::vector<CString> str, T_ROBOT_MOVE_SPEED& val)
{
	if (str.size() < 3)
	{
		return false;
	}
	val.dSpeed = atof(str[0]);
	val.dACC = atof(str[1]);
	val.dDEC = atof(str[2]);
	return true;
}

bool CStringTo(std::vector<CString> str, CvPoint3D64f& val)
{
	if (str.size() < 3)
	{
		return false;
	}
	val.x = atof(str[0]);
	val.y = atof(str[1]);
	val.z = atof(str[2]);
	return true;
}

bool CStringTo(std::vector<CString> str, T_ENTITY_RECT& val)
{
	if (str.size() < 6)
	{
		return false;
	}
	val.tMin.x = atof(str[0]);
	val.tMin.y = atof(str[1]);
	val.tMin.z = atof(str[2]);
	val.tMax.x = atof(str[3]);
	val.tMax.y = atof(str[4]);
	val.tMax.z = atof(str[5]);
	return true;
}













#endif