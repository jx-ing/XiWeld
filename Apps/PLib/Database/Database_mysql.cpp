#include "stdafx.h"
#include "Database_mysql.h"

#if ENABLE_MY_SQL

Database_mysql::~Database_mysql()
{
	Disconnect();
}

CString Database_mysql::GetErrorInfo()
{
	if (!m_bConnect)
	{
		return "MySql No Connect!";
	}
	CString strErrorInfo;
	strErrorInfo.Format("ErrorNo:%d\r\n%s", mysql_errno(&m_sqlCon), mysql_error(&m_sqlCon));
	return strErrorInfo;
}

int Database_mysql::QueryCmd(CString strCmd)
{
	CHECK_DATABASE_CONNECT;
	return mysql_query(&m_sqlCon, strCmd.GetBuffer());
}

MYSQL_RES* Database_mysql::GetQueryResult()
{
	if (!m_bConnect)
	{
		return NULL;
	}
	return mysql_store_result(&m_sqlCon);
}

void Database_mysql::ReleaseQueryResult(MYSQL_RES* mqlResult)
{
	if (mqlResult == NULL)
	{
		return;
	}
	mysql_free_result(mqlResult);
}

int Database_mysql::CreateDatabases(CString strDBName)
{
	CHECK_DATABASE_CONNECT;
	CString strQueryData;
	strQueryData.Format("create database  if not exists %s COLLATE armscii8_bin;", strDBName);
	return mysql_query(&m_sqlCon, strQueryData.GetBuffer());
}

int Database_mysql::Connect(CString strIP, CString strUser, CString strKey, CString strDBName, int nPort)
{
	CHECK_DATABASE_CONNECT;
	mysql_init(&m_sqlCon);
	if (!mysql_real_connect(&m_sqlCon, strIP, strUser,
		strKey, strDBName, nPort, NULL, 0))
	{
		return false;
	}
	return mysql_query(&m_sqlCon, "set names 'gb2312'");
}

int Database_mysql::Disconnect()
{
	CHECK_DATABASE_CONNECT;
	mysql_close(&m_sqlCon);
	return 0;
}

int Database_mysql::ShowDatabases(std::vector<CString> vstrDBName)
{
	CHECK_DATABASE_CONNECT;
	CString strQueryData;
	strQueryData.Format("show databases;");
	int nRtn = mysql_query(&m_sqlCon, strQueryData.GetBuffer());
	if (nRtn != 0)
	{
		return nRtn;
	}
	MYSQL_RES* result;
	result = mysql_store_result(&m_sqlCon);
	if (result == NULL)
	{
		return -1;
	}

	MYSQL_ROW row;
	while (row = mysql_fetch_row(result))
	{
		vstrDBName.push_back(row[0]);
	}

	mysql_free_result(result);
	return nRtn;
}

int Database_mysql::UseDatabases(CString strDBName)
{
	CHECK_DATABASE_CONNECT;
	CString strQueryData;
	strQueryData.Format("use %s;", strDBName);
	return mysql_query(&m_sqlCon, strQueryData.GetBuffer());
}

int Database_mysql::CreatTable(CString strTableName, std::vector<T_FIELD_INFO> vtFieldInfo)
{
	CHECK_DATABASE_CONNECT;
	CString strTableInfo;
	strTableInfo.Format("create table if not exists %s(id int unsigned auto_increment primary key comment '主键id'", strTableName);
	for (size_t i = 0; i < vtFieldInfo.size(); i++)
	{
		strTableInfo = strTableInfo + "," + GetField(vtFieldInfo[i]);
		if (vtFieldInfo[i].foreignKeyTableName != "")
		{
			strTableInfo = strTableInfo + ",FOREIGN KEY("+ vtFieldInfo[i].strName+") references " + vtFieldInfo[i].foreignKeyTableName + "(id)";
		}
	}
	strTableInfo += ",update_date DATETIME NULL DEFAULT now() comment '更新时间',creation_date DATETIME NULL DEFAULT now() comment '创建时间')engine = innodb; "; 
//	XiMessageBox(strTableInfo);
	return mysql_query(&m_sqlCon, strTableInfo.GetBuffer());
}

int Database_mysql::DeleteTable(CString strTableName)
{
	CHECK_DATABASE_CONNECT;
	CString strQueryData;
	strQueryData.Format("drop table if exists %s;", strTableName);
	return mysql_query(&m_sqlCon, strQueryData.GetBuffer());
}

int Database_mysql::ClearTable(CString strTableName)
{
	CHECK_DATABASE_CONNECT;
	CString strQueryData;
	strQueryData.Format("truncate table %s;", strTableName);
	return mysql_query(&m_sqlCon, strQueryData.GetBuffer());
}

int Database_mysql::QueryTableInfo(CString strTableName, std::vector<T_QUERY_FIELD_INFO> &vtFieldInfo)
{
	CHECK_DATABASE_CONNECT;
	CString strQueryData;
	strQueryData.Format("desc %s;", strTableName);
	MYSQL_CHECK_RTN(mysql_query(&m_sqlCon, strQueryData.GetBuffer()));

	MYSQL_RES* result;
	result = mysql_store_result(&m_sqlCon);
	if (result == NULL)
	{
		return -1;
	}

	MYSQL_ROW row;
	while (row = mysql_fetch_row(result))
	{
		T_QUERY_FIELD_INFO tFieldInfo;
		tFieldInfo.strName = row[0];
		tFieldInfo.strType = row[1];
		tFieldInfo.strNull = row[2];
		tFieldInfo.strKey = row[3];
		tFieldInfo.strDefault = row[4];
		tFieldInfo.strExtra = row[5];
		vtFieldInfo.push_back(tFieldInfo);
	}

	mysql_free_result(result);
	return 0;
}

int Database_mysql::RenameTable(CString strTableName, CString strNewTableName)
{
	CHECK_DATABASE_CONNECT;
	CString strQueryData;
	strQueryData.Format("alter table %s rename to %s;", strTableName, strNewTableName);
	return mysql_query(&m_sqlCon, strQueryData.GetBuffer());
}

int Database_mysql::AddField(CString strTableName, T_FIELD_INFO tFieldInfo, CString strPos)
{
	CHECK_DATABASE_CONNECT;
	CString strQueryData;
	strQueryData.Format("alter table %s add %s %s;", strTableName, GetField(tFieldInfo), strPos);
	return mysql_query(&m_sqlCon, strQueryData.GetBuffer());
}

int Database_mysql::DeleteField(CString strTableName, CString strFieldName)
{
	CHECK_DATABASE_CONNECT;
	CString strQueryData;
	strQueryData.Format("alter table %s drop %s;", strTableName, strFieldName);
	return mysql_query(&m_sqlCon, strQueryData.GetBuffer());
}

int Database_mysql::ChangeField(CString strTableName, CString strFieldName, T_FIELD_INFO tNewFieldInfo)
{
	CHECK_DATABASE_CONNECT;
	CString strQueryData;
	strQueryData.Format("alter table %s change %s %s;", strTableName, strFieldName, GetField(tNewFieldInfo));
	return mysql_query(&m_sqlCon, strQueryData.GetBuffer());
}

int Database_mysql::InsertData(CString strTableName, std::map<CString, CString> mstrData)
{
	CHECK_DATABASE_CONNECT;
	CString strQueryData;
	CString strFieldName;
	CString strData;
	for (auto iter = mstrData.begin(); iter != mstrData.end(); iter++)
	{
		strFieldName = strFieldName + iter->first + ",";
		strData = strData + iter->second + ",";
	}
	strFieldName = strFieldName.Left(strFieldName.GetLength() - 1);
	strData = strData.Left(strData.GetLength() - 1);
	strQueryData.Format("insert into %s (%s) values(%s);", strTableName, strFieldName, strData);
	return mysql_query(&m_sqlCon, strQueryData.GetBuffer());
}

int Database_mysql::InsertData(CString strTableName, std::vector<CString> vstrData)
{
	CHECK_DATABASE_CONNECT;
	CString strQueryData;
	CString strData;
	for (size_t i = 0; i < vstrData.size(); i++)
	{
		strData = strData + vstrData[i] + ",";
	}
	strData = strData.Left(strData.GetLength() - 1);
	strQueryData.Format("insert into %s values(null, %s, now(), now());", strTableName, strData);
	return mysql_query(&m_sqlCon, strQueryData.GetBuffer());
}

int Database_mysql::InsertData(CString strTableName, std::vector< std::vector<CString>> vvstrData)
{
	CHECK_DATABASE_CONNECT;
	CString strQueryData;
	CString strTotalData;
	for (size_t i = 0; i < vvstrData.size(); i++)
	{
		CString strData;
		strData = "(null, " + strData;
		for (size_t j = 0; j < vvstrData[i].size(); j++)
		{
			strData = strData + vvstrData[i][j] + ",";
		}
		strTotalData = strTotalData + strData + "now(), now()),";
	}
	strTotalData = strTotalData.Left(strTotalData.GetLength() - 1);
	strQueryData.Format("insert into %s values%s;", strTableName, strTotalData);
	return mysql_query(&m_sqlCon, strQueryData.GetBuffer());
}

int Database_mysql::DeleteData(CString strTableName, CString strMark)
{
	CHECK_DATABASE_CONNECT;
	CString strQueryData;
	strQueryData.Format("delete from %s where %s;", strTableName, strMark);
	return mysql_query(&m_sqlCon, strQueryData.GetBuffer());
}

int Database_mysql::CleanData(CString strTableName)
{
	CHECK_DATABASE_CONNECT;
	CString strQueryData;
	strQueryData.Format("truncate table %s;", strTableName);
	return mysql_query(&m_sqlCon, strQueryData.GetBuffer());
}

int Database_mysql::UpdateData(CString strTableName, std::map<CString, CString> mstrNewData, CString strMark)
{
	CHECK_DATABASE_CONNECT;
	CString strQueryData;
	CString strData;
	for (auto iter = mstrNewData.begin(); iter != mstrNewData.end(); iter++)
	{
		strData = strData + iter->first + "=" + iter->second + ",";
	}
	strData = strData.Left(strData.GetLength() - 1);
	strQueryData.Format("update %s set %s,update_date=now() where %s;", strTableName, strData, strMark);
	return mysql_query(&m_sqlCon, strQueryData.GetBuffer());
}

//零件的数量更新专用
int Database_mysql::UpdateData1(CString strTableName, std::map<CString, CString> mstrNewData, CString strMark)
{
	CHECK_DATABASE_CONNECT;
	CString strQueryData;
	CString strData;
	for (auto iter = mstrNewData.begin(); iter != mstrNewData.end(); iter++)
	{
		strData = strData + iter->first + "=" + iter->second + ",";
	}
	strData = strData.Left(strData.GetLength() - 1);
	strQueryData.Format("update %s set %s where %s;", strTableName, strData, strMark);
	return mysql_query(&m_sqlCon, strQueryData.GetBuffer());
}

int Database_mysql::QueryData(CString strTableName, std::vector<std::vector<CString>>& vvstrData)
{
	CHECK_DATABASE_CONNECT;
	vvstrData.clear();
	CString strQueryData;
	strQueryData.Format("select * from %s ORDER BY id asc;", strTableName);
	MYSQL_CHECK_RTN(mysql_query(&m_sqlCon, strQueryData.GetBuffer()));

	MYSQL_RES* result;
	result = mysql_store_result(&m_sqlCon);
	if (result == NULL)
	{
		return -1;
	}

	MYSQL_ROW row;
	while (row = mysql_fetch_row(result))
	{
		std::vector<CString> vstrData;
		for (unsigned int i = 0; i < result->field_count; i++)
		{
			vstrData.push_back(row[i]);
		}
		vvstrData.push_back(vstrData);
	}

	mysql_free_result(result);
	return 0;
}

int Database_mysql::QueryData(CString strTableName, CString strMark, std::vector<std::vector<CString>>& vvstrData)
{
	CHECK_DATABASE_CONNECT;
	vvstrData.clear();
	CString strQueryData;
	strQueryData.Format("select * from %s where %s;", strTableName, strMark);
	MYSQL_CHECK_RTN(mysql_query(&m_sqlCon, strQueryData.GetBuffer()));

	MYSQL_RES* result;
	result = mysql_store_result(&m_sqlCon);
	if (result == NULL)
	{
		return -1;
	}

	MYSQL_ROW row;
	while (row = mysql_fetch_row(result))
	{
		std::vector<CString> vstrData;
		for (unsigned int i = 0; i < result->field_count; i++)
		{
			vstrData.push_back(row[i]);
		}
		vvstrData.push_back(vstrData);
	}

	mysql_free_result(result);
	return 0;
}

int Database_mysql::QueryData(int formulaID, std::vector<std::vector<CString>>& vvstrData)
{
	CHECK_DATABASE_CONNECT;
	vvstrData.clear();
	CString strQueryData;
	strQueryData.Format("SELECT p.* FROM formula AS f JOIN formula_part AS fp ON f.id = fp.FormulaID JOIN part AS p ON fp.PartID = p.id WHERE f.id=%d;", formulaID);
	MYSQL_CHECK_RTN(mysql_query(&m_sqlCon, strQueryData.GetBuffer()));

	MYSQL_RES* result;
	result = mysql_store_result(&m_sqlCon);
	if (result == NULL)
	{
		return -1;
	}

	MYSQL_ROW row;
	while (row = mysql_fetch_row(result))
	{
		std::vector<CString> vstrData;
		for (unsigned int i = 0; i < result->field_count; i++)
		{
			vstrData.push_back(row[i]);
		}
		vvstrData.push_back(vstrData);
	}

	mysql_free_result(result);
	return 0;
}

int Database_mysql::QueryDataNum(CString strTableName, unsigned long long& nNum)
{
	CHECK_DATABASE_CONNECT;
	CString strQueryData;
	strQueryData.Format("SELECT COUNT(*) FROM %s;", strTableName);
	MYSQL_CHECK_RTN(mysql_query(&m_sqlCon, strQueryData.GetBuffer()));

	MYSQL_RES* result;
	result = mysql_store_result(&m_sqlCon);
	if (result == NULL)
	{
		return -1;
	}

	MYSQL_ROW row; 
	std::vector<std::vector<CString>> vvstrData;
	while (row = mysql_fetch_row(result))
	{
		std::vector<CString> vstrData;
		for (unsigned int i = 0; i < result->field_count; i++)
		{
			vstrData.push_back(row[i]);
		}
		vvstrData.push_back(vstrData);
	}
	nNum = atoi(vvstrData.back().back());

	mysql_free_result(result);
	return 0;
}

CString Database_mysql::GetField(T_FIELD_INFO tFieldInfo)
{
	if (tFieldInfo.nType >= g_nMaxTypeNum)
	{
		return CString();
	}
	CString strType = g_aacValType[tFieldInfo.nType];
	CString strLenght;
	if (tFieldInfo.nLenght > 0 && tFieldInfo.nType != 1)
	{
		strLenght.Format("(%d)", tFieldInfo.nLenght);
	}
	CString strUnsigned;
	if (tFieldInfo.bUnsigned)
	{
		strUnsigned = "unsigned";
	}
	CString strDefault = tFieldInfo.strDefault;
	if ((tFieldInfo.nType == 3 || tFieldInfo.nType == 4) && !strDefault.IsEmpty())
	{
		strDefault = "default \'" + strDefault + "\'";
	}
	else if (!strDefault.IsEmpty())
	{
		strDefault = "default " + strDefault;
	}
	else
	{
		strDefault = "null";
	}
	if (tFieldInfo.nType == 1)
	{
		strDefault = "null";
	}
	CString strExplanatoryNote = tFieldInfo.strExplanatoryNote;
	strExplanatoryNote = "COMMENT \'" + strExplanatoryNote + "\'";
	CString strField;
	strField.Format("%s %s%s %s %s %s", tFieldInfo.strName, strType, strLenght, strUnsigned, strDefault, strExplanatoryNote);
	if (tFieldInfo.nType == 3 || tFieldInfo.nType == 6)
	{
		strField = strField + " COLLATE 'utf8_bin'";
	}
	return strField;
}

#endif // ENABLE_MY_SQL



