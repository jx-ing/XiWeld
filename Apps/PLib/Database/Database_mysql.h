#pragma once
#include ".\Apps\PLib\BasicFunc\ChoiceResources.h"
#if ENABLE_MY_SQL

#include ".\Apps\PLib\BasicFunc\FileProcess.h"
#include <map>
#include "mysql.h"

/*
* MYSQL_CHECK_RTN
* 检查返回值
*
*/
#define MYSQL_CHECK_RTN(nRtn) {int n = nRtn;if(n != 0)return n;}

/*
* CHECK_DATABASE_CONNECT
* 检查数据库是否连接
*
*/
#define  CHECK_DATABASE_CONNECT if(!m_bConnect) return -1;

/*
* aacValType
* 数据库中变量类型
* 
* 注：BOOL 类型用 TINYINT(1) 替代
*/
const char g_aacValType[][128] =
{
	{"INT"}, {"TINYINT(1)"}, {"DOUBLE"}, {"CHAR"}, {"DATETIME"}, {"DATE"}, {"TEXT"}
};
const int g_nMaxTypeNum = 7;

typedef struct 
{
	CString strName;//字段名
	int nType;//类型
	int nLenght = -1;//长度限制
	bool bUnsigned = false;//是否无符号
	CString strDefault = "";//默认值
	CString strExplanatoryNote = "";//说明
	CString foreignKeyTableName = "";//外键信息,需要填入参照表表名
}T_FIELD_INFO;

typedef struct
{
	CString strName;//字段名
	CString strType;//类型
	CString strNull;//是否可为 null
	CString strKey;//索引
	CString strDefault;//默认值
	CString strExtra;//校对规则
}T_QUERY_FIELD_INFO;

class Database_mysql
{
public:
	~Database_mysql();

	/************************************************************************/
	/************************************************************************/
	/*************************		   通用操作 	  ***********************/
	/************************************************************************/
	/************************************************************************/

	/*
	* GetErrorInfo
	* 获取错误信息
	* param 	
	* return	错误信息
	*/
	CString GetErrorInfo();

	/*
	* QueryCmd
	* 查询消息
	* param
	* return	0:成功	其他:错误代码
	*/
	int QueryCmd(CString strCmd);

	/*
	* GetQueryResult
	* 获取查询结果
	* param
	* return	结果数据
	*/
	MYSQL_RES* GetQueryResult();

	/*
	* ReleaseQueryResult
	* 释放查询结果
	* param
	* return	
	*/
	void ReleaseQueryResult(MYSQL_RES* mqlResult);

	/************************************************************************/
	/************************************************************************/
	/*************************		 数据库操作 	  ***********************/
	/************************************************************************/
	/************************************************************************/
	/*
	* CreateDatabases
	* 创建数据库
	* param 	strDBName:数据库名
	* return	0:成功	其他:错误代码
	*/
	int CreateDatabases(CString strDBName);

	/*
	* Connect
	* 连接数据库
	* param 	strIP:ip地址	strUser:用户名	strKey:密码	strDBName:数据库名	nPort:端口号
	* return	0:成功	其他:错误代码
	*/
	int Connect(CString strIP, CString strUser, CString strKey, CString strDBName, int nPort);

	/*
	* Disconnect
	* 断开数据库连接
	* param
	* return	0:成功	其他:错误代码
	*/
	int Disconnect();

	/*
	* ShowDatabases
	* 显示所有数据库
	* param		vstrDBName:数据库名
	* return	0:成功	其他:错误代码
	*/
	int ShowDatabases(std::vector<CString> vstrDBName);

	/*
	* UseDatabases
	* 使用数据库
	* param		strDBName:数据库名
	* return	0:成功	其他:错误代码
	*/
	int UseDatabases(CString strDBName);


	/************************************************************************/
	/************************************************************************/
	/*************************		    表操作 		  ***********************/
	/************************************************************************/
	/************************************************************************/
	/*
	* CreatTable
	* 创建新表
	* param 	strTableName:表名	vtFieldInfo:字段信息
	* return	0:成功	其他:错误代码
	*/
	int CreatTable(CString strTableName, std::vector<T_FIELD_INFO> vtFieldInfo);

	/*
	* DeleteTable
	* 删除表
	* param		strTableName:表名
	* return	0:成功	其他:错误代码
	*/
	int DeleteTable(CString strTableName);

	/*
	* ClearTable
	* 清空表
	* param		strTableName:表名
	* return	0:成功	其他:错误代码
	*/
	int ClearTable(CString strTableName);

	/*
	* QueryTableInfo
	* 查询表结构
	* param		strTableName:表名	vtFieldInfo:返回表字段信息
	* return	0:成功	其他:错误代码
	*/
	int QueryTableInfo(CString strTableName, std::vector<T_QUERY_FIELD_INFO>& vtFieldInfo);

	/*
	* RenameTable
	* 表重命名
	* param		strTableName:表名	strNewTableName:新表名
	* return	0:成功	其他:错误代码
	*/
	int RenameTable(CString strTableName, CString strNewTableName);


	/************************************************************************/
	/************************************************************************/
	/*************************		   字段操作 	  ***********************/
	/************************************************************************/
	/************************************************************************/

	/*
	* AddField
	* 表中添加新字段
	* param		strTableName:表名	vtFieldInfo:返回表字段信息	strPos:位置标志，默认在最后
	* return	0:成功	其他:错误代码
	*/
	int AddField(CString strTableName, T_FIELD_INFO tFieldInfo, CString strPos = "");

	/*
	* DeleteField
	* 删除表中字段
	* param		strTableName:表名	strFieldName:字段名
	* return	0:成功	其他:错误代码
	*/
	int DeleteField(CString strTableName, CString strFieldName);

	/*
	* ChangeField
	* 修改表中字段
	* param		strTableName:表名	strFieldName:字段名	tNewFieldInfo:新字段信息
	* return	0:成功	其他:错误代码
	*/
	int ChangeField(CString strTableName, CString strFieldName, T_FIELD_INFO tNewFieldInfo);


	/************************************************************************/
	/************************************************************************/
	/*************************		   数据操作 	  ***********************/
	/************************************************************************/
	/************************************************************************/

	/*
	* InsertData
	* 插入数据
	* param		strTableName:表名	mstrData:数据，结构为 [字段名][数据值]
	* return	0:成功	其他:错误代码
	* 注：数据类型位字符时，需加单引号 ['数据值']
	*/
	int InsertData(CString strTableName, std::map<CString, CString> mstrData);

	/*
	* InsertData
	* 插入数据
	* param		strTableName:表名	vstrData:数据，必须覆盖所有字段，暂无数据用 null 占位
	* return	0:成功	其他:错误代码
	* 注：数据类型位字符时，需加单引号 ['数据值']
	*/
	int InsertData(CString strTableName, std::vector<CString> vstrData);

	/*
	* InsertData
	* 插入数据
	* param		strTableName:表名	vvstrData:多组数据，必须覆盖所有字段，暂无数据用 null 占位
	* return	0:成功	其他:错误代码
	* 注：数据类型位字符时，需加单引号 ['数据值']
	*/
	int InsertData(CString strTableName, std::vector< std::vector<CString>> vvstrData);

	/*
	* DeleteData
	* 删除数据
	* param		strTableName:表名	strMark:标志，如: <[字段名]=[字段值]> 或 <[字段名] is null> 或 <[字段名] is not null> 等, 可用 or 或 and
	* return	0:成功	其他:错误代码
	*/
	int DeleteData(CString strTableName, CString strMark);

	/*
	* CleanData
	* 清空表中数据
	* param		strTableName:表名
	* return	0:成功	其他:错误代码
	*/
	int CleanData(CString strTableName);

	/*
	* UpdateData
	* 更新数据
	* param		strTableName:表名	mstrNewData:数据，结构为 [字段名][数据值]	
	*			strMark:标志，如: <[字段名]=[字段值]> 或 <[字段名] is null> 或 <[字段名] is not null> 等, 可用 or 或 and
	* return	0:成功	其他:错误代码
	*/
	int UpdateData(CString strTableName, std::map<CString, CString> mstrNewData, CString strMark);

	/*
	* UpdateData1
	* 更新数据---【零件数量更新专用】
	* param		strTableName:表名	mstrNewData:数据，结构为 [字段名][数据值]
	*			strMark:标志，如: <[字段名]=[字段值]> 或 <[字段名] is null> 或 <[字段名] is not null> 等, 可用 or 或 and
	* return	0:成功	其他:错误代码
	*/
	int UpdateData1(CString strTableName, std::map<CString, CString> mstrNewData, CString strMark);

	/*
	* QueryData
	* 查询表中所有数据
	* param		strTableName:表名	vvstrData:数据
	* return	0:成功	其他:错误代码
	*/
	int QueryData(CString strTableName, std::vector< std::vector<CString>>& vvstrData);

	/*
	* QueryData
	* 三表联查（formula,formula_part,part）
	* param		formulaID:formulaID	 vvstrData:数据
	* return	0:成功	其他:错误代码
	*/
	int QueryData(int formulaID, std::vector<std::vector<CString>>& vvstrData);

	/*
	* QueryData
	* 查询表中所有数据
	* param		strTableName:表名	
				strMark:标志，如: <[字段名]=[字段值]> 或 <[字段名] is null> 或 <[字段名] is not null> 或 <[字段名] in([数据值], [数据值], [数据值]...)> 等 可用 limit 限制查询数量
				vvstrData:
	* return	0:成功	其他:错误代码
	*/
	int QueryData(CString strTableName, CString strMark, std::vector< std::vector<CString>>& vvstrData);

	/*
	* QueryDataNum
	* 查询表中数据数量
	* param		strTableName:表名	nNum:表中数据数量
	* return	0:成功	其他:错误代码
	*/
	int QueryDataNum(CString strTableName, unsigned long long& nNum);




private:
	bool m_bConnect = true;
	MYSQL m_sqlCon;
	CString GetField(T_FIELD_INFO tFieldInfo);
};

#endif // ENABLE_MY_SQL

