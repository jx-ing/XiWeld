#pragma once
#include ".\Apps\PLib\BasicFunc\ChoiceResources.h"
#if ENABLE_MY_SQL

#include ".\Apps\PLib\BasicFunc\FileProcess.h"
#include <map>
#include "mysql.h"

/*
* MYSQL_CHECK_RTN
* ��鷵��ֵ
*
*/
#define MYSQL_CHECK_RTN(nRtn) {int n = nRtn;if(n != 0)return n;}

/*
* CHECK_DATABASE_CONNECT
* ������ݿ��Ƿ�����
*
*/
#define  CHECK_DATABASE_CONNECT if(!m_bConnect) return -1;

/*
* aacValType
* ���ݿ��б�������
* 
* ע��BOOL ������ TINYINT(1) ���
*/
const char g_aacValType[][128] =
{
	{"INT"}, {"TINYINT(1)"}, {"DOUBLE"}, {"CHAR"}, {"DATETIME"}, {"DATE"}, {"TEXT"}
};
const int g_nMaxTypeNum = 7;

typedef struct 
{
	CString strName;//�ֶ���
	int nType;//����
	int nLenght = -1;//��������
	bool bUnsigned = false;//�Ƿ��޷���
	CString strDefault = "";//Ĭ��ֵ
	CString strExplanatoryNote = "";//˵��
	CString foreignKeyTableName = "";//�����Ϣ,��Ҫ������ձ����
}T_FIELD_INFO;

typedef struct
{
	CString strName;//�ֶ���
	CString strType;//����
	CString strNull;//�Ƿ��Ϊ null
	CString strKey;//����
	CString strDefault;//Ĭ��ֵ
	CString strExtra;//У�Թ���
}T_QUERY_FIELD_INFO;

class Database_mysql
{
public:
	~Database_mysql();

	/************************************************************************/
	/************************************************************************/
	/*************************		   ͨ�ò��� 	  ***********************/
	/************************************************************************/
	/************************************************************************/

	/*
	* GetErrorInfo
	* ��ȡ������Ϣ
	* param 	
	* return	������Ϣ
	*/
	CString GetErrorInfo();

	/*
	* QueryCmd
	* ��ѯ��Ϣ
	* param
	* return	0:�ɹ�	����:�������
	*/
	int QueryCmd(CString strCmd);

	/*
	* GetQueryResult
	* ��ȡ��ѯ���
	* param
	* return	�������
	*/
	MYSQL_RES* GetQueryResult();

	/*
	* ReleaseQueryResult
	* �ͷŲ�ѯ���
	* param
	* return	
	*/
	void ReleaseQueryResult(MYSQL_RES* mqlResult);

	/************************************************************************/
	/************************************************************************/
	/*************************		 ���ݿ���� 	  ***********************/
	/************************************************************************/
	/************************************************************************/
	/*
	* CreateDatabases
	* �������ݿ�
	* param 	strDBName:���ݿ���
	* return	0:�ɹ�	����:�������
	*/
	int CreateDatabases(CString strDBName);

	/*
	* Connect
	* �������ݿ�
	* param 	strIP:ip��ַ	strUser:�û���	strKey:����	strDBName:���ݿ���	nPort:�˿ں�
	* return	0:�ɹ�	����:�������
	*/
	int Connect(CString strIP, CString strUser, CString strKey, CString strDBName, int nPort);

	/*
	* Disconnect
	* �Ͽ����ݿ�����
	* param
	* return	0:�ɹ�	����:�������
	*/
	int Disconnect();

	/*
	* ShowDatabases
	* ��ʾ�������ݿ�
	* param		vstrDBName:���ݿ���
	* return	0:�ɹ�	����:�������
	*/
	int ShowDatabases(std::vector<CString> vstrDBName);

	/*
	* UseDatabases
	* ʹ�����ݿ�
	* param		strDBName:���ݿ���
	* return	0:�ɹ�	����:�������
	*/
	int UseDatabases(CString strDBName);


	/************************************************************************/
	/************************************************************************/
	/*************************		    ����� 		  ***********************/
	/************************************************************************/
	/************************************************************************/
	/*
	* CreatTable
	* �����±�
	* param 	strTableName:����	vtFieldInfo:�ֶ���Ϣ
	* return	0:�ɹ�	����:�������
	*/
	int CreatTable(CString strTableName, std::vector<T_FIELD_INFO> vtFieldInfo);

	/*
	* DeleteTable
	* ɾ����
	* param		strTableName:����
	* return	0:�ɹ�	����:�������
	*/
	int DeleteTable(CString strTableName);

	/*
	* ClearTable
	* ��ձ�
	* param		strTableName:����
	* return	0:�ɹ�	����:�������
	*/
	int ClearTable(CString strTableName);

	/*
	* QueryTableInfo
	* ��ѯ��ṹ
	* param		strTableName:����	vtFieldInfo:���ر��ֶ���Ϣ
	* return	0:�ɹ�	����:�������
	*/
	int QueryTableInfo(CString strTableName, std::vector<T_QUERY_FIELD_INFO>& vtFieldInfo);

	/*
	* RenameTable
	* ��������
	* param		strTableName:����	strNewTableName:�±���
	* return	0:�ɹ�	����:�������
	*/
	int RenameTable(CString strTableName, CString strNewTableName);


	/************************************************************************/
	/************************************************************************/
	/*************************		   �ֶβ��� 	  ***********************/
	/************************************************************************/
	/************************************************************************/

	/*
	* AddField
	* ����������ֶ�
	* param		strTableName:����	vtFieldInfo:���ر��ֶ���Ϣ	strPos:λ�ñ�־��Ĭ�������
	* return	0:�ɹ�	����:�������
	*/
	int AddField(CString strTableName, T_FIELD_INFO tFieldInfo, CString strPos = "");

	/*
	* DeleteField
	* ɾ�������ֶ�
	* param		strTableName:����	strFieldName:�ֶ���
	* return	0:�ɹ�	����:�������
	*/
	int DeleteField(CString strTableName, CString strFieldName);

	/*
	* ChangeField
	* �޸ı����ֶ�
	* param		strTableName:����	strFieldName:�ֶ���	tNewFieldInfo:���ֶ���Ϣ
	* return	0:�ɹ�	����:�������
	*/
	int ChangeField(CString strTableName, CString strFieldName, T_FIELD_INFO tNewFieldInfo);


	/************************************************************************/
	/************************************************************************/
	/*************************		   ���ݲ��� 	  ***********************/
	/************************************************************************/
	/************************************************************************/

	/*
	* InsertData
	* ��������
	* param		strTableName:����	mstrData:���ݣ��ṹΪ [�ֶ���][����ֵ]
	* return	0:�ɹ�	����:�������
	* ע����������λ�ַ�ʱ����ӵ����� ['����ֵ']
	*/
	int InsertData(CString strTableName, std::map<CString, CString> mstrData);

	/*
	* InsertData
	* ��������
	* param		strTableName:����	vstrData:���ݣ����븲�������ֶΣ����������� null ռλ
	* return	0:�ɹ�	����:�������
	* ע����������λ�ַ�ʱ����ӵ����� ['����ֵ']
	*/
	int InsertData(CString strTableName, std::vector<CString> vstrData);

	/*
	* InsertData
	* ��������
	* param		strTableName:����	vvstrData:�������ݣ����븲�������ֶΣ����������� null ռλ
	* return	0:�ɹ�	����:�������
	* ע����������λ�ַ�ʱ����ӵ����� ['����ֵ']
	*/
	int InsertData(CString strTableName, std::vector< std::vector<CString>> vvstrData);

	/*
	* DeleteData
	* ɾ������
	* param		strTableName:����	strMark:��־����: <[�ֶ���]=[�ֶ�ֵ]> �� <[�ֶ���] is null> �� <[�ֶ���] is not null> ��, ���� or �� and
	* return	0:�ɹ�	����:�������
	*/
	int DeleteData(CString strTableName, CString strMark);

	/*
	* CleanData
	* ��ձ�������
	* param		strTableName:����
	* return	0:�ɹ�	����:�������
	*/
	int CleanData(CString strTableName);

	/*
	* UpdateData
	* ��������
	* param		strTableName:����	mstrNewData:���ݣ��ṹΪ [�ֶ���][����ֵ]	
	*			strMark:��־����: <[�ֶ���]=[�ֶ�ֵ]> �� <[�ֶ���] is null> �� <[�ֶ���] is not null> ��, ���� or �� and
	* return	0:�ɹ�	����:�������
	*/
	int UpdateData(CString strTableName, std::map<CString, CString> mstrNewData, CString strMark);

	/*
	* UpdateData1
	* ��������---�������������ר�á�
	* param		strTableName:����	mstrNewData:���ݣ��ṹΪ [�ֶ���][����ֵ]
	*			strMark:��־����: <[�ֶ���]=[�ֶ�ֵ]> �� <[�ֶ���] is null> �� <[�ֶ���] is not null> ��, ���� or �� and
	* return	0:�ɹ�	����:�������
	*/
	int UpdateData1(CString strTableName, std::map<CString, CString> mstrNewData, CString strMark);

	/*
	* QueryData
	* ��ѯ������������
	* param		strTableName:����	vvstrData:����
	* return	0:�ɹ�	����:�������
	*/
	int QueryData(CString strTableName, std::vector< std::vector<CString>>& vvstrData);

	/*
	* QueryData
	* �������飨formula,formula_part,part��
	* param		formulaID:formulaID	 vvstrData:����
	* return	0:�ɹ�	����:�������
	*/
	int QueryData(int formulaID, std::vector<std::vector<CString>>& vvstrData);

	/*
	* QueryData
	* ��ѯ������������
	* param		strTableName:����	
				strMark:��־����: <[�ֶ���]=[�ֶ�ֵ]> �� <[�ֶ���] is null> �� <[�ֶ���] is not null> �� <[�ֶ���] in([����ֵ], [����ֵ], [����ֵ]...)> �� ���� limit ���Ʋ�ѯ����
				vvstrData:
	* return	0:�ɹ�	����:�������
	*/
	int QueryData(CString strTableName, CString strMark, std::vector< std::vector<CString>>& vvstrData);

	/*
	* QueryDataNum
	* ��ѯ������������
	* param		strTableName:����	nNum:������������
	* return	0:�ɹ�	����:�������
	*/
	int QueryDataNum(CString strTableName, unsigned long long& nNum);




private:
	bool m_bConnect = true;
	MYSQL m_sqlCon;
	CString GetField(T_FIELD_INFO tFieldInfo);
};

#endif // ENABLE_MY_SQL

