#pragma once
 
#include "CRange.h"
#include "CWorkbook.h"
#include "CWorkbooks.h"
#include "CWorksheet.h"
#include "CWorksheets.h"
#include "CApplication.h"

#define vOpt COleVariant((long) DISP_E_PARAMNOTFOUND, VT_ERROR)

//排序方式
#define xlAscending (long) 1										//升序排序									
#define xlDescending (long) 2										//降序排序	

//有无标题
#define xlHeader (long) 1											//选择的区域有标题
#define xlNoHeader (long) 2											//选择的区域无标题

//区分大小写
#define xlMatchCase COleVariant((long) 1)							//区分大小写
#define xlIgnoreCase COleVariant((long) 0)							//不区分大小写

//排序方向
#define xlTopToBottom (long) 1										//垂直方向进行排序
#define xlLeftToRight (long) 2										//水平方向进行排序

//排序依据
#define xlPinYin (long) 1											//拼音排序
#define xlStroke (long) 2											//笔画排序

//指定要搜索的数据的类型
#define xlComments COleVariant( -4144L )							//批注
#define xlFormulas COleVariant( -4123L )							//公式
#define xlValues COleVariant( -4163L )								//值

//匹配方式
#define xlWhole COleVariant( 1L )									//全字匹配
#define xlPart COleVariant( 2L )									//部分匹配

//指定搜索区域的次序
#define xlByRows COleVariant( 1L )									//搜索行
#define xlByColumns COleVariant( 2L )								//搜索列

//指定搜索区域时的搜索方向
#define xlNext 1L													//下一个
#define xlPrev 2L													//上一个

typedef enum
{
	GET_CELL_BLANK			= 1,	//空白
	GET_CELL_DATE			= 2,	//实际为日期类型
	GET_CELL_INT			= 3,	//实际为int类型
	GET_CELL_DOUBLE			= 4,	//实际为double类型
	GET_CELL_CSTRING		= 5,	//实际为CString类型
}E_GET_CELL_TYPE;

typedef struct
{
	unsigned long ulRowNo;
	std::vector<CString> vstrData;
}T_EXCEL_CSTRING_DATA;

typedef struct
{
	long lRow;
	long lColumn;
}T_CELL_NO;

class COpExcel
{
	/**************************************基本操作**************************************/
public:
	//构造
	COpExcel(void);

	//析构
	~COpExcel(void);
 
	//初始化EXCEL OLE
	static BOOL InitExcel();

	//释放EXCEL的 OLE
	static void ReleaseExcel();

	//T_CELL_NO转CString
	CString GetCellName(T_CELL_NO tCellNo);

	/**************************************单个Excel操作**************************************/
public:
	//显示Excel
	void ShowInExcel(BOOL bShow);

	//打开文件
	BOOL OpenExcelFile(const TCHAR * file_name);

	//关闭打开的Excel 文件，有时候打开EXCEL文件就要
	void CloseExcelFile(BOOL if_save = FALSE);

	//另存为一个EXCEL文件
	void SaveasXSLFile(const CString &xls_file);

	//取得打开文件的名称
	CString GetOpenFileName();

	/**************************************工作表操作**************************************/
public:
	//使用某个sheet
	BOOL LoadSheet(long table_index, BOOL pre_load = FALSE);

	//通过名称使用某个sheet
	BOOL LoadSheet(const TCHAR* sheet, BOOL pre_load = FALSE);

	//通过序号取得某个Sheet的名称
	CString GetSheetName(long table_index);

	//得到Sheet的总数
	int GetSheetCount();

	//取得打开sheet的名称
	CString GetLoadSheetName();

	/**************************************行操作**************************************/
public:
	//取得行的总数
	int GetRowCount();

	//按行号删除某行
	void DeleteRow(int nRowNo);

	//按行号插入一行
	void InsertRow(int nRowNo);

	//按行号插入一行并写入一行数据
	void InsertRow(int nRowNo, std::vector<double> vdExcelData);

	//按行写入数据,每行数据可以不等量
	void SetCell(std::vector<T_EXCEL_CSTRING_DATA> vtExcelData);

	//从特定行号开始按行顺序写入数据,每行数据可以不等量
	void SetCell(long lRowNo, std::vector<std::vector<double>> vvdExcelData);

	/**************************************列操作**************************************/
public:
	//取得列的总数
	int GetColumnCount();

	//取得列的名称，比如27->AA
	CString GetColumnName(long iColumn);

	//以某列为基准排序
	void Sort(long nColumnNo, long nSortOrder = xlAscending, long nHeader = xlHeader, long nArrangement = xlPinYin);

	/**************************************区域操作**************************************/
public:
	//设定一个CELL的值,建议设定完成后再次使用前进行一次预加载
	void SetCell(long irow, long icolumn, CString new_string);

	//得到一个CELL的值
	E_GET_CELL_TYPE GetCell(double &dValue, long iRow, long iColumn);
	E_GET_CELL_TYPE GetCell(int &nValue, long iRow, long iColumn);
	E_GET_CELL_TYPE GetCell(CString &strValue, long iRow, long iColumn);
	CString GetCellByName(CString rowName,CString colName);

	//当前工作表全部范围内查找关键词，返回所有查找到的行号和列号
	void Find(CString strFind, std::vector<T_CELL_NO> &vtCellNo);

	//当前工作表设定范围内查找关键词，返回所有查找到的行号和列号
	void Find(CString strFind, T_CELL_NO tFirstCell, T_CELL_NO tEndCell,std::vector<T_CELL_NO> &vtCellNo);
	void Find(CString strFind, CRange range, std::vector<T_CELL_NO> &vtCellNo);

protected:
	//预先加载
	void PreLoadSheet(); 
 
public:
	CString					m_strFileName;		//打开的EXCEL文件名称
	CWorkbooks				m_Books;
	CWorkbook				m_Book;
	CWorksheets				m_sheets;
	CWorksheet				m_sheet;
	CRange					m_Range;
 	static CApplication		m_app;
	BOOL					m_bAlreadyPreload;	//预加载标志
	COleSafeArray			m_arrayMessage;		//预加载内容
	LPDISPATCH				m_lpDisp;  
};
//以下是新版Excel库，速度更快更稳定

#include ".\Apps\PLib\BasicFunc\Const.h"
#include <afxdb.h>
#include <odbcinst.h>
#pragma comment(lib,"odbc32.lib")
#pragma comment(lib,"odbccp32.lib")
#pragma comment(lib,"legacy_stdio_definitions.lib")

class OpenExcel
{
public:
	OpenExcel();
	~OpenExcel();

	bool Init(XiBase::CLog *pLog);
	bool Open(CString sExcelFile);
	bool Restart();
	bool Close();
	bool Read(CString sSheet, std::vector<std::vector<CString>>& vvsValue);
	bool Read(CString sSheet, std::map<CString, std::vector<CString>>& vvsValue);//以第一列为map索引
	bool Read(CString sSheet, std::vector<std::vector<double>>& vvdValue);
	bool Write(CString sSheet, const std::vector<CString>& vsTitle, const std::vector<std::vector<CString>>& vvsValue);
	bool Write(CString sSheet, const std::vector<CString>& vsTitle, const std::vector<std::vector<double>>& vvdValue);
	bool Drop(CString sSheet, const std::vector<CString>& vsTitle);
	bool Add(CString sSheet, const std::vector<CString>& vsTitle, const std::vector<CString>& vsValue);
	bool Update(CString sSheet, const std::vector<CString>& vsTitle, const std::vector<CString>& vsValue, int nRow);
	CString GetErrorString();
	void ShowErrorString();

private:
	//获取创建表格语句
	CString GetCreateString(CString sSheet, const std::vector<CString>& vsTitle, bool bNew = true);
	//获取删除表格语句
	CString GetDropString(CString sSheet);
	//获取查询表格语句
	CString GetSelectString(CString sSheet);
	//获取插入表格语句
	CString GetInsertString(CString sSheet, const std::vector<CString>& vsTitle, int nRow = -1);
	//获取修改表格语句
	CString GetUpdateString(CString sSheet, const std::vector<CString>& vsTitle, int nRow);

	void SetErrorString(CString sError = _T(""));
	CString GetExcelDriver(CString sDriverName);

	XiBase::CLog* m_pLog = NULL;
	CString m_sErrorInfo = _T("");
	CString m_sExcelFile = _T("");
	CString m_sDriver = _T("");
	CDatabase* m_pDatabase = NULL;
};