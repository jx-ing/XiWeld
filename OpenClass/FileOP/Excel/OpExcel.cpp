#include "StdAfx.h"
#include "OpExcel.h"
 
#ifdef _DEBUG
#define new DEBUG_NEW
#endif
 
COpExcel::COpExcel(void):m_bAlreadyPreload(FALSE)
{
 
}
 
COpExcel::~COpExcel(void)
{
 
}
 
COleVariant
covTrue((short)TRUE),
covFalse((short)FALSE),
covOptional((long)DISP_E_PARAMNOTFOUND, VT_ERROR);
 
CApplication COpExcel::m_app;
 
//初始化EXCEL文件，
BOOL COpExcel::InitExcel()
{ 
	BOOL bRtn =FALSE ;
		
	if (m_app.CreateDispatch("Excel.Application"))
	{
		bRtn = TRUE;
	}
 
	if (!bRtn)
	{
		AfxMessageBox(_T("创建Excel服务失败,你可能没有安装EXCEL，请检查!")); 
		return bRtn;
	}
 
	m_app.put_DisplayAlerts(FALSE); 
	return bRtn;
}
 
//
void COpExcel::ReleaseExcel()
{ 
	m_app.Quit();
	m_app.ReleaseDispatch();
	m_app=NULL;
}

CString COpExcel::GetCellName(T_CELL_NO tCellNo)
{
	CString strCellName;
	strCellName.Format("%d", tCellNo.lRow);
	strCellName = GetColumnName(tCellNo.lColumn) + strCellName;
	return strCellName;
}
 
 
//关闭打开的Excel 文件,默认情况不保存文件
void COpExcel::CloseExcelFile(BOOL if_save)
{
	//如果已经打开，关闭文件
	if (m_strFileName.IsEmpty() == FALSE)
	{
		if (if_save)
		{
			SaveasXSLFile(m_strFileName);
		}
		else
		{
			m_Book.Close(COleVariant(short(FALSE)), COleVariant(m_strFileName), covOptional);
			m_Books.Close();
		}
		
		//打开文件的名称清空
		m_strFileName.Empty();
	}

	m_Range.ReleaseDispatch();
	m_sheet.ReleaseDispatch();
	m_sheets.ReleaseDispatch();	
	m_Book.ReleaseDispatch();
	m_Books.ReleaseDispatch();
}
 
//打开excel文件
BOOL COpExcel::OpenExcelFile(const TCHAR *file_name)
{
	//先关闭
	CloseExcelFile();
 
	//m_Books.AttachDispatch(m_app.get_Workbooks(),1);  
	//COleVariant varPath(file_name);  
	//m_Book.AttachDispatch(m_Books.Add(varPath));  
 
	//利用模板文件建立新文档 
	m_Books.AttachDispatch(m_app.get_Workbooks(),true); 
 
	//LPDISPATCH lpDis = NULL;
	m_lpDisp = m_Books.Add(COleVariant(file_name)); 
	if (m_lpDisp)
	{
		m_Book.AttachDispatch(m_lpDisp); 
		//得到Worksheets 
		m_sheets.AttachDispatch(m_Book.get_Worksheets(),true); 
 
		//记录打开的文件名称
		m_strFileName = file_name;
 
		return TRUE;
	}
 
	return TRUE;
}
 
 
//
void COpExcel::ShowInExcel(BOOL bShow)
{
	m_app.put_Visible(bShow);
	m_app.put_UserControl(bShow);
}
 
void COpExcel::SaveasXSLFile(const CString &xls_file)
{
	m_Book.SaveAs(COleVariant(xls_file),
		covOptional,
		covOptional,
		covOptional,
		covOptional,
		covOptional,
		0,
		covOptional,
		covOptional,
		covOptional,
		covOptional,
		covOptional);
	return;
}
 
 
int COpExcel::GetSheetCount()
{
	return m_sheets.get_Count();
}
 
 
CString COpExcel::GetSheetName(long table_index)
{
	CWorksheet sheet;
	sheet.AttachDispatch(m_sheets.get_Item(COleVariant((long)table_index)),true);
	CString name = sheet.get_Name();
	sheet.ReleaseDispatch();
	return name;
}
 
//按照序号加载Sheet表格,可以提前加载所有的表格内部数据
BOOL COpExcel::LoadSheet(long table_index,BOOL pre_load)
{
	LPDISPATCH lpDis = NULL;
	m_Range.ReleaseDispatch();
	m_sheet.ReleaseDispatch();
	lpDis = m_sheets.get_Item(COleVariant((long)table_index));
	if (lpDis)
	{
		m_sheet.AttachDispatch(lpDis,true);
		m_Range.AttachDispatch(m_sheet.get_Cells(), true);
	}
	else
	{
		return FALSE;
	}
 
	m_bAlreadyPreload = FALSE;
	//如果进行预先加载
	if (pre_load)
	{
		PreLoadSheet();
		m_bAlreadyPreload = TRUE;
	}
 
	return TRUE;
}
 
 
//按照名称加载Sheet表格,可以提前加载所有的表格内部数据
BOOL COpExcel::LoadSheet(const TCHAR* sheet,BOOL pre_load)
{
	LPDISPATCH lpDis = NULL;
	m_Range.ReleaseDispatch();
	m_sheet.ReleaseDispatch();
	lpDis = m_sheets.get_Item(COleVariant(sheet));
	if (lpDis)
	{
		m_sheet.AttachDispatch(lpDis,true);
		m_Range.AttachDispatch(m_sheet.get_Cells(), true);
 
	}
	else
	{
		return FALSE;
	}
	//
	m_bAlreadyPreload = FALSE;
	//如果进行预先加载
	if (pre_load)
	{
		m_bAlreadyPreload = TRUE;
		PreLoadSheet();
	}
 
	return TRUE;
}
 
 
//得到列的总数
int COpExcel::GetColumnCount()
{
	CRange range;
	CRange usedRange;
	usedRange.AttachDispatch(m_sheet.get_UsedRange(), true);
	range.AttachDispatch(usedRange.get_Columns(), true);
	int count = range.get_Count();
	usedRange.ReleaseDispatch();
	range.ReleaseDispatch();
	return count;
}
 
//得到行的总数
int COpExcel::GetRowCount()
{
	CRange range;
	CRange usedRange;
	usedRange.AttachDispatch(m_sheet.get_UsedRange(), true);
	range.AttachDispatch(usedRange.get_Rows(), true);
	int count = range.get_Count();
	usedRange.ReleaseDispatch();
	range.ReleaseDispatch();
	return count;
}
 
void COpExcel::DeleteRow(int nRowNo)
{
	CString strRowNo;
	strRowNo.Format("A%d", nRowNo);
	LPDISPATCH  lpDisp = NULL;
	lpDisp = m_sheet.get_Range(COleVariant(strRowNo), COleVariant(strRowNo));
	m_Range.AttachDispatch(lpDisp);
	m_Range.AttachDispatch(m_Range.get_EntireRow());
	m_Range.Delete(vtMissing);
}

void COpExcel::InsertRow(int nRowNo)
{
	CString strRowNo;
	strRowNo.Format("A%d", nRowNo);
	LPDISPATCH  lpDisp = NULL;
	lpDisp = m_sheet.get_Range(COleVariant(strRowNo), COleVariant(strRowNo));
	m_Range.AttachDispatch(lpDisp);
	m_Range.AttachDispatch(m_Range.get_EntireRow());
	m_Range.Insert(vtMissing, _variant_t(0));
}

void COpExcel::InsertRow(int nRowNo, std::vector<double> vdExcelData)
{
	std::vector<std::vector<double>> vvdExcelData;
	vvdExcelData.push_back(vdExcelData);
	InsertRow(nRowNo);
	SetCell(nRowNo, vvdExcelData);
}

//返回打开的EXCEL文件名称
CString COpExcel::GetOpenFileName()
{
	return m_strFileName;
}
 
//取得打开sheet的名称
CString COpExcel::GetLoadSheetName()
{
	return m_sheet.get_Name();
}
 
 
//取得列的名称，比如27->AA
CString COpExcel::GetColumnName(long icolumn)
{   
	CString strColumnName = _T("");
	while(icolumn > 0)
	{
		int num_data = icolumn % 26;
		icolumn /= 26;
		if (num_data == 0)
		{
			num_data = 26;
			icolumn--;
		}
		char ch = 'A' + num_data - 1;
		strColumnName = ch + strColumnName;
	}
	return strColumnName;
}
 
//预先加载
void COpExcel::PreLoadSheet()
{
	CRange used_range;
	used_range = m_sheet.get_UsedRange();    
 
	VARIANT ret_ary = used_range.get_Value2();
	if (!(ret_ary.vt & VT_ARRAY))
	{
		return;
	}
	m_arrayMessage.Clear();
	m_arrayMessage.Attach(ret_ary); 
}
 
void COpExcel::SetCell(long irow, long icolumn,CString new_string)
{
	COleVariant new_value(new_string);
	CRange start_range;
	CRange write_range;
	start_range.AttachDispatch(m_sheet.get_Cells());
	write_range.AttachDispatch(start_range.get_Item(COleVariant(irow), COleVariant(icolumn)).pdispVal);
	write_range.put_Value2(new_value);
	start_range.ReleaseDispatch();
	write_range.ReleaseDispatch();

// 	if (already_preload_)//溢出
// 	{
// 		long read_address[2];
// 		read_address[0] = irow;
// 		read_address[1] = icolumn;
// 		ole_safe_array_.PutElement(read_address, &new_value);
// 	}
}
 
void COpExcel::SetCell(std::vector<T_EXCEL_CSTRING_DATA> vtExcelData)
{
	if (vtExcelData.size() <= 0)
	{
		return;
	}
	CRange start_range;
	CRange write_range;
	start_range.AttachDispatch(m_sheet.get_Cells());
	for (long i = 0; i < vtExcelData.size(); i++)
	{
		for (long j = 0; j < vtExcelData[i].vstrData.size(); j++)
		{
			write_range.AttachDispatch(start_range.get_Item(COleVariant((ULONGLONG)vtExcelData[i].ulRowNo), COleVariant(j + 1)).pdispVal);
			write_range.put_Value2(COleVariant(vtExcelData[i].vstrData[j]));
		}	
	}	
	start_range.ReleaseDispatch();
	write_range.ReleaseDispatch();
}

void COpExcel::SetCell(long lRowNo, std::vector<std::vector<double>> vvdExcelData)
{
	if (vvdExcelData.size() <= 0)
	{
		return;
	}
	CRange start_range;
	CRange write_range;
	start_range.AttachDispatch(m_sheet.get_Cells());
	for (long i = 0; i < vvdExcelData.size(); i++)
	{
		for (long j = 0; j < vvdExcelData[i].size(); j++)
		{
			write_range.AttachDispatch(start_range.get_Item(COleVariant((ULONGLONG)(lRowNo + i)), COleVariant(j + 1)).pdispVal);
			write_range.put_Value2(COleVariant(vvdExcelData[i][j]));
		}
	}
	start_range.ReleaseDispatch();
	write_range.ReleaseDispatch();
}

E_GET_CELL_TYPE COpExcel::GetCell(CString &strValue, long iRow, long iColumn)
{  
	COleVariant vResult ; 
	//字符串
	if (m_bAlreadyPreload == FALSE)
	{
		m_Range.AttachDispatch(m_Range.get_Item (COleVariant((long)iRow),COleVariant((long)iColumn)).pdispVal, true);
		vResult =m_Range.get_Value2();
	}
	//如果数据依据预先加载了
	else
	{
		long read_address[2];
		VARIANT val;
		read_address[0] = iRow;
		read_address[1] = iColumn;
		m_arrayMessage.GetElement(read_address, &val);
		vResult = val;
	}
 
	E_GET_CELL_TYPE eGetCellReturn;
	if(vResult.vt == VT_BSTR)       //字符串
	{
		strValue = vResult.bstrVal;
		eGetCellReturn = GET_CELL_CSTRING;
	}
	else if (vResult.vt == VT_INT)	//int
	{
		strValue.Format(_T("%d"),vResult.pintVal);
		eGetCellReturn = GET_CELL_INT;
	}
	else if (vResult.vt == VT_R8)     //8字节的数字
	{
		strValue.Format(_T("%lf"),vResult.dblVal);
		eGetCellReturn = GET_CELL_DOUBLE;
	}
	else if(vResult.vt == VT_DATE)    //时间格式
	{
		SYSTEMTIME st;
		VariantTimeToSystemTime(vResult.date, &st);
		CTime tm(st);
		strValue = tm.Format(_T("%Y-%m-%d"));
		eGetCellReturn = GET_CELL_DATE;
	}
	else if(vResult.vt == VT_EMPTY)   //单元格空的
	{
		strValue = _T("");
		eGetCellReturn = GET_CELL_BLANK;
	} 
 
	m_Range.ReleaseDispatch();
 
	return eGetCellReturn;
} 
 
E_GET_CELL_TYPE COpExcel::GetCell(double &dValue, long iRow, long iColumn)
{
	dValue = 0;
	COleVariant vResult;
	//字符串
	if (m_bAlreadyPreload == FALSE)
	{
		m_Range.AttachDispatch(m_Range.get_Item(COleVariant((long)iRow), COleVariant((long)iColumn)).pdispVal, true);
		vResult = m_Range.get_Value2();
	}
	//如果数据依据预先加载了
	else
	{
		long read_address[2];
		VARIANT val;
		read_address[0] = iRow;
		read_address[1] = iColumn;
		m_arrayMessage.GetElement(read_address, &val);
		vResult = val;
	}

	E_GET_CELL_TYPE eGetCellReturn;
	if (vResult.vt == VT_BSTR)       //字符串
	{
		eGetCellReturn = GET_CELL_CSTRING;
	}
	else if (vResult.vt == VT_INT)	//int
	{
		eGetCellReturn = GET_CELL_INT;
	}
	else if (vResult.vt == VT_R8)     //8字节的数字
	{
		dValue = vResult.dblVal;
		eGetCellReturn = GET_CELL_DOUBLE;
	}
	else if (vResult.vt == VT_DATE)    //时间格式
	{
		eGetCellReturn = GET_CELL_DATE;
	}
	else if (vResult.vt == VT_EMPTY)   //单元格空的
	{
		eGetCellReturn = GET_CELL_BLANK;
	}

	m_Range.ReleaseDispatch();

	return eGetCellReturn;
}

E_GET_CELL_TYPE COpExcel::GetCell(int &nValue, long iRow, long iColumn)
{
	nValue = 0;
	COleVariant vResult;
	//字符串
	if (m_bAlreadyPreload == FALSE)
	{
		m_Range.AttachDispatch(m_Range.get_Item(COleVariant((long)iRow), COleVariant((long)iColumn)).pdispVal, true);
		vResult = m_Range.get_Value2();
	}
	//如果数据依据预先加载了
	else
	{
		long read_address[2];
		VARIANT val;
		read_address[0] = iRow;
		read_address[1] = iColumn;
		m_arrayMessage.GetElement(read_address, &val);
		vResult = val;
	}

	E_GET_CELL_TYPE eGetCellReturn;
	if (vResult.vt == VT_BSTR)       //字符串
	{
		eGetCellReturn = GET_CELL_CSTRING;
	}
	else if (vResult.vt == VT_INT)	//int
	{
		nValue = vResult.intVal;
		eGetCellReturn = GET_CELL_INT;
	}
	else if (vResult.vt == VT_R8)     //8字节的数字
	{
		eGetCellReturn = GET_CELL_DOUBLE;
	}
	else if (vResult.vt == VT_DATE)    //时间格式
	{
		eGetCellReturn = GET_CELL_DATE;
	}
	else if (vResult.vt == VT_EMPTY)   //单元格空的
	{
		eGetCellReturn = GET_CELL_BLANK;
	}

	m_Range.ReleaseDispatch();

	return eGetCellReturn;
}

CString COpExcel::GetCellByName(CString rowName,CString colName)  
{  
	COleVariant value;  
	CString strValue;  
	long row=0,col=0;  
	long re_row=0,re_col=0;
 
	m_Range.AttachDispatch(m_sheet.get_Cells(),TRUE);  
	for (row=1,col=1;col<m_Range.get_Column();col++)  
	{  
		value=m_Range.get_Item(_variant_t(row),_variant_t(col));                  //返回的类型是VT_DISPATCH 这是一个指针  
		m_Range.AttachDispatch(value.pdispVal,TRUE);  
		VARIANT value2=m_Range.get_Text();  
		CString strValue= COLE2CT(value2.bstrVal);
		if (strValue==colName)  
			break;  
	}  
	re_col=col;  
	for (row=1,row=1;row<m_Range.get_Row();row++)  
	{  
		value=m_Range.get_Item(_variant_t(row),_variant_t(col));                  //返回的类型是VT_DISPATCH 这是一个指针  
		m_Range.AttachDispatch(value.pdispVal,TRUE);  
		VARIANT value2=m_Range.get_Text();  
		CString strValue=COLE2CT(value2.bstrVal);  
		if (strValue==rowName)        
			break;  
	}  
	re_row=row; 

	GetCell(strValue, re_row, re_col);
	return strValue; 
}  

void COpExcel::Sort(long nColumnNo, long nSortOrder, long nHeader, long nArrangement)
{
	CString strRange1;
	strRange1 = GetColumnName(nColumnNo);
	int nColumnCount = GetColumnCount();
	CString strRange2;
	strRange2.Format("%d", nColumnCount);
	strRange2 = strRange1 + strRange2;
	strRange1 += "1";
	VARIANT key1;
	V_VT(&key1) = VT_DISPATCH;
	V_DISPATCH(&key1) = m_sheet.get_Range(COleVariant(strRange1), COleVariant(strRange2));
	m_Range.AttachDispatch(m_sheet.get_Cells(), true);
	m_Range.Sort(key1, nSortOrder, vOpt, vOpt, nSortOrder, vOpt, nSortOrder, nHeader, vOpt, xlMatchCase, xlTopToBottom, nArrangement, 0, 0, 0);
}

void COpExcel::Find(CString strFind, std::vector<T_CELL_NO> &vtCellNo)
{
	CRange range = m_sheet.get_UsedRange();
	Find(strFind, range, vtCellNo);
}

void COpExcel::Find(CString strFind, T_CELL_NO tFirstCell, T_CELL_NO tEndCell, std::vector<T_CELL_NO> &vtCellNo)
{
	CRange range; 
	range.AttachDispatch(m_sheet.get_Range(COleVariant(GetCellName(tFirstCell)), COleVariant(GetCellName(tEndCell))));
	Find(strFind, range, vtCellNo);
}

void COpExcel::Find(CString strFind, CRange range, std::vector<T_CELL_NO> &vtCellNo)
{
	T_CELL_NO vtNo;
	vtCellNo.clear();
	LPDISPATCH lpDisp = NULL;
	COleVariant lIsDouble = 1L;
	lpDisp = range.Find(COleVariant(strFind), vOpt, xlFormulas, xlPart,
		xlByRows, xlNext, xlMatchCase, lIsDouble, vOpt);
	if (lpDisp)
	{
		CRange iFirst = range;
		iFirst.AttachDispatch(lpDisp);
		iFirst.Select();
		iFirst.Activate();
		CRange iNext = range.FindNext(vOpt);
		do
		{
			vtNo.lRow = iNext.get_Row();
			vtNo.lColumn = iNext.get_Column();
			vtCellNo.push_back(vtNo);
			iNext = range.FindNext(_variant_t(iNext));
		} while (iNext.get_Row() != iFirst.get_Row() || iNext.get_Column() != iFirst.get_Column());
	}
}

OpenExcel::OpenExcel()
{
}

OpenExcel::~OpenExcel()
{
	Close();
}

CString OpenExcel::GetExcelDriver(CString sDriverName)
{
	CString sDriver;
	TCHAR szDrivers[4096];
	memset(szDrivers, 0, sizeof(szDrivers));
	WORD wRet = 0;
	// 获取已安装驱动的名称(函数在odbcinst.h里)
	if (SQLGetInstalledDrivers(szDrivers, _countof(szDrivers), &wRet))
	{
		LPTSTR pszDrv = szDrivers;
		// 检索已安装的驱动是否有Excel
		while (*pszDrv)
		{
			CString str = CString(pszDrv);
			CStringA StrA = static_cast<CStringA>(str);
			char* str2 = StrA.GetBuffer();
			if (strstr(str2, sDriverName) != 0)
			{
				sDriver = CString(str2);
				return sDriver;
			}
			pszDrv += _tcslen(pszDrv) + 1;
		}
	}
	return _T("");
}

bool OpenExcel::Init(XiBase::CLog* pLog)
{
	m_pLog = pLog;
	// 打开文件
	m_sDriver = GetExcelDriver("Microsoft Excel Driver (*.xls)"); // Excel安装xls驱动
	if (m_sDriver.IsEmpty())
	{
		m_sDriver = GetExcelDriver("Microsoft Excel Driver (*.xls, *.xlsx, *.xlsm, *.xlsb)"); // Excel安装xlsx驱动
		if (m_sDriver.IsEmpty())
		{
			SetErrorString("未安装Excel驱动程序！");
			return false;
		}
	}
	return true;
}

bool OpenExcel::Open(CString sExcelFile)
{
	XiBase::GetSystemPath(sExcelFile);
	m_sExcelFile = sExcelFile;
	try
	{
		m_pDatabase = new CDatabase();
		CString sSql;
		sSql.Format(CString("DRIVER={%s};DSN='';FIRSTROWHASNAMES=1;READONLY=FALSE;CREATE_DB=\"%s\";DBQ=%s"), m_sDriver, sExcelFile, sExcelFile);
		// 创建数据库 (既Excel表格文件)
		m_pDatabase->OpenEx(sSql, CDatabase::noOdbcDialog);
	}
	catch(CDBException *e)
	{
		//数据库操作产生异常时...
		SetErrorString(_T("Excel错误：") + e->m_strError);
		delete m_pDatabase;
		m_pDatabase = NULL;
		return false;
	}
	catch (...)
	{
		SetErrorString(_T("Excel打开失败！"));
		delete m_pDatabase;
		m_pDatabase = NULL;
		return false;
	}
	return true;
}

bool OpenExcel::Restart()
{
	Close();
	return Open(m_sExcelFile);
}

bool OpenExcel::Close()
{
	if (NULL == m_pDatabase)
	{
		SetErrorString(_T("Excel未打开！"));
		return false;
	}
	m_pDatabase->Close();
	delete m_pDatabase;
	m_pDatabase = NULL;
	return true;
}

bool OpenExcel::Read(CString sSheet, std::vector<std::vector<CString>>& vvsValue)
{
	vvsValue.clear();
	if (NULL == m_pDatabase)
	{
		SetErrorString(_T("Excel未打开！"));
		return false;
	}
	try
	{
		//准备阶段2
		/*
		要实现对结果集的数据操作，就要用到CRecordSet类。
		CRecordSet类定义了从数据库接收或者发送数据到数据库的成员变量，CRecordSet类定义的记录集可以是表的所有列，也可以是其中的一列，这是由SQL语句决定的。
		CRecordSet类的成员变量m_hstmt代表了定义该记录集的SQL语句句柄，m_nFields成员变量保存了记录集中字段的个数，m_nParams成员变量保存了记录集所使用的参数个数
		*/
		CRecordset recset(m_pDatabase);
		//设置读取的查询语句
		CString sql = GetSelectString(sSheet);
		if (!recset.Open(CRecordset::forwardOnly, sql, CRecordset::readOnly))//执行查询语句readOnly
		{
			SetErrorString(_T("查询失败"));//测试
			return false;
		}
		short nFields = recset.GetODBCFieldCount();//得到总共多少列

		//准备阶段3
		CString str;
		while (!recset.IsEOF())
		{
			std::vector<CString> vsValue;
			vsValue.clear();
			bool bIsEmpty = true;
			for (size_t i = 0; i < nFields; i++)
			{
				recset.GetFieldValue(short(i), str);
				vsValue.push_back(str);
				if (!str.IsEmpty())
				{
					bIsEmpty = false;
				}
			}
			if (!bIsEmpty)
			{
			vvsValue.push_back(vsValue);
			}
			recset.MoveNext();
		}
		recset.Close();
		return true;
	}
	catch (CDBException* e)
	{
		// 数据库操作产生异常时...
		SetErrorString(_T("Excel读取错误：") + e->m_strError);
	}
	catch (...)
	{
		SetErrorString(_T("Excel读取错误！"));
	}
	return false;
}

bool OpenExcel::Read(CString sSheet, std::map<CString, std::vector<CString>>& vvsValue)
{
	vvsValue.clear();
	if (NULL == m_pDatabase)
	{
		SetErrorString(_T("Excel未打开！"));
		return false;
	}
	try
	{
		//准备阶段2
		/*
		要实现对结果集的数据操作，就要用到CRecordSet类。
		CRecordSet类定义了从数据库接收或者发送数据到数据库的成员变量，CRecordSet类定义的记录集可以是表的所有列，也可以是其中的一列，这是由SQL语句决定的。
		CRecordSet类的成员变量m_hstmt代表了定义该记录集的SQL语句句柄，m_nFields成员变量保存了记录集中字段的个数，m_nParams成员变量保存了记录集所使用的参数个数
		*/
		CRecordset recset(m_pDatabase);
		//设置读取的查询语句
		CString sql = GetSelectString(sSheet);
		if (!recset.Open(CRecordset::forwardOnly, sql, CRecordset::readOnly))//执行查询语句readOnly
		{
			SetErrorString(_T("查询失败"));//测试
			return false;
		}
		short nFields = recset.GetODBCFieldCount();//得到总共多少列

		//准备阶段3
		CString str;
		while (!recset.IsEOF())
		{
			std::vector<CString> vsValue;
			vsValue.clear();
			bool bIsEmpty = true;
			for (size_t i = 0; i < nFields; i++)
			{
				recset.GetFieldValue(short(i), str);
				vsValue.push_back(str);
				if (!str.IsEmpty())
				{
					bIsEmpty = false;
				}
			}
			if (!bIsEmpty)
			{
				vvsValue[vsValue[0]] = vsValue;
			}
			recset.MoveNext();
		}
		recset.Close();
		return true;
	}
	catch (CDBException* e)
	{
		// 数据库操作产生异常时...
		SetErrorString(_T("Excel读取错误：") + e->m_strError);
	}
	catch (...)
	{
		SetErrorString(_T("Excel读取错误！"));
	}
	return false;
}

bool OpenExcel::Read(CString sSheet, std::vector<std::vector<double>>& vvdValue)
{
	vvdValue.clear();
	std::vector<std::vector<CString>> vvsValue;
	if (!Read(sSheet, vvsValue))
	{
		return false;
	}

	for (size_t i = 0; i < vvsValue.size(); i++)
	{
		std::vector<double> vdValue;
		vdValue.clear();
		for (size_t j = 0; j < vvsValue[i].size(); j++)
		{
			vdValue.push_back(XiBase::CStringTodouble(vvsValue[i][j]));
		}
		vvdValue.push_back(vdValue);
	}
	return true;
}

bool OpenExcel::Write(CString sSheet, const std::vector<CString>& vsTitle, const std::vector<std::vector<CString>>& vvsValue)
{
	size_t nDataSize = vsTitle.size();
	if (nDataSize < 1)
	{
		SetErrorString(_T("标题数目不足1"));
		return false;
	}
	for (size_t i = 0; i < vvsValue.size(); i++)
	{
		if (vvsValue[i].size() != nDataSize)
		{
			SetErrorString(_T("数据数目和标题数目不一致！"));
			return false;
		}
	}
	
	CString sTitle = GetCreateString(sSheet, vsTitle, !Drop(sSheet, vsTitle));
	try
	{
		m_pDatabase->ExecuteSQL(sTitle);
		for (size_t i = 0; i < vvsValue.size(); i++)
		{
			CString sValue = GetInsertString(sSheet, vsTitle, i + 1);
			for (size_t j = 0; j < vvsValue[i].size(); j++)
			{
				sValue = sValue + CString("'") + vvsValue[i][j] + CString("',");
	}
			sValue = sValue.Left(sValue.GetLength() - 1);
			sValue += ")";
			m_pDatabase->ExecuteSQL(sValue);
		}
		return true;
	}
	catch (CDBException* e)
	{
		// 数据库操作产生异常时...
		SetErrorString(_T("Excel读取错误：") + e->m_strError);
	}
	catch (...)
	{
		SetErrorString(_T("Excel读取错误！"));
	}
	return false;
}

bool OpenExcel::Write(CString sSheet, const std::vector<CString>& vsTitle, const std::vector<std::vector<double>>& vvdValue)
{
	std::vector<std::vector<CString>> vvsValue;
	for (size_t i = 0; i < vvdValue.size(); i++)
	{
		std::vector<CString> vsValue;
		vsValue.clear();
		for (size_t j = 0; j < vvdValue[i].size(); j++)
		{
			CString sTemp;
			sTemp.Format("%.6lf", vvdValue[i][j]);
			vsValue.push_back(sTemp);
		}
		vvsValue.push_back(vsValue);
	}

	if (!Write(sSheet, vsTitle, vvsValue))
	{
		return false;
	}
	return true;
}

bool OpenExcel::Drop(CString sSheet, const std::vector<CString>& vsTitle)
{
	try
	{
		m_pDatabase->ExecuteSQL(GetDropString(sSheet));
		return true;
	}
	catch (CDBException* e)
	{
		// 数据库操作产生异常时...
		SetErrorString(_T("Excel读取错误：") + e->m_strError);
	}
	catch (...)
	{
		SetErrorString(_T("Excel读取错误！"));
	}
	return false;
}

bool OpenExcel::Add(CString sSheet, const std::vector<CString>& vsTitle, const std::vector<CString>& vsValue)
{
	if (NULL == m_pDatabase)
	{
		SetErrorString(_T("Excel未打开！"));
		return false;
	}

	short nFields = 0;
	try
	{
		CRecordset recset(m_pDatabase);
		//设置读取的查询语句
		CString sql = GetSelectString(sSheet);
		if (!recset.Open(CRecordset::forwardOnly, sql, CRecordset::readOnly))//执行查询语句
		{
			SetErrorString(_T("查询失败"));//测试
			return false;
		}
		nFields = recset.GetODBCFieldCount();//得到总共多少列
		recset.Close();
	}
	catch (...)
	{
		std::vector<std::vector<CString>> vvsValue;
		vvsValue.push_back(vsValue);
		return Write(sSheet, vsTitle, vvsValue);
	}

	size_t nDataSize = vsTitle.size();
	if (nDataSize < 1)
	{
		SetErrorString(_T("标题数目不足1"));
		return false;
	}
	if (nDataSize != (size_t)nFields)
	{
		SetErrorString(_T("标题数目不符合原表格"));
		return false;
	}
	if (vsValue.size() != nDataSize)
	{
		SetErrorString(_T("数据数目和标题数目不一致！"));
		return false;
	}

	CString sInsert = GetInsertString(sSheet, vsTitle);
	try
		{
			CString sValue = sInsert;
		for (size_t j = 0; j < vsValue.size(); j++)
			{
			sValue = sValue + CString("'") + vsValue[j] + CString("',");
			}
			sValue = sValue.Left(sValue.GetLength() - 1);
			sValue += ")";
			m_pDatabase->ExecuteSQL(sValue);
		return true;
	}
	catch (CDBException* e)
	{
		// 数据库操作产生异常时...
		SetErrorString(_T("Excel读取错误：") + e->m_strError);
		}
	catch (...)
	{
		SetErrorString(_T("Excel读取错误！"));
	}
	return false;
}

bool OpenExcel::Update(CString sSheet, const std::vector<CString>& vsTitle, const std::vector<CString>& vsValue, int nRow)
{
	CString sInsert = GetUpdateString(sSheet, vsTitle, nRow);
	try
	{
		//CString sValue = sInsert;
		//for (size_t j = 0; j < vsValue.size(); j++)
		//{
		//	sValue = sValue + CString("'") + vsValue[j] + CString("',");
		//}
		//sValue = sValue.Left(sValue.GetLength() - 1);
		//sValue += ")";
		m_pDatabase->ExecuteSQL(sInsert);
		return true;
	}
	catch (CDBException* e)
	{
		// 数据库操作产生异常时...
		SetErrorString(_T("Excel读取错误：") + e->m_strError);
	}
	catch (...)
	{
		SetErrorString(_T("Excel读取错误！"));
	}
	return false;
}

CString OpenExcel::GetCreateString(CString sSheet, const std::vector<CString>& vsTitle, bool bNew)
{
	CString sTitle = _T("CREATE TABLE [" + sSheet + "] (");
	if (!bNew)
	{
		sTitle = _T("CREATE TABLE [" + sSheet + "$A1:IV65536] (");
	}
	for (size_t i = 0; i < vsTitle.size(); i++)
		{
		sTitle = sTitle + CString("[") + vsTitle[i] + CString("]  TEXT, ");
		}
	sTitle = sTitle.Left(sTitle.GetLength() - 2);
	sTitle += ")";
	return sTitle;
}

CString OpenExcel::GetDropString(CString sSheet)
{
	return "DROP TABLE [" + sSheet + "$A1:IV65536]";
}

CString OpenExcel::GetSelectString(CString sSheet)
{
	return _T("SELECT * FROM [" + sSheet + "$]");
}

CString OpenExcel::GetInsertString(CString sSheet, const std::vector<CString>& vsTitle, int nRow)
{
	CString sInsert;
	if (nRow > 0)
	{
		sInsert.Format(_T("INSERT INTO [%s$A1:IV%d] ("), sSheet, nRow);
	}
	else
	{
		sInsert = "INSERT INTO [" + sSheet + "$] (";
	}
	for (size_t i = 0; i < vsTitle.size(); i++)
	{
		sInsert = sInsert + CString("[") + vsTitle[i] + CString("] ,");
	}
	sInsert = sInsert.Left(sInsert.GetLength() - 1);
	sInsert += ") values (";
	return sInsert;
}

CString OpenExcel::GetUpdateString(CString sSheet, const std::vector<CString>& vsTitle, int nRow)
{
	CString sInsert;
	sInsert.Format(_T("UPDATE [%s$A%d:IV%d] SET "), sSheet, nRow - 1, nRow);
	for (size_t i = 0; i < vsTitle.size(); i++)
	{
		sInsert = sInsert + CString("[") + vsTitle[i] + CString("]='eee',");
	}
	sInsert = sInsert.Left(sInsert.GetLength() - 1);
	return sInsert;
}

void OpenExcel::SetErrorString(CString sError)
{
	m_pLog->Write(sError);
	m_sErrorInfo = sError;
}

CString OpenExcel::GetErrorString()
{
	return m_sErrorInfo;
}

void OpenExcel::ShowErrorString()
{
	if (!m_sErrorInfo.IsEmpty())
	{
		XiMessageBox(m_pLog, m_sErrorInfo);
	}
}