#pragma once

#include <VECTOR>
#include <map>
#include <iostream>
#include <fstream>
#include <atlconv.h>

#define MAX_ALL_SECTION_SIZE 1024 * 1024
#define MAX_ALL_KEY_SIZE 1024 * 1024
#define MAX_VALUE_SIZE 1024 * 1024

//段名、关键词、值
#define INI_DATA std::map<CString, std::map<CString, CString>>

//常用字符串、路径、文件、文件夹操作
namespace XiBaseDll
{
	/************************************************************************/
	/*                                字符串                                */
	/************************************************************************/
	//字符串分割，sToken是多个字符时，每个字符都是分隔符
	std::vector<CString> TokenizeCString(CString sSrc, CString sToken);

	//Utf8转GBK
	CString Utf8ToGBK(const CString &sUtf8);

	//Utf8转GBK
	CString GBKToUtf8(const CString &sGBK);

	//保留字符串内指定的字符
	CString ClearOtherCharacter(CString sSrc, CString sCharacter);

	//保留字符串内的大小写字母、数字和下划线
	CString OnlyLettersNumbersUnderscores(CString sSrc);

	//判断字符串是否是布尔数据
	bool CheckCStringType_bool(CString sData);

	//判断字符串是否是整型数据
	bool CheckCStringType_long(CString sData);

	//判断字符串是否是浮点数据
	bool CheckCStringType_double(CString sData);

	//判断字符串是否是无符号整型数据
	bool CheckCStringType_unsignedlong(CString sData);

	//传入16进制数字符串，返回unsigned long long类型数
	bool CStringToULongLong(unsigned long long &ullNum, CString strNum);

	bool ULongLongToCString(CString &strNum, unsigned long long ullNum);

	/************************************************************************/
	/*                                 路径                                 */
	/************************************************************************/
	//获取当前进程的路径
	CString GetModuleFilePath();

	//从完整路径中获取文件名
	bool GetFileNameFromPath(CString sFilePath, CString &sFileName, bool bSuffix);

	//从完整路径中获取路径
	bool GetFilePathFromPath(CString sFilePath, CString &sFileName);

	//从完整路径中去掉后缀名
	bool GetFilePathFromPathWithoutSuffix(CString sFilePath, CString &sFileName);

	//选择文件界面
	CString OpenFileDlg(CDialog *pFatherDlg, CString sFileType, CString sDefaultPath);
	bool OpenFileDlg(CDialog *pFatherDlg, CString sFileType, CString sDefaultPath, std::vector< CString> &vstrFileList);

	//选择文件夹
	bool OpenFolder(CDialog *pFatherDlg, CString sDefaultPath, CString sTitle, CString &sPath);

	/************************************************************************/
	/*                               配置文件                               */
	/************************************************************************/
	//加载全部ini参数
	INI_DATA LoadIni(CString sFileName);

	//读取一个参数
	bool ReadIni(const CString &sFileName, const CString &sSection, const CString &sKey, CString &sValue);

	//写入一个参数
	bool WriteIni(const CString &sFileName, const CString &sSection, const CString &sKey, const CString &sValue);

	//删除一个参数
	bool DelIniKey(const CString &sFileName, const CString &sSection, const CString &sKey);

	//删除一个段
	bool DelIniSection(const CString &sFileName, const CString &sSection);

	/************************************************************************/
	/*                               通用文件                               */
	/************************************************************************/
	//检查文件是否存在（若选择创建，文件夹不存在也会创建）
	bool CheckFileExists(CString sFileName, bool bCreate = false);

	//在指定路径下寻找指定扩展名的全部文件
	bool FindFile(std::vector<CString> &vstrFileList, CString strPath, CString strFileType, bool bIncludeSubfolder = true, bool bReturnPathOrName = true);
	bool FindFile(std::vector<CString> &vstrFileList, CString strPath, std::vector<CString> vstrFileType, bool bIncludeSubfolder = true, bool bReturnPathOrName = true);
	
	//安全的文件读取,默认使用fopen_s，并有报错功能，nShFlag设为_SH_DENYNO相当于fopen
	errno_t XI_fopen_s(FILE **file, CString strFileName, _In_z_ const char * _Mode, int nShFlag = _SH_SECURE);

	/************************************************************************/
	/*                                文件夹                                */
	/************************************************************************/
	//检查文件夹是否存在，不存在则创建一个
	long CheckFolder(CString strPath, bool bCreate = true);//1:文件夹存在，0:文件夹不存在但已创建/不需要创建，其它(负数):报错

	//复制整个文件夹到指定路径下
	long CopyFolder(CString strSrcFolderName, CString strDstPath, bool bCopySubfolder = true);

	//清理文件夹中的文件
	void DelFiles(CString directory_path);
}

static int CALLBACK BrowseCallbackProc(HWND hwnd, UINT uMsg, LPARAM lParam, LPARAM lpData);

