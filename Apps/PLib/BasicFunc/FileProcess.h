#pragma once
#include "afxwin.h"
#include <vector>
#include ".\OpenClass\FileOP\ini\opini.h"

typedef struct
{
	CString strKeyName;
	CString strKeyVal;
}T_INI_KEY;

typedef struct
{
	CString strSectionName;
	std::vector<T_INI_KEY> vtKey;
}T_INI_SECTION;

typedef struct
{
	CString strFileName;
	std::vector<T_INI_SECTION> vtSection;
}T_INI_FILE_INFO;

//std::vector<CString> TokenizeCString(CString sSrc, CString sToken);

std::vector<CString> GetTotalFilePath(CString strFileName, CString strSuffix = "*", bool bPath = true);
std::vector<CString> GetTotalFilePath(std::vector<CString> strFileName, CString strSuffix = "*");//递归算法

std::vector<CString> DeleteTotalFilePathFolder(std::vector<CString> vstrFileName);
std::vector<CString> ScreenTotalFilePathSuffix(std::vector<CString> vstrFileName, std::vector<CString> vstrSuffix);
std::vector<CString> ScreenTotalFilePathSuffix(std::vector<CString> vstrFileName, CString strSuffix);

T_INI_FILE_INFO GetIniFileInfo(CString strFilePath);
T_INI_KEY GetKeyVal(CString strData);
T_INI_SECTION GetSectionVal(std::vector<CString> vtrData);
T_INI_FILE_INFO ReplaceIniFile(T_INI_FILE_INFO tNewFile, T_INI_FILE_INFO tOldFile);
void SaveIniFile(T_INI_FILE_INFO tFile);

//CString操作
CString DeleteContinue(CString str, char cContinue);//删除连续字符
CString DeletePrefix(CString str, CString strPrefix);//删除strPrefix之前字符
CString DeleteSuffix(CString str);//删除后缀
CString DeletePath(CString str);//删除路径
CString GetPath(CString str);//得到路径
CString DeleteDrackets(CString str);//删除括号里内容
CString DeleteChinese(CString str, bool bDeleteAfter = false);//删除汉字
CString Replace(CString str, char cOld, char cNew);//替换，cNew=NULL为删除
int StringSimilarity(CString str1, CString str2, int nMode = 1);//nMode = 1从前往后，nMode = 2从后往前
