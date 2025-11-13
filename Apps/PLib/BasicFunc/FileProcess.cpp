#include "stdafx.h"
#include "FileProcess.h"

//字符串分割，sToken是多个字符时，每个字符都是分隔符
//std::vector<CString> TokenizeCString(CString sSrc, CString sToken)
//{
//	std::vector<CString> vsRtnVal;
//	vsRtnVal.clear();
//	CString sTemp;
//	sTemp.Empty();
//	int nPos = 0;
//	while (true)
//	{
//		sTemp = sSrc.Tokenize(sToken, nPos);
//		if (!sTemp.IsEmpty())
//		{
//			vsRtnVal.push_back(sTemp);
//		}
//		else
//		{
//			break;
//		}
//	}
//	return vsRtnVal;
//}

std::vector<CString> GetTotalFilePath(CString strFileName, CString strSuffix, bool bPath)
{
	std::vector<CString> vstrFileName;
	vstrFileName.clear();
	int nFilePos = strFileName.ReverseFind('\\');
	int nSuffixPos = strFileName.ReverseFind('.');
	if (nFilePos < nSuffixPos)
	{
		return vstrFileName;
	}
	if (strFileName.Right(1) != "\\")
	{
		strFileName += "\\";
	}
	CFileFind finder;
	BOOL bWorking = finder.FindFile(strFileName + strSuffix);
	while (bWorking)
	{
		bWorking = finder.FindNextFile();
		CString strName = finder.GetFileName();
		if (strName.Left(1) != ".")
		{
			strName = strFileName + strName;
			if (bPath)
			{
				nFilePos = strName.ReverseFind('\\');
				nSuffixPos = strName.ReverseFind('.');
				if (nFilePos > nSuffixPos)
				{
					strName += "\\";
				}
			}
			vstrFileName.push_back(strName);
		}
	}
	return vstrFileName;
}

std::vector<CString> GetTotalFilePath(std::vector<CString> strFileName, CString strSuffix)
{
	std::vector<CString> vstrFileName;
	vstrFileName.clear();
	vstrFileName.insert(vstrFileName.end(), strFileName.begin(), strFileName.end());

	for (int n = 0; n < strFileName.size(); n++)
	{
		std::vector<CString> vstrTemp = GetTotalFilePath(strFileName[n], strSuffix);
		std::vector<CString> vstrTemp_ = GetTotalFilePath(vstrTemp, strSuffix);
		vstrFileName.insert(vstrFileName.end(), vstrTemp_.begin(), vstrTemp_.end());

	}
	vstrFileName = DeleteTotalFilePathFolder(vstrFileName);
	return vstrFileName;
}

std::vector<CString> DeleteTotalFilePathFolder(std::vector<CString> vstrFileName)
{
	for (int n = 0; n < vstrFileName.size(); n++)
	{
		int nFilePos = vstrFileName[n].ReverseFind('\\');
		int nSuffixPos = vstrFileName[n].ReverseFind('.');
		if (nFilePos > nSuffixPos)
		{
			vstrFileName.erase(vstrFileName.begin() + n, vstrFileName.begin() + n + 1);
			n--;
		}
	}
	return vstrFileName;
}

std::vector<CString> ScreenTotalFilePathSuffix(std::vector<CString> vstrFileName, std::vector<CString> vstrSuffix)
{
	std::vector<CString> vstrResult;
	for (int n = 0; n < vstrFileName.size(); n++)
	{
		for (int m = 0; m < vstrSuffix.size(); m++)
		{
			int nSuffixLenght = vstrSuffix[m].GetLength();
			if (vstrFileName[n].Right(nSuffixLenght) == vstrSuffix[m])
			{
				vstrResult.push_back(vstrFileName[n]);
				break;
			}
		}
	}
	return vstrResult;
}

std::vector<CString> ScreenTotalFilePathSuffix(std::vector<CString> vstrFileName, CString strSuffix)
{
	std::vector<CString> vstrResult;
	for (int n = 0; n < vstrFileName.size(); n++)
	{
		int nSuffixLenght = strSuffix.GetLength();
		if (vstrFileName[n].Right(nSuffixLenght) == strSuffix)
		{
			vstrResult.push_back(vstrFileName[n]);
		}
	}
	return vstrResult;
}

CString DeleteContinue(CString str, char cContinue)
{
	int nLenght = str.GetLength();
	char *acData;
	acData = new char[nLenght + 1];
	int nOffset = 0;
	std::vector<int> vnNo;
	for (int n = 0; n < nLenght; n++)
	{
		char temp = str.GetAt(n);
		if (temp == cContinue)
		{
			continue;
		}
		vnNo.push_back(n);
	}
	int nNo = 0;
	for (int m = 0; m < vnNo.size(); m++)
	{
		acData[nNo++] = str.GetAt(vnNo[m]);
		if (m + 1 < vnNo.size())
		{
			if (vnNo[m + 1] - vnNo[m] == 2)
			{
				acData[nNo++] = str.GetAt(vnNo[m] + 1);
			}
		}
	}
	acData[nNo] = '\0';
	str.Format("%s", acData);
	delete[]acData;
	return str;
}

CString DeletePrefix(CString str, CString strPrefix)
{
	int nPreLenght = strPrefix.GetLength();
	if (nPreLenght >= str.GetLength())
	{
		return str;
	}
	int n;
	for (n = 0; n < str.GetLength() - nPreLenght; n++)
	{
		if (str.Mid(n, nPreLenght) == strPrefix)
		{
			str = str.Right(str.GetLength() - n);
			break;
		}
	}
	return str;
}

CString DeleteSuffix(CString str)
{
	int nPos;
	nPos = str.ReverseFind('.');
	if (nPos == -1)
	{
		return str;
	}
	str = str.Left(nPos);
	return str;
}

CString DeletePath(CString str)
{
	int nPos;
	nPos = str.ReverseFind('\\');
	if (nPos != -1)
	{
		int nLenght = str.GetLength();
		str = str.Right(nLenght - nPos - 1);
	}
	return str;
}

CString GetPath(CString str)
{
	int nPos;
	nPos = str.ReverseFind('\\');
	if (nPos != -1)
	{
		str = str.Left(nPos);
	}
	return str;
}

CString DeleteDrackets(CString str)
{
	CString strTemp = "（）";
	char cStart = strTemp.GetAt(0);
	char cEnd = strTemp.GetAt(3);
	int nLenght = str.GetLength();
	char *acData;
	acData = new char[nLenght + 1];
	int nOffset = 0;
	int nNum = 0;
	for (int n = 0; n < nLenght; n++)
	{
		char temp = str.GetAt(n);
		if (temp == '(' || temp == cStart)
		{
			nNum++;
			nOffset++;
			continue;
		}
		if (nNum > 0)
		{
			if (temp == ')' || temp == cEnd)
			{
				nNum--;
			}
			nOffset++;
			continue;
		}
		acData[n - nOffset] = temp;
	}
	acData[nLenght - nOffset] = '\0';
	str.Format("%s", acData);
	delete[]acData;
	return str;
}

CString DeleteChinese(CString str, bool bDeleteAfter)
{
	int nLenght = str.GetLength();
	char *acData;
	acData = new char[nLenght + 1];
	int nDataLenght = 0;
	for (int n = 0; n < nLenght; n++)
	{
		char temp = str.GetAt(n);
		if (temp <= 0x7f && temp > 0)
		{
			acData[nDataLenght] = temp;
			nDataLenght++;
		}
		else
		{
			if (bDeleteAfter)
			{
				break;
			}
		}
	}
	acData[nDataLenght] = '\0';
	str.Format("%s", acData);
	delete[]acData;
	return str;
}

CString Replace(CString str, char cOld, char cNew)
{
	int nLenght = str.GetLength();
	char *acData;
	acData = new char[nLenght + 1];
	int nOffset = 0;
	for (int n = 0; n < nLenght; n++)
	{
		char temp = str.GetAt(n);
		if (temp <= 0x7f && temp > 0)
		{
			if (temp == cOld)
			{
				if (cNew == NULL)
				{
					nOffset++;
					continue;
				}
				temp = cNew;
			}
		}
		acData[n - nOffset] = temp;
	}
	acData[nLenght - nOffset] = '\0';
	str.Format("%s", acData);
	delete[]acData;
	return str;
}

int StringSimilarity(CString str1, CString str2, int nMode /*= 1*/)
{
	int nLenght1 = str1.GetLength();
	int nLenght2 = str2.GetLength();
	int nLenght = nLenght1 < nLenght2 ? nLenght1 : nLenght2;
	int nSimilarity = 0;
	for (int n = 0; n < nLenght; n++)
	{
		if (1 == nMode)
		{
			if (str1.GetAt(n) == str2.GetAt(n))
			{
				nSimilarity++;
			}
		}
		else if (2 == nMode)
		{
			if (str1.GetAt(nLenght1 - n - 1) == str2.GetAt(nLenght2 - n - 1))
			{
				nSimilarity++;
			}
		}
	}
	return nSimilarity;
}

T_INI_FILE_INFO GetIniFileInfo(CString strFilePath)
{
	T_INI_FILE_INFO tInfo; 
	T_INI_SECTION tIniSection;
	tInfo.strFileName = strFilePath;
	std::vector<std::vector<CString>> vvstrFileData;
	std::vector<CString> vstrFileData;
	CStdioFile mFile;
	if (mFile.Open(strFilePath, CFile::modeReadWrite))
	{
		if (mFile.GetLength() == 0)
		{
			mFile.Close();
			return tInfo;
		}
		CString strData = "";
		while (mFile.ReadString(strData))
		{
			strData = Replace(strData, ' ', NULL);
			if (strData == "")
			{
				continue;
			}
			if (strData.Left(1) == "#")
			{
				continue;
			}
			if (strData.Left(1) == "[" && strData.Right(1) == "]")
			{
				if (vstrFileData.size() > 0)
				{
					tIniSection = GetSectionVal(vstrFileData);
					tInfo.vtSection.push_back(tIniSection);
					vvstrFileData.push_back(vstrFileData);
					vstrFileData.clear();
				}
			}
			vstrFileData.push_back(strData);
		}
		tIniSection = GetSectionVal(vstrFileData);
		tInfo.vtSection.push_back(tIniSection);
		vvstrFileData.push_back(vstrFileData);
		mFile.Close();
	}
	return tInfo;
}

T_INI_KEY GetKeyVal(CString strData)
{
	T_INI_KEY tIniKey;
	int nLenght = strData.GetLength();
	int nMarkPos = strData.Find('=');
	if (nLenght == nMarkPos || -1 == nMarkPos)
	{
		tIniKey.strKeyName = strData;
		tIniKey.strKeyVal = "";
		return tIniKey;
	}
	tIniKey.strKeyName = strData.Left(nMarkPos);
	tIniKey.strKeyVal = strData.Right(nLenght - nMarkPos - 1);
	return tIniKey;
}

T_INI_SECTION GetSectionVal(std::vector<CString> vtrData)
{
	T_INI_SECTION tIniSection;
	if (vtrData[0].Left(1) != "[" && vtrData[0].Right(1) == "]")
	{
		return tIniSection;
	}
	vtrData[0] = Replace(vtrData[0], '[', NULL);
	vtrData[0] = Replace(vtrData[0], ']', NULL);
	tIniSection.strSectionName = vtrData[0];
	for (int n = 1; n < vtrData.size(); n++)
	{
		tIniSection.vtKey.push_back(GetKeyVal(vtrData[n]));
	}
	return tIniSection;
}

T_INI_FILE_INFO ReplaceIniFile(T_INI_FILE_INFO tNewFile, T_INI_FILE_INFO tOldFile)
{
	if (DeletePath(tNewFile.strFileName) != DeletePath(tOldFile.strFileName))
	{
		return tNewFile;
	}
	tNewFile.strFileName = tOldFile.strFileName;
	for (int n = 0; n < tNewFile.vtSection.size(); n++)
	{
		for (int m = 0; m < tOldFile.vtSection.size(); m++)
		{
			if (tNewFile.vtSection[n].strSectionName == tOldFile.vtSection[m].strSectionName)
			{
				for (int i = 0; i < tNewFile.vtSection[n].vtKey.size(); i++)
				{
					for (int j = 0; j < tOldFile.vtSection[m].vtKey.size(); j++)
					{
						if (tNewFile.vtSection[n].vtKey[i].strKeyName == tOldFile.vtSection[m].vtKey[j].strKeyName && tOldFile.vtSection[m].vtKey[j].strKeyVal != "")
						{
							tNewFile.vtSection[n].vtKey[i].strKeyVal = tOldFile.vtSection[m].vtKey[j].strKeyVal;
						}
					}
				}
			}
		}
	}
	return tNewFile;
}

void SaveIniFile(T_INI_FILE_INFO tFile)
{
	COPini cOpini;
	cOpini.SetFileName(false, tFile.strFileName);
	for (int n = 0; n < tFile.vtSection.size(); n++)
	{
		cOpini.SetSectionName(tFile.vtSection[n].strSectionName);
		for (int m = 0; m < tFile.vtSection[n].vtKey.size(); m++)
		{
			cOpini.WriteString(tFile.vtSection[n].vtKey[m].strKeyName, tFile.vtSection[n].vtKey[m].strKeyVal);
		}
	}
}

