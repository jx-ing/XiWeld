#include "stdafx.h"
#include "PartialPointCloudProcess.h"

#include "Apps/SmallPiece/Infrastructure/zlib/XiZip.h"

namespace image_process
{
PartialPointCloudProcess::PartialPointCloudProcess()
{
}

PartialPointCloudProcess::~PartialPointCloudProcess()
{
}

void PartialPointCloudProcess::init(CString sDataFolder)
{
	XiBase::AddSlash(sDataFolder);
	CheckFolder(sDataFolder);
	m_sDataFolder = sDataFolder;
	m_nErrorThreshold = PARA_FLAT_TRACK(nMaxProcessPointCloudErrorTimes);
	m_nErrorCount = 0;// 错误次数清零
}

bool PartialPointCloudProcess::process(int nImageNoS, int nImageNoE, std::vector<CvPoint3D64f>& vtPointCloud)
{
	// 初始化数据
	m_vtResult.clear();
	double dProcessSampDis = 2.0;// 点云提取结果中点间距

	// 获取时间
	CTime cTime;
	cTime = CTime::GetCurrentTime();

	// 生成文件夹
	CString sFileFolder;
	sFileFolder.Format(m_sDataFolder + "%04d-%02d-%02d_%02d-%02d-%02d_S%d-E%d\\",
		cTime.GetYear(), cTime.GetMonth(), cTime.GetDay(),
		cTime.GetHour(), cTime.GetMinute(), cTime.GetSecond(),
		nImageNoS, nImageNoE);
	CheckFolder(sFileFolder);

	// 保存输入
	CHECK_FALSE_RETURN(saveInputPara(nImageNoS, nImageNoE, sFileFolder));
	CHECK_FALSE_RETURN(savePointCloud(vtPointCloud, sFileFolder));

	// 开始处理
	RiserEndPointInfo tWeldInfo[3] = { 0 };
	try
	{
		int nWeldInfoNum = GetRiserEndPoint(vtPointCloud.data(), vtPointCloud.size(),
			tWeldInfo, m_tCameraNorm, m_tPlaneHNorm, m_tRefLine, 2,
			(char*)(const char*)m_sConfigFile);

		double dDis = TwoPointDis3D(tWeldInfo[0].staPnt, tWeldInfo[0].endPnt);
		double dDisX = tWeldInfo[0].endPnt.x - tWeldInfo[0].staPnt.x;
		double dDisY = tWeldInfo[0].endPnt.y - tWeldInfo[0].staPnt.y;
		double dDisZ = tWeldInfo[0].endPnt.z - tWeldInfo[0].staPnt.z;

		// 处理失败
		if ((0 >= nWeldInfoNum || dDis < dProcessSampDis)// 处理无焊缝结果 || 焊缝起点终点相近
			&& (m_nErrorCount <= m_nErrorThreshold))// 连续错误数量不超过阈值
		{
			m_nErrorCount++;
			// 连续失败多次
			if (m_nErrorCount > m_nErrorThreshold)
			{
				return false;
			}

			return true;
		}

		// 处理成功
		m_nErrorCount = 0;
		int nNo = (int)(dDis / dProcessSampDis);
		double dStepDis = dDis / (double)nNo;
		m_vtResult.resize(nNo + 1);
		for (int i = 0; i <= nNo; i++)
		{
			m_vtResult[i].x = tWeldInfo[0].staPnt.x + dDisX * i / nNo;
			m_vtResult[i].y = tWeldInfo[0].staPnt.y + dDisY * i / nNo;
			m_vtResult[i].z = tWeldInfo[0].staPnt.z + dDisZ * i / nNo;
		}
		CHECK_FALSE_RETURN(saveResult(sFileFolder));
		return true;
	}
	catch (...)
	{
		return false;//异常直接算失败
	}
	return false;
}

void PartialPointCloudProcess::saveErrorData(bool bAutoSave, CString sName)
{
	// 找出所有文件夹
	std::vector<CString> vsFileList(0);
	CFileFind finder;
	CString sDir = m_sDataFolder + _T("*.*");
	BOOL bWorking = finder.FindFile(sDir);
	while (bWorking)
	{
		bWorking = finder.FindNextFile();
		if (finder.IsDirectory() && !finder.IsDots())
		{
			vsFileList.push_back(finder.GetFileName());
		}
	}
	if (vsFileList.size() == 0)
		return;

	// 排序
	std::sort(vsFileList.begin(), vsFileList.end(),
		[](const CString& a, const CString& b)
		{
			return a.Compare(b) < 0;
		});

	// 保存路径
	CString sSavePath = m_sErrorDataDir + vsFileList[0] + sName + "\\";

	// 保存图像库信息
	XiBase::COPini opini;
	opini.SetFileName(sSavePath + "para.ini");
	opini.SetSectionName("Base");

	//获取文件上次修改日期
	WIN32_FIND_DATA FindFileData;
	HANDLE hFind = FindFirstFile("GFPGJointLib.dll", &FindFileData);
	if (hFind == INVALID_HANDLE_VALUE)
	{
		opini.WriteString("GFPGJointLibTime", "Unknown");
	}
	else
	{
		// 获取最后修改时间
		FILETIME ftWrite = FindFileData.ftLastWriteTime;
		SYSTEMTIME stUTC, stLocal;
		FileTimeToSystemTime(&ftWrite, &stUTC);
		SystemTimeToTzSpecificLocalTime(NULL, &stUTC, &stLocal);

		//保存日期
		CString sTime;
		sTime.Format("%04d-%02d-%02d_%02d:%02d:%02d",
			stLocal.wYear, stLocal.wMonth, stLocal.wDay,
			stLocal.wHour, stLocal.wMinute, stLocal.wSecond);
		opini.WriteString("GFPGJointLibTime", sTime);
	}

	// 自动保存/手动保存
	opini.WriteString("bAutoSave", bAutoSave);

	// 复制数据
	XiBase::CopyFolder(m_sDataFolder + vsFileList[0], sSavePath);

	// 复制配置文件
	vsFileList.clear();
	XiBase::FindFile(vsFileList, m_sConfigFolder, "ini", false, false);
	for (size_t i = 0; i < vsFileList.size(); i++)
	{
		if (vsFileList[i].Find("GetRiserEndPoint"))
		{
			CopyFile(m_sConfigFolder + vsFileList[i], sSavePath + vsFileList[i], false);
		}
	}

	//// 压缩文件夹
	//std::string srcPath = sSavePath;
	//std::string dstPath = sSavePath.Left(sSavePath.GetLength() - 1) + ".zip";
	//xi::zip::ZipDir(srcPath, dstPath);

	//// 删除源文件夹
	//DelFiles(sSavePath);
	//RemoveDirectory(sSavePath);
}

void PartialPointCloudProcess::openErrorDataFolder()
{
	ShellExecute(NULL, "open", m_sErrorDataDir, NULL, NULL, SW_SHOW);
}

bool PartialPointCloudProcess::saveInputPara(int nImageNoS, int nImageNoE, CString sFileFolder) const
{
	FILE* pfPointCloudInput = fopen(sFileFolder + "InputPara.txt", "w");
	if (pfPointCloudInput == NULL)
	{
		return false;
	}

	fprintf(pfPointCloudInput, m_sConfigFile + "\n"
		"nImageNoS: %d nImageNoE: %d \n"
		"tCameraNorm: %11.3lf %11.3lf %11.3lf \n"
		"tPlaneHNorm: %11.3lf %11.3lf %11.3lf \n"
		"tRefPtn[0]: %11.3lf %11.3lf %11.3lf \n"
		"tRefPtn[1]: %11.3lf %11.3lf %11.3lf \n",
		nImageNoS, nImageNoE,
		m_tCameraNorm.x, m_tCameraNorm.y, m_tCameraNorm.z,
		m_tPlaneHNorm.x, m_tPlaneHNorm.y, m_tPlaneHNorm.z,
		m_tRefLine[0].x, m_tRefLine[0].y, m_tRefLine[0].z,
		m_tRefLine[1].x, m_tRefLine[1].y, m_tRefLine[1].z);

	fclose(pfPointCloudInput);
	return true;
}

bool PartialPointCloudProcess::savePointCloud(const std::vector<CvPoint3D64f>& vtPointCloud, CString sFileFolder)
{
	FILE* pfPointCloud = fopen(sFileFolder + "PointCloud.txt", "w");
	if (pfPointCloud == NULL)
	{
		return false;
	}

	for (int i = 0; i < vtPointCloud.size(); i++)
	{
		fprintf(pfPointCloud, "%4d %11.3lf %11.3lf %11.3lf \n",
			i, vtPointCloud[i].x, vtPointCloud[i].y, vtPointCloud[i].z);
	}

	fclose(pfPointCloud);
	return true;
}

bool PartialPointCloudProcess::saveResult(CString sFileFolder)
{
	FILE* pfPointCloudResult = fopen(sFileFolder + "Result.txt", "w");
	if (pfPointCloudResult == NULL)
	{
		return false;
	}

	for (int i = 0; i < m_vtResult.size(); i++)
	{
		fprintf(pfPointCloudResult, "%4d %11.3lf %11.3lf %11.3lf \n",
			i, m_vtResult[i].x, m_vtResult[i].y, m_vtResult[i].z);
	}

	fclose(pfPointCloudResult);
	return true;
}

}