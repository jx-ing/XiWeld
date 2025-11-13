#include "stdafx.h"
#include "FlatWeldEndpointSearch.h"

#include "Apps/SmallPiece/Infrastructure/zlib/XiZip.h"

namespace image_process
{
FlatWeldEndpointSearch::FlatWeldEndpointSearch()
{
	m_sDataFolder = _T(".\\WeldData\\RobotA\\FlatWeldEndpointSearch\\");
	m_sErrorDataDir = _T(".\\ErrorData\\FlatWeldEndpointSearch\\");
	makesureDataFolderExist();
}

FlatWeldEndpointSearch::~FlatWeldEndpointSearch()
{
	closeFullPointCloudFile();
}

void FlatWeldEndpointSearch::makesureDataFolderExist()
{
	XiBase::CheckFolder(getSrcImageFolder(), true);
	XiBase::CheckFolder(getDstImageFolder(), true);
	XiBase::CheckFolder(getPointCloudFolder(), true);
	XiBase::CheckFolder(m_sErrorDataDir, true);
	CheckFileExists(getOutPutParaFile(), true);
}

void FlatWeldEndpointSearch::deleteOldData()
{
	XiBase::DelFiles(getSrcImageFolder());
	XiBase::DelFiles(getDstImageFolder());
	deleteOldFullPointCloudFile();
}

CString FlatWeldEndpointSearch::getLockSrcImageName()
{
	CString sName;
	sName.Format(getSrcImageFolder() + _T("Lock.jpg"));
	return sName;
}

CString FlatWeldEndpointSearch::getLockDstImageName()
{
	CString sName;
	sName.Format(getDstImageFolder() + _T("Lock.jpg"));
	return sName;
}

CString FlatWeldEndpointSearch::getSrcImageName(int index)
{
	CString sName;
	sName.Format(getSrcImageFolder() + _T("%d.bin"), index);
	return sName;
}

CString FlatWeldEndpointSearch::getDstImageName(int index)
{
	CString sName;
	sName.Format(getDstImageFolder() + _T("%d.bin"), index);
	return sName;
}

void FlatWeldEndpointSearch::saveSrcImage(int index)
{
	auto savePath = getSrcImageName(index);
	BinaryImgSaveAsynchronous((char*)(const char*)savePath, m_pSrcImage, 0);
}

void FlatWeldEndpointSearch::loadSrcImage()
{
	XUI::MesBox::PopError("暂不支持读取搜端点图片");
}

void FlatWeldEndpointSearch::saveDstImage(int index)
{
	XUI::MesBox::PopError("暂不支持保存搜端点处理后扫描图片");
}

void FlatWeldEndpointSearch::loadDstImage()
{
	XUI::MesBox::PopError("暂不支持读取搜端点处理后扫描图片");
}

CString FlatWeldEndpointSearch::getPointCloudFolder()
{
	return m_sDataFolder + _T("PointCloud\\");
}

bool FlatWeldEndpointSearch::lockLaser()
{
	// 采图
	if (!captureImage(3))
	{
		m_pRobot->m_cLog->Write("lockLaser 采图失败！");
		return false;
	}

	//保存锁定图
	cvSaveImage(getLockSrcImageName(), m_pSrcImage);

	//保存锁定参数
	bool assembleFilp = PARA_FLAT_MEASURE(bImageDirection);
	bool crossFilp = PARA_FLAT_MEASURE(bCrossFilp);
	XiBase::COPini opini;
	opini.SetFileName(getOutPutParaFile());
	opini.SetSectionName("Lock");
	opini.WriteString("assembleFilp", assembleFilp);
	opini.WriteString("crossFilp", crossFilp);
	opini.WriteString("LaserLockConfigName", m_sLaserLockConfigName);

	//锁定
	CvPoint cpKeyPoint, LeftPtn, RightPtn;
	int nRst = WidgetLaserLock(m_pSrcImage, &m_tSearchLockLaserInfo, 
		assembleFilp, crossFilp, (char*)(const char*)m_sLaserLockConfigName);
	if (nRst > 0)
	{
		cpKeyPoint = m_tSearchLockLaserInfo.crossPoint;
		LeftPtn = m_tSearchLockLaserInfo.verticalPlatePnt;
		RightPtn = m_tSearchLockLaserInfo.bottomPlatePnt;
	}
	else
	{
		saveErrorData(true, "搜端点初始锁定失败");
		XUI::MesBox::PopError("搜索激光斜率锁定失败！(错误代码：找不到关键点)");
		return false;
	}

	//显示结果
	cvCvtColor(m_pSrcImage, m_pDstImage, CV_GRAY2RGB);
	cvLine(m_pDstImage, cpKeyPoint, LeftPtn, CV_RGB(0, 255, 0), 2);
	cvLine(m_pDstImage, cpKeyPoint, RightPtn, CV_RGB(0, 0, 255), 2);
	cvCircle(m_pDstImage, cpKeyPoint, 10, CV_RGB(255, 0, 0), 3);

	//保存结果
	cvSaveImage(getLockSrcImageName(), m_pDstImage);
	return true;
}

bool FlatWeldEndpointSearch::beforeProcessImage()
{
	closeFullPointCloudFile();

	CTime cTime;
	cTime = CTime::GetCurrentTime();
	m_sPointCloudFileName.Format(getPointCloudFolder() + "%04d-%02d-%02d_%02d-%02d-%02d.txt",
		cTime.GetYear(), cTime.GetMonth(), cTime.GetDay(),
		cTime.GetHour(), cTime.GetMinute(), cTime.GetSecond());
	return 0 == XI_fopen_s(&m_pFullPointCloudFile, m_sPointCloudFileName, "w");
}

bool FlatWeldEndpointSearch::initImageProcessPara(int index, const T_ANGLE_PULSE& tCapPulse, const T_ROBOT_COORS& tCapCoord)
{
	m_tProcessPara.index = index;
	m_tProcessPara.tCapPulse = tCapPulse;
	m_tProcessPara.tCapCoord = tCapCoord;

	saveImageProcessPara(index, m_tProcessPara);
	return true;
}

bool FlatWeldEndpointSearch::processImage()
{
	m_vtPoints.clear();

	// 锁定激光
	if (!WidgetLaserTrack(m_pSrcImage, &m_tSearchLockLaserInfo, (char*)(const char*)m_sLaserTrackConfigName))
	{
		m_pRobot->m_cLog->Write("WidgetLaserTrack处理失败");
		//return false;
	}

	// 提取二维点
	std::vector<CvPoint> vtPoint2D(10000);
	int nLength = WidgetLaserPntExt(m_pSrcImage, &m_tSearchLockLaserInfo, vtPoint2D.data(), (char*)(const char*)m_sLaserPntConfigName);
	if (nLength <= 0)
	{
		m_pRobot->m_cLog->Write("WidgetLaserPntExt处理失败");
		return false;
	}
	vtPoint2D.resize(nLength);

	// 二维点转三维点
	auto vtPoint3D = m_pUnit->TranImageToBase(m_nCameraNo,
		vtPoint2D, m_tProcessPara.tCapCoord, m_tProcessPara.tCapPulse);

	for (auto& point : vtPoint3D)
	{
		Three_DPoint point3D{
			point.dX + m_tProcessPara.tCapCoord.dBX,
			point.dY + m_tProcessPara.tCapCoord.dBY,
			point.dZ + m_tProcessPara.tCapCoord.dBZ };
		m_vtPoints.push_back(point3D);
		fprintf(m_pFullPointCloudFile, "%4d %11.3lf %11.3lf %11.3lf\n",
			m_tProcessPara.index, point3D.x, point3D.y, point3D.z);
	}
	fflush(m_pFullPointCloudFile);
	return true;
}

bool FlatWeldEndpointSearch::afterProcessImage()
{
	closeFullPointCloudFile();
	return true;
}

void FlatWeldEndpointSearch::drawResult()
{
	XUI::MesBox::PopError("暂不支持搜端点结果展示");
}

void FlatWeldEndpointSearch::saveErrorData(bool bAutoSave, CString sName)
{
	// 当前时间
	SYSTEMTIME tTime;
	GetLocalTime(&tTime);

	// 保存路径
	CString sSavePath;
	sSavePath.Format(m_sErrorDataDir + "%04d-%02d-%02d-%02d-%02d-%02d-" + sName + "\\",
		tTime.wYear, tTime.wMonth, tTime.wDay, tTime.wHour, tTime.wMinute, tTime.wSecond);

	// 保存图像库信息
	XiBase::COPini opini;
	opini.SetFileName(getOutPutParaFile());
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
	CopyFolder(m_sDataFolder, sSavePath);
	CopyFolder(m_sConfigFolder, sSavePath + "Configs\\");

	//// 压缩文件夹
	//std::string srcPath = sSavePath;
	//std::string dstPath = sSavePath.Left(sSavePath.GetLength() - 1) + ".zip";
	//xi::zip::ZipDir(srcPath, dstPath);

	//// 删除源文件夹
	//DelFiles(sSavePath);
	//RemoveDirectory(sSavePath);
}

void FlatWeldEndpointSearch::closeFullPointCloudFile()
{
	if (m_pFullPointCloudFile)
	{
		fclose(m_pFullPointCloudFile);
		m_pFullPointCloudFile = nullptr;
	}
}

void FlatWeldEndpointSearch::deleteOldFullPointCloudFile(int nRetentionFileCount)
{
	std::vector<CString> vsFileList;
	XiBase::FindFile(vsFileList, getPointCloudFolder(), "txt", false);

	if (nRetentionFileCount > vsFileList.size())
		return;

	std::sort(vsFileList.begin(), vsFileList.end(),
		[](const CString& a, const CString& b)
		{
			return a.Compare(b) < 0;
		});

	for (int i = 0; i < vsFileList.size() - nRetentionFileCount; ++i)
	{
		DeleteFile(vsFileList[i]);
	}
}

void FlatWeldEndpointSearch::saveImageProcessPara(int index, const ImageProcessPara& para) const
{
	XiBase::COPini opini;
	opini.SetFileName(getOutPutParaFile());
	CString sSectionName;
	sSectionName.Format("Image_%d", index);
	opini.SetSectionName(sSectionName);
	opini.WriteString("tCapCoord", "", para.tCapCoord);
	opini.WriteString("tCapPulse", "", para.tCapPulse);
}

}