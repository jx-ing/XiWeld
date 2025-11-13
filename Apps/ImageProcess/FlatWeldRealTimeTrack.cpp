#include "stdafx.h"
#include "FlatWeldRealTimeTrack.h"

#include "Apps/SmallPiece/Infrastructure/zlib/XiZip.h"

namespace image_process
{
FlatWeldRealTimeTrack::FlatWeldRealTimeTrack()
{
	m_sDataFolder = _T(".\\WeldData\\RobotA\\FlatWeldRealTimeTrack\\");
	m_sErrorDataDir = _T(".\\ErrorData\\FlatWeldRealTimeTrack\\");
	makesureDataFolderExist();
}

FlatWeldRealTimeTrack::~FlatWeldRealTimeTrack()
{
	deleteWidgetLaserTrackTool();
	deleteImageProcessPointer();
	closeFullPointCloudFile();
	if (m_pSearchLockLaserInfo)
	{
		delete m_pSearchLockLaserInfo;
		m_pSearchLockLaserInfo = nullptr;
	}
}

void FlatWeldRealTimeTrack::makesureDataFolderExist()
{
	XiBase::CheckFolder(getSrcImageFolder(), true);
	XiBase::CheckFolder(getDstImageFolder(), true);
	XiBase::CheckFolder(getPointCloudFolder(), true);
	XiBase::CheckFolder(m_sErrorDataDir, true);
	CheckFileExists(getOutPutParaFile(), true);
}

void FlatWeldRealTimeTrack::deleteOldData()
{
	XiBase::DelFiles(getSrcImageFolder());
	XiBase::DelFiles(getDstImageFolder());
	deleteOldFullPointCloudFile();
}

CString FlatWeldRealTimeTrack::getLockSrcImageName()
{
	CString sName;
	sName.Format(getSrcImageFolder() + _T("Lock.jpg"));
	return sName;
}

CString FlatWeldRealTimeTrack::getLockDstImageName()
{
	CString sName;
	sName.Format(getDstImageFolder() + _T("Lock.jpg"));
	return sName;
}

CString FlatWeldRealTimeTrack::getSrcImageName(int index)
{
	CString sName;
	sName.Format(getSrcImageFolder() + _T("%d.bin"), index);
	return sName;
}

CString FlatWeldRealTimeTrack::getDstImageName(int index)
{
	CString sName;
	sName.Format(getDstImageFolder() + _T("%d.bin"), index);
	return sName;
}

void FlatWeldRealTimeTrack::saveSrcImage(int index)
{
	auto savePath = getSrcImageName(index);
	BinaryImgSaveAsynchronous((char*)(const char*)savePath, m_pSrcImage, 0);
}

void FlatWeldRealTimeTrack::loadSrcImage()
{
	XUI::MesBox::PopError("暂不支持读取搜端点图片");
}

void FlatWeldRealTimeTrack::saveDstImage(int index)
{
	XUI::MesBox::PopError("暂不支持保存搜端点处理后扫描图片");
}

void FlatWeldRealTimeTrack::loadDstImage()
{
	XUI::MesBox::PopError("暂不支持读取搜端点处理后扫描图片");
}

CString FlatWeldRealTimeTrack::getPointCloudFolder()
{
	return m_sDataFolder + _T("PointCloud\\");
}

bool FlatWeldRealTimeTrack::beforeProcessImage()
{
	closeFullPointCloudFile();
	deleteWidgetLaserTrackTool();
	deleteImageProcessPointer();
	m_pWidgetLaserTrackTool = new WidgetLaserTrackTool();

	auto cstrTrackDipParamPath = m_pUnit->GetTrackTipDataFileName(m_nCameraNo);
	m_pTraceImgProcess = new GROUP_STAND_DIP_NS::CImageProcess(
		m_pCamera->m_nImageWidth, 
		m_pCamera->m_nImageHeight,
		1,
		cstrTrackDipParamPath);

	CTime cTime;
	cTime = CTime::GetCurrentTime();
	m_sPointCloudFileName.Format(getPointCloudFolder() + "%04d-%02d-%02d_%02d-%02d-%02d.txt",
		cTime.GetYear(), cTime.GetMonth(), cTime.GetDay(),
		cTime.GetHour(), cTime.GetMinute(), cTime.GetSecond());
	if (0 != XI_fopen_s(&m_pFullPointCloudFile, m_sPointCloudFileName, "w"))
	{
		XUI::MesBox::PopError("无法打开文件{0}", m_sPointCloudFileName);
		return false;
	}
	return true;
}

bool FlatWeldRealTimeTrack::lockLaser(bool bIsContinueWeld)
{
	if (bIsContinueWeld)
	{
		m_pSrcImage = cvLoadImage(getLockSrcImageName(), 0);
	}
	else
	{
		// 采图
		if (!captureImage(3))
		{
			m_pRobot->m_cLog->Write("lockLaser 采图失败！");
			return false;
		}
		//保存锁定图
		cvSaveImage(getLockSrcImageName(), m_pSrcImage);
	}

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
	int nRst = WidgetLaserLock(m_pSrcImage, m_pSearchLockLaserInfo,
		assembleFilp, crossFilp, (char*)(const char*)m_sLaserLockConfigName);
	if (nRst > 0)
	{
		cpKeyPoint = m_pSearchLockLaserInfo[0].crossPoint;
		LeftPtn = m_pSearchLockLaserInfo[0].verticalPlatePnt;
		RightPtn = m_pSearchLockLaserInfo[0].bottomPlatePnt;
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

	// 根据返回结果生成旧接口参考线
	m_pTraceImgProcess->HandlockKeyPoint(cpKeyPoint, LeftPtn, RightPtn,
		m_tFrontLine, m_tBackLine);

	//初始化图像处理参数
	m_pWidgetLaserTrackTool->AddLockInfo(m_pSearchLockLaserInfo);
	if (false)
	{
		m_pRobot->m_cLog->Write("输入：多层多道跟踪参数");
		m_pWidgetLaserTrackTool->AddTrackParamSet("WidgetLaserTrack_MultiLayer");
		m_pWidgetLaserTrackTool->AddTrackParamSet("WidgetLaserTrack_Big");
	}
	else
	{
		m_pRobot->m_cLog->Write("输入：单道跟踪参数");
		m_pWidgetLaserTrackTool->AddTrackParamSet("WidgetLaserTrack");
		m_pWidgetLaserTrackTool->AddTrackParamSet("WidgetLaserTrack_Small");
		m_pWidgetLaserTrackTool->AddTrackParamSet("WidgetLaserTrack_Big");
	}
	return true;
}

bool FlatWeldRealTimeTrack::initImageProcessPara(int index, const T_ANGLE_PULSE& tCapPulse, const T_ROBOT_COORS& tCapCoord)
{
	m_tProcessPara.index = index;
	m_tProcessPara.tCapPulse = tCapPulse;
	m_tProcessPara.tCapCoord = tCapCoord;

	saveImageProcessPara(index, m_tProcessPara);
	return true;
}

bool FlatWeldRealTimeTrack::processImage()
{
	m_vtPoints.clear();

	// 激光跟踪配置文件名称
	CString sLaserTrackConfigName;
	if (m_tProcessPara.index < 3)
	{
		sLaserTrackConfigName.Format("WidgetLaserTrack_Big");
	}
	else
	{
		sLaserTrackConfigName.Format("WidgetLaserTrack");
	}
	if (false)//多层多道
	{
		sLaserTrackConfigName.Format("WidgetLaserTrack_MultiLayer");
	}

	// 锁定激光
	if (!m_pWidgetLaserTrackTool->TryTrack(m_pSrcImage, m_pSearchLockLaserInfo, 50,
		(char*)(const char*)sLaserTrackConfigName))
	{
		m_pRobot->m_cLog->Write("TryTrack处理失败");
		m_bGetKeyPoint = false;
	}
	else
	{
		// 根据返回结果生成旧接口参考线
		m_pTraceImgProcess->HandlockKeyPoint(m_pSearchLockLaserInfo[0].crossPoint,
			m_pSearchLockLaserInfo[0].verticalPlatePnt, m_pSearchLockLaserInfo[0].bottomPlatePnt,
			m_tFrontLine, m_tBackLine);
		m_bGetKeyPoint = true;
	}

	// 提取二维点
	std::vector<CvPoint> vtPoint2D(10000);
	int nLength = FindLaserMidPiontEEEEEInTrack_Fix(m_pSrcImage, m_tFrontLine, m_tBackLine,
		vtPoint2D.data(), (char*)(const char*)m_sLaserPntConfigName);
	//int nLength = WidgetLaserPntExt(m_pSrcImage, m_pSearchLockLaserInfo, vtPoint2D.data(), (char*)(const char*)m_sLaserPntConfigName);
	if (nLength <= 0)
	{
		m_bGetKeyPoint = false;
		m_pRobot->m_cLog->Write("FindLaserMidPiontEEEEEInTrack_Fix处理失败");
		return false;
	}
	vtPoint2D.resize(nLength + 1);
	vtPoint2D.back() = m_pSearchLockLaserInfo[0].crossPoint;//尾部添加关键点
	m_tKeyPointPixelCoord = vtPoint2D.back();

	// 二维点转三维点
	auto vtPoint3D = m_pUnit->TranImageToBase(m_nCameraNo,
		vtPoint2D, m_tProcessPara.tCapCoord, m_tProcessPara.tCapPulse);
	m_tKeyPointCoord = vtPoint3D.back();
	vtPoint3D.pop_back();//删除尾部关键点

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

bool FlatWeldRealTimeTrack::afterProcessImage()
{
	closeFullPointCloudFile();
	return true;
}

void FlatWeldRealTimeTrack::drawResult()
{
	// 转颜色空间(转RGB彩色图)
	cvCvtColor(m_pSrcImage, m_pDstImage, CV_GRAY2RGB);
	cvCircle(m_pDstImage, m_tKeyPointPixelCoord, 8, CV_RGB(255, 0, 0), 4);
}

void FlatWeldRealTimeTrack::saveErrorData(bool bAutoSave, CString sName)
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

	// 找激光点参数
	CopyFile(m_sLaserPntConfigPath, sSavePath + "Configs\\" + m_sLaserPntConfigName + ".ini", false);

	//// 压缩文件夹
	//std::string srcPath = sSavePath;
	//std::string dstPath = sSavePath.Left(sSavePath.GetLength() - 1) + ".zip";
	//xi::zip::ZipDir(srcPath, dstPath);

	//// 删除源文件夹
	//DelFiles(sSavePath);
	//RemoveDirectory(sSavePath);
}

void FlatWeldRealTimeTrack::closeFullPointCloudFile()
{
	if (m_pFullPointCloudFile)
	{
		fclose(m_pFullPointCloudFile);
		m_pFullPointCloudFile = nullptr;
	}
}

void FlatWeldRealTimeTrack::deleteOldFullPointCloudFile(int nRetentionFileCount)
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

void FlatWeldRealTimeTrack::saveImageProcessPara(int index, const ImageProcessPara& para) const
{
	XiBase::COPini opini;
	opini.SetFileName(getOutPutParaFile());
	CString sSectionName;
	sSectionName.Format("Image_%d", index);
	opini.SetSectionName(sSectionName);
	opini.WriteString("tCapCoord", "", para.tCapCoord);
	opini.WriteString("tCapPulse", "", para.tCapPulse);
}

void FlatWeldRealTimeTrack::deleteWidgetLaserTrackTool()
{
	if (m_pWidgetLaserTrackTool)
	{
		delete m_pWidgetLaserTrackTool;
		m_pWidgetLaserTrackTool = nullptr;
	}
}

void FlatWeldRealTimeTrack::deleteImageProcessPointer()
{
	if (m_pTraceImgProcess)
	{
		delete m_pTraceImgProcess;
		m_pTraceImgProcess = nullptr;
	}
}

}