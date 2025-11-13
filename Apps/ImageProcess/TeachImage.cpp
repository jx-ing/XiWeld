#include "stdafx.h"
#include "TeachImage.h"

#include "Apps/SmallPiece/Infrastructure/zlib/XiZip.h"

namespace image_process
{
TeachImage::TeachImage()
{
	m_sDataFolder = _T(".\\WeldData\\RobotA\\TeachImage\\"); 
	m_sErrorDataDir = _T(".\\ErrorData\\Teach\\");
	makesureDataFolderExist();
}

TeachImage::~TeachImage()
{
}

void TeachImage::makesureDataFolderExist()
{
	XiBase::CheckFolder(getSrcImageFolder(), true);
	XiBase::CheckFolder(getDstImageFolder(), true);
	XiBase::CheckFolder(m_sErrorDataDir, true);
	CheckFileExists(getOutPutParaFile(), true);
}

void TeachImage::deleteOldData()
{
	XiBase::DelFiles(getSrcImageFolder());
	XiBase::DelFiles(getDstImageFolder());
}

CString TeachImage::getSrcImageName(int index)
{
	CString sName;
	sName.Format(getSrcImageFolder() + _T("%d.jpg"), index);
	return sName;
}

CString TeachImage::getDstImageName(int index)
{
	CString sName;
	sName.Format(getDstImageFolder() + _T("%d.jpg"), index);
	return sName;
}

void TeachImage::saveSrcImage(int index)
{
	cvSaveImage(getSrcImageName(index), m_pSrcImage);
}

void TeachImage::loadSrcImage()
{
	m_vpSrcImages.clear();
	int nIndex = 0;
	auto sName = getSrcImageName(nIndex);
	while (CheckFileExists(sName))
	{
		m_vpSrcImages.push_back(cvLoadImage(sName, 1));
		nIndex++;
		sName = getSrcImageName(nIndex);
	}
}

void TeachImage::saveDstImage(int index)
{
	cvSaveImage(getDstImageName(index), m_pDstImage);
}

void TeachImage::loadDstImage()
{
	m_vpDstImages.clear();
	int nIndex = 0;
	auto sName = getDstImageName(nIndex);
	while (CheckFileExists(sName))
	{
		m_vpDstImages.push_back(cvLoadImage(sName, 3));
		nIndex++;
		sName = getDstImageName(nIndex);
	}
}

bool TeachImage::initImageProcessPara(int index, int nMeasureType, double dExAxlePos,
	const T_ANGLE_PULSE& tCapPulse)
{
	m_tProcessPara.index = index;
	m_tProcessPara.measureType = nMeasureType;
	m_tProcessPara.dExAxlePos = dExAxlePos;
	m_tProcessPara.tCapPulse = tCapPulse;

	if (nMeasureType & (E_DOUBLE_LONG_LINE))
	{
		m_tProcessPara.assembleFilp = PARA_FLAT_MEASURE(bImageDirection);
		m_tProcessPara.crossFilp = PARA_FLAT_MEASURE(bCrossFilp);
		m_tProcessPara.configFile = m_sLaserPntConfigName[0];
	}
	else if (nMeasureType & (E_LS_RL_FLIP))
	{
		m_tProcessPara.assembleFilp = true;
		m_tProcessPara.crossFilp = !PARA_FLAT_MEASURE(bCrossFilp);
		m_tProcessPara.configFile = m_sLaserPntConfigName[1];
	}
	else if (nMeasureType & (E_LL_RS_FLIP))
	{
		m_tProcessPara.assembleFilp = false;
		m_tProcessPara.crossFilp = !PARA_FLAT_MEASURE(bCrossFilp);
		m_tProcessPara.configFile = m_sLaserPntConfigName[2];
	}
	else
	{
		XUI::MesBox::PopInfo("示教失败，请检查测量点类型{0}是否正确", nMeasureType);
		return false;
	}

	// 保存图像处理参数
	saveImageProcessPara(index, m_tProcessPara);

	return true;
}

bool TeachImage::processImage()
{
	// 初始化示教结果
	initTeachResult(m_tProcessPara.dExAxlePos, m_tProcessPara.tCapPulse);

	// 直线上二维点总数
	int nLinePtnNum = 5;

	// 理论相机中心三维点
	T_ROBOT_COORS tCameraCoors;
	m_pRobot->RobotKinematics(m_tTeachResult.tRobotPulse,
		m_pRobot->m_tTools.tCameraTool, tCameraCoors);

	// 获取二维点
	long long tTime = XI_clock();
	WidgetLaserInfo* pLaserInfo = new WidgetLaserInfo[10];
	int nKeyPointCount = WidgetLaserLock(m_pSrcImage, pLaserInfo,
		m_tProcessPara.assembleFilp, m_tProcessPara.crossFilp,
		(char*)(const char*)m_tProcessPara.configFile);

	// 选择二维点
	if (nKeyPointCount == 1)
	{
		WidgetLaserInfo2TeachResult(pLaserInfo[0], m_tTeachResult);
	}
	else if (nKeyPointCount > 1)
	{
		CvPoint tmpPoint;
		T_ROBOT_COORS tmpKeyPoint;
		double wDis;
		double zDis;
		vector<double> vtWorldDis;
		vector<double> vtHeightDis;
		vector<double> vtZ;
		for (int keyNum = 0; keyNum < nKeyPointCount; keyNum++)
		{
			tmpPoint = pLaserInfo[keyNum].crossPoint;
			tmpKeyPoint = m_pUnit->TranImageToBase(m_nCameraNo,
				tmpPoint, m_tTeachResult.tRobotCoors, m_tTeachResult.tRobotPulse);
			wDis = TwoPointDis(tmpKeyPoint.dX, tmpKeyPoint.dY, tmpKeyPoint.dZ,
				tCameraCoors.dX, tCameraCoors.dY, tCameraCoors.dZ);
			zDis = fabs(tmpKeyPoint.dZ - tCameraCoors.dZ);
			vtHeightDis.push_back(zDis);
			vtWorldDis.push_back(wDis);
			vtZ.push_back(tmpKeyPoint.dZ);
		}

		int realKey = 0;
		auto minWorldPosition = min_element(vtWorldDis.begin(), vtWorldDis.end());
		realKey = minWorldPosition - vtWorldDis.begin();
		WidgetLaserInfo2TeachResult(pLaserInfo[realKey], m_tTeachResult);
	}
	WriteLog("Process%d %d 示教处理时间 %d",
		m_tProcessPara.index, m_tProcessPara.measureType, XI_clock() - tTime);

	// 如果点数不够，保存错误数据
	if (0 >= nKeyPointCount
		|| 0 >= m_tTeachResult.vtLeftPtns2D.size()
		|| 0 >= m_tTeachResult.vtLeftPtns3D.size())
	{
		showSrcImage();
		saveErrorData(true, "示教失败");
		XUI::MesBox::PopError("示教失败，请检查激光图片{0}是否正常", m_tProcessPara.index);
		return false;
	}

	// 二维点转三维点
	trans2DTo3D(m_tTeachResult);

	WriteLog("测量焊枪位置:%11.3lf %11.3lf %11.3lf", m_tTeachResult.tRobotCoors.dX,
		m_tTeachResult.tRobotCoors.dY, m_tTeachResult.tRobotCoors.dZ);
	WriteLog("测量相机位置:%11.3lf %11.3lf %11.3lf",
		tCameraCoors.dX, tCameraCoors.dY, tCameraCoors.dZ);
	WriteLog("先测后焊中心二维点：%d %d", m_tTeachResult.tKeyPtn2D.x, m_tTeachResult.tKeyPtn2D.y);
	WriteLog("先测后焊二转三直角：%.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf",
		m_tTeachResult.tRobotCoors.dX, m_tTeachResult.tRobotCoors.dY,
		m_tTeachResult.tRobotCoors.dZ, m_tTeachResult.tRobotCoors.dRX,
		m_tTeachResult.tRobotCoors.dRY, m_tTeachResult.tRobotCoors.dRZ);
	WriteLog("先测后焊二转三脉冲：%d %d %d %d %d %d",
		m_tTeachResult.tRobotPulse.nSPulse, m_tTeachResult.tRobotPulse.nLPulse,
		m_tTeachResult.tRobotPulse.nUPulse, m_tTeachResult.tRobotPulse.nRPulse,
		m_tTeachResult.tRobotPulse.nBPulse, m_tTeachResult.tRobotPulse.nTPulse);
	return true;
}

void TeachImage::drawResult()
{
	// 转颜色空间(转RGB彩色图)
	cvCvtColor(m_pSrcImage, m_pDstImage, CV_GRAY2RGB);

	for (int i = 0; i < m_tTeachResult.vtLeftPtns2D.size(); i++)
	{
		cvCircle(m_pDstImage, m_tTeachResult.vtLeftPtns2D[i], 6, CV_RGB(0, 255, 0), 2); // 左线
	}
	for (int i = 0; i < m_tTeachResult.vtRightPtns2D.size(); i++)
	{
		cvCircle(m_pDstImage, m_tTeachResult.vtRightPtns2D[i], 6, CV_RGB(0, 0, 255), 2); // 右线
	}
	cvCircle(m_pDstImage, m_tTeachResult.tKeyPtn2D, 6, CV_RGB(255, 0, 0), 2); // 交点

}

void TeachImage::saveErrorData(bool bAutoSave, CString sName)
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

	// 压缩文件夹
	std::string srcPath = sSavePath;
	std::string dstPath = sSavePath.Left(sSavePath.GetLength() - 1) + ".zip";
	xi::zip::ZipDir(srcPath, dstPath);

	// 删除源文件夹
	DelFiles(sSavePath);
	RemoveDirectory(sSavePath);
}

void TeachImage::saveImageProcessPara(int index, const ImageProcessPara& para) const
{
	XiBase::COPini opini;
	opini.SetFileName(getOutPutParaFile());
	CString sSectionName;
	sSectionName.Format("Image_%d", index);
	opini.SetSectionName(sSectionName);
	opini.WriteString("measureType", para.measureType);
	opini.WriteString("assembleFilp", para.assembleFilp);
	opini.WriteString("crossFilp", para.crossFilp);
	opini.WriteString("configFile", para.configFile);
	opini.WriteString("dExAxlePos", para.dExAxlePos);
	opini.WriteString("tCapPulse", "", para.tCapPulse);
}

void TeachImage::loadImageProcessPara(int index, ImageProcessPara& para) const
{
	XiBase::COPini opini;
	opini.SetFileName(getOutPutParaFile());
	CString sSectionName;
	sSectionName.Format("Image_%d", index);
	opini.SetSectionName(sSectionName);
	opini.ReadString("measureType", &para.measureType);
	opini.ReadString("assembleFilp", &para.assembleFilp);
	opini.ReadString("crossFilp", &para.crossFilp);
	opini.ReadString("configFile", para.configFile);
	opini.ReadString("dExAxlePos", &para.dExAxlePos);
	opini.ReadString("tCapPulse", "", para.tCapPulse);
}

void TeachImage::initTeachResult(
	double dExAxlePos, const T_ANGLE_PULSE& tCapPulse)
{
	m_tTeachResult.vtLeftPtns2D.clear();
	m_tTeachResult.vtRightPtns2D.clear();
	m_tTeachResult.vtLeftPtns3D.clear();
	m_tTeachResult.vtRightPtns3D.clear();
	m_tTeachResult.dExAxlePos = dExAxlePos;
	m_tTeachResult.tRobotPulse = tCapPulse;
	m_pRobot->RobotKinematics(m_tTeachResult.tRobotPulse,
		m_pRobot->m_tTools.tGunTool, m_tTeachResult.tRobotCoors);
}

void TeachImage::WidgetLaserInfo2TeachResult(
	const WidgetLaserInfo& tWidgetLaserInfo, T_TEACH_RESULT& result)
{
	m_tTeachResult.tKeyPtn2D = tWidgetLaserInfo.crossPoint;
	int nStep = tWidgetLaserInfo.samPntNum / 5;
	for (int i = 0; i < tWidgetLaserInfo.samPntNum; i += nStep)
	{
		// 激光图左侧激光线 立板
		m_tTeachResult.vtLeftPtns2D.push_back(tWidgetLaserInfo.bottomPlateLineSamPnt[i]);

		// 激光图右侧激光线 底板
		m_tTeachResult.vtRightPtns2D.push_back(tWidgetLaserInfo.verticalPlateLineSamPnt[i]);
	}
}

void TeachImage::trans2DTo3D(T_TEACH_RESULT& result)
{
	// 交点：二维点转三维坐标(机器人坐标)
	T_ROBOT_COORS tRetCoord = m_pUnit->TranImageToBase(m_nCameraNo,
		result.tKeyPtn2D, result.tRobotCoors, result.tRobotPulse);
	result.tKeyPtn3D = P2P(tRetCoord);

	// 左线：二维点转三维坐标(机器人坐标)
	for (auto& pnt : result.vtLeftPtns2D)
	{
		tRetCoord = m_pUnit->TranImageToBase(m_nCameraNo,
			pnt, result.tRobotCoors, result.tRobotPulse);
		result.vtLeftPtns3D.push_back(P2P(tRetCoord));
	}

	// 右线：交点二维点转三维坐标(机器人坐标)
	for (auto& pnt : result.vtRightPtns2D)
	{
		tRetCoord = m_pUnit->TranImageToBase(m_nCameraNo,
			pnt, result.tRobotCoors, result.tRobotPulse);
		result.vtRightPtns3D.push_back(P2P(tRetCoord));
	}
}

}