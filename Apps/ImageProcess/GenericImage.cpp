#include "stdafx.h"
#include "GenericImage.h"

#include "LocalFiles/ExLib/Vision/include/GFPGJointLib.h"

namespace image_process
{
GenericImage::GenericImage()
{
	releaseResources();
}

GenericImage::~GenericImage()
{
	releaseResources();
}

void GenericImage::setShowImage(IplImage** pShowImg)
{
	m_pShowImg = pShowImg;
}

void GenericImage::setHardware(CUnit* pUnit, int nCameraNo)
{
	m_pUnit = pUnit;
	m_nCameraNo = nCameraNo;
	m_pCamera = (CDHGigeImageCapture*)m_pUnit->GetCameraCtrl(m_nCameraNo);
	m_pRobot = m_pUnit->GetRobotCtrl();
}

void GenericImage::createImageBuffer()
{
	m_pSrcImage = cvCreateImage(
		cvSize(m_pCamera->m_nImageWidth, m_pCamera->m_nImageHeight), IPL_DEPTH_8U, 1);
	m_pDstImage = cvCreateImage(
		cvSize(m_pCamera->m_nImageWidth, m_pCamera->m_nImageHeight), IPL_DEPTH_8U, 3);
}

CString GenericImage::getSrcImageFolder() const
{
	return m_sDataFolder + _T("Src\\");
}

CString GenericImage::getDstImageFolder() const
{
	return m_sDataFolder + _T("Dst\\");
}

CString GenericImage::getOutPutParaFile() const
{
	return m_sDataFolder + _T("para.ini");
}

CString GenericImage::getDataFolder() const
{
	return m_sDataFolder;
}

bool GenericImage::openCamera(E_DHGIGE_ACQUISITION_MODE eCaptureMode, E_DHGIGE_CALL_BACK eCallBackMode)
{
	return 0 == m_pUnit->SwitchDHCamera(m_nCameraNo, true, true, eCaptureMode, eCallBackMode);
}

bool GenericImage::closeCamera()
{
	return 0 == m_pUnit->SwitchDHCamera(m_nCameraNo, false);
}

bool GenericImage::startAcquisition()
{
	return m_pCamera->StartAcquisition();
}

bool GenericImage::captureImage(int nTryTimes)
{
	if (!m_pCamera->CaptureImage(m_pSrcImage, nTryTimes))
	{
		m_pCamera->ShowErrorString();
		return false;
	}
	return true;
}

void GenericImage::showSrcImage()
{
	if (m_pSrcImage && m_pShowImg && *m_pShowImg)
		cvCvtColor(m_pSrcImage, *m_pShowImg, CV_GRAY2RGB);
}

void GenericImage::showDstImage()
{
	if (m_pDstImage && m_pShowImg && *m_pShowImg)
		cvCopyImage(m_pDstImage, *m_pShowImg);
}

void GenericImage::openErrorDataFolder()
{
	ShellExecute(NULL, "open", m_sErrorDataDir, NULL, NULL, SW_SHOW);
}

void GenericImage::releaseResources()
{
	releaseSrcImages();
	releaseDstImages();
}

void GenericImage::releaseSrcImages()
{
	cvReleaseImage(&m_pSrcImage);
	m_pSrcImage = nullptr;

	for (auto pImage : m_vpSrcImages)
	{
		cvReleaseImage(&pImage);
	}
	m_vpSrcImages.clear();
}

void GenericImage::releaseDstImages()
{
	cvReleaseImage(&m_pDstImage);
	m_pDstImage = nullptr;

	for (auto pImage : m_vpDstImages)
	{
		cvReleaseImage(&pImage);
	}
	m_vpDstImages.clear();
}

}