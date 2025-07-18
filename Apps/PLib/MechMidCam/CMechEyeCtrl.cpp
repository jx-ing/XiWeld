#include "stdafx.h"
#include ".\Apps\PLib\MechMidCam\CMechEyeCtrl.h"

#if ENABLE_MECH_EYE

CMechEyeCtrl::CMechEyeCtrl(T_MECHEYE_CAMREA_DRIVER_PARAM tMecheyeCameraDriverPara)
{
	m_tMecheyeCameraDriverPara = tMecheyeCameraDriverPara;
	m_pMechMindDevice = new MechEyeDevice;
}

CMechEyeCtrl::~CMechEyeCtrl()
{
	if (m_pMechMindDevice != NULL)
	{
		m_pMechMindDevice->disconnect();
	}
}

int CMechEyeCtrl::InitMechEyeCamera()
{
	ErrorStatus status;
	status = m_pMechMindDevice->connect(m_tMecheyeCameraDriverPara.strDeviceAddress.GetBuffer(), m_tMecheyeCameraDriverPara.nPort);	
	if (!status.isOK())
	{
		return status.errorCode;
	}
	MechEyeDeviceInfo tDeviceInfo;
	status = m_pMechMindDevice->getDeviceInfo(tDeviceInfo);
	if (!status.isOK())
	{
		return status.errorCode;
	}
	m_tMecheyeCameraDriverPara.strModelName = tDeviceInfo.model.c_str();
	m_tMecheyeCameraDriverPara.strHardwareVersion = tDeviceInfo.hardwareVersion.c_str();
	m_tMecheyeCameraDriverPara.strFirmwareVersion = tDeviceInfo.firmwareVersion.c_str();
	m_tMecheyeCameraDriverPara.strID = tDeviceInfo.id.c_str();
	return 0;
}

inline void saveMap(mmind::api::ColorMap color, std::string path)
{
	cv::Mat color8UC3 = cv::Mat(color.height(), color.width(), CV_8UC3, color.data());
	cv::imwrite(path, color8UC3);
	std::cout << "Capture and save color image : " << path << std::endl;
}

inline void saveMap(mmind::api::DepthMap depth, std::string path)
{
	cv::Mat depth32F = cv::Mat(depth.height(), depth.width(), CV_32FC1, depth.data());
	cv::imwrite(path, depth32F);
	std::cout << "Capture and save depth image : " << path << std::endl;
}

int CMechEyeCtrl::CapturePointXYZ(std::vector<MECH_EYE_POINT> &vpntData)
{
	mmind::api::PointXYZMap pointXYZMap;
	auto status = m_pMechMindDevice->capturePointXYZMap(pointXYZMap);
	if (!status.isOK()) 
	{
		return status.errorCode;
	}
	MECH_EYE_POINT temp;
	for (size_t i = 0; i < pointXYZMap.height() * pointXYZMap.width(); i++)
	{
		temp.x = pointXYZMap[i].x;
		temp.y = pointXYZMap[i].y;
		temp.z = pointXYZMap[i].z;
		vpntData.push_back(temp);
	}
	return 0;
}

int CMechEyeCtrl::CapturePointXYZ(cv::Mat& mData)
{
	mmind::api::PointXYZMap pointXYZMap;
	auto status = m_pMechMindDevice->capturePointXYZMap(pointXYZMap);
	if (!status.isOK())
	{
		return status.errorCode;
	}

	for (size_t i = 0; i <  pointXYZMap.width(); i++)
	{
		pointXYZMap[i].x = pointXYZMap[i].x * 1000.0;
		pointXYZMap[i].y = pointXYZMap[i].y * 1000.0;
		pointXYZMap[i].z = pointXYZMap[i].z * 1000.0;
	}
	mData = cv::Mat(pointXYZMap.height(), pointXYZMap.width(), CV_32FC3, pointXYZMap.data());
	cv::imwrite("test.jpg", mData);
	return 0;
}

int CMechEyeCtrl::CaptureDepth(cv::Mat& mData)
{
	mmind::api::DepthMap depthMap;
	auto status = m_pMechMindDevice->captureDepthMap(depthMap);
	if (!status.isOK())
	{
		return status.errorCode;
	}
	mData = cv::Mat(depthMap.height(), depthMap.width(), CV_32FC1, depthMap.data());
	cv::imwrite("test.png", mData);
	return 0;
}

int CMechEyeCtrl::CaptureColor(cv::Mat& mData)
{
	mmind::api::ColorMap colorMap;
	auto status = m_pMechMindDevice->captureColorMap(colorMap);
	if (!status.isOK())
	{
		return status.errorCode;
	}
	mData = cv::Mat(colorMap.height(), colorMap.width(), CV_8UC3, colorMap.data());
	cv::imwrite("test.jpg", mData);
	return 0;
}

int CMechEyeCtrl::GetCameraInternalParam(DeviceIntri& param)
{
	auto status = m_pMechMindDevice->getDeviceIntri(param);
	if (!status.isOK())
	{
		return status.errorCode;
	}
	return 0;
}

int CMechEyeCtrl::GetCameraResolution(DeviceResolution& param)
{
	auto status = m_pMechMindDevice->getDeviceResolution(param);
	if (!status.isOK())
	{
		return status.errorCode;
	}
	return 0;
}

T_MECHEYE_CAMREA_DRIVER_PARAM CMechEyeCtrl::GetCameraBaseInfo()
{
	return m_tMecheyeCameraDriverPara;
}

MechEyeDevice* CMechEyeCtrl::GetMechEyeDevice()
{
	return m_pMechMindDevice;
}

#endif // ENABLE_MECH_EYE
