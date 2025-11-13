#include "StdAfx.h"
#include ".\Apps\PLib\KinectCam\KinectControl.h"


CKinectControl::CKinectControl(T_DEPTH_CAMREA_DRIVER_PARAM tDepthParam)
{
	m_tDepthParam = tDepthParam;
	m_Sensor = NULL;
	m_ColorImage = NULL;
	m_ColorImage = new cv::Mat(GetColorCameraHeight(), GetColorCameraWidth(), CV_8UC4);	//kinect获取到的是BGRA四通道图像
	m_DepthImage = NULL;
	m_DepthImage = new cv::Mat(GetDepthCameraHeight(), GetDepthCameraWidth(), CV_16UC1);
	m_InfraredImage = NULL;
	m_InfraredImage = new cv::Mat(GetInfraredCameraHeight(), GetInfraredCameraWidth(), CV_16UC1);

	m_pIColorSource = NULL;
	m_pIColorReader = NULL;
	m_pIColorFrame = NULL;
	m_pIDepthSource = NULL;
	m_pIDepthReader = NULL;
	m_pIDepthFrame = NULL;
	m_pIInfraredSource = NULL;
	m_pIInfraredReader = NULL;
	m_pIInfraredFrame = NULL;

	m_bContinuousImageAcquisition = false;
	m_bColorImg = false;
	m_bDepthImg = false;
	m_bInfraredImg = false;

	InitKinect();
}


CKinectControl::~CKinectControl()
{
	if (m_InfraredImage != NULL)
	{
		m_InfraredImage->release();
		m_InfraredImage = NULL;
	}
	if (m_DepthImage != NULL)
	{
		m_DepthImage->release();
		m_DepthImage = NULL;
	}
	if (m_ColorImage != NULL)
	{
		m_ColorImage->release();
		m_ColorImage = NULL;
	}
	CloseKinect();
}

bool CKinectControl::InitKinect()
{
	if (GetLocalDebugMark() || GetDeviceType() == 6)
	{
		return true;
	}
	if (GetDepthCameraType() == 3)
	{
		m_kDevice = NULL;

	Sleep(3000);//给深度相机充足的打开时间

		uint32_t device_count = k4a_device_get_installed_count();
		if (device_count <= 0)
		{
			XUI::MesBox::PopOkCancel(_T("获取深度传感器失败, {0}"), (int)device_count);
			return false;
		}
	}
	else if (GetDepthCameraType() == 2)
	{
		m_Hr = S_OK;
		m_Hr = GetDefaultKinectSensor(&m_Sensor);  //获取感应器
		if (m_Hr != S_OK)
		{
			XiMessageBox(_T("获取深度传感器失败！"));
			return false;
		}
	}
	return true;
}


bool CKinectControl::GetOneColorImage_k4a()
{
	k4a_capture_t sensor_capture;
	switch (k4a_device_get_capture(m_kDevice, &sensor_capture, 1500))
	{
	case K4A_WAIT_RESULT_SUCCEEDED:
		break;
	case K4A_WAIT_RESULT_TIMEOUT:
		//XiMessageBox(_T("深度相机采图超时！"));
		return false;
		break;
	case K4A_WAIT_RESULT_FAILED:
		//XiMessageBox(_T("深度相机采图失败！"));
		return false;
		break;
	}
	k4a_image_t image = k4a_capture_get_color_image(sensor_capture);
	k4a_capture_release(sensor_capture);
	if (image == NULL)
	{
		return false;
	}
	cv::Mat tTemp = cv::Mat(k4a_image_get_height_pixels(image), k4a_image_get_width_pixels(image), CV_8UC4, k4a_image_get_buffer(image));
	*m_ColorImage = tTemp.clone();
	k4a_image_release(image);
	tTemp.release();
//	k4a_image_release(image);
//	k4a_device_stop_cameras(m_kDevice);
	return true;
}

bool CKinectControl::GetOneDepthImage_k4a(BOOL bDistortion)
{
	k4a_capture_t sensor_capture;
	switch (k4a_device_get_capture(m_kDevice, &sensor_capture, 1500))
	{
	case K4A_WAIT_RESULT_SUCCEEDED:
		break;
	case K4A_WAIT_RESULT_TIMEOUT:
		//XiMessageBox(_T("深度相机采图超时！"));
		return false;
		break;
	case K4A_WAIT_RESULT_FAILED:
		//XiMessageBox(_T("深度相机采图失败！"));
		return false;
		break;
	}
	k4a_image_t image = k4a_capture_get_depth_image(sensor_capture);
	k4a_capture_release(sensor_capture);
	if (image == NULL)
	{
		return false;
	}
	cv::Mat tTemp = cv::Mat(k4a_image_get_height_pixels(image), k4a_image_get_width_pixels(image), CV_16UC1, k4a_image_get_buffer(image));

	if (bDistortion)
	{
		cv::Mat dust;
		Distortion(tTemp, dust);
		*m_DepthImage = dust.clone();
		dust.release();
	}
	else
	{
		*m_DepthImage = tTemp.clone();
	}
	k4a_image_release(image);
	tTemp.release();
/*	m_DepthImage = */
//	k4a_image_release(image);
//	k4a_device_stop_cameras(m_kDevice);
	return true;
}

bool CKinectControl::GetOneInfraredImage_k4a(BOOL bDistortion)
{
	k4a_capture_t sensor_capture;
	switch (k4a_device_get_capture(m_kDevice, &sensor_capture, 1500))
	{
	case K4A_WAIT_RESULT_SUCCEEDED:
		break;
	case K4A_WAIT_RESULT_TIMEOUT:
		//XiMessageBox(_T("深度相机采图超时！"));
		return false;
		break;
	case K4A_WAIT_RESULT_FAILED:
		//XiMessageBox(_T("深度相机采图失败！"));
		return false;
		break;
	}
	k4a_image_t image = k4a_capture_get_ir_image(sensor_capture);
	k4a_capture_release(sensor_capture);
	if (image == NULL)
	{
		return false;
	}
	cv::Mat tTemp = cv::Mat(k4a_image_get_height_pixels(image), k4a_image_get_width_pixels(image), CV_16UC1, k4a_image_get_buffer(image));
	if (bDistortion)
	{
		cv::Mat dust;
		Distortion(tTemp, dust);
		*m_InfraredImage = dust.clone();
	}
	else
	{
		*m_InfraredImage = tTemp.clone();
	}
	k4a_image_release(image);
	tTemp.release();
//	k4a_device_stop_cameras(m_kDevice);
	return true;
}

bool CKinectControl::GetOneColorImage_v2()
{
	bool bReturn = true;
	long long time = XI_clock();
	while (m_pIColorReader->AcquireLatestFrame(&m_pIColorFrame) != S_OK)
	{
		if (1500 < XI_clock() - time)
		{
			//XiMessageBox(_T("深度相机采图超时！"));
			bReturn = false;
			break;
		}
	}
	if (m_pIColorFrame != NULL)
	{
		cv::Mat	*Image;
		Image = new cv::Mat(GetColorCameraHeight(), GetColorCameraWidth(), CV_8UC4);
		m_pIColorFrame->CopyConvertedFrameDataToArray(GetColorCameraHeight() * GetColorCameraWidth() * 4, Image->data, ColorImageFormat_Bgra);
		cv::flip(*Image, *m_ColorImage, 1);
		Image->release();
	}
	return bReturn;
}

bool CKinectControl::GetOneDepthImage_v2()
{
	bool bReturn = true;
	long long time = XI_clock();
	while (m_pIDepthReader->AcquireLatestFrame(&m_pIDepthFrame) != S_OK)
	{
		if (1500 < XI_clock() - time)
		{
			//XiMessageBox(_T("深度相机采图超时！"));
			bReturn = false;
			break;
		}
	}
	if (m_pIDepthFrame != NULL)
	{
		cv::Mat	*Image;
		Image = new cv::Mat(GetDepthCameraHeight(), GetDepthCameraWidth(), CV_16UC1);
		m_pIDepthFrame->CopyFrameDataToArray(GetDepthCameraWidth() * GetDepthCameraHeight(), (UINT16 *)Image->data);
		cv::flip(*Image, *m_DepthImage, 1);
		Image->release();
	}
	return bReturn;
}

bool CKinectControl::GetOneInfraredImage_v2()
{
	bool bReturn = true;
	long long time = XI_clock();
	while (m_pIInfraredReader->AcquireLatestFrame(&m_pIInfraredFrame) != S_OK)
	{
		if (1500 < XI_clock() - time)
		{
			//XiMessageBox(_T("深度相机采图超时！"));
			bReturn = false;
			break;
		}
	}
	if (m_pIInfraredFrame != NULL)
	{
		cv::Mat	*Image;
		Image = new cv::Mat(GetInfraredCameraHeight(), GetInfraredCameraWidth(), CV_16UC1);
		m_pIInfraredFrame->CopyFrameDataToArray(GetInfraredCameraWidth() * GetInfraredCameraHeight(), (UINT16 *)Image->data);
		cv::flip(*Image, *m_InfraredImage, 1);
		Image->release();
	}
	return bReturn;
}


int CKinectControl::OpenKinect(int nWaitTime)
{
	if (GetLocalDebugMark())
	{
		return 0;
	}
	if (GetDepthCameraType() == 3)
	{
		k4a_result_t result = k4a_device_open(K4A_DEVICE_DEFAULT, &m_kDevice);
		if (K4A_RESULT_SUCCEEDED != result)
		{
			m_kDevice = NULL;
			XiMessageBox(_T("深度相机打开失败！"));
			return 1;
		}
		m_K4aConfigMode = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
		m_K4aConfigMode.camera_fps = K4A_FRAMES_PER_SECOND_5;
		m_K4aConfigMode.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
		m_K4aConfigMode.color_resolution = K4A_COLOR_RESOLUTION_OFF;
		m_K4aConfigMode.depth_mode = K4A_DEPTH_MODE_WFOV_UNBINNED;
		//	m_K4aConfigMode.synchronized_images_only = true;
		//  	if (!StartCameras(m_K4aConfigMode))
		//  	{
		//  		return;
		//  	}
	}
	else if (GetDepthCameraType() == 2)
	{
		m_Hr = S_OK;
		m_Hr = m_Sensor->Open();          //打开感应器
		WaitForAvailable(nWaitTime);
		if (m_Hr != S_OK)
		{
			XiMessageBox(_T("深度相机打开失败！"));
			return m_Hr;
		}
	}
	return 1;
}

bool CKinectControl::IsAvailable()
{
	if (GetDepthCameraType() == 3)
	{
		return true;
		// 	if (false == GetDepthImage())
		// 	{
		// 		return false;
		// 	}
	}
	else if (GetDepthCameraType() == 2)
	{
		BOOLEAN bAvailable;
		m_Hr = m_Sensor->get_IsAvailable(&bAvailable);//判断是否可用
		if (m_Hr != S_OK || bAvailable == FALSE)
		{
			return false;
		}
	}
	return true;
}

void CKinectControl::WaitForAvailable(int nWaitTime)
{
	if (nWaitTime <= 0)
	{
		return;
	}
	long long time = XI_clock();
	while (!IsAvailable())
	{
		Sleep(100);
		if (nWaitTime < XI_clock() - time)
		{
			XiMessageBox(_T("全景相机开启失败！请检查设备是否正常连接！"));
			return;
		}
	}
}

void CKinectControl::CloseKinect()
{
	if (GetLocalDebugMark() || GetDeviceType() == 6)
	{
		return;
	}
	if (m_bContinuousImageAcquisition)
	{
		CloseContinuousImageAcquisition();
	}
	if (GetDepthCameraType() == 3)
	{
		if (m_kDevice != NULL)
		{
			k4a_device_stop_cameras(m_kDevice);
			k4a_device_close(m_kDevice);
		}
	}
	else if (GetDepthCameraType() == 2)
	{
		if (m_Sensor != NULL)
		{
			m_Sensor->Close();
			m_Sensor->Release();
			m_Sensor = NULL;
		}
	}
}

bool CKinectControl::GetColorImage()
{
	bool bRtn = true;
	if (m_bContinuousImageAcquisition)
	{
		return false;
	}
	if (GetDepthCameraType() == 3)
	{
		if (!StartCameras(m_K4aConfigMode))
		{
			return false;
		}
		bRtn = GetOneColorImage_k4a();
		CloseCameras();
	}
	else if (GetDepthCameraType() == 2)
	{
		m_Hr = S_OK;
		//采图
		m_Hr = m_Sensor->get_ColorFrameSource(&m_pIColorSource);
		m_Hr = m_pIColorSource->OpenReader(&m_pIColorReader);
		bRtn = GetOneColorImage_v2();

		//释放资源
		if (m_pIColorFrame != NULL)
		{
			m_pIColorFrame->Release();
			m_pIColorFrame = NULL;
		}
		if (m_pIColorReader != NULL)
		{
			m_pIColorReader->Release();
			m_pIColorReader = NULL;
		}
		if (m_pIColorSource != NULL)
		{
			m_pIColorSource->Release();
			m_pIColorSource = NULL;
		}
	}
	return bRtn;
}

bool CKinectControl::GetDepthImage(BOOL bDistortion)
{
	bool bRtn = true;
	if (m_bContinuousImageAcquisition)
	{
		return false;
	}
	if (GetDepthCameraType() == 3)
	{
		if (!StartCameras(m_K4aConfigMode))
		{
			return false;
		}
		//Sleep(500);
		bRtn = GetOneDepthImage_k4a(bDistortion);
		CloseCameras();
	}
	else if (GetDepthCameraType() == 2)
	{
		m_Hr = S_OK;

		//采图
		m_Hr = m_Sensor->get_DepthFrameSource(&m_pIDepthSource);
		m_Hr = m_pIDepthSource->OpenReader(&m_pIDepthReader);
		bRtn = GetOneDepthImage_v2();

		//释放资源
		if (m_pIDepthFrame != NULL)
		{
			m_pIDepthFrame->Release();
			m_pIDepthFrame = NULL;
		}
		if (m_pIDepthReader != NULL)
		{
			m_pIDepthReader->Release();
			m_pIDepthReader = NULL;
		}
		if (m_pIDepthSource != NULL)
		{
			m_pIDepthSource->Release();
			m_pIDepthSource = NULL;
		}
	}
	return bRtn;
}

bool CKinectControl::GetInfraredImage(BOOL bDistortion)
{
	bool bRtn = true;
	if (m_bContinuousImageAcquisition)
	{
		return false;
	}
	if (GetDepthCameraType() == 3)
	{
		if (!StartCameras(m_K4aConfigMode))
		{
			return false;
		}
		bRtn = GetOneInfraredImage_k4a(bDistortion);
		CloseCameras();
	}
	else if (GetDepthCameraType() == 2)
	{
		m_Hr = S_OK;

		//采图
		m_Hr = m_Sensor->get_InfraredFrameSource(&m_pIInfraredSource);
		m_Hr = m_pIInfraredSource->OpenReader(&m_pIInfraredReader);
		bRtn = GetOneInfraredImage_v2();

		//释放资源
		if (m_pIInfraredFrame != NULL)
		{
			m_pIInfraredFrame->Release();
			m_pIInfraredFrame = NULL;
		}
		if (m_pIInfraredReader != NULL)
		{
			m_pIInfraredReader->Release();
			m_pIInfraredReader = NULL;
		}
		if (m_pIInfraredSource != NULL)
		{
			m_pIInfraredSource->Release();
			m_pIInfraredSource = NULL;
		}
	}
	return bRtn;
}

void CKinectControl::SetImgType(bool bColorImg /*= true*/, bool bDepthImg /*= true*/, bool bInfraredImg /*= true*/)
{
	if (m_bContinuousImageAcquisition)
	{
		return;
	}
	m_bColorImg = bColorImg;
	m_bDepthImg = bDepthImg;
	m_bInfraredImg = bInfraredImg;
}

void CKinectControl::OpenContinuousImageAcquisition()
{
	AfxBeginThread(ThreadContinuousImageAcquisition, this);
}

UINT CKinectControl::ThreadContinuousImageAcquisition(void *pParam)
{
	CKinectControl *pcMyObj = (CKinectControl *)pParam;
	pcMyObj->m_bContinuousImageAcquisition = true;
	pcMyObj->ContinuousImageAcquisition();
	pcMyObj->m_bContinuousImageAcquisition = false;
	return 0;
}

void CKinectControl::ContinuousImageAcquisition()
{
	//检测
	if ((!m_bColorImg)&&(!m_bDepthImg)&&(!m_bInfraredImg))
	{
		XiMessageBox("请先设置好连续采图的采图类型再使用连续采图！");
		return;
	}

	if (GetDepthCameraType() == 3)
	{
		if (!StartCameras(m_K4aConfigMode))
		{
			XiMessageBox("配置相机失败，请重试！");
			return;
		}
	}
	else if (GetDepthCameraType() == 2)
	{
		//初始化
		if (m_bColorImg)
		{
			m_Sensor->get_ColorFrameSource(&m_pIColorSource);
			m_pIColorSource->OpenReader(&m_pIColorReader);
		}
		if (m_bDepthImg)
		{
			m_Sensor->get_DepthFrameSource(&m_pIDepthSource);
			m_pIDepthSource->OpenReader(&m_pIDepthReader);
		}
		if (m_bInfraredImg)
		{
			m_Sensor->get_InfraredFrameSource(&m_pIInfraredSource);
			m_pIInfraredSource->OpenReader(&m_pIInfraredReader);
		}
	}
	//连续采图
	while (m_bColorImg || m_bDepthImg || m_bInfraredImg)
	{
		if (GetDepthCameraType() == 3)
		{
			if (!GetCapture())
			{
				continue;
			}

			//彩图
			if (m_bColorImg)
			{
				GetColorImage_k4a(m_ColorImage);
			}
			//深度图
			if (m_bDepthImg)
			{
				GetDepthImage_k4a(m_DepthImage);
			}
			//红外图
			if (m_bInfraredImg)
			{
				GetInfraredImage_k4a(m_InfraredImage);
			}
			k4a_capture_release(m_K4aCapture);
			Sleep(100);
		}
		else if (GetDepthCameraType() == 2)
		{
			//彩图
			if (m_bColorImg)
			{
				GetOneColorImage_v2();
			}
			//深度图
			if (m_bDepthImg)
			{
				GetOneDepthImage_v2();
			}
			//红外图
			if (m_bInfraredImg)
			{
				GetOneInfraredImage_v2();
			}

			//清理资源
			if (m_pIColorFrame != NULL)
			{
				m_pIColorFrame->Release();
				m_pIColorFrame = NULL;
			}
			if (m_pIDepthFrame != NULL)
			{
				m_pIDepthFrame->Release();
				m_pIDepthFrame = NULL;
			}
			if (m_pIInfraredFrame != NULL)
			{
				m_pIInfraredFrame->Release();
				m_pIInfraredFrame = NULL;
			}
		}
	}
	if (GetDepthCameraType() == 3)
	{
		k4a_device_stop_cameras(m_kDevice);
	}
	//清理资源
	if (m_pIColorFrame != NULL)
	{
		m_pIColorFrame->Release();
		m_pIColorFrame = NULL;
	}
	if (m_pIDepthFrame != NULL)
	{
		m_pIDepthFrame->Release();
		m_pIDepthFrame = NULL;
	}
	if (m_pIInfraredFrame != NULL)
	{
		m_pIInfraredFrame->Release();
		m_pIInfraredFrame = NULL;
	}
	if (m_pIColorReader != NULL)
	{
		m_pIColorReader->Release();
		m_pIColorReader = NULL;
	}
	if (m_pIColorSource != NULL)
	{
		m_pIColorSource->Release();
		m_pIColorSource = NULL;
	}	
	if (m_pIDepthReader != NULL)
	{
		m_pIDepthReader->Release();
		m_pIDepthReader = NULL;
	}
	if (m_pIDepthSource != NULL)
	{
		m_pIDepthSource->Release();
		m_pIDepthSource = NULL;
	}
	if (m_pIInfraredReader != NULL)
	{
		m_pIInfraredReader->Release();
		m_pIInfraredReader = NULL;
	}
	if (m_pIInfraredSource != NULL)
	{
		m_pIInfraredSource->Release();
		m_pIInfraredSource = NULL;
	}
}

void CKinectControl::CloseContinuousImageAcquisition()
{
	m_bColorImg = false;
	m_bDepthImg = false;
	m_bInfraredImg = false;
	while (m_bContinuousImageAcquisition)
	{
		Sleep(20);
		DoEvent();
	}
}


bool CKinectControl::GetKinectSerialNumber(CString sSerialNumber)
{
	char *serial_number = NULL;
	size_t serial_number_length = 0;
	if (K4A_BUFFER_RESULT_TOO_SMALL != k4a_device_get_serialnum(m_kDevice, NULL, &serial_number_length))
	{
		k4a_device_close(m_kDevice);
		m_kDevice = NULL;
		return false;
	}
	serial_number = (char *)malloc(serial_number_length);
	if (serial_number == NULL)
	{
		k4a_device_close(m_kDevice);
		m_kDevice = NULL;
		return false;
	}
	if (K4A_BUFFER_RESULT_SUCCEEDED != k4a_device_get_serialnum(m_kDevice, serial_number, &serial_number_length))
	{
		free(serial_number);
		serial_number = NULL;
		k4a_device_close(m_kDevice);
		m_kDevice = NULL;
		return false;
	}
	sSerialNumber.Format("%s", serial_number);
	return true;
}

bool CKinectControl::StartCameras(k4a_device_configuration_t K4aConfigMode)
{
	if (K4A_RESULT_SUCCEEDED != k4a_device_start_cameras(m_kDevice, &K4aConfigMode))
	{
		if (m_nReNum < 3)
		{
			m_nReNum++;
			WriteLog("重新深度采图：%d", m_nReNum);
			CloseKinect();
			Sleep(500);
			OpenKinect();
			if (true == StartCameras(K4aConfigMode))
			{
				return true;
			}
		}
		else
		{
			m_nReNum = 0;
			WriteLog("已达到最大重试次数！！");
			return false;
		}

		return false;
	}
	m_nReNum = 0;
	return true;
}

bool CKinectControl::CloseCameras()
{
	k4a_device_stop_cameras(m_kDevice);
	return true;
}

bool CKinectControl::GetCapture(int nWaitTime)
{
	switch (k4a_device_get_capture(m_kDevice, &m_K4aCapture, nWaitTime))
	{
	case K4A_WAIT_RESULT_SUCCEEDED:
		break;
	case K4A_WAIT_RESULT_TIMEOUT:
		XiMessageBox(_T("深度相机采图超时！"));
		return false;
		break;
	case K4A_WAIT_RESULT_FAILED:
		XiMessageBox(_T("深度相机采图失败！"));
		return false;
		break;
	}
	return true;
}

bool CKinectControl::GetColorImage_k4a(cv::Mat	*ColorImage)
{
	k4a_image_t image = k4a_capture_get_color_image(m_K4aCapture);
	if (image == NULL)
	{
		return false;
	}
	cv::Mat tTemp = cv::Mat(k4a_image_get_height_pixels(image), k4a_image_get_width_pixels(image), CV_8UC4, k4a_image_get_buffer(image));
	*ColorImage = tTemp.clone();
	k4a_image_release(image);
	tTemp.release();
	return true;
}

bool CKinectControl::GetDepthImage_k4a(cv::Mat	*DepthImage)
{
	k4a_image_t image = k4a_capture_get_depth_image(m_K4aCapture);
	if (image == NULL)
	{
		return false;
	}
	cv::Mat tTemp = cv::Mat(k4a_image_get_height_pixels(image), k4a_image_get_width_pixels(image), CV_16UC1, k4a_image_get_buffer(image));
	cv::Mat dust;
	Distortion(tTemp, dust);

//	flip(dust, dust, 0);
	*DepthImage = dust.clone();
	k4a_image_release(image);
	tTemp.release();
	dust.release();
	return true;
}

bool CKinectControl::GetInfraredImage_k4a(cv::Mat	*InfraredImage)
{
	k4a_image_t image = k4a_capture_get_ir_image(m_K4aCapture);
	if (image == NULL)
	{
		return false;
	}
	cv::Mat tTemp = cv::Mat(k4a_image_get_height_pixels(image), k4a_image_get_width_pixels(image), CV_16UC1, k4a_image_get_buffer(image));
	cv::Mat dust;
	Distortion(tTemp, dust);
	*InfraredImage = dust.clone();
	k4a_image_release(image);
	tTemp.release();
	dust.release();
	return true;
}

bool CKinectControl::GetDepthPointCloud(std::vector<cv::Point3d> &vcPointCloud)
{
	k4a_calibration_t calibration;
	k4a_result_t result = k4a_device_get_calibration(m_kDevice, m_K4aConfigMode.depth_mode, m_K4aConfigMode.color_resolution, &calibration);
	if (K4A_RESULT_SUCCEEDED != result)
	{
		return false;
	}
	k4a_transformation_t transformation = k4a_transformation_create(&calibration);

	if (!StartCameras(m_K4aConfigMode))
	{
		return false;
	}

	k4a_capture_t sensor_capture;
	switch (k4a_device_get_capture(m_kDevice, &sensor_capture, 1500))
	{
	case K4A_WAIT_RESULT_SUCCEEDED:
		break;
	case K4A_WAIT_RESULT_TIMEOUT:
		XiMessageBox(_T("深度相机采图超时！"));
		return false;
		break;
	case K4A_WAIT_RESULT_FAILED:
		XiMessageBox(_T("深度相机采图失败！"));
		return false;
		break;
	}
	k4a_image_t image = k4a_capture_get_depth_image(sensor_capture);
	k4a_capture_release(sensor_capture);
	if (image == NULL)
	{
		return false;
	}
	k4a_image_t xyz_image = nullptr;
	result = k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
		k4a_image_get_width_pixels(image), k4a_image_get_height_pixels(image),
		k4a_image_get_width_pixels(image) * 3 * static_cast<int32_t>(sizeof(int16_t)), &xyz_image);
	if (K4A_RESULT_SUCCEEDED != result)
	{
		return false;
	}
	result = k4a_transformation_depth_image_to_point_cloud(transformation, image, K4A_CALIBRATION_TYPE_DEPTH, xyz_image);
	if (K4A_RESULT_SUCCEEDED != result)
	{
		return false;
	}
	cv::Mat cv_xyzImage = cv::Mat(k4a_image_get_height_pixels(xyz_image), k4a_image_get_width_pixels(xyz_image), CV_16SC3, 
		(void *)k4a_image_get_buffer(xyz_image), static_cast<size_t>(k4a_image_get_stride_bytes(xyz_image)));
	cv::Mat cv_xyzImage_32F;
	cv_xyzImage.convertTo(cv_xyzImage_32F, CV_32FC3, 1.0 / 1000, 0);
	
	convertMatToPcl(cv_xyzImage_32F, vcPointCloud);
	k4a_image_release(image);
	CloseCameras();	
	return true;
}

void CKinectControl::Distortion(cv::Mat mSrc, cv::Mat &mDst)
{
	if (!GetDepthCameraEnableUndistort())
	{
		mSrc.copyTo(mDst);
		return;
	}
	cv::Mat new_matrix;
	cv::undistort(mSrc, mDst, GetDepthCameraCameraMatrix(), GetDepthCameraDistCoeffs(), new_matrix);
}
// void CKinectControl::Distortion(cv::Mat mSrc, cv::Mat &mDst)
// {
// 	cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
// 	cameraMatrix.at<double>(0, 0) = 464.639;
// 	cameraMatrix.at<double>(0, 1) = 0;
// 	cameraMatrix.at<double>(0, 2) = 521.159;
// 	cameraMatrix.at<double>(1, 0) = 0;
// 	cameraMatrix.at<double>(1, 1) = 460.272;
// 	cameraMatrix.at<double>(1, 2) = 515.781;
// 	cameraMatrix.at<double>(2, 0) = 0;
// 	cameraMatrix.at<double>(2, 1) = 0;
// 	cameraMatrix.at<double>(2, 2) = 1;
// 	cv::Mat distCoeffs = cv::Mat::zeros(5, 1, CV_64F);
// 	distCoeffs.at<double>(0, 0) = -0.259179;
// 	distCoeffs.at<double>(1, 0) = 0.062160;
// 	distCoeffs.at<double>(2, 0) = 0.002656;
// 	distCoeffs.at<double>(3, 0) = 0.001331;
// 	distCoeffs.at<double>(4, 0) = 0;
// 
// 	cv::Mat new_matrix;
// 	cv::undistort(mSrc, mDst, cameraMatrix, distCoeffs, new_matrix);
// }

bool CKinectControl::convertMatToPcl(const cv::Mat& xyzDepth, std::vector<cv::Point3d> &vcPointCloud)
{
	// cv::Mat矩阵的数据类型必须是 CV_32FC3
	if (21 != xyzDepth.type())
	{
		std::cerr << "Error in convertMatToPcl()." << std::endl;
		std::cerr << "The type of input data(cv::Mat) must be CV_32FC3." << std::endl;
		return false;
	}

	auto depthWidth = xyzDepth.cols;
	auto depthHeight = xyzDepth.rows;

	std::ofstream outCloudPt(".//cloudpt.txt");
	for (int iRows = 0; iRows < depthHeight; iRows++)
	{
		for (int iCols = 0; iCols < depthWidth; iCols++)
		{
			cv::Point3d pt;
			pt.x = xyzDepth.at<cv::Vec3f>(iRows, iCols)[0];
			pt.y = xyzDepth.at<cv::Vec3f>(iRows, iCols)[1];
			pt.z = xyzDepth.at<cv::Vec3f>(iRows, iCols)[2];
			vcPointCloud.push_back(pt);
			outCloudPt << pt.x << " " << pt.y << " " << pt.z << std::endl;
		}
	}
	return true;
}

int CKinectControl::GetDepthCameraType()
{
	return m_tDepthParam.nCameraType;
}

int CKinectControl::GetDepthCameraWidth()
{
	return m_tDepthParam.nDepthWidth;
}

int CKinectControl::GetDepthCameraHeight()
{
	return m_tDepthParam.nDepthHeight;
}

int CKinectControl::GetColorCameraWidth()
{
	return m_tDepthParam.nColorWidth;
}

int CKinectControl::GetColorCameraHeight()
{
	return m_tDepthParam.nColorHeight;
}

int CKinectControl::GetInfraredCameraWidth()
{
	return m_tDepthParam.nInfraredWidth;
}

int CKinectControl::GetInfraredCameraHeight()
{
	return m_tDepthParam.nInfraredHeight;
}

bool CKinectControl::GetDepthCameraEnableUndistort()
{
	return m_tDepthParam.bEnableUndistort;
}

cv::Mat CKinectControl::GetDepthCameraCameraMatrix()
{
	return m_tDepthParam.cvCameraMatrix;
}

cv::Mat CKinectControl::GetDepthCameraDistCoeffs()
{
	return m_tDepthParam.cvDistCoeffs;
}